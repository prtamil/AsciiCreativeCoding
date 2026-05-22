/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diff_drive_robot.c — two-wheeled robot with no steering wheel
 *
 * DEMO: A robot that has TWO INDEPENDENT WHEELS on a single fixed axle
 *       — and that's the whole steering mechanism. There is no
 *       steering wheel. There is no joystick that controls direction.
 *       To turn left, the LEFT wheel goes slower than the RIGHT one.
 *       To turn right, the right wheel goes slower than the left one.
 *       To spin in place, the wheels go in OPPOSITE directions. To
 *       drive straight, the wheels run at the same speed.
 *
 *       Real-world examples:
 *           ─ iRobot Roomba (vacuum)
 *           ─ tank-style RC toys
 *           ─ electric wheelchairs (joystick controls each wheel
 *                                   independently)
 *           ─ many small lab/research robots
 *
 *       This file is a minimal teaching simulator: you press keys,
 *       the keys translate into "left wheel speed" and "right wheel
 *       speed", and the robot moves wherever those two numbers
 *       cause it to go. The screen shows the robot from above, the
 *       wheels marked L and R, a yellow heading arrow, the wheel
 *       velocity arrows (green = forward, red = reverse), and a
 *       trail of recent positions.
 *
 * Study alongside:
 *   robots/walking_robot.c           — bipedal contrast: legs
 *                                      instead of wheels.
 *   animation/hexpod_tripod.c        — 6-leg insect locomotion.
 *   physics/stroke_engine.c          — slider-crank kinematics
 *                                      (similar idea: gears/wheels
 *                                       drive position).
 *   matrix_rain/matrix_rain.c        — same project conventions
 *                                      (variable-dt loop, HUD spec,
 *                                       theme palettes).
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  trail        — ring buffer of recent positions
 *   §6  robot        — Pose + WheelSpeeds + Commands sub-structs;
 *                       Robot root (pose + wheels + cmd + trail + axle);
 *                       compute_wheels (inverse kin) + step_pose
 *                       (forward kin) + robot_tick orchestrator
 *   §7  scene        — World + InputState + SimControls sub-structs;
 *                       Scene root (world + robot + input + sim);
 *                       scene_init / _resize / _tick + scene_apply_keys
 *                       split into apply_full_stop +
 *                       apply_throttle_ramp_or_decay +
 *                       apply_turn_ramp_or_decay +
 *                       apply_spin_in_place_override +
 *                       clamp_commands_to_physical_limits
 *   §8  render       — draw the robot, wheels, arrows, trail, HUD;
 *                       WheelPositions + compute_wheel_positions;
 *                       render_robot split into paint_glyph_at_cell +
 *                       paint_heading_arrow + paint_wheel_label;
 *                       render_hud split into body_linear_velocity +
 *                       body_angular_velocity +
 *                       instantaneous_turn_radius_px +
 *                       radians_to_degrees + format_hud_status +
 *                       draw_hud_status + draw_hud_hint
 *   §9  screen       — ncurses init / present / HUD spec
 *   §10 app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            full stop (zero both v_cmd and w_cmd)
 *   p                pause / resume physics
 *   r                reset to centre
 *   W / ↑            throttle forward (ramps v_cmd up)
 *   S / ↓            throttle backward (ramps v_cmd down/negative)
 *   A / ←            turn left
 *   D / →            turn right
 *   Z                spin in place LEFT (instant max ω, v=0)
 *   E                spin in place RIGHT (instant max ω, v=0)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/diff_drive_robot.c \
 *       -o diff_drive_robot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two-step kinematics, run once per frame.
 *
 *                 (1) FORWARD KINEMATICS — given the two wheel speeds
 *                     vL (left) and vR (right), compute how the
 *                     robot's body position (x, y) and heading θ
 *                     change in the next dt seconds.
 *
 *                         v = (vL + vR) / 2          ← body speed
 *                         ω = (vR − vL) / axle       ← spin rate
 *                         x += v · cos(θ) · dt
 *                         y += v · sin(θ) · dt
 *                         θ += ω · dt
 *
 *                 (2) INVERSE KINEMATICS — the user wants to go
 *                     forward at speed v_cmd and turn at rate w_cmd.
 *                     Compute what wheel speeds achieve that.
 *
 *                         vL = v_cmd − w_cmd · axle / 2
 *                         vR = v_cmd + w_cmd · axle / 2
 *
 *                 The user's keys nudge `v_cmd` and `w_cmd`; (2)
 *                 turns those into `vL`, `vR`; (1) turns those into
 *                 a new pose. That's the entire physics loop.
 *
 * Data-structure: Scene = World + Robot + InputState + SimControls,
 *                 each a named sub-struct (see §7 for the layered-
 *                 ownership diagram).  Robot itself decomposes into
 *                 Pose (px, py, θ) + WheelSpeeds (vL, vR) + Commands
 *                 (v_cmd, w_cmd) + a ring-buffer Trail + the constant
 *                 axle (§6).  The three Robot sub-structs mirror the
 *                 kinematic pipeline:
 *
 *                   user keys → cmd → wheels → pose
 *                              (inverse kin)  (forward kin)
 *
 *                 No heap allocation post-init.
 *
 * Rendering     : Painter's order with the body always on top.
 *                 Trail dots first (oldest → newest), then wheel
 *                 velocity arrows (green/red), then the heading
 *                 arrow (yellow), then 'L' and 'R' wheel labels,
 *                 then the '@' body. HUD on row 0, hint strip on
 *                 the bottom row.
 *
 * Performance   : O(1) per frame. Six trig calls (2 cos, 2 sin,
 *                 plus one cos and one sin per arrow drawn).
 *                 Microseconds. ncurses redraw is the dominant
 *                 cost.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Differential-drive kinematics (the math in §6) ─────────────
 *   [1] Wikipedia, "Differential wheeled robot" — formulas, geometry,
 *       real-world examples (Roomba, tank-style RC, etc.).  Concise
 *       summary of the same (v, ω) ↔ (vL, vR) maps this file
 *       implements.
 *       https://en.wikipedia.org/wiki/Differential_wheeled_robot
 *   [2] Siegwart, R., Nourbakhsh, I. & Scaramuzza, D. (2011),
 *       "Introduction to Autonomous Mobile Robots" (2nd ed.), MIT
 *       Press, Ch. 3 — the textbook reference for differential-
 *       drive kinematics.  §3.2.2 (pose representation), §3.2.4
 *       (inverse + forward kinematics) map directly onto this
 *       file's Pose, Commands, WheelSpeeds sub-structs (§6).
 *   [3] LaValle, S. M. (2006), "Planning Algorithms", Cambridge
 *       University Press, §13.1.2 — the UNICYCLE MODEL.  Our (v, ω)
 *       command pair IS the unicycle's state; the (vL, vR) wheel-
 *       speed primitives are one realisation.  Free online at
 *       https://lavalle.pl/planning/
 *
 *   ── Why a diff-drive robot can't slide sideways ────────────────
 *   [4] Wikipedia, "Nonholonomic system" — explains why the robot
 *       has 3-DOF in CONFIGURATION (x, y, θ) but only 2-DOF in
 *       VELOCITY (forward + turn).  A car parallel-parking is the
 *       same constraint.  Without this constraint, the robot
 *       could just translate sideways — and the simulation would
 *       lose its character.
 *       https://en.wikipedia.org/wiki/Nonholonomic_system
 *   [5] Dudek, G. & Jenkin, M. (2010), "Computational Principles of
 *       Mobile Robotics" (2nd ed.), Cambridge University Press,
 *       §4.2 — wheel-speed primitives + nonholonomic constraints
 *       in the same chapter, from a computational angle.
 *
 *   ── Euler integration (the per-tick step_pose math) ────────────
 *   [6] Hairer, E., Lubich, C. & Wanner, G. (2006), "Geometric
 *       Numerical Integration" (2nd ed.), Springer — explicit-Euler
 *       reference.  §6 step_pose uses plain forward Euler:
 *           x += v · cos(θ) · dt
 *           y += v · sin(θ) · dt
 *           θ += ω · dt
 *       This is non-symplectic but the system has no stiff terms
 *       (no inertia, ω is prescribed directly), so drift is
 *       imperceptible at the dt = 1/60 s frame budget.
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [7] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics.  This file
 *       deliberately uses VARIABLE dt: the diff-drive ODE is
 *       non-stiff and unconditionally stable under forward Euler.
 *       Fiedler's DT_CAP rule (100 ms hard cap, DT_CAP_SEC here)
 *       is the only piece we adopt.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [8] Raymond, E. S., "NCURSES Programming HOWTO" — §6 (colour
 *       pairs) for per-element semantic colouring (body / wheels /
 *       arrows / trail), §11 (output options) for the wnoutrefresh
 *       + doupdate diff-only write pattern.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     robots/walking_robot.c       — the LEGGED contrast: pose comes
 *       from foot positions instead of wheel speeds.  Read this
 *       second to see how the kinematic pipeline changes when the
 *       contact model changes.
 *     animation/hexpod_tripod.c    — 6-leg insect locomotion (gait
 *       coordination on top of leg kinematics).
 *     physics/stroke_engine.c      — slider-crank kinematics
 *       (similar idea: gears/wheels drive position via a
 *       parametric curve).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A differential drive robot has two independent driven wheels on a
 * shared axle. Each wheel can be set to any speed (positive, zero,
 * or negative) independently of the other. Steering is purely an
 * emergent property of running them at different speeds.
 *
 * If you understand four motions, you understand the entire concept:
 *
 * THE FOUR MOTIONS
 * ────────────────
 *
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │                                                              │
 *  │    DRIVE STRAIGHT       TURN RIGHT       SPIN IN PLACE       │
 *  │    ────────────────     ───────────      ────────────────    │
 *  │                                                              │
 *  │    vL  =  vR            vL = 2           vL = -5             │
 *  │           ↑   ↑         vR = 8           vR = +5             │
 *  │           │   │             ⤴                                │
 *  │     ┌────L─R────┐       ┌───L─R───┐       ┌────L─R────┐      │
 *  │     │           │       │      ↘  │       │     ↻     │      │
 *  │     │  (body)   │  →    │ (curves)│       │ (rotates  │      │
 *  │     │           │       │   right │       │  in place)│      │
 *  │     └───────────┘       └─────────┘       └───────────┘      │
 *  │                                                              │
 *  │    PIVOT — slow wheel = pivot, fast wheel sweeps the arc:    │
 *  │                                                              │
 *  │    vL = 0                                                    │
 *  │    vR = 5                                                    │
 *  │     ┌────L─R────┐    ← L stuck, R goes forward               │
 *  │     │           │      → robot pivots around L               │
 *  │     │  ↻        │      → ICC (centre of arc) = position of L │
 *  │     │           │                                            │
 *  │     └───────────┘                                            │
 *  │                                                              │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *
 * WHY "DIFFERENTIAL"?
 * ───────────────────
 * Because the steering effect comes from the DIFFERENCE between the
 * two wheel speeds. Not from a steering rod, not from a joint —
 * just (vR − vL).
 *
 *   vR − vL  >  0   →   turn right
 *   vR − vL  <  0   →   turn left
 *   vR − vL  =  0   →   straight (no matter what vR + vL is)
 *
 * The MAGNITUDE of the difference, divided by the axle width, is
 * the angular velocity ω in radians per second:
 *
 *      ω = (vR − vL) / axle
 *
 * The SUM divided by 2 is the body's straight-ahead speed v:
 *
 *      v = (vL + vR) / 2
 *
 * That's the whole forward kinematics. Two scalars in (vL, vR), two
 * scalars out (v, ω).
 *
 * ICC — "INSTANTANEOUS CENTRE OF CURVATURE"
 * ─────────────────────────────────────────
 * When ω ≠ 0, the robot follows a circular arc. The CENTRE of that
 * circle (called the ICC) lies on the axle's extension, on the side
 * of the SLOWER wheel:
 *
 *           ICC × ─ ─ ─ ─ ─ ─ ─ ─ ┐
 *                │                │  R = turn radius (signed)
 *                │                ↓
 *                ┌────L─R────┐
 *                │           │
 *                │  (body)   │
 *                └───────────┘
 *
 *           R = (axle / 2) · (vR + vL) / (vR − vL)
 *             = v / ω
 *
 *  Slow wheel near ICC, fast wheel sweeps wider arc, body's centre
 *  travels at v on a circle of radius |R|. When vL = vR, R = ∞
 *  (straight line). When vL = −vR, R = 0 (spin in place).
 *
 * NONHOLONOMIC CONSTRAINT
 * ───────────────────────
 * The robot can't slide sideways. The wheels resist lateral motion
 * (they roll, they don't slip). In math:
 *
 *      ẋ · sin θ  −  ẏ · cos θ  =  0           (lateral velocity = 0)
 *
 * This constraint is automatically satisfied by `ẋ = v · cos θ` and
 * `ẏ = v · sin θ` — the velocity vector is always along the heading.
 * No projection step or penalty needed.
 *
 * Compare with an OMNIDIRECTIONAL robot (mecanum wheels, omni wheels):
 * it CAN slide sideways and its kinematics have no such restriction.
 * That's the holonomic case. Diff drive is nonholonomic.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. dt = wall-clock seconds since last frame, capped.
 *  2. Read keys, set the boolean flags `fwd`, `rev`, `left`, `right`,
 *     `spin_l`, `spin_r`, `stop`.
 *  3. Translate keys into commands:
 *      - W / S nudges v_cmd up/down at V_RATE per second.
 *      - A / D nudges w_cmd left/right at W_RATE per second.
 *      - When no turn key is held, w_cmd decays exponentially toward 0.
 *      - Z / E sets v_cmd = 0 and w_cmd = ±W_MAX (instant in-place spin).
 *      - Space sets both commands to 0.
 *      - Clamp v_cmd to [-V_MAX, V_MAX], w_cmd to [-W_MAX, W_MAX].
 *  4. INVERSE KINEMATICS — turn (v_cmd, w_cmd) into (vL, vR):
 *           vL = v_cmd − w_cmd · axle / 2
 *           vR = v_cmd + w_cmd · axle / 2
 *  5. FORWARD KINEMATICS + Euler integration — turn (vL, vR) into a
 *     new pose:
 *           v = (vL + vR) / 2
 *           ω = (vR − vL) / axle
 *           x += v · cos(θ) · dt
 *           y += v · sin(θ) · dt
 *           θ += ω · dt
 *           wrap θ into (−π, π]
 *  6. WRAP — if the body left a screen edge, teleport it to the
 *     opposite edge. Robot stays visible forever.
 *  7. PUSH every TRAIL_SAMPLE_STEP frames, push (x, y) into the trail
 *     ring buffer.
 *  8. RENDER.
 *
 * KEY FORMULAS
 * ────────────
 *   Inverse kinematics (commands → wheels):
 *           vL = v_cmd − w_cmd · L/2
 *           vR = v_cmd + w_cmd · L/2
 *
 *   Forward kinematics (wheels → body velocity):
 *           v  = (vL + vR) / 2
 *           ω  = (vR − vL) / L
 *
 *   Pose update (Euler):
 *           x  ← x + v · cos(θ) · dt
 *           y  ← y + v · sin(θ) · dt
 *           θ  ← θ + ω · dt
 *
 *   Turn radius:
 *           R = v / ω         (R > 0 right, < 0 left, ±∞ straight)
 *
 *   Heading wrap:
 *           θ ∈ (−π, π]       (so atan2 / lerp behave well)
 *
 *   Coordinate convention (terminal screens are y-down):
 *           θ = 0      → faces RIGHT  (+x)
 *           θ = π/2    → faces DOWN   (+y)
 *           ω > 0      → CLOCKWISE on screen (because y is down)
 *
 *
 * Background you need
 * ───────────────────
 *   - Vec2 / scalar arithmetic.
 *   - Trig: (cos θ, sin θ) is the unit vector at angle θ.
 *   - Variable-timestep main loop (CLAUDE.md §Architecture).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lagrangian mechanics. Diff-drive kinematics are PURE
 *     KINEMATICS (geometry of motion); no forces / torques /
 *     mass involved.
 *   - Wheel slip, friction modelling. We assume PURE ROLLING
 *     (lateral velocity always zero — T4 below).
 *   - Path planning (A*, RRT). The user drives directly with
 *     keys; no planner.
 *   - Sensor models, SLAM. No sensors here.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum { TARGET_FPS = 60 };

/* ── §1.2 cell pixel dimensions (the aspect-ratio bridge) ─────────── */
/*
 * All physics happens in PIXEL SPACE — a uniform grid where 1 px
 * is 1/8 of a cell wide and 1/16 of a cell tall. This decouples
 * the simulation from terminal size and corrects for the 2:1
 * cell aspect ratio that would otherwise stretch circles into
 * vertical ellipses.
 */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 robot geometry (pixels) ─────────────────────────────────── */
/*
 * AXLE_PX — distance between the two wheels, in pixels.
 *   With CELL_W = 8 px, 36 px ≈ 4.5 cells — a small robot on a
 *   typical terminal. Wider axle → slower turns at the same wheel
 *   speed difference (because ω = (vR-vL)/axle).
 *
 * ARROW_PX — length of the yellow heading arrow at full extension.
 *
 * VEL_ARROW_PX — length of the green/red wheel velocity arrow when
 *   the wheel is at ±V_MAX. At lower speeds the arrow shrinks
 *   linearly with |v_wheel|.
 */
#define AXLE_PX        36.0f
#define ARROW_PX       34.0f
#define VEL_ARROW_PX   28.0f

/* ── §1.4 dynamics (physical units) ───────────────────────────────── */
/*
 * V_MAX, W_MAX — physical limits.
 *   V_MAX = 180 px/sec → robot crosses a 100-col terminal in ~4 sec.
 *   W_MAX = 3 rad/sec  → full rotation in 2π/3 ≈ 2.1 sec.
 *
 * V_RATE, W_RATE — how fast the COMMAND ramps up while the throttle
 *   key is held.
 *   V_RATE = 12 · V_MAX → reaches V_MAX in 1/12 ≈ 0.083 sec (snappy).
 *   W_RATE = 15 · W_MAX → reaches W_MAX in 1/15 ≈ 0.067 sec.
 *
 * V_DECAY_PER_SEC — multiplier on v_cmd per second when no throttle
 *   key is held.
 *   1.0 = no friction (robot keeps coasting indefinitely until braked).
 *
 * W_DECAY_PER_SEC — multiplier on w_cmd per second when no turn key
 *   is held.
 *   0.001 = strong damping → ω drops to 0.1 % of its value over 1 sec
 *   so straight-driving doesn't acquire residual rotation from a
 *   brief turn input.
 */
#define V_MAX            180.0f
#define W_MAX              3.0f
#define V_RATE          (V_MAX * 12.0f)
#define W_RATE          (W_MAX * 15.0f)
#define V_DECAY_PER_SEC    1.000f
#define W_DECAY_PER_SEC    0.001f

/* ── §1.5 trail ring buffer ──────────────────────────────────────── */
enum {
    TRAIL_CAP             = 600,
    /* Push a sample every Nth frame so the trail stretches further
     * for the same memory budget. 2 = sample at 30 Hz on a 60-fps loop. */
    TRAIL_SAMPLE_STEP     = 2,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.7 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.8 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..6 — robot semantic colours */
    CP_BODY      = 1,           /* '@'                white          */
    CP_HEAD,                    /* heading arrow      yellow         */
    CP_WHL_L,                   /* 'L' wheel label    green          */
    CP_WHL_R,                   /* 'R' wheel label    magenta        */
    CP_VEL_FWD,                 /* wheel arrow forward  lime         */
    CP_VEL_REV,                 /* wheel arrow reverse  red          */

    /* 7..8 — trail */
    CP_TRAIL_NEW,               /* fresh trail dots   cyan           */
    CP_TRAIL_OLD,               /* aged trail dots    dim blue       */

    /* 9..10 — HUD spec, theme-independent */
    PAIR_HUD,                   /* row 0 status       yellow A_BOLD  */
    PAIR_HINT,                  /* bottom row hint    cyan   A_BOLD  */
};

/* ── §1.9 HUD layout ──────────────────────────────────────────────── */
#define HUD_BUF_LEN  120

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Each visual element gets one fixed semantic colour. The palette is
 * not theme-cycled — this is a teaching simulator, not a screensaver.
 *
 *   body         '@'       white
 *   heading      arrow     yellow
 *   left wheel   'L'       green       (so 'L' for green/Left is mnemonic)
 *   right wheel  'R'       magenta
 *   forward vel  arrow     lime green
 *   reverse vel  arrow     red
 *   fresh trail  '.'       cyan        (recent path)
 *   old trail    ':'       dim blue    (older path, fades visually)
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) are bound separately on the
 * default terminal background (-1) per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,       255, COLOR_BLACK);    /* near-white     */
        init_pair(CP_HEAD,       226, COLOR_BLACK);    /* yellow         */
        init_pair(CP_WHL_L,       46, COLOR_BLACK);    /* green          */
        init_pair(CP_WHL_R,      201, COLOR_BLACK);    /* magenta        */
        init_pair(CP_VEL_FWD,     82, COLOR_BLACK);    /* lime           */
        init_pair(CP_VEL_REV,    196, COLOR_BLACK);    /* red            */
        init_pair(CP_TRAIL_NEW,   51, COLOR_BLACK);    /* cyan           */
        init_pair(CP_TRAIL_OLD,   25, COLOR_BLACK);    /* dim blue       */
        init_pair(PAIR_HUD,      226, -1);             /* yellow on bg   */
        init_pair(PAIR_HINT,      51, -1);             /* cyan on bg     */
    } else {
        init_pair(CP_BODY,     COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HEAD,     COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_WHL_L,    COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_WHL_R,    COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_VEL_FWD,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_VEL_REV,  COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TRAIL_NEW,COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD,COLOR_BLUE,    COLOR_BLACK);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Physics lives in PIXEL SPACE. A pixel is 1/8 of a cell wide and
 * 1/16 of a cell tall, so circles drawn in pixel space look round
 * on screen. Conversion to cell coordinates happens ONLY at draw
 * time (px_to_cx, px_to_cy), and the world dimensions in pixels
 * (pw, ph) are derived from the terminal size in cells.
 */
static inline int   pw       (int cols)  { return cols * CELL_W; }
static inline int   ph       (int rows)  { return rows * CELL_H; }
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================== */
/* §5  trail — ring buffer of recent positions                           */
/* ===================================================================== */

/*
 * Trail — circular ring buffer of the robot's recent positions.
 *
 * Intent
 *   Renders as the fading dot trail behind the robot.  Stores the
 *   last TRAIL_CAP positions, sampled every TRAIL_SAMPLE_STEP
 *   frames so the VISIBLE trail length stretches further for the
 *   same memory budget (sampling at 1/N the frame rate makes the
 *   trail N× longer in time).
 *
 *   Two parallel float arrays (`px[]`, `py[]`) instead of a single
 *   `Vec2[]` because the read path (render_trail) iterates each
 *   axis independently — saving one indirection per cell in the
 *   inner draw loop.
 *
 * Why a ring buffer (not a fixed-length array + memmove)
 *   • O(1) push: just write at `head` and bump.  memmove would be
 *     O(TRAIL_CAP) per frame — wasteful when only the LATEST entry
 *     changes.
 *   • Constant memory: the buffer never grows; old entries are
 *     silently overwritten by new ones (the "forgetting" is implicit).
 *   • Wrap-aware iteration: render_trail walks `(head - 1 - k +
 *     TRAIL_CAP) % TRAIL_CAP` to read newest-first.
 *
 * Why subsample (TRAIL_SAMPLE_STEP > 1)
 *   At 60 fps with TRAIL_CAP = 64, raw 1-sample-per-frame gives a
 *   trail covering ~1 second of motion — too short to read as a
 *   trajectory.  TRAIL_SAMPLE_STEP = 4 gives ~4 seconds without
 *   enlarging the buffer.  Trade: trail dots are placed at
 *   slightly coarser intervals (fine — render_trail's age-based
 *   fade hides the discrete spacing).
 *
 * Members
 *   px[TRAIL_CAP]  Past x positions in pixel space, ring-buffered.
 *   py[TRAIL_CAP]  Past y positions, parallel to px[].
 *   head           Index of the NEXT write slot.  After the buffer
 *                  fills, `head` wraps and the oldest entry becomes
 *                  the next slot to overwrite.
 *   count          Number of valid entries (∈ [0, TRAIL_CAP]).
 *                  Grows from 0 to TRAIL_CAP on startup; stays at
 *                  TRAIL_CAP once the buffer is full.
 *   skip           Frames since last sample.  When it reaches
 *                  TRAIL_SAMPLE_STEP, the next position is pushed
 *                  and skip resets to 0.
 *
 * Invariants
 *   0 ≤ head  < TRAIL_CAP.
 *   0 ≤ count ≤ TRAIL_CAP.
 *   0 ≤ skip  < TRAIL_SAMPLE_STEP.
 *   count < TRAIL_CAP  ⇒  px[0..count-1] are the valid positions
 *                          (the buffer hasn't wrapped yet).
 *   count == TRAIL_CAP ⇒  every slot is valid; head points at the
 *                          OLDEST entry (next to be overwritten).
 *
 * References
 *   Knuth, "The Art of Computer Programming" Vol. 1 §2.2.2 —
 *     circular buffer ("circular queue") as the canonical bounded
 *     FIFO data structure.  Same pattern as a ring-shift in DSP.
 */
typedef struct {
    float px[TRAIL_CAP];
    float py[TRAIL_CAP];
    int   head;             /* index of next write slot       */
    int   count;            /* valid entries (≤ TRAIL_CAP)    */
    int   skip;             /* frames since last sample        */
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = t->skip = 0;
}

static void trail_push(Trail *t, float px, float py)
{
    if (++t->skip < TRAIL_SAMPLE_STEP) return;
    t->skip          = 0;
    t->px[t->head]   = px;
    t->py[t->head]   = py;
    t->head          = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §6  robot — state, inverse and forward kinematics, integration         */
/* ===================================================================== */

/* ── §6.1 Robot sub-structs ───────────────────────────────────────── */

/*
 * Pose — robot's position + heading in WORLD pixel space.
 *
 * Intent
 *   The full configuration vector of a 2-D mobile robot:
 *   (x, y) for position, θ for heading.  Standard SE(2)
 *   representation [Siegwart §3.2.2].  Every frame, step_pose
 *   advances this from the current wheel speeds (forward kinematics).
 *
 * Coordinate convention
 *   Pixel space, top-left origin → +x is RIGHT, +y is DOWN.
 *   θ = 0 points along +x (east); θ = π/2 points along +y (south).
 *   This is "graphics convention" — the math-textbook y-axis is
 *   FLIPPED, which matters for the wheel-offset formulas in
 *   render_robot.  See [2] Siegwart §3.2.4 for the world-frame
 *   integration.
 *
 * Members
 *   px, py   Position in WORLD pixel space (top-left = 0, 0).
 *            Wrapped at world edges by wrap_position to give the
 *            visual a torus topology (drive off the right edge,
 *            re-enter from the left).
 *   theta    Heading in radians; wrapped to (−π, π] each tick by
 *            wrap_theta to keep printf-able values readable.
 *
 * Invariants
 *   0 ≤ px < world.wpx, 0 ≤ py < world.hpx after wrap_position.
 *   −π < theta ≤ π after wrap_theta.
 *
 * References
 *   [2] Siegwart §3.2.2 — pose representation for differential-drive.
 */
typedef struct {
    float px, py, theta;
} Pose;

/*
 * WheelSpeeds — current wheel linear speeds (pixels/sec).
 *
 * Intent
 *   The "intermediate" state in the kinematic pipeline:
 *
 *     Commands  → INVERSE KINEMATICS → WheelSpeeds
 *     WheelSpeeds → FORWARD KINEMATICS → Pose
 *
 *   Computed every frame by compute_wheels from the user's
 *   (v_cmd, w_cmd) commands and the axle length.  Read every frame
 *   by step_pose to advance the Pose.  Reflected in the renderer
 *   as green/red velocity arrows at each wheel.
 *
 * Why store wheel speeds (rather than recompute on demand)
 *   Two consumers need them: the integrator AND the renderer.
 *   The renderer wants per-wheel direction arrows whose length
 *   reads as |vL| / V_MAX and |vR| / V_MAX.  Caching means
 *   one compute_wheels per frame instead of two.
 *
 * Members
 *   vL   Left wheel  linear speed (pixels/sec).  Positive = forward.
 *   vR   Right wheel linear speed (pixels/sec).  Positive = forward.
 *
 * Invariants
 *   |vL| ≤ V_MAX, |vR| ≤ V_MAX (clamped in compute_wheels).
 *
 * References
 *   [2] Siegwart §3.2.4 — inverse kinematics for differential-drive.
 *   [4] Dudek & Jenkin §4.2 — wheel-speed primitives.
 */
typedef struct {
    float vL, vR;
} WheelSpeeds;

/*
 * Commands — user's high-level (linear, angular) command pair.
 *
 * Intent
 *   What the user is ASKING the robot to do: drive forward at v_cmd
 *   while turning at w_cmd.  Distinct from WheelSpeeds because this
 *   is the user-input layer; the robot has to translate it to wheel
 *   speeds via INVERSE KINEMATICS each frame.
 *
 *   The keyboard input handler (scene_apply_keys) ramps these
 *   smoothly so a key-down doesn't snap the robot to V_MAX instantly.
 *
 * Members
 *   v_cmd    Desired linear speed (pixels/sec).  Updated by W/S
 *            keys with V_RATE ramp; decays at V_DECAY_PER_SEC.
 *   w_cmd    Desired angular speed (radians/sec).  Updated by A/D
 *            keys with W_RATE ramp; decays at W_DECAY_PER_SEC.
 *
 * Invariants
 *   |v_cmd| ≤ V_MAX, |w_cmd| ≤ W_MAX (clamped in scene_apply_keys).
 *
 * References
 *   [2] Siegwart §3.2.4 — (v, ω) is the canonical command pair for
 *       differential-drive in the unicycle model.
 */
typedef struct {
    float v_cmd, w_cmd;
} Commands;

/*
 * Robot — the simulated agent.
 *
 * Layered ownership
 *
 *     Robot
 *       ├── pose     : Pose          ← (px, py, θ) configuration
 *       ├── wheels   : WheelSpeeds   ← (vL, vR) intermediate state
 *       ├── cmd      : Commands      ← (v_cmd, w_cmd) user inputs
 *       ├── trail    : Trail         ← ring buffer of past positions
 *       └── axle     : float         ← wheel separation (config; frozen)
 *
 *   Three layered kinematic stages of state:
 *     INPUT  → cmd   (user keys)
 *     INVERSE KINEMATICS → wheels  (compute_wheels)
 *     FORWARD KINEMATICS → pose    (step_pose)
 *
 *   This makes the data-flow direction explicit in the struct shape.
 *
 * Why `paused` lives on Scene.sim (not on Robot)
 *   `paused` is a SCENE-level concern (gate for the whole tick), not
 *   a per-robot property.  Moving it to SimControls keeps Robot
 *   purely about the simulated agent.
 *
 * References
 *   [2] Siegwart, Nourbakhsh & Scaramuzza (2011),
 *       "Introduction to Autonomous Mobile Robots" (2nd ed.), MIT
 *       Press, §3.2 — canonical reference for differential-drive
 *       kinematics in textbook form.
 *   [4] Dudek & Jenkin (2010), "Computational Principles of Mobile
 *       Robotics" (2nd ed.), §4.2 — wheel-speed primitives.
 */
typedef struct {
    Pose        pose;
    WheelSpeeds wheels;
    Commands    cmd;
    Trail       trail;
    float       axle;
} Robot;

/* ── §6.2 compute_wheels — INVERSE KINEMATICS (commands → wheels) ── */

/*
 * The user thinks "I want to go forward at v_cmd while turning at
 * w_cmd". The robot only knows how to set wheel speeds.
 *
 * Given an axle of width L = robot.axle and body centre velocity v,
 * angular ω:
 *
 *     left wheel  travels a smaller arc on a right turn (ω > 0):
 *         vL = v − ω · L/2
 *     right wheel travels a larger arc:
 *         vR = v + ω · L/2
 *
 * That's it — two scalar subtractions/additions. The hard part is
 * remembering it's the DIFFERENCE that causes turning, not a
 * separate steering input.
 *
 * Each wheel is independently clamped to ±V_MAX so neither motor
 * saturates. If the commanded combo would saturate, the actual
 * (v, ω) achieved is less than commanded — but the robot still
 * moves consistently, just slower.
 */
static void compute_wheels(Robot *r)
{
    float half = r->axle * 0.5f;
    r->wheels.vL = clampf(r->cmd.v_cmd - r->cmd.w_cmd * half, -V_MAX, V_MAX);
    r->wheels.vR = clampf(r->cmd.v_cmd + r->cmd.w_cmd * half, -V_MAX, V_MAX);
}

/* ── §6.3 step_pose — FORWARD KINEMATICS + Euler integration ──────── */

/*
 * Given the two wheel speeds, recover (v, ω):
 *
 *     v  = (vL + vR) / 2          (the body's centre speed)
 *     ω  = (vR − vL) / axle       (the body's spin rate)
 *
 * Then advance the pose by one Euler step:
 *
 *     x ← x + v · cos(θ) · dt
 *     y ← y + v · sin(θ) · dt
 *     θ ← θ + ω · dt
 *
 * Forward Euler is more than accurate enough at 60 fps: the worst-
 * case arc error per step is O((ω·dt)²) — a fraction of a pixel.
 * For higher ω or larger dt you'd switch to the exact ICC arc
 * integrator (see CONCEPTS).
 *
 * After the integration, we wrap θ into (−π, π] so the HUD readout
 * (in degrees) doesn't show "thirty-thousand degrees" after a long
 * spin and so atan2 / lerp behave well.
 */
static void step_pose(Robot *r, float dt)
{
    float v     = (r->wheels.vL + r->wheels.vR) * 0.5f;
    float omega = (r->wheels.vR - r->wheels.vL) / r->axle;

    r->pose.px    += v     * cosf(r->pose.theta) * dt;
    r->pose.py    += v     * sinf(r->pose.theta) * dt;
    r->pose.theta += omega * dt;

    /* Wrap heading into (−π, π]. */
    while (r->pose.theta >  (float)M_PI) r->pose.theta -= 2.0f * (float)M_PI;
    while (r->pose.theta < -(float)M_PI) r->pose.theta += 2.0f * (float)M_PI;
}

/* ── §6.4 wrap_position — toroidal screen edges ──────────────────── */

/*
 * If the robot drives off the right edge it reappears on the left;
 * off the bottom it reappears at the top, and so on. This is not
 * physically realistic but keeps the robot visible at all times,
 * which is what you want in a teaching simulator.
 */
static void wrap_position(Robot *r, int wpx, int hpx)
{
    if (r->pose.px <  0.0f)         r->pose.px += (float)wpx;
    if (r->pose.px >= (float)wpx)   r->pose.px -= (float)wpx;
    if (r->pose.py <  0.0f)         r->pose.py += (float)hpx;
    if (r->pose.py >= (float)hpx)   r->pose.py -= (float)hpx;
}

/* ── §6.5 robot_init / robot_reset ───────────────────────────────── */

static void robot_init(Robot *r, int wpx, int hpx)
{
    memset(r, 0, sizeof *r);
    r->axle  = AXLE_PX;
    r->pose.px    = (float)wpx * 0.5f;
    r->pose.py    = (float)hpx * 0.5f;
    r->pose.theta = 0.0f;
}

static void robot_reset(Robot *r, int wpx, int hpx)
{
    /* Preserve constant geometry across reset; everything else zero. */
    float axle = r->axle;
    memset(r, 0, sizeof *r);
    r->axle  = axle;
    r->pose.px    = (float)wpx * 0.5f;
    r->pose.py    = (float)hpx * 0.5f;
    r->pose.theta = 0.0f;
}

/* ── §6.6 robot_tick — one frame of physics ──────────────────────── */

/*
 * The orchestrator.  Reads the user's (v_cmd, w_cmd), runs
 * INVERSE KINEMATICS to find wheel speeds, runs FORWARD KINEMATICS
 * + Euler integration to advance the pose, wraps at the world
 * boundary, and pushes the new position into the trail buffer.
 *
 * Pause is handled by the CALLER (scene_tick) so this function is
 * pure physics — easier to reason about in isolation.
 */
static void robot_tick(Robot *r, float dt, int wpx, int hpx)
{
    compute_wheels(r);                      /* commands → wheel speeds */
    step_pose     (r, dt);                  /* wheel speeds → new pose */
    wrap_position (r, wpx, hpx);
    trail_push    (&r->trail, r->pose.px, r->pose.py);
}

/* ===================================================================== */
/* §7  scene — input → command translation, tick orchestration            */
/* ===================================================================== */

/*
 * Keys — one bool per action, set fresh each frame from getch().
 *
 * Intent
 *   Bit-vector-style snapshot of the keyboard state at the start of
 *   each tick.  collect_input populates this by draining ncurses'
 *   input queue; scene_apply_keys consumes it to update Commands.
 *
 *   Using a struct of bools (rather than a bitmask) keeps
 *   scene_apply_keys readable and lets MULTIPLE keys be active
 *   simultaneously without bitwise ops (e.g. W + A → drive forward
 *   while turning left).
 *
 * Members
 *   fwd, rev        Throttle (W/S keys) — ramps cmd.v_cmd up/down.
 *   left, right     Turn (A/D keys) — ramps cmd.w_cmd left/right.
 *   spin_l, spin_r  Instant spin in place at ±W_MAX (Z/E keys).
 *   stop            Full-stop override (SPACE) — zeros both commands.
 *
 * Invariants
 *   All fields false at the start of each frame (memset in collect_input).
 */
typedef struct {
    bool fwd, rev, left, right, spin_l, spin_r, stop;
} Keys;

/*
 * InputState — the keyboard input layer.
 *
 * Intent
 *   Single-field wrapper around Keys.  Pedagogically useful as a
 *   named sub-struct of Scene so the data flow reads as
 *
 *       scene.input → scene.robot.cmd → scene.robot.wheels → scene.robot.pose
 *
 *   matching the kinematics pipeline (input → commands → wheels → pose).
 *
 * Members
 *   keys   Per-frame keyboard snapshot (see Keys docblock).
 */
typedef struct {
    Keys keys;
} InputState;

/*
 * World — pixel-space dimensions of the simulated world.
 *
 * Intent
 *   The simulation operates in PIXEL space (with CELL_W × CELL_H
 *   sub-pixels per cell — see §4 coords).  World holds the active
 *   pixel extents, derived from the terminal size minus the two
 *   HUD rows (one top, one bottom).
 *
 * Members
 *   wpx   World width  in PIXELS (= pw(cols)).
 *   hpx   World height in PIXELS (= ph(rows - 2); 2 reserved rows
 *         for HUD + hint strip).
 *
 * Invariants
 *   wpx > 0, hpx > 0 after scene_init.
 *   Recomputed on every SIGWINCH via scene_resize.
 *
 * References
 *   CLAUDE.md §"Core Architecture / Coordinate / Physics" for the
 *   pixel ↔ cell coordinate bridge.
 */
typedef struct {
    int wpx, hpx;
} World;

/*
 * SimControls — user-facing playback knob.
 *
 * Intent
 *   The `paused` flag gates the tick stage of the simulation loop
 *   while letting the renderer keep running (so the HUD stays live).
 *   Lives on Scene (not on Robot) because it's a SCENE-level concern
 *   — the user pauses THE WORLD, not the robot specifically.
 *
 * Members
 *   paused   true → scene_tick early-returns; rendering continues.
 *            Toggled by 'p' / 'P' key.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── world    : World        ← wpx + hpx (pixel extent)
 *       ├── robot    : Robot        ← the simulated agent
 *       ├── input    : InputState   ← per-frame Keys snapshot
 *       └── sim      : SimControls  ← paused
 *
 *   Data-flow direction (read top-to-bottom):
 *
 *       input.keys  → robot.cmd       (scene_apply_keys; user intent)
 *       robot.cmd   → robot.wheels    (compute_wheels; inverse kinematics)
 *       robot.wheels → robot.pose     (step_pose; forward kinematics)
 *       world.wpx/hpx → wrap_position (torus boundary)
 *
 *   The struct shape mirrors the data flow: every helper that
 *   advances one stage takes the relevant sub-struct as a parameter,
 *   making the "what does this function touch" question answerable
 *   from the signature.
 */
typedef struct {
    World       world;
    Robot       robot;
    InputState  input;
    SimControls sim;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->world.wpx = pw(cols);
    /* Reserve top row for HUD, bottom row for hint strip. */
    s->world.hpx = ph(rows - 2);
    memset(&s->input.keys, 0, sizeof s->input.keys);
    s->sim.paused = false;
    robot_init(&s->robot, s->world.wpx, s->world.hpx);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->world.wpx = pw(cols);
    s->world.hpx = ph(rows - 2);
}

/*
 * scene_apply_keys — translate the per-frame Keys flags into
 * updates to the robot's command state (v_cmd, w_cmd).
 *
 * Throttle:
 *   fwd / rev held → v_cmd ramps at V_RATE per second.
 *   No throttle held → v_cmd *= V_DECAY_PER_SEC^dt (≈ no decay
 *     at default V_DECAY_PER_SEC = 1.0).
 *
 * Turn:
 *   left / right held → w_cmd ramps at W_RATE per second.
 *   No turn held → w_cmd *= W_DECAY_PER_SEC^dt (strong damping).
 *
 * Spin override:
 *   spin_l / spin_r set v_cmd = 0 and w_cmd = ±W_MAX directly.
 *   This is "rotate in place", overriding any throttle ramp.
 *
 * Stop:
 *   space sets both commands to 0 immediately.
 *
 * All commands clamped to physical limits at the end.
 */
/* apply_full_stop — SPACE key.  Zeros both commands instantly.
 * Highest-priority override.  Other keys can still modify the
 * commands AFTER this in the same frame (e.g. SPACE + W in the
 * same frame nets to W's ramp). */
static inline void apply_full_stop(Commands *cmd) {
    cmd->v_cmd = 0.0f;
    cmd->w_cmd = 0.0f;
}

/* apply_throttle_ramp_or_decay — W/S key handling.
 *
 *   key_held → ramp v_cmd at ±V_RATE per second (forward Euler).
 *   no key  → exponential decay v_cmd *= V_DECAY_PER_SEC^dt
 *             (frame-rate independent — same time-constant at any dt).
 */
static inline void apply_throttle_ramp_or_decay(Commands *cmd, const Keys *k,
                                                 float dt) {
    if (k->fwd) cmd->v_cmd += V_RATE * dt;
    if (k->rev) cmd->v_cmd -= V_RATE * dt;
    if (!k->fwd && !k->rev) cmd->v_cmd *= powf(V_DECAY_PER_SEC, dt);
}

/* apply_turn_ramp_or_decay — A/D key handling.  Same shape as the
 * throttle helper but operating on the angular command. */
static inline void apply_turn_ramp_or_decay(Commands *cmd, const Keys *k,
                                             float dt) {
    if (k->right) cmd->w_cmd += W_RATE * dt;
    if (k->left)  cmd->w_cmd -= W_RATE * dt;
    if (!k->right && !k->left) cmd->w_cmd *= powf(W_DECAY_PER_SEC, dt);
}

/* apply_spin_in_place_override — Z/E key handling.  Direct
 * assignment (NOT a ramp): set v_cmd = 0 and w_cmd = ±W_MAX, so the
 * robot rotates in place at maximum angular velocity.  Overrides
 * any throttle/turn ramp from the same frame. */
static inline void apply_spin_in_place_override(Commands *cmd, const Keys *k) {
    if (k->spin_r) { cmd->v_cmd = 0.0f; cmd->w_cmd =  W_MAX; }
    if (k->spin_l) { cmd->v_cmd = 0.0f; cmd->w_cmd = -W_MAX; }
}

/* clamp_commands_to_physical_limits — final saturation step.  Any
 * combination of the above can push the commands past V_MAX / W_MAX;
 * this guarantees the integrator + inverse-kinematics stage sees
 * values within achievable bounds. */
static inline void clamp_commands_to_physical_limits(Commands *cmd) {
    cmd->v_cmd = clampf(cmd->v_cmd, -V_MAX, V_MAX);
    cmd->w_cmd = clampf(cmd->w_cmd, -W_MAX, W_MAX);
}

/*
 * scene_apply_keys — translate the per-frame Keys flags into
 * updates to Commands.  Order matters:
 *
 *   (1) STOP override     — zero both commands if SPACE held
 *   (2) THROTTLE ramp     — W/S keys ramp v_cmd; decay if neither held
 *   (3) TURN ramp         — A/D keys ramp w_cmd; decay if neither held
 *   (4) SPIN override     — Z/E keys force (v_cmd=0, w_cmd=±W_MAX)
 *   (5) CLAMP             — saturate to V_MAX / W_MAX
 *
 * STOP and SPIN apply AFTER the ramps so they win — a user can hit
 * SPACE to instantly halt regardless of whether throttle is held.
 */
static void scene_apply_keys(Scene *s, float dt)
{
    Commands  *cmd = &s->robot.cmd;
    const Keys *k  = &s->input.keys;

    if (k->stop) apply_full_stop(cmd);                       /* (1) */
    apply_throttle_ramp_or_decay  (cmd, k, dt);              /* (2) */
    apply_turn_ramp_or_decay      (cmd, k, dt);              /* (3) */
    apply_spin_in_place_override  (cmd, k);                  /* (4) */
    clamp_commands_to_physical_limits(cmd);                  /* (5) */
}

static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    scene_apply_keys(s, dt);
    robot_tick(&s->robot, dt, s->world.wpx, s->world.hpx);
}

/* ===================================================================== */
/* §8  render                                                             */
/* ===================================================================== */

/* ── §8.1 in_bounds — guard for "draw me only if I'm on screen" ──── */

/*
 * Top row 0 reserved for HUD; bottom row reserved for hint strip.
 * Drawables must live in rows 1..rows-2. cy is allowed up to rows-1
 * exclusive; cy >= rows-1 means we're trying to draw on the hint row.
 */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* ── §8.2 draw_line_dotted — a directional arrow body ────────────── */

/*
 * Step uniformly in PIXEL space, converting each sample to a cell
 * and skipping duplicates. The character grows brighter along the
 * line so the arrow has a clear "from → to" feel without using
 * angle-dependent diagonal characters.
 *
 *   t < 0.40 → '.'   (faint, near the start)
 *   t < 0.75 → 'o'   (medium)
 *   t ≥ 0.75 → '0'   (boldest, near the tip)
 */
static void draw_line_dotted(float x0, float y0, float x1, float y1,
                             int cp, attr_t extra, int cols, int rows)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.5f) return;

    int   steps   = (int)(len / (float)CELL_W) + 1;
    int   prev_cx = -9999, prev_cy = -9999;

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)(steps > 0 ? steps : 1);
        int   cx = px_to_cx(x0 + dx * t);
        int   cy = px_to_cy(y0 + dy * t);
        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (!in_bounds(cx, cy, cols, rows)) continue;

        chtype ch = (t < 0.40f) ? '.' : (t < 0.75f) ? 'o' : '0';
        mvaddch(cy, cx, ch | (chtype)COLOR_PAIR(cp) | (chtype)extra);
    }
}

/* ── §8.3 tip_char — arrowhead character chosen by direction ─────── */

/*
 * Cardinal directions get sharp ASCII arrows (>, v, <, ^) that
 * immediately read as directional. Diagonal octants get 'o' — a
 * round, neutral character that doesn't claim a specific angle
 * (which '\' or '/' would inevitably misrepresent on a 2:1 cell
 * grid).
 */
static chtype tip_char(float theta)
{
    float deg = fmodf(theta * (180.0f / (float)M_PI) + 360.0f, 360.0f);
    if (deg < 22.5f  || deg >= 337.5f) return '>';   /* east       */
    if (deg < 67.5f )                  return 'o';   /* south-east */
    if (deg < 112.5f)                  return 'v';   /* south      */
    if (deg < 157.5f)                  return 'o';   /* south-west */
    if (deg < 202.5f)                  return '<';   /* west       */
    if (deg < 247.5f)                  return 'o';   /* north-west */
    if (deg < 292.5f)                  return '^';   /* north      */
    return                                    'o';   /* north-east */
}

/* ── §8.4 draw_wheel_arrow — green/red arrow per wheel ───────────── */

/*
 * Draws a velocity arrow originating at the wheel's pixel position
 * and extending along ±heading by a length proportional to
 * |v_wheel| / V_MAX. Forward = green, reverse = red.
 *
 * If the wheel is essentially stopped (|v| < threshold), no arrow
 * is drawn — the absence is information.
 */
static void draw_wheel_arrow(float wx_px, float wy_px,
                             float v_wheel, float theta,
                             int cols, int rows)
{
    float alen = fabsf(v_wheel / V_MAX) * VEL_ARROW_PX;
    if (alen < 1.0f) return;

    float sign = (v_wheel >= 0.0f) ? 1.0f : -1.0f;
    float ex   = wx_px + cosf(theta) * sign * alen;
    float ey   = wy_px + sinf(theta) * sign * alen;
    int   cp   = (v_wheel >= 0.0f) ? CP_VEL_FWD : CP_VEL_REV;

    draw_line_dotted(wx_px, wy_px, ex, ey, cp, A_BOLD, cols, rows);

    int eax = px_to_cx(ex), eay = px_to_cy(ey);
    if (in_bounds(eax, eay, cols, rows)) {
        float ta = (v_wheel >= 0.0f) ? theta : theta + (float)M_PI;
        mvaddch(eay, eax, tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
    }
}

/* ── §8.5 render_trail — paint past positions ────────────────────── */

/*
 * Iterate newest-first (head−1, head−2, …). Age is the fraction of
 * the way from "newest" (k=0) to "oldest" (k=count-1):
 *
 *   age < 0.12  → BOLD '.' bright cyan   (very recent)
 *   age < 0.35  → DIM  '.' cyan          (recent, fading)
 *   age ≥ 0.35  → DIM  ':' dim blue      (old — note the ':' for
 *                                          visual recede)
 */
static void render_trail(const Robot *r, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int tc  = px_to_cx(r->trail.px[idx]);
        int tr  = px_to_cy(r->trail.py[idx]);
        if (!in_bounds(tc, tr, cols, rows)) continue;

        float  age = (float)k / (float)(r->trail.count > 1 ? r->trail.count - 1 : 1);
        int    cp  = (age < 0.35f) ? CP_TRAIL_NEW : CP_TRAIL_OLD;
        attr_t at  = (age < 0.12f) ? A_BOLD       : A_DIM;
        char   ch  = (age < 0.35f) ? '.'          : ':';

        attron (COLOR_PAIR(cp) | at);
        mvaddch(tr, tc, (chtype)ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/* ── §8.6 render_robot — body, wheels, arrows ────────────────────── */

/*
 * WheelPositions — pixel coordinates of the two wheels.
 *
 * Computed from pose + axle: the wheels sit PERPENDICULAR to the
 * heading at ±axle/2 from the body centre.  In y-down screen
 * coordinates:
 *
 *     left_wheel  = body + ( +sin θ, −cos θ ) · axle/2
 *     right_wheel = body + ( −sin θ, +cos θ ) · axle/2
 *
 * (Math-textbook formulas assume y-UP, so they have the signs
 *  flipped.  Watch out — see [2] Siegwart §3.2 figure 3.5.)
 */
typedef struct {
    float lx, ly;   /* left  wheel pixel coordinates */
    float rx, ry;   /* right wheel pixel coordinates */
} WheelPositions;

/*
 * compute_wheel_positions — body pose → wheel pixel positions.
 *
 * Forward kinematics for the wheel locations only (the body's
 * forward kinematics are in step_pose).  Both wheels are
 * BODY-FIXED — they rotate with the heading, so their positions
 * are recomputed each frame from the current θ.
 */
static inline WheelPositions compute_wheel_positions(const Robot *r) {
    float sinT = sinf(r->pose.theta);
    float cosT = cosf(r->pose.theta);
    float half_axle = r->axle * 0.5f;

    WheelPositions wp;
    /* left wheel = body + (+sinθ, −cosθ) · axle/2  (y-down convention) */
    wp.lx = r->pose.px + sinT * half_axle;
    wp.ly = r->pose.py - cosT * half_axle;
    /* right wheel = body + (−sinθ, +cosθ) · axle/2 */
    wp.rx = r->pose.px - sinT * half_axle;
    wp.ry = r->pose.py + cosT * half_axle;
    return wp;
}

/*
 * paint_glyph_at_cell — attron / mvaddch / attroff sandwich for one
 * cell, gated on in_bounds.  Centralises the colour-pair setup +
 * bounds check + chtype cast used across all per-cell paints.
 */
static inline void paint_glyph_at_cell(int cx, int cy, chtype glyph,
                                        int color_pair, attr_t extra_attrs,
                                        int cols, int rows) {
    if (!in_bounds(cx, cy, cols, rows)) return;
    attron (COLOR_PAIR(color_pair) | extra_attrs);
    mvaddch(cy, cx, glyph | (chtype)COLOR_PAIR(color_pair) | (chtype)extra_attrs);
    attroff(COLOR_PAIR(color_pair) | extra_attrs);
}

/*
 * paint_heading_arrow — yellow dotted line from body centre extending
 * ARROW_PX pixels along the current heading.  Sub-helpers:
 *   - draw_line_dotted     — the shaft (dotted line in pixel space)
 *   - paint_glyph_at_cell  — the arrowhead glyph at the tip
 * The arrowhead glyph is chosen by tip_char() based on θ — cardinal
 * directions get sharp arrows (> v < ^); diagonals get 'o'.
 */
static inline void paint_heading_arrow(const Robot *r, int cols, int rows) {
    float cosT = cosf(r->pose.theta), sinT = sinf(r->pose.theta);
    float tip_x_px = r->pose.px + cosT * ARROW_PX;
    float tip_y_px = r->pose.py + sinT * ARROW_PX;

    /* shaft */
    draw_line_dotted(r->pose.px, r->pose.py, tip_x_px, tip_y_px,
                     CP_HEAD, A_BOLD, cols, rows);

    /* arrowhead at the tip */
    int tip_cx = px_to_cx(tip_x_px), tip_cy = px_to_cy(tip_y_px);
    paint_glyph_at_cell(tip_cx, tip_cy, tip_char(r->pose.theta),
                        CP_HEAD, A_BOLD, cols, rows);
}

/*
 * paint_wheel_label — a single 'L' or 'R' glyph at the wheel's
 * cell position.  Drawn after the velocity arrows but before the
 * body so the body always wins overlap at the centre.
 */
static inline void paint_wheel_label(float wx_px, float wy_px,
                                      chtype label_glyph, int color_pair,
                                      int cols, int rows) {
    paint_glyph_at_cell(px_to_cx(wx_px), px_to_cy(wy_px),
                        label_glyph, color_pair, A_BOLD, cols, rows);
}

/*
 * render_robot — paint the robot at its current pose.
 *
 * Painter's order — LAST write wins:
 *   (1) Trail dots           — oldest at the bottom of the z-stack
 *   (2) Wheel velocity arrows — green = forward, red = reverse
 *   (3) Heading arrow         — yellow dotted line from body centre
 *   (4) Wheel labels 'L', 'R' — anchor points for the user
 *   (5) Body '@'              — always on top
 *
 * Wheel positions come from compute_wheel_positions (body-fixed,
 * recomputed each frame from θ).
 */
static void render_robot(const Robot *r, int cols, int rows)
{
    WheelPositions wp = compute_wheel_positions(r);

    /* (1) Trail dots — oldest first; bottom of the z-stack */
    render_trail(r, cols, rows);

    /* (2) Wheel velocity arrows — visual encoding of (vL, vR) */
    draw_wheel_arrow(wp.lx, wp.ly, r->wheels.vL, r->pose.theta, cols, rows);
    draw_wheel_arrow(wp.rx, wp.ry, r->wheels.vR, r->pose.theta, cols, rows);

    /* (3) Heading arrow from body centre — direction the body faces */
    paint_heading_arrow(r, cols, rows);

    /* (4) Wheel labels — 'L' / 'R' anchor characters */
    paint_wheel_label(wp.lx, wp.ly, 'L', CP_WHL_L, cols, rows);
    paint_wheel_label(wp.rx, wp.ry, 'R', CP_WHL_R, cols, rows);

    /* (5) Body — drawn last, never occluded */
    paint_glyph_at_cell(px_to_cx(r->pose.px), px_to_cy(r->pose.py),
                        '@', CP_BODY, A_BOLD, cols, rows);
}

/* ── §8.7 render_hud — yellow status row 0, cyan hint bottom row ─── */

/* OMEGA_INFINITESIMAL_THRESHOLD — below this |ω|, the instantaneous
 * curvature radius is effectively infinite (a straight line).  We
 * print "INF" in the HUD instead of a numeric R to avoid 10⁹-style
 * garbage values.  Threshold is in rad/sec; 1e-3 ≈ 0.06 deg/sec,
 * imperceptible. */
enum { /* enums can't hold floats — kept as #define-style note */ HUD_FLOAT_NOTES = 0 };
#define OMEGA_INFINITESIMAL_THRESHOLD 1e-3f   /* rad/sec */
#define R_INFINITE_DISPLAY_THRESHOLD  9999.0f /* pixels; bigger → render "INF" */
enum { HUD_TOP_ROW = 0 };

/* body_linear_velocity — average of the two wheel speeds.
 * Derived from the forward kinematics: v = (vL + vR) / 2. */
static inline float body_linear_velocity(const Robot *r) {
    return (r->wheels.vL + r->wheels.vR) * 0.5f;
}

/* body_angular_velocity — wheel-speed difference / axle.
 * Derived from the forward kinematics: ω = (vR − vL) / axle. */
static inline float body_angular_velocity(const Robot *r) {
    return (r->wheels.vR - r->wheels.vL) / r->axle;
}

/* instantaneous_turn_radius_px — radius of the instantaneous circle
 * the body is travelling on.  R = v / ω at non-zero ω; returns a
 * sentinel large value (1e9) when |ω| is below the infinitesimal
 * threshold so the caller can detect "straight-line" and print "INF". */
static inline float instantaneous_turn_radius_px(float v, float omega) {
    if (fabsf(omega) <= OMEGA_INFINITESIMAL_THRESHOLD) return 1e9f;
    return v / omega;
}

/* radians_to_degrees — unit conversion used only by the HUD. */
static inline float radians_to_degrees(float radians) {
    return radians * (180.0f / (float)M_PI);
}

/* hud_paint_text_padded — paint `text` at (row, col) in pair `pair`
 * + A_BOLD, then pad the rest of the row with spaces so the row's
 * background colour reaches edge-to-edge (otherwise the terminal's
 * default bg shows through past the printed text). */
static inline void hud_paint_text_padded(int row, int col, int pair,
                                          const char *text,
                                          int total_cols) {
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    int used = (int)strlen(text);
    for (int x = col + used; x < total_cols; x++)
        mvaddch(row, x, ' ');
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* hud_paint_text — same as above but WITHOUT padding (the bottom
 * key-hint strip doesn't need to span the full width). */
static inline void hud_paint_text(int row, int col, int pair,
                                   const char *text) {
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format_hud_status — write the top-row status string into `buf`.
 * Reports fps, pose (px, py, θ°), body velocity v, body angular
 * velocity ω, instantaneous turn radius R (or "INF" when straight),
 * the two wheel speeds, and the paused/running tag. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const Robot *r = &s->robot;
    float v     = body_linear_velocity (r);
    float omega = body_angular_velocity(r);
    float R     = instantaneous_turn_radius_px(v, omega);
    float theta_deg = radians_to_degrees(r->pose.theta);
    const char *state_label = s->sim.paused ? "PAUSED" : "running";

    bool render_R_as_infinity = (fabsf(R) > R_INFINITE_DISPLAY_THRESHOLD);
    if (render_R_as_infinity) {
        snprintf(buf, buflen,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:INF  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->pose.px, r->pose.py, theta_deg, v, omega,
                 r->wheels.vL, r->wheels.vR, state_label);
    } else {
        snprintf(buf, buflen,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:%+6.0f  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->pose.px, r->pose.py, theta_deg, v, omega, R,
                 r->wheels.vL, r->wheels.vR, state_label);
    }
}

/* draw_hud_status — left-justified status row 0 (PAIR_HUD yellow). */
static void draw_hud_status(const Scene *s, double fps, int cols) {
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    hud_paint_text_padded(HUD_TOP_ROW, 0, PAIR_HUD, buf, cols);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan). */
static void draw_hud_hint(int rows) {
    static const char *KEY_HINT =
        " q:quit  spc:stop  p:pause  r:reset  "
        "WS:throttle  AD:turn  ZE:spin in place ";
    hud_paint_text(rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/*
 * render_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0 (PAIR_HUD, BOLD): fps + pose + (v, ω) + R + wheel speeds + state
 *   Bottom row (PAIR_HINT, BOLD): key bindings strip
 *
 * Both pairs sit on default background (-1) so they remain legible
 * regardless of what the simulation paints behind them.
 */
static void render_hud(const Scene *s, double fps, int cols, int rows)
{
    draw_hud_status(s, fps, cols);
    draw_hud_hint  (rows);
}

/* ── §8.8 scene_draw — one frame's worth of rendering ────────────── */

static void scene_draw(const Scene *s, double fps, int cols, int rows)
{
    erase();
    render_robot(&s->robot, cols, rows);
    render_hud  (s, fps, cols, rows);
}

/* ===================================================================== */
/* §9  screen — ncurses init / present                                    */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Tracks the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the back buffer;
 *   this struct is the source-of-truth for CELL-space dimensions,
 *   refreshed via getmaxyx in screen_init / screen_resize.
 *
 *   The simulation lives in PIXEL space (CELL_W × CELL_H sub-pixels
 *   per cell, see §4 coords).  Scene.world.{wpx, hpx} TRACKS this
 *   struct (they're the pixel-space view of the same dimensions,
 *   derived via `pw(cols)` and `ph(rows-2)` in scene_init).  Two
 *   layers exist because the sim helpers reason in pixel units
 *   (smooth float positions, sub-cell precision) while the renderer
 *   reasons in cells.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_present touch ncurses' initscr / endwin /
 *     doupdate.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with every other demo in this project — `{cols, rows}`
 *     is the canonical Screen shape.
 *
 * Members
 *   cols   Terminal width in CELLS (getmaxyx).
 *   rows   Terminal height in CELLS.  Row 0 hosts the HUD status
 *          strip; row (rows - 1) hosts the key-hint strip.  The
 *          simulation paints into rows [1, rows-2].
 *
 * Invariants
 *   cols > 0, rows > 2 (need at least 1 sim row + 2 HUD rows).
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   scene_resize (which also recomputes World.wpx/hpx at the new
 *   extent).
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10  app — signals, resize, variable-dt main loop                      */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; this accumulates frame_count + elapsed
 * nanoseconds over a 500 ms window and emits a smoothed `display`
 * value each time the window fills.  Same shape as the FpsCounter on
 * every other file in this project.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene         simulation state (World + Robot + InputState + sim)
 *   screen        terminal cell extent + ncurses lifecycle
 *   fps           rolling-window fps estimator for HUD
 *   running       sig_atomic_t flag cleared by SIGINT / SIGTERM + 'q'
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 *
 * `g_app` is the program's only file-scope mutable state.  Signal
 * handlers reach the flags through it; everything else flows via
 * `App *app` parameter.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * collect_input — drain the ncurses input queue, populate the
 * per-frame Keys flags, and handle "instant-effect" keys (quit,
 * pause, reset). Multiple keys can be active in one frame so the
 * collector ORs the bools together.
 */
static void collect_input(App *app)
{
    Keys *k = &app->scene.input.keys;
    memset(k, 0, sizeof *k);

    int ch;
    while ((ch = getch()) != ERR) {
        switch (ch) {
        case 'q': case 'Q': case 27:
            app->running = 0;
            break;

        case 'w': case 'W': case KEY_UP:    k->fwd    = true; break;
        case 's': case 'S': case KEY_DOWN:  k->rev    = true; break;
        case 'a': case 'A': case KEY_LEFT:  k->left   = true; break;
        case 'd': case 'D': case KEY_RIGHT: k->right  = true; break;
        case 'e': case 'E':                 k->spin_r = true; break;
        case 'z': case 'Z':                 k->spin_l = true; break;
        case ' ':                           k->stop   = true; break;

        case 'p': case 'P':
            app->scene.sim.paused = !app->scene.sim.paused;
            break;

        case 'r': case 'R':
            robot_reset(&app->scene.robot,
                        app->scene.world.wpx, app->scene.world.hpx);
            trail_clear(&app->scene.robot.trail);
            break;

        default: break;
        }
    }
}

/*
 * main — variable-dt render loop.
 *
 *   (1) atexit + signal handlers
 *   (2) bring up Screen + App; init Scene at current terminal size
 *   (3) loop:
 *       (a) handle pending SIGWINCH
 *       (b) measure dt (capped to prevent spiral-of-death)
 *       (c) drain non-blocking input
 *       (d) advance Scene (apply_keys + robot_tick)
 *       (e) rolling-window fps counter
 *       (f) draw + present
 *       (g) frame cap: sleep so we don't burn the next slot's budget
 */
int main(void)
{
    /* (1) atexit + signal handlers */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (2) Bring up Screen + App; init Scene */
    App *app     = &g_app;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* (3a) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize (&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* (3b) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3c) drain input */
        collect_input(app);

        /* (3d) advance the Scene (apply_keys + robot_tick) */
        scene_tick(&app->scene, dt);

        /* (3e) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (3f) draw + present */
        scene_draw(&app->scene, app->fps.display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* (3g) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
