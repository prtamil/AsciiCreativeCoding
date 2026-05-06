/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * moving_jump_spring_leg_robot.c — pogo-stick robot hopping across procedural terrain
 *
 * DEMO: A spring-loaded one-legged robot at the left edge of the
 *       screen. It LOADS its spring (body sinks, energy stored
 *       quadratically), RELEASES (energy → upward velocity),
 *       FLIES through the air on a parabolic arc under gravity,
 *       LANDS on the procedural terrain, and starts loading
 *       again. The camera stays still while the robot hops
 *       rightward; once the robot reaches 80 % of the screen the
 *       camera follows so fresh terrain scrolls in. The cycle
 *       repeats forever.
 *
 *           ┌──────────────────────────────────────────────────┐
 *           │                                                  │
 *           │            O                                     │
 *           │           ╱ ╲     ← parabolic arc (FLIGHT)       │
 *           │         ╱     ╲                                  │
 *           │       ╱         ╲                                │
 *           │      .           ↘                               │
 *           │  @                  *                            │
 *           │  |                                               │
 *           │  )    ← spring leg                               │
 *           │  |                                               │
 *           │  v        slope-adapted launch angle             │
 *           │ /‾\___    /‾‾‾\___/‾‾‾‾\__/‾‾‾                   │
 *           │####\_____/#####\_/######\##                      │
 *           └──────────────────────────────────────────────────┘
 *
 *           '@' COMPRESS — body sinks, spring loads
 *           'O' FLIGHT   — projectile motion under gravity
 *           '*' LAND     — impact, body locked, brief flash
 *
 *       Real-world analogues:
 *           ─ pogo stick (the obvious one)
 *           ─ a flea jumping (uses a pre-loaded resilin spring)
 *           ─ MIT Cheetah's leg (real-world springy-legged robot)
 *           ─ catapult / trebuchet (release stored energy)
 *
 * Study alongside:
 *   robots/diff_drive_robot.c       — wheeled contrast: rolling
 *                                      instead of jumping.
 *   particle_systems/2stroke.c      — slider-crank kinematics
 *                                      (similar "stored energy
 *                                      released cyclically" theme).
 *   matrix_rain/fireworks_rain.c    — also has a state machine
 *                                      (IDLE → RISING → EXPLODED)
 *                                      and projectile physics.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  noise        — 1-D value noise (terrain primitive)
 *   §6  terrain      — height query, slope, surface glyph
 *   §7  trail        — ring buffer of recent body positions
 *   §8  robot        — Phase, Robot, spring physics, per-state ticks
 *   §9  render       — paint terrain, trail, spring, body, PE bar, HUD
 *   §10 screen       — ncurses init / present
 *   §11 app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume physics
 *   r                reset to start
 *   f                cycle floor (FLAT ↔ PERLIN)
 *   n                new terrain seed (only matters in PERLIN mode)
 *   a                cycle horizontal speed (1.0× .. 3.0×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/moving_jump_spring_leg_robot.c \
 *       -o moving_jump_spring_leg_robot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three-phase state machine driven by simple physics.
 *
 *                 (A) COMPRESS — spring is being loaded. The body
 *                     sinks toward the foot as `spring_compress`
 *                     ramps from 0 to SPRING_COMPRESS_MAX over
 *                     T_COMPRESS seconds. At full compression we
 *                     compute the launch velocity from Hooke's law
 *                     (energy conservation) and the slope-adapted
 *                     launch angle, then transition to FLIGHT.
 *
 *                 (B) FLIGHT — Newtonian projectile motion. Forward
 *                     Euler integrates `body_py += vy · dt` and
 *                     `vy += GRAVITY · dt`. Horizontal velocity vx
 *                     is constant (no air resistance). When the
 *                     foot's projected position reaches the floor
 *                     under the body's current x, transition to LAND.
 *
 *                 (C) LAND — body locked at landing pose for a brief
 *                     T_LAND seconds (impact flash visualised as '*').
 *                     Then transition back to COMPRESS, completing
 *                     the cycle.
 *
 *                 The camera is a slim helper: while the robot stays
 *                 in the left 80 % of the screen, the camera is
 *                 frozen at world origin. Once the robot crosses 80 %,
 *                 the camera exponentially chases a target that
 *                 keeps the robot pinned at the trigger column —
 *                 producing a smooth follow-cam without snapping.
 *
 * Data-structure: One Robot struct holds: pose (body_px, body_py),
 *                 foot position, velocity (vx, vy), the current
 *                 spring compression in pixels, the phase enum and a
 *                 phase-time accumulator, the cam_x scroll offset,
 *                 the floor mode flag, the speed-level index, the
 *                 trail ring buffer, and a paused flag. No heap
 *                 allocation post-init.
 *
 * Rendering     : Painter's order — last write wins:
 *                 (1) trail dots (oldest first so newest paints over)
 *                 (2) terrain (multi-row column sweep)
 *                 (3) spring leg coil (only during COMPRESS / LAND)
 *                 (4) body glyph (always on top)
 *                 (5) energy gauge (only during COMPRESS)
 *                 (6) HUD row 0 + hint row last
 *                 The whole frame is in cell coordinates; conversion
 *                 from pixel-space physics happens at the very end.
 *
 * Performance   : O(cols) per frame for terrain + trail. Single
 *                 floor_y_at() call per terrain column = O(cols)
 *                 trig calls plus the two-octave noise lookup. At
 *                 100 cols × 60 fps that's ~6000 noise samples/sec,
 *                 microseconds. ncurses redraw dominates.
 *
 * References    :
 *   Wikipedia, "Hooke's law" — F = -kx and the quadratic potential
 *     U = ½kx².  https://en.wikipedia.org/wiki/Hooke%27s_law
 *   Wikipedia, "Projectile motion" — derivation of range and peak
 *     for ballistic motion under uniform gravity.
 *     https://en.wikipedia.org/wiki/Projectile_motion
 *   Marc Raibert, "Legged Robots That Balance" (MIT Press, 1986) —
 *     the foundational paper on spring-mass robot locomotion;
 *     the SLIP (Spring-Loaded Inverted Pendulum) model directly
 *     inspires this demo.
 *   Inigo Quilez, "Value noise" — the cosine-interpolated 1-D noise
 *     used here for terrain.  https://iquilezles.org/articles/noise/
 *   This project, robots/diff_drive_robot.c — the wheeled contrast.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The robot is a spring on a stick. It LOADS the spring (storing
 * energy quadratically: PE = ½kx²), RELEASES (the spring snaps back,
 * converting potential energy to kinetic energy and launching the
 * body upward at an angle), FLIES through the air under gravity (a
 * parabolic arc — vx constant, vy decelerated then accelerated by
 * gravity), LANDS on the terrain (briefly stunned), then LOADS
 * again. Three discrete phases. The terrain is just a height
 * function that changes where the robot lands.
 *
 * THE JUMP CYCLE — three phases visualised
 * ────────────────────────────────────────
 *
 *     COMPRESS               FLIGHT                  LAND
 *     ──────────────         ─────────────────       ────────────
 *
 *      ┌───┐                       O                  ┌───┐
 *      │@  │                      ╱ ╲                 │ * │
 *      │|  │                    ╱     ╲               │   │
 *      │)  │                  ╱         ╲             │   │
 *      │|  │                ╱             ╲           │   │
 *      │v  │              ╱                 ↘         │   │
 *      └───┘            ●                     ●       └───┘
 *      foot                launch              land
 *      planted             instant             impact
 *
 *      • spring_compress     • vx constant         • body locked
 *        ramps 0 → MAX       • vy decelerates      • duration
 *      • PE = ½kx²             then accelerates      T_LAND
 *      • duration              under GRAVITY        • flash '*'
 *        T_COMPRESS          • trail recorded      • → COMPRESS
 *      • PE bar fills        • detect floor
 *      • → FLIGHT              contact → LAND
 *
 *
 * HOOKE'S LAW — first principles
 * ──────────────────────────────
 *
 *   A spring resists being compressed. The harder you push, the
 *   harder it pushes back:
 *
 *       F = -k · x         (force is opposite to displacement,
 *                           proportional to displacement)
 *
 *   The work you do compressing the spring is stored as potential
 *   energy. Integrating force from 0 to x gives the QUADRATIC
 *   potential:
 *
 *       PE = ½ · k · x²
 *
 *       ────────────────────────────────────────────────
 *       Compress 2× as far → store 4× as much energy.
 *       That's why the energy gauge fills SLOWLY at first
 *       and ACCELERATES near the end of compression.
 *       ────────────────────────────────────────────────
 *
 *   When the spring releases, all that PE becomes kinetic energy:
 *
 *       ½ · m · v² = ½ · k · x²
 *       ⟹  v = x · √(k / m)
 *
 *   With our defaults (m = 1, k = SPRING_K = 25, x_max = 48 px):
 *
 *       v_launch = 48 · √25 = 240 px/sec
 *
 *   That's the body's launch speed. We split it into vx and vy by
 *   the launch angle θ:
 *
 *       vx = v_launch · cos θ
 *       vy = -v_launch · sin θ        (negative = upward in y-down)
 *
 *
 * PROJECTILE MOTION — first principles
 * ────────────────────────────────────
 *
 *   Once airborne, only gravity acts on the robot. Two equations:
 *
 *       horizontal:    x(t) = x₀ + vx · t          (vx constant)
 *       vertical:      y(t) = y₀ + vy · t + ½ · g · t²
 *
 *   The trajectory is a PARABOLA opening downward. Useful numbers:
 *
 *       time to peak     :  t_peak = |vy| / g
 *       peak height      :  h_peak = vy² / (2g)
 *       time to land     :  t_land = 2 · t_peak              (flat ground)
 *       horizontal range :  R      = 2 · vx · |vy| / g       (flat ground)
 *
 *   Worked example (default config, flat ground):
 *
 *       v_launch = 240 px/sec, θ = 50°, g = 200 px/sec²
 *       vx       = 240 · cos50° ≈ 154 px/sec
 *       vy       = 240 · sin50° ≈ 184 px/sec
 *       t_peak   = 184 / 200 = 0.92 sec
 *       h_peak   = 184² / (2·200) ≈ 85 px ≈ 5 cells above launch
 *       t_land   = 1.84 sec
 *       R        = 2 · 154 · 184 / 200 ≈ 283 px ≈ 35 cols
 *
 *   So one default jump covers about 35 columns and reaches about
 *   5 cells above the launch height. On an 80-col terminal that's
 *   2-3 jumps before the camera has to pan. Time it with a
 *   stopwatch — the math is verifiable.
 *
 *
 * STATE MACHINE
 * ─────────────
 *
 *      ┌──────────┐  phase_t ≥ T_COMPRESS    ┌────────┐
 *      │ COMPRESS │ ───────────────────────► │ FLIGHT │
 *      │  (LOAD)  │   compute v_launch       │  (FLY) │
 *      │          │   set (vx, vy)            │        │
 *      │   '@'    │                          │   'O'  │
 *      └──────────┘                          └────────┘
 *           ▲                                    │
 *           │ phase_t ≥ T_LAND               foot reaches floor
 *           │                                set vx=vy=0; lock pose
 *           │                                    │
 *           │                                    ▼
 *      ┌──────────┐                         ┌────────┐
 *      │ COMPRESS │ ◄────────────────────── │  LAND  │
 *      └──────────┘                         │  '*'   │
 *                                           └────────┘
 *
 *
 * CAMERA — RIGHT-EDGE FOLLOW
 * ──────────────────────────
 *
 *   While the robot is in the LEFT 80 % of the screen, the camera
 *   doesn't move. Once the robot crosses CAM_TRIGGER × screen
 *   width, the camera target jumps to (body_px − trigger), and the
 *   actual cam_x exponentially chases that target at CAM_SPEED.
 *   This produces a smooth slide-cam (no snap) that pins the robot
 *   at the trigger column once it's "running with the camera".
 *
 *      ┌────────────────────────────────────┐
 *      │                                    │
 *      │  →  →  →  →  →  R                  │
 *      │                ↑                   │
 *      │           80 % trigger             │
 *      │                                    │
 *      └────────────────────────────────────┘
 *      cam_x = 0 (frozen)         past trigger:
 *                                 cam_x → body_px - trigger
 *                                 (exponential chase)
 *
 *   World coordinates: every robot/terrain position is in PIXEL
 *   space and ABSOLUTE (world coords), so old terrain at world_x =
 *   1000 still hashes to the same noise value when you scroll back.
 *   The camera is just a screen offset:  screen_x = world_x − cam_x.
 *
 *
 * SLOPE-ADAPTED LAUNCH ANGLE
 * ──────────────────────────
 *
 *   On flat ground we launch at LAUNCH_ANGLE (default 50°). On a
 *   slope, the robot tilts the angle to better suit the terrain:
 *
 *       slope < 0  (ascending)  → θ increases (steeper jump)
 *       slope > 0  (descending) → θ decreases (shallower jump)
 *       slope = 0  (flat)       → θ = LAUNCH_ANGLE
 *
 *       θ_eff = clamp(LAUNCH_ANGLE - slope · SLOPE_SCALE,
 *                     LAUNCH_MIN, LAUNCH_MAX)
 *
 *   Why? On an ascending slope, a flat-style jump might smack the
 *   wall in front. Steepening the jump clears the rise. On a
 *   descending slope, a flat jump overshoots; a shallower angle
 *   rides the descent.
 *
 *
 * KEY FORMULAS
 * ────────────
 *   Spring stored energy  : PE = ½ · SPRING_K · x²
 *   Spring launch speed   : v  = x · √(SPRING_K / m)     (m = 1)
 *   Launch decomposition  : vx =  v · cos θ
 *                           vy = -v · sin θ              (y-down: up = negative)
 *   Free fall vertical    : y(t) = y₀ + vy·t + ½·g·t²
 *   Free fall horizontal  : x(t) = x₀ + vx·t
 *   Slope (central diff)  : s = atan2(Δy, 2·Δx)          at foot position
 *   Effective angle       : θ_eff = clamp(LAUNCH_ANGLE − s · SLOPE_SCALE,
 *                                          LAUNCH_MIN, LAUNCH_MAX)
 *   Floor contact test    : foot_proj_y ≥ floor_y_at(body_x)
 *
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Forward-Euler integration. We use plain `vy += g·dt; py += vy·dt`
 *    rather than the more accurate `py += (v0 + v1)/2 · dt`. At 60 fps
 *    the worst-case position error per step is ½·g·dt² = 0.027 px —
 *    invisible. For higher dt or stiffer physics we'd switch to RK4
 *    or Verlet, but here Euler is fine.
 *
 *  • Slope finite difference uses central difference (`floor(x+δ) -
 *    floor(x-δ)`) so it's unbiased and second-order accurate. Don't
 *    accidentally use forward difference — it lags the actual slope.
 *
 *  • `phase_t` accumulates in seconds since the last phase transition.
 *    Reset it to 0 every time you switch phases. A leftover phase_t
 *    from the previous phase causes immediate re-trigger.
 *
 *  • Camera target. We clamp `cam_x ≥ 0` so we don't scroll left of
 *    world origin. The robot can never lose the camera leftward.
 *
 *  • PERLIN mode hashes world_x → height. If you reset the seed
 *    (key 'n') the existing terrain you can already see CHANGES,
 *    because future calls hash the same world_x to a different value.
 *    By design.
 *
 *  • Resize. SIGWINCH triggers screen_resize and rebases base_y so
 *    terrain stays at ~72 % of the new height. Trail and pose are
 *    preserved.
 *
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' to reset. Robot spawns near left edge in COMPRESS.
 *    Energy gauge fills quadratically (slow at first, fast at end).
 *
 *  • Stopwatch the COMPRESS phase: should be exactly T_COMPRESS = 0.45 sec.
 *
 *  • At launch, peak height should be ≈ 5 cells above launch (default).
 *    Visually: head reaches about row mid_screen − 5 at apex.
 *
 *  • Range per jump on flat ground: ≈ 35 columns at 1.0× speed.
 *
 *  • Press 'a' to cycle speeds. At 3.0× the range becomes ≈ 105
 *    columns — robot easily leaves the screen each jump and the
 *    camera pans every cycle.
 *
 *  • Switch to PERLIN ('f'): launch angle now visibly adapts to
 *    terrain. Notice the slope readout in the HUD.
 *
 *  • Press 'p' (pause) at apex. The robot freezes mid-arc — verify
 *    that the trail trail is a clean parabola.
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

/* ── §1.2 cell pixel dimensions ──────────────────────────────────── */
/*
 * All physics is in PIXEL SPACE — a uniform grid where 1 px is
 * 1/8 cell wide and 1/16 cell tall. Conversion to cells happens
 * only at draw time. Keeps the math isotropic regardless of
 * the terminal's 2:1 cell aspect ratio.
 */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 spring (Hooke's law) ────────────────────────────────────── */
/*
 * SPRING_REST           — uncompressed leg length in pixels.
 *                         At rest the body sits this far above the foot.
 * SPRING_COMPRESS_MAX   — maximum compression in pixels.
 *                         At full compression the body has sunk this much.
 * SPRING_K              — spring stiffness coefficient.
 *                         Determines launch speed: v = x · √(k/m), m=1.
 *
 *     v_launch = SPRING_COMPRESS_MAX · √SPRING_K
 *              = 48 · √25 = 48 · 5 = 240 px/sec
 *
 * BODY_HALF_H           — pixel half-height of body glyph for ground contact.
 */
#define SPRING_REST          80.0f
#define SPRING_COMPRESS_MAX  48.0f
#define SPRING_K             25.0f
#define BODY_HALF_H           8.0f

/* ── §1.4 launch geometry ────────────────────────────────────────── */
/*
 * LAUNCH_ANGLE — base launch angle from horizontal in radians.
 *   50° gives a near-balanced jump: vx ≈ 154, vy ≈ 184 at v=240.
 *   That balances arc height (≈5 cells) against horizontal range
 *   (≈35 cols), so 2–3 jumps fill an 80-col screen.
 *
 * SLOPE_SCALE — how much terrain slope deflects the launch angle.
 *   0.5 → a 20° slope shifts the launch by 10°.
 *   Higher = more responsive to slope; lower = ignore slope.
 *
 * LAUNCH_MIN, LAUNCH_MAX — clamp the slope-adapted angle to a sane
 *   range so degenerate slopes don't produce vertical or horizontal
 *   launches.
 */
#define LAUNCH_ANGLE  (50.0f * (float)M_PI / 180.0f)
#define SLOPE_SCALE    0.5f
#define LAUNCH_MIN    (25.0f * (float)M_PI / 180.0f)
#define LAUNCH_MAX    (82.0f * (float)M_PI / 180.0f)

/* ── §1.5 gravity ─────────────────────────────────────────────────── */
/*
 * Downward acceleration in pixels per second². 200 px/sec² gives
 * a peak height of ~85 px (≈5 cells) at default launch — visually
 * obvious without dwarfing the screen.
 */
#define GRAVITY  200.0f

/* ── §1.6 phase timing (seconds) ──────────────────────────────────── */
/*
 * T_COMPRESS — duration of the spring-loading phase. The energy
 *   gauge fills over this time. 0.45 s feels deliberate: long enough
 *   to read but not boring.
 *
 * T_LAND — duration of the impact flash. 0.14 s is short — just
 *   long enough to register the '*' visually.
 */
#define T_COMPRESS  0.45f
#define T_LAND      0.14f

/* ── §1.7 terrain ─────────────────────────────────────────────────── */
/*
 * Two-octave value noise — sum of one BIG slow wave and one SMALL
 * fast wave. The big wave gives rolling hills; the small wave adds
 * roughness on top. Three octaves works too but is overkill for
 * this teaching demo.
 *
 *   T_AMP    : amplitude (peak-to-peak / 2) in pixels.
 *              112 px = 7 cells of vertical swing.
 *
 *   T_FREQ_BIG : frequency of the big wave in cycles per pixel.
 *              0.0028 → wavelength ≈ 360 px ≈ 45 cells.
 *
 *   T_FREQ_SMALL : frequency of the small bumps.
 *              0.014 → wavelength ≈ 70 px ≈ 9 cells.
 *
 *   The two amplitudes (60 % big + 40 % small = 1.0) sum to 1.0 so
 *   the combined output is in [0, 1] just like a single noise call.
 */
#define T_AMP            112.0f
#define T_FREQ_BIG       0.0028f
#define T_FREQ_SMALL     0.014f
#define T_WEIGHT_BIG     0.6f
#define T_WEIGHT_SMALL   0.4f

#define NOISE_N          512    /* lattice size; power of 2 for fast & */

/* ── §1.8 camera ──────────────────────────────────────────────────── */
/*
 * CAM_TRIGGER — robot screen fraction at which camera starts to pan.
 *   0.80 → robot must walk across 80 % of screen before camera moves.
 *
 * CAM_SPEED — exponential catch-up rate when triggered, per second.
 *   6.0 → time constant 1/6 ≈ 167 ms. Smooth slide, no snap.
 *
 * CAM_START_COL — column where the robot spawns at reset.
 *   3 cells from left edge. Gives the robot a clear head start.
 */
#define CAM_TRIGGER     0.80f
#define CAM_SPEED       6.0f
#define CAM_START_COL   3

/* ── §1.9 trail ───────────────────────────────────────────────────── */
/* Ring buffer of recent body positions; older entries fade visually. */
enum { TRAIL_CAP = 1500 };

/* ── §1.10 horizontal speed cycle ('a' key) ───────────────────────── */
/*
 * Multiplier on vx at launch. ONLY scales horizontal motion — arc
 * height stays the same so the jump shape stays readable while the
 * robot covers more ground.
 */
enum { SPEED_LEVELS = 5 };
static const float SPEED_MULTS[SPEED_LEVELS] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };

/* ── §1.11 dt cap (spiral-of-death guard) ─────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.12 ncurses pair IDs ───────────────────────────────────────── */
enum {
    /* 1..6 — robot & spring */
    CP_BODY      = 1,        /* '@' grounded                 white       */
    CP_SPRING_LO,            /* spring low compression       yellow      */
    CP_SPRING_MD,            /* spring medium                orange      */
    CP_SPRING_HI,            /* spring high (loaded)         red         */
    CP_FLIGHT,               /* 'O' airborne                 cyan        */
    CP_LAND,                 /* '*' impact                   magenta     */

    /* 7..8 — trail */
    CP_TRAIL,                /* fresh trail '.'              blue        */
    CP_TRAIL_OLD,            /* aged trail ':'               dim blue    */

    /* 9..11 — terrain & PE */
    CP_SURF,                 /* terrain surface              green       */
    CP_ROCK,                 /* terrain sub-surface          dim green   */
    CP_PE,                   /* PE bar fill                  yellow      */

    /* 12..13 — HUD spec, theme-independent */
    PAIR_HUD,                /* row 0 status                 yellow BOLD */
    PAIR_HINT,               /* bottom row hint              cyan BOLD   */
};

/* ── §1.13 timing primitives ──────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.14 HUD layout ─────────────────────────────────────────────── */
#define HUD_BUF_LEN  192

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
 * Palette is fixed (no themes) because this is a teaching demo and
 * one consistent set of semantic colours is easier to memorise:
 *
 *   body              white     (always visible)
 *   spring low        yellow    (just loaded)
 *   spring medium     orange    (mid-load)
 *   spring high       red       (fully loaded — about to fire)
 *   flight body 'O'   cyan      (airborne)
 *   land flash '*'    magenta   (impact)
 *   trail fresh       blue      (recent arc)
 *   trail old         dim blue  (older arc, ':' for visual recede)
 *   terrain surface   green     ('_' '/' '\')
 *   terrain rock      dim green ('#' fill)
 *   PE bar            yellow    (turns red when nearly full)
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) bind separately on default
 * terminal background per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,        255, COLOR_BLACK);    /* near-white   */
        init_pair(CP_SPRING_LO,   226, COLOR_BLACK);    /* yellow       */
        init_pair(CP_SPRING_MD,   208, COLOR_BLACK);    /* orange       */
        init_pair(CP_SPRING_HI,   196, COLOR_BLACK);    /* red          */
        init_pair(CP_FLIGHT,       51, COLOR_BLACK);    /* cyan         */
        init_pair(CP_LAND,        201, COLOR_BLACK);    /* magenta      */
        init_pair(CP_TRAIL,        27, COLOR_BLACK);    /* blue         */
        init_pair(CP_TRAIL_OLD,    25, COLOR_BLACK);    /* dim blue     */
        init_pair(CP_SURF,         46, COLOR_BLACK);    /* bright green */
        init_pair(CP_ROCK,         28, COLOR_BLACK);    /* dim green    */
        init_pair(CP_PE,          226, COLOR_BLACK);    /* yellow       */
        init_pair(PAIR_HUD,       226, -1);             /* yellow on bg */
        init_pair(PAIR_HINT,       51, -1);             /* cyan on bg   */
    } else {
        init_pair(CP_BODY,      COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_SPRING_LO, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_SPRING_MD, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_SPRING_HI, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_FLIGHT,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_LAND,      COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_TRAIL,     COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_SURF,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_ROCK,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_PE,        COLOR_YELLOW,  COLOR_BLACK);
        init_pair(PAIR_HUD,     COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,    COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Physics works in PIXEL SPACE; conversion to cell coordinates
 * happens only at draw time. `pw`/`ph` give world dimensions in
 * pixels. `px_to_cx`/`px_to_cy` round to the nearest cell.
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
/* §5  noise — 1-D value noise for terrain                                */
/* ===================================================================== */

/*
 * Value noise: a table of NOISE_N random values in [0, 1] is hashed
 * by integer position; values between integer points are
 * interpolated. We use COSINE interpolation so the resulting curve
 * is C¹-continuous (smooth, no kinks at lattice points):
 *
 *     f(t) = a · (1 - smooth(t)) + b · smooth(t)
 *     where  smooth(t) = (1 - cos(π·t)) / 2
 *
 * The resulting noise(x) ∈ [0, 1] for all real x.
 */
static float g_noise[NOISE_N];

static void noise_init(unsigned seed)
{
    srand(seed);
    for (int i = 0; i < NOISE_N; i++)
        g_noise[i] = (float)rand() / (float)RAND_MAX;
}

static float noise1d(float x)
{
    int   xi = (int)floorf(x) & (NOISE_N - 1);
    float xf = x - floorf(x);
    float a  = g_noise[xi];
    float b  = g_noise[(xi + 1) & (NOISE_N - 1)];
    float t  = (1.0f - cosf(xf * (float)M_PI)) * 0.5f;
    return a * (1.0f - t) + b * t;
}

/*
 * Two-octave fBm — sum of one big slow wave and one small fast wave.
 * Result is in [0, 1] because the weights sum to 1.
 */
static float terrain_noise(float world_x)
{
    return noise1d(world_x * T_FREQ_BIG  ) * T_WEIGHT_BIG
         + noise1d(world_x * T_FREQ_SMALL) * T_WEIGHT_SMALL;
}

/* ===================================================================== */
/* §6  terrain — height query, slope, surface glyph                       */
/* ===================================================================== */

typedef enum { FLOOR_FLAT = 0, FLOOR_PERLIN, FLOOR_COUNT } FloorMode;
static const char *FLOOR_NAMES[FLOOR_COUNT] = { "FLAT  ", "PERLIN" };

/*
 * floor_y_at — world y of the terrain surface at world x.
 *
 *   FLAT    : returns base_y unchanged.
 *   PERLIN  : centres noise at 0.5, scales to ±T_AMP around base_y.
 */
static float floor_y_at(float world_x, FloorMode mode, float base_y)
{
    if (mode == FLOOR_FLAT) return base_y;
    return base_y + (terrain_noise(world_x) - 0.5f) * 2.0f * T_AMP;
}

/*
 * floor_slope — surface slope at world_x (radians).
 *
 *   slope = atan2(Δy, Δx)
 *
 * with Δy obtained by central difference (sampling 2·CELL_W to
 * each side of the foot). Central difference is unbiased and
 * second-order accurate — the right choice for a smooth slope
 * estimate.
 *
 *   slope > 0  →  ground falls to the right  (descending)
 *   slope < 0  →  ground rises to the right  (ascending)
 */
static float floor_slope(float world_x, FloorMode mode, float base_y)
{
    float dx = (float)(CELL_W * 2);
    float dy = floor_y_at(world_x + dx, mode, base_y)
             - floor_y_at(world_x - dx, mode, base_y);
    return atan2f(dy, 2.0f * dx);
}

/*
 * surface_glyph — char to draw at the surface row, based on the
 * height delta to the next column. Rough approximation of slope:
 *
 *   delta < -threshold  →  '/'   ascending right
 *   delta >  threshold  →  '\\'  descending right
 *   else                →  '_'   flat
 */
static chtype surface_glyph(float dy)
{
    if (dy < -CELL_H * 0.20f) return '/';
    if (dy >  CELL_H * 0.20f) return '\\';
    return '_';
}

/* ===================================================================== */
/* §7  trail — ring buffer of recent body positions                       */
/* ===================================================================== */

/*
 * Stores the last TRAIL_CAP world positions. Newest at index
 * `(head − 1 + TRAIL_CAP) mod TRAIL_CAP`; oldest at `head` itself
 * once the buffer is full.
 */
typedef struct {
    float wx[TRAIL_CAP];
    float wy[TRAIL_CAP];
    int   head;
    int   count;
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = 0;
}

static void trail_push(Trail *t, float wx, float wy)
{
    t->wx[t->head] = wx;
    t->wy[t->head] = wy;
    t->head        = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §8  robot — Phase, Robot, spring physics, per-state ticks              */
/* ===================================================================== */

/* ── §8.1 Phase + Robot type ──────────────────────────────────────── */

typedef enum {
    PHASE_COMPRESS = 0,    /* spring loading, body sinking      */
    PHASE_FLIGHT,          /* projectile arc                    */
    PHASE_LAND,            /* impact flash, body locked         */
} Phase;

static const char *PHASE_NAMES[] = { "LOAD", "FLY ", "LAND" };

/*
 * Robot — full simulation state.
 *
 *   Pose:
 *     body_px, body_py     centre of body in pixel space
 *     foot_px, foot_py     ground contact point (fixed during
 *                          COMPRESS / LAND, set on landing)
 *     vx, vy               velocity in pixel/sec (only meaningful
 *                          during FLIGHT)
 *
 *   Spring:
 *     spring_compress      current compression in pixels
 *                          (0 at rest, SPRING_COMPRESS_MAX at full)
 *
 *   Phase machinery:
 *     phase                COMPRESS / FLIGHT / LAND
 *     phase_t              seconds since current phase began
 *
 *   Camera:
 *     cam_x                world x of left screen edge
 *
 *   Cached terrain values (computed during COMPRESS):
 *     slope_angle          atan slope at current foot position
 *     eff_angle            slope-adapted launch angle
 *
 *   World:
 *     base_y               flat-floor y; perlin terrain centred here
 *     floor_mode           FLAT or PERLIN
 *
 *   UI:
 *     speed_level          index into SPEED_MULTS, cycled by 'a'
 *     launch_count         how many jumps so far (HUD readout)
 *     paused               if true, robot_tick returns early
 *
 *   Trail (ring buffer of past positions).
 */
typedef struct {
    float     body_px, body_py;
    float     foot_px, foot_py;
    float     vx, vy;
    float     spring_compress;

    Phase     phase;
    float     phase_t;

    float     cam_x;
    float     slope_angle;
    float     eff_angle;

    float     base_y;
    FloorMode floor_mode;

    int       speed_level;
    int       launch_count;
    bool      paused;

    Trail     trail;
} Robot;

/* ── §8.2 spring physics — Hooke's law & energy ──────────────────── */

/* Stored potential energy in the spring (relative units; m=1, k=K).
 * Used by the PE gauge HUD; not by the integrator. */
static inline float spring_energy(float compress)
{
    return 0.5f * SPRING_K * compress * compress;
}

/* Body height above foot for a given compression. */
static inline float leg_length(float compress)
{
    return SPRING_REST - compress;
}

/*
 * pose_from_spring — recompute body position from foot + compression.
 *
 *   body_py = foot_py − leg_length − BODY_HALF_H
 *
 * Called during COMPRESS and LAND, when the robot is grounded and
 * the body's vertical position is determined entirely by spring
 * state. (During FLIGHT body_py is integrated separately.)
 */
static void pose_from_spring(Robot *r)
{
    r->body_px = r->foot_px;
    r->body_py = r->foot_py - leg_length(r->spring_compress) - BODY_HALF_H;
}

/* ── §8.3 effective launch angle (slope-adapted) ─────────────────── */

/*
 * Slope > 0 (descending) → reduce angle (shallower jump, ride down).
 * Slope < 0 (ascending)  → increase angle (steeper jump, clear rise).
 * Clamp to [LAUNCH_MIN, LAUNCH_MAX] so degenerate slopes can't make
 * the launch nearly vertical or nearly horizontal.
 */
static float effective_launch_angle(float slope)
{
    return clampf(LAUNCH_ANGLE - slope * SLOPE_SCALE,
                  LAUNCH_MIN, LAUNCH_MAX);
}

/* ── §8.4 cam_update — right-edge follow camera ──────────────────── */

/*
 * If the robot's screen position has crossed CAM_TRIGGER × screen
 * width, the camera target is set to (body_px − trigger), which
 * keeps the robot pinned at the trigger column. cam_x exponentially
 * chases that target at CAM_SPEED — so the camera slides smoothly
 * toward it instead of snapping.
 *
 * cam_x is clamped to ≥ 0 so we never scroll left of world origin.
 */
static void cam_update(Robot *r, float dt, int cols)
{
    float scr_w   = (float)pw(cols);
    float bot_sx  = r->body_px - r->cam_x;          /* screen x of robot */
    float trigger = scr_w * CAM_TRIGGER;

    if (bot_sx > trigger) {
        float target  = r->body_px - trigger;
        r->cam_x     += (target - r->cam_x) * CAM_SPEED * dt;
    }

    if (r->cam_x < 0.0f) r->cam_x = 0.0f;
}

/* ── §8.5 compress_tick — load the spring, then launch ───────────── */

/*
 * The body sinks toward the foot as `spring_compress` ramps linearly
 * from 0 to SPRING_COMPRESS_MAX over T_COMPRESS seconds. We sample
 * the terrain slope at the foot to compute the slope-adapted launch
 * angle (used at the moment of release).
 *
 * On reaching full compression, convert stored spring energy into
 * launch velocity using Hooke's-law energy conservation:
 *
 *     v = compress · √(SPRING_K / m)
 *     (m = 1, so v = compress · √K = 48 · 5 = 240 px/sec at full)
 *
 * Decompose v into (vx, vy) using the effective launch angle and the
 * horizontal speed multiplier. Clear compress and transition to FLIGHT.
 */
static void compress_tick(Robot *r)
{
    float prog = clampf(r->phase_t / T_COMPRESS, 0.0f, 1.0f);
    r->spring_compress = SPRING_COMPRESS_MAX * prog;
    pose_from_spring(r);

    /* Sample slope continuously during loading so launch sees the
     * latest terrain (in case the foot has moved — it hasn't here,
     * but defensive code is cheap). */
    r->slope_angle = floor_slope(r->foot_px, r->floor_mode, r->base_y);
    r->eff_angle   = effective_launch_angle(r->slope_angle);

    if (r->phase_t >= T_COMPRESS) {
        float v_launch = r->spring_compress * sqrtf(SPRING_K);
        float vx_mult  = SPEED_MULTS[r->speed_level];

        r->vx =  v_launch * cosf(r->eff_angle) * vx_mult;
        r->vy = -v_launch * sinf(r->eff_angle);   /* y-down: up = negative */

        r->spring_compress = 0.0f;                /* leg now fully extended */
        r->phase           = PHASE_FLIGHT;
        r->phase_t         = 0.0f;
        r->launch_count++;
    }
}

/* ── §8.6 flight_tick — projectile motion, detect landing ────────── */

/*
 * Forward-Euler integration of horizontal and vertical motion under
 * uniform gravity. Push every flight position into the trail buffer
 * so the parabolic arc is visible. Test for floor contact:
 *
 *     foot_proj_y = body_py + BODY_HALF_H + SPRING_REST
 *
 * is the y the foot WOULD have if the leg were fully extended. When
 * that exceeds the terrain height under the body, the foot has
 * "punched into" the ground — time to land. Snap the foot to the
 * surface, zero the velocity, and transition to LAND.
 */
static void flight_tick(Robot *r, float dt)
{
    /* Newtonian projectile motion. */
    r->vy      += GRAVITY * dt;
    r->body_px += r->vx * dt;
    r->body_py += r->vy * dt;

    trail_push(&r->trail, r->body_px, r->body_py);

    /* Floor contact test. */
    float foot_proj  = r->body_py + BODY_HALF_H + SPRING_REST;
    float floor_here = floor_y_at(r->body_px, r->floor_mode, r->base_y);
    if (foot_proj >= floor_here) {
        r->foot_px         = r->body_px;
        r->foot_py         = floor_here;
        r->spring_compress = 0.0f;
        pose_from_spring(r);
        r->vx = r->vy = 0.0f;

        r->phase   = PHASE_LAND;
        r->phase_t = 0.0f;
    }
}

/* ── §8.7 land_tick — brief stun, then back to COMPRESS ──────────── */

/*
 * The body is locked at the landing pose (set in flight_tick). We
 * just count down T_LAND. The visual difference is the body glyph
 * ('*' instead of '@') and the absence of the spring coil.
 */
static void land_tick(Robot *r)
{
    pose_from_spring(r);
    if (r->phase_t >= T_LAND) {
        r->phase   = PHASE_COMPRESS;
        r->phase_t = 0.0f;
    }
}

/* ── §8.8 robot_tick — orchestrator: dispatch on phase ──────────── */

/*
 * One frame. If paused, do nothing — but cam_update STILL runs so
 * the user can pan with input even while frozen (… though we don't
 * have a manual pan key; cam_update under pause is harmless because
 * body_px doesn't change).
 *
 * phase_t is accumulated centrally so each per-state tick can read
 * it without re-implementing the bookkeeping.
 */
static void robot_tick(Robot *r, float dt, int cols)
{
    if (r->paused) return;

    cam_update(r, dt, cols);
    r->phase_t += dt;

    switch (r->phase) {
    case PHASE_COMPRESS: compress_tick(r);     break;
    case PHASE_FLIGHT:   flight_tick(r, dt);   break;
    case PHASE_LAND:     land_tick(r);         break;
    }
}

/* ── §8.9 robot_init / robot_reset ────────────────────────────────── */

static void robot_init(Robot *r, int rows)
{
    /* Preserve user-tunable settings across init. */
    FloorMode fm  = r->floor_mode;
    int       sl  = r->speed_level;

    memset(r, 0, sizeof *r);
    r->floor_mode  = fm;
    r->speed_level = sl;

    /* Terrain baseline at 72 % of screen height — leaves room above
     * for arcs and energy gauge, room below for terrain-fill. */
    r->base_y      = (float)ph(rows) * 0.72f;

    /* Spawn near the left edge, planted on the surface. */
    r->foot_px     = (float)(CAM_START_COL * CELL_W);
    r->foot_py     = floor_y_at(r->foot_px, r->floor_mode, r->base_y);
    r->slope_angle = floor_slope(r->foot_px, r->floor_mode, r->base_y);
    r->eff_angle   = effective_launch_angle(r->slope_angle);

    r->phase       = PHASE_COMPRESS;
    r->phase_t     = 0.0f;
    r->cam_x       = 0.0f;

    pose_from_spring(r);
}

/* Light reset for the 'r' key — preserves theme/speed/floor mode. */
static void robot_reset(Robot *r, int rows)
{
    trail_clear(&r->trail);
    robot_init(r, rows);
}

/* ===================================================================== */
/* §9  render                                                             */
/* ===================================================================== */

/* ── §9.1 helpers ────────────────────────────────────────────────── */

/* Convert world x to screen column via camera offset. */
static inline int scr_cx(float world_px, float cam_x)
{
    return px_to_cx(world_px - cam_x);
}

/*
 * Top row 0 reserved for HUD; bottom row reserved for hint strip.
 * Drawables must live in rows 1..rows-2.
 */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* ── §9.2 render_terrain — per-column sweep ──────────────────────── */

/*
 * For every screen column, compute the terrain row from the noise
 * function (camera offset converts screen column → world x), then
 * paint:
 *
 *   row surf       surface glyph   bright green
 *   row surf+1     texture pattern dim green   (alternating . and :)
 *   row surf+2..   solid fill      dark green  (alternating # and ' ')
 *
 * The texture row gives the surface a fuzzy/grassy edge; the solid
 * fill gives weight to the terrain mass. Both use simple `sc % 2`
 * checkerboards so the pattern is stable regardless of camera scroll.
 */
static void render_terrain(FloorMode mode, float base_y, float cam_x,
                           int cols, int rows)
{
    for (int sc = 0; sc < cols; sc++) {
        float wx      = cam_x + (float)(sc * CELL_W);
        float fy      = floor_y_at(wx, mode, base_y);
        float fy_next = floor_y_at(wx + (float)CELL_W, mode, base_y);
        int   surf    = px_to_cy(fy);
        if (surf < 1)        surf = 1;
        if (surf > rows - 2) surf = rows - 2;

        chtype sg = (mode == FLOOR_PERLIN)
                  ? surface_glyph(fy_next - fy)
                  : '_';

        attron(COLOR_PAIR(CP_SURF) | A_BOLD);
        mvaddch(surf, sc, sg);
        attroff(COLOR_PAIR(CP_SURF) | A_BOLD);

        if (surf + 1 < rows - 1) {
            attron(COLOR_PAIR(CP_SURF) | A_DIM);
            mvaddch(surf + 1, sc, (sc % 2 == 0) ? ':' : '.');
            attroff(COLOR_PAIR(CP_SURF) | A_DIM);
        }

        attron(COLOR_PAIR(CP_ROCK) | A_DIM);
        for (int r = surf + 2; r < rows - 1; r++)
            mvaddch(r, sc, (sc % 2 == 0) ? '#' : ' ');
        attroff(COLOR_PAIR(CP_ROCK) | A_DIM);
    }
}

/* ── §9.3 render_trail — newest brightest, fading leftward ───────── */

/*
 * Iterate newest-first so the freshest dots paint last (top of the
 * stack) and old ones paint first (under any overlaps).
 *
 *   age in [0, 1]  where 0 = newest, 1 = oldest
 *     age < 0.15 → BOLD '.' fresh blue
 *     age < 0.40 → '.' blue
 *     age ≥ 0.40 → DIM ':' dim blue   (':' for visual recede)
 */
static void render_trail(const Robot *r, float cam_x, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int cx  = scr_cx     (r->trail.wx[idx], cam_x);
        int cy  = px_to_cy   (r->trail.wy[idx]);
        if (!in_bounds(cx, cy, cols, rows)) continue;

        float  age = (r->trail.count > 1)
                   ? (float)k / (float)(r->trail.count - 1) : 0.0f;
        int    cp  = (age < 0.40f) ? CP_TRAIL : CP_TRAIL_OLD;
        attr_t at  = (age < 0.15f) ? A_BOLD
                   : (age < 0.40f) ? A_NORMAL : A_DIM;
        chtype ch  = (age < 0.40f) ? '.' : ':';

        attron(COLOR_PAIR(cp) | at);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/* ── §9.4 render_spring — coil leg between body bottom and foot ──── */

/*
 * Draw a coil from the body bottom row down to the foot row. The
 * coil pattern { '(', '|', ')', '|' } cycles per row; visually the
 * coil "shrinks" as the body descends (fewer rows between body and
 * foot during full compression).
 *
 * Color shifts yellow → orange → red as compression rises, giving
 * an immediate visual cue of how loaded the spring is.
 */
static void render_spring(const Robot *r, float cam_x, int cols, int rows)
{
    int cx       = scr_cx(r->body_px, cam_x);
    int body_bot = px_to_cy(r->body_py + BODY_HALF_H);
    int foot_cy  = px_to_cy(r->foot_py);

    if (cx < 0 || cx >= cols) return;

    float ratio = r->spring_compress / SPRING_COMPRESS_MAX;
    int    cp   = (ratio < 0.30f) ? CP_SPRING_LO
                : (ratio < 0.70f) ? CP_SPRING_MD : CP_SPRING_HI;
    attr_t at   = (ratio > 0.70f) ? A_BOLD : A_NORMAL;

    static const chtype coil[4] = { '(', '|', ')', '|' };
    for (int row = body_bot; row < foot_cy; row++) {
        if (row < 1 || row >= rows - 1) continue;
        attron(COLOR_PAIR(cp) | at);
        mvaddch(row, cx, coil[(row - body_bot) & 3]);
        attroff(COLOR_PAIR(cp) | at);
    }

    /* Foot marker. */
    if (foot_cy >= 1 && foot_cy < rows - 1) {
        attron(COLOR_PAIR(CP_ROCK));
        mvaddch(foot_cy, cx, 'v');
        attroff(COLOR_PAIR(CP_ROCK));
    }
}

/* ── §9.5 render_body — phase-coded glyph at body position ───────── */

/*
 *   PHASE_COMPRESS  →  '@'  white BOLD     (planted, loading)
 *   PHASE_FLIGHT    →  'O'  cyan BOLD      (airborne)
 *   PHASE_LAND      →  '*'  magenta BOLD   (impact flash)
 */
static void render_body(const Robot *r, float cam_x, int cols, int rows)
{
    int cx = scr_cx(r->body_px, cam_x);
    int cy = px_to_cy(r->body_py);
    if (!in_bounds(cx, cy, cols, rows)) return;

    chtype ch; int cp;
    switch (r->phase) {
    case PHASE_COMPRESS: ch = '@'; cp = CP_BODY;   break;
    case PHASE_FLIGHT:   ch = 'O'; cp = CP_FLIGHT; break;
    default:             ch = '*'; cp = CP_LAND;   break;
    }

    attron(COLOR_PAIR(cp) | A_BOLD);
    mvaddch(cy, cx, ch);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* ── §9.6 render_pe_bar — vertical PE gauge, COMPRESS only ────────── */

/*
 * Visualises stored spring energy. Height of fill ∝ PE / PE_max:
 *
 *     PE = ½kx²        →  PE / PE_max = (x / x_max)²
 *
 * So the bar fills SLOWLY at first and ACCELERATES near full
 * compression — a deliberate non-linearity that teaches the
 * quadratic energy law. When the bar tops 80 % we switch the fill
 * colour to red and the topmost cell to '!' for "about to fire".
 */
static void render_pe_bar(const Robot *r, int cols, int rows)
{
    if (r->phase != PHASE_COMPRESS) return;

    int bar_x   = cols - 3;
    int bar_bot = rows - 2;
    int bar_h   = (rows > 12) ? rows - 10 : 4;
    int bar_top = bar_bot - bar_h;
    if (bar_x < 0 || bar_top < 1) return;

    float ratio  = spring_energy(r->spring_compress)
                 / spring_energy(SPRING_COMPRESS_MAX);
    int   filled = (int)(ratio * (float)bar_h + 0.5f);

    /* Label "PE" above the bar. */
    if (bar_top - 1 >= 1) {
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(bar_top - 1, bar_x, "PE");
        attroff(COLOR_PAIR(PAIR_HUD));
    }

    for (int i = 0; i < bar_h; i++) {
        int row = bar_bot - i;
        if (row < 1 || row >= rows - 1) continue;
        if (i < filled) {
            int    fcp = (ratio > 0.80f) ? CP_SPRING_HI : CP_PE;
            chtype fc  = (ratio > 0.80f && i == filled - 1) ? '!' : '|';
            attron(COLOR_PAIR(fcp) | A_BOLD);
            mvaddch(row, bar_x,     fc);
            mvaddch(row, bar_x + 1, fc);
            attroff(COLOR_PAIR(fcp) | A_BOLD);
        } else {
            attron(COLOR_PAIR(PAIR_HUD));
            mvaddch(row, bar_x,     '.');
            mvaddch(row, bar_x + 1, '.');
            attroff(COLOR_PAIR(PAIR_HUD));
        }
    }
}

/* ── §9.7 render_hud — yellow status row 0 + cyan hint bottom row ── */

/*
 * Status row (row 0, PAIR_HUD, BOLD): floor mode, current phase,
 * compression / max compression, stored PE, slope angle, effective
 * launch angle, speed multiplier, robot screen column, total
 * camera scroll, jump count, fps.
 *
 * Hint strip (bottom row, PAIR_HINT, BOLD): full key list.
 *
 * The 'PAUSED' banner overlays the centre of the screen when the
 * simulation is frozen.
 */
static void render_hud(const Robot *r, double fps, int cols, int rows)
{
    int robot_scr = scr_cx(r->body_px, r->cam_x);

    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  [%s] %-4s  cmp:%3.0f/%3.0f  PE:%5.0f  "
             "slope:%+5.1f°  launch:%4.1f°  spd:%.1fx  col:%-3d  "
             "cam:%5.0f  jumps:%-3d  %s ",
             fps, FLOOR_NAMES[r->floor_mode], PHASE_NAMES[r->phase],
             r->spring_compress, SPRING_COMPRESS_MAX,
             spring_energy(r->spring_compress),
             r->slope_angle * (180.0f / (float)M_PI),
             r->eff_angle   * (180.0f / (float)M_PI),
             SPEED_MULTS[r->speed_level],
             robot_scr, r->cam_x, r->launch_count,
             r->paused ? "PAUSED" : "running");

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    /* Pad rest of row 0 with spaces so HUD bg is uniform. */
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  f:floor  n:new-terrain  "
             "a:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);

    if (r->paused) {
        int mx = cols / 2 - 4;
        if (mx >= 0) {
            attron(COLOR_PAIR(CP_LAND) | A_BOLD);
            mvprintw(rows / 2, mx, " PAUSED ");
            attroff(COLOR_PAIR(CP_LAND) | A_BOLD);
        }
    }
}

/* ── §9.8 scene_draw — paint one frame ───────────────────────────── */

/*
 * Painter's order — last write wins:
 *   (1) trail  (background; oldest first, newer overwrites older)
 *   (2) terrain
 *   (3) spring leg (only if grounded)
 *   (4) body glyph (always on top of leg)
 *   (5) PE bar (only during COMPRESS)
 *   (6) HUD on row 0 + hint strip on bottom row
 */
static void scene_draw(const Robot *r, double fps, int cols, int rows)
{
    erase();

    float cam_x = r->cam_x;

    render_trail   (r, cam_x, cols, rows);
    render_terrain (r->floor_mode, r->base_y, cam_x, cols, rows);

    if (r->phase == PHASE_COMPRESS || r->phase == PHASE_LAND)
        render_spring(r, cam_x, cols, rows);

    render_body    (r, cam_x, cols, rows);
    render_pe_bar  (r, cols, rows);
    render_hud     (r, fps, cols, rows);
}

/* ===================================================================== */
/* §10  screen — ncurses init / present                                   */
/* ===================================================================== */

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
/* §11  app — signals, resize, variable-dt main loop                      */
/* ===================================================================== */

typedef struct {
    Robot                 robot;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Map one keypress to an action. */
static void app_handle_key(App *app, int ch)
{
    Robot *r = &app->robot;
    switch (ch) {
    case 'q': case 'Q': case 27:
        app->running = 0;
        break;

    case ' ': case 'p': case 'P':
        r->paused = !r->paused;
        break;

    case 'r': case 'R':
        robot_reset(r, app->screen.rows);
        break;

    case 'f': case 'F':
        r->floor_mode = (FloorMode)((r->floor_mode + 1) % FLOOR_COUNT);
        break;

    case 'n': case 'N':
        noise_init((unsigned)clock_ns());
        break;

    case 'a': case 'A':
        r->speed_level = (r->speed_level + 1) % SPEED_LEVELS;
        break;

    default: break;
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    noise_init((unsigned)(clock_ns() & 0xFFFFFFFF));

    App *app     = &g_app;
    app->running = 1;
    app->robot.floor_mode = FLOOR_PERLIN;       /* default to interesting terrain */

    screen_init(&app->screen);
    robot_init (&app->robot, app->screen.rows);

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->robot.base_y = (float)ph(app->screen.rows) * 0.72f;
            app->need_resize  = 0;
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        int ch;
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        /* (4) advance physics */
        robot_tick(&app->robot, dt, app->screen.cols);

        /* (5) rolling fps display (0.5 s window) */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (6) draw + present */
        scene_draw(&app->robot, fps_display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
