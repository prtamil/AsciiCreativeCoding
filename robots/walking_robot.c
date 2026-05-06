/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * walking_robot.c — bipedal stick-figure robot walking forward and backward
 *
 * DEMO: A two-legged robot that walks across the screen like a human:
 *       one leg lifts and swings forward (SWING phase), plants the
 *       foot, then the body moves over that planted foot (STANCE
 *       phase) while the other leg does the same thing offset by
 *       half a cycle. Two legs, alternating, forever. Press 'r' to
 *       reverse direction; press '+'/'-' to change pace.
 *
 *       The figure is intentionally minimal: a stick figure with
 *       two arms and two legs, each rendered with simple ASCII
 *       line glyphs. The shape is recognisable as a person walking;
 *       the colours separate left (green) from right (magenta) so
 *       you can track each limb through its cycle.
 *
 *           ┌─────────────────────────────────────┐
 *           │                                     │
 *           │              O      ← head          │
 *           │              |                      │
 *           │             /|\     ← arms          │
 *           │              |      ← spine         │
 *           │              |                      │
 *           │             / \     ← hips fork     │
 *           │            /   \    ← thighs        │
 *           │           /     \   ← shins         │
 *           │          *       #  ← left foot     │
 *           │         (swing) (planted)           │
 *           │ ─────────────────────────────────   │
 *           │ ground                              │
 *           └─────────────────────────────────────┘
 *
 * Real-world analogues:
 *           Boston Dynamics Atlas (more joints, same idea)
 *           ASIMO and other research bipeds
 *           Any animation studio's biped rig
 *
 * Study alongside:
 *   animation/ik_helloworld.c       — the 2-link IK math (law of
 *                                     cosines) that this file uses
 *                                     for stance-leg posing.
 *   animation/fk_helloworld.c       — the FK math (angles → position)
 *                                     used for swing-leg posing.
 *   animation/fk_ik_helloworld.c    — both modes side by side; this
 *                                     walking file uses both ALTERNATELY
 *                                     (FK during swing, IK during stance).
 *   robots/diff_drive_robot.c       — wheeled contrast: rolling
 *                                     instead of stepping.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus body/left/right/feet pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  robot        — Robot type, gait math, FK swing, IK stance,
 *                      foot-lock, init/reset, tick (sub-sectioned)
 *   §6  render       — draw bones, ground, robot, HUD (sub-sectioned)
 *   §7  screen       — ncurses init / present
 *   §8  app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                reverse direction
 *   ↑ / +            speed up
 *   ↓ / -            slow down
 *   .                step one frame (while paused)
 *   g                toggle ground grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/walking_robot.c \
 *       -o walking_robot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Procedural sinusoidal gait with foot contact locking.
 *
 *                 (A) PHASE — a single float `phase` advances at
 *                     `2π · walk_freq · direction` rad/sec. Each leg
 *                     reads a phase offset by 0 (left) or π (right).
 *
 *                 (B) SWING — when sin(φ_leg) > 0, the leg is in the
 *                     air. Forward kinematics: pick thigh and knee
 *                     angles as smooth functions of φ_leg, then
 *                     compute knee and foot positions from the hip.
 *                          thigh_angle = -SWING_AMP · cos(φ)
 *                          knee_bend   =  LIFT_AMP · sin(φ)
 *                     The thigh sweeps from -SWING (back) at φ=0
 *                     through 0 at π/2 to +SWING (forward) at π.
 *                     The knee bend peaks at mid-swing (π/2) then
 *                     straightens for landing.
 *
 *                 (C) STANCE — when sin(φ_leg) ≤ 0, the foot is on
 *                     the ground at a LOCKED world position. As the
 *                     body advances, we use 2-link analytical IK
 *                     (law of cosines) to compute the knee.
 *
 *                 (D) FOOT LOCK — at the moment of touch-down (sin
 *                     transitions + → −), we record the foot's FK
 *                     landing position in `foot_lock[i]`. From that
 *                     instant until the next swing, foot_lock[i] is
 *                     where the foot stays.
 *
 *                 (E) BODY — the hip bobs vertically by sin(2φ)
 *                     (2× per stride; the body rises at toe-off
 *                     and dips at heel-strike) and sways laterally
 *                     by cos(φ) (1× per stride; the torso leans
 *                     over whichever foot is planted).
 *
 *                 (F) ARMS — single-segment arms swing as
 *                     ARM_SWING · sin(φ + π_offset). The offset is
 *                     the OPPOSITE of the leg on the same side, so
 *                     the left arm swings forward when the right
 *                     leg swings forward — this is how humans
 *                     balance angular momentum while walking.
 *
 * Data-structure: One Robot struct holds: motion (x, phase,
 *                 walk_freq, walk_speed, direction), screen-derived
 *                 constants (ground_y, base_hip_y), foot lock data
 *                 (per-leg locked position + on-ground flag), and
 *                 the computed joint positions for the current
 *                 frame. No heap allocation post-init.
 *
 * Rendering     : Painter's order — ground line first, body parts
 *                 next, then the head and feet on top:
 *                   (1) ground line (and optional grid)
 *                   (2) spine
 *                   (3) arms (left + right)
 *                   (4) legs (left + right)
 *                   (5) head 'O'
 *                   (6) feet — '*' swinging, '#' planted
 *                   (7) HUD row 0 (yellow) + hint row last (cyan)
 *
 * Performance   : O(1) per frame — about 12 trig calls plus 10
 *                 line draws. Microseconds; ncurses redraw is the
 *                 dominant cost.
 *
 * References    :
 *   Wikipedia, "Bipedal gait" — phases of walking, swing/stance
 *     classification, double-support periods.
 *     https://en.wikipedia.org/wiki/Bipedal_gait
 *   Wikipedia, "Inverse kinematics" — for the law-of-cosines IK
 *     used here on stance legs.
 *     https://en.wikipedia.org/wiki/Inverse_kinematics
 *   Marc Raibert, "Legged Robots That Balance" (MIT Press, 1986) —
 *     the classic textbook on hopping and walking robot control.
 *   This project, animation/fk_ik_helloworld.c — both forward and
 *     inverse kinematics in a single 3-link arm; walking uses the
 *     same two ideas alternately (FK during swing, IK during stance).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Walking is alternating "balance on one leg / step the other forward".
 * For each leg, half the cycle is in the air (SWING — moving forward)
 * and half is on the ground (STANCE — body moves over it). The two
 * legs are 180° out of phase: while one is planted, the other is
 * mid-air. That's why a human doesn't fall over — at every moment,
 * at least one foot is on the ground.
 *
 *
 * ANALOGY: WALKING IS CONTROLLED FALLING
 * ──────────────────────────────────────
 *
 * Stand still on one leg. Lift the other and let your body lean
 * forward. You start to fall — gravity pulls your COM ahead of your
 * planted foot. JUST before you fall, you swing the lifted leg
 * forward and plant it. Now you're balancing on the new foot. Lift
 * the OLD leg, lean forward, fall, swing it forward, plant. Repeat.
 *
 * That's walking. It's a sequence of catches: every step rescues a
 * fall in progress. The brain runs this loop on autopilot; this
 * file runs the same loop with sinusoidal angle functions instead
 * of a feedback controller.
 *
 *
 * ONE LEG'S WALK CYCLE
 * ─────────────────────
 *
 *   Phase φ moves from 0 to 2π per stride.
 *
 *      ┌──── SWING (foot in air) ────┐ ┌─── STANCE (foot on ground) ──┐
 *
 *      φ=0           φ=π/2          φ=π           φ=3π/2          φ=2π
 *      ↓              ↓              ↓              ↓              ↓
 *      foot         foot at        foot          body crosses    foot
 *      lifts off    apex           lands         over foot       lifts off
 *      (toe-off)    (peak)         (heel-strike) (mid-stance)    (toe-off)
 *
 *
 *      thigh angle:   -SWING_AMP · cos(φ)
 *
 *          φ=0      :  -SWING (leg is BACK behind hip — toe-off)
 *          φ=π/2    :   0     (vertical, mid-swing)
 *          φ=π      :  +SWING (leg is FORWARD ahead of hip — landing)
 *          φ=3π/2   :   0     (vertical, body over foot)
 *          φ=2π     :  -SWING (leg is BACK again, toe-off, repeats)
 *
 *
 *      knee bend:     LIFT_AMP · sin(φ)     (only matters during swing)
 *
 *          φ=0      :   0    (knee straight, foot on ground)
 *          φ=π/2    :  +LIFT (max bend, foot at peak)
 *          φ=π      :   0    (knee straight, foot landing)
 *          φ=3π/2   :  -LIFT (negative — but we only USE this during
 *                              swing where it's positive)
 *
 *
 * TWO LEGS COORDINATED (180° out of phase)
 * ────────────────────────────────────────
 *
 *      Left leg phase:    0      π/2     π      3π/2    2π
 *      Left:    [SWING ──────────────] [STANCE ─────────] (next cycle)
 *
 *      Right leg phase:   π      3π/2    2π     π/2     π
 *      Right:   [STANCE ──────────────] [SWING ──────────]
 *
 *      At every moment, exactly ONE leg is swinging and ONE is planted.
 *      That's why the robot doesn't fall over.
 *
 *
 * BODY MOTION
 * ───────────
 *
 *   Hip vertical:    hip_y = base_y + BOB_AMP · sin(2φ)    ← 2× per stride
 *
 *      φ=0      sin(0)   =  0     hip at base (foot transitioning)
 *      φ=π/4    sin(π/2) = +1     hip up (toe pushing off)
 *      φ=π/2    sin(π)   =  0     hip at base
 *      φ=3π/4   sin(3π/2)= -1     hip dipped (heel just struck)
 *      φ=π      sin(2π)  =  0     hip at base
 *
 *   Two peaks and two troughs per full stride — exactly what a real
 *   walking person's pelvis does (the "pelvic bob").
 *
 *   Hip lateral sway:  hip_x_offset = SWAY_AMP · cos(φ)    ← 1× per stride
 *
 *      Body leans toward whichever foot is planted.
 *      Without sway, the COM would fall outside the support polygon
 *      and the robot would tip sideways.
 *
 *
 * FK vs IK
 * ────────
 *
 *   The two halves of the gait use OPPOSITE kinds of kinematics:
 *
 *      SWING  : Forward Kinematics (FK)
 *               — pick angles, compute foot position
 *               — easy: just trig
 *
 *      STANCE : Inverse Kinematics (IK)
 *               — foot is locked, hip is moving
 *               — compute knee from those two
 *               — solved by law of cosines (same as ik_helloworld.c)
 *
 *
 * FOOT LOCK
 * ─────────
 *
 *   The KEY trick for natural-looking walking: every time a leg's
 *   sin(φ_leg) crosses 0 from positive to negative (touchdown), we
 *   record the foot's current FK position into foot_lock[leg]. From
 *   that instant until the next swing, the IK uses foot_lock[leg]
 *   as the fixed end-effector while the hip slides forward.
 *
 *   Without foot lock, the foot would slide along the ground during
 *   stance — looks like the robot is moonwalking.
 *
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. dt = wall-clock seconds since last frame, capped.
 *  2. Advance phase: phase += 2π · walk_freq · direction · dt
 *  3. Advance body x: x += walk_speed · direction · dt
 *  4. Detect touchdown for each leg: if sin(φ_old) > 0 and
 *     sin(φ_new) ≤ 0, lock foot at current FK landing position.
 *  5. compute_pose:
 *     a. body: bob, sway, hip_c, hip_j[2], torso_top, head
 *     b. arms: shoulder + ARM_LEN · (sin θ, cos θ)
 *     c. legs: per leg, swing → FK, stance → IK from foot_lock
 *  6. Wrap x at screen edges (toroidal).
 *  7. Render: ground → spine → arms → legs → head → feet → HUD.
 *
 *
 * KEY FORMULAS
 * ────────────
 *   thigh_angle  = -SWING_AMP · cos(φ_leg)
 *   knee_bend    =  LIFT_AMP · sin(φ_leg)         [swing only]
 *   shin_angle   =  thigh_angle - knee_bend       ← MINUS, not plus.
 *                  (heel folds up toward butt — the anatomical
 *                   direction. Plus would slant the lower leg
 *                   forward and produce the Smooth-Criminal lean.)
 *   knee_pos     = hip + UPPER_LEG · (sin θ_thigh,    cos θ_thigh)
 *   foot_pos     = knee + LOWER_LEG · (sin θ_shin,    cos θ_shin)
 *   stance_knee  = solve_ik2(hip, foot_lock, UPPER_LEG, LOWER_LEG)
 *   stride_len   = 2 · (UPPER + LOWER) · sin(SWING_AMP)
 *   walk_speed   = walk_freq · stride_len         ← coupled, not free
 *   bob          = BOB_AMP  · sin(2φ)
 *   sway         = SWAY_AMP · cos(φ)
 *   arm_angle    = ARM_SWING · sin(φ + π · arm_offset)
 *
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Sign convention. We use y-DOWN screen coordinates, so positive
 *    y is BELOW. The thigh "forward" in walking direction +x means
 *    the foot is right of the hip; we encode that by:
 *        knee.x = hip.x + L · sin(θ)
 *        knee.y = hip.y + L · cos(θ)
 *    With θ=0 the leg is vertical (knee straight below hip), with
 *    θ=+SWING it points down-forward, with θ=-SWING down-back.
 *
 *  • Foot lock detection — we sample sin(φ_old) and sin(φ_new) each
 *    frame. With variable dt, the cross point may be missed if dt
 *    is huge; we cap dt to avoid that.
 *
 *  • IK domain — the law of cosines fails if hip-to-foot distance
 *    is greater than UPPER_LEG + LOWER_LEG. We clamp the distance
 *    inside solve_ik2 so the leg stretches as a straight line at
 *    the limit instead of NaN-ing.
 *
 *  • Wrapping. When the robot leaves the right edge we shift it
 *    + foot_lock back to the left edge so the simulation continues
 *    seamlessly. Same for the other direction.
 *
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' to reverse. The robot walks backward — same gait,
 *    just played in reverse direction.
 *
 *  • Press 'space' to pause. Verify you can see ONE foot planted
 *    ('#') and ONE foot in the air ('*') — they should never both
 *    be in the same state.
 *
 *  • Press '.' (period) repeatedly while paused. Watch one leg
 *    cycle through the four key phases: lift, peak, land, plant.
 *
 *  • Press '+' to max speed. The gait frequency increases too —
 *    the robot's stride length stays roughly the same, it just
 *    cycles faster.
 *
 *  • Watch the arms: when the LEFT leg swings forward, the LEFT
 *    arm swings BACK. Counter-rotation = balancing angular
 *    momentum. Real humans do this without thinking.
 *
 *  • Watch the hip y-coordinate: it rises and falls TWICE per
 *    stride. That's the bob.
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
/* Physics in pixel space (8 px wide, 16 px tall per terminal cell). */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 robot proportions (pixel space) ────────────────────────── */
/*
 * Bigger proportions than the previous version so each body segment
 * spans 3-5 cells — line glyphs read smoothly without flickering.
 *
 * Total height = HEAD_R + TORSO_LEN + UPPER_LEG_LEN + LOWER_LEG_LEN
 *              = 8 + 72 + 56 + 48 = 184 px ≈ 11.5 cells.
 * Width at shoulders = 2·SHOULDER_W = 48 px = 6 cells.
 */
#define HEAD_R           8.0f       /* head radius (just for placement) */
#define TORSO_LEN       72.0f       /* hip → top of torso (4.5 cells)   */
#define SHOULDER_W      24.0f       /* half shoulder width              */
#define HIP_W           14.0f       /* half hip width                   */
#define UPPER_LEG_LEN   56.0f       /* hip → knee (3.5 cells)           */
#define LOWER_LEG_LEN   48.0f       /* knee → foot (3 cells)            */
#define ARM_LEN         60.0f       /* shoulder → hand (single segment) */

/* ── §1.4 gait amplitudes ────────────────────────────────────────── */
/*
 * SWING_AMP   — peak thigh angle from vertical (rad). 0.35 rad ≈ 20°,
 *               a calm walking stride; legs stay close to upright.
 * LIFT_AMP    — peak knee bend at mid-swing (rad). 0.45 rad ≈ 26°.
 * ARM_SWING   — peak arm swing from vertical (rad). 0.30 rad ≈ 17°.
 * BOB_AMP     — vertical hip bob, pixels (2× per stride).
 * SWAY_AMP    — lateral hip sway, pixels (1× per stride).
 */
#define SWING_AMP    0.35f
#define LIFT_AMP     0.45f
#define ARM_SWING    0.30f
#define BOB_AMP      3.0f
#define SWAY_AMP     4.0f

/* ── §1.5 gait dynamics ──────────────────────────────────────────── */
/*
 * walk_freq is the ONLY pace control. walk_speed is derived from it
 * via the stride length so the foot-plant geometry and body-advance
 * speed always stay consistent. Without this coupling, the foot
 * would land far ahead of the hip and the body would never cross
 * over it during stance — the legs would visibly lean forward all
 * the time (the "MJ smooth-criminal" lean).
 *
 *     stride_length = 2 · (UPPER_LEG + LOWER_LEG) · sin(SWING_AMP)
 *                   = 2 · 104 · sin(0.55) ≈ 108.8 px
 *     walk_speed    = walk_freq · stride_length
 */
#define WALK_FREQ_DEFAULT   1.6f    /* Hz — strides per second          */
#define WALK_FREQ_MIN       0.4f
#define WALK_FREQ_MAX       4.5f
#define WALK_FREQ_STEP      0.2f

/* ── §1.6 ground grid ────────────────────────────────────────────── */
#define GRID_PERIOD     6           /* tick mark every N cols */

/* ── §1.7 dt cap (spiral-of-death guard) ─────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.8 timing primitives ──────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.9 ncurses pair IDs ───────────────────────────────────────── */
enum {
    /* 1..2 — body */
    CP_BODY     = 1,        /* head, torso, neck, hip band — white     */
    CP_BODY_DIM,            /* connectors, less prominent              */
    /* 3..4 — left side (arm + leg) */
    CP_LEFT,                /* green                                    */
    /* 5    — right side */
    CP_RIGHT,               /* magenta                                  */
    /* 6..7 — feet */
    CP_FOOT_PLANT,          /* yellow — foot on ground                 */
    CP_FOOT_SWING,          /* cyan   — foot in air                    */
    /* 8    — ground */
    CP_GROUND,              /* dim gray                                 */
    /* 9..10 — HUD spec */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.10 HUD layout ─────────────────────────────────────────────── */
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
 * Four-colour palette — minimum needed to read the figure clearly:
 *
 *   body         white     head, torso, spine, hip band
 *   left side    green     left arm + left leg
 *   right side   magenta   right arm + right leg
 *   feet (planted) yellow  '#' foot on ground
 *   feet (swing)   cyan    '*' foot in air
 *   ground       dim grey  the line + grid
 *
 * Left/right colour separation lets a learner track which leg is
 * doing what during the gait cycle. Foot colour switches between
 * yellow (planted) and cyan (swing) so contact state is obvious.
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) bind separately on default
 * background per CLAUDE.md HUD spec.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,         255, -1);    /* near-white          */
        init_pair(CP_BODY_DIM,     244, -1);    /* dim grey-white      */
        init_pair(CP_LEFT,          82, -1);    /* lime green          */
        init_pair(CP_RIGHT,        201, -1);    /* magenta             */
        init_pair(CP_FOOT_PLANT,   226, -1);    /* yellow              */
        init_pair(CP_FOOT_SWING,    51, -1);    /* cyan                */
        init_pair(CP_GROUND,       240, -1);    /* dim grey            */
        init_pair(PAIR_HUD,        226, -1);    /* yellow on bg        */
        init_pair(PAIR_HINT,        51, -1);    /* cyan on bg          */
    } else {
        init_pair(CP_BODY,        COLOR_WHITE,   -1);
        init_pair(CP_BODY_DIM,    COLOR_WHITE,   -1);
        init_pair(CP_LEFT,        COLOR_GREEN,   -1);
        init_pair(CP_RIGHT,       COLOR_MAGENTA, -1);
        init_pair(CP_FOOT_PLANT,  COLOR_YELLOW,  -1);
        init_pair(CP_FOOT_SWING,  COLOR_CYAN,    -1);
        init_pair(CP_GROUND,      COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Physics happens in PIXEL space (8 px per cell horizontally, 16 px
 * per cell vertically). Conversion to cell coordinates happens only
 * at draw time. This keeps the gait math isotropic — circular knee
 * rotations look round even though terminal cells are 2× tall.
 */
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_pw(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_ph(int rows){ return (float)rows * CELL_H; }
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================== */
/* §5  robot — Robot type, gait math, FK swing, IK stance, foot lock      */
/* ===================================================================== */

/* ── §5.1 Vec2 + Robot type ──────────────────────────────────────── */

typedef struct { float x, y; } Vec2;

/*
 * Robot — full simulation state.
 *
 *   Motion (the unknowns advanced every frame):
 *     x          hip-centre x in pixel space
 *     phase      walking phase φ (radians, unbounded)
 *     walk_freq  gait frequency in Hz (strides per second)
 *     walk_speed pixel translation rate
 *     direction  +1 forward, -1 backward
 *
 *   Screen-derived constants (rebuilt on resize):
 *     ground_y     y of ground line in pixel space
 *     base_hip_y   y of hip in default standing pose
 *
 *   Per-leg foot-lock state:
 *     foot_lock[2]      world position where the foot was planted
 *     on_ground[2]      true while sin(φ_leg) ≤ 0 (stance)
 *
 *   Computed joints (refreshed every tick):
 *     hip_c, hip_j[2]   pelvis centre + per-leg hip joints
 *     torso_top         top-of-torso point (where shoulders attach)
 *     head_c            head centre
 *     shoulder[2]       per-arm shoulder pivots
 *     hand[2]           per-arm hand tips (single-segment arms)
 *     knee[2]           per-leg knee positions
 *     foot[2]           per-leg foot positions
 *
 *   UI / control:
 *     paused, step_once, show_grid, ...
 */
typedef struct {
    /* motion */
    float x, phase;
    float walk_freq, walk_speed;
    int   direction;            /* +1 forward, -1 backward */

    /* screen-derived */
    float ground_y, base_hip_y;

    /* foot lock */
    Vec2  foot_lock[2];
    bool  on_ground[2];

    /* computed joints */
    Vec2  hip_c;
    Vec2  hip_j[2];
    Vec2  torso_top, head_c;
    Vec2  shoulder[2], hand[2];
    Vec2  knee[2], foot[2];

    /* UI */
    bool  paused, step_once, show_grid;
} Robot;

/* ── §5.2 solve_ik2 — 2-link analytical IK (law of cosines) ──────── */

/*
 * Given a root joint H and target T at distance d = |T − H|, find
 * the elbow/knee position J that places joint A−B at the right
 * lengths U (upper) and L (lower).
 *
 *   cos α = (d² + U² − L²) / (2·d·U)
 *   α     = acos(cos α)
 *   φ     = atan2(T.y − H.y, T.x − H.x)
 *   J     = H + U · (cos(φ − α), sin(φ − α))
 *
 * The "minus" in (φ − α) places the bend on a chosen side of the
 * H→T line. For walking we want the knee bending FORWARD (+x in the
 * walk direction), which φ − α gives us in y-down coordinates.
 *
 * Distance is clamped to [|U−L|, U+L] so a degenerate target
 * (foot too close or too far from hip) just collapses into a
 * straight leg instead of NaN-ing.
 *
 * See animation/ik_helloworld.c for the same math taught from
 * scratch with diagrams.
 */
static Vec2 solve_ik2(Vec2 hip, Vec2 foot, float U, float L)
{
    float dx   = foot.x - hip.x;
    float dy   = foot.y - hip.y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1e-6f) dist = 1e-6f;

    float min_r = fabsf(U - L) + 0.5f;
    float max_r = U + L - 0.5f;
    float clamped = clampf(dist, min_r, max_r);

    float phi    = atan2f(dy, dx);
    float cos_a  = (clamped*clamped + U*U - L*L) / (2.0f * clamped * U);
    float ang    = phi - acosf(clampf(cos_a, -1.0f, 1.0f));

    return (Vec2){ hip.x + U * cosf(ang),
                   hip.y + U * sinf(ang) };
}

/* ── §5.3 leg_swing_pose — Forward Kinematics during swing ────────── */

/*
 * SWING phase math.
 *
 *   thigh_angle = -SWING_AMP · cos(φ_leg)
 *                  φ_leg = 0    →  -SWING (leg back, toe-off)
 *                  φ_leg = π/2  →   0     (vertical, mid-swing)
 *                  φ_leg = π    →  +SWING (leg forward, landing)
 *
 *   knee_bend   =  LIFT_AMP · sin(φ_leg)
 *                  peaks at π/2 (mid-swing) then straightens
 *
 *   shin_angle  =  thigh_angle - knee_bend
 *
 * The MINUS is anatomical. A human knee folds so the heel comes UP
 * toward the butt — the shin rotates BACKWARDS relative to the
 * thigh, not forwards. With (+) the foot would always lead the
 * knee forward and the lower leg would visibly slant in the walk
 * direction (the "Smooth-Criminal lean"). With (-), at mid-swing
 * the foot tucks BEHIND the knee like a real marching step.
 *
 * Knee and foot computed by walking the joint chain forward from
 * the hip. The +sin/+cos pattern places the joint below-and-along
 * the angle (y-down screen convention).
 */
static void leg_swing_pose(Robot *r, int i, Vec2 hip, float phi_leg, float dir)
{
    float thigh = -SWING_AMP * cosf(phi_leg) * dir;
    float ke    =  LIFT_AMP  * sinf(phi_leg);
    float shin  =  thigh - ke;

    Vec2 knee = { hip.x + UPPER_LEG_LEN * sinf(thigh),
                  hip.y + UPPER_LEG_LEN * cosf(thigh) };

    Vec2 foot = { knee.x + LOWER_LEG_LEN * sinf(shin),
                  knee.y + LOWER_LEG_LEN * cosf(shin) };

    /* Don't let the swinging foot punch through the ground. */
    if (foot.y > r->ground_y) foot.y = r->ground_y;

    r->knee[i]      = knee;
    r->foot[i]      = foot;
    r->on_ground[i] = false;
}

/* ── §5.4 leg_stance_pose — Inverse Kinematics during stance ─────── */

/*
 * STANCE phase math.
 *
 *   foot is LOCKED at foot_lock[i].
 *   hip moves forward as the body advances.
 *   knee = solve_ik2(hip, foot_lock[i], UPPER_LEG, LOWER_LEG)
 *
 * The IK has a unique solution (with the "knee-forward" choice of
 * sign in solve_ik2) because the leg is a 2-link chain whose
 * endpoints we know.
 */
static void leg_stance_pose(Robot *r, int i, Vec2 hip)
{
    Vec2 foot = r->foot_lock[i];
    foot.y    = r->ground_y;

    r->knee[i]      = solve_ik2(hip, foot, UPPER_LEG_LEN, LOWER_LEG_LEN);
    r->foot[i]      = foot;
    r->on_ground[i] = true;
}

/* ── §5.5 compute_arm — single-segment arm swing ─────────────────── */

/*
 * Each arm is a single segment from shoulder to hand. The swing
 * is counter-phase to the leg on the same side: when the LEFT
 * leg swings forward, the LEFT arm swings BACK. This is how
 * humans (and bipedal robots) balance angular momentum.
 *
 *   arm_phase = phase + π · counter_offset
 *   arm_angle = ARM_SWING · sin(arm_phase) · direction
 *   hand      = shoulder + ARM_LEN · (sin(angle), cos(angle))
 */
static void compute_arm(Robot *r, int i, float phase, float dir)
{
    /* arm i (0=left, 1=right) is counter-phase to leg i:
     * left arm offsets by π (matches right leg);
     * right arm has no offset (matches left leg). */
    float arm_phase = phase + (i == 0 ? (float)M_PI : 0.0f);
    float angle     = ARM_SWING * sinf(arm_phase) * dir;

    r->hand[i].x = r->shoulder[i].x + ARM_LEN * sinf(angle);
    r->hand[i].y = r->shoulder[i].y + ARM_LEN * cosf(angle);
}

/* ── §5.6 compute_pose — orchestrator (body + arms + legs) ──────── */

/*
 * Build the full pose for the current `phase` and `x`. Order:
 *   1. body: bob + sway → hip_c, hip_j[2]
 *   2. torso top, head
 *   3. shoulders (placed from torso_top + sway)
 *   4. arms (FK from shoulders)
 *   5. legs:
 *        for each leg i:
 *          φ_leg = phase + (i==1 ? π : 0)
 *          if sin(φ_leg) > 0 → leg_swing_pose (FK)
 *          else              → leg_stance_pose (IK)
 *
 * Sign of `dir` flips angle conventions when walking backward so
 * the limbs visibly retrograde the gait.
 */
static void compute_pose(Robot *r)
{
    float phi  = r->phase;
    float dir  = (float)r->direction;

    /* (1) body — bob and sway derive hip position. */
    float bob  = BOB_AMP  * sinf(2.0f * phi);
    float sway = SWAY_AMP * cosf(phi)   * dir;

    r->hip_c   = (Vec2){ r->x,                 r->base_hip_y + bob };
    r->hip_j[0]= (Vec2){ r->hip_c.x - HIP_W,   r->hip_c.y          };
    r->hip_j[1]= (Vec2){ r->hip_c.x + HIP_W,   r->hip_c.y          };

    /* (2) torso + head — sway shifts the upper body laterally. */
    r->torso_top = (Vec2){ r->hip_c.x + sway, r->hip_c.y - TORSO_LEN };
    r->head_c    = (Vec2){ r->torso_top.x,    r->torso_top.y - HEAD_R };

    /* (3) shoulders — placed at torso_top, half-shoulder-width apart. */
    r->shoulder[0] = (Vec2){ r->torso_top.x - SHOULDER_W, r->torso_top.y };
    r->shoulder[1] = (Vec2){ r->torso_top.x + SHOULDER_W, r->torso_top.y };

    /* (4) arms. */
    for (int i = 0; i < 2; i++)
        compute_arm(r, i, phi, dir);

    /* (5) legs — phase per leg, then FK or IK. */
    for (int i = 0; i < 2; i++) {
        float phi_leg = phi + (i == 1 ? (float)M_PI : 0.0f);
        if (sinf(phi_leg) > 0.0f) {
            leg_swing_pose (r, i, r->hip_j[i], phi_leg, dir);
        } else {
            leg_stance_pose(r, i, r->hip_j[i]);
        }
    }
}

/* ── §5.7 robot_init / robot_reset ───────────────────────────────── */

/*
 * robot_set_pace — clamp walk_freq and derive walk_speed.
 *
 * Speed is NOT a free parameter. The body must advance exactly one
 * stride length per stride period for the legs to look natural —
 * otherwise the hip never crosses over the planted foot and BOTH
 * legs visibly lean in the walking direction during stance.
 *
 *     stride_length = 2 · (UPPER + LOWER) · sin(SWING_AMP)
 *     walk_speed    = walk_freq · stride_length
 *
 * Always set the pace through this function so the two stay
 * coupled.
 */
static void robot_set_pace(Robot *r, float freq)
{
    if (freq < WALK_FREQ_MIN) freq = WALK_FREQ_MIN;
    if (freq > WALK_FREQ_MAX) freq = WALK_FREQ_MAX;

    float stride = 2.0f * (UPPER_LEG_LEN + LOWER_LEG_LEN) * sinf(SWING_AMP);
    r->walk_freq  = freq;
    r->walk_speed = freq * stride;
}

static void robot_init(Robot *r, int cols, int rows)
{
    memset(r, 0, sizeof *r);

    robot_set_pace(r, WALK_FREQ_DEFAULT);
    r->direction  = +1;
    r->show_grid  = true;
    r->phase      = 0.30f;            /* small offset so we don't start in a singular pose */

    r->ground_y   = (float)((rows - 4) * CELL_H);
    r->base_hip_y = r->ground_y - (UPPER_LEG_LEN + LOWER_LEG_LEN) * 0.88f;
    r->x          = (float)(cols * CELL_W) * 0.30f;

    /* Seed foot locks so the first stance leg has a sensible plant
     * position. One step forward, one step back from current x. */
    float step = (r->walk_speed / r->walk_freq) * 0.25f;
    r->foot_lock[0] = (Vec2){ r->x - step, r->ground_y };
    r->foot_lock[1] = (Vec2){ r->x + step, r->ground_y };

    compute_pose(r);
}

static void robot_reset(Robot *r, int cols, int rows)
{
    /* Preserve user-tuned settings across reset. */
    float freq  = r->walk_freq;
    int   dir   = r->direction;
    bool  grid  = r->show_grid;

    robot_init(r, cols, rows);

    robot_set_pace(r, freq);
    r->direction  = dir;
    r->show_grid  = grid;
}

/* ── §5.8 robot_tick — phase advance + foot lock + pose refresh ──── */

/*
 * One frame of physics:
 *   1. Skip if paused (unless single-step).
 *   2. Advance phase and x.
 *   3. Update screen-derived constants in case of resize.
 *   4. Detect touchdown: if sin(phi_old) > 0 and sin(phi_new) ≤ 0,
 *      the leg is entering stance. Lock foot at the FK landing pos
 *      computed from the new phase. (At touchdown, knee_bend ≈ 0
 *      so we just use thigh-only FK for the foot position.)
 *   5. Run compute_pose().
 *   6. Wrap x at screen edges (toroidal world).
 */
static void robot_tick(Robot *r, float dt, int cols, int rows)
{
    if (r->paused && !r->step_once) return;
    r->step_once = false;

    float phi_old = r->phase;
    float dir     = (float)r->direction;

    /* (2) advance */
    r->phase += 2.0f * (float)M_PI * r->walk_freq * dir * dt;
    r->x     += r->walk_speed * dir * dt;

    /* (3) screen-derived constants */
    r->ground_y   = (float)((rows - 4) * CELL_H);
    r->base_hip_y = r->ground_y - (UPPER_LEG_LEN + LOWER_LEG_LEN) * 0.88f;

    /* Pre-compute hip positions so touchdown FK uses the new hip. */
    float bob = BOB_AMP * sinf(2.0f * r->phase);
    Vec2 hip_j[2] = {
        { r->x - HIP_W, r->base_hip_y + bob },
        { r->x + HIP_W, r->base_hip_y + bob },
    };

    /* (4) touchdown detection — lock foot at FK landing pos. */
    for (int i = 0; i < 2; i++) {
        float phi_old_l = phi_old   + (i == 1 ? (float)M_PI : 0.0f);
        float phi_new_l = r->phase  + (i == 1 ? (float)M_PI : 0.0f);
        if (sinf(phi_old_l) > 0.0f && sinf(phi_new_l) <= 0.0f) {
            float thigh = -SWING_AMP * cosf(phi_new_l) * dir;
            float foot_x = hip_j[i].x
                         + UPPER_LEG_LEN * sinf(thigh)
                         + LOWER_LEG_LEN * sinf(thigh);   /* knee_bend≈0 */
            r->foot_lock[i] = (Vec2){ foot_x, r->ground_y };
        }
    }

    /* (5) full pose */
    compute_pose(r);

    /* (6) toroidal wrap so robot stays visible across the screen. */
    float sw = (float)(cols * CELL_W);
    if (r->x > sw + 80.0f) {
        r->x              -= sw + 160.0f;
        r->foot_lock[0].x -= sw + 160.0f;
        r->foot_lock[1].x -= sw + 160.0f;
    }
    if (r->x < -80.0f) {
        r->x              += sw + 160.0f;
        r->foot_lock[0].x += sw + 160.0f;
        r->foot_lock[1].x += sw + 160.0f;
    }
}

/* ===================================================================== */
/* §6  render — bones, ground, robot, HUD                                 */
/* ===================================================================== */

/* ── §6.1 helpers ────────────────────────────────────────────────── */

static inline bool in_screen(int r, int c, int rows, int cols)
{
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

/*
 * bone_glyph — pick the best ASCII character for a line segment
 * in pixel-space direction (dx, dy).
 *
 *   Mostly horizontal      → '-'
 *   Mostly vertical        → '|'
 *   NE / SW diagonal       → '/'
 *   NW / SE diagonal       → '\'
 *
 * The angle is measured in math (y-up) coordinates, so we negate dy
 * before atan2.  Reduce to [0, 180°) since direction is unsigned.
 */
static chtype bone_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx) * (180.0f / (float)M_PI);
    if (ang <    0.0f) ang += 360.0f;
    if (ang >= 180.0f) ang -= 180.0f;
    if (ang <  22.5f || ang >= 157.5f) return '-';
    if (ang <  67.5f )                 return '/';
    if (ang < 112.5f )                 return '|';
    return                                  '\\';
}

/* ── §6.2 draw_bone — Bresenham line with one bone glyph ─────────── */

/*
 * Stamp a single bone glyph (from bone_glyph) along a Bresenham
 * line in cell coordinates. Same glyph for every cell — at this
 * resolution, picking a single direction-appropriate glyph reads
 * cleaner than mixing glyphs per sub-segment.
 */
static void draw_bone(Vec2 a, Vec2 b, int cp, attr_t at,
                      int rows, int cols)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    if (sqrtf(dx*dx + dy*dy) < 0.5f) return;
    chtype g = bone_glyph(dx, dy);

    int r0 = px_to_cy(a.y), c0 = px_to_cx(a.x);
    int r1 = px_to_cy(b.y), c1 = px_to_cx(b.x);
    int dr = r1 - r0, dc = c1 - c0;
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = abs(dr) - abs(dc);
    int r = r0, c = c0;

    for (;;) {
        if (in_screen(r, c, rows, cols)) {
            attron (COLOR_PAIR(cp) | at);
            mvaddch(r, c, g);
            attroff(COLOR_PAIR(cp) | at);
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs(dc)) { err -= abs(dc); r += sr; }
        if (e2 <  abs(dr)) { err += abs(dr); c += sc; }
    }
}

/* ── §6.3 render_ground — line + optional scrolling grid ─────────── */

static void render_ground(const Robot *r, int rows, int cols)
{
    int gy = px_to_cy(r->ground_y);
    if (gy < 0 || gy >= rows) return;

    /* Solid ground line. */
    attron (COLOR_PAIR(CP_GROUND) | A_BOLD);
    for (int c = 0; c < cols; c++)
        mvaddch(gy, c, '_');
    attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);

    if (!r->show_grid) return;

    /* Tick marks at gy+1 — offset scrolls with robot.x so the grid
     * appears to move past while the robot stays roughly fixed on
     * screen (toroidal world). */
    if (gy + 1 >= rows - 1) return;
    int offset = (int)(r->x / (float)CELL_W) % GRID_PERIOD;
    if (offset < 0) offset += GRID_PERIOD;

    attron(COLOR_PAIR(CP_GROUND) | A_DIM);
    for (int c = 0; c < cols; c++) {
        chtype ch = ((c + offset) % GRID_PERIOD == 0) ? '|' : ' ';
        mvaddch(gy + 1, c, ch);
    }
    attroff(COLOR_PAIR(CP_GROUND) | A_DIM);
}

/* ── §6.4 render_robot — body, arms, legs, head, feet ────────────── */

/*
 * Painter's order — last write wins:
 *
 *   1. spine         hip_c → torso_top
 *   2. arms          shoulder → hand   (left green, right magenta)
 *   3. legs          hip → knee → foot (left green, right magenta)
 *   4. hip band      hip_j[0] → hip_j[1]
 *   5. shoulder band shoulder[0] → shoulder[1]
 *   6. head          'O' at head_c
 *   7. feet          '#' planted (yellow) or '*' swing (cyan)
 *
 * The spine goes UNDER everything else so arm and leg bones can
 * cross over it cleanly. Head and feet go on TOP because they're
 * the most distinctive visual elements.
 */
static void render_robot(const Robot *r, int rows, int cols)
{
    /* (1) spine */
    draw_bone(r->hip_c, r->torso_top, CP_BODY, A_BOLD, rows, cols);

    /* (2) arms */
    draw_bone(r->shoulder[0], r->hand[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(r->shoulder[1], r->hand[1], CP_RIGHT, A_BOLD, rows, cols);

    /* (3) legs — thigh and shin per side */
    draw_bone(r->hip_j[0], r->knee[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(r->knee[0],  r->foot[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(r->hip_j[1], r->knee[1], CP_RIGHT, A_BOLD, rows, cols);
    draw_bone(r->knee[1],  r->foot[1], CP_RIGHT, A_BOLD, rows, cols);

    /* (4) hip band */
    draw_bone(r->hip_j[0], r->hip_j[1], CP_BODY_DIM, A_DIM, rows, cols);

    /* (5) shoulder band */
    draw_bone(r->shoulder[0], r->shoulder[1], CP_BODY_DIM, A_DIM, rows, cols);

    /* (6) head — single 'O' glyph, drawn last over neck/collar */
    {
        int hcx = px_to_cx(r->head_c.x);
        int hcy = px_to_cy(r->head_c.y);
        if (in_screen(hcy, hcx, rows, cols)) {
            attron (COLOR_PAIR(CP_BODY) | A_BOLD);
            mvaddch(hcy, hcx, 'O');
            attroff(COLOR_PAIR(CP_BODY) | A_BOLD);
        }
    }

    /* (7) feet — '#' planted (yellow) or '*' swing (cyan) */
    for (int i = 0; i < 2; i++) {
        int fcx = px_to_cx(r->foot[i].x);
        int fcy = px_to_cy(r->foot[i].y);
        if (!in_screen(fcy, fcx, rows, cols)) continue;
        int    cp = r->on_ground[i] ? CP_FOOT_PLANT : CP_FOOT_SWING;
        chtype ch = r->on_ground[i] ? '#'           : '*';
        attron (COLOR_PAIR(cp) | A_BOLD);
        mvaddch(fcy, fcx, ch);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
}

/* ── §6.5 render_hud — yellow status row 0 + cyan hint bottom row ── */

/*
 * Row 0: fps, freq, speed, direction, paused/running.
 * Bottom row: full key list.
 *
 * Both pairs are bound on default terminal background so they
 * remain legible against any backdrop.
 */
static void render_hud(const Robot *r, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  freq:%.2f Hz  speed:%5.0f px/s  dir:%s  legL:%s  legR:%s  %s ",
             fps, r->walk_freq, r->walk_speed,
             r->direction == 1 ? "→ forward" : "← reverse",
             r->on_ground[0] ? "STANCE" : "swing ",
             r->on_ground[1] ? "STANCE" : "swing ",
             r->paused ? "PAUSED" : "running");

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reverse  ↑/↓:speed  .:step  g:grid ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §6.6 scene_draw — paint full frame ──────────────────────────── */

static void scene_draw(const Robot *r, double fps, int rows, int cols)
{
    erase();
    render_ground(r, rows, cols);
    render_robot (r, rows, cols);
    render_hud   (r, fps, rows, cols);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present                                    */
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
/* §8  app — signals, resize, variable-dt main loop                       */
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

static void app_handle_key(App *app, int ch)
{
    Robot *r = &app->robot;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        app->running = 0;
        break;

    case ' ':
        r->paused = !r->paused;
        break;

    case 'r': case 'R':
        r->direction = -r->direction;
        break;

    case '+': case '=': case KEY_UP:
        robot_set_pace(r, r->walk_freq + WALK_FREQ_STEP);
        break;

    case '-': case '_': case KEY_DOWN:
        robot_set_pace(r, r->walk_freq - WALK_FREQ_STEP);
        break;

    case '.':
        r->step_once = true;
        break;

    case 'g': case 'G':
        r->show_grid = !r->show_grid;
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

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    robot_init (&app->robot, app->screen.cols, app->screen.rows);

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            robot_reset(&app->robot, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
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
        robot_tick(&app->robot, dt,
                   app->screen.cols, app->screen.rows);

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
                   app->screen.rows, app->screen.cols);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
