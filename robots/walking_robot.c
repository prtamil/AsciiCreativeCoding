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
 *   §5  robot        — Gait / GroundAnchor / FootLocks / Pose / RobotUI
 *                      composed into Robot; FK swing, IK stance,
 *                      foot-lock, init/reset, tick (sub-sectioned)
 *   §6  render       — draw bones, ground, robot, HUD (sub-sectioned)
 *   §7  screen       — ncurses init / present
 *   §8  scene        — FrameTimer + Scene container; signals, resize,
 *                      variable-dt main loop
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
 * Data-structure: A Scene owns the whole program state. Inside it the
 *                 Robot is composed of five concept-sized sub-structs
 *                 so each field's role is obvious at the access site:
 *
 *                   Gait         — phase, walk_freq, walk_speed, direction
 *                                    (the cycle that drives everything)
 *                   GroundAnchor — ground_y, base_hip_y
 *                                    (screen-derived placement, rebuilt
 *                                     on resize)
 *                   FootLocks    — per-leg planted positions + on-ground
 *                                    flag (key to natural-looking stance)
 *                   Pose         — hip, torso, head, shoulders, hands,
 *                                    knees, feet — refreshed every tick
 *                   RobotUI      — paused / step_once / show_grid
 *
 *                 Plus r->x (hip-centre world position) at the top
 *                 level since it's the single anchor every other
 *                 position is computed relative to. No heap
 *                 allocation post-init.
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
 * References    : grouped by the concepts this file teaches. Ten
 *                  entries — one or two strong picks per topic. The
 *                  in-project cross-reference to fk_ik_helloworld.c
 *                  is the cheapest place to start.
 *
 *   ── Bipedal locomotion & legged robotics (concepts A, B, C) ──────
 *   Raibert, "Legged Robots That Balance" (MIT Press, 1986). The
 *     foundational text on hopping and walking robot control. The
 *     SWING / STANCE phase decomposition used here traces back to
 *     chapter 2.
 *   Inman, Ralston & Todd, "Human Walking" (2nd ed., Williams &
 *     Wilkins, 1994). The canonical biomechanics reference on the
 *     human gait cycle — touchdown, mid-stance, toe-off, swing.
 *     Source of the "controlled falling" mental model.
 *   Winter, "Biomechanics and Motor Control of Human Movement"
 *     (Wiley, 4th ed., 2009). Quantitative gait analysis — joint
 *     angle trajectories, COM motion, pelvic bob/sway. The sin(φ)
 *     and sin(2φ) shapes used in §5 are direct simplifications of
 *     what Winter measures from real walkers.
 *
 *   ── Forward / inverse kinematics (concepts B, C, F) ──────────────
 *   Parent, "Computer Animation: Algorithms and Techniques" (Morgan
 *     Kaufmann, 3rd ed., 2012). Chapters 5–6 cover FK and analytical
 *     2-link IK exactly as used here — same law-of-cosines derivation
 *     as solve_ik2().
 *   Buss, "Introduction to Inverse Kinematics with Jacobian
 *     Transpose, Pseudoinverse and Damped Least Squares" (UCSD
 *     tech report, 2009). Free PDF, comprehensive treatment of IK
 *     methods beyond the analytic 2-link case used here.
 *     https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
 *   Boulic, Magnenat-Thalmann & Thalmann, "A global human walking
 *     model with real-time kinematic personification" (The Visual
 *     Computer 6:344–358, 1990). Procedural walking in computer
 *     animation — the parametric-trajectory approach this file uses,
 *     predating mocap-driven character rigs.
 *   This project, animation/fk_ik_helloworld.c — forward AND inverse
 *     kinematics in a single 3-link arm; walking_robot.c uses the
 *     same two ideas ALTERNATELY (FK during swing, IK during stance).
 *
 *   ── Rendering (§6) ───────────────────────────────────────────────
 *   Bresenham, "Algorithm for computer control of a digital plotter"
 *     (IBM Systems Journal 4(1):25–30, 1965). The line-drawing
 *     algorithm implemented in draw_bone(). Unusually readable
 *     original paper.
 *   Padala, "NCURSES Programming HOWTO" (The Linux Documentation
 *     Project). Practical reference for the ncurses API used in
 *     §3 (color_init) and §6 (mvaddch, attron, attroff).
 *     https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *   ── Game loop & numerical timing (§8 main loop) ──────────────────
 *   Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004). The
 *     canonical write-up of the dt-cap pattern in main() — explains
 *     why measuring wall-clock dt each frame and capping it is
 *     enough to keep the gait stable at any frame rate.
 *     https://gafferongames.com/post/fix_your_timestep/
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
 *
 * Background you need
 * ───────────────────
 *   - 2-link FK (fk_helloworld T2-T3) — angles → joint positions.
 *   - 2-link IK (ik_helloworld T3-T5) — law of cosines.
 *   - diff_drive_robot T2 — Euler integration of pose.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real biomechanical gait analysis (zero-moment point, capture
 *     point control). The gait is purely OPEN-LOOP procedural; no
 *     stability feedback.
 *   - Centre-of-mass dynamics. The body is rigidly procedural —
 *     it doesn't fall.
 *   - Inverse dynamics (forces / torques). Pure kinematics.
 *   - Reinforcement learning policies. Hand-crafted gait.
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

/* ── §1.11 rest pose & initial conditions ─────────────────────────── */

/* Rest leg extension fraction — base_hip_y sits at (UPPER+LOWER)·this
 * above the ground line. 0.88 leaves the knees slightly bent, which
 *   (a) avoids the IK singularity at full extension (cos α → ±1),
 *   (b) matches relaxed human standing (locked knees ≠ comfortable). */
#define STAND_LEG_RATIO         0.88f

/* Phase φ at boot — small offset so the first frame isn't in a
 * singular sin(0) = 0 configuration where touchdown detection would
 * fire spuriously on tick 1. */
#define INITIAL_PHASE_RAD       0.30f

/* Seed foot-lock distance as a fraction of one stride period's travel.
 *   (walk_speed / walk_freq) = stride_length
 *   seed_offset                = stride_length · FOOT_LOCK_SEED_FRAC
 * Places each seed foot a quarter-stride from x at boot. */
#define FOOT_LOCK_SEED_FRAC     0.25f

/* Initial body x as a fraction of screen width (so the bot starts
 * well inside the frame, not flush with an edge). */
#define INITIAL_X_FRAC          0.30f

/* Rows reserved at the bottom of the screen so the ground line, the
 * grid-tick row, and the hint strip don't crowd each other.
 *   ground_y = (rows - GROUND_BOTTOM_ROWS) · CELL_H */
#define GROUND_BOTTOM_ROWS      4

/* ── §1.12 toroidal wrap ───────────────────────────────────────────── */

/* When x drifts outside [ −WRAP_MARGIN_PX , screen_w + WRAP_MARGIN_PX ]
 * the body teleports across by WRAP_TOTAL_PX. The two distances are
 * intentionally different so the bot WALKS off-screen before popping
 * in on the opposite side (no visible teleport). */
#define WRAP_MARGIN_PX         80.0f
#define WRAP_TOTAL_PX         160.0f

/* ── §1.13 IK numerical safeties ──────────────────────────────────── */

/* Minimum hip → foot distance before solve_ik2() substitutes a tiny
 * non-zero value, avoiding 1/0 in the cos α denominator when the
 * limb is degenerately folded. */
#define IK_DIST_FLOOR          1e-6f

/* Slack added/subtracted from the reachable interval [|U-L|, U+L] so
 * a foot RIGHT at full extension doesn't push cos α to ±1.0 exactly
 * and trigger an acos(1) = 0 collapse. */
#define IK_REACH_SLACK          0.5f

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
/*
 * Palette table — each row maps one role to its 256-mode and 8-mode
 * foreground codes. color_init() walks this once with the correct
 * column selected at runtime. Reading the table top-to-bottom gives
 * the full palette in one glance.
 */
typedef struct {
    short pair;          /* CP_* / PAIR_* id from §1.9               */
    short fg256;         /* foreground when COLORS >= 256            */
    short fg8;           /* foreground in 8-colour fallback          */
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair             256    8-mode-fallback   role                */
    { CP_BODY,          255,   COLOR_WHITE   }, /* near-white         */
    { CP_BODY_DIM,      244,   COLOR_WHITE   }, /* dim grey-white     */
    { CP_LEFT,           82,   COLOR_GREEN   }, /* lime green         */
    { CP_RIGHT,         201,   COLOR_MAGENTA }, /* magenta            */
    { CP_FOOT_PLANT,    226,   COLOR_YELLOW  }, /* yellow             */
    { CP_FOOT_SWING,     51,   COLOR_CYAN    }, /* cyan               */
    { CP_GROUND,        240,   COLOR_WHITE   }, /* dim grey           */
    { PAIR_HUD,         226,   COLOR_YELLOW  }, /* yellow on bg       */
    { PAIR_HINT,         51,   COLOR_CYAN    }, /* cyan on bg         */
};
#define PALETTE_LEN  (int)(sizeof PALETTE / sizeof PALETTE[0])

static void color_init(void)
{
    start_color();
    use_default_colors();

    bool truecolor = (COLORS >= 256);
    for (int i = 0; i < PALETTE_LEN; i++) {
        short fg = truecolor ? PALETTE[i].fg256 : PALETTE[i].fg8;
        init_pair(PALETTE[i].pair, fg, -1);
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

/* ── §5.1 Vec2 + sub-structs + Robot composition ──────────────────── */

/*
 * Vec2 — 2-D pixel-space point.
 *
 * INTENT
 *   Every joint position, foot lock, and screen-derived constant in
 *   this file lives in the same continuous pixel-space coordinate
 *   system (CELL_W × CELL_H sub-pixels per character cell). Vec2 is
 *   the shared currency. Conversion to discrete cell coordinates
 *   happens once at draw time via px_to_cx / px_to_cy in §4.
 *
 * CONVENTION
 *   x grows to the right; y grows DOWN (standard screen convention).
 *   Math that uses y-up (e.g. bone_glyph's angle test) negates dy
 *   at the boundary.
 */
typedef struct {
    float x;        /* horizontal, pixels, x grows right            */
    float y;        /* vertical,   pixels, y grows DOWN (screen)    */
} Vec2;

/*
 * The Robot used to be a flat type with twenty-odd fields in five
 * unrelated groups. The composition below makes each field's role
 * obvious at the access site:
 *
 *      r->x                  hip-centre world position
 *      r->gait.phase         cycle position (the unknown advanced each tick)
 *      r->anchor.ground_y    screen-derived placement
 *      r->locks.pos[i]       per-leg planted position
 *      r->pose.knee[i]       computed joint position this frame
 *      r->ui.paused          keyboard flag
 *
 * Same data, same algorithm — only the namespace changed.
 */

/*
 * §5.1.1 Gait — the cycle that drives every limb motion.
 *
 * INTENT
 *   One float — phase — IS the simulation's state. Every limb angle,
 *   every body offset, every touchdown event derives from sin/cos of
 *   this phase. The other three fields parameterise HOW phase advances:
 *      dφ/dt = 2π · walk_freq · direction
 *      dx/dt =      walk_speed · direction
 *
 * WHY walk_speed IS DERIVED (NOT FREE)
 *   The body must advance exactly ONE stride length per stride period
 *   for the legs to look natural. Otherwise the foot lands far ahead
 *   of the hip and the body never crosses over it during stance —
 *   producing the "Smooth-Criminal lean" where both legs slope in the
 *   walk direction. The coupling is:
 *
 *      stride_length = 2 · (UPPER_LEG + LOWER_LEG) · sin(SWING_AMP)
 *      walk_speed    = walk_freq · stride_length
 *
 *   ALWAYS set the pace via robot_set_pace() so the two stay coupled.
 *
 * ALGORITHM REFERENCES
 *   Boulic, Magnenat-Thalmann & Thalmann (1990) — phase-driven
 *     parametric gait, the same single-clock approach used here.
 *   Inman/Ralston/Todd (1994) — empirical stride-period vs.
 *     stride-length linearity that motivates the coupling above.
 */
typedef struct {
    float phase;             /* φ, radians. UNBOUNDED — wraps via   *
                              * sin/cos rather than explicit mod 2π.*
                              * Advances each frame by              *
                              *   2π · walk_freq · direction · dt . *
                              * Init: 0.30 (small offset so the     *
                              * first frame isn't in a singular     *
                              * sin(0) = 0 configuration).          */

    float walk_freq;         /* Hz — strides per second, the ONLY  *
                              * user-tunable pace dial. Clamped to  *
                              * [WALK_FREQ_MIN, WALK_FREQ_MAX]      *
                              * = [0.4, 4.5]. Default 1.6 Hz        *
                              * (calm human walking pace).          */

    float walk_speed;        /* px/sec — body translation rate.    *
                              * NEVER set directly; derived inside *
                              * robot_set_pace() as                *
                              *   walk_freq · stride_length .       */

    int   direction;         /* +1 forward, -1 backward. Flipped   *
                              * by 'r' key. Multiplies dφ/dt and   *
                              * dx/dt so reversing direction       *
                              * retrogrades the gait.              */
} Gait;

/*
 * §5.1.2 GroundAnchor — screen-derived placement constants.
 *
 * INTENT
 *   The pose math never reads ncurses dimensions directly. Instead,
 *   these two values are computed once at init (and again on
 *   SIGWINCH) from the current terminal size, and every joint
 *   formula reads them from here. Resize handling becomes a 2-field
 *   write; no other code path needs to know the screen changed.
 *
 * RATIONALE FOR 0.88 FACTOR
 *   base_hip_y sits at  ground_y − (UPPER + LOWER) · 0.88 .
 *   The legs are NOT fully extended at rest — 88% of full length
 *   leaves the knees slightly bent, which:
 *     (a) avoids a divide-by-zero singularity in solve_ik2() when
 *         both legs are perfectly straight (cos α → ±1, acos → 0
 *         which would collapse the IK to a degenerate point);
 *     (b) matches how humans actually stand — locked-straight legs
 *         are a chronic-pain posture, not a relaxed one.
 */
typedef struct {
    float ground_y;          /* y of ground line in pixel space.    *
                              * Computed from screen height as     *
                              *   (rows - 4) · CELL_H .             *
                              * The 4-row margin leaves room for    *
                              * HUD row 0 + hint row last +         *
                              * grid-tick row gy+1. Updated on      *
                              * SIGWINCH inside robot_tick().       */

    float base_hip_y;        /* y of hip in DEFAULT standing pose, *
                              * before any bob is applied:          *
                              *   ground_y − (UPPER+LOWER) · 0.88 .  *
                              * Every frame's hip_c.y derives from *
                              * here as  base_hip_y + BOB_AMP·sin(2φ).*/
} GroundAnchor;

/*
 * §5.1.3 FootLocks — per-leg planted positions + stance flag.
 *
 * INTENT
 *   The KEY trick for natural-looking walking. At the moment a leg
 *   transitions from swing → stance (sin(φ_leg) crosses + → -), we
 *   record where the foot is in the world into pos[i]. From that
 *   instant until the next swing, IK uses pos[i] as the fixed end-
 *   effector while the hip slides forward OVER it.
 *
 *   Without foot lock, the foot would slide along the ground during
 *   stance and the robot would look like it was MOONWALKING — toes
 *   pointed forward but feet drifting back relative to the body.
 *
 * STATE MACHINE PER LEG (driven by sin(φ_leg) sign)
 *
 *       sin(φ_leg) > 0   →  SWING   : FK from hip drives foot
 *       sin(φ_leg) ≤ 0   →  STANCE  : pos[i] anchors foot, IK from hip
 *
 *   The on_ground[i] flag mirrors the sign so the renderer picks
 *   the right glyph ('#' planted yellow, '*' swing cyan) and the
 *   HUD shows STANCE / swing per leg without re-evaluating sin.
 *
 * WHY POSITION IS CAPTURED AT TOUCHDOWN (not held constant globally)
 *   pos[i] needs to advance with the body — every new step plants the
 *   foot at a NEW world position. Capturing at the touchdown event
 *   gives us the right cadence automatically: each stride plants one
 *   foot, and that foot anchors the body for a half-cycle.
 *
 * ALGORITHM REFERENCES
 *   Raibert (1986) — the swing/stance decomposition this struct
 *     embodies is the central abstraction in legged-robot control.
 *   Inman/Ralston/Todd (1994) — empirical foot-trajectory data
 *     that motivates the "lock and let hip cross" model.
 */
typedef struct {
    Vec2 pos[2];             /* world position where leg i is      *
                              * planted, in pixels. Captured at    *
                              * the swing → stance transition by   *
                              * touchdown detection in robot_tick. *
                              * y is always ground_y (foot rests   *
                              * on the ground line).               */

    bool on_ground[2];       /* true while leg i is in stance     *
                              * (sin(φ_leg) ≤ 0). Used by the     *
                              * renderer for foot glyph + colour  *
                              * and by the HUD for STANCE/swing.  */
} FootLocks;

/*
 * §5.1.4 Pose — every joint position the renderer needs this frame.
 *
 * INTENT
 *   Written entirely by compute_pose(); read-only afterwards. The
 *   renderer's job is just "draw a line from A to B" for every pair
 *   of connected joints in this struct. Caching them here means the
 *   trig math runs once per frame, not once per render-pass.
 *
 * KINEMATIC HIERARCHY (built top → bottom inside compute_pose)
 *
 *      hip_c                     pelvis centre (= world x, base_y + bob)
 *        ├── hip_j[0,1]          left/right hip joints (±HIP_W from hip_c)
 *        │     └── knee[0,1]     FK from hip if swing, IK if stance
 *        │           └── foot[0,1]  FK from knee, OR locked at FootLocks.pos
 *        └── torso_top           hip_c + sway, UP by TORSO_LEN
 *              ├── shoulder[0,1] left/right shoulders (±SHOULDER_W from top)
 *              │     └── hand[0,1] single-segment arm swing from shoulder
 *              └── head_c        torso_top, UP by HEAD_R  ('O' glyph)
 *
 *   Each parent's position must be known before its children — that's
 *   why compute_pose() walks the chain strictly top-down.
 *
 * WHY CACHED INSTEAD OF RECOMPUTED PER DRAW
 *   render_robot() makes 10 draw_bone() calls referencing 12 joint
 *   positions; computing each pair on-demand would either re-evaluate
 *   the same trig multiple times or require careful sequencing. A
 *   plain "compute once, read many times" struct is simpler and the
 *   storage is trivial (≈ 100 bytes).
 *
 * ALGORITHM REFERENCES
 *   Parent (2012) §5–6 — forward-kinematic joint chains in computer
 *     animation. Same parent-before-child walk.
 *   Buss (2009) — IK survey covering the stance-leg case
 *     (the law-of-cosines solver in solve_ik2()).
 */
typedef struct {
    /* — pelvis & torso ————————————————————————————————————————— */
    Vec2 hip_c;              /* pelvis centre. ROOT of the whole  *
                              * kinematic chain.                  */
    Vec2 hip_j[2];           /* per-leg hip joints, ±HIP_W from   *
                              * hip_c on x. Root for each leg.    */
    Vec2 torso_top;          /* top of torso. hip_c + lateral     *
                              * sway, UP by TORSO_LEN. The sway   *
                              * shift is what makes the upper     *
                              * body lean over the planted foot.  */
    Vec2 head_c;             /* head centre. torso_top, UP by    *
                              * HEAD_R. Rendered as 'O' glyph.   */

    /* — arms ——————————————————————————————————————————————————— */
    Vec2 shoulder[2];        /* per-arm pivot, ±SHOULDER_W from  *
                              * torso_top on x.                  */
    Vec2 hand[2];            /* per-arm hand tip. Single-segment *
                              * FK from shoulder by ARM_LEN at   *
                              * arm_angle. Counter-phase to leg  *
                              * on the same side (angular        *
                              * momentum balance).               */

    /* — legs ——————————————————————————————————————————————————— */
    Vec2 knee[2];            /* per-leg knee position. FK from   *
                              * hip when swinging; IK via        *
                              * solve_ik2() when in stance.      */
    Vec2 foot[2];            /* per-leg foot position. FK from   *
                              * knee when swinging; = locks.pos  *
                              * when in stance. Clamped above    *
                              * ground_y so a swing foot can't   *
                              * punch through the floor.         */
} Pose;

/*
 * §5.1.5 RobotUI — keyboard-toggled flags.
 *
 * INTENT
 *   Separate "human input state" from physics state. Whether the
 *   simulation is paused or the grid is showing has nothing to do
 *   with the gait — these flags exist purely because there's a
 *   human at the keyboard. Grouping them in their own sub-struct
 *   makes that obvious at the type level.
 *
 * PRESERVATION ACROSS RESET
 *   robot_reset() preserves the user-tuned flags (show_grid, plus
 *   walk_freq and direction from Gait) across resize — pressing
 *   'r' resets the SIMULATION but not the user's preferences.
 */
typedef struct {
    bool paused;             /* spacebar toggles. When true,       *
                              * robot_tick() returns immediately   *
                              * (unless step_once is also set).    */

    bool step_once;          /* '.' key sets this. Lets one tick   *
                              * happen while paused, then is        *
                              * cleared inside robot_tick().        *
                              * Useful for frame-by-frame study.    */

    bool show_grid;          /* 'g' key toggles. When true,        *
                              * render_ground() draws tick marks   *
                              * at gy+1 that scroll with x — gives *
                              * the illusion the world is moving   *
                              * past while the robot stays mostly  *
                              * centred on screen.                 */
} RobotUI;

/*
 * §5.1.6 Robot — composition of the five concept-sized sub-structs.
 *
 * INTENT
 *   Before the refactor this was a flat type with ~20 fields in five
 *   unrelated groups. The composition makes the role of every access
 *   obvious at the use site:
 *
 *      r->x                  hip-centre world position
 *      r->gait.phase         cycle position (the unknown)
 *      r->anchor.ground_y    screen-derived placement
 *      r->locks.pos[i]       per-leg planted position
 *      r->pose.knee[i]       computed joint this frame
 *      r->ui.paused          keyboard flag
 *
 *   Same bytes, same algorithm — only the namespace changed.
 *
 * LAYERING (read these in order to understand the simulation)
 *
 *   1. Gait         — what is being advanced (φ, and x via walk_speed)
 *   2. GroundAnchor — where the world floor IS
 *   3. FootLocks    — which feet are currently PLANTED
 *   4. Pose         — every joint POSITION (computed each tick)
 *   5. RobotUI      — human flags
 *
 * WHY x IS AT THE TOP LEVEL (NOT INSIDE Gait)
 *   x is the single ANCHOR every other position derives from. Every
 *   joint is at some offset relative to x, every foot lock is in
 *   the same world coordinate system. Keeping it at the top makes
 *   `r->x` the obvious place to read "where is the robot" — moving
 *   it inside Gait would imply it's a gait parameter, which it
 *   isn't (Gait drives HOW x advances, but x itself is world state).
 */
typedef struct {
    float        x;          /* hip-centre x in pixel space (world).*
                              * Advances each tick by               *
                              *   walk_speed · direction · dt .     *
                              * Wrapped toroidally at screen edges  *
                              * inside robot_tick().                */

    Gait         gait;       /* phase + freq + speed + direction.  *
                              * See §5.1.1.                         */

    GroundAnchor anchor;     /* ground_y + base_hip_y. Resized on  *
                              * SIGWINCH. See §5.1.2.              */

    FootLocks    locks;      /* per-leg pos + on_ground flag.     *
                              * See §5.1.3.                       */

    Pose         pose;       /* every joint position this frame.  *
                              * Written by compute_pose(). §5.1.4.*/

    RobotUI      ui;         /* keyboard flags. See §5.1.5.      */
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
    /* (1) hip → foot displacement and distance */
    float dx   = foot.x - hip.x;
    float dy   = foot.y - hip.y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < IK_DIST_FLOOR) dist = IK_DIST_FLOOR;

    /* (2) clamp distance into the reachable interval with slack to
     *     avoid the cos α = ±1 collapse at exact full extension. */
    float min_reach = fabsf(U - L) + IK_REACH_SLACK;
    float max_reach = U + L        - IK_REACH_SLACK;
    float reach     = clampf(dist, min_reach, max_reach);

    /* (3) law of cosines — angle between (hip → foot) and (hip → knee) */
    float cos_alpha = (reach*reach + U*U - L*L) / (2.0f * reach * U);
    float alpha     = acosf(clampf(cos_alpha, -1.0f, 1.0f));

    /* (4) knee = hip + U · (cos(φ−α), sin(φ−α))
     *     φ is the angle of the hip→foot vector; subtracting α
     *     bends the knee on the chosen side of the line. */
    float phi       = atan2f(dy, dx);
    float knee_ang  = phi - alpha;
    return (Vec2){ hip.x + U * cosf(knee_ang),
                   hip.y + U * sinf(knee_ang) };
}

/* ── §5.3 leg_swing_pose — FK helpers + swing pose ────────────────── */

/* Forward-kinematic walk down one leg: thigh angle → knee, shin
 * angle → foot. Y-down screen coordinates use sin for x-offset
 * (+sin = forward) and cos for y-offset (+cos = down).
 *
 * See MENTAL MODEL → "ONE LEG'S WALK CYCLE" for the full sweep of
 * thigh_angle through the phase, and KEY FORMULAS for the
 * shin = thigh − bend  derivation (the minus is anatomical: the
 * heel folds UP toward the butt, not the Smooth-Criminal lean). */
static Vec2 fk_advance(Vec2 root, float angle, float len)
{
    return (Vec2){ root.x + len * sinf(angle),
                   root.y + len * cosf(angle) };
}

static void leg_swing_pose(Robot *r, int i, Vec2 hip, float phi_leg, float dir)
{
    /* (1) gait angles from phase
     *     thigh sweeps -SWING (back) → 0 → +SWING (forward) over swing
     *     knee bends LIFT at mid-swing then straightens for landing
     *     shin = thigh - bend (anatomical: heel folds UP toward butt) */
    float thigh_angle = -SWING_AMP * cosf(phi_leg) * dir;
    float knee_bend   =  LIFT_AMP  * sinf(phi_leg);
    float shin_angle  =  thigh_angle - knee_bend;

    /* (2) FK chain — hip → knee → foot */
    Vec2 knee = fk_advance(hip,  thigh_angle, UPPER_LEG_LEN);
    Vec2 foot = fk_advance(knee, shin_angle,  LOWER_LEG_LEN);

    /* (3) Foot clamp — don't let the swinging foot punch through ground */
    if (foot.y > r->anchor.ground_y) foot.y = r->anchor.ground_y;

    /* (4) publish to Pose + mark this leg as swing */
    r->pose.knee[i]       = knee;
    r->pose.foot[i]       = foot;
    r->locks.on_ground[i] = false;
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
    Vec2 foot = r->locks.pos[i];
    foot.y    = r->anchor.ground_y;

    r->pose.knee[i]       = solve_ik2(hip, foot, UPPER_LEG_LEN, LOWER_LEG_LEN);
    r->pose.foot[i]       = foot;
    r->locks.on_ground[i] = true;
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

    r->pose.hand[i].x = r->pose.shoulder[i].x + ARM_LEN * sinf(angle);
    r->pose.hand[i].y = r->pose.shoulder[i].y + ARM_LEN * cosf(angle);
}

/* ── §5.6 compute_pose — pose helpers + orchestrator ──────────────── */

/* Vertical hip bob — 2× per stride (pelvic bob is the canonical
 * walking signature; up at toe-off, down at heel-strike). */
static inline float hip_bob_offset(float phi)
{
    return BOB_AMP * sinf(2.0f * phi);
}

/* Lateral hip sway — 1× per stride. Torso leans over whichever foot
 * is currently planted (without this the COM falls outside the
 * support polygon). */
static inline float torso_sway_offset(float phi, float dir)
{
    return SWAY_AMP * cosf(phi) * dir;
}

/* Per-leg phase = global phase + π for the right leg, so the two
 * legs are 180° out of phase (one swinging, one planted). */
static inline float leg_phase(float phi, int i)
{
    return phi + (i == 1 ? (float)M_PI : 0.0f);
}

/* True if a leg's phase puts it in SWING (sin > 0). */
static inline bool leg_is_swinging(float phi_leg)
{
    return sinf(phi_leg) > 0.0f;
}

/* Step 1 — Place pelvis. hip_c is the body's anchor; the two hip
 * joints sit ±HIP_W to either side at the same y. */
static void pose_place_pelvis(Pose *p, float x, float base_y, float bob)
{
    p->hip_c    = (Vec2){ x,                 base_y + bob };
    p->hip_j[0] = (Vec2){ p->hip_c.x - HIP_W, p->hip_c.y  };
    p->hip_j[1] = (Vec2){ p->hip_c.x + HIP_W, p->hip_c.y  };
}

/* Step 2 — torso top + head. Sway shifts the upper body laterally
 * relative to the pelvis. Head sits HEAD_R above torso_top. */
static void pose_place_torso(Pose *p, float sway)
{
    p->torso_top = (Vec2){ p->hip_c.x + sway,   p->hip_c.y - TORSO_LEN };
    p->head_c    = (Vec2){ p->torso_top.x,      p->torso_top.y - HEAD_R };
}

/* Step 3 — shoulders. Placed at torso_top, half-shoulder-width apart. */
static void pose_place_shoulders(Pose *p)
{
    p->shoulder[0] = (Vec2){ p->torso_top.x - SHOULDER_W, p->torso_top.y };
    p->shoulder[1] = (Vec2){ p->torso_top.x + SHOULDER_W, p->torso_top.y };
}

/* Step 5 — one leg's pose: swing (FK) or stance (IK) by phase sign. */
static void pose_place_one_leg(Robot *r, int i, float phi, float dir)
{
    float phi_leg = leg_phase(phi, i);
    if (leg_is_swinging(phi_leg))
        leg_swing_pose (r, i, r->pose.hip_j[i], phi_leg, dir);
    else
        leg_stance_pose(r, i, r->pose.hip_j[i]);
}

/*
 * compute_pose — build the full pose from (x, phase, direction).
 *
 * Reads as a five-step pipeline (top-down through the kinematic
 * hierarchy: each child needs its parent's position):
 *
 *   1. pelvis  (hip_c, hip_j[2])  — bob places the hip vertically
 *   2. torso   (torso_top, head)  — sway shifts upper body laterally
 *   3. shoulders                  — anchored to torso_top
 *   4. arms                       — FK from shoulders, counter-phase
 *   5. legs                       — per leg: FK if swing, IK if stance
 */
static void compute_pose(Robot *r)
{
    Pose *p   = &r->pose;
    float phi = r->gait.phase;
    float dir = (float)r->gait.direction;

    float bob  = hip_bob_offset(phi);
    float sway = torso_sway_offset(phi, dir);

    pose_place_pelvis   (p, r->x, r->anchor.base_hip_y, bob);
    pose_place_torso    (p, sway);
    pose_place_shoulders(p);

    for (int i = 0; i < 2; i++) compute_arm(r, i, phi, dir);
    for (int i = 0; i < 2; i++) pose_place_one_leg(r, i, phi, dir);
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

    float stride       = 2.0f * (UPPER_LEG_LEN + LOWER_LEG_LEN) * sinf(SWING_AMP);
    r->gait.walk_freq  = freq;
    r->gait.walk_speed = freq * stride;
}

/* Recompute screen-derived anchors from the current row count. Called
 * by robot_init() at boot and by robot_tick() every frame in case
 * SIGWINCH changed the row count between frames. */
static void anchor_update_from_screen(GroundAnchor *a, int rows)
{
    a->ground_y   = (float)((rows - GROUND_BOTTOM_ROWS) * CELL_H);
    a->base_hip_y = a->ground_y
                  - (UPPER_LEG_LEN + LOWER_LEG_LEN) * STAND_LEG_RATIO;
}

/* Seed the two foot locks at boot — one step ahead, one step behind
 * x — so the first stance leg has a sensible plant position to
 * anchor IK against. */
static void seed_foot_locks(Robot *r)
{
    float stride_len  = r->gait.walk_speed / r->gait.walk_freq;
    float seed_offset = stride_len * FOOT_LOCK_SEED_FRAC;
    r->locks.pos[0] = (Vec2){ r->x - seed_offset, r->anchor.ground_y };
    r->locks.pos[1] = (Vec2){ r->x + seed_offset, r->anchor.ground_y };
}

/*
 * robot_init — one-shot startup. Pseudocode pipeline:
 *   1. zero everything (so memset-zero fields are explicit defaults)
 *   2. set the pace dial + initial direction + initial phase offset
 *   3. compute screen-derived anchors (ground + rest hip height)
 *   4. place the body at INITIAL_X_FRAC of screen width
 *   5. seed the foot locks ±stride/4 from x
 *   6. build the first pose so frame 0 already renders correctly
 */
static void robot_init(Robot *r, int cols, int rows)
{
    memset(r, 0, sizeof *r);

    /* (2) initial gait + UI defaults */
    robot_set_pace(r, WALK_FREQ_DEFAULT);
    r->gait.direction = +1;
    r->gait.phase     = INITIAL_PHASE_RAD;
    r->ui.show_grid   = true;

    /* (3) screen-derived placement constants */
    anchor_update_from_screen(&r->anchor, rows);

    /* (4) body x */
    r->x = (float)(cols * CELL_W) * INITIAL_X_FRAC;

    /* (5) seed foot locks */
    seed_foot_locks(r);

    /* (6) first pose for frame 0 */
    compute_pose(r);
}

static void robot_reset(Robot *r, int cols, int rows)
{
    /* Preserve user-tuned settings across reset. */
    float freq  = r->gait.walk_freq;
    int   dir   = r->gait.direction;
    bool  grid  = r->ui.show_grid;

    robot_init(r, cols, rows);

    robot_set_pace(r, freq);
    r->gait.direction = dir;
    r->ui.show_grid   = grid;
}

/* ── §5.8 robot_tick — physics helpers + per-frame tick ───────────── */

/* Provisional hip-joint positions for THIS frame's phase, used by
 * touchdown detection BEFORE compute_pose() has run. Same formula
 * as pose_place_pelvis but written into a local rather than Pose. */
static void compute_provisional_hips(const Robot *r, Vec2 hip_j[2])
{
    float bob = hip_bob_offset(r->gait.phase);
    hip_j[0] = (Vec2){ r->x - HIP_W, r->anchor.base_hip_y + bob };
    hip_j[1] = (Vec2){ r->x + HIP_W, r->anchor.base_hip_y + bob };
}

/* Is this leg's phase crossing from swing into stance this tick?
 *   sin(phi_old) > 0   (was swinging)
 *   sin(phi_new) ≤ 0   (now in stance)
 *
 * The crossing event is "touchdown" — the foot is touching the
 * ground for the first time this stride.                             */
static inline bool leg_just_touched_down(float phi_old, float phi_new)
{
    return sinf(phi_old) > 0.0f && sinf(phi_new) <= 0.0f;
}

/* At touchdown, knee_bend ≈ 0 (sin(φ_leg) just crossed zero), so a
 * thigh-only FK gives the foot's landing x. Two LEG_LEN·sin(thigh)
 * terms because UPPER and LOWER both point the same way when the
 * knee is straight. y is always ground_y for a planted foot.        */
static Vec2 fk_landing_foot(Vec2 hip, float thigh_angle, float ground_y)
{
    float foot_x = hip.x
                 + UPPER_LEG_LEN * sinf(thigh_angle)
                 + LOWER_LEG_LEN * sinf(thigh_angle);
    return (Vec2){ foot_x, ground_y };
}

/* Detect touchdown for one leg, and if it happened, capture the new
 * lock position from FK at the (provisional) hip. */
static void leg_detect_touchdown(Robot *r, int i,
                                 float phi_old, Vec2 prov_hip)
{
    float dir       = (float)r->gait.direction;
    float phi_old_l = leg_phase(phi_old,       i);
    float phi_new_l = leg_phase(r->gait.phase, i);

    if (!leg_just_touched_down(phi_old_l, phi_new_l)) return;

    float thigh_angle = -SWING_AMP * cosf(phi_new_l) * dir;
    r->locks.pos[i]   = fk_landing_foot(prov_hip, thigh_angle,
                                        r->anchor.ground_y);
}

/* Toroidal world wrap. When x walks off one edge of the screen
 * (plus a small margin so the bot goes fully off-screen first), it
 * teleports to the other side. Foot locks are shifted with the body
 * so the leg geometry stays continuous across the seam. */
static void wrap_world_position(Robot *r, int cols)
{
    float screen_w = (float)(cols * CELL_W);
    float teleport = screen_w + WRAP_TOTAL_PX;

    if (r->x > screen_w + WRAP_MARGIN_PX) {
        r->x              -= teleport;
        r->locks.pos[0].x -= teleport;
        r->locks.pos[1].x -= teleport;
    }
    if (r->x < -WRAP_MARGIN_PX) {
        r->x              += teleport;
        r->locks.pos[0].x += teleport;
        r->locks.pos[1].x += teleport;
    }
}

/*
 * robot_tick — one frame of "physics". Pseudocode pipeline:
 *
 *   1. early-out if paused (unless step-once)
 *   2. advance phase + body x at the current pace
 *   3. refresh screen-derived anchors (handles SIGWINCH mid-tick)
 *   4. detect per-leg touchdown and capture new lock positions
 *   5. compute full pose for the renderer
 *   6. toroidal world wrap so the bot loops back into view
 */
static void robot_tick(Robot *r, float dt, int cols, int rows)
{
    /* (1) pause gate */
    if (r->ui.paused && !r->ui.step_once) return;
    r->ui.step_once = false;

    /* (2) advance — single phase float drives everything */
    float phi_old = r->gait.phase;
    float dir     = (float)r->gait.direction;
    r->gait.phase += 2.0f * (float)M_PI * r->gait.walk_freq * dir * dt;
    r->x          += r->gait.walk_speed * dir * dt;

    /* (3) refresh anchors in case the screen resized between frames */
    anchor_update_from_screen(&r->anchor, rows);

    /* (4) touchdown detection — uses provisional hips at NEW phase */
    Vec2 prov_hip[2];
    compute_provisional_hips(r, prov_hip);
    for (int i = 0; i < 2; i++)
        leg_detect_touchdown(r, i, phi_old, prov_hip[i]);

    /* (5) full pose for the renderer */
    compute_pose(r);

    /* (6) toroidal world wrap */
    wrap_world_position(r, cols);
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

/* Skip drawing if the segment is shorter than half a pixel — happens
 * during transient zero-length configurations and would otherwise
 * stamp a stray glyph at a degenerate position. */
#define BONE_MIN_LEN_PX  0.5f

/*
 * Bresenham line — rasterise (a.x, a.y) → (b.x, b.y) in pixel space
 * onto character cells, stamping one direction-appropriate glyph
 * along the way. Classic "stepped error term" algorithm:
 *
 *      initial err = |Δrow| − |Δcol|
 *      each step:  advance whichever axis the error currently favours
 *                  and adjust err by the OTHER axis's |Δ|.
 *
 * See Bresenham (1965) in References.
 */
static void draw_bone(Vec2 a, Vec2 b, int cp, attr_t at,
                      int rows, int cols)
{
    /* (1) skip degenerate segments */
    float dxf = b.x - a.x;
    float dyf = b.y - a.y;
    if (sqrtf(dxf*dxf + dyf*dyf) < BONE_MIN_LEN_PX) return;

    /* (2) one glyph for the whole segment, picked from overall slope */
    chtype glyph = bone_glyph(dxf, dyf);

    /* (3) endpoints in cell space, span + step direction per axis */
    int r0 = px_to_cy(a.y), c0 = px_to_cx(a.x);
    int r1 = px_to_cy(b.y), c1 = px_to_cx(b.x);
    int dr = r1 - r0, dc = c1 - c0;
    int step_r = (r0 < r1) ? 1 : -1;
    int step_c = (c0 < c1) ? 1 : -1;
    int abs_dr = abs(dr), abs_dc = abs(dc);

    /* (4) walk the line, decision variable picks which axis advances */
    int err = abs_dr - abs_dc;
    int r = r0, c = c0;
    for (;;) {
        if (in_screen(r, c, rows, cols)) {
            attron (COLOR_PAIR(cp) | at);
            mvaddch(r, c, glyph);
            attroff(COLOR_PAIR(cp) | at);
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs_dc) { err -= abs_dc; r += step_r; }   /* step row */
        if (e2 <  abs_dr) { err += abs_dr; c += step_c; }   /* step col */
    }
}

/* ── §6.3 render_ground — line + optional scrolling grid ─────────── */

/* Solid horizontal underscore line marking the ground surface. */
static void ground_paint_line(int gy, int cols)
{
    attron (COLOR_PAIR(CP_GROUND) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(gy, c, '_');
    attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);
}

/* World column offset to use for the grid pattern, derived from x so
 * tick marks SCROLL with the robot's motion. mod handles negative x
 * (when walking backward). */
static int grid_scroll_offset(float x)
{
    int off = (int)(x / (float)CELL_W) % GRID_PERIOD;
    return off < 0 ? off + GRID_PERIOD : off;
}

/* Scrolling tick marks on the row directly below the ground line,
 * one '|' every GRID_PERIOD cells. The scroll makes the world look
 * like it's moving past while the bot stays roughly centred. */
static void ground_paint_grid(int gy, int cols, float x, int rows)
{
    if (gy + 1 >= rows - 1) return;        /* no room above hint strip */
    int offset = grid_scroll_offset(x);

    attron(COLOR_PAIR(CP_GROUND) | A_DIM);
    for (int c = 0; c < cols; c++) {
        chtype ch = ((c + offset) % GRID_PERIOD == 0) ? '|' : ' ';
        mvaddch(gy + 1, c, ch);
    }
    attroff(COLOR_PAIR(CP_GROUND) | A_DIM);
}

/* render_ground — ground line first, optional scrolling grid below. */
static void render_ground(const Robot *r, int rows, int cols)
{
    int gy = px_to_cy(r->anchor.ground_y);
    if (gy < 0 || gy >= rows) return;

    ground_paint_line(gy, cols);
    if (r->ui.show_grid) ground_paint_grid(gy, cols, r->x, rows);
}

/* ── §6.4 render_robot — body-part painters + driver ──────────────── */

/* Stamp a single character at a pixel-space position if in-screen. */
static void put_glyph(Vec2 pos, chtype glyph, int cp, attr_t at,
                      int rows, int cols)
{
    int cx = px_to_cx(pos.x);
    int cy = px_to_cy(pos.y);
    if (!in_screen(cy, cx, rows, cols)) return;
    attron (COLOR_PAIR(cp) | at);
    mvaddch(cy, cx, glyph);
    attroff(COLOR_PAIR(cp) | at);
}

/* Bright vertical-ish line from hip centre to torso top. */
static void paint_spine(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_c, p->torso_top, CP_BODY, A_BOLD, rows, cols);
}

/* Single-segment arms: shoulder → hand, colour-coded by side. */
static void paint_arms(const Pose *p, int rows, int cols)
{
    draw_bone(p->shoulder[0], p->hand[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->shoulder[1], p->hand[1], CP_RIGHT, A_BOLD, rows, cols);
}

/* Two-segment legs: hip → knee → foot, colour-coded by side. Thigh
 * and shin draw as separate bones so each picks its own glyph. */
static void paint_legs(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_j[0], p->knee[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->knee[0],  p->foot[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->hip_j[1], p->knee[1], CP_RIGHT, A_BOLD, rows, cols);
    draw_bone(p->knee[1],  p->foot[1], CP_RIGHT, A_BOLD, rows, cols);
}

/* Thin connector between the two hip joints — visual cue that they
 * form a single rigid pelvis. */
static void paint_hip_band(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_j[0], p->hip_j[1], CP_BODY_DIM, A_DIM, rows, cols);
}

/* Thin connector between the two shoulders. */
static void paint_shoulder_band(const Pose *p, int rows, int cols)
{
    draw_bone(p->shoulder[0], p->shoulder[1], CP_BODY_DIM, A_DIM, rows, cols);
}

/* Head — single 'O' glyph drawn over the top of any neck/collar line. */
static void paint_head(const Pose *p, int rows, int cols)
{
    put_glyph(p->head_c, 'O', CP_BODY, A_BOLD, rows, cols);
}

/* Per-foot glyph + colour: '#' yellow if planted, '*' cyan if swing.
 * Contact state comes from FootLocks.on_ground (the renderer never
 * re-evaluates sin(φ_leg) — it trusts the physics layer's flag). */
static void paint_feet(const Robot *r, int rows, int cols)
{
    for (int i = 0; i < 2; i++) {
        bool   planted = r->locks.on_ground[i];
        int    cp      = planted ? CP_FOOT_PLANT : CP_FOOT_SWING;
        chtype glyph   = planted ? '#'           : '*';
        put_glyph(r->pose.foot[i], glyph, cp, A_BOLD, rows, cols);
    }
}

/*
 * render_robot — paint the figure in painter's order.
 *
 *   1. spine        — UNDER everything else so arms/legs cross cleanly
 *   2. arms         — left green, right magenta
 *   3. legs         — left green, right magenta (thigh + shin each side)
 *   4. hip band     — dim connector between the two hip joints
 *   5. shoulder band — dim connector between the two shoulders
 *   6. head         — 'O' on top of any neck/collar overlap
 *   7. feet         — '#' planted (yellow) or '*' swing (cyan)
 */
static void render_robot(const Robot *r, int rows, int cols)
{
    const Pose *p = &r->pose;
    paint_spine         (p,    rows, cols);
    paint_arms          (p,    rows, cols);
    paint_legs          (p,    rows, cols);
    paint_hip_band      (p,    rows, cols);
    paint_shoulder_band (p,    rows, cols);
    paint_head          (p,    rows, cols);
    paint_feet          (r,    rows, cols);
}

/* ── §6.5 render_hud — HUD format + paint helpers + driver ────────── */

/* Build the HUD top-row status string. Order chosen so the eye lands
 * on fps first, then live gait values, then leg-phase summary. */
static void hud_format_status(const Robot *r, double fps,
                              char *buf, size_t n)
{
    const char *dir_str  = r->gait.direction == 1 ? "→ forward" : "← reverse";
    const char *legL_str = r->locks.on_ground[0] ? "STANCE" : "swing ";
    const char *legR_str = r->locks.on_ground[1] ? "STANCE" : "swing ";
    const char *run_str  = r->ui.paused ? "PAUSED" : "running";

    snprintf(buf, n,
             " %5.1f fps  freq:%.2f Hz  speed:%5.0f px/s  "
             "dir:%s  legL:%s  legR:%s  %s ",
             fps, r->gait.walk_freq, r->gait.walk_speed,
             dir_str, legL_str, legR_str, run_str);
}

/* Paint the HUD top row (yellow bold) and pad to the right edge so
 * the colour band extends across the whole terminal width. */
static void hud_paint_status_row(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom-row hint strip — lists every interactive key. */
static void hud_paint_hint_strip(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reverse  ↑/↓:speed  .:step  g:grid ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* render_hud — status row 0 + hint strip at rows-1. */
static void render_hud(const Robot *r, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];
    hud_format_status   (r, fps, buf, sizeof buf);
    hud_paint_status_row(buf, cols);
    hud_paint_hint_strip(rows);
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

/*
 * Screen — the current ncurses window dimensions.
 *
 * INTENT
 *   Single point of truth for "how big is the terminal RIGHT NOW".
 *   Every renderer that takes (rows, cols) ultimately reads them
 *   from here via the Scene pointer in main(). SIGWINCH updates
 *   these two ints and the new geometry propagates everywhere.
 *
 * MUTATION RULE
 *   Modified only inside screen_init() and screen_resize(). Read by
 *   the main loop, every renderer, and the physics (for the toroidal
 *   wrap in robot_tick). Never changes mid-frame — physics and
 *   renderer treat them as immutable for the duration of one tick.
 */
typedef struct {
    int cols;                /* terminal width in character cells   */
    int rows;                /* terminal height in character cells  */
} Screen;

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
/* §8  scene — FrameTimer + Scene container; signals, resize, main loop   */
/* ===================================================================== */

/*
 * FrameTimer — wall-clock bookkeeping for the main loop.
 *
 * INTENT
 *   Holds the two timing concerns that used to be loose locals in
 *   main():
 *     (1) measuring dt between frames so robot_tick() advances by
 *         exactly the wall-clock interval (with DT_CAP_SEC guard);
 *     (2) computing a rolling-window fps for the HUD that stays
 *         legible — instantaneous fps fluctuates wildly.
 *
 *   Pulling these into a named struct turns four disconnected locals
 *   into one object the loop body can read and pass around if the
 *   loop ever grows.
 *
 * WHY A 0.5-SECOND ROLLING WINDOW (not instantaneous fps)
 *   Per-frame fps fluctuates wildly — one slow frame reads as
 *   "32 fps", one fast frame as "200 fps". A 0.5-sec smoothing
 *   window gives a value the human eye can actually read. Shorter
 *   window = jumpier; longer = stale.
 *
 * WHY NO SUB-STEPPING (unlike perlin_terrain_bot.c)
 *   The walking gait is pure sin/cos evaluation at the current
 *   phase — there's no forward-Euler integrator that needs small
 *   dt for stability. A large dt just advances the phase by more,
 *   and the next frame still draws the correct pose. The DT_CAP_SEC
 *   guard is enough.
 *
 * ALGORITHM REFERENCE
 *   Glenn Fiedler, "Fix Your Timestep!" (2004) — last_ns is the
 *   "previous frame timestamp" from that article; the dt-cap
 *   pattern in main() is the simpler variant of his accumulator
 *   recipe.
 */
typedef struct {
    int64_t last_ns;         /* clock_ns() reading at the start of *
                              * the previous frame. Each frame:    *
                              *   dt_ns   = now - last_ns          *
                              *   last_ns = now                    *
                              * Reset after resize so the post-    *
                              * resize frame doesn't see a huge    *
                              * dt spike.                          */

    int64_t fps_accum_ns;    /* ns elapsed since the last          *
                              * fps_display update. Resets when    *
                              * it crosses NS_PER_SEC / 2 = 0.5s.  */

    int     fps_frames;      /* count of frames included in        *
                              * fps_accum_ns.                       *
                              *   fps = fps_frames · 1e9            *
                              *      / fps_accum_ns                 */

    double  fps_display;     /* most recent rolling-window fps    *
                              * reading, shown in the HUD top row.*
                              * Updated ≈ 2× per second.          */
} FrameTimer;

/*
 * Scene — the top-level container.
 *
 * INTENT
 *   One single struct owns every long-lived piece of state the
 *   program needs. A single global instance (g_scene) replaces what
 *   used to be a file-scope App. Signal handlers flip the volatile
 *   flags on this instance; main() reads them. Every render and
 *   physics function takes Robot* / Screen* / Scene* explicitly so
 *   data dependencies are visible at the call site — no hidden
 *   globals anywhere else.
 *
 * MEMBERS
 *
 *      Robot       — the simulation state (composed of five
 *                     concept-sized sub-structs; see §5.1.6)
 *      Screen      — ncurses dimensions, refreshed on SIGWINCH
 *      FrameTimer  — dt + rolling-fps state
 *      running     — main-loop continue flag (signal-cleared)
 *      need_resize — SIGWINCH handler sets this
 *
 * WHY A GLOBAL AT ALL
 *   Signal handlers can't take parameters and must be async-signal-
 *   safe (POSIX). They need a path to the running / need_resize
 *   flags. A single file-scope Scene gives them that path without
 *   polluting the rest of the API (which passes Scene* explicitly
 *   everywhere else).
 *
 * INITIALIZATION ORDER (see main())
 *   1. screen_init   — bring ncurses up, learn rows/cols
 *   2. robot_init    — depends on screen dims (for ground_y / x default)
 *   3. timer.last_ns — seed for dt measurement
 */
typedef struct {
    Robot                 robot;       /* simulation state. See      *
                                         * §5.1.6 for composition.    */

    Screen                screen;      /* ncurses dimensions, kept   *
                                         * in sync via SIGWINCH.      */

    FrameTimer            timer;       /* dt + rolling-fps state.    */

    volatile sig_atomic_t running;     /* main-loop continue flag.   *
                                         * Cleared by SIGINT/SIGTERM  *
                                         * (on_exit_signal) or by    *
                                         * 'q' / Q / ESC keys.        *
                                         * volatile + sig_atomic_t   *
                                         * for async-signal safety.   */

    volatile sig_atomic_t need_resize; /* set by SIGWINCH handler    *
                                         * (on_resize_signal). Main   *
                                         * loop reads + clears it    *
                                         * at the top of each frame  *
                                         * and re-syncs screen +     *
                                         * resets the robot.         */
} Scene;

static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Spacebar — pause/resume the gait. */
static void key_pause_toggle(Robot *r) { r->ui.paused = !r->ui.paused; }

/* 'r' — reverse direction. Multiplies dφ/dt and dx/dt so the gait
 * retrogrades smoothly without resetting phase. */
static void key_reverse_direction(Robot *r)
{
    r->gait.direction = -r->gait.direction;
}

/* '+' / '-' / arrows — nudge walk frequency. walk_speed is rederived
 * inside robot_set_pace so stride geometry stays consistent. */
static void key_pace_nudge(Robot *r, float delta)
{
    robot_set_pace(r, r->gait.walk_freq + delta);
}

/* '.' — advance exactly one tick while paused (cleared by tick). */
static void key_step_once(Robot *r) { r->ui.step_once = true; }

/* 'g' — toggle the scrolling ground tick marks. */
static void key_grid_toggle(Robot *r) { r->ui.show_grid = !r->ui.show_grid; }

/*
 * scene_handle_key — dispatch one keystroke to its named action.
 * Each case is one helper call; the switch reads as the keymap.
 */
static void scene_handle_key(Scene *scene, int ch)
{
    Robot *r = &scene->robot;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:  scene->running = 0;            break;
    case ' ':                                key_pause_toggle(r);          break;
    case 'r': case 'R':                      key_reverse_direction(r);     break;
    case '+': case '=': case KEY_UP:         key_pace_nudge(r, +WALK_FREQ_STEP); break;
    case '-': case '_': case KEY_DOWN:       key_pace_nudge(r, -WALK_FREQ_STEP); break;
    case '.':                                key_step_once(r);             break;
    case 'g': case 'G':                      key_grid_toggle(r);           break;
    default:                                                               break;
    }
}

/* Re-sync everything to the new terminal size after SIGWINCH. */
static void scene_handle_resize(Scene *scene)
{
    screen_resize(&scene->screen);
    robot_reset(&scene->robot, scene->screen.cols, scene->screen.rows);
    scene->need_resize   = 0;
    scene->timer.last_ns = clock_ns();   /* avoid huge dt next frame */
}

/* Measure wall-clock dt since last frame; also writes raw dt_ns back
 * to *out_dt_ns for the fps accumulator. Returns dt clamped to
 * DT_CAP_SEC (spiral-of-death guard). */
static float frame_measure_dt(FrameTimer *tm, int64_t now_ns,
                              int64_t *out_dt_ns)
{
    int64_t dt_ns = now_ns - tm->last_ns;
    tm->last_ns   = now_ns;
    *out_dt_ns    = dt_ns;
    float dt = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;
    return dt;
}

/* Tick the rolling-fps accumulator. Recomputes fps_display once the
 * window crosses 0.5 sec (keeps the HUD reading legible to humans). */
static void frame_tick_fps(FrameTimer *tm, int64_t dt_ns)
{
    tm->fps_accum_ns += dt_ns;
    tm->fps_frames++;
    if (tm->fps_accum_ns >= NS_PER_SEC / 2) {
        tm->fps_display  = (double)tm->fps_frames * 1e9
                         / (double)tm->fps_accum_ns;
        tm->fps_accum_ns = 0;
        tm->fps_frames   = 0;
    }
}

/* Drain all queued keystrokes into scene_handle_key. */
static void scene_drain_input(Scene *scene)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_key(scene, ch);
}

/* Sleep the remainder of TICK_NS so we hit TARGET_FPS regardless of
 * how fast this frame ran. now_ns is the frame's start timestamp. */
static void frame_cap_to_target_fps(int64_t now_ns, int64_t tick_ns)
{
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(tick_ns - elapsed);
}

/*
 * main — orchestrator. Reads as the program's lifecycle:
 *
 *   SETUP:
 *     1. install signal handlers + atexit cleanup
 *     2. initialise screen + robot in dependency order
 *     3. seed the frame timer
 *
 *   LOOP (each frame):
 *     1. handle any pending resize
 *     2. measure dt + tick fps
 *     3. drain keyboard input
 *     4. advance physics (robot_tick)
 *     5. draw + present
 *     6. sleep to hit TARGET_FPS
 */
int main(void)
{
    /* SETUP — signals + atexit */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* SETUP — subsystems in dependency order */
    Scene *scene   = &g_scene;
    scene->running = 1;
    screen_init(&scene->screen);
    robot_init (&scene->robot, scene->screen.cols, scene->screen.rows);
    scene->timer.last_ns = clock_ns();

    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    /* LOOP */
    while (scene->running) {
        /* (1) handle resize first so subsequent steps see new geometry */
        if (scene->need_resize) scene_handle_resize(scene);

        /* (2) frame timing — dt for physics, raw dt_ns for fps */
        int64_t now_ns = clock_ns();
        int64_t dt_ns;
        float dt = frame_measure_dt(&scene->timer, now_ns, &dt_ns);
        frame_tick_fps(&scene->timer, dt_ns);

        /* (3) drain queued keystrokes */
        scene_drain_input(scene);

        /* (4) advance physics */
        robot_tick(&scene->robot, dt,
                   scene->screen.cols, scene->screen.rows);

        /* (5) draw + present */
        scene_draw(&scene->robot, scene->timer.fps_display,
                   scene->screen.rows, scene->screen.cols);
        screen_present();

        /* (6) sleep the remainder of the frame budget */
        frame_cap_to_target_fps(now_ns, TICK_NS);
    }

    screen_free(&scene->screen);
    return 0;
}
