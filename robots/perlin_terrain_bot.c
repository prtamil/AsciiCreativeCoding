/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_terrain_bot.c — self-balancing wheel-bot crossing a Perlin landscape
 *
 * DEMO: A two-wheeled robot like a Segway. By itself it would tip over
 *       in milliseconds — gravity is unstable for an inverted
 *       pendulum. Stay-upright is achieved by a PID controller
 *       reading the body's lean angle and pushing the wheels forward
 *       or backward to keep the pendulum balanced. The terrain is
 *       procedurally generated Perlin noise that scrolls past as the
 *       robot drives forward; on slopes the controller adapts its
 *       setpoint so the bot leans into the hill rather than fighting
 *       gravity directly.
 *
 *           ┌────────────────────────────────────────────┐
 *           │     . .  ✦                                 │
 *           │                                            │
 *           │           *      ← top of body (beacon)    │
 *           │            \                               │
 *           │             \                              │
 *           │              \                             │
 *           │             O═O ← wheels (chassis tilts    │
 *           │            /‾‾‾‾\___      with terrain)    │
 *           │       /‾‾‾‾      ‾‾‾\___                   │
 *           │  /‾‾‾‾                ‾‾‾‾\__              │
 *           │##############################              │
 *           └────────────────────────────────────────────┘
 *
 *       Three view modes (toggle with `m`):
 *           TELEMETRY — live values + bar gauges
 *           EQUATIONS — the math with live numbers substituted
 *           PHASE     — phase-space portrait of (θ, ω)
 *
 *       Three real concepts at play:
 *           1. INVERTED PENDULUM — gravity pulls the body away from
 *              vertical. Without control, θ grows exponentially.
 *           2. CART-POLE COUPLING — pushing the wheels horizontally
 *              tilts the body the OTHER way. That's how the bot
 *              corrects itself.
 *           3. PID CONTROL — three-term feedback that produces the
 *              motor force every tick from the body's lean error.
 *
 * Real-world analogues:
 *           Segway PT, Ninebot, hoverboards — same physics
 *           Boston Dynamics Handle (taller pendulum, wheeled base)
 *           Kid balancing a broom on their palm
 *
 * Study alongside:
 *   robots/diff_drive_robot.c             — wheels, no balancing
 *                                            (simpler kinematics).
 *   robots/moving_jump_spring_leg_robot.c — one-legged jumping robot
 *                                            (different actuation,
 *                                             same Perlin terrain).
 *   particle_systems/ragdoll_figure.c     — gravity-driven articulated
 *                                            body without control.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-element semantic pairs
 *   §4  noise        — Noise context (perm table) + Perlin/fBm
 *   §5  terrain      — Terrain (borrows Noise*); height, slope, ring buf
 *   §6  bot          — Pendulum / CartPole / PID / Motion / PhaseTrail /
 *                       BotUI composed into Bot; PID + cart-pole physics
 *   §7  render       — paint terrain, robot, telemetry, equations, phase
 *   §8  screen       — ncurses init / present
 *   §9  scene        — FrameTimer + Scene container; signals, resize,
 *                       variable-dt main loop with substeps
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                reset bot + new terrain
 *   ↑ / ↓            change drive speed
 *   p                toggle PID controller (watch the bot fall)
 *   m                cycle view (TELEMETRY / EQUATIONS / PHASE)
 *   g                cycle gain preset (BALANCED / HIGH-Kp / NO-Kd / NO-Ki)
 *   + / -            tweak Kp manually
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/perlin_terrain_bot.c \
 *       -o perlin_terrain_bot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three layers stacked together.
 *
 *                 (A) PERLIN TERRAIN — 5-octave fBm gives a smoothly
 *                     rolling height function h(x). The bot's
 *                     world_x advances at drive_spd; we sample
 *                     h(world_x) for the wheel contact and use a
 *                     central-difference slope α as a feed-forward
 *                     hint to the controller.
 *
 *                 (B) INVERTED PENDULUM — the body is a rigid rod
 *                     of length L, mass m_p, hinged at the wheel
 *                     axle. Its state is (θ, ω) — angle from
 *                     vertical and angular velocity. Gravity tips
 *                     it: τ_gravity = m·g·L·sin(θ). Without
 *                     control, θ runs away exponentially.
 *
 *                 (C) PID CONTROLLER — every tick reads the lean
 *                     error e = θ − θ_ref, computes
 *                          F = Kp·e + Ki·∫e + Kd·de/dt
 *                     and applies F as horizontal motor force on
 *                     the cart (axle). Cart-pole coupling tilts
 *                     the body the other way; the controller's job
 *                     is to keep θ near 0 despite gravity.
 *
 *                 The setpoint θ_ref is shifted by the terrain slope
 *                 (θ_ref = -α · SLOPE_FEED) so the bot leans into the
 *                 hill — the slope pushes the body, and we ask the
 *                 controller to expect that.
 *
 * Data-structure: A Scene owns the entire program state, composed of
 *                 five concept-sized pieces:
 *
 *                   Noise       — Perlin permutation table (was a global)
 *                   Terrain     — heights ring buffer, borrows Noise*
 *                   Bot         — six concept-sized sub-structs:
 *                       Pendulum   the unknowns (θ, ω)
 *                       CartPole   derived dynamics (θ_eff, ẍ, θ̈, M_eff, F)
 *                       PID        gains, integrator, setpoint, term outs
 *                       Motion     world_x, drive_spd, slope α, dist_m
 *                       PhaseTrail ring buffer of last (θ, ω) samples
 *                       BotUI      paused, fallen, view, preset_idx
 *                   Screen      — ncurses dimensions
 *                   FrameTimer  — dt + rolling-fps bookkeeping
 *
 *                 Each sub-struct names ONE concept the reader needs.
 *                 The Bot type used to be 30 flat fields; the same
 *                 data now reads as "balancing has six pieces, here
 *                 they are." Terrain is still a TBUF-entry ring buffer
 *                 of heights indexed by world column.
 *
 * Rendering     : Painter's order — last write wins:
 *                   (1) Sky (with sparse stars)
 *                   (2) Terrain — surface row + texture row + rock
 *                       fill below
 *                   (3) Robot — chassis, wheels, body line, beacon
 *                   (4) Right-side panel — telemetry / equations /
 *                       phase portrait depending on view mode
 *                   (5) HUD row 0 (yellow) + hint row last (cyan)
 *
 * Performance   : Per frame: O(cols) for terrain, O(1) for robot,
 *                 O(HIST_LEN) for phase trail. Sub-stepping in the
 *                 physics integrator keeps the controller stable
 *                 even at low frame rates: each frame splits dt
 *                 into N substeps of size ≤ TICK_DT_TARGET = 1/120
 *                 sec, and each substep runs one integration step.
 *
 * References    : grouped by the concepts this file teaches. Ten
 *                  entries — one or two strong picks per topic. Free
 *                  PDFs called out where the author has posted them.
 *
 *   ── Feedback control & PID  (concepts (B), (C); §6 pid_step) ─────
 *   Åström & Murray, "Feedback Systems: An Introduction for
 *     Scientists and Engineers" (Princeton, 2008). Free PDF at
 *     https://fbsbook.org . The modern intro to feedback — explains
 *     WHY closing the loop on error works. Read ch. 1, 3, 11.
 *   Åström & Hägglund, "Advanced PID Control" (ISA, 2006). The
 *     reference text on PID tuning, anti-windup, and feed-forward.
 *     Every controller you write afterwards is better.
 *
 *   ── Cart-pole dynamics  (concept (B); §6.4 cart_pole_step) ───────
 *   Spong, Hutchinson, Vidyasagar, "Robot Modeling and Control"
 *     (Wiley, 2006). Lagrangian derivation of the cart-pole
 *     equations of motion that §6.4 implements. Read §6.5.
 *
 *   ── Phase space & dynamical systems  (§7.6 phase portrait) ───────
 *   Strogatz, "Nonlinear Dynamics and Chaos" (Westview, 2nd ed,
 *     2014). The standard intro to phase portraits, limit cycles,
 *     and stability. Chapters 5–6 most relevant; the trajectories
 *     drawn there look exactly like our (θ, ω) trail.
 *
 *   ── Perlin noise & procedural terrain  (concept (A); §4) ─────────
 *   Perlin, "An Image Synthesizer" (SIGGRAPH 1985). The original
 *     gradient-noise paper — short, precise, foundational.
 *   Perlin, "Improving Noise" (SIGGRAPH 2002). Introduces the C²
 *     quintic fade  fade(t) = 6t⁵ − 15t⁴ + 10t³  used in §4
 *     (replacing the 1985 cubic to eliminate derivative kinks).
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   Ebert, Musgrave, Peachey, Perlin, Worley, "Texturing &
 *     Modeling: A Procedural Approach" (Morgan Kaufmann, 3rd ed,
 *     2002). The procedural-textures playbook — fBm, 1/f spectrum,
 *     terrain synthesis. §4's fbm() follows the recipe in ch. 16.
 *
 *   ── Rendering  (§7) ──────────────────────────────────────────────
 *   Bresenham, "Algorithm for computer control of a digital plotter"
 *     (IBM Systems Journal 4(1):25–30, 1965). The line-drawing
 *     algorithm implemented in draw_line(). Unusually readable
 *     original paper.
 *   Padala, "NCURSES Programming HOWTO" (The Linux Documentation
 *     Project). Practical reference for the ncurses API used in
 *     §3 (color_init) and §7 (put_ch, put_str, draw_line, draw_bar).
 *     https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *   ── Game loop & numerical timing  (§9 main loop) ─────────────────
 *   Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004). The
 *     canonical write-up of the accumulator + sub-step pattern
 *     in main() — explains why splitting dt into ≤ TICK_DT_TARGET
 *     sub-steps keeps the integrator stable at any frame rate.
 *     https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three independent ideas working together:
 *   1. Gravity wants to tip the bot over — it's an inverted pendulum
 *      and that's fundamentally unstable.
 *   2. Pushing the wheels forward/backward makes the body lean the
 *      OTHER way — that's the cart-pole coupling that makes balancing
 *      possible.
 *   3. A PID controller reads the lean error and computes the right
 *      amount of force to push, every single tick. Slope-aware: the
 *      setpoint shifts so the bot leans into the hill.
 *
 *
 * ANALOGY: BALANCING A BROOM ON YOUR HAND
 * ───────────────────────────────────────
 *
 *   When you balance a broom upright on your palm, you don't push UP
 *   on the broom — you slide your hand SIDEWAYS. If the broom tilts
 *   right, you slide your hand right (faster than the broom is
 *   tipping). That accelerates the broom's base rightward, which
 *   makes the top rotate LEFT (relative to the base), which corrects
 *   the lean.
 *
 *   Your hand = the cart (wheel). The broom = the pendulum. Your
 *   eyes-and-arm = the PID controller, reading the broom's tilt and
 *   computing where to slide your hand.
 *
 *   The bot does exactly this, except instead of "where to slide my
 *   hand" the controller outputs "what motor force to apply to the
 *   wheel". Same physics.
 *
 *
 * GEOMETRY DIAGRAM
 * ────────────────
 *
 *      (gravity ↓)                   * ← top of body (mass m_p)
 *                                   /
 *                                  /  ← rod of length L
 *                                 /    pendulum state: (θ, ω)
 *                                /     θ  = angle from vertical
 *                               /      ω  = θ̇  = angular velocity
 *                              /
 *                       O═════O ← cart (wheels)
 *                       │  ↑
 *                       │  F = motor force from PID (horizontal)
 *                       │
 *                       └──→ x ← cart position
 *                            ẍ = horizontal acceleration
 *
 *      The CRUCIAL coupling:
 *        • Pushing the cart RIGHT (ẍ > 0) tilts the body LEFT.
 *        • Pushing the cart LEFT  (ẍ < 0) tilts the body RIGHT.
 *      That's how the controller corrects lean — by accelerating
 *      the BASE in the same direction as the lean (so the top is
 *      "left behind" and rotates back toward vertical).
 *
 *
 * THE CART-POLE EQUATIONS (Lagrangian)
 * ────────────────────────────────────
 *
 *      M_eff = M_cart + m_pole · sin²θ_eff
 *
 *      ẍ      = (F + m_pole · sin θ_eff · (L · ω² − g · cos θ_eff)) / M_eff
 *
 *      θ̈      = (g · sin θ_eff − ẍ · cos θ_eff) / L
 *
 *      θ_eff = θ + α   (α = terrain slope; gives "lean from gravity",
 *                        not "lean from chassis")
 *
 *      Then forward Euler:
 *          ω ← ω + θ̈ · dt
 *          θ ← θ + ω · dt
 *
 *      Why M_eff = M_cart + m_pole · sin²θ_eff?
 *      Some of the pole's mass effectively belongs to the cart's
 *      inertia (the part being dragged horizontally as the pole
 *      rotates). That fraction is sin²θ — zero at θ = 0 (pole
 *      perfectly upright; cart inertia is just M_cart) and 1 at
 *      θ = 90° (pole horizontal; the entire pole is being shoved
 *      horizontally).
 *
 *
 * PID CONTROLLER — block diagram
 * ──────────────────────────────
 *
 *      θ_ref ──┐                   ┌────────────────┐
 *              │  e = θ − θ_ref    │     Kp · e     │  P term
 *      θ ──────┼─►  error  ─────►  │  + Ki · ∫e     │  I term
 *                                  │  + Kd · de/dt  │  D term
 *                                  └────────┬───────┘
 *                                           ▼  F
 *                                  ┌────────────────┐
 *                                  │  Cart-pole     │
 *                                  │  dynamics      │
 *                                  └────────┬───────┘
 *                                           ▼
 *                              new θ, ω ────┘ (closes the loop)
 *
 *      • P  (Kp · e)    — instant stiffness. "Tilt 0.1 rad? Apply 12 N back."
 *                         Larger Kp → faster correction but more overshoot.
 *      • I  (Ki · ∫e)   — drift correction. On a slope a fixed lean accumulates
 *                         in the integral; Ki sums that error and cancels the
 *                         steady-state offset. Anti-windup clamps the integral
 *                         so it can't grow unbounded.
 *      • D  (Kd · de/dt) — damping. Resists rapid changes in error,
 *                         brakes oscillation. Without Kd, the system
 *                         oscillates forever (underdamped).
 *
 *
 * SLOPE FEED-FORWARD
 * ──────────────────
 *
 *   On a slope of α radians, gravity pulls the body away from
 *   chassis-perpendicular. To stay upright with respect to gravity,
 *   the body must LEAN into the slope. We tell the controller to
 *   target a tilted setpoint:
 *
 *       θ_ref = -SLOPE_FEED · α
 *
 *   With SLOPE_FEED = 0.65, on a 20° slope (α ≈ 0.35 rad), the
 *   setpoint becomes ≈ -13°. The bot's body leans 13° into the
 *   slope, perfectly upright with respect to gravity (modulo the
 *   feed-forward gain).
 *
 *   Without slope feed-forward, only the integral term would
 *   correct slope drift — too slowly to follow rapidly changing
 *   terrain. With feed-forward, slope is mostly handled
 *   pre-emptively and Ki only mops up small residuals.
 *
 *
 * PHASE PORTRAIT
 * ──────────────
 *
 *   Plotting (θ, ω) over time on a 2-D plane shows the controller's
 *   character:
 *
 *      ω ↑
 *        │           UNDERDAMPED (small Kd)
 *        │     ╭─.─╮            wide circles
 *        │    ╱     ╲           that slowly shrink
 *        │   ╱  ●    ╲
 *        │  ╱         ╲    →
 *        │ ╱           ╲
 *        │╱             ╲       STABLE (good Kd)
 *        ┼────────────────────  tight inward spiral
 *        │                ╲     converges to (0, 0)
 *        │                 ╲
 *        │  OVERDAMPED      ╲   →  ────────────●
 *        │  (large Kd)              slow direct
 *        │                          path to origin
 *        │                                          → θ
 *
 *   In the demo's phase view you'll see the actual trajectory.
 *   Press 'g' to cycle gains and watch how the trajectory shape
 *   changes — that's a tuning lesson by direct observation.
 *
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Measure dt = wall-clock seconds since last frame.
 *  2. Compute n_substeps = ceil(dt / TICK_DT_TARGET), capped at 16.
 *     This keeps the integrator stable when frames take longer
 *     than the physics timestep.
 *  3. For each substep:
 *       a. Advance world_x by drive_spd · sub_dt.
 *       b. Sample terrain slope α at the new x.
 *       c. Compute θ_eff = θ + α and θ_ref = -SLOPE_FEED · α.
 *       d. PID:
 *           e = θ − θ_ref
 *           ∫e += e · sub_dt   (clamped by anti-windup)
 *           de/dt = (e − e_prev) / sub_dt
 *           F = Kp·e + Ki·∫e + Kd·de/dt    (clamped to ±F_MAX)
 *       e. Cart-pole physics:
 *           M_eff = M_cart + m_pole · sin²θ_eff
 *           ẍ     = (F + m_pole·sin·(L·ω² − g·cos)) / M_eff
 *           θ̈     = (g·sin θ_eff − ẍ·cos θ_eff) / L
 *           ω    += θ̈ · sub_dt
 *           θ    += ω · sub_dt
 *       f. Append (θ, ω) to phase-history ring buffer.
 *       g. If |θ_eff| > FALL_ANGLE → mark as fallen; stop ticking.
 *  4. erase()
 *  5. Render terrain → robot → side panel (telemetry / equations /
 *     phase) → HUD row 0 + hint strip.
 *  6. doupdate; sleep to TARGET_FPS.
 *
 *
 *
 * Background you need
 * ───────────────────
 *   - diff_drive_robot T2 (Euler integration of pose).
 *   - Newton's second law, gravity as constant downward
 *     acceleration.
 *   - Trig: sin / cos of small angles.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Lagrangian / Hamiltonian mechanics. The cart-pole
 *     equations are presented; we don't derive them from
 *     first principles.
 *   - Optimal control (LQR, MPC). PID is a much simpler
 *     special case.
 *   - State-space / pole placement. PID is "transfer-function
 *     control" — works without ever writing down a state-space
 *     model.
 *   - Real-time embedded programming. Single-threaded soft
 *     real-time at 60 fps is plenty.
 *
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

/* ── §1.1 frame rate + sub-step target ────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/*
 * TICK_DT_TARGET — desired physics step. Each frame is split into
 * sub-steps no larger than this so the cart-pole integrator stays
 * stable even at slow frame rates. 1/120 sec is small enough for
 * the default gains to be well within the stability region.
 *
 * MAX_SUBSTEPS — cap so a long-stalled frame doesn't loop forever.
 */
#define TICK_DT_TARGET   (1.0f / 120.0f)
#define MAX_SUBSTEPS     16

/* ── §1.2 cell pixel dimensions ──────────────────────────────────── */
/* Terminal cells are roughly 2× tall as wide (8 × 16 px). Physics
 * is in pixel space; cell conversion happens at draw time. */
#define CELL_W   8
#define CELL_H  16

/* ── §1.3 cart-pole physics (SI) ─────────────────────────────────── */
/*
 * GRAVITY   — m/s²; standard Earth value.
 * PEND_LEN  — m; rod length from axle to body centre-of-mass.
 *             Longer rod = slower oscillation (τ ∝ √(L/g)).
 * MASS_CART — kg; effective mass of chassis + wheels.
 * MASS_POLE — kg; mass of the body above the axle.
 * MAX_FORCE — N; motor force clamp (saturation).
 *             80 N keeps "everything bounded" for our default
 *             pendulum geometry.
 * FALL_ANGLE — rad; "fallen over" threshold (~60°). At larger
 *              angles even unbounded F couldn't recover quickly,
 *              so we just stop ticking.
 */
#define GRAVITY      9.81f
#define PEND_LEN     1.00f
#define MASS_CART    4.00f
#define MASS_POLE    2.00f
#define MAX_FORCE  200.00f
#define FALL_ANGLE   1.05f

/* ── §1.4 PID defaults + anti-windup ─────────────────────────────── */
/*
 * KP_DEF — proportional gain. Larger = faster correction, more overshoot.
 * KI_DEF — integral gain. Larger = faster drift removal but risk of windup.
 * KD_DEF — derivative gain. Larger = more damping, slower response.
 * WINDUP_MAX — clamp on the integrator's accumulated value (rad·sec).
 *             Prevents unbounded growth on sustained errors.
 */
#define KP_DEF      120.0f
#define KI_DEF        0.20f
#define KD_DEF       18.0f
#define WINDUP_MAX    5.00f

/* ── §1.5 slope feed-forward ─────────────────────────────────────── */
/*
 * SLOPE_FEED — fraction of terrain slope α fed to θ_ref.
 *   1.0 cancels slope perfectly in theory but amplifies high-frequency
 *   slope changes; 0.65 leaves headroom for the PID to smooth residuals.
 */
#define SLOPE_FEED   0.65f

/* ── §1.6 robot geometry (pixels) ────────────────────────────────── */
#define WHEEL_R      18.0f      /* wheel radius (only used for axle height) */
#define AXLE_HW      26.0f      /* half the axle width (wheel separation) */
#define BODY_H       96.0f      /* body length axle → top mass */

/* ── §1.7 drive speed cycle ──────────────────────────────────────── */
#define DRIVE_DEF    55.0f
#define DRIVE_STEP   15.0f
#define DRIVE_MAX   160.0f
#define DRIVE_MIN     0.0f
#define PIX_PER_M   100.0f     /* HUD scale: pixels per metre  */

/* ── §1.8 terrain ─────────────────────────────────────────────────── */
/*
 * TBUF       — power of two; ring-buffer size for terrain heights.
 * T_FREQ     — Perlin frequency per world column. Smaller = bigger hills.
 * T_AMP_F    — amplitude as fraction of screen height.
 * T_MID_F    — vertical placement of the terrain centre line.
 */
#define TBUF       1024
#define TMASK      (TBUF - 1)
#define T_FREQ      0.022f
#define T_AMP_F     0.20f
#define T_MID_F     0.62f

/* ── §1.9 phase-history ring buffer ─────────────────────────────── */
enum { HIST_LEN = 240 };

/* ── §1.10 view modes ────────────────────────────────────────────── */
typedef enum {
    VIEW_TELEMETRY = 0,
    VIEW_EQUATIONS,
    VIEW_PHASE,
    VIEW_COUNT
} ViewMode;

static const char *VIEW_NAMES[VIEW_COUNT] = {
    "TELEMETRY", "EQUATIONS", "PHASE-SPACE"
};

/* ── §1.11 dt cap (spiral-of-death guard) ────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.12 ncurses pair IDs ──────────────────────────────────────── */
enum {
    /* 1..2 — sky & stars */
    CP_SKY      = 1,
    CP_STAR,
    /* 3..4 — terrain */
    CP_SURF,
    CP_ROCK,
    /* 5..7 — robot */
    CP_CHASSIS,
    CP_WHEEL,
    CP_BEACON,
    /* 8..10 — UI */
    CP_DIM,             /* dim text                            */
    CP_GOOD,            /* green positive indicator            */
    CP_WARN,            /* red warning                         */
    /* 11..12 — value text */
    CP_VAL,             /* live numeric values                 */
    CP_EQ,              /* equation labels                     */
    /* 13..14 — bar gauge */
    CP_BAR_POS,
    CP_BAR_NEG,
    /* 15..16 — HUD spec */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.13 timing primitives ─────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.14 HUD layout ────────────────────────────────────────────── */
#define HUD_BUF_LEN  160
#define PANEL_W       32

/* ── §1.15 PID tuning & initial conditions ───────────────────────── */

/* Tiny seed perturbation applied to θ in bot_init / bot_reset. Small
 * enough that the bot is "almost upright" at start, large enough
 * that the controller has a non-zero error to push against on tick 1.
 * 0.04 rad ≈ 2.3°.                                                   */
#define INITIAL_LEAN_RAD     0.04f

/* PID derivative numerical floor — when dt < this we skip the
 * derivative term to avoid 0/0. Below ~1 ns of dt the noise in the
 * subtraction would swamp any real signal anyway.                    */
#define PID_DT_FLOOR         1e-9f

/* Manual Kp tuning via '+' / '-' keys.                              */
#define KP_NUDGE             5.0f    /* keypress step                */
#define KP_MAX_USER        400.0f    /* clamp ceiling                */

/* ── §1.16 telemetry / HUD display ranges ────────────────────────── */

/* Bar gauge geometry (cells), inside each telemetry row.            */
#define BAR_WIDTH            14      /* total bar cells (centred)    */
#define BAR_COL_OFFSET       16      /* bar's left edge relative c0  */

/* Bar full-scale values — what 100% deflection means in physical
 * units. Pick these to match the typical "alarming" extreme of each
 * quantity so the bar visually saturates exactly when the regime
 * becomes interesting.                                              */
#define THETA_BAR_RANGE_DEG  35.0f   /* lean angle ±°                */
#define OMEGA_BAR_RANGE       6.0f   /* angular vel ± rad/sec        */
#define SLOPE_BAR_RANGE_DEG  25.0f   /* terrain slope ±°             */
#define D_BAR_FRAC            0.4f   /* D bar fills at this·MAX_FORCE*/
#define SAT_WARN_FRAC         0.85f  /* F warns red past this·F_MAX  */

/* Colour-change thresholds for live readouts (degrees).             */
#define THETA_WARN_DEG       20.0f   /* lean turns red past this     */
#define SLOPE_WARN_DEG       15.0f   /* slope turns red past this    */

/* ── §1.17 phase-portrait plot ───────────────────────────────────── */

/* Plot box in character cells, centred inside the side panel.       */
#define PHASE_PLOT_W         29
#define PHASE_PLOT_H         12

/* Physical full-range of the plot axes — values outside these clip. */
#define PHASE_TH_RANGE        0.6f   /* x-axis half-range, rad       */
#define PHASE_OM_RANGE        5.0f   /* y-axis half-range, rad/sec   */

/* Trail age bands — newest 15% red, next 35% yellow, oldest 50%
 * dim grey. Encodes time direction in a static portrait.            */
#define TRAIL_NEW_BAND        0.85f
#define TRAIL_MID_BAND        0.50f

/* PID regime classification thresholds — read by phase_classify_*. */
#define KD_UNDERDAMP_LIMIT    1.0f   /* Kd below this → underdamped  */
#define KD_OVERDAMP_LIMIT    50.0f   /* Kd above this → overdamped   */
#define CONVERGED_ERR_RAD     0.03f  /* |e| below this → STABLE      */

/* ── §1.18 render misc ───────────────────────────────────────────── */

/* Beacon (body top) pulse — flips bright/dim every 1/(2·HZ) seconds
 * based on dist_m so the pulse is travel-paced, not wall-clock-paced
 * (slows when bot stops; makes the visual link bot↔terrain stronger).*/
#define BEACON_PULSE_HZ       4.0f

/* Terrain surface glyph threshold — when |Δh| over one cell exceeds
 * this FRACTION of CELL_W, the surface uses '/' or '\\' instead of
 * '_' to show the slope. (A slope of 0.22 ≈ 12.4°.)                  */
#define SLOPE_GLYPH_FRAC      0.22f

/* Lean readout label — printed this many cells LEFT of the axle so
 * "+12.3°" doesn't overlap the bot itself.                          */
#define LEAN_LABEL_DC         6

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
 * Each visual element gets one fixed semantic colour. Layout:
 *
 *   sky background      dark blue
 *   stars               bright yellow (sparse)
 *   terrain surface     bright green
 *   terrain rock fill   dim green
 *   robot chassis       white
 *   robot wheel         cyan
 *   beacon              red (warning-ish; tip of body, easy to spot)
 *   dim text            grey
 *   stable indicator    bright green
 *   warning indicator   bright red
 *   live values         light yellow
 *   equation labels     light cyan
 *   bar pos             cyan
 *   bar neg             magenta
 *
 * HUD pairs (PAIR_HUD, PAIR_HINT) bind separately on default
 * terminal background per CLAUDE.md HUD spec.
 */
/*
 * Palette table — each row is one role mapped to its 256-mode and
 * 8-mode foreground codes. color_init() walks this once with the
 * correct column selected at runtime. Reading the table top-to-
 * bottom gives the full palette in one glance.
 */
typedef struct {
    short pair;         /* CP_* / PAIR_* id from §1.12               */
    short fg256;        /* foreground when COLORS >= 256             */
    short fg8;          /* foreground in 8-colour fallback           */
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair         256-mode  8-mode-fallback   role                 */
    { CP_SKY,         25,     COLOR_BLUE     }, /* dark blue          */
    { CP_STAR,       226,     COLOR_YELLOW   }, /* yellow             */
    { CP_SURF,        46,     COLOR_GREEN    }, /* bright green       */
    { CP_ROCK,        28,     COLOR_GREEN    }, /* dim green          */
    { CP_CHASSIS,    255,     COLOR_WHITE    }, /* near-white         */
    { CP_WHEEL,       51,     COLOR_CYAN     }, /* cyan               */
    { CP_BEACON,     196,     COLOR_RED      }, /* red                */
    { CP_DIM,        244,     COLOR_WHITE    }, /* grey               */
    { CP_GOOD,        82,     COLOR_GREEN    }, /* lime               */
    { CP_WARN,       196,     COLOR_RED      }, /* red                */
    { CP_VAL,        229,     COLOR_YELLOW   }, /* pale yellow        */
    { CP_EQ,         159,     COLOR_CYAN     }, /* light cyan         */
    { CP_BAR_POS,     51,     COLOR_CYAN     }, /* cyan               */
    { CP_BAR_NEG,    201,     COLOR_MAGENTA  }, /* magenta            */
    { PAIR_HUD,      226,     COLOR_YELLOW   }, /* yellow on default  */
    { PAIR_HINT,      51,     COLOR_CYAN     }, /* cyan on default    */
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
/* §4  noise — Noise context, Perlin 1-D, fBm                             */
/* ===================================================================== */

/*
 * Noise — Perlin gradient-noise context.
 *
 * INTENT
 *   Wrap the 512-entry permutation table so terrain generation has
 *   an explicit, addressable context object rather than a hidden
 *   file-scope global. Every caller of perlin1() and fbm() holds a
 *   pointer to one; the dependency is visible at every call site.
 *
 * WHY A SEPARATE TYPE
 *   The 1985 Perlin paper presents noise as a pure function noise(x),
 *   but every real implementation needs *some* seeded state — the
 *   permutation table pins down WHICH noise function we got. Hiding
 *   it in a global made the dependency invisible and only one noise
 *   field was ever possible; making it a struct lets Scene own it
 *   (and lets a future demo instantiate independent noise sources
 *   for, e.g., terrain + clouds).
 *
 * INVARIANT
 *   perm[i] == perm[i & 255]  for i in [256, 512). The table is
 *   the permutation of [0..255] under a Fisher-Yates shuffle, then
 *   DOUBLED so the integer-lattice lookup in perlin1() — which
 *   reads perm[xi] and perm[xi + 1] with xi in [0, 255] — never
 *   wraps and needs no branch.
 *
 * SMOOTHING
 *   We use Perlin's 2002 quintic fade  fade(t) = 6t⁵ − 15t⁴ + 10t³
 *   rather than the 1985 cubic. The quintic is C²-continuous, so
 *   the resulting surface has continuous CURVATURE — no visible
 *   kinks where derivatives would otherwise jump.
 *
 * ALGORITHM REFERENCES
 *   Perlin (1985), "An Image Synthesizer", SIGGRAPH — original.
 *   Perlin (2002), "Improving Noise", SIGGRAPH — the C² quintic fade
 *     used here. https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   Ebert et al. (2002), "Texturing & Modeling", ch. 16 — fBm wrap.
 */
typedef struct {
    unsigned char perm[512];   /* perm[0..255]   = Fisher-Yates of   *
                                * [0..255], seeded by srand(seed)    *
                                * inside noise_init(); changes only  *
                                * when noise_init() is called.       *
                                *                                    *
                                * perm[256..511] = exact duplicate   *
                                * of [0..255] so a lattice lookup    *
                                * `perm[xi + 1]` with xi in [0,255]  *
                                * never wraps and needs no branch.   */
} Noise;

static void noise_init(Noise *n, unsigned int seed)
{
    unsigned char p[256];
    srand(seed);
    for (int i = 0; i < 256; i++) p[i] = (unsigned char)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = p[i]; p[i] = p[j]; p[j] = t;
    }
    for (int i = 0; i < 512; i++) n->perm[i] = p[i & 255];
}

static inline float fade (float t) { return t*t*t*(t*(t*6.0f - 15.0f) + 10.0f); }
static inline float lerp (float a, float b, float t) { return a + t * (b - a); }
static inline float grad1(int h, float x) { return (h & 1) ? x : -x; }

static float perlin1(const Noise *n, float x)
{
    int   xi = (int)floorf(x) & 255;
    float xf = x - floorf(x);
    return lerp(grad1(n->perm[xi],     xf       ),
                grad1(n->perm[xi + 1], xf - 1.0f),
                fade(xf));
}

/*
 * Fractional Brownian Motion — sum of 5 perlin octaves at increasing
 * frequency and decreasing amplitude. Result is in [-1, +1] roughly.
 * Each octave doubles frequency and halves amplitude — the classic
 * 1/f spectrum that gives nature-like terrain.
 */
static float fbm(const Noise *n, float x)
{
    float v = 0.0f, amp = 0.5f, freq = 1.0f;
    for (int i = 0; i < 5; i++) {
        v   += amp * perlin1(n, x * freq);
        amp *= 0.5f;
        freq *= 2.0f;
    }
    return v;
}

/* ===================================================================== */
/* §5  terrain — height query, slope, ring buffer                         */
/* ===================================================================== */

/*
 * Terrain — scrolling 1-D heightfield, indexed by world column.
 *
 * INTENT
 *   Cache height samples for every world column the bot has reached
 *   so we never recompute fBm() for a column already drawn. The bot
 *   moves right; new columns are lazily generated by terrain_ensure()
 *   and stored in a power-of-two ring buffer indexed by  wc & TMASK.
 *
 * WHY A RING BUFFER (not a growing array)
 *   The bot's world_x grows unbounded, but the screen shows only
 *   ~cols columns at any moment. Phase-1 rule "no malloc after init"
 *   forbids a growing buffer; the ring with TBUF=1024 columns is
 *   sized so the visible window always lies inside it. Old columns
 *   get overwritten as wc wraps — and we never look at them again
 *   because the camera has moved on.
 *
 * WHY h[] IS PIXEL-SPACE
 *   Physics is in pixel space (CELL_W × CELL_H sub-pixels per
 *   character cell). Storing the surface in the same units lets the
 *   integrator query  h[wc & TMASK]  directly without a coordinate
 *   transform; the px → cell conversion happens once at draw time
 *   in §7 via px_to_cy().
 *
 * WHY noise IS BORROWED, NOT OWNED
 *   Scene owns the single Noise instance. Terrain only reads it.
 *   Marking the pointer  const Noise *  documents both "Terrain
 *   reads noise, never modifies it" and "Terrain does not free
 *   noise — that's Scene's job".
 *
 * GENERATION ALGORITHM
 *   h[wc & TMASK] = mid + fbm(noise, wc · T_FREQ) · amp
 *      mid = rows · T_MID_F · CELL_H        (vertical placement)
 *      amp = rows · T_AMP_F · CELL_H        (peak-to-peak / 2)
 *   fbm = sum of 5 Perlin octaves at increasing freq / halving amp
 *      → "pink-noise" 1/f spectrum that LOOKS like natural terrain.
 *
 * ALGORITHM REFERENCES
 *   Ebert et al. (2002), "Texturing & Modeling", ch. 16 — fBm,
 *     1/f spectrum, terrain synthesis recipe.
 *   Mandelbrot (1982), "The Fractal Geometry of Nature" — origin
 *     of the fBm concept and its self-similar character.
 */
typedef struct {
    const Noise *noise;     /* borrowed; Scene owns the instance.    *
                              * Read by terrain_h_at() → fbm().       *
                              * Never freed by Terrain.               */

    float h[TBUF];          /* surface height per world column, in   *
                              * pixels measured DOWN from screen top  *
                              * (larger h → lower point on screen).   *
                              * Indexed by  wc & TMASK ; TBUF = 1024. *
                              * Slot c is valid iff c <= gen_col.     *
                              * Filled lazily by terrain_ensure().    */

    int   gen_col;          /* highest world column already populated.*
                              * terrain_ensure() fills (gen_col, upto].*
                              * Init = -1 (no columns yet); grows     *
                              * monotonically across the run.         */

    int   rows;             /* current terminal height in cells.     *
                              * Needed for the amplitude/midline      *
                              * formula above. Updated only when      *
                              * SIGWINCH arrives — never per-frame.   */
} Terrain;

/* World y of the surface at a given world column. */
static float terrain_h_at(const Terrain *t, int wc)
{
    float mid = (float)t->rows * T_MID_F * (float)CELL_H;
    float amp = (float)t->rows * T_AMP_F * (float)CELL_H;
    return mid + fbm(t->noise, (float)wc * T_FREQ) * amp;
}

/*
 * Terrain slope at a world column, in radians.
 *
 *   slope = atan2(Δh, Δx)   — central or forward difference would
 *                              both work; we use forward difference
 *                              for the cached buffer (h[c+1] − h[c]).
 *
 *   Sign convention in screen-y (which is positive DOWN):
 *     dh > 0 → terrain falls to the right → slope > 0 (descending)
 *     dh < 0 → terrain rises to the right → slope < 0 (ascending)
 *   We negate to match the cart-pole convention (positive = uphill
 *   to the right) so the controller sees an intuitive sign.
 */
static float terrain_slope_at(const Terrain *t, int wc)
{
    float dh = t->h[(wc + 1) & TMASK] - t->h[wc & TMASK];
    return -atanf(dh / (float)CELL_W);
}

static void terrain_ensure(Terrain *t, int upto)
{
    for (int c = t->gen_col + 1; c <= upto; c++)
        t->h[c & TMASK] = terrain_h_at(t, c);
    if (upto > t->gen_col) t->gen_col = upto;
}

static void terrain_init(Terrain *t, const Noise *n, int rows, int cols)
{
    t->noise   = n;
    t->rows    = rows;
    t->gen_col = -1;
    terrain_ensure(t, cols + 64);
}

/* ===================================================================== */
/* §6  bot — Bot type, PID, cart-pole physics, init/reset                 */
/* ===================================================================== */

/* ── §6.1 GainPreset (for the 'g' key teaching cycle) ─────────────── */

/*
 * GainPreset — one entry in the PRESETS[] table that the 'g' key
 * cycles through. Each preset is a triple of PID gains plus the
 * two-line lesson the phase-space panel displays so the reader
 * knows what symptom to expect.
 *
 * INTENT
 *   Make the PID-tuning intuition into something you can SEE:
 *   pressing 'g' swaps gains and the (θ, ω) trail visibly changes
 *   character (tight spiral → wide circles → drifting line). The
 *   lesson strings name the symptom in the user's own words so the
 *   reader doesn't have to figure out what they're looking at.
 *
 * WHY EMBEDDED IN THE FILE (not a config file / CLI flag)
 *   Pedagogy: showing four contrasting presets in a fixed cycle is
 *   the textbook way to teach PID tuning. Loading from disk would
 *   put the lesson out of reach of a first-time reader.
 *
 * ALGORITHM REFERENCE
 *   Åström & Hägglund (2006), "Advanced PID Control" — the
 *   underdamped / overdamped / steady-state-drift triad shown by
 *   these presets is the canonical "what each term does" example.
 */
typedef struct {
    const char *name;       /* short label shown in HUD + telemetry,  *
                              * padded to 8 chars for column alignment.*
                              * Examples: "BALANCED", "NO Kd   ".      */

    const char *lesson_l1;  /* line 1 of the on-screen mini-lesson    *
                              * shown in the phase-space panel.        */

    const char *lesson_l2;  /* line 2 — keep both lines ≤ 28 chars so  *
                              * they fit in PANEL_W = 32 with margin.  */

    float       kp;         /* proportional gain, N / rad.           *
                              * Higher = stiffer correction, more     *
                              * overshoot. Default preset uses 120.   */

    float       ki;         /* integral gain, N / (rad·sec).         *
                              * Higher = faster steady-state-error    *
                              * removal, more windup risk. Default 0.2.*/

    float       kd;         /* derivative gain, N / (rad/sec).       *
                              * Higher = more damping, slower         *
                              * response. Default 18; the "NO Kd"     *
                              * preset sets this to 0 to make the     *
                              * underdamped failure mode visible.     */
} GainPreset;

static const GainPreset PRESETS[] = {
    { "BALANCED",
      "well-tuned baseline.",
      "fast, damped, slope-aware.",
      120.0f, 0.20f, 18.0f },

    { "HIGH Kp ",
      "stiffer P → more overshoot",
      "& oscillation on slopes.",
      240.0f, 0.20f, 18.0f },

    { "NO Kd   ",
      "no damping → underdamped:",
      "bot oscillates forever.",
      120.0f, 0.20f,  0.0f },

    { "NO Ki   ",
      "no integral → steady-state",
      "drift on every slope.",
      120.0f, 0.00f, 18.0f },
};
#define N_PRESETS  (int)(sizeof PRESETS / sizeof PRESETS[0])

/* ── §6.2 Bot type — composed from six concept-sized sub-structs ──── */

/*
 * The old Bot held thirty flat fields with no grouping; the reader
 * had to remember which fields belonged to which concept. Splitting
 * the state into six concept-sized sub-structs makes each field's
 * role obvious at the access site:
 *
 *      b->pend.theta      — pendulum unknown
 *      b->pid.kp          — controller gain
 *      b->mot.world_x     — motion / world position
 *
 * Same data, same algorithm — only the namespace changed.
 */

/*
 * §6.2.1 Pendulum — the TWO unknowns the integrator advances.
 *
 * INTENT
 *   Make explicit that "the state of the dynamical system" is
 *   exactly (θ, ω) — two floats. Everything else in CartPole is
 *   bookkeeping intermediates we cache so the equations panel can
 *   display them; given just (θ, ω, F, α) the whole system is
 *   determined.
 *
 * STATE VECTOR (control-theory term)
 *   x = [θ, θ̇]ᵀ = [theta, omega]
 *   ẋ = [ω, θ̈]ᵀ — the right-hand side comes from cart-pole equations.
 *
 * SIGN CONVENTION
 *   θ > 0  → body leans RIGHT (positive x) from chassis-perpendicular.
 *   ω > 0  → θ is INCREASING (body rotating right faster).
 *   The controller drives θ → setpoint (and ω → 0 in steady state).
 *
 * ALGORITHM REFERENCE
 *   Spong/Hutchinson/Vidyasagar (2006) §6.5 — state-space form of
 *   the cart-pole equations of motion; "Pendulum" here is exactly
 *   their state vector for the inverted-pendulum-on-cart system.
 */
typedef struct {
    float theta;        /* lean angle from chassis-perpendicular,    *
                          * radians. Initialised to 0.04 (≈2.3°) so   *
                          * the controller has something to push       *
                          * against on startup. |theta_eff| > FALL_ANGLE*
                          * (≈ 1.05 rad ≈ 60°) triggers the fall      *
                          * banner.                                    */

    float omega;        /* angular velocity θ̇, radians/sec. Sign     *
                          * matches theta. Forward-Euler update each   *
                          * substep:  ω ← ω + θ̈ · dt . Reset to 0 by  *
                          * bot_reset() (preserves user toggles only). */
} Pendulum;

/*
 * §6.2.2 CartPole — quantities DERIVED from (Pendulum, alpha, F)
 * every substep. Written by cart_pole_step() and pid_step(); read
 * by the equations panel for the "math with live numbers" display.
 *
 * INTENT
 *   These are NOT independent state — given (θ, ω, α, F) you can
 *   reconstruct every field. They live in the struct only because
 *
 *     (1) the equations panel wants to show them every frame
 *         without recomputation; and
 *     (2) keeping intermediates as named fields makes
 *         cart_pole_step() read line-by-line like the textbook
 *         equations (no anonymous "tmp1, tmp2" locals).
 *
 * EQUATIONS (Lagrangian cart-pole on a sloped surface)
 *   θ_eff = θ + α                                       effective lean
 *   M_eff = M_cart + m_pole · sin²θ_eff                 effective inertia
 *   ẍ      = (F + m_pole·sin θ_eff·(L·ω² − g·cos θ_eff)) / M_eff
 *   θ̈      = (g·sin θ_eff − ẍ·cos θ_eff) / L
 *   ω ← ω + θ̈·dt                                       forward Euler
 *   θ ← θ + ω·dt
 *
 * WHY sin²θ_eff IN M_eff
 *   Some of the pole's mass effectively belongs to the cart's
 *   inertia (the part being dragged horizontally as the pole
 *   rotates). That fraction is sin²θ_eff — zero when the pole is
 *   vertical (M_eff = M_cart) and 1 when the pole is horizontal
 *   (the whole pole is shoved sideways with the cart).
 *
 * ALGORITHM REFERENCES
 *   Spong/Hutchinson/Vidyasagar (2006) §6.5 — Lagrangian derivation.
 *   Goldstein/Poole/Safko, "Classical Mechanics" — Lagrangian basics.
 *   (Newton-Euler derivation gives the same equations; either route
 *   reproduces the equations above.)
 */
typedef struct {
    float theta_eff;    /* θ + α, radians. The "lean from gravity"   *
                          * — what the integrator actually uses,      *
                          * because gravity pulls along world-down,   *
                          * not along chassis-perpendicular. Fall     *
                          * test compares |theta_eff| > FALL_ANGLE.   */

    float theta_ddot;   /* θ̈, rad/s². Angular acceleration this step.*
                          * Comes from gravity torque minus the cart  *
                          * acceleration's coupling term, divided by  *
                          * rod length L.                             */

    float x_ddot;       /* ẍ, m/s². Cart's horizontal acceleration   *
                          * this step. Drives the cart-pole COUPLING: *
                          * pushing the cart right tilts the body     *
                          * left (cos sign in θ̈ equation).            */

    float M_eff;        /* M_cart + m_pole·sin²θ_eff, kg. Effective  *
                          * inertia seen by force F. Always ≥ M_cart; *
                          * equals M_cart exactly when θ_eff = 0      *
                          * (pole perfectly vertical).                */

    float F;            /* Motor force applied to the cart this step,*
                          * Newtons. Written by pid_step(), clamped   *
                          * to ±MAX_FORCE (200 N). Read by            *
                          * cart_pole_step(). When PID is off, F = 0  *
                          * and the bot tips in ≈1 sec.               */
} CartPole;

/*
 * §6.2.3 PID — three-term feedback controller state.
 *
 * INTENT
 *   Implement the canonical PID control law
 *       F = Kp·e + Ki·∫e + Kd·de/dt          where e = θ − θ_ref
 *   with two practical refinements:
 *     • ANTI-WINDUP — clamp the integrator to ±WINDUP_MAX so a long
 *       sustained error (e.g. PID off, then re-enabled) can't make
 *       the integral dominate the output for many seconds afterward.
 *     • OUTPUT SATURATION — clamp F to ±MAX_FORCE to model the real
 *       motor's physical limit.
 *
 *   Per-term outputs (p, i, d, out) are stored separately so the
 *   equations panel can break the sum apart and the reader can
 *   multiply Kp·e by hand and verify the displayed P term matches.
 *
 * WHY SLOPE FEED-FORWARD (in setpoint, not as a fourth term)
 *   On a slope of α radians, gravity pulls the body away from
 *   chassis-perpendicular. To stay upright wrt gravity the body must
 *   lean INTO the slope. Setting setpoint = −SLOPE_FEED·α biases the
 *   error so the controller targets the right lean directly,
 *   without the integrator having to drag itself there. The integral
 *   then only mops up small residuals — much faster than waiting
 *   for ∫e to grow large enough to cancel slope drift on its own.
 *
 * READS / WRITES
 *   READS:  Pendulum.theta (current lean), Motion.alpha (via setpoint).
 *   WRITES: CartPole.F (the cart-pole equations consume it next).
 *
 * ALGORITHM REFERENCES
 *   Åström & Hägglund (2006), "Advanced PID Control" — entire book.
 *     Anti-windup is ch. 3; feed-forward is ch. 6.
 *   Åström & Murray (2008), "Feedback Systems" ch. 11 — PID from
 *     first principles. Free PDF at https://fbsbook.org .
 */
typedef struct {
    /* — gains (user-tunable; see PRESETS[] in §6.1) ——————————————— */
    float kp;               /* proportional, N/rad. Default 120.     *
                              * Higher → faster correction, more     *
                              * overshoot. +/- keys nudge by 5,       *
                              * clamped to [0, 400].                  */
    float ki;               /* integral, N/(rad·sec). Default 0.20.  *
                              * Higher → faster drift removal, more   *
                              * windup risk.                          */
    float kd;               /* derivative, N/(rad/sec). Default 18.  *
                              * Higher → more damping, slower         *
                              * response. The "NO Kd" preset sets     *
                              * this to 0 to show underdamped chaos.  */

    /* — controller state (written each substep) ——————————————————— */
    float setpoint;         /* θ_ref = −SLOPE_FEED · α. Slope-aware  *
                              * lean target so the bot leans INTO    *
                              * the hill. With SLOPE_FEED = 0.65, on *
                              * a 20° slope the body leans ≈13° into *
                              * it. Updated every substep.            */

    float integ;            /* ∫e, units rad·sec. Anti-windup keeps  *
                              * this in [-WINDUP_MAX, +WINDUP_MAX]    *
                              * = [-5, +5]. Reset to 0 by:            *
                              *   bot_reset(), gain-preset change,    *
                              *   'p' toggle, bot_apply_preset().     */

    float prev_err;         /* e from the previous substep, used by  *
                              * the derivative term:                  *
                              *   de/dt ≈ (e − prev_err) / dt         */

    /* — per-term outputs (display only) ——————————————————————————— */
    float p;                /* = kp · e         instant stiffness    */
    float i;                /* = ki · integ     drift removal        */
    float d;                /* = kd · de/dt     damping              */
    float out;              /* = p + i + d, clamped to ±MAX_FORCE.   *
                              * Copied to CartPole.F by pid_step().   */

    /* — enable toggle ———————————————————————————————————————————— */
    bool  on;               /* 'p' key toggles. When false, pid_step *
                              * forces p = i = d = out = 0 and the   *
                              * bot runs as a free inverted pendulum *
                              * (tips over in ≈ 1 sec). Persisted    *
                              * across bot_reset().                   */
} PID;

/*
 * §6.2.4 Motion — world-frame kinematics.
 *
 * INTENT
 *   Decouple "where the bot is in the world" from "how the body is
 *   posed relative to vertical" (Pendulum). The cart slides forward
 *   along the terrain at drive_spd — pure kinematics, no force
 *   integration — while Pendulum and PID handle the balancing.
 *
 * WHY pixel-space (not metres) for world_x
 *   Physics is in pixel-space throughout this codebase (CELL_W ×
 *   CELL_H sub-pixels per character cell). The terrain ring buffer
 *   indexes by pixel-column, so storing world_x in pixels lets us
 *   index directly without conversion:
 *       wc = (int)(world_x / CELL_W)
 *   The metres-per-pixel constant PIX_PER_M = 100 is used ONLY for
 *   HUD display: drive_spd / PIX_PER_M = m/s, dist_m for the odometer.
 *
 * SLOPE CONVENTION (used by Motion.alpha)
 *   alpha > 0  → terrain ascending to the right (uphill)
 *   alpha < 0  → terrain descending to the right (downhill)
 *   alpha = 0  → flat. Sampled at the WHEEL CONTACT column, not the
 *   centre of the chassis, so steep transitions look right.
 *
 * ALGORITHM REFERENCE
 *   Standard rigid-body kinematics — no algorithm needed beyond
 *   Newton's first law (drive_spd is constant unless the user
 *   adjusts it).
 */
typedef struct {
    float world_x;          /* horizontal position along the terrain *
                              * in pixel units. Monotonically grows   *
                              * (bot only drives forward). Wraps      *
                              * through the terrain ring buffer; the  *
                              * camera follows, so wrap is invisible. */

    float drive_spd;        /* forward speed, pixels/sec. Controlled *
                              * by ↑/↓ keys, clamped to               *
                              * [DRIVE_MIN, DRIVE_MAX] = [0, 160].    *
                              * 0 = stopped (pure inverted-pendulum   *
                              * test). Default DRIVE_DEF = 55 px/sec  *
                              * (≈ 0.55 m/s after PIX_PER_M scaling). */

    float spin_angle;       /* wheel rotation angle, rad. Cosmetic   *
                              * only — currently NOT rendered (wheels *
                              * are static 'O' glyphs). Kept so a     *
                              * future variant can show visible       *
                              * spokes without re-plumbing.           */

    float alpha;            /* terrain slope at the wheel-contact    *
                              * column, radians. Positive = ascending *
                              * to the right. Updated each substep   *
                              * from terrain_slope_at(). Feeds:       *
                              *   CartPole.theta_eff = θ + α          *
                              *   PID.setpoint = −SLOPE_FEED · α      */

    float dist_m;           /* cumulative travel, metres. HUD       *
                              * odometer. Incremented each substep:   *
                              *   dist_m += drive_spd · dt / PIX_PER_M*
                              * Reset to 0 by bot_reset().            */
} Motion;

/*
 * §6.2.5 PhaseTrail — ring buffer of the last HIST_LEN = 240 (θ, ω)
 * samples, plotted by render_panel_phase() as a 2-D phase portrait.
 *
 * INTENT
 *   Show the controller's CHARACTER as a 2-D trajectory rather than
 *   two side-by-side time-plots. The shape of the curve IS the
 *   controller's behaviour:
 *       inward spiral  →  stable, well-damped
 *       wide circles   →  underdamped (Kd too small)
 *       slow line      →  overdamped (Kd too large)
 *       drift line     →  steady-state error (Ki too small)
 *   The reader sees what they're trying to learn directly.
 *
 * WHY TWO PARALLEL ARRAYS (not array-of-struct)
 *   We plot the two coordinates independently (θ on x-axis, ω on y).
 *   At HIST_LEN = 240 the whole buffer is ≈ 2 KB and fits in L1
 *   trivially, so cache behaviour is the same either way. Parallel
 *   arrays make the renderer's "walk the indices" loop slightly
 *   clearer; pick whichever reads better, it's a wash.
 *
 * RING SEMANTICS
 *   head is the index of the NEXT slot to write. The newest valid
 *   sample is at (head − 1 + HIST_LEN) % HIST_LEN. fill grows from
 *   0 → HIST_LEN then saturates, so the renderer never plots
 *   uninitialised slots before the buffer first fills.
 *
 * WHY 240 SAMPLES
 *   At 60 fps and 1 substep/frame in steady state, 240 samples =
 *   ≈ 4 seconds of history. Long enough to show a complete recovery
 *   transient, short enough that an old trajectory fades before it
 *   confuses a new gain change.
 *
 * ALGORITHM REFERENCE
 *   Strogatz (2014), "Nonlinear Dynamics and Chaos", ch. 5-6 —
 *   phase portraits and trajectories in 2-D dynamical systems.
 *   The pictures there look exactly like what render_panel_phase()
 *   draws (see also figs in §MENTAL MODEL → PHASE PORTRAIT above).
 */
typedef struct {
    float theta[HIST_LEN];  /* θ samples, radians. Oldest valid     *
                              * sample is at slot                    *
                              *   (head - fill + HIST_LEN) % HIST_LEN *
                              * newest at  (head - 1) % HIST_LEN .   */

    float omega[HIST_LEN];  /* ω samples, rad/sec. Same indexing as *
                              * theta[] (parallel arrays).            */

    int   head;             /* index of the NEXT slot to write.     *
                              * Advances each substep:               *
                              *   head = (head + 1) % HIST_LEN       *
                              * Init = 0.                             */

    int   fill;             /* count of valid samples in the buffer *
                              * so far, clamped to HIST_LEN. Lets    *
                              * the renderer skip empty slots during *
                              * the first 4 sec after reset.         */
} PhaseTrail;

/*
 * §6.2.6 BotUI — keyboard-driven interactive flags.
 *
 * INTENT
 *   Separate "human input state" from physics state. Whether the
 *   bot is paused or which panel is showing has nothing to do with
 *   the dynamics — these flags exist purely because there's a human
 *   at the keyboard. Grouping them in a separate sub-struct makes
 *   that obvious at the type level.
 *
 * WHY KEEP THESE ACROSS bot_reset()
 *   When the user presses 'r' to reset, they want the SIMULATION
 *   to reset — not their personal preferences. So bot_reset()
 *   memcpys the whole Bot to zero, then explicitly RESTORES the
 *   BotUI fields (plus pid.on and mot.drive_spd). Putting them in
 *   their own sub-struct documents that intent at the type level
 *   and makes the restore code visually obvious.
 *
 * WHY 'fallen' IS NOT A PURE PHYSICS FACT
 *   "Fallen" is a renderer-friendly event flag, not a continuous
 *   physical quantity: it's TRUE iff the integrator has been told
 *   to give up. Putting it next to render-relevant flags (paused,
 *   view) instead of next to (θ, ω) reflects that it's read by the
 *   renderer, not by the equations of motion.
 */
typedef struct {
    bool     paused;        /* spacebar toggles. When true,           *
                              * bot_substep() is skipped → the bot     *
                              * freezes mid-air. Useful for studying   *
                              * a particular tilt frame, or staging    *
                              * a screenshot. Cleared on bot_reset().  */

    bool     fallen;        /* set true when |theta_eff| > FALL_ANGLE  *
                              * (≈ 60°). bot_substep() early-returns,  *
                              * render_bot() overlays the "FALLEN"     *
                              * banner. Cleared only on bot_reset();   *
                              * the user must press 'r' to recover.    */

    ViewMode view;          /* TELEMETRY / EQUATIONS / PHASE — which   *
                              * right-side panel is visible. Cycled    *
                              * by 'm' key. Persists across reset      *
                              * (user's chosen view is preserved).     */

    int      preset_idx;    /* index into PRESETS[] (§6.1). Cycled by  *
                              * 'g' key, mod N_PRESETS. 0 = BALANCED   *
                              * (default). Persists across reset.      *
                              * Changing it via 'g' calls              *
                              * bot_apply_preset() which clears integ. */
} BotUI;

/*
 * §6.2.7 Bot — composition of the six sub-structs above.
 *
 * INTENT
 *   Before the refactor this was 30 flat fields; the reader had to
 *   remember which field belonged to which concept. The composition
 *   makes the role of every access obvious at the use site:
 *
 *       b->pend.theta   pendulum unknown (the ODE state)
 *       b->cp.F         cart-pole derived (motor force this step)
 *       b->pid.kp       controller gain (a tuning parameter)
 *       b->mot.world_x  motion (where the bot is in the world)
 *       b->trail.head   phase trail (display ring buffer)
 *       b->ui.fallen    interactive flag (event marker for renderer)
 *
 *   Same bytes, same algorithm — only the namespace changed. The
 *   payoff is that reading any line of physics now tells you which
 *   conceptual layer it touches.
 *
 * LAYERING (read these in order to understand the simulation)
 *
 *   1. Pendulum    — the unknowns: what the ODE is integrating
 *   2. CartPole    — the equations: what writes those unknowns
 *   3. PID         — the controller: what drives the equations
 *   4. Motion      — the cart kinematics: where the controller acts
 *   5. PhaseTrail  — record/replay of (θ, ω) for the phase panel
 *   6. BotUI       — human flags, not part of physics
 *
 * WRITE ORDERING PER SUBSTEP (see bot_substep in §6.6)
 *
 *   1. Motion.world_x  += drive_spd · dt          kinematics
 *   2. Motion.alpha     = terrain_slope_at(...)    slope sample
 *   3. CartPole.theta_eff = pend.theta + alpha     effective lean
 *      PID.setpoint     = -SLOPE_FEED · alpha     slope feed-forward
 *   4. pid_step          → writes CartPole.F       controller
 *   5. cart_pole_step    → writes pend.theta/omega dynamics
 *   6. phase_push        → writes trail.theta/omega trace
 *
 *   Each step reads the previous step's writes. The integrator is
 *   forward Euler — explicit, dt-stable for small dt only, hence
 *   the sub-stepping in main().
 */
typedef struct {
    Pendulum   pend;        /* the two unknowns (θ, ω). See §6.2.1.   */
    CartPole   cp;          /* derived dynamics per substep. §6.2.2.  */
    PID        pid;         /* controller state + gains. §6.2.3.      */
    Motion     mot;         /* world position + slope cache. §6.2.4.  */
    PhaseTrail trail;       /* (θ, ω) history ring buffer. §6.2.5.    */
    BotUI      ui;          /* keyboard-driven flags. §6.2.6.         */
} Bot;

/* ── §6.3 PID controller — small named helpers + pid_step ─────────── */

/* Saturate a signed value to the symmetric range [-limit, +limit].
 * Used twice in pid_step: anti-windup on the integrator, output
 * saturation on the final motor force. */
static inline float clamp_symmetric(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/* Error signal — what the controller is trying to drive to zero.
 *      e = θ − θ_ref
 * Positive e ⇒ "leaning further than the slope-aware setpoint asks
 * for". Negative e ⇒ "leaning less / opposite direction". */
static inline float pid_error_signal(float theta, float setpoint)
{
    return theta - setpoint;
}

/* Forward-Euler integrate-and-clamp:  ∫e += e·dt , then anti-windup.
 * Anti-windup bounds the integrator's accumulated memory so that a
 * long sustained error can't dominate the output forever once the
 * regime changes. */
static inline void pid_accumulate_with_antiwindup(PID *pid, float err, float dt)
{
    pid->integ = clamp_symmetric(pid->integ + err * dt, WINDUP_MAX);
}

/* Backward-difference derivative of error: (e − e_prev) / dt.
 * Below PID_DT_FLOOR the subtraction becomes numerically meaningless
 * so we return 0 — losing one frame of D is harmless. */
static inline float pid_error_derivative(PID *pid, float err, float dt)
{
    float de_dt = (dt > PID_DT_FLOOR) ? (err - pid->prev_err) / dt : 0.0f;
    pid->prev_err = err;
    return de_dt;
}

/* When PID is toggled off, every term + the motor output is zero. */
static inline void pid_zero_outputs(PID *pid)
{
    pid->p = pid->i = pid->d = pid->out = 0.0f;
}

/*
 * pid_step — one tick of the three-term PID law.
 *
 *      F = clamp( Kp·e + Ki·∫e + Kd·de/dt , ±MAX_FORCE )
 *
 * Each step is one named helper so the body reads as the textbook
 * equation, not as a flat block of float arithmetic.
 */
static void pid_step(Bot *b, float dt)
{
    PID *pid = &b->pid;

    /* (0) controller disengaged → no force, all terms zero */
    if (!pid->on) {
        pid_zero_outputs(pid);
        b->cp.F = 0.0f;
        return;
    }

    /* (1) error signal */
    float err = pid_error_signal(b->pend.theta, pid->setpoint);

    /* (2) integrate (with anti-windup) and differentiate the error */
    pid_accumulate_with_antiwindup(pid, err, dt);
    float de_dt = pid_error_derivative(pid, err, dt);

    /* (3) three-term sum — each term kept for the equations panel */
    pid->p   = pid->kp * err;
    pid->i   = pid->ki * pid->integ;
    pid->d   = pid->kd * de_dt;
    pid->out = clamp_symmetric(pid->p + pid->i + pid->d, MAX_FORCE);

    /* (4) publish the motor force to the cart-pole dynamics step */
    b->cp.F = pid->out;
}

/* ── §6.4 cart-pole dynamics (Lagrangian) — physics helpers + step ── */

/*
 * Effective inertia seen by the horizontal force F.
 *      M_eff = M_cart + m_pole · sin²θ_eff
 * sin²θ_eff is the fraction of the pole's mass being DRAGGED
 * horizontally as the pole rotates — it adds to the cart's mass.
 * Equals M_cart when θ_eff = 0, equals M_cart + m_pole when θ_eff =
 * 90° (pole horizontal, entire pole shoved sideways with the cart).
 */
static inline float pole_effective_inertia(float theta_eff)
{
    float sin_th = sinf(theta_eff);
    return MASS_CART + MASS_POLE * sin_th * sin_th;
}

/*
 * Horizontal acceleration of the cart axle (the "ẍ" of the Lagrangian).
 *      ẍ = ( F + m_pole·sinθ_eff·(L·ω² − g·cosθ_eff) ) / M_eff
 * Middle term is the centripetal/gravitational reaction the rotating
 * pole exerts back on its own pivot.
 */
static inline float cart_horizontal_accel(float F, float theta_eff,
                                          float omega, float M_eff)
{
    float sin_th = sinf(theta_eff);
    float cos_th = cosf(theta_eff);
    float pole_reaction = MASS_POLE * sin_th *
                          (PEND_LEN * omega * omega - GRAVITY * cos_th);
    return (F + pole_reaction) / M_eff;
}

/*
 * Angular acceleration of the pole (the "θ̈" of the Lagrangian).
 *      θ̈ = ( g·sinθ_eff − ẍ·cosθ_eff ) / L
 * First term: gravity tips the pole; second term: cart-pole COUPLING
 * — cart's horizontal acceleration tilts the pole the OTHER way
 * (because the base translates and the top rotationally lags). This
 * is the term the controller exploits to recover from a lean.
 */
static inline float pole_angular_accel(float theta_eff, float x_ddot)
{
    return (GRAVITY * sinf(theta_eff) - x_ddot * cosf(theta_eff)) / PEND_LEN;
}

/* Forward-Euler step: advance (ω, θ) one timestep at the given θ̈.
 *      ω ← ω + θ̈ · dt
 *      θ ← θ + ω · dt
 * Symplectic-Euler order (update ω before θ) — keeps phase-portrait
 * spirals tight even with relatively large dt. */
static inline void pendulum_euler_advance(Pendulum *pe, float theta_ddot,
                                          float dt)
{
    pe->omega += theta_ddot * dt;
    pe->theta += pe->omega  * dt;
}

/*
 * cart_pole_step — one tick of the Lagrangian cart-pole dynamics.
 * Body reads as the four-line algorithm; each line is one named
 * physics helper above.
 */
static void cart_pole_step(Bot *b, float dt)
{
    CartPole *cp = &b->cp;
    Pendulum *pe = &b->pend;

    /* (1) effective inertia — how much "mass" the horizontal force sees */
    cp->M_eff      = pole_effective_inertia(cp->theta_eff);

    /* (2) cart acceleration — motor force + pole reaction over inertia  */
    cp->x_ddot     = cart_horizontal_accel(cp->F, cp->theta_eff,
                                           pe->omega, cp->M_eff);

    /* (3) pole acceleration — gravity tipping − cart's coupling         */
    cp->theta_ddot = pole_angular_accel(cp->theta_eff, cp->x_ddot);

    /* (4) integrate (ω, θ) forward by dt                                */
    pendulum_euler_advance(pe, cp->theta_ddot, dt);
}

/* ── §6.5 phase history push ──────────────────────────────────────── */

static void phase_push(Bot *b)
{
    PhaseTrail *tr = &b->trail;
    tr->theta[tr->head] = b->pend.theta;
    tr->omega[tr->head] = b->pend.omega;
    tr->head = (tr->head + 1) % HIST_LEN;
    if (tr->fill < HIST_LEN) tr->fill++;
}

/* ── §6.6 bot_substep — substep helpers + the six-step physics tick ── */

/* Kinematics — cart slides forward at drive_spd. No forces, just
 * Newton's first law plus odometer + wheel-spin bookkeeping. */
static inline void motion_advance(Motion *mo, float dt)
{
    mo->world_x    += mo->drive_spd * dt;
    mo->dist_m     += mo->drive_spd * dt / PIX_PER_M;
    mo->spin_angle += (mo->drive_spd / WHEEL_R) * dt;
}

/* World column under the wheel — used to sample terrain.h[]. */
static inline int wheel_world_column(const Motion *mo)
{
    return (int)(mo->world_x / (float)CELL_W);
}

/* Derive the two slope-dependent inputs the dynamics needs:
 *      θ_eff    = θ + α        (effective lean from gravity)
 *      θ_ref   = −SLOPE_FEED · α  (slope-aware PID setpoint)
 * Both are recomputed every substep because α changes as the bot
 * traverses new terrain. */
static inline void apply_slope_feed_forward(Bot *b)
{
    float alpha = b->mot.alpha;
    b->cp.theta_eff = b->pend.theta + alpha;
    b->pid.setpoint = -alpha * SLOPE_FEED;
}

/* Fall test — |θ_eff| past FALL_ANGLE means even unbounded F can't
 * recover in time, so we just stop ticking and show the banner. */
static inline void detect_fall(Bot *b)
{
    if (fabsf(b->cp.theta_eff) > FALL_ANGLE) b->ui.fallen = true;
}

/*
 * bot_substep — one physics timestep. Six named steps; each step is
 * either a helper call or one line of cross-layer wiring. The whole
 * algorithm fits in a screen with no nested logic.
 *
 * Called N times per frame from main(), with sub_dt ≤ TICK_DT_TARGET,
 * so the forward-Euler integrator stays well inside its stability
 * region regardless of frame rate.
 */
static void bot_substep(Bot *b, const Terrain *t, float sub_dt)
{
    if (b->ui.fallen) return;

    /* (1) advance kinematics; sample slope at the new wheel position */
    motion_advance(&b->mot, sub_dt);
    b->mot.alpha = terrain_slope_at(t, wheel_world_column(&b->mot));

    /* (2) derive (θ_eff, setpoint) from the new slope                */
    apply_slope_feed_forward(b);

    /* (3) controller → motor force F                                 */
    pid_step(b, sub_dt);

    /* (4) cart-pole dynamics → new (θ, ω)                            */
    cart_pole_step(b, sub_dt);

    /* (5) record the (θ, ω) sample for the phase portrait            */
    phase_push(b);

    /* (6) fall test — sets ui.fallen if |θ_eff| > FALL_ANGLE         */
    detect_fall(b);
}

/* ── §6.7 init / reset / preset ──────────────────────────────────── */

static void bot_apply_preset(Bot *b, int idx)
{
    const GainPreset *p = &PRESETS[idx];
    b->pid.kp    = p->kp;
    b->pid.ki    = p->ki;
    b->pid.kd    = p->kd;
    b->pid.integ = 0.0f;        /* clear integral on gain change */
}

/* The four settings the user can change at runtime and expects to
 * survive a reset. Carried out of bot_reset() and back in after the
 * memset, so 'r' resets physics WITHOUT wiping the user's choices. */
typedef struct {
    bool     pid_on;
    float    drive_spd;
    ViewMode view;
    int      preset_idx;
} UserPrefs;

static UserPrefs grab_user_prefs(const Bot *b)
{
    return (UserPrefs){
        .pid_on     = b->pid.on,
        .drive_spd  = b->mot.drive_spd,
        .view       = b->ui.view,
        .preset_idx = b->ui.preset_idx,
    };
}

static void restore_user_prefs(Bot *b, UserPrefs p)
{
    b->pid.on        = p.pid_on;
    b->mot.drive_spd = p.drive_spd;
    b->ui.view       = p.view;
    b->ui.preset_idx = p.preset_idx;
}

/*
 * bot_reset — 'r' key. Resets physics to its boot state but keeps
 * the user's interactive choices (PID on/off, drive speed, view,
 * preset). Same body as bot_init, except prefs come from the
 * existing bot rather than the defaults.
 */
static void bot_reset(Bot *b)
{
    UserPrefs prefs = grab_user_prefs(b);
    memset(b, 0, sizeof *b);
    restore_user_prefs(b, prefs);
    b->pend.theta = INITIAL_LEAN_RAD;
    bot_apply_preset(b, prefs.preset_idx);
}

/*
 * bot_init — one-shot startup. Same shape as bot_reset, but the
 * "preserved" prefs are the defaults from §1 instead of the
 * (uninitialised) current bot.
 */
static void bot_init(Bot *b)
{
    UserPrefs defaults = {
        .pid_on     = true,
        .drive_spd  = DRIVE_DEF,
        .view       = VIEW_TELEMETRY,
        .preset_idx = 0,
    };
    memset(b, 0, sizeof *b);
    restore_user_prefs(b, defaults);
    b->pend.theta = INITIAL_LEAN_RAD;
    bot_apply_preset(b, defaults.preset_idx);
}

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* ── §7.1 helpers ────────────────────────────────────────────────── */

static inline int   px_to_cx (float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline bool  in_screen(int r, int c, int rows, int cols)
{
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

static void put_ch(int r, int c, chtype ch, attr_t a, int cp,
                   int rows, int cols)
{
    if (!in_screen(r, c, rows, cols)) return;
    attron (a | COLOR_PAIR(cp));
    mvaddch(r, c, ch);
    attroff(a | COLOR_PAIR(cp));
}

static void put_str(int r, int c, const char *s, attr_t a, int cp,
                    int rows, int cols)
{
    if (r < 0 || r >= rows) return;
    attron (a | COLOR_PAIR(cp));
    for (int i = 0; s[i] && c + i < cols; i++)
        mvaddch(r, c + i, (unsigned char)s[i]);
    attroff(a | COLOR_PAIR(cp));
}

/*
 * Bresenham line picker — chooses a glyph based on segment slope so
 * a long line "looks line-like" without per-cell trig.
 */
static chtype seg_glyph(int dr, int dc)
{
    if (dr == 0) return '-';
    if (dc == 0) return '|';
    return (dr * dc < 0) ? '/' : '\\';
}

/*
 * Bresenham line — rasterise (x0,y0)→(x1,y1) pixel-space endpoints
 * onto character cells, choosing a glyph ONCE from the segment's
 * overall slope. Classic "stepped error term" algorithm:
 *
 *      initial error = |Δrow| − |Δcol|
 *      each step: advance whichever axis the error currently favours,
 *                 update the error by the OTHER axis's |Δ|.
 *
 * See Bresenham (1965) in References.
 */
static void draw_line(float x0, float y0, float x1, float y1,
                      attr_t a, int cp, int rows, int cols)
{
    /* (1) Endpoints in cell space */
    int r0 = px_to_cy(y0), c0 = px_to_cx(x0);
    int r1 = px_to_cy(y1), c1 = px_to_cx(x1);

    /* (2) Span and step direction along each axis */
    int dr = r1 - r0, dc = c1 - c0;
    int step_r = (r0 < r1) ? 1 : -1;
    int step_c = (c0 < c1) ? 1 : -1;
    int abs_dr = abs(dr), abs_dc = abs(dc);

    /* (3) Pick the glyph from the overall slope (once, not per cell) */
    chtype glyph = seg_glyph(dr, dc);

    /* (4) Walk the line, decision variable controls which axis steps */
    int err = abs_dr - abs_dc;
    int r = r0, c = c0;
    for (;;) {
        put_ch(r, c, glyph, a, cp, rows, cols);
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs_dc) { err -= abs_dc; r += step_r; }   /* step row */
        if (e2 <  abs_dr) { err += abs_dr; c += step_c; }   /* step col */
    }
}

/* Empty bar background — W spaces and a centre '|' tick mark. */
static void bar_paint_track(int r, int c, int W, int mid, int rows, int cols)
{
    attron(COLOR_PAIR(CP_DIM));
    for (int i = 0; i < W; i++) put_ch(r, c + i, ' ', 0, CP_DIM, rows, cols);
    put_ch(r, mid, '|', 0, CP_DIM, rows, cols);
    attroff(COLOR_PAIR(CP_DIM));
}

/* Filled portion of the bar — `cells` '=' glyphs walking outward from
 * mid, in `step` direction (+1 for positive v, -1 for negative). */
static void bar_paint_fill(int r, int mid, int cells, int step, int cp,
                           int rows, int cols)
{
    attron(COLOR_PAIR(cp) | A_BOLD);
    for (int i = 0; i < cells; i++)
        put_ch(r, mid + step * (i + 1), '=', A_BOLD, cp, rows, cols);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * Centred horizontal bar gauge for v ∈ [-1, +1]. Two passes:
 *   (1) paint the empty track + midline mark
 *   (2) paint the filled portion outward from mid in v's sign direction
 */
static void draw_bar(int r, int c, int W, float v,
                     int cp_pos, int cp_neg, int rows, int cols)
{
    int half  = W / 2;
    int mid   = c + half;
    int cells = (int)(fabsf(v) * (float)half);
    if (cells > half) cells = half;

    bar_paint_track(r, c, W, mid, rows, cols);

    int step = (v >= 0.0f) ? +1 : -1;
    int cp   = (v >= 0.0f) ? cp_pos : cp_neg;
    bar_paint_fill(r, mid, cells, step, cp, rows, cols);
}

/* ── §7.2 render_terrain — sky, surface, rock fill ───────────────── */

static bool is_star(int r, int c)
{
    /* Cheap deterministic hash so stars stay in fixed cells. */
    unsigned h = (unsigned)(r * 7919 + c * 6271);
    h ^= h >> 13; h *= 0x45d9f3bU; h ^= h >> 17;
    return (h % 60) == 0;
}

/* Pick the surface glyph for a column based on the local Δh:
 *   |Δh| > SLOPE_GLYPH_FRAC · CELL_W  →  '/' or '\\' (sloped)
 *   otherwise                          →  '_'        (near-flat)
 */
static chtype surface_glyph(float dh)
{
    float thresh = (float)CELL_W * SLOPE_GLYPH_FRAC;
    if (dh >  thresh) return '/';
    if (dh < -thresh) return '\\';
    return '_';
}

/* Sky cell — sparse hash-selected stars on otherwise empty rows. */
static void paint_sky_cell(int r, int sc, int rows, int cols)
{
    if (is_star(r, sc))
        put_ch(r, sc, '.', A_BOLD, CP_STAR, rows, cols);
}

/* Surface cell — slope-shaped glyph ( _ / \ ), red if steep. */
static void paint_surface_cell(int r, int sc, chtype sg, float dh,
                               int rows, int cols)
{
    bool steep = fabsf(dh) > (float)CELL_W * SLOPE_GLYPH_FRAC;
    int  cp    = steep ? CP_ROCK : CP_SURF;
    put_ch(r, sc, sg, A_BOLD, cp, rows, cols);
}

/* Grass row directly below the surface — colon/dot alternation. */
static void paint_grass_cell(int r, int sc, int rows, int cols)
{
    put_ch(r, sc, (sc % 3 == 0) ? ':' : '.', A_DIM, CP_SURF, rows, cols);
}

/* Rock fill below the grass — sparse '#' over default background. */
static void paint_rock_cell(int r, int sc, int rows, int cols)
{
    put_ch(r, sc, (sc % 2 == 0) ? '#' : ' ', A_DIM, CP_ROCK, rows, cols);
}

/* Cell-y of the surface for world column wc, clamped so the terrain
 * never touches the HUD row (row 0) or the hint strip (row rows-1). */
static int terrain_surface_row(const Terrain *t, int wc, int rows)
{
    int surf = px_to_cy(t->h[wc & TMASK]);
    if (surf < 1)        return 1;
    if (surf > rows - 2) return rows - 2;
    return surf;
}

/* Slope estimate at world column wc (forward difference, pixel units).
 * Used both for the surface glyph choice and the steep-cell colour. */
static float terrain_dh_at(const Terrain *t, int wc)
{
    return t->h[(wc + 1) & TMASK] - t->h[wc & TMASK];
}

/*
 * render_terrain — paint the entire background each frame.
 *
 *   For each on-screen column sc:
 *     1. Map sc → world column wc (camera follows the bot).
 *     2. Find the surface cell-row and the local slope.
 *     3. Classify each row as sky / surface / grass / rock and paint.
 */
static void render_terrain(const Terrain *t, int bot_wc, int bot_sc,
                           int rows, int cols)
{
    for (int sc = 0; sc < cols; sc++) {
        /* (1) screen column → world column */
        int wc = bot_wc - bot_sc + sc;
        if (wc < 0) wc = 0;

        /* (2) surface row + local slope (drives glyph + colour) */
        int    surf = terrain_surface_row(t, wc, rows);
        float  dh   = terrain_dh_at(t, wc);
        chtype sg   = surface_glyph(dh);

        /* (3) classify each row vertically and paint accordingly */
        for (int r = 0; r < rows; r++) {
            if      (r <  surf)     paint_sky_cell    (r, sc, rows, cols);
            else if (r == surf)     paint_surface_cell(r, sc, sg, dh, rows, cols);
            else if (r == surf + 1) paint_grass_cell  (r, sc, rows, cols);
            else                    paint_rock_cell   (r, sc, rows, cols);
        }
    }
}

/* ── §7.3 render_bot — clean wheels + tilted body line ───────────── */

/*
 * The robot is intentionally minimal:
 *
 *      *           ← beacon (top of body)
 *       \
 *        \         ← body line (tilts with θ_eff = θ + α)
 *         \
 *        O═O       ← chassis '═' between two 'O' wheels
 *
 * Total: 6 cells of body + 1 beacon + 3 chassis = ~10 cells.
 * Lean is shown by the body line's actual angle in cell space.
 */
/*
 * The four key pixel-space "joint" positions render_bot uses. Building
 * them all in one place means each draw_line/put_ch below reads from
 * this struct — no scattered coord math across the renderer.
 */
typedef struct {
    float ax_x, ax_y;   /* axle (cart pivot)  — anchored above terrain */
    float lw_x, lw_y;   /* left wheel  — rotated by terrain slope α    */
    float rw_x, rw_y;   /* right wheel — rotated by terrain slope α    */
    float top_x, top_y; /* body top    — rotated by lean θ_eff         */
} BotPixels;

/*
 * Build the bot's four pixel-space joint positions.
 *   axle:   anchored at bot_sc · CELL_W (screen column), lifted
 *           WHEEL_R above the terrain surface.
 *   wheels: rotate the offset (±AXLE_HW, 0) about the axle by the
 *           TERRAIN SLOPE α so the chassis sits flat on the ground.
 *   top:    rotate the offset (0, -BODY_H) about the axle by the
 *           GRAVITY-FRAME LEAN θ_eff so the pole leans relative to
 *           gravity, NOT relative to the chassis.
 *
 * The two rotations use DIFFERENT angles — that distinction (chassis
 * follows ground, body follows gravity) IS the whole demo.
 */
static BotPixels compute_bot_skeleton(const Bot *b, float h_px, int bot_sc)
{
    BotPixels px;

    /* Axle anchor — fixed screen column, one wheel-radius above ground */
    px.ax_x = (float)(bot_sc * CELL_W);
    px.ax_y = h_px - WHEEL_R;

    /* Wheels — rotate (±AXLE_HW, 0) about axle by terrain slope α */
    float cos_a = cosf(b->mot.alpha), sin_a = sinf(b->mot.alpha);
    px.lw_x = px.ax_x - AXLE_HW * cos_a;
    px.lw_y = px.ax_y + AXLE_HW * sin_a;
    px.rw_x = px.ax_x + AXLE_HW * cos_a;
    px.rw_y = px.ax_y - AXLE_HW * sin_a;

    /* Body top — rotate (0, -BODY_H) about axle by gravity-frame lean */
    float th = b->cp.theta_eff;
    px.top_x = px.ax_x + BODY_H * sinf(th);
    px.top_y = px.ax_y - BODY_H * cosf(th);

    return px;
}

/* Two 'O' wheel glyphs centred on the wheel pixel positions. */
static void paint_wheels(const BotPixels *px, int rows, int cols)
{
    put_ch(px_to_cy(px->lw_y), px_to_cx(px->lw_x),
           'O', A_BOLD, CP_WHEEL, rows, cols);
    put_ch(px_to_cy(px->rw_y), px_to_cx(px->rw_x),
           'O', A_BOLD, CP_WHEEL, rows, cols);
}

/*
 * Body-top beacon — '*' that pulses bold/dim every 1/(2·BEACON_PULSE_HZ)
 * SECONDS OF TRAVEL. Pulse rate scales with dist_m (how far the bot
 * has moved) rather than wall time — so it slows when the bot stops
 * and makes the visual link bot↔terrain stronger.
 */
static void paint_beacon(const BotPixels *px, float dist_m,
                         int rows, int cols)
{
    int    pulse_phase = (int)(dist_m * BEACON_PULSE_HZ);
    attr_t pulse_attr  = (pulse_phase & 1) ? A_BOLD : A_DIM;
    put_ch(px_to_cy(px->top_y), px_to_cx(px->top_x),
           '*', pulse_attr, CP_BEACON, rows, cols);
}

/*
 * Floating "+12.3°" label near the axle — green normally, red past
 * THETA_WARN_DEG so the reader notices the danger zone at a glance.
 */
static void paint_lean_readout(const BotPixels *px, float theta_eff_rad)
{
    float deg = theta_eff_rad * (180.0f / (float)M_PI);
    int   cp  = (fabsf(deg) > THETA_WARN_DEG) ? CP_WARN : CP_GOOD;
    int   ar  = px_to_cy(px->ax_y);
    int   ac  = px_to_cx(px->ax_x);
    attron (COLOR_PAIR(cp));
    mvprintw(ar, ac - LEAN_LABEL_DC, "%+5.1f°", deg);
    attroff(COLOR_PAIR(cp));
}

/* Centre-of-screen banner shown when the fall flag is set. */
static void paint_fallen_banner(int rows, int cols)
{
    int mr = rows / 2;
    int mc = cols / 2 - 9;
    if (mc < 0) mc = 0;
    attron (COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
    mvprintw(mr, mc, "  !! FALLEN !!  ");
    attroff(COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
    attron (COLOR_PAIR(CP_DIM));
    mvprintw(mr + 1, mc - 1, "g=preset  r=reset");
    attroff(COLOR_PAIR(CP_DIM));
}

/*
 * render_bot — paint the robot in painter's order:
 *
 *   1. Compute the four pixel-space joints (axle, two wheels, top).
 *   2. Draw chassis line (between wheels) + body pole (axle → top).
 *   3. Stamp wheel glyphs + pulsing body-top beacon.
 *   4. Overlay decorations: lean readout, fallen banner.
 */
static void render_bot(const Bot *b, const Terrain *t, int bot_sc,
                       int rows, int cols)
{
    int   wc   = (int)(b->mot.world_x / (float)CELL_W);
    float h_px = t->h[wc & TMASK];

    /* (1) four pixel-space joint positions */
    BotPixels px = compute_bot_skeleton(b, h_px, bot_sc);

    /* (2) chassis between wheels, body line from axle to top */
    draw_line(px.lw_x, px.lw_y, px.rw_x, px.rw_y,
              A_BOLD, CP_CHASSIS, rows, cols);
    draw_line(px.ax_x, px.ax_y, px.top_x, px.top_y,
              A_BOLD, CP_CHASSIS, rows, cols);

    /* (3) wheels and pulsing beacon */
    paint_wheels(&px, rows, cols);
    paint_beacon(&px, b->mot.dist_m, rows, cols);

    /* (4) overlays */
    paint_lean_readout(&px, b->cp.theta_eff);
    if (b->ui.fallen) paint_fallen_banner(rows, cols);
}

/* ── §7.4 panel helpers + render_panel_telemetry ──────────────────── */

/* One label-only row in a side panel. Returns the next row to write. */
static int panel_text_row(int r, int c0, const char *s, attr_t a, int cp,
                          int rows, int cols)
{
    put_str(r, c0, s, a, cp, rows, cols);
    return r + 1;
}

/* Label on the left + horizontal bar gauge on the right. The label
 * is already-formatted text (caller does the snprintf); the bar
 * shows val_norm in [-1, +1] using the (cp_pos, cp_neg) pair. */
static int panel_text_bar_row(int r, int c0, const char *s, attr_t a, int cp,
                              float val_norm, int cp_pos, int cp_neg,
                              int rows, int cols)
{
    put_str(r, c0, s, a, cp, rows, cols);
    draw_bar(r, c0 + BAR_COL_OFFSET, BAR_WIDTH, val_norm,
             cp_pos, cp_neg, rows, cols);
    return r + 1;
}

/*
 * render_panel_telemetry — live numeric readouts + bar gauges.
 *
 *   Section 1: dynamical state (θ_eff, ω, α) — each with a centred bar
 *              scaled to the alarming extreme of that quantity.
 *   Section 2: cart motion (spd, dist) — text only, no bar.
 *   Section 3: PID per-term outputs (P, I, D) + total F (saturation-warn).
 *   Section 4: current gain values + active preset name.
 *
 * Bars use full-scale ranges from §1.16; warn-colour cutoffs from §1.16.
 */
static void render_panel_telemetry(const Bot *b, int rows, int cols, int c0)
{
    const PID *pid = &b->pid;
    char buf[64];
    int  r = 0;

    /* ── header ── */
    r = panel_text_row(r, c0, " TELEMETRY              ",
                       A_BOLD, CP_VAL, rows, cols);

    /* ── section 1: dynamical state ── */
    /* θ_eff — gravity-frame lean, ° */
    {
        float deg     = b->cp.theta_eff * (180.0f / (float)M_PI);
        int   cp_lean = fabsf(deg) > THETA_WARN_DEG ? CP_WARN : CP_GOOD;
        snprintf(buf, sizeof buf, " theta_eff %+6.2f° ", deg);
        r = panel_text_bar_row(r, c0, buf, A_BOLD, cp_lean,
                               deg / THETA_BAR_RANGE_DEG,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    }
    /* ω — angular velocity, rad/sec */
    {
        snprintf(buf, sizeof buf, " omega    %+6.2f  ", b->pend.omega);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               b->pend.omega / OMEGA_BAR_RANGE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    }
    /* α — terrain slope, ° (swapped pos/neg pair so uphill colours warm) */
    {
        float sdeg     = b->mot.alpha * (180.0f / (float)M_PI);
        int   cp_slope = fabsf(sdeg) > SLOPE_WARN_DEG ? CP_WARN : CP_VAL;
        snprintf(buf, sizeof buf, " slope α  %+6.2f° ", sdeg);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, cp_slope,
                               sdeg / SLOPE_BAR_RANGE_DEG,
                               CP_BAR_NEG, CP_BAR_POS, rows, cols);
    }

    /* ── section 2: cart motion (text-only, no bars) ── */
    snprintf(buf, sizeof buf, " spd  %+5.2f m/s     ",
             b->mot.drive_spd / PIX_PER_M);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
    snprintf(buf, sizeof buf, " dist %7.1f m       ", b->mot.dist_m);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

    /* ── section 3: PID per-term outputs ── */
    r++;   /* blank line */
    r = panel_text_row(r, c0, " PID OUTPUTS            ",
                       A_BOLD, CP_VAL, rows, cols);
    if (pid->on) {
        snprintf(buf, sizeof buf, " P  Kp·e   %+8.2f ", pid->p);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               pid->p / MAX_FORCE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);

        snprintf(buf, sizeof buf, " I  Ki·∫e  %+8.2f ", pid->i);
        r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

        snprintf(buf, sizeof buf, " D  Kd·θ̇   %+8.2f ", pid->d);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               pid->d / (MAX_FORCE * D_BAR_FRAC),
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);

        snprintf(buf, sizeof buf, " F total  %+8.2f N", pid->out);
        int cp_F = fabsf(pid->out) > MAX_FORCE * SAT_WARN_FRAC
                 ? CP_WARN : CP_GOOD;
        r = panel_text_bar_row(r, c0, buf, A_BOLD, cp_F,
                               pid->out / MAX_FORCE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    } else {
        r = panel_text_row(r, c0, " PID  DISABLED          ",
                           A_BOLD, CP_WARN, rows, cols);
        r += 3;
    }

    /* ── section 4: current gains + active preset ── */
    r++;
    snprintf(buf, sizeof buf, " Kp:%.0f Ki:%.2f Kd:%.0f  ",
             pid->kp, pid->ki, pid->kd);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " preset: %s ",
             PRESETS[b->ui.preset_idx].name);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
}

/* ── §7.5 render_panel_equations — math with live values ─────────── */

/* Two-line "caption: value" block — dim label on top, bright value on
 * the next row. The whole equations panel is just a stack of these. */
static int panel_equation_pair(int r, int c0,
                               const char *caption, const char *value,
                               attr_t a_val, int cp_val,
                               int rows, int cols)
{
    put_str(r,     c0 + 1, caption, A_DIM, CP_DIM, rows, cols);
    put_str(r + 1, c0,     value,   a_val, cp_val, rows, cols);
    return r + 2;
}

/*
 * render_panel_equations — show the cart-pole and PID equations with
 * their current numerical values substituted. The learner can multiply
 * Kp · e by hand and verify it matches the displayed P term — making
 * the panel a "cell-level debugger" for the controller.
 *
 * Layout: caption row (what the equation IS), then value row (current
 * numbers). One panel_equation_pair() per concept.
 */
static void render_panel_equations(const Bot *b, int rows, int cols, int c0)
{
    const PID *pid = &b->pid;
    float err = pid_error_signal(b->pend.theta, pid->setpoint);
    char  buf[80];
    int   r = 0;

    /* ── header ── */
    r = panel_text_row(r, c0, " EQUATIONS               ",
                       A_BOLD, CP_VAL, rows, cols);

    /* ── cart-pole derived quantities ── */
    snprintf(buf, sizeof buf, " θ_eff = θ + α = %+5.3f rad (%+5.1f°)",
             b->cp.theta_eff, b->cp.theta_eff * (180.0f / (float)M_PI));
    r = panel_equation_pair(r, c0, "Effective lean from grav:", buf,
                            A_BOLD, CP_VAL, rows, cols);

    snprintf(buf, sizeof buf, " ẍ = %+6.3f m/s²", b->cp.x_ddot);
    r = panel_equation_pair(r, c0, "Horizontal accel of base:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " θ̈ = %+6.3f rad/s²", b->cp.theta_ddot);
    r = panel_equation_pair(r, c0, "Pendulum angular accel:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    /* ── PID law, broken into setpoint + error + three terms ── */
    r++;
    r = panel_text_row(r, c0, " PID                    ",
                       A_BOLD, CP_VAL, rows, cols);

    snprintf(buf, sizeof buf, " θ_ref = -%.2f·α = %+5.3f rad",
             SLOPE_FEED, pid->setpoint);
    r = panel_equation_pair(r, c0, "Setpoint (slope feed):", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " e = θ − θ_ref = %+5.3f", err);
    r = panel_equation_pair(r, c0, "Error drives PID:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.0f · %+.4f = %+.2f", pid->kp, err, pid->p);
    r = panel_equation_pair(r, c0, "P — instant stiffness:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.2f · ∫e = %+.2f", pid->ki, pid->i);
    r = panel_equation_pair(r, c0, "I — drift removal:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.0f · θ̇ = %+.2f", pid->kd, pid->d);
    r = panel_equation_pair(r, c0, "D — damping:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    /* ── total motor force F (saturation-warning colour) ── */
    int cp_F = fabsf(pid->out) > MAX_FORCE * SAT_WARN_FRAC
             ? CP_WARN : CP_GOOD;
    snprintf(buf, sizeof buf, " F = %+.2f N (max ±%.0f)",
             pid->out, MAX_FORCE);
    r = panel_text_row(r, c0, buf, A_BOLD, cp_F, rows, cols);
}

/* ── §7.6 render_panel_phase — phase-space portrait of (θ, ω) ────── */

/* Plot geometry — origin + scale factors for the phase box. */
typedef struct {
    int   r0, c0;           /* top-left of plot rectangle (cells)  */
    int   mid_r, mid_c;     /* (θ=0, ω=0) origin in cell space    */
    float th_to_cells;      /* cells-per-rad on the θ axis         */
    float om_to_cells;      /* cells-per-(rad/sec) on the ω axis  */
} PhasePlot;

/* Configure the plot from its top-left corner. Scale factors map
 * a value in PHASE_*_RANGE onto half the plot's width/height, so the
 * trail saturates exactly at the edge of the box. */
static PhasePlot phase_configure(int plot_r0, int c0)
{
    PhasePlot p;
    p.r0    = plot_r0;
    p.c0    = c0;
    p.mid_r = plot_r0 + PHASE_PLOT_H / 2;
    p.mid_c = c0 + 1 + PHASE_PLOT_W / 2;
    p.th_to_cells = ((float)PHASE_PLOT_W * 0.5f) / PHASE_TH_RANGE;
    p.om_to_cells = ((float)PHASE_PLOT_H * 0.5f) / PHASE_OM_RANGE;
    return p;
}

/* Cross-hair axes + origin '+' + four axis labels. Painted first so
 * the trail and current point overlay them. */
static void phase_draw_axes(const PhasePlot *p, int rows, int cols)
{
    /* Vertical θ=0 line, horizontal ω=0 line, origin glyph */
    for (int i = 0; i < PHASE_PLOT_H; i++)
        put_ch(p->r0 + i, p->mid_c, '|', A_DIM, CP_DIM, rows, cols);
    for (int i = 0; i < PHASE_PLOT_W; i++)
        put_ch(p->mid_r, p->c0 + 1 + i, '-', A_DIM, CP_DIM, rows, cols);
    put_ch(p->mid_r, p->mid_c, '+', A_DIM, CP_DIM, rows, cols);

    /* Labels at the four extremes */
    put_str(p->mid_r - 1, p->c0 + 1,                  "-θ", A_DIM, CP_DIM, rows, cols);
    put_str(p->mid_r - 1, p->c0 + PHASE_PLOT_W,       "+θ", A_DIM, CP_DIM, rows, cols);
    put_str(p->r0,                p->mid_c - 2,       "+ω", A_DIM, CP_DIM, rows, cols);
    put_str(p->r0 + PHASE_PLOT_H - 1, p->mid_c - 2,   "-ω", A_DIM, CP_DIM, rows, cols);
}

/* Map (θ, ω) into cell coordinates inside the plot box. ω's sign
 * flips because screen-y grows DOWN while ω grows UP.               */
static inline int phase_plot_col(const PhasePlot *p, float theta)
{
    return p->mid_c + (int)(theta * p->th_to_cells);
}
static inline int phase_plot_row(const PhasePlot *p, float omega)
{
    return p->mid_r - (int)(omega * p->om_to_cells);
}

/* Pick a trail glyph colour by sample age:
 *   newest 15%  → red    (age in TRAIL_NEW_BAND .. 1.0)
 *   middle 35%  → yellow (age in TRAIL_MID_BAND .. TRAIL_NEW_BAND)
 *   oldest 50%  → dim grey
 * Encodes time direction in an otherwise static portrait. */
static inline int trail_color_for_age(float age01)
{
    if (age01 > TRAIL_NEW_BAND) return CP_WARN;
    if (age01 > TRAIL_MID_BAND) return CP_VAL;
    return CP_DIM;
}

/* Walk the ring buffer oldest → newest and paint each sample. */
static void phase_draw_trail(const PhaseTrail *tr, const PhasePlot *p,
                             int rows, int cols)
{
    int n = tr->fill;
    for (int i = 0; i < n; i++) {
        int   idx = (tr->head - n + i + HIST_LEN) % HIST_LEN;
        int   pc  = phase_plot_col(p, tr->theta[idx]);
        int   pr  = phase_plot_row(p, tr->omega[idx]);
        float age = (float)i / (float)n;
        put_ch(pr, pc, '.', A_NORMAL, trail_color_for_age(age), rows, cols);
    }
}

/* Stamp the CURRENT (θ, ω) point as a bright '@' on top. */
static void phase_draw_current_point(const Pendulum *pe, const PhasePlot *p,
                                     int rows, int cols)
{
    int pc = phase_plot_col(p, pe->theta);
    int pr = phase_plot_row(p, pe->omega);
    put_ch(pr, pc, '@', A_BOLD, CP_BEACON, rows, cols);
}

/* Regime classification — one-line label + colour describing what
 * the trajectory is doing. Order matters: PID-off first, then damping
 * extremes, then "converged vs settling" by error magnitude. */
static const char *phase_classify_regime(const Bot *b, int *out_cp)
{
    if (!b->pid.on)                     { *out_cp = CP_WARN; return "PID OFF (free fall)";  }
    if (b->pid.kd < KD_UNDERDAMP_LIMIT) { *out_cp = CP_VAL;  return "UNDERDAMPED (no Kd)";  }
    if (b->pid.kd > KD_OVERDAMP_LIMIT)  { *out_cp = CP_VAL;  return "OVERDAMPED (high Kd)"; }

    float abs_err = fabsf(b->pend.theta - b->pid.setpoint);
    if (abs_err < CONVERGED_ERR_RAD)    { *out_cp = CP_GOOD; return "STABLE — converged";   }
    *out_cp = CP_VAL;                                        return "SETTLING — correcting";
}

/*
 * render_panel_phase — phase-space portrait of (θ, ω).
 *
 *   stable     →  inward spiral converging to origin
 *   underdamped →  wide circles slowly shrinking
 *   overdamped →  slow direct path to origin
 *   drift      →  trajectory wanders off-axis (steady-state error)
 *
 * Reads as a five-step pipeline: header, axes, trail, current point,
 * text panel.
 */
static void render_panel_phase(const Bot *b, int rows, int cols, int c0)
{
    char buf[64];
    int  r = 0;

    /* (1) header */
    r = panel_text_row(r, c0, " PHASE PORTRAIT         ",
                       A_BOLD, CP_VAL, rows, cols);
    r = panel_text_row(r, c0, " (θ vs ω)               ",
                       A_DIM,  CP_DIM, rows, cols);

    /* (2) plot geometry + axes */
    PhasePlot plot = phase_configure(r, c0);
    phase_draw_axes(&plot, rows, cols);

    /* (3) trail (oldest → newest, newest paints on top) */
    phase_draw_trail(&b->trail, &plot, rows, cols);

    /* (4) current point */
    phase_draw_current_point(&b->pend, &plot, rows, cols);

    /* (5) text panel below the plot */
    r = plot.r0 + PHASE_PLOT_H + 1;

    int regime_cp;
    const char *regime = phase_classify_regime(b, &regime_cp);
    r = panel_text_row(r, c0 + 1, regime, A_BOLD, regime_cp, rows, cols);

    snprintf(buf, sizeof buf, " θ=%+.3f ω=%+.3f",
             b->pend.theta, b->pend.omega);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

    /* Active preset's mini-lesson — links gain change ↔ trail shape. */
    r++;
    const GainPreset *pp = &PRESETS[b->ui.preset_idx];
    snprintf(buf, sizeof buf, " [%s]", pp->name);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
    r = panel_text_row(r, c0 + 1, pp->lesson_l1, A_DIM, CP_DIM, rows, cols);
    r = panel_text_row(r, c0 + 1, pp->lesson_l2, A_DIM, CP_DIM, rows, cols);
}

/* ── §7.7 render_hud — top-row status + bottom-row hints ────────── */

/* One-line status string for the HUD top row. Order chosen so the
 * eye lands on FPS first, then the mode words, then the live numbers. */
static void hud_format_status(const Bot *b, double fps, char *buf, size_t n)
{
    /* Distance to the fall threshold — "how much margin do I have?". */
    float margin_deg = (FALL_ANGLE - fabsf(b->cp.theta_eff))
                       * (180.0f / (float)M_PI);

    const char *mode = b->ui.paused ? "PAUSED "
                     : b->ui.fallen ? "FALLEN " : "running";
    const char *pid_flag = b->pid.on ? "" : "  NO-PID ";

    snprintf(buf, n,
             " %5.1f fps  %s%s  α=%+5.1f°  margin=%4.1f°  "
             "dist=%5.1fm  view=[%s]  preset=%s ",
             fps, mode, pid_flag,
             b->mot.alpha * (180.0f / (float)M_PI),
             margin_deg, b->mot.dist_m,
             VIEW_NAMES[b->ui.view], PRESETS[b->ui.preset_idx].name);
}

/* Paint the HUD top row (yellow bold), then pad to the right edge so
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
             " q:quit  spc:pause  r:reset  ↑↓:speed  "
             "p:PID  m:view  g:preset  +/-:Kp ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* render_hud — the two-line UI frame: status row 0, hint row bottom. */
static void render_hud(const Bot *b, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];
    hud_format_status(b, fps, buf, sizeof buf);
    hud_paint_status_row(buf, cols);
    hud_paint_hint_strip(rows);
}

/* ── §7.8 scene_draw — full frame ─────────────────────────────────── */

/* Left column of the right-side panel — clamped to column 1 if the
 * terminal is too narrow to fit PANEL_W. */
static int side_panel_origin_col(int cols)
{
    int c0 = cols - PANEL_W;
    return c0 < 1 ? 1 : c0;
}

/* Dispatch to the right side-panel renderer for the active view mode. */
static void render_side_panel(const Bot *b, int rows, int cols)
{
    int c0 = side_panel_origin_col(cols);
    switch (b->ui.view) {
    case VIEW_TELEMETRY: render_panel_telemetry(b, rows, cols, c0); break;
    case VIEW_EQUATIONS: render_panel_equations(b, rows, cols, c0); break;
    case VIEW_PHASE:     render_panel_phase    (b, rows, cols, c0); break;
    default: break;
    }
}

/*
 * scene_draw — one frame, painter's order.
 *
 *   1. erase the back buffer
 *   2. backdrop: terrain (sky + surface + rock fill)
 *   3. foreground: the robot
 *   4. right-side panel for the active view mode
 *   5. HUD frame (status row 0 + hint row last)
 *
 * Reads exactly as the rendering pipeline; no scattered coord math.
 */
static void scene_draw(const Bot *b, const Terrain *t, double fps,
                       int bot_sc, int rows, int cols)
{
    erase();

    int bot_wc = (int)(b->mot.world_x / (float)CELL_W);
    render_terrain  (t, bot_wc, bot_sc, rows, cols);   /* (2) */
    render_bot      (b, t,      bot_sc, rows, cols);   /* (3) */
    render_side_panel(b,                rows, cols);   /* (4) */
    render_hud      (b, fps,            rows, cols);   /* (5) */
}

/* ===================================================================== */
/* §8  screen — ncurses init / present                                    */
/* ===================================================================== */

/*
 * Screen — the current ncurses window dimensions.
 *
 * INTENT
 *   The single point of truth for "how big is the terminal RIGHT
 *   NOW". Every renderer that takes (rows, cols) parameters
 *   ultimately reads them from here via the Scene pointer in main().
 *
 * WHY A STRUCT FOR TWO INTS
 *   Two reasons:
 *     (1) Pairs them — renderers always need both together; passing
 *         a Screen* is one parameter instead of two.
 *     (2) Localises resize handling — SIGWINCH writes to one struct,
 *         every subsequent frame sees the new geometry automatically.
 *   Same argument applies in reverse to FrameTimer (four loose
 *   timing ints became one named struct).
 *
 * MUTATION RULE
 *   These two fields change ONLY inside screen_init() and
 *   screen_resize(). They never change mid-frame — render and
 *   physics treat them as immutable for the duration of one tick.
 */
typedef struct {
    int cols;               /* terminal width in character cells     */
    int rows;               /* terminal height in character cells    */
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
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
/* §9  scene — FrameTimer + Scene container; signals, resize, main loop   */
/* ===================================================================== */

/*
 * FrameTimer — wall-clock bookkeeping for the main loop.
 *
 * INTENT
 *   Holds the two timing concerns that used to be local accumulators
 *   in main():
 *     (1) measuring dt between frames so physics can sub-step at a
 *         fixed maximum integrator step (TICK_DT_TARGET = 1/120 s);
 *     (2) computing a rolling-window FPS for the HUD that doesn't
 *         flicker once-per-frame.
 *
 *   Pulling these into a named struct turns five disconnected
 *   variables-in-a-function into one object the loop body can pass
 *   around or hand off if the loop ever grows.
 *
 * WHY A 0.5-SECOND ROLLING WINDOW (not instantaneous fps)
 *   Per-frame fps fluctuates wildly — one slow frame reads as
 *   "32 fps", one fast frame as "200 fps". A 0.5-sec window smooths
 *   this so the HUD value is legible and useful for actual tuning.
 *   Window length is a tradeoff: shorter = jumpier, longer = stale.
 *
 * ALGORITHM REFERENCE
 *   Glenn Fiedler ("Gaffer on Games"), "Fix Your Timestep!" (2004).
 *   FrameTimer.last_ns is the "previous frame timestamp" from that
 *   article; main()'s accumulator + sub-step structure is the
 *   "free the physics from the renderer" pattern verbatim.
 *   https://gafferongames.com/post/fix_your_timestep/
 */
typedef struct {
    int64_t last_ns;        /* clock_ns() reading at the START of    *
                              * the previous frame. Each frame:       *
                              *   dt_ns   = now_ns - last_ns          *
                              *   last_ns = now_ns                    *
                              * Reset after resize so the post-resize *
                              * frame doesn't see a huge dt spike.    */

    int64_t fps_accum_ns;   /* sum of frame durations (in ns) since   *
                              * the last fps_display update. Once it  *
                              * exceeds NS_PER_SEC / 2 (= 0.5 sec) we *
                              * compute a new fps_display and reset.  */

    int     fps_frames;     /* count of frames included in            *
                              * fps_accum_ns.                          *
                              *   fps = fps_frames · 1e9 / fps_accum_ns*
                              * Reset together with fps_accum_ns.     */

    double  fps_display;    /* most recent rolling-window fps value, *
                              * shown in the HUD top row. Updated     *
                              * roughly twice per second; stays       *
                              * legible to a human glance.            */
} FrameTimer;

/*
 * Scene — the top-level container.
 *
 * INTENT
 *   One single struct owns every long-lived piece of state the
 *   program needs. A single global instance (g_scene) replaces
 *   what used to be TWO file-scope globals (g_perm + g_app). Signal
 *   handlers flip the volatile flags on this instance; main() reads
 *   them. Every render and physics function takes pointers into
 *   Scene members explicitly, so each function's data dependencies
 *   are visible at the call site — no hidden globals.
 *
 * MEMBERS (in conceptual layer order)
 *
 *      Noise       Perlin permutation context (formerly g_perm)
 *      Terrain     scrolling heightfield, borrows Noise*
 *      Bot         six concept-sized sub-structs (see §6.2.7)
 *      Screen      ncurses dimensions, refreshed on SIGWINCH
 *      FrameTimer  dt + rolling-fps state for the main loop
 *      bot_sc      camera column (where the bot is drawn on screen)
 *      running     SIGINT / SIGTERM / ESC clears this
 *      need_resize SIGWINCH sets this
 *
 * WHY A GLOBAL AT ALL
 *   Signal handlers can't take parameters and must run as fast as
 *   possible (POSIX async-signal-safety rules). They need a path
 *   to the running/need_resize flags. A single file-scope Scene
 *   gives them that without complicating the rest of the API
 *   (which passes Scene* explicitly everywhere it matters).
 *
 * INITIALIZATION ORDER (see main())
 *   1. noise_init    populate the permutation table from time(NULL)
 *   2. screen_init   bring ncurses up, learn rows/cols
 *   3. terrain_init  needs noise AND screen.rows; fills the ring buf
 *                    ahead of the bot
 *   4. bot_init      physics state; depends on nothing external
 *   5. timer.last_ns seed for dt measurement
 *   6. bot_sc        derived from screen.cols (cols/3)
 *
 * INVARIANT
 *   No render or physics function ever reads a Scene field directly
 *   via g_scene — they always receive a pointer (Bot*, Terrain*,
 *   Scene*) so the call graph documents every dependency. The
 *   global is touched only by signal handlers and by main().
 */
typedef struct {
    Noise                 noise;       /* seeded permutation table — *
                                         * Scene owns the only        *
                                         * instance in the program.   */

    Terrain               terrain;     /* heightfield. Holds a       *
                                         * borrowed `const Noise*`    *
                                         * pointing back to .noise    *
                                         * above.                     */

    Bot                   bot;         /* simulation state — six     *
                                         * concept-sized sub-structs. *
                                         * See §6.2.7.                */

    Screen                screen;      /* ncurses dimensions, kept   *
                                         * in sync via SIGWINCH.      */

    FrameTimer            timer;       /* dt + rolling-fps state.    */

    int                   bot_sc;      /* screen column where the    *
                                         * bot is rendered — the     *
                                         * camera follows by holding  *
                                         * the bot at this column.    *
                                         * Set to cols/3 on init and  *
                                         * after each resize.         */

    volatile sig_atomic_t running;     /* main-loop continue flag.   *
                                         * Cleared by SIGINT/SIGTERM  *
                                         * (on_exit_signal) or by the *
                                         * 'q' / Q / ESC keys.        *
                                         * volatile + sig_atomic_t    *
                                         * for async-signal safety.   */

    volatile sig_atomic_t need_resize; /* set by SIGWINCH handler    *
                                         * (on_resize_signal). Main   *
                                         * loop reads + clears it at  *
                                         * the top of each iteration  *
                                         * and re-syncs screen/bot_sc.*/
} Scene;

static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Generic clamp to [lo, hi]. Used by the speed/Kp keyboard nudges so
 * the bounds check is one line, not a four-line pair of ifs. */
static inline float clamp_range(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Toggle the PID controller. Integral is cleared on disable so
 * re-enabling doesn't see a stale accumulation. */
static void key_pid_toggle(Bot *b)
{
    b->pid.on    = !b->pid.on;
    b->pid.integ = 0.0f;
}

/* Cycle through the three side-panel views (TELEMETRY/EQUATIONS/PHASE). */
static void key_view_cycle(Bot *b)
{
    b->ui.view = (ViewMode)((b->ui.view + 1) % VIEW_COUNT);
}

/* Cycle through gain presets and apply the new one (clears integ). */
static void key_preset_cycle(Bot *b)
{
    b->ui.preset_idx = (b->ui.preset_idx + 1) % N_PRESETS;
    bot_apply_preset(b, b->ui.preset_idx);
}

/* Full reset — bot back to initial conditions AND regenerate terrain. */
static void key_reset_simulation(Scene *scene)
{
    bot_reset(&scene->bot);
    terrain_init(&scene->terrain, &scene->noise,
                 scene->screen.rows, scene->screen.cols);
}

/* ↑/↓ — nudge drive speed by ±DRIVE_STEP, clamped to [DRIVE_MIN, MAX]. */
static void key_drive_speed_nudge(Bot *b, float delta)
{
    b->mot.drive_spd = clamp_range(b->mot.drive_spd + delta,
                                   DRIVE_MIN, DRIVE_MAX);
}

/* +/- — nudge Kp by ±KP_NUDGE, clamped to [0, KP_MAX_USER]. */
static void key_kp_nudge(Bot *b, float delta)
{
    b->pid.kp = clamp_range(b->pid.kp + delta, 0.0f, KP_MAX_USER);
}

/*
 * scene_handle_key — dispatch one keystroke to its named action.
 * Each case is one helper call; the switch reads as the keymap.
 */
static void scene_handle_key(Scene *scene, int ch)
{
    Bot *b = &scene->bot;
    switch (ch) {
    case 'q': case 'Q': case 27:  scene->running = 0;                    break;
    case ' ':                     b->ui.paused = !b->ui.paused;          break;
    case 'p': case 'P':           key_pid_toggle(b);                     break;
    case 'r': case 'R':           key_reset_simulation(scene);           break;
    case 'm': case 'M':           key_view_cycle(b);                     break;
    case 'g': case 'G':           key_preset_cycle(b);                   break;
    case KEY_UP:                  key_drive_speed_nudge(b, +DRIVE_STEP); break;
    case KEY_DOWN:                key_drive_speed_nudge(b, -DRIVE_STEP); break;
    case '+': case '=':           key_kp_nudge(b, +KP_NUDGE);            break;
    case '-':                     key_kp_nudge(b, -KP_NUDGE);            break;
    default:                                                             break;
    }
}

/* Re-sync everything to the new terminal size. Called from the main
 * loop when SIGWINCH has set need_resize. */
static void scene_handle_resize(Scene *scene)
{
    screen_resize(&scene->screen);
    scene->bot_sc        = scene->screen.cols / 3;
    scene->terrain.rows  = scene->screen.rows;
    scene->need_resize   = 0;
    scene->timer.last_ns = clock_ns();   /* avoid huge dt next frame */
}

/* Measure wall-clock dt since last frame and feed the rolling-fps
 * accumulator. Returns dt clamped to DT_CAP_SEC (spiral-of-death
 * guard) and writes the raw dt_ns to *out_dt_ns for the fps update. */
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

/* Ensure terrain columns are generated ahead of the camera so the
 * renderer never reads an uninitialised slot. The "+ 32" margin is
 * cheap insurance — fbm() is fast and the ring buffer overwrites
 * old columns we no longer see. */
static void ensure_terrain_ahead(Scene *scene)
{
    int upto = (int)(scene->bot.mot.world_x / (float)CELL_W)
             + scene->screen.cols + 32;
    terrain_ensure(&scene->terrain, upto);
}

/* Advance physics with adaptive sub-stepping for integrator stability.
 *   n_sub = ceil(dt / TICK_DT_TARGET), capped at MAX_SUBSTEPS so a
 *   stalled frame can't loop forever. Each substep is ≤ TICK_DT_TARGET,
 *   which keeps forward-Euler well within its stability region. */
static void scene_advance_physics(Scene *scene, float dt)
{
    int n_sub = (int)ceilf(dt / TICK_DT_TARGET);
    if (n_sub < 1)            n_sub = 1;
    if (n_sub > MAX_SUBSTEPS) n_sub = MAX_SUBSTEPS;

    float sub_dt = dt / (float)n_sub;
    for (int i = 0; i < n_sub; i++)
        bot_substep(&scene->bot, &scene->terrain, sub_dt);
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
 *     1. install signal handlers and atexit cleanup
 *     2. initialise subsystems in dependency order:
 *        Noise → Screen → Terrain (needs both) → Bot → camera + timer
 *
 *   LOOP (each frame):
 *     1. handle any pending resize
 *     2. measure dt (capped) + tick fps
 *     3. drain keyboard input
 *     4. ensure terrain is generated ahead of the camera
 *     5. advance physics (sub-stepped) unless paused/fallen
 *     6. draw the scene + present
 *     7. sleep to hit TARGET_FPS
 */
int main(void)
{
    /* SETUP — signals + atexit */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* SETUP — subsystems in dependency order */
    Scene *scene = &g_scene;
    scene->running = 1;

    noise_init  (&scene->noise, (unsigned int)time(NULL));
    screen_init (&scene->screen);
    terrain_init(&scene->terrain, &scene->noise,
                 scene->screen.rows, scene->screen.cols);
    bot_init    (&scene->bot);

    scene->bot_sc        = scene->screen.cols / 3;
    scene->timer.last_ns = clock_ns();

    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    /* LOOP */
    while (scene->running) {
        /* (1) handle resize before reading anything else */
        if (scene->need_resize) scene_handle_resize(scene);

        /* (2) frame timing — dt for physics, raw dt_ns for fps */
        int64_t now_ns;
        int64_t dt_ns;
        now_ns = clock_ns();
        float dt = frame_measure_dt(&scene->timer, now_ns, &dt_ns);
        frame_tick_fps(&scene->timer, dt_ns);

        /* (3) drain queued keystrokes */
        scene_drain_input(scene);

        /* (4) generate terrain columns ahead of the camera */
        ensure_terrain_ahead(scene);

        /* (5) advance physics (paused/fallen → skip) */
        if (!scene->bot.ui.paused && !scene->bot.ui.fallen)
            scene_advance_physics(scene, dt);

        /* (6) draw + present */
        scene_draw(&scene->bot, &scene->terrain, scene->timer.fps_display,
                   scene->bot_sc, scene->screen.rows, scene->screen.cols);
        screen_present();

        /* (7) sleep the remainder of the frame budget */
        frame_cap_to_target_fps(now_ns, TICK_NS);
    }

    screen_free(&scene->screen);
    return 0;
}
