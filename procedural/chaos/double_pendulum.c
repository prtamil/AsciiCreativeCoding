/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * double_pendulum.c
 *   — The canonical mechanical-chaos demo.  Two coupled pendulums
 *     swung from a single anchor; the lower bob's path traces a
 *     long fading trail.  30 named initial conditions n/p-cyclable
 *     across six behavioural classes — tiny changes in the starting
 *     (θ₁, θ₂, ω₁, ω₂) produce wildly different long-term motion,
 *     "sensitive dependence on initial conditions" made visible.
 *
 * DEMO: A double pendulum is governed by a 4-D nonlinear ODE
 *         dθ₁/dt = ω₁,  dω₁/dt = f₁(θ₁,θ₂,ω₁,ω₂)
 *         dθ₂/dt = ω₂,  dω₂/dt = f₂(θ₁,θ₂,ω₁,ω₂)
 *       integrated with RK4 at fixed dt.  30 presets in 6 groups —
 *       small oscillation, moderate swings, near-separatrix
 *       transition, full chaos, high-energy rotation, exotic
 *       asymmetric.  n/p cycle; default opens on PRESET_CHAOS.
 *
 * Study alongside:
 *   ../fractals/lyapunov.c       — Lyapunov fractal: the *quantitative*
 *                                  measure of chaos as a 2-D image.
 *   ../fields/marching_squares_isocontours_showcase.c — Tier 5
 *                                  PENDULUM pattern shows the SINGLE
 *                                  pendulum's phase portrait; this
 *                                  file demonstrates what happens when
 *                                  you couple two of them.
 *   ./bifurcation.c              — discrete-map chaos (logistic) for
 *                                  comparison with this continuous case.
 *   ./strange_attractor.c        — strange-attractor zoo including the
 *                                  3-D Lorenz ODE (the other canonical
 *                                  chaos example).
 *
 * Section map:
 *   §1 config   — physics, presets, themes, HUD, sim/render budgets
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes (PAIR_TRAIL_BASE+0..3)
 *   §4 coord    — pixel-space ↔ cell-space conversion
 *   §5 physics  — System + Anchor + State sub-structs, derivatives, RK4
 *   §6 trail    — ring buffer of bob-tip pixel positions
 *   §7 state    — PresetState + PaletteState typed wrappers around the
 *                 two cycled-by-key selections (presets[] / themes[])
 *   §8 scene    — Scene composes pendulum + trail + preset + palette;
 *                 tick, reset, resize, theme apply
 *   §9 screen   — draw rods (Bresenham), draw trail, HUD layout
 *   §10 app     — signals, resize, key dispatch table, FrameClock,
 *                 main pseudocode driver and its named loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-apply current preset)
 *   n / N      next preset    (p / P previous)
 *   t / T      next / previous theme
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/double_pendulum.c \
 *       -o double_pendulum -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Lagrangian double pendulum.  State = (θ₁, θ₂, ω₁, ω₂).
 *                 The 4-D nonlinear ODE has no closed-form solution; we
 *                 integrate with RK4 (4th-order Runge-Kutta) at a fixed
 *                 substep dt = 1 / (sim_fps × INT_STEPS_PER_TICK), with
 *                 INT_STEPS_PER_TICK = 8 substeps per render tick to
 *                 keep ω-amplitudes stable when the bob whips around.
 *
 * Data-struct   : DoublePendulum splits into three named sub-structs —
 *                 PendulumSystem (L₁, L₂, m₁, m₂, g — the invariants),
 *                 PendulumAnchor (px, py — the pivot in screen space),
 *                 PendulumState  (θ₁, θ₂, ω₁, ω₂ — the 4-D phase-space
 *                 coordinate that evolves under RK4).  Trail is a ring
 *                 buffer (TRAIL_MAX = 4000) of the lower bob's pixel
 *                 positions — the chaotic figure-eight (or whatever
 *                 shape emerges) draws itself as the buffer fills.
 *
 * Rendering     : ASCII-only.  Each rod is a Bresenham line whose glyph
 *                 is DIRECTION-AWARE — `-` for horizontal steps, `|`
 *                 for vertical, `/` or `\` for diagonals — picked by
 *                 bresenham_glyph_for_step() so the rod reads as a
 *                 coherent line at any angle.  Upper and lower rods
 *                 differ in COLOUR, not glyph (PAIR_ROD_TOP vs
 *                 PAIR_ROD_BOT).  The trail fades through
 *                 TRAIL_BAND_COUNT = 4 colour-and-glyph bands
 *                 (`,` `.` `.` `o`) from oldest to newest.  Pivot,
 *                 joint, and bob are bracketed glyphs: `[+]`, `O`, `(@)`.
 *
 * Performance   : ~4 µs per RK4 step on modern hardware.  At INT_STEPS_
 *                 PER_TICK = 8 substeps × 60 tick/sec = 480 RK4
 *                 evaluations/sec = ~2 ms/sec — trivial.  Trail draw is
 *                 O(TRAIL_MAX) Bresenham points per frame.
 *
 * References    : DOUBLE-PENDULUM CHAOS  (the algorithm)
 *                 [1] Shinbrot, T., Grebogi, C., Wisdom, J., Yorke, J. A.
 *                     (1992) — "Chaos in a double pendulum",
 *                     Am. J. Phys. 60(6), pp. 491-499.  The standard
 *                     reference: full Lagrangian derivation of the
 *                     equations of motion implemented in pendulum_deriv,
 *                     identification of the separatrix that divides
 *                     regular swing from rotation, photographs of the
 *                     scribbled bob trajectories that the §6 trail mimics.
 *                 [2] Levien, R. B., Tan, S. M. (1993) — "Double pendulum:
 *                     An experiment in chaos", Am. J. Phys. 61(11),
 *                     pp. 1038-1044.  Experimental Poincaré-section
 *                     plots — useful for picking initial conditions for
 *                     the 30-preset zoo in §1.
 *                 [3] Strogatz, S. H. (2015) — "Nonlinear Dynamics and
 *                     Chaos" (2nd ed.), Westview Press, §9-10.
 *                     Hamiltonian / Lagrangian chapter covers separatrix,
 *                     KAM tori, and the mode-mixing pattern the
 *                     small-amplitude PARALLEL/MIRROR presets exhibit.
 *
 *                 LAGRANGIAN MECHANICS  (where the equations come from)
 *                 [4] Goldstein, H., Poole, C., Safko, J. (2001) —
 *                     "Classical Mechanics" (3rd ed.), Addison-Wesley,
 *                     Ch. 1-2.  The textbook derivation of the
 *                     Euler-Lagrange equations and worked example for a
 *                     compound pendulum.
 *                 [5] Landau, L. D., Lifshitz, E. M. (1976) — "Mechanics"
 *                     (3rd ed.), Pergamon, §5-9.  Compact derivation of
 *                     the same physics, with the small-oscillation normal
 *                     modes used to choose the Group-A preset values.
 *
 *                 NUMERICAL INTEGRATION  (RK4 + fixed-timestep loop)
 *                 [6] Press, W. H., Teukolsky, S. A., Vetterling, W. T.,
 *                     Flannery, B. P. (2007) — "Numerical Recipes"
 *                     (3rd ed.), Cambridge, Ch. 17.1.  RK4 derivation,
 *                     stability properties, and the 4-stage Butcher
 *                     tableau implemented in pendulum_rk4().
 *                 [7] Fiedler, G. (2004) — "Fix Your Timestep!",
 *                     gafferongames.com.  The accumulator pattern used
 *                     by app_drain_fixed_timestep() — decouples
 *                     render rate (variable) from physics rate (constant)
 *                     so the chaos is deterministic across frame rates.
 *
 *                 RENDERING & TERMINAL I/O  (how the math reaches the screen)
 *                 [8] Bresenham, J. E. (1965) — "Algorithm for computer
 *                     control of a digital plotter", IBM Systems Journal
 *                     4(1), pp. 25-30.  The integer-only line-stepping
 *                     algorithm used by draw_rod() in §9 (extended with
 *                     a direction-aware glyph choice in {`-`, `|`, `/`, `\`}).
 *                 [9] Gookin, D. (2007) — "Programmer's Guide to NCurses",
 *                     Wiley.  The double-buffered rendering model
 *                     (wnoutrefresh + doupdate), color-pair theory, and
 *                     SIGWINCH handling used throughout §3 and §9-10.
 *
 *                 COMPARE IN PROJECT  (the chaos zoo)
 *                 • ./bifurcation.c       — DISCRETE 1-D map chaos
 *                                           (logistic map period doubling).
 *                 • ./cobweb_diagram.c    — graphical iteration of 1-D maps,
 *                                           shares the PresetState pattern.
 *                 • ./strange_attractor.c — 3-D ODE chaos (Lorenz + 8 more),
 *                                           uses the same RK4 helper shape.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Hang a pendulum from another pendulum.  The bottom bob's motion is
 * governed by 4 coupled equations and has no closed-form solution.
 * Two trajectories started ε apart in angle stay ε apart for a few
 * swings and then diverge by 2^(t/T_λ) where T_λ is the Lyapunov time.
 * Within seconds, the bottom bob's path becomes wildly different.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Energy is conserved (no friction).  At LOW total energy, both bobs
 * swing back and forth near vertical — periodic, integrable-like
 * motion.  At HIGH total energy, the second pendulum can rotate all
 * the way over the top: now the system explores a 3-D constant-energy
 * surface in 4-D phase space, and that exploration is CHAOTIC.
 *
 * ALGORITHM IN STEPS  (per sim tick — pendulum_rk4 + scene_tick)
 * ──────────────────
 *  1. slope_start    = f(state)                                ← STAGE 1
 *  2. slope_mid_1    = f(state + ½·dt·slope_start)             ← STAGE 2
 *  3. slope_mid_2    = f(state + ½·dt·slope_mid_1)             ← STAGE 3
 *  4. slope_end      = f(state +    dt·slope_mid_2)            ← STAGE 4
 *  5. state += dt · (slope_start + 2·slope_mid_1
 *                     + 2·slope_mid_2 + slope_end) / 6         ← UPDATE
 *  6. Forward-KINEMATICS (exact sin/cos, NOT Euler) of the
 *     new (θ₁, θ₂) → bob-2 pixel position → trail_push().
 *
 * KEY FORMULAS  (with m₁=m₂=m, L₁=L₂=L, g=gravity)
 * ────────────
 *   Δ = θ₁ − θ₂
 *   den = 2m + m − m·cos(2Δ) = m(3 − cos(2Δ))
 *
 *   dω₁/dt = (−g(2m+m)sin θ₁ − m·g·sin(θ₁−2θ₂)
 *             − 2·sin(Δ)·m·(ω₂²L + ω₁²L·cos Δ)) / (L·den)
 *
 *   dω₂/dt = (2·sin(Δ)·(ω₁²L(m+m) + g(m+m)cos θ₁
 *             + ω₂²L·m·cos Δ)) / (L·den)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • dt too large → energy drift (RK4 conserves energy only to O(dt⁴)).
 *    Pick dt ≤ 0.005 s for visually stable behaviour on this scale.
 *  • cos(2Δ) → 1 makes den → 2m (no singularity — but stiff when bobs
 *    are aligned and spinning fast).
 *  • Trail length too long → cluttered; too short → can't see the
 *    figure.  TRAIL_MAX = 4000 is a good middle ground.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Group-A preset (e.g. WHISPER, MIRROR): bottom bob traces a
 *    slowly precessing oval (quasi-periodic).  Total energy roughly
 *    constant across a minute.
 *  • Group-D preset (CHAOS, STORM, RIOT): bottom bob's trail fills
 *    the whole reachable region within ~20 s; energy roughly
 *    constant across a minute.
 *  • Reset (r) with the same preset → same trajectory (deterministic).
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,   /* 4 trail-age band slots: +0..+3       */
    PAIR_ROD_TOP      =   7,
    PAIR_ROD_BOT      =   8,
    PAIR_BOB          =   9,
    PAIR_PIVOT        =  10,
    PAIR_JOINT        =  11,   /* intermediate joint 'O' between rods  */
};

/* HUD layout */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Pixel-space: each character cell is CELL_W × CELL_H sub-pixels.
 * Continuous physics happens in pixel coords; we divide by these to
 * map to terminal cells.  Matches the project's standard. */
#define CELL_W                   8
#define CELL_H                  16

/* Render budget: target 60 fps composite. */
#define RENDER_FPS_TARGET       60
#define NS_PER_SEC              1000000000LL
#define NS_PER_MS                  1000000LL
#define TICK_NS(f)              (NS_PER_SEC / (f))
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS    100   /* spiral-of-death cap */

/* Physics — masses are fixed dimensionless 1; arm length L (in
 * pixels) and gravity g (in px / s²) are derived from the current
 * terminal size in scene_compute_geometry() so the pendulum fills
 * the visible area at any resolution.  Period stays at ~2 s
 * regardless of size: small-angle T = 2π√(L/g), and we lock the
 * ratio g/L so the visual feel is consistent. */
#define PEND_M1                   1.0f   /* bob 1 mass (dimensionless)  */
#define PEND_M2                   1.0f   /* bob 2 mass (dimensionless)  */

/* Arm length, expressed as a fraction of the drawable height.  Each
 * arm gets ARM_CELLS_OF_DRAWABLE × (drawable_h cells).  Both arms
 * combined must fit within the HALF-height above the centred pivot
 * so the CHAOS preset (lower bob loops over the top) has room.
 * Clamped to [MIN, MAX] cells for tiny or huge terminals. */
#define ARM_CELLS_OF_DRAWABLE     0.22f
#define ARM_CELLS_MIN                4
#define ARM_CELLS_MAX               14

/* Gravity-to-length ratio — keeps the natural small-angle period
 * T = 2π√(L/g) ≈ 2.0 s at ANY arm length:
 *     T = 2 s ⇒ g / L = (2π / T)² ≈ 9.87  →  rounded to 10 for
 *     a clean visual cadence. */
#define PEND_GRAVITY_PER_LENGTH  10.0f

/* Integration substeps per sim tick.  At sim_fps = 60, this gives
 * dt_substep = 1/(60·INT_STEPS_PER_TICK) ≈ 2 ms — well within RK4
 * stability for the chaotic energy regime even at large L. */
#define INT_STEPS_PER_TICK         8

/* Trail ring buffer — how many recent bob-2 positions to keep. */
#define TRAIL_MAX               4000
#define TRAIL_BAND_COUNT           4

/*
 * Preset — enum index into the 30-entry presets[] table.
 *
 * Each preset's (θ₁, θ₂, ω₁, ω₂) lands in a DISTINCT visual class so
 * cycling through with n/p tours the full behavioural zoo of a
 * double pendulum:
 *
 *   Group A  small-amplitude oscillation modes        (6 presets)
 *   Group B  moderate swings + pure-velocity kicks    (6 presets)
 *   Group C  near-separatrix transition               (5 presets)
 *   Group D  full chaotic regime                      (7 presets)
 *   Group E  high-energy rotational                   (4 presets)
 *   Group F  exotic asymmetric                        (2 presets)
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Shinbrot et al. (1992)        — surveys the small-amp normal
 *                                       modes (PARALLEL, MIRROR) and the
 *                                       chaos onset above the separatrix
 *                                       (CHAOS, CASCADE, TWIST).
 *   [2] Levien & Tan (1993)           — Poincaré-section maps used to
 *                                       pick the Group-D chaotic initial
 *                                       conditions (CHAOS, STORM, RIOT).
 *   [3] Strogatz §9-10                — mode-mixing pattern that BEATING
 *                                       exhibits and the route from
 *                                       regular swing to rotation.
 *   [5] Landau & Lifshitz §5-9        — small-oscillation normal-mode
 *                                       analysis used for Group A.
 */
typedef enum {
    /* Group A — small-amplitude regular oscillation (integrable-like) */
    PRESET_WHISPER = 0,   /* (5°, 0°, 0, 0)        top-only tiny tap   */
    PRESET_MIRROR,        /* (15°, -15°, 0, 0)     pure anti-phase     */
    PRESET_PARALLEL,      /* (20°, 20°, 0, 0)      pure in-phase       */
    PRESET_BEATING,       /* (10°, -25°, 0, 0)     mixed modes → beats */
    PRESET_TWANG,         /* (0°, 30°, 0, 0)       bottom-only release */
    PRESET_KNOCK,         /* (30°, 0°, 0, 0)       top-only release    */

    /* Group B — moderate amplitude + pure-velocity kicks */
    PRESET_SWING,         /* (45°, 30°, 0, 0)      classic in-phase    */
    PRESET_CROSS,         /* (60°, -60°, 0, 0)     wide anti-phase     */
    PRESET_CRESCENT,      /* (80°, 80°, 0, 0)      both near horizontal */
    PRESET_SHEAR,         /* (90°, -45°, 0, 0)     top horizontal      */
    PRESET_NUDGE,         /* (0°, 0°, 0, 2)        pure ω₂ kick        */
    PRESET_PROD,          /* (0°, 0°, 3, 0)        pure ω₁ kick        */

    /* Group C — near-separatrix transition */
    PRESET_PEEL,          /* (45°, 90°, 0, 0)      bottom horizontal   */
    PRESET_ELBOW,         /* (90°, 90°, 0, 0)      L-shape both at 90° */
    PRESET_KINK,          /* (90°, -90°, 0, 0)     L-shape mirrored    */
    PRESET_STAB,          /* (120°, 0°, 0, 0)      top past horizontal */
    PRESET_TIPTOE,        /* (170°, 0°, 0, 0)      top near inverted   */

    /* Group D — full chaotic regime */
    PRESET_CHAOS,         /* (135°, 175°, 0, 0)    classic chaos demo  */
    PRESET_CASCADE,       /* (170°, 170°, 0, 0)    both near top in-phase */
    PRESET_TWIST,         /* (160°, -160°, 0, 0)   both near top mirrored */
    PRESET_STORM,         /* (100°, 200°, 0, 0)    bottom past inverted*/
    PRESET_RUMBLE,        /* (45°, 90°, 3, 0)      angles + top spin   */
    PRESET_SHAKE,         /* (60°, -60°, 0, 3)     counter-angles + spin */
    PRESET_RIOT,          /* (179°, 1°, 0, 0)      knife-edge inverted */

    /* Group E — high-energy rotational */
    PRESET_SPIN,          /* (0°, 0°, 5, 0)        top-only rotation   */
    PRESET_ORBIT,         /* (0°, 0°, 5, 5)        co-rotating both    */
    PRESET_ENGINE,        /* (0°, 0°, 5, -5)       counter-rotating    */
    PRESET_PROPLR,        /* (0°, 0°, 10, 0)       fast top spin       */

    /* Group F — exotic asymmetric */
    PRESET_STAR,          /* (45°, -135°, 3, -3)   opposing angle + spin */
    PRESET_NOVA,          /* (160°, 30°, -4, 6)    wild asymmetric chaos */

    N_PRESETS,
} Preset;

/*
 * PendulumPreset — one row of the 30-entry presets[] table: a named
 * 4-tuple (θ₁, θ₂, ω₁, ω₂) of initial conditions.
 *
 * INTENT
 *   The double-pendulum ODE has identical PARAMETERS (L1, L2, m1, m2, g)
 *   across the whole demo — what changes is WHERE in the 4-D phase
 *   space the integrator starts.  This struct names a single starting
 *   point.  Cycling presets is "pick a different initial condition";
 *   the rest of the system never changes.
 *
 * CONTEXT
 *   Lives only in the const-table presets[] above.  Looked up by
 *   §7's PresetState wrapper (which holds the row index) via
 *   preset_state_active().  Consumed by §5's
 *   pendulum_state_init_from_preset() which converts degrees → radians
 *   and copies the four values into the pendulum's PendulumState.
 *   Never mutated — entirely read-only at runtime.
 *
 * MEMBER LOGIC
 *   name       : 8-char label (space-padded so the HUD row stays
 *                column-aligned across all 30 entries).  Picked to
 *                evoke the visual class — "WHISPER" for tiny taps,
 *                "RIOT" for knife-edge inversion, "NOVA" for wild
 *                asymmetric chaos.  Reading the table top-to-bottom
 *                is itself the documentation of the behavioural zoo.
 *   theta1_deg : angle of the UPPER rod from downward vertical,
 *                in DEGREES (more readable than radians at the table).
 *                0° = hanging straight down, 90° = horizontal,
 *                180° = inverted (straight up).
 *   theta2_deg : angle of the LOWER rod from downward vertical, same
 *                convention.  Both angles are independent — the
 *                lower angle is NOT relative to the upper rod (this
 *                is the standard "Lagrangian-coordinate" convention
 *                of ref [1] Shinbrot, not the "joint-relative" form).
 *   omega1     : initial angular velocity dθ₁/dt in RADIANS / SECOND.
 *                Positive = upper rod swinging clockwise (when viewed
 *                with screen up = real up).  Zero in most presets;
 *                non-zero values inject kinetic energy.
 *   omega2     : initial angular velocity dθ₂/dt, same convention.
 *                In rotational presets (SPIN, ORBIT, PROPLR) the
 *                pair (ω₁, ω₂) sets the spin axis and direction.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Shinbrot eq. (1) — defines (θ₁, θ₂) measured independently
 *       from downward vertical, the convention used by this struct.
 *   [2] Levien & Tan — Poincaré-section plots that guide which
 *       (θ, ω) tuples land in regular vs chaotic regions.
 */
typedef struct {
    const char *name;
    float       theta1_deg;
    float       theta2_deg;
    float       omega1;
    float       omega2;
} PendulumPreset;

/* 30 named initial conditions ordered simple → complex.  Each falls
 * in a distinct VISUAL class so n-cycling through them tours the full
 * behavioural zoo (oscillation modes → moderate swings → separatrix
 * transition → chaotic regime → pure rotation → exotic chaos).  Same
 * pendulum geometry (L1, L2, m1, m2, g) across the whole table —
 * the ONLY things changing are the four initial-condition values. */
static const PendulumPreset presets[N_PRESETS] = {
    /* Group A — small-amplitude regular oscillation ─────────────────── */
    { "WHISPER ",    5.0f,    0.0f,  0.0f,  0.0f },
    { "MIRROR  ",   15.0f,  -15.0f,  0.0f,  0.0f },
    { "PARALLEL",   20.0f,   20.0f,  0.0f,  0.0f },
    { "BEATING ",   10.0f,  -25.0f,  0.0f,  0.0f },
    { "TWANG   ",    0.0f,   30.0f,  0.0f,  0.0f },
    { "KNOCK   ",   30.0f,    0.0f,  0.0f,  0.0f },

    /* Group B — moderate amplitude + pure-velocity kicks ────────────── */
    { "SWING   ",   45.0f,   30.0f,  0.0f,  0.0f },
    { "CROSS   ",   60.0f,  -60.0f,  0.0f,  0.0f },
    { "CRESCENT",   80.0f,   80.0f,  0.0f,  0.0f },
    { "SHEAR   ",   90.0f,  -45.0f,  0.0f,  0.0f },
    { "NUDGE   ",    0.0f,    0.0f,  0.0f,  2.0f },
    { "PROD    ",    0.0f,    0.0f,  3.0f,  0.0f },

    /* Group C — near-separatrix transition ──────────────────────────── */
    { "PEEL    ",   45.0f,   90.0f,  0.0f,  0.0f },
    { "ELBOW   ",   90.0f,   90.0f,  0.0f,  0.0f },
    { "KINK    ",   90.0f,  -90.0f,  0.0f,  0.0f },
    { "STAB    ",  120.0f,    0.0f,  0.0f,  0.0f },
    { "TIPTOE  ",  170.0f,    0.0f,  0.0f,  0.0f },

    /* Group D — full chaotic regime ─────────────────────────────────── */
    { "CHAOS   ",  135.0f,  175.0f,  0.0f,  0.0f },
    { "CASCADE ",  170.0f,  170.0f,  0.0f,  0.0f },
    { "TWIST   ",  160.0f, -160.0f,  0.0f,  0.0f },
    { "STORM   ",  100.0f,  200.0f,  0.0f,  0.0f },
    { "RUMBLE  ",   45.0f,   90.0f,  3.0f,  0.0f },
    { "SHAKE   ",   60.0f,  -60.0f,  0.0f,  3.0f },
    { "RIOT    ",  179.0f,    1.0f,  0.0f,  0.0f },

    /* Group E — high-energy rotational ──────────────────────────────── */
    { "SPIN    ",    0.0f,    0.0f,  5.0f,  0.0f },
    { "ORBIT   ",    0.0f,    0.0f,  5.0f,  5.0f },
    { "ENGINE  ",    0.0f,    0.0f,  5.0f, -5.0f },
    { "PROPLR  ",    0.0f,    0.0f, 10.0f,  0.0f },

    /* Group F — exotic asymmetric ───────────────────────────────────── */
    { "STAR    ",   45.0f, -135.0f,  3.0f, -3.0f },
    { "NOVA    ",  160.0f,   30.0f, -4.0f,  6.0f },
};

/*
 * Theme — colour assignment for every rendered element in one palette.
 *
 * INTENT
 *   Decouples "what gets drawn" (the §9 painters) from "what colour it
 *   should be" so a single cycle key (t/T) swaps the entire visual
 *   identity without changing one line of physics or rendering logic.
 *   Every glyph the user sees gets its colour from exactly one of the
 *   fields below — no glyph is hardcoded to a colour anywhere.
 *
 * CONTEXT
 *   Lives only in the const-table themes[] of N_THEMES entries.
 *   Selected by §7's PaletteState wrapper (which holds the row index)
 *   via palette_state_active().  Pushed into ncurses by theme_apply()
 *   in §3 which calls init_pair(PAIR_*, theme.field, -1) for each
 *   field.  Never mutated at runtime — t/T just changes which row
 *   gets fed to init_pair.
 *
 * MEMBER LOGIC
 *   name           : 8-char label shown on row-1 HUD as the current
 *                    theme name.  Space-padded for column alignment.
 *   band[0..N-1]   : the TRAIL-AGE colour ramp.  band[0] = oldest
 *                    visible trail point (dimmest); band[N-1] = newest
 *                    (brightest).  Must be a monotonic perceptual
 *                    gradient — see §9 trail_paint, which assigns each
 *                    sample to a band by relative age.  Every entry
 *                    must be ≥ 24 in xterm-256 (see CLAUDE.md "Theme
 *                    Palette Brightness") so even the dimmest tier
 *                    stays legible against default-black with A_BOLD.
 *   rod_top        : colour of the UPPER rod glyphs ('-' '|' '/' '\').
 *                    Drawn via PAIR_ROD_TOP after init_pair() copies
 *                    this value in.  Conventionally the brighter of
 *                    the two rod colours so the upper arm reads first.
 *   rod_bot        : colour of the LOWER rod glyphs.  Shifted slightly
 *                    in hue from rod_top so the joint reads as
 *                    "two distinct arms meeting", not one long rod.
 *   joint          : 'O' glyph at the meeting point of the two rods.
 *                    Picked perceptually BETWEEN rod_bot and bob so
 *                    the joint reads as a transition node rather than
 *                    competing with the bob for attention.
 *   bob            : '(@)' bracketed weight at the chain tip — the
 *                    object whose trajectory the trail is recording.
 *                    Usually the most saturated colour in the theme.
 *   pivot          : '[+]' bracketed anchor at the suspension point.
 *                    Picked to read as "fixed reference" — often a
 *                    desaturated grey or a neutral high-luminance shade.
 *
 * REFERENCES
 *   • CLAUDE.md "Theme Palette Brightness" — the project-wide rule
 *     that band[0] must sit at xterm-256 index ≥ 24 to avoid the
 *     "A_DIM = invisible" trap.
 *   • CLAUDE.md "HUD Standard" — PAIR_HUD/PAIR_HINT use bright yellow
 *     + cyan and are NOT per-theme; only the simulation colours
 *     listed in this struct change as themes cycle.
 */
typedef struct {
    const char *name;
    short       band[TRAIL_BAND_COUNT];
    short       rod_top;
    short       rod_bot;
    short       joint;
    short       bob;
    short       pivot;
} Theme;

#define N_THEMES 10

/* All band entries ≥ 39 in the xterm-256 cube so even the dimmest
 * trail tier stays legible on default-black with A_BOLD.  `joint`
 * sits perceptually between rod_bot and bob so the joint reads as
 * "where the two arms meet" rather than competing with the bob. */
static const Theme themes[N_THEMES] = {
    { "DEFAULT",  {  75, 123, 220, 231 }, 220, 208, 214, 196, 231 },
    { "MATRIX",   {  77, 118, 156, 194 }, 156, 118, 119,  82, 231 },
    { "NOVA",     { 135, 171, 207, 219 }, 213, 207, 213, 219, 231 },
    { "MONO",     { 247, 250, 253, 255 }, 254, 253, 254, 255, 231 },
    { "OCEAN",    {  81, 117, 159, 195 }, 159, 117,  81,  51, 231 },
    { "FIRE",     { 208, 214, 220, 227 }, 226, 208, 220, 196, 231 },
    { "EARTH",    { 143, 179, 215, 222 }, 222, 215, 215, 173, 231 },
    { "FOREST",   { 114, 150, 157, 194 }, 156, 150, 150, 119, 231 },
    { "DESERT",   { 179, 215, 222, 229 }, 222, 215, 215, 179, 231 },
    { "ARCTIC",   { 117, 159, 195, 231 }, 231, 195, 195, 117, 231 },
};

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

/* theme_apply_pairs_256color — push the 5 + N_TRAIL_BANDS colour-pair
 * assignments for a 256-colour terminal.  Each pair maps one render
 * role (rod_top, joint, trail-band-i, …) to one xterm-256 index from
 * the Theme row.  All foregrounds use the terminal's default
 * background (-1) so the demo composites cleanly on dark or light. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_ROD_TOP, t->rod_top, -1);
    init_pair(PAIR_ROD_BOT, t->rod_bot, -1);
    init_pair(PAIR_JOINT,   t->joint,   -1);
    init_pair(PAIR_BOB,     t->bob,     -1);
    init_pair(PAIR_PIVOT,   t->pivot,   -1);
}

/* theme_apply_pairs_8color_fallback — single fallback assignment for
 * 8-colour terminals where the per-theme xterm-256 palette can't be
 * expressed.  All N_THEMES collapse to the SAME 8-colour set — the
 * trade-off is "lose theme variety, keep legibility everywhere".
 * The chosen colours match the t/T-default theme's perceptual roles
 * (warm = rods, hot = bob, cool = trail, neutral = pivot). */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_ROD_TOP, COLOR_YELLOW,  -1);
    init_pair(PAIR_ROD_BOT, COLOR_MAGENTA, -1);
    init_pair(PAIR_JOINT,   COLOR_YELLOW,  -1);
    init_pair(PAIR_BOB,     COLOR_RED,     -1);
    init_pair(PAIR_PIVOT,   COLOR_WHITE,   -1);
}

/* theme_apply — top-level dispatcher: clamp the index, branch on
 * terminal capability, delegate to the appropriate helper. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  coord — pixel ↔ cell                                               */
/* ===================================================================== */

static inline int px_to_cell_x(float px) { return (int)(px / (float)CELL_W); }
static inline int px_to_cell_y(float py) { return (int)(py / (float)CELL_H); }

/* ===================================================================== */
/* §5  physics — double pendulum + RK4                                    */
/* ===================================================================== */

/*
 * PendulumSystem — the INVARIANT physical parameters of the chain.
 *
 * INTENT
 *   Separates "what the system IS" (lengths, masses, gravity) from
 *   "what its current configuration is" (PendulumState) and "where
 *   it lives in screen space" (PendulumAnchor).  These three concepts
 *   are independent — the Lagrangian derivation cares only about the
 *   system; the renderer cares about the anchor; the integrator cares
 *   about the state.
 *
 * CONTEXT
 *   Set once at scene_compute_geometry, which scales L and g together
 *   so the small-angle period T = 2π√(L/g) stays constant at ~2 s.
 *   Read by every pendulum_deriv() call.  Const-during-simulation.
 *
 * MEMBER LOGIC
 *   L1, L2 : arm lengths in pixels.  Equal in this demo for visual
 *            symmetry; the Lagrangian supports L1 ≠ L2 if you want.
 *   m1, m2 : bob masses (dimensionless 1 here).
 *   g      : gravitational acceleration in px/s².  Scales with L to
 *            preserve period across terminal sizes.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Shinbrot et al. (1992)  — the closed-form θ̈ expressions
 *                                 implemented in pendulum_deriv().
 *   [4] Goldstein Ch. 1-2       — the Euler-Lagrange derivation that
 *                                 produces those expressions from
 *                                 first principles (kinetic + potential
 *                                 energy of the two-bob system).
 *   [5] Landau & Lifshitz §5-9  — small-amplitude period T = 2π√(L/g),
 *                                 used by scene_compute_geometry to
 *                                 keep T ≈ 2 s constant across resizes.
 */
typedef struct {
    float L1, L2, m1, m2, g;
} PendulumSystem;

/*
 * PendulumAnchor — pivot position in pixel space.
 *
 * INTENT
 *   The forward-kinematics function pendulum_bob_positions() needs
 *   to know WHERE on screen the chain hangs from.  Splitting the
 *   anchor from PendulumSystem lets scene_compute_geometry move the
 *   pivot on resize without touching physics parameters, and keeps
 *   the Lagrangian derivation free of any screen-space concerns.
 *   The pendulum dynamics are TRANSLATION-INVARIANT — moving the
 *   pivot does not change θ̈ — so the split is also physically clean.
 *
 * CONTEXT
 *   One instance lives inside DoublePendulum (embedded, not pointer).
 *   Written by scene_compute_geometry() at init / resize.  Read by
 *   pendulum_bob_positions() every frame to convert (θ₁, θ₂) → bob
 *   pixel positions, and by scene_paint() to draw the '[+]' pivot
 *   marker.  Const during a single tick of physics.
 *
 * MEMBER LOGIC
 *   px : pivot x in PIXELS (= cell_x × CELL_W + offset).  Placed at
 *        0.5 × map_w × CELL_W — the horizontal centre of the drawable.
 *   py : pivot y in PIXELS.  Placed at the VERTICAL centre of the
 *        drawable (HUD_TOP_ROWS + draw_h/2), not the top — so the
 *        CHAOS preset's lower bob has room to swing over the pivot.
 *
 * REFERENCES
 *   • Header ref [4] Goldstein §1.4 — translation invariance of the
 *     Euler-Lagrange equations, the formal justification for keeping
 *     "where the system is" separate from "what the system is".
 */
typedef struct {
    float px, py;
} PendulumAnchor;

/*
 * PendulumState — the 4-D ORBIT of the chain.
 *
 * INTENT
 *   The dynamical state vector y for the ODE  dy/dt = f(y).  "Where
 *   we are in iterating the system" — the configuration that evolves
 *   under pendulum_rk4().  Kept as its own type so it can be passed by
 *   value into pendulum_deriv() and combined with state_add() (k1, k2,
 *   k3, k4 are all PendulumState) without dragging system + anchor
 *   through the math.  The 4-D phase space (θ₁, θ₂, ω₁, ω₂) is the
 *   exact set of coordinates Shinbrot uses to draw the Poincaré
 *   sections that classify regular vs chaotic behaviour.
 *
 * CONTEXT
 *   One instance lives inside DoublePendulum.state.  Written by
 *   pendulum_rk4() once per substep (INT_STEPS_PER_TICK times per
 *   sim tick).  Reset by pendulum_state_init_from_preset() on
 *   preset change / r-press / resize.  Read by pendulum_deriv() at
 *   each RK4 stage, by pendulum_bob_positions() to compute the
 *   bob pixel locations, and by §9 scene_paint().
 *
 * MEMBER LOGIC
 *   th1, th2 : angles (rad) of upper / lower rod from downward
 *              vertical (= Shinbrot's "absolute angle" convention,
 *              not the "joint-relative" alternative).  Stored
 *              UNWRAPPED — values can grow past ±π as the bob
 *              rotates over the top (the chaos demonstration).  Not
 *              normalised because sin/cos handle any input naturally
 *              and a wrapping discontinuity would corrupt RK4's
 *              intermediate k-values.
 *   w1,  w2  : angular velocities dθ/dt (rad/s).  Sign convention
 *              follows the angle convention — positive ω₁ rotates the
 *              upper rod clockwise (viewed with screen-up = real up).
 *              No bound — in rotational presets ω can hit ±15 rad/s.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Shinbrot §III — defines the (θ₁, θ₂, ω₁, ω₂) phase-space
 *       coordinates used by this struct.
 *   [6] Numerical Recipes Ch. 17.1 — explains why an ODE state
 *       vector is best implemented as a flat by-value type for RK4:
 *       k_i need cheap structural copies, not pointer aliasing.
 */
typedef struct {
    float th1, th2;
    float w1,  w2;
} PendulumState;

/*
 * DoublePendulum — composite: SYSTEM + ANCHOR + STATE.
 *
 * INTENT
 *   Bundle the three independent concerns under one named type so
 *   functions that need "the whole pendulum" take one pointer
 *   instead of three.  RK4 mutates only .state; resize mutates only
 *   .system + .anchor; render reads all three.  The three-way split
 *   mirrors the three-way separation in the textbook treatment:
 *   PARAMETERS / COORDINATES / EMBEDDING.
 *
 * CONTEXT
 *   One instance lives on Scene.  Sub-structs are touched separately:
 *     .system : written once by scene_compute_geometry (init / resize).
 *     .anchor : written once by scene_compute_geometry (init / resize).
 *     .state  : written ~480 times/sec by pendulum_rk4 substeps;
 *               reset by pendulum_state_init_from_preset on r/n/p.
 *
 * MEMBER LOGIC
 *   system : §5 PendulumSystem — lengths, masses, gravity.
 *   anchor : §5 PendulumAnchor — pivot pixel position.
 *   state  : §5 PendulumState  — current (θ₁, θ₂, ω₁, ω₂).
 *
 * REFERENCES (numbered in file-header References block)
 *   [4] Goldstein §2.1-2.2 — the textbook PARAMETERS / GENERALISED
 *       COORDINATES / CONSTRAINTS triad that this struct's three-way
 *       decomposition reproduces in code form.
 */
typedef struct {
    PendulumSystem system;
    PendulumAnchor anchor;
    PendulumState  state;
} DoublePendulum;

/* RK4_BUTCHER_WEIGHT_SUM — denominator of the RK4 weighted average:
 *   (k1 + 2k2 + 2k3 + k4) / 6.  Lives at the top of §5 because both
 *   the per-component averaging helper and the doc for pendulum_rk4
 *   reference it. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f

/* RK4_MIDPOINT_FRACTION — the ½ in y + ½·dt·k₁ at stages 2 and 3 of
 * the classical RK4 tableau (Press et al. 17.1 eq. 17.1.3). */
#define RK4_MIDPOINT_FRACTION    0.5f

/* lagrangian_mass_coupling_denominator — the (2m₁ + m₂ - m₂·cos(2d))
 * factor common to BOTH angular accelerations θ̈₁ and θ̈₂.  Geometrically:
 * captures how strongly the two bobs' inertias couple at the current
 * relative angle d = θ₁ - θ₂.  Always positive for non-degenerate
 * masses (m₁, m₂ > 0), so it's safe in the denominator without checks.
 *
 *   d = 0           → den = 2m₁                       (chain straight)
 *   d = π/2         → den = 2m₁ + m₂                  (right-angle)
 *   d = π           → den = 2m₁ + 2m₂                 (folded back)
 *
 * See Shinbrot eq. (3): the common denominator that emerges when the
 * Euler-Lagrange equations are solved jointly for θ̈₁ and θ̈₂. */
static inline float lagrangian_mass_coupling_denominator(const PendulumSystem *sys,
                                                         float angle_diff)
{
    return 2.0f * sys->m1 + sys->m2 - sys->m2 * cosf(2.0f * angle_diff);
}

/* lagrangian_upper_angular_acceleration — θ̈₁ from the Euler-Lagrange
 * equation; Shinbrot eq. (1).  The numerator combines three physical
 * torques on the upper rod:
 *
 *   (a) GRAVITY RESTORING TORQUE on the upper bob plus its share of
 *       the lower bob's weight     →  -g · (2m₁+m₂) · sin θ₁
 *   (b) BACK-REACTION from the lower bob's gravity, mediated by the
 *       coupling angle             →  -m₂ · g · sin(θ₁ - 2θ₂)
 *   (c) CENTRIPETAL COUPLING from both bobs spinning, scaled by
 *       sin(angle_diff)            →  -2·sin(d) · m₂ · (centrip_lower
 *                                       + centrip_upper · cos(d))
 *
 * Numerator / (L₁ · denom) gives the angular acceleration. */
static inline float lagrangian_upper_angular_acceleration(
    const PendulumState *s, const PendulumSystem *sys,
    float sin_diff, float cos_diff, float denom)
{
    float upper_inertia_factor  = 2.0f * sys->m1 + sys->m2;   /* (a) coef   */
    float centripetal_upper     = s->w1 * s->w1 * sys->L1;    /* ω₁² · L₁   */
    float centripetal_lower     = s->w2 * s->w2 * sys->L2;    /* ω₂² · L₂   */

    float gravity_restoring_term =
        -sys->g * upper_inertia_factor * sinf(s->th1);                 /* (a) */
    float gravity_back_reaction_term =
        -sys->m2 * sys->g * sinf(s->th1 - 2.0f * s->th2);              /* (b) */
    float centripetal_coupling_term =
        -2.0f * sin_diff * sys->m2 *
            (centripetal_lower + centripetal_upper * cos_diff);        /* (c) */

    float numerator = gravity_restoring_term
                    + gravity_back_reaction_term
                    + centripetal_coupling_term;
    return numerator / (sys->L1 * denom);
}

/* lagrangian_lower_angular_acceleration — θ̈₂ from the Euler-Lagrange
 * equation; Shinbrot eq. (2).  Numerator = 2·sin(d) × the three
 * driving contributions on the lower rod:
 *
 *   (a) CENTRIPETAL FEED-FORWARD from the upper bob's spin
 *                                  →  ω₁² · L₁ · (m₁+m₂)
 *   (b) GRAVITY TORQUE on the lower bob, mediated by the upper-rod
 *       projection cos θ₁          →  g · (m₁+m₂) · cos θ₁
 *   (c) CENTRIPETAL FEEDBACK from the lower bob's own spin, with the
 *       coupling cos(d) factor     →  ω₂² · L₂ · m₂ · cos d
 *
 * Numerator / (L₂ · denom) gives θ̈₂. */
static inline float lagrangian_lower_angular_acceleration(
    const PendulumState *s, const PendulumSystem *sys,
    float sin_diff, float cos_diff, float denom)
{
    float total_mass        = sys->m1 + sys->m2;
    float centripetal_upper = s->w1 * s->w1 * sys->L1;
    float centripetal_lower = s->w2 * s->w2 * sys->L2;

    float centripetal_feedforward_term =
        centripetal_upper * total_mass;                                /* (a) */
    float gravity_projection_term =
        sys->g * total_mass * cosf(s->th1);                            /* (b) */
    float centripetal_feedback_term =
        centripetal_lower * sys->m2 * cos_diff;                        /* (c) */

    float numerator = 2.0f * sin_diff *
                      (centripetal_feedforward_term
                     + gravity_projection_term
                     + centripetal_feedback_term);
    return numerator / (sys->L2 * denom);
}

/* pendulum_deriv — evaluate the right-hand side f(y) of dy/dt = f(y)
 * for the 4-D state y = (θ₁, θ₂, ω₁, ω₂).  This is the heart of the
 * simulation; everything else is bookkeeping around this function.
 *
 *   STEP 1 — compute the SHARED geometric scalars derived from the
 *            relative angle d = θ₁ - θ₂  (sin d, cos d, mass-coupling
 *            denominator).  Computed once, reused in both accelerations.
 *   STEP 2 — fill the KINEMATIC half of f(y):  dθᵢ/dt = ωᵢ  (identity).
 *   STEP 3 — fill the DYNAMIC half of f(y):    dωᵢ/dt = θ̈ᵢ  via the
 *            Lagrangian helpers.
 *
 * See header refs [4] Goldstein Ch. 1-2 for the Euler-Lagrange
 * derivation and [1] Shinbrot eq. (1)-(2) for the closed-form. */
static PendulumState pendulum_deriv(const PendulumState *s,
                                    const PendulumSystem *sys)
{
    /* STEP 1 — geometric scalars at the current relative angle */
    float angle_diff = s->th1 - s->th2;
    float sin_diff   = sinf(angle_diff);
    float cos_diff   = cosf(angle_diff);
    float denom      = lagrangian_mass_coupling_denominator(sys, angle_diff);

    /* STEP 2 — kinematic half: dθ/dt is just ω */
    PendulumState out;
    out.th1 = s->w1;
    out.th2 = s->w2;

    /* STEP 3 — dynamic half: dω/dt is the Lagrangian acceleration */
    out.w1 = lagrangian_upper_angular_acceleration(s, sys, sin_diff, cos_diff, denom);
    out.w2 = lagrangian_lower_angular_acceleration(s, sys, sin_diff, cos_diff, denom);
    return out;
}

/* state_add — y + h·k, returned as a new PendulumState. */
static inline PendulumState state_add(const PendulumState *a,
                                      float h, const PendulumState *k)
{
    PendulumState r;
    r.th1 = a->th1 + h * k->th1;
    r.th2 = a->th2 + h * k->th2;
    r.w1  = a->w1  + h * k->w1;
    r.w2  = a->w2  + h * k->w2;
    return r;
}

/* rk4_butcher_weighted_average — Simpson-rule combination of the 4
 * slope estimates:  (k₁ + 2k₂ + 2k₃ + k₄) / 6.  The (1, 2, 2, 1) / 6
 * weights are the classical 4th-order Butcher tableau coefficients
 * (Press et al. 17.1 eq. 17.1.3).  Operates component-wise on the
 * 4-D PendulumState — k_i are themselves derivative-shaped states. */
static inline PendulumState rk4_butcher_weighted_average(
    const PendulumState *k1, const PendulumState *k2,
    const PendulumState *k3, const PendulumState *k4)
{
    PendulumState avg;
    avg.th1 = (k1->th1 + 2.0f * k2->th1 + 2.0f * k3->th1 + k4->th1)
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.th2 = (k1->th2 + 2.0f * k2->th2 + 2.0f * k3->th2 + k4->th2)
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.w1  = (k1->w1  + 2.0f * k2->w1  + 2.0f * k3->w1  + k4->w1 )
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.w2  = (k1->w2  + 2.0f * k2->w2  + 2.0f * k3->w2  + k4->w2 )
              / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* pendulum_rk4 — single RK4 step of size dt.  Standard 4-stage scheme:
 *
 *   STAGE 1 — slope_start    = f(y)                  ← @ t
 *   STAGE 2 — slope_midpoint = f(y + ½dt · slope_start)        ← @ t+½dt
 *   STAGE 3 — slope_midpoint'= f(y + ½dt · slope_midpoint)     ← @ t+½dt
 *   STAGE 4 — slope_end      = f(y +  dt · slope_midpoint')    ← @ t+dt
 *   UPDATE  — y ← y + dt · (slope_start + 2·slope_mid + 2·slope_mid' + slope_end) / 6
 *
 * See header ref [6] Numerical Recipes Ch. 17.1 for the Butcher
 * tableau, local truncation error O(dt⁵), and stability properties.
 * RK4 is the default choice here because the Lyapunov time T_λ ~ 1 s
 * is long compared to the substep (dt/8 ≈ 2 ms) — a 4th-order method
 * keeps energy drift below visual noise over the trail's 480-frame
 * lifetime without the cost of an adaptive scheme. */
static void pendulum_rk4(DoublePendulum *p, float dt)
{
    const PendulumSystem *sys = &p->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    /* STAGE 1 — slope at the start of the interval */
    PendulumState slope_start    = pendulum_deriv(&p->state, sys);

    /* STAGE 2 — slope at the midpoint, using stage-1 slope */
    PendulumState midpoint_state_1 = state_add(&p->state, half_dt, &slope_start);
    PendulumState slope_midpoint_1 = pendulum_deriv(&midpoint_state_1, sys);

    /* STAGE 3 — slope at the midpoint, using stage-2 slope */
    PendulumState midpoint_state_2 = state_add(&p->state, half_dt, &slope_midpoint_1);
    PendulumState slope_midpoint_2 = pendulum_deriv(&midpoint_state_2, sys);

    /* STAGE 4 — slope at the end of the interval */
    PendulumState endpoint_state   = state_add(&p->state, dt, &slope_midpoint_2);
    PendulumState slope_end        = pendulum_deriv(&endpoint_state, sys);

    /* UPDATE — Butcher-weighted average gives the effective slope */
    PendulumState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_midpoint_1, &slope_midpoint_2, &slope_end);
    p->state = state_add(&p->state, dt, &effective_slope);
}

/* pendulum_bob_positions — forward-kinematics from (anchor, θ₁, θ₂)
 * to the two bob pixel positions.  Vertical axis points DOWN (screen
 * convention), so cos θ flips sign relative to the textbook y-up
 * derivation — but the dynamics are identical. */
static void pendulum_bob_positions(const DoublePendulum *p,
                                   float *b1x, float *b1y,
                                   float *b2x, float *b2y)
{
    *b1x = p->anchor.px + p->system.L1 * sinf(p->state.th1);
    *b1y = p->anchor.py + p->system.L1 * cosf(p->state.th1);
    *b2x = *b1x         + p->system.L2 * sinf(p->state.th2);
    *b2y = *b1y         + p->system.L2 * cosf(p->state.th2);
}

/* pendulum_state_init_from_preset — apply a PendulumPreset's named
 * initial conditions (degrees + rad/s) to a PendulumState (radians).
 * Pure helper — no system, no anchor: just (θ, ω) initialisation. */
static void pendulum_state_init_from_preset(PendulumState *s,
                                            const PendulumPreset *preset)
{
    s->th1 = preset->theta1_deg * (float)M_PI / 180.0f;
    s->th2 = preset->theta2_deg * (float)M_PI / 180.0f;
    s->w1  = preset->omega1;
    s->w2  = preset->omega2;
}

/* ===================================================================== */
/* §6  trail — bob-2 path ring buffer                                     */
/* ===================================================================== */

/*
 * Trail — fixed-capacity ring buffer of recent lower-bob pixel positions.
 *
 * INTENT
 *   The signature of double-pendulum chaos is the SCRIBBLE that the
 *   bottom bob traces out over time — a path so dense and tangled that
 *   no closed-form description exists.  This struct stores the last
 *   TRAIL_MAX samples of that path so §9 can re-draw the scribble each
 *   frame, fading older points to communicate "time direction" without
 *   any extra motion cue.  Watching the trail accumulate IS the
 *   pedagogical demonstration — the static frame would show a single
 *   pendulum pose, indistinguishable from a regular pendulum at rest.
 *
 * CONTEXT
 *   One instance lives on Scene.  Fed by scene_tick() once per physics
 *   tick via trail_push() (called immediately after pendulum_rk4()
 *   advances the state, with the freshly-computed bob-2 position).
 *   Drawn by §9 trail_paint() which walks oldest→newest and maps each
 *   sample's age to one of TRAIL_BAND_COUNT colour bands.  Cleared by
 *   trail_reset() on preset change / r-press / resize.  Sized at
 *   TRAIL_MAX = 4000 ≈ 67 s of history at 60 Hz — long enough to
 *   demonstrate the chaotic drift, short enough that the ring buffer
 *   fits comfortably in cache (~32 KB).
 *
 * DATA-STRUCTURE LOGIC
 *   Ring buffer chosen over a growing list for two reasons:
 *     1. ZERO ALLOCATION in the hot path — per CLAUDE.md "Memory
 *        Allocation" the simulation tick must not call malloc/free.
 *     2. CONSTANT cost per frame regardless of how long the program
 *        has been running — push and full-buffer iteration are both
 *        O(TRAIL_MAX), no resize step ever occurs.
 *   Newest sample is at index `head`; oldest is at
 *   `(head - count + 1) mod TRAIL_MAX` while filling, and at
 *   `(head + 1) mod TRAIL_MAX` once full.  trail_push() advances head
 *   FIRST then writes, so the head index is always the just-written
 *   slot (matches the §9 iteration convention).
 *
 * MEMBER LOGIC
 *   px[i] : x pixel coordinate of the i-th stored bob-2 position.
 *           Stored in PIXEL space (not cell space) so the trail
 *           inherits sub-cell precision from the physics integrator —
 *           the conversion to cell coords happens only at paint time
 *           via §4 px_to_cell_x().
 *   py[i] : y pixel coordinate (downward-positive, screen convention).
 *   head  : index of the MOST RECENT (newest) sample.  Wraps modulo
 *           TRAIL_MAX.  When count < TRAIL_MAX, valid samples occupy
 *           indices (head - count + 1 .. head) wrapping around.
 *   count : number of valid samples currently stored.  Saturates at
 *           TRAIL_MAX once the buffer fills; from then on the oldest
 *           sample is overwritten on every push.  Lets trail_paint()
 *           skip empty slots before the first TRAIL_MAX ticks.
 *
 * REFERENCES
 *   • Sedgewick, R. — "Algorithms" 4th ed., §1.3.  Canonical
 *     reference for ring-buffer / circular-queue implementation.
 *   • Header ref [1] Shinbrot — Fig. 3-4 photographs of the
 *     bob-2 trajectory that this trail is reproducing.
 */
typedef struct {
    float px[TRAIL_MAX];
    float py[TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }

static void trail_push(Trail *t, float px, float py)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->px[t->head] = px;
    t->py[t->head] = py;
    if (t->count < TRAIL_MAX) t->count++;
}

/* trail_oldest_index — index of the OLDEST still-valid sample in the
 * ring.  Both before and after the buffer wraps, the formula
 * (head - count + 1) mod TRAIL_MAX correctly points at the start of
 * the live window.  The "+ TRAIL_MAX" before the mod handles the C
 * "% on negative numerator is implementation-defined" trap. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* trail_band_for_age — map a sample's "age in the buffer" to one of
 * TRAIL_BAND_COUNT colour/glyph tiers.  Linear partition:
 *
 *     band = (N-1) - ⌊age · N / count⌋
 *
 * AGE = 0 (the newest sample) → band = N-1 (brightest tier).
 * AGE = count-1 (oldest)      → band = 0   (dimmest tier).
 *
 * Clamped to [0, N-1] for safety against integer-division edge cases. */
static inline int trail_band_for_age(int age, int count)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age * TRAIL_BAND_COUNT) / count;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* ===================================================================== */
/* §7  state — typed wrappers around the two cycled selections          */
/* ===================================================================== */

/*
 * PresetState — typed wrapper around "which initial condition is loaded".
 *
 * INTENT
 *   The Preset enum is just an integer — it tells you nothing about the
 *   semantics of "next preset", "previous preset", or "look up the
 *   active row in presets[]".  Wrapping it in a struct + helpers makes
 *   the binding-table in app_handle_key read as
 *
 *       case 'n': preset_state_cycle_next(&s->preset); scene_reset(s);
 *
 *   instead of doing modular arithmetic inline at the call site.  This
 *   is the "smart constructor / value-object" pattern — a one-field
 *   struct exists not to hold data but to ATTACH a named API to that
 *   data so the call sites read as intentions, not arithmetic.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_preset_next/prev
 *   when the user presses n/p; read by scene_load_active_preset() (via
 *   preset_state_active() → presets[current] → pendulum_state_init…).
 *   Also read by the HUD writers for the row-1 "preset:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table in §1.  Must be in
 *             [0, N_PRESETS) — the cycle helpers guarantee this by
 *             modular arithmetic.  Initial value is set by scene_init
 *             to PRESET_CHAOS so the demo opens on the dramatic case.
 *
 * REFERENCES
 *   • Fowler, M. — "Refactoring" (2nd ed.), ch. "Replace Primitive
 *     with Object".  The pattern this struct implements — wrapping a
 *     bare int in a named type so the operations on it can hang off
 *     the type rather than being scattered at the call sites.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial)        { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)               { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)               { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const PendulumPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState — both are "an index that cycles through
 *   a const table on key-press" — but kept as a SEPARATE type because
 *   their tables differ (presets[] vs themes[]) and conflating them
 *   would just be punning.  Distinct types let scene_apply_theme() take
 *   exactly the thing it needs and refuse anything else, and avoid the
 *   class of bug where a "next" key on the wrong wrapper would silently
 *   index past N_PRESETS into N_THEMES or vice versa.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_theme_next/prev
 *   when the user presses t/T.  After every mutation, the caller invokes
 *   palette_state_apply() which forwards to §3 theme_apply(current) —
 *   the only thing that touches ncurses init_pair() calls.  Read by the
 *   HUD writer for the row-1 "theme:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table in §1.  Must be in
 *             [0, N_THEMES).  Initial value is 0 (set by scene_init);
 *             theme 0 is conventionally the most legible default.
 *
 * REFERENCES
 *   • Fowler "Refactoring" — same "Replace Primitive with Object"
 *     pattern as PresetState.  The deliberate type-distinction
 *     (PresetState vs PaletteState despite identical shape) is the
 *     "Tiny Types" / strong-typing idea from Domain-Driven Design:
 *     values from different domains shouldn't share a type even when
 *     the underlying representation is the same.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial)      { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)             { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)             { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)   { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)            { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundles every piece of mutable state into one struct so the App
 *   layer owns one Scene (not a dozen loose globals) and the main
 *   loop drives it through a tiny named-method API.  Each sub-field
 *   is a NAMED CONCEPT — pendulum / trail / preset / palette — so
 *   adding "and now a strobe overlay" later is a new field, not a
 *   refactor.
 *
 *   The decomposition follows the "what the simulation IS made of"
 *   axis: a physical body (pendulum), a record of where it has been
 *   (trail), a selection of which initial condition is active
 *   (preset), and a selection of which colour palette is showing
 *   (palette).  PRESENTATION live-flags (paused) and grid dims
 *   (map_w/h) sit at the top level — they belong to the Scene as a
 *   whole, not to any one sub-system.
 *
 * CONTEXT
 *   One instance is embedded inside App.  Mutated by:
 *     • scene_tick     (~60 Hz, advances .pendulum.state + .trail)
 *     • scene_reset    (r/n/p   — reloads preset, clears trail)
 *     • scene_resize   (SIGWINCH — re-derives geometry, also resets)
 *     • app_handle_key (sub-states .preset, .palette, .paused)
 *   Read by:
 *     • scene_paint    (every frame — draws all sub-systems)
 *     • HUD writers    (preset / palette / paused for the status line)
 *
 * MEMBER LOGIC
 *   pendulum : §5 DoublePendulum — SYSTEM + ANCHOR + 4-D ODE state.
 *              Mutated by scene_tick(); read by scene_paint().
 *   trail    : §6 Trail — ring buffer of recent lower-bob pixel
 *              positions, the characteristic "scribble" of the
 *              chaotic regime.  Drives the visual story of the demo.
 *   preset   : §7 PresetState — wrapper around "which row of presets[]
 *              is loaded".  n/p cycle it; scene_reset() applies the
 *              active row to the pendulum state.
 *   palette  : §7 PaletteState — wrapper around "which row of themes[]
 *              is active".  t/T cycle it; scene_apply_theme() pushes
 *              the change into ncurses.
 *   paused   : when true, scene_tick early-returns — physics frozen
 *              but rendering continues so the user can study the pose.
 *   map_w/h  : grid dimensions in CELLS available to the simulation
 *              (terminal size minus HUD_BAND_RESERVED_ROWS).  Drives
 *              scene_compute_geometry which sizes the pendulum to fit
 *              the drawable area at any terminal resolution.
 *
 * REFERENCES
 *   • Gamma, Helm, Johnson, Vlissides (1995) — "Design Patterns",
 *     "Composite" pattern.  A Scene is the COMPOSITE root: a single
 *     object that holds a tree of sub-objects, with one tick/paint
 *     call fanning out to each leaf in deterministic order.
 *   • Sussman, G. J., Wisdom, J. (2014) — "Structure and Interpretation
 *     of Classical Mechanics".  The "system / state / observable" tri-
 *     partition this Scene mirrors at the simulation-architecture level.
 */
typedef struct {
    DoublePendulum pendulum;
    Trail          trail;
    PresetState    preset;
    PaletteState   palette;
    bool           paused;
    int            map_w, map_h;
} Scene;

/* GEOMETRY_DRAWABLE_HEIGHT_FLOOR — minimum cells of vertical room the
 * pendulum needs.  Below this the chain collapses to a few cells and
 * loses all visual character; we floor to keep the demo usable on
 * very small terminals (and never divide by something near zero). */
#define GEOMETRY_DRAWABLE_HEIGHT_FLOOR  8

/* GEOMETRY_ARM_CELLS_PER_HALF_FLOOR — after the half-drawable cap is
 * applied, this is the minimum arm length the chain can collapse to
 * before the chain becomes unreadable.  Two arms of 3 cells = swing
 * radius 6 cells, the smallest that still reads as a pendulum. */
#define GEOMETRY_ARM_CELLS_PER_HALF_FLOOR  3

/* GEOMETRY_PIVOT_HORIZONTAL_FRACTION / VERTICAL_FRACTION_OF_DRAWABLE —
 * named handles for the "centre the pivot" coefficients.  Horizontal:
 * dead centre of the terminal.  Vertical: centre of the drawable
 * band (HUD-reserved rows already subtracted upstream). */
#define GEOMETRY_PIVOT_HORIZONTAL_FRACTION         0.5f
#define GEOMETRY_PIVOT_VERTICAL_CELL_HALF_OFFSET   0.5f   /* half-cell so the
                                                            anchor sits on
                                                            the row's middle */

/* geometry_drawable_height — strip HUD reservation, floor to a
 * sensible minimum.  The "drawable height" is the vertical band that
 * scene_paint may write to without overwriting the HUD. */
static int geometry_drawable_height(int map_h)
{
    int draw_h = map_h - HUD_BAND_RESERVED_ROWS;
    if (draw_h < GEOMETRY_DRAWABLE_HEIGHT_FLOOR)
        draw_h = GEOMETRY_DRAWABLE_HEIGHT_FLOOR;
    return draw_h;
}

/* geometry_pick_arm_cells — choose an arm length in cells from the
 * drawable height.  Clamped TWICE:
 *
 *   PASS 1 — global [ARM_CELLS_MIN, ARM_CELLS_MAX] envelope for very
 *            small / very large terminals.
 *   PASS 2 — "two arms fit within the upper half-drawable" cap, so
 *            the lower bob has room to swing OVER the centred pivot
 *            in CHAOS-class presets.  Floor at ARM_CELLS_PER_HALF_FLOOR
 *            so the chain can't collapse to invisibility.
 */
static int geometry_pick_arm_cells(int draw_h)
{
    int arm_cells = (int)((float)draw_h * ARM_CELLS_OF_DRAWABLE);

    /* PASS 1 — global envelope */
    if (arm_cells < ARM_CELLS_MIN) arm_cells = ARM_CELLS_MIN;
    if (arm_cells > ARM_CELLS_MAX) arm_cells = ARM_CELLS_MAX;

    /* PASS 2 — fit two arms in the half above the pivot */
    int half_drawable = draw_h / 2;
    int max_arm_for_swing_over = (half_drawable - 1) / 2;
    if (arm_cells > max_arm_for_swing_over)
        arm_cells = max_arm_for_swing_over;
    if (arm_cells < GEOMETRY_ARM_CELLS_PER_HALF_FLOOR)
        arm_cells = GEOMETRY_ARM_CELLS_PER_HALF_FLOOR;
    return arm_cells;
}

/* geometry_set_physics_scale — set L₁, L₂ from arm length and rescale
 * gravity so the small-angle period T = 2π√(L/g) stays at ~2 s
 * regardless of terminal size.  This is the "visual cadence" knob —
 * keeps the same physical feel across resolutions. */
static void geometry_set_physics_scale(PendulumSystem *sys, int arm_cells)
{
    float arm_length_px = (float)arm_cells * (float)CELL_H;
    sys->L1 = arm_length_px;
    sys->L2 = arm_length_px;
    sys->g  = PEND_GRAVITY_PER_LENGTH * arm_length_px;
}

/* geometry_set_pivot_center — place the anchor at the horizontal ×
 * VERTICAL centre of the drawable band.  Vertical-centre (not top!)
 * is the load-bearing detail: the lower bob in CHAOS-class presets
 * loops OVER the pivot, so we need clearance above as well as below. */
static void geometry_set_pivot_center(PendulumAnchor *anc, int map_w, int draw_h)
{
    int draw_mid_row = HUD_TOP_ROWS + draw_h / 2;
    anc->px = GEOMETRY_PIVOT_HORIZONTAL_FRACTION
            * (float)map_w * (float)CELL_W;
    anc->py = ((float)draw_mid_row + GEOMETRY_PIVOT_VERTICAL_CELL_HALF_OFFSET)
            * (float)CELL_H;
}

/* scene_compute_geometry — drive the 4-step size-adaptive pipeline:
 *
 *   STEP 1 — figure out vertical room available (after HUD reservation)
 *   STEP 2 — pick an arm length that fills that room without overflowing
 *   STEP 3 — set L, g together so the small-angle period stays constant
 *   STEP 4 — centre the pivot in the drawable band
 *
 * Reads s->map_w, s->map_h.  Writes s->pendulum.system + .anchor. */
static void scene_compute_geometry(Scene *s)
{
    int draw_h    = geometry_drawable_height(s->map_h);
    int arm_cells = geometry_pick_arm_cells(draw_h);
    geometry_set_physics_scale(&s->pendulum.system, arm_cells);
    geometry_set_pivot_center (&s->pendulum.anchor, s->map_w, draw_h);
}

/* scene_load_active_preset — apply the active preset row from §7's
 * PresetState onto the pendulum's dynamic state.  System / anchor
 * are untouched — preset cycling does NOT resize the pendulum. */
static void scene_load_active_preset(Scene *s)
{
    pendulum_state_init_from_preset(&s->pendulum.state,
                                    preset_state_active(&s->preset));
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s)
{
    scene_load_active_preset(s);
    trail_reset(&s->trail);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->map_w  = mw;
    s->map_h  = mh;
    s->paused = false;

    /* Sub-state initialisation — each wrapper owns its own default. */
    preset_state_init(&s->preset,  PRESET_CHAOS);  /* headline first    */
    palette_state_init(&s->palette, 0);            /* first theme       */

    /* Pendulum: masses are scale-invariant, geometry depends on grid. */
    s->pendulum.system.m1 = PEND_M1;
    s->pendulum.system.m2 = PEND_M2;
    scene_compute_geometry(s);   /* sets L1, L2, g, anchor (size-adaptive) */
    scene_reset(s);              /* applies preset, clears trail          */
}

static void scene_resize(Scene *s, int mw, int mh)
{
    s->map_w = mw;
    s->map_h = mh;
    scene_compute_geometry(s);   /* re-derives L, g, anchor for new size  */
    scene_reset(s);
}

/* scene_tick — advance one rendered frame of physics + trail.
 * INT_STEPS_PER_TICK RK4 substeps per call keeps the integration
 * stable when the bob whips around. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float sub_dt = dt / (float)INT_STEPS_PER_TICK;
    for (int i = 0; i < INT_STEPS_PER_TICK; i++)
        pendulum_rk4(&s->pendulum, sub_dt);

    float b1x, b1y, b2x, b2y;
    pendulum_bob_positions(&s->pendulum, &b1x, &b1y, &b2x, &b2y);
    trail_push(&s->trail, b2x, b2y);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal-adapter handle: ncurses lifecycle + current
 * window dimensions.
 *
 * INTENT
 *   ncurses owns the actual terminal mode and back-buffer; this struct
 *   is just the THIN OWNERSHIP HANDLE that the program passes around.
 *   Bundling (cols, rows) here lets every painter take ONE Screen
 *   parameter instead of calling getmaxyx() at every draw site.
 *   Funnelling all init/teardown through screen_init/screen_free keeps
 *   the ncurses dependency at a single boundary — nothing else in the
 *   file calls initscr / endwin / getmaxyx directly.
 *
 * CONTEXT
 *   One instance lives on App.  Initialised by screen_init() during
 *   app_bootstrap().  Refreshed by screen_resize() inside
 *   app_handle_pending_resize() whenever SIGWINCH fires.  Read by
 *   every painter that needs to clip against the terminal bounds
 *   (trail_paint, draw_rod, the HUD writers).  Freed at program exit
 *   via screen_free(); endwin() also runs from atexit(cleanup) as a
 *   safety net if the program crashes before main() returns.
 *
 * MEMBER LOGIC
 *   cols : current terminal width  in CHARACTER CELLS (not pixels —
 *          conversion to pixels uses CELL_W from §1).  Updated only by
 *          screen_init / screen_resize via getmaxyx(stdscr, rows, cols).
 *   rows : current terminal height in CHARACTER CELLS.  Same update
 *          rule.  Row 0 is the top of the terminal, row (rows-1) is
 *          the bottom; the HUD reserves HUD_TOP_ROWS rows at the top
 *          and HUD_BOTTOM_ROWS rows at the bottom (see §1).
 *
 * REFERENCES
 *   • Header ref [9] Gookin "Programmer's Guide to NCurses" Ch. 2 —
 *     the initscr / endwin / getmaxyx / wnoutrefresh / doupdate API
 *     surface used by the four screen_* helpers in §9.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *sc)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc)   { (void)sc; endwin(); }
static void screen_resize(Screen *sc) { endwin(); refresh();
                                        getmaxyx(stdscr, sc->rows, sc->cols); }
static void screen_present(void)      { wnoutrefresh(stdscr); doupdate(); }

/* in_drawable — clip predicate so painters never overwrite the HUD. */
static inline bool in_drawable(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols
        && cy >= HUD_TOP_ROWS && cy < rows - HUD_BOTTOM_ROWS;
}

/* bresenham_glyph_for_step — pick the ASCII glyph that conveys the
 * direction of the NEXT Bresenham step, so the rod reads as a coherent
 * line at any angle rather than a string of identical marks:
 *
 *     step_x only          → '-'    (horizontal segment)
 *     step_y only          → '|'    (vertical   segment)
 *     both (diagonal step) → '/' or '\\' by sign(sx) vs sign(sy):
 *                              sx ==  sy  → '\\'   ↘ or ↖
 *                              sx == -sy  → '/'    ↗ or ↙
 *
 * Direction-aware ASCII rendering is the small extension Bresenham
 * (1965) doesn't cover — see header ref [8] + Foley §3.2. */
static inline chtype bresenham_glyph_for_step(bool step_x, bool step_y,
                                              int sx, int sy)
{
    if (step_x && step_y) return (sx == sy) ? '\\' : '/';
    if (step_x)           return '-';
    return '|';
}

/* draw_rod — Bresenham line on the cell grid, with each plotted cell
 * carrying a direction-aware glyph instead of a uniform mark.  The
 * line is clipped against the HUD-aware drawable area at PLOT time
 * (the loop still walks past clipped cells so the Bresenham state
 * stays consistent).
 *
 *   STEP 1 — initialise (dx, dy, sx, sy, err) per Bresenham §1965
 *   STEP 2 — plot current cell using the glyph for the NEXT step's
 *            direction (the rod's local angle), then check endpoint
 *   STEP 3 — advance (err, x0, y0) by one Bresenham step
 */
static void draw_rod(int x0, int y0, int x1, int y1, int pair,
                     int cols, int rows)
{
    /* STEP 1 — Bresenham state */
    int delta_x        = abs(x1 - x0);
    int delta_y        = abs(y1 - y0);
    int step_dir_x     = (x0 < x1) ? 1 : -1;
    int step_dir_y     = (y0 < y1) ? 1 : -1;
    int error_accumul  = delta_x - delta_y;

    attron(COLOR_PAIR(pair) | A_BOLD);
    for (;;) {
        /* STEP 2 — plot with the glyph that matches the next step */
        if (in_drawable(x0, y0, cols, rows)) {
            int doubled_error = 2 * error_accumul;
            bool will_step_x  = (doubled_error > -delta_y);
            bool will_step_y  = (doubled_error <  delta_x);
            chtype glyph = bresenham_glyph_for_step(will_step_x, will_step_y,
                                                    step_dir_x,  step_dir_y);
            mvaddch(y0, x0, glyph);
        }
        if (x0 == x1 && y0 == y1) break;

        /* STEP 3 — Bresenham advance */
        int doubled_error = 2 * error_accumul;
        if (doubled_error > -delta_y) { error_accumul -= delta_y; x0 += step_dir_x; }
        if (doubled_error <  delta_x) { error_accumul += delta_x; y0 += step_dir_y; }
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_pivot_marker — '[+]' bracket at the chain's anchor.  Three
 * cells wide so the pivot reads as a fixed reference point rather
 * than competing with the rod glyphs on either side.  Falls back to
 * a single '+' if the pivot is too close to a screen edge. */
static void draw_pivot_marker(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
    if (cx > 0 && cx < cols - 1) {
        mvaddch(cy, cx - 1, '[');
        mvaddch(cy, cx,     '+');
        mvaddch(cy, cx + 1, ']');
    } else {
        mvaddch(cy, cx, '+');
    }
    attroff(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
}

/* draw_joint — single 'O' marker at the intermediate joint where the
 * top and bottom rods meet.  Painted AFTER the rods so the joint
 * glyph overwrites any rod glyph that lands on the same cell. */
static void draw_joint(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_JOINT) | A_BOLD);
    mvaddch(cy, cx, 'O');
    attroff(COLOR_PAIR(PAIR_JOINT) | A_BOLD);
}

/* draw_end_bob — '(@)' bracketed bob at the chain tip.  Three cells
 * wide so the tip reads as a substantial weight at the end of the
 * arm (not just a dot).  Falls back to a single '@' near edges. */
static void draw_end_bob(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_BOB) | A_BOLD);
    if (cx > 0 && cx < cols - 1) {
        mvaddch(cy, cx - 1, '(');
        mvaddch(cy, cx,     '@');
        mvaddch(cy, cx + 1, ')');
    } else {
        mvaddch(cy, cx, '@');
    }
    attroff(COLOR_PAIR(PAIR_BOB) | A_BOLD);
}

/* trail_glyph_for_band — vary the glyph by age tier so the trail
 * gains visual texture in addition to its colour fade.  Newest tier
 * gets the most substantial mark; oldest tier gets the most discreet. */
static inline char trail_glyph_for_band(int band)
{
    /* 4 bands: 3 = newest, 0 = oldest */
    if (band >= TRAIL_BAND_COUNT - 1)   return 'o';   /* newest 25%  */
    if (band == TRAIL_BAND_COUNT - 2)   return '.';
    if (band == TRAIL_BAND_COUNT - 3)   return '.';
    return ',';                                       /* oldest 25%  */
}

/* trail_paint — walk the ring oldest→newest, drawing each sample in
 * the band that corresponds to its age (0 = oldest = dimmest, N-1 =
 * newest = brightest).  Glyph varies with band so the trail reads as
 * a fading tail, not a string of identical dots.
 *
 *   STEP 1 — short-circuit on empty
 *   STEP 2 — locate oldest sample in the ring
 *   STEP 3 — for each sample: derive (age, band, cell) and plot it
 */
static void trail_paint(const Trail *t, int cols, int rows)
{
    /* STEP 1 */
    if (t->count == 0) return;

    /* STEP 2 */
    int oldest = trail_oldest_index(t);

    /* STEP 3 */
    for (int i = 0; i < t->count; i++) {
        int sample_index = (oldest + i) % TRAIL_MAX;
        int age          = t->count - 1 - i;          /* 0 = newest */
        int band         = trail_band_for_age(age, t->count);

        int cx = px_to_cell_x(t->px[sample_index]);
        int cy = px_to_cell_y(t->py[sample_index]);
        if (!in_drawable(cx, cy, cols, rows)) continue;

        int pair = PAIR_TRAIL_BASE + band;
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(cy, cx, (chtype)(unsigned char)trail_glyph_for_band(band));
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * PendulumCellPositions — the three (x, y) cell positions that
 * scene_paint needs in order to draw the chain: anchor, joint (bob1),
 * tip (bob2).  Computed from the pendulum's pixel-space state by
 * pendulum_compute_cell_positions().
 *
 * INTENT
 *   Hoists the px→cell conversion out of scene_paint so the painter
 *   reads as a back-to-front layering call list, not a mix of
 *   coordinate-conversion math and ncurses calls.
 *
 * MEMBER LOGIC
 *   anchor_*  : pivot (top of upper rod, where '[+]' is drawn).
 *   joint_*   : midpoint between rods (where 'O' is drawn).
 *   tip_*     : end of lower rod (where '(@)' is drawn, and the
 *               position the trail records).
 */
typedef struct {
    int anchor_x, anchor_y;
    int joint_x,  joint_y;
    int tip_x,    tip_y;
} PendulumCellPositions;

/* pendulum_compute_cell_positions — forward-kinematics → cells.
 * Runs pendulum_bob_positions() to get the two bobs in pixel space,
 * then funnels every coordinate through px_to_cell_{x,y} so the rest
 * of the renderer can work in pure cell space. */
static PendulumCellPositions pendulum_compute_cell_positions(const DoublePendulum *p)
{
    float b1x, b1y, b2x, b2y;
    pendulum_bob_positions(p, &b1x, &b1y, &b2x, &b2y);

    PendulumCellPositions pos;
    pos.anchor_x = px_to_cell_x(p->anchor.px);
    pos.anchor_y = px_to_cell_y(p->anchor.py);
    pos.joint_x  = px_to_cell_x(b1x);
    pos.joint_y  = px_to_cell_y(b1y);
    pos.tip_x    = px_to_cell_x(b2x);
    pos.tip_y    = px_to_cell_y(b2y);
    return pos;
}

/* scene_paint — render one frame, back→front so foreground markers
 * overwrite background detail:
 *
 *   STEP 0 — convert pendulum to CELL positions (one place, once)
 *   STEP 1 — TRAIL : oldest→newest, four-tier glyph + colour fade
 *   STEP 2 — RODS  : direction-aware Bresenham, top in PAIR_ROD_TOP,
 *                    bottom in PAIR_ROD_BOT
 *   STEP 3 — PIVOT : '[+]' bracket at the anchor
 *   STEP 4 — JOINT : 'O' marker where the two rods meet
 *   STEP 5 — BOB   : '(@)' bracketed weight at the chain tip
 */
static void scene_paint(const Scene *s, int cols, int rows)
{
    PendulumCellPositions p = pendulum_compute_cell_positions(&s->pendulum);

    trail_paint(&s->trail, cols, rows);
    draw_rod         (p.anchor_x, p.anchor_y, p.joint_x, p.joint_y,
                      PAIR_ROD_TOP, cols, rows);
    draw_rod         (p.joint_x,  p.joint_y,  p.tip_x,   p.tip_y,
                      PAIR_ROD_BOT, cols, rows);
    draw_pivot_marker(p.anchor_x, p.anchor_y, cols, rows);
    draw_joint       (p.joint_x,  p.joint_y,  cols, rows);
    draw_end_bob     (p.tip_x,    p.tip_y,    cols, rows);
}

static void hud_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DOUBLE PENDULUM ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    char buf[HUD_COLS + 1];
    const PendulumPreset *active = preset_state_active(&s->preset);
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d] ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* HUD_PARAM_CELL_WIDTH_* — printed widths of the three row-1 cells
 * including their leading + trailing spaces.  Used to lay them out
 * sequentially without overlap.  Sized to the fixed-width %-8s /
 * %4d format specifiers above. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

/* hud_param_row — three side-by-side cells on row 1:
 *
 *   STEP 1 — PRESET cell    (bold, primary)
 *   STEP 2 — THEME  cell    (bold, primary)
 *   STEP 3 — TRAIL  cell    (non-bold so it reads as a secondary metric)
 *
 * Each cell uses a fixed printed width so the layout doesn't shift
 * as the preset / theme names change. */
static void hud_param_row(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    /* STEP 1 — preset */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " preset:%-8s ", preset_state_active(&s->preset)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    /* STEP 2 — theme */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    /* STEP 3 — trail length (secondary metric, non-bold) */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " trail:%4d/%d ", s->trail.count, TRAIL_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top_left_title();
    hud_top_right_status(sc->cols, fps, sim_fps, s);
    hud_param_row(s);
    hud_bottom_hint(sc->rows);
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — the top-level composition root.
 *
 * INTENT
 *   Owns the Scene (mutable simulation state), the Screen (terminal
 *   adapter), the simulation tick rate (sim_fps), the chosen map
 *   size, and the two volatile flags written from signal handlers.
 *   Everything else in the program is reached through this struct —
 *   there is no other mutable global state.  This makes the program
 *   read top-down: main() = "drive the App"; each helper = "do one
 *   named thing to the App".
 *
 *   Concentrating mutable state in one struct also disciplines the
 *   signal-handling story: the SIGWINCH and SIGINT handlers may only
 *   touch volatile sig_atomic_t fields on g_app — every other piece
 *   of state is reachable only through main-loop functions.
 *
 * CONTEXT
 *   Exactly ONE instance exists: the file-scope g_app (immediately
 *   below this struct).  Signal handlers need a callable-from-signal
 *   target, and POSIX async-signal-safety requires that to be a
 *   global; hence g_app rather than a stack-local App.  All non-signal
 *   accesses pass an App* explicitly so the rest of the file remains
 *   free of global lookups.
 *
 *   Lifetime:
 *     app_bootstrap    → fills scene + screen + sim_fps + map dims
 *     main loop        → mutates everything except 'running'
 *                        which is read in the while-condition
 *     atexit(cleanup)  → calls endwin() as a final safety net
 *
 * MEMBER LOGIC
 *   scene       : §8 Scene — pendulum, trail, preset, palette, paused.
 *                 The simulation; "what the user sees".
 *   screen      : §9 Screen — cols / rows + ncurses lifecycle handle.
 *                 The output device.
 *   sim_fps     : current PHYSICS tick rate in Hz.  Distinct from
 *                 RENDER_FPS_TARGET — the integrator can step at any
 *                 rate while the renderer stays locked at 60 fps.
 *                 Adjusted by ] / [.  Clamped to [SIM_FPS_MIN, MAX].
 *   map_w       : cells of horizontal SIMULATION area (full terminal).
 *   map_h       : cells of vertical SIMULATION area (= screen.rows -
 *                 HUD_BAND_RESERVED_ROWS).  These are cached on App so
 *                 scene_compute_geometry has stable inputs across the
 *                 frame; recomputed by app_pick_map_size on resize.
 *   running     : 0 → main loop exits.  Set by SIGINT/SIGTERM handler
 *                 and by app_handle_key on q/ESC.  Declared
 *                 volatile sig_atomic_t for async-signal safety per
 *                 POSIX.1-2017 §2.4.3 (signal actions).
 *   need_resize : 1 → next frame triggers a SIGWINCH rebuild via
 *                 app_handle_pending_resize.  Set by the SIGWINCH
 *                 handler.  Same volatile sig_atomic_t requirement.
 *
 * REFERENCES (numbered in file-header References block)
 *   [7] Fiedler "Fix Your Timestep!" — explains why sim_fps and
 *       RENDER_FPS_TARGET are kept as INDEPENDENT knobs on the App:
 *       physics determinism vs render smoothness are orthogonal.
 *   [9] Gookin "Programmer's Guide to NCurses" Ch. 11 (Signal
 *       Handling) — SIGWINCH pattern reproduced by .need_resize +
 *       app_handle_pending_resize.
 *   • POSIX.1-2017 §2.4.3 — async-signal safety rules that justify
 *       the volatile sig_atomic_t typing of running / need_resize.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* signal stubs — the OS only allows touching volatile sig_atomic_t. */
static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* main_install_signal_handlers — wire SIGINT/TERM → quit and SIGWINCH
 * → "resize on next frame".  Called once before the main loop.  Kept
 * outside main() so main() reads as a pseudocode driver. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_pick_map_size — clamp the screen dims minus the HUD band into a
 * sensible (MIN..MAX) cell-count window.  Called both at bootstrap
 * and after every resize. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* app_bootstrap — full first-time initialisation.
 *   STEP 1 — seed the RNG (used only for srand — physics is deterministic)
 *   STEP 2 — set running + default sim rate
 *   STEP 3 — bring up ncurses + measure terminal
 *   STEP 4 — pick the map size + build the Scene
 */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — rebuild screen + scene if SIGWINCH set
 * the flag.  Caller must reset its accumulators after this returns
 * so the new frame budget is measured from "now". */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_resize(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_compute_frame_dt — sample the wall clock since last frame and
 * cap the result to SIM_MAX_FRAME_DT_MS so a paused-then-resumed
 * terminal can't dump a huge backlog into the integrator. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler's accumulator pattern: keep
 * pulling FIXED-SIZE physics steps out of `sim_accum` until what's
 * left is less than one tick.  Decouples render rate (variable) from
 * physics rate (constant, deterministic).
 *
 * See header ref [7] Fiedler "Fix Your Timestep!" for the canonical
 * writeup of why a variable-dt integrator is a non-starter for chaotic
 * systems — different machine speeds would produce different bob
 * trajectories.  Capping dt (see app_compute_frame_dt) prevents the
 * "spiral of death" where a slow frame queues up more substeps than
 * the next frame can process. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — every FPS_UPDATE_MS, compute frames/sec from
 * the count + accumulated time and reset the window. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep just long enough that the
 * render loop runs at RENDER_FPS regardless of how fast the rest of
 * the frame took.  Sleep BEFORE drawing so the I/O time doesn't add
 * jitter on top of the budget. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint scene + HUD into ncurses' back buffer
 * and flip.  All ncurses output is funnelled through here. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* app_sim_rate_faster / slower — clamped mutators for the physics
 * tick rate.  Named functions so the binding table reads cleanly. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* app_toggle_pause / app_reset_pendulum — one-liners, but named so
 * the key-binding table reads as a list of intentions, not actions. */
static void app_toggle_pause   (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_pendulum (App *app) { scene_reset(&app->scene); }

/* app_cycle_preset_next / prev — advance the PresetState wrapper, then
 * load the new active row onto the pendulum.  Trail is cleared too so
 * the previous preset's scribble doesn't bleed into the new one. */
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene);
}

/* app_cycle_theme_next / prev — advance the PaletteState wrapper and
 * push the new palette into ncurses.  No reset needed — physics keeps
 * running through the colour change. */
static void app_cycle_theme_next(App *app)
{
    palette_state_cycle_next(&app->scene.palette);
    scene_apply_theme(&app->scene);
}
static void app_cycle_theme_prev(App *app)
{
    palette_state_cycle_prev(&app->scene.palette);
    scene_apply_theme(&app->scene);
}

/* app_poll_keyboard — non-blocking getch() + dispatch via app_handle_key.
 * Returns true to keep looping, false on q/ESC. */
static bool app_handle_key(App *app, int ch);

static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* app_handle_key — key-binding table.  Each case calls ONE named
 * mutator from above; nothing here does modular arithmetic, struct
 * dereferences, or ncurses calls.  Reads as a list of intentions:
 *
 *     "q  → quit
 *      space → toggle pause
 *      r → reset
 *      ] / [ → faster / slower sim
 *      t / T → cycle palette
 *      n / p → cycle preset" */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_pendulum   (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the wall-clock + accumulator state the main loop
 * threads through its per-frame helpers.
 *
 * INTENT
 *   Bundles the 5 timing locals that main() would otherwise carry
 *   loose (frame_time, sim_accum, fps_accum, frame_count, fps_display)
 *   into one named concept.  Mirrors the App / Scene pattern: every
 *   piece of mutable state belongs to a NAMED OBJECT, not to main's
 *   stack frame.  The main loop becomes a 10-line pseudocode driver
 *   over FrameClock + App.
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock timestamp (ns) at the boundary of the
 *                 LAST frame.  app_compute_frame_dt subtracts the
 *                 current clock from this to get dt.
 *   sim_accum   : leftover ns NOT YET consumed by a physics tick.
 *                 The Fiedler fixed-step accumulator.
 *   fps_accum   : ns of frame time accumulated since the last fps
 *                 readout refresh.
 *   frame_count : number of frames in the current fps window.
 *   fps_display : most recent fps figure, smoothed by the FPS_UPDATE_MS
 *                 window length.  Displayed in the top-right HUD.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

/* frame_clock_init — boot the clock at the current wall-time and zero
 * the accumulators.  Called once before the main loop starts. */
static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

/* frame_clock_reset_after_resize — re-baseline the frame_time and
 * drop any pending physics-tick backlog.  Called once after every
 * SIGWINCH rebuild; without this the next dt would include all the
 * wall-time spent rebuilding the screen. */
static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

/* frame_clock_advance — fold the new frame's dt into both the sim
 * accumulator (for fixed-step drain) and the fps meter (for the
 * HUD), and bump the frame counter.  One named call replaces three
 * loose accumulator updates in the loop body. */
static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* main — the whole simulation as a pseudocode driver.
 *
 *   install signal handlers
 *   bootstrap (curses + scene)
 *   init frame clock
 *   while running:
 *     if pending resize → rebuild screen, reset clock
 *     dt ← wall-clock since last frame
 *     advance clock (sim_accum, fps_accum, frame_count)
 *     drain fixed-timestep physics
 *     update fps meter
 *     throttle to render target
 *     present frame
 *     poll keyboard (may flip running)
 *   tear down
 */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
