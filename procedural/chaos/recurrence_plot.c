/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * recurrence_plot.c
 *   — Recurrence Plot.  A 2-D image R(i, j) = 1 if the trajectory's
 *     state at time i is "close" (within ε) to its state at time j.
 *     The image's TEXTURE reveals the underlying dynamics: periodic
 *     systems make sharp parallel diagonals; chaotic systems make
 *     short broken segments; pure noise makes a uniform speckle.
 *
 * DEMO: Sample a trajectory at N regularly-spaced times, then for
 *       every pair (i, j) check whether the distance between sample
 *       i and sample j is below ε.  Paint white if close, black if
 *       not.  30 presets in 8 groups n/p-cyclable, ordered simple →
 *       complex:
 *         A) Trivial / linear (CONSTANT → full grid, LINEAR → diag stripe)
 *         B) Simple periodic (SINE × 3 frequencies, SQUARE, TRIANGLE)
 *         C) Compound periodic (SAW, DUET, QUASI-periodic, BEAT)
 *         D) Damped / gated (DAMPED, BURST, WANE)
 *         E) Stochastic (NOISE, WALK, AR1 × 2 coefficients, BROWN)
 *         F) Logistic-map cascade (P2 → P4 → ONSET → CHAOS → P3 window)
 *         G) Other chaos (TENT, LORENZ x-coord, HENON)
 *         H) Mixed / exotic (RAMP_SINE, STEPS)
 *       Each generator produces a qualitatively distinct recurrence
 *       pattern — periodic signals make clean parallel diagonals;
 *       chaotic signals make broken speckle; walks make growing
 *       blocks along the main diagonal; etc.
 *
 * Study alongside:
 *   ./bifurcation.c        — diagnostic that exposes chaos via the
 *                            PARAMETER axis instead of time axis.
 *   ./sensitive_dependence.c — diagnostic that exposes chaos via
 *                            TWO trajectories instead of one.
 *
 * Section map:
 *   §1  config   — constants, 30-preset table, SignalKind enum, themes
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — HUD pairs + 10 themes
 *   §5  physics  — Lorenz (system + state) + named-stage RK4
 *   §6  recurrence — RecurrenceImage: samples[] + R[i,j] matrix
 *   §7  state    — PresetState + PaletteState typed wrappers
 *   §8  scene    — SignalState + signal_step dispatcher + Scene composite
 *   §9  screen   — recurrence_paint, HUD writers
 *   §10 app      — signals, resize, key dispatch table, FrameClock,
 *                  main pseudocode driver + named loop helpers
 *
 * Keys: q/ESC quit | space pause | r reset | n/p preset | t/T theme | ]/[ Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/recurrence_plot.c \
 *       -o recurrence_plot -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two sources:
 *                 1) Sine wave  y(t) = sin(t·ω)  (1-D state).
 *                 2) Lorenz     project to scalar x(t).
 *                 We collect N samples (one per tick over the
 *                 build-up phase).  The recurrence matrix R is
 *                 N × N where R(i, j) = 1 iff |x_i − x_j| < ε.
 *                 The image fills progressively over BUILD_TICKS
 *                 ticks (one row per tick) so the user sees it form.
 *
 * Data-struct   : float samples[N_MAX] + uint8 R[N_MAX × N_MAX].
 *                 N is bounded by min(cols, rows-2) so each pixel
 *                 of R maps to exactly one cell.  ε is preset-tuned.
 *
 * Rendering     : R(i, j)=1 → '#' in PAIR_RECUR; =0 → blank.  Two
 *                 reference glyphs along the main diagonal (always
 *                 1 since R(i, i) = always close) anchor the image.
 *
 * References    : Numbered so inline code can cite [n].
 *
 *   RECURRENCE PLOTS — the technique this file implements
 *   [1] Eckmann, J.-P., Kamphorst, S. O., Ruelle, D. (1987).
 *       "Recurrence Plots of Dynamical Systems", Europhysics
 *       Letters 4(9), pp. 973-977.  THE original paper:
 *       defines R(i,j) = Θ(ε − ‖x_i − x_j‖) and identifies
 *       the four texture classes used in our zoo.
 *   [2] Marwan, N., Romano, M. C., Thiel, M., Kurths, J. (2007).
 *       "Recurrence plots for the analysis of complex systems",
 *       Physics Reports 438(5-6), pp. 237-329.  The definitive
 *       modern review; introduces Recurrence Quantification
 *       Analysis (RQA) and the texture-classification table
 *       referenced in HOW TO VERIFY below.
 *   [3] Webber, C. L. Jr., Zbilut, J. P. (2005).  "Recurrence
 *       Quantification Analysis of Nonlinear Dynamical
 *       Systems", in *Tutorials in Contemporary Nonlinear
 *       Methods for the Behavioral Sciences* (NSF), ch. 2.
 *       Pedagogical walk-through with worked examples.
 *
 *   SIGNAL SOURCES — every preset draws from one of these systems
 *   [4] Lorenz, E. N. (1963).  "Deterministic Nonperiodic Flow",
 *       J. Atmos. Sci. 20, pp. 130-141.  σ=10, ρ=28, β=8/3
 *       canonical values used by SIG_LORENZ_X.
 *   [5] Hénon, M. (1976).  "A two-dimensional mapping with a
 *       strange attractor", Comm. Math. Phys. 50, pp. 69-77.
 *       a=1.4, b=0.3 used by SIG_HENON_X/Y.
 *   [6] May, R. M. (1976).  "Simple mathematical models with
 *       very complicated dynamics", Nature 261, pp. 459-467.
 *       Logistic map x_{n+1} = r·x_n·(1−x_n); the period-
 *       doubling cascade we sample in SIG_LOGISTIC.
 *   [7] Box, G. E. P., Jenkins, G. M. (1970).  *Time Series
 *       Analysis: Forecasting and Control*.  Background for
 *       AR(1) and ARMA signals (SIG_AR1).
 *
 *   NUMERICAL BUILDING BLOCKS
 *   [8] Press, W. H. et al. (2007).  *Numerical Recipes* (3rd
 *       ed.), §17.1.  Classical RK4 with the Butcher
 *       (1/6, 1/3, 1/3, 1/6) tableau used by lorenz_rk4_full.
 *   [9] Marsaglia, G. (2003).  "Xorshift RNGs", J. Statistical
 *       Software 8(14).  The 32-bit xorshift PRNG that drives
 *       all stochastic signals (random walk, AR(1), noise).
 *
 *   FRAMEWORK
 *  [10] Fiedler, G. (2004).  "Fix Your Timestep!", gafferongames.com.
 *       Accumulator pattern used by app_drain_fixed_timestep so the
 *       Lorenz integrator and the screen redraw decouple.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * For each pair of moments (i, j) in a trajectory, ask: "Were the
 * system's states at times i and j roughly the same?"  Plot the
 * answer as a black/white pixel.  The result is a 2-D fingerprint
 * of the dynamics, with characteristic textures:
 *
 *   • Long straight diagonals      → strict periodicity
 *   • Broken diagonals + speckle   → chaos
 *   • Uniform speckle              → noise
 *   • Empty corners                → trend / non-stationarity
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The image is always symmetric (R(i,j) = R(j,i)) and always has a
 * unit diagonal (R(i,i) = 1).  Off-diagonal lines parallel to the
 * main diagonal of length L mean two segments of the trajectory
 * stayed close for L steps — a "shadow" of one orbit by another.
 *
 * KEY FORMULAS
 * ────────────
 *   R(i, j) = 1 if |x_i − x_j| < ε else 0
 *   PERIODIC: x_t = sin(t · ω)
 *   CHAOTIC:  x_t from Lorenz with (σ=10, ρ=28, β=8/3)
 *
 * HOW TO VERIFY  (texture classifications follow Marwan et al. [2] §3)
 * ─────────────
 *  • PERIODIC: image fills with neat parallel diagonal stripes
 *    spaced one period of the sine apart.
 *  • CHAOTIC: image fills with short broken diagonal segments and
 *    a "cloud" texture — no long straight lines.
 *  • NOISE:   uniform random speckle, no diagonal structure.
 *  • DRIFT:   bright band along the main diagonal, fading toward
 *             the corners — non-stationarity / trend.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── OVERALL PSEUDOCODE ───────────────────────────────────────────────── *
 *
 *   bootstrap()                                                 // §10
 *       set up ncurses, palette, signal handlers
 *       App.scene  = fresh Scene with PRESET_SINE_MED + theme 0
 *       App.side   = clamp(min(cols, rows-2), 16, N_MAX)        // §10
 *
 *   FrameClock clk = frame_clock_init()                         // §10
 *   while App.running:                                          // main()
 *       if SIGWINCH: rebuild Screen + Scene; reset clk          // §10
 *       dt = app_compute_frame_dt()                             // §10
 *       frame_clock_advance(clk, dt)                            // §10
 *       while clk.sim_accum >= TICK_NS(App.sim_fps):            // §10
 *           scene_tick(Scene, dt_sec)                           // §8
 *               if paused or picture complete: return
 *               x = signal_step(SignalState, active_preset)     // §8
 *               R.samples[collected++] = x                      // §6
 *               recurrence_compute_row(R, collected-1, ε)       // §6
 *       app_update_fps_meter()                                  // §10
 *       app_throttle_to_render_target()                         // §10
 *       app_present_frame():                                    // §10
 *           erase()
 *           recurrence_paint(R, cols, rows)                     // §9
 *               for each (i,j): plot '#' if R[i,j], '\' on i==j
 *           hud_top + hud_param + hud_hint                      // §9
 *           wnoutrefresh + doupdate                             // §9
 *       app_poll_keyboard():                                    // §10
 *           q/ESC          → quit
 *           SPC            → toggle pause
 *           r              → reset picture (re-collect samples)
 *           [/]            → slower/faster sim
 *           n/p / t/T      → cycle preset / theme
 *
 *   teardown(): screen_free → endwin (via atexit cleanup)       // §10
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
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_RECUR       =   3,
    PAIR_DIAG        =   4,
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

#define N_MAX                    140    /* trajectory length & matrix side */
#define LORENZ_DT                 0.02f
#define HENON_A                   1.4f
#define HENON_B                   0.3f

/*
 * SignalKind — which time-series generator drives the recurrence plot.
 *
 * INTENT
 *   The enum is the dispatch tag for signal_step.  Each kind produces
 *   a qualitatively distinct recurrence pattern (per [1] [2]) so
 *   cycling through them is a guided tour of the recurrence-analysis
 *   visual zoo:
 *
 *     Simple periodic     : sine / square / triangle / sawtooth
 *     Compound periodic   : two sines, quasi-periodic, beating
 *     Damped / gated      : damped sine, bursts, amplitude wane
 *     Stochastic          : white noise, random walk, AR(1), brownian
 *     Discrete-map chaos  : logistic (multiple r), tent, Hénon
 *     Continuous chaos    : Lorenz
 *     Mixed / exotic      : linear ramp + sine, step jumps, constant,
 *                           pure linear (only-diagonal limit)
 *
 * CONTEXT
 *   Stored as RPPreset::kind; switched on by signal_step (§8).
 *   Adding a new generator means: add an enum entry, add a row in
 *   presets[], add a case in signal_step.
 *
 * MEMBER LOGIC
 *   See the per-line comments — each entry names the formula it
 *   implements.  Most entries read at least p1 from the preset row.
 *
 * REFERENCES
 *   [1] Eckmann et al. 1987; [2] Marwan et al. 2007 (texture classes).
 *   [4]-[7], [9] for the per-generator formulas.
 */
typedef enum {
    SIG_CONSTANT = 0,    /* x(t) = const                  — full grid    */
    SIG_LINEAR,          /* x(t) = a·t                    — diag stripe  */
    SIG_SINE,            /* sin(ω t)                                     */
    SIG_SQUARE,          /* sign(sin(ω t))                               */
    SIG_TRIANGLE,        /* triangle wave                                */
    SIG_SAWTOOTH,        /* sawtooth                                     */
    SIG_TWO_SINES,       /* sin(ω₁ t) + 0.5·sin(ω₂ t)                    */
    SIG_QUASI_GOLDEN,    /* sin(ω t) + sin(φ·ω t)  (φ = golden ratio)    */
    SIG_BEAT,            /* sin(ω₁ t) + sin(ω₂ t), ω₁ ≈ ω₂  → beats      */
    SIG_DAMPED_SINE,     /* e^(-γ t) · sin(ω t)                          */
    SIG_BURST,           /* sin(ω t) gated on/off every burst_period     */
    SIG_WANE,            /* sin(ω t) with slowly shrinking amplitude     */
    SIG_NOISE,           /* uniform white noise [-1, 1]                  */
    SIG_RANDOM_WALK,     /* cumulative noise                             */
    SIG_AR1,             /* x[t+1] = α·x[t] + noise                      */
    SIG_BROWNIAN,        /* random walk with larger steps                */
    SIG_LOGISTIC,        /* x[t+1] = r·x[t]·(1 - x[t])  (1-D map)        */
    SIG_TENT,            /* tent map x[t+1] = μ · min(x, 1-x)            */
    SIG_LORENZ_X,        /* Lorenz x-component (continuous chaos)        */
    SIG_HENON_X,         /* Hénon map x-component (2-D discrete chaos)   */
    SIG_RAMP_SINE,       /* a·t + sin(ω t)  — drift + oscillation        */
    SIG_STEP_JUMPS,      /* piecewise-constant random jumps              */
} SignalKind;

/*
 * Preset — the 30-entry index space for the presets[] table.
 *
 * INTENT
 *   Named indices into the presets[] table, grouped by signal
 *   character (A trivial → H mixed/exotic).  Ordering goes simple
 *   → complex so the user can scroll forward with 'n' and watch the
 *   recurrence picture monotonically grow in visual complexity.
 *   N_PRESETS is the count sentinel; PresetState cycles modulo it.
 *
 * CONTEXT
 *   Used only as table indices; the entries themselves are defined
 *   below in the presets[] static array.  Group dividers in the enum
 *   line up with dividers in the array initialiser.
 *
 * MEMBER LOGIC
 *   Each name follows the SIGNAL_SHAPE convention.  Groups:
 *     A (2)  : trivial / linear      — degenerate sanity cases
 *     B (5)  : simple periodic       — sine, square, triangle, …
 *     C (4)  : compound periodic     — two-sine, quasi, beat
 *     D (3)  : damped / gated        — envelope-shaped sinusoids
 *     E (5)  : stochastic            — noise, walk, AR(1), brownian
 *     F (5)  : logistic cascade      — r = 3.2, 3.5, 3.57, 3.9, 3.83
 *     G (3)  : other chaos           — tent, Lorenz, Hénon
 *     H (2)  : mixed / exotic        — ramp+sine, step jumps
 *
 * REFERENCES
 *   [2] Marwan et al. (2007) — the texture taxonomy these groups
 *       are designed to demonstrate.
 */
typedef enum {
    /* Group A — trivial / linear (2) */
    PRESET_CONSTANT = 0,
    PRESET_LINEAR,
    /* Group B — simple periodic (5) */
    PRESET_SINE_SLOW,
    PRESET_SINE_MED,
    PRESET_SINE_FAST,
    PRESET_SQUARE,
    PRESET_TRIANGLE,
    /* Group C — compound periodic (4) */
    PRESET_SAW,
    PRESET_DUET,
    PRESET_QUASI,
    PRESET_BEAT,
    /* Group D — damped / gated (3) */
    PRESET_DAMPED,
    PRESET_BURST,
    PRESET_WANE,
    /* Group E — stochastic (4) */
    PRESET_NOISE,
    PRESET_WALK,
    PRESET_AR_STRONG,
    PRESET_AR_WEAK,
    PRESET_BROWN,
    /* Group F — logistic-map cascade (5) */
    PRESET_LOGI_P2,
    PRESET_LOGI_P4,
    PRESET_LOGI_ONSET,
    PRESET_LOGI_CHAOS,
    PRESET_LOGI_P3,
    /* Group G — other chaos (3) */
    PRESET_TENT,
    PRESET_LORENZ,
    PRESET_HENON,
    /* Group H — mixed / exotic (2) */
    PRESET_RAMP_SINE,
    PRESET_STEPS,
    N_PRESETS,
} Preset;

/*
 * RPPreset — one named time-series generator with its parameters.
 *
 * INTENT
 *   Each row of the presets[] table fully describes one entry in
 *   the 30-preset zoo: WHICH generator (kind), HOW STRICT the
 *   recurrence test is (epsilon), and the up-to-three free
 *   parameters the generator expects.  Cycling through this
 *   table is the only way the user changes what the plot shows.
 *
 * CONTEXT
 *   Read by §8 signal_step (dispatches on kind, reads p1/p2/p3),
 *   by scene_active_eps (returns epsilon), and by the HUD.  The
 *   PresetState wrapper in §7 stores an index INTO this table.
 *
 * MEMBER LOGIC
 *   name    : 8-char HUD label, padded to align the parameter column.
 *   kind    : which generator (SignalKind, §1 enum).
 *   epsilon : recurrence threshold |x(i) − x(j)| < ε → R(i,j) = 1.
 *             Tuned per signal so the visible recurrence density is
 *             comparable across presets (sin ∈ [-1, 1] needs ε ≈ 0.1
 *             while a random walk needs ε ≈ 0.5 for similar density).
 *   p1,p2,p3: signal-specific parameters (frequency, damping rate,
 *             logistic r, etc.).  Unused fields are 0; consult
 *             signal_step() for the per-kind convention.
 *
 * REFERENCES
 *   [1] Eckmann, Kamphorst, Ruelle (1987) — definition of ε.
 *   [2] Marwan et al. (2007) §3.2 — ε-selection heuristics.
 */
typedef struct {
    const char *name;
    SignalKind  kind;
    float       epsilon;
    float       p1, p2, p3;
} RPPreset;

/* 30 named time-series, ordered simple → complex.  Group divisions
 * match the SignalKind comment above. */
static const RPPreset presets[N_PRESETS] = {
    /* Group A — trivial / linear ─────────────────────────────────── */
    { "CONSTANT", SIG_CONSTANT,     0.10f,  0.50f,  0.0f,  0.0f },
    { "LINEAR  ", SIG_LINEAR,       0.10f,  0.015f, 0.0f,  0.0f },
    /* Group B — simple periodic ─────────────────────────────────── */
    { "SINE_SLO", SIG_SINE,         0.10f,  0.15f,  0.0f,  0.0f },
    { "SINE_MED", SIG_SINE,         0.10f,  0.45f,  0.0f,  0.0f },
    { "SINE_FAS", SIG_SINE,         0.10f,  0.95f,  0.0f,  0.0f },
    { "SQUARE  ", SIG_SQUARE,       0.10f,  0.40f,  0.0f,  0.0f },
    { "TRIANGLE", SIG_TRIANGLE,     0.10f,  0.40f,  0.0f,  0.0f },
    /* Group C — compound periodic ───────────────────────────────── */
    { "SAW     ", SIG_SAWTOOTH,     0.10f,  0.40f,  0.0f,  0.0f },
    { "DUET    ", SIG_TWO_SINES,    0.15f,  0.30f,  0.71f, 0.0f },
    { "QUASI   ", SIG_QUASI_GOLDEN, 0.15f,  0.35f,  0.0f,  0.0f },
    { "BEAT    ", SIG_BEAT,         0.15f,  0.50f,  0.55f, 0.0f },
    /* Group D — damped / gated ──────────────────────────────────── */
    { "DAMPED  ", SIG_DAMPED_SINE,  0.10f,  0.50f,  0.015f,0.0f },
    { "BURST   ", SIG_BURST,        0.10f,  0.50f, 20.0f,  0.0f },
    { "WANE    ", SIG_WANE,         0.10f,  0.45f,  0.006f,0.0f },
    /* Group E — stochastic ──────────────────────────────────────── */
    { "NOISE   ", SIG_NOISE,        0.30f,  0.0f,   0.0f,  0.0f },
    { "WALK    ", SIG_RANDOM_WALK,  0.80f,  0.0f,   0.0f,  0.0f },
    { "AR_STRO ", SIG_AR1,          0.30f,  0.95f,  0.0f,  0.0f },
    { "AR_WEAK ", SIG_AR1,          0.30f,  0.50f,  0.0f,  0.0f },
    { "BROWN   ", SIG_BROWNIAN,     2.00f,  0.0f,   0.0f,  0.0f },
    /* Group F — logistic-map cascade ────────────────────────────── */
    { "LOGI_P2 ", SIG_LOGISTIC,     0.04f,  3.20f,  0.0f,  0.0f },
    { "LOGI_P4 ", SIG_LOGISTIC,     0.04f,  3.50f,  0.0f,  0.0f },
    { "LOGI_ON ", SIG_LOGISTIC,     0.06f,  3.57f,  0.0f,  0.0f },
    { "LOGI_CH ", SIG_LOGISTIC,     0.06f,  3.70f,  0.0f,  0.0f },
    { "LOGI_P3 ", SIG_LOGISTIC,     0.06f,  3.835f, 0.0f,  0.0f },
    /* Group G — other chaos ─────────────────────────────────────── */
    { "TENT    ", SIG_TENT,         0.04f,  2.00f,  0.0f,  0.0f },
    { "LORENZ  ", SIG_LORENZ_X,     1.50f,  0.0f,   0.0f,  0.0f },
    { "HENON   ", SIG_HENON_X,      0.20f,  0.0f,   0.0f,  0.0f },
    /* Group H — mixed / exotic ──────────────────────────────────── */
    { "RAMP_SIN", SIG_RAMP_SINE,    0.15f,  0.40f,  0.003f,0.0f },
    { "STEPS   ", SIG_STEP_JUMPS,   0.10f,  0.05f,  0.0f,  0.0f },
};

/*
 * Theme — one named palette for the recurrence picture.
 *
 * INTENT
 *   The visual identity of a recurrence plot is dominated by exactly
 *   two colours: the matrix dots (R(i,j) = 1) and the always-on main
 *   diagonal.  Wrapping those two short codes plus a name keeps the
 *   themes[] table a flat readable catalogue.
 *
 * CONTEXT
 *   Indexed by PaletteState (§7); applied by theme_apply (§3); cycled
 *   by the t/T keys via app_cycle_theme_next/prev (§10).  All palette
 *   values follow the project Brightness Rule (CLAUDE.md): every code
 *   lives in the bright half of the 256-colour cube so the dots stay
 *   visible against the default black background.
 *
 * MEMBER LOGIC
 *   name  : 7-char HUD label.
 *   recur : 256-colour code for R(i,j) = 1 dots (PAIR_RECUR).
 *   diag  : 256-colour code for the main diagonal (PAIR_DIAG).
 *
 * REFERENCES
 *   CLAUDE.md "Theme Palette Brightness".
 */
typedef struct { const char *name; short recur, diag; } Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", 117, 220 },
    { "MATRIX",  118, 226 },
    { "NOVA",    207, 226 },
    { "MONO",    253, 226 },
    { "OCEAN",   159, 226 },
    { "FIRE",    208, 226 },
    { "EARTH",   215, 226 },
    { "FOREST",  150, 226 },
    { "DESERT",  222, 226 },
    { "ARCTIC",  195, 226 },
};

/* ===================================================================== */
/* §2 clock + §3 color                                                    */
/* ===================================================================== */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* theme_apply — push the chosen palette to the ncurses pair table.
 *
 * DRIVER PSEUDOCODE
 *   if idx out of range:  idx = 0                    // graceful fallback
 *   if terminal has 256 colours:
 *       PAIR_RECUR ← themes[idx].recur               // matrix dots
 *       PAIR_DIAG  ← themes[idx].diag                // main diagonal
 *   else:
 *       PAIR_RECUR ← CYAN ;  PAIR_DIAG ← YELLOW      // 8-colour fallback
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) {
        init_pair(PAIR_RECUR, t->recur, -1);
        init_pair(PAIR_DIAG,  t->diag,  -1);
    } else {
        init_pair(PAIR_RECUR, COLOR_CYAN, -1);
        init_pair(PAIR_DIAG,  COLOR_YELLOW, -1);
    }
}
static void color_init(void)
{ start_color(); use_default_colors();
  if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
  else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
  theme_apply(0); }

/* ===================================================================== */
/* §5  physics — Lorenz ODE (one of the 30 signal sources)                */
/* ===================================================================== */

/*
 * The Lorenz equation comes up in this file as ONE signal generator
 * among 30 (the SIG_LORENZ_X case in signal_step).  Even though we
 * never vary its parameters here, we still split it into
 * LorenzSystem + LorenzState + Lorenz composite so the §5 code reads
 * as a clean ODE + RK4 model the reader can study in isolation —
 * matching the abstraction layout of every other chaos demo in this
 * project (./poincare_section.c, ./strange_attractor.c).
 */

/* Canonical Lorenz parameters (Lorenz 1963 §III) — fixed for this
 * file because we use Lorenz as a fixed chaotic signal source, not
 * as a sweepable bifurcation parameter. */
#define LORENZ_SIGMA   10.0f
#define LORENZ_RHO     28.0f
#define LORENZ_BETA    (8.0f / 3.0f)

/*
 * Vec3 — generic 3-component vector.
 *
 * INTENT
 *   Generic numerical 3-vec; used both as a Lorenz phase point and
 *   as a Lorenz derivative dy/dt.  Flat by-value struct so RK4
 *   stages combine cheaply via state_add.
 *
 * CONTEXT
 *   Returned by lorenz_deriv, stored as LorenzState inside Lorenz,
 *   and (legacy) inlined as the Vec3 field of SignalState for the
 *   SIG_LORENZ_X case.
 *
 * MEMBER LOGIC
 *   x, y, z : the three scalar components.  Lorenz interprets them as
 *             (convection, x-temperature, y-temperature) gradients;
 *             other call sites just use them as a generic 3-tuple.
 *
 * REFERENCES
 *   [4] Lorenz 1963, eqs. 25-27.
 */
typedef struct {
    float x, y, z;
} Vec3;

/*
 * LorenzState — semantic alias for Vec3 at the "ODE phase point" use
 * site.  Same memory layout; the typedef just labels intent so calls
 * like state_add(LorenzState, dt, slope) read as ODE arithmetic, not
 * generic vector arithmetic.  No new fields.
 */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — invariant parameters of the Lorenz ODE.
 *
 * INTENT
 *   Split parameters away from state so lorenz_deriv is a pure
 *   function of (state, system).  Keeps the door open for sweeps
 *   (Phase 2 work) without rewiring callers, and matches the
 *   System+State+Composite layout used in every other chaos demo
 *   (poincare_section.c, double_pendulum.c, henon_heiles.c).
 *
 * CONTEXT
 *   In this file we use the canonical (σ=10, ρ=28, β=8/3); the
 *   system is never mutated at runtime.  lorenz_system_default()
 *   is the only constructor.
 *
 * MEMBER LOGIC
 *   sigma : Prandtl number σ (rate of convective heat transfer).
 *   rho   : Rayleigh number ρ (temperature gradient driving force).
 *   beta  : geometric factor β (= 8/3 for the original derivation).
 *
 * REFERENCES
 *   [4] Lorenz 1963 §III — "Numerical integration of the convection
 *       equations" with these specific parameter values.
 */
typedef struct {
    float sigma, rho, beta;
} LorenzSystem;

/*
 * Lorenz — composite: invariant system + current state.
 *
 * INTENT
 *   One value carrying everything needed to advance the ODE; matches
 *   the abstraction layout of every chaos demo in this project.
 *
 * CONTEXT
 *   Owned transiently inside lorenz_rk4 (the legacy Vec3 wrapper);
 *   not stored long-term because the §1 parameters are constant for
 *   this file.  lorenz_rk4_full takes Lorenz* directly.
 *
 * MEMBER LOGIC
 *   system : the (σ, ρ, β) triple — never mutated by lorenz_rk4_full.
 *   state  : the (x, y, z) phase point — mutated each RK4 step.
 *
 * REFERENCES
 *   [4] Lorenz 1963; [8] Numerical Recipes §17.1 for the integrator.
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

static inline LorenzSystem lorenz_system_default(void)
{
    return (LorenzSystem){ LORENZ_SIGMA, LORENZ_RHO, LORENZ_BETA };
}

/* lorenz_deriv — evaluate dy/dt = f(state, system).  Lorenz 1963
 * [4] §III equations 25-27; each line of the body maps 1-to-1 to
 * one published formula. */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);                 /* σ(y − x) */
    dy.y = s->x * (sys->rho - s->z) - s->y;            /* x(ρ − z) − y */
    dy.z = s->x * s->y - sys->beta * s->z;             /* xy − βz */
    return dy;
}

/* state_add — y + h·k, returned as a new LorenzState. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* RK4 Butcher tableau constants — Numerical Recipes [8] §17.1. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* rk4_butcher_weighted_average — (k₁ + 2k₂ + 2k₃ + k₄) / 6. */
static inline LorenzState rk4_butcher_weighted_average(
    const LorenzState *k1, const LorenzState *k2,
    const LorenzState *k3, const LorenzState *k4)
{
    LorenzState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* lorenz_rk4_full — one fixed-step RK4 update of a Lorenz composite.
 * Classical 4-stage Runge-Kutta; see Numerical Recipes [8] §17.1.
 *
 *   STAGE 1 — slope_start = f(y)
 *   STAGE 2 — slope_mid_1 = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2 = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end   = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 */
static void lorenz_rk4_full(Lorenz *L, float dt)
{
    const LorenzSystem *sys = &L->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    LorenzState slope_start    = lorenz_deriv(&L->state, sys);
    LorenzState midpoint_1     = state_add(&L->state, half_dt, &slope_start);
    LorenzState slope_mid_1    = lorenz_deriv(&midpoint_1, sys);
    LorenzState midpoint_2     = state_add(&L->state, half_dt, &slope_mid_1);
    LorenzState slope_mid_2    = lorenz_deriv(&midpoint_2, sys);
    LorenzState endpoint       = state_add(&L->state, dt, &slope_mid_2);
    LorenzState slope_end      = lorenz_deriv(&endpoint, sys);

    LorenzState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    L->state = state_add(&L->state, dt, &effective_slope);
}

/* lorenz_rk4 — adapter that lets signal_step pass its bare Vec3
 * lorenz-state without plumbing a whole Lorenz composite through
 * SignalState.  Builds a transient composite, RK4-steps it, copies
 * the new state back.  Canonical (σ, ρ, β); same numerics as
 * lorenz_rk4_full. */
static void lorenz_rk4(Vec3 *s, float dt)
{
    Lorenz tmp = { lorenz_system_default(), *s };
    lorenz_rk4_full(&tmp, dt);
    *s = tmp.state;
}

/* ===================================================================== */
/* §6  recurrence matrix                                                  */
/* ===================================================================== */

/*
 * RecurrenceImage — the recurrence matrix and its underlying samples.
 *
 * INTENT
 *   Implements Eckmann/Kamphorst/Ruelle [1]:
 *     R(i, j) = Θ(ε − |x_i − x_j|),  Θ = Heaviside step.
 *   We use scalar samples and the L¹ distance |x_i − x_j| rather
 *   than full state-space embedding because each preset already
 *   projects to a single observable (x-coordinate, scalar map
 *   iterate, scalar noise process) — the "scalar variant" in
 *   Marwan et al. [2] §2.1.
 *
 * CONTEXT
 *   Filled row-by-row in scene_tick (one row per fixed tick), so the
 *   user sees the matrix BUILD UP from the top instead of appearing
 *   in one frame.  Painted by recurrence_paint (§9).  Owned by Scene.
 *
 * MEMBER LOGIC
 *   n         : current side length (clamped to min(cols, rows-2)
 *               and to N_MAX).  Set by recurrence_reset.
 *   samples[] : the time-series x_0 .. x_{n-1}; fed by signal_step.
 *               Bounded at compile time so we never call malloc.
 *   R[]       : the n × n recurrence matrix flattened row-major;
 *               cell (i, j) lives at index i*n + j.  1 byte per cell
 *               is enough — values are 0 or 1.
 *   rows_done : how many rows of R have been filled; advances 1 per
 *               sample collected.  When rows_done == n the picture
 *               is complete and scene_tick goes idle.
 *
 * REFERENCES
 *   [1] Eckmann, Kamphorst, Ruelle (1987) — the R(i,j) definition.
 *   [2] Marwan et al. (2007) §2.1 — scalar / embedded variants.
 */
typedef struct {
    int     n;
    float   samples[N_MAX];
    uint8_t R[N_MAX * N_MAX];
    int     rows_done;     /* rows of R already computed */
} RecurrenceImage;

static void recurrence_reset(RecurrenceImage *r, int n)
{
    r->n = n; r->rows_done = 0;
    memset(r->R, 0, sizeof r->R);
}

/* recurrence_compute_row — fill one row of R using the [1] criterion. */
static void recurrence_compute_row(RecurrenceImage *r, int i, float eps)
{
    for (int j = 0; j < r->n; j++) {
        float d = fabsf(r->samples[i] - r->samples[j]);
        r->R[i * r->n + j] = (d < eps) ? 1u : 0u;
    }
}

/* ===================================================================== */
/* §7  state — typed wrappers around the two cycled selections           */
/* ===================================================================== */

/*
 * PresetState — typed wrapper around "which signal preset is loaded".
 *
 * INTENT
 *   The bare Preset enum is just an int — wrapping it in a struct
 *   plus cycle helpers makes the key-binding table read as a list
 *   of intentions ("cycle to next preset") rather than modular
 *   arithmetic.  This is the "Replace Primitive with Object"
 *   refactoring pattern.  Critically, it makes a "next theme"
 *   key physically unable to cycle the presets[] table.
 *
 * CONTEXT
 *   Owned by Scene; mutated only via preset_state_init,
 *   preset_state_cycle_next, preset_state_cycle_prev.  Read via
 *   preset_state_active (returns const RPPreset *).  The 'n'/'N'
 *   and 'p'/'P' keys are the only mutation paths at runtime.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS).  Cycle helpers keep this invariant
 *             with modular arithmetic on N_PRESETS = 30.
 *
 * REFERENCES
 *   Fowler, M. — "Refactoring", Replace Primitive with Object.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const RPPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState, distinct type so a "next theme" key
 *   physically cannot cycle the presets[] table.  Bundles the active
 *   index plus the helpers that re-apply the theme to ncurses pairs.
 *
 * CONTEXT
 *   Owned by Scene; mutated via palette_state_cycle_next/prev (t/T
 *   keys) followed by scene_apply_theme (re-runs theme_apply with
 *   the new index).
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§1).  Must be in
 *             [0, N_THEMES) — cycle helpers maintain this.
 *
 * REFERENCES
 *   Fowler, M. — "Refactoring", Replace Primitive with Object.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)       { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * SignalState — all the mutable bookkeeping any of the 30 signal
 * generators might need.
 *
 * INTENT
 *   Most generators touch ONE field of this struct; chaos generators
 *   (Lorenz, Hénon, logistic) touch their respective state field.
 *   Bundling them ALL in one struct lets signal_step be a single
 *   dispatcher with no per-kind allocation, no tagged union, no
 *   ownership transfer.  When a new preset is loaded we just reset
 *   the whole struct (scene_reset) and the unused fields cost
 *   nothing — a handful of floats.
 *
 * CONTEXT
 *   Owned by Scene; read+written exclusively by signal_step (§8);
 *   reset on every scene_reset() so the same preset reproduces the
 *   same recurrence plot (xorshift seed is fixed at 0xCAFEBABE).
 *
 * MEMBER LOGIC
 *   t_phase       : phase accumulator for periodic signals (sine,
 *                   square, triangle, sawtooth, beat).  Advances by
 *                   the per-preset frequency p1 each step.  Units:
 *                   radians; wraps naturally in sinf/fmodf.
 *   lorenz        : 3-D state of the Lorenz ODE [4]; sampled by
 *                   SIG_LORENZ_X.  Seeded to (1, 1, 1).
 *   henon_x/y     : 2-D state of the Hénon map [5]; SIG_HENON_X
 *                   reads henon_x.  Seeded near (0.1, 0).
 *   walk_acc      : accumulator for SIG_RANDOM_WALK and SIG_BROWNIAN.
 *   ar1_prev      : previous value for SIG_AR1 (Box & Jenkins [7]).
 *   logistic_prev : previous value for SIG_LOGISTIC [6] and SIG_TENT.
 *                   Must be in (0, 1) — seeded to 0.5.
 *   rng_state     : xorshift PRNG state [9] for every stochastic
 *                   signal.  Fixed seed 0xCAFEBABE → reproducibility.
 *   sample_index  : monotonically increasing per scene_reset; used by
 *                   SIG_BURST (gates on/off every burst_period) and
 *                   SIG_STEP_JUMPS (probabilistic jumps).
 *   step_level    : current value held by SIG_STEP_JUMPS between
 *                   jumps; advances when the RNG fires a "jump".
 *
 * REFERENCES
 *   [4] Lorenz, [5] Hénon, [6] May, [7] Box & Jenkins, [9] Marsaglia.
 */
typedef struct {
    float    t_phase;
    Vec3     lorenz;
    float    henon_x, henon_y;
    float    walk_acc;
    float    ar1_prev;
    float    logistic_prev;
    uint32_t rng_state;
    int      sample_index;
    float    step_level;
} SignalState;

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundle every piece of mutable state into ONE struct so the App
 *   layer owns one Scene and the main loop drives it through a tiny
 *   named-method API (scene_init, scene_reset, scene_tick).  Each
 *   sub-field is a named concept, kept in its own type so the
 *   compiler enforces the boundary between "what generates samples"
 *   (signal), "what stores them" (R), and "what the user picked"
 *   (preset / palette).
 *
 * CONTEXT
 *   Owned by App as a value (not pointer); accessed by the main loop
 *   helpers in §10 and the HUD writers in §9.  All mutation goes
 *   through named scene_* helpers so the call sites stay declarative.
 *
 * MEMBER LOGIC
 *   R         : §6 RecurrenceImage — samples[] + matrix R[i,j].
 *               Built up row-by-row over R.n ticks.
 *   collected : how many samples have been pushed so far.  When this
 *               hits R.n, the picture is complete and scene_tick goes
 *               idle.  Equal to R.rows_done by construction.
 *   signal    : §8 SignalState — generator bookkeeping (phase, ODE
 *               state, RNG state, etc).
 *   preset    : §7 PresetState — index into presets[] (signal kind
 *               + parameters + epsilon).
 *   palette   : §7 PaletteState — index into themes[].
 *   paused    : when true, scene_tick early-returns; the picture
 *               freezes mid-build.
 *
 * REFERENCES
 *   Sussman & Wisdom, "Structure and Interpretation of Classical
 *   Mechanics" (2014) — the System/State/Composite split this
 *   layer mirrors at the simulation-root level.
 */
typedef struct {
    RecurrenceImage R;
    int             collected;
    SignalState     signal;
    PresetState     preset;
    PaletteState    palette;
    bool            paused;
} Scene;

/* scene_active_preset / scene_active_eps — convenience accessors so
 * call sites don't repeat "presets[s->preset.current]". */
static inline const RPPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static inline float scene_active_eps(const Scene *s)
{
    return scene_active_preset(s)->epsilon;
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* rng_xorshift32 — Marsaglia 32-bit xorshift [9].  Three XOR-shift
 * stages; same seed → same sequence, so a reset gives the same
 * noisy signal each time and the recurrence pattern is reproducible. */
static inline uint32_t rng_xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* rng_uniform_signed — uniform random in [-1, 1]. */
static inline float rng_uniform_signed(uint32_t *s)
{
    return (float)((int32_t)rng_xorshift32(s)) / (float)INT32_MAX;
}

/* signal_step — return the next sample value for the active generator.
 *
 * DRIVER PSEUDOCODE
 *   st.sample_index += 1
 *   switch p.kind:
 *       CONSTANT             → return p.p1
 *       LINEAR               → return p.p1 · sample_index
 *       SINE/SQUARE/TRI/SAW  → v = wave(t_phase);  t_phase += p.p1;  return v
 *       TWO_SINES/QUASI/BEAT → v = sin(t) + α·sin(β·t);  advance t_phase
 *       DAMPED/BURST/WANE    → v = envelope(sample_index) · sin(t_phase)
 *       NOISE                → return xorshift_signed(rng_state)
 *       RANDOM_WALK/BROWNIAN → walk_acc += scale · noise;  return walk_acc
 *       AR1                  → ar1_prev = α·ar1_prev + (1-α)·noise
 *       LOGISTIC             → logistic_prev = r·x·(1-x);  return it
 *       TENT                 → logistic_prev = μ · min(x, 1-x);  return it
 *       LORENZ_X             → 5 × lorenz_rk4(dt);  return lorenz.x
 *       HENON_X              → (x', y') = (1 - a·x² + y, b·x);  return x'
 *       RAMP_SINE            → return p.p2·sample_index + sin(t_phase)
 *       STEP_JUMPS           → with prob p.p1: step_level = new_random
 *                              return step_level
 *
 * Each branch is a few lines so the whole function reads as the
 * complete catalogue of generators side-by-side.  Most generators
 * mutate exactly ONE field of SignalState; chaos generators (Lorenz,
 * Hénon, logistic) mutate their respective state fields. */
static float signal_step(SignalState *st, const RPPreset *p)
{
    st->sample_index++;

    switch (p->kind) {
    case SIG_CONSTANT:
        return p->p1;

    case SIG_LINEAR:
        /* x = a · t, a = p1 */
        return p->p1 * (float)st->sample_index;

    case SIG_SINE: {
        float v = sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_SQUARE: {
        float v = (sinf(st->t_phase) >= 0.0f) ? 1.0f : -1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_TRIANGLE: {
        /* normalised triangle: phase ∈ [0, 2π), value ∈ [-1, 1] */
        float frac = fmodf(st->t_phase, 2.0f * (float)M_PI) / (2.0f * (float)M_PI);
        float v = 4.0f * fabsf(frac - 0.5f) - 1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_SAWTOOTH: {
        float frac = fmodf(st->t_phase, 2.0f * (float)M_PI) / (2.0f * (float)M_PI);
        float v = 2.0f * frac - 1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_TWO_SINES: {
        float v = sinf(st->t_phase) + 0.5f * sinf(p->p2 / p->p1 * st->t_phase);
        st->t_phase += p->p1;
        return 0.6f * v;
    }

    case SIG_QUASI_GOLDEN: {
        const float golden = 1.6180339887f;
        float v = 0.5f * (sinf(st->t_phase) + sinf(golden * st->t_phase));
        st->t_phase += p->p1;
        return v;
    }

    case SIG_BEAT: {
        /* Two close frequencies → audible beating pattern */
        float v = 0.5f * (sinf(st->t_phase) + sinf(p->p2 / p->p1 * st->t_phase));
        st->t_phase += p->p1;
        return v;
    }

    case SIG_DAMPED_SINE: {
        float envelope = expf(-p->p2 * (float)st->sample_index * 0.05f);
        float v = envelope * sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_BURST: {
        /* On for burst_period samples, off for burst_period samples */
        int burst_period = (int)p->p2;
        if (burst_period < 1) burst_period = 20;
        int phase = (st->sample_index / burst_period) & 1;
        float v = (phase == 0) ? sinf(st->t_phase) : 0.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_WANE: {
        /* Amplitude slowly shrinks toward zero */
        float amp = 1.0f - p->p2 * (float)st->sample_index;
        if (amp < 0.0f) amp = 0.0f;
        float v = amp * sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_NOISE:
        return rng_uniform_signed(&st->rng_state);

    case SIG_RANDOM_WALK: {
        float step = 0.1f * rng_uniform_signed(&st->rng_state);
        st->walk_acc += step;
        return st->walk_acc;
    }

    case SIG_AR1: {
        /* AR(1) — Box & Jenkins [7].  x[t+1] = α·x[t] + (1-α)·noise */
        float alpha = p->p1;
        float noise = rng_uniform_signed(&st->rng_state);
        st->ar1_prev = alpha * st->ar1_prev + (1.0f - alpha) * noise;
        return st->ar1_prev;
    }

    case SIG_BROWNIAN: {
        float step = 0.3f * rng_uniform_signed(&st->rng_state);
        st->walk_acc += step;
        return st->walk_acc;
    }

    case SIG_LOGISTIC: {
        /* Logistic map — May 1976 [6].  x[t+1] = r · x[t] · (1 - x[t]) */
        float r = p->p1;
        st->logistic_prev = r * st->logistic_prev * (1.0f - st->logistic_prev);
        return st->logistic_prev;
    }

    case SIG_TENT: {
        /* Tent map x[t+1] = μ · min(x, 1 - x) */
        float mu = p->p1;
        float x  = st->logistic_prev;
        st->logistic_prev = mu * ((x < 0.5f) ? x : (1.0f - x));
        return st->logistic_prev;
    }

    case SIG_LORENZ_X: {
        /* Lorenz 1963 [4]: advance the ODE 5 RK4 steps, then sample x */
        for (int k = 0; k < 5; k++) lorenz_rk4(&st->lorenz, LORENZ_DT);
        return st->lorenz.x;
    }

    case SIG_HENON_X: {
        /* Hénon 1976 [5]: x[t+1] = 1 - a·x² + y;  y[t+1] = b·x */
        float x = st->henon_x, y = st->henon_y;
        st->henon_x = 1.0f - HENON_A * x * x + y;
        st->henon_y = HENON_B * x;
        return st->henon_x;
    }

    case SIG_RAMP_SINE: {
        float v = p->p2 * (float)st->sample_index + sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_STEP_JUMPS: {
        /* Piecewise constant: with probability p1 per step, jump to
         * a new random level; otherwise hold the previous level. */
        float roll = (float)rng_xorshift32(&st->rng_state) / (float)UINT32_MAX;
        if (roll < p->p1)
            st->step_level = 2.0f * (float)rng_xorshift32(&st->rng_state)
                           / (float)UINT32_MAX - 1.0f;
        return st->step_level;
    }
    }
    return 0.0f;   /* unreachable; switch covers all kinds */
}

static void scene_collect_one(Scene *s)
{
    if (s->collected >= s->R.n) return;
    s->R.samples[s->collected] = signal_step(&s->signal, scene_active_preset(s));
    s->collected++;
}

/* scene_reset — re-seed Scene so the current preset rebuilds from t=0.
 *
 * DRIVER PSEUDOCODE
 *   n = clamp(side, _, N_MAX)
 *   recurrence_reset(R, n)                      // clear samples + matrix
 *   collected = 0
 *   reset every SignalState field to its canonical seed:
 *       t_phase, walk_acc, ar1_prev, sample_index, step_level → 0
 *       lorenz                                  → (1, 1, 1)
 *       henon_x, henon_y                        → (0.1, 0)
 *       logistic_prev                           → 0.5
 *       rng_state                               → 0xCAFEBABE  (fixed!)
 *
 * The fixed RNG seed is the reproducibility contract: the same preset
 * always paints the same recurrence picture. */
static void scene_reset(Scene *s, int side)
{
    int n = side; if (n > N_MAX) n = N_MAX;
    recurrence_reset(&s->R, n);
    s->collected = 0;

    /* Re-seed all signal-state fields with the same default each time
     * so the same preset produces the SAME recurrence plot on every r. */
    s->signal.t_phase       = 0.0f;
    s->signal.lorenz        = (Vec3){ 1.0f, 1.0f, 1.0f };
    s->signal.henon_x       = 0.1f;
    s->signal.henon_y       = 0.0f;
    s->signal.walk_acc      = 0.0f;
    s->signal.ar1_prev      = 0.0f;
    s->signal.logistic_prev = 0.5f;
    s->signal.rng_state     = 0xCAFEBABEu;
    s->signal.sample_index  = 0;
    s->signal.step_level    = 0.0f;
}

static void scene_init(Scene *s, int side)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_SINE_MED);    /* simple, clear */
    palette_state_init(&s->palette, 0);
    scene_reset(s, side);
}

/* scene_tick — advance the simulation by one fixed-timestep tick.
 *
 * DRIVER PSEUDOCODE
 *   if paused: return                                    // freeze picture
 *   if collected == R.n: return                          // picture done
 *   eps = active_preset.epsilon
 *   x_new = signal_step(signal, active_preset)           // §8
 *   R.samples[collected] = x_new ;  collected += 1       // §6
 *   // fill the just-added row (row i contains R[i, 0..i])
 *   recurrence_compute_row(R, i = collected-1, eps)      // §6
 *   // back-fill column i in all earlier rows (symmetry)
 *   for j in 0 .. i-1:
 *       R[j, i] = ( |samples[j] - samples[i]| < eps )
 *
 * Result: every existing entry of R is current with respect to the
 * samples collected so far.  recurrence_paint can render at any time. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;                                /* per-tick generators ignore wall-clock dt */
    if (s->paused) return;
    if (s->collected >= s->R.n) return;      /* picture complete — go idle */

    float eps = scene_active_eps(s);
    scene_collect_one(s);
    int i = s->collected - 1;

    recurrence_compute_row(&s->R, i, eps);

    /* Back-fill column i in every prior row: row j was finalised when
     * sample j+1 was the newest one, so its slot at column i has been
     * sitting at 0 ever since.  Filling it now keeps R symmetric. */
    for (int j = 0; j < i; j++) {
        float d = fabsf(s->R.samples[j] - s->R.samples[i]);
        s->R.R[j * s->R.n + i] = (d < eps) ? 1u : 0u;
    }
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal viewport dimensions cache.
 *
 * INTENT
 *   ncurses' COLS / LINES are global macros; caching them in a
 *   struct gives the layout code one explicit handle to read and
 *   means a resize touches a single named state.  Every draw
 *   function takes Screen* (or its width/height) explicitly so the
 *   layout math is decoupled from ncurses globals.
 *
 * CONTEXT
 *   Owned by App.  Initialised by screen_init; resized by
 *   screen_resize when SIGWINCH fires; consulted everywhere a draw
 *   needs to know the viewport.
 *
 * MEMBER LOGIC
 *   cols : current terminal width  in cells.
 *   rows : current terminal height in cells.
 */
typedef struct { int cols, rows; } Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* recurrence_paint — render the N×N R matrix to ncurses cells.
 *
 * DRIVER PSEUDOCODE
 *   n = R.n
 *   (gx0, gy0) = centre of viewport, clamped within HUD band
 *   for i in 0 .. n-1:
 *       sy = gy0 + i;  skip if outside drawable band
 *       for j in 0 .. n-1:
 *           sx = gx0 + j;  skip if outside cols
 *           if i == j:      paint '\' in PAIR_DIAG  (always-on anchor)
 *           else if R[i,j]: paint '#' in PAIR_RECUR (one recurrence)
 *           else:           leave cell blank        (no recurrence)
 *
 * Texture classes (Marwan et al. [2] §3) should be visually
 * recognisable in the resulting image:
 *   periodicity  → parallel diagonals
 *   chaos        → short broken diagonals + speckle
 *   noise        → uniform speckle
 *   drift/trend  → faded corners, bright central band
 */
static void recurrence_paint(const RecurrenceImage *r, int cols, int rows)
{
    int n = r->n;
    int gx0 = (cols - n) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - n) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;

    for (int i = 0; i < n; i++) {
        int sy = gy0 + i;
        if (sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) continue;
        for (int j = 0; j < n; j++) {
            int sx = gx0 + j;
            if (sx < 0 || sx >= cols) continue;
            if (i == j) {
                attron(COLOR_PAIR(PAIR_DIAG) | A_BOLD);
                mvaddch(sy, sx, '\\');
                attroff(COLOR_PAIR(PAIR_DIAG) | A_BOLD);
            } else if (r->R[i * n + j]) {
                attron(COLOR_PAIR(PAIR_RECUR) | A_BOLD);
                mvaddch(sy, sx, '#');
                attroff(COLOR_PAIR(PAIR_RECUR) | A_BOLD);
            }
        }
    }
}

static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " RECURRENCE PLOT ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    const RPPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  ε:%.2f  %d/%d ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)active->epsilon,
             s->collected, s->R.n);
    int hx = cols - (int)strlen(buf); if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static void hud_param(const Scene *s)
{
    int x = HUD_LEFT_MARGIN;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", scene_active_preset(s)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 19;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " R(i,j)=1 if |x_i-x_j|<ε    matrix:%dx%d ", s->R.n, s->R.n);
    attroff(COLOR_PAIR(PAIR_HUD));
}
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
/* screen_draw — paint one full frame.
 *
 * DRIVER PSEUDOCODE
 *   erase()                                          // clear back buffer
 *   recurrence_paint(R, cols, rows)                  // the matrix itself
 *   hud_top(cols, fps, sim_fps, scene)               // top status line
 *   hud_param(scene)                                 // preset parameters
 *   hud_hint(rows)                                   // bottom key hints
 *   // caller (app_present_frame) does the doupdate */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{ erase(); recurrence_paint(&s->R, sc->cols, sc->rows);
  hud_top(sc->cols, fps, sim_fps, s); hud_param(s); hud_hint(sc->rows); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   The "everything else" container.  Owns the simulation (Scene),
 *   the viewport (Screen), the simulation-rate knob (sim_fps), the
 *   chosen matrix side, and the two volatile signal-handler flags.
 *   Lives as a single file-scope g_app so the signal handlers can
 *   touch it without indirection.
 *
 * CONTEXT
 *   Created by app_bootstrap; mutated by app_handle_key (and the
 *   named app_* mutators it calls); torn down by screen_free +
 *   atexit(cleanup) → endwin.  main() never reaches into the
 *   sub-fields directly — every access goes through a named helper.
 *
 * MEMBER LOGIC
 *   scene       : the entire simulation state (§8 Scene).
 *   screen      : cached terminal dimensions (§9 Screen).
 *   sim_fps     : fixed-timestep rate the scene_tick is driven at;
 *                 clamped to [SIM_FPS_MIN, SIM_FPS_MAX] = [10, 240].
 *   side        : current matrix side N (= min(cols, rows-2), N_MAX).
 *                 Recomputed on every resize.
 *   running     : main-loop guard.  Cleared by SIGINT/TERM via the
 *                 on_exit_signal handler, or by 'q' / ESC.
 *   need_resize : SIGWINCH flag; consumed by app_handle_pending_resize.
 *                 Both flags MUST be volatile sig_atomic_t (signal-
 *                 handler write + main-loop read).
 *
 * REFERENCES
 *   POSIX.1-2008 §2.4.3 — signal-safe writes via sig_atomic_t.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   side;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* main_install_signal_handlers — wire SIGINT/TERM → quit, SIGWINCH → resize. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_pick_side(App *app)
{
    int w = app->screen.cols;
    int h = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    int side = (w < h) ? w : h;
    if (side > N_MAX) side = N_MAX;
    if (side < 16)    side = 16;
    app->side = side;
}

/* app_bootstrap — first-time initialisation. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_side(app);
    scene_init(&app->scene, app->side);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_side(app);
    scene_reset(&app->scene, app->side);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * SIM_MAX_FRAME_DT_MS. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler [10] accumulator pattern.  Runs
 * scene_tick() at the chosen sim_fps regardless of the render rate. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — refresh fps every FPS_UPDATE_MS ms. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep so render runs at 60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint + flip back buffer. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Clamped sim-rate mutators. */
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

/* One-line mutators — named so the binding table reads as intentions. */
static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_picture     (App *app) { scene_reset(&app->scene, app->side); }
static void app_cycle_theme_next  (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev  (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->side);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->side);
}

static bool app_handle_key(App *app, int ch);

/* app_poll_keyboard — non-blocking getch + dispatch.  false = quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* app_handle_key — key-binding table; each case calls ONE named mutator. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_picture    (app); break;
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
 * FrameClock — wall-clock + accumulator state for the main loop.
 *
 * INTENT
 *   Bundle the 5 timing locals (frame_time, sim_accum, fps_accum,
 *   frame_count, fps_display) into ONE named concept so main()
 *   reads as a pseudocode driver: "init clock, advance clock, drain
 *   simulation".  Threading them through helpers as a single
 *   FrameClock* removes 5 separate parameter lists.
 *
 * CONTEXT
 *   Lives as a local in main(); not owned by App because it is pure
 *   loop scaffolding (a different driver would replace it whole).
 *   Mutated by frame_clock_init, frame_clock_advance,
 *   frame_clock_reset_after_resize, app_drain_fixed_timestep,
 *   app_update_fps_meter.
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock ns at the START of the previous frame.
 *                 app_compute_frame_dt subtracts this from now() and
 *                 stamps it forward.
 *   sim_accum   : ns of unspent simulation time.  Drained in
 *                 multiples of TICK_NS(sim_fps) by the inner loop.
 *   fps_accum   : ns since the last fps recalculation.
 *   frame_count : frames rendered since the last fps recalculation.
 *   fps_display : the displayed fps value (refreshed every
 *                 FPS_UPDATE_MS = 500 ms so the HUD doesn't flicker).
 *
 * REFERENCES
 *   [10] Fiedler — "Fix Your Timestep!".  The classic accumulator
 *        pattern this struct implements.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* main — the whole simulation as a pseudocode driver.
 *
 * DRIVER PSEUDOCODE
 *   install signal handlers (SIGINT/TERM/WINCH)
 *   app_bootstrap()                       // ncurses + colour + Scene
 *   FrameClock clk = frame_clock_init()
 *   while running:
 *       if need_resize: rebuild Screen + Scene;  clk reset
 *       dt = compute_frame_dt(&clk.frame_time)   // capped at 100 ms
 *       frame_clock_advance(clk, dt)
 *       drain_fixed_timestep(scene, &clk.sim_accum)   // scene_tick × N
 *       update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display)
 *       throttle_to_render_target(clk.frame_time, dt)  // sleep to 60 fps
 *       present_frame(app, clk.fps_display)            // erase+draw+flip
 *       if !poll_keyboard(app): running = 0            // q/ESC → quit
 *   screen_free()                                      // endwin via atexit
 *
 * Body deliberately reads top-to-bottom as the OVERALL PSEUDOCODE
 * block in the file header — every named call is the corresponding
 * line in that block. */
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
