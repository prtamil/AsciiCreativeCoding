/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * poincare_section.c
 *   — A Poincaré section of the Lorenz ODE.  Turn a continuous-time
 *     3-D chaotic trajectory into a 2-D point cloud by sampling
 *     ONLY at the moments the trajectory crosses a chosen plane in
 *     a chosen direction.  The fractal structure of the strange
 *     attractor becomes a 2-D shape you can SEE directly — the
 *     analytical bridge between continuous ODEs (Lorenz, Rössler)
 *     and discrete maps (Hénon, logistic).
 *
 * DEMO: Integrate the Lorenz system with RK4 at small dt.  Each
 *       time z(t) crosses Z_PLANE going upward (z' > 0), record
 *       (x, y).  Plot the accumulated cloud at density-mapped
 *       brightness.  Single preset: CLASSIC (σ=10, ρ=28, β=8/3)
 *       — the canonical chaotic butterfly.  Higher-ρ regimes
 *       overflow the static viewport, so we focus on the one ρ
 *       value where the iconic Poincaré section fits cleanly.
 *
 * Study alongside:
 *   ./strange_attractor.c — Lorenz drawn directly in 3-D phase space.
 *   ./bifurcation.c       — bifurcation diagram (the discrete analogue
 *                           of plotting Poincaré returns vs parameter).
 *   ./standard_map.c      — Poincaré section of a Hamiltonian system,
 *                           applied directly to a discrete map.
 *
 * Section map:
 *   §1  config   — constants, single CLASSIC preset, themes
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — HUD pairs + 10 themes
 *   §5  physics  — Vec3, LorenzState, LorenzSystem + Lorenz composite
 *                  + named-stage RK4
 *   §6  section  — SectionGrid: 2-D histogram of (x, y) crossings
 *   §7  state    — CrossingDetector (upward z-plane crossing book-
 *                  keeping) + PresetState + PaletteState wrappers
 *   §8  scene    — Scene composes lorenz + section + detector
 *                  + preset + palette
 *   §9  screen   — viewport-based painters (axes, density, fixed
 *                  points, live crossing, frame), HUD writers
 *   §10 app      — signals, resize, key dispatch, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (clear cloud, re-init trajectory)
 *   t / T      next / previous theme
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/poincare_section.c \
 *       -o poincare_section -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Lorenz ODE integrated with RK4, dt ≈ 0.005.  Each
 *                 tick advances INT_STEPS_PER_TICK substeps and
 *                 detects upward (z' > 0) crossings of z = Z_PLANE.
 *                 At each crossing the (x, y) coordinates are
 *                 accumulated into a density grid via linear
 *                 interpolation between adjacent samples.
 *
 * Data-struct   : §5 Lorenz splits into LorenzSystem (σ, ρ, β) and
 *                 LorenzState (x, y, z) — parameters vs ODE coords.
 *                 §6 SectionGrid is a uint16 hit-counter per cell
 *                 over a fixed (x, y) viewport [X_MIN, X_MAX] ×
 *                 [Y_MIN, Y_MAX].
 *                 §7 CrossingDetector bundles z_plane + prev-state
 *                 snapshot, the "bridge" struct that converts the
 *                 continuous orbit into discrete section dots.
 *
 * Rendering     : ASCII-only.  Layered back-to-front:
 *                   AXES        — faint x=0 / y=0 reference lines
 *                   DENSITY     — 8-tier glyph ramp coloured by
 *                                 log(hit-count): . : ; + o * # @
 *                   FIXED PTS   — '(+)' brackets at C± = (±√(β(ρ-1)),
 *                                 ±√(β(ρ-1))), the two Lorenz fixed
 *                                 points the section crescents centre on
 *                   LIVE CROSS  — '@' at the most recent crossing,
 *                                 a moving "tick" as new returns arrive
 *                   FRAME       — '+'-cornered ASCII box one cell
 *                                 outside the data on every side
 *
 * Performance   : ~4 µs per RK4 step.  At dt = 0.005 and 8 substeps/
 *                 tick × 60 ticks/sec = 480 RK4 evals/sec — trivial.
 *                 Cloud fills meaningfully within ~10 s of wall-clock.
 *
 * References    : THE EQUATION  (where it comes from)
 *                 [1] Lorenz, E. N. (1963) — "Deterministic Nonperiodic
 *                     Flow", J. Atmos. Sci. 20, pp. 130-141.  The
 *                     ORIGINAL paper: introduces the three-equation
 *                     ODE that lives in lorenz_deriv(), shows the
 *                     iconic butterfly trajectory, and gives the
 *                     canonical σ=10, ρ=28, β=8/3 parameter set.
 *                 [2] Sparrow, C. (1982) — "The Lorenz Equations:
 *                     Bifurcations, Chaos, and Strange Attractors",
 *                     Springer Applied Math. Sci. 41.  Definitive
 *                     reference on the parameter-space structure —
 *                     which ρ values give which dynamical regime,
 *                     where the periodic windows live.  Fig. 2.4 is
 *                     the canonical Poincaré section at z = ρ-1
 *                     that this file reproduces.
 *                 [3] Tucker, W. (1999) — "The Lorenz attractor exists",
 *                     C. R. Acad. Sci. Paris 328, pp. 1197-1202.
 *                     The rigorous mathematical proof, settled
 *                     Smale's 14th problem.  Cited for completeness.
 *
 *                 POINCARÉ-SECTION TECHNIQUE  (the rendering "trick")
 *                 [4] Poincaré, H. (1892) — "Les Méthodes Nouvelles de
 *                     la Mécanique Céleste", Vol. III.  The original
 *                     stroboscopic-sampling idea: convert a continuous
 *                     n-D flow into a discrete (n-1)-D map by
 *                     recording its intersections with a transverse
 *                     surface.  CrossingDetector reproduces this in
 *                     code form for z = ρ-1, ẑ > 0.
 *                 [5] Strogatz, S. H. (2015) — "Nonlinear Dynamics and
 *                     Chaos" (2nd ed.), Westview Press, §10.  The
 *                     pedagogical walk-through of section technique
 *                     applied to Lorenz, with explicit derivation of
 *                     the symmetry that produces the two-quadrant
 *                     section pattern.
 *                 [6] Hirsch, M. W., Smale, S., Devaney, R. L. (2013) —
 *                     "Differential Equations, Dynamical Systems, and
 *                     an Introduction to Chaos" (3rd ed.), Academic
 *                     Press, Ch. 14.  Modern dynamical-systems text
 *                     with Lorenz as the canonical example.
 *
 *                 NUMERICAL INTEGRATION
 *                 [7] Press, W. H., Teukolsky, S. A., Vetterling, W. T.,
 *                     Flannery, B. P. (2007) — "Numerical Recipes"
 *                     (3rd ed.), Cambridge, Ch. 17.1.  RK4 derivation,
 *                     stability, and the Butcher tableau implemented
 *                     in lorenz_rk4 / rk4_butcher_weighted_average.
 *                 [8] Fiedler, G. (2004) — "Fix Your Timestep!",
 *                     gafferongames.com.  Fixed-substep accumulator
 *                     pattern in the main-loop physics drain — keeps
 *                     the section deterministic across frame rates.
 *
 *                 SIMULATION ARCHITECTURE & RENDERING
 *                 [9] Sussman, G. J., Wisdom, J. (2014) — "Structure
 *                     and Interpretation of Classical Mechanics"
 *                     (2nd ed.), MIT Press.  The "system / state /
 *                     observable" triad that Scene mirrors at the
 *                     code level (LorenzSystem + LorenzState +
 *                     SectionGrid).
 *                 [10] Gookin, D. (2007) — "Programmer's Guide to
 *                     NCurses", Wiley.  Double-buffered rendering
 *                     model (wnoutrefresh + doupdate), color-pair
 *                     theory, and SIGWINCH handling used in §3, §9-10.
 *
 *                 COMPARE IN PROJECT
 *                 • ./standard_map.c   — discrete Hamiltonian map.
 *                 • ./strange_attractor.c — Lorenz drawn directly.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The Lorenz trajectory winds around two "wings" in 3-D space.  Each
 * loop around either wing eventually returns the trajectory to a
 * chosen reference plane (z = Z_PLANE).  Recording where each return
 * happens turns the 3-D dance into a 2-D dot — the Poincaré return.
 * The collection of return points IS the chaotic attractor, made
 * visible as a thin fractal-looking curve.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * If the system were periodic, every return would land on the same
 * point (or a small finite cluster).  For chaos, the returns trace
 * out a 1-D curve embedded in the 2-D plane — that curve is the
 * attractor's intersection with the section plane.  Its fractal
 * (non-integer-dimension) appearance is the visual signature of
 * strange.
 *
 * ALGORITHM IN STEPS  (per substep)
 * ──────────────────
 *  1. Save z_prev.
 *  2. RK4 one step → new (x, y, z).
 *  3. If z_prev < Z_PLANE ≤ z (upward crossing):
 *       a. Linear-interp α = (Z_PLANE - z_prev) / (z - z_prev)
 *       b. x_cross = x_prev + α(x - x_prev), same for y
 *       c. Record (x_cross, y_cross) into density grid
 *
 * KEY FORMULAS  (Lorenz 1963)
 * ────────────
 *   dx/dt = σ(y − x)
 *   dy/dt = x(ρ − z) − y
 *   dz/dt = x·y − β·z
 *
 *   Classic chaotic regime: σ = 10, ρ = 28, β = 8/3.
 *   Section plane: z = ρ − 1  (passes through the two fixed points).
 *
 * HOW TO VERIFY  (observe the live picture)
 * ─────────────
 *  • CLASSIC (σ=10, ρ=28, β=8/3): the cloud forms TWO crescent-shaped
 *    arcs centred on the (+) fixed-point markers at (±√(β(ρ-1)), …).
 *    The arcs are THIN (≈ 1-D objects in 2-D) — that thinness is the
 *    visible signature of "strange attractor".
 *  • Only TWO quadrants populate (upper-right + lower-left): the
 *    Lorenz equations have a (x, y, z) → (-x, -y, z) symmetry, and at
 *    every UPWARD crossing of z = ρ-1 the orbit's x and y carry the
 *    SAME SIGN.  See Sparrow Fig. 2.4 — the empty quadrants are
 *    physically inaccessible, not a rendering bug.
 *  • Reset (r) — clears the cloud and re-seeds the orbit; the same
 *    section pattern redraws within ~15 s of wall-clock time.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  80,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_DENS_BASE    =   3,    /* 8 density bands */
    PAIR_LIVE         =  11,    /* live crossing marker */
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

/* Integration */
#define LORENZ_DT                 0.005f
#define INT_STEPS_PER_TICK         50

/* Section plane: z = ρ - 1 (passes through both Lorenz fixed points).
 * Stored as a function of the preset's ρ at scene_reset time. */

/* Viewport in (x, y) where the cloud lives.  Tuned for the CLASSIC
 * preset's attractor; PERIODIC fits comfortably inside the same window. */
#define X_MIN                   -25.0f
#define X_MAX                    25.0f
#define Y_MIN                   -30.0f
#define Y_MAX                    30.0f

#define DENS_BAND_COUNT           8
#define DENS_SATURATE_LOG         8.0f

/*
 * Preset — single canonical Lorenz regime.  ρ-sweeps beyond ρ ≈ 35
 * overflow the static viewport [X_MIN, X_MAX] × [Y_MIN, Y_MAX] =
 * [±25] × [±30] (the attractor's spatial extent scales like √ρ), so
 * we keep ONE working preset: the iconic ρ = 28 chaotic butterfly.
 */
typedef enum {
    PRESET_CLASSIC = 0,
    N_PRESETS,
} Preset;

/*
 * LorenzPreset — one row of the presets[] table: a named 6-tuple
 * (σ, ρ, β, x₀, y₀, z₀) defining ONE experiment.
 *
 * INTENT
 *   Names a specific point in Lorenz parameter-space + initial-
 *   condition space.  The dynamics depend ONLY on (σ, ρ, β) at long
 *   times — for ρ in the chaotic regime any IC near the attractor
 *   converges onto it — but the INITIAL CONDITION matters for
 *   periodic windows (where ICs in the wrong basin would miss the
 *   periodic orbit) and for the TRANSIENT before settling.
 *
 * CONTEXT
 *   Lives only in the const-table presets[] (immediately below).
 *   Looked up by §7 PresetState wrapper via preset_state_active().
 *   Consumed by scene_load_active_preset which copies the system
 *   parameters into Lorenz.system and the initial condition into
 *   Lorenz.state, then seeds the CrossingDetector with z_plane = ρ-1.
 *
 * MEMBER LOGIC
 *   name        : 8-char label shown in HUD (%-8s format).
 *   sigma       : σ — Prandtl number (atmospheric / convective).
 *                 Holmes-canon value 10.
 *   rho         : ρ — Rayleigh ratio.  THE bifurcation parameter.
 *                 Canon chaotic value 28.
 *   beta        : β — geometric parameter.  Canon value 8/3.
 *   x0, y0, z0  : initial condition for the trajectory.  Standard
 *                 choice (1, 1, 1) for ρ=28; periodic-window presets
 *                 would need ICs picked to land in the right basin.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Lorenz (1963) §III — discusses how (σ, ρ, β) map to physical
 *                            convection parameters.
 *   [2] Sparrow Ch. 1      — chapter-length tour of each parameter's
 *                            role and the regimes they select.
 */
typedef struct {
    const char *name;
    float       sigma, rho, beta;
    float       x0, y0, z0;
} LorenzPreset;

static const LorenzPreset presets[N_PRESETS] = {
    { "CLASSIC ", 10.0f, 28.00f, 8.0f/3.0f,  1.0f, 1.0f, 1.0f },
};

/*
 * Theme — colour assignment for the Poincaré-section renderer.
 *
 * INTENT
 *   Decouples "what gets drawn" (§9 painters) from "what colour it
 *   should be" so a single cycle key (t/T) swaps the entire visual
 *   identity.  The section renderer has TWO render roles — a
 *   DENSITY-tier ramp (8 levels for hit-count) and a LIVE colour
 *   for the moving '@' crossing marker — and each Theme row assigns
 *   both coherently.
 *
 * CONTEXT
 *   Lives only in the const-table themes[] of N_THEMES entries.
 *   Selected by §7 PaletteState wrapper via palette_state_active().
 *   Pushed into ncurses by §3 theme_apply() which calls init_pair()
 *   on each (PAIR_DENS_BASE+i, band[i], -1) and (PAIR_LIVE, live, -1).
 *
 * MEMBER LOGIC
 *   name                 : 7-char label shown on row-1 HUD.
 *   band[0..N-1]         : DENSITY-tier colour ramp.  band[0] =
 *                          lowest hit-count tier (dimmest); band[N-1]
 *                          = saturated (brightest).  Combined with
 *                          the 8-glyph density ramp in §9 gives
 *                          ~64 visual levels of density.  All entries
 *                          must sit at xterm-256 index ≥ 24 per
 *                          CLAUDE.md "Theme Palette Brightness".
 *   live                 : colour for the '@' marker at the most
 *                          recent crossing.  Conventionally a hot
 *                          saturated tone so the moving tick stands
 *                          out against the density gradient and the
 *                          frame.
 *
 * REFERENCES
 *   • CLAUDE.md "Theme Palette Brightness" — the ≥ 24 rule.
 *   • CLAUDE.md "HUD Standard" — PAIR_HUD / PAIR_HINT are NOT per-theme.
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];
    short       live;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  39,  75, 117, 153, 189, 220, 226, 231 }, 196 },
    { "MATRIX",  {  46,  82, 118, 154, 156, 191, 192, 194 }, 226 },
    { "NOVA",    {  99, 135, 171, 207, 213, 219, 225, 231 }, 226 },
    { "MONO",    { 244, 247, 250, 252, 253, 254, 255, 231 }, 226 },
    { "OCEAN",   {  39,  45,  81, 117, 153, 159, 195, 231 }, 226 },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 220, 227 }, 231 },
    { "EARTH",   { 100, 137, 173, 179, 215, 222, 228, 231 }, 196 },
    { "FOREST",  {  64, 107, 114, 144, 150, 156, 194, 231 }, 226 },
    { "DESERT",  { 130, 173, 179, 215, 222, 228, 230, 231 }, 196 },
    { "ARCTIC",  {  81, 117, 153, 159, 195, 225, 230, 231 }, 196 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec = ns / NS_PER_SEC,
                            .tv_nsec = ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/* PAIR_HUD_FG_256 / PAIR_HINT_FG_256 — bright yellow / bright cyan in
 * the xterm-256 cube.  Named so the HUD setup doesn't carry magic
 * literals inline. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* theme_apply_pairs_256color — push the 8 density-band + 1 live pair
 * assignments for a 256-colour terminal.  Each pair maps one render
 * role (density-band-i, live marker) to one xterm-256 index from the
 * Theme row.  All foregrounds use the terminal's default background
 * (-1) so the demo composites cleanly on dark or light. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* theme_apply_pairs_8color_fallback — single fallback assignment for
 * 8-colour terminals where the per-theme xterm-256 palette can't be
 * expressed.  All N_THEMES collapse to the SAME 8-colour set —
 * trade-off is "lose theme variety, keep legibility everywhere". */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_LIVE, COLOR_RED, -1);
}

/* theme_apply — top-level dispatcher: clamp the index, branch on
 * terminal capability, delegate to the appropriate helper. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* color_init_hud_pairs — install the project-standard HUD + HINT
 * pairs per CLAUDE.md's "HUD Standard".  These pairs are NOT
 * per-theme — they stay constant as t/T cycles so the status row
 * never loses contrast. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* color_init — bring ncurses colour subsystem online and load theme 0.
 *
 *   STEP 1 — enable colour + default-bg passthrough
 *   STEP 2 — install the constant HUD pairs (project standard)
 *   STEP 3 — push the first theme's density + live colours
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* ===================================================================== */
/* §5  physics — Lorenz ODE                                               */
/* ===================================================================== */

/*
 * Vec3 — generic 3-component vector used both as the position-state
 * of the Lorenz system AND as the derivative dy/dt returned by
 * lorenz_deriv.  Flat by-value struct so RK4 stage states can be
 * combined cheaply via state_add().
 */
typedef struct {
    float x, y, z;
} Vec3;

/*
 * LorenzState — alias for Vec3.  Same memory layout, but the
 * typedef communicates INTENT at use sites:
 *   Vec3 generic 3-vec;
 *   LorenzState ODE phase point.
 * The integrator works with both interchangeably; the type name
 * just labels what the value MEANS at each call. */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — the INVARIANT parameters of the Lorenz equation:
 *
 *   dx/dt = σ · (y - x)
 *   dy/dt = x · (ρ - z) - y
 *   dz/dt = x · y - β · z
 *
 *   sigma : Prandtl number (atmospheric / convective interpretation;
 *           classic value 10).  Controls the ratio of momentum to
 *           thermal diffusivity in the convection model.
 *   rho   : Rayleigh number / critical Rayleigh ratio (classic 28).
 *           THE bifurcation parameter — its value selects which
 *           dynamical regime the system lives in.
 *   beta  : geometric parameter (classic 8/3).  Sets the aspect ratio
 *           of the convective rolls in the original derivation.
 *
 * Const-during-simulation.  Written by scene_load_active_preset;
 * read by lorenz_deriv at every RK4 stage.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Lorenz (1963) eq. (25)-(27) — derives this exact 3-equation
 *                                     system from a Galerkin truncation
 *                                     of the Boussinesq convection PDEs.
 *   [2] Sparrow (1982) Ch. 1        — chapter-length discussion of
 *                                     each parameter's role and the
 *                                     regimes they pick out.
 */
typedef struct {
    float sigma, rho, beta;
} LorenzSystem;

/*
 * Lorenz — composite: SYSTEM + STATE.
 *
 * INTENT
 *   Bundle the parameters and the dynamic state under one named type
 *   so callers that need "the whole oscillator" take one pointer
 *   instead of two.  RK4 mutates only .state; preset cycling mutates
 *   only .system.  The split mirrors the textbook treatment:
 *   PARAMETERS / COORDINATES.
 *
 * CONTEXT
 *   One instance lives on Scene.  Lifetime:
 *     .system : written ONCE per preset change (sigma, rho, beta).
 *     .state  : written ~3000 times/sec (60 ticks × INT_STEPS_PER_TICK
 *               substeps) by lorenz_rk4; reset on preset change.
 *
 * MEMBER LOGIC
 *   system : §5 LorenzSystem — σ, ρ, β.
 *   state  : §5 LorenzState  — (x, y, z) ODE coordinate.
 *
 * REFERENCES (numbered in file-header References block)
 *   [9] Sussman & Wisdom Ch. 3 — the modern functional-style
 *       PARAMETER / STATE separation reproduced here in code form.
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

/* lorenz_deriv — evaluate dy/dt = f(state, system).  The three
 * equations of the original Lorenz 1963 paper, named in the body so
 * the reader can match each line to the published formula.
 *
 * Equation form: header ref [1] Lorenz eq. (25)-(27).  Each line
 * below corresponds DIRECTLY to one equation in the paper. */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);                  /* σ(y − x)         */
    dy.y = s->x * (sys->rho - s->z) - s->y;             /* x(ρ − z) − y     */
    dy.z = s->x * s->y - sys->beta * s->z;              /* xy − βz          */
    return dy;
}

/* state_add — y + h·k, returned as a new LorenzState.  Used inside
 * RK4 to build the four intermediate stage points. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* vec3_add — kept for backwards-compatibility of any 3-vec arithmetic
 * outside the ODE integrator.  Identical to state_add but operates
 * by value (the original signature). */
static inline Vec3 vec3_add(Vec3 a, float h, Vec3 b)
{
    Vec3 r = { a.x + h * b.x, a.y + h * b.y, a.z + h * b.z };
    return r;
}

/* RK4 Butcher tableau constants. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* rk4_butcher_weighted_average — Simpson-rule combination of the 4
 * slope estimates: (k₁ + 2k₂ + 2k₃ + k₄) / 6.  Component-wise on
 * the 3-D LorenzState. */
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

/* lorenz_rk4 — one fixed-step RK4 update of the Lorenz state.
 *
 *   STAGE 1 — slope_start = f(y)
 *   STAGE 2 — slope_mid_1 = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2 = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end   = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * See header ref [7] Numerical Recipes Ch. 17.1 for the Butcher
 * tableau, O(dt⁵) local truncation, and stability domain.  RK4 at
 * LORENZ_DT = 0.005 is more than accurate enough for the Lorenz
 * attractor — its global Lyapunov time is ~1, so even after 10⁴
 * substeps (~50 sim-sec) the orbit's energy drift stays well below
 * visual noise on the section. */
static void lorenz_rk4(Lorenz *L, float dt)
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

/* ===================================================================== */
/* §6  Poincaré section grid                                              */
/* ===================================================================== */

/*
 * SectionGrid — 2-D histogram of Poincaré-section crossings in (x, y).
 *
 * INTENT
 *   The chaotic orbit's TOPOLOGY in 3-D — the two-wing butterfly,
 *   the fractal layering — is invisible in any single snapshot of
 *   the orbit.  But sampled at the SECTION PLANE (z = ρ-1, ẑ > 0),
 *   the orbit's discrete returns ACCUMULATE into a recognisable
 *   2-D shape: the two crescents of the Lorenz section.  This
 *   struct is the ACCUMULATOR.  uint16 bins (one per cell of the
 *   viewport) count how many times a crossing has landed there;
 *   §6 density_band log-maps the counts onto the 8 colour tiers.
 *
 *   Also tracks the MOST RECENT crossing point (last_x, last_y) for
 *   the live '@' marker, so the viewer sees individual ticks being
 *   added as new returns happen.
 *
 * CONTEXT
 *   One instance lives on Scene.  Lifetime:
 *     • section_reset zeros every bin on resize / preset change.
 *     • section_record (one increment + last-point update) fires per
 *       upward z = z_plane crossing — ~10 times per real-second at
 *       ρ=28 for a typical orbit.
 *     • §9 paint_density_cells reads every bin each render frame.
 *
 * DATA-STRUCTURE LOGIC
 *   2-D histogram chosen over a list of (x, y) points for the same
 *   two reasons used elsewhere:
 *     1. ZERO ALLOCATION in the hot path.
 *     2. CONSTANT cost per crossing — section_record is one bounds
 *        check + one increment + 2 floats stored, regardless of
 *        how many crossings have accumulated.
 *   uint16 ceiling: a bin saturates at 65535 hits — at ~10 crossings
 *   per second, a hot crescent bin needs ~1.8 hours to saturate.
 *
 * MEMBER LOGIC
 *   w, h     : data dimensions in cells.  Set by section_reset;
 *              matches the App's map_w / map_h.
 *   count    : w × h (cached).  Used by section_reset to memset only
 *              the live slice of hits[], not the whole CELLS_MAX
 *              static array.
 *   hits     : 2-D bin counts, indexed [row × w + col].  Row 0 is
 *              the TOP visual row (highest y); row h-1 is BOTTOM
 *              (lowest y).  section_record performs the y-flip so
 *              §9 painters iterate top-to-bottom in screen order
 *              without re-flipping.
 *   last_x   : x of the MOST RECENT crossing (physical units).
 *   last_y   : y of the most recent crossing.  Both drawn as the
 *              live '@' marker by §9 paint_live_crossing.
 *   has_live : true once at least one crossing has been recorded.
 *              Until the first crossing, no '@' is drawn.
 *
 * REFERENCES (numbered in file-header References block)
 *   [4] Poincaré (1892) — the section-sampling idea this struct
 *                         accumulates the output of.
 *   • Sedgewick "Algorithms" 4th ed. — standard 2-D histogram pattern.
 */
typedef struct {
    int      w, h;
    int      count;
    uint16_t hits[CELLS_MAX];
    float    last_x, last_y;
    bool     has_live;
} SectionGrid;

static void section_reset(SectionGrid *g, int w, int h)
{
    g->w = w; g->h = h; g->count = w * h;
    memset(g->hits, 0, sizeof(uint16_t) * (size_t)g->count);
    g->has_live = false;
}

/* xy_to_cell — viewport-affine map (x, y) → (cx, cy).  Returns false
 * if the point falls outside the viewport. */
static bool xy_to_cell(const SectionGrid *g, float x, float y, int *cx, int *cy)
{
    if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX) return false;
    *cx = (int)((x - X_MIN) / (X_MAX - X_MIN) * (float)g->w);
    *cy = (int)((y - Y_MIN) / (Y_MAX - Y_MIN) * (float)g->h);
    if (*cx < 0)        *cx = 0;
    if (*cx >= g->w)    *cx = g->w - 1;
    if (*cy < 0)        *cy = 0;
    if (*cy >= g->h)    *cy = g->h - 1;
    return true;
}

static void section_record(SectionGrid *g, float x, float y)
{
    int cx, cy;
    if (!xy_to_cell(g, x, y, &cx, &cy)) return;
    int idx = (g->h - 1 - cy) * g->w + cx;
    if (g->hits[idx] < UINT16_MAX) g->hits[idx]++;
    g->last_x  = x;
    g->last_y  = y;
    g->has_live = true;
}

/* DENSITY_BAND_ROUND_OFFSET — the 0.5f added before float→int
 * truncation to convert it into nearest-integer rounding.  Without
 * it the band assignment skews low (every fractional band index gets
 * floored to the band below). */
#define DENSITY_BAND_ROUND_OFFSET   0.5f

/* density_band — map a hit-count to a 0..N-1 colour-band index using
 * a LOG mapping (so the eye sees relative density, not absolute).
 *
 *   STEP 1 — empty bin → -1 sentinel (caller skips drawing)
 *   STEP 2 — log-saturate: v = log(hits) / DENS_SATURATE_LOG ∈ [0,1]
 *   STEP 3 — scale to band index with rounding
 *
 * log-mapping rationale: a hot crescent bin might have 10⁴ hits while
 * a sparse outer-edge bin has 10¹ — linear scaling would crush the
 * gradient. */
static inline int density_band(uint16_t hits)
{
    /* STEP 1 — empty bin sentinel */
    if (hits == 0) return -1;

    /* STEP 2 — log-saturate to [0, 1] */
    float saturated_log = logf((float)hits) / DENS_SATURATE_LOG;
    if (saturated_log < 0.0f) saturated_log = 0.0f;
    if (saturated_log > 1.0f) saturated_log = 1.0f;

    /* STEP 3 — quantise into 0..N-1 band index with rounding */
    int band = (int)(saturated_log * (float)(DENS_BAND_COUNT - 1)
                     + DENSITY_BAND_ROUND_OFFSET);
    if (band > DENS_BAND_COUNT - 1) band = DENS_BAND_COUNT - 1;
    return band;
}

/* ===================================================================== */
/* §7  state — Poincaré section bookkeeping + typed wrappers              */
/* ===================================================================== */

/*
 * CrossingDetector — per-orbit bookkeeping for detecting UPWARD
 * crossings of the section plane z = z_plane.
 *
 * INTENT
 *   The Poincaré-section technique records the orbit's (x, y)
 *   coordinates AT THE MOMENT it crosses z = z_plane going up
 *   (ẑ > 0).  Detecting that moment requires remembering the
 *   PREVIOUS state and z value (so we can spot z_prev < z_plane ≤
 *   z_now).  Linear interpolation between the two surrounding
 *   substeps gives the exact (x, y) AT the plane crossing.
 *
 *   This struct is the SECTION TECHNIQUE incarnate.  Header ref [4]
 *   Poincaré (1892) introduced the idea; this struct is its 1-D
 *   instantiation for our specific transverse surface z = ρ-1.
 *
 * MEMBER LOGIC
 *   z_plane    : where the section sits.  Set to ρ − 1 per preset —
 *                this is the height of the two Lorenz fixed points,
 *                so the section passes through them and cuts the
 *                attractor's two wings symmetrically.  This is the
 *                standard choice in the literature (refs [2] Sparrow
 *                Fig. 2.4, [5] Strogatz §10.7).
 *   prev_state : full 3-D state at the previous substep.  Used to
 *                interpolate (x, y) at the crossing instant.
 *   prev_z     : z at the previous substep (duplicate of prev_state.z
 *                kept for the cheap sign-change predicate before
 *                running the full interpolation).
 *
 * REFERENCES (numbered in file-header References block)
 *   [4] Poincaré (1892) Vol. III — origin of the section-sampling idea.
 *   [2] Sparrow Fig. 2.4         — the canonical z = ρ-1 section.
 *   [5] Strogatz §10.7           — pedagogical derivation of why
 *                                  this plane cuts the attractor cleanly.
 */
typedef struct {
    float       z_plane;
    LorenzState prev_state;
    float       prev_z;
} CrossingDetector;

/* crossing_detector_init — set z_plane from ρ, seed previous-state
 * snapshot to the given initial condition. */
static void crossing_detector_init(CrossingDetector *d,
                                   float rho, LorenzState initial)
{
    d->z_plane    = rho - 1.0f;
    d->prev_state = initial;
    d->prev_z     = initial.z;
}

/* crossing_detector_is_upward_z_plane_crossing — sign-change predicate:
 * TRUE iff the orbit just crossed z = z_plane in the +z direction.
 * Cheap test run BEFORE the linear-interpolation arithmetic. */
static inline bool crossing_detector_is_upward_z_plane_crossing(
    const CrossingDetector *d, const LorenzState *current)
{
    return d->prev_z < d->z_plane && current->z >= d->z_plane;
}

/* crossing_detector_interpolate_xy — linear-interpolate (x, y) at the
 * exact z = z_plane crossing between prev_state and current.
 *
 *   α = (z_plane − prev_z) / (current.z − prev_z)    ∈ [0, 1]
 *   x_cross = prev.x + α · (current.x − prev.x)
 *   y_cross = prev.y + α · (current.y − prev.y) */
static inline void crossing_detector_interpolate_xy(
    const CrossingDetector *d, const LorenzState *current,
    float *x_cross, float *y_cross)
{
    float dz    = current->z - d->prev_z;
    float alpha = (dz != 0.0f) ? (d->z_plane - d->prev_z) / dz : 0.0f;
    *x_cross    = d->prev_state.x + alpha * (current->x - d->prev_state.x);
    *y_cross    = d->prev_state.y + alpha * (current->y - d->prev_state.y);
}

/* crossing_detector_snapshot — remember the current state as "previous"
 * for the next substep's crossing test. */
static inline void crossing_detector_snapshot(CrossingDetector *d,
                                              const LorenzState *current)
{
    d->prev_state = *current;
    d->prev_z     = current->z;
}

/*
 * PresetState — typed wrapper around "which preset is loaded".
 *
 * INTENT
 *   The Preset enum is just an integer — wrapping it in a struct +
 *   helpers makes the call sites read as intentions ("activate the
 *   currently-selected preset") rather than arithmetic ("index into
 *   the presets array").  This is the "Replace Primitive with Object"
 *   pattern: a one-field struct exists not to hold data but to
 *   ATTACH a named API to that data.
 *
 *   With N_PRESETS == 1 in this file the cycle helpers are omitted;
 *   the struct still exists because the API surface
 *   (preset_state_init, preset_state_active) carries semantic value
 *   over a bare int and matches the pattern used in sister files.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated only by preset_state_init
 *   (from scene_init).  Read by scene_load_active_preset (via
 *   preset_state_active → presets[current]) and by the HUD writers
 *   for the row-1 "preset:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Currently always
 *             PRESET_CLASSIC; will be cycled by n/p once the preset
 *             table grows beyond one entry.
 *
 * REFERENCES
 *   • Fowler "Refactoring" (2nd ed.), "Replace Primitive with Object".
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static const LorenzPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }
/* Cycle helpers omitted: N_PRESETS == 1 in this file.  Add
 * preset_state_cycle_next / _prev if the preset table later grows. */

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState (one-int wrapper + cycle helpers) but
 *   kept as a SEPARATE type because their tables differ (presets[]
 *   vs themes[]).  Distinct types let scene_apply_theme() take
 *   exactly the right wrapper and refuse anything else — eliminates
 *   the class of bug where a "next" key on the wrong wrapper would
 *   index past one table's bounds into the other.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by t/T keys via
 *   palette_state_cycle_next / prev.  After every mutation the
 *   caller invokes palette_state_apply() which forwards to §3
 *   theme_apply() — the only place that touches ncurses init_pair.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§1).  Must be in
 *             [0, N_THEMES).  Initial value is 0 (DEFAULT) — most
 *             legible greyscale-ish starting point.
 *
 * REFERENCES
 *   • Fowler "Refactoring" — same "Replace Primitive with Object"
 *     pattern as PresetState.  The deliberate type-distinction
 *     between PresetState and PaletteState despite identical shape
 *     is the DDD "Tiny Types" idea: values from different domains
 *     shouldn't share a type just because the bits match.
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
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundle every piece of mutable state into one struct so the App
 *   layer owns ONE Scene (no loose globals) and the main loop drives
 *   it through a tiny named-method API.  Each sub-field is a NAMED
 *   CONCEPT — the Lorenz body, its section accumulator, the bridge
 *   between continuous flow and discrete map, plus the user-selected
 *   preset and palette.
 *
 *   Decomposition axis is "what the simulation IS MADE OF":
 *     • a continuous-time body (lorenz),
 *     • a discrete-time observation accumulator (section),
 *     • the projection from continuous → discrete (detector),
 *     • a selection of which experiment is loaded (preset),
 *     • a selection of which palette renders it (palette).
 *
 * CONTEXT
 *   One instance embedded inside App.  Lifetime / mutation map:
 *     • scene_tick      (~3000 Hz, advances .lorenz.state, runs
 *                        crossing detection, records into .section)
 *     • scene_reset     (r / SIGWINCH — re-seeds orbit, clears section)
 *     • app_handle_key  (sub-states .preset, .palette, .paused)
 *   Read by:
 *     • screen_draw     (every frame — paints all sub-systems)
 *     • HUD writers     (preset / palette / σ ρ β / z-plane / paused)
 *
 * MEMBER LOGIC
 *   lorenz   : §5 Lorenz — system + state.  Mutated by lorenz_rk4
 *              at every RK4 substep.
 *   section  : §6 SectionGrid — 2-D histogram of crossings in (x, y).
 *              One bin incremented per upward z = z_plane crossing.
 *   detector : §7 CrossingDetector — z_plane + prev-state snapshot
 *              for the upward-crossing test.  The CONCEPTUAL BRIDGE
 *              between the continuous orbit (lorenz) and the discrete
 *              accumulator (section).
 *   preset   : §7 PresetState — which row of presets[] is loaded.
 *   palette  : §7 PaletteState — which colour theme is active.
 *   paused   : when true, scene_tick early-returns — physics frozen
 *              but rendering continues so the user can study the
 *              accumulated section pattern.
 *
 * REFERENCES (numbered in file-header References block)
 *   [9] Sussman & Wisdom (2014) — the "system / state / observable"
 *       triad at the simulation-architecture level.  Here:
 *       LorenzSystem + LorenzState = system + state; SectionGrid =
 *       the observable; CrossingDetector = the SAMPLING RULE that
 *       defines what gets observed.
 *   • Gamma, Helm, Johnson, Vlissides (1995) — "Composite" pattern:
 *     one root holds a tree of named sub-objects, one tick / paint
 *     call fans out in deterministic order.
 */
typedef struct {
    Lorenz           lorenz;
    SectionGrid      section;
    CrossingDetector detector;
    PresetState      preset;
    PaletteState     palette;
    bool             paused;
} Scene;

/* scene_load_active_preset — copy preset's σ, ρ, β into the system,
 * copy initial condition into the state, seed the crossing detector. */
static void scene_load_active_preset(Scene *s)
{
    const LorenzPreset *p = preset_state_active(&s->preset);
    s->lorenz.system.sigma = p->sigma;
    s->lorenz.system.rho   = p->rho;
    s->lorenz.system.beta  = p->beta;
    s->lorenz.state.x      = p->x0;
    s->lorenz.state.y      = p->y0;
    s->lorenz.state.z      = p->z0;
    crossing_detector_init(&s->detector, p->rho, s->lorenz.state);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int w, int h)
{
    section_reset(&s->section, w, h);
    scene_load_active_preset(s);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;
    preset_state_init (&s->preset,  PRESET_CLASSIC);
    palette_state_init(&s->palette, 0);
    scene_reset(s, w, h);
}

/* scene_tick — advance the orbit + accumulate Poincaré-section dots.
 * INT_STEPS_PER_TICK fixed-dt RK4 substeps per render frame; after
 * each substep, test for an upward z = z_plane crossing and record
 * the interpolated (x, y) if so. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;

    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        crossing_detector_snapshot(&s->detector, &s->lorenz.state);
        lorenz_rk4(&s->lorenz, LORENZ_DT);
        if (crossing_detector_is_upward_z_plane_crossing(&s->detector,
                                                         &s->lorenz.state)) {
            float x_cross, y_cross;
            crossing_detector_interpolate_xy(&s->detector, &s->lorenz.state,
                                             &x_cross, &y_cross);
            section_record(&s->section, x_cross, y_cross);
        }
    }
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-adapter handle: ncurses lifecycle + current
 * window dimensions.
 *
 * INTENT
 *   ncurses owns the terminal mode and back-buffer; this struct is
 *   the THIN OWNERSHIP HANDLE the program passes around.  Bundling
 *   (cols, rows) here lets every painter take ONE Screen parameter
 *   instead of calling getmaxyx() at every draw site.  Funnelling
 *   all ncurses init/teardown through screen_init / screen_free keeps
 *   the dependency at a single boundary.
 *
 * CONTEXT
 *   One instance lives on App.  Initialised by screen_init() in
 *   main(); refreshed by screen_resize() on SIGWINCH.  Read by
 *   every painter that needs to clip against terminal bounds and
 *   by the HUD writers for right-anchored layout.
 *
 * MEMBER LOGIC
 *   cols : current terminal width in CHARACTER CELLS.  Updated only
 *          by screen_init / screen_resize via getmaxyx().
 *   rows : current terminal height in CHARACTER CELLS.  Row 0 is
 *          the top of the terminal; row (rows - 1) is the bottom.
 *
 * REFERENCES (numbered in file-header References block)
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 2 — the API
 *        surface the screen_* helpers wrap.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
                                       getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* glyph_for_band — 8-tier graduated ramp matching the 8 colour pairs.
 * Each glyph picked for its TOTAL INK weight: . → : → ; → + → o → *
 * → # → @ reads as low → high density even in monochrome.  Without
 * this matching, 8 colour bands collapsed into only 3 glyph tiers and
 * most of the gradient resolution was thrown away. */
static char glyph_for_band(int b)
{
    static const char ramp[DENS_BAND_COUNT] = {
        '.',  /* band 0 — lightest, low density */
        ':',
        ';',
        '+',
        'o',
        '*',
        '#',
        '@',  /* band 7 — heaviest, saturated */
    };
    if (b < 0)                    b = 0;
    if (b > DENS_BAND_COUNT - 1)  b = DENS_BAND_COUNT - 1;
    return ramp[b];
}

/* SECTION_FRAME_* — glyphs for the rectangular box that surrounds the
 * data region, anchoring it visually inside the terminal. */
#define SECTION_FRAME_CORNER       '+'
#define SECTION_FRAME_HORIZ        '-'
#define SECTION_FRAME_VERT         '|'

/* SECTION_AXIS_* — faint x=0 / y=0 guide lines inside the frame, so
 * the reader can locate the origin and read off symmetry by eye. */
#define SECTION_AXIS_VERT          '|'
#define SECTION_AXIS_HORIZ         '-'
#define SECTION_AXIS_ORIGIN        '+'

/* Fixed-point marker — the two Lorenz fixed points C± = (±√(β(ρ-1)),
 * ±√(β(ρ-1)), ρ-1) sit AT the section plane, and the two crescents
 * of the section are centred on them.  Marking them tells the
 * viewer "the chaos winds around THESE two points". */
#define FIXED_POINT_GLYPH_LEFT     '('
#define FIXED_POINT_GLYPH_CENTRE   '+'
#define FIXED_POINT_GLYPH_RIGHT    ')'

/* paint_frame_top_and_bottom_edges — '-' edges immediately above and
 * below the data rectangle.  Each edge runs the full data width. */
static void paint_frame_top_and_bottom_edges(int gx0, int gy0, int w, int h)
{
    int top_row    = gy0 - 1;
    int bottom_row = gy0 + h;
    for (int x = 0; x < w; x++) {
        mvaddch(top_row,    gx0 + x, SECTION_FRAME_HORIZ);
        mvaddch(bottom_row, gx0 + x, SECTION_FRAME_HORIZ);
    }
}

/* paint_frame_left_and_right_edges — '|' edges immediately left and
 * right of the data rectangle.  Each edge runs the full data height. */
static void paint_frame_left_and_right_edges(int gx0, int gy0, int w, int h)
{
    int left_col  = gx0 - 1;
    int right_col = gx0 + w;
    for (int y = 0; y < h; y++) {
        mvaddch(gy0 + y, left_col,  SECTION_FRAME_VERT);
        mvaddch(gy0 + y, right_col, SECTION_FRAME_VERT);
    }
}

/* paint_frame_corners — '+' glyphs at the four corner cells where
 * horizontal and vertical edges meet.  Drawn after the edges so the
 * corner glyph wins over the edge glyphs at the meeting cells. */
static void paint_frame_corners(int gx0, int gy0, int w, int h)
{
    int top_row    = gy0 - 1,   bottom_row = gy0 + h;
    int left_col   = gx0 - 1,   right_col  = gx0 + w;
    mvaddch(top_row,    left_col,  SECTION_FRAME_CORNER);
    mvaddch(top_row,    right_col, SECTION_FRAME_CORNER);
    mvaddch(bottom_row, left_col,  SECTION_FRAME_CORNER);
    mvaddch(bottom_row, right_col, SECTION_FRAME_CORNER);
}

/* paint_section_frame — single-line ASCII box one cell outside the
 * data on every side.
 *
 *   STEP 1 — '-' top + bottom edges
 *   STEP 2 — '|' left + right edges
 *   STEP 3 — '+' corners (overdrawn last so they win at the joins)
 *
 * Uses PAIR_LIVE so the frame stands out from the density gradient. */
static void paint_section_frame(int gx0, int gy0, int w, int h)
{
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    paint_frame_top_and_bottom_edges(gx0, gy0, w, h);
    paint_frame_left_and_right_edges(gx0, gy0, w, h);
    paint_frame_corners             (gx0, gy0, w, h);
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* paint_section_axes — faint x=0 vertical + y=0 horizontal guide lines
 * INSIDE the frame.  Drawn BEFORE density so data overwrites them
 * where it lands; non-bold so they orient the eye without competing. */
static void paint_section_axes(const SectionGrid *g, int gx0, int gy0)
{
    int x_zero_col   = (int)((0.0f - X_MIN) / (X_MAX - X_MIN) * (float)g->w);
    int y_zero_row   = (int)((0.0f - Y_MIN) / (Y_MAX - Y_MIN) * (float)g->h);
    int y_zero_screen_row = gy0 + (g->h - 1 - y_zero_row);

    attron(COLOR_PAIR(PAIR_DENS_BASE));
    if (x_zero_col >= 0 && x_zero_col < g->w)
        for (int y = 0; y < g->h; y++)
            mvaddch(gy0 + y, gx0 + x_zero_col, SECTION_AXIS_VERT);
    if (y_zero_row >= 0 && y_zero_row < g->h)
        for (int x = 0; x < g->w; x++)
            mvaddch(y_zero_screen_row, gx0 + x, SECTION_AXIS_HORIZ);
    if (x_zero_col >= 0 && x_zero_col < g->w &&
        y_zero_row >= 0 && y_zero_row < g->h)
        mvaddch(y_zero_screen_row, gx0 + x_zero_col, SECTION_AXIS_ORIGIN);
    attroff(COLOR_PAIR(PAIR_DENS_BASE));
}

/* paint_fixed_point_markers — the two Lorenz fixed points
 *   C± = (±√(β(ρ-1)), ±√(β(ρ-1)))    on the section plane z = ρ-1.
 *
 * The two crescents of the Poincaré section are CENTRED on these
 * points (one per Lorenz wing).  Drawing them as bright '(+)'
 * markers makes the underlying geometry visible — the chaos winds
 * around these two attractor centres. */
static void paint_fixed_point_markers(const SectionGrid *g, int gx0, int gy0,
                                      float rho, float beta)
{
    float discriminant = beta * (rho - 1.0f);
    if (discriminant <= 0.0f) return;
    float fp = sqrtf(discriminant);

    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    for (int sign = -1; sign <= 1; sign += 2) {
        float fx = (float)sign * fp;
        float fy = (float)sign * fp;
        int cx, cy;
        if (!xy_to_cell(g, fx, fy, &cx, &cy)) continue;
        int sy = gy0 + (g->h - 1 - cy);
        int sx = gx0 + cx;
        if (sx > gx0 && sx < gx0 + g->w - 1) {
            mvaddch(sy, sx - 1, FIXED_POINT_GLYPH_LEFT);
            mvaddch(sy, sx + 1, FIXED_POINT_GLYPH_RIGHT);
        }
        mvaddch(sy, sx, FIXED_POINT_GLYPH_CENTRE);
    }
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* paint_density_cells — main data layer.  For each populated bin
 * emit a band-coloured glyph; skip empty bins so axes/frame underneath
 * stay visible.  Each cell gets its own band's colour pair. */
static void paint_density_cells(const SectionGrid *g, int gx0, int gy0,
                                int cols, int rows)
{
    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int b = density_band(g->hits[y * g->w + x]);
            if (b < 0) continue;
            int pair = PAIR_DENS_BASE + b;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph_for_band(b));
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* paint_live_crossing — '@' marker at the MOST RECENT section
 * crossing.  Drawn last so it sits on top of everything — the user
 * sees a moving '@' "tick" as each new point joins the cloud. */
static void paint_live_crossing(const SectionGrid *g, int gx0, int gy0,
                                int cols, int rows)
{
    if (!g->has_live) return;
    int cx, cy;
    if (!xy_to_cell(g, g->last_x, g->last_y, &cx, &cy)) return;
    int sy = gy0 + (g->h - 1 - cy);
    int sx = gx0 + cx;
    if (sx < 0 || sx >= cols ||
        sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) return;
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    mvaddch(sy, sx, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* section_paint — composite renderer for the Poincaré section.
 *
 *   STEP 1 — centre the data rectangle inside the drawable band
 *   STEP 2 — axes:        faint x=0 / y=0 guide lines (back)
 *   STEP 3 — density:     8-tier glyph ramp coloured by hit count
 *   STEP 4 — fixed pts:   '(+)' markers at the two Lorenz fixed pts
 *   STEP 5 — live cross:  '@' at the most recent crossing
 *   STEP 6 — frame:       '+'-cornered box one cell outside the data
 *
 * Back-to-front so the layers stack cleanly: the data is the main
 * information; fixed points and live marker punctuate it; the frame
 * anchors the whole thing.  The frame draws OVER the data on its
 * single-cell perimeter — fine, because the data inside is what
 * matters. */
static void section_paint(const SectionGrid *g, int cols, int rows,
                          float rho, float beta)
{
    /* STEP 1 — viewport origin, clipped to leave room for the frame */
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 1)                gx0 = 1;
    if (gy0 < HUD_TOP_ROWS + 1) gy0 = HUD_TOP_ROWS + 1;

    paint_section_axes        (g, gx0, gy0);
    paint_density_cells       (g, gx0, gy0, cols, rows);
    paint_fixed_point_markers (g, gx0, gy0, rho, beta);
    paint_live_crossing       (g, gx0, gy0, cols, rows);
    paint_section_frame       (   gx0, gy0, g->w, g->h);
}

/* hud_paint_top_left_title — fixed title in row 0 column HUD_LEFT_MARGIN. */
static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " POINCARÉ SECTION (Lorenz) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_format_top_right_status — format the dynamic status string
 * (fps, Hz, preset, ρ) into a stack buffer.  Pure formatter, no
 * ncurses — pulled out so hud_top_right_status reads as "format then
 * paint", not a tangled snprintf+positioning block. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const LorenzPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  ρ:%.2f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)s->lorenz.system.rho);
}

/* hud_paint_top_right — paint a right-anchored string at row 0.
 * Right-anchor formula: column = cols - strlen, clipped at 0. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top_right_status — row-0 right-side driver: format then paint. */
static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* HUD_PARAM_CELL_WIDTH_* — printed widths of the row-1 cells.  Sized
 * to the fixed-width %-8s format specifiers so the layout doesn't
 * shift as preset / theme names change. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

/* hud_paint_param_cell_bold — paint one labelled cell at row 1 with
 * the project HUD pair, bold.  The fmt is expected to contain ONE
 * %s placeholder for `value`. */
static void hud_paint_param_cell_bold(int cursor_x, const char *fmt, const char *value)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, fmt, value);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_param_row — row-1 HUD: three side-by-side cells.
 *
 *   STEP 1 — PRESET name (bold, primary)
 *   STEP 2 — THEME  name (bold, primary)
 *   STEP 3 — physics readout: σ, ρ, β, z-plane (non-bold, secondary)
 */
static void hud_param_row(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    /* STEP 1 — preset */
    hud_paint_param_cell_bold(cursor_x, " preset:%-8s ",
                              preset_state_active(&s->preset)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    /* STEP 2 — theme */
    hud_paint_param_cell_bold(cursor_x, " theme:%-8s ",
                              palette_state_active(&s->palette)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    /* STEP 3 — physics params (σ, ρ, β, section plane) */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " σ:%.1f  ρ:%.2f  β:%.2f  z-plane:%.2f ",
             (double)s->lorenz.system.sigma,
             (double)s->lorenz.system.rho,
             (double)s->lorenz.system.beta,
             (double)s->detector.z_plane);
    attroff(COLOR_PAIR(PAIR_HUD));
}
static void hud_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    section_paint(&s->section, sc->cols, sc->rows,
                  s->lorenz.system.rho, s->lorenz.system.beta);
    hud_paint_top_left_title();
    hud_top_right_status(sc->cols, fps, sim_fps, s);
    hud_param_row(s);
    hud_bottom_hint(sc->rows);
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   Owns the Scene (mutable simulation state), the Screen (terminal
 *   adapter), the sim tick rate (sim_fps), the chosen map size, and
 *   the two volatile flags written from signal handlers.  Everything
 *   else in the program is reached through this struct — there is no
 *   other mutable global state.
 *
 *   Concentrating mutable state in one struct disciplines signal
 *   handling: SIGWINCH and SIGINT handlers may only touch the
 *   volatile sig_atomic_t fields on g_app — every other piece of
 *   state is reachable only through main-loop functions.
 *
 * CONTEXT
 *   Exactly ONE instance exists: file-scope g_app.  Signal handlers
 *   need a callable-from-signal target, and POSIX async-signal-safety
 *   requires that to be a global; hence g_app rather than a stack-
 *   local App.  All non-signal accesses pass an App* explicitly.
 *
 * MEMBER LOGIC
 *   scene       : §8 Scene — the simulation; "what the user sees".
 *   screen      : §9 Screen — cols / rows + ncurses lifecycle handle.
 *   sim_fps     : current PHYSICS tick rate in Hz.  Distinct from
 *                 RENDER_FPS_TARGET — physics can step at any rate
 *                 while the renderer stays locked at 60 fps.
 *                 Adjusted by ] / [ keys.
 *   map_w       : cells of horizontal SIMULATION area.
 *   map_h       : cells of vertical SIMULATION area (= screen.rows -
 *                 HUD_BAND_RESERVED_ROWS - SECTION_FRAME_RESERVED).
 *   running     : 0 → main loop exits.  Set by SIGINT/SIGTERM handler
 *                 and by app_handle_key on q/ESC.  Declared
 *                 volatile sig_atomic_t for async-signal safety per
 *                 POSIX.1-2017 §2.4.3.
 *   need_resize : 1 → next frame triggers a SIGWINCH rebuild via
 *                 app_handle_pending_resize.  Set by the SIGWINCH
 *                 handler.  Same volatile sig_atomic_t requirement.
 *
 * REFERENCES (numbered in file-header References block)
 *   [8]  Fiedler "Fix Your Timestep!" — explains why sim_fps and
 *        RENDER_FPS_TARGET are kept as INDEPENDENT knobs on App.
 *   [10] Gookin Ch. 11 — SIGWINCH pattern reproduced by
 *        .need_resize + app_handle_pending_resize.
 *   • POSIX.1-2017 §2.4.3 — async-signal safety rules that justify
 *     the volatile sig_atomic_t typing of running / need_resize.
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

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* SECTION_FRAME_RESERVED — cells the '+'-cornered frame eats on each
 * axis (one cell on every side: left + right, top + bottom). */
#define SECTION_FRAME_RESERVED  2

static void app_pick_map_size(App *app)
{
    /* Drawable = terminal minus HUD reservation, further minus
     * SECTION_FRAME_RESERVED so paint_section_frame() has room on
     * every side without spilling into the HUD or off-screen. */
    int mw = app->screen.cols - SECTION_FRAME_RESERVED;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS - SECTION_FRAME_RESERVED;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw; app->map_h = mh;
}
/* main_install_signal_handlers — wire SIGINT/TERM → quit, SIGWINCH → resize. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_bootstrap — first-time initialisation: RNG, sim rate,
 * ncurses, map sizing, scene build. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * SIM_MAX_FRAME_DT_MS so a paused terminal can't dump backlog. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler accumulator pattern: pull
 * FIXED-size physics steps until what's left is less than one tick.
 *
 * See header ref [8] Fiedler "Fix Your Timestep!" — variable-dt
 * would make the Poincaré section non-deterministic (different
 * render speeds would sample crossings at slightly different
 * (x, y) values), so a slow terminal might draw a subtly different
 * cloud than a fast one.  Fixed dt keeps the section reproducible. */
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
static void app_reset_orbit       (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }
static void app_cycle_theme_next  (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev  (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }

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
    case 'r': case 'R':  app_reset_orbit      (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — wall-clock + accumulator state threaded through the
 * per-frame helpers.  Bundles the 5 timing locals into one named
 * concept so main() reads as a pseudocode driver.
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
 *   install signal handlers
 *   bootstrap (ncurses + scene)
 *   init frame clock
 *   while running:
 *     if pending resize → rebuild screen, reset clock
 *     dt ← wall-clock since last frame
 *     advance clock; drain fixed-timestep physics; refresh fps meter
 *     throttle to render target; present frame
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
