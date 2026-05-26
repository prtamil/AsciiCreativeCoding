/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sensitive_dependence.c
 *   — The "butterfly effect" made literal.  Two Lorenz trajectories
 *     are launched from initial conditions that differ by ε ≈ 10⁻⁶.
 *     For the first ~Lyapunov-time seconds the two paths are
 *     indistinguishable.  Then they diverge EXPONENTIALLY until they
 *     occupy completely different lobes of the strange attractor.
 *
 * DEMO: Up to 3 Lorenz trajectories evolved with identical RK4,
 *       seeded ε apart in initial conditions.  10 presets walk
 *       three axes of variability — projection (which 2-D slice
 *       of the 3-D state to plot), perturbation size (ε ladder),
 *       and layout (overlay / split / log|δ| stacked / log|δ|
 *       full screen).
 *
 *       Presets (cycle with n/p):
 *         1  CLASSIC   — XZ butterfly, 2 trajectories, ε=1e-6 (default)
 *         2  TOP_DOWN  — XY view, 2 trajectories
 *         3  SIDE      — YZ view, 2 trajectories
 *         4  EPS_BIG   — XZ, ε=1e-3 (diverges in seconds)
 *         5  EPS_TINY  — XZ, ε=1e-12 (stays close for minutes)
 *         6  TRIO      — 3 trajectories (base, base±ε) fanning out
 *         7  SPLIT     — A on left half, B on right half, compare
 *         8  DELTA     — overlay (top 60%) + log|δ| plot (bottom 40%)
 *         9  LOG_ONLY  — only the log|δ| plot, full screen
 *        10  TRIO_LOG  — TRIO overlay + log|δ| plot
 *
 * Study alongside:
 *   ./strange_attractor.c   — same Lorenz, ten variations.
 *   ../fractals/lyapunov.c  — Lyapunov exponent as a 2-D parameter
 *                             portrait (related quantitative idea).
 *
 * Section map:
 *   §1  config   — constants, 10-preset table, themes, projection/layout enums
 *   §2  clock    — monotonic timer + sleep
 *   §3  color    — HUD pairs + 10 themes
 *   §5  physics  — Lorenz (system + state) composite + named-stage RK4
 *   §6  trail    — Trail ring buffer + LogPlot ring buffer
 *   §7  state    — PresetState + PaletteState typed wrappers
 *   §8  scene    — Scene composite (Lorenz × TRAJ_MAX + Trails + LogPlot
 *                  + PresetState + PaletteState + viewport)
 *   §9  screen   — projection / layout dispatch, paint_trail, paint_log_plot
 *   §10 app      — signals, resize, key dispatch table, FrameClock,
 *                  main pseudocode driver + named loop helpers
 *
 * Keys: q/ESC quit | space pause | r reset | n/p preset | t/T theme | ]/[ Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/sensitive_dependence.c \
 *       -o sensitive_dependence -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : 2-3 Lorenz integrations side-by-side, identical
 *                 RK4 at dt = LORENZ_DT.  Base initial condition is
 *                 (1.0, 1.0, 1.0); per-preset ε seeds trajectory B
 *                 at base + ε·x̂ and (TRIO) trajectory C at base − ε·x̂.
 *                 At each substep compute δ(t) = ‖A − B‖₃ and push
 *                 log δ(t) into the LogPlot ring for the DELTA /
 *                 LOG_ONLY / TRIO_LOG presets.
 *
 * Data-struct   : Lorenz composite × TRAJ_MAX (system + state per
 *                 trajectory) + one Trail of projected (u, v) samples
 *                 per trajectory (4 age bands).  A LogPlot ring of
 *                 pre-logf'd |δ| values for the plot presets.
 *
 * Rendering     : Trails in two distinct colour ramps so A and B
 *                 stay separable (TRIO adds a solid colour for C).
 *                 Layout is per-preset: FULL / SPLIT / PLOT_BOTTOM
 *                 (attractor top, log plot bottom 40%) / PLOT_ONLY.
 *                 Projection is per-preset (XZ butterfly / XY top-down
 *                 / YZ side).
 *
 * References    : Numbered so inline code can cite [n].
 *
 *   PHYSICS / DYNAMICAL SYSTEMS
 *   [1] Lorenz, E. N. (1963).  "Deterministic Nonperiodic Flow",
 *       J. Atmos. Sci. 20, pp. 130-141.  THE original paper: defines
 *       the three equations and the (σ, ρ, β) = (10, 28, 8/3)
 *       regime; first observation of sensitive dependence.
 *   [2] Lorenz, E. N. (1972).  "Predictability: Does the flap of
 *       a butterfly's wings in Brazil set off a tornado in Texas?"
 *       AAAS talk that named the phenomenon.  The 1979 Bull. AMS
 *       reprint is the citable form.
 *   [3] Strogatz, S. H. (2015).  *Nonlinear Dynamics and Chaos*
 *       (2nd ed.), §9 (Lorenz equations & Hopf bifurcation) and
 *       §10.2 (Lyapunov exponent of 1-D maps as a template for
 *       the higher-dimensional case).  Pedagogical companion text.
 *   [4] Sparrow, C. (1982).  *The Lorenz Equations: Bifurcations,
 *       Chaos, and Strange Attractors*.  Applied Math Sciences 41.
 *       Definitive monograph; the source of the canonical Lyapunov
 *       value λ ≈ 0.906 referenced in the HUD.
 *   [5] Eckmann, J.-P., Ruelle, D. (1985).  "Ergodic theory of
 *       chaos and strange attractors", Rev. Mod. Phys. 57(3),
 *       pp. 617-656.  Formal definition of sensitive dependence
 *       on initial conditions and Lyapunov spectrum.
 *
 *   ALGORITHMS
 *   [6] Wolf, A., Swift, J. B., Swinney, H. L., Vastano, J. A.
 *       (1985).  "Determining Lyapunov exponents from a time
 *       series", Physica D 16(3), pp. 285-317.  The standard
 *       algorithm for MEASURING λ from observed |δ(t)|; the
 *       slope of our log|δ| plot is exactly the Wolf et al.
 *       largest-exponent estimate in the unsaturated regime.
 *
 *   NUMERICS
 *   [7] Press, W. H. et al. (2007).  *Numerical Recipes* (3rd ed.),
 *       §17.1.  Classical 4-stage Runge-Kutta with the Butcher
 *       (1/6, 1/3, 1/3, 1/6) tableau used by lorenz_rk4_step.
 *
 *   FRAMEWORK
 *   [8] Fiedler, G. (2004).  "Fix Your Timestep!",
 *       gafferongames.com.  Accumulator pattern used by
 *       app_drain_fixed_timestep so all trajectories integrate
 *       at the same wall-clock-independent fixed step.
 *   [9] Sussman, G. J., Wisdom, J. (2014).  *Structure and
 *       Interpretation of Classical Mechanics* (2nd ed.), §1.6
 *       "Coordinate systems and states".  The System / State /
 *       Composite split used in §5 (LorenzSystem + LorenzState
 *       + Lorenz composite).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two systems start nearly identical.  Their states stay nearly
 * identical for a while.  Then their separation grows exponentially
 * — δ(t) ≈ δ₀ · e^(λ·t) — until the separation is comparable to the
 * size of the attractor itself.  After that they wander
 * independently on the SAME attractor but are uncorrelated.
 *
 * KEY FORMULAS  (Lorenz 1963 [1], Eckmann-Ruelle 1985 [5])
 * ────────────
 *   dx/dt = σ(y − x), dy/dt = x(ρ − z) − y, dz/dt = x·y − β·z
 *   (σ, ρ, β) = (10, 28, 8/3) — chaotic regime
 *
 *   δ(t) = ‖A(t) − B(t)‖
 *   log δ(t) ≈ log δ₀ + λ·t  (Lyapunov regime)
 *
 *   λ_Lorenz ≈ 0.906  (Sparrow 1982 [4]) ⇒ T_λ = 1/λ ≈ 1.1 s.
 *   Every T_λ seconds the separation grows by factor e ≈ 2.7.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • CLASSIC  : the two trails are visibly identical for ~7 seconds
 *               (~7 Lyapunov times of growth from 1e-6 → ~1e-3),
 *               then visibly separate; by ~10s they're on different
 *               lobes of the butterfly.
 *  • EPS_TINY : same shape but the indistinguishable phase lasts
 *               ~25-35 seconds — the headline butterfly-effect
 *               experience.  Time-to-diverge scales as -log(ε)/λ.
 *  • LOG_ONLY : the log|δ| curve is a near-straight line with slope
 *               ≈ LORENZ_LAMBDA (0.906) in the unsaturated regime,
 *               then plateaus when |δ| saturates near attractor size.
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
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_A_BASE      =   3,    /* 4 age bands for trail A */
    PAIR_B_BASE      =   7,    /* 4 age bands for trail B */
    PAIR_LIVE_A      =  11,
    PAIR_LIVE_B      =  12,
    PAIR_PLOT        =  13,    /* log|δ| curve */
    PAIR_AXIS        =  14,
    PAIR_LIVE_C      =  15,    /* 3rd trajectory (TRIO presets) */
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

#define LORENZ_DT                 0.01f
#define INT_STEPS_PER_TICK         5
#define TRAIL_MAX               2500
#define TRAIL_BAND_COUNT           4
#define PLOT_MAX                2000
#define TRAJ_MAX                   3    /* up to 3 trajectories (TRIO presets) */

/* Lorenz parameters (chaotic regime) */
#define LZ_SIGMA                 10.0f
#define LZ_RHO                   28.0f
#define LZ_BETA                  (8.0f / 3.0f)

/* Initial conditions — base.  Per-preset epsilon and seed direction below. */
#define IC_X                      1.0f
#define IC_Y                      1.0f
#define IC_Z                      1.0f

/* Per-projection viewport bounds.  Each (proj, axis) pair has its
 * own (min, max) so the attractor fills the drawable band whichever
 * 2-D slice we pick.  Bounds chosen wide enough to contain the
 * canonical Lorenz attractor with a small margin. */
#define X_AXIS_MIN              -25.0f
#define X_AXIS_MAX               25.0f
#define Y_AXIS_MIN              -30.0f
#define Y_AXIS_MAX               30.0f
#define Z_AXIS_MIN                0.0f
#define Z_AXIS_MAX               50.0f

/* Log-plot range — vertical axis of log|δ| plot.  LOG_MAX is the log
 * of the attractor's diameter (≈ 60).  The plot's lower bound is
 * derived per-preset by log_floor_for_eps() so the curve enters from
 * the bottom edge regardless of the active ε. */
#define LOG_PLOT_FLOOR_PAD        2.0f   /* log units below log(ε) */
#define LOG_MAX                   4.5f   /* log of attractor diameter */

/* Lyapunov exponent of canonical Lorenz — Sparrow 1982 [4].  Shown in
 * the HUD as a reference value the user can verify by reading the
 * slope of the LOG_ONLY preset's curve. */
#define LORENZ_LAMBDA              0.906f

/* Layout dividers and plot guards — named so paint_layout_* and
 * paint_log_plot read as intentions, not magic ints. */
#define PAINT_GUTTER_WIDTH           1     /* SPLIT: vertical divider cells     */
#define PAINT_DIVIDER_ROWS           1     /* PLOT_BOTTOM: horizontal divider   */
#define LOG_PLOT_BAND_NUMERATOR      2     /* PLOT_BOTTOM: bottom band ~ 2/5    */
#define LOG_PLOT_BAND_DENOMINATOR    5
#define PLOT_MIN_WIDTH_CELLS         4     /* below this, plot is unreadable    */
#define PLOT_MIN_HEIGHT_CELLS        3

/* HUD row-1 column widths — fixed so the three labelled segments
 * align across all 10 presets and 10 themes. */
#define HUD_PARAM_PRESET_WIDTH      19
#define HUD_PARAM_THEME_WIDTH       17

/*
 * Projection — which two axes of the 3-D Lorenz state we render.
 *
 * INTENT
 *   The Lorenz attractor is a 3-D object; the terminal is 2-D.  Each
 *   2-D projection drops one axis and reveals a different visual
 *   signature of the same dynamics.  Cycling the projection presets
 *   teaches the user that the BUTTERFLY shape is just ONE viewpoint
 *   on a more general 3-D structure.
 *
 * CONTEXT
 *   Read by trajectory_to_projection() (§8) which extracts the (u, v)
 *   pair from a 3-D LorenzState, and by projection_axis_bounds() (§9)
 *   which returns the per-projection (u_min, u_max, v_min, v_max).
 *   Stored as SDPreset::proj.
 *
 * MEMBER LOGIC
 *   PROJ_XZ : classic "butterfly" view — the iconic Lorenz silhouette.
 *             The two lobes appear because z spirals UP then x flips
 *             sign, tracing the wings.
 *   PROJ_XY : view from above — shows the convection-roll dynamic
 *             before z-dynamics are introduced.
 *   PROJ_YZ : sideways view — wing-flap dynamic; less iconic but
 *             reveals the asymmetry between the two lobes.
 *
 * REFERENCES
 *   [1] Lorenz 1963 §III plots all three projections in Fig. 1-3.
 */
typedef enum { PROJ_XZ = 0, PROJ_XY, PROJ_YZ } Projection;

/*
 * Layout — how the drawable band is partitioned for a given preset.
 *
 * INTENT
 *   Different presets need different real-estate splits: a full
 *   overlay maximises attractor detail; a side-by-side SPLIT lets
 *   the user compare trails without overlap; the PLOT_BOTTOM layout
 *   stacks the attractor and the log|δ| plot vertically so neither
 *   competes for the same cells (the original DIVERGENCE preset
 *   bug); PLOT_ONLY hands the screen to the divergence plot for
 *   pedagogy.
 *
 * CONTEXT
 *   Read by scene_paint() (§9) which dispatches into one of four
 *   named painters (paint_layout_full / _split / _plot_bottom /
 *   _plot_only).  Stored as SDPreset::layout.
 *
 * MEMBER LOGIC
 *   LAYOUT_FULL        : trajectories fill the whole drawable band.
 *   LAYOUT_SPLIT       : drawable split in half horizontally, one
 *                        trajectory per side (1-cell gutter divider).
 *   LAYOUT_PLOT_BOTTOM : attractor top (~60%), log|δ| plot bottom
 *                        (~40%), horizontal divider between.
 *   LAYOUT_PLOT_ONLY   : log|δ| plot fills the whole drawable band
 *                        — the educational view of [6] Lyapunov
 *                        exponent estimation.
 */
typedef enum {
    LAYOUT_FULL = 0,
    LAYOUT_SPLIT,
    LAYOUT_PLOT_BOTTOM,
    LAYOUT_PLOT_ONLY,
} Layout;

/*
 * Preset — the 10-entry index space for the presets[] table.
 *
 * INTENT
 *   Named indices into the presets[] table, ordered by pedagogical
 *   progression: classic view first (anchor), then alternate
 *   projections (XY, YZ), then the ε ladder (BIG → TINY) which
 *   teaches that time-to-diverge scales as -log(ε)/λ, then more
 *   trajectories (TRIO), then the layout variants (SPLIT, DELTA,
 *   LOG_ONLY, TRIO_LOG) which surface the log|δ| evidence.
 *
 * CONTEXT
 *   Used only as table indices; the entries themselves live in the
 *   presets[] array below.  PresetState (§7) wraps a single instance
 *   and cycles modulo N_PRESETS.  PRESET_CLASSIC is the boot-state
 *   default chosen by scene_init.
 *
 * MEMBER LOGIC
 *   See per-line comments — each entry pairs a (projection, layout,
 *   n_traj, ε) tuple into a memorable demo of sensitive dependence.
 *
 * REFERENCES
 *   [1] Lorenz 1963 — canonical CLASSIC preset.
 *   [2] Lorenz 1972 — "butterfly effect" name, the EPS_TINY preset's
 *       headline experience (one part in 10¹² perturbation visibly
 *       diverging in ~25-35 simulated seconds).
 *   [6] Wolf et al. (1985) — the LOG_ONLY preset visualises this
 *       algorithm's input directly.
 */
typedef enum {
    PRESET_CLASSIC = 0,   /* XZ butterfly, 2 trajectories, ε=1e-6   */
    PRESET_TOP_DOWN,      /* XY view, 2 trajectories                */
    PRESET_SIDE,          /* YZ view, 2 trajectories                */
    PRESET_EPS_BIG,       /* XZ, ε=1e-3 — diverges in seconds       */
    PRESET_EPS_TINY,      /* XZ, ε=1e-12 — stays close for minutes  */
    PRESET_TRIO,          /* XZ, 3 trajectories (base, base±ε)      */
    PRESET_SPLIT,         /* XZ, side-by-side                       */
    PRESET_DELTA,         /* XZ overlay + log|δ| plot (no overlap)  */
    PRESET_LOG_ONLY,      /* log|δ| plot fills the screen           */
    PRESET_TRIO_LOG,      /* TRIO + log plot                        */
    N_PRESETS,
} Preset;

/*
 * SDPreset — one row of the 10-preset demo zoo.
 *
 * INTENT
 *   Each row fully describes one entry in the catalogue: HUD label,
 *   which projection to render, which layout to partition the
 *   viewport with, how many trajectories to integrate, and how big
 *   the initial perturbation between trajectory 0 and 1 is.  Cycling
 *   through the table is the only way the user changes the demo.
 *
 * CONTEXT
 *   Read by scene_seed_perturbations (eps), scene_tick (n_traj,
 *   proj), trajectory_to_projection (proj), all painters (layout,
 *   proj), and the HUD writers (name + eps + n_traj).  The
 *   PresetState wrapper in §7 holds an index INTO this table.
 *
 * MEMBER LOGIC
 *   name    : 8-char HUD label, padded so the parameter column aligns.
 *   proj    : which 2-D projection of the Lorenz state to render
 *             (Projection enum).
 *   layout  : how the drawable band is partitioned (Layout enum).
 *   n_traj  : number of trajectories integrated.  Currently 2 or 3;
 *             TRAJ_MAX = 3 is the hard cap.
 *   eps     : initial ‖A − B‖ perturbation magnitude (applied along
 *             the +x axis for trajectory 1, ±x for trajectory 2 in
 *             TRIO).  Range covers six orders of magnitude (1e-3 to
 *             1e-12) so the time-to-diverge ladder is visible.
 *
 * REFERENCES
 *   [1] Lorenz 1963 — the demonstration this struct parameterises.
 *   [2] Lorenz 1972 — the ε-ladder dramatises the butterfly metaphor.
 */
typedef struct {
    const char *name;
    Projection  proj;
    Layout      layout;
    int         n_traj;
    float       eps;
} SDPreset;

static const SDPreset presets[N_PRESETS] = {
    /*  name        proj      layout              n  ε       */
    { "CLASSIC ", PROJ_XZ, LAYOUT_FULL,        2, 1e-6f  },
    { "TOP_DOWN", PROJ_XY, LAYOUT_FULL,        2, 1e-6f  },
    { "SIDE    ", PROJ_YZ, LAYOUT_FULL,        2, 1e-6f  },
    { "EPS_BIG ", PROJ_XZ, LAYOUT_FULL,        2, 1e-3f  },
    { "EPS_TINY", PROJ_XZ, LAYOUT_FULL,        2, 1e-12f },
    { "TRIO    ", PROJ_XZ, LAYOUT_FULL,        3, 1e-6f  },
    { "SPLIT   ", PROJ_XZ, LAYOUT_SPLIT,       2, 1e-6f  },
    { "DELTA   ", PROJ_XZ, LAYOUT_PLOT_BOTTOM, 2, 1e-6f  },
    { "LOG_ONLY", PROJ_XZ, LAYOUT_PLOT_ONLY,   2, 1e-6f  },
    { "TRIO_LOG", PROJ_XZ, LAYOUT_PLOT_BOTTOM, 3, 1e-6f  },
};

/*
 * Theme — one named palette for the demo.
 *
 * INTENT
 *   Group every colour code the renderer needs into ONE flat row so a
 *   "next theme" key cycles a single table.  Sensitive-dependence is
 *   special among the chaos demos because it overlays TWO ramps that
 *   must stay distinguishable: each theme's A and B ramp share a hue
 *   family (so the name is honest — OCEAN is all blues, FIRE all
 *   warms) but differ in shade/brightness so the user can still tell
 *   trajectory A from trajectory B.
 *
 * CONTEXT
 *   Indexed by PaletteState (§7); installed by theme_apply (§3) which
 *   pushes each band into PAIR_A_BASE + i / PAIR_B_BASE + i, the
 *   markers into PAIR_LIVE_A / _B / _C, and the helper colours into
 *   PAIR_PLOT and PAIR_AXIS.  Cycled by t/T via
 *   app_cycle_theme_next/prev.  All palette values follow the project
 *   Brightness Rule (CLAUDE.md): every code lives in the bright half
 *   of the 256-colour cube so the dots stay legible on default black.
 *
 * MEMBER LOGIC
 *   name    : 7-char HUD label.
 *   a[]     : TRAIL_BAND_COUNT (4) 256-colour codes for trajectory A,
 *             sorted dimmest oldest → brightest newest.
 *   b[]     : same shape for trajectory B.  Shares hue family with
 *             a[] but is distinguishable.
 *   live_a  : '@' marker colour for trajectory A (PAIR_LIVE_A).
 *   live_b  : '@' marker colour for trajectory B (PAIR_LIVE_B).
 *   live_c  : single solid colour for trajectory C in TRIO presets.
 *             No age banding — keeps the theme table small AND the
 *             3-way overlay readable (a third banded ramp would
 *             saturate the visible cells).
 *   plot    : log|δ| curve colour for DELTA / LOG_ONLY / TRIO_LOG.
 *   axis    : subtle axis-frame colour (dividers, plot border).
 *
 * REFERENCES
 *   CLAUDE.md "Theme Palette Brightness".
 */
typedef struct {
    const char *name;
    short       a[TRAIL_BAND_COUNT];
    short       b[TRAIL_BAND_COUNT];
    short       live_a, live_b, live_c, plot, axis;
} Theme;

/*
 * Each theme provides TWO ramps in the theme's hue family — A and B
 * must be distinguishable (different shade/brightness) so the user
 * can tell the two trajectories apart, but both must fit the theme
 * name.  Hue choices:
 *
 *   DEFAULT : blue (A) vs warm red→yellow (B) — max-contrast for new users
 *   MATRIX  : deep green (A) vs bright lime (B)
 *   NOVA    : magenta (A) vs bright yellow (B)               — "explosion" pair
 *   MONO    : light gray (A) vs medium gray (B)
 *   OCEAN   : deep cyan (A) vs sky/ice blue (B)
 *   FIRE    : red→orange (A) vs yellow→white (B)             — both warm
 *   EARTH   : brown (A) vs olive/sienna (B)
 *   FOREST  : dark green (A) vs leaf green (B)
 *   DESERT  : sand (A) vs rust/burnt (B)
 *   ARCTIC  : ice blue (A) vs near-white cyan (B)
 *
 * All entries stay 30+ / 244+ per the brightness rule (CLAUDE.md). */
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    /* name        A: oldest→newest    B: oldest→newest    lv_a lv_b lv_c plot axis */
    { "DEFAULT",   {  75, 117, 153, 195 }, { 196, 202, 208, 220 },  51, 220, 213, 226, 244 },
    { "MATRIX",    {  34,  40,  46,  82 }, { 154, 190, 226, 228 },  46, 226,  87, 220, 244 },
    { "NOVA",      { 165, 171, 207, 219 }, { 208, 214, 220, 227 }, 213, 226,  51, 219, 244 },
    { "MONO",      { 244, 247, 250, 253 }, { 240, 242, 245, 248 }, 255, 244, 226, 226, 240 },
    { "OCEAN",     {  31,  38,  44,  51 }, { 117, 159, 195, 231 },  51, 195, 117, 226, 244 },
    { "FIRE",      { 196, 202, 208, 214 }, { 220, 226, 228, 229 }, 196, 226, 208, 226, 244 },
    { "EARTH",     {  94, 130, 137, 143 }, { 100, 142, 178, 215 }, 130, 178, 215, 220, 244 },
    { "FOREST",    {  34,  40,  46,  82 }, { 113, 149, 185, 191 },  46, 154, 220, 226, 244 },
    { "DESERT",    { 137, 179, 215, 222 }, { 130, 166, 208, 214 }, 215, 208, 226, 220, 244 },
    { "ARCTIC",    { 117, 153, 195, 231 }, {  51,  87, 123, 159 }, 231,  51, 220, 226, 244 },
};

/* ===================================================================== */
/* §2 clock + §3 color                                                    */
/* ===================================================================== */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* theme_install_256 — push a Theme's bright-cube codes into all the
 * ncurses pair classes the renderer needs (4 A-bands + 4 B-bands +
 * 3 live markers + plot + axis). */
static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++) {
        init_pair(PAIR_A_BASE + i, t->a[i], -1);
        init_pair(PAIR_B_BASE + i, t->b[i], -1);
    }
    init_pair(PAIR_LIVE_A, t->live_a, -1);
    init_pair(PAIR_LIVE_B, t->live_b, -1);
    init_pair(PAIR_LIVE_C, t->live_c, -1);
    init_pair(PAIR_PLOT,   t->plot,   -1);
    init_pair(PAIR_AXIS,   t->axis,   -1);
}

/* theme_install_8color_fallback — graceful degradation for the few
 * terminals that advertise only 8 colours.  All A-band cells collapse
 * to cyan and all B-band cells to red; the third trajectory is
 * magenta; plot/axis are yellow/white. */
static inline void theme_install_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++) {
        init_pair(PAIR_A_BASE + i, COLOR_CYAN, -1);
        init_pair(PAIR_B_BASE + i, COLOR_RED,  -1);
    }
    init_pair(PAIR_LIVE_A, COLOR_CYAN,    -1);
    init_pair(PAIR_LIVE_B, COLOR_RED,     -1);
    init_pair(PAIR_LIVE_C, COLOR_MAGENTA, -1);
    init_pair(PAIR_PLOT,   COLOR_YELLOW,  -1);
    init_pair(PAIR_AXIS,   COLOR_WHITE,   -1);
}

/* theme_apply — push the chosen palette to the ncurses pair table.
 *
 * DRIVER PSEUDOCODE
 *   if idx out of range:  idx = 0                  // graceful fallback
 *   if terminal has 256 colours:
 *       theme_install_256(themes[idx])
 *   else:
 *       theme_install_8color_fallback()
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_install_256(&themes[idx]);
    else               theme_install_8color_fallback();
}
static void color_init(void) { start_color(); use_default_colors();
    if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
    else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
    theme_apply(0); }

/* ===================================================================== */
/* §5  Lorenz ODE — system + state composite, RK4 integrator             */
/* ===================================================================== */

/*
 * Vec3 — generic 3-component vector.
 *
 * INTENT
 *   The single 3-tuple this file traffics in.  Used three ways:
 *     • a Lorenz phase point (LorenzState alias)
 *     • a Lorenz derivative dy/dt (output of lorenz_deriv)
 *     • a 2-D projection sample after trajectory_to_projection
 *   Flat by-value struct so RK4 stages combine cheaply via state_add
 *   without any heap allocation.
 *
 * CONTEXT
 *   Returned by lorenz_deriv; stored as LorenzState inside Lorenz;
 *   read by trajectory_to_projection.  Same shape used in every
 *   other 3-D chaos demo in this project (rossler, strange_attractor).
 *
 * MEMBER LOGIC
 *   x, y, z : scalar components.  In Lorenz context they are the
 *             three observables of the ODE — x is the convection
 *             intensity, y the horizontal temperature variation,
 *             z the vertical temperature variation.
 *
 * REFERENCES
 *   [1] Lorenz 1963 — physical interpretation of (x, y, z).
 */
typedef struct { float x, y, z; } Vec3;

/*
 * LorenzState — semantic alias for Vec3 at the "ODE phase point" site.
 *
 * INTENT
 *   Same memory layout as Vec3; the typedef just labels intent so a
 *   call like state_add(LorenzState, dt, slope) reads as ODE
 *   arithmetic, not generic vector arithmetic.  No new fields.
 *   Mirrors the pattern in ./rossler_attractor.c (RosslerState = Vec3).
 *
 * CONTEXT
 *   Lives inside Lorenz::state.  Mutated by lorenz_rk4_step on every
 *   substep.  Read by trajectory_to_projection (for trail/marker
 *   rendering) and by delta_norm (for the log|δ| plot).
 *
 * MEMBER LOGIC
 *   Inherited from Vec3 — x, y, z scalar components, interpreted as
 *   the Lorenz phase-point coordinates at the current sim time.
 *
 * REFERENCES
 *   [1] Lorenz 1963 — the (x, y, z) coordinate triple these scalars name.
 */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — invariant parameters of the Lorenz ODE [1].
 *
 * INTENT
 *   Split parameters away from state so lorenz_deriv becomes a PURE
 *   function of (state, system).  In this demo all trajectories share
 *   the canonical (10, 28, 8/3) so the system field is the SAME for
 *   every Lorenz composite — sensitive dependence requires identical
 *   physics; only initial conditions differ.  The split is kept
 *   (rather than collapsing to bare constants) because every other
 *   chaos demo in this project uses the System+State+Composite layout
 *   (Sussman & Wisdom [9] §1.6) and consistency matters.
 *
 * CONTEXT
 *   Read-only inside lorenz_deriv and lorenz_rk4_step.  Mutated only
 *   in scene_seed_perturbations (which calls lorenz_system_canonical
 *   to populate it).
 *
 * MEMBER LOGIC
 *   sigma : Prandtl number σ — rate of convective heat transfer.
 *           Canonical 10.0.
 *   rho   : Rayleigh number ρ — temperature-gradient driving force.
 *           Canonical 28.0 — past the supercritical Hopf bifurcation
 *           at ρ ≈ 24.74, deep into chaos.
 *   beta  : geometric factor β = 8/3 in the original derivation
 *           (Lorenz [1] §III).  Aspect ratio of the convection cells.
 *
 * REFERENCES
 *   [1] Lorenz 1963 §II — derivation from the Saltzman convection
 *       equations; the original σ, ρ, β values.
 *   [4] Sparrow 1982 §1.2 — definitive analysis of the
 *       parameter-space structure around (10, 28, 8/3).
 *   [9] Sussman & Wisdom §1.6 — system/state separation rationale.
 */
typedef struct { float sigma, rho, beta; } LorenzSystem;

/*
 * Lorenz — composite: invariant system + current state.
 *
 * INTENT
 *   One value carrying everything needed to advance ONE Lorenz
 *   trajectory through RK4.  Mirrors the abstraction layout used by
 *   every chaos demo in this project (Rössler, double pendulum,
 *   Hénon-Heiles).  The 3 trajectories in Scene live in this struct
 *   so a per-trajectory step is a single named call
 *   (lorenz_rk4_step(&traj[i], dt)) rather than three loose float
 *   parameters that callers must keep synchronised.
 *
 * CONTEXT
 *   Owned by Scene as an array (traj[TRAJ_MAX]).  lorenz_rk4_step
 *   mutates state while reading system as const.
 *   scene_seed_perturbations rebuilds the whole composite on every
 *   r-key / preset-change.
 *
 * MEMBER LOGIC
 *   system : the (σ, ρ, β) triple — never mutated by lorenz_rk4_step.
 *            Identical across every trajectory in this demo.
 *   state  : the (x, y, z) phase point — mutated each substep.
 *            DIFFERS between trajectories by exactly ε in x.
 *
 * REFERENCES
 *   [1] Lorenz 1963 — the ODE this composite advances.
 *   [7] Numerical Recipes §17.1 — the RK4 integrator that operates
 *       on this composite.
 *   [9] Sussman & Wisdom §1.6 — System/State separation rationale.
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

/* lorenz_system_canonical — the (σ=10, ρ=28, β=8/3) regime from
 * Lorenz 1963 [1] §III.  All trajectories in this demo use these
 * values; varying parameters is left to ./strange_attractor.c. */
static inline LorenzSystem lorenz_system_canonical(void)
{
    return (LorenzSystem){ LZ_SIGMA, LZ_RHO, LZ_BETA };
}

/* lorenz_deriv — evaluate dy/dt = f(state, system).  Lorenz 1963 [1]
 * §III equations 25-27; each line maps 1-to-1 to one published formula:
 *
 *   dx/dt = σ(y − x)
 *   dy/dt = x(ρ − z) − y
 *   dz/dt = xy − βz
 */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);              /* σ(y − x)        */
    dy.y = s->x * (sys->rho - s->z) - s->y;         /* x(ρ − z) − y    */
    dy.z = s->x * s->y - sys->beta * s->z;          /* xy − βz         */
    return dy;
}

/* state_add — y + h·k, returned as a new LorenzState.  Pure helper
 * for combining an RK4 stage's slope into a midpoint estimate. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* RK4 Butcher tableau constants — Numerical Recipes [7] §17.1. */
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

/* lorenz_rk4_step — one fixed-step RK4 update of the composite.
 * Classical 4-stage Runge-Kutta with the Butcher (1/6, 1/3, 1/3, 1/6)
 * tableau (Numerical Recipes [7] §17.1):
 *
 *   STAGE 1 — slope_start = f(y)
 *   STAGE 2 — slope_mid_1 = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2 = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end   = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 */
static void lorenz_rk4_step(Lorenz *L, float dt)
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
/* §6  trail + log_plot — ring buffers for visible streak & δ history   */
/* ===================================================================== */

/*
 * Trail — fixed-capacity ring buffer of (u, v) 2-D projection samples.
 *
 * INTENT
 *   Decouple "how long the visible streak is" from "how fast we
 *   advance the integrator" — the renderer reads up to TRAIL_MAX
 *   samples, the integrator pushes one every RK4 substep.  No malloc.
 *   Each trajectory in Scene gets its own Trail so two or three can
 *   be overlaid without sample interleaving.
 *
 * MEMBER LOGIC
 *   x[], y[] : projected (u, v) coords (NOT raw 3-D state).  This
 *              lets paint_trail draw directly without re-running
 *              trajectory_to_projection on every sample.
 *   head     : ring-buffer index of the most recent sample.
 *   count    : valid-sample count, saturates at TRAIL_MAX.
 */
typedef struct {
    float x[TRAIL_MAX], y[TRAIL_MAX];
    int   head, count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }
static void trail_push(Trail *t, float px, float py)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->x[t->head] = px;
    t->y[t->head] = py;
    if (t->count < TRAIL_MAX) t->count++;
}

/*
 * LogPlot — fixed-capacity ring buffer of log|δ(t)| samples used by
 * the DELTA / LOG_ONLY / TRIO_LOG presets.
 *
 * INTENT
 *   Pre-computing logf at push time (once per sample) is much cheaper
 *   than logf at paint time (once per visible cell).  Storing log|δ|
 *   instead of |δ| also lets the plot's y-axis be linear in log-space,
 *   so the largest Lyapunov exponent λ shows up as a literal slope
 *   (Wolf et al. [6]) until |δ| saturates near the attractor size.
 *
 * MEMBER LOGIC
 *   v[]   : pre-logf'd sample values.  Clamped to log_floor by the
 *           paint code (so the curve enters from the bottom edge).
 *   head  : ring index of the most recent sample.
 *   count : valid-sample count, saturates at PLOT_MAX.
 */
typedef struct {
    float v[PLOT_MAX];
    int   head, count;
} LogPlot;

static void plot_reset(LogPlot *p) { p->head = 0; p->count = 0; }
static void plot_push(LogPlot *p, float v)
{
    p->head = (p->head + 1) % PLOT_MAX;
    p->v[p->head] = v;
    if (p->count < PLOT_MAX) p->count++;
}

/* ===================================================================== */
/* §7  state — typed wrappers around the two cycled selections           */
/* ===================================================================== */

/*
 * PresetState — typed wrapper around "which preset is loaded".
 *
 * INTENT
 *   Wrap the bare Preset enum in a struct so the key-binding table
 *   reads as intentions ("cycle to next preset") rather than modular
 *   arithmetic on N_PRESETS.  Replace-Primitive-with-Object pattern;
 *   typing also prevents a "next theme" keystroke from accidentally
 *   cycling presets[] or vice versa.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS) = [0, 10).
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const SDPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState; distinct type so a t/T keystroke
 *   physically cannot reach the presets[] table.  Bundles the active
 *   index plus the helper that re-pushes the theme to ncurses pairs.
 *   Replace-Primitive-with-Object pattern (Fowler).
 *
 * CONTEXT
 *   Owned by Scene; mutated via palette_state_cycle_next/prev (t/T
 *   keys) followed immediately by scene_apply_theme, which pushes the
 *   resulting band[]/live/plot/axis codes back into the ncurses pair
 *   table so the next frame paints with the new palette.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§1).  Must be in
 *             [0, N_THEMES) = [0, 10); cycle helpers maintain this
 *             invariant via modular arithmetic on N_THEMES.
 *
 * REFERENCES
 *   Fowler, M. — *Refactoring*, "Replace Primitive with Object".
 */
typedef struct { int current; } PaletteState;

static void palette_state_init      (PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)              { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)              { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)    { return &themes[p->current]; }
static void palette_state_apply     (const PaletteState *p)        { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundle every piece of mutable state into ONE struct so the App
 *   layer owns one Scene and the main loop drives it through a tiny
 *   named-method API (scene_init, scene_reset, scene_tick,
 *   scene_apply_theme).  Each sub-field is its own typed concept so
 *   the compiler enforces the boundary between physics (the Lorenz
 *   composites), geometry (trails + log plot), selection (preset,
 *   palette), and runtime (paused, t_sim).
 *
 * CONTEXT
 *   Owned by App as a value.  Accessed by §9 painters and §10 app
 *   helpers.  All mutation goes through scene_* / app_* helpers so
 *   call sites stay declarative.
 *
 * MEMBER LOGIC
 *   traj[]    : up to TRAJ_MAX Lorenz composites (system + state).
 *               All share the canonical (σ, ρ, β) but start from
 *               ε-different initial conditions:
 *                 traj[0]: base = (IC_X, IC_Y, IC_Z)
 *                 traj[1]: base + ε·x̂
 *                 traj[2]: base − ε·x̂   (TRIO presets only)
 *   trail[]   : ring buffer per trajectory holding the active
 *               2-D projection (u, v).  Re-cleared on r/n/p.
 *   log_delta : ring buffer of log‖traj[0].state − traj[1].state‖
 *               values; used by DELTA / LOG_ONLY / TRIO_LOG presets.
 *   t_sim     : seconds of simulated time since reset; HUD readout.
 *   preset    : §7 PresetState — which row of presets[].
 *   palette   : §7 PaletteState — which colour theme.
 *   paused    : when true, scene_tick early-returns.
 *
 * REFERENCES
 *   [9] Sussman & Wisdom (2014), *Structure and Interpretation of
 *       Classical Mechanics*, §1.6 — the System / State / Composite
 *       split this layer mirrors at the simulation-root level.
 */
typedef struct {
    Lorenz       traj [TRAJ_MAX];
    Trail        trail[TRAJ_MAX];
    LogPlot      log_delta;
    float        t_sim;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
} Scene;

/* scene_active_preset — accessor so call sites don't repeat
 * presets[s->preset.current] everywhere. */
static inline const SDPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}

/* scene_apply_theme — push the active palette to ncurses pairs. */
static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* scene_seed_perturbations — apply the active preset's ε to each
 * trajectory.  Slot 0 is the base; slot 1 is base + ε·x̂; slot 2
 * (TRIO only) is base − ε·x̂ so the trio fans symmetrically.  Every
 * slot gets the canonical Lorenz system; only the state differs.
 *
 * This is the experiment Lorenz [1] [2] originally ran — two
 * trajectories from initial conditions that differ by a tiny
 * fraction of the attractor diameter, integrated with identical
 * physics.  Eckmann-Ruelle [5] §1 formalises why those trajectories
 * MUST diverge exponentially in any system with positive λ. */
static void scene_seed_perturbations(Scene *s)
{
    float eps = scene_active_preset(s)->eps;
    LorenzSystem sys = lorenz_system_canonical();

    s->traj[0] = (Lorenz){ sys, (LorenzState){ IC_X,        IC_Y, IC_Z } };
    s->traj[1] = (Lorenz){ sys, (LorenzState){ IC_X + eps,  IC_Y, IC_Z } };
    s->traj[2] = (Lorenz){ sys, (LorenzState){ IC_X - eps,  IC_Y, IC_Z } };
}

static void scene_reset(Scene *s)
{
    scene_seed_perturbations(s);
    for (int i = 0; i < TRAJ_MAX; i++) trail_reset(&s->trail[i]);
    plot_reset(&s->log_delta);
    s->t_sim = 0.0f;
}
static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_CLASSIC);
    palette_state_init(&s->palette, 0);
    scene_reset(s);
}

/* trajectory_to_projection — extract the (u, v) pair for the active
 * projection.  Centralises the axis selection so paint and trail
 * code reads world coordinates through one named accessor. */
static inline void trajectory_to_projection(const LorenzState *p, Projection proj,
                                            float *u, float *v)
{
    switch (proj) {
        default:
        case PROJ_XZ: *u = p->x; *v = p->z; break;
        case PROJ_XY: *u = p->x; *v = p->y; break;
        case PROJ_YZ: *u = p->y; *v = p->z; break;
    }
}

/* delta_norm — ‖a − b‖₃ between two trajectory states.  The HUD
 * shows |δ| directly; LogPlot stores its log.  In the unsaturated
 * regime log|δ(t)| ≈ log|δ₀| + λ·t (Eckmann-Ruelle [5] §1, Wolf
 * et al. [6] §2) — the visible slope of the LOG_ONLY preset's
 * curve IS the largest Lyapunov exponent. */
static inline float delta_norm(const LorenzState *a, const LorenzState *b)
{
    float dx = a->x - b->x, dy = a->y - b->y, dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* log_floor_for_eps — bottom of the log|δ| plot's vertical axis,
 * derived from the active ε so the curve starts a few log units
 * BELOW log(ε), not at a fixed -14. */
static inline float log_floor_for_eps(float eps)
{
    return logf(eps) - LOG_PLOT_FLOOR_PAD;
}

/* trajectory_advance_and_record_sample — one RK4 substep for one
 * trajectory, plus the projected (u, v) trail sample.  Named so
 * scene_advance_one_substep reads as "for each trajectory: advance
 * + record" rather than 4 inline statements. */
static inline void trajectory_advance_and_record_sample(
    Lorenz *traj, Trail *trail, Projection proj)
{
    lorenz_rk4_step(traj, LORENZ_DT);
    float u, v;
    trajectory_to_projection(&traj->state, proj, &u, &v);
    trail_push(trail, u, v);
}

/* scene_record_divergence — push log‖traj[0] − traj[1]‖ onto the
 * LogPlot.  Clamps the log to log_floor so a zero δ (impossible
 * after the first substep, but cheap to guard) doesn't −inf the
 * curve.  This is the [6] Wolf et al. raw input signal. */
static inline void scene_record_divergence(Scene *s, float log_floor)
{
    float d = delta_norm(&s->traj[0].state, &s->traj[1].state);
    plot_push(&s->log_delta, (d > 0.0f) ? logf(d) : log_floor);
}

/* scene_advance_one_substep — one RK4 substep for every live
 * trajectory, then record divergence into the log plot.  This is the
 * smallest indivisible step of the simulation: every trajectory MUST
 * advance with identical dt or the sensitive-dependence demo would
 * measure numerical drift instead of chaos. */
static inline void scene_advance_one_substep(Scene *s,
                                             const SDPreset *active,
                                             float log_floor)
{
    for (int t = 0; t < active->n_traj; t++)
        trajectory_advance_and_record_sample(
            &s->traj[t], &s->trail[t], active->proj);
    s->t_sim += LORENZ_DT;
    scene_record_divergence(s, log_floor);
}

/* scene_tick — advance the simulation by one fixed-timestep tick.
 *
 * DRIVER PSEUDOCODE
 *   if paused: return                               // freeze sim
 *   active     = scene_active_preset(s)
 *   log_floor  = log_floor_for_eps(active.eps)      // per-preset axis
 *   repeat INT_STEPS_PER_TICK times:
 *       scene_advance_one_substep(s, active, log_floor)
 */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;

    const SDPreset *active = scene_active_preset(s);
    float log_floor = log_floor_for_eps(active->eps);

    for (int sub = 0; sub < INT_STEPS_PER_TICK; sub++)
        scene_advance_one_substep(s, active, log_floor);
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
 *   means a resize touches a single named state.  Every painter
 *   takes Screen* (or cols/rows) explicitly so layout math is
 *   decoupled from ncurses globals.
 *
 * CONTEXT
 *   Owned by App.  Initialised by screen_init; resized by
 *   screen_resize when SIGWINCH fires; consulted everywhere a draw
 *   needs the viewport.  Re-read after resize before scene_reset is
 *   called.
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

/* projection_axis_bounds — (u_min, u_max, v_min, v_max) for each of
 * the three 2-D projections.  Wide enough to contain the canonical
 * Lorenz attractor with a small margin so the camera doesn't crop. */
static inline void projection_axis_bounds(Projection proj,
                                          float *u_min, float *u_max,
                                          float *v_min, float *v_max)
{
    switch (proj) {
        default:
        case PROJ_XZ: *u_min = X_AXIS_MIN; *u_max = X_AXIS_MAX;
                      *v_min = Z_AXIS_MIN; *v_max = Z_AXIS_MAX; break;
        case PROJ_XY: *u_min = X_AXIS_MIN; *u_max = X_AXIS_MAX;
                      *v_min = Y_AXIS_MIN; *v_max = Y_AXIS_MAX; break;
        case PROJ_YZ: *u_min = Y_AXIS_MIN; *u_max = Y_AXIS_MAX;
                      *v_min = Z_AXIS_MIN; *v_max = Z_AXIS_MAX; break;
    }
}

/* world_u_to_cell_x / world_v_to_cell_y — map a world-space coord
 * within [min, max] onto an integer cell coord within [gx0, gx0+w).
 * Vertical axis is flipped (v_max at the TOP of the screen). */
static inline int world_u_to_cell_x(float u, int gx0, int w,
                                    float u_min, float u_max)
{
    int c = gx0 + (int)((u - u_min) / (u_max - u_min) * (float)w);
    if (c < gx0)        c = gx0;
    if (c >= gx0 + w)   c = gx0 + w - 1;
    return c;
}
static inline int world_v_to_cell_y(float v, int gy0, int h,
                                    float v_min, float v_max)
{
    int c = gy0 + (h - 1) - (int)((v - v_min) / (v_max - v_min) * (float)h);
    if (c < gy0)        c = gy0;
    if (c >= gy0 + h)   c = gy0 + h - 1;
    return c;
}

/* paint_trail_in_rect — render one ring buffer of (u, v) samples
 * into a viewport rectangle, using the given pair_base for age-band
 * colours.  When `pair_base < 0` the trail paints as a single solid
 * colour pair `live_pair` (used for trajectory C in TRIO presets). */
/* trail_oldest_index — ring-buffer offset of the oldest valid sample.
 * (head − count + 1) mod TRAIL_MAX, with TRAIL_MAX added to keep the
 * intermediate non-negative under C's truncating mod. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* trail_age_band — map a sample's age (0 = newest, n-1 = oldest)
 * onto one of TRAIL_BAND_COUNT colour tiers.  Newer samples get
 * higher band indices = brighter colours.  Clamped to a valid band. */
static inline int trail_age_band(int age_from_newest, int n)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age_from_newest * TRAIL_BAND_COUNT) / n;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* trail_pair_for_age — which ncurses pair to use for ONE trail
 * sample.  Banded ramp when pair_base ≥ 0 (trajectories A, B);
 * solid live_pair when pair_base < 0 (trajectory C, which uses a
 * single colour rather than its own 4-band ramp). */
static inline short trail_pair_for_age(int pair_base, int live_pair,
                                       int age_from_newest, int n)
{
    if (pair_base < 0) return (short)live_pair;
    return (short)(pair_base + trail_age_band(age_from_newest, n));
}

/* paint_cell — bracket attron / mvaddch / attroff so painters can
 * place a glyph in one line instead of three.  Glyph cast handles
 * the chtype sign-extension trap (CLAUDE.md "Common ncurses Bugs"). */
static inline void paint_cell(int sy, int sx, char glyph, short pair_id)
{
    attron(COLOR_PAIR(pair_id) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair_id) | A_BOLD);
}

/* paint_trail_in_rect — render one trail buffer into a viewport.
 *
 * DRIVER PSEUDOCODE
 *   if trail empty: return
 *   n = trail.count
 *   for i = 0 .. n-1:                              // oldest → newest
 *       sample = trail_sample_at_offset(i)
 *       (sx, sy) = world_coord_to_cell(sample, rect, bounds)
 *       pair  = trail_pair_for_age(pair_base, live_pair, age, n)
 *       paint_cell(sy, sx, '.', pair)
 */
static void paint_trail_in_rect(const Trail *tr, int pair_base, int live_pair,
                                int gx0, int gy0, int w, int h,
                                float u_min, float u_max,
                                float v_min, float v_max)
{
    if (tr->count == 0) return;
    int n      = tr->count;
    int oldest = trail_oldest_index(tr);

    for (int i = 0; i < n; i++) {
        int idx              = (oldest + i) % TRAIL_MAX;
        int age_from_newest  = n - 1 - i;
        int sx               = world_u_to_cell_x(tr->x[idx], gx0, w, u_min, u_max);
        int sy               = world_v_to_cell_y(tr->y[idx], gy0, h, v_min, v_max);
        short pair           = trail_pair_for_age(pair_base, live_pair,
                                                  age_from_newest, n);
        paint_cell(sy, sx, '.', pair);
    }
}

/* paint_log_plot_axis_frame — subtle L-shape axis (left edge +
 * bottom edge) in PAIR_AXIS.  Top and right edges left open so the
 * curve can extend to the latest sample without bumping the border. */
static void paint_log_plot_axis_frame(int gx0, int gy0, int w, int h)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int y = 0; y < h; y++) mvaddch(gy0 + y, gx0, (chtype)'|');
    for (int x = 0; x < w; x++) mvaddch(gy0 + h - 1, gx0 + x, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* log_plot_newest_first_index — ring offset of the i-th newest
 * sample (i=0 is the latest, i=count-1 is the oldest).  Centralises
 * the ring-buffer wrap so paint_log_plot reads as a clean newest →
 * oldest walk. */
static inline int log_plot_newest_first_index(const LogPlot *p, int i_from_newest)
{
    int oldest = (p->head - p->count + 1 + PLOT_MAX) % PLOT_MAX;
    return (oldest + p->count - 1 - i_from_newest) % PLOT_MAX;
}

/* log_value_to_cell_y — clamp a log|δ| value into the plot's vertical
 * range, then map it onto an integer cell row.  Top of the rect is
 * LOG_MAX; bottom is log_floor.  The linear-y mapping is exactly
 * what makes the largest Lyapunov exponent λ ([6] Wolf et al.) show
 * up as a literal slope in the chart. */
static inline int log_value_to_cell_y(float lg, float log_floor,
                                      int gy0, int h)
{
    if (lg < log_floor) lg = log_floor;
    if (lg > LOG_MAX)   lg = LOG_MAX;
    float fraction = (lg - log_floor) / (LOG_MAX - log_floor);
    return gy0 + (h - 1) - (int)(fraction * (float)(h - 1));
}

/* paint_log_plot_curve — paint the visible samples newest → oldest,
 * placing one '*' per cell column.  Stops at w-1 samples since the
 * axis frame occupies column gx0. */
static void paint_log_plot_curve(const LogPlot *p, int gx0, int gy0,
                                 int w, int h, float log_floor)
{
    int n = p->count;
    int max_samples = (n < w - 1) ? n : (w - 1);

    attron(COLOR_PAIR(PAIR_PLOT) | A_BOLD);
    for (int i = 0; i < max_samples; i++) {
        int   idx = log_plot_newest_first_index(p, i);
        float lg  = p->v[idx];
        int   sy  = log_value_to_cell_y(lg, log_floor, gy0, h);
        int   sx  = gx0 + (w - 1) - i;
        if (sx > gx0 && sy >= gy0 && sy < gy0 + h)
            mvaddch(sy, sx, (chtype)'*');
    }
    attroff(COLOR_PAIR(PAIR_PLOT) | A_BOLD);
}

/* paint_log_plot — log|δ(t)| curve with a faint axis frame.
 *
 * DRIVER PSEUDOCODE
 *   if no samples OR too small: return                     // unreadable
 *   paint_log_plot_axis_frame(rect)                        // L-shape
 *   paint_log_plot_curve(plot, rect, log_floor)            // '*' curve
 */
static void paint_log_plot(const LogPlot *p, int gx0, int gy0,
                           int w, int h, float log_floor)
{
    if (p->count == 0
     || w < PLOT_MIN_WIDTH_CELLS
     || h < PLOT_MIN_HEIGHT_CELLS) return;

    paint_log_plot_axis_frame(gx0, gy0, w, h);
    paint_log_plot_curve     (p, gx0, gy0, w, h, log_floor);
}

/*
 * Rect — a sub-rectangle of the drawable band.
 *
 * INTENT
 *   The layout dispatcher partitions the drawable band into one or
 *   two rectangles (full / split / plot_bottom).  Passing each one
 *   as a Rect (rather than 4 loose ints) makes paint_attractor_panel
 *   and paint_log_plot take a single named argument and prevents the
 *   "swapped gx0/gy0" bug class.
 *
 * MEMBER LOGIC
 *   gx0 : leftmost column of the rect (inclusive).
 *   gy0 : top row of the rect (inclusive).
 *   w   : width in cells.  Rect spans columns [gx0, gx0 + w).
 *   h   : height in cells.  Rect spans rows    [gy0, gy0 + h).
 */
typedef struct { int gx0, gy0, w, h; } Rect;

/* paint_live_marker — '@' glyph at the trajectory's current state.
 *
 * DRIVER PSEUDOCODE
 *   (u, v)   = trajectory_to_projection(state, proj)
 *   (sx, sy) = world_coord_to_cell(u, v, rect, bounds)
 *   paint_cell(sy, sx, '@', pair)
 */
static void paint_live_marker(const Vec3 *p, Projection proj, short pair,
                              const Rect *r,
                              float u_min, float u_max,
                              float v_min, float v_max)
{
    float u, v;
    trajectory_to_projection(p, proj, &u, &v);
    int sx = world_u_to_cell_x(u, r->gx0, r->w, u_min, u_max);
    int sy = world_v_to_cell_y(v, r->gy0, r->h, v_min, v_max);
    paint_cell(sy, sx, '@', pair);
}

/* trail_pair_table / live_pair_table — index 0=A, 1=B, 2=C.  A pair
 * base of −1 means "no banded ramp; use live_pair as a single solid
 * colour" (used by trajectory C in TRIO presets). */
static const int trail_pair_table[TRAJ_MAX] = { PAIR_A_BASE, PAIR_B_BASE, -1 };
static const int live_pair_table [TRAJ_MAX] = { PAIR_LIVE_A, PAIR_LIVE_B, PAIR_LIVE_C };

/* paint_attractor_panel — render n trails + live markers into ONE
 * rectangle.  Used standalone (full / plot_bottom layouts) and twice
 * per frame in SPLIT layout (once per half).
 *
 * DRIVER PSEUDOCODE
 *   active = scene_active_preset(s)
 *   bounds = projection_axis_bounds(active.proj)
 *   for t = trail_start .. trail_start + trail_count - 1, t < n_traj:
 *       paint_trail_in_rect(trail[t], pair_table[t], live_table[t],
 *                           rect, bounds)
 *   for t = trail_start .. trail_start + trail_count - 1, t < n_traj:
 *       paint_live_marker(traj[t].state, proj, live_table[t],
 *                         rect, bounds)
 *
 * Two passes (all trails first, then all markers) so the '@' markers
 * always sit on top of the '.' trails regardless of paint order. */
static void paint_attractor_panel(const Scene *s, const Rect *r,
                                  int trail_start, int trail_count)
{
    const SDPreset *active = scene_active_preset(s);
    Projection proj = active->proj;

    float u_min, u_max, v_min, v_max;
    projection_axis_bounds(proj, &u_min, &u_max, &v_min, &v_max);

    for (int t = trail_start; t < trail_start + trail_count; t++) {
        if (t >= active->n_traj) break;
        paint_trail_in_rect(&s->trail[t], trail_pair_table[t], live_pair_table[t],
                            r->gx0, r->gy0, r->w, r->h,
                            u_min, u_max, v_min, v_max);
    }
    for (int t = trail_start; t < trail_start + trail_count; t++) {
        if (t >= active->n_traj) break;
        paint_live_marker(&s->traj[t].state, proj, (short)live_pair_table[t], r,
                          u_min, u_max, v_min, v_max);
    }
}

/* drawable_band_full — the entire drawable area as a single Rect. */
static inline Rect drawable_band_full(int cols, int rows)
{
    return (Rect){ 0, HUD_TOP_ROWS, cols, rows - HUD_BAND_RESERVED_ROWS };
}

/* paint_vertical_divider / _horizontal_divider — single-cell-wide
 * dividers in PAIR_AXIS, used to mark the SPLIT gutter and the
 * PLOT_BOTTOM panel boundary. */
static void paint_vertical_divider(int col, int gy0, int h)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int y = 0; y < h; y++) mvaddch(gy0 + y, col, (chtype)'|');
    attroff(COLOR_PAIR(PAIR_AXIS));
}
static void paint_horizontal_divider(int row, int cols)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int x = 0; x < cols; x++) mvaddch(row, x, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* paint_layout_full — every trajectory overlaid in the full band. */
static void paint_layout_full(const Scene *s, int cols, int rows)
{
    Rect band = drawable_band_full(cols, rows);
    paint_attractor_panel(s, &band, 0, TRAJ_MAX);
}

/* paint_layout_split — drawable cut horizontally in half; trajectory
 * A on the left, B on the right, with a single-cell vertical gutter
 * between them.
 *
 * DRIVER PSEUDOCODE
 *   drawable_h  = rows - HUD_BAND_RESERVED_ROWS
 *   half_w      = (cols - PAINT_GUTTER_WIDTH) / 2
 *   left_rect   = (0,                        HUD_TOP_ROWS, half_w, drawable_h)
 *   right_rect  = (half_w + PAINT_GUTTER_WIDTH, HUD_TOP_ROWS, half_w, drawable_h)
 *   paint_attractor_panel(left,  trajectory A only)
 *   paint_attractor_panel(right, trajectory B only)
 *   paint_vertical_divider(at column half_w)
 */
static void paint_layout_split(const Scene *s, int cols, int rows)
{
    int drawable_h = rows - HUD_BAND_RESERVED_ROWS;
    int half_w     = (cols - PAINT_GUTTER_WIDTH) / 2;

    Rect left  = { 0,                          HUD_TOP_ROWS, half_w, drawable_h };
    Rect right = { half_w + PAINT_GUTTER_WIDTH, HUD_TOP_ROWS, half_w, drawable_h };

    paint_attractor_panel(s, &left,  0, 1);    /* trajectory A only */
    paint_attractor_panel(s, &right, 1, 1);    /* trajectory B only */
    paint_vertical_divider(half_w, HUD_TOP_ROWS, drawable_h);
}

/* log_plot_band_height — height (in cells) of the bottom log|δ| band
 * in PLOT_BOTTOM / TRIO_LOG layouts.  ~40% of drawable, named so the
 * fraction lives in §1 constants. */
static inline int log_plot_band_height(int drawable_h)
{
    return drawable_h * LOG_PLOT_BAND_NUMERATOR / LOG_PLOT_BAND_DENOMINATOR;
}

/* paint_layout_plot_bottom — attractor overlay on top (~60%), log|δ|
 * plot on bottom (~40%), horizontal divider between.
 *
 * DRIVER PSEUDOCODE
 *   drawable_h  = rows - HUD_BAND_RESERVED_ROWS
 *   plot_h      = log_plot_band_height(drawable_h)
 *   attr_h      = drawable_h - plot_h - PAINT_DIVIDER_ROWS
 *   attr_rect   = top    band of `attr_h` rows
 *   plot_rect   = bottom band of `plot_h` rows
 *   paint_attractor_panel(attr_rect, all trajectories)
 *   paint_log_plot       (plot_rect, log_floor for active.eps)
 *   paint_horizontal_divider(between the two)
 */
static void paint_layout_plot_bottom(const Scene *s, int cols, int rows)
{
    const SDPreset *active = scene_active_preset(s);
    int drawable_h = rows - HUD_BAND_RESERVED_ROWS;
    int plot_h     = log_plot_band_height(drawable_h);
    int attr_h     = drawable_h - plot_h - PAINT_DIVIDER_ROWS;

    Rect attr_rect = { 0, HUD_TOP_ROWS,
                       cols, attr_h };
    Rect plot_rect = { 0, HUD_TOP_ROWS + attr_h + PAINT_DIVIDER_ROWS,
                       cols, plot_h };

    paint_attractor_panel(s, &attr_rect, 0, TRAJ_MAX);
    paint_log_plot(&s->log_delta,
                   plot_rect.gx0, plot_rect.gy0, plot_rect.w, plot_rect.h,
                   log_floor_for_eps(active->eps));
    paint_horizontal_divider(HUD_TOP_ROWS + attr_h, cols);
}

/* paint_layout_plot_only — log|δ| plot fills the drawable band.
 *
 * DRIVER PSEUDOCODE
 *   rect = drawable_band_full(cols, rows)
 *   paint_log_plot(rect, log_floor for active.eps)
 */
static void paint_layout_plot_only(const Scene *s, int cols, int rows)
{
    const SDPreset *active = scene_active_preset(s);
    Rect r = drawable_band_full(cols, rows);
    paint_log_plot(&s->log_delta, r.gx0, r.gy0, r.w, r.h,
                   log_floor_for_eps(active->eps));
}

/* scene_paint — dispatch on the active layout.
 *
 * DRIVER PSEUDOCODE
 *   switch active.layout:
 *       FULL        → paint_layout_full
 *       SPLIT       → paint_layout_split
 *       PLOT_BOTTOM → paint_layout_plot_bottom
 *       PLOT_ONLY   → paint_layout_plot_only
 */
static void scene_paint(const Scene *s, int cols, int rows)
{
    switch (scene_active_preset(s)->layout) {
        case LAYOUT_FULL:        paint_layout_full       (s, cols, rows); break;
        case LAYOUT_SPLIT:       paint_layout_split      (s, cols, rows); break;
        case LAYOUT_PLOT_BOTTOM: paint_layout_plot_bottom(s, cols, rows); break;
        case LAYOUT_PLOT_ONLY:   paint_layout_plot_only  (s, cols, rows); break;
    }
}

/* hud_write_title — left-aligned bold "SENSITIVE DEPENDENCE (Lorenz)". */
static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " SENSITIVE DEPENDENCE (Lorenz) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_write_status_right — right-aligned status text on row 0
 * (fps, sim_fps, preset name + [n/N], sim time, current |δ|). */
static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const SDPreset *active = scene_active_preset(s);
    float d = delta_norm(&s->traj[0].state, &s->traj[1].state);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  t:%.1fs  |δ|:%.2e ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, (int)N_PRESETS,
             (double)s->t_sim, (double)d);

    int hx = cols - (int)strlen(buf); if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row 0 of the HUD.
 *
 * DRIVER PSEUDOCODE
 *   hud_write_title()                       // left-aligned banner
 *   hud_write_status_right(...)             // right-aligned status
 */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_write_title();
    hud_write_status_right(cols, fps, sim_fps, s);
}

/* hud_write_preset_label / _theme_label / _eps_traj_lambda — three
 * column segments of row 1, each a one-line "label : value" pair. */
static inline void hud_write_preset_label(int x, const SDPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", active->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_theme_label(int x, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_eps_traj_lambda(int x, const SDPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ε:%.0e  n:%d  Lorenz λ≈%.3f ",
             (double)active->eps, active->n_traj, (double)LORENZ_LAMBDA);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_param — row 1 of the HUD: three labelled column segments.
 *
 * DRIVER PSEUDOCODE
 *   x = HUD_LEFT_MARGIN
 *   hud_write_preset_label    (x, active);  x += HUD_PARAM_PRESET_WIDTH
 *   hud_write_theme_label     (x, scene);   x += HUD_PARAM_THEME_WIDTH
 *   hud_write_eps_traj_lambda (x, active)
 */
static void hud_param(const Scene *s)
{
    const SDPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label   (x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label    (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_eps_traj_lambda(x, active);
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
 *   erase()                                 // clear back buffer
 *   scene_paint(scene, cols, rows)          // layout dispatch
 *   hud_top   (cols, fps, sim_fps, scene)   // banner + status
 *   hud_param (scene)                       // preset + theme + params
 *   hud_hint  (rows)                        // key hints
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top   (sc->cols, fps, sim_fps, s);
    hud_param (s);
    hud_hint  (sc->rows);
}

/* ===================================================================== */
/* §10 app — signals, resize, key dispatch, FrameClock, main loop        */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   The "everything else" container.  Owns the simulation (Scene),
 *   the viewport (Screen), the simulation-rate knob (sim_fps), and
 *   the two volatile signal-handler flags.  Lives as a file-scope
 *   g_app so signal handlers can touch it without indirection.
 *
 * MEMBER LOGIC
 *   scene       : the simulation (§8 Scene).
 *   screen      : cached terminal dimensions (§9 Screen).
 *   sim_fps     : fixed-timestep rate scene_tick is driven at;
 *                 clamped to [SIM_FPS_MIN, SIM_FPS_MAX] = [10, 240].
 *   running     : main-loop guard; cleared by SIGINT/TERM or 'q'.
 *   need_resize : SIGWINCH flag; consumed by handle_pending_resize.
 *                 Both flags MUST be volatile sig_atomic_t.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
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

/* app_bootstrap — first-time initialisation. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * SIM_MAX_FRAME_DT_MS so a long stall doesn't spiral the integrator. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler [8] accumulator pattern.  Runs
 * scene_tick at the chosen sim_fps regardless of the render rate, so
 * BOTH trajectories integrate at the same wall-clock-independent
 * fixed step (essential — sensitive dependence requires identical
 * dt or the divergence would be from numerics, not chaos). */
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
static void app_toggle_pause     (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_attractor  (App *app) { scene_reset(&app->scene); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene);   /* per-preset ε requires re-seeding */
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene);
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
    case 'r': case 'R':  app_reset_attractor  (app); break;
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
 *   frame_count, fps_display) into ONE named concept so main() reads
 *   as a pseudocode driver.  Threading them through helpers as a
 *   single FrameClock* removes 5 separate parameter lists.
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock ns at the START of the previous frame.
 *   sim_accum   : ns of unspent simulation time, drained in multiples
 *                 of TICK_NS(sim_fps) by app_drain_fixed_timestep.
 *   fps_accum   : ns elapsed since the last fps recalculation.
 *   frame_count : frames rendered since the last fps recalc.
 *   fps_display : displayed fps, refreshed every FPS_UPDATE_MS.
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

/* main — the whole simulation as a pseudocode driver. */
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
