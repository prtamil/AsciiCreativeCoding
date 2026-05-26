/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * standard_map.c
 *   — Chirikov-Taylor "Standard Map".  The canonical visualisation of
 *     HAMILTONIAN (conservative) chaos: a 2-D area-preserving map on
 *     a torus that, depending on a single parameter K, shows either
 *     concentric KAM tori (regular orbits) or a chaotic sea with
 *     islands of regular motion inside it.
 *
 * DEMO: For each of NUM_TRAJECTORIES = 900 initial conditions
 *       (uniformly spaced on the torus), repeatedly apply
 *         p_{n+1} = p_n + K · sin(x_n)              (mod 2π)
 *         x_{n+1} = x_n + p_{n+1}                   (mod 2π)
 *       and accumulate the visited cells into a density grid.  Each
 *       cell's brightness = log(count), mapped onto a 10-band ×
 *       10-glyph gradient.  Regular trajectories trace thin curves
 *       (high density along a 1-D path); chaotic trajectories spread
 *       thin across a 2-D region.
 *
 *       5 presets, chosen for maximum visual contrast — each
 *       produces a distinctly different settled picture.  Cycle
 *       with n / p:
 *
 *         1  INTEGRBL  K=0.00     horizontal stripes (pure rotation)
 *         2  RIPPLES   K=0.20     all KAM tori, gentle wiggles
 *         3  WAVES     K=0.40     clear island chains forming
 *         4  CHAOS_L   K=2.50     classic chaotic sea + islands
 *         5  STORM     K=5.00     chaos dominant, scattered islands
 *
 * Study alongside:
 *   ./bifurcation.c          — DISSIPATIVE 1-D map chaos (logistic).
 *   ./double_pendulum.c      — CONTINUOUS Hamiltonian chaos.
 *   ./poincare_section.c     — same idea (sample-and-plot) but
 *                              applied to a continuous-time ODE.
 *
 * Section map:
 *   §1  config    — constants, 5-preset table, themes, density gradient
 *   §2  clock     — monotonic timer + sleep
 *   §3  color     — HUD pairs + 10 themes (PAIR_DENS_BASE+0..9)
 *   §5  physics   — ChirikovSystem + ChirikovState + iterate
 *   §6  density   — DensityGrid (uint16 hit-counter per cell)
 *   §7  state     — TrajectoryEnsemble + PresetState + PaletteState
 *   §8  scene     — Scene composite (Ensemble + DensityGrid + selections)
 *   §9  screen    — density paint + HUD writers
 *   §10 app       — signals, resize, key dispatch, FrameClock, main
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (clear density, re-init trajectories)
 *   n / N      next preset    (p / P previous)
 *   t / T      next / previous theme
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/standard_map.c \
 *       -o standard_map -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Chirikov-Taylor (standard) map.  An exactly
 *                 area-preserving (symplectic) map on the 2-torus
 *                 T² = [0, 2π)² with one bifurcation parameter K.
 *                 NUM_TRAJECTORIES independent initial conditions are
 *                 iterated in lock-step; each visited cell of a fixed
 *                 grid increments a uint16 density counter.
 *
 * Data-struct   : DensityGrid (uint16_t × CELLS_MAX) — small enough
 *                 for BSS, large enough to count millions of hits
 *                 before saturating.  TrajectoryEnsemble = one
 *                 shared ChirikovSystem (K) + NUM_TRAJECTORIES
 *                 ChirikovStates (x, p).  No other history is kept
 *                 (the DensityGrid IS the history).
 *
 * Rendering     : Density → log-mapped band ∈ [0, DENS_BAND_COUNT)
 *                 → palette colour pair × glyph tier.  10 bands × 10
 *                 glyphs ('. , : ; + o * X # @') = 100 visually
 *                 distinct intensity levels.  Phase space (x, p) ∈
 *                 [0, 2π)² fills the drawable area; (0, 0) is the
 *                 bottom-left corner (y inverted in phase_to_cell so
 *                 increasing p draws upward, textbook orientation).
 *                 Auto-stops at MAX_ITERATIONS_PER_TRAJ so each
 *                 preset settles to a stable picture.
 *
 * Performance   : Per tick: NUM_TRAJECTORIES × ITERATES_PER_TRAJ_PER_TICK
 *                 sin() + 2 mods + 1 density increment per iterate.
 *                 At 256 trajectories × 30 iterates × 60 Hz ≈ 460K
 *                 sin/sec — trivial.  Total work bounded by
 *                 MAX_ITERATIONS_PER_TRAJ so the loop quiesces.
 *
 * References    : Numbered so inline code can cite [n].
 *
 *   PHYSICS / DYNAMICAL SYSTEMS
 *   [1] Chirikov, B. V. (1979).  "A universal instability of many-
 *       dimensional oscillator systems", Physics Reports 52(5),
 *       pp. 263-379.  THE original paper: defines the standard
 *       map p' = p + K·sin(x), x' = x + p' and identifies a
 *       global-stochasticity threshold near K ≈ 1.
 *   [2] Greene, J. M. (1979).  "A method for determining a
 *       stochastic transition", J. Math. Phys. 20, pp. 1183-1201.
 *       Calculates the precise threshold K_c = 0.9716354 at which
 *       the last golden-ratio KAM torus breaks — the value
 *       hard-coded in KC_GREENE and shown in the HUD.
 *   [3] Lichtenberg, A. J., Lieberman, M. A. (1992).  *Regular and
 *       Chaotic Dynamics* (2nd ed.), §4.1.  Standard textbook
 *       treatment of the kicked rotor → standard map derivation,
 *       and §3.2 for the KAM theorem in this setting.
 *   [4] Arnold, V. I. (1989).  *Mathematical Methods of Classical
 *       Mechanics* (2nd ed.), §53.  Kolmogorov-Arnold-Moser
 *       theorem: most quasi-periodic tori survive a small
 *       Hamiltonian perturbation.  The standard map IS the
 *       canonical test case.
 *   [5] Strogatz, S. H. (2015).  *Nonlinear Dynamics and Chaos*
 *       (2nd ed.), §12.  Pedagogical companion text — though
 *       Strogatz focuses on dissipative chaos, his treatment of
 *       phase-space methods backs the visualisation approach.
 *
 *   FRAMEWORK
 *   [6] Fiedler, G. (2004).  "Fix Your Timestep!",
 *       gafferongames.com.  Accumulator pattern used by
 *       app_drain_fixed_timestep so the integrator runs at a
 *       fixed sim_fps regardless of render rate.
 *   [7] Sussman, G. J., Wisdom, J. (2014).  *Structure and
 *       Interpretation of Classical Mechanics* (2nd ed.), §1.6
 *       "Coordinate systems and states".  The System / State /
 *       Composite split used in §5 (ChirikovSystem +
 *       ChirikovState + the ensemble layout).
 *
 *   COMPARE IN PROJECT
 *   • ./bifurcation.c     — dissipative 1-D map chaos.
 *   • ./double_pendulum.c — continuous-time Hamiltonian.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * "Kicked rotor": a free rotor receiving an impulsive sin-shaped
 * kick once per period.  Between kicks the momentum p is constant
 * and the angle x advances by p.  At each kick p jumps by K·sin(x).
 * No friction → exactly area-preserving → trajectories cover
 * 2-D regions but never spirals.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * K = 0: completely integrable.  Every trajectory lies on a
 *        horizontal line (p constant).
 * K small: KAM tori survive (Arnold [4] §53) — each trajectory
 *        still traces a 1-D invariant curve, just deformed.
 * K ≈ 0.9716 (Greene [2]): the GOLDEN-RATIO KAM torus breaks;
 *        chaotic trajectories can now wander vertically across
 *        the entire torus.
 * K large: only ISLAND tori around stable periodic orbits survive;
 *        the rest is a connected chaotic sea.
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────
 *  repeat ITERATES_PER_TRAJ_PER_TICK times:
 *    for each ChirikovState s in ensemble.state[]:
 *      s = chirikov_step(s, ensemble.system)       // (mod 2π wrap inside)
 *      density_record(grid, s)                      // ++ hit count
 *  stop when iterations_done ≥ MAX_ITERATIONS_PER_TRAJ.
 *
 * KEY FORMULAS  (Chirikov 1979 [1], Lichtenberg & Lieberman [3] §4.1)
 * ────────────
 *  p_{n+1} = p_n + K · sin(x_n)         (mod 2π)
 *  x_{n+1} = x_n + p_{n+1}              (mod 2π)
 *
 *  Jacobian determinant = 1 ↔ area-preserving (the defining
 *  property of a symplectic / Hamiltonian map).
 *
 *  K_c = 0.9716354 (Greene [2]) — threshold above which the
 *  golden-ratio invariant torus breaks and chaotic trajectories
 *  can wander unboundedly in p.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • INTEGRBL (K = 0.00):  16 perfectly horizontal stripes (one per
 *    starting p-value on the 16×16 grid) — no kick, no chaos.
 *  • RIPPLES  (K = 0.20):  every stripe deformed into a gentle sine
 *    — all KAM tori intact, well below Greene's threshold.
 *  • WAVES    (K = 0.40):  clear island chains appear around the
 *    elliptic fixed point at (x, p) = (π, 0); separatrix-like
 *    figure-eight structures emerge; no large fuzzy region yet.
 *  • CHAOS_L  (K = 2.50):  a fuzzy chaotic sea fills most of the torus
 *    with several visible "island chains" of regular tori around
 *    period-N points — the iconic standard-map silhouette.
 *  • STORM    (K = 5.00):  chaos dominant, only small period-N islands
 *    around stable orbits survive.
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
#define TAU (2.0f * (float)M_PI)

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
    PAIR_DENS_BASE    =   3,   /* DENS_BAND_COUNT slots: +0..+9 */
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

/* Trajectory ensemble — uniformly spaced grid of starting conditions.
 *
 * 16 × 16 = 256 trajectories.  Counter-intuitively, FEWER orbits
 * iterated MORE times gives the cleanest textbook picture: each
 * trajectory has time to trace its full KAM curve (~500-1000 points)
 * before the others overdraw it.  With 900+ trajectories the curves
 * blend into a uniform fog and every preset looks the same. */
#define TRAJ_GRID_W              16
#define TRAJ_GRID_H              16
#define NUM_TRAJECTORIES         (TRAJ_GRID_W * TRAJ_GRID_H)

/* Iterations per trajectory per tick.  At 30 × 60 Hz = 1800
 * iterations/sec per trajectory, a full KAM curve traces in ~½
 * second.  900K total iterations/sec is still trivial. */
#define ITERATES_PER_TRAJ_PER_TICK   30

/* Auto-stop budget — total iterations PER TRAJECTORY at which the
 * picture is "done" and further iteration would just saturate cells.
 * 6000 iterations × 60 Hz / 30-per-tick ≈ 3.3 simulated seconds.
 * After this, scene_tick stops iterating so the user sees a clean
 * settled image instead of a slowly-saturating blur.  Press 'r' to
 * redo from the same initial conditions. */
#define MAX_ITERATIONS_PER_TRAJ  6000

/* Density gradient — 10 bands × 10 glyph tiers = 100 visually
 * distinct brightness levels.  Saturation log chosen so a typical
 * dense KAM curve reaches the top band (~5000 hits at MAX iter
 * budget) and the chaotic sea sits in the mid-bands instead of
 * uniformly saturating. */
#define DENS_BAND_COUNT          10
#define DENS_SATURATE_LOG         8.5f   /* log(~5000) at top band */

/* Greene's golden-mean KAM threshold — last invariant torus breaks
 * at K = K_c.  Below this all orbits are bounded vertically; above,
 * a connected chaotic sea spans the torus.  This value is the
 * principal numerical result of Greene 1979 [2] and is hard-coded
 * here so the HUD can show "BELOW Kc" / "ABOVE Kc" annotations. */
#define KC_GREENE                 0.9716354f

/* KC_DEAD_BAND — half-width of the "AT Kc" zone in the HUD regime
 * annotation.  Any K within ±KC_DEAD_BAND of KC_GREENE renders as
 * "AT Kc"; this keeps slight float drift from flickering the label. */
#define KC_DEAD_BAND              0.01f

/* HUD row-1 column widths — fixed so the three labelled segments
 * align across all 5 presets and 10 themes. */
#define HUD_PARAM_PRESET_WIDTH   19
#define HUD_PARAM_THEME_WIDTH    17

/* Density grid minimum dimensions — degenerate sizes (too narrow or
 * too short) get clamped UP to these floors so the grid stays usable
 * on tiny terminals.  CELLS_MAX caps the other direction. */
#define MAP_W_FLOOR              16
#define MAP_H_FLOOR               8

/*
 * Preset — the 5-entry index space for the presets[] table.
 *
 * INTENT
 *   Named indices into the presets[] table.  K values chosen for
 *   maximum visual contrast — each one produces a distinctly
 *   different settled picture so cycling 'n' walks the user from
 *   pure rotation (K=0) past the KAM tori regime, across Greene's
 *   threshold [2], into the chaotic sea, and finally into the
 *   chaos-dominant strong-K regime.
 *
 * CONTEXT
 *   Used only as table indices; entries themselves live in the
 *   presets[] static array below.  PresetState (§7) wraps a single
 *   instance and cycles modulo N_PRESETS.  PRESET_WAVES is the boot-
 *   state default chosen by scene_init — middle of the cascade, a
 *   good anchor that immediately shows non-trivial KAM structure.
 *
 * MEMBER LOGIC
 *   PRESET_INTEGRABLE : K = 0.00.  No kick; orbit p stays constant
 *                       forever; renders as 16 horizontal stripes
 *                       (one per starting p value on the 16×16 grid).
 *   PRESET_RIPPLES    : K = 0.20.  All KAM tori intact (well below
 *                       Greene threshold [2]); curves are gently
 *                       deformed sinusoidals.
 *   PRESET_WAVES      : K = 0.40.  First clear island chains appear
 *                       around the elliptic fixed points.
 *   PRESET_CHAOS_L    : K = 2.50.  Classic chaotic sea + regular
 *                       islands — the iconic standard-map picture.
 *   PRESET_STORM      : K = 5.00.  Chaos dominant; only small period-N
 *                       islands around stable orbits survive.
 *
 * REFERENCES
 *   [1] Chirikov 1979 — the canonical K parameterisation.
 *   [2] Greene 1979 — KC_GREENE = 0.9716 sits between WAVES and CHAOS_L.
 */
typedef enum {
    PRESET_INTEGRABLE = 0,
    PRESET_RIPPLES,
    PRESET_WAVES,
    PRESET_CHAOS_L,
    PRESET_STORM,
    N_PRESETS,
} Preset;

/*
 * MapPreset — one row of the K-parameter zoo.
 *
 * INTENT
 *   Each row fully describes one entry in the catalogue: HUD label
 *   and the single Chirikov coupling K that defines the demo.  K is
 *   the ONLY parameter of the standard map — there are no sweep
 *   variables, no per-row tuning knobs, just K.
 *
 * CONTEXT
 *   Read by scene_load_active_preset_into_system (loads K into the
 *   ensemble's shared ChirikovSystem), by the HUD writers (display
 *   name + K + threshold annotation), and by the §5 docs.  The
 *   PresetState wrapper in §7 holds an INDEX into this table.
 *
 * MEMBER LOGIC
 *   name : 8-char HUD label, padded so the parameter column aligns.
 *   K    : Chirikov coupling [1].  Compare to KC_GREENE [2] to know
 *          whether the chaotic region is bounded (K < Kc) or globally
 *          connected across the torus (K > Kc).
 *
 * REFERENCES
 *   [1] Chirikov 1979 — parameter naming.
 *   [2] Greene 1979 — significance of the threshold.
 */
typedef struct {
    const char *name;
    float       K;
} MapPreset;

static const MapPreset presets[N_PRESETS] = {
    { "INTEGRBL", 0.00f },   /* horizontal stripes (no kick)     */
    { "RIPPLES ", 0.20f },   /* all KAM tori, gentle wiggles     */
    { "WAVES   ", 0.40f },   /* clear island chains forming      */
    { "CHAOS_L ", 2.50f },   /* classic chaotic sea + islands    */
    { "STORM   ", 5.00f },   /* sea dominant, scattered islands  */
};

/*
 * Theme — one named palette for the density renderer.
 *
 * INTENT
 *   Group the colour codes that define a density picture's visual
 *   identity into ONE flat row, so a "next theme" key cycles a
 *   single table.  Standard map is special among the chaos demos
 *   because its renderer needs ONE monotone-brightness ramp from
 *   sparsest visited cells (band 0) to most-visited cells (band 9);
 *   the ramp's hue family defines the theme.
 *
 * CONTEXT
 *   Indexed by PaletteState (§7); installed by theme_apply (§3)
 *   which pushes each band[i] into PAIR_DENS_BASE + i.  Cycled by
 *   t/T via app_cycle_theme_next/prev.  All palette values follow
 *   the project Brightness Rule (CLAUDE.md): every code lives in
 *   the bright half of the 256-colour cube so even band 0 is
 *   legible on default black.
 *
 * MEMBER LOGIC
 *   name    : 7-char HUD label.
 *   band[]  : DENS_BAND_COUNT (10) 256-colour codes, sorted
 *             dimmest → brightest.  density_paint reads index b
 *             returned by density_band() and looks up band[b].
 *
 * REFERENCES
 *   CLAUDE.md "Theme Palette Brightness".
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];   /* density ramp: low → high */
} Theme;

#define N_THEMES 10

/* Density ramps — monotone-brightness 10-step palettes, all in the
 * bright half of the 256-colour cube (≥ 30 / ≥ 244) per CLAUDE.md so
 * even band 0 is visible.  Each ramp climbs dimmest → brightest left
 * → right so band index directly tracks visual brightness.  Hue stays
 * within each theme's family so the name is honest. */
static const Theme themes[N_THEMES] = {
    /* name        band0  b1   b2   b3   b4   b5   b6   b7   b8   b9 */
    { "DEFAULT", {  31,  39,  45,  81, 117, 153, 189, 220, 226, 231 } },
    { "MATRIX",  {  28,  34,  40,  46,  82, 118, 154, 190, 226, 228 } },
    { "NOVA",    {  93,  99, 135, 171, 207, 213, 219, 225, 227, 231 } },
    { "MONO",    { 240, 244, 247, 249, 251, 252, 253, 254, 255, 231 } },
    { "OCEAN",   {  24,  31,  38,  45,  81, 117, 159, 195, 225, 231 } },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 214, 220, 226, 228 } },
    { "EARTH",   {  94, 130, 137, 143, 178, 215, 222, 228, 230, 231 } },
    { "FOREST",  {  28,  34,  64, 100, 107, 113, 149, 185, 191, 228 } },
    { "DESERT",  { 130, 166, 173, 179, 215, 222, 228, 230, 231,  255 } },
    { "ARCTIC",  {  24,  31,  45,  51,  81, 117, 159, 195, 225, 231 } },
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
    struct timespec req = { .tv_sec = ns / NS_PER_SEC,
                            .tv_nsec = ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/* theme_install_256 — push a Theme's DENS_BAND_COUNT bright-cube
 * codes into the ncurses pair table (PAIR_DENS_BASE + 0 .. + 9). */
static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
}

/* theme_install_8color_fallback — graceful degradation for the few
 * terminals that advertise only 8 colours.  Cycles the basic ANSI
 * palette dim → bright across the 10 density bands. */
static inline void theme_install_8color_fallback(void)
{
    static const short fb[DENS_BAND_COUNT] = {
        COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,
        COLOR_GREEN,   COLOR_YELLOW,  COLOR_YELLOW,  COLOR_MAGENTA,
        COLOR_RED,     COLOR_WHITE,
    };
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, fb[i], -1);
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
static void color_init(void)
{
    start_color(); use_default_colors();
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
/* §5  physics — Chirikov standard map: system + state + iterate          */
/* ===================================================================== */

/*
 * ChirikovSystem — invariant parameter(s) of the map [1].
 *
 * INTENT
 *   Split the kick strength K away from the state so chirikov_step is
 *   a pure function of (state, system) — same shape as the Lorenz /
 *   Rössler composites in this project even though there's only one
 *   parameter.  Consistency makes the §5 layout instantly familiar
 *   to readers who've studied another chaos demo (Sussman & Wisdom
 *   [7] §1.6).
 *
 * MEMBER LOGIC
 *   K : Chirikov coupling.  Bifurcation parameter — below KC_GREENE
 *       all chaotic motion is bounded vertically in p (KAM theorem,
 *       Arnold [4] §53); above it the last invariant golden-ratio
 *       torus breaks (Greene [2]) and chaos spans the whole torus
 *       T² = [0, 2π)².
 *
 * REFERENCES
 *   [1] Chirikov 1979 — original definition.
 *   [3] Lichtenberg & Lieberman §4.1 — kicked-rotor derivation.
 */
typedef struct { float K; } ChirikovSystem;

/*
 * ChirikovState — one orbit's phase-point (x, p) ∈ T² = [0, 2π)².
 *
 * INTENT
 *   Pair the angle x and the momentum p as a single named value so
 *   chirikov_step takes ONE arg instead of two float pointers, and
 *   so the trajectory ensemble can store an array of states as
 *   array-of-struct rather than two parallel float arrays.
 *
 * MEMBER LOGIC
 *   x : angle, [0, 2π).  Wraps via wrap_tau after each step.
 *   p : momentum, [0, 2π).  Wraps via wrap_tau after each step.
 *
 * REFERENCES
 *   [3] Lichtenberg & Lieberman §4 — coordinates of the kicked rotor.
 */
typedef struct { float x, p; } ChirikovState;

/* wrap_tau — wrap a real number into [0, 2π).  fmodf can return
 * negatives for negative inputs, so add TAU once if needed. */
static inline float wrap_tau(float v)
{
    v = fmodf(v, TAU);
    if (v < 0.0f) v += TAU;
    return v;
}

/* chirikov_step — one iterate of the standard map.  Chirikov 1979 [1]
 * §I, Lichtenberg & Lieberman [3] §4.1; each line of the body maps
 * 1-to-1 to one published formula:
 *
 *   p_{n+1} = p_n + K · sin(x_n)   (mod 2π)    momentum kick
 *   x_{n+1} = x_n + p_{n+1}        (mod 2π)    free rotation
 *
 * Pure function: returns the next state without mutating its inputs.
 * The Jacobian determinant of this map is 1 — area-preserving, the
 * defining property of a symplectic (Hamiltonian) map [4] §53. */
static inline ChirikovState chirikov_step(ChirikovState s, ChirikovSystem sys)
{
    s.p = wrap_tau(s.p + sys.K * sinf(s.x));
    s.x = wrap_tau(s.x + s.p);
    return s;
}

/* ===================================================================== */
/* §6  density — accumulating hit-counter grid                            */
/* ===================================================================== */

/*
 * DensityGrid — uint16 hit-counter per phase-space cell.
 *
 * INTENT
 *   The "memory" of the simulation: which (x, p) cells of the torus
 *   the trajectories have visited, and how many times.  Log-mapping
 *   the count onto a colour band turns this counter array into a
 *   density-of-visits picture — KAM curves show as bright 1-D
 *   filaments, chaotic regions as 2-D textured clouds.
 *
 * CONTEXT
 *   Written by density_record (one increment per chirikov_step).
 *   Read by density_paint (§9) which log-maps hits → band → glyph.
 *   Sized to the terminal at startup and on SIGWINCH; cell count is
 *   bounded by CELLS_MAX so the array lives in BSS without malloc.
 *
 * MEMBER LOGIC
 *   w, h  : current viewport dimensions in cells (set by density_reset).
 *   count : w × h, cached so loops don't multiply each frame.
 *   hits  : flat row-major buffer of hit counts.  Saturates at
 *           UINT16_MAX = 65535 (which log-maps well below the top
 *           band, so saturation is invisible in practice).
 *
 * REFERENCES
 *   [3] Lichtenberg & Lieberman §4 — phase-space density
 *       visualisation of area-preserving maps.
 */
typedef struct {
    int      w, h;
    int      count;
    uint16_t hits[CELLS_MAX];
} DensityGrid;

static void density_reset(DensityGrid *d, int w, int h)
{
    d->w = w; d->h = h; d->count = w * h;
    memset(d->hits, 0, sizeof(uint16_t) * (size_t)d->count);
}

/* phase_to_cell — map a (x, p) phase point onto the (sx, sy) cell of
 * the density grid.  Y is INVERTED so larger p draws further up on
 * screen, matching textbook orientation. */
static inline void phase_to_cell(const DensityGrid *d,
                                 float x, float p,
                                 int *cx, int *cy)
{
    int x_cell = (int)((x / TAU) * (float)d->w);
    int y_cell = (int)((p / TAU) * (float)d->h);
    if (x_cell < 0)        x_cell = 0;
    if (x_cell >= d->w)    x_cell = d->w - 1;
    if (y_cell < 0)        y_cell = 0;
    if (y_cell >= d->h)    y_cell = d->h - 1;
    *cx = x_cell;
    *cy = d->h - 1 - y_cell;            /* invert y for textbook orientation */
}

/* density_record — increment the count at one phase point. */
static inline void density_record(DensityGrid *d, ChirikovState s)
{
    int cx, cy;
    phase_to_cell(d, s.x, s.p, &cx, &cy);
    int idx = cy * d->w + cx;
    if (d->hits[idx] < UINT16_MAX) d->hits[idx]++;
}

/* density_band — log-scale count → band ∈ [0, DENS_BAND_COUNT-1].
 * Returns −1 for unvisited cells so density_paint can skip them. */
static inline int density_band(uint16_t hits)
{
    if (hits == 0) return -1;
    float v = logf((float)hits) / DENS_SATURATE_LOG;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    int b = (int)(v * (float)(DENS_BAND_COUNT - 1) + 0.5f);
    if (b > DENS_BAND_COUNT - 1) b = DENS_BAND_COUNT - 1;
    return b;
}

/* ===================================================================== */
/* §7  state — TrajectoryEnsemble + PresetState + PaletteState           */
/* ===================================================================== */

/*
 * TrajectoryEnsemble — NUM_TRAJECTORIES parallel orbits sharing ONE
 * Chirikov system.
 *
 * INTENT
 *   All orbits in this demo use the SAME K (changes only when the
 *   user cycles a preset), so the system is stored once at the
 *   ensemble level and the per-orbit state is an array of
 *   ChirikovState.  This makes the "shared physics, varied initial
 *   conditions" structure explicit at the type level — exactly what
 *   the standard-map demo is about.
 *
 * CONTEXT
 *   Owned by Scene.  Reset on r/n/p: ensemble_init_uniform_grid
 *   places trajectories on a regular TRAJ_GRID_W × TRAJ_GRID_H
 *   lattice of starting (x, p) ∈ [0, 2π)².  Iterated by scene_tick
 *   in lock-step (every orbit advances by ITERATES_PER_TRAJ_PER_TICK
 *   steps per tick).
 *
 * MEMBER LOGIC
 *   system   : the shared (K) parameter, loaded from the active
 *              preset by scene_apply_preset_to_system.
 *   state[]  : per-orbit (x, p) phase-points.  Stored as an
 *              array-of-struct (AoS) so chirikov_step takes one
 *              named value — cleaner than parallel float arrays.
 *
 * REFERENCES
 *   [1] Chirikov 1979; [3] Lichtenberg & Lieberman §4.
 */
typedef struct {
    ChirikovSystem system;
    ChirikovState  state[NUM_TRAJECTORIES];
} TrajectoryEnsemble;

/* ensemble_init_uniform_grid — place NUM_TRAJECTORIES orbits on an
 * evenly-spaced lattice of starting (x, p) inside (0, 2π)² with a
 * one-cell margin so the corners aren't artificially dense.  Run on
 * every reset / preset cycle.  K is loaded separately. */
static void ensemble_init_uniform_grid(TrajectoryEnsemble *e)
{
    int idx = 0;
    for (int j = 0; j < TRAJ_GRID_H; j++) {
        float fy = (float)(j + 1) / (float)(TRAJ_GRID_H + 1);
        for (int i = 0; i < TRAJ_GRID_W; i++) {
            float fx = (float)(i + 1) / (float)(TRAJ_GRID_W + 1);
            e->state[idx].x = fx * TAU;
            e->state[idx].p = fy * TAU;
            idx++;
        }
    }
}

/*
 * PresetState — typed wrapper around "which K-preset is loaded".
 *
 * INTENT
 *   Wrap the bare Preset enum so the key-binding table reads as
 *   intentions ("cycle to next preset") rather than modular
 *   arithmetic on N_PRESETS.  Typing prevents a t/T keystroke from
 *   accidentally cycling the presets[] table.  Replace-Primitive-
 *   with-Object pattern (Fowler, *Refactoring*).
 *
 * MEMBER LOGIC
 *   current : index into presets[].  Must be in [0, N_PRESETS).
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const MapPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState; distinct type so a t/T keystroke
 *   physically cannot reach the presets[] table.  Bundles the
 *   active index plus the helper that re-pushes the theme to the
 *   ncurses pair table.  Replace-Primitive-with-Object pattern.
 *
 * CONTEXT
 *   Owned by Scene; mutated via palette_state_cycle_next/prev (t/T
 *   keys) followed immediately by scene_apply_theme, which pushes
 *   the resulting band[] codes back into the ncurses pair table so
 *   the next frame paints with the new palette.
 *
 * MEMBER LOGIC
 *   current : index into themes[].  Must be in [0, N_THEMES) =
 *             [0, 10); cycle helpers maintain this invariant via
 *             modular arithmetic on N_THEMES.
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
/* §8  scene — composite owner of all mutable simulation state           */
/* ===================================================================== */

/*
 * Scene — composition root.
 *
 * INTENT
 *   Bundle every piece of mutable state into ONE struct so App owns
 *   one Scene and the main loop drives it through a tiny named-
 *   method API.  Each sub-field is its own typed concept so the
 *   compiler enforces the boundary between physics (the trajectory
 *   ensemble), memory (the density grid), and selection (preset,
 *   palette).
 *
 * CONTEXT
 *   Owned by App as a value.  Accessed by §9 painters and §10 app
 *   helpers.  Auto-stops after MAX_ITERATIONS_PER_TRAJ iterations
 *   per trajectory so the picture settles to a stable image.
 *
 * MEMBER LOGIC
 *   density          : §6 DensityGrid — the visible hit-count picture.
 *   ensemble         : §7 TrajectoryEnsemble — shared K + per-orbit
 *                      ChirikovState[NUM_TRAJECTORIES].
 *   preset           : §7 PresetState — index into presets[].
 *   palette          : §7 PaletteState — index into themes[].
 *   paused           : when true, scene_tick early-returns.
 *   iterations_done  : per-trajectory iteration counter.  When this
 *                      reaches MAX_ITERATIONS_PER_TRAJ the picture is
 *                      "done" and scene_tick stops iterating.
 *
 * REFERENCES
 *   [7] Sussman & Wisdom *SICM* §1.6 — System / State / Composite
 *       split this layer mirrors at the simulation-root level.
 */
typedef struct {
    DensityGrid        density;
    TrajectoryEnsemble ensemble;
    PresetState        preset;
    PaletteState       palette;
    bool               paused;
    int                iterations_done;
} Scene;

/* Accessors so call sites don't repeat presets[s->preset.current]. */
static inline const MapPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* scene_load_active_preset_into_system — copy the active preset's K
 * onto the ensemble's shared system.  Called on reset and on preset
 * cycle so chirikov_step sees the right K every tick. */
static void scene_load_active_preset_into_system(Scene *s)
{
    s->ensemble.system.K = scene_active_preset(s)->K;
}

static void scene_reset(Scene *s, int w, int h)
{
    density_reset(&s->density, w, h);
    ensemble_init_uniform_grid(&s->ensemble);
    scene_load_active_preset_into_system(s);
    s->iterations_done = 0;
}
static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_WAVES);   /* middle of cascade */
    palette_state_init(&s->palette, 0);              /* DEFAULT theme    */
    scene_reset(s, w, h);
}

/* scene_is_settled — true once every trajectory has spent its full
 * iteration budget.  Past this point further integration would just
 * saturate cells, so scene_tick returns early. */
static inline bool scene_is_settled(const Scene *s)
{
    return s->iterations_done >= MAX_ITERATIONS_PER_TRAJ;
}

/* ensemble_advance_one_step — one chirikov_step for every orbit,
 * plus the matching density increment.  Inner kernel of scene_tick. */
static inline void ensemble_advance_one_step(TrajectoryEnsemble *e,
                                             DensityGrid *d)
{
    for (int i = 0; i < NUM_TRAJECTORIES; i++) {
        e->state[i] = chirikov_step(e->state[i], e->system);
        density_record(d, e->state[i]);
    }
}

/* scene_tick — advance the simulation by one fixed-timestep tick.
 *
 * DRIVER PSEUDOCODE
 *   if paused or settled: return
 *   repeat ITERATES_PER_TRAJ_PER_TICK times:
 *       ensemble_advance_one_step(ensemble, density)
 *   iterations_done += ITERATES_PER_TRAJ_PER_TICK
 */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    if (scene_is_settled(s)) return;

    for (int step = 0; step < ITERATES_PER_TRAJ_PER_TICK; step++)
        ensemble_advance_one_step(&s->ensemble, &s->density);

    s->iterations_done += ITERATES_PER_TRAJ_PER_TICK;
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
 *   takes Screen* (or cols/rows) explicitly so the layout math is
 *   decoupled from ncurses globals.
 *
 * CONTEXT
 *   Owned by App.  Initialised by screen_init; resized by
 *   screen_resize when SIGWINCH fires; consulted everywhere a draw
 *   needs the viewport.  Drives app_pick_map_size which clamps the
 *   density grid to fit (with CELLS_MAX ceilings to keep the static
 *   hits[] buffer bounded).
 *
 * MEMBER LOGIC
 *   cols : current terminal width  in cells.
 *   rows : current terminal height in cells.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
                                       getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void)     { wnoutrefresh(stdscr); doupdate(); }

/* glyph_for_band — 10-step ASCII gradient.  Glyph density tracks band
 * brightness, so a low-coverage band paints sparse dots while a
 * saturated cell paints a solid '@'.  Combined with the 10 colour
 * pairs and A_BOLD, this gives ~100 visually distinct intensity
 * levels — enough to render KAM tori as crisp curves and the chaotic
 * sea as a textured cloud at the same time. */
static char glyph_for_band(int b)
{
    static const char tiers[DENS_BAND_COUNT] = {
        '.', ',', ':', ';', '+', 'o', '*', 'X', '#', '@'
    };
    if (b < 0)                   return ' ';
    if (b >= DENS_BAND_COUNT)    return '@';
    return tiers[b];
}

/* density_band_to_pair — colour pair for a given density band.
 * Single source of truth so paint code and themes stay in sync. */
static inline short density_band_to_pair(int band)
{
    return (short)(PAIR_DENS_BASE + band);
}

/* density_grid_origin — top-left cell of the density grid inside the
 * drawable band, centred horizontally AND vertically.  Both origins
 * are clamped so the grid never overlaps the HUD even when the
 * terminal is taller than the grid. */
static inline void density_grid_origin(const DensityGrid *d,
                                       int cols, int rows,
                                       int *gx0, int *gy0)
{
    int x = (cols - d->w) / 2;
    int y = ((rows - HUD_BAND_RESERVED_ROWS) - d->h) / 2 + HUD_TOP_ROWS;
    if (x < 0)              x = 0;
    if (y < HUD_TOP_ROWS)   y = HUD_TOP_ROWS;
    *gx0 = x;
    *gy0 = y;
}

/* density_cell_in_drawable_band — clipping predicate.  True iff the
 * candidate (sx, sy) lies strictly inside the drawable band (HUD top
 * and HUD bottom both excluded). */
static inline bool density_cell_in_drawable_band(int sx, int sy,
                                                 int cols, int rows)
{
    return sx >= 0 && sx < cols
        && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* paint_density_cell — render ONE non-empty cell of the density grid.
 * Skips unvisited cells (band < 0 from density_band) so the
 * background stays untouched. */
static inline void paint_density_cell(int sy, int sx, int band)
{
    if (band < 0) return;
    short pair  = density_band_to_pair(band);
    char  glyph = glyph_for_band(band);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* density_paint — render the entire DensityGrid as a coloured field.
 *
 * DRIVER PSEUDOCODE
 *   (gx0, gy0) = density_grid_origin(d, cols, rows)
 *   for y in 0 .. d.h-1:
 *       sy = gy0 + y
 *       for x in 0 .. d.w-1:
 *           sx = gx0 + x
 *           if !density_cell_in_drawable_band(sx, sy): continue
 *           band = density_band(d.hits[y * d.w + x])
 *           paint_density_cell(sy, sx, band)        // skips band < 0
 */
static void density_paint(const DensityGrid *d, int cols, int rows)
{
    int gx0, gy0;
    density_grid_origin(d, cols, rows, &gx0, &gy0);

    for (int y = 0; y < d->h; y++) {
        int sy = gy0 + y;
        for (int x = 0; x < d->w; x++) {
            int sx = gx0 + x;
            if (!density_cell_in_drawable_band(sx, sy, cols, rows)) continue;
            int band = density_band(d->hits[y * d->w + x]);
            paint_density_cell(sy, sx, band);
        }
    }
}

/* hud_status_word — string shown next to the preset bracket in row 0.
 * Reflects what the simulation is doing right now: settled (done),
 * paused, or actively iterating (preset name). */
static inline const char *hud_status_word(const Scene *s)
{
    if (s->paused)            return "PAUSED  ";
    if (scene_is_settled(s))  return "DONE    ";
    return scene_active_preset(s)->name;
}

/* hud_iteration_percent — progress 0..100 toward the iteration cap.
 * Past 100 returns 100 (the cap might be slightly overshot inside one
 * tick's worth of iteration). */
static inline int hud_iteration_percent(const Scene *s)
{
    int pct = (s->iterations_done * 100) / MAX_ITERATIONS_PER_TRAJ;
    if (pct > 100) pct = 100;
    return pct;
}

/* regime_label_for_K — KAM-regime annotation given the active K.
 * KC_DEAD_BAND prevents jitter on the boundary. */
static inline const char *regime_label_for_K(float K)
{
    if (K < KC_GREENE - KC_DEAD_BAND) return "BELOW Kc";
    if (K > KC_GREENE + KC_DEAD_BAND) return "ABOVE Kc";
    return "AT Kc   ";
}

/* hud_write_title — left-aligned bold "STANDARD MAP (Chirikov)". */
static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " STANDARD MAP (Chirikov) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_write_status_right — right-aligned row-0 status text:
 * fps, sim_fps, status word, preset [n/N], K, iteration percent. */
static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const MapPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  K:%.4f  %3d%% ",
             fps, sim_fps, hud_status_word(s),
             s->preset.current + 1, (int)N_PRESETS,
             (double)active->K, hud_iteration_percent(s));
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row 0 of the HUD.
 *
 * DRIVER PSEUDOCODE
 *   hud_write_title()                       // left banner
 *   hud_write_status_right(...)             // right status
 */
static void hud_top_left_title(void) { hud_write_title(); }

static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    hud_write_status_right(cols, fps, sim_fps, s);
}

/* hud_write_preset_label / _theme_label / _threshold_segment — three
 * column segments of row 1, each a one-line "label : value" pair. */
static inline void hud_write_preset_label(int x, const MapPreset *active)
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
static inline void hud_write_threshold_segment(int x, float K)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " trajs:%d  %s  Kc=%.4f ",
             NUM_TRAJECTORIES, regime_label_for_K(K), (double)KC_GREENE);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_param_row — row 1 of the HUD: three labelled column segments.
 *
 * DRIVER PSEUDOCODE
 *   x = HUD_LEFT_MARGIN
 *   hud_write_preset_label    (x, active);   x += HUD_PARAM_PRESET_WIDTH
 *   hud_write_theme_label     (x, scene);    x += HUD_PARAM_THEME_WIDTH
 *   hud_write_threshold_segment(x, active.K)
 */
static void hud_param_row(const Scene *s)
{
    const MapPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label   (x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label    (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_threshold_segment(x, active->K);
}

static void hud_bottom_hint(int rows)
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
 *   density_paint(scene.density, cols, rows)         // phase-space image
 *   hud_top_left_title()                             // banner
 *   hud_top_right_status(cols, fps, sim_fps, scene)  // right status
 *   hud_param_row(scene)                             // preset+theme+regime
 *   hud_bottom_hint(rows)                            // key hints
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    density_paint(&s->density, sc->cols, sc->rows);
    hud_top_left_title();
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
 *   The "everything else" container.  Owns the simulation (Scene),
 *   the viewport (Screen), the simulation-rate knob (sim_fps), the
 *   density-grid dimensions (map_w/h, decoupled from screen dims so
 *   they can be clamped to CELLS_MAX without warping the layout),
 *   and the two volatile signal-handler flags.  Lives as a file-
 *   scope g_app so signal handlers can touch it without indirection.
 *
 * CONTEXT
 *   Created by app_bootstrap; mutated by app_handle_key and its
 *   named mutators; torn down by screen_free + atexit cleanup.
 *   main() never reaches into the sub-fields directly — every
 *   access goes through a named helper.
 *
 * MEMBER LOGIC
 *   scene       : the entire simulation state (§8 Scene) — ensemble
 *                 + density grid + selections.
 *   screen      : cached terminal dimensions (§9 Screen).
 *   sim_fps     : fixed-timestep rate scene_tick is driven at;
 *                 clamped to [SIM_FPS_MIN, SIM_FPS_MAX] = [10, 240].
 *                 Drives TICK_NS(sim_fps) which feeds the Fiedler [6]
 *                 accumulator in app_drain_fixed_timestep.
 *   map_w/h     : density-grid dimensions in cells.  Picked by
 *                 app_pick_map_size from screen × HUD reservation
 *                 with both safety floors and CELLS_MAX ceilings.
 *   running     : main-loop guard.  Cleared by SIGINT/TERM via
 *                 on_exit_signal, or by 'q'/ESC via app_handle_key.
 *   need_resize : SIGWINCH flag; consumed by
 *                 app_handle_pending_resize at the top of each
 *                 main-loop iteration.  Both flags MUST be volatile
 *                 sig_atomic_t (signal-handler write + main-loop
 *                 read, no other synchronisation).
 *
 * REFERENCES
 *   POSIX.1-2008 §2.4.3 — signal-safe writes via sig_atomic_t.
 *   [6] Fiedler — "Fix Your Timestep!" (drives sim_fps usage).
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

/* main_install_signal_handlers — wire SIGINT/TERM → quit, SIGWINCH → resize. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_pick_map_size — clamp the density grid to the terminal viewport.
 *
 * DRIVER PSEUDOCODE
 *   mw = screen.cols
 *   mh = screen.rows - HUD_BAND_RESERVED_ROWS
 *   clamp mw into [MAP_W_FLOOR, MAP_W_MAX]
 *   clamp mh into [MAP_H_FLOOR, MAP_H_MAX]
 *   app.map_w, app.map_h = mw, mh
 *
 * Floors avoid degenerate sizes on tiny terminals; ceilings keep the
 * static CELLS_MAX buffer from overflowing on huge ones. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < MAP_W_FLOOR) mw = MAP_W_FLOOR;
    if (mh < MAP_H_FLOOR) mh = MAP_H_FLOOR;
    if (mw > MAP_W_MAX)   mw = MAP_W_MAX;
    if (mh > MAP_H_MAX)   mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* app_bootstrap — first-time initialisation. */
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

/* app_drain_fixed_timestep — Fiedler [6] accumulator pattern.  Runs
 * scene_tick at the chosen sim_fps regardless of the render rate. */
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
static void app_reset_picture    (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
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
 *   frame_count, fps_display) into ONE named concept so main() reads
 *   as a pseudocode driver: "init clock, advance clock, drain
 *   simulation, throttle to render rate, present frame".  Threading
 *   them through helpers as a single FrameClock* removes 5 separate
 *   parameter lists from app_compute_frame_dt /
 *   app_drain_fixed_timestep / app_update_fps_meter.
 *
 * CONTEXT
 *   Lives as a local in main(); not owned by App because it is pure
 *   loop scaffolding (a different driver would replace it whole).
 *   Mutated by frame_clock_init, frame_clock_advance,
 *   frame_clock_reset_after_resize, app_drain_fixed_timestep, and
 *   app_update_fps_meter.
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock ns at the START of the previous frame.
 *                 app_compute_frame_dt subtracts this from clock_ns()
 *                 and stamps it forward.
 *   sim_accum   : ns of unspent simulation time.  Drained in multiples
 *                 of TICK_NS(sim_fps) by the inner while-loop of
 *                 app_drain_fixed_timestep — this is the Fiedler [6]
 *                 accumulator that lets the integrator and renderer
 *                 advance at independent rates.
 *   fps_accum   : ns elapsed since the last fps recalculation.
 *   frame_count : frames rendered since the last fps recalculation.
 *   fps_display : the displayed fps value, refreshed every
 *                 FPS_UPDATE_MS = 500 ms so the HUD doesn't flicker.
 *
 * REFERENCES
 *   [6] Fiedler — "Fix Your Timestep!" — the accumulator pattern
 *       this struct implements.
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
 *   main_install_signal_handlers()                       // SIGINT/TERM/WINCH
 *   app_bootstrap(app)                                   // ncurses + Scene
 *   clk = frame_clock_init()
 *   while running:
 *       if need_resize:
 *           app_handle_pending_resize(app)
 *           frame_clock_reset_after_resize(clk)
 *       dt = app_compute_frame_dt(&clk.frame_time)
 *       frame_clock_advance(clk, dt)
 *       app_drain_fixed_timestep(app, &clk.sim_accum)    // scene_tick × N
 *       app_update_fps_meter(...)
 *       app_throttle_to_render_target(clk.frame_time, dt) // sleep to 60 fps
 *       app_present_frame(app, clk.fps_display)
 *       if !app_poll_keyboard(app):  running = 0         // q/ESC → quit
 *   screen_free(app)                                     // endwin via atexit
 *
 * Body reads top-to-bottom as the OVERALL flow of the program; every
 * named call is one line in this block. */
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
