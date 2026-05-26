/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * henon_heiles.c
 *   — The Hénon-Heiles potential.  A 2-degree-of-freedom Hamiltonian
 *     system originally proposed in 1964 to model the motion of a
 *     star around a galactic centre.  Energy E is the bifurcation
 *     parameter: at LOW E the orbits are regular (closed curves on
 *     a Poincaré section); above a critical E the section explodes
 *     into a chaotic sea with KAM islands.
 *
 * DEMO: Integrate N_TRAJ initial conditions, each at the same total
 *       energy E, with RK4.  Each time x crosses 0 going up (ẋ > 0),
 *       record (y, py) into a density grid.  7 presets n/p-cyclable,
 *       each at a qualitatively distinct point on the E axis:
 *         QUIET (0.02) → LOW_E (0.07) → BUDDING (0.10) → MIXED (0.12)
 *         → BROKEN (0.14) → HIGH_E (0.16) → BRINK (0.165, near-escape)
 *       Default opens on PRESET_MIXED — the most pedagogically rich
 *       view (KAM islands AND chaotic sea coexist).
 *
 * Study alongside:
 *   ./standard_map.c       — same KAM-vs-chaos picture, but applied
 *                            directly to a discrete map (skips ODE).
 *   ./poincare_section.c   — same trick (sample at plane crossings)
 *                            applied to dissipative Lorenz instead
 *                            of conservative Hénon-Heiles.
 *   ./double_pendulum.c    — another 2-DOF Hamiltonian.  HH is the
 *                            canonical test case in the literature.
 *
 * Section map:
 *   §1 config   — constants, 7-preset table, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes (PAIR_DENS_BASE+0..7)
 *   §5 physics  — HHState + Hamilton's equations, RK4 with named stages,
 *                 energy helpers (potential / kinetic / total)
 *   §6 section  — SectionGrid: 2-D histogram of x=0 crossings in (y, py)
 *   §7 state    — TrajectoryEnsemble (N_TRAJ orbits + crossing detector)
 *                 + PresetState + PaletteState typed wrappers
 *   §8 scene    — Scene composes ensemble + section + preset + palette
 *   §9 screen   — PoincareViewport, layered painters (axes, boundary,
 *                 density, frame), HUD writers
 *   §10 app     — signals, resize, key dispatch table, FrameClock,
 *                 main pseudocode driver + named loop helpers
 *
 * Keys: q/ESC quit | space pause | r reset | n/p preset |
 *       t/T theme | ]/[ Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/henon_heiles.c \
 *       -o henon_heiles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : 4-D autonomous Hamiltonian flow.  State = (x, y, px,
 *                 py) per trajectory.  RK4 with named-stage helpers at
 *                 a fixed substep HH_DT = 0.05, with INT_STEPS_PER_TICK
 *                 substeps per render tick.  An ENSEMBLE of N_TRAJ
 *                 orbits is seeded on the x = 0 plane (linearly spaced
 *                 y values, py = 0, px chosen so all start at the same
 *                 total energy E).  Every substep, each orbit is
 *                 RK4-advanced and tested for an upward (ẋ > 0) zero-
 *                 crossing of x; if found, (y, py) are linearly
 *                 interpolated AT the crossing and recorded into the
 *                 Poincaré-section histogram.  See §5 hh_rk4 + §7
 *                 trajectory_advance_one for the named-stage pipeline.
 *
 * Data-struct   : §5 HHState (x, y, px, py — one trajectory's 4-D
 *                 phase point, used by RK4 and the crossing detector).
 *                 §7 TrajectoryEnsemble (N_TRAJ HHStates + prev[] and
 *                 prev_x[] for crossing-test bookkeeping).
 *                 §6 SectionGrid (uint16 hits per cell, w × h grid in
 *                 (y, py) viewport coords; log-saturating density
 *                 mapping via density_band).
 *
 * Rendering     : ASCII-only.  Phase-space viewport (PoincareViewport)
 *                 maps (y, py) ∈ [Y_MIN, Y_MAX] × [PY_MIN, PY_MAX] to
 *                 a centred terminal rectangle, with a 4-layer paint
 *                 pipeline:
 *                   AXES (back)      — faint y=0 / py=0 guide lines
 *                   BOUNDARY         — `'` / `,` glyphs for the
 *                                       ±py_max energy perimeter
 *                   DENSITY          — 8-tier graduated glyph ramp
 *                                       (.  :  ;  +  o  *  #  @)
 *                                       coloured by log(hits) band
 *                   FRAME (front)    — '+'-cornered '-' / '|' box one
 *                                       cell outside the data
 *                 The eight glyphs match the eight density-band colour
 *                 pairs so the gradient reads even on monochrome.
 *
 * References    : THE PROBLEM  (where the equation comes from)
 *                 [1] Hénon, M., Heiles, C. (1964) — "The applicability
 *                     of the third integral of motion: Some numerical
 *                     experiments", Astron. J. 69, pp. 73-79.  The
 *                     ORIGINAL paper.  Introduces the cubic-saddle
 *                     potential V(x, y) = ½(x² + y²) + x²y - y³/3 (in
 *                     hh_potential()), the x = 0 Poincaré section
 *                     convention (in trajectory_ensemble_advance), and
 *                     the canonical low-E / high-E plots that the
 *                     PRESET_LOW_E and PRESET_HIGH_E entries reproduce.
 *                 [2] Tabor, M. (1989) — "Chaos and Integrability in
 *                     Nonlinear Dynamics", Wiley, Ch. 4.  Pedagogical
 *                     walk-through of Hénon-Heiles as the textbook
 *                     example of a 2-DOF non-integrable Hamiltonian.
 *
 *                 KAM THEORY  (why tori dissolve as E rises)
 *                 [3] Arnold, V. I. (1989) — "Mathematical Methods
 *                     of Classical Mechanics" (2nd ed.), Springer,
 *                     Appendix 8.  The KAM theorem: under sufficiently
 *                     irrational frequency ratios, "most" tori survive
 *                     small Hamiltonian perturbations.  HH's E acts as
 *                     the perturbation amplitude in this picture.
 *                 [4] Lichtenberg, A. J., Lieberman, M. A. (1992) —
 *                     "Regular and Chaotic Dynamics" (2nd ed.),
 *                     Springer §3.2.  The KAM-tori-dissolution
 *                     sequence as E increases — exactly the bifurcation
 *                     walk the 7-preset table in §1 traces.
 *
 *                 POINCARÉ-SECTION TECHNIQUE  (the rendering "trick")
 *                 [5] Poincaré, H. (1892) — "Les Méthodes Nouvelles
 *                     de la Mécanique Céleste", Vol. III.  The
 *                     stroboscopic-sampling trick this file uses:
 *                     convert a continuous flow into a discrete map
 *                     by recording the orbit's intersections with a
 *                     transverse surface (here: the x = 0 plane,
 *                     ẋ > 0 branch).
 *
 *                 NUMERICAL INTEGRATION
 *                 [6] Press, W. H., Teukolsky, S. A., Vetterling, W. T.,
 *                     Flannery, B. P. (2007) — "Numerical Recipes"
 *                     (3rd ed.), Cambridge, Ch. 17.1.  RK4 derivation,
 *                     local truncation O(dt⁵), and the Butcher tableau
 *                     in §5 hh_rk4 / rk4_butcher_weighted_average.
 *                 [7] Yoshida, H. (1990) — "Construction of higher
 *                     order symplectic integrators", Phys. Lett. A
 *                     150.  HH is HAMILTONIAN — symplectic schemes
 *                     conserve energy to machine precision over
 *                     arbitrarily long runs (RK4 has O(dt⁴) drift).
 *                     This file uses RK4 for simplicity; Yoshida's
 *                     4th-order symplectic would be the upgrade if
 *                     the section ever shows visible E-drift.
 *                 [8] Fiedler, G. (2004) — "Fix Your Timestep!",
 *                     gafferongames.com.  Accumulator pattern used
 *                     by app_drain_fixed_timestep — keeps the
 *                     trajectories deterministic across frame rates
 *                     even though the demo crosses many drive cycles
 *                     before the structure emerges.
 *
 *                 SIMULATION ARCHITECTURE & RENDERING
 *                 [9] Sussman, G. J., Wisdom, J. (2014) — "Structure
 *                     and Interpretation of Classical Mechanics"
 *                     (2nd ed.), MIT Press.  The "system / state /
 *                     observable" triad that Scene mirrors at the
 *                     code-architecture level (DuffingSystem-style
 *                     decomposition adapted to a parameter-less
 *                     Hamiltonian here).
 *                 [10] Gookin, D. (2007) — "Programmer's Guide to
 *                     NCurses", Wiley.  Double-buffered rendering
 *                     model (wnoutrefresh + doupdate), color-pair
 *                     theory, and SIGWINCH handling used in §3, §9-10.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Hénon and Heiles asked: in a galaxy, does a third constant of
 * motion exist (beyond energy and z-component of angular momentum)?
 * If YES → orbits are integrable, predictable.  If NO → chaos.
 * They plotted Poincaré sections at increasing energies and watched
 * the picture transition from neat closed curves (3rd integral
 * exists locally) to a chaotic sea (3rd integral does NOT exist).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Phase space is 4-D.  Energy E is conserved → trajectories live on
 * a 3-D "constant-E surface".  Slicing that surface with the plane
 * x = 0 gives a 2-D Poincaré section showing (y, py) at each return.
 *   Regular orbits → closed curves (a third constant exists).
 *   Chaotic orbits → 2-D filled cloud (only E constrains them).
 *
 * KEY FORMULAS
 * ────────────
 *   V(x, y) = ½(x² + y²) + x²y − y³/3
 *   H = ½(px² + py²) + V(x, y)
 *
 *   ẋ  =  px
 *   ẏ  =  py
 *   ṗx = -x - 2xy
 *   ṗy = -y - x² + y²
 *
 * HOW TO VERIFY  (cycle through with n / p)
 * ─────────────
 *  • QUIET    (E=0.020): a few tiny central tori — barely-perturbed.
 *  • LOW_E    (E=0.070): nested closed curves with 3-fold symmetry —
 *                        the classic "integrable-like" reference plate.
 *  • BUDDING  (E=0.100): chain-of-three islands appear at the resonance.
 *  • MIXED    (E=0.120): islands AND a thin chaotic ring coexist
 *                        (DEFAULT — the most pedagogically rich view).
 *  • BROKEN   (E=0.140): most outer tori have dissolved; chaos growing.
 *  • HIGH_E   (E=0.160): chaotic sea with surviving islands — the
 *                        classic Hénon-Heiles chaos plate.
 *  • BRINK    (E=0.165): near E_escape = 1/6; attractor barely bounded.
 *  • Reset (r) with the same preset → exact-same figure
 *    (deterministic; fixed seed pattern).
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
    MAP_W_MAX        = 200, MAP_H_MAX = 56, CELLS_MAX = MAP_W_MAX * MAP_H_MAX,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_DENS_BASE   =   3,    /* 8 density bands */
    PAIR_BOUNDARY    =  11,
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

#define HH_DT                     0.05f
#define INT_STEPS_PER_TICK         60
#define N_TRAJ                     32

/* Section viewport (y, py) — Hénon-Heiles bound below E_escape = 1/6 */
#define Y_MIN                    -0.5f
#define Y_MAX                     0.7f
#define PY_MIN                   -0.5f
#define PY_MAX                    0.5f

#define DENS_BAND_COUNT           8
#define DENS_SAT_LOG              7.0f

/*
 * Preset — index into a 7-entry table walking the Hénon-Heiles
 * energy axis from "almost integrable" up to "near escape", ordered
 * simple → complex.  Each preset picks an E that is QUALITATIVELY
 * distinct from its neighbours, not just a quantitative refinement:
 *
 *   E ≲ 0.04   only the small central tori — barely-perturbed Kepler
 *   E ≈ 0.07   classic LOW_E plot: nested closed curves (KAM tori)
 *   E ≈ 0.10   chains-of-three islands appear (resonance birth)
 *   E ≈ 0.12   islands + a thin chaotic ring around them coexist
 *   E ≈ 0.14   most outer tori have dissolved; chaos dominates
 *   E ≈ 0.16   classic HIGH_E plot: chaotic sea with surviving islands
 *   E ≈ 0.165  near E_escape = 1/6; attractor barely bounded
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Hénon & Heiles (1964)        — original paper; Fig. 3 is the
 *                                      canonical E = 0.08333… and
 *                                      E = 0.125 sections that
 *                                      PRESET_LOW_E and PRESET_MIXED
 *                                      reproduce in qualitative form.
 *   [3] Arnold §App.8 (KAM theorem)  — explains why low-E tori survive
 *                                      (PRESET_QUIET / PRESET_LOW_E)
 *                                      and start dissolving as E rises.
 *   [4] Lichtenberg & Lieberman §3.2 — the KAM-tori-dissolution sequence
 *                                      that the 7-preset walk traces.
 */
typedef enum {
    PRESET_QUIET = 0,    /* E = 0.02   tiny central tori only           */
    PRESET_LOW_E,        /* E = 0.07   classic regular-regime plot      */
    PRESET_BUDDING,      /* E = 0.10   chain-of-three islands appear    */
    PRESET_MIXED,        /* E = 0.12   islands + thin chaotic ring      */
    PRESET_BROKEN,       /* E = 0.14   most tori dissolved              */
    PRESET_HIGH_E,       /* E = 0.16   classic chaotic-regime plot      */
    PRESET_BRINK,        /* E = 0.165  near-escape, attractor bounded   */
    N_PRESETS,
} Preset;

/*
 * HHPreset — one row of the 7-entry presets[] table: a named energy E.
 *
 * INTENT
 *   The Hénon-Heiles Hamiltonian has NO free parameters — the equation
 *   is fixed once and for all by Hénon & Heiles 1964.  The ONLY thing
 *   that changes between experiments is the TOTAL ENERGY E at which
 *   the ensemble is seeded.  E is the bifurcation parameter; sweeping
 *   it walks the KAM-tori-dissolution sequence from "almost integrable"
 *   to "almost-unbounded".  Bundling each chosen E with a human-readable
 *   name turns the bifurcation walk into a browsable table.
 *
 * CONTEXT
 *   Lives only in the const-table presets[] (immediately below).
 *   Looked up by §7's PresetState wrapper via preset_state_active().
 *   Consumed by §7 trajectory_ensemble_init_at_energy() which seeds
 *   N_TRAJ orbits at the .E value.  Read by §9 to draw the
 *   energy-boundary curve.  Never mutated at runtime.
 *
 * MEMBER LOGIC
 *   name : 8-char label shown in HUD (%-8s format).  Space-padded for
 *          column alignment.  Picked to evoke each preset's visual
 *          class (QUIET / BUDDING / BROKEN / BRINK).
 *   E    : total energy H = T + V chosen for the experiment.  Must be
 *          POSITIVE (so V can be reached) and BELOW E_escape = 1/6 ≈
 *          0.1667 (above which orbits run off to infinity over the
 *          cubic saddle).  The HH bifurcation lives entirely in
 *          E ∈ (0, 1/6).
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Hénon & Heiles (1964) — Figs. 3-4 use E = 0.08333… and
 *                                E = 0.125; the 7-preset table walks
 *                                the same E-axis with finer steps.
 *   [4] Lichtenberg & Lieberman §3.2 — KAM-tori-dissolution sequence
 *                                       parameterised by E.
 */
typedef struct {
    const char *name;
    float       E;
} HHPreset;

/* Energy values ordered simple → complex.  E_escape = 1/6 ≈ 0.1667
 * is the upper bound: above it, trajectories climb the cubic saddle
 * and run off to infinity (unbounded orbits — no Poincaré return). */
static const HHPreset presets[N_PRESETS] = {
    { "QUIET   ", 0.020f },
    { "LOW_E   ", 0.070f },
    { "BUDDING ", 0.100f },
    { "MIXED   ", 0.120f },
    { "BROKEN  ", 0.140f },
    { "HIGH_E  ", 0.160f },
    { "BRINK   ", 0.165f },
};

/*
 * Theme — colour assignment for the Poincaré-section renderer.
 *
 * INTENT
 *   Decouples "what gets drawn" (the §9 painters) from "what colour
 *   it should be" so a single cycle key (t/T) swaps the entire
 *   visual identity without changing one line of physics or rendering
 *   logic.  The HH renderer has two render roles — a DENSITY ramp
 *   (8 tiers, oldest-to-newest crossing density) and a BOUNDARY
 *   colour (frame + axes + energy-boundary curve) — and one Theme
 *   row assigns both coherently.
 *
 * CONTEXT
 *   Lives only in the const-table themes[] of N_THEMES entries.
 *   Selected by §7's PaletteState wrapper via palette_state_active().
 *   Pushed into ncurses by §3 theme_apply() which calls init_pair()
 *   on each (PAIR_*, field, -1) triple.  Never mutated at runtime —
 *   t/T just changes which row gets fed to init_pair.
 *
 * MEMBER LOGIC
 *   name           : 7-char label (different from the 8-char preset
 *                    labels — themes have one less character of
 *                    header in the table).  Padded for column align.
 *   band[0..N-1]   : the 8-tier DENSITY ramp.  band[0] = lowest hit
 *                    count tier (dimmest); band[N-1] = saturated tier
 *                    (brightest).  Must be a monotonic perceptual
 *                    gradient — paint_density_cells maps log(hits) to
 *                    a band index, so the ramp's tiers carry quantitative
 *                    meaning, not just decoration.  Every entry must
 *                    sit at xterm-256 ≥ 24 per CLAUDE.md "Theme Palette
 *                    Brightness".
 *   bnd            : single colour index for the BOUNDARY layer — the
 *                    frame box, the y=0 / py=0 axes, and the energy-
 *                    boundary curve.  Conventionally a neutral mid-
 *                    luminance grey so the frame anchors the eye
 *                    without competing with the data.
 *
 * REFERENCES
 *   • CLAUDE.md "Theme Palette Brightness" — the ≥ 24 rule that
 *     keeps band[0] legible against default-black.
 *   • CLAUDE.md "HUD Standard" — PAIR_HUD/PAIR_HINT use bright
 *     yellow + cyan and are NOT per-theme.
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];
    short       bnd;
} Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  39,  75, 117, 153, 189, 220, 226, 231 }, 244 },
    { "MATRIX",  {  46,  82, 118, 154, 156, 191, 192, 194 }, 244 },
    { "NOVA",    {  99, 135, 171, 207, 213, 219, 225, 231 }, 244 },
    { "MONO",    { 244, 247, 250, 252, 253, 254, 255, 231 }, 240 },
    { "OCEAN",   {  39,  45,  81, 117, 153, 159, 195, 231 }, 244 },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 220, 227 }, 244 },
    { "EARTH",   { 100, 137, 173, 179, 215, 222, 228, 231 }, 244 },
    { "FOREST",  {  64, 107, 114, 144, 150, 156, 194, 231 }, 244 },
    { "DESERT",  { 130, 173, 179, 215, 222, 228, 230, 231 }, 244 },
    { "ARCTIC",  {  81, 117, 153, 159, 195, 225, 230, 231 }, 244 },
};

/* ===================================================================== */
/* §2 clock + §3 color                                                    */
/* ===================================================================== */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* PAIR_HUD_FG_256 / PAIR_HINT_FG_256 — bright yellow / bright cyan in
 * the xterm-256 cube.  Named so the HUD setup doesn't carry magic
 * literals inline. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* theme_apply_pairs_256color — push the 8 density-band + 1 boundary
 * colour-pair assignments for a 256-colour terminal.  Each pair maps
 * one render role (density-band-i / boundary) to one xterm-256 index
 * from the Theme row.  Foregrounds use the terminal's default
 * background (-1) so the demo composites cleanly on dark or light. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
    init_pair(PAIR_BOUNDARY, t->bnd, -1);
}

/* theme_apply_pairs_8color_fallback — single fallback assignment for
 * 8-colour terminals where per-theme xterm-256 indices can't be
 * expressed.  All N_THEMES collapse to the SAME 8-colour set — the
 * trade-off is "lose theme variety, keep legibility everywhere". */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_BOUNDARY, COLOR_WHITE, -1);
}

/* theme_apply — top-level dispatcher: clamp the index, branch on
 * terminal capability, delegate to the appropriate helper. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* color_init_hud_pairs — set up the project-standard HUD pair (bright
 * yellow) and HINT pair (bright cyan) per CLAUDE.md's "HUD Standard".
 * These pairs are NOT per-theme — they stay constant as t/T cycles so
 * the status row never loses contrast. */
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
 *   STEP 3 — push the first theme's colours
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* ===================================================================== */
/* §5  physics — Hénon-Heiles Hamiltonian flow                            */
/* ===================================================================== */

/*
 * HHState — one trajectory's 4-D phase-space coordinate (x, y, px, py).
 *
 * INTENT
 *   The state vector y for the autonomous Hamiltonian ODE
 *   dy/dt = f(y).  Carries everything an orbit needs to advance:
 *   both POSITION (x, y) AND CONJUGATE MOMENTUM (px, py).  Unlike a
 *   driven system (no explicit time-dependence here — HH is
 *   autonomous), no time field is needed; the orbit is determined by
 *   its starting 4-tuple plus the equations of motion.
 *
 *   Stored as a flat by-value struct so RK4 stage states (slope_start
 *   … slope_end) can be created and combined cheaply.  The Hamiltonian
 *   gives Hamilton's equations:
 *     H(x, y, px, py) = ½(px² + py²)            ← kinetic T
 *                     + ½(x² + y²) + x²y - y³/3 ← potential V(x, y)
 *
 *     ẋ  = ∂H/∂px =  px
 *     ẏ  = ∂H/∂py =  py
 *     ṗx = -∂H/∂x = -x - 2xy
 *     ṗy = -∂H/∂y = -y - x² + y²
 *
 * CONTEXT
 *   N_TRAJ instances live inside TrajectoryEnsemble.state[].  Lifetime:
 *     • trajectory_ensemble_init_at_energy seeds (x=0, y, px, py=0)
 *       on r / n / p.
 *     • hh_rk4 mutates ~32 × 60 × INT_STEPS_PER_TICK = ~115k times/sec.
 *     • trajectory_is_upward_x_crossing reads (x, px) every substep
 *       to detect the Poincaré-section condition.
 *
 * MEMBER LOGIC
 *   x, y   : configuration-space position.  HH's triangular cubic
 *            well has minimum at (0, 0) and three saddle points at
 *            radius 1 forming an equilateral triangle around it.
 *            Bound orbits stay inside the triangle; E > 1/6 lets
 *            them escape over a saddle.
 *   px, py : conjugate momenta (here equal to ẋ, ẏ since masses
 *            are 1).  Stored as floats — the kinetic-budget
 *            calculation in hh_make_state_on_x0_at_energy() needs
 *            sqrtf precision for the px seed; full double is overkill
 *            for the visualisation scale.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Hénon & Heiles (1964) §II eq. (1) — the Hamiltonian itself.
 *   [3] Arnold §3                          — general Hamiltonian
 *                                            framework, Poisson
 *                                            brackets, symplectic
 *                                            geometry.
 *   [9] Sussman & Wisdom Ch. 3             — modern functional take
 *                                            on the same material;
 *                                            architectural inspiration
 *                                            for the state/system split.
 */
typedef struct {
    float x, y;
    float px, py;
} HHState;

/* hh_potential — V(x, y).  Triangular cubic well: three saddle
 * points form an equilateral triangle around the central minimum.
 * Above E_escape = 1/6 the saddles are accessible and orbits escape. */
static inline float hh_potential(float x, float y)
{
    return 0.5f * (x*x + y*y) + x*x*y - y*y*y / 3.0f;
}

/* hh_kinetic — T = ½(px² + py²). */
static inline float hh_kinetic(float px, float py)
{
    return 0.5f * (px*px + py*py);
}

/* hh_total_energy — H = T + V.  Conserved (to RK4 truncation) along
 * any orbit; used to seed the ensemble at a chosen energy. */
static inline float hh_total_energy(const HHState *s)
{
    return hh_kinetic(s->px, s->py) + hh_potential(s->x, s->y);
}

/* hh_deriv — evaluate dy/dt = f(y) for one HHState.  Returns the
 * derivative as another HHState — kinematic identity for (x, y) and
 * the negated potential gradient for (px, py). */
static inline HHState hh_deriv(const HHState *s)
{
    HHState out;
    out.x  =  s->px;                                  /* ẋ  =  px           */
    out.y  =  s->py;                                  /* ẏ  =  py           */
    out.px = -s->x - 2.0f * s->x * s->y;              /* ṗx = -∂V/∂x        */
    out.py = -s->y -        s->x * s->x + s->y * s->y;/* ṗy = -∂V/∂y        */
    return out;
}

/* state_add — y + h·k, returned as a new HHState.  Used inside RK4
 * to build intermediate stage points. */
static inline HHState state_add(const HHState *a, float h, const HHState *k)
{
    HHState r;
    r.x  = a->x  + h * k->x;
    r.y  = a->y  + h * k->y;
    r.px = a->px + h * k->px;
    r.py = a->py + h * k->py;
    return r;
}

/* RK4 Butcher-tableau constants. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* rk4_butcher_weighted_average — Simpson-rule combination of the 4
 * slope estimates: (k₁ + 2k₂ + 2k₃ + k₄) / 6.  Component-wise on
 * the 4-D HHState. */
static inline HHState rk4_butcher_weighted_average(
    const HHState *k1, const HHState *k2,
    const HHState *k3, const HHState *k4)
{
    HHState avg;
    avg.x  = (k1->x  + 2.0f*k2->x  + 2.0f*k3->x  + k4->x ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y  = (k1->y  + 2.0f*k2->y  + 2.0f*k3->y  + k4->y ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.px = (k1->px + 2.0f*k2->px + 2.0f*k3->px + k4->px) / RK4_BUTCHER_WEIGHT_SUM;
    avg.py = (k1->py + 2.0f*k2->py + 2.0f*k3->py + k4->py) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* hh_rk4 — one fixed-step RK4 update of an HHState.
 *
 *   STAGE 1 — slope_start = f(y)
 *   STAGE 2 — slope_mid_1 = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2 = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end   = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * See header ref [6] Numerical Recipes Ch. 17.1 for the Butcher
 * tableau, O(dt⁵) local truncation, and stability domain.
 *
 * NOTE on integrator choice: HH is HAMILTONIAN, so SYMPLECTIC schemes
 * (ref [7] Yoshida 1990) conserve energy to machine precision over
 * arbitrarily long runs.  RK4 has O(dt⁴) energy drift per orbit.  At
 * HH_DT = 0.05 over the ~10⁴ steps needed for the section to fill,
 * the drift is < 0.1 % of E — invisible.  If you push to hours-long
 * runs or sub-percent E, switch to a 4th-order symplectic scheme. */
static void hh_rk4(HHState *s, float dt)
{
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    HHState slope_start    = hh_deriv(s);
    HHState midpoint_1     = state_add(s, half_dt, &slope_start);
    HHState slope_mid_1    = hh_deriv(&midpoint_1);
    HHState midpoint_2     = state_add(s, half_dt, &slope_mid_1);
    HHState slope_mid_2    = hh_deriv(&midpoint_2);
    HHState endpoint       = state_add(s, dt, &slope_mid_2);
    HHState slope_end      = hh_deriv(&endpoint);

    HHState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    *s = state_add(s, dt, &effective_slope);
}

/* hh_make_state_on_x0_at_energy — initialise an HHState sitting on
 * the x = 0 plane with the given y, py = 0, and px chosen so the
 * total energy equals E_target.
 *
 *   T = E - V(0, y)            ⇒  ½(px² + py²) = E - V
 *   px = √(2(E - V))           (taking the positive branch — orbit
 *                               moving in +x direction)
 *
 * If V(0, y) > E the state is energetically forbidden — px is
 * clamped to 0 (the caller will see no x-crossing for this seed). */
static inline HHState hh_make_state_on_x0_at_energy(float y, float E_target)
{
    float V              = hh_potential(0.0f, y);
    float kinetic_budget = 2.0f * (E_target - V);
    if (kinetic_budget < 0.0f) kinetic_budget = 0.0f;
    HHState s = { 0.0f, y, sqrtf(kinetic_budget), 0.0f };
    return s;
}

/* ===================================================================== */
/* §6  section grid                                                       */
/* ===================================================================== */

/*
 * SectionGrid — 2-D histogram of Poincaré-section crossings in (y, py).
 *
 * INTENT
 *   The orbit's TOPOLOGY in phase space — closed curves vs chaotic
 *   sea — is invisible in a single snapshot; you need many crossings
 *   accumulated to "draw the picture".  This struct is the
 *   ACCUMULATOR: a w×h grid of uint16 hit-counts, one bin per cell.
 *   At paint time density_band(hits) maps the count to one of
 *   DENS_BAND_COUNT = 8 visual tiers via a log-mapping, so the eye
 *   reads HIGH density as a sharp KAM curve and LOW density as a
 *   diffuse chaotic background.
 *
 * CONTEXT
 *   One instance lives on Scene.  Lifetime:
 *     • section_reset(w, h)  — called on r / n / p / SIGWINCH;
 *                              zeros all bins and sets dimensions.
 *     • section_record(y,py) — called once per orbit per upward
 *                              x-crossing by trajectory_ensemble_
 *                              advance; increments one bin.
 *     • paint_density_cells  — reads .hits to build the visual.
 *   Capacity: hits[CELLS_MAX] is statically sized to
 *   MAP_W_MAX × MAP_H_MAX (the largest viewport we ever use).
 *
 * DATA-STRUCTURE LOGIC
 *   2-D histogram chosen over a list of (y, py) points for the same
 *   two reasons the Trail in duffing uses a ring buffer:
 *     1. ZERO ALLOCATION in the hot path.
 *     2. CONSTANT cost per crossing — section_record is one bounds
 *        check + one increment regardless of how long the demo has
 *        been running.
 *   uint16 ceiling: a bin saturates at 65535 hits — at ~115k
 *   substeps/sec ÷ 32 trajs ≈ 3.5k substeps/traj/sec, with one
 *   crossing every ~2π / (orbit period) ≈ 1 hit/sec/traj.  So a hot
 *   KAM-curve bin needs hours to saturate.  Plenty.
 *
 * MEMBER LOGIC
 *   w, h  : data dimensions in cells.  Set by section_reset; matches
 *           the PoincareViewport's w/h at paint time.
 *   count : w × h (cached).  Used by section_reset to memset only
 *           the live slice of hits[], not the whole CELLS_MAX array.
 *   hits  : 2-D bin counts indexed [row × w + col].  Row 0 is the
 *           TOP visual row (= py near +PY_MAX); row h-1 is the BOTTOM
 *           (= py near -PY_MAX).  This row-flip happens in
 *           section_record so paint_density_cells can iterate
 *           top-to-bottom in screen order without re-flipping.
 *
 * REFERENCES (numbered in file-header References block)
 *   [5] Poincaré (1892) — the section-sampling idea this struct
 *                         accumulates the output of.
 */
typedef struct {
    int      w, h, count;
    uint16_t hits[CELLS_MAX];
} SectionGrid;

static void section_reset(SectionGrid *g, int w, int h)
{ g->w = w; g->h = h; g->count = w * h;
  memset(g->hits, 0, sizeof(uint16_t) * (size_t)g->count); }

static bool yp_to_cell(const SectionGrid *g, float y, float py, int *cx, int *cy)
{
    if (y < Y_MIN || y > Y_MAX || py < PY_MIN || py > PY_MAX) return false;
    *cx = (int)((y  - Y_MIN ) / (Y_MAX  - Y_MIN ) * (float)g->w);
    *cy = (int)((py - PY_MIN) / (PY_MAX - PY_MIN) * (float)g->h);
    if (*cx < 0)        *cx = 0;
    if (*cx >= g->w)    *cx = g->w - 1;
    if (*cy < 0)        *cy = 0;
    if (*cy >= g->h)    *cy = g->h - 1;
    return true;
}

static void section_record(SectionGrid *g, float y, float py)
{
    int cx, cy; if (!yp_to_cell(g, y, py, &cx, &cy)) return;
    int idx = (g->h - 1 - cy) * g->w + cx;
    if (g->hits[idx] < UINT16_MAX) g->hits[idx]++;
}

/* DENSITY_BAND_ROUND_OFFSET — the 0.5f added before float→int truncation
 * to convert truncation into nearest-integer rounding.  Without it the
 * band assignment skews low (every fractional band index gets floored
 * to the band below). */
#define DENSITY_BAND_ROUND_OFFSET   0.5f

/* density_band — map a hit-count to a 0..N-1 colour-band index using
 * a LOG mapping (so the eye sees relative density, not absolute).
 *
 *   STEP 1 — empty bin → -1 sentinel (caller skips)
 *   STEP 2 — log-saturate: v = log(hits) / DENS_SAT_LOG ∈ [0, 1]
 *   STEP 3 — scale to band index with rounding
 *
 * log-mapping rationale: a hot KAM curve might have 10⁴ hits while a
 * cold chaotic background has 10¹ — linear scaling would crush the
 * gradient.  log(N) over a saturation point of e^7 ≈ 1100 keeps the
 * full range readable. */
static inline int density_band(uint16_t hits)
{
    /* STEP 1 — empty cell sentinel */
    if (hits == 0) return -1;

    /* STEP 2 — log-saturate to [0, 1] */
    float saturated_log = logf((float)hits) / DENS_SAT_LOG;
    if (saturated_log < 0.0f) saturated_log = 0.0f;
    if (saturated_log > 1.0f) saturated_log = 1.0f;

    /* STEP 3 — quantise into 0..N-1 band index with rounding */
    int band = (int)(saturated_log * (float)(DENS_BAND_COUNT - 1)
                     + DENSITY_BAND_ROUND_OFFSET);
    if (band > DENS_BAND_COUNT - 1) band = DENS_BAND_COUNT - 1;
    return band;
}

/* ===================================================================== */
/* §7  state — typed wrappers + trajectory ensemble                       */
/* ===================================================================== */

/*
 * TrajectoryEnsemble — fixed-size pool of N_TRAJ Hénon-Heiles orbits
 * evolving simultaneously, all at the same total energy E but with
 * different initial y coordinates on the x = 0 plane.
 *
 * INTENT
 *   The Poincaré-section visualisation works by integrating MANY
 *   orbits and recording their (y, py) at every upward crossing of
 *   x = 0.  Each orbit traces ITS OWN curve (or chaotic region) on
 *   the section — the ensemble lets us see ALL the KAM tori plus
 *   the chaotic sea in one image, rather than running 32 separate
 *   simulations.  Bundling the trajectory pool + the per-orbit
 *   bookkeeping (previous full state + previous x for sign-change
 *   test) under one named struct keeps Scene clean and turns
 *   "advance + detect" into one well-named call.
 *
 *   The "ensemble at fixed E" framing comes directly from Hénon &
 *   Heiles' original 1964 plates — they chose one E, spread initial
 *   conditions across the energy-allowed band, and let each orbit
 *   paint its own section signature.
 *
 * CONTEXT
 *   One instance lives on Scene.  Lifetime:
 *     • trajectory_ensemble_init_at_energy(E) — called on r / n / p
 *       and SIGWINCH.  Re-seeds all N_TRAJ trajectories at the active
 *       preset's energy with y spread linearly across the seed band.
 *     • trajectory_ensemble_advance(dt) — called INT_STEPS_PER_TICK
 *       times per scene_tick (so ~28k substeps/sec at sim_fps = 60).
 *       Advances every orbit by one RK4 step, detects x-crossings,
 *       and records crossings to the SectionGrid.
 *   Const layout: count = N_TRAJ at all times (no adaptive resizing).
 *
 * MEMBER LOGIC
 *   state [i] : current 4-D phase point of trajectory i.  Mutated by
 *               hh_rk4 every substep.
 *   prev  [i] : previous 4-D phase point — needed to linear-interpolate
 *               (y, py) at the moment x crosses zero.  Updated to
 *               the pre-RK4 state at the start of every substep.
 *   prev_x[i] : previous x specifically — duplicated for the cheap
 *               sign-change predicate (prev_x < 0 && state.x ≥ 0)
 *               BEFORE invoking the full interpolation.  Could be
 *               derived from prev[i].x; the duplicate avoids a pointer
 *               chase in the hot path.
 *   count     : N_TRAJ (constant).  Kept on the struct so future
 *               adaptive-ensemble experiments have one obvious knob.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Hénon & Heiles (1964) §III — their "spread initial conditions
 *                                    at fixed E and superimpose the
 *                                    sections" methodology.
 *   [5] Poincaré (1892)            — the section-sampling idea.
 */
typedef struct {
    HHState state [N_TRAJ];
    HHState prev  [N_TRAJ];
    float   prev_x[N_TRAJ];
    int     count;
} TrajectoryEnsemble;

/* TRAJECTORY_SEED_Y_MIN / MAX — band of initial y values for the
 * ensemble seeding.  Inside the energy-allowed region for all
 * presets so px² ≥ 0 for every trajectory at seed time.  Spread
 * linearly between these bounds gives a fair sampling of the energy
 * surface at x = 0. */
#define TRAJECTORY_SEED_Y_MIN  -0.4f
#define TRAJECTORY_SEED_Y_MAX   0.6f

/* trajectory_ensemble_init_at_energy — seed N_TRAJ trajectories on
 * the x = 0 plane with y linearly distributed across the allowed
 * band, py = 0, and px chosen so each starts at exactly E_target. */
static void trajectory_ensemble_init_at_energy(TrajectoryEnsemble *ens,
                                               float E_target)
{
    ens->count = N_TRAJ;
    for (int i = 0; i < N_TRAJ; i++) {
        float fraction = (float)(i + 1) / (float)(N_TRAJ + 1);
        float y_seed   = TRAJECTORY_SEED_Y_MIN
                       + fraction * (TRAJECTORY_SEED_Y_MAX - TRAJECTORY_SEED_Y_MIN);
        HHState seed   = hh_make_state_on_x0_at_energy(y_seed, E_target);
        ens->state [i] = seed;
        ens->prev  [i] = seed;
        ens->prev_x[i] = seed.x;
    }
}

/* trajectory_is_upward_x_crossing — sign-change predicate: TRUE iff
 * the orbit just crossed x = 0 going in the +x direction.  Three
 * conditions: prev_x < 0, current x ≥ 0, current px > 0 (the third
 * picks the UPWARD branch — the standard Hénon-Heiles section
 * convention). */
static inline bool trajectory_is_upward_x_crossing(float prev_x,
                                                   const HHState *current)
{
    return prev_x < 0.0f
        && current->x >= 0.0f
        && current->px > 0.0f;
}

/* trajectory_interpolate_section_point — linear-interpolate (y, py)
 * at the moment x crosses 0 between prev and current.
 *
 *   α = -prev_x / (current_x - prev_x)   ∈ [0, 1]
 *   y_at_x0  = prev.y  + α · (current.y  - prev.y)
 *   py_at_x0 = prev.py + α · (current.py - prev.py)
 *
 * α is the fractional position of the zero crossing inside the
 * RK4 substep.  Cheap and accurate when the substep is small. */
static inline void trajectory_interpolate_section_point(
    const HHState *prev, float prev_x, const HHState *current,
    float *y_at_x0, float *py_at_x0)
{
    float dx    = current->x - prev_x;
    float alpha = (dx != 0.0f) ? -prev_x / dx : 0.0f;
    *y_at_x0    = prev->y  + alpha * (current->y  - prev->y );
    *py_at_x0   = prev->py + alpha * (current->py - prev->py);
}

/* trajectory_ensemble_advance — RK4-advance every trajectory by one
 * substep, then for each detect an upward x-crossing and record the
 * interpolated (y, py) into the SectionGrid.
 *
 * The "sample on plane crossings" idea is the POINCARÉ SECTION
 * technique — header ref [5] Poincaré (1892) Vol. III.  Converts a
 * continuous 4-D flow into a discrete 2-D map by recording the
 * orbit's intersections with a transverse surface.  Reduces the
 * dimensionality of "what to look at" while preserving the
 * topological structure (KAM tori → closed curves on the section;
 * chaos → 2-D area-filling sets on the section). */
/* trajectory_advance_one — process ONE orbit for ONE substep:
 *   STEP 1 — snapshot the pre-RK4 state (for crossing test + interp)
 *   STEP 2 — RK4 advance the live state by dt
 *   STEP 3 — if the substep just crossed x = 0 upward, interpolate
 *            (y, py) at the exact crossing and push into the section.
 * Per-orbit helper extracted so the outer loop in
 * trajectory_ensemble_advance reads as "for each orbit, advance one". */
static void trajectory_advance_one(TrajectoryEnsemble *ens, int i,
                                   SectionGrid *section, float dt)
{
    /* STEP 1 — snapshot */
    ens->prev  [i] = ens->state[i];
    ens->prev_x[i] = ens->state[i].x;

    /* STEP 2 — RK4 step */
    hh_rk4(&ens->state[i], dt);

    /* STEP 3 — detect + record section crossing */
    if (!trajectory_is_upward_x_crossing(ens->prev_x[i], &ens->state[i]))
        return;
    float y_at_x0, py_at_x0;
    trajectory_interpolate_section_point(
        &ens->prev[i], ens->prev_x[i], &ens->state[i],
        &y_at_x0, &py_at_x0);
    section_record(section, y_at_x0, py_at_x0);
}

static void trajectory_ensemble_advance(TrajectoryEnsemble *ens,
                                        SectionGrid *section, float dt)
{
    for (int i = 0; i < ens->count; i++)
        trajectory_advance_one(ens, i, section, dt);
}

/*
 * PresetState — typed wrapper around "which energy row of presets[]
 * is loaded".
 *
 * INTENT
 *   The Preset enum is just an integer — it tells you nothing about
 *   the semantics of "next", "previous", or "look up the active row
 *   in presets[]".  Wrapping it in a struct + cycle helpers makes the
 *   binding table in app_handle_key read as
 *
 *       case 'n': preset_state_cycle_next(&s->preset); scene_reset(s, ...);
 *
 *   instead of inline modular arithmetic.  This is the "Replace
 *   Primitive with Object" pattern — a one-field struct exists not to
 *   hold data but to ATTACH a named API to that data so the call sites
 *   read as intentions.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_preset_next /
 *   prev when n/p are pressed; read by scene_load_active_energy
 *   (via preset_state_active → presets[current].E → trajectory_
 *   ensemble_init_at_energy).  Also read by the HUD writers for the
 *   row-1 "preset:NAME" label and by section_paint via
 *   preset_state_active_energy() for the energy-boundary curve.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS) — cycle helpers guarantee this by
 *             modular arithmetic.  Initial value set by scene_init
 *             to PRESET_MIXED — the most pedagogically interesting
 *             opener (islands + chaos coexist).
 *
 * REFERENCES
 *   • Fowler "Refactoring" (2nd ed.), "Replace Primitive with Object".
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial)        { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)               { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)               { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const HHPreset *preset_state_active(const PresetState *p)  { return &presets[p->current]; }
static float           preset_state_active_energy(const PresetState *p)
                                                                  { return presets[p->current].E; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState (one-int wrapper + cycle helpers) but
 *   kept as a SEPARATE type because their tables differ (presets[]
 *   vs themes[]) and conflating them would just be punning.  Distinct
 *   types let scene_apply_theme() take exactly the right wrapper and
 *   refuse anything else — eliminates the bug class where a "next"
 *   key on the wrong wrapper would index past N_THEMES into N_PRESETS
 *   or vice versa.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_theme_next /
 *   prev when t/T are pressed.  After every mutation, the caller
 *   invokes palette_state_apply() → §3 theme_apply(current), which is
 *   the only place that touches ncurses init_pair() calls.  Read by
 *   HUD writers for the row-1 "theme:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§3).  Must be in
 *             [0, N_THEMES).  Initial value is 0 (set by scene_init);
 *             theme 0 ("DEFAULT") is conventionally the most legible.
 *
 * REFERENCES
 *   • Fowler "Refactoring" — same "Replace Primitive with Object"
 *     pattern.  The deliberate type-distinction between PresetState
 *     and PaletteState despite identical shape is the DDD "Tiny
 *     Types" idea: values from different domains shouldn't share a
 *     type just because the bits match.
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
 *   Bundle every piece of mutable state into one struct so the App
 *   layer owns ONE Scene (no loose globals) and the main loop drives
 *   it through a tiny named-method API.  Each sub-field is a NAMED
 *   CONCEPT:
 *     • the trajectory pool (ensemble),
 *     • the Poincaré-section accumulator (section),
 *     • the selected energy preset (preset),
 *     • the selected colour palette (palette).
 *   Presentation flags (paused) live at the top level — they belong
 *   to the Scene as a whole, not to any sub-system.
 *
 *   Decomposition axis: "what the simulation IS MADE OF":
 *     • a population of orbits being integrated (ensemble),
 *     • a record of where they've been when crossing the section
 *       (section histogram),
 *     • a selection of which experiment is loaded (preset),
 *     • a selection of which palette renders it (palette).
 *
 * CONTEXT
 *   One instance embedded inside App.  Lifetime / mutation map:
 *     • scene_tick      (~60 Hz × INT_STEPS_PER_TICK substeps,
 *                        advances .ensemble + .section)
 *     • scene_reset     (r / n / p — re-seeds ensemble, clears
 *                        section histogram)
 *     • app_handle_key  (sub-states .preset, .palette, .paused)
 *   Read by:
 *     • scene_paint     (every frame — draws all sub-systems)
 *     • HUD writers     (preset / palette / E / paused for the
 *                        status line)
 *
 * MEMBER LOGIC
 *   ensemble : §7 TrajectoryEnsemble — N_TRAJ trajectories evolving
 *              at the active preset's energy.  Mutated by scene_tick();
 *              re-seeded by scene_load_active_energy().
 *   section  : §6 SectionGrid — 2-D histogram of crossings in (y, py).
 *              One bin incremented per upward x = 0 crossing per orbit.
 *              Lives in cell coords matching the on-screen plot.
 *   preset   : §7 PresetState — which energy row of presets[] is
 *              loaded.  n/p cycle it; scene_reset() applies it.
 *   palette  : §7 PaletteState — which theme is active.  t/T cycle
 *              it; scene_apply_theme() pushes into ncurses.
 *   paused   : when true, scene_tick early-returns — physics frozen
 *              but rendering continues so the user can study the
 *              accumulated section pattern.
 *
 * REFERENCES (numbered in file-header References block)
 *   [9] Sussman & Wisdom (2014) — the "system / state / observable"
 *       triad at the simulation-architecture level.  Scene's
 *       ensemble = system + state, section = observable, preset /
 *       palette = experiment selection.
 *   • Gamma, Helm, Johnson, Vlissides (1995) "Design Patterns" —
 *     Composite pattern: one root holds a tree of named sub-objects,
 *     one tick/paint call fans out in deterministic order.
 */
typedef struct {
    TrajectoryEnsemble ensemble;
    SectionGrid        section;
    PresetState        preset;
    PaletteState       palette;
    bool               paused;
} Scene;

/* scene_load_active_energy — re-seed the ensemble at the currently-
 * selected preset's energy.  System parameters of Hénon-Heiles are
 * fixed (the Hamiltonian has no free constants); the only thing
 * preset cycling changes is which energy E the ensemble starts at. */
static void scene_load_active_energy(Scene *s)
{
    trajectory_ensemble_init_at_energy(&s->ensemble,
                                       preset_state_active_energy(&s->preset));
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int section_w, int section_h)
{
    section_reset(&s->section, section_w, section_h);
    scene_load_active_energy(s);
}

static void scene_init(Scene *s, int section_w, int section_h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;

    preset_state_init (&s->preset,  PRESET_MIXED);  /* headline preset      */
    palette_state_init(&s->palette, 0);             /* first theme          */

    scene_reset(s, section_w, section_h);
}

/* scene_tick — advance the physics + accumulate Poincaré-section
 * crossings.  INT_STEPS_PER_TICK substeps per render frame keeps
 * the orbits crisp at sim_fps = 60. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int substep = 0; substep < INT_STEPS_PER_TICK; substep++)
        trajectory_ensemble_advance(&s->ensemble, &s->section, HH_DT);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-adapter handle: ncurses lifecycle + current
 * window dimensions.
 *
 * INTENT
 *   ncurses owns the actual terminal mode and back-buffer; this
 *   struct is the THIN OWNERSHIP HANDLE that the program passes
 *   around.  Bundling (cols, rows) here lets every painter take ONE
 *   Screen parameter instead of calling getmaxyx() at every draw
 *   site.  Funnelling all init/teardown through screen_init /
 *   screen_free keeps the ncurses dependency at a single boundary —
 *   nothing else in the file calls initscr / endwin / getmaxyx.
 *
 * CONTEXT
 *   One instance lives on App.  Initialised by screen_init() during
 *   app_bootstrap().  Refreshed by screen_resize() inside
 *   app_handle_pending_resize() whenever SIGWINCH fires.  Read by
 *   section_paint and the HUD writers to clip against terminal
 *   bounds and to right-anchor HUD strings.  Freed at exit via
 *   screen_free(); endwin() also runs from atexit(cleanup) as a
 *   safety net if the program crashes before main() returns.
 *
 * MEMBER LOGIC
 *   cols : current terminal width in CHARACTER CELLS.  Updated only
 *          by screen_init / screen_resize via getmaxyx(stdscr, ...).
 *   rows : current terminal height in CHARACTER CELLS.  Row 0 is the
 *          top of the terminal; row (rows - 1) is the bottom.  The
 *          HUD reserves HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS
 *          at the bottom (see §1).
 *
 * REFERENCES (numbered in file-header References block)
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 2 — the initscr
 *        / endwin / getmaxyx / wnoutrefresh / doupdate API surface
 *        that the screen_* helpers wrap.
 */
typedef struct {
    int cols;
    int rows;
} Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* glyph_for_band — 8-tier density ramp matching the 8 colour pairs.
 * Each glyph picked for its TOTAL INK weight in a cell — the
 * progression .  :  ;  +  o  *  #  @  reads as low → high density
 * even without colour, so the eye picks up the topology of the
 * Poincaré section from the GLYPH gradient alone. */
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

/* SECTION_FRAME_*  — glyphs for the rectangular box that surrounds the
 * Poincaré section, so the (y, py) data sits inside a clearly-defined
 * viewing window instead of floating in the terminal. */
#define SECTION_FRAME_CORNER     '+'
#define SECTION_FRAME_HORIZ      '-'
#define SECTION_FRAME_VERT       '|'

/* SECTION_AXIS_*  — faint y=0 / py=0 guide lines inside the frame, so
 * the reader can locate the origin and read off symmetry by eye.
 * Drawn FIRST (back layer); density / boundary overwrite them. */
#define SECTION_AXIS_VERT        '|'
#define SECTION_AXIS_HORIZ       '-'
#define SECTION_AXIS_ORIGIN      '+'

/* SECTION_BOUNDARY_TOP / BOTTOM  — distinct glyphs for the upper
 * (py = +py_max) and lower (py = -py_max) energy-boundary curves.
 * Chosen so they read as "edge-of-allowed-region" markers rather
 * than competing with the density ramp:
 *
 *   '''  sits at the TOP of the cell box → marks the upper bound
 *   ,,,  sits at the BOTTOM of the cell box → marks the lower bound
 */
#define SECTION_BOUNDARY_TOP     '\''
#define SECTION_BOUNDARY_BOTTOM  ','

/*
 * PoincareViewport — the on-screen rectangle that maps the (y, py)
 * phase plane to terminal cells, plus a back-pointer to the
 * SectionGrid whose hit-counts get rendered into it.
 *
 * INTENT
 *   Bundles the four numbers (origin cell + cell dimensions) that
 *   every section painter needs.  Without this struct each painter
 *   would take 4 loose ints and the (y, py) → cell conversion would
 *   be scattered.  With it, viewport construction happens ONCE per
 *   frame in section_paint and every painter takes one viewport.
 *
 *   The viewport's PURPOSE is to render the Poincaré-section figure
 *   the way Hénon & Heiles drew it in their original 1964 plates:
 *   (y, py) on a 2-D Cartesian frame, energy boundary visible, with
 *   each orbit's hit-density showing through.  This struct + its
 *   helpers make that figure reproducible per-frame.
 *
 * MEMBER LOGIC
 *   gx0, gy0 : cell coords of the data rectangle's top-left corner.
 *              Centred inside the drawable band, clipped one cell
 *              inside so the frame fits around the data.
 *   w, h     : data-rectangle width / height in cells.  Matches the
 *              underlying SectionGrid's w / h.
 *
 * REFERENCES (numbered in file-header References block)
 *   [1]  Hénon & Heiles (1964) Figs. 3-4 — the canonical (y, py)
 *        plots this viewport reproduces in ASCII.
 *   [5]  Poincaré (1892) — original conception of "view the dynamics
 *        by sampling on a transverse plane".
 *   [10] Gookin NCurses Ch. 4 — mvaddch / wnoutrefresh model that
 *        every painter funnels through.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} PoincareViewport;

/* viewport_centered_origin_x — column offset that centres a
 * width-w inner rectangle inside an outer-width `outer_cols`,
 * with a 1-cell margin on the left for the frame. */
static inline int viewport_centered_origin_x(int outer_cols, int w)
{
    int gx0 = (outer_cols - w) / 2;
    return (gx0 < 1) ? 1 : gx0;
}

/* viewport_centered_origin_y_in_drawable — row offset that centres
 * a height-h inner rectangle inside the drawable band (rows minus
 * HUD reservation), with a 1-cell margin on top for the frame. */
static inline int viewport_centered_origin_y_in_drawable(int outer_rows, int h)
{
    int gy0 = ((outer_rows - HUD_BAND_RESERVED_ROWS) - h) / 2 + HUD_TOP_ROWS;
    return (gy0 < HUD_TOP_ROWS + 1) ? HUD_TOP_ROWS + 1 : gy0;
}

/* poincare_viewport_build — compute the viewport for the current
 * frame from the section's data dimensions and the terminal size. */
static PoincareViewport poincare_viewport_build(const SectionGrid *section,
                                                int cols, int rows)
{
    PoincareViewport vp;
    vp.w   = section->w;
    vp.h   = section->h;
    vp.gx0 = viewport_centered_origin_x(cols, vp.w);
    vp.gy0 = viewport_centered_origin_y_in_drawable(rows, vp.h);
    return vp;
}

/* phase_to_cell_x / _y — convert a single PHYSICAL (y, py) coordinate
 * to its cell index inside the viewport (LOCAL 0..w-1 / 0..h-1).
 * The terminal y-axis is flipped at plot time (high py → high row
 * from the top) so positive momentum reads as "above" the py=0 axis. */
static inline int phase_to_cell_x(const PoincareViewport *vp, float y_phys)
{
    int cx = (int)((y_phys - Y_MIN) / (Y_MAX - Y_MIN) * (float)vp->w);
    if (cx < 0)         cx = 0;
    if (cx >= vp->w)    cx = vp->w - 1;
    return cx;
}
static inline int phase_to_cell_y(const PoincareViewport *vp, float py_phys)
{
    int cy = (int)((py_phys - PY_MIN) / (PY_MAX - PY_MIN) * (float)vp->h);
    if (cy < 0)         cy = 0;
    if (cy >= vp->h)    cy = vp->h - 1;
    return cy;
}

/* poincare_plot — write one glyph at the SCREEN cell corresponding to
 * physical (y, py).  Single funnel point for the viewport's screen-
 * coord transform: gy0 + (h - 1 - cy) flips the y-axis (cells grow
 * down, py grows up). */
static inline void poincare_plot(const PoincareViewport *vp,
                                 float y_phys, float py_phys, chtype glyph)
{
    int cx = phase_to_cell_x(vp, y_phys);
    int cy = phase_to_cell_y(vp, py_phys);
    mvaddch(vp->gy0 + (vp->h - 1 - cy), vp->gx0 + cx, glyph);
}

/* viewport_paint_vertical_line — paint a vertical line at LOCAL
 * column cx_local (0..w-1) across the viewport's full height.  Caller
 * sets the attribute/colour before invoking. */
static void viewport_paint_vertical_line(const PoincareViewport *vp,
                                         int cx_local, chtype glyph)
{
    for (int y = 0; y < vp->h; y++)
        mvaddch(vp->gy0 + (vp->h - 1 - y), vp->gx0 + cx_local, glyph);
}

/* viewport_paint_horizontal_line — paint a horizontal line at LOCAL
 * row cy_local (0..h-1) across the viewport's full width. */
static void viewport_paint_horizontal_line(const PoincareViewport *vp,
                                           int cy_local, chtype glyph)
{
    for (int x = 0; x < vp->w; x++)
        mvaddch(vp->gy0 + (vp->h - 1 - cy_local), vp->gx0 + x, glyph);
}

/* paint_section_frame — single-line ASCII box around the data region.
 * '+'-cornered, '-'/'|'-edged.  Bright PAIR_BOUNDARY + A_BOLD so the
 * frame anchors the eye to "this is the data". */
/* paint_frame_top_and_bottom_edges — '-' edges immediately above and
 * below the data rectangle.  Each edge runs the full data width. */
static void paint_frame_top_and_bottom_edges(const PoincareViewport *vp)
{
    int top_row    = vp->gy0 - 1;
    int bottom_row = vp->gy0 + vp->h;
    for (int x = 0; x < vp->w; x++) {
        mvaddch(top_row,    vp->gx0 + x, SECTION_FRAME_HORIZ);
        mvaddch(bottom_row, vp->gx0 + x, SECTION_FRAME_HORIZ);
    }
}

/* paint_frame_left_and_right_edges — '|' edges immediately left and
 * right of the data rectangle.  Each edge runs the full data height. */
static void paint_frame_left_and_right_edges(const PoincareViewport *vp)
{
    int left_col  = vp->gx0 - 1;
    int right_col = vp->gx0 + vp->w;
    for (int y = 0; y < vp->h; y++) {
        mvaddch(vp->gy0 + y, left_col,  SECTION_FRAME_VERT);
        mvaddch(vp->gy0 + y, right_col, SECTION_FRAME_VERT);
    }
}

/* paint_frame_corners — '+' glyphs at the four corner cells where
 * horizontal and vertical edges meet.  Drawn after the edges so the
 * corner glyph wins over the edge glyphs at the meeting cells. */
static void paint_frame_corners(const PoincareViewport *vp)
{
    int top_row    = vp->gy0 - 1,    bottom_row = vp->gy0 + vp->h;
    int left_col   = vp->gx0 - 1,    right_col  = vp->gx0 + vp->w;
    mvaddch(top_row,    left_col,  SECTION_FRAME_CORNER);
    mvaddch(top_row,    right_col, SECTION_FRAME_CORNER);
    mvaddch(bottom_row, left_col,  SECTION_FRAME_CORNER);
    mvaddch(bottom_row, right_col, SECTION_FRAME_CORNER);
}

/* paint_section_frame — single-line ASCII box around the data region.
 *
 *   STEP 1 — '-' top + bottom edges
 *   STEP 2 — '|' left + right edges
 *   STEP 3 — '+' corners (overdrawn last so they win at the joins)
 *
 * Bright PAIR_BOUNDARY + A_BOLD so the frame anchors the eye to
 * "this is the data". */
static void paint_section_frame(const PoincareViewport *vp)
{
    attron(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
    paint_frame_top_and_bottom_edges(vp);
    paint_frame_left_and_right_edges(vp);
    paint_frame_corners             (vp);
    attroff(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
}

/* paint_section_axes — faint y=0 (vertical) and py=0 (horizontal)
 * reference lines INSIDE the frame.  Drawn BEFORE density so the
 * data points overwrite the axes where they coincide. */
static void paint_section_axes(const PoincareViewport *vp)
{
    int y_zero_col   = phase_to_cell_x(vp, 0.0f);
    int py_zero_row  = phase_to_cell_y(vp, 0.0f);
    int origin_screen_row = vp->gy0 + (vp->h - 1 - py_zero_row);

    attron(COLOR_PAIR(PAIR_BOUNDARY));
    viewport_paint_vertical_line  (vp, y_zero_col,  SECTION_AXIS_VERT);
    viewport_paint_horizontal_line(vp, py_zero_row, SECTION_AXIS_HORIZ);
    mvaddch(origin_screen_row, vp->gx0 + y_zero_col, SECTION_AXIS_ORIGIN);
    attroff(COLOR_PAIR(PAIR_BOUNDARY));
}

/* energy_max_py_at_y — solve  H(0, y, px=0, py) = E  for py:
 *   py = √(2(E - V(0, y)))      (positive branch)
 * Returns -1 if V(0, y) > E (energetically forbidden at this y). */
static inline float energy_max_py_at_y(float y, float E)
{
    float V = hh_potential(0.0f, y);
    if (V > E) return -1.0f;
    return sqrtf(2.0f * (E - V));
}

/* CELL_CENTER_SAMPLE_FRACTION — the 0.5f offset that picks the
 * MIDDLE of each cell when converting cell-index → physical
 * coordinate.  Using 0.0 (cell's left edge) would shift the energy
 * boundary half a cell to the left of where its data lands. */
#define CELL_CENTER_SAMPLE_FRACTION  0.5f

/* viewport_cell_x_to_y_phys — physical y coordinate at the centre of
 * data column cx.  Inverse of phase_to_cell_x. */
static inline float viewport_cell_x_to_y_phys(const PoincareViewport *vp, int cx)
{
    return Y_MIN + ((float)cx + CELL_CENTER_SAMPLE_FRACTION) / (float)vp->w
                 * (Y_MAX - Y_MIN);
}

/* paint_energy_boundary — for each data column (y), mark the upper
 * limit +py_max and its mirror -py_max with distinct glyphs.  Reads
 * as "edge of the energetically-allowed region".
 *
 *   STEP 1 — for each column, find its physical y coordinate
 *   STEP 2 — solve  H = E  for py_max; skip if energetically forbidden
 *   STEP 3 — plot the upper (+py_max) and lower (-py_max) boundary cells
 */
static void paint_energy_boundary(const PoincareViewport *vp, float E)
{
    attron(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
    for (int cx = 0; cx < vp->w; cx++) {
        /* STEP 1 — column → physical y */
        float y_phys = viewport_cell_x_to_y_phys(vp, cx);

        /* STEP 2 — py_max = √(2(E - V(0, y))); -1 means forbidden */
        float py_max = energy_max_py_at_y(y_phys, E);
        if (py_max < 0.0f) continue;

        /* STEP 3 — upper + lower perimeter glyphs */
        poincare_plot(vp, y_phys, +py_max, SECTION_BOUNDARY_TOP);
        poincare_plot(vp, y_phys, -py_max, SECTION_BOUNDARY_BOTTOM);
    }
    attroff(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
}

/* screen_row_inside_drawable — TRUE iff terminal row `sy` lies inside
 * the drawable band (between HUD_TOP_ROWS and rows - HUD_BOTTOM_ROWS).
 * Used to clip painter scanlines against the HUD reservation. */
static inline bool screen_row_inside_drawable(int sy, int rows)
{
    return sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* screen_col_inside_terminal — TRUE iff terminal column `sx` lies in
 * the writable column range [0, cols). */
static inline bool screen_col_inside_terminal(int sx, int cols)
{
    return sx >= 0 && sx < cols;
}

/* paint_density_cell — emit ONE filled-cell glyph at the band-coloured
 * density tier corresponding to `hits`.  Returns silently if the bin
 * is empty (density_band → -1).  Single-cell helper extracted so the
 * outer 2-D scan in paint_density_cells reads as a clean nested loop. */
static void paint_density_cell(int sy, int sx, uint16_t hits)
{
    int band = density_band(hits);
    if (band < 0) return;
    int pair = PAIR_DENS_BASE + band;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph_for_band(band));
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* paint_density_cells — main data layer.
 *
 *   STEP 1 — for each row of the data grid, compute its screen row
 *            and skip if outside the drawable band
 *   STEP 2 — for each column inside that row, compute the screen
 *            column and skip if outside the terminal
 *   STEP 3 — emit a band-coloured glyph for the cell's hit count
 *            (paint_density_cell skips empty bins so axes / boundary
 *            underneath stay visible)
 */
static void paint_density_cells(const PoincareViewport *vp,
                                const SectionGrid *g,
                                int cols, int rows)
{
    for (int y = 0; y < vp->h; y++) {
        int sy = vp->gy0 + y;
        if (!screen_row_inside_drawable(sy, rows)) continue;

        for (int x = 0; x < vp->w; x++) {
            int sx = vp->gx0 + x;
            if (!screen_col_inside_terminal(sx, cols)) continue;

            paint_density_cell(sy, sx, g->hits[y * g->w + x]);
        }
    }
}

/* section_paint — composite renderer for the Poincaré section.
 *
 *   STEP 1 — build the viewport (centre data inside the drawable band)
 *   STEP 2 — axes:     faint y=0 / py=0 guide lines INSIDE the frame
 *   STEP 3 — boundary: ' and , marking the (E, V) energy perimeter
 *   STEP 4 — density:  8-tier glyph ramp coloured by hit count
 *   STEP 5 — frame:    '+'-cornered box one cell outside the data
 *
 * Order matters: back-to-front so the high-information density layer
 * sits on top of the static orientation layers (axes, boundary), and
 * the frame on top of everything to anchor the eye. */
static void section_paint(const SectionGrid *section, int cols, int rows, float E)
{
    PoincareViewport vp = poincare_viewport_build(section, cols, rows);
    paint_section_axes   (&vp);
    paint_energy_boundary(&vp, E);
    paint_density_cells  (&vp, section, cols, rows);
    paint_section_frame  (&vp);
}

/* hud_paint_top_left_title — fixed title in row 0 column HUD_LEFT_MARGIN. */
static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " HÉNON-HEILES (Poincaré) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_format_top_right_status — format the dynamic status string
 * (fps, Hz, preset name + index, E) into a stack buffer.  Pure
 * formatter — no ncurses calls.  Pulled out so hud_top reads as
 * "format then paint", not a tangled snprintf+positioning block. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const HHPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  E:%.3f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)active->E);
}

/* hud_paint_top_right — paint a right-anchored string at row 0.
 * Right-anchor formula: column = cols - strlen, clipped at 0 so a
 * narrow terminal doesn't push the start negative. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row-0 HUD driver.
 *
 *   STEP 1 — top-left title cell (constant text)
 *   STEP 2 — format the dynamic status into a buffer
 *   STEP 3 — paint the buffer right-anchored to `cols`
 */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* HUD_PARAM_CELL_WIDTH_* — printed widths of the row-1 cells including
 * their leading + trailing spaces.  Sized to the fixed-width %-8s
 * format specifiers so the layout doesn't shift as preset / theme
 * names change. */
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

/* hud_param — row-1 HUD: three side-by-side cells.
 *
 *   STEP 1 — PRESET name cell    (bold, primary)
 *   STEP 2 — THEME name cell     (bold, primary)
 *   STEP 3 — section convention + trajectory count + E_escape
 *            (non-bold, secondary)
 */
static void hud_param(const Scene *s)
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

    /* STEP 3 — section convention + diagnostics */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " section:x=0 (ẋ>0)   trajs:%d   E_escape≈0.1667 ",
             s->ensemble.count);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    section_paint(&s->section, sc->cols, sc->rows,
                  preset_state_active_energy(&s->preset));
    hud_top  (sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint (sc->rows);
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
 *   other mutable global state.  This makes the program read top-
 *   down: main() = "drive the App"; each helper = "do one named thing
 *   to the App".
 *
 *   Concentrating mutable state in one struct disciplines signal
 *   handling: SIGWINCH and SIGINT handlers may only touch the
 *   volatile sig_atomic_t fields on g_app — everything else is
 *   reachable only through main-loop functions.
 *
 * CONTEXT
 *   Exactly ONE instance exists: the file-scope g_app (immediately
 *   below this struct).  Signal handlers need a callable-from-signal
 *   target, and POSIX async-signal-safety requires that to be a
 *   global; hence g_app.  All non-signal accesses pass an App*
 *   explicitly so the rest of the file remains free of global lookups.
 *
 *   Lifetime:
 *     app_bootstrap    → fills scene + screen + sim_fps + map dims
 *     main loop        → mutates everything except 'running', which
 *                        is read in the while-condition
 *     atexit(cleanup)  → endwin() safety net
 *
 * MEMBER LOGIC
 *   scene       : §8 Scene — ensemble, section, preset, palette,
 *                 paused.  The simulation; what the user sees.
 *   screen      : §9 Screen — cols / rows + ncurses lifecycle handle.
 *   sim_fps     : current PHYSICS tick rate in Hz.  Distinct from
 *                 RENDER_FPS_TARGET — the integrator can step at any
 *                 rate while the renderer stays at 60 fps.  Adjusted
 *                 by ] / [.  Clamped to [SIM_FPS_MIN, MAX].
 *   map_w       : cells of horizontal SIMULATION area (= section.w).
 *   map_h       : cells of vertical SIMULATION area (= section.h =
 *                 screen.rows - HUD_BAND_RESERVED_ROWS - SECTION_FRAME_
 *                 RESERVED).  Cached on App so scene_paint has stable
 *                 inputs across the frame; recomputed by
 *                 app_pick_map_size on resize.
 *   running     : 0 → main loop exits.  Set by SIGINT/SIGTERM handler
 *                 and by app_handle_key on q/ESC.  Declared volatile
 *                 sig_atomic_t for async-signal safety per
 *                 POSIX.1-2017 §2.4.3.
 *   need_resize : 1 → next frame triggers a SIGWINCH rebuild via
 *                 app_handle_pending_resize.  Set by the SIGWINCH
 *                 handler.  Same volatile sig_atomic_t requirement.
 *
 * REFERENCES (numbered in file-header References block)
 *   [8]  Fiedler "Fix Your Timestep!" — explains why sim_fps and
 *        RENDER_FPS_TARGET are kept as INDEPENDENT knobs on App.
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 11 (Signal
 *        Handling) — the .need_resize + app_handle_pending_resize
 *        pattern.
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

/* SECTION_FRAME_RESERVED — cells the section's '+'/'-'/'|' frame eats
 * on each axis (one per side: left + right, top + bottom). */
#define SECTION_FRAME_RESERVED  2

/* main_install_signal_handlers — wire SIGINT/TERM → quit and
 * SIGWINCH → "resize on next frame". */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_pick_map_size — clamp terminal dims minus HUD reservation
 * minus the frame margin into the (MIN, MAX) data envelope. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols - SECTION_FRAME_RESERVED;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS - SECTION_FRAME_RESERVED;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
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

/* app_compute_frame_dt — wall-clock since last frame, capped at
 * SIM_MAX_FRAME_DT_MS so paused-then-resumed can't burst-feed. */
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
 * See header ref [8] Fiedler "Fix Your Timestep!".  Variable-dt
 * would make the section non-deterministic — different render
 * speeds would sample the cos(ωt)... well, there's no drive here,
 * but the integrator's energy-drift behaviour DOES depend on
 * step size.  Fixed dt keeps the figure reproducible across machines. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — refresh displayed fps every FPS_UPDATE_MS ms. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep so render loop sticks at
 * RENDER_FPS_TARGET.  Sleep BEFORE drawing so I/O time doesn't add
 * jitter. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint scene + HUD and flip the back buffer. */
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
static void app_toggle_pause   (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_ensemble (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }

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
    case 'r': case 'R':  app_reset_ensemble   (app); break;
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
 * FrameClock — wall-clock + accumulator state threaded through the
 * per-frame helpers.
 *
 * INTENT
 *   Bundle the 5 timing locals that main() would otherwise carry loose
 *   (frame_time, sim_accum, fps_accum, frame_count, fps_display) into
 *   one named concept.  Mirrors the App / Scene pattern: every piece
 *   of mutable state belongs to a NAMED OBJECT, not to main's stack
 *   frame.  The main loop becomes a 10-line pseudocode driver over
 *   FrameClock + App.
 *
 *   The clock separates TWO independent timescales:
 *     • a wall-clock RENDER timeline (frame_time, fps_*) for what the
 *       user sees,
 *     • a PHYSICS accumulator (sim_accum) for the fixed-step Fiedler
 *       drain inside the loop.
 *
 * CONTEXT
 *   One instance lives on main()'s stack.  Lifetime:
 *     frame_clock_init                 — once before the loop
 *     frame_clock_reset_after_resize   — after each SIGWINCH rebuild
 *     frame_clock_advance              — once per frame
 *     app_drain_fixed_timestep         — reads / mutates .sim_accum
 *     app_update_fps_meter             — reads / mutates .fps_*
 *     app_throttle_to_render_target    — reads .frame_time
 *     app_present_frame                — reads .fps_display
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock timestamp (ns) at the boundary of the
 *                 LAST frame.  app_compute_frame_dt subtracts the
 *                 current clock from this to get dt.
 *   sim_accum   : leftover ns NOT YET consumed by a physics tick.
 *                 The Fiedler fixed-step accumulator (header ref [8]).
 *   fps_accum   : ns of frame time accumulated since the last fps
 *                 readout refresh.
 *   frame_count : number of frames in the current fps window.
 *   fps_display : most recent fps figure, smoothed by the
 *                 FPS_UPDATE_MS window length.  Displayed in the
 *                 top-right HUD.
 *
 * REFERENCES (numbered in file-header References block)
 *   [8] Fiedler "Fix Your Timestep!" — accumulator pattern that
 *       .sim_accum implements.
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
 *   bootstrap (curses + scene)
 *   init frame clock
 *   while running:
 *     if pending resize → rebuild screen, reset clock
 *     dt ← wall-clock since last frame
 *     advance clock; drain fixed-timestep physics; update fps meter
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
