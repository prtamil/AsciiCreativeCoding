/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ising.c — Ising Model  (Magnetic Phase Transition)
 *
 * ── WHAT YOU SEE ON SCREEN ───────────────────────────────────────────
 *  A full-screen grid where every character is a tiny magnet ("spin")
 *  pointing UP (one colour + glyph, e.g. green '#') or DOWN (the
 *  other, e.g. dim green '.').  The picture is alive — cells flicker
 *  as spins flip thousands of times per second.
 *
 *  Three visual regimes, set by temperature T:
 *
 *    HOT  (T > 2.27, the default start)  →  the lattice looks like
 *      coloured TV static.  Up/down are ~50/50, no shape survives more
 *      than a blink.  HUD tag reads "disordered", |M| ≈ 0, sparkline
 *      flat near the floor.
 *
 *    COOL (press 'c', or hold ↓)  →  big single-colour BLOBS ("domains")
 *      grow and swallow each other.  Bright outlined cells trace the
 *      boundary where two domains meet — that's where the action is;
 *      interior cells go dim and recede.  HUD reads "ordered", |M|
 *      climbs from 0 toward 1, sparkline ramps up.
 *
 *    NEAR Tc (≈ 2.27, press 'c' slowly with ↓)  →  fractal patches at
 *      every size flicker in and out — the famous critical regime.
 *      HUD reads "≈Tc".
 *
 *  Top HUD row: temperature, phase tag, |M|, 24-frame |M| sparkline
 *  (watch it climb as you cool past Tc — that's the spontaneous-
 *  magnetisation breakthrough), sweep count, active pattern, theme.
 *  Bottom row: all keys.  Press 'n' to cycle 10 starting shapes — the
 *  Bubble shrinks under surface tension, Rings die one-by-one from
 *  the inside, Halves roughen along the wall, Cluster cascades small
 *  bubbles first and large last (phase-ordering kinetics in action).
 * ─────────────────────────────────────────────────────────────────────
 *
 * Each cell is a spin: +1 (up) or −1 (down).  The Monte Carlo Metropolis
 * algorithm proposes random single-spin flips with acceptance probability
 *
 *   P(flip) = min(1,  exp(−ΔE / kT))   where  ΔE = 2·s·Σ_nbr
 *
 * At high temperature T, spins are random (paramagnetic phase).
 * Cool below the critical temperature Tc = 2 / ln(1+√2) ≈ 2.2692 and
 * magnetic domains spontaneously appear (ferromagnetic phase).  The
 * exact analytical Tc is Onsager's 1944 result [2].
 *
 * Keys: q quit  spc pause  n/p pattern  r reset  ↑↓ temperature  h hot  c cold
 * t/T theme
 *
 * Patterns (n/p cycle, 0..9, simple → complex):
 *   0 Random   1 All Up   2 All Down  3 Checker  4 Stripes-H
 *   5 Stripes-V 6 Halves  7 Bubble    8 Rings    9 Cluster
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/ising.c \
 *       -o ising -lncurses -lm
 *
 * §1 config  §2 clock  §3 scene  §4 color  §5 grid  §6 draw  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Metropolis Monte Carlo (MCMC) [3].
 *                  Randomly propose a spin flip; accept with probability
 *                  P = min(1, exp(−ΔE / kT)).  Over many steps this
 *                  samples the Boltzmann distribution at temperature T.
 *                  Not a real-time physics simulation — it's a
 *                  statistical sampler converging toward thermal
 *                  equilibrium [4, 5].  Boltzmann factors precomputed
 *                  once per T change (g_scene.boltz[]) so the inner loop is
 *                  branch-free except for the accept/reject coin.
 *
 * Physics        : 2D Ising model of ferromagnetism [1].
 *                  Each spin s = ±1 interacts with its 4 nearest
 *                  neighbours on a toroidal lattice.  ΔE = 2·s·Σ_nbr —
 *                  the energy cost of flipping one spin given its
 *                  environment.  Below Tc = 2 / ln(1+√2) ≈ 2.2692 (in
 *                  units where J=k_B=1, Onsager's exact 1944 result
 *                  [2]), large aligned domains spontaneously appear —
 *                  the textbook 2nd-order phase transition.  As T
 *                  approaches Tc from below, the correlation length
 *                  diverges and the domain structure becomes scale-
 *                  free (fractal).
 *
 * Math           : The ΔE formula exploits H = −J·Σ s_i·s_j (sum of
 *                  pairwise products).  Flipping spin i changes H by
 *                  2·J·s_i·Σ_nbr s_j; with J=1 this is ΔE = 2·s·Σ_nbr,
 *                  taking values in {−8, −4, 0, +4, +8} on a square
 *                  lattice.  Only dE = +4 and dE = +8 need expf() —
 *                  ΔE ≤ 0 flips are always accepted.
 *
 * Patterns       : 10 initial conditions cycle with n / p (ordering
 *                  simple → complex by visual structure & relaxation
 *                  dynamics).  Surface-tension / curvature-driven
 *                  motion of domain walls [6] is the dominant
 *                  phase-ordering mechanism — a circular droplet of
 *                  radius r shrinks with lifetime ∝ r², small bubbles
 *                  in pat_cluster vanish first while large ones
 *                  persist for many sweeps.
 *
 * Rendering      : Boundary-aware lattice highlighting — each cell's
 *                  ncurses attribute (cell_attr) is picked from the
 *                  neighbour-agreement count: 4 → A_DIM (interior),
 *                  3 → A_NORMAL, ≤2 → A_BOLD (domain wall / frustrated
 *                  → POPS).  The eye is drawn to where the physics is.
 *                  |M| sparkline [7] (24-sample ASCII bars in the HUD)
 *                  traces the order parameter over time so the
 *                  spontaneous-magnetisation breakthrough below Tc is
 *                  immediately legible.  10 themes [8] give the lattice
 *                  visual identity without changing the simulation.
 *
 * Performance    : metropolis_attempts_per_frame() = N_cells ×
 *                  FLIPS_PER_CELL / 1000 attempts per render frame
 *                  (the /1000 throttle leaves CPU headroom on large
 *                  terminals while still showing visible domain
 *                  dynamics at 30 fps).  Each attempt is one cached
 *                  Boltzmann-table lookup plus one rand() comparison
 *                  — branch-free aside from the accept/reject coin.
 *
 * References (cite inline as [n]):
 *
 *   [1] Ising, E. (1925) — "Beitrag zur Theorie des Ferromagnetismus",
 *       *Zeitschrift für Physik* 31 (1), 253–258.  The original 1D
 *       paper that defined the model (and famously concluded it has
 *       no phase transition — true in 1D, false in 2D).  Backs the
 *       Hamiltonian and the ±1 spin convention.
 *
 *   [2] Onsager, L. (1944) — "Crystal Statistics. I. A Two-Dimensional
 *       Model with an Order-Disorder Transition", *Physical Review*
 *       65, 117–149.  The tour-de-force exact solution of the 2D
 *       Ising model: closed-form free energy, critical exponents, and
 *       the analytical Tc = 2 / ln(1+√2) hard-coded in T_CRIT.
 *
 *   [3] Metropolis, N.; Rosenbluth, A. W.; Rosenbluth, M. N.;
 *       Teller, A. H.; Teller, E. (1953) — "Equation of State
 *       Calculations by Fast Computing Machines", *J. Chemical
 *       Physics* 21 (6), 1087–1092.  The original Metropolis
 *       acceptance rule P = min(1, exp(−ΔE/kT)) — the heart of
 *       grid_step.  Backs the entire MCMC framework.
 *
 *   [4] Newman, M. E. J.; Barkema, G. T. (1999) — *Monte Carlo Methods
 *       in Statistical Physics*, Oxford University Press.  Practical
 *       handbook for MCMC on lattice systems: precomputed-Boltzmann
 *       tricks, equilibration heuristics, autocorrelation pitfalls.
 *       Chapter 3 covers the Ising-specific implementation we use.
 *
 *   [5] Landau, D. P.; Binder, K. (2014) — *A Guide to Monte Carlo
 *       Simulations in Statistical Physics*, 4th ed., Cambridge.
 *       The canonical practitioner's text with extensive 2D-Ising
 *       case studies; backs the FLIPS_PER_CELL = 50 sweep-rate
 *       choice and the qualitative visualisation strategy.
 *
 *   [6] Bray, A. J. (1994) — "Theory of Phase-Ordering Kinetics",
 *       *Advances in Physics* 43 (3), 357–459.  The canonical review
 *       of domain coarsening: Allen-Cahn curvature-driven motion
 *       (bubble lifetime ∝ r²), L(t) ∝ t^(1/2) growth law, multi-
 *       scale relaxation in cluster patterns.  Backs the dynamics
 *       you watch in patterns 6, 7, 8, 9.
 *
 *   [7] Tufte, E. R. (2006) — *Beautiful Evidence*, Graphics Press.
 *       Coins the term "sparkline" (Ch. 2) — a small, intense,
 *       word-sized graphic that gives a time series at a glance.
 *       Backs the |M| history bars in the HUD.
 *
 *   [8] Ware, C. (2020) — *Information Visualization: Perception for
 *       Design*, 4th ed., Morgan Kaufmann.  Ch. 4 on colour pairs
 *       and perceptual contrast — backs the up/down two-tone theme
 *       table where each theme picks two hues guaranteed to read
 *       against each other.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_W_MAX 200
#define GRID_H_MAX 60
#define HUD_ROWS 2
#define N_THEMES 10
#define N_PATTERNS 10
#define STRIPE_W 6 /* width of horizontal / vertical / diagonal stripes */

/* T_INIT: starting temperature (in units where J = k_B = 1).
 * 3.0 > T_CRIT → starts in the disordered (paramagnetic) phase.
 * The user can cool down with ↓ to watch domain nucleation occur.         */
#define T_INIT 3.0f

/* T_CRIT: 2D Ising critical temperature = 2 / ln(1 + √2) ≈ 2.2692.
 * Exact analytical result by Lars Onsager (1944).
 * Above T_CRIT: spins disordered, net magnetisation ≈ 0.
 * Below T_CRIT: long-range order forms, |⟨s⟩| → 1 as T → 0.
 * At T_CRIT exactly: scale-free domain structure (fractal-like).          */
#define T_CRIT 2.269f

#define T_MIN 0.1f /* avoid T→0 where exp() arguments → ∞             */
#define T_MAX 5.0f
#define T_STEP 0.05f /* temperature adjustment per keypress              */

/* FLIPS_PER_CELL: simulation-rate knob, raw budget BEFORE the /1000
 * throttle inside metropolis_attempts_per_frame() — so the effective
 * rate is FLIPS_PER_CELL/1000 attempts per cell per frame.  Bump it
 * to equilibrate faster (more CPU per frame); divide by less to make
 * the lattice churn harder. */
#define FLIPS_PER_CELL 50
#define RENDER_NS (1000000000LL / 30)

enum {
  CP_UP = 1,   /* up-spin glyph   — theme up_fg                       */
  CP_DN = 2,   /* down-spin glyph — theme dn_fg                       */
  CP_HUD = 3,  /* canonical HUD yellow (226) — status labels & values */
  CP_HINT = 4, /* canonical key-legend cyan (51) — neutral action keys*/
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  scene state — Scene struct + the single g_scene instance           */
/* ===================================================================== */

/* SPARK_LEN — magnetisation sparkline history length.  Defined here
 * (above the Scene struct) because Scene.mhist[] needs it to size.
 * 24 frames @ 30 fps ≈ 0.8 s of trend visibility — long enough to see
 * |M| ramping up as you cool past Tc, short enough to fit beside the
 * rest of the HUD on an 80-column terminal.                          */
#define SPARK_LEN 24

/* ── Scene: unified scene-state container ────────────────────────────── *
 *
 * The single container for everything mutable in the world the demo
 * simulates and paints.  Every world-state member is reached as
 * g_scene.<field>, so when reading any function you can tell at a
 * glance which writes touch the scene and which are pure locals.
 *
 * WHY a struct (and not just discipline + the g_ prefix):
 *
 *   1. CONTAINMENT — one declaration, one initial-state literal.  No
 *      new global can sneak into the "world" without being added to
 *      the struct, so "what does the simulation consist of?" stays
 *      truthful as the file grows.
 *
 *   2. PORTABILITY — a single Scene * argument would turn the head-
 *      less sampler into a pure function you could call with a
 *      different Scene to run two lattices side by side, A/B-compare
 *      temperatures, or replay from a saved snapshot.  We don't pass
 *      it explicitly yet (file-scope g_scene), but the door is open.
 *
 *   3. DOCUMENTATION LOCALITY — sim-state and render-state fields
 *      sit in adjacent blocks here, so "what is the world made of"
 *      fits on one screen rather than being scattered across the
 *      color/grid/draw sections.
 *
 * WHY ONE Scene (and not separate SimState + RenderState structs):
 *   the two halves share the same lifetime — born together at init,
 *   die together at endwin(), resize together on SIGWINCH.  Splitting
 *   them would force a third aggregating struct for zero benefit.  We
 *   mark the sim/render divide with a comment divider INSIDE the
 *   struct instead.
 *
 * LOCALITY INTENT: simulation members come first, render members
 * second, with a clear divider between them.  The Metropolis sampler
 * must NEVER read a field from the render half — that would couple
 * physics to presentation.  The renderer IS allowed to read sim
 * fields freely (that's its job).
 *
 * WHY this section sits ABOVE §4 color: §4 color_init() reads
 * g_scene.theme to apply the initial palette, so the Scene type and
 * its single instance must be in scope before §4 begins.
 *
 * EXCLUDED from Scene (and why):
 *   - g_quit / g_resize: volatile sig_atomic_t flags written from
 *     signal handlers.  Kept as free file-scope globals because
 *     sig_atomic_t access semantics are easiest to reason about at
 *     file scope, and they're lifecycle/control flags, not world
 *     state.
 *   - k_themes[], PATTERNS[]: immutable tables, not scene state.
 *
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {

  /* ── SIMULATION HALF ─────────────────────────────────────────────── *
   * Everything the Metropolis sampler [3] needs to advance the lattice
   * by one frame.  Pure physics — no terminal, no UI.  Listed in
   * dependency order: lattice → live dims → temperature → diagnostic
   * counter → per-T acceptance cache.                                 */

  signed char spin[GRID_H_MAX][GRID_W_MAX];
  /* The lattice itself.  ±1 per cell.  signed char (NOT int) for
   * L1-cache density: 200×60 = 12 KB total, comfortably resident.
   * The Metropolis hot loop does one random read + four neighbour
   * reads per attempt at ~18M attempts/s on one thread — staying
   * in L1 is what makes that rate possible.  Toroidal boundary
   * handled at read time (modulo on the index), not stored.      */

  int gh, gw;
  /* Effective lattice dims.  Recomputed on SIGWINCH from
   * (screen rows − HUD_ROWS, screen cols), clamped to
   * GRID_*_MAX.  The spin array above is dimensioned for the
   * maximum so resize never reallocates; these counters are
   * how far the hot loops actually iterate.                     */

  float temp;
  /* Temperature T in units where J = k_B = 1 — DIMENSIONLESS,
   * not SI Kelvin.  Universal convention in lattice-model
   * literature so the critical point Tc ≈ 2.269 is a pure
   * number (Onsager [2]).  Sole input to boltz_update().        */

  long long sweeps;
  /* Frame counter (++ per grid_step call).  Pure diagnostic for
   * the HUD — "how long since this pattern started relaxing".
   * long long because a long session at 30 fps can exceed 2³¹
   * frames over ~2 years of continuous uptime.                  */

  float boltz[3];
  /* Precomputed Boltzmann factors exp(−ΔE/kT).
   *
   * On a 2D square lattice with s ∈ {±1} and Σ_nbr ∈ {±4,±2,0},
   * the flip energy ΔE = 2·s·Σ_nbr can ONLY take five values:
   *   ΔE = −8, −4, 0  → always accept (P = 1, no exp needed)
   *   ΔE = +4         → accept with P = boltz[0] = exp(−4/T)
   *   ΔE = +8         → accept with P = boltz[1] = exp(−8/T)
   * Cache only the two POSITIVE values.  Recomputing once per
   * temperature change turns the inner loop from one expf() per
   * attempt (~50 ns) into one array read (~1 ns) — single
   * biggest speed win in this file.  Newman & Barkema [4]
   * §3.1.4 describes the trick verbatim.  Slot [2] is unused
   * (legacy of an earlier dE/4 indexing scheme), kept for
   * alignment.                                                  */

  int pattern;
  /* Index into PATTERNS[].  Sim-state, NOT render-state:
   * changing it RESETS the lattice via apply_current_pattern().
   * n/p re-initialises the lattice; t/T (which only changes
   * .theme below) does not.                                     */

  /* ── RENDERING HALF ──────────────────────────────────────────────── *
   * Everything scene_draw() needs to paint a frame.  Theme cycling
   * (t/T), pause (space), and resize (SIGWINCH) all touch this half;
   * none perturb lattice evolution.                                  */

  int rows, cols;
  /* Current TERMINAL dims (recomputed on SIGWINCH).  Distinct
   * from gh/gw above: those are how big the LATTICE is, these
   * are how big the SCREEN is.  Row 0 and row rows-1 are HUD;
   * the lattice fills rows [1, rows-2].  cols is unsplit —
   * lattice and HUD share the width.                            */

  bool paused;
  /* UI latch.  When true, scene_draw() keeps painting (so the
   * user sees a stable picture) but the main loop skips
   * grid_step — lets the user freeze a snapshot to inspect
   * domain structure without the texture shifting under their
   * eyes.  Sparkline still gets fresh samples (all the same
   * frozen |M|), producing a visibly flat tail.                 */

  float mhist[SPARK_LEN];
  int mhead;
  /* Magnetisation history — circular buffer of the last
   * SPARK_LEN |M| samples, one pushed per render.  Rendered as
   * a 5-level ASCII sparkline (Tufte [7]) in the HUD.  Lives in
   * the render half because the simulation never reads it —
   * pure presentation.  mhead is the write index: the next
   * sample lands at mhist[mhead], then mhead advances mod
   * SPARK_LEN.  Watching this trace climb floor → ceiling as
   * you cool past Tc literally IS the phase transition made
   * visible — |M| is the order parameter, and the spontaneous
   * breakthrough below Tc is the Onsager [2] singularity
   * rendered in ASCII.                                          */

  int theme;
  /* Index into k_themes[].  Cycled by t/T.  Affects only the
   * colour pairs and glyphs in §6; simulation is untouched.    */
} Scene;

/* Single instance — the entire world the demo simulates and paints.
 *
 * INITIALISATION: only .temp is explicit (T_INIT = 3.0 > Tc, so the
 * demo starts in the disordered phase and you can cool down with ↓ to
 * watch domain nucleation happen).  Everything else relies on the C
 * default-zero rule for designated initialisers:
 *
 *   .pattern = 0  → PATTERNS[0] = Random         (filled in by
 *                                                 apply_current_pattern()
 *                                                 right after init)
 *   .theme   = 0  → k_themes[0] = Matrix         (phosphor green CRT)
 *   .paused  = 0  → running
 *   .mhist[] = {0}, .mhead = 0  → sparkline starts flat at floor
 *   .boltz[] = {0}              → overwritten by boltz_update() pre-loop
 *   .spin[][] = {0}             → overwritten by apply_current_pattern()
 *   .gh/.gw  = 0, .rows/.cols = 0  → set from getmaxyx() pre-loop
 *   .sweeps  = 0                → starts counting from zero          */
static Scene g_scene = {
    .temp = T_INIT,
};

/* ===================================================================== */
/* §4  color / theme                                                      */
/* ===================================================================== */

/*
 * Theme — two-tone visual identity bundle for the lattice.
 *
 * WHY a struct (and not loose globals): a theme couples up-spin AND
 *   down-spin appearance together — both colours AND both glyphs — so
 *   the lattice always reads as a coherent palette PAIR rather than an
 *   arbitrary mix.  One t/T keystroke swaps the entire visual language
 *   atomically: no half-themed transient states between presses.
 *
 * WHY two foreground sets per row (256-colour + 8-colour fallback):
 *   theme_apply() inspects ncurses' COLORS at runtime — modern xterms
 *   get the 256-cube pair, legacy / monochrome terminals get the
 *   COLOR_* fallback.  Both paths live on the same row so the table
 *   stays a one-line-per-theme grid you can scan vertically.
 *
 * WHY chtype glyphs live here (not in a separate char table): a "look"
 *   is colour + character together.  '#' on '.' reads as solid/grain;
 *   '*' on ' ' reads as starfield; '~' on '-' as ripple.  Splitting
 *   them would let you accidentally pair a starfield colour with a
 *   ripple glyph — bundling prevents that class of mistake.
 *
 * INVARIANT: themes NEVER influence the simulation — strictly a §4
 * (colour) concern.  Pair choices follow Ware [8] §4 (perceptual
 * contrast for categorical two-tone encoding) and the CLAUDE.md
 * "Theme Palette Brightness" rule (every entry sits in the bright
 * half of the cube so A_DIM never makes domain-wall cells invisible).
 */
typedef struct {
  const char *name;     /* HUD label: "Matrix", "Ocean", …            */
  short up_fg, dn_fg;   /* 256-colour fg (0..255) per spin state.
                         * Hand-tuned per row so up vs down are
                         * mutually distinguishable on black bg.      */
  short up_fg8, dn_fg8; /* 8-colour fallback (COLOR_*).  Picked
                         * when ncurses reports COLORS < 256.         */
  chtype up_ch, dn_ch;  /* ASCII glyph per spin state.  Paired for
                         * filled-vs-sparse TEXTURE contrast so the
                         * lattice still reads in pure monochrome.    */
} Theme;

/*
 * 10 themes — color pairs and characters give each lattice its identity.
 *
 *  Matrix  — phosphor green on black, classic CRT look
 *  Nova    — white stars scattered across a deep-purple void
 *  Mono    — pure greyscale, minimal
 *  Fire    — orange flame flickering over dark ember
 *  Ocean   — cyan ripple on deep navy
 *  Void    — hot-magenta pinpoints in absolute black
 *  Amber   — warm yellow on dark brown
 *  Neon    — hot pink on dark purple, cyberpunk
 *  Ice     — pale blue on cold navy
 *  Plasma  — red-hot against electric blue
 */
static const Theme k_themes[N_THEMES] = {
    /*  name      up256  dn256  up8            dn8           up_ch  dn_ch */
    {"Matrix", 46, 28, COLOR_GREEN, COLOR_GREEN, '#', '.'},
    {"Nova", 231, 57, COLOR_WHITE, COLOR_BLUE, '*', ' '},
    {"Mono", 231, 244, COLOR_WHITE, COLOR_BLACK, '#', '.'},
    {"Fire", 214, 88, COLOR_YELLOW, COLOR_RED, '^', '.'},
    {"Ocean", 51, 25, COLOR_CYAN, COLOR_BLUE, '~', '-'},
    {"Void", 201, 24, COLOR_MAGENTA, COLOR_BLACK, '@', ' '},
    {"Amber", 226, 94, COLOR_YELLOW, COLOR_RED, '#', '.'},
    {"Neon", 199, 54, COLOR_MAGENTA, COLOR_BLUE, '+', '-'},
    {"Ice", 159, 25, COLOR_CYAN, COLOR_BLUE, '*', '.'},
    {"Plasma", 196, 33, COLOR_RED, COLOR_BLUE, '#', '.'},
};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  if (COLORS >= 256) {
    init_pair(CP_UP, t->up_fg, -1);
    init_pair(CP_DN, t->dn_fg, -1);
  } else {
    init_pair(CP_UP, t->up_fg8, -1);
    init_pair(CP_DN, t->dn_fg8, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* canonical bright yellow */
    init_pair(CP_HINT, 51, -1); /* canonical bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
  theme_apply(g_scene.theme);
}

/* ===================================================================== */
/* §5  grid — mutates the simulation half of g_scene                      */
/* ===================================================================== */

/* All scene state lives in the Scene struct in §3 above (g_scene).  §5
 * holds the operations that ADVANCE the simulation: the per-T Boltzmann
 * recompute (boltz_update), the ten initial-condition patterns (pat_*
 * + apply_current_pattern), and the Metropolis sampler decomposed into
 * five named primitives + grid_step.  §5 functions only write the
 * SIMULATION HALF of g_scene; they never touch the rendering half. */

static void boltz_update(void) {
  g_scene.boltz[0] = expf(-4.f / g_scene.temp);
  g_scene.boltz[1] = expf(-8.f / g_scene.temp);
}

/* ── Initial-condition patterns ─────────────────────────────────────── *
 *
 * Each pat_* function fills the lattice with a specific starting
 * configuration; the Metropolis sampler then evolves it under the
 * current temperature.  Different patterns relax with VERY different
 * dynamics — that's the pedagogical point:
 *
 *   Random       — pure thermal sampling baseline (no structure)
 *   All Up/Down  — equilibrium ferromagnetic state; melts above Tc
 *   Checker      — antiferromagnetic / max-frustration; collapses fast
 *   Stripes      — domain walls anneal; narrow stripes merge into wider
 *   Halves       — single flat interface; roughens with T (Kardar-
 *                  Parisi-Zhang dynamics in the wall)
 *   Bubble       — surface tension shrinks a droplet (lifetime ∝ r²)
 *   Rings        — multi-scale curvature, ring count drops one-by-one
 *   Cluster      — many bubbles of random size; multi-time-scale
 *                  cascade (small bubbles vanish first)
 *
 * Ordering in PATTERNS[] is simple → complex by visual structure.
 */

static void pat_random(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = (rand() & 1) ? 1 : -1;
}

static void pat_all_up(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = 1;
}

static void pat_all_down(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = -1;
}

static void pat_checker(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((r + c) & 1) ? 1 : -1;
}

static void pat_stripes_h(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((r / STRIPE_W) & 1) ? 1 : -1;
}

static void pat_stripes_v(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((c / STRIPE_W) & 1) ? 1 : -1;
}

static void pat_halves(void) {
  int mid = g_scene.gw / 2;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = (c < mid) ? 1 : -1;
}

static void pat_bubble(void) {
  /* Single circular droplet of +1 in a sea of −1.  Surface tension
   * shrinks it at a rate ∝ 1/r (curvature-driven motion). */
  int cr = g_scene.gh / 2, cc = g_scene.gw / 2;
  int rad = (g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw) / 3;
  int rad2 = rad * rad;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++) {
      int dr = r - cr, dc = c - cc;
      g_scene.spin[r][c] = (dr * dr + dc * dc < rad2) ? 1 : -1;
    }
}

static void pat_rings(void) {
  /* Concentric annular rings of alternating spin.  Inner rings have
   * sharper curvature → shrink first; you can watch the count drop
   * ring-by-ring as the lattice anneals. */
  int cr = g_scene.gh / 2, cc = g_scene.gw / 2;
  int small_dim = g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw;
  int ring_w = small_dim / 10;
  if (ring_w < 2)
    ring_w = 2;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++) {
      int dr = r - cr, dc = c - cc;
      int dist = (int)sqrtf((float)(dr * dr + dc * dc));
      g_scene.spin[r][c] = ((dist / ring_w) & 1) ? 1 : -1;
    }
}

static void pat_cluster(void) {
  /* Background of −1 with ~12 +1 bubbles of random size and
   * position.  Cascade: small bubbles disappear in seconds, large
   * ones persist for many sweeps — the most multi-scale start. */
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = -1;

  int n_bubbles = 12;
  int max_rad = (g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw) / 5;
  if (max_rad < 3)
    max_rad = 3;
  for (int b = 0; b < n_bubbles; b++) {
    int cr = rand() % g_scene.gh;
    int cc = rand() % g_scene.gw;
    int rad = 2 + rand() % max_rad;
    int rad2 = rad * rad;
    for (int r = 0; r < g_scene.gh; r++)
      for (int c = 0; c < g_scene.gw; c++) {
        int dr = r - cr, dc = c - cc;
        if (dr * dr + dc * dc < rad2)
          g_scene.spin[r][c] = 1;
      }
  }
}

/*
 * Pattern — one entry in the n/p initial-condition cycle.
 *
 * WHY a struct of {name, init-fn}: the n/p key needs to walk a list of
 *   arbitrary lattice-fill routines AND show a human label for the
 *   current one in the HUD.  Function-pointer + name is the minimum
 *   machinery; PATTERNS[] becomes a single table that you extend by
 *   appending one row to add a new initial condition — no switch
 *   statements, no enum-to-string maps, no key-handler branches.
 *
 * WHY initial conditions matter (the pedagogical point):
 *   the Metropolis sampler [3] is the same regardless of starting
 *   state — but DIFFERENT INITIAL CONDITIONS RELAX VIA DIFFERENT
 *   DYNAMICS.  A circular Bubble shrinks via curvature-driven motion
 *   (Bray [6], lifetime ∝ r²); a flat Halves interface roughens via
 *   KPZ-like wall fluctuations; Stripes anneal by wall annihilation;
 *   Cluster cascades small bubbles first and large last.  Cycling
 *   through PATTERNS[] is a guided tour of phase-ordering kinetics,
 *   not just a fresh thermal equilibrium each time.
 *
 * WHY init() takes no args: each function reads the current g_scene.gh /
 *   g_scene.gw at call-time, so a SIGWINCH-driven resize naturally
 *   re-applies the pattern at the new dimensions with no plumbing.
 *   Also lets apply_current_pattern() be a single indirect call.
 */
typedef struct {
  const char *name;   /* HUD label: "Random", "Bubble", …          */
  void (*init)(void); /* fills g_scene.spin[][] from scratch with this
                       * pattern's starting configuration.  Reads
                       * g_scene.gh, g_scene.gw; writes nothing else. */
} Pattern;

static const Pattern PATTERNS[N_PATTERNS] = {
    /* simple → complex (visual structure & relaxation dynamics) */
    {"Random", pat_random},       /* 0 — thermal baseline           */
    {"All Up", pat_all_up},       /* 1 — trivial ordered            */
    {"All Down", pat_all_down},   /* 2 — mirror of All Up           */
    {"Checker", pat_checker},     /* 3 — antiferromagnetic frust.   */
    {"Stripes-H", pat_stripes_h}, /* 4 — horizontal anneal          */
    {"Stripes-V", pat_stripes_v}, /* 5 — vertical anneal            */
    {"Halves", pat_halves},       /* 6 — single flat interface      */
    {"Bubble", pat_bubble},       /* 7 — droplet surface tension    */
    {"Rings", pat_rings},         /* 8 — multi-scale curvature      */
    {"Cluster", pat_cluster},     /* 9 — multi-bubble cascade       */
};

/* Re-apply the active pattern (and reset the sweep counter).  Called
 * on startup, on r-reset, on n / p pattern switch, and on resize
 * (the grid dimensions just changed). */
static void apply_current_pattern(void) {
  PATTERNS[g_scene.pattern].init();
  g_scene.sweeps = 0;
}

/* ── Metropolis Monte Carlo primitives ──────────────────────────────── *
 *
 * Single-flip Metropolis [3] is the same six-step recipe every attempt:
 *
 *   1. PICK a random lattice site                  (uniform sampling)
 *   2. READ its current spin s                     (±1)
 *   3. SUM its 4 nearest neighbours Σ_nbr          (toroidal BC)
 *   4. COMPUTE flip energy ΔE = 2·s·Σ_nbr          (Ising Hamiltonian)
 *   5. ACCEPT or REJECT via the Boltzmann coin     (heart of MCMC)
 *   6. APPLY the flip if accepted                  (s → −s)
 *
 * Each step gets its own named helper below so grid_step reads as the
 * textbook recipe rather than a tangle of array indexing + arithmetic.
 *                                                                       *
 * ─────────────────────────────────────────────────────────────────────── */

/* (1) Uniform random site on the lattice.  The "single-flip" variant
 * of Metropolis (vs the "sweep" variant which visits every cell in
 * order); uniform sampling preserves detailed balance trivially. */
static void lattice_pick_random_site(int *r, int *c) {
  *r = rand() % g_scene.gh;
  *c = rand() % g_scene.gw;
}

/* (3) Σ_nbr s_j — sum of the 4 nearest-neighbour spins on a TOROIDAL
 * lattice (wraps north↔south and west↔east at the edges).  Returns a
 * value in {−4, −2, 0, +2, +4}.  Periodic BC removes edge artefacts so
 * a finite-size domain relaxes like an infinite-system patch. */
static int lattice_sum_neighbors_toroidal(int r, int c) {
  return g_scene.spin[r > 0 ? r - 1 : g_scene.gh - 1][c] +
         g_scene.spin[r < g_scene.gh - 1 ? r + 1 : 0][c] +
         g_scene.spin[r][c > 0 ? c - 1 : g_scene.gw - 1] +
         g_scene.spin[r][c < g_scene.gw - 1 ? c + 1 : 0];
}

/* (4) ΔE = 2·s·Σ_nbr — the energy CHANGE caused by flipping spin s in
 * a neighbourhood with sum Σ_nbr.  Derivation: the Hamiltonian
 * H = −J·Σ s_i·s_j is a sum of pairwise products; flipping s_i negates
 * four such products, changing H by 2·J·s_i·Σ_nbr.  With J = 1 the
 * value lives in {−8, −4, 0, +4, +8}. */
static int metropolis_delta_energy(int s, int sum_nbr) {
  return 2 * s * sum_nbr;
}

/* (5) Metropolis acceptance rule: P(flip) = min(1, exp(−ΔE/kT)).
 *
 * ΔE ≤ 0 → flip ALWAYS accepted (lowers or preserves energy).
 * ΔE > 0 → accept with probability exp(−ΔE/kT), drawn from the
 *          pre-cached Boltzmann table (one array read, no expf()).
 *
 * This asymmetry — easy downhill, conditional uphill — is what drives
 * the chain toward the Boltzmann distribution at temperature T. */
static bool metropolis_accept_flip(int dE) {
  if (dE <= 0)
    return true;
  float prob = (dE == 4) ? g_scene.boltz[0] : g_scene.boltz[1];
  return ((float)rand() / (float)RAND_MAX) < prob;
}

/* (6) s → −s.  Signed-char cast pacifies the type-narrowing warning. */
static void lattice_flip_spin(int r, int c) {
  g_scene.spin[r][c] = (signed char)(-g_scene.spin[r][c]);
}

/*
 * grid_step — perform n_attempts single-spin Metropolis attempts, then
 * increment the diagnostic sweep counter.  The body reads as the six-
 * step recipe documented above.
 */
static void grid_step(int n_attempts) {
  for (int f = 0; f < n_attempts; f++) {
    int r, c;
    lattice_pick_random_site(&r, &c);                   /* (1) */
    int s = g_scene.spin[r][c];                         /* (2) */
    int sum_nbr = lattice_sum_neighbors_toroidal(r, c); /* (3) */
    int dE = metropolis_delta_energy(s, sum_nbr);       /* (4) */
    if (metropolis_accept_flip(dE))                     /* (5) */
      lattice_flip_spin(r, c);                          /* (6) */
  }
  g_scene.sweeps++;
}

/* ===================================================================== */
/* §6  draw — reads g_scene, writes to ncurses                            */
/* ===================================================================== */

/* All rendering state lives in the unified Scene struct in §3 above —
 * the "Scene: unified scene-state container" doc block, specifically
 * its RENDERING HALF (rows, cols, paused, mhist, mhead, theme).
 * §6 here holds only the rendering FUNCTIONS; the data they read sits
 * on g_scene alongside the simulation half. */

static float magnetisation(void) {
  long long sum = 0;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      sum += g_scene.spin[r][c];
  return fabsf((float)sum / (float)(g_scene.gh * g_scene.gw));
}

static void mhist_push(float m) {
  g_scene.mhist[g_scene.mhead] = m;
  g_scene.mhead = (g_scene.mhead + 1) % SPARK_LEN;
}

/* Five-level ASCII bar set for |M| ∈ [0, 1].  Ordered low → high so
 * mapping is just (int)(m·4 + 0.5). */
static const char k_spark_glyph[5] = {'_', '.', '-', '*', '#'};

/*
 * hud_sparkline — render SPARK_LEN history samples as ASCII bars at
 * (row, *col), advancing the column cursor.  Coloured CP_UP because
 * the magnitude tracks alignment with the up-spin tier.  Clips at
 * max_col so the HUD never wraps on narrow terminals.
 */
static void hud_sparkline(int row, int *col, int max_col) {
  int avail = max_col - *col;
  if (avail <= 0)
    return;
  int n = SPARK_LEN < avail ? SPARK_LEN : avail;
  attron(COLOR_PAIR(CP_UP) | A_BOLD);
  for (int i = 0; i < n; i++) {
    int idx = (g_scene.mhead + i) % SPARK_LEN;
    int lvl = (int)(g_scene.mhist[idx] * 4.0f + 0.5f);
    if (lvl < 0)
      lvl = 0;
    if (lvl > 4)
      lvl = 4;
    mvaddch(row, *col + i, (chtype)(unsigned char)k_spark_glyph[lvl]);
  }
  attroff(COLOR_PAIR(CP_UP) | A_BOLD);
  *col += n;
}

/*
 * neighbor_agreement — how many of the 4 nearest neighbours agree
 * with cell (r, c).  Periodic boundary (toroidal lattice).
 *
 *   4 → deep interior of an aligned domain (no local frustration)
 *   3 → soft edge — one dissenter neighbour
 *   2 → domain wall — half agree, half don't (50/50 boundary)
 *   1 → frustrated cell (rare, mostly at high T)
 *   0 → fully surrounded by opposites (very rare, ΔE = −8 if flipped)
 *
 * Drives the per-cell rendering attribute in cell_attr() — interior
 * cells go A_DIM (recede), boundaries go A_BOLD (POP).
 */
static int neighbor_agreement(int r, int c, int s) {
  int u = g_scene.spin[r > 0 ? r - 1 : g_scene.gh - 1][c];
  int d = g_scene.spin[r < g_scene.gh - 1 ? r + 1 : 0][c];
  int l = g_scene.spin[r][c > 0 ? c - 1 : g_scene.gw - 1];
  int rt = g_scene.spin[r][c < g_scene.gw - 1 ? c + 1 : 0];
  return (u == s) + (d == s) + (l == s) + (rt == s);
}

/*
 * cell_attr — pick the ncurses attribute for one lattice cell so that
 * DOMAIN STRUCTURE reads at a glance:
 *
 *   interior   → A_DIM    (recedes into background — boring, stable)
 *   soft edge  → A_NORMAL (mild transition)
 *   wall / frust → A_BOLD (bright outline — where the physics is)
 *
 * Below Tc you see big dim domains with bright outlines around
 * dissenter clusters.  At Tc you see scale-free fractal flicker.
 * Above Tc almost every cell is at a boundary → bright noise.
 */
static attr_t cell_attr(int r, int c, int s) {
  int agree = neighbor_agreement(r, c, s);
  if (agree >= 4)
    return A_DIM;
  if (agree == 3)
    return A_NORMAL;
  return A_BOLD;
}

/*
 * hud_seg — print one HUD segment with its own colour pair + attr,
 * advancing the caller's column cursor.  Used to compose multicolor
 * HUD rows where each segment's hue is tied to what it describes
 * (e.g. the "ordered" phase tag in the up-spin colour, |M| in the
 * down-spin colour).  Clips to the row width and bails early so
 * narrow terminals don't wrap onto the next line.
 */
static void hud_seg(int row, int *col, int max_col, int pair, attr_t attr,
                    const char *fmt, ...) {
  if (*col >= max_col)
    return;
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n < 0)
    n = 0;
  attron(COLOR_PAIR(pair) | attr);
  mvprintw(row, *col, "%.*s", max_col - *col, buf);
  attroff(COLOR_PAIR(pair) | attr);
  *col += n;
}

/* ── lattice_paint: draw every spin with domain-aware attributes ────── *
 *
 * For each cell (r, c) in the live grid:
 *   - colour pair + glyph come from the active theme (up vs down)
 *   - visual emphasis comes from cell_attr() — interior cells go A_DIM
 *     and recede, boundary cells go A_BOLD and POP.  The eye follows
 *     domain walls without conscious effort.
 *
 * Lattice fills screen rows [1, rows-2]; row 0 and row rows-1 are HUD. */
static void lattice_paint(void) {
  const Theme *th = &k_themes[g_scene.theme];
  for (int r = 0; r < g_scene.gh && r + 1 < g_scene.rows - 1; r++) {
    for (int c = 0; c < g_scene.gw && c < g_scene.cols; c++) {
      int s = g_scene.spin[r][c];
      int cp;
      chtype ch;
      if (s > 0) {
        cp = CP_UP;
        ch = th->up_ch;
      } else {
        cp = CP_DN;
        ch = th->dn_ch;
      }
      attr_t at = cell_attr(r, c, s);
      attron(COLOR_PAIR(cp) | at);
      mvaddch(r + 1, c, ch);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/* ── HUD top-row sub-renderers ───────────────────────────────────────── *
 *
 * Each helper paints one logical chunk of the status bar and advances
 * the shared column cursor *col.  Reading hud_render_status_bar()
 * top-to-bottom tells you what information appears in what order; the
 * bar is a list of named labels, not a wall of hud_seg() calls.
 *
 * Colour discipline (Ware [8]): the on-screen number/word shares its
 * colour pair with the lattice phenomenon it describes — |M| in
 * up-spin colour because it measures up-spin dominance, "ordered" in
 * up-spin colour for the same reason, "disordered" in down-spin colour
 * to read as the disordered mix.                                       */

static void hud_label_temperature_and_phase(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, " Ising ");
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, " T=");
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, "%.3f", g_scene.temp);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, " (Tc=%.3f", T_CRIT);

  /* Phase classification by distance to the Onsager critical point.
   * Tag colour matches the lattice colour you should be seeing:
   * up-spin hue for ordered, down-spin hue for disordered, neutral
   * bold for the fractal critical regime. */
  float tc_dist = fabsf(g_scene.temp - T_CRIT);
  if (tc_dist < 0.1f)
    hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, " ≈Tc");
  else if (g_scene.temp < T_CRIT)
    hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, " ordered");
  else
    hud_seg(0, col, g_scene.cols, CP_DN, A_BOLD, " disordered");

  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, ")");
}

/* |M| is the ORDER PARAMETER of the Ising model — its sudden growth
 * from 0 to 1 as T drops below Tc IS the phase transition rendered
 * legibly.  Spontaneous-magnetisation breakthrough = Tufte [7]
 * sparkline ramp from floor to ceiling. */
static void hud_label_order_parameter(int *col, float m) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  |M|=");
  hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, "%.4f ", m);
  hud_sparkline(0, col, g_scene.cols);
}

static void hud_label_run_metadata(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, "  sweeps:%lld", g_scene.sweeps);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  pat:[%d] ",
          g_scene.pattern);
  hud_seg(0, col, g_scene.cols, CP_DN, A_BOLD, "%s",
          PATTERNS[g_scene.pattern].name);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  [");
  hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, "%s",
          k_themes[g_scene.theme].name);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "] ");
}

static void hud_label_pause_state(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, "%s",
          g_scene.paused ? "PAUSED" : "running");
}

/* Top HUD row — the live diagnostic strip.  Composed left-to-right
 * from the four labels above. */
static void hud_render_status_bar(float m) {
  int col = 0;
  hud_label_temperature_and_phase(&col);
  hud_label_order_parameter(&col, m);
  hud_label_run_metadata(&col);
  hud_label_pause_state(&col);
}

/* Bottom HUD row — action-key legend grouped by purpose: lifecycle
 * (CP_HINT), reset / pattern (CP_DN), temperature (CP_HUD / CP_DN /
 * CP_UP — cold=order=up, hot=disorder=down), theme (CP_HUD). */
static void hud_render_key_legend(void) {
  int b = 0;
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " q:quit ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " spc:pause ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_DN, A_BOLD, " n/p:pattern ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " r:reset ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HUD, A_BOLD, " up/dn:temp ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_DN, A_BOLD, " h:hot ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_UP, A_BOLD, " c:cold ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HUD, A_BOLD, " t/T:theme ");
}

/*
 * scene_draw — one full frame: paint the lattice, sample the order
 * parameter, feed it to the sparkline history, then render the two HUD
 * rows.  Four named steps, pseudocode-clean.
 */
static void scene_draw(void) {
  lattice_paint();

  float m = magnetisation();
  mhist_push(m);

  hud_render_status_bar(m);
  hud_render_key_legend();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit = 1;
  if (s == SIGWINCH)
    g_resize = 1;
}

static void cleanup(void) { endwin(); }

/* ── App lifecycle helpers ──────────────────────────────────────────── *
 *
 * Splitting main() into named verbs lets the reader see the canonical
 * ncurses + signal + simulation startup at a glance, and the main loop
 * read as input → simulate → render → frame-cap.                       */

static void app_install_signal_handlers(void) {
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);
}

static void app_init_terminal(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE); /* getch is non-blocking — UI never stalls */
  curs_set(0);           /* hide hardware cursor                    */
  typeahead(-1);         /* ncurses won't peek stdin mid-diff-write */
}

/* Re-clamp the scene's lattice dims to fit the current terminal size.
 * Called once at startup and again on each SIGWINCH.  The lattice
 * array is sized for GRID_*_MAX so this never reallocates. */
static void scene_fit_to_terminal(void) {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  g_scene.rows = rows;
  g_scene.cols = cols;
  g_scene.gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
  g_scene.gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
}

/* SIGWINCH companion (called from the main loop, NOT from the signal
 * handler — handlers only flip flags).  Tears down ncurses, refits the
 * scene to the new terminal size, and re-applies the active pattern. */
static void handle_resize_event(void) {
  g_resize = 0;
  endwin();
  refresh();
  scene_fit_to_terminal();
  apply_current_pattern();
}

/* ── User-action helpers (called from handle_key_input) ─────────────── */

/* Advance the pattern index by dir (±1), wrapping at N_PATTERNS, then
 * re-apply.  N_PATTERNS added to keep the modulus non-negative for
 * dir = −1 (C's % is implementation-defined for negative operands). */
static void pattern_advance(int dir) {
  g_scene.pattern = (g_scene.pattern + N_PATTERNS + dir) % N_PATTERNS;
  apply_current_pattern();
}

/* Set temperature to a specific value, then refresh the Boltzmann
 * cache.  Used by the h/c hotkeys for instant hot/cold jumps. */
static void temperature_set(float t) {
  g_scene.temp = t;
  boltz_update();
}

/* Add delta to temperature (with clamping to [T_MIN, T_MAX]), then
 * refresh the Boltzmann cache.  Used by ↑/↓ for fine adjustment. */
static void temperature_nudge(float delta) {
  g_scene.temp += delta;
  if (g_scene.temp < T_MIN)
    g_scene.temp = T_MIN;
  if (g_scene.temp > T_MAX)
    g_scene.temp = T_MAX;
  boltz_update();
}

/* Advance the theme index by dir (±1), wrapping at N_THEMES, then
 * re-apply the colour pairs.  Same modulus-positivity trick as
 * pattern_advance. */
static void theme_advance(int dir) {
  g_scene.theme = (g_scene.theme + N_THEMES + dir) % N_THEMES;
  theme_apply(g_scene.theme);
}

/* Dispatch one keypress.  Cases grouped by purpose: lifecycle,
 * simulation reset, temperature control, visual theme. */
static void handle_key_input(int ch) {
  switch (ch) {
  /* lifecycle */
  case 'q':
  case 'Q':
  case 27:
    g_quit = 1;
    break;
  case ' ':
    g_scene.paused = !g_scene.paused;
    break;

  /* simulation reset / pattern */
  case 'r':
  case 'R':
    apply_current_pattern();
    break;
  case 'n':
    pattern_advance(+1);
    break;
  case 'p':
    pattern_advance(-1);
    break;

  /* temperature control */
  case 'h':
  case 'H':
    temperature_set(T_MAX);
    break;
  case 'c':
  case 'C':
    temperature_set(0.5f);
    break;
  case KEY_UP:
    temperature_nudge(+T_STEP);
    break;
  case KEY_DOWN:
    temperature_nudge(-T_STEP);
    break;

  /* visual theme */
  case 't':
    theme_advance(+1);
    break;
  case 'T':
    theme_advance(-1);
    break;

  default:
    break;
  }
}

/* ── Per-frame engine helpers ───────────────────────────────────────── */

/* Number of single-spin Metropolis attempts to perform per render
 * frame.  Scales with lattice area × throttle factor — chosen so
 * domain dynamics are visible at 30 fps without saturating CPU on
 * large terminals. */
static int metropolis_attempts_per_frame(void) {
  return g_scene.gh * g_scene.gw * FLIPS_PER_CELL / 1000;
}

/* One full ncurses present cycle: clear, paint, diff-flush. */
static void app_render_frame(void) {
  erase();
  scene_draw();
  wnoutrefresh(stdscr);
  doupdate();
}

/*
 * main — pseudocode of the entire program: init phase, then a loop
 * that does input → simulate → render → frame-cap forever (until q).
 */
int main(void) {
  /* ── Initialisation ──────────────────────────────────────────── */
  srand((unsigned)time(NULL));
  atexit(cleanup);
  app_install_signal_handlers();
  app_init_terminal();
  color_init();
  scene_fit_to_terminal();
  boltz_update();
  apply_current_pattern();

  /* ── Main loop ───────────────────────────────────────────────── */
  while (!g_quit) {
    if (g_resize)
      handle_resize_event();
    handle_key_input(getch());

    long long now = clock_ns();
    if (!g_scene.paused)
      grid_step(metropolis_attempts_per_frame());
    app_render_frame();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
