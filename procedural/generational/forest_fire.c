/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * forest_fire.c — Drossel–Schwabl forest-fire cellular automaton
 *
 * DEMO: A 3-state probabilistic CA on a rows × cols grid models fire
 *       ecology in real time.  Trees regrow on empty land at probability
 *       p, lightning ignites trees at probability f, fire spreads to
 *       any 4-neighbour (or optionally 8-neighbour) tree and burns out
 *       in one tick.  At the critical ratio p/f the fire-cluster-size
 *       distribution is a power law — the system self-organises onto
 *       the critical point with no parameter tuning.  Five colour
 *       themes, four behavioural presets, live keyboard adjustment of
 *       p and f.
 *
 * Study alongside: artistic/fire.c (rule-driven heat propagation, no CA grid)
 *                  artistic/forest_fire-style demos elsewhere in the repo
 *
 * Section map (cut by concern — see ARCHITECTURE block):
 *   §1 config      — sizes, p/f step+bounds, theme + preset tables, sim Hz
 *   §2 perf/delays — monotonic clock + sleep
 *   §3 state       — all mutable globals (grid, params, stats, flash, rng)
 *   §4 logic       — has_fire_neighbor (pure)
 *   §5 simulation  — RNG, seed/reset, forest_step (the tick)
 *   §6 effects     — ash overlay (painted in §5) + HUD action-flash
 *   §7 render      — themes, grid + HUD draw
 *   §8 platform    — ncurses, signals, resize, input, main loop
 *
 * Keys:
 *   q / ESC    quit                       p / space  pause / resume
 *   r          reset (reseed grid)        n / N      next / prev preset
 *   t / T      next / prev theme          g / G      grow prob p up / down
 *   l / L      lightning prob f up/down
 *   + / -      sim speed up / down
 *
 * Presets:
 *   0  Classic     — p=0.030 f=0.0002, balanced; moderate clusters
 *   1  Dense       — p=0.060 f=0.0001, fast growth; catastrophic burns
 *   2  Sparse      — p=0.010 f=0.0010, sparse; frequent small fires
 *   3  Smouldering — p=0.020 f=0.0003, 8-neighbour spread; creeping fires
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/forest_fire.c \
 *       -o forest_fire -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Drossel–Schwabl probabilistic cellular automaton (1992).
 *                  Three-state grid (EMPTY = 0, TREE = 1, FIRE = 2) with
 *                  synchronous update: every cell's next state is computed
 *                  from the CURRENT generation, then all next states are
 *                  committed at once (double-buffer pattern).  Without the
 *                  double buffer, fire "races" through trees in scan order
 *                  and the spread looks anisotropic.
 *
 *                  Update rules:
 *                    FIRE  → EMPTY    (fire burns out in one tick)
 *                    TREE  → FIRE     if any 4-/8-neighbour is FIRE
 *                    TREE  → FIRE     with probability f (lightning)
 *                    TREE  → TREE     otherwise
 *                    EMPTY → TREE     with probability p (regrowth)
 *                    EMPTY → EMPTY    otherwise
 *
 *                  ASH overlay: a separate `ash[r][c]` flag records cells
 *                  that JUST burned, rendered '.' for one tick.  Pure
 *                  cosmetic; doesn't affect the next CA step.
 *
 * Data-structure : Two `uint8_t grid[ROWS_MAX][COLS_MAX]` buffers (current
 *                  and next), plus a 1-tick `uint8_t ash[…][…]` flag grid.
 *                  All static BSS — 192 KB total at 128×512.  An xorshift32
 *                  inline RNG provides the per-cell uniform float; the
 *                  hot loop avoids `rand()` overhead and keeps the inner
 *                  per-cell test under ~10 ns.
 *
 * Physics        : Self-organised criticality (Bak, Tang & Wiesenfeld 1987).
 *                  At the critical p/f ratio, fire-cluster size has no
 *                  characteristic scale: P(s) ∝ s^(−τ), τ ≈ 1.19.  The
 *                  system tunes itself onto this critical point — large
 *                  p/f → dense clusters → catastrophic burns reset density
 *                  → small p/f temporarily → sparse → fewer fires → density
 *                  rebuilds.  No parameter sweep needed.
 *
 *                  The 4-neighbour (von Neumann) kernel produces axis-aligned
 *                  fire fronts; the 8-neighbour (Moore) kernel of the
 *                  Smouldering preset is more isotropic.
 *
 * Rendering      : Per-cell glyph + colour pair selected from the active
 *                  theme.  EMPTY → space (terminal bg shows through);
 *                  TREE → '^' tree-top; FIRE → alternating '*'/',' to
 *                  flicker; ASH → '.' for one tick.  All glyph stamps go
 *                  through a `mark_cell()` helper that performs the
 *                  (chtype)(unsigned char) cast and bounds-check.
 *
 *                  HUD: PAIR_HUD (bright yellow) on row 0 right shows
 *                  preset + theme + tree/fire % + p/f values + sim Hz +
 *                  paused state; PAIR_HINT (bright cyan) on row N-1
 *                  shows the key reference.  Both A_BOLD per spec.
 *
 * Performance    : O(rows × cols) per CA tick.  At 80 × 24 = 1920 cells,
 *                  20 sim Hz → 38 K cells/s.  Each cell does at most
 *                  4 (or 8) neighbour reads + 1 RNG draw — negligible.
 *
 * References     : Concepts —
 *                  [1] Drossel & Schwabl, "Self-organized critical forest-fire
 *                      model," Phys. Rev. Lett. 69, 1629 (1992) — THE canonical
 *                      paper; this file implements its exact rules.
 *                  [2] Bak, Tang & Wiesenfeld, "Self-organized criticality: An
 *                      explanation of 1/f noise," Phys. Rev. Lett. 59, 381
 *                      (1987) — the SOC framework this CA falls under.
 *                  [3] Bak, "How Nature Works: The Science of Self-Organized
 *                      Criticality" (Copernicus, 1996) — the definitive,
 *                      readable book on why systems tune themselves critical.
 *                  [4] Christensen & Moloney, "Complexity and Criticality"
 *                      (Imperial College Press, 2005) — textbook that derives
 *                      the forest-fire model's scaling exponents in full.
 *                  [5] Malamud, Morein & Turcotte, "Forest Fires: An Example of
 *                      Self-Organized Critical Behavior," Science 281, 1840
 *                      (1998) — real wildfire size data match the model.
 *                  [6] Stauffer & Aharony, "Introduction to Percolation Theory"
 *                      (Taylor & Francis, 1994) — the percolation transition
 *                      behind fire-cluster connectivity and the critical p/f.
 *                  [7] Gutenberg & Richter, "Frequency of earthquakes in
 *                      California," 1944 — the same power-law size law, in
 *                      seismology (the cross-domain SOC signature).
 *                  Rendering / implementation —
 *                  [8] Marsaglia, "Xorshift RNGs," J. Stat. Soft. 8(14) (2003)
 *                      — the per-cell xorshift32 the hot loop uses instead of
 *                      rand() (see §2 / Data-structure).
 *                  [9] Ben-Halim, Raymond et al., "Writing Programs with
 *                      NCURSES" (NCURSES HOWTO) — colour pairs, erase/refresh;
 *                      the §3 colour + §5 draw model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The whole simulation is FOUR rules applied to every cell, every tick,
 * SIMULTANEOUSLY.  Rule 1: a fire is gone next tick (it burns out).
 * Rule 2: a tree near a fire is on fire next tick.  Rule 3: a tree
 * with no fire neighbour MAY randomly catch fire from lightning.
 * Rule 4: an empty cell MAY randomly grow a tree.  That's it.  Two
 * dials (p = grow rate, f = lightning rate) and you get fire ecology
 * — and, for free, a system that sits on a critical point where fire
 * sizes follow a power law.  No physics, no global rule, no balancing
 * — just four local rules, applied everywhere at once.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a giant chessboard where each square is a cell of forest.
 * Every second, a celestial referee inspects every square at once and
 * writes the next state on a parallel chessboard.  Trees grow into
 * empty squares occasionally; lightning lights occasional trees;
 * fire spreads to neighbours; fire ages out.  The "next" board is
 * then the new "current" board.  Repeat forever.  Watch what emerges.
 *
 * The double-buffer is essential: if you rewrote the same chessboard
 * in scan order, fire near (0, 0) would already be empty by the time
 * you got to (0, 1), and the spread would race east-and-down rather
 * than spreading uniformly.  Two boards, write-then-flip, gives the
 * isotropic spread you actually want.
 *
 * The MAGIC of this CA is what happens with NO TUNING.  Crank p high,
 * trees grow fast, dense clumps form, lightning eventually hits, the
 * whole clump burns, density drops, you're back to sparse — and the
 * fire size that just happened is "as big as it had to be" given the
 * cluster.  Run for long enough and the fire size histogram is a
 * straight line on a log-log plot — a power law.  The system finds
 * the critical point on its own (Bak et al. SOC).
 *
 * ALGORITHM IN STEPS  (per simulation tick)
 * ─────────────────────────────────────────
 *   1. Zero the ash overlay: ash[r][c] = 0.
 *   2. For every cell (r, c):
 *      a. If grid[r][c] == FIRE:
 *           next[r][c] = EMPTY; ash[r][c] = 1.
 *      b. Else if grid[r][c] == TREE:
 *           if any 4-/8-neighbour is FIRE OR rng() < f:
 *             next[r][c] = FIRE.
 *           else:
 *             next[r][c] = TREE.
 *      c. Else (EMPTY):
 *           next[r][c] = (rng() < p) ? TREE : EMPTY.
 *   3. memcpy(grid <- next).
 *   4. Recompute n_tree / n_fire / n_empty for the HUD.
 *   5. Render: per cell, dispatch on grid[r][c] (+ ash overlay).
 *
 * KEY FORMULAS
 * ────────────
 *   Fire-cluster size distribution at critical p/f:
 *     P(s) ∝ s^(−τ)        τ ≈ 1.19
 *
 *   Critical ratio scaling (Drossel–Schwabl, 1992):
 *     p / f → ∞  but  p → 0  AND  p · L² → ∞
 *     ("slow grow, even slower lightning, large grid" limit)
 *
 *   p/f informally:
 *     average cluster size ∝ (p/f)^β    β ≈ 1
 *     (large p/f → big clusters → catastrophic fires)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Synchronous update is mandatory: a single in-place write would
 *     make fire race east/south through trees in one tick rather than
 *     spreading isotropically over many ticks.  The Forest's `grid` and
 *     `next` are the double-buffer; `memcpy(grid, next, …)` flips them.
 *   - Boundary: the grid is finite; cells at edges have fewer
 *     neighbours.  Bounds-checked in `forest_step` (no toroidal wrap).
 *     Real fires don't wrap around the world either.
 *   - 8-neighbour spread (Smouldering preset) is more isotropic but
 *     ALSO faster — the critical p/f shifts.  Preset 3 lowers p
 *     accordingly so total fire area per tick stays comparable.
 *   - Theme palette: every entry must be in the bright half of the
 *     256-cube (≥ 24) per CLAUDE.md.  EMPTY cells render as space
 *     with bg=-1 (terminal default) so the user's terminal background
 *     shows through; tinting via dark grayscale (234-240) is forbidden.
 *   - HUD bottom row spans the full width; if the grid is ≥ 80 cols the hint
 *     fits, otherwise it's truncated.  No terminal under 80 cols is
 *     supported well.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - At startup with default Classic preset, you should see green
 *     trees densifying to ~60 % coverage within a few seconds, then
 *     occasional `*`/`,` fire bursts spreading outward through tree
 *     clusters and leaving '.' ash trails.  HUD: "Trees:60% Fire:1%".
 *   - Press `g` repeatedly — tree % climbs toward 90 %; fires that
 *     start now spread further before hitting empty cells.  Press
 *     `G` to bring it back.
 *   - Cycle presets with `n` — Dense visibly increases tree cover,
 *     Sparse leaves big bare patches, Smouldering (8-nbr) makes
 *     fires take diagonal corners that 4-nbr can't.
 *   - Cycle themes with `t` — palette changes immediately on next
 *     render frame; HUD pairs stay yellow/cyan regardless.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layers) ───────────────────────────────────────────────
 *
 * Re-cut into labelled layers. Data is aggregated on one Scene (the Forest + its
 * knobs); the table is the contract for who writes what. Each tick, forest_step()
 * (§5) is the ONE call that advances simulation state. Workers take the narrowest
 * type (Forest* / const Forest* / const Scene*); only scene_init takes Scene*.
 *
 *   LAYER          §   MUTATES                              KEY FUNCTIONS
 *   ------------  --   ----------------------------------   ----------------------
 *   CONFIG         1   (constants + theme/preset tables)   —
 *   PERF/DELAYS    2   (local timing only)                 clock_ns, clock_sleep_ns
 *   STATE          3   defines Forest + Scene (g_scene)    —
 *   LOGIC          4   nothing (pure read)                 has_fire_neighbor
 *   SIMULATION     5   Forest (grid/next/census/tick) and  forest_step <-- the tick,
 *                      paints the ash EFFECT; g_rng         forest_seed, scene_init,
 *                                                          rng_next, rng_float
 *   EFFECTS        6   Forest.ash (in §5), g_flash          hud_flash
 *   RENDER         7   terminal + ncurses palette only      scene_draw, draw_forest,
 *                                                          draw_hud, mark_cell, theme_apply
 *   PLATFORM/APP   8   g_resize/g_quit, Forest dims, and    main, screen_init,
 *                      Scene knobs / theme via key &        screen_resize, signals
 *                      resize events
 *
 * Per-tick combine (main, fixed-timestep): when not paused and a tick is due,
 *   1. SIMULATION — forest_step(): one synchronous forest-fire CA update; writes
 *      forest.next, flips into forest.grid, paints the 1-tick ash overlay, tallies.
 *   RENDER (erase → draw_forest → draw_hud) and the PERFORMANCE shell (next_tick
 *   schedule + frame sleep) wrap it in main and advance no simulation state.
 *
 * Notes / contracts:
 *   (a) Signature convention IN FORCE: a pure read takes a const pointer
 *       (const Forest* / const Scene*); a mutator takes Forest* / Scene*. Only
 *       scene_init takes the whole Scene; everything else takes a sub-type.
 *   (b) LOGIC (§4) is one pure read — has_fire_neighbor — no mutation, no I/O;
 *       reordering or deleting render/effects cannot change its result.
 *   (c) forest_step() is the sole per-tick state advance. scene_init / forest_seed
 *       (§5), screen_resize (§8), the key handlers, and hud_flash (§6) also mutate
 *       state, but are INIT or USER EVENTS, not part of the tick.
 *   EFFECTS: two cosmetic-only buffers — Forest.ash (the 1-tick burnt-ground
 *       overlay, PAINTED inside forest_step §5, READ by draw_forest) and the HUD
 *       action-flash g_flash (SET by hud_flash on a key, READ by draw_hud).
 *       Neither feeds back into the CA.
 *   DELAYS: the paused gate (Scene.paused, checked in main before the tick) and the inter-
 *       frame sleep (clock_sleep_ns, §2) — interwoven, not a standalone layer.
 * ──────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX   128
#define COLS_MAX   512

/* Cell states */
#define EMPTY  0
#define TREE   1
#define FIRE   2

/* Live p/f adjustment: step per keypress, and clamp bounds. The starting
 * values are not here — each preset (k_presets) supplies its own p and f. */
#define P_GROW_STEP  0.005f
#define P_FIRE_STEP  0.0001f
#define P_GROW_MIN   0.001f
#define P_GROW_MAX   0.200f
#define P_FIRE_MIN   0.00001f
#define P_FIRE_MAX   0.010f

/* Simulation timing */
#define SIM_FPS_DEF   20
#define SIM_FPS_MIN    2
#define SIM_FPS_MAX   60
#define SIM_FPS_STEP   2

#define N_PRESETS  4
#define N_THEMES   5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define FLASH_NS    (NS_PER_SEC * 9 / 10)   /* HUD action-flash lifetime (~0.9 s)        */
#define SLEEP_MARGIN_NS  1000000LL           /* wake ~1 ms early so we never oversleep a tick */

/*
 * Colour pair IDs.
 *
 *   CP_TREE / CP_FIRE1 / CP_FIRE2 / CP_ASH — theme-tinted FOREGROUND
 *     colours; background = -1 (terminal default).
 *   PAIR_HUD / PAIR_HINT — theme-independent bright yellow / bright
 *     cyan, A_BOLD on draw, per the project HUD spec.
 *
 * EMPTY cells render as space — no colour pair, terminal background
 * shows through.  The previous tinted-empty design used grayscale
 * indices 234-240, which the CLAUDE.md brightness rule reserves as
 * "NEVER use".  Letting empty cells inherit the terminal background
 * is both spec-compliant and visually cleaner — the forest is the
 * trees, not the dirt between them.
 */
enum {
    CP_TREE   = 1,
    CP_FIRE1  = 2,   /* dim fire   */
    CP_FIRE2  = 3,   /* bright fire */
    CP_ASH    = 4,   /* one-tick ash overlay after fire burnout */
    PAIR_HUD  = 8,   /* bright yellow — top status            */
    PAIR_HINT = 9,   /* bright cyan   — bottom key hint       */
};

/*
 * Theme — one named palette for the scene glyphs (tree, two fire shades, ash).
 * Two fire colours exist so adjacent burning cells can alternate bright/dim and
 * a fire patch visibly crackles (see draw_forest); EMPTY needs no colour — it is
 * drawn as a space and the terminal background shows through.
 *
 * Every index sits in the BRIGHT half of the 256-colour cube (dimmest ≥ 28,
 * primaries above): grayscale 232-239 and cube indices below 24 are forbidden by
 * the project brightness rule (COLOR.md) because they render near-black under
 * A_BOLD/A_DIM. Each theme also carries an 8-colour fallback for terminals
 * without 256-colour support (chosen by g_has_256 in theme_apply).
 */
typedef struct {
    short tree;          /* live tree '^'                                      */
    short fire1, fire2;  /* dim ',' / bright '*' fire — alternated to flicker  */
    short ash;           /* one-tick burnt-ground '.'                          */
    short tree8, fire18, fire28, ash8;   /* 8-colour fallbacks for the above   */
    const char *name;    /* shown in the HUD                                   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0  Classic — green forest, orange-red fire */
    {  34, 202, 196, 244,
       COLOR_GREEN, COLOR_YELLOW, COLOR_RED, COLOR_WHITE,
       "Classic" },
    /* 1  Night   — emerald forest, yellow-white fire */
    {  35, 214, 226, 247,
       COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
       "Night" },
    /* 2  Autumn  — amber trees, deep red fire */
    { 130, 196, 160, 245,
       COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_WHITE,
       "Autumn" },
    /* 3  Boreal  — teal trees, yellow-white fire */
    {  37, 220, 231, 250,
       COLOR_CYAN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
       "Boreal" },
    /* 4  Lava    — green trees, magenta-white fire */
    {  29, 201, 207, 248,
       COLOR_GREEN, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE,
       "Lava" },
};

/*
 * Preset — a named operating point in the forest-fire model's parameter space.
 * The Drossel-Schwabl dynamics turn on two rates: p (a resting cell grows a tree)
 * and f (a tree spontaneously ignites — "lightning"). Their RATIO f/p is what
 * matters: as f/p → 0 the model SELF-ORGANISES to a critical state where fire-
 * cluster sizes follow a power law P(s) ∝ s^-τ (Drossel & Schwabl, PRL 69, 1629,
 * 1992; the SOC framework, Bak-Tang-Wiesenfeld, PRL 59, 381, 1987). Each preset
 * is a different (p, f) so one keypress walks the phase diagram.
 */
typedef struct {
    float p_grow;          /* p: EMPTY → TREE per tick (regrowth rate)          */
    float p_fire;          /* f: TREE  → FIRE per tick (lightning rate)         */
    bool  eight_neighbor;  /* spread kernel: 8-way Moore vs 4-way von Neumann   */
    float density;         /* tree fraction the grid is seeded with (0..1)      */
    const char *name;      /* shown in the HUD                                  */
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { 0.030f, 0.0002f, false, 0.60f, "Classic" },
    { 0.060f, 0.0001f, false, 0.70f, "Dense"   },
    { 0.010f, 0.0010f, false, 0.40f, "Sparse"  },
    { 0.020f, 0.0003f, true,  0.55f, "Smoulder"},
};

/* ===================================================================== */
/* §2  perf / delays  —  monotonic clock + sleep primitives               */
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
/* §3  state  —  all mutable globals (written by SIMULATION §5 / APP §8)   */
/* ===================================================================== */

/*
 * Forest — the lattice the Drossel-Schwabl forest-fire model runs on (PRL 69,
 * 1992), the canonical demonstration of self-organised criticality. Each cell is
 * EMPTY / TREE / FIRE and updates by four local rules per tick (see forest_step).
 *
 * WHY double-buffered: the update is SYNCHRONOUS — the whole next generation must
 * be computed from the current one. A single in-place grid would let fire "race"
 * east/south through trees within one tick (scan-order artefact), making spread
 * look anisotropic; so grid/next are two full generations and forest_step writes
 * `next`, then flips it into `grid`.
 */
typedef struct {
    /* the lattice — SIMULATION writes, LOGIC + RENDER read */
    uint8_t grid[ROWS_MAX][COLS_MAX];   /* current generation (EMPTY/TREE/FIRE) */
    uint8_t next[ROWS_MAX][COLS_MAX];   /* next generation, then swapped into grid */
    int     rows, cols;                 /* active extent this run (≤ ROWS/COLS_MAX);
                                           sized from the terminal, below the HUD   */

    /* EFFECT overlay (§6) — cells that burned THIS tick, rendered '.'. Set by
     * forest_step on each FIRE→EMPTY, cleared at the next tick's start. Cosmetic:
     * never read back by the CA. */
    uint8_t ash[ROWS_MAX][COLS_MAX];

    /* census + clock, recomputed by forest_step each tick (read by the HUD) */
    int     n_tree, n_fire, n_empty;    /* cell counts; the tree fraction is the
                                           order parameter that self-tunes critical */
    long    tick;                       /* steps elapsed; also drives the fire
                                           bright/dim flicker parity in draw_forest  */
} Forest;

/*
 * Scene — the whole program in one place, read like a table of contents: the
 * forest (WHAT is simulated), the model knobs the user drives it with (HOW), the
 * run-state, and the palette. p_grow/p_fire/eight_neighbor are the LIVE copy of
 * the active Preset's parameters that the user then tweaks with keys — so each
 * tick reads these, not k_presets.
 */
typedef struct {
    Forest forest;          /* WHAT is simulated                              */

    /* HOW the user drives the model — the live Drossel-Schwabl parameters.
     * The ratio f/p (p_fire / p_grow) sets where on the SOC phase diagram we sit. */
    float  p_grow;          /* p: empty → tree per tick   (g / G keys)        */
    float  p_fire;          /* f: tree  → fire, lightning (l / L keys)        */
    bool   eight_neighbor;  /* 8-way (Moore) vs 4-way (von Neumann) spread    */
    int    preset;          /* which Preset is loaded     (n / N keys)        */
    int    sim_fps;         /* CA steps per second        (+ / - keys)        */

    bool   paused;          /* run-state: when set, forest_step is skipped    */
    int    theme;           /* RENDER: palette index      (t / T keys)        */
} Scene;

static Scene g_scene = {
    .preset  = 0,
    .sim_fps = SIM_FPS_DEF,
    .theme   = 0,
};

/* xorshift32 RNG state (SIMULATION §5 owns it) */
static uint32_t g_rng = 12345;

/* terminal capability + transient HUD action-flash (cosmetic EFFECT §6) */
static bool    g_has_256;
static char    g_flash[48];
static int64_t g_flash_until = 0;

/* platform signal flags */
static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

/* ===================================================================== */
/* §4  logic  —  pure decisions: no mutation, no I/O                      */
/* ===================================================================== */

/* True if any of the four (dr,dc)-offset neighbours of (r,c) is FIRE (in-bounds). */
static bool any_neighbour_on_fire(const Forest *f, int r, int c,
                                  const int *dr, const int *dc)
{
    for (int d = 0; d < 4; d++) {
        int nr = r + dr[d], nc = c + dc[d];
        if (nr >= 0 && nr < f->rows && nc >= 0 && nc < f->cols
            && f->grid[nr][nc] == FIRE) return true;
    }
    return false;
}

/*
 * has_fire_neighbor — is (r,c) next to a burning cell this generation? The
 * 4 orthogonal (von Neumann) neighbours always count; the 4 diagonal (Moore)
 * neighbours count only in the 8-way presets. No toroidal wrap — edge cells
 * simply have fewer neighbours.
 */
static bool has_fire_neighbor(const Forest *f, int r, int c, bool eight)
{
    static const int orth_dr[4] = {-1, 1, 0, 0}, orth_dc[4] = { 0, 0,-1, 1};
    static const int diag_dr[4] = {-1,-1, 1, 1}, diag_dc[4] = {-1, 1,-1, 1};

    if (any_neighbour_on_fire(f, r, c, orth_dr, orth_dc)) return true;
    if (!eight) return false;
    return any_neighbour_on_fire(f, r, c, diag_dr, diag_dc);
}

/* ===================================================================== */
/* §5  simulation  —  the only layer that advances state                  */
/* ===================================================================== */

/* xorshift32: fast per-cell RNG for the hot loop (avoids rand() overhead) */
static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
/* Returns uniform float in [0,1) */
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1 << 24);
}

static void forest_seed(Forest *f, float density)
{
    for (int r = 0; r < f->rows; r++)
        for (int c = 0; c < f->cols; c++) {
            f->grid[r][c] = (rng_float() < density) ? TREE : EMPTY;
            f->ash[r][c]  = 0;
        }
}

/* Orchestrator: load the chosen preset's parameters and re-seed the forest. */
static void scene_init(Scene *s, int preset)
{
    s->preset         = preset;
    s->p_grow         = k_presets[preset].p_grow;
    s->p_fire         = k_presets[preset].p_fire;
    s->eight_neighbor = k_presets[preset].eight_neighbor;
    s->forest.tick    = 0;
    forest_seed(&s->forest, k_presets[preset].density);
}

/*
 * forest_step — one synchronous CA tick.
 *
 *   1. Zero ash overlay (one-tick after-burn marker).
 *   2. For every cell, dispatch on state and write next[r][c]:
 *        FIRE  → EMPTY (mark ash)
 *        TREE  → FIRE if neighbour fire OR rng < p_fire; else TREE
 *        EMPTY → TREE with probability p_grow; else EMPTY
 *      Tally the tree / fire / empty counts for the HUD as we go.
 *   3. memcpy(grid ← next) — flip the double-buffer.
 *   4. Advance tick counter.
 */
static void forest_step(Forest *f, float p_grow, float p_fire, bool eight)
{
    int nt = 0, nf = 0, ne = 0;
    memset(f->ash, 0, sizeof f->ash);   /* clear last tick's burn overlay */

    /* the four Drossel-Schwabl rules, applied synchronously to every cell */
    for (int r = 0; r < f->rows; r++) {
        for (int c = 0; c < f->cols; c++) {
            uint8_t state = f->grid[r][c];
            uint8_t next  = state;

            if (state == FIRE) {                 /* fire burns out in one tick */
                next = EMPTY; f->ash[r][c] = 1; ne++;
            } else if (state == TREE) {          /* catch from a neighbour, or by lightning */
                bool ignite = has_fire_neighbor(f, r, c, eight)
                           || rng_float() < p_fire;
                next = ignite ? FIRE : TREE;
                if (ignite) nf++; else nt++;
            } else {                             /* EMPTY: regrow a tree at rate p */
                bool grow = rng_float() < p_grow;
                next = grow ? TREE : EMPTY;
                if (grow) nt++; else ne++;
            }
            f->next[r][c] = next;
        }
    }

    memcpy(f->grid, f->next, (size_t)f->rows * COLS_MAX);   /* commit: next → current */
    f->n_tree  = nt;
    f->n_fire  = nf;
    f->n_empty = ne;
    f->tick++;
}

/* ===================================================================== */
/* §6  effects  —  cosmetic-only state                                    */
/* ===================================================================== */

/*
 * Two cosmetic-only buffers, neither of which feeds back into the CA:
 *   - Forest.ash (§3) — the 1-tick burnt-ground overlay, PAINTED inside
 *     forest_step (§5) and READ by draw_forest (§7).
 *   - g_flash    (§3) — the HUD action-flash below, SET on a key, READ by draw_hud.
 */

/*
 * hud_flash — pop a brief, bright message on the HUD's top-left to confirm a key
 * action. The grow/lightning dials change probabilities whose effect on the
 * forest is gradual; this gives the keypress an instant, unmistakable response.
 */
static void hud_flash(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_flash, sizeof g_flash, fmt, ap);
    va_end(ap);
    g_flash_until = clock_ns() + FLASH_NS;
}

/* ===================================================================== */
/* §7  render  —  state → screen, reads only; palette setup               */
/* ===================================================================== */

/*
 * theme_apply — register the chosen palette with ncurses.
 *
 * Background = -1 (terminal default) for every scene pair so the
 * empty-grid background is the user's terminal colour.  PAIR_HUD /
 * PAIR_HINT are theme-independent and registered once in
 * screen_init() — this function only updates the four scene pairs
 * so cycling themes doesn't disturb the HUD.
 */
static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_TREE,   th->tree,   -1);
        init_pair(CP_FIRE1,  th->fire1,  -1);
        init_pair(CP_FIRE2,  th->fire2,  -1);
        init_pair(CP_ASH,    th->ash,    -1);
    } else {
        init_pair(CP_TREE,   th->tree8,   -1);
        init_pair(CP_FIRE1,  th->fire18,  -1);
        init_pair(CP_FIRE2,  th->fire28,  -1);
        init_pair(CP_ASH,    th->ash8,    -1);
    }
}

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check
 * that would otherwise be repeated at every mvaddch site.  The
 * double cast prevents sign-extension on character values > 127
 * (per CLAUDE.md "Common ncurses Bugs").  Off-screen cells are
 * silently dropped.
 */
static void mark_cell(int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(cy, cx, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_forest_cell — render one cell's glyph + colour at grid (r,c). The cell
 * is drawn one row down (r+1) to leave row 0 for the HUD. FIRE flickers: the
 * bright/dim choice alternates on a checkerboard that also advances with the
 * tick, so a burning patch crackles frame to frame.
 */
static void draw_forest_cell(const Forest *f, int r, int c, int cols, int rows)
{
    uint8_t state = f->grid[r][c];

    if (state == EMPTY) {
        if (f->ash[r][c])
            mark_cell(c, r + 1, '.', CP_ASH, A_DIM, cols, rows);   /* fresh burn scar */
        /* else: leave blank — terminal bg shows through */
    } else if (state == TREE) {
        mark_cell(c, r + 1, '^', CP_TREE, A_BOLD, cols, rows);
    } else { /* FIRE */
        bool bright = ((r + c + (int)f->tick) & 1);   /* checkerboard flicker phase */
        if (bright)
            mark_cell(c, r + 1, '*', CP_FIRE2, A_BOLD, cols, rows);
        else
            mark_cell(c, r + 1, ',', CP_FIRE1, A_NORMAL, cols, rows);
    }
}

static void draw_forest(const Forest *f, int cols, int rows)
{
    /* Reserve top + bottom rows for HUD bars. */
    int draw_rows = (f->rows < rows - 2) ? f->rows : rows - 2;
    int draw_cols = (f->cols < cols)     ? f->cols : cols;

    for (int r = 0; r < draw_rows; r++)
        for (int c = 0; c < draw_cols; c++)
            draw_forest_cell(f, r, c, cols, rows);
}

/*
 * draw_status — row-0 right-aligned readout: preset, theme, live tree/fire %,
 * the p/f rates, sim Hz, tick, run-state. PAIR_HUD bright yellow, A_BOLD.
 */
static void draw_status(const Scene *s, int cols)
{
    const Forest *f = &s->forest;
    int total = f->n_tree + f->n_fire + f->n_empty;
    float tree_pct = total > 0 ? 100.0f * (float)f->n_tree / (float)total : 0;
    float fire_pct = total > 0 ? 100.0f * (float)f->n_fire / (float)total : 0;

    char buf[160];
    snprintf(buf, sizeof buf,
        " [%d/%d %s] %s  trees:%5.1f%% fire:%4.1f%%  p=%.4f f=%.5f"
        "  sim:%dHz tick:%ld  %s ",
        s->preset + 1, N_PRESETS, k_presets[s->preset].name, k_themes[s->theme].name,
        tree_pct, fire_pct, s->p_grow, s->p_fire,
        s->sim_fps, f->tick,
        s->paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(buf);   /* right-align */
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_action_flash — the transient top-left key confirmation (EFFECT §6),
 * reverse-video so it stands out from the steady status bar. */
static void draw_action_flash(void)
{
    if (clock_ns() < g_flash_until) {
        attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
        mvprintw(0, 1, " %s ", g_flash);
        attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
    }
}

/* draw_keyhint — the bottom-row key legend. PAIR_HINT bright cyan, A_BOLD
 * (A_DIM is forbidden — it would mute the cyan to near-invisible over fire). */
static void draw_keyhint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme"
        "  g/G:grow  l/L:lightning  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void draw_hud(const Scene *s, int cols, int rows)
{
    draw_status(s, cols);
    draw_action_flash();
    draw_keyhint(rows);
}

/*
 * scene_draw — orchestrator.  erase the screen, draw the grid into
 * rows [1, rows−2), then over-stamp the HUD bars on rows 0 and rows−1.
 */
static void scene_draw(const Scene *s)
{
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    draw_forest(&s->forest, cols, rows);
    draw_hud   (s, cols, rows);
}

/* ===================================================================== */
/* §8  platform / app  —  ncurses, signals, resize, input, main loop      */
/* ===================================================================== */

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);

    start_color();
    use_default_colors();         /* allow bg = -1 (terminal default) */
    g_has_256 = (COLORS >= 256);

    /* Theme-independent HUD pairs — registered once, never re-registered
     * by theme_apply() so cycling themes leaves the HUD readable. */
    if (g_has_256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }

    theme_apply(g_scene.theme);
}

/* Size the forest to the current terminal, reserving 2 rows for the HUD bars. */
static void forest_fit_terminal(Forest *f)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    f->rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    f->cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
}

/* Re-read the terminal size after SIGWINCH and resize the forest to fit. */
static void screen_resize(Forest *f)
{
    endwin();
    refresh();
    forest_fit_terminal(f);
    g_resize = 0;
}

/* Route quit / resize signals to their flags. */
static void install_signals(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);
}

/* Seed the xorshift RNG from the wall clock (mixed so it is never 0). */
static void seed_rng(void)
{
    g_rng = (uint32_t)time(NULL) ^ 0xDEADBEEFu;
}

/*
 * handle_key — apply one keypress to the scene: quit, pause, reset, cycle the
 * preset / theme, nudge the p/f rates (with a HUD flash), or change sim rate.
 */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 27: g_quit = 1; break;

    case 'p': case ' ':
        s->paused = !s->paused;
        break;

    case 'r':
        scene_init(s, s->preset);
        break;

    case 'n':
        scene_init(s, (s->preset + 1) % N_PRESETS);
        break;
    case 'N':
        scene_init(s, (s->preset + N_PRESETS - 1) % N_PRESETS);
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        break;

    case 'g':
        if (s->p_grow < P_GROW_MAX) s->p_grow += P_GROW_STEP;
        hud_flash("grow  p=%.4f  +", s->p_grow);
        break;
    case 'G':
        if (s->p_grow > P_GROW_MIN) s->p_grow -= P_GROW_STEP;
        hud_flash("grow  p=%.4f  -", s->p_grow);
        break;

    case 'l':
        if (s->p_fire < P_FIRE_MAX) s->p_fire += P_FIRE_STEP;
        hud_flash("lightning  f=%.5f  +", s->p_fire);
        break;
    case 'L':
        if (s->p_fire > P_FIRE_MIN) s->p_fire -= P_FIRE_STEP;
        hud_flash("lightning  f=%.5f  -", s->p_fire);
        break;

    case '+': case '=':
        if (s->sim_fps < SIM_FPS_MAX) s->sim_fps += SIM_FPS_STEP;
        break;
    case '-':
        if (s->sim_fps > SIM_FPS_MIN) s->sim_fps -= SIM_FPS_STEP;
        break;
    }
}

int main(void)
{
    install_signals();
    seed_rng();
    screen_init();
    forest_fit_terminal(&g_scene.forest);
    scene_init(&g_scene, g_scene.preset);

    int64_t next_tick = clock_ns();

    while (!g_quit) {
        Scene *s = &g_scene;

        /* drain pending input */
        int ch;
        while ((ch = getch()) != ERR)
            handle_key(s, ch);

        if (g_resize) {
            screen_resize(&s->forest);
            scene_init(s, s->preset);
        }

        /* advance the CA when a fixed-timestep tick is due (and not paused) */
        int64_t now = clock_ns();
        if (!s->paused && now >= next_tick) {
            forest_step(&s->forest, s->p_grow, s->p_fire, s->eight_neighbor);
            next_tick = now + TICK_NS(s->sim_fps);
        }

        /* render one frame */
        scene_draw(s);
        wnoutrefresh(stdscr);
        doupdate();

        /* sleep until just before the next tick */
        clock_sleep_ns(next_tick - clock_ns() - SLEEP_MARGIN_NS);
    }

    endwin();
    return 0;
}
