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
 * Section map:
 *   §1 config   — grid sizes, default p/f, presets, sim Hz range
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 themes + PAIR_HUD/PAIR_HINT
 *   §4 grid     — double-buffer state, xorshift RNG, grid_step + helpers
 *   §5 scene    — mark_cell + draw_grid + draw_hud
 *   §6 screen   — ncurses init, resize handling
 *   §7 app      — signals, input, main loop
 *
 * Keys:
 *   q / ESC    quit                       p / space  pause / resume
 *   r          reset (reseed grid)        n / N      next / prev preset
 *   t / T      next / prev theme          g / G      grow prob p up / down
 *   l / L      lightning prob f up/down   s          scatter ignitions
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
 * References     : Drossel & Schwabl, "Self-organized critical forest-
 *                    fire model," Phys. Rev. Lett. 69 (1992) — the
 *                    canonical paper.
 *                  Bak, Tang & Wiesenfeld, "Self-organized criticality:
 *                    An explanation of 1/f noise," Phys. Rev. Lett. 59
 *                    (1987) — the SOC framework this CA falls under.
 *                  Gutenberg & Richter, "Frequency of earthquakes in
 *                    California," 1944 — same power-law size distribution
 *                    in geophysics.
 *                  Wikipedia: "Forest-fire model" — concise summary of
 *                    the rules and the SOC interpretation.
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
 *     spreading isotropically over many ticks.  `g_grid` and `g_next`
 *     are the double-buffer; `memcpy(g_grid, g_next, …)` flips them.
 *   - Boundary: the grid is finite; cells at edges have fewer
 *     neighbours.  Bounds-checked in `grid_step` (no toroidal wrap).
 *     Real fires don't wrap around the world either.
 *   - 8-neighbour spread (Smouldering preset) is more isotropic but
 *     ALSO faster — the critical p/f shifts.  Preset 3 lowers p
 *     accordingly so total fire area per tick stays comparable.
 *   - Theme palette: every entry must be in the bright half of the
 *     256-cube (≥ 24) per CLAUDE.md.  EMPTY cells render as space
 *     with bg=-1 (terminal default) so the user's terminal background
 *     shows through; tinting via dark grayscale (234-240) is forbidden.
 *   - HUD bottom row spans the full width; if `g_cols >= 80` the hint
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
 *   - Press `s` (manual scatter) — 12 trees suddenly turn FIRE; you
 *     can watch the fronts spread in real time.
 *   - Cycle presets with `n` — Dense visibly increases tree cover,
 *     Sparse leaves big bare patches, Smouldering (8-nbr) makes
 *     fires take diagonal corners that 4-nbr can't.
 *   - Cycle themes with `t` — palette changes immediately on next
 *     render frame; HUD pairs stay yellow/cyan regardless.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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

#define ROWS_MAX   128
#define COLS_MAX   512

/* Cell states */
#define EMPTY  0
#define TREE   1
#define FIRE   2

/* Default probabilities */
#define P_GROW_DEF   0.030f   /* P: empty → tree per tick                  */
#define P_FIRE_DEF   0.0002f  /* F: tree  → fire (lightning) per tick       */
#define P_GROW_STEP  0.005f
#define P_FIRE_STEP  0.0001f
#define P_GROW_MIN   0.001f
#define P_GROW_MAX   0.200f
#define P_FIRE_MIN   0.00001f
#define P_FIRE_MAX   0.010f

/* Manual lightning scatter */
#define SCATTER_COUNT  12    /* number of random ignitions per scatter key  */

/* Simulation timing */
#define SIM_FPS_DEF   20
#define SIM_FPS_MIN    2
#define SIM_FPS_MAX   60
#define SIM_FPS_STEP   2

#define N_PRESETS  4
#define N_THEMES   5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
/* §3  color / theme                                                      */
/* ===================================================================== */

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
 * Theme — one named palette for the four scene-glyph colours.
 *
 * Every entry sits in the bright half of the 256-colour cube (≥ 28
 * for the dimmest, primaries for the rest).  No grayscale 232-239,
 * no cube indices below 24 — those are forbidden by the brightness
 * rule and would render as near-black under A_BOLD or A_DIM.
 */
typedef struct {
    short tree, fire1, fire2, ash;       /* 256-colour indices  */
    short tree8, fire18, fire28, ash8;   /* 8-colour fallback   */
    const char *name;
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

static bool g_has_256;

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

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static uint8_t g_grid[ROWS_MAX][COLS_MAX];  /* current state              */
static uint8_t g_next[ROWS_MAX][COLS_MAX];  /* next state (double-buffer) */
static uint8_t g_ash [ROWS_MAX][COLS_MAX];  /* 1-tick ash flag            */

static int g_rows, g_cols;   /* actual grid dimensions (set from terminal) */

/* Preset configuration */
typedef struct {
    float p_grow;
    float p_fire;
    bool  eight_neighbor;   /* preset 3: 8-way spread instead of 4-way    */
    float density;          /* initial tree density                        */
    const char *name;
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { 0.030f, 0.0002f, false, 0.60f, "Classic" },
    { 0.060f, 0.0001f, false, 0.70f, "Dense"   },
    { 0.010f, 0.0010f, false, 0.40f, "Sparse"  },
    { 0.020f, 0.0003f, true,  0.55f, "Smoulder"},
};

/* Simulation state */
static float g_p_grow;
static float g_p_fire;
static bool  g_eight_neighbor;
static int   g_preset  = 0;
static int   g_theme   = 0;
static int   g_sim_fps = SIM_FPS_DEF;
static bool  g_paused  = false;

/* Statistics (displayed in HUD) */
static int g_n_tree, g_n_fire, g_n_empty;
static long g_tick = 0;

/* ------------------------------------------------------------------ */
/* LCG fast random for hot inner loop: xorshift32                      */
/* ------------------------------------------------------------------ */
static uint32_t g_rng = 12345;
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

/* ------------------------------------------------------------------ */

static void grid_seed(float density)
{
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++) {
            g_grid[r][c] = (rng_float() < density) ? TREE : EMPTY;
            g_ash[r][c]  = 0;
        }
}

static void scene_init(int preset)
{
    g_preset        = preset;
    g_p_grow        = k_presets[preset].p_grow;
    g_p_fire        = k_presets[preset].p_fire;
    g_eight_neighbor = k_presets[preset].eight_neighbor;
    g_tick          = 0;
    grid_seed(k_presets[preset].density);
}

/* ------------------------------------------------------------------ */
/* Single simulation step — decomposed into per-state helpers         */
/* ------------------------------------------------------------------ */

/*
 * has_fire_neighbor — return true if any 4- (and optionally 8-)
 * neighbour of cell (r, c) is FIRE in the CURRENT generation.
 * Bounds-checked; cells at edges have fewer neighbours.
 */
static bool has_fire_neighbor(int r, int c, bool eight)
{
    static const int dr4[4] = {-1, 1, 0, 0};
    static const int dc4[4] = { 0, 0,-1, 1};
    for (int d = 0; d < 4; d++) {
        int nr = r + dr4[d], nc = c + dc4[d];
        if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols
            && g_grid[nr][nc] == FIRE) return true;
    }
    if (!eight) return false;

    static const int dr8[4] = {-1,-1, 1, 1};
    static const int dc8[4] = {-1, 1,-1, 1};
    for (int d = 0; d < 4; d++) {
        int nr = r + dr8[d], nc = c + dc8[d];
        if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols
            && g_grid[nr][nc] == FIRE) return true;
    }
    return false;
}

/*
 * grid_step — one synchronous CA tick.
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
static void grid_step(void)
{
    int nt = 0, nf = 0, ne = 0;
    memset(g_ash, 0, sizeof(g_ash));

    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            uint8_t state = g_grid[r][c];
            uint8_t next  = state;

            if (state == FIRE) {
                next = EMPTY; g_ash[r][c] = 1; ne++;
            } else if (state == TREE) {
                bool ignite = has_fire_neighbor(r, c, g_eight_neighbor)
                           || rng_float() < g_p_fire;
                next = ignite ? FIRE : TREE;
                if (ignite) nf++; else nt++;
            } else { /* EMPTY */
                next = (rng_float() < g_p_grow) ? TREE : EMPTY;
                if (next == TREE) nt++; else ne++;
            }
            g_next[r][c] = next;
        }
    }

    memcpy(g_grid, g_next, (size_t)g_rows * COLS_MAX);
    g_n_tree  = nt;
    g_n_fire  = nf;
    g_n_empty = ne;
    g_tick++;
}

/* Manual scatter: randomly ignite SCATTER_COUNT trees */
static void grid_scatter(void)
{
    int tries = SCATTER_COUNT * 8;
    int lit   = 0;
    while (lit < SCATTER_COUNT && tries-- > 0) {
        int r = (int)(rng_float() * (float)g_rows);
        int c = (int)(rng_float() * (float)g_cols);
        if (r >= 0 && r < g_rows && c >= 0 && c < g_cols
                && g_grid[r][c] == TREE) {
            g_grid[r][c] = FIRE;
            lit++;
        }
    }
}

/* ===================================================================== */
/* §5  scene draw                                                         */
/* ===================================================================== */

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
 * draw_grid — paint the CA state into the terminal.
 *
 * Per-cell glyph + colour pair selection:
 *   EMPTY (no ash) → ' '  (terminal background shows through)
 *   EMPTY (ash)    → '.' CP_ASH A_DIM   (one-tick burnt-ground overlay)
 *   TREE           → '^' CP_TREE A_BOLD
 *   FIRE bright    → '*' CP_FIRE2 A_BOLD  (flicker phase by parity)
 *   FIRE dim       → ',' CP_FIRE1 A_NORMAL
 *
 * The fire flicker uses `(r + c + tick) & 1` so adjacent cells in
 * the same fire patch alternate bright/dim and the patch appears to
 * crackle frame to frame.
 */
static void draw_grid(int cols, int rows)
{
    /* Reserve top + bottom rows for HUD bars. */
    int draw_rows = (g_rows < rows - 2) ? g_rows : rows - 2;
    int draw_cols = (g_cols < cols)     ? g_cols : cols;

    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {
            uint8_t state = g_grid[r][c];

            if (state == EMPTY) {
                if (g_ash[r][c])
                    mark_cell(c, r + 1, '.', CP_ASH, A_DIM, cols, rows);
                /* else: leave blank — terminal bg shows through */
            } else if (state == TREE) {
                mark_cell(c, r + 1, '^', CP_TREE, A_BOLD, cols, rows);
            } else { /* FIRE */
                bool bright = ((r + c + (int)g_tick) & 1);
                if (bright)
                    mark_cell(c, r + 1, '*', CP_FIRE2, A_BOLD, cols, rows);
                else
                    mark_cell(c, r + 1, ',', CP_FIRE1, A_NORMAL, cols, rows);
            }
        }
    }
}

/*
 * draw_hud — top-right status (PAIR_HUD) + bottom key hint (PAIR_HINT).
 *
 * Both rows use A_BOLD per the project HUD spec — A_DIM is forbidden
 * on the hint bar because it would mute the cyan to near-invisible
 * over a bright forest backdrop.  Background = -1 (terminal default).
 */
static void draw_hud(int cols, int rows)
{
    int total = g_n_tree + g_n_fire + g_n_empty;
    float tree_pct = total > 0 ? 100.0f * (float)g_n_tree / (float)total : 0;
    float fire_pct = total > 0 ? 100.0f * (float)g_n_fire / (float)total : 0;

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[160];
    snprintf(buf, sizeof buf,
        " [%s] %s  trees:%5.1f%% fire:%4.1f%%  p=%.4f f=%.5f"
        "  sim:%dHz tick:%ld  %s ",
        k_presets[g_preset].name, k_themes[g_theme].name,
        tree_pct, fire_pct, g_p_grow, g_p_fire,
        g_sim_fps, g_tick,
        g_paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme"
        "  g/G:grow  l/L:lightning  s:scatter  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * scene_draw — orchestrator.  erase the screen, draw the grid into
 * rows [1, rows−2), then over-stamp the HUD bars on rows 0 and rows−1.
 */
static void scene_draw(void)
{
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    draw_grid(cols, rows);
    draw_hud (cols, rows);
}

/* ===================================================================== */
/* §6  screen / ncurses init                                              */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

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

    theme_apply(g_theme);
}

static void screen_resize(void)
{
    endwin();
    refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    g_cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    /* Seed RNG from time */
    g_rng = (uint32_t)time(NULL) ^ 0xDEADBEEFu;

    screen_init();

    {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
        g_cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
    }

    scene_init(g_preset);

    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;

            case 'p': case ' ':
                g_paused = !g_paused;
                break;

            case 'r':
                scene_init(g_preset);
                break;

            case 'n':
                scene_init((g_preset + 1) % N_PRESETS);
                break;
            case 'N':
                scene_init((g_preset + N_PRESETS - 1) % N_PRESETS);
                break;

            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme);
                break;

            case 'g':
                if (g_p_grow < P_GROW_MAX) g_p_grow += P_GROW_STEP;
                break;
            case 'G':
                if (g_p_grow > P_GROW_MIN) g_p_grow -= P_GROW_STEP;
                break;

            case 'l':
                if (g_p_fire < P_FIRE_MAX) g_p_fire += P_FIRE_STEP;
                break;
            case 'L':
                if (g_p_fire > P_FIRE_MIN) g_p_fire -= P_FIRE_STEP;
                break;

            case 's':
                grid_scatter();
                break;

            case '+': case '=':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;
            case '-':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── handle resize ── */
        if (g_resize) {
            screen_resize();
            scene_init(g_preset);
        }

        /* ── tick ── */
        int64_t now = clock_ns();
        if (!g_paused && now >= next_tick) {
            grid_step();
            next_tick = now + TICK_NS(g_sim_fps);
        }

        /* ── render ── */
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next tick ── */
        int64_t sleep_ns = next_tick - clock_ns() - 1000000LL;
        clock_sleep_ns(sleep_ns);
    }

    endwin();
    return 0;
}
