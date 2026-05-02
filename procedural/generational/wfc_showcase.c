/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wfc_showcase.c — Wave Function Collapse, scaled up for spectacle.
 *
 * DEMO: Full-terminal grid (~200×60 cells), 34-tile alphabet of light /
 *       heavy / double box-drawing pipes that only connect to their own
 *       weight class. Five random seed collapses at startup ripple
 *       outward, meeting in the middle with visible interference; the
 *       three weight classes form coloured "domains" (cyan-light /
 *       pink-heavy / gold-double) separated by blank gaps. After full
 *       collapse the grid holds, dissolves in a yellow flash, and the
 *       whole thing loops forever.
 *
 * Study alongside: wfc_learn.c — same algorithm, slowed down with an
 *       entropy heatmap and a propagation queue you can single-step
 *       through. Read learn first; this file extends it for visual punch.
 *
 * Section map:
 *   §1 config   — grid, glow rates, weight palette, alphabet count
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + per-weight tile colors + flash/wave
 *   §5 wfc      — 34-tile table, 4-valued edges, compat, Grid (uint64_t)
 *   §6 scene    — multi-seed init, auto-loop state machine, glow decay
 *   §7 screen   — UTF-8 render; per-weight color routing
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate re-seed, no flash)
 *   +/-        more / fewer ops per tick
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wfc_showcase.c -o wfc_showcase \
 *       -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Tile-based Wave Function Collapse — same algorithm as
 *                  wfc_learn.c but with a 34-tile alphabet. The edge
 *                  state per side is no longer binary {connect, no} but
 *                  4-valued {NONE, LIGHT, HEAVY, DOUBLE}, encoding which
 *                  weight class connects there. Two tiles match across an
 *                  edge iff their facing edges have the SAME state — so
 *                  different weights meet only at NONE-NONE seams,
 *                  producing visible coloured domains.
 *
 * Data-structure : uint64_t bitmask per cell (34 bits used, 30 spare).
 *                  Compat table compat[34][4] of uint64_t. Flat 1-D grid;
 *                  propagation queue with per-cell in_queue[] flag (same
 *                  fix as learn — see learn.c for the bug story).
 *
 * Rendering      : ASCII glyphs only ({' ', '-', '|', '+'}) for terminal
 *                  portability. Each weight class draws in its own colour:
 *                  light=cyan, heavy=pink, double=gold — colour IS the
 *                  weight differentiator since all junctions render as '+'.
 *                  A per-cell prop_glow buffer
 *                  decays slowly (rate 1.5/s) so the propagation wave
 *                  remains visible as a yellow ribbon for ~1.5 s after
 *                  it passes; collapse_glow flashes red briefly on each
 *                  deliberate min-entropy collapse. After DONE: hold,
 *                  reset every cell to full superposition with a yellow
 *                  prop_glow flash, then re-seed.
 *
 * Performance    : O(N · K) per full grid collapse, where N is cells and
 *                  K is alphabet size (34). Worst case ≈ 300 M ops for a
 *                  200×60 grid; we throttle to ops_per_tick (default 32)
 *                  so the spectacle unfolds over ~5–10 s. No allocation
 *                  after init.
 *
 * References     : • Same as wfc_learn.c — Gumin's WFC repo, Karth &
 *                    Smith 2017, Boris the Brave's tutorial.
 *                  • Unicode Box Drawing block (U+2500–U+257F) for the
 *                    full pipe inventory used here:
 *                    https://en.wikipedia.org/wiki/Box-drawing_character
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell holds a 64-bit "could be" set over 34 pipe tiles. The
 * showcase differs from learn only in alphabet size and pacing —
 * everything in this block is a delta against learn's mental model.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * A pond with five stones dropped at once. Each stone is a forced
 * collapse; each ripple is constraint propagation. Where ripples
 * meet, they don't cancel — they MERGE, because constraints are
 * monotone (once a tile is removed it never comes back). So the
 * pattern always converges, but the boundary geometry depends on
 * which ripple arrived first.
 *
 * Three ripple "colours" exist (light / heavy / double weight). At
 * a domain boundary, both sides agree on NONE — so domains are
 * separated by a thin moat of blank cells where the pipe network
 * simply stops. That blank seam is the visible signature of the
 * 4-valued edge state.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Seed: pick N_INITIAL_SEEDS random cells; collapse each to a
 *     uniformly-chosen tile from its full superposition.
 *  2. Drain the propagation queue (same as learn step 3–4).
 *  3. Pick lowest-entropy uncollapsed cell, collapse, drain.
 *  4. When every cell has entropy 1 → state = HOLD.
 *  5. After HOLD_SECONDS → reset every cell to full superposition,
 *     paint the screen yellow via prop_glow=1.0 (the "supernova"
 *     flash), and goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Edge match (4-valued)         : a.edge[d] == b.edge[opposite(d)]
 *  Compat (precomputed)          : compat[t][d] = ⋃ {1<<t' : edges match}
 *  Mask type                     : uint64_t   (34 bits used)
 *  Popcount                      : __builtin_popcountll
 *  Lowest-bit index              : __builtin_ctzll
 *  Glow decay (per frame)        : glow *= exp(-rate * dt)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CONTRADICTIONS are MORE common with multiple seeds: two ripples
 *    of incompatible weight colliding in a tight space can leave a
 *    cell with no legal tile. Strategy here is "tolerate" — flag bad
 *    and trigger an early reset, rather than backtrack. The visual
 *    cost is one early restart every ~5 runs; backtracking would
 *    cost much more code and the showcase can afford the occasional
 *    glitch.
 *
 *  • UINT64_T MASKS need __builtin_popcountll and __builtin_ctzll
 *    (NOT the 32-bit variants). On x86-64 these are single POPCNT/
 *    TZCNT instructions; performance is fine.
 *
 *  • COLOR PAIRS: PAIR_HUD (226 yellow) and PAIR_HINT (51 cyan) are
 *    RESERVED across the whole project (see CLAUDE.md HUD Standard).
 *    Tile-light deliberately uses 117 (a bluer cyan than 51) so it
 *    is distinguishable from the hint strip.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Distinct domains visible: any rendered frame after stabilisation
 *    should show clear cyan / pink / gold "regions" separated by
 *    blanks. If everything is one colour, weight isolation is broken.
 *  • Five seed flashes at startup: count the red flashes in the first
 *    half-second. Should equal N_INITIAL_SEEDS exactly.
 *  • Loop completes: leave the demo running 60 s. You should see at
 *    least 3 full collapse-and-reset cycles. If it freezes mid-cycle,
 *    a contradiction wasn't caught and the state machine stalled.
 *  • HUD tile counts: collapsed_count should monotonically rise to
 *    total_cells, then snap back to 0 on the supernova reset. If it
 *    overshoots total_cells, double-counting is back.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <locale.h>
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
    GRID_W_MAX        = 240,
    GRID_H_MAX        =  80,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,        /* spectacle pace — full grid in ≈8 s */
    OPS_PER_TICK_MAX  = 512,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    N_TILES           =  34,        /* 1 blank + 11 light + 11 heavy + 11 double */
    N_INITIAL_SEEDS   =   5,        /* multi-ripple startup */

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_LIGHT        =   3,        /* tile-weight: light box-drawing  */
    PAIR_HEAVY        =   4,        /* tile-weight: heavy box-drawing  */
    PAIR_DOUBLE       =   5,        /* tile-weight: double box-drawing */
    PAIR_FLASH        =   6,        /* deliberate-collapse red flash   */
    PAIR_WAVE         =   7,        /* propagation glow yellow         */
    PAIR_SEED_FLASH   =   8,        /* startup seed glow white         */
};

/*
 * Glow decay rates. Slower than learn.c (4.0) because the showcase
 * wants the trail to LINGER — the goal here is spectacle, not
 * pedagogy. exp(-1.5 * 1.0) ≈ 0.22 → glow falls to ~22% in 1 second,
 * ~5% in 2 seconds. Long enough for the eye to follow the wavefront
 * across a wide grid.
 */
#define GLOW_DECAY_RATE      1.5f
#define GLOW_FLASH_THRESHOLD 0.05f

/* Hold and supernova timings — state-machine in §6 scene. */
#define HOLD_SECONDS         1.6f   /* pause on a finished pattern */
#define SUPERNOVA_GLOW       1.0f   /* prop_glow assigned at reset */

/* All bits set for our 34-tile alphabet. */
#define ALL_TILES_MASK ((N_TILES >= 64) ? (~(uint64_t)0) \
                                        : (((uint64_t)1 << N_TILES) - 1u))

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Per CLAUDE.md HUD Standard:
 *   PAIR_HUD  = 226 (bright yellow), A_BOLD on use
 *   PAIR_HINT =  51 (bright cyan),   A_BOLD on use
 *
 * Tile weights are colored to be distinct from HUD/HINT and from each
 * other:
 *   LIGHT  = 117 (bluer cyan than HINT)
 *   HEAVY  = 213 (pink/magenta)
 *   DOUBLE = 220 (gold)
 *   FLASH  = 196 (red), WAVE = 226 (same yellow as HUD ok — never
 *           drawn on HUD row), SEED = 231 (near-white).
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_LIGHT,      117, -1);
        init_pair(PAIR_HEAVY,      213, -1);
        init_pair(PAIR_DOUBLE,     220, -1);
        init_pair(PAIR_FLASH,      196, -1);
        init_pair(PAIR_WAVE,       226, -1);
        init_pair(PAIR_SEED_FLASH, 231, -1);
    } else {
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_LIGHT,      COLOR_CYAN,    -1);
        init_pair(PAIR_HEAVY,      COLOR_MAGENTA, -1);
        init_pair(PAIR_DOUBLE,     COLOR_YELLOW,  -1);
        init_pair(PAIR_FLASH,      COLOR_RED,     -1);
        init_pair(PAIR_WAVE,       COLOR_YELLOW,  -1);
        init_pair(PAIR_SEED_FLASH, COLOR_WHITE,   -1);
    }
}

/* ===================================================================== */
/* §5  wfc — alphabet, compat, grid                                       */
/* ===================================================================== */

/*
 * Direction encoding: same as wfc_learn.c (N=0, E=1, S=2, W=3).
 * Read learn.c for the why.
 */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/*
 * EdgeState — 4-valued edge label.
 *
 * NONE      = no connection on this side (blank, or stub-end).
 * LIGHT     = connects to a light-weight (cyan) tile on the matching side.
 * HEAVY     = connects to a heavy-weight (pink) tile.
 * DOUBLE    = connects to a double-weight (gold) tile.
 * (All three weight classes render with the same ASCII glyph set;
 * the visible difference is COLOUR.)
 *
 * Two tiles match across direction d iff
 *   tiles[a].edge[d] == tiles[b].edge[opposite(d)].
 * Equality of EdgeState — not just non-zero — so light/heavy/double
 * weights never connect across each other; they meet at NONE seams.
 */
typedef enum {
    EDGE_NONE   = 0,
    EDGE_LIGHT  = 1,
    EDGE_HEAVY  = 2,
    EDGE_DOUBLE = 3,
} EdgeState;

typedef enum {
    WEIGHT_LIGHT  = 0,
    WEIGHT_HEAVY  = 1,
    WEIGHT_DOUBLE = 2,
    WEIGHT_BLANK  = 3,   /* visual class only — blank has no edges to any */
} Weight;

typedef struct {
    const char *glyph;
    EdgeState   edge[N_DIRS];   /* [N, E, S, W] */
    Weight      weight;         /* drives color choice */
} Tile;

/*
 * tiles[] — 34-entry alphabet.
 *
 *   0           : blank (no connections, weight=BLANK)
 *   1..11       : light pipes — cyan, glyphs in {' ', '-', '|', '+'}
 *   12..22      : heavy pipes — pink, same glyph set
 *   23..33      : double pipes — gold, same glyph set
 *
 * Each weight is the full 11-tile T-junction set, so the constraint
 * solver always has every connectivity option available — no dead-end
 * stubs that would force contradictions.
 */
#define E_NONE   EDGE_NONE
#define E_L      EDGE_LIGHT
#define E_H      EDGE_HEAVY
#define E_D      EDGE_DOUBLE

/* ASCII-only glyphs for terminal portability (no UTF-8/locale dependency).
 * The three weight classes use the same character set ({' ', '-', '|',
 * '+'}); the visible difference between LIGHT/HEAVY/DOUBLE comes from
 * COLOR alone (cyan / pink / gold). Connectivity is fully tracked in the
 * 4-valued edge[] flags, so the algorithm enforces "weights only meet
 * across NONE seams" exactly as before. */
static const Tile tiles[N_TILES] = {
    /* 0  blank */ { " ",  {E_NONE, E_NONE, E_NONE, E_NONE}, WEIGHT_BLANK  },

    /* light — weight LIGHT, edges in {NONE, LIGHT} */
    /* 1  ─ */ { "-", {E_NONE, E_L,    E_NONE, E_L   }, WEIGHT_LIGHT },
    /* 2  │ */ { "|", {E_L,    E_NONE, E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 3  ┌ */ { "+", {E_NONE, E_L,    E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 4  ┐ */ { "+", {E_NONE, E_NONE, E_L,    E_L   }, WEIGHT_LIGHT },
    /* 5  └ */ { "+", {E_L,    E_L,    E_NONE, E_NONE}, WEIGHT_LIGHT },
    /* 6  ┘ */ { "+", {E_L,    E_NONE, E_NONE, E_L   }, WEIGHT_LIGHT },
    /* 7  ├ */ { "+", {E_L,    E_L,    E_L,    E_NONE}, WEIGHT_LIGHT },
    /* 8  ┤ */ { "+", {E_L,    E_NONE, E_L,    E_L   }, WEIGHT_LIGHT },
    /* 9  ┬ */ { "+", {E_NONE, E_L,    E_L,    E_L   }, WEIGHT_LIGHT },
    /*10  ┴ */ { "+", {E_L,    E_L,    E_NONE, E_L   }, WEIGHT_LIGHT },
    /*11  ┼ */ { "+", {E_L,    E_L,    E_L,    E_L   }, WEIGHT_LIGHT },

    /* heavy — weight HEAVY, edges in {NONE, HEAVY} */
    /*12  ━ */ { "-", {E_NONE, E_H,    E_NONE, E_H   }, WEIGHT_HEAVY },
    /*13  ┃ */ { "|", {E_H,    E_NONE, E_H,    E_NONE}, WEIGHT_HEAVY },
    /*14  ┏ */ { "+", {E_NONE, E_H,    E_H,    E_NONE}, WEIGHT_HEAVY },
    /*15  ┓ */ { "+", {E_NONE, E_NONE, E_H,    E_H   }, WEIGHT_HEAVY },
    /*16  ┗ */ { "+", {E_H,    E_H,    E_NONE, E_NONE}, WEIGHT_HEAVY },
    /*17  ┛ */ { "+", {E_H,    E_NONE, E_NONE, E_H   }, WEIGHT_HEAVY },
    /*18  ┣ */ { "+", {E_H,    E_H,    E_H,    E_NONE}, WEIGHT_HEAVY },
    /*19  ┫ */ { "+", {E_H,    E_NONE, E_H,    E_H   }, WEIGHT_HEAVY },
    /*20  ┳ */ { "+", {E_NONE, E_H,    E_H,    E_H   }, WEIGHT_HEAVY },
    /*21  ┻ */ { "+", {E_H,    E_H,    E_NONE, E_H   }, WEIGHT_HEAVY },
    /*22  ╋ */ { "+", {E_H,    E_H,    E_H,    E_H   }, WEIGHT_HEAVY },

    /* double — weight DOUBLE, edges in {NONE, DOUBLE} */
    /*23  ═ */ { "-", {E_NONE, E_D,    E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*24  ║ */ { "|", {E_D,    E_NONE, E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*25  ╔ */ { "+", {E_NONE, E_D,    E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*26  ╗ */ { "+", {E_NONE, E_NONE, E_D,    E_D   }, WEIGHT_DOUBLE },
    /*27  ╚ */ { "+", {E_D,    E_D,    E_NONE, E_NONE}, WEIGHT_DOUBLE },
    /*28  ╝ */ { "+", {E_D,    E_NONE, E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*29  ╠ */ { "+", {E_D,    E_D,    E_D,    E_NONE}, WEIGHT_DOUBLE },
    /*30  ╣ */ { "+", {E_D,    E_NONE, E_D,    E_D   }, WEIGHT_DOUBLE },
    /*31  ╦ */ { "+", {E_NONE, E_D,    E_D,    E_D   }, WEIGHT_DOUBLE },
    /*32  ╩ */ { "+", {E_D,    E_D,    E_NONE, E_D   }, WEIGHT_DOUBLE },
    /*33  ╬ */ { "+", {E_D,    E_D,    E_D,    E_D   }, WEIGHT_DOUBLE },
};

#undef E_NONE
#undef E_L
#undef E_H
#undef E_D

/* compat[t][d] — see wfc_learn.c §5 for the why. uint64_t because
 * we have 34 tiles. Built once at startup. */
static uint64_t compat[N_TILES][N_DIRS];

static void wfc_build_compat(void)
{
    for (int t = 0; t < N_TILES; t++) {
        for (int d = 0; d < N_DIRS; d++) {
            uint64_t mask = 0;
            int od = opposite(d);
            for (int t2 = 0; t2 < N_TILES; t2++) {
                if (tiles[t].edge[d] == tiles[t2].edge[od])
                    mask |= ((uint64_t)1 << t2);
            }
            compat[t][d] = mask;
        }
    }
}

/* Grid — see wfc_learn.c §5 for invariant docs. The structure here is
 * identical except the mask type is uint64_t instead of uint16_t. */
typedef struct {
    int      w, h;
    uint64_t mask         [GRID_W_MAX * GRID_H_MAX];
    float    collapse_glow[GRID_W_MAX * GRID_H_MAX];
    float    prop_glow    [GRID_W_MAX * GRID_H_MAX];
    int      queue        [GRID_W_MAX * GRID_H_MAX];
    bool     in_queue     [GRID_W_MAX * GRID_H_MAX];
    int      qhead, qtail;

    bool     done;
    bool     bad;

    int      collapsed_count;
    int      total_cells;
} Grid;

static void grid_reset(Grid *g, int w, int h, float startup_glow)
{
    g->w = w;
    g->h = h;
    g->total_cells = w * h;
    g->collapsed_count = 0;
    g->done  = false;
    g->bad   = false;
    g->qhead = 0;
    g->qtail = 0;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        g->mask[i]          = ALL_TILES_MASK;
        g->collapse_glow[i] = 0.0f;
        g->prop_glow[i]     = startup_glow;   /* "supernova" yellow flash */
        g->in_queue[i]      = false;
    }
}

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

static inline void grid_enqueue(Grid *g, int idx)
{
    if (g->in_queue[idx]) return;
    if (g->qtail >= g->total_cells) return;
    g->queue[g->qtail++] = idx;
    g->in_queue[idx] = true;
}

/*
 * grid_pick_min_entropy — same heuristic as learn.c, with reservoir
 * sampling among ties to avoid row-major bias. Returns -1 when no
 * uncollapsed cell exists.
 */
static int grid_pick_min_entropy(const Grid *g)
{
    int best_n  = N_TILES + 1;
    int tied    = 0;
    int chosen  = -1;
    int n_cells = g->total_cells;

    for (int i = 0; i < n_cells; i++) {
        int p = __builtin_popcountll(g->mask[i]);
        if (p <= 1) continue;
        if (p < best_n) {
            best_n = p;
            chosen = i;
            tied   = 1;
        } else if (p == best_n) {
            tied++;
            if ((rand() % tied) == 0) chosen = i;
        }
    }
    return chosen;
}

static void grid_collapse_cell(Grid *g, int idx)
{
    uint64_t m = g->mask[idx];
    int n = __builtin_popcountll(m);
    if (n <= 1) return;

    int pick = rand() % n;
    int chosen_tile = -1;
    while (m) {
        int t = __builtin_ctzll(m);
        if (pick == 0) { chosen_tile = t; break; }
        pick--;
        m &= m - 1;
    }

    g->mask[idx] = (uint64_t)1 << chosen_tile;
    g->collapse_glow[idx] = 1.0f;
    g->collapsed_count++;
    grid_enqueue(g, idx);
}

/*
 * grid_propagate_one — see learn.c for the AC-3 logic. Mask is now
 * uint64_t and the inner OR-loop walks up to N_TILES bits.
 */
static bool grid_propagate_one(Grid *g)
{
    if (g->qhead >= g->qtail) return false;

    int idx = g->queue[g->qhead++];
    g->in_queue[idx] = false;
    int x = idx % g->w;
    int y = idx / g->w;
    uint64_t my_mask = g->mask[idx];

    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!grid_in_bounds(g, nx, ny)) continue;
        int nidx = grid_idx(g, nx, ny);

        uint64_t allowed = 0;
        uint64_t m = my_mask;
        while (m) {
            int t = __builtin_ctzll(m);
            allowed |= compat[t][d];
            m &= m - 1;
        }

        uint64_t before = g->mask[nidx];
        uint64_t after  = before & allowed;
        if (after != before) {
            g->mask[nidx] = after;
            g->prop_glow[nidx] = 1.0f;
            if (after == 0) {
                g->bad = true;
            } else if (__builtin_popcountll(after) == 1
                    && __builtin_popcountll(before) > 1) {
                g->collapsed_count++;
            }
            grid_enqueue(g, nidx);
        }
    }
    return true;
}

static bool grid_step(Grid *g)
{
    if (g->bad || g->done) return false;
    if (grid_propagate_one(g)) return true;

    int idx = grid_pick_min_entropy(g);
    if (idx < 0) { g->done = true; return false; }
    grid_collapse_cell(g, idx);
    return true;
}

/*
 * grid_seed_random — pick a random uncollapsed cell and force-collapse
 * it. Used at startup to drop N_INITIAL_SEEDS ripple centres.
 *
 * The "reservoir among uncollapsed" pattern means we don't have to
 * track collapse history — every call sees the current grid state.
 */
static void grid_seed_random(Grid *g)
{
    int chosen = -1;
    int tied   = 0;
    int n_cells = g->total_cells;
    for (int i = 0; i < n_cells; i++) {
        if (__builtin_popcountll(g->mask[i]) > 1) {
            tied++;
            if ((rand() % tied) == 0) chosen = i;
        }
    }
    if (chosen >= 0) grid_collapse_cell(g, chosen);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   GROWING    : algorithm running normally — collapses + propagation.
 *                Enters HOLD when grid_step returns false with done=true.
 *   HOLD       : finished pattern displayed for HOLD_SECONDS.
 *                Enters GROWING via supernova reset on timeout.
 *
 * Contradictions (g.bad=true) trigger an immediate reset without HOLD —
 * the user shouldn't have to stare at a stuck grid.
 *
 * `paused` is independent of the state machine: when paused, scene_tick
 * is a no-op (no ops, no glow decay, no state transitions). This freezes
 * whatever is on screen, useful for screenshots.
 */
typedef enum { SCENE_GROWING = 0, SCENE_HOLD = 1 } SceneState;

typedef struct {
    Grid       g;
    SceneState state;
    float      hold_timer;     /* counts down in HOLD state */
    bool       paused;
    int        ops_per_tick;
} Scene;

/*
 * scene_supernova_reset — full re-seed with a yellow flash.
 *
 * Sets every cell back to full superposition AND assigns prop_glow=1.0
 * to the entire grid, which the renderer paints yellow → fades back to
 * tile colours over ~1.5 s. After the wash, drops N_INITIAL_SEEDS
 * collapses to start the new pattern.
 *
 * Called on first init, on user 'r', on contradiction, and when HOLD
 * timer expires.
 */
static void scene_supernova_reset(Scene *s, int gw, int gh)
{
    grid_reset(&s->g, gw, gh, SUPERNOVA_GLOW);
    s->state      = SCENE_GROWING;
    s->hold_timer = 0.0f;
    for (int i = 0; i < N_INITIAL_SEEDS; i++) {
        grid_seed_random(&s->g);
    }
}

static void scene_init(Scene *s, int gw, int gh)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    scene_supernova_reset(s, gw, gh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Glow decay first — same reasoning as learn.c. */
    float decay = expf(-GLOW_DECAY_RATE * dt);
    int n = s->g.total_cells;
    for (int i = 0; i < n; i++) {
        s->g.collapse_glow[i] *= decay;
        s->g.prop_glow[i]     *= decay;
    }

    switch (s->state) {
    case SCENE_GROWING: {
        /* Run up to ops_per_tick algorithm operations. */
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!grid_step(&s->g)) break;
        }
        if (s->g.bad) {
            /* Contradiction — restart immediately. */
            scene_supernova_reset(s, s->g.w, s->g.h);
        } else if (s->g.done) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
    } break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_supernova_reset(s, s->g.w, s->g.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Same canonical pattern as wfc_learn.c §7 / framework.c §7:
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate
 *
 * UTF-8: setlocale + mvaddstr (multi-byte glyphs).
 *
 * Rendering priority per cell:
 *   collapse_glow > threshold  →  red bold (PAIR_FLASH)
 *   prop_glow     > threshold  →  yellow bold (PAIR_WAVE)
 *   collapsed                  →  weight colour from PAIR_LIGHT/HEAVY/DOUBLE
 *   uncollapsed                →  blank
 *
 * We do NOT draw entropy digits in the showcase — they're a teaching
 * crutch. Uncollapsed cells just stay invisible until a wave lands or
 * the cell collapses, which keeps the spectacle clean.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    setlocale(LC_ALL, "");
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

/* Map a tile's weight class to its color pair. Blank uses LIGHT but
 * wouldn't be drawn anyway (renderer skips uncollapsed/blank cells). */
static int weight_pair(Weight w)
{
    switch (w) {
    case WEIGHT_LIGHT:  return PAIR_LIGHT;
    case WEIGHT_HEAVY:  return PAIR_HEAVY;
    case WEIGHT_DOUBLE: return PAIR_DOUBLE;
    case WEIGHT_BLANK:  return PAIR_LIGHT;   /* irrelevant — see above */
    }
    return PAIR_LIGHT;
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid *g = &s->g;

    /* Center the grid (with a 1-row HUD top, 1-row hint bottom margin). */
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - 2) - g->h) / 2 + 1;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 1) gy0 = 1;

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 1 || sy >= rows - 1) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int      idx = grid_idx(g, x, y);
            uint64_t m   = g->mask[idx];
            int      p   = __builtin_popcountll(m);
            float    cg  = g->collapse_glow[idx];
            float    pg  = g->prop_glow[idx];
            bool     collapsed = (p == 1);

            /* Glow priority: red flash > yellow wave > weight color. */
            int color_pair = -1;
            int attr       = A_NORMAL;

            if (cg > GLOW_FLASH_THRESHOLD) {
                color_pair = PAIR_FLASH;
                attr       = A_BOLD;
            } else if (pg > GLOW_FLASH_THRESHOLD) {
                color_pair = PAIR_WAVE;
                attr       = A_BOLD;
            } else if (collapsed) {
                int t = __builtin_ctzll(m);
                if (tiles[t].weight == WEIGHT_BLANK) continue;   /* skip */
                color_pair = weight_pair(tiles[t].weight);
                attr       = A_BOLD;
            } else {
                continue;   /* uncollapsed and no glow — leave blank */
            }

            attron(COLOR_PAIR(color_pair) | attr);
            if (collapsed) {
                int t = __builtin_ctzll(m);
                mvaddstr(sy, sx, tiles[t].glyph);
            } else {
                /* Glow on uncollapsed cell — paint a solid block so the
                 * wave is visible even before collapse decides anything. */
                mvaddch(sy, sx, (chtype)(unsigned char)'#');
            }
            attroff(COLOR_PAIR(color_pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Grid *g = &s->g;
    const char *state_str =
        s->paused          ? "PAUSED " :
        (s->state == SCENE_HOLD) ? "HOLD   " :
        g->bad             ? "BAD!   " :
                             "GROWING";

    /* Top-right status — PAIR_HUD bold per CLAUDE.md HUD Standard. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops/tick:%3d  %s  %5d/%-5d ",
             fps, sim_fps, s->ops_per_tick, state_str,
             g->collapsed_count, g->total_cells);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Top-left title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WAVE FUNCTION COLLAPSE — showcase ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — A_BOLD, never A_DIM (CLAUDE.md). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   grid_w, grid_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_grid_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;     /* leave HUD top + hint bottom */
    if (mw < 8) mw = 8;
    if (mh < 4) mh = 4;
    if (mw > GRID_W_MAX) mw = GRID_W_MAX;
    if (mh > GRID_H_MAX) mh = GRID_H_MAX;
    app->grid_w = mw;
    app->grid_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_grid_size(app);
    scene_supernova_reset(&app->scene, app->grid_w, app->grid_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_supernova_reset(s, app->grid_w, app->grid_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    wfc_build_compat();
    app_pick_grid_size(app);
    scene_init(&app->scene, app->grid_w, app->grid_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
