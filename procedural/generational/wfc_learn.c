/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wfc_learn.c — Wave Function Collapse, slowed down so you can see it think.
 *
 * DEMO: A 50×20 grid starts in full superposition (every cell could be any
 *       of 12 box-drawing pipe tiles). Each tick the algorithm performs ONE
 *       atomic operation — either collapsing the lowest-entropy cell, or
 *       popping one cell off the propagation queue and tightening its
 *       neighbours. You watch the constraint wave ripple outward in yellow,
 *       collapses flash red, and an entropy heatmap shows how many tile
 *       options each cell still has.
 *
 * Study alongside: ../../matrix_rain/matrix_rain.c (cell-space sim reference)
 *
 * Section map:
 *   §1 config   — grid size, tile count, glow rates, colors
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses pairs (entropy gradient + flash colors)
 *   §5 wfc      — Tile table, compat table, Grid, collapse + propagate
 *   §6 scene    — one operation per tick; glow decay; entropy stats
 *   §7 screen   — UTF-8 box-drawing render via mvaddstr
 *   §8 app      — signals, resize, main loop
 *
 * (No §4 — this is a cell-space simulation. Position IS cell index.)
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset to full superposition + new RNG seed
 *   s          single step (one op, regardless of pause state)
 *   a          toggle auto / step mode
 *   e          toggle entropy heatmap on/off
 *   + / =      more ops per tick (faster)
 *   -          fewer ops per tick (slower)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wfc_learn.c -o wfc_learn -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wave Function Collapse (tile-based variant). Each cell
 *                  starts as a "superposition" — a bitmask of every tile
 *                  it could still be. Repeatedly: (1) pick the cell with
 *                  the fewest remaining options, (2) collapse it to a
 *                  single random tile, (3) propagate the constraint
 *                  outward — for every neighbour, eliminate any tile
 *                  whose connecting edge is now incompatible. Stop when
 *                  every cell has exactly one option.
 *
 *                  This is the simpler "tile + adjacency" WFC, NOT the
 *                  overlapping-pattern WFC from Maxim Gumin's original
 *                  paper. Adjacency is hand-authored: a tile's east edge
 *                  must match its eastern neighbour's west edge (both
 *                  connect, or neither does). With pipe tiles this gives
 *                  a guaranteed-connected network.
 *
 * Data-structure : Each cell is one uint16_t bitmask — bit i set means
 *                  "tile i is still possible here". popcount(mask) is the
 *                  cell's entropy. A flat queue<cell-id> drives propagation
 *                  (BFS-style); we precompute compat[tile][dir] = bitmask
 *                  of tiles that can sit on the d-side of `tile`, so the
 *                  inner propagation step is just a few OR/AND ops.
 *
 * Rendering      : Collapsed cells render as their UTF-8 box-drawing glyph
 *                  in cyan. Uncollapsed cells render as a digit (1–9, A–C)
 *                  showing remaining options, coloured along an entropy
 *                  gradient (high=dim blue → low=bright orange). Two glow
 *                  buffers (float per cell, decay each frame): collapse_glow
 *                  paints fresh collapses red; prop_glow paints the
 *                  propagation wavefront yellow. The two glows decay
 *                  exponentially so the wave is visible without leaving
 *                  permanent stains.
 *
 * Performance    : O(N) per propagation pop (N = tiles per cell = 12).
 *                  Whole-grid collapse for a 50×20 grid is ≈3 000 ops,
 *                  fast enough that we deliberately throttle to 1–8 ops
 *                  per tick (user-tunable) so the human can FOLLOW the
 *                  algorithm. No dynamic allocation after init.
 *
 * References     : • Gumin 2016 — "WaveFunctionCollapse" GitHub repo and
 *                    accompanying README (the canonical reference):
 *                    https://github.com/mxgmn/WaveFunctionCollapse
 *                  • Karth & Smith 2017 — "WaveFunctionCollapse is
 *                    Constraint Solving in the Wild" (academic framing):
 *                    proceedings of FDG 2017.
 *                  • Robert Heaton — "WaveFunctionCollapse explained":
 *                    https://robertheaton.com/2018/12/17/wavefunction-collapse-algorithm/
 *                  • Boris the Brave — "WFC tile-based tutorial":
 *                    https://www.boristhebrave.com/2020/04/13/wave-function-collapse-explained/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell on the grid is in a "superposition" — a set of every tile it
 * could still be — and the algorithm shrinks those sets one cell at a
 * time until every cell is forced into a single tile. Two operations
 * alternate: COLLAPSE picks the most-constrained uncollapsed cell and
 * locks it to one random allowed tile; PROPAGATE walks outward from that
 * change, deleting any tile from a neighbour's set whose connecting edge
 * no longer matches. Repeat until every set has size 1.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine 12 transparent jigsaw pieces stacked on every cell. Where a
 * cell is "undecided", you see all 12 stacked together. When a piece's
 * edge can no longer connect to a neighbour, you slide that piece OUT
 * of the stack on that cell. When a stack is reduced to a single piece,
 * the cell is decided. The algorithm never adds pieces back — it only
 * removes them. This monotone shrinking is what makes WFC terminate.
 *
 * ALGORITHM IN STEPS  (one outer iteration = one collapse + propagation)
 * ────────────────────────────────────────────────────────────────────
 *  1. ENTROPY SCAN. Walk every cell. The "entropy" of a cell is the
 *     number of tiles still possible there (popcount of its bitmask).
 *     Find the cell with the smallest entropy > 1, breaking ties at
 *     random. (Cells with entropy 1 are already decided; skip.)
 *  2. COLLAPSE. Pick one tile uniformly from that cell's set; set the
 *     cell's mask to just that one bit. Push the cell onto a queue.
 *  3. PROPAGATE. Pop a cell from the queue. For each of its four
 *     neighbours, compute "given this cell's possibilities, which tiles
 *     can still sit beside me?" — the union of compat[t][dir] for every
 *     tile t still possible here. Intersect with the neighbour's mask.
 *     If the neighbour shrank, push IT onto the queue.
 *  4. Repeat step 3 until the queue is empty.
 *  5. Goto step 1. Stop when every cell has entropy 1 (DONE), or when
 *     a propagation eliminates a neighbour's last option (CONTRADICTION).
 *
 * KEY FORMULAS
 * ────────────
 *  Entropy of a cell             : popcount(mask)
 *  Edges-match adjacency rule    : tile_a.edge[d] == tile_b.edge[opposite(d)]
 *  opposite(d)                   : (d + 2) mod 4              (N↔S, E↔W)
 *  Compat table  (precomputed)   : compat[t][d] = ⋃ {1<<t' : edges match}
 *  Allowed-in-direction (per cell) :
 *      allowed[d] = ⋃ over t ∈ mask of compat[t][d]
 *  Propagation update            : neighbour.mask = neighbour.mask ∧ allowed[d]
 *  Cell index in flat array      : idx = y * grid_w + x
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CONTRADICTION. When a neighbour's mask becomes 0, no tile fits.
 *    Rare with this 12-tile alphabet (every edge state is reachable
 *    from at least one tile) but possible at corners with restrictive
 *    early collapses. Flag and require user reset rather than backtrack.
 *
 *  • QUEUE OVERFLOW. Without an "already in queue" guard, a single cell
 *    can be enqueued up to N_TILES-1 times (once per shrink), blowing
 *    a queue sized to total_cells. Use the in_queue[] flag — at most
 *    one entry per cell, and that's enough because the cell's mask
 *    always reflects its latest state.
 *
 *  • TIE-BREAKING IN ENTROPY SCAN. If you pick the FIRST minimum found,
 *    the algorithm collapses in row-major order and the result looks
 *    biased toward the top-left corner. Reservoir sampling among ties
 *    fixes this — output looks naturally distributed.
 *
 *  • DOUBLE-COUNTING COLLAPSED CELLS. A cell can collapse two ways:
 *    explicitly (chosen by the entropy scan) or implicitly (propagation
 *    drove its mask down to 1). Count the second only when popcount
 *    transitions from >1 to ==1, never when a 1-mask cell is re-popped
 *    from the queue.
 *
 *  • MIN-ENTROPY HEURISTIC. Picking randomly instead of min-entropy
 *    "works" but produces 5–10× more contradictions on this tile set.
 *    Always collapse the most-constrained cell first.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial state: every cell shows 'C' (12 = entropy 12) — the full
 *    superposition. If any cell starts smaller, ALL_TILES_MASK is wrong.
 *  • After one collapse + full propagation: collapsed_count should be
 *    1 + (number of cells whose entropy dropped to exactly 1 by cascade).
 *    Check the HUD counter increases monotonically, never resets.
 *  • Final state: every cell shows a pipe glyph (' ', '-', '|', or '+');
 *    no cell shows a digit or '!'. If '!' appears, propagation hit a
 *    contradiction.
 *  • Connectedness: every line segment must connect — pick any pipe and
 *    trace its connections; you should never end on a tile whose edge
 *    points into a blank or into a non-matching neighbour. (This is
 *    GUARANTEED by the adjacency rule; if violated, compat[] is buggy.)
 *  • Quick compat sanity: tile 0 (blank) does not connect on any side.
 *    Its north neighbour must therefore present a non-connecting south
 *    edge — i.e. compat[0][N] equals the bitmask of tiles whose edge[S]
 *    is 0. From our table that's tiles {0, 1, 5, 6, 10} — none of
 *    which protrude downward into the blank.
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

/*
 * All magic numbers live here. The grid dimensions are the upper bound;
 * actual usage is min(GRID_*, terminal-size-minus-HUD).
 */
enum {
    GRID_W_MAX        = 200,    /* max grid columns (cell-space) */
    GRID_H_MAX        =  80,    /* max grid rows                 */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,    /* one WFC operation per tick */
    OPS_PER_TICK_DEF  =   2,
    OPS_PER_TICK_MAX  =  64,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    N_TILES           =  12,    /* size of tile alphabet — see §5 tiles[] */

    /* Color pair indices — defined in §3 color_init() */
    PAIR_TILE         =   1,    /* collapsed cell glyph (cyan)        */
    PAIR_FLASH        =   2,    /* collapse_glow (red)                */
    PAIR_WAVE         =   3,    /* prop_glow (yellow)                 */
    PAIR_HUD          =   4,    /* HUD status — reserved bright yellow (226) */
    PAIR_HINT         =   5,    /* bottom hint strip — reserved bright cyan (51) */
    PAIR_BORDER       =   6,    /* grid border (dim white)            */
    PAIR_ENT_HI       =   7,    /* entropy: high (lots of options)    */
    PAIR_ENT_MID      =   8,    /* entropy: medium                    */
    PAIR_ENT_LO       =   9,    /* entropy: low (close to collapse)   */
};

/* Glow decay: glow *= exp(-GLOW_DECAY_RATE * dt) each frame.
 * Rate 4.0 → glow falls to ~e^-1 ≈ 37% in 0.25 s, ~2% in 1 s.
 * Tuned so the propagation wave is clearly visible at default speed
 * without leaving stains that confuse the next collapse. */
#define GLOW_DECAY_RATE      4.0f
#define GLOW_FLASH_THRESHOLD 0.05f   /* below this we draw plain colour */

/* All bits set for a tile bitmask — initial superposition value.
 * (1 << 12) - 1 = 0x0FFF. */
#define ALL_TILES_MASK ((1u << N_TILES) - 1u)

/* Timing primitives. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 * Same canonical helper as framework.c. CLOCK_MONOTONIC never goes
 * backwards (NTP-safe), unlike CLOCK_REALTIME.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep ns nanoseconds, skip if budget already spent.
 * Called BEFORE render so the budget covers physics only, not I/O.
 */
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
 * color_init() — define color pairs used by the renderer.
 *
 * The entropy gradient (PAIR_ENT_HI → MID → LO) deliberately runs from
 * a calm cool blue at high entropy to a hot orange at low entropy: the
 * cell is "heating up" as its options shrink toward 1. This is the
 * standard simulated-annealing / Boltzmann visual metaphor.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_TILE,    87,  -1);   /* light cyan (distinct from HINT) */
        init_pair(PAIR_FLASH,  196,  -1);   /* hot red            */
        init_pair(PAIR_WAVE,   220,  -1);   /* gold (distinct from HUD)        */
        init_pair(PAIR_HUD,    226,  -1);   /* bright yellow — reserved        */
        init_pair(PAIR_HINT,    51,  -1);   /* bright cyan   — reserved        */
        init_pair(PAIR_BORDER, 240,  -1);   /* mid grey           */
        init_pair(PAIR_ENT_HI,  27,  -1);   /* deep blue (cool)   */
        init_pair(PAIR_ENT_MID, 99,  -1);   /* purple             */
        init_pair(PAIR_ENT_LO, 208,  -1);   /* orange (hot)       */
    } else {
        init_pair(PAIR_TILE,    COLOR_WHITE,   -1);
        init_pair(PAIR_FLASH,   COLOR_RED,     -1);
        init_pair(PAIR_WAVE,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HUD,     COLOR_YELLOW,  -1);   /* reserved */
        init_pair(PAIR_HINT,    COLOR_CYAN,    -1);   /* reserved */
        init_pair(PAIR_BORDER,  COLOR_WHITE,   -1);
        init_pair(PAIR_ENT_HI,  COLOR_BLUE,    -1);
        init_pair(PAIR_ENT_MID, COLOR_MAGENTA, -1);
        init_pair(PAIR_ENT_LO,  COLOR_GREEN,   -1);
    }
}

/* ===================================================================== */
/* §5  wfc — the Wave Function Collapse algorithm                         */
/* ===================================================================== */

/*
 * Direction encoding. Order is critical for opposite()/compat[] below.
 * Cell at (x,y); neighbour in direction d sits at:
 *   N → (x  , y-1)
 *   E → (x+1, y  )
 *   S → (x  , y+1)
 *   W → (x-1, y  )
 * opposite(d) = (d + 2) % 4   →   N↔S, E↔W.
 */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/*
 * Tile — one entry in our 12-element pipe alphabet.
 *
 * `glyph` is a NUL-terminated UTF-8 string; we render via mvaddstr because
 * box-drawing characters (U+2500 block) are multi-byte. ASCII fallbacks
 * are NOT provided — if the user's terminal can't display UTF-8 the
 * lines look wrong but the algorithm still runs.
 *
 * `edge[d]` = 1 if this tile connects on side d. Two tiles match across
 * direction d iff  a.edge[d] == b.edge[opposite(d)].
 *
 * The 12 tiles cover every edge-pattern WHERE the connection count is
 * 0, 2, 3, or 4 — i.e. no single-stub "dead ends". This guarantees a
 * fully connected pipe network with no dangling lines.
 */
typedef struct {
    const char *glyph;
    uint8_t     edge[N_DIRS];   /* [N, E, S, W] connect flags */
} Tile;

/* ASCII-only glyphs — chosen for terminal portability (no UTF-8/locale
 * dependency). All 9 junction types render as '+'; straights are '-' and
 * '|'; blank is space. The connectivity is still fully tracked in the
 * edge[] flags, so the algorithm distinguishes T-down from T-up etc.
 * even though they look identical. The visible result is the classic
 * ASCII-pipes aesthetic. */
static const Tile tiles[N_TILES] = {
    /*  0 */ { " ",  {0, 0, 0, 0} },   /* blank — no connections */
    /*  1 */ { "-",  {0, 1, 0, 1} },   /* horizontal */
    /*  2 */ { "|",  {1, 0, 1, 0} },   /* vertical   */
    /*  3 */ { "+",  {0, 1, 1, 0} },   /* corner: down + right */
    /*  4 */ { "+",  {0, 0, 1, 1} },   /* corner: down + left  */
    /*  5 */ { "+",  {1, 1, 0, 0} },   /* corner: up   + right */
    /*  6 */ { "+",  {1, 0, 0, 1} },   /* corner: up   + left  */
    /*  7 */ { "+",  {1, 1, 1, 0} },   /* T: no west  */
    /*  8 */ { "+",  {1, 0, 1, 1} },   /* T: no east  */
    /*  9 */ { "+",  {0, 1, 1, 1} },   /* T: no north */
    /* 10 */ { "+",  {1, 1, 0, 1} },   /* T: no south */
    /* 11 */ { "+",  {1, 1, 1, 1} },   /* cross — all four */
};

/*
 * compat[t][d] = bitmask of tiles that can be placed in direction d
 *                from a cell containing tile t.
 *
 * Built once at startup. Inner propagation step then becomes:
 *
 *   allowed_in_dir = OR over t in cell.mask of compat[t][d]
 *   neighbour.mask &= allowed_in_dir
 *
 * Two passes through 12*12*4 = 576 elements at startup. Trivial.
 */
static uint16_t compat[N_TILES][N_DIRS];

static void wfc_build_compat(void)
{
    for (int t = 0; t < N_TILES; t++) {
        for (int d = 0; d < N_DIRS; d++) {
            uint16_t mask = 0;
            int od = opposite(d);
            for (int t2 = 0; t2 < N_TILES; t2++) {
                /* Compatible iff our edge on side d matches their edge
                 * on the OPPOSITE side. (We want to put t2 in their cell;
                 * their face that touches us is their `od` side.) */
                if (tiles[t].edge[d] == tiles[t2].edge[od])
                    mask |= (uint16_t)(1u << t2);
            }
            compat[t][d] = mask;
        }
    }
}

/*
 * popcount16 — number of bits set in a 16-bit mask.
 *
 * GCC's __builtin_popcount is fine on x86 but we keep this portable
 * and explicit for the learner. The algorithm is "Kernighan's bit
 * counter": each iteration `m & (m-1)` clears the lowest set bit.
 */
static inline int popcount16(uint16_t m)
{
    int c = 0;
    while (m) { m &= (uint16_t)(m - 1); c++; }
    return c;
}

/*
 * Grid — the simulation heart.
 *
 * Stored as 1-D arrays addressed by  idx = y*w + x.  Two pieces of
 * persistent state per cell:
 *   mask          — uint16_t bitmask of remaining tile options
 *   collapse_glow — float [0..1] painting the cell red, decays each frame
 *   prop_glow     — float [0..1] painting the cell yellow, decays
 *
 * Plus a propagation queue: cell IDs scheduled for outward update.
 * A queued cell may already have been processed since enqueue (its
 * neighbours might re-enqueue it); we re-process and that's harmless,
 * because tightening a mask is idempotent.
 *
 * `done` becomes true once every cell has popcount=1.
 * `bad`  becomes true if any cell mask is 0 (contradiction — no tile
 *        can fit). We surface this in the HUD and refuse further ops
 *        until reset. Contradictions are RARE with this tile-set since
 *        the 12 tiles together span every edge state, but a corner
 *        case (literally — corner cells with restricted neighbours)
 *        can still deadlock occasionally.
 */
typedef struct {
    int      w, h;
    uint16_t mask        [GRID_W_MAX * GRID_H_MAX];
    float    collapse_glow[GRID_W_MAX * GRID_H_MAX];
    float    prop_glow    [GRID_W_MAX * GRID_H_MAX];

    int      queue[GRID_W_MAX * GRID_H_MAX];
    bool     in_queue[GRID_W_MAX * GRID_H_MAX];
    int      qhead, qtail;       /* simple linear queue, never wraps     */

    bool     done;
    bool     bad;

    /* Stats for HUD — refreshed lazily by callers. */
    int      collapsed_count;    /* cells with popcount==1               */
    int      total_cells;        /* w*h                                  */
} Grid;

/*
 * grid_reset() — back to full superposition + empty queue.
 * Called on startup, on 'r', and after a contradiction.
 */
static void grid_reset(Grid *g, int w, int h)
{
    g->w = w;
    g->h = h;
    g->total_cells = w * h;
    g->collapsed_count = 0;
    g->done = false;
    g->bad  = false;
    g->qhead = 0;
    g->qtail = 0;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        g->mask[i]          = ALL_TILES_MASK;
        g->collapse_glow[i] = 0.0f;
        g->prop_glow[i]     = 0.0f;
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
    /* in_queue flag — at most one queue entry per cell. The cell's
     * `mask` already holds the most-shrunk state, so a single pending
     * pop is enough to propagate every shrink that happened since the
     * last pop. Without this flag the queue could grow to O(N·N_TILES)
     * and overflow; with it, the queue is bounded by total_cells. */
    if (g->in_queue[idx]) return;
    if (g->qtail >= g->total_cells) return;        /* defensive */
    g->queue[g->qtail++] = idx;
    g->in_queue[idx] = true;
}

/*
 * grid_pick_min_entropy() — find the uncollapsed cell with the fewest
 * remaining options, breaking ties uniformly at random.
 *
 * Why min-entropy? It's a heuristic from the original WFC paper:
 * collapsing the most-constrained cell first minimises the chance of
 * later contradictions. (Picking randomly works too but causes 5–10×
 * more dead-ends.) "Entropy" here is just popcount — for tile-based
 * WFC the proper Shannon entropy reduces to popcount when tile weights
 * are equal, so this is the standard simplification.
 *
 * Returns -1 if no uncollapsed cell exists (we're done).
 */
static int grid_pick_min_entropy(const Grid *g)
{
    int best_n   = N_TILES + 1;
    int tied     = 0;
    int chosen   = -1;
    int n_cells  = g->total_cells;

    for (int i = 0; i < n_cells; i++) {
        int p = popcount16(g->mask[i]);
        if (p <= 1) continue;            /* skip collapsed and contradicted */
        if (p < best_n) {
            best_n = p;
            chosen = i;
            tied   = 1;
        } else if (p == best_n) {
            /* Reservoir sampling: keep `chosen` with probability 1/(tied+1). */
            tied++;
            if ((rand() % tied) == 0) chosen = i;
        }
    }
    return chosen;
}

/*
 * grid_collapse_cell() — collapse one cell to a single random tile.
 *
 * Uniform pick among set bits in mask. (Real WFC weights tiles by
 * frequency from a sample image; we don't, because every pipe tile
 * should appear equally often for a clean visual.)
 *
 * After collapse:
 *   • mask = single bit
 *   • collapse_glow flashed to 1.0
 *   • cell pushed onto propagation queue so neighbours get tightened
 */
static void grid_collapse_cell(Grid *g, int idx)
{
    uint16_t m = g->mask[idx];
    int n = popcount16(m);
    if (n <= 1) return;

    int pick = rand() % n;

    /* Walk the bits to find the pick'th one. */
    int chosen_tile = -1;
    for (int t = 0; t < N_TILES; t++) {
        if (m & (1u << t)) {
            if (pick == 0) { chosen_tile = t; break; }
            pick--;
        }
    }
    g->mask[idx] = (uint16_t)(1u << chosen_tile);
    g->collapse_glow[idx] = 1.0f;
    g->collapsed_count++;
    grid_enqueue(g, idx);
}

/*
 * grid_propagate_one() — pop ONE cell from the queue and tighten its
 * four neighbours.
 *
 * For each direction d:
 *   1. allowed = OR over set bits t of compat[t][d]
 *      = "tiles a neighbour in direction d could possibly be"
 *   2. new = neighbour.mask & allowed
 *      = intersect with what the neighbour ALREADY allowed
 *   3. if changed: write back, paint prop_glow, push neighbour
 *      onto the queue so the constraint propagates further
 *
 * We process exactly ONE pop per call. The caller can loop to drain
 * faster; this granularity is what makes the wavefront visible.
 *
 * Returns true if any work happened (queue had something).
 */
static bool grid_propagate_one(Grid *g)
{
    if (g->qhead >= g->qtail) return false;

    int idx = g->queue[g->qhead++];
    g->in_queue[idx] = false;
    int x = idx % g->w;
    int y = idx / g->w;
    uint16_t my_mask = g->mask[idx];

    /* Pre-compute "what can my d-neighbour be?" for each direction. */
    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!grid_in_bounds(g, nx, ny)) continue;
        int nidx = grid_idx(g, nx, ny);

        uint16_t allowed = 0;
        uint16_t m = my_mask;
        while (m) {
            int t = __builtin_ctz(m);   /* index of lowest set bit */
            allowed |= compat[t][d];
            m &= (uint16_t)(m - 1);
        }

        uint16_t before = g->mask[nidx];
        uint16_t after  = before & allowed;
        if (after != before) {
            g->mask[nidx] = after;
            g->prop_glow[nidx] = 1.0f;
            if (after == 0) {
                /* Contradiction — no tile fits. Algorithm has hit a
                 * dead end. We flag it; the app loop will pause input
                 * and the user can press 'r' to start over. */
                g->bad = true;
            } else if (popcount16(after) == 1 && popcount16(before) > 1) {
                /* Neighbour "collapsed itself" by elimination. Count it
                 * but don't flash the red collapse glow — that's reserved
                 * for the deliberate min-entropy pick, so the user can
                 * tell intentional collapses from cascade collapses. */
                g->collapsed_count++;
            }
            grid_enqueue(g, nidx);
        }
    }
    return true;
}

/*
 * grid_step() — perform exactly ONE WFC operation.
 *
 * Priorities (in this order):
 *   1. If propagation queue has work, do one pop. (Wave keeps moving.)
 *   2. Else, find the lowest-entropy uncollapsed cell and collapse it.
 *   3. Else, mark `done`.
 *
 * Returns true if we did work, false if nothing to do (done or bad).
 */
static bool grid_step(Grid *g)
{
    if (g->bad || g->done) return false;
    if (grid_propagate_one(g)) return true;

    int idx = grid_pick_min_entropy(g);
    if (idx < 0) { g->done = true; return false; }
    grid_collapse_cell(g, idx);
    return true;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene wraps the Grid plus user-facing modes:
 *
 *   auto         — true: keep stepping every tick. false: only on 's'.
 *   paused       — frozen if true; 's' still single-steps.
 *   show_entropy — true: uncollapsed cells render as digits.
 *                  false: uncollapsed cells render as blank space.
 *   ops_per_tick — how many grid_step() calls per scene_tick.
 *                  Higher = faster but the wave blurs together.
 *   step_request — set true by the 's' key; consumed by next tick.
 */
typedef struct {
    Grid   g;
    bool   auto_run;
    bool   paused;
    bool   show_entropy;
    int    ops_per_tick;
    bool   step_request;

    /* Cumulative counters for HUD — reset on grid_reset. */
    long   total_collapses;
    long   total_propagations;
} Scene;

static void scene_reset(Scene *s, int gw, int gh)
{
    grid_reset(&s->g, gw, gh);
    s->total_collapses    = 0;
    s->total_propagations = 0;
}

static void scene_init(Scene *s, int gw, int gh)
{
    memset(s, 0, sizeof *s);
    s->auto_run     = true;
    s->paused       = false;
    s->show_entropy = true;
    s->ops_per_tick = OPS_PER_TICK_DEF;
    scene_reset(s, gw, gh);
}

/*
 * scene_tick() — advance the simulation by one fixed dt.
 *
 * Loops over (paused ? 0 : ops_per_tick) WFC operations, plus one
 * extra if the user pressed 's' for a single step.
 *
 * Glow buffers decay BEFORE we run new ops: that way an op fired this
 * tick lands on a freshly-decayed buffer and shows up at full intensity
 * for one frame, the way a flash should look.
 */
static void scene_tick(Scene *s, float dt)
{
    /* Decay glows. exp(-rate*dt) is the textbook RC-circuit decay; we
     * just compute it directly because dt is small and bounded. */
    float decay = expf(-GLOW_DECAY_RATE * dt);
    int n = s->g.total_cells;
    for (int i = 0; i < n; i++) {
        s->g.collapse_glow[i] *= decay;
        s->g.prop_glow[i]     *= decay;
    }

    int ops = 0;
    if (!s->paused && s->auto_run) ops = s->ops_per_tick;
    if (s->step_request)           { ops += 1; s->step_request = false; }

    for (int i = 0; i < ops; i++) {
        bool was_propagating = (s->g.qhead < s->g.qtail);
        if (!grid_step(&s->g)) break;
        if (was_propagating) s->total_propagations++;
        else                 s->total_collapses++;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Same canonical pattern as framework.c §7:
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate
 *
 * The grid is drawn with a 1-cell margin from the edges, framed by a
 * dim border, with the HUD on top and key hints at the bottom.
 *
 * UTF-8 rendering: the box-drawing tile glyphs are multi-byte UTF-8
 * strings. We use mvaddstr(y, x, glyph) (not mvaddch — that takes a
 * single 8-bit chtype). setlocale(LC_ALL,"") in screen_init activates
 * the user's UTF-8 locale so the bytes pass through unmangled.
 */
typedef struct {
    int cols, rows;
} Screen;

static void screen_init(Screen *s)
{
    /* CRITICAL — without this, mvaddstr emits raw bytes the terminal
     * may interpret as Latin-1 and you get "â” â" garbage. Activates
     * whatever LC_ALL/LANG the user has set (typically en_US.UTF-8). */
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

/*
 * entropy_glyph() — what character to print for an uncollapsed cell?
 *
 * popcount=1  → handled by caller (draw the actual tile glyph)
 * popcount=2..9   → digit '2'..'9'
 * popcount=10..12 → letter 'A'..'C'
 * popcount=0  → '!' (contradiction marker)
 *
 * Single ASCII char so we can draw with mvaddch without UTF-8 fuss.
 */
static char entropy_glyph(int p)
{
    if (p == 0) return '!';
    if (p <= 9) return (char)('0' + p);
    return (char)('A' + (p - 10));
}

/*
 * entropy_color_pair() — colour ramp for the entropy heatmap.
 *
 * High popcount (lots of choices) feels "cool" — calm blue.
 * Low popcount (close to collapse) feels "hot" — orange.
 * 1/3 splits at p=4 and p=8 for our 12-tile alphabet.
 */
static int entropy_color_pair(int p)
{
    if (p >= 8) return PAIR_ENT_HI;
    if (p >= 4) return PAIR_ENT_MID;
    return PAIR_ENT_LO;
}

/*
 * scene_draw() — render the grid + glows + tiles into stdscr.
 *
 * The grid is centered with a dim ASCII border (+ - |). For each cell:
 *
 *   if collapse_glow > threshold    → bright red, draw entropy digit
 *                                     (or the tile glyph if collapsed)
 *   else if prop_glow > threshold   → bright yellow, same
 *   else if collapsed (popcount=1)  → cyan, tile glyph
 *   else if show_entropy            → entropy-graded color, digit
 *   else                            → space
 *
 * Note we deliberately DRAW the new glyph even for the propagation
 * wave so the user sees the option count change in real time.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid *g = &s->g;

    /* Center the grid; account for 1-cell border on each side and HUD. */
    int gx0 = (cols - g->w) / 2;
    int gy0 = (rows - g->h) / 2;
    if (gx0 < 1) gx0 = 1;
    if (gy0 < 1) gy0 = 1;

    /* ── border ─────────────────────────────────────────────────────── */
    attron(COLOR_PAIR(PAIR_BORDER) | A_DIM);
    for (int x = -1; x <= g->w; x++) {
        if (gx0 + x >= 0 && gx0 + x < cols) {
            if (gy0 - 1 >= 0)         mvaddch(gy0 - 1,    gx0 + x, '-');
            if (gy0 + g->h < rows)    mvaddch(gy0 + g->h, gx0 + x, '-');
        }
    }
    for (int y = 0; y < g->h; y++) {
        if (gy0 + y >= 0 && gy0 + y < rows) {
            if (gx0 - 1 >= 0)         mvaddch(gy0 + y, gx0 - 1, '|');
            if (gx0 + g->w < cols)    mvaddch(gy0 + y, gx0 + g->w, '|');
        }
    }
    if (gx0 - 1 >= 0 && gy0 - 1 >= 0)
        mvaddch(gy0 - 1, gx0 - 1, '+');
    if (gx0 + g->w < cols && gy0 - 1 >= 0)
        mvaddch(gy0 - 1, gx0 + g->w, '+');
    if (gx0 - 1 >= 0 && gy0 + g->h < rows)
        mvaddch(gy0 + g->h, gx0 - 1, '+');
    if (gx0 + g->w < cols && gy0 + g->h < rows)
        mvaddch(gy0 + g->h, gx0 + g->w, '+');
    attroff(COLOR_PAIR(PAIR_BORDER) | A_DIM);

    /* ── cells ──────────────────────────────────────────────────────── */
    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int       idx = grid_idx(g, x, y);
            uint16_t  m   = g->mask[idx];
            int       p   = popcount16(m);
            float     cg  = g->collapse_glow[idx];
            float     pg  = g->prop_glow[idx];
            bool      collapsed = (p == 1);

            /* Decide the foreground colour (priority: flash > wave > base). */
            int color_pair;
            int attr = A_NORMAL;
            if (cg > GLOW_FLASH_THRESHOLD) {
                color_pair = PAIR_FLASH;
                attr = A_BOLD;
            } else if (pg > GLOW_FLASH_THRESHOLD) {
                color_pair = PAIR_WAVE;
                attr = A_BOLD;
            } else if (collapsed) {
                color_pair = PAIR_TILE;
            } else {
                color_pair = entropy_color_pair(p);
                attr = A_DIM;
            }

            /* Decide the glyph. */
            attron(COLOR_PAIR(color_pair) | attr);
            if (collapsed) {
                int t = __builtin_ctz(m);
                mvaddstr(sy, sx, tiles[t].glyph);
            } else if (s->show_entropy || cg > GLOW_FLASH_THRESHOLD
                                       || pg > GLOW_FLASH_THRESHOLD) {
                mvaddch(sy, sx, (chtype)(unsigned char)entropy_glyph(p));
            } else {
                mvaddch(sy, sx, ' ');
            }
            attroff(COLOR_PAIR(color_pair) | attr);
        }
    }
}

/*
 * screen_draw() — one full frame: erase, scene, HUD, hint strip.
 * Called once per render. No flush yet — that's screen_present().
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Grid *g = &s->g;
    const char *mode_str =
        g->bad         ? "CONTRADICTION" :
        g->done        ? "DONE         " :
        s->paused      ? "PAUSED       " :
        s->auto_run    ? "AUTO         " :
                         "STEP         ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops/tick:%2d  %s  %4d/%-4d  q:%-3d ",
             fps, sim_fps, s->ops_per_tick, mode_str,
             g->collapsed_count, g->total_cells,
             g->qtail - g->qhead);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Title bar — top-left. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WAVE FUNCTION COLLAPSE — learn ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hint — bottom-left. A_BOLD per HUD standard (CLAUDE.md). */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  s:step  a:auto  e:entropy  +/-:speed  [/]:Hz ");
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

/*
 * app_pick_grid_size() — choose grid dimensions that fit inside the
 * current terminal, with margin for HUD (1 row top, 1 row bottom) and
 * the 1-cell decorative border on every side.
 *
 * Clamped to GRID_W_MAX / GRID_H_MAX so propagation queue can't overflow.
 */
static void app_pick_grid_size(App *app)
{
    int mw = app->screen.cols - 4;           /* 2 margin + 2 border */
    int mh = app->screen.rows - 4;           /* 1 hud + 1 hint + 2 border */
    if (mw < 8)  mw = 8;
    if (mh < 4)  mh = 4;
    if (mw > GRID_W_MAX) mw = GRID_W_MAX;
    if (mh > GRID_H_MAX) mh = GRID_H_MAX;
    app->grid_w = mw;
    app->grid_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_grid_size(app);
    /* On resize, re-spawn the grid so it always fills the screen.
     * Losing the in-progress collapse is acceptable for a teaching demo
     * — and prevents trying to render a 200-wide grid into a 60-col
     * terminal. */
    scene_reset(&app->scene, app->grid_w, app->grid_h);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        scene_reset(s, app->grid_w, app->grid_h);
        break;

    case 's': case 'S':
        s->step_request = true;       /* one op next tick */
        break;

    case 'a': case 'A':
        s->auto_run = !s->auto_run;
        break;

    case 'e': case 'E':
        s->show_entropy = !s->show_entropy;
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

/* ── end §8 — to see this same algorithm scaled up for spectacle,
 *   read wfc_showcase.c (next file in this folder). ── */
