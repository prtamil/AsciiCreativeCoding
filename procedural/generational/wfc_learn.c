/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wfc_learn.c — Wave Function Collapse, slowed right down so you can watch it
 * decide. Every cell on the grid starts as "could be any pipe tile"; the
 * program repeatedly picks the most boxed-in cell, locks it to one tile, and
 * lets that choice ripple out and rule tiles out of its neighbours — one tiny
 * step per tick so the wave is visible. The result is a fully-connected ASCII
 * pipe network.
 *
 * Study alongside: matrix_rain.c (another cell-space sim) and wfc_showcase.c
 *   (this same algorithm run fast, for spectacle instead of teaching).
 *
 * This is the simple "tile + adjacency" flavour of WFC, not the
 * overlapping-pattern original. Good starting points:
 *   • Boris the Brave (2020), "WFC tile-based, explained" — the formulation
 *     this file implements.
 *   • Gumin (2016) WaveFunctionCollapse repo — named and popularised it.
 *   • Mackworth (1977), AC-3 arc consistency — the propagate step is exactly
 *     this: shrink a neighbour to edge-compatible tiles, repeat.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wfc_learn.c -o wfc_learn -lncurses -lm
 */

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

/* ── §1 config ── */

/* Every tunable number lives here. The grid sizes are upper bounds; the real
 * grid is whatever fits the terminal once the HUD and border are subtracted. */
enum {
    GRID_W_MAX        = 200,    /* max grid columns (cell-space) */
    GRID_H_MAX        =  80,    /* max grid rows                 */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,    /* solver steps per tick */
    OPS_PER_TICK_DEF  =   2,
    OPS_PER_TICK_MAX  =  64,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    N_TILES           =  12,    /* how many pipe shapes there are — see §3 tiles[] */

    /* Colour pair numbers, set up in §5 color_init(). */
    PAIR_TILE         =   1,    /* a decided cell's pipe (cyan) */
    PAIR_FLASH        =   2,    /* a chosen collapse (red) */
    PAIR_WAVE         =   3,    /* the ripple (yellow) */
    PAIR_HUD          =   4,    /* HUD text — reserved bright yellow (226) */
    PAIR_HINT         =   5,    /* bottom key strip — reserved bright cyan (51) */
    PAIR_BORDER       =   6,    /* the frame (dim white) */
    PAIR_ENT_HI       =   7,    /* heatmap: many options left */
    PAIR_ENT_MID      =   8,    /* heatmap: middling */
    PAIR_ENT_LO       =   9,    /* heatmap: nearly decided */
};

/* How fast a glow (the coloured flash on a cell) fades each frame. At rate 4.0
 * a flash drops to about a third of its brightness in a quarter second — fast
 * enough that the wave reads as motion, slow enough to actually see. */
#define GLOW_DECAY_RATE      4.0f
#define GLOW_FLASH_THRESHOLD 0.05f   /* dimmer than this counts as "no glow" */

/* Starting state for a cell: every tile is still possible. (1<<12)-1 = 0x0FFF. */
#define ALL_TILES_MASK ((1u << N_TILES) - 1u)

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* The screen never redraws faster than this, no matter how high the sim Hz is. */
#define RENDER_CAP_FPS  60
/* If one frame stalls badly (laptop slept, terminal hung), pretend no more than
 * this much time passed — otherwise the catch-up loop would run forever. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

/* ── §2 performance — clock + sleep for the frame cap ── */

/* A steady clock in nanoseconds. MONOTONIC only ever moves forward, so a clock
 * adjustment mid-run can't make our timing jump backwards. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for ns nanoseconds; do nothing if we've already used up the frame. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 simulation — the WFC core: tiles, adjacency table, grid, collapse + propagate ── */

/* The four neighbours of a cell. The order N,E,S,W matters: opposite() relies on
 * "two steps around" landing you on the facing side (N<->S, E<->W). */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/*
 * Tile — one kind of pipe piece. A tile is fully described by which of its four
 * sides have a pipe sticking out, so it needs just two fields:
 *
 *   glyph   : the single character drawn for it (' ', '-', '|', '+'). It's a
 *             string rather than a char so every tile prints the same way via
 *             mvaddstr. All nine junction pieces look identical ('+') on screen
 *             — colour and animation tell them apart, not the glyph (the
 *             project's ASCII-only rule).
 *   edge[d] : 1 if a pipe sticks out on side d, ordered [N, E, S, W]. This is
 *             the whole rule for what can sit next to what: two tiles fit across
 *             a shared edge only when both have a pipe there or both don't — i.e.
 *             a.edge[d] == b.edge[opposite(d)]. wfc_build_compat() bakes these
 *             flags into the fast lookup table the propagator actually uses.
 *
 * Why these exact 12: they cover every piece with 0, 2, 3, or 4 connections —
 * deliberately leaving out the single dead-end stub. No dead-ends is what makes
 * the finished result one fully-connected pipe network with no loose ends.
 */
typedef struct {
    const char *glyph;
    uint8_t     edge[N_DIRS];   /* pipe on side [N, E, S, W]? 1 = yes */
} Tile;

/* The tile alphabet. Many share the '+' glyph but differ in their edges, so the
 * algorithm tells a T-junction from a cross even though they look the same. */
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

/* compat[t][d] answers, in one bitmask: "if a cell holds tile t, which tiles
 * are allowed to sit on its d side?" We build it once so the hot propagation
 * loop is a couple of bit operations instead of re-checking edges every time. */
static uint16_t compat[N_TILES][N_DIRS];

/* Fill compat[][] from the edge flags: tile t2 may sit on tile t's d side
 * exactly when the two faces that touch agree (both have a pipe, or neither). */
static void wfc_build_compat(void)
{
    for (int t = 0; t < N_TILES; t++) {
        for (int d = 0; d < N_DIRS; d++) {
            uint16_t mask = 0;
            int od = opposite(d);
            for (int t2 = 0; t2 < N_TILES; t2++) {
                if (tiles[t].edge[d] == tiles[t2].edge[od])
                    mask |= (uint16_t)(1u << t2);
            }
            compat[t][d] = mask;
        }
    }
}

/* Count the set bits in a mask — i.e. how many tiles a cell still allows.
 * The trick m & (m-1) clears the lowest set bit, so the loop runs once per
 * option rather than 16 times. (This count is what we call a cell's entropy.) */
static inline int popcount16(uint16_t m)
{
    int c = 0;
    while (m) { m &= (uint16_t)(m - 1); c++; }
    return c;
}

/* Find the n'th still-allowed tile (counting from 0). Lets us turn a random
 * number into a fair choice among a cell's remaining options. */
static int nth_set_tile(uint16_t mask, int n)
{
    for (int t = 0; t < N_TILES; t++) {
        if (mask & (1u << t)) {
            if (n == 0) return t;
            n--;
        }
    }
    return -1;   /* can't happen while n < popcount(mask) */
}

/* Given a cell whose options are `mask`, which tiles is a neighbour on side d
 * still allowed to be? Pool together what each option here permits next door. */
static uint16_t allowed_in_dir(uint16_t mask, int d)
{
    uint16_t allowed = 0;
    while (mask) {
        int t = __builtin_ctz(mask);   /* next still-allowed tile here */
        allowed |= compat[t][d];
        mask &= (uint16_t)(mask - 1);
    }
    return allowed;
}

/*
 * Grid — everything the solver works on. All the per-cell arrays are flat and
 * addressed by idx = y*w + x.
 *
 * Per-cell state:
 *   mask          — which tiles this cell could still be, one bit per tile.
 *                   This is the cell's set of options. The algorithm only ever
 *                   clears bits (rules tiles out), never sets them — that
 *                   one-way shrinking is exactly why WFC always finishes.
 *   collapse_glow — 0..1, how red to flash the cell right now (fades each
 *                   frame). Purely cosmetic; never read by the algorithm.
 *   prop_glow     — 0..1, how yellow to flash it — the ripple of the wave.
 *                   Also cosmetic only.
 *
 * The propagation queue (queue / in_queue / qhead / qtail) is the to-do list of
 * cells whose options just shrank and whose neighbours therefore need a recheck.
 * We pop cells off the front and process them until the list empties. A cell can
 * land on the list more than once; reprocessing it is harmless because we always
 * act on its current options. in_queue[] keeps each cell on the list at most
 * once, which also stops the fixed-size queue from ever overflowing.
 *
 *   done — true once every cell is pinned down to a single tile.
 *   bad  — true if some cell ran out of options entirely (a contradiction: no
 *          tile fits there). We show it in the HUD and stop until the user
 *          resets. It's rare with these 12 tiles but can still happen, usually
 *          in a tight corner.
 */
typedef struct {
    int      w, h;
    uint16_t mask        [GRID_W_MAX * GRID_H_MAX];   /* options left, per cell */
    float    collapse_glow[GRID_W_MAX * GRID_H_MAX];  /* red flash, 0..1 */
    float    prop_glow    [GRID_W_MAX * GRID_H_MAX];  /* yellow flash, 0..1 */

    int      queue[GRID_W_MAX * GRID_H_MAX];          /* cells waiting to ripple */
    bool     in_queue[GRID_W_MAX * GRID_H_MAX];       /* already on the queue? */
    int      qhead, qtail;       /* front and back; never wraps around */

    bool     done;               /* every cell pinned to one tile */
    bool     bad;                /* some cell has zero options left */

    int      collapsed_count;    /* cells already pinned to one tile (for the HUD) */
    int      total_cells;        /* w*h */
} Grid;

/* Wipe the grid back to "every cell could be anything" and an empty queue.
 * Used at startup, on 'r', and after a resize. */
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
    /* Skip if it's already queued — one slot per cell is enough, since we read
     * the cell's current options when we get to it, and it keeps the fixed-size
     * queue from overflowing. */
    if (g->in_queue[idx]) return;
    if (g->qtail >= g->total_cells) return;        /* belt and braces */
    g->queue[g->qtail++] = idx;
    g->in_queue[idx] = true;
}

/* Pick the still-undecided cell with the fewest options left, choosing at random
 * among ties. Going after the most boxed-in cell first is the standard WFC move:
 * it's where a wrong guess does the least damage, so it leads to far fewer
 * dead-ends than picking at random. Returns -1 when nothing's left to decide. */
static int grid_pick_min_entropy(const Grid *g)
{
    int best_n   = N_TILES + 1;
    int tied     = 0;
    int chosen   = -1;
    int n_cells  = g->total_cells;

    for (int i = 0; i < n_cells; i++) {
        int entropy = popcount16(g->mask[i]);
        if (entropy <= 1) continue;             /* already decided, or stuck */
        if (entropy < best_n) {
            best_n = entropy;
            chosen = i;
            tied   = 1;
        } else if (entropy == best_n) {
            /* Give every tied cell an equal shot at being the pick, in one pass. */
            tied++;
            if ((rand() % tied) == 0) chosen = i;
        }
    }
    return chosen;
}

/* Commit one cell to a single tile, picked at random from its remaining options.
 * (We weight all tiles equally so every pipe shape shows up about as often.)
 * Flashes the cell red and queues it so its neighbours get rechecked. */
static void grid_collapse_cell(Grid *g, int idx)
{
    uint16_t m = g->mask[idx];
    int n = popcount16(m);
    if (n <= 1) return;

    int chosen_tile = nth_set_tile(m, rand() % n);

    g->mask[idx] = (uint16_t)(1u << chosen_tile);
    g->collapse_glow[idx] = 1.0f;                    /* red flash for a chosen collapse */
    g->collapsed_count++;
    grid_enqueue(g, idx);
}

/* Rule out, from one neighbour, any tile our cell's options forbid across side d.
 * If that actually removed something, flash the ripple, note what happened, and
 * queue the neighbour so its own neighbours get rechecked in turn. */
static void tighten_neighbour(Grid *g, int nidx, uint16_t my_mask, int d)
{
    uint16_t before = g->mask[nidx];
    uint16_t after  = before & allowed_in_dir(my_mask, d);
    if (after == before) return;            /* nothing changed next door */

    g->mask[nidx] = after;
    g->prop_glow[nidx] = 1.0f;              /* yellow flash marks the ripple */

    if (after == 0) {
        /* Out of options — a contradiction. Flag it and the app stops until 'r'. */
        g->bad = true;
    } else if (popcount16(after) == 1 && popcount16(before) > 1) {
        /* This neighbour got narrowed down to one tile on its own. Count it, but
         * don't flash it red — red is reserved for cells we deliberately picked,
         * so the watcher can tell a real choice from a knock-on effect. */
        g->collapsed_count++;
    }
    grid_enqueue(g, nidx);
}

/* Take one cell off the queue and recheck its four neighbours. Doing just one
 * cell per call is what lets you watch the ripple spread. True if there was work. */
static bool grid_propagate_one(Grid *g)
{
    if (g->qhead >= g->qtail) return false;

    int idx = g->queue[g->qhead++];
    g->in_queue[idx] = false;
    int x = idx % g->w;                     /* flat index back to x,y */
    int y = idx / g->w;
    uint16_t my_mask = g->mask[idx];

    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!grid_in_bounds(g, nx, ny)) continue;
        tighten_neighbour(g, grid_idx(g, nx, ny), my_mask, d);
    }
    return true;
}

/* Do one unit of work: finish any pending ripple first, otherwise pick and
 * collapse the next cell; if there's neither, we're done. Returns false when
 * there's nothing left to do (finished or stuck). */
static bool grid_step(Grid *g)
{
    if (g->bad || g->done) return false;
    if (grid_propagate_one(g)) return true;

    int idx = grid_pick_min_entropy(g);
    if (idx < 0) { g->done = true; return false; }
    grid_collapse_cell(g, idx);
    return true;
}

/* ── §4 simulation — the scene and its once-per-tick update ── */

/*
 * Scene — the grid plus the knobs that control how it runs and how it looks.
 *   g              — the grid being solved.
 *   auto_run       — keep stepping on its own, or only when 's' is pressed.
 *   paused         — freeze everything.
 *   ops_per_tick   — how many steps to do per tick; higher is faster but the
 *                    wave blurs together.
 *   step_request   — set by 's', does exactly one extra step on the next tick.
 *   show_entropy   — show the option-count digit on undecided cells, or leave
 *                    them blank.
 *   total_collapses / total_propagations — running tallies for the HUD only;
 *                    they never change what the algorithm does.
 */
typedef struct {
    Grid   g;
    bool   auto_run;
    bool   paused;
    bool   show_entropy;
    int    ops_per_tick;
    bool   step_request;
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

/* Fade every cell's red and yellow flash a little toward zero this tick. */
static void decay_glows(Grid *g, float dt)
{
    float decay = expf(-GLOW_DECAY_RATE * dt);
    int n = g->total_cells;
    for (int i = 0; i < n; i++) {
        g->collapse_glow[i] *= decay;
        g->prop_glow[i]     *= decay;
    }
}

/* Advance the sim one tick: fade old flashes, then do this tick's steps. We fade
 * first so a step that fires this tick lands on a clean cell and shows at full
 * brightness for one frame, the way a flash should. */
static void scene_tick(Scene *s, float dt)
{
    decay_glows(&s->g, dt);

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

/* ── §5 render — colours, the option-count heatmap, the pipe grid, the HUD ── */

/* Set up the colour pairs. The heatmap runs cool-blue (lots of options) to
 * hot-orange (nearly decided), so a cell visibly "heats up" as it closes in. */
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

/* Screen — just the terminal's current size in cells. It's a separate little
 * type so the drawing code can be handed only this and never reach into the
 * whole App, which keeps drawing cleanly apart from the simulation.
 *   cols, rows — terminal width and height, refreshed on resize; used to centre
 *                the grid and place the HUD rows. */
typedef struct {
    int cols, rows;
} Screen;

static void screen_init(Screen *s)
{
    /* Honour the user's locale. Our glyphs are plain ASCII, so this is just
     * insurance in case a future tile alphabet uses multi-byte characters. */
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

/* Show an undecided cell's option count as one character: digits 2..9, then
 * letters A..C for 10..12, and '!' for a cell that's run out of options. */
static char entropy_glyph(int p)
{
    if (p == 0) return '!';
    if (p <= 9) return (char)('0' + p);
    return (char)('A' + (p - 10));
}

/* Heatmap colour for an undecided cell: lots of options = cool blue, few =
 * hot orange, split into thirds across our 12 tiles. */
static int entropy_color_pair(int p)
{
    if (p >= 8) return PAIR_ENT_HI;
    if (p >= 4) return PAIR_ENT_MID;
    return PAIR_ENT_LO;
}

/* Draw a dim +/-/| frame just outside the grid, clipped to the screen edges. */
static void scene_draw_border(int gx0, int gy0, int gw, int gh, int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_BORDER) | A_DIM);
    for (int x = -1; x <= gw; x++) {
        if (gx0 + x >= 0 && gx0 + x < cols) {
            if (gy0 - 1 >= 0)       mvaddch(gy0 - 1,  gx0 + x, '-');
            if (gy0 + gh < rows)    mvaddch(gy0 + gh, gx0 + x, '-');
        }
    }
    for (int y = 0; y < gh; y++) {
        if (gy0 + y >= 0 && gy0 + y < rows) {
            if (gx0 - 1 >= 0)       mvaddch(gy0 + y, gx0 - 1,  '|');
            if (gx0 + gw < cols)    mvaddch(gy0 + y, gx0 + gw, '|');
        }
    }
    if (gx0 - 1 >= 0 && gy0 - 1 >= 0)        mvaddch(gy0 - 1,  gx0 - 1,  '+');
    if (gx0 + gw < cols && gy0 - 1 >= 0)     mvaddch(gy0 - 1,  gx0 + gw, '+');
    if (gx0 - 1 >= 0 && gy0 + gh < rows)     mvaddch(gy0 + gh, gx0 - 1,  '+');
    if (gx0 + gw < cols && gy0 + gh < rows)  mvaddch(gy0 + gh, gx0 + gw, '+');
    attroff(COLOR_PAIR(PAIR_BORDER) | A_DIM);
}

/* Paint one cell. Colour goes to whichever applies first: red flash, then
 * yellow ripple, then a decided tile, then the heatmap. We keep showing the
 * option-count digit even under the ripple so you can watch the count tick down. */
static void draw_cell(const Scene *s, int idx, int sy, int sx)
{
    const Grid *g = &s->g;
    uint16_t  m   = g->mask[idx];
    int       p   = popcount16(m);
    float     cg  = g->collapse_glow[idx];
    float     pg  = g->prop_glow[idx];
    bool      collapsed = (p == 1);

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

/* Draw the whole grid: centre it, frame it, then paint every visible cell. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid *g = &s->g;

    /* Centre it, keeping a cell of room for the border and HUD rows. */
    int gx0 = (cols - g->w) / 2;
    int gy0 = (rows - g->h) / 2;
    if (gx0 < 1) gx0 = 1;
    if (gy0 < 1) gy0 = 1;

    scene_draw_border(gx0, gy0, g->w, g->h, cols, rows);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            draw_cell(s, grid_idx(g, x, y), sy, sx);
        }
    }
}

/* mode_label() — the current run state as a fixed-width HUD label. */
static const char *mode_label(const Scene *s)
{
    if (s->g.bad)    return "CONTRADICTION";
    if (s->g.done)   return "DONE         ";
    if (s->paused)   return "PAUSED       ";
    if (s->auto_run) return "AUTO         ";
    return                  "STEP         ";
}

/* Lay out one full frame: clear, grid, HUD, hint strip. Nothing reaches the
 * terminal yet — screen_present() does the actual flush. */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Grid *g = &s->g;
    const char *mode_str = mode_label(s);

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

/* ── §6 app — signals, key and resize events, the main loop ── */

/*
 * App — the whole running program tied together: the simulation, the terminal
 * it draws to, and the loop state. It lives in one global so the signal handlers
 * can flip its flags.
 *   scene, screen — the simulation and where it's drawn.
 *   sim_fps       — how many ticks per second (the [ and ] keys).
 *   grid_w/grid_h — the grid size in use, re-fit to the terminal on resize and
 *                   capped so the queue can never overflow.
 *   running       — clear this to leave the loop and quit.
 *   need_resize   — set by the window-change signal; handled next loop.
 */
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

/* Size the grid to fill the terminal, minus room for the HUD rows and the
 * border, and capped at the maximums so the fixed queue can't overflow. */
static void app_pick_grid_size(App *app)
{
    int mw = app->screen.cols - 4;           /* a margin and a border each side */
    int mh = app->screen.rows - 4;           /* HUD row, hint row, two borders */
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
    /* Start a fresh grid sized to the new terminal. Throwing away the
     * in-progress solve is fine for a teaching demo, and it avoids trying to
     * cram a huge grid into a small window. */
    scene_reset(&app->scene, app->grid_w, app->grid_h);
    app->need_resize = 0;
}

/* Act on one keypress; return false only when the user asked to quit. */
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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* don't try to catch up forever */

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
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}

/* For this same algorithm run fast for spectacle, see wfc_showcase.c. */
