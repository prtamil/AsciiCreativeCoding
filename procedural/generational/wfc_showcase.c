/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wfc_showcase.c — Wave Function Collapse, big and pretty.
 *
 * Fills the whole terminal with a self-assembling maze of ASCII pipes. It
 * starts from five random seeds that grow outward and meet in the middle,
 * sorting themselves into three coloured regions (light / heavy / double),
 * each region kept apart by thin blank gaps. When the grid is full it pauses,
 * then wipes and grows a new one, forever.
 *
 * Sister file: wfc_learn.c is the same algorithm slowed way down with a
 * step-by-step view — read that one first if you want to understand how it
 * works. This file just makes it look good.
 *
 * References (the code can't tell you these):
 *   Gumin (2016), the original Wave Function Collapse:
 *     https://github.com/mxgmn/WaveFunctionCollapse
 *   Merrell (2007), "Example-Based Model Synthesis" — the idea of letting each
 *     tile edge carry a label so only matching edges may touch.
 *   Mackworth (1977), AC-3 arc consistency — the trimming step (see propagate).
 *   xterm 256-colour chart: https://jonasjacek.github.io/colors/
 *
 * Keys: q/ESC quit | space pause | r reset | t/T theme | +/- speed | [/] Hz
 * Build: gcc -std=c11 -O2 -Wall -Wextra wfc_showcase.c -o wfc_showcase -lncurses -lm
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

enum {
    GRID_W_MAX        = 240,
    GRID_H_MAX        =  80,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,        /* how fast the grid fills — full in ~8 s */
    OPS_PER_TICK_MAX  = 512,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    N_TILES           =  34,        /* 1 blank + 11 light + 11 heavy + 11 double */
    N_INITIAL_SEEDS   =   5,        /* how many seeds we drop to start */

    /* ncurses colour slots. HUD and HINT are fixed project-wide; the rest
     * get recoloured by the active theme. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_LIGHT        =   3,
    PAIR_HEAVY        =   4,
    PAIR_DOUBLE       =   5,
    PAIR_WAVE         =   6,
};

/* How fast the yellow growth wave fades. Lower = lingers longer. We keep it
 * slow on purpose so the eye can follow the wavefront across the wide grid. */
#define GLOW_DECAY_RATE      1.5f
#define GLOW_FLASH_THRESHOLD 0.05f   /* below this the wave cell is invisible */

#define HOLD_SECONDS         1.6f   /* how long a finished grid sits before wiping */

/* A 34-bit number with all 34 tile bits set — "every tile is still possible". */
#define ALL_TILES_MASK ((N_TILES >= 64) ? (~(uint64_t)0) \
                                        : (((uint64_t)1 << N_TILES) - 1u))

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define RENDER_CAP_FPS  60          /* never redraw faster than this */
/* If one frame stalls (slow terminal, suspended), pretend no more than this much
 * time passed — otherwise the sim tries to "catch up" with a huge burst. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

/* ── §2 performance — a monotonic clock and a sleep, for capping the frame rate ── */

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

/* ── §3 simulation — the Wave Function Collapse core: tiles, the rules, the grid ── */

/* The four neighbours, in a fixed order: North, East, South, West. */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/*
 * EdgeState — what a tile offers on one of its four sides.
 *
 * Every side of every tile carries one of these labels. The rule for whether
 * two tiles may sit next to each other is dead simple: their touching sides
 * must carry the SAME label.
 *
 *   NONE   — nothing connects here (a flat wall, or just blank space)
 *   LIGHT  — a light-weight pipe sticks out here
 *   HEAVY  — a heavy-weight pipe sticks out here
 *   DOUBLE — a double-weight pipe sticks out here
 *
 * Because matching needs the labels to be equal, a LIGHT side can never touch
 * a HEAVY side. So the three pipe families can only meet where both sides say
 * NONE — and that NONE-meets-NONE seam is the blank gap you see between the
 * coloured regions on screen.
 */
typedef enum {
    EDGE_NONE   = 0,
    EDGE_LIGHT  = 1,
    EDGE_HEAVY  = 2,
    EDGE_DOUBLE = 3,
} EdgeState;

/*
 * Weight — which of the three colour families a tile belongs to.
 *
 * Kept separate from the edge labels on purpose: the edges decide what may
 * connect, this decides what colour to paint. We need a separate colour signal
 * because all three families draw with the very same characters, so colour is
 * the only thing on screen that tells the regions apart.
 *
 *   BLANK is the empty tile — it has no pipes and never gets drawn.
 */
typedef enum {
    WEIGHT_LIGHT  = 0,
    WEIGHT_HEAVY  = 1,
    WEIGHT_DOUBLE = 2,
    WEIGHT_BLANK  = 3,
} Weight;

/*
 * Tile — one shape in the 34-shape alphabet the maze is built from.
 *
 *   glyph  — the single character drawn for it: one of ' ' '-' '|' '+'
 *   edge   — its four side labels in [N, E, S, W] order; these ARE the rules
 *            for what may sit beside it (see EdgeState)
 *   weight — which colour family it belongs to; used only when drawing
 *
 * The alphabet is one blank plus the complete set of 11 pipe shapes, repeated
 * once per colour family (1 + 11*3 = 34). Having the full set means every
 * connection a pipe could need is always available, which keeps the solver from
 * painting itself into avoidable dead ends.
 */
typedef struct {
    const char *glyph;
    EdgeState   edge[N_DIRS];   /* [N, E, S, W] */
    Weight      weight;
} Tile;

/*
 * The 34 tiles. Slot 0 is blank; 1-11 are the light family, 12-22 heavy,
 * 23-33 double. All three families use the exact same characters — the comment
 * glyph beside each (┌ ┐ etc.) is just a hint of the shape, not what's drawn.
 */
#define E_NONE   EDGE_NONE
#define E_L      EDGE_LIGHT
#define E_H      EDGE_HEAVY
#define E_D      EDGE_DOUBLE

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

/* For tile t and direction d, compat[t][d] is the set of tiles that may legally
 * sit on t's d-side (one bit per tile). Worked out once at startup so the hot
 * loop just reads it instead of re-checking edge labels every time. */
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

/*
 * Grid — the whole maze-in-progress and everything needed to fill it in.
 *
 * The key idea: every cell starts out "it could be any tile" and we keep
 * crossing tiles off the list until only one is left. A cell's list of
 * still-possible tiles lives in `mask`, one bit per tile. We only ever remove
 * tiles, never add them back — and that one-way shrinking is exactly why the
 * process always finishes.
 *
 * All the per-cell arrays are flat: cell (x,y) lives at index y*w + x.
 *   mask      — the still-possible tiles for each cell (the maze itself)
 *   prop_glow — pure eye-candy: the yellow growth wave. Set to 1.0 the moment a
 *               cell's list shrinks, fades over time, read only when drawing.
 *               Nothing in the algorithm reads it back.
 *   queue / in_queue / qhead / qtail — the to-do list. When a cell's options
 *               shrink, its neighbours need re-checking, so they go here.
 *               in_queue stops a cell from being queued twice.
 *   done      — true once every cell is down to a single tile
 *   bad       — true if some cell ran out of legal tiles (a contradiction). Can
 *               happen when two differently-coloured growths collide head-on;
 *               we just notice it and start over.
 *   collapsed_count / total_cells — for the on-screen counter
 *   w, h      — grid size in cells
 */
typedef struct {
    int      w, h;
    uint64_t mask         [GRID_W_MAX * GRID_H_MAX];
    float    prop_glow    [GRID_W_MAX * GRID_H_MAX];
    int      queue        [GRID_W_MAX * GRID_H_MAX];
    bool     in_queue     [GRID_W_MAX * GRID_H_MAX];
    int      qhead, qtail;

    bool     done;
    bool     bad;

    int      collapsed_count;
    int      total_cells;
} Grid;

static void grid_reset(Grid *g, int w, int h)
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
        g->mask[i]          = ALL_TILES_MASK;   /* every cell could be anything */
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
    if (g->in_queue[idx]) return;
    if (g->qtail >= g->total_cells) return;
    g->queue[g->qtail++] = idx;
    g->in_queue[idx] = true;
}

/* Walk the set tiles in `mask` and return the n'th one (counting from 0). Lets a
 * random number 0..count-1 pick a tile fairly. */
static int nth_set_tile(uint64_t mask, int n)
{
    while (mask) {
        int t = __builtin_ctzll(mask);
        if (n == 0) return t;
        n--;
        mask &= mask - 1;
    }
    return -1;   /* can't happen as long as n is in range */
}

/* Given the tiles a cell could be, return every tile a neighbour on its d-side
 * is still allowed to be — just the union of each possible tile's rule list.
 * This is the trimming step that makes one choice ripple out to its neighbours. */
static uint64_t allowed_in_dir(uint64_t mask, int d)
{
    uint64_t allowed = 0;
    while (mask) {
        int t = __builtin_ctzll(mask);
        allowed |= compat[t][d];
        mask &= mask - 1;
    }
    return allowed;
}

/*
 * Find the undecided cell with the fewest options left (the most "committed"
 * one) and return its index, or -1 if none are left. On ties we pick one at
 * random so the maze doesn't always grow top-left first.
 */
static int grid_pick_min_entropy(const Grid *g)
{
    int best_n  = N_TILES + 1;
    int tied    = 0;
    int chosen  = -1;
    int n_cells = g->total_cells;

    for (int i = 0; i < n_cells; i++) {
        int entropy = __builtin_popcountll(g->mask[i]);   /* options left */
        if (entropy <= 1) continue;          /* already decided, or a dead cell */
        if (entropy < best_n) {
            best_n = entropy;
            chosen = i;
            tied   = 1;
        } else if (entropy == best_n) {
            tied++;
            if ((rand() % tied) == 0) chosen = i;   /* fair random pick among ties */
        }
    }
    return chosen;
}

/* Commit a cell to a single random tile from those it still allows, then queue it
 * so its neighbours get re-checked. */
static void grid_collapse_cell(Grid *g, int idx)
{
    uint64_t m = g->mask[idx];
    int n = __builtin_popcountll(m);
    if (n <= 1) return;

    int chosen_tile = nth_set_tile(m, rand() % n);

    g->mask[idx] = (uint64_t)1 << chosen_tile;
    g->collapsed_count++;
    grid_enqueue(g, idx);
}

/*
 * Re-check one neighbour now that this cell has changed: drop any of the
 * neighbour's tiles the new rules forbid. If that actually removed something,
 * light up the wave there, note if it just got decided (or ran out of options),
 * and queue it so the change keeps spreading.
 */
static void tighten_neighbour(Grid *g, int nidx, uint64_t my_mask, int d)
{
    uint64_t before = g->mask[nidx];
    uint64_t after  = before & allowed_in_dir(my_mask, d);
    if (after == before) return;            /* nothing changed for this neighbour */

    g->mask[nidx]      = after;
    g->prop_glow[nidx] = 1.0f;              /* light up the growth wave here */
    if (after == 0) {
        g->bad = true;                      /* no tile fits — a contradiction */
    } else if (__builtin_popcountll(after) == 1
            && __builtin_popcountll(before) > 1) {
        g->collapsed_count++;               /* narrowed all the way to one tile */
    }
    grid_enqueue(g, nidx);
}

/* Take one cell off the to-do list and re-check its four neighbours. We do just
 * one per call so the wavefront stays visible as it spreads. Returns false when
 * the to-do list is empty. */
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
        tighten_neighbour(g, grid_idx(g, nx, ny), my_mask, d);
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

/* Pick a random still-undecided cell and lock it to one tile. Called a handful
 * of times at the start to plant the seeds the maze grows from. */
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

/* ── §4 simulation — the scene: one place where everything advances each tick ── */

/*
 * Two phases the demo loops between:
 *   GROWING — the maze is filling in
 *   HOLD    — the maze is full; sit and admire it for a moment before wiping
 * (A contradiction skips HOLD and restarts right away — no point staring at a
 *  grid that got stuck.)
 */
typedef enum { SCENE_GROWING = 0, SCENE_HOLD = 1 } SceneState;

/*
 * Scene — everything about the running demo, grouped by what it's for.
 *   g            — the grid being filled in
 *   state        — GROWING or HOLD
 *   hold_timer   — seconds left to sit on a finished maze
 *   paused       — when true the whole tick is skipped (freezes the picture)
 *   ops_per_tick — how much work to do each tick; the +/- keys change it
 *   theme        — which colour palette is active; the t/T keys change it
 */
typedef struct {
    Grid       g;
    SceneState state;
    float      hold_timer;
    bool       paused;
    int        ops_per_tick;
    int        theme;
} Scene;

/* Wipe the grid clean and plant fresh seeds to grow a new maze. Called at
 * startup, on 'r', on a theme change, when a maze finishes its HOLD, and
 * whenever a growth gets stuck. */
static void scene_reset(Scene *s, int gw, int gh)
{
    grid_reset(&s->g, gw, gh);
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
    scene_reset(s, gw, gh);
}

/* Dim every cell's growth-wave glow a little this tick, so the yellow trail
 * fades out smoothly behind the advancing front. */
static void decay_wave(Grid *g, float dt)
{
    float decay = expf(-GLOW_DECAY_RATE * dt);
    int n = g->total_cells;
    for (int i = 0; i < n; i++) {
        g->prop_glow[i] *= decay;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_wave(&s->g, dt);          /* fade yesterday's wave before drawing today's */

    switch (s->state) {
    case SCENE_GROWING: {
        /* Do a chunk of solving work; bigger ops_per_tick = faster fill. */
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!grid_step(&s->g)) break;
        }
        if (s->g.bad) {
            scene_reset(s, s->g.w, s->g.h);     /* got stuck — start over */
        } else if (s->g.done) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
    } break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->g.w, s->g.h);
        }
        break;
    }
}

/* ── §5 render — palettes, themes, drawing the grid, and the HUD ── */

/*
 * Theme — one named colour scheme for the three pipe families. Colour is the
 * only way the families are told apart on screen (they all draw with the same
 * characters), so a theme just picks three colours and leaves the fixed UI
 * colours alone. Each scheme has a full-colour set and a fallback for terminals
 * that only do 8 colours.
 *   name — shown in the HUD
 *   c    — light / heavy / double, on a 256-colour terminal
 *   c8   — the same three on an 8-colour terminal
 */
typedef struct {
    const char *name;
    short c [3];
    short c8[3];
} Theme;

#define N_THEMES 10

static const Theme THEMES[N_THEMES] = {
    /*  name        light heavy double   8-colour fallback (light/heavy/double)   */
    { "DEFAULT", { 117, 213, 220 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_YELLOW } },
    { "AURORA",  {  51, 207, 154 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_GREEN  } },
    { "EMBER",   { 226, 208, 196 }, { COLOR_YELLOW, COLOR_YELLOW,  COLOR_RED    } },
    { "OCEAN",   { 159,  45,  33 }, { COLOR_CYAN,   COLOR_CYAN,    COLOR_BLUE   } },
    { "FOREST",  { 190, 154,  34 }, { COLOR_GREEN,  COLOR_GREEN,   COLOR_YELLOW } },
    { "NEON",    {  51, 201, 226 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_YELLOW } },
    { "CANDY",   { 159, 219, 213 }, { COLOR_CYAN,   COLOR_MAGENTA, COLOR_MAGENTA} },
    { "ICE",     { 195, 159, 117 }, { COLOR_WHITE,  COLOR_CYAN,    COLOR_CYAN   } },
    { "SUNSET",  { 220, 208, 204 }, { COLOR_YELLOW, COLOR_YELLOW,  COLOR_RED    } },
    { "MONO",    { 252, 246, 255 }, { COLOR_WHITE,  COLOR_WHITE,   COLOR_WHITE  } },
};

/* Switch the three pipe-family colours to the chosen theme. Safe to call while
 * running (the t/T keys do); the fixed UI colours aren't touched. Falls back to
 * the 8-colour set on basic terminals. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &THEMES[idx];
    const short *c = (COLORS >= 256) ? t->c : t->c8;
    init_pair(PAIR_LIGHT,  c[0], -1);
    init_pair(PAIR_HEAVY,  c[1], -1);
    init_pair(PAIR_DOUBLE, c[2], -1);
}

/* Set up colours once: the fixed UI colours (status line, hint line, growth
 * wave), then the default theme's three pipe colours on top. */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_WAVE,       226, -1);
    } else {
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_WAVE,       COLOR_YELLOW,  -1);
    }
    theme_apply(0);   /* start on the default theme, matching Scene.theme = 0 */
}

/* Screen — just the terminal's current size in cells. Kept as its own little
 * type so the drawing code asks for a Screen and nothing more. cols/rows are
 * re-read whenever the window is resized. */
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

/* Which colour slot to use for a pipe family. Blank maps to LIGHT but blank
 * never gets drawn, so the value is just a placeholder. */
static int weight_pair(Weight w)
{
    switch (w) {
    case WEIGHT_LIGHT:  return PAIR_LIGHT;
    case WEIGHT_HEAVY:  return PAIR_HEAVY;
    case WEIGHT_DOUBLE: return PAIR_DOUBLE;
    case WEIGHT_BLANK:  return PAIR_LIGHT;
    }
    return PAIR_LIGHT;
}

/* Draw one cell. A decided cell shows its pipe in the theme colour; an
 * undecided cell that the wave is currently passing through shows a yellow '#';
 * everything else is left blank. So pipes appear in colour as they settle and
 * the yellow front rides along just ahead of them. */
static void draw_cell(const Scene *s, int idx, int sy, int sx)
{
    const Grid *g = &s->g;
    uint64_t m = g->mask[idx];
    bool collapsed = (__builtin_popcountll(m) == 1);

    if (collapsed) {
        int t = __builtin_ctzll(m);
        if (tiles[t].weight == WEIGHT_BLANK) return;     /* the gap tile — draw nothing */
        int color_pair = weight_pair(tiles[t].weight);
        attron(COLOR_PAIR(color_pair) | A_BOLD);
        mvaddstr(sy, sx, tiles[t].glyph);
        attroff(COLOR_PAIR(color_pair) | A_BOLD);
    } else if (g->prop_glow[idx] > GLOW_FLASH_THRESHOLD) {
        attron(COLOR_PAIR(PAIR_WAVE) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)'#');
        attroff(COLOR_PAIR(PAIR_WAVE) | A_BOLD);
    }
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid *g = &s->g;

    /* Centre the grid, keeping one row free at top (status) and bottom (hints). */
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
            draw_cell(s, grid_idx(g, x, y), sy, sx);
        }
    }
}

/* The current state as a same-width word for the status line. */
static const char *mode_label(const Scene *s)
{
    if (s->paused)             return "PAUSED ";
    if (s->state == SCENE_HOLD) return "HOLD   ";
    if (s->g.bad)              return "BAD!   ";
    return                            "GROWING";
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Grid *g = &s->g;
    const char *state_str = mode_label(s);

    /* Top-right status line: fps, sim rate, speed, state, and the cell counter. */
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

    /* Top-left title and the theme that's currently showing. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WFC showcase  theme:%-7s ", THEMES[s->theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key reminder. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t:theme  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6 app — signals, key and resize handling, and the main loop ── */

/*
 * App — the whole running program tied together.
 *   scene             — the simulation
 *   screen            — where it's drawn
 *   sim_fps           — how many sim ticks per second; the [ and ] keys change it
 *   grid_w / grid_h   — grid size, refitted to the window on resize (capped so the
 *                       fixed arrays can't overflow)
 *   running           — cleared by a quit signal to end the loop
 *   need_resize       — set by a window-resize signal; handled at the top of the loop
 * The two flags are touched from signal handlers, so they're sig_atomic_t.
 * There's one global App so the signal handlers can reach it.
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

static void app_pick_grid_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;     /* keep a row for the status and hint lines */
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
    scene_reset(&app->scene, app->grid_w, app->grid_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->grid_w, app->grid_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        scene_reset(s, app->grid_w, app->grid_h);   /* grow a fresh maze in the new colours */
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        scene_reset(s, app->grid_w, app->grid_h);   /* grow a fresh maze in the new colours */
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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* don't let a stalled frame snowball */

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
