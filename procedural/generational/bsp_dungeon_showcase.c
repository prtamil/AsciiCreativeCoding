/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bsp_dungeon_showcase.c — watch a roguelike dungeon build itself.
 *
 * Chops the map into rectangles, drops a room in each smallest piece, then
 * walks L-shaped corridors between rooms so the whole thing connects up.
 * The three phases play out on screen; when done, a glowing path blinks
 * between two far-apart rooms until you press n for a fresh map.
 *
 * Sister file: ./maze_backtracker.c — the other classic generator (one
 * twisty corridor instead of many distinct rooms). BSP is the Rogue /
 * NetHack style.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra bsp_dungeon_showcase.c \
 *       -o bsp_dungeon -lncurses -lm
 */

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
    /* Map size in cells. One map cell = one terminal cell. */
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    MIN_MAP_W         =  16,    /* don't bother below this — too cramped  */
    MIN_MAP_H         =   8,
    HUD_ROWS          =   2,    /* top data bar + bottom hint take 2 rows */

    /* A rectangle smaller than this won't be cut any further.
     * Smaller numbers → more, tinier rooms; larger → fewer, bigger ones. */
    MIN_LEAF_W        =  10,
    MIN_LEAF_H        =   6,

    /* Smallest room we'll allow, and how far in from the rectangle's
     * edges a room may sit (random, so rooms float a bit). */
    MIN_ROOM_W        =   4,
    MIN_ROOM_H        =   3,
    MAX_ROOM_MARGIN   =   3,

    /* Hard cap on tree nodes. With these leaf sizes on a 200×56 map the
     * tree never gets close, so 512 is plenty of headroom. */
    MAX_NODES         = 512,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,
    RENDER_FPS        =  60,    /* how often we repaint the screen        */
    DT_CAP_MS         = 100,    /* ignore stalls longer than this so the
                                 * sim can't suddenly fast-forward        */

    /* How much of each phase to reveal per tick. Building goes slow so
     * you can watch each cut; carving and corridors run faster. +/- scales
     * all three. */
    BUILD_STEPS_MIN   =   1,
    BUILD_STEPS_DEF   =   1,
    BUILD_STEPS_MAX   =  64,

    CARVE_STEPS_DEF   =  12,
    CORRIDOR_STEPS_DEF=   8,
    PACE_MAX          = 256,    /* most carve/corridor cells per tick     */

    FPS_UPDATE_MS     = 500,

    N_THEMES          =   5,
    GLOW_TIERS        =   4,    /* steps in a hot→cool fade               */

    /* ncurses colour-pair slots. Each glowing effect gets GLOW_TIERS
     * pairs in a row so a fading cut can step from white-hot down to cool
     * instead of being one flat colour. HUD/HINT are reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,
    PAIR_FLOOR        =   4,
    PAIR_ROOM_0       =   5,    /* +0..+3: a room cell freshly dug        */
    PAIR_COR_0        =   9,    /* +0..+3: a corridor cell freshly dug    */
    PAIR_PART_0       =  13,    /* +0..+3: the cyan partition outlines    */
    PAIR_PATH_0       =  17,    /* +0..+3: the blinking solution path     */
};

/* How fast a fresh glow fades. Cuts fade quickly (snappy); the partition
 * outlines fade slowly so they stay readable while the map is being built. */
#define CARVE_GLOW_DECAY    3.5f
#define PART_GLOW_DECAY     1.5f
#define GLOW_THRESHOLD      0.05f      /* below this a glow counts as gone */

/* When the map is finished: the solution path pulses and the whole dungeon
 * "breathes" gently, like flickering torchlight. */
#define PATH_BLINK_RATE     6.0f       /* how fast the path pulses        */
#define BREATHE_RATE        2.2f       /* how fast the breathe rises/falls */
#define BREATHE_BASE        0.65f      /* breathe rides a sine wave between */
#define BREATHE_AMP         0.35f      /*   BASE-AMP and BASE+AMP          */
#define BREATHE_FLOOR_HI    0.75f      /* above this, floor brightens     */
#define BREATHE_WALL_LO     0.40f      /* below this, walls dim           */

/* A wall is drawn brighter the more floor it touches, faking a depth cue:
 * a wall hugged by a room reads as lit, a lone edge reads as shadowed. */
#define TORCH_LIT_NB        5          /* this many floor neighbours → bold */
#define TORCH_DIM_NB        2          /* fewer than this → dim; none → hidden */
#define FLAGSTONE_MOD       11         /* roughly 1 floor cell in 11 looks brighter */

/* Tile kinds. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELLS_MAX  (MAP_W_MAX * MAP_H_MAX)

/* clamp v to [lo, hi] — keeps range-limits named at the call site. */
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

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
 * Theme — one complete colour scheme, switchable with t/T. It names the
 * settled wall and floor colours, plus four little colour ramps (hot at the
 * front, cool at the back). When a cell is freshly dug it glows; as the glow
 * fades the renderer steps down its ramp, so a cut flashes white-hot and
 * cools to the theme's signature colour.
 *
 * Every colour stays in the bright half of the 256-colour range so even the
 * coolest end of a ramp is still visible against a dark terminal.
 */
typedef struct {
    const char *name;         /* short label shown in the HUD              */
    short wall;               /* settled '#' wall                          */
    short floor;              /* settled '.' floor                         */
    short room[GLOW_TIERS];   /* a room cell, freshly dug → faded          */
    short cor [GLOW_TIERS];   /* a corridor cell, freshly dug → faded      */
    short part[GLOW_TIERS];   /* a partition outline, fresh → faded        */
    short path[GLOW_TIERS];   /* the finished-map solution path            */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "EMBER ", 246, 67,
      {231,218,212,175}, {231,229,220,208}, {231,159,117, 74}, {231,227,214,202} },
    { "ARCANE", 103, 61,
      {231,219,177,134}, {231,159, 87, 45}, {231,189,147, 99}, {231,213,171,129} },
    { "TOXIC ", 101, 65,
      {231,191,154,106}, {231,229,220,178}, {231,158,119, 71}, {231,226,190,154} },
    { "ICE   ", 152, 67,
      {231,195,153,111}, {231,159,123, 81}, {231,195,159,123}, {231,159,123, 75} },
    { "MONO  ", 250, 242,
      {255,252,248,245}, {255,251,247,244}, {255,250,246,243}, {255,253,250,246} },
};

/* load one ramp into its block of consecutive colour pairs. */
static void theme_bind_ramp(int base_pair, const short ramp[GLOW_TIERS])
{
    for (int i = 0; i < GLOW_TIERS; i++)
        init_pair((short)(base_pair + i), ramp[i], -1);
}

static void theme_bind_256(const Theme *t)
{
    init_pair(PAIR_WALL,  t->wall,  -1);
    init_pair(PAIR_FLOOR, t->floor, -1);
    theme_bind_ramp(PAIR_ROOM_0, t->room);
    theme_bind_ramp(PAIR_COR_0,  t->cor);
    theme_bind_ramp(PAIR_PART_0, t->part);
    theme_bind_ramp(PAIR_PATH_0, t->path);
}

/* fallback for old 8-colour terminals: one flat colour per effect. */
static void theme_bind_8(void)
{
    init_pair(PAIR_WALL,  COLOR_WHITE, -1);
    init_pair(PAIR_FLOOR, COLOR_BLUE,  -1);
    for (int i = 0; i < GLOW_TIERS; i++) {
        init_pair((short)(PAIR_ROOM_0 + i), COLOR_MAGENTA, -1);
        init_pair((short)(PAIR_COR_0  + i), COLOR_YELLOW,  -1);
        init_pair((short)(PAIR_PART_0 + i), COLOR_CYAN,    -1);
        init_pair((short)(PAIR_PATH_0 + i), COLOR_YELLOW,  -1);
    }
}

static void theme_apply(int idx)
{
    const Theme *t = &THEMES[idx];
    if (COLORS >= 256) theme_bind_256(t);
    else               theme_bind_8();
}

/* turn a glow strength (1 = fresh, fading toward 0) into a ramp step,
 * where 0 is the hottest colour. */
static int glow_tier(float g)
{
    if (g > 0.66f) return 0;
    if (g > 0.40f) return 1;
    if (g > 0.18f) return 2;
    return GLOW_TIERS - 1;
}

static void color_init(void)
{
    start_color();
    use_default_colors();
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
/* §4  geometry — Point / Rect: the algorithm is rectangle subdivision     */
/* ===================================================================== */

/*
 * The whole algorithm is rectangle bookkeeping: the map is a Rect, every cut
 * splits a Rect in two, and a room is a smaller Rect sitting inside one.
 * Point is just a single map cell (a room centre, a corridor corner). Both
 * are passed and returned by value.
 */
typedef struct {
    int x, y;        /* column, row of a map cell (0-based, top-left)   */
} Point;
typedef struct {
    int x, y;        /* top-left corner cell                            */
    int w, h;        /* size in cells; covers x..x+w-1, y..y+h-1        */
} Rect;

static Point rect_center(Rect r) { return (Point){ r.x + r.w / 2, r.y + r.h / 2 }; }

/* cut a rect into left/right pieces at column offset `at`. */
static void rect_split_v(Rect b, int at, Rect *lo, Rect *hi)
{
    *lo = (Rect){ b.x,      b.y, at,       b.h };
    *hi = (Rect){ b.x + at, b.y, b.w - at, b.h };
}

/* cut a rect into top/bottom pieces at row offset `at`. */
static void rect_split_h(Rect b, int at, Rect *lo, Rect *hi)
{
    *lo = (Rect){ b.x, b.y,      b.w, at       };
    *hi = (Rect){ b.x, b.y + at, b.w, b.h - at };
}

/* ===================================================================== */
/* §5  dungeon — data model, pure reads, and generation effects           */
/*                                                                        */
/* Rule of thumb: a `Dungeon *` argument means the function changes the    */
/* world; `const Dungeon *` means it only reads. Types and reads first,    */
/* then the effects that grow the dungeon phase by phase.                  */
/* ===================================================================== */

/*
 * Fifo — a simple first-in-first-out queue of cell or node numbers. The
 * dungeon reveals its work in the order it was added: leaves waiting to be
 * cut, room cells waiting to be dug, corridor cells waiting to be dug. All
 * three are just lists of ints, so one queue type covers them all (and the
 * pathfinding search reuses an emptied one as its to-visit list).
 *
 *   No wrap-around (head and tail only ever move forward): each number is
 *   added at most once per phase and the queue is cleared between phases, so
 *   the cursors can't run off the end. Simpler than a circular buffer.
 */
typedef struct {
    int items[CELLS_MAX]; /* storage, big enough for the largest phase (carve) */
    int head;             /* where the next pop comes from                     */
    int tail;             /* where the next push goes; count = tail - head     */
} Fifo;

static void fifo_clear(Fifo *q)       { q->head = q->tail = 0; }
static bool fifo_empty(const Fifo *q) { return q->head >= q->tail; }
static int  fifo_pop  (Fifo *q)       { return q->items[q->head++]; }
static void fifo_push (Fifo *q, int v)        /* silently drops if somehow full */
{
    if (q->tail < CELLS_MAX) q->items[q->tail++] = v;
}

/*
 * BSPNode — one rectangle in the chopping tree. It's either split into two
 * children, or it's a leaf small enough to hold a single room. The shape of
 * this tree IS the dungeon's layout: the repeated halving tiles the map with
 * no overlaps, and because the tree is all one connected piece, joining the
 * two sides at every fork wires every room to every other (the classic BSP
 * dungeon recipe, Fuchs/Kedem/Naylor 1980, popularised by Buck 2014).
 *
 * Children are array slots in Dungeon.nodes[], not pointers, so the whole
 * tree sits in one fixed array — no malloc, and a child link is just an int.
 */
typedef struct {
    Rect bounds;        /* the rectangle this node covers (whole map at root) */
    int  left, right;   /* child slot numbers; both -1 means this is a leaf   */
    bool has_room;      /* leaf only: true once a room has been placed here   */
    Rect room;          /* the room inside, only meaningful for a leaf with a
                         * room (forks have no room; they only route corridors) */
} BSPNode;

/*
 * Dungeon — all the data for one generated map (no behaviour). Which phase
 * is currently running lives in the Scene enum (§6); this struct just holds
 * what those phases read and change. It comes in four groups:
 *   1. the map      — the grid of tiles you see, plus glow values per cell
 *   2. the tree     — the rectangle chopping that decides the layout
 *   3. the queues   — to-do lists that drive the cell-by-cell reveal
 *   4. the path     — the finished-map route, plus tallies for the HUD
 * Everything is a fixed-size array, so starting a new map is just resetting
 * fields in place — nothing is allocated or freed at runtime.
 */
typedef struct {
    /* ── the visible map ── */
    int      w, h;                      /* grid size in cells               */
    int      total_cells;               /* w·h, kept handy as a loop bound  */
    uint8_t  tiles[CELLS_MAX];          /* TILE_WALL or TILE_FLOOR per cell  */
    float    carve_glow[CELLS_MAX];     /* fresh-dig flash, fades each tick  */
    float    part_glow[CELLS_MAX];      /* partition-outline glow, fades too */
    uint8_t  part_type[CELLS_MAX];      /* which outline glyph: '+' '-' '|' or 0 */

    /* ── the chopping tree ── */
    BSPNode  nodes[MAX_NODES];          /* every rectangle, root at slot 0   */
    int      n_nodes;                   /* how many slots are in use         */
    int      root_idx;                  /* the whole-map node (always 0)     */

    /* ── to-do lists, one per phase; pop a few each tick to animate ── */
    Fifo     build;      /* leaves still waiting to be cut                 */
    Fifo     carve;      /* room cells still waiting to be dug             */
    Fifo     corridor;   /* corridor cells still waiting to be dug         */

    /* ── the finished-map solution path (blinks until you press n) ── */
    uint8_t  on_path[CELLS_MAX];        /* 1 = this cell is on the route    */
    int      bfs_prev[CELLS_MAX];       /* for the search: which cell we came
                                         * from (-2 unseen, -1 start)       */
    int      path_len;                  /* how many cells the route is long */

    /* ── running tallies shown in the HUD ── */
    int      leaf_count;                /* rooms-to-be, grows while cutting */
    int      room_count;                /* rooms placed                     */
    int      corridor_count;            /* corridors planned                */
} Dungeon;

static inline int dun_idx(const Dungeon *d, int x, int y) { return y * d->w + x; }
static inline bool dun_in_bounds(const Dungeon *d, int x, int y)
{
    return x >= 0 && x < d->w && y >= 0 && y < d->h;
}

/* ── generation effects (change the Dungeon; ordered by §6) ───────────── */

/* reset the visible map to solid wall, with no glow, outline, or path. */
static void dun_clear_raster(Dungeon *d)
{
    for (int i = 0; i < d->total_cells; i++) {
        d->tiles[i]      = TILE_WALL;
        d->carve_glow[i] = 0.0f;
        d->part_glow[i]  = 0.0f;
        d->part_type[i]  = 0;
        d->on_path[i]    = 0;
    }
    d->path_len = 0;
}

static void dun_reset(Dungeon *d, int w, int h)
{
    d->w = w;  d->h = h;  d->total_cells = w * h;

    dun_clear_raster(d);

    /* Start the tree as a single rectangle: the whole map. */
    d->n_nodes  = 0;
    d->root_idx = d->n_nodes++;
    d->nodes[d->root_idx] = (BSPNode){
        .bounds = { 0, 0, w, h }, .left = -1, .right = -1, .has_room = false
    };

    /* Cutting begins with just that rectangle on the to-do list. */
    fifo_clear(&d->build);
    fifo_push(&d->build, d->root_idx);
    fifo_clear(&d->carve);
    fifo_clear(&d->corridor);

    d->leaf_count     = 1;        /* the root rectangle counts as one */
    d->room_count     = 0;
    d->corridor_count = 0;
}

/*
 * Light up the outline of one rectangle so you can watch the tree grow.
 * Corners get '+', the top/bottom edges get '-', the sides get '|'. Two
 * touching rectangles share an edge and both paint the same glyph there,
 * so it doesn't matter which one paints first. Called as each rectangle
 * is created.
 */
static void dun_paint_outline(Dungeon *d, int idx)
{
    Rect b = d->nodes[idx].bounds;
    int x0 = b.x;
    int y0 = b.y;
    int x1 = b.x + b.w - 1;
    int y1 = b.y + b.h - 1;
    if (!dun_in_bounds(d, x0, y0) || !dun_in_bounds(d, x1, y1)) return;

    /* Top and bottom rows: corners are '+', the rest is '-'. */
    for (int x = x0; x <= x1; x++) {
        int top = dun_idx(d, x, y0);
        int bot = dun_idx(d, x, y1);
        d->part_glow[top] = 1.0f;
        d->part_glow[bot] = 1.0f;
        char kind = (x == x0 || x == x1) ? '+' : '-';
        d->part_type[top] = (uint8_t)kind;
        d->part_type[bot] = (uint8_t)kind;
    }
    /* Left and right columns: '|', skipping the corners already done above. */
    for (int y = y0 + 1; y < y1; y++) {
        int lf = dun_idx(d, x0, y);
        int rt = dun_idx(d, x1, y);
        d->part_glow[lf] = 1.0f;
        d->part_glow[rt] = 1.0f;
        d->part_type[lf] = (uint8_t)'|';
        d->part_type[rt] = (uint8_t)'|';
    }
}

/*
 * Decide which way to cut a rectangle. We cut along the longer side (so
 * pieces stay roughly square), unless only one direction has room, in which
 * case we're forced. For a near-square rectangle we flip a coin. Returns
 * true to cut vertically (into a left and right piece).
 */
static bool split_axis_vertical(int w, int h, bool can_w, bool can_h)
{
    if (can_w && !can_h) return true;
    if (can_h && !can_w) return false;
    if (w * 10 > h * 14) return true;     /* clearly wider  → cut vertically   */
    if (h * 10 > w * 14) return false;    /* clearly taller → cut horizontally */
    return (rand() & 1);                  /* near-square → coin flip            */
}

/*
 * Take the next rectangle off the to-do list and try to cut it in two.
 * Returns false only when the list is empty, meaning the cutting phase is
 * over. A rectangle too small to cut just stays as it is (becomes a room
 * site).
 */
static bool dun_split_step(Dungeon *d)
{
    if (fifo_empty(&d->build)) return false;

    int  idx = fifo_pop(&d->build);
    Rect b   = d->nodes[idx].bounds;

    /* Too small to cut either way → leave it as a room site. */
    bool can_split_w = b.w >= 2 * MIN_LEAF_W;
    bool can_split_h = b.h >= 2 * MIN_LEAF_H;
    if (!can_split_w && !can_split_h) return true;

    /* Out of tree slots → stop cutting this one. */
    if (d->n_nodes + 2 > MAX_NODES) return true;

    /* Pick a direction, then a random spot to cut that leaves both pieces
     * at least MIN_LEAF wide/tall. */
    bool vertical = split_axis_vertical(b.w, b.h, can_split_w, can_split_h);
    int  span     = vertical ? b.w : b.h;
    int  min_leaf = vertical ? MIN_LEAF_W : MIN_LEAF_H;
    int  min_pos  = min_leaf, max_pos = span - min_leaf;
    if (max_pos <= min_pos) return true;             /* no valid cut spot */
    int  at = min_pos + rand() % (max_pos - min_pos);

    Rect lo, hi;
    if (vertical) rect_split_v(b, at, &lo, &hi);
    else          rect_split_h(b, at, &lo, &hi);

    int left_idx  = d->n_nodes++;
    int right_idx = d->n_nodes++;
    d->nodes[left_idx]  = (BSPNode){ .bounds = lo, .left = -1, .right = -1, .has_room = false };
    d->nodes[right_idx] = (BSPNode){ .bounds = hi, .left = -1, .right = -1, .has_room = false };

    /* Attach the two new pieces and queue them up for cutting too. */
    d->nodes[idx].left  = left_idx;
    d->nodes[idx].right = right_idx;
    d->leaf_count += 1;         /* one rectangle became two → net +1 */
    fifo_push(&d->build, left_idx);
    fifo_push(&d->build, right_idx);

    dun_paint_outline(d, left_idx);
    dun_paint_outline(d, right_idx);

    return true;
}

/* Pick a room width or height that fits inside a rectangle of `leaf_dim`,
 * leaving a wall border. Falls back to the smallest allowed size when the
 * rectangle is too tight to give any choice. */
static int dun_pick_room_dim(int leaf_dim, int min_room, int max_margin)
{
    int margin_total = 2 * max_margin;
    int max_room = leaf_dim - margin_total;
    if (max_room < min_room) max_room = min_room;
    if (max_room > leaf_dim - 2) max_room = leaf_dim - 2;
    if (max_room < min_room) max_room = min_room;
    int range = max_room - min_room + 1;
    if (range <= 0) range = 1;
    return min_room + rand() % range;
}

/*
 * Build a room rectangle that floats inside a leaf: pick a size that leaves
 * at least a one-cell wall border all round, then a random position within
 * that border so rooms don't all hug the same corner.
 */
static Rect room_inside_leaf(Rect b)
{
    int rw = dun_pick_room_dim(b.w, MIN_ROOM_W, MAX_ROOM_MARGIN);
    int rh = dun_pick_room_dim(b.h, MIN_ROOM_H, MAX_ROOM_MARGIN);
    if (rw > b.w - 2) rw = b.w - 2;          /* leave a 1-cell wall border */
    if (rh > b.h - 2) rh = b.h - 2;
    if (rw < 1) rw = 1;
    if (rh < 1) rh = 1;

    int max_x_off = b.w - rw - 1; if (max_x_off < 1) max_x_off = 1;
    int max_y_off = b.h - rh - 1; if (max_y_off < 1) max_y_off = 1;
    return (Rect){ b.x + 1 + rand() % max_x_off,
                   b.y + 1 + rand() % max_y_off, rw, rh };
}

/* add every cell of a room to the carve to-do list so it digs in slowly. */
static void dun_queue_room_cells(Dungeon *d, Rect room)
{
    for (int ry = 0; ry < room.h; ry++)
        for (int rx = 0; rx < room.w; rx++) {
            int cx = room.x + rx, cy = room.y + ry;
            if (dun_in_bounds(d, cx, cy))
                fifo_push(&d->carve, dun_idx(d, cx, cy));
        }
}

/* once cutting is done, give every leaf rectangle a room. */
static void dun_place_rooms(Dungeon *d)
{
    for (int i = 0; i < d->n_nodes; i++) {
        BSPNode *n = &d->nodes[i];
        if (n->left != -1) continue;        /* a fork, not a leaf — skip */
        n->room     = room_inside_leaf(n->bounds);
        n->has_room = true;
        d->room_count++;
        dun_queue_room_cells(d, n->room);
    }
}

/* dig out the next room cell, flashing it as freshly cut. Returns false
 * when there's nothing left to dig. */
static bool dun_carve_step(Dungeon *d)
{
    if (fifo_empty(&d->carve)) return false;
    int idx = fifo_pop(&d->carve);
    d->tiles[idx]      = TILE_FLOOR;
    d->carve_glow[idx] = 1.0f;
    return true;
}

/*
 * Pick one room from the half of the tree under `idx` and return its centre.
 * It walks down to a leaf, flipping a coin at each fork. Corridor planning
 * uses this to grab a room on each side of a fork to connect.
 */
static Point dun_subtree_room_centre(const Dungeon *d, int idx)
{
    int cur = idx;
    while (d->nodes[cur].left != -1)
        cur = (rand() & 1) ? d->nodes[cur].left : d->nodes[cur].right;
    return rect_center(d->nodes[cur].room);
}

/* queue a straight run of cells from a to b (they must line up on a row or
 * a column) so the corridor digs out one cell at a time. */
static void dun_queue_corridor_segment(Dungeon *d, Point a, Point b)
{
    int dx = (b.x > a.x) ? 1 : (b.x < a.x) ? -1 : 0;
    int dy = (b.y > a.y) ? 1 : (b.y < a.y) ? -1 : 0;
    int x = a.x, y = a.y;
    while (1) {
        if (dun_in_bounds(d, x, y)) fifo_push(&d->corridor, dun_idx(d, x, y));
        if (x == b.x && y == b.y) break;
        x += dx;
        y += dy;
    }
}

/*
 * Plan one L-shaped corridor at every fork in the tree, joining a room from
 * the left side to a room from the right. It walks the tree bottom-up so the
 * inner forks connect first. The L turns at a random corner — going across
 * then down, or down then across — just for variety.
 */
static void dun_plan_corridors(Dungeon *d, int idx)
{
    const BSPNode *n = &d->nodes[idx];
    if (n->left == -1) return;

    dun_plan_corridors(d, n->left);
    dun_plan_corridors(d, n->right);

    Point a     = dun_subtree_room_centre(d, n->left);
    Point b     = dun_subtree_room_centre(d, n->right);
    Point elbow = (rand() & 1) ? (Point){ b.x, a.y }   /* across, then down */
                               : (Point){ a.x, b.y };  /* down, then across */
    dun_queue_corridor_segment(d, a, elbow);
    dun_queue_corridor_segment(d, elbow, b);
    d->corridor_count++;
}

/* dig out the next corridor cell. The renderer tints it gold while the
 * corridor phase is running; once the glow fades it's plain floor. Returns
 * false when there's nothing left to dig. */
static bool dun_corridor_step(Dungeon *d)
{
    if (fifo_empty(&d->corridor)) return false;
    int idx = fifo_pop(&d->corridor);
    d->tiles[idx]      = TILE_FLOOR;
    d->carve_glow[idx] = 1.0f;
    return true;
}

/*
 * Find the two rooms nearest opposite corners of the map — the ones we'll
 * draw a glowing route between. We score each room by x+y: smallest is up in
 * the top-left, largest is down in the bottom-right. Returns false if there
 * are no rooms at all.
 */
static bool dun_extreme_rooms(const Dungeon *d, Point *a, Point *b)
{
    int  lo = 0, hi = 0;
    bool first = true;
    for (int i = 0; i < d->n_nodes; i++) {
        const BSPNode *n = &d->nodes[i];
        if (n->left != -1 || !n->has_room) continue;   /* leaves with a room */
        Point c   = rect_center(n->room);
        int   key = c.x + c.y;
        if (first || key < lo) { lo = key; *a = c; }    /* nearest top-left  */
        if (first || key > hi) { hi = key; *b = c; }    /* nearest bot-right */
        first = false;
    }
    return !first;
}

/*
 * Flood out from `start` over floor cells, one ring at a time, until we
 * reach `goal`. For each cell we note which neighbour we arrived from, so we
 * can trace the route back afterward. Because it spreads evenly in all
 * directions, the first time we touch the goal is the shortest way there
 * (the classic maze-solving search, Moore 1959). It borrows the now-empty
 * corridor queue's storage as its to-visit list.
 */
static void dun_bfs(Dungeon *d, int start, int goal)
{
    for (int i = 0; i < d->total_cells; i++) d->bfs_prev[i] = -2;  /* not seen yet */
    int *frontier = d->corridor.items;
    int qh = 0, qt = 0;
    d->bfs_prev[start] = -1;                       /* -1 marks where we began */
    frontier[qt++] = start;

    static const int DX4[4] = { 1, -1, 0, 0 };
    static const int DY4[4] = { 0, 0, 1, -1 };
    while (qh < qt) {
        int cur = frontier[qh++];
        if (cur == goal) break;
        int cx = cur % d->w, cy = cur / d->w;
        for (int k = 0; k < 4; k++) {
            int nx = cx + DX4[k], ny = cy + DY4[k];
            if (!dun_in_bounds(d, nx, ny)) continue;
            int ni = dun_idx(d, nx, ny);
            if (d->tiles[ni] != TILE_FLOOR || d->bfs_prev[ni] != -2) continue;
            d->bfs_prev[ni] = cur;                 /* remember how we got here */
            frontier[qt++] = ni;
        }
    }
}

/* follow the came-from trail back from the goal to the start, lighting up
 * each cell on the route. */
static void dun_mark_path(Dungeon *d, int goal)
{
    if (d->bfs_prev[goal] == -2) return;           /* goal never reached (shouldn't happen) */
    for (int c = goal; c != -1; c = d->bfs_prev[c]) {
        d->on_path[c] = 1;
        d->path_len++;
    }
}

/* work out the glowing route shown on the finished map: pick the two
 * corner-most rooms, find the shortest way between them, and mark it. */
static void dun_plan_path(Dungeon *d)
{
    for (int i = 0; i < d->total_cells; i++) d->on_path[i] = 0;
    d->path_len = 0;

    Point a, b;
    if (!dun_extreme_rooms(d, &a, &b)) return;     /* no rooms — nothing to draw */

    int start = dun_idx(d, a.x, a.y);
    int goal  = dun_idx(d, b.x, b.y);
    dun_bfs(d, start, goal);
    dun_mark_path(d, goal);
}

/* ===================================================================== */
/* §6  scene — the phase state machine that sequences the effects         */
/* ===================================================================== */

/*
 * The four phases the map goes through, in order:
 *   PARTITION — chop the map into rectangles. Done when nothing's left to
 *               cut; then we place a room in each leaf.
 *   CARVE     — dig out the rooms cell by cell. Done when all are dug;
 *               then we plan the corridors.
 *   CORRIDORS — dig out the corridors. Done when all are dug; then we work
 *               out the solution path.
 *   HOLD      — finished. The path blinks until you press n for a new map.
 *
 * A freshly dug cell is tinted by which phase is running (room colour in
 * CARVE, corridor colour in CORRIDORS), so the renderer checks this value.
 */
typedef enum {
    SCENE_PARTITION = 0,
    SCENE_CARVE     = 1,
    SCENE_CORRIDORS = 2,
    SCENE_HOLD      = 3,
} SceneState;

/*
 * Scene — one running build: the dungeon data plus the knobs that drive and
 * present it. `state` says which phase is active; the per-tick counts set
 * how fast the reveal goes; theme and anim_time are just for looks. This is
 * the single thing the renderer reads and the main loop advances.
 */
typedef struct {
    Dungeon     d;                  /* the map being built (§5)              */
    SceneState  state;              /* which phase is running now            */
    bool        paused;             /* stop building; keep drawing           */
    int         build_per_tick;     /* cuts revealed per tick (kept slow)    */
    int         carve_per_tick;     /* room cells dug per tick               */
    int         corridor_per_tick;  /* corridor cells dug per tick           */
    float       anim_time;          /* keeps ticking; drives the blink/breathe */
    int         theme;              /* which colour scheme (t/T)             */
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    dun_reset(&s->d, mw, mh);
    s->state = SCENE_PARTITION;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->build_per_tick   = BUILD_STEPS_DEF;
    s->carve_per_tick   = CARVE_STEPS_DEF;
    s->corridor_per_tick= CORRIDOR_STEPS_DEF;
    scene_reset(s, mw, mh);
}

/* fade every glow a little, so fresh cuts cool off over time. */
static void scene_decay_glows(Dungeon *d, float dt)
{
    float carve_d = expf(-CARVE_GLOW_DECAY * dt);
    float part_d  = expf(-PART_GLOW_DECAY  * dt);
    for (int i = 0; i < d->total_cells; i++) {
        d->carve_glow[i] *= carve_d;
        d->part_glow[i]  *= part_d;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    s->anim_time += dt;
    scene_decay_glows(&s->d, dt);

    switch (s->state) {

    case SCENE_PARTITION:
        for (int i = 0; i < s->build_per_tick; i++) {
            if (!dun_split_step(&s->d)) {
                /* nothing left to cut → place rooms, start carving */
                dun_place_rooms(&s->d);
                s->state = SCENE_CARVE;
                break;
            }
        }
        break;

    case SCENE_CARVE:
        for (int i = 0; i < s->carve_per_tick; i++) {
            if (!dun_carve_step(&s->d)) {
                /* rooms all dug → plan corridors, start digging them */
                dun_plan_corridors(&s->d, s->d.root_idx);
                s->state = SCENE_CORRIDORS;
                break;
            }
        }
        break;

    case SCENE_CORRIDORS:
        for (int i = 0; i < s->corridor_per_tick; i++) {
            if (!dun_corridor_step(&s->d)) {
                /* map complete → work out the glowing path, then hold */
                dun_plan_path(&s->d);
                s->state = SCENE_HOLD;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        /* Done. The path keeps blinking (anim_time still advances above);
         * we wait here until the user presses n for a new map. */
        break;
    }
}

/* ===================================================================== */
/* §7  render — PURE: const Scene → screen (mutates nothing but the term)  */
/* ===================================================================== */

/*
 * Drawing is pure: it reads the scene and paints the terminal, changing
 * nothing else. Each frame is erase → draw the map → draw the HUD → flush.
 *
 * Glyphs are all plain ASCII: '#' wall, '.' floor, and '+' '-' '|' for the
 * rectangle outlines while the map is being cut. Walls only show where they
 * touch a floor — the dead space inside walls is left blank, which gives the
 * classic roguelike look.
 *
 * When several things want the same cell, the brightest wins, in this order:
 *   blinking path > a freshly dug cell > a rectangle outline > settled floor
 *   or wall. The glow strength picks how hot a colour to use, so a fading
 *   cut cools off smoothly like an ember.
 */
/* Screen — the terminal's current size, re-read on start and on resize. */
typedef struct {
    int cols, rows;   /* width and height in character cells */
} Screen;

static void screen_init(Screen *s)
{
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
 * Count how many of a cell's 8 neighbours are floor. The renderer uses this
 * twice: a wall with no floor around it is dead space and stays blank, and a
 * wall touching more floor is drawn brighter, faking torchlight and depth.
 */
static int tile_floor_neighbours(const Dungeon *d, int x, int y)
{
    int count = 0;
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (dun_in_bounds(d, nx, ny) &&
                d->tiles[dun_idx(d, nx, ny)] == TILE_FLOOR) count++;
        }
    return count;
}

/* scatter a few brighter floor cells, like worn flagstones, using a cheap
 * pattern so the same cells always light up. */
static bool flagstone_at(int x, int y) { return (x * 3 + y * 7) % FLAGSTONE_MOD == 0; }

/* what to draw at one cell; draw=false means leave it blank. */
typedef struct { int pair; attr_t attr; char glyph; bool draw; } CellDraw;

/*
 * Decide how one cell should look. Brightest thing wins, checked top to
 * bottom: blinking path, then a freshly dug cell, then a rectangle outline,
 * then plain floor, then wall. While the map is finished and holding, the
 * blink and breathe values nudge the brightness up and down.
 */
static CellDraw cell_appearance(const Scene *s, int x, int y,
                                bool holding, float blink, float breathe)
{
    const Dungeon *d   = &s->d;
    int     idx = dun_idx(d, x, y);
    float   cg  = d->carve_glow[idx], pg = d->part_glow[idx];
    uint8_t k   = d->tiles[idx];
    int     dig = (s->state == SCENE_CORRIDORS) ? PAIR_COR_0 : PAIR_ROOM_0;
    int     t;

    if (holding && d->on_path[idx]) {                 /* on the blinking path */
        t = glow_tier(blink);
        return (CellDraw){ PAIR_PATH_0 + t, (t <= 1) ? A_BOLD : A_NORMAL, '*', true };
    }
    if (cg > GLOW_THRESHOLD) {                         /* freshly dug, still glowing */
        t = glow_tier(cg);
        return (CellDraw){ dig + t, (t <= 1) ? A_BOLD : A_NORMAL,
                           (k == TILE_FLOOR) ? '.' : '#', true };
    }
    if (pg > GLOW_THRESHOLD && k == TILE_WALL && d->part_type[idx]) { /* rectangle outline */
        t = glow_tier(pg);
        return (CellDraw){ PAIR_PART_0 + t, (t == 0) ? A_BOLD : A_NORMAL,
                           (char)d->part_type[idx], true };
    }
    if (k == TILE_FLOOR) {                             /* settled floor */
        attr_t a = flagstone_at(x, y) ? A_NORMAL : A_DIM;
        if (holding && breathe > BREATHE_FLOOR_HI) a = A_NORMAL;
        return (CellDraw){ PAIR_FLOOR, a, '.', true };
    }
    /* Wall: only drawn where it touches floor, brighter the more it touches. */
    int nb = tile_floor_neighbours(d, x, y);
    if (nb == 0) return (CellDraw){ 0, A_NORMAL, ' ', false };  /* dead space, leave blank */
    attr_t a = (nb >= TORCH_LIT_NB) ? A_BOLD
             : (nb >= TORCH_DIM_NB) ? A_NORMAL : A_DIM;
    if (holding && breathe < BREATHE_WALL_LO) a = A_DIM;
    return (CellDraw){ PAIR_WALL, a, '#', true };
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Dungeon *d = &s->d;

    /* Centre the map, leaving the top and bottom rows for the HUD. */
    int gx0 = (cols - d->w) / 2;             if (gx0 < 0) gx0 = 0;
    int gy0 = ((rows - HUD_ROWS) - d->h) / 2 + 1; if (gy0 < 1) gy0 = 1;

    /* On the finished map the path pulses and the whole thing breathes. */
    bool  holding = (s->state == SCENE_HOLD);
    float blink   = 0.5f + 0.5f * sinf(s->anim_time * PATH_BLINK_RATE);  /* sways 0..1 */
    float breathe = holding ? BREATHE_BASE + BREATHE_AMP * sinf(s->anim_time * BREATHE_RATE)
                            : 1.0f;

    for (int y = 0; y < d->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < d->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            CellDraw c = cell_appearance(s, x, y, holding, blink, breathe);
            if (!c.draw) continue;
            attron(COLOR_PAIR(c.pair) | c.attr);
            mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
            attroff(COLOR_PAIR(c.pair) | c.attr);
        }
    }
}

/* draw one full-width HUD bar, clipped to `cols` so a long line can't spill
 * onto the map. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* the phase name for the HUD, padded to a fixed width so the bar doesn't
 * jiggle as the text changes. */
static const char *phase_label(const Scene *s)
{
    if (s->paused)                     return "PAUSED   ";
    if (s->state == SCENE_PARTITION)   return "PARTITION";
    if (s->state == SCENE_CARVE)       return "CARVING  ";
    if (s->state == SCENE_CORRIDORS)   return "CORRIDOR ";
    return "HOLD     ";
}

/* fill in the top status bar: phase, theme, live tallies, speeds, and fps. */
static void hud_status_line(char *buf, size_t n, const Scene *s,
                            double fps, int sim_fps)
{
    const Dungeon *d = &s->d;
    snprintf(buf, n,
             " BSP_DUNGEON  %s  theme:%s (%d/%d)  leaves:%-3d  rooms:%-3d  "
             "cor:%-3d  pace:%d/%d/%d  %5.1f fps  %3d Hz ",
             phase_label(s), THEMES[s->theme].name, s->theme + 1, N_THEMES,
             d->leaf_count, d->room_count, d->corridor_count,
             s->build_per_tick, s->carve_per_tick, s->corridor_per_tick,
             fps, sim_fps);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    char data[180];
    hud_status_line(data, sizeof data, s, fps, sim_fps);

    const char *keys =       /* bottom bar — lists every key you can press */
        " q:quit  spc:pause  n:new map  t/T:theme  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app — orchestration: input → effects → delay → render              */
/* ===================================================================== */

/*
 * FpsMeter — averages the frame rate over a short window so the number in
 * the HUD reads steadily instead of jittering every frame. It tallies frames
 * and elapsed time, divides once the window is up, and resets.
 */
typedef struct {
    int64_t accum_ns;   /* time tallied since the last readout          */
    int     frames;     /* frames tallied since the last readout        */
    double  value;      /* last fps figure, shown in the HUD            */
} FpsMeter;

static void fps_meter_tick(FpsMeter *m, int64_t dt)
{
    m->frames++;
    m->accum_ns += dt;
    if (m->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        m->value    = (double)m->frames / ((double)m->accum_ns / (double)NS_PER_SEC);
        m->frames   = 0;
        m->accum_ns = 0;
    }
}

/*
 * App — the whole running program: the Scene plus the loop's own
 * housekeeping (terminal size, timing, the fps meter, and the signal flags).
 * Kept separate from Scene because these are about running the program, not
 * about the dungeon itself. There's one global instance (g_app) so the
 * signal handlers, which can't take arguments, can reach the flags.
 */
typedef struct {
    Scene                 scene;       /* the build in progress (§6)          */
    Screen                screen;      /* terminal size                       */
    int                   sim_fps;     /* how often the build steps; the
                                        * repaint is capped at RENDER_FPS      */
    int                   map_w, map_h;/* dungeon size, fit to the terminal   */
    int64_t               sim_accum;   /* banks real time so the build can run
                                        * a fixed number of steps per second   */
    FpsMeter              fps;          /* the rolling fps figure               */
    volatile sig_atomic_t running;     /* 0 = quit; set by SIGINT/SIGTERM.
                                        * volatile sig_atomic_t so it's safe to
                                        * change from a signal handler          */
    volatile sig_atomic_t need_resize; /* set by SIGWINCH, handled atop the loop */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* size the dungeon to the terminal, minus the two HUD rows, and within the
 * fixed array limits. */
static void app_pick_map_size(App *app)
{
    app->map_w = clampi(app->screen.cols,            MIN_MAP_W, MAP_W_MAX);
    app->map_h = clampi(app->screen.rows - HUD_ROWS, MIN_MAP_H, MAP_H_MAX);
}

/* speed up the reveal one notch by roughly doubling each phase's per-tick
 * rate, kept within limits. */
static void scene_pace_faster(Scene *s)
{
    if (s->build_per_tick < BUILD_STEPS_MAX) s->build_per_tick *= 2;
    s->build_per_tick    = clampi(s->build_per_tick,        BUILD_STEPS_MIN, BUILD_STEPS_MAX);
    s->carve_per_tick    = clampi(s->carve_per_tick    * 2, 1, PACE_MAX);
    s->corridor_per_tick = clampi(s->corridor_per_tick * 2, 1, PACE_MAX);
}
static void scene_pace_slower(Scene *s)
{
    s->build_per_tick    = clampi(s->build_per_tick    / 2, BUILD_STEPS_MIN, BUILD_STEPS_MAX);
    s->carve_per_tick    = clampi(s->carve_per_tick    / 2, 1, PACE_MAX);
    s->corridor_per_tick = clampi(s->corridor_per_tick / 2, 1, PACE_MAX);
}

/* nudge the build tick rate up or down one step, within limits. */
static void app_nudge_fps(App *app, int delta)
{
    app->sim_fps = clampi(app->sim_fps + delta, SIM_FPS_MIN, SIM_FPS_MAX);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->sim_accum   = 0;
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'n': case 'N':                 /* generate the next map */
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme - 1 + N_THEMES) % N_THEMES;
        theme_apply(s->theme);
        break;
    case '=': case '+': scene_pace_faster(s);            break;
    case '-':           scene_pace_slower(s);            break;
    case ']':           app_nudge_fps(app, +SIM_FPS_STEP); break;
    case '[':           app_nudge_fps(app, -SIM_FPS_STEP); break;
    default: break;
    }
    return true;
}

/* how long since the last frame, capped so one big stall (a slow terminal,
 * a paused window) can't make the build suddenly jump ahead. */
static int64_t frame_delta(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    return dt > DT_CAP_MS * NS_PER_MS ? DT_CAP_MS * NS_PER_MS : dt;
}

/* advance the build by a steady number of steps per second, no matter how
 * fast the screen happens to be repainting. */
static void app_advance(App *app, int64_t dt)
{
    int64_t tick   = TICK_NS(app->sim_fps);
    float   dt_sec = (float)tick / (float)NS_PER_SEC;
    app->sim_accum += dt;
    while (app->sim_accum >= tick) {
        scene_tick(&app->scene, dt_sec);
        app->sim_accum -= tick;
    }
}

/* sleep off the rest of the frame so we don't repaint faster than RENDER_FPS. */
static void app_pace_frame(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(TICK_NS(RENDER_FPS) - elapsed);
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    int64_t frame_time = clock_ns();
    app->sim_accum     = 0;

    /*
     * The main loop, five steps each time round:
     *   INPUT   read a keypress (may start a new map, change theme, etc.)
     *   TIME    measure how long since last time
     *   EFFECTS run the build forward by that much
     *   DELAY   wait so we don't draw too fast
     *   RENDER  paint the screen
     */
    while (app->running) {
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int ch = getch();                                  /* INPUT   */
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;

        int64_t dt = frame_delta(&frame_time);             /* TIME    */

        app_advance(app, dt);                              /* EFFECTS */
        fps_meter_tick(&app->fps, dt);
        app_pace_frame(frame_time, dt);                    /* DELAY   */

        screen_draw(&app->screen, &app->scene,             /* RENDER  */
                    app->fps.value, app->sim_fps);
        screen_present();
    }

    screen_free(&app->screen);
    return 0;
}
