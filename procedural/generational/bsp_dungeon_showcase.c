/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bsp_dungeon_showcase.c — Binary Space Partition dungeon, animated.
 *
 * DEMO: Watch a classic roguelike dungeon assemble in three visible
 *       phases. PARTITION: a single rectangle (the whole map) is
 *       recursively split with cyan grid lines, dividing into leaf
 *       cells. CARVE: inside each leaf, a smaller pink room appears
 *       cell-by-cell. CORRIDORS: gold L-shaped passages walk between
 *       sibling rooms, joining everything into one connected dungeon.
 *       HOLD; supernova flash; restart forever with a new layout.
 *
 * Study alongside: ./maze_backtracker.c — the OTHER classic dungeon-
 *       like generator. DFS makes one twisty corridor; BSP makes many
 *       distinct rooms with controlled corridors between. BSP is what
 *       Rogue / NetHack / classic roguelikes use.
 *
 * Section map:
 *   §1 config   — map size, leaf min size, palette, phase pacing
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + wall / floor / partition / room / corridor
 *   §5 bsp      — BSPNode tree, split, room placement, corridor planning
 *   §6 scene    — PARTITION / CARVE / CORRIDORS / HOLD state machine
 *   §7 screen   — ASCII render: # walls, . floors, partition outlines
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate restart)
 *   + / =      faster phase pacing
 *   -          slower phase pacing
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra bsp_dungeon_showcase.c \
 *       -o bsp_dungeon -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Binary Space Partition for dungeon generation. Three
 *                  stages:
 *                    1. PARTITION the map by repeatedly splitting the
 *                       largest leaf rectangle along its longer axis.
 *                       Stops when leaves are below a size threshold.
 *                       Result: a binary tree of axis-aligned rectangles
 *                       tiling the map with no overlap.
 *                    2. CARVE one room inside each leaf, with random
 *                       padding so rooms don't fill their leaf entirely.
 *                    3. CONNECT sibling rooms via L-shaped corridors —
 *                       a post-order traversal of the BSP tree, joining
 *                       a representative room from each subtree at every
 *                       internal node. Result: a connected, axis-aligned
 *                       dungeon.
 *
 * Data-structure : Static array of BSPNode (max 256). Each node stores
 *                  its bounding box, child indices (-1 for leaves), and —
 *                  for leaves — the carved room rectangle inside.
 *
 *                  Map: a flat tile grid (TILE_WALL or TILE_FLOOR) plus
 *                  per-cell glow floats for carve / partition / supernova
 *                  flashes. No allocation after init.
 *
 * Rendering      : ASCII only. '#' for walls (rendered only where a wall
 *                  cell touches a floor cell — interior void stays blank
 *                  for the classic roguelike look), '.' for floor.
 *                  During PARTITION, leaf rectangles are also drawn as
 *                  cyan outlines with '+' corners and '-' / '|' edges.
 *                  Per-cell carve_glow paints fresh-cut cells in their
 *                  phase colour (pink for rooms, gold for corridors).
 *
 * Performance    : O(N · log N) where N is the number of map cells —
 *                  splitting halves each region down a tree of depth
 *                  ~log₂(N). Carving and corridor planning are O(N) in
 *                  the carved area. We throttle each phase independently
 *                  (build_per_tick, carve_per_tick, corridor_per_tick)
 *                  so the spectacle plays at human-readable speed.
 *
 * References     : • Buck, Jamis — "Rooms and Mazes: A Procedural
 *                    Dungeon Generator" (the canonical BSP-dungeon
 *                    write-up):
 *                    https://journal.stuffwithstuff.com/2014/12/21/rooms-and-mazes/
 *                  • RogueBasin — "Basic BSP Dungeon Generation":
 *                    http://www.roguebasin.com/index.php?title=Basic_BSP_Dungeon_generation
 *                  • Wikipedia — "Binary space partitioning":
 *                    https://en.wikipedia.org/wiki/Binary_space_partitioning
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a dungeon of distinct rooms with controlled connectivity,
 * don't try to design rooms directly. Instead, RECURSIVELY CHOP the
 * map into smaller rectangles, then drop a room inside each smallest
 * piece, then walk corridors between the rooms following the chopping
 * tree itself. The tree structure of the chops automatically becomes
 * the connection graph — every room reaches every other room because
 * the tree is connected.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sheet of paper. Cut it in half (one slice). Cut each half
 * in half again. Repeat until pieces are small enough. Now in each
 * piece, draw a smaller rectangle (the room). To wire up the rooms,
 * for every cut you made (in REVERSE order), draw a line between
 * SOMETHING-on-the-left and SOMETHING-on-the-right of that cut.
 * That line becomes a corridor. The cut order determines the corridor
 * order; the tree of cuts becomes the connection graph.
 *
 * Three layers in the visual:
 *   1. CYAN outlined rectangles = current set of leaves in the BSP
 *      tree. As splits happen, big rectangles vanish and are replaced
 *      by their two children. By the end of PARTITION, you see ~30–80
 *      small cyan rectangles tiling the map.
 *   2. PINK '#' / '.' painting = rooms being carved, one cell at a
 *      time, inside each leaf. Each room is smaller than its leaf so
 *      walls remain on the leaf boundary.
 *   3. GOLD '#' / '.' painting = L-corridors being walked from one
 *      room's centre to a sibling's. Two segments per corridor.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. nodes[0] = the whole map. build_queue = [0]. Map is all
 *     TILE_WALL.
 *  2. PARTITION STEP (one operation):
 *     a. Pop node N from build_queue.
 *     b. If N is too small to split (both dims < 2 × MIN_LEAF) OR a
 *        random stop fires: leave it as a leaf. Goto next iteration.
 *     c. Otherwise pick split axis: the longer dimension biased; ties
 *        broken randomly. Pick a random split position in
 *        [MIN_LEAF, dim − MIN_LEAF].
 *     d. Create two child nodes covering the two halves; record the
 *        child indices on N. Push children onto build_queue.
 *  3. When build_queue empties: place a random room inside every leaf
 *     (random margin so the room doesn't touch the leaf edges).
 *     Transition to CARVE.
 *  4. CARVE STEP: pop a room cell from the carve queue, set it to
 *     TILE_FLOOR with pink carve_glow. When the queue empties: plan
 *     corridors and transition to CORRIDORS.
 *  5. CORRIDOR PLAN: post-order traversal of the BSP tree. At each
 *     non-leaf node N, pick one room from N.left's subtree and one
 *     from N.right's subtree (random representatives), append the
 *     L-path between them to corridor_queue.
 *  6. CORRIDOR STEP: pop a cell from corridor_queue, set TILE_FLOOR
 *     with gold carve_glow. When empty: transition to HOLD.
 *  7. HOLD: wait HOLD_SECONDS, then supernova-reset and goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Split axis bias               : if w > 1.4·h → vertical line
 *                                  if h > 1.4·w → horizontal line
 *                                  else random
 *  Split position (vertical cut) : x = MIN_LEAF + rand() %
 *                                      (w − 2·MIN_LEAF)
 *  Room placement (in leaf b)    : margin = rand() % MAX_MARGIN
 *                                  room = inset(b, margin)
 *                                  width  ≥ MIN_ROOM_W
 *                                  height ≥ MIN_ROOM_H
 *  Subtree representative        : recurse to a leaf via random child
 *                                  → return room centre
 *  L-corridor                    : (x1,y1) → (x2,y2) is two segments
 *                                  EITHER (x1,y1)→(x2,y1)→(x2,y2)
 *                                  OR     (x1,y1)→(x1,y2)→(x2,y2)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SPLIT POSITION. If MIN_LEAF is too large relative to the leaf
 *    being split, max − min in the random call goes ≤ 0 and rand() %
 *    becomes a divide-by-zero or wraps to 0/garbage. Always check
 *    `if (max <= min) return;` BEFORE the modulo.
 *
 *  • ROOM MARGIN. Likewise, MAX_MARGIN must be small enough that the
 *    leaf can fit MIN_ROOM_W × MIN_ROOM_H + 2·MAX_MARGIN. If a leaf
 *    is too narrow, fall back to placing the room with margin 0/1
 *    rather than failing — the player just gets a room that fills
 *    its leaf.
 *
 *  • CORRIDOR THROUGH WALL. An L-corridor between two rooms passes
 *    through the leaves' shared boundary by definition (it's how
 *    they connect). The corridor cells are set to TILE_FLOOR
 *    UNCONDITIONALLY — overwriting whatever was there. Don't try to
 *    "respect" the partition lines or the corridor will fail to
 *    cross.
 *
 *  • DEPTH BOUND. Without a depth cap, deeply unbalanced trees can
 *    push past MAX_NODES. Cap the recursion: stop splitting when
 *    n_nodes + 2 > MAX_NODES, treat the leaf as terminal.
 *
 *  • RAND(): for small ranges it's biased, but for dungeon variety
 *    that bias is invisible. Don't bother with rejection sampling
 *    unless you're benchmarking statistical uniformity.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • After PARTITION: every leaf has at least MIN_LEAF in both dims.
 *    Smaller leaves mean the split-stop guard is loose.
 *  • After CARVE: every leaf has exactly one TILE_FLOOR rectangle
 *    inside it; no FLOOR outside any leaf. (Corridors break this
 *    later — that's by design.)
 *  • After CORRIDORS: every room is reachable from every other room.
 *    Run a BFS from any FLOOR cell; if FLOOR cells exist that BFS
 *    doesn't reach, a corridor failed to connect.
 *  • Visual: the PARTITION cyan outlines must TILE the map exactly —
 *    no overlap, no gaps. If you see gaps, child rectangles aren't
 *    inheriting the parent's bounds correctly (off-by-one on
 *    `child.x = parent.x + split`).
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
    /* Map size in cells (= terminal cells; 1:1 mapping, no §4). */
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    /* Minimum leaf dimensions — leaves below this don't split.
     * Smaller MIN_LEAF → more rooms; larger → fewer, bigger rooms. */
    MIN_LEAF_W        =  10,
    MIN_LEAF_H        =   6,

    /* Room placement margin range — random inset from leaf edges. */
    MIN_ROOM_W        =   4,
    MIN_ROOM_H        =   3,
    MAX_ROOM_MARGIN   =   3,

    /* Static BSP tree array bound. With MIN_LEAF=10×6 on a 200×56 map
     * the deepest tree has ~7 levels → 256 leaves comfortably fits. */
    MAX_NODES         = 512,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Per-tick operation rates per phase. Build is slow (you want to
     * SEE each split); carve and corridor are fast (visible reveal but
     * not a slog). User can scale via +/-. */
    BUILD_STEPS_MIN   =   1,
    BUILD_STEPS_DEF   =   1,
    BUILD_STEPS_MAX   =  64,

    CARVE_STEPS_DEF   =  12,
    CORRIDOR_STEPS_DEF=   8,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls — mid grey         */
    PAIR_FLOOR        =   4,        /* '.' floors — steel blue dim  */
    PAIR_PARTITION    =   5,        /* BSP outlines — light cyan    */
    PAIR_ROOM_DIG     =   6,        /* fresh room cell — pink flash */
    PAIR_CORRIDOR_DIG =   7,        /* fresh corridor — gold flash  */
    PAIR_SUPERNOVA    =   8,        /* reset flash — yellow         */
};

/* Glow decay rates. Carve glows fast (instant feel), partition slower
 * (so the BSP outlines stay readable through the build phase). */
#define CARVE_GLOW_DECAY    3.5f
#define PART_GLOW_DECAY     1.5f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* Tile kinds. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELLS_MAX  (MAP_W_MAX * MAP_H_MAX)

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

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,         226, -1);
        init_pair(PAIR_HINT,         51, -1);
        init_pair(PAIR_WALL,        246, -1);   /* mid grey */
        init_pair(PAIR_FLOOR,        67, -1);   /* steel blue */
        init_pair(PAIR_PARTITION,   117, -1);   /* light cyan */
        init_pair(PAIR_ROOM_DIG,    213, -1);   /* pink */
        init_pair(PAIR_CORRIDOR_DIG,220, -1);   /* gold */
        init_pair(PAIR_SUPERNOVA,   226, -1);   /* yellow */
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_WALL,        COLOR_WHITE,   -1);
        init_pair(PAIR_FLOOR,       COLOR_BLUE,    -1);
        init_pair(PAIR_PARTITION,   COLOR_CYAN,    -1);
        init_pair(PAIR_ROOM_DIG,    COLOR_MAGENTA, -1);
        init_pair(PAIR_CORRIDOR_DIG,COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA,   COLOR_YELLOW,  -1);
    }
}

/* ===================================================================== */
/* §5  bsp                                                                */
/* ===================================================================== */

/*
 * BSPNode — one rectangular region in the partition tree.
 *
 *   x, y, w, h    : bounding box in map coordinates
 *   left, right   : indices into the nodes[] array; -1 for a leaf
 *   has_room      : true once a room has been placed in this leaf
 *   room_x..h     : room rectangle inside the leaf (only valid if leaf
 *                   AND has_room)
 *
 * Internal nodes (left != -1) don't have rooms of their own — rooms
 * live only in leaves. Internal nodes only matter for corridor planning.
 */
typedef struct {
    int  x, y, w, h;
    int  left, right;
    bool has_room;
    int  room_x, room_y, room_w, room_h;
} BSPNode;

/*
 * Dungeon — the whole simulation state.
 *
 * The state of the run is encoded by the Scene's enum (§6); this
 * struct is data only. Tile grid + BSP tree + the three work queues
 * (build, carve, corridor) live here.
 */
typedef struct {
    int      w, h;
    int      total_cells;
    uint8_t  tiles[CELLS_MAX];          /* TILE_WALL or TILE_FLOOR */
    float    carve_glow[CELLS_MAX];     /* per-cell carve flash */
    float    part_glow[CELLS_MAX];      /* per-cell partition outline glow */
    uint8_t  part_type[CELLS_MAX];      /* '+', '-', '|', or 0 (none) */
    float    supernova_glow[CELLS_MAX];

    BSPNode  nodes[MAX_NODES];
    int      n_nodes;
    int      root_idx;

    /* PARTITION-phase queue: indices of nodes still to consider for
     * splitting. Pop one per build step; if too small/random-stop,
     * leave as leaf; otherwise create children + push them. */
    int      build_queue[MAX_NODES];
    int      build_qhead, build_qtail;

    /* CARVE-phase queue: cell indices to set to TILE_FLOOR. Built
     * from leaf rooms when partition completes. */
    int      carve_queue[CELLS_MAX];
    int      carve_qhead, carve_qtail;

    /* CORRIDOR-phase queue: cell indices to set to TILE_FLOOR. Built
     * from L-corridors during corridor planning. */
    int      corridor_queue[CELLS_MAX];
    int      corridor_qhead, corridor_qtail;

    /* Stats for HUD. */
    int      leaf_count;
    int      room_count;
    int      corridor_count;
} Dungeon;

static inline int dun_idx(const Dungeon *d, int x, int y) { return y * d->w + x; }
static inline bool dun_in_bounds(const Dungeon *d, int x, int y)
{
    return x >= 0 && x < d->w && y >= 0 && y < d->h;
}

static void dun_reset(Dungeon *d, int w, int h)
{
    d->w = w;
    d->h = h;
    d->total_cells = w * h;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        d->tiles[i]          = TILE_WALL;
        d->carve_glow[i]     = 0.0f;
        d->part_glow[i]      = 0.0f;
        d->part_type[i]      = 0;
        d->supernova_glow[i] = 1.0f;
    }

    d->n_nodes = 0;
    d->root_idx = d->n_nodes++;
    d->nodes[d->root_idx] = (BSPNode){
        .x = 0, .y = 0, .w = w, .h = h,
        .left = -1, .right = -1, .has_room = false
    };

    d->build_qhead = 0;
    d->build_qtail = 0;
    d->build_queue[d->build_qtail++] = d->root_idx;

    d->carve_qhead    = 0;
    d->carve_qtail    = 0;
    d->corridor_qhead = 0;
    d->corridor_qtail = 0;

    d->leaf_count     = 1;     /* the root is a leaf */
    d->room_count     = 0;
    d->corridor_count = 0;
}

/*
 * dun_paint_outline — paint part_glow + per-cell glyph kind on the
 * boundary of node `idx`.
 *
 * Each outline cell is one of:
 *   '+' corner (one of the 4 leaf corners)
 *   '-' horizontal edge (top or bottom row, interior x)
 *   '|' vertical edge (left or right column, interior y)
 *
 * Adjacent leaves share edges; both leaves paint the same kind onto
 * the shared cell, so the type is consistent regardless of paint order.
 *
 * Called whenever a node is created so the BSP tree grows visibly in
 * cyan during the PARTITION phase.
 */
static void dun_paint_outline(Dungeon *d, int idx)
{
    BSPNode *n = &d->nodes[idx];
    int x0 = n->x;
    int y0 = n->y;
    int x1 = n->x + n->w - 1;
    int y1 = n->y + n->h - 1;
    if (!dun_in_bounds(d, x0, y0) || !dun_in_bounds(d, x1, y1)) return;

    /* Top + bottom rows: '+' on corners, '-' on the interior. */
    for (int x = x0; x <= x1; x++) {
        int top = dun_idx(d, x, y0);
        int bot = dun_idx(d, x, y1);
        d->part_glow[top] = 1.0f;
        d->part_glow[bot] = 1.0f;
        char kind = (x == x0 || x == x1) ? '+' : '-';
        d->part_type[top] = (uint8_t)kind;
        d->part_type[bot] = (uint8_t)kind;
    }
    /* Left + right columns: '+' on corners (already set above), '|' on
     * the interior. Don't overwrite '+' corners — only set if currently
     * empty or already '|'. */
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
 * dun_split_step — perform one split operation on the next queued node.
 *
 * Returns true if work happened (split or rejected as final leaf),
 * false if the build queue is empty (PARTITION phase done).
 *
 * Splitting decision:
 *   • If both dims < 2·MIN_LEAF  → can't split, mark as final leaf.
 *   • Else pick axis: longer dim biased, ties broken randomly.
 *   • Pick split position in [MIN_LEAF, dim − MIN_LEAF].
 *   • Create two child nodes; queue them.
 */
static bool dun_split_step(Dungeon *d)
{
    if (d->build_qhead >= d->build_qtail) return false;

    int idx = d->build_queue[d->build_qhead++];
    BSPNode *n = &d->nodes[idx];

    /* Refuse to split if both children would be < MIN_LEAF. */
    bool can_split_w = n->w >= 2 * MIN_LEAF_W;
    bool can_split_h = n->h >= 2 * MIN_LEAF_H;
    if (!can_split_w && !can_split_h) {
        return true;            /* leaf — leave as is */
    }

    /* Refuse if there's no room in the nodes[] array for two more. */
    if (d->n_nodes + 2 > MAX_NODES) {
        return true;
    }

    /* Pick axis. Prefer the longer dimension; bias slope is 1.4×. */
    bool vertical_cut;          /* vertical cut = split x, cut goes top-to-bottom */
    if (can_split_w && !can_split_h) {
        vertical_cut = true;
    } else if (can_split_h && !can_split_w) {
        vertical_cut = false;
    } else if (n->w * 10 > n->h * 14) {
        vertical_cut = true;
    } else if (n->h * 10 > n->w * 14) {
        vertical_cut = false;
    } else {
        vertical_cut = (rand() & 1);
    }

    int left_idx  = d->n_nodes++;
    int right_idx = d->n_nodes++;

    if (vertical_cut) {
        int min_pos = MIN_LEAF_W;
        int max_pos = n->w - MIN_LEAF_W;
        if (max_pos <= min_pos) {
            d->n_nodes -= 2;
            return true;
        }
        int split = min_pos + rand() % (max_pos - min_pos);
        d->nodes[left_idx]  = (BSPNode){
            .x = n->x,           .y = n->y,
            .w = split,          .h = n->h,
            .left = -1, .right = -1, .has_room = false
        };
        d->nodes[right_idx] = (BSPNode){
            .x = n->x + split,   .y = n->y,
            .w = n->w - split,   .h = n->h,
            .left = -1, .right = -1, .has_room = false
        };
    } else {
        int min_pos = MIN_LEAF_H;
        int max_pos = n->h - MIN_LEAF_H;
        if (max_pos <= min_pos) {
            d->n_nodes -= 2;
            return true;
        }
        int split = min_pos + rand() % (max_pos - min_pos);
        d->nodes[left_idx]  = (BSPNode){
            .x = n->x, .y = n->y,
            .w = n->w, .h = split,
            .left = -1, .right = -1, .has_room = false
        };
        d->nodes[right_idx] = (BSPNode){
            .x = n->x, .y = n->y + split,
            .w = n->w, .h = n->h - split,
            .left = -1, .right = -1, .has_room = false
        };
    }

    /* Hook children into parent (re-fetch n; nodes[] may have moved
     * if we relied on a different array — we don't, indices are stable). */
    d->nodes[idx].left  = left_idx;
    d->nodes[idx].right = right_idx;

    d->leaf_count += 1;         /* +2 children, -1 split parent → +1 net */

    /* Queue children for further splitting consideration. */
    d->build_queue[d->build_qtail++] = left_idx;
    d->build_queue[d->build_qtail++] = right_idx;

    /* Paint partition outlines for both new leaves so the BSP tree is
     * visible during the build phase. */
    dun_paint_outline(d, left_idx);
    dun_paint_outline(d, right_idx);

    return true;
}

/*
 * dun_place_rooms — at end of PARTITION phase, drop a room inside
 * every leaf and queue the room cells for animated carving.
 *
 * Room placement: random margin from each leaf edge in [0, MAX_ROOM_MARGIN].
 * Falls back to margin=1 / margin=0 when leaf is too tight.
 */
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

static void dun_place_rooms(Dungeon *d)
{
    for (int i = 0; i < d->n_nodes; i++) {
        BSPNode *n = &d->nodes[i];
        if (n->left != -1) continue;        /* internal node */

        int rw = dun_pick_room_dim(n->w, MIN_ROOM_W, MAX_ROOM_MARGIN);
        int rh = dun_pick_room_dim(n->h, MIN_ROOM_H, MAX_ROOM_MARGIN);
        if (rw > n->w - 2) rw = n->w - 2;
        if (rh > n->h - 2) rh = n->h - 2;
        if (rw < 1) rw = 1;
        if (rh < 1) rh = 1;

        int max_x_off = n->w - rw - 1;
        int max_y_off = n->h - rh - 1;
        if (max_x_off < 1) max_x_off = 1;
        if (max_y_off < 1) max_y_off = 1;
        int x_off = 1 + rand() % max_x_off;
        int y_off = 1 + rand() % max_y_off;

        n->room_x = n->x + x_off;
        n->room_y = n->y + y_off;
        n->room_w = rw;
        n->room_h = rh;
        n->has_room = true;
        d->room_count++;

        /* Queue the room cells for the CARVE phase, in raster order. */
        for (int ry = 0; ry < rh; ry++) {
            for (int rx = 0; rx < rw; rx++) {
                int cx = n->room_x + rx;
                int cy = n->room_y + ry;
                if (!dun_in_bounds(d, cx, cy)) continue;
                d->carve_queue[d->carve_qtail++] = dun_idx(d, cx, cy);
            }
        }
    }
}

/*
 * dun_carve_step — pop one cell from the carve queue, set TILE_FLOOR
 * with pink carve_glow flash. Returns true if a cell was carved.
 */
static bool dun_carve_step(Dungeon *d)
{
    if (d->carve_qhead >= d->carve_qtail) return false;
    int idx = d->carve_queue[d->carve_qhead++];
    d->tiles[idx]      = TILE_FLOOR;
    d->carve_glow[idx] = 1.0f;
    return true;
}

/*
 * dun_subtree_room_centre — return the centre (x, y) of a randomly
 * chosen room from the subtree rooted at `idx`. Recurses to a leaf.
 *
 * Used in corridor planning: at every internal node we connect a room
 * from the left subtree to a room from the right subtree.
 */
static void dun_subtree_room_centre(const Dungeon *d, int idx,
                                    int *out_x, int *out_y)
{
    int cur = idx;
    while (d->nodes[cur].left != -1) {
        cur = (rand() & 1) ? d->nodes[cur].left : d->nodes[cur].right;
    }
    const BSPNode *n = &d->nodes[cur];
    *out_x = n->room_x + n->room_w / 2;
    *out_y = n->room_y + n->room_h / 2;
}

/*
 * dun_queue_corridor_segment — append every cell along the axis-aligned
 * segment from (x1,y1) to (x2,y2) to the corridor queue, in order.
 *
 * The segment must be axis-aligned (either x1==x2 or y1==y2). Step
 * direction is computed from the sign of the delta.
 */
static void dun_queue_corridor_segment(Dungeon *d,
                                       int x1, int y1, int x2, int y2)
{
    int dx = (x2 > x1) ? 1 : (x2 < x1) ? -1 : 0;
    int dy = (y2 > y1) ? 1 : (y2 < y1) ? -1 : 0;
    int x = x1, y = y1;
    while (1) {
        if (dun_in_bounds(d, x, y)) {
            d->corridor_queue[d->corridor_qtail++] = dun_idx(d, x, y);
        }
        if (x == x2 && y == y2) break;
        x += dx;
        y += dy;
    }
}

/*
 * dun_plan_corridors — post-order traversal of the BSP tree; at every
 * internal node, queue an L-corridor between a left-subtree room and
 * a right-subtree room. Two segments per L: random of (h-then-v) or
 * (v-then-h) for visual variety.
 */
static void dun_plan_corridors(Dungeon *d, int idx)
{
    const BSPNode *n = &d->nodes[idx];
    if (n->left == -1) return;

    dun_plan_corridors(d, n->left);
    dun_plan_corridors(d, n->right);

    int x1, y1, x2, y2;
    dun_subtree_room_centre(d, n->left,  &x1, &y1);
    dun_subtree_room_centre(d, n->right, &x2, &y2);

    if (rand() & 1) {
        dun_queue_corridor_segment(d, x1, y1, x2, y1);
        dun_queue_corridor_segment(d, x2, y1, x2, y2);
    } else {
        dun_queue_corridor_segment(d, x1, y1, x1, y2);
        dun_queue_corridor_segment(d, x1, y2, x2, y2);
    }
    d->corridor_count++;
}

/*
 * dun_corridor_step — pop one cell from the corridor queue, set
 * TILE_FLOOR with gold carve_glow. Returns true if a cell was set.
 */
static bool dun_corridor_step(Dungeon *d)
{
    if (d->corridor_qhead >= d->corridor_qtail) return false;
    int idx = d->corridor_queue[d->corridor_qhead++];
    d->tiles[idx]      = TILE_FLOOR;
    /* Use a NEGATIVE-sign trick on carve_glow to mark "this was a
     * corridor"? No — simpler: the renderer reads the carve_glow
     * and the phase to choose color. We track corridor cells via the
     * Scene's current state (CORRIDORS) when painting glow, so renders
     * during this phase use gold. After the glow fades the cell shows
     * as plain floor (no phase distinction needed). */
    d->carve_glow[idx] = 1.0f;
    return true;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   PARTITION   — split nodes one per build step. Transitions to CARVE
 *                 when build queue empties (and dun_place_rooms is run).
 *   CARVE       — pop room cells from carve_queue, set TILE_FLOOR.
 *                 Transitions to CORRIDORS when carve queue empties
 *                 (and dun_plan_corridors is run from root).
 *   CORRIDORS   — pop corridor cells, set TILE_FLOOR. Transitions to
 *                 HOLD when corridor queue empties.
 *   HOLD        — wait HOLD_SECONDS then dun_reset and back to PARTITION.
 *
 * Carve_glow gets coloured differently depending on the active phase
 * (pink in CARVE, gold in CORRIDORS). The Scene state IS the phase,
 * so the renderer reads s->state to pick the right pair.
 */
typedef enum {
    SCENE_PARTITION = 0,
    SCENE_CARVE     = 1,
    SCENE_CORRIDORS = 2,
    SCENE_HOLD      = 3,
} SceneState;

typedef struct {
    Dungeon     d;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         build_per_tick;
    int         carve_per_tick;
    int         corridor_per_tick;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    dun_reset(&s->d, mw, mh);
    s->state      = SCENE_PARTITION;
    s->hold_timer = 0.0f;
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

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay every glow buffer. */
    float carve_d = expf(-CARVE_GLOW_DECAY * dt);
    float part_d  = expf(-PART_GLOW_DECAY  * dt);
    float nova_d  = expf(-SUPERNOVA_DECAY  * dt);
    int n = s->d.total_cells;
    for (int i = 0; i < n; i++) {
        s->d.carve_glow[i]     *= carve_d;
        s->d.part_glow[i]      *= part_d;
        s->d.supernova_glow[i] *= nova_d;
    }

    switch (s->state) {

    case SCENE_PARTITION:
        for (int i = 0; i < s->build_per_tick; i++) {
            if (!dun_split_step(&s->d)) {
                /* Partition done — place rooms then move to CARVE. */
                dun_place_rooms(&s->d);
                s->state = SCENE_CARVE;
                break;
            }
        }
        break;

    case SCENE_CARVE:
        for (int i = 0; i < s->carve_per_tick; i++) {
            if (!dun_carve_step(&s->d)) {
                /* Carve done — plan corridors and move to CORRIDORS. */
                dun_plan_corridors(&s->d, s->d.root_idx);
                s->state = SCENE_CORRIDORS;
                break;
            }
        }
        break;

    case SCENE_CORRIDORS:
        for (int i = 0; i < s->corridor_per_tick; i++) {
            if (!dun_corridor_step(&s->d)) {
                s->state = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->d.w, s->d.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Same canonical pattern as other procedural files §7 / framework.c:
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate
 *
 * ASCII glyphs only:
 *   '#' wall (rendered only where adjacent to floor — interior void
 *       stays blank for the classic roguelike aesthetic),
 *   '.' floor,
 *   '+' partition outline corner,
 *   '-' partition outline horizontal,
 *   '|' partition outline vertical.
 *
 * Per-cell glow priority (highest wins):
 *   supernova_glow > carve_glow > part_glow > base
 *   The carve_glow color depends on the CURRENT scene state (pink for
 *   CARVE, gold for CORRIDORS) so the same glow buffer doubles for
 *   both phases.
 */
typedef struct { int cols, rows; } Screen;

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
 * tile_has_floor_neighbour — does any 8-connected neighbour of (x,y)
 * have TILE_FLOOR? Used to decide whether a wall cell should render
 * as '#' (visible — borders a floor) or as ' ' (interior void).
 *
 * We use 8-connected so diagonal floors also bring out the wall —
 * this gives the cleaner roguelike outline (every wall touching a
 * room is drawn, not just orthogonal neighbours).
 */
static bool tile_has_floor_neighbour(const Dungeon *d, int x, int y)
{
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (!dun_in_bounds(d, nx, ny)) continue;
            if (d->tiles[dun_idx(d, nx, ny)] == TILE_FLOOR) return true;
        }
    }
    return false;
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Dungeon *d = &s->d;

    /* Center the map in the available area (rows-2 leaving HUD margins). */
    int gx0 = (cols - d->w) / 2;
    int gy0 = ((rows - 2) - d->h) / 2 + 1;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 1) gy0 = 1;

    int carve_pair = (s->state == SCENE_CORRIDORS) ? PAIR_CORRIDOR_DIG
                                                   : PAIR_ROOM_DIG;

    for (int y = 0; y < d->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < d->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = dun_idx(d, x, y);
            float ng  = d->supernova_glow[idx];
            float cg  = d->carve_glow[idx];
            float pg  = d->part_glow[idx];
            uint8_t k = d->tiles[idx];

            int pair = -1, attr = A_NORMAL;
            char glyph = ' ';

            if (ng > GLOW_THRESHOLD) {
                /* Supernova: bright yellow '*' across the grid. */
                pair = PAIR_SUPERNOVA;
                attr = A_BOLD;
                glyph = '*';
            } else if (cg > GLOW_THRESHOLD) {
                /* Fresh-cut floor — pink (room) or gold (corridor)
                 * depending on the active phase. */
                pair  = carve_pair;
                attr  = A_BOLD;
                glyph = (k == TILE_FLOOR) ? '.' : '#';
            } else if (pg > GLOW_THRESHOLD && k == TILE_WALL
                       && d->part_type[idx] != 0) {
                /* Partition outline — '+', '-', or '|' from part_type.
                 * Only painted on walls so it doesn't fight with the
                 * carved floor cells. */
                pair  = PAIR_PARTITION;
                attr  = A_BOLD;
                glyph = (char)d->part_type[idx];
            } else if (k == TILE_FLOOR) {
                pair  = PAIR_FLOOR;
                attr  = A_DIM;
                glyph = '.';
            } else {
                /* Wall — draw '#' only if a floor cell is nearby
                 * (8-connected). Pure interior void stays blank. */
                if (tile_has_floor_neighbour(d, x, y)) {
                    pair  = PAIR_WALL;
                    attr  = A_NORMAL;
                    glyph = '#';
                } else {
                    continue;   /* skip — leave blank */
                }
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Dungeon *d = &s->d;
    const char *state_str =
        s->paused                         ? "PAUSED   " :
        (s->state == SCENE_PARTITION)     ? "PARTITION" :
        (s->state == SCENE_CARVE)         ? "CARVING  " :
        (s->state == SCENE_CORRIDORS)     ? "CORRIDOR " :
                                            "HOLD     ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  rooms:%-3d  cor:%-3d ",
             fps, sim_fps, state_str, d->room_count, d->corridor_count);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " BSP DUNGEON ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " #:wall  .:floor  +:partition  pink:room-dig  gold:corridor "
             " q:quit  spc:pause  r:reset  +/-:speed ");
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
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->build_per_tick    < BUILD_STEPS_MAX) s->build_per_tick    *= 2;
        if (s->build_per_tick    > BUILD_STEPS_MAX) s->build_per_tick     = BUILD_STEPS_MAX;
        s->carve_per_tick    *= 2;
        s->corridor_per_tick *= 2;
        if (s->carve_per_tick    > 256) s->carve_per_tick    = 256;
        if (s->corridor_per_tick > 256) s->corridor_per_tick = 256;
        break;
    case '-':
        s->build_per_tick    /= 2;
        s->carve_per_tick    /= 2;
        s->corridor_per_tick /= 2;
        if (s->build_per_tick    < BUILD_STEPS_MIN) s->build_per_tick    = BUILD_STEPS_MIN;
        if (s->carve_per_tick    < 1)               s->carve_per_tick    = 1;
        if (s->corridor_per_tick < 1)               s->corridor_per_tick = 1;
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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
