/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * quad_tree_helloworld.c — animated quadtree: insert points, watch a full
 * leaf split into four quadrants, then sweep a query rectangle and light up
 * the points inside it. Companion to algorithms/quadtree.c (plain CLI version).
 *
 * Quadtree: Finkel & Bentley, "Quad trees", Acta Informatica 4(1), 1974.
 * Keys: q/ESC quit, space pause, n next/reset, r reset, [/] sim Hz.
 * Build: gcc -std=c11 -O2 -Wall -Wextra quad_tree_helloworld.c -o qt_hello -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — all tunable constants, named so no magic numbers leak ── */

/* simulation loop pacing */
enum {
    SIM_HZ_MIN     = 10,
    SIM_HZ_DEFAULT = 60,
    SIM_HZ_MAX     = 120,
    SIM_HZ_STEP    = 10,
    FPS_WINDOW_MS  = 500,   /* fps averaged over this sliding window     */
};

/* quadtree shape limits */
enum {
    LEAF_CAPACITY  = 3,   /* a leaf holding this many points splits on the next one */
    MAX_TREE_DEPTH = 5,   /* root is depth 0; refuse to split past this (coincident points) */
    NODE_POOL_SIZE = 512, /* fixed node store; 30 points can't make more than ~200 nodes */
};

/* demo pacing */
enum {
    DEMO_POINT_COUNT  = 30,              /* total points inserted in phase 1 */
    QUERY_RESULT_CAP  = DEMO_POINT_COUNT,/* upper bound on range-query hits  */
};
#define SECONDS_PER_INSERTION  0.42f    /* insert one new point every ~0.4 s */
#define QUERY_PHASE_DURATION   22.0f    /* seconds before auto-reset          */

/* query rectangle (sizes are fractions of the [0,1] tree space) */
#define QUERY_RECT_WIDTH    0.28f
#define QUERY_RECT_HEIGHT   0.28f
#define QUERY_DRIFT_SPEED   0.18f   /* tree-space units per second */

/* screen layout: yellow data band on top, cyan key-hint band on bottom,
 * tree canvas + info panel between them */
enum {
    INFO_PANEL_COLS = 28,  /* right-side panel width in terminal columns */
    HUD_TOP_ROWS    =  2,  /* row 0: live status  ;  row 1: static config */
    HUD_BOT_ROWS    =  1,  /* last row: key-hint action bar               */
};

/* color-pair names (the actual colors are set in color_init).
 * HUD bars are kept separate from the panel so each can be retinted alone. */
enum {
    CP_ROOT_BORDER =  1,   /* depth-0 node border       white, bold       */
    CP_D1_BORDER   =  2,   /* depth-1 node border       cyan              */
    CP_D2_BORDER   =  3,   /* depth-2 node border       blue              */
    CP_DEEP_BORDER =  4,   /* depth 3+ node border      grey, dim         */
    CP_POINT_IDLE  =  5,   /* ordinary inserted point   yellow            */
    CP_POINT_HIT   =  6,   /* point inside query rect   green, bold       */
    CP_QUERY_BOX   =  7,   /* animated query rectangle  orange            */
    CP_PANEL       =  8,   /* right-side info panel     gold              */
    CP_HUD         =  9,   /* top data bar              bright yellow + A_BOLD */
    CP_HINT        = 10,   /* bottom action bar         bright cyan   + A_BOLD */
};

/* timing helpers */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(hz)  (NS_PER_SEC / (hz))

/* ── §2 clock — monotonic time source and a sleep helper ── */

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

/* ── §3 color — set up the color pairs (256-color, with 8-color fallback) ── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_ROOT_BORDER, 255, COLOR_BLACK);  /* white            */
        init_pair(CP_D1_BORDER,    51, COLOR_BLACK);  /* cyan             */
        init_pair(CP_D2_BORDER,    33, COLOR_BLACK);  /* blue             */
        init_pair(CP_DEEP_BORDER, 240, COLOR_BLACK);  /* grey             */
        init_pair(CP_POINT_IDLE,  226, COLOR_BLACK);  /* yellow           */
        init_pair(CP_POINT_HIT,    46, COLOR_BLACK);  /* green            */
        init_pair(CP_QUERY_BOX,   208, COLOR_BLACK);  /* orange           */
        init_pair(CP_PANEL,       220, COLOR_BLACK);  /* gold             */
        init_pair(CP_HUD,         226, COLOR_BLACK);  /* bright yellow    */
        init_pair(CP_HINT,         51, COLOR_BLACK);  /* bright cyan      */
    } else {
        init_pair(CP_ROOT_BORDER, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_D1_BORDER,   COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_D2_BORDER,   COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_DEEP_BORDER, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_POINT_IDLE,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_POINT_HIT,   COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_QUERY_BOX,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_PANEL,       COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_HUD,         COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_HINT,        COLOR_CYAN,    COLOR_BLACK);
    }
}

/* ── §4 canvas — the terminal region the tree draws into, with clipped writes ── */

/*
 * TreeCanvas — the box of terminal cells the tree is allowed to draw in:
 * left of the info panel, below the top HUD, above the bottom HUD. Computed
 * once per frame by canvas_make and read by every draw helper, so the
 * clipping rule and the cols/rows -> cell mapping live in exactly one place.
 * Passed by value (it's tiny). A resize just re-runs canvas_make next frame.
 *
 * Bounds are half-open (the "one-past-last" fields are NOT writable), which
 * matches `for (r = top; r < bottom; r++)` and makes the height a clean
 * subtraction with no off-by-one.
 *
 * Members:
 *   top_row     first writable row (inclusive)     = HUD_TOP_ROWS
 *   bottom_row  one past the last writable row      = rows - HUD_BOT_ROWS
 *   right_col   one past the last writable column   = cols - INFO_PANEL_COLS
 *               (canvas_make floors bottom/right so a tiny terminal still
 *                leaves a non-empty area to draw into)
 * Map a normalised point (nx, ny) in [0,1] to a cell via canvas_col/canvas_row.
 */
typedef struct {
    int top_row;    /* first writable row (inclusive)        */
    int bottom_row; /* one past the last writable row        */
    int right_col;  /* one past the last writable column     */
} TreeCanvas;

static TreeCanvas canvas_make(int terminal_cols, int terminal_rows)
{
    int right = terminal_cols - INFO_PANEL_COLS;
    int bot   = terminal_rows - HUD_BOT_ROWS;
    return (TreeCanvas){
        .top_row    = HUD_TOP_ROWS,
        .bottom_row = bot   > HUD_TOP_ROWS + 1 ? bot   : HUD_TOP_ROWS + 2,
        .right_col  = right > 1                 ? right : 2,
    };
}

/* Map normalised x ∈ [0,1] → terminal column inside the tree area */
static inline int canvas_col(TreeCanvas cv, float nx)
{
    return (int)(nx * (float)cv.right_col);
}

/* Map normalised y ∈ [0,1] → terminal row inside the tree area */
static inline int canvas_row(TreeCanvas cv, float ny)
{
    return cv.top_row + (int)(ny * (float)(cv.bottom_row - cv.top_row));
}

/* Draw one character, clipped to the tree area */
static void canvas_put(TreeCanvas cv, int row, int col, chtype ch)
{
    if (row >= cv.top_row && row < cv.bottom_row
     && col >= 0          && col < cv.right_col)
        mvaddch(row, col, ch);
}

/* ── §5 quadtree — point/rect types, the node pool, insert, query, stats ── */

/* geometry types */

/*
 * Vec2 — one point the tree stores, in normalised [0,1] x [0,1] space.
 * Coordinates are floats in [0,1] rather than screen cells so the tree
 * survives terminal resizes untouched: only the [0,1] -> cell mapping
 * changes, never the data. Floats (not ints) because subdivision keeps
 * halving the range, so deep nodes need sub-cell precision.
 *
 * y grows DOWNWARD (terminal convention), so small y = top, large y = bottom
 * — this lines up with the NW/SW child naming.
 *
 * Members:
 *   x  horizontal, 0..1, left to right
 *   y  vertical,   0..1, top to bottom
 */
typedef struct {
    float x;     /* horizontal, 0..1, left to right */
    float y;     /* vertical,   0..1, top to bottom */
} Vec2;

/*
 * Rect — an axis-aligned box in [0,1] tree space. It's both a node's region
 * (fixed once the node is created) and the moving query rectangle.
 *
 * A rect covers [x, x+w) x [y, y+h) — the right and bottom edges are NOT
 * included. That half-open rule is what makes a split clean: the four
 * children tile the parent with no cell shared and no cell missed. If edges
 * were inclusive, the midpoint cells would land in two children at once.
 * Stored as (x, y, w, h) rather than two corners so splitting just halves
 * w and h.
 *
 * Members:
 *   x, y   top-left corner (included), 0..1
 *   w, h   width and height; covers [x, x+w) x [y, y+h)
 * w and h are >= 0 and halve at every level of depth; at MAX_TREE_DEPTH a
 * full leaf keeps overflowing instead of splitting into zero-size children.
 */
typedef struct {
    float x;     /* left  edge (included), 0..1 */
    float y;     /* top   edge (included), 0..1 */
    float w;     /* width;  covers [x, x+w)     */
    float h;     /* height; covers [y, y+h)     */
} Rect;

/* Is (px, py) inside r? Half-open edges so touching rects don't both claim it. */
static inline bool rect_contains_point(Rect r, float px, float py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/* True when rectangles a and b share any area */
static inline bool rect_overlaps(Rect a, Rect b)
{
    return !(b.x       > a.x + a.w   /* b entirely right of a */
          || b.x + b.w < a.x         /* b entirely left of a  */
          || b.y       > a.y + a.h   /* b entirely below a    */
          || b.y + b.h < a.y);       /* b entirely above a    */
}

/* quadtree node */

/* Names for the four child slots, clearer than raw 0..3. */
enum { NW = 0, NE = 1, SW = 2, SE = 3, NUM_CHILDREN = 4 };
#define NO_CHILD  (-1)   /* an empty child slot (so a node with NW empty is a leaf) */

/*
 * QuadrantRects — the four sub-rectangles a split produces, returned as one
 * value. Field order (nw/ne/sw/se) matches the NW/NE/SW/SE child indices so
 * subdivide's allocation lines pair up by name.
 *
 * Members: the parent's four quadrants, top-left/top-right/bottom-left/bottom-right.
 */
typedef struct {
    Rect nw;    /* top-left     quadrant */
    Rect ne;    /* top-right    quadrant */
    Rect sw;    /* bottom-left  quadrant */
    Rect se;    /* bottom-right quadrant */
} QuadrantRects;

/* Cut r in half on each axis into its four quadrants. Half-open edges mean
 * every cell of r lands in exactly one quadrant. */
static QuadrantRects compute_quadrant_rects(Rect r)
{
    float half_w = r.w * 0.5f;
    float half_h = r.h * 0.5f;
    float mid_x  = r.x + half_w;
    float mid_y  = r.y + half_h;
    return (QuadrantRects){
        .nw = { r.x,   r.y,   half_w, half_h },
        .ne = { mid_x, r.y,   half_w, half_h },
        .sw = { r.x,   mid_y, half_w, half_h },
        .se = { mid_x, mid_y, half_w, half_h },
    };
}

/*
 * QuadNode — one node of the tree. A node owns a region and is either a LEAF
 * (holds up to LEAF_CAPACITY points, no children) or INTERNAL (four children,
 * no points of its own). A leaf turns internal once subdivide() splits it.
 *
 * There's no is_leaf flag: "children[NW] is empty" already tells you it's a
 * leaf (node_is_leaf names that check). Storing points and children in one
 * struct keeps allocation to a single path — the wasted points[] buffer on
 * internal nodes is a few KB at most, not worth a tagged union.
 *
 * Children are POOL INDICES, not pointers: the pool lives inside Scene on the
 * stack, so an address could move between runs, but an index never does.
 *
 * Going leaf -> internal is one-way; this demo never merges children back.
 *
 * Members:
 *   bounds                  the [0,1] region this node owns; fixed for life
 *   points[LEAF_CAPACITY]   stored points, entries [0, point_count) valid;
 *                           only meaningful on a leaf
 *   point_count             0..LEAF_CAPACITY on a leaf; always 0 internal
 *   children[NUM_CHILDREN]  NW/NE/SW/SE indices into the pool, or NO_CHILD;
 *                           all empty <=> leaf, all set <=> internal
 *   depth                   0 at the root, +1 per level; caps splitting at
 *                           MAX_TREE_DEPTH and picks the border color
 */
typedef struct {
    Rect bounds;                    /* the [0,1] region this node owns      */
    Vec2 points[LEAF_CAPACITY];     /* stored points (only valid on a leaf) */
    int  point_count;               /* 0..LEAF_CAPACITY on a leaf; 0 internal */
    int  children[NUM_CHILDREN];    /* NW/NE/SW/SE pool indices, or NO_CHILD */
    int  depth;                     /* 0 = root; at most MAX_TREE_DEPTH      */
} QuadNode;

static inline bool node_is_leaf(const QuadNode *n) { return n->children[NW] == NO_CHILD; }
static inline bool node_is_full(const QuadNode *n) { return n->point_count == LEAF_CAPACITY; }

/* node pool */

/*
 * NodePool — one fixed array holding every node the tree can ever use, so
 * there's no malloc/free in the loop. Allocation is a bump: hand out the
 * next slot and increment count. Reset is just count := 0 plus a new root,
 * so there's nothing per-node to free. All nodes sit contiguously, which is
 * also cache-friendly for the recursive walks.
 *
 * Members:
 *   nodes[NODE_POOL_SIZE]   the storage; entries [0, count) are live
 *   count                   next free slot, also the live-node count
 *   root_idx                index of the root (0 after a reset, but stored
 *                           so callers don't have to assume that)
 */
typedef struct {
    QuadNode nodes[NODE_POOL_SIZE];
    int      count;       /* next free slot, also the live-node count     */
    int      root_idx;    /* index of the root; 0 after pool_reset        */
} NodePool;

/* Will `slots_needed` more nodes fit? subdivide needs 4 at once. */
static bool pool_has_room(const NodePool *pool, int slots_needed)
{
    return pool->count + slots_needed <= NODE_POOL_SIZE;
}

/* Hand out the next slot as a fresh empty leaf, or NO_CHILD if the pool is full. */
static int node_alloc(NodePool *pool, Rect region, int depth)
{
    if (pool->count >= NODE_POOL_SIZE) return NO_CHILD;
    int idx = pool->count++;
    pool->nodes[idx] = (QuadNode){
        .bounds      = region,
        .point_count = 0,
        .children    = { NO_CHILD, NO_CHILD, NO_CHILD, NO_CHILD },
        .depth       = depth,
    };
    return idx;
}

/* Throw the whole tree away and start with a fresh root over all of [0,1].
 * Doesn't wipe memory — stale nodes are just overwritten as they're reused. */
static void pool_reset(NodePool *pool)
{
    pool->count    = 0;
    Rect full_space = { .x = 0.0f, .y = 0.0f, .w = 1.0f, .h = 1.0f };
    pool->root_idx = node_alloc(pool, full_space, /*depth=*/0);
}

/* core operations */

/* subdivide calls tree_insert when re-homing the parent's old points */
static bool tree_insert(NodePool *pool, int node_idx, float x, float y);

/*
 * Move the parent's points down into its new children. We copy them out
 * first because re-inserting will overwrite the child storage we're walking.
 * Exactly one child's region contains each point, so the first insert that
 * accepts it wins and we stop.
 */
static void redistribute_points_into_children(NodePool *pool, int parent_idx)
{
    QuadNode *parent  = &pool->nodes[parent_idx];
    int       old_count = parent->point_count;

    Vec2 displaced[LEAF_CAPACITY];
    memcpy(displaced, parent->points, (size_t)old_count * sizeof(Vec2));
    parent->point_count = 0;

    for (int i = 0; i < old_count; i++)
        for (int c = 0; c < NUM_CHILDREN; c++)
            if (tree_insert(pool, parent->children[c],
                            displaced[i].x, displaced[i].y))
                break;
}

/* Split a full leaf: create its four quadrant children, then push the
 * parent's old points down into them. Afterwards the parent is internal.
 * Caller must have checked the pool has room for 4 more nodes. */
static void subdivide(NodePool *pool, int parent_idx)
{
    QuadrantRects q = compute_quadrant_rects(pool->nodes[parent_idx].bounds);
    int next_depth = pool->nodes[parent_idx].depth + 1;

    QuadNode *parent = &pool->nodes[parent_idx];
    parent->children[NW] = node_alloc(pool, q.nw, next_depth);
    parent->children[NE] = node_alloc(pool, q.ne, next_depth);
    parent->children[SW] = node_alloc(pool, q.sw, next_depth);
    parent->children[SE] = node_alloc(pool, q.se, next_depth);

    redistribute_points_into_children(pool, parent_idx);
}

/* Store (x, y) in this leaf if there's room. Returns false when the leaf is
 * full, which is the caller's cue to subdivide. Caller must pass a leaf. */
static bool try_append_to_leaf(QuadNode *node, float x, float y)
{
    if (node_is_full(node)) return false;
    node->points[node->point_count++] = (Vec2){x, y};
    return true;
}

/* Hand (x, y) to whichever child contains it. Exactly one will, so the
 * first child that accepts wins; reaching the end means a broken invariant.
 * Caller must pass an internal node. */
static bool route_into_quadrant_child(NodePool *pool, QuadNode *node,
                                      float x, float y)
{
    for (int c = 0; c < NUM_CHILDREN; c++)
        if (tree_insert(pool, node->children[c], x, y))
            return true;
    return false;
}

/*
 * Add point (x, y) under node_idx. Reject it if it's outside this node's
 * region (false). On a leaf with room, store it. On a full leaf, split and
 * descend. On an internal node, hand it to the right child.
 *
 * Returns false for out-of-bounds OR when we hit the depth/pool limit. The
 * out-of-bounds false is also what lets redistribution try all four children
 * and let the one that fits accept the point.
 */
static bool tree_insert(NodePool *pool, int node_idx, float x, float y)
{
    if (node_idx == NO_CHILD)                            return false;

    QuadNode *node = &pool->nodes[node_idx];
    if (!rect_contains_point(node->bounds, x, y))        return false;

    if (node_is_leaf(node)) {
        if (try_append_to_leaf(node, x, y))              return true;
        if (node->depth >= MAX_TREE_DEPTH
         || !pool_has_room(pool, NUM_CHILDREN))          return false;

        subdivide(pool, node_idx);
        node = &pool->nodes[node_idx];   /* refetch after potential move */
        /* fall through: node is now internal */
    }

    return route_into_quadrant_child(pool, node, x, y);
}

/*
 * Collect every stored point that falls inside search_area into results[].
 * The key move (and why a quadtree beats a flat scan): if a node's region
 * doesn't even touch the search rectangle, its entire subtree is skipped.
 */
static void tree_query(const NodePool *pool, int node_idx, Rect search_area,
                       Vec2 *results, int *result_count, int result_cap)
{
    if (node_idx == NO_CHILD || *result_count >= result_cap) return;

    const QuadNode *node = &pool->nodes[node_idx];

    if (!rect_overlaps(node->bounds, search_area)) return;     /* prune */

    for (int i = 0; i < node->point_count && *result_count < result_cap; i++) {
        Vec2 p = node->points[i];
        if (rect_contains_point(search_area, p.x, p.y))
            results[(*result_count)++] = p;
    }

    for (int c = 0; c < NUM_CHILDREN; c++)
        tree_query(pool, node->children[c], search_area,
                   results, result_count, result_cap);
}

/* stats for the HUD: walk the tree to count nodes / find deepest leaf */

static int tree_total_nodes(const NodePool *pool, int node_idx)
{
    if (node_idx == NO_CHILD) return 0;
    int count = 1;
    for (int c = 0; c < NUM_CHILDREN; c++)
        count += tree_total_nodes(pool, pool->nodes[node_idx].children[c]);
    return count;
}

static int tree_current_depth(const NodePool *pool, int node_idx)
{
    if (node_idx == NO_CHILD) return -1;
    if (node_is_leaf(&pool->nodes[node_idx]))
        return pool->nodes[node_idx].depth;
    int deepest = 0;
    for (int c = 0; c < NUM_CHILDREN; c++) {
        int d = tree_current_depth(pool, pool->nodes[node_idx].children[c]);
        if (d > deepest) deepest = d;
    }
    return deepest;
}

/* ── §6 scene — demo state, the two-phase loop, and all drawing ── */

/* which half of the demo we're in */

/*
 * DemoPhase — the demo loops forever between two phases so you see both
 * operations animated: PHASE_INSERT grows the tree one point at a time,
 * PHASE_QUERY bounces the search rectangle and refreshes the matches.
 * Insert flips to query once DEMO_POINT_COUNT points are in; query flips
 * back to a fresh insert after QUERY_PHASE_DURATION. 'n' and 'r' jump
 * manually along the same edges.
 */
typedef enum {
    PHASE_INSERT,   /* points stream in; leaves split as they fill */
    PHASE_QUERY,    /* the orange rectangle bounces and lights up hits */
} DemoPhase;

/* the moving search rectangle */

/*
 * QueryBox — the search rectangle that bounces around during PHASE_QUERY,
 * bundling its position/size with its velocity. advance_query_box moves it
 * each tick; tree_query reads bounds to find the points inside it.
 *
 * It's a plain bounce: a velocity component flips sign when the box hits a
 * wall of [0,1]. The two axes are kept as separate floats (not a Vec2) on
 * purpose, since each wall reflects only one of them — separate fields make
 * a "flip both by mistake" bug harder.
 *
 * Members:
 *   bounds    position + size; w/h stay fixed, only x/y move
 *   drift_x   horizontal speed, tree-units/sec; sign flips on left/right wall
 *   drift_y   vertical speed,   tree-units/sec; sign flips on top/bottom wall
 */
typedef struct {
    Rect  bounds;    /* position + size; only x/y move */
    float drift_x;   /* horizontal speed, tree-units/sec */
    float drift_y;   /* vertical speed,   tree-units/sec */
} QueryBox;

/* the whole demo's state */

/*
 * Scene — all of the demo's state for one run, held in main() and passed by
 * pointer everywhere. The only globals are the signal flags in §8, because a
 * signal handler can't be handed a pointer.
 *
 * Members:
 *   pool                 the tree itself (every node lives here)
 *   phase                INSERT or QUERY
 *   time_in_phase        seconds since the current phase started
 *   next_insert_in       seconds until the next point drops (insert phase)
 *   points_inserted      how many points have landed so far
 *   query                the bouncing search rectangle
 *   query_results[]      points the last query found
 *   query_result_count   how many of those are valid
 *   paused               when true, scene_tick does nothing
 *   sim_hz               target tick rate, changed with [ and ]
 *   rows, cols           this frame's terminal size
 */
typedef struct {
    /* Tree */
    NodePool  pool;

    /* Demo state machine */
    DemoPhase phase;
    float     time_in_phase;    /* seconds since this phase began         */
    float     next_insert_in;   /* seconds until the next point is added  */
    int       points_inserted;  /* how many points have been inserted     */

    /* Query box + last-tick results */
    QueryBox  query;
    Vec2      query_results[QUERY_RESULT_CAP];
    int       query_result_count;

    /* UI / pacing */
    bool      paused;
    int       sim_hz;

    /* Terminal extent */
    int       rows;
    int       cols;
} Scene;

/* info-panel helpers */

/*
 * PanelWriter — a write-cursor for the right-side info panel. Each
 * panel_text/divider/blank call prints at current_row and steps it down, so
 * the caller just lists lines top-to-bottom without tracking row numbers.
 * Past last_row the helpers quietly do nothing, so a short terminal shows a
 * truncated panel instead of crashing.
 *
 * Members:
 *   panel_col     left edge of the panel, in terminal columns
 *   current_row   next row to print on; bumped by every helper
 *   last_row      one past the last writable row; helpers stop here
 */
typedef struct {
    int panel_col;    /* left edge of the panel, in columns      */
    int current_row;  /* next row to print on                    */
    int last_row;     /* one past the last writable row          */
} PanelWriter;

static PanelWriter panel_begin(int terminal_cols, int terminal_rows)
{
    return (PanelWriter){
        .panel_col   = terminal_cols - INFO_PANEL_COLS,
        .current_row = HUD_TOP_ROWS,
        .last_row    = terminal_rows - HUD_BOT_ROWS - 1,
    };
}

/* Print one line of text with given color pair and attribute */
static void panel_text(PanelWriter *pw, int color_pair, attr_t attr,
                       const char *fmt, ...)
{
    if (pw->current_row >= pw->last_row) return;
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    attron(COLOR_PAIR(color_pair) | attr);
    mvprintw(pw->current_row++, pw->panel_col, "%s", buf);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* Draw a full-width horizontal divider line */
static void panel_divider(PanelWriter *pw, int terminal_cols)
{
    if (pw->current_row >= pw->last_row) return;
    attron(COLOR_PAIR(CP_DEEP_BORDER) | A_DIM);
    for (int c = pw->panel_col; c < terminal_cols; c++)
        mvaddch(pw->current_row, c, '-');
    attroff(COLOR_PAIR(CP_DEEP_BORDER) | A_DIM);
    pw->current_row++;
}

/* Advance one blank row */
static void panel_blank(PanelWriter *pw)
{
    pw->current_row++;
}

/* node border colors */

/* Color a node's border by depth so the nesting is visible at a glance:
 * root white, depth 1 cyan, depth 2 blue, deeper grey. */
static int border_color_for_depth(int depth)
{
    static const int pair[] = {
        CP_ROOT_BORDER,   /* depth 0 */
        CP_D1_BORDER,     /* depth 1 */
        CP_D2_BORDER,     /* depth 2 */
        CP_DEEP_BORDER,   /* depth 3+ */
    };
    int clamped = depth < 3 ? depth : 3;
    return pair[clamped];
}

static attr_t border_intensity_for_depth(int depth)
{
    if (depth == 0) return A_BOLD;    /* root stands out                 */
    if (depth == 1) return A_NORMAL;
    return A_DIM;                     /* deep nodes recede visually      */
}

/* node drawing */

static void draw_node_border(const QuadNode *node, TreeCanvas cv)
{
    int left   = canvas_col(cv, node->bounds.x);
    int top    = canvas_row(cv, node->bounds.y);
    int right  = canvas_col(cv, node->bounds.x + node->bounds.w);
    int bottom = canvas_row(cv, node->bounds.y + node->bounds.h);

    int    pair      = border_color_for_depth(node->depth);
    attr_t intensity = border_intensity_for_depth(node->depth);

    attron(COLOR_PAIR(pair) | intensity);

    /* Four corners */
    canvas_put(cv, top,    left,  '+');
    canvas_put(cv, top,    right, '+');
    canvas_put(cv, bottom, left,  '+');
    canvas_put(cv, bottom, right, '+');

    /* Horizontal edges (top and bottom) */
    for (int c = left + 1; c < right; c++) {
        canvas_put(cv, top,    c, '-');
        canvas_put(cv, bottom, c, '-');
    }

    /* Vertical edges (left and right) */
    for (int r = top + 1; r < bottom; r++) {
        canvas_put(cv, r, left,  '|');
        canvas_put(cv, r, right, '|');
    }

    attroff(COLOR_PAIR(pair) | intensity);
}

/* Was p one of the last query's hits? Exact float == is fine here because the
 * results are byte-for-byte copies of the stored points, never recomputed. */
static bool point_was_found(const Scene *sc, Vec2 p)
{
    for (int i = 0; i < sc->query_result_count; i++)
        if (sc->query_results[i].x == p.x && sc->query_results[i].y == p.y)
            return true;
    return false;
}

static void draw_node_points(const QuadNode *node, const Scene *sc, TreeCanvas cv)
{
    for (int i = 0; i < node->point_count; i++) {
        Vec2 p   = node->points[i];
        int  col = canvas_col(cv, p.x);
        int  row = canvas_row(cv, p.y);

        bool inside_query = (sc->phase == PHASE_QUERY) && point_was_found(sc, p);

        if (inside_query) {
            attron(COLOR_PAIR(CP_POINT_HIT) | A_BOLD);
            canvas_put(cv, row, col, '*');   /* '*' = point is found/lit up  */
            attroff(COLOR_PAIR(CP_POINT_HIT) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_POINT_IDLE) | A_BOLD);
            canvas_put(cv, row, col, 'o');   /* 'o' = ordinary resting point */
            attroff(COLOR_PAIR(CP_POINT_IDLE) | A_BOLD);
        }
    }
}

/* Recurse through the tree: draw every node's border then its points */
static void draw_tree(int node_idx, const Scene *sc, TreeCanvas cv)
{
    if (node_idx == NO_CHILD) return;
    const QuadNode *node = &sc->pool.nodes[node_idx];

    draw_node_border(node, cv);
    draw_node_points(node, sc, cv);

    for (int c = 0; c < NUM_CHILDREN; c++)
        draw_tree(node->children[c], sc, cv);
}

/* query rectangle drawing */

static void draw_query_box(const QueryBox *query, TreeCanvas cv)
{
    int left   = canvas_col(cv, query->bounds.x);
    int top    = canvas_row(cv, query->bounds.y);
    int right  = canvas_col(cv, query->bounds.x + query->bounds.w);
    int bottom = canvas_row(cv, query->bounds.y + query->bounds.h);

    attron(COLOR_PAIR(CP_QUERY_BOX) | A_BOLD);

    /* Bracket corners distinguish this from tree-node borders ('+') */
    canvas_put(cv, top,    left,  '[');
    canvas_put(cv, top,    right, ']');
    canvas_put(cv, bottom, left,  '[');
    canvas_put(cv, bottom, right, ']');

    /* Tilde on top/bottom edges gives a "scanning" feel */
    for (int c = left + 1; c < right; c++) {
        canvas_put(cv, top,    c, '~');
        canvas_put(cv, bottom, c, '~');
    }
    for (int r = top + 1; r < bottom; r++) {
        canvas_put(cv, r, left,  '|');
        canvas_put(cv, r, right, '|');
    }

    attroff(COLOR_PAIR(CP_QUERY_BOX) | A_BOLD);
}

/* info panel drawing */

static void draw_insert_explanation(PanelWriter *pw)
{
    panel_text(pw, CP_PANEL, A_BOLD,   "OP: INSERT");
    panel_blank(pw);
    panel_text(pw, CP_PANEL, A_NORMAL, "1. Find leaf for (x,y)");
    panel_text(pw, CP_PANEL, A_NORMAL, "2. Leaf not full:");
    panel_text(pw, CP_PANEL, A_NORMAL, "     store point.");
    panel_text(pw, CP_PANEL, A_NORMAL, "3. Leaf FULL:");
    panel_text(pw, CP_PANEL, A_NORMAL, "     SUBDIVIDE -> 4");
    panel_text(pw, CP_PANEL, A_NORMAL, "     NW / NE / SW / SE");
    panel_text(pw, CP_PANEL, A_NORMAL, "     redistribute pts.");
    panel_text(pw, CP_PANEL, A_NORMAL, "4. Route to child");
    panel_text(pw, CP_PANEL, A_NORMAL, "     containing (x,y).");
}

static void draw_query_explanation(PanelWriter *pw)
{
    panel_text(pw, CP_PANEL, A_BOLD,   "OP: RANGE QUERY");
    panel_blank(pw);
    panel_text(pw, CP_PANEL, A_NORMAL, "Orange box moves.");
    panel_text(pw, CP_PANEL, A_NORMAL, "Found pts glow green *");
    panel_blank(pw);
    panel_text(pw, CP_PANEL, A_NORMAL, "1. Rect misses node?");
    panel_text(pw, CP_PANEL, A_NORMAL, "     PRUNE subtree.");
    panel_text(pw, CP_PANEL, A_NORMAL, "2. Check node's pts");
    panel_text(pw, CP_PANEL, A_NORMAL, "     against rect.");
    panel_text(pw, CP_PANEL, A_NORMAL, "3. Recurse children");
    panel_text(pw, CP_PANEL, A_NORMAL, "     that overlap.");
    panel_blank(pw);
    panel_text(pw, CP_PANEL, A_NORMAL, "Cost: O(log N + k)");
    panel_text(pw, CP_PANEL, A_NORMAL, "k = points found");
}

static void draw_border_legend(PanelWriter *pw)
{
    panel_text(pw, CP_PANEL,       A_BOLD,   "BORDERS BY DEPTH");
    panel_text(pw, CP_ROOT_BORDER, A_BOLD,   "depth 0  (root)");
    panel_text(pw, CP_D1_BORDER,   A_NORMAL, "depth 1");
    panel_text(pw, CP_D2_BORDER,   A_DIM,    "depth 2");
    panel_text(pw, CP_DEEP_BORDER, A_DIM,    "depth 3+");
}

static void draw_info_panel(const Scene *sc, int terminal_cols, int terminal_rows)
{
    PanelWriter pw = panel_begin(terminal_cols, terminal_rows);

    /* Vertical separator between tree area and panel */
    attron(COLOR_PAIR(CP_DEEP_BORDER) | A_DIM);
    for (int r = HUD_TOP_ROWS; r < terminal_rows - HUD_BOT_ROWS; r++)
        mvaddch(r, pw.panel_col - 1, '|');
    attroff(COLOR_PAIR(CP_DEEP_BORDER) | A_DIM);

    panel_text(&pw, CP_PANEL, A_BOLD, "QUADTREE HELLO WORLD");
    panel_text(&pw, CP_PANEL, A_BOLD, "4-child 2D space tree");
    panel_divider(&pw, terminal_cols);

    /* Operation-specific explanation */
    if (sc->phase == PHASE_INSERT)
        draw_insert_explanation(&pw);
    else
        draw_query_explanation(&pw);

    panel_divider(&pw, terminal_cols);

    /* Live statistics */
    panel_text(&pw, CP_PANEL, A_NORMAL,
               "Points : %d / %d", sc->points_inserted, DEMO_POINT_COUNT);
    panel_text(&pw, CP_PANEL, A_NORMAL,
               "Nodes  : %d",       tree_total_nodes(&sc->pool, sc->pool.root_idx));
    panel_text(&pw, CP_PANEL, A_NORMAL,
               "Depth  : %d  (cap=%d)",
               tree_current_depth(&sc->pool, sc->pool.root_idx), LEAF_CAPACITY);
    if (sc->phase == PHASE_QUERY)
        panel_text(&pw, CP_PANEL, A_NORMAL,
                   "Found  : %d", sc->query_result_count);

    /* Border color legend — only if there is room below the stats */
    if (pw.current_row < terminal_rows - 7) {
        panel_divider(&pw, terminal_cols);
        draw_border_legend(&pw);
    }
}

/* one frame of the scene */

/* Paint the tree first, then the query box and panel on top, so those two
 * always win any cell a tree border tries to share. */
static void scene_draw(const Scene *sc)
{
    TreeCanvas canvas = canvas_make(sc->cols, sc->rows);

    draw_tree(sc->pool.root_idx, sc, canvas);

    if (sc->phase == PHASE_QUERY)
        draw_query_box(&sc->query, canvas);

    draw_info_panel(sc, sc->cols, sc->rows);
}

/* reset / seeding */

/* Wipe the Scene to zero but keep the things the user controls: terminal
 * size and sim_hz. Wiping wholesale (rather than clearing field by field)
 * means a future field can't be forgotten here. */
static void reset_scene_state_preserving_settings(Scene *s)
{
    int saved_rows   = s->rows;
    int saved_cols   = s->cols;
    int saved_sim_hz = s->sim_hz;

    memset(s, 0, sizeof *s);

    s->rows   = saved_rows;
    s->cols   = saved_cols;
    s->sim_hz = saved_sim_hz;
}

/* Start the insert phase. The first point comes in faster than the steady
 * rhythm so the canvas isn't blank for a full interval at startup. */
static void seed_insert_phase(Scene *s)
{
    s->phase          = PHASE_INSERT;
    s->next_insert_in = SECONDS_PER_INSERTION * 0.25f;
}

/* Place the query rectangle near the top-left, drifting diagonally. Each axis
 * gets QUERY_DRIFT_SPEED * 0.7071 (that's 1/sqrt(2)) so the combined diagonal
 * speed comes out to exactly QUERY_DRIFT_SPEED. */
static void seed_query_box(QueryBox *q)
{
    q->bounds  = (Rect){ .x = 0.08f, .y = 0.08f,
                         .w = QUERY_RECT_WIDTH, .h = QUERY_RECT_HEIGHT };
    q->drift_x = QUERY_DRIFT_SPEED * 0.7071f;   /* 1 / sqrt(2) */
    q->drift_y = QUERY_DRIFT_SPEED * 0.7071f;
}

/* Start the whole demo over: empty tree, fresh insert phase, query box
 * primed. Runs on 'r' and automatically when the query phase times out. */
static void scene_reset(Scene *s)
{
    reset_scene_state_preserving_settings(s);
    pool_reset(&s->pool);
    seed_insert_phase(s);
    seed_query_box(&s->query);
}

static void scene_advance_phase(Scene *s)
{
    if (s->phase == PHASE_QUERY) {
        scene_reset(s);   /* cycling past query → full restart           */
        return;
    }
    s->phase         = PHASE_QUERY;
    s->time_in_phase = 0.0f;
}

/* per-tick work for each phase */

static void insert_random_point(Scene *s)
{
    /* Keep points away from the very edge so border drawing looks clean */
    float x = 0.03f + (float)(rand() % 9400) / 10000.0f;  /* [0.03, 0.97] */
    float y = 0.03f + (float)(rand() % 9400) / 10000.0f;
    tree_insert(&s->pool, s->pool.root_idx, x, y);
    s->points_inserted++;
    s->next_insert_in = SECONDS_PER_INSERTION;
}

/* Move the box by velocity * dt. No acceleration — constant speed between
 * bounces. */
static inline void integrate_query_box_euler(QueryBox *q, float dt)
{
    q->bounds.x += q->drift_x * dt;
    q->bounds.y += q->drift_y * dt;
}

/* Bounce the box off the [0,1] walls: pin it back inside and flip the speed
 * that crossed. Each axis is handled on its own, so a corner hit reflects
 * both. Using fabsf (not *= -1) keeps the sign correct even if several ticks
 * in a row leave the box touching the same wall. */
static inline void bounce_query_box_off_walls(QueryBox *q)
{
    if (q->bounds.x < 0.0f) {
        q->bounds.x = 0.0f;
        q->drift_x  =  fabsf(q->drift_x);   /* reflect → move right     */
    }
    if (q->bounds.y < 0.0f) {
        q->bounds.y = 0.0f;
        q->drift_y  =  fabsf(q->drift_y);   /* reflect → move down      */
    }
    if (q->bounds.x + q->bounds.w > 1.0f) {
        q->bounds.x = 1.0f - q->bounds.w;
        q->drift_x  = -fabsf(q->drift_x);   /* reflect → move left      */
    }
    if (q->bounds.y + q->bounds.h > 1.0f) {
        q->bounds.y = 1.0f - q->bounds.h;
        q->drift_y  = -fabsf(q->drift_y);   /* reflect → move up        */
    }
}

/* One tick of box motion: move, then bounce off any wall it crossed. */
static void advance_query_box(QueryBox *q, float dt)
{
    integrate_query_box_euler(q, dt);
    bounce_query_box_off_walls(q);
}

/* Insert phase: drop one point each time the countdown expires, until the
 * tree is full, then switch to the query phase. The countdown paces the
 * points so each insertion (and any split) is visible. */
static void tick_insert_phase(Scene *s, float dt)
{
    s->next_insert_in -= dt;
    if (s->next_insert_in <= 0.0f && s->points_inserted < DEMO_POINT_COUNT) {
        insert_random_point(s);
        if (s->points_inserted >= DEMO_POINT_COUNT)
            scene_advance_phase(s);
    }
}

/* Re-run the query from scratch each tick so the lit-up points follow the
 * moving box. tree_query is cheap enough that recomputing beats tracking
 * changes incrementally. */
static void refresh_query_results(Scene *s)
{
    s->query_result_count = 0;
    tree_query(&s->pool, s->pool.root_idx, s->query.bounds,
               s->query_results, &s->query_result_count, QUERY_RESULT_CAP);
}

/* Query phase: move the box, re-find the hits, and after a while restart the
 * whole demo so it loops without any input. */
static void tick_query_phase(Scene *s, float dt)
{
    advance_query_box(&s->query, dt);
    refresh_query_results(s);
    if (s->time_in_phase > QUERY_PHASE_DURATION)
        scene_reset(s);
}

/* Advance the demo by dt seconds, running whichever phase is active. Pausing
 * freezes everything. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    s->time_in_phase += dt;

    if (s->phase == PHASE_INSERT) tick_insert_phase(s, dt);
    else                          tick_query_phase (s, dt);
}

/* ── §7 frame — HUD bars and the per-frame paint+flush ── */

/* Top HUD: row 0 is the live status (fps, Hz, phase, paused), row 1 the
 * fixed tree config. */
static void draw_top_hud(const Scene *sc, double fps)
{
    const char *phase_label = (sc->phase == PHASE_INSERT) ? "INSERT" : "QUERY";
    char status[80];
    snprintf(status, sizeof status,
             " %5.1f fps  sim:%3d Hz  [%s]  %s ",
             fps, sc->sim_hz, phase_label, sc->paused ? "PAUSED" : "running");

    int hud_start_col = sc->cols - INFO_PANEL_COLS - (int)strlen(status);
    if (hud_start_col < 0) hud_start_col = 0;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hud_start_col, "%s", status);
    mvprintw(1, 0, " Quadtree  capacity=%d pts/leaf  max depth=%d  demo: %d pts",
             LEAF_CAPACITY, MAX_TREE_DEPTH, DEMO_POINT_COUNT);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom HUD: the key-hint bar, listing every key handle_input accepts. */
static void draw_bottom_hud(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  space:pause  n:next/reset  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Paint everything and push one diff to the terminal. The wnoutrefresh +
 * doupdate pair is the ncurses way to emit a single update per frame. */
static void render_one_frame(const Scene *sc, double fps)
{
    erase();
    scene_draw(sc);
    draw_top_hud(sc, fps);
    draw_bottom_hud(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 app — signals, input, ncurses setup, and the main loop ── */

/* Signal flags — file-scope because a signal handler can't take a pointer.
 * Everything else lives on main's stack. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_exit_signal  (int s) { (void)s; g_quit   = 1; }
static void on_resize_signal(int s) { (void)s; g_resize = 1; }
static void cleanup         (void)  { endwin(); }

/* Act on one keypress. Returns false only when the user asks to quit. */
static bool handle_input(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            sc->paused = !sc->paused;       break;
    case 'n': case 'N':  scene_advance_phase(sc);        break;
    case 'r': case 'R':  scene_reset(sc);                break;
    case ']':
        if (sc->sim_hz < SIM_HZ_MAX) sc->sim_hz += SIM_HZ_STEP;
        break;
    case '[':
        if (sc->sim_hz > SIM_HZ_MIN) sc->sim_hz -= SIM_HZ_STEP;
        break;
    default: break;
    }
    return true;
}

/* Bring up ncurses: raw keys, no echo, hidden cursor, non-blocking getch,
 * keypad on, typeahead off (so input never interrupts the screen write). */
static void init_ncurses_session(void)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
}

/* React to a terminal resize. The tree is untouched (its coords are in
 * [0,1]); only the screen size changes, which canvas_make picks up next frame. */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * main — set things up, then run the loop: advance the sim in fixed steps,
 * cap rendering at 60 fps, draw, read input. The accumulator keeps the sim
 * rate (user-adjustable) separate from the frame rate.
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    init_ncurses_session();

    Scene scene = {0};
    scene.sim_hz = SIM_HZ_DEFAULT;
    getmaxyx(stdscr, scene.rows, scene.cols);
    scene_reset(&scene);

    int64_t frame_start = clock_ns();
    int64_t sim_accum   = 0;       /* nanoseconds "owed" to the sim     */
    int64_t fps_accum   = 0;       /* elapsed ns in the current fps win  */
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (!g_quit) {

        if (g_resize) {
            handle_resize(&scene);
            frame_start = clock_ns();
            sim_accum   = 0;
        }

        /* time since last frame */
        int64_t now     = clock_ns();
        int64_t elapsed = now - frame_start;
        frame_start     = now;
        if (elapsed > 100 * NS_PER_MS) elapsed = 100 * NS_PER_MS; /* pause guard */

        /* run as many fixed sim steps as the elapsed time has banked */
        int64_t tick_duration_ns = TICK_NS(scene.sim_hz);
        float   tick_duration_s  = (float)tick_duration_ns / (float)NS_PER_SEC;

        sim_accum += elapsed;
        while (sim_accum >= tick_duration_ns) {
            scene_tick(&scene, tick_duration_s);
            sim_accum -= tick_duration_ns;
        }

        /* fps, averaged over a 500 ms window */
        frame_count++;
        fps_accum += elapsed;
        if (fps_accum >= FPS_WINDOW_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* sleep off the rest of the frame to cap at 60 fps */
        int64_t time_spent = clock_ns() - frame_start + elapsed;
        clock_sleep_ns(NS_PER_SEC / 60 - time_spent);

        render_one_frame(&scene, fps_display);

        int ch = getch();
        if (ch != ERR && !handle_input(&scene, ch))
            g_quit = 1;
    }

    return 0;
}
