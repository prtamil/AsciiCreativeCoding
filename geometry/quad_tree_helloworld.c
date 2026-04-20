/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * geometry/quad_tree_helloworld.c — Quadtree hello-world: basic operations
 *
 * WHAT THIS SHOWS:
 *   A quadtree partitions 2-D space recursively into four quadrants
 *   (NW / NE / SW / SE) whenever a region holds more than LEAF_CAPACITY
 *   points.  Three core operations are animated live:
 *
 *     INSERT    — add a point; subdivide the leaf if at capacity
 *     SUBDIVIDE — split one node into four children; redistribute its points
 *     QUERY     — range query: find all points inside a search rectangle
 *
 * DEMO FLOW:
 *   Phase 1 (INSERT) — random points appear one by one every ~0.4 s;
 *                      watch subdivisions happen in real time.
 *   Phase 2 (QUERY)  — an orange rectangle bounces around the tree;
 *                      points inside it glow green.
 *   Automatically resets after the query phase ends.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   n         next phase  (during query: full reset)
 *   r         reset tree and restart from phase 1
 *   ] / [     raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/quad_tree_helloworld.c \
 *       -o qt_hello -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 canvas  §5 quadtree  §6 scene
 * §7 screen  §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Node layout:
 *   Each node owns a rectangular slice of [0,1]×[0,1] space.
 *   While it is a leaf it stores up to LEAF_CAPACITY data points directly.
 *   When it overflows, subdivide() splits it into four equal children
 *   and moves the existing points into the appropriate child.
 *
 *   children[NW]  children[NE]
 *   children[SW]  children[SE]      -1 means "no child" (leaf node)
 *
 * INSERT (tree_insert):
 *   1. Point outside this node's region? → reject.
 *   2. Leaf with room?                   → store here, done.
 *   3. Leaf that is full?                → subdivide, then re-route.
 *   4. Internal (non-leaf) node?         → delegate to the matching child.
 *
 * QUERY (tree_query):
 *   At each node: if the search rectangle does NOT overlap this node's
 *   region, skip the ENTIRE subtree — this is the O(log N + k) pruning
 *   that makes quadtree queries efficient (k = points found).
 *
 * Pool allocator:
 *   All nodes live in a fixed-size static array; no malloc or free.
 *   Reset = set the used-count back to zero and re-create the root node.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── simulation loop ───────────────────────────────────────────────── */
enum {
    SIM_HZ_MIN     = 10,
    SIM_HZ_DEFAULT = 60,
    SIM_HZ_MAX     = 120,
    SIM_HZ_STEP    = 10,
    FPS_WINDOW_MS  = 500,   /* fps averaged over this sliding window     */
};

/* ── quadtree structure ────────────────────────────────────────────── */
enum {
    LEAF_CAPACITY  = 3,   /* points per leaf before it must subdivide   */
    MAX_TREE_DEPTH = 5,   /* root = depth 0; deepest allowed leaf = 5   */
    NODE_POOL_SIZE = 512, /* static node pool; no malloc ever needed.   */
                          /* worst-case node count with 30 points and   */
                          /* cap=3 is well under 200; 512 is generous.  */
};

/* ── demo pacing ───────────────────────────────────────────────────── */
enum {
    DEMO_POINT_COUNT  = 30,              /* total points inserted in phase 1 */
    QUERY_RESULT_CAP  = DEMO_POINT_COUNT,/* upper bound on range-query hits  */
};
#define SECONDS_PER_INSERTION  0.42f    /* insert one new point every ~0.4 s */
#define QUERY_PHASE_DURATION   22.0f    /* seconds before auto-reset          */

/* ── query rectangle ───────────────────────────────────────────────── */
#define QUERY_RECT_WIDTH    0.28f   /* fraction of normalised [0,1] space   */
#define QUERY_RECT_HEIGHT   0.28f
#define QUERY_DRIFT_SPEED   0.18f   /* movement: tree-space units per second */

/* ── screen layout ─────────────────────────────────────────────────── */
enum {
    INFO_PANEL_COLS   = 28,  /* right-side panel width in terminal columns */
    TREE_AREA_TOP_ROW =  2,  /* rows 0-1 are the HUD banner; tree starts here */
};

/* ── color pair names ──────────────────────────────────────────────── */
/*
 * Using named constants instead of raw integers eliminates magic numbers
 * from every draw call.  The values 1..8 are the ncurses pair indices
 * assigned in color_init().
 */
enum {
    CP_ROOT_BORDER = 1,  /* depth-0 node border  white, bold            */
    CP_D1_BORDER   = 2,  /* depth-1 node border  cyan                   */
    CP_D2_BORDER   = 3,  /* depth-2 node border  blue                   */
    CP_DEEP_BORDER = 4,  /* depth 3+ node border grey, dim              */
    CP_POINT_IDLE  = 5,  /* ordinary inserted point  yellow             */
    CP_POINT_HIT   = 6,  /* point inside query rect  green, bold        */
    CP_QUERY_BOX   = 7,  /* animated query rectangle orange             */
    CP_PANEL       = 8,  /* info panel and HUD text  gold               */
};

/* ── timing helpers ────────────────────────────────────────────────── */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(hz)  (NS_PER_SEC / (hz))

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
        init_pair(CP_ROOT_BORDER, 255, COLOR_BLACK);  /* white           */
        init_pair(CP_D1_BORDER,    51, COLOR_BLACK);  /* cyan            */
        init_pair(CP_D2_BORDER,    33, COLOR_BLACK);  /* blue            */
        init_pair(CP_DEEP_BORDER, 240, COLOR_BLACK);  /* grey            */
        init_pair(CP_POINT_IDLE,  226, COLOR_BLACK);  /* yellow          */
        init_pair(CP_POINT_HIT,    46, COLOR_BLACK);  /* green           */
        init_pair(CP_QUERY_BOX,   208, COLOR_BLACK);  /* orange          */
        init_pair(CP_PANEL,       220, COLOR_BLACK);  /* gold            */
    } else {
        init_pair(CP_ROOT_BORDER, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_D1_BORDER,   COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_D2_BORDER,   COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_DEEP_BORDER, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_POINT_IDLE,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_POINT_HIT,   COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_QUERY_BOX,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_PANEL,       COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  canvas — safe drawing within the tree area                        */
/* ===================================================================== */

/*
 * The quadtree visualisation occupies only the LEFT portion of the
 * terminal.  TreeCanvas records those boundaries so every draw helper
 * can clip safely without re-computing them each time.
 *
 * Normalised tree coordinates (nx, ny) ∈ [0,1]×[0,1] map to terminal
 * cells via canvas_col() and canvas_row().
 */
typedef struct {
    int top_row;    /* first row belonging to the tree area              */
    int bottom_row; /* one-past-last row  (exclusive upper bound)        */
    int right_col;  /* one-past-last col  (exclusive upper bound)        */
} TreeCanvas;

static TreeCanvas canvas_make(int terminal_cols, int terminal_rows)
{
    int right = terminal_cols - INFO_PANEL_COLS;
    int bot   = terminal_rows - 2;
    return (TreeCanvas){
        .top_row    = TREE_AREA_TOP_ROW,
        .bottom_row = bot   > TREE_AREA_TOP_ROW + 1 ? bot   : TREE_AREA_TOP_ROW + 2,
        .right_col  = right > 1                      ? right : 2,
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

/* ===================================================================== */
/* §5  quadtree — types, pool, core operations, statistics               */
/* ===================================================================== */

/* ── primitive geometry types ──────────────────────────────────────── */

/* A 2-D point in normalised tree space [0,1]×[0,1] */
typedef struct { float x, y; } Vec2;

/* An axis-aligned rectangle in tree space */
typedef struct { float x, y, w, h; } Rect;

/* True when (px, py) is inside rectangle r.
 * Uses half-open interval [x, x+w) so adjacent rects don't double-count. */
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

/* ── quadtree node ─────────────────────────────────────────────────── */

/* Child slot names — clearer than using raw indices 0..3 */
enum { NW = 0, NE = 1, SW = 2, SE = 3, NUM_CHILDREN = 4 };
#define NO_CHILD  (-1)   /* sentinel: slot is empty (leaf node)          */

typedef struct {
    Rect bounds;                    /* region this node owns in [0,1]×[0,1] */
    Vec2 points[LEAF_CAPACITY];     /* data points (valid only when leaf)   */
    int  point_count;               /* 0..LEAF_CAPACITY for leaves; 0 for internal */
    int  children[NUM_CHILDREN];    /* NW/NE/SW/SE indices; NO_CHILD if absent */
    int  depth;                     /* 0 = root; larger = deeper            */
} QuadNode;

static inline bool node_is_leaf(const QuadNode *n) { return n->children[NW] == NO_CHILD; }
static inline bool node_is_full(const QuadNode *n) { return n->point_count == LEAF_CAPACITY; }

/* ── node pool ─────────────────────────────────────────────────────── */

/*
 * All nodes come from this fixed array.  No heap allocation ever occurs.
 * Resetting the tree is O(1): set g_node_count = 0 and re-create root.
 */
static QuadNode g_nodes[NODE_POOL_SIZE];
static int      g_node_count;   /* how many nodes are currently allocated */
static int      g_root;         /* index of the root node                 */

static bool pool_has_room(int slots_needed)
{
    return g_node_count + slots_needed <= NODE_POOL_SIZE;
}

static int node_alloc(Rect region, int depth)
{
    if (g_node_count >= NODE_POOL_SIZE) return NO_CHILD;
    int idx = g_node_count++;
    g_nodes[idx] = (QuadNode){
        .bounds      = region,
        .point_count = 0,
        .children    = { NO_CHILD, NO_CHILD, NO_CHILD, NO_CHILD },
        .depth       = depth,
    };
    return idx;
}

static void tree_reset(void)
{
    g_node_count = 0;
    Rect full_space = { .x = 0.0f, .y = 0.0f, .w = 1.0f, .h = 1.0f };
    g_root = node_alloc(full_space, /*depth=*/0);
}

/* ── core operations ───────────────────────────────────────────────── */

/* Forward declaration: subdivide() calls tree_insert() for redistribution */
static bool tree_insert(int node_idx, float x, float y);

/*
 * subdivide — split a full leaf into four equal quadrant children.
 *
 * After splitting, the parent becomes an internal (non-leaf) node and
 * its existing points are redistributed into the appropriate child.
 *
 * Before:                  After:
 *   [parent: 3 pts]          [parent: 0 pts]
 *                             /    |    |    \
 *                           [NW]  [NE]  [SW]  [SE]
 *                         (points redistributed among children)
 */
static void subdivide(int parent_idx)
{
    QuadNode *parent  = &g_nodes[parent_idx];
    float     half_w  = parent->bounds.w * 0.5f;
    float     half_h  = parent->bounds.h * 0.5f;
    float     left    = parent->bounds.x;
    float     top     = parent->bounds.y;
    float     mid_x   = left + half_w;
    float     mid_y   = top  + half_h;
    int       next_depth = parent->depth + 1;

    parent->children[NW] = node_alloc((Rect){left,  top,   half_w, half_h}, next_depth);
    parent->children[NE] = node_alloc((Rect){mid_x, top,   half_w, half_h}, next_depth);
    parent->children[SW] = node_alloc((Rect){left,  mid_y, half_w, half_h}, next_depth);
    parent->children[SE] = node_alloc((Rect){mid_x, mid_y, half_w, half_h}, next_depth);

    /*
     * Redistribute the parent's existing points into the new children.
     * We copy them out first because tree_insert() modifies point_count.
     */
    int  old_count = parent->point_count;
    Vec2 displaced[LEAF_CAPACITY];
    memcpy(displaced, parent->points, (size_t)old_count * sizeof(Vec2));
    parent->point_count = 0;

    for (int i = 0; i < old_count; i++)
        for (int c = 0; c < NUM_CHILDREN; c++)
            if (tree_insert(parent->children[c], displaced[i].x, displaced[i].y))
                break;
}

/*
 * tree_insert — add point (x, y) into the subtree rooted at node_idx.
 *
 * Returns true when the point was accepted, false when it lies outside
 * this node's region or the pool/depth limit was reached.
 */
static bool tree_insert(int node_idx, float x, float y)
{
    if (node_idx == NO_CHILD) return false;

    QuadNode *node = &g_nodes[node_idx];

    if (!rect_contains_point(node->bounds, x, y))
        return false;   /* (x,y) is outside this node's region           */

    if (node_is_leaf(node)) {
        if (!node_is_full(node)) {
            /* Leaf has room: store the point here and we're done */
            node->points[node->point_count++] = (Vec2){x, y};
            return true;
        }

        /* Leaf is full: split into four children, then re-route below */
        if (node->depth < MAX_TREE_DEPTH && pool_has_room(NUM_CHILDREN)) {
            subdivide(node_idx);
            /* node->children are now set; fall through to delegate */
        } else {
            return false;   /* depth limit reached or pool exhausted     */
        }
    }

    /* Internal node: let the child whose region contains (x,y) handle it */
    for (int c = 0; c < NUM_CHILDREN; c++)
        if (tree_insert(node->children[c], x, y))
            return true;

    return false;
}

/*
 * tree_query — collect every point inside search_area into results[].
 *
 * The key efficiency step is the overlap check at the top: if search_area
 * does not overlap this node's region, the entire subtree is pruned.
 * Only the nodes whose regions overlap the search area are visited,
 * giving O(log N + k) complexity instead of O(N).
 */
static void tree_query(int node_idx, Rect search_area,
                       Vec2 *results, int *result_count, int result_cap)
{
    if (node_idx == NO_CHILD || *result_count >= result_cap) return;

    QuadNode *node = &g_nodes[node_idx];

    /* Pruning: skip this whole subtree if it cannot contain any matches */
    if (!rect_overlaps(node->bounds, search_area)) return;

    /* This node overlaps the search area — check each point it stores */
    for (int i = 0; i < node->point_count && *result_count < result_cap; i++) {
        Vec2 p = node->points[i];
        if (rect_contains_point(search_area, p.x, p.y))
            results[(*result_count)++] = p;
    }

    /* Recurse into children; they will prune themselves if needed */
    for (int c = 0; c < NUM_CHILDREN; c++)
        tree_query(node->children[c], search_area, results, result_count, result_cap);
}

/* ── statistics (called once per frame; O(nodes), negligible cost) ── */

static int tree_total_nodes(int node_idx)
{
    if (node_idx == NO_CHILD) return 0;
    int count = 1;
    for (int c = 0; c < NUM_CHILDREN; c++)
        count += tree_total_nodes(g_nodes[node_idx].children[c]);
    return count;
}

static int tree_current_depth(int node_idx)
{
    if (node_idx == NO_CHILD) return -1;
    if (node_is_leaf(&g_nodes[node_idx])) return g_nodes[node_idx].depth;
    int deepest = 0;
    for (int c = 0; c < NUM_CHILDREN; c++) {
        int d = tree_current_depth(g_nodes[node_idx].children[c]);
        if (d > deepest) deepest = d;
    }
    return deepest;
}

/* ===================================================================== */
/* §6  scene — demo state, drawing helpers, tick, render                 */
/* ===================================================================== */

/* ── demo phase ────────────────────────────────────────────────────── */

typedef enum {
    PHASE_INSERT,   /* animate point insertions; watch subdivisions      */
    PHASE_QUERY,    /* animate bouncing range-query rectangle            */
} DemoPhase;

/* ── query box ─────────────────────────────────────────────────────── */

typedef struct {
    Rect  bounds;    /* current position and size in tree space          */
    float drift_x;  /* horizontal speed: tree-space units per second    */
    float drift_y;  /* vertical speed:   tree-space units per second    */
} QueryBox;

/* ── scene state ───────────────────────────────────────────────────── */

typedef struct {
    DemoPhase phase;
    float     time_in_phase;    /* seconds elapsed since this phase began */
    float     next_insert_in;   /* seconds until the next point is added  */
    int       points_inserted;  /* how many points have been inserted     */

    QueryBox  query;            /* the animated search rectangle          */
    Vec2      query_results[QUERY_RESULT_CAP];  /* points found last tick */
    int       query_result_count;

    bool      paused;
} Scene;

/* ── info panel helpers ────────────────────────────────────────────── */

/*
 * PanelWriter tracks the current write position inside the right-side
 * info panel so each helper just increments a row counter automatically.
 * Eliminates the repetitive (row, col) bookkeeping in every mvprintw.
 */
typedef struct {
    int panel_col;  /* left edge of the panel in terminal columns        */
    int current_row;
    int last_row;   /* one-past the last writable row (clipping bound)  */
} PanelWriter;

static PanelWriter panel_begin(int terminal_cols, int terminal_rows)
{
    return (PanelWriter){
        .panel_col   = terminal_cols - INFO_PANEL_COLS,
        .current_row = TREE_AREA_TOP_ROW,
        .last_row    = terminal_rows - 2,
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

/* ── border rendering helpers ──────────────────────────────────────── */

/*
 * Choose border color by depth so the recursive structure is immediately
 * visible: root = bright white, depth 1 = cyan, depth 2 = blue, deeper = grey.
 */
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

/* ── node drawing ──────────────────────────────────────────────────── */

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

/*
 * Check whether a point p was returned by the last range query.
 *
 * Linear scan over query_results (at most DEMO_POINT_COUNT entries).
 * Exact float comparison is correct here: the values in query_results
 * are copied verbatim from g_nodes — they are never recomputed.
 */
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
    QuadNode *node = &g_nodes[node_idx];

    draw_node_border(node, cv);
    draw_node_points(node, sc, cv);

    for (int c = 0; c < NUM_CHILDREN; c++)
        draw_tree(node->children[c], sc, cv);
}

/* ── query rectangle drawing ───────────────────────────────────────── */

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

/* ── info panel drawing ────────────────────────────────────────────── */

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
    for (int r = TREE_AREA_TOP_ROW; r < terminal_rows - 2; r++)
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
               "Nodes  : %d",       tree_total_nodes(g_root));
    panel_text(&pw, CP_PANEL, A_NORMAL,
               "Depth  : %d  (cap=%d)", tree_current_depth(g_root), LEAF_CAPACITY);
    if (sc->phase == PHASE_QUERY)
        panel_text(&pw, CP_PANEL, A_NORMAL,
                   "Found  : %d", sc->query_result_count);

    /* Border color legend — only if there is room below the stats */
    if (pw.current_row < terminal_rows - 7) {
        panel_divider(&pw, terminal_cols);
        draw_border_legend(&pw);
    }
}

/* ── scene_draw ────────────────────────────────────────────────────── */

static void scene_draw(const Scene *sc, int terminal_cols, int terminal_rows)
{
    TreeCanvas canvas = canvas_make(terminal_cols, terminal_rows);

    draw_tree(g_root, sc, canvas);

    if (sc->phase == PHASE_QUERY)
        draw_query_box(&sc->query, canvas);

    draw_info_panel(sc, terminal_cols, terminal_rows);
}

/* ── scene lifecycle ───────────────────────────────────────────────── */

static void scene_reset(Scene *s)
{
    memset(s, 0, sizeof *s);
    tree_reset();

    s->phase          = PHASE_INSERT;
    /* Start slightly earlier so the first point appears quickly */
    s->next_insert_in = SECONDS_PER_INSERTION * 0.25f;

    /* Query box: top-left corner, drifting diagonally.
     * 1/sqrt(2) ≈ 0.7071 splits QUERY_DRIFT_SPEED equally between axes. */
    s->query.bounds  = (Rect){ .x = 0.08f, .y = 0.08f,
                               .w = QUERY_RECT_WIDTH, .h = QUERY_RECT_HEIGHT };
    s->query.drift_x = QUERY_DRIFT_SPEED * 0.7071f;
    s->query.drift_y = QUERY_DRIFT_SPEED * 0.7071f;
}

static void scene_init(Scene *s) { scene_reset(s); }

static void scene_advance_phase(Scene *s)
{
    if (s->phase == PHASE_QUERY) {
        scene_reset(s);   /* cycling past query → full restart           */
        return;
    }
    s->phase         = PHASE_QUERY;
    s->time_in_phase = 0.0f;
}

/* ── scene_tick helpers ────────────────────────────────────────────── */

static void insert_random_point(Scene *s)
{
    /* Keep points away from the very edge so border drawing looks clean */
    float x = 0.03f + (float)(rand() % 9400) / 10000.0f;  /* [0.03, 0.97] */
    float y = 0.03f + (float)(rand() % 9400) / 10000.0f;
    tree_insert(g_root, x, y);
    s->points_inserted++;
    s->next_insert_in = SECONDS_PER_INSERTION;
}

static void advance_query_box(QueryBox *q, float dt)
{
    q->bounds.x += q->drift_x * dt;
    q->bounds.y += q->drift_y * dt;

    /* Bounce off the four walls of [0,1] space */
    if (q->bounds.x < 0.0f) {
        q->bounds.x =  0.0f;
        q->drift_x  =  fabsf(q->drift_x);   /* reflect: move right      */
    }
    if (q->bounds.y < 0.0f) {
        q->bounds.y =  0.0f;
        q->drift_y  =  fabsf(q->drift_y);   /* reflect: move down        */
    }
    if (q->bounds.x + q->bounds.w > 1.0f) {
        q->bounds.x = 1.0f - q->bounds.w;
        q->drift_x  = -fabsf(q->drift_x);  /* reflect: move left         */
    }
    if (q->bounds.y + q->bounds.h > 1.0f) {
        q->bounds.y = 1.0f - q->bounds.h;
        q->drift_y  = -fabsf(q->drift_y);  /* reflect: move up           */
    }
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* no pixel-space physics; unused           */
    if (s->paused) return;

    s->time_in_phase += dt;

    if (s->phase == PHASE_INSERT) {
        s->next_insert_in -= dt;
        if (s->next_insert_in <= 0.0f && s->points_inserted < DEMO_POINT_COUNT) {
            insert_random_point(s);
            if (s->points_inserted >= DEMO_POINT_COUNT)
                scene_advance_phase(s);
        }
    } else {   /* PHASE_QUERY */
        advance_query_box(&s->query, dt);

        /* Run the range query every tick so the result stays current */
        s->query_result_count = 0;
        tree_query(g_root, s->query.bounds,
                   s->query_results, &s->query_result_count, QUERY_RESULT_CAP);

        if (s->time_in_phase > QUERY_PHASE_DURATION)
            scene_reset(s);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_hz)
{
    erase();
    scene_draw(sc, s->cols, s->rows);

    /* ── HUD banner (top two rows) ─────────────────────────────────── */
    const char *phase_label = (sc->phase == PHASE_INSERT) ? "INSERT" : "QUERY";
    char status[80];
    snprintf(status, sizeof status,
             " %5.1f fps  sim:%3d Hz  [%s]  %s ",
             fps, sim_hz, phase_label, sc->paused ? "PAUSED" : "running");

    int hud_start_col = s->cols - INFO_PANEL_COLS - (int)strlen(status);
    if (hud_start_col < 0) hud_start_col = 0;

    attron(COLOR_PAIR(CP_PANEL) | A_BOLD);
    mvprintw(0, hud_start_col, "%s", status);
    mvprintw(1, 0, " Quadtree  capacity=%d pts/leaf  max depth=%d  demo: %d pts",
             LEAF_CAPACITY, MAX_TREE_DEPTH, DEMO_POINT_COUNT);
    attroff(COLOR_PAIR(CP_PANEL) | A_BOLD);

    /* ── control hint (bottom row) ──────────────────────────────────── */
    attron(COLOR_PAIR(CP_PANEL) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  space:pause  n:next/reset  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(CP_PANEL) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_hz;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            s->paused = !s->paused;    break;
    case 'n': case 'N':  scene_advance_phase(s);    break;
    case 'r': case 'R':  scene_reset(s);            break;
    case ']':
        if (app->sim_hz < SIM_HZ_MAX) app->sim_hz += SIM_HZ_STEP;
        break;
    case '[':
        if (app->sim_hz > SIM_HZ_MIN) app->sim_hz -= SIM_HZ_STEP;
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
    app->sim_hz  = SIM_HZ_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_start = clock_ns();
    int64_t sim_accum   = 0;       /* nanoseconds "owed" to the physics sim  */
    int64_t fps_accum   = 0;       /* elapsed ns inside the current fps window */
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            frame_start = clock_ns();
            sim_accum   = 0;
        }

        /* ── measure elapsed time since last frame ────────────────── */
        int64_t now      = clock_ns();
        int64_t elapsed  = now - frame_start;
        frame_start      = now;
        if (elapsed > 100 * NS_PER_MS) elapsed = 100 * NS_PER_MS; /* pause guard */

        /* ── fixed-step physics accumulator ──────────────────────── */
        int64_t tick_duration_ns = TICK_NS(app->sim_hz);
        float   tick_duration_s  = (float)tick_duration_ns / (float)NS_PER_SEC;

        sim_accum += elapsed;
        while (sim_accum >= tick_duration_ns) {
            if (!app->scene.paused)
                scene_tick(&app->scene, tick_duration_s,
                           app->screen.cols, app->screen.rows);
            sim_accum -= tick_duration_ns;
        }

        /* ── fps counter (500 ms sliding window) ──────────────────── */
        frame_count++;
        fps_accum += elapsed;
        if (fps_accum >= FPS_WINDOW_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── sleep to cap render rate at 60 fps ──────────────────── */
        int64_t time_spent = clock_ns() - frame_start + elapsed;
        clock_sleep_ns(NS_PER_SEC / 60 - time_spent);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_hz);

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
