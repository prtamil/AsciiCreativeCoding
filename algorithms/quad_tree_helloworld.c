/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * algorithms/quad_tree_helloworld.c — Quadtree hello-world: basic operations
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
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/quad_tree_helloworld.c \
 *       -o qt_hello -lncurses -lm
 *
 * §1 config  §2 clock  §3 color   §4 canvas  §5 quadtree  §6 scene
 * §7 frame   §8 app
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
 * References
 * ──────────
 *   ── Quadtrees: original + canonical ─────────────────────────────
 *   [1] Finkel, R. A. & Bentley, J. L. (1974), "Quad trees: a data
 *       structure for retrieval on composite keys", Acta Informatica
 *       4(1), pp. 1-9 — the ORIGINAL quadtree paper.  Short and
 *       readable; the best place to start.
 *   [2] Samet, H. (1984), "The quadtree and related hierarchical
 *       data structures", ACM Computing Surveys 16(2), pp. 187-260
 *       — the canonical SURVEY paper.  Covers the family of variants
 *       (point, region, MX, PR) we did NOT implement.
 *   [3] Samet, H. (2006), "Foundations of Multidimensional and Metric
 *       Data Structures", Morgan Kaufmann — the encyclopaedic
 *       reference for everything spatial; §1.4, §2.1 cover quadtrees
 *       in exhaustive depth.
 *
 *   ── Computational geometry textbooks ────────────────────────────
 *   [4] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 14 ("Quadtrees: Non-Uniform Mesh
 *       Generation") gives proofs of subdivision cost and range-query
 *       bounds.
 *   [5] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge — practical C-implementation companion to [4].
 *
 *   ── Pool allocator (the no-malloc design in §5) ─────────────────
 *   [6] Knuth, D. E. (1997), "The Art of Computer Programming, Vol. 1:
 *       Fundamental Algorithms" (3rd ed.), Addison-Wesley — §2.5
 *       "Dynamic Storage Allocation" introduces the pool / free-list
 *       families this file's NodePool is the simplest member of.
 *   [7] Nystrom, R. (2014), "Game Programming Patterns" — the
 *       "Object Pool" chapter (gameprogrammingpatterns.com/object-pool)
 *       gives the modern motivation: predictable allocation, no GC /
 *       fragmentation, hot-loop friendliness.
 *
 *   ── Animation loop (the §6 phase state machine) ─────────────────
 *   [8] Nystrom, R. (2014), "Game Programming Patterns" — the
 *       "Game Loop" chapter — covers fixed-timestep vs variable
 *       loops and the patterns this file's phase-1/phase-2 state
 *       machine implements.
 *   [9] West, G. (Glenn Fiedler), "Fix Your Timestep!" —
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical online reference for animation-loop pacing
 *       (decoupled simulation tick vs render frame).
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Quadtree — comprehensive
 *       overview of the four common variants and pseudocode for each.
 *
 *   Note on RENDERING: the visualizer uses ncurses direct cell
 *   addressing (mvaddch / mvprintw); no Bresenham, no anti-aliasing.
 *   Borders are axis-aligned only, so no diagonal-line subtleties.
 *   The query rectangle's motion is a Lissajous figure (independent
 *   sinusoidal velocities on each axis); see Lissajous (1857) for
 *   the original paper, though the motion is just two sin/cos calls.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Watch a quadtree GROW.  Each tick a new random point arrives;
 * if it lands in a leaf at full capacity, the leaf SUBDIVIDES
 * into 4 children (NW/NE/SW/SE) — visible as four nested
 * borders appearing.  After all points are inserted, an orange
 * QUERY RECTANGLE bounces around; points inside it glow green
 * and the algorithm prunes whole subtrees that don't overlap.
 *
 * The animation IS the algorithm: every visible event (point
 * arrives, subdivision happens, query result changes) corresponds
 * to one operation of the underlying data structure.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  PHASE 1 (INSERT): once per ~0.4 sec
 *    1. Generate random point (x, y) ∈ [0, 1]².
 *    2. Descend tree: at each non-leaf, route to NW/NE/SW/SE
 *       child whose rect contains the point.
 *    3. At leaf: if leaf has room (count < 3), append; done.
 *       Else: subdivide leaf into 4 children, redistribute
 *       leaf's existing points + the new one into the
 *       correct children, recurse if needed.
 *
 *  PHASE 2 (QUERY): every render frame
 *    1. Move orange rectangle along Lissajous path.
 *    2. Recursively query tree:
 *         if node.rect doesn't overlap rectangle: prune
 *         if leaf: scan; report points inside rectangle
 *         else: recurse into all 4 children
 *    3. Render: green for reported points, faded for others.
 *
 *  After ~22 sec of phase 2, auto-reset to phase 1 with a fresh
 *  empty tree.
 *
 * KEY FORMULAS
 * ────────────
 *   Quadrant routing: given point (x, y) and node rect
 *                     [x0, x0+w] × [y0, y0+h]:
 *     mid_x = x0 + w/2;  mid_y = y0 + h/2
 *     west  = (x < mid_x);  north = (y < mid_y)
 *     index: NW=0, NE=1, SW=2, SE=3
 *
 *   Bounding-box overlap (rect_overlap):
 *     R1 ∩ R2 ≠ ∅ iff
 *       R1.x_min ≤ R2.x_max AND R1.x_max ≥ R2.x_min
 *       AND same for y
 *
 *   Pool allocator (NodePool in §5):
 *     pool.nodes[NODE_POOL_SIZE] fixed-size storage
 *     pool.count = next-free slot (also total alive nodes)
 *     allocate: idx := count++;  initialise pool.nodes[idx]
 *     reset:    count := 0       (no per-node free — bump back to start)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first.
 *      algorithms/quadtree.c is the LIBRARY-style sibling of this
 *      file (CLI, "press Enter" demo, no animation).  This file is
 *      the ANIMATED ncurses version.  Same algorithm.
 *      algorithms/bsp_tree.c and algorithms/kd_tree.c cover the
 *      2-children alternatives.
 *   2. §5 quadtree — tree_insert + tree_query + subdivide.
 *      THE HEART of this file.  Each driver carries a pseudocode
 *      docblock; read those before the bodies.
 *   3. §6 scene — phase 1 (INSERT) + phase 2 (QUERY) state machine.
 *   4. §4 canvas — the screen-space rendering of the [0, 1]² tree.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   sc                            the Scene struct, passed by pointer to
 *                                 every sim / draw / input function.
 *   sc->pool.nodes[]              static node storage (no malloc).
 *   sc->pool.count                next-free slot, also = total alive nodes.
 *   sc->pool.root_idx             index of the root node in pool.nodes[].
 *   QuadNode.children[4]          NW/NE/SW/SE indices into the pool
 *                                 (or NO_CHILD = -1 for a leaf).
 *   QuadNode.point_count, .points leaf data — only valid when leaf
 *                                 (i.e. children[NW] == NO_CHILD).
 *   sc->query                     the orange bouncing query rectangle.
 *   sc->query_results[], .count   points found by the most recent query.
 *   sc->sim_hz                    user-adjustable tick rate (via [/]).
 *
 * Background you need
 * ───────────────────
 *   - Recursion + tree pointer manipulation.
 *   - Static memory pool (no malloc/free).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Octrees (3-D analogue).
 *   - Loose / point-region quadtree variants.
 *   - Real-time spatial database systems (R-trees etc.).
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
/*
 * The screen is partitioned into:
 *   top HUD_TOP_ROWS rows      — data banner (live stats + static config)
 *   bottom HUD_BOT_ROWS row    — action key hints
 *   between them               — the tree canvas (left) + info panel (right)
 *
 * The two HUD bands follow the project HUD Standard (CLAUDE.md):
 *   top    = bright yellow + A_BOLD (CP_HUD)
 *   bottom = bright cyan   + A_BOLD (CP_HINT)
 *
 * canvas_make in §4 clips the tree-area rows to [HUD_TOP_ROWS,
 * rows − HUD_BOT_ROWS) so animation never overdraws either bar.
 */
enum {
    INFO_PANEL_COLS = 28,  /* right-side panel width in terminal columns */
    HUD_TOP_ROWS    =  2,  /* row 0: live status  ;  row 1: static config */
    HUD_BOT_ROWS    =  1,  /* last row: key-hint action bar               */
};

/* ── color pair names ──────────────────────────────────────────────── */
/*
 * Using named constants instead of raw integers eliminates magic numbers
 * from every draw call.  The values 1..N are the ncurses pair indices
 * assigned in color_init().
 *
 * CP_HUD and CP_HINT are reserved for the top/bottom HUD bars per the
 * project HUD Standard — kept SEPARATE from CP_PANEL (the right-side
 * info panel) so the three roles can be re-themed independently.
 */
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

/* ===================================================================== */
/* §4  canvas — safe drawing within the tree area                        */
/* ===================================================================== */

/*
 * TreeCanvas — the rectangular region of the terminal where the tree
 * is drawn (left band, between the top/bottom HUD strips, ending where
 * the right-side info panel begins).
 *
 * Intent
 *   ONE struct that records the four-corner clipping bounds of the
 *   tree visualisation, computed ONCE per frame by canvas_make.  Every
 *   draw helper (canvas_put, draw_node_border, draw_query_box, …) reads
 *   the same bounds without re-deriving them, so:
 *     - the clipping rule is defined in one place,
 *     - resizes are picked up automatically (canvas_make re-runs),
 *     - the (rows, cols) → (top, bottom, right) translation isn't
 *       duplicated in every drawing function.
 *
 * Why EXCLUSIVE upper bounds (bottom_row, right_col)
 *   Standard half-open convention: a write is valid iff
 *     top_row ≤ row < bottom_row   AND   0 ≤ col < right_col
 *   matches the natural for-loop form `for (int r = top; r < bot; r++)`
 *   and lets `bottom_row - top_row` give the height in cells directly
 *   (no off-by-one).
 *
 * Why a STRUCT (not three loose ints in every signature)
 *   Pass-by-value (12 bytes); fits in registers.  Adding a fourth field
 *   later (e.g. a left margin for line numbers) is a one-line edit;
 *   no signature churn at every draw_* call site.
 *
 * Members
 *   top_row     first row belonging to the tree area    (INCLUSIVE; = HUD_TOP_ROWS)
 *   bottom_row  one-past-last row of the tree area      (EXCLUSIVE; = rows − HUD_BOT_ROWS)
 *   right_col   one-past-last column of the tree area   (EXCLUSIVE; = cols − INFO_PANEL_COLS)
 *
 * Invariants (after canvas_make)
 *   top_row ≥ HUD_TOP_ROWS  and  top_row < bottom_row
 *   right_col ≥ 2  (canvas_make floors it so even a tiny terminal
 *                   gives draw helpers a non-zero work area)
 *
 *   Normalised tree coords (nx, ny) ∈ [0, 1]² map to terminal cells
 *   via canvas_col(cv, nx) and canvas_row(cv, ny).
 */
typedef struct {
    int top_row;    /* first row belonging to the tree area (INCLUSIVE)  */
    int bottom_row; /* one-past-last row  (EXCLUSIVE upper bound)        */
    int right_col;  /* one-past-last col  (EXCLUSIVE upper bound)        */
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

/* ===================================================================== */
/* §5  quadtree — types, pool, core operations, statistics               */
/* ===================================================================== */

/* ── primitive geometry types ──────────────────────────────────────── */

/*
 * Vec2 — a 2-D point in NORMALISED tree space [0, 1] × [0, 1].
 *
 * Intent
 *   The atomic datum the quadtree stores.  Every leaf holds an array
 *   of up to LEAF_CAPACITY of these.  No label / payload here — the
 *   demo doesn't need to identify individual points, only paint and
 *   query them.
 *
 * Why NORMALISED [0, 1] coordinates (not pixel / cell)
 *   Tree state lives in a RESIZE-INVARIANT coordinate system.  The
 *   terminal can shrink or grow at any time; the tree's data and
 *   structure are unchanged — only the canvas_col / canvas_row
 *   mapping moves.  Without normalisation we'd have to rebuild the
 *   tree on every SIGWINCH.
 *
 * Why FLOAT (not int)
 *   Continuous space — points can land anywhere in [0, 1], not on a
 *   pixel grid.  Subdivision halves the coordinate range, so deep
 *   nodes use sub-cell precision; integers would round-collapse those.
 *
 * Convention
 *   y axis points DOWN (terminal convention): higher y → lower on
 *   screen.  This matches the QuadNode child naming (NW = small y,
 *   SW = large y).
 *
 * Why pass-by-value
 *   8 bytes; fits in registers on every modern ABI.  No aliasing
 *   concerns when both query_results[] entries and node points sit
 *   in the same caller frame.
 *
 * Members
 *   x  horizontal coordinate; 0 ≤ x ≤ 1  (left → right)
 *   y  vertical   coordinate; 0 ≤ y ≤ 1  (top → bottom)
 */
typedef struct {
    float x;     /* horizontal, [0, 1] — left → right    */
    float y;     /* vertical,   [0, 1] — top → bottom    */
} Vec2;

/*
 * Rect — axis-aligned rectangle in NORMALISED tree space.
 *
 * Intent
 *   The boundary of one quadtree node.  Stays constant from
 *   node_alloc to pool_reset — re-subdivision allocates fresh child
 *   nodes each with its own (smaller) Rect; the parent's Rect is
 *   unchanged.  Also used as the bouncing query rectangle in
 *   QueryBox.
 *
 * Why HALF-OPEN intervals (upper EXCLUSIVE)
 *   Subdivision must EXACTLY partition the parent into four children
 *   that share no cell and together cover every cell.  Half-open
 *   intervals achieve this trivially:
 *     NW: [x,     x+w/2)  ×  [y,     y+h/2)
 *     NE: [x+w/2, x+w  )  ×  [y,     y+h/2)
 *     SW: [x,     x+w/2)  ×  [y+h/2, y+h  )
 *     SE: [x+w/2, x+w  )  ×  [y+h/2, y+h  )
 *   With inclusive bounds the four midpoint cells would each belong
 *   to TWO children simultaneously — ambiguous routing, duplicated
 *   points after subdivision.  See `rect_contains_point` below for
 *   the predicate that enforces this.
 *
 * Why STORE (x, y, w, h) — not (x1, y1, x2, y2)
 *   subdivide() halves `w` and `h` at every level; storing width
 *   directly avoids a `(x2 - x1)` calculation in every split.
 *   The conversion happens once at the `rect_contains_point` call.
 *
 * Members
 *   x, y   top-left corner, INCLUSIVE          (0 ≤ x, y ≤ 1)
 *   w, h   width and height; rect covers [x, x+w) × [y, y+h)
 *
 * Invariants
 *   w ≥ 0 and h ≥ 0.  After repeated subdivision, w and h shrink by
 *   half each level (d levels deep ⇒ w = w_root / 2^d).  At depth
 *   MAX_TREE_DEPTH the leaf accepts overflow rather than splitting
 *   into 0-sized children — see the depth check in tree_insert.
 *
 * Reference: Finkel & Bentley (1974) [1] — the half-open convention
 *   IS the original PR-quadtree's subdivision rule.
 */
typedef struct {
    float x;     /* left   edge, INCLUSIVE — 0 ≤ x ≤ 1   */
    float y;     /* top    edge, INCLUSIVE — 0 ≤ y ≤ 1   */
    float w;     /* width;  rect covers [x, x+w)         */
    float h;     /* height; rect covers [y, y+h)         */
} Rect;

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

/*
 * QuadrantRects — the four child rectangles a subdivision produces.
 *
 *   Output of compute_quadrant_rects: the four sub-rects that EXACTLY
 *   partition a parent Rect (half-open, so no cell belongs to two
 *   children and no cell is uncovered).  Naming this group as a struct
 *   lets the geometry primitive return ONE value, and decouples the
 *   "compute four child rects" math (geometry layer) from the
 *   "allocate four child nodes" step (tree-building layer).
 *
 *   Field order (nw / ne / sw / se) MATCHES QuadNode.children's NW/NE/
 *   SW/SE indexing so subdivide's allocation lines read as obvious
 *   matched pairs:
 *       parent->children[NW] = node_alloc(pool, q.nw, …);
 *       parent->children[NE] = node_alloc(pool, q.ne, …);   ← same name
 *
 *   Reference: Finkel & Bentley (1974) [1] — the canonical PR-quadtree
 *   subdivision rule this struct embodies.
 */
typedef struct {
    Rect nw;    /* NW (top-left)     quadrant of the parent              */
    Rect ne;    /* NE (top-right)    quadrant of the parent              */
    Rect sw;    /* SW (bottom-left)  quadrant of the parent              */
    Rect se;    /* SE (bottom-right) quadrant of the parent              */
} QuadrantRects;

/*
 * compute_quadrant_rects — partition `r` into NW/NE/SW/SE half-open
 * sub-rectangles that share no cell and together cover `r` exactly.
 *
 *   Pseudocode:
 *     half_w := r.w / 2;          half_h := r.h / 2
 *     mid_x  := r.x + half_w;     mid_y  := r.y + half_h
 *     return {
 *       NW: [r.x,   mid_x) × [r.y,   mid_y),
 *       NE: [mid_x, r.x+w) × [r.y,   mid_y),
 *       SW: [r.x,   mid_x) × [mid_y, r.y+h),
 *       SE: [mid_x, r.x+w) × [mid_y, r.y+h),
 *     }
 *
 *   THE geometry primitive of the quadtree.  Used by subdivide.
 *   Half-open semantics make the partition EXACT — every cell in
 *   `r` lands in exactly one of the four output rects.
 */
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
 * QuadNode — one node in the quadtree.  LEAF or INTERNAL.
 *
 * Intent
 *   The fundamental tree element.  A node owns a region (bounds) and
 *   is either a LEAF (points[] holds up to LEAF_CAPACITY data points,
 *   children[] are all NO_CHILD) or an INTERNAL node (children[] are
 *   non-NO_CHILD, points[] is unused).  A leaf becomes internal via
 *   subdivide() when a (LEAF_CAPACITY + 1)th point lands in its region.
 *
 * Leaf-vs-internal discrimination
 *   Encoded in ONE check: `children[NW] == NO_CHILD` means leaf.  No
 *   separate `is_leaf` flag — the all-children-absent state IS the
 *   leaf marker.  subdivide() flips a leaf to internal in one
 *   allocation burst; nothing in the codebase creates the mixed state.
 *   The `node_is_leaf` helper below names this check explicitly.
 *
 * Why DATA + CHILDREN in the same struct (vs separate Leaf / Internal)
 *   - Single allocation path (node_alloc returns one type).
 *   - No tagged union; the discriminator is the children-empty test
 *     the recursive walks were going to do anyway.
 *   - Indexed-allocation friendly: NodePool's bump allocator hands
 *     out one node-sized slot regardless of whether the caller will
 *     keep it as a leaf or eventually subdivide it.
 *   Cost: each internal node carries a wasted points[LEAF_CAPACITY]
 *   buffer (LEAF_CAPACITY × sizeof(Vec2) = 24 bytes at LEAF_CAPACITY=3).
 *   At pool size 512 nodes this caps at ~12 KB wasted in the worst
 *   case; negligible.
 *
 * Why INDICES (not pointers) for children
 *   The pool storage (NodePool.nodes[]) lives inside Scene, which
 *   lives on the stack.  Pointer fields would have to be patched if
 *   Scene moved; indices are position-independent.  Also: NO_CHILD
 *   (-1) is a more obvious "absent" sentinel than NULL inside a
 *   packed numeric field.
 *
 * Compass-direction naming (NW, NE, SW, SE)
 *   Mirrors the on-screen geometry (y-down terminal convention):
 *     NW = north-west (small x, small y) → top-left
 *     NE = north-east (large x, small y) → top-right
 *     SW = south-west (small x, large y) → bottom-left
 *     SE = south-east (large x, large y) → bottom-right
 *   subdivide() allocates the four children in this exact order so
 *   the code reads top-left → top-right → bottom-left → bottom-right
 *   (page-scanning order).
 *
 * Lifecycle (a node's life is a 3-state machine)
 *
 *   node_alloc()
 *      │
 *      ▼
 *   ┌──────────────────┐  insert past LEAF_CAPACITY  ┌──────────────┐
 *   │ LEAF (cnt ≤ CAP) │ ─────── subdivide() ──────► │   INTERNAL    │
 *   │ points[] valid   │                             │  4 children   │
 *   │ children=NO_CHILD│                             │  point_count=0│
 *   └──────────────────┘                             └──────────────┘
 *      │                                                       │
 *      ▼                                                       ▼
 *   pool_reset() — bumps NodePool.count back to 0; all nodes drop
 *                  (no per-node free needed — that's why a pool wins
 *                  over malloc/free for short-lived demos like this)
 *
 *   The LEAF → INTERNAL transition is ONE-WAY here: this demo never
 *   merges children back into a leaf (a "coarsening" operation some
 *   quadtree variants support).  See Samet [2] §3.2 for variants.
 *
 * Members
 *   bounds                  region this node owns in [0, 1]² tree space;
 *                           constant from node_alloc to pool_reset
 *   points[LEAF_CAPACITY]   point storage; only entries [0, point_count)
 *                           are valid; only meaningful when this is a LEAF
 *   point_count             0 ≤ point_count ≤ LEAF_CAPACITY when leaf;
 *                           always 0 when internal
 *   children[NUM_CHILDREN]  NW/NE/SW/SE indices into NodePool.nodes[];
 *                           ALL == NO_CHILD ⇔ this is a leaf;
 *                           ALL != NO_CHILD ⇔ this is an internal node
 *   depth                   0 = root; depth = parent.depth + 1.  Used by
 *                           the depth-limit check in tree_insert (caps
 *                           at MAX_TREE_DEPTH to prevent infinite splits
 *                           on coincident points) and by the renderer
 *                           to pick border colour / intensity.
 *
 * Invariants
 *   (children[NW] == NO_CHILD) ⇔ same for NE, SW, SE
 *   children[c] != NO_CHILD → point_count == 0     (internal nodes hold no points)
 *   children[c] == NO_CHILD → 0 ≤ point_count ≤ LEAF_CAPACITY
 *   Every point in points[] satisfies rect_contains_point(bounds, …)
 *   Each child's bounds is exactly one quadrant of `bounds`
 *   depth ≤ MAX_TREE_DEPTH
 *
 * References
 *   [1] Finkel & Bentley 1974 — the original PR-quadtree node layout.
 *   [2] Samet 1984 §3 — variants (region quadtree, MX quadtree, etc.)
 *       that store data differently; ours is the point-region (PR)
 *       quadtree with bucket capacity > 1.
 *   [7] Nystrom "Object Pool" — the pool-allocator pattern the
 *       NodePool implements; per-node `int children[]` indices are
 *       chosen here precisely to make the pool relocatable.
 */
typedef struct {
    Rect bounds;                    /* region this node owns in [0,1]×[0,1] */
    Vec2 points[LEAF_CAPACITY];     /* data points (valid only when leaf)   */
    int  point_count;               /* 0..LEAF_CAPACITY leaf; 0 if internal */
    int  children[NUM_CHILDREN];    /* NW/NE/SW/SE pool indices; NO_CHILD if absent */
    int  depth;                     /* 0 = root; ≤ MAX_TREE_DEPTH           */
} QuadNode;

static inline bool node_is_leaf(const QuadNode *n) { return n->children[NW] == NO_CHILD; }
static inline bool node_is_full(const QuadNode *n) { return n->point_count == LEAF_CAPACITY; }

/* ── node pool ─────────────────────────────────────────────────────── */

/*
 * NodePool — fixed-capacity storage for the whole quadtree.
 *
 * Intent
 *   Every node the tree will ever need is pre-allocated here at
 *   program start.  No malloc/free in the hot loop; resetting the
 *   tree is O(1) (set count to 0 and re-create the root).
 *
 * Why a STATIC POOL (not malloc per node)
 *   - Predictable allocation: no fragmentation, no allocator lock,
 *     no GC pauses.  Every node-add costs one bump of `count`.
 *   - Cache-friendly: all nodes sit in one contiguous block, so
 *     traversal pulls neighbours into the same cache line.
 *   - Deterministic upper bound: NODE_POOL_SIZE caps the worst case
 *     and pool_has_room enforces it explicitly.
 *   - Reset is one assignment, not a tree-walking free().
 *
 * Why INDICES (not pointers) for children
 *   When the pool is part of Scene and Scene lives on the stack, the
 *   pool's address changes between runs.  Pointer fields would have
 *   to be patched; indices are position-independent.  Also: -1 (the
 *   NO_CHILD sentinel) is a more obvious "absent" than NULL inside
 *   a packed numeric field.
 *
 * Members
 *   nodes[NODE_POOL_SIZE]   the storage; entries [0, count) are valid
 *   count                   next-free slot (also the current node count)
 *   root_idx                index of the root node (always 0 after reset,
 *                           but stored explicitly so a caller doesn't
 *                           have to know that)
 *
 * Invariants
 *   0 ≤ count    ≤ NODE_POOL_SIZE
 *   0 ≤ root_idx <  count   (after pool_reset has run at least once)
 *   nodes[root_idx].depth == 0
 *
 * References [6] Knuth TAOCP Vol. 1 §2.5 — pool/free-list allocators;
 *   [7] Nystrom — "Object Pool" pattern.
 */
typedef struct {
    QuadNode nodes[NODE_POOL_SIZE];
    int      count;       /* next-free slot, also = total alive nodes      */
    int      root_idx;    /* index of the root; always 0 after pool_reset  */
} NodePool;

/*
 * pool_has_room — true iff `slots_needed` more nodes will fit.
 *   Used by tree_insert before calling subdivide (which always
 *   allocates NUM_CHILDREN = 4 fresh nodes).
 */
static bool pool_has_room(const NodePool *pool, int slots_needed)
{
    return pool->count + slots_needed <= NODE_POOL_SIZE;
}

/*
 * node_alloc — bump-allocate one fresh node initialised as a leaf.
 *
 *   Pseudocode:
 *     if pool is full: return NO_CHILD
 *     idx := count++
 *     pool->nodes[idx] := { bounds, no points, no children, depth }
 *     return idx
 *
 *   The "bump" allocator: just increment a counter.  Constant-time,
 *   no free list, no headers.  Reset by setting count back to 0.
 */
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

/*
 * pool_reset — start over: drop every node and recreate the root.
 *   O(1) — does NOT zero the storage, just resets count to 0 and
 *   allocates a fresh root.  Old node memory is silently reused
 *   on next allocation.
 */
static void pool_reset(NodePool *pool)
{
    pool->count    = 0;
    Rect full_space = { .x = 0.0f, .y = 0.0f, .w = 1.0f, .h = 1.0f };
    pool->root_idx = node_alloc(pool, full_space, /*depth=*/0);
}

/* ── core operations ───────────────────────────────────────────────── */

/* Forward declaration: subdivide() calls tree_insert() for redistribution */
static bool tree_insert(NodePool *pool, int node_idx, float x, float y);

/*
 * subdivide — split a full leaf into four equal quadrant children.
 *
 *   Pseudocode:
 *     allocate four child nodes for the four sub-rectangles
 *     wire them as parent->children[NW..SE]
 *     snapshot parent's points; clear parent->point_count
 *     for each displaced point: tree_insert into the matching child
 *
 *   After splitting, the parent becomes an INTERNAL (non-leaf) node
 *   and its existing points are redistributed into the appropriate
 *   child.  Precondition: pool has room for NUM_CHILDREN more nodes
 *   (checked by the caller, tree_insert).
 *
 * Before:                  After:
 *   [parent: 3 pts]          [parent: 0 pts]
 *                             /    |    |    \
 *                           [NW]  [NE]  [SW]  [SE]
 *                         (points redistributed among children)
 */
/*
 * redistribute_points_into_children — snapshot the parent's points,
 * clear its count, then re-insert each into the new children.
 *
 *   Pseudocode:
 *     snapshot displaced[] := parent->points[0 .. count − 1]
 *     reset parent->point_count := 0   (so the children-walk below
 *                                       treats parent as internal)
 *     for each displaced point p:
 *       try tree_insert(child_c, p) for c in NW, NE, SW, SE
 *       first success wins (exactly one child's boundary contains p
 *       by the QuadrantRects partition invariant)
 *
 *   The snapshot is necessary because tree_insert overwrites point
 *   storage on the children we descend into; copying parent's points
 *   out first lets us route them safely.
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

/*
 * subdivide — split a full leaf into four equal quadrant children.
 *
 *   Pseudocode:
 *     q := compute_quadrant_rects(parent->bounds)      ← geometry
 *     allocate four child nodes with bounds q.nw, q.ne, q.sw, q.se
 *     redistribute_points_into_children                ← route old points
 *
 *   After splitting, `parent` is an INTERNAL node:
 *     point_count == 0;  children[] all non-NO_CHILD
 *
 *   Precondition: pool has room for NUM_CHILDREN more nodes (checked
 *   by the caller, tree_insert, via pool_has_room).
 */
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

/*
 * try_append_to_leaf — store (x, y) in node->points[] if there's room.
 *
 *   Pseudocode:
 *     if leaf at capacity:        return false       (caller subdivides)
 *     node->points[point_count++] := (x, y)
 *     return true                                    (appended)
 *
 *   The fast path: just one array write + one counter bump.  When
 *   this returns false the leaf is full and the caller (tree_insert)
 *   triggers subdivide().
 *
 *   Precondition: `node` is a LEAF (caller checked node_is_leaf).
 */
static bool try_append_to_leaf(QuadNode *node, float x, float y)
{
    if (node_is_full(node)) return false;
    node->points[node->point_count++] = (Vec2){x, y};
    return true;
}

/*
 * route_into_quadrant_child — deliver (x, y) to the unique child
 * whose boundary contains it.
 *
 *   Pseudocode:
 *     for c in {NW, NE, SW, SE}:
 *       if tree_insert(child[c], x, y):  return true
 *     return false                       (unreachable when called correctly)
 *
 *   By the QuadrantRects partition invariant EXACTLY ONE of the four
 *   children's boundaries contains (x, y), so the first success wins
 *   and the loop short-circuits.  Reaching the false return means the
 *   caller's boundary check at the top of tree_insert succeeded but
 *   no child accepted — that's a bug (or the caller broke the
 *   precondition that this node is internal).
 *
 *   Precondition: `node` is INTERNAL (all four children non-NO_CHILD).
 */
static bool route_into_quadrant_child(NodePool *pool, QuadNode *node,
                                      float x, float y)
{
    for (int c = 0; c < NUM_CHILDREN; c++)
        if (tree_insert(pool, node->children[c], x, y))
            return true;
    return false;
}

/*
 * tree_insert — add point (x, y) into the subtree rooted at node_idx.
 *
 *   Pseudocode (dispatch on node kind):
 *     if node missing OR (x,y) outside bounds:  return false
 *     if leaf:
 *       if try_append_to_leaf succeeds:         return true
 *       if depth-limit / pool-limit hit:        return false
 *       subdivide(node)                         ← now internal
 *       (fall through to route_into_quadrant_child)
 *     return route_into_quadrant_child(node, x, y)
 *
 *   Returns true when accepted, false when out-of-bounds OR resource-
 *   limited.  The false-return path on out-of-bounds is what lets
 *   redistribute_points_into_children walk the 4 children and let
 *   the geometrically-correct child accept each displaced point.
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
 * tree_query — collect every point inside search_area into results[].
 *
 *   Pseudocode (depth-first, prune by rect_overlaps):
 *     if missing node OR results full:           return
 *     if NOT rect_overlaps(node->bounds, query): return   ← PRUNE
 *     for each stored point p:
 *       if rect_contains_point(query, p):  results[count++] := p
 *     recurse into all 4 children
 *
 *   The pruning step gives quadtrees their efficiency: when a node's
 *   region doesn't overlap the query, the WHOLE SUBTREE is skipped in
 *   O(1).  Cost: O(log N + k) average, where k = matches returned.
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

/* ── statistics (called once per frame; O(nodes), negligible cost) ── */

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

/* ===================================================================== */
/* §6  scene — demo state, drawing helpers, tick, render                 */
/* ===================================================================== */

/* ── demo phase ────────────────────────────────────────────────────── */

/*
 * DemoPhase — the two halves of the demo's state machine.
 *
 * Intent
 *   The demo cycles forever between two visually distinct phases so
 *   the viewer sees BOTH the tree-construction and tree-query
 *   operations animated.  scene_tick dispatches on this enum:
 *     PHASE_INSERT  → grow the tree one point at a time
 *     PHASE_QUERY   → bounce the search rectangle, refresh results
 *
 *   State transitions (see scene_tick / scene_advance_phase):
 *
 *     PHASE_INSERT ─── DEMO_POINT_COUNT inserted ──► PHASE_QUERY
 *          ▲                                              │
 *          │                                              │
 *          └─── QUERY_PHASE_DURATION elapsed (scene_reset)┘
 *
 *   Manual 'n' (next) and 'r' (reset) jump along the same edges.
 *
 * Why an enum (not int 0/1)
 *   Self-documenting at every comparison site: `phase == PHASE_INSERT`
 *   reads as English.  Adding a third phase (e.g. PHASE_REMOVE) later
 *   becomes a one-line struct edit; comparisons still compile-check.
 *
 * Members
 *   PHASE_INSERT  growing the tree: random points stream in every
 *                 SECONDS_PER_INSERTION seconds; subdivisions fire
 *                 when a leaf overflows.
 *   PHASE_QUERY   tree is full; the orange query rectangle bounces
 *                 around and we run tree_query every tick to refresh
 *                 which points glow green.
 */
typedef enum {
    PHASE_INSERT,   /* animate point insertions; watch subdivisions      */
    PHASE_QUERY,    /* animate bouncing range-query rectangle            */
} DemoPhase;

/* ── query box ─────────────────────────────────────────────────────── */

/*
 * QueryBox — the bouncing search rectangle in PHASE_QUERY.
 *
 * Intent
 *   Combines the rectangle's GEOMETRY (where + how big) with its
 *   MOTION (drift velocity).  scene_tick calls advance_query_box every
 *   frame to update position; tree_query reads `bounds` to find which
 *   points are inside.
 *
 *   The motion is a simple AXIS-ALIGNED BOUNCE: each velocity component
 *   flips sign when the rect hits a wall of [0, 1]² space.  Not strictly
 *   a Lissajous figure (those use sinusoidal speeds, not constant
 *   speeds with reflection) but the visual rhythm is similar.
 *
 * Why STORE velocity in the struct (not derive from time)
 *   The bounce changes velocity reactively (sign flip at wall contact),
 *   so it has to be stateful.  Recomputing from time + initial seed
 *   would either re-derive the bounce sequence each frame (expensive)
 *   or precompute a path (precludes responsive resize).
 *
 * Why drift_x / drift_y as separate FLOATS (not a Vec2)
 *   The two axes are reflected independently — each wall hit flips one
 *   component without touching the other.  Treating them as a single
 *   Vec2 would invite a `v = -v` mistake that reflects BOTH.  Distinct
 *   names + distinct fields → distinct semantics.
 *
 * Members
 *   bounds    current position and size in tree space [0, 1]²
 *             (`bounds.w` and `bounds.h` stay constant for the run —
 *              QUERY_RECT_WIDTH / HEIGHT — only `bounds.x` and
 *              `bounds.y` move)
 *   drift_x   horizontal velocity, tree-space units per second.
 *             Sign flips at left/right wall contact.
 *   drift_y   vertical   velocity, tree-space units per second.
 *             Sign flips at top/bottom wall contact.
 *
 * Invariants (after advance_query_box runs each tick)
 *   0 ≤ bounds.x ≤ 1 − bounds.w
 *   0 ≤ bounds.y ≤ 1 − bounds.h
 *   |drift_x| and |drift_y| are constant for the lifetime of the
 *   QueryBox (only the SIGNS flip)
 *
 * Reference: Lissajous (1857) for the general harmonic-motion family;
 *   our rectangle uses constant speeds with reflection rather than
 *   true Lissajous sinusoidal speeds.
 */
typedef struct {
    Rect  bounds;    /* current position + size in tree space            */
    float drift_x;   /* horizontal velocity, tree-units/second           */
    float drift_y;   /* vertical   velocity, tree-units/second           */
} QueryBox;

/* ── scene state ───────────────────────────────────────────────────── */

/*
 * Scene — owns ALL persistent simulation + UI state for one run.
 *
 * Intent
 *   One instance lives in main() and is passed by pointer to every
 *   simulation, draw, and input function.  No file-scope mutables
 *   for the simulation; only the signal-handler flags (g_quit /
 *   g_resize in §8) stay as globals because POSIX signal handlers
 *   can't be passed a pointer.
 *
 * Sub-structures
 *   NodePool   the fixed-size tree storage (§5) — embedded here so
 *              the tree's lifetime matches the Scene's
 *   QueryBox   the bouncing search rectangle (geometry + velocity)
 *
 * Members
 *   ── Tree (the data structure being demoed) ──────────────────────
 *   pool                  every node lives in pool.nodes[]; root_idx
 *                         identifies the entry point.
 *
 *   ── Demo state machine (phase 1 → phase 2 → reset) ─────────────
 *   phase                 PHASE_INSERT or PHASE_QUERY
 *   time_in_phase         seconds elapsed since the current phase began
 *   next_insert_in        seconds until the next point is added (phase 1)
 *   points_inserted       how many random points have been placed
 *
 *   ── Query box + most recent results ────────────────────────────
 *   query                 the bouncing rectangle (position + velocity)
 *   query_results[]       points found by the most recent tree_query
 *   query_result_count    number of valid entries in query_results
 *
 *   ── UI / pacing ────────────────────────────────────────────────
 *   paused                pause flag; scene_tick is a no-op while true
 *   sim_hz                target simulation rate (Hz); adjusted by [/]
 *
 *   ── Terminal extent (refreshed each frame) ─────────────────────
 *   rows, cols            LINES/COLS snapshot for the current frame
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

/* ── info panel helpers ────────────────────────────────────────────── */

/*
 * PanelWriter — write-cursor for the right-side info panel.
 *
 * Intent
 *   Each `panel_text` / `panel_divider` / `panel_blank` call advances
 *   the cursor down by one row.  The caller never touches the row
 *   counter directly — just calls helpers in document order and they
 *   "type" downward.  This removes per-line (row, col) bookkeeping
 *   that draw_info_panel would otherwise need at every mvprintw.
 *
 *   Same idiom as the cursor in any text-editor write loop, or the
 *   `gl_NextPosition` pattern in old fixed-function GL drawing.
 *
 * Why a STRUCT (not three loose args on every helper)
 *   - Single value to pass: `panel_text(&pw, …)` instead of
 *     `panel_text(&col, &row, last, …)` with three out-params.
 *   - Adding a fourth field later (e.g. an indent column for nested
 *     sections) is a one-line edit; no signature churn.
 *
 * Why `last_row` as a CLIPPING BOUND (not an assertion)
 *   The info panel can overflow on a short terminal — we want the
 *   demo to keep rendering with a TRUNCATED panel, not crash.  Each
 *   helper silently no-ops past `last_row`, so the visual gracefully
 *   degrades.  Same principle as the `valid` flag in
 *   `ChartLayout` from algorithms/network_sim.c.
 *
 * Members
 *   panel_col     left edge of the panel in terminal columns
 *                 (= terminal_cols − INFO_PANEL_COLS)
 *   current_row   next row to write; bumped by every helper
 *   last_row      EXCLUSIVE upper bound; helpers no-op once
 *                 current_row >= last_row (clip rather than crash)
 *
 * Invariants
 *   panel_col >= 0
 *   HUD_TOP_ROWS ≤ current_row  (starts there in panel_begin)
 *   current_row ≤ last_row + 1  (helpers may bump past once, but
 *                                the no-op guard catches the next call)
 */
typedef struct {
    int panel_col;    /* left edge of the panel in terminal columns       */
    int current_row;  /* next row to write; bumped by every helper        */
    int last_row;     /* one-past the last writable row (clip bound)      */
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
 * are copied verbatim from sc->pool.nodes — they are never recomputed.
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
    const QuadNode *node = &sc->pool.nodes[node_idx];

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

/* ── scene_draw ────────────────────────────────────────────────────── */

/*
 * scene_draw — render one frame: tree + (maybe) query box + info panel.
 *
 *   Pseudocode (layered draw, painter's order):
 *     canvas := canvas_make(sc->cols, sc->rows)
 *     draw_tree (root, sc, canvas)        ← borders + points
 *     if PHASE_QUERY:  draw_query_box     ← orange bouncing rect
 *     draw_info_panel                     ← right-side gold panel
 *
 *   Tree first so query box and info panel paint on top of any
 *   border / point glyph that intrudes into their cells.
 */
static void scene_draw(const Scene *sc)
{
    TreeCanvas canvas = canvas_make(sc->cols, sc->rows);

    draw_tree(sc->pool.root_idx, sc, canvas);

    if (sc->phase == PHASE_QUERY)
        draw_query_box(&sc->query, canvas);

    draw_info_panel(sc, sc->cols, sc->rows);
}

/* ── scene lifecycle ───────────────────────────────────────────────── */

/*
 * reset_scene_state_preserving_settings — zero everything in Scene
 * EXCEPT the user-controlled settings (rows/cols/sim_hz).
 *
 *   Pseudocode:
 *     snapshot { rows, cols, sim_hz }
 *     memset(s, 0, sizeof *s)            ← wipe all fields
 *     restore { rows, cols, sim_hz }
 *
 *   The save/wipe/restore idiom: cheaper than listing every field
 *   to clear (which would rot when a new field is added), and
 *   FORCES the reset to be exhaustive — anything not in the
 *   preserve-set goes to its zero value.
 */
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

/*
 * seed_insert_phase — set up the demo to start in PHASE_INSERT.
 *
 *   Initial next_insert_in is shorter than steady-state SECONDS_PER_
 *   INSERTION (×0.25) so the first point appears quickly instead of
 *   the user staring at an empty canvas for a full insert interval.
 */
static void seed_insert_phase(Scene *s)
{
    s->phase          = PHASE_INSERT;
    s->next_insert_in = SECONDS_PER_INSERTION * 0.25f;
}

/*
 * seed_query_box — place the query rectangle at its starting position
 * and velocity.
 *
 *   Position: top-left corner, slightly inset (0.08, 0.08).
 *   Velocity: equal components on each axis, magnitudes scaled by
 *             1/√2 ≈ 0.7071 so the resultant speed equals
 *             QUERY_DRIFT_SPEED exactly (Pythagorean composition).
 *   Result:   diagonal drift toward the opposite corner.
 */
static void seed_query_box(QueryBox *q)
{
    q->bounds  = (Rect){ .x = 0.08f, .y = 0.08f,
                         .w = QUERY_RECT_WIDTH, .h = QUERY_RECT_HEIGHT };
    q->drift_x = QUERY_DRIFT_SPEED * 0.7071f;   /* 1 / sqrt(2) */
    q->drift_y = QUERY_DRIFT_SPEED * 0.7071f;
}

/*
 * scene_reset — wipe scene state and reseed for a fresh PHASE_INSERT run.
 *
 *   Pseudocode (4 named steps):
 *     reset_scene_state_preserving_settings  ← wipe, keep rows/cols/sim_hz
 *     pool_reset                              ← drop every tree node
 *     seed_insert_phase                       ← start the insertion loop
 *     seed_query_box                          ← prime the bouncing rectangle
 *
 *   Called on 'r' (manual reset) and at the end of PHASE_QUERY (auto-
 *   cycle in scene_tick).  Preserves only the terminal extent and the
 *   user's sim_hz preference; everything else is cleared.
 */
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

/* ── scene_tick helpers ────────────────────────────────────────────── */

static void insert_random_point(Scene *s)
{
    /* Keep points away from the very edge so border drawing looks clean */
    float x = 0.03f + (float)(rand() % 9400) / 10000.0f;  /* [0.03, 0.97] */
    float y = 0.03f + (float)(rand() % 9400) / 10000.0f;
    tree_insert(&s->pool, s->pool.root_idx, x, y);
    s->points_inserted++;
    s->next_insert_in = SECONDS_PER_INSERTION;
}

/*
 * integrate_query_box_euler — advance position by velocity for dt seconds.
 *
 *   Explicit Euler integration of a point mass:
 *       x_{t+1} := x_t + v · dt
 *
 *   The kinematic primitive — every motion-of-mass-points solver
 *   reduces to this.  No acceleration term: the query box travels at
 *   constant velocity between wall reflections.
 */
static inline void integrate_query_box_euler(QueryBox *q, float dt)
{
    q->bounds.x += q->drift_x * dt;
    q->bounds.y += q->drift_y * dt;
}

/*
 * bounce_query_box_off_walls — elastic reflection against [0, 1]² walls.
 *
 *   When the box crosses a wall, the corresponding velocity component
 *   flips sign and the position is CLAMPED back into the box:
 *     left  wall (x < 0):           drift_x ← |drift_x|;   x ← 0
 *     right wall (x + w > 1):       drift_x ← −|drift_x|;  x ← 1 − w
 *     top   wall (y < 0):           drift_y ← |drift_y|;   y ← 0
 *     bottom wall (y + h > 1):      drift_y ← −|drift_y|;  y ← 1 − h
 *
 *   Two independent 1-D reflections (one per axis) — the box can hit a
 *   corner and bounce off BOTH walls in the same tick.  Using fabsf
 *   instead of `*= -1` is robust to repeated near-wall ticks: a small
 *   sub-step that doesn't actually escape the wall still has the right
 *   sign after fabsf, so the reflection is idempotent.
 *
 *   Simplest possible boundary condition for a particle in a box —
 *   no restitution coefficient, no friction.  Sufficient because the
 *   per-tick step size (drift * dt) is small relative to the box.
 */
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

/*
 * advance_query_box — advance the query rectangle by ONE tick.
 *
 *   Pseudocode:
 *     integrate_query_box_euler(q, dt)    ← kinematic step (x += v · dt)
 *     bounce_query_box_off_walls(q)       ← elastic reflection at the box
 *
 *   The classic 2-step physics update: integrate, then resolve
 *   constraints.  Same shape as marching_squares.c's blobs_step.
 */
static void advance_query_box(QueryBox *q, float dt)
{
    integrate_query_box_euler(q, dt);
    bounce_query_box_off_walls(q);
}

/*
 * tick_insert_phase — one tick of the tree-construction phase.
 *
 *   Pseudocode:
 *     countdown next_insert_in by dt
 *     if timer expired AND haven't reached DEMO_POINT_COUNT yet:
 *       insert_random_point   (also resets next_insert_in)
 *       if reached the cap: scene_advance_phase (→ PHASE_QUERY)
 *
 *   The countdown gives the user time to watch each insertion (and
 *   any subdivision it triggers) instead of all points appearing at
 *   once.  SECONDS_PER_INSERTION sets the rhythm; insert_random_point
 *   resets the timer to that value after each insert.
 */
static void tick_insert_phase(Scene *s, float dt)
{
    s->next_insert_in -= dt;
    if (s->next_insert_in <= 0.0f && s->points_inserted < DEMO_POINT_COUNT) {
        insert_random_point(s);
        if (s->points_inserted >= DEMO_POINT_COUNT)
            scene_advance_phase(s);
    }
}

/*
 * refresh_query_results — re-run tree_query and store the matches.
 *
 *   Pseudocode:
 *     query_result_count := 0           ← drop last frame's matches
 *     tree_query(pool, root, query_box, &results[], &count, CAP)
 *
 *   Called every tick in PHASE_QUERY so the visible green-glow set
 *   tracks the query rectangle's motion frame-by-frame.  Counts and
 *   results are reset every tick — no incremental update; the
 *   underlying tree_query is fast enough (O(log N + k)) that the
 *   full re-evaluation is cheaper than maintaining incremental state.
 */
static void refresh_query_results(Scene *s)
{
    s->query_result_count = 0;
    tree_query(&s->pool, s->pool.root_idx, s->query.bounds,
               s->query_results, &s->query_result_count, QUERY_RESULT_CAP);
}

/*
 * tick_query_phase — one tick of the range-query phase.
 *
 *   Pseudocode:
 *     advance_query_box       ← integrate + bounce off walls
 *     refresh_query_results   ← re-run tree_query at new position
 *     if PHASE_QUERY has run too long: scene_reset (→ new PHASE_INSERT)
 *
 *   Three named steps: motion → query → auto-cycle.  The auto-cycle
 *   timer (QUERY_PHASE_DURATION) makes the demo loop forever without
 *   user input.
 */
static void tick_query_phase(Scene *s, float dt)
{
    advance_query_box(&s->query, dt);
    refresh_query_results(s);
    if (s->time_in_phase > QUERY_PHASE_DURATION)
        scene_reset(s);
}

/*
 * scene_tick — advance the demo state machine by dt seconds.
 *
 *   Pseudocode (pure phase dispatch):
 *     if paused: return
 *     time_in_phase += dt
 *     switch phase:
 *       PHASE_INSERT → tick_insert_phase(s, dt)
 *       PHASE_QUERY  → tick_query_phase (s, dt)
 *
 *   Each phase's logic lives in its own named function; this driver
 *   just selects which one runs.  Pausing freezes everything (no
 *   tick increment, no phase work).
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    s->time_in_phase += dt;

    if (s->phase == PHASE_INSERT) tick_insert_phase(s, dt);
    else                          tick_query_phase (s, dt);
}

/* ===================================================================== */
/* §7  frame render                                                       */
/* ===================================================================== */

/*
 * draw_top_hud — top HUD_TOP_ROWS rows carry DATA.
 *   Row 0 (right-aligned, just left of the info panel): live status —
 *          fps, sim Hz, phase, paused/running.
 *   Row 1 (left-aligned, full width):                   static config —
 *          capacity, max depth, demo point count.
 *   Both rows use CP_HUD (bright yellow + A_BOLD) per HUD Standard.
 */
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

/*
 * draw_bottom_hud — last row carries ACTION key hints.
 *   Bright cyan + A_BOLD per the project HUD Standard (CLAUDE.md).
 *   Lists every interactive key bound in handle_input.
 */
static void draw_bottom_hud(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  space:pause  n:next/reset  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * render_one_frame — paint + flush one ncurses frame.
 *
 *   Pseudocode (layered draw):
 *     erase                  ← clear stdscr's virtual buffer
 *     scene_draw(sc)         ← tree + (maybe) query box + info panel
 *     draw_top_hud           ← yellow data bar (rows 0, 1)
 *     draw_bottom_hud        ← cyan action bar (last row)
 *     wnoutrefresh           ← stage the diff
 *     doupdate               ← flush ONE diff to the terminal
 *
 *   The wnoutrefresh + doupdate split is the standard ncurses idiom
 *   for "only emit one diff per frame".
 */
static void render_one_frame(const Scene *sc, double fps)
{
    erase();
    scene_draw(sc);
    draw_top_hud(sc, fps);
    draw_bottom_hud(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/* Signal flags — file-scope because POSIX signal handlers can't be
 * passed a pointer.  Everything else lives on main's stack frame. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_exit_signal  (int s) { (void)s; g_quit   = 1; }
static void on_resize_signal(int s) { (void)s; g_resize = 1; }
static void cleanup         (void)  { endwin(); }

/*
 * handle_input — dispatch one keypress against the scene.
 *
 *   Returns true if the program should keep running, false if the
 *   user requested quit.  Quit signals (SIGINT/SIGTERM) reach the
 *   same exit path via g_quit; this just adds the keyboard route.
 */
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

/*
 * init_ncurses_session — bring up ncurses for this demo.
 *   raw keys (cbreak), no echo, hidden cursor, non-blocking getch,
 *   arrow keys (keypad), no typeahead, colours initialised.
 */
static void init_ncurses_session(void)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
}

/*
 * handle_resize — refresh ncurses size cache and update Scene's extent.
 *   Tree state (sc->pool) is unchanged; only screen dimensions move,
 *   and canvas_make recomputes the canvas extent next frame.
 */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * main — own the Scene, drive the fixed-timestep loop.
 *
 *   Pseudocode:
 *     init RNG, signals, ncurses
 *     scene := { sim_hz default, rows/cols from terminal }
 *     scene_reset(&scene)                  ← seed pool + phase
 *     while !g_quit:
 *       if g_resize: handle_resize
 *       advance fixed-step sim (accumulator pattern)
 *       update fps over 500 ms window
 *       sleep to cap render at 60 fps
 *       render_one_frame
 *       handle_input(getch)
 *
 *   The accumulator pattern decouples sim ticks (sc.sim_hz, user-
 *   adjustable) from frames (60 fps cap) — see Fix Your Timestep [9].
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

        /* ── measure elapsed time since last frame ────────────────── */
        int64_t now     = clock_ns();
        int64_t elapsed = now - frame_start;
        frame_start     = now;
        if (elapsed > 100 * NS_PER_MS) elapsed = 100 * NS_PER_MS; /* pause guard */

        /* ── fixed-step accumulator ──────────────────────────────── */
        int64_t tick_duration_ns = TICK_NS(scene.sim_hz);
        float   tick_duration_s  = (float)tick_duration_ns / (float)NS_PER_SEC;

        sim_accum += elapsed;
        while (sim_accum >= tick_duration_ns) {
            scene_tick(&scene, tick_duration_s);
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

        render_one_frame(&scene, fps_display);

        int ch = getch();
        if (ch != ERR && !handle_input(&scene, ch))
            g_quit = 1;
    }

    return 0;
}
