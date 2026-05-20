/*
 * algorithms/kd_tree.c — K-D Tree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1 — the kd-tree library
 *     Data types (Point, KDNode, BBox, GridCanvas, QueryResult),
 *     geometry helpers, memory management, core operations
 *     (kd_insert, kd_query), inspection helpers, and the ASCII
 *     grid visualizer.  No I/O — a reusable module.
 *
 *   PART 2 — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every split live, then
 *     runs a range query and prints the prune trace.  Press Enter
 *     between steps.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/kd_tree.c -o kd_tree
 *
 * Run:
 *   ./kd_tree
 */

/* ── CONCEPTS ───────────────────────────────────────────────────────────── *
 *
 * K-D tree vs Quadtree vs BSP tree (all three are under algorithms/):
 *
 *   Quadtree  — every leaf holds up to LEAF_CAPACITY points; when it
 *               overflows it splits into FOUR equal children (NW/NE/SW/SE).
 *               Both axes are cut simultaneously.  Good for spatial hashing
 *               of points and 2-D range queries.
 *
 *   BSP tree  — every leaf splits into TWO children; axis alternates with
 *               depth (even → vertical, odd → horizontal).  Each node stores
 *               the split line, not a data point.  Classic use: Doom/Quake
 *               back-to-front rendering (pre-computed at level build time).
 *
 *   K-D tree  — each node holds EXACTLY ONE data point AND acts as the
 *               split plane.  Axis alternates with depth.  Every internal
 *               node is simultaneously a stored datum and a spatial divider.
 *               This makes insertion and exact-match search simple: just
 *               compare the relevant coordinate and go left or right.
 *               Trade-off: deletion is more complex (requires re-insertion
 *               of the subtree, or a "tombstone" flag).
 *
 * Time complexity:
 *   Insert (kd_insert):   O(log N) average, O(N) worst case (unbalanced)
 *   Range query (kd_query): O(√N + k) average for 2-D, k = points found
 *   Nearest-neighbour:    O(log N) average (not implemented here)
 *   The √N factor in range queries comes from the fact that in 2-D, a
 *   worst-case strip query touches O(√N) subtrees — this is tighter
 *   than the O(N) brute force but looser than 1-D binary search O(log N).
 *
 * Alternating axis split (kd_insert, kd_query):
 *   At depth 0, 2, 4 … the split is on X (vertical dividing line).
 *   At depth 1, 3, 5 … the split is on Y (horizontal dividing line).
 *   This guarantees that every 2-D region is eventually subdivided along
 *   both axes, preventing degeneracy from axis-aligned point clusters.
 *
 * Bounding-box pruning (kd_query):
 *   Each recursive call narrows the bounding box by halving it along the
 *   current node's split axis.  If the narrowed box does NOT overlap the
 *   query rectangle, the ENTIRE subtree is skipped in O(1).  This is the
 *   source of the √N efficiency — whole branches are pruned without visiting
 *   individual points.
 *
 * References
 * ──────────
 *   ── KD-trees: original + canonical ──────────────────────────────
 *   [1] Bentley, J. L. (1975), "Multidimensional binary search trees
 *       used for associative searching", Communications of the ACM
 *       18(9), pp. 509-517 — the ORIGINAL kd-tree paper.  Defines
 *       the alternating-axis insertion and range query implemented
 *       in this file.  ~9 pages; the best place to start.
 *   [2] Friedman, J. H., Bentley, J. L. & Finkel, R. A. (1977), "An
 *       algorithm for finding best matches in logarithmic expected
 *       time", ACM Trans. Math. Software 3(3), pp. 209-226 — nearest-
 *       neighbour search.  Made kd-trees practical for machine
 *       learning and graphics; the algorithm we did NOT implement.
 *   [3] Bentley, J. L. (1990), "K-d trees for semidynamic point sets",
 *       6th Symposium on Computational Geometry (SCG '90), pp. 187-197
 *       — perfectly balanced kd-tree via median splits.  Addresses
 *       the insertion-order sensitivity that makes our incremental
 *       build potentially degenerate on sorted input.
 *   [4] Samet, H. (2006), "Foundations of Multidimensional and Metric
 *       Data Structures", Morgan Kaufmann — the encyclopaedic
 *       reference for everything spatial.  §1.5, §4.2 cover kd-tree
 *       variants in exhaustive depth (k > 2, bulk loading, paging).
 *
 *   ── Computational geometry textbooks ────────────────────────────
 *   [5] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 5 covers kd-trees + range trees
 *       with full proofs of the O(√N + k) query bound used by
 *       kd_query.
 *   [6] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge — practical C-implementation companion to [5].
 *       Working code for many computational-geometry primitives.
 *   [7] Knuth, D. E. (1998), "The Art of Computer Programming, Vol. 3:
 *       Sorting and Searching" (2nd ed.), Addison-Wesley — §6.5
 *       "Retrieval on Secondary Keys" gives the historical context
 *       for multidimensional searching (pre-Bentley).
 *
 *   ── Range searching refinements ─────────────────────────────────
 *   [8] Lueker, G. S. (1978), "A data structure for orthogonal range
 *       queries", 19th FOCS, pp. 28-34 — fractional cascading for
 *       range trees; how to push the query bound below O(√N).
 *   [9] Bentley, J. L. & Maurer, H. A. (1980), "Efficient worst-case
 *       data structures for range searching", Information Sciences
 *       28, pp. 91-112 — survey of worst-case bounds for range
 *       queries on multiple dimensions.
 *
 *   ── Online quick reference ──────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/K-d_tree — comprehensive
 *       overview with construction, insertion, range and NN query
 *       pseudocode in several languages.  Good first stop after [1].
 *
 *   Note on RENDERING: the ASCII visualizer uses only direct axis-
 *   aligned glyph placement (no Bresenham, no anti-aliasing).  The
 *   only non-trivial trick is the '+' intersection merge in
 *   grid_put_line; grid_print uses plain ANSI escape codes (ECMA-48).
 *   No external rendering references needed.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A K-D TREE is a binary search tree where each node holds ONE
 * data point AND that point's coordinate IS the split.  The
 * split axis ALTERNATES with depth: depth 0 cuts on x (the
 * stored point's x is the split value), depth 1 cuts on y,
 * depth 2 on x again, etc.  Insertion is "go left if smaller,
 * right if greater along the current axis."  Range queries
 * prune subtrees whose region can't possibly contain query
 * points.
 *
 * KD-TREE vs OTHER SPATIAL TREES
 * ──────────────────────────────
 *
 *   QUADTREE: every internal split = 4 children; both axes at
 *             once; data only in leaves.  Wider but shallower.
 *
 *   BSP TREE: 2 children, splits at MIDPOINT of region; data
 *             only in leaves.  Insertion-order independent.
 *
 *   KD TREE:  2 children, splits at the DATA POINT'S coord;
 *             data IN every node.  No "internal-only" nodes →
 *             tighter memory.  Best for nearest-neighbour
 *             search.  Sensitive to insertion order.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  INSERT(point P):
 *    1. If root is null: root = new node with P, depth 0.
 *    2. Else: descend from root:
 *       at each node N at depth d:
 *         axis = (d % 2 == 0) ? X : Y
 *         if P[axis] < N.point[axis]:
 *           if N.left is null: N.left = new node with P; done
 *           else: descend N.left, increment depth
 *         else: same with N.right
 *
 *  RANGE QUERY(rectangle R):
 *    Recursively descend with a "current bounding box" that
 *    narrows along the split axis at each level:
 *
 *      query(node, R, current_box):
 *        if current_box doesn't overlap R: return  ← prune
 *        if node.point in R: report it
 *        axis = depth-dependent
 *        split current_box along axis at node.point[axis]:
 *          left_box  = current_box ∩ {axis < split}
 *          right_box = current_box ∩ {axis ≥ split}
 *        query(node.left,  R, left_box)
 *        query(node.right, R, right_box)
 *
 * KEY FORMULAS
 * ────────────
 *   Split axis at depth d:  d % 2 == 0 → X
 *                           d % 2 == 1 → Y
 *
 *   Insertion comparison:   point[axis] < node.point[axis] → left
 *
 *   Bounding box halving:   left_box.max[axis]  = node.point[axis]
 *                           right_box.min[axis] = node.point[axis]
 *
 *   Bounding box overlap:   along each axis, max1 ≥ min2 AND min1 ≤ max2
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first.  Read
 *      algorithms/bsp_tree.c first if "midpoint splits" are familiar;
 *      this file's "data-point splits" is the natural contrast.
 *      algorithms/quadtree.c covers the quadtree variant.
 *   2. PART 1: KDNode + BBox + helpers + kd_insert + kd_query —
 *      THE LIBRARY.  Each driver carries a pseudocode docblock; read
 *      those before the bodies.
 *   3. PART 2: main() with step-by-step insertion + query.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   KDNode                     one node = one data point + 2 children.
 *   .point                     the (x, y) data this node holds.
 *   .left, .right              children pointers.
 *   depth                      tracked during traversal (not stored)
 *                              — determines split axis.
 *
 * Background you need
 * ───────────────────
 *   - Recursion + binary-tree pointer manipulation.
 *   - Comparison-based search (binary search idea).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Self-balancing trees (red-black, AVL).  K-D balance is
 *     handled by INSERTION ORDER, not by tree rotations.
 *   - K-D nearest-neighbour search.  We only do range queries
 *     here.
 *   - Higher-dimensional K-D trees (k > 2).  The principle
 *     generalises by cycling through axes 0, 1, ..., k-1 at
 *     each depth level.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ─── ANSI color codes ──────────────────────────────────────────────── */
#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_DIM     "\033[2m"
#define CLR_YELLOW  "\033[1;33m"   /* points in the tree              */
#define CLR_GREEN   "\033[1;32m"   /* points found by a query         */
#define CLR_CYAN    "\033[36m"     /* line intersections (+)          */
#define CLR_RED     "\033[1;31m"   /* query rectangle                 */
#define CLR_MAGENTA "\033[35m"     /* x-split lines (vertical |)      */
#define CLR_BLUE    "\033[1;34m"   /* y-split lines (horizontal -)    */
#define CLR_GOLD    "\033[33m"     /* headers and labels              */

/* ================================================================
 * PART 1 — DATA STRUCTURES
 * ================================================================ */

/* ── constants ──────────────────────────────────────────────────── */

/*
 * The demo space is a 56 × 22 integer grid.
 * Points have (x, y) in [0, SPACE_W) × [0, SPACE_H).
 * These dimensions fit in an 80-column terminal once the border is drawn.
 */
#define SPACE_W   56
#define SPACE_H   22

/* Maximum points a single range query can return. */
#define QUERY_CAP 64

/* ── data types ─────────────────────────────────────────────────── */

/*
 * Point — an atomic data point stored inside the kd-tree.
 *
 * Intent
 *   This is the smallest unit the kd-tree organises.  Every KDNode
 *   wraps exactly one Point; the tree's topology IS just a spatial
 *   sort of a Point set.  The tree NEVER inspects the label — it
 *   routes purely on (x, y) — so Point is conceptually a 2-D sort
 *   key + an opaque payload.
 *
 * Why integer coordinates (and not float)
 *   The demo space is a fixed SPACE_W × SPACE_H character grid (§1).
 *   Integers let us address grid cells directly without rounding;
 *   the renderer plots a point at cells[y][x] with no conversion,
 *   and kd_query's < / >= comparisons stay exact (no FP rounding
 *   flipping a near-equal compare).  A production kd-tree on
 *   continuous data would use float/double; the algorithm is
 *   identical, only the comparator type changes.
 *
 * Why a `label` field
 *   Pedagogical only — the step-by-step demo narrates "insert
 *   A(28,11), then B(14,5)" so the letter has to travel with the
 *   point through the tree.  In a real application this slot
 *   carries whatever payload the use case needs (record id,
 *   pointer to a row, metric value).  The kd-tree code never
 *   reads it — only the render and dump layers do.
 *
 * Members
 *   x      grid column,  0 ≤ x < SPACE_W   — split axis when node.axis == 0
 *   y      grid row,     0 ≤ y < SPACE_H   — split axis when node.axis == 1
 *                                            (y grows DOWNWARD — screen convention)
 *   label  opaque payload; the tree treats it as a black box and only
 *          the renderer / dumper inspect it ('A'..'Z' by demo convention)
 *
 * References
 *   The "sort key + opaque payload" pattern is the standard binary-
 *   search-tree node generalised to k dimensions — see Knuth TAOCP
 *   Vol. 3 §6.2 (BST) and Bentley 1975 [1] for the k-D extension.
 */
typedef struct {
    int  x;       /* grid column [0, SPACE_W)  — primary key when axis==0 */
    int  y;       /* grid row    [0, SPACE_H)  — primary key when axis==1 */
    char label;   /* display glyph 'A'..'Z' — opaque to the tree routing  */
} Point;

/*
 * KDNode — one node in the kd-tree.  Both a stored datum AND a splitter.
 *
 * Intent
 *   Each KDNode does THREE jobs that other spatial structures split
 *   across different node types:
 *     (1) Holds one data point   — like a leaf in a quadtree.
 *     (2) Acts as a splitting plane — like an internal node in a BSP.
 *     (3) Routes child traversal via an axis-aligned comparison.
 *   Collapsing all three into one node type is why kd-trees are so
 *   memory-tight (no separate internal/leaf classes) and why
 *   insert + exact-match search are both O(log N) average.
 *
 * Alternating-axis policy
 *   `axis` is fixed at allocation time by kd_new based on the node's
 *   DEPTH in the tree:
 *
 *     depth 0, 2, 4, … → axis = 0  → split on X (vertical line)
 *     depth 1, 3, 5, … → axis = 1  → split on Y (horizontal line)
 *
 *   See axis_at_depth() in §1.  Alternating guarantees both
 *   dimensions get used as filtering criteria; without it the tree
 *   would degenerate to a 1-D BST on one axis only (see Bentley
 *   1975 §3 for the original justification).
 *
 * Why CACHE `axis` per node (instead of recomputing depth % 2)
 *   Two reasons:
 *     (i) Query / render / free recursion don't need a `depth`
 *         parameter — every node carries its own split rule.  Keeps
 *         the public signatures clean (kd_query takes 4 args, not 5).
 *    (ii) Makes the node SELF-DESCRIBING — given any subtree root
 *         you can route a new point without knowing where the
 *         subtree sits in its parent.  Useful for subtree-rebuild,
 *         tree-merge, and other operations not implemented here.
 *   Costs one extra int per node.  Negligible at any reasonable N.
 *
 * Routing rule (mirrored exactly by kd_insert)
 *   coord_on_axis(p, axis) <  coord_on_axis(point, axis)  → go LEFT
 *   coord_on_axis(p, axis) >= coord_on_axis(point, axis)  → go RIGHT
 *   Ties + duplicates fall RIGHT — consistent with bsearch and the
 *   STL multiset convention.
 *
 * Why NO parent pointer
 *   All current operations are top-down recursive (insert, query,
 *   render, free).  A parent pointer would double per-node memory
 *   for zero current benefit.  If you ever implement DELETION you'll
 *   need either a parent pointer or to walk down with a parent
 *   stack — see Bentley 1990 [3] for the standard approach.
 *
 * Members
 *   point   the stored datum AND the split position
 *           (split value = point.x if axis==0 else point.y)
 *   axis    0 → split on X (vertical line through point.x)
 *           1 → split on Y (horizontal line through point.y)
 *           Fixed at kd_new time; never mutated after.
 *   left    subtree: every descendant p has coord[axis] <  point.coord[axis]
 *           (NULL when the slot is empty — a "leaf edge")
 *   right   subtree: every descendant p has coord[axis] >= point.coord[axis]
 *           (NULL when empty; ties + duplicates land here)
 *
 * Invariants
 *   axis ∈ {0, 1}
 *   Every node in left  has coord[axis] strictly less than this node's
 *   Every node in right has coord[axis] greater-or-equal
 *   (these are the per-node BST invariants on the split axis — the
 *   OTHER axis is unconstrained at this level and gets sorted deeper)
 *
 * References
 *   [1] Bentley 1975 §3 defines this exact layout (one point + two
 *       children + alternating axis).  Our KDNode is the direct C
 *       transliteration of his Algol-ish pseudocode.
 *   [3] Bentley 1990 covers semidynamic point sets — deletion,
 *       rebalancing, bulk insert.
 */
typedef struct KDNode KDNode;
struct KDNode {
    Point   point;   /* stored datum AND the split position                          */
    int     axis;    /* 0 = x-split (vertical), 1 = y-split (horizontal); set once   */
    KDNode *left;    /* subtree: descendants have coord[axis] <  point.coord[axis]   */
    KDNode *right;   /* subtree: descendants have coord[axis] >= point.coord[axis]   */
};

/*
 * BBox — inclusive axis-aligned bounding box.  [xmin, xmax] × [ymin, ymax].
 *
 * Intent
 *   A BBox names a RECTANGULAR REGION of grid space.  Three distinct
 *   roles in this file, all served by the same type:
 *     (a) Current subtree's territory — passed down kd_query and
 *         grid_render_tree.  At the root it's the full grid; every
 *         recursion narrows it along the current node's axis.
 *     (b) User's QUERY RECTANGLE — the "give me all stored points
 *         inside R" parameter to kd_query / show_query.
 *     (c) Rendering territory — the pixel region grid_render_tree
 *         is allowed to paint a split line within.
 *   All three are axis-aligned rectangles on the integer grid, so
 *   one struct + one set of geometry helpers (bbox_overlap,
 *   point_in_bbox, bbox_split_left/right) serves every caller.
 *
 * INCLUSIVE convention (both ends)
 *   A point (px, py) is INSIDE iff
 *     xmin ≤ px ≤ xmax  AND  ymin ≤ py ≤ ymax.
 *   Chosen for two reasons:
 *     (i) Match kd_query's "is point in query rect" predicate — you
 *         want to FIND a point that sits at a corner of R.
 *    (ii) Let bbox_split_left/right produce two halves that PARTITION
 *         the parent EXACTLY — no gap, no overlap — by giving the
 *         left half max = value-1 and the right half min = value.
 *         The cell at coord==value belongs to right; the cell at
 *         coord==value-1 belongs to left.
 *
 * Why pass by VALUE (not pointer)
 *   16 bytes (4 × int).  Fits in registers on a 64-bit ABI.  Pass-by-
 *   value avoids aliasing concerns in the recursion where the same
 *   `query_rect` is passed unchanged but the per-call `box` narrows.
 *   The two are clearly separate values in the caller's frame.
 *
 * Members
 *   xmin   left   edge, inclusive — 0 ≤ xmin ≤ xmax in a valid box
 *   ymin   top    edge, inclusive — 0 ≤ ymin ≤ ymax (y grows DOWNWARD)
 *   xmax   right  edge, inclusive
 *   ymax   bottom edge, inclusive
 *
 * Degenerate / empty case
 *   xmin > xmax  OR  ymin > ymax  →  bbox_overlap returns false
 *   against anything; the recursion in kd_query prunes naturally.
 *   bbox_split_left can produce such a box when value == xmin or
 *   value == ymin (the left half is empty) — handled by the prune.
 *
 * References
 *   The bbox-pruning idiom is folklore in computational geometry.
 *   The tightest analysis for kd-trees is in de Berg et al. [5] §5.2
 *   (proves O(√N + k) 2-D range-query cost from this one primitive).
 *   Lueker [8] improves it further with fractional cascading.
 */
typedef struct {
    int xmin;   /* left   edge, inclusive — 0 ≤ xmin ≤ xmax            */
    int ymin;   /* top    edge, inclusive — 0 ≤ ymin ≤ ymax (y is DOWN) */
    int xmax;   /* right  edge, inclusive                              */
    int ymax;   /* bottom edge, inclusive                              */
} BBox;

/*
 * GridCanvas — the ASCII character buffer the visualizer paints into.
 *
 * Intent
 *   Decouples PAINTING (the grid_* helpers below) from PRINTING
 *   (grid_print walks the buffer and emits ANSI-coloured stdout).
 *   This two-phase pattern lets us:
 *     - Paint splits + labels + query frame in any order without
 *       worrying about render-time interleaving.
 *     - Re-paint the same canvas for many frames without re-allocating
 *       (currently we allocate per call — caching is a one-line change).
 *     - Swap the printer in future (e.g. HTML, SVG) without touching
 *       the painters — the canvas IS the abstraction boundary.
 *
 * Why a struct (not a plain 2-D array typedef)
 *   Lets every visualizer helper take `GridCanvas *gc` and read as
 *   "method on a canvas".  No file-scope render state: show_tree and
 *   show_query each own a fresh canvas on their stack frame, so two
 *   threads or two nested renders can never alias each other.
 *
 * Storage layout
 *   `cells[row][col]` — ROW-MAJOR, matching terminal scanout order.
 *   row ∈ [0, SPACE_H);  col ∈ [0, SPACE_W).  The +1 column per row
 *   is a NUL terminator so any row can be treated as a printable C
 *   string (useful for ad-hoc debugging); grid_print itself walks
 *   per-character to apply per-glyph colour.
 *
 * Glyph alphabet (single byte per cell — see glyph_color_for)
 *     ' '       empty space (default fill from grid_clear)
 *     '|'       x-split — vertical line through an x-axis node
 *     '-'       y-split — horizontal line through a y-axis node
 *     '+'       split intersection (set by grid_put_line when | meets -)
 *     'A'..'Z'  data-point label
 *     '*'       data-point highlighted as a query result
 *     '[' ']'   query-rectangle vertical/corner frame
 *     '~'       query-rectangle horizontal frame
 *   New visual states are added by extending this alphabet + adding a
 *   case to glyph_color_for; no other code changes.
 *
 * Why ≈ 1.2 KB on the stack is fine
 *   SPACE_H × (SPACE_W + 1) = 22 × 57 = 1254 bytes.  Well within any
 *   reasonable stack limit (typically 1-8 MiB).  Lifetime is one
 *   show_tree / show_query call — no deep recursion below.
 *
 * Members
 *   cells   [SPACE_H][SPACE_W + 1]  — row-major glyph buffer.
 *                                     cells[r][SPACE_W] is the per-row
 *                                     NUL terminator; never paint there.
 *
 * Invariants (when handed to grid_print)
 *   Every cells[r][c] for c < SPACE_W holds one of the alphabet glyphs.
 *   Every cells[r][SPACE_W] == '\0'.
 *
 * References
 *   The paint-then-blit / paint-then-print split is the same pattern
 *   ncurses uses (write to stdscr, then doupdate flushes one diff).
 *   Here we do the manual equivalent without the ncurses dependency.
 */
typedef struct {
    char cells[SPACE_H][SPACE_W + 1];   /* row-major; cells[r][SPACE_W] is NUL */
} GridCanvas;

/*
 * QueryResult — fixed-capacity output buffer for kd_query.
 *
 * Intent
 *   One struct that replaces the old `(Point *results, int *count,
 *   int capacity)` parameter trio.  Three concrete wins:
 *     (i) kd_query's signature drops from 12 params to 4.
 *    (ii) The "how many matches and where they live" invariant is
 *         encoded in one type, easy to forward to show_query later.
 *   (iii) Fixed-size storage means callers don't malloc — one
 *         QueryResult on the stack covers the whole query lifecycle.
 *
 * Why a FIXED capacity (QUERY_CAP) instead of a growable buffer
 *   The demo answers a single query of bounded size; growing on
 *   demand would just hide a malloc/realloc behind a slower API.
 *   If kd_query fills past QUERY_CAP it silently drops further
 *   matches (count is clamped at QUERY_CAP); callers must size
 *   QUERY_CAP for their expected worst case.  For N=12 demo points
 *   any QUERY_CAP ≥ 12 is safe.
 *
 * Caller protocol
 *     QueryResult result = { .count = 0 };
 *     kd_query(root, bbox_full_grid(), query_rect, &result);
 *     // result.points[0 .. result.count - 1] now hold the matches.
 *     // Order is depth-first left-before-right (insertion-order
 *     // adjacent for sibling nodes); not sorted by distance — if you
 *     // need that, sort the slice after the call.
 *
 * Members
 *   points   [QUERY_CAP]  — match buffer; ONLY entries [0, count)
 *                           are initialised after kd_query returns.
 *   count                 — number of valid entries; 0 on init, grows
 *                           MONOTONICALLY during one kd_query call,
 *                           CAPPED at QUERY_CAP.
 *
 * Invariants (any time)
 *   0 ≤ count ≤ QUERY_CAP
 *   points[i] ∈ query_rect    for 0 ≤ i < count   (post-call)
 *   points[count .. QUERY_CAP-1] are UNINITIALISED — do not read.
 *
 * References
 *   The "output struct with fixed buffer" pattern is from K&R-era C
 *   APIs (e.g. POSIX getcwd, snprintf): the caller owns the storage,
 *   the callee fills it, no ownership ambiguity around returned
 *   pointers.  See Knuth TAOCP Vol. 1 §1.4.1 for the underlying
 *   "preallocated workspace" discipline.
 */
typedef struct {
    Point points[QUERY_CAP];   /* match buffer — entries [0, count) are valid    */
    int   count;               /* match count;  0 ≤ count ≤ QUERY_CAP            */
} QueryResult;

/* ── BBox + axis helpers ────────────────────────────────────────── */

/*
 * axis_at_depth / coord_on_axis — the two primitives that turn
 * "alternating-axis kd-tree" into a single-line policy.
 *
 *   axis_at_depth(d)       even d → 0 (x-split, vertical line)
 *                          odd  d → 1 (y-split, horizontal line)
 *
 *   coord_on_axis(p, axis) returns p.x when axis==0, p.y when axis==1.
 *                          Generalises trivially to k ≥ 2 by indexing
 *                          a coords[k] array instead.
 *
 *   Together they let the same comparison `coord_on_axis(p, axis) <
 *   coord_on_axis(node->point, axis)` work at any depth.
 */
static inline int axis_at_depth(int depth)       { return depth % 2; }
static inline int coord_on_axis(Point p, int ax) { return ax == 0 ? p.x : p.y; }

/* The full grid extent — the bounding box at the root of every recursion. */
static inline BBox bbox_full_grid(void) {
    return (BBox){0, 0, SPACE_W - 1, SPACE_H - 1};
}

/*
 * bbox_overlap — two AABBs overlap iff their projections overlap on
 * BOTH axes.  Equivalently, NON-overlap on either axis is enough to
 * rule out intersection — that's the form the body tests.
 *
 *   This is the PRUNING PREDICATE in kd_query: when the current
 *   subtree's box doesn't overlap the query rect, the whole subtree
 *   is skipped in O(1).  That's where the √N comes from.
 */
static inline bool bbox_overlap(BBox a, BBox b) {
    return !(a.xmin > b.xmax || a.xmax < b.xmin ||
             a.ymin > b.ymax || a.ymax < b.ymin);
}

/* Point (with grid coords) inside an inclusive box. */
static inline bool point_in_bbox(Point p, BBox b) {
    return p.x >= b.xmin && p.x <= b.xmax &&
           p.y >= b.ymin && p.y <= b.ymax;
}

/*
 * bbox_split_left / bbox_split_right — partition a box along an axis
 * at `value`.  Mirrors the kd_insert routing rule:
 *
 *   left  half:  coord[axis] <  value    →  axis_max := value − 1
 *   right half:  coord[axis] >= value    →  axis_min := value
 *
 * The two halves partition the parent exactly: no overlap, no gap.
 * The grid renderer uses the SAME split when drawing the dividing
 * line, so the visual splits and the algorithmic splits stay in sync.
 */
static inline BBox bbox_split_left(BBox b, int axis, int value) {
    if (axis == 0) b.xmax = value - 1;
    else           b.ymax = value - 1;
    return b;
}
static inline BBox bbox_split_right(BBox b, int axis, int value) {
    if (axis == 0) b.xmin = value;
    else           b.ymin = value;
    return b;
}

/* ── memory management ──────────────────────────────────────────── */

/* Allocate and initialise a new leaf node for point p at depth. */
static KDNode *kd_new(Point p, int depth)
{
    KDNode *n = malloc(sizeof *n);
    assert(n != NULL);
    n->point = p;
    n->axis  = depth % 2;   /* even depth → x-split, odd → y-split */
    n->left  = n->right = NULL;
    return n;
}

/* Free a node and all descendants (post-order traversal). */
void kd_free(KDNode *node)
{
    if (!node) return;
    kd_free(node->left);
    kd_free(node->right);
    free(node);
}

/* ── core operations ────────────────────────────────────────────── */

/*
 * kd_insert — add point p into the subtree rooted at 'node'.
 *
 * Returns the (possibly new) root of the subtree so the caller can
 * write:  root = kd_insert(root, p, 0);
 *
 * Pseudocode:
 *   if node is null:  return a fresh leaf with p at this depth
 *   axis  := node->axis                       (set when node was created)
 *   if coord_on_axis(p, axis) <  coord_on_axis(node->point, axis):
 *     node->left  := kd_insert(node->left,  p, depth + 1)
 *   else:
 *     node->right := kd_insert(node->right, p, depth + 1)
 *
 * Because the axis is cached in node->axis at allocation time, query
 * and traversal functions don't need a `depth` parameter — they read
 * axis directly from each node.  Insert keeps `depth` only so a new
 * leaf can pick its own axis via axis_at_depth.
 */
KDNode *kd_insert(KDNode *node, Point p, int depth)
{
    if (!node) return kd_new(p, depth);   /* found the insertion slot */

    int axis  = node->axis;
    int coord = coord_on_axis(p,           axis);
    int split = coord_on_axis(node->point, axis);

    if (coord < split) node->left  = kd_insert(node->left,  p, depth + 1);
    else               node->right = kd_insert(node->right, p, depth + 1);

    return node;
}

/*
 * kd_query — collect all stored points lying inside query_rect into out.
 *
 * Pseudocode (depth-first, prune by bbox_overlap):
 *   if out is full or node is null:  return
 *   if NOT bbox_overlap(box, query_rect):  return        ← prune subtree
 *   if point_in_bbox(node->point, query_rect):  add to out
 *   axis  := node->axis
 *   split := coord_on_axis(node->point, axis)
 *   kd_query(node->left,  bbox_split_left (box, axis, split), query_rect, out)
 *   kd_query(node->right, bbox_split_right(box, axis, split), query_rect, out)
 *
 * `box` is the bounding box of the CURRENT subtree (caller passes
 * bbox_full_grid() at the root).  It narrows along the current axis
 * at every level, exactly mirroring how kd_insert routes new points.
 *
 * PRUNING (the source of the O(√N) average cost in 2-D):
 *   When `box` doesn't overlap query_rect, every point in this
 *   subtree is geometrically outside the query, so the whole subtree
 *   is skipped in O(1).  Without this check the cost would be O(N).
 *
 * Parameters:
 *   node        — current subtree root
 *   box         — current subtree's bounding box (narrows on recursion)
 *   query_rect  — caller's query rectangle (constant through recursion)
 *   out         — fixed-capacity result buffer (caller zeroes out->count)
 */
void kd_query(KDNode *node, BBox box, BBox query_rect, QueryResult *out)
{
    if (!node || out->count >= QUERY_CAP)     return;
    if (!bbox_overlap(box, query_rect))       return;   /* prune subtree */

    if (point_in_bbox(node->point, query_rect))
        out->points[out->count++] = node->point;

    int axis  = node->axis;
    int split = coord_on_axis(node->point, axis);
    kd_query(node->left,  bbox_split_left (box, axis, split), query_rect, out);
    kd_query(node->right, bbox_split_right(box, axis, split), query_rect, out);
}

/* ── inspection helpers ─────────────────────────────────────────── */

/* Total number of nodes in the tree. */
int kd_node_count(KDNode *node)
{
    if (!node) return 0;
    return 1 + kd_node_count(node->left) + kd_node_count(node->right);
}

/*
 * kd_depth — return the depth of the deepest node (root = depth 0).
 * 'current' is the depth of 'node' in the full tree.
 * A NULL child returns current-1 (the parent's depth).
 */
int kd_depth(KDNode *node, int current)
{
    if (!node) return current - 1;
    int dl = kd_depth(node->left,  current + 1);
    int dr = kd_depth(node->right, current + 1);
    return dl > dr ? dl : dr;
}

/*
 * kd_dump — print an indented text representation.
 *
 * Example output after all 12 points are inserted:
 *   root   axis=X  A(28,11)
 *     LEFT   axis=Y  B(14,5)
 *       LEFT   axis=X  D(7,2)
 *         LEFT   (empty)
 *         RIGHT  axis=Y  E(21,2)
 *       RIGHT  axis=X  F(7,16)
 *         LEFT   axis=Y  L(4,8)
 *         RIGHT  axis=Y  G(21,16)
 *     RIGHT  axis=Y  C(42,5)
 *       LEFT   axis=X  H(35,2)
 *         LEFT   (empty)
 *         RIGHT  axis=Y  I(49,2)
 *       RIGHT  axis=X  J(35,16)
 *         LEFT   (empty)
 *         RIGHT  axis=Y  K(49,16)
 */
void kd_dump(KDNode *node, int depth, const char *label)
{
    for (int i = 0; i < depth * 2; i++) putchar(' ');
    printf(CLR_DIM "%-7s" CLR_RESET, label);

    if (!node) {
        printf("(empty)\n");
        return;
    }

    const char *axis_name = (node->axis == 0) ? "X" : "Y";
    printf("axis=%s  ", axis_name);
    printf(CLR_YELLOW "%c" CLR_RESET "(%d,%d)\n",
           node->point.label, node->point.x, node->point.y);

    kd_dump(node->left,  depth + 1, "LEFT");
    kd_dump(node->right, depth + 1, "RIGHT");
}

/* ── ASCII grid visualizer ──────────────────────────────────────── */

/*
 * The visualizer paints every node's splitting line into a GridCanvas
 * (SPACE_W × SPACE_H character buffer), then walks the canvas with
 * grid_print to colour each glyph.  No global state — the canvas is
 * a local in show_tree / show_query, passed to every helper.
 *
 * Glyph alphabet (matches the legend printed in main):
 *   ' '       empty space
 *   |         x-split  (vertical line through an x-axis node)   magenta
 *   -         y-split  (horizontal line through a y-axis node)  blue
 *   +         intersection where a | and a - line cross         cyan
 *   A..Z      data point label                                  yellow
 *   *         data point found by a range query                 green
 *   [ ] ~     query rectangle frame                             red
 *
 * Painter's-order: grid_render_tree first draws the split line, then
 * recurses into the two halved sub-boxes, THEN paints the data point
 * over whatever line passed through that cell — so a node label is
 * never hidden by its own split.
 */

static void grid_clear(GridCanvas *gc)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(gc->cells[r], ' ', SPACE_W);
        gc->cells[r][SPACE_W] = '\0';
    }
}

/*
 * grid_put_line — paint a line character ('|' or '-') with intersection
 * merging.  When a vertical meets a horizontal in the same cell, emit '+'
 * so crossings are visible; otherwise paint the requested glyph (and
 * never overwrite an already-correct '+').
 */
static void grid_put_line(GridCanvas *gc, int x, int y, char ch)
{
    if (x < 0 || x >= SPACE_W || y < 0 || y >= SPACE_H) return;
    char cur = gc->cells[y][x];
    if ((ch == '|' && cur == '-') || (ch == '-' && cur == '|'))
        gc->cells[y][x] = '+';
    else if (cur != '+')   /* don't overwrite an already-correct '+' */
        gc->cells[y][x] = ch;
}

/* Place a glyph at (x, y), always overriding whatever line was there. */
static void grid_put_point(GridCanvas *gc, int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        gc->cells[y][x] = ch;
}

/*
 * grid_draw_query_rect — frame the query rectangle with [ ] ~ glyphs.
 *
 *   Top / bottom edges use '~', left / right use '[' / ']'.  Corners
 *   are forced to '[' or ']' so the frame reads as a box, not a wave.
 *   Distinct from the split-line alphabet so frames never look like splits.
 */
static void grid_draw_query_rect(GridCanvas *gc, BBox r)
{
    for (int x = r.xmin; x <= r.xmax; x++) {
        grid_put_point(gc, x, r.ymin, '~');
        grid_put_point(gc, x, r.ymax, '~');
    }
    for (int y = r.ymin; y <= r.ymax; y++) {
        grid_put_point(gc, r.xmin, y, '[');
        grid_put_point(gc, r.xmax, y, ']');
    }
    grid_put_point(gc, r.xmin, r.ymin, '[');
    grid_put_point(gc, r.xmax, r.ymin, ']');
    grid_put_point(gc, r.xmin, r.ymax, '[');
    grid_put_point(gc, r.xmax, r.ymax, ']');
}

/*
 * point_is_highlighted — is this node's point in the highlight set?
 *   Used by grid_render_tree to swap label → '*' for query matches.
 */
static bool point_is_highlighted(Point p, const QueryResult *highlight)
{
    if (!highlight) return false;
    for (int i = 0; i < highlight->count; i++)
        if (highlight->points[i].x == p.x && highlight->points[i].y == p.y)
            return true;
    return false;
}

/*
 * grid_render_tree — recursive painter for splits + labels.
 *
 *   Pseudocode:
 *     if node is null:  return
 *     axis  := node->axis
 *     split := coord_on_axis(node->point, axis)
 *     paint the split line spanning the current box on `axis`
 *     recurse into node->left  with bbox_split_left (box, axis, split)
 *     recurse into node->right with bbox_split_right(box, axis, split)
 *     paint the label glyph (or '*' if highlighted) at node->point
 *
 *   The recursion order matches kd_query exactly: same bbox split,
 *   same axis policy.  That's why the visible drawing matches the
 *   tree's algorithmic partitioning cell-for-cell.
 *
 *   `highlight` is the QueryResult to mark with '*'; pass NULL to
 *   render plain labels (no query in progress).
 */
static void grid_render_tree(GridCanvas *gc, KDNode *node, BBox box,
                             const QueryResult *highlight)
{
    if (!node) return;

    int axis  = node->axis;
    int split = coord_on_axis(node->point, axis);

    if (axis == 0) {
        for (int y = box.ymin; y <= box.ymax; y++)
            grid_put_line(gc, split, y, '|');
    } else {
        for (int x = box.xmin; x <= box.xmax; x++)
            grid_put_line(gc, x, split, '-');
    }

    grid_render_tree(gc, node->left,
                     bbox_split_left (box, axis, split), highlight);
    grid_render_tree(gc, node->right,
                     bbox_split_right(box, axis, split), highlight);

    /* Paint the data point AFTER recursion so it sits on top of any
     * line that passes through its cell.                             */
    char glyph = point_is_highlighted(node->point, highlight)
               ? '*' : node->point.label;
    grid_put_point(gc, node->point.x, node->point.y, glyph);
}

/*
 * glyph_color_for — pick the ANSI colour code for a single canvas glyph.
 *   Decouples the colour table from grid_print's loop.  Extending the
 *   alphabet (e.g. a new highlight class) only requires a new case here.
 */
static const char *glyph_color_for(char ch)
{
    if (ch >= 'A' && ch <= 'Z')               return CLR_YELLOW;
    if (ch == '*')                            return CLR_GREEN;
    if (ch == '[' || ch == ']' || ch == '~')  return CLR_RED;
    if (ch == '|')                            return CLR_MAGENTA;
    if (ch == '-')                            return CLR_BLUE;
    if (ch == '+')                            return CLR_CYAN;
    return NULL;   /* plain putchar */
}

/* Print the canvas with a surrounding box and per-character ANSI colour. */
static void grid_print(const GridCanvas *gc)
{
    printf(CLR_DIM "+");
    for (int c = 0; c < SPACE_W; c++) putchar('-');
    printf("+" CLR_RESET "\n");

    for (int r = 0; r < SPACE_H; r++) {
        printf(CLR_DIM "|" CLR_RESET);
        for (int c = 0; c < SPACE_W; c++) {
            char ch = gc->cells[r][c];
            const char *col = glyph_color_for(ch);
            if (col) printf("%s%c" CLR_RESET, col, ch);
            else     putchar(ch);
        }
        printf(CLR_DIM "|" CLR_RESET "\n");
    }

    printf(CLR_DIM "+");
    for (int c = 0; c < SPACE_W; c++) putchar('-');
    printf("+" CLR_RESET "\n");
}

/* ================================================================
 * PART 2 — STEP-BY-STEP DEMO
 * ================================================================ */

/* ── demo helpers ───────────────────────────────────────────────── */

static void press_enter(void)
{
    printf(CLR_DIM "  [press Enter]" CLR_RESET);
    fflush(stdout);
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF)
        ;
}

static void print_separator(void)
{
    printf(CLR_DIM
           "------------------------------------------------------------\n"
           CLR_RESET);
}

/*
 * show_tree — render the current kd-tree state and print a summary line.
 *   Allocates a fresh GridCanvas on the stack (≈ 1.2 KB), paints the
 *   tree into it from the full-grid bbox, prints, then drops it.
 */
static void show_tree(const char *title, KDNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);

    GridCanvas gc;
    grid_clear(&gc);
    if (root) grid_render_tree(&gc, root, bbox_full_grid(), NULL);
    grid_print(&gc);

    if (!root)
        printf("  nodes=0  depth=-\n");
    else
        printf("  nodes=%d  depth=%d\n",
               kd_node_count(root), kd_depth(root, 0));
}

/*
 * show_query — render tree + query rectangle + highlighted found points.
 *   Same canvas pattern as show_tree, plus a query-frame overlay and a
 *   summary line listing each found point.  Pass `result` to mark its
 *   points with '*' instead of their label glyph.
 */
static void show_query(const char *title, KDNode *root,
                       BBox query_rect, const QueryResult *result)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);

    GridCanvas gc;
    grid_clear(&gc);
    grid_render_tree(&gc, root, bbox_full_grid(), result);
    grid_draw_query_rect(&gc, query_rect);
    grid_print(&gc);

    printf("  Query rect: [%d,%d] to [%d,%d]   Found %d point%s:",
           query_rect.xmin, query_rect.ymin,
           query_rect.xmax, query_rect.ymax,
           result->count, result->count == 1 ? "" : "s");
    for (int i = 0; i < result->count; i++)
        printf("  " CLR_GREEN "%c(%d,%d)" CLR_RESET,
               result->points[i].label,
               result->points[i].x, result->points[i].y);
    putchar('\n');
}

/* ── main: the walkthrough ──────────────────────────────────────── */

int main(void)
{
    /* ── introduction ─────────────────────────────────────────── */

    printf("\n");
    print_separator();
    printf(CLR_BOLD "  K-D TREE — data structure walkthrough\n" CLR_RESET);
    print_separator();
    printf(
        "\n"
        "  A k-d tree (k-dimensional tree) partitions a 2-D space\n"
        "  using alternating axis-aligned splitting planes.\n"
        "\n"
        "  Unlike a quadtree (4 quadrants, internal nodes hold no data),\n"
        "  a k-d tree is a BINARY tree where EVERY node:\n"
        "    1. Stores exactly one data point.\n"
        "    2. Acts as a splitting plane that divides its region in two.\n"
        "\n"
        "  The split axis alternates with tree depth:\n"
        "    depth 0, 2, 4, \xe2\x80\xa6 \xe2\x86\x92 axis X  (vertical line through the point)\n"
        "    depth 1, 3, 5, \xe2\x80\xa6 \xe2\x86\x92 axis Y  (horizontal line through the point)\n"
        "\n"
        "  Routing rule:\n"
        "    coord[axis] < split  \xe2\x86\x92  go LEFT\n"
        "    coord[axis] >= split \xe2\x86\x92  go RIGHT\n"
        "\n"
        "  This demo inserts 12 labelled points into a %d\xc3\x97%d grid,\n"
        "  pausing after each group so you can watch the space subdivide.\n"
        "  Then it runs a range query and shows the pruning.\n"
        "\n"
        "  Legend:  " CLR_MAGENTA "|" CLR_RESET " x-split   "
                      CLR_BLUE    "-" CLR_RESET " y-split   "
                      CLR_CYAN    "+" CLR_RESET " intersection   "
                      CLR_YELLOW  "A" CLR_RESET " point\n"
        "           " CLR_RED    "[ ~ ]" CLR_RESET " query rect   "
                      CLR_GREEN  "*" CLR_RESET " found\n",
        SPACE_W, SPACE_H
    );
    press_enter();

    /* ── Step 1: empty tree ───────────────────────────────────── */

    KDNode *root = NULL;

    show_tree("Step 1 \xe2\x80\x94 empty tree  (no nodes, no splits yet)", root);
    printf(
        "  The %d\xc3\x97%d grid is empty.\n"
        "  The first point inserted will become the root at depth 0.\n",
        SPACE_W, SPACE_H
    );
    press_enter();

    /* ── Step 2: root node ────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 2 \xe2\x80\x94 insert A(28,11) \xe2\x80\x94 becomes the root\n" CLR_RESET);
    printf(
        "\n"
        "  A is inserted at depth 0  \xe2\x86\x92  axis = X.\n"
        "  Its splitting plane is a vertical line at x=28, spanning the\n"
        "  full grid height.  Space is now divided:\n"
        "    LEFT  half:  x in [0, 27]\n"
        "    RIGHT half:  x in [28, 55]\n"
        "\n"
        "  A itself sits on the split line.  Points with x < 28 will\n"
        "  be routed LEFT; points with x >= 28 will go RIGHT.\n"
    );
    press_enter();

    root = kd_insert(root, (Point){28, 11, 'A'}, 0);
    show_tree("  After inserting A \xe2\x80\x94 root created, space split vertically", root);
    printf("  Root A(28,11): axis=X, vertical split at x=28.\n");
    press_enter();

    /* ── Step 3: one child each side of the root ─────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 3 \xe2\x80\x94 insert B(14,5) and C(42,5)\n" CLR_RESET);
    printf(
        "\n"
        "  B(14,5):  x=14 < 28  \xe2\x86\x92  goes A.LEFT\n"
        "            depth 1  \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=5\n"
        "            splits A's left half [0,27]\xc3\x97[0,21] into:\n"
        "              TOP    strip: [0,27]\xc3\x97[0, 4]\n"
        "              BOTTOM strip: [0,27]\xc3\x97[5,21]\n"
        "\n"
        "  C(42,5):  x=42 >= 28  \xe2\x86\x92  goes A.RIGHT\n"
        "            depth 1  \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=5\n"
        "            splits A's right half [28,55]\xc3\x97[0,21] into:\n"
        "              TOP    strip: [28,55]\xc3\x97[0, 4]\n"
        "              BOTTOM strip: [28,55]\xc3\x97[5,21]\n"
        "\n"
        "  Together B and C draw a full-width horizontal at y=5.\n"
        "  Where A's vertical (x=28) crosses it, you will see a '+'.\n"
    );
    press_enter();

    root = kd_insert(root, (Point){14,  5, 'B'}, 0);
    root = kd_insert(root, (Point){42,  5, 'C'}, 0);
    show_tree("  After inserting B C \xe2\x80\x94 four rectangular regions formed", root);
    printf(
        "  B(14,5) y-split and C(42,5) y-split together divide the grid\n"
        "  into four quadrant-like regions (but done in two binary cuts,\n"
        "  not one quad split).\n"
    );
    press_enter();

    /* ── Step 4: top-left region ──────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 4 \xe2\x80\x94 insert D(7,2) and E(21,2)  [top-left region]\n"
           CLR_RESET);
    printf(
        "\n"
        "  D(7,2):  x=7  < 28  \xe2\x86\x92  A.left  (B)\n"
        "           y=2  <  5  \xe2\x86\x92  B.left\n"
        "           depth 2    \xe2\x86\x92  axis = X  \xe2\x86\x92  vertical split at x=7\n"
        "           region [0,27]\xc3\x97[0,4] is cut at x=7.\n"
        "\n"
        "  E(21,2): x=21 < 28  \xe2\x86\x92  A.left  (B)\n"
        "           y=2  <  5  \xe2\x86\x92  B.left  (D)\n"
        "           x=21 >= 7  \xe2\x86\x92  D.right\n"
        "           depth 3    \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=2\n"
        "           region [7,27]\xc3\x97[0,4] is cut at y=2.\n"
    );
    press_enter();

    root = kd_insert(root, (Point){ 7,  2, 'D'}, 0);
    root = kd_insert(root, (Point){21,  2, 'E'}, 0);
    show_tree("  After inserting D E \xe2\x80\x94 top-left region subdivided", root);
    printf(
        "  D's vertical (x=7) meets B's horizontal (y=5) edge with a '+'.\n"
        "  E's horizontal (y=2) subdivides D's right sub-region.\n"
    );
    press_enter();

    /* ── Step 5: bottom-left region ──────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 5 \xe2\x80\x94 insert F(7,16) G(21,16) L(4,8)  [bottom-left]\n"
           CLR_RESET);
    printf(
        "\n"
        "  F(7,16):  x=7  < 28, y=16 >= 5  \xe2\x86\x92  A.left \xe2\x86\x92 B.right\n"
        "            depth 2  \xe2\x86\x92  axis = X  \xe2\x86\x92  vertical split at x=7\n"
        "            region [0,27]\xc3\x97[5,21] cut at x=7.\n"
        "            F's vertical continues D's line below B's horizontal.\n"
        "\n"
        "  G(21,16): x=21 < 28, y=16 >= 5  \xe2\x86\x92  A.left \xe2\x86\x92 B.right \xe2\x86\x92 F.right\n"
        "            depth 3  \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=16\n"
        "            region [7,27]\xc3\x97[5,21] cut at y=16.\n"
        "\n"
        "  L(4,8):   x=4  < 28, y=8  >= 5  \xe2\x86\x92  A.left \xe2\x86\x92 B.right \xe2\x86\x92 F.left\n"
        "            depth 3  \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=8\n"
        "            region [0,6]\xc3\x97[5,21] cut at y=8.\n"
    );
    press_enter();

    root = kd_insert(root, (Point){ 7, 16, 'F'}, 0);
    root = kd_insert(root, (Point){21, 16, 'G'}, 0);
    root = kd_insert(root, (Point){ 4,  8, 'L'}, 0);
    show_tree("  After inserting F G L \xe2\x80\x94 bottom-left region subdivided", root);
    printf(
        "  The entire left half now has a fine grid of splits.\n"
        "  D's and F's verticals share x=7, forming one continuous line\n"
        "  across the full left half (broken only by B's horizontal at y=5).\n"
    );
    press_enter();

    /* ── Step 6: right half ───────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD
           "  Step 6 \xe2\x80\x94 insert H(35,2) I(49,2) J(35,16) K(49,16)  [right half]\n"
           CLR_RESET);
    printf(
        "\n"
        "  H(35,2):  x=35 >= 28  \xe2\x86\x92  A.right (C)\n"
        "            y=2  <   5  \xe2\x86\x92  C.left\n"
        "            depth 2     \xe2\x86\x92  axis = X  \xe2\x86\x92  vertical split at x=35\n"
        "            region [28,55]\xc3\x97[0,4] cut at x=35.\n"
        "\n"
        "  I(49,2):  x=49 >= 28  \xe2\x86\x92  A.right \xe2\x86\x92 C.left \xe2\x86\x92 H.right\n"
        "            depth 3     \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=2\n"
        "            Mirrors E's placement on the right side.\n"
        "\n"
        "  J(35,16): x=35 >= 28, y=16 >= 5  \xe2\x86\x92  A.right \xe2\x86\x92 C.right\n"
        "            depth 2     \xe2\x86\x92  axis = X  \xe2\x86\x92  vertical split at x=35\n"
        "            H's and J's verticals together span the full right half.\n"
        "\n"
        "  K(49,16): mirrors G in the bottom-right  \xe2\x86\x92  J.right\n"
        "            depth 3     \xe2\x86\x92  axis = Y  \xe2\x86\x92  horizontal split at y=16\n"
    );
    press_enter();

    root = kd_insert(root, (Point){35,  2, 'H'}, 0);
    root = kd_insert(root, (Point){49,  2, 'I'}, 0);
    root = kd_insert(root, (Point){35, 16, 'J'}, 0);
    root = kd_insert(root, (Point){49, 16, 'K'}, 0);
    show_tree("  After inserting H I J K \xe2\x80\x94 final tree state (12 points)", root);
    printf(
        "  The grid is now partitioned into 13 distinct rectangular cells\n"
        "  using only 12 binary splits (one per node).  Each row of the\n"
        "  tree adds one cut that refines exactly one existing cell.\n"
    );
    press_enter();

    /* ── Step 7: tree dump ────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 7 \xe2\x80\x94 tree structure (indented text dump)\n" CLR_RESET);
    printf("\n");
    kd_dump(root, 0, "root");
    printf(
        "\n"
        "  Total nodes : %d  (every node holds one point AND one split)\n"
        "  Tree depth  : %d  (root = 0)\n"
        "\n"
        "  Reading the tree:\n"
        "    axis=X means this node's vertical line routes its subtree.\n"
        "    axis=Y means this node's horizontal line routes its subtree.\n"
        "    LEFT child has coord[axis] < this node's coord[axis].\n"
        "    RIGHT child has coord[axis] >= this node's coord[axis].\n",
        kd_node_count(root),
        kd_depth(root, 0)
    );
    press_enter();

    /* ── Step 8: range query ──────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD
           "  Step 8 \xe2\x80\x94 range query: find all points in [0,0]..[20,10]\n"
           CLR_RESET);
    printf(
        "\n"
        "  kd_query() tracks each subtree's bounding box and applies:\n"
        "\n"
        "    PRUNE   \xe2\x80\x94 if the bounding box does NOT overlap the query\n"
        "             rectangle, skip the node and all its descendants.\n"
        "    COLLECT \xe2\x80\x94 if it overlaps, check this node's point;\n"
        "             add to results if it lies inside the query rect.\n"
        "\n"
        "  Visit trace  (depth-first, left before right):\n"
        "    A  bbox=[0,55]\xc3\x97[0,21] overlaps \xe2\x86\x92 A(28,11): x=28>20  NO\n"
        "    A.left  \xe2\x86\x92 B  bbox=[0,27]\xc3\x97[0,21] overlaps \xe2\x86\x92 B(14,5):  YES\n"
        "      B.left  \xe2\x86\x92 D  bbox=[0,27]\xc3\x97[0,4]  overlaps \xe2\x86\x92 D(7,2):   YES\n"
        "        D.left  (NULL) \xe2\x86\x92 skip\n"
        "        D.right \xe2\x86\x92 E  bbox=[7,27]\xc3\x97[0,4]  overlaps \xe2\x86\x92 E(21,2): x=21>20  NO\n"
        "      B.right \xe2\x86\x92 F  bbox=[0,27]\xc3\x97[5,21] overlaps \xe2\x86\x92 F(7,16): y=16>10  NO\n"
        "        F.left  \xe2\x86\x92 L  bbox=[0,6]\xc3\x97[5,21]  overlaps \xe2\x86\x92 L(4,8):  YES\n"
        "        F.right \xe2\x86\x92 G  bbox=[7,27]\xc3\x97[5,21] overlaps \xe2\x86\x92 G(21,16): x>20,y>10  NO\n"
        "    A.right \xe2\x86\x92 C  bbox=[28,55]\xc3\x97[0,21]: xmin=28 > query_rect.xmax=20  \xe2\x80\x94 PRUNED\n"
        "\n"
        "  The entire right half (C H I J K) is pruned in a single check.\n"
    );
    press_enter();

    BBox        query_rect = { 0, 0, 20, 10 };
    QueryResult result     = { .count = 0 };

    kd_query(root, bbox_full_grid(), query_rect, &result);

    show_query("  Query result", root, query_rect, &result);
    press_enter();

    /* ── Done ─────────────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Done.\n" CLR_RESET
           "\n"
           "  Summary of operations implemented in this file:\n"
           "\n"
           "    kd_insert(root, point, depth)            O(log N) average\n"
           "    kd_query(root, box, query_rect, &out)    O(\xe2\x88\x9aN + k) average\n"
           "    kd_node_count / kd_depth                 O(N)\n"
           "    kd_dump                                  O(N)\n"
           "    kd_free(root)                            O(N)\n"
           "\n"
           "  Key differences from quadtree:\n"
           "    \xe2\x80\xa2 Binary tree (2 children) vs. quad tree (4 children)\n"
           "    \xe2\x80\xa2 Every node stores one point AND acts as a splitting plane\n"
           "    \xe2\x80\xa2 Splits alternate X/Y with depth instead of cutting both at once\n"
           "    \xe2\x80\xa2 Simpler insertion: no leaf-capacity concept, no subdivide() step\n"
           "    \xe2\x80\xa2 O(\xe2\x88\x9aN) range queries vs O(N) linear scan (for uniform data)\n"
           "\n");

    kd_free(root);
    return 0;
}
