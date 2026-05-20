/*
 * algorithms/quadtree.c — Quadtree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1 — the quadtree library
 *     Data types (Point, Rect, QuadNode, QuadrantRects, QueryRect,
 *     QueryResult, GridCanvas), geometry helpers, memory management,
 *     core operations (qt_insert, qt_query, subdivide), inspection
 *     helpers, and the ASCII grid visualizer.  No I/O — a reusable
 *     module.
 *
 *   PART 2 — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every subdivision live, then
 *     runs a range query and prints the prune trace.  Press Enter
 *     between steps.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/quadtree.c -o quadtree
 *
 * Run:
 *   ./quadtree
 */

/* ── CONCEPTS ───────────────────────────────────────────────────────────── *
 *
 * What problem does a quadtree solve?
 *   Given N points in 2-D space, answer "which points are inside rectangle R?"
 *   efficiently.  Brute force is O(N) — check every point.  A quadtree gives
 *   O(log N + k) average, where k = number of points found.  The key insight:
 *   skip whole subtrees whose region does NOT overlap R.
 *
 * Quadtree vs BSP tree vs K-D tree (all three are in geometry/):
 *
 *   Quadtree  — (this file) leaf holds up to LEAF_CAPACITY points; when full,
 *               it subdivides into FOUR children (NW/NE/SW/SE) simultaneously.
 *               Internal nodes hold NO data — all data lives in leaves.
 *               Simple to implement; good when points cluster in few regions.
 *
 *   BSP tree  — splits into TWO children per node; axis alternates (V/H).
 *               Each node owns its boundary rect and the split line position.
 *               Split at midpoint (not at the inserted point).
 *               Classical Doom/Quake rendering algorithm.
 *
 *   K-D tree  — each node holds EXACTLY ONE data point AND is the split.
 *               No "region" concept; split position = the stored point's coord.
 *               Best average-case for nearest-neighbour search.
 *
 * Subdivision trigger (qt_insert):
 *   A leaf subdivides when inserting would make count exceed LEAF_CAPACITY.
 *   After subdividing, the leaf's existing points are re-inserted into the
 *   appropriate child (their coordinates determine NW/NE/SW/SE).
 *   LEAF_CAPACITY=4 means subdivisions happen visibly during the 12-point demo.
 *
 * Half-open intervals (Rect):
 *   The region is x ∈ [x, x+w) and y ∈ [y, y+h).  The upper bound is
 *   EXCLUSIVE.  This makes subdivision exact: the four children share no
 *   pixels and together cover the parent exactly:
 *     NW: [x,   x+w/2) × [y,   y+h/2)
 *     NE: [x+w/2, x+w) × [y,   y+h/2)
 *     SW: [x,   x+w/2) × [y+h/2, y+h)
 *     SE: [x+w/2, x+w) × [y+h/2, y+h)
 *   With inclusive bounds, the four midpoint cells would each belong to TWO
 *   children simultaneously, creating ambiguity.
 *
 * AABB pruning (qt_query):
 *   Before examining any node, check if the query rect overlaps the node's
 *   boundary.  Non-overlap → skip the ENTIRE subtree in O(1).
 *   This is the source of the O(log N + k) bound.
 *
 * Complexity:
 *   Insert:      O(log N) average when points are spatially distributed.
 *   Range query: O(log N + k) average, k = points returned.
 *   Worst case (all points in same cell): O(N) for both.
 *
 * References
 * ──────────
 *   ── Quadtrees: original + canonical ─────────────────────────────
 *   [1] Finkel, R. A. & Bentley, J. L. (1974), "Quad trees: a data
 *       structure for retrieval on composite keys", Acta Informatica
 *       4(1), pp. 1-9 — the ORIGINAL quadtree paper.  Defines the
 *       4-children-per-split structure implemented here.  Short and
 *       readable; the best place to start.
 *   [2] Samet, H. (1984), "The quadtree and related hierarchical
 *       data structures", ACM Computing Surveys 16(2), pp. 187-260 —
 *       the canonical SURVEY paper.  Covers point quadtrees, region
 *       quadtrees, PR quadtrees, MX quadtrees, and the family of
 *       variants we did NOT implement.
 *   [3] Samet, H. (2006), "Foundations of Multidimensional and Metric
 *       Data Structures", Morgan Kaufmann — the encyclopaedic
 *       reference for everything spatial.  Ch. 1.4, 2.1 cover
 *       quadtrees in exhaustive depth, including bulk loading,
 *       neighbour finding, and secondary-storage variants.
 *   [4] Samet, H. (1990), "The Design and Analysis of Spatial Data
 *       Structures", Addison-Wesley — earlier and shorter than [3];
 *       still the standard textbook citation for many CG papers.
 *
 *   ── Computational geometry textbooks ────────────────────────────
 *   [5] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 14 ("Quadtrees: Non-Uniform Mesh
 *       Generation") gives proofs of subdivision cost and range-
 *       query bounds.
 *   [6] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge — practical C-implementation companion to [5].
 *       §1.5 on the basic geometric primitives used here.
 *
 *   ── Famous applications ────────────────────────────────────────
 *   [7] Barnes, J. & Hut, P. (1986), "A hierarchical O(N log N)
 *       force-calculation algorithm", Nature 324, pp. 446-449 —
 *       the OCTREE (3-D quadtree) for N-body simulation.  Single
 *       most-cited application of the data structure outside CG.
 *   [8] Greengard, L. & Rokhlin, V. (1987), "A fast algorithm for
 *       particle simulations", J. Computational Physics 73, pp. 325-
 *       348 — Fast Multipole Method.  Builds on the Barnes-Hut idea
 *       with a better error bound.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *   [9] https://en.wikipedia.org/wiki/Quadtree — comprehensive
 *       overview including the four common variants (point, region,
 *       edge, polygonal map) and pseudocode for each.
 *
 *   Note on RENDERING: the ASCII visualizer uses only direct axis-
 *   aligned glyph placement (no Bresenham, no anti-aliasing).  The
 *   only non-trivial trick is the '+' boundary-corner merge when
 *   subdivision edges meet; the colour formatting uses plain ANSI
 *   escape codes (ECMA-48).  No external rendering references needed.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A QUADTREE recursively divides a 2-D region into FOUR EQUAL
 * QUADRANTS (NW, NE, SW, SE) whenever a region holds more than
 * LEAF_CAPACITY points.  Internal nodes hold NO data; every
 * point lives in some leaf.  The tree adapts to the data:
 * dense regions get subdivided many times, sparse regions
 * stay as single leaves.
 *
 * QUADTREE vs OTHER SPATIAL TREES
 * ───────────────────────────────
 *   QUADTREE:  4 children per split, both axes at once;
 *              SHALLOWER tree but more memory per internal
 *              node.  Best for clustered data.
 *
 *   BSP TREE:  2 children, alternating axis, mid-point split.
 *              See algorithms/bsp_tree.c.
 *
 *   KD-TREE:   2 children, alternating axis, data-point split,
 *              data IN nodes.  See algorithms/kd_tree.c.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  INSERT(point P):
 *    1. Start at root.  If P doesn't fall in root's rect,
 *       reject.
 *    2. While current node is INTERNAL:
 *         determine which quadrant P falls in (NW/NE/SW/SE)
 *         descend to that child
 *    3. Now at a LEAF.  Append P to leaf.data.
 *    4. If leaf.count > LEAF_CAPACITY:
 *         compute four child rects (sub-quadrants of leaf)
 *         create four child nodes
 *         redistribute leaf.data into appropriate child
 *         convert leaf → internal node
 *
 *  RANGE QUERY(rectangle R):
 *    1. If current node's rect doesn't overlap R: return.
 *    2. If LEAF: scan data; report points inside R.
 *    3. Else (INTERNAL): recurse into all 4 children.
 *
 * KEY FORMULAS
 * ────────────
 *   Quadrant of point P in rect (x, y, w, h):
 *     mid_x = x + w/2;  mid_y = y + h/2
 *     west  = (P.x <  mid_x);  east = !west
 *     north = (P.y <  mid_y);  south = !north
 *     quadrant = NW | NE | SW | SE
 *
 *   Half-open intervals (CRUCIAL):
 *     each rect is [x, x+w) × [y, y+h) — UPPER BOUND EXCLUSIVE
 *     so that the 4 sub-quadrants share NO POINTS at the
 *     midpoint and together cover the parent exactly.
 *
 *   Bounding-box overlap:
 *     R1 and R2 overlap iff:
 *       R1.x_max ≥ R2.x_min AND R1.x_min ≤ R2.x_max
 *       AND same for y
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first.
 *      algorithms/quad_tree_helloworld.c is the ANIMATED ncurses
 *      sibling — same algorithm, different demo style.
 *      algorithms/bsp_tree.c and algorithms/kd_tree.c are the
 *      2-children alternatives.
 *   2. PART 1: data types (Point / Rect / QuadNode / QuadrantRects /
 *      QueryRect / QueryResult / GridCanvas) + qt_insert + subdivide
 *      + qt_query.  THE LIBRARY.  Each driver carries a pseudocode
 *      docblock; read those before the bodies.
 *   3. PART 2: main() with step-by-step demo.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   QuadNode                   one node — internal or leaf.
 *   .rect                      bounding rect.  Half-open.
 *   .children[4]               NW, NE, SW, SE pointers (NULL for
 *                              leaf).
 *   .data[], .count            leaf-only data array.
 *   LEAF_CAPACITY = 4          subdivide threshold.
 *
 * Background you need
 * ───────────────────
 *   - Recursion + tree pointer manipulation.
 *   - Half-open intervals [a, b) — left inclusive, right
 *     exclusive.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Octrees (3-D analogue).  Same idea with 8 children.
 *   - Loose quadtrees, point-region quadtrees, etc.  Variants.
 *   - Self-balancing.  Quadtrees adapt to data; balance is
 *     emergent.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ─── ANSI color codes (work in any modern terminal without a library) ── */
#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_DIM     "\033[2m"
#define CLR_YELLOW  "\033[1;33m"   /* points in the tree             */
#define CLR_GREEN   "\033[1;32m"   /* points found by a query        */
#define CLR_CYAN    "\033[36m"     /* tree borders                   */
#define CLR_RED     "\033[1;31m"   /* query rectangle                */
#define CLR_GOLD    "\033[33m"     /* headers and labels             */

/* ================================================================
 * PART 1 — DATA STRUCTURES
 * ================================================================ */

/* ── constants ─────────────────────────────────────────────────── */

/*
 * LEAF_CAPACITY — how many data points a leaf node holds before it
 * must split into four children.  Kept small (4) so subdivisions
 * happen visibly during the 12-point demo.
 */
#define LEAF_CAPACITY  4

/*
 * The demo space is a 56 × 22 integer grid.  Points have (x, y)
 * coordinates within [0, SPACE_W) × [0, SPACE_H).
 * These dimensions fit comfortably inside an 80-column terminal
 * once the grid border (+2 columns, +2 rows) is drawn.
 */
#define SPACE_W  56
#define SPACE_H  22

/* Maximum points a single query can return */
#define QUERY_CAP  64

/* ── data types ─────────────────────────────────────────────────── */

/*
 * Point — an atomic data point stored inside the quadtree.
 *
 * Intent
 *   The smallest unit the tree organises.  Every leaf holds an array
 *   of up to LEAF_CAPACITY of these; internal nodes hold none.  The
 *   tree NEVER inspects the label — it routes purely on (x, y) — so
 *   Point is conceptually a 2-D sort key + an opaque payload.
 *
 * Why integer coordinates (not float)
 *   The demo space is a fixed SPACE_W × SPACE_H character grid.
 *   Integers let us address grid cells directly without rounding;
 *   the renderer plots a point at cells[y][x] with no conversion,
 *   and rect_contains_point's < / >= comparisons stay exact (no FP
 *   rounding flipping a near-equal compare at the half-open boundary).
 *
 * Why a `label` field
 *   Pedagogical only — the step-by-step demo narrates "insert
 *   A(14,5), then B(42,5)" so the letter has to travel with the
 *   point.  Real applications carry whatever payload they need
 *   (row pointer, metric value).  The tree never reads it.
 *
 * Members
 *   x      grid column,  0 ≤ x < SPACE_W
 *   y      grid row,     0 ≤ y < SPACE_H   (y grows DOWN — screen convention)
 *   label  opaque payload; tree treats it as a black box, only the
 *          renderer / dumper inspect it ('A'..'Z' by demo convention)
 *
 * References
 *   The "sort key + opaque payload" pattern is standard for BST-family
 *   nodes — see Knuth TAOCP Vol. 3 §6.2.  The 2-D generalisation is
 *   the basis of all spatial indices ([1] Finkel & Bentley 1974).
 */
typedef struct {
    int  x;       /* grid column [0, SPACE_W)                              */
    int  y;       /* grid row    [0, SPACE_H)  (y grows DOWN)              */
    char label;   /* display glyph 'A'..'Z' — opaque to the tree routing   */
} Point;

/*
 * Rect — axis-aligned rectangle using HALF-OPEN intervals.
 *
 *   Region covered:  x ∈ [x, x+w)   and   y ∈ [y, y+h)
 *
 * Intent
 *   The boundary of one quadtree node.  Stays constant from node
 *   creation to free — re-subdivision allocates fresh child nodes
 *   each with its own (smaller) Rect; the parent's Rect is unchanged.
 *
 * Why HALF-OPEN bounds (upper exclusive)
 *   Subdivision must EXACTLY partition the parent into four children
 *   that share no cell and together cover every cell.  Half-open
 *   intervals achieve this trivially:
 *     NW: [x,     x+w/2)  ×  [y,     y+h/2)
 *     NE: [x+w/2, x+w  )  ×  [y,     y+h/2)
 *     SW: [x,     x+w/2)  ×  [y+h/2, y+h  )
 *     SE: [x+w/2, x+w  )  ×  [y+h/2, y+h  )
 *   With inclusive bounds the four midpoint cells would each belong
 *   to TWO children simultaneously — ambiguous routing, duplicated
 *   points after subdivision.
 *
 *   Same convention as integer slicing in most languages (Python's
 *   a[i:j], C++'s std::span<begin, end>) — upper bound exclusive
 *   composes cleanly with width arithmetic (width = end − start).
 *
 * Worked subdivision (root = 56×22 ⇒ four children)
 *   parent     {0,0, 56, 22}    covers (0..55, 0..21)
 *     NW       {0,0, 28, 11}    covers (0..27,  0..10)
 *     NE       {28,0,28, 11}    covers (28..55, 0..10)
 *     SW       {0,11,28, 11}    covers (0..27,  11..21)
 *     SE       {28,11,28,11}    covers (28..55, 11..21)
 *
 *   Every cell of the parent lands in exactly one child.  Repeated
 *   subdivision halves both axes each level: depth d ⇒ leaves of
 *   width 56/2^d × 22/2^d.  Integer division rounds DOWN, so a
 *   leaf of width 1 produces children of width 0 — caught in
 *   practice by the leaf-capacity check (no over-cap insert into a
 *   1×1 leaf).
 *
 * Members
 *   x, y   top-left corner, INCLUSIVE
 *   w, h   width / height; the rect covers [x, x+w) × [y, y+h)
 *
 * Invariants
 *   w ≥ 0  and  h ≥ 0  (rect_split_into_quadrants halves both via /2,
 *   so a 1×1 leaf splitting yields 0×0 children — a degenerate case
 *   the leaf-capacity check prevents in practice).
 */
typedef struct {
    int x, y;     /* top-left corner, inclusive                            */
    int w, h;     /* width and height; rect covers [x, x+w) × [y, y+h)     */
} Rect;

/*
 * QuadNode — one node in the quadtree.  LEAF or INTERNAL.
 *
 * Intent
 *   The fundamental tree element.  A node owns a region (boundary)
 *   and is either a LEAF (data[] holds up to LEAF_CAPACITY points,
 *   children are NULL) or an INTERNAL node (children non-NULL, data[]
 *   and count are unused).  A leaf becomes internal via subdivide()
 *   when a (LEAF_CAPACITY + 1)th point lands in its region.
 *
 * Leaf-vs-internal discrimination
 *   Encoded as a SINGLE check: `node->nw == NULL` means leaf.
 *   No separate `is_leaf` flag — the all-children-NULL state is the
 *   leaf marker, the all-children-non-NULL state is the internal
 *   marker.  subdivide() flips a leaf to internal in one allocation
 *   burst; nothing in the codebase creates the mixed state.
 *
 * Lifecycle (a node's life is a 3-state machine)
 *
 *   node_new()
 *      │
 *      ▼
 *   ┌──────────────────┐    insert past LEAF_CAPACITY    ┌──────────────┐
 *   │  LEAF (count≤K)  │ ──────── subdivide() ─────────► │   INTERNAL    │
 *   │  data[] valid    │                                 │  4 children   │
 *   │  children=NULL   │                                 │  count=0      │
 *   └──────────────────┘                                 └──────────────┘
 *      │                                                        │
 *      ▼                                                        ▼
 *   tree_free() — post-order recursion: free children first, then self
 *
 *   The LEAF → INTERNAL transition is ONE-WAY: this implementation
 *   never merges children back into a leaf (a "coarsening" operation
 *   some quadtree variants support).  Deletion in general is the
 *   tricky operation; we only handle bulk free via tree_free.
 *
 * Why DATA + CHILDREN in the same struct (vs separate Leaf / Internal types)
 *   - Single allocation path (node_new for both).
 *   - No tagged union needed; the discriminator is the children-NULL
 *     test, which the recursive walks were going to do anyway.
 *   - Cost: each internal node carries a wasted data[LEAF_CAPACITY]
 *     buffer (≈ 36 bytes per node at LEAF_CAPACITY=4).  At demo-scale
 *     N=12, negligible.  At production scale you would split into
 *     Leaf / Internal subclasses or use a memory pool.
 *
 * Compass-direction naming (nw / ne / sw / se)
 *   Mirrors the on-screen geometry:
 *     nw = north-west (top-left,     small x, small y)
 *     ne = north-east (top-right,    large x, small y)
 *     sw = south-west (bottom-left,  small x, large y)
 *     se = south-east (bottom-right, large x, large y)
 *   Reading the order nw / ne / sw / se in code maps to scanning a
 *   page top-left → top-right → bottom-left → bottom-right.
 *
 * Members
 *   boundary               region this node owns; constant from
 *                          node_new to tree_free
 *   data[LEAF_CAPACITY]    point storage; only entries [0, count) valid;
 *                          only meaningful when this is a LEAF
 *   count                  number of points stored; 0 ≤ count ≤
 *                          LEAF_CAPACITY for a leaf; always 0 for an
 *                          internal node
 *   nw, ne, sw, se         child pointers; ALL NULL ⇔ this is a leaf;
 *                          ALL non-NULL ⇔ this is an internal node
 *
 * Invariants
 *   (nw == NULL) ⇔ (ne == NULL) ⇔ (sw == NULL) ⇔ (se == NULL)
 *   nw != NULL  →  count == 0   (internal nodes hold no points)
 *   nw == NULL  →  0 ≤ count ≤ LEAF_CAPACITY
 *   Every point in data[] satisfies rect_contains_point(boundary, …)
 *   Each child's boundary is exactly one quadrant of `boundary`
 *
 * References
 *   [1] Finkel & Bentley 1974 — the original PR-quadtree node layout.
 *   [2] Samet 1984 §3 — variants (region quadtree, MX quadtree)
 *       that store data differently; ours is the point-region (PR)
 *       quadtree.
 */
typedef struct QuadNode QuadNode;
struct QuadNode {
    Rect      boundary;              /* region this node owns                */
    Point     data[LEAF_CAPACITY];   /* point storage; leaf-only             */
    int       count;                 /* 0 ≤ count ≤ LEAF_CAPACITY; 0 if internal */
    QuadNode *nw, *ne, *sw, *se;     /* children; NULL ⇔ this is a leaf      */
};

/*
 * QuadrantRects — the four child rectangles a subdivision produces.
 *
 * Intent
 *   The output of rect_split_into_quadrants: the four sub-rects that
 *   EXACTLY partition a parent Rect.  Naming this group as a struct
 *   lets the geometry primitive return ONE value, and decouples the
 *   "compute four child rects" math (geometry layer) from the
 *   "allocate four child nodes" step (tree-building layer) — both
 *   happen in subdivide but stay conceptually distinct.
 *
 * Why a STRUCT (not four separate Rect out-params or returns)
 *   - Returns one struct; the caller does `q.nw`, `q.ne`, … which
 *     reads as English instead of indexing into an anonymous array.
 *   - Pass-by-value: 4 × 16 = 64 bytes, fits in a couple of registers
 *     on 64-bit ABIs (or one cache line on every ABI).
 *   - Adding a fifth derived rect (e.g. the centre cell, for some
 *     loose-quadtree variant) is a one-line struct edit; no
 *     signature churn at every call site.
 *
 * Naming order (nw / ne / sw / se) — matches QuadNode's children
 *   The fields are in the SAME order as QuadNode.{nw,ne,sw,se} so a
 *   reader scanning subdivide sees the same NW→NE→SW→SE rhythm in
 *   both the geometry struct and the tree-node allocations:
 *       node->nw = node_new(q.nw);
 *       node->ne = node_new(q.ne);   ← matched pairs
 *       node->sw = node_new(q.sw);
 *       node->se = node_new(q.se);
 *
 * Members
 *   nw   north-west quadrant: [x,     x+w/2) × [y,     y+h/2)
 *   ne   north-east quadrant: [x+w/2, x+w  ) × [y,     y+h/2)
 *   sw   south-west quadrant: [x,     x+w/2) × [y+h/2, y+h  )
 *   se   south-east quadrant: [x+w/2, x+w  ) × [y+h/2, y+h  )
 *
 * Invariants (after rect_split_into_quadrants(parent))
 *   nw.x == sw.x == parent.x                       (left edge aligned)
 *   ne.x == se.x == parent.x + parent.w / 2        (mid x aligned)
 *   nw.y == ne.y == parent.y                       (top edge aligned)
 *   sw.y == se.y == parent.y + parent.h / 2        (mid y aligned)
 *   nw.w == ne.w == sw.w == se.w == parent.w / 2   (all same width)
 *   nw.h == ne.h == sw.h == se.h == parent.h / 2   (all same height)
 *   No two of the four overlap; every cell of `parent` is in exactly one.
 *
 * Reference: Finkel & Bentley (1974) [1] for the canonical PR-quadtree
 *   subdivision rule this struct embodies.  Samet (1984) [2] §3.1
 *   surveys variant rules (e.g. unequal splits for hierarchical k-d).
 */
typedef struct {
    Rect nw;      /* NW (top-left)     quadrant of the parent           */
    Rect ne;      /* NE (top-right)    quadrant of the parent           */
    Rect sw;      /* SW (bottom-left)  quadrant of the parent           */
    Rect se;      /* SE (bottom-right) quadrant of the parent           */
} QuadrantRects;

/*
 * QueryRect — inclusive axis-aligned rectangle for range queries.
 *
 * Intent
 *   The user-visible "find all points in this rectangle" parameter
 *   to qt_query.  Named DIFFERENTLY from Rect because the bounds
 *   convention is different:
 *     Rect      uses HALF-OPEN bounds   (node boundaries; for clean
 *                                        partition arithmetic)
 *     QueryRect uses INCLUSIVE bounds   (user input; "from corner
 *                                        (x1, y1) to corner (x2, y2)"
 *                                        reads naturally as inclusive)
 *
 * Why a separate type (not just reuse Rect)
 *   The semantic confusion of "is the boundary inclusive or exclusive?"
 *   would dog every caller.  Two named types make it impossible to
 *   pass one where the other was meant — the compiler enforces the
 *   distinction.  Cost: one extra typedef (8 lines); benefit: every
 *   caller's intent is self-documenting and the predicates
 *   (rect_overlaps_query, point_in_query_rect) can use the natural
 *   form of each side without a mental conversion.
 *
 * Relationship to Rect (when you need to convert)
 *   For INTEGER coordinates, the set { x : x ∈ [a, b) } and the set
 *   { x : a ≤ x ≤ b−1 } cover the same integers.  So:
 *     half-open  Rect{x, y, w, h}     ↔  inclusive QueryRect{x, y, x+w−1, y+h−1}
 *     inclusive  QueryRect{a,b,c,d}   ↔  half-open  Rect{a, b, c−a+1, d−b+1}
 *   This file doesn't convert in practice — the predicates work on
 *   each side natively — but the equivalence is worth knowing for
 *   readers who jump in from kd_tree.c (which uses BBox throughout
 *   with inclusive bounds and a value-minus-1 trick for partition).
 *
 * Members
 *   x1, y1   top-left corner,     INCLUSIVE  (0 ≤ x1 < SPACE_W typical)
 *   x2, y2   bottom-right corner, INCLUSIVE  (x1 ≤ x2, y1 ≤ y2)
 *
 * Invariants
 *   x1 ≤ x2 and y1 ≤ y2  (degenerate equal-corner queries are
 *                         allowed: a single-cell range covering one
 *                         integer cell.  Caller must ensure ordering
 *                         — qt_query does not normalise.)
 *
 * Reference
 *   The inclusive [a, b] form for orthogonal range queries is the
 *   classical formulation — see de Berg et al. (2008) [5] Ch. 5 for
 *   the kd-tree / range-tree analyses and Ch. 14 for the quadtree
 *   variant.  The half-open form Rect uses is the variant native to
 *   subdivision-tree internals.
 */
typedef struct {
    int x1;       /* left   edge, INCLUSIVE — 0 ≤ x1 ≤ x2              */
    int y1;       /* top    edge, INCLUSIVE — 0 ≤ y1 ≤ y2              */
    int x2;       /* right  edge, INCLUSIVE                            */
    int y2;       /* bottom edge, INCLUSIVE                            */
} QueryRect;

/*
 * QueryResult — fixed-capacity output buffer for qt_query.
 *
 * Intent
 *   Replaces the old (Point *results, int *count, int capacity)
 *   parameter trio with one struct.  Three wins:
 *     (i)  qt_query's signature drops from 8 params to 3.
 *    (ii)  The "how many matches and where they live" invariant is
 *          encoded in one type, easy to forward to show_query later.
 *   (iii)  Fixed-size storage means callers don't malloc — one
 *          QueryResult on the stack covers the whole query lifecycle.
 *
 * Why a FIXED capacity (QUERY_CAP)
 *   Demo answers a single query of bounded size; growing on demand
 *   would just hide a malloc behind a slower API.  If qt_query fills
 *   past QUERY_CAP it silently drops further matches (count is
 *   clamped); callers must size QUERY_CAP for their expected worst
 *   case.  For N=12 demo points any QUERY_CAP ≥ 12 is safe.
 *
 * Caller protocol
 *     QueryResult result = { .count = 0 };
 *     qt_query(root, query_rect, &result);
 *     // result.points[0 .. result.count − 1] now hold the matches.
 *     // Order is depth-first NW→NE→SW→SE; not sorted by distance.
 *
 * Members
 *   points[QUERY_CAP]   match buffer; entries [0, count) are valid
 *   count               number of valid entries; 0 ≤ count ≤ QUERY_CAP
 *
 * Invariants
 *   0 ≤ count ≤ QUERY_CAP at all times.
 *   points[i] ∈ query_rect for 0 ≤ i < count (after qt_query returns).
 *   points[count .. QUERY_CAP − 1] are UNINITIALISED — do not read.
 *
 * References
 *   The "output struct with fixed buffer" pattern is from K&R-era C
 *   APIs (getcwd, snprintf) — caller owns storage, callee fills it,
 *   no ownership ambiguity around returned pointers.
 */
typedef struct {
    Point points[QUERY_CAP];   /* match buffer; entries [0, count) are valid */
    int   count;               /* match count; 0 ≤ count ≤ QUERY_CAP         */
} QueryResult;

/*
 * GridCanvas — the ASCII character buffer the visualizer paints into.
 *
 * Intent
 *   Decouples PAINTING (the grid_* helpers below) from PRINTING
 *   (grid_print walks the buffer and emits ANSI-coloured stdout).
 *   Two-phase rendering means we can:
 *     - Paint borders + points + query frame in any order without
 *       worrying about render-time interleaving with terminal state.
 *     - Re-paint the same canvas for multiple views without
 *       re-allocating.
 *     - Swap the printer in future (HTML, SVG) without touching the
 *       painters.
 *
 * Why a STRUCT (not a plain 2-D array typedef)
 *   Lets every visualizer helper take `GridCanvas *gc` and read as
 *   "method on a canvas".  No file-scope render state: show_tree
 *   and show_query each own a fresh canvas on their stack frame, so
 *   two threads or two nested renders can never alias each other.
 *
 * Storage layout
 *   cells[row][col] — ROW-MAJOR, matching terminal scanout.  The +1
 *   column per row is a NUL terminator so any row can be treated as
 *   a printable C string (useful for debugging); grid_print itself
 *   walks per-character to apply per-glyph colour.
 *
 * Glyph alphabet
 *     ' '       empty space
 *     '+'       node-boundary corner
 *     '-'       horizontal node boundary edge
 *     '|'       vertical   node boundary edge
 *     'A'..'Z'  data-point label
 *     '*'       data-point highlighted as a query match
 *     '[' ']'   query-rectangle vertical / corner frame
 *     '~'       query-rectangle horizontal frame
 *
 * Members
 *   cells[SPACE_H][SPACE_W + 1]   row-major glyph buffer;
 *                                 cells[r][SPACE_W] is NUL.
 *
 * Invariants
 *   After grid_clear: every cells[r][c] for c < SPACE_W is ' '.
 *   After any sequence of grid_put / grid_draw_*: every cell is one
 *   of the alphabet glyphs.  cells[r][SPACE_W] is always '\0'.
 */
typedef struct {
    char cells[SPACE_H][SPACE_W + 1];   /* row-major; cells[r][SPACE_W] is NUL */
} GridCanvas;

/* ── memory management ──────────────────────────────────────────── */

/* Allocate and initialise a new leaf node covering 'boundary'. */
static QuadNode *node_new(Rect boundary)
{
    QuadNode *n = malloc(sizeof *n);
    assert(n != NULL);
    n->boundary = boundary;
    n->count    = 0;
    n->nw = n->ne = n->sw = n->se = NULL;
    return n;
}

/* Free a node and all of its descendants (post-order traversal). */
void tree_free(QuadNode *node)
{
    if (!node) return;
    tree_free(node->nw);
    tree_free(node->ne);
    tree_free(node->sw);
    tree_free(node->se);
    free(node);
}

/* ── rectangle / quadrant geometry helpers ──────────────────────── */

/*
 * rect_contains_point — true iff (px, py) lies inside half-open `r`.
 *
 *   Used by qt_insert to route a new point to the right child node,
 *   and by subdivide's redistribution loop to find which child a
 *   displaced point belongs in.
 */
static bool rect_contains_point(Rect r, int px, int py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/*
 * point_in_query_rect — true iff `p` lies inside the inclusive
 * query rectangle `q`.
 *
 *   The "collect" predicate in qt_query: every visited node's points
 *   are tested against this before being added to the result buffer.
 *   INCLUSIVE on both ends — matches the natural "from corner (x1,y1)
 *   to corner (x2,y2)" reading of a user-provided range.
 */
static bool point_in_query_rect(Point p, QueryRect q)
{
    return p.x >= q.x1 && p.x <= q.x2
        && p.y >= q.y1 && p.y <= q.y2;
}

/*
 * rect_overlaps_query — true iff the half-open node `boundary`
 * overlaps the inclusive query rect `q` on BOTH axes.
 *
 *   THE PRUNING PREDICATE — when this returns false, qt_query skips
 *   the whole subtree in O(1).  That's the source of the O(log N + k)
 *   bound (vs O(N) for a linear scan).
 *
 *   Non-overlap on the x-axis happens when EITHER:
 *     query starts past the boundary's right edge:  q.x1 ≥ boundary.x + w
 *     query ends   before the boundary's left edge: q.x2 <  boundary.x
 *   Same logic on y.  Any other case = overlap.  The negation lets us
 *   short-circuit on the FIRST non-overlap axis (DeMorgan saves a few
 *   integer comparisons in the common pruned case).
 */
static bool rect_overlaps_query(Rect boundary, QueryRect q)
{
    int bx = boundary.x, by = boundary.y;
    int bw = boundary.w, bh = boundary.h;
    return !(q.x1 >= bx + bw || q.x2 < bx
          || q.y1 >= by + bh || q.y2 < by);
}

/*
 * rect_split_into_quadrants — partition `r` into NW/NE/SW/SE half-open
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
 *   THE geometry primitive of the quadtree.  Used by subdivide
 *   (to allocate the four child rects with correct boundaries)
 *   and could be used by the renderer too (we don't, currently —
 *   the renderer walks the already-built children's `boundary` fields).
 *
 *   Half-open semantics make the partition EXACT: every integer cell
 *   in `r` falls into exactly one of the four output rects, and the
 *   four output rects share no cell.
 *
 *   Reference: Finkel & Bentley (1974) [1] for the subdivision rule.
 */
static QuadrantRects rect_split_into_quadrants(Rect r)
{
    int half_w = r.w / 2;
    int half_h = r.h / 2;
    int mid_x  = r.x + half_w;
    int mid_y  = r.y + half_h;
    return (QuadrantRects){
        .nw = { r.x,   r.y,   half_w, half_h },
        .ne = { mid_x, r.y,   half_w, half_h },
        .sw = { r.x,   mid_y, half_w, half_h },
        .se = { mid_x, mid_y, half_w, half_h },
    };
}

/* ── core operations ────────────────────────────────────────────── */

/* Forward declaration: subdivide() needs qt_insert() to redistribute. */
static bool qt_insert(QuadNode *node, Point p);

/*
 * redistribute_leaf_into_children — snapshot the leaf's data[], clear
 * the leaf's count, then re-insert each snapshot point into the
 * (now-non-NULL) children.
 *
 *   Pseudocode:
 *     snapshot displaced[] := node->data[0 .. count − 1]
 *     reset node->count := 0      (so the children-walk below treats
 *                                  node as internal, not over-capacity)
 *     for each displaced point p:
 *       try qt_insert(child_c, p) for c in nw, ne, sw, se
 *       the first success wins (exactly one child's boundary
 *       contains p, by the partition invariant)
 *
 *   The snapshot is necessary because qt_insert overwrites node->data
 *   indirectly: it traverses into children whose count fields we are
 *   updating, but the leaf node itself stays at count=0 throughout.
 */
static void redistribute_leaf_into_children(QuadNode *node)
{
    int n = node->count;
    Point displaced[LEAF_CAPACITY];
    memcpy(displaced, node->data, (size_t)n * sizeof(Point));
    node->count = 0;

    QuadNode *children[4] = { node->nw, node->ne, node->sw, node->se };
    for (int i = 0; i < n; i++)
        for (int c = 0; c < 4; c++)
            if (qt_insert(children[c], displaced[i])) break;
}

/*
 * subdivide — split a full leaf into four equal child nodes.
 *
 *   Pseudocode:
 *     q := rect_split_into_quadrants(node->boundary)
 *     allocate four child leaves with boundaries q.nw, q.ne, q.sw, q.se
 *     redistribute_leaf_into_children(node)
 *
 *   After subdivide(), `node` has become an INTERNAL node:
 *     count = 0
 *     nw, ne, sw, se all non-NULL
 *     each child holds the displaced points that fall in its quadrant
 *
 *   Before:  [node: LEAF_CAPACITY pts]
 *
 *   After:   [node: internal, 0 pts]
 *              /         |         |         \
 *           [nw]       [ne]      [sw]       [se]
 *        (pts redistributed by which quadrant they fall in)
 */
static void subdivide(QuadNode *node)
{
    QuadrantRects q = rect_split_into_quadrants(node->boundary);
    node->nw = node_new(q.nw);
    node->ne = node_new(q.ne);
    node->sw = node_new(q.sw);
    node->se = node_new(q.se);
    redistribute_leaf_into_children(node);
}

/*
 * try_append_to_leaf — store p in node->data[] if there's room.
 *
 *   Pseudocode:
 *     if node->count < LEAF_CAPACITY:
 *       node->data[node->count++] := p
 *       return true                  ← appended; no subdivision needed
 *     else:
 *       return false                 ← full; caller must subdivide
 *
 *   The "leaf still has capacity" fast-path: just one array write +
 *   one counter bump.  When this returns false the leaf is full and
 *   the caller (qt_insert) triggers subdivide().
 *
 *   Precondition: `node` is a LEAF (node->nw == NULL).
 */
static bool try_append_to_leaf(QuadNode *node, Point p)
{
    if (node->count >= LEAF_CAPACITY) return false;
    node->data[node->count++] = p;
    return true;
}

/*
 * route_into_quadrant_child — deliver p to the unique child whose
 * boundary contains it.
 *
 *   Pseudocode:
 *     for c in {nw, ne, sw, se}:
 *       if qt_insert(child[c], p):  return true
 *     return false                  ← unreachable when called correctly
 *
 *   By the QuadrantRects partition invariant, EXACTLY ONE of the four
 *   children's boundaries contains p, so the first success wins and
 *   the loop short-circuits.  Reaching the false return means the
 *   caller's boundary check (rect_contains_point at the top of
 *   qt_insert) succeeded but no child accepted — that's a bug.
 *
 *   Precondition: `node` is INTERNAL (all four children non-NULL).
 */
static bool route_into_quadrant_child(QuadNode *node, Point p)
{
    QuadNode *children[4] = { node->nw, node->ne, node->sw, node->se };
    for (int c = 0; c < 4; c++)
        if (qt_insert(children[c], p)) return true;
    return false;
}

/*
 * qt_insert — add point p into the subtree rooted at `node`.
 *
 *   Pseudocode:
 *     if NOT rect_contains_point(node->boundary, p):   return false
 *     if node is LEAF:
 *       if try_append_to_leaf succeeds:                return true
 *       else:                                          subdivide(node)
 *                                                      (now internal)
 *     return route_into_quadrant_child(node, p)
 *
 *   The leaf-vs-internal discrimination is one branch: node->nw == NULL.
 *   After subdivide() the node is internal, so the same
 *   route_into_quadrant_child call delivers p to the correct grandchild.
 *
 *   Returns true if p was accepted somewhere in this subtree.
 *   Returns false ONLY if p falls outside node->boundary — used
 *   internally during redistribution (a displaced point trying every
 *   child until one's boundary accepts it).
 */
bool qt_insert(QuadNode *node, Point p)
{
    if (!rect_contains_point(node->boundary, p.x, p.y)) return false;

    if (!node->nw) {                                /* leaf path */
        if (try_append_to_leaf(node, p))            return true;
        subdivide(node);                            /* full → split */
        /* fall through: node is now internal */
    }

    return route_into_quadrant_child(node, p);      /* internal path */
}

/*
 * qt_query — collect all points lying inside `q` into `out`.
 *
 *   Pseudocode (depth-first, prune by rect_overlaps_query):
 *     if out is full or node is null:                       return
 *     if NOT rect_overlaps_query(node->boundary, q):        return   ← PRUNE
 *     for each point p in node->data:
 *       if point_in_query_rect(p, q):  out->points[out->count++] := p
 *     recurse into node->nw, ne, sw, se
 *
 *   The pruning step is what gives quadtrees their efficiency: when
 *   a node's boundary doesn't overlap the query, the WHOLE SUBTREE
 *   is skipped in O(1) — no per-point work.  Visits only the nodes
 *   whose region straddles the query boundary plus those wholly
 *   inside it.  Average cost: O(log N + k), where k = matches.
 *
 *   Parameters:
 *     node    current subtree root (NULL is a valid no-op)
 *     q       inclusive query rectangle
 *     out     fixed-capacity result buffer; caller zeroes out->count
 *
 *   Returns nothing; results land in `out->points[0 .. out->count-1]`.
 *   The full-bag silent drop (when out->count reaches QUERY_CAP) is
 *   intentional — see QueryResult docblock for the rationale.
 */
void qt_query(QuadNode *node, QueryRect q, QueryResult *out)
{
    if (!node || out->count >= QUERY_CAP)              return;
    if (!rect_overlaps_query(node->boundary, q))       return;   /* prune subtree */

    for (int i = 0; i < node->count && out->count < QUERY_CAP; i++)
        if (point_in_query_rect(node->data[i], q))
            out->points[out->count++] = node->data[i];

    qt_query(node->nw, q, out);
    qt_query(node->ne, q, out);
    qt_query(node->sw, q, out);
    qt_query(node->se, q, out);
}

/* ── inspection helpers ─────────────────────────────────────────── */

/* Count the total number of nodes in the tree (including internal). */
int tree_node_count(QuadNode *node)
{
    if (!node) return 0;
    return 1
         + tree_node_count(node->nw) + tree_node_count(node->ne)
         + tree_node_count(node->sw) + tree_node_count(node->se);
}

/* Count total data points across all leaf nodes. */
int tree_point_count(QuadNode *node)
{
    if (!node) return 0;
    return node->count
         + tree_point_count(node->nw) + tree_point_count(node->ne)
         + tree_point_count(node->sw) + tree_point_count(node->se);
}

/*
 * max4 — maximum of four ints, expanded inline-friendly.
 *
 *   The quadtree's 4-ary fan-out makes "max over the 4 children"
 *   appear in tree_depth (and could appear in any future per-subtree
 *   aggregate).  Three pairwise max calls; standard fold pattern.
 */
static inline int max4(int a, int b, int c, int d)
{
    int m = a > b ? a : b;
    m     = m > c ? m : c;
    return  m > d ? m : d;
}

/*
 * tree_depth — depth of the deepest leaf below `node` (root = 0).
 *
 *   Pseudocode:
 *     if node is null:    return current_depth − 1   (no node here)
 *     if node is LEAF:    return current_depth       (this IS the deepest)
 *     else:               return max4 of the four children's depths
 *
 *   `current_depth` is the depth `node` itself sits at; callers pass
 *   0 when calling on the root.  The "+1" recursion threads the depth
 *   downward without a separate accumulator.
 */
int tree_depth(QuadNode *node, int current_depth)
{
    if (!node)     return current_depth - 1;
    if (!node->nw) return current_depth;             /* leaf */
    return max4(
        tree_depth(node->nw, current_depth + 1),
        tree_depth(node->ne, current_depth + 1),
        tree_depth(node->sw, current_depth + 1),
        tree_depth(node->se, current_depth + 1)
    );
}

/*
 * print_indent — emit `depth * 2` spaces for the dump's tree shape.
 *   Standard tree-printing idiom: each depth level is 2 columns deeper.
 */
static void print_indent(int depth)
{
    for (int i = 0; i < depth * 2; i++) putchar(' ');
}

/*
 * print_node_header — emit `LABEL [x,y W×H]` for one node.
 *   The common prefix every dump line starts with — DIM label
 *   followed by the boundary rect in (x,y,w,h) form.  The "× =
 *   U+00D7" comment is historical; we keep the UTF-8 bytes inline
 *   so the dump shows ✗-style multiplication, not the ASCII 'x'.
 */
static void print_node_header(const QuadNode *node, const char *label)
{
    printf(CLR_DIM "%s " CLR_RESET, label);
    printf("[%d,%d %d\xc3\x97%d]",          /* × = UTF-8 U+00D7  */
           node->boundary.x, node->boundary.y,
           node->boundary.w, node->boundary.h);
}

/*
 * print_leaf_data_line — emit " — N pt(s): A(x,y), B(x,y), …" newline.
 *
 *   Pseudocode:
 *     emit " — N pt(s): "
 *     for each point i in 0..count-1:
 *       emit LABEL(x,y) in yellow
 *       if not last:  emit ", "
 *     emit newline
 *
 *   Singular / plural agreement ("1 pt" vs "2 pts") via the ternary.
 *   Only called when the leaf has at least one point — empty / internal
 *   nodes use their own one-liner suffix (tree_dump dispatches).
 */
static void print_leaf_data_line(const QuadNode *node)
{
    printf(" — %d pt%s: ", node->count, node->count == 1 ? "" : "s");
    for (int i = 0; i < node->count; i++) {
        printf(CLR_YELLOW "%c" CLR_RESET "(%d,%d)",
               node->data[i].label, node->data[i].x, node->data[i].y);
        if (i < node->count - 1) printf(", ");
    }
    putchar('\n');
}

/*
 * tree_dump — print an indented text representation of the tree.
 *
 *   Pseudocode (3-way dispatch on node kind):
 *     if node is null:                  return
 *     print_indent(depth);  print_node_header(node, label)
 *     if internal:                      emit " — internal\n"
 *     elif leaf with 0 points:          emit " — empty\n"
 *     else (leaf with ≥1 points):       print_leaf_data_line(node)
 *     recurse into NW / NE / SW / SE
 *
 * Example output:
 *   [0,0 56×22] — internal
 *     NW [0,0 28×11] — internal
 *       NW [0,0 14×5] — 2 pts: E(4,3) H(10,2)
 *       NE [14,0 14×5] — empty
 *       SW [0,5 14×6] — 1 pt: F(7,8)
 *       SE [14,5 14×6] — 2 pts: A(14,5) G(20,9)
 *     NE [28,0 28×11] — 3 pts: B(42,5) I(38,3) L(44,8)
 *     ...
 */
void tree_dump(QuadNode *node, int depth, const char *label)
{
    if (!node) return;

    print_indent(depth);
    print_node_header(node, label);

    if (node->nw)              printf(" — internal\n");
    else if (node->count == 0) printf(" — empty\n");
    else                       print_leaf_data_line(node);

    tree_dump(node->nw, depth + 1, "NW");
    tree_dump(node->ne, depth + 1, "NE");
    tree_dump(node->sw, depth + 1, "SW");
    tree_dump(node->se, depth + 1, "SE");
}

/* ── ASCII grid visualizer ──────────────────────────────────────── */

/*
 * The visualizer paints into a GridCanvas (SPACE_W × SPACE_H character
 * buffer), then walks the canvas with grid_print to colour each glyph.
 * No global state — the canvas is a local in show_tree / show_query,
 * passed to every helper.  See GridCanvas docblock for the alphabet
 * and storage convention.
 *
 * Painter's-order: grid_render_tree first draws each node's border,
 * recurses into its children (which paint their own borders into the
 * interior), THEN paints the data point glyph over whatever border
 * line passed through that cell — so a label is never hidden by its
 * own enclosing rectangle.
 */

/* grid_clear — fill canvas with spaces; set per-row NUL terminator. */
static void grid_clear(GridCanvas *gc)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(gc->cells[r], ' ', SPACE_W);
        gc->cells[r][SPACE_W] = '\0';
    }
}

/* grid_put — place a single glyph at (x, y), clipped to canvas bounds. */
static void grid_put(GridCanvas *gc, int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        gc->cells[y][x] = ch;
}

/*
 * grid_draw_border — frame `r` with '+' corners, '-' top/bottom edges,
 * '|' left/right edges.  Interior of `r` is left untouched.
 *
 *   Used by grid_render_tree to draw every quadtree node's boundary.
 *   When two adjacent nodes share an edge their borders overlap with
 *   the same glyph, so no special intersection-merge logic is needed
 *   (unlike kd_tree.c which had to merge '|' meets '-' into '+').
 */
static void grid_draw_border(GridCanvas *gc, Rect r)
{
    int x0 = r.x,           y0 = r.y;
    int x1 = r.x + r.w - 1, y1 = r.y + r.h - 1;

    for (int x = x0; x <= x1; x++) {
        grid_put(gc, x, y0, '-');
        grid_put(gc, x, y1, '-');
    }
    for (int y = y0; y <= y1; y++) {
        grid_put(gc, x0, y, '|');
        grid_put(gc, x1, y, '|');
    }
    grid_put(gc, x0, y0, '+');
    grid_put(gc, x1, y0, '+');
    grid_put(gc, x0, y1, '+');
    grid_put(gc, x1, y1, '+');
}

/*
 * grid_draw_query_rect — frame `q` with [ ] ~ glyphs.  Top / bottom
 * edges use '~', left / right use '[' / ']'.  Corners are forced to
 * '[' or ']' so the frame reads as a box, not a wave.  Distinct from
 * the tree-border alphabet (+ - |) so frames never look like splits.
 */
static void grid_draw_query_rect(GridCanvas *gc, QueryRect q)
{
    for (int x = q.x1; x <= q.x2; x++) {
        grid_put(gc, x, q.y1, '~');
        grid_put(gc, x, q.y2, '~');
    }
    for (int y = q.y1; y <= q.y2; y++) {
        grid_put(gc, q.x1, y, '[');
        grid_put(gc, q.x2, y, ']');
    }
    grid_put(gc, q.x1, q.y1, '[');
    grid_put(gc, q.x2, q.y1, ']');
    grid_put(gc, q.x1, q.y2, '[');
    grid_put(gc, q.x2, q.y2, ']');
}

/*
 * point_is_highlighted — is this stored point in the highlight set?
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
 * paint_node_points — emit each stored point as its label glyph,
 * swapped to '*' when the point appears in `highlight`.
 *
 *   Pseudocode:
 *     for each stored point p in node->data:
 *       glyph := point_is_highlighted(p, highlight) ? '*' : p.label
 *       grid_put(gc, p.x, p.y, glyph)
 *
 *   Called by grid_render_tree AFTER the border is drawn so that
 *   labels overwrite any border segment passing through the same
 *   cell — a label is never hidden by its own enclosing rectangle.
 *   Internal nodes have count == 0, so the loop is a no-op there.
 */
static void paint_node_points(GridCanvas *gc, const QuadNode *node,
                              const QueryResult *highlight)
{
    for (int i = 0; i < node->count; i++) {
        Point p     = node->data[i];
        char  glyph = point_is_highlighted(p, highlight) ? '*' : p.label;
        grid_put(gc, p.x, p.y, glyph);
    }
}

/*
 * grid_render_tree — recursively paint borders + labels for every node.
 *
 *   Pseudocode (painter's order, parent-first):
 *     if node is null:           return
 *     grid_draw_border           ← this node's rect frame
 *     paint_node_points          ← labels for any stored points
 *     recurse into nw, ne, sw, se
 *
 *   Parent border first so child borders draw the interior
 *   subdivision lines on top; point labels paint LAST within each
 *   node so they sit visibly on whatever border passes through
 *   their cell.
 *
 *   `highlight` is the QueryResult to mark with '*'; pass NULL to
 *   render plain labels (no query in progress).
 */
static void grid_render_tree(GridCanvas *gc, QuadNode *node,
                             const QueryResult *highlight)
{
    if (!node) return;

    grid_draw_border (gc, node->boundary);
    paint_node_points(gc, node, highlight);

    grid_render_tree(gc, node->nw, highlight);
    grid_render_tree(gc, node->ne, highlight);
    grid_render_tree(gc, node->sw, highlight);
    grid_render_tree(gc, node->se, highlight);
}

/*
 * glyph_color_for — pick the ANSI colour code for a single canvas glyph.
 *   Decouples the colour table from grid_print's loop.  Extending the
 *   alphabet (new highlight class) only requires a new case here.
 */
static const char *glyph_color_for(char ch)
{
    if (ch >= 'A' && ch <= 'Z')               return CLR_YELLOW;
    if (ch == '*')                            return CLR_GREEN;
    if (ch == '[' || ch == ']' || ch == '~')  return CLR_RED;
    if (ch == '+' || ch == '-' || ch == '|')  return CLR_CYAN;
    return NULL;     /* plain putchar */
}

/* grid_print — print the canvas inside a dim border, colouring per glyph. */
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
    /* consume any leftover newline then wait for the user */
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF)
        ;
}

static void print_separator(void)
{
    printf(CLR_DIM "------------------------------------------------------------\n"
           CLR_RESET);
}

/*
 * show_tree — render the current tree state and print a stats summary.
 *
 *   Allocates a fresh GridCanvas on the stack (≈ 1.2 KB), paints the
 *   tree into it (no highlights), prints it, then drops it.
 */
static void show_tree(const char *title, QuadNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);

    GridCanvas gc;
    grid_clear(&gc);
    grid_render_tree(&gc, root, NULL);
    grid_print(&gc);

    printf("  nodes=%d  points=%d  depth=%d\n",
           tree_node_count(root),
           tree_point_count(root),
           tree_depth(root, 0));
}

/*
 * show_query — render tree + query rectangle + highlighted matches.
 *
 *   Same canvas pattern as show_tree, plus a query-frame overlay and
 *   a summary line listing each found point.  Pass `result` (non-NULL)
 *   to mark its points with '*' instead of their label glyph.
 */
static void show_query(const char *title, QuadNode *root,
                       QueryRect q, const QueryResult *result)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);

    GridCanvas gc;
    grid_clear(&gc);
    grid_render_tree(&gc, root, result);
    grid_draw_query_rect(&gc, q);
    grid_print(&gc);

    printf("  Query rect: [%d,%d] to [%d,%d]   Found %d point%s:",
           q.x1, q.y1, q.x2, q.y2,
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
    printf(CLR_BOLD "  QUADTREE — data structure walkthrough\n" CLR_RESET);
    print_separator();
    printf(
        "\n"
        "  A quadtree partitions a 2-D space into four quadrants\n"
        "  (NW / NE / SW / SE) whenever a node accumulates more\n"
        "  than LEAF_CAPACITY=%d points.\n"
        "\n"
        "  This demo inserts 12 labelled points into a %d×%d grid,\n"
        "  pausing so you can watch every subdivision happen.\n"
        "  Then it runs a range query and shows the pruning.\n"
        "\n"
        "  Legend:  " CLR_CYAN "+ - |" CLR_RESET " tree borders   "
                      CLR_YELLOW "A" CLR_RESET " data point   "
                      CLR_RED   "[ ~ ]" CLR_RESET " query rect   "
                      CLR_GREEN "*" CLR_RESET " found\n",
        LEAF_CAPACITY, SPACE_W, SPACE_H
    );
    press_enter();

    /* ── create the tree ──────────────────────────────────────── */

    Rect full_space = { .x = 0, .y = 0, .w = SPACE_W, .h = SPACE_H };
    QuadNode *root  = node_new(full_space);

    show_tree("Step 1 — empty tree  (capacity = 4 pts per leaf)",  root);
    printf("  Root is one leaf covering the whole %d×%d space.\n", SPACE_W, SPACE_H);
    press_enter();

    /* ── insert four points: root fills up but does not split yet ── */

    print_separator();
    printf(CLR_BOLD "  Step 2 — insert A B C D\n" CLR_RESET);
    printf("  Each goes into the single root leaf.  No subdivision yet.\n");
    press_enter();

    qt_insert(root, (Point){14,  5, 'A'});
    qt_insert(root, (Point){42,  5, 'B'});
    qt_insert(root, (Point){14, 16, 'C'});
    qt_insert(root, (Point){42, 16, 'D'});
    show_tree("  After inserting A B C D", root);
    printf("  Root holds %d/%d points — exactly at capacity.\n",
           root->count, LEAF_CAPACITY);
    press_enter();

    /* ── fifth point triggers subdivision ────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 3 — insert E(4,3) — triggers ROOT SUBDIVISION\n"
           CLR_RESET);
    printf(
        "\n"
        "  The root already holds LEAF_CAPACITY=%d points.\n"
        "  qt_insert() sees the leaf is full and calls subdivide():\n"
        "\n"
        "    1. Calculate midpoint of root: (%d, %d)\n"
        "    2. Allocate four child nodes: NW / NE / SW / SE\n"
        "    3. Redistribute A B C D into the correct child\n"
        "    4. Insert E into the correct child\n",
        LEAF_CAPACITY, SPACE_W / 2, SPACE_H / 2
    );
    press_enter();

    qt_insert(root, (Point){ 4,  3, 'E'});
    show_tree("  After inserting E — root is now an internal node", root);
    printf("  Root: internal.  Children: NW=[A,E]  NE=[B]  SW=[C]  SE=[D]\n");
    press_enter();

    /* ── fill NW to capacity ──────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 4 — insert F(7,8) and G(20,9) into NW\n" CLR_RESET);
    printf("  Both land in the NW quadrant [0,0..28,11].  NW will reach capacity.\n");
    press_enter();

    qt_insert(root, (Point){ 7,  8, 'F'});
    qt_insert(root, (Point){20,  9, 'G'});
    show_tree("  After inserting F G — NW holds 4 points [A E F G]", root);
    printf("  NW is at capacity (%d/%d).  The next point landing here will split it.\n",
           LEAF_CAPACITY, LEAF_CAPACITY);
    press_enter();

    /* ── NW subdivides ────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 5 — insert H(10,2) — triggers NW SUBDIVISION\n"
           CLR_RESET);
    printf(
        "\n"
        "  H lands in the NW quadrant, which is full.\n"
        "  NW midpoint: (%d, %d).  NW splits into four grandchildren:\n"
        "\n"
        "    NW.NW [0,0  14×5]  gets: E(4,3)  H(10,2)\n"
        "    NW.NE [14,0 14×5]  gets: (empty)\n"
        "    NW.SW [0,5  14×6]  gets: F(7,8)\n"
        "    NW.SE [14,5 14×6]  gets: A(14,5)  G(20,9)\n",
        SPACE_W / 4, SPACE_H / 4
    );
    press_enter();

    qt_insert(root, (Point){10,  2, 'H'});
    show_tree("  After inserting H — NW has split into four grandchildren", root);
    press_enter();

    /* ── fill remaining quadrants ─────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 6 — insert I J K L to fill NE / SE / SW\n" CLR_RESET);
    press_enter();

    qt_insert(root, (Point){38,  3, 'I'});   /* NE */
    qt_insert(root, (Point){48, 16, 'J'});   /* SE */
    qt_insert(root, (Point){20, 16, 'K'});   /* SW */
    qt_insert(root, (Point){44,  8, 'L'});   /* NE */
    show_tree("  After inserting I J K L — final tree state", root);
    press_enter();

    /* ── full tree dump ───────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 7 — tree structure (indented text dump)\n" CLR_RESET);
    printf("\n");
    tree_dump(root, 0, "root");
    printf("\n"
           "  Total nodes  : %d  (1 root + internal + leaf nodes)\n"
           "  Total points : %d\n"
           "  Tree depth   : %d  (root = 0)\n",
           tree_node_count(root),
           tree_point_count(root),
           tree_depth(root, 0));
    press_enter();

    /* ── range query ──────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 8 — range query: find all points in [3,2]..[23,9]\n"
           CLR_RESET);
    printf(
        "\n"
        "  qt_query() visits each node and applies two rules:\n"
        "\n"
        "    PRUNE  — if the search rect does NOT overlap a node's\n"
        "             boundary, skip that node and ALL its descendants.\n"
        "    COLLECT — if a node's boundary overlaps, check each point\n"
        "             it holds; if the point is inside the rect, add it.\n"
        "\n"
        "  This is the O(log N + k) property: only a small fraction of\n"
        "  nodes are visited.  NE, SW, SE are pruned immediately.\n"
    );
    press_enter();

    QueryRect   q      = { .x1 = 3, .y1 = 2, .x2 = 23, .y2 = 9 };
    QueryResult result = { .count = 0 };
    qt_query  (root, q, &result);
    show_query("  Query result", root, q, &result);

    printf(
        "\n"
        "  Nodes visited: root → NW → NW.NW (E,H found) → NW.NE (empty)\n"
        "               → NW.SW (F found) → NW.SE (A,G found)\n"
        "  NE, SW, SE pruned (their boundaries do not overlap the rect).\n"
    );
    press_enter();

    /* ── cleanup ──────────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Done.\n" CLR_RESET
           "\n"
           "  Summary of operations implemented in this file:\n"
           "\n"
           "    qt_insert(root, point)               O(log N)\n"
           "    qt_query(root, query_rect, &out)     O(log N + k)\n"
           "    subdivide(node)                      O(LEAF_CAPACITY)\n"
           "    tree_node_count / tree_point_count   O(nodes)\n"
           "    tree_depth / tree_dump               O(nodes)\n"
           "    tree_free(root)                      O(nodes)\n"
           "\n");

    tree_free(root);
    return 0;
}
