/*
 * geometry/bsp_tree.c — Binary Space Partition tree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1  (lines ~130-340)  — the BSP tree library
 *     Data structures, memory management, core operations,
 *     inspection helpers, and an ASCII grid visualizer.
 *     This part has no I/O — it is a reusable module.
 *
 *   PART 2  (lines ~340-end) — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every split live,
 *     then demonstrates a range query.  Press Enter each step.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/bsp_tree.c -o bsp_tree
 *
 * Run:
 *   ./bsp_tree
 */

/* ── CONCEPTS ───────────────────────────────────────────────────────────── *
 *
 * BSP tree vs Quadtree vs K-D tree (all three are in geometry/):
 *
 *   Quadtree  — splits into FOUR equal children (NW/NE/SW/SE) when a leaf
 *               overflows LEAF_CAPACITY.  Both axes cut simultaneously.
 *               Each internal node holds no data; data lives in leaves only.
 *
 *   K-D tree  — each node holds EXACTLY ONE data point AND is the split.
 *               Axis alternates with depth.  Faster nearest-neighbour search.
 *               No concept of a "region" per node — split tracks one coord.
 *
 *   BSP tree  — (this file) splits into TWO children per node; each node
 *               owns the boundary rectangle of its region.  Internal nodes
 *               store the split axis and position; data lives in leaf nodes.
 *               Closest to how the original game BSP trees worked.
 *
 * Historical context — Doom (1993) and Quake:
 *   Doom precomputed a BSP tree over the map's line segments at build time.
 *   At render time, the engine traversed the BSP front-to-back to determine
 *   which walls were visible — O(log N) per frame instead of O(N) per segment.
 *   Quake used BSP for collision detection between the player and geometry.
 *   This demo uses the simpler "axis-aligned point BSP" (not arbitrary planes),
 *   which is sufficient to show the spatial subdivision structure.
 *
 * Split axis alternation:
 *   Even depth (0, 2, 4…) → SPLIT_VERTICAL   — a vertical line x = split_pos.
 *                            front = left  half  (x < split_pos)
 *                            back  = right half  (x ≥ split_pos)
 *   Odd  depth (1, 3, 5…) → SPLIT_HORIZONTAL — a horizontal line y = split_pos.
 *                            front = top    half (y < split_pos)
 *                            back  = bottom half (y ≥ split_pos)
 *   Alternating axes prevents one axis from dominating, keeping the tree balanced
 *   in both dimensions (assuming points are spatially distributed).
 *
 * Split position (bsp_insert):
 *   The split_pos is set to the MIDPOINT of the boundary region along the
 *   split axis.  This guarantees O(log N) depth even for sorted inputs —
 *   unlike the K-D tree where split_pos equals the inserted point's coordinate,
 *   making depth sensitive to insertion order.
 *
 * Time complexity:
 *   Insert:      O(log N) average (balanced mid-point split)
 *   Range query: O(√N + k) average in 2-D, k = points found
 *   The bounding-box pruning in bsp_query skips whole subtrees in O(1),
 *   same mechanism as kd_query — check overlap before recursing.
 *
 * References
 * ──────────
 *   ── The foundational BSP paper ──────────────────────────────────
 *   [1] Fuchs, H., Kedem, Z. M. & Naylor, B. F. (1980), "On Visible
 *       Surface Generation by A Priori Tree Structures", SIGGRAPH
 *       '80 — the original BSP paper. Introduced the front-to-back
 *       traversal idea that this file's bsp_query inherits.
 *
 *   ── Multi-dimensional spatial indexes (the family) ──────────────
 *   [2] Bentley, J. L. (1975), "Multidimensional binary search trees
 *       used for associative searching", CACM 18(9), pp. 509-517 —
 *       k-d tree paper. The alternating-axis split in subdivide()
 *       comes directly from this work.
 *   [3] Finkel, R. A. & Bentley, J. L. (1974), "Quad trees: a data
 *       structure for retrieval on composite keys", Acta Informatica
 *       4(1) — the quadtree alternative; cited in the
 *       BSP-vs-Quadtree-vs-K-D contrast in CONCEPTS.
 *   [4] Samet, H. (2006), "Foundations of Multidimensional and
 *       Metric Data Structures", Morgan Kaufmann — encyclopaedic
 *       reference for the whole family of spatial indexes.
 *
 *   ── Computational geometry (formal treatment) ───────────────────
 *   [5] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 12 BSP trees; Ch. 5 k-d trees;
 *       formal treatment of range queries and the O(√N + k) bound.
 *
 *   ── Game-engine applications ────────────────────────────────────
 *   [6] Abrash, M. (1997), "Graphics Programming Black Book", Coriolis
 *       — chapters on the Doom + Quake BSP renderers. Practical
 *       account of how Carmack applied Fuchs/Kedem/Naylor at game
 *       frame rates on 1990s hardware.
 *   [7] van Waveren, J. M. P. (2001), "The Quake III Arena Bot",
 *       Master's thesis, TU Delft — modern BSP usage for collision
 *       and visibility queries; cites the same Fuchs paper as ref.
 *
 *   ── Range searching (the bsp_query analysis) ────────────────────
 *   [8] Lee, D. T. & Wong, C. K. (1977), "Worst-case analysis for
 *       region and partial region searches in multidimensional
 *       binary search trees and balanced quad trees", Acta
 *       Informatica 9(1) — the O(√N + k) range-query bound.
 *
 *   ── Online quick reference ──────────────────────────────────────
 *   [9] https://en.wikipedia.org/wiki/Binary_space_partitioning
 *  [10] https://en.wikipedia.org/wiki/K-d_tree
 *  [11] https://en.wikipedia.org/wiki/Range_searching
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A BSP TREE recursively divides space by AXIS-ALIGNED CUTS.
 * Every internal node has TWO children — one for each side of
 * its cut.  Cuts ALTERNATE BETWEEN AXES with depth: depth 0
 * cuts vertical (x), depth 1 horizontal (y), depth 2 vertical
 * again, etc.  Points live in LEAF nodes; when a leaf overflows
 * LEAF_CAPACITY, it splits at its bounding box's midpoint along
 * the next axis.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you're filing index cards into a tall, narrow filing
 * cabinet.  When a drawer fills up, you put a divider in the
 * middle and split the cards by which half they fall in.  The
 * NEXT time a drawer fills, you SPLIT THE OTHER WAY (turning
 * the cabinet 90°).  After many splits the cabinet is a perfect
 * binary tree of dividers; each card has a unique path from
 * the top.  Querying "find all cards in this rectangle" walks
 * the tree, skipping any drawer whose dividers exclude the
 * region.
 *
 * BSP vs OTHER SPATIAL TREES
 * ──────────────────────────
 *
 *   QUADTREE: every internal node splits BOTH axes at once,
 *             producing 4 children.  Simpler conceptually but
 *             more children per level → SHALLOWER tree.  See
 *             algorithms/quadtree.c.
 *
 *   K-D TREE: like BSP but each NODE holds a data point AND IS
 *             the split.  No "leaf capacity"; every level holds
 *             one point.  See algorithms/kd_tree.c.
 *
 *   BSP TREE: 2 children per internal node; data only in leaves;
 *             splits at MIDPOINT of region (not at a data point).
 *             Closest to game-engine BSP usage.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  INSERT(point P):
 *    1. Start at root.
 *    2. While current node is INTERNAL:
 *         consult node's split axis + split_pos
 *         descend to FRONT or BACK child accordingly
 *    3. Now at a LEAF.  Append P to leaf's data array.
 *    4. If leaf's count > LEAF_CAPACITY:
 *         compute next split axis (alternates with depth)
 *         compute split_pos = midpoint of leaf's region
 *         create two child leaves
 *         redistribute leaf's data into front/back child
 *         convert leaf → internal node
 *
 *  RANGE QUERY(rectangle R):
 *    1. Start at root.
 *    2. If current node's BOUNDING BOX doesn't overlap R:
 *         skip whole subtree.  ← THE KEY OPTIMISATION
 *    3. Else if current is LEAF:
 *         scan leaf's data, report points inside R.
 *    4. Else (INTERNAL):
 *         recurse into FRONT child + BACK child.
 *
 * KEY FORMULAS
 * ────────────
 *   Split axis at depth d:  d % 2 == 0 → vertical (x-cut)
 *                           d % 2 == 1 → horizontal (y-cut)
 *
 *   Mid-point split:        split_pos = (rect.min + rect.max) / 2
 *                           along the chosen axis
 *
 *   Front/back assignment:  vertical:    point.x < split_pos → front
 *                           horizontal:  point.y < split_pos → front
 *
 *   Bounding-box overlap:   for each axis: (rect1.min ≤ rect2.max)
 *                                          AND (rect1.max ≥ rect2.min)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Mid-point split assumes data is roughly UNIFORMLY
 *     distributed.  For clustered data, split_pos can fall
 *     OUTSIDE the actual data range, leaving one child empty
 *     and the other still overflowing.  The KD tree (T2)
 *     splits at the data's MEDIAN instead — better for
 *     skewed data but more expensive.
 *   • Points exactly on the split line: convention is "<
 *     goes front, ≥ goes back."  Consistent so queries route
 *     correctly.
 *   • Subtree rooted at a node has a tighter "region" than
 *     just the bounding box implied by ancestor splits.  We
 *     store the rect explicitly per node so query pruning is
 *     tight.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Insert 12 points one at a time (PART 2).  Watch the
 *     splits cascade.  Tree depth should be ⌈log₂(12 / LEAF_CAPACITY)⌉.
 *   • Range query at the end should report only the points
 *     actually inside the query rectangle.  HUD shows count.
 *   • Compare insertion order: random vs. sorted vs. clustered.
 *     Random produces balanced tree; clustered produces
 *     unbalanced (mid-point split fails).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Read algorithms/quadtree.c first if you want the simpler
 *      4-child version; algorithms/kd_tree.c after this for the
 *      "data-point splits" alternative.
 *   2. PART 1 of the file: BSPNode struct + bsp_insert + bsp_query.
 *      THE LIBRARY.  Self-contained, no I/O.  Read AFTER tutorials
 *      T1-T5 below.
 *   3. PART 2: main() with step-by-step insertion + query.  Press
 *      Enter to advance one operation; observe the tree grow.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   BSPNode                    one node of the tree.  Internal or leaf.
 *   .rect                      bounding rectangle this node represents.
 *   .split                     SPLIT_VERTICAL or SPLIT_HORIZONTAL.
 *   .split_pos                 the cut value along the split axis.
 *   .front, .back              children pointers (NULL for leaves).
 *   .data[], .count            leaf data array (only filled for leaves).
 *   LEAF_CAPACITY = 4          when leaf has > this, split.
 *
 * Background you need
 * ───────────────────
 *   - Recursion + binary-tree pointer manipulation.
 *   - 2-D rectangles and bounding-box overlap test.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real BSP-with-arbitrary-planes (Doom-style).  We use the
 *     simpler axis-aligned variant.
 *   - 3-D BSP for visibility.  This is 2-D for spatial point
 *     storage.
 *   - Self-balancing tree theory.  Mid-point splits give
 *     O(log N) for uniform data; we don't enforce balance for
 *     adversarial inputs.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a BSP tree from first principles.
 *
 *   T1  The space-partitioning problem
 *   T2  BSP vs Quadtree vs K-D tree — three answers, same problem
 *   T3  Mid-point split with axis alternation
 *   T4  Bounding-box pruning — the query speedup
 *   T5  Why this isn't real BSP — the axis-aligned simplification
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE SPACE-PARTITIONING PROBLEM
 * ──────────────────────────────────
 * Given N points in 2-D, answer queries efficiently:
 *
 *     "which points lie inside this rectangle R?"
 *
 * Brute force: scan all N points, test each.  O(N) per query.
 * For N = 1M this is too slow for interactive use.
 *
 * SPATIAL DATA STRUCTURE INSIGHT: pre-process the points once
 * into a structure that lets you SKIP big regions during a
 * query.  Pay O(N log N) to build, then O(log N + k) per query
 * where k = number of points returned.
 *
 * Three classical 2-D structures:
 *   - QUADTREE (4 children, both axes cut)
 *   - KD-TREE (2 children, alternates axes, data in nodes)
 *   - BSP TREE (2 children, alternates axes, data in leaves)
 *
 * All three have the SAME ASYMPTOTIC COMPLEXITY for typical
 * queries on well-distributed data: O(log N + k).  They
 * differ in constant factors and code complexity.
 *
 * T2  BSP VS QUADTREE VS K-D TREE
 * ───────────────────────────────
 *
 *   QUADTREE                       K-D TREE
 *
 *   ┌───────┬───────┐              ┌───────┬───────┐
 *   │       │       │              │       │       │
 *   │  NW   │  NE   │              │       │       │
 *   │       │       │              │       │       │
 *   ├───────┼───────┤              ├───────┤       │
 *   │       │       │              │       │       │
 *   │  SW   │  SE   │              │       │       │
 *   │       │       │              │       │       │
 *   └───────┴───────┘              └───────┴───────┘
 *
 *   Splits BOTH axes at once,      Splits ONE axis at the
 *   4 children, generally          DATA POINT's coord, axis
 *   regular grid.                  alternates by depth.
 *
 *
 *   BSP TREE (this file)
 *
 *   ┌───────┬───────┐
 *   │       │       │
 *   │       │       │
 *   │       ├───────┤
 *   │       │       │
 *   │       │       │
 *   │       │       │
 *   └───────┴───────┘
 *
 *   Splits ONE axis at MIDPOINT of region;
 *   alternates by depth; data in leaves.
 *
 *
 * Tradeoffs:
 *
 *   QUADTREE:  4 children → shallower tree, simpler intuition,
 *              wastes memory if data is concentrated in 1 quad.
 *
 *   KD-TREE:   storing data in nodes → no wasted "internal-only"
 *              node memory.  Best for nearest-neighbour queries
 *              (short stack walk).  Sensitive to insertion order.
 *
 *   BSP TREE:  splits at REGION midpoint (not data point) →
 *              insertion-order independent, balanced for uniform
 *              data.  Closest to game-engine BSP idiom.
 *
 * For an animated visualiser of the SAME quadtree+query, see
 * algorithms/quad_tree_helloworld.c.
 *
 * T3  MID-POINT SPLIT WITH AXIS ALTERNATION
 * ─────────────────────────────────────────
 * When a leaf overflows, split it at the MIDPOINT of its
 * region along the next axis:
 *
 *     leaf at depth d has region [x_min, x_max] × [y_min, y_max]
 *     next split axis = (d % 2 == 0) ? VERTICAL : HORIZONTAL
 *     split_pos = (axis == VERTICAL) ? (x_min + x_max) / 2
 *                                    : (y_min + y_max) / 2
 *
 *     create FRONT child:  region with the "less than split" half
 *     create BACK child:   region with the "≥ split" half
 *     redistribute leaf's data points into one child each
 *     convert leaf → internal node (split_pos, front, back)
 *
 * ALTERNATING AXES is what gives the tree balanced spatial
 * subdivision in BOTH x AND y over time.  If we always cut x
 * (no alternation), the tree would have very thin horizontal
 * strips — bad for queries that span x.
 *
 * MID-POINT (vs. median or data-point) keeps the recursion
 * depth O(log N) for UNIFORMLY DISTRIBUTED data even under
 * adversarial insertion order.  For non-uniform data, mid-
 * point can be far from optimal — but the K-D tree's
 * data-point split ALSO has worst cases.  No free lunch.
 *
 * T4  BOUNDING-BOX PRUNING — THE QUERY SPEEDUP
 * ────────────────────────────────────────────
 * Range query: find all points inside rectangle R.
 *
 * Naive: scan every leaf.  Cost O(N).
 * Smart: PRUNE subtrees whose bounding box doesn't overlap R.
 *
 *     query(node, R):
 *       if NOT overlap(node.rect, R): return        ← skip
 *       if node is leaf:
 *         for each point in node.data:
 *           if point in R: report
 *       else:
 *         query(node.front, R)
 *         query(node.back, R)
 *
 * PERFORMANCE GAIN: for a query rectangle that intersects only
 * a small fraction of the tree's region, we descend a SHALLOW
 * subset of the tree — typically O(log N + k) where k is the
 * answer size.
 *
 * Bounding-box overlap is a 4-comparison test (per axis: rect1
 * left ≤ rect2 right AND rect1 right ≥ rect2 left).  Cheap.
 *
 * Same pruning idea is used by:
 *   - Octrees (3-D version)
 *   - R-trees (database spatial indexing)
 *   - Bounding Volume Hierarchies (graphics raytracing)
 *
 * The whole point of spatial trees is the LOGARITHMIC PRUNING.
 * Without it, the tree gives no advantage over a flat array.
 *
 * T5  WHY THIS ISN'T REAL BSP — THE AXIS-ALIGNED SIMPLIFICATION
 * ─────────────────────────────────────────────────────────────
 * Real BSP trees, as used in Doom (1993) and similar engines,
 * are MORE GENERAL than this file:
 *
 *   - Splits are ARBITRARY PLANES (in 3-D) or arbitrary lines
 *     (in 2-D), not just axis-aligned.
 *   - Splits often align with WALL POLYGONS in the scene,
 *     allowing the tree to determine what's in front/behind
 *     each wall for visibility calculations.
 *   - Pre-computed at MAP BUILD TIME, not at insertion (Doom
 *     compiled BSP from level geometry).
 *
 * The full algorithm needed:
 *   - Pick a SPLITTING PLANE (heuristic: choose the plane that
 *     minimises CUT polygons + balances the tree).
 *   - When a polygon STRADDLES the split, SPLIT THE POLYGON
 *     into two sub-polygons (one per side).
 *   - Render front-to-back via in-order traversal — gives
 *     correct occlusion without a Z-buffer.
 *
 * This file simplifies to AXIS-ALIGNED splits + POINT data.
 * Sufficient to teach the SPACE PARTITIONING idea without the
 * polygon-clipping complexity.
 *
 * To go further, see:
 *   - 3-D BSP rendering (Doom rendering technique)
 *   - R-tree (database of bounding boxes)
 *   - Octree (3-D analogue of quadtree)
 *
 * For visibility computation in 2-D, the actual visibility
 * polygon (algorithms/visibility_polygon.c) is more direct.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ─── ANSI color codes (work in any modern terminal without a library) ── */
#define CLR_RESET    "\033[0m"
#define CLR_BOLD     "\033[1m"
#define CLR_DIM      "\033[2m"
#define CLR_YELLOW   "\033[1;33m"  /* data points in the tree        */
#define CLR_GREEN    "\033[1;32m"  /* points found by a query        */
#define CLR_CYAN     "\033[36m"    /* outer boundary border          */
#define CLR_RED      "\033[1;31m"  /* query rectangle                */
#define CLR_MAGENTA  "\033[1;35m"  /* vertical split lines (!)       */
#define CLR_BLUE     "\033[1;34m"  /* horizontal split lines (=)     */
#define CLR_GOLD     "\033[33m"    /* headers and labels             */

/* ================================================================
 * PART 1 — DATA STRUCTURES
 * ================================================================ */

/* ── constants ─────────────────────────────────────────────────── */

/*
 * LEAF_CAPACITY — how many data points a leaf node holds before it
 * must split into two children.  Kept small (4) so splits happen
 * visibly during the 12-point demo.
 */
#define LEAF_CAPACITY  4

/*
 * The demo space is a 56 × 22 integer grid.  Points have (x, y)
 * coordinates within [0, SPACE_W) × [0, SPACE_H).
 * These dimensions fit inside an 80-column terminal once the grid
 * outer border (+2 columns, +2 rows) is added.
 */
#define SPACE_W  56
#define SPACE_H  22

/* Maximum points a single query can return */
#define QUERY_CAP  64

/* ── data types ─────────────────────────────────────────────────── */

/*
 * Point — one indexed data point stored at a BSP leaf.
 *
 * Intent
 *   The BSP tree's payload type. A Point is what the user INSERTS
 *   into the tree (bsp_insert) and what the range query EMITS
 *   (bsp_query). Each leaf owns up to LEAF_CAPACITY of them in its
 *   data[] array; internal nodes own none.
 *
 *   The label field is purely RENDERING — the ASCII visualiser and
 *   tree_dump print it so the human can match insertion order to
 *   spatial position on the grid. The tree algorithm itself never
 *   reads label.
 *
 * Why integer coordinates (not float)
 *   This is a CLI teaching demo on a fixed-resolution character
 *   grid (SPACE_W × SPACE_H). Float would add precision headaches
 *   (e.g. point ON the splitting line, FP equality) without any
 *   visual benefit at 1 pixel = 1 cell. Integer half-open
 *   intervals give an exact partition.
 *
 *   Real game-engine BSP trees (Doom, Quake) use float coordinates
 *   because the geometry isn't grid-aligned — see ref [6] Abrash.
 *
 * Field semantics
 *   x, y  — coordinates in [0, SPACE_W) × [0, SPACE_H)
 *           (sim-only; consumed by rect_contains_point + queries)
 *   label — single ASCII letter for visualisation; ignored by algo
 *
 * References [1] Fuchs/Kedem/Naylor §2 for the "data point as
 *   tree leaf payload" pattern; [5] de Berg §12.1.
 */
typedef struct {
    int  x;       /* horizontal grid coordinate (algorithmic)   */
    int  y;       /* vertical   grid coordinate (algorithmic)   */
    char label;   /* display character; render-only, not read by algorithm */
} Point;

/*
 * Rect — an axis-aligned rectangle covering a HALF-OPEN region.
 *
 * Intent
 *   The "world region" a BSPNode is responsible for. Every BSPNode
 *   stores a Rect (its `boundary`); bsp_insert checks point
 *   membership against it (rect_contains_point), and bsp_query
 *   prunes whole subtrees against it (rect_overlaps_range).
 *
 *   Internal nodes' boundaries TILE EXACTLY: front.boundary ∪
 *   back.boundary = parent.boundary, with empty intersection. This
 *   tile-exactly invariant is what makes the BSP partition a true
 *   PARTITION of space (every point belongs to exactly one leaf).
 *
 * Half-open interval convention
 *   Region = [x, x+w) × [y, y+h)
 *   • LEFT/TOP edges are INCLUDED.
 *   • RIGHT/BOTTOM edges are EXCLUDED.
 *
 *   Why half-open: subdivision becomes EXACT with no overlap and no
 *   gap. Splitting [x, x+w) at midpoint x+w/2 yields:
 *
 *       front = [x,       x + w/2)
 *       back  = [x + w/2, x + w  )
 *
 *   Adjacent at x + w/2; no pixel is in both halves; no pixel is
 *   missed. Closed intervals would force tie-break logic at the
 *   boundary; half-open is the standard CG convention for that
 *   exact reason. Ref [5] de Berg §12.2.
 *
 * Field semantics
 *   x, y — top-left corner (INCLUSIVE end of half-open interval).
 *   w, h — extent; the region's right edge is x+w (EXCLUSIVE).
 *
 * References [1] Fuchs/Kedem/Naylor §3 for BSP-tile invariant;
 *   [5] de Berg §12.2 for half-open partition convention.
 */
typedef struct {
    int x;    /* top-left x, INCLUSIVE  */
    int y;    /* top-left y, INCLUSIVE  */
    int w;    /* width;  right edge = x+w EXCLUSIVE */
    int h;    /* height; bottom edge = y+h EXCLUSIVE*/
} Rect;

/*
 * SplitAxis — which axis an internal node's dividing line runs along.
 *
 * Intent
 *   Tags an internal node with the orientation of its split. The
 *   §5b subdivide() chooses the axis from the node's depth:
 *
 *       even depth → SPLIT_VERTICAL    (cut along x = split_pos)
 *       odd  depth → SPLIT_HORIZONTAL  (cut along y = split_pos)
 *
 *   This depth-alternating pattern is the k-d tree axis cycling
 *   (ref [2] Bentley 1975); it keeps the tree balanced in BOTH
 *   spatial dimensions even if the input data is biased along one
 *   axis. Without alternation, all points clustered in a thin
 *   horizontal strip would force only-x cuts forever, giving an
 *   unbalanced tree.
 *
 *   The enum is consumed by:
 *     - the visualiser (paints the splitting line on the grid)
 *     - tree_dump (prints "VERTICAL split at x=…" labels)
 *     - bsp_query (no — query uses rect_overlaps_range, not the axis)
 *
 * Per-value semantics
 *   SPLIT_VERTICAL   — vertical line at x = split_pos:
 *                       front = LEFT  half (x < split_pos)
 *                       back  = RIGHT half (x ≥ split_pos)
 *   SPLIT_HORIZONTAL — horizontal line at y = split_pos:
 *                       front = TOP    half (y < split_pos)
 *                       back  = BOTTOM half (y ≥ split_pos)
 *
 * References [1] Fuchs/Kedem/Naylor for the BSP partition idea;
 *   [2] Bentley 1975 for the alternating-axis k-d strategy.
 */
typedef enum {
    SPLIT_VERTICAL,    /* dividing line is vertical:   x = split_pos */
    SPLIT_HORIZONTAL,  /* dividing line is horizontal: y = split_pos */
} SplitAxis;

/*
 * BSPNode — one node in the BSP tree.
 *
 * Intent
 *   A node is either a LEAF or an INTERNAL node, distinguished by
 *   whether front/back are non-NULL. The two states use DIFFERENT
 *   subsets of the fields:
 *
 *     Leaf node     — front == NULL, back == NULL.
 *                     data[0..count-1] holds the points; count ≤
 *                     LEAF_CAPACITY. split_pos + split_axis unused.
 *
 *     Internal node — front, back both non-NULL.
 *                     count = 0; data[] logically empty.
 *                     split_axis + split_pos describe the cut.
 *
 *   front/back semantics (set in subdivide → vertical/horizontal_bisect):
 *     SPLIT_VERTICAL   → front = LEFT half,  back = RIGHT  half
 *     SPLIT_HORIZONTAL → front = TOP  half,  back = BOTTOM half
 *
 *   The CHILDREN TILE the PARENT exactly:
 *     front.boundary ∪ back.boundary = node.boundary
 *     front.boundary ∩ back.boundary = ∅
 *   (This is the key invariant that makes route_into_children safe —
 *   any point in the parent's region falls in exactly one child.)
 *
 * Why ONE struct for both states (instead of a tagged union)
 *   The two states share `boundary` and `depth`, which dominate the
 *   memory cost. The state-specific fields (data[] vs split_*) fit
 *   together comfortably — a 5-point LEAF_CAPACITY × Point is about
 *   60 bytes; the split scalars are 8 bytes. The wasted bytes per
 *   internal node aren't worth the API complexity of a tagged union
 *   or two separate types.
 *
 *   This is the SAME pattern as the quadtree in algorithms/quadtree.c
 *   — leaf vs internal distinguished by a null-child predicate, not
 *   by a discriminant.
 *
 * Field locality
 *   ── Shared (used in both states) ──
 *      boundary, depth
 *
 *   ── Leaf-only (count > 0 iff this is a leaf) ──
 *      data[], count
 *
 *   ── Internal-only (set when front/back become non-NULL) ──
 *      split_axis, split_pos, front, back
 *
 *   You can detect the state with EITHER predicate:
 *     • front == NULL   (used by node_is_leaf — canonical)
 *     • count > 0       (true only for non-empty leaves;
 *                        EMPTY leaves and INTERNAL nodes both have
 *                        count == 0, so this is NOT a leaf predicate.)
 *
 * References [1] Fuchs/Kedem/Naylor §3 — BSP tree node model;
 *   [5] de Berg §12.2 — formal partition invariants.
 */
typedef struct BSPNode BSPNode;
struct BSPNode {
    /* ── Shared state (used by leaf and internal alike) ───────── */
    Rect      boundary;             /* region this node is responsible for   */
    int       depth;                /* 0 = root; determines split axis        */

    /* ── Leaf-only state ──────────────────────────────────────── *
     * Used while front == NULL. When the node is promoted to     *
     * internal by subdivide(), count is reset to 0 and the       *
     * points are redistributed to children.                      */
    Point     data[LEAF_CAPACITY];  /* points stored at this leaf            */
    int       count;                /* valid entries in data[]; 0 = empty    */

    /* ── Internal-only state ──────────────────────────────────── *
     * Set by subdivide() when this node is promoted. Front +    *
     * back tile `boundary` exactly along (split_axis, split_pos).*/
    SplitAxis split_axis;           /* orientation of the dividing line       */
    int       split_pos;            /* coordinate of the dividing line        */
    BSPNode  *front;                /* LEFT (V) or TOP    (H); NULL = leaf    */
    BSPNode  *back;                 /* RIGHT(V) or BOTTOM (H); NULL = leaf    */
};

/* ── memory management ──────────────────────────────────────────── */

/* Allocate and initialise a new leaf node covering 'boundary' at 'depth'. */
static BSPNode *node_new(Rect boundary, int depth)
{
    BSPNode *n = malloc(sizeof *n);
    assert(n != NULL);
    n->boundary  = boundary;
    n->count     = 0;
    n->depth     = depth;
    n->split_pos = 0;
    n->split_axis = SPLIT_VERTICAL;
    n->front = n->back = NULL;
    return n;
}

/* Free a node and all of its descendants (post-order traversal). */
void tree_free(BSPNode *node)
{
    if (!node) return;
    tree_free(node->front);
    tree_free(node->back);
    free(node);
}

/* ── rectangle helpers ──────────────────────────────────────────── */

/* True when (px, py) falls inside the half-open rectangle r. */
static bool rect_contains_point(Rect r, int px, int py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/*
 * True when the half-open rectangle 'boundary' overlaps the inclusive
 * search range [x1, x2] × [y1, y2].
 *
 * Non-overlap on x: search starts at or after the right edge (x1 >= bx+bw)
 *                   or ends before the left edge              (x2 <  bx).
 * Same logic on y.  Any other case means the two regions overlap.
 */
static bool rect_overlaps_range(Rect boundary,
                                int x1, int y1, int x2, int y2)
{
    int bx = boundary.x, by = boundary.y;
    int bw = boundary.w, bh = boundary.h;
    return !(x1 >= bx + bw || x2 < bx || y1 >= by + bh || y2 < by);
}

/* ── core operations ────────────────────────────────────────────── */

/* Forward declaration: subdivide() calls bsp_insert() to redistribute. */
static bool bsp_insert(BSPNode *node, Point p);

/*
 * vertical_bisect — cut rect b in two halves along a VERTICAL line.
 *
 *   Splitting line:   x = b.x + b.w/2     (the returned `split_pos`)
 *
 *     before:                          after:
 *     ┌───────────────────┐            ┌─────────┬─────────┐
 *     │                   │            │  front  │   back  │
 *     │         b         │     →      │  (LEFT) │  (RIGHT)│
 *     │                   │            │         │         │
 *     └───────────────────┘            └─────────┴─────────┘
 *
 *   The two halves TILE b exactly: front ∪ back = b, front ∩ back = ∅.
 *   When b.w is odd, the back half is one pixel wider (b.w − w/2 vs
 *   w/2) — the integer-floor convention keeps the partition exact.
 *
 *   Used by subdivide when the parent depth is EVEN (alternating
 *   vertical/horizontal cuts give a k-d tree on x,y axes).
 */
static int vertical_bisect(Rect b, Rect *front, Rect *back)
{
    int half_w = b.w / 2;
    *front = (Rect){ b.x,             b.y, half_w,       b.h };
    *back  = (Rect){ b.x + half_w,    b.y, b.w - half_w, b.h };
    return b.x + half_w;                  /* x-coordinate of the cut */
}

/*
 * horizontal_bisect — cut rect b in two halves along a HORIZONTAL line.
 *
 *   Splitting line:   y = b.y + b.h/2     (the returned `split_pos`)
 *
 *     before:                          after:
 *     ┌───────────┐                    ┌───────────┐
 *     │           │                    │   front   │ (TOP)
 *     │     b     │             →      ├───────────┤
 *     │           │                    │   back    │ (BOTTOM)
 *     └───────────┘                    └───────────┘
 *
 *   Used by subdivide when the parent depth is ODD.
 */
static int horizontal_bisect(Rect b, Rect *front, Rect *back)
{
    int half_h = b.h / 2;
    *front = (Rect){ b.x, b.y,          b.w, half_h       };
    *back  = (Rect){ b.x, b.y + half_h, b.w, b.h - half_h };
    return b.y + half_h;                  /* y-coordinate of the cut */
}

/*
 * redistribute_points_into_children — move all points from a freshly
 * split node into its two children via bsp_insert().
 *
 *   After bisection, the parent node is no longer a leaf: it owns
 *   front + back children, and its own data[] is logically empty
 *   (count = 0). The points that USED to live in data[] must be
 *   re-routed into whichever child contains them.
 *
 *   Snapshot first: bsp_insert() will mutate node->count when it
 *   recurses, so we copy data[] to a local buffer before zeroing the
 *   count. Then we feed each point through bsp_insert(node->front);
 *   if that rejects (point is on the other side of the splitting
 *   line), bsp_insert(node->back) takes it. Exactly one accepts,
 *   because front + back tile the parent exactly.
 */
static void redistribute_points_into_children(BSPNode *node)
{
    int   old_count = node->count;
    Point displaced[LEAF_CAPACITY];
    memcpy(displaced, node->data, (size_t)old_count * sizeof(Point));
    node->count = 0;                      /* this node is now internal */

    for (int i = 0; i < old_count; i++) {
        if (!bsp_insert(node->front, displaced[i]))
            bsp_insert(node->back, displaced[i]);
    }
}

/*
 * subdivide — promote a full leaf into an internal node with two
 * children and re-route its points.
 *
 *   Pseudocode:
 *     1. Choose the cut axis from this node's depth:
 *          even depth → vertical_bisect   (cut along an x = const line)
 *          odd  depth → horizontal_bisect (cut along a y = const line)
 *        Alternating x / y at successive depths is the k-d tree axis
 *        cycling pattern; cycling gives balanced partitioning when the
 *        point distribution is unknown.
 *     2. Create child nodes for front + back halves.
 *     3. redistribute_points_into_children — re-insert the leaf's
 *        former points into whichever child now owns each point.
 *
 *   After this, node->front and node->back are non-NULL and node->
 *   count is 0 — it is now strictly an INTERNAL node in the BSP tree.
 *
 *   Reference: Bentley (1975), "Multidimensional binary search trees
 *   used for associative searching", CACM 18(9) — original k-d tree
 *   paper that motivates the alternating-axis split.
 */
static void subdivide(BSPNode *node)
{
    Rect b      = node->boundary;
    int  next_d = node->depth + 1;

    /* (1) choose axis from depth + cut into two halves */
    Rect front_rect, back_rect;
    if (node->depth % 2 == 0) {
        node->split_axis = SPLIT_VERTICAL;
        node->split_pos  = vertical_bisect  (b, &front_rect, &back_rect);
    } else {
        node->split_axis = SPLIT_HORIZONTAL;
        node->split_pos  = horizontal_bisect(b, &front_rect, &back_rect);
    }

    /* (2) instantiate children */
    node->front = node_new(front_rect, next_d);
    node->back  = node_new(back_rect,  next_d);

    /* (3) re-route the leaf's old points into the children */
    redistribute_points_into_children(node);
}

/*
 * node_is_leaf — true if this node has no children.
 *
 *   In a BSP tree, "leaf" and "internal" are mutually exclusive:
 *   a leaf has data[] but no children; an internal node has both
 *   children but logically empty data[] (count = 0). The presence
 *   of node->front is the canonical leaf/internal predicate.
 */
static inline bool node_is_leaf(const BSPNode *node)
{
    return node->front == NULL;
}

/* leaf_has_room — true if a leaf node can accept one more point. */
static inline bool leaf_has_room(const BSPNode *node)
{
    return node->count < LEAF_CAPACITY;
}

/* store_in_leaf — append p to the node's data array (precondition: room). */
static inline void store_in_leaf(BSPNode *node, Point p)
{
    node->data[node->count++] = p;
}

/*
 * route_into_children — send p into whichever child owns its region.
 *
 *   front + back tile the parent boundary exactly, so EXACTLY ONE
 *   child contains p — but we don't know which side of the splitting
 *   line p falls on without recomputing it. Easiest: try front first;
 *   if it rejects (because rect_contains_point returns false), try
 *   back. One of the two MUST accept (assuming p was inside the
 *   parent's boundary, which the caller has already verified).
 */
static inline bool route_into_children(BSPNode *node, Point p)
{
    if (bsp_insert(node->front, p)) return true;
    return bsp_insert(node->back, p);
}

/*
 * bsp_insert — add point p into the subtree rooted at 'node'.
 *
 *   Pseudocode:
 *     1. if p OUTSIDE this node's boundary → reject (caller will
 *        try the sibling subtree, which holds the other half).
 *     2. if this is a LEAF with room → store_in_leaf and accept.
 *     3. if this is a LEAF without room → subdivide (now internal),
 *        then fall through.
 *     4. route_into_children → recurse into front or back.
 *
 *   Step 3 is the key invariant: a leaf NEVER stays above LEAF_CAPACITY.
 *   When it would overflow, the leaf is promoted to an internal node
 *   before any insertion proceeds. This bounds the worst-case node
 *   data[] size and keeps the average leaf occupancy near LEAF_CAPACITY/2.
 *
 *   Returns:
 *     true  — point was accepted somewhere in this subtree.
 *     false — point was outside this node's boundary (caller routes
 *             to sibling). False should never propagate up to the
 *             root for a point inside the world bounds.
 */
bool bsp_insert(BSPNode *node, Point p)
{
    /* (1) boundary test — reject if outside */
    if (!rect_contains_point(node->boundary, p.x, p.y))
        return false;

    /* (2) leaf with room — store and done */
    if (node_is_leaf(node) && leaf_has_room(node)) {
        store_in_leaf(node, p);
        return true;
    }

    /* (3) leaf without room — promote to internal, then fall through */
    if (node_is_leaf(node))
        subdivide(node);

    /* (4) internal — route into the child that owns p's region */
    return route_into_children(node, p);
}

/*
 * point_in_query_range — inclusive AABB containment test.
 *
 *   true iff (p.x, p.y) lies in [x1, x2] × [y1, y2] (closed interval).
 *
 *   This is the per-point filter inside the query — we already know
 *   the LEAF's boundary overlaps the query range (otherwise the
 *   subtree would have been pruned), but individual points within
 *   that leaf may still be outside the search rectangle.
 */
static inline bool point_in_query_range(Point p,
                                        int x1, int y1, int x2, int y2)
{
    return p.x >= x1 && p.x <= x2 && p.y >= y1 && p.y <= y2;
}

/*
 * collect_points_at_node — emit this node's data[] points that fall
 * within the query rectangle, stopping if results[] fills up.
 *
 *   Internal nodes have count = 0, so this loop is a no-op for them
 *   — the cost is paid only at leaf nodes. The capacity guard inside
 *   the loop lets the caller cap result count without needing a
 *   separate post-filter pass.
 */
static inline void collect_points_at_node(const BSPNode *node,
                                          int x1, int y1, int x2, int y2,
                                          Point *results, int *count,
                                          int capacity)
{
    for (int i = 0; i < node->count && *count < capacity; i++) {
        if (point_in_query_range(node->data[i], x1, y1, x2, y2))
            results[(*count)++] = node->data[i];
    }
}

/*
 * bsp_query — find all points inside the inclusive rectangle
 *             [x1, x2] × [y1, y2] and append them to results[].
 *
 *   Pseudocode:
 *     1. EARLY OUT if node is null or results[] is full.
 *     2. AABB PRUNING: if this node's boundary doesn't overlap the
 *        query range, the entire subtree can't contain a match —
 *        skip both children.
 *     3. collect_points_at_node  (no-op on internal nodes)
 *     4. recurse into front, then back.
 *
 *   AABB pruning is the algorithmic core. Without it the query would
 *   visit every node — O(N). With it, expected complexity is
 *   O(log N + k) where k is the number of points reported, the same
 *   asymptotic guarantee as a quadtree. The CONSTANT FACTOR here is
 *   smaller than a quadtree's because each node has only TWO children
 *   instead of four, so fewer subtree pointers are dereferenced per
 *   level of descent. Refs: Bentley 1975 §2 (k-d tree analysis).
 */
void bsp_query(BSPNode *node,
               int x1, int y1, int x2, int y2,
               Point *results, int *count, int capacity)
{
    /* (1) early-out gates */
    if (!node || *count >= capacity) return;

    /* (2) AABB pruning: skip entire subtree if no overlap */
    if (!rect_overlaps_range(node->boundary, x1, y1, x2, y2)) return;

    /* (3) emit this node's matching points (no-op on internals) */
    collect_points_at_node(node, x1, y1, x2, y2, results, count, capacity);

    /* (4) recurse into both children */
    bsp_query(node->front, x1, y1, x2, y2, results, count, capacity);
    bsp_query(node->back,  x1, y1, x2, y2, results, count, capacity);
}

/* ── inspection helpers ─────────────────────────────────────────── */

/* Count total nodes in the tree (internal + leaf). */
int tree_node_count(BSPNode *node)
{
    if (!node) return 0;
    return 1 + tree_node_count(node->front) + tree_node_count(node->back);
}

/* Count total data points across all leaf nodes. */
int tree_point_count(BSPNode *node)
{
    if (!node) return 0;
    return node->count
         + tree_point_count(node->front)
         + tree_point_count(node->back);
}

/* Return the deepest leaf depth (root = depth 0). */
int tree_depth(BSPNode *node)
{
    if (!node)        return -1;
    if (!node->front) return node->depth;   /* leaf */
    int df = tree_depth(node->front);
    int db = tree_depth(node->back);
    return df > db ? df : db;
}

/*
 * tree_dump — print an indented text representation of the tree.
 *
 * Example output:
 *   root [0,0 56×22] — VERTICAL split at x=28
 *     front [0,0 28×22] — HORIZONTAL split at y=11
 *       front [0,0 28×11] — 3 pts: A(7,5)  F(21,3)  H(14,2)
 *       back  [0,11 28×11] — 4 pts: C(7,16)  E(14,11)  G(7,19)  I(21,19)
 *     back  [28,0 28×22] — HORIZONTAL split at y=11
 *       front [28,0 28×11] — 3 pts: B(42,5)  J(35,3)  K(49,8)
 *       back  [28,11 28×11] — 2 pts: D(42,16)  L(35,19)
 */
void tree_dump(BSPNode *node, int depth, const char *label)
{
    if (!node) return;

    for (int i = 0; i < depth * 2; i++) putchar(' ');

    printf(CLR_DIM "%s " CLR_RESET, label);
    printf("[%d,%d %d\xc3\x97%d]",          /* × = UTF-8 U+00D7        */
           node->boundary.x, node->boundary.y,
           node->boundary.w,  node->boundary.h);

    if (node->front) {
        if (node->split_axis == SPLIT_VERTICAL)
            printf(" — " CLR_MAGENTA "VERTICAL" CLR_RESET
                   " split at x=%d\n", node->split_pos);
        else
            printf(" — " CLR_BLUE "HORIZONTAL" CLR_RESET
                   " split at y=%d\n", node->split_pos);
    } else if (node->count == 0) {
        printf(" — empty\n");
    } else {
        printf(" — %d pt%s: ", node->count, node->count == 1 ? "" : "s");
        for (int i = 0; i < node->count; i++) {
            printf(CLR_YELLOW "%c" CLR_RESET "(%d,%d)  ",
                   node->data[i].label, node->data[i].x, node->data[i].y);
        }
        putchar('\n');
    }

    tree_dump(node->front, depth + 1, "front");
    tree_dump(node->back,  depth + 1, "back ");
}

/* ── ASCII grid visualizer ──────────────────────────────────────── */

/*
 * The visualizer renders the BSP tree into a SPACE_W × SPACE_H
 * character grid, then prints it with ANSI colors.
 *
 * Characters used:
 *   ' '           empty space
 *   + - |         outer boundary border (cyan)
 *   !             vertical split line (magenta)
 *   =             horizontal split line (blue)
 *   A..Z          data point label (yellow)
 *   *             data point found by a query (green)
 *   [ ] ~         query rectangle (red)
 *
 * Split lines use distinct characters (! and =) so they are visually
 * distinct from the outer border (- and |) even without color.
 */

static char g_grid[SPACE_H][SPACE_W + 1];   /* +1 for null terminator */

static void grid_clear(void)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(g_grid[r], ' ', SPACE_W);
        g_grid[r][SPACE_W] = '\0';
    }
}

/* Place a character at (x, y), clipped to the grid bounds. */
static void grid_put(int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        g_grid[y][x] = ch;
}

/* Draw the outer bounding box of 'r' using + - | characters. */
static void grid_draw_border(Rect r)
{
    int x0 = r.x,           y0 = r.y;
    int x1 = r.x + r.w - 1, y1 = r.y + r.h - 1;

    for (int x = x0; x <= x1; x++) {
        grid_put(x, y0, '-');
        grid_put(x, y1, '-');
    }
    for (int y = y0; y <= y1; y++) {
        grid_put(x0, y, '|');
        grid_put(x1, y, '|');
    }
    grid_put(x0, y0, '+');
    grid_put(x1, y0, '+');
    grid_put(x0, y1, '+');
    grid_put(x1, y1, '+');
}

/* Draw the query rectangle using [ ] ~ to distinguish from tree borders. */
static void grid_draw_query_rect(int x1, int y1, int x2, int y2)
{
    for (int x = x1; x <= x2; x++) {
        grid_put(x, y1, '~');
        grid_put(x, y2, '~');
    }
    for (int y = y1; y <= y2; y++) {
        grid_put(x1, y, '[');
        grid_put(x2, y, ']');
    }
    grid_put(x1, y1, '[');
    grid_put(x2, y1, ']');
    grid_put(x1, y2, '[');
    grid_put(x2, y2, ']');
}

/*
 * Recursively render BSP split lines, then leaf points.
 *
 *   Internal node → draw the split line (! or =) within its boundary.
 *   Leaf node     → draw each stored point, marking found ones as '*'.
 *
 * The outer boundary box is drawn separately by show_tree() so it
 * always appears on top of split-line endpoints.
 */
static void grid_render_bsp(BSPNode *node, Point *found, int found_count)
{
    if (!node) return;

    if (node->front) {
        /* ── Internal node: draw the split line ──────────────────── */
        if (node->split_axis == SPLIT_VERTICAL) {
            /* '!' runs vertically at x = split_pos */
            int x = node->split_pos;
            for (int y = node->boundary.y;
                     y < node->boundary.y + node->boundary.h; y++)
                grid_put(x, y, '!');
        } else {
            /* '=' runs horizontally at y = split_pos */
            int y = node->split_pos;
            for (int x = node->boundary.x;
                     x < node->boundary.x + node->boundary.w; x++)
                grid_put(x, y, '=');
        }
    } else {
        /* ── Leaf node: draw stored points ───────────────────────── */
        for (int i = 0; i < node->count; i++) {
            int  px       = node->data[i].x;
            int  py       = node->data[i].y;
            bool is_found = false;
            for (int f = 0; f < found_count; f++)
                if (found[f].x == px && found[f].y == py) {
                    is_found = true;
                    break;
                }
            grid_put(px, py, is_found ? '*' : node->data[i].label);
        }
    }

    grid_render_bsp(node->front, found, found_count);
    grid_render_bsp(node->back,  found, found_count);
}

/* Print the grid with a surrounding box and ANSI colors per character. */
static void grid_print(void)
{
    printf(CLR_CYAN "+");
    for (int c = 0; c < SPACE_W; c++) putchar('-');
    printf("+" CLR_RESET "\n");

    for (int r = 0; r < SPACE_H; r++) {
        printf(CLR_CYAN "|" CLR_RESET);
        for (int c = 0; c < SPACE_W; c++) {
            char ch = g_grid[r][c];
            if (ch >= 'A' && ch <= 'Z') {
                printf(CLR_YELLOW  "%c" CLR_RESET, ch);  /* data point      */
            } else if (ch == '*') {
                printf(CLR_GREEN   "%c" CLR_RESET, ch);  /* found point     */
            } else if (ch == '[' || ch == ']' || ch == '~') {
                printf(CLR_RED     "%c" CLR_RESET, ch);  /* query rect      */
            } else if (ch == '!') {
                printf(CLR_MAGENTA "%c" CLR_RESET, ch);  /* vertical split  */
            } else if (ch == '=') {
                printf(CLR_BLUE    "%c" CLR_RESET, ch);  /* horizontal split*/
            } else if (ch == '+' || ch == '-' || ch == '|') {
                printf(CLR_CYAN    "%c" CLR_RESET, ch);  /* outer border    */
            } else {
                putchar(ch);
            }
        }
        printf(CLR_CYAN "|" CLR_RESET "\n");
    }

    printf(CLR_CYAN "+");
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
 * show_tree — render the current tree state to the grid and print it.
 * The outer boundary box is drawn after the split lines so it is
 * always clean at the edges.
 */
static void show_tree(const char *title, BSPNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_bsp(root, NULL, 0);
    grid_draw_border(root->boundary);   /* outer box drawn last */
    grid_print();
    printf("  nodes=%d  points=%d  depth=%d\n",
           tree_node_count(root),
           tree_point_count(root),
           tree_depth(root));
}

/*
 * show_query — render the tree with a query rectangle overlay and
 *              highlight found points.  Print found labels.
 */
static void show_query(const char *title, BSPNode *root,
                       int x1, int y1, int x2, int y2,
                       Point *found, int found_count)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_bsp(root, found, found_count);
    grid_draw_border(root->boundary);
    grid_draw_query_rect(x1, y1, x2, y2);
    grid_print();

    printf("  Query rect: [%d,%d] to [%d,%d]   Found %d point%s:",
           x1, y1, x2, y2, found_count, found_count == 1 ? "" : "s");
    for (int i = 0; i < found_count; i++)
        printf("  " CLR_GREEN "%c(%d,%d)" CLR_RESET,
               found[i].label, found[i].x, found[i].y);
    putchar('\n');
}

/* ── main: the walkthrough ──────────────────────────────────────── */

int main(void)
{
    /* ── introduction ─────────────────────────────────────────── */

    printf("\n");
    print_separator();
    printf(CLR_BOLD "  BSP TREE — Binary Space Partition walkthrough\n" CLR_RESET);
    print_separator();
    printf(
        "\n"
        "  A BSP tree splits a rectangular region with a single\n"
        "  axis-aligned line, creating two children:\n"
        "\n"
        "    front — left half  (VERTICAL)   or top    half (HORIZONTAL)\n"
        "    back  — right half (VERTICAL)   or bottom half (HORIZONTAL)\n"
        "\n"
        "  The axis alternates with depth:\n"
        "    depth 0 → " CLR_MAGENTA "VERTICAL" CLR_RESET
        "   (split left/right)   shown as " CLR_MAGENTA "!" CLR_RESET "\n"
        "    depth 1 → " CLR_BLUE "HORIZONTAL" CLR_RESET
        " (split top/bottom)  shown as " CLR_BLUE "=" CLR_RESET "\n"
        "    depth 2 → VERTICAL again, and so on.\n"
        "\n"
        "  vs. Quadtree: BSP uses ONE split line → 2 children.\n"
        "                Quadtree uses a cross   → 4 children.\n"
        "\n"
        "  This demo inserts 12 labelled points into a %d×%d grid.\n"
        "\n"
        "  Legend:  " CLR_CYAN "border" CLR_RESET
                  "  " CLR_MAGENTA "! V-split" CLR_RESET
                  "  " CLR_BLUE "= H-split" CLR_RESET
                  "  " CLR_YELLOW "A" CLR_RESET " point"
                  "  " CLR_RED "[ ~ ]" CLR_RESET " query"
                  "  " CLR_GREEN "*" CLR_RESET " found\n",
        SPACE_W, SPACE_H
    );
    press_enter();

    /* ── create the tree ──────────────────────────────────────── */

    Rect full_space = { .x = 0, .y = 0, .w = SPACE_W, .h = SPACE_H };
    BSPNode *root   = node_new(full_space, 0);

    show_tree("Step 1 — empty tree  (capacity = 4 pts per leaf)", root);
    printf("  Root is one leaf covering the whole %d×%d space.\n"
           "  depth=0 → next split will be VERTICAL at x=%d.\n",
           SPACE_W, SPACE_H, SPACE_W / 2);
    press_enter();

    /* ── insert four points: root fills up but does not split ─── */

    print_separator();
    printf(CLR_BOLD "  Step 2 — insert A B C D\n" CLR_RESET);
    printf(
        "  Four points spread across both halves.\n"
        "  All go into the single root leaf.  No split yet.\n"
    );
    press_enter();

    bsp_insert(root, (Point){  7,  5, 'A'});   /* left half  */
    bsp_insert(root, (Point){ 42,  5, 'B'});   /* right half */
    bsp_insert(root, (Point){  7, 16, 'C'});   /* left half  */
    bsp_insert(root, (Point){ 42, 16, 'D'});   /* right half */
    show_tree("  After inserting A B C D", root);
    printf("  Root holds %d/%d points — exactly at capacity.\n",
           root->count, LEAF_CAPACITY);
    press_enter();

    /* ── fifth point triggers the first split ────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 3 — insert E(14,11) — triggers ROOT VERTICAL SPLIT\n"
           CLR_RESET);
    printf(
        "\n"
        "  Root is full.  bsp_insert() calls subdivide():\n"
        "\n"
        "    depth=0 → VERTICAL split at x=%d\n"
        "\n"
        "    front [0,0  28×22]  ← x < 28  gets: A(7,5)  C(7,16)  E(14,11)\n"
        "    back  [28,0 28×22]  ← x ≥ 28  gets: B(42,5) D(42,16)\n"
        "\n"
        "  The " CLR_MAGENTA "!" CLR_RESET " column at x=28 divides the space.\n",
        SPACE_W / 2
    );
    press_enter();

    bsp_insert(root, (Point){ 14, 11, 'E'});
    show_tree("  After inserting E — root is now an internal node", root);
    printf("  Root: VERTICAL internal.  "
           "front=[A C E]  back=[B D]\n");
    press_enter();

    /* ── fill front to capacity ──────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 4 — insert F(21,3) — front reaches capacity\n"
           CLR_RESET);
    printf("  F lands in front [0,0 28×22] (x=21 < 28).\n"
           "  front now holds [A C E F] = %d/%d — at capacity.\n"
           "  One more point landing here will trigger a split.\n",
           LEAF_CAPACITY, LEAF_CAPACITY);
    press_enter();

    bsp_insert(root, (Point){ 21,  3, 'F'});
    show_tree("  After inserting F — front at capacity", root);
    press_enter();

    /* ── front splits horizontally ───────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 5 — insert G(7,19) — triggers FRONT HORIZONTAL SPLIT\n"
           CLR_RESET);
    printf(
        "\n"
        "  G lands in front, which is full.  subdivide() runs on front:\n"
        "\n"
        "    depth=1 → HORIZONTAL split at y=%d\n"
        "\n"
        "    front.front [0,0  28×11]  ← y < 11  gets: A(7,5)   F(21,3)\n"
        "    front.back  [0,11 28×11]  ← y ≥ 11  gets: C(7,16)  E(14,11)\n"
        "\n"
        "  Then G(7,19) lands in front.back (y=19 ≥ 11).\n"
        "  The " CLR_BLUE "=" CLR_RESET
        " row at y=11 now cuts the left half top/bottom.\n",
        SPACE_H / 2
    );
    press_enter();

    bsp_insert(root, (Point){  7, 19, 'G'});
    show_tree("  After inserting G — front has split", root);
    printf("  front.front=[A F]  front.back=[C E G]\n");
    press_enter();

    /* ── fill remaining regions ──────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 6 — insert H I J K L\n" CLR_RESET);
    printf(
        "  H(14,2)  → front.front  (x<28, y<11)  now 3 pts\n"
        "  I(21,19) → front.back   (x<28, y≥11)  now 4 pts — at capacity\n"
        "  J(35,3)  → back         (x≥28)         now 3 pts\n"
        "  K(49,8)  → back         (x≥28)         now 4 pts — at capacity\n"
        "  L(35,19) → back is full → BACK SPLITS HORIZONTALLY at y=%d\n"
        "             back.front=[B J K]  back.back=[D] then L → back.back\n",
        SPACE_H / 2
    );
    press_enter();

    bsp_insert(root, (Point){ 14,  2, 'H'});   /* front.front */
    bsp_insert(root, (Point){ 21, 19, 'I'});   /* front.back  */
    bsp_insert(root, (Point){ 35,  3, 'J'});   /* back        */
    bsp_insert(root, (Point){ 49,  8, 'K'});   /* back        */
    bsp_insert(root, (Point){ 35, 19, 'L'});   /* back splits */
    show_tree("  After inserting H I J K L — final tree state", root);
    press_enter();

    /* ── full tree dump ───────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 7 — tree structure (indented text dump)\n" CLR_RESET);
    printf("\n");
    tree_dump(root, 0, "root");
    printf(
        "\n"
        "  Total nodes  : %d  (3 internal + 4 leaf)\n"
        "  Total points : %d\n"
        "  Tree depth   : %d  (root = 0)\n"
        "\n"
        "  Splits performed:\n"
        "    1. root        → " CLR_MAGENTA "VERTICAL"   CLR_RESET " at x=28  (depth 0)\n"
        "    2. front       → " CLR_BLUE    "HORIZONTAL" CLR_RESET " at y=11  (depth 1)\n"
        "    3. back        → " CLR_BLUE    "HORIZONTAL" CLR_RESET " at y=11  (depth 1)\n",
        tree_node_count(root),
        tree_point_count(root),
        tree_depth(root)
    );
    press_enter();

    /* ── range query ──────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Step 8 — range query: find all points in [5,0]..[25,12]\n"
           CLR_RESET);
    printf(
        "\n"
        "  bsp_query() applies AABB pruning at each node:\n"
        "\n"
        "    root [0,0 56×22]   overlaps [5..25, 0..12]? YES → recurse\n"
        "      front [0,0 28×22] overlaps?                YES → recurse\n"
        "        front.front [0,0 28×11]  overlaps?       YES → check A✓ F✓ H✓\n"
        "        front.back  [0,11 28×11] overlaps?       YES → check C✗ E✓ G✗ I✗\n"
        "      back [28,0 28×22] overlaps [5..25]?       " CLR_GREEN "NO → PRUNED" CLR_RESET "\n"
        "\n"
        "  The entire right half (back) is skipped in one test.\n"
        "  This is the O(log N + k) efficiency of the tree.\n"
    );
    press_enter();

    int   qx1 = 5,  qy1 = 0;
    int   qx2 = 25, qy2 = 12;
    Point found[QUERY_CAP];
    int   found_count = 0;
    bsp_query(root, qx1, qy1, qx2, qy2, found, &found_count, QUERY_CAP);
    show_query("  Query result", root, qx1, qy1, qx2, qy2, found, found_count);
    press_enter();

    /* ── cleanup ──────────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Done.\n" CLR_RESET
           "\n"
           "  Summary of operations implemented in this file:\n"
           "\n"
           "    bsp_insert(root, point)             O(log N)\n"
           "    bsp_query(root, x1,y1,x2,y2, ...)  O(log N + k)\n"
           "    subdivide(node)                     O(LEAF_CAPACITY)\n"
           "    tree_node_count / tree_point_count  O(nodes)\n"
           "    tree_depth / tree_dump              O(nodes)\n"
           "    tree_free(root)                     O(nodes)\n"
           "\n"
           "  BSP vs. Quadtree trade-offs:\n"
           "    BSP   — 2 children, rectangular half-spaces, game-dev classic\n"
           "    Quad  — 4 children, square quadrants, better for dense uniform data\n"
           "    Both  — O(log N + k) queries via the same AABB pruning idea\n"
           "\n");

    tree_free(root);
    return 0;
}
