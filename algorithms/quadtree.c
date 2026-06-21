/*
 * algorithms/quadtree.c — quadtree data structure with a step-by-step terminal demo.
 *
 * A quadtree keeps 2-D points organised for fast "what's in this box?" lookups:
 * split a crowded square into four, and split again, until each little square
 * holds only a few points.  PART 1 is the reusable library; PART 2 (main) walks
 * through 12 insertions and a range query, pausing on Enter.  Plain ANSI colour,
 * no ncurses, no libm.
 *
 * Sister files: quad_tree_helloworld.c (animated ncurses version), bsp_tree.c and
 * kd_tree.c (the 2-children-per-split alternatives).
 * Original algorithm: Finkel & Bentley, "Quad trees" (Acta Informatica, 1974).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/quadtree.c -o quadtree
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── ANSI colour codes — raw escape sequences, no library needed ── */
#define CLR_RESET   "\033[0m"
#define CLR_BOLD    "\033[1m"
#define CLR_DIM     "\033[2m"
#define CLR_YELLOW  "\033[1;33m"   /* points in the tree             */
#define CLR_GREEN   "\033[1;32m"   /* points found by a query        */
#define CLR_CYAN    "\033[36m"     /* tree borders                   */
#define CLR_RED     "\033[1;31m"   /* query rectangle                */
#define CLR_GOLD    "\033[33m"     /* headers and labels             */

/* ── PART 1 — the quadtree library: types, geometry, insert/query, visualizer ── */

/* ── constants — tunable sizes for the demo ── */

/* How many points a leaf holds before it has to split into four.
 * Small on purpose so the 12-point demo shows several splits happening. */
#define LEAF_CAPACITY  4

/* Size of the play area, in grid cells.  Points live at integer (x, y) inside
 * it.  56x22 fits an 80-column terminal once the +1-cell border is drawn. */
#define SPACE_W  56
#define SPACE_H  22

/* Most points one query can return; extras past this are silently dropped. */
#define QUERY_CAP  64

/* ── data types — the building blocks of the tree ── */

/*
 * Point — one data point the tree stores.  Just a position plus a name tag.
 *
 * The tree sorts everything by position alone; it never looks at the label.
 * Integer coordinates (not float) because the play area is a character grid:
 * a point maps straight to a grid cell with no rounding, and the boundary
 * comparisons (< and >=) stay exact at the half-open edges.  The label is
 * only there so the demo can say "insert A, then B"; real code would carry
 * whatever payload it likes here.
 *
 *   x      grid column, 0 to SPACE_W-1
 *   y      grid row,    0 to SPACE_H-1   (y grows DOWNWARD, like the screen)
 *   label  display letter 'A'..'Z'; ignored by the tree, used by the drawing
 */
typedef struct {
    int  x;       /* grid column [0, SPACE_W)                              */
    int  y;       /* grid row    [0, SPACE_H)  (y grows DOWN)              */
    char label;   /* display glyph 'A'..'Z' — opaque to the tree routing   */
} Point;

/*
 * Rect — the rectangular region one tree node owns.
 *
 * The region runs from its top-left corner up to BUT NOT INCLUDING the right
 * and bottom edges (half-open: x covers [x, x+w), y covers [y, y+h)).  That
 * "not including" detail matters: when a node splits into four equal pieces,
 * the cell on the dividing line has to belong to exactly one piece.  With the
 * upper edge excluded, the four children tile the parent perfectly — no shared
 * cells, no gaps.  (If both edges were included, the centre row and column
 * would land in two children at once.)  Same idea as Python's a[i:j] slicing.
 *
 * A node's Rect never changes once created; splitting makes four brand-new
 * smaller Rects and leaves the parent's alone.
 *
 *   x, y   top-left corner (included in the region)
 *   w, h   width and height; region is [x, x+w) wide, [y, y+h) tall
 *
 * Width and height are always >= 0.  Splitting halves them with integer
 * division, so a 1-wide leaf would yield 0-wide children — but the capacity
 * check stops a tiny leaf from ever splitting, so that never happens.
 */
typedef struct {
    int x, y;     /* top-left corner, inclusive                            */
    int w, h;     /* width and height; rect covers [x, x+w) × [y, y+h)     */
} Rect;

/*
 * QuadNode — one node of the tree.  It is in one of two states at any moment:
 *
 *   LEAF      — a bucket: holds up to LEAF_CAPACITY points, no children.
 *   INTERNAL  — a fork: has four children (NW/NE/SW/SE), holds no points.
 *
 * How do you tell which? There's no flag — the child pointers tell you.  All
 * four NULL means leaf; all four set means internal.  A leaf flips to internal
 * the moment one too many points lands in it (see subdivide).  That flip is
 * one-way: this code never merges children back into a leaf, and the only
 * cleanup is tree_free freeing the whole tree at once.
 *
 * Points and children share one struct (rather than two separate types) so
 * there's a single allocation path and the simple all-NULL check distinguishes
 * the two states.  The cost is that internal nodes carry an unused data[]
 * buffer; harmless at this demo's scale.
 *
 * The compass names map to where each child sits on screen:
 *   nw = top-left, ne = top-right, sw = bottom-left, se = bottom-right.
 *
 *   boundary     the region this node owns (fixed for the node's whole life)
 *   data[]       its points; only meaningful for a leaf; entries [0,count) valid
 *   count        how many points stored; 0..LEAF_CAPACITY in a leaf, 0 if internal
 *   nw,ne,sw,se  child pointers; all NULL = leaf, all set = internal
 *
 * Guarantees this code maintains: the four child pointers are always all-NULL
 * or all-set together; an internal node has count 0; every point in data[]
 * really sits inside boundary; each child's region is exactly one quadrant of
 * this node's region.
 *
 * This is the "point-region (PR) quadtree" of Finkel & Bentley (1974);
 * Samet (1984) surveys the other flavours that store data differently.
 */
typedef struct QuadNode QuadNode;
struct QuadNode {
    Rect      boundary;              /* region this node owns                */
    Point     data[LEAF_CAPACITY];   /* point storage; leaf-only             */
    int       count;                 /* 0 ≤ count ≤ LEAF_CAPACITY; 0 if internal */
    QuadNode *nw, *ne, *sw, *se;     /* children; NULL ⇔ this is a leaf      */
};

/*
 * QuadrantRects — the four equal pieces a rectangle splits into.
 *
 * When a node splits, it carves its region into four quadrants.  Bundling them
 * in one struct lets the geometry helper return all four at once, and lets the
 * caller write q.nw / q.ne by name instead of juggling an array.  The fields
 * are in the same NW/NE/SW/SE order as QuadNode's children, so the splitting
 * code reads as neat matched pairs.
 *
 *   nw  top-left piece,     ne  top-right piece
 *   sw  bottom-left piece,  se  bottom-right piece
 *
 * The four pieces are all the same size (parent halved each way), don't
 * overlap, and together cover every cell of the parent exactly once.
 */
typedef struct {
    Rect nw;      /* NW (top-left)     quadrant of the parent           */
    Rect ne;      /* NE (top-right)    quadrant of the parent           */
    Rect sw;      /* SW (bottom-left)  quadrant of the parent           */
    Rect se;      /* SE (bottom-right) quadrant of the parent           */
} QuadrantRects;

/*
 * QueryRect — the search box you hand to qt_query: "find every point in here".
 *
 * It's a separate type from Rect on purpose, because the edge rule is the
 * opposite.  A QueryRect INCLUDES all four edges — you give it two opposite
 * corners and everything from corner to corner counts, which is how a person
 * naturally reads "from (x1,y1) to (x2,y2)".  (Rect, used for node regions,
 * excludes its far edges.)  Keeping them as distinct types means the compiler
 * stops you mixing one up for the other.
 *
 *   x1, y1   top-left corner, included
 *   x2, y2   bottom-right corner, included
 *
 * The caller must give them in order (x1 <= x2, y1 <= y2); qt_query won't fix
 * a flipped box.  A box where the corners are equal is fine — it asks about a
 * single cell.
 *
 * The inclusive [a,b] form is the textbook way to state a range query; see
 * de Berg et al. (2008), Computational Geometry, Ch. 5 and 14.
 */
typedef struct {
    int x1;       /* left   edge, INCLUSIVE — 0 ≤ x1 ≤ x2              */
    int y1;       /* top    edge, INCLUSIVE — 0 ≤ y1 ≤ y2              */
    int x2;       /* right  edge, INCLUSIVE                            */
    int y2;       /* bottom edge, INCLUSIVE                            */
} QueryRect;

/*
 * QueryResult — where qt_query drops the points it finds.
 *
 * The caller makes one of these (on the stack, with count zeroed) and passes
 * it in; qt_query fills points[] and bumps count.  No malloc anywhere — the
 * caller owns the storage, the callee just fills it.
 *
 * The buffer is a fixed size (QUERY_CAP).  If a query matches more points than
 * that, the extras are quietly dropped rather than growing the buffer; size
 * QUERY_CAP for your worst case.  Matches come out in tree-walk order
 * (NW, NE, SW, SE), not sorted by distance.
 *
 *   points[]   the matches; only entries [0, count) are filled in
 *   count      how many were found, 0..QUERY_CAP
 *
 * Don't read points[count] and beyond — those slots are uninitialised.
 *
 * Usage:
 *     QueryResult result = { .count = 0 };
 *     qt_query(root, query_rect, &result);
 *     // result.points[0 .. result.count-1] are the hits.
 */
typedef struct {
    Point points[QUERY_CAP];   /* match buffer; entries [0, count) are valid */
    int   count;               /* match count; 0 ≤ count ≤ QUERY_CAP         */
} QueryResult;

/*
 * GridCanvas — a scratch grid of characters the picture is drawn onto.
 *
 * Drawing happens in two phases: the grid_* helpers paint characters into this
 * grid in any order, then grid_print walks the finished grid and prints it with
 * colour.  Separating "lay down characters" from "print with colour" means the
 * painters don't fight over terminal state, and you could swap in a different
 * printer later.  Each show_* call keeps its own canvas on the stack, so there's
 * no shared drawing state to trip over.
 *
 * Stored row-by-row, the way the terminal scans it out.  Each row has one extra
 * column holding a string terminator, so a row can be printed as a plain C
 * string if needed.
 *
 * The characters that appear on it:
 *     ' '       blank
 *     '+'       corner of a node boundary
 *     '-' '|'   horizontal / vertical node boundary edge
 *     'A'..'Z'  a point's label
 *     '*'       a point that matched the query
 *     '[' ']'   query box left/right edge and corners
 *     '~'       query box top/bottom edge
 *
 *   cells[row][col]   the grid; cells[r][SPACE_W] is always the '\0' terminator
 */
typedef struct {
    char cells[SPACE_H][SPACE_W + 1];   /* row-major; cells[r][SPACE_W] is NUL */
} GridCanvas;

/* ── memory management — build and tear down nodes ── */

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

/* ── rectangle / quadrant geometry — the spatial tests ── */

/*
 * Is point (px, py) inside rect r?  (Far edges excluded, per Rect's rule.)
 * qt_insert uses this to pick which child a point belongs in.
 */
static bool rect_contains_point(Rect r, int px, int py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/*
 * Is point p inside the search box q?  (All edges count — it's inclusive.)
 * qt_query tests each candidate point with this before keeping it.
 */
static bool point_in_query_rect(Point p, QueryRect q)
{
    return p.x >= q.x1 && p.x <= q.x2
        && p.y >= q.y1 && p.y <= q.y2;
}

/*
 * Do a node's region and the search box touch at all?
 *
 * This is the trick that makes a quadtree fast.  If a node's region doesn't
 * touch the search box, then nothing inside that node OR any of its children
 * can match — so qt_query skips the whole branch without looking at a single
 * point.  That's why a query only visits a handful of nodes instead of all N.
 *
 * Two boxes miss each other when one sits entirely left/right/above/below the
 * other; the formula just checks those four "completely past the edge" cases
 * and negates them.
 */
static bool rect_overlaps_query(Rect boundary, QueryRect q)
{
    int bx = boundary.x, by = boundary.y;
    int bw = boundary.w, bh = boundary.h;
    return !(q.x1 >= bx + bw || q.x2 < bx
          || q.y1 >= by + bh || q.y2 < by);
}

/*
 * Cut rect r into its four equal quadrants.
 *
 * Find the midpoint, then build the four corner rectangles around it.  Because
 * Rect excludes its far edges, the four pieces tile r perfectly: no overlaps,
 * no gaps.  subdivide() calls this to give each new child its region.
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

/* ── core operations — insert and query ── */

/* qt_insert and subdivide call each other, so qt_insert is declared first. */
static bool qt_insert(QuadNode *node, Point p);

/*
 * After a leaf has just sprouted four children, move its points down into them.
 *
 * Copy the points out first, zero the leaf's count, then re-insert each point;
 * since the node now has children, each point flows into whichever quadrant
 * contains it.  The copy matters because we're emptying the leaf as we go.
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
 * Turn a full leaf into an internal node: make four child leaves, one per
 * quadrant, then push the old points down into them.  The node now holds no
 * points of its own — its children do.
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
 * Drop p into this leaf if it still has a free slot; return false if it's full.
 * A false answer is qt_insert's signal that it's time to subdivide.
 * Assumes node is a leaf.
 */
static bool try_append_to_leaf(QuadNode *node, Point p)
{
    if (node->count >= LEAF_CAPACITY) return false;
    node->data[node->count++] = p;
    return true;
}

/*
 * Hand p down to whichever child's region holds it.
 *
 * Exactly one of the four quadrants contains the point, so we just try each in
 * turn and stop at the first that accepts.  Returning false would mean the
 * point fit this node but none of its quadrants — impossible, so a bug.
 * Assumes node is internal (all four children exist).
 */
static bool route_into_quadrant_child(QuadNode *node, Point p)
{
    QuadNode *children[4] = { node->nw, node->ne, node->sw, node->se };
    for (int c = 0; c < 4; c++)
        if (qt_insert(children[c], p)) return true;
    return false;
}

/*
 * Add point p somewhere in the tree under `node`.
 *
 * If p doesn't fall in this node's region, refuse it.  At a leaf, drop it in if
 * there's room; if the leaf is full, split it and keep going.  At an internal
 * node, pass p down to the right child.  Returns true if p found a home.
 * (It only returns false when p is outside the region — used during the
 * redistribution above, where a point tries each child until one fits.)
 */
bool qt_insert(QuadNode *node, Point p)
{
    if (!rect_contains_point(node->boundary, p.x, p.y)) return false;

    if (!node->nw) {                                /* leaf path */
        if (try_append_to_leaf(node, p))            return true;
        subdivide(node);                            /* full → split, now internal */
    }

    return route_into_quadrant_child(node, p);      /* internal path */
}

/*
 * Find every point inside the search box q and collect them into `out`.
 *
 * Walk the tree, but skip any node whose region doesn't touch the box — that
 * one check throws away whole branches at once, which is the whole point of the
 * structure.  At a node that does touch, test its own points, then recurse into
 * the four children.  Results land in out->points; the caller zeroes out->count
 * first.  If the box matches more than QUERY_CAP points, the rest are dropped
 * (see QueryResult).
 */
void qt_query(QuadNode *node, QueryRect q, QueryResult *out)
{
    if (!node || out->count >= QUERY_CAP)              return;
    if (!rect_overlaps_query(node->boundary, q))       return;   /* whole branch can't match */

    for (int i = 0; i < node->count && out->count < QUERY_CAP; i++)
        if (point_in_query_rect(node->data[i], q))
            out->points[out->count++] = node->data[i];

    qt_query(node->nw, q, out);
    qt_query(node->ne, q, out);
    qt_query(node->sw, q, out);
    qt_query(node->se, q, out);
}

/* ── inspection helpers — count, measure, and print the tree ── */

/* Count every node in the tree, internal and leaf alike. */
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

/* Largest of four ints — used to take the deepest of a node's four children. */
static inline int max4(int a, int b, int c, int d)
{
    int m = a > b ? a : b;
    m     = m > c ? m : c;
    return  m > d ? m : d;
}

/*
 * How many levels deep is the deepest leaf?  Root counts as level 0.
 * Pass current_depth = 0 when calling on the root; each level down adds 1.
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

/* Indent a dump line: two spaces per tree level. */
static void print_indent(int depth)
{
    for (int i = 0; i < depth * 2; i++) putchar(' ');
}

/*
 * Print one node's header: its label and region, e.g. "NW [14,0 14x5]".
 * The "x" is the UTF-8 multiply sign (U+00D7), written as raw bytes so the
 * dump shows a proper "x" rather than the letter x.
 */
static void print_node_header(const QuadNode *node, const char *label)
{
    printf(CLR_DIM "%s " CLR_RESET, label);
    printf("[%d,%d %d\xc3\x97%d]",          /* × = UTF-8 U+00D7  */
           node->boundary.x, node->boundary.y,
           node->boundary.w, node->boundary.h);
}

/*
 * Print a leaf's point list, e.g. " - 2 pts: E(4,3), H(10,2)".
 * The ternary just keeps "1 pt" / "2 pts" grammatically right.
 * Only called for a leaf that actually has points; tree_dump handles the rest.
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
 * Print the whole tree as an indented outline, one line per node, deeper nodes
 * indented further.  Each line says whether the node is internal, an empty
 * leaf, or a leaf with points.  Example:
 *
 *   root [0,0 56x22] - internal
 *     NW [0,0 28x11] - internal
 *       NW [0,0 14x5] - 2 pts: E(4,3), H(10,2)
 *       NE [14,0 14x5] - empty
 *     NE [28,0 28x11] - 3 pts: B(42,5), I(38,3), L(44,8)
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

/* ── ASCII grid visualizer — draw the tree as a picture ── */

/*
 * The picture is built by painting characters into a GridCanvas, then printing
 * it in colour.  Borders are drawn outer-node-first, point labels last, so a
 * label always sits on top of any border line crossing its cell instead of
 * being hidden by it.  See the GridCanvas comment for the character set.
 */

/* Wipe the canvas to all-blank and re-set each row's string terminator. */
static void grid_clear(GridCanvas *gc)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(gc->cells[r], ' ', SPACE_W);
        gc->cells[r][SPACE_W] = '\0';
    }
}

/* Place one character at (x, y), ignoring it if it falls off the canvas. */
static void grid_put(GridCanvas *gc, int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        gc->cells[y][x] = ch;
}

/*
 * Draw a box around rect r: '+' at the corners, '-' top and bottom, '|' on the
 * sides.  Inside is left alone.  Adjacent node borders happen to overlap with
 * the same character, so there's no corner-merging to worry about.
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
 * Draw the search box using a different character set ('~' top/bottom, '[' ']'
 * sides and corners) so it stands out from the tree's '+ - |' borders.
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

/* Is point p one of the query matches?  Used to draw matches as '*'. */
static bool point_is_highlighted(Point p, const QueryResult *highlight)
{
    if (!highlight) return false;
    for (int i = 0; i < highlight->count; i++)
        if (highlight->points[i].x == p.x && highlight->points[i].y == p.y)
            return true;
    return false;
}

/*
 * Stamp each of this node's points onto the canvas — its label normally, or
 * '*' if it matched the query.  Internal nodes have no points, so this does
 * nothing for them.
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
 * Paint the whole tree: each node's border, then its point labels, then its
 * children (so inner subdivision lines land on top of the outer borders).
 * Pass `highlight` to mark its points with '*', or NULL for plain labels.
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

/* Which colour does a given character get?  NULL means print it plain. */
static const char *glyph_color_for(char ch)
{
    if (ch >= 'A' && ch <= 'Z')               return CLR_YELLOW;
    if (ch == '*')                            return CLR_GREEN;
    if (ch == '[' || ch == ']' || ch == '~')  return CLR_RED;
    if (ch == '+' || ch == '-' || ch == '|')  return CLR_CYAN;
    return NULL;     /* plain putchar */
}

/* Print the finished canvas to the terminal, framed and coloured per character. */
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

/* ── PART 2 — the step-by-step demo (main) ── */

/* ── demo helpers — pause prompts and the walkthrough ── */

/* Pause until the user presses Enter, swallowing any stray input first. */
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
    printf(CLR_DIM "------------------------------------------------------------\n"
           CLR_RESET);
}

/* Draw the tree right now, with a title and a node/point/depth summary line. */
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
 * Like show_tree, but also draws the search box over the tree and lists the
 * points it found.  Pass `result` to mark matches with '*'.
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

/* ── main — insert 12 points one stage at a time, then run a query ── */

int main(void)
{
    /* intro screen + legend */

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

    /* start with one empty leaf covering the whole space */

    Rect full_space = { .x = 0, .y = 0, .w = SPACE_W, .h = SPACE_H };
    QuadNode *root  = node_new(full_space);

    show_tree("Step 1 — empty tree  (capacity = 4 pts per leaf)",  root);
    printf("  Root is one leaf covering the whole %d×%d space.\n", SPACE_W, SPACE_H);
    press_enter();

    /* four points fit in the root leaf — fills it up, no split yet */

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

    /* the fifth point overflows the root, forcing the first split */

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

    /* pile points into NW until it too is full */

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

    /* one more point in NW makes a deeper level of grandchildren */

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

    /* scatter the last points across the other quadrants */

    print_separator();
    printf(CLR_BOLD "  Step 6 — insert I J K L to fill NE / SE / SW\n" CLR_RESET);
    press_enter();

    qt_insert(root, (Point){38,  3, 'I'});   /* NE */
    qt_insert(root, (Point){48, 16, 'J'});   /* SE */
    qt_insert(root, (Point){20, 16, 'K'});   /* SW */
    qt_insert(root, (Point){44,  8, 'L'});   /* NE */
    show_tree("  After inserting I J K L — final tree state", root);
    press_enter();

    /* show the finished tree as a text outline */

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

    /* run a box query and watch most of the tree get skipped */

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

    /* recap and free the tree */

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
