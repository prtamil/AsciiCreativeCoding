/*
 * geometry/quadtree.c — Quadtree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1  (lines ~60-310)  — the quadtree library
 *     Data structures, memory management, core operations,
 *     inspection helpers, and an ASCII grid visualizer.
 *     This part has no I/O — it is a reusable module.
 *
 *   PART 2  (lines ~310-end) — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every subdivision live,
 *     then demonstrates a range query.  Press Enter each step.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/quadtree.c -o quadtree
 *
 * Run:
 *   ./quadtree
 */

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
 * Point — a data point stored inside the quadtree.
 *   x, y   — integer coordinates in [0, SPACE_W) × [0, SPACE_H)
 *   label  — single uppercase letter for display (e.g. 'A')
 */
typedef struct {
    int  x, y;
    char label;
} Point;

/*
 * Rect — an axis-aligned rectangle using half-open intervals.
 *   x, y   — top-left corner (inclusive)
 *   w, h   — width and height
 *   Region covered: x in [x, x+w)  and  y in [y, y+h)
 *
 * Half-open makes subdivision exact: the two halves [x, x+w/2) and
 * [x+w/2, x+w) share no pixels and together cover the whole region.
 */
typedef struct {
    int x, y;
    int w, h;
} Rect;

/*
 * QuadNode — one node in the quadtree.
 *
 * A node is either a LEAF or an INTERNAL node:
 *
 *   Leaf node    — nw/ne/sw/se are all NULL.
 *                  Holds up to LEAF_CAPACITY points in data[].
 *
 *   Internal node — has four children (nw, ne, sw, se).
 *                   data[] and count are unused (always 0).
 *                   Exists only to route insertions and queries.
 *
 *   nw = north-west (top-left)     ne = north-east (top-right)
 *   sw = south-west (bottom-left)  se = south-east (bottom-right)
 */
typedef struct QuadNode QuadNode;
struct QuadNode {
    Rect      boundary;              /* region this node is responsible for  */
    Point     data[LEAF_CAPACITY];   /* points stored here (leaf only)       */
    int       count;                 /* number of points currently stored    */
    QuadNode *nw, *ne, *sw, *se;    /* children; NULL means this is a leaf  */
};

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

/* ── rectangle helpers ──────────────────────────────────────────── */

/* True when (px, py) falls inside the half-open rectangle r. */
static bool rect_contains_point(Rect r, int px, int py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/*
 * True when the half-open rectangle 'node_boundary' overlaps the
 * inclusive search range [x1, x2] × [y1, y2].
 *
 * Non-overlap on the x-axis happens when:
 *   search starts at or after the boundary's right edge  (x1 >= bx+bw)
 *   search ends   before  the boundary's left  edge      (x2 <  bx)
 * Same logic on y.  Any other case = overlap.
 */
static bool rect_overlaps_range(Rect boundary,
                                int x1, int y1, int x2, int y2)
{
    int bx = boundary.x, by = boundary.y;
    int bw = boundary.w, bh = boundary.h;
    return !(x1 >= bx + bw || x2 < bx || y1 >= by + bh || y2 < by);
}

/* ── core operations ────────────────────────────────────────────── */

/* Forward declaration: subdivide() needs qt_insert() to redistribute. */
static bool qt_insert(QuadNode *node, Point p);

/*
 * subdivide — split a full leaf into four equal child nodes.
 *
 * The node's existing points are redistributed into the children.
 * After subdivide(), 'node' becomes an internal node: count = 0,
 * nw/ne/sw/se are all non-NULL.
 *
 *   Before:  [node: LEAF_CAPACITY pts]
 *
 *   After:   [node: internal, 0 pts]
 *              /         |         |         \
 *           [nw]       [ne]      [sw]       [se]
 *        (pts redistributed)
 */
static void subdivide(QuadNode *node)
{
    int half_w = node->boundary.w / 2;
    int half_h = node->boundary.h / 2;
    int left   = node->boundary.x;
    int top    = node->boundary.y;
    int mid_x  = left + half_w;
    int mid_y  = top  + half_h;

    node->nw = node_new((Rect){ left,  top,   half_w, half_h });
    node->ne = node_new((Rect){ mid_x, top,   half_w, half_h });
    node->sw = node_new((Rect){ left,  mid_y, half_w, half_h });
    node->se = node_new((Rect){ mid_x, mid_y, half_w, half_h });

    /*
     * Redistribute this leaf's points into the new children.
     * We snapshot them first because qt_insert() modifies node->count.
     */
    int   old_count              = node->count;
    Point displaced[LEAF_CAPACITY];
    memcpy(displaced, node->data, (size_t)old_count * sizeof(Point));
    node->count = 0;

    for (int i = 0; i < old_count; i++) {
        QuadNode *children[4] = { node->nw, node->ne, node->sw, node->se };
        for (int c = 0; c < 4; c++)
            if (qt_insert(children[c], displaced[i])) break;
    }
}

/*
 * qt_insert — add point p into the subtree rooted at 'node'.
 *
 * Returns true if the point was accepted.
 * Returns false if it lies outside this node's boundary (used
 * internally during redistribution after subdivision).
 */
bool qt_insert(QuadNode *node, Point p)
{
    if (!rect_contains_point(node->boundary, p.x, p.y))
        return false;   /* point is outside this node's region           */

    if (!node->nw) {
        /* ── Leaf node ─────────────────────────────────────────── */
        if (node->count < LEAF_CAPACITY) {
            /* Room available: store the point here */
            node->data[node->count++] = p;
            return true;
        }
        /* Leaf is full: split into four children, then re-route */
        subdivide(node);
        /* node->nw is now non-NULL; fall through to internal-node path */
    }

    /* ── Internal node ──────────────────────────────────────────── */
    /* Delegate to whichever child's boundary contains the point */
    QuadNode *children[4] = { node->nw, node->ne, node->sw, node->se };
    for (int c = 0; c < 4; c++)
        if (qt_insert(children[c], p)) return true;

    return false;   /* should not happen if boundary covers the point   */
}

/*
 * qt_query — find all points inside the inclusive rectangle
 *            [x1, x2] × [y1, y2] and append them to results[].
 *
 * The key efficiency step is the overlap test at the top of each call:
 * if the search range does NOT overlap this node's boundary, the
 * entire subtree is skipped (pruned).  Only overlapping nodes are
 * visited, giving O(log N + k) instead of a full O(N) scan.
 *
 * Parameters:
 *   results  — caller-allocated array to receive found points
 *   count    — pointer to the running count (caller initialises to 0)
 *   capacity — maximum number of results to store
 */
void qt_query(QuadNode *node,
              int x1, int y1, int x2, int y2,
              Point *results, int *count, int capacity)
{
    if (!node || *count >= capacity) return;

    /* ── Pruning step ────────────────────────────────────────────── */
    /* If the search range doesn't touch this node's region at all,   */
    /* skip this node AND every descendant — they can't hold matches. */
    if (!rect_overlaps_range(node->boundary, x1, y1, x2, y2)) return;

    /* ── Point check ─────────────────────────────────────────────── */
    /* The range overlaps this node.  Check each point it holds.      */
    for (int i = 0; i < node->count && *count < capacity; i++) {
        int px = node->data[i].x, py = node->data[i].y;
        if (px >= x1 && px <= x2 && py >= y1 && py <= y2)
            results[(*count)++] = node->data[i];
    }

    /* ── Recurse ─────────────────────────────────────────────────── */
    /* Each child will prune itself independently if needed.          */
    qt_query(node->nw, x1, y1, x2, y2, results, count, capacity);
    qt_query(node->ne, x1, y1, x2, y2, results, count, capacity);
    qt_query(node->sw, x1, y1, x2, y2, results, count, capacity);
    qt_query(node->se, x1, y1, x2, y2, results, count, capacity);
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

/* Return the deepest leaf depth (root = depth 0). */
int tree_depth(QuadNode *node, int current_depth)
{
    if (!node) return current_depth - 1;
    if (!node->nw) return current_depth;   /* leaf */
    int d0 = tree_depth(node->nw, current_depth + 1);
    int d1 = tree_depth(node->ne, current_depth + 1);
    int d2 = tree_depth(node->sw, current_depth + 1);
    int d3 = tree_depth(node->se, current_depth + 1);
    int mx = d0 > d1 ? d0 : d1;
        mx = mx > d2 ? mx : d2;
        mx = mx > d3 ? mx : d3;
    return mx;
}

/*
 * tree_dump — print an indented text representation of the tree.
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

    for (int i = 0; i < depth * 2; i++) putchar(' ');

    printf(CLR_DIM "%s " CLR_RESET, label);
    printf("[%d,%d %d\xc3\x97%d]",          /* × = UTF-8 U+00D7         */
           node->boundary.x, node->boundary.y,
           node->boundary.w,  node->boundary.h);

    if (node->nw) {
        printf(" — internal\n");
    } else if (node->count == 0) {
        printf(" — empty\n");
    } else {
        printf(" — %d pt%s: ", node->count, node->count == 1 ? "" : "s");
        for (int i = 0; i < node->count; i++) {
            printf(CLR_YELLOW "%c" CLR_RESET "(%d,%d)",
                   node->data[i].label, node->data[i].x, node->data[i].y);
            if (i < node->count - 1) printf(", ");
        }
        putchar('\n');
    }

    tree_dump(node->nw, depth + 1, "NW");
    tree_dump(node->ne, depth + 1, "NE");
    tree_dump(node->sw, depth + 1, "SW");
    tree_dump(node->se, depth + 1, "SE");
}

/* ── ASCII grid visualizer ──────────────────────────────────────── */

/*
 * The visualizer renders the quadtree into a SPACE_W × SPACE_H
 * character grid, then prints it with a surrounding border.
 *
 * Characters used:
 *   ' '       empty space
 *   +  -  |   tree node border corners, horizontal, vertical edges
 *   A..Z       data point label (yellow)
 *   *          data point found by a query (green)
 *   [  ]  ~    query rectangle corners/edges (red)
 */

static char g_grid[SPACE_H][SPACE_W + 1];   /* +1 for null terminator */

static void grid_clear(void)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(g_grid[r], ' ', SPACE_W);
        g_grid[r][SPACE_W] = '\0';
    }
}

/* Place a single character at (x, y), clipped to the grid bounds. */
static void grid_put(int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        g_grid[y][x] = ch;
}

/*
 * Draw the border of 'rect' onto the grid using +, -, | characters.
 * Corners use '+'.  Interior of the rect is left untouched.
 */
static void grid_draw_border(Rect r)
{
    int x0 = r.x,           y0 = r.y;
    int x1 = r.x + r.w - 1, y1 = r.y + r.h - 1;

    /* Horizontal edges */
    for (int x = x0; x <= x1; x++) {
        grid_put(x, y0, '-');
        grid_put(x, y1, '-');
    }
    /* Vertical edges */
    for (int y = y0; y <= y1; y++) {
        grid_put(x0, y, '|');
        grid_put(x1, y, '|');
    }
    /* Four corners */
    grid_put(x0, y0, '+');
    grid_put(x1, y0, '+');
    grid_put(x0, y1, '+');
    grid_put(x1, y1, '+');
}

/* Draw the query rectangle using [, ], ~ to distinguish from tree borders. */
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
 * Recursively render all nodes' borders, then their points.
 * Borders are drawn parent-first; children's borders create the
 * interior division lines.  Points are placed last so they appear
 * on top of any border that happens to share the same cell.
 */
static void grid_render_tree(QuadNode *node,
                             Point *found, int found_count)
{
    if (!node) return;

    grid_draw_border(node->boundary);

    for (int i = 0; i < node->count; i++) {
        char ch  = node->data[i].label;
        int  px  = node->data[i].x;
        int  py  = node->data[i].y;

        /* Check if this point is in the found set */
        bool is_found = false;
        for (int f = 0; f < found_count; f++)
            if (found[f].x == px && found[f].y == py) { is_found = true; break; }

        grid_put(px, py, is_found ? '*' : ch);
    }

    grid_render_tree(node->nw, found, found_count);
    grid_render_tree(node->ne, found, found_count);
    grid_render_tree(node->sw, found, found_count);
    grid_render_tree(node->se, found, found_count);
}

/* Print the grid with a surrounding box and ANSI colors per character. */
static void grid_print(void)
{
    /* Top border of the surrounding box */
    printf(CLR_DIM "+");
    for (int c = 0; c < SPACE_W; c++) putchar('-');
    printf("+" CLR_RESET "\n");

    for (int r = 0; r < SPACE_H; r++) {
        printf(CLR_DIM "|" CLR_RESET);
        for (int c = 0; c < SPACE_W; c++) {
            char ch = g_grid[r][c];
            if (ch >= 'A' && ch <= 'Z') {
                printf(CLR_YELLOW "%c" CLR_RESET, ch);   /* data point   */
            } else if (ch == '*') {
                printf(CLR_GREEN  "%c" CLR_RESET, ch);   /* found point  */
            } else if (ch == '[' || ch == ']' || ch == '~') {
                printf(CLR_RED    "%c" CLR_RESET, ch);   /* query rect   */
            } else if (ch == '+' || ch == '-' || ch == '|') {
                printf(CLR_CYAN   "%c" CLR_RESET, ch);   /* tree border  */
            } else {
                putchar(ch);
            }
        }
        printf(CLR_DIM "|" CLR_RESET "\n");
    }

    /* Bottom border */
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
 * show_tree — render the current tree state to the grid and print it
 *             along with a one-line summary of the tree's statistics.
 */
static void show_tree(const char *title, QuadNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_tree(root, NULL, 0);
    grid_print();
    printf("  nodes=%d  points=%d  depth=%d\n",
           tree_node_count(root),
           tree_point_count(root),
           tree_depth(root, 0));
}

/*
 * show_query — render the tree with a query rectangle overlay and
 *              highlight the found points.  Print found labels.
 */
static void show_query(const char *title, QuadNode *root,
                       int x1, int y1, int x2, int y2,
                       Point *found, int found_count)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_tree(root, found, found_count);
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

    int    qx1 = 3, qy1 = 2, qx2 = 23, qy2 = 9;
    Point  found[QUERY_CAP];
    int    found_count = 0;
    qt_query(root, qx1, qy1, qx2, qy2, found, &found_count, QUERY_CAP);
    show_query("  Query result", root, qx1, qy1, qx2, qy2, found, found_count);

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
           "    qt_insert(root, point)              O(log N)\n"
           "    qt_query(root, x1,y1,x2,y2, ...)   O(log N + k)\n"
           "    subdivide(node)                     O(LEAF_CAPACITY)\n"
           "    tree_node_count / tree_point_count  O(nodes)\n"
           "    tree_depth / tree_dump              O(nodes)\n"
           "    tree_free(root)                     O(nodes)\n"
           "\n");

    tree_free(root);
    return 0;
}
