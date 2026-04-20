/*
 * geometry/kd_tree.c — K-D Tree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1  (lines ~110-280)  — the kd-tree library
 *     Data structures, memory management, core operations,
 *     inspection helpers, and an ASCII grid visualizer.
 *     This part has no I/O — it is a reusable module.
 *
 *   PART 2  (lines ~280-end) — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every split live,
 *     then demonstrates a range query.  Press Enter each step.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/kd_tree.c -o kd_tree
 *
 * Run:
 *   ./kd_tree
 */

/* ── CONCEPTS ───────────────────────────────────────────────────────────── *
 *
 * K-D tree vs Quadtree vs BSP tree (all three are in geometry/):
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
 * Reading order:
 *   1. KDNode struct — understand how one node encodes a point AND a split.
 *   2. kd_insert — trace one insertion by hand through a 3-level tree.
 *   3. kd_query  — trace the pruning rule: draw the bounding box on paper.
 *   4. draw_tree_grid — see how the grid visualizer maps the tree back to 2-D.
 *   5. main() PART 2 — run the demo and predict each split before pressing Enter.
 *
 * ─────────────────────────────────────────────────────────────────────────── */

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
 * Point — a data point stored inside the kd-tree.
 *   x, y   — integer coordinates in [0, SPACE_W) × [0, SPACE_H)
 *   label  — single uppercase letter for display (e.g. 'A')
 */
typedef struct {
    int  x, y;
    char label;
} Point;

/*
 * KDNode — one node in the kd-tree.
 *
 * Every node holds exactly ONE data point and simultaneously acts as
 * a splitting plane.  The split axis alternates with insertion depth:
 *
 *   depth 0, 2, 4, … → axis 0 → split on X  (vertical dividing line)
 *   depth 1, 3, 5, … → axis 1 → split on Y  (horizontal dividing line)
 *
 * Children:
 *   left  — points whose coord[axis] <  this node's coord[axis]
 *   right — points whose coord[axis] >= this node's coord[axis]
 *           (ties and duplicates on the same coordinate go right)
 *
 * Contrast with a quadtree where internal nodes hold NO data and
 * always split both axes at once into four children.  Here every
 * node is simultaneously a data record and a spatial divider, and
 * we only cut ONE axis per level.
 */
typedef struct KDNode KDNode;
struct KDNode {
    Point   point;   /* the data point stored at this node           */
    int     axis;    /* 0 = x-split (vertical), 1 = y-split (horiz) */
    KDNode *left;    /* coord[axis] <  point[axis]                   */
    KDNode *right;   /* coord[axis] >= point[axis]                   */
};

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
 * Algorithm:
 *   At every node we compare p's relevant coordinate (x or y,
 *   determined by node->axis) against this node's split value.
 *   If p's coord < split  → recurse left.
 *   If p's coord >= split → recurse right.
 *   When we reach NULL we have found the correct leaf slot and
 *   allocate a new node there.
 *
 * Because the axis is stored in node->axis (set once at allocation),
 * query and traversal functions do not need a 'depth' parameter —
 * they can read axis directly from each node.
 */
KDNode *kd_insert(KDNode *node, Point p, int depth)
{
    if (!node) return kd_new(p, depth);   /* found the insertion slot */

    int coord = (node->axis == 0) ? p.x       : p.y;
    int split = (node->axis == 0) ? node->point.x : node->point.y;

    if (coord < split)
        node->left  = kd_insert(node->left,  p, depth + 1);
    else
        node->right = kd_insert(node->right, p, depth + 1);

    return node;
}

/*
 * kd_query — collect all points inside the inclusive rectangle
 *            [qx1,qx2] × [qy1,qy2] into results[].
 *
 * xmin/ymin/xmax/ymax describe the bounding box of the current
 * subtree.  The caller passes the full grid extents for the root.
 * At each recursive call the bounding box is halved along the
 * current node's axis, exactly mirroring how kd_insert routes.
 *
 * Pruning:
 *   Before examining a node we check whether its bounding box
 *   overlaps the query rectangle.  If it does NOT, the whole
 *   subtree is skipped — no point inside could be in range.
 *   This gives O(√N + k) average cost for 2-D (vs O(N) linear scan).
 *
 * Parameters:
 *   results  — caller-allocated array to receive found points
 *   count    — pointer to running count (caller initialises to 0)
 *   capacity — max number of results to store
 */
void kd_query(KDNode *node,
              int xmin, int ymin, int xmax, int ymax,
              int qx1,  int qy1,  int qx2,  int qy2,
              Point *results, int *count, int capacity)
{
    if (!node || *count >= capacity) return;

    /* ── Pruning step ────────────────────────────────────────────── */
    /* Does this node's bounding box overlap the query rectangle?    */
    /* Non-overlap when the box starts past the query edge on any    */
    /* axis, or ends before the query starts on any axis.            */
    if (xmin > qx2 || xmax < qx1 || ymin > qy2 || ymax < qy1) return;

    /* ── Point check ─────────────────────────────────────────────── */
    int px = node->point.x, py = node->point.y;
    if (px >= qx1 && px <= qx2 && py >= qy1 && py <= qy2)
        results[(*count)++] = node->point;

    /* ── Recurse, halving the bounding box along this node's axis ── */
    if (node->axis == 0) {
        int sx = node->point.x;
        kd_query(node->left,  xmin, ymin, sx - 1, ymax,
                 qx1, qy1, qx2, qy2, results, count, capacity);
        kd_query(node->right, sx,   ymin, xmax,   ymax,
                 qx1, qy1, qx2, qy2, results, count, capacity);
    } else {
        int sy = node->point.y;
        kd_query(node->left,  xmin, ymin, xmax, sy - 1,
                 qx1, qy1, qx2, qy2, results, count, capacity);
        kd_query(node->right, xmin, sy,   xmax, ymax,
                 qx1, qy1, qx2, qy2, results, count, capacity);
    }
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
 * The visualizer renders every node's splitting line into a
 * SPACE_W × SPACE_H character grid, then prints it with a border.
 *
 * Characters used:
 *   ' '       empty space
 *   |         x-split (vertical line through an x-axis node)   magenta
 *   -         y-split (horizontal line through a y-axis node)  blue
 *   +         intersection where a | and a - line cross        cyan
 *   A..Z      data point label                                  yellow
 *   *         data point found by a range query                green
 *   [ ] ~     query rectangle                                   red
 *
 * The render function is called with the bounding box of the current
 * subtree.  It draws the splitting line, recurses into children with
 * the halved bounding box, then places the point label on top so it
 * always appears over any line that passes through the same cell.
 */

static char g_grid[SPACE_H][SPACE_W + 1];   /* +1 for null terminator */

static void grid_clear(void)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(g_grid[r], ' ', SPACE_W);
        g_grid[r][SPACE_W] = '\0';
    }
}

/*
 * Place a line character ('|' or '-') handling intersections.
 * If the cell already holds the perpendicular line character,
 * write '+' instead so crossings are visible.
 */
static void grid_put_line(int x, int y, char ch)
{
    if (x < 0 || x >= SPACE_W || y < 0 || y >= SPACE_H) return;
    char cur = g_grid[y][x];
    if ((ch == '|' && cur == '-') || (ch == '-' && cur == '|'))
        g_grid[y][x] = '+';
    else if (cur != '+')   /* don't overwrite an already-correct '+' */
        g_grid[y][x] = ch;
}

/* Place a data point, always overriding any line character. */
static void grid_put_point(int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        g_grid[y][x] = ch;
}

/* Draw the query rectangle using [ ] ~ to distinguish from splits. */
static void grid_draw_query_rect(int x1, int y1, int x2, int y2)
{
    for (int x = x1; x <= x2; x++) {
        grid_put_point(x, y1, '~');
        grid_put_point(x, y2, '~');
    }
    for (int y = y1; y <= y2; y++) {
        grid_put_point(x1, y, '[');
        grid_put_point(x2, y, ']');
    }
    grid_put_point(x1, y1, '[');  grid_put_point(x2, y1, ']');
    grid_put_point(x1, y2, '[');  grid_put_point(x2, y2, ']');
}

/*
 * Recursively render splitting lines then point labels.
 *
 * xmin/ymin/xmax/ymax — the bounding box of the current subtree.
 * The root call passes [0, 0, SPACE_W-1, SPACE_H-1].
 *
 * For each node:
 *   axis 0 (x-split) → '|' at x=point.x spanning [ymin, ymax]
 *                    → left  child gets xmax = point.x - 1
 *                    → right child gets xmin = point.x
 *   axis 1 (y-split) → '-' at y=point.y spanning [xmin, xmax]
 *                    → left  child gets ymax = point.y - 1
 *                    → right child gets ymin = point.y
 * Point label is drawn last so it appears on top of its own split line.
 */
static void grid_render_tree(KDNode *node,
                             int xmin, int ymin, int xmax, int ymax,
                             Point *found, int found_count)
{
    if (!node) return;

    if (node->axis == 0) {
        int sx = node->point.x;
        for (int y = ymin; y <= ymax; y++) grid_put_line(sx, y, '|');
        grid_render_tree(node->left,  xmin, ymin, sx - 1, ymax, found, found_count);
        grid_render_tree(node->right, sx,   ymin, xmax,   ymax, found, found_count);
    } else {
        int sy = node->point.y;
        for (int x = xmin; x <= xmax; x++) grid_put_line(x, sy, '-');
        grid_render_tree(node->left,  xmin, ymin, xmax, sy - 1, found, found_count);
        grid_render_tree(node->right, xmin, sy,   xmax, ymax,   found, found_count);
    }

    /* Draw the data point on top of whatever line passes through it */
    bool is_found = false;
    for (int f = 0; f < found_count; f++)
        if (found[f].x == node->point.x && found[f].y == node->point.y)
            { is_found = true; break; }
    grid_put_point(node->point.x, node->point.y,
                   is_found ? '*' : node->point.label);
}

/* Print the grid with a surrounding box and per-character ANSI color. */
static void grid_print(void)
{
    printf(CLR_DIM "+");
    for (int c = 0; c < SPACE_W; c++) putchar('-');
    printf("+" CLR_RESET "\n");

    for (int r = 0; r < SPACE_H; r++) {
        printf(CLR_DIM "|" CLR_RESET);
        for (int c = 0; c < SPACE_W; c++) {
            char ch = g_grid[r][c];
            if      (ch >= 'A' && ch <= 'Z') printf(CLR_YELLOW  "%c" CLR_RESET, ch);
            else if (ch == '*')              printf(CLR_GREEN   "%c" CLR_RESET, ch);
            else if (ch == '[' || ch == ']'
                  || ch == '~')              printf(CLR_RED     "%c" CLR_RESET, ch);
            else if (ch == '|')              printf(CLR_MAGENTA "%c" CLR_RESET, ch);
            else if (ch == '-')              printf(CLR_BLUE    "%c" CLR_RESET, ch);
            else if (ch == '+')              printf(CLR_CYAN    "%c" CLR_RESET, ch);
            else                             putchar(ch);
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
 */
static void show_tree(const char *title, KDNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    if (root)
        grid_render_tree(root, 0, 0, SPACE_W - 1, SPACE_H - 1, NULL, 0);
    grid_print();
    if (!root)
        printf("  nodes=0  depth=-\n");
    else
        printf("  nodes=%d  depth=%d\n",
               kd_node_count(root), kd_depth(root, 0));
}

/*
 * show_query — render tree + query rectangle + highlighted found points.
 */
static void show_query(const char *title, KDNode *root,
                       int x1, int y1, int x2, int y2,
                       Point *found, int found_count)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_tree(root, 0, 0, SPACE_W - 1, SPACE_H - 1, found, found_count);
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
        "    A.right \xe2\x86\x92 C  bbox=[28,55]\xc3\x97[0,21]: xmin=28 > qx2=20  \xe2\x80\x94 PRUNED\n"
        "\n"
        "  The entire right half (C H I J K) is pruned in a single check.\n"
    );
    press_enter();

    int   qx1 = 0, qy1 = 0, qx2 = 20, qy2 = 10;
    Point found[QUERY_CAP];
    int   found_count = 0;

    kd_query(root, 0, 0, SPACE_W - 1, SPACE_H - 1,
             qx1, qy1, qx2, qy2, found, &found_count, QUERY_CAP);

    show_query("  Query result", root, qx1, qy1, qx2, qy2, found, found_count);
    press_enter();

    /* ── Done ─────────────────────────────────────────────────── */

    print_separator();
    printf(CLR_BOLD "  Done.\n" CLR_RESET
           "\n"
           "  Summary of operations implemented in this file:\n"
           "\n"
           "    kd_insert(root, point, depth)          O(log N) average\n"
           "    kd_query(root, bounds, rect, \xe2\x80\xa6)        O(\xe2\x88\x9aN + k) average\n"
           "    kd_node_count / kd_depth               O(N)\n"
           "    kd_dump                                O(N)\n"
           "    kd_free(root)                          O(N)\n"
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
