/*
 * algorithms/kd_tree.c — a 2-D k-d tree with a step-by-step ASCII demo.
 *
 * A k-d tree sorts points by x, then y, then x again, level by level, so a
 * range search can skip whole regions instead of checking every point. Part 1
 * is the reusable tree (insert + range query); part 2 walks main() through
 * inserting 12 points and one query, pausing on Enter. Sister structures:
 * algorithms/quadtree.c, algorithms/bsp_tree.c.
 *
 * Algorithm: Bentley, "Multidimensional binary search trees" (CACM 1975).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/kd_tree.c -o kd_tree
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── ANSI color codes — what each color flags on screen ── */
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

/* ── §1 PART 1: the k-d tree library — sizes the grid lives on ── */

/* The drawing area: a 56-wide by 22-tall grid of cells. Every point's (x,y)
 * is a cell address in [0,SPACE_W) x [0,SPACE_H). Sized to fit an 80-column
 * terminal once the surrounding border is drawn. */
#define SPACE_W   56
#define SPACE_H   22

/* Most points one range query can hand back. Matches over this are dropped. */
#define QUERY_CAP 64

/* ── data types ── */

/*
 * Point — an atomic data point stored inside the kd-tree.
 *
 * The one thing the tree sorts. The tree only ever looks at (x,y) to decide
 * left or right; the label just rides along so the demo can say "insert A".
 * Coordinates are whole numbers because each one is a cell on the screen grid
 * (a real app would use floats — only the comparison type changes).
 *
 * Members
 *   x      column, 0..SPACE_W-1.  The value the tree splits on at even depths.
 *   y      row,    0..SPACE_H-1.  The value the tree splits on at odd depths.
 *                  (y grows downward, as on a screen)
 *   label  a letter A..Z, drawn on screen; the tree never reads it.
 */
typedef struct {
    int  x;       /* column 0..SPACE_W-1 — split value at even depths */
    int  y;       /* row    0..SPACE_H-1 — split value at odd depths   */
    char label;   /* display letter A..Z; ignored by the tree itself   */
} Point;

/*
 * KDNode — one node in the tree. It does two jobs at once: it stores a point,
 * and it splits the screen with a line drawn through that point. The split is
 * vertical or horizontal depending on the node's depth, and that one line
 * decides which child every later point goes to. Storing the point and the
 * split in the same node (rather than separate leaf/internal types) is what
 * keeps a k-d tree compact.
 *
 * Each node remembers its own axis (set once, by depth, in kd_new) so the
 * recursive walks don't have to carry a depth counter — a node knows how to
 * route on its own. There's no parent pointer because every operation here
 * walks top-down; deletion would need one (see Bentley 1990).
 *
 * Members
 *   point   the stored point; its x (or y) is also the split line's position.
 *   axis    0 = split on x (a vertical line); 1 = split on y (horizontal).
 *           Fixed when the node is born; never changes.
 *   left    child holding points on the smaller side of the split (NULL = empty).
 *   right   child holding points on the equal-or-larger side (ties go here).
 */
typedef struct KDNode KDNode;
struct KDNode {
    Point   point;   /* the stored point; its coord on `axis` is the split line */
    int     axis;    /* 0 = vertical (x) split, 1 = horizontal (y); set once    */
    KDNode *left;    /* points whose coord on this axis is smaller              */
    KDNode *right;   /* points whose coord is equal or larger (ties land here)  */
};

/*
 * BBox — a rectangle of grid cells, edges included on both sides. Used for
 * three things here, all the same shape: the patch of grid a subtree owns
 * (shrinks as you walk down), the search rectangle the user asks about, and
 * the area the renderer is allowed to draw a split line into.
 *
 * "Edges included" means a point exactly on a side still counts as inside.
 * That choice also lets a split cut the rectangle into two halves that fit
 * together perfectly with no gap and no overlap (left gets up to value-1,
 * right starts at value). Passed around by value — it's only four ints.
 *
 * Members
 *   xmin   left edge   (inclusive)
 *   ymin   top edge    (inclusive; y grows downward)
 *   xmax   right edge  (inclusive)
 *   ymax   bottom edge (inclusive)
 * A valid box has xmin<=xmax and ymin<=ymax; an "inside-out" box (min>max)
 * is treated as empty and is pruned away by bbox_overlap.
 */
typedef struct {
    int xmin;   /* left edge,  inclusive */
    int ymin;   /* top edge,   inclusive (y grows downward) */
    int xmax;   /* right edge, inclusive */
    int ymax;   /* bottom edge, inclusive */
} BBox;

/*
 * GridCanvas — a paper-doll grid of characters the demo draws on. The grid_*
 * helpers paint into it (split lines, point letters, query frame) in any
 * order, then grid_print walks it once and colors each character. Separating
 * "draw" from "print" means the drawing code never worries about color codes.
 *
 * Each row carries one extra column holding a string terminator, so a row can
 * be printed as a plain C string if you ever need to.
 *
 * What a cell can hold (see glyph_color_for for the matching colors):
 *     ' '       empty
 *     '|'       a vertical split line
 *     '-'       a horizontal split line
 *     '+'       where a vertical and horizontal line cross
 *     'A'..'Z'  a stored point
 *     '*'       a stored point that a query found
 *     '[' ']'   left/right side (and corners) of the query rectangle
 *     '~'       top/bottom of the query rectangle
 *
 * Members
 *   cells   [row][col] grid of characters. cells[row][SPACE_W] is the per-row
 *           string terminator — leave it alone, never draw there.
 */
typedef struct {
    char cells[SPACE_H][SPACE_W + 1];   /* [row][col]; last col per row is the '\0' */
} GridCanvas;

/*
 * QueryResult — where kd_query drops the points it found. The caller makes one
 * (on the stack, no malloc), zeroes count, and passes it in; the search fills
 * it. Storage is a fixed array, so if a query somehow finds more than
 * QUERY_CAP points the extras are silently dropped — size QUERY_CAP for your
 * worst case (any value >= 12 is plenty for this 12-point demo).
 *
 * Found points come out in tree-walk order (left child before right), not
 * sorted by distance; sort the slice yourself if you need that.
 *
 * Members
 *   points   the matches; only points[0 .. count-1] are filled in.
 *   count    how many matches; starts at 0, never exceeds QUERY_CAP.
 */
typedef struct {
    Point points[QUERY_CAP];   /* the matches; valid up to index count-1 */
    int   count;               /* how many were found, 0..QUERY_CAP      */
} QueryResult;

/* ── box + axis helpers — the small geometry the tree leans on ── */

/* The two rules that make the tree alternate x, y, x, y... down the levels:
 * even depth splits on x (axis 0), odd depth splits on y (axis 1); and
 * coord_on_axis reads the x or y of a point depending on the axis. */
static inline int axis_at_depth(int depth)       { return depth % 2; }
static inline int coord_on_axis(Point p, int ax) { return ax == 0 ? p.x : p.y; }

/* The whole grid — the box every search starts from at the root. */
static inline BBox bbox_full_grid(void) {
    return (BBox){0, 0, SPACE_W - 1, SPACE_H - 1};
}

/* Do two rectangles touch or overlap? This is the pruning test: if a
 * subtree's box misses the search rectangle, the whole subtree is skipped. */
static inline bool bbox_overlap(BBox a, BBox b) {
    return !(a.xmin > b.xmax || a.xmax < b.xmin ||
             a.ymin > b.ymax || a.ymax < b.ymin);
}

/* Is a point inside the (edge-inclusive) box? */
static inline bool point_in_bbox(Point p, BBox b) {
    return p.x >= b.xmin && p.x <= b.xmax &&
           p.y >= b.ymin && p.y <= b.ymax;
}

/* Cut a box in two at `value` along one axis, the same way the tree routes
 * points: the left half stops just before value, the right half starts at
 * value. The renderer cuts with these same calls, so the lines on screen
 * land exactly where the tree's splits are. */
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

/* ── memory: make and free nodes ── */

/* Make a new childless node holding p. Its axis is fixed here, by depth. */
static KDNode *kd_new(Point p, int depth)
{
    KDNode *n = malloc(sizeof *n);
    assert(n != NULL);
    n->point = p;
    n->axis  = depth % 2;   /* even depth splits on x, odd on y */
    n->left  = n->right = NULL;
    return n;
}

/* Free a node and everything below it. Caller must not touch it after. */
void kd_free(KDNode *node)
{
    if (!node) return;
    kd_free(node->left);
    kd_free(node->right);
    free(node);
}

/* ── the two core operations: insert and range query ── */

/*
 * Drop point p into the tree. At each node, compare p against that node's
 * split value on its axis: smaller goes left, equal-or-larger goes right.
 * Keep going until you fall off the tree, and put p there. Returns the
 * subtree root so the caller can write: root = kd_insert(root, p, 0).
 *
 * `depth` is only here so a brand-new node can pick its own axis; once a node
 * exists it remembers its axis, so the search functions don't need depth.
 */
KDNode *kd_insert(KDNode *node, Point p, int depth)
{
    if (!node) return kd_new(p, depth);   /* empty slot — p lands here */

    int axis  = node->axis;
    int coord = coord_on_axis(p,           axis);
    int split = coord_on_axis(node->point, axis);

    if (coord < split) node->left  = kd_insert(node->left,  p, depth + 1);
    else               node->right = kd_insert(node->right, p, depth + 1);

    return node;
}

/*
 * Find every stored point inside query_rect. The trick that makes a k-d tree
 * fast: as we walk down, we track the patch of grid (`box`) the current
 * subtree can possibly cover. If that patch doesn't even touch the search
 * rectangle, none of its points can match, so we skip the whole subtree in one
 * step instead of checking each point. That skipping is the whole point of the
 * structure — it's why a range search beats scanning all the points.
 *
 *   node        the subtree we're looking at
 *   box         the grid patch this subtree covers (shrinks as we descend)
 *   query_rect  the rectangle we're searching (same the whole way down)
 *   out         where matches are collected (caller sets out->count to 0)
 */
void kd_query(KDNode *node, BBox box, BBox query_rect, QueryResult *out)
{
    if (!node || out->count >= QUERY_CAP)     return;
    if (!bbox_overlap(box, query_rect))       return;   /* nothing here can match */

    if (point_in_bbox(node->point, query_rect))
        out->points[out->count++] = node->point;

    int axis  = node->axis;
    int split = coord_on_axis(node->point, axis);
    kd_query(node->left,  bbox_split_left (box, axis, split), query_rect, out);
    kd_query(node->right, bbox_split_right(box, axis, split), query_rect, out);
}

/* ── inspecting the tree: count, depth, text dump ── */

/* How many points are stored. */
int kd_node_count(KDNode *node)
{
    if (!node) return 0;
    return 1 + kd_node_count(node->left) + kd_node_count(node->right);
}

/* How tall the tree is (root counts as 0). Call with current = 0 at the root;
 * a missing child reports its parent's depth so empty slots don't add height. */
int kd_depth(KDNode *node, int current)
{
    if (!node) return current - 1;
    int dl = kd_depth(node->left,  current + 1);
    int dr = kd_depth(node->right, current + 1);
    return dl > dr ? dl : dr;
}

/*
 * Print the tree as indented text — one line per node, deeper nodes indented
 * more, so you can read the parent/child shape. Example after all 12 points:
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

/* ── drawing the tree as ASCII art ── */

/* These helpers paint split lines and points onto a GridCanvas, then grid_print
 * walks it once and colors each character. Points are drawn last (after their
 * split lines) so a node's letter never gets hidden under its own line. */

/* Blank the whole canvas to spaces. */
static void grid_clear(GridCanvas *gc)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(gc->cells[r], ' ', SPACE_W);
        gc->cells[r][SPACE_W] = '\0';
    }
}

/* Draw one cell of a split line. If a vertical and horizontal line meet in the
 * same cell, draw a '+' so the crossing shows; an existing '+' is left alone. */
static void grid_put_line(GridCanvas *gc, int x, int y, char ch)
{
    if (x < 0 || x >= SPACE_W || y < 0 || y >= SPACE_H) return;
    char cur = gc->cells[y][x];
    if ((ch == '|' && cur == '-') || (ch == '-' && cur == '|'))
        gc->cells[y][x] = '+';
    else if (cur != '+')   /* keep a crossing we already drew */
        gc->cells[y][x] = ch;
}

/* Stamp a character at (x, y), painting over any line already there. */
static void grid_put_point(GridCanvas *gc, int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        gc->cells[y][x] = ch;
}

/* Draw the search rectangle as a frame: '~' along top/bottom, '[' and ']' down
 * the sides and at the corners. Uses characters the split lines never use, so
 * the search box is easy to tell apart from the tree's own lines. */
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

/* Was this point one the query found? grid_render_tree uses it to draw a
 * found point as '*' instead of its letter. */
static bool point_is_highlighted(Point p, const QueryResult *highlight)
{
    if (!highlight) return false;
    for (int i = 0; i < highlight->count; i++)
        if (highlight->points[i].x == p.x && highlight->points[i].y == p.y)
            return true;
    return false;
}

/* Walk the tree and paint each node's split line across the patch of grid it
 * owns, then draw the point on top. It splits the box and recurses exactly the
 * way kd_query does, so the picture matches the tree cell-for-cell. Pass a
 * query result as `highlight` to mark found points with '*'; NULL for none. */
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

    /* Draw the point last so its own split line can't hide it. */
    char glyph = point_is_highlighted(node->point, highlight)
               ? '*' : node->point.label;
    grid_put_point(gc, node->point.x, node->point.y, glyph);
}

/* The color for one character on the canvas (NULL = print it uncolored). */
static const char *glyph_color_for(char ch)
{
    if (ch >= 'A' && ch <= 'Z')               return CLR_YELLOW;
    if (ch == '*')                            return CLR_GREEN;
    if (ch == '[' || ch == ']' || ch == '~')  return CLR_RED;
    if (ch == '|')                            return CLR_MAGENTA;
    if (ch == '-')                            return CLR_BLUE;
    if (ch == '+')                            return CLR_CYAN;
    return NULL;
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

/* ── §2 PART 2: the step-by-step demo — small printing helpers ── */

/* Wait until the user presses Enter (lets them read each step). */
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

/* Draw the tree as it stands now, with a title and a node/depth summary. */
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

/* Like show_tree, but also draws the search rectangle, marks the points it
 * found with '*', and lists them below. */
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

/* ── main: build the tree one group at a time, then query it ── */

int main(void)
{
    /* ── intro: explain the rules and the legend ── */

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
