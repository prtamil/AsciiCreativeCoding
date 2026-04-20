/*
 * geometry/bsp_tree.c — Binary Space Partition tree: data structure, operations, visual demo
 *
 * This file is in two parts:
 *
 *   PART 1  (lines ~65-340)  — the BSP tree library
 *     Data structures, memory management, core operations,
 *     inspection helpers, and an ASCII grid visualizer.
 *     This part has no I/O — it is a reusable module.
 *
 *   PART 2  (lines ~340-end) — step-by-step demo in main()
 *     Inserts 12 labelled points, shows every split live,
 *     then demonstrates a range query.  Press Enter each step.
 *
 * A BSP tree splits a rectangular region with a single axis-aligned line,
 * creating two children (front / back) instead of four.  The split axis
 * alternates: even depth = VERTICAL (left/right), odd = HORIZONTAL (top/bottom).
 *
 * Contrast with the quadtree (see geometry/quadtree.c):
 *   Quadtree  — splits into 4 equal quadrants at once with a cross (+)
 *   BSP tree  — splits into 2 halves, one axis at a time (! or =)
 *
 * Historical note: BSP trees powered Doom (1993) for back-to-front rendering
 * and Quake for collision detection.  The tree was pre-computed at build time
 * to allow O(log N) visibility queries at run time.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/bsp_tree.c -o bsp_tree
 *
 * Run:
 *   ./bsp_tree
 */

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
 * Point — a data point stored inside the BSP tree.
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
 * SplitAxis — which axis an internal node split along.
 *
 *   SPLIT_VERTICAL   — a vertical line at x = split_pos divides the
 *                      region into front (left,  x < split_pos) and
 *                      back  (right, x >= split_pos).
 *
 *   SPLIT_HORIZONTAL — a horizontal line at y = split_pos divides the
 *                      region into front (top,    y < split_pos) and
 *                      back  (bottom, y >= split_pos).
 *
 * The axis is chosen by depth: even depth → VERTICAL, odd → HORIZONTAL.
 * This keeps the tree balanced along both dimensions.
 */
typedef enum { SPLIT_VERTICAL, SPLIT_HORIZONTAL } SplitAxis;

/*
 * BSPNode — one node in the BSP tree.
 *
 * A node is either a LEAF or an INTERNAL node:
 *
 *   Leaf node     — front and back are both NULL.
 *                   Holds up to LEAF_CAPACITY points in data[].
 *
 *   Internal node — front and back are non-NULL children.
 *                   data[] and count are unused (always 0).
 *                   Records the split axis and position.
 *
 *   front = left  half (VERTICAL)   or top    half (HORIZONTAL)
 *   back  = right half (VERTICAL)   or bottom half (HORIZONTAL)
 */
typedef struct BSPNode BSPNode;
struct BSPNode {
    Rect       boundary;             /* region this node is responsible for   */
    Point      data[LEAF_CAPACITY];  /* points stored here (leaf only)        */
    int        count;                /* number of points currently stored     */
    int        depth;                /* 0 = root; determines split axis       */
    int        split_pos;            /* coordinate of the dividing line       */
    SplitAxis  split_axis;           /* VERTICAL or HORIZONTAL (internal only)*/
    BSPNode   *front;                /* left (V) or top (H);  NULL = leaf     */
    BSPNode   *back;                 /* right (V) or bottom (H); NULL = leaf  */
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
 * subdivide — split a full leaf node into two children with one line.
 *
 * The split axis alternates with depth:
 *   Even depth → VERTICAL   split: left/right halves at x = boundary.x + w/2
 *   Odd  depth → HORIZONTAL split: top/bottom halves at y = boundary.y + h/2
 *
 * After subdivide(), 'node' becomes an internal node: count = 0,
 * front and back are both non-NULL.
 *
 *   Before:  [node: LEAF_CAPACITY pts]
 *
 *   After (VERTICAL):
 *     [node: internal, 0 pts, split_pos = x + w/2]
 *        /                     \
 *     [front: left half]    [back: right half]
 *     (pts redistributed)
 */
static void subdivide(BSPNode *node)
{
    Rect b       = node->boundary;
    int  next_d  = node->depth + 1;

    if (node->depth % 2 == 0) {
        /* ── Vertical split: divide left / right ─────────────────── */
        int half_w       = b.w / 2;
        node->split_axis = SPLIT_VERTICAL;
        node->split_pos  = b.x + half_w;

        Rect front_rect = { b.x,             b.y, half_w,      b.h };
        Rect back_rect  = { b.x + half_w,    b.y, b.w - half_w, b.h };
        node->front = node_new(front_rect, next_d);
        node->back  = node_new(back_rect,  next_d);
    } else {
        /* ── Horizontal split: divide top / bottom ────────────────── */
        int half_h       = b.h / 2;
        node->split_axis = SPLIT_HORIZONTAL;
        node->split_pos  = b.y + half_h;

        Rect front_rect = { b.x, b.y,          b.w, half_h      };
        Rect back_rect  = { b.x, b.y + half_h, b.w, b.h - half_h };
        node->front = node_new(front_rect, next_d);
        node->back  = node_new(back_rect,  next_d);
    }

    /*
     * Redistribute this leaf's points into the two children.
     * Snapshot them first because bsp_insert() modifies node->count.
     */
    int   old_count             = node->count;
    Point displaced[LEAF_CAPACITY];
    memcpy(displaced, node->data, (size_t)old_count * sizeof(Point));
    node->count = 0;

    for (int i = 0; i < old_count; i++) {
        if (!bsp_insert(node->front, displaced[i]))
            bsp_insert(node->back, displaced[i]);
    }
}

/*
 * bsp_insert — add point p into the subtree rooted at 'node'.
 *
 * Returns true if the point was accepted.
 * Returns false if it lies outside this node's boundary (used
 * internally during redistribution after subdivision).
 */
bool bsp_insert(BSPNode *node, Point p)
{
    if (!rect_contains_point(node->boundary, p.x, p.y))
        return false;   /* point is outside this node's region            */

    if (!node->front) {
        /* ── Leaf node ─────────────────────────────────────────── */
        if (node->count < LEAF_CAPACITY) {
            /* Room available: store the point here */
            node->data[node->count++] = p;
            return true;
        }
        /* Leaf is full: split into two children, then re-route */
        subdivide(node);
        /* node->front is now non-NULL; fall through to internal path */
    }

    /* ── Internal node ──────────────────────────────────────────── */
    /* Try front first; if it rejects (point is on the other side), try back */
    if (bsp_insert(node->front, p)) return true;
    if (bsp_insert(node->back,  p)) return true;

    return false;   /* should not happen if boundary covers the point    */
}

/*
 * bsp_query — find all points inside the inclusive rectangle
 *             [x1, x2] × [y1, y2] and append them to results[].
 *
 * The same AABB pruning used in the quadtree applies here:
 * if the search range does NOT overlap a node's boundary, the
 * entire subtree (both children) is skipped.
 *
 * Because the tree has only two children instead of four, fewer
 * subtrees are visited per level — the constant factor is smaller,
 * though the asymptotic complexity is the same O(log N + k).
 */
void bsp_query(BSPNode *node,
               int x1, int y1, int x2, int y2,
               Point *results, int *count, int capacity)
{
    if (!node || *count >= capacity) return;

    /* ── Pruning step ────────────────────────────────────────────── */
    if (!rect_overlaps_range(node->boundary, x1, y1, x2, y2)) return;

    /* ── Point check ─────────────────────────────────────────────── */
    for (int i = 0; i < node->count && *count < capacity; i++) {
        int px = node->data[i].x, py = node->data[i].y;
        if (px >= x1 && px <= x2 && py >= y1 && py <= y2)
            results[(*count)++] = node->data[i];
    }

    /* ── Recurse into both children ─────────────────────────────── */
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
