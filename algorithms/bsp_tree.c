/*
 * bsp_tree.c — A BSP (Binary Space Partition) tree that stores 2-D points
 * and answers "which points are inside this rectangle?" quickly. It splits
 * space with alternating vertical/horizontal cuts. Part 1 is the reusable
 * tree library; Part 2 is a step-by-step terminal demo (press Enter to advance).
 *
 * This is the simplified axis-aligned, point-storing variant — not the
 * arbitrary-plane BSP used by Doom/Quake. Siblings: algorithms/quadtree.c
 * (4-way split) and algorithms/kd_tree.c (data lives in nodes, not leaves).
 * Algorithm: Fuchs, Kedem & Naylor, "On Visible Surface Generation by A
 * Priori Tree Structures", SIGGRAPH '80; alternating-axis idea from
 * Bentley, "Multidimensional binary search trees", CACM 18(9), 1975.
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/bsp_tree.c -o bsp_tree
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── ANSI color codes — raw escape strings, so no color library is needed ── */
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

/* ── §1 PART 1: the reusable tree library — constants ── */

/* How many points a leaf holds before it must split into two.
 * Small (4) on purpose, so the demo splits often enough to watch. */
#define LEAF_CAPACITY  4

/* The demo world is a 56-by-22 grid of cells. Points live at integer
 * (x, y) inside [0, SPACE_W) by [0, SPACE_H). Sized to fit an 80-column
 * terminal once the +2/+2 outer border is drawn around it. */
#define SPACE_W  56
#define SPACE_H  22

/* Most points one range query can return. */
#define QUERY_CAP  64

/* ── §2 data types — the payload, the regions, the nodes ── */

/*
 * Point — one (x, y) location stored in the tree.
 *
 * This is what you insert and what a query hands back. Every leaf keeps
 * up to LEAF_CAPACITY of these; internal (split) nodes keep none.
 *
 * Coordinates are integers, not floats, because the demo draws onto a
 * fixed character grid (one cell = one position) — floats would only add
 * rounding headaches (e.g. a point sitting exactly on a split line) with
 * no visual gain. Real game-engine BSP trees use floats since their
 * geometry isn't grid-aligned.
 *
 * Members:
 *   x, y  — grid position, in [0, SPACE_W) by [0, SPACE_H).
 *   label — a single letter for display only; the algorithm never reads it.
 *           It just lets a human match a dot on screen to its insertion order.
 */
typedef struct {
    int  x;       /* horizontal grid position */
    int  y;       /* vertical grid position   */
    char label;   /* display letter only; ignored by the algorithm */
} Point;

/*
 * Rect — the rectangular patch of the world a node is in charge of.
 *
 * Every node stores one of these. Insert uses it to ask "is this point
 * inside my patch?"; query uses it to ask "could my patch hold anything
 * the search wants?" and skips the whole subtree if not.
 *
 * The two halves a node splits into cover its patch perfectly: together
 * they equal the parent, and they never overlap. That gap-free, no-overlap
 * tiling is what guarantees every point lands in exactly one leaf.
 *
 * The rectangle is "half-open": the left and top edges belong to it, but
 * the right and bottom edges do not. That is the trick that makes a clean
 * split: cutting [x, x+w) at the middle gives [x, mid) and [mid, x+w) —
 * they touch at mid, share no cell, and miss no cell. Closed edges would
 * need fiddly tie-breaking at the seam.
 *
 * Members:
 *   x, y — top-left corner; this corner IS included.
 *   w, h — width and height; the right edge (x+w) and bottom edge (y+h)
 *          are just past the rectangle, NOT included.
 */
typedef struct {
    int x;    /* top-left x, included     */
    int y;    /* top-left y, included     */
    int w;    /* width;  right edge x+w is excluded  */
    int h;    /* height; bottom edge y+h is excluded */
} Rect;

/*
 * SplitAxis — which way a node's dividing line runs.
 *
 * A node cuts its patch either with a vertical line (left/right halves)
 * or a horizontal line (top/bottom halves). The choice alternates with
 * depth: even depths cut vertical, odd depths cut horizontal. Alternating
 * keeps the tree from getting lopsided — if it only ever cut one way,
 * points strung out along that direction would pile into a thin tower of
 * splits. (This alternating trick comes from the k-d tree, Bentley 1975.)
 *
 * Members:
 *   SPLIT_VERTICAL   — vertical line at x = split_pos; front = left half
 *                      (x < split_pos), back = right half (x >= split_pos).
 *   SPLIT_HORIZONTAL — horizontal line at y = split_pos; front = top half
 *                      (y < split_pos), back = bottom half (y >= split_pos).
 */
typedef enum {
    SPLIT_VERTICAL,    /* dividing line is vertical:   x = split_pos */
    SPLIT_HORIZONTAL,  /* dividing line is horizontal: y = split_pos */
} SplitAxis;

/*
 * BSPNode — one node of the tree; it is in one of two modes.
 *
 *   Leaf     — no children (front == NULL). It just holds up to
 *              LEAF_CAPACITY points in data[]. split_* are unused.
 *   Internal — has both children. Its data[] is emptied (count = 0) and
 *              split_axis/split_pos describe where it cut its patch.
 *
 * A node turns from leaf into internal in subdivide() when a leaf overflows.
 * The two children carve up the parent's patch with no overlap and no gap,
 * so any point in the parent lands in exactly one child — that is why the
 * insert routine can safely "try front, else back."
 *
 * One struct serves both modes (rather than two types or a tagged union):
 * the shared fields dominate the size, the mode-specific fields are small,
 * and the wasted bytes aren't worth the extra API. The quadtree sibling
 * uses the same "null child means leaf" trick.
 *
 * To tell the modes apart, test front == NULL (this is what node_is_leaf
 * does). Do NOT test count > 0 for that: an empty leaf and an internal node
 * both have count == 0, so count can't distinguish them.
 *
 * Members:
 *   boundary   — the patch of world this node owns (both modes).
 *   depth      — distance from root; root is 0. Its parity picks the cut
 *                direction (even = vertical, odd = horizontal).
 *   data       — the points held here; valid only in leaf mode.
 *   count      — how many slots of data[] are filled; 0..LEAF_CAPACITY.
 *   split_axis — which way this node cut (internal mode only).
 *   split_pos  — the x (vertical) or y (horizontal) of the cut line.
 *   front      — left/top child; NULL means this node is a leaf.
 *   back       — right/bottom child.
 */
typedef struct BSPNode BSPNode;
struct BSPNode {
    /* shared by both modes */
    Rect      boundary;             /* patch of world this node owns          */
    int       depth;                /* 0 = root; parity picks the cut axis    */

    /* leaf mode only (front == NULL); cleared to count=0 when it splits */
    Point     data[LEAF_CAPACITY];  /* points stored at this leaf            */
    int       count;                /* filled entries in data[]; 0 = empty    */

    /* internal mode only (set by subdivide when the node splits) */
    SplitAxis split_axis;           /* which way the dividing line runs       */
    int       split_pos;            /* coordinate of the dividing line        */
    BSPNode  *front;                /* left (V) or top (H) child; NULL = leaf */
    BSPNode  *back;                 /* right (V) or bottom (H) child          */
};

/* ── §3 memory: build and tear down nodes ── */

/* Make a fresh empty leaf covering 'boundary' at 'depth'. Caller frees it
 * (eventually via tree_free on the root). */
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

/* Free a whole subtree: children first, then the node itself. Call on the
 * root to release everything. */
void tree_free(BSPNode *node)
{
    if (!node) return;
    tree_free(node->front);
    tree_free(node->back);
    free(node);
}

/* ── §4 rectangle tests — is a point inside, do two regions touch ── */

/* Is the point (px, py) inside rectangle r? (Right/bottom edges count as
 * outside, matching the half-open convention.) */
static bool rect_contains_point(Rect r, int px, int py)
{
    return px >= r.x && px < r.x + r.w
        && py >= r.y && py < r.y + r.h;
}

/*
 * Does this node's patch overlap the search box [x1,x2] x [y1,y2] at all?
 * It's easier to spot the four ways they CAN'T touch (search is fully to
 * the left, right, above, or below the patch) and negate that — anything
 * else means they share some area. This is the test that lets a query skip
 * an entire subtree at once.
 */
static bool rect_overlaps_range(Rect boundary,
                                int x1, int y1, int x2, int y2)
{
    int bx = boundary.x, by = boundary.y;
    int bw = boundary.w, bh = boundary.h;
    return !(x1 >= bx + bw || x2 < bx || y1 >= by + bh || y2 < by);
}

/* ── §5 core operations — insert a point, split a full leaf, query a box ── */

/* Forward declaration: subdivide() calls bsp_insert() to hand its old
 * points down to the new children. */
static bool bsp_insert(BSPNode *node, Point p);

/*
 * Cut rectangle b down the middle into a left ('front') and right ('back')
 * half, and return the x where the cut sits. The two halves cover b exactly
 * with no overlap. If b's width is odd the right half gets the extra column;
 * that's fine, it stays an exact partition. Used when a node at an even
 * depth needs to split.
 */
static int vertical_bisect(Rect b, Rect *front, Rect *back)
{
    int half_w = b.w / 2;
    *front = (Rect){ b.x,             b.y, half_w,       b.h };
    *back  = (Rect){ b.x + half_w,    b.y, b.w - half_w, b.h };
    return b.x + half_w;                  /* x-coordinate of the cut */
}

/*
 * Same as vertical_bisect but cutting across the middle into a top ('front')
 * and bottom ('back') half; returns the y of the cut. Used at odd depths.
 */
static int horizontal_bisect(Rect b, Rect *front, Rect *back)
{
    int half_h = b.h / 2;
    *front = (Rect){ b.x, b.y,          b.w, half_h       };
    *back  = (Rect){ b.x, b.y + half_h, b.w, b.h - half_h };
    return b.y + half_h;                  /* y-coordinate of the cut */
}

/*
 * Hand a freshly-split node's old points down to its two new children.
 *
 * The node just became internal, so its own data[] no longer counts. Each
 * old point belongs to whichever child contains it: try the front child,
 * and if it refuses (point is on the other side of the cut) the back child
 * takes it — exactly one accepts.
 *
 * We copy data[] into a local buffer first, because the inserts below will
 * be writing to node->count as they recurse; reading and writing the same
 * array at once would corrupt it.
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
 * Turn an overflowing leaf into an internal (split) node.
 *
 * It cuts the patch in two — vertically at even depths, horizontally at odd
 * depths (alternating keeps the tree balanced; see SplitAxis) — makes a
 * child for each half, and hands the leaf's old points down to them. After
 * this the node has two children and an empty data[], so it is internal.
 */
static void subdivide(BSPNode *node)
{
    Rect b      = node->boundary;
    int  next_d = node->depth + 1;

    /* pick the cut direction from depth, then split the patch in two */
    Rect front_rect, back_rect;
    if (node->depth % 2 == 0) {
        node->split_axis = SPLIT_VERTICAL;
        node->split_pos  = vertical_bisect  (b, &front_rect, &back_rect);
    } else {
        node->split_axis = SPLIT_HORIZONTAL;
        node->split_pos  = horizontal_bisect(b, &front_rect, &back_rect);
    }

    node->front = node_new(front_rect, next_d);
    node->back  = node_new(back_rect,  next_d);

    redistribute_points_into_children(node);
}

/* A node is a leaf exactly when it has no front child. */
static inline bool node_is_leaf(const BSPNode *node)
{
    return node->front == NULL;
}

/* Can this leaf take one more point without overflowing? */
static inline bool leaf_has_room(const BSPNode *node)
{
    return node->count < LEAF_CAPACITY;
}

/* Append p to this leaf's points. Caller must have checked there's room. */
static inline void store_in_leaf(BSPNode *node, Point p)
{
    node->data[node->count++] = p;
}

/*
 * Push p down to whichever child contains it. We don't bother working out
 * which side of the cut p is on — just try the front child, and if it says
 * "not mine" the back child gets it. Exactly one will accept, since the two
 * children cover the parent with no overlap.
 */
static inline bool route_into_children(BSPNode *node, Point p)
{
    if (bsp_insert(node->front, p)) return true;
    return bsp_insert(node->back, p);
}

/*
 * Add point p into the subtree at 'node'. Returns true if p landed here,
 * false if p was outside this node's patch (so a caller can try a sibling).
 *
 * Walk it down: reject if p isn't in our patch; if we're a leaf with room,
 * just store it; if we're a full leaf, split first (so a leaf never holds
 * more than LEAF_CAPACITY), then fall through and hand p to a child.
 */
bool bsp_insert(BSPNode *node, Point p)
{
    if (!rect_contains_point(node->boundary, p.x, p.y))
        return false;

    if (node_is_leaf(node) && leaf_has_room(node)) {
        store_in_leaf(node, p);
        return true;
    }

    if (node_is_leaf(node))
        subdivide(node);

    return route_into_children(node, p);
}

/* Is point p inside the search box [x1,x2] x [y1,y2]? (Edges count as
 * inside here, unlike the half-open node patches.) Even when a leaf's patch
 * overlaps the search box, some of its points can still fall outside it, so
 * each point is checked individually. */
static inline bool point_in_query_range(Point p,
                                        int x1, int y1, int x2, int y2)
{
    return p.x >= x1 && p.x <= x2 && p.y >= y1 && p.y <= y2;
}

/* Copy this node's points that fall inside the search box into results[],
 * stopping once results[] is full. Internal nodes have count == 0, so this
 * does nothing for them — only leaves actually hold points. */
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
 * Find every point inside the box [x1,x2] x [y1,y2] and append them to
 * results[] (up to capacity), counting them in *count.
 *
 * The trick that makes this fast: at each node, if its patch doesn't even
 * touch the search box, skip the whole subtree at once — no need to look at
 * any of its points. Otherwise collect this node's matching points and
 * recurse into both children. Skipping whole subtrees is what turns a
 * scan-everything search into a fast one (same idea a quadtree uses).
 */
void bsp_query(BSPNode *node,
               int x1, int y1, int x2, int y2,
               Point *results, int *count, int capacity)
{
    if (!node || *count >= capacity) return;

    /* the prune: patch doesn't reach the box, so nothing here can match */
    if (!rect_overlaps_range(node->boundary, x1, y1, x2, y2)) return;

    collect_points_at_node(node, x1, y1, x2, y2, results, count, capacity);

    bsp_query(node->front, x1, y1, x2, y2, results, count, capacity);
    bsp_query(node->back,  x1, y1, x2, y2, results, count, capacity);
}

/* ── §6 inspection — count nodes/points/depth, dump the tree as text ── */

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
 * Print the tree as indented text, one node per line, deeper nodes more
 * indented. Each line shows the node's patch, then either its split line
 * (internal) or its points (leaf). Example:
 *   root [0,0 56x22] — VERTICAL split at x=28
 *     front [0,0 28x22] — HORIZONTAL split at y=11
 *       front [0,0 28x11] — 3 pts: A(7,5)  F(21,3)  H(14,2)
 *       ...
 */
void tree_dump(BSPNode *node, int depth, const char *label)
{
    if (!node) return;

    for (int i = 0; i < depth * 2; i++) putchar(' ');

    printf(CLR_DIM "%s " CLR_RESET, label);
    printf("[%d,%d %d\xc3\x97%d]",          /* \xc3\x97 = the times sign x */
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

/* ── §7 ASCII visualizer — paint the tree onto a character grid ── */

/*
 * Draws the tree into a character grid and prints it in color. The glyphs:
 *   space   empty
 *   + - |   outer border
 *   !       vertical split line
 *   =       horizontal split line
 *   A..Z    a stored point
 *   *       a point the current query found
 *   [ ] ~   the query rectangle
 *
 * Split lines get their own glyphs (! and =) so they read differently from
 * the border (- and |) even with color turned off.
 */

static char g_grid[SPACE_H][SPACE_W + 1];   /* +1 column for the row's '\0' */

static void grid_clear(void)
{
    for (int r = 0; r < SPACE_H; r++) {
        memset(g_grid[r], ' ', SPACE_W);
        g_grid[r][SPACE_W] = '\0';
    }
}

/* Write ch at (x, y), silently ignoring anything off the grid. */
static void grid_put(int x, int y, char ch)
{
    if (x >= 0 && x < SPACE_W && y >= 0 && y < SPACE_H)
        g_grid[y][x] = ch;
}

/* Draw the box outline of rectangle r with + - | glyphs. */
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

/* Draw the search box with [ ] ~ so it stands apart from the tree borders. */
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
 * Walk the tree painting it onto the grid: each internal node draws its
 * split line, each leaf draws its points (a point the query found becomes
 * '*' instead of its letter). The outer border is painted afterwards by the
 * caller so it stays clean where split lines reach the edge.
 */
static void grid_render_bsp(BSPNode *node, Point *found, int found_count)
{
    if (!node) return;

    if (node->front) {
        if (node->split_axis == SPLIT_VERTICAL) {
            int x = node->split_pos;
            for (int y = node->boundary.y;
                     y < node->boundary.y + node->boundary.h; y++)
                grid_put(x, y, '!');
        } else {
            int y = node->split_pos;
            for (int x = node->boundary.x;
                     x < node->boundary.x + node->boundary.w; x++)
                grid_put(x, y, '=');
        }
    } else {
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

/* Print the grid, wrapping it in a border and coloring each glyph by what
 * it represents (point, found, query, split, border). */
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

/* ── §8 PART 2: the walkthrough demo — helpers ── */

/* Wait for the user to press Enter (swallowing any other keys first). */
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

/* Print a titled snapshot of the tree plus its node/point/depth counts.
 * The border is drawn last so split lines never poke through it. */
static void show_tree(const char *title, BSPNode *root)
{
    printf("\n" CLR_GOLD CLR_BOLD "%s" CLR_RESET "\n", title);
    grid_clear();
    grid_render_bsp(root, NULL, 0);
    grid_draw_border(root->boundary);
    grid_print();
    printf("  nodes=%d  points=%d  depth=%d\n",
           tree_node_count(root),
           tree_point_count(root),
           tree_depth(root));
}

/* Like show_tree, but also overlays the search box and lists the points the
 * query found (highlighted as '*' on the grid). */
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
