/*
 * 07_trihexagonal.c — trihexagonal tiling: hex grid + triangular grid overlaid
 *
 * DEMO: Two grids drawn simultaneously — the flat-top hexagonal grid (cyan)
 *       and the equilateral triangular grid whose vertices coincide with the
 *       hex vertices (green). Together they approximate the trihexagonal
 *       Archimedean tiling (vertex config 3.6.3.6). Toggle each grid with h/t.
 *       An '@' cursor moves between hex cells with arrow keys; the cursor hex
 *       is highlighted across both visible layers.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (hex algorithm)
 *                  grids/hex_grids/05_triangular.c (triangle algorithm)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs
 *   §4 coords   — pixel↔cell notes, dual grid relationship
 *   §5 draw     — hex layer + triangle layer
 *   §5b cursor  — movement vectors, cursor_draw
 *   §6 scene    — state struct + visibility toggles
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  h:hex  t:tri  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 07_trihexagonal.c -o 07_trihexagonal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : The trihexagonal tiling is Archimedean tiling #3.6.3.6:
 *                  at every vertex, the faces alternate triangle-hex-triangle-hex.
 *                  It can be constructed by superimposing:
 *                    (A) a flat-top hex grid of side s
 *                    (B) a triangular grid whose vertices coincide with the
 *                        hex vertices (triangle side = s, scaled by √3)
 *                  The combination creates the star-of-David / snowflake
 *                  pattern characteristic of the trihexagonal tiling.
 *
 * Data-structure : Two independent rasterizers: hex_layer() and tri_layer().
 *                  Both classify every screen cell independently. A cell can
 *                  be hit by either, neither, or both (drawn in the order hex
 *                  → tri so triangle edges overwrite hex edges at intersections).
 *
 *                  Hex layer: flat-top cube coordinates (01_flat_top algorithm).
 *                  Tri layer: three-family stripe classifier (05_triangular),
 *                             but with tri_size scaled so triangle vertices
 *                             align with hex vertices.
 *
 * Rendering      : Hex borders drawn in PAIR_HEX (cyan). Triangle borders
 *                  drawn in PAIR_TRI (green). Cursor hex drawn in PAIR_CURSOR
 *                  (white-on-blue). Where they share a border cell (at hex-edge
 *                  midpoints), the triangle color wins (drawn last). Toggle
 *                  visibility with 'h' and 't' keys to see each grid
 *                  independently, then combine to observe the interaction.
 *
 * Performance    : Two O(rows×cols) passes per frame. Total ~2× 01_flat_top.
 *                  Still well within 60 fps budget on any modern terminal.
 *
 * References     :
 *   Trihexagonal tiling (Wikipedia)
 *     https://en.wikipedia.org/wiki/Trihexagonal_tiling
 *   Archimedean tilings enumeration
 *     https://en.wikipedia.org/wiki/Archimedean_tiling
 *   Hex–triangle duality
 *     https://www.redblobgames.com/grids/hexagons/#map-storage
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The trihexagonal tiling is the result of overlaying the hexagonal and
 * triangular tilings so their vertices coincide. At every vertex in the
 * combined tiling, exactly 2 triangles and 2 hexagons meet in alternating
 * order (triangle-hex-triangle-hex) — the vertex configuration 3.6.3.6.
 *
 * This file implements the tiling as TWO INDEPENDENT RASTERIZERS that both
 * scan every screen pixel. The hex rasterizer runs first (PAIR_HEX, cyan),
 * then the triangle rasterizer overwrites some pixels (PAIR_TRI, green).
 * The cursor hex is highlighted during the hex pass (PAIR_CURSOR), then the
 * '@' is drawn last on top of everything.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The hex and triangular grids are DUALS of each other in a specific sense:
 *   - Each hexagon center is surrounded by 6 triangle vertices
 *   - Each triangle vertex is shared by 6 triangles AND 6 hexagons (in full)
 *   - The "dual" (Voronoi/Delaunay) relationship: if you place a point at
 *     every hex center and connect adjacent ones, you get a triangular lattice.
 *     Vice versa: connecting triangle centers gives a hex lattice.
 *
 * When both grids are drawn together at the SAME SIZE (tri_size = hex_size),
 * their vertices coincide. The shared vertices are where the edges of both
 * grids meet, creating the star-of-David "snowflake" pattern.
 *
 * Use the 'h' and 't' keys to toggle layers:
 *   h only: pure flat-top hex grid   (see 01_flat_top)
 *   t only: pure triangular grid     (see 05_triangular, but same hex size)
 *   both:   the combined trihexagonal tiling
 *
 * DRAWING METHOD  (two-pass rendering)
 * ─────────────────────────────────────
 *  PASS 1 — hex_layer (if show_hex):
 *   Same pipeline as 01_flat_top grid_draw.
 *   Cursor hex border drawn in PAIR_CURSOR.
 *   Non-cursor borders drawn in PAIR_HEX (cyan).
 *
 *  PASS 2 — tri_layer (if show_tri):
 *   Same pipeline as 05_triangular grid_draw, but centered with ox/oy.
 *   tri_size = hex_size so triangle vertices align with hex vertices.
 *   Drawn AFTER hex_layer → triangle edges overwrite hex edges at crossings.
 *   Drawn without cursor awareness (cursor is hex-cell-based, not tri-based).
 *
 *  PASS 3 — cursor_draw:
 *   Draw '@' at the flat-top forward matrix position of (cQ,cR).
 *   Always drawn last so '@' is on top of both grid layers.
 *
 * KEY FORMULAS
 * ────────────
 *  WHY tri_size = hex_size for vertex alignment:
 *    A flat-top hex of side s has vertices at angles 0°,60°,...,300°,
 *    each at distance s from the center.
 *    An equilateral triangle of side s has edge length s.
 *    The triangular grid's stripe period h = s × √3/2 = the hex apothem.
 *    When both grids are drawn with the same s, their vertices coincide
 *    because the triangle vertices lie exactly on the hex vertex lattice.
 *
 *  Draw-order conflict at shared edge midpoints:
 *    Hex edges and triangle edges both pass through the midpoints of
 *    hex sides. These pixels are drawn by BOTH passes. The tri_layer
 *    (drawn second) wins → these midpoints show green not cyan.
 *    This is intentional — the combined pattern shows the triangle edges
 *    "cutting through" the hex edges.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The cursor uses hex coordinates (Q, R). If show_hex is OFF, the cursor
 *    still moves through hex space (invisible hex lattice). The '@' and
 *    cursor highlighting disappear when hex layer is off, but movement still
 *    works — cursor_draw is still called regardless of show_hex.
 *    (Actually: '@' is always drawn; only hex borders are gated by show_hex.)
 *
 *  • When both layers are off (h=OFF, t=OFF), only '@' is visible at the
 *    cursor hex center — the grid is blank.
 *
 *  • Triangular stripe alignment: the tri_layer uses px/py centered the same
 *    way as hex_layer (both subtract ox/oy). This keeps both grids centered on
 *    the same origin hex, ensuring vertex coincidence at origin.
 *
 * HOW TO VERIFY  (HEX_SIZE=14, 80×24 terminal)
 * ─────────────
 *  Origin hex (Q=0, R=0): hex center at (ox=40, oy=11). '@' at (11,40). ✓
 *
 *  A hex vertex at math 0° from center is at pixel (14, 0):
 *    col = 40 + 14/2 = 47.  That column should show a triangle-grid line.
 *    Triangular n2 = (√3×14 + 0) / (14×√3/2) = (14√3)/(7√3) = 2.0 → integer!
 *    edge_frac(2.0) = 0 → on a triangle border. ✓ The hex vertex and the
 *    triangle grid line coincide at column 47. Vertex alignment confirmed.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ────────────────────────────────────────────────────────── */

#define CELL_W              2
#define CELL_H              4

/* Hex side length in pixels; triangle side = hex_size (they share vertices) */
#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* Border width for both layers */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35
#define BORDER_W_STEP       0.02

#define TICK_NS            16666667LL

/* ── §2 clock ─────────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ─────────────────────────────────────────────────────────── */

#define PAIR_HEX     1   /* cyan  — hex grid borders */
#define PAIR_TRI     2   /* green — triangle grid borders */
#define PAIR_CURSOR  3   /* cursor hex border + '@' character */
#define PAIR_HUD     4
#define PAIR_HINT    5

static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(PAIR_HEX,    COLOR_CYAN,  COLOR_BLACK);
    init_pair(PAIR_TRI,    COLOR_GREEN, COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLOR_CYAN,  COLOR_BLACK);
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * px = (col - ox) × CELL_W,  py = (row - oy) × CELL_H.
 * Grid centered on screen: ox = cols/2,  oy = (rows-1)/2.
 *
 * DUAL GRID RELATIONSHIP:
 *   If the hex grid has side length s, the triangular dual grid has:
 *   - Triangle vertices at the hex vertices (shared lattice)
 *   - Triangle side = s (same side length)
 *   - The triangle grid stripe period h = s × √3/2
 *
 *   The two grids together create the trihexagonal tiling because:
 *   - Every hex contributes 6 boundary edges
 *   - Every triangle contributes 3 boundary edges
 *   - At each vertex, exactly 2 triangles and 2 hexes meet (config 3.6.3.6)
 *
 *   Constructing the triangle grid from the hex grid:
 *   The triangular lattice has the SAME √3/2 inter-line spacing as the hex
 *   edge-to-center distance, just with a different phase offset. For vertex
 *   alignment, the triangle stripe origin must match the hex vertex lattice.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * MENTAL MODEL — dual grids and the trihexagonal tiling:
 *
 *   Imagine painting each hexagon center red and each triangle center blue.
 *   - Red points form the HEX LATTICE (the centers of the triangular tiling)
 *   - Blue points form the TRIANGLE LATTICE (the centers of the hex tiling)
 *   These two lattices are DUALS of each other: the Voronoi diagram of one
 *   is the Delaunay triangulation of the other.
 *
 *   The trihexagonal tiling sits "between" these two lattices. Its edges
 *   run through the midpoints of both hex edges and triangle edges, creating
 *   the alternating triangle-hexagon-triangle-hexagon vertex configuration.
 *
 *   Visual experiment with 'h' and 't' keys:
 *     - h only: pure hex grid (01_flat_top pattern)
 *     - t only: pure triangular grid (05_triangular pattern, finer cells)
 *     - both:   the trihexagonal overlay — star-of-David / snowflake pattern
 *
 *   Why triangle stripe period = hex_size × √3/2:
 *     Adjacent hex vertices are s apart (hex side). The triangular grid
 *     whose vertices coincide has triangle side = s. The stripe period
 *     (perpendicular distance between parallel lines) = s × √3/2.
 *     This is the SAME as the hex apothem (center-to-edge distance).
 */

/*
 * angle_char — same as 01_flat_top. See that file for documentation.
 */
static char angle_char(double theta) {
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/*
 * edge_frac — same as 05_triangular. See that file for documentation.
 *   0 = on a grid line, 0.5 = farthest from any line.
 */
static double edge_frac(double v) {
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * hex_layer — draw flat-top hex grid borders.
 *
 * THE FORMULA (per screen cell):
 *   Identical to 01_flat_top grid_draw.
 *   Cursor hex (cQ, cR) drawn in PAIR_CURSOR instead of PAIR_HEX.
 *
 * DRAW ORDER NOTE:
 *   hex_layer is drawn FIRST. tri_layer (drawn after) will overwrite some
 *   hex border cells where the two grids share edge-midpoint pixels.
 *   The cursor hex borders drawn in PAIR_CURSOR survive only in regions
 *   not overwritten by the triangle grid.
 */
static void hex_layer(int rows, int cols, double size, double border_w,
                       int cQ, int cR, int ox, int oy) {
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - border_w;

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_HEX)    | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * tri_layer — draw the triangular grid borders in PAIR_TRI.
 *
 * THE FORMULA (per screen cell):
 *   Same three-family stripe classifier as 05_triangular grid_draw.
 *   tri_size = hex_size so triangle vertices coincide with hex vertices.
 *
 *   h = hex_size × √3/2
 *   n1 = py / h              → family 1: horizontal lines at py = k·h
 *   n2 = (√3·px + py) / h   → family 2: −60° lines
 *   n3 = (−√3·px + py) / h  → family 3: +60° lines
 *   dmin = min(edge_frac(n1), edge_frac(n2), edge_frac(n3))
 *   if dmin < border_w: draw '-', '/', or '\' in PAIR_TRI
 *
 * DRAWN AFTER hex_layer — triangle edges overwrite hex edges at crossings,
 * making the triangle edges appear "dominant" at shared boundary points.
 */
static void tri_layer(int rows, int cols, double hex_size, double border_w,
                       int ox, int oy) {
    /* Triangle side = hex_size; stripe period h = hex_size × √3/2 */
    double h   = hex_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    attron(COLOR_PAIR(PAIR_TRI) | A_BOLD);
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < border_w) {
                mvaddch(row, col, (chtype)(unsigned char)ch);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_TRI) | A_BOLD);
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * HEX_DIR — 4-direction movement in axial (Q, R) space.
 *
 * The cursor navigates the hex grid (not the triangle grid — the triangles
 * are just a visual overlay). Same direction table as 01_flat_top:
 *
 *                 UP: (Q=0, R=-1)
 *                       ↑
 *   LEFT: (Q=-1, R=0) ← ● → RIGHT: (Q=+1, R=0)
 *                       ↓
 *                DOWN: (Q=0, R=+1)
 *
 * Effect: the cursor hex is highlighted in PAIR_CURSOR on the hex layer.
 * The triangle layer overwrites most border cells of the cursor hex, but
 * the cursor '@' and some hex border pixels remain visible in blue.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

/*
 * cursor_draw — place '@' at the center cell of flat-top hex (cQ, cR).
 *
 * THE FORMULA (flat-top forward matrix, same as 01_flat_top):
 *   cx_pix = size × 3/2 × cQ
 *   cy_pix = size × (√3/2 × cQ  +  √3 × cR)
 *   col = ox + (int)(cx_pix / CELL_W)
 *   row = oy + (int)(cy_pix / CELL_H)
 *
 * Called after BOTH layers so '@' is always on top and fully visible,
 * regardless of which layers are toggled. The hex center pixel is interior
 * to both grids, so both hex_layer and tri_layer skip it (they only draw
 * borders) — '@' lands on an empty cell.
 */
static void cursor_draw(double size, int cQ, int cR,
                         int ox, int oy, int rows, int cols) {
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = size * 1.5      * (double)cQ;
    double cy_pix = size * (sq3_2   * (double)cQ + sq3 * (double)cR);
    int col = ox + (int)(cx_pix / CELL_W);
    int row = oy + (int)(cy_pix / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ─────────────────────────────────────────────────────────── */

typedef struct {
    double hex_size, border_w;
    int    cQ, cR;
    int    show_hex, show_tri;
    int    paused;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size = HEX_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->show_hex = 1;
    s->show_tri = 1;
    s->paused   = 0;
}
static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    if (s->show_hex) hex_layer(rows, cols, s->hex_size, s->border_w,
                                s->cQ, s->cR, ox, oy);
    if (s->show_tri) tri_layer(rows, cols, s->hex_size, s->border_w, ox, oy);
    cursor_draw(s->hex_size, s->cQ, s->cR, ox, oy, rows, cols);
}

/* ── §7 screen ────────────────────────────────────────────────────────── */

static void screen_init(void) {
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
}
static void screen_draw_hud(const Scene *s, int rows, int cols, double fps) {
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  hex:%s  tri:%s  %5.1f fps  %s ",
             s->cQ, s->cR, s->hex_size,
             s->show_hex ? "ON " : "OFF",
             s->show_tri ? "ON " : "OFF",
             fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  h:hex  t:tri  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }
static void cleanup(void) { endwin(); }

/* ── §8 app ───────────────────────────────────────────────────────────── */

static volatile sig_atomic_t running = 1, need_resize = 0;
static void sig_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) running = 0;
    if (sig == SIGWINCH) need_resize = 1;
}

int main(void) {
    atexit(cleanup);
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    signal(SIGWINCH, sig_handler);
    screen_init();
    int rows, cols; getmaxyx(stdscr, rows, cols);
    Scene sc; scene_init(&sc); color_init();
    int64_t prev = clock_ns(); double fps = 60.0;

    while (running) {
        if (need_resize) {
            need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: running = 0; break;
            case 'p': sc.paused ^= 1; break;
            case 'h': sc.show_hex ^= 1; break;
            case 't': sc.show_tri ^= 1; break;
            case 'r': sc.cQ = 0; sc.cR = 0; break;
            case KEY_UP:    sc.cQ += HEX_DIR[0][0]; sc.cR += HEX_DIR[0][1]; break;
            case KEY_DOWN:  sc.cQ += HEX_DIR[1][0]; sc.cR += HEX_DIR[1][1]; break;
            case KEY_LEFT:  sc.cQ += HEX_DIR[2][0]; sc.cR += HEX_DIR[2][1]; break;
            case KEY_RIGHT: sc.cQ += HEX_DIR[3][0]; sc.cR += HEX_DIR[3][1]; break;
            case '+': case '=':
                if (sc.hex_size < HEX_SIZE_MAX) { sc.hex_size += HEX_SIZE_STEP; } break;
            case '-':
                if (sc.hex_size > HEX_SIZE_MIN) { sc.hex_size -= HEX_SIZE_STEP; } break;
            case '[':
                if (sc.border_w > BORDER_W_MIN) { sc.border_w -= BORDER_W_STEP; } break;
            case ']':
                if (sc.border_w < BORDER_W_MAX) { sc.border_w += BORDER_W_STEP; } break;
            }
        }
        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0) fps = fps * 0.9 + 1e9 / (double)dt * 0.1;
        erase();
        scene_draw(&sc, rows, cols);
        screen_draw_hud(&sc, rows, cols, fps);
        screen_present();
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
