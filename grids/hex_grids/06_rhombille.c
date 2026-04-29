/*
 * 06_rhombille.c — rhombille tiling: hexagons divided into 3 rhombuses each
 *
 * DEMO: Each hexagon is split into three rhombuses by drawing "spoke" lines
 *       from the hex center to every other vertex (at 0°, 120°, 240°).
 *       The result looks like an isometric view of 3D cubes stacked in a
 *       corner — the classic "cube illusion" from the rhombille tiling.
 *       An '@' cursor moves between hex cells with arrow keys.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (hex border algorithm)
 *                  grids/hex_grids/05_triangular.c (triangular dual of rhombille)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs, 4 themes
 *   §4 coords   — pixel↔cell notes, spoke directions
 *   §5 draw     — hex borders + 3-spoke interior
 *   §5b cursor  — movement vectors, cursor_draw
 *   §6 scene    — state struct + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:spoke
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 06_rhombille.c -o 06_rhombille -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Rhombille tiling = flat-top hex grid + 3 "spoke" line
 *                  segments per hex. Each spoke goes from the hex center to
 *                  one of 3 alternating vertices (at math angles 0°, 120°,
 *                  240°). This divides each hex into 3 congruent rhombuses.
 *                  Together, adjacent rhombuses from neighboring hexes create
 *                  the illusion of 3D isometric cubes.
 *
 * Data-structure : Two-level classification per cell:
 *                  Level 1: compute cube dist. If dist > 0.5-border_w → hex border.
 *                  Level 2: if interior, check distance from each of 3 spokes.
 *                    Spoke k has direction d_k = (cos θ_k_screen, sin θ_k_screen).
 *                    Project (px-cx, py-cy) onto d_k → along-spoke t and perp dist.
 *                    If 0 ≤ t ≤ size and perp ≤ SPOKE_W_PX → spoke border.
 *
 * Rendering      : Spoke character = angle_char(θ_k_screen) — the direction of
 *                  the spoke line itself (NOT +π/2 as for hex borders, because
 *                  we want the line character aligned with the spoke direction).
 *                  Spoke at screen 0° → '-'; 120° → '/'; 240° → '\'.
 *                  Hex border: angle_char(theta + π/2) as in 01_flat_top.
 *
 *                  Why alternating vertices (not all 6): connecting center to
 *                  all 6 vertices gives 6 triangles (hexagonal star). Using
 *                  every other vertex gives 3 rhombuses (the rhombille pattern).
 *                  The 3 spoke directions are exactly 120° apart, matching the
 *                  3 cube face orientations in isometric projection.
 *
 * Performance    : O(rows × cols × 3) per frame. The spoke check adds 3 dot
 *                  products per interior cell — still O(rows×cols) overall.
 *
 * References     :
 *   Rhombille tiling (Wikipedia)
 *     https://en.wikipedia.org/wiki/Rhombille_tiling
 *   Isometric projection geometry
 *     https://en.wikipedia.org/wiki/Isometric_projection
 *   Red Blob Games hex grid guide
 *     https://www.redblobgames.com/grids/hexagons/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Rhombille = flat-top hexagonal grid + 3 "spoke" segments inside each hex.
 * The hex grid provides the outer structure (same as 01_flat_top). Inside
 * each hex, three lines emanate from the center to alternating vertices
 * (0°, 120°, 240°). This creates the distinctive diamond/rhombus pattern
 * that gives the tiling its name.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Every pixel is classified in two levels:
 *
 *   LEVEL 1 — Hex border check (same as 01_flat_top):
 *     cube dist ≥ 0.5 − border_w?  → hex border character
 *
 *   LEVEL 2 — Spoke check (interior pixels only):
 *     For each of 3 spokes, compute:
 *       t    = projection of (pixel−center) onto spoke direction
 *       perp = perpendicular distance from (pixel−center) to spoke line
 *     If 0 ≤ t ≤ hex_size AND perp ≤ spoke_w → draw spoke character
 *
 * The two levels give four possible outcomes for any pixel:
 *   hex border + no spoke → border character (angle_char)
 *   interior + on spoke   → spoke character ('-', '/', '\')
 *   interior + no spoke   → empty (skip)
 *   (hex border + on spoke: border wins by being checked first)
 *
 * WHY ALTERNATING VERTICES:
 *   Flat-top hex has vertices at math angles 0°, 60°, 120°, 180°, 240°, 300°.
 *   Connecting center to all 6 → 6 triangles (star of David / hexagonal star).
 *   Connecting center to 3 alternating (0°, 120°, 240°) → 3 rhombuses.
 *   The 3-spoke version is the rhombille tiling and creates the cube illusion.
 *   The 6-spoke version would be the hexagonal star tiling.
 *
 * DRAWING METHOD  (hex border + spoke projection)
 * ─────────────────────────────────────────────────
 *  1. Flat-top hex pipeline: pixel → cube → nearest hex (Q,R) + cube dist.
 *
 *  2. If dist ≥ limit: draw hex border character (same as 01_flat_top).
 *
 *  3. Else (interior pixel):
 *     Compute vector v = (px − cx, py − cy) from hex center.
 *     For k = 0, 1, 2:
 *       t    = vx × SPOKE_D[k][0] + vy × SPOKE_D[k][1]   (dot product)
 *       perp = |vx × SPOKE_D[k][1] − vy × SPOKE_D[k][0]|  (cross magnitude)
 *       If 0 ≤ t ≤ size AND perp ≤ spoke_w: draw SPOKE_C[k]; break.
 *
 * KEY FORMULAS
 * ────────────
 *  Spoke direction k in screen space (y-down):
 *    Math angles 0°, 120°, 240° → screen directions:
 *    d[k] = (cos(math_angle_k), −sin(math_angle_k))
 *    k=0: (1.0,  0.0)         → screen right → '-'
 *    k=1: (−0.5, −√3/2)      → screen upper-left → '\'
 *    k=2: (−0.5, +√3/2)      → screen lower-left → '/'
 *
 *  Sign flip on sin: screen y increases downward (opposite to math convention).
 *    cos(120°) = −0.5,  −sin(120°) = −√3/2 ≈ −0.866
 *    cos(240°) = −0.5,  −sin(240°) = +√3/2 ≈ +0.866
 *
 *  Along-spoke projection (dot product):
 *    t = vx × dx + vy × dy
 *    t ∈ [0, size] → pixel is "along" the spoke (not behind center, not past vertex)
 *
 *  Perpendicular distance (2D cross product magnitude):
 *    perp = |vx × dy − vy × dx|
 *    perp ≤ spoke_w → pixel is within spoke_w pixels of the spoke line
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Spoke check must use t >= 0 (spoke starts at center) AND t <= size
 *    (spoke ends at vertex). Without t <= size, spokes would extend to
 *    the next hex and produce incorrect patterns.
 *
 *  • The 3 spoke characters are pre-assigned by direction (SPOKE_C[3]):
 *    k=0 ('-'), k=1 ('\'), k=2 ('/'). This is the SPOKE direction angle,
 *    NOT the tangent angle. Spokes are straight lines, not hex-tangent curves.
 *
 *  • cursor hex: both its border AND its spokes are drawn in PAIR_CURSOR.
 *    This gives the cursor hex a fully blue-highlighted appearance.
 *
 * HOW TO VERIFY  (HEX_SIZE=14, spoke_w=2.5, cursor at Q=0, R=0)
 * ─────────────
 *  Spoke 0 goes from center (0,0) rightward to vertex (14, 0).
 *  Pixel at (col=ox+4, row=oy): px=8, py=0, vx=8, vy=0.
 *    t    = 8×1.0 + 0×0.0 = 8. 0 ≤ 8 ≤ 14 ✓
 *    perp = |8×0.0 − 0×1.0| = 0 ≤ 2.5 ✓ → draws '-'. ✓
 *
 *  Pixel at (col=ox+4, row=oy+1): px=8, py=4, vx=8, vy=4.
 *    t    = 8×1.0 + 4×0.0 = 8.
 *    perp = |8×0.0 − 4×1.0| = 4 > 2.5 → NOT on spoke 0.
 *    Check spoke 1: d=(−0.5, −0.866).
 *    t    = 8×(−0.5) + 4×(−0.866) = −4−3.46 = −7.46 < 0 → not on spoke 1.
 *    → Interior, no spoke → nothing drawn. ✓
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

#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.30
#define BORDER_W_STEP       0.02

/* Spoke half-width in pixels. 1 terminal column = CELL_W=2 pixels. */
#define SPOKE_W_DEFAULT     2.5
#define SPOKE_W_MIN         0.5
#define SPOKE_W_MAX         6.0
#define SPOKE_W_STEP        0.5

#define N_THEMES            4
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

#define PAIR_BORDER  1
#define PAIR_SPOKE   2   /* interior spokes — slightly dimmer for depth */
#define PAIR_CURSOR  3   /* cursor hex border + '@' character */
#define PAIR_HUD     4
#define PAIR_HINT    5

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};
static void color_init(int theme) {
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_SPOKE,  THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLOR_BLACK,      COLOR_CYAN);
    init_pair(PAIR_HINT,   COLOR_CYAN,       COLOR_BLACK);
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * px = (col - ox) × CELL_W,  py = (row - oy) × CELL_H.
 * Grid centered on screen: ox = cols/2,  oy = (rows-1)/2.
 * Flat-top hex cube coords and center formula — see 01_flat_top.c §4.
 *
 * Spoke directions (screen space, y increases downward):
 *   Math 0° vertex (right):         screen dir = ( 1.0,    0.0  ) → '-'
 *   Math 120° vertex (upper-left):  screen dir = (−0.5, −√3/2  ) → '\'
 *   Math 240° vertex (lower-left):  screen dir = (−0.5, +√3/2  ) → '/'
 * The screen y-component is −sin(math_angle) because y is flipped.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * MENTAL MODEL — rhombille = hex + 3 spokes:
 *
 *   Start with a flat-top hex. It has 6 vertices at angles 0°,60°,120°,
 *   180°,240°,300° from the center. Draw lines from center to the ODD-
 *   numbered vertices (0°, 120°, 240°). This creates 3 equal rhombuses:
 *
 *        * ← vertex at 60° (not connected)
 *       / \
 *      /   \
 *     *--C--*   C = center; spokes go to left(*), upper-right(*), lower-right(*)
 *      \   /   (the 3 ODD vertices at 0°,120°,240° in this diagram)
 *       \ /
 *        *
 *
 *   Each pair of adjacent hexes shares a rhombus boundary along their
 *   shared edge, creating the seamless rhombille tiling.
 *
 *   The 3 spoke directions at 120° apart match the 3 axis directions of
 *   isometric projection. If you shade the 3 rhombuses differently (top,
 *   left, right face), you see a 3D cube floating in space.
 *
 *   In ASCII: we can't shade, but we can vary the character per spoke:
 *     '-' for horizontal top-face boundary
 *     '\' for upper-left face boundary
 *     '/' for lower-left face boundary
 *
 *   For detecting if a cell is on spoke k:
 *     1. Compute vector v = (px - cx, py - cy) from hex center to cell
 *     2. Project onto spoke direction d_k: t = v · d_k
 *     3. Perpendicular distance: perp = |v × d_k|  (2D cross magnitude)
 *     4. On spoke if: 0 ≤ t ≤ hex_size  AND  perp ≤ spoke_w
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
 * SPOKE_D — three spoke directions in screen space (y-down):
 *   d[k] = (cos(math_angle_k), −sin(math_angle_k))
 *   k=0: math 0°   → (1, 0)         → '-'
 *   k=1: math 120° → (−0.5, −√3/2)  → '\'
 *   k=2: math 240° → (−0.5, +√3/2)  → '/'
 *
 * The −sin is needed because screen y increases downward (opposite math).
 */
static const double SPOKE_D[3][2] = {
    {  1.0,        0.0       },  /* math 0°   → screen right     → '-' */
    { -0.5,       -0.8660254 },  /* math 120° → screen upper-left → '\' */
    { -0.5,        0.8660254 },  /* math 240° → screen lower-left → '/' */
};
static const char SPOKE_C[3] = { '-', '\\', '/' };

/*
 * grid_draw — rasterize rhombille: hex borders + interior spokes.
 *
 * THE PIPELINE (per screen cell):
 *
 *   pixel → cube → nearest hex (Q,R) + cube dist  [same as 01_flat_top]
 *      │
 *      ├── dist ≥ limit (hex border):
 *      │     angle_char(theta+π/2) in PAIR_CURSOR or PAIR_BORDER
 *      │
 *      └── dist < limit (interior):
 *            v = (px−cx, py−cy)   ← vector from hex center to pixel
 *            For k = 0..2:
 *              t    = v · SPOKE_D[k]           ← along-spoke distance
 *              perp = |v × SPOKE_D[k]|         ← perpendicular distance
 *              if 0 ≤ t ≤ size AND perp ≤ spoke_w:
 *                draw SPOKE_C[k] in PAIR_CURSOR (on_cur) or PAIR_SPOKE
 *                break  ← only one spoke character per pixel
 */
static void grid_draw(int rows, int cols,
                       double size, double border_w, double spoke_w,
                       int cQ, int cR, int ox, int oy) {
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - border_w;

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            /* Flat-top fractional cube coords */
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            /* cube_round */
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

            /* Hex center in pixel space */
            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);

            int on_cur = (Q == cQ && R == cR);

            if (dist >= limit) {
                /* Hex border — cursor hex uses PAIR_CURSOR */
                double theta = atan2(py - cy, px - cx);
                char ch = angle_char(theta + M_PI / 2.0);
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else {
                /* Interior: check 3 spokes */
                double vx = px - cx, vy = py - cy;
                for (int k = 0; k < 3; k++) {
                    double dx = SPOKE_D[k][0], dy = SPOKE_D[k][1];
                    double t    = vx * dx + vy * dy;
                    double perp = fabs(vx * dy - vy * dx);
                    if (t >= 0.0 && t <= size && perp <= spoke_w) {
                        int attr = on_cur ? COLOR_PAIR(PAIR_CURSOR)
                                          : COLOR_PAIR(PAIR_SPOKE);
                        attron(attr);
                        mvaddch(row, col, (chtype)(unsigned char)SPOKE_C[k]);
                        attroff(attr);
                        break;
                    }
                }
            }
        }
    }
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * HEX_DIR — 4-direction movement in axial (Q, R) space.
 *
 * The rhombille tiling uses the same underlying flat-top hex grid as
 * 01_flat_top, so the same axial direction table applies.
 *
 *                 UP: (Q=0, R=-1)
 *                       ↑
 *   LEFT: (Q=-1, R=0) ← ● → RIGHT: (Q=+1, R=0)
 *                       ↓
 *                DOWN: (Q=0, R=+1)
 *
 * Moving the cursor shifts the cube illusion's "anchor" hex — the
 * 3-rhombus pattern is identical in every hex, so only the highlight
 * moves, not the underlying tiling geometry.
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
 * THE FORMULA (flat-top forward matrix):
 *   cx_pix = size × 3/2 × cQ
 *   cy_pix = size × (√3/2 × cQ  +  √3 × cR)
 *   col = ox + (int)(cx_pix / CELL_W)
 *   row = oy + (int)(cy_pix / CELL_H)
 *
 * Same as 01_flat_top cursor_draw — the rhombille tiling uses the same
 * flat-top hex lattice for cell centers.
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
    double hex_size, border_w, spoke_w;
    int    cQ, cR;
    int    theme, paused;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size = HEX_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->spoke_w  = SPOKE_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->theme = 0; s->paused = 0;
}
static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->hex_size, s->border_w, s->spoke_w,
              s->cQ, s->cR, ox, oy);
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
             " Q:%+d R:%+d  size:%.0f  spoke:%.1f  theme:%d  %5.1f fps  %s ",
             s->cQ, s->cR, s->hex_size, s->spoke_w, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:spoke ");
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
    Scene sc; scene_init(&sc); color_init(sc.theme);
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
            case 't': sc.theme = (sc.theme + 1) % N_THEMES; color_init(sc.theme); break;
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
                if (sc.spoke_w > SPOKE_W_MIN) { sc.spoke_w -= SPOKE_W_STEP; } break;
            case ']':
                if (sc.spoke_w < SPOKE_W_MAX) { sc.spoke_w += SPOKE_W_STEP; } break;
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
