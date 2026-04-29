/*
 * 01_flat_top.c — flat-top hexagonal grid with keyboard-controlled cursor
 *
 * DEMO: Fills the screen with flat-top hexagons. A '@' cursor starts at the
 *       origin hex and moves between adjacent hexes with arrow keys — the
 *       current hex border glows white-on-blue. Resize hexes with +/-.
 *
 * Study alongside: grids/hex_grids/02_pointy_top.c (same cursor logic,
 *                  different transform matrix)
 *
 * Section map:
 *   §1 config      — tunable constants
 *   §2 clock       — monotonic timer + sleep
 *   §3 color       — ncurses color pairs
 *   §4 coords      — pixel↔cell conversion, centering offset
 *   §5 draw        — hex rasterizer: cube coords + border detection
 *   §5b cursor     — movement vectors, cursor_draw
 *   §6 scene       — state struct + scene_draw
 *   §7 screen      — ncurses display layer
 *   §8 app         — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 01_flat_top.c -o 01_flat_top -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Flat-top hex tiling via cube coordinates (Q+R+S=0).
 *                  pixel→fractional cube via flat-top inverse matrix,
 *                  cube_round (fix-largest-error) → nearest hex (Q,R),
 *                  cube dist = max(|fq-Q|,|fr-R|,|fs-S|) → border/interior.
 *
 * Cursor movement : Stored as axial (cQ, cR). Arrow keys apply deltas from
 *                  HEX_DIR[4][2] — 4 of the 6 hex faces. See §5b.
 *
 * Rendering      : Grid centered on screen: pixel origin = screen center.
 *                  Border char = angle_char(theta + π/2). Cursor hex border
 *                  drawn in PAIR_CURSOR; '@' drawn at hex centroid cell.
 *
 * References     :
 *   Red Blob Games hex guide — https://www.redblobgames.com/grids/hexagons/
 *   Cube rounding            — https://www.redblobgames.com/grids/hexagons/#rounding
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every screen pixel belongs to exactly one hexagon. The cube coordinate
 * system (Q+R+S=0) embeds a 2-D hex plane in 3-D space — but the constraint
 * Q+R+S=0 keeps us on a diagonal plane. Within this system, "which hex am
 * I closest to?" reduces to: round the fractional cube coordinates to the
 * nearest integers, with one correction step to restore Q+R+S=0. The cube
 * distance formula then tells us how close to the hex center we are, which
 * determines border vs. interior.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine coloring every hexagon a different color and asking: "for a given
 * pixel, which color does it inherit?" The answer is the color of the nearest
 * hex center. The boundary between two hexes is the set of pixels equidistant
 * from both centers — i.e., where cube distance = 0.5. Everything closer than
 * 0.5 − border_w is interior (invisible); everything between 0.5 − border_w
 * and 0.5 is "border" and gets a line character.
 *
 * You never store a hex grid array. You never loop over hexes. You ask the
 * formula at every pixel, per frame. No data structure, just arithmetic.
 *
 * DRAWING METHOD  (raster-scan pipeline)
 * ──────────────────────────────────────
 *  1. Center the grid: ox = cols/2, oy = (rows-1)/2.
 *     Pixel: px = (col-ox)*CELL_W, py = (row-oy)*CELL_H.
 *     Now hex (Q=0,R=0) maps to pixel (0,0) = screen center.
 *
 *  2. Flat-top inverse matrix — pixel → fractional cube:
 *       fq =  (2/3 * px) / size
 *       fr = (-px/3 + √3*py/3) / size
 *       fs = -fq - fr
 *
 *  3. cube_round — round all three, restore Q+R+S=0 by fixing the
 *     component with the largest rounding error:
 *       if dq is largest: Q = -rr - rs
 *       if dr is largest: R = -rq - rs
 *       otherwise:        S = -rq - rr   (S not stored; implicit)
 *
 *  4. Cube distance — how far is the pixel from hex (Q,R)?
 *       dist = max(|fq-Q|, |fr-R|, |fs-S|)
 *     0 = exact center, 0.5 = edge midpoint, 2/3 = vertex.
 *
 *  5. Threshold: dist < (0.5 - border_w) → interior → skip.
 *
 *  6. For border pixels: compute the hex center (flat-top forward matrix):
 *       cx = size * 3/2 * Q
 *       cy = size * (√3/2 * Q + √3 * R)
 *     Then the radial angle: theta = atan2(py-cy, px-cx).
 *
 *  7. Pick a line character: angle_char(theta + π/2).
 *     The +π/2 rotates from radial to tangent direction.
 *
 *  8. Draw in cursor color (Q==cQ && R==cR) or border color.
 *
 * KEY FORMULAS
 * ────────────
 *  Flat-top inverse matrix (pixel → fractional cube):
 *    fq =  2/3 * px / size
 *    fr = (-1/3 * px + √3/3 * py) / size
 *
 *  Flat-top forward matrix (hex → pixel center):
 *    cx = size * 3/2 * Q
 *    cy = size * (√3/2 * Q  +  √3 * R)
 *
 *  Pixel → terminal cell (with centering):
 *    col = ox + (int)(cx / CELL_W)
 *    row = oy + (int)(cy / CELL_H)
 *
 *  Cube distance (border detection):
 *    dist = max(|fq-Q|, |fr-R|, |fs-S|)
 *    interior  if dist < 0.5 - border_w
 *    border    if dist ≥ 0.5 - border_w
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • cube_round: rounding three values independently can break Q+R+S=0.
 *    Always fix the component with the LARGEST error — that component was
 *    rounded away from its "true" value most, so enforcing the constraint
 *    via it keeps the other two more accurate.
 *
 *  • CELL_H / CELL_W = 4/2 = 2. Terminal characters are ~2× taller than
 *    wide in pixels. CELL_W=2, CELL_H=4 compensates so one pixel step is
 *    the same distance in both axes when projected to screen.
 *
 *  • ox = cols/2 is integer division. At odd column counts, the origin is
 *    shifted half a cell left. This is fine — the grid tiles infinitely.
 *
 *  • Last row (row == rows-1) is reserved for the HUD. Raster scan stops
 *    at row < rows-1.
 *
 *  • Resize: ox/oy recompute each frame from the current rows/cols; the
 *    grid recenters automatically. cQ/cR remain valid (hex coords are
 *    independent of terminal size).
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14, CELL_W=2, CELL_H=4)
 * ─────────────
 *  Screen center: ox=40, oy=11.
 *  Origin hex (Q=0,R=0): cx=0, cy=0 → col=40, row=11. Cursor '@' at (11,40).
 *
 *  Pixel at cell (col=40, row=11): px=0, py=0 → fq=0, fr=0 → Q=0,R=0, dist=0.
 *  → interior pixel (dist < 0.4). Skipped — no character drawn there.
 *
 *  Pixel at cell (col=40, row=9): py=(9-11)*4=-8.
 *    fq=0, fr=(-8/3)/14 ≈ -0.19, dist≈0.19 < 0.40 → interior, skip.
 *
 *  Pixel at cell (col=40, row=8): py=(8-11)*4=-12.
 *    fr=(-12/3)/14 ≈ -0.286, dist≈0.286 < 0.40 → interior, skip.
 *
 *  Pixel at cell (col=40, row=7): py=-16.
 *    fr=(-16/3)/14 ≈ -0.381.  dist≈0.381 < 0.40 → interior, skip.
 *
 *  Pixel at cell (col=40, row=6): py=-20.
 *    fr ≈ -0.476.  dist≈0.476 ≥ 0.40 → BORDER. cy=0, cx=0.
 *    theta=atan2(-20-0, 0-0)=atan2(-20,0)=−π/2.
 *    angle_char(−π/2 + π/2) = angle_char(0) = '-'. ✓ Top of hex is horizontal.
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

#define CELL_W             2
#define CELL_H             4

#define HEX_SIZE_DEFAULT  14.0
#define HEX_SIZE_MIN       6.0
#define HEX_SIZE_MAX      40.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.10
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_THEMES           4
#define TICK_NS           16666667LL

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

#define PAIR_BORDER   1
#define PAIR_CURSOR   2   /* cursor hex border + '@' character */
#define PAIR_HUD      3
#define PAIR_HINT     4

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
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLOR_BLACK,      COLOR_CYAN);
    init_pair(PAIR_HINT,   COLOR_CYAN,       COLOR_BLACK);
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * The grid is centered on screen. For terminal cell (col, row):
 *   px = (col - ox) × CELL_W      ← pixel relative to screen center
 *   py = (row - oy) × CELL_H
 * where  ox = cols/2,  oy = (rows-1)/2.
 * Hex (Q=0, R=0) center maps to pixel (0,0) = screen center.
 * CELL_H/CELL_W = 2 matches the ~2:1 terminal character aspect ratio,
 * so hexes appear circular rather than stretched vertically.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * angle_char — map a tangent angle to the best-fit ASCII line character.
 *
 * THE FORMULA (tangent-to-character mapping):
 *
 *   Input: theta = atan2(py-cy, px-cx) + π/2
 *
 *   WHY + π/2:
 *   atan2 gives the RADIAL angle from hex center to pixel.
 *   Adding π/2 converts it to the TANGENT angle — the direction the hex
 *   edge runs at that point. The character must align with the edge, not
 *   point toward the center.
 *
 *   The result is folded into [0, π) with fmod because ASCII characters
 *   are symmetric under 180° rotation ('-' looks the same upside-down).
 *
 *   After folding into [0°, 180°):
 *     [  0°,  22.5°) → '-'   near-horizontal  (flat-top hex top/bottom edges)
 *     [ 22.5°,  67.5°) → '\'  diagonal, \-slope (upper-left/lower-right edges)
 *     [ 67.5°, 112.5°) → '|'  near-vertical    (flat-top hex has no vertical edges
 *                                                but oblique segments get this)
 *     [112.5°, 157.5°) → '/'  diagonal, /-slope (lower-left/upper-right edges)
 *     [157.5°, 180°) → '-'   wraps back to horizontal
 *
 *   Flat-top hex edge tangent angles (measured from horizontal):
 *     Top edge:         0° → '-'
 *     Upper-right edge: 60° → '\'
 *     Lower-right edge: 120° → '/'
 *     Bottom edge:      180° → '-'  (same as 0° after fold)
 *     Lower-left edge:  240° → '\'  (same as 60° after fold)
 *     Upper-left edge:  300° → '/'  (same as 120° after fold)
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
 * grid_draw — rasterize the flat-top hex grid with cursor hex highlighted.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row)
 *      │
 *      ▼  Center the grid (origin hex → screen center)
 *   px = (col − ox) × CELL_W
 *   py = (row − oy) × CELL_H
 *      │
 *      ▼  Flat-top inverse matrix: pixel → fractional cube
 *   fq = (2/3 × px) / size
 *   fr = (−px/3 + √3·py/3) / size
 *   fs = −fq − fr
 *      │
 *      ▼  cube_round: round each axis, restore Q+R+S=0 by fixing
 *         the component that was rounded by the most
 *   (rq,rr,rs) = (round(fq), round(fr), round(fs))
 *   dq=|rq−fq|, dr=|rr−fr|, ds=|rs−fs|
 *   if dq largest: Q = −rr−rs
 *   if dr largest: R = −rq−rs
 *   else: Q=rq, R=rr  (S implicit from Q+R+S=0)
 *      │
 *      ▼  Cube distance: how far from hex (Q,R) center?
 *   fQ=Q, fR=R, fS=−Q−R
 *   dist = max(|fq−Q|, |fr−R|, |fs−S|)
 *      │
 *      ├── dist < (0.5 − border_w)  →  interior: skip (continue)
 *      │
 *      └── dist ≥ (0.5 − border_w)  →  border: draw character
 *            cx = size × 3/2 × Q         ← flat-top forward matrix (x)
 *            cy = size × (√3/2·Q + √3·R) ← flat-top forward matrix (y)
 *            theta = atan2(py−cy, px−cx)
 *            ch = angle_char(theta + π/2)
 *            color = PAIR_CURSOR if Q==cQ&&R==cR, else PAIR_BORDER
 *
 * WHY CUBE DISTANCE EQUALS 0.5 AT HEX EDGES:
 *   At the exact midpoint of the shared edge between hex (0,0) and hex (1,0),
 *   the fractional cube position is fq=0.5, fr=0, fs=-0.5.
 *   dist = max(|0.5-0|, |0-0|, |-0.5-0|) = 0.5. ✓
 *   Any pixel with dist ≥ 0.5-border_w is "close enough to an edge" → border.
 */
static void grid_draw(int rows, int cols, double size, double border_w,
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
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * HEX_DIR — movement vectors for arrow keys in axial (Q, R) space.
 *
 * The flat-top hex grid has 6 faces. Arrow keys cover 4 of them:
 *
 *                 UP: (Q=0, R=-1)
 *                       ↑
 *   LEFT: (Q=-1, R=0) ← ● → RIGHT: (Q=+1, R=0)
 *                       ↓
 *                DOWN: (Q=0, R=+1)
 *
 * Screen direction: RIGHT=east, LEFT=west, UP=upper-left, DOWN=lower-right.
 * The unmapped faces are NE (+1,-1) and SW (-1,+1) (diagonal in screen).
 * Same deltas apply for pointy-top (02), rhombille (06), trihex (07).
 *
 * WHY THESE DELTAS WORK:
 *   Flat-top forward matrix gives cx = 3/2·Q·s, cy = (√3/2·Q + √3·R)·s.
 *   Δ(Q=+1, R=0): Δcx = +3/2·s > 0 (moves right). ✓
 *   Δ(Q=0,  R=+1): Δcy = +√3·s > 0 (moves down in screen). ✓
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* 0 = UP    — R decreases */
    { 0, +1 },   /* 1 = DOWN  — R increases */
    {-1,  0 },   /* 2 = LEFT  — Q decreases */
    {+1,  0 },   /* 3 = RIGHT — Q increases */
};

/*
 * cursor_draw — place '@' at the center cell of hex (cQ, cR).
 *
 * THE FORMULA (flat-top forward matrix → terminal cell):
 *
 *   Pixel center of hex (cQ, cR):
 *     cx_pix = size × 3/2 × cQ
 *     cy_pix = size × (√3/2 × cQ  +  √3 × cR)
 *
 *   Terminal cell (with centering offset ox, oy):
 *     col = ox + (int)(cx_pix / CELL_W)
 *     row = oy + (int)(cy_pix / CELL_H)
 *
 * The integer truncation (not round) keeps '@' slightly inside the interior
 * rather than on the border character, so it is always visible.
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
    int    cQ, cR;    /* cursor hex — axial coordinates */
    int    theme, paused;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size = HEX_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->theme = 0; s->paused = 0;
}
static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->hex_size, s->border_w, s->cQ, s->cR, ox, oy);
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
             " Q:%+d R:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             s->cQ, s->cR, s->hex_size, s->border_w, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
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
            case 'r': sc.cQ = 0; sc.cR = 0; break;
            case 't':
                sc.theme = (sc.theme + 1) % N_THEMES;
                color_init(sc.theme);
                break;
            /* Arrow keys: apply HEX_DIR deltas to cursor */
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
