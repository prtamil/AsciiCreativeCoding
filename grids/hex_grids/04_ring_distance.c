/*
 * 04_ring_distance.c — hex ring-distance coloring with movable '@' cursor
 *
 * DEMO: Every hex is colored by its ring distance from the '@' cursor.
 *       Ring 0 = cursor (white); ring 1 = 6 neighbours (cyan); etc.
 *       Move the cursor with arrow keys and watch the color rings follow.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (same §5b cursor section)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ring color pairs
 *   §4 coords   — pixel↔cell, centering offset
 *   §5 draw     — ring-colored grid rasterizer
 *   §5b cursor  — movement vectors, cursor_draw
 *   §6 scene    — state struct + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 04_ring_distance.c -o 04_ring_distance -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Ring distance = max(|ΔQ|,|ΔR|,|ΔS|) = (|ΔQ|+|ΔR|+|ΔS|)/2.
 *                  Counts minimum hex steps between two hexes.
 *                  Ring k has exactly 6k hexes (k>0); ring 0 = 1 hex.
 *
 * Cursor movement : HEX_DIR[4][2] same as 01. Cursor moves in O(1); the
 *                  entire grid recolors instantly since distance is recomputed
 *                  per-cell each frame.
 *
 * Rendering      : Interior cells (dist < limit) filled with a colored space
 *                  so ring bands show background color, making the pattern vivid.
 *                  Border cells drawn with ring color + bold.
 *
 * References     :
 *   Hex distance — https://www.redblobgames.com/grids/hexagons/#distances
 *   Ring counts  — https://www.redblobgames.com/grids/hexagons/#rings
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * In a hex grid, the "ring distance" between two hexes is the minimum number
 * of single-hex steps to walk from one to the other. This is the hex analogue
 * of Manhattan distance in a rectangular grid, but for hexagons. Because each
 * hex has 6 neighbours, the set of hexes at ring distance exactly k forms a
 * hexagonal ring with 6k members — the hexagonal version of a circle.
 *
 * The cube distance formula computes this in O(1) without any graph traversal:
 *   ring = max(|ΔQ|, |ΔR|, |ΔS|)
 *        = (|ΔQ| + |ΔR| + |ΔS|) / 2    (both forms are equivalent)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture concentric hexagonal rings painted on the floor, each one a
 * different color, centered on the cursor hex. The cursor is ring 0 (white).
 * The 6 immediate neighbors are ring 1 (cyan). The next 12 hexes are ring 2
 * (green), then ring 3 (yellow), etc. Moving the cursor moves the entire
 * color pattern instantly — no animation, just a recompute per frame.
 *
 * The visual is striking because the hex grid's isotropy shows clearly:
 * every ring is a perfect "circle" in hex geometry, even though it looks
 * like a concentric hexagon on screen.
 *
 * The coloring of INTERIOR cells (not just borders) is key to seeing the
 * rings. If only border cells are colored, you see lines; filling the
 * interior means each ring band shows a solid color.
 *
 * DRAWING METHOD  (extends 01_flat_top with interior fill + ring color)
 * ──────────────────────────────────────────────────────────────────────
 *  Same pipeline as 01_flat_top up through cube_round and dist computation.
 *
 *  Then:
 *  1. Compute ring = hex_ring_dist(Q, R, cQ, cR) using the current cursor.
 *  2. Select color pair:
 *       ring == 0 → PAIR_CURSOR (white-on-blue)
 *       ring  > 0 → PAIR_RING + (ring % N_RING_COLORS)  (cycles)
 *  3. Interior cells (dist < limit): draw a SPACE in the ring color.
 *     This fills the hex's background with the ring color.
 *  4. Border cells (dist ≥ limit): draw angle_char with bold ring color.
 *
 * KEY FORMULAS
 * ────────────
 *  Ring distance between cursor (cQ,cR) and hex (Q,R):
 *    ΔQ = Q − cQ,  ΔR = R − cR,  ΔS = −ΔQ − ΔR
 *    ring = max(|ΔQ|, |ΔR|, |ΔS|)
 *         = (|ΔQ| + |ΔR| + |ΔS|) / 2
 *
 *  Number of hexes at ring distance k:
 *    k == 0: 1 hex (cursor itself)
 *    k >= 1: 6k hexes (hexagonal ring)
 *    Total hexes within distance k: 1 + 6*(1+2+...+k) = 3k²+3k+1
 *
 *  Color cycling: color index = ring % N_RING_COLORS
 *    Rings 0,6,12,... → white (PAIR_CURSOR)
 *    Rings 1,7,13,... → cyan
 *    Rings 2,8,14,... → green
 *    etc.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Interior fill uses ' ' (space), which sets the BACKGROUND color not the
 *    foreground. You MUST use the COLOR_PAIR attribute so the background color
 *    of the space is the ring color.
 *
 *  • Ring distance is always an integer; the mod % N_RING_COLORS is safe for
 *    all non-negative ring values. ring == 0 is special-cased for the cursor.
 *
 *  • Cursor movement recolors the ENTIRE grid each frame since every hex's
 *    ring distance changes. This is O(rows×cols) per frame — intentionally so.
 *    No caching is needed because the frame budget easily covers it.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14, cursor at (0,0))
 * ─────────────
 *  Hex (0,0): ring = max(0,0,0) = 0 → PAIR_CURSOR (white-on-blue). ✓
 *  Hex (1,0): ΔQ=1, ΔR=0, ΔS=−1 → ring = max(1,0,1) = 1 → PAIR_RING+1 (cyan). ✓
 *  Hex (2,0): ΔQ=2 → ring = 2 → green. ✓
 *  Hex (1,1): ΔQ=1, ΔR=1, ΔS=−2 → ring = max(1,1,2) = 2 → green. ✓
 *    (Confirms that (1,1) and (2,0) are both ring 2 — they're equidistant.)
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

#define BORDER_W_DEFAULT   0.12
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_RING_COLORS      6   /* palette cycles after this many rings */

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

#define PAIR_RING    1   /* 1..N_RING_COLORS for rings 0..N-1, then wrap */
#define PAIR_CURSOR  7   /* cursor hex overlay */
#define PAIR_HUD     8
#define PAIR_HINT    9

static const short RING_FG[N_RING_COLORS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_GREEN,
    COLOR_YELLOW, COLOR_MAGENTA, COLOR_BLUE,
};
static void color_init(void) {
    start_color();
    use_default_colors();
    for (int i = 0; i < N_RING_COLORS; i++)
        init_pair(PAIR_RING + i, RING_FG[i], COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLOR_BLACK,  COLOR_CYAN);
    init_pair(PAIR_HINT,   COLOR_CYAN,   COLOR_BLACK);
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * px = (col - ox) × CELL_W,  py = (row - oy) × CELL_H.
 * Flat-top pixel↔cube as in 01_flat_top.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * hex_ring_dist — cube distance between cursor (cQ,cR) and hex (Q,R).
 *
 * THE FORMULA:
 *   ΔQ = Q − cQ,  ΔR = R − cR,  ΔS = −ΔQ − ΔR   (S = −Q−R always)
 *   ring = max(|ΔQ|, |ΔR|, |ΔS|)
 *
 * EQUIVALENT FORM:
 *   ring = (|ΔQ| + |ΔR| + |ΔS|) / 2
 *   Both forms give the same integer result. The max form is slightly
 *   faster (no division); the sum form is more intuitive.
 *
 * WHY IT WORKS:
 *   In cube space, each axis (Q, R, S) constrains one "dimension" of
 *   motion. Moving diagonally in screen space changes two cube axes by
 *   ±1 simultaneously (but the third adjusts to maintain Q+R+S=0).
 *   The maximum component gives the minimum number of steps because you
 *   can always move diagonally to reduce two components at once.
 */
static int hex_ring_dist(int Q, int R, int cQ, int cR) {
    int dQ = Q - cQ, dR = R - cR, dS = -dQ - dR;
    int a = abs(dQ), b = abs(dR), c = abs(dS);
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

/*
 * angle_char — same as 01_flat_top. See that file for full documentation.
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
 * grid_draw — rasterize flat-top hex grid with ring-distance coloring.
 *
 * THE PIPELINE (extends 01_flat_top with interior fill and ring color):
 *
 *   pixel → cube → nearest hex (Q,R)  [same as 01_flat_top]
 *      │
 *      ▼  Compute ring distance from cursor
 *   ring = hex_ring_dist(Q, R, cQ, cR)
 *   pair = (ring==0) ? PAIR_CURSOR : PAIR_RING + (ring % N_RING_COLORS)
 *      │
 *      ├── dist < limit (interior):
 *      │     Draw ' ' (space) with ring's COLOR_PAIR.
 *      │     The space's BACKGROUND fills the hex interior with ring color.
 *      │
 *      └── dist ≥ limit (border):
 *            Draw angle_char with bold ring color.
 *            The entire hex border glows in its ring's color.
 *
 * WHY FILL INTERIOR:
 *   If only border cells are drawn, the ring bands appear as colored outlines
 *   with black gaps between them. Filling the interior makes each ring a solid
 *   colored region — the concentric-hexagon pattern becomes obvious.
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

            int ring = hex_ring_dist(Q, R, cQ, cR);
            int pair = (ring == 0) ? PAIR_CURSOR
                                   : PAIR_RING + (ring % N_RING_COLORS);
            int attr = COLOR_PAIR(pair);

            if (dist < limit) {
                /* Interior: fill with ring background color */
                attron(attr);
                mvaddch(row, col, ' ');
                attroff(attr);
            } else {
                /* Border: ring-colored character */
                double cx = size * 1.5 * fQ;
                double cy = size * (sq3_2 * fQ + sq3 * fR);
                double theta = atan2(py - cy, px - cx);
                char ch = angle_char(theta + M_PI / 2.0);
                attron(attr | A_BOLD);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr | A_BOLD);
            }
        }
    }
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * HEX_DIR — 4-direction axial movement (same table as 01–03).
 *
 * Effect on ring pattern:
 *   Moving RIGHT (Q+1): ring-0 hex shifts east; rings re-center instantly.
 *   Moving DOWN  (R+1): ring-0 hex shifts lower-right; rings follow.
 * The entire gradient redraws per frame — no caching needed.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

/*
 * cursor_draw — '@' at cursor hex center, drawn on top of ring fill.
 *
 * THE FORMULA (flat-top forward matrix, same as 01_flat_top):
 *   cx_pix = size × 3/2 × cQ
 *   cy_pix = size × (√3/2 × cQ  +  √3 × cR)
 *   col = ox + (int)(cx_pix / CELL_W)
 *   row = oy + (int)(cy_pix / CELL_H)
 *
 * Drawn after grid_draw so '@' is on top of the ring fill at the cursor hex.
 */
static void cursor_draw(double size, int cQ, int cR,
                         int ox, int oy, int rows, int cols) {
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = size * 1.5    * (double)cQ;
    double cy_pix = size * (sq3_2 * (double)cQ + sq3 * (double)cR);
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
    int    paused;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size = HEX_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    s->paused = 0;
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
    int dist_from_origin = hex_ring_dist(s->cQ, s->cR, 0, 0);
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  dist-from-origin:%d  size:%.0f  %5.1f fps  %s ",
             s->cQ, s->cR, dist_from_origin, s->hex_size, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move cursor  +/-:size  [/]:border ");
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
