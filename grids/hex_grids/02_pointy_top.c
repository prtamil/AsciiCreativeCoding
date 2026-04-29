/*
 * 02_pointy_top.c — pointy-top hexagonal grid with keyboard-controlled cursor
 *
 * DEMO: Same cube-coordinate + cursor logic as 01_flat_top.c but with the
 *       hex rotated 30°. Pointy-top flat sides face left/right → '|'.
 *       '@' cursor moves between hexes with arrow keys.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c
 *                  (identical §5b cursor section; only transform matrix differs)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs
 *   §4 coords   — pixel↔cell, centering offset, pointy-top matrix
 *   §5 draw     — pointy-top hex rasterizer
 *   §5b cursor  — movement vectors, cursor_draw
 *   §6 scene    — state struct + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 02_pointy_top.c -o 02_pointy_top -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pointy-top hex = flat-top rotated 30°. Only the 2×2
 *                  inverse matrix changes:
 *                    fq = (√3/3·px − 1/3·py) / s
 *                    fr = (2/3·py) / s
 *                  Cube→pixel: cx = s(√3Q + √3/2·R), cy = s·3/2·R.
 *
 * Cursor movement : Same HEX_DIR[4][2] deltas as 01_flat_top — the axial
 *                  system is orientation-independent. RIGHT still adds Q+1,
 *                  UP subtracts R by 1, etc. cursor_draw uses the pointy-top
 *                  forward matrix for hex→pixel conversion.
 *
 * References     :
 *   Pointy-top matrix — https://www.redblobgames.com/grids/hexagons/#hex-to-pixel-pointy
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pointy-top is flat-top rotated 30° around the screen center. The cube
 * coordinate math (Q+R+S=0, cube_round, cube distance, border threshold) is
 * IDENTICAL. Only the 2×2 transform matrix that converts between pixel space
 * and cube space changes. Compare the two matrices side-by-side:
 *
 *   Flat-top inverse:               Pointy-top inverse:
 *     fq = (2/3·px) / s               fq = (√3/3·px − 1/3·py) / s
 *     fr = (−1/3·px + √3/3·py) / s   fr = (2/3·py) / s
 *
 * The flat-top matrix has a pure-x term for fq; the pointy-top matrix has a
 * pure-y term for fr. This is exactly a 30° rotation of the basis vectors.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * In flat-top orientation the hex has a flat edge at the top. Move the whole
 * hex grid 30° counterclockwise: now a vertex points up, and the flat edge
 * faces left/right. The tiling still covers the plane the same way — just
 * rotated. Every formula that worked on cube coordinates still works; only the
 * front-door (pixel→cube) and back-door (cube→pixel) change to reflect the
 * new orientation.
 *
 * The cursor movement table (HEX_DIR) is in AXIAL space (Q,R), which is
 * independent of orientation. So the same 4 arrow keys work for both files.
 *
 * DRAWING METHOD  (identical to 01, with different matrix entries)
 * ──────────────────────────────────────────────────────────────────
 *  1. Center the grid: px = (col−ox)×CELL_W, py = (row−oy)×CELL_H.
 *
 *  2. Pointy-top inverse matrix:
 *       fq = (√3/3·px − 1/3·py) / size
 *       fr = (2/3·py) / size
 *       fs = −fq − fr
 *
 *  3. cube_round (same as 01_flat_top — orientation-independent).
 *
 *  4. Cube distance (same as 01_flat_top).
 *
 *  5. Skip interior pixels (dist < 0.5 − border_w).
 *
 *  6. Pointy-top forward matrix for hex center:
 *       cx = size × (√3·Q + √3/2·R)
 *       cy = size × 3/2 × R
 *
 *  7. angle_char(theta + π/2) — same function as 01_flat_top.
 *
 *  8. Draw character in cursor or border color.
 *
 * KEY FORMULAS
 * ────────────
 *  Pointy-top inverse matrix (pixel → fractional cube):
 *    fq = (√3/3·px − 1/3·py) / size
 *    fr = (2/3·py) / size
 *
 *  Pointy-top forward matrix (hex → pixel center):
 *    cx = size × (√3·Q + √3/2·R)
 *    cy = size × 3/2 × R
 *
 *  Derivation: pointy-top forward = flat-top forward × R(−30°), where
 *    R(−30°) is a 30° clockwise rotation matrix. The inverse is R(+30°).
 *    Working out the product gives the formulas above.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The flat top/bottom edges that show '-' in flat-top now become the
 *    left/right edges and show '|'. The diagonal edges (\ and /) remain but
 *    swap roles. angle_char is the same function — it adapts automatically
 *    because theta is computed from the actual pixel→center geometry.
 *
 *  • cursor_draw MUST use the pointy-top forward matrix, not flat-top.
 *    Using the wrong formula shifts '@' to the wrong hex center visually.
 *
 *  • HEX_DIR deltas are the same in both files because they operate in
 *    axial (Q,R) space, not screen space.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14)
 * ─────────────
 *  Origin hex (Q=0,R=0) center: cx=0, cy=0 → col=ox=40, row=oy=11.
 *  '@' at (11,40). ✓ Same screen position as 01_flat_top.
 *
 *  Check that top vertex is vertical:
 *  Pixel directly above center: py=−16 (row=7), px=0.
 *    fq = (0 − (−16)/3) / 14 = (16/3)/14 ≈ 0.381
 *    fr = (2/3·(−16)) / 14 = −32/42 ≈ −0.762
 *    cube_round → Q=0, R=−1 (the hex directly above).
 *    Pointy-top: hex (0,−1) center: cx=0, cy=14·(−1.5)=−21 pixels.
 *    That hex's top vertex is at cy=−21−14 = screen up. ✓ Vertex points up.
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
#define PAIR_CURSOR   2
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
 * Centering: px = (col - ox) × CELL_W, py = (row - oy) × CELL_H.
 *
 * Pointy-top pixel↔cube:
 *   fq = (√3/3·px − 1/3·py) / size   (inverse matrix row 1)
 *   fr = (2/3·py) / size               (inverse matrix row 2)
 *   cx = size × (√3·Q + √3/2·R)       (forward matrix column 1)
 *   cy = size × 3/2 × R               (forward matrix column 2)
 *
 * Compare flat-top: only the matrix entries differ; cube_round, cube dist,
 * angle_char, and HEX_DIR are identical.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * angle_char — same as 01_flat_top. See that file for full documentation.
 *
 * Works unchanged for pointy-top because theta is computed from the actual
 * geometry (atan2 of pixel relative to hex center), not from orientation
 * assumptions. The pointy-top edges happen to produce different theta values
 * that naturally map to '|' on the left/right faces.
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
 * grid_draw — pointy-top hex rasterizer.
 *
 * THE PIPELINE (differences from 01_flat_top marked with ★):
 *
 *   px = (col−ox)×CELL_W,  py = (row−oy)×CELL_H
 *      │
 *      ▼  ★ Pointy-top inverse matrix (note the row-major swap):
 *   fq = (√3/3·px − 1/3·py) / size     ← mixes x and y
 *   fr = (2/3·py) / size                ← pure y
 *   fs = −fq − fr
 *      │
 *      ▼  cube_round (identical to flat-top)
 *      │
 *      ▼  cube distance (identical to flat-top)
 *      │
 *      ├── interior: skip
 *      │
 *      └── border:
 *            ★ cx = size × (√3·Q + √3/2·R)   ← pointy-top forward x
 *            ★ cy = size × 3/2 × R            ← pointy-top forward y
 *            theta = atan2(py−cy, px−cx)
 *            ch = angle_char(theta + π/2)
 *
 * WHY THE MATRIX SWAP WORKS:
 *   Pointy-top = flat-top rotated 30° CCW. The rotation matrix is:
 *     [cos30   −sin30] = [√3/2  −1/2]
 *     [sin30    cos30]   [1/2    √3/2]
 *   Applying this to the flat-top inverse gives the pointy-top inverse
 *   above. The cube_round and distance logic are rotation-invariant.
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

            /* Pointy-top inverse matrix */
            double fq = (sq3_3 * px - 1.0/3.0 * py) / size;
            double fr = (2.0/3.0 * py) / size;
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

            /* Pointy-top forward matrix */
            double cx = size * (sq3 * fQ + sq3_2 * fR);
            double cy = size * 1.5 * fR;
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
 * HEX_DIR — 4-direction movement in axial (Q, R).
 *
 * Pointy-top screen layout (same deltas as flat-top):
 *
 *               UP: (0,-1)  ← cy = 3/2·R·s, so R-1 moves up
 *                    ↑
 *   LEFT: (-1,0) ← ● → RIGHT: (+1,0)   ← cx = √3·Q·s, Q+1 moves right
 *                    ↓
 *              DOWN: (0,+1)
 *
 * WHY SAME DELTAS AS FLAT-TOP:
 *   HEX_DIR operates in axial (Q,R) space which has no notion of screen
 *   orientation. Both the flat-top and pointy-top grids use the same axial
 *   coordinate labels — only the pixel positions of those hexes differ.
 *   A step of Δ(Q=+1, R=0) moves one hex to the right in both orientations.
 *
 * Unmapped faces for pointy-top: (Q+1,R-1) upper-right and (Q-1,R+1) lower-left.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

/*
 * cursor_draw — place '@' at the center cell of pointy-top hex (cQ, cR).
 *
 * THE FORMULA (pointy-top forward matrix):
 *
 *   cx_pix = size × (√3 × cQ  +  √3/2 × cR)
 *   cy_pix = size × 3/2 × cR
 *
 *   col = ox + (int)(cx_pix / CELL_W)
 *   row = oy + (int)(cy_pix / CELL_H)
 *
 * Compare to flat-top cursor_draw:
 *   flat-top:   cx = 3/2·Q·s,  cy = (√3/2·Q + √3·R)·s
 *   pointy-top: cx = (√3·Q + √3/2·R)·s,  cy = 3/2·R·s
 * The Q and R roles are "swapped" between x and y — reflecting the 30° rotation.
 */
static void cursor_draw(double size, int cQ, int cR,
                         int ox, int oy, int rows, int cols) {
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = size * (sq3   * (double)cQ + sq3_2 * (double)cR);
    double cy_pix = size * 1.5    * (double)cR;
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
            case 't': sc.theme = (sc.theme + 1) % N_THEMES; color_init(sc.theme); break;
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
