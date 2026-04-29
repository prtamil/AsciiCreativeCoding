/*
 * 03_axial.c — flat-top hex grid with axial labels and keyboard cursor
 *
 * DEMO: Each hexagon shows its "Q,R" axial coordinates. The cursor hex is
 *       highlighted white-on-blue with '@' at its center. Q=0/R=0/S=0 axes
 *       glow in cyan/green/yellow so you can trace them across the plane.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (same §5b cursor section)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — axis color pairs
 *   §4 coords   — pixel↔cell, centering offset
 *   §5 draw     — border pass + label pass
 *   §5b cursor  — movement vectors, cursor_draw
 *   §6 scene    — state struct + two-pass scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 03_axial.c -o 03_axial -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-pass render. Pass 1: hex borders colored by axis
 *                  membership. Pass 2: enumerate visible hexes, print label
 *                  at each center. Labels land on interior cells (dist≈0)
 *                  skipped in pass 1.
 *
 * Cursor movement : Same HEX_DIR[4][2] as 01_flat_top. '@' drawn in a third
 *                  pass so it always sits on top of the label. Cursor hex
 *                  border is drawn in PAIR_CURSOR in pass 1.
 *
 * References     :
 *   Axial vs cube — https://www.redblobgames.com/grids/hexagons/#coordinates-axial
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Axial coordinates (Q, R) are a 2-component alias for cube coordinates.
 * Since S = −Q−R is always derivable, you only need to store Q and R.
 * This file makes the coordinate structure visible: each hex displays its
 * own (Q,R) label, and the three coordinate axes are colored differently
 * so you can see how the hex plane is partitioned.
 *
 * The three axis planes of cube space slice the hex plane into six sectors:
 *   Q=0 axis  (cyan):   hexes where Q=0  — a diagonal band of hexes
 *   R=0 axis  (green):  hexes where R=0  — another diagonal band
 *   S=0 axis  (yellow): hexes where S=0, i.e., Q+R=0 — the third band
 *   Origin    (white):  the single hex where Q=R=0 (S=0 automatically)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the hexagonal plane as the diagonal cross-section of a 3-D cubic
 * lattice. Each hex corresponds to a unit cube whose (x,y,z) coordinates
 * satisfy x+y+z=0. The Q=0 axis is the set of cubes where x=0 — a plane
 * in 3-D that slices the hex plane as a line of hexes.
 *
 * Moving RIGHT (+Q): you climb along the Q-axis; the label's Q number
 *   increases by 1, R stays the same.
 * Moving DOWN (+R): R number increases by 1, Q stays the same.
 * Moving diagonally (would be NE or SW): both Q and R change.
 *
 * The label in each hex lets you directly read off the coordinate system
 * rather than counting from the origin — very useful when learning hex math.
 *
 * DRAWING METHOD  (two-pass rendering)
 * ─────────────────────────────────────
 *  PASS 1 — border rasterizer (same as 01_flat_top):
 *   For each screen cell, compute cube coords (Q,R) and cube distance.
 *   If border pixel: draw angle_char in the color given by hex_color(Q,R).
 *   hex_color priority: cursor > origin > Q-axis > R-axis > S-axis > default.
 *
 *  PASS 2 — label printer:
 *   Loop over all (Q,R) whose hex center would be on screen.
 *   Compute screen cell (cx_cell, cy_cell) from flat-top forward matrix.
 *   Print "Q,R" centered on that cell in the appropriate axis color.
 *   This pass only writes interior cells (never border cells), so pass 1
 *   characters are not overwritten by labels.
 *
 *  PASS 3 — cursor '@':
 *   Draw '@' on top of the label in the cursor hex. Must be last so '@'
 *   overwrites the label character at the cursor hex center.
 *
 * KEY FORMULAS
 * ────────────
 *  hex_color priority (highest wins):
 *    cursor      if Q==cQ && R==cR
 *    origin      if Q==0  && R==0
 *    Q-axis      if Q==0   (S-axis check below would also match when R=0,Q=0
 *                           but origin already catches that)
 *    R-axis      if R==0
 *    S-axis      if −Q−R==0  (i.e., Q+R=0)
 *    default     otherwise
 *
 *  Axis membership test:
 *    Q=0 axis: the set of hexes at Q=0 — a diagonal NW-SE band in screen space
 *    R=0 axis: horizontal band (R=0 → cy = √3/2·Q·s ∝ Q, so it's NOT horizontal
 *              in screen space for flat-top! It's a slanted band through origin)
 *    S=0 axis: Q+R=0 → cy = √3·R·s + √3/2·Q·s = √3·(−Q)·s + √3/2·Q·s = −√3/2·Q·s
 *              Also slanted.
 *
 *  Visible hex count (for label pass iteration bounds):
 *    Qmax ≈ cols × CELL_W / (1.5 × size)
 *    Rmax ≈ rows × CELL_H / (√3  × size)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Label centering: "Q,R" has variable length (e.g., "-10,-3" is 7 chars).
 *    Use lx = cx_cell − len/2 and guard lx >= 0 && lx+len < cols.
 *
 *  • At large sizes the label may be wider than the hex interior. Guard with
 *    cx_cell > 2 && cx_cell < cols-2 to avoid partial labels at screen edges.
 *
 *  • hex_color: cursor check must come FIRST — if cQ=0, the cursor hex is
 *    also on the Q-axis; we want cursor color to win over axis color.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=20)
 * ─────────────
 *  Origin hex (Q=0, R=0): label "0,0" drawn in white (PAIR_ORIGIN), '@' on top.
 *  Hex (1, 0): cx = 20×1.5=30 pixels → col = 40+15=55.  label "1,0" in green (R=0).
 *  Hex (0, 1): cy = 20×√3=34.6 px → row = 11+8=19.     label "0,1" in cyan (Q=0).
 *  Hex (1,-1): Q+R = 0 → S-axis color (yellow).         label "1,-1".
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

#define HEX_SIZE_DEFAULT  20.0
#define HEX_SIZE_MIN      10.0
#define HEX_SIZE_MAX      50.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.08
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.25
#define BORDER_W_STEP      0.02

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

#define PAIR_DEFAULT  1
#define PAIR_Q_AXIS   2   /* cyan   — hexes where Q=0 */
#define PAIR_R_AXIS   3   /* green  — hexes where R=0 */
#define PAIR_S_AXIS   4   /* yellow — hexes where S=Q+R=0 */
#define PAIR_ORIGIN   5   /* white  — the single hex (0,0,0) */
#define PAIR_CURSOR   6   /* cursor hex: white-on-blue */
#define PAIR_HUD      7
#define PAIR_HINT     8

static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(PAIR_DEFAULT, COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_Q_AXIS,  COLOR_CYAN,   COLOR_BLACK);
    init_pair(PAIR_R_AXIS,  COLOR_GREEN,  COLOR_BLACK);
    init_pair(PAIR_S_AXIS,  COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_ORIGIN,  COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,     COLOR_BLACK,  COLOR_CYAN);
    init_pair(PAIR_HINT,    COLOR_CYAN,   COLOR_BLACK);
}

/*
 * hex_color — pick color pair for hex (Q,R) based on axis membership.
 *
 * THE PRIORITY ORDER (first match wins):
 *   1. cursor   — must be first so moving onto an axis hex shows cursor color
 *   2. origin   — (0,0) gets special white display
 *   3. Q=0 axis — bold cyan diagonal band
 *   4. R=0 axis — green band
 *   5. S=0 axis — yellow band (S = −Q−R, so S=0 means Q+R=0)
 *   6. default  — all other hexes
 */
static int hex_color(int Q, int R, int cQ, int cR) {
    if (Q == cQ && R == cR)    return PAIR_CURSOR;
    if (Q == 0 && R == 0)      return PAIR_ORIGIN;
    if (Q == 0)                 return PAIR_Q_AXIS;
    if (R == 0)                 return PAIR_R_AXIS;
    if (-Q - R == 0)            return PAIR_S_AXIS;
    return PAIR_DEFAULT;
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * px = (col - ox) × CELL_W,  py = (row - oy) × CELL_H,  ox=cols/2, oy=(rows-1)/2.
 * Flat-top pixel↔cube: same as 01_flat_top §4.
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * angle_char — same function as 01_flat_top. See that file for documentation.
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
 * grid_draw_borders — Pass 1: hex borders colored by axis membership.
 *
 * THE FORMULA (per screen cell):
 *
 *   Same pipeline as 01_flat_top grid_draw — pixel → cube → border char.
 *   The only difference: instead of PAIR_BORDER/PAIR_CURSOR, the color is
 *   chosen by hex_color(Q, R, cQ, cR), which returns an axis-specific pair.
 *
 *   Border cells are visited in raster order. Interior cells are skipped
 *   (dist < limit). This means label pass 2 can safely write to interior
 *   cells without colliding with border characters.
 *
 *   Attribute selection:
 *     PAIR_ORIGIN or PAIR_CURSOR: A_BOLD for prominence
 *     Axis colors (Q/R/S):        no bold — dimmer than axis origin
 */
static void grid_draw_borders(int rows, int cols, double size, double border_w,
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

            int pair = hex_color(Q, R, cQ, cR);
            int attr = (pair == PAIR_ORIGIN || pair == PAIR_CURSOR)
                       ? (COLOR_PAIR(pair) | A_BOLD) : COLOR_PAIR(pair);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * grid_draw_labels — Pass 2: print "Q,R" at each visible hex center.
 *
 * THE FORMULA (hex enumeration + forward projection):
 *
 *   Iteration bounds (conservative — screens outside are clipped below):
 *     Qmax = cols×CELL_W / (1.5×size) + 3
 *     Rmax = rows×CELL_H / (√3×size)  + 3
 *
 *   For each (Q,R) in [−Qmax..Qmax] × [−Rmax..Rmax]:
 *     cx_pix = size × 3/2 × Q         ← flat-top forward x
 *     cy_pix = size × (√3/2×Q + √3×R) ← flat-top forward y
 *     cx_cell = ox + (int)(cx_pix / CELL_W)
 *     cy_cell = oy + (int)(cy_pix / CELL_H)
 *
 *   If cy_cell and cx_cell are on screen: format label, print centered.
 *   Label text: snprintf(buf, sizeof buf, "%d,%d", Q, R)
 *   Centering:  lx = cx_cell - len/2
 *
 * Attribute selection:
 *   cursor hex: A_BOLD | A_REVERSE — very visible (inverted colors)
 *   origin:     A_BOLD
 *   axis hexes: A_DIM  — labels recede; borders dominate visually
 */
static void grid_draw_labels(int rows, int cols, double size,
                              int cQ, int cR, int ox, int oy) {
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    int Qmax = (int)(cols * CELL_W / (1.5 * size)) + 3;
    int Rmax = (int)(rows * CELL_H / (sq3  * size)) + 3;

    for (int Q = -Qmax; Q <= Qmax; Q++) {
        for (int R = -Rmax; R <= Rmax; R++) {
            double cx_pix = size * 1.5  * (double)Q;
            double cy_pix = size * (sq3_2 * (double)Q + sq3 * (double)R);
            int cx_cell = ox + (int)(cx_pix / CELL_W);
            int cy_cell = oy + (int)(cy_pix / CELL_H);
            if (cy_cell < 1 || cy_cell >= rows - 1) continue;
            if (cx_cell < 2 || cx_cell >= cols - 2) continue;

            char buf[12];
            int len = snprintf(buf, sizeof buf, "%d,%d", Q, R);
            int lx  = cx_cell - len / 2;
            if (lx < 0 || lx + len >= cols) continue;

            int pair = hex_color(Q, R, cQ, cR);
            int attr = (pair == PAIR_CURSOR) ? (COLOR_PAIR(pair) | A_BOLD | A_REVERSE)
                     : (pair == PAIR_ORIGIN) ? (COLOR_PAIR(pair) | A_BOLD)
                     :                          (COLOR_PAIR(pair) | A_DIM);
            attron(attr);
            mvprintw(cy_cell, lx, "%s", buf);
            attroff(attr);
        }
    }
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * HEX_DIR — 4-direction axial movement (same as 01_flat_top §5b).
 *
 * Navigating axial labels:
 *   RIGHT (+Q): Q value on label increases by 1, R unchanged.
 *   DOWN  (+R): R value on label increases by 1, Q unchanged.
 * The axis highlighting follows the cursor — moving onto a Q=0 hex while
 * the cursor is there shows PAIR_CURSOR, not PAIR_Q_AXIS.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

/*
 * cursor_draw — '@' at cursor hex center, drawn last (on top of label).
 *
 * THE FORMULA (flat-top forward matrix, same as 01_flat_top):
 *   cx_pix = size × 3/2 × cQ
 *   cy_pix = size × (√3/2 × cQ + √3 × cR)
 *   col = ox + (int)(cx_pix / CELL_W)
 *   row = oy + (int)(cy_pix / CELL_H)
 *
 * Drawn AFTER grid_draw_labels so '@' overwrites the label character at
 * the center cell. The label is still partially visible in adjacent cells.
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
    grid_draw_borders(rows, cols, s->hex_size, s->border_w, s->cQ, s->cR, ox, oy);
    grid_draw_labels (rows, cols, s->hex_size,               s->cQ, s->cR, ox, oy);
    cursor_draw      (s->hex_size, s->cQ, s->cR, ox, oy, rows, cols);
}

/* ── §7 screen ────────────────────────────────────────────────────────── */

static void screen_init(void) {
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
}
static void screen_draw_hud(const Scene *s, int rows, int cols, double fps) {
    int S = -s->cQ - s->cR;
    char buf[96];
    snprintf(buf, sizeof buf,
             " cursor Q:%+d R:%+d S:%+d  size:%.0f  %5.1f fps  %s ",
             s->cQ, s->cR, S, s->hex_size, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move  +/-:size  [/]:border "
             " cyan=Q  green=R  yellow=S  white=origin ");
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
