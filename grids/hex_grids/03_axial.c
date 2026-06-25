/*
 * 03_axial.c — a flat-top hex grid where every hexagon is labeled with its
 * own coordinates. You drive a cursor around with the arrow keys; the three
 * coordinate axes glow in different colors so you can see how hex space is
 * laid out.
 *
 * Hexes use axial coordinates (Q, R): two numbers per hex. There's a third,
 * S = -Q-R, but it's always derivable, so we only ever store Q and R.
 *
 * Sister file: grids/hex_grids/01_flat_top.c — same grid and cursor; this one
 * adds the coordinate labels on top.
 * Coordinate background: https://www.redblobgames.com/grids/hexagons/#coordinates-axial
 */

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

/* ── §1 config ── */

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

/* How fast the on-screen FPS number reacts. Smaller = steadier, slower to move. */
#define FPS_EWMA_ALPHA     0.05

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ── */

#define PAIR_DEFAULT  1
#define PAIR_Q_AXIS   2   /* cyan   — the line of hexes where Q is 0 */
#define PAIR_R_AXIS   3   /* green  — the line of hexes where R is 0 */
#define PAIR_S_AXIS   4   /* yellow — the line where S is 0 (i.e. Q+R is 0) */
#define PAIR_ORIGIN   5   /* white  — the one hex at the center, (0,0) */
#define PAIR_CURSOR   6   /* the hex you're currently on: white on blue */
#define PAIR_HUD      7   /* yellow status bar */
#define PAIR_HINT     8   /* cyan key-hint footer */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_DEFAULT, COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_Q_AXIS,  COLOR_CYAN,   COLOR_BLACK);
    init_pair(PAIR_R_AXIS,  COLOR_GREEN,  COLOR_BLACK);
    init_pair(PAIR_S_AXIS,  COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_ORIGIN,  COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/*
 * Picks the color for one hex. Order matters: the first thing that matches
 * wins. The cursor is checked first on purpose — if you park the cursor on a
 * hex that also sits on an axis, you want it to look like the cursor, not the
 * axis.
 */
static int hex_color(int Q, int R, int cQ, int cR)
{
    if (Q == cQ && R == cR)    return PAIR_CURSOR;
    if (Q == 0 && R == 0)      return PAIR_ORIGIN;
    if (Q == 0)                return PAIR_Q_AXIS;
    if (R == 0)                return PAIR_R_AXIS;
    if (-Q - R == 0)           return PAIR_S_AXIS;
    return PAIR_DEFAULT;
}

/* ── §4 formula — turning hex coordinates into screen cells, and back ── */

/*
 * GridCtx — everything we need to know to draw the grid at its current size
 * and position. Recomputed whenever the terminal resizes or the user changes
 * the hex size. Same layout as 01_flat_top.c.
 */
typedef struct {
    /* Size of the terminal, in character cells. */
    int rows, cols;

    /* How big each hexagon is and how thick its border looks.
     * hex_size: roughly the corner-to-center distance, in sub-pixels.
     * border_w: fraction of a hex (0..0.5) treated as "edge" rather than inside. */
    double hex_size;
    double border_w;
    /* How many sub-pixels one terminal cell is worth, width and height.
     * Terminal cells are taller than they are wide, so these differ. */
    int    cell_w, cell_h;

    /* The screen cell that the grid's center (hex 0,0) sits on. */
    int    ox, oy;

    /* Rough count of how far the grid reaches in each direction. Advisory only:
     * used to bound loops, not to clip — the per-hex on-screen checks do that. */
    int    max_q, max_r;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = (rows - 1) / 2;
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/* Given a hex's (Q, R), find the screen cell its center lands on. */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5    * (double)Q;
    double cy_pix = g->hex_size * (sq3_2 * (double)Q + sq3 * (double)R);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/* Snaps fractional hex coordinates to the nearest real hex. Full write-up
 * lives in 01_flat_top.c. */
static void cube_round(double fq, double fr, double fs, int *Q, int *R)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
}

/* The reverse of ctx_to_screen: which hex does a screen cell fall in?
 * Kept for reference and symmetry; this demo doesn't currently call it. */
__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (2.0/3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/* Picks a slash, dash or bar to suggest the slope of a hex edge at this point.
 * Same as 01_flat_top.c. */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)   return '\\';
    else if (t < 5.0 * M_PI / 8.0)   return '|';
    else if (t < 7.0 * M_PI / 8.0)   return '/';
    else                              return '-';
}

/*
 * Pass 1: draw the hex outlines. We walk every screen cell, figure out which
 * hex it belongs to and how close it is to that hex's edge, and only draw the
 * cells near an edge — the insides are left blank on purpose so the labels in
 * pass 2 have room. Each edge is tinted by which axis (if any) its hex is on.
 * Same pixel-by-pixel approach as 01_flat_top.c.
 */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double size  = g->hex_size;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

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
 * Pass 2: stamp each hex's "Q,R" label in its middle. We sweep a range of
 * (Q,R) values wide enough to cover the screen (a little extra, then we just
 * skip any whose center lands off-screen), and print the text centered on the
 * hex. The cursor and origin labels are made bold so they stand out; ordinary
 * axis labels are dimmed so the colored outlines stay the star of the show.
 */
static void ctx_draw_labels(const GridCtx *g, int cQ, int cR)
{
    double sq3 = sqrt(3.0);
    int Qmax = (int)(g->cols * g->cell_w / (1.5 * g->hex_size)) + 3;
    int Rmax = (int)(g->rows * g->cell_h / (sq3 * g->hex_size)) + 3;

    for (int Q = -Qmax; Q <= Qmax; Q++) {
        for (int R = -Rmax; R <= Rmax; R++) {
            int sr, sc;
            ctx_to_screen(g, Q, R, &sr, &sc);
            if (sr < 1 || sr >= g->rows - 1) continue;
            if (sc < 2 || sc >= g->cols - 2) continue;

            char buf[12];
            int len = snprintf(buf, sizeof buf, "%d,%d", Q, R);
            int lx  = sc - len / 2;
            if (lx < 0 || lx + len >= g->cols) continue;

            int pair = hex_color(Q, R, cQ, cR);
            int attr = (pair == PAIR_CURSOR) ? (COLOR_PAIR(pair) | A_BOLD | A_REVERSE)
                     : (pair == PAIR_ORIGIN) ? (COLOR_PAIR(pair) | A_BOLD)
                     :                          (COLOR_PAIR(pair) | A_DIM);
            attron(attr);
            mvprintw(sr, lx, "%s", buf);
            attroff(attr);
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — where the '@' currently sits, as a hex coordinate (q, r). That's
 * all the state we need; everything else about it is computed from the grid.
 * Same struct as 01_flat_top.c.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * HEX_DIR — how each arrow key nudges (Q, R). Right/left change Q, down/up
 * change R, so you can watch a label's two numbers tick up and down as you
 * move. Same table as 01_flat_top.c.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/* Drops the '@' on the cursor hex. Called after the labels so it lands on top
 * of the label's middle character instead of being hidden under it. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    int S = -cur->q - cur->r;
    char buf[96];
    snprintf(buf, sizeof buf,
             " cursor Q:%+d R:%+d S:%+d  size:%.0f  %5.1f fps  %s ",
             cur->q, cur->r, S, g->hex_size, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move  +/-:size  [/]:border "
             " cyan=Q  green=R  yellow=S  white=origin ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    erase();
    ctx_draw_bg    (g, cur->q, cur->r);   /* Pass 1: borders */
    ctx_draw_labels(g, cur->q, cur->r);   /* Pass 2: labels  */
    cursor_draw    (cur, g);              /* Pass 3: '@'     */
    hud_draw       (g, cur, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    color_init();
    int paused = 0;

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    double fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) { g.hex_size += HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) { g.hex_size -= HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '[':
                if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
            case ']':
                if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
