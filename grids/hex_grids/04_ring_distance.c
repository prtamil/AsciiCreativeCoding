/*
 * 04_ring_distance.c — color a hex grid by how far each hex is from a movable '@'.
 *
 * Every hex is tinted by its "ring distance" from the cursor: the fewest
 * single-hex steps to walk there. The cursor is ring 0, its 6 neighbours are
 * ring 1, and so on, so the colors form concentric hexagon rings that follow
 * the cursor as you move it with the arrow keys.
 *
 * Sister file: grids/hex_grids/01_flat_top.c (shares the Cursor + HEX_DIR setup).
 * Hex distance and ring math: https://www.redblobgames.com/grids/hexagons/#distances
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

#define HEX_SIZE_DEFAULT  14.0
#define HEX_SIZE_MIN       6.0
#define HEX_SIZE_MAX      40.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.12
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_RING_COLORS      6   /* colors repeat once you pass this many rings */

#define TICK_NS           16666667LL

/* How much each frame nudges the on-screen FPS number; small = steadier reading. */
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

#define PAIR_RING    1   /* one color per ring; rings beyond the palette reuse these */
#define PAIR_CURSOR  7   /* the hex the '@' sits on */
#define PAIR_HUD     8   /* yellow status bar */
#define PAIR_HINT    9   /* cyan key-hint footer */

static const short RING_FG[N_RING_COLORS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_GREEN,
    COLOR_YELLOW, COLOR_MAGENTA, COLOR_BLUE,
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_RING_COLORS; i++)
        init_pair(PAIR_RING + i, RING_FG[i], COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — turn screen cells into hexes, then measure ring distance ── */

/*
 * GridCtx — everything we need to lay flat-top hexagons over the terminal.
 * One of these is filled in at startup (and on resize / size changes) and then
 * read by every drawing routine; nothing here changes during a single frame.
 */
typedef struct {
    /* Terminal size in character cells. */
    int rows, cols;

    /* Hex shape and placement. */
    double hex_size;        /* radius of a hex, in sub-cell pixels; bigger = fewer, larger hexes */
    double border_w;        /* fraction of a hex treated as edge rather than fill, 0..0.5 */
    int    cell_w, cell_h;  /* sub-pixels per character cell, so squares look square */

    /* Where hex (0,0) lands — roughly the middle of the screen. */
    int    ox, oy;

    /* Rough count of how many hexes fit each way; only a sizing hint. */
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

static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5    * (double)Q;
    double cy_pix = g->hex_size * (sq3_2 * (double)Q + sq3 * (double)R);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/* Snaps a fractional hex coordinate to the nearest real hex. See 01_flat_top. */
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

/*
 * The fewest hex steps from one hex to another — the "ring distance" that
 * drives all the coloring. Hexes use three axes (Q, R, and S = -Q-R) that
 * always sum to zero; the biggest gap along any one axis is the answer,
 * because you can shrink two axes at once by stepping diagonally.
 */
static int cube_dist(int Q, int R, int cQ, int cR)
{
    int dQ = Q - cQ, dR = R - cR, dS = -dQ - dR;
    int a = abs(dQ), b = abs(dR), c = abs(dS);
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

/* Picks an ASCII slash/bar to draw a hex edge at the given tilt. See 01_flat_top. */
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
 * Paints the whole grid for one frame. For each screen cell it works out which
 * hex it belongs to, measures that hex's ring distance from the cursor, and
 * colors it. We fill the inside of each hex (not just its outline) so each ring
 * shows as a solid color band — otherwise you'd see thin colored rings with
 * black gaps and the pattern would be hard to read.
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

            int ring = cube_dist(Q, R, cQ, cR);
            int pair = (ring == 0) ? PAIR_CURSOR
                                   : PAIR_RING + (ring % N_RING_COLORS);
            int attr = COLOR_PAIR(pair);

            if (dist < limit) {
                /* Inside the hex: a colored space paints the background solid. */
                attron(attr);
                mvaddch(row, col, ' ');
                attroff(attr);
            } else {
                /* On a hex edge: draw a slash in the ring's color. */
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

/* ── §5 cursor ── */

/*
 * Cursor — which hex the '@' is on, named by its two hex coordinates (q, r).
 * That's all the state we keep; every ring color is recomputed from it each
 * frame, so moving the cursor instantly recolors the whole grid.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * HEX_DIR — the (q, r) step each arrow key adds to the cursor. Same table
 * as 01–03. Each move shifts where the rings are centered, and the next frame
 * redraws them around the new spot.
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

/* Stamps the '@' on the cursor hex. Called after the grid fill so it sits on top. */
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
    int dist_from_origin = cube_dist(cur->q, cur->r, 0, 0);
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  dist-from-origin:%d  size:%.0f  %5.1f fps  %s ",
             cur->q, cur->r, dist_from_origin, g->hex_size, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move cursor  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, paused, fps);
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
