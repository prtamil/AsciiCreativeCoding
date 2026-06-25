/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_hex_direct.c — move a cursor over a hex grid and drop/remove objects on it.
 *
 * Each object remembers its hex (q,r), not a screen spot, so it stays stuck to
 * its hex across a resize. "Direct" placement = SPACE toggles an object on the
 * cursor's hex. The grid drawing is the same recipe as hex_grids/01_flat_top.c.
 *
 * Sister files: hex_grids/01_flat_top.c (grid only), rect_grids_placement/01_direct.c.
 * Hex math: https://www.redblobgames.com/grids/hexagons/
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

#define CELL_W              2     /* pixels per char cell (2 wide x 4 tall) — */
#define CELL_H              4     /* undoes the cell's tall aspect so hexes look round */

#define HEX_SIZE_DEFAULT   14.0   /* centre-to-corner, pixels; bigger = fewer hexes */
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10  /* outline band width, 0..0.5 (0 = none, 0.5 = solid) */
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

#define MAX_OBJ           256     /* room for far more objects than fit on screen */
#define FRAME_NS    16666667LL    /* one frame at ~60 fps, in nanoseconds */

#define FPS_EWMA_ALPHA      0.05  /* small = steadier on-screen fps number */

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

#define PAIR_GRID    1   /* the hex outlines            */
#define PAIR_CURSOR  2   /* the hex you're on + the '@' */
#define PAIR_OBJ     3   /* dropped '*' objects         */
#define PAIR_HUD     4
#define PAIR_HINT    5

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR, COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,    COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
}

/* ── §4 hex mapping & lattice ── */

/* GridCtx — the hex grid for one frame: window size, hex size/look, and where
 * hex (0,0) lands on screen (recomputed on resize so the grid stays centred). */
typedef struct {
    int    rows, cols;        /* terminal size, in character cells */
    double hex_size;          /* centre-to-corner distance, pixels */
    double border_w;          /* outline band width, 0..0.5 */
    int    ox, oy;            /* screen cell that hex (0,0) sits on */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols,
                      double hex_size, double border_w)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->border_w = border_w;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
}

/* one hex's centre in pixels, measured from the grid centre (flat-top layout) */
static void hex_center_pixel(double size, int q, int r, double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5 * (double)q;
    *cy = size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
}

/* recipe step 1 — hex address (q,r) -> the screen cell at its centre. Rounds to
 * nearest so a dropped object lands dead-centre in its hex. */
static void axial_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double cx, cy;
    hex_center_pixel(g->hex_size, q, r, &cx, &cy);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/* the reverse — a pixel offset from grid centre -> fractional hex (q,r,s).
 * s = -q-r, so the three coordinates always sum to zero. */
static void screen_to_axial_frac(const GridCtx *g, double px, double py,
                                 double *fq, double *fr, double *fs)
{
    *fq = (2.0 / 3.0 * px) / g->hex_size;
    *fr = (-1.0 / 3.0 * px + sqrt(3.0) / 3.0 * py) / g->hex_size;
    *fs = -*fq - *fr;
}

/* snap fractional (q,r,s) to the nearest real hex. Rounding each on its own can
 * push their sum off zero, so we re-derive whichever we rounded most. */
static void cube_round(double fq, double fr, double fs, int *q, int *r)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *q = -rr - rs; *r = rr; }
    else if (dr > ds)            { *q = rq; *r = -rq - rs; }
    else                         { *q = rq; *r = rr; }
}

/* how near a hex edge a fractional point is: 0 at the centre, 0.5 at an edge */
static double hex_edge_distance(double fq, double fr, double fs, int q, int r)
{
    double dq = fabs(fq - (double)q);
    double dr = fabs(fr - (double)r);
    double ds = fabs(fs - (double)(-q - r));
    double d = dq;
    if (dr > d) d = dr;
    if (ds > d) d = ds;
    return d;
}

/* the ASCII glyph whose slant lies along an edge at angle theta: '-' flattish,
 * '|' steep, '/' and '\' between. The glyphs look the same flipped 180°, so we
 * fold theta into [0,pi). */
static char edge_glyph(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                            return '-';
}

/* recipe step 2 — draw the grid: for every screen cell, find its hex and how
 * near an edge it sits. Near an edge -> an outline glyph; deep inside -> blank.
 * The cursor's hex is drawn in its own colour. */
static void draw_lattice(const GridCtx *g, int curq, int curr)
{
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;

            double fq, fr, fs;
            screen_to_axial_frac(g, px, py, &fq, &fr, &fs);
            int q, r;
            cube_round(fq, fr, fs, &q, &r);

            if (hex_edge_distance(fq, fr, fs, q, r) < limit) continue;  /* inside the hex */

            double cx, cy;
            hex_center_pixel(g->hex_size, q, r, &cx, &cy);
            char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);

            int attr = (q == curq && r == curr)
                       ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                       : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5 pool — the objects placed on the grid ── */

/* Obj — one dropped object: which hex it's on (q,r) and its glyph. Storing the
 * hex address (not a screen spot) is what keeps it stuck to its hex on resize.
 * Pool — all objects packed into the front of items[]; count is how many. */
typedef struct { int q, r; char glyph; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int q, int r)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].q == q && p->items[i].r == r)
            return i;
    return -1;
}

/* recipe step 3 (direct placement) — drop an object on this hex, or pick it back
 * up if one's already there. Removal fills the gap with the last item; order
 * never matters since each hex holds at most one. */
static void pool_toggle(Pool *p, int q, int r)
{
    int i = pool_find(p, q, r);
    if (i >= 0) {
        p->items[i] = p->items[--p->count];
        return;
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, '*' };
}

static void pool_clear(Pool *p) { p->count = 0; }

/* draw every object on top of the grid, each at its hex's centre */
static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        axial_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── §6 cursor ── */

/* Cursor — where the player is, as a hex address (q,r). Unbounded: it can roam
 * off-screen and back, since the grid is conceptually endless. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

/* what each arrow key adds to (q,r). Four arrows reach four of a hex's six
 * neighbours; the two diagonals are reached by combining moves. */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

/* draw the '@' on the player's hex. Unlike objects this truncates instead of
 * rounding, nudging '@' just inside the hex so it never lands on the outline. */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    double cx, cy;
    hex_center_pixel(g->hex_size, cur->q, cur->r, &cx, &cy);
    int col = g->ox + (int)(cx / CELL_W);
    int row = g->oy + (int)(cy / CELL_H);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7 scene ── */

static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                      double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
             " q:%+d r:%+d  obj:%d  size:%.0f  %5.1f fps ",
             cur->q, cur->r, p->count, g->hex_size, fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  spc:toggle  C:clear  r:reset  +/-:size  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        double fps)
{
    erase();
    draw_lattice(g, cur->q, cur->r);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ── §9 app ── */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) g_running = 0;
    if (sig == SIGWINCH)                 g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    screen_init();

    int rows, cols; getmaxyx(stdscr, rows, cols);
    GridCtx g;   ctx_init(&g, rows, cols, HEX_SIZE_DEFAULT, BORDER_W_DEFAULT);
    Cursor  cur; cursor_reset(&cur);
    Pool    pool; pool_clear(&pool);

    double fps = 60.0;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            ctx_init(&g, rows, cols, g.hex_size, g.border_w);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'r': cursor_reset(&cur); break;
            case 'C': pool_clear(&pool); break;
            case ' ': pool_toggle(&pool, cur.q, cur.r); break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) g.hex_size += HEX_SIZE_STEP;
                break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) g.hex_size -= HEX_SIZE_STEP;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
