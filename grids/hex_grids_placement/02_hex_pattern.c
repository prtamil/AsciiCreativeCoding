/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_hex_pattern.c — stamp a shape (disc, ring, row, col) of objects onto a hex grid.
 *
 * Move the '@' cursor, pick a shape with 1-4, size it with +/-, SPACE stamps it.
 * Each shape is a yes/no test on a cell's hex-offset from the cursor; stamping
 * walks the box of offsets and keeps those that pass. A live preview tracks the
 * cursor without touching the pool. Hex mapping/lattice is the same recipe as
 * 01_hex_direct.c — this file's distinct step is the pattern offset table + stamp.
 *
 * Sister files: 01_hex_direct.c (toggle single hexes; full geometry write-up),
 *               grids/rect_grids_placement/02_patterns.c (same idea on squares).
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

#define PAT_N_DEFAULT       3     /* shape radius in hex steps; biggest disc = 217 hexes */
#define PAT_N_MIN           0
#define PAT_N_MAX           8

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

#define PAIR_GRID      1   /* the hex outlines                       */
#define PAIR_CURSOR    2   /* the hex you're on + the '@'            */
#define PAIR_OBJ       3   /* stamped objects                        */
#define PAIR_PREVIEW   4   /* the shape outline you see before stamping */
#define PAIR_HUD       5
#define PAIR_HINT      6

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,    COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,     COLORS >= 256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_PREVIEW, COLORS >= 256 ?  82 : COLOR_GREEN,   -1);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,    -1);
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
 * nearest so a stamped object lands dead-centre in its hex. */
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

/* how many hex steps apart two hexes are — the measure every shape rule uses.
 * Formula: (|dq|+|dr|+|dq+dr|)/2, the fewest one-neighbour moves between them. */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
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

/* Obj — one stamped object: which hex it's on (q,r) and its glyph. Storing the
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

/* add a hex to the pool, or refresh its glyph if one's already there — stamping
 * a new shape over an old one just changes the mark, never doubles up the cell. */
static void pool_place(Pool *p, int q, int r, char glyph)
{
    int i = pool_find(p, q, r);
    if (i >= 0) {
        p->items[i].glyph = glyph;
        return;
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, glyph };
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

/* ── §7 patterns — this file's distinct step: a shape of objects by hex offset ── */

/* The four shapes you can stamp. N_PAT is the count, used to size the tables
 * below and to bound mode selection. */
typedef enum { PAT_DISC=0, PAT_RING=1, PAT_ROW=2, PAT_COL=3, N_PAT=4 } PatMode;

/* one entry per mode: the mark it leaves and its HUD name. We skip '-' and '|'
 * as marks — they'd blend into the hex outline glyphs. */
static const char  PAT_GLYPH[N_PAT] = { '*', 'o', '=', ':' };
static const char *PAT_NAME [N_PAT] = { "disc", "ring", "row", "col" };

/* the yes/no test at the core of every shape: is offset (dq,dr) from the cursor
 * part of the shape of size N? Each mode is a one-line rule on the hex distance,
 * so a new shape is just a new case. */
static int pat_test(PatMode mode, int dq, int dr, int N)
{
    int d = hex_dist(0, 0, dq, dr);
    switch (mode) {
    case PAT_DISC: return d <= N;
    case PAT_RING: return d == N;
    case PAT_ROW:  return dr == 0 && abs(dq) <= N;
    case PAT_COL:  return dq == 0 && abs(dr) <= N;
    default:       return 0;
    }
}

/* live preview — outline (in green) every hex the current shape would cover,
 * without touching the pool. Same lattice walk as draw_lattice, but it skips any
 * cell whose hex isn't in the shape. */
static void pat_overlay(const GridCtx *g, PatMode mode, int N,
                         int curq, int curr)
{
    double limit = 0.5 - g->border_w;

    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;

            double fq, fr, fs;
            screen_to_axial_frac(g, px, py, &fq, &fr, &fs);
            int q, r;
            cube_round(fq, fr, fs, &q, &r);

            if (!pat_test(mode, q - curq, r - curr, N)) continue;
            if (hex_edge_distance(fq, fr, fs, q, r) < limit) continue;  /* inside the hex */

            double cx, cy;
            hex_center_pixel(g->hex_size, q, r, &cx, &cy);
            char ch = edge_glyph(atan2(py - cy, px - cx) + M_PI / 2.0);
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

/* recipe step 3 (pattern placement) — commit the shape: walk the box of offsets
 * -N..+N in both axes, and add to the pool every hex that passes the test. */
static void pat_stamp(Pool *pool, PatMode mode, int N, int curq, int curr)
{
    char glyph = PAT_GLYPH[mode];
    for (int dr = -N; dr <= N; dr++) {
        for (int dq = -N; dq <= N; dq++) {
            if (!pat_test(mode, dq, dr, N)) continue;
            pool_place(pool, curq + dq, curr + dr, glyph);
        }
    }
}

/* ── §8 scene ── */

/* SceneCfg — the user's current choices: which shape, how big, preview on/off. */
typedef struct {
    PatMode pat_mode;     /* which shape is selected (disc/ring/row/col) */
    int     pat_n;        /* its size, PAT_N_MIN..PAT_N_MAX */
    int     show_preview; /* 1 = show the live outline, 0 = hide it */
} SceneCfg;

static void hud_draw(const GridCtx *g, const Pool *p, const SceneCfg *cfg,
                      double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " %s N:%d  obj:%d  %5.1f fps  prev:%s ",
             PAT_NAME[cfg->pat_mode], cfg->pat_n, p->count, fps,
             cfg->show_preview ? "on " : "off");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  1-4:pattern  +/-:N  spc:stamp  p:preview  C:clear  r:reset  q/ESC:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                        const SceneCfg *cfg, double fps)
{
    erase();
    draw_lattice(g, cur->q, cur->r);
    pool_draw(p, g);
    if (cfg->show_preview)
        pat_overlay(g, cfg->pat_mode, cfg->pat_n, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, p, cfg, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ── §10 app ── */

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
    SceneCfg cfg = { PAT_DISC, PAT_N_DEFAULT, 1 };

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
            case 'p': cfg.show_preview ^= 1; break;
            case '1': cfg.pat_mode = PAT_DISC; break;
            case '2': cfg.pat_mode = PAT_RING; break;
            case '3': cfg.pat_mode = PAT_ROW;  break;
            case '4': cfg.pat_mode = PAT_COL;  break;
            case ' ': pat_stamp(&pool, cfg.pat_mode, cfg.pat_n, cur.q, cur.r); break;
            case KEY_UP:    cursor_move(&cur, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (cfg.pat_n < PAT_N_MAX) cfg.pat_n++;
                break;
            case '-':
                if (cfg.pat_n > PAT_N_MIN) cfg.pat_n--;
                break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;

        scene_draw(&g, &pool, &cur, &cfg, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
