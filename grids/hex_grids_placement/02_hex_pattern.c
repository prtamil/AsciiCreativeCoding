/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_hex_pattern.c — stamp shapes (disc, ring, row, col) onto a hex grid.
 *
 * Move the '@' cursor with the arrows, pick a shape with 1-4, grow or shrink
 * it with +/-, then press SPACE to "stamp" that shape onto the grid. A live
 * preview shows the shape before you commit it.
 *
 * Sister files: 01_hex_direct.c (toggle single hexes; full geometry write-up),
 *               grids/rect_grids_placement/02_patterns.c (same idea on squares).
 * Hex math reference: https://www.redblobgames.com/grids/hexagons/
 */

/*
 * THE IDEA
 *
 * Each shape is just a yes/no test on a cell's offset (dq, dr) from the cursor.
 * The key measurement is "how many hex steps away is this cell?" — hex_dist.
 * Given that one number, the four shapes are simple rules:
 *
 *   disc  — every hex within N steps          (distance <= N)
 *   ring  — only the hexes exactly N steps out (distance == N)
 *   row   — the straight line through the cursor along one axis
 *   col   — the straight line through the cursor along the other axis
 *
 * To stamp, we walk the small square of offsets from -N to +N in both
 * directions, ask each one "are you in the shape?", and keep the ones that
 * pass. The square has some waste (a disc leaves the corners empty) but it's
 * tiny — at most (2N+1)² cells, 289 for the biggest N — so we don't optimize.
 *
 * Stamped hexes live in a Pool and stay put as you move around or resize the
 * terminal. The preview redraws every frame so it always tracks the cursor.
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

/* ── §1  config ── */

#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

/* How big a shape can get. The biggest disc is 217 cells, so it fits MAX_OBJ. */
#define PAT_N_DEFAULT       3
#define PAT_N_MIN           0
#define PAT_N_MAX           8

#define MAX_OBJ            256
#define FRAME_NS    16666667LL

/* How fast the fps number reacts to change — small means smooth and steady. */
#define FPS_EWMA_ALPHA      0.05

/* ── §2  clock ── */

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

/* ── §3  color ── */

#define PAIR_GRID      1
#define PAIR_CURSOR    2
#define PAIR_OBJ       3
#define PAIR_PREVIEW   4   /* the shape outline you see before stamping */
#define PAIR_HUD       5   /* status bar, yellow */
#define PAIR_HINT      6   /* key hints, cyan */

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

/* ── §4  gridctx ── */

/*
 * GridCtx — everything we need to know to draw the hex grid right now.
 *
 * It bundles the terminal's current size with the two knobs that control how
 * the hexes look, plus where the (0,0) hex lands on screen. Bundling it means
 * a redraw or a resize just rebuilds this one struct.
 */
typedef struct {
    int    rows, cols;   /* terminal size in characters, refreshed on resize */
    double hex_size;     /* radius of a hex in sub-pixels; bigger = fewer, larger hexes */
    double border_w;     /* hex border thickness, 0..0.5 of the hex (0.10 = thin) */
    int    ox, oy;       /* screen column/row where hex (0,0) sits — the grid's anchor */
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

/*
 * Snaps a fractional hex position to the nearest real hex cell.
 * Rounding each coordinate on its own can drift off the grid, so we round all
 * three and nudge back the one that moved the most. Full write-up in
 * 01_hex_direct.c §4.
 */
static void cube_round(double fq, double fr, double fs, int *q, int *r)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *q = -rr - rs; *r = rr; }
    else if (dr > ds)             { *q = rq; *r = -rq - rs; }
    else                          { *q = rq; *r = rr; }
}

/*
 * Turns a hex (q, r) into the screen cell where its center lands.
 * Terminal cells are taller than they are wide, so we squash by CELL_W/CELL_H
 * to keep the hexes looking round rather than stretched.
 */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *col, int *row)
{
    double sq3 = sqrt(3.0);
    double cx  = g->hex_size * 1.5 * (double)q;
    double cy  = g->hex_size * (sq3 * 0.5 * (double)q + sq3 * (double)r);
    *col = g->ox + (int)round(cx / CELL_W);
    *row = g->oy + (int)round(cy / CELL_H);
}

/*
 * How many hex steps apart two hexes are — the heart of every shape rule.
 * Think of it as walking from one hex to the other, one neighbor at a time;
 * this counts the fewest steps. (Formula: (|dq|+|dr|+|dq+dr|)/2.)
 */
static int hex_dist(int q1, int r1, int q2, int r2)
{
    int dq = q2 - q1, dr = r2 - r1;
    return (abs(dq) + abs(dr) + abs(dq + dr)) / 2;
}

/*
 * Picks the ASCII character that best traces a hex edge at a given angle.
 * Each border pixel knows which way the edge runs there, and we pick the
 * line glyph (- \ | /) closest to that direction so the outline looks smooth.
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/*
 * Draws the whole hex grid by asking, for each screen cell, "which hex am I in,
 * and am I on its border?" Only border cells get a line glyph, which leaves the
 * hex interiors empty. The hex under the cursor is highlighted. Full pipeline
 * write-up in 01_hex_direct.c §4.
 */
static void ctx_draw_bg(const GridCtx *g, int curq, int curr)
{
    double size  = g->hex_size;
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int q, r;
            cube_round(fq, fr, fs, &q, &r);
            double fS   = (double)(-q - r);
            double dist = fabs(fq - (double)q);
            double d2   = fabs(fr - (double)r);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)q;
            double cy    = size * (sq3_2 * (double)q + sq3 * (double)r);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            int on_cur = (q == curq && r == curr);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ── §5  pool ── */

/* One stamped hex: where it is (q, r) and which character marks it. */
typedef struct { int q, r; char glyph; } Obj;

/*
 * Pool — the bag of hexes the user has stamped so far.
 *
 * Just a fixed array plus how many slots are used. A plain array is plenty:
 * at most a few hundred hexes and we only ever scan or append. It never grows
 * past MAX_OBJ, so there's no allocation and nothing to free.
 */
typedef struct {
    Obj items[MAX_OBJ];  /* stamped hexes, slots 0..count-1 are live */
    int count;           /* how many slots are in use, 0..MAX_OBJ */
} Pool;

/*
 * Records a stamped hex, or refreshes one already there so we never store the
 * same cell twice. Stamping a new shape over an old one just changes its mark.
 */
static void pool_place(Pool *p, int q, int r, char glyph)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].q == q && p->items[i].r == r) {
            p->items[i].glyph = glyph;
            return;
        }
    }
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ q, r, glyph };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        ctx_to_screen(g, p->items[i].q, p->items[i].r, &col, &row);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── §6  cursor ── */

/* Cursor — which hex the '@' is sitting on, in hex (q, r) coordinates. */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur) { cur->q = 0; cur->r = 0; }

static const int HEX_DIR[4][2] = {
    { 0, -1 }, { 0, +1 }, {-1, 0}, {+1, 0}
};

static void cursor_move(Cursor *cur, int dq, int dr)
{
    cur->q += dq; cur->r += dr;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx    = g->hex_size * 1.5    * (double)cur->q;
    double cy    = g->hex_size * (sq3_2 * (double)cur->q + sq3 * (double)cur->r);
    int col = g->ox + (int)(cx / CELL_W);
    int row = g->oy + (int)(cy / CELL_H);
    if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §7  patterns ── */

/*
 * The four shapes you can stamp. N_PAT is just the count, handy for sizing
 * the glyph and name tables below and for cycling through modes.
 */
typedef enum { PAT_DISC=0, PAT_RING=1, PAT_ROW=2, PAT_COL=3, N_PAT=4 } PatMode;

/* The mark each shape leaves, one per mode. We skip '-' and '|' on purpose:
 * they'd blend into the hex border characters and you couldn't tell them apart. */
static const char PAT_GLYPH[N_PAT] = { '*', 'o', '=', ':' };
static const char *PAT_NAME[N_PAT] = { "disc", "ring", "row", "col" };

/*
 * The yes/no test at the core of every shape: is this offset part of the shape?
 * Each mode is a one-line rule on the hex distance, so adding a new shape later
 * is just adding a case. We test every offset in the box rather than cleverly
 * tracing each shape — slower in theory, but trivially small and easy to read.
 */
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

/*
 * Draws the live preview: outlines every hex the current shape would cover, in
 * bright green, without touching the pool. It reuses the grid-drawing loop but
 * skips any cell that isn't part of the shape. We outline whole hexes rather
 * than dotting their centers so the shape reads clearly at a glance.
 */
static void pat_overlay(const GridCtx *g, PatMode mode, int N,
                         int curq, int curr)
{
    double size  = g->hex_size;
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    attron(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * CELL_W;
            double py = (double)(row - g->oy) * CELL_H;
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;
            int q, r;
            cube_round(fq, fr, fs, &q, &r);
            if (!pat_test(mode, q - curq, r - curr, N)) continue;
            double fS   = (double)(-q - r);
            double dist = fabs(fq - (double)q);
            double d2   = fabs(fr - (double)r);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;
            double cx    = size * 1.5 * (double)q;
            double cy    = size * (sq3_2 * (double)q + sq3 * (double)r);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_PREVIEW) | A_BOLD);
}

/*
 * Commits the shape: walks every offset in the box and adds each hex that
 * passes the test to the pool. This is the one that actually changes state;
 * the preview only looks.
 */
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

/* ── §8  scene ── */

/*
 * SceneCfg — the user's current choices: which shape, how big, preview on/off.
 * Keeping these together lets the draw and key-handling code pass one struct
 * around instead of three loose variables.
 */
typedef struct {
    PatMode pat_mode;     /* which shape is selected (disc/ring/row/col) */
    int     pat_n;        /* its size, PAT_N_MIN..PAT_N_MAX */
    int     show_preview; /* 1 = show the live outline, 0 = hide it */
} SceneCfg;

/* Status bar (top-right) and the key hints (bottom row). */
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
    ctx_draw_bg(g, cur->q, cur->r);
    pool_draw(p, g);
    if (cfg->show_preview)
        pat_overlay(g, cfg->pat_mode, cfg->pat_n, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, p, cfg, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9  screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(screen_cleanup);
}

/* ── §10  app ── */

/* Set from signal handlers, so they must be the safe-to-touch-async type. */
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
