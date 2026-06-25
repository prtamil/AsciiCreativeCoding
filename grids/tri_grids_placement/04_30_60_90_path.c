/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_30_60_90_path.c — straight-line path between two triangles
 *
 * Drop a START marker (S) and an END marker (E) on the triangle grid,
 * and the program highlights every triangle that a straight line between
 * them passes through. Move '@' with the arrows; 's' sets START, 'e' sets
 * END. The grid here is the kisrhombille tiling — plain triangles with
 * three extra lines (medians) drawn inside each one for decoration.
 *
 * Sister files: 04_30_60_90_direct.c (place markers by hand),
 *               01_equilateral_path.c (same idea, plain triangles).
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W   0.10
#define MEDIAN_T   0.05

#define MAX_OBJ    1024
#define N_THEMES   4

#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_MEDIAN 2
#define PAIR_CURSOR 3
#define PAIR_START  4
#define PAIR_END    5
#define PAIR_PATH   6
#define PAIR_HUD    7
#define PAIR_HINT   8

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static const short THEME_FG[N_THEMES][3] = {
    /* edge,  start,  end */
    {  75,  82, 196 },
    {  39, 226, 207 },
    { 207,  82,  39 },
    {  15,  82, 196 },
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_GREEN,   COLOR_RED     },
    { COLOR_BLUE,    COLOR_YELLOW,  COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_GREEN,   COLOR_BLUE    },
    { COLOR_WHITE,   COLOR_GREEN,   COLOR_RED     },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_s = THEME_FG[theme][1]; fg_n = THEME_FG[theme][2];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_s = THEME_FG_8[theme][1]; fg_n = THEME_FG_8[theme][2];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_MEDIAN, COLORS >= 256 ?  39 : COLOR_BLUE,   -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 gridctx ── */

typedef struct {
    int    rows, cols;
    int    cw, ch;
    int    ox, oy;
    double tri_size;
    double border_w;
    double median_t;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, double tri_size)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->tri_size = tri_size;
    g->border_w = BORDER_W;
    g->median_t = MEDIAN_T;
}

static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}

static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

static char tri_edge_char(int up, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    if (up == 0) {
        l1 = 1.0 - fa - fb; ch1 = '/';
        l2 = fa;            ch2 = '\\';
        l3 = fb;            ch3 = '_';
    } else {
        l1 = 1.0 - fb;       ch1 = '_';
        l2 = fa + fb - 1.0;  ch2 = '/';
        l3 = 1.0 - fa;       ch3 = '\\';
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

static char tri_median_char(int up, double fa, double fb, double *out_min)
{
    static const double INV_SQRT2 = 0.70710678118654752440;
    static const double INV_SQRT5 = 0.44721359549995793928;
    double m1, m2, m3; char ch1, ch2, ch3;
    if (up == 0) {
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(fa + 2.0*fb - 1) * INV_SQRT5; ch2 = '/';
        m3 = fabs(2.0*fa + fb - 1) * INV_SQRT5; ch3 = '|';
    } else {
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(2.0*fa + fb - 2) * INV_SQRT5; ch2 = '|';
        m3 = fabs(fa + 2.0*fb - 2) * INV_SQRT5; ch3 = '/';
    }
    char ch = ch1; double m = m1;
    if (m2 < m) { m = m2; ch = ch2; }
    if (m3 < m) { m = m3; ch = ch3; }
    *out_min = m;
    return ch;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx, &cy);
    *scol = g->ox + (int)(cx / g->cw);
    *srow = g->oy + (int)(cy / g->ch);
}

static void ctx_draw_bg(const GridCtx *g)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;
            int tC, tR, tU; double fa, fb, em, mm;
            pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
            char ech = tri_edge_char  (tU, fa, fb, &em);
            char mch = tri_median_char(tU, fa, fb, &mm);
            if (em < g->border_w && em <= mm) {
                attron(COLOR_PAIR(PAIR_BORDER));
                mvaddch(row, col, (chtype)(unsigned char)ech);
                attroff(COLOR_PAIR(PAIR_BORDER));
            } else if (mm < g->median_t) {
                attron(COLOR_PAIR(PAIR_MEDIAN));
                mvaddch(row, col, (chtype)(unsigned char)mch);
                attroff(COLOR_PAIR(PAIR_MEDIAN));
            }
        }
    }
}

/* ── §5 pool ── */

typedef struct { int col, row, up; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ];  int count; } Pool;

static int pool_find(const Pool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive
            && p->items[i].col == col
            && p->items[i].row == row
            && p->items[i].up  == up)
            return i;
    return -1;
}

static void pool_place(Pool *p, int col, int row, int up, char glyph)
{
    if (pool_find(p, col, row, up) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ col, row, up, glyph, true };
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sc, sr;
        ctx_to_screen(g, p->items[i].col, p->items[i].row, p->items[i].up,
                      &sc, &sr);
        if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ── §6 cursor ── */

typedef struct {
    int col, row, up;
    int sCol, sRow, sUp;        /* where START sits */
    int eCol, eRow, eUp;        /* where END sits */
    int has_start, has_end;
    int theme;
    int paused;
} Cursor;

static const int TRI_DIR[4][2][3] = {
    { { -1,  0,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, { +1,  0,  0 } },
    { {  0, -1,  1 }, {  0,  0,  0 } },
    { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->up = 0;
    cur->has_start = 0; cur->has_end = 0;
    cur->theme  = 0;
    cur->paused = 0;
}

static void cursor_move(Cursor *cur, int arrow)
{
    const int *t = TRI_DIR[arrow][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sc, sr;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, '@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

static void marker_draw(const GridCtx *g, int col, int row, int up,
                        char glyph, int pair)
{
    int sc, sr;
    ctx_to_screen(g, col, row, up, &sc, &sr);
    if (sc < 0 || sc >= g->cols || sr < 0 || sr >= g->rows - 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sr, sc, glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* ── §7 mode — walk the line, collect the triangles it crosses ── */

/* Steps along the straight line from START to END, checking which
 * triangle each step lands in. Re-run whenever a marker or the size
 * changes, since both shift where that line falls. */
static void path_compute(Pool *p, const Cursor *cur, const GridCtx *g)
{
    pool_clear(p);
    if (!cur->has_start || !cur->has_end) return;

    double sx, sy, ex, ey;
    tri_centroid_pixel(cur->sCol, cur->sRow, cur->sUp, g->tri_size, &sx, &sy);
    tri_centroid_pixel(cur->eCol, cur->eRow, cur->eUp, g->tri_size, &ex, &ey);

    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) {
        pool_place(p, cur->sCol, cur->sRow, cur->sUp, '*');
        return;
    }
    double step = g->tri_size * 0.25;
    int    n    = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t  = (double)i / (double)n;
        double px = sx + t * dx;
        double py = sy + t * dy;
        int    tC, tR, tU; double fa, fb;
        pixel_to_tri(px, py, g->tri_size, &tC, &tR, &tU, &fa, &fb);
        pool_place(p, tC, tR, tU, '*');
    }
}

/* ── §8 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, const Pool *path,
                     double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             path->count, g->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [04 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, const Pool *path,
                       double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(path, g);
    if (cur->has_start)
        marker_draw(g, cur->sCol, cur->sRow, cur->sUp, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(g, cur->eCol, cur->eRow, cur->eUp, 'E', PAIR_END);
    cursor_draw(cur, g);
    hud_draw(g, cur, path, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §10 app ── */

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

    Cursor cur;  cursor_reset(&cur);
    Pool   path; pool_clear(&path);
    screen_init(cur.theme);

    GridCtx g;   ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, g.tri_size);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r':
                    cursor_reset(&cur); pool_clear(&path);
                    ctx_init(&g, LINES, COLS, TRI_SIZE_DEFAULT);
                    color_init(cur.theme);
                    break;
                case ' ':
                    cur.has_start = 0; cur.has_end = 0; pool_clear(&path);
                    break;
                case 's':
                    cur.sCol = cur.col; cur.sRow = cur.row; cur.sUp = cur.up;
                    cur.has_start = 1; path_compute(&path, &cur, &g);
                    break;
                case 'e':
                    cur.eCol = cur.col; cur.eRow = cur.row; cur.eUp = cur.up;
                    cur.has_end = 1; path_compute(&path, &cur, &g);
                    break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, 0); break;
                case KEY_RIGHT: cursor_move(&cur, 1); break;
                case KEY_UP:    cursor_move(&cur, 2); break;
                case KEY_DOWN:  cursor_move(&cur, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) {
                        g.tri_size += TRI_SIZE_STEP; path_compute(&path, &cur, &g);
                    } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) {
                        g.tri_size -= TRI_SIZE_STEP; path_compute(&path, &cur, &g);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA)
            + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &cur, &path, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
