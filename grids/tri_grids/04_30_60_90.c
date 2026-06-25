/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_30_60_90.c — a grid of equilateral triangles, each sliced into six.
 *
 * We start with the equilateral grid from 01 and draw three lines inside
 * every triangle, each running from a corner to the middle of the opposite
 * side. Those three lines cut each triangle into six smaller right triangles
 * (the 30-60-90 kind). Arrow keys walk a cursor over the whole triangles;
 * the little ones are just lines on screen, nothing we store.
 *
 * Companion files: 01_equilateral.c (same grid math) and 03_double_diagonal.c
 * (the same "slice each tile" idea, but applied to squares).
 *
 * Names for the curious: this pattern is the "kisrhombille tiling", and the
 * three slicing lines are a triangle's "medians".
 *   https://en.wikipedia.org/wiki/Kisrhombille_tiling
 *   https://en.wikipedia.org/wiki/Special_right_triangle  (30-60-90 triangle)
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

/* ── §1 config ── */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.30
#define BORDER_W_STEP    0.02

/* How close a point must be to a slicing line to count as "on" it.
   Bigger = thicker lines; smaller = thinner. Tuned by eye. */
#define MEDIAN_T 0.05

#define N_THEMES 4

/* The FPS number jitters frame to frame, so we smooth it: each frame nudges
   the shown value a little toward the latest reading instead of replacing it. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_BORDER 1
#define PAIR_MEDIAN 2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

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

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  median */
    {  75,  39 },
    {  82, 226 },
    { 207, 196 },
    {  15,  87 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,    COLOR_BLUE   },
    { COLOR_GREEN,   COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_RED    },
    { COLOR_WHITE,   COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_m;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_m = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_m = THEME_FG_8[theme][1]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_MEDIAN, fg_m, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ?  15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — grid geometry, pixel↔triangle, edge + median ── */

/*
 * GridCtx — everything we need to know about the current grid: how big the
 * triangles are, how thick the lines are, and where the grid sits on screen.
 * All the drawing math reads from one of these. Same geometry as 01, plus the
 * median_t setting for the three internal slicing lines.
 */
typedef struct {
    /* size of the terminal window, in character cells */
    int rows, cols;

    /* the triangles */
    double tri_size;       /* triangle side length, in pixels                 */
    double border_w;       /* how close to an outer edge counts as "on" it    */
    double median_t;       /* how close to a slicing line counts as "on" it   */
    int    cw, ch;         /* pixels per character cell (CELL_W, CELL_H);
                              terminal cells are taller than wide, so these
                              differ to keep triangles looking even           */

    /* where pixel (0,0) lands on screen — we centre the grid here */
    int    ox, oy;

    /* rough count of triangles that fit across/down; advisory only */
    int    max_col, max_row;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows;
    g->cols = cols;
    g->cw   = CELL_W;
    g->ch   = CELL_H;
    g->ox   = cols / 2;
    g->oy   = (rows - 1) / 2;
    if (g->tri_size <= 0.0) g->tri_size = TRI_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    if (g->median_t <= 0.0) g->median_t = MEDIAN_T;
    g->max_col = (int)((double)cols * CELL_W / g->tri_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / (sqrt(3.0) * 0.5 * g->tri_size)) + 1;
}

/*
 * Given a pixel, tell us which triangle it falls in and where inside it.
 * (fa, fb) are how far along the triangle's two slanted axes we are — like
 * reading off a tilted graph-paper grid. Same math as 01_equilateral.c.
 */
static void ctx_pixel_to_tri(const GridCtx *g, double px, double py,
                             int *col, int *row, int *up,
                             double *fa, double *fb)
{
    double h = g->tri_size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / g->tri_size - 0.5 * b;
    int    c = (int)floor(a);
    int    r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}

/*
 * Find the centre point of a given triangle, in pixels — that's where we
 * park the cursor mark. Same map as 01.
 */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * For a point inside a triangle, find which of the three outer edges is
 * closest, and report that distance plus the character that draws it
 * (/, \, or _). Same as 01_equilateral.c.
 */
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
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * For a point inside a triangle, find which of the three slicing lines is
 * closest, and report that distance plus the character that draws it.
 * Each line is described by a little equation; how far the point is from a
 * line is just how far that equation is from zero (scaled so different lines
 * compare fairly). The closest of the three wins. A triangle points either
 * down or up, and the two cases use different lines, so we handle them apart.
 */
static char tri_median_char(int up, double fa, double fb, double *out_min)
{
    static const double INV_SQRT2 = 0.70710678118654752440;
    static const double INV_SQRT5 = 0.44721359549995793928;
    double m1, m2, m3;
    char   ch1, ch2, ch3;
    if (up == 0) {                          /* downward-pointing triangle */
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(fa + 2.0*fb - 1.0) * INV_SQRT5; ch2 = '/';
        m3 = fabs(2.0*fa + fb - 1.0) * INV_SQRT5; ch3 = '|';
    } else {                                /* upward-pointing triangle */
        m1 = fabs(fa - fb)         * INV_SQRT2; ch1 = '\\';
        m2 = fabs(2.0*fa + fb - 2.0) * INV_SQRT5; ch2 = '|';
        m3 = fabs(fa + 2.0*fb - 2.0) * INV_SQRT5; ch3 = '/';
    }
    char   ch = ch1;
    double m  = m1;
    if (m2 < m) { m = m2; ch = ch2; }
    if (m3 < m) { m = m3; ch = ch3; }
    *out_min = m;
    return ch;
}

/*
 * Walk every character cell on screen and decide what, if anything, it shows.
 * For each cell we find its triangle and how close it is to an outer edge and
 * to a slicing line. An outer edge wins ties and gets drawn first; otherwise a
 * nearby slicing line gets drawn; otherwise the cell is empty interior. Cells
 * inside the cursor's triangle are recoloured to highlight it.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb, em, mm;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);
            char ech = tri_edge_char(tU, fa, fb, &em);
            char mch = tri_median_char(tU, fa, fb, &mm);

            int on_cur = (tC == cC && tR == cR && tU == cU);
            if (em < g->border_w && em <= mm) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ech);
                attroff(attr);
            } else if (mm < g->median_t) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : COLOR_PAIR(PAIR_MEDIAN);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)mch);
                attroff(attr);
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — which triangle is currently selected. col/row pick the spot on the
 * grid; up says whether it's the upward- or downward-pointing triangle there.
 * The slicing lines are just decoration — the cursor only ever sits on a whole
 * triangle, never one of the six little ones. Same as 01_equilateral.c.
 */
typedef struct { int col, row, up; } Cursor;

/* Lookup table for moving the cursor. Stepping in a direction can flip an
   up triangle to a down one (or move to a neighbour), so each direction
   stores how col/row/up change. Same table as 01_equilateral.c. */
static const int TRI_DIR[4][2][3] = {
    /* LEFT  */ { { -1,  0,  1 }, {  0,  0,  0 } },
    /* RIGHT */ { {  0,  0,  1 }, { +1,  0,  0 } },
    /* UP    */ { {  0, -1,  1 }, {  0,  0,  0 } },
    /* DOWN  */ { {  0,  0,  1 }, {  0, +1,  0 } },
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->col = 0;
    cur->row = 0;
    cur->up  = 0;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->col, cur->row, cur->up, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             g->tri_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [04 30-60-90] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
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
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.tri_size = TRI_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    g.median_t = MEDIAN_T;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           paused ^= 1; break;
                case 'r':           cursor_reset(&cur, &g); break;
                case 't':
                    theme = (theme + 1) % N_THEMES;
                    color_init(theme);
                    break;
                case KEY_LEFT:  cursor_move(&cur, &g, 0); break;
                case KEY_RIGHT: cursor_move(&cur, &g, 1); break;
                case KEY_UP:    cursor_move(&cur, &g, 2); break;
                case KEY_DOWN:  cursor_move(&cur, &g, 3); break;
                case '+': case '=':
                    if (g.tri_size < TRI_SIZE_MAX) { g.tri_size += TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '-':
                    if (g.tri_size > TRI_SIZE_MIN) { g.tri_size -= TRI_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
                case '[':
                    if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - t0; t0 = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
