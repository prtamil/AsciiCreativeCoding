/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_30_60_90.c — equilateral triangle grid, each triangle sliced into six.
 *
 * Builds on 01's equilateral grid (§4 screen_to_tri / edge_glyph / draw_lattice
 * are shared verbatim). DISTINCT here: draw_lattice also overlays each
 * triangle's three medians (corner -> opposite-side midpoint). The medians cut
 * each triangle into six 30-60-90 right triangles — median_glyph picks which
 * median line a point is nearest. The cursor still walks whole triangles only.
 *
 * Sister: 01_equilateral.c (same grid math), 03_double_diagonal.c (slice-tile).
 * Refs: Kisrhombille tiling  https://en.wikipedia.org/wiki/Kisrhombille_tiling
 *       30-60-90 triangle    https://en.wikipedia.org/wiki/Special_right_triangle
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

#define MEDIAN_T 0.05   /* how close to a median counts as "on" it; thicker = bigger */

#define N_THEMES 4

#define FPS_EWMA_ALPHA 0.05     /* small = steadier on-screen fps number */

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

/* ── §4 tri mapping & lattice ── */

/* GridCtx — the triangle grid for one frame. Same as 01's, plus median_t for
 * the three internal slicing lines. Centred on screen, centre cell = origin
 * (0,0). tri_size/border_w/median_t live here (not as constants) so +/- [/]
 * tune them live. The plane is infinite; max_col/row are a rough reach. */
typedef struct {
    int    rows, cols;       /* terminal size in cells */
    double tri_size;         /* triangle side length, sub-pixels */
    double border_w;         /* how close to an outer edge still counts as on it */
    double median_t;         /* how close to a median still counts as on it */
    int    cw, ch;           /* sub-pixels per cell (CELL_W, CELL_H) */
    int    ox, oy;           /* centre cell = grid origin (0,0) */
    int    max_col, max_row; /* rough on-screen reach, not a hard boundary */
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

/* recipe step 1 (reverse) — a pixel -> which triangle (col,row,up) it lands in,
 * plus where inside it (fa,fb). Undo the slant: a = px/size - b/2. Whole parts
 * pick the diamond; fa+fb >= 1 means the up triangle. Same as 01. */
static void screen_to_tri(const GridCtx *g, double px, double py,
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

/* the middle of a triangle, in pixels (forward: triangle -> point) */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

/* the terminal cell at a triangle's middle. Truncates (not rounds) on purpose,
 * nudging '@' inside so it never lands on an outline char and hides. */
static void tri_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/* the line char for a point inside a triangle, by nearest outer edge: '/', '\',
 * '_'. out_min returns the distance, so the caller can skip deep-inside points.
 * Same as 01. */
static char edge_glyph(int up, double fa, double fb, double *out_min)
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

/* THE DISTINCT MATH — the three medians that cut each triangle into six 30-60-90
 * right triangles. Each median is a line "expr = 0" in (fa,fb); the point's
 * distance to it is |expr| scaled by 1/|normal| so the three compare fairly.
 * Nearest median wins and its glyph ('\', '/', '|') is returned. Up- and
 * down-pointing triangles use different line equations, so split the two. */
static char median_glyph(int up, double fa, double fb, double *out_min)
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

/* recipe step 2 — draw the grid: every cell -> its triangle, then the nearest
 * outer edge and nearest median. An outer edge within border_w wins (and ties)
 * and is drawn; else a median within median_t is drawn; else empty interior.
 * The cursor's triangle is recoloured. */
static void draw_lattice(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb, em, mm;
            screen_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);
            char ech = edge_glyph(tU, fa, fb, &em);
            char mch = median_glyph(tU, fa, fb, &mm);

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

/* Cursor — which whole triangle is selected: col/row pick the diamond, up its
 * half (0 = down, 1 = up). The medians are decoration; the cursor never sits on
 * one of the six little triangles. Same as 01. */
typedef struct { int col, row, up; } Cursor;

/* move table: [arrow][current half] -> (d_col, d_row, new_up). When an arrow has
 * no edge to cross, the entry just flips to the other half of the same diamond.
 * arrows: 0=LEFT 1=RIGHT 2=UP 3=DOWN; halves: 0=down, 1=up. Same as 01. */
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

/* recipe step 3 — step the cursor one triangle. No clamp; the plane is infinite. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dir)
{
    (void)g;
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0];
    cur->row += t[1];
    cur->up   = t[2];
}

/* put '@' in the cursor's triangle; after the grid so it lands on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    tri_to_screen(g, cur->col, cur->row, cur->up, &sr, &sc);
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
    draw_lattice(g, cur->col, cur->row, cur->up);
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
