/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric.c — a triangle grid colored to look like a stack of cubes.
 *
 * It's the same triangular tiling as 01_equilateral.c, but instead of
 * drawing the edges between triangles, we fill each triangle with one of
 * six colors. Picked the right way, the colors trick the eye into seeing
 * a wall of stacked 3-D cubes (the classic isometric look). Arrow keys
 * walk a cursor from triangle to triangle.
 *
 * Sister file: 01_equilateral.c — same grid and same cursor logic; that
 * one draws the edges, this one fills the cells with color.
 *
 * Why six triangles meet at a vertex and why that reads as cubes:
 *   Isometric projection — https://en.wikipedia.org/wiki/Isometric_projection
 *   Triangular tiling    — https://en.wikipedia.org/wiki/Triangular_tiling
 *
 * Keys: arrows move @   r reset   t theme   p pause   +/- size   q/ESC quit
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

#define TRI_SIZE_DEFAULT 16.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define N_PALETTE  6
#define N_THEMES   3

/* How fast the FPS number on screen reacts. Small = smooth and steady. */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_FILL_BASE  1                              /* 1..6 */
#define PAIR_CURSOR    (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_HUD       (PAIR_CURSOR + 1)
#define PAIR_HINT      (PAIR_HUD + 1)

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

/*
 * Three color themes, six colors each. Press 't' to cycle them. We keep two
 * copies: rich 256-color values for modern terminals, and a plain 8-color
 * version as a fallback for old ones.
 */
static const short PAL256[N_THEMES][N_PALETTE] = {
    /* warm  */ { 196, 214, 226, 118, 39, 129 },
    /* cool  */ {  21,  39,  82, 226, 207, 21  },
    /* mono  */ {  15,  87,  39,   0, 39, 87   },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,  COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE, COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE,COLOR_CYAN,   COLOR_BLUE,  COLOR_BLACK,   COLOR_BLUE,    COLOR_CYAN    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        /* We fill cells with a blank space, so only the background color shows. */
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula — screen ↔ triangle, plus the cube-coloring trick ── */

/*
 * GridCtx — everything we need to know about the current grid layout.
 *
 * It answers two questions: how big are the triangles, and where on screen
 * does the grid sit. Given a screen cell, the helpers below use these
 * numbers to figure out which triangle that cell falls inside. Same idea as
 * 01_equilateral.c, but with no edge-width field since we only fill cells.
 */
typedef struct {
    /* Size of the terminal, in character cells. */
    int rows, cols;

    double tri_size;       /* triangle side length, measured in pixels       */
    int    cw, ch;         /* how many pixels wide/tall one character cell is */

    /* Where pixel (0,0) lands on screen — we center the grid here.          */
    int    ox, oy;

    /* Rough guess at how far the cursor can roam; just an advisory bound.    */
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
    g->max_col = (int)((double)cols * CELL_W / g->tri_size) + 1;
    g->max_row = (int)((double)rows * CELL_H / (sqrt(3.0) * 0.5 * g->tri_size)) + 1;
}

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

/* Finds the center point of a triangle, so we can mark it (same as 01). */
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx_pix, double *cy_pix)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx_pix = (a + 0.5 * b) * size;
    *cy_pix = b * h;
}

__attribute__((unused))
static void ctx_to_screen(const GridCtx *g, int col, int row, int up,
                          int *sr, int *sc)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, g->tri_size, &cx_pix, &cy_pix);
    *sc = g->ox + (int)(cx_pix / g->cw);
    *sr = g->oy + (int)(cy_pix / g->ch);
}

/*
 * Picks which of the six colors a triangle gets — this is the whole trick.
 * Six triangles meet at every grid corner, and this little formula hands
 * each of those six a different color. The result reads as the three faces
 * of stacked cubes (top, left, right), so the flat grid looks 3-D.
 */
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE;
    if (k < 0) k += N_PALETTE;
    return k;
}

/*
 * Paints the whole screen. For every cell we find its triangle, look up that
 * triangle's color, and fill the cell with it. The cursor's cell gets an '@'
 * in reverse video on top, so it stands out no matter what color it sits on.
 */
static void ctx_draw_bg(const GridCtx *g, int cC, int cR, int cU)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cw;
            double py = (double)(row - g->oy) * g->ch;

            int    tC, tR, tU;
            double fa, fb;
            ctx_pixel_to_tri(g, px, py, &tC, &tR, &tU, &fa, &fb);

            int p    = palette_index(tC, tR, tU);
            int pair = PAIR_FILL_BASE + p;
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));

            int on_cur = (tC == cC && tR == cR && tU == cU);
            if (on_cur) {
                attron(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
                mvaddch(row, col, '@');
                attroff(COLOR_PAIR(pair) | A_BOLD | A_REVERSE);
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — which triangle the '@' is sitting on. Same as 01_equilateral.c.
 *   col, row : which rhombus of the grid, like a square-grid x and y.
 *   up       : 0 for the lower triangle of that rhombus, 1 for the upper one.
 * There's no separate draw function — ctx_draw_bg paints the '@' as it goes,
 * which keeps it on the right background color.
 */
typedef struct { int col, row, up; } Cursor;

/*
 * Movement table, same as 01_equilateral.c. To move in a direction we need
 * to know which triangle we're on (the 'up' flag), because a left arrow from
 * an upper triangle lands somewhere different than from a lower one. Each row
 * is a direction; the two entries inside are the up=0 and up=1 cases; the
 * three numbers are how much col, row, and up change.
 */
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

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             g->tri_size, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " arrows:move  +/-:size  t:theme  r:reset  p:pause  q:quit  [05 isometric] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->col, cur->row, cur->up);
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
    typeahead(-1);   /* stop ncurses peeking at input mid-draw; avoids tearing */
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
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            /* Terminal was resized: poke ncurses to learn the new size, then
             * re-center the grid for it. */
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
