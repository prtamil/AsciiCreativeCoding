/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_scatter.c — random scatter colored by distance to cursor
 *
 * DEMO: A random scatter of N triangles fills a square region around the
 *       cursor. Each triangle is colored on a 6-stop gradient by its
 *       cell-distance from the cursor — closer = warm, farther = cool.
 *       Press SPACE to reseed; +/- to change density (N).
 *
 * Study alongside: 01_equilateral_direct.c (manual placement),
 *                  01_equilateral_patterns.c (preset stamps),
 *                  01_equilateral_path.c (line-of-sight path).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, MAX_OBJ, RADIUS, DENSITY
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — gradient palette (6 distance buckets)
 *   §4 formula  — pixel ↔ lattice + centroid + edge char
 *   §5 pool     — ScatterPool: store (col, row, up, dist_bucket)
 *   §6 cursor   — TRI_DIR + step + draw
 *   §7 scatter  — random spawn + distance bucketing
 *   §8 scene    — grid_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:reseed  +/-:density   r:reset
 *        t:theme  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/01_equilateral_scatter.c \
 *       -o 01_equilateral_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Random scatter. Pick N random (Δcol, Δrow, up) within
 *                  ±RADIUS of the cursor; color by Manhattan-style cell
 *                  distance from cursor, bucketed into 6 gradient slots.
 *
 * Distance metric: |Δcol| + |Δrow| + (Δup ? 1 : 0). Cheap and roughly
 *                  matches actual edge-walk distance on the equilateral
 *                  lattice for short distances.
 *
 * Re-seeding     : SPACE re-randomises with a new seed (xor'd by clock).
 *                  Moving the cursor does NOT re-seed — but recolours
 *                  the existing scatter as the cursor moves.
 *
 * References     :
 *   Linear congruential generator — Numerical Recipes ch. 7
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN      6.0
#define TRI_SIZE_MAX     40.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W   0.10
#define MAX_OBJ    1024
#define SCATTER_RADIUS    12

#define DENSITY_DEFAULT  120
#define DENSITY_MIN       20
#define DENSITY_MAX      500
#define DENSITY_STEP      20

#define N_BUCKETS  6
#define N_THEMES   3

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_BUCK0  3   /* nearest, hottest */
#define PAIR_BUCK1  4
#define PAIR_BUCK2  5
#define PAIR_BUCK3  6
#define PAIR_BUCK4  7
#define PAIR_BUCK5  8   /* farthest, coolest */
#define PAIR_HUD    9
#define PAIR_HINT   10

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_GRAD[N_THEMES][N_BUCKETS] = {
    /* 256-color: hot → cold */
    { 196, 202, 214, 226,  82,  39 },   /* warm */
    {  39,  82, 226, 214, 202, 196 },   /* cool */
    {  15, 250, 244, 240, 236, 232 },   /* mono dim */
};
static const short THEME_GRAD_8[N_THEMES][N_BUCKETS] = {
    { COLOR_RED,  COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE },
    { COLOR_BLUE, COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,   COLOR_RED  },
    { COLOR_WHITE,COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE},
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? THEME_GRAD[theme][i] : THEME_GRAD_8[theme][i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_BORDER, COLORS >= 256 ? 248 : COLOR_WHITE, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15  : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0  : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75  : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

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
    char ch1, ch2, ch3;
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

static void tri_to_screen(int col, int row, int up, double size,
                          int ox, int oy, int *scol, int *srow)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(col, row, up, size, &cx_pix, &cy_pix);
    *scol = ox + (int)(cx_pix / CELL_W);
    *srow = oy + (int)(cy_pix / CELL_H);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int col, row, up; } TPos;
typedef struct { TPos pos[MAX_OBJ]; int count; } ScatterPool;

static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}

static int triangle_distance(int aC, int aR, int aU, int bC, int bR, int bU)
{
    int d = abs(aC - bC) + abs(aR - bR);
    if (aU != bU) d += 1;
    return d;
}

static int distance_bucket(int dist, int max_d)
{
    if (max_d <= 0) return 0;
    int b = (dist * N_BUCKETS) / (max_d + 1);
    if (b >= N_BUCKETS) b = N_BUCKETS - 1;
    return b;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;
    int    density;
    double tri_size;
    int    theme;
    int    paused;
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
    cur->density   = DENSITY_DEFAULT;
    cur->tri_size  = TRI_SIZE_DEFAULT;
    cur->theme     = 0;
    cur->paused    = 0;
}

static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scatter                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scatter_seed(ScatterPool *sp, const Cursor *cur)
{
    sp->count = 0;
    g_seed ^= (unsigned int)clock_ns();
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    int tries = 0;
    while (sp->count < max && tries < max * 4) {
        int dC = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int dR = (int)(frand() * (2 * SCATTER_RADIUS + 1)) - SCATTER_RADIUS;
        int up = frand() > 0.5 ? 1 : 0;
        sp->pos[sp->count++] = (TPos){ cur->col + dC, cur->row + dR, up };
        tries++;
    }
}

static void scatter_draw(const ScatterPool *sp, const Cursor *cur,
                         int ox, int oy, int rows, int cols)
{
    for (int i = 0; i < sp->count; i++) {
        int sc, sr;
        tri_to_screen(sp->pos[i].col, sp->pos[i].row, sp->pos[i].up,
                      cur->tri_size, ox, oy, &sc, &sr);
        if (sc < 0 || sc >= cols || sr < 0 || sr >= rows - 1) continue;

        int dist = triangle_distance(sp->pos[i].col, sp->pos[i].row, sp->pos[i].up,
                                     cur->col, cur->row, cur->up);
        int b = distance_bucket(dist, SCATTER_RADIUS * 2);
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, '*');
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, double size, int ox, int oy)
{
    attron(COLOR_PAIR(PAIR_BORDER));
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int    tC, tR, tU;
            double fa, fb, m;
            pixel_to_tri(px, py, size, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= BORDER_W) continue;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
}

static void cursor_draw(const Cursor *cur, int ox, int oy, int rows, int cols)
{
    int sc, sr;
    tri_to_screen(cur->col, cur->row, cur->up, cur->tri_size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ScatterPool *sp, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur->tri_size, ox, oy);
    scatter_draw(sp, cur, ox, oy, rows, cols);
    cursor_draw(cur, ox, oy, rows, cols);

    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             sp->count, cur->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [01 scatter] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    Cursor      cur; cursor_reset(&cur);
    ScatterPool sp;  sp.count = 0;
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    scatter_seed(&sp, &cur);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          cur.paused ^= 1; break;
                case 'r':          cursor_reset(&cur); scatter_seed(&sp, &cur);
                                   color_init(cur.theme); break;
                case ' ':          scatter_seed(&sp, &cur); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.density < DENSITY_MAX) {
                        cur.density += DENSITY_STEP; scatter_seed(&sp, &cur);
                    } break;
                case '-':
                    if (cur.density > DENSITY_MIN) {
                        cur.density -= DENSITY_STEP; scatter_seed(&sp, &cur);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, &sp, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
