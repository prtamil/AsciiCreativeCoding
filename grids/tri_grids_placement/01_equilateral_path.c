/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_equilateral_path.c — line-of-sight path between two triangles
 *
 * DEMO: Two markers — START (green S) and END (red E) — sit on the
 *       equilateral grid. Move '@' with arrows; press 's' to set START
 *       at the cursor, 'e' to set END. The path between START and END
 *       is computed by walking pixel coordinates along the centroid-to-
 *       centroid line and recording which triangle each sampled pixel
 *       lies in. Recomputed every time you move a marker.
 *
 * Study alongside: 01_equilateral_direct.c (manual placement),
 *                  01_equilateral_patterns.c (preset stamps).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, MAX_OBJ
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs: edge / cursor / start / end / path / HUD
 *   §4 formula  — pixel ↔ lattice + centroid + edge char
 *   §5 pool     — ObjectPool for path triangles
 *   §6 cursor   — TRI_DIR + step + draw + START / END markers
 *   §7 path     — pixel walk between two centroids → triangle list
 *   §8 scene    — grid_draw + scene_draw
 *   §9 screen   — ncurses init / cleanup
 *  §10 app      — signals, main loop
 *
 * Keys:  arrows:move  s:set-start  e:set-end  spc:clear-path
 *        +/-:size  t:theme  r:reset  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/01_equilateral_path.c \
 *       -o 01_equilateral_path -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Line-rasterize between two centroids in PIXEL space;
 *                  for each sampled pixel, ask pixel_to_tri "which triangle
 *                  am I in?" and record uniques. The result is the ordered
 *                  set of triangles traversed by the straight line —
 *                  effectively a "line of sight" path.
 *
 * Why pixel walk : True graph BFS on the triangular lattice would also
 *                  work, but the line-walk is simpler and produces a
 *                  visually intuitive "straight" path. Each adjacent pair
 *                  in the resulting path differs by an edge crossing.
 *
 * Sampling step  : The pixel walk samples every ~size/4 pixels — fine
 *                  enough to never skip a triangle along the line.
 *
 * References     :
 *   Bresenham line — https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *   Line-of-sight on grids — Red Blob Games
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
#define N_THEMES   4

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_START  3
#define PAIR_END    4
#define PAIR_PATH   5
#define PAIR_HUD    6
#define PAIR_HINT   7

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
    start_color();
    use_default_colors();
    short fg_e, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_s = THEME_FG[theme][1]; fg_n = THEME_FG[theme][2];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_s = THEME_FG_8[theme][1]; fg_n = THEME_FG_8[theme][2];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
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

typedef struct { int col, row, up; } TPath;
typedef struct { TPath items[MAX_OBJ]; int count; } PathPool;

static void path_clear(PathPool *p) { p->count = 0; }

static int path_contains(const PathPool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].col == col && p->items[i].row == row && p->items[i].up == up)
            return 1;
    }
    return 0;
}

static void path_add(PathPool *p, int col, int row, int up)
{
    if (p->count >= MAX_OBJ) return;
    if (path_contains(p, col, row, up)) return;
    p->items[p->count++] = (TPath){ col, row, up };
}

static void path_draw(const PathPool *p, double size,
                      int ox, int oy, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        tri_to_screen(p->items[i].col, p->items[i].row, p->items[i].up,
                      size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, '*');
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, up;
    int    sCol, sRow, sUp;     /* START marker */
    int    eCol, eRow, eUp;     /* END marker  */
    int    has_start, has_end;
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
    cur->has_start = 0; cur->has_end = 0;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->theme    = 0;
    cur->paused   = 0;
}

static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

static void marker_draw(int col, int row, int up, double size,
                        int ox, int oy, int rows, int cols,
                        char glyph, int pair)
{
    int sc, sr;
    tri_to_screen(col, row, up, size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sr, sc, glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  path — pixel walk between two centroids → triangle list             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * path_compute — walk the line from START centroid to END centroid in
 * pixel space, sampling at step size·0.25, and collect the unique
 * triangles each sample lands in.
 */
static void path_compute(PathPool *p, const Cursor *cur)
{
    path_clear(p);
    if (!cur->has_start || !cur->has_end) return;

    double sx, sy, ex, ey;
    tri_centroid_pixel(cur->sCol, cur->sRow, cur->sUp, cur->tri_size, &sx, &sy);
    tri_centroid_pixel(cur->eCol, cur->eRow, cur->eUp, cur->tri_size, &ex, &ey);

    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) {
        path_add(p, cur->sCol, cur->sRow, cur->sUp);
        return;
    }
    double step = cur->tri_size * 0.25;
    int    n    = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx;
        double py = sy + t * dy;
        int    tC, tR, tU;
        double fa, fb;
        pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
        path_add(p, tC, tR, tU);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, double size, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int    tC, tR, tU;
            double fa, fb, m;
            pixel_to_tri(px, py, size, &tC, &tR, &tU, &fa, &fb);
            char ch = tri_edge_char(tU, fa, fb, &m);
            if (m >= BORDER_W) continue;
            attron(COLOR_PAIR(PAIR_BORDER));
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(PAIR_BORDER));
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const PathPool *p, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur->tri_size, ox, oy);
    path_draw(p, cur->tri_size, ox, oy, rows, cols);

    if (cur->has_start)
        marker_draw(cur->sCol, cur->sRow, cur->sUp, cur->tri_size,
                    ox, oy, rows, cols, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(cur->eCol, cur->eRow, cur->eUp, cur->tri_size,
                    ox, oy, rows, cols, 'E', PAIR_END);

    /* Cursor (drawn last so always visible) */
    {
        int sc, sr;
        tri_to_screen(cur->col, cur->row, cur->up, cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
            mvaddch(sr, sc, '@');
            attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        }
    }

    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, cur->up ? "/\\" : "\\/",
             p->count, cur->tri_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [01 path] ");
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

    Cursor    cur;  cursor_reset(&cur);
    PathPool  path; path_clear(&path);
    screen_init(cur.theme);

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
                case 'r':          cursor_reset(&cur); path_clear(&path);
                                   color_init(cur.theme); break;
                case ' ':          cur.has_start = 0; cur.has_end = 0;
                                   path_clear(&path); break;
                case 's':
                    cur.sCol = cur.col; cur.sRow = cur.row; cur.sUp = cur.up;
                    cur.has_start = 1; path_compute(&path, &cur);
                    break;
                case 'e':
                    cur.eCol = cur.col; cur.eRow = cur.row; cur.eUp = cur.up;
                    cur.has_end = 1; path_compute(&path, &cur);
                    break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) {
                        cur.tri_size += TRI_SIZE_STEP; path_compute(&path, &cur);
                    } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) {
                        cur.tri_size -= TRI_SIZE_STEP; path_compute(&path, &cur);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, &path, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
