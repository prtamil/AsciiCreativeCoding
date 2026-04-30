/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_isometric_path.c — line-of-sight path on the iso (solid-fill) grid
 *
 * DEMO: Iso solid-color triangular grid (6-cycle palette). Move '@' with
 *       arrows; `s` sets START, `e` sets END. The path between markers
 *       is computed by pixel-walking the centroid-to-centroid line and
 *       recording each triangle the line passes through. Path cells are
 *       overlaid with bright `*` glyphs.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/05_isometric_path.c \
 *       -o 05_isometric_path -lncurses -lm
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

/* §1 */
#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4
#define TRI_SIZE_DEFAULT 14.0
#define TRI_SIZE_MIN     6.0
#define TRI_SIZE_MAX    40.0
#define TRI_SIZE_STEP    2.0
#define MAX_OBJ  1024
#define N_PALETTE 6
#define N_THEMES  3

#define PAIR_FILL_BASE 1                        /* 1..6 */
#define PAIR_PATH     (PAIR_FILL_BASE + N_PALETTE)
#define PAIR_START    (PAIR_PATH + 1)
#define PAIR_END      (PAIR_START + 1)
#define PAIR_CURSOR   (PAIR_END + 1)
#define PAIR_HUD      (PAIR_CURSOR + 1)
#define PAIR_HINT     (PAIR_HUD + 1)

/* §2 */
static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec = (time_t)(ns/1000000000LL), .tv_nsec = (long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* §3 — same iso 6-cycle palette as 05_isometric_direct */
static const short PAL256[N_THEMES][N_PALETTE] = {
    { 196, 214, 226, 118, 39, 129 },
    {  21,  39,  82, 226, 207, 21  },
    {  15,  87,  39,   0, 39, 87   },
};
static const short PAL8[N_THEMES][N_PALETTE] = {
    { COLOR_RED,  COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,    COLOR_BLUE,    COLOR_MAGENTA },
    { COLOR_BLUE, COLOR_CYAN,   COLOR_GREEN, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE,COLOR_CYAN,   COLOR_BLUE,  COLOR_BLACK,   COLOR_BLUE,    COLOR_CYAN    },
};
static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_PALETTE; i++) {
        short bg = (COLORS >= 256) ? PAL256[theme][i] : PAL8[theme][i];
        init_pair(PAIR_FILL_BASE + i, COLOR_BLACK, bg);
    }
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_START,  COLORS >= 256 ?  82 : COLOR_GREEN,  COLOR_BLACK);
    init_pair(PAIR_END,    COLORS >= 256 ? 196 : COLOR_RED,    COLOR_BLACK);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLACK);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0  : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75  : COLOR_CYAN,  -1);
}

/* §4 formula (equilateral skew lattice — same as 01) */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *up,
                         double *fa, double *fb)
{
    double h = size * sqrt(3.0) * 0.5;
    double b = py / h;
    double a = px / size - 0.5 * b;
    int c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r; *fa = a-(double)c; *fb = b-(double)r;
    *up = (*fa + *fb >= 1.0) ? 1 : 0;
}
static void tri_centroid_pixel(int col, int row, int up, double size,
                               double *cx, double *cy)
{
    double h = size * sqrt(3.0) * 0.5;
    double a = (up == 0) ? ((double)col + 1.0/3.0) : ((double)col + 2.0/3.0);
    double b = (up == 0) ? ((double)row + 1.0/3.0) : ((double)row + 2.0/3.0);
    *cx = (a + 0.5 * b) * size; *cy = b * h;
}
static void tri_to_screen(int col, int row, int up, double size,
                          int ox, int oy, int *scol, int *srow)
{
    double cx, cy; tri_centroid_pixel(col, row, up, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W); *srow = oy + (int)(cy / CELL_H);
}
static int palette_index(int col, int row, int up)
{
    int k = col + 2 * row + up;
    k %= N_PALETTE; if (k < 0) k += N_PALETTE;
    return k;
}

/* §5 pool */
typedef struct { int col, row, up; } TPath;
typedef struct { TPath items[MAX_OBJ]; int count; } PathPool;

static void path_clear(PathPool *p) { p->count = 0; }
static int path_contains(const PathPool *p, int col, int row, int up)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].col == col && p->items[i].row == row && p->items[i].up == up) return 1;
    return 0;
}
static void path_add(PathPool *p, int col, int row, int up)
{
    if (p->count >= MAX_OBJ || path_contains(p, col, row, up)) return;
    p->items[p->count++] = (TPath){ col, row, up };
}

/* §6 cursor */
typedef struct {
    int col, row, up;
    int sCol, sRow, sUp, eCol, eRow, eUp;
    int has_start, has_end;
    double tri_size;
    int theme, paused;
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
    cur->theme = 0; cur->paused = 0;
}
static void cursor_step(Cursor *cur, int dir)
{
    const int *t = TRI_DIR[dir][cur->up];
    cur->col += t[0]; cur->row += t[1]; cur->up = t[2];
}

/* §7 path */
static void path_compute(PathPool *p, const Cursor *cur)
{
    path_clear(p);
    if (!cur->has_start || !cur->has_end) return;
    double sx, sy, ex, ey;
    tri_centroid_pixel(cur->sCol, cur->sRow, cur->sUp, cur->tri_size, &sx, &sy);
    tri_centroid_pixel(cur->eCol, cur->eRow, cur->eUp, cur->tri_size, &ex, &ey);
    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) { path_add(p, cur->sCol, cur->sRow, cur->sUp); return; }
    double step = cur->tri_size * 0.25;
    int n = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx, py = sy + t * dy;
        int tC, tR, tU; double fa, fb;
        pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
        path_add(p, tC, tR, tU);
    }
}

/* §8 scene — solid fill background, then path + markers + cursor on top */
static void scene_draw(int rows, int cols, const Cursor *cur,
                       const PathPool *p, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    /* Solid iso fill */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int tC, tR, tU; double fa, fb;
            pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tU, &fa, &fb);
            int pair = PAIR_FILL_BASE + palette_index(tC, tR, tU);
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, ' ');
            attroff(COLOR_PAIR(pair));
        }
    }
    /* Path overlay */
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        tri_to_screen(p->items[i].col, p->items[i].row, p->items[i].up,
                      cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, '*');
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);

    /* Markers + cursor */
    if (cur->has_start) {
        int sc, sr;
        tri_to_screen(cur->sCol, cur->sRow, cur->sUp, cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            attron(COLOR_PAIR(PAIR_START) | A_BOLD);
            mvaddch(sr, sc, 'S');
            attroff(COLOR_PAIR(PAIR_START) | A_BOLD);
        }
    }
    if (cur->has_end) {
        int sc, sr;
        tri_to_screen(cur->eCol, cur->eRow, cur->eUp, cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            attron(COLOR_PAIR(PAIR_END) | A_BOLD);
            mvaddch(sr, sc, 'E');
            attroff(COLOR_PAIR(PAIR_END) | A_BOLD);
        }
    }
    {
        int sc, sr;
        tri_to_screen(cur->col, cur->row, cur->up, cur->tri_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
            mvaddch(sr, sc, '@');
            attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
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
             " arrows:move  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  r:reset  q:quit  [05 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
    wnoutrefresh(stdscr); doupdate();
}

/* §9 screen */
static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* §10 app */
static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGWINCH, on_signal);
    Cursor   cur;  cursor_reset(&cur);
    PathPool path; path_clear(&path);
    screen_init(cur.theme);
    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();
    while (g_running) {
        if (g_need_resize) { g_need_resize = 0; endwin(); refresh(); rows = LINES; cols = COLS; }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r': cursor_reset(&cur); path_clear(&path); color_init(cur.theme); break;
                case ' ': cur.has_start = 0; cur.has_end = 0; path_clear(&path); break;
                case 's': cur.sCol = cur.col; cur.sRow = cur.row; cur.sUp = cur.up;
                          cur.has_start = 1; path_compute(&path, &cur); break;
                case 'e': cur.eCol = cur.col; cur.eRow = cur.row; cur.eUp = cur.up;
                          cur.has_end = 1; path_compute(&path, &cur); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
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
        t0 = now;
        scene_draw(rows, cols, &cur, &path, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
