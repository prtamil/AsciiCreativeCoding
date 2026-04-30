/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal_patterns.c — preset stamps on the tetrakis grid
 *
 * Press 1..5 to stamp a preset; SPACE clears.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/03_double_diagonal_patterns.c \
 *       -o 03_double_diagonal_patterns -lncurses -lm
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

/* §1 config */
#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4
#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0
#define BORDER_W 0.10
#define MAX_OBJ  512
#define N_GLYPHS 6
#define N_THEMES 4
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3
#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_OBJECT 3
#define PAIR_HUD    4
#define PAIR_HINT   5

static const char GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };
static const char *DIR_NAME[4]     = { "N", "E", "S", "W" };

/* §2 clock */
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

/* §3 color */
static const short THEME_FG[N_THEMES][2] = { {  82, 226 }, { 207, 226 }, { 207,  82 }, {  15,  39 } };
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_GREEN, COLOR_YELLOW }, { COLOR_MAGENTA, COLOR_YELLOW },
    { COLOR_MAGENTA, COLOR_GREEN }, { COLOR_WHITE, COLOR_CYAN },
};
static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e = (COLORS >= 256) ? THEME_FG[theme][0] : THEME_FG_8[theme][0];
    short fg_o = (COLORS >= 256) ? THEME_FG[theme][1] : THEME_FG_8[theme][1];
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* §4 formula */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *dir, double *fa, double *fb)
{
    double inv = 1.0/size;
    double a = px*inv, b = py*inv;
    int c = (int)floor(a), r = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c; *fb = b - (double)r;
    double dx = *fa - 0.5, dy = *fb - 0.5;
    double adx = fabs(dx), ady = fabs(dy);
    if (adx > ady) *dir = (dx > 0.0) ? DIR_E : DIR_W;
    else           *dir = (dy > 0.0) ? DIR_S : DIR_N;
}
static void tri_centroid_pixel(int col, int row, int dir, double size,
                               double *cx, double *cy)
{
    double a, b;
    switch (dir) {
        case DIR_N: a = 0.5;     b = 1.0/6.0; break;
        case DIR_E: a = 5.0/6.0; b = 0.5;     break;
        case DIR_S: a = 0.5;     b = 5.0/6.0; break;
        default:    a = 1.0/6.0; b = 0.5;     break;
    }
    *cx = ((double)col + a) * size;
    *cy = ((double)row + b) * size;
}
static char tri_edge_char(int dir, double fa, double fb, double *out_min)
{
    double l1, l2, l3; char ch1, ch2, ch3;
    switch (dir) {
        case DIR_N: l1=1.0-fa-fb; ch1='/'; l2=fa-fb; ch2='\\'; l3=2.0*fb; ch3='_'; break;
        case DIR_E: l1=fa-fb; ch1='\\'; l2=fa+fb-1.0; ch2='/'; l3=2.0*(1.0-fa); ch3='|'; break;
        case DIR_S: l1=fb-fa; ch1='\\'; l2=fa+fb-1.0; ch2='/'; l3=2.0*(1.0-fb); ch3='_'; break;
        default:    l1=1.0-fa-fb; ch1='\\'; l2=fb-fa; ch2='/'; l3=2.0*fa; ch3='|'; break;
    }
    char ch = ch1; double m = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m; return ch;
}
static void tri_to_screen(int col, int row, int dir, double size,
                          int ox, int oy, int *scol, int *srow)
{
    double cx, cy;
    tri_centroid_pixel(col, row, dir, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W);
    *srow = oy + (int)(cy / CELL_H);
}

/* §5 pool */
typedef struct { int col, row, dir; char glyph; } TObj;
typedef struct { TObj objs[MAX_OBJ]; int count; } ObjectPool;

static void pool_clear(ObjectPool *p) { p->count = 0; }
static int pool_find(const ObjectPool *p, int col, int row, int dir)
{
    for (int i = 0; i < p->count; i++)
        if (p->objs[i].col == col && p->objs[i].row == row && p->objs[i].dir == dir) return i;
    return -1;
}
static void pool_add(ObjectPool *p, int col, int row, int dir, char glyph)
{
    if (p->count >= MAX_OBJ || pool_find(p, col, row, dir) >= 0) return;
    p->objs[p->count++] = (TObj){ col, row, dir, glyph };
}
static void pool_draw(const ObjectPool *p, double size, int ox, int oy, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        tri_to_screen(p->objs[i].col, p->objs[i].row, p->objs[i].dir, size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->objs[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* §6 cursor */
typedef struct {
    int col, row, dir; double tri_size; int glyph_idx, theme, paused;
} Cursor;

static const int TETRA_DIR[4][4][3] = {
    { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->dir = DIR_N;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->glyph_idx = 0; cur->theme = 0; cur->paused = 0;
}
static void cursor_step(Cursor *cur, int arrow)
{
    const int *t = TETRA_DIR[arrow][cur->dir];
    cur->col += t[0]; cur->row += t[1]; cur->dir = t[2];
}
static void cursor_draw(const Cursor *cur, int ox, int oy, int rows, int cols)
{
    int sc, sr;
    tri_to_screen(cur->col, cur->row, cur->dir, cur->tri_size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* §7 patterns */
#define PAT_END { 0xDEAD, 0, 0 }
#define IS_END(p) ((p)[0] == 0xDEAD)

static const int PAT_RING[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_S }, {  0,  0, DIR_W },
    PAT_END
};
static const int PAT_LINE[][3] = {
    {  0, 0, DIR_N }, {  0, 0, DIR_E }, {  0, 0, DIR_S }, {  0, 0, DIR_W },
    {  1, 0, DIR_N }, {  1, 0, DIR_E }, {  1, 0, DIR_S }, {  1, 0, DIR_W },
    PAT_END
};
static const int PAT_STAR[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_S }, {  0,  0, DIR_W },
    { -1,  0, DIR_E }, { +1,  0, DIR_W },
    {  0, -1, DIR_S }, {  0, +1, DIR_N },
    { -1, -1, DIR_S }, { +1, +1, DIR_N },
    PAT_END
};
static const int PAT_TRI[][3] = {
    {  0,  0, DIR_N }, {  0,  0, DIR_E }, {  0,  0, DIR_W }, PAT_END
};
static void pattern_stamp(ObjectPool *pool, const int (*pat)[3],
                          int cC, int cR, char glyph)
{
    for (int i = 0; !IS_END(pat[i]); i++)
        pool_add(pool, cC + pat[i][0], cR + pat[i][1], pat[i][2], glyph);
}
static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}
static void pattern_scatter(ObjectPool *pool, int cC, int cR, char glyph)
{
    g_seed ^= (unsigned int)clock_ns();
    int n = 10, tries = 0;
    while (n > 0 && tries < 100) {
        int dC = (int)(frand() * 9) - 4;
        int dR = (int)(frand() * 9) - 4;
        int dir = (int)(frand() * 4);
        int prev = pool->count;
        pool_add(pool, cC + dC, cR + dR, dir, glyph);
        if (pool->count > prev) n--;
        tries++;
    }
}

/* §8 scene */
static void grid_draw(int rows, int cols, double size, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int tC, tR, tD; double fa, fb, m;
            pixel_to_tri(px, py, size, &tC, &tR, &tD, &fa, &fb);
            char ch = tri_edge_char(tD, fa, fb, &m);
            if (m >= BORDER_W) continue;
            attron(COLOR_PAIR(PAIR_BORDER));
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(PAIR_BORDER));
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ObjectPool *pool, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur->tri_size, ox, oy);
    pool_draw(pool, cur->tri_size, ox, oy, rows, cols);
    cursor_draw(cur, ox, oy, rows, cols);

    char buf[128];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->dir],
             pool->count, GLYPHS[cur->glyph_idx], cur->tri_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  1:ring 2:line 3:star 4:tri 5:scatter  spc:clear  g:glyph  +/-:size  q:quit  [03 patterns] ");
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
    Cursor cur; cursor_reset(&cur);
    ObjectPool pool; pool_clear(&pool);
    screen_init(cur.theme);
    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();
    while (g_running) {
        if (g_need_resize) { g_need_resize = 0; endwin(); refresh(); rows = LINES; cols = COLS; }
        int ch;
        while ((ch = getch()) != ERR) {
            char glyph = GLYPHS[cur.glyph_idx];
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r': cursor_reset(&cur); pool_clear(&pool); color_init(cur.theme); break;
                case ' ': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case '1': pattern_stamp(&pool, PAT_RING,  cur.col, cur.row, glyph); break;
                case '2': pattern_stamp(&pool, PAT_LINE,  cur.col, cur.row, glyph); break;
                case '3': pattern_stamp(&pool, PAT_STAR,  cur.col, cur.row, glyph); break;
                case '4': pattern_stamp(&pool, PAT_TRI,   cur.col, cur.row, glyph); break;
                case '5': pattern_scatter(&pool, cur.col, cur.row, glyph); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) { cur.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) { cur.tri_size -= TRI_SIZE_STEP; } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0 = now;
        scene_draw(rows, cols, &cur, &pool, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
