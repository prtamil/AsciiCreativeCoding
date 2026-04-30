/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_direct.c — direct placement on hex-with-radii grid
 *
 * DEMO: Flat-top hex grid; each hex is split into 6 equilateral wedges
 *       by 3 diagonals through the centre. Cursor lives at (Q, R, sector)
 *       where sector ∈ 0..5. Arrows move the hex; ',' / '.' rotate the
 *       cursor sub-triangle. SPACE toggles a glyph at the cursor wedge.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/06_hex_subdivision_direct.c \
 *       -o 06_hex_subdivision_direct -lncurses -lm
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

/* §1 */
#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4
#define HEX_SIZE_DEFAULT 16.0
#define HEX_SIZE_MIN      8.0
#define HEX_SIZE_MAX     40.0
#define HEX_SIZE_STEP     2.0
#define BORDER_W 0.10
#define RADIUS_T_FRAC 0.12
#define MAX_OBJ  256
#define N_GLYPHS 6
#define N_THEMES 4
#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_OBJECT 4
#define PAIR_HUD    5
#define PAIR_HINT   6

static const char GLYPHS[N_GLYPHS] = { '*', 'o', '+', '#', 'X', '%' };

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

/* §3 */
static const short THEME_FG[N_THEMES][3] = {
    {  75,  39, 226 }, {  82, 226, 207 }, { 207, 196,  82 }, {  15,  87, 226 },
};
static const short THEME_FG_8[N_THEMES][3] = {
    { COLOR_CYAN,    COLOR_BLUE,  COLOR_YELLOW }, { COLOR_GREEN, COLOR_YELLOW, COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_RED,   COLOR_GREEN  }, { COLOR_WHITE, COLOR_CYAN,   COLOR_YELLOW  },
};
static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_r, fg_o;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0]; fg_r = THEME_FG[theme][1]; fg_o = THEME_FG[theme][2]; }
    else               { fg_e = THEME_FG_8[theme][0]; fg_r = THEME_FG_8[theme][1]; fg_o = THEME_FG_8[theme][2]; }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_OBJECT, fg_o, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* §4 formula — flat-top hex + sector classifier */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if (t < M_PI / 8.0)         return '-';
    if (t < 3.0 * M_PI / 8.0)   return '\\';
    if (t < 5.0 * M_PI / 8.0)   return '|';
    if (t < 7.0 * M_PI / 8.0)   return '/';
    return '-';
}
/* sector_of removed — placement uses cursor-tracked sector, not pixel-derived */
static void pixel_to_hex(double px, double py, double size,
                         int *Q, int *R, double *dist)
{
    double sq3 = sqrt(3.0), sq3_3 = sq3 / 3.0;
    double fq = (2.0 / 3.0 * px) / size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
    double fs = -fq - fr;
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
    double fQ = (double)*Q, fR = (double)*R, fS = (double)(-*Q - *R);
    double d = fabs(fq - fQ);
    double d2 = fabs(fr - fR);
    double d3 = fabs(fs - fS);
    if (d2 > d) d = d2;
    if (d3 > d) d = d3;
    *dist = d;
}
static void hex_centre_pixel(int Q, int R, double size,
                              double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5      * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}
static void wedge_centroid_pixel(int Q, int R, int sector, double size,
                                 double *cx_pix, double *cy_pix)
{
    double cx, cy;
    hex_centre_pixel(Q, R, size, &cx, &cy);
    double ang = (double)sector * M_PI / 3.0;
    double r   = size * sqrt(3.0) / 3.0;
    *cx_pix = cx + r * cos(ang);
    *cy_pix = cy + r * sin(ang);
}
static void wedge_to_screen(int Q, int R, int sector, double size,
                             int ox, int oy, int *scol, int *srow)
{
    double cx, cy;
    wedge_centroid_pixel(Q, R, sector, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W);
    *srow = oy + (int)(cy / CELL_H);
}

/* §5 pool */
typedef struct { int Q, R, sector; char glyph; } HObj;
typedef struct { HObj objs[MAX_OBJ]; int count; } ObjectPool;

static void pool_clear(ObjectPool *p) { p->count = 0; }
static int pool_find(const ObjectPool *p, int Q, int R, int sector)
{
    for (int i = 0; i < p->count; i++)
        if (p->objs[i].Q == Q && p->objs[i].R == R && p->objs[i].sector == sector) return i;
    return -1;
}
static void pool_remove_at(ObjectPool *p, int idx)
{
    if (idx < 0 || idx >= p->count) return;
    p->objs[idx] = p->objs[p->count - 1]; p->count--;
}
static void pool_toggle(ObjectPool *p, int Q, int R, int sector, char glyph)
{
    int idx = pool_find(p, Q, R, sector);
    if (idx >= 0) { pool_remove_at(p, idx); return; }
    if (p->count >= MAX_OBJ) return;
    p->objs[p->count++] = (HObj){ Q, R, sector, glyph };
}
static void pool_draw(const ObjectPool *p, double size,
                      int ox, int oy, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        wedge_to_screen(p->objs[i].Q, p->objs[i].R, p->objs[i].sector,
                        size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, (chtype)(unsigned char)p->objs[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJECT) | A_BOLD);
}

/* §6 cursor */
typedef struct {
    int Q, R, sector; double hex_size;
    int glyph_idx, theme, paused;
} Cursor;
static const int HEX_DIR[4][2] = {
    { 0, -1 }, { 0, +1 }, { -1, 0 }, { +1, 0 },
};
static void cursor_reset(Cursor *cur)
{
    cur->Q = 0; cur->R = 0; cur->sector = 0;
    cur->hex_size = HEX_SIZE_DEFAULT;
    cur->glyph_idx = 0; cur->theme = 0; cur->paused = 0;
}
static void cursor_step_hex(Cursor *cur, int idx)
{
    cur->Q += HEX_DIR[idx][0]; cur->R += HEX_DIR[idx][1];
}
static void cursor_rotate_sector(Cursor *cur, int delta)
{
    cur->sector = (cur->sector + delta + 6) % 6;
}
static void cursor_draw(const Cursor *cur, int ox, int oy, int rows, int cols)
{
    int sc, sr;
    wedge_to_screen(cur->Q, cur->R, cur->sector, cur->hex_size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* §7 scene */
static void grid_draw(int rows, int cols, const Cursor *cur, int ox, int oy)
{
    double sq3 = sqrt(3.0), sq3_2 = sq3 * 0.5;
    double limit_inner = 0.5 - BORDER_W;
    double radius_t    = cur->hex_size * RADIUS_T_FRAC * 0.5;
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;
            int Q, R; double dist;
            pixel_to_hex(px, py, cur->hex_size, &Q, &R, &dist);
            double cx, cy;
            hex_centre_pixel(Q, R, cur->hex_size, &cx, &cy);
            double dxp = px - cx, dyp = py - cy;
            if (dist >= limit_inner) {
                double theta = atan2(dyp, dxp);
                char ch = angle_char(theta + M_PI / 2.0);
                attron(COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                continue;
            }
            double r0 = fabs(dyp);
            double r1 = fabs(0.5 * dyp - sq3_2 * dxp);
            double r2 = fabs(0.5 * dyp + sq3_2 * dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/'; }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) {
                attron(COLOR_PAIR(PAIR_RADIUS));
                mvaddch(row, col, (chtype)(unsigned char)rch);
                attroff(COLOR_PAIR(PAIR_RADIUS));
            }
        }
    }
}

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ObjectPool *pool, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    pool_draw(pool, cur->hex_size, ox, oy, rows, cols);
    cursor_draw(cur, ox, oy, rows, cols);

    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  obj:%d  glyph:%c  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->Q, cur->R, cur->sector,
             pool->count, GLYPHS[cur->glyph_idx], cur->hex_size,
             cur->theme, fps, cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:hex  ,/.: sector  spc:toggle  g:glyph  C:clear  +/-:size  t:theme  r:reset  q:quit  [06 hex direct] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
    wnoutrefresh(stdscr); doupdate();
}

/* §8 screen */
static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* §9 app */
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
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p': cur.paused ^= 1; break;
                case 'r': cursor_reset(&cur); pool_clear(&pool); color_init(cur.theme); break;
                case 'C': pool_clear(&pool); break;
                case 'g': cur.glyph_idx = (cur.glyph_idx + 1) % N_GLYPHS; break;
                case ' ': pool_toggle(&pool, cur.Q, cur.R, cur.sector, GLYPHS[cur.glyph_idx]); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
                case KEY_UP:    cursor_step_hex(&cur, 0); break;
                case KEY_DOWN:  cursor_step_hex(&cur, 1); break;
                case KEY_LEFT:  cursor_step_hex(&cur, 2); break;
                case KEY_RIGHT: cursor_step_hex(&cur, 3); break;
                case ',': case '<': cursor_rotate_sector(&cur, -1); break;
                case '.': case '>': cursor_rotate_sector(&cur, +1); break;
                case '+': case '=':
                    if (cur.hex_size < HEX_SIZE_MAX) { cur.hex_size += HEX_SIZE_STEP; } break;
                case '-':
                    if (cur.hex_size > HEX_SIZE_MIN) { cur.hex_size -= HEX_SIZE_STEP; } break;
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
