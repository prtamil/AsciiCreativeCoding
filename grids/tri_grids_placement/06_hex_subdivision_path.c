/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_path.c — line-of-sight path on hex-subdivision grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/06_hex_subdivision_path.c \
 *       -o 06_hex_subdivision_path -lncurses -lm
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

#define TARGET_FPS 60
#define CELL_W 2
#define CELL_H 4
#define HEX_SIZE_DEFAULT 16.0
#define HEX_SIZE_MIN      8.0
#define HEX_SIZE_MAX     40.0
#define HEX_SIZE_STEP     2.0
#define BORDER_W 0.10
#define RADIUS_T_FRAC 0.12
#define MAX_OBJ  1024
#define N_THEMES 4
#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_START  4
#define PAIR_END    5
#define PAIR_PATH   6
#define PAIR_HUD    7
#define PAIR_HINT   8

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

static const short THEME_FG[N_THEMES][4] = {
    {  75, 39,  82, 196 }, {  82, 226, 226, 207 }, { 207, 196,  82,  39 }, {  15, 87, 82, 196 },
};
static const short THEME_FG_8[N_THEMES][4] = {
    { COLOR_CYAN, COLOR_BLUE, COLOR_GREEN, COLOR_RED },
    { COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_MAGENTA },
    { COLOR_MAGENTA, COLOR_RED, COLOR_GREEN, COLOR_BLUE },
    { COLOR_WHITE, COLOR_CYAN, COLOR_GREEN, COLOR_RED },
};
static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg_e, fg_r, fg_s, fg_n;
    if (COLORS >= 256) {
        fg_e = THEME_FG[theme][0]; fg_r = THEME_FG[theme][1];
        fg_s = THEME_FG[theme][2]; fg_n = THEME_FG[theme][3];
    } else {
        fg_e = THEME_FG_8[theme][0]; fg_r = THEME_FG_8[theme][1];
        fg_s = THEME_FG_8[theme][2]; fg_n = THEME_FG_8[theme][3];
    }
    init_pair(PAIR_BORDER, fg_e, -1);
    init_pair(PAIR_RADIUS, fg_r, -1);
    init_pair(PAIR_START,  fg_s, -1);
    init_pair(PAIR_END,    fg_n, -1);
    init_pair(PAIR_PATH,   COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15  : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0  : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75  : COLOR_CYAN,  -1);
}

static char angle_char(double theta)
{
    double t = fmod(theta, M_PI); if (t < 0.0) t += M_PI;
    if (t < M_PI / 8.0)         return '-';
    if (t < 3.0 * M_PI / 8.0)   return '\\';
    if (t < 5.0 * M_PI / 8.0)   return '|';
    if (t < 7.0 * M_PI / 8.0)   return '/';
    return '-';
}
static int sector_of(double dx, double dy)
{
    double ang = atan2(dy, dx);
    int s = (int)floor((ang + M_PI / 6.0) / (M_PI / 3.0));
    s %= 6; if (s < 0) s += 6;
    return s;
}
static void pixel_to_hex(double px, double py, double size,
                         int *Q, int *R, double *dist)
{
    double sq3 = sqrt(3.0), sq3_3 = sq3/3.0;
    double fq = (2.0/3.0 * px) / size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
    double fs = -fq - fr;
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq-fq), dr = fabs((double)rr-fr), ds = fabs((double)rs-fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
    double fQ = (double)*Q, fR = (double)*R, fS = (double)(-*Q-*R);
    double d = fabs(fq-fQ); double d2 = fabs(fr-fR); double d3 = fabs(fs-fS);
    if (d2 > d) { d = d2; }
    if (d3 > d) { d = d3; }
    *dist = d;
}
static void hex_centre_pixel(int Q, int R, double size, double *cx, double *cy)
{
    double sq3 = sqrt(3.0);
    *cx = size * 1.5 * (double)Q;
    *cy = size * (sq3*0.5 * (double)Q + sq3 * (double)R);
}
static void wedge_centroid_pixel(int Q, int R, int sector, double size, double *cx, double *cy)
{
    double hx, hy; hex_centre_pixel(Q, R, size, &hx, &hy);
    double ang = (double)sector * M_PI / 3.0;
    double r   = size * sqrt(3.0) / 3.0;
    *cx = hx + r * cos(ang); *cy = hy + r * sin(ang);
}
static void wedge_to_screen(int Q, int R, int sector, double size,
                            int ox, int oy, int *scol, int *srow)
{
    double cx, cy; wedge_centroid_pixel(Q, R, sector, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W); *srow = oy + (int)(cy / CELL_H);
}

typedef struct { int Q, R, sector; } HPath;
typedef struct { HPath items[MAX_OBJ]; int count; } PathPool;
static void path_clear(PathPool *p) { p->count = 0; }
static int path_contains(const PathPool *p, int Q, int R, int s)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].Q == Q && p->items[i].R == R && p->items[i].sector == s) return 1;
    return 0;
}
static void path_add(PathPool *p, int Q, int R, int s)
{
    if (p->count >= MAX_OBJ || path_contains(p, Q, R, s)) return;
    p->items[p->count++] = (HPath){ Q, R, s };
}
static void path_draw(const PathPool *p, double size, int ox, int oy, int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_PATH) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int sc, sr;
        wedge_to_screen(p->items[i].Q, p->items[i].R, p->items[i].sector, size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1)
            mvaddch(sr, sc, '*');
    }
    attroff(COLOR_PAIR(PAIR_PATH) | A_BOLD);
}

typedef struct {
    int Q, R, sector;
    int sQ, sR, sSec, eQ, eR, eSec;
    int has_start, has_end;
    double hex_size; int theme, paused;
} Cursor;
static const int HEX_DIR[4][2] = { { 0, -1 }, { 0, +1 }, { -1, 0 }, { +1, 0 } };

static void cursor_reset(Cursor *cur)
{
    cur->Q = 0; cur->R = 0; cur->sector = 0;
    cur->has_start = 0; cur->has_end = 0;
    cur->hex_size = HEX_SIZE_DEFAULT;
    cur->theme = 0; cur->paused = 0;
}
static void cursor_step_hex(Cursor *cur, int idx)
{
    cur->Q += HEX_DIR[idx][0]; cur->R += HEX_DIR[idx][1];
}
static void cursor_rotate_sector(Cursor *cur, int delta)
{
    cur->sector = (cur->sector + delta + 6) % 6;
}
static void marker_draw(int Q, int R, int s, double size,
                        int ox, int oy, int rows, int cols, char glyph, int pair)
{
    int sc, sr; wedge_to_screen(Q, R, s, size, ox, oy, &sc, &sr);
    if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
        attron(COLOR_PAIR(pair) | A_BOLD); mvaddch(sr, sc, glyph); attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * path_compute — pixel-walk between two wedge centroids; for each
 * sample, identify the (Q, R, sector) triple it falls in.
 */
static void path_compute(PathPool *p, const Cursor *cur)
{
    path_clear(p);
    if (!cur->has_start || !cur->has_end) return;
    double sx, sy, ex, ey;
    wedge_centroid_pixel(cur->sQ, cur->sR, cur->sSec, cur->hex_size, &sx, &sy);
    wedge_centroid_pixel(cur->eQ, cur->eR, cur->eSec, cur->hex_size, &ex, &ey);
    double dx = ex - sx, dy = ey - sy;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist < 1e-6) { path_add(p, cur->sQ, cur->sR, cur->sSec); return; }
    double step = cur->hex_size * 0.25;
    int n = (int)(dist / step) + 1;
    for (int i = 0; i <= n; i++) {
        double t = (double)i / (double)n;
        double px = sx + t * dx, py = sy + t * dy;
        int Q, R; double dd;
        pixel_to_hex(px, py, cur->hex_size, &Q, &R, &dd);
        double cx, cy; hex_centre_pixel(Q, R, cur->hex_size, &cx, &cy);
        int s = sector_of(px - cx, py - cy);
        path_add(p, Q, R, s);
    }
}

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
            double cx, cy; hex_centre_pixel(Q, R, cur->hex_size, &cx, &cy);
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
            double r1 = fabs(0.5*dyp - sq3_2*dxp);
            double r2 = fabs(0.5*dyp + sq3_2*dxp);
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

static void scene_draw(int rows, int cols, const Cursor *cur, const PathPool *p, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    path_draw(p, cur->hex_size, ox, oy, rows, cols);
    if (cur->has_start)
        marker_draw(cur->sQ, cur->sR, cur->sSec, cur->hex_size, ox, oy, rows, cols, 'S', PAIR_START);
    if (cur->has_end)
        marker_draw(cur->eQ, cur->eR, cur->eSec, cur->hex_size, ox, oy, rows, cols, 'E', PAIR_END);
    {
        int sc, sr;
        wedge_to_screen(cur->Q, cur->R, cur->sector, cur->hex_size, ox, oy, &sc, &sr);
        if (sc >= 0 && sc < cols && sr >= 0 && sr < rows - 1) {
            attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
            mvaddch(sr, sc, '@');
            attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        }
    }

    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  path:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->Q, cur->R, cur->sector,
             p->count, cur->hex_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:hex  ,/.: sector  s:set-start  e:set-end  spc:clear  +/-:size  t:theme  q:quit  [06 path] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
    wnoutrefresh(stdscr); doupdate();
}

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

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
                case 's': cur.sQ = cur.Q; cur.sR = cur.R; cur.sSec = cur.sector;
                          cur.has_start = 1; path_compute(&path, &cur); break;
                case 'e': cur.eQ = cur.Q; cur.eR = cur.R; cur.eSec = cur.sector;
                          cur.has_end = 1; path_compute(&path, &cur); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
                case KEY_UP:    cursor_step_hex(&cur, 0); break;
                case KEY_DOWN:  cursor_step_hex(&cur, 1); break;
                case KEY_LEFT:  cursor_step_hex(&cur, 2); break;
                case KEY_RIGHT: cursor_step_hex(&cur, 3); break;
                case ',': case '<': cursor_rotate_sector(&cur, -1); break;
                case '.': case '>': cursor_rotate_sector(&cur, +1); break;
                case '+': case '=':
                    if (cur.hex_size < HEX_SIZE_MAX) {
                        cur.hex_size += HEX_SIZE_STEP; path_compute(&path, &cur);
                    } break;
                case '-':
                    if (cur.hex_size > HEX_SIZE_MIN) {
                        cur.hex_size -= HEX_SIZE_STEP; path_compute(&path, &cur);
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
