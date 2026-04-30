/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_hex_subdivision_scatter.c — distance-colored scatter on hex-subdivision
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids_placement/06_hex_subdivision_scatter.c \
 *       -o 06_hex_subdivision_scatter -lncurses -lm
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
#define SCATTER_RADIUS 8
#define DENSITY_DEFAULT 80
#define DENSITY_MIN     20
#define DENSITY_MAX    400
#define DENSITY_STEP    20
#define N_BUCKETS 6
#define N_THEMES  3

#define PAIR_BORDER 1
#define PAIR_RADIUS 2
#define PAIR_CURSOR 3
#define PAIR_BUCK0  4
#define PAIR_HUD    10
#define PAIR_HINT   11

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

static const short THEME_GRAD[N_THEMES][N_BUCKETS] = {
    { 196, 202, 214, 226, 82, 39 },
    { 39, 82, 226, 214, 202, 196 },
    { 15, 250, 244, 240, 236, 232 },
};
static const short THEME_GRAD_8[N_THEMES][N_BUCKETS] = {
    { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE },
    { COLOR_BLUE, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED },
    { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
};
static void color_init(int theme)
{
    start_color(); use_default_colors();
    for (int i = 0; i < N_BUCKETS; i++) {
        short fg = (COLORS >= 256) ? THEME_GRAD[theme][i] : THEME_GRAD_8[theme][i];
        init_pair(PAIR_BUCK0 + i, fg, -1);
    }
    init_pair(PAIR_BORDER, COLORS >= 256 ? 248 : COLOR_WHITE, -1);
    init_pair(PAIR_RADIUS, COLORS >= 256 ? 250 : COLOR_BLUE, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
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
    double r = size * sqrt(3.0) / 3.0;
    *cx = hx + r * cos(ang); *cy = hy + r * sin(ang);
}
static void wedge_to_screen(int Q, int R, int sector, double size,
                            int ox, int oy, int *scol, int *srow)
{
    double cx, cy; wedge_centroid_pixel(Q, R, sector, size, &cx, &cy);
    *scol = ox + (int)(cx / CELL_W); *srow = oy + (int)(cy / CELL_H);
}

typedef struct { int Q, R, sector; } HPos;
typedef struct { HPos pos[MAX_OBJ]; int count; } ScatterPool;

static unsigned int g_seed = 1;
static double frand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((double)((g_seed >> 16) & 0x7FFF)) / 32767.0;
}
/* Cube distance between two hexes; +1 if sectors differ. */
static int hex_distance(int aQ, int aR, int bQ, int bR)
{
    int aS = -aQ - aR, bS = -bQ - bR;
    int dq = abs(aQ - bQ), dr = abs(aR - bR), ds = abs(aS - bS);
    int m = (dq > dr) ? dq : dr;
    if (ds > m) m = ds;
    return m;
}
static int wedge_distance(int aQ, int aR, int aS, int bQ, int bR, int bS)
{
    int hd = hex_distance(aQ, aR, bQ, bR);
    int sd = abs(aS - bS); if (sd > 3) sd = 6 - sd;
    return hd + sd;
}
static int distance_bucket(int dist, int max_d)
{
    if (max_d <= 0) return 0;
    int b = (dist * N_BUCKETS) / (max_d + 1);
    if (b >= N_BUCKETS) b = N_BUCKETS - 1;
    return b;
}

typedef struct {
    int Q, R, sector; int density; double hex_size; int theme, paused;
} Cursor;
static const int HEX_DIR[4][2] = { { 0, -1 }, { 0, +1 }, { -1, 0 }, { +1, 0 } };

static void cursor_reset(Cursor *cur)
{
    cur->Q = 0; cur->R = 0; cur->sector = 0;
    cur->density = DENSITY_DEFAULT;
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

static void scatter_seed(ScatterPool *sp, const Cursor *cur)
{
    sp->count = 0;
    g_seed ^= (unsigned int)clock_ns();
    int max = (cur->density < MAX_OBJ) ? cur->density : MAX_OBJ;
    int tries = 0;
    while (sp->count < max && tries < max * 4) {
        int dQ = (int)(frand() * (2*SCATTER_RADIUS+1)) - SCATTER_RADIUS;
        int dR = (int)(frand() * (2*SCATTER_RADIUS+1)) - SCATTER_RADIUS;
        int s  = (int)(frand() * 6);
        sp->pos[sp->count++] = (HPos){ cur->Q + dQ, cur->R + dR, s };
        tries++;
    }
}

static void scatter_draw(const ScatterPool *sp, const Cursor *cur,
                         int ox, int oy, int rows, int cols)
{
    for (int i = 0; i < sp->count; i++) {
        int sc, sr;
        wedge_to_screen(sp->pos[i].Q, sp->pos[i].R, sp->pos[i].sector,
                        cur->hex_size, ox, oy, &sc, &sr);
        if (sc < 0 || sc >= cols || sr < 0 || sr >= rows - 1) continue;
        int dist = wedge_distance(sp->pos[i].Q, sp->pos[i].R, sp->pos[i].sector,
                                  cur->Q, cur->R, cur->sector);
        int b = distance_bucket(dist, SCATTER_RADIUS * 2);
        attron(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
        mvaddch(sr, sc, '*');
        attroff(COLOR_PAIR(PAIR_BUCK0 + b) | A_BOLD);
    }
}

static void grid_draw(int rows, int cols, const Cursor *cur, int ox, int oy)
{
    double sq3 = sqrt(3.0), sq3_2 = sq3 * 0.5;
    double limit_inner = 0.5 - BORDER_W;
    double radius_t    = cur->hex_size * RADIUS_T_FRAC * 0.5;
    attron(COLOR_PAIR(PAIR_BORDER));
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
                mvaddch(row, col, (chtype)(unsigned char)ch);
                continue;
            }
            double r0 = fabs(dyp);
            double r1 = fabs(0.5*dyp - sq3_2*dxp);
            double r2 = fabs(0.5*dyp + sq3_2*dxp);
            char rch = '-'; double rmin = r0;
            if (r1 < rmin) { rmin = r1; rch = '/'; }
            if (r2 < rmin) { rmin = r2; rch = '\\'; }
            if (rmin < radius_t) mvaddch(row, col, (chtype)(unsigned char)rch);
        }
    }
    attroff(COLOR_PAIR(PAIR_BORDER));
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

static void scene_draw(int rows, int cols, const Cursor *cur,
                       const ScatterPool *sp, double fps)
{
    erase();
    int ox = cols / 2, oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    scatter_draw(sp, cur, ox, oy, rows, cols);
    cursor_draw(cur, ox, oy, rows, cols);

    char buf[128];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d sec:%d  N:%d  size:%.0f  theme:%d  %5.1f fps  %s ",
             cur->Q, cur->R, cur->sector,
             sp->count, cur->hex_size, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:hex  ,/.:sector  spc:reseed  +/-:density  t:theme  r:reset  q:quit  [06 scatter] ");
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
    ScatterPool sp; sp.count = 0;
    g_seed = (unsigned int)clock_ns();
    screen_init(cur.theme);
    scatter_seed(&sp, &cur);
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
                case 'r': cursor_reset(&cur); scatter_seed(&sp, &cur); color_init(cur.theme); break;
                case ' ': scatter_seed(&sp, &cur); break;
                case 't': cur.theme = (cur.theme + 1) % N_THEMES; color_init(cur.theme); break;
                case KEY_UP:    cursor_step_hex(&cur, 0); break;
                case KEY_DOWN:  cursor_step_hex(&cur, 1); break;
                case KEY_LEFT:  cursor_step_hex(&cur, 2); break;
                case KEY_RIGHT: cursor_step_hex(&cur, 3); break;
                case ',': case '<': cursor_rotate_sector(&cur, -1); break;
                case '.': case '>': cursor_rotate_sector(&cur, +1); break;
                case '+': case '=':
                    if (cur.density < DENSITY_MAX) { cur.density += DENSITY_STEP; scatter_seed(&sp, &cur); } break;
                case '-':
                    if (cur.density > DENSITY_MIN) { cur.density -= DENSITY_STEP; scatter_seed(&sp, &cur); } break;
            }
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0 = now;
        scene_draw(rows, cols, &cur, &sp, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
