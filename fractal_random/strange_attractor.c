/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * strange_attractor.c — Point Density Attractor
 *
 * Iterates a 2-D map millions of times and accumulates hit counts in a
 * density grid matching the terminal size.  After log-normalizing, the
 * density is mapped to a nebula palette: black → deep blue → blue →
 * cyan → white.
 *
 * Six named attractors (Clifford / de Jong family):
 *   1  "Clifford-A"   a=-1.4  b=1.6  c=1.0  d=0.7
 *   2  "de Jong-B"    a=-1.7  b=1.3  c=-0.1 d=-1.2
 *   3  "Marek-C"      a=-2.0  b=-2.0 c=-1.2 d=2.0
 *   4  "Svensson"     a=1.5   b=-1.8 c=1.6  d=0.9
 *   5  "Bedhead"      a=-0.81 b=-0.92 c=0.0 d=0.0  (uses own formula)
 *   6  "Rampe"        a=1.0   b=-1.2 c=-0.5 d=0.5
 *
 * All use: x' = sin(a·y) + c·cos(a·x)
 *           y' = sin(b·x) + d·cos(b·y)
 * except Bedhead which uses its own formula.
 *
 * Keys: q quit  1-6 attractor  r reset density  p pause  +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/strange_attractor.c \
 *       -o strange_attractor -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 attractor  §5 density  §6 draw  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_W_MAX  320
#define GRID_H_MAX  100
#define HUD_ROWS      2

#define ITERS_PER_FRAME  200000    /* orbit samples added each frame  */
#define WARMUP_ITERS       5000    /* discard initial transient        */
#define RENDER_NS   (1000000000LL / 30)

enum { CP_D0=1, CP_D1, CP_D2, CP_D3, CP_D4, CP_HUD };

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color — nebula: black → dark blue → blue → cyan → white            */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_D0,  17, -1);   /* very dark blue  */
        init_pair(CP_D1,  21, -1);   /* blue            */
        init_pair(CP_D2,  27, -1);   /* med blue        */
        init_pair(CP_D3,  51, -1);   /* cyan            */
        init_pair(CP_D4, 231, -1);   /* white           */
        init_pair(CP_HUD, 226, -1);
    } else {
        init_pair(CP_D0, COLOR_BLUE,   -1);
        init_pair(CP_D1, COLOR_BLUE,   -1);
        init_pair(CP_D2, COLOR_CYAN,   -1);
        init_pair(CP_D3, COLOR_CYAN,   -1);
        init_pair(CP_D4, COLOR_WHITE,  -1);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  attractor definitions                                              */
/* ===================================================================== */

typedef struct {
    const char *name;
    float a, b, c, d;
    int   type;   /* 0=Clifford  1=Bedhead  2=Svensson */
} AttrDef;

static const AttrDef ATTRS[] = {
    { "Clifford-A", -1.4f,  1.6f,  1.0f,  0.7f, 0 },
    { "de Jong-B",  -1.7f,  1.3f, -0.1f, -1.2f, 0 },
    { "Marek-C",    -2.0f, -2.0f, -1.2f,  2.0f, 0 },
    { "Svensson",    1.5f, -1.8f,  1.6f,  0.9f, 2 },
    { "Bedhead",    -0.81f,-0.92f,  0.0f,  0.0f, 1 },
    { "Rampe",       1.0f, -1.2f, -0.5f,  0.5f, 0 },
};
#define N_ATTRS  ((int)(sizeof(ATTRS)/sizeof(ATTRS[0])))

static int g_attr_idx = 0;

/* One Clifford iteration */
static void clifford_step(float a, float b, float c, float d,
                           float *x, float *y)
{
    float nx = sinf(a * *y) + c * cosf(a * *x);
    float ny = sinf(b * *x) + d * cosf(b * *y);
    *x = nx; *y = ny;
}

/* Bedhead: x' = sin(x*y/b)*y + cos(a*x-y), y' = x + sin(y)/b */
static void bedhead_step(float a, float b, float *x, float *y)
{
    float b1 = (fabsf(b) < 1e-4f) ? 1e-4f : b;
    float nx = sinf(*x * *y / b1) * *y + cosf(a * *x - *y);
    float ny = *x + sinf(*y) / b1;
    *x = nx; *y = ny;
}

/* Svensson: x' = d*sin(a*x)-sin(b*y), y' = c*cos(a*x)+cos(b*y) */
static void svensson_step(float a, float b, float c, float d,
                           float *x, float *y)
{
    float nx = d * sinf(a * *x) - sinf(b * *y);
    float ny = c * cosf(a * *x) + cosf(b * *y);
    *x = nx; *y = ny;
}

/* ===================================================================== */
/* §5  density grid                                                       */
/* ===================================================================== */

static unsigned int g_density[GRID_H_MAX][GRID_W_MAX];
static int   g_gh, g_gw;
static long long g_total_pts = 0;
static float g_ox, g_oy;   /* current orbit position */

/* Attractor bounding box for mapping (set by exploration) */
static float g_xmin, g_xmax, g_ymin, g_ymax;

static void density_reset(void)
{
    memset(g_density, 0, sizeof g_density);
    g_total_pts = 0;
    g_xmin = -3.f; g_xmax = 3.f;
    g_ymin = -3.f; g_ymax = 3.f;
    g_ox = 0.1f; g_oy = 0.1f;
    /* warmup */
    const AttrDef *at = &ATTRS[g_attr_idx];
    for (int i = 0; i < WARMUP_ITERS; i++) {
        if (at->type == 1) bedhead_step(at->a, at->b, &g_ox, &g_oy);
        else if (at->type == 2) svensson_step(at->a, at->b, at->c, at->d, &g_ox, &g_oy);
        else clifford_step(at->a, at->b, at->c, at->d, &g_ox, &g_oy);
    }
    /* find bbox from 50000 sample points */
    float mx=g_ox, Mx=g_ox, my=g_oy, My=g_oy;
    float sx=g_ox, sy=g_oy;
    for (int i = 0; i < 50000; i++) {
        if (at->type == 1) bedhead_step(at->a, at->b, &sx, &sy);
        else if (at->type == 2) svensson_step(at->a, at->b, at->c, at->d, &sx, &sy);
        else clifford_step(at->a, at->b, at->c, at->d, &sx, &sy);
        if (sx < mx) mx=sx; if (sx > Mx) Mx=sx;
        if (sy < my) my=sy; if (sy > My) My=sy;
    }
    float xm=(Mx-mx)*0.05f, ym=(My-my)*0.05f;
    g_xmin=mx-xm; g_xmax=Mx+xm;
    g_ymin=my-ym; g_ymax=My+ym;
}

static void density_add(int iters)
{
    const AttrDef *at = &ATTRS[g_attr_idx];
    float xs = g_xmax - g_xmin, ys = g_ymax - g_ymin;
    if (xs < 1e-6f) xs = 1e-6f;
    if (ys < 1e-6f) ys = 1e-6f;

    for (int i = 0; i < iters; i++) {
        if (at->type == 1) bedhead_step(at->a, at->b, &g_ox, &g_oy);
        else if (at->type == 2) svensson_step(at->a, at->b, at->c, at->d, &g_ox, &g_oy);
        else clifford_step(at->a, at->b, at->c, at->d, &g_ox, &g_oy);

        int col = (int)((g_ox - g_xmin) / xs * (g_gw - 1));
        int row = (int)((g_oy - g_ymin) / ys * (g_gh - 1));
        if (col >= 0 && col < g_gw && row >= 0 && row < g_gh)
            g_density[row][col]++;
        g_total_pts++;
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

static void scene_draw(void)
{
    /* find max density for log normalization */
    unsigned int dmax = 1;
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            if (g_density[r][c] > dmax) dmax = g_density[r][c];

    float log_max = logf((float)dmax + 1.f);

    for (int r = 0; r < g_gh && r+HUD_ROWS < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            unsigned int d = g_density[r][c];
            if (d == 0) continue;
            float t = logf((float)d + 1.f) / log_max;  /* 0..1 */
            int cp; chtype ch;
            if      (t < 0.20f) { cp = CP_D0; ch = '.'; }
            else if (t < 0.40f) { cp = CP_D1; ch = ':'; }
            else if (t < 0.65f) { cp = CP_D2; ch = '+'; }
            else if (t < 0.85f) { cp = CP_D3; ch = '*'; }
            else                { cp = CP_D4; ch = '#'; }
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(r + HUD_ROWS, c, ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " StrangeAttractor  q:quit  1-6:attractor  r:reset  p:pause  +/-:speed");
    mvprintw(1, 0,
        " [%d] %s  a=%.2f b=%.2f c=%.2f d=%.2f  pts:%lld  %s",
        g_attr_idx+1, ATTRS[g_attr_idx].name,
        ATTRS[g_attr_idx].a, ATTRS[g_attr_idx].b,
        ATTRS[g_attr_idx].c, ATTRS[g_attr_idx].d,
        (long long)g_total_pts,
        g_paused ? "PAUSED" : "accumulating");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static int g_speed = 1;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    density_reset();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
            g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
            density_reset();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': density_reset(); break;
        case '+': case '=': g_speed *= 2; if (g_speed > 8) g_speed = 8; break;
        case '-': g_speed /= 2; if (g_speed < 1) g_speed = 1; break;
        default:
            if (ch >= '1' && ch <= '6') {
                g_attr_idx = ch - '1';
                if (g_attr_idx >= N_ATTRS) g_attr_idx = N_ATTRS-1;
                density_reset();
            }
            break;
        }

        long long now = clock_ns();

        if (!g_paused)
            density_add(ITERS_PER_FRAME * g_speed);

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
