/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wave_2d.c — 2-D Wave Equation / Huygens Interference
 *
 * Solves the scalar wave PDE on the terminal grid:
 *
 *   ∂²u/∂t² = c² · ∇²u
 *
 * Discretised with explicit Euler (FDTD):
 *
 *   u[t+1] = 2·u[t] − u[t−1] + C² · ∇²u[t]
 *
 * where C = c·dt/h must satisfy C·√2 < 1 for stability (2-D CFL condition).
 * Here c=0.40, dt=1, h=1 → C²=0.16 → C·√2=0.566 < 1 ✓
 *
 * Point sources (SPACE) emit circular wavefronts; multiple sources interfere
 * to produce standing-wave patterns.  Keys 1–5 toggle persistent oscillating
 * sources at fixed positions.
 *
 * Color maps u-value: negative=blue  zero=black  positive=white.
 *
 * Keys: q quit  SPACE impulse at centre  1-5 toggle oscillators
 *       p pause  r reset  +/- sim speed  c clear sources
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/wave_2d.c \
 *       -o wave_2d -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 grid  §5 sources  §6 draw  §7 app
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

#define C2         0.16f    /* (c·dt/h)²: CFL = 0.40, stable in 2D ✓ */
#define DAMP       0.999f   /* per-step global damping                 */
#define BORDER_W   4        /* sponge-layer width (cells)              */
#define BORDER_DAMP 0.80f   /* damping factor at sponge cells          */

#define IMPULSE_AMP  4.0f
#define OSC_AMP      0.50f
#define OSC_FREQ     0.20f  /* oscillator frequency (radians/step)     */

#define STEPS_PER_FRAME  4
#define RENDER_NS        (1000000000LL / 30)

enum {
    CP_BL2 = 1,  /* deep negative — deep blue      */
    CP_BL1,       /* negative      — blue           */
    CP_BL0,       /* slightly neg  — dim cyan       */
    CP_WH0,       /* slightly pos  — dim white      */
    CP_WH1,       /* positive      — white          */
    CP_WH2,       /* strongly pos  — bright white   */
    CP_SRC,       /* source marker                  */
    CP_HUD,
};

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
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_BL2,  17, -1);   /* dark navy    */
        init_pair(CP_BL1,  21, -1);   /* bright blue  */
        init_pair(CP_BL0,  27, -1);   /* blue         */
        init_pair(CP_WH0, 250, -1);   /* light grey   */
        init_pair(CP_WH1, 255, -1);   /* near white   */
        init_pair(CP_WH2, 231, -1);   /* pure white   */
        init_pair(CP_SRC, 208, -1);   /* orange       */
        init_pair(CP_HUD, 244, -1);
    } else {
        init_pair(CP_BL2, COLOR_BLUE,   -1);
        init_pair(CP_BL1, COLOR_BLUE,   -1);
        init_pair(CP_BL0, COLOR_CYAN,   -1);
        init_pair(CP_WH0, COLOR_WHITE,  -1);
        init_pair(CP_WH1, COLOR_WHITE,  -1);
        init_pair(CP_WH2, COLOR_WHITE,  -1);
        init_pair(CP_SRC, COLOR_YELLOW, -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static float g_u[GRID_H_MAX][GRID_W_MAX];   /* current field         */
static float g_up[GRID_H_MAX][GRID_W_MAX];  /* previous field        */
static float g_u2[GRID_H_MAX][GRID_W_MAX];  /* scratch               */
static int   g_gh, g_gw;

static void grid_clear(void)
{
    memset(g_u,  0, sizeof g_u);
    memset(g_up, 0, sizeof g_up);
}

static void grid_step(void)
{
    for (int r = 0; r < g_gh; r++) {
        for (int c = 0; c < g_gw; c++) {
            int ru = r > 0      ? r-1 : 0;
            int rd = r < g_gh-1 ? r+1 : g_gh-1;
            int cl = c > 0      ? c-1 : 0;
            int cr = c < g_gw-1 ? c+1 : g_gw-1;

            float lap = g_u[ru][c] + g_u[rd][c] + g_u[r][cl] + g_u[r][cr]
                        - 4.f * g_u[r][c];

            g_u2[r][c] = 2.f*g_u[r][c] - g_up[r][c] + C2*lap;
            g_u2[r][c] *= DAMP;

            /* sponge layer: extra damping at all four borders */
            int dist = r;
            if (g_gh-1-r < dist) dist = g_gh-1-r;
            if (c < dist)        dist = c;
            if (g_gw-1-c < dist) dist = g_gw-1-c;
            if (dist < BORDER_W) {
                float f = (float)dist / (float)BORDER_W; /* 0..1 */
                g_u2[r][c] *= BORDER_DAMP + (1.f - BORDER_DAMP)*f;
            }
        }
    }
    /* swap u↔up, then u2→u */
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++) {
            g_up[r][c] = g_u[r][c];
            g_u[r][c]  = g_u2[r][c];
        }
}

static void inject_impulse(int cr, int cc, float amp)
{
    for (int r = cr-4; r <= cr+4; r++)
        for (int c = cc-4; c <= cc+4; c++) {
            if (r < 0||r>=g_gh||c < 0||c>=g_gw) continue;
            float dx = (float)(c-cc), dy = (float)(r-cr);
            g_u[r][c] += amp * expf(-(dx*dx+dy*dy)/8.f);
        }
}

/* ===================================================================== */
/* §5  sources — persistent oscillating sources                           */
/* ===================================================================== */

typedef struct { float fr, fc; float phase; bool active; } Source;

#define N_SOURCES 5
static Source g_src[N_SOURCES];
static long long g_step = 0;

static void sources_init(int gh, int gw)
{
    /* five positions: centre plus four quadrant centres */
    float fr[5] = { 0.5f,  0.25f, 0.25f, 0.75f, 0.75f };
    float fc[5] = { 0.5f,  0.25f, 0.75f, 0.25f, 0.75f };
    for (int i = 0; i < N_SOURCES; i++) {
        g_src[i].fr     = fr[i] * gh;
        g_src[i].fc     = fc[i] * gw;
        g_src[i].phase  = 0.f;
        g_src[i].active = false;
    }
}

static void sources_step(void)
{
    for (int i = 0; i < N_SOURCES; i++) {
        if (!g_src[i].active) continue;
        int cr = (int)g_src[i].fr;
        int cc = (int)g_src[i].fc;
        if (cr >= 0 && cr < g_gh && cc >= 0 && cc < g_gw)
            g_u[cr][cc] += OSC_AMP * sinf(g_src[i].phase);
        g_src[i].phase += OSC_FREQ;
    }
    g_step++;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static int  g_speed = STEPS_PER_FRAME;

static void scene_draw(void)
{
    for (int r = HUD_ROWS; r < g_gh && r < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            float u = g_u[r][c];
            int   cp; chtype ch;
            if      (u < -0.60f) { cp = CP_BL2; ch = '#'; }
            else if (u < -0.20f) { cp = CP_BL1; ch = '+'; }
            else if (u < -0.05f) { cp = CP_BL0; ch = '.'; }
            else if (u <  0.05f) { cp = 0;       ch = ' '; }
            else if (u <  0.20f) { cp = CP_WH0; ch = '.'; }
            else if (u <  0.60f) { cp = CP_WH1; ch = '+'; }
            else                 { cp = CP_WH2; ch = '#'; }

            if (cp) {
                attron(COLOR_PAIR(cp));
                mvaddch(r, c, ch);
                attroff(COLOR_PAIR(cp));
            }
        }
    }

    /* mark active sources */
    for (int i = 0; i < N_SOURCES; i++) {
        if (!g_src[i].active) continue;
        int r = (int)g_src[i].fr, c = (int)g_src[i].fc;
        if (r >= HUD_ROWS && r < g_rows && c < g_cols) {
            attron(COLOR_PAIR(CP_SRC) | A_BOLD);
            mvaddch(r, c, (chtype)('1' + i));
            attroff(COLOR_PAIR(CP_SRC) | A_BOLD);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Wave2D  q:quit  spc:impulse  1-5:toggle osc  p:pause  r:reset  +/-:speed  c:clear");
    mvprintw(1, 0,
        " CFL=0.40  C²=%.2f  speed:%dx  step:%lld  %s",
        C2, g_speed, (long long)g_step,
        g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

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
    g_gh = rows < GRID_H_MAX ? rows : GRID_H_MAX;
    g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    sources_init(g_gh, g_gw);
    grid_clear();
    inject_impulse(g_gh/2, g_gw/2, IMPULSE_AMP);

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = rows < GRID_H_MAX ? rows : GRID_H_MAX;
            g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
            sources_init(g_gh, g_gw);
            grid_clear();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': inject_impulse(g_gh/2, g_gw/2, IMPULSE_AMP); break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': grid_clear(); g_step = 0; break;
        case 'c': case 'C':
            for (int i = 0; i < N_SOURCES; i++) g_src[i].active = false;
            break;
        case '1': case '2': case '3': case '4': case '5': {
            int i = ch - '1';
            g_src[i].active = !g_src[i].active;
            g_src[i].phase  = 0.f;
            break;
        }
        case '+': case '=': g_speed++; if (g_speed > 16) g_speed = 16; break;
        case '-': g_speed--; if (g_speed < 1) g_speed = 1; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused) {
            for (int s = 0; s < g_speed; s++) {
                sources_step();
                grid_step();
            }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
