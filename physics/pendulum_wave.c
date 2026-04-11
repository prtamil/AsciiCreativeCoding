/*
 * pendulum_wave.c — Pendulum Wave
 *
 * N=15 pendulums with lengths chosen so that pendulum n completes
 * (N_BASE + n) full oscillations in T_SYNC seconds:
 *
 *   ω_n = 2π (N_BASE + n) / T_SYNC
 *
 * At t=0 all swing in phase.  They drift apart into gorgeous wave patterns
 * and then "clap" back to perfect synchrony at t=T_SYNC.
 *
 * Each pendulum is drawn as a vertical string of '|' chars with a coloured
 * bob ('O') at the displaced position.  The colour cycles through the
 * rainbow across the pendulum array.
 *
 * Keys: q quit  p pause  r reset  +/- amplitude  SPACE jump T_SYNC
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/pendulum_wave.c \
 *       -o pendulum_wave -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 draw  §6 app
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_PEND       15         /* number of pendulums           */
#define N_BASE       40         /* lowest oscillation count      */
#define T_SYNC       60.0f      /* resync period (seconds)       */
#define AMP_INIT     0.70f      /* initial amplitude (0..1)      */
#define AMP_STEP     0.05f

#define SIM_DT       (1.0f / 60.0f)   /* sim step (seconds)   */
#define RENDER_NS    (1000000000LL / 60)
#define HUD_ROWS     3

enum {
    CP_P0=1, CP_P1, CP_P2, CP_P3, CP_P4,
    CP_P5, CP_P6, CP_P7, CP_P8, CP_P9,
    CP_PA, CP_PB, CP_PC, CP_PD, CP_PE,
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
/* §3  color — 15 rainbow colors + HUD                                   */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* 15-step rainbow: red → orange → yellow → green → cyan → blue → magenta */
        static const int pal256[15] = {
            196, 202, 208, 214, 226, 118, 46, 49, 51, 45, 27, 21, 57, 129, 201
        };
        for (int i = 0; i < N_PEND; i++)
            init_pair(CP_P0 + i, pal256[i], -1);
    } else {
        static const int pal8[15] = {
            COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
            COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
            COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE
        };
        for (int i = 0; i < N_PEND; i++)
            init_pair(CP_P0 + i, pal8[i], -1);
    }
    init_pair(CP_HUD, COLORS >= 256 ? 244 : COLOR_WHITE, -1);
}

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

static float g_omega[N_PEND];   /* angular frequency of each pendulum   */
static float g_time  = 0.f;
static float g_amp   = AMP_INIT;
static bool  g_paused;

static void physics_init(void)
{
    for (int i = 0; i < N_PEND; i++)
        g_omega[i] = (float)(2.0 * M_PI * (N_BASE + i) / T_SYNC);
}

/* θ_n(t) = amp · sin(ω_n · t) */
static float theta(int n, float t)
{
    return g_amp * sinf(g_omega[n] * t);
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int g_rows, g_cols;

static void scene_draw(void)
{
    int pend_area_rows = g_rows - HUD_ROWS;
    if (pend_area_rows < 4) return;

    int col_w = g_cols / N_PEND;
    if (col_w < 2) col_w = 2;

    /* Pivot row at top of pend area, bob row range below */
    int pivot_row = HUD_ROWS;

    for (int i = 0; i < N_PEND; i++) {
        int center_col = (int)((i + 0.5f) * g_cols / N_PEND);
        float th = theta(i, g_time);
        /* bob horizontal displacement in columns */
        int bob_col = center_col + (int)(th * ((float)g_cols / (float)N_PEND));
        /* bob vertical: pendulum length proportional to col band center */
        int string_len = pend_area_rows - 2;
        /* Use half-range for bob y position (always same string length) */
        /* Bob row based on pendulum length (shorter = higher = more freq) */
        float L_frac = (float)(N_BASE + i) / (float)(N_BASE + N_PEND - 1);
        int bob_row = pivot_row + 1 + (int)(string_len * L_frac);
        if (bob_row >= g_rows) bob_row = g_rows - 1;

        int cp = CP_P0 + i;

        /* pivot marker */
        if (pivot_row < g_rows)
            mvaddch(pivot_row, center_col, '^');

        /* string: line from pivot (pivot_row, center_col) to bob (bob_row, bob_col)
         * Interpolate column for each row; pick char from local slope. */
        int dr = bob_row - pivot_row;
        int dc = bob_col - center_col;
        float slope = (dr > 0) ? (float)dc / (float)dr : 0.f;
        chtype sch;
        if      (fabsf(slope) < 0.35f) sch = '|';
        else if (slope > 0)            sch = '\\';
        else                           sch = '/';

        attron(COLOR_PAIR(cp) | A_DIM);
        for (int s = 1; s < dr; s++) {
            int r = pivot_row + s;
            int c = center_col + (int)(slope * s + 0.5f);
            if (r < g_rows && c >= 0 && c < g_cols)
                mvaddch(r, c, sch);
        }
        attroff(COLOR_PAIR(cp) | A_DIM);

        /* bob: displaced horizontally */
        if (bob_row < g_rows && bob_col >= 0 && bob_col < g_cols) {
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(bob_row, bob_col, 'O');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    /* HUD */
    float t_rem = T_SYNC - fmodf(g_time, T_SYNC);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " PendulumWave  q:quit  p:pause  r:reset  +/-:amplitude  spc:sync");
    mvprintw(1, 0,
        " N=%d  N_base=%d  T_sync=%.0fs  t=%.2fs  T_next_sync=%.2fs",
        N_PEND, N_BASE, T_SYNC, g_time, t_rem);
    mvprintw(2, 0,
        " amplitude=%.2f  ω_range=[%.3f,%.3f] rad/s  %s",
        g_amp, g_omega[0], g_omega[N_PEND-1],
        g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
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
    getmaxyx(stdscr, g_rows, g_cols);
    physics_init();

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            last = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': g_time = 0.f; break;
        case '+': case '=':
            g_amp += AMP_STEP; if (g_amp > 0.95f) g_amp = 0.95f; break;
        case '-':
            g_amp -= AMP_STEP; if (g_amp < 0.05f) g_amp = 0.05f; break;
        case ' ':
            /* jump to next sync point */
            g_time = ceilf(g_time / T_SYNC) * T_SYNC;
            break;
        default: break;
        }

        long long now = clock_ns();
        long long dt_ns = now - last;
        last = now;
        if (dt_ns > 100000000LL) dt_ns = 100000000LL;

        if (!g_paused)
            g_time += (float)dt_ns * 1e-9f;

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
