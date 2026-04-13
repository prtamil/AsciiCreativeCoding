/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bifurcation.c — Logistic Map Bifurcation Diagram
 *
 * x_{n+1} = r · x_n · (1 − x_n)
 *
 * For each screen column a distinct r value is chosen.  After WARMUP
 * transient iterations the next PLOT values are plotted as coloured dots;
 * their y position encodes x ∈ (0,1).  The result is the classic
 * period-doubling cascade ending in chaos at r ≈ 3.5699 (Feigenbaum δ).
 *
 * The view auto-zooms toward the accumulation point so the self-similar
 * fractal structure gradually fills the screen.  Press SPACE to pause.
 *
 * Keys:  q quit   ← → pan   + - zoom   r reset   spc pause auto-zoom
 *
 * Color encodes x value: blue(low) → cyan → green → yellow → red(high).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/bifurcation.c \
 *       -o bifurcation -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 diagram  §5 screen  §6 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define WARMUP        500     /* transient iterations discarded per column   */
#define PLOT          300     /* attractor values plotted per column          */
#define HUD_ROWS        3     /* rows reserved at top for HUD                */
#define RENDER_NS  (1000000000LL / 30)
#define ZOOM_FACTOR  1.25f
#define PAN_FRAC     0.12f    /* pan moves this fraction of current width     */

/* Feigenbaum accumulation point: r∞ where period-doubling cascades to chaos */
#define FEIGENBAUM_R  3.5699456718695f

enum { CP_B=1, CP_C, CP_G, CP_Y, CP_R, CP_HUD };

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
        init_pair(CP_B,   33, -1);   /* blue   — low x      */
        init_pair(CP_C,   51, -1);   /* cyan                */
        init_pair(CP_G,   46, -1);   /* green               */
        init_pair(CP_Y,  226, -1);   /* yellow              */
        init_pair(CP_R,  196, -1);   /* red    — high x     */
        init_pair(CP_HUD, 226, -1);   /* grey   — HUD text   */
    } else {
        init_pair(CP_B,   COLOR_BLUE,   -1);
        init_pair(CP_C,   COLOR_CYAN,   -1);
        init_pair(CP_G,   COLOR_GREEN,  -1);
        init_pair(CP_Y,   COLOR_YELLOW, -1);
        init_pair(CP_R,   COLOR_RED,    -1);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  diagram                                                            */
/* ===================================================================== */

static int   g_rows, g_cols;
static float g_r_min  = 2.50f;
static float g_r_max  = 4.00f;
static bool  g_paused = false;

/*
 * diagram_draw() — inner loop of the bifurcation computation.
 *
 * Each column maps to one r value.  The logistic map is iterated
 * WARMUP + PLOT times; the first WARMUP values are discarded (transient).
 * Each surviving x is plotted at the matching screen row.
 * Column x-values use the current [r_min, r_max] window.
 */
static void diagram_draw(void)
{
    int plot_h = g_rows - HUD_ROWS;
    if (plot_h < 2) return;

    float r_span = g_r_max - g_r_min;

    for (int cx = 0; cx < g_cols; cx++) {
        float r = g_r_min + r_span * (float)cx / (float)(g_cols - 1);
        if (r <= 0.f || r > 4.f) continue;

        float x = 0.5f;
        for (int i = 0; i < WARMUP; i++)
            x = r * x * (1.f - x);

        for (int i = 0; i < PLOT; i++) {
            x = r * x * (1.f - x);
            int row = HUD_ROWS + plot_h - 1
                    - (int)(x * (float)(plot_h - 1) + 0.5f);
            if (row < HUD_ROWS || row >= g_rows) continue;

            int cp = x < 0.2f ? CP_B :
                     x < 0.4f ? CP_C :
                     x < 0.6f ? CP_G :
                     x < 0.8f ? CP_Y : CP_R;
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(row, cx, '.');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }
}

static void hud_draw(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Bifurcation  q:quit  ←→:pan  +/-:zoom  r:reset  spc:pause auto-zoom");

    mvprintw(1, 0, " r ∈ [%.6f, %.6f]  Δr = %.6f  target: r∞=%.6f  %s",
        g_r_min, g_r_max, g_r_max - g_r_min, FEIGENBAUM_R,
        g_paused ? "PAUSED" : "zooming→r∞");

    /* r-axis tick labels at 5 evenly spaced positions */
    for (int i = 0; i <= 4; i++) {
        int cx = (int)((float)i / 4.f * (float)(g_cols - 1));
        float r = g_r_min + (g_r_max - g_r_min) * (float)i / 4.f;
        if (cx < g_cols - 6)
            mvprintw(2, cx, "%.3f", r);
    }
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §5  screen + §6  app                                                   */
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
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);

    long long last = clock_ns();

    while (!g_quit) {

        /* ── resize ─────────────────────────────────────────────── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            last = clock_ns();
            continue;
        }

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        float w = g_r_max - g_r_min;

        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;

        case ' ':
            g_paused = !g_paused;
            break;

        case 'r': case 'R':
            g_r_min = 2.50f; g_r_max = 4.00f; g_paused = false;
            break;

        case KEY_LEFT: {
            float p = w * PAN_FRAC;
            g_r_min -= p; g_r_max -= p;
            if (g_r_min < 0.f) { g_r_max += -g_r_min; g_r_min = 0.f; }
            break;
        }
        case KEY_RIGHT: {
            float p = w * PAN_FRAC;
            g_r_min += p; g_r_max += p;
            if (g_r_max > 4.f) { g_r_min -= g_r_max - 4.f; g_r_max = 4.f; }
            break;
        }
        case '+': case '=': {
            float c = (g_r_min + g_r_max) * 0.5f;
            float hw = w / ZOOM_FACTOR * 0.5f;
            g_r_min = c - hw; g_r_max = c + hw;
            if (g_r_min < 0.f) g_r_min = 0.f;
            if (g_r_max > 4.f) g_r_max = 4.f;
            break;
        }
        case '-': {
            float c = (g_r_min + g_r_max) * 0.5f;
            float hw = w * ZOOM_FACTOR * 0.5f;
            g_r_min = c - hw; g_r_max = c + hw;
            if (g_r_min < 0.f) g_r_min = 0.f;
            if (g_r_max > 4.f) g_r_max = 4.f;
            break;
        }
        default: break;
        }

        /*
         * Auto-zoom: slowly shrink the window toward the Feigenbaum
         * accumulation point (0.3% smaller each frame).
         * Stops when width < 0.002 to avoid numerical collapse.
         */
        if (!g_paused && (g_r_max - g_r_min) > 0.002f) {
            float c  = FEIGENBAUM_R;
            float hw = (g_r_max - g_r_min) * 0.5f * 0.997f;
            float lo = c - hw, hi = c + hw;
            if (lo < 0.f) lo = 0.f;
            if (hi > 4.f) hi = 4.f;
            g_r_min = lo; g_r_max = hi;
        }

        /* ── draw ────────────────────────────────────────────────── */
        long long now = clock_ns();
        last = now;
        (void)last;

        erase();
        hud_draw();
        diagram_draw();
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }

    return 0;
}
