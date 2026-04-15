/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lenia.c — Lenia (Continuous Life)
 *
 * Continuous-state, continuous-time generalisation of Game of Life.
 * Each cell u ∈ [0,1] evolves via:
 *
 *   u(t+dt) = clip(u(t) + dt · (2·G(K⊛u(t)) - 1), 0, 1)
 *
 * where K is a ring-shaped convolution kernel (all weight on a hollow shell
 * at radius R) and G is a Gaussian-shaped growth function:
 *
 *   G(x) = exp(-((x-μ)/σ)² / 2)
 *
 * With the right parameters (μ=0.15, σ=0.015) self-organising "creatures"
 * emerge and glide across the screen.
 *
 * Presets (1-3):
 *   1  Orbium  — μ=0.15 σ=0.015 R=13 dt=0.10  (the classic glider)
 *   2  Aquarium — μ=0.26 σ=0.036 R=10 dt=0.10  (jellyfish-like)
 *   3  Scutium  — μ=0.17 σ=0.015 R=8  dt=0.15  (shield shape)
 *
 * Keys: q quit  1/2/3 preset  p pause  r random  +/- speed  SPACE impulse
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/lenia.c \
 *       -o lenia -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 kernel  §5 grid  §6 draw  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Continuous cellular automaton with convolution kernel.
 *                  Each step: convolve the state grid with kernel K (weighted
 *                  ring), evaluate growth function G, apply update rule.
 *                  Naïve O(W·H·πR²) convolution — for R=13 and a 200×60 grid
 *                  that's ~6M ops per step.
 *
 * Physics/Biology: Lenia (Bert Wang-Chak Chan, 2019).
 *                  A continuous generalisation of Conway's Game of Life.
 *                  The ring kernel captures "neighbourhood density at radius R"
 *                  — analogous to how a biological cell senses chemical
 *                  gradients from nearby cells.  Self-organisation emerges
 *                  from the tension between growth (G>0.5) and decay (G<0.5).
 *
 * Math           : Growth function G(x) = exp(−(x−μ)²/(2σ²)).
 *                  This is a Gaussian centred at μ.  When the kernel
 *                  convolution result matches μ exactly, G=1 and the cell
 *                  grows toward 1.  Far from μ, G→0 and the cell decays.
 *                  The parameter pair (μ, σ) defines a "species" of creature.
 *
 * Performance    : The convolution sums weights from all cells within
 *                  radius R.  Normalised so total kernel weight = 1.
 *                  Pre-building the kernel O(R²) once amortises the cost
 *                  across all (W×H) cells per step.
 * ─────────────────────────────────────────────────────────────────────── */

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

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_ROWS      2
#define KERNEL_R_MAX 20   /* max kernel radius in cells */
#define RENDER_NS   (1000000000LL / 30)

typedef struct {
    const char *name;
    float mu, sigma, R, dt;
} Preset;

static const Preset PRESETS[] = {
    { "Orbium",   0.150f, 0.015f, 13.f, 0.10f },
    { "Aquarium", 0.260f, 0.036f, 10.f, 0.10f },
    { "Scutium",  0.170f, 0.015f,  8.f, 0.15f },
};
#define N_PRESETS  3

static int   g_preset = 0;
static float g_mu, g_sigma, g_kern_R, g_dt;

enum { CP_U0=1, CP_U1, CP_U2, CP_U3, CP_U4, CP_HUD };

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
        init_pair(CP_U0,  17, -1);   /* dark navy    — 0.0-0.2  */
        init_pair(CP_U1,  27, -1);   /* blue         — 0.2-0.4  */
        init_pair(CP_U2,  51, -1);   /* cyan         — 0.4-0.6  */
        init_pair(CP_U3, 195, -1);   /* pale cyan    — 0.6-0.8  */
        init_pair(CP_U4, 231, -1);   /* white        — 0.8-1.0  */
        init_pair(CP_HUD,244, -1);
    } else {
        init_pair(CP_U0, COLOR_BLUE,  -1);
        init_pair(CP_U1, COLOR_BLUE,  -1);
        init_pair(CP_U2, COLOR_CYAN,  -1);
        init_pair(CP_U3, COLOR_CYAN,  -1);
        init_pair(CP_U4, COLOR_WHITE, -1);
        init_pair(CP_HUD,COLOR_WHITE, -1);
    }
}

/* ===================================================================== */
/* §4  kernel                                                             */
/* ===================================================================== */

/* Kernel weights: ring at radius R, Gaussian shell with thickness 0.2R */
static float g_kw[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
static int   g_kdr[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
static int   g_kdc[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
static int   g_kn;   /* number of kernel entries */

static void kernel_build(float R)
{
    g_kn = 0;
    int iR = (int)ceilf(R);
    if (iR > KERNEL_R_MAX) iR = KERNEL_R_MAX;
    float total = 0.f;

    /* Ring kernel: weight = exp(-((r-R)/0.2R)^2/2) */
    for (int dr = -iR; dr <= iR; dr++) {
        for (int dc = -iR; dc <= iR; dc++) {
            float r = sqrtf((float)(dr*dr + dc*dc));
            float x = (r - R) / (0.2f * R + 0.001f);
            float w = expf(-x*x * 0.5f);
            if (w < 0.001f) continue;
            int k = g_kn++;
            g_kdr[k] = dr; g_kdc[k] = dc; g_kw[k] = w;
            total += w;
        }
    }
    /* normalize */
    if (total > 0.f)
        for (int k = 0; k < g_kn; k++) g_kw[k] /= total;
}

static float growth(float U)
{
    float x = (U - g_mu) / g_sigma;
    return expf(-x*x * 0.5f);
}

/* ===================================================================== */
/* §5  grid                                                               */
/* ===================================================================== */

static float g_u[GRID_H_MAX][GRID_W_MAX];
static float g_u2[GRID_H_MAX][GRID_W_MAX];
static int   g_gh, g_gw;
static int   g_speed = 1;

static void grid_random(void)
{
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            g_u[r][c] = (float)rand()/RAND_MAX;
}

static void grid_clear(void)
{
    memset(g_u, 0, sizeof g_u);
}

/* Place an "orbium seed" at center */
static void grid_seed_center(void)
{
    grid_clear();
    int cr = g_gh/2, cc = g_gw/2;
    int R = (int)g_kern_R;
    for (int dr = -R; dr <= R; dr++)
        for (int dc = -R; dc <= R; dc++) {
            int r = cr+dr, c = cc+dc;
            if (r<0||r>=g_gh||c<0||c>=g_gw) continue;
            float rr = sqrtf((float)(dr*dr + dc*dc));
            if (rr <= R * 0.8f)
                g_u[r][c] = (float)rand()/RAND_MAX;
        }
}

static void grid_step(void)
{
    for (int r = 0; r < g_gh; r++) {
        for (int c = 0; c < g_gw; c++) {
            /* convolution: sum kernel-weighted neighbors */
            float U = 0.f;
            for (int k = 0; k < g_kn; k++) {
                int nr = (r + g_kdr[k] + g_gh) % g_gh;
                int nc = (c + g_kdc[k] + g_gw) % g_gw;
                U += g_kw[k] * g_u[nr][nc];
            }
            float du = g_dt * (2.f * growth(U) - 1.f);
            float nv = g_u[r][c] + du;
            g_u2[r][c] = nv < 0.f ? 0.f : nv > 1.f ? 1.f : nv;
        }
    }
    memcpy(g_u, g_u2, sizeof g_u);
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

static void scene_draw(void)
{
    for (int r = 0; r < g_gh && r+HUD_ROWS < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            float u = g_u[r][c];
            int cp; chtype ch;
            if      (u < 0.05f) continue;
            else if (u < 0.20f) { cp = CP_U0; ch = '.'; }
            else if (u < 0.40f) { cp = CP_U1; ch = ':'; }
            else if (u < 0.60f) { cp = CP_U2; ch = '+'; }
            else if (u < 0.80f) { cp = CP_U3; ch = '*'; }
            else                { cp = CP_U4; ch = '#'; }
            attron(COLOR_PAIR(cp));
            mvaddch(r + HUD_ROWS, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Lenia  q:quit  1/2/3:preset  p:pause  r:random  +/-:speed  spc:seed");
    mvprintw(1, 0,
        " [%d] %-9s μ=%.3f σ=%.3f R=%.0f dt=%.2f  speed:%dx  %s",
        g_preset+1, PRESETS[g_preset].name,
        g_mu, g_sigma, g_kern_R, g_dt, g_speed,
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

static void load_preset(int p)
{
    g_preset = p;
    g_mu     = PRESETS[p].mu;
    g_sigma  = PRESETS[p].sigma;
    g_kern_R = PRESETS[p].R;
    g_dt     = PRESETS[p].dt;
    kernel_build(g_kern_R);
    grid_seed_center();
}

int main(void)
{
    srand((unsigned)time(NULL));
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
    load_preset(0);

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
            g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
            load_preset(g_preset);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': grid_random(); break;
        case ' ': grid_seed_center(); break;
        case '1': load_preset(0); break;
        case '2': load_preset(1); break;
        case '3': load_preset(2); break;
        case '+': case '=': g_speed++; if (g_speed > 4) g_speed = 4; break;
        case '-': g_speed--; if (g_speed < 1) g_speed = 1; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused)
            for (int s = 0; s < g_speed; s++) grid_step();

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
