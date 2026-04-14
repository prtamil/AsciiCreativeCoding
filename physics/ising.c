/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ising.c — Ising Model  (Magnetic Phase Transition)
 *
 * Each cell is a spin: +1 (up) or −1 (down).  The Monte Carlo Metropolis
 * algorithm proposes random single-spin flips with acceptance probability
 *
 *   P(flip) = min(1,  exp(−ΔE / kT))   where  ΔE = 2·s·Σ_nbr
 *
 * At high temperature T, spins are random (paramagnetic phase).
 * Cool below the critical temperature Tc ≈ 2.269 / ln(1+√2) ≈ 2.269 and
 * magnetic domains spontaneously appear (ferromagnetic phase).
 *
 * The HUD shows temperature and mean magnetisation |⟨s⟩|.
 *
 * Keys: q quit  ↑↓ temperature  p pause  r randomise  h hot  c cold  t/T theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/ising.c \
 *       -o ising -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 grid  §5 draw  §6 app
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

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_ROWS      2
#define N_THEMES     10

#define T_INIT   3.0f   /* start hot                 */
#define T_CRIT   2.269f /* 2D Ising critical temp    */
#define T_MIN    0.1f
#define T_MAX    5.0f
#define T_STEP   0.05f

/* Flips per frame — proportional to grid size so rate is cells/step */
#define FLIPS_PER_CELL   50
#define RENDER_NS  (1000000000LL / 30)

enum { CP_UP=1, CP_DN, CP_HUD };

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
/* §3  color / theme                                                      */
/* ===================================================================== */

typedef struct {
    const char *name;
    short  up_fg,  dn_fg;    /* 256-color foreground for each spin state */
    short  up_fg8, dn_fg8;   /* 8-color fallback                         */
    chtype up_ch,  dn_ch;    /* character drawn for each spin state       */
} Theme;

/*
 * 10 themes — color pairs and characters give each lattice its identity.
 *
 *  Matrix  — phosphor green on black, classic CRT look
 *  Nova    — white stars scattered across a deep-purple void
 *  Mono    — pure greyscale, minimal
 *  Fire    — orange flame flickering over dark ember
 *  Ocean   — cyan ripple on deep navy
 *  Void    — hot-magenta pinpoints in absolute black
 *  Amber   — warm yellow on dark brown
 *  Neon    — hot pink on dark purple, cyberpunk
 *  Ice     — pale blue on cold navy
 *  Plasma  — red-hot against electric blue
 */
static const Theme k_themes[N_THEMES] = {
/*  name      up256  dn256  up8            dn8           up_ch  dn_ch */
  { "Matrix",    46,    22, COLOR_GREEN,   COLOR_GREEN,  '#',   '.'  },
  { "Nova",     231,    57, COLOR_WHITE,   COLOR_BLUE,   '*',   ' '  },
  { "Mono",     231,   238, COLOR_WHITE,   COLOR_BLACK,  '#',   '.'  },
  { "Fire",     214,    88, COLOR_YELLOW,  COLOR_RED,    '^',   '.'  },
  { "Ocean",     51,    17, COLOR_CYAN,    COLOR_BLUE,   '~',   '-'  },
  { "Void",     201,    16, COLOR_MAGENTA, COLOR_BLACK,  '@',   ' '  },
  { "Amber",    226,    94, COLOR_YELLOW,  COLOR_RED,    '#',   '.'  },
  { "Neon",     199,    54, COLOR_MAGENTA, COLOR_BLUE,   '+',   '-'  },
  { "Ice",      159,    25, COLOR_CYAN,    COLOR_BLUE,   '*',   '.'  },
  { "Plasma",   196,    21, COLOR_RED,     COLOR_BLUE,   '#',   '.'  },
};

static int g_theme = 0;

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    if (COLORS >= 256) {
        init_pair(CP_UP, t->up_fg, -1);
        init_pair(CP_DN, t->dn_fg, -1);
    } else {
        init_pair(CP_UP, t->up_fg8, -1);
        init_pair(CP_DN, t->dn_fg8, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_HUD, COLORS >= 256 ? 244 : COLOR_WHITE, -1);
    theme_apply(g_theme);
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static signed char g_spin[GRID_H_MAX][GRID_W_MAX]; /* +1 or -1 */
static int g_gh, g_gw;
static float g_temp = T_INIT;
static long long g_sweeps = 0;

/* Precomputed Boltzmann factors: dE can be -8,-4,0,4,8 (2D sq lattice) */
/* Keyed by dE/4 ∈ {-2,-1,0,1,2}: only need factors for dE=4 and dE=8 */
static float g_boltz[3];   /* [0]=exp(-4/T), [1]=exp(-8/T) */

static void boltz_update(void)
{
    g_boltz[0] = expf(-4.f / g_temp);
    g_boltz[1] = expf(-8.f / g_temp);
}

static void grid_random(void)
{
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            g_spin[r][c] = (rand() & 1) ? 1 : -1;
    g_sweeps = 0;
}

static void grid_step(int n_flips)
{
    for (int f = 0; f < n_flips; f++) {
        int r = rand() % g_gh;
        int c = rand() % g_gw;
        int s = g_spin[r][c];

        int nb = g_spin[r>0?r-1:g_gh-1][c]
               + g_spin[r<g_gh-1?r+1:0][c]
               + g_spin[r][c>0?c-1:g_gw-1]
               + g_spin[r][c<g_gw-1?c+1:0];

        int dE = 2 * s * nb;   /* ΔE if we flip s: dE = 2s·Σnbr */

        bool flip;
        if (dE <= 0) {
            flip = true;
        } else {
            /* dE is 4 or 8 */
            float prob = (dE == 4) ? g_boltz[0] : g_boltz[1];
            flip = ((float)rand() / (float)RAND_MAX) < prob;
        }
        if (flip) g_spin[r][c] = (signed char)-s;
    }
    g_sweeps++;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;

static float magnetisation(void)
{
    long long sum = 0;
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            sum += g_spin[r][c];
    return fabsf((float)sum / (float)(g_gh * g_gw));
}

static void scene_draw(void)
{
    const Theme *th = &k_themes[g_theme];
    for (int r = 0; r < g_gh && r+HUD_ROWS < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            int    cp;
            chtype ch;
            if (g_spin[r][c] > 0) { cp = CP_UP; ch = th->up_ch; }
            else                   { cp = CP_DN; ch = th->dn_ch; }
            attron(COLOR_PAIR(cp));
            mvaddch(r + HUD_ROWS, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    float m = magnetisation();
    float tc_dist = fabsf(g_temp - T_CRIT);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Ising  q:quit  ↑↓:temp  p:pause  r:random  h:hot  c:cold  t/T:theme");
    mvprintw(1, 0,
        " T=%.3f (Tc=%.3f %s)  |M|=%.4f  sweeps:%lld  [%s]  %s",
        g_temp, T_CRIT,
        tc_dist < 0.1f ? "≈Tc" : (g_temp < T_CRIT ? "ordered" : "disordered"),
        m, g_sweeps,
        k_themes[g_theme].name,
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
    boltz_update();
    grid_random();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
            g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
            grid_random();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': grid_random(); break;
        case 'h': case 'H':
            g_temp = T_MAX; boltz_update(); grid_random(); break;
        case 'c': case 'C':
            g_temp = 0.5f; boltz_update(); break;
        case KEY_UP:
            g_temp += T_STEP;
            if (g_temp > T_MAX) g_temp = T_MAX;
            boltz_update();
            break;
        case KEY_DOWN:
            g_temp -= T_STEP;
            if (g_temp < T_MIN) g_temp = T_MIN;
            boltz_update();
            break;
        case 't':
            g_theme = (g_theme + 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        case 'T':
            g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused)
            grid_step(g_gh * g_gw * FLIPS_PER_CELL / 1000);

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
