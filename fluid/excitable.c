/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * excitable.c — Greenberg-Hastings Excitable Medium
 *
 * N-state cellular automaton on the terminal grid:
 *
 *   State 0          Resting    — quiescent, susceptible to excitation
 *   State 1          Excited    — firing; can excite any 4-neighbour
 *   States 2..N-1    Refractory — recovering; immune to excitation
 *
 * Rule (von Neumann neighbourhood, periodic boundary):
 *   cell == 0 AND any neighbour == 1  →  cell = 1   (excite)
 *   cell >  0                         →  cell = (cell+1) % N  (advance)
 *
 * N (total states, min 5, max 20) sets the refractory depth:
 *   small N → short recovery, dense fast waves
 *   large N → long recovery, sparse slow waves
 *
 * 4 presets:
 *   1 SPIRAL  — broken wave front; open end curls into a rotating spiral
 *   2 DOUBLE  — two counter-rotating spirals from offset broken fronts
 *   3 RINGS   — concentric target waves (radially periodic IC)
 *   4 CHAOS   — random 5% seeding → turbulent multi-spiral field
 *
 * Keys: q quit  p pause  r reset  1-4 preset  +/- N  t/T theme  spc pulse
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/excitable.c \
 *       -o excitable -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 grid  §5 presets  §6 draw  §7 app
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

#define GRID_H_MAX  120
#define GRID_W_MAX  360

#define N_MIN         5    /* minimum refractory depth */
#define N_MAX        20    /* maximum — also max color pairs allocated */
#define N_DEF        12    /* default */

#define RENDER_NS  (1000000000LL / 30)   /* display: 30 fps  */
#define STEP_NS    (1000000000LL / 15)   /* CA step: 15 /sec */
#define HUD_ROWS    2
#define N_THEMES    5

/* Color pair layout: 1=HUD, 2..N_MAX+1=state 0..N_MAX-1 */
enum { CP_HUD = 1, CP_S0 = 2 };

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

/*
 * Each theme defines a 12-entry 256-colour ramp: index 0 = peak (state 1),
 * index 11 = darkest (state 0).  States 2..N-1 interpolate through 1..9.
 */
static const char *k_theme_names[N_THEMES] = {
    "Fire", "Ice", "Matrix", "Plasma", "Mono"
};

static const short k_ramps256[N_THEMES][12] = {
    /* Fire:   white → yellow → orange → red → dark */
    { 231, 230, 226, 220, 214, 208, 202, 196, 160,  88,  52,  16 },
    /* Ice:    white → cyan → blue → dark navy */
    { 231, 195, 159, 123,  87,  51,  45,  39,  33,  27,  21,  17 },
    /* Matrix: white → bright-green → green → black */
    { 231,  82,  46,  40,  34,  28,  22,  22,  16,  16,  16,  16 },
    /* Plasma: white → magenta → purple → dark */
    { 231, 213, 207, 201, 165, 129,  93,  57,  54,  53,  17,  16 },
    /* Mono:   white → grey gradient → black */
    { 231, 252, 248, 244, 240, 236, 234, 233, 232, 232,  16,  16 },
};

static const short k_ramps8[N_THEMES][3] = {  /* peak / mid / dim */
    { COLOR_WHITE, COLOR_YELLOW,  COLOR_RED     },
    { COLOR_WHITE, COLOR_CYAN,    COLOR_BLUE    },
    { COLOR_WHITE, COLOR_GREEN,   COLOR_GREEN   },
    { COLOR_WHITE, COLOR_MAGENTA, COLOR_BLUE    },
    { COLOR_WHITE, COLOR_WHITE,   COLOR_BLACK   },
};

static int g_theme = 0;
static int g_n     = N_DEF;   /* current total state count */

/* Recompute all state color pairs.  Call when theme OR g_n changes. */
static void theme_apply(int ti)
{
    init_pair(CP_HUD, COLORS >= 256 ? 244 : COLOR_WHITE, -1);

    for (int s = 0; s < N_MAX; s++) {
        int ri;
        if      (s == 0) ri = 11;   /* resting: darkest */
        else if (s == 1) ri = 0;    /* excited: brightest */
        else {
            ri = 1 + (s - 2) * 9 / (g_n > 2 ? g_n - 2 : 1);
            if (ri > 10) ri = 10;
        }
        short fg = (COLORS >= 256) ? k_ramps256[ti][ri]
                 : (s == 0) ? k_ramps8[ti][2]
                 : (s == 1) ? k_ramps8[ti][0]
                 :             k_ramps8[ti][1];
        init_pair((short)(CP_S0 + s), fg, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static uint8_t g_gridA[GRID_H_MAX][GRID_W_MAX];
static uint8_t g_gridB[GRID_H_MAX][GRID_W_MAX];
static uint8_t (*g_cur)[GRID_W_MAX];
static uint8_t (*g_nxt)[GRID_W_MAX];

static int  g_gh, g_gw;
static bool g_paused = false;

static void grid_clear(void)
{
    memset(g_gridA, 0, sizeof(g_gridA));
    memset(g_gridB, 0, sizeof(g_gridB));
    g_cur = g_gridA;
    g_nxt = g_gridB;
}

/*
 * One GH CA step with 4-neighbour (von Neumann) connectivity
 * and periodic (toroidal) boundaries.
 */
static void ca_step(void)
{
    int n = g_n;
    for (int r = 0; r < g_gh; r++) {
        int ru = (r > 0)      ? r - 1 : g_gh - 1;
        int rd = (r < g_gh-1) ? r + 1 : 0;
        for (int c = 0; c < g_gw; c++) {
            int cl = (c > 0)      ? c - 1 : g_gw - 1;
            int cr = (c < g_gw-1) ? c + 1 : 0;
            uint8_t s = g_cur[r][c];
            uint8_t ns;
            if (s == 0) {
                /* Resting: excite if any von-Neumann neighbour is excited */
                ns = (g_cur[ru][c] == 1 ||
                      g_cur[rd][c] == 1 ||
                      g_cur[r][cl] == 1 ||
                      g_cur[r][cr] == 1) ? 1 : 0;
            } else {
                /* Advance through refractory, then back to resting */
                ns = (uint8_t)((s + 1) % n);
            }
            g_nxt[r][c] = ns;
        }
    }
    /* swap buffers */
    uint8_t (*tmp)[GRID_W_MAX] = g_cur;
    g_cur = g_nxt;
    g_nxt = tmp;
}

/* ===================================================================== */
/* §5  presets                                                            */
/* ===================================================================== */

static int g_preset = 1;

static void preset_apply(int p)
{
    grid_clear();
    int gh = g_gh, gw = g_gw, n = g_n;
    uint8_t (*cur)[GRID_W_MAX] = g_cur;

    if (p == 1) {
        /*
         * SPIRAL — a horizontal wave front covering the left half of
         * the middle row, with a refractory wake above it.
         * The open right end has no downstream support and curls
         * into a self-sustaining rotating spiral.
         */
        int mr = gh / 2, mc = gw / 2;
        for (int c = 0; c < mc; c++) {
            cur[mr][c] = 1;
            for (int dr = 1; dr < n - 1 && mr - dr >= 0; dr++)
                cur[mr - dr][c] = (uint8_t)(dr + 1);
        }

    } else if (p == 2) {
        /*
         * DOUBLE — two broken fronts placed in opposite quadrants.
         * Each open end curls the opposite way, producing a pair of
         * counter-rotating spirals that interact but never annihilate.
         */
        int mr1 = gh / 3, mr2 = gh * 2 / 3, mc = gw / 2;
        for (int c = 0; c < mc; c++) {
            cur[mr1][c] = 1;
            for (int dr = 1; dr < n - 1 && mr1 - dr >= 0; dr++)
                cur[mr1 - dr][c] = (uint8_t)(dr + 1);
        }
        for (int c = mc; c < gw; c++) {
            cur[mr2][c] = 1;
            for (int dr = 1; dr < n - 1 && mr2 + dr < gh; dr++)
                cur[mr2 + dr][c] = (uint8_t)(dr + 1);
        }

    } else if (p == 3) {
        /*
         * RINGS — radially periodic initial condition.
         * dist % N gives each ring its phase; the ×2 row factor corrects
         * for the terminal's 2:1 character aspect ratio so rings look circular.
         */
        int cr = gh / 2, cc = gw / 2;
        for (int r = 0; r < gh; r++) {
            for (int c = 0; c < gw; c++) {
                float dr   = (float)(r - cr) * 2.0f;  /* aspect correction */
                float dc   = (float)(c - cc);
                int   dist = (int)sqrtf(dr * dr + dc * dc);
                int   s    = (n - 1) - (dist % n);
                cur[r][c]  = (uint8_t)(s >= 0 ? s : 0);
            }
        }

    } else {
        /*
         * CHAOS — ~5% of cells seeded as excited, the rest resting.
         * Many small expanding rings collide, nucleate spirals, and
         * fill the screen with turbulent multi-spiral activity.
         */
        for (int r = 0; r < gh; r++)
            for (int c = 0; c < gw; c++)
                cur[r][c] = (rand() % 20 == 0) ? 1 : 0;
    }
}

/* Trigger a small rectangular pulse at the centre. */
static void pulse_centre(void)
{
    int cr = g_gh / 2, cc = g_gw / 2;
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r >= 0 && r < g_gh && c >= 0 && c < g_gw)
                g_cur[r][c] = 1;
        }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int g_rows, g_cols;

/*
 * Refractory character ramp (7 entries): just-excited → about-to-rest.
 * State 1 → '#' (bold), states 2..N-1 index into k_rchar[].
 */
static const char k_rchar[] = "@*+=-.'";

static void scene_draw(void)
{
    for (int r = 0; r < g_gh && r + HUD_ROWS < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            int s = g_cur[r][c];
            if (s == 0) continue;   /* resting: leave blank */

            chtype ch;
            attr_t at;
            if (s == 1) {
                ch = '#'; at = A_BOLD;
            } else {
                int ri = (s - 2) * 7 / (g_n > 2 ? g_n - 2 : 1);
                if (ri >= 7) ri = 6;
                ch = (chtype)(unsigned char)k_rchar[ri];
                at = (s == 2) ? A_BOLD : A_NORMAL;
            }
            attron(COLOR_PAIR(CP_S0 + s) | at);
            mvaddch(r + HUD_ROWS, c, ch);
            attroff(COLOR_PAIR(CP_S0 + s) | at);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Excitable  q:quit  p:pause  r:reset  1-4:preset"
        "  +/-:N  t/T:theme  spc:pulse");
    mvprintw(1, 0,
        " N=%d (refractory depth)  preset=%s  [%s]  %s",
        g_n,
        g_preset == 1 ? "Spiral" :
        g_preset == 2 ? "Double" :
        g_preset == 3 ? "Rings"  : "Chaos",
        k_theme_names[g_theme],
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

static void resize_and_reset(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    preset_apply(g_preset);
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
    resize_and_reset();

    long long step_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            resize_and_reset();
            last_ns  = clock_ns();
            step_acc = 0;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R':
            preset_apply(g_preset);
            step_acc = 0;
            break;
        case '1': case '2': case '3': case '4':
            g_preset = ch - '0';
            preset_apply(g_preset);
            step_acc = 0;
            break;
        case ' ':
            pulse_centre();
            break;
        case '+': case '=':
            if (g_n < N_MAX) {
                g_n++;
                theme_apply(g_theme);
                /* increasing N is safe: no existing state exceeds new limit */
            }
            break;
        case '-':
            if (g_n > N_MIN) {
                g_n--;
                theme_apply(g_theme);
                /* clamp any states that now exceed the new N */
                for (int r = 0; r < g_gh; r++)
                    for (int c = 0; c < g_gw; c++)
                        if (g_cur[r][c] >= (uint8_t)g_n)
                            g_cur[r][c] = 0;
            }
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

        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        if (dt > 100000000LL) dt = 100000000LL;
        last_ns = now_ns;

        if (!g_paused) {
            step_acc += dt;
            while (step_acc >= STEP_NS) {
                ca_step();
                step_acc -= STEP_NS;
            }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }
    return 0;
}
