/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * langton.c — Langton's Ant and Multi-Colour Turmites
 *
 * An ant walks a toroidal grid.  The rule string encodes what to do on
 * each cell colour: 'R' = turn right, 'L' = turn left.  After turning,
 * the cell advances to the next colour and the ant steps forward.
 *
 * With rule "RL" (Langton's original):
 *   colour 0 → turn R, flip to colour 1, step
 *   colour 1 → turn L, flip to colour 0, step
 * After ~10 000 steps a diagonal "highway" emerges from chaos.
 *
 * Presets:
 *   RL        Classic Langton — highway after ~10 k steps
 *   LR        Symmetric fractal
 *   LLRR      Growing square spiral
 *   RLR       Chaotic triangle
 *   LRRL      Complex branching
 *   RRLL      Symmetric highway variant
 *   RLLR      Tiling pattern
 *   LLRRR     Irregular growth
 *
 * Multiple ants of the same rule start at spread positions and share
 * the grid — their trails interact, producing emergent structures.
 *
 * Cell colours are mapped to a palette; ants drawn as bold '@'.
 *
 * Keys:
 *   n / p     next / previous preset
 *   1-3       set number of ants (1, 2, or 3)
 *   r         reset grid and ants
 *   + / =     more steps per frame  (up to 2000)
 *   - / _     fewer steps per frame
 *   space     pause / resume
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/langton.c -o langton -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 sim  §5 scene
 *           §6 screen  §7 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS      33333333LL
#define MAX_ROWS     128
#define MAX_COLS     320
#define STEPS_DEF    200
#define STEPS_MAX    2000
#define MAX_ANTS     3
#define MAX_COLORS   8       /* max cell-state colours per rule           */

enum { CP_C0=1, CP_C1, CP_C2, CP_C3, CP_C4, CP_C5, CP_C6, CP_C7,
       CP_ANT, CP_HUD };

/* direction vectors: 0=N 1=E 2=S 3=W */
static const int DR[4] = {-1,  0, 1,  0};
static const int DC[4] = { 0,  1, 0, -1};

#define N_PRESETS 8
static const struct {
    const char *rule;
    const char *name;
} PRESETS[N_PRESETS] = {
    { "RL",    "Langton RL (highway)"  },
    { "LR",    "LR (fractal)"          },
    { "LLRR",  "LLRR (square spiral)"  },
    { "RLR",   "RLR (chaotic)"         },
    { "LRRL",  "LRRL (complex)"        },
    { "RRLL",  "RRLL (symmetric)"      },
    { "RLLR",  "RLLR (tiling)"         },
    { "LLRRR", "LLRRR (irregular)"     },
};

/* colour palettes per number of states (index 2..8) */
static const short PAL256[MAX_COLORS] =
    { -1, 202, 226, 82, 51, 45, 201, 196 };  /* bg, orange, yellow, green, cyan, blue, magenta, red */
static const short PAL8[MAX_COLORS] =
    { -1, COLOR_RED, COLOR_YELLOW, COLOR_GREEN,
      COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA, COLOR_WHITE };

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

static int g_n_colors;   /* = strlen(current rule) */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_ANT, (COLORS >= 256) ? 231 : COLOR_WHITE, -1);
    init_pair(CP_HUD, (COLORS >= 256) ?  82 : COLOR_GREEN, -1);
    /* state 0 = background (space), states 1..N-1 get colour pairs */
    for (int i = 1; i < g_n_colors && i < MAX_COLORS; i++) {
        short fg = (COLORS >= 256) ? PAL256[i] : PAL8[i];
        init_pair(CP_C0 + i, fg, -1);
    }
}

/* ── §4 simulation ───────────────────────────────────────────────────── */

static uint8_t g_grid[MAX_ROWS][MAX_COLS];

static struct Ant { int r, c, dir; } g_ants[MAX_ANTS];
static int g_n_ants;
static int g_rows, g_cols;
static int g_preset;
static const char *g_rule;
static int g_steps, g_paused;
static long long g_total_steps;

static void sim_reset(void)
{
    if (g_rows < 1 || g_cols < 1) return;
    memset(g_grid, 0, sizeof(g_grid));
    g_total_steps = 0;

    /* spread ants evenly */
    static const float ROFF[3] = {0.5f, 0.35f, 0.65f};
    static const float COFF[3] = {0.5f, 0.35f, 0.65f};
    static const int   DIRS[3] = {0, 1, 3};
    for (int i = 0; i < g_n_ants; i++) {
        g_ants[i].r   = (int)(ROFF[i] * g_rows) % g_rows;
        g_ants[i].c   = (int)(COFF[i] * g_cols) % g_cols;
        g_ants[i].dir = DIRS[i % 3];
    }
}

static void set_preset(int idx)
{
    g_preset  = ((idx % N_PRESETS) + N_PRESETS) % N_PRESETS;
    g_rule    = PRESETS[g_preset].rule;
    g_n_colors = (int)strlen(g_rule);
    if (g_n_colors < 2) g_n_colors = 2;
    if (g_n_colors > MAX_COLORS) g_n_colors = MAX_COLORS;
    color_init();
    sim_reset();
}

static void ant_step(struct Ant *ant)
{
    int state = g_grid[ant->r][ant->c];
    char turn  = g_rule[state % g_n_colors];
    /* turn right or left */
    ant->dir = (turn == 'R') ? (ant->dir + 1) % 4
                             : (ant->dir + 3) % 4;
    /* advance cell state */
    g_grid[ant->r][ant->c] = (uint8_t)((state + 1) % g_n_colors);
    /* move forward */
    ant->r = (ant->r + DR[ant->dir] + g_rows) % g_rows;
    ant->c = (ant->c + DC[ant->dir] + g_cols) % g_cols;
}

static void sim_tick(void)
{
    if (g_paused) return;
    for (int s = 0; s < g_steps; s++) {
        for (int i = 0; i < g_n_ants; i++)
            ant_step(&g_ants[i]);
    }
    g_total_steps += g_steps;
}

/* ── §5 scene ────────────────────────────────────────────────────────── */

static void scene_draw(void)
{
    for (int r = 0; r < g_rows - 1; r++) {
        uint8_t *row = g_grid[r];
        for (int c = 0; c < g_cols - 1; c++) {
            int st = row[c];
            if (st == 0) { mvaddch(r, c, ' '); continue; }
            int cp = CP_C0 + (st % MAX_COLORS);
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, (chtype)(unsigned char)'#');
            attroff(COLOR_PAIR(cp));
        }
    }

    /* draw ants on top */
    attron(COLOR_PAIR(CP_ANT) | A_BOLD);
    for (int i = 0; i < g_n_ants; i++) {
        int r = g_ants[i].r, c = g_ants[i].c;
        if (r < g_rows - 1 && c < g_cols - 1)
            mvaddch(r, c, '@');
    }
    attroff(COLOR_PAIR(CP_ANT) | A_BOLD);
}

static void scene_hud(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " %-20s  ants=%d  steps=%lld  spd=%d  %s"
             "  n/p:rule  1-3:ants  r:reset  +/-:spd  q:quit",
             PRESETS[g_preset].name, g_n_ants, g_total_steps, g_steps,
             g_paused ? "PAUSED " : "       ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §6 screen ───────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    g_rows = rows;
    g_cols = cols;
    sim_reset();
    erase();
}

/* ── §7 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    g_steps  = STEPS_DEF;
    g_paused = 0;
    g_n_ants = 1;
    screen_resize();
    set_preset(0);

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0; break;
            case ' ':           g_paused ^= 1; break;
            case 'n':           set_preset(g_preset + 1); break;
            case 'p':           set_preset(g_preset - 1); break;
            case 'r':           sim_reset();              break;
            case '1': g_n_ants = 1; sim_reset(); break;
            case '2': g_n_ants = 2; sim_reset(); break;
            case '3': g_n_ants = 3; sim_reset(); break;
            case '+': case '=':
                g_steps = (g_steps < STEPS_MAX) ? g_steps * 2 : STEPS_MAX;
                break;
            case '-': case '_':
                g_steps = (g_steps > 1) ? g_steps / 2 : 1;
                break;
            }
        }

        sim_tick();
        erase();
        scene_draw();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
