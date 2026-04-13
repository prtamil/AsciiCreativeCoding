/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sandpile.c — Bak-Tang-Wiesenfeld Abelian Sandpile
 *
 * Grains are dropped one at a time at the centre of the grid.
 * Any cell with ≥ 4 grains topples: it loses 4 grains and each of its four
 * cardinal neighbours gains 1.  Grains that fall off the border are lost.
 * Toppling cascades until every cell holds 0–3 grains (the stable state).
 *
 * The emergent pattern exhibits self-organised criticality:
 *   — avalanche sizes follow a power-law (1/f) distribution
 *   — most grain drops trigger tiny, local topplings
 *   — occasionally a single grain tips a cascade across the entire pile
 *   — the long-run stable pattern has intricate fractal / quasi-crystalline
 *     symmetry that was not designed in — it arises purely from the rule
 *
 * Colour by grain count (stable state):
 *   0 grains — blank              (background)
 *   1 grain  — dim blue      '.'  (sparse)
 *   2 grains — green         '+'  (medium)
 *   3 grains — bright gold   '#'  (near-critical)
 *
 * Avalanche cells (≥ 4) are shown in bright red during the animated
 * avalanche mode ('v' key) so you can watch the wave propagate.
 *
 * Keys:
 *   r         reset (clear grid, restart drop counter)
 *   space     pause / resume
 *   v         toggle avalanche visualisation (slow per-step vs instant)
 *   + / =     more drops per frame
 *   - / _     fewer drops per frame
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/sandpile.c -o sandpile -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 grid  §5 sim
 *           §6 scene   §7 screen §8 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL   /* ~30 fps                               */
#define MAX_ROWS      128
#define MAX_COLS      320
#define QMAX          (MAX_ROWS * MAX_COLS + 1)  /* avalanche queue size   */

#define DROPS_DEF     10           /* grain drops per frame (default)       */
#define DROPS_MAX     500          /* maximum drops per frame               */

/* characters for each grain count */
static const char GRAIN_CH[4] = { ' ', '.', '+', '#' };

enum { CP_G1=1, CP_G2, CP_G3, CP_TOPPLE, CP_HUD };

/* ── §2 clock ───────────────────────────────────────────────────────────── */

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

/* ── §3 color ───────────────────────────────────────────────────────────── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_G1,     18,  -1);  /* dark blue   — 1 grain            */
        init_pair(CP_G2,     34,  -1);  /* green       — 2 grains           */
        init_pair(CP_G3,     220, -1);  /* gold        — 3 grains           */
        init_pair(CP_TOPPLE, 196, -1);  /* bright red  — toppling (≥4)      */
        init_pair(CP_HUD,    82,  -1);  /* green HUD                        */
    } else {
        init_pair(CP_G1,     COLOR_BLUE,    -1);
        init_pair(CP_G2,     COLOR_GREEN,   -1);
        init_pair(CP_G3,     COLOR_YELLOW,  -1);
        init_pair(CP_TOPPLE, COLOR_RED,     -1);
        init_pair(CP_HUD,    COLOR_GREEN,   -1);
    }
}

/* ── §4 grid ────────────────────────────────────────────────────────────── */

static uint8_t g_grid[MAX_ROWS][MAX_COLS];
static int     g_rows, g_cols, g_ca_rows;
static int     g_drops, g_paused, g_vis;  /* g_vis: show avalanche steps   */
static long long g_total_drops;
static long      g_last_avalanche;

/* avalanche BFS queue */
static int     g_qr   [QMAX];
static int     g_qc   [QMAX];
static uint8_t g_inq  [MAX_ROWS][MAX_COLS];
static int     g_qhead, g_qtail;

/* drop-point (centre) */
static int g_cr, g_cc;

static void grid_clear(void)
{
    memset(g_grid, 0, sizeof(g_grid));
    g_total_drops   = 0;
    g_last_avalanche = 0;
    g_cr = g_ca_rows / 2;
    g_cc = g_cols    / 2;
}

/* ── §5 simulation ──────────────────────────────────────────────────────── */

static void enq(int r, int c)
{
    if (g_inq[r][c]) return;
    g_inq[r][c] = 1;
    g_qr[g_qtail] = r;
    g_qc[g_qtail] = c;
    g_qtail = (g_qtail + 1) % QMAX;
}

/* Run the avalanche to completion.
 * Returns the number of topple events.
 * If g_vis is set, returns after one "wave" (one pass through the current
 * queue front) so the caller can render between waves.                       */
static long avalanche_step(int full)
{
    long topples = 0;

    while (g_qhead != g_qtail) {
        int r = g_qr[g_qhead];
        int c = g_qc[g_qhead];
        g_qhead = (g_qhead + 1) % QMAX;
        g_inq[r][c] = 0;

        if (g_grid[r][c] < 4) continue;  /* stabilised since enqueue        */

        g_grid[r][c] -= 4;
        topples++;

        static const int DR[4] = {-1,  0, 1, 0};
        static const int DC[4] = { 0,  1, 0,-1};
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (nr >= 0 && nr < g_ca_rows && nc >= 0 && nc < g_cols) {
                g_grid[nr][nc]++;
                if (g_grid[nr][nc] >= 4) enq(nr, nc);
            }
        }
        if (g_grid[r][c] >= 4) enq(r, c);   /* still unstable: re-enqueue  */

        if (!full) break;   /* one topple per visual frame                  */
    }
    return topples;
}

static int avalanche_pending(void) { return g_qhead != g_qtail; }

static void drop_grain(void)
{
    g_grid[g_cr][g_cc]++;
    g_total_drops++;
    if (g_grid[g_cr][g_cc] >= 4) {
        memset(g_inq, 0, sizeof(g_inq));
        g_qhead = g_qtail = 0;
        enq(g_cr, g_cc);
    }
}

static void sim_tick(void)
{
    if (g_paused) return;

    if (g_vis) {
        /* animated mode: one topple per frame, drop new grain when stable   */
        if (avalanche_pending()) {
            g_last_avalanche += avalanche_step(0);
        } else {
            drop_grain();
            g_last_avalanche = 0;
        }
    } else {
        /* instant mode: drop g_drops grains, run each to full stability     */
        for (int i = 0; i < g_drops; i++) {
            drop_grain();
            if (avalanche_pending())
                g_last_avalanche = avalanche_step(1);
        }
    }
}

/* ── §6 scene ───────────────────────────────────────────────────────────── */

static void scene_grid(void)
{
    for (int r = 0; r < g_ca_rows; r++) {
        for (int c = 0; c < g_cols - 1; c++) {
            int g = g_grid[r][c];
            if (g == 0) { mvaddch(r, c, ' '); continue; }

            int cp;
            char ch;
            attr_t bold = 0;
            if (g >= 4) {
                cp   = CP_TOPPLE;
                ch   = '*';
                bold = A_BOLD;
            } else {
                cp   = CP_G1 + g - 1;   /* CP_G1=1 grain … CP_G3=3 grains  */
                ch   = GRAIN_CH[g];
                bold = (g == 3) ? A_BOLD : 0;
            }
            attron(COLOR_PAIR(cp) | bold);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* mark drop point */
    attron(COLOR_PAIR(CP_TOPPLE) | A_BOLD);
    if (g_grid[g_cr][g_cc] == 0)
        mvaddch(g_cr, g_cc, (chtype)(unsigned char)'@');
    attroff(COLOR_PAIR(CP_TOPPLE) | A_BOLD);
}

static void scene_hud(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " grains=%-8lld  last_avalanche=%-7ld  rate=%d/f  %s  %s"
             "  r:reset  +/-:rate  v:vis  spc:pause  q:quit",
             (long long)g_total_drops, g_last_avalanche, g_drops,
             g_vis    ? "VIS " : "    ",
             g_paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §7 screen ──────────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    g_rows    = rows;
    g_cols    = cols;
    g_ca_rows = rows - 1;   /* full screen minus HUD row                    */
    if (g_ca_rows < 1) g_ca_rows = 1;
    grid_clear();
    erase();
}

/* ── §8 app ─────────────────────────────────────────────────────────────── */

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
    g_drops  = DROPS_DEF;
    g_paused = 0;
    g_vis    = 0;
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize();

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
            case 'q': case 'Q': g_running = 0;               break;
            case ' ':           g_paused ^= 1;                break;
            case 'r':           grid_clear(); erase();        break;
            case 'v': case 'V': g_vis ^= 1;                  break;
            case '+': case '=':
                if (g_drops < DROPS_MAX) g_drops++;
                break;
            case '-': case '_':
                if (g_drops > 1) g_drops--;
                break;
            }
        }

        sim_tick();
        erase();
        scene_grid();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
