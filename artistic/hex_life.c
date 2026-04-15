/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hex_life.c — Hexagonal Game of Life
 *
 * Game of Life on a hexagonal grid using offset-rows layout.
 * Each cell has 6 neighbours.  Rule B2/S34:
 *   Born    if dead  and has exactly 2 live neighbours.
 *   Survive if alive and has 3 or 4 live neighbours.
 *
 * Hex offset-rows: odd rows are shifted right by one terminal column.
 * For even row r, neighbours of (r,c): (r-1,c-1),(r-1,c),(r,c-1),(r,c+1),(r+1,c-1),(r+1,c)
 * For odd  row r, neighbours of (r,c): (r-1,c),(r-1,c+1),(r,c-1),(r,c+1),(r+1,c),(r+1,c+1)
 *
 * Keys: q quit  SPACE seed random  p pause  r reset  1-5 preset patterns
 *       +/- sim speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/hex_life.c \
 *       -o hex_life -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 grid  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Conway's Game of Life adapted for a hexagonal grid.
 *                  Each cell has 6 neighbours (vs 8 for square grids).
 *                  Rule B2/S34: born on 2 live neighbours, survives on 3–4.
 *                  Uses double-buffering: write next generation to the
 *                  inactive buffer, then swap — O(rows × cols) per step.
 *
 * Data-structure : Flat 2-D array with uint8_t age field.  Offset-rows
 *                  layout: odd rows shifted right by 0.5 cells visually.
 *                  Neighbour coordinates differ for even vs odd rows (two
 *                  lookup tables, one per parity).
 *
 * Physics        : Hexagonal CA rules produce qualitatively different
 *                  behaviour than square-grid Life: the B2/S34 rule creates
 *                  stable oscillators and gliders unique to the hex topology.
 *                  6-neighbour symmetry eliminates diagonal artefacts present
 *                  in square-grid CAs (more isotropic diffusion of patterns).
 *
 * Rendering      : Age-based colour: newborn → yellow, young → cyan,
 *                  mature → teal, elder → dark.  Dead cells shown as dim
 *                  dots to visualise the hex grid structure.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
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

#define STEPS_PER_FRAME  1
#define RENDER_NS        (1000000000LL / 15)   /* 15 fps for visibility */

enum {
    CP_DEAD = 1,   /* dead   — dark dot, outlines hex grid */
    CP_AGE0,       /* newborn (age 0)  — bright yellow     */
    CP_AGE1,       /* young  (age 1-4) — bright cyan       */
    CP_AGE2,       /* mature (age 5-14)— teal              */
    CP_AGE3,       /* elder  (age 15+) — dark blue-green   */
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
        init_pair(CP_DEAD, 234,   -1);   /* near-black dot, no background */
        init_pair(CP_AGE0, 226,   -1);   /* bright yellow  — newborn   */
        init_pair(CP_AGE1,  51,   -1);   /* bright cyan    — young     */
        init_pair(CP_AGE2,  37,   -1);   /* medium teal    — mature    */
        init_pair(CP_AGE3,  23,   -1);   /* dark blue-green — elder    */
        init_pair(CP_HUD,  232,  250);   /* dark text on silver bar    */
    } else {
        init_pair(CP_DEAD, COLOR_BLACK,  -1);
        init_pair(CP_AGE0, COLOR_YELLOW, -1);
        init_pair(CP_AGE1, COLOR_CYAN,   -1);
        init_pair(CP_AGE2, COLOR_CYAN,   -1);
        init_pair(CP_AGE3, COLOR_BLUE,   -1);
        init_pair(CP_HUD,  COLOR_BLACK,  COLOR_WHITE);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Each cell stores: 0=dead, 1=alive, 2=just-born (for coloring)
 * We use two buffers and swap.
 */
static signed char g_grid[GRID_H_MAX][GRID_W_MAX];
static signed char g_next[GRID_H_MAX][GRID_W_MAX];
static unsigned char g_age [GRID_H_MAX][GRID_W_MAX]; /* generations alive */
static unsigned char g_anext[GRID_H_MAX][GRID_W_MAX];
static int g_gh, g_gw;
static long long g_gen = 0;

/* B2/S34 hex neighbour offsets (even row, odd row) */
static const int HEX_DR[6]    = { -1, -1,  0, 0,  1,  1 };
static const int HEX_DC_EVEN[6] = { -1,  0, -1, 1, -1,  0 };
static const int HEX_DC_ODD[6]  = {  0,  1, -1, 1,  0,  1 };

static int hex_count(int r, int c)
{
    const int *dc = (r & 1) ? HEX_DC_ODD : HEX_DC_EVEN;
    int cnt = 0;
    for (int k = 0; k < 6; k++) {
        int nr = r + HEX_DR[k];
        int nc = c + dc[k];
        if (nr >= 0 && nr < g_gh && nc >= 0 && nc < g_gw)
            cnt += (g_grid[nr][nc] != 0) ? 1 : 0;
    }
    return cnt;
}

static void grid_step(void)
{
    for (int r = 0; r < g_gh; r++) {
        for (int c = 0; c < g_gw; c++) {
            int n = hex_count(r, c);
            int alive = (g_grid[r][c] != 0);
            if (alive && (n == 3 || n == 4)) {
                g_next[r][c]  = 1;
                /* cap age at 255 to avoid overflow */
                g_anext[r][c] = g_age[r][c] < 255 ? g_age[r][c] + 1 : 255;
            } else if (!alive && n == 2) {
                g_next[r][c]  = 1;
                g_anext[r][c] = 0;   /* newborn */
            } else {
                g_next[r][c]  = 0;
                g_anext[r][c] = 0;
            }
        }
    }
    g_gen++;
    memcpy(g_grid, g_next,  sizeof g_grid);
    memcpy(g_age,  g_anext, sizeof g_age);
}

static void grid_random(int density_pct)
{
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            g_grid[r][c] = (rand() % 100 < density_pct) ? 1 : 0;
    memset(g_age, 0, sizeof g_age);
    g_gen = 0;
}

static void grid_clear(void)
{
    memset(g_grid, 0, sizeof g_grid);
    memset(g_age,  0, sizeof g_age);
    g_gen = 0;
}

/* place a small seed pattern at center */
static void grid_seed(void)
{
    grid_clear();
    int cr = g_gh / 2, cc = g_gw / 2;
    /* small dense cluster */
    for (int dr = -3; dr <= 3; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = cr+dr, c = cc+dc;
            if (r>=0&&r<g_gh&&c>=0&&c<g_gw)
                g_grid[r][c] = rand()%2;
        }
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static int  g_speed = STEPS_PER_FRAME;

static long long g_live = 0;

static void count_live(void)
{
    g_live = 0;
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            if (g_grid[r][c]) g_live++;
}

/* Each hex cell is 2 terminal columns wide.
 * Even rows: cell c starts at col c*2.
 * Odd  rows: cell c starts at col c*2 + 1.
 * This 1-column stagger creates proper hexagonal tiling.
 *
 * Age → character + colour:
 *   age 0  (newborn) : <> bright yellow
 *   age 1–4 (young)  : () bright cyan
 *   age 5–14 (mature): [] teal
 *   age 15+ (elder)  : -- dim blue-green
 *   dead             : .  dark dot — outlines the hex lattice
 */
static void scene_draw(void)
{
    for (int r = 0; r < g_gh && r + HUD_ROWS < g_rows; r++) {
        int offset = r & 1;   /* odd rows shift right 1 col */
        for (int c = 0; c < g_gw; c++) {
            int sc = c * 2 + offset;
            int sr = r + HUD_ROWS;
            if (sc + 1 >= g_cols) break;

            signed char alive = g_grid[r][c];
            if (!alive) {
                /* dead: dim dot marks the hex vertex — shows grid structure */
                attron(COLOR_PAIR(CP_DEAD) | A_DIM);
                mvaddch(sr, sc,   '.');
                mvaddch(sr, sc+1, ' ');
                attroff(COLOR_PAIR(CP_DEAD) | A_DIM);
            } else {
                unsigned char age = g_age[r][c];
                int    cp;
                attr_t at;
                char   l, ri;
                if (age == 0)       { cp = CP_AGE0; at = A_BOLD; l = '<'; ri = '>'; }
                else if (age < 5)   { cp = CP_AGE1; at = A_BOLD; l = '('; ri = ')'; }
                else if (age < 15)  { cp = CP_AGE2; at = 0;      l = '['; ri = ']'; }
                else                { cp = CP_AGE3; at = A_DIM;  l = '-'; ri = '-'; }
                attron(COLOR_PAIR(cp) | at);
                mvaddch(sr, sc,   (chtype)(unsigned char)l);
                mvaddch(sr, sc+1, (chtype)(unsigned char)ri);
                attroff(COLOR_PAIR(cp) | at);
            }
        }
    }

    /* HUD — silver bar at top */
    attron(COLOR_PAIR(CP_HUD));
    for (int c = 0; c < g_cols; c++) mvaddch(0, c, ' ');
    mvprintw(0, 1,
        "HexLife B2/S34  q:quit  spc:random  p:pause  r:reset  1-5:density  +/-:speed"
        "  gen:%lld  live:%lld  %s",
        (long long)g_gen, g_live, g_paused ? "[PAUSED]" : "");
    for (int c = 0; c < g_cols; c++) mvaddch(1, c, ' ');
    mvprintw(1, 1,
        "<> newborn   () young   [] mature   -- elder   .  dead");
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
    g_gw = ((cols - 2) / 2) < GRID_W_MAX ? ((cols - 2) / 2) : GRID_W_MAX;

    grid_random(35);

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
            g_gw = ((cols - 2) / 2) < GRID_W_MAX ? ((cols - 2) / 2) : GRID_W_MAX;
            grid_random(35);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': grid_random(35); break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': grid_clear(); break;
        case 's': case 'S': grid_seed(); break;
        case '1': grid_random(20); break;
        case '2': grid_random(35); break;
        case '3': grid_random(50); break;
        case '4': grid_random(65); break;
        case '5': grid_random(80); break;
        case '+': case '=': g_speed++; if (g_speed > 10) g_speed = 10; break;
        case '-': g_speed--; if (g_speed < 1) g_speed = 1; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused)
            for (int s = 0; s < g_speed; s++) grid_step();

        count_live();
        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
