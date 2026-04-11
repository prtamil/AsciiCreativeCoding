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
    CP_DEAD = 1,   /* dead cell — dark bg       */
    CP_LIVE,       /* live cell — cyan           */
    CP_NEW,        /* just born — bright white   */
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
        init_pair(CP_DEAD, 236, 236);   /* dark grey on dark grey */
        init_pair(CP_LIVE,  51, -1);    /* cyan                   */
        init_pair(CP_NEW,  231, -1);    /* bright white           */
        init_pair(CP_HUD,  244, -1);
    } else {
        init_pair(CP_DEAD, COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_LIVE, COLOR_CYAN,  -1);
        init_pair(CP_NEW,  COLOR_WHITE, -1);
        init_pair(CP_HUD,  COLOR_WHITE, -1);
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
            if (alive)
                g_next[r][c] = (n == 3 || n == 4) ? 1 : 0;
            else
                g_next[r][c] = (n == 2) ? 2 : 0;   /* 2 = newborn */
        }
    }
    g_gen++;
    memcpy(g_grid, g_next, sizeof g_grid);
}

static void grid_random(int density_pct)
{
    for (int r = 0; r < g_gh; r++)
        for (int c = 0; c < g_gw; c++)
            g_grid[r][c] = (rand() % 100 < density_pct) ? 1 : 0;
    g_gen = 0;
}

static void grid_clear(void)
{
    memset(g_grid, 0, sizeof g_grid);
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

static void scene_draw(void)
{
    for (int r = 0; r < g_gh && r + HUD_ROWS < g_rows; r++) {
        int offset = r & 1;   /* odd rows shifted right by 1 */
        for (int c = 0; c < g_gw; c++) {
            int sc = c + offset;
            int sr = r + HUD_ROWS;
            if (sc >= g_cols) continue;

            signed char v = g_grid[r][c];
            int cp; chtype ch;
            if (v == 0)      { cp = CP_DEAD; ch = '.'; }
            else if (v == 2) { cp = CP_NEW;  ch = '#'; }
            else             { cp = CP_LIVE; ch = 'o'; }

            attron(COLOR_PAIR(cp) | (v == 2 ? A_BOLD : 0));
            mvaddch(sr, sc, ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " HexLife B2/S34  q:quit  spc:random  p:pause  r:reset  1-5:presets  +/-:speed");
    mvprintw(1, 0,
        " gen:%lld  live:%lld  speed:%dx  %s",
        (long long)g_gen, g_live, g_speed,
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
    g_gw = (cols - 1) < GRID_W_MAX ? (cols - 1) : GRID_W_MAX;

    grid_random(35);

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
            g_gw = (cols - 1) < GRID_W_MAX ? (cols - 1) : GRID_W_MAX;
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
