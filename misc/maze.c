/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze.c — Maze Generation + Solve
 *
 * Recursive-backtracker DFS generates the maze wall-by-wall; the advancing
 * frontier is shown in real time.  Once complete, BFS finds the shortest
 * path from top-left to bottom-right and draws it in a contrasting colour.
 *
 * Wall encoding: 4 bits per cell — N=1  E=2  S=4  W=8
 * Display: 2×2 pixels per cell → screen size (2·H+1) × (2·W+1).
 *
 * Keys: q quit  r regenerate  SPACE skip-to-solve  p pause  1/2/3 sizes
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/maze.c \
 *       -o maze -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 maze  §5 solve  §6 draw  §7 app
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

#define MAZE_W_MAX  90
#define MAZE_H_MAX  23
#define HUD_ROWS     2
#define RENDER_NS   (1000000000LL / 30)
#define GEN_STEPS    4   /* DFS steps per frame during generation */
#define SOL_STEPS   16   /* BFS steps per frame during solve      */

#define WALL_N  1
#define WALL_E  2
#define WALL_S  4
#define WALL_W  8

enum Phase { PH_GENERATE, PH_SOLVE, PH_DONE };
enum { CP_WALL=1, CP_CELL, CP_FRONT, CP_PATH, CP_DONE_PATH, CP_HUD };

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
        init_pair(CP_WALL,      232, 232);  /* black on black  */
        init_pair(CP_CELL,      255, 235);  /* white on dark   */
        init_pair(CP_FRONT,     226,  94);  /* yellow on brown */
        init_pair(CP_PATH,       51,  17);  /* cyan on dark blue*/
        init_pair(CP_DONE_PATH, 231,  22);  /* white on green  */
        init_pair(CP_HUD,       244,  -1);
    } else {
        init_pair(CP_WALL,      COLOR_BLACK, COLOR_BLACK);
        init_pair(CP_CELL,      COLOR_WHITE, COLOR_BLACK);
        init_pair(CP_FRONT,     COLOR_YELLOW,COLOR_BLACK);
        init_pair(CP_PATH,      COLOR_CYAN,  COLOR_BLACK);
        init_pair(CP_DONE_PATH, COLOR_GREEN, COLOR_BLACK);
        init_pair(CP_HUD,       COLOR_WHITE, -1);
    }
}

/* ===================================================================== */
/* §4  maze generation (iterative DFS)                                    */
/* ===================================================================== */

static unsigned char g_walls[MAZE_H_MAX][MAZE_W_MAX]; /* wall bitmask   */
static unsigned char g_vis[MAZE_H_MAX][MAZE_W_MAX];   /* visited in DFS */
static int g_mh, g_mw;

/* DFS stack */
static struct { int r, c; } g_dfs[MAZE_H_MAX * MAZE_W_MAX + 1];
static int g_dfs_top;

static enum Phase g_phase;
static bool g_paused;

static const int DR[4] = {-1, 0, 1,  0};
static const int DC[4] = { 0, 1, 0, -1};
/* wall bits for each direction */
static const int D_WALL[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
static const int D_OPP[4]  = { WALL_S, WALL_W, WALL_N, WALL_E };

static void maze_reset(void)
{
    memset(g_walls, 0x0F, sizeof g_walls); /* all walls closed */
    memset(g_vis,   0,    sizeof g_vis);
    g_dfs_top = 0;
    g_dfs[0].r = 0; g_dfs[0].c = 0;
    g_dfs_top = 1;
    g_vis[0][0] = 1;
    g_phase = PH_GENERATE;
}

/* One DFS step: try to advance from the top of stack */
static bool gen_step(void)
{
    if (g_dfs_top == 0) return false; /* done */

    int r = g_dfs[g_dfs_top-1].r;
    int c = g_dfs[g_dfs_top-1].c;

    /* shuffle directions */
    int dirs[4] = {0,1,2,3};
    for (int i = 3; i > 0; i--) {
        int j = rand() % (i+1);
        int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t;
    }

    bool moved = false;
    for (int k = 0; k < 4 && !moved; k++) {
        int d = dirs[k];
        int nr = r + DR[d], nc = c + DC[d];
        if (nr>=0 && nr<g_mh && nc>=0 && nc<g_mw && !g_vis[nr][nc]) {
            g_vis[nr][nc] = 1;
            g_walls[r][c]   &= (unsigned char)~D_WALL[d];   /* open wall */
            g_walls[nr][nc] &= (unsigned char)~D_OPP[d];
            g_dfs[g_dfs_top].r = nr;
            g_dfs[g_dfs_top].c = nc;
            g_dfs_top++;
            moved = true;
        }
    }
    if (!moved) g_dfs_top--;    /* backtrack */
    return true;
}

/* ===================================================================== */
/* §5  BFS solve                                                          */
/* ===================================================================== */

static int g_parent[MAZE_H_MAX][MAZE_W_MAX]; /* encoded: r*MAZE_W_MAX+c */
static unsigned char g_bfs_vis[MAZE_H_MAX][MAZE_W_MAX];
static struct { int r, c; } g_bq[MAZE_H_MAX * MAZE_W_MAX];
static int g_bq_head, g_bq_tail;
static unsigned char g_on_path[MAZE_H_MAX][MAZE_W_MAX];

static void solve_start(void)
{
    memset(g_bfs_vis, 0, sizeof g_bfs_vis);
    memset(g_on_path, 0, sizeof g_on_path);
    memset(g_parent, -1, sizeof g_parent);
    g_bq_head = g_bq_tail = 0;
    g_bq[g_bq_tail].r = 0; g_bq[g_bq_tail].c = 0; g_bq_tail++;
    g_bfs_vis[0][0] = 1;
    g_phase = PH_SOLVE;
}

/* One BFS step */
static bool solve_step(void)
{
    if (g_bq_head >= g_bq_tail) { g_phase = PH_DONE; return false; }
    int r = g_bq[g_bq_head].r;
    int c = g_bq[g_bq_head].c;
    g_bq_head++;

    if (r == g_mh-1 && c == g_mw-1) {
        /* trace back the path */
        int pr = r, pc = c;
        while (pr >= 0 && pc >= 0) {
            g_on_path[pr][pc] = 1;
            int enc = g_parent[pr][pc];
            if (enc < 0) break;
            pr = enc / MAZE_W_MAX;
            pc = enc % MAZE_W_MAX;
        }
        g_phase = PH_DONE;
        return false;
    }

    for (int d = 0; d < 4; d++) {
        if (g_walls[r][c] & D_WALL[d]) continue; /* wall present */
        int nr = r + DR[d], nc = c + DC[d];
        if (nr>=0 && nr<g_mh && nc>=0 && nc<g_mw && !g_bfs_vis[nr][nc]) {
            g_bfs_vis[nr][nc] = 1;
            g_parent[nr][nc]  = r * MAZE_W_MAX + c;
            g_bq[g_bq_tail].r = nr; g_bq[g_bq_tail].c = nc; g_bq_tail++;
        }
    }
    return true;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int g_rows, g_cols;

/*
 * Render the maze into the terminal.
 * Pixel grid: (2*g_mh+1) rows × (2*g_mw+1) cols, offset by HUD_ROWS.
 *
 * Cell (r,c) → pixel (2r+1+HUD, 2c+1).
 * Horizontal wall between (r,c) and (r+1,c) → pixel (2r+2+HUD, 2c+1).
 * Vertical wall between (r,c) and (r,c+1) → pixel (2r+1+HUD, 2c+2).
 * Corners → pixel (2r+HUD, 2c).
 */
static void scene_draw(void)
{
    /* draw wall grid */
    for (int pr = 0; pr <= 2*g_mh && pr+HUD_ROWS < g_rows; pr++) {
        for (int pc = 0; pc <= 2*g_mw && pc < g_cols; pc++) {
            int sr = pr + HUD_ROWS;
            bool is_corner = !(pr&1) && !(pc&1);
            bool is_hwall  = !(pr&1) &&  (pc&1);
            bool is_vwall  =  (pr&1) && !(pc&1);
            bool is_cell   =  (pr&1) &&  (pc&1);

            chtype ch; int cp;
            if (is_corner) {
                ch = '+'; cp = CP_WALL;
            } else if (is_hwall) {
                int r = pr/2 - 1, c = pc/2;
                bool wall_open = (r<0) ? false : !(g_walls[r][c] & WALL_S);
                ch = wall_open ? ' ' : '-';
                cp = CP_WALL;
            } else if (is_vwall) {
                int r = pr/2, c = pc/2 - 1;
                bool wall_open = (c<0) ? false : !(g_walls[r][c] & WALL_E);
                ch = wall_open ? ' ' : '|';
                cp = CP_WALL;
            } else if (is_cell) {
                int r = pr/2, c = pc/2;
                if (g_phase == PH_GENERATE) {
                    if (g_dfs_top > 0 &&
                        g_dfs[g_dfs_top-1].r == r &&
                        g_dfs[g_dfs_top-1].c == c)
                    { ch = '@'; cp = CP_FRONT; }
                    else if (g_vis[r][c]) { ch = ' '; cp = CP_CELL; }
                    else                  { ch = '#'; cp = CP_WALL; }
                } else {
                    if (g_on_path[r][c])      { ch = '*'; cp = CP_DONE_PATH; }
                    else if (g_bfs_vis[r][c]) { ch = '.'; cp = CP_PATH; }
                    else if (g_vis[r][c])     { ch = ' '; cp = CP_CELL; }
                    else                      { ch = '#'; cp = CP_WALL; }
                }
            } else {
                ch = ' '; cp = 0;
            }

            if (cp) {
                attron(COLOR_PAIR(cp));
                mvaddch(sr, pc, ch);
                attroff(COLOR_PAIR(cp));
            } else {
                mvaddch(sr, pc, ch);
            }
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Maze  q:quit  r:regen  spc:skip-to-solve  p:pause  1/2/3:size");
    const char *phase_str = (g_phase == PH_GENERATE) ? "generating" :
                            (g_phase == PH_SOLVE) ? "solving (BFS)" : "done";
    mvprintw(1, 0, " %dx%d  %s  %s",
        g_mw, g_mh, phase_str, g_paused ? "[PAUSED]" : "");
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

static void calc_dims(int rows, int cols, int *mh, int *mw)
{
    *mh = (rows - HUD_ROWS - 1) / 2;
    *mw = (cols - 1) / 2;
    if (*mh > MAZE_H_MAX) *mh = MAZE_H_MAX;
    if (*mw > MAZE_W_MAX) *mw = MAZE_W_MAX;
    if (*mh < 2) *mh = 2;
    if (*mw < 2) *mw = 2;
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
    getmaxyx(stdscr, g_rows, g_cols);
    calc_dims(g_rows, g_cols, &g_mh, &g_mw);
    maze_reset();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            calc_dims(g_rows, g_cols, &g_mh, &g_mw);
            maze_reset();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'r': case 'R': maze_reset(); break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case ' ':
            if (g_phase == PH_GENERATE) {
                while (gen_step()) {}
                solve_start();
            } else if (g_phase == PH_SOLVE) {
                while (solve_step()) {}
                g_phase = PH_DONE;
            }
            break;
        case '1': g_mh = 10; g_mw = 40; maze_reset(); break;
        case '2': calc_dims(g_rows, g_cols, &g_mh, &g_mw); maze_reset(); break;
        case '3':
            g_mh = MAZE_H_MAX; g_mw = MAZE_W_MAX; maze_reset(); break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused) {
            if (g_phase == PH_GENERATE) {
                for (int s = 0; s < GEN_STEPS; s++)
                    if (!gen_step()) { solve_start(); break; }
            } else if (g_phase == PH_SOLVE) {
                for (int s = 0; s < SOL_STEPS; s++)
                    if (!solve_step()) break;
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
