/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ant_colony.c — Pheromone Stigmergy / Ant Colony Optimisation
 *
 * 50 ants wander a terminal grid, deposit pheromone, and collectively
 * discover shortest paths between the nest and two food sources.
 *
 * Rules
 * -----
 *   Searching: move in 8 directions, biased by (a) pheromone at candidate
 *              cells and (b) a gentle pull toward the nearest food source.
 *              Deposit a small pheromone trail.
 *   Returning: bee-line toward the nest (max-dot-product direction), deposit
 *              a heavy pheromone trail so followers can find the path.
 *   At food:   flip to RETURNING state.
 *   At nest:   increment food count, flip to SEARCHING.
 *
 * Pheromone evaporates by EVAP_RATE each tick.  Short paths get reinforced
 * faster → classical ACO positive feedback → shortest-path emergence.
 *
 * Keys:  q quit   spc pause   r reset   + - speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/ant_colony.c \
 *       -o ant_colony -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 pheromone grid  §5 ants
 * §6 scene   §7 screen §8 app
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
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_ANTS         50
#define EVAP_RATE      0.003f   /* pheromone lost per tick (multiplicative)  */
#define DEPOSIT_RETURN 0.40f    /* pheromone deposited by returning ant       */
#define DEPOSIT_SEARCH 0.04f    /* pheromone deposited by searching ant       */
#define FOOD_BIAS      3.0f     /* weight of food-direction pull vs pheromone */
#define PH_MAX         1.0f     /* pheromone concentration ceiling            */
#define FOOD_RADIUS    2        /* Manhattan radius to detect food/nest       */

#define GRID_W_MAX     320
#define GRID_H_MAX     100
#define HUD_ROWS         3

#define RENDER_NS     (1000000000LL / 30)

enum {
    CP_PH1 = 1, CP_PH2, CP_PH3, CP_PH4,   /* pheromone intensities */
    CP_ANT_S,   /* searching ant — yellow                            */
    CP_ANT_R,   /* returning ant — cyan                              */
    CP_FOOD,    /* food source   — green bold                        */
    CP_NEST,    /* nest          — magenta bold                      */
    CP_HUD,
};

/* 8-connected direction offsets (N NE E SE S SW W NW) */
static const int DDX[8] = { 0, 1, 1, 1, 0,-1,-1,-1 };
static const int DDY[8] = {-1,-1, 0, 1, 1, 1, 0,-1 };

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
        init_pair(CP_PH1,  17, -1);    /* very dark blue   */
        init_pair(CP_PH2,  21, -1);    /* blue             */
        init_pair(CP_PH3,  33, -1);    /* light blue       */
        init_pair(CP_PH4,  51, -1);    /* cyan             */
        init_pair(CP_ANT_S, 226, -1);  /* yellow           */
        init_pair(CP_ANT_R,  87, -1);  /* aqua             */
        init_pair(CP_FOOD,   46, -1);  /* green            */
        init_pair(CP_NEST,  201, -1);  /* magenta          */
        init_pair(CP_HUD,   244, -1);  /* grey             */
    } else {
        init_pair(CP_PH1,  COLOR_BLUE,    -1);
        init_pair(CP_PH2,  COLOR_BLUE,    -1);
        init_pair(CP_PH3,  COLOR_CYAN,    -1);
        init_pair(CP_PH4,  COLOR_CYAN,    -1);
        init_pair(CP_ANT_S, COLOR_YELLOW, -1);
        init_pair(CP_ANT_R, COLOR_CYAN,   -1);
        init_pair(CP_FOOD,  COLOR_GREEN,  -1);
        init_pair(CP_NEST,  COLOR_MAGENTA,-1);
        init_pair(CP_HUD,   COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  pheromone grid                                                     */
/* ===================================================================== */

static float g_ph[GRID_H_MAX][GRID_W_MAX];
static int   g_grid_h, g_grid_w;

static void ph_init(void)
{
    for (int r = 0; r < g_grid_h; r++)
        for (int c = 0; c < g_grid_w; c++)
            g_ph[r][c] = 0.f;
}

static void ph_evaporate(void)
{
    float decay = 1.f - EVAP_RATE;
    for (int r = 0; r < g_grid_h; r++)
        for (int c = 0; c < g_grid_w; c++) {
            g_ph[r][c] *= decay;
            if (g_ph[r][c] < 0.001f) g_ph[r][c] = 0.f;
        }
}

static void ph_deposit(int row, int col, float amount)
{
    if (row < 0 || row >= g_grid_h || col < 0 || col >= g_grid_w) return;
    g_ph[row][col] += amount;
    if (g_ph[row][col] > PH_MAX) g_ph[row][col] = PH_MAX;
}

static float ph_at(int row, int col)
{
    if (row < 0 || row >= g_grid_h || col < 0 || col >= g_grid_w) return 0.f;
    return g_ph[row][col];
}

/* ===================================================================== */
/* §5  ants                                                               */
/* ===================================================================== */

typedef enum { SEARCHING, RETURNING } AntState;

typedef struct {
    int       row, col;
    int       dir;        /* current facing direction 0-7 */
    AntState  state;
} Ant;

static Ant g_ants[N_ANTS];
static int g_nest_row, g_nest_col;
static int g_food_row[2], g_food_col[2];
static int g_food_collected = 0;

static bool in_range(int r, int c, int tr, int tc, int radius)
{
    int dr = r - tr, dc = c - tc;
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc) <= radius;
}

/*
 * best_dir_toward() — direction (0-7) that most closely points from
 *                     (sr,sc) toward (tr,tc).
 */
static int best_dir_toward(int sr, int sc, int tr, int tc)
{
    float dx = (float)(tc - sc), dy = (float)(tr - sr);
    float best_dot = -1e9f;
    int   best = 0;
    for (int d = 0; d < 8; d++) {
        float dot = (float)DDX[d]*dx + (float)DDY[d]*dy;
        if (dot > best_dot) { best_dot = dot; best = d; }
    }
    return best;
}

static void ant_init(int i)
{
    g_ants[i].row   = g_nest_row + (rand() % 3 - 1);
    g_ants[i].col   = g_nest_col + (rand() % 3 - 1);
    g_ants[i].dir   = rand() % 8;
    g_ants[i].state = SEARCHING;
}

static void ant_tick(Ant *a)
{
    int r = a->row, c = a->col;

    if (a->state == SEARCHING) {
        ph_deposit(r, c, DEPOSIT_SEARCH);

        /* Candidate directions: forward arc (dir-1, dir, dir+1) */
        int cands[3] = {
            (a->dir + 7) % 8,
            a->dir,
            (a->dir + 1) % 8
        };

        /* Find nearest food source for directional bias */
        float best_fd = 1e9f;
        int   fi = 0;
        for (int k = 0; k < 2; k++) {
            float dx = (float)(g_food_col[k] - c);
            float dy = (float)(g_food_row[k] - r);
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < best_fd) { best_fd = d; fi = k; }
        }

        /* Weight each candidate by pheromone + food-direction bias */
        float weights[3];
        float total = 0.f;
        for (int k = 0; k < 3; k++) {
            int d   = cands[k];
            int nr  = r + DDY[d];
            int nc  = c + DDX[d];
            float ph = ph_at(nr, nc) + 0.05f;   /* base noise */

            /* food bias: dot product with direction toward food */
            float fdx = (float)(g_food_col[fi] - c);
            float fdy = (float)(g_food_row[fi] - r);
            float fn  = sqrtf(fdx*fdx + fdy*fdy);
            if (fn > 0.f) { fdx /= fn; fdy /= fn; }
            float bias = FOOD_BIAS * ((float)DDX[d]*fdx + (float)DDY[d]*fdy + 1.f);
            weights[k] = ph * bias;
            if (weights[k] < 0.f) weights[k] = 0.f;
            total += weights[k];
        }

        /* Probabilistic selection */
        int chosen = 1;   /* default: forward */
        if (total > 0.f) {
            float rnd = (float)rand() / (float)RAND_MAX * total;
            float acc = 0.f;
            for (int k = 0; k < 3; k++) {
                acc += weights[k];
                if (rnd <= acc) { chosen = k; break; }
            }
        }
        int nd = cands[chosen];
        int nr = r + DDY[nd], nc = c + DDX[nd];

        /* Bounce off walls */
        if (nr < HUD_ROWS || nr >= g_grid_h || nc < 0 || nc >= g_grid_w) {
            nd = (nd + 4) % 8;   /* reverse */
            nr = r + DDY[nd]; nc = c + DDX[nd];
            if (nr < HUD_ROWS || nr >= g_grid_h || nc < 0 || nc >= g_grid_w) {
                nr = r; nc = c;
            }
        }
        a->dir = nd; a->row = nr; a->col = nc;

        /* Detect food */
        for (int k = 0; k < 2; k++) {
            if (in_range(a->row, a->col, g_food_row[k], g_food_col[k], FOOD_RADIUS)) {
                a->state = RETURNING;
                a->dir   = best_dir_toward(a->row, a->col, g_nest_row, g_nest_col);
                return;
            }
        }

    } else { /* RETURNING */
        ph_deposit(r, c, DEPOSIT_RETURN);

        /* Move toward nest with slight random deviation */
        int ideal = best_dir_toward(r, c, g_nest_row, g_nest_col);
        int offset = (rand() % 3) - 1;   /* -1, 0, or +1 deviation */
        int nd = (ideal + offset + 8) % 8;
        int nr = r + DDY[nd], nc = c + DDX[nd];

        if (nr < HUD_ROWS || nr >= g_grid_h || nc < 0 || nc >= g_grid_w) {
            nd = ideal; nr = r + DDY[nd]; nc = c + DDX[nd];
        }
        a->dir = nd; a->row = nr; a->col = nc;

        /* Detect nest */
        if (in_range(a->row, a->col, g_nest_row, g_nest_col, FOOD_RADIUS)) {
            g_food_collected++;
            a->state = SEARCHING;
            a->dir   = rand() % 8;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

static bool g_paused = false;
static int  g_speed  = 1;   /* ticks per frame */

static void scene_init(int rows, int cols)
{
    g_grid_h = rows < GRID_H_MAX ? rows : GRID_H_MAX;
    g_grid_w = cols < GRID_W_MAX ? cols : GRID_W_MAX;

    g_nest_row = rows / 2;
    g_nest_col = cols / 2;

    int margin = cols / 5;
    g_food_row[0] = rows / 3;      g_food_col[0] = margin;
    g_food_row[1] = rows * 2 / 3;  g_food_col[1] = cols - margin;

    g_food_collected = 0;
    ph_init();

    for (int i = 0; i < N_ANTS; i++) ant_init(i);
}

static void scene_tick(void)
{
    if (g_paused) return;
    for (int i = 0; i < N_ANTS; i++) ant_tick(&g_ants[i]);
    ph_evaporate();
}

static void scene_draw(int rows, int cols)
{
    /* ── pheromone layer ─────────────────────────────────────────── */
    for (int r = HUD_ROWS; r < g_grid_h && r < rows; r++) {
        for (int c = 0; c < g_grid_w && c < cols; c++) {
            float p = g_ph[r][c];
            if (p < 0.04f) continue;
            int   cp = p < 0.15f ? CP_PH1 :
                       p < 0.40f ? CP_PH2 :
                       p < 0.70f ? CP_PH3 : CP_PH4;
            chtype ch = p < 0.15f ? '.' :
                        p < 0.40f ? ':' :
                        p < 0.70f ? '+' : '#';
            attron(COLOR_PAIR(cp));
            mvaddch(r, c, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    /* ── food sources ────────────────────────────────────────────── */
    for (int k = 0; k < 2; k++) {
        attron(COLOR_PAIR(CP_FOOD) | A_BOLD);
        mvprintw(g_food_row[k], g_food_col[k] - 1, "[*]");
        attroff(COLOR_PAIR(CP_FOOD) | A_BOLD);
    }

    /* ── nest ────────────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_NEST) | A_BOLD);
    mvprintw(g_nest_row, g_nest_col - 1, "(@)");
    attroff(COLOR_PAIR(CP_NEST) | A_BOLD);

    /* ── ants ────────────────────────────────────────────────────── */
    for (int i = 0; i < N_ANTS; i++) {
        int r = g_ants[i].row, c = g_ants[i].col;
        if (r < 0 || r >= rows || c < 0 || c >= cols) continue;
        int cp = g_ants[i].state == RETURNING ? CP_ANT_R : CP_ANT_S;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvaddch(r, c, 'o');
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    /* ── HUD ─────────────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " AntColony  q:quit  spc:pause  r:reset  +/-:speed  "
        "[@]=nest  [*]=food  o=ant");
    mvprintw(1, 0,
        " ants:%d  food_collected:%d  speed:%dx  %s",
        N_ANTS, g_food_collected, g_speed,
        g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  screen + §8  app                                                   */
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
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));

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

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_init(rows, cols);

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            scene_init(rows, cols);
            last = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;                      break;
        case ' ':                    g_paused = !g_paused;             break;
        case 'r': case 'R':          scene_init(rows, cols);           break;
        case '+': case '=':
            g_speed++; if (g_speed > 8) g_speed = 8;                  break;
        case '-':
            g_speed--; if (g_speed < 1) g_speed = 1;                  break;
        default: break;
        }

        long long now = clock_ns();
        long long dt  = now - last;
        last = now;
        if (dt > 100000000LL) dt = 100000000LL;
        (void)dt;

        for (int i = 0; i < g_speed; i++) scene_tick();

        erase();
        scene_draw(rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }

    return 0;
}
