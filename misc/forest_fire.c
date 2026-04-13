/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * forest_fire.c — Drossel-Schwabl Forest Fire Cellular Automaton
 *
 * Classic 3-state probabilistic CA modelling fire ecology.
 * Each cell is one of: EMPTY, TREE, or FIRE.
 *
 * Update rules (applied simultaneously to all cells):
 *   FIRE  → EMPTY    (fire burns out in one tick)
 *   TREE  → FIRE     if any of the 4 orthogonal neighbours is FIRE (spread)
 *   TREE  → FIRE     with probability f (lightning strike)
 *   TREE  → TREE     otherwise
 *   EMPTY → TREE     with probability p (regrowth)
 *   EMPTY → EMPTY    otherwise
 *
 * Key insight: the ratio p/f controls the characteristic cluster size.
 *   Large p/f → trees grow fast, large dense clusters → catastrophic fires
 *   Small p/f → sparse trees → frequent small fires, power-law size distribution
 *
 * At the critical ratio the fire size distribution is a power law:
 *   P(s) ∝ s^{−τ}  with τ ≈ 1.19  (self-organised criticality)
 *
 * Reference: Drossel & Schwabl, PRL 69(11):1629, 1992.
 *
 * Presets:
 *   0  Classic    — p=0.030 f=0.0002, balanced; moderate clusters
 *   1  Dense      — p=0.060 f=0.0001, fast growth; large catastrophic burns
 *   2  Sparse     — p=0.010 f=0.0010, sparse; frequent small fires
 *   3  Smouldering — p=0.020 f=0.0003, 8-neighbour spread; slow creeping fires
 *
 * Keys:
 *   q / ESC    quit
 *   p / space  pause / resume
 *   r          reset (reseed grid)
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   g / G      growth prob p up / down
 *   l / L      lightning prob f up / down
 *   s          scatter lightning (manual ignitions across grid)
 *   + / -      sim speed up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/forest_fire.c \
 *       -o forest_fire -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 grid  §5 scene  §6 screen  §7 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define ROWS_MAX   128
#define COLS_MAX   512

/* Cell states */
#define EMPTY  0
#define TREE   1
#define FIRE   2

/* Default probabilities */
#define P_GROW_DEF   0.030f   /* P: empty → tree per tick                  */
#define P_FIRE_DEF   0.0002f  /* F: tree  → fire (lightning) per tick       */
#define P_GROW_STEP  0.005f
#define P_FIRE_STEP  0.0001f
#define P_GROW_MIN   0.001f
#define P_GROW_MAX   0.200f
#define P_FIRE_MIN   0.00001f
#define P_FIRE_MAX   0.010f

/* Manual lightning scatter */
#define SCATTER_COUNT  12    /* number of random ignitions per scatter key  */

/* Simulation timing */
#define SIM_FPS_DEF   20
#define SIM_FPS_MIN    2
#define SIM_FPS_MAX   60
#define SIM_FPS_STEP   2

#define N_PRESETS  4
#define N_THEMES   5

#define NS_PER_SEC  1000000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

enum {
    CP_EMPTY = 1,
    CP_TREE  = 2,
    CP_FIRE1 = 3,   /* dim fire  */
    CP_FIRE2 = 4,   /* bright fire */
    CP_ASH   = 5,   /* ash / embers (just-burned cell, one tick after FIRE) */
    CP_HUD   = 6,
};

typedef struct {
    short empty,  tree,  fire1, fire2, ash, hud;
    short empty8, tree8, fire18, fire28, ash8, hud8;
    const char *name;
} Theme;

/* 256-color palettes; 8-color fallbacks */
static const Theme k_themes[N_THEMES] = {
    /* 0  Classic — green forest, orange-red fire */
    { 236, 34, 202, 196, 240, 244,
      COLOR_BLACK, COLOR_GREEN, COLOR_YELLOW, COLOR_RED, COLOR_WHITE, COLOR_WHITE,
      "Classic" },
    /* 1  Night   — dark green forest, white-blue fire */
    { 235, 22, 214, 226, 238, 244,
      COLOR_BLACK, COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      "Night" },
    /* 2  Autumn  — orange trees, red fire */
    { 236, 130, 196, 160, 240, 244,
      COLOR_BLACK, COLOR_YELLOW, COLOR_RED, COLOR_RED, COLOR_WHITE, COLOR_WHITE,
      "Autumn" },
    /* 3  Boreal  — cyan-tinted trees, yellow-white fire */
    { 235, 30, 220, 231, 238, 244,
      COLOR_BLACK, COLOR_CYAN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      "Boreal" },
    /* 4  Lava    — very dark trees, magenta-white fire */
    { 234, 22, 201, 207, 238, 244,
      COLOR_BLACK, COLOR_GREEN, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      "Lava" },
};

static bool g_has_256;

static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_EMPTY, th->empty, th->empty);
        init_pair(CP_TREE,  th->tree,  th->empty);
        init_pair(CP_FIRE1, th->fire1, th->empty);
        init_pair(CP_FIRE2, th->fire2, th->empty);
        init_pair(CP_ASH,   th->ash,   th->empty);
        init_pair(CP_HUD,   th->hud,   236);
    } else {
        init_pair(CP_EMPTY, th->empty8,  th->empty8);
        init_pair(CP_TREE,  th->tree8,   COLOR_BLACK);
        init_pair(CP_FIRE1, th->fire18,  COLOR_BLACK);
        init_pair(CP_FIRE2, th->fire28,  COLOR_BLACK);
        init_pair(CP_ASH,   th->ash8,    COLOR_BLACK);
        init_pair(CP_HUD,   th->hud8,    COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

static uint8_t g_grid[ROWS_MAX][COLS_MAX];  /* current state              */
static uint8_t g_next[ROWS_MAX][COLS_MAX];  /* next state (double-buffer) */
static uint8_t g_ash [ROWS_MAX][COLS_MAX];  /* 1-tick ash flag            */

static int g_rows, g_cols;   /* actual grid dimensions (set from terminal) */

/* Preset configuration */
typedef struct {
    float p_grow;
    float p_fire;
    bool  eight_neighbor;   /* preset 3: 8-way spread instead of 4-way    */
    float density;          /* initial tree density                        */
    const char *name;
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { 0.030f, 0.0002f, false, 0.60f, "Classic" },
    { 0.060f, 0.0001f, false, 0.70f, "Dense"   },
    { 0.010f, 0.0010f, false, 0.40f, "Sparse"  },
    { 0.020f, 0.0003f, true,  0.55f, "Smoulder"},
};

/* Simulation state */
static float g_p_grow;
static float g_p_fire;
static bool  g_eight_neighbor;
static int   g_preset  = 0;
static int   g_theme   = 0;
static int   g_sim_fps = SIM_FPS_DEF;
static bool  g_paused  = false;

/* Statistics (displayed in HUD) */
static int g_n_tree, g_n_fire, g_n_empty;
static long g_tick = 0;

/* ------------------------------------------------------------------ */
/* LCG fast random for hot inner loop: xorshift32                      */
/* ------------------------------------------------------------------ */
static uint32_t g_rng = 12345;
static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
/* Returns uniform float in [0,1) */
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1 << 24);
}

/* ------------------------------------------------------------------ */

static void grid_seed(float density)
{
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++) {
            g_grid[r][c] = (rng_float() < density) ? TREE : EMPTY;
            g_ash[r][c]  = 0;
        }
}

static void scene_init(int preset)
{
    g_preset        = preset;
    g_p_grow        = k_presets[preset].p_grow;
    g_p_fire        = k_presets[preset].p_fire;
    g_eight_neighbor = k_presets[preset].eight_neighbor;
    g_tick          = 0;
    grid_seed(k_presets[preset].density);
}

/* ------------------------------------------------------------------ */
/* Single simulation step                                               */
/* ------------------------------------------------------------------ */

static void grid_step(void)
{
    int nt = 0, nf = 0, ne = 0;
    memset(g_ash, 0, sizeof(g_ash));

    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            uint8_t state = g_grid[r][c];

            if (state == FIRE) {
                /* Fire burns out */
                g_next[r][c] = EMPTY;
                g_ash[r][c]  = 1;
                ne++;
                continue;
            }

            if (state == TREE) {
                /* Check neighbours for fire spread */
                bool neighbor_fire = false;

                /* 4-neighbour orthogonal check */
                int dr4[4] = {-1, 1, 0, 0};
                int dc4[4] = { 0, 0,-1, 1};
                for (int d = 0; d < 4; d++) {
                    int nr = r + dr4[d];
                    int nc = c + dc4[d];
                    if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols)
                        if (g_grid[nr][nc] == FIRE) {
                            neighbor_fire = true;
                            break;
                        }
                }

                /* Optional 8-neighbour diagonal check */
                if (!neighbor_fire && g_eight_neighbor) {
                    int dr8[4] = {-1,-1, 1, 1};
                    int dc8[4] = {-1, 1,-1, 1};
                    for (int d = 0; d < 4; d++) {
                        int nr = r + dr8[d];
                        int nc = c + dc8[d];
                        if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols)
                            if (g_grid[nr][nc] == FIRE) {
                                neighbor_fire = true;
                                break;
                            }
                    }
                }

                if (neighbor_fire || rng_float() < g_p_fire) {
                    g_next[r][c] = FIRE;
                    nf++;
                } else {
                    g_next[r][c] = TREE;
                    nt++;
                }
                continue;
            }

            /* EMPTY — probabilistic regrowth */
            if (rng_float() < g_p_grow) {
                g_next[r][c] = TREE;
                nt++;
            } else {
                g_next[r][c] = EMPTY;
                ne++;
            }
        }
    }

    memcpy(g_grid, g_next, (size_t)g_rows * COLS_MAX);
    g_n_tree  = nt;
    g_n_fire  = nf;
    g_n_empty = ne;
    g_tick++;
}

/* Manual scatter: randomly ignite SCATTER_COUNT trees */
static void grid_scatter(void)
{
    int tries = SCATTER_COUNT * 8;
    int lit   = 0;
    while (lit < SCATTER_COUNT && tries-- > 0) {
        int r = (int)(rng_float() * (float)g_rows);
        int c = (int)(rng_float() * (float)g_cols);
        if (r >= 0 && r < g_rows && c >= 0 && c < g_cols
                && g_grid[r][c] == TREE) {
            g_grid[r][c] = FIRE;
            lit++;
        }
    }
}

/* ===================================================================== */
/* §5  scene draw                                                         */
/* ===================================================================== */

/*
 * Character set for each state.
 *
 * EMPTY  — space (invisible; bg color shows)
 * TREE   — '^' looks like a treetop; alternate 'T' or '|'
 * FIRE   — '*' (bright), ',' (dim edge of fire)
 * ASH    — '.' (just burned)
 */
static void scene_draw(void)
{
    erase();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    /* Clamp grid to terminal */
    int draw_rows = (g_rows < rows - 2) ? g_rows : rows - 2;
    int draw_cols = (g_cols < cols)     ? g_cols : cols;

    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {
            uint8_t state = g_grid[r][c];

            if (state == EMPTY) {
                if (g_ash[r][c]) {
                    attron(COLOR_PAIR(CP_ASH));
                    mvaddch(r, c, '.');
                    attroff(COLOR_PAIR(CP_ASH));
                } else {
                    attron(COLOR_PAIR(CP_EMPTY));
                    mvaddch(r, c, ' ');
                    attroff(COLOR_PAIR(CP_EMPTY));
                }
            } else if (state == TREE) {
                attron(COLOR_PAIR(CP_TREE) | A_BOLD);
                mvaddch(r, c, '^');
                attroff(COLOR_PAIR(CP_TREE) | A_BOLD);
            } else { /* FIRE */
                /* Alternate brightness by position for flickering effect */
                bool bright = ((r + c + (int)(g_tick)) & 1);
                if (bright) {
                    attron(COLOR_PAIR(CP_FIRE2) | A_BOLD);
                    mvaddch(r, c, '*');
                    attroff(COLOR_PAIR(CP_FIRE2) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(CP_FIRE1));
                    mvaddch(r, c, ',');
                    attroff(COLOR_PAIR(CP_FIRE1));
                }
            }
        }
    }

    /* HUD */
    int total = g_n_tree + g_n_fire + g_n_empty;
    float tree_pct  = total > 0 ? 100.0f * (float)g_n_tree  / (float)total : 0;
    float fire_pct  = total > 0 ? 100.0f * (float)g_n_fire  / (float)total : 0;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(rows - 2, 0,
        " Forest Fire | Preset: %s | Theme: %s | FPS: %d ",
        k_presets[g_preset].name, k_themes[g_theme].name, g_sim_fps);
    mvprintw(rows - 1, 0,
        " Trees:%5.1f%% Fire:%4.1f%% | p=%.4f f=%.5f | "
        "%s | tick:%ld | [g/G]row [l/L]ight [s]catter [n/N]preset [t/T]heme",
        tree_pct, fire_pct,
        g_p_grow, g_p_fire,
        g_paused ? "PAUSED" : "running",
        g_tick);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §6  screen / ncurses init                                              */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    start_color();
    g_has_256 = (COLORS >= 256);
    theme_apply(g_theme);
}

static void screen_resize(void)
{
    endwin();
    refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    g_cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
    g_resize = 0;
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    /* Seed RNG from time */
    g_rng = (uint32_t)time(NULL) ^ 0xDEADBEEFu;

    screen_init();

    {
        int rows, cols;
        getmaxyx(stdscr, rows, cols);
        g_rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
        g_cols = (cols     < COLS_MAX) ? cols      : COLS_MAX;
    }

    scene_init(g_preset);

    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;

            case 'p': case ' ':
                g_paused = !g_paused;
                break;

            case 'r':
                scene_init(g_preset);
                break;

            case 'n':
                scene_init((g_preset + 1) % N_PRESETS);
                break;
            case 'N':
                scene_init((g_preset + N_PRESETS - 1) % N_PRESETS);
                break;

            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme);
                break;

            case 'g':
                if (g_p_grow < P_GROW_MAX) g_p_grow += P_GROW_STEP;
                break;
            case 'G':
                if (g_p_grow > P_GROW_MIN) g_p_grow -= P_GROW_STEP;
                break;

            case 'l':
                if (g_p_fire < P_FIRE_MAX) g_p_fire += P_FIRE_STEP;
                break;
            case 'L':
                if (g_p_fire > P_FIRE_MIN) g_p_fire -= P_FIRE_STEP;
                break;

            case 's':
                grid_scatter();
                break;

            case '+': case '=':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;
            case '-':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── handle resize ── */
        if (g_resize) {
            screen_resize();
            scene_init(g_preset);
        }

        /* ── tick ── */
        int64_t now = clock_ns();
        if (!g_paused && now >= next_tick) {
            grid_step();
            next_tick = now + TICK_NS(g_sim_fps);
        }

        /* ── render ── */
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next tick ── */
        int64_t sleep_ns = next_tick - clock_ns() - 1000000LL;
        clock_sleep_ns(sleep_ns);
    }

    endwin();
    return 0;
}
