/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diffusion_map.c  —  Diffusion-Limited Aggregation without symmetry
 *
 * Particles random-walk from a launch circle until they touch the growing
 * cluster and stick.  Without the D6 symmetry constraint of snowflake.c,
 * the aggregate is free to grow in any direction, producing natural fractal
 * branching arms and organic, asymmetric dendrites — the classic DLA
 * morphology described by Witten & Sander (1981).
 *
 * A second "Eden" mode skips the random walk entirely: a random frontier
 * cell (any empty cell adjacent to the cluster) is chosen uniformly and
 * added.  Eden growth is much faster but produces rounder, less fractal
 * shapes (no diffusion bias toward tips).
 *
 * Color is by age_delta = current_frame - cell_join_frame:
 *   newest cells = bright / bold; oldest = dim
 *
 * Themes cycle across 5 color palettes (5 age levels each).
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   r         reset (clear grid, re-seed)
 *   t / T     next / prev theme
 *   +         more walkers per frame (max 10)
 *   -         fewer walkers per frame (min 1)
 *   n         toggle DLA / Eden mode
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra diffusion_map.c -o diffusion_map -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  lcg rng
 *   §4  color / themes
 *   §5  grid
 *   §6  dla walker
 *   §7  eden growth
 *   §8  scene
 *   §9  screen
 *   §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two aggregation modes in one file:
 *                  DLA (Diffusion-Limited Aggregation, Witten & Sander 1981):
 *                    Particles launched from a ring, random-walk until they
 *                    touch the cluster.  Tip-screening effect: tips extend
 *                    further from the centre and capture walkers preferentially,
 *                    creating fractal branching with D ≈ 1.7 in 2D.
 *                  Eden model: directly attach a random frontier cell.
 *                    No diffusion → no tip screening → compact, rounder shapes
 *                    (D → 2 as cluster grows; no fractal structure at large scales).
 *
 * Math           : DLA fractal dimension D ≈ 1.71 in 2D.
 *                  Cluster radius R ~ N^(1/D) where N = number of particles.
 *                  Comparison between modes in the same code illustrates how
 *                  diffusion (randomness in the approach path) is necessary for
 *                  fractal self-similar morphology.
 *
 * Performance    : DLA walker cost: O(R²) expected random-walk steps per particle
 *                  (hitting probability from radius 2R to R ≈ 1/(log R) in 2D).
 *                  Eden mode: O(frontier size) per particle — much faster.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
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

enum {
    ROWS_MAX        =  80,
    COLS_MAX        = 300,

    RENDER_FPS      =  30,

    WALKER_MIN      =   1,
    WALKER_DEFAULT  =   3,
    WALKER_MAX      =  10,

    MAX_STEPS       = 500,   /* max random-walk steps before aborting     */
    LAUNCH_PAD      =   3,   /* launch circle = g_radius + LAUNCH_PAD     */

    N_THEMES        =   5,
    N_AGE_LEVELS    =   5,   /* color pairs CP_A0 … CP_A4                 */
    CP_HUD          =   1,   /* color pair for HUD text                   */
    CP_A0           =   2,   /* newest cells (index offset 0)             */
    /* CP_A1 = 3, CP_A2 = 4, CP_A3 = 5, CP_A4 = 6                        */

    FPS_UPDATE_MS   = 500,
};

/* age_delta thresholds: 0-5, 6-20, 21-80, 81-300, 300+ */
static const int AGE_THRESH[N_AGE_LEVELS] = { 5, 20, 80, 300, INT32_MAX };

#define RENDER_NS   (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ===================================================================== */
/* §2  clock                                                             */
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
/* §3  lcg rng                                                           */
/* ===================================================================== */

static uint32_t g_lcg;

static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* integer in [0, n) */
static int lcg_i(int n)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (int)((g_lcg >> 8) % (uint32_t)n);
}

/* ===================================================================== */
/* §4  color / themes                                                    */
/* ===================================================================== */

/*
 * 5 themes × 5 age levels (newest → oldest):
 *
 *  Coral:  231 226 208 196 124  white→yellow→orange→red→dark red
 *  Ice:    231 123  51  27  17  white→cyan→blue→dark
 *  Lava:   231 220 208 196  88  white→yellow→orange→red→dark
 *  Plasma: 231 213 165  93  54  white→pink→purple→dark
 *  Mono:   255 251 244 238 235  bright→dark grays
 */
static const short THEME_FG[N_THEMES][N_AGE_LEVELS] = {
    { 231, 226, 208, 196, 124 },  /* Coral  */
    { 231, 123,  51,  27,  17 },  /* Ice    */
    { 231, 220, 208, 196,  88 },  /* Lava   */
    { 231, 213, 165,  93,  54 },  /* Plasma */
    { 255, 251, 244, 238, 235 },  /* Mono   */
};

static const char *THEME_NAME[N_THEMES] = {
    "Coral", "Ice", "Lava", "Plasma", "Mono"
};

/* Fallback 8-color equivalents for each age level */
static const short THEME_FG8[N_AGE_LEVELS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_RED, COLOR_RED
};

static int g_theme;

static void color_init_theme(void)
{
    /* HUD pair */
    if (COLORS >= 256)
        init_pair(CP_HUD, 250, COLOR_BLACK);
    else
        init_pair(CP_HUD, COLOR_WHITE, COLOR_BLACK);

    /* Age level pairs CP_A0 … CP_A4 */
    for (int i = 0; i < N_AGE_LEVELS; i++) {
        if (COLORS >= 256)
            init_pair((short)(CP_A0 + i),
                      THEME_FG[g_theme][i], COLOR_BLACK);
        else
            init_pair((short)(CP_A0 + i),
                      THEME_FG8[i], COLOR_BLACK);
    }
}

/* Map age_delta to color pair index (CP_A0 … CP_A4) */
static int age_pair(int age_delta)
{
    for (int i = 0; i < N_AGE_LEVELS; i++)
        if (age_delta <= AGE_THRESH[i])
            return CP_A0 + i;
    return CP_A0 + N_AGE_LEVELS - 1;
}

/* Map age_delta to display character */
static chtype age_char(int age_delta)
{
    if (age_delta <= AGE_THRESH[0]) return (chtype)'@';
    if (age_delta <= AGE_THRESH[1]) return (chtype)'#';
    if (age_delta <= AGE_THRESH[2]) return (chtype)'+';
    if (age_delta <= AGE_THRESH[3]) return (chtype)':';
    return (chtype)'.';
}

/* ===================================================================== */
/* §5  grid                                                              */
/* ===================================================================== */

static uint8_t  g_grid[ROWS_MAX][COLS_MAX]; /* 0=empty, 1=cluster */
static uint16_t g_age [ROWS_MAX][COLS_MAX]; /* frame when cell joined */

static int g_rows, g_cols;
static int g_cx, g_cy;          /* center of grid */
static int g_radius;            /* max Chebyshev distance of cluster from center */
static int g_frame;             /* simulation frame counter */
static int g_cluster_size;      /* total cells in cluster */

static void grid_reset(void)
{
    memset(g_grid, 0, sizeof g_grid);
    memset(g_age,  0, sizeof g_age);
    g_cx     = g_cols / 2;
    g_cy     = g_rows / 2;
    g_radius = 0;
    g_frame  = 1;
    g_cluster_size = 0;

    /* Seed: single cell at center */
    g_grid[g_cy][g_cx] = 1;
    g_age [g_cy][g_cx] = 1;
    g_cluster_size     = 1;
}

static void grid_set_size(int cols, int rows)
{
    g_cols = (cols < COLS_MAX) ? cols : COLS_MAX;
    g_rows = (rows < ROWS_MAX) ? rows : ROWS_MAX;
}

/* Check 4-neighbor adjacency to cluster */
static bool grid_has_cluster_neighbor(int r, int c)
{
    if (r > 0          && g_grid[r-1][c]) return true;
    if (r < g_rows - 1 && g_grid[r+1][c]) return true;
    if (c > 0          && g_grid[r][c-1]) return true;
    if (c < g_cols - 1 && g_grid[r][c+1]) return true;
    return false;
}

/* Add cell (r,c) to cluster */
static void grid_add_cell(int r, int c)
{
    g_grid[r][c] = 1;
    g_age [r][c] = (uint16_t)(g_frame & 0xFFFF);
    g_cluster_size++;

    /* Update Chebyshev radius */
    int dr = abs(r - g_cy);
    int dc = abs(c - g_cx);
    int ch = (dr > dc) ? dr : dc;
    if (ch > g_radius) g_radius = ch;
}

static void grid_draw(void)
{
    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            if (!g_grid[r][c]) continue;

            int age_delta = g_frame - (int)(g_age[r][c]);
            if (age_delta < 0) age_delta = 0;

            int pair  = age_pair(age_delta);
            chtype ch = age_char(age_delta);
            attr_t attr = (attr_t)COLOR_PAIR(pair);
            if (age_delta <= AGE_THRESH[0]) attr |= A_BOLD;

            attron(attr);
            mvaddch(r, c, ch);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §6  dla walker                                                        */
/* ===================================================================== */

static const int DR4[4] = { -1, 1,  0, 0 };
static const int DC4[4] = {  0, 0, -1, 1 };

/*
 * Run one DLA walker:
 *   1. Launch at random point on circle radius (g_radius + LAUNCH_PAD).
 *   2. Random-walk up to MAX_STEPS steps.
 *   3. Stick if any 4-neighbor is in cluster.
 *   4. Abort if outside kill_radius or grid bounds.
 * Returns true if the walker stuck.
 */
static bool dla_walker_run(void)
{
    float angle = lcg_f() * 6.28318f;
    float dist  = (float)(g_radius + LAUNCH_PAD);

    int r = g_cy + (int)roundf(dist * sinf(angle) * 0.5f);
    int c = g_cx + (int)roundf(dist * cosf(angle));

    /* Clamp to grid */
    if (r < 0) r = 0;
    if (r >= g_rows) r = g_rows - 1;
    if (c < 0) c = 0;
    if (c >= g_cols) c = g_cols - 1;

    int kill_radius = g_radius * 2 + LAUNCH_PAD * 3;

    for (int step = 0; step < MAX_STEPS; step++) {
        /* Bounds check */
        if (r < 0 || r >= g_rows || c < 0 || c >= g_cols)
            return false;

        /* Kill radius check (Chebyshev) */
        int dr = abs(r - g_cy);
        int dc = abs(c - g_cx);
        int ch = (dr > dc) ? dr : dc;
        if (ch > kill_radius)
            return false;

        /* Check for cluster neighbor → stick */
        if (grid_has_cluster_neighbor(r, c)) {
            if (g_grid[r][c] == 0) {
                grid_add_cell(r, c);
                return true;
            }
            /* already occupied, move away one step */
        }

        /* Random walk step */
        int dir = lcg_i(4);
        r += DR4[dir];
        c += DC4[dir];
    }
    return false;
}

/* ===================================================================== */
/* §7  eden growth                                                       */
/* ===================================================================== */

/*
 * Eden mode: collect all frontier cells (empty cells adjacent to cluster),
 * pick one uniformly at random, add it.
 * Uses a dynamic array allocated on-stack (bounded by grid size).
 *
 * Returns true if a cell was added.
 */
static bool eden_step(void)
{
    /* Collect frontier cells */
    static int fr[ROWS_MAX * COLS_MAX];
    static int fc[ROWS_MAX * COLS_MAX];
    int n = 0;

    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            if (g_grid[r][c]) continue;
            if (grid_has_cluster_neighbor(r, c)) {
                fr[n] = r;
                fc[n] = c;
                n++;
            }
        }
    }
    if (n == 0) return false;

    int idx = lcg_i(n);
    grid_add_cell(fr[idx], fc[idx]);
    return true;
}

/* ===================================================================== */
/* §8  scene                                                             */
/* ===================================================================== */

typedef struct {
    bool paused;
    bool eden_mode;
    int  n_walkers;
} Scene;

static Scene g_scene;

static void scene_init(void)
{
    g_scene.paused    = false;
    g_scene.eden_mode = false;
    g_scene.n_walkers = WALKER_DEFAULT;
}

static void scene_tick(void)
{
    if (g_scene.paused) return;

    g_frame++;

    if (g_scene.eden_mode) {
        for (int i = 0; i < g_scene.n_walkers; i++)
            eden_step();
    } else {
        for (int i = 0; i < g_scene.n_walkers; i++)
            dla_walker_run();
    }
}

/* ===================================================================== */
/* §9  screen                                                            */
/* ===================================================================== */

static int g_scr_rows, g_scr_cols;

static void screen_init_ncurses(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
    color_init_theme();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
}

static void screen_draw_hud(double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " %s | %s | walkers:%d | size:%-5d | %4.1f fps ",
             g_scene.eden_mode ? "Eden" : "DLA ",
             THEME_NAME[g_theme],
             g_scene.n_walkers,
             g_cluster_size,
             fps);

    int hud_row = g_rows < g_scr_rows ? g_rows : g_scr_rows - 1;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hud_row, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10 app                                                               */
/* ===================================================================== */

static volatile sig_atomic_t g_running    = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_resize(void)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
    grid_set_size(g_scr_cols, g_scr_rows - 1);
    grid_reset();
    g_need_resize = 0;
}

static bool app_handle_key(int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case 'p': case 'P': case ' ':
        g_scene.paused = !g_scene.paused;
        break;

    case 'r': case 'R':
        grid_reset();
        break;

    case 't':
        g_theme = (g_theme + 1) % N_THEMES;
        color_init_theme();
        break;

    case 'T':
        g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
        color_init_theme();
        break;

    case '+': case '=':
        if (g_scene.n_walkers < WALKER_MAX)
            g_scene.n_walkers++;
        break;

    case '-':
        if (g_scene.n_walkers > WALKER_MIN)
            g_scene.n_walkers--;
        break;

    case 'n': case 'N':
        g_scene.eden_mode = !g_scene.eden_mode;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_lcg = (uint32_t)(ts.tv_nsec ^ ts.tv_sec);

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    g_theme = 0;

    screen_init_ncurses();
    grid_set_size(g_scr_cols, g_scr_rows - 1);
    grid_reset();
    scene_init();

    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;
    int64_t last_frame  = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            app_resize();
            scene_init();
            last_frame = clock_ns();
            fps_accum  = 0;
            frame_count = 0;
        }

        scene_tick();

        /* FPS measurement */
        int64_t now = clock_ns();
        int64_t dt  = now - last_frame;
        last_frame  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;
        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum   = 0;
            frame_count = 0;
        }

        /* Draw */
        erase();
        grid_draw();
        screen_draw_hud(fps_display);
        screen_present();

        /* Key input */
        int ch = getch();
        if (ch != ERR && !app_handle_key(ch))
            g_running = 0;

        /* Frame cap */
        int64_t elapsed = clock_ns() - last_frame + dt;
        clock_sleep_ns(RENDER_NS - elapsed);
    }

    endwin();
    return 0;
}
