/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fluid_sph.c — SPH Fluid Simulation
 *
 * Original algorithm: interactive particle fluid playground.
 * Translated into framework.c §1–§8 structure with ncurses rendering.
 *
 * Physics kernel: w = d/H − 1  (positive for d > H, zero for d ≤ H)
 * Density:        ρᵢ = Σⱼ w²
 * Pressure force: F = w · (REST − ρᵢ − ρⱼ) · P / ρᵢ  (along i→j)
 * Viscosity:      F += (vᵢ − vⱼ) · V
 * Integration:    Symplectic Euler + wall bounce + damping
 *
 * Scenes:
 *   1  blob drop      — sphere of particles falling under gravity
 *   2  column         — rectangular block falling
 *   3  fountain       — 50 particles stacked at center bottom
 *   4  collision      — two blobs launched toward each other
 *   5  rain           — random horizontal shower from top
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   1–5       load scene
 *   g         toggle gravity
 *   v         toggle viscosity
 *   r         reload current scene
 *   +         spawn extra blob at top
 *   t         cycle colour theme
 *   ] / [     raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/fluid_sph.c -o fluid_sph -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 entity
 *           §6 scene   §7 screen §8 app
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define MAX_PARTICLES   5000

/* SPH constants — kept identical to original */
#define SMOOTH_RADIUS   2.2
#define PRESSURE_K      0.04
#define VISCOSITY_K     0.03
#define GRAVITY_G       0.08
#define WALL_DAMPING    0.6
#define SPH_DT          0.12    /* fixed physics timestep (original DT)  */
#define REST_SUM        6.0     /* target ρᵢ + ρⱼ at equilibrium         */

/*
 * Spatial grid for O(N) neighbour search.
 * Cell size GCELL ≥ SMOOTH_RADIUS ensures all neighbours within the kernel
 * radius are in the 3×3 block of surrounding cells.  Each particle only
 * checks ~9 × (avg particles/cell) pairs instead of all N.
 * Typical speedup: N / avg_neighbours  ≈  800 / 15  ≈  50×.
 */
#define GCELL    3          /* cell side length — must be ≥ SMOOTH_RADIUS  */
#define GMAX_W  90          /* max grid columns  (terminal ≤ 270 cols / 3) */
#define GMAX_H  22          /* max grid rows     (terminal ≤  66 rows / 3) */

/* density thresholds for character / colour zones */
#define T_DENSE         3.5
#define T_MID           1.2
#define T_EDGE          0.1

/* rows reserved at bottom for HUD */
#define HUD_ROWS        2

/* colour pairs */
#define CP_DENSE        1
#define CP_MID          2
#define CP_EDGE         3
#define CP_BORDER       4
#define CP_HUD          5

#define N_THEMES       10

/* framework timing */
#define SIM_FPS_MIN      10
#define SIM_FPS_DEFAULT  60
#define SIM_FPS_MAX     120
#define SIM_FPS_STEP     10
#define FPS_UPDATE_MS   500
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * 10 themes × 3 particle levels (dense / mid / edge).
 * Background is always -1 (transparent — terminal default).
 * Columns: DENSE  MID  EDGE
 */
static const short THEMES[N_THEMES][3] = {
    {  51,  39,  27 },   /* 0 ocean  — bright cyan → blue          */
    { 196, 208, 220 },   /* 1 lava   — red → orange → yellow       */
    { 226, 214, 196 },   /* 2 fire   — white-yellow → orange → red */
    {  46,  34,  22 },   /* 3 matrix — bright green → dark green   */
    { 231, 141,  93 },   /* 4 nova   — white → violet → purple     */
    { 231, 159, 117 },   /* 5 ice    — white → sky → steel blue    */
    { 220, 208, 197 },   /* 6 sunset — yellow → orange → rose      */
    { 196, 160, 124 },   /* 7 blood  — bright red → crimson → dark */
    { 201, 198, 165 },   /* 8 neon   — magenta → pink → soft purple*/
    { 154, 118,  46 },   /* 9 acid   — yellow-green → green        */
};

static const char *THEME_NAMES[N_THEMES] = {
    "ocean","lava","fire","matrix","nova","ice","sunset","blood","neon","acid"
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();   /* enables -1 as transparent background */
    if (theme < 0 || theme >= N_THEMES) theme = 0;
    if (COLORS >= 256) {
        init_pair(CP_DENSE,  THEMES[theme][0], -1);   /* no background */
        init_pair(CP_MID,    THEMES[theme][1], -1);
        init_pair(CP_EDGE,   THEMES[theme][2], -1);
        init_pair(CP_BORDER, 242,              -1);
        init_pair(CP_HUD,    226,              -1);   /* yellow HUD    */
    } else {
        init_pair(CP_DENSE,  COLOR_CYAN,   -1);
        init_pair(CP_MID,    COLOR_CYAN,   -1);
        init_pair(CP_EDGE,   COLOR_BLUE,   -1);
        init_pair(CP_BORDER, COLOR_WHITE,  -1);
        init_pair(CP_HUD,    COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

/*
 * The physics grid IS the cell grid (integer column/row), so no pixel↔cell
 * conversion is needed.  §4 is kept as a template stub for completeness.
 * See framework.c §4 for the full explanation of when this section matters.
 */
#define CELL_W   8
#define CELL_H  16

static inline int px_to_cell_x(float px) { return (int)floorf(px / CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / CELL_H + 0.5f); }
/* suppress unused-function warnings when §4 is not exercised */
static inline void _coords_unused(void) { (void)px_to_cell_x; (void)px_to_cell_y; }

/* ===================================================================== */
/* §5  entity — Particle + SPH physics                                   */
/* ===================================================================== */

typedef struct {
    double x, y;
    double vx, vy;
    double ax, ay;
    double density;
} Particle;

/* global simulation state (physics functions are pure operations on these) */
static Particle g_p[MAX_PARTICLES];
static int      g_n          = 0;
static bool     g_gravity    = true;
static bool     g_viscosity  = true;

/* physics boundary — set from actual terminal size before each tick */
static int g_phys_cols = 80;
static int g_phys_rows = 22;

/* ── particle creation helpers ─────────────────────────────────────── */

static void particle_add(double x, double y)
{
    if (g_n >= MAX_PARTICLES) return;
    g_p[g_n] = (Particle){ .x = x, .y = y };
    g_n++;
}

static void add_blob(int cx, int cy, int r)
{
    for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
        if (dx*dx + dy*dy <= r*r)
            particle_add(cx + dx, cy + dy);
}

static void add_rectangle(int x0, int y0, int w, int h)
{
    for (int dy = 0; dy < h; dy++)
    for (int dx = 0; dx < w; dx++)
        particle_add(x0 + dx, y0 + dy);
}

/* ── SPH kernel ────────────────────────────────────────────────────── */

/*
 * sph_kernel(d) — compact-support kernel.
 *
 *   w = d / H − 1
 *   For d < H:  w < 0  →  returns w²  (positive density contribution)
 *   For d ≥ H:  w ≥ 0  →  returns 0   (outside support radius, no contribution)
 *
 * The sign of w is what drives the force direction (see sph_forces):
 *   overpacked  (ρᵢ+ρⱼ > REST) → w*(neg) → positive along dx → repulsion ✓
 *   underpacked (ρᵢ+ρⱼ < REST) → w*(pos) → negative along dx → cohesion  ✓
 *
 * REST_SUM = 6.0 is calibrated for SMOOTH_RADIUS=2.2 at unit particle spacing:
 *   interior particle has ~15 neighbours, avg w²≈1/6, density ≈ 3 per particle.
 *   Equilibrium: REST_SUM = ρᵢ + ρⱼ = 3 + 3 = 6.
 */
static double sph_kernel(double d)
{
    double w = d / SMOOTH_RADIUS - 1.0;
    return (w < 0.0) ? w * w : 0.0;   /* contributes for d < H only */
}

static double sph_dist(const Particle *a, const Particle *b,
                       double *dx, double *dy)
{
    *dx = a->x - b->x;
    *dy = a->y - b->y;
    return sqrt((*dx) * (*dx) + (*dy) * (*dy));
}

/* ── spatial grid ──────────────────────────────────────────────────── */

/*
 * Linked-list-per-cell grid.  Uses two arrays:
 *   g_ghead[gy][gx] — index of first particle in cell, or -1
 *   g_gnext[i]      — index of next particle in the same cell, or -1
 *
 * grid_build() runs once per physics step (positions don't change mid-step).
 * Neighbour iteration: loop over the 3×3 block of cells around particle i,
 * walk each cell's linked list.  All neighbours within SMOOTH_RADIUS are
 * guaranteed to be in those 9 cells because GCELL ≥ SMOOTH_RADIUS.
 */
static int g_ghead[GMAX_H][GMAX_W];
static int g_gnext[MAX_PARTICLES];
static int g_gw, g_gh;

static void grid_build(void)
{
    g_gw = g_phys_cols / GCELL + 2;
    g_gh = g_phys_rows / GCELL + 2;
    if (g_gw > GMAX_W) g_gw = GMAX_W;
    if (g_gh > GMAX_H) g_gh = GMAX_H;

    for (int gy = 0; gy < g_gh; gy++)
        for (int gx = 0; gx < g_gw; gx++)
            g_ghead[gy][gx] = -1;

    for (int i = 0; i < g_n; i++) {
        int cx = (int)(g_p[i].x / GCELL);
        int cy = (int)(g_p[i].y / GCELL);
        if (cx < 0) cx = 0; else if (cx >= g_gw) cx = g_gw - 1;
        if (cy < 0) cy = 0; else if (cy >= g_gh) cy = g_gh - 1;
        g_gnext[i]       = g_ghead[cy][cx];
        g_ghead[cy][cx]  = i;
    }
}

/* ── physics passes ────────────────────────────────────────────────── */

static void sph_density(void)
{
    for (int i = 0; i < g_n; i++) {
        g_p[i].density = 0.0;
        int cx = (int)(g_p[i].x / GCELL);
        int cy = (int)(g_p[i].y / GCELL);
        for (int gy = cy-1; gy <= cy+1; gy++) {
            if (gy < 0 || gy >= g_gh) continue;
            for (int gx = cx-1; gx <= cx+1; gx++) {
                if (gx < 0 || gx >= g_gw) continue;
                for (int j = g_ghead[gy][gx]; j != -1; j = g_gnext[j]) {
                    double dx, dy;
                    double d = sph_dist(&g_p[i], &g_p[j], &dx, &dy);
                    g_p[i].density += sph_kernel(d);
                }
            }
        }
    }
}

static void sph_forces(void)
{
    for (int i = 0; i < g_n; i++) {
        Particle *pi = &g_p[i];
        pi->ax = 0.0;
        pi->ay = g_gravity ? GRAVITY_G : 0.0;

        int cx = (int)(pi->x / GCELL);
        int cy = (int)(pi->y / GCELL);

        for (int gy = cy-1; gy <= cy+1; gy++) {
            if (gy < 0 || gy >= g_gh) continue;
            for (int gx = cx-1; gx <= cx+1; gx++) {
                if (gx < 0 || gx >= g_gw) continue;
                for (int j = g_ghead[gy][gx]; j != -1; j = g_gnext[j]) {
                    if (i == j) continue;
                    Particle *pj = &g_p[j];

                    double dx, dy;
                    double d = sph_dist(pi, pj, &dx, &dy);
                    double w = d / SMOOTH_RADIUS - 1.0;
                    if (w >= 0.0) continue;

                    /* pressure + cohesion */
                    double pressure_force =
                        (REST_SUM - pi->density - pj->density) * PRESSURE_K;
                    double force = w * pressure_force / (pi->density + 0.001);
                    pi->ax += dx * force;
                    pi->ay += dy * force;

                    /* viscosity — damp toward neighbour velocity */
                    if (g_viscosity) {
                        double weight = -w;
                        pi->ax += (pj->vx - pi->vx) * VISCOSITY_K * weight;
                        pi->ay += (pj->vy - pi->vy) * VISCOSITY_K * weight;
                    }
                }
            }
        }
    }
}

static void sph_integrate(void)
{
    double xmax = (double)(g_phys_cols - 2);
    double ymax = (double)(g_phys_rows - 2);

    for (int i = 0; i < g_n; i++) {
        Particle *p = &g_p[i];
        p->vx += p->ax * SPH_DT;
        p->vy += p->ay * SPH_DT;
        p->x  += p->vx * SPH_DT;
        p->y  += p->vy * SPH_DT;

        if (p->x < 1.0)   { p->x = 1.0;   p->vx *= -WALL_DAMPING; }
        if (p->x > xmax)  { p->x = xmax;  p->vx *= -WALL_DAMPING; }
        if (p->y < 1.0)   { p->y = 1.0;   p->vy *= -WALL_DAMPING; }
        if (p->y > ymax)  { p->y = ymax;  p->vy *= -WALL_DAMPING; }
    }
}

static void sph_step(void)
{
    grid_build();    /* O(N)   — build spatial index once per step */
    sph_density();   /* O(N·k) — k = avg neighbours ≈ 15          */
    sph_forces();    /* O(N·k)                                     */
    sph_integrate(); /* O(N)                                       */
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    int  id;       /* current scene number 1–5     */
    int  theme;
    bool paused;
} Scene;

static void scene_load(Scene *s, int id)
{
    g_n    = 0;
    s->id  = id;
    int cx = g_phys_cols / 2;
    int cy = g_phys_rows / 2;

    switch (id) {
    case 1:   /* massive blob drop */
        add_blob(cx, 6, 12);
        break;

    case 2:   /* wide column collapse */
        add_rectangle(cx - 18, 2, 36, 16);
        break;

    case 3:   /* fountain burst — dense stack at base */
        for (int i = 0; i < 700; i++)
            particle_add(cx + (rand() % 9 - 4), g_phys_rows - 4);
        break;

    case 4:   /* head-on collision — large blobs */
        add_blob(cx - 20, cy, 10);
        add_blob(cx + 20, cy, 10);
        for (int i = 0; i < g_n; i++)
            g_p[i].vx = (i < g_n / 2) ? 2.5 : -2.5;
        break;

    case 5:   /* rain curtain — heavy shower from top */
        for (int i = 0; i < 800; i++)
            particle_add(rand() % (g_phys_cols - 4) + 2, rand() % 6 + 1);
        break;

    default: break;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->id     = 1;
    s->theme  = 0;
    s->paused = false;
    g_gravity   = true;
    g_viscosity = true;
    g_phys_cols = cols;
    g_phys_rows = rows - HUD_ROWS;
    color_init(s->theme);
    scene_load(s, 1);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)dt;   /* physics uses hardcoded SPH_DT — same as original */
    if (s->paused) return;
    g_phys_cols = cols;
    g_phys_rows = rows - HUD_ROWS;
    sph_step();
}

/*
 * scene_draw() — render all particles into WINDOW *w.
 *
 * Each particle is drawn at (round(x), round(y)) with a character and
 * colour that reflects its local density:
 *
 *   density ≥ T_DENSE  → '#' bold  (dense core)
 *   density ≥ T_MID    → 'o'       (fluid body)
 *   density ≥ T_EDGE   → '.'       (sparse edge / droplet)
 *   density  < T_EDGE  → ' '       (isolated — not drawn)
 *
 * Wall border is drawn last so it is always visible.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    int fluid_rows = rows - HUD_ROWS;

    /* ── particles ── */
    for (int i = 0; i < g_n; i++) {
        int cx = (int)(g_p[i].x + 0.5);
        int cy = (int)(g_p[i].y + 0.5);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= fluid_rows) continue;

        double d = g_p[i].density;
        if (d < T_EDGE) continue;

        int   pair;
        chtype ch;
        if (d >= T_DENSE) {
            pair = CP_DENSE;
            ch   = (chtype)'#' | A_BOLD;
        } else if (d >= T_MID) {
            pair = CP_MID;
            ch   = (chtype)'o';
        } else {
            pair = CP_EDGE;
            ch   = (chtype)'.';
        }

        wattron(w, COLOR_PAIR(pair));
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(pair));
    }

    /* ── border ── */
    wattron(w, COLOR_PAIR(CP_BORDER) | A_DIM);
    for (int c = 0; c < cols; c++) {
        mvwaddch(w, 0,             c, '-');
        mvwaddch(w, fluid_rows - 1, c, '-');
    }
    for (int r = 1; r < fluid_rows - 1; r++) {
        mvwaddch(w, r, 0,        '|');
        mvwaddch(w, r, cols - 1, '|');
    }
    mvwaddch(w, 0,             0,        '+');
    mvwaddch(w, 0,             cols - 1, '+');
    mvwaddch(w, fluid_rows - 1, 0,        '+');
    mvwaddch(w, fluid_rows - 1, cols - 1, '+');
    wattroff(w, COLOR_PAIR(CP_BORDER) | A_DIM);

    /* ── status line (row rows-2) ── */
    wattron(w, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(rows - 2, 0,
             " Scene:%d  n:%d  gravity:%s  viscosity:%s  theme:%s",
             s->id, g_n,
             g_gravity   ? "ON " : "OFF",
             g_viscosity ? "ON " : "OFF",
             THEME_NAMES[s->theme]);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols; int rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — key hints on last row, bold yellow */
    wattron(stdscr, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " %.1ffps sim:%dHz | 1-5:scene  g:grav  v:visc"
             "  r:reset  b:blob  t:theme  +/-:speed  spc:pause  q:quit",
             fps, sim_fps);
    wattroff(stdscr, COLOR_PAIR(CP_HUD) | A_BOLD);

    if (sc->paused) {
        wattron(stdscr, A_REVERSE | A_BOLD);
        mvprintw(0, s->cols - 8, " PAUSED ");
        wattroff(stdscr, A_REVERSE | A_BOLD);
    }
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case '1': case '2': case '3': case '4': case '5':
        scene_load(s, ch - '0');
        break;

    case 'g': case 'G':
        g_gravity = !g_gravity;
        break;

    case 'v': case 'V':
        g_viscosity = !g_viscosity;
        break;

    case 'r': case 'R':
        scene_load(s, s->id);
        break;

    case 'b': case 'B':
        add_blob(rand() % (g_phys_cols - 6) + 3, 3, 3);
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % N_THEMES;
        color_init(s->theme);
        break;

    case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '-':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            g_phys_cols      = app->screen.cols;
            g_phys_rows      = app->screen.rows - HUD_ROWS;
            frame_time       = clock_ns();
            sim_accum        = 0;
            app->need_resize = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── fixed-step sim accumulator ──────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha ───────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── fps counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
