/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voronoi.c — Animated Voronoi Diagram
 *
 * N_SEEDS seed points drift with Langevin Brownian motion and bounce off
 * the screen boundary.  Each frame every terminal cell is coloured by its
 * nearest seed (brute-force O(cells × seeds) nearest-neighbour search).
 *
 * PHYSICS
 * ───────
 * Langevin equation for each seed:
 *   dv/dt = −γ·v + σ·ξ      (ξ = uniform random in [−1,1])
 * Discrete update per tick:
 *   v += (−DAMP·v + NOISE·ξ) · dt
 *   p += v · dt
 * This gives self-limiting Brownian motion (terminal speed ≈ NOISE/DAMP).
 *
 * DRAWING
 * ───────
 * For each cell centre (px, py) in pixel space:
 *   • find d1 (nearest seed distance) and d2 (second nearest)
 *   • if d2 − d1 < BORDER_PX → border cell → draw '+' at normal brightness
 *   • if d1 < SEED_PX         → seed centre  → draw 'O' bold
 *   • otherwise                → interior    → draw '.' dim
 * All cells coloured with their nearest seed's colour pair.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset seeds   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra voronoi.c -o voronoi -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Brute-force Voronoi diagram — O(cells × seeds) per frame.
 *                  For each cell, scan all seeds and find the nearest.
 *                  Efficient for small N (N_SEEDS ≤ 30); for larger N, a
 *                  Fortune's sweep-line algorithm gives O(N log N).
 *
 * Math           : The Voronoi diagram partitions the plane into regions where
 *                  each region is the set of points closer to one seed than all
 *                  others.  The dual graph of the Voronoi diagram is the Delaunay
 *                  triangulation (every Voronoi edge connects the circumcentres
 *                  of two Delaunay triangles).
 *                  Border detection: cell is a border cell when the distance to
 *                  the nearest seed and second-nearest differ by less than BORDER_PX.
 *                  This approximates the Voronoi edge without exact line computation.
 *
 * Physics        : Seeds move under the Langevin equation:
 *                    dv/dt = −γ·v + σ·ξ  (ξ = white noise)
 *                  This is Ornstein-Uhlenbeck process — Brownian motion with
 *                  mean-reverting velocity (terminal speed = NOISE/DAMP).
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    N_SEEDS         = 24,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions (logical sub-pixel spacing) */
#define CELL_W  8
#define CELL_H  16

/* Langevin motion parameters */
#define DAMP     2.0f    /* velocity damping coefficient (s⁻¹)             */
#define NOISE   60.0f    /* random force amplitude (px/s per √s)           */

/* Drawing thresholds in pixels */
#define BORDER_PX  15.0f  /* d2−d1 threshold for border cell              */
#define SEED_PX    12.0f  /* d1 threshold for seed-centre cell             */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  75, COLOR_BLACK);
        init_pair(7, 201, COLOR_BLACK);
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline float pw(int cols) { return (float)cols * CELL_W; }
static inline float ph(int rows) { return (float)rows * CELL_H; }

/* ===================================================================== */
/* §5  entity — Seed, Voronoi                                             */
/* ===================================================================== */

typedef struct {
    float px, py;   /* position in pixel space                            */
    float vx, vy;   /* velocity (px/s)                                    */
    int   pair;     /* colour pair 1–7                                    */
} Seed;

typedef struct {
    Seed  seeds[N_SEEDS];
    bool  paused;
} Voronoi;

/* randf — uniform float in [−1, 1] */
static float randf(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

static void voronoi_reset(Voronoi *v, int cols, int rows)
{
    float W = pw(cols);
    float H = ph(rows);
    float mx = (float)CELL_W * 3;
    float my = (float)CELL_H * 2;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s  = &v->seeds[i];
        s->px    = mx + (float)rand() / RAND_MAX * (W - 2*mx);
        s->py    = my + (float)rand() / RAND_MAX * (H - 2*my);
        s->vx    = randf() * 20.0f;
        s->vy    = randf() * 20.0f;
        s->pair  = (i % N_COLORS) + 1;
    }
    /* Shuffle colour assignments */
    for (int i = N_SEEDS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = v->seeds[i].pair;
        v->seeds[i].pair = v->seeds[j].pair;
        v->seeds[j].pair = t;
    }
}

static void voronoi_init(Voronoi *v, int cols, int rows)
{
    memset(v, 0, sizeof *v);
    voronoi_reset(v, cols, rows);
}

/*
 * voronoi_tick — Langevin motion + wall bounce.
 *
 * Each seed undergoes over-damped Brownian motion:
 *   v += (−DAMP·v + NOISE·ξ) · dt
 * A soft inward force near walls prevents escape:
 *   if px < margin: vx += WALL_PUSH · dt
 */
static void voronoi_tick(Voronoi *v, float dt, int cols, int rows)
{
    if (v->paused) return;

    float W   = pw(cols);
    float H   = ph(rows);
    float mxL = (float)CELL_W * 2;
    float myT = (float)CELL_H * 2;
    float mxR = W - mxL;
    float myB = H - myT;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s = &v->seeds[i];

        /* Langevin: damp + random kick */
        s->vx += (-DAMP * s->vx + NOISE * randf()) * dt;
        s->vy += (-DAMP * s->vy + NOISE * randf()) * dt;

        /* Integrate */
        s->px += s->vx * dt;
        s->py += s->vy * dt;

        /* Bounce off walls */
        if (s->px < mxL) { s->px = mxL; s->vx =  fabsf(s->vx); }
        if (s->px > mxR) { s->px = mxR; s->vx = -fabsf(s->vx); }
        if (s->py < myT) { s->py = myT; s->vy =  fabsf(s->vy); }
        if (s->py > myB) { s->py = myB; s->vy = -fabsf(s->vy); }
    }
}

/*
 * voronoi_draw — per-cell nearest-seed search and rendering.
 *
 * Cell centre pixel: (col·CELL_W + CELL_W/2, row·CELL_H + CELL_H/2).
 * Distance uses pixel-space Euclidean metric so Voronoi regions have
 * correct proportions (not distorted by terminal aspect ratio).
 */
static void voronoi_draw(const Voronoi *v, WINDOW *w, int cols, int rows)
{
    float half_cw = (float)CELL_W * 0.5f;
    float half_ch = (float)CELL_H * 0.5f;

    for (int row = 1; row < rows - 1; row++) {
        float cy = (float)row * CELL_H + half_ch;

        for (int col = 0; col < cols; col++) {
            float cx = (float)col * CELL_W + half_cw;

            float d1 = 1e18f, d2 = 1e18f;
            int   best = 0;

            for (int k = 0; k < N_SEEDS; k++) {
                float dx = cx - v->seeds[k].px;
                float dy = cy - v->seeds[k].py;
                float d  = dx*dx + dy*dy;   /* compare squared distances */
                if (d < d1) { d2 = d1; d1 = d; best = k; }
                else if (d < d2) { d2 = d; }
            }

            d1 = sqrtf(d1);
            d2 = sqrtf(d2);

            int   pair = v->seeds[best].pair;
            chtype attr;
            char   ch;

            if (d1 < SEED_PX) {
                ch   = 'O';
                attr = A_BOLD;
            } else if (d2 - d1 < BORDER_PX) {
                ch   = '+';
                attr = 0;
            } else {
                ch   = '.';
                attr = A_DIM;
            }

            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Voronoi voronoi; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    voronoi_init(&s->voronoi, cols, rows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    voronoi_tick(&s->voronoi, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    voronoi_draw(&s->voronoi, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  seeds:%d  %s ",
             fps, sim_fps, N_SEEDS,
             sc->voronoi.paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " VORONOI ");
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    Voronoi *v = &app->scene.voronoi;
    Screen  *s = &app->screen;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': v->paused = !v->paused; break;
    case 'r': case 'R':
        voronoi_reset(v, s->cols, s->rows);
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
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
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            voronoi_reset(&app->scene.voronoi,
                          app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
