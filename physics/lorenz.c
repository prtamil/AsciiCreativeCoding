/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lorenz.c — Lorenz Strange Attractor
 *
 * Integrates the three Lorenz ODEs with RK4.  Two trajectories run in
 * parallel: a main trajectory and a ghost (ε-offset) that diverges on
 * the Lyapunov timescale, making chaos tangible.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Lorenz ODEs (σ=10, ρ=28, β=8/3):
 *   dx/dt = σ(y − x)
 *   dy/dt = x(ρ − z) − y
 *   dz/dt = xy − βz
 *
 * Integrated with RK4, step h=0.005 Lorenz-time units.
 * SUB_STEPS=8 sub-steps per sim tick → 0.04 Lorenz-time/tick.
 * At 60 ticks/s: 2.4 Lorenz-time/s advance.
 *
 * Lyapunov exponent λ≈0.9 → e-folding time ≈1.11 Lorenz-time.
 * Ghost ε-offset=0.01 reaches attractor size (~30) after ~8.9
 * Lyapunov times ≈ 3.7 s real-time.  Visible divergence within 4 s.
 *
 * PROJECTION
 * ─────────────────────────────────────────────────────────────────────
 * Orthographic with azimuth φ and elevation θ.
 * Attractor centered at (0,0,25) before projection.
 *   rx =  x·cosφ + y·sinφ
 *   ry = −x·sinφ + y·cosφ
 *   sx = rx
 *   sy = ry·cosθ + (z−25)·sinθ
 *   col = cx + sx·scale
 *   row = cy − sy·scale·ASPECT
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart both trajectories
 *   g         toggle ghost trajectory
 *   a         toggle auto-rotation
 *   ← →       azimuth (manual)
 *   ↑ ↓       elevation
 *   ] / [     sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lorenz.c -o lorenz -lncurses -lm
 */

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
    TRAIL_LEN       = 2500,   /* ring-buffer length per trajectory          */
    SUB_STEPS       = 8,      /* RK4 sub-steps per sim tick                 */
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* Lorenz parameters */
#define L_SIGMA   10.0f
#define L_RHO     28.0f
#define L_BETA    (8.0f / 3.0f)
#define L_H       0.005f    /* RK4 step (Lorenz time units)                */
#define GHOST_EPS 0.01f     /* initial ε-separation for ghost trajectory   */

/* View */
#define CELL_W  8
#define CELL_H  16
#define ASPECT  ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

#define VIEW_PHI_DEFAULT    0.5f    /* initial azimuth (rad)               */
#define VIEW_THETA_DEFAULT  0.55f   /* initial elevation (rad)             */
#define VIEW_PHI_SPEED      0.08f   /* auto-rotation speed (rad/s)         */

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
        init_pair(1, 196, COLOR_BLACK);   /* red     */
        init_pair(2, 208, COLOR_BLACK);   /* orange  */
        init_pair(3, 226, COLOR_BLACK);   /* yellow  */
        init_pair(4,  46, COLOR_BLACK);   /* green   */
        init_pair(5,  51, COLOR_BLACK);   /* cyan    */
        init_pair(6,  75, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
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
/* §4  coords — orthographic 3-D → 2-D projection                        */
/* ===================================================================== */

/*
 * project() — map a Lorenz-space point to terminal (col, row).
 *
 * The attractor is centred at (0,0,25) before projection.
 * Returns false if the point falls outside the terminal.
 */
static bool project(float lx, float ly, float lz,
                    float phi, float theta, float scale,
                    int cx, int cy, int cols, int rows,
                    int *out_col, int *out_row)
{
    float px = lx;
    float py = ly;
    float pz = lz - 25.0f;   /* centre attractor on z=25 */

    /* azimuth rotation */
    float rx =  px * cosf(phi) + py * sinf(phi);
    float ry = -px * sinf(phi) + py * cosf(phi);

    /* elevation rotation → screen coords */
    float sx = rx;
    float sy = ry * cosf(theta) + pz * sinf(theta);

    int col = cx + (int)(sx * scale);
    int row = cy - (int)(sy * scale * ASPECT);

    *out_col = col;
    *out_row = row;
    return (col >= 0 && col < cols && row >= 1 && row < rows - 1);
}

/* ===================================================================== */
/* §5  entity — Lorenz                                                    */
/* ===================================================================== */

/* ── Ring-buffer trail ─────────────────────────────────────────────── */

typedef struct {
    float x[TRAIL_LEN];
    float y[TRAIL_LEN];
    float z[TRAIL_LEN];
    int   head;   /* index of newest point */
    int   count;  /* number of valid points (≤ TRAIL_LEN) */
} Trail;

static void trail_push(Trail *t, float x, float y, float z)
{
    t->head = (t->head + 1) % TRAIL_LEN;
    t->x[t->head] = x;
    t->y[t->head] = y;
    t->z[t->head] = z;
    if (t->count < TRAIL_LEN) t->count++;
}

static void trail_clear(Trail *t)
{
    t->head  = 0;
    t->count = 0;
}

/* ── Lorenz ODE ────────────────────────────────────────────────────── */

static void lorenz_deriv(float x, float y, float z,
                          float *dx, float *dy, float *dz)
{
    *dx = L_SIGMA * (y - x);
    *dy = x * (L_RHO - z) - y;
    *dz = x * y - L_BETA * z;
}

static void lorenz_rk4(float *x, float *y, float *z, float h)
{
    float k1x, k1y, k1z;
    float k2x, k2y, k2z;
    float k3x, k3y, k3z;
    float k4x, k4y, k4z;

    lorenz_deriv(*x, *y, *z, &k1x, &k1y, &k1z);
    lorenz_deriv(*x + h*0.5f*k1x, *y + h*0.5f*k1y, *z + h*0.5f*k1z,
                 &k2x, &k2y, &k2z);
    lorenz_deriv(*x + h*0.5f*k2x, *y + h*0.5f*k2y, *z + h*0.5f*k2z,
                 &k3x, &k3y, &k3z);
    lorenz_deriv(*x + h*k3x, *y + h*k3y, *z + h*k3z,
                 &k4x, &k4y, &k4z);

    *x += (h / 6.0f) * (k1x + 2.0f*k2x + 2.0f*k3x + k4x);
    *y += (h / 6.0f) * (k1y + 2.0f*k2y + 2.0f*k3y + k4y);
    *z += (h / 6.0f) * (k1z + 2.0f*k2z + 2.0f*k3z + k4z);
}

/* ── Lorenz entity ─────────────────────────────────────────────────── */

typedef struct {
    /* main trajectory */
    float  mx, my, mz;
    Trail  mt;
    /* ghost trajectory (ε-offset) */
    float  gx, gy, gz;
    Trail  gt;
    /* view */
    float  phi;
    float  theta;
    bool   auto_rotate;
    bool   show_ghost;
    bool   paused;
} Lorenz;

static void lorenz_init(Lorenz *l)
{
    /* start near (1,1,1) — approximately on the attractor */
    l->mx = 1.0f;  l->my = 1.0f;  l->mz = 1.0f;
    l->gx = 1.0f + GHOST_EPS;
    l->gy = 1.0f;
    l->gz = 1.0f;
    trail_clear(&l->mt);
    trail_clear(&l->gt);
    trail_push(&l->mt, l->mx, l->my, l->mz);
    trail_push(&l->gt, l->gx, l->gy, l->gz);

    l->phi         = VIEW_PHI_DEFAULT;
    l->theta       = VIEW_THETA_DEFAULT;
    l->auto_rotate = true;
    l->show_ghost  = true;
    l->paused      = false;
}

static void lorenz_tick(Lorenz *l, float dt)
{
    if (l->paused) return;

    if (l->auto_rotate)
        l->phi += VIEW_PHI_SPEED * dt;

    for (int s = 0; s < SUB_STEPS; s++) {
        lorenz_rk4(&l->mx, &l->my, &l->mz, L_H);
        lorenz_rk4(&l->gx, &l->gy, &l->gz, L_H);
        trail_push(&l->mt, l->mx, l->my, l->mz);
        trail_push(&l->gt, l->gx, l->gy, l->gz);
    }
}

/*
 * lorenz_draw_trail — render one trail with age-based colour.
 *
 * Iterates from newest (age=0) to oldest (age=1).
 * Skips duplicate projected cells to bound draw calls.
 *
 * Main trail colour ramp: red-bold → red → orange → yellow-dim → dim
 * Ghost trail: all magenta-dim
 */
static void lorenz_draw_trail(const Trail *t, bool is_ghost,
                               float phi, float theta, float scale,
                               int cx, int cy, int cols, int rows,
                               WINDOW *w)
{
    int last_col = -999, last_row = -999;

    for (int k = 0; k < t->count; k++) {
        /* newest point first */
        int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
        float age = (float)k / (float)(t->count > 1 ? t->count - 1 : 1);

        int col, row;
        if (!project(t->x[idx], t->y[idx], t->z[idx],
                     phi, theta, scale, cx, cy, cols, rows, &col, &row))
            continue;

        /* skip duplicate cell */
        if (col == last_col && row == last_row) continue;
        last_col = col;
        last_row = row;

        chtype attr;
        if (is_ghost) {
            attr = COLOR_PAIR(7) | A_DIM;
        } else if (age < 0.15f) {
            attr = COLOR_PAIR(1) | A_BOLD;
        } else if (age < 0.30f) {
            attr = COLOR_PAIR(1);
        } else if (age < 0.55f) {
            attr = COLOR_PAIR(2);
        } else if (age < 0.75f) {
            attr = COLOR_PAIR(3) | A_DIM;
        } else {
            attr = A_DIM;
        }

        char ch = is_ghost ? ',' : '.';
        if (k == 0) ch = is_ghost ? 'x' : 'O';   /* head marker */

        wattron(w, attr);
        mvwaddch(w, row, col, ch);
        wattroff(w, attr);
    }
}

static void lorenz_draw(const Lorenz *l, WINDOW *w, int cols, int rows)
{
    int cx = cols / 2;
    int cy = rows / 2;

    /*
     * Scale: fill ~80% of usable height; clip horizontally at boundaries.
     * Max projected |sy| ≈ 39 at default elevation.
     */
    float usable = (float)(rows - 4);
    float scale  = usable * 0.80f / (39.0f * ASPECT);

    /* ghost drawn first (underneath main) */
    if (l->show_ghost)
        lorenz_draw_trail(&l->gt, true,  l->phi, l->theta, scale,
                          cx, cy, cols, rows, w);

    lorenz_draw_trail(&l->mt, false, l->phi, l->theta, scale,
                      cx, cy, cols, rows, w);

    /* separation indicator: distance between heads */
    float dx = l->gx - l->mx;
    float dy = l->gy - l->my;
    float dz = l->gz - l->mz;
    float sep = sqrtf(dx*dx + dy*dy + dz*dz);

    wattron(w, COLOR_PAIR(4) | A_DIM);
    mvwprintw(w, rows - 3, 1, " ε-sep: %.4f ", sep);
    wattroff(w, COLOR_PAIR(4) | A_DIM);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Lorenz lorenz;
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    lorenz_init(&s->lorenz);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    lorenz_tick(&s->lorenz, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    lorenz_draw(&s->lorenz, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
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
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Lorenz *l = &sc->lorenz;

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  ghost:%s  rot:%s ",
             fps, sim_fps,
             l->show_ghost  ? "on " : "off",
             l->auto_rotate ? "auto" : "manual");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, 1, " LORENZ ATTRACTOR ");
    attroff(COLOR_PAIR(1) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:restart  g:ghost  a:auto-rot  arrows:view  [/]:Hz ");
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

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Lorenz *l = &app->scene.lorenz;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        l->paused = !l->paused;
        break;

    case 'r': case 'R':
        lorenz_init(l);
        break;

    case 'g': case 'G':
        l->show_ghost = !l->show_ghost;
        break;

    case 'a': case 'A':
        l->auto_rotate = !l->auto_rotate;
        break;

    case KEY_LEFT:
        l->auto_rotate = false;
        l->phi -= 0.1f;
        break;
    case KEY_RIGHT:
        l->auto_rotate = false;
        l->phi += 0.1f;
        break;

    case KEY_UP:
        l->theta += 0.05f;
        if (l->theta > 1.4f) l->theta = 1.4f;
        break;
    case KEY_DOWN:
        l->theta -= 0.05f;
        if (l->theta < 0.1f) l->theta = 0.1f;
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
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
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
