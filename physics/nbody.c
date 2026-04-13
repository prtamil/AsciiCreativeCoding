/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nbody.c — N-Body Gravity Simulation
 *
 * Point masses with softened 1/r² gravity integrated with velocity Verlet.
 * Trails reveal orbital paths, slingshots, ejections, and figure-8
 * choreographies.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Force on body i from body j:
 *   F_ij = G · m_i · m_j · (r_j − r_i) / (|r_j − r_i|² + ε²)^(3/2)
 *
 * ε (softening) prevents 1/r² singularity when bodies pass close.
 *
 * Velocity Verlet integration (2nd-order symplectic):
 *   x_new = x + v·dt + ½·a·dt²
 *   compute a_new from x_new
 *   v_new = v + ½·(a + a_new)·dt
 *
 * Physics lives in pixel space (px, py).
 *   pw = cols × CELL_W,  ph = rows × CELL_H
 *   Body drawn at: px_to_cell_x(px), px_to_cell_y(py)
 *
 * Three presets:
 *   0  Galaxy     — 20 random bodies in a disc; random masses 1–4
 *   1  Black Hole — central mass=100 + 20 orbiting bodies mass=1
 *   2  Figure-8   — 3-body Chenciner-Montgomery choreography (equal masses)
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart current preset
 *   n / p     next / previous preset
 *   t         toggle trails
 *   ] / [     sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra nbody.c -o nbody -lncurses -lm
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

    MAX_BODIES      = 32,
    TRAIL_LEN       = 150,   /* trail ring-buffer length per body     */
    SUB_STEPS       = 4,     /* velocity-Verlet sub-steps per tick    */
    N_PRESETS       = 3,
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* Gravity constant (pixel³ / (mass · s²)) */
#define G_CONST      500000.0f
/* Softening squared (pixels²) — prevents singularity */
#define SOFT2        (40.0f * 40.0f)

/* Figure-8 length scale (pixels from screen centre) */
#define F8_SCALE     150.0f

/* Bounds multiplier: deactivate body if |pos| > EJECT_FACTOR × screen */
#define EJECT_FACTOR 2.5f

/* Cell dimensions for pixel↔cell conversion */
#define CELL_W  8
#define CELL_H  16

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

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Body, NBody                                               */
/* ===================================================================== */

/* ── Body ──────────────────────────────────────────────────────────── */

typedef struct {
    float  x,  y;           /* position (pixels)            */
    float  vx, vy;          /* velocity (pixels/s)           */
    float  ax, ay;          /* acceleration from last step   */
    float  mass;
    int    color;            /* 1-based color pair index      */

    /* trail ring-buffer */
    float  tx[TRAIL_LEN];
    float  ty[TRAIL_LEN];
    int    thead;            /* index of newest trail point   */
    int    tcount;

    bool   active;           /* false if ejected off-screen   */
} Body;

static void body_trail_push(Body *b, float x, float y)
{
    b->thead = (b->thead + 1) % TRAIL_LEN;
    b->tx[b->thead] = x;
    b->ty[b->thead] = y;
    if (b->tcount < TRAIL_LEN) b->tcount++;
}

/* ── NBody ─────────────────────────────────────────────────────────── */

typedef struct {
    Body  bodies[MAX_BODIES];
    int   n;
    bool  show_trails;
    bool  paused;
    int   preset;
} NBody;

/* ── Force computation O(n²) ───────────────────────────────────────── */

static void nbody_forces(Body *bodies, int n)
{
    for (int i = 0; i < n; i++) {
        bodies[i].ax = 0.0f;
        bodies[i].ay = 0.0f;
    }
    for (int i = 0; i < n; i++) {
        if (!bodies[i].active) continue;
        for (int j = i + 1; j < n; j++) {
            if (!bodies[j].active) continue;
            float dx  = bodies[j].x - bodies[i].x;
            float dy  = bodies[j].y - bodies[i].y;
            float r2  = dx*dx + dy*dy + SOFT2;
            float r   = sqrtf(r2);
            float inv = G_CONST / (r2 * r);   /* G / r³ */
            bodies[i].ax += inv * bodies[j].mass * dx;
            bodies[i].ay += inv * bodies[j].mass * dy;
            bodies[j].ax -= inv * bodies[i].mass * dx;
            bodies[j].ay -= inv * bodies[i].mass * dy;
        }
    }
}

/* ── Velocity Verlet step ───────────────────────────────────────────── */

static void nbody_step(NBody *nb, float dt, int cols, int rows)
{
    float ax_old[MAX_BODIES], ay_old[MAX_BODIES];

    /* save old accelerations */
    for (int i = 0; i < nb->n; i++) {
        ax_old[i] = nb->bodies[i].ax;
        ay_old[i] = nb->bodies[i].ay;
    }

    /* position update */
    float limit_x = (float)pw(cols) * EJECT_FACTOR;
    float limit_y = (float)ph(rows) * EJECT_FACTOR;

    for (int i = 0; i < nb->n; i++) {
        Body *b = &nb->bodies[i];
        if (!b->active) continue;
        b->x += b->vx * dt + 0.5f * ax_old[i] * dt * dt;
        b->y += b->vy * dt + 0.5f * ay_old[i] * dt * dt;

        /* eject check */
        if (fabsf(b->x) > limit_x || fabsf(b->y) > limit_y)
            b->active = false;
    }

    /* new accelerations */
    nbody_forces(nb->bodies, nb->n);

    /* velocity update */
    for (int i = 0; i < nb->n; i++) {
        Body *b = &nb->bodies[i];
        if (!b->active) continue;
        b->vx += 0.5f * (ax_old[i] + b->ax) * dt;
        b->vy += 0.5f * (ay_old[i] + b->ay) * dt;
    }
}

/* ── Preset initialisation ─────────────────────────────────────────── */

/*
 * Preset 0 — Galaxy
 * 20 random bodies (mass 1–4) placed in a disc.
 * Each given near-circular velocity around the system CoM.
 */
static void preset_galaxy(NBody *nb, int cols, int rows)
{
    nb->n = 20;
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;
    float R  = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.30f;

    for (int i = 0; i < nb->n; i++) {
        Body *b = &nb->bodies[i];
        /* random point in disc */
        float r   = R * sqrtf((float)rand() / (float)RAND_MAX);
        float ang = 2.0f * (float)M_PI * ((float)rand() / (float)RAND_MAX);
        b->x    = cx + r * cosf(ang);
        b->y    = cy + r * sinf(ang);
        b->mass = 1.0f + 3.0f * ((float)rand() / (float)RAND_MAX);
        b->color= (i % N_COLORS) + 1;
        b->active = true;
        b->thead = 0; b->tcount = 0;
        b->ax = 0; b->ay = 0;
    }

    /* estimate total mass for orbital velocity */
    float total_mass = 0;
    for (int i = 0; i < nb->n; i++) total_mass += nb->bodies[i].mass;

    for (int i = 0; i < nb->n; i++) {
        Body *b = &nb->bodies[i];
        float dx = b->x - cx;
        float dy = b->y - cy;
        float r  = sqrtf(dx*dx + dy*dy);
        if (r < 1.0f) { b->vx = 0; b->vy = 0; continue; }
        float v = sqrtf(G_CONST * total_mass / r) * 0.75f;
        b->vx = -dy / r * v;
        b->vy =  dx / r * v;
    }

    /* subtract CoM velocity */
    float pcx = 0, pcy = 0, total = 0;
    for (int i = 0; i < nb->n; i++) {
        pcx   += nb->bodies[i].vx * nb->bodies[i].mass;
        pcy   += nb->bodies[i].vy * nb->bodies[i].mass;
        total += nb->bodies[i].mass;
    }
    for (int i = 0; i < nb->n; i++) {
        nb->bodies[i].vx -= pcx / total;
        nb->bodies[i].vy -= pcy / total;
    }

    nbody_forces(nb->bodies, nb->n);
}

/*
 * Preset 1 — Black Hole
 * Central mass=100 at screen centre + 20 orbiting light bodies.
 */
static void preset_blackhole(NBody *nb, int cols, int rows)
{
    nb->n = 21;
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;

    /* body 0 = central black hole */
    Body *bh = &nb->bodies[0];
    bh->x = cx; bh->y = cy;
    bh->vx = 0; bh->vy = 0;
    bh->ax = 0; bh->ay = 0;
    bh->mass  = 100.0f;
    bh->color = 3;   /* yellow */
    bh->active = true;
    bh->thead = 0; bh->tcount = 0;

    float min_r = 100.0f;
    float max_r = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.28f;

    for (int i = 1; i < nb->n; i++) {
        Body *b = &nb->bodies[i];
        float r   = min_r + (max_r - min_r) * ((float)rand() / (float)RAND_MAX);
        float ang = 2.0f * (float)M_PI * ((float)rand() / (float)RAND_MAX);
        b->x    = cx + r * cosf(ang);
        b->y    = cy + r * sinf(ang);
        b->mass = 1.0f;
        b->color= ((i - 1) % (N_COLORS - 1)) + 1;
        b->active = true;
        b->thead = 0; b->tcount = 0;
        b->ax = 0; b->ay = 0;

        /* circular orbit around central mass */
        float v = sqrtf(G_CONST * bh->mass / r) * 0.98f;
        float dx = b->x - cx, dy = b->y - cy;
        b->vx = -dy / r * v;
        b->vy =  dx / r * v;
    }

    nbody_forces(nb->bodies, nb->n);
}

/*
 * Preset 2 — Figure-8
 * Chenciner-Montgomery 3-body choreography (equal masses).
 *
 * Natural units: G=1, m=1, positions in nat_len.
 * Rescaled to pixel space with:
 *   pos_scale = F8_SCALE  (pixels per natural length)
 *   time_scale = sqrt(pos_scale³ / G_CONST)
 *   vel_scale  = pos_scale / time_scale
 *
 * q1 = (-0.97000436, 0.24308753),  q3 = (0.97000436, -0.24308753)
 * v1 = v3 = ( 0.46620369,  0.43236573)
 * v2 = (-0.93240737, -0.86473146)
 */
static void preset_figure8(NBody *nb, int cols, int rows)
{
    nb->n = 3;
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;

    float L    = F8_SCALE;
    float ts   = sqrtf(L * L * L / G_CONST);  /* time scale (s/nat_time) */
    float vs   = L / ts;                        /* vel scale (px/s per nat_vel) */

    /* natural-unit ICs */
    float q1x = -0.97000436f, q1y =  0.24308753f;
    float q3x =  0.97000436f, q3y = -0.24308753f;
    float v1x =  0.46620369f, v1y =  0.43236573f;
    float v2x = -0.93240737f, v2y = -0.86473146f;

    Body *b0 = &nb->bodies[0];
    b0->x  = cx + q1x * L;  b0->y  = cy + q1y * L;
    b0->vx = v1x * vs;       b0->vy = v1y * vs;
    b0->mass = 1.0f; b0->color = 1; b0->active = true;
    b0->thead = 0; b0->tcount = 0; b0->ax = 0; b0->ay = 0;

    Body *b1 = &nb->bodies[1];
    b1->x  = cx;      b1->y  = cy;
    b1->vx = v2x*vs;  b1->vy = v2y*vs;
    b1->mass = 1.0f; b1->color = 4; b1->active = true;
    b1->thead = 0; b1->tcount = 0; b1->ax = 0; b1->ay = 0;

    Body *b2 = &nb->bodies[2];
    b2->x  = cx + q3x * L;  b2->y  = cy + q3y * L;
    b2->vx = v1x * vs;       b2->vy = v1y * vs;
    b2->mass = 1.0f; b2->color = 5; b2->active = true;
    b2->thead = 0; b2->tcount = 0; b2->ax = 0; b2->ay = 0;

    nbody_forces(nb->bodies, nb->n);
}

static const char *preset_names[N_PRESETS] = {
    "Galaxy", "Black Hole", "Figure-8"
};

static void nbody_init(NBody *nb, int preset, int cols, int rows)
{
    memset(nb, 0, sizeof *nb);
    nb->show_trails = true;
    nb->paused      = false;
    nb->preset      = preset;

    switch (preset) {
    default:
    case 0: preset_galaxy(nb, cols, rows);    break;
    case 1: preset_blackhole(nb, cols, rows); break;
    case 2: preset_figure8(nb, cols, rows);   break;
    }
}

static void nbody_tick(NBody *nb, float dt, int cols, int rows)
{
    if (nb->paused) return;

    float sub_dt = dt / (float)SUB_STEPS;
    for (int s = 0; s < SUB_STEPS; s++) {
        nbody_step(nb, sub_dt, cols, rows);
        /* record trail every sub-step */
        for (int i = 0; i < nb->n; i++) {
            if (nb->bodies[i].active)
                body_trail_push(&nb->bodies[i],
                                nb->bodies[i].x, nb->bodies[i].y);
        }
    }
}

static void nbody_draw(const NBody *nb, WINDOW *w, int cols, int rows)
{
    int active = 0;
    for (int i = 0; i < nb->n; i++) {
        const Body *b = &nb->bodies[i];
        if (!b->active) continue;
        active++;

        int cx = px_to_cell_x(b->x);
        int cy = px_to_cell_y(b->y);

        /* draw trail */
        if (nb->show_trails) {
            for (int k = 1; k < b->tcount; k++) {
                int idx = (b->thead - k + TRAIL_LEN) % TRAIL_LEN;
                int tx  = px_to_cell_x(b->tx[idx]);
                int ty  = px_to_cell_y(b->ty[idx]);
                if (tx < 0 || tx >= cols || ty < 1 || ty >= rows - 1) continue;
                float age  = (float)k / (float)TRAIL_LEN;
                chtype atr = (age < 0.4f) ? (chtype)COLOR_PAIR(b->color)
                                           : (chtype)(COLOR_PAIR(b->color) | A_DIM);
                wattron(w, atr);
                mvwaddch(w, ty, tx, '.');
                wattroff(w, atr);
            }
        }

        /* draw body */
        if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1) {
            char ch;
            if (b->mass > 50.0f)     ch = '@';
            else if (b->mass > 3.0f) ch = 'O';
            else if (b->mass > 1.5f) ch = 'o';
            else                      ch = '*';
            wattron(w, COLOR_PAIR(b->color) | A_BOLD);
            mvwaddch(w, cy, cx, ch);
            wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
        }
    }

    /* active body count HUD */
    wattron(w, COLOR_PAIR(4) | A_DIM);
    mvwprintw(w, rows - 3, 1, " bodies: %d ", active);
    wattroff(w, COLOR_PAIR(4) | A_DIM);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    NBody nbody;
    int   cols, rows;  /* last known terminal size (for restart) */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->cols = cols; s->rows = rows;
    memset(&s->nbody, 0, sizeof s->nbody);
    nbody_init(&s->nbody, 0, cols, rows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    s->cols = cols; s->rows = rows;
    nbody_tick(&s->nbody, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    nbody_draw(&s->nbody, w, cols, rows);
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

    const NBody *nb = &sc->nbody;

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  trails:%s ",
             fps, sim_fps, nb->show_trails ? "on " : "off");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(2) | A_BOLD);
    mvprintw(0, 1, " N-BODY: %s ", preset_names[nb->preset]);
    attroff(COLOR_PAIR(2) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:restart  n/p:preset  t:trails  [/]:Hz ");
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
    NBody  *nb = &app->scene.nbody;
    Screen *sc = &app->screen;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        nb->paused = !nb->paused;
        break;

    case 'r': case 'R':
        nbody_init(nb, nb->preset, sc->cols, sc->rows);
        break;

    case 'n': case 'N':
        nbody_init(nb, (nb->preset + 1) % N_PRESETS, sc->cols, sc->rows);
        break;

    case 'p': case 'P': {
        int p = nb->preset - 1;
        if (p < 0) p = N_PRESETS - 1;
        nbody_init(nb, p, sc->cols, sc->rows);
        break;
    }

    case 't': case 'T':
        nb->show_trails = !nb->show_trails;
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
