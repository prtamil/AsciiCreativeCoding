/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spirograph.c — Spirograph (Hypotrochoid)
 *
 * Three simultaneous hypotrochoid curves in different colours.  Each curve
 * traces a decaying trail on a per-cell float canvas; the canvas fades each
 * tick so older parts of the curve dim and eventually vanish, letting the
 * current trace shine.  Parameters drift slowly so the pattern evolves.
 *
 * PARAMETRIC FORMULA (hypotrochoid)
 * ───────────────────────────────────
 *   x(t) = (R − r)·cos(t)  +  d·cos((R−r)/r · t)
 *   y(t) = (R − r)·sin(t)  −  d·sin((R−r)/r · t)
 *
 * R = outer radius, r = rolling (inner) radius, d = pen distance from centre.
 * The ratio R/r determines the number of petals; d sets petal size.
 *
 * SLOW DRIFT
 * ──────────
 * r drifts sinusoidally between r_min and r_max at DRIFT_RATE rad/s,
 * gradually shifting between two distinct patterns without any keypress.
 *
 * CANVAS / TRAIL
 * ──────────────
 * float canvas[MAX_ROWS][MAX_COLS]  stores brightness [0,1] per cell.
 * int   cpair[MAX_ROWS][MAX_COLS]   stores the colour pair last written.
 *
 * Each tick:
 *   1. canvas *= FADE  (global brightness decay)
 *   2. For each curve: trace TRACE_STEPS points along the parameter
 *      interval [t, t+DELTA_T) and stamp canvas cells at brightness 1.0.
 *   3. t += DELTA_T.
 *
 * Scale so the largest curve fits within 85% of the shorter screen dimension.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset canvas   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra spirograph.c -o spirograph -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Parametric curve tracing with a float canvas + fade.
 *                  Rather than drawing discrete points, each curve advances
 *                  T_STEPS_PER_FRAME timesteps per tick, writing brightness
 *                  values to a float canvas.  The canvas decays by FADE_RATE
 *                  per tick, creating the fading trail effect.
 *
 * Math           : Hypotrochoid parametric equations:
 *                    x(t) = (R−r)·cos(t) + d·cos((R−r)/r · t)
 *                    y(t) = (R−r)·sin(t) − d·sin((R−r)/r · t)
 *                  The curve closes when (R−r)/r = p/q (rational ratio).
 *                  With p=3, q=1: deltoid (3-cusp astroid).
 *                  Irrational ratios → the curve never closes (dense in annulus).
 *                  Pen distance d controls petal amplitude: d<(R−r) → inside,
 *                  d>(R−r) → outside → loops at each reversal point.
 *
 * Rendering      : Three simultaneous curves with different r values.
 *                  r drifts sinusoidally (DRIFT_RATE), creating continuous
 *                  morphing between distinct petal patterns without keystrokes.
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
    N_CURVES        = 3,
    MAX_ROWS        = 80,
    MAX_COLS        = 240,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W   8
#define CELL_H   16

/*
 * FADE — canvas brightness multiplier per tick.
 * At 60 Hz: 0.985^60 ≈ 0.40 → trail visible for ~1.5 seconds.
 */
#define FADE        0.985f

/*
 * DELTA_T — parameter advance per tick (radians).
 * TRACE_STEPS sub-samples per tick fill the curve without gaps.
 */
#define DELTA_T     0.08f
#define TRACE_STEPS 60

/* Slow parameter drift */
#define DRIFT_RATE  0.04f   /* rad/s — one full drift cycle every ~157 s  */

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
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int px_to_col(float px) { return (int)floorf(px / CELL_W + 0.5f); }
static inline int px_to_row(float py) { return (int)floorf(py / CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  entity — Curve, Spirograph                                         */
/* ===================================================================== */

/*
 * Curve parameters.
 * R, r_base, r_amp describe the rolling radii:
 *   r_actual(t) = r_base + r_amp · sin(drift_phase)
 * d is the pen offset.  All in "unit" scale; multiplied by scale at draw time.
 */
typedef struct {
    float R;        /* outer radius (unit scale)                          */
    float r_base;   /* inner radius base (unit scale)                     */
    float r_amp;    /* drift amplitude for r                              */
    float d;        /* pen offset (unit scale)                            */
    float t;        /* current parameter value (radians)                  */
    float drift;    /* drift phase (radians), advances by DRIFT_RATE·dt   */
    int   pair;     /* ncurses colour pair                                */
} Curve;

typedef struct {
    float  canvas[MAX_ROWS][MAX_COLS];   /* brightness per cell [0,1]     */
    int    cpair[MAX_ROWS][MAX_COLS];    /* colour pair per cell           */
    Curve  curves[N_CURVES];
    bool   paused;
} Spirograph;

/*
 * Three presets (unit scale, approximately normalised so max extent ≈ 1):
 *   curve 0: R=5, r≈3, d=5.5 → 5-lobed  (cyan)
 *   curve 1: R=7, r≈2, d=7.0 → 7-lobed  (magenta)
 *   curve 2: R=6, r≈4, d=6.0 → 3-lobed  (yellow)
 * Curves are staggered in parameter phase by 2π/3 so they overlap nicely.
 */
static void spirograph_reset_curves(Spirograph *sg)
{
    sg->curves[0] = (Curve){ 5.0f, 3.0f, 0.8f, 5.5f, 0.0f,               0.0f, 5 };
    sg->curves[1] = (Curve){ 7.0f, 2.0f, 0.6f, 7.0f, 2.0f*(float)M_PI/3.0f, 1.0f, 7 };
    sg->curves[2] = (Curve){ 6.0f, 4.0f, 0.9f, 6.0f, 4.0f*(float)M_PI/3.0f, 2.0f, 3 };
}

static void spirograph_init(Spirograph *sg)
{
    memset(sg, 0, sizeof *sg);
    spirograph_reset_curves(sg);
}

static void spirograph_clear_canvas(Spirograph *sg)
{
    memset(sg->canvas, 0, sizeof sg->canvas);
}

/*
 * spirograph_tick — advance one physics step.
 *
 * 1. Fade all canvas values by FADE.
 * 2. For each curve: trace TRACE_STEPS sub-steps across DELTA_T, stamp
 *    the canvas at each point with brightness 1.0 and the curve's colour.
 * 3. Advance t and drift.
 */
static void spirograph_tick(Spirograph *sg, float dt, int cols, int rows)
{
    if (sg->paused) return;

    /* Compute scale: largest curve has R+d ≈ 12.5 units.
     * Fit 85% of the shorter screen dimension. */
    float screen_px = (float)((cols * CELL_W < rows * CELL_H)
                               ? cols * CELL_W : rows * CELL_H);
    float max_extent = 12.5f;
    float scale = screen_px * 0.425f / max_extent;  /* 0.85/2 per half-screen */

    float cx_px = (float)(cols * CELL_W) * 0.5f;
    float cy_px = (float)(rows * CELL_H) * 0.5f;

    /* 1. Fade canvas */
    for (int r = 0; r < rows && r < MAX_ROWS; r++)
        for (int c = 0; c < cols && c < MAX_COLS; c++)
            sg->canvas[r][c] *= FADE;

    /* 2. Trace each curve */
    for (int ci = 0; ci < N_CURVES; ci++) {
        Curve *cv = &sg->curves[ci];

        float r_actual = cv->r_base + cv->r_amp * sinf(cv->drift);
        if (r_actual < 0.5f) r_actual = 0.5f;   /* prevent degenerate cases */

        float Rmr  = cv->R - r_actual;   /* R − r */
        float ratio = Rmr / r_actual;    /* (R−r)/r */

        for (int step = 0; step < TRACE_STEPS; step++) {
            float tt = cv->t + (float)step * (DELTA_T / (float)TRACE_STEPS);

            float x = Rmr * cosf(tt) + cv->d * cosf(ratio * tt);
            float y = Rmr * sinf(tt) - cv->d * sinf(ratio * tt);

            /* pixel position */
            float px = cx_px + x * scale;
            float py = cy_px + y * scale;

            int col = px_to_col(px);
            int row = px_to_row(py);

            /* skip row 0 (HUD) and last row (key hint) */
            if (col < 0 || col >= cols || col >= MAX_COLS) continue;
            if (row < 1 || row >= rows - 1 || row >= MAX_ROWS) continue;

            sg->canvas[row][col] = 1.0f;
            sg->cpair[row][col]  = cv->pair;
        }

        cv->t     += DELTA_T;
        cv->drift += DRIFT_RATE * dt;
    }
}

static void spirograph_draw(const Spirograph *sg, WINDOW *w, int cols, int rows)
{
    static const char CHARS[] = " .,:+*#@";
    for (int row = 1; row < rows - 1 && row < MAX_ROWS; row++) {
        for (int col = 0; col < cols && col < MAX_COLS; col++) {
            float b = sg->canvas[row][col];
            if (b < 0.04f) continue;

            int pair = sg->cpair[row][col];
            if (pair < 1 || pair > 7) pair = 5;

            int ci = (int)(b * 7.0f);
            if (ci < 0) ci = 0;
            if (ci > 7) ci = 7;
            char ch = CHARS[ci];

            chtype attr = (b > 0.7f) ? A_BOLD : (b < 0.25f) ? A_DIM : 0;

            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Spirograph sg; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    spirograph_init(&s->sg);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    spirograph_tick(&s->sg, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    spirograph_draw(&s->sg, w, cols, rows);
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
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
             fps, sim_fps, sc->sg.paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " SPIROGRAPH ");
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
    Spirograph *sg = &app->scene.sg;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sg->paused = !sg->paused; break;
    case 'r': case 'R':
        spirograph_reset_curves(sg);
        spirograph_clear_canvas(sg);
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
            spirograph_clear_canvas(&app->scene.sg);
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
