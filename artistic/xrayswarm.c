/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * xrayswarm.c — ray swarm simulation
 *
 * Swarms of particles: each swarm has a wandering queen (@) and workers (*)
 * that shoot outward as rays, park at the screen edge, then fly back in.
 * Fading directional trails give a bright-ray-on-dark look.
 * Phase cycle: DIVERGE → PAUSE (retract) → CONVERGE → PAUSE → repeat.
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   r         reset
 *   + / -     add / remove a swarm (1–5)
 *   [ / ]     lower / raise simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       artistic/xrayswarm.c \
 *       -o xrayswarm -lncurses -lm
 *
 * §1 config   §2 clock   §3 color   §4 coords   §5 entity
 * §6 scene    §7 screen  §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Swarm simulation with 4-phase state machine per worker.
 *                  DIVERGE: workers shoot outward from queen along fixed
 *                  headings; PAUSE (retract): workers brake and hold; then
 *                  CONVERGE: workers fly back toward queen; second PAUSE;
 *                  repeat.  Queen wanders with smooth Brownian-like steering.
 *
 * Data-structure : Per-worker ring buffer of TRAIL_LEN=48 past positions.
 *                  Older trail positions drawn at decreasing brightness,
 *                  creating the long fading ray appearance.  Buffer is a
 *                  circular array with head pointer — O(1) append.
 *
 * Physics        : Queen steering: each tick a small random angle δθ is
 *                  added to the heading, bounded to avoid tight circles
 *                  (Ornstein-Uhlenbeck-like bounded random walk).
 *                  Worker heading locked to outward direction from queen
 *                  at the start of DIVERGE; held fixed during flight.
 *
 * Rendering      : Cell-aspect correction: CELL_W=8, CELL_H=16 — worker
 *                  pixel positions are divided by (CELL_H/CELL_W) in y to
 *                  produce equal angular spacing on screen.  Trail chars
 *                  selected from brightness ramp based on trail age.
 *
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

#define CELL_W   8
#define CELL_H  16

enum {
    N_SWARMS_DEFAULT = 3,
    N_SWARMS_MAX     = 5,
    N_WORKERS        = 20,    /* workers per swarm                        */
    TRAIL_LEN        = 48,    /* history slots per bug — long = ray look  */

    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MIN      = 10,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     = 10,

    FPS_UPDATE_MS    = 500,
};

/* Queen */
#define QUEEN_JITTER   35.0f
#define QUEEN_DAMP     0.96f
#define QUEEN_SPEED    80.0f

/* Workers — high inertia = straight-line rays */
#define WORK_JITTER    4.0f
#define WORK_DAMP      0.994f
#define WORK_SPEED    380.0f

/* Phase durations — timer drives transitions, not worker state */
#define DIVERGE_DUR    3.5f    /* seconds rays shoot outward              */
#define CONVERGE_DUR   3.5f    /* seconds rays fly inward                 */
#define PAUSE_DUR      0.7f    /* seconds between phases (trails retract) */
#define ARRIVE_DIST   40.0f    /* px — worker "arrives" at queen          */
#define CONVERGE_STEER 5.0f    /* homing correction strength (per second) */

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
 * One color pair per swarm (up to 5) + one for the HUD.
 * 256-color: vivid saturated hues; 8-color: basic fallback.
 */
enum {
    CP_S0 = 1,   /* swarm 0 — cyan    */
    CP_S1,       /* swarm 1 — green   */
    CP_S2,       /* swarm 2 — yellow  */
    CP_S3,       /* swarm 3 — magenta */
    CP_S4,       /* swarm 4 — red     */
    CP_HUD,      /* status bar        */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_S0,  51,  -1);   /* cyan    */
        init_pair(CP_S1,  46,  -1);   /* green   */
        init_pair(CP_S2, 226,  -1);   /* yellow  */
        init_pair(CP_S3, 201,  -1);   /* magenta */
        init_pair(CP_S4, 196,  -1);   /* red     */
        init_pair(CP_HUD, 244, -1);   /* grey    */
    } else {
        init_pair(CP_S0, COLOR_CYAN,    -1);
        init_pair(CP_S1, COLOR_GREEN,   -1);
        init_pair(CP_S2, COLOR_YELLOW,  -1);
        init_pair(CP_S3, COLOR_MAGENTA, -1);
        init_pair(CP_S4, COLOR_RED,     -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

static const int swarm_colors[N_SWARMS_MAX] = {
    CP_S0, CP_S1, CP_S2, CP_S3, CP_S4
};

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity                                                             */
/* ===================================================================== */

/*
 * SwarmState — three-phase cycle per swarm:
 *
 *   ST_DIVERGE  rays shoot outward; workers exit screen → instantly respawn
 *               at queen so new rays fire continuously.  Phase ends on timer.
 *   ST_PAUSE    trails shrink to nothing; queen still wanders
 *   ST_CONVERGE rays home in from edges; workers arrive → instantly respawn
 *               at a new random edge.  Phase ends on timer.
 *
 *   DIVERGE → (DIVERGE_DUR s) → PAUSE → CONVERGE → (CONVERGE_DUR s) → PAUSE → …
 */
typedef enum { ST_DIVERGE, ST_PAUSE, ST_CONVERGE } SwarmState;

typedef struct {
    float px, py;
    float vx, vy;
    float tpx[TRAIL_LEN];
    float tpy[TRAIL_LEN];
    int   thead;
    int   tlen;
    float park_px, park_py;   /* position where this worker parked at screen edge */
} Bug;

typedef struct {
    Bug        queen;
    Bug        workers[N_WORKERS];
    int        cp;
    SwarmState state;
    SwarmState next_state;
    float      phase_timer;   /* seconds elapsed in current active phase   */
    float      pause_timer;
    float      locked_qx, locked_qy; /* queen pos locked at start of CONVERGE */
} Swarm;

/* ── helpers ── */

static float randf(void) { return (float)rand() / (float)RAND_MAX; }
static float randc(void) { return randf() * 2.0f - 1.0f; }

static void trail_push(Bug *b, float px, float py)
{
    b->tpx[b->thead] = px;
    b->tpy[b->thead] = py;
    b->thead = (b->thead + 1) % TRAIL_LEN;
    if (b->tlen < TRAIL_LEN) b->tlen++;
}

static void trail_clear(Bug *b) { b->thead = 0; b->tlen = 0; }

static void clamp_speed(Bug *b, float max_spd)
{
    float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (spd > max_spd && spd > 0.0f) {
        float s = max_spd / spd;
        b->vx *= s; b->vy *= s;
    }
}


/* ── diverge phase ── */

/*
 * Launch a worker outward from the queen.
 * index used to evenly stagger launch angles across N_WORKERS.
 */
static void worker_launch_diverge(Bug *w, const Bug *q, int index)
{
    w->px = q->px;
    w->py = q->py;
    float angle = (float)index * (2.0f * 3.14159265f / (float)N_WORKERS)
                + randf() * 0.5f;
    float spd = WORK_SPEED * (0.55f + 0.45f * randf());
    w->vx = cosf(angle) * spd;
    w->vy = sinf(angle) * spd;
    trail_clear(w);
}

static void worker_diverge_tick(Bug *w, float dt, float max_px, float max_py)
{
    /* Already parked at screen edge — wait for PAUSE to retract the trail */
    if (w->vx == 0.0f && w->vy == 0.0f) return;

    w->vx += randc() * WORK_JITTER * dt;
    w->vy += randc() * WORK_JITTER * dt;
    w->vx *= WORK_DAMP;
    w->vy *= WORK_DAMP;
    clamp_speed(w, WORK_SPEED);
    trail_push(w, w->px, w->py);
    w->px += w->vx * dt;
    w->py += w->vy * dt;
    /* Exited screen → park at edge; timer will trigger the PAUSE transition */
    if (w->px < 0.0f || w->px > max_px ||
        w->py < 0.0f || w->py > max_py) {
        if (w->px < 0.0f)    w->px = 0.0f;
        if (w->px > max_px)  w->px = max_px;
        if (w->py < 0.0f)    w->py = 0.0f;
        if (w->py > max_py)  w->py = max_py;
        w->vx = 0.0f;
        w->vy = 0.0f;
    }
}

/* ── converge phase ── */

/*
 * Launch a worker back from its stored park position toward the locked queen.
 * park_px/py were saved when the worker hit the screen edge during DIVERGE,
 * so this is the exact reverse of that ray's outward journey.
 */
static void worker_launch_converge(Bug *w, float target_x, float target_y)
{
    w->px = w->park_px;
    w->py = w->park_py;
    float dx   = target_x - w->px;
    float dy   = target_y - w->py;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1.0f) dist = 1.0f;
    float spd  = WORK_SPEED * (0.7f + 0.3f * randf());
    w->vx = (dx / dist) * spd;
    w->vy = (dy / dist) * spd;
    trail_clear(w);
}

static void worker_converge_tick(Bug *w, float target_x, float target_y, float dt)
{
    /* Already arrived and parked — nothing to do */
    if (w->vx == 0.0f && w->vy == 0.0f) return;

    float dx   = target_x - w->px;
    float dy   = target_y - w->py;
    float dist = sqrtf(dx*dx + dy*dy);

    /* Arrived: park at queen, trail retraction happens during PAUSE */
    if (dist < ARRIVE_DIST) {
        w->px = target_x;
        w->py = target_y;
        w->vx = 0.0f;
        w->vy = 0.0f;
        return;
    }

    /* Strong homing toward locked queen — less jitter so path retraces cleanly */
    float tx = (dx / dist) * WORK_SPEED;
    float ty = (dy / dist) * WORK_SPEED;
    w->vx += (tx - w->vx) * CONVERGE_STEER * dt;
    w->vy += (ty - w->vy) * CONVERGE_STEER * dt;
    w->vx += randc() * (WORK_JITTER * 0.4f) * dt;
    w->vy += randc() * (WORK_JITTER * 0.4f) * dt;
    w->vx *= WORK_DAMP;
    w->vy *= WORK_DAMP;
    clamp_speed(w, WORK_SPEED);

    trail_push(w, w->px, w->py);
    w->px += w->vx * dt;
    w->py += w->vy * dt;
}

/* ── state machine ── */

static void swarm_init(Swarm *s, int cp, float max_px, float max_py)
{
    s->cp          = cp;
    s->state       = ST_DIVERGE;
    s->next_state  = ST_CONVERGE;
    s->phase_timer = 0.0f;
    s->pause_timer = 0.0f;

    s->queen.px = randf() * max_px;
    s->queen.py = randf() * max_py;
    s->queen.vx = 0.0f;
    s->queen.vy = 0.0f;
    trail_clear(&s->queen);

    for (int i = 0; i < N_WORKERS; i++)
        worker_launch_diverge(&s->workers[i], &s->queen, i);
}

static void swarm_tick(Swarm *sw, float dt, float max_px, float max_py)
{
    /* Queen is stationary — rays always return to their origin */

    switch (sw->state) {

    case ST_DIVERGE:
        sw->phase_timer += dt;
        for (int j = 0; j < N_WORKERS; j++)
            worker_diverge_tick(&sw->workers[j], dt, max_px, max_py);
        if (sw->phase_timer >= DIVERGE_DUR) {
            /* Lock queen position and record each worker's park position */
            sw->locked_qx = sw->queen.px;
            sw->locked_qy = sw->queen.py;
            sw->queen.vx  = 0.0f;
            sw->queen.vy  = 0.0f;
            for (int j = 0; j < N_WORKERS; j++) {
                sw->workers[j].park_px = sw->workers[j].px;
                sw->workers[j].park_py = sw->workers[j].py;
            }
            sw->state       = ST_PAUSE;
            sw->next_state  = ST_CONVERGE;
            sw->pause_timer = PAUSE_DUR;
        }
        break;

    case ST_PAUSE:
        /* Shrink every trail one slot per tick — visual retraction */
        for (int j = 0; j < N_WORKERS; j++) {
            Bug *w = &sw->workers[j];
            if (w->tlen > 0) w->tlen--;
        }
        sw->pause_timer -= dt;
        if (sw->pause_timer <= 0.0f) {
            sw->phase_timer = 0.0f;
            if (sw->next_state == ST_CONVERGE) {
                /* Launch each worker from its stored park position back to locked queen */
                for (int j = 0; j < N_WORKERS; j++)
                    worker_launch_converge(&sw->workers[j],
                                          sw->locked_qx, sw->locked_qy);
                sw->state = ST_CONVERGE;
            } else {
                for (int j = 0; j < N_WORKERS; j++)
                    worker_launch_diverge(&sw->workers[j], &sw->queen, j);
                sw->state = ST_DIVERGE;
            }
        }
        break;

    case ST_CONVERGE:
        sw->phase_timer += dt;
        for (int j = 0; j < N_WORKERS; j++)
            worker_converge_tick(&sw->workers[j],
                                 sw->locked_qx, sw->locked_qy, dt);
        if (sw->phase_timer >= CONVERGE_DUR) {
            sw->state       = ST_PAUSE;
            sw->next_state  = ST_DIVERGE;
            sw->pause_timer = PAUSE_DUR;
        }
        break;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Swarm swarms[N_SWARMS_MAX];
    int   n_swarms;
    bool  paused;
    float max_px, max_py;    /* pixel-space bounds, updated on resize     */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px   = (float)(cols * CELL_W - 1);
    s->max_py   = (float)(rows * CELL_H - 1);
    s->n_swarms = N_SWARMS_DEFAULT;
    s->paused   = false;

    for (int i = 0; i < N_SWARMS_MAX; i++)
        swarm_init(&s->swarms[i], swarm_colors[i], s->max_px, s->max_py);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    if (s->paused) return;
    for (int i = 0; i < s->n_swarms; i++)
        swarm_tick(&s->swarms[i], dt, s->max_px, s->max_py);
}

static const char *state_name(SwarmState st)
{
    switch(st) {
    case ST_DIVERGE:  return "DIVERGE ";
    case ST_PAUSE:    return "PAUSE   ";
    case ST_CONVERGE: return "CONVERGE";
    }
    return "";
}

/*
 * trail_char() — pick a line-drawing character based on the direction
 * of motion between two consecutive trail positions.
 *
 * Terminal cells are CELL_H/CELL_W ≈ 2× taller than wide, so we scale
 * dy by CELL_W and dx by CELL_H before comparing to get aspect-correct
 * slope thresholds.
 *
 *   mostly horizontal  →  '-'
 *   mostly vertical    →  '|'
 *   diagonal ↘ or ↖   →  '\'
 *   diagonal ↗ or ↙   →  '/'
 */
static chtype trail_char(float dx, float dy)
{
    /* Normalize for cell aspect ratio */
    float adx = fabsf(dx) * (float)CELL_H;   /* horizontal extent in px  */
    float ady = fabsf(dy) * (float)CELL_W;   /* vertical   extent in px  */

    if (adx > ady * 2.2f) return '-';
    if (ady > adx * 2.2f) return '|';
    /* diagonal: sign(dx)==sign(dy) → '\', else '/' */
    return ((dx > 0.0f) == (dy > 0.0f)) ? '\\' : '/';
}

/*
 * Pass-based rendering — solves the cross-swarm overwrite problem.
 *
 * When multiple swarms' trails occupy the same cell, the last draw wins.
 * If we draw swarm-by-swarm, dim tails of a later swarm overwrite bold
 * tips of an earlier one, making rays look like they terminate at the
 * wrong queen.
 *
 * Fix: render across ALL swarms in brightness order —
 *   Pass 0 (DP_DIM)    → dim dots     (oldest 20% of every trail)
 *   Pass 1 (DP_MID)    → normal body  (middle 40%)
 *   Pass 2 (DP_BRIGHT) → bold tips    (newest 40%)
 *   Pass 3 (DP_HEAD)   → worker heads (*), then queen heads (@)
 *
 * Bright tips always paint last and are never overwritten by dim tails.
 */
typedef enum { DP_DIM, DP_MID, DP_BRIGHT, DP_HEAD } DrawPass;

static void draw_bug(WINDOW *w, const Bug *b, int cp, bool is_queen,
                     DrawPass pass, float alpha, float dt_sec, int cols, int rows)
{
    if (pass == DP_HEAD) {
        float draw_px = b->px + b->vx * alpha * dt_sec;
        float draw_py = b->py + b->vy * alpha * dt_sec;
        int   cx = px_to_cell_x(draw_px);
        int   cy = px_to_cell_y(draw_py);
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            chtype ch = is_queen ? '@' : '*';
            wattron(w, COLOR_PAIR(cp) | A_BOLD);
            mvwaddch(w, cy, cx, ch);
            wattroff(w, COLOR_PAIR(cp) | A_BOLD);
        }
        return;
    }

    int len = b->tlen;
    for (int i = len - 1; i >= 0; i--) {
        int idx = (b->thead - 1 - i + TRAIL_LEN * 2) % TRAIL_LEN;
        int cx  = px_to_cell_x(b->tpx[idx]);
        int cy  = px_to_cell_y(b->tpy[idx]);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        float norm = (len > 1) ? 1.0f - (float)i / (float)(len - 1) : 1.0f;

        chtype ch;   attr_t attr;
        DrawPass seg;

        if (norm > 0.60f) {
            seg = DP_BRIGHT; ch = trail_char(b->vx, b->vy); attr = A_BOLD;
        } else if (norm > 0.20f) {
            seg = DP_MID;    ch = trail_char(b->vx, b->vy); attr = A_NORMAL;
        } else {
            seg = DP_DIM;    ch = '.';                       attr = A_DIM;
        }

        if (seg != pass) continue;   /* only draw this brightness tier */

        wattron(w, COLOR_PAIR(cp) | attr);
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(cp) | attr);
    }
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    /* Three brightness passes across all swarms — dim never wins over bright */
    for (DrawPass pass = DP_DIM; pass <= DP_BRIGHT; pass++) {
        for (int i = 0; i < s->n_swarms; i++) {
            const Swarm *sw = &s->swarms[i];
            for (int j = 0; j < N_WORKERS; j++) {
                const Bug *bug = &sw->workers[j];
                if (bug->tlen == 0) continue;
                draw_bug(w, bug, sw->cp, false, pass, alpha, dt_sec, cols, rows);
            }
        }
    }

    /* Head pass — workers first, queens on top of everything */
    for (int i = 0; i < s->n_swarms; i++) {
        const Swarm *sw = &s->swarms[i];
        for (int j = 0; j < N_WORKERS; j++)
            draw_bug(w, &sw->workers[j], sw->cp, false,
                     DP_HEAD, alpha, dt_sec, cols, rows);
    }
    for (int i = 0; i < s->n_swarms; i++) {
        const Swarm *sw = &s->swarms[i];
        draw_bug(w, &sw->queen, sw->cp, true,
                 DP_HEAD, alpha, dt_sec, cols, rows);
    }
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
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top bar: show state of swarm 0 as representative */
    const char *phase = sc->paused ? "PAUSED"
                      : state_name(sc->swarms[0].state);
    char buf[80];
    snprintf(buf, sizeof buf,
             " xrayswarm  queens:%-2d  %s  %4.1ffps  sim:%dHz ",
             sc->n_swarms, phase, fps, sim_fps);
    wattron(stdscr, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(stdscr, 0, 0, "%s", buf);
    wattroff(stdscr, COLOR_PAIR(CP_HUD) | A_DIM);

    /* Key hint — bottom bar */
    wattron(stdscr, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(stdscr, s->rows - 1, 0,
              " q:quit  spc:pause  r:reset  +/-:swarms  [/]:Hz ");
    wattroff(stdscr, COLOR_PAIR(CP_HUD) | A_DIM);
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

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize  = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    /* Recompute pixel bounds so bugs stay inside new terminal dimensions */
    app->scene.max_px  = (float)(app->screen.cols * CELL_W - 1);
    app->scene.max_py  = (float)(app->screen.rows * CELL_H - 1);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;

    case '+': case '=':
        if (sc->n_swarms < N_SWARMS_MAX) {
            swarm_init(&sc->swarms[sc->n_swarms],
                       swarm_colors[sc->n_swarms],
                       sc->max_px, sc->max_py);
            sc->n_swarms++;
        }
        break;

    case '-':
        if (sc->n_swarms > 1) sc->n_swarms--;
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

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
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

        /* ── FPS counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep before render ────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
