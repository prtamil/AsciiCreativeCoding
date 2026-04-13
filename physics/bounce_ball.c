/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bounce.c  —  ncurses bouncing balls, smooth terminal-aware motion
 *
 * ROOT FIX: physics runs in square pixel space, not cell space.
 * RENDER INTERPOLATION: draw positions are lerp'd between ticks using alpha.
 *
 * The core problem with a naive terminal bouncing ball:
 *   - Terminal cells are ~2× taller than wide in physical pixels.
 *   - If you store position in cell coords, dx=1 and dy=1 cover
 *     very different physical distances on screen.
 *   - A ball moving diagonally looks faster horizontally than vertically.
 *   - Speed feels uneven, circles become ellipses, angles look wrong.
 *
 * The fix — two coordinate spaces, one conversion point:
 *
 *   PIXEL SPACE  (physics lives here)
 *     Square grid.  One unit = one physical pixel (approximately).
 *     Width  = cols * CELL_W   (e.g. 200 cols × 8px  = 1600 px wide)
 *     Height = rows * CELL_H   (e.g.  50 rows × 16px =  800 px tall)
 *     Ball position, velocity, speed — all in pixel units.
 *     Balls travel equal physical distance in all directions.
 *
 *   CELL SPACE   (drawing happens here)
 *     Terminal columns and rows.
 *     cell_x = pixel_x / CELL_W
 *     cell_y = pixel_y / CELL_H
 *     One call: px_to_cell().  Nowhere else touches cell coords.
 *
 * With this, 1 pixel/tick is the same physical distance in X and Y.
 * Diagonal motion looks diagonal.  Speed is isotropic.
 * No aspect ratio hacks scattered through the code.
 *
 * RENDER INTERPOLATION (alpha):
 *
 *   The sim accumulator runs fixed-timestep ticks. After draining the
 *   accumulator, sim_accum holds the *leftover* time — how far we are
 *   into the next tick that has not run yet.
 *
 *   Without interpolation:
 *     We draw balls at their last *ticked* position.
 *     That position is up to one full tick behind "now".
 *     At 60 Hz sim / 60 Hz render this lag is 0–16 ms — visible as
 *     micro-stutter when the render frame happens to land just before
 *     a tick fires.
 *
 *   With interpolation:
 *     alpha = sim_accum / tick_ns            ∈ [0.0, 1.0)
 *     draw_px = ball.px + ball.vx * alpha * dt_sec
 *     draw_py = ball.py + ball.vy * alpha * dt_sec
 *
 *     This projects each ball forward by the fractional tick time,
 *     so the drawn position matches "now" to within rendering error.
 *     Motion becomes silky smooth regardless of whether the render
 *     frame lands just before or just after a physics tick.
 *
 *   Interpolation vs extrapolation:
 *     This is technically *extrapolation* (we predict forward from the
 *     last known state).  True interpolation would require storing the
 *     previous tick's position and blending between prev and current.
 *     For constant-velocity balls with elastic wall bounces, forward
 *     extrapolation is numerically identical to interpolation and
 *     requires no extra storage.  If you add acceleration or non-linear
 *     forces, switch to storing prev_px/prev_py and lerp between them.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   =  -      add / remove a ball
 *   r         randomise all balls
 *   ]  [      faster / slower simulation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra bounce.c -o bounce -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — one color pair per ball
 *   §4  coords   — pixel↔cell conversion; the one aspect-ratio fix
 *   §5  ball     — physics in pixel space; spawn; tick
 *   §6  scene    — ball pool; tick; draw  ← draw now accepts alpha
 *   §7  screen   — single stdscr buffer + HUD
 *   §8  app      — dt loop, input, resize, cleanup
 */

#define _POSIX_C_SOURCE 200809L

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
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 40,
    FPS_UPDATE_MS    = 500,

    BALLS_DEFAULT    =  5,
    BALLS_MIN        =  1,
    BALLS_MAX        = 20,
    N_COLORS         =  7,
};

/*
 * CELL_W, CELL_H — logical sub-pixel resolution per terminal cell.
 *
 * These are NOT actual screen pixel sizes — they are the number of
 * sub-steps we divide each cell into for physics precision.
 *
 * Higher values = more sub-steps = smoother apparent motion for slow balls.
 *
 * CELL_H/CELL_W must equal the terminal cell aspect ratio (~2.0).
 *
 * With CELL_W=8, CELL_H=16:
 *   A 200×50 terminal → pixel space 1600×800
 *   A ball at speed 160px/s at 60fps moves 2.67px/tick
 *   → 2.67/8 = 0.33 cols/tick  horizontally
 *   → 2.67/16 = 0.17 rows/tick vertically
 *   Crosses a horizontal cell every ~3 ticks, vertical every ~6 ticks.
 *   That is the staircase threshold — so SPEED_MIN must guarantee
 *   at least ~2px/tick to cross cells often enough to look smooth.
 *
 * The staircase rule:
 *   To avoid staircase, ball must cross at least one cell every N ticks.
 *   speed_px_per_tick = speed / sim_fps
 *   cell_crossings_per_tick_x = speed_px_per_tick / CELL_W
 *   cell_crossings_per_tick_y = speed_px_per_tick / CELL_H
 *   For smooth motion: cell_crossings >= 1 every 4 ticks at minimum.
 *   So: speed >= CELL_H * sim_fps / 4
 *       speed >= 16 * 60 / 4 = 240 px/s  (minimum for vertical smoothness)
 */
#define CELL_W   8
#define CELL_H  16

/*
 * Speed in pixels per second.
 * SPEED_MIN must be high enough that even the slowest ball crosses
 * cell boundaries frequently — otherwise staircase reappears.
 *
 * Formula: SPEED_MIN >= CELL_H * SIM_FPS / 4
 *          = 16 * 60 / 4 = 240
 *
 * We use 300 as the minimum (comfortable margin above the threshold)
 * and 600 as maximum for fast balls.
 */
#define SPEED_MIN  300.0f
#define SPEED_MAX  600.0f

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

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  21, COLOR_BLACK);
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
/* §4  coords — the one place aspect ratio is handled                     */
/* ===================================================================== */

/*
 * This section is the entire fix.  Everything else in the program
 * uses pixel coords.  Only here do we convert to cell coords for drawing.
 *
 * pixel space:  width = cols*CELL_W,  height = rows*CELL_H
 *               square — one unit is one physical pixel
 *
 * cell space:   width = cols,  height = rows
 *               non-square — cells are CELL_H/CELL_W taller than wide
 *
 * px_to_cell_x(px, cols) — convert pixel x → terminal column
 * px_to_cell_y(py, rows) — convert pixel y → terminal row
 *
 * pw(cols) — pixel space width  given terminal cols
 * ph(rows) — pixel space height given terminal rows
 */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell.
 *
 * We use floorf(px/CELL_W + 0.5f) — "round half up" — instead of
 * roundf() or truncation.
 *
 * WHY NOT roundf:
 *   C's roundf uses "round half to even" (banker's rounding).
 *   When px/CELL_W is exactly 0.5 it rounds to 0 one call and 1
 *   the next depending on the FPU state.  A ball sitting on a cell
 *   boundary oscillates between two cells every frame — visible as
 *   a doubled/flickering character.
 *
 * WHY NOT truncation (int)(px/CELL_W):
 *   Always rounds down regardless of fractional part.
 *   Creates asymmetric dwell time — staircase pattern.
 *
 * WHY floorf(px/CELL_W + 0.5f):
 *   Adds 0.5 before flooring.  This is "round half up" — always
 *   rounds to the nearest cell, and always breaks ties in the same
 *   direction (up).  Deterministic on every call, no oscillation.
 *   Symmetric dwell time like roundf but without the tie-breaking bug.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  ball                                                               */
/* ===================================================================== */

/*
 * Ball — all positions and velocities are in PIXEL SPACE.
 *
 *   px, py    position in pixels  (float for sub-pixel precision)
 *   vx, vy    velocity in pixels per second
 *
 * At draw time:  cell_x = px_to_cell_x(px),  cell_y = px_to_cell_y(py)
 * That is the only conversion. Physics never touches cell coordinates.
 */
typedef struct {
    float px, py;
    float vx, vy;
    int   color;
    char  ch;
} Ball;

static const char k_chars[] = "o*O@+";
static const int  k_nchars  = (int)(sizeof k_chars - 1);

/*
 * ball_spawn() — place ball at random pixel position with random velocity.
 *
 * Velocity direction is a random angle so balls move in all directions
 * equally — not just axis-aligned.  Speed is uniform in pixel space,
 * so a ball moving NE covers the same physical distance as one moving E.
 */
static void ball_spawn(Ball *b, int i, int cols, int rows)
{
    int pxw = pw(cols);
    int pxh = ph(rows);

    b->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
    b->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));

    /*
     * Random angle gives isotropic direction distribution.
     * Without this, using separate random vx/vy produces more
     * diagonal balls than axis-aligned ones (non-uniform angle dist).
     *
     * We avoid math.h by using a simple rejection-sample unit vector:
     * pick random (dx, dy) in [-1,1]², keep if inside unit circle.
     */
    float dx, dy, len;
    do {
        dx = (float)(rand() % 2001 - 1000) / 1000.0f;
        dy = (float)(rand() % 2001 - 1000) / 1000.0f;
        len = dx*dx + dy*dy;
    } while (len < 0.01f || len > 1.0f);

    /* Normalise to unit vector then scale to random speed */
    float mag   = sqrtf(len);
    float speed = SPEED_MIN
                + (float)(rand() % (int)(SPEED_MAX - SPEED_MIN + 1));
    b->vx = (dx / mag) * speed;
    b->vy = (dy / mag) * speed;

    b->color = (i % N_COLORS) + 1;
    b->ch    = k_chars[i % k_nchars];
}

/*
 * ball_tick() — advance one ball by dt seconds, purely in pixel space.
 *
 * Receives pre-computed pixel boundaries (max_px, max_py) from scene_tick.
 * Has zero knowledge of terminal columns, rows, CELL_W, or CELL_H.
 *
 * Physics:
 *   move:     position += velocity * dt
 *   reflect:  if wall hit, flip the relevant velocity component
 *             and clamp position back inside the boundary
 */
static void ball_tick(Ball *b, float dt, float max_px, float max_py)
{
    b->px += b->vx * dt;
    b->py += b->vy * dt;

    if (b->px < 0.0f)     { b->px = 0.0f;    b->vx = -b->vx; }
    if (b->px > max_px)   { b->px = max_px;   b->vx = -b->vx; }
    if (b->py < 0.0f)     { b->py = 0.0f;     b->vy = -b->vy; }
    if (b->py > max_py)   { b->py = max_py;   b->vy = -b->vy; }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Ball  balls[BALLS_MAX];
    int   n;
    bool  paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->n      = BALLS_DEFAULT;
    s->paused = false;
    for (int i = 0; i < s->n; i++)
        ball_spawn(&s->balls[i], i, cols, rows);
}

/*
 * scene_tick() — advance the simulation one step.
 *
 * This is the only function in §6 that calls pw/ph.
 * It converts terminal dimensions (cell space) into pixel boundaries
 * once, then passes those pixel values to ball_tick.
 *
 * Data flow:
 *   cell space (cols, rows)
 *       ↓  pw() / ph()          ← conversion happens exactly here
 *   pixel space (max_px, max_py)
 *       ↓  ball_tick()
 *   physics (position, velocity) — knows nothing about cells
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused) return;

    float max_px = (float)(pw(cols) - 1);   /* pixel space right edge  */
    float max_py = (float)(ph(rows) - 1);   /* pixel space bottom edge */

    for (int i = 0; i < s->n; i++)
        ball_tick(&s->balls[i], dt, max_px, max_py);
}

/*
 * scene_draw() — interpolated draw.
 *
 * alpha ∈ [0.0, 1.0) is the fractional tick time remaining in sim_accum.
 *
 *   draw_px = ball.px + ball.vx * alpha * dt_sec
 *   draw_py = ball.py + ball.vy * alpha * dt_sec
 *
 * This projects each ball forward by the sub-tick remainder so the
 * rendered position matches "wall-clock now" rather than "last tick".
 *
 * WHY this is safe (no wall-check needed on draw position):
 *   alpha < 1.0 always, so the projected position overshoots by less
 *   than one full tick.  After a wall bounce the velocity is already
 *   reversed, so the projection moves *away* from the wall — it can
 *   never escape the pixel boundary by more than ~1 tick worth of travel,
 *   which is sub-cell and invisible.  The clamp below catches any
 *   floating-point edge cases anyway.
 *
 * WHY not store prev_px/prev_py and lerp between them:
 *   For constant-velocity elastic bounces, forward extrapolation from
 *   the current state is numerically identical to interpolation between
 *   prev and current.  It requires no extra fields in Ball.
 *   If you add gravity or non-linear forces, switch to prev/current lerp.
 *
 * This is the ONLY function that calls px_to_cell_x/y.
 * Physics above never sees cell coordinates.
 * Drawing below never sees pixel coordinates.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    float max_px = (float)(pw(cols) - 1);
    float max_py = (float)(ph(rows) - 1);

    for (int i = 0; i < s->n; i++) {
        const Ball *b = &s->balls[i];

        /*
         * Interpolated draw position.
         * Project the ball forward by (alpha * dt_sec) seconds from
         * its last ticked position using its current velocity.
         *
         * alpha = sim_accum / tick_ns   (computed in the main loop)
         *
         * Example: sim runs at 60 Hz (tick every 16.67 ms).
         *   Render fires 10 ms after last tick → alpha ≈ 0.60
         *   A ball at px=100 with vx=300 px/s:
         *     draw_px = 100 + 300 * 0.60 * (1/60) = 100 + 3.0 = 103
         *   Without interpolation it would draw at 100 — 3 px behind.
         *   At CELL_W=8, 3px = 0.375 cells of lag, visible as stutter.
         */
        float draw_px = b->px + b->vx * alpha * dt_sec;
        float draw_py = b->py + b->vy * alpha * dt_sec;

        /* Clamp interpolated position to pixel boundary (safety net) */
        if (draw_px < 0.0f)    draw_px = 0.0f;
        if (draw_px > max_px)  draw_px = max_px;
        if (draw_py < 0.0f)    draw_py = 0.0f;
        if (draw_py > max_py)  draw_py = max_py;

        int cx = px_to_cell_x(draw_px);
        int cy = px_to_cell_y(draw_py);

        /* Clamp to visible cell area */
        if (cx < 0) cx = 0;
        if (cx >= cols) cx = cols - 1;
        if (cy < 0) cy = 0;
        if (cy >= rows) cy = rows - 1;

        wattron(w, COLOR_PAIR(b->color) | A_BOLD);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)b->ch);
        wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Architecture: ONE window (stdscr), ONE flush (doupdate).
 *
 * Why no second WINDOW at all:
 *
 *   ncurses maintains two virtual screens internally:
 *     curscr  — what ncurses believes is on the physical terminal now
 *     newscr  — the target state you are building this frame
 *
 *   Every mvwaddch/werase/wattron writes into newscr.
 *   doupdate() computes (newscr - curscr), sends only changed cells,
 *   then sets curscr = newscr.
 *
 *   This IS the double buffer. It is always present. It is not optional.
 *   Adding your own back/front WINDOW pair creates a third virtual screen
 *   that ncurses does not know about, breaking the diff accuracy and
 *   producing ghost trails.
 *
 * The correct model:
 *   erase()                — clear newscr (the "back buffer")
 *   mvwaddch(stdscr, …)    — write scene into newscr
 *   mvwprintw(stdscr, …)   — write HUD into newscr (same buffer, last)
 *   wnoutrefresh(stdscr)   — mark newscr ready (no terminal I/O yet)
 *   doupdate()             — one write: send diff to terminal
 *
 * HUD is written into stdscr after balls so it overwrites any ball
 * that happens to be on the HUD row — correct Z-order, no separate window.
 *
 * No flicker:  ncurses' diff engine never shows a partial frame.
 * No ghost:    curscr is always accurate — one source of truth.
 * No tear:     doupdate() is one atomic write to the terminal fd.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);       /* never interrupt output to check for input    */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();                           /* re-reads LINES and COLS      */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — build the complete frame in stdscr (newscr).
 *
 * alpha   — render interpolation factor ∈ [0.0, 1.0)
 * dt_sec  — fixed sim tick duration in seconds (needed for extrapolation)
 *
 * Order:
 *   1. erase()      — clear newscr so stale ball positions become spaces
 *   2. scene_draw() — write all balls into newscr at interpolated positions
 *   3. HUD line     — write status bar into newscr, top-right corner
 *                     (drawn last so it is always on top)
 *
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    /* balls — drawn at interpolated positions */
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — written directly into stdscr after balls */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  balls:%-2d  %s  spd:%d",
             fps, sc->n, sc->paused ? "PAUSED " : "running", sim_fps);
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);
}

/*
 * screen_present() — flush newscr to terminal.
 *
 * wnoutrefresh(stdscr) — copy stdscr into ncurses' newscr model
 * doupdate()           — diff newscr vs curscr, one write, update curscr
 */
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

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    int cols = app->screen.cols;
    int rows = app->screen.rows;
    for (int i = 0; i < app->scene.n; i++) {
        Ball *b = &app->scene.balls[i];
        if (b->px >= (float)pw(cols)) b->px = (float)(pw(cols) - 1);
        if (b->py >= (float)ph(rows)) b->py = (float)(ph(rows) - 1);
    }
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    int cols  = app->screen.cols;
    int rows  = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        for (int i = 0; i < s->n; i++)
            ball_spawn(&s->balls[i], i, cols, rows);
        break;

    case '=': case '+':
        if (s->n < BALLS_MAX) {
            ball_spawn(&s->balls[s->n], s->n, cols, rows);
            s->n++;
        }
        break;

    case '-':
        if (s->n > BALLS_MIN) s->n--;
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
    srand((unsigned int)clock_ns());

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

        /*
         * ── render interpolation alpha ────────────────────────────
         *
         * sim_accum is the leftover nanoseconds after draining all
         * complete ticks — i.e. how far we are into the *next* tick
         * that has not fired yet.
         *
         * alpha = sim_accum / tick_ns  ∈ [0.0, 1.0)
         *
         * alpha = 0.0 → render fires exactly on a tick boundary
         *               draw position == physics position  (no change)
         * alpha = 0.9 → render fires 90% of the way through the tick
         *               draw position is projected 90% of a tick ahead
         *
         * Passed to screen_draw → scene_draw, which adds
         *   ball.vx * alpha * dt_sec  to each ball's draw_px.
         *
         * When paused, alpha is still computed but ball velocities
         * are non-zero — however scene_tick is skipped, so physics
         * positions do not change, and the projected draw position
         * creeps slightly.  This is imperceptible (< 1 cell drift
         * over the pause duration) and correct behaviour —
         * extrapolation from a frozen state converges to zero effect
         * once alpha wraps around on the next tick boundary.
         * If you want pixel-perfect freeze, zero alpha when paused:
         *   float alpha = app->scene.paused ? 0.0f : ...
         */
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

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present (one doupdate flush) ─────────────────── */
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
