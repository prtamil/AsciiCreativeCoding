/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * constellation.c — particle constellation effect in ncurses
 *
 * Stars drift slowly across the terminal.  When two stars come within
 * CONNECT_DIST pixels of each other an ASCII line is drawn between them
 * using slope-matched characters.  Lines fade with distance via bold
 * weight and stippling, creating a live constellation map.
 *
 * ROOT FIX: same pixel-space / cell-space separation as bounce_ball.c.
 * Physics runs entirely in square pixel space; only px_to_cell_x/y()
 * converts to terminal columns/rows for drawing.
 *
 * RENDER INTERPOLATION:
 *   Stars carry prev_px/prev_py (previous tick position) so we can lerp:
 *     draw_px = prev_px + (px - prev_px) * alpha
 *   This is true interpolation, not extrapolation, which is required
 *   here because wander acceleration changes velocity each tick — forward
 *   extrapolation from the current state would drift from the real
 *   in-between position.
 *
 * CONNECTION RENDERING:
 *   For each pair (i < j) within CONNECT_DIST pixels:
 *     ratio = dist / connect_dist   (0.0 = same spot, 1.0 = edge)
 *     ratio < 0.50  → bold, every cell drawn       (close, bright)
 *     ratio < 0.75  → normal, every cell drawn      (medium)
 *     ratio < 1.00  → normal, every 2nd cell drawn  (far, stippled)
 *   Slope character is selected from pixel-space angle (physically correct):
 *     0–22.5°   '-'
 *     22.5–67.5° '\' or '/'  (sign of dx_px × dy_px)
 *     67.5–90°  '|'
 *   Thin-line rendering (one cell per major-axis step) prevents the doubled
 *   diagonal characters ('\\'  '//') that vanilla Bresenham produces.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   =  -      add / remove a star
 *   r         randomise all stars
 *   ]  [      faster / slower simulation
 *   c         cycle connection threshold (tight / normal / wide)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra constellation.c -o constellation -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — night-sky palette
 *   §4  coords   — pixel↔cell conversion; the one aspect-ratio fix
 *   §5  star     — pixel-space physics with wander + speed cap
 *   §6  scene    — star pool; tick; Bresenham line draw
 *   §7  screen   — single stdscr buffer + HUD
 *   §8  app      — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Star wander (Ornstein-Uhlenbeck) + proximity-graph line drawing.
 *                  Each star applies a small random velocity increment (wander)
 *                  bounded to prevent runaway speed.  For each pair of stars
 *                  within CONNECT_DIST pixels, a line is drawn with slope-matched
 *                  characters and distance-based stippling.
 *
 * Physics        : Wander: each tick, velocity perturbed by small random
 *                  δvx, δvy bounded at WANDER_FORCE; speed capped at SPEED_MAX.
 *                  Stars bounce off screen edges with velocity reflection.
 *
 * Rendering      : Render interpolation: draw position lerped between previous
 *                  and current tick position at sub-frame alpha — prevents
 *                  jitter when render rate ≠ sim rate.  Thin-line Bresenham:
 *                  one cell per major-axis step to avoid doubled diagonal chars.
 *                  Line brightness: ratio < 0.50 → bold; ratio < 0.75 → normal;
 *                  ratio < 1.00 → stippled (every 2nd cell drawn).
 *
 * Math           : Slope character selection from pixel-space angle:
 *                  0–22.5° → '─'; 22.5–67.5° → '╲'/'╱'; 67.5–90° → '│'.
 *                  CONNECT_DIST in pixels; aspect-corrected before comparison.
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 54,
    FPS_UPDATE_MS    = 500,

    STARS_DEFAULT    = 30,
    STARS_MIN        =  5,
    STARS_MAX        = 80,

    N_STAR_COLORS    =  6,   /* color pairs 1..6 for stars               */
    CONN_PAIR        =  7,   /* color pair for all connection lines       */
    HUD_PAIR         =  8,   /* color pair for HUD text                  */
};

/*
 * CELL_W, CELL_H — sub-pixel units per terminal cell.
 * CELL_H / CELL_W = 2 matches the typical terminal cell aspect ratio.
 * All physics lives in this square pixel space.
 */
#define CELL_W   8
#define CELL_H  16

/*
 * Star drift speed (pixels per second).
 *
 * The staircase rule (speed >= CELL_H × fps / 4 ≈ 240 px/s) is
 * intentionally relaxed here.  The visual goal is slow, drifting stars.
 * Wander force keeps motion non-axis-aligned, reducing staircase
 * visibility.  Lerp interpolation eliminates sub-cell jitter between ticks.
 */
#define SPEED_MIN   50.0f
#define SPEED_MAX  120.0f

/*
 * Wander: random acceleration (px/s²) added each tick.
 * Keeps star paths gently curving rather than straight forever.
 * SPEED_CAP prevents unlimited runaway from accumulated wander.
 */
#define WANDER_ACCEL  20.0f
#define SPEED_CAP    130.0f

/*
 * Connection distance presets (pixels).  Higher = more connections.
 * At 200 px on a 200-col × 50-row terminal (1600 × 800 px space):
 *   each star's connect circle covers ~10 % of the screen area
 *   → expect ~3 neighbours per star → ~45 lines for 30 stars.
 */
static const float k_connect_presets[] = { 120.0f, 200.0f, 280.0f };
static const char *k_connect_names[]   = { "tight", "normal", "wide" };
enum { N_CONNECT_PRESETS = 3 };

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
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
 * Night-sky / constellation palette.
 *
 * Stars (pairs 1–6): bright, saturated — each stands out individually.
 * Connection lines (pair 7): single dim steel-blue so the graph reads
 *   as background structure, not competing with the stars.
 * HUD (pair 8): yellow for easy readability.
 *
 * 8-color fallback maps to the nearest named color.
 */
static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(1,  15, COLOR_BLACK);   /* bright white   — star */
        init_pair(2,  51, COLOR_BLACK);   /* bright cyan    — star */
        init_pair(3,  39, COLOR_BLACK);   /* cornflower     — star */
        init_pair(4, 201, COLOR_BLACK);   /* hot magenta    — star */
        init_pair(5, 147, COLOR_BLACK);   /* light purple   — star */
        init_pair(6, 159, COLOR_BLACK);   /* pale cyan      — star */
        init_pair(7,  24, COLOR_BLACK);   /* steel blue     — connection lines */
        init_pair(8, 226, COLOR_BLACK);   /* yellow         — HUD  */
    } else {
        init_pair(1, COLOR_WHITE,   COLOR_BLACK);
        init_pair(2, COLOR_CYAN,    COLOR_BLACK);
        init_pair(3, COLOR_BLUE,    COLOR_BLACK);
        init_pair(4, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_WHITE,   COLOR_BLACK);
        init_pair(7, COLOR_BLUE,    COLOR_BLACK);   /* connections */
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);   /* HUD         */
    }
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                    */
/* ===================================================================== */

/*
 * Pixel-space extents from terminal dimensions.
 * pw/ph are the only callers of CELL_W/CELL_H outside §5.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * "Round half up" avoids the banker's rounding oscillation that roundf()
 * can produce when a star sits exactly on a cell boundary.
 * See bounce_ball.c §4 for the detailed explanation.
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
/* §5  star                                                               */
/* ===================================================================== */

/*
 * Star — all positions and velocities live in PIXEL SPACE.
 *
 *   px, py          current tick position (pixels)
 *   prev_px, prev_py  previous tick position (pixels)
 *   vx, vy          velocity (pixels per second)
 *
 * WHY prev_px / prev_py instead of forward extrapolation:
 *   The wander acceleration changes velocity each tick, so the draw
 *   position after alpha ticks into the future cannot be predicted from
 *   the current state alone without solving the acceleration integral.
 *   Lerp between prev and current is exact for any acceleration profile:
 *
 *     draw_px = prev_px + (px - prev_px) * alpha   alpha ∈ [0, 1)
 *
 *   bounce_ball.c uses forward extrapolation only because its balls have
 *   constant velocity between ticks (no acceleration) — the two methods
 *   are equivalent in that special case.
 */
typedef struct {
    float px,      py;       /* position — current tick  (pixel space) */
    float prev_px, prev_py; /* position — previous tick (pixel space) */
    float vx,      vy;       /* velocity (px/s)                        */
    int   color;             /* color pair index (1..N_STAR_COLORS)    */
    char  ch;                /* star symbol                            */
} Star;

static const char k_star_chars[] = "*+o@.";
static const int  k_n_star_chars = (int)(sizeof k_star_chars - 1);

static void star_spawn(Star *s, int idx, int cols, int rows)
{
    int pxw = pw(cols);
    int pxh = ph(rows);

    s->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
    s->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));
    s->prev_px = s->px;
    s->prev_py = s->py;

    /*
     * Isotropic random direction via rejection-sample unit disk.
     * Separate random vx/vy produces too many diagonal stars; this
     * gives a uniform angle distribution.
     */
    float dx, dy, len;
    do {
        dx  = (float)(rand() % 2001 - 1000) / 1000.0f;
        dy  = (float)(rand() % 2001 - 1000) / 1000.0f;
        len = dx*dx + dy*dy;
    } while (len < 0.01f || len > 1.0f);

    float mag   = sqrtf(len);
    float speed = SPEED_MIN
                + (float)(rand() % (int)(SPEED_MAX - SPEED_MIN + 1));
    s->vx = (dx / mag) * speed;
    s->vy = (dy / mag) * speed;

    s->color = (idx % N_STAR_COLORS) + 1;
    s->ch    = k_star_chars[idx % k_n_star_chars];
}

/*
 * star_tick() — advance one star by dt seconds in pixel space.
 *
 * Steps each tick:
 *   1. Save current position into prev (used for lerp in draw).
 *   2. Apply wander: random acceleration nudge makes paths curve gently.
 *   3. Clamp speed to SPEED_CAP (wander can accumulate otherwise).
 *   4. Integrate velocity → position.
 *   5. Elastic wall bounce (flip velocity component, clamp position).
 *
 * All units are pixels and pixels/second.  No cell coordinates here.
 */
static void star_tick(Star *s, float dt, float max_px, float max_py)
{
    /* 1. save for lerp */
    s->prev_px = s->px;
    s->prev_py = s->py;

    /* 2. wander: random acceleration in [-WANDER_ACCEL, +WANDER_ACCEL] */
    float ax = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    float ay = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    s->vx += ax * dt;
    s->vy += ay * dt;

    /* 3. speed cap */
    float spd = sqrtf(s->vx * s->vx + s->vy * s->vy);
    if (spd > SPEED_CAP) {
        float inv = SPEED_CAP / spd;
        s->vx *= inv;
        s->vy *= inv;
    }

    /* 4. integrate */
    s->px += s->vx * dt;
    s->py += s->vy * dt;

    /* 5. elastic wall bounce */
    if (s->px < 0.0f)    { s->px = 0.0f;    s->vx = -s->vx; }
    if (s->px > max_px)  { s->px = max_px;   s->vx = -s->vx; }
    if (s->py < 0.0f)    { s->py = 0.0f;     s->vy = -s->vy; }
    if (s->py > max_py)  { s->py = max_py;   s->vy = -s->vy; }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Star  stars[STARS_MAX];
    int   n;
    bool  paused;
    int   connect_preset;   /* index into k_connect_presets */
} Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->n              = STARS_DEFAULT;
    sc->connect_preset = 1;   /* "normal" */
    for (int i = 0; i < sc->n; i++)
        star_spawn(&sc->stars[i], i, cols, rows);
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    if (sc->paused) return;

    float max_px = (float)(pw(cols) - 1);
    float max_py = (float)(ph(rows) - 1);

    for (int i = 0; i < sc->n; i++)
        star_tick(&sc->stars[i], dt, max_px, max_py);
}

/*
 * draw_line() — per-step character selection + stipple + first-writer wins.
 *
 * Character is chosen at each Bresenham step from the LOCAL movement,
 * not the overall line angle:
 *
 *   Shallow (adx ≥ ady) — loop advances one column per step:
 *     next step will move y  →  '\' or '/'   (true diagonal step)
 *     next step keeps y      →  '-'           (horizontal run)
 *
 *   Steep (ady > adx) — loop advances one row per step:
 *     next step will move x  →  '\' or '/'   (true diagonal step)
 *     next step keeps x      →  '|'           (vertical run)
 *
 * The diagonal direction ('\' vs '/') is fixed for the whole line:
 *   sx * sy > 0  (right-down or left-up)   →  '\'
 *   sx * sy < 0  (right-up  or left-down)  →  '/'
 *
 * This guarantees at most one diagonal character per row (shallow) or
 * per column (steep) — no doubled '\\'  or '//' anywhere.
 *
 * `used` is a flat [rows × cols] bool grid; cells already drawn are
 * skipped so overlapping connections do not overwrite each other.
 *
 * Out-of-bounds cells are silently skipped.
 */
static void draw_line(WINDOW *w,
                      int x0, int y0, int x1, int y1,
                      chtype attr, int stipple,
                      int cols, int rows,
                      bool *used)
{
    int adx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int ady = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int step = 0;
    char diag = (sx * sy > 0) ? '\\' : '/';

    if (adx == 0 && ady == 0) {
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows
                && !used[y0 * cols + x0]) {
            used[y0 * cols + x0] = true;
            wattron(w, attr);
            mvwaddch(w, y0, x0, (chtype)(unsigned char)diag);
            wattroff(w, attr);
        }
        return;
    }

    if (adx >= ady) {
        /* shallow: one cell per column */
        int err = adx / 2;
        for (int x = x0; x != x1 + sx; x += sx) {
            int next_err = err - ady;
            char ch = (next_err < 0) ? diag : '-';
            if (x >= 0 && x < cols && y0 >= 0 && y0 < rows
                    && step % stipple == 0 && !used[y0 * cols + x]) {
                used[y0 * cols + x] = true;
                wattron(w, attr);
                mvwaddch(w, y0, x, (chtype)(unsigned char)ch);
                wattroff(w, attr);
            }
            step++;
            err = next_err;
            if (err < 0) { y0 += sy; err += adx; }
        }
    } else {
        /* steep: one cell per row */
        int err = ady / 2;
        for (int y = y0; y != y1 + sy; y += sy) {
            int next_err = err - adx;
            char ch = (next_err < 0) ? diag : '|';
            if (x0 >= 0 && x0 < cols && y >= 0 && y < rows
                    && step % stipple == 0 && !used[y * cols + x0]) {
                used[y * cols + x0] = true;
                wattron(w, attr);
                mvwaddch(w, y, x0, (chtype)(unsigned char)ch);
                wattroff(w, attr);
            }
            step++;
            err = next_err;
            if (err < 0) { x0 += sx; err += ady; }
        }
    }
}

/*
 * scene_draw() — render one frame at interpolated time alpha.
 *
 * alpha ∈ [0.0, 1.0) is the fractional tick elapsed since the last
 * physics step.  Used to lerp each star between its previous and current
 * tick position:
 *
 *   draw_px = prev_px + (px - prev_px) * alpha
 *   draw_py = prev_py + (py - prev_py) * alpha
 *
 * Draw order:
 *   1. Connection lines first (drawn into background).
 *   2. Stars after — overwriting any line that passed through the cell —
 *      so star symbols are always fully visible on top of lines.
 *
 * Connection line style by normalised distance ratio = dist / connect_dist:
 *   ratio < 0.50  →  A_BOLD, stipple 1 (every cell)      bright, close
 *   ratio < 0.75  →  normal, stipple 1 (every cell)      medium
 *   ratio < 1.00  →  normal, stipple 2 (every 2nd cell)  fading, far
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha)
{
    float connect_dist = k_connect_presets[sc->connect_preset];
    float cdist_sq     = connect_dist * connect_dist;

    /* --- pre-compute lerp'd pixel and cell positions for all stars ---- */
    float dpx[STARS_MAX], dpy[STARS_MAX];
    int   dcx[STARS_MAX], dcy[STARS_MAX];

    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        dpx[i] = s->prev_px + (s->px - s->prev_px) * alpha;
        dpy[i] = s->prev_py + (s->py - s->prev_py) * alpha;
        dcx[i] = px_to_cell_x(dpx[i]);
        dcy[i] = px_to_cell_y(dpy[i]);
        if (dcx[i] < 0)     dcx[i] = 0;
        if (dcx[i] >= cols) dcx[i] = cols - 1;
        if (dcy[i] < 0)     dcy[i] = 0;
        if (dcy[i] >= rows) dcy[i] = rows - 1;
    }

    /* --- 1. connection lines ------------------------------------------ */

    /* One cell visited by any connection line is not redrawn by others.
     * This prevents the mixed-character thick-bundle look that appears
     * when several lines overlap in the same screen region. */
    bool cell_used[rows][cols];
    memset(cell_used, 0, sizeof cell_used);

    for (int i = 0; i < sc->n - 1; i++) {
        for (int j = i + 1; j < sc->n; j++) {

            float dx_px   = dpx[j] - dpx[i];
            float dy_px   = dpy[j] - dpy[i];
            float dist_sq = dx_px * dx_px + dy_px * dy_px;

            if (dist_sq >= cdist_sq) continue;

            float dist  = sqrtf(dist_sq);
            float ratio = dist / connect_dist;   /* 0.0 (close) → 1.0 (edge) */

            chtype attr;
            int    stipple;

            if (ratio < 0.50f) {
                attr    = COLOR_PAIR(CONN_PAIR) | A_BOLD;
                stipple = 1;
            } else if (ratio < 0.75f) {
                attr    = COLOR_PAIR(CONN_PAIR);
                stipple = 1;
            } else {
                attr    = COLOR_PAIR(CONN_PAIR);
                stipple = 2;
            }

            draw_line(w,
                      dcx[i], dcy[i], dcx[j], dcy[j],
                      attr, stipple,
                      cols, rows,
                      &cell_used[0][0]);
        }
    }

    /* --- 2. stars — drawn after lines so they are always on top ------- */
    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        wattron(w, COLOR_PAIR(s->color) | A_BOLD);
        mvwaddch(w, dcy[i], dcx[i], (chtype)(unsigned char)s->ch);
        wattroff(w, COLOR_PAIR(s->color) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen wraps the ncurses single-window model.
 *
 * One WINDOW (stdscr), one flush (doupdate).  ncurses' internal
 * curscr / newscr pair IS the double buffer — no extra WINDOW needed.
 * erase() → mvwaddch() → mvprintw() → wnoutrefresh() → doupdate().
 * HUD is drawn last so it always overlays the scene.
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
    typeahead(-1);      /* never interrupt output to check for input    */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();          /* re-reads LINES and COLS                      */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  stars:%-2d  conn:%-6s  spd:%-2d  %s",
             fps, sc->n,
             k_connect_names[sc->connect_preset],
             sim_fps,
             sc->paused ? "PAUSED " : "running");
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
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

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    int   cols = app->screen.cols;
    int   rows = app->screen.rows;
    float mpx  = (float)(pw(cols) - 1);
    float mpy  = (float)(ph(rows) - 1);
    for (int i = 0; i < app->scene.n; i++) {
        Star *s = &app->scene.stars[i];
        if (s->px      > mpx) s->px      = mpx;
        if (s->py      > mpy) s->py      = mpy;
        if (s->prev_px > mpx) s->prev_px = mpx;
        if (s->prev_py > mpy) s->prev_py = mpy;
    }
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc   = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        for (int i = 0; i < sc->n; i++)
            star_spawn(&sc->stars[i], i, cols, rows);
        break;

    case '=': case '+':
        if (sc->n < STARS_MAX) {
            star_spawn(&sc->stars[sc->n], sc->n, cols, rows);
            sc->n++;
        }
        break;

    case '-':
        if (sc->n > STARS_MIN) sc->n--;
        break;

    case 'c': case 'C':
        sc->connect_preset = (sc->connect_preset + 1) % N_CONNECT_PRESETS;
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
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* pause guard */

        /* ── fixed-timestep sim accumulator ─────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /*
         * alpha = fractional tick elapsed since last physics step.
         * Passed to scene_draw for lerp interpolation.
         *   alpha = 0.0 → draw at ticked position (no change)
         *   alpha = 0.9 → draw 90 % of the way toward the next tick
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

        /* ── frame cap ───────────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ─────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
