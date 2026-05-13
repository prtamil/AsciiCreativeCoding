/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * constellation.c — drifting stars wired by an O(N²) proximity graph.
 *
 * Stars wander with a bounded random walk in pixel space; every render
 * frame, all C(N, 2) pairs are scanned and pairs within CONNECT_DIST get
 * a thin ASCII line in one of three brightness tiers.  The graph has
 * ZERO state — it's recomputed from positions each frame.  Seven themes
 * cycle the seven rendering pair IDs without redrawing.
 *
 * Section map
 *   §1 config       sim/display constants, themes table, connect presets
 *   §2 clock        monotonic-ns timer + sleep
 *   §3 color        7-theme palette + theme_apply + color_init
 *   §4 coords       pw/ph + px_to_cell_*: the ONE aspect-ratio fix
 *   §5 star         Star struct + star_spawn + star_tick (5 steps)
 *   §6 scene        Scene + lifecycle + scene_tick + draw_line + scene_draw
 *   §7 screen       ncurses init/draw/resize, HUD + HINT bars
 *   §8 app          signals, fixed-step main loop with lerp alpha
 */

/* ── CONCEPTS & ALGORITHMS ───────────────────────────────────────────── *
 *
 *   Proximity graph        O(N²) per-frame pair scan; redraw from current
 *                          state — no edge list to maintain, no staleness.
 *                          → Newman, *Networks: An Introduction*, Ch. 6
 *
 *   Ornstein-Uhlenbeck     Bounded random walk: random kick + magnitude
 *                          clamp.  Direction wanders, speed bounded.
 *                          → Uhlenbeck & Ornstein (1930), Phys. Rev. 36: 823
 *
 *   Bresenham line (thin)  ONE cell per MAJOR-axis step + slope-matched
 *                          glyph.  Vanilla Bresenham doubles diagonals.
 *                          → Bresenham (1965), IBM Systems J. 4(1): 25
 *
 *   Pixel/cell split       Physics in square pixels (CELL_W=8, CELL_H=16);
 *                          rendering in cells.  ONE conversion at draw time.
 *                          → bounce_ball.c §4 (this repo) — first appearance
 *
 *   Render lerp            draw_p = prev_p + (p − prev_p)·alpha   (BACKWARDS
 *                          interpolation; never overshoots the next tick).
 *                          → Glenn Fiedler, "Fix Your Timestep!" (2004)
 *
 *   Painter's algorithm    Lines under, stars over.  Foreground always wins.
 *                          → Newell, Newell & Sancha (1972) hidden-surface
 *
 *   Squared-distance cull  d² < D²  instead of  d < D.  sqrt only runs for
 *                          pairs that actually draw — kills ~95% of sqrtfs.
 *                          → standard numerical-recipes idiom
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── OVERALL PSEUDOCODE ──────────────────────────────────────────────── *
 *
 *   init:
 *     install signals; start ncurses (themes + HUD/HINT pairs)
 *     scene_init: spawn STARS_DEFAULT stars at random positions
 *
 *   loop while running:
 *     if need_resize:        re-query terminal; clamp out-of-bounds stars
 *     dt   = clock_now − last_frame_time              (capped at 100 ms)
 *     while sim_accum ≥ tick_ns:                       fixed-step physics
 *       scene_tick(dt_sec)
 *       sim_accum -= tick_ns
 *     alpha = sim_accum / tick_ns                      ∈ [0, 1)
 *     update fps_display every FPS_UPDATE_MS
 *     sleep to ~60 fps render
 *     scene_draw(alpha) + screen_draw(HUD, HINT) + screen_present
 *     getch + app_handle_key                            non-blocking input
 *
 *   cleanup:
 *     atexit endwin restores the terminal
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── DRIVER PSEUDOCODE ───────────────────────────────────────────────── *
 *
 * scene_tick(dt):
 *   if paused: return
 *   max_px, max_py ← pw(cols)-1, ph(rows)-1
 *   for each star in stars[0..n-1]:
 *     star_tick(s, dt, max_px, max_py)
 *
 *   Pure physics; never touches ncurses.  Stars do NOT see each other
 *   here — that's a render-time concern (scene_draw_connections).
 *
 *
 * star_tick(s, dt, max_px, max_py):                    (5 labelled steps)
 *   Step 1  prev_px, prev_py ← px, py                  save for render lerp
 *   Step 2  v += uniform(±WANDER_ACCEL) · dt           Ornstein-Uhlenbeck
 *   Step 3  if |v| > SPEED_CAP: v *= SPEED_CAP / |v|   magnitude clamp
 *   Step 4  px, py += v · dt                           explicit Euler
 *   Step 5  for each wall: snap into bounds + negate normal vel
 *
 *   Order matters: step 1 saves prev BEFORE integration so the renderer
 *   lerps between the OLD and NEW positions.  Step 3 (cap) before step 4
 *   (integrate) so a spike of wander can't briefly produce a velocity
 *   huge enough to teleport past the wall in one frame.
 *
 *
 * scene_draw(alpha):                                    (3 labelled steps)
 *   Step 1  compute_lerp_positions  → dpx/dpy/dcx/dcy caches
 *   Step 2  cell_used ← zero        per-frame VLA bool[rows][cols]
 *           scene_draw_connections  O(N²/2) pair scan + brightness buckets
 *                                    + thin-Bresenham draw_line
 *   Step 3  scene_draw_stars        paint star glyphs LAST (foreground wins)
 *
 *   `Scene *` is const throughout.  cell_used is the only allocation in
 *   the hot path — first-writer-wins prevents overlapping lines from
 *   producing mojibake.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * Keys
 *   q | Q | ESC      quit                spc        pause / resume
 *   r                randomise all stars
 *   = / +            add a star          -          remove a star (down to STARS_MIN)
 *   c                cycle connect preset (tight / normal / wide)
 *   t / T            next / prev theme
 *   ] / [            raise / lower sim_fps
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/constellation.c \
 *       -o constellation -lncurses -lm
 */

/* ── REFERENCES ──────────────────────────────────────────────────────── *
 *
 * PAPERS
 *   Bresenham, J. E. (1965)
 *     "Algorithm for computer control of a digital plotter"
 *     IBM Systems Journal 4(1): 25–30.
 *   Uhlenbeck, G. E. & Ornstein, L. S. (1930)
 *     "On the Theory of the Brownian Motion"
 *     Phys. Rev. 36: 823–841.
 *   Newell, M. E., Newell, R. G. & Sancha, T. L. (1972)
 *     "A solution to the hidden surface problem"
 *     Proc. ACM Annual Conference: 443–450.  (Painter's algorithm.)
 *
 * BOOKS
 *   Newman, M. — "Networks: An Introduction"  (Oxford UP, 2010)
 *     Ch. 6 covers proximity / threshold graphs.
 *   Press et al. — "Numerical Recipes in C"  (3rd ed., 2007)
 *     §1.1.3 squared-distance idiom; §17.1 fixed-step integration.
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

    HUD_COLS         = 64,   /* widened to fit [theme] suffix             */
    FPS_UPDATE_MS    = 500,

    STARS_DEFAULT    = 30,
    STARS_MIN        =  5,
    STARS_MAX        = 80,

    N_STAR_COLORS    =  6,   /* color pairs 1..6 for stars               */
    CONN_PAIR        =  7,   /* color pair for all connection lines       */
    HUD_PAIR         =  8,   /* bright yellow 226 + A_BOLD — top-right    */
    HINT_PAIR        =  9,   /* bright cyan   51  + A_BOLD — bottom-left  */
};

/* Sub-pixel units per terminal cell.  16/8 = 2 matches a typical
 * monospace cell's pixel aspect ratio, which is what makes physics in
 * (CELL_W, CELL_H) units render isotropic on screen. */
#define CELL_W   8
#define CELL_H  16

/* Star initial speed range (pixels/second).  Slow on purpose — fast
 * stars produce visible staircase aliasing on near-axis-aligned motion;
 * wander force + lerp interpolation handle the rest. */
#define SPEED_MIN   50.0f
#define SPEED_MAX  120.0f

/* Wander envelope: WANDER_ACCEL bounds the random-kick magnitude per
 * second; SPEED_CAP bounds the resulting velocity magnitude. */
#define WANDER_ACCEL  20.0f
#define SPEED_CAP    130.0f

/* Connection distance presets in PIXELS — cycled by the 'c' key.  At
 * 200 px on a 200×50 terminal (1600×800 px), each star's connect
 * circle covers ~10 % of screen area, yielding ~45 lines at N=30. */
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
 * ConstTheme — one palette: 6 star hues + 1 connection-line hue.
 *
 *   name        cycled label, shown in HUD
 *   star_fg256  256-colour palette (rich terminals)
 *   star_fg8    8-colour fallback  (basic terminals)
 *   conn_fg256  connection-line colour, 256-mode
 *   conn_fg8    connection-line colour, 8-mode
 *
 * Lifecycle: const table; theme_apply rebinds colour-pair IDs 1..7 to a
 *   chosen entry on startup and on every t/T keystroke.  Existing glyphs
 *   on screen pick up the new colour on the next mvaddch — no clear, no
 *   redraw.  HUD_PAIR=8 / HINT_PAIR=9 sit OUTSIDE this range so the
 *   chrome never changes colour as themes cycle.
 */
typedef struct {
    const char *name;
    int         star_fg256[6];   /* 256-colour palette (rich terminals)    */
    int         star_fg8  [6];   /* 8-colour fallback (basic terminals)    */
    int         conn_fg256;      /* connection-line colour, 256-mode       */
    int         conn_fg8;        /* connection-line colour, 8-mode         */
} ConstTheme;

#define N_THEMES  7

static const ConstTheme k_themes[N_THEMES] = {
    /* The classic — was the only palette before theme support was added. */
    { "night",
      {  15,  51,  39, 201, 147, 159 },
      { COLOR_WHITE,  COLOR_CYAN,  COLOR_BLUE,
        COLOR_MAGENTA, COLOR_CYAN,  COLOR_WHITE },
      24,  COLOR_BLUE },

    /* Greens and teals — northern-lights vibe. */
    { "aurora",
      {  46,  51,  87, 119, 156, 158 },
      { COLOR_GREEN, COLOR_CYAN,  COLOR_CYAN,
        COLOR_GREEN, COLOR_GREEN, COLOR_CYAN },
      30,  COLOR_GREEN },

    /* Saturated magentas / pinks — a star-forming region. */
    { "nebula",
      { 201, 207, 213, 219, 225, 159 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN },
      91,  COLOR_MAGENTA },

    /* Cool blues + whites — clear winter sky. */
    { "winter",
      { 195, 159, 153, 117,  75,  39 },
      { COLOR_WHITE, COLOR_CYAN, COLOR_CYAN,
        COLOR_CYAN,  COLOR_BLUE, COLOR_BLUE },
      24,  COLOR_BLUE },

    /* Warm reds → oranges → yellows — campfire / brazier sky. */
    { "ember",
      { 226, 220, 214, 208, 202, 196 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_RED,    COLOR_RED,    COLOR_RED },
      130, COLOR_RED },

    /* Deep indigos + cool whites — empty-space void. */
    { "void",
      { 252, 248, 141, 105,  99,  63 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_BLUE,
        COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE },
      53,  COLOR_BLUE },

    /* Whites + light greys — minimalist / B&W photograph. */
    { "mono",
      { 255, 253, 252, 250, 248, 246 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
        COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
      244, COLOR_WHITE },
};

/* theme_apply — rebind the 7 rendering pair IDs (1..7) to theme[idx]. */
static void theme_apply(int theme)
{
    const ConstTheme *th = &k_themes[theme];
    if (COLORS >= 256) {
        for (int i = 0; i < 6; i++)
            init_pair(1 + i, th->star_fg256[i], COLOR_BLACK);
        init_pair(CONN_PAIR, th->conn_fg256, COLOR_BLACK);
    } else {
        for (int i = 0; i < 6; i++)
            init_pair(1 + i, th->star_fg8[i],   COLOR_BLACK);
        init_pair(CONN_PAIR, th->conn_fg8,   COLOR_BLACK);
    }
}

/* color_init — start_color + apply initial theme + pin HUD/HINT pairs. */
static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);
    if (COLORS >= 256) {
        init_pair(HUD_PAIR,  226, -1);    /* bright yellow */
        init_pair(HINT_PAIR,  51, -1);    /* bright cyan   */
    } else {
        init_pair(HUD_PAIR,  COLOR_YELLOW, -1);
        init_pair(HINT_PAIR, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                    */
/* ===================================================================== */

/* pw / ph — cells → pixels.  The only callers of CELL_W/CELL_H proper. */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/* px_to_cell_x / _y — pixel → cell (round half up; see bounce_ball.c §4). */
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
 * Star — the actor.  Drifts across the screen with a bounded random
 * walk; connection lines are derived from pairs at draw time, NOT stored.
 *
 *   px, py            position THIS tick      (pixels, float)
 *   prev_px, prev_py  position at LAST tick   (pixels) — for render lerp
 *   vx, vy            velocity                (pixels/sec)
 *   color             colour-pair id          1..N_STAR_COLORS
 *   ch                glyph chosen at spawn from k_star_chars  ("*+o@.")
 *
 * Lifecycle:
 *   spawn at scene_init / on 'r' / from app_do_resize for out-of-bounds
 *   slots.  No destruction — Scene.n defines the live slice [0..n-1];
 *   "remove a star" is `n--`, slots stay allocated.
 *
 * Why prev_px / prev_py:
 *   wander adds random acceleration each tick; forward-extrapolation
 *   from the current state would diverge from the next real tick.  Lerp
 *   between (prev) and (curr) is exact for any acceleration profile.
 */
typedef struct {
    float px,      py;       /* current-tick position  (pixel space)    */
    float prev_px, prev_py;  /* previous-tick position (pixel space)    */
    float vx,      vy;       /* velocity (px/s)                         */
    int   color;             /* colour-pair id (1..N_STAR_COLORS)        */
    char  ch;                /* star glyph from k_star_chars            */
} Star;

/* Glyph alphabet for stars — cycled by index so 30 stars cover all 5. */
static const char k_star_chars[] = "*+o@.";
static const int  k_n_star_chars = (int)(sizeof k_star_chars - 1);

/* star_spawn — fill one Star slot with a random in-bounds pos + isotropic vel. */
static void star_spawn(Star *s, int idx, int cols, int rows)
{
    int pxw = pw(cols);
    int pxh = ph(rows);

    s->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
    s->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));
    s->prev_px = s->px;
    s->prev_py = s->py;

    /* Rejection-sample inside the unit disk for uniform direction. */
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
 * star_tick — DRIVER.  One frame of per-star physics.
 *
 *   Step 1   prev_px, prev_py ← px, py             save for render lerp
 *   Step 2   v += uniform(±WANDER_ACCEL) · dt      Ornstein-Uhlenbeck wander
 *   Step 3   if |v| > SPEED_CAP: v *= SPEED_CAP/|v| magnitude clamp
 *   Step 4   px, py += v · dt                      explicit Euler
 *   Step 5   for each wall: snap + negate normal vel    elastic reflection
 *
 * INPUTS / UNITS
 *   s              Star to advance (mutated)
 *   dt             frame time in seconds (typically 1 / sim_fps)
 *   max_px, max_py playable rectangle bounds in pixels: pw(cols)-1, ph(rows)-1
 *
 * WHY THE STEP ORDER MATTERS
 *   Step 1 must precede 2-4 so prev tracks the LAST tick's position, not
 *   this one — that's what makes render lerp interpolate between two
 *   known endpoints.  Step 3 (cap) before step 4 (integrate) so a spike
 *   of wander can't briefly produce a velocity huge enough to teleport
 *   past a wall in one frame.  Step 5 after step 4 so we test the
 *   FRESHLY integrated position against the bounds.
 *
 * WHY IT EXISTS (vs inlining in scene_tick)
 *   scene_tick stays at orchestrator level — "for each active star,
 *   advance".  If we ever swap wander for gravity or attractor physics,
 *   only star_tick changes.
 */
static void star_tick(Star *s, float dt, float max_px, float max_py)
{
    /* Step 1 — save position for render lerp. */
    s->prev_px = s->px;
    s->prev_py = s->py;

    /* Step 2 — wander: random acceleration in ±WANDER_ACCEL. */
    float ax = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    float ay = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
    s->vx += ax * dt;
    s->vy += ay * dt;

    /* Step 3 — magnitude clamp so wander can't run away. */
    float spd = sqrtf(s->vx * s->vx + s->vy * s->vy);
    if (spd > SPEED_CAP) {
        float inv = SPEED_CAP / spd;
        s->vx *= inv;
        s->vy *= inv;
    }

    /* Step 4 — explicit Euler integration. */
    s->px += s->vx * dt;
    s->py += s->vy * dt;

    /* Step 5 — elastic wall reflection (snap into bounds + negate vel). */
    if (s->px < 0.0f)    { s->px = 0.0f;     s->vx = -s->vx; }
    if (s->px > max_px)  { s->px = max_px;   s->vx = -s->vx; }
    if (s->py < 0.0f)    { s->py = 0.0f;     s->vy = -s->vy; }
    if (s->py > max_py)  { s->py = max_py;   s->vy = -s->vy; }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — owns the star pool plus run-time knobs.  The whole simulation
 * state in one struct.  No malloc after init; no other dynamic state.
 *
 *   stars            fixed-size pool of STARS_MAX = 80 stars
 *   n                active count; live slice is stars[0..n-1]
 *   paused           if true, scene_tick early-returns
 *   connect_preset   index into k_connect_presets — distance threshold
 *   current_theme    index into k_themes[] — palette
 *
 * The "active prefix" pattern (no per-slot `active` flag) works because
 * stars are interchangeable — once spawned, no slot identity matters.
 *   add a star    → star_spawn(&stars[n]); n++   (O(1) write)
 *   remove a star → n--                          (O(1), no resource freed)
 */
typedef struct {
    Star stars[STARS_MAX];
    int  n;                  /* active prefix: stars[0..n-1] live */
    bool paused;
    int  connect_preset;     /* index into k_connect_presets */
    int  current_theme;      /* index into k_themes[]        */
} Scene;

/* scene_init — zero the Scene, install defaults, spawn STARS_DEFAULT stars. */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->n              = STARS_DEFAULT;
    sc->connect_preset = 1;   /* "normal" */
    sc->current_theme  = 0;   /* "night"  */
    for (int i = 0; i < sc->n; i++)
        star_spawn(&sc->stars[i], i, cols, rows);
}

/*
 * scene_tick — DRIVER.  Per-frame physics orchestrator.
 *
 *   if paused:  return
 *   max_px, max_py ← pw(cols)-1, ph(rows)-1     right + bottom walls in pixels
 *   for i = 0..n-1:
 *     star_tick(stars[i], dt, max_px, max_py)
 *
 * INPUTS / UNITS
 *   sc          Scene mutated; every active star advances by dt
 *   dt          frame time in seconds (after any global speed scaling)
 *   cols, rows  terminal extent in cells; converted to pixel walls here
 *
 * Stars do NOT see each other in tick — that's a render-time concern,
 * handled by scene_draw_connections via the pair scan.  Tick is pure
 * physics, never touches ncurses; this is what lets a future test
 * harness or batch renderer drive the same Scene without scaffolding.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    if (sc->paused) return;

    float max_px = (float)(pw(cols) - 1);
    float max_py = (float)(ph(rows) - 1);

    for (int i = 0; i < sc->n; i++)
        star_tick(&sc->stars[i], dt, max_px, max_py);
}

/* draw_line — thin-Bresenham + slope glyph + stipple stride + first-writer-wins. */
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

/* clamp_cell_{x,y} — pin a cell index into [0, extent). */
static inline int clamp_cell_x(int v, int cols)
{
    if (v < 0)     return 0;
    if (v >= cols) return cols - 1;
    return v;
}
static inline int clamp_cell_y(int v, int rows)
{
    if (v < 0)     return 0;
    if (v >= rows) return rows - 1;
    return v;
}

/* compute_lerp_positions — fill dpx/dpy/dcx/dcy caches for n active stars. */
static void compute_lerp_positions(const Scene *sc, float alpha,
                                   int cols, int rows,
                                   float *dpx, float *dpy,
                                   int   *dcx, int   *dcy)
{
    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        dpx[i] = s->prev_px + (s->px - s->prev_px) * alpha;
        dpy[i] = s->prev_py + (s->py - s->prev_py) * alpha;
        dcx[i] = clamp_cell_x(px_to_cell_x(dpx[i]), cols);
        dcy[i] = clamp_cell_y(px_to_cell_y(dpy[i]), rows);
    }
}

/* pick_connect_style — three brightness buckets from ratio ∈ [0, 1). */
static void pick_connect_style(float ratio, chtype *out_attr, int *out_stipple)
{
    if (ratio < 0.50f) {
        *out_attr    = COLOR_PAIR(CONN_PAIR) | A_BOLD;
        *out_stipple = 1;
    } else if (ratio < 0.75f) {
        *out_attr    = COLOR_PAIR(CONN_PAIR);
        *out_stipple = 1;
    } else {
        *out_attr    = COLOR_PAIR(CONN_PAIR);
        *out_stipple = 2;
    }
}

/* scene_draw_connections — O(N²/2) pair scan; squared-distance cull, then draw. */
static void scene_draw_connections(const Scene *sc, WINDOW *w,
                                   int cols, int rows,
                                   const float *dpx, const float *dpy,
                                   const int   *dcx, const int   *dcy,
                                   float connect_dist, bool *cell_used)
{
    float cdist_sq = connect_dist * connect_dist;

    for (int i = 0; i < sc->n - 1; i++) {
        for (int j = i + 1; j < sc->n; j++) {
            float dx_px   = dpx[j] - dpx[i];
            float dy_px   = dpy[j] - dpy[i];
            float dist_sq = dx_px * dx_px + dy_px * dy_px;
            if (dist_sq >= cdist_sq) continue;        /* cull, no sqrt */

            float dist  = sqrtf(dist_sq);
            float ratio = dist / connect_dist;        /* 0=close, 1=edge */

            chtype attr;
            int    stipple;
            pick_connect_style(ratio, &attr, &stipple);

            draw_line(w,
                      dcx[i], dcy[i], dcx[j], dcy[j],
                      attr, stipple,
                      cols, rows,
                      cell_used);
        }
    }
}

/* scene_draw_stars — paint star glyphs LAST so they sit on top of any line. */
static void scene_draw_stars(const Scene *sc, WINDOW *w,
                             const int *dcx, const int *dcy)
{
    for (int i = 0; i < sc->n; i++) {
        const Star *s = &sc->stars[i];
        wattron(w, COLOR_PAIR(s->color) | A_BOLD);
        mvwaddch(w, dcy[i], dcx[i], (chtype)(unsigned char)s->ch);
        wattroff(w, COLOR_PAIR(s->color) | A_BOLD);
    }
}

/*
 * scene_draw — DRIVER.  Per-frame render orchestrator at sub-tick alpha.
 *
 *   Step 1   compute_lerp_positions  → dpx/dpy/dcx/dcy caches (n stars)
 *   Step 2   cell_used ← zero        per-frame VLA bool[rows][cols]
 *            scene_draw_connections  pair scan + brightness buckets +
 *                                    thin-Bresenham draw_line
 *   Step 3   scene_draw_stars        paint glyphs LAST (foreground wins)
 *
 * INPUTS / UNITS
 *   sc          Scene to read (const).
 *   w           destination WINDOW (typically stdscr).
 *   cols, rows  terminal extent in cells; sizes the cell_used VLA.
 *   alpha       ∈ [0, 1).  0 = at last tick; →1 = at next tick.
 *
 * WHY THE LAYER ORDER
 *   Lines first, stars last (painter's algorithm).  Reverse and a star's
 *   glyph can be overwritten by a later line passing through the same
 *   cell — visible "missing star" artefact.
 *
 * WHY IT EXISTS (vs inlining the three steps in screen_draw)
 *   The cache arrays (dpx/dpy/dcx/dcy) are computed ONCE here and read
 *   by both downstream passes.  Splitting further would either re-lerp
 *   (wasted work) or thread a "PrecomputedPositions" struct through
 *   every signature.  Three named steps in one function reads as the
 *   algorithm.
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* Step 1 — lerp every star into pixel + cell caches. */
    float dpx[STARS_MAX], dpy[STARS_MAX];
    int   dcx[STARS_MAX], dcy[STARS_MAX];
    compute_lerp_positions(sc, alpha, cols, rows, dpx, dpy, dcx, dcy);

    /* Step 2 — proximity-graph pair scan, drawing connection lines.
     * cell_used is cleared per frame; first-writer wins inside draw_line. */
    bool cell_used[rows][cols];
    memset(cell_used, 0, sizeof cell_used);

    float connect_dist = k_connect_presets[sc->connect_preset];
    scene_draw_connections(sc, w, cols, rows,
                           dpx, dpy, dcx, dcy,
                           connect_dist, &cell_used[0][0]);

    /* Step 3 — paint stars LAST so they sit on top of any line glyphs. */
    scene_draw_stars(sc, w, dcx, dcy);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s, int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void format_hud_text(char *buf, size_t n,
                            const Scene *sc, double fps, int sim_fps)
{
    snprintf(buf, n,
             " %5.1f fps  stars:%-2d  conn:%-6s  spd:%-2d  [%s]  %s ",
             fps, sc->n,
             k_connect_names[sc->connect_preset],
             sim_fps,
             k_themes[sc->current_theme].name,
             sc->paused ? "PAUSED " : "running");
}

static void paint_hud_topright(int row, int cols, const char *text)
{
    int hud_x = cols - (int)strlen(text);
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(row, hud_x, "%s", text);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

static void paint_hint_bottomleft(int row, const char *text)
{
    attron(COLOR_PAIR(HINT_PAIR) | A_BOLD);
    mvprintw(row, 0, "%s", text);
    attroff(COLOR_PAIR(HINT_PAIR) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    char hud_text[HUD_COLS + 1];
    format_hud_text(hud_text, sizeof hud_text, sc, fps, sim_fps);
    paint_hud_topright(0, s->cols, hud_text);

    paint_hint_bottomleft(s->rows - 1,
        " q/ESC:quit  spc:pause  +/-:stars  c:connect"
        "  r:randomise  ]/[:speed  t/T:theme ");
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

    case 't':
        sc->current_theme = (sc->current_theme + 1) % N_THEMES;
        theme_apply(sc->current_theme);
        break;
    case 'T':
        sc->current_theme = (sc->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->current_theme);
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

static void app_install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

int main(void)
{
    srand((unsigned int)clock_ns());
    app_install_signals();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, app->scene.current_theme);
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
