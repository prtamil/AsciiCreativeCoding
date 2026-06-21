/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * jellyfish.c — bioluminescent jellyfish that swim by pulsing their bells.
 *
 * Each jelly cycles: rest and sink → squeeze (jet upward) → coast → bloom open.
 * Biology refs: Dabiri et al. (2005), Gemmell et al. (2013).
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

/* ── §1 config — all the tunable numbers, named once ── */

#define CELL_W   8
#define CELL_H  16

enum {
    N_JELLIES_DEFAULT = 1,
    N_JELLIES_MAX     = 8,
    N_TENTACLES       = 3,     /* left, center, right                     */
    TENTACLE_SEGS     = 20,
    FPS_UPDATE_MS     = 500,
    TARGET_FPS        = 60,
};

/* Bell geometry (pixels) */
#define BELL_RX_BASE    72.0f
#define BELL_RY_BASE   144.0f

/* How far the bell is allowed to squeeze shut (1 = open, lower = tighter). */
#define MIN_OPEN         0.85f  /* tightest crown height                   */
#define HEAD_MIN_OPEN    0.85f  /* tightest body width — squeezes harder   */

/* Bell outline is sliced into horizontal bands by height (1 = top/apex,
 * 0 = bottom/rim). Each threshold is where one band hands off to the next. */
#define BELL_NY_ROOF      0.94f  /* above this: flat closing roof  '_'        */
#define BELL_NY_CROWN     0.88f  /* down to here: rounded shoulders '(' ')'   */
#define BELL_CROWN_SPAN   0.12f  /* height of the crown band                  */
#define BELL_NY_SHOULDER  0.40f  /* down to here: steep side walls '/' '\\'   */
#define BELL_NY_RIM       0.14f  /* below this: flat rim line     '-'         */
#define BELL_INTERIOR_R2  0.60f  /* past this from centre: faint dots '.'     */

/* ── Swim physics: jet up, then settle back down ── */
#define GRAVITY          30.0f  /* downward accel px/s² while resting (sinking) */
#define JET_THRUST      190.0f  /* upward speed kick at the end of the squeeze */
#define DRAG_VY           2.4f  /* how fast vertical speed bleeds away         */
#define IDLE_DUR_MIN      0.55f /* shortest rest between pulses (s)            */
#define IDLE_DUR_MAX      1.10f /* longest rest between pulses (s)             */
#define DUR_CONTRACT      0.19f /* squeeze time (s) — quick                    */
#define DUR_GLIDE         0.38f /* coast time (s) after the jet                */
#define DUR_EXPAND        0.68f /* bloom-open time (s) — slow                  */
#define TENT_LAG          0.09f /* how much tentacles trail when moving fast   */

/* Tentacle sway strength at each point in the pulse (1 = full wave, ~0 = stiff). */
#define WAVE_FULL          1.00f /* full sway — resting / fully open bell      */
#define WAVE_GLIDE         0.12f /* nearly straight while coasting             */
#define WAVE_CONTRACT_END  0.15f /* sway left at the end of the squeeze        */
#define WAVE_CONTRACT_DROP 0.85f /* sway lost over the squeeze                 */
#define WAVE_EXPAND_RISE   0.88f /* sway regained as the bell blooms           */

/* Shape of the tentacle wave: segment length, sway size, ripple spacing/speed. */
#define TENT_SEG_PX     14.0f
#define TENT_WAVE_AMP   19.0f
#define TENT_WAVE_K      0.30f
#define TENT_WAVE_SPD    3.2f
#define TENT_PHASE_OFF   0.80f

/* Tentacles fade from solid near the bell to faint at the tips (0=root, 1=tip). */
#define TENT_DEPTH_NEAR   0.20f  /* up to here: bold  '|'                    */
#define TENT_DEPTH_MID    0.45f  /* up to here: plain '|'                    */
#define TENT_DEPTH_FAR    0.70f  /* up to here: dim   ':' ; beyond: dim '.'  */

/* Gentle side-to-side drift of each jelly. */
#define JELLY_DRIFT_A   15.0f
#define JELLY_DRIFT_HZ   0.30f

/* Where jellies appear, as fractions of the world box (px = fraction*max_*). */
#define SPAWN_HERO_CY     0.55f  /* lone hero jelly starts just below mid       */
#define SPAWN_ZONE_MARGIN 0.10f  /* left inset within each fleet lane           */
#define SPAWN_ZONE_FILL   0.80f  /* usable width of a lane                      */
#define SPAWN_BAND_TOP    0.85f  /* topmost spread row (fraction of height)     */
#define SPAWN_BAND_RANGE  0.70f  /* vertical spread span across the fleet       */
#define SPAWN_JITTER      0.25f  /* random cy jitter as fraction of a lane      */
#define SPAWN_BELOW       0.40f  /* extra below-screen offset when entering     */
#define RESPAWN_CY        0.85f  /* recycle drop-in height (fraction)           */
#define RESPAWN_LANE_LO   0.15f  /* recycle lane low bound (fraction)           */
#define RESPAWN_LANE_SPAN 0.70f  /* recycle lane span (fraction)               */

#define TAU  6.28318530f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* One colour pair per jelly, plus HUD/hint pairs. §4 jelly_spawn picks a jelly's
 * pair from jelly_cp[]; §5 color_init defines what each pair actually looks like. */
enum {
    CP_J0 = 1, CP_J1, CP_J2, CP_J3,
    CP_J4,     CP_J5, CP_J6, CP_J7,
    CP_HUD, CP_HINT,
};

static const int jelly_cp[N_JELLIES_MAX] = {
    CP_J0, CP_J1, CP_J2, CP_J3, CP_J4, CP_J5, CP_J6, CP_J7
};

/* ── §2 performance — read the clock and sleep ── */

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

/* ── §3 logic — pure timing curves, no state touched ── */

/* Smooth S-curve: 0 in, 1 out, eased at both ends. */
static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* Two motion curves for the bell, both take t in [0,1]:
 *   contract starts fast then slows (a quick snap shut),
 *   expand starts slow then speeds up (a gentle bloom open).    */
static float curve_contract(float t)
{
    float inv = 1.0f - t;
    return 1.0f - inv * inv;
}
static float curve_expand(float t)
{
    return smoothstep(t);
}

/* ── §4 simulation — the only code that changes jellyfish/scene state ── */

/* The four stages a jellyfish loops through forever, in this order:
 *   IDLE → CONTRACT → GLIDE → EXPAND → IDLE.
 * A real jelly's bell is basically one muscle: it can only squeeze (which
 * jets water out and pushes it forward) then relax. So its swimming is
 * naturally a repeating pulse, and one stage value picks both the physics
 * rule and the bell shape for that moment. How long each stage lasts is set
 * by the IDLE_DUR_* / DUR_* numbers in §1. */
typedef enum {
    PHASE_IDLE,      /* bell open; just sinks and waits, then fires a squeeze   */
    PHASE_CONTRACT,  /* bell squeezes shut; an upward kick lands at the very end */
    PHASE_GLIDE,     /* bell held shut; coast upward while drag slows it down    */
    PHASE_EXPAND,    /* bell slowly blooms back open, then loops to IDLE         */
} PulsePhase;

/* Everything about one jellyfish. It's one flat struct on purpose: nearly all
 * of its look comes from a single number, bell_open (how squeezed the bell is),
 * and the rest are cheap things recomputed from it each frame. All lengths are
 * in pixels — there are CELL_W=8 by CELL_H=16 pixels per terminal character,
 * and the pixel→character conversion happens only at draw time. */
typedef struct {
    /* Position and side-to-side drift, in pixels. Up/down is real physics;
     * left/right is just a cosmetic wobble around drift_cx. */
    float cx, cy;            /* centre of the bell (px); only cy moves by physics */
    float drift_cx;          /* the x it wobbles around (px)                      */
    float drift_phase;       /* where it is in its side-to-side wobble (radians)  */

    /* The swim engine: which pulse stage we're in, and the up/down motion. */
    float      vy;           /* up/down speed (px/s); negative = rising. Set to
                              * -JET_THRUST at the end of the squeeze, then bled
                              * off a little every frame by drag                  */
    float      bell_open;    /* how open the bell is, 1 = relaxed down to MIN_OPEN
                              * = fully squeezed. Drives crown_f and head_f        */
    float      idle_dur;     /* how long to rest this cycle (s), re-rolled in
                              * [IDLE_DUR_MIN..MAX] so jellies don't pulse in sync */
    PulsePhase phase;        /* current pulse stage — picks the physics rule      */
    float      phase_t;      /* seconds spent in this stage; once it passes the
                              * stage's duration, move to the next stage           */

    /* Tentacles aren't physically simulated — each is just a moving sine wave.
     * wave_scale dials its size, so the sway goes limp during the fast jet. */
    float tent_phase[N_TENTACLES];  /* where each tentacle's wave is (radians)    */
    float wave_scale;        /* sway size, 1 = full (resting) down to ~0.12
                              * (streaming straight back while coasting)           */

    /* Two shape numbers derived from bell_open and cached so the renderer never
     * has to know the pulse stages exist — it just scales geometry by these. */
    float crown_f;           /* dome HEIGHT scale (the bell squashes shorter)      */
    float head_f;            /* dome WIDTH scale (the body pinches narrower)       */

    int cp;                  /* this jelly's colour-pair id, picked at spawn so
                              * each one glows a different hue                      */
} Jellyfish;

/* Random float in [0,1). */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* Decide where a new jelly starts. The first jelly (idx 0, spread) sits centred
 * near mid-height; the rest get spread across the width, and either staggered
 * down the screen (spread) or started just below it so they swim up into view. */
static void jelly_seed_position(Jellyfish *j, int idx,
                                float max_px, float max_py, bool spread)
{
    if (idx == 0 && spread) {
        j->drift_cx = max_px * 0.5f;
        j->cx       = j->drift_cx;
        j->cy       = max_py * SPAWN_HERO_CY;
        return;
    }

    /* Horizontal: a random x inside this jelly's own vertical lane */
    float zone_w  = max_px / (float)N_JELLIES_MAX;
    float zone_lo = zone_w * (idx % N_JELLIES_MAX) + zone_w * SPAWN_ZONE_MARGIN;
    float zone_hi = zone_lo + zone_w * SPAWN_ZONE_FILL;
    j->drift_cx   = zone_lo + randf() * (zone_hi - zone_lo);
    j->cx         = j->drift_cx;

    /* Vertical: spread fans them down the screen; otherwise start below it */
    float slot   = (float)(idx % N_JELLIES_MAX) / (float)(N_JELLIES_MAX - 1);
    float jitter = (max_py / (float)N_JELLIES_MAX) * SPAWN_JITTER;
    j->cy = spread
          ? max_py * (SPAWN_BAND_TOP - SPAWN_BAND_RANGE * slot)
            + (randf() - 0.5f) * jitter
          : max_py + TENTACLE_SEGS * TENT_SEG_PX * SPAWN_BELOW + BELL_RY_BASE;
}

static void jelly_spawn(Jellyfish *j, int idx,
                        float max_px, float max_py, bool spread)
{
    jelly_seed_position(j, idx, max_px, max_py, spread);

    /* Random starting wave/drift phases, and this jelly's glow colour */
    j->drift_phase = randf() * TAU;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] = randf() * TAU;
    j->cp = jelly_cp[idx % N_JELLIES_MAX];

    /* Start resting, with a random offset so the jellies don't all pulse together */
    j->vy         = 0.0f;
    j->bell_open  = 1.0f;
    j->wave_scale = WAVE_FULL;
    j->phase      = PHASE_IDLE;
    j->phase_t    = randf() * IDLE_DUR_MAX;
    j->idle_dur   = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);

    j->crown_f = j->head_f = 1.0f;
}

/* Turn bell_open into the two shape numbers the renderer draws with. The body
 * width is made to pinch harder than the dome height shrinks, which reads more
 * like a real squeeze. Called once per tick; the renderer never calls it. */
static void bell_set_render_fractions(Jellyfish *j)
{
    float open01 = (j->bell_open - MIN_OPEN) / (1.0f - MIN_OPEN); /* 0=shut 1=open */
    j->crown_f = j->bell_open;
    j->head_f  = HEAD_MIN_OPEN + (1.0f - HEAD_MIN_OPEN) * open01;
}

/* Run one step of the pulse cycle: pick this stage's bell shape, sway, and
 * up/down kick, advance to the next stage when its time is up, then refresh
 * the cached render shape numbers. */
static void jelly_pulse_tick(Jellyfish *j, float dt)
{
    j->phase_t += dt;

    switch (j->phase) {

    case PHASE_IDLE:
        /* Bell open, slowly sinking, tentacles swaying freely */
        j->bell_open  = 1.0f;
        j->wave_scale = WAVE_FULL;
        j->vy += GRAVITY * dt;
        if (j->phase_t >= j->idle_dur) {
            j->phase   = PHASE_CONTRACT;
            j->phase_t = 0.0f;
        }
        break;

    case PHASE_CONTRACT: {
        /* Bell snaps shut and the sway dies down; the upward kick lands at the end */
        float frac = j->phase_t / DUR_CONTRACT;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * (1.0f - curve_contract(frac));
        j->wave_scale = WAVE_FULL - WAVE_CONTRACT_DROP * frac;
        if (j->phase_t >= DUR_CONTRACT) {
            j->bell_open  = MIN_OPEN;
            j->wave_scale = WAVE_CONTRACT_END;
            j->vy         = -JET_THRUST;
            j->phase      = PHASE_GLIDE;
            j->phase_t    = 0.0f;
        }
        break;
    }

    case PHASE_GLIDE:
        /* Coasting upward, tentacles streaming nearly straight behind */
        j->bell_open  = MIN_OPEN;
        j->wave_scale = WAVE_GLIDE;
        if (j->phase_t >= DUR_GLIDE) {
            j->phase   = PHASE_EXPAND;
            j->phase_t = 0.0f;
        }
        break;

    case PHASE_EXPAND: {
        /* Bell slowly blooms open and the sway comes back */
        float frac = j->phase_t / DUR_EXPAND;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * curve_expand(frac);
        j->wave_scale = WAVE_GLIDE + WAVE_EXPAND_RISE * smoothstep(frac);
        if (j->phase_t >= DUR_EXPAND) {
            j->bell_open  = 1.0f;
            j->wave_scale = WAVE_FULL;
            j->phase      = PHASE_IDLE;
            j->phase_t    = 0.0f;
            j->idle_dur   = IDLE_DUR_MIN
                          + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
        }
        break;
    }
    }

    bell_set_render_fractions(j);
}

/* Once a jelly (tentacles and all) has swum off the top, drop it back near the
 * bottom in a fresh column and restart it, so they keep streaming up forever. */
static void jelly_wrap_offscreen(Jellyfish *j, float max_px, float max_py)
{
    float tent_extent = TENTACLE_SEGS * TENT_SEG_PX;
    if (j->cy + tent_extent >= 0.0f) return;   /* still partly on screen */

    j->cy          = max_py * RESPAWN_CY;
    j->drift_cx    = max_px * (RESPAWN_LANE_LO + RESPAWN_LANE_SPAN * randf());
    j->drift_phase = randf() * TAU;
    j->vy          = 0.0f;
    j->phase       = PHASE_IDLE;
    j->phase_t     = 0.0f;
    j->idle_dur    = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
    j->bell_open   = 1.0f;
    j->wave_scale  = WAVE_FULL;
    j->crown_f = j->head_f = 1.0f;
}

static void jelly_tick(Jellyfish *j, float dt, float max_px, float max_py)
{
    jelly_pulse_tick(j, dt);

    j->vy *= expf(-DRAG_VY * dt);                  /* up/down speed bleeds off   */

    /* Advance the drift wobble and each tentacle's wave */
    j->drift_phase += TAU * JELLY_DRIFT_HZ * dt;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] += TENT_WAVE_SPD * j->wave_scale * dt;

    /* Move: up/down from speed, left/right from the drift wobble */
    j->cy += j->vy * dt;
    j->cx  = j->drift_cx + JELLY_DRIFT_A * sinf(j->drift_phase);

    /* Keep the drift from walking off the sides */
    if (j->cx < BELL_RX_BASE)            j->cx = BELL_RX_BASE;
    if (j->cx > max_px - BELL_RX_BASE)   j->cx = max_px - BELL_RX_BASE;

    jelly_wrap_offscreen(j, max_px, max_py);
}

/* The whole simulated world. The jelly array is allocated full size up front and
 * never grown or freed — n_jellies just says how many are currently alive, so
 * the +/- keys never allocate anything at runtime. max_px/max_py are the world
 * size in pixels (from the terminal size), so the same swim looks the same at
 * any terminal size. */
typedef struct {
    Jellyfish jellies[N_JELLIES_MAX]; /* fixed pool; only the first n_jellies live */
    int       n_jellies;     /* how many are alive, 1..N_JELLIES_MAX (+/- keys)    */
    bool      paused;        /* freezes the sim; drawing keeps going               */
    float     max_px, max_py;/* world size in px; refreshed at init and on resize  */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px    = (float)(cols * CELL_W);
    s->max_py    = (float)(rows * CELL_H);
    s->n_jellies = N_JELLIES_DEFAULT;
    s->paused    = false;
    for (int i = 0; i < N_JELLIES_MAX; i++)
        jelly_spawn(&s->jellies[i], i, s->max_px, s->max_py, /*spread=*/true);
}

/* Advance every live jelly by one step (does nothing while paused). */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    for (int i = 0; i < s->n_jellies; i++)
        jelly_tick(&s->jellies[i], dt, s->max_px, s->max_py);
}

/* ── §5 render — draw the scene; only reads sim state ── */

/* Pixel position → terminal character cell, used only when drawing. */
static inline int px_to_col(float px) { return (int)floorf(px / (float)CELL_W); }
static inline int px_to_row(float py) { return (int)floorf(py / (float)CELL_H); }

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_J0,  51, -1);
        init_pair(CP_J1, 105, -1);
        init_pair(CP_J2, 213, -1);
        init_pair(CP_J3,  86, -1);
        init_pair(CP_J4, 159, -1);
        init_pair(CP_J5, 141, -1);
        init_pair(CP_J6, 219, -1);
        init_pair(CP_J7, 123, -1);
        init_pair(CP_HUD,  226, -1);   /* bright yellow on default bg */
        init_pair(CP_HINT,  51, -1);   /* bright cyan   on default bg */
    } else {
        init_pair(CP_J0, COLOR_CYAN,    -1);
        init_pair(CP_J1, COLOR_BLUE,    -1);
        init_pair(CP_J2, COLOR_MAGENTA, -1);
        init_pair(CP_J3, COLOR_GREEN,   -1);
        init_pair(CP_J4, COLOR_WHITE,   -1);
        init_pair(CP_J5, COLOR_CYAN,    -1);
        init_pair(CP_J6, COLOR_MAGENTA, -1);
        init_pair(CP_J7, COLOR_BLUE,    -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* Draw one horizontal row of the bell. Based on how high up the row sits (ny),
 * pick which part of the outline it is (roof, crown, walls, rim) and fill in
 * that part's characters between the bell's left and right edges. An early
 * return just means this row is finished. */
static void draw_dome_row(WINDOW *win, const Jellyfish *j,
                          int r, float ny, int cols)
{
    float abs_ny  = fabsf(ny);
    float nx_edge = sqrtf(1.0f - ny * ny);          /* how wide the bell is at this row */
    float Rx      = BELL_RX_BASE * j->head_f;        /* bell width, pinched by head_f    */
    int   cl      = px_to_col(j->cx - Rx * nx_edge);
    int   cr      = px_to_col(j->cx + Rx * nx_edge);

    /* closing roof */
    if (abs_ny > BELL_NY_ROOF) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        for (int c = cl; c <= cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* rounded crown */
    if (abs_ny > BELL_NY_CROWN) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        if (cl >= 0 && cl < cols) mvwaddch(win, r, cl, '(');
        if (cr >= 0 && cr < cols && cr != cl) mvwaddch(win, r, cr, ')');
        for (int c = cl + 1; c < cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* side characters — steep walls, gentler shoulders, or the rim */
    chtype lch, rch;
    attr_t ea;
    if (abs_ny > BELL_NY_SHOULDER) {
        lch = '/';  rch = '\\'; ea = A_BOLD;
    } else if (abs_ny > BELL_NY_RIM) {
        lch = '(';  rch = ')';  ea = A_BOLD;
    } else {
        lch = '~';  rch = '~';  ea = A_NORMAL;
    }

    if (cl >= 0 && cl < cols) {
        wattron(win, COLOR_PAIR(j->cp) | ea);
        mvwaddch(win, r, cl, lch);
        wattroff(win, COLOR_PAIR(j->cp) | ea);
    }
    if (cr >= 0 && cr < cols && cr != cl) {
        wattron(win, COLOR_PAIR(j->cp) | ea);
        mvwaddch(win, r, cr, rch);
        wattroff(win, COLOR_PAIR(j->cp) | ea);
    }

    /* rim line */
    if (abs_ny <= BELL_NY_RIM) {
        wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
        for (int c = cl; c <= cr; c++)
            if (c >= 0 && c < cols) mvwaddch(win, r, c, '-');
        wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
        return;
    }

    /* faint dots filling the outer part of the body, leaving the centre clear */
    for (int c = cl + 1; c < cr; c++) {
        if (c < 0 || c >= cols) continue;
        float pcx = c * (float)CELL_W + (float)CELL_W * 0.5f;
        float nx  = (pcx - j->cx) / (BELL_RX_BASE * j->head_f + 0.001f);
        float r2  = nx * nx + ny * ny;
        if (r2 < BELL_INTERIOR_R2) continue;
        wattron(win, COLOR_PAIR(j->cp) | A_DIM);
        mvwaddch(win, r, c, '.');
        wattroff(win, COLOR_PAIR(j->cp) | A_DIM);
    }
}

/* Draw the whole bell, one row at a time. When the bell is squeezed its top is
 * lower, so we skip rows above that lowered top; the rest go to draw_dome_row. */
static void draw_bell(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    float Ry = BELL_RY_BASE;                          /* bell height (px) */

    /* When squeezed, the top of the bell sits lower — skip rows above it */
    float crown_ny_limit = BELL_NY_CROWN + BELL_CROWN_SPAN * j->crown_f;

    int row_top = px_to_row(j->cy - Ry) - 1;
    int row_bot = px_to_row(j->cy)     + 1;
    if (row_top < 0)      row_top = 0;
    if (row_bot >= rows)  row_bot = rows - 1;

    for (int r = row_top; r <= row_bot; r++) {
        float py = r * (float)CELL_H + (float)CELL_H * 0.5f;
        float ny = (py - j->cy) / Ry;                /* 0 at the rim, -1 at the top */
        if (ny > 0.0f || ny < -1.0f) continue;       /* below the rim or above the top */
        if (fabsf(ny) > crown_ny_limit) continue;    /* above the squeezed top         */
        draw_dome_row(win, j, r, ny, cols);
    }
}

/* How far this tentacle segment swings sideways: a ripple running down the
 * tentacle, scaled by wave_scale so the sway goes limp during the fast jet. */
static float tentacle_wave_offset(const Jellyfish *j, int k, int seg)
{
    return TENT_WAVE_AMP * j->wave_scale
         * sinf(j->tent_phase[k] - seg * TENT_WAVE_K + (float)k * TENT_PHASE_OFF);
}

/* Pick the character (and brightness, via *attr) for a tentacle segment, going
 * from solid bars near the bell to faint dots at the tip. */
static char tentacle_glyph(float depth, attr_t *attr)
{
    if (depth < TENT_DEPTH_NEAR) { *attr = A_BOLD;   return '|'; }
    if (depth < TENT_DEPTH_MID)  { *attr = A_NORMAL; return '|'; }
    if (depth < TENT_DEPTH_FAR)  { *attr = A_DIM;    return ':'; }
    *attr = A_DIM; return '.';
}

/* Draw the tentacles hanging from the rim. Each is a column of segments that
 * sway sideways and trail behind when the jelly is moving fast. */
static void draw_tentacles(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    float Rx        = BELL_RX_BASE * j->head_f;
    float lag_total = -j->vy * TENT_LAG;   /* faster motion = more trailing      */

    for (int k = 0; k < N_TENTACLES; k++) {
        float t_frac = (N_TENTACLES > 1)
                     ? (float)k / (float)(N_TENTACLES - 1) : 0.5f;
        float base_x = j->cx + (t_frac - 0.5f) * 2.0f * Rx;   /* where it hangs from */

        for (int seg = 0; seg < TENTACLE_SEGS; seg++) {
            float depth = (float)seg / (float)(TENTACLE_SEGS - 1);  /* 0=root 1=tip */
            float px = base_x + tentacle_wave_offset(j, k, seg);
            float py = j->cy + seg * TENT_SEG_PX + lag_total * depth;

            int c = px_to_col(px);
            int r = px_to_row(py);
            if (c < 0 || c >= cols || r < 0 || r >= rows) continue;

            attr_t at;
            char   ch = tentacle_glyph(depth, &at);
            wattron(win, COLOR_PAIR(j->cp) | at);
            mvwaddch(win, r, c, ch);
            wattroff(win, COLOR_PAIR(j->cp) | at);
        }
    }
}

/* On-screen readout: stats top-right, key hints bottom-left, kept bright so
 * they stay readable over the animation. */
static void draw_hud(WINDOW *win, const Scene *s, int cols, int rows, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  jellies:%d  %s ",
             fps, s->n_jellies, s->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, x, "%s", buf);
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);

    wattron(win, COLOR_PAIR(CP_HINT) | A_BOLD);
    mvwprintw(win, rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:count ");
    wattroff(win, COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(const Scene *s, WINDOW *win,
                       int cols, int rows, double fps)
{
    for (int i = 0; i < s->n_jellies; i++) {
        draw_tentacles(win, &s->jellies[i], cols, rows);
        draw_bell(win, &s->jellies[i], cols, rows);
    }
    draw_hud(win, s, cols, rows, fps);
}

/* The terminal we draw on. ncurses owns the actual screen buffer, so all we keep
 * is its current size in characters; that lets the draw loop avoid asking ncurses
 * for the size every frame. Refreshed at startup and whenever the window resizes. */
typedef struct {
    int cols, rows;          /* terminal width/height in characters */
} Screen;

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

static void screen_render(const Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §6 app — main loop, key handling, and frame timing ── */

/* Ties the world to the terminal. It's a single global because signal handlers
 * can't be passed any data — they can only reach program state through a
 * file-scope object. The two flags are how a signal talks to the main loop, and
 * sig_atomic_t is the type C guarantees is safe to set inside a handler. */
typedef struct {
    Scene                 scene;       /* the simulated world (§4)              */
    Screen                screen;      /* the terminal we draw on (§5)          */
    volatile sig_atomic_t running;     /* a quit signal clears this → loop ends */
    volatile sig_atomic_t need_resize; /* a resize signal sets this             */
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

/* Handle a keypress. Returns false to quit. */
static bool app_key(App *app, int ch)
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
        if (sc->n_jellies < N_JELLIES_MAX) {
            int i = sc->n_jellies++;
            jelly_spawn(&sc->jellies[i], i,
                        sc->max_px, sc->max_py, /*spread=*/false);
        }
        break;
    case '-':
        if (sc->n_jellies > 1) sc->n_jellies--;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev      = clock_ns();
    int64_t fps_acc   = 0;
    int     fps_count = 0;
    double  fps_disp  = 0.0;

    while (app->running) {

        /* Apply a pending resize before we measure the frame time */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->scene.max_px = (float)(app->screen.cols * CELL_W);
            app->scene.max_py = (float)(app->screen.rows * CELL_H);
            app->need_resize  = 0;
            prev = clock_ns();
        }

        /* Time since last frame, capped so a long stall can't fast-forward the sim */
        int64_t now   = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt);

        /* Update the displayed fps a couple of times a second */
        fps_count++;
        fps_acc += dt_ns;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_acc / (double)NS_PER_SEC);
            fps_count = 0;
            fps_acc   = 0;
        }

        screen_render(&app->screen, &app->scene, fps_disp);

        /* Sleep off the rest of the frame to hold a steady frame rate */
        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        int ch = getch();
        if (ch != ERR && !app_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
