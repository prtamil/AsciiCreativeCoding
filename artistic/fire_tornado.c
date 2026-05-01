/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire_tornado.c — fire tornado from a swirling pool of embers (Stage 2)
 *
 * DEMO: A tall conical column of embers rotates around a central vertical
 *       axis, narrowing as it rises. Each ember lives in cylindrical
 *       coordinates (height, phase, radius); a side-view 2-D projection
 *       maps cylindrical position to screen cells, with depth conveyed
 *       by which side of the column the ember is currently on (sin of
 *       its phase). Heat fades from white-hot at the base to dim red at
 *       the top, both via a 5-stop ASCII ramp (`# o * . `) and a
 *       matching colour ramp.
 *
 *       Stage 2 adds three layers of embellishment on top of Stage 1:
 *         • A 1-D heat strip at the ground row that flickers, diffuses
 *           and self-injects fuel (a tiny cellular-automaton fire mat).
 *         • Sparks — particles spawned at random ember positions with
 *           outward radial velocity; they fly out, fall under gravity,
 *           cool fast, and disappear.
 *         • Wind — a slow horizontal sway of the upper funnel via a
 *           single sinusoid, scaled by `(y / height)` so the base stays
 *           planted while the top swings.
 *
 * Study alongside: particle_systems/fire.c (heat-diffusion CA — different
 *                  approach; this file uses 3-D-ish particles instead),
 *                  particle_systems/smoke.c (similar pool + buoyancy
 *                  pattern, no rotation),
 *                  flocking/flocking.c (similar agent + force model).
 *
 * Section map:
 *   §1 config    — pool size, height, base radius, heat / spark / wind
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 5-stop heat ramp per theme + 4 themes
 *   §4 random    — frand
 *   §5 ember     — Ember struct + init / respawn / tick / draw (with wind)
 *   §6 tornado   — Spark + base_heat + Tornado pool/lifecycle
 *   §7 scene     — base_heat_draw + ember two-pass + sparks + HUD
 *   §8 screen    — ncurses init / cleanup
 *   §9 app       — signals, dt + world_time tracking, key handling, loop
 *
 * Keys:  [/]   ember count (50..600)
 *        -/+   spin speed (0.25× .. 4×)
 *        ,/.   funnel height (shorter / taller)
 *        w     toggle wind (off / default amplitude)
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/fire_tornado.c \
 *       -o fire_tornado -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three layered systems share the same heat ramp:
 *                 (1) ember pool — cylindrical particles around the axis
 *                     (Stage 1); each ember rises, rotates, drifts inward,
 *                     cools, and respawns at the base.
 *                 (2) base flame mat — 1-D heat array at the ground row.
 *                     Each frame: exponential decay, 3-tap sideways
 *                     diffusion, and 1–3 random heat injections per
 *                     frame biased toward the axis (triangular dist).
 *                     Renders as flickering glyphs along the bottom.
 *                 (3) spark pool — small particles spawned at random
 *                     ember positions with outward radial velocity;
 *                     gravity pulls them down, they cool fast, draw
 *                     bright then fade out.
 *                 Wind is a single horizontal sinusoid in time, scaled
 *                 by `y / height` so the base is fixed and the upper
 *                 funnel sways. Applied uniformly to every projection.
 *
 * Data-structure: Tornado holds Ember[N_EMBERS_MAX], Spark[N_SPARKS_MAX],
 *                 base_heat[BASE_HEAT_W] inline. No allocation after
 *                 init. Active counts and modes are config knobs.
 *
 * Rendering     : Per frame, in this draw order:
 *                   base_heat → embers (back) → embers (front) → sparks
 *                 The two-pass ember loop preserves depth ordering. The
 *                 base flame mat draws first so embers near the base
 *                 (y ≈ 0) overpaint it cleanly. Sparks draw last so
 *                 they appear in front of everything else.
 *
 * Performance   : O(N_EMBERS + N_SPARKS + BASE_HEAT_W) per frame. At
 *                 N_EMBERS=250, N_SPARKS=40, BASE_HEAT_W=60 it's a few
 *                 thousand mvaddch + a 1-D blur per frame — trivial.
 *
 * References    :
 *   Reeves, "Particle Systems — A Technique for Modelling a Class of
 *     Fuzzy Objects" SIGGRAPH (1983) — the foundational fire-particle
 *     paper; this file is a stylised cylindrical-pool variant.
 *   Bourke, "Character representation of greyscale images"
 *     (paulbourke.net/dataformats/asciiart/) — heat ramp inspiration.
 *   Sanglard, "Game Engine Black Book: Doom" (2018) ch. 11 — the
 *     1-D heat-diffusion fire algorithm that inspired §6 base flame.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The "tornado" is a 1-D phase variable per ember pretending to be 3-D
 * rotation. Each ember has `(y, phase, radius)` in cylindrical coords.
 * Side-view 2-D projection is `screen_col = axis + radius·cos(phase)`,
 * `screen_row = base − y`. The trick that sells the rotation: an ember
 * with `sin(phase) > 0` is on the camera-facing half (drawn bright);
 * `sin(phase) < 0` is the back half (drawn dim). As phase advances each
 * frame, a single ember alternates between bright and dim — that
 * alternation, across the whole pool, reads as a 3-D rotating column.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a glass of fizzy fire viewed from the side. Bubbles rise
 * along helical paths, faster near the centre, slower near the rim.
 * Bubbles on the camera side are bright; bubbles on the far side are
 * dimmer (they're behind the column, attenuated). The funnel narrows
 * with height because hot bubbles near the axis rise fastest and pull
 * the radius inward — the periphery cools and falls away. We don't
 * actually compute "behind" via 3-D geometry; we just dim the ember
 * when sin(phase) < 0, and the eye fills in the depth.
 *
 * DRAWING METHOD  (per frame, three layers)
 * ──────────────
 *  1. erase()
 *  2. base_heat — 1-D heat strip at the ground row:
 *       decay  : heat[i] *= (1 − BASE_DECAY · dt)
 *       blur   : heat[i] = 0.25·heat[i-1] + 0.5·heat[i] + 0.25·heat[i+1]
 *       inject : 1–3 random cells get heat ≈ 0.85, biased to centre
 *  3. embers — two passes for depth ordering:
 *       pass 1: every ember with sin(phase) < 0 (back half), A_DIM
 *       pass 2: every ember with sin(phase) ≥ 0 (front half), A_BOLD
 *  4. sparks — outward-thrown particles drawn last so they appear in
 *     front of everything else
 *
 * KEY FORMULAS
 * ────────────
 *  Cylindrical → screen (with wind tilt):
 *    sx = axis_x + wind_offset(y) + ASPECT_X · radius · cos(phase)
 *    sy = base_y − y
 *
 *  Per-ember dynamics (per tick):
 *    phase  += omega · omega_mult · dt          (omega ∝ 1/radius)
 *    y      += y_vel · dt                       (y_vel ∝ 1 − r/R_max)
 *    radius *= (1 − RADIUS_DECAY · dt)          (slow inward drift)
 *    temp   -= COOL_RATE · dt
 *    if y ≥ height || temp ≤ 0: respawn at base
 *
 *  Wind tilt (sway scaled by height):
 *    wind_offset(y) = wind_amp · sin(world_time · WIND_FREQ) · (y/height)
 *    base stays planted (y=0), top swings ±wind_amp
 *
 *  Heat ramp index:
 *    bucket = floor(temp · 4.99)        (clamped to 0..4)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • A_DIM on already-dark colours = invisible. The earliest version
 *    used dark cyan (24) for the cold end + A_DIM and the back-half
 *    embers vanished. Cold-end colours are now `45/34/178/134` —
 *    visible even with A_DIM applied.
 *
 *  • Radius near zero — `omega ∝ 1/radius` would blow up. Guarded with
 *    `fmaxf(e->radius * 0.4f, 0.6f)` in ember_respawn.
 *
 *  • Cell aspect — terminals are ~2× taller than wide. Without the
 *    `ASPECT_X = 2` stretch, the column would render as a vertical
 *    ellipse, not a round funnel.
 *
 *  • Two-pass draw avoids per-frame depth sorting. Sorting 250 embers
 *    by sin(phase) would work too but adds O(N log N) per frame for
 *    no visual gain.
 *
 *  • Resize keeps embers in flight (good — feels continuous), only
 *    repositions the axis. base_heat is cleared on resize implicitly
 *    via the next tick's decay; no special handling needed.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Watch a single ember through one revolution: it should fade
 *    (sin(phase) crosses 0 from + to −), reappear bright on the other
 *    side, and rise visibly during the cycle.
 *
 *  • Press `[` to drop ember count to 50. The column thins; you can
 *    track individual embers more clearly.
 *
 *  • Press `+` to crank spin: the centre spins faster than the edge
 *    (omega ∝ 1/r), which you can see as differential rotation —
 *    inner embers complete a revolution while outer ones barely move.
 *
 *  • Press `w` to toggle wind: the top of the funnel sways back and
 *    forth over ~14 seconds, base stays planted.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS          60

/* Pool sizing. Embers above N_EMBERS_DEFAULT are unused but the array
 * is sized for the max so '[' / ']' just changes the active count. */
#define N_EMBERS_DEFAULT    250
#define N_EMBERS_MIN         50
#define N_EMBERS_MAX        600
#define N_EMBERS_STEP        50

/* Geometry. Height is in screen cells; base radius is in cells too. */
#define HEIGHT_FRAC          0.85f
#define HEIGHT_FRAC_MIN      0.40f
#define HEIGHT_FRAC_MAX      0.95f
#define HEIGHT_FRAC_STEP     0.05f
#define BASE_RADIUS_CELLS    9.0f

/* Dynamics. y_vel and cool rate are tuned so the average ember spends
 * ~3 seconds rising from base to top, plenty of time to spin visibly. */
#define Y_VEL_BASE           6.0f
#define OMEGA_BASE           5.0f
#define OMEGA_MULT_DEFAULT   1.0f
#define OMEGA_MULT_MIN       0.25f
#define OMEGA_MULT_MAX       4.0f
#define OMEGA_MULT_STEP      0.25f
#define RADIUS_DECAY         0.6f
#define COOL_RATE            0.32f

/* Cell aspect — terminal cells are ~2× taller than wide. Stretch the
 * horizontal radius to compensate so the column reads as round. */
#define ASPECT_X             2.0f

/* Base flame mat — 1-D heat strip at the ground row. */
#define BASE_HEAT_W          60       /* cells, centred on the axis        */
#define BASE_HEAT_DECAY      2.0f     /* heat *= max(0, 1 − decay·dt)      */
#define BASE_INJECT_MIN      1        /* injections per frame, lower bound */
#define BASE_INJECT_MAX      3
#define BASE_INJECT_HEAT     0.85f    /* injected heat level (+ small jitter) */

/* Sparks — particles thrown outward from the column. */
#define N_SPARKS_MAX         40
#define SPARK_SPAWN_HZ      10.0f     /* spawns per second on average      */
#define SPARK_SPEED_MIN      8.0f     /* cells/sec at spawn                 */
#define SPARK_SPEED_MAX     20.0f
#define SPARK_GRAVITY       20.0f     /* cells/sec², pulls sparks down     */
#define SPARK_COOL           1.5f     /* temp/sec — sparks cool fast        */
#define SPARK_GLYPH          '*'

/* Wind — slow horizontal sway of the upper funnel. */
#define WIND_AMP_DEFAULT     3.0f     /* max cells of horizontal tilt at top */
#define WIND_FREQ            0.45f    /* rad/sec of axis sway                */

#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pair IDs */
#define PAIR_HEAT_0   1   /* coolest                                       */
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5   /* hottest — white                                */
#define PAIR_HUD      6
#define PAIR_HINT     7

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* 5-stop heat ramp per theme. Index 0 = coolest, 4 = hottest. */
static const short HEAT_256[N_THEMES][5] = {
    /* classic fire */ {  52, 196, 208, 226, 231 },
    /* blue fire    */ {  17,  21,  39,  51, 231 },
    /* toxic green  */ {  22,  28,  76, 154, 231 },
    /* hellfire     */ {  53, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_HUD,  x256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT, x256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  random                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* Map a temperature in [0, 1] to a heat-bucket index in [0, 4]. */
static int heat_bucket(float temp)
{
    int b = (int)(temp * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/* Glyphs by heat bucket (0 = coldest, 4 = hottest). */
static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };

/* Wind tilt at height `y` for the current world time.  Zero at the
 * base, sinusoidal at the top, scaled by wind amplitude. */
static float wind_offset(float y, float height, float world_time, float wind_amp)
{
    if (height < 1.0f) return 0.0f;
    float t_norm = y / height;
    return wind_amp * sinf(world_time * WIND_FREQ) * t_norm;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  ember                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    float y;          /* height above base (cells)                         */
    float phase;      /* radians around axis                                */
    float radius;     /* horizontal distance from axis (cells)              */
    float y_vel;      /* cells/sec, set at spawn from radius                */
    float omega;      /* rad/sec, set at spawn (smaller radius → faster)    */
    float temp;       /* 0..1, drives glyph + colour                        */
    int   alive;
} Ember;

/*
 * ember_respawn — drop the ember back at the base with a fresh random
 * phase and radius. Inner-radius embers get faster vertical velocity
 * and faster spin (omega ∝ 1/radius).
 *
 *   initial=1: spread the ember somewhere along the lifetime so the
 *              column is fully populated from frame 0 (used at startup
 *              and on 'r' / shape change).
 *   initial=0: classic respawn at the base.
 */
static void ember_respawn(Ember *e, float height, int initial)
{
    e->y      = initial ? frand() * height : 0.0f;
    e->phase  = frand() * 2.0f * (float)M_PI;
    float u   = frand();
    e->radius = BASE_RADIUS_CELLS * sqrtf(u);
    float r_norm = e->radius / BASE_RADIUS_CELLS;
    e->y_vel  = Y_VEL_BASE * (1.4f - 0.7f * r_norm);
    e->omega  = OMEGA_BASE / fmaxf(e->radius * 0.4f, 0.6f);
    e->temp   = 0.92f + 0.08f * frand();
    if (initial) e->temp *= (1.0f - 0.7f * (e->y / height));
    e->alive  = 1;
}

/*
 * ember_tick — one simulation step. Advance phase, height; cool; pull
 * radius inward. If expired (top reached or burned out), respawn.
 */
static void ember_tick(Ember *e, float height, float omega_mult, float dt)
{
    if (!e->alive) {
        ember_respawn(e, height, 0);
        return;
    }
    e->phase  += e->omega * omega_mult * dt;
    e->y      += e->y_vel * dt;
    e->radius *= (1.0f - RADIUS_DECAY * dt);
    e->temp   -= COOL_RATE * dt;
    if (e->y >= height || e->temp <= 0.0f) e->alive = 0;
}

/*
 * ember_draw — project (y, phase, radius) to screen and emit one cell.
 *
 *   screen_col = axis_x + wind_tilt(y) + ASPECT_X · radius · cos(phase)
 *   screen_row = base_y - y
 *   depth      = sin(phase)        +1 = front, −1 = back
 *
 * Front embers drawn with A_BOLD, back with A_DIM — the alternation is
 * what makes the rotation legible from a 2-D side view.
 */
static void ember_draw(const Ember *e,
                       int axis_x, int base_y, float height,
                       float world_time, float wind_amp,
                       int rows, int cols, int back_pass)
{
    if (!e->alive) return;
    float depth = sinf(e->phase);
    int   is_back = (depth < 0.0f);
    if ( back_pass &&  !is_back) return;
    if (!back_pass &&   is_back) return;

    float w_off = wind_offset(e->y, height, world_time, wind_amp);
    int sc = (int)((float)axis_x + w_off + ASPECT_X * e->radius * cosf(e->phase));
    int sr = (int)((float)base_y - e->y);
    if (sr < 0 || sr >= rows - 1) return;
    if (sc < 0 || sc >= cols)     return;

    int    bucket = heat_bucket(e->temp);
    chtype attr   = COLOR_PAIR(PAIR_HEAT_0 + bucket);
    attr |= is_back ? A_DIM : A_BOLD;
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  tornado (embers + base flame + sparks + lifecycle)                  */
/* ═══════════════════════════════════════════════════════════════════════ */

/* ───────── Spark — a single outward-thrown ember. ─────────────────────── */

typedef struct {
    float x, y;       /* screen-space position (cell units, float)         */
    float vx, vy;     /* velocity (cells/sec)                               */
    float temp;       /* 0..1                                                */
    int   alive;
} Spark;

/*
 * spark_tick — apply gravity, advance, cool. Die on cool-out or
 * leaving the screen.
 */
static void spark_tick(Spark *s, float dt, int rows, int cols)
{
    if (!s->alive) return;
    s->vy  += SPARK_GRAVITY * dt;
    s->x   += s->vx * dt;
    s->y   += s->vy * dt;
    s->temp -= SPARK_COOL * dt;
    if (s->temp <= 0.0f
        || s->x < 0 || s->x >= cols
        || s->y < 0 || s->y >= rows - 1)
        s->alive = 0;
}

static void spark_draw(const Spark *s, int rows, int cols)
{
    if (!s->alive) return;
    int sr = (int)s->y, sc = (int)s->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = heat_bucket(s->temp);
    attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    mvaddch(sr, sc, (chtype)(unsigned char)SPARK_GLYPH);
    attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
}

/* ───────── Base flame mat — 1-D heat strip with diffusion + injection. ─ */

/*
 * base_heat_tick — exponential decay → 3-tap sideways blur → random
 * heat injections at axis-biased indices. The blur is double-buffered
 * via a static scratch array; safe at fixed BASE_HEAT_W.
 */
static void base_heat_tick(float *heat, float dt)
{
    /* Decay everything. */
    float keep = 1.0f - BASE_HEAT_DECAY * dt;
    if (keep < 0.0f) keep = 0.0f;
    for (int i = 0; i < BASE_HEAT_W; i++)
        heat[i] *= keep;

    /* 3-tap blur (reflective edges). */
    static float tmp[BASE_HEAT_W];
    for (int i = 0; i < BASE_HEAT_W; i++) {
        float l = (i > 0)              ? heat[i - 1] : heat[i];
        float r = (i < BASE_HEAT_W - 1) ? heat[i + 1] : heat[i];
        tmp[i] = 0.25f * l + 0.50f * heat[i] + 0.25f * r;
    }
    memcpy(heat, tmp, sizeof tmp);

    /* Inject heat at random positions, biased toward the axis via a
     * triangular distribution (sum of two uniform samples). */
    int n_inject = BASE_INJECT_MIN + (rand() % (BASE_INJECT_MAX - BASE_INJECT_MIN + 1));
    float center = BASE_HEAT_W * 0.5f;
    float spread = BASE_HEAT_W * 0.35f;
    for (int k = 0; k < n_inject; k++) {
        float u   = frand() + frand() - 1.0f;          /* [-1, 1] triangular */
        int   idx = (int)(center + u * spread);
        if (idx < 0)             idx = 0;
        if (idx >= BASE_HEAT_W)  idx = BASE_HEAT_W - 1;
        float h = BASE_INJECT_HEAT + 0.15f * frand();
        if (heat[idx] < h) heat[idx] = h;
    }
}

static void base_heat_draw(const float *heat, int axis_x, int base_y,
                           int rows, int cols)
{
    if (base_y < 0 || base_y >= rows - 1) return;
    int half = BASE_HEAT_W / 2;
    for (int i = 0; i < BASE_HEAT_W; i++) {
        if (heat[i] < 0.05f) continue;
        int sc = axis_x - half + i;
        if (sc < 0 || sc >= cols) continue;
        int bucket = heat_bucket(heat[i]);
        attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
        mvaddch(base_y, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
        attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    }
}

/* ───────── Tornado — the umbrella state for all three sub-systems. ───── */

typedef struct {
    /* Sub-system pools. */
    Ember embers[N_EMBERS_MAX];
    Spark sparks[N_SPARKS_MAX];
    float base_heat[BASE_HEAT_W];

    /* Geometry. */
    int   axis_x;
    int   base_y;
    float height;

    /* Knobs. */
    int   n;                  /* active ember count                       */
    float height_frac;        /* fraction of (rows-1) used for height     */
    float omega_mult;         /* spin multiplier                           */
    float wind_amp;            /* 0 disables wind                           */

    /* Runtime. */
    float world_time;          /* seconds, only advances when not paused   */
    float spark_accum;         /* fractional spark spawns carried over     */

    /* UI. */
    int   theme;
    int   paused;
} Tornado;

static void tornado_position(Tornado *t, int rows, int cols)
{
    t->axis_x = cols / 2;
    t->base_y = rows - 2;
    t->height = (float)(rows - 1) * t->height_frac;
}

/*
 * tornado_reseed — re-randomise all embers along the lifetime, clear
 * the base heat strip, kill all sparks, reset the wind clock. Called
 * at startup and on 'r' or any shape change.
 */
static void tornado_reseed(Tornado *t)
{
    for (int i = 0; i < t->n; i++)
        ember_respawn(&t->embers[i], t->height, /*initial=*/1);
    for (int i = 0; i < BASE_HEAT_W; i++)
        t->base_heat[i] = 0.0f;
    for (int i = 0; i < N_SPARKS_MAX; i++)
        t->sparks[i].alive = 0;
    t->spark_accum = 0.0f;
    t->world_time  = 0.0f;
}

/*
 * tornado_spawn_spark — pick a random alive ember; project its current
 * position to screen; throw a spark in a random direction with a slight
 * upward bias.
 */
static void tornado_spawn_spark(Tornado *t)
{
    /* Find a dead spark slot. */
    int slot = -1;
    for (int i = 0; i < N_SPARKS_MAX; i++)
        if (!t->sparks[i].alive) { slot = i; break; }
    if (slot < 0) return;

    /* Pick a random alive ember as the source. */
    int src = -1;
    for (int tries = 0; tries < 8; tries++) {
        int k = rand() % t->n;
        if (t->embers[k].alive) { src = k; break; }
    }
    if (src < 0) return;
    const Ember *e = &t->embers[src];

    float w_off = wind_offset(e->y, t->height, t->world_time, t->wind_amp);
    float x = (float)t->axis_x + w_off + ASPECT_X * e->radius * cosf(e->phase);
    float y = (float)t->base_y - e->y;

    Spark *s = &t->sparks[slot];
    s->x = x;
    s->y = y;
    float ang   = frand() * 2.0f * (float)M_PI;
    float speed = SPARK_SPEED_MIN + frand() * (SPARK_SPEED_MAX - SPARK_SPEED_MIN);
    s->vx = cosf(ang) * speed * ASPECT_X;
    s->vy = sinf(ang) * speed * 0.5f - speed * 0.3f;   /* upward bias */
    s->temp  = 0.95f + 0.05f * frand();
    s->alive = 1;
}

/*
 * tornado_tick — advance every sub-system by dt. Sparks spawn at a
 * fractional rate handled via accumulator + while-loop so spawn rate
 * stays accurate at any frame rate.
 */
static void tornado_tick(Tornado *t, float dt, int rows, int cols)
{
    /* Embers. */
    for (int i = 0; i < t->n; i++)
        ember_tick(&t->embers[i], t->height, t->omega_mult, dt);

    /* Base flame mat. */
    base_heat_tick(t->base_heat, dt);

    /* Sparks. */
    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_tick(&t->sparks[i], dt, rows, cols);

    t->spark_accum += SPARK_SPAWN_HZ * dt;
    while (t->spark_accum >= 1.0f) {
        t->spark_accum -= 1.0f;
        tornado_spawn_spark(t);
    }

    t->world_time += dt;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols, const Tornado *t, double fps)
{
    erase();

    /* Bottom layer — base flame mat at the ground row. */
    base_heat_draw(t->base_heat, t->axis_x, t->base_y, rows, cols);

    /* Middle layers — back-side embers, then front-side embers. */
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/1);
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/0);

    /* Top layer — sparks (in front of everything). */
    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_draw(&t->sparks[i], rows, cols);

    /* HUD — top right */
    char buf[160];
    snprintf(buf, sizeof buf,
             " embers:%d  spin:%.2f×  height:%.0f%%  wind:%s  theme:%d  "
             "%5.1f fps  %s ",
             t->n, t->omega_mult, t->height_frac * 100.0f,
             (t->wind_amp > 0.001f) ? "on " : "off",
             t->theme, fps,
             t->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Key hints — bottom left */
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " [/]:embers  -/+:spin  ,/.:height  w:wind  t:theme  "
             "r:reset  p:pause  q:quit  [fire_tornado stage 2] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Tornado g_tornado;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_tornado.n           = N_EMBERS_DEFAULT;
    g_tornado.height_frac = HEIGHT_FRAC;
    g_tornado.omega_mult  = OMEGA_MULT_DEFAULT;
    g_tornado.wind_amp    = WIND_AMP_DEFAULT;
    g_tornado.theme       = 0;

    screen_init(g_tornado.theme);

    int rows = LINES, cols = COLS;
    tornado_position(&g_tornado, rows, cols);
    tornado_reseed(&g_tornado);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            tornado_position(&g_tornado, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_tornado.paused ^= 1; break;
                case 'r':          tornado_reseed(&g_tornado); break;
                case 't':          g_tornado.theme = (g_tornado.theme + 1) % N_THEMES;
                                   color_init(g_tornado.theme); break;
                case 'w':
                    g_tornado.wind_amp = (g_tornado.wind_amp > 0.001f)
                                       ? 0.0f : WIND_AMP_DEFAULT;
                    break;
                case '[':
                    if (g_tornado.n - N_EMBERS_STEP >= N_EMBERS_MIN)
                        g_tornado.n -= N_EMBERS_STEP;
                    break;
                case ']':
                    if (g_tornado.n + N_EMBERS_STEP <= N_EMBERS_MAX) {
                        int old_n = g_tornado.n;
                        g_tornado.n += N_EMBERS_STEP;
                        for (int i = old_n; i < g_tornado.n; i++)
                            ember_respawn(&g_tornado.embers[i],
                                          g_tornado.height, /*initial=*/1);
                    }
                    break;
                case '-':
                    if (g_tornado.omega_mult - OMEGA_MULT_STEP >= OMEGA_MULT_MIN)
                        g_tornado.omega_mult -= OMEGA_MULT_STEP;
                    break;
                case '+': case '=':
                    if (g_tornado.omega_mult + OMEGA_MULT_STEP <= OMEGA_MULT_MAX)
                        g_tornado.omega_mult += OMEGA_MULT_STEP;
                    break;
                case ',':
                    if (g_tornado.height_frac - HEIGHT_FRAC_STEP >= HEIGHT_FRAC_MIN) {
                        g_tornado.height_frac -= HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
                case '.':
                    if (g_tornado.height_frac + HEIGHT_FRAC_STEP <= HEIGHT_FRAC_MAX) {
                        g_tornado.height_frac += HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_tornado.paused) tornado_tick(&g_tornado, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_tornado, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
