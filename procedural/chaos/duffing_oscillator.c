/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * duffing_oscillator.c — a driven double-well oscillator that slides from
 * smooth motion into chaos as you turn up the drive. We solve its equation
 * of motion step by step and draw the path it traces in (position, speed)
 * space as a fading trail. The 30 presets walk you from a gentle wobble to
 * the famous chaotic "strange attractor".
 *
 * The equation and its chaos: Holmes (1979), "A nonlinear oscillator with a
 * strange attractor" — source of the default settings (gamma=0.25, omega=1,
 * A=0.50). Sister demos: ./double_pendulum.c (chaos with no outside push),
 * ./bifurcation.c (the same period-doubling in a simpler map),
 * ./poincare_section.c (the same attractor sampled as a dot cloud).
 */

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

/* §1  config */

enum {
    MAP_W_MAX        = 200, MAP_H_MAX = 56,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_TRAIL_BASE  =   3,    /* first of 4 color pairs, one per trail-age band */
    PAIR_AXIS        =   7, PAIR_LIVE = 8,
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

#define DUFFING_DT                0.01f
#define INT_STEPS_PER_TICK          8

#define TRAIL_MAX                3000
#define TRAIL_BAND_COUNT            4

/* The window onto (position, speed) space we map to the screen */
#define X_MIN                    -2.0f
#define X_MAX                     2.0f
#define V_MIN                    -2.0f
#define V_MAX                     2.0f

/*
 * Preset — names a row in the presets[] table below. They run simplest to
 * wildest, so pressing n/p tours the whole behaviour zoo of the oscillator
 * in order: gentle single-well wobbles, then a doubling chain that splits
 * each loop into two, four, eight (the classic route into chaos), then full
 * chaos in one well, then chaos that jumps between both wells, plus a few
 * oddities (periodic islands hidden inside chaos, light-friction runs,
 * off-beat drive frequencies). The trailing note on each line gives the
 * settings and what you'll see.
 */
typedef enum {
    /* Group A — small-amplitude single-well oscillation */
    PRESET_WHISPER = 0,    /* A=0.05, x₀=+1     tiny drive at right minimum */
    PRESET_HUSH,           /* A=0.05, x₀=-1     mirror in the left well     */
    PRESET_BREATH,         /* A=0.10            barely-perceptible osc      */
    PRESET_PURR,           /* A=0.15            small period-1              */
    PRESET_HUM,            /* A=0.20            steady period-1              */
    PRESET_CHIRP,          /* A=0.20, x₀=saddle small osc from unstable top */

    /* Group B — period-doubling cascade (Feigenbaum route to chaos) */
    PRESET_WALTZ,          /* A=0.28            period-2                   */
    PRESET_RONDO,          /* A=0.32            period-2 fuller            */
    PRESET_CADENCE,        /* A=0.36            period-4                   */
    PRESET_RIPPLE,         /* A=0.40            period-4 / period-8 onset  */
    PRESET_WEAVE,          /* A=0.45            band-merging               */
    PRESET_KNIT,           /* A=0.48            chaos onset                */

    /* Group C — single-well chaos (well-bounded strange attractor) */
    PRESET_SLEW,           /* A=0.50            Holmes' canonical chaos    */
    PRESET_JITTER,         /* A=0.55                                       */
    PRESET_STIR,           /* A=0.58                                       */
    PRESET_SHUFFLE,        /* A=0.60                                       */

    /* Group D — cross-well strange attractor (escape between both wells) */
    PRESET_CROSS,          /* A=0.70, x₀=0      from saddle                */
    PRESET_STRADDLE,       /* A=0.80            wide excursions            */
    PRESET_SWING,          /* A=0.85            both wells, full           */
    PRESET_BLOOM,          /* A=1.00            full Holmes attractor      */
    PRESET_RIOT,           /* A=1.10            high-energy chaos          */

    /* Group E — periodic windows hidden inside chaos */
    PRESET_RESET,          /* A=0.65            period-3 window            */
    PRESET_MIRROR,         /* A=0.72, x₀=-1     period-5 in left well      */
    PRESET_RING,           /* A=0.95            period-3 in cross-well     */

    /* Group F — low damping, high-energy (large attractor, slow decay) */
    PRESET_SLACK,          /* γ=0.10, A=0.40    light damping              */
    PRESET_LOOSE,          /* γ=0.10, A=0.50    extended attractor         */
    PRESET_GLIDE,          /* γ=0.08, A=0.45                               */
    PRESET_DRIFT,          /* γ=0.05, A=0.40    near-Hamiltonian           */

    /* Group G — off-resonance drive frequencies */
    PRESET_SLOW,           /* ω=0.7, A=0.40     sub-resonance              */
    PRESET_RAPID,          /* ω=1.4, A=0.40     super-resonance            */

    N_PRESETS,
} Preset;

/*
 * DuffingPreset — one ready-made experiment: five numbers that fully set up
 * a run. They split into two ideas. Three of them say what the oscillator IS
 * (how much friction, how the outside push beats, how hard it pushes); the
 * other two say where the motion STARTS. For the chaotic presets the start
 * barely matters in the long run — every start settles onto the same shape —
 * but for the calm ones it can decide which of two resting places you land
 * in, and it always shapes the first few seconds of trail.
 *
 *   name  : short label shown in the HUD; padded to 8 chars so the readout
 *           lines up. Chosen to hint at the look (WHISPER tiny, RIOT violent).
 *   gamma : friction (energy bleed). Never negative. 0.25 is the standard;
 *           the light-friction presets drop to 0.05-0.10.
 *   omega : how fast the outside push beats, in radians/sec. 1.0 is standard;
 *           a couple of presets detune it on purpose.
 *   A     : how hard the push shoves — the chaos dial. Tiny (0.05) gives a
 *           gentle wobble; around 0.5 and up you get chaos.
 *   x0    : starting position. +1 / -1 sit at the bottom of the right / left
 *           valley; 0 sits balanced on the hump between them.
 *   v0    : starting speed. Usually 0 (released from rest); a few give it a
 *           small initial kick.
 */
typedef struct {
    const char *name;
    float       gamma;
    float       omega;
    float       A;
    float       x0, v0;
} DuffingPreset;

/* The 30 presets, simplest first. Most just turn up the push strength A to
 * walk from a calm wobble into chaos; the last few instead change the
 * friction or the push frequency to reach shapes the A-sweep can't. */
static const DuffingPreset presets[N_PRESETS] = {
    /* Group A — small-amplitude single-well oscillation ─────────────── */
    { "WHISPER ", 0.25f, 1.0f, 0.05f,  1.0f,  0.0f },
    { "HUSH    ", 0.25f, 1.0f, 0.05f, -1.0f,  0.0f },
    { "BREATH  ", 0.25f, 1.0f, 0.10f,  1.0f,  0.0f },
    { "PURR    ", 0.25f, 1.0f, 0.15f,  1.0f,  0.0f },
    { "HUM     ", 0.25f, 1.0f, 0.20f,  1.0f,  0.0f },
    { "CHIRP   ", 0.25f, 1.0f, 0.20f,  0.0f,  0.20f },

    /* Group B — period-doubling cascade ─────────────────────────────── */
    { "WALTZ   ", 0.25f, 1.0f, 0.28f,  1.0f,  0.0f },
    { "RONDO   ", 0.25f, 1.0f, 0.32f,  1.0f,  0.0f },
    { "CADENCE ", 0.25f, 1.0f, 0.36f,  1.0f,  0.0f },
    { "RIPPLE  ", 0.25f, 1.0f, 0.40f,  1.0f,  0.0f },
    { "WEAVE   ", 0.25f, 1.0f, 0.45f,  1.0f,  0.0f },
    { "KNIT    ", 0.25f, 1.0f, 0.48f,  1.0f,  0.0f },

    /* Group C — single-well chaos ───────────────────────────────────── */
    { "SLEW    ", 0.25f, 1.0f, 0.50f,  1.0f,  0.0f },   /* Holmes canon */
    { "JITTER  ", 0.25f, 1.0f, 0.55f,  1.0f,  0.0f },
    { "STIR    ", 0.25f, 1.0f, 0.58f,  1.0f,  0.0f },
    { "SHUFFLE ", 0.25f, 1.0f, 0.60f,  1.0f,  0.0f },

    /* Group D — cross-well strange attractor ────────────────────────── */
    { "CROSS   ", 0.25f, 1.0f, 0.70f,  0.0f,  0.0f },
    { "STRADDLE", 0.25f, 1.0f, 0.80f,  0.0f,  0.0f },
    { "SWING   ", 0.25f, 1.0f, 0.85f,  0.0f,  0.0f },
    { "BLOOM   ", 0.25f, 1.0f, 1.00f,  0.0f,  0.0f },
    { "RIOT    ", 0.25f, 1.0f, 1.10f,  0.0f,  0.0f },

    /* Group E — periodic windows inside chaos ───────────────────────── */
    { "RESET   ", 0.25f, 1.0f, 0.65f,  1.0f,  0.0f },
    { "MIRROR  ", 0.25f, 1.0f, 0.72f, -1.0f,  0.0f },
    { "RING    ", 0.25f, 1.0f, 0.95f,  1.0f,  0.0f },

    /* Group F — low damping, high-energy ────────────────────────────── */
    { "SLACK   ", 0.10f, 1.0f, 0.40f,  1.0f,  0.0f },
    { "LOOSE   ", 0.10f, 1.0f, 0.50f,  0.0f,  0.0f },
    { "GLIDE   ", 0.08f, 1.0f, 0.45f,  0.0f,  0.0f },
    { "DRIFT   ", 0.05f, 1.0f, 0.40f,  0.0f,  0.0f },

    /* Group G — off-resonance frequencies ───────────────────────────── */
    { "SLOW    ", 0.25f, 0.7f, 0.40f,  1.0f,  0.0f },
    { "RAPID   ", 0.25f, 1.4f, 0.40f,  1.0f,  0.0f },
};

/*
 * Theme — one colour scheme: a colour for each thing we draw. Keeping these
 * in a table lets the t/T keys swap the whole look without touching any
 * drawing or physics code.
 *
 *   name    : short label shown in the HUD, padded to 8 chars.
 *   band[]  : the fade ramp for the trail, dimmest to brightest. band[0] is
 *             the oldest visible point, the last is the newest. These are
 *             xterm-256 colour numbers; keep them all >= 24 or the dim end
 *             vanishes on a black terminal (see CLAUDE.md "Theme Palette
 *             Brightness").
 *   axis    : colour of the crosshair reference lines. Kept muted so the
 *             lines orient you without fighting the trail.
 *   live    : colour of the moving '@' dot. The boldest colour, so your eye
 *             can follow it against the trail it leaves behind.
 */
typedef struct {
    const char *name;
    short       band[TRAIL_BAND_COUNT];
    short       axis;
    short       live;
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT",  {  75, 123, 220, 231 },  244, 196 },
    { "MATRIX",   {  77, 118, 156, 194 },  244, 226 },
    { "NOVA",     { 135, 171, 207, 219 },  244, 226 },
    { "MONO",     { 247, 250, 253, 255 },  240, 226 },
    { "OCEAN",    {  81, 117, 159, 195 },  244, 226 },
    { "FIRE",     { 208, 214, 220, 227 },  244, 231 },
    { "EARTH",    { 143, 179, 215, 222 },  244, 196 },
    { "FOREST",   { 114, 150, 157, 194 },  244, 226 },
    { "DESERT",   { 179, 215, 222, 229 },  244, 196 },
    { "ARCTIC",   { 117, 159, 195, 231 },  244, 196 },
};

/* §2  clock */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* §3  color */

/* bright yellow / bright cyan for the HUD, named so the numbers aren't
 * scattered around inline */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* Hand each drawing role its theme colour on a 256-colour terminal. The -1
 * background means "leave the terminal's own background", so it looks right
 * on dark or light. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_AXIS, t->axis, -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* Fallback for old 8-colour terminals: they can't show the fancy palettes,
 * so every theme looks the same here. We trade variety for staying readable
 * everywhere. */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_AXIS, COLOR_WHITE, -1);
    init_pair(PAIR_LIVE, COLOR_RED,   -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* The HUD colours stay fixed (bright yellow + cyan) no matter which theme is
 * picked, so the status text never loses contrast against the animation. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* §5  physics — the Duffing equation */

/*
 * DuffingSystem — what the oscillator IS: the three dials that don't change
 * while it runs. Kept apart from the moving state so the slope function can
 * take them as a read-only side input, and so changing the experiment only
 * touches this struct.
 *
 *   gamma : friction. Bleeds energy out at a rate set by the current speed.
 *           More friction means the motion settles down sooner; near zero it
 *           barely settles at all.
 *   omega : how fast the outside push beats, in radians/sec. One push cycle
 *           takes 2*pi/omega seconds.
 *   A     : how hard the push shoves. This is the chaos dial: turn it up and
 *           the motion goes from a single loop, to doubling loops, to chaos.
 */
typedef struct {
    float gamma;
    float omega;
    float A;
} DuffingSystem;

/*
 * DuffingState — where the oscillator IS right now: the moving numbers that
 * the solver advances each step. It's a small by-value bundle so the solver
 * can make cheap throwaway copies of it (its halfway guesses are just more
 * DuffingStates).
 *
 *   x : position. The landscape has two valleys (at -1 and +1) and a hump
 *       between them (at 0). Usually the motion rocks inside one valley; with
 *       a strong enough push it jumps over the hump between both. Stored
 *       without limits; we only clip it when drawing.
 *   v : speed (how fast x is changing).
 *   t : seconds elapsed since the last reset. We carry time as part of the
 *       state on purpose: the push A*cos(omega*t) depends on the clock, so
 *       the solver must roll time forward in lockstep with x and v — the
 *       halfway guesses need the push value at the halfway time, not the
 *       start time. A float is plenty for runs of a few minutes.
 */
typedef struct {
    float x;
    float v;
    float t;
} DuffingState;

/*
 * Duffing — the whole oscillator in one handle: the fixed dials plus the
 * moving state. Bundled so callers pass one pointer. Changing the experiment
 * touches only .system; advancing the motion touches only .state.
 */
typedef struct {
    DuffingSystem system;
    DuffingState  state;
} Duffing;

/* The heart of the physics: given the current state, work out how fast each
 * part is changing right now. Position changes at the current speed; speed
 * changes from three pulls added together — friction dragging it down, the
 * landscape pushing it toward a valley, and the outside push. We also report
 * that time advances at rate 1, so the solver can roll the clock forward in
 * step with everything else. */
static inline DuffingState duffing_deriv(const DuffingState *s,
                                         const DuffingSystem *sys)
{
    float damping_term        = -sys->gamma * s->v;
    float linear_restoring    =  s->x;
    float cubic_stiffening    = -s->x * s->x * s->x;
    float periodic_drive_term =  sys->A * cosf(sys->omega * s->t);

    DuffingState out;
    out.x = s->v;
    out.v = damping_term + linear_restoring + cubic_stiffening
          + periodic_drive_term;
    out.t = 1.0f;
    return out;
}

/* Step from one state along a slope by amount h, as a fresh state. The
 * solver uses this to build its halfway guesses and its final update. */
static inline DuffingState state_add(const DuffingState *a,
                                     float h, const DuffingState *k)
{
    DuffingState r;
    r.x = a->x + h * k->x;
    r.v = a->v + h * k->v;
    r.t = a->t + h * k->t;
    return r;
}

/* Blend the four slope guesses into one best estimate, weighting the two
 * middle ones double. This is the standard recipe for the RK4 method. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f
static inline DuffingState rk4_butcher_weighted_average(
    const DuffingState *k1, const DuffingState *k2,
    const DuffingState *k3, const DuffingState *k4)
{
    DuffingState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.v = (k1->v + 2.0f*k2->v + 2.0f*k3->v + k4->v) / RK4_BUTCHER_WEIGHT_SUM;
    avg.t = (k1->t + 2.0f*k2->t + 2.0f*k3->t + k4->t) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Advance the oscillator by one small time step (RK4). Rather than trust a
 * single slope across the whole step, it samples the slope four times — at
 * the start, twice in the middle, once at the end — and blends them. That
 * blend tracks the curved true motion far better than one straight guess, so
 * we can take bigger steps without the chaos drifting off course. Time has to
 * roll forward inside those samples too, because the outside push is a
 * function of the clock. */
static void duffing_rk4(Duffing *d, float dt)
{
    const DuffingSystem *sys = &d->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    DuffingState slope_start    = duffing_deriv(&d->state, sys);

    DuffingState midpoint_1     = state_add(&d->state, half_dt, &slope_start);
    DuffingState slope_mid_1    = duffing_deriv(&midpoint_1, sys);

    DuffingState midpoint_2     = state_add(&d->state, half_dt, &slope_mid_1);
    DuffingState slope_mid_2    = duffing_deriv(&midpoint_2, sys);

    DuffingState endpoint       = state_add(&d->state, dt, &slope_mid_2);
    DuffingState slope_end      = duffing_deriv(&endpoint, sys);

    DuffingState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    d->state = state_add(&d->state, dt, &effective_slope);
}

/* Put the oscillator at the preset's starting position and speed, clock at
 * zero. The fixed dials are loaded separately, so a future "nudge the start
 * but keep the same system" tweak touches only this one. */
static inline void duffing_state_init_from_preset(DuffingState *s,
                                                  const DuffingPreset *preset)
{
    s->x = preset->x0;
    s->v = preset->v0;
    s->t = 0.0f;
}

static inline void duffing_system_init_from_preset(DuffingSystem *sys,
                                                   const DuffingPreset *preset)
{
    sys->gamma = preset->gamma;
    sys->omega = preset->omega;
    sys->A     = preset->A;
}

/* §6  trail */

/*
 * Trail — the breadcrumb history of where the oscillator has been. A single
 * dot tells you nothing; it's the SHAPE the dots trace out over time that
 * shows whether the motion is a simple loop or chaos. So we remember the last
 * few thousand (position, speed) points and redraw the whole trace each
 * frame, fading the old end so you can tell which way time runs.
 *
 * It's a ring buffer — a fixed array reused round and round. We pick that
 * (over a growing list) because the per-frame loop must never stop to
 * allocate memory, and the cost stays the same no matter how long the demo
 * runs: once full, each new point just overwrites the oldest.
 *
 *   x[], v[] : the stored points, in real physics units (not screen cells);
 *              we convert to cells only when drawing, to keep full accuracy.
 *   head     : array slot of the newest point. Each push moves head forward
 *              first, then writes, so head always points at the latest entry.
 *   count    : how many points are currently stored, capped at the array size.
 */
typedef struct {
    float x[TRAIL_MAX];
    float v[TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *tr) { tr->head = 0; tr->count = 0; }

static void trail_push(Trail *tr, float x, float v)
{
    tr->head = (tr->head + 1) % TRAIL_MAX;
    tr->x[tr->head] = x;
    tr->v[tr->head] = v;
    if (tr->count < TRAIL_MAX) tr->count++;
}

/* Where the oldest still-kept point sits in the ring. Works whether the
 * buffer is still filling or already wrapped around. */
static inline int trail_oldest_index(const Trail *tr)
{
    return (tr->head - tr->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* Decide which fade band a point belongs to from how old it is. Splits the
 * trail evenly: the newest points get the brightest band, the oldest the
 * dimmest. */
static inline int trail_band_for_age(int age, int count)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age * TRAIL_BAND_COUNT) / count;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* §7  state — which preset and which theme are selected */

/*
 * PresetState — just "which preset is loaded", wrapped in its own little type
 * so the key handler can say preset_state_cycle_next() instead of doing the
 * wrap-around arithmetic by hand at every call site.
 *
 *   current : row number in the presets[] table. The cycle helpers keep it in
 *             range. Starts on SLEW, the headline chaotic case, so the demo
 *             opens on something interesting.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const DuffingPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — "which theme is active". Same one-number shape as
 * PresetState, but kept a separate type on purpose: the two index different
 * tables, so giving them distinct types stops a slip where a "next theme"
 * accidentally walks off the end of the preset list, or vice versa.
 *
 *   current : row number in the themes[] table. Starts at 0, the default look.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)       { theme_apply(p->current); }

/* §8  scene */

/*
 * Scene — everything that changes while the demo runs, gathered in one place
 * so there are no loose globals and the main loop drives it through a handful
 * of named calls.
 *
 *   duffing : the oscillator itself (its dials and its current motion).
 *   trail   : the breadcrumb history of where it has been.
 *   preset  : which experiment is selected (n/p cycle it).
 *   palette : which colour theme is selected (t/T cycle it).
 *   paused  : when set, the physics freezes but drawing keeps going, so you
 *             can study a frozen pose.
 *   map_w/h : how many character cells of the terminal we have to draw into,
 *             after reserving the HUD rows. Used to centre the picture.
 */
typedef struct {
    Duffing      duffing;
    Trail        trail;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
    int          map_w, map_h;
} Scene;

/* Load the selected preset: both the fixed dials and the starting motion.
 * Clearing the trail is left to scene_reset(). */
static void scene_load_active_preset(Scene *s)
{
    const DuffingPreset *preset = preset_state_active(&s->preset);
    duffing_system_init_from_preset(&s->duffing.system, preset);
    duffing_state_init_from_preset (&s->duffing.state,  preset);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s)
{
    trail_reset(&s->trail);
    scene_load_active_preset(s);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->map_w  = mw;
    s->map_h  = mh;
    s->paused = false;

    preset_state_init (&s->preset,  PRESET_SLEW);  /* open on the headline chaos */
    palette_state_init(&s->palette, 0);
    scene_reset(s);
}

static void scene_resize(Scene *s, int mw, int mh)
{
    s->map_w = mw;
    s->map_h = mh;
}

/* Move the simulation forward one frame. We take several small solver steps
 * per frame rather than one big one, so the math stays accurate even when the
 * motion swings fast, and drop a breadcrumb after each. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        duffing_rk4(&s->duffing, DUFFING_DT);
        trail_push(&s->trail, s->duffing.state.x, s->duffing.state.v);
    }
}

/* §9  screen */

/*
 * Screen — our small handle on the terminal: how big it is right now, plus
 * the one spot that talks to ncurses for setup and teardown. Carrying the
 * width and height here saves every drawing routine from asking the terminal
 * its size over and over.
 *
 *   cols : terminal width in character cells.
 *   rows : terminal height in character cells. Row 0 is the top; the HUD
 *          claims a couple of rows top and bottom (see §1).
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1); color_init();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * PhaseSpaceViewport — the on-screen rectangle the picture is drawn into, and
 * the recipe for turning a (position, speed) pair into a cell inside it. The
 * whole point of the demo is this picture: plot speed against position and the
 * shape that emerges tells you whether the motion is a tidy loop or chaos.
 * Bundling the rectangle's corner and size here means we work it out once a
 * frame and every drawing routine just borrows it.
 *
 *   gx0, gy0 : cell position of the rectangle's top-left corner.
 *   w, h     : rectangle size in cells.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} PhaseSpaceViewport;

/* Left edge that centres a width-w box in a width-`outer_cols` space, never
 * letting it start off the left edge. */
static inline int viewport_centered_origin_x(int outer_cols, int w)
{
    int gx0 = (outer_cols - w) / 2;
    return (gx0 < 0) ? 0 : gx0;
}

/* Top edge that centres a height-h box in the space left between the top and
 * bottom HUD rows, so the picture never overlaps the status text. */
static inline int viewport_centered_origin_y_in_drawable(int outer_rows, int h)
{
    int gy0 = ((outer_rows - HUD_BAND_RESERVED_ROWS) - h) / 2 + HUD_TOP_ROWS;
    return (gy0 < HUD_TOP_ROWS) ? HUD_TOP_ROWS : gy0;
}

/* Work out this frame's drawing rectangle: take the available size from the
 * scene and centre it between the HUD rows. */
static PhaseSpaceViewport phase_space_viewport_build(const Scene *s,
                                                     int cols, int rows)
{
    PhaseSpaceViewport vp;
    vp.w   = s->map_w;
    vp.h   = s->map_h;
    vp.gx0 = viewport_centered_origin_x(cols, vp.w);
    vp.gy0 = viewport_centered_origin_y_in_drawable(rows, vp.h);
    return vp;
}

/* Turn a real position or speed into a column or row inside the rectangle,
 * clamped to its edges. (The actual top-to-bottom flip, so faster reads as
 * higher up, happens where we plot.) */
static inline int phase_to_cell_x(const PhaseSpaceViewport *vp, float x)
{
    int cx = (int)((x - X_MIN) / (X_MAX - X_MIN) * (float)vp->w);
    if (cx < 0)         cx = 0;
    if (cx >= vp->w)    cx = vp->w - 1;
    return cx;
}
static inline int phase_to_cell_y(const PhaseSpaceViewport *vp, float v)
{
    int cy = (int)((v - V_MIN) / (V_MAX - V_MIN) * (float)vp->h);
    if (cy < 0)         cy = 0;
    if (cy >= vp->h)    cy = vp->h - 1;
    return cy;
}

/* Put one character at the screen cell for a real (position, speed). This is
 * the one place that flips top-for-bottom, so higher speed shows higher up
 * even though screen rows count downward. */
static inline void phase_space_plot(const PhaseSpaceViewport *vp,
                                    float x, float v, chtype glyph)
{
    int cx = phase_to_cell_x(vp, x);
    int cy = phase_to_cell_y(vp, v);
    mvaddch(vp->gy0 + (vp->h - 1 - cy), vp->gx0 + cx, glyph);
}

/* Caller picks the colour first; these just fill a column or a row. */
static void viewport_paint_vertical_line(const PhaseSpaceViewport *vp,
                                         int cx_local, chtype glyph)
{
    for (int y = 0; y < vp->h; y++)
        mvaddch(vp->gy0 + (vp->h - 1 - y), vp->gx0 + cx_local, glyph);
}

static void viewport_paint_horizontal_line(const PhaseSpaceViewport *vp,
                                           int cy_local, chtype glyph)
{
    for (int x = 0; x < vp->w; x++)
        mvaddch(vp->gy0 + (vp->h - 1 - cy_local), vp->gx0 + x, glyph);
}

/* Draw the crosshair through the centre (position 0, speed 0). Without it the
 * trail just floats; with it you can see which valley the motion is in and how
 * often it passes through the middle. */
static void phase_space_paint_axes(const PhaseSpaceViewport *vp)
{
    int x_zero_col = phase_to_cell_x(vp, 0.0f);
    int v_zero_row = phase_to_cell_y(vp, 0.0f);

    attron(COLOR_PAIR(PAIR_AXIS));
    viewport_paint_vertical_line  (vp, x_zero_col, '|');
    viewport_paint_horizontal_line(vp, v_zero_row, '-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* the character used for every trail point; named so a future per-band glyph
 * has one place to change */
#define TRAIL_GLYPH  '.'

/* Draw the whole trail, oldest point first, colouring each by age so the tail
 * fades out behind the moving dot. */
static void phase_space_paint_trail(const PhaseSpaceViewport *vp,
                                    const Trail *tr)
{
    if (tr->count == 0) return;

    int oldest = trail_oldest_index(tr);

    for (int i = 0; i < tr->count; i++) {
        int sample_index = (oldest + i) % TRAIL_MAX;
        int age          = tr->count - 1 - i;          /* 0 = newest */
        int band         = trail_band_for_age(age, tr->count);

        int pair = PAIR_TRAIL_BASE + band;
        attron(COLOR_PAIR(pair) | A_BOLD);
        phase_space_plot(vp, tr->x[sample_index], tr->v[sample_index], TRAIL_GLYPH);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* The '@' at the current position. Drawn last so it sits on top of the trail,
 * giving the eye a moving dot to follow. */
static void phase_space_paint_live_marker(const PhaseSpaceViewport *vp,
                                          const DuffingState *state)
{
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    phase_space_plot(vp, state->x, state->v, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* Draw one frame back to front: crosshair, then trail, then the live dot. */
static void scene_paint(const Scene *s, int cols, int rows)
{
    PhaseSpaceViewport vp = phase_space_viewport_build(s, cols, rows);
    phase_space_paint_axes(&vp);
    phase_space_paint_trail(&vp, &s->trail);
    phase_space_paint_live_marker(&vp, &s->duffing.state);
}

static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DUFFING OSCILLATOR ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Build the right-hand status text. Kept separate from drawing so the build
 * and the placement read as two clear steps. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const DuffingPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  A:%.2f  t:%.1fs ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)s->duffing.system.A,
             (double)s->duffing.state.t);
}

/* Print text flush against the right edge of row 0, clamped so a narrow
 * terminal doesn't push the start off-screen. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* How wide each row-1 cell prints, spaces included. Fixed widths so the
 * readout doesn't jump around as preset and theme names change length. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

static void hud_paint_param_cell_bold(int cursor_x, const char *fmt, const char *value)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, fmt, value);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1: the current preset name, theme name, then the friction / push-speed
 * / trail-fill readouts, laid out left to right. */
static void hud_param(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    hud_paint_param_cell_bold(cursor_x, " preset:%-8s ",
                              preset_state_active(&s->preset)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    hud_paint_param_cell_bold(cursor_x, " theme:%-8s ",
                              palette_state_active(&s->palette)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " γ:%.2f ω:%.2f  trail:%d/%d ",
             (double)s->duffing.system.gamma,
             (double)s->duffing.system.omega,
             s->trail.count, TRAIL_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top(sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint(sc->rows);
}

/* §10 app */

/*
 * App — the one box holding everything that changes: the simulation, the
 * terminal handle, the physics speed, the drawing size, and two flags the
 * signal handlers set. There's no other global mutable state, so the program
 * reads top-down: main() drives the App, each helper does one thing to it.
 *
 *   scene       : the simulation — what the user sees.
 *   screen      : the terminal (its size and our setup/teardown of ncurses).
 *   sim_fps     : how many physics steps per second. Separate from the fixed
 *                 60-fps drawing rate, so you can speed the physics up or down
 *                 (the ]/[ keys) without changing how smoothly it draws.
 *   map_w/h     : the drawing area size in cells, cached so a frame sees one
 *                 steady value; recomputed on resize.
 *   running     : goes to 0 to quit. A signal handler or the q key sets it.
 *   need_resize : set by the window-resize signal; the next frame rebuilds.
 *
 * running and need_resize are volatile sig_atomic_t because signal handlers
 * write them — the only kind of variable a handler is allowed to touch safely.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

/* One global instance, because signal handlers can't be handed a pointer —
 * they have to reach a fixed address. Everything else passes App* around. */
static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Ctrl-C / kill ask us to quit; a window resize asks us to rebuild next frame. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Pick the drawing size from the terminal minus the HUD rows, kept within
 * sensible minimum and maximum bounds. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* One-time startup: random seed, default speed, terminal, size, scene. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_resize(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* How long since the last frame. Capped, so if the program was stalled (a
 * suspended terminal, say) we don't suddenly owe the physics a huge backlog. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Run the physics in fixed-size steps, taking as many as the elapsed time has
 * earned. We insist on a fixed step on purpose: chaos is wildly sensitive, so
 * a step size that drifted with the frame rate would give a different motion
 * on every machine. The fixed step keeps a reset reproducible. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the shown fps only every so often, so the number doesn't flicker. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Wait just long enough to hold a steady frame rate. We sleep before drawing,
 * not after, so the time the drawing itself takes doesn't add jitter. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and flip it to the screen. The only place that shows output. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Nudge the physics speed up or down, staying within bounds. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_oscillator  (App *app) { scene_reset(&app->scene); }

static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene);
}

static void app_cycle_theme_next(App *app)
{
    palette_state_cycle_next(&app->scene.palette);
    scene_apply_theme(&app->scene);
}
static void app_cycle_theme_prev(App *app)
{
    palette_state_cycle_prev(&app->scene.palette);
    scene_apply_theme(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* Check for a keypress without blocking and act on it. Returns false to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The key map: each key calls one of the actions above. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_oscillator (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the timekeeping the main loop carries from one frame to the
 * next, gathered so the loop body stays a short, readable list of steps.
 *
 *   frame_time  : when the last frame happened, so we can measure the gap.
 *   sim_accum   : time that has piled up but not yet been spent on physics
 *                 steps; the physics drains it in fixed chunks.
 *   fps_accum   : time piled up since we last refreshed the fps number.
 *   frame_count : frames counted in the current fps window.
 *   fps_display : the fps figure currently shown in the HUD.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* Set up, then loop every frame: handle any resize, measure the time gap, run
 * that much physics, draw, and read the keyboard — until asked to quit. */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
