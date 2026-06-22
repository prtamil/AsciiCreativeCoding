/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * recurrence_plot.c — paint a dot at (i, j) whenever a signal looked
 * about the same at moment i as it did at moment j.  The pattern of
 * dots is a fingerprint of the signal: repeating signals make clean
 * diagonal stripes, chaos makes broken speckle, noise makes an even
 * fuzz.  Cycle 30 different signals to compare the fingerprints.
 *
 * Original idea: Eckmann, Kamphorst & Ruelle 1987; texture taxonomy:
 * Marwan et al. 2007.  Sister diagnostics in this folder:
 * bifurcation.c (chaos along a parameter) and sensitive_dependence.c
 * (chaos via two trajectories).
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* §1 config */

enum {
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_RECUR       =   3,
    PAIR_DIAG        =   4,
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

#define N_MAX                    140    /* how many samples we keep = the matrix side */
#define LORENZ_DT                 0.02f
#define HENON_A                   1.4f
#define HENON_B                   0.3f

/*
 * SignalKind — names the family of signal generator to run.  signal_step
 * looks at this tag to decide which formula to use, and each family draws
 * a recognisably different fingerprint, so flipping through them is a tour
 * of what recurrence plots can show: plain repeating waves, sums of waves,
 * fading/gated waves, random processes, the chaotic maps (logistic, tent,
 * Hénon, Lorenz), and a couple of oddballs.  Adding a generator means
 * adding an entry here, a row in presets[], and a case in signal_step.
 * The per-entry notes give the formula each one uses.
 */
typedef enum {
    SIG_CONSTANT = 0,    /* stays flat            -> every dot lit, full grid */
    SIG_LINEAR,          /* steady climb          -> single diagonal stripe   */
    SIG_SINE,            /* plain sine wave                                   */
    SIG_SQUARE,          /* sine, but snapped to +1 / -1                      */
    SIG_TRIANGLE,        /* triangle wave                                     */
    SIG_SAWTOOTH,        /* sawtooth                                          */
    SIG_TWO_SINES,       /* one sine plus a quieter second sine               */
    SIG_QUASI_GOLDEN,    /* two sines whose periods never line up (golden ratio) */
    SIG_BEAT,            /* two nearly-equal sines -> slow beating             */
    SIG_DAMPED_SINE,     /* sine fading toward zero                           */
    SIG_BURST,           /* sine switched on and off in blocks                */
    SIG_WANE,            /* sine whose volume slowly drops                    */
    SIG_NOISE,           /* fresh random value each step, in [-1, 1]          */
    SIG_RANDOM_WALK,     /* keep adding small random steps                    */
    SIG_AR1,             /* this value = mostly the last value + a little noise */
    SIG_BROWNIAN,        /* random walk with bigger steps                     */
    SIG_LOGISTIC,        /* the classic chaos map x -> r*x*(1-x)              */
    SIG_TENT,            /* tent map: fold the line in half each step         */
    SIG_LORENZ_X,        /* x-coordinate of the Lorenz attractor (smooth chaos) */
    SIG_HENON_X,         /* x-coordinate of the Hénon map (jumpy chaos)       */
    SIG_RAMP_SINE,       /* a steady climb with a sine riding on top          */
    SIG_STEP_JUMPS,      /* holds a level, then randomly hops to a new one    */
} SignalKind;

/*
 * Preset — friendly names for the rows of the presets[] table below.
 * They are ordered simplest-first and grouped (A trivial through H
 * exotic) so pressing 'n' walks you from a plain repeating wave up to
 * full chaos and the fingerprints get steadily busier.  N_PRESETS sits
 * at the end as the count; the cycling helpers wrap around using it.
 * The group comments here line up with the same dividers in presets[].
 */
typedef enum {
    /* Group A — trivial / linear (2) */
    PRESET_CONSTANT = 0,
    PRESET_LINEAR,
    /* Group B — simple periodic (5) */
    PRESET_SINE_SLOW,
    PRESET_SINE_MED,
    PRESET_SINE_FAST,
    PRESET_SQUARE,
    PRESET_TRIANGLE,
    /* Group C — compound periodic (4) */
    PRESET_SAW,
    PRESET_DUET,
    PRESET_QUASI,
    PRESET_BEAT,
    /* Group D — damped / gated (3) */
    PRESET_DAMPED,
    PRESET_BURST,
    PRESET_WANE,
    /* Group E — stochastic (4) */
    PRESET_NOISE,
    PRESET_WALK,
    PRESET_AR_STRONG,
    PRESET_AR_WEAK,
    PRESET_BROWN,
    /* Group F — logistic-map cascade (5) */
    PRESET_LOGI_P2,
    PRESET_LOGI_P4,
    PRESET_LOGI_ONSET,
    PRESET_LOGI_CHAOS,
    PRESET_LOGI_P3,
    /* Group G — other chaos (3) */
    PRESET_TENT,
    PRESET_LORENZ,
    PRESET_HENON,
    /* Group H — mixed / exotic (2) */
    PRESET_RAMP_SINE,
    PRESET_STEPS,
    N_PRESETS,
} Preset;

/*
 * RPPreset — one row of the picker: a generator plus its settings.
 * Each row is everything needed to draw one fingerprint, and cycling
 * rows is the only way the picture changes.  signal_step reads kind +
 * p1..p3 to make samples; epsilon decides how close two samples must
 * be to count as a match.
 *
 *   name    : the short label shown in the HUD (padded to line up).
 *   kind    : which generator to run (see SignalKind above).
 *   epsilon : "how close is close enough" for two samples to light a
 *             dot.  Hand-tuned per signal so each fingerprint has a
 *             similar amount of ink -- a sine living in [-1, 1] wants
 *             a small value while a wandering random walk wants a
 *             bigger one.
 *   p1,p2,p3: knobs the generator reads (speed, fade rate, the
 *             logistic r, and so on).  Unused ones are left at 0;
 *             signal_step says what each means for each kind.
 */
typedef struct {
    const char *name;
    SignalKind  kind;
    float       epsilon;
    float       p1, p2, p3;
} RPPreset;

/* The 30 signals, simplest first.  Groups match the Preset enum above. */
static const RPPreset presets[N_PRESETS] = {
    /* A: trivial / linear */
    { "CONSTANT", SIG_CONSTANT,     0.10f,  0.50f,  0.0f,  0.0f },
    { "LINEAR  ", SIG_LINEAR,       0.10f,  0.015f, 0.0f,  0.0f },
    /* B: simple periodic */
    { "SINE_SLO", SIG_SINE,         0.10f,  0.15f,  0.0f,  0.0f },
    { "SINE_MED", SIG_SINE,         0.10f,  0.45f,  0.0f,  0.0f },
    { "SINE_FAS", SIG_SINE,         0.10f,  0.95f,  0.0f,  0.0f },
    { "SQUARE  ", SIG_SQUARE,       0.10f,  0.40f,  0.0f,  0.0f },
    { "TRIANGLE", SIG_TRIANGLE,     0.10f,  0.40f,  0.0f,  0.0f },
    /* C: compound periodic */
    { "SAW     ", SIG_SAWTOOTH,     0.10f,  0.40f,  0.0f,  0.0f },
    { "DUET    ", SIG_TWO_SINES,    0.15f,  0.30f,  0.71f, 0.0f },
    { "QUASI   ", SIG_QUASI_GOLDEN, 0.15f,  0.35f,  0.0f,  0.0f },
    { "BEAT    ", SIG_BEAT,         0.15f,  0.50f,  0.55f, 0.0f },
    /* D: damped / gated */
    { "DAMPED  ", SIG_DAMPED_SINE,  0.10f,  0.50f,  0.015f,0.0f },
    { "BURST   ", SIG_BURST,        0.10f,  0.50f, 20.0f,  0.0f },
    { "WANE    ", SIG_WANE,         0.10f,  0.45f,  0.006f,0.0f },
    /* E: random */
    { "NOISE   ", SIG_NOISE,        0.30f,  0.0f,   0.0f,  0.0f },
    { "WALK    ", SIG_RANDOM_WALK,  0.80f,  0.0f,   0.0f,  0.0f },
    { "AR_STRO ", SIG_AR1,          0.30f,  0.95f,  0.0f,  0.0f },
    { "AR_WEAK ", SIG_AR1,          0.30f,  0.50f,  0.0f,  0.0f },
    { "BROWN   ", SIG_BROWNIAN,     2.00f,  0.0f,   0.0f,  0.0f },
    /* F: logistic map, walked from calm to chaotic */
    { "LOGI_P2 ", SIG_LOGISTIC,     0.04f,  3.20f,  0.0f,  0.0f },
    { "LOGI_P4 ", SIG_LOGISTIC,     0.04f,  3.50f,  0.0f,  0.0f },
    { "LOGI_ON ", SIG_LOGISTIC,     0.06f,  3.57f,  0.0f,  0.0f },
    { "LOGI_CH ", SIG_LOGISTIC,     0.06f,  3.70f,  0.0f,  0.0f },
    { "LOGI_P3 ", SIG_LOGISTIC,     0.06f,  3.835f, 0.0f,  0.0f },
    /* G: other chaos */
    { "TENT    ", SIG_TENT,         0.04f,  2.00f,  0.0f,  0.0f },
    { "LORENZ  ", SIG_LORENZ_X,     1.50f,  0.0f,   0.0f,  0.0f },
    { "HENON   ", SIG_HENON_X,      0.20f,  0.0f,   0.0f,  0.0f },
    /* H: mixed / exotic */
    { "RAMP_SIN", SIG_RAMP_SINE,    0.15f,  0.40f,  0.003f,0.0f },
    { "STEPS   ", SIG_STEP_JUMPS,   0.10f,  0.05f,  0.0f,  0.0f },
};

/*
 * Theme — a named colour scheme.  A recurrence plot is basically two
 * colours: the dots and the diagonal line down the middle, so a theme
 * is just those two colour codes plus a label.  The t/T keys cycle
 * them.  All codes are deliberately from the bright half of the
 * 256-colour set so dots stay visible on a black terminal.
 *
 *   name  : the label shown in the HUD.
 *   recur : colour of the match dots.
 *   diag  : colour of the always-lit centre diagonal.
 */
typedef struct { const char *name; short recur, diag; } Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", 117, 220 },
    { "MATRIX",  118, 226 },
    { "NOVA",    207, 226 },
    { "MONO",    253, 226 },
    { "OCEAN",   159, 226 },
    { "FIRE",    208, 226 },
    { "EARTH",   215, 226 },
    { "FOREST",  150, 226 },
    { "DESERT",  222, 226 },
    { "ARCTIC",  195, 226 },
};

/* §2 clock + §3 color */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* Load one theme's colours; falls back to cyan/yellow on poor terminals. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) {
        init_pair(PAIR_RECUR, t->recur, -1);
        init_pair(PAIR_DIAG,  t->diag,  -1);
    } else {
        init_pair(PAIR_RECUR, COLOR_CYAN, -1);
        init_pair(PAIR_DIAG,  COLOR_YELLOW, -1);
    }
}
static void color_init(void)
{ start_color(); use_default_colors();
  if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
  else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
  theme_apply(0); }

/* §5 physics — the Lorenz system, just one of the 30 signal sources */

/*
 * Lorenz is only one signal here (the SIG_LORENZ_X case), and we never
 * change its settings.  We still build it as system + state + composite,
 * matching the other chaos demos in this folder, so this section reads
 * as a tidy standalone model you can study on its own.
 */

/* The textbook Lorenz settings (Lorenz 1963).  Fixed here -- this file
 * only uses Lorenz as a chaotic signal, never sweeps the values. */
#define LORENZ_SIGMA   10.0f
#define LORENZ_RHO     28.0f
#define LORENZ_BETA    (8.0f / 3.0f)

/* Vec3 — a plain 3-number vector (used as a Lorenz point and as its slope). */
typedef struct {
    float x, y, z;
} Vec3;

/* Same as Vec3, renamed where we mean "a point on the Lorenz path" so the
 * step-math reads like motion rather than generic vector arithmetic. */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — the three fixed knobs of the Lorenz equations.  Kept
 * apart from the moving state so the slope function is a clean function
 * of (where we are, which system); also leaves room to sweep them later.
 * We always use the textbook values and never change them at runtime.
 *
 *   sigma : how fast the convection rolls respond.
 *   rho   : how hard the temperature difference drives the flow.
 *   beta  : a shape factor (8/3 in the original derivation).
 */
typedef struct {
    float sigma, rho, beta;
} LorenzSystem;

/*
 * Lorenz — the fixed knobs plus the current point: everything one step
 * of the integrator needs in a single value.
 *
 *   system : the (sigma, rho, beta) knobs -- never changed by a step.
 *   state  : the current (x, y, z) point -- moved by each step.
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

static inline LorenzSystem lorenz_system_default(void)
{
    return (LorenzSystem){ LORENZ_SIGMA, LORENZ_RHO, LORENZ_BETA };
}

/* Which way the Lorenz point is heading right now (the three Lorenz
 * equations, one per line). */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);
    dy.y = s->x * (sys->rho - s->z) - s->y;
    dy.z = s->x * s->y - sys->beta * s->z;
    return dy;
}

/* Take a step of size h along slope k, as a fresh point. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* Weights for the 4-step RK4 method (Numerical Recipes, ch. 17). */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* Blend the four trial slopes, weighting the two middle ones double. */
static inline LorenzState rk4_butcher_weighted_average(
    const LorenzState *k1, const LorenzState *k2,
    const LorenzState *k3, const LorenzState *k4)
{
    LorenzState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Move the Lorenz point forward one step.  RK4 takes four trial slopes
 * (start, two at the midpoint, end), then steps along their weighted
 * average -- far more accurate than a single slope for the same step. */
static void lorenz_rk4_full(Lorenz *L, float dt)
{
    const LorenzSystem *sys = &L->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    LorenzState slope_start    = lorenz_deriv(&L->state, sys);
    LorenzState midpoint_1     = state_add(&L->state, half_dt, &slope_start);
    LorenzState slope_mid_1    = lorenz_deriv(&midpoint_1, sys);
    LorenzState midpoint_2     = state_add(&L->state, half_dt, &slope_mid_1);
    LorenzState slope_mid_2    = lorenz_deriv(&midpoint_2, sys);
    LorenzState endpoint       = state_add(&L->state, dt, &slope_mid_2);
    LorenzState slope_end      = lorenz_deriv(&endpoint, sys);

    LorenzState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    L->state = state_add(&L->state, dt, &effective_slope);
}

/* Same step as above, but on a bare Vec3 so signal_step doesn't have to
 * carry a whole Lorenz around: wrap it, step it, copy the result back. */
static void lorenz_rk4(Vec3 *s, float dt)
{
    Lorenz tmp = { lorenz_system_default(), *s };
    lorenz_rk4_full(&tmp, dt);
    *s = tmp.state;
}

/* §6 recurrence matrix */

/*
 * RecurrenceImage — the signal we collected and the grid of dots built
 * from it.  Dot (i, j) is lit when sample i and sample j are within
 * epsilon of each other.  We compare single numbers (each signal is one
 * value per step) rather than full multi-dimensional states, which is the
 * simplest, "scalar" form of the technique.  The grid fills one row per
 * tick so you watch it draw itself.
 *
 *   n         : side of the square grid -- also how many samples we keep.
 *               Sized to fit the terminal, capped at N_MAX.
 *   samples[] : the signal values so far, x[0]..x[n-1].  Fixed-size so we
 *               never need malloc.
 *   R[]       : the n-by-n grid, stored as one flat row-major array (row i,
 *               column j lives at i*n + j).  One byte per cell -- it's just
 *               lit or not.
 *   rows_done : how many rows are filled; once it reaches n the picture is
 *               finished and the simulation idles.
 */
typedef struct {
    int     n;
    float   samples[N_MAX];
    uint8_t R[N_MAX * N_MAX];
    int     rows_done;
} RecurrenceImage;

static void recurrence_reset(RecurrenceImage *r, int n)
{
    r->n = n; r->rows_done = 0;
    memset(r->R, 0, sizeof r->R);
}

/* Fill row i: light each cell where sample i is within eps of sample j. */
static void recurrence_compute_row(RecurrenceImage *r, int i, float eps)
{
    for (int j = 0; j < r->n; j++) {
        float d = fabsf(r->samples[i] - r->samples[j]);
        r->R[i * r->n + j] = (d < eps) ? 1u : 0u;
    }
}

/* §7 state — small wrappers around the two things the user can cycle */

/*
 * PresetState — remembers which signal is selected.  It's just an index,
 * but wrapping it in its own type with next/prev helpers makes the key
 * handlers read as intentions and stops a "next theme" key from ever
 * touching the signal list by mistake.
 *
 *   current : row in presets[]; the helpers keep it in range by wrapping.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const RPPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — remembers which colour theme is active.  Same idea as
 * PresetState, deliberately a separate type so theme keys and signal keys
 * can't be crossed.
 *
 *   current : row in themes[]; the helpers keep it in range by wrapping.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)       { theme_apply(p->current); }

/* §8 scene */

/*
 * SignalState — the running memory the signal generators need.  Most
 * signals use just one field; the chaos ones use their own state field.
 * Keeping them all in one struct lets signal_step be a single switch with
 * no allocation -- the few unused floats cost nothing.  It's reset on
 * every restart, and the random seed is fixed, so a given signal always
 * draws the exact same picture.
 *
 *   t_phase       : how far along a repeating wave is (sine, square,
 *                   triangle, sawtooth, beat).  Steps forward by the
 *                   signal's speed each time.
 *   lorenz        : the moving point of the Lorenz attractor.  Starts at
 *                   (1, 1, 1).
 *   henon_x/y     : the moving point of the Hénon map.  Starts near (0.1, 0).
 *   walk_acc      : running total for the random-walk and Brownian signals.
 *   ar1_prev      : last value, for the "mostly-last-value" AR(1) signal.
 *   logistic_prev : last value, for the logistic and tent maps.  Lives in
 *                   (0, 1); starts at 0.5.
 *   rng_state     : the random-number generator's state.  Fixed start
 *                   value so every run is identical.
 *   sample_index  : how many samples we've made; used by the burst signal
 *                   (on/off blocks) and the step-jump signal.
 *   step_level    : the level the step-jump signal currently holds between
 *                   its random hops.
 */
typedef struct {
    float    t_phase;
    Vec3     lorenz;
    float    henon_x, henon_y;
    float    walk_acc;
    float    ar1_prev;
    float    logistic_prev;
    uint32_t rng_state;
    int      sample_index;
    float    step_level;
} SignalState;

/*
 * Scene — all the moving simulation state in one place, so the main loop
 * can drive it through a handful of scene_* calls.  Each piece is its own
 * named type, splitting "what makes the samples" from "what stores them"
 * from "what the user picked".
 *
 *   R         : the samples plus the grid of dots (built up row by row).
 *   collected : how many samples we have; when it reaches R.n the picture
 *               is finished and ticking stops.
 *   signal    : the generator's running memory.
 *   preset    : which signal is selected.
 *   palette   : which colour theme is selected.
 *   paused    : while true, ticks do nothing and the picture freezes.
 */
typedef struct {
    RecurrenceImage R;
    int             collected;
    SignalState     signal;
    PresetState     preset;
    PaletteState    palette;
    bool            paused;
} Scene;

/* Shortcuts for "the selected signal" and "its match threshold". */
static inline const RPPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static inline float scene_active_eps(const Scene *s)
{
    return scene_active_preset(s)->epsilon;
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* A tiny fast random generator (Marsaglia's xorshift).  Same start value
 * always replays the same sequence, which is what makes a given signal
 * draw the identical picture every time. */
static inline uint32_t rng_xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* A random float spread evenly across [-1, 1]. */
static inline float rng_uniform_signed(uint32_t *s)
{
    return (float)((int32_t)rng_xorshift32(s)) / (float)INT32_MAX;
}

/* Produce the next sample for the selected signal.  One short branch per
 * signal, so the whole switch reads as the full catalogue side by side. */
static float signal_step(SignalState *st, const RPPreset *p)
{
    st->sample_index++;

    switch (p->kind) {
    case SIG_CONSTANT:
        return p->p1;

    case SIG_LINEAR:
        /* a steady climb: value grows by p1 each step */
        return p->p1 * (float)st->sample_index;

    case SIG_SINE: {
        float v = sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_SQUARE: {
        float v = (sinf(st->t_phase) >= 0.0f) ? 1.0f : -1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_TRIANGLE: {
        /* climb up then back down, between -1 and 1 */
        float frac = fmodf(st->t_phase, 2.0f * (float)M_PI) / (2.0f * (float)M_PI);
        float v = 4.0f * fabsf(frac - 0.5f) - 1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_SAWTOOTH: {
        float frac = fmodf(st->t_phase, 2.0f * (float)M_PI) / (2.0f * (float)M_PI);
        float v = 2.0f * frac - 1.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_TWO_SINES: {
        float v = sinf(st->t_phase) + 0.5f * sinf(p->p2 / p->p1 * st->t_phase);
        st->t_phase += p->p1;
        return 0.6f * v;
    }

    case SIG_QUASI_GOLDEN: {
        const float golden = 1.6180339887f;
        float v = 0.5f * (sinf(st->t_phase) + sinf(golden * st->t_phase));
        st->t_phase += p->p1;
        return v;
    }

    case SIG_BEAT: {
        /* two nearly-equal tones that drift in and out of step */
        float v = 0.5f * (sinf(st->t_phase) + sinf(p->p2 / p->p1 * st->t_phase));
        st->t_phase += p->p1;
        return v;
    }

    case SIG_DAMPED_SINE: {
        float envelope = expf(-p->p2 * (float)st->sample_index * 0.05f);
        float v = envelope * sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_BURST: {
        /* sine on for a block of samples, silent for the next block */
        int burst_period = (int)p->p2;
        if (burst_period < 1) burst_period = 20;
        int phase = (st->sample_index / burst_period) & 1;
        float v = (phase == 0) ? sinf(st->t_phase) : 0.0f;
        st->t_phase += p->p1;
        return v;
    }

    case SIG_WANE: {
        /* sine whose loudness slowly drops to zero */
        float amp = 1.0f - p->p2 * (float)st->sample_index;
        if (amp < 0.0f) amp = 0.0f;
        float v = amp * sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_NOISE:
        return rng_uniform_signed(&st->rng_state);

    case SIG_RANDOM_WALK: {
        float step = 0.1f * rng_uniform_signed(&st->rng_state);
        st->walk_acc += step;
        return st->walk_acc;
    }

    case SIG_AR1: {
        /* mostly keep the last value, nudge it with a little noise */
        float alpha = p->p1;
        float noise = rng_uniform_signed(&st->rng_state);
        st->ar1_prev = alpha * st->ar1_prev + (1.0f - alpha) * noise;
        return st->ar1_prev;
    }

    case SIG_BROWNIAN: {
        float step = 0.3f * rng_uniform_signed(&st->rng_state);
        st->walk_acc += step;
        return st->walk_acc;
    }

    case SIG_LOGISTIC: {
        /* the classic chaos map: next = r * x * (1 - x) */
        float r = p->p1;
        st->logistic_prev = r * st->logistic_prev * (1.0f - st->logistic_prev);
        return st->logistic_prev;
    }

    case SIG_TENT: {
        /* fold the unit line in half each step, then stretch by mu */
        float mu = p->p1;
        float x  = st->logistic_prev;
        st->logistic_prev = mu * ((x < 0.5f) ? x : (1.0f - x));
        return st->logistic_prev;
    }

    case SIG_LORENZ_X: {
        /* move the Lorenz point a few steps, then read its x */
        for (int k = 0; k < 5; k++) lorenz_rk4(&st->lorenz, LORENZ_DT);
        return st->lorenz.x;
    }

    case SIG_HENON_X: {
        /* step the Hénon map and read its new x */
        float x = st->henon_x, y = st->henon_y;
        st->henon_x = 1.0f - HENON_A * x * x + y;
        st->henon_y = HENON_B * x;
        return st->henon_x;
    }

    case SIG_RAMP_SINE: {
        float v = p->p2 * (float)st->sample_index + sinf(st->t_phase);
        st->t_phase += p->p1;
        return v;
    }

    case SIG_STEP_JUMPS: {
        /* hold the current level; now and then (chance p1) hop to a new one */
        float roll = (float)rng_xorshift32(&st->rng_state) / (float)UINT32_MAX;
        if (roll < p->p1)
            st->step_level = 2.0f * (float)rng_xorshift32(&st->rng_state)
                           / (float)UINT32_MAX - 1.0f;
        return st->step_level;
    }
    }
    return 0.0f;   /* never reached -- the switch handles every kind */
}

static void scene_collect_one(Scene *s)
{
    if (s->collected >= s->R.n) return;
    s->R.samples[s->collected] = signal_step(&s->signal, scene_active_preset(s));
    s->collected++;
}

/* Wipe the picture and put the signal back to its fixed starting point,
 * so the current signal redraws from scratch -- identically every time. */
static void scene_reset(Scene *s, int side)
{
    int n = side; if (n > N_MAX) n = N_MAX;
    recurrence_reset(&s->R, n);
    s->collected = 0;

    /* same starting values every time -> same picture every time */
    s->signal.t_phase       = 0.0f;
    s->signal.lorenz        = (Vec3){ 1.0f, 1.0f, 1.0f };
    s->signal.henon_x       = 0.1f;
    s->signal.henon_y       = 0.0f;
    s->signal.walk_acc      = 0.0f;
    s->signal.ar1_prev      = 0.0f;
    s->signal.logistic_prev = 0.5f;
    s->signal.rng_state     = 0xCAFEBABEu;
    s->signal.sample_index  = 0;
    s->signal.step_level    = 0.0f;
}

static void scene_init(Scene *s, int side)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_SINE_MED);    /* a clear, simple start */
    palette_state_init(&s->palette, 0);
    scene_reset(s, side);
}

/* One step of the simulation: grab the next sample, then update the grid
 * so it matches every sample collected so far (ready to draw any time). */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;                                /* generators step once per tick, not by wall time */
    if (s->paused) return;
    if (s->collected >= s->R.n) return;      /* picture finished -- nothing to do */

    float eps = scene_active_eps(s);
    scene_collect_one(s);
    int i = s->collected - 1;

    recurrence_compute_row(&s->R, i, eps);

    /* The grid is symmetric, so cell (j, i) matches cell (i, j).  Earlier
     * rows were filled before sample i existed, leaving their column i
     * blank; light it now so the picture stays a mirror image. */
    for (int j = 0; j < i; j++) {
        float d = fabsf(s->R.samples[j] - s->R.samples[i]);
        s->R.R[j * s->R.n + i] = (d < eps) ? 1u : 0u;
    }
}

/* §9 screen */

/*
 * Screen — the current terminal size.  ncurses keeps width/height in
 * globals; copying them here gives the drawing code one tidy handle and
 * one place to update when the window is resized.
 *
 *   cols : terminal width  in characters.
 *   rows : terminal height in characters.
 */
typedef struct { int cols, rows; } Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* Draw the grid centred on screen: '#' for a match, '\' down the middle
 * diagonal (where every sample matches itself), blank for no match. */
static void recurrence_paint(const RecurrenceImage *r, int cols, int rows)
{
    int n = r->n;
    int gx0 = (cols - n) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - n) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;

    for (int i = 0; i < n; i++) {
        int sy = gy0 + i;
        if (sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) continue;
        for (int j = 0; j < n; j++) {
            int sx = gx0 + j;
            if (sx < 0 || sx >= cols) continue;
            if (i == j) {
                attron(COLOR_PAIR(PAIR_DIAG) | A_BOLD);
                mvaddch(sy, sx, '\\');
                attroff(COLOR_PAIR(PAIR_DIAG) | A_BOLD);
            } else if (r->R[i * n + j]) {
                attron(COLOR_PAIR(PAIR_RECUR) | A_BOLD);
                mvaddch(sy, sx, '#');
                attroff(COLOR_PAIR(PAIR_RECUR) | A_BOLD);
            }
        }
    }
}

static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " RECURRENCE PLOT ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    const RPPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  ε:%.2f  %d/%d ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)active->epsilon,
             s->collected, s->R.n);
    int hx = cols - (int)strlen(buf); if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static void hud_param(const Scene *s)
{
    int x = HUD_LEFT_MARGIN;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", scene_active_preset(s)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 19;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " R(i,j)=1 if |x_i-x_j|<ε    matrix:%dx%d ", s->R.n, s->R.n);
    attroff(COLOR_PAIR(PAIR_HUD));
}
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
/* Lay out one full frame: clear, draw the grid, then the HUD lines. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{ erase(); recurrence_paint(&s->R, sc->cols, sc->rows);
  hud_top(sc->cols, fps, sim_fps, s); hud_param(s); hud_hint(sc->rows); }

/* §10 app */

/*
 * App — the whole program in one struct.  It lives at file scope as g_app
 * so the signal handlers can flip its flags directly.  main() only ever
 * touches it through the named app_* helpers, never the fields raw.
 *
 *   scene       : all the simulation state.
 *   screen      : the cached terminal size.
 *   sim_fps     : how many simulation steps per second (10..240).
 *   side        : the grid size, recomputed to fit the window on resize.
 *   running     : stays true until 'q'/ESC or a quit signal.
 *   need_resize : set by the window-resize signal, handled next loop.
 *                 Both flags are volatile sig_atomic_t -- the rule for a
 *                 value written in a signal handler and read in the loop.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   side;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Catch Ctrl-C / kill as "quit" and window-resize as "relayout". */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_pick_side(App *app)
{
    int w = app->screen.cols;
    int h = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    int side = (w < h) ? w : h;
    if (side > N_MAX) side = N_MAX;
    if (side < 16)    side = 16;
    app->side = side;
}

/* One-time startup: ncurses, colours, and the first scene. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_side(app);
    scene_init(&app->scene, app->side);
}

/* Window changed size: re-measure and rebuild the grid to fit. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_side(app);
    scene_reset(&app->scene, app->side);
    app->need_resize = 0;
}

/* How long since the last frame, capped so a long stall can't make the
 * simulation try to catch up with a huge burst of steps. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Run as many fixed-size simulation steps as the elapsed time has earned,
 * so the simulation runs at its own steady rate no matter the frame rate. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the shown fps a couple of times a second so it reads steadily. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep off the rest of the frame's time budget to hold ~60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and show it. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed up / slow down the simulation, kept within the allowed range. */
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

/* Named one-liners so the key table below reads like plain intentions. */
static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_picture     (App *app) { scene_reset(&app->scene, app->side); }
static void app_cycle_theme_next  (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev  (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->side);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->side);
}

static bool app_handle_key(App *app, int ch);

/* Check for a keypress without blocking; returns false to mean "quit". */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The key map: each key calls one named action. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_picture    (app); break;
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
 * FrameClock — the loop's timekeeping in one bundle, so main() reads as
 * "start clock, advance clock, run the steps it earned".
 *
 *   frame_time  : when the last frame started.
 *   sim_accum   : leftover time not yet turned into simulation steps.
 *   fps_accum   : time piled up since we last recalculated fps.
 *   frame_count : frames drawn since we last recalculated fps.
 *   fps_display : the fps number on screen (refreshed twice a second so
 *                 it doesn't flicker).
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

/* The whole program: set up, then loop -- handle resize, advance the
 * clock, run the simulation steps it earned, draw, and check for keys. */
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
