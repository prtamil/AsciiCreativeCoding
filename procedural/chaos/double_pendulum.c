/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * double_pendulum.c — a pendulum hung from another pendulum, swinging
 * forever with no friction.  The bottom weight's path is drawn as a
 * fading trail; press n/p to flip between 30 named starting positions,
 * from gentle wobbles to wild chaos.  Tiny changes to the start lead to
 * wildly different motion, which is the whole point of the demo.
 *
 * The equations of motion come from Shinbrot et al., "Chaos in a double
 * pendulum", Am. J. Phys. 60(6) 1992.  Sister demos in this folder:
 * bifurcation.c and strange_attractor.c (other classic chaos examples).
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

/* §1  config */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,   /* 4 trail-age band slots: +0..+3       */
    PAIR_ROD_TOP      =   7,
    PAIR_ROD_BOT      =   8,
    PAIR_BOB          =   9,
    PAIR_PIVOT        =  10,
    PAIR_JOINT        =  11,   /* intermediate joint 'O' between rods  */
};

/* HUD layout */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* The physics runs in fine-grained "pixels"; a character cell is this
 * many pixels wide and tall.  Dividing pixels by these gives the
 * terminal cell to draw in. */
#define CELL_W                   8
#define CELL_H                  16

/* Aim for 60 drawn frames a second. */
#define RENDER_FPS_TARGET       60
#define NS_PER_SEC              1000000000LL
#define NS_PER_MS                  1000000LL
#define TICK_NS(f)              (NS_PER_SEC / (f))
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS    100   /* never advance physics more than this in one frame */

/* The two bobs weigh the same (1 is just a convenient number — only
 * the ratio matters).  The arm length and gravity get worked out from
 * the terminal size later so the pendulum fills the screen at any size
 * while still swinging at the same comfortable pace. */
#define PEND_M1                   1.0f
#define PEND_M2                   1.0f

/* How long each arm is, as a fraction of the height we can draw in.
 * Both arms together have to fit in the top half above the pivot so
 * the wilder presets can loop the bottom weight over the top.  The
 * MIN/MAX keep it sane on tiny or huge terminals. */
#define ARM_CELLS_OF_DRAWABLE     0.22f
#define ARM_CELLS_MIN                4
#define ARM_CELLS_MAX               14

/* We tie gravity to arm length so the swing always takes about 2
 * seconds no matter how big the arms end up.  10 is the ratio that
 * lands on that 2-second feel. */
#define PEND_GRAVITY_PER_LENGTH  10.0f

/* The motion is too fast and twitchy to compute in one jump per drawn
 * frame, so each frame is split into this many tiny sub-steps.  Smaller
 * steps keep the math from blowing up when the bob whips around. */
#define INT_STEPS_PER_TICK         8

/* How many recent bottom-weight positions the fading trail remembers. */
#define TRAIL_MAX               4000
#define TRAIL_BAND_COUNT           4

/*
 * Preset — which of the 30 named starting positions is loaded.  They
 * are grouped from tamest to wildest so cycling with n/p walks you
 * through the whole range of behaviour:
 *
 *   Group A  tiny gentle wobbles                  (6)
 *   Group B  moderate swings, some velocity kicks (6)
 *   Group C  right on the edge of going chaotic   (5)
 *   Group D  full-blown chaos                     (7)
 *   Group E  spinning all the way round           (4)
 *   Group F  lopsided exotic cases                (2)
 *
 * The per-entry numbers below are the starting angles and spins; the
 * short note on each says what it looks like.
 */
typedef enum {
    /* Group A — tiny gentle wobbles */
    PRESET_WHISPER = 0,   /* (5°, 0°, 0, 0)        top-only tiny tap   */
    PRESET_MIRROR,        /* (15°, -15°, 0, 0)     pure anti-phase     */
    PRESET_PARALLEL,      /* (20°, 20°, 0, 0)      pure in-phase       */
    PRESET_BEATING,       /* (10°, -25°, 0, 0)     mixed modes → beats */
    PRESET_TWANG,         /* (0°, 30°, 0, 0)       bottom-only release */
    PRESET_KNOCK,         /* (30°, 0°, 0, 0)       top-only release    */

    /* Group B — moderate swings, some velocity kicks */
    PRESET_SWING,         /* (45°, 30°, 0, 0)      classic in-phase    */
    PRESET_CROSS,         /* (60°, -60°, 0, 0)     wide anti-phase     */
    PRESET_CRESCENT,      /* (80°, 80°, 0, 0)      both near horizontal */
    PRESET_SHEAR,         /* (90°, -45°, 0, 0)     top horizontal      */
    PRESET_NUDGE,         /* (0°, 0°, 0, 2)        pure ω₂ kick        */
    PRESET_PROD,          /* (0°, 0°, 3, 0)        pure ω₁ kick        */

    /* Group C — right on the edge of going chaotic */
    PRESET_PEEL,          /* (45°, 90°, 0, 0)      bottom horizontal   */
    PRESET_ELBOW,         /* (90°, 90°, 0, 0)      L-shape both at 90° */
    PRESET_KINK,          /* (90°, -90°, 0, 0)     L-shape mirrored    */
    PRESET_STAB,          /* (120°, 0°, 0, 0)      top past horizontal */
    PRESET_TIPTOE,        /* (170°, 0°, 0, 0)      top near inverted   */

    /* Group D — full-blown chaos */
    PRESET_CHAOS,         /* (135°, 175°, 0, 0)    classic chaos demo  */
    PRESET_CASCADE,       /* (170°, 170°, 0, 0)    both near top in-phase */
    PRESET_TWIST,         /* (160°, -160°, 0, 0)   both near top mirrored */
    PRESET_STORM,         /* (100°, 200°, 0, 0)    bottom past inverted*/
    PRESET_RUMBLE,        /* (45°, 90°, 3, 0)      angles + top spin   */
    PRESET_SHAKE,         /* (60°, -60°, 0, 3)     counter-angles + spin */
    PRESET_RIOT,          /* (179°, 1°, 0, 0)      knife-edge inverted */

    /* Group E — spinning all the way round */
    PRESET_SPIN,          /* (0°, 0°, 5, 0)        top-only rotation   */
    PRESET_ORBIT,         /* (0°, 0°, 5, 5)        co-rotating both    */
    PRESET_ENGINE,        /* (0°, 0°, 5, -5)       counter-rotating    */
    PRESET_PROPLR,        /* (0°, 0°, 10, 0)       fast top spin       */

    /* Group F — lopsided exotic cases */
    PRESET_STAR,          /* (45°, -135°, 3, -3)   opposing angle + spin */
    PRESET_NOVA,          /* (160°, 30°, -4, 6)    wild asymmetric chaos */

    N_PRESETS,
} Preset;

/*
 * PendulumPreset — one named starting position.  Everything else about
 * the pendulum (arm lengths, masses, gravity) stays the same all demo
 * long; the only thing a preset changes is where the two arms start and
 * how fast they're already moving.  That is the heart of the demo: the
 * same machine, set going from slightly different starts, behaves wildly
 * differently.  Read-only — these rows are never changed at runtime.
 *
 *   name       : short label shown in the readout.  Space-padded so the
 *                readout doesn't jiggle as you flip presets.  Names hint
 *                at the look ("WHISPER" = tiny tap, "RIOT" = almost
 *                balanced upside-down, "NOVA" = wild lopsided chaos).
 *   theta1_deg : starting angle of the TOP arm, in degrees.  0 = hanging
 *                straight down, 90 = sideways, 180 = pointing straight up.
 *   theta2_deg : starting angle of the BOTTOM arm, same scale.  Measured
 *                from straight-down too, not relative to the top arm.
 *   omega1     : how fast the top arm is already turning at the start,
 *                in radians per second (0 in most presets; a non-zero
 *                value gives it a shove).
 *   omega2     : same for the bottom arm.  In the spinning presets these
 *                two together set which way and how fast it whirls.
 */
typedef struct {
    const char *name;
    float       theta1_deg;
    float       theta2_deg;
    float       omega1;
    float       omega2;
} PendulumPreset;

/* The 30 starting positions, ordered tamest to wildest.  Columns are
 * name, top angle, bottom angle, top spin, bottom spin. */
static const PendulumPreset presets[N_PRESETS] = {
    /* Group A — tiny gentle wobbles */
    { "WHISPER ",    5.0f,    0.0f,  0.0f,  0.0f },
    { "MIRROR  ",   15.0f,  -15.0f,  0.0f,  0.0f },
    { "PARALLEL",   20.0f,   20.0f,  0.0f,  0.0f },
    { "BEATING ",   10.0f,  -25.0f,  0.0f,  0.0f },
    { "TWANG   ",    0.0f,   30.0f,  0.0f,  0.0f },
    { "KNOCK   ",   30.0f,    0.0f,  0.0f,  0.0f },

    /* Group B — moderate swings, some velocity kicks */
    { "SWING   ",   45.0f,   30.0f,  0.0f,  0.0f },
    { "CROSS   ",   60.0f,  -60.0f,  0.0f,  0.0f },
    { "CRESCENT",   80.0f,   80.0f,  0.0f,  0.0f },
    { "SHEAR   ",   90.0f,  -45.0f,  0.0f,  0.0f },
    { "NUDGE   ",    0.0f,    0.0f,  0.0f,  2.0f },
    { "PROD    ",    0.0f,    0.0f,  3.0f,  0.0f },

    /* Group C — right on the edge of going chaotic */
    { "PEEL    ",   45.0f,   90.0f,  0.0f,  0.0f },
    { "ELBOW   ",   90.0f,   90.0f,  0.0f,  0.0f },
    { "KINK    ",   90.0f,  -90.0f,  0.0f,  0.0f },
    { "STAB    ",  120.0f,    0.0f,  0.0f,  0.0f },
    { "TIPTOE  ",  170.0f,    0.0f,  0.0f,  0.0f },

    /* Group D — full-blown chaos */
    { "CHAOS   ",  135.0f,  175.0f,  0.0f,  0.0f },
    { "CASCADE ",  170.0f,  170.0f,  0.0f,  0.0f },
    { "TWIST   ",  160.0f, -160.0f,  0.0f,  0.0f },
    { "STORM   ",  100.0f,  200.0f,  0.0f,  0.0f },
    { "RUMBLE  ",   45.0f,   90.0f,  3.0f,  0.0f },
    { "SHAKE   ",   60.0f,  -60.0f,  0.0f,  3.0f },
    { "RIOT    ",  179.0f,    1.0f,  0.0f,  0.0f },

    /* Group E — spinning all the way round */
    { "SPIN    ",    0.0f,    0.0f,  5.0f,  0.0f },
    { "ORBIT   ",    0.0f,    0.0f,  5.0f,  5.0f },
    { "ENGINE  ",    0.0f,    0.0f,  5.0f, -5.0f },
    { "PROPLR  ",    0.0f,    0.0f, 10.0f,  0.0f },

    /* Group F — lopsided exotic cases */
    { "STAR    ",   45.0f, -135.0f,  3.0f, -3.0f },
    { "NOVA    ",  160.0f,   30.0f, -4.0f,  6.0f },
};

/*
 * Theme — one full set of colours for everything on screen.  Pressing
 * t/T swaps the whole look without touching any of the drawing or
 * physics code: every glyph picks its colour from exactly one field
 * here.  Read-only; t/T just chooses which row is in use.
 *
 *   name        : short label shown in the readout, space-padded so the
 *                 layout stays put when names change.
 *   band[0..N-1]: the fade colours for the trail, from oldest (band[0],
 *                 dimmest) to newest (band[N-1], brightest).  Keep every
 *                 colour bright enough to see on a black background —
 *                 the project rule is xterm-256 index 24 or higher, or
 *                 the dimmest dots vanish.
 *   rod_top     : colour of the top arm.  Usually the brighter of the
 *                 two arm colours so the eye reads it first.
 *   rod_bot     : colour of the bottom arm.  Slightly different from the
 *                 top so the bend between them reads as two arms, not
 *                 one long stick.
 *   joint       : the 'O' where the two arms meet.  Picked between the
 *                 bottom-arm colour and the weight colour so it looks
 *                 like a connecting point, not another weight.
 *   bob         : the '(@)' weight at the very end — the thing the trail
 *                 is tracing.  Usually the boldest colour in the theme.
 *   pivot       : the '[+]' fixed point it all hangs from.  Picked to
 *                 read as a calm, fixed anchor (often a neutral grey).
 */
typedef struct {
    const char *name;
    short       band[TRAIL_BAND_COUNT];
    short       rod_top;
    short       rod_bot;
    short       joint;
    short       bob;
    short       pivot;
} Theme;

#define N_THEMES 10

/* Even the dimmest trail colour is kept bright enough to see on black. */
static const Theme themes[N_THEMES] = {
    { "DEFAULT",  {  75, 123, 220, 231 }, 220, 208, 214, 196, 231 },
    { "MATRIX",   {  77, 118, 156, 194 }, 156, 118, 119,  82, 231 },
    { "NOVA",     { 135, 171, 207, 219 }, 213, 207, 213, 219, 231 },
    { "MONO",     { 247, 250, 253, 255 }, 254, 253, 254, 255, 231 },
    { "OCEAN",    {  81, 117, 159, 195 }, 159, 117,  81,  51, 231 },
    { "FIRE",     { 208, 214, 220, 227 }, 226, 208, 220, 196, 231 },
    { "EARTH",    { 143, 179, 215, 222 }, 222, 215, 215, 173, 231 },
    { "FOREST",   { 114, 150, 157, 194 }, 156, 150, 150, 119, 231 },
    { "DESERT",   { 179, 215, 222, 229 }, 222, 215, 215, 179, 231 },
    { "ARCTIC",   { 117, 159, 195, 231 }, 231, 195, 195, 117, 231 },
};

/* §2  clock */

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

/* §3  color */

/* Hand each on-screen part its colour from the chosen theme, on a
 * 256-colour terminal.  Drawing on the terminal's own background (-1)
 * means the demo looks right on a dark or a light terminal. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_ROD_TOP, t->rod_top, -1);
    init_pair(PAIR_ROD_BOT, t->rod_bot, -1);
    init_pair(PAIR_JOINT,   t->joint,   -1);
    init_pair(PAIR_BOB,     t->bob,     -1);
    init_pair(PAIR_PIVOT,   t->pivot,   -1);
}

/* Old 8-colour terminals can't show the rich themes, so every theme
 * falls back to this one fixed set of basic colours.  We lose the
 * variety but stay readable everywhere. */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_ROD_TOP, COLOR_YELLOW,  -1);
    init_pair(PAIR_ROD_BOT, COLOR_MAGENTA, -1);
    init_pair(PAIR_JOINT,   COLOR_YELLOW,  -1);
    init_pair(PAIR_BOB,     COLOR_RED,     -1);
    init_pair(PAIR_PIVOT,   COLOR_WHITE,   -1);
}

/* Apply theme number `idx`, using the rich or the fallback colours
 * depending on what the terminal can do. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* §4  coord — pixel <-> cell */

static inline int px_to_cell_x(float px) { return (int)(px / (float)CELL_W); }
static inline int px_to_cell_y(float py) { return (int)(py / (float)CELL_H); }

/* §5  physics — the swinging double pendulum */

/*
 * PendulumSystem — the fixed facts about the pendulum that never change
 * while it swings: how long the arms are, how heavy the weights are, and
 * how strong gravity is.  Kept apart from where it's hanging (the anchor)
 * and how it's moving right now (the state) because those three things
 * change at different times and for different reasons.
 *
 *   L1, L2 : arm lengths in pixels (equal here; the math allows unequal).
 *   m1, m2 : the two weights (just 1 here).
 *   g      : strength of gravity in pixels per second per second.  Scaled
 *            along with arm length so the swing keeps the same pace at
 *            any terminal size.
 */
typedef struct {
    float L1, L2, m1, m2, g;
} PendulumSystem;

/*
 * PendulumAnchor — the fixed point on screen the whole thing hangs from.
 * Kept separate from the physics because moving the pendulum around the
 * screen doesn't change how it swings, only where you see it.
 *
 *   px, py : the pivot's position in pixels.  Placed at the middle of
 *            the drawable area — middle vertically as well as across, so
 *            the wilder presets have room to loop the weight up over the
 *            top of the pivot.
 */
typedef struct {
    float px, py;
} PendulumAnchor;

/*
 * PendulumState — how the pendulum is moving right this instant: the two
 * arm angles and how fast each is turning.  These four numbers are the
 * only thing that changes as time passes; the step function rewrites them
 * many times a second.  It's a small plain struct so we can copy it around
 * cheaply while computing each step.
 *
 *   th1, th2 : the two arm angles in radians, measured from straight-down.
 *              Left to grow without limit (they can pass a full turn when
 *              the arm loops over the top) — sin/cos cope with any value,
 *              and snapping them back into a range would jolt the step math.
 *   w1,  w2  : how fast each arm is turning, in radians per second.  No
 *              limit; in the spinning presets these can get large.
 */
typedef struct {
    float th1, th2;
    float w1,  w2;
} PendulumState;

/*
 * DoublePendulum — the whole pendulum in one bundle: its fixed facts
 * (system), where it hangs (anchor), and how it's moving now (state).
 * Grouped so a function that needs "the pendulum" takes one pointer.
 * The stepping touches only state; resizing touches only system and
 * anchor; drawing reads all three.
 */
typedef struct {
    PendulumSystem system;
    PendulumAnchor anchor;
    PendulumState  state;
} DoublePendulum;

/* The 6 we divide by when blending the four slope estimates below. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f

/* The "half" in "step half a tick" used by two of the four slope tries. */
#define RK4_MIDPOINT_FRACTION    0.5f

/* A shared bottom-of-the-fraction term that shows up in both arms'
 * acceleration formulas — roughly, how strongly the two arms tug on each
 * other given the angle between them.  It's always positive for real
 * weights, so it's safe to divide by without checking.  This is the
 * common denominator from Shinbrot eq. (3). */
static inline float lagrangian_mass_coupling_denominator(const PendulumSystem *sys,
                                                         float angle_diff)
{
    return 2.0f * sys->m1 + sys->m2 - sys->m2 * cosf(2.0f * angle_diff);
}

/* How fast the TOP arm's spin is speeding up or slowing down right now.
 * This is Shinbrot eq. (1).  It adds up three pulls on the top arm:
 * gravity trying to swing it back down, a tug back from the bottom arm's
 * weight, and the throw from both arms whirling.  The (a)/(b)/(c) tags on
 * the lines below match those three pulls. */
static inline float lagrangian_upper_angular_acceleration(
    const PendulumState *s, const PendulumSystem *sys,
    float sin_diff, float cos_diff, float denom)
{
    float upper_inertia_factor  = 2.0f * sys->m1 + sys->m2;
    float centripetal_upper     = s->w1 * s->w1 * sys->L1;  /* top arm's whirl */
    float centripetal_lower     = s->w2 * s->w2 * sys->L2;  /* bottom arm's whirl */

    float gravity_restoring_term =
        -sys->g * upper_inertia_factor * sinf(s->th1);                 /* (a) */
    float gravity_back_reaction_term =
        -sys->m2 * sys->g * sinf(s->th1 - 2.0f * s->th2);              /* (b) */
    float centripetal_coupling_term =
        -2.0f * sin_diff * sys->m2 *
            (centripetal_lower + centripetal_upper * cos_diff);        /* (c) */

    float numerator = gravity_restoring_term
                    + gravity_back_reaction_term
                    + centripetal_coupling_term;
    return numerator / (sys->L1 * denom);
}

/* How fast the BOTTOM arm's spin is changing right now — Shinbrot
 * eq. (2).  Three things drive it: the throw fed down from the top arm's
 * spin, gravity acting on the bottom weight, and the bottom arm's own
 * whirl feeding back.  The (a)/(b)/(c) tags match those three. */
static inline float lagrangian_lower_angular_acceleration(
    const PendulumState *s, const PendulumSystem *sys,
    float sin_diff, float cos_diff, float denom)
{
    float total_mass        = sys->m1 + sys->m2;
    float centripetal_upper = s->w1 * s->w1 * sys->L1;
    float centripetal_lower = s->w2 * s->w2 * sys->L2;

    float centripetal_feedforward_term =
        centripetal_upper * total_mass;                                /* (a) */
    float gravity_projection_term =
        sys->g * total_mass * cosf(s->th1);                            /* (b) */
    float centripetal_feedback_term =
        centripetal_lower * sys->m2 * cos_diff;                        /* (c) */

    float numerator = 2.0f * sin_diff *
                      (centripetal_feedforward_term
                     + gravity_projection_term
                     + centripetal_feedback_term);
    return numerator / (sys->L2 * denom);
}

/* The heart of the simulation: given where the pendulum is right now,
 * work out how all four numbers are changing — the two arms turn at their
 * current spin speeds, and the two spin speeds change by the accelerations
 * above.  Everything else in the file is just plumbing around this.  The
 * shared angle-between-the-arms terms are computed once and reused. */
static PendulumState pendulum_deriv(const PendulumState *s,
                                    const PendulumSystem *sys)
{
    float angle_diff = s->th1 - s->th2;
    float sin_diff   = sinf(angle_diff);
    float cos_diff   = cosf(angle_diff);
    float denom      = lagrangian_mass_coupling_denominator(sys, angle_diff);

    PendulumState out;
    out.th1 = s->w1;
    out.th2 = s->w2;

    out.w1 = lagrangian_upper_angular_acceleration(s, sys, sin_diff, cos_diff, denom);
    out.w2 = lagrangian_lower_angular_acceleration(s, sys, sin_diff, cos_diff, denom);
    return out;
}

/* Nudge every number in a state forward by h times a slope, as a copy. */
static inline PendulumState state_add(const PendulumState *a,
                                      float h, const PendulumState *k)
{
    PendulumState r;
    r.th1 = a->th1 + h * k->th1;
    r.th2 = a->th2 + h * k->th2;
    r.w1  = a->w1  + h * k->w1;
    r.w2  = a->w2  + h * k->w2;
    return r;
}

/* Blend the four slope guesses into one good average slope, weighting the
 * two middle guesses twice as much: (k1 + 2k2 + 2k3 + k4) / 6.  This is the
 * classic Runge-Kutta recipe; the middle of the step matters most. */
static inline PendulumState rk4_butcher_weighted_average(
    const PendulumState *k1, const PendulumState *k2,
    const PendulumState *k3, const PendulumState *k4)
{
    PendulumState avg;
    avg.th1 = (k1->th1 + 2.0f * k2->th1 + 2.0f * k3->th1 + k4->th1)
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.th2 = (k1->th2 + 2.0f * k2->th2 + 2.0f * k3->th2 + k4->th2)
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.w1  = (k1->w1  + 2.0f * k2->w1  + 2.0f * k3->w1  + k4->w1 )
              / RK4_BUTCHER_WEIGHT_SUM;
    avg.w2  = (k1->w2  + 2.0f * k2->w2  + 2.0f * k3->w2  + k4->w2 )
              / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Step the pendulum forward by a slice of time dt.  Naively you'd just
 * follow the current slope, but that drifts.  Instead we sample the slope
 * four times across the step — once at the start, twice in the middle,
 * once at the end — then move along their weighted average.  This is the
 * Runge-Kutta 4 method: far more accurate per step than the naive way, and
 * accurate enough that the motion's energy stays steady on screen. */
static void pendulum_rk4(DoublePendulum *p, float dt)
{
    const PendulumSystem *sys = &p->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    /* slope at the start */
    PendulumState slope_start    = pendulum_deriv(&p->state, sys);

    /* slope at the midpoint, using the start slope to get there */
    PendulumState midpoint_state_1 = state_add(&p->state, half_dt, &slope_start);
    PendulumState slope_midpoint_1 = pendulum_deriv(&midpoint_state_1, sys);

    /* slope at the midpoint again, this time using the previous midpoint slope */
    PendulumState midpoint_state_2 = state_add(&p->state, half_dt, &slope_midpoint_1);
    PendulumState slope_midpoint_2 = pendulum_deriv(&midpoint_state_2, sys);

    /* slope at the end */
    PendulumState endpoint_state   = state_add(&p->state, dt, &slope_midpoint_2);
    PendulumState slope_end        = pendulum_deriv(&endpoint_state, sys);

    /* move along the blended slope */
    PendulumState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_midpoint_1, &slope_midpoint_2, &slope_end);
    p->state = state_add(&p->state, dt, &effective_slope);
}

/* Turn the two arm angles into the on-screen positions of the two weights.
 * y grows downward on a terminal, so cos goes with the y step here. */
static void pendulum_bob_positions(const DoublePendulum *p,
                                   float *b1x, float *b1y,
                                   float *b2x, float *b2y)
{
    *b1x = p->anchor.px + p->system.L1 * sinf(p->state.th1);
    *b1y = p->anchor.py + p->system.L1 * cosf(p->state.th1);
    *b2x = *b1x         + p->system.L2 * sinf(p->state.th2);
    *b2y = *b1y         + p->system.L2 * cosf(p->state.th2);
}

/* Load a preset's starting angles and spins into the live state,
 * converting the angles from degrees to radians on the way in. */
static void pendulum_state_init_from_preset(PendulumState *s,
                                            const PendulumPreset *preset)
{
    s->th1 = preset->theta1_deg * (float)M_PI / 180.0f;
    s->th2 = preset->theta2_deg * (float)M_PI / 180.0f;
    s->w1  = preset->omega1;
    s->w2  = preset->omega2;
}

/* §6  trail — the fading path the bottom weight leaves behind */

/*
 * Trail — the last TRAIL_MAX positions of the bottom weight, so we can
 * redraw its scribbled path each frame with older points faded out.  That
 * scribble IS the demo: a single frozen frame would just look like an
 * ordinary pendulum; only the building-up trail shows the chaos.
 *
 * It's a ring buffer — a fixed array used round-and-round.  When it fills
 * up, the newest position overwrites the oldest.  We chose this because it
 * never grows (no allocating mid-run) and costs the same every frame no
 * matter how long the program has been running.
 *
 *   px[i], py[i] : the i-th stored position, kept in fine pixel units so
 *                  the path stays smooth; converted to terminal cells only
 *                  when it's drawn.
 *   head         : where the newest position sits.  push() steps head
 *                  forward first, then writes, so head is always the slot
 *                  just written.
 *   count        : how many positions are stored so far.  Climbs until the
 *                  array is full, then stays at TRAIL_MAX.  Lets the
 *                  drawing code skip empty slots early on.
 */
typedef struct {
    float px[TRAIL_MAX];
    float py[TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }

static void trail_push(Trail *t, float px, float py)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->px[t->head] = px;
    t->py[t->head] = py;
    if (t->count < TRAIL_MAX) t->count++;
}

/* Which slot holds the oldest position still in the trail.  The extra
 * "+ TRAIL_MAX" keeps the result positive — C's % on a negative number
 * isn't reliable. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* Sort a trail point into one of the fade levels by its age: newest
 * points land in the brightest level, oldest in the dimmest. */
static inline int trail_band_for_age(int age, int count)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age * TRAIL_BAND_COUNT) / count;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* §7  state — which preset and which theme are picked */

/*
 * PresetState — remembers which of the 30 presets is loaded.  It's just an
 * index, but wrapping it in its own type lets the helpers below ("go to
 * next", "look up the active one") hang off it, so the key handler reads as
 * plain intentions instead of array-wraparound math.
 *
 *   current : row number in the presets[] table.  The next/prev helpers
 *             wrap it around so it always stays in range.  Starts on the
 *             CHAOS preset so the demo opens on something dramatic.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial)        { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)               { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)               { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const PendulumPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — remembers which colour theme is showing.  Same shape as
 * PresetState, but kept as its own type on purpose: it indexes a different
 * table, and giving each its own type stops a "next theme" key from ever
 * accidentally walking off into the preset list.
 *
 *   current : row number in the themes[] table.  Starts at 0, the most
 *             readable default theme.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial)      { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)             { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)             { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)   { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)            { theme_apply(p->current); }

/* §8  scene */

/*
 * Scene — everything about the running demo that changes, gathered in one
 * place so the rest of the program drives a single object instead of a pile
 * of loose globals.
 *
 *   pendulum : the pendulum itself (its facts, where it hangs, how it moves).
 *   trail    : the fading path the bottom weight has traced.
 *   preset   : which starting position is loaded (n/p change it).
 *   palette  : which colour theme is showing (t/T change it).
 *   paused   : when true the physics freezes but the screen keeps drawing,
 *              so you can study a frozen pose.
 *   map_w/h  : how many cells wide and tall the drawing area is (the
 *              terminal minus the rows reserved for the readout).  The
 *              pendulum is sized to fill this, so it fits any window.
 */
typedef struct {
    DoublePendulum pendulum;
    Trail          trail;
    PresetState    preset;
    PaletteState   palette;
    bool           paused;
    int            map_w, map_h;
} Scene;

/* Smallest vertical room we'll let the pendulum have; below this it
 * shrinks to nothing and stops looking like a pendulum. */
#define GEOMETRY_DRAWABLE_HEIGHT_FLOOR  8

/* Shortest arm we'll allow once everything else has been squeezed; two
 * 3-cell arms is the smallest that still reads as a pendulum. */
#define GEOMETRY_ARM_CELLS_PER_HALF_FLOOR  3

/* Where to put the pivot: dead centre across, and half-way down the
 * drawing area (the half-cell nudge sits it on the middle of a row). */
#define GEOMETRY_PIVOT_HORIZONTAL_FRACTION         0.5f
#define GEOMETRY_PIVOT_VERTICAL_CELL_HALF_OFFSET   0.5f

/* How many rows tall the drawing area is, after setting aside the readout
 * rows, never dropping below a usable minimum. */
static int geometry_drawable_height(int map_h)
{
    int draw_h = map_h - HUD_BAND_RESERVED_ROWS;
    if (draw_h < GEOMETRY_DRAWABLE_HEIGHT_FLOOR)
        draw_h = GEOMETRY_DRAWABLE_HEIGHT_FLOOR;
    return draw_h;
}

/* Pick an arm length that fills the available height.  First clamp it to a
 * sensible min/max, then shrink it if needed so both arms fit in the half
 * above the centred pivot — that leaves room for the weight to loop over
 * the top in the chaotic presets. */
static int geometry_pick_arm_cells(int draw_h)
{
    int arm_cells = (int)((float)draw_h * ARM_CELLS_OF_DRAWABLE);

    if (arm_cells < ARM_CELLS_MIN) arm_cells = ARM_CELLS_MIN;
    if (arm_cells > ARM_CELLS_MAX) arm_cells = ARM_CELLS_MAX;

    int half_drawable = draw_h / 2;
    int max_arm_for_swing_over = (half_drawable - 1) / 2;
    if (arm_cells > max_arm_for_swing_over)
        arm_cells = max_arm_for_swing_over;
    if (arm_cells < GEOMETRY_ARM_CELLS_PER_HALF_FLOOR)
        arm_cells = GEOMETRY_ARM_CELLS_PER_HALF_FLOOR;
    return arm_cells;
}

/* Set the arm lengths from the chosen size, and scale gravity to match so
 * the swing keeps the same ~2-second pace at any terminal size. */
static void geometry_set_physics_scale(PendulumSystem *sys, int arm_cells)
{
    float arm_length_px = (float)arm_cells * (float)CELL_H;
    sys->L1 = arm_length_px;
    sys->L2 = arm_length_px;
    sys->g  = PEND_GRAVITY_PER_LENGTH * arm_length_px;
}

/* Put the pivot in the middle of the drawing area — middle vertically too,
 * not at the top, so the weight has room to loop up over it. */
static void geometry_set_pivot_center(PendulumAnchor *anc, int map_w, int draw_h)
{
    int draw_mid_row = HUD_TOP_ROWS + draw_h / 2;
    anc->px = GEOMETRY_PIVOT_HORIZONTAL_FRACTION
            * (float)map_w * (float)CELL_W;
    anc->py = ((float)draw_mid_row + GEOMETRY_PIVOT_VERTICAL_CELL_HALF_OFFSET)
            * (float)CELL_H;
}

/* Size the pendulum to the current window: find the room, pick arm
 * lengths, set the physics scale, and centre the pivot. */
static void scene_compute_geometry(Scene *s)
{
    int draw_h    = geometry_drawable_height(s->map_h);
    int arm_cells = geometry_pick_arm_cells(draw_h);
    geometry_set_physics_scale(&s->pendulum.system, arm_cells);
    geometry_set_pivot_center (&s->pendulum.anchor, s->map_w, draw_h);
}

/* Load the currently-chosen preset's start into the pendulum.  Only the
 * motion changes — the size and pivot stay put. */
static void scene_load_active_preset(Scene *s)
{
    pendulum_state_init_from_preset(&s->pendulum.state,
                                    preset_state_active(&s->preset));
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s)
{
    scene_load_active_preset(s);
    trail_reset(&s->trail);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->map_w  = mw;
    s->map_h  = mh;
    s->paused = false;

    preset_state_init(&s->preset,  PRESET_CHAOS);  /* open on the dramatic one */
    palette_state_init(&s->palette, 0);

    s->pendulum.system.m1 = PEND_M1;
    s->pendulum.system.m2 = PEND_M2;
    scene_compute_geometry(s);
    scene_reset(s);
}

static void scene_resize(Scene *s, int mw, int mh)
{
    s->map_w = mw;
    s->map_h = mh;
    scene_compute_geometry(s);
    scene_reset(s);
}

/* Move the pendulum forward one tick, in several tiny sub-steps for
 * stability, then record the new bottom-weight spot on the trail. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float sub_dt = dt / (float)INT_STEPS_PER_TICK;
    for (int i = 0; i < INT_STEPS_PER_TICK; i++)
        pendulum_rk4(&s->pendulum, sub_dt);

    float b1x, b1y, b2x, b2y;
    pendulum_bob_positions(&s->pendulum, &b1x, &b1y, &b2x, &b2y);
    trail_push(&s->trail, b2x, b2y);
}

/* §9  screen — drawing everything to the terminal */

/*
 * Screen — a small handle for the terminal: just its current size, plus
 * the few functions that bring ncurses up and take it down.  Passing one
 * of these around means a drawing function asks it for the size instead of
 * everyone querying the terminal themselves, and all the ncurses setup and
 * teardown lives in one spot.
 *
 *   cols : terminal width in character cells.
 *   rows : terminal height in character cells.  Both are refreshed only on
 *          start-up and on resize.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *sc)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc)   { (void)sc; endwin(); }
static void screen_resize(Screen *sc) { endwin(); refresh();
                                        getmaxyx(stdscr, sc->rows, sc->cols); }
static void screen_present(void)      { wnoutrefresh(stdscr); doupdate(); }

/* True if a cell is inside the drawing area, so we never scribble over the
 * readout rows at the top and bottom. */
static inline bool in_drawable(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols
        && cy >= HUD_TOP_ROWS && cy < rows - HUD_BOTTOM_ROWS;
}

/* Pick the character that makes an arm look like a real line at its current
 * slant: '-' for a flat bit, '|' for a steep bit, '/' or '\' for a slope. */
static inline chtype bresenham_glyph_for_step(bool step_x, bool step_y,
                                              int sx, int sy)
{
    if (step_x && step_y) return (sx == sy) ? '\\' : '/';
    if (step_x)           return '-';
    return '|';
}

/* Draw one arm as a straight line of characters between two points, using
 * the slant-aware character at each spot.  This is Bresenham's line method:
 * it walks the line one cell at a time using only integer steps.  Cells
 * outside the drawing area are skipped but still walked, so the line stays
 * straight even when part of it is off-area. */
static void draw_rod(int x0, int y0, int x1, int y1, int pair,
                     int cols, int rows)
{
    int delta_x        = abs(x1 - x0);
    int delta_y        = abs(y1 - y0);
    int step_dir_x     = (x0 < x1) ? 1 : -1;
    int step_dir_y     = (y0 < y1) ? 1 : -1;
    int error_accumul  = delta_x - delta_y;

    attron(COLOR_PAIR(pair) | A_BOLD);
    for (;;) {
        if (in_drawable(x0, y0, cols, rows)) {
            int doubled_error = 2 * error_accumul;
            bool will_step_x  = (doubled_error > -delta_y);
            bool will_step_y  = (doubled_error <  delta_x);
            chtype glyph = bresenham_glyph_for_step(will_step_x, will_step_y,
                                                    step_dir_x,  step_dir_y);
            mvaddch(y0, x0, glyph);
        }
        if (x0 == x1 && y0 == y1) break;

        int doubled_error = 2 * error_accumul;
        if (doubled_error > -delta_y) { error_accumul -= delta_y; x0 += step_dir_x; }
        if (doubled_error <  delta_x) { error_accumul += delta_x; y0 += step_dir_y; }
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the fixed top point as '[+]', or just '+' if it's jammed against
 * the screen edge. */
static void draw_pivot_marker(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
    if (cx > 0 && cx < cols - 1) {
        mvaddch(cy, cx - 1, '[');
        mvaddch(cy, cx,     '+');
        mvaddch(cy, cx + 1, ']');
    } else {
        mvaddch(cy, cx, '+');
    }
    attroff(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
}

/* Draw the 'O' where the two arms meet.  Drawn after the arms so it sits
 * on top of them. */
static void draw_joint(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_JOINT) | A_BOLD);
    mvaddch(cy, cx, 'O');
    attroff(COLOR_PAIR(PAIR_JOINT) | A_BOLD);
}

/* Draw the bottom weight as '(@)' so it reads as a chunky weight, or just
 * '@' near the screen edge. */
static void draw_end_bob(int cx, int cy, int cols, int rows)
{
    if (!in_drawable(cx, cy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_BOB) | A_BOLD);
    if (cx > 0 && cx < cols - 1) {
        mvaddch(cy, cx - 1, '(');
        mvaddch(cy, cx,     '@');
        mvaddch(cy, cx + 1, ')');
    } else {
        mvaddch(cy, cx, '@');
    }
    attroff(COLOR_PAIR(PAIR_BOB) | A_BOLD);
}

/* Newer parts of the trail get a bolder mark, older parts a fainter one,
 * so the tail reads as fading even apart from its colour. */
static inline char trail_glyph_for_band(int band)
{
    if (band >= TRAIL_BAND_COUNT - 1)   return 'o';   /* newest */
    if (band == TRAIL_BAND_COUNT - 2)   return '.';
    if (band == TRAIL_BAND_COUNT - 3)   return '.';
    return ',';                                       /* oldest */
}

/* Draw the whole trail from oldest to newest, fading each point's colour
 * and mark by its age, so it looks like a tail trailing off behind the
 * weight. */
static void trail_paint(const Trail *t, int cols, int rows)
{
    if (t->count == 0) return;

    int oldest = trail_oldest_index(t);

    for (int i = 0; i < t->count; i++) {
        int sample_index = (oldest + i) % TRAIL_MAX;
        int age          = t->count - 1 - i;          /* 0 = newest */
        int band         = trail_band_for_age(age, t->count);

        int cx = px_to_cell_x(t->px[sample_index]);
        int cy = px_to_cell_y(t->py[sample_index]);
        if (!in_drawable(cx, cy, cols, rows)) continue;

        int pair = PAIR_TRAIL_BASE + band;
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(cy, cx, (chtype)(unsigned char)trail_glyph_for_band(band));
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * PendulumCellPositions — the three points the drawing code needs, already
 * turned into terminal cells: the pivot at the top, the joint between the
 * arms, and the weight at the tip.  Working these out up front keeps the
 * drawing function a clean list of "draw this, then that".
 */
typedef struct {
    int anchor_x, anchor_y;
    int joint_x,  joint_y;
    int tip_x,    tip_y;
} PendulumCellPositions;

/* Work out the pivot, joint, and tip in terminal cells from the pendulum's
 * current angles. */
static PendulumCellPositions pendulum_compute_cell_positions(const DoublePendulum *p)
{
    float b1x, b1y, b2x, b2y;
    pendulum_bob_positions(p, &b1x, &b1y, &b2x, &b2y);

    PendulumCellPositions pos;
    pos.anchor_x = px_to_cell_x(p->anchor.px);
    pos.anchor_y = px_to_cell_y(p->anchor.py);
    pos.joint_x  = px_to_cell_x(b1x);
    pos.joint_y  = px_to_cell_y(b1y);
    pos.tip_x    = px_to_cell_x(b2x);
    pos.tip_y    = px_to_cell_y(b2y);
    return pos;
}

/* Draw one frame, back to front so the markers land on top: trail first,
 * then the two arms, then the pivot, joint, and weight. */
static void scene_paint(const Scene *s, int cols, int rows)
{
    PendulumCellPositions p = pendulum_compute_cell_positions(&s->pendulum);

    trail_paint(&s->trail, cols, rows);
    draw_rod         (p.anchor_x, p.anchor_y, p.joint_x, p.joint_y,
                      PAIR_ROD_TOP, cols, rows);
    draw_rod         (p.joint_x,  p.joint_y,  p.tip_x,   p.tip_y,
                      PAIR_ROD_BOT, cols, rows);
    draw_pivot_marker(p.anchor_x, p.anchor_y, cols, rows);
    draw_joint       (p.joint_x,  p.joint_y,  cols, rows);
    draw_end_bob     (p.tip_x,    p.tip_y,    cols, rows);
}

static void hud_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DOUBLE PENDULUM ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    char buf[HUD_COLS + 1];
    const PendulumPreset *active = preset_state_active(&s->preset);
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d] ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Fixed widths of the preset and theme boxes (text plus its spaces) so the
 * next box always starts in the same spot, however long the names are. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

/* The second readout line: current preset, current theme, and trail length
 * side by side.  The trail count is dimmer since it's just a side note. */
static void hud_param_row(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " preset:%-8s ", preset_state_active(&s->preset)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " trail:%4d/%d ", s->trail.count, TRAIL_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_bottom_hint(int rows)
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
    hud_top_left_title();
    hud_top_right_status(sc->cols, fps, sim_fps, s);
    hud_param_row(s);
    hud_bottom_hint(sc->rows);
}

/* §10 app — startup, the main loop, and key handling */

/*
 * App — the whole program in one struct: the scene, the terminal, the
 * physics speed, the window size, and two flags the signal handlers set.
 * Nothing else holds program state, so the rest of the file just does one
 * named thing to the App at a time.
 *
 *   scene       : the simulation — what you actually see.
 *   screen      : the terminal it's drawn on.
 *   sim_fps     : how many times a second the physics steps.  Separate from
 *                 the 60-fps drawing rate; ] and [ change it.
 *   map_w/map_h : the drawing area in cells, remembered here so the sizing
 *                 code has steady numbers to work from; recomputed on resize.
 *   running     : set to 0 to stop the program — by q/ESC or by a kill
 *                 signal.  Marked volatile sig_atomic_t because a signal
 *                 handler writes it, and that's the only kind of variable a
 *                 handler is allowed to touch safely.
 *   need_resize : set to 1 by the resize signal; the next frame notices it
 *                 and rebuilds.  Same volatile sig_atomic_t reason.
 *
 * There's exactly one of these, the global g_app, because a signal handler
 * has to reach it and signal handlers can't be handed a pointer.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Tiny signal handlers — a handler may only flip a flag, nothing more. */
static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Hook up the signals: Ctrl-C / kill quit, a window resize asks for a
 * rebuild.  The cleanup runs at exit so the terminal is always restored. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Work out the drawing area: the terminal minus the readout rows, kept
 * within sensible limits.  Run at start-up and on every resize. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* First-time setup: start ncurses, measure the terminal, size the drawing
 * area, and build the scene. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* If the window was resized, rebuild the screen and re-fit the scene.  The
 * caller resets its clock afterward so the rebuild time isn't counted. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_resize(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* How much time passed since the last frame, capped so that a long stall
 * (say the window was buried for a while) doesn't dump a huge backlog of
 * physics onto the next frame. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Step the physics in fixed-size ticks, taking as many as the saved-up time
 * allows.  Fixed ticks matter here: a chaotic system run with variable-size
 * steps would behave differently on a fast machine than a slow one, so we
 * keep the step size constant no matter the frame rate. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Every half-second or so, work out the recent frames-per-second for the
 * readout and start a fresh count. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep just enough to hold a steady frame rate.  We sleep before drawing
 * so the time spent drawing doesn't throw off the pacing. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the scene and readout, then flip it to the screen in one go. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed up / slow down the physics, kept within its allowed range. */
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

static void app_toggle_pause   (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_pendulum (App *app) { scene_reset(&app->scene); }

/* Switch to the next / previous preset and start it fresh — the reset also
 * wipes the old trail so it doesn't bleed into the new one. */
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

/* Switch to the next / previous colour theme.  No reset — the pendulum
 * keeps swinging right through the colour change. */
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

/* Read a key if one is waiting.  Returns false only when it's time to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Map each key to one action.  Returns false on quit. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_pendulum   (app); break;
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
 * FrameClock — the bits of timekeeping the main loop carries from one
 * frame to the next, gathered in one place instead of as loose variables.
 *
 *   frame_time  : when the last frame happened, used to measure how much
 *                 time has passed.
 *   sim_accum   : time that has built up but not yet been turned into
 *                 physics steps.
 *   fps_accum   : time built up since the last frames-per-second readout.
 *   frame_count : frames counted since that last readout.
 *   fps_display : the latest frames-per-second figure shown in the readout.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

/* Start the clock now and clear the counters. */
static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

/* Restart the clock after a resize so the time spent rebuilding the screen
 * doesn't get counted as a giant frame. */
static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

/* Fold this frame's elapsed time into the physics and fps counters. */
static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* Set up, then loop: handle any resize, step the physics, draw a frame, and
 * read a key — until something asks to quit.  Then tidy up. */
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
