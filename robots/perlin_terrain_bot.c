/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_terrain_bot.c — a two-wheeled self-balancing robot (think Segway)
 * driving across rolling Perlin-noise terrain. A PID controller reads the
 * body's lean and pushes the wheels to keep it upright; on slopes it leans
 * the bot into the hill. Side panel shows live telemetry, the equations, or
 * a phase-space picture of how the controller behaves.
 *
 * Sister demos: robots/diff_drive_robot.c (wheels, no balancing),
 *   robots/moving_jump_spring_leg_robot.c (same terrain, hopping leg).
 * Key references: Perlin "Improving Noise" (SIGGRAPH 2002, the quintic
 *   fade used in §4) https://mrl.cs.nyu.edu/~perlin/paper445.pdf ;
 *   Åström & Murray "Feedback Systems" (free PDF, https://fbsbook.org) for
 *   the PID; Fiedler "Fix Your Timestep!" for the sub-stepped main loop.
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — every tunable in one place ──────────────────────────── */

/* §1.1 frame rate + sub-step target */
enum {
    TARGET_FPS = 60,
};

/* We split each frame into sub-steps no bigger than this so the physics
 * integrator stays stable even when frames come slowly; MAX_SUBSTEPS caps
 * the loop so one very long frame can't spin forever. */
#define TICK_DT_TARGET   (1.0f / 120.0f)
#define MAX_SUBSTEPS     16

/* §1.2 cell pixel dimensions. Terminal cells are about twice as tall as
 * wide. Physics runs in pixels; we convert to cells only at draw time. */
#define CELL_W   8
#define CELL_H  16

/*
 * §1.3 cart-pole physics (SI units).
 * GRAVITY   — m/s², standard Earth value.
 * PEND_LEN  — m, rod length from axle to body's centre of mass. Longer rod
 *             sways more slowly.
 * MASS_CART — kg, chassis + wheels.
 * MASS_POLE — kg, the body above the axle.
 * MAX_FORCE — N, the strongest push the motor can give (it can't exceed this).
 * FALL_ANGLE — rad, ~60°. Past this we give up and stop ticking — even a
 *             max push couldn't recover in time.
 */
#define GRAVITY      9.81f
#define PEND_LEN     1.00f
#define MASS_CART    4.00f
#define MASS_POLE    2.00f
#define MAX_FORCE  200.00f
#define FALL_ANGLE   1.05f

/*
 * §1.4 PID gains + anti-windup. The three gains weight the controller's
 * three terms (see PID struct in §6). Bigger Kp corrects faster but
 * overshoots; bigger Ki removes slow drift but can "wind up"; bigger Kd
 * damps wobble but slows response. WINDUP_MAX caps how much the integral
 * term is allowed to accumulate so it can't run away.
 */
#define KP_DEF      120.0f
#define KI_DEF        0.20f
#define KD_DEF       18.0f
#define WINDUP_MAX    5.00f

/*
 * §1.5 slope feed-forward. How much of the terrain slope we hand the
 * controller as a head start so the bot leans into the hill on its own.
 * 1.0 would cancel the slope perfectly in theory but reacts too sharply;
 * 0.65 leaves the PID room to smooth out the rest.
 */
#define SLOPE_FEED   0.65f

/* §1.6 robot geometry, in pixels. */
#define WHEEL_R      18.0f      /* wheel radius (only used to lift the axle) */
#define AXLE_HW      26.0f      /* half the gap between the two wheels */
#define BODY_H       96.0f      /* body length, axle up to the top mass */

/* §1.7 drive speed (pixels/sec) — default, per-keypress step, and limits;
 * PIX_PER_M converts pixels to metres for the HUD only. */
#define DRIVE_DEF    55.0f
#define DRIVE_STEP   15.0f
#define DRIVE_MAX   160.0f
#define DRIVE_MIN     0.0f
#define PIX_PER_M   100.0f

/*
 * §1.8 terrain shape.
 * TBUF   — size of the height ring buffer (power of two).
 * T_FREQ — how fast the noise wiggles per column; smaller = wider hills.
 * T_AMP_F — hill height as a fraction of the screen.
 * T_MID_F — where the average ground level sits down the screen.
 */
#define TBUF       1024
#define TMASK      (TBUF - 1)
#define T_FREQ      0.022f
#define T_AMP_F     0.20f
#define T_MID_F     0.62f

/* §1.9 how many (θ, ω) samples the phase-portrait trail remembers. */
enum { HIST_LEN = 240 };

/* §1.10 view modes */
typedef enum {
    VIEW_TELEMETRY = 0,
    VIEW_EQUATIONS,
    VIEW_PHASE,
    VIEW_COUNT
} ViewMode;

static const char *VIEW_NAMES[VIEW_COUNT] = {
    "TELEMETRY", "EQUATIONS", "PHASE-SPACE"
};

/* §1.11 longest dt we'll trust. If a frame stalls, we cap dt here so the
 * physics doesn't take one giant unstable jump (the "spiral of death"). */
#define DT_CAP_SEC  0.10f

/* §1.12 ncurses colour-pair IDs */
enum {
    /* 1..2 — sky & stars */
    CP_SKY      = 1,
    CP_STAR,
    /* 3..4 — terrain */
    CP_SURF,
    CP_ROCK,
    /* 5..7 — robot */
    CP_CHASSIS,
    CP_WHEEL,
    CP_BEACON,
    /* 8..10 — UI */
    CP_DIM,             /* dim text                            */
    CP_GOOD,            /* green positive indicator            */
    CP_WARN,            /* red warning                         */
    /* 11..12 — value text */
    CP_VAL,             /* live numeric values                 */
    CP_EQ,              /* equation labels                     */
    /* 13..14 — bar gauge */
    CP_BAR_POS,
    CP_BAR_NEG,
    /* 15..16 — HUD spec */
    PAIR_HUD,
    PAIR_HINT,
};

/* §1.13 nanosecond conversions for the monotonic clock. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* §1.14 HUD text buffer size + width of the right-side panel (cells). */
#define HUD_BUF_LEN  160
#define PANEL_W       32

/* §1.15 startup + manual-tuning constants. */

/* The bot starts tilted a hair (~2.3°) instead of perfectly upright, so the
 * controller has a real error to push against on the very first tick. */
#define INITIAL_LEAN_RAD     0.04f

/* If a step's dt is smaller than this we skip the D term — dividing such a
 * tiny number would be all noise, no signal. */
#define PID_DT_FLOOR         1e-9f

/* '+' / '-' keys nudge Kp by this much, up to this ceiling. */
#define KP_NUDGE             5.0f
#define KP_MAX_USER        400.0f

/* §1.16 telemetry bar gauges + warning colours. */

/* Bar size and where it starts, in cells, inside each telemetry row. */
#define BAR_WIDTH            14
#define BAR_COL_OFFSET       16

/* What a fully-filled bar means for each value — chosen so the bar maxes out
 * right when that quantity gets alarming. */
#define THETA_BAR_RANGE_DEG  35.0f   /* lean angle */
#define OMEGA_BAR_RANGE       6.0f   /* angular velocity */
#define SLOPE_BAR_RANGE_DEG  25.0f   /* terrain slope */
#define D_BAR_FRAC            0.4f   /* D bar fills at this fraction of MAX_FORCE */
#define SAT_WARN_FRAC         0.85f  /* motor force turns red past this much of max */

/* Readouts turn red once they pass these angles. */
#define THETA_WARN_DEG       20.0f   /* lean */
#define SLOPE_WARN_DEG       15.0f   /* slope */

/* §1.17 phase-portrait plot. */

/* Plot box size in cells, centred in the side panel. */
#define PHASE_PLOT_W         29
#define PHASE_PLOT_H         12

/* How much of each axis the box spans; anything bigger gets clipped. */
#define PHASE_TH_RANGE        0.6f   /* θ axis, rad */
#define PHASE_OM_RANGE        5.0f   /* ω axis, rad/sec */

/* Trail colours fade with age (newest red, then yellow, oldest grey) so you
 * can read the direction of time in a still picture. */
#define TRAIL_NEW_BAND        0.85f
#define TRAIL_MID_BAND        0.50f

/* Thresholds the phase panel uses to label what the controller is doing. */
#define KD_UNDERDAMP_LIMIT    1.0f   /* Kd below this → wobbly (underdamped) */
#define KD_OVERDAMP_LIMIT    50.0f   /* Kd above this → sluggish (overdamped) */
#define CONVERGED_ERR_RAD     0.03f  /* lean error below this → settled */

/* §1.18 rendering odds and ends. */

/* The body-top beacon blinks once per this-much travel (not wall-clock time),
 * so it slows when the bot stops and ties the blink to the moving terrain. */
#define BEACON_PULSE_HZ       4.0f

/* When the ground rises/falls more than this fraction of a cell, draw the
 * surface as '/' or '\\' instead of '_' to show the slope. */
#define SLOPE_GLYPH_FRAC      0.22f

/* Print the "+12.3°" lean label this many cells left of the axle so it
 * doesn't sit on top of the bot. */
#define LEAN_LABEL_DC         6

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

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

/* ── §3 color — one fixed colour per on-screen element ───────────────── */

/*
 * One row per visual role: the colour-pair id, the colour to use on a
 * 256-colour terminal, and a plain-8-colour fallback. color_init() walks
 * this table once and binds each pair, so the whole palette reads top to
 * bottom in one place.
 */
typedef struct {
    short pair;         /* which colour-pair id this row sets (see §1.12) */
    short fg256;        /* colour on a 256-colour terminal */
    short fg8;          /* colour on a plain 8-colour terminal */
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair         256-mode  8-mode-fallback   role                 */
    { CP_SKY,         25,     COLOR_BLUE     }, /* dark blue          */
    { CP_STAR,       226,     COLOR_YELLOW   }, /* yellow             */
    { CP_SURF,        46,     COLOR_GREEN    }, /* bright green       */
    { CP_ROCK,        28,     COLOR_GREEN    }, /* dim green          */
    { CP_CHASSIS,    255,     COLOR_WHITE    }, /* near-white         */
    { CP_WHEEL,       51,     COLOR_CYAN     }, /* cyan               */
    { CP_BEACON,     196,     COLOR_RED      }, /* red                */
    { CP_DIM,        244,     COLOR_WHITE    }, /* grey               */
    { CP_GOOD,        82,     COLOR_GREEN    }, /* lime               */
    { CP_WARN,       196,     COLOR_RED      }, /* red                */
    { CP_VAL,        229,     COLOR_YELLOW   }, /* pale yellow        */
    { CP_EQ,         159,     COLOR_CYAN     }, /* light cyan         */
    { CP_BAR_POS,     51,     COLOR_CYAN     }, /* cyan               */
    { CP_BAR_NEG,    201,     COLOR_MAGENTA  }, /* magenta            */
    { PAIR_HUD,      226,     COLOR_YELLOW   }, /* yellow on default  */
    { PAIR_HINT,      51,     COLOR_CYAN     }, /* cyan on default    */
};
#define PALETTE_LEN  (int)(sizeof PALETTE / sizeof PALETTE[0])

static void color_init(void)
{
    start_color();
    use_default_colors();

    bool truecolor = (COLORS >= 256);
    for (int i = 0; i < PALETTE_LEN; i++) {
        short fg = truecolor ? PALETTE[i].fg256 : PALETTE[i].fg8;
        init_pair(PALETTE[i].pair, fg, -1);
    }
}

/* ── §4 noise — Perlin gradient noise, 1-D, plus fBm ─────────────────── */

/*
 * Noise — the seeded state that decides which random-but-smooth landscape
 * we get. It's a shuffled lookup table; the seed picks the shuffle, and the
 * shuffle picks the terrain. Wrapping it in a struct (instead of a hidden
 * global) means every function that uses noise takes a pointer to one, so
 * you can see at each call who depends on it — and a future demo could have
 * two independent noise fields (say terrain and clouds).
 *
 * The table holds a shuffle of 0..255, then a second copy of the same
 * shuffle right after it. The duplicate lets perlin1() read perm[i] and
 * perm[i+1] without ever running off the end — no wrap check needed.
 *
 * References: Perlin "An Image Synthesizer" (SIGGRAPH 1985), "Improving
 *   Noise" (SIGGRAPH 2002, the smoother fade we use below),
 *   https://mrl.cs.nyu.edu/~perlin/paper445.pdf .
 */
typedef struct {
    unsigned char perm[512];   /* [0..255] is a shuffle of 0..255 (set by   *
                                * noise_init); [256..511] repeats it so a   *
                                * perm[i+1] lookup never wraps.             */
} Noise;

static void noise_init(Noise *n, unsigned int seed)
{
    unsigned char p[256];
    srand(seed);
    for (int i = 0; i < 256; i++) p[i] = (unsigned char)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = p[i]; p[i] = p[j]; p[j] = t;
    }
    for (int i = 0; i < 512; i++) n->perm[i] = p[i & 255];
}

static inline float fade (float t) { return t*t*t*(t*(t*6.0f - 15.0f) + 10.0f); }
static inline float lerp (float a, float b, float t) { return a + t * (b - a); }
static inline float grad1(int h, float x) { return (h & 1) ? x : -x; }

static float perlin1(const Noise *n, float x)
{
    int   xi = (int)floorf(x) & 255;
    float xf = x - floorf(x);
    return lerp(grad1(n->perm[xi],     xf       ),
                grad1(n->perm[xi + 1], xf - 1.0f),
                fade(xf));
}

/*
 * fbm — stacks 5 copies of the noise, each one twice as wiggly and half as
 * tall as the last, and adds them up. Big copies give the broad hills, small
 * copies add fine roughness — the mix is what makes it look like real land.
 * Result lands roughly in [-1, +1].
 */
static float fbm(const Noise *n, float x)
{
    float v = 0.0f, amp = 0.5f, freq = 1.0f;
    for (int i = 0; i < 5; i++) {
        v   += amp * perlin1(n, x * freq);
        amp *= 0.5f;
        freq *= 2.0f;
    }
    return v;
}

/* ── §5 terrain — height lookup, slope, scrolling ring buffer ────────── */

/*
 * Terrain — the ground, stored as one height per world column. The bot only
 * ever drives right, so we compute each new column's height once and keep it
 * in a ring buffer; the screen only ever shows a window of columns, and old
 * ones behind the camera get overwritten (we never look back at them). This
 * also keeps us inside the "no malloc after startup" rule — fixed-size buffer,
 * no growing array.
 *
 * Heights are in pixels, the same units the physics uses, so the bot can read
 * the ground directly with no conversion; pixels become screen rows only at
 * draw time. The noise pointer is borrowed — Scene owns and frees it, Terrain
 * only reads it (that's what the const says).
 */
typedef struct {
    const Noise *noise;     /* borrowed from Scene; read-only, never freed here */

    float h[TBUF];          /* ground height per column, in pixels down from the *
                              * top of the screen (bigger = lower). Indexed by    *
                              * wc & TMASK; a slot is valid once gen_col reaches it.*/

    int   gen_col;          /* highest column filled so far. Starts at -1 and only *
                              * grows, as the bot reaches new ground.              */

    int   rows;             /* terminal height in cells; sets how tall/where the   *
                              * hills sit. Only changes on a resize.               */
} Terrain;

/* World y of the ground at a column: middle line plus the noise times the
 * hill height, both scaled to the current screen size. */
static float terrain_h_at(const Terrain *t, int wc)
{
    float mid = (float)t->rows * T_MID_F * (float)CELL_H;
    float amp = (float)t->rows * T_AMP_F * (float)CELL_H;
    return mid + fbm(t->noise, (float)wc * T_FREQ) * amp;
}

/*
 * Terrain slope at a column, in radians: how steeply the ground tilts there,
 * found from the height difference to the next column. We flip the sign so
 * "uphill to the right" comes out positive — the sign the controller expects.
 */
static float terrain_slope_at(const Terrain *t, int wc)
{
    float dh = t->h[(wc + 1) & TMASK] - t->h[wc & TMASK];
    return -atanf(dh / (float)CELL_W);
}

static void terrain_ensure(Terrain *t, int upto)
{
    for (int c = t->gen_col + 1; c <= upto; c++)
        t->h[c & TMASK] = terrain_h_at(t, c);
    if (upto > t->gen_col) t->gen_col = upto;
}

static void terrain_init(Terrain *t, const Noise *n, int rows, int cols)
{
    t->noise   = n;
    t->rows    = rows;
    t->gen_col = -1;
    terrain_ensure(t, cols + 64);
}

/* ── §6 bot — state, PID controller, cart-pole physics, reset ────────── */

/* §6.1 gain presets — the 'g'-key teaching cycle */

/*
 * GainPreset — one set of PID gains plus a two-line note for the phase
 * panel. Pressing 'g' swaps between these so you can watch the trail change
 * shape (tight spiral vs. wide circles vs. drift) and read, in plain words,
 * which knob caused it. They live in the file on purpose — the contrast is
 * the lesson, and a config file would hide it from a first-time reader.
 */
typedef struct {
    const char *name;       /* short label in the HUD, padded to 8 chars to line up */

    const char *lesson_l1;  /* two lines of plain-language explanation shown in the *
                              * phase panel; kept under ~28 chars to fit the panel. */
    const char *lesson_l2;

    float       kp;         /* proportional gain — higher stiffens correction but   *
                              * overshoots more. */
    float       ki;         /* integral gain — higher kills slow drift faster but   *
                              * risks winding up. */
    float       kd;         /* derivative gain — higher damps wobble but slows the  *
                              * response. The "NO Kd" preset sets it to 0 to show    *
                              * the bot wobbling forever. */
} GainPreset;

static const GainPreset PRESETS[] = {
    { "BALANCED",
      "well-tuned baseline.",
      "fast, damped, slope-aware.",
      120.0f, 0.20f, 18.0f },

    { "HIGH Kp ",
      "stiffer P → more overshoot",
      "& oscillation on slopes.",
      240.0f, 0.20f, 18.0f },

    { "NO Kd   ",
      "no damping → underdamped:",
      "bot oscillates forever.",
      120.0f, 0.20f,  0.0f },

    { "NO Ki   ",
      "no integral → steady-state",
      "drift on every slope.",
      120.0f, 0.00f, 18.0f },
};
#define N_PRESETS  (int)(sizeof PRESETS / sizeof PRESETS[0])

/* §6.2 Bot — built from six small structs, one per idea */

/*
 * The bot's state is split into six small pieces so each field's job is
 * obvious where you use it: b->pend.theta is clearly the pendulum, b->pid.kp
 * clearly a controller gain, b->mot.world_x clearly its position. Same data,
 * just grouped by what it means.
 */

/*
 * §6.2.1 Pendulum — the two numbers that fully describe the body's motion:
 * its lean angle and how fast that angle is changing. Everything else the
 * physics computes (in CartPole) can be rebuilt from these two plus the slope
 * and the motor force; they're the real "state". Positive theta = leaning
 * right; positive omega = leaning further right. The controller's job is to
 * drive theta to its target and omega to zero.
 */
typedef struct {
    float theta;        /* lean angle from straight-up-relative-to-chassis, rad. *
                          * Starts at a tiny tilt; once the gravity-frame lean    *
                          * passes FALL_ANGLE the bot is declared fallen.        */

    float omega;        /* how fast theta is changing, rad/sec. Same sign as     *
                          * theta. Reset to 0 on bot_reset().                    */
} Pendulum;

/*
 * §6.2.2 CartPole — the values the physics works out each step from the
 * pendulum, the slope, and the motor force. None of these are independent
 * state; they're caches. We keep them as named fields for two reasons: the
 * "equations" panel shows them live, and naming each step lets the physics
 * code read line by line like the textbook instead of a tangle of temporaries.
 * The math is the standard cart-pole-on-a-slope (Spong/Hutchinson/Vidyasagar
 * 2006, §6.5).
 */
typedef struct {
    float theta_eff;    /* the lean that gravity actually feels = theta + slope. *
                          * Gravity pulls straight down, not square to the        *
                          * chassis, so this — not theta — drives the physics and *
                          * the fall test.                                        */

    float theta_ddot;   /* how fast the body's lean is accelerating this step,   *
                          * rad/s². Gravity tipping it, minus the push-back from  *
                          * the cart accelerating.                                */

    float x_ddot;       /* how fast the wheels are accelerating sideways, m/s².  *
                          * This is the lever the controller uses — shoving the   *
                          * base one way tips the body the other way.             */

    float M_eff;        /* effective weight the motor force has to move, kg.     *
                          * Least when the body is straight up (just the cart),   *
                          * more as it leans (some of the body drags along).      */

    float F;            /* motor force on the wheels this step, N. Set by        *
                          * pid_step (capped at ±MAX_FORCE), used by              *
                          * cart_pole_step. Zero when the PID is off, so the bot  *
                          * tips over in about a second.                          */
} CartPole;

/*
 * §6.2.3 PID — the controller that keeps the bot up. Each step it looks at
 * how far the lean is from where it should be (the error) and pushes the
 * wheels by a blend of three reactions: P pushes proportional to the current
 * error, I pushes based on error piled up over time (kills slow drift), and D
 * pushes against how fast the error is changing (damps wobble). Two real-world
 * safety valves: the I term is capped so it can't pile up forever (anti-
 * windup), and the final push is capped at the motor's limit. The per-term
 * outputs are stored separately so the equations panel can show the breakdown.
 *
 * On a slope, the "where it should be" target is tilted into the hill ahead of
 * time (the setpoint trick below) so the controller doesn't have to discover
 * the slope the slow way. See Åström & Murray "Feedback Systems" (ch. 11, free
 * PDF at https://fbsbook.org) and Åström & Hägglund "Advanced PID Control".
 */
typedef struct {
    /* gains — how hard each of the three reactions pushes (see §6.1 presets) */
    float kp;               /* proportional: bigger = snappier but overshoots */
    float ki;               /* integral: bigger = kills drift faster, more windup risk */
    float kd;               /* derivative: bigger = more damping, slower response */

    /* live controller state, rewritten every step */
    float setpoint;         /* the lean we're aiming for. On a slope this leans  *
                              * into the hill (= -SLOPE_FEED * slope) so the bot  *
                              * stays upright against gravity.                    */

    float integ;            /* the running pile-up of error the I term uses, kept *
                              * within ±WINDUP_MAX. Cleared on reset, PID toggle,  *
                              * and preset change.                                 */

    float prev_err;         /* last step's error, so we can see how fast it's     *
                              * changing for the D term.                           */

    /* the three pushes and their total — kept only so the panel can show them */
    float p;                /* kp * error */
    float i;                /* ki * integ */
    float d;                /* kd * (rate of change of error) */
    float out;              /* p + i + d, capped at ±MAX_FORCE; copied to the motor. */

    bool  on;               /* 'p' key. When off, the bot gets no push and tips   *
                              * over in about a second. Kept across a reset.       */
} PID;

/*
 * §6.2.4 Motion — where the bot is and how fast it's driving, kept apart from
 * how it's leaning (that's Pendulum). The cart just coasts forward along the
 * ground at drive_spd; balancing is somebody else's job. Position is stored in
 * pixels so it can index the terrain buffer directly; PIX_PER_M only converts
 * to metres for the HUD readouts.
 */
typedef struct {
    float world_x;          /* how far along the ground the bot is, in pixels.   *
                              * Only ever grows (it drives forward). Wraps through *
                              * the ring buffer invisibly as the camera follows.  */

    float drive_spd;        /* forward speed, pixels/sec, set by ↑/↓ within       *
                              * [0, 160]. 0 = parked, which is a pure balancing    *
                              * test.                                              */

    float spin_angle;       /* wheel spin angle — tracked but not drawn (wheels   *
                              * are plain 'O's). Kept so a future version could     *
                              * show spokes without rewiring.                      */

    float alpha;            /* ground slope under the wheels, rad; positive =     *
                              * uphill to the right. Refreshed each step and feeds *
                              * both the physics and the controller's target.      */

    float dist_m;           /* total distance travelled, metres — the HUD         *
                              * odometer. Reset on bot_reset().                    */
} Motion;

/*
 * §6.2.5 PhaseTrail — the last 240 (lean, lean-speed) samples, drawn as a
 * single curve (lean across, lean-speed up). The shape of that curve tells
 * you at a glance how the controller behaves: a tight inward spiral means
 * stable and well-damped, wide circles mean it's wobbly, a slow straight run
 * to the centre means sluggish, and a curve that drifts off-centre means
 * leftover steady error. It's a ring buffer — 240 samples is about 4 seconds,
 * long enough to see a full recovery but short enough that old history fades.
 * (Strogatz, "Nonlinear Dynamics and Chaos", ch. 5-6.)
 */
typedef struct {
    float theta[HIST_LEN];  /* lean samples, rad (parallel arrays — same index). */
    float omega[HIST_LEN];  /* lean-speed samples, rad/sec. */

    int   head;             /* where the next sample goes; newest is just before it. */
    int   fill;             /* how many slots are filled so far (caps at HIST_LEN),  *
                              * so the drawer skips empty ones right after a reset.   */
} PhaseTrail;

/*
 * §6.2.6 BotUI — the flags that exist only because a human is at the
 * keyboard: paused or not, which panel is showing, which preset is picked.
 * They have nothing to do with the physics, which is why they're grouped on
 * their own — and why a reset keeps them (pressing 'r' resets the simulation,
 * not your preferences). "fallen" lives here too because it's really a "show
 * the FALLEN banner" event flag the drawer reads, not a physics quantity.
 */
typedef struct {
    bool     paused;        /* spacebar — freezes the bot in place for a closer   *
                              * look or a screenshot. Cleared on reset.            */

    bool     fallen;        /* true once the bot tips past FALL_ANGLE; physics     *
                              * stops and the FALLEN banner shows. Only 'r' clears  *
                              * it.                                                 */

    ViewMode view;          /* which side panel is showing; 'm' cycles it. Kept    *
                              * across a reset.                                     */

    int      preset_idx;    /* which gain preset is active (see §6.1); 'g' cycles  *
                              * it. Kept across a reset.                            */
} BotUI;

/*
 * §6.2.7 Bot — the six pieces above bundled together. Reading any line of
 * physics now tells you which piece it touches: b->pend is the lean, b->cp the
 * physics math, b->pid the controller, b->mot the position, b->trail the phase
 * history, b->ui the keyboard flags. A good reading order is top to bottom —
 * what's leaning, the equations that move it, the controller that drives them,
 * where it all happens, then the display and input bits. One physics step
 * (bot_substep in §6.6) walks them in that same order.
 */
typedef struct {
    Pendulum   pend;        /* lean + lean-speed (§6.2.1) */
    CartPole   cp;          /* physics values this step (§6.2.2) */
    PID        pid;         /* the controller (§6.2.3) */
    Motion     mot;         /* position + slope (§6.2.4) */
    PhaseTrail trail;       /* phase-portrait history (§6.2.5) */
    BotUI      ui;          /* keyboard flags (§6.2.6) */
} Bot;

/* §6.3 PID controller — small helpers + pid_step */

/* Squeeze a value into [-limit, +limit]. Used for both the anti-windup cap
 * on the integral and the motor-force limit. */
static inline float clamp_symmetric(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/* How far the lean is from its target — the thing the controller works to
 * zero out. Positive means leaning more than asked, negative means less. */
static inline float pid_error_signal(float theta, float setpoint)
{
    return theta - setpoint;
}

/* Add this step's error to the running pile-up, then cap it so a long-standing
 * error can't keep dominating the output after things settle. */
static inline void pid_accumulate_with_antiwindup(PID *pid, float err, float dt)
{
    pid->integ = clamp_symmetric(pid->integ + err * dt, WINDUP_MAX);
}

/* How fast the error is changing, for the damping term. If the step is too
 * tiny to measure (below PID_DT_FLOOR) we skip it — losing one frame is fine. */
static inline float pid_error_derivative(PID *pid, float err, float dt)
{
    float de_dt = (dt > PID_DT_FLOOR) ? (err - pid->prev_err) / dt : 0.0f;
    pid->prev_err = err;
    return de_dt;
}

/* When the controller is off, every term and the motor output go to zero. */
static inline void pid_zero_outputs(PID *pid)
{
    pid->p = pid->i = pid->d = pid->out = 0.0f;
}

/*
 * pid_step — one tick of the controller: measure the lean error, blend the
 * three reactions (proportional + integral + derivative), cap the result at
 * the motor limit, and hand it to the physics as the motor force. Each step is
 * its own named helper so the body reads like the textbook formula.
 */
static void pid_step(Bot *b, float dt)
{
    PID *pid = &b->pid;

    /* (0) controller disengaged → no force, all terms zero */
    if (!pid->on) {
        pid_zero_outputs(pid);
        b->cp.F = 0.0f;
        return;
    }

    /* (1) error signal */
    float err = pid_error_signal(b->pend.theta, pid->setpoint);

    /* (2) integrate (with anti-windup) and differentiate the error */
    pid_accumulate_with_antiwindup(pid, err, dt);
    float de_dt = pid_error_derivative(pid, err, dt);

    /* (3) three-term sum — each term kept for the equations panel */
    pid->p   = pid->kp * err;
    pid->i   = pid->ki * pid->integ;
    pid->d   = pid->kd * de_dt;
    pid->out = clamp_symmetric(pid->p + pid->i + pid->d, MAX_FORCE);

    /* (4) publish the motor force to the cart-pole dynamics step */
    b->cp.F = pid->out;
}

/* §6.4 cart-pole physics — helpers + one step */

/*
 * How much "weight" the sideways motor force has to move. It's the cart plus
 * however much of the body is being dragged along sideways — least when the
 * body stands straight up, most when it lies flat.
 */
static inline float pole_effective_inertia(float theta_eff)
{
    float sin_th = sinf(theta_eff);
    return MASS_CART + MASS_POLE * sin_th * sin_th;
}

/*
 * How fast the wheels accelerate sideways: the motor push plus the push-back
 * the swinging body gives its own pivot, divided by that effective weight.
 */
static inline float cart_horizontal_accel(float F, float theta_eff,
                                          float omega, float M_eff)
{
    float sin_th = sinf(theta_eff);
    float cos_th = cosf(theta_eff);
    float pole_reaction = MASS_POLE * sin_th *
                          (PEND_LEN * omega * omega - GRAVITY * cos_th);
    return (F + pole_reaction) / M_eff;
}

/*
 * How fast the body's lean accelerates: gravity tipping it over, minus the
 * tilt caused by the wheels accelerating underneath it. That second part is
 * exactly the coupling the controller leans on — shove the base, the top tips
 * the other way, and the bot recovers.
 */
static inline float pole_angular_accel(float theta_eff, float x_ddot)
{
    return (GRAVITY * sinf(theta_eff) - x_ddot * cosf(theta_eff)) / PEND_LEN;
}

/* Step the lean and lean-speed forward by one dt. We update the speed first,
 * then the angle — that ordering keeps the phase-portrait spirals tidy even
 * when the step is a bit coarse. */
static inline void pendulum_euler_advance(Pendulum *pe, float theta_ddot,
                                          float dt)
{
    pe->omega += theta_ddot * dt;
    pe->theta += pe->omega  * dt;
}

/*
 * cart_pole_step — one step of the physics: work out the effective weight, the
 * sideways acceleration of the wheels, the angular acceleration of the body,
 * then nudge the lean and lean-speed forward. Each line is one helper above.
 */
static void cart_pole_step(Bot *b, float dt)
{
    CartPole *cp = &b->cp;
    Pendulum *pe = &b->pend;

    /* (1) effective inertia — how much "mass" the horizontal force sees */
    cp->M_eff      = pole_effective_inertia(cp->theta_eff);

    /* (2) cart acceleration — motor force + pole reaction over inertia  */
    cp->x_ddot     = cart_horizontal_accel(cp->F, cp->theta_eff,
                                           pe->omega, cp->M_eff);

    /* (3) pole acceleration — gravity tipping − cart's coupling         */
    cp->theta_ddot = pole_angular_accel(cp->theta_eff, cp->x_ddot);

    /* (4) integrate (ω, θ) forward by dt                                */
    pendulum_euler_advance(pe, cp->theta_ddot, dt);
}

/* §6.5 record one (lean, lean-speed) sample for the phase trail */

static void phase_push(Bot *b)
{
    PhaseTrail *tr = &b->trail;
    tr->theta[tr->head] = b->pend.theta;
    tr->omega[tr->head] = b->pend.omega;
    tr->head = (tr->head + 1) % HIST_LEN;
    if (tr->fill < HIST_LEN) tr->fill++;
}

/* §6.6 bot_substep — helpers + the six-step physics tick */

/* Roll the cart forward at its drive speed, and bump the odometer and the
 * wheel-spin angle along with it. */
static inline void motion_advance(Motion *mo, float dt)
{
    mo->world_x    += mo->drive_spd * dt;
    mo->dist_m     += mo->drive_spd * dt / PIX_PER_M;
    mo->spin_angle += (mo->drive_spd / WHEEL_R) * dt;
}

/* Which ground column the wheel sits over right now. */
static inline int wheel_world_column(const Motion *mo)
{
    return (int)(mo->world_x / (float)CELL_W);
}

/* From the current slope, set the two things the physics and controller need:
 * the lean gravity actually feels, and a lean target that leans into the hill.
 * Redone every step because the slope changes as the bot moves. */
static inline void apply_slope_feed_forward(Bot *b)
{
    float alpha = b->mot.alpha;
    b->cp.theta_eff = b->pend.theta + alpha;
    b->pid.setpoint = -alpha * SLOPE_FEED;
}

/* If the bot has tipped past the point of recovery, mark it fallen so the
 * physics stops and the banner shows. */
static inline void detect_fall(Bot *b)
{
    if (fabsf(b->cp.theta_eff) > FALL_ANGLE) b->ui.fallen = true;
}

/*
 * bot_substep — one physics step, in six plain stages. Called several times
 * per frame with a small dt so the simple integrator stays stable no matter
 * how fast or slow the frames come.
 */
static void bot_substep(Bot *b, const Terrain *t, float sub_dt)
{
    if (b->ui.fallen) return;

    /* (1) advance kinematics; sample slope at the new wheel position */
    motion_advance(&b->mot, sub_dt);
    b->mot.alpha = terrain_slope_at(t, wheel_world_column(&b->mot));

    /* (2) derive (θ_eff, setpoint) from the new slope                */
    apply_slope_feed_forward(b);

    /* (3) controller → motor force F                                 */
    pid_step(b, sub_dt);

    /* (4) cart-pole dynamics → new (θ, ω)                            */
    cart_pole_step(b, sub_dt);

    /* (5) record the (θ, ω) sample for the phase portrait            */
    phase_push(b);

    /* (6) fall test — sets ui.fallen if |θ_eff| > FALL_ANGLE         */
    detect_fall(b);
}

/* §6.7 init / reset / preset */

static void bot_apply_preset(Bot *b, int idx)
{
    const GainPreset *p = &PRESETS[idx];
    b->pid.kp    = p->kp;
    b->pid.ki    = p->ki;
    b->pid.kd    = p->kd;
    b->pid.integ = 0.0f;        /* start the new gains with a clean integral */
}

/* The handful of choices the user makes at runtime that should survive a
 * reset. We copy them out before wiping the bot and copy them back after, so
 * 'r' restarts the physics without throwing away the user's settings. */
typedef struct {
    bool     pid_on;
    float    drive_spd;
    ViewMode view;
    int      preset_idx;
} UserPrefs;

static UserPrefs grab_user_prefs(const Bot *b)
{
    return (UserPrefs){
        .pid_on     = b->pid.on,
        .drive_spd  = b->mot.drive_spd,
        .view       = b->ui.view,
        .preset_idx = b->ui.preset_idx,
    };
}

static void restore_user_prefs(Bot *b, UserPrefs p)
{
    b->pid.on        = p.pid_on;
    b->mot.drive_spd = p.drive_spd;
    b->ui.view       = p.view;
    b->ui.preset_idx = p.preset_idx;
}

/*
 * bot_reset — 'r' key. Restarts the physics from the boot state while keeping
 * the user's current settings. Same as bot_init except the settings come from
 * the existing bot instead of the defaults.
 */
static void bot_reset(Bot *b)
{
    UserPrefs prefs = grab_user_prefs(b);
    memset(b, 0, sizeof *b);
    restore_user_prefs(b, prefs);
    b->pend.theta = INITIAL_LEAN_RAD;
    bot_apply_preset(b, prefs.preset_idx);
}

/*
 * bot_init — first-time setup. Same as bot_reset, but the kept settings are
 * the §1 defaults rather than an existing bot's.
 */
static void bot_init(Bot *b)
{
    UserPrefs defaults = {
        .pid_on     = true,
        .drive_spd  = DRIVE_DEF,
        .view       = VIEW_TELEMETRY,
        .preset_idx = 0,
    };
    memset(b, 0, sizeof *b);
    restore_user_prefs(b, defaults);
    b->pend.theta = INITIAL_LEAN_RAD;
    bot_apply_preset(b, defaults.preset_idx);
}

/* ── §7 render — terrain, robot, side panels, HUD ────────────────────── */

/* §7.1 drawing helpers */

static inline int   px_to_cx (float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline bool  in_screen(int r, int c, int rows, int cols)
{
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

static void put_ch(int r, int c, chtype ch, attr_t a, int cp,
                   int rows, int cols)
{
    if (!in_screen(r, c, rows, cols)) return;
    attron (a | COLOR_PAIR(cp));
    mvaddch(r, c, ch);
    attroff(a | COLOR_PAIR(cp));
}

static void put_str(int r, int c, const char *s, attr_t a, int cp,
                    int rows, int cols)
{
    if (r < 0 || r >= rows) return;
    attron (a | COLOR_PAIR(cp));
    for (int i = 0; s[i] && c + i < cols; i++)
        mvaddch(r, c + i, (unsigned char)s[i]);
    attroff(a | COLOR_PAIR(cp));
}

/* Pick one character for a whole line based on its overall direction, so it
 * looks line-like without doing trig per cell. */
static chtype seg_glyph(int dr, int dc)
{
    if (dr == 0) return '-';
    if (dc == 0) return '|';
    return (dr * dc < 0) ? '/' : '\\';
}

/*
 * Draw a straight line of characters from one pixel point to another. It walks
 * cell by cell using Bresenham's classic integer trick — an error counter
 * decides at each step which way to move so the line stays straight. The glyph
 * is chosen once from the line's slope. (Bresenham, 1965.)
 */
static void draw_line(float x0, float y0, float x1, float y1,
                      attr_t a, int cp, int rows, int cols)
{
    /* (1) Endpoints in cell space */
    int r0 = px_to_cy(y0), c0 = px_to_cx(x0);
    int r1 = px_to_cy(y1), c1 = px_to_cx(x1);

    /* (2) Span and step direction along each axis */
    int dr = r1 - r0, dc = c1 - c0;
    int step_r = (r0 < r1) ? 1 : -1;
    int step_c = (c0 < c1) ? 1 : -1;
    int abs_dr = abs(dr), abs_dc = abs(dc);

    /* (3) Pick the glyph from the overall slope (once, not per cell) */
    chtype glyph = seg_glyph(dr, dc);

    /* (4) Walk the line, decision variable controls which axis steps */
    int err = abs_dr - abs_dc;
    int r = r0, c = c0;
    for (;;) {
        put_ch(r, c, glyph, a, cp, rows, cols);
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs_dc) { err -= abs_dc; r += step_r; }   /* step row */
        if (e2 <  abs_dr) { err += abs_dr; c += step_c; }   /* step col */
    }
}

/* Draw the empty gauge: a row of blanks with a '|' marking the centre. */
static void bar_paint_track(int r, int c, int W, int mid, int rows, int cols)
{
    attron(COLOR_PAIR(CP_DIM));
    for (int i = 0; i < W; i++) put_ch(r, c + i, ' ', 0, CP_DIM, rows, cols);
    put_ch(r, mid, '|', 0, CP_DIM, rows, cols);
    attroff(COLOR_PAIR(CP_DIM));
}

/* Fill the gauge: lay down '=' marks from the centre outward, going left for a
 * negative value and right for a positive one. */
static void bar_paint_fill(int r, int mid, int cells, int step, int cp,
                           int rows, int cols)
{
    attron(COLOR_PAIR(cp) | A_BOLD);
    for (int i = 0; i < cells; i++)
        put_ch(r, mid + step * (i + 1), '=', A_BOLD, cp, rows, cols);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* A centre-zero gauge for a value in [-1, +1]: draw the empty track, then fill
 * outward from the middle in the value's direction. */
static void draw_bar(int r, int c, int W, float v,
                     int cp_pos, int cp_neg, int rows, int cols)
{
    int half  = W / 2;
    int mid   = c + half;
    int cells = (int)(fabsf(v) * (float)half);
    if (cells > half) cells = half;

    bar_paint_track(r, c, W, mid, rows, cols);

    int step = (v >= 0.0f) ? +1 : -1;
    int cp   = (v >= 0.0f) ? cp_pos : cp_neg;
    bar_paint_fill(r, mid, cells, step, cp, rows, cols);
}

/* §7.2 render_terrain — sky, ground surface, rock fill */

/* True if this cell should hold a star. A quick hash of the position decides,
 * so stars stay put instead of flickering frame to frame. */
static bool is_star(int r, int c)
{
    unsigned h = (unsigned)(r * 7919 + c * 6271);
    h ^= h >> 13; h *= 0x45d9f3bU; h ^= h >> 17;
    return (h % 60) == 0;
}

/* Choose the surface character: '/' or '\\' where the ground is sloped enough,
 * '_' where it's roughly flat. */
static chtype surface_glyph(float dh)
{
    float thresh = (float)CELL_W * SLOPE_GLYPH_FRAC;
    if (dh >  thresh) return '/';
    if (dh < -thresh) return '\\';
    return '_';
}

/* Sky cell — a faint star where the hash says so, otherwise nothing. */
static void paint_sky_cell(int r, int sc, int rows, int cols)
{
    if (is_star(r, sc))
        put_ch(r, sc, '.', A_BOLD, CP_STAR, rows, cols);
}

/* The ground's top edge — slope-shaped, and red where it's steep. */
static void paint_surface_cell(int r, int sc, chtype sg, float dh,
                               int rows, int cols)
{
    bool steep = fabsf(dh) > (float)CELL_W * SLOPE_GLYPH_FRAC;
    int  cp    = steep ? CP_ROCK : CP_SURF;
    put_ch(r, sc, sg, A_BOLD, cp, rows, cols);
}

/* Grass just under the surface — alternating ':' and '.'. */
static void paint_grass_cell(int r, int sc, int rows, int cols)
{
    put_ch(r, sc, (sc % 3 == 0) ? ':' : '.', A_DIM, CP_SURF, rows, cols);
}

/* Rock filling everything below the grass — a sparse '#' texture. */
static void paint_rock_cell(int r, int sc, int rows, int cols)
{
    put_ch(r, sc, (sc % 2 == 0) ? '#' : ' ', A_DIM, CP_ROCK, rows, cols);
}

/* Screen row of the ground at a column, nudged inward so it never lands on the
 * HUD row at the top or the hint strip at the bottom. */
static int terrain_surface_row(const Terrain *t, int wc, int rows)
{
    int surf = px_to_cy(t->h[wc & TMASK]);
    if (surf < 1)        return 1;
    if (surf > rows - 2) return rows - 2;
    return surf;
}

/* How much the ground rises or falls from this column to the next — drives
 * both the surface glyph and its colour. */
static float terrain_dh_at(const Terrain *t, int wc)
{
    return t->h[(wc + 1) & TMASK] - t->h[wc & TMASK];
}

/*
 * render_terrain — paint the whole backdrop. For each on-screen column it
 * finds the matching ground column (the camera tracks the bot), works out
 * where the surface sits and how steep it is, then fills the column top to
 * bottom: sky, the surface line, grass, then rock.
 */
static void render_terrain(const Terrain *t, int bot_wc, int bot_sc,
                           int rows, int cols)
{
    for (int sc = 0; sc < cols; sc++) {
        /* (1) screen column → world column */
        int wc = bot_wc - bot_sc + sc;
        if (wc < 0) wc = 0;

        /* (2) surface row + local slope (drives glyph + colour) */
        int    surf = terrain_surface_row(t, wc, rows);
        float  dh   = terrain_dh_at(t, wc);
        chtype sg   = surface_glyph(dh);

        /* (3) classify each row vertically and paint accordingly */
        for (int r = 0; r < rows; r++) {
            if      (r <  surf)     paint_sky_cell    (r, sc, rows, cols);
            else if (r == surf)     paint_surface_cell(r, sc, sg, dh, rows, cols);
            else if (r == surf + 1) paint_grass_cell  (r, sc, rows, cols);
            else                    paint_rock_cell   (r, sc, rows, cols);
        }
    }
}

/* §7.3 render_bot — wheels, chassis, and the leaning body line */

/*
 * The four key points of the robot, in pixels: where the axle, the two wheels,
 * and the top of the body sit. We build them all here so the drawing code just
 * reads from this struct instead of repeating the geometry math.
 */
typedef struct {
    float ax_x, ax_y;   /* axle (the pivot), sitting just above the ground */
    float lw_x, lw_y;   /* left wheel,  tilted with the ground slope */
    float rw_x, rw_y;   /* right wheel, tilted with the ground slope */
    float top_x, top_y; /* top of the body, tilted by the lean */
} BotPixels;

/*
 * Work out those four points. The axle is pinned to a fixed screen column, a
 * little above the ground. The wheels swing around the axle by the GROUND
 * slope, so the chassis sits flat on the hill. The body top swings around the
 * axle by the LEAN instead — so the body tilts relative to gravity, not to the
 * chassis. Those two different tilts are the whole point of the demo: the
 * chassis follows the hill, the body fights to stay upright.
 */
static BotPixels compute_bot_skeleton(const Bot *b, float h_px, int bot_sc)
{
    BotPixels px;

    /* axle: fixed column, lifted one wheel-radius off the ground */
    px.ax_x = (float)(bot_sc * CELL_W);
    px.ax_y = h_px - WHEEL_R;

    /* wheels: spread out along the hill by tilting with the ground slope */
    float cos_a = cosf(b->mot.alpha), sin_a = sinf(b->mot.alpha);
    px.lw_x = px.ax_x - AXLE_HW * cos_a;
    px.lw_y = px.ax_y + AXLE_HW * sin_a;
    px.rw_x = px.ax_x + AXLE_HW * cos_a;
    px.rw_y = px.ax_y - AXLE_HW * sin_a;

    /* body top: tilted by the lean against gravity */
    float th = b->cp.theta_eff;
    px.top_x = px.ax_x + BODY_H * sinf(th);
    px.top_y = px.ax_y - BODY_H * cosf(th);

    return px;
}

/* Stamp an 'O' at each wheel. */
static void paint_wheels(const BotPixels *px, int rows, int cols)
{
    put_ch(px_to_cy(px->lw_y), px_to_cx(px->lw_x),
           'O', A_BOLD, CP_WHEEL, rows, cols);
    put_ch(px_to_cy(px->rw_y), px_to_cx(px->rw_x),
           'O', A_BOLD, CP_WHEEL, rows, cols);
}

/* A '*' at the top of the body that blinks bright/dim. It blinks with distance
 * travelled, not the clock, so it slows when the bot stops and ties the blink
 * to the scrolling ground. */
static void paint_beacon(const BotPixels *px, float dist_m,
                         int rows, int cols)
{
    int    pulse_phase = (int)(dist_m * BEACON_PULSE_HZ);
    attr_t pulse_attr  = (pulse_phase & 1) ? A_BOLD : A_DIM;
    put_ch(px_to_cy(px->top_y), px_to_cx(px->top_x),
           '*', pulse_attr, CP_BEACON, rows, cols);
}

/* The "+12.3°" lean label next to the axle — green normally, red once the lean
 * gets dangerous so it catches the eye. */
static void paint_lean_readout(const BotPixels *px, float theta_eff_rad)
{
    float deg = theta_eff_rad * (180.0f / (float)M_PI);
    int   cp  = (fabsf(deg) > THETA_WARN_DEG) ? CP_WARN : CP_GOOD;
    int   ar  = px_to_cy(px->ax_y);
    int   ac  = px_to_cx(px->ax_x);
    attron (COLOR_PAIR(cp));
    mvprintw(ar, ac - LEAN_LABEL_DC, "%+5.1f°", deg);
    attroff(COLOR_PAIR(cp));
}

/* The "FALLEN" banner in the middle of the screen, shown after the bot tips. */
static void paint_fallen_banner(int rows, int cols)
{
    int mr = rows / 2;
    int mc = cols / 2 - 9;
    if (mc < 0) mc = 0;
    attron (COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
    mvprintw(mr, mc, "  !! FALLEN !!  ");
    attroff(COLOR_PAIR(CP_WARN) | A_BOLD | A_BLINK);
    attron (COLOR_PAIR(CP_DIM));
    mvprintw(mr + 1, mc - 1, "g=preset  r=reset");
    attroff(COLOR_PAIR(CP_DIM));
}

/*
 * render_bot — draw the robot back to front: find its four key points, draw
 * the chassis line and the body line, stamp the wheels and the blinking
 * beacon, then add the lean label and (if it fell) the banner.
 */
static void render_bot(const Bot *b, const Terrain *t, int bot_sc,
                       int rows, int cols)
{
    int   wc   = (int)(b->mot.world_x / (float)CELL_W);
    float h_px = t->h[wc & TMASK];

    /* (1) four pixel-space joint positions */
    BotPixels px = compute_bot_skeleton(b, h_px, bot_sc);

    /* (2) chassis between wheels, body line from axle to top */
    draw_line(px.lw_x, px.lw_y, px.rw_x, px.rw_y,
              A_BOLD, CP_CHASSIS, rows, cols);
    draw_line(px.ax_x, px.ax_y, px.top_x, px.top_y,
              A_BOLD, CP_CHASSIS, rows, cols);

    /* (3) wheels and pulsing beacon */
    paint_wheels(&px, rows, cols);
    paint_beacon(&px, b->mot.dist_m, rows, cols);

    /* (4) overlays */
    paint_lean_readout(&px, b->cp.theta_eff);
    if (b->ui.fallen) paint_fallen_banner(rows, cols);
}

/* §7.4 panel helpers + the telemetry panel */

/* Write one text row of a panel; returns the next row down so callers can
 * stack rows without tracking line numbers. */
static int panel_text_row(int r, int c0, const char *s, attr_t a, int cp,
                          int rows, int cols)
{
    put_str(r, c0, s, a, cp, rows, cols);
    return r + 1;
}

/* A panel row with text on the left and a bar gauge on the right (the caller
 * formats the text; val_norm in [-1, +1] drives the bar). */
static int panel_text_bar_row(int r, int c0, const char *s, attr_t a, int cp,
                              float val_norm, int cp_pos, int cp_neg,
                              int rows, int cols)
{
    put_str(r, c0, s, a, cp, rows, cols);
    draw_bar(r, c0 + BAR_COL_OFFSET, BAR_WIDTH, val_norm,
             cp_pos, cp_neg, rows, cols);
    return r + 1;
}

/*
 * render_panel_telemetry — the live numbers panel. Four blocks: the bot's
 * state (lean, lean-speed, slope) each with a gauge; how fast and how far it's
 * driven; the three PID pushes plus their total motor force; and the current
 * gains and preset name.
 */
static void render_panel_telemetry(const Bot *b, int rows, int cols, int c0)
{
    const PID *pid = &b->pid;
    char buf[64];
    int  r = 0;

    r = panel_text_row(r, c0, " TELEMETRY              ",
                       A_BOLD, CP_VAL, rows, cols);

    /* the bot's state — lean, lean-speed, slope, each with a gauge */
    {
        float deg     = b->cp.theta_eff * (180.0f / (float)M_PI);
        int   cp_lean = fabsf(deg) > THETA_WARN_DEG ? CP_WARN : CP_GOOD;
        snprintf(buf, sizeof buf, " theta_eff %+6.2f° ", deg);
        r = panel_text_bar_row(r, c0, buf, A_BOLD, cp_lean,
                               deg / THETA_BAR_RANGE_DEG,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    }
    {
        snprintf(buf, sizeof buf, " omega    %+6.2f  ", b->pend.omega);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               b->pend.omega / OMEGA_BAR_RANGE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    }
    {
        /* slope bar swaps its colours so uphill reads warm */
        float sdeg     = b->mot.alpha * (180.0f / (float)M_PI);
        int   cp_slope = fabsf(sdeg) > SLOPE_WARN_DEG ? CP_WARN : CP_VAL;
        snprintf(buf, sizeof buf, " slope α  %+6.2f° ", sdeg);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, cp_slope,
                               sdeg / SLOPE_BAR_RANGE_DEG,
                               CP_BAR_NEG, CP_BAR_POS, rows, cols);
    }

    /* how fast and how far it's driven (plain text, no gauge) */
    snprintf(buf, sizeof buf, " spd  %+5.2f m/s     ",
             b->mot.drive_spd / PIX_PER_M);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
    snprintf(buf, sizeof buf, " dist %7.1f m       ", b->mot.dist_m);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

    /* the three PID pushes and their total */
    r++;   /* blank line */
    r = panel_text_row(r, c0, " PID OUTPUTS            ",
                       A_BOLD, CP_VAL, rows, cols);
    if (pid->on) {
        snprintf(buf, sizeof buf, " P  Kp·e   %+8.2f ", pid->p);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               pid->p / MAX_FORCE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);

        snprintf(buf, sizeof buf, " I  Ki·∫e  %+8.2f ", pid->i);
        r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

        snprintf(buf, sizeof buf, " D  Kd·θ̇   %+8.2f ", pid->d);
        r = panel_text_bar_row(r, c0, buf, A_NORMAL, CP_VAL,
                               pid->d / (MAX_FORCE * D_BAR_FRAC),
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);

        snprintf(buf, sizeof buf, " F total  %+8.2f N", pid->out);
        int cp_F = fabsf(pid->out) > MAX_FORCE * SAT_WARN_FRAC
                 ? CP_WARN : CP_GOOD;
        r = panel_text_bar_row(r, c0, buf, A_BOLD, cp_F,
                               pid->out / MAX_FORCE,
                               CP_BAR_POS, CP_BAR_NEG, rows, cols);
    } else {
        r = panel_text_row(r, c0, " PID  DISABLED          ",
                           A_BOLD, CP_WARN, rows, cols);
        r += 3;
    }

    /* the current gains and which preset is active */
    r++;
    snprintf(buf, sizeof buf, " Kp:%.0f Ki:%.2f Kd:%.0f  ",
             pid->kp, pid->ki, pid->kd);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_DIM, rows, cols);
    snprintf(buf, sizeof buf, " preset: %s ",
             PRESETS[b->ui.preset_idx].name);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
}

/* §7.5 the equations panel — the formulas with live numbers */

/* One caption-then-value pair: a dim label, then the live value below it. The
 * equations panel is just a stack of these. */
static int panel_equation_pair(int r, int c0,
                               const char *caption, const char *value,
                               attr_t a_val, int cp_val,
                               int rows, int cols)
{
    put_str(r,     c0 + 1, caption, A_DIM, CP_DIM, rows, cols);
    put_str(r + 1, c0,     value,   a_val, cp_val, rows, cols);
    return r + 2;
}

/*
 * render_panel_equations — the same formulas the physics and controller use,
 * shown with the current numbers plugged in. You can multiply Kp by the error
 * yourself and check it matches the P term on screen — so the panel doubles as
 * a live check on the controller.
 */
static void render_panel_equations(const Bot *b, int rows, int cols, int c0)
{
    const PID *pid = &b->pid;
    float err = pid_error_signal(b->pend.theta, pid->setpoint);
    char  buf[80];
    int   r = 0;

    r = panel_text_row(r, c0, " EQUATIONS               ",
                       A_BOLD, CP_VAL, rows, cols);

    /* what the physics worked out this step */
    snprintf(buf, sizeof buf, " θ_eff = θ + α = %+5.3f rad (%+5.1f°)",
             b->cp.theta_eff, b->cp.theta_eff * (180.0f / (float)M_PI));
    r = panel_equation_pair(r, c0, "Effective lean from grav:", buf,
                            A_BOLD, CP_VAL, rows, cols);

    snprintf(buf, sizeof buf, " ẍ = %+6.3f m/s²", b->cp.x_ddot);
    r = panel_equation_pair(r, c0, "Horizontal accel of base:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " θ̈ = %+6.3f rad/s²", b->cp.theta_ddot);
    r = panel_equation_pair(r, c0, "Pendulum angular accel:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    /* the controller, broken into target, error, and the three pushes */
    r++;
    r = panel_text_row(r, c0, " PID                    ",
                       A_BOLD, CP_VAL, rows, cols);

    snprintf(buf, sizeof buf, " θ_ref = -%.2f·α = %+5.3f rad",
             SLOPE_FEED, pid->setpoint);
    r = panel_equation_pair(r, c0, "Setpoint (slope feed):", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " e = θ − θ_ref = %+5.3f", err);
    r = panel_equation_pair(r, c0, "Error drives PID:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.0f · %+.4f = %+.2f", pid->kp, err, pid->p);
    r = panel_equation_pair(r, c0, "P — instant stiffness:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.2f · ∫e = %+.2f", pid->ki, pid->i);
    r = panel_equation_pair(r, c0, "I — drift removal:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    snprintf(buf, sizeof buf, " %.0f · θ̇ = %+.2f", pid->kd, pid->d);
    r = panel_equation_pair(r, c0, "D — damping:", buf,
                            A_NORMAL, CP_EQ, rows, cols);

    /* the total push to the motor, red if it's hitting the limit */
    int cp_F = fabsf(pid->out) > MAX_FORCE * SAT_WARN_FRAC
             ? CP_WARN : CP_GOOD;
    snprintf(buf, sizeof buf, " F = %+.2f N (max ±%.0f)",
             pid->out, MAX_FORCE);
    r = panel_text_row(r, c0, buf, A_BOLD, cp_F, rows, cols);
}

/* §7.6 the phase panel — the (lean vs lean-speed) picture */

/* Where the plot box sits and how to map values into it. */
typedef struct {
    int   r0, c0;           /* top-left corner of the box, in cells */
    int   mid_r, mid_c;     /* the centre, where lean and lean-speed are zero */
    float th_to_cells;      /* cells per radian of lean (across) */
    float om_to_cells;      /* cells per rad/sec of lean-speed (up/down) */
} PhasePlot;

/* Set up the plot from its top-left corner, scaling each axis so the trail
 * just reaches the edge of the box at its full range. */
static PhasePlot phase_configure(int plot_r0, int c0)
{
    PhasePlot p;
    p.r0    = plot_r0;
    p.c0    = c0;
    p.mid_r = plot_r0 + PHASE_PLOT_H / 2;
    p.mid_c = c0 + 1 + PHASE_PLOT_W / 2;
    p.th_to_cells = ((float)PHASE_PLOT_W * 0.5f) / PHASE_TH_RANGE;
    p.om_to_cells = ((float)PHASE_PLOT_H * 0.5f) / PHASE_OM_RANGE;
    return p;
}

/* Draw the cross-hair axes, the centre '+', and the four edge labels first, so
 * the trail and the current dot land on top. */
static void phase_draw_axes(const PhasePlot *p, int rows, int cols)
{
    for (int i = 0; i < PHASE_PLOT_H; i++)
        put_ch(p->r0 + i, p->mid_c, '|', A_DIM, CP_DIM, rows, cols);
    for (int i = 0; i < PHASE_PLOT_W; i++)
        put_ch(p->mid_r, p->c0 + 1 + i, '-', A_DIM, CP_DIM, rows, cols);
    put_ch(p->mid_r, p->mid_c, '+', A_DIM, CP_DIM, rows, cols);

    put_str(p->mid_r - 1, p->c0 + 1,                  "-θ", A_DIM, CP_DIM, rows, cols);
    put_str(p->mid_r - 1, p->c0 + PHASE_PLOT_W,       "+θ", A_DIM, CP_DIM, rows, cols);
    put_str(p->r0,                p->mid_c - 2,       "+ω", A_DIM, CP_DIM, rows, cols);
    put_str(p->r0 + PHASE_PLOT_H - 1, p->mid_c - 2,   "-ω", A_DIM, CP_DIM, rows, cols);
}

/* Turn a (lean, lean-speed) value into a cell inside the box. The up/down
 * sign flips because screen rows grow downward but we want lean-speed up. */
static inline int phase_plot_col(const PhasePlot *p, float theta)
{
    return p->mid_c + (int)(theta * p->th_to_cells);
}
static inline int phase_plot_row(const PhasePlot *p, float omega)
{
    return p->mid_r - (int)(omega * p->om_to_cells);
}

/* Colour a trail dot by how recent it is — newest red, middle yellow, oldest
 * grey — so you can read which way time runs in a still picture. */
static inline int trail_color_for_age(float age01)
{
    if (age01 > TRAIL_NEW_BAND) return CP_WARN;
    if (age01 > TRAIL_MID_BAND) return CP_VAL;
    return CP_DIM;
}

/* Draw the trail, oldest sample to newest. */
static void phase_draw_trail(const PhaseTrail *tr, const PhasePlot *p,
                             int rows, int cols)
{
    int n = tr->fill;
    for (int i = 0; i < n; i++) {
        int   idx = (tr->head - n + i + HIST_LEN) % HIST_LEN;
        int   pc  = phase_plot_col(p, tr->theta[idx]);
        int   pr  = phase_plot_row(p, tr->omega[idx]);
        float age = (float)i / (float)n;
        put_ch(pr, pc, '.', A_NORMAL, trail_color_for_age(age), rows, cols);
    }
}

/* Mark where the bot is right now with a bright '@' on top of the trail. */
static void phase_draw_current_point(const Pendulum *pe, const PhasePlot *p,
                                     int rows, int cols)
{
    int pc = phase_plot_col(p, pe->theta);
    int pr = phase_plot_row(p, pe->omega);
    put_ch(pr, pc, '@', A_BOLD, CP_BEACON, rows, cols);
}

/* Put a plain-words label + colour on what the trail is doing. Checked in
 * order: PID off, then too-wobbly / too-sluggish, then settled vs. still
 * correcting based on how big the error is. */
static const char *phase_classify_regime(const Bot *b, int *out_cp)
{
    if (!b->pid.on)                     { *out_cp = CP_WARN; return "PID OFF (free fall)";  }
    if (b->pid.kd < KD_UNDERDAMP_LIMIT) { *out_cp = CP_VAL;  return "UNDERDAMPED (no Kd)";  }
    if (b->pid.kd > KD_OVERDAMP_LIMIT)  { *out_cp = CP_VAL;  return "OVERDAMPED (high Kd)"; }

    float abs_err = fabsf(b->pend.theta - b->pid.setpoint);
    if (abs_err < CONVERGED_ERR_RAD)    { *out_cp = CP_GOOD; return "STABLE — converged";   }
    *out_cp = CP_VAL;                                        return "SETTLING — correcting";
}

/*
 * render_panel_phase — draw the lean-vs-lean-speed picture: header, axes, the
 * fading trail, the current dot, then a few lines of text below. The trail's
 * shape tells the story — a tight inward spiral is well-behaved, wide circles
 * mean wobbly, a slow straight run means sluggish, drifting off-centre means
 * leftover error.
 */
static void render_panel_phase(const Bot *b, int rows, int cols, int c0)
{
    char buf[64];
    int  r = 0;

    /* (1) header */
    r = panel_text_row(r, c0, " PHASE PORTRAIT         ",
                       A_BOLD, CP_VAL, rows, cols);
    r = panel_text_row(r, c0, " (θ vs ω)               ",
                       A_DIM,  CP_DIM, rows, cols);

    /* (2) plot geometry + axes */
    PhasePlot plot = phase_configure(r, c0);
    phase_draw_axes(&plot, rows, cols);

    /* (3) trail (oldest → newest, newest paints on top) */
    phase_draw_trail(&b->trail, &plot, rows, cols);

    /* (4) current point */
    phase_draw_current_point(&b->pend, &plot, rows, cols);

    /* (5) text panel below the plot */
    r = plot.r0 + PHASE_PLOT_H + 1;

    int regime_cp;
    const char *regime = phase_classify_regime(b, &regime_cp);
    r = panel_text_row(r, c0 + 1, regime, A_BOLD, regime_cp, rows, cols);

    snprintf(buf, sizeof buf, " θ=%+.3f ω=%+.3f",
             b->pend.theta, b->pend.omega);
    r = panel_text_row(r, c0, buf, A_NORMAL, CP_VAL, rows, cols);

    /* the current preset's note, tying the gains to the trail shape */
    r++;
    const GainPreset *pp = &PRESETS[b->ui.preset_idx];
    snprintf(buf, sizeof buf, " [%s]", pp->name);
    r = panel_text_row(r, c0, buf, A_BOLD, CP_VAL, rows, cols);
    r = panel_text_row(r, c0 + 1, pp->lesson_l1, A_DIM, CP_DIM, rows, cols);
    r = panel_text_row(r, c0 + 1, pp->lesson_l2, A_DIM, CP_DIM, rows, cols);
}

/* §7.7 render_hud — status line on top, key hints on the bottom */

/* Build the top status line: fps first, then the mode words, then the live
 * numbers, in that reading order. */
static void hud_format_status(const Bot *b, double fps, char *buf, size_t n)
{
    /* how far the lean is from tipping over — the safety margin */
    float margin_deg = (FALL_ANGLE - fabsf(b->cp.theta_eff))
                       * (180.0f / (float)M_PI);

    const char *mode = b->ui.paused ? "PAUSED "
                     : b->ui.fallen ? "FALLEN " : "running";
    const char *pid_flag = b->pid.on ? "" : "  NO-PID ";

    snprintf(buf, n,
             " %5.1f fps  %s%s  α=%+5.1f°  margin=%4.1f°  "
             "dist=%5.1fm  view=[%s]  preset=%s ",
             fps, mode, pid_flag,
             b->mot.alpha * (180.0f / (float)M_PI),
             margin_deg, b->mot.dist_m,
             VIEW_NAMES[b->ui.view], PRESETS[b->ui.preset_idx].name);
}

/* Paint the top status row, padding with spaces to the right edge so the
 * coloured band runs the full width. */
static void hud_paint_status_row(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom-row hint strip — lists every interactive key. */
static void hud_paint_hint_strip(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  ↑↓:speed  "
             "p:PID  m:view  g:preset  +/-:Kp ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* render_hud — the two-line UI frame: status row 0, hint row bottom. */
static void render_hud(const Bot *b, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];
    hud_format_status(b, fps, buf, sizeof buf);
    hud_paint_status_row(buf, cols);
    hud_paint_hint_strip(rows);
}

/* §7.8 scene_draw — the whole frame */

/* Left edge of the side panel, pulled in to column 1 if the terminal is too
 * narrow to fit a full panel. */
static int side_panel_origin_col(int cols)
{
    int c0 = cols - PANEL_W;
    return c0 < 1 ? 1 : c0;
}

/* Draw whichever side panel the current view mode selects. */
static void render_side_panel(const Bot *b, int rows, int cols)
{
    int c0 = side_panel_origin_col(cols);
    switch (b->ui.view) {
    case VIEW_TELEMETRY: render_panel_telemetry(b, rows, cols, c0); break;
    case VIEW_EQUATIONS: render_panel_equations(b, rows, cols, c0); break;
    case VIEW_PHASE:     render_panel_phase    (b, rows, cols, c0); break;
    default: break;
    }
}

/*
 * scene_draw — one whole frame, back to front: clear, the terrain backdrop,
 * the robot on top, the side panel, then the HUD frame.
 */
static void scene_draw(const Bot *b, const Terrain *t, double fps,
                       int bot_sc, int rows, int cols)
{
    erase();

    int bot_wc = (int)(b->mot.world_x / (float)CELL_W);
    render_terrain  (t, bot_wc, bot_sc, rows, cols);   /* (2) */
    render_bot      (b, t,      bot_sc, rows, cols);   /* (3) */
    render_side_panel(b,                rows, cols);   /* (4) */
    render_hud      (b, fps,            rows, cols);   /* (5) */
}

/* ── §8 screen — ncurses setup and present ───────────────────────────── */

/*
 * Screen — the terminal's current size, the one place "how big is the window"
 * lives. Everything that draws reads its rows and cols from here. These only
 * change on startup and on a resize, never mid-frame, so a frame can treat the
 * size as fixed while it draws.
 */
typedef struct {
    int cols;               /* width in cells */
    int rows;               /* height in cells */
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

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 scene — frame timer, the whole-program state, the main loop ──── */

/*
 * FrameTimer — the loop's timekeeping. It does two jobs: measure how long each
 * frame took (so the physics can chop a long frame into small stable steps),
 * and work out a steady frames-per-second number for the HUD. The fps is
 * averaged over about half a second — a per-frame number jumps around far too
 * much to read. (The sub-stepped loop follows Fiedler's "Fix Your Timestep!".)
 */
typedef struct {
    int64_t last_ns;        /* timestamp at the start of the previous frame;     *
                              * this frame's dt is now minus this. Reset after a  *
                              * resize so the next frame isn't a huge jump.       */

    int64_t fps_accum_ns;   /* total frame time piled up since the last fps       *
                              * update; once it passes half a second we refresh.  */

    int     fps_frames;     /* how many frames went into that pile-up. */

    double  fps_display;    /* the fps number the HUD shows, refreshed about     *
                              * twice a second so it stays readable.             */
} FrameTimer;

/*
 * Scene — one struct holding everything the program keeps alive: the noise,
 * the terrain, the bot, the screen size, the frame timer, the camera column,
 * and two flags. There's one global instance because signal handlers can't be
 * passed arguments and need somewhere to set "please quit" and "please resize";
 * everything else gets a pointer into this struct, so each function's inputs
 * are visible at the call and the global is only ever touched by the signal
 * handlers and main(). It's set up in dependency order (see main): noise, then
 * the screen, then the terrain that needs both, then the bot.
 */
typedef struct {
    Noise                 noise;       /* the seeded noise; Scene owns the only one. */

    Terrain               terrain;     /* the ground; borrows the noise above. */

    Bot                   bot;         /* the simulation (see §6.2.7). */

    Screen                screen;      /* terminal size, kept current on resize. */

    FrameTimer            timer;       /* frame timing + fps. */

    int                   bot_sc;      /* the screen column the bot is drawn at — the *
                                         * camera follows by pinning it here. Set to    *
                                         * a third of the way across, redone on resize. */

    volatile sig_atomic_t running;     /* loop keeps going while this is set; cleared  *
                                         * by Ctrl-C/kill or the quit keys. The volatile *
                                         * type is what makes it safe to touch from a    *
                                         * signal handler.                              */

    volatile sig_atomic_t need_resize; /* a resize signal sets this; the loop notices  *
                                         * at the top and re-reads the new size.        */
} Scene;

static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Squeeze a value into [lo, hi] — keeps the keyboard nudges below to one line. */
static inline float clamp_range(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Turn the controller on or off, clearing the integral so it doesn't come back
 * with a stale pile-up. */
static void key_pid_toggle(Bot *b)
{
    b->pid.on    = !b->pid.on;
    b->pid.integ = 0.0f;
}

/* Step to the next side-panel view. */
static void key_view_cycle(Bot *b)
{
    b->ui.view = (ViewMode)((b->ui.view + 1) % VIEW_COUNT);
}

/* Step to the next gain preset and load it. */
static void key_preset_cycle(Bot *b)
{
    b->ui.preset_idx = (b->ui.preset_idx + 1) % N_PRESETS;
    bot_apply_preset(b, b->ui.preset_idx);
}

/* Restart the bot and build fresh terrain. */
static void key_reset_simulation(Scene *scene)
{
    bot_reset(&scene->bot);
    terrain_init(&scene->terrain, &scene->noise,
                 scene->screen.rows, scene->screen.cols);
}

/* Nudge drive speed up or down within its limits. */
static void key_drive_speed_nudge(Bot *b, float delta)
{
    b->mot.drive_spd = clamp_range(b->mot.drive_spd + delta,
                                   DRIVE_MIN, DRIVE_MAX);
}

/* Nudge Kp up or down within its limits. */
static void key_kp_nudge(Bot *b, float delta)
{
    b->pid.kp = clamp_range(b->pid.kp + delta, 0.0f, KP_MAX_USER);
}

/* Send one keystroke to its action — the switch reads like the keymap. */
static void scene_handle_key(Scene *scene, int ch)
{
    Bot *b = &scene->bot;
    switch (ch) {
    case 'q': case 'Q': case 27:  scene->running = 0;                    break;
    case ' ':                     b->ui.paused = !b->ui.paused;          break;
    case 'p': case 'P':           key_pid_toggle(b);                     break;
    case 'r': case 'R':           key_reset_simulation(scene);           break;
    case 'm': case 'M':           key_view_cycle(b);                     break;
    case 'g': case 'G':           key_preset_cycle(b);                   break;
    case KEY_UP:                  key_drive_speed_nudge(b, +DRIVE_STEP); break;
    case KEY_DOWN:                key_drive_speed_nudge(b, -DRIVE_STEP); break;
    case '+': case '=':           key_kp_nudge(b, +KP_NUDGE);            break;
    case '-':                     key_kp_nudge(b, -KP_NUDGE);            break;
    default:                                                             break;
    }
}

/* Catch everything up to the new terminal size after a resize. */
static void scene_handle_resize(Scene *scene)
{
    screen_resize(&scene->screen);
    scene->bot_sc        = scene->screen.cols / 3;
    scene->terrain.rows  = scene->screen.rows;
    scene->need_resize   = 0;
    scene->timer.last_ns = clock_ns();   /* so the next frame isn't a huge jump */
}

/* Work out how long this frame took, capped so one slow frame can't make the
 * physics take a wild jump; also hands back the raw time for the fps counter. */
static float frame_measure_dt(FrameTimer *tm, int64_t now_ns,
                              int64_t *out_dt_ns)
{
    int64_t dt_ns = now_ns - tm->last_ns;
    tm->last_ns   = now_ns;
    *out_dt_ns    = dt_ns;

    float dt = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;
    return dt;
}

/* Add this frame to the fps counter, refreshing the displayed number about
 * twice a second so it stays readable. */
static void frame_tick_fps(FrameTimer *tm, int64_t dt_ns)
{
    tm->fps_accum_ns += dt_ns;
    tm->fps_frames++;
    if (tm->fps_accum_ns >= NS_PER_SEC / 2) {
        tm->fps_display  = (double)tm->fps_frames * 1e9
                         / (double)tm->fps_accum_ns;
        tm->fps_accum_ns = 0;
        tm->fps_frames   = 0;
    }
}

/* Handle every keystroke waiting in the buffer. */
static void scene_drain_input(Scene *scene)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_key(scene, ch);
}

/* Make sure the ground a bit past the right edge is generated before we try to
 * draw it, so the renderer never reads an empty column. */
static void ensure_terrain_ahead(Scene *scene)
{
    int upto = (int)(scene->bot.mot.world_x / (float)CELL_W)
             + scene->screen.cols + 32;
    terrain_ensure(&scene->terrain, upto);
}

/* Run the physics for this frame, chopped into small enough steps to stay
 * stable: more steps for a longer frame, capped so a stalled frame can't loop
 * forever. */
static void scene_advance_physics(Scene *scene, float dt)
{
    int n_sub = (int)ceilf(dt / TICK_DT_TARGET);
    if (n_sub < 1)            n_sub = 1;
    if (n_sub > MAX_SUBSTEPS) n_sub = MAX_SUBSTEPS;

    float sub_dt = dt / (float)n_sub;
    for (int i = 0; i < n_sub; i++)
        bot_substep(&scene->bot, &scene->terrain, sub_dt);
}

/* Sleep off whatever's left of the frame's time budget so the frame rate stays
 * steady no matter how quickly this frame finished. */
static void frame_cap_to_target_fps(int64_t now_ns, int64_t tick_ns)
{
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(tick_ns - elapsed);
}

/*
 * main — the program's life story. Set up the signal handlers and the
 * subsystems in order (noise, screen, the terrain that needs both, the bot),
 * then loop every frame: handle a resize, measure time, read the keyboard,
 * generate ground ahead, run the physics unless paused or fallen, draw, and
 * sleep to hold the frame rate.
 */
int main(void)
{
    /* SETUP — signals + atexit */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* SETUP — subsystems in dependency order */
    Scene *scene = &g_scene;
    scene->running = 1;

    noise_init  (&scene->noise, (unsigned int)time(NULL));
    screen_init (&scene->screen);
    terrain_init(&scene->terrain, &scene->noise,
                 scene->screen.rows, scene->screen.cols);
    bot_init    (&scene->bot);

    scene->bot_sc        = scene->screen.cols / 3;
    scene->timer.last_ns = clock_ns();

    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    /* LOOP */
    while (scene->running) {
        /* (1) handle resize before reading anything else */
        if (scene->need_resize) scene_handle_resize(scene);

        /* (2) frame timing — dt for physics, raw dt_ns for fps */
        int64_t now_ns;
        int64_t dt_ns;
        now_ns = clock_ns();
        float dt = frame_measure_dt(&scene->timer, now_ns, &dt_ns);
        frame_tick_fps(&scene->timer, dt_ns);

        /* (3) drain queued keystrokes */
        scene_drain_input(scene);

        /* (4) generate terrain columns ahead of the camera */
        ensure_terrain_ahead(scene);

        /* (5) advance physics (paused/fallen → skip) */
        if (!scene->bot.ui.paused && !scene->bot.ui.fallen)
            scene_advance_physics(scene, dt);

        /* (6) draw + present */
        scene_draw(&scene->bot, &scene->terrain, scene->timer.fps_display,
                   scene->bot_sc, scene->screen.rows, scene->screen.cols);
        screen_present();

        /* (7) sleep the remainder of the frame budget */
        frame_cap_to_target_fps(now_ns, TICK_NS);
    }

    screen_free(&scene->screen);
    return 0;
}
