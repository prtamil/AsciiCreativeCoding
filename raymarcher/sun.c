/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun.c — 3D Solar Simulation  (learner-annotated)
 *
 * Animated raymarched sun:
 *   - Noise-displaced sphere SDF → boiling lava surface
 *   - Domain-warped fBm → churning granulation texture
 *   - 8 solar flares: blast explosion → magnetic arch → decay
 *   - Exponential corona/atmosphere glow
 *   - Temperature-mapped 256-color palette
 *   - 4 switchable themes with complementary sun/flare colors
 *
 * Keys:  q/ESC quit  |  space pause  |  t theme  |  r reset
 * Build: gcc -std=c11 -O2 -Wall -Wextra raymarcher/sun.c -o sun -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config       — all tunable constants (start here to experiment)
 *   §2  clock        — monotonic timer + sleep
 *   §3  color/theme  — 256-color palette, 4 themes
 *   §4  vec3 math    — 3D vector operations
 *   §5  noise/fBm    — value noise, fBm, domain warping
 *   §6  SDF          — signed distance functions (sphere, capsule, smooth-union)
 *   §7  flares       — state machine: blast → peak → decay
 *   §8  simulation   — pure physics state (time + flares), no rendering
 *   §9  raymarcher   — sphere-trace rays, shade hits, accumulate corona
 *  §10  canvas       — pixel buffer + draw to screen
 *  §11  renderer     — owns canvas + theme, bridges sim → screen
 *  §12  screen       — ncurses terminal setup/teardown
 *  §13  app          — main loop: tick → render → draw
 */

/* ── HOW THIS WORKS ──────────────────────────────────────────────────────── *
 *
 *  PIPELINE (one frame):
 *
 *    sim_tick()    advance time, step each flare state machine
 *         ↓
 *    renderer_render()   for every terminal cell, cast one ray:
 *         ↓                 a. pixel (col, row) → 3D ray direction
 *    rm_cast()             b. sphere-march: step by SDF value each iteration
 *         ↓                c. hit  → sample fBm temperature → color pair
 *    Pixel buffer          d. miss → accumulate corona glow, check star hash
 *         ↓
 *    canvas_draw()   map each Pixel → ncurses char + color attribute
 *
 *  ── KEY TECHNIQUES ───────────────────────────────────────────────────────
 *
 *  Raymarching / Sphere Tracing
 *    Instead of solving ray–surface intersections analytically, we step
 *    along the ray by the SDF value at the current position. Because the
 *    SDF tells us "the nearest surface is at least this far away," we can
 *    never overshoot. Stop when SDF < RM_HIT_EPS.
 *    Reference: https://iquilezles.org/articles/raymarchingdf/
 *
 *  Signed Distance Function (SDF)
 *    f(p) = distance from point p to the nearest surface surface.
 *    Negative values mean we are inside the surface.
 *    Sun  = sphere SDF with fBm noise subtracted from the radius.
 *    Flare = capsule SDFs smooth-unioned (smin) into the sphere.
 *    Reference: https://iquilezles.org/articles/distfunctions/
 *
 *  fBm (Fractional Brownian Motion)
 *    Sum N octaves of value noise, each at 2× frequency and ½ amplitude.
 *    Low octaves set the large lumps; high octaves add fine detail.
 *    Domain warping: evaluate fBm(p + noise(p)) instead of fBm(p).
 *    The input offset creates swirling, hurricane-like patterns.
 *    Reference: https://iquilezles.org/articles/warp/
 *
 *  Smooth Minimum (smin)
 *    Blend two SDF values smoothly over a region of radius k.
 *    Without it, unioning sphere + capsule produces a sharp crease.
 *    Reference: https://iquilezles.org/articles/smin/
 *
 *  Cubic Bézier Arch (flares)
 *    Each flare is a tube swept along a cubic Bézier curve — the classic
 *    magnetic-field-line arch. Control points push out from each foot
 *    perpendicular to the sphere, so the tube emerges at a right angle
 *    (like real coronal magnetic loops).
 *    Reference: https://en.wikipedia.org/wiki/Bézier_curve
 *
 *  Rodrigues Rotation
 *    Rotate vector v around unit axis k by angle θ in one formula.
 *    Used to distribute flare feet and to orient the arch tangentially.
 *    Reference: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 *
 *  Fixed-Timestep Loop
 *    Physics runs at SIM_FPS ticks/sec regardless of render frame rate.
 *    An accumulator collects real elapsed time; we drain it in fixed steps.
 *    Flare timing stays consistent whether the terminal renders at 10 or 60 fps.
 *    Reference: https://gafferongames.com/post/fix_your_timestep/
 *
 * ─────────────────────────────────────────────────────────────────────────*/

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — all tunable constants                                     */
/*                                                                        */
/* Everything here is intentionally in one place. To experiment with      */
/* the simulation, change values here — no hunting through the code.      */
/* Each constant shows units and a context note so the value makes sense. */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 24,  /* physics ticks per second (independent of render fps) */
    SIM_FPS_MIN     = 10,
    SIM_FPS_MAX     = 60,
    HUD_COLS        = 52,  /* status line buffer width (chars)                     */
    FPS_UPDATE_MS   = 500, /* how often the fps counter refreshes (ms)             */
    N_FLARES        = 8,   /* total flare slots; 2–4 typically active at once      */
    N_THEMES        = 4,
};

/* ── Raymarcher ─────────────────────────────────────────────────────── */
/* Increase RM_MAX_STEPS for sharper detail at cost of frame rate.        */
#define RM_MAX_STEPS   72     /* max sphere-trace steps per ray                       */
#define RM_HIT_EPS     0.018f /* hit threshold (world units); smaller = sharper edges */
#define RM_MAX_DIST    18.0f  /* bail-out distance (world units)                      */
#define RM_NORM_EPS    0.004f /* finite-difference step for surface normal estimation */

/* ── Camera ─────────────────────────────────────────────────────────── */
#define CAM_Z          5.5f   /* camera distance from origin; sun sits at (0,0,0)     */
#define FOV_HALF_TAN   0.60f  /* tan(FOV/2); 0.60 ≈ 60° horizontal FOV               */
#define CELL_ASPECT    2.0f   /* terminal cell height/width ratio; corrects oval look */

/* ── Sun geometry ───────────────────────────────────────────────────── */
/* DISP_AMP / SUN_RADIUS ≈ 17%  — surface can bulge/sink ~17% of radius. */
#define SUN_RADIUS     1.30f  /* base sphere radius (world units)                      */
#define DISP_AMP       0.22f  /* fBm displacement amplitude                            */
#define DISP_FREQ      1.70f  /* noise frequency on the surface; higher = more detail  */
#define DISP_SPEED     0.28f  /* surface animation speed (world units / sec)           */
#define WARP_AMP       0.40f  /* domain warp offset strength                           */
#define WARP_FREQ      0.90f  /* domain warp spatial frequency                         */

/* ── Flare geometry ─────────────────────────────────────────────────── */
/*
 * Scale context: SUN_RADIUS = 1.30, so at FLARE_MAX_H = 1.75 a flare
 * apex reaches 1.30 + 1.75 = 3.05 world units from the origin — more
 * than twice the sun's radius. Reduce FLARE_MAX_H for smaller flares.
 */
#define FLARE_MAX_H    1.75f  /* max arch apex height above sphere surface             */
#define FLARE_SMIN_K   0.18f  /* smooth-union blend radius (world units)               */
#define FLARE_ARC_R    0.044f /* arc tube base radius ≈ 3.4% of sun radius             */
#define FLARE_ARC_SEGS 8      /* Bézier arc subdivisions; 8 gives a visually smooth arc */
#define BEZIER_CTRL    0.65f  /* control point push factor; 0 = straight, 1 = very arched */
#define FLARE_SPARK_N  10     /* number of spark projectiles per blast event            */
#define SPARK_REACH    1.50f  /* max spark travel distance from epicenter (world units) */
#define SPARK_R        0.045f /* spark sphere radius (world units)                      */
#define SPARK_CTR_BIAS 1.80f  /* how strongly sparks cluster toward center_dir          */
#define FLARE_FLOW_N   4      /* plasma flow particles per arc during PEAK phase        */
#define FLOW_R         0.028f /* flow particle sphere radius (world units)              */
#define FLARE_FLOW_SPD 1.50f  /* arc lengths a flow particle covers per cycle           */

/* ── Corona ─────────────────────────────────────────────────────────── */
#define CORONA_SCALE   3.8f   /* glow falloff steepness; higher = tighter, sharper halo */
#define CORONA_BRIGHT  0.55f  /* peak corona brightness weight                          */
#define CORONA_WEIGHT  0.04f  /* per-step accumulation weight                           */
#define CORONA_THRESH  0.22f  /* minimum value to render; suppresses stray dot pixels   */

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock — monotonic timer + sleep                                    */
/*                                                                        */
/* CLOCK_MONOTONIC never jumps backwards (unlike wall clock), so dt is    */
/* always a positive, sane number even when the system clock is adjusted. */
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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/*                                                                        */
/* ncurses color pairs map (foreground, background) to an index 1..N.     */
/* We allocate:                                                            */
/*   pairs  1..32  — sun surface temperature gradient (cool → hot)        */
/*   pair   33     — star color                                            */
/*   pair   34     — corona glow color                                     */
/*   pairs 35..42  — flare plasma gradient (edge → core)                  */
/*   pair   43     — HUD text (always bright white, theme-independent)     */
/*                                                                        */
/* Each theme defines two independent gradients: sun[] and flare[].       */
/* Sun and flare colors are complementary hues so they're visually        */
/* distinct even in dense scenes.                                         */
/* ===================================================================== */

#define N_TEMP_PAIRS  32   /* sun surface gradient steps                  */
#define CP_STAR       33
#define CP_CORONA     34
#define N_FLARE_PAIRS  8   /* flare plasma gradient steps                 */
#define CP_FLARE_BASE 35
#define CP_HUD        43   /* always bright white — theme-independent     */

typedef struct {
    const char *name;
    short sun[N_TEMP_PAIRS];     /* surface gradient: cool → hot           */
    short flare[N_FLARE_PAIRS];  /* flare gradient: edge → core            */
    short corona;                /* corona halo color                       */
    short star;                  /* star field color                        */
    /* 8-color terminal fallback */
    short sun8_lo;               /* cool-half sun color                    */
    short sun8_hi;               /* hot-half sun color                     */
    short flare8;                /* flare fallback color                   */
} Theme;

/*
 * 256-color palette reference: https://www.ditig.com/256-colors-cheat-sheet
 * Color pair numbers below are xterm-256color indices.
 */
static const Theme k_themes[N_THEMES] = {
    {
        /* Solar: warm orange-red sun, complementary electric cyan flares */
        "Solar",
        { 88, 124, 160, 160, 166, 196,
          202, 208, 208, 214, 214, 220,
          220, 221, 221, 222, 222, 223,
          226, 226, 227, 227, 228, 228,
          229, 229, 230, 230, 230, 255, 231, 15 },
        { 45, 51, 87, 123, 159, 195, 231, 15 },
        214, 238,
        COLOR_RED, COLOR_YELLOW, COLOR_CYAN,
    },
    {
        /* Green Sun: dark-green → lime sun, complementary hot-magenta flares */
        "Green Sun",
        { 22,  22,  28,  28,  34,  34,
          40,  40,  46,  46,  82,  82,
          118, 118, 154, 154, 155, 155,
          190, 190, 191, 191, 192, 192,
          193, 193, 194, 230, 230, 231, 231, 15 },
        { 125, 161, 197, 205, 213, 219, 225, 231 },
        82, 238,
        COLOR_GREEN, COLOR_WHITE, COLOR_MAGENTA,
    },
    {
        /* Blue Sun: navy → cyan sun, complementary hot-orange flares */
        "Blue Sun",
        { 17,  18,  19,  20,  21,  21,
          27,  27,  33,  33,  39,  39,
          45,  45,  51,  51,  87,  87,
          123, 123, 159, 159, 195, 195,
          195, 231, 231, 231, 231, 255, 231, 15 },
        { 130, 166, 202, 208, 214, 220, 226, 230 },
        39, 238,
        COLOR_BLUE, COLOR_CYAN, COLOR_YELLOW,
    },
    {
        /* Nova: purple → magenta sun, complementary electric-lime flares */
        "Nova",
        { 53,  54,  90,  90, 127, 128,
          164, 165, 201, 201, 207, 207,
          213, 213, 219, 219, 225, 225,
          231, 231, 231, 255, 255, 255,
          231, 231, 231, 231, 255, 255, 231, 15 },
        { 64, 70, 76, 118, 154, 190, 229, 231 },
        201, 240,
        COLOR_MAGENTA, COLOR_WHITE, COLOR_GREEN,
    },
};

/* Apply a theme by re-initializing all color pairs. ncurses allows this
 * at runtime, so themes switch instantly without reinitializing the terminal. */
static void color_apply_theme(int ti)
{
    const Theme *t = &k_themes[ti];
    if (COLORS >= 256) {
        for (int i = 0; i < N_TEMP_PAIRS; i++)
            init_pair((short)(i + 1), t->sun[i], -1);
        init_pair(CP_STAR,   t->star,   -1);
        init_pair(CP_CORONA, t->corona, -1);
        for (int i = 0; i < N_FLARE_PAIRS; i++)
            init_pair((short)(CP_FLARE_BASE + i), t->flare[i], -1);
        init_pair(CP_HUD, 231, -1);
    } else {
        /* 8-color fallback: split gradient at midpoint */
        for (int i = 0; i < N_TEMP_PAIRS; i++) {
            short fg = (i < 12) ? t->sun8_lo : (i < 24) ? t->sun8_hi : COLOR_WHITE;
            init_pair((short)(i + 1), fg, -1);
        }
        init_pair(CP_STAR,   COLOR_WHITE,  -1);
        init_pair(CP_CORONA, t->sun8_hi,   -1);
        for (int i = 0; i < N_FLARE_PAIRS; i++)
            init_pair((short)(CP_FLARE_BASE + i), t->flare8, -1);
        init_pair(CP_HUD, COLOR_WHITE, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();  /* -1 background = terminal default (transparent) */
    color_apply_theme(0);
}

/*
 * temp_attr() — map temperature t ∈ [0,1] (cool → hot) to a color pair.
 *
 * ASCII density ramp: sparse characters look "cool/dark", dense look "hot/bright".
 * Color pair index scales linearly across the N_TEMP_PAIRS gradient.
 */
static attr_t temp_attr(float t, char *ch_out)
{
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;

    static const char ramp[] = ".,;:ox%#$@";
    int ri = (int)(t * 9.f);
    if (ri < 0) ri = 0;
    if (ri > 9) ri = 9;
    *ch_out = ramp[ri];

    int pi = (int)(t * (float)(N_TEMP_PAIRS - 1));
    if (pi < 0) pi = 0;
    if (pi >= N_TEMP_PAIRS) pi = N_TEMP_PAIRS - 1;

    attr_t a = COLOR_PAIR(pi + 1);
    if (t > 0.75f) a |= A_BOLD;
    return a;
}

/*
 * flare_attr() — map flare intensity ft ∈ [0,1] (edge → core) to flare palette.
 *
 * ft=0: outer boundary of the arc tube (dim, theme base color)
 * ft=1: core of the arc (bright, hot plasma)
 */
static attr_t flare_attr(float ft, char *ch_out)
{
    if (ft < 0.f) ft = 0.f;
    if (ft > 1.f) ft = 1.f;

    static const char ramp[] = ".,;:o+#@";
    int ri = (int)(ft * 7.f);
    if (ri < 0) ri = 0;
    if (ri > 7) ri = 7;
    *ch_out = ramp[ri];

    int pi = (int)(ft * (float)(N_FLARE_PAIRS - 1));
    if (pi < 0) pi = 0;
    if (pi >= N_FLARE_PAIRS) pi = N_FLARE_PAIRS - 1;

    attr_t a = COLOR_PAIR(CP_FLARE_BASE + pi);
    if (ft > 0.5f) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  vec3 math                                                          */
/*                                                                        */
/* Minimal 3D vector library — only what this simulation needs.           */
/* Reference: any linear-algebra textbook, e.g. "3D Math Primer"         */
/* by Dunn & Parberry — https://gamemath.com/                             */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)  { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)              { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)              { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3mul(V3 a, float s)           { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3dot(V3 a, V3 b)              { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(V3 a)                    { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float l = v3len(a);
    return l > 1e-7f ? v3mul(a, 1.f / l) : v3(0, 0, 1);
}
static inline V3 v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}

/*
 * Rodrigues rotation: rotate v around unit axis k by angle theta.
 * Formula: v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
 * Used to place flare feet at arbitrary orientations on the sphere.
 * Reference: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 */
static inline V3 v3rotate(V3 v, V3 k, float theta)
{
    float c = cosf(theta), s = sinf(theta);
    return v3add(
        v3add(v3mul(v, c), v3mul(v3cross(k, v), s)),
        v3mul(k, v3dot(k, v) * (1.f - c))
    );
}

/* Utility scalar helpers */
static inline float clmpf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
static inline float lerpf(float a, float b, float t)   { return a + (b - a) * t; }

/*
 * smoothstep — cubic Hermite interpolation, result ∈ [0,1].
 * Returns 0 when x ≤ e0, 1 when x ≥ e1, smooth curve in between.
 * Reference: https://en.wikipedia.org/wiki/Smoothstep
 */
static inline float smoothstep(float e0, float e1, float x)
{
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

/* Y-axis rotation matrix — used by sdf_sun to apply sun orientation */
static inline V3 rot_y(V3 p, float a)
{
    float c = cosf(a), s = sinf(a);
    return v3(c*p.x + s*p.z, p.y, -s*p.x + c*p.z);
}

/* ===================================================================== */
/* §5  hash + value noise + fBm                                           */
/*                                                                        */
/* Builds organic, cloud-like texture from integer hash functions.        */
/*                                                                        */
/*   hash1()     integer → float ∈ [0,1]   (the random primitive)         */
/*   noise3()    smooth 3D value noise     (trilinear hash interpolation) */
/*   fbm3()      5-octave fBm              (layered noise)                */
/*   warped_fbm() domain-warped fBm        (swirling texture)             */
/*                                                                        */
/* Reference (fBm + domain warp): https://iquilezles.org/articles/warp/  */
/* ===================================================================== */

/*
 * hash1() — integer lattice point → pseudo-random float ∈ [0, 1].
 * Uses a multiply-xorshift pair (fast, good distribution).
 * Any change to the multipliers/seeds changes the noise pattern.
 */
static inline float hash1(int ix, int iy, int iz)
{
    unsigned u = (unsigned)(ix*127 + iy*311 + iz*743 + ix*iy*17 + iy*iz*29);
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    u = u ^ (u >> 16);
    return (float)(u & 0xFFFFFF) / (float)0x1000000;
}

/*
 * noise3() — smooth value noise via trilinear interpolation.
 *
 * 1. Find the 8 integer lattice corners around p.
 * 2. Hash each corner to a random value.
 * 3. Interpolate using quintic fade curves (C2-continuous → no visible seams).
 *
 * Quintic fade: t³(6t²-15t+10)  smoother than cubic t²(3-2t)
 * Reference: Perlin, K. "Improving Noise" (SIGGRAPH 2002)
 */
static float noise3(V3 p)
{
    int ix = (int)floorf(p.x), iy = (int)floorf(p.y), iz = (int)floorf(p.z);
    float fx = p.x - floorf(p.x);
    float fy = p.y - floorf(p.y);
    float fz = p.z - floorf(p.z);

    /* Quintic fade — C2-continuous, removes lattice artifacts */
    float ux = fx*fx*fx*(fx*(fx*6.f - 15.f) + 10.f);
    float uy = fy*fy*fy*(fy*(fy*6.f - 15.f) + 10.f);
    float uz = fz*fz*fz*(fz*(fz*6.f - 15.f) + 10.f);

    /* Hash all 8 corners of the unit cube */
    float v000 = hash1(ix,   iy,   iz  ), v100 = hash1(ix+1, iy,   iz  );
    float v010 = hash1(ix,   iy+1, iz  ), v110 = hash1(ix+1, iy+1, iz  );
    float v001 = hash1(ix,   iy,   iz+1), v101 = hash1(ix+1, iy,   iz+1);
    float v011 = hash1(ix,   iy+1, iz+1), v111 = hash1(ix+1, iy+1, iz+1);

    /* Trilinear interpolation across x, then y, then z */
    float x0 = lerpf(v000, v100, ux), x1 = lerpf(v010, v110, ux);
    float x2 = lerpf(v001, v101, ux), x3 = lerpf(v011, v111, ux);
    float y0 = lerpf(x0, x1, uy),     y1 = lerpf(x2, x3, uy);
    return lerpf(y0, y1, uz);
}

/*
 * fbm3() — 5-octave fractional Brownian motion.
 *
 * Each octave doubles the frequency and halves the amplitude.
 * The irrational frequency multiplier (2.1) avoids lattice alignment
 * between octaves, preventing visible grid artifacts.
 *
 * Returns ≈ [-0.5, 0.5] (zero-centered).
 */
static float fbm3(V3 p)
{
    float v = 0.f, amp = 0.5f, freq = 1.f;
    for (int i = 0; i < 5; i++) {
        v    += amp * (noise3(v3mul(p, freq)) - 0.5f);
        amp  *= 0.5f;
        freq *= 2.1f;  /* slightly irrational to prevent lattice alignment */
    }
    return v;  /* ≈ [-0.5, 0.5] */
}

/*
 * warped_fbm() — two-pass domain warping.
 *
 * Pass 1: sample a 3D noise field q at p → get a random offset vector
 * Pass 2: evaluate fBm at (p + q) instead of at p
 *
 * Effect: the fBm result appears to flow and swirl. The input point is
 * "warped" by the offset, creating hurricanes and vortices that pure fBm
 * can't produce. Used for the sun's churning granulation.
 */
static float warped_fbm(V3 p, float time)
{
    /* Animate the surface by drifting the sample point over time */
    V3 pa = v3add(p, v3(time * DISP_SPEED, time * DISP_SPEED * 0.7f, 0.f));

    /* Pass 1: sample noise field for warp offset (three independent channels) */
    V3 warp = v3(
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(1.7f, 9.2f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(8.3f, 2.8f, 0.f))),
        noise3(v3add(v3mul(pa, WARP_FREQ), v3(3.1f, 5.4f, 0.f)))
    );

    /* Pass 2: evaluate fBm at the warped position */
    V3 warped = v3add(pa, v3mul(warp, WARP_AMP));
    return fbm3(v3mul(warped, DISP_FREQ));
}

/* ===================================================================== */
/* §6  SDF — signed distance functions                                    */
/*                                                                        */
/* Each function returns the distance from point p to the nearest         */
/* point on the described surface. Negative = inside the surface.         */
/*                                                                        */
/* Primitives:                                                            */
/*   sdf_capsule()  — segment a→b, uniform radius r                       */
/*   smin()         — smooth union of two distances                       */
/*   sdf_sun()      — full scene SDF: sphere + fBm displacement + flares  */
/*                                                                        */
/* Reference: https://iquilezles.org/articles/distfunctions/             */
/* ===================================================================== */

/*
 * sdf_capsule() — distance from p to a rounded cylinder from a to b.
 *
 * Project p onto segment ab (clamped to [0,1]), then measure the
 * perpendicular distance and subtract radius r.
 */
static float sdf_capsule(V3 p, V3 a, V3 b, float r)
{
    V3    pa = v3sub(p, a);
    V3    ba = v3sub(b, a);
    float h  = clmpf(v3dot(pa, ba) / v3dot(ba, ba), 0.f, 1.f);
    return v3len(v3sub(pa, v3mul(ba, h))) - r;
}

/*
 * smin() — polynomial smooth minimum.
 *
 * Blends two SDF values a and b smoothly over a region of width k.
 * Where the surfaces are more than k apart: behaves like min(a,b).
 * Where they are within k: replaces the sharp crease with a smooth blend.
 * Reference: https://iquilezles.org/articles/smin/
 */
static float smin(float a, float b, float k)
{
    float h = fmaxf(k - fabsf(a - b), 0.f) / k;
    return fminf(a, b) - h*h*h * k * (1.f / 6.f);
}

/*
 * SunHit — result of a full scene SDF evaluation.
 *   dist    : signed distance to the nearest surface
 *   temp    : surface temperature ∈ [0,1] (drives color choice)
 *   flare_t : flare intensity ∈ [0,1] (0 = sun surface, >0 = flare plasma)
 */
typedef struct { float dist; float temp; float flare_t; } SunHit;

/* Forward declaration — flares are defined in §7 but used in §6 */
typedef struct Flare Flare;

static SunHit sdf_sun(V3 p, float time, float ry,
                      const Flare *flares, int n_flares);

/* ===================================================================== */
/* §7  flare lifecycle — state machine: BLAST → PEAK → DECAY             */
/*                                                                        */
/*  DORMANT  waiting for next eruption                                    */
/*    ↓  (dormant_t expires)                                              */
/*  BLAST    explosion: sparks burst from epicenter; arch begins forming  */
/*    ↓  (phase reaches 1.0)                                              */
/*  PEAK     full magnetic loop; plasma flows along Bézier arc           */
/*    ↓  (phase reaches 1.0)                                              */
/*  DECAY    arch collapses; height shrinks back to zero; embers fall     */
/*    ↓  (phase reaches 1.0)                                              */
/*  DEAD     → immediately transitions to DORMANT with random wait time  */
/*                                                                        */
/* The arch shape is a cubic Bézier curve with two feet on the sphere    */
/* surface and control points pushed outward so the tube emerges          */
/* perpendicular to the surface (like a real coronal magnetic loop).      */
/* ===================================================================== */

typedef enum { FL_DORMANT, FL_BLAST, FL_PEAK, FL_DECAY, FL_DEAD } FlareState;

struct Flare {
    FlareState state;
    float      phase;      /* progress ∈ [0,1] within the current state */
    float      height;     /* current arch apex height (world units)    */
    float      max_h;      /* randomised peak height for this eruption  */
    float      blast_dur;  /* duration of BLAST phase (seconds)         */
    float      peak_dur;   /* duration of PEAK  phase (seconds)         */
    float      decay_dur;  /* duration of DECAY phase (seconds)         */
    float      dormant_t;  /* remaining dormant wait time (seconds)     */

    V3  foot_a, foot_b;              /* arch foot unit directions on sphere */
    V3  center_dir;                  /* epicenter unit direction             */
    V3  spark_dir[FLARE_SPARK_N];    /* outward spark trajectory directions */
    float spark_spd[FLARE_SPARK_N];  /* per-spark speed multipliers         */
};

/* ── LCG pseudo-random (fast, seeded per-flare) ─────────────────────── */
/*
 * Linear Congruential Generator — simple PRNG for per-flare randomness.
 * Not cryptographic; just needs good enough distribution for visual variety.
 * Reference: https://en.wikipedia.org/wiki/Linear_congruential_generator
 */
static float lcg(unsigned *u)
{
    *u = *u * 1664525u + 1013904223u;
    return (float)((*u >> 8) & 0xFFFF) / 65535.f;
}

/*
 * flare_spawn() — initialise one flare from DORMANT/DEAD into BLAST.
 *
 * Randomises: height, phase durations, position on sphere, foot separation,
 * and spark directions. The seed is per-flare so each slot behaves differently.
 */
static void flare_spawn(Flare *f, unsigned seed)
{
    unsigned u = seed;
    float r1 = lcg(&u), r2 = lcg(&u), r3 = lcg(&u);
    float r4 = lcg(&u), r5 = lcg(&u);

    f->state     = FL_BLAST;
    f->phase     = 0.f;
    f->height    = 0.f;
    f->dormant_t = 0.f;
    f->blast_dur = 0.6f + r1 * 0.5f;
    f->peak_dur  = 1.2f + r2 * 2.0f;
    f->decay_dur = 0.8f + r3 * 1.0f;
    f->max_h     = FLARE_MAX_H * (0.45f + r4 * 0.55f);

    /* Spherical coordinates: avoid poles (lat ∈ [0.15π, 0.85π]) */
    float lon    = r1 * (float)(2.0 * M_PI);
    float lat    = (0.15f + r2 * 0.70f) * (float)M_PI;
    float sinlat = sinf(lat), coslat = cosf(lat);
    f->center_dir = v3norm(v3(sinlat * cosf(lon), coslat, sinlat * sinf(lon)));

    /* Tangent axis for foot separation (perpendicular to center_dir) */
    V3 up   = v3(0.f, 1.f, 0.f);
    V3 tang = (fabsf(f->center_dir.y) < 0.85f)
              ? v3norm(v3cross(f->center_dir, up))
              : v3norm(v3cross(f->center_dir, v3(1.f, 0.f, 0.f)));
    tang = v3rotate(tang, f->center_dir, r5 * (float)(2.0 * M_PI));

    float half_sep = 0.22f + r3 * 0.28f;
    f->foot_a = v3rotate(f->center_dir, tang,  half_sep);
    f->foot_b = v3rotate(f->center_dir, tang, -half_sep);

    /* Sparks: mostly outward from center, with random spread */
    for (int i = 0; i < FLARE_SPARK_N; i++) {
        float sr1 = lcg(&u), sr2 = lcg(&u), sr3 = lcg(&u);
        V3 spread = v3(sr1 - 0.5f, sr2 - 0.5f, sr3 - 0.5f);
        f->spark_dir[i] = v3norm(v3add(v3mul(f->center_dir, SPARK_CTR_BIAS), spread));
        f->spark_spd[i] = 0.6f + sr1 * 0.8f;
    }
}

/* Stagger initial flare phases so the screen isn't empty at startup */
static void flare_init_staggered(Flare *flares, int n)
{
    for (int i = 0; i < n; i++) {
        flare_spawn(&flares[i], (unsigned)(i * 7919 + 31337));
        float frac = (float)i / (float)n;
        flares[i].phase = frac;
        if (i % 3 == 0)      flares[i].state = FL_PEAK;
        else if (i % 3 == 1) flares[i].state = FL_DECAY;
    }
}

/*
 * flare_tick() — advance one flare by dt seconds.
 *
 * Each state updates phase, drives the height curve via smoothstep,
 * then checks for state transition. The seed is time-varying so each
 * eruption gets a fresh random configuration.
 */
static void flare_tick(Flare *f, float dt, unsigned seed)
{
    switch (f->state) {

    case FL_DORMANT:
        f->dormant_t -= dt;
        if (f->dormant_t <= 0.f) flare_spawn(f, seed);
        break;

    case FL_BLAST:
        f->phase  += dt / f->blast_dur;
        /* Arc grows from 0 to max_h: smoothstep eases in/out */
        f->height  = f->max_h * smoothstep(0.f, 1.f, f->phase);
        if (f->phase >= 1.f) {
            f->state  = FL_PEAK;
            f->phase  = 0.f;
            f->height = f->max_h;
        }
        break;

    case FL_PEAK:
        f->phase += dt / f->peak_dur;
        /* Gentle magnetic pulsing: ±4% height oscillation */
        f->height = f->max_h * (1.f + 0.04f * sinf(f->phase * (float)M_PI * 6.f));
        if (f->phase >= 1.f) {
            f->state = FL_DECAY;
            f->phase = 0.f;
        }
        break;

    case FL_DECAY:
        f->phase  += dt / f->decay_dur;
        f->height  = f->max_h * (1.f - smoothstep(0.f, 1.f, f->phase));
        if (f->phase >= 1.f) {
            f->state  = FL_DEAD;
            f->height = 0.f;
        }
        break;

    case FL_DEAD: {
        unsigned u = seed;
        f->state     = FL_DORMANT;
        f->dormant_t = 0.8f + lcg(&u) * 3.0f;
        f->height    = 0.f;
        break;
    }
    }
}

/* ── Bézier arc geometry ─────────────────────────────────────────────── */

/*
 * bezier_arc() — evaluate the cubic Bézier arch at parameter t ∈ [0,1].
 *
 * P0 = foot_a on sphere surface
 * P1 = foot_a pushed out by h × BEZIER_CTRL  (outward tangent control)
 * P2 = foot_b pushed out by h × BEZIER_CTRL
 * P3 = foot_b on sphere surface
 *
 * The outward push at P1/P2 makes the tube emerge perpendicular to the
 * sphere at the feet — like real solar magnetic field lines.
 */
static V3 bezier_arc(V3 fa, V3 fb, float h, float t)
{
    V3 p0 = v3mul(fa, SUN_RADIUS);
    V3 p1 = v3mul(fa, SUN_RADIUS + h * BEZIER_CTRL);
    V3 p2 = v3mul(fb, SUN_RADIUS + h * BEZIER_CTRL);
    V3 p3 = v3mul(fb, SUN_RADIUS);
    float mt = 1.f - t;
    return v3add(
        v3add(v3mul(p0, mt*mt*mt),    v3mul(p1, 3.f*mt*mt*t)),
        v3add(v3mul(p2, 3.f*mt*t*t), v3mul(p3, t*t*t))
    );
}

/*
 * arc_radius() — tube radius varies along t.
 *
 * Feet (t ≈ 0, 1): narrow → plasma constricts at footpoints
 * Apex (t ≈ 0.5):  widest → magnetic pressure bulge
 */
static float arc_radius(float t)
{
    float mid = sinf(t * (float)M_PI);  /* 0 at feet, 1 at apex */
    return FLARE_ARC_R * (0.60f + 0.40f * mid);
}

/*
 * FlareSDF — result of evaluating one flare's SDF.
 *   dist : distance to nearest flare surface
 *   ft   : flare intensity ∈ [0,1] at that surface point
 */
typedef struct { float dist; float ft; } FlareSDF;

/*
 * flare_sdf() — full SDF + intensity for one flare.
 *
 * Three overlapping contributions:
 *   1. Arc   — the main Bézier tube (all states while height > 0)
 *   2. Sparks — blast projectiles (BLAST state only)
 *   3. Flow  — plasma blobs cycling along the arc (PEAK + DECAY)
 *   4. Embers — falling fragments returning to footpoints (DECAY)
 *
 * smoothstep fade-in on all particle radii prevents sudden pop-in at t=0.
 */
static FlareSDF flare_sdf(V3 p, const Flare *f)
{
    FlareSDF out = { 1e9f, 0.f };
    if (f->state == FL_DORMANT) return out;

    /* Global fade out over the DECAY phase */
    float fade = (f->state == FL_DECAY) ? smoothstep(1.f, 0.f, f->phase) : 1.f;

    /* ── 1. Arc tube (Bézier, FLARE_ARC_SEGS capsule segments) ───── */
    if (f->height > 0.01f) {
        /* Fade the arc in at BLAST start so it doesn't pop in instantly */
        float arc_fadein = (f->state == FL_BLAST)
                           ? smoothstep(0.f, 0.25f, f->phase)
                           : 1.f;

        float best_arc = 1e9f;
        for (int seg = 0; seg < FLARE_ARC_SEGS; seg++) {
            float t0 = (float) seg      / (float)FLARE_ARC_SEGS;
            float t1 = (float)(seg + 1) / (float)FLARE_ARC_SEGS;
            V3    pa = bezier_arc(f->foot_a, f->foot_b, f->height, t0);
            V3    pb = bezier_arc(f->foot_a, f->foot_b, f->height, t1);
            float r  = arc_radius((t0 + t1) * 0.5f);
            float d  = sdf_capsule(p, pa, pb, r);
            if (d < best_arc) best_arc = d;
        }
        out.dist = best_arc;

        /* Only clearly-inside pixels get high ft — avoids border flicker */
        float arc_ft = smoothstep(FLARE_ARC_R * 0.8f, -FLARE_ARC_R * 0.5f, best_arc);
        out.ft = arc_ft * fade * arc_fadein;
    }

    /* ── 2. Blast sparks ─────────────────────────────────────────── */
    if (f->state == FL_BLAST) {
        V3    origin = v3mul(f->center_dir, SUN_RADIUS);
        float sp     = f->phase;
        /* Smooth fade-in/out prevents sudden appearance or disappearance */
        float fadein  = smoothstep(0.f,  0.08f, sp);
        float fadeout = smoothstep(1.f,  0.75f, sp);
        float sp_bright = fadein * fadeout;

        for (int i = 0; i < FLARE_SPARK_N; i++) {
            float reach = SPARK_REACH * f->spark_spd[i] * smoothstep(0.f, 1.f, sp);
            V3    spos  = v3add(origin, v3mul(f->spark_dir[i], reach));
            float sr    = SPARK_R * fadein * (1.f - sp * 0.70f);
            if (sr < 0.005f) continue;
            float sd = v3len(v3sub(p, spos)) - sr;
            if (sd < out.dist) {
                out.dist = sd;
                out.ft   = fmaxf(out.ft, sp_bright * (0.7f + f->spark_spd[i] * 0.3f));
            }
        }
    }

    /* ── 3. Flow particles (plasma cycling along arc during PEAK) ─── */
    if (f->state == FL_PEAK || f->state == FL_DECAY) {
        float flow_fadein = (f->state == FL_PEAK)
                            ? smoothstep(0.f, 0.15f, f->phase)
                            : 1.f;
        float flow_fade = fade * flow_fadein;

        for (int i = 0; i < FLARE_FLOW_N; i++) {
            float offset = (float)i / (float)FLARE_FLOW_N;
            float pt     = fmodf(f->phase * FLARE_FLOW_SPD + offset, 1.f);
            V3    fpos   = bezier_arc(f->foot_a, f->foot_b, f->height, pt);
            float fd     = v3len(v3sub(p, fpos)) - FLOW_R;
            if (fd < out.dist) {
                out.dist = fd;
                float heat = 0.6f + 0.4f * sinf(pt * (float)M_PI);
                out.ft = fmaxf(out.ft, heat * flow_fade);
            }
        }
    }

    /* ── 4. Decay embers — fall back toward footpoints ───────────── */
    if (f->state == FL_DECAY) {
        float ep      = f->phase;
        float em_fade = smoothstep(0.f, 0.08f, ep) * smoothstep(0.55f, 0.35f, ep);
        if (em_fade > 0.01f) {
            V3 fa = v3mul(f->foot_a, SUN_RADIUS);
            V3 fb = v3mul(f->foot_b, SUN_RADIUS);
            for (int side = 0; side < 2; side++) {
                V3    base = (side == 0) ? fa : fb;
                V3    dir  = v3norm(base);
                float off  = 0.28f * (1.f - ep * 1.8f);
                if (off < 0.f) off = 0.f;
                V3    epos = v3add(base, v3mul(dir, off));
                float er   = SPARK_R * 0.55f * em_fade;
                float ed   = v3len(v3sub(p, epos)) - er;
                if (ed < out.dist) {
                    out.dist = ed;
                    out.ft   = fmaxf(out.ft, 0.75f * em_fade);
                }
            }
        }
    }

    return out;
}

/* ── Full scene SDF ─────────────────────────────────────────────────── */

/*
 * sdf_sun() — evaluates the complete scene distance field at point p.
 *
 * Steps:
 *   1. Optionally rotate the query point (ry = sun orientation angle)
 *   2. Sphere SDF displaced by fBm noise → boiling surface
 *   3. Derive temperature from noise value (bulges = hotter)
 *   4. Smooth-union all active flare SDFs into the sphere
 */
static SunHit sdf_sun(V3 p, float time, float ry,
                      const Flare *flares, int n_flares)
{
    V3 lp = rot_y(p, -ry);  /* inverse rotation: rotate query point, not geometry */

    /* Displaced sphere: SDF(sphere) - fBm_noise × amplitude */
    float noise    = warped_fbm(lp, time);          /* ≈ [-0.5, 0.5] */
    float d_sun    = v3len(lp) - (SUN_RADIUS + noise * DISP_AMP);

    /* Temperature: positive noise (bulge) → hotter; negative (pit) → cooler */
    float raw_temp = smoothstep(-0.3f, 0.3f, noise + 0.1f);

    float d_total = d_sun;
    float best_ft = 0.f;

    for (int i = 0; i < n_flares; i++) {
        FlareSDF fs = flare_sdf(lp, &flares[i]);
        if (fs.dist >= 1e8f) continue;
        d_total = smin(d_total, fs.dist, FLARE_SMIN_K);
        if (fs.ft > best_ft) best_ft = fs.ft;
    }

    SunHit h;
    h.dist    = d_total;
    h.temp    = clmpf(raw_temp, 0.f, 1.f);
    h.flare_t = best_ft;
    return h;
}

/* ===================================================================== */
/* §8  simulation — pure physics state                                    */
/*                                                                        */
/* Simulation owns: elapsed time and all flare state machines.           */
/* It knows nothing about ncurses, pixels, or colors — that belongs in   */
/* the Renderer (§11). This separation makes each part easier to reason  */
/* about independently.                                                   */
/* ===================================================================== */

typedef struct {
    float time;              /* total elapsed simulation time (seconds) */
    bool  paused;
    Flare flares[N_FLARES];
} Simulation;

static void sim_init(Simulation *s)
{
    memset(s, 0, sizeof *s);
    flare_init_staggered(s->flares, N_FLARES);
}

/* Reset to initial conditions — keeps current theme */
static void sim_reset(Simulation *s)
{
    s->time = 0.f;
    flare_init_staggered(s->flares, N_FLARES);
}

/*
 * sim_tick() — advance physics by one fixed timestep dt (seconds).
 *
 * Called by the fixed-timestep accumulator in the main loop.
 * dt is always the same value (1/SIM_FPS), giving deterministic physics.
 */
static void sim_tick(Simulation *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
    for (int i = 0; i < N_FLARES; i++) {
        unsigned seed = (unsigned)(i * 6271 + (int)(s->time * 100.f));
        flare_tick(&s->flares[i], dt, seed);
    }
}

/* ===================================================================== */
/* §9  raymarcher — sphere-trace + shading                                */
/*                                                                        */
/* rm_cast() fires one ray from the camera through the given pixel.       */
/* It iterates the sphere-tracing loop and returns a Pixel with all       */
/* the information canvas_draw() needs to pick a color and character.     */
/* ===================================================================== */

/*
 * Pixel — result of tracing one ray.
 *
 *   hit     : true if the ray hit the sun (dist, temp, flare_t valid)
 *   dist    : ray distance to hit point (world units)
 *   temp    : surface temperature ∈ [0,1]
 *   flare_t : flare intensity ∈ [0,1]; 0 = sun surface, >0.55 = flare
 *   corona  : accumulated glow brightness ∈ [0,1] (only valid when !hit)
 *   is_star : ray aligned with a star hash bin (only valid when !hit)
 */
typedef struct {
    float dist;
    float temp;
    float flare_t;
    float corona;
    bool  is_star;
    bool  hit;
} Pixel;

/* Star field: hash the ray direction to a coarse bin; bright on match */
static bool star_at(V3 rd)
{
    int ix = (int)floorf(rd.x * 120.f);
    int iy = (int)floorf(rd.y * 120.f);
    int iz = (int)floorf(rd.z * 120.f);
    unsigned u = (unsigned)(ix*127 + iy*311 + iz*743);
    u = (u ^ (u >> 16)) * 0x45d9f3bU;
    return (u & 0xFFF) == 0;  /* ~1 in 4096 directions */
}

/*
 * rm_cast() — sphere-trace one ray and return the shaded Pixel.
 *
 * 1. Convert (col, row) → normalised screen coordinate (u, v) ∈ [-1, 1]
 * 2. Build ray origin (camera) and direction (through pixel)
 * 3. March: step by SDF value; accumulate corona on near-miss passes
 * 4. On hit: compute surface normal via finite differences; apply limb darkening
 * 5. On miss: return corona accumulation + star check
 */
static Pixel rm_cast(int px_col, int py_row, int cw, int ch,
                     float time, float ry,
                     const Flare *flares, int n_flares)
{
    /* Step 1: pixel → NDC [-1, 1], correct for cell aspect ratio */
    float u  =  ((float)px_col + 0.5f) / (float)cw * 2.f - 1.f;
    float v  = -((float)py_row + 0.5f) / (float)ch * 2.f + 1.f;
    float pa = ((float)ch * CELL_ASPECT) / (float)cw;  /* pixel aspect correction */

    /* Step 2: ray origin at camera, direction toward pixel */
    V3 ro = v3(0.f, 0.f, CAM_Z);
    V3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));

    Pixel pix        = {0};
    float corona_acc = 0.f;
    float t          = 0.1f;  /* start slightly in front of camera */

    /* Step 3: sphere-tracing loop */
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        V3     p  = v3add(ro, v3mul(rd, t));
        SunHit sh = sdf_sun(p, time, ry, flares, n_flares);

        /* Accumulate corona: exponential falloff from surface */
        float near = fmaxf(0.f, sh.dist);
        corona_acc += expf(-near * CORONA_SCALE) * CORONA_BRIGHT * CORONA_WEIGHT;

        if (sh.dist < RM_HIT_EPS) {
            /* Step 4: hit — compute normal and apply limb darkening */
            float e = RM_NORM_EPS;
            V3 n;
            n.x = sdf_sun(v3add(p, v3( e, 0, 0)), time, ry, flares, n_flares).dist
                - sdf_sun(v3add(p, v3(-e, 0, 0)), time, ry, flares, n_flares).dist;
            n.y = sdf_sun(v3add(p, v3(0,  e, 0)), time, ry, flares, n_flares).dist
                - sdf_sun(v3add(p, v3(0, -e, 0)), time, ry, flares, n_flares).dist;
            n.z = sdf_sun(v3add(p, v3(0, 0,  e)), time, ry, flares, n_flares).dist
                - sdf_sun(v3add(p, v3(0, 0, -e)), time, ry, flares, n_flares).dist;
            V3 N = v3norm(n);
            V3 V = v3norm(v3sub(ro, p));
            float ndv = fabsf(v3dot(N, V));
            /* Limb darkening: edges appear ~20% cooler (N·V near 0 at limb) */
            float temp = sh.temp * (0.80f + 0.20f * ndv);

            pix.hit     = true;
            pix.dist    = t;
            pix.temp    = clmpf(temp, 0.f, 1.f);
            pix.flare_t = sh.flare_t;
            pix.corona  = 0.f;
            return pix;
        }

        if (t > RM_MAX_DIST) break;
        /* Safe step: advance by SDF value (guaranteed not to overshoot surface) */
        t += fmaxf(sh.dist * 0.85f, 0.005f);
    }

    /* Step 5: miss — return corona and star */
    pix.hit     = false;
    pix.corona  = clmpf(corona_acc, 0.f, 1.f);
    pix.is_star = star_at(rd);
    return pix;
}

/* ===================================================================== */
/* §10  canvas — pixel buffer                                             */
/*                                                                        */
/* Canvas stores one Pixel per terminal cell. Rendering fills it via      */
/* rm_cast(); drawing maps each Pixel to an ncurses char + color attr.    */
/* ===================================================================== */

typedef struct {
    int    w, h;
    Pixel *pixels;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols; c->h = rows;
    c->pixels = calloc((size_t)(cols * rows), sizeof(Pixel));
}

static void canvas_free(Canvas *c)
{
    free(c->pixels); c->pixels = NULL; c->w = c->h = 0;
}

/* Fill canvas: one rm_cast() call per cell */
static void canvas_fill(Canvas *c, float time, float ry,
                         const Flare *flares, int n_flares)
{
    for (int py = 0; py < c->h; py++)
        for (int px = 0; px < c->w; px++)
            c->pixels[py*c->w + px] =
                rm_cast(px, py, c->w, c->h, time, ry, flares, n_flares);
}

/*
 * canvas_draw() — map each Pixel to an ncurses character + color attribute.
 *
 * Priority order (highest to lowest):
 *   1. Hit + high flare_t  → flare plasma color
 *   2. Hit                 → sun surface temperature color
 *   3. Corona glow > thresh → corona glow characters
 *   4. Star                → star dot
 *   5. Empty               → leave blank
 */
static void canvas_draw(const Canvas *c, int tcols, int trows)
{
    int ox = (tcols - c->w) / 2;
    int oy = (trows - c->h) / 2;

    for (int y = 0; y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            int tx = ox + x, ty = oy + y;
            if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows) continue;

            const Pixel *px = &c->pixels[y*c->w + x];
            char ch; attr_t a;

            if (px->hit) {
                if (px->flare_t > 0.55f) {
                    /* Clearly inside arc tube or spark — flare color */
                    a = flare_attr(px->flare_t, &ch);
                } else {
                    /* Sun surface — temperature-mapped color */
                    a = temp_attr(px->temp, &ch);
                }
                attron(a);
                mvaddch(ty, tx, (chtype)(unsigned char)ch);
                attroff(a);

            } else if (px->corona > CORONA_THRESH) {
                /* Corona halo: remap threshold→1.0 to avoid stray dots */
                float ct = (px->corona - CORONA_THRESH) / (1.f - CORONA_THRESH);
                static const char cglow[] = ",;:o+";
                int ci = (int)(ct * 4.9f); if (ci > 4) ci = 4;
                ch = cglow[ci];
                attr_t ca = COLOR_PAIR(CP_CORONA);
                if (ct > 0.4f) ca |= A_BOLD;
                attron(ca);
                mvaddch(ty, tx, (chtype)(unsigned char)ch);
                attroff(ca);

            } else if (px->is_star) {
                attron(COLOR_PAIR(CP_STAR) | A_BOLD);
                mvaddch(ty, tx, '.');
                attroff(COLOR_PAIR(CP_STAR) | A_BOLD);
            }
            /* else: background — leave blank */
        }
    }
}

/* ===================================================================== */
/* §11  renderer — bridges simulation → screen                            */
/*                                                                        */
/* Renderer owns: the pixel buffer (Canvas) and the active theme index.   */
/* It takes a const Simulation* as input and has no physics state of      */
/* its own — it only transforms simulation state into pixels.             */
/*                                                                        */
/*   renderer_render()  →  fills Canvas from Simulation                   */
/*   renderer_draw()    →  maps Canvas pixels to ncurses cells            */
/* ===================================================================== */

typedef struct {
    Canvas canvas;
    int    theme;
} Renderer;

static void renderer_init(Renderer *r, int cols, int rows)
{
    canvas_alloc(&r->canvas, cols, rows);
    r->theme = 0;
}

static void renderer_free(Renderer *r)
{
    canvas_free(&r->canvas);
}

static void renderer_resize(Renderer *r, int cols, int rows)
{
    canvas_free(&r->canvas);
    canvas_alloc(&r->canvas, cols, rows);
}

/* Fill the canvas using the current simulation state */
static void renderer_render(Renderer *r, const Simulation *sim)
{
    canvas_fill(&r->canvas, sim->time, 0.f, sim->flares, N_FLARES);
}

/* Draw canvas + HUD to the ncurses screen */
static void renderer_draw(const Renderer *r, int cols, int rows,
                          double fps, bool paused, int active_flares)
{
    canvas_draw(&r->canvas, cols, rows);

    /* Top HUD: fps / flares / theme / pause state */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, "%4.1f fps  flares:%d/%d  [%s]  %s",
             fps, active_flares, N_FLARES,
             k_themes[r->theme].name,
             paused ? "PAUSED" : "");
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom HUD: key reference */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(rows - 1, 0, " q:quit  spc:pause  t:theme  r:reset");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §12  screen — ncurses terminal setup / teardown                        */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);  /* non-blocking getch() */
    keypad(stdscr, TRUE);
    typeahead(-1);          /* disable typeahead detection (reduces input lag) */
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
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void)
{
    /* Two-phase refresh: stage then flush — one terminal write, no flicker */
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §13  app — main loop                                                   */
/*                                                                        */
/* Owns Simulation + Renderer + Screen. Wires together:                   */
/*   fixed-timestep accumulator → sim_tick()                              */
/*   renderer_render() + renderer_draw() each frame                       */
/*   input routing → Simulation or Renderer                               */
/*                                                                        */
/* Fixed-timestep pattern:                                                */
/*   accumulator += real_dt                                               */
/*   while (accumulator >= tick_duration):                                */
/*       sim_tick(fixed_dt)                                               */
/*       accumulator -= tick_duration                                     */
/* ===================================================================== */

typedef struct {
    Simulation sim;
    Renderer   rend;
    Screen     screen;
    int        sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int sig) { (void)sig; g_app.running = 0; }
static void on_resize(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup  (void)   { endwin(); }

static void app_do_resize(App *a)
{
    screen_resize(&a->screen);
    renderer_resize(&a->rend, a->screen.cols, a->screen.rows);
    a->need_resize = 0;
}

/* Count flares in an active state (blast / peak / decay) */
static int app_count_active(const App *a)
{
    int n = 0;
    for (int i = 0; i < N_FLARES; i++) {
        FlareState st = a->sim.flares[i].state;
        if (st == FL_BLAST || st == FL_PEAK || st == FL_DECAY) n++;
    }
    return n;
}

static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;  /* ESC or q → quit */
    case ' ':
        a->sim.paused = !a->sim.paused;
        break;
    case 't': case 'T':
        a->rend.theme = (a->rend.theme + 1) % N_THEMES;
        color_apply_theme(a->rend.theme);
        break;
    case 'r':
        sim_reset(&a->sim);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  on_exit);
    signal(SIGTERM, on_exit);
    signal(SIGWINCH, on_resize);

    App *app    = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    sim_init(&app->sim);
    renderer_init(&app->rend, app->screen.cols, app->screen.rows);

    /* FPS tracking */
    int64_t frame_t = clock_ns();
    int64_t sim_acc = 0;      /* physics accumulator (ns)         */
    int64_t fps_acc = 0;      /* fps counter accumulator (ns)     */
    int     fps_cnt = 0;
    double  fpsd    = 0.0;

    while (app->running) {
        if (app->need_resize) { app_do_resize(app); frame_t = clock_ns(); sim_acc = 0; }

        /* Measure real elapsed time; cap at 100ms to survive window resize stalls */
        int64_t now  = clock_ns();
        int64_t dt   = now - frame_t;
        frame_t      = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* Fixed-timestep accumulator: drain in SIM_FPS-sized steps */
        int64_t tick    = TICK_NS(app->sim_fps);
        float   tick_s  = (float)tick / (float)NS_PER_SEC;
        sim_acc        += dt;
        while (sim_acc >= tick) {
            sim_tick(&app->sim, tick_s);
            sim_acc -= tick;
        }

        /* Render: fill pixel buffer from sim state, then draw to terminal */
        renderer_render(&app->rend, &app->sim);

        erase();  /* mark virtual screen dirty (no write — avoids flicker) */
        renderer_draw(&app->rend, app->screen.cols, app->screen.rows,
                      fpsd, app->sim.paused, app_count_active(app));
        screen_present();

        /* Update FPS counter every FPS_UPDATE_MS ms */
        fps_cnt++; fps_acc += dt;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fpsd    = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
            fps_cnt = 0;
            fps_acc = 0;
        }

        /* Sleep to target ~24 fps render rate */
        int64_t elapsed = clock_ns() - frame_t + dt;
        clock_sleep_ns(NS_PER_SEC / 24 - elapsed);

        /* Process one key per frame (non-blocking) */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    renderer_free(&app->rend);
    screen_free(&app->screen);
    return 0;
}
