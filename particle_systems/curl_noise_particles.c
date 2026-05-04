/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_particles.c — particles advected by 2-D curl-noise field
 *
 * DEMO: A few hundred particles drift across the screen, but they
 *       don't fall, fly, or follow gravity — they're advected by a
 *       VELOCITY FIELD computed from the CURL of an fBm scalar
 *       noise field:  v = (∂N/∂y, −∂N/∂x).  By construction this
 *       field is DIVERGENCE-FREE — no point in space gains or
 *       loses particles, so the swarm never collapses to a sink
 *       and never bursts from a source. It just SWIRLS, like
 *       smoke in a draft. A short trail behind each particle
 *       traces the local streamline so the field's structure is
 *       visible at a glance.
 *
 *       Patterns:
 *         CALM         low-frequency noise → slow lazy eddies
 *         TURBULENT    multi-octave fBm → chaotic small swirls
 *         HURRICANE    low-freq noise + global rotation around centre
 *         WIND_TUNNEL  curl noise + horizontal wind bias (rightward drift)
 *
 *       This is the visual classic for fluid-feel particles WITHOUT
 *       solving Navier-Stokes. The Reynolds-paper "Curl Noise for
 *       Procedural Fluid Flow" (Bridson 2007) showed that taking the
 *       curl of any smooth scalar field gives an incompressible flow.
 *
 * Study alongside:
 *   fluid/navier_stokes.c   — actual fluid solver (more accurate,
 *                             much more code).
 *   particle_systems/vortex.c — also force-field driven, but with
 *                             a single central drain instead of a
 *                             distributed velocity field.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair velocity-magnitude ramp
 *   §4 noise     — Perlin/fBm scaffold
 *   §5 curl      — finite-difference curl from fBm
 *   §6 particle  — Particle struct + trail buffer
 *   §7 scene     — pool, advection tick, draw
 *   §8 screen    — ncurses init / draw / resize
 *   §9 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (new noise permutation, redistribute particles)
 *   n / N      next pattern   (CALM → TURBULENT → HURRICANE → WIND_TUNNEL)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/curl_noise_particles.c \
 *       -o curl_noise_particles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pure ADVECTION of particles through a velocity
 *                  field. The velocity field is the CURL of a smooth
 *                  scalar noise field — for 2-D this collapses to:
 *
 *                    N(x, y, t)   = fBm(x · k + t · drift_x,
 *                                       y · k + t · drift_y, octaves)
 *                    v_x(x, y, t) = +∂N/∂y
 *                    v_y(x, y, t) = −∂N/∂x
 *
 *                  Computed numerically with central differences
 *                  (h = 0.5 cells in physical space).
 *
 *                  Because curl·(∇N) = 0, the divergence of v is
 *                  identically zero — so the flow is incompressible.
 *                  Particles can never accumulate at a point or
 *                  vanish from one. This is what gives the
 *                  "fluid-feel" visual: no clumps, no holes, just
 *                  smooth flowing swirls.
 *
 *                  Each tick:
 *                    1. For every active particle: sample velocity
 *                       at its current position via 4 fBm evals
 *                       (centre-difference curl); advance position
 *                       by v · dt.
 *                    2. WRAP at screen edges (toroidal); particles
 *                       never die — they cycle forever.
 *                    3. Push current position into the particle's
 *                       trail ring buffer.
 *                    4. Render trail oldest-first (dim to bright)
 *                       with colour by current velocity magnitude:
 *                       slow particles dim, fast particles bright.
 *
 *                  Pattern variations layer extra terms on top of
 *                  the base curl-noise:
 *                    HURRICANE   adds a constant angular rotation
 *                                around the screen centre (broken
 *                                divergence-free, but visually adds
 *                                a dominant cyclone)
 *                    WIND_TUNNEL adds a constant +x velocity (also
 *                                breaks divergence-free, but
 *                                imperceptibly — wrap means
 *                                particles cycle right→left)
 *
 * Data-structure : Particle[MAX_PARTICLES] object pool with a
 *                  per-particle TRAIL ring buffer of last 4
 *                  positions. No malloc at runtime.
 *
 * Rendering      : ASCII only. Particle head: ramp slot picked from
 *                  velocity magnitude. Trail: 3 previous positions
 *                  rendered with progressively dimmer ramp slots.
 *
 * Performance    : Per-tick cost: N · 4 · OCTAVES Perlin lookups.
 *                  At N = 400, OCTAVES = 4: 6400 lookups/tick →
 *                  384 k/sec at 60 fps. Each Perlin lookup is ~30 ns
 *                  (8 hash + interpolation), so ~12 ms per second of
 *                  CPU. Comfortably real-time.
 *
 * References     :
 *   • Bridson, R., Houriham, J. & Nordenstam, M. (2007) — "Curl-Noise
 *     for Procedural Fluid Flow", *ACM TOG* 26(3) (SIGGRAPH).
 *     The paper that introduced this technique. It also shows how
 *     to extend to 3-D and how to enforce no-flow boundaries.
 *   • Wikipedia — [Stream function](https://en.wikipedia.org/wiki/Stream_function).
 *     The 2-D equivalent: any scalar function ψ defines a
 *     divergence-free velocity field via (∂ψ/∂y, −∂ψ/∂x). curl-
 *     noise is exactly this with ψ = fBm.
 *   • Iñigo Quílez — "Domain warping" / curl noise tutorials,
 *     https://iquilezles.org/articles/warp/.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't simulate fluid. Use noise to FAKE one. Take any smooth
 * 2-D scalar field N(x, y) — fBm works perfectly — and compute the
 * vector field (∂N/∂y, −∂N/∂x). That's a divergence-free velocity
 * field by mathematical identity (curl of gradient = 0, applied to
 * the orthogonal direction). Drop particles in it, advect them
 * forward in time. They flow like smoke.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a topographic map of hills and valleys. Now imagine
 * water on top — but instead of flowing downhill (gradient flow),
 * imagine it flowing along the CONTOUR LINES, with the higher
 * land on its left. That's curl flow. Streamlines never cross,
 * particles never converge, the field never "sources" anywhere.
 * Make the topography a smoothly-evolving fBm and you have an
 * eternal swirling wind.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SEED noise permutation (Perlin 256-element scrambled table).
 *
 *  2. SPAWN N particles at random positions across the screen.
 *
 *  3. EACH TICK, per particle (px, py):
 *     a. Sample fBm noise at 4 nearby points:
 *          N+x = fbm(px+h, py, t)
 *          N-x = fbm(px-h, py, t)
 *          N+y = fbm(px, py+h, t)
 *          N-y = fbm(px, py-h, t)
 *     b. Compute partial derivatives via central diff:
 *          ∂N/∂x ≈ (N+x − N-x) / (2h)
 *          ∂N/∂y ≈ (N+y − N-y) / (2h)
 *     c. Velocity:
 *          vx = +∂N/∂y · field_mag
 *          vy = −∂N/∂x · field_mag (with aspect correction)
 *     d. Pattern-specific additions:
 *          HURRICANE: add tangential rotation around centre
 *          WIND_TUNNEL: add constant +vx
 *     e. Advance: px += vx · dt; py += vy · dt
 *     f. Wrap at edges: px = (px + cols) mod cols; same for y
 *     g. Push (px, py) into trail ring buffer
 *
 *  4. RENDER trail (oldest dim → newest bright) + head, coloured
 *     by velocity magnitude.
 *
 *  5. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Curl of scalar (2-D stream function ψ):
 *    v = (∂ψ/∂y, -∂ψ/∂x)
 *    div v = ∂vx/∂x + ∂vy/∂y
 *          = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0
 *    → flow is INCOMPRESSIBLE
 *
 *  Time-shifted fBm (drives slow field evolution):
 *    ψ(x, y, t) = fbm(x·k + t·drift_x, y·k + t·drift_y, octaves)
 *
 *  Central-difference partial derivative:
 *    ∂N/∂x ≈ (N(x+h) − N(x−h)) / (2h)
 *
 *  Aspect-correct velocity (cells are 2× taller; sample noise in
 *  physical coords, convert v back to cell coords):
 *    py_phys = cy · ASPECT_Y
 *    vx_cell = vx_phys
 *    vy_cell = vy_phys / ASPECT_Y
 *
 *  Speed → ramp slot (visualisation):
 *    speed = √(vx_cell² + (vy_cell · ASPECT_Y)²)
 *    slot  = ⌊clamp(speed / SPEED_REF, 0, 1) · 7⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DIVERGENCE-FREENESS REQUIRES SMOOTH NOISE. If noise is C¹
 *    discontinuous (cubic-ramp value noise has C¹ kinks at lattice
 *    edges), the central-difference curl will have spurious sources
 *    and sinks at lattice boundaries. Use Perlin's quintic fade
 *    function `t³(6t² − 15t + 10)` (C² continuous) for smoother
 *    derivatives. We do.
 *
 *  • H TOO SMALL → noisy curl (numerical precision lost). H too
 *    LARGE → derivative too smoothed (loses local structure).
 *    H = 0.5 cells is a reasonable balance for fBm at this freq.
 *
 *  • OCTAVE COUNT vs PERFORMANCE. Each fBm sample costs O(octaves).
 *    Particle requires 4 fBm samples per tick. 4 octaves × 4 samples
 *    = 16 noise lookups per particle. With 400 particles, that's
 *    6.4k lookups/tick — comfortable.
 *
 *  • TIME EVOLUTION. We translate the noise input by `t · drift`
 *    each tick, so the field appears to scroll over time. Particles
 *    follow the new streamlines. For more "structural" change use
 *    3-D noise (sample at (x, y, t)); we use 2-D + drift for speed.
 *
 *  • WRAP. Particles wrap at the boundary because the noise field
 *    is conceptually infinite. Without wrap, particles would
 *    eventually all leak off the screen edges and the swarm would
 *    disappear.
 *
 *  • HURRICANE BREAKS DIV-FREE. Adding a global rotation around
 *    the centre is technically incompressible too (rotation
 *    preserves area), so we're fine. WIND_TUNNEL adds a constant
 *    vx; that's also incompressible (∂vx/∂x = 0).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Particles freeze in place. Resume: motion
 *    continues from where it stopped.
 *
 *  • CALM. Lazy big swirls — particles slowly orbit around invisible
 *    eddies. The noise structure is at low frequency so the eddies
 *    are SCREEN-SIZED.
 *
 *  • TURBULENT. Many small eddies. Particles get caught and ejected
 *    rapidly, the swarm forms transient streaks.
 *
 *  • HURRICANE. The whole field rotates around the screen centre
 *    while small eddies modulate the flow locally. Particles trace
 *    spirals as they orbit + drift.
 *
 *  • WIND_TUNNEL. Particles drift right (with eddy modulation).
 *    Watch a particle exit the right edge and re-enter from the
 *    left (toroidal wrap).
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a recognisably
 *    different fluid colour: TROPICAL (green/cyan), AURORA (cycling
 *    colours by speed), INFRARED (blue at slow → red at fast), etc.
 *
 *  • Reseed (`r`). The noise permutation regenerates and particles
 *    redistribute — a new flow field appears.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SPEED_MIN        =   1,
    SPEED_DEF        =   8,
    SPEED_MAX        =  64,

    MAX_PARTICLES    =  600,
    TRAIL_LEN        =    4,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = 8 speed-magnitude tints    */
    PAIR_SKY         =  11,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Curl finite-difference step (physical units). */
#define CURL_H            0.5f

/* Speed → ramp slot reference (cells/sec). With sqrt mapping below,
 * a particle at exactly SPEED_REF c/s renders in slot 7 (brightest).
 * Lower → slow particles look brighter. */
#define SPEED_REF        25.0f

/* Pattern enum. */
typedef enum {
    PATTERN_CALM        = 0,
    PATTERN_TURBULENT   = 1,
    PATTERN_HURRICANE   = 2,
    PATTERN_WIND_TUNNEL = 3,
    N_PATTERNS          = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_CALM:        return "CALM       ";
    case PATTERN_TURBULENT:   return "TURBULENT  ";
    case PATTERN_HURRICANE:   return "HURRICANE  ";
    case PATTERN_WIND_TUNNEL: return "WIND_TUNNEL";
    default:                  return "?          ";
    }
}

/*
 * PatternParams — controls the curl noise field:
 *
 *   target_count   : number of particles
 *   fbm_octaves    : noise octaves (1 = single sine, 4 = rich)
 *   fbm_freq       : base spatial frequency (1/cells)
 *   field_mag      : multiplier on curl velocity
 *   time_drift_x   : per-second translation of noise input in x
 *   time_drift_y   : per-second translation of noise input in y
 *   global_rot     : added angular velocity around centre (rad/sec)
 *   wind_bias      : added horizontal velocity (cells/sec)
 */
typedef struct {
    int   target_count;
    int   fbm_octaves;
    float fbm_freq;
    float field_mag;
    float time_drift_x;
    float time_drift_y;
    float global_rot;
    float wind_bias;
} PatternParams;

/*
 * field_mag values are large because the curl of fBm at frequency k
 * has magnitude ~k (Perlin gradients are roughly unit-magnitude per
 * unit input). At k = 0.025 (CALM) you need field_mag ≈ 800 to get
 * particles moving at meaningful screen speeds (~20 cells/sec). The
 * patterns with ROT or WIND additions could get away with smaller
 * field_mag because the additive term provides baseline motion.
 */
static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  cnt  oct   freq    mag     tdx    tdy    rot    wind */
    /* CALM         */ { 280,  2,  0.025f,  800.0f, 0.30f, 0.20f, 0.00f,  0.0f },
    /* TURBULENT    */ { 450,  4,  0.060f, 1100.0f, 0.80f, 0.50f, 0.00f,  0.0f },
    /* HURRICANE    */ { 380,  2,  0.020f,  500.0f, 0.30f, 0.20f, 0.40f,  0.0f },
    /* WIND_TUNNEL  */ { 380,  3,  0.045f,  600.0f, 0.50f, 0.30f, 0.00f, 18.0f },
};

/*
 * Themes — each is an 8-step gradient indexed by particle SPEED
 * (slow → fast). The visual encodes velocity magnitude as colour
 * temperature, making the flow field's structure visible at a glance.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* slow → fast */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name         ramp[0..7]  (slow → fast)                          sky */

    { "DEFAULT",   {  24,  31,  38,  45,  87, 117, 153, 195 },          234 },
    { "TROPICAL",  {  29,  35,  37,  44,  50,  86, 122, 159 },          234 },
    { "FIRE",      {  88, 124, 130, 166, 196, 208, 214, 226 },          233 },
    { "AURORA",    {  43,  79, 115, 121, 157, 195, 230, 231 },          234 },
    { "VIOLET",    {  53,  91, 134, 165, 207, 213, 219, 225 },          234 },
    { "ICE",       {  24,  31,  67, 110, 117, 153, 195, 231 },          235 },
    { "NEON",      { 199, 213, 207, 219, 225, 231, 195, 153 },          234 },
    { "COPPER",    { 130, 137, 173, 179, 215, 222, 229, 230 },          234 },
    { "MONO",      { 240, 243, 245, 247, 249, 251, 253, 255 },          232 },
    { "INFRARED",  {  17,  21,  39,  46, 154, 226, 208, 196 },          233 },
};

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

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_SKY, t->sky, -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_CYAN, -1);
        init_pair(PAIR_SKY, COLOR_BLACK, -1);
    }
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

/* ===================================================================== */
/* §4  noise — Perlin scaffold (copied inline per self-contained-file)   */
/* ===================================================================== */

static uint8_t perm[512];

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade_q(float t) { return t*t*t*(t*(t*6.f-15.f)+10.f); }
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* fBm with N octaves. Returns roughly [-0.5, 0.5]. */
static float fbm2(float x, float y, int octaves)
{
    float total = 0, amp = 1, freq = 1, max_amp = 0;
    for (int o = 0; o < octaves; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.0f;
}

/* ===================================================================== */
/* §5  curl — finite-difference curl of fBm                              */
/* ===================================================================== */

/*
 * curl_velocity_at — sample the curl-noise velocity field at the
 * given physical position.
 *
 * v = (+∂N/∂y, -∂N/∂x)
 *
 * Computed by 4 fBm samples (central differences with step CURL_H).
 * Plus pattern-specific additions: time-drift handled by caller (the
 * noise input includes time offset); global rotation and wind bias
 * applied here as additive velocity terms.
 *
 * Returns velocity in PHYSICAL coords (caller converts y back to cells).
 */
static void curl_velocity_at(float px_phys, float py_phys, float t,
                             const PatternParams *pp,
                             float screen_cx_phys, float screen_cy_phys,
                             float *out_vx_phys, float *out_vy_phys)
{
    float k = pp->fbm_freq;
    float tx = t * pp->time_drift_x;
    float ty = t * pp->time_drift_y;

    float n_xp = fbm2((px_phys + CURL_H) * k + tx, py_phys * k + ty,
                     pp->fbm_octaves);
    float n_xm = fbm2((px_phys - CURL_H) * k + tx, py_phys * k + ty,
                     pp->fbm_octaves);
    float n_yp = fbm2(px_phys * k + tx, (py_phys + CURL_H) * k + ty,
                     pp->fbm_octaves);
    float n_ym = fbm2(px_phys * k + tx, (py_phys - CURL_H) * k + ty,
                     pp->fbm_octaves);

    float dN_dx = (n_xp - n_xm) / (2.0f * CURL_H);
    float dN_dy = (n_yp - n_ym) / (2.0f * CURL_H);

    float vx =  dN_dy * pp->field_mag;
    float vy = -dN_dx * pp->field_mag;

    /* HURRICANE: global rotation around screen centre. */
    if (pp->global_rot > 0.0f) {
        float dx = px_phys - screen_cx_phys;
        float dy = py_phys - screen_cy_phys;
        vx += -dy * pp->global_rot;
        vy +=  dx * pp->global_rot;
    }
    /* WIND_TUNNEL: constant +x bias. */
    if (pp->wind_bias != 0.0f) {
        vx += pp->wind_bias;
    }

    *out_vx_phys = vx;
    *out_vy_phys = vy;
}

/* ===================================================================== */
/* §6  particle                                                           */
/* ===================================================================== */

typedef struct {
    float px, py;        /* current cell-space position                 */
    float trail_x[TRAIL_LEN];
    float trail_y[TRAIL_LEN];
    int   trail_head;
    int   trail_count;
    bool  active;
} Particle;

/* Cheap LCG. */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §7  scene — pool, advection tick, draw                                */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    float     time_accum;       /* drives noise time evolution         */

    Particle  particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s)
{
    for (int i = 0; i < MAX_PARTICLES; i++) s->particles[i].active = false;
}

static void scene_spawn_particle(Scene *s, int idx)
{
    Particle *p = &s->particles[idx];
    p->px = lcg_unit(&s->rng) * (float)s->cols;
    p->py = lcg_unit(&s->rng) * (float)(s->rows - 1);
    p->trail_head  = 0;
    p->trail_count = 0;
    /* Pre-fill trail with current position. */
    for (int k = 0; k < TRAIL_LEN; k++) {
        p->trail_x[k] = p->px;
        p->trail_y[k] = p->py;
    }
    p->active = true;
}

static void scene_populate_to_target(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_count;
    if (target > MAX_PARTICLES) target = MAX_PARTICLES;
    int active = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) active++;
    /* Spawn shortfall in first inactive slots. */
    int spawned = 0;
    for (int i = 0; i < MAX_PARTICLES && spawned < target - active; i++) {
        if (!s->particles[i].active) {
            scene_spawn_particle(s, i);
            spawned++;
        }
    }
    /* Deactivate surplus from end of pool. */
    if (active > target) {
        int to_remove = active - target;
        for (int i = MAX_PARTICLES - 1; i >= 0 && to_remove > 0; i--) {
            if (s->particles[i].active) {
                s->particles[i].active = false;
                to_remove--;
            }
        }
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_CALM;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time_accum      = 0.0f;
    perm_shuffle((int)(s->rng & 0xFFFF));
    scene_clear_particles(s);
    scene_populate_to_target(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    perm_shuffle((int)(s->rng & 0xFFFF));
    scene_clear_particles(s);
    scene_populate_to_target(s);
}

/*
 * scene_tick — sample curl-noise velocity at every particle and
 * advance positions. Wrap at screen edges (toroidal). Push trail.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->time_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    int rows_eff = s->rows - 1;
    float screen_cx_phys = (float)s->cols * 0.5f;
    float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Convert cell position → physical position for noise sampling. */
        float px_phys = p->px;
        float py_phys = p->py * ASPECT_Y;

        float vx_phys, vy_phys;
        curl_velocity_at(px_phys, py_phys, s->time_accum, pp,
                         screen_cx_phys, screen_cy_phys,
                         &vx_phys, &vy_phys);

        /* Convert physical velocity back to cell-space. */
        float vx_cell = vx_phys;
        float vy_cell = vy_phys / ASPECT_Y;

        p->px += vx_cell * dt;
        p->py += vy_cell * dt;

        /* Toroidal wrap. */
        while (p->px < 0.0f)              p->px += (float)s->cols;
        while (p->px >= (float)s->cols)   p->px -= (float)s->cols;
        while (p->py < 0.0f)              p->py += (float)rows_eff;
        while (p->py >= (float)rows_eff)  p->py -= (float)rows_eff;

        /* Push trail. */
        p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
        p->trail_x[p->trail_head] = p->px;
        p->trail_y[p->trail_head] = p->py;
        if (p->trail_count < TRAIL_LEN) p->trail_count++;
    }
}

/*
 * scene_draw — render trail (oldest dim → newest bright) + head with
 * speed-magnitude colour.
 */
static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;
    const PatternParams *pp = &pattern_params[s->current_pattern];

    float screen_cx_phys = (float)s->cols * 0.5f;
    float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Compute current speed for colour slot. */
        float vx_phys, vy_phys;
        curl_velocity_at(p->px, p->py * ASPECT_Y, s->time_accum, pp,
                         screen_cx_phys, screen_cy_phys,
                         &vx_phys, &vy_phys);
        float speed = sqrtf(vx_phys * vx_phys + vy_phys * vy_phys);
        /* sqrt mapping pushes low speeds into higher ramp slots
         * (a particle at 25% of SPEED_REF gets slot 50% of brightest
         * instead of 25%). Otherwise most particles cluster in slot 0
         * and disappear into the dark theme tints. */
        float f = speed / SPEED_REF;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        f = sqrtf(f);
        int head_slot = (int)(f * 7.999f);
        if (head_slot < 1) head_slot = 1;     /* always visible */
        if (head_slot > 7) head_slot = 7;

        /* Trail (oldest → newest); head overdraws last. */
        int n = p->trail_count;
        for (int k = 0; k < n; k++) {
            int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
            float tx = p->trail_x[idx];
            float ty = p->trail_y[idx];
            int ix = (int)(tx + 0.5f);
            int iy = (int)(ty + 0.5f);
            if (ix < 0 || ix >= s->cols) continue;
            if (iy < 0 || iy >= rows_eff) continue;

            int slot = head_slot - (n - 1 - k);
            if (slot < 0) slot = 0;
            char glyph;
            int  attr;
            if (k == n - 1) {
                /* Head */
                glyph = (head_slot >= 5) ? '*' : (head_slot >= 2) ? '+' : '.';
                attr  = (head_slot >= 6) ? A_BOLD
                      : (head_slot <= 1) ? A_DIM
                      :                    A_NORMAL;
            } else {
                glyph = (slot >= 4) ? '+' : (slot >= 1) ? '.' : '`';
                attr  = (slot <= 1) ? A_DIM : A_NORMAL;
            }
            int pair = PAIR_RAMP_BASE + slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static int scene_active_count(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) n++;
    return n;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);
    const PatternParams *pp = &pattern_params[s->current_pattern];

    const char *state_str = s->paused ? "PAUSED     " : pattern_name(s->current_pattern);

    char buf[210];
    snprintf(buf, sizeof buf,
             " CURL_NOISE   %s   theme:%-8s   N:%4d  oct:%d  freq:%.3f   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             active, pp->fbm_octaves, (double)pp->fbm_freq,
             fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_populate_to_target(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_populate_to_target(s);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
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

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
