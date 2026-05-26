/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vector_field_arrows_showcase.c
 *   — Visualise 2-D vector fields f(x, y) → (u, v) as ASCII arrows.
 *     30 patterns spanning gradient fields, classical analytic fields,
 *     physics fields, divergence-free flows, dynamical-system phase
 *     portraits, and time-varying fields.
 *
 * DEMO: Every cell carries a 2-D vector v = (u, v).  The active
 *       pattern decides how to compute it; the renderer maps:
 *         direction → 8-way ASCII arrow glyph via atan2(vy, vx)
 *         magnitude → palette band brightness (0..3)
 *       Result: a field of arrows showing what a particle at each
 *       cell would feel.  Watch radial sources spray, vortices spin,
 *       saddle points push two ways at once, and ∇f always point
 *       uphill on a scalar field.
 *
 *       30 patterns in 6 tiers (cycle with n / p):
 *         Tier 1 GRADIENT  — ∇f for chosen scalars (paraboloid,
 *                            saddle, periodic, ripple, noise) — the
 *                            BRIDGE: scalar field → vector field
 *         Tier 2 ANALYTIC  — classical 2-D fields (radial source/
 *                            sink, rotation, shear, uniform diagonal)
 *         Tier 3 PHYSICS   — Coulomb point charge, electric dipole,
 *                            current-wire magnetic, gravity (1/r²),
 *                            quadrupole
 *         Tier 4 SOLENOID  — divergence-free / incompressible flows
 *                            (curl noise, stream function vortex
 *                            grid, vortex pair, Poiseuille channel,
 *                            noisy uniform)
 *         Tier 5 DYNAMICS  — 2-D ODE phase portraits (stable node,
 *                            stable spiral, Hopf limit cycle, Van der
 *                            Pol, nonlinear pendulum)
 *         Tier 6 ANIMATED  — time-varying (rotating dipole,
 *                            travelling wave, breathing radial,
 *                            orbiting vortex, drifting curl noise)
 *
 * Study alongside:
 *   ./curl_noise_vector_field.c     — specific curl-noise demo.  This
 *       file includes CURL_NOISE as one of 30; the dedicated demo
 *       goes deeper on that single technique.
 *   ./flow_field_particles.c        — particles riding a vector field.
 *       This file draws the FIELD itself; that one draws what flows on it.
 *   ./magnetic_fields.c             — physics magnetic field with a
 *       different visualisation strategy (lines, not per-cell arrows).
 *   ./perin_noise_flow_showcase.c   — Perlin-gradient flow.
 *
 * Section map:
 *   §1 config   — grid, patterns, glyph constants, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 vector   — noise primitives, scalar/gradient/curl helpers
 *   §6 arrow    — 8-direction ASCII arrow picker + magnitude band
 *   §7 patterns — 30 vector field visualisations + dispatch table
 *   §8 scene    — Field (per-cell arrow buffer) + Scene state + tick
 *   §9 screen   — ASCII render; HUD with pattern/tier/theme readout
 *   §10 app     — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume animation
 *   r          reset (new noise seed)
 *   n / N      next pattern   (p / P previous)
 *   t / T      next / previous theme
 *   + / =      faster animation drift (× 2)
 *   -          slower animation drift (/ 2)
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra vector_field_arrows_showcase.c \
 *       -o vector_field_arrows -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-cell direct vector field sampling.  Each cell
 *                  computes a (vx, vy) from the active pattern's
 *                  formula — analytic closed-form, scalar gradient
 *                  via finite difference, curl of a potential, or
 *                  ODE phase-space velocity.  Direction is binned to
 *                  one of 8 ASCII glyphs via atan2(vy, vx); magnitude
 *                  is quantised to 4 palette bands.
 *
 * Data-structure : Per-cell render output buffers (glow, band, glyph)
 *                  on a Field struct.  No persistent vector buffer —
 *                  patterns compute (vx, vy) on the fly and emit the
 *                  arrow glyph + band directly.  A small NoiseField
 *                  holds the seed used by Tier 1 GRAD_NOISE and Tier
 *                  4 CURL_NOISE (re-rolled on r).
 *
 * Rendering      : ASCII-only.  8 directional glyphs:
 *                      east  → '>'    west  → '<'
 *                      north → '^'    south → 'v'
 *                      NE / SW → '/'  (slope-ambiguous: sign carried
 *                                       by flow context)
 *                      NW / SE → '\'  (same caveat)
 *                  Magnitude below ARROW_DEAD_ZONE renders as '.'
 *                  in band 0 — marks field zeros and equilibrium
 *                  points where direction is meaningless.  The bin
 *                  formula is exact: bin = ((round(atan2/(π/4))) + 8) & 7.
 *
 * Performance    : O(W · H) per frame.  atan2 + sqrt per cell ≈ 50 ns
 *                  on modern hardware → 0.6 ms for an 11K-cell grid;
 *                  comfortable at 60 Hz.  Tier 1 GRAD_NOISE and Tier 4
 *                  CURL_NOISE add 4-6 noise samples per cell for the
 *                  finite-difference derivative, still well under budget.
 *
 * References     : VECTOR FIELD VISUALISATION
 *                  • Helman, J. & Hesselink, L. (1989) — "Representation
 *                    and Display of Vector Field Topology in Fluid Flow
 *                    Data Sets", IEEE Computer 22(8).  Foundational —
 *                    classifies the kinds of critical points (sources,
 *                    sinks, saddles, centres, spirals) that Tiers 2-5
 *                    of this file enumerate by example.
 *                  • Quilez, I. — "2D distance functions" and
 *                    "Curl noise":
 *                    https://iquilezles.org/articles/curlnoise/
 *                  • Bridson, R., Hourihan, J., Nordenstam, M. (2007) —
 *                    "Curl-Noise for Procedural Fluid Flow", SIGGRAPH'07.
 *                    Source for Tier 4 CURL_NOISE — divergence-free
 *                    flows from a noise potential via the curl operator.
 *
 *                  DYNAMICAL SYSTEMS (Tier 5)
 *                  • Strogatz, S. (1994) — "Nonlinear Dynamics and
 *                    Chaos".  Standard reference for the phase
 *                    portraits drawn in Tier 5 (stable node, spiral,
 *                    Hopf bifurcation, Van der Pol, pendulum).
 *                  • Van der Pol, B. (1926) — "On 'relaxation
 *                    oscillations'", Philosophical Magazine.  Original
 *                    paper for the limit-cycle equation in Tier 5.
 *
 *                  ASCII RENDERING
 *                  • Bourke, P. — "Character representation of grey
 *                    scale images":
 *                    http://paulbourke.net/dataformats/asciiart/
 *
 *                  COMPARE IN PROJECT
 *                  • ./curl_noise_vector_field.c — dedicated curl-noise
 *                    demo; one Tier 4 pattern here is its 30-pattern
 *                    cousin.
 *                  • ./flow_field_particles.c — particles ON a vector
 *                    field; this file shows the field ITSELF.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A vector field assigns a 2-D arrow (direction + magnitude) to every
 * point in the plane.  We sample on a grid and draw each arrow as a
 * single ASCII character whose orientation matches the direction.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine releasing a fleet of tiny tokens, one per cell.  The field
 * tells each token which way to drift and how hard to push.  Freeze
 * the moment of release and ink an arrow showing each token's
 * instantaneous velocity.  Where the field has a SINK, all arrows
 * point inward; a SOURCE, outward; a SADDLE, half push one way and
 * half the other; a CENTRE, arrows curl around it.  The arrows ARE
 * the field — they are not particles in motion.
 *
 * COORDINATE CONVENTION
 * ─────────────────────
 * Screen Y grows DOWN (ncurses convention).  All patterns compute
 * vectors in SCREEN coordinates: (vx > 0, vy > 0) means rightward +
 * DOWNWARD on screen.  Textbook physics/math formulas usually assume
 * Y-up; when porting, NEGATE vy.  Tier 5 phase-portrait patterns
 * document this explicitly per-pattern.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Pattern computes (vx, vy) for cell (x, y), optionally using
 *     time t (Tier 6) and the NoiseField seed (Tier 1.5 / 4.1).
 *  2. Compute magnitude |v| = √(vx² + vy²).
 *  3. Normalise |v| to [0, 1] via a pattern-specific scale
 *     (typically  |v|/(|v|+s)  or  tanh(|v|/s) ).
 *  4. If |v|_norm < ARROW_DEAD_ZONE: draw '.' in band 0 (this is a
 *     field zero — direction is meaningless).
 *  5. Else: bin = (round(atan2(vy, vx) / (π/4)) + 8) mod 8;
 *           glyph = ARROW_GLYPHS[bin];
 *           band  = quartile of |v|_norm.
 *  6. Renderer paints (glyph, band) at the cell.
 *
 * KEY FORMULAS
 * ────────────
 *  Bin index           : bin = (round(atan2(vy, vx) / (π/4)) + 8) & 7
 *  Gradient (central
 *  finite difference,
 *  h = 1)              : ∂f/∂x ≈ (f(x+1, y) - f(x-1, y)) / 2
 *                        ∂f/∂y ≈ (f(x, y+1) - f(x, y-1)) / 2
 *  2-D curl of φ       : v = (∂φ/∂y, -∂φ/∂x)
 *                        (divergence-free by construction)
 *  Inverse-square      : v = -k · (dx, dy) / (dx² + dy²)^1.5
 *  Magnitude saturator : |v|_norm = |v| / (|v| + s_scale)
 *                        — bounded in [0, 1), avoids ∞ near singularities
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Singularities (point charges exactly at the cell location): add
 *    ε to denominator squared so r² + ε > 0.
 *  • Tiny |v|: atan2 angle is meaningless — DEAD_ZONE gives '.'.
 *  • Slope glyphs '/' and '\' don't carry sign — NE and SW share '/',
 *    NW and SE share '\'.  Surrounding flow disambiguates.
 *  • Screen Y-down: a "northward" math vector has negative vy here.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • RADIAL_OUT: top-centre cell is '^' (up); bottom-right is '\' (SE).
 *  • ROTATION (textbook CCW): visually CW on screen because of Y-flip;
 *                             top moves right.
 *  • POINT_CHARGE: arrows radiate from screen centre, dimming with 1/r².
 *  • CURL_NOISE: arrows curve smoothly, no obvious sources / sinks.
 *  • STABLE_NODE: every arrow points toward screen centre.
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  80,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/* HUD layout — top carries data, bottom carries actions. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Drift multiplier — cranked by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      4
#define DRIFT_MULT_MAX      16

/* Per-cell field-time drift increment per second (radians/sec
 * conceptually — used by Tier 6 animated patterns). */
#define FIELD_DRIFT         1.0f

/* Magnitude below this normalised threshold renders as '.' instead
 * of a directional arrow.  Marks field zeros / equilibrium points. */
#define ARROW_DEAD_ZONE     0.06f

/* Bin width for arrow direction binning (8 cardinal+diagonal bins). */
#define ARROW_BIN_WIDTH     ((float)(M_PI / 4.0))

/* Noise — value-noise field used by GRAD_NOISE (Tier 1) and CURL_NOISE
 * (Tier 4).  Frequency scale is small so the noise is smooth across
 * several cells; the finite-difference gradient stays well-behaved. */
#define NOISE_FREQ          0.10f

/* Tier 1 GRADIENT — knobs for the periodic and ripple scalars. */
#define GRAD_PERIODIC_FREQ  0.25f
#define GRAD_RIPPLE_FREQ    0.30f

/* Tier 3 PHYSICS — dipole half-separation (cells) and singularity
 * softening ε (avoids divide-by-zero at charge centres). */
#define DIPOLE_HALF_SEP     8.0f
#define QUADRUPOLE_HALF     6.0f
#define COULOMB_SOFT_EPS    1.5f

/* Tier 4 SOLENOID — stream-function vortex grid frequency. */
#define STREAM_FREQ_X       0.10f
#define STREAM_FREQ_Y       0.20f

/* Tier 5 DYNAMICS — phase-space half-extent each pattern uses to map
 * screen coords to its native phase-space domain. */
#define PHASE_HALF_EXTENT       3.0f      /* most patterns: x_p, y_p ∈ [-3, 3] */
#define PENDULUM_X_HALF_EXTENT  3.14159f  /* pendulum: x_p ∈ [-π, π]            */
#define VDP_MU                  1.0f      /* Van der Pol non-linearity         */
#define HOPF_MU                 1.0f      /* Hopf limit-cycle radius² parameter */

/* Tier 6 ANIMATED — period constants (seconds). */
#define ROT_DIPOLE_PERIOD   6.0f
#define TRAVEL_WAVE_PERIOD  4.0f
#define BREATHE_PERIOD      4.0f
#define ORBIT_PERIOD        8.0f
#define ORBIT_RADIUS_FRAC   0.30f         /* fraction of half-diagonal */

/* Magnitude saturator scales — per pattern.  |v|_norm = |v|/(|v|+s).
 * Larger s → field has to be stronger before saturating to band 3. */
#define SCALE_RADIAL_HALFDIAG  1.0f       /* normalize by half-diagonal */
#define SCALE_GRADIENT         0.5f       /* gradients of scalars       */
#define SCALE_NOISE_GRAD       2.0f       /* noise gradients are small  */
#define SCALE_INV_SQ           0.4f       /* 1/r² fields                */
#define SCALE_INV_R            1.5f       /* 1/r fields (magnetic wire) */
#define SCALE_BOUNDED          0.8f       /* sin/cos bounded outputs    */
#define SCALE_PHASE            2.0f       /* phase-portrait velocities  */

/*
 * Pattern — 30 vector-field visualisations in 6 complexity tiers.
 * Enum order MUST match `vector_patterns[]` in §7 (compiler enforces
 * via fixed-size [N_PATTERNS] initialiser).
 *
 *   Tier 1 GRADIENT  : GRAD_PARABOLOID, GRAD_SADDLE, GRAD_PERIODIC,
 *                      GRAD_RIPPLE, GRAD_NOISE
 *   Tier 2 ANALYTIC  : RADIAL_OUT, RADIAL_IN, ROTATION, SHEAR_X,
 *                      UNIFORM_TILTED
 *   Tier 3 PHYSICS   : POINT_CHARGE, DIPOLE, WIRE_MAGNETIC,
 *                      GRAVITY, QUADRUPOLE
 *   Tier 4 SOLENOID  : CURL_NOISE, STREAM_GRID, VORTEX_PAIR,
 *                      CHANNEL_FLOW, NOISY_UNIFORM
 *   Tier 5 DYNAMICS  : STABLE_NODE, STABLE_SPIRAL, HOPF_CYCLE,
 *                      VAN_DER_POL, PENDULUM
 *   Tier 6 ANIMATED  : ROTATING_DIPOLE, TRAVELLING_WAVE,
 *                      BREATHING_RADIAL, ORBITING_VORTEX, DRIFT_CURL
 */
typedef enum {
    /* Tier 1 — GRADIENT: ∇f for scalar f */
    PATTERN_GRAD_PARABOLOID = 0,
    PATTERN_GRAD_SADDLE,
    PATTERN_GRAD_PERIODIC,
    PATTERN_GRAD_RIPPLE,
    PATTERN_GRAD_NOISE,
    /* Tier 2 — ANALYTIC: classical 2-D fields */
    PATTERN_RADIAL_OUT,
    PATTERN_RADIAL_IN,
    PATTERN_ROTATION,
    PATTERN_SHEAR_X,
    PATTERN_UNIFORM_TILTED,
    /* Tier 3 — PHYSICS: real-world named fields */
    PATTERN_POINT_CHARGE,
    PATTERN_DIPOLE,
    PATTERN_WIRE_MAGNETIC,
    PATTERN_GRAVITY,
    PATTERN_QUADRUPOLE,
    /* Tier 4 — SOLENOID: divergence-free flows */
    PATTERN_CURL_NOISE,
    PATTERN_STREAM_GRID,
    PATTERN_VORTEX_PAIR,
    PATTERN_CHANNEL_FLOW,
    PATTERN_NOISY_UNIFORM,
    /* Tier 5 — DYNAMICS: 2-D ODE phase portraits */
    PATTERN_STABLE_NODE,
    PATTERN_STABLE_SPIRAL,
    PATTERN_HOPF_CYCLE,
    PATTERN_VAN_DER_POL,
    PATTERN_PENDULUM,
    /* Tier 6 — ANIMATED: time-varying */
    PATTERN_ROTATING_DIPOLE,
    PATTERN_TRAVELLING_WAVE,
    PATTERN_BREATHING_RADIAL,
    PATTERN_ORBITING_VORTEX,
    PATTERN_DRIFT_CURL,
    N_PATTERNS,
} Pattern;

/* Forward decls — definitions live in §7 alongside the dispatch table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

typedef struct {
    const char *name;
    short       band[4];        /* xterm-256 indices, dark → bright */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  24,   33,  220,  231 } },
    { "MATRIX",  {  22,   34,   46,  118 } },
    { "NOVA",    {  53,  129,  201,  219 } },
    { "MONO",    { 234,  244,  250,  254 } },
    { "OCEAN",   {  24,   33,   39,   51 } },
    { "FIRE",    {  52,  124,  208,  226 } },
    { "EARTH",   {  58,  100,  173,  230 } },
    { "FOREST",  {  22,   28,   64,  144 } },
    { "DESERT",  {  94,  130,  173,  222 } },
    { "ARCTIC",  {  24,   39,  159,  231 } },
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
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  vector primitives — noise, scalar gradients, 2-D curl              */
/* ===================================================================== */

/*
 * Per-run noise seed for the Tier 1 GRAD_NOISE and Tier 4 CURL_NOISE
 * patterns.  Re-rolled on every r keypress so each reset shows a
 * fresh field topology without reseeding the global rand() state.
 */
static uint32_t noise_seed = 0;

static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/* Lattice corner scalar ∈ [0, 1] — same (xi, yi, seed) → same value.
 * The same primitive backs the sibling field showcases. */
static inline float lattice_scalar(int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + noise_seed;
    return (float)(hash32(h) >> 8) * (1.0f / 16777215.0f);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

/* Smooth value noise at continuous (x, y) — bilinear blend of the
 * 4 surrounding lattice corners, smoothstep interpolants. */
static float noise_sample(float x, float y)
{
    int   xi = (int)floorf(x);
    int   yi = (int)floorf(y);
    float ux = smoothstep01(x - (float)xi);
    float uy = smoothstep01(y - (float)yi);
    float v00 = lattice_scalar(xi,     yi);
    float v10 = lattice_scalar(xi + 1, yi);
    float v01 = lattice_scalar(xi,     yi + 1);
    float v11 = lattice_scalar(xi + 1, yi + 1);
    return lerpf(lerpf(v00, v10, ux), lerpf(v01, v11, ux), uy);
}

/*
 * Central finite-difference gradient of any scalar f at (x, y).
 *
 *   ∂f/∂x ≈ (f(x+h) - f(x-h)) / (2h)
 *   ∂f/∂y ≈ (f(y+h) - f(y-h)) / (2h)
 *
 * Step h = 1 cell.  Used by GRAD_NOISE and the §7 helpers that take
 * any scalar function.  The 2D curl helper below shares the same
 * derivative form.
 */
typedef float (*ScalarFn2D)(float x, float y);

static inline void scalar_gradient(ScalarFn2D f, float x, float y,
                                   float *vx, float *vy)
{
    *vx = 0.5f * (f(x + 1.0f, y) - f(x - 1.0f, y));
    *vy = 0.5f * (f(x, y + 1.0f) - f(x, y - 1.0f));
}

/*
 * 2-D curl of a scalar potential φ:
 *   v = (∂φ/∂y, -∂φ/∂x)
 *
 * By construction this v has zero divergence (∇·v = ∂vx/∂x + ∂vy/∂y
 * = ∂²φ/∂x∂y - ∂²φ/∂x∂y = 0).  Used by CURL_NOISE and DRIFT_CURL
 * (Tier 4 / Tier 6) for the divergence-free flow visualisations
 * characteristic of incompressible fluids.
 */
static inline void scalar_curl_2d(ScalarFn2D phi, float x, float y,
                                  float *vx, float *vy)
{
    /*  vx =  ∂φ/∂y     */
    *vx =  0.5f * (phi(x, y + 1.0f) - phi(x, y - 1.0f));
    /*  vy = -∂φ/∂x     */
    *vy = -0.5f * (phi(x + 1.0f, y) - phi(x - 1.0f, y));
}

/* Scalars used by Tier 1 GRADIENT patterns.  Defined here so the
 * §7 patterns can call scalar_gradient() on them. */
static float scalar_paraboloid (float x, float y) { return x * x + y * y; }
static float scalar_saddle     (float x, float y) { return x * x - y * y; }
static float scalar_periodic   (float x, float y)
{
    return sinf(x * GRAD_PERIODIC_FREQ) * cosf(y * GRAD_PERIODIC_FREQ);
}
static float scalar_ripple     (float x, float y)
{
    return cosf(GRAD_RIPPLE_FREQ * sqrtf(x * x + y * y));
}
static float scalar_noise      (float x, float y)
{
    return noise_sample(x * NOISE_FREQ, y * NOISE_FREQ);
}

/* ===================================================================== */
/* §6  arrow rendering — direction → glyph, magnitude → palette band      */
/* ===================================================================== */

/*
 * ARROW_GLYPHS — bin index → ASCII character.  Bins are atan2(vy, vx)
 * rounded to the nearest multiple of π/4.  Bin 0 = east (vx > 0,
 * vy ≈ 0), going CCW in atan2 sense (which is CW visually on screen
 * because Y grows DOWN).
 *
 *   bin 0   east       (atan2 ≈  0)            '>'
 *   bin 1   SE on scrn (atan2 ≈  π/4)          '\'   (math NE)
 *   bin 2   south scrn (atan2 ≈  π/2)          'v'   (math S)
 *   bin 3   SW on scrn (atan2 ≈  3π/4)         '/'   (math NW)
 *   bin 4   west       (atan2 ≈ ±π)            '<'
 *   bin 5   NW on scrn (atan2 ≈ -3π/4)         '\'   (math SW)
 *   bin 6   north scrn (atan2 ≈ -π/2)          '^'   (math N)
 *   bin 7   NE on scrn (atan2 ≈ -π/4)          '/'   (math SE)
 *
 * Slope ambiguity: '/' is shared between bin 3 and bin 7, '\' between
 * bins 1 and 5.  The surrounding flow disambiguates visually.
 */
static const char ARROW_GLYPHS[8] = {
    '>', '\\', 'v', '/', '<', '\\', '^', '/',
};

/* Bin formula: bin = (round(angle / (π/4)) + 8) mod 8.
 * The +8 then & 7 normalises negative-angle results into [0, 8). */
static inline int arrow_bin_from_angle(float angle)
{
    return ((int)floorf(angle / ARROW_BIN_WIDTH + 0.5f) + 8) & 7;
}

/* Magnitude saturator: |v| / (|v| + s).  Bounded in [0, 1), monotone,
 * with the half-saturation point at |v| = s.  Safe for 1/r and 1/r²
 * fields that blow up near singularities; the singularity just maps
 * to band 3 instead of overflowing. */
static inline float mag_saturate(float mag, float scale)
{
    return mag / (mag + scale);
}

/* Quartile bucket: normalised magnitude → band ∈ {0, 1, 2, 3}. */
static inline uint8_t mag_to_band(float mag_norm)
{
    int b = (int)(mag_norm * 3.999f);
    if (b < 0) b = 0;
    if (b > 3) b = 3;
    return (uint8_t)b;
}

/* Cell-level emit primitives.  Each pattern function writes via
 * these so the three per-cell outputs are always set together. */
static inline void cell_skip(float *gl, uint8_t *bn, char *gy)
{
    *gl = 0.0f; *bn = 0; *gy = 0;
}

/* "Field zero" marker — '.' in band 0 — for cells where direction
 * is meaningless because magnitude is tiny.  Visually marks
 * equilibria, nulls, and pattern centres. */
static inline void cell_emit_dot(float *gl, uint8_t *bn, char *gy)
{
    *gl = 1.0f; *bn = 0; *gy = '.';
}

/*
 * The arrow emitter — picks glyph from atan2(vy, vx) bin, picks
 * band from the supplied normalised magnitude.  Below DEAD_ZONE
 * the cell renders as a dot instead.
 */
static inline void cell_emit_arrow(float vx, float vy, float mag_norm,
                                   float *gl, uint8_t *bn, char *gy)
{
    if (mag_norm < ARROW_DEAD_ZONE) {
        cell_emit_dot(gl, bn, gy);
        return;
    }
    int bin = arrow_bin_from_angle(atan2f(vy, vx));
    *gl = 1.0f;
    *bn = mag_to_band(mag_norm);
    *gy = ARROW_GLYPHS[bin];
}

/* ===================================================================== */
/* §7  patterns — 30 vector-field visualisations + dispatch               */
/* ===================================================================== */

/*
 * Pattern signature — called PER CELL.  Receives the cell coords +
 * grid dims (so patterns can compute the screen centre), plus the
 * animation accumulator field_time (used by Tier 6).  Writes the
 * three per-cell outputs the §9 renderer reads:
 *   out_glow  — 1.0 = paint, 0.0 = skip
 *   out_band  — palette band ∈ {0..3}
 *   out_glyph — ASCII char to draw; 0 = skip
 *
 * Helper conventions inside patterns:
 *   cx, cy   — screen centre (w/2, h/2)
 *   dx, dy   — offset from centre (cell-units)
 *   r, r²    — radial distance + square; ε-softened where needed
 *   half_dg  — half-diagonal of the grid, used as a natural scale
 */
typedef void (*VectorPatternFn)(int x, int y, int w, int h, float field_time,
                                float *out_glow, uint8_t *out_band, char *out_glyph);

static inline float grid_half_diag(int w, int h)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    return sqrtf(cx * cx + cy * cy);
}

/* ---------- Tier 1 — GRADIENT: ∇f for chosen scalars ----------------- */

/* GRAD_PARABOLOID — ∇(x²+y²) = (2x, 2y).  Radial outward; magnitude
 * grows linearly with distance from the centre. */
static void pattern_grad_paraboloid(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_paraboloid, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
                    gl, bn, gy);
}

/* GRAD_SADDLE — ∇(x²-y²) = (2x, -2y).  Pushes outward along ±x,
 * inward along ±y.  Classic saddle equilibrium at origin. */
static void pattern_grad_saddle(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_saddle, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
                    gl, bn, gy);
}

/* GRAD_PERIODIC — ∇(sin(kx)·cos(ky)).  Periodic grid of maxima/minima
 * with vortex-like rotation patterns at the saddle points between. */
static void pattern_grad_periodic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h; (void)t;
    float vx, vy;
    scalar_gradient(scalar_periodic, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_RIPPLE — ∇cos(k·r), where r = √(x²+y²).  Concentric "lake
 * ripples" of inward/outward arrows alternating each ring. */
static void pattern_grad_ripple(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_ripple, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_NOISE — ∇(value-noise).  Smooth-but-random gradient field; the
 * arrows trace the noise's topography (uphill = arrows point up the
 * noise slope, like a steepest-ascent map). */
static void pattern_grad_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h; (void)t;
    float vx, vy;
    scalar_gradient(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
                    gl, bn, gy);
}

/* ---------- Tier 2 — ANALYTIC: classical 2-D vector fields ----------- */

/* RADIAL_OUT — v = (dx, dy).  Pure radial source; everything outward. */
static void pattern_radial_out(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = (float)x - cx, vy = (float)y - cy;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* RADIAL_IN — v = -(dx, dy).  Pure radial sink; everything inward. */
static void pattern_radial_in(int x, int y, int w, int h, float t,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = -((float)x - cx), vy = -((float)y - cy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* ROTATION — v = (-dy, dx).  Textbook math-CCW rotation; on a screen
 * with Y growing down it APPEARS clockwise (top moves right). */
static void pattern_rotation(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float vx = -dy, vy = dx;
    float mag = sqrtf(dx * dx + dy * dy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* SHEAR_X — v = (dy, 0).  Top row moves one way, bottom row the
 * other; magnitude grows linearly with vertical distance from centre. */
static void pattern_shear_x(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)x; (void)t; (void)w;
    float cy = 0.5f * (float)h;
    float vx = (float)y - cy, vy = 0.0f;
    float mag = fabsf(vx);
    cell_emit_arrow(vx, vy, mag_saturate(mag, 0.5f * (float)h),
                    gl, bn, gy);
}

/* UNIFORM_TILTED — v = (cos 30°, sin 30°).  Constant flow at 30° below
 * horizontal (mild SE drift).  Sanity-check pattern: every cell
 * should show the same glyph. */
static void pattern_uniform_tilted(int x, int y, int w, int h, float t,
                                   float *gl, uint8_t *bn, char *gy)
{
    (void)x; (void)y; (void)w; (void)h; (void)t;
    float vx = cosf((float)M_PI / 6.0f);
    float vy = sinf((float)M_PI / 6.0f);
    cell_emit_arrow(vx, vy, 0.7f, gl, bn, gy);
}

/* ---------- Tier 3 — PHYSICS: real-world named fields --------------- */

/* POINT_CHARGE — Coulomb-like outward field from a single positive
 * charge at the screen centre.  v = (dx, dy) / r², softened by ε to
 * keep the centre cell finite. */
static void pattern_point_charge(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = dx / r2, vy = dy / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* DIPOLE — two charges at (cx - d, cy) [+1] and (cx + d, cy) [-1].
 * Superposed Coulomb fields → classic dipole pattern: arrows leave
 * the + pole, curve through space, and converge at the - pole. */
static void pattern_dipole(int x, int y, int w, int h, float t,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* + charge at (cx - d, cy) */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* − charge at (cx + d, cy) → field points toward it (attractive
     * direction for a test + charge) */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* WIRE_MAGNETIC — infinite straight wire carrying current OUT of the
 * page along +z at screen centre.  B = (1/r) · θ̂ where θ̂ is the
 * right-hand-rule tangent.  In screen coords:  vx = -dy/r², vy = dx/r²
 * (factor of 1/r² because we divide unit tangent by r). */
static void pattern_wire_magnetic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* GRAVITY — point mass at screen centre.  Attractive inverse-square:
 * v = -(dx, dy) / r³ (unit-radial scaled by 1/r²). */
static void pattern_gravity(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float r  = sqrtf(r2);
    float vx = -dx / (r2 * r), vy = -dy / (r2 * r);
    float mag = 1.0f / r2;
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* QUADRUPOLE — 4 alternating charges at (±d, ±d) with signs:
 *   (+d, +d): +     (+d, -d): -
 *   (-d, +d): -     (-d, -d): +
 * Net field shows the characteristic 4-fold pattern with crisp null
 * lines along the axes. */
static void pattern_quadrupole(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = QUADRUPOLE_HALF;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = 0.0f, vy = 0.0f;

    /* Charge layout: signs[i] · field_from((cx+ox[i], cy+oy[i])) */
    static const float ox[4] = { +1.0f, +1.0f, -1.0f, -1.0f };
    static const float oy[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
    static const float sg[4] = { +1.0f, -1.0f, -1.0f, +1.0f };

    for (int i = 0; i < 4; i++) {
        float ax = (float)x - (cx + d * ox[i]);
        float ay = (float)y - (cy + d * oy[i]);
        float ar2 = ax * ax + ay * ay + eps2;
        vx += sg[i] * ax / ar2;
        vy += sg[i] * ay / ar2;
    }
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* ---------- Tier 4 — SOLENOID: divergence-free flows ----------------- */

/* CURL_NOISE — v = (∂φ/∂y, -∂φ/∂x), φ = value-noise.  Divergence-free
 * by construction.  Reads as smooth, curling, "fluid-like" flow with
 * no obvious sources or sinks. */
static void pattern_curl_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h; (void)t;
    float vx, vy;
    scalar_curl_2d(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
                    gl, bn, gy);
}

/* Stream function ψ for STREAM_GRID — a 2-D grid of alternating
 * vortices (sin · sin), each cell of the grid spins one way, its
 * neighbours spin the other. */
static float scalar_stream_grid(float x, float y)
{
    return sinf(x * STREAM_FREQ_X) * sinf(y * STREAM_FREQ_Y);
}

/* STREAM_GRID — v = curl(ψ) for the grid stream function above.
 * Produces a tiled grid of counter-rotating vortices. */
static void pattern_stream_grid(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h; (void)t;
    float vx, vy;
    scalar_curl_2d(scalar_stream_grid, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_BOUNDED * 0.3f), gl, bn, gy);
}

/* VORTEX_PAIR — two opposite rotational centres: one CCW at (cx - d, cy),
 * one CW at (cx + d, cy).  Superposed magnetic-wire fields with
 * opposite currents.  Between them: strong directed flow; outside:
 * decay with distance. */
static void pattern_vortex_pair(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* CCW vortex at (cx - d, cy) */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* CW vortex at (cx + d, cy) — sign flips on the tangent */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = (-ay / ar2) + ( by / br2);
    float vy = ( ax / ar2) + (-bx / br2);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* CHANNEL_FLOW — Poiseuille parabolic profile in a channel along x.
 * v = (1 - (2y/h - 1)², 0).  Fastest in the middle, zero at the
 * walls.  Always-positive vx → pure rightward flow with varying speed. */
static void pattern_channel_flow(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    (void)x; (void)w; (void)t;
    float u = 2.0f * (float)y / (float)(h - 1) - 1.0f;     /* u ∈ [-1, +1] */
    float vx = 1.0f - u * u;                               /* parabolic */
    float vy = 0.0f;
    cell_emit_arrow(vx, vy, vx * 0.9f, gl, bn, gy);
}

/* NOISY_UNIFORM — v = (1, 0) + small noise gradient.  Uniform
 * rightward flow with a sprinkle of turbulence; demonstrates how a
 * mean flow plus small perturbation reads to the eye. */
static void pattern_noisy_uniform(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h; (void)t;
    float nx, ny;
    scalar_gradient(scalar_noise, (float)x, (float)y, &nx, &ny);
    float vx = 1.0f + nx * 4.0f;
    float vy =        ny * 4.0f;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, 1.5f), gl, bn, gy);
}

/* ---------- Tier 5 — DYNAMICS: 2-D ODE phase portraits -------------- *
 *
 * Phase portraits map each screen cell to a point (x_p, y_p) in the
 * ODE's native phase space, then plot (dx/dt, dy/dt) as an arrow.
 *
 * The patterns below use SCREEN-y-down throughout.  Textbook
 * formulas usually assume y-up; we adopt the screen convention so
 * the arrows render literally — no sign flip in the emit call.
 * The visible "up direction" on screen corresponds to NEGATIVE y_p
 * in our parameterisation; this is documented per-pattern.
 */

/* Helper — map screen (x, y) to phase-space (x_p, y_p) ∈ [-h, +h]
 * using the supplied half-extent.  Screen y-down → y_p grows DOWN in
 * the phase-space too (textbook flips this; we don't, see note above). */
static inline void cell_to_phase(int x, int y, int w, int h, float half_extent,
                                 float *xp, float *yp)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    *xp = ((float)x - cx) * (half_extent / cx);
    *yp = ((float)y - cy) * (half_extent / cy);
}

/* STABLE_NODE — dx/dt = -x_p,  dy/dt = -y_p.
 * Every trajectory approaches the origin along straight lines.
 * Looks like RADIAL_IN but with linear-in-distance magnitude. */
static void pattern_stable_node(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp, vy = -yp;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* STABLE_SPIRAL — dx/dt = -x_p - y_p,  dy/dt =  x_p - y_p.
 * Linear system with complex eigenvalues (negative real part) →
 * spiral attractor: trajectories rotate inward to the origin. */
static void pattern_stable_spiral(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp - yp;
    float vy =  xp - yp;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* HOPF_CYCLE — Hopf normal form:
 *   dx/dt = (μ - r²) x - y         where r² = x² + y²
 *   dy/dt = x + (μ - r²) y
 *
 * For μ > 0 there is a STABLE LIMIT CYCLE at r = √μ.  Arrows inside
 * the cycle push outward; arrows outside push inward; arrows ON the
 * cycle rotate tangentially.  The classic Hopf bifurcation visualisation. */
static void pattern_hopf_cycle(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float r2  = xp * xp + yp * yp;
    float mu_minus_r2 = HOPF_MU - r2;
    float vx = mu_minus_r2 * xp - yp;
    float vy = xp + mu_minus_r2 * yp;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* VAN_DER_POL — relaxation oscillator (van der Pol 1926):
 *   dx/dt = y
 *   dy/dt = μ (1 - x²) y - x
 *
 * For μ > 0, all trajectories converge to a unique LIMIT CYCLE
 * (non-circular, "relaxation" shape).  Origin is unstable. */
static void pattern_van_der_pol(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = yp;
    float vy = VDP_MU * (1.0f - xp * xp) * yp - xp;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_PHASE * 2.0f), gl, bn, gy);
}

/* PENDULUM — undamped nonlinear pendulum:
 *   dx/dt = y          (x = angle, y = angular velocity)
 *   dy/dt = -sin(x)
 *
 * x ranges over [-π, π] (one full rotation).  Trajectories show:
 *   - Closed orbits near (0, 0): small swings
 *   - Open curves above / below: full rotations
 *   - SEPARATRIX: the curve passing through (±π, 0) — the boundary
 *     between oscillation and rotation. */
static void pattern_pendulum(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    (void)t;
    /* Phase x ∈ [-π, π]; phase y ∈ [-3, 3]. */
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float xp = ((float)x - cx) * (PENDULUM_X_HALF_EXTENT / cx);
    float yp = ((float)y - cy) * (PHASE_HALF_EXTENT / cy);
    float vx = yp;
    float vy = -sinf(xp);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* ---------- Tier 6 — ANIMATED: time-varying vector fields ----------- */

/* ROTATING_DIPOLE — like DIPOLE, but the dipole's orientation rotates
 * around the screen centre at constant angular speed.  At t=0 the
 * dipole is horizontal; one period later it's back. */
static void pattern_rotating_dipole(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float omega = 2.0f * (float)M_PI / ROT_DIPOLE_PERIOD;
    float ang = omega * t;
    float dx_pole = DIPOLE_HALF_SEP * cosf(ang);
    float dy_pole = DIPOLE_HALF_SEP * sinf(ang);
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* + at (cx + dx_pole, cy + dy_pole); − at the opposite side */
    float ax = (float)x - (cx + dx_pole), ay = (float)y - (cy + dy_pole);
    float ar2 = ax * ax + ay * ay + eps2;
    float bx = (float)x - (cx - dx_pole), by = (float)y - (cy - dy_pole);
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* TRAVELLING_WAVE — v = (sin(kx - ωt), 0).  Plane wave moving in
 * the +x direction.  Watch the bright/dark bands ripple rightward. */
static void pattern_travelling_wave(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    (void)y; (void)w; (void)h;
    float k     = 0.25f;
    float omega = 2.0f * (float)M_PI / TRAVEL_WAVE_PERIOD;
    float vx    = sinf(k * (float)x - omega * t);
    float vy    = 0.0f;
    float mag   = fabsf(vx);
    cell_emit_arrow(vx, vy, mag * 0.95f, gl, bn, gy);
}

/* BREATHING_RADIAL — RADIAL_OUT with magnitude modulated by sin(ωt).
 * Half the cycle: outward source.  Other half: inward sink.  Pulses
 * between the two extremes through zero (where everything reads as
 * '.' field-zero markers). */
static void pattern_breathing_radial(int x, int y, int w, int h, float t,
                                     float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float omega = 2.0f * (float)M_PI / BREATHE_PERIOD;
    float s = sinf(omega * t);
    float vx = dx * s, vy = dy * s;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* ORBITING_VORTEX — a single rotational vortex whose centre traces
 * a circle around the screen centre.  Combines spatial structure
 * with temporal motion. */
static void pattern_orbiting_vortex(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx     = 0.5f * (float)w, cy = 0.5f * (float)h;
    float half_d = grid_half_diag(w, h);
    float omega  = 2.0f * (float)M_PI / ORBIT_PERIOD;
    float r_orb  = half_d * ORBIT_RADIUS_FRAC;
    float vcx    = cx + r_orb * cosf(omega * t);
    float vcy    = cy + r_orb * sinf(omega * t);
    float eps2   = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    float dx = (float)x - vcx, dy = (float)y - vcy;
    float r2 = dx * dx + dy * dy + eps2;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* DRIFT_CURL — CURL_NOISE with the noise potential drifting in y over
 * time.  Same divergence-free structure as Tier 4 CURL_NOISE but the
 * field evolves continuously, like ASCII smoke. */
static void pattern_drift_curl(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h;
    /* Sample noise at (x, y + drift·t) — finite-diff via 4 reads. */
    float drift = t * 0.7f;
    float n_yp = noise_sample((float)x * NOISE_FREQ, ((float)y + 1.0f + drift) * NOISE_FREQ);
    float n_ym = noise_sample((float)x * NOISE_FREQ, ((float)y - 1.0f + drift) * NOISE_FREQ);
    float n_xp = noise_sample(((float)x + 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    float n_xm = noise_sample(((float)x - 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    /* v = (∂φ/∂y, -∂φ/∂x) */
    float vx =  0.5f * (n_yp - n_ym);
    float vy = -0.5f * (n_xp - n_xm);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
                    gl, bn, gy);
}

/* ---------- Dispatch table ------------------------------------------- */

typedef struct {
    const char      *name;        /* 10-char padded for HUD alignment  */
    const char      *tier;        /* 7-char "N-LABEL " padded          */
    VectorPatternFn  sample;
} VectorPattern;

static const VectorPattern vector_patterns[N_PATTERNS] = {
    /* Tier 1 — GRADIENT */
    [PATTERN_GRAD_PARABOLOID] = { "GRAD-PARAB", "1-GRAD ", pattern_grad_paraboloid },
    [PATTERN_GRAD_SADDLE]     = { "GRAD-SADDL", "1-GRAD ", pattern_grad_saddle     },
    [PATTERN_GRAD_PERIODIC]   = { "GRAD-PERIO", "1-GRAD ", pattern_grad_periodic   },
    [PATTERN_GRAD_RIPPLE]     = { "GRAD-RIPPL", "1-GRAD ", pattern_grad_ripple     },
    [PATTERN_GRAD_NOISE]      = { "GRAD-NOISE", "1-GRAD ", pattern_grad_noise      },
    /* Tier 2 — ANALYTIC */
    [PATTERN_RADIAL_OUT]      = { "RADIAL-OUT", "2-ANLY ", pattern_radial_out      },
    [PATTERN_RADIAL_IN]       = { "RADIAL-IN ", "2-ANLY ", pattern_radial_in       },
    [PATTERN_ROTATION]        = { "ROTATION  ", "2-ANLY ", pattern_rotation        },
    [PATTERN_SHEAR_X]         = { "SHEAR-X   ", "2-ANLY ", pattern_shear_x         },
    [PATTERN_UNIFORM_TILTED]  = { "UNIF-TILT ", "2-ANLY ", pattern_uniform_tilted  },
    /* Tier 3 — PHYSICS */
    [PATTERN_POINT_CHARGE]    = { "POINT-CHG ", "3-PHYS ", pattern_point_charge    },
    [PATTERN_DIPOLE]          = { "DIPOLE    ", "3-PHYS ", pattern_dipole          },
    [PATTERN_WIRE_MAGNETIC]   = { "WIRE-MAG  ", "3-PHYS ", pattern_wire_magnetic   },
    [PATTERN_GRAVITY]         = { "GRAVITY   ", "3-PHYS ", pattern_gravity         },
    [PATTERN_QUADRUPOLE]      = { "QUADRUPOLE", "3-PHYS ", pattern_quadrupole      },
    /* Tier 4 — SOLENOID */
    [PATTERN_CURL_NOISE]      = { "CURL-NOISE", "4-SOLN ", pattern_curl_noise      },
    [PATTERN_STREAM_GRID]     = { "STREAM-GRD", "4-SOLN ", pattern_stream_grid     },
    [PATTERN_VORTEX_PAIR]     = { "VORTX-PAIR", "4-SOLN ", pattern_vortex_pair     },
    [PATTERN_CHANNEL_FLOW]    = { "CHAN-FLOW ", "4-SOLN ", pattern_channel_flow    },
    [PATTERN_NOISY_UNIFORM]   = { "NOISY-UNI ", "4-SOLN ", pattern_noisy_uniform   },
    /* Tier 5 — DYNAMICS */
    [PATTERN_STABLE_NODE]     = { "STBL-NODE ", "5-DYNS ", pattern_stable_node     },
    [PATTERN_STABLE_SPIRAL]   = { "STBL-SPRL ", "5-DYNS ", pattern_stable_spiral   },
    [PATTERN_HOPF_CYCLE]      = { "HOPF-CYCLE", "5-DYNS ", pattern_hopf_cycle      },
    [PATTERN_VAN_DER_POL]     = { "VAN-DR-POL", "5-DYNS ", pattern_van_der_pol     },
    [PATTERN_PENDULUM]        = { "PENDULUM  ", "5-DYNS ", pattern_pendulum        },
    /* Tier 6 — ANIMATED */
    [PATTERN_ROTATING_DIPOLE] = { "ROT-DIPOLE", "6-ANIM ", pattern_rotating_dipole },
    [PATTERN_TRAVELLING_WAVE] = { "TRAV-WAVE ", "6-ANIM ", pattern_travelling_wave },
    [PATTERN_BREATHING_RADIAL]= { "BREATHE   ", "6-ANIM ", pattern_breathing_radial},
    [PATTERN_ORBITING_VORTEX] = { "ORBT-VORTX", "6-ANIM ", pattern_orbiting_vortex },
    [PATTERN_DRIFT_CURL]      = { "DRIFT-CURL", "6-ANIM ", pattern_drift_curl      },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return vector_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return vector_patterns[p].tier;
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct {
    int      w, h;
    int      total_cells;

    /* Per-cell render outputs — patterns write, §9 reads.
     *   trail_glow  : 1.0 = paint at this cell, 0.0 = skip
     *   trail_color : palette band ∈ {0..3} → PAIR_BAND_BASE + band
     *   trail_glyph : ASCII char to draw; 0 = skip (defensive)
     */
    float    trail_glow [CELLS_MAX];
    uint8_t  trail_color[CELLS_MAX];
    char     trail_glyph[CELLS_MAX];

    float    field_time;   /* drift accumulator (seconds) — drives Tier 6 */
} Field;

static inline int field_idx(const Field *f, int x, int y) { return y * f->w + x; }

static void field_reset(Field *f, int w, int h)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->field_time  = 0.0f;
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
        f->trail_glyph[i] = 0;
    }
    /* Re-roll noise seed so noise-driven patterns refresh on r. */
    noise_seed = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/* Dispatch the active pattern at every cell. */
static void field_render(Field *f, Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return;
    VectorPatternFn sample = vector_patterns[p].sample;

    for (int y = 0; y < f->h; y++) {
        for (int x = 0; x < f->w; x++) {
            int idx = field_idx(f, x, y);
            sample(x, y, f->w, f->h, f->field_time,
                   &f->trail_glow[idx],
                   &f->trail_color[idx],
                   &f->trail_glyph[idx]);
        }
    }
}

typedef struct {
    Field   F;
    bool    paused;
    int     drift_mult;
    int     current_theme;
    Pattern current_pattern;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    field_reset(&s->F, mw, mh);
    field_render(&s->F, s->current_pattern);     /* paint initial frame */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->drift_mult      = DRIFT_MULT_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_GRAD_PARABOLOID;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;
    f->field_time += FIELD_DRIFT * (float)s->drift_mult * dt;
    field_render(f, s->current_pattern);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Field *f = &s->F;

    /* Centre the field inside the viewport (HUD-aware). */
    int gx0 = (cols - f->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - f->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;

    for (int y = 0; y < f->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < f->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int  idx = field_idx(f, x, y);
            if (f->trail_glow[idx] <= 0.0f) continue;
            char glyph = f->trail_glyph[idx];
            if (glyph == 0 || glyph == ' ') continue;

            int pair = PAIR_BAND_BASE + (f->trail_color[idx] & 3);
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED    "
                          : pattern_name(s->current_pattern);

    /* Row 0 right — primary state with [N/M] pattern index. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d ",
             fps, sim_fps, state_str,
             (int)s->current_pattern + 1, N_PATTERNS,
             s->drift_mult);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " VECTOR FIELD ARROWS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + tier + theme + palette swatches + parameters. */
    int x = HUD_LEFT_MARGIN;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-10s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 21;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " tier:%-7s ", pattern_tier(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 15;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  t:%.1fs  map:%dx%d ",
             (double)f->field_time, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint — actions only. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:drift  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

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

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            s->paused = !s->paused; break;
    case 'r': case 'R':  scene_reset(s, app->map_w, app->map_h); break;
    case '=': case '+':
        if (s->drift_mult < DRIFT_MULT_MAX) s->drift_mult *= 2;
        if (s->drift_mult > DRIFT_MULT_MAX) s->drift_mult = DRIFT_MULT_MAX;
        break;
    case '-':
        s->drift_mult /= 2;
        if (s->drift_mult < DRIFT_MULT_MIN) s->drift_mult = DRIFT_MULT_MIN;
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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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
