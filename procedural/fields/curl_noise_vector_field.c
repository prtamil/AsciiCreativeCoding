/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_vector_field.c
 *   — Divergence-free curl noise: 5 ways to see a vector field.
 *
 * DEMO: Curl noise is the ∇×ψ of a scalar noise potential — a 2-D
 *       vector field that is exactly DIVERGENCE-FREE by construction.
 *       Particles flowing through it swirl into vortices and never
 *       pile up or run dry; the field's mass is conserved. Five
 *       patterns visualise the same underlying field in different ways:
 *         PARTICLES — 256 particles trace streamlines (default)
 *         VECTOR    — sparse arrow glyphs '>', '<', '^', 'v', '/', '\'
 *                     showing the field direction at lattice points
 *         POTENTIAL — render the scalar potential ψ as a heightmap
 *         CURL_MAG  — render |∇×ψ| as density heat — bright at
 *                     vortex centres, dim at irrotational regions
 *         WARPED    — particle flow over a domain-warped potential
 *                     (more chaotic, eddy-rich streamlines)
 *       Field drifts so the flow evolves rather than looping.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — particles flow on a noise GRADIENT
 *       (∇ψ), which is divergent. Curl flow uses the perpendicular
 *       (∇×ψ), which is divergence-free. Side-by-side: gradient
 *       streams converge to peaks; curl streams loop forever.
 *   ./domain_warped_noise_iq_style.c — domain warping technique used
 *       by the WARPED pattern here.
 *   ./worley_cellular_noise.c — discrete cells; this file is smooth.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 noise    — Perlin + fBm + potential + curl
 *   §6 patterns — 5 visualisations of the field
 *   §7 scene    — Field, particles, scene state, per-frame update
 *   §8 screen   — ASCII render: density glyphs + arrow glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new permutation)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster particles / drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra curl_noise_vector_field.c \
 *       -o curl_noise -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Curl noise is a way to derive a smooth, divergence-
 *                  free 2-D vector field from a scalar noise function.
 *
 *                  Given a scalar potential ψ(x, y), define
 *                      v(x, y) = (∂ψ/∂y, −∂ψ/∂x).
 *                  This v is identically divergence-free by the
 *                  cross-derivative theorem:
 *                      ∇·v = ∂vx/∂x + ∂vy/∂y
 *                          = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0.
 *                  So a particle flowing along v never converges to a
 *                  source or diverges to a sink — the field has no
 *                  "wells" or "fountains". Combined with a smooth ψ
 *                  (e.g. Perlin / fBm) you get vortex-rich flow that
 *                  looks like fluid or smoke.
 *
 *                  We compute v by central finite difference:
 *                      ∂ψ/∂y ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / (2ε)
 *                      ∂ψ/∂x ≈ (ψ(x+ε, y) − ψ(x−ε, y)) / (2ε)
 *                  with ε small enough to capture local curvature
 *                  but large enough to keep the noise output stable
 *                  (we use ε = 0.5 noise-units).
 *
 *                  Five patterns visualise the field; PARTICLES is the
 *                  default ("flow" view), VECTOR shows arrows at
 *                  lattice points, POTENTIAL shows the underlying
 *                  scalar ψ as a heightmap, CURL_MAG shows |v| as
 *                  density (bright at vortex cores), WARPED applies
 *                  domain warping to ψ before taking the curl.
 *
 * Data-structure : Permutation table for Perlin (re-shuffled at reset).
 *                  Per-cell glow + colour buffers + an optional glyph
 *                  buffer (used by VECTOR for arrow direction). Static
 *                  particle pool for PARTICLES and WARPED. No heap.
 *
 * Rendering      : ASCII only. Density-based '.', '*', '#' for the
 *                  field-as-scalar patterns. The VECTOR pattern uses
 *                  arrow glyphs '>', '<', '^', 'v', '/', '\\' chosen
 *                  by the local velocity direction. Each pattern
 *                  populates trail_glow + trail_color uniformly so
 *                  the colour-banding logic stays the same.
 *
 * Performance    : 4 fBm calls per cell for CURL_MAG (the most
 *                  expensive pattern), each with 3 octaves. ~135 K
 *                  perlin/sec on a 200×56 grid at 60 Hz, well under
 *                  1 % of one core on modern hardware.
 *
 * References     : • Bridson, Houriham & Nordenstam (2007) — "Curl-
 *                    Noise for Procedural Fluid Flow" (the original
 *                    SIGGRAPH paper introducing curl noise to
 *                    graphics):
 *                    https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph2007-curlnoise.pdf
 *                  • Inigo Quilez — "Useful little functions" (curl
 *                    noise sketches):
 *                    https://iquilezles.org/articles/
 *                  • Wikipedia — "Vector field":
 *                    https://en.wikipedia.org/wiki/Vector_field
 *                  • Compare ./perin_noise_flow_showcase.c — gradient
 *                    flow (divergent) vs curl flow (divergence-free).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Most procedural flow demos use the GRADIENT of a noise field to
 * push particles. That works but it has a major flaw: gradients
 * point UPHILL — particles converge into local maxima and pile up.
 * Curl noise fixes this by rotating the gradient 90° (∇×ψ), which
 * gives a velocity that swirls AROUND maxima instead of toward
 * them. The result is divergence-free flow: particles never
 * accumulate; they orbit eternally.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the noise as a bumpy hill landscape. The GRADIENT points
 * uphill, so particles roll toward peaks (and stop). The CURL
 * points along contour lines — perpendicular to the gradient — so
 * particles trace the contours forever. Add multiple peaks and
 * valleys to the landscape and the contours form swirling vortices
 * around each one. That's curl noise: vortex-rich, conservative
 * flow, all from one scalar function.
 *
 * Visible layers (per pattern):
 *   PARTICLES : particle trails curving around invisible vortex
 *               centres. Watch any one '#' and it will trace a
 *               smooth curve, never a straight line.
 *   VECTOR    : arrow glyphs at every 4×2 lattice point showing
 *               the local velocity direction.
 *   POTENTIAL : the SOURCE function ψ as a heightmap. The peaks
 *               you see here are the vortex CENTRES of PARTICLES.
 *   CURL_MAG  : |v| = √(vx² + vy²). Bright cells are where the
 *               flow is fastest (in tight vortices); dark cells
 *               are smooth gradient regions.
 *   WARPED    : particles, but the potential is itself warped by
 *               another noise — vortices break apart and reform.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Shuffle the perm[] table for Perlin noise. For PARTICLES
 *     and WARPED, spawn N particles at random in-bounds positions.
 *  2. PER FRAME:
 *     a. Drift field_time by FIELD_DRIFT × dt (for animated patterns).
 *     b. Pattern-specific:
 *        • PARTICLES: for each particle, compute (vx, vy) via curl
 *          of potential(x, y, t); step by v · dt; respawn on OOB.
 *        • VECTOR: at every 4×2 cell, compute v, store an arrow
 *          glyph corresponding to v's direction.
 *        • POTENTIAL: at every cell, glow = potential(x, y, t)
 *          remapped to [0, 1].
 *        • CURL_MAG: at every cell, compute v, glow = |v| / max.
 *        • WARPED: like PARTICLES but use warped_potential(x, y, t).
 *     c. Render trail_glow / trail_glyph through the standard
 *        density-band pipeline.
 *  3. Periodic permutation reshuffle for variety.
 *
 * KEY FORMULAS
 * ────────────
 *  Scalar potential              : ψ(x, y) = fbm(x · scale, y · scale + t)
 *  Curl (2-D)                    : v = (∂ψ/∂y, −∂ψ/∂x)
 *  Central finite difference     : ∂ψ/∂x ≈ (ψ(x+ε) − ψ(x−ε)) / (2ε)
 *  Particle update               : (x, y) ← (x, y) + v · dt
 *  Divergence (zero by design)   : ∇·v = ∂vx/∂x + ∂vy/∂y = 0
 *  Curl magnitude                : |v| = √(vx² + vy²)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CHOICE OF ε. Too small and floating-point noise dominates the
 *    finite difference (noisy velocities). Too large and you blur
 *    over local features. ε = 0.5 noise-units is a sweet spot for
 *    smooth Perlin/fBm input.
 *
 *  • DIVERGENCE-FREE BY CONSTRUCTION. Numerical errors in finite
 *    differences mean v is APPROXIMATELY divergence-free, not
 *    exactly. For a showcase the approximation is invisible; for
 *    actual fluid simulation you'd use exact analytic derivatives.
 *
 *  • PARTICLE RESPAWN. Particles still leave the visible map if
 *    they wander off the edge (the field doesn't loop). We respawn
 *    on OOB. Without respawn, the screen empties over time.
 *
 *  • WARPED COMBO COST. WARPED uses warped_potential which itself
 *    needs 3 fbm calls; combined with the 4-sample finite difference
 *    that's 12 fbm per particle per tick. Still fast enough.
 *
 *  • VECTOR LATTICE SPACING. Drawing arrows at every cell is too
 *    dense to read. We sample every 4×2 cells (matching terminal
 *    aspect ratio so the lattice looks uniform). Sparser is fine
 *    too — try every 8×4 if your terminal is small.
 *
 *  • FIELD AT (0, 0). Perlin noise is 0 at the origin by design.
 *    Don't use the curl at (0, 0) as a sanity check — it's exactly
 *    zero, and so is the velocity. Sample anywhere else.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PARTICLES pattern: every particle traces SMOOTH curves. Never
 *    a perfectly straight line for more than a few cells. If you
 *    see straight lines, the curl computation is broken (you're
 *    probably reading the gradient).
 *  • VECTOR pattern: arrows form coherent local "wind" — neighbouring
 *    arrows point in similar directions. If they look random, the
 *    field is being evaluated wrong (e.g. uncorrelated noise per
 *    cell).
 *  • POTENTIAL: smooth rolling hills, no axis bias.
 *  • CURL_MAG: bright "hot spots" at points where the potential's
 *    gradient is steepest. These are the vortex centres of PARTICLES.
 *  • WARPED: particles still trace smooth curves (still divergence-
 *    free) but with sharper turns and more breakup of long streamlines.
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

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* For PARTICLES and WARPED. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* Speed: cells/sec along the curl velocity. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* Periodic reset cadence. */
    RESET_TICKS_DEF   = 12 * 60,

    /* VECTOR pattern: arrow lattice spacing (in cells). 4x2 chosen
     * because terminal cells are ~2x taller than wide, so this gives
     * a roughly square arrow grid in pixel space. */
    VECTOR_LATTICE_X  =   4,
    VECTOR_LATTICE_Y  =   2,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Glow decays for particle trails (only). Static-field patterns
 * overwrite the buffer each frame so they don't decay. */
#define TRAIL_GLOW_DECAY    0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale and animation speed. */
#define NOISE_SCALE         0.05f
#define FIELD_DRIFT         0.10f
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* Finite-difference step for ∂ψ/∂x and ∂ψ/∂y. ε = 0.5 noise-units
 * is a sweet spot — small enough to capture local curvature, large
 * enough that float jitter doesn't dominate. */
#define CURL_EPS            0.5f

/* fBm octaves for the potential. */
#define FBM_OCTAVES         3

/* WARPED pattern: warp magnitude. */
#define WARP_AMOUNT         3.0f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

typedef enum {
    PATTERN_PARTICLES = 0,
    PATTERN_VECTOR    = 1,
    PATTERN_POTENTIAL = 2,
    PATTERN_CURL_MAG  = 3,
    PATTERN_WARPED    = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_PARTICLES: return "PARTICLES";
    case PATTERN_VECTOR:    return "VECTOR   ";
    case PATTERN_POTENTIAL: return "POTENTIAL";
    case PATTERN_CURL_MAG:  return "CURL_MAG ";
    case PATTERN_WARPED:    return "WARPED   ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the rest of the procedural showcases.
 */
typedef struct {
    const char *name;
    short       band[4];
    short       flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 234,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  52,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
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
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  noise — Perlin + fBm + potential + curl                            */
/* ===================================================================== */

static uint8_t perm[512];

static void perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i]       = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

static inline float grad(int hash, float x, float y)
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
    x -= floorf(x);
    y -= floorf(y);

    float u = fade(x);
    float v = fade(y);

    int A  = perm[X    ] + Y;
    int B  = perm[X + 1] + Y;

    float n00 = grad(perm[A    ], x,        y       );
    float n10 = grad(perm[B    ], x - 1.0f, y       );
    float n01 = grad(perm[A + 1], x,        y - 1.0f);
    float n11 = grad(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/*
 * fbm — 3-octave fractional Brownian motion. Output ≈ [-1, 1].
 */
static float fbm(float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/*
 * potential — the scalar function ψ(x, y, t) whose curl is our
 * velocity field. Plain fBm with time drifted into the y-axis.
 */
static inline float potential(float x, float y, float t)
{
    return fbm(x, y + t);
}

/*
 * warped_potential — ψ with one level of domain warping applied
 * before the final fbm sample. Particles flowing through ∇× of this
 * see more chaotic streamlines.
 */
static float warped_potential(float x, float y, float t)
{
    float qx = fbm(x,        y       + t);
    float qy = fbm(x + 5.2f, y + 1.3f + t);
    return fbm(x + WARP_AMOUNT * qx,
               y + WARP_AMOUNT * qy + t);
}

/*
 * curl_at — central-difference curl of the chosen potential at (x, y, t).
 * Returns (vx, vy) via out-pointers.
 *
 * Mathematical identity: v = (∂ψ/∂y, −∂ψ/∂x). Numerical:
 *   ∂ψ/∂y ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / (2ε)
 *   ∂ψ/∂x ≈ (ψ(x+ε, y) − ψ(x−ε, y)) / (2ε)
 *
 * The `warp` flag selects between potential() and warped_potential().
 */
static void curl_at(float x, float y, float t, bool warp,
                    float *out_vx, float *out_vy)
{
    float yp, ym, xp, xm;
    if (warp) {
        yp = warped_potential(x, y + CURL_EPS, t);
        ym = warped_potential(x, y - CURL_EPS, t);
        xp = warped_potential(x + CURL_EPS, y, t);
        xm = warped_potential(x - CURL_EPS, y, t);
    } else {
        yp = potential(x, y + CURL_EPS, t);
        ym = potential(x, y - CURL_EPS, t);
        xp = potential(x + CURL_EPS, y, t);
        xm = potential(x - CURL_EPS, y, t);
    }
    float dpsi_dy = (yp - ym) / (2.0f * CURL_EPS);
    float dpsi_dx = (xp - xm) / (2.0f * CURL_EPS);
    *out_vx =  dpsi_dy;
    *out_vy = -dpsi_dx;
}

/* ===================================================================== */
/* §6  patterns — 5 visualisations of the same field                      */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * arrow_for — pick an arrow glyph from a velocity direction. Used by
 * the VECTOR pattern.
 *
 * Axis-aligned slow → '.'. Mostly horizontal → '>' or '<'. Mostly
 * vertical → '^' or 'v'. Diagonal → '/' or '\\' (only one symbol per
 * diagonal direction since '\\' visually represents both NW→SE and
 * SE→NW — the field's direction is implied by neighbouring arrows).
 */
static char arrow_for(float vx, float vy)
{
    float ax = fabsf(vx), ay = fabsf(vy);
    if (ax < 0.05f && ay < 0.05f) return '.';
    if (ax > 2.0f * ay) return vx > 0.0f ? '>' : '<';
    if (ay > 2.0f * ax) return vy > 0.0f ? 'v' : '^';
    return ((vx * vy) > 0.0f) ? '\\' : '/';
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Particle (used by PARTICLES and WARPED).
 */
typedef struct {
    float x, y;
    int   color_idx;
    int   age;
    int   max_age;
} Particle;

/*
 * Field — the simulation state. Three buffers per cell:
 *   trail_glow   : intensity (drives the density glyph + colour band)
 *   trail_color  : palette index 0..3
 *   trail_glyph  : when non-zero, OVERRIDES the density glyph (used
 *                  by the VECTOR pattern to render arrows directly)
 */
typedef struct {
    int      w, h;
    int      total_cells;

    float    trail_glow [CELLS_MAX];
    uint8_t  trail_color[CELLS_MAX];
    char     trail_glyph[CELLS_MAX];   /* 0 = use density glyph */
    float    supernova_glow_t;

    Particle particles[MAX_PARTICLES];
    int      n_particles;

    float    field_time;
    int      reset_countdown;
} Field;

static inline int field_idx(const Field *f, int x, int y) { return y * f->w + x; }
static inline bool field_in_bounds(const Field *f, int x, int y)
{
    return x >= 0 && x < f->w && y >= 0 && y < f->h;
}

static void particle_spawn(Field *f, Particle *p)
{
    p->x         = (float)(rand() % f->w);
    p->y         = (float)(rand() % f->h);
    p->color_idx = rand() & 3;
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/*
 * particle_step_curl — sample curl noise at the particle's position,
 * step by velocity * dt. `warp` selects warped vs plain potential.
 */
static void particle_step_curl(Field *f, Particle *p, float dt, int speed, bool warp)
{
    float vx, vy;
    curl_at(p->x * NOISE_SCALE,
            p->y * NOISE_SCALE,
            f->field_time, warp, &vx, &vy);

    /* Normalise so the magnitude doesn't drown out direction. */
    float m = sqrtf(vx * vx + vy * vy);
    if (m > 1e-6f) { vx /= m; vy /= m; }

    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
    p->age++;

    int cx = (int)p->x;
    int cy = (int)p->y;
    if (field_in_bounds(f, cx, cy)) {
        int idx = field_idx(f, cx, cy);
        f->trail_glow[idx]  = 1.0f;
        f->trail_color[idx] = (uint8_t)p->color_idx;
        f->trail_glyph[idx] = 0;        /* density glyph */
    }

    if (p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)f->w
        || p->y < 0.0f || p->y >= (float)f->h) {
        particle_spawn(f, p);
    }
}

/*
 * field_clear — wipe all per-cell buffers. Called when switching
 * between particle-based and static-field patterns so leftover
 * trails don't ghost into the next pattern's view.
 */
static void field_clear(Field *f)
{
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
        f->trail_glyph[i] = 0;
    }
}

/*
 * field_update_static — for non-particle patterns, recompute every
 * cell each frame.
 */
static void field_update_static(Field *f, Pattern p)
{
    float t = f->field_time;
    int n_cells = f->total_cells;
    /* For VECTOR: most cells stay blank, only the lattice points
     * carry an arrow. So we wipe first, then fill selectively. */
    if (p == PATTERN_VECTOR) {
        for (int i = 0; i < n_cells; i++) {
            f->trail_glow[i]  = 0.0f;
            f->trail_glyph[i] = 0;
        }
    }

    for (int y = 0; y < f->h; y++) {
        for (int x = 0; x < f->w; x++) {
            int idx = field_idx(f, x, y);
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            float g = 0.0f;
            int   col = 0;
            char  glyph = 0;

            switch (p) {
            case PATTERN_VECTOR: {
                if ((x % VECTOR_LATTICE_X) != 0) continue;
                if ((y % VECTOR_LATTICE_Y) != 0) continue;
                float vx, vy;
                curl_at(fx, fy, t, false, &vx, &vy);
                float mag = sqrtf(vx * vx + vy * vy);
                g     = clampf(mag * 1.5f, 0.0f, 1.0f);
                col   = (int)(g * 3.999f) & 3;
                glyph = arrow_for(vx, vy);
                break;
            }
            case PATTERN_POTENTIAL: {
                float psi = potential(fx, fy, t);
                g   = psi * 0.5f + 0.5f;
                col = (int)(g * 3.999f) & 3;
                break;
            }
            case PATTERN_CURL_MAG: {
                float vx, vy;
                curl_at(fx, fy, t, false, &vx, &vy);
                float mag = sqrtf(vx * vx + vy * vy);
                g   = clampf(mag * 1.5f, 0.0f, 1.0f);
                col = (int)(g * 3.999f) & 3;
                break;
            }
            default:
                continue;
            }

            f->trail_glow[idx]  = g;
            f->trail_color[idx] = (uint8_t)(col & 3);
            f->trail_glyph[idx] = (uint8_t)glyph;
        }
    }
}

typedef struct {
    Field   F;
    bool    paused;
    int     speed;
    int     drift_mult;
    int     current_theme;
    Pattern current_pattern;
    Pattern prev_pattern;       /* for switch detection / buffer wipe */
} Scene;

static void field_reset(Field *f, int w, int h)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->n_particles = N_PARTICLES_DEF;
    f->field_time = 0.0f;
    f->reset_countdown = RESET_TICKS_DEF;
    f->supernova_glow_t = 1.0f;
    field_clear(f);
    perm_shuffle();
    for (int i = 0; i < f->n_particles; i++) particle_spawn(f, &f->particles[i]);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    field_reset(&s->F, mw, mh);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->drift_mult      = DRIFT_MULT_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_PARTICLES;
    s->prev_pattern    = PATTERN_PARTICLES;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    /* Wipe buffers when switching patterns so leftover trails don't
     * ghost. (Detected by remembering last frame's pattern.) */
    if (s->current_pattern != s->prev_pattern) {
        field_clear(f);
        s->prev_pattern = s->current_pattern;
    }

    /* Supernova fade. */
    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Field drift for animated patterns. */
    f->field_time += FIELD_DRIFT * (float)s->drift_mult * dt;

    /* Pattern-specific simulation. */
    bool is_particle = (s->current_pattern == PATTERN_PARTICLES
                     || s->current_pattern == PATTERN_WARPED);

    if (is_particle) {
        /* Trail decay. */
        float decay_t = expf(-TRAIL_GLOW_DECAY * dt);
        for (int i = 0; i < f->total_cells; i++) {
            f->trail_glow[i] *= decay_t;
        }
        bool warped = (s->current_pattern == PATTERN_WARPED);
        for (int i = 0; i < f->n_particles; i++) {
            particle_step_curl(f, &f->particles[i], dt, s->speed, warped);
        }
    } else {
        field_update_static(f, s->current_pattern);
    }

    f->reset_countdown--;
    if (f->reset_countdown <= 0) {
        field_reset(f, f->w, f->h);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
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

    int gx0 = (cols - f->w) / 2;
    int gy0 = ((rows - 3) - f->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < f->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < f->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = field_idx(f, x, y);
            float ng = f->supernova_glow_t;
            float tg = f->trail_glow[idx];

            int  pair, attr;
            char glyph;

            if (ng > GLOW_THRESHOLD) {
                if (((x ^ y) & 3) != 0 && tg <= GLOW_THRESHOLD) continue;
                pair  = PAIR_SUPERNOVA;
                attr  = A_BOLD;
                glyph = '*';
            } else if (f->trail_glyph[idx] != 0
                    && tg > GLOW_THRESHOLD) {
                /* Pattern-supplied glyph (e.g. arrow from VECTOR). */
                pair  = PAIR_BAND_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = f->trail_glyph[idx];
            } else if (tg > GLYPH_HIGH_THRESH) {
                pair  = PAIR_BAND_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = '#';
            } else if (tg > GLYPH_MID_THRESH) {
                pair  = PAIR_BAND_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = '*';
            } else if (tg > GLOW_THRESHOLD) {
                pair  = PAIR_BAND_BASE + (f->trail_color[idx] & 3);
                attr  = A_NORMAL;
                glyph = '.';
            } else {
                continue;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED   "
                          : pattern_name(s->current_pattern);

    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d  reset:%4.1fs ",
             fps, sim_fps, state_str, s->speed, (double)reset_secs);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " CURL NOISE VECTOR FIELD ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;
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
    mvprintw(1, x,
             "  drift:x%-2d  eps:%.2f  map:%dx%d ",
             s->drift_mult, CURL_EPS, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " .:low  *:mid  #:high  >^v</\\:vector | n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
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
    int mh = app->screen.rows - 3;
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
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
        if (s->drift_mult < DRIFT_MULT_MAX) s->drift_mult *= 2;
        if (s->drift_mult > DRIFT_MULT_MAX) s->drift_mult = DRIFT_MULT_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
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
