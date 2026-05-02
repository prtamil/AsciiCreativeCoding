/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * simplex_noise_clouds.c
 *   — Ken Perlin's simplex noise, five cloud-themed patterns.
 *
 * DEMO: A flowing cloudscape generated from 2-D simplex noise — the
 *       improved-noise variant Perlin published in 2001 to fix the
 *       directional bias and grid artefacts of his original 1985
 *       Perlin noise. Five different patterns visualise the same
 *       noise function in different ways:
 *         CLOUDS     — plain fBm simplex (default; soft cumulus)
 *         WISPS      — anisotropic stretch — horizontal cloud streaks
 *         TURBULENCE — Σ |octaveᵢ|, sharper-edged storm clouds
 *         BILLOW     — |simplex|, puffy bubble clouds
 *         RIDGED     — 1 − |simplex|, sharp wispy cirrus ridges
 *       Field drifts so all patterns evolve. Cycle patterns with n/p,
 *       themes with t/T.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — Perlin (1985) gradient noise;
 *       the predecessor to simplex. Shape similarities; simplex is
 *       faster and less directionally biased.
 *   ./domain_warped_noise_iq_style.c — domain warping, also Perlin-
 *       based. Different "what to do with the noise" technique.
 *   ./worley_cellular_noise.c — cellular noise instead of gradient.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 noise    — Simplex 2-D + fBm
 *   §6 patterns — 5 cloud-style noise mappings
 *   §7 scene    — Field, scene state, per-frame grid update
 *   §8 screen   — ASCII render: density-graded glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new permutation)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra simplex_noise_clouds.c \
 *       -o simplex_clouds -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Simplex noise (Perlin 2001). The mathematical fix
 *                  for several issues with the original 1985 Perlin
 *                  noise:
 *                    • PERLIN samples a square lattice, requiring
 *                      bilinear interpolation across 4 corners. The
 *                      square has a directional bias — features tend
 *                      to align with axes.
 *                    • SIMPLEX samples a triangular (simplex) lattice
 *                      — in 2-D, a single triangle suffices, with
 *                      barycentric-style contribution from each of its
 *                      3 vertices. Triangles tile space isotropically,
 *                      so no axis bias.
 *                    • Faster too: 3 vertex contributions in 2-D vs 4
 *                      for Perlin, scaling better in higher dimensions
 *                      (the gap widens as N grows).
 *
 *                  The 2-D algorithm:
 *                    1. SKEW input (x, y) so unit squares become unit
 *                       triangles: skew factor F2 = (√3 − 1)/2.
 *                    2. Find which simplex (one of two triangles inside
 *                       each skewed unit square) contains the point —
 *                       trivially "lower" if x0 > y0, "upper" otherwise.
 *                    3. UNSKEW each of the 3 simplex vertices back to
 *                       Cartesian and compute (Δx, Δy) from the query.
 *                    4. For each vertex: t = 0.5 − Δ² (the radial
 *                       falloff; goes negative outside the support
 *                       circle, in which case the contribution is 0);
 *                       if t > 0, contribution = t⁴ × (gradient · Δ).
 *                    5. Sum the 3 contributions, scale by 70 so the
 *                       output range is roughly [−1, 1].
 *
 *                  Five patterns build on this primitive — see §6.
 *
 * Data-structure : Same 256-entry permutation table as Perlin, plus a
 *                  12-direction gradient table (vs Perlin's 8). Per-cell
 *                  glow + colour buffers as in the other field files.
 *                  No allocation post-init.
 *
 * Rendering      : ASCII only, density-graded ('.', '*', '#') in 4
 *                  theme palette colours. Each pattern produces glow ∈
 *                  [0, 1] which selects both the glyph (by threshold)
 *                  and the colour band (by quartile).
 *
 * Performance    : 1 simplex2() call per FBM octave per cell. CLOUDS
 *                  uses 4 octaves → 4 simplex calls per cell. With
 *                  ~11 K cells at 60 Hz that's ~2.6 M simplex calls/sec
 *                  — roughly half the cost of equivalent Perlin
 *                  thanks to the 3-vertex vs 4-vertex difference.
 *                  Fits in well under 5 % CPU on modern hardware.
 *
 * References     : • Perlin, K. (2001) — "Noise hardware":
 *                    https://www.csee.umbc.edu/~olano/s2002c36/ch02.pdf
 *                  • Stefan Gustavson — "Simplex noise demystified"
 *                    (the canonical implementation walkthrough):
 *                    https://weber.itn.liu.se/~stegu/simplexnoise/simplexnoise.pdf
 *                  • Wikipedia — "Simplex noise":
 *                    https://en.wikipedia.org/wiki/Simplex_noise
 *                  • Inigo Quilez — "Better fbm" (companion FBM
 *                    techniques applicable to either Perlin or Simplex):
 *                    https://iquilezles.org/articles/fbm/
 *                  • Compare ../generational/voronoi_region_map.c for
 *                    "discrete cells" instead of "smooth field".
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Simplex noise produces a smooth scalar field over arbitrary
 * coordinates — like Perlin noise, but built on a triangular lattice
 * instead of a square one. The result has no preferred axis; the
 * "wind" of the noise moves diagonally just as readily as
 * horizontally. For cloud-like patterns, that isotropy reads as
 * organic — clouds shouldn't have a grain.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a tiled honeycomb of equilateral triangles. Each triangle
 * vertex has a random gradient direction. To sample at a point P:
 *   1. Find which triangle P is in.
 *   2. From each of the 3 vertices, draw a vector to P, dot it with
 *      that vertex's gradient, and weight by a radial falloff.
 *   3. Sum those 3 contributions.
 * Because triangles are isotropic (no preferred axis) and only 3
 * vertices contribute (vs 4 for square Perlin), the function is
 * cheaper AND more visually neutral.
 *
 * Layered with fractional Brownian motion (sum noise at progressively
 * doubled frequency / halved amplitude), the result is a multi-scale
 * cloud field — large rolling shapes with small bumps on top.
 *
 * The 5 patterns differ in HOW they map noise to glow:
 *   • CLOUDS   : raw fBm — soft, smooth cumulus.
 *   • WISPS    : sample at non-uniform x/y scale — clouds stretch
 *                horizontally into streaks.
 *   • TURBULENCE: sum |octave| at each scale — sharper, more chaotic
 *                (this is Perlin's "turbulence" function from his
 *                marble-rendering paper).
 *   • BILLOW   : take |fbm| — bumpy puffs since negatives flip up.
 *   • RIDGED   : 1 − |fbm| — peaks where fbm crosses zero, sharp
 *                ridges visible.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Convert cell coord to noise space: (fx, fy) = (x, y) · NOISE_SCALE.
 *  2. Run the active pattern's noise function with current field_time
 *     added to fy (slow drift).
 *  3. Map noise → glow ∈ [0, 1] and colour band ∈ {0, 1, 2, 3}.
 *  4. Render: pick density glyph from glow; theme colour from band.
 *  5. Periodic supernova reset reshuffles the permutation table.
 *
 * KEY FORMULAS
 * ────────────
 *  Simplex skew (2-D)            : F2 = (√3 − 1) / 2 ≈ 0.366
 *  Simplex unskew (2-D)          : G2 = (3 − √3) / 6 ≈ 0.211
 *  Falloff per vertex            : t = 0.5 − (Δx² + Δy²)
 *                                  contribution = t⁴ · (grad · (Δx, Δy))
 *                                  (zero if t ≤ 0)
 *  Output scale                  : multiply by 70 so result is roughly
 *                                  in [−1, 1]
 *  fBm                            : Σᵢ aᵢ · simplex(2ⁱ x), aᵢ = 1/2ⁱ
 *  Turbulence                    : Σᵢ aᵢ · |simplex(2ⁱ x)|
 *  Billow                        : |fbm(x)|
 *  Ridged                        : 1 − |fbm(x)|
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SKEW MUST USE THE EXACT CONSTANTS. F2 and G2 are derived from
 *    sqrt(3); using approximations like 0.36 or 0.21 leaves visible
 *    seams at the lattice. Use the full-precision constants below
 *    (or compute them via sqrtf at startup).
 *
 *  • FALLOFF EXPONENT. The standard implementation uses t⁴; some
 *    older ones use t³. t⁴ gives smoother gradients (C² continuous),
 *    matching the upgrade from Perlin's quintic fade. Don't lower it
 *    or you'll see lattice creases.
 *
 *  • RANGE OF SIMPLEX OUTPUT. Pre-scaled, 2-D simplex returns values
 *    roughly in [−0.07, 0.07]. We multiply by 70 to get roughly
 *    [−1, 1]; the bound is empirical, not a hard guarantee. Patterns
 *    that take fabsf or 1−fabsf can occasionally overshoot — clamp
 *    on render.
 *
 *  • TURBULENCE BIAS. Σ |octaves| is always ≥ 0 (no negative values),
 *    so its mean is well above 0.5. Normalising by Σ aᵢ keeps it in
 *    [0, 1] but with mean ~ 0.5 — visually correct.
 *
 *  • WISPS ASPECT RATIO. We sample with different x/y scales (e.g.
 *    x · 0.4 vs y · 1.0) so noise features get longer in x. Too
 *    extreme an aspect (x · 0.1) and the streaks become 1-D bands;
 *    keep around 0.3–0.6 for natural-looking wisps.
 *
 *  • DRIFT VS PATTERN. CLOUDS / WISPS / BILLOW evolve smoothly with
 *    drift. RIDGED can flicker because crossing zero in fbm flips
 *    the ridge — that's the algorithm, not a bug.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • CLOUDS pattern looks like soft rolling clouds — round shapes,
 *    no axis grain. If you see vertical or horizontal streaking, the
 *    skew constants are wrong (probably F2 = 0.36 instead of full
 *    precision).
 *  • WISPS shows clearly elongated horizontal cloud streaks; the
 *    aspect ratio of features should be visible.
 *  • TURBULENCE has sharper, denser detail than CLOUDS — like
 *    storm/cumulonimbus rather than gentle cumulus.
 *  • BILLOW pattern peaks at simplex-zero crossings (you'll see
 *    valley-like dim regions where billows separate).
 *  • RIDGED pattern has sharp light-coloured ridges — verify by
 *    eye that they're THIN and CONNECTED, not blobby.
 *  • Drift visible: leave running for a few seconds; clouds should
 *    move slowly. If frozen, FIELD_DRIFT or scene_tick is wrong.
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

    /* Reset cadence in ticks. ~12 s per noise seed. */
    RESET_TICKS_DEF   = 12 * 60,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale: ~0.04 noise-units per cell → ~8 noise-units across
 * the grid → 3-4 large feature periods visible. */
#define NOISE_SCALE         0.04f

/* Field drift in noise-coord units per second. */
#define FIELD_DRIFT         0.10f

/* Drift multiplier — cranked by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* fBm stack: 4 octaves with halving amplitude / doubling frequency. */
#define FBM_OCTAVES         4

/* WISPS pattern: x-scale multiplier (smaller = longer horizontal
 * features). Y-scale stays at 1. */
#define WISPS_X_SCALE       0.40f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Simplex skew (F2) and unskew (G2) constants. Full precision
 * matters — see EDGE CASES in MENTAL MODEL. Values from
 * Stefan Gustavson's reference implementation.
 */
#define SIMPLEX_F2          0.36602540378443864f   /* (sqrt(3) - 1) / 2 */
#define SIMPLEX_G2          0.21132486540518713f   /* (3 - sqrt(3)) / 6 */
#define SIMPLEX_OUTPUT_GAIN 70.0f                  /* empirical bound for [-1,1] */

/*
 * Pattern — five cloud-style mappings of simplex noise. Cycle with n/p.
 *
 *   CLOUDS     : plain fBm — soft cumulus
 *   WISPS      : anisotropic — horizontal cloud streaks
 *   TURBULENCE : Σ |octave| — sharper, storm-like
 *   BILLOW     : |fbm| — bumpy puffs
 *   RIDGED     : 1 − |fbm| — wispy ridges (cirrus)
 */
typedef enum {
    PATTERN_CLOUDS     = 0,
    PATTERN_WISPS      = 1,
    PATTERN_TURBULENCE = 2,
    PATTERN_BILLOW     = 3,
    PATTERN_RIDGED     = 4,
    N_PATTERNS         = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_CLOUDS:     return "CLOUDS    ";
    case PATTERN_WISPS:      return "WISPS     ";
    case PATTERN_TURBULENCE: return "TURBULENCE";
    case PATTERN_BILLOW:     return "BILLOW    ";
    case PATTERN_RIDGED:     return "RIDGED    ";
    default:                 return "?         ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the rest of the procedural showcases.
 * Each defines 4 cloud-band colours plus a flash accent.
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
/* §5  noise — Simplex 2-D + fBm                                          */
/* ===================================================================== */

/*
 * Permutation table (256 entries, doubled to 512). Reshuffled at each
 * reset so different runs produce different noise fields. Same idea as
 * Perlin's perm[] but used differently — see grad2[] below.
 */
static uint8_t simplex_p[512];

/*
 * grad2[][2] — 12 gradient directions for 2-D simplex noise. The
 * standard set: edges of a regular icosahedron projected to 2-D, plus
 * extras for table-size convenience. Don't shuffle these; the 12-set
 * is balanced (sum-to-zero) and matches Perlin/Gustavson references.
 */
static const int8_t grad2[12][2] = {
    {  1,  1 }, { -1,  1 }, {  1, -1 }, { -1, -1 },
    {  1,  0 }, { -1,  0 }, {  1,  0 }, { -1,  0 },
    {  0,  1 }, {  0, -1 }, {  0,  1 }, {  0, -1 },
};

/*
 * simplex_perm_shuffle — Fisher-Yates the 0..255 sequence using rand,
 * then duplicate into simplex_p[256..511]. Called at every reset.
 */
static void simplex_perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        simplex_p[i]       = base[i];
        simplex_p[i + 256] = base[i];
    }
}

/*
 * simplex2 — 2-D simplex noise. Returns roughly [-1, 1].
 *
 * Algorithm: see CONCEPTS / MENTAL MODEL above. Implementation
 * follows Stefan Gustavson's reference verbatim.
 */
static float simplex2(float xin, float yin)
{
    /* Skew input space: square grid → triangular simplex grid. */
    float s = (xin + yin) * SIMPLEX_F2;
    int   i = (int)floorf(xin + s);
    int   j = (int)floorf(yin + s);

    float t  = (float)(i + j) * SIMPLEX_G2;
    float X0 = (float)i - t;
    float Y0 = (float)j - t;
    float x0 = xin - X0;
    float y0 = yin - Y0;

    /* Determine which simplex (lower or upper triangle). */
    int i1, j1;
    if (x0 > y0) { i1 = 1; j1 = 0; }    /* lower triangle */
    else         { i1 = 0; j1 = 1; }    /* upper triangle */

    /* Offsets to the other two simplex corners. */
    float x1 = x0 - (float)i1 + SIMPLEX_G2;
    float y1 = y0 - (float)j1 + SIMPLEX_G2;
    float x2 = x0 - 1.0f + 2.0f * SIMPLEX_G2;
    float y2 = y0 - 1.0f + 2.0f * SIMPLEX_G2;

    /* Hash gradient indices for each corner. */
    int ii  = i & 255;
    int jj  = j & 255;
    int gi0 = simplex_p[ii      + simplex_p[jj     ]] % 12;
    int gi1 = simplex_p[ii + i1 + simplex_p[jj + j1]] % 12;
    int gi2 = simplex_p[ii + 1  + simplex_p[jj + 1 ]] % 12;

    /* Per-corner contributions. The (0.5 - r²)⁴ falloff zeroes out
     * smoothly outside each corner's support disk. */
    float n0 = 0.0f, n1 = 0.0f, n2 = 0.0f;
    float t0 = 0.5f - x0 * x0 - y0 * y0;
    if (t0 > 0.0f) {
        t0 *= t0;
        n0 = t0 * t0 * ((float)grad2[gi0][0] * x0 + (float)grad2[gi0][1] * y0);
    }
    float t1 = 0.5f - x1 * x1 - y1 * y1;
    if (t1 > 0.0f) {
        t1 *= t1;
        n1 = t1 * t1 * ((float)grad2[gi1][0] * x1 + (float)grad2[gi1][1] * y1);
    }
    float t2 = 0.5f - x2 * x2 - y2 * y2;
    if (t2 > 0.0f) {
        t2 *= t2;
        n2 = t2 * t2 * ((float)grad2[gi2][0] * x2 + (float)grad2[gi2][1] * y2);
    }

    return SIMPLEX_OUTPUT_GAIN * (n0 + n1 + n2);
}

/*
 * fbm_simplex — fractional Brownian motion stack of simplex noise.
 * 4 octaves with doubling frequency / halving amplitude. Output
 * normalised by the running amplitude sum so it stays in roughly
 * [−1, 1].
 */
static float fbm_simplex(float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * simplex2(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/*
 * fbm_simplex_abs — fractional Brownian motion of |simplex| (Perlin's
 * "turbulence" function). Always ≥ 0 because of the absolute value.
 * Used by the TURBULENCE pattern — produces sharper-edged textures
 * because the |·| introduces folds at every zero crossing of the
 * underlying noise.
 */
static float fbm_simplex_abs(float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * fabsf(simplex2(x * freq, y * freq));
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — 5 cloud-style noise mappings                            */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * pattern_clouds — plain fBm, mapped to [0, 1]. Soft, smooth cumulus
 * cloud look. The baseline against which the other patterns vary.
 */
static float pattern_clouds(float x, float y, float t)
{
    return fbm_simplex(x, y + t) * 0.5f + 0.5f;
}

/*
 * pattern_wisps — anisotropic stretch. By sampling at a SMALLER x
 * scale we make horizontal features last longer in screen space —
 * cloud streaks instead of round puffs. Drift is also slightly
 * accelerated (×1.5) so wisps appear to fly past.
 */
static float pattern_wisps(float x, float y, float t)
{
    return fbm_simplex(x * WISPS_X_SCALE, y + t * 1.5f) * 0.5f + 0.5f;
}

/*
 * pattern_turbulence — Perlin's "turbulence" function: Σ aᵢ ·
 * |simplex(2ⁱx)|. Always non-negative, so its mean sits well above
 * 0.5 — we still range-map for the glow but expect a brighter,
 * denser look than CLOUDS. The |·| introduces folds at every zero
 * crossing of the underlying noise, producing the sharp-edged
 * "storm cloud" appearance.
 */
static float pattern_turbulence(float x, float y, float t)
{
    return clampf(fbm_simplex_abs(x, y + t), 0.0f, 1.0f);
}

/*
 * pattern_billow — |fbm|. Negative noise values flip up so peaks form
 * at every zero crossing of the underlying fbm. Visually: bumpy puffs
 * with valleys between them.
 */
static float pattern_billow(float x, float y, float t)
{
    return clampf(fabsf(fbm_simplex(x, y + t)), 0.0f, 1.0f);
}

/*
 * pattern_ridged — 1 − |fbm|. Inverse of BILLOW: peaks at noise zeros,
 * valleys at extremes. Looks like sharp cirrus ridges or marble veins.
 * The classic "ridged multifractal" function from Musgrave's terrain
 * synthesis work, applied here over simplex instead of Perlin.
 */
static float pattern_ridged(float x, float y, float t)
{
    return clampf(1.0f - fabsf(fbm_simplex(x, y + t)), 0.0f, 1.0f);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    int      w, h;
    int      total_cells;

    float    trail_glow [CELLS_MAX];
    uint8_t  trail_color[CELLS_MAX];
    float    supernova_glow_t;

    float    field_time;
    int      reset_countdown;
} Field;

static inline int field_idx(const Field *f, int x, int y) { return y * f->w + x; }

static void field_reset(Field *f, int w, int h)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->field_time = 0.0f;
    f->reset_countdown = RESET_TICKS_DEF;
    f->supernova_glow_t = 1.0f;
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
    }
    simplex_perm_shuffle();
}

/*
 * field_update_grid — sample the active pattern at every cell.
 */
static void field_update_grid(Field *f, Pattern p)
{
    float t = f->field_time;
    for (int y = 0; y < f->h; y++) {
        for (int x = 0; x < f->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            float g = 0.0f;
            switch (p) {
            case PATTERN_CLOUDS:     g = pattern_clouds    (fx, fy, t); break;
            case PATTERN_WISPS:      g = pattern_wisps     (fx, fy, t); break;
            case PATTERN_TURBULENCE: g = pattern_turbulence(fx, fy, t); break;
            case PATTERN_BILLOW:     g = pattern_billow    (fx, fy, t); break;
            case PATTERN_RIDGED:     g = pattern_ridged    (fx, fy, t); break;
            default:                                                    break;
            }
            if (g < 0.0f) g = 0.0f;
            if (g > 1.0f) g = 1.0f;
            int idx = field_idx(f, x, y);
            f->trail_glow[idx]  = g;
            f->trail_color[idx] = (uint8_t)((int)(g * 3.999f) & 3);
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
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->drift_mult      = DRIFT_MULT_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_CLOUDS;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    f->field_time += FIELD_DRIFT * (float)s->drift_mult * dt;

    field_update_grid(f, s->current_pattern);

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
                          ? "PAUSED    "
                          : pattern_name(s->current_pattern);

    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  drift:x%-2d  reset:%4.1fs ",
             fps, sim_fps, state_str, s->drift_mult, (double)reset_secs);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " SIMPLEX NOISE CLOUDS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + colour swatches + parameters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-10s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 21;
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
             "  scale:%.2f  oct:%d  map:%dx%d ",
             NOISE_SCALE, FBM_OCTAVES, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " .:low  *:mid  #:high | n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  q:quit ");
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
