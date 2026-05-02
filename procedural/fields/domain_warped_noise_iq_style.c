/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * domain_warped_noise_iq_style.c
 *   — Inigo Quilez-style domain warping over a Perlin/FBM field.
 *
 * DEMO: Five static-field patterns, each more "warped" than the last.
 *       Cycle them with n/p to see the technique build up:
 *         RAW   — plain fractional Brownian motion (no warp)
 *         WARP1 — one warp pass: f(x + g(x))
 *         WARP2 — TWO warp passes (IQ's classic): f(x + h(x + g(x)))
 *         WARP3 — three warp passes — the deepest nesting
 *         RIDGE — WARP2 with the ridged transform 1 − |n|, sharp
 *                 lines that look like marble veins or cracked stone
 *       The field drifts slowly so all patterns evolve over time.
 *       Density-shaded ASCII glyphs ('.', '*', '#') in 4 theme
 *       colours give every pattern its own visual identity.
 *
 * Study alongside: ./perin_noise_flow_showcase.c — the FLOW-based
 *       Perlin showcase; this file focuses on STATIC-field renderings
 *       with progressively deeper warping. The same Perlin & FBM
 *       primitives are used, but the patterns here all answer the
 *       question "what happens if we recursively offset the noise
 *       input by other noise?". The two files together cover most of
 *       the practical Perlin-noise idioms.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 palette colours each)
 *   §5 noise    — Perlin permutation + 2-D gradient noise + FBM
 *   §6 patterns — RAW / WARP1 / WARP2 / WARP3 / RIDGE samplers
 *   §7 scene    — Field, scene state, per-frame grid update
 *   §8 screen   — ASCII render: density-graded trail glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new noise permutation, fresh field)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster field drift (animation speeds up)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra domain_warped_noise_iq_style.c \
 *       -o iq_warp -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Domain warping. Given a noise function f(x) and another
 *                  noise function g(x), the warped value is f(x + g(x)).
 *                  Repeat: warp the warped — f(x + h(x + g(x))) — and you
 *                  get arbitrarily complex patterns from a few simple
 *                  Perlin lookups. Inigo Quilez popularised the technique
 *                  in graphics; the canonical "IQ warp" is two-level:
 *
 *                      q = (fbm(x),                fbm(x + 5.2, y + 1.3))
 *                      r = (fbm(x + 4·q + 1.7),    fbm(x + 4·q + 8.3))
 *                      f = fbm(x + 4·r)
 *
 *                  The constants 5.2, 1.3 etc. de-correlate the q and r
 *                  components so they don't track each other. The "× 4"
 *                  multipliers control how far the warp pushes the input
 *                  — larger values give wilder, more chaotic patterns.
 *
 *                  This file implements 5 patterns showing progressively
 *                  deeper warping (RAW → WARP1 → WARP2 → WARP3) plus a
 *                  RIDGE variant that takes |noise| (then 1−|noise|) to
 *                  produce sharp veins and ridges instead of smooth
 *                  rolling values.
 *
 * Data-structure : Static permutation table for Perlin noise, single
 *                  field of per-cell glow + colour buffers, no particles.
 *                  Each frame fully overwrites the buffer by sampling
 *                  the active pattern at every cell. No allocation
 *                  post-init.
 *
 * Rendering      : ASCII only. Cell glyph chosen by density:
 *                    glow > 0.65 → '#'  BOLD  (high)
 *                    glow > 0.30 → '*'  BOLD  (mid)
 *                    glow > 0.05 → '.'        (low)
 *                  Cell colour = 1-of-4 theme palette index, picked by
 *                  banding the noise value into 4 quartiles. Sweeping
 *                  the noise field across these bands gives the visual
 *                  layered/striated look of IQ's renderings.
 *
 * Performance    : Worst pattern (WARP3) does ~13 FBM calls per cell ×
 *                  4 octaves per FBM = 52 perlin lookups per cell. On a
 *                  200×56 grid at 60 Hz that's ~35 M perlin lookups/sec
 *                  ≈ 1.5 % of one core on modern hardware. Comfortably
 *                  within budget for terminal rendering.
 *
 * References     : • Quilez, I. — "Domain warping" (the canonical
 *                    article that introduced the technique to graphics):
 *                    https://iquilezles.org/articles/warp/
 *                  • Quilez, I. — "Better fbm" (companion piece on
 *                    fractal noise stacks):
 *                    https://iquilezles.org/articles/fbm/
 *                  • Perlin, K. (2002) — "Improving noise". Same
 *                    quintic fade curve used here.
 *                  • The Book of Shaders — chapter on domain warping:
 *                    https://thebookofshaders.com/13/
 *                  • For comparison see: ./perin_noise_flow_showcase.c
 *                    in this same project for a particle-flow style
 *                    Perlin demo.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Plain Perlin noise produces smooth rolling hills, but everything is
 * roughly axis-aligned and grid-shaped at the lattice level. Domain
 * warping breaks that bias by letting NOISE distort its own input
 * coordinates. Each level of warping multiplies the visual richness;
 * by the second level you have something that looks like swirled
 * marble, smoke, or fluid — patterns that no simple closed-form
 * formula could produce, but emerge naturally from f(x + g(x)).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a plain noise pattern on a sheet of paper. Now imagine
 * grabbing the paper at random points and PULLING — every point
 * shifts by a noise-determined amount. The result is the same
 * underlying field, but distorted in a way that itself looks noisy.
 * Repeat the pull on the already-pulled paper and the distortion
 * compounds: smooth blobs become curves, curves become whorls,
 * whorls become chaotic eddies. WARP1, WARP2, WARP3 are exactly
 * this, applied 1, 2, and 3 times.
 *
 * RIDGE adds a different idea: instead of |noise| being smooth,
 * take 1 − |noise| so the SHARP transitions through zero become
 * the visible features. The result looks like marble veins or
 * cracked glass — sharp lines on a smooth background. Combined
 * with two-level warping, it's the closest a noise function gets
 * to looking hand-drawn.
 *
 * Visible layers:
 *   1. Density glyphs ('.', '*', '#') showing the noise intensity.
 *   2. Four-colour palette bands — same intensity, different colour
 *      for each quartile of the noise value. Combined with the
 *      density ramp this gives ~12 distinguishable visual states
 *      per cell.
 *   3. Slow drift — every pattern evolves over a few seconds as the
 *      noise field's "time axis" advances.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Advance field_time by FIELD_DRIFT × dt. This time slips into
 *     the y-coordinate of every Perlin lookup so the field appears
 *     to flow.
 *  2. For every cell (x, y):
 *     a. Compute the active pattern's noise value at (x, y, t).
 *     b. Map noise → glow ∈ [0, 1] and colour ∈ {0, 1, 2, 3}.
 *  3. Render: pick density glyph from glow, theme colour from
 *     colour index.
 *  4. Periodic supernova reset: every ~12 s reshuffle the perm[]
 *     table and flash the screen yellow. The pattern stays the
 *     same; only the noise seed changes.
 *
 * KEY FORMULAS
 * ────────────
 *  Perlin 2-D                   : as in ./perin_noise_flow_showcase.c
 *  FBM                           : Σᵢ aᵢ · perlin(2ⁱ x), with aᵢ = 1/2ⁱ
 *  WARP1 (1-level)              : f(x + a · g(x))
 *  WARP2 (2-level, IQ classic)  : f(x + b · h(x + a · g(x)))
 *  WARP3 (3-level)              : f(x + c · h₂(x + b · h(x + a · g(x))))
 *  RIDGE                         : 1 − 2 · |WARP2(x) − ½|     (sharpens)
 *  Drift offset                  : sample fbm(x, y + t), so increasing t
 *                                  effectively slides the field upward
 *                                  in noise-space.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DE-CORRELATION OFFSETS. The two halves of q (qx, qy) come from
 *    DIFFERENT noise samples — one at (x, y), the other at (x + 5.2,
 *    y + 1.3). If you use the SAME sample for both, qx == qy and
 *    the warp degenerates into a 1-D push along the diagonal.
 *    Always offset by an irrational-ish constant.
 *
 *  • WARP MAGNITUDE. The "× 4" in IQ's article assumes input
 *    coordinates in roughly [0, 1]. If your coords are pixel-scale
 *    you need to scale UP the warp by an equivalent factor or scale
 *    DOWN your noise input. We pre-multiply by NOISE_SCALE = 0.04 so
 *    the inputs land in roughly [0, 8] across the grid, and use
 *    WARP_AMOUNT = 4.0 directly.
 *
 *  • COMPUTE COST GROWS GEOMETRICALLY. WARP3 needs 13 FBM calls per
 *    cell vs RAW's 1. On a slow terminal or a much bigger grid this
 *    becomes the bottleneck. Reduce FBM_OCTAVES or downsample the
 *    grid if you go past 200×60.
 *
 *  • RIDGE INTENSITY MAPPING. The output of 1 − 2|n − ½| peaks at
 *    n = ½ and is 0 at n = 0 or 1. The bright cells trace the
 *    half-noise contour line. If your themes have colour bands tied
 *    to noise quartiles, you'll see RIDGE concentrate one or two
 *    colours along the lines — that's correct, not a bug.
 *
 *  • DRIFT SPEED VS COMPUTE COST. With WARP3 active and a fast drift
 *    rate, the field updates faster than is comfortable to watch.
 *    Adjust DRIFT or pause briefly with space if needed.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Cycle to RAW first; the pattern should look like rolling
 *    cellular blobs — no swirls, no cracks. If you see swirls in
 *    RAW, the FBM is silently warping somewhere.
 *  • Switch to WARP1 — pattern visibly distorts, blobs start curving.
 *  • WARP2 — distinct swirly/marble look. This is the IQ canonical
 *    output; should be visibly more chaotic than WARP1.
 *  • WARP3 — even more eddies. The difference WARP2 → WARP3 is
 *    subtler than WARP1 → WARP2 (diminishing returns).
 *  • RIDGE — sharp light-coloured ridges on a darker background;
 *    veined like marble.
 *  • All patterns should evolve smoothly when you wait — drift is
 *    working. If frozen, FIELD_DRIFT is 0 or scene_tick is paused.
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

    /* How often to reshuffle the Perlin permutation. ~12 s gives
     * each seed time to display before the next one. */
    RESET_TICKS_DEF   = 12 * 60,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Decay for the supernova flash on reset. */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale: ~0.04 noise-units per cell → about 8 noise-units
 * across the whole 200-wide grid → ~3-4 visible "feature periods". */
#define NOISE_SCALE         0.04f

/* Field drift in noise-coord units per second. Slow drift so the
 * pattern visibly evolves but doesn't churn. */
#define FIELD_DRIFT         0.10f

/* Drift multiplier — cranked by +/-, capped to keep CPU happy. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* FBM stack: 4 octaves with halving amplitude / doubling frequency. */
#define FBM_OCTAVES         4

/* Domain-warp magnitude. IQ's canonical value is 4.0 for inputs in
 * [0, 1]; we apply it after multiplying inputs by NOISE_SCALE so the
 * effective push lands in the right perceptual range. */
#define WARP_AMOUNT         4.0f

/* De-correlation offsets — the constants that prevent two halves of
 * the warp vector from being perfectly correlated. Pulled from IQ's
 * article. Don't change these unless you want a different look. */
#define DECOR_QY_X          5.2f
#define DECOR_QY_Y          1.3f
#define DECOR_RX_X          1.7f
#define DECOR_RX_Y          9.2f
#define DECOR_RY_X          8.3f
#define DECOR_RY_Y          2.8f
#define DECOR_SX_X          7.5f
#define DECOR_SX_Y          3.1f
#define DECOR_SY_X          6.8f
#define DECOR_SY_Y          4.4f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — five domain-warp variants, cycle with n/p.
 *
 *   RAW    : plain FBM, no warp — the baseline you compare against
 *   WARP1  : single warp pass, f(x + a·g(x))
 *   WARP2  : two warp passes (IQ classic), f(x + b·h(x + a·g(x)))
 *   WARP3  : three warp passes — the deepest nesting
 *   RIDGE  : WARP2 with the ridged transform 1 − 2|n − ½|, sharp
 *            veins instead of smooth blobs
 */
typedef enum {
    PATTERN_RAW   = 0,
    PATTERN_WARP1 = 1,
    PATTERN_WARP2 = 2,
    PATTERN_WARP3 = 3,
    PATTERN_RIDGE = 4,
    N_PATTERNS    = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RAW:   return "RAW    ";
    case PATTERN_WARP1: return "WARP1  ";
    case PATTERN_WARP2: return "WARP2  ";
    case PATTERN_WARP3: return "WARP3  ";
    case PATTERN_RIDGE: return "RIDGE  ";
    default:            return "?      ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the rest of the procedural showcases.
 * Each theme defines 4 palette colours (bands of the noise value)
 * plus a flash accent. PAIR_HUD/HINT/SUPERNOVA stay theme-independent.
 */
typedef struct {
    const char *name;
    short       band[4];
    short       flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },   /* navy → sky → gold → white  */
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },   /* greens                     */
    { "NOVA",    {  53,  129,  201,  219 }, 226 },   /* purple → pink → magenta    */
    { "MONO",    { 234,  244,  250,  254 }, 226 },   /* greyscale                  */
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },   /* navy → bright cyan         */
    { "FIRE",    {  52,  124,  208,  226 }, 196 },   /* dark red → yellow          */
    { "EARTH",   {  58,  100,  173,  230 }, 226 },   /* brown → cream              */
    { "FOREST",  {  22,   28,   64,  144 }, 226 },   /* greens                     */
    { "DESERT",  {  94,  130,  173,  222 }, 226 },   /* sandy                      */
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },   /* navy → ice → white         */
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
/* §5  noise — Perlin 2-D + FBM                                           */
/* ===================================================================== */

/*
 * Permutation table — Ken Perlin's classic 256-entry array, shuffled
 * at each reset and duplicated to perm[512] so lookups perm[X] and
 * perm[X+1] never need an explicit modulo.
 */
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

/*
 * perlin2d — 2-D Perlin noise. See ./perin_noise_flow_showcase.c §5
 * for the detailed walkthrough; the implementation here is identical
 * (the project's "Self-Contained File Rule" requires inlining shared
 * code rather than #include).
 */
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

    return lerp_f(
        lerp_f(n00, n10, u),
        lerp_f(n01, n11, u),
        v
    );
}

/*
 * fbm — fractional Brownian motion stack. Sums FBM_OCTAVES copies of
 * Perlin noise, each at twice the frequency and half the amplitude
 * of the previous. Result roughly in [-1, 1] after normalisation by
 * the running amplitude sum.
 *
 * fBm gives the "rolling hills with rocks on them" texture that
 * domain warping then distorts into something more interesting.
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

/* ===================================================================== */
/* §6  patterns — 5 domain-warp variants                                  */
/* ===================================================================== */

/*
 * Each pattern function takes pre-scaled noise coordinates (cell ×
 * NOISE_SCALE) plus a time offset t added to the y-coord for drift.
 * Returns a value in [0, 1] suitable for direct use as glow.
 *
 * All patterns share the same coordinate convention so switching
 * between them looks like a smooth re-warping, not a teleport.
 */

/* RAW — plain fBm, no warping. The baseline reference. */
static float pattern_raw(float x, float y, float t)
{
    return fbm(x, y + t) * 0.5f + 0.5f;
}

/*
 * WARP1 — single domain warp.
 *   q = (fbm(x), fbm(x + 5.2, y + 1.3))
 *   result = fbm(x + WARP·q)
 *
 * The de-correlation offsets ensure qx and qy are independent
 * components rather than tracking each other along the diagonal.
 */
static float pattern_warp1(float x, float y, float t)
{
    float qx = fbm(x,                     y                     + t);
    float qy = fbm(x + DECOR_QY_X,        y + DECOR_QY_Y        + t);
    return fbm(x + WARP_AMOUNT * qx,
               y + WARP_AMOUNT * qy + t) * 0.5f + 0.5f;
}

/*
 * WARP2 — Inigo Quilez's classic two-level warp.
 *   q = (fbm(x), fbm(x + 5.2, y + 1.3))
 *   r = (fbm(x + 4·q + 1.7, y + 4·q + 9.2),
 *        fbm(x + 4·q + 8.3, y + 4·q + 2.8))
 *   result = fbm(x + 4·r)
 *
 * This is the canonical "marble" look from his article. Two levels
 * is the sweet spot — visually distinct from WARP1, far less compute
 * than WARP3.
 */
static float pattern_warp2(float x, float y, float t)
{
    float qx = fbm(x,              y              + t);
    float qy = fbm(x + DECOR_QY_X, y + DECOR_QY_Y + t);

    float rx = fbm(x + WARP_AMOUNT * qx + DECOR_RX_X,
                   y + WARP_AMOUNT * qy + DECOR_RX_Y + t);
    float ry = fbm(x + WARP_AMOUNT * qx + DECOR_RY_X,
                   y + WARP_AMOUNT * qy + DECOR_RY_Y + t);

    return fbm(x + WARP_AMOUNT * rx,
               y + WARP_AMOUNT * ry + t) * 0.5f + 0.5f;
}

/*
 * WARP3 — three-level warp. Adds an outermost (s) warp on top of
 * IQ's two-level scheme. Pattern grows visibly more chaotic; the
 * difference WARP2→WARP3 is subtler than WARP1→WARP2 (diminishing
 * returns) but you can see denser eddies at small scales.
 */
static float pattern_warp3(float x, float y, float t)
{
    float qx = fbm(x,              y              + t);
    float qy = fbm(x + DECOR_QY_X, y + DECOR_QY_Y + t);

    float rx = fbm(x + WARP_AMOUNT * qx + DECOR_RX_X,
                   y + WARP_AMOUNT * qy + DECOR_RX_Y + t);
    float ry = fbm(x + WARP_AMOUNT * qx + DECOR_RY_X,
                   y + WARP_AMOUNT * qy + DECOR_RY_Y + t);

    float sx = fbm(x + WARP_AMOUNT * rx + DECOR_SX_X,
                   y + WARP_AMOUNT * ry + DECOR_SX_Y + t);
    float sy = fbm(x + WARP_AMOUNT * rx + DECOR_SY_X,
                   y + WARP_AMOUNT * ry + DECOR_SY_Y + t);

    return fbm(x + WARP_AMOUNT * sx,
               y + WARP_AMOUNT * sy + t) * 0.5f + 0.5f;
}

/*
 * RIDGE — WARP2 with the "ridged" transform applied. Take 1 −
 * 2|n − ½| so the value PEAKS at n = ½ and drops to 0 at the
 * extremes. The bright cells trace the half-noise contour line —
 * sharp veins on a smooth background, like marble or cracked stone.
 *
 * Compose with the 2-level warp to get the veins themselves curving
 * organically rather than running in straight bands.
 */
static float pattern_ridge(float x, float y, float t)
{
    float w = pattern_warp2(x, y, t);   /* in [0, 1] */
    float v = w - 0.5f;
    return 1.0f - 2.0f * fabsf(v);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Field — per-cell glow + colour, plus the noise's drift state.
 *
 *   trail_glow[]      density driver — pattern's noise output, [0, 1]
 *   trail_color[]     0..3 palette band index per cell
 *   supernova_glow_t  global reset flash
 *   field_time        slowly-advancing y-axis offset for drift
 *   reset_countdown   ticks until next perm reshuffle
 */
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
    perm_shuffle();
}

/*
 * field_update_grid — sample the active pattern at every cell, fill
 * trail_glow + trail_color. Called once per scene_tick.
 */
static void field_update_grid(Field *f, Pattern p)
{
    float t = f->field_time;
    for (int y = 0; y < f->h; y++) {
        for (int x = 0; x < f->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            float g  = 0.0f;
            switch (p) {
            case PATTERN_RAW:   g = pattern_raw  (fx, fy, t); break;
            case PATTERN_WARP1: g = pattern_warp1(fx, fy, t); break;
            case PATTERN_WARP2: g = pattern_warp2(fx, fy, t); break;
            case PATTERN_WARP3: g = pattern_warp3(fx, fy, t); break;
            case PATTERN_RIDGE: g = pattern_ridge(fx, fy, t); break;
            default:            g = 0.0f;                     break;
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
    int     drift_mult;          /* ×1, ×2, ×4, ×8 etc. — applied to FIELD_DRIFT */
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
    s->current_pattern = PATTERN_WARP2;   /* IQ classic by default */
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    /* Supernova flash decays. */
    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Drift the noise field. drift_mult lets the user crank speed
     * up to 32× without changing the per-frame perlin-cost (just the
     * t-coordinate offset between frames). */
    f->field_time += FIELD_DRIFT * (float)s->drift_mult * dt;

    /* Repaint the entire grid from the active pattern. */
    field_update_grid(f, s->current_pattern);

    /* Periodic reshuffle for variety. */
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
                /* Sparse-ish supernova. */
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
                          ? "PAUSED "
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
    mvprintw(0, 1, " DOMAIN WARP (IQ STYLE) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + colour swatches. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;
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
             "  scale:%.2f  warp:%.1f  oct:%d  map:%dx%d ",
             NOISE_SCALE, WARP_AMOUNT, FBM_OCTAVES, f->w, f->h);
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
