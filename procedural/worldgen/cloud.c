/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloud.c
 *   — Procedural cloud layers from a stack of fractional-Brownian-
 *     motion (fBm) noise samples.  Each cell of the sky asks "how
 *     dense is the cloud here?", the answer comes from one or more
 *     fBm evaluations, and the cell is drawn at a glyph + colour
 *     intensity that maps to that density.  Wind drifts the noise
 *     coordinates over time, so the clouds slide across the screen
 *     forever.
 *
 * DEMO: A sky.  Soft white shapes drift slowly from left to right.
 *       Cycle four cloud morphologies with n / p:
 *
 *         CUMULUS    big puffy clouds — single fBm sampled through a
 *                    domain-warp pre-pass so the silhouettes look
 *                    organic instead of mathematically clean.
 *         CIRRUS     thin high-altitude wisps — single high-frequency
 *                    fBm with anisotropic scaling (compressed in y)
 *                    that smears every blob horizontally.
 *         STRATUS    flat low-altitude layered bands — low-frequency
 *                    fBm with vertical compression so features form
 *                    long horizontal streaks.
 *         STACKED    cirrus + cumulus + stratus rendered TOGETHER in
 *                    altitude order; where all three pile up densely,
 *                    occasional '/' lightning bolts strike between
 *                    high and low layers.
 *
 *       'r' reseeds the Perlin permutation table — same algorithm,
 *       new cloud arrangement.  No phase machine, no rebuild cycle:
 *       the wind just keeps blowing.
 *
 * Study alongside:
 *   ../fields/perin_noise_flow_showcase.c
 *      — Perlin / fBm reference. The `fbm2` here is the same scaffold,
 *        parameterised by octave count so each pattern picks the
 *        amount of detail it needs.
 *   ../worldgen/hydraulic.c
 *      — uses the same fBm field as a HEIGHT MAP that gets carved.
 *        Here the field is a DENSITY MAP that gets thresholded into
 *        glyphs. Two domains, one noise scaffold.
 *
 * Section map:
 *   §1 config    — wind, scales, thresholds, themes, glyph tables
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 cloud     — hash, perlin, fbm with octave count, domain warp,
 *                  per-pattern density functions, level helper
 *   §6 scene     — wind / drift state, scene_tick (just advances time)
 *   §7 screen    — four pattern renderers + HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the wind
 *   r          reseed (new cloud arrangement, same wind)
 *   n / N      next pattern  (CUMULUS → CIRRUS → STRATUS → STACKED)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster wind
 *   -          slower wind
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/cloud.c \
 *     -o cloud -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *
 *                  (1) FRACTIONAL BROWNIAN MOTION (fBm). A single
 *                      Perlin-noise sample is a smooth scalar field
 *                      with one characteristic feature size. STACK
 *                      several Perlin samples at doubled frequency
 *                      and halved amplitude, sum them, and you get
 *                      detail at every scale — the classic fBm
 *                      formula:
 *                          f(x, y) = Σ_k a_k · perlin(x·2^k, y·2^k)
 *                                     where a_k = 0.5^k
 *                      Normalise to roughly [0, 1] and threshold:
 *                      every cell becomes opaque (cloud) or
 *                      transparent (clear sky) depending on whether
 *                      the field exceeds a chosen threshold.
 *
 *                  (2) MORPHOLOGY VIA SAMPLING SHAPE. The cloud TYPE
 *                      is encoded in HOW you sample fBm:
 *                        CUMULUS  — moderate frequency, isotropic
 *                                   scaling, plus a domain-warp
 *                                   pre-pass (sample fBm to OFFSET
 *                                   the input coords of another fBm)
 *                                   to make the silhouettes swirly.
 *                        CIRRUS   — high frequency, ANISOTROPIC
 *                                   scaling: scale_y ≫ scale_x so
 *                                   features look stretched
 *                                   horizontally.
 *                        STRATUS  — low frequency, anisotropic with
 *                                   y stretched a different amount
 *                                   to give long flat banded layers.
 *                        STACKED  — three independent fBm fields
 *                                   sampled at the same cell, one
 *                                   per altitude band, rendered in
 *                                   z-order so cirrus shows in
 *                                   gaps of cumulus, cumulus shows
 *                                   in gaps of stratus.
 *
 *                  Plus DRIFT — every sample's input coords get a
 *                  time-dependent offset (wind), so the cloud field
 *                  appears to scroll. Different layers drift at
 *                  different speeds; high-altitude cirrus moves
 *                  slightly faster than ground-level stratus, the
 *                  classic parallax-of-the-sky effect.
 *
 * Data-structure : NONE for the clouds. Just a couple of floats —
 *                  total time, wind offset, pattern index. The whole
 *                  sky is a function of (cell_x, cell_y, time);
 *                  re-evaluated every frame, no caching, no
 *                  allocation. Rendering at 240×80 × 60 fps takes
 *                  on the order of one Perlin call per cell per
 *                  frame, well under 1 % of a current CPU.
 *
 * Rendering      : ASCII only. Per cell: compute density, find which
 *                  of five LEVELS it falls into (clear / wispy / thin
 *                  / dense / core), pick a glyph from a per-pattern
 *                  table, render in the matching ramp tint. Glyphs
 *                  vary by pattern so the same density value reads
 *                  differently depending on cloud type:
 *                    CUMULUS  thin '.' / dense 'o' / core '@'
 *                    CIRRUS   thin '`' / dense '~' / core '-'
 *                    STRATUS  thin ',' / dense '_' / core '#'
 *                    STACKED  draws each layer's glyph in turn
 *                  STACKED additionally fires occasional '/' bolts
 *                  in PAIR_HOT where all three layers are dense
 *                  simultaneously — the storm-cell condition.
 *
 * Performance    : The full screen is recomputed each frame. With
 *                  3 fBm calls per cell (CUMULUS warp pass) at 4
 *                  octaves each, that is ~12 Perlin samples / cell.
 *                  At 19 200 cells × 60 fps × 50 ns per sample ≈ 60
 *                  ms / sec, around 6 % of one core. STACKED uses 5
 *                  fBm calls per cell — about 10 %. No allocation in
 *                  steady state.
 *
 * References     :
 *   • Perlin, K. (1985)  An Image Synthesizer, SIGGRAPH 1985.
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Mandelbrot, B. (1968)  Fractional Brownian motions.
 *     SIAM Review 10:422-437. (where fBm comes from.)
 *   • Inigo Quilez — "Domain warping"
 *     https://iquilezles.org/articles/warp/   (the cumulus pre-pass.)
 *   • Lague, S. — "Coding Adventure: Procedural Worlds"
 *     https://www.youtube.com/watch?v=eaXk97ujbPQ
 *   • Wikipedia — Cloud / Cumulus / Cirrus / Stratus
 *     https://en.wikipedia.org/wiki/Cloud
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To draw a cloud, do not draw a cloud — draw the THRESHOLD of a
 * smooth scalar field. Fractional Brownian motion gives you a scalar
 * field that has interesting features at every scale; ask "is this
 * field over 0.6 here?" and the YES regions look exactly like clouds,
 * because clouds in nature are exactly that: regions where some
 * scalar (water-vapor density) exceeds a threshold. The MORPHOLOGY
 * (puffy vs wispy vs banded) is just the SHAPE you sample the field
 * at — isotropic spheres, horizontally stretched ovals, vertically
 * stacked bands.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sheet of cotton ball stuffing. It has a lumpy density —
 * thicker in some places, thinner in others, with structure at many
 * scales (small fibres clump into tufts which clump into wads). Now
 * cut a 2-D cross-section through it. The lumps you see are the
 * density's THRESHOLD CONTOURS — places where the cotton crosses some
 * fixed threshold. Slide a window over the cross-section: it looks
 * like clouds drifting.
 *
 * That's literally the algorithm. fBm is the cotton-density function;
 * thresholding turns it into binary "cloud / not-cloud"; soft
 * thresholding (multiple bands) gives shaded glyphs. The wind is just
 * the cross-section sliding over the cotton.
 *
 * Different cloud TYPES come from how you tilt and squeeze the
 * cotton sheet. Stretch it horizontally → the lumps become long
 * streaks → CIRRUS. Squash it vertically → the lumps become flat
 * pancakes → STRATUS. Leave it alone (and add a domain-warp jiggle
 * for organicness) → CUMULUS.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. PERMUTATION TABLE — shuffle perm[256] from a seed and
 *     duplicate to 512 (Perlin's standard trick to skip the modulo
 *     in lookup).
 *  2. EACH FRAME:
 *     a. Advance wind: wind_x += WIND_X · dt · speed_mul
 *                      wind_y += WIND_Y · dt · speed_mul
 *                      warp_t += DRIFT  · dt · speed_mul
 *     b. For every screen cell (sx, sy):
 *         i. Compute density per the active pattern:
 *              CUMULUS   (3 fBm calls — 2 for warp, 1 for main)
 *              CIRRUS    (1 fBm call,  high freq, x-stretched)
 *              STRATUS   (1 fBm call,  low freq,  y-stretched bands)
 *              STACKED   (3 fBm calls, three altitude layers)
 *        ii. Convert density → level via thresholds.
 *       iii. If level > 0:
 *              glyph = PATTERN_GLYPHS[pattern][level - 1]
 *              ramp  = PATTERN_RAMP_LEVEL[pattern][level - 1]
 *              attr  = A_DIM / A_NORMAL / A_BOLD by level
 *              mvaddch (sy, sx, glyph)
 *  3. STACKED-only post-processing: where the cumulus AND stratus
 *     fields BOTH exceed STORM_THRESH, occasional '/' lightning
 *     glyphs are placed on a sparse hash gate.
 *
 * KEY FORMULAS
 * ────────────
 *  fBm with N octaves:
 *     f(x, y) = (1/A) · Σ_{k=0..N-1} a_k · perlin(x·2^k, y·2^k)
 *     A       = Σ a_k = 1 + 0.5 + 0.25 + … (so f ∈ roughly [-1, 1])
 *     We re-map to [0, 1] with f' = 0.5·f + 0.5.
 *
 *  Cumulus density (domain warp pre-pass):
 *     qx = fbm(x · WARP_S, y · WARP_S + warp_t)
 *     qy = fbm(x · WARP_S + 5.2, y · WARP_S + 1.3 + warp_t)
 *     d  = fbm((x + qx · WARP_AMT + wind_x) · CUMULUS_S,
 *              (y + qy · WARP_AMT + wind_y) · CUMULUS_S · ASPECT_Y)
 *
 *  Cirrus density (anisotropic, high-frequency):
 *     d = fbm((x + wind_x · 1.4) · CIRRUS_SX,
 *             (y + wind_y      ) · CIRRUS_SY)        // scale_y > scale_x
 *
 *  Stratus density (anisotropic, low-frequency):
 *     d = fbm((x + wind_x · 0.7) · STRATUS_SX,
 *             (y + wind_y      ) · STRATUS_SY)
 *
 *  Density → level (5 buckets per pattern):
 *     d ≤ T_thin  → 0   (clear sky, do not draw)
 *     ≤ T_med    → 1   (wispy edge)
 *     ≤ T_dense  → 2   (medium body)
 *     ≤ T_peak   → 3   (dense)
 *     >          → 4   (core / highlight)
 *
 *  STACKED lightning gate:
 *     storm = (d_cumulus > T_storm) AND (d_stratus > T_storm)
 *     bolt  = storm AND (hash3(x, y, ⌊30·t⌋) % 4000 == 0)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • ASPECT RATIO. Terminal cells are 2× taller than wide. Multiply
 *    sy by ASPECT_Y_F when sampling fBm so cumulus blobs render as
 *    visual circles, not horizontal ovals. The CIRRUS pattern wants
 *    THE OPPOSITE — a tall scale_y so features are stretched
 *    horizontally — so it bypasses the aspect correction.
 *
 *  • WIND CONTINUITY. wind_x grows monotonically with time. After
 *    days it can exceed float precision (~10^7 cells of drift
 *    before single-precision Perlin starts to bin into the same
 *    integer cells). For a real long-running demo, modulo wind_x
 *    by perm size periodically; for an interactive showcase that
 *    rarely runs more than minutes, ignore.
 *
 *  • DOMAIN WARP COST. Each warped cumulus cell does 3 fBm calls
 *    (qx, qy, main). At 60 fps × 240×80 cells × 4 octaves that is
 *    ~14 M Perlin samples per second. Modern hardware handles it
 *    easily, but if you ever need to scale to 8 K terminals, drop
 *    the warp passes to 2 octaves.
 *
 *  • FLOAT-VS-INT IN FLOOR. Perlin's permutation lookup uses
 *    `(int)floorf(x) & 255`. For NEGATIVE x, (int) cast truncates
 *    toward zero — wrong direction; floorf rounds toward −∞ — right
 *    direction. Don't replace floorf with a plain (int) cast or the
 *    cloud field becomes discontinuous at x = 0.
 *
 *  • ANISOTROPIC SCALES NEED ADJUSTING THRESHOLDS. CIRRUS uses a
 *    much higher scale_y than scale_x. The fBm output range is the
 *    same, but the spatial distribution of high values changes —
 *    you typically need a slightly LOWER threshold for cirrus to
 *    get a similar sky-fill ratio.
 *
 *  • LAYER DRAW ORDER IN STACKED. Render BACK to FRONT (cirrus →
 *    cumulus → stratus). If you draw stratus first and let cirrus
 *    overdraw, the high cirrus appears IN FRONT of the low stratus —
 *    the wrong altitude order — and the depth illusion breaks.
 *
 *  • LIGHTNING DENSITY. The storm gate (cumulus AND stratus both
 *    dense) is rare; the hash modulo (% 4000) is rarer; their
 *    product gives ≈ 1 lightning glyph per ~50 frames. Tune the hash
 *    modulo if you want more / fewer flashes; the 30·t in the seed
 *    means flashes regenerate at 30 Hz so they don't stick.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). Clouds freeze in place — no drift, no animation.
 *    Resume: clouds slide from exactly where they froze. Verifies
 *    the wind accumulator.
 *
 *  • Press 'r'. The cloud arrangement reshapes (different perm
 *    table) but the wind speed and pattern are unchanged. Verifies
 *    perm reseed.
 *
 *  • CUMULUS pattern. Clouds should look ROUND-ish, with organic
 *    swirly outlines (the domain-warp signature). If they look like
 *    perfect blobs with no swirl, the warp pre-pass is missing.
 *
 *  • CIRRUS pattern. Clouds should be obviously HORIZONTAL streaks,
 *    much wider than tall. If they look round like cumulus, the
 *    anisotropic scale is wrong.
 *
 *  • STRATUS pattern. Clouds form horizontal BANDS that span much
 *    of the screen width. If you see localised blobs instead, the
 *    y-scale isn't compressed enough.
 *
 *  • STACKED pattern. Three glyph types should be visible in the
 *    SAME view at different cells: '~' (cirrus high), 'o'/'O'
 *    (cumulus mid), '#' (stratus low). Where stratus and cumulus
 *    are BOTH dense, occasional '/' bolts flash in red.
 *
 *  • Wind direction. Watch a cloud edge for a few seconds; it
 *    should slide leftward-to-rightward at speed proportional to
 *    the user's "speed" knob. + key doubles the rate; − halves it.
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
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 cloud-density tints   */
    PAIR_HOT            =  11,    /* lightning bolt accent            */
    PAIR_COLD           =  12,    /* unused but reserved              */
    PAIR_FLASH          =  13,    /* reseed flash                     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Cell aspect — terminal cells are ~2× taller than wide.  Multiplies
 * y in fBm sampling so cumulus blobs render visually round on screen.
 * CIRRUS / STRATUS DELIBERATELY override this with their own y-scale
 * to produce horizontally-stretched features. */
#define ASPECT_Y_F           2.0f

/* Wind — base velocities at SPEED_DEF. The user's speed knob scales
 * these linearly. */
#define WIND_X_BASE          6.0f      /* cells / sec */
#define WIND_Y_BASE          0.4f
#define WARP_DRIFT_BASE      0.05f     /* warp-noise time evolution */

/* fBm octave counts — chosen per pattern for desired detail-vs-cost. */
#define OCT_WARP             2         /* domain-warp pre-pass        */
#define OCT_CUMULUS          4
#define OCT_CIRRUS           3
#define OCT_STRATUS          3
#define OCT_STACK_HIGH       3         /* cirrus layer in STACKED     */
#define OCT_STACK_MID        4         /* cumulus layer in STACKED    */
#define OCT_STACK_LOW        3         /* stratus layer in STACKED    */

/* Pattern-specific noise scales. */
#define CUMULUS_SCALE        0.025f
#define CUMULUS_WARP_SCALE   0.012f
#define CUMULUS_WARP_AMT     3.0f

#define CIRRUS_SCALE_X       0.060f
#define CIRRUS_SCALE_Y       0.180f    /* >> scale_x → horizontal smear */

#define STRATUS_SCALE_X      0.018f
#define STRATUS_SCALE_Y      0.060f    /* >  scale_x → flat bands       */

/* STACKED layer scales. */
#define STK_HIGH_SCALE_X     0.060f
#define STK_HIGH_SCALE_Y     0.180f
#define STK_MID_SCALE        0.030f
#define STK_LOW_SCALE_X      0.020f
#define STK_LOW_SCALE_Y      0.060f

/* Storm threshold — when cumulus AND stratus both exceed this in
 * STACKED, lightning becomes possible at that cell. */
#define STORM_THRESH         0.62f
#define LIGHTNING_HASH_MOD   4000u

/* Pattern. */
typedef enum {
    PATTERN_CUMULUS = 0,
    PATTERN_CIRRUS  = 1,
    PATTERN_STRATUS = 2,
    PATTERN_STACKED = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_CUMULUS: return "CUMULUS";
    case PATTERN_CIRRUS:  return "CIRRUS ";
    case PATTERN_STRATUS: return "STRATUS";
    case PATTERN_STACKED: return "STACKED";
    default:              return "?      ";
    }
}

/*
 * Density level — five buckets [0..4].  Level 0 = clear sky (do not
 * draw), 4 = bright cloud core. Each pattern picks four THRESHOLDS
 * and four GLYPHS for the four non-zero levels.
 */
#define N_LEVELS 5

typedef struct {
    float thresholds[4];
    char  glyphs    [4];
    int   ramp      [4];        /* index into the 8-step theme ramp  */
} CloudPattern;

static const CloudPattern PATTERNS[N_PATTERNS] = {
    /* CUMULUS — round puffy. Higher thresholds (clear → wisp → core). */
    { { 0.50f, 0.62f, 0.74f, 0.85f },
      { '.',   'o',   'O',   '@'   },
      {  3,     5,     6,     7    } },
    /* CIRRUS — wispy stretched. Slightly lower thresholds. */
    { { 0.48f, 0.58f, 0.66f, 0.76f },
      { '`',   '~',   '~',   '-'   },
      {  3,     4,     5,     6    } },
    /* STRATUS — flat bands. Soft band edges. */
    { { 0.42f, 0.52f, 0.62f, 0.74f },
      { ',',   '_',   '~',   '#'   },
      {  3,     4,     5,     7    } },
    /* STACKED — uses its own per-layer rendering, but provides
     * fallback CUMULUS-like settings for the central cumulus layer. */
    { { 0.50f, 0.62f, 0.74f, 0.85f },
      { '.',   'o',   'O',   '@'   },
      {  3,     5,     6,     7    } },
};

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Every entry sits in the bright half of the 256-colour space so
 * even A_DIM cells stay legible against a default-black terminal.
 * See "Theme Palette Brightness" in /CLAUDE.md.
 *
 * ramp[0..7]  cool → warm / dim → bright; cloud-density mapping uses
 *             slots 3..7 so the four cloud levels are visually well-
 *             separated from clear sky.
 * hot         lightning accent (STACKED storm bolts).
 * cold        reserved for future weather features.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    short       hot;
    short       cold;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7   hot cold */
    { "DEFAULT",{ 24,  31,  67, 110, 153, 195, 231, 255 }, 226,  39 },
    { "MATRIX", { 28,  34,  40,  76, 118, 154, 192, 230 }, 226,  39 },
    { "NOVA",   { 60,  91, 134, 165, 207, 219, 225, 231 }, 226,  39 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226,  39 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 226,  21 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226,  21 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 226,  39 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 226,  39 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 226,  21 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 226,  39 },
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
        init_pair(PAIR_HOT,  t->hot,  -1);
        init_pair(PAIR_COLD, t->cold, -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_WHITE,COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,  COLOR_YELLOW, -1);
        init_pair(PAIR_COLD, COLOR_CYAN,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  cloud — hash, perlin, fbm, density, level                          */
/* ===================================================================== */

/* hash3 — same routine as other showcases. Used by the lightning gate. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* Perlin scaffold — same as other showcases. */
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

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
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

/*
 * fbm2_n — fBm with a parameterised octave count. Each octave doubles
 * the frequency and halves the amplitude. Output is normalised to
 * roughly [0, 1].
 */
static float fbm2_n(float x, float y, int octaves)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* ----------------------------------------------------------------------- *
 * Density samplers — one per pattern.                                      *
 * ----------------------------------------------------------------------- */

/*
 * cumulus_density — domain-warped fBm. Two cheap fBm samples build
 * an offset vector (qx, qy) that distorts the input coords of a
 * third fBm. This breaks the "perfect blob" look of plain Perlin and
 * produces swirly cumulus silhouettes.
 */
static float cumulus_density(float x, float y, float wind_x, float wind_y, float warp_t)
{
    float qx = fbm2_n(x * CUMULUS_WARP_SCALE,
                      y * CUMULUS_WARP_SCALE + warp_t, OCT_WARP);
    float qy = fbm2_n((x + 5.2f) * CUMULUS_WARP_SCALE,
                      (y + 1.3f) * CUMULUS_WARP_SCALE + warp_t, OCT_WARP);
    float wx = (x + qx * CUMULUS_WARP_AMT + wind_x);
    float wy = (y + qy * CUMULUS_WARP_AMT + wind_y);
    return fbm2_n(wx * CUMULUS_SCALE,
                  wy * CUMULUS_SCALE * ASPECT_Y_F, OCT_CUMULUS);
}

/* High-frequency, anisotropic — feature size much smaller in y so
 * blobs read as horizontal streaks. */
static float cirrus_density(float x, float y, float wind_x, float wind_y)
{
    return fbm2_n((x + wind_x * 1.4f) * CIRRUS_SCALE_X,
                  (y + wind_y       ) * CIRRUS_SCALE_Y, OCT_CIRRUS);
}

/* Low-frequency, mildly anisotropic — long flat bands. */
static float stratus_density(float x, float y, float wind_x, float wind_y)
{
    return fbm2_n((x + wind_x * 0.7f) * STRATUS_SCALE_X,
                  (y + wind_y       ) * STRATUS_SCALE_Y, OCT_STRATUS);
}

/* STACKED layer densities — sampled at slightly different scales /
 * speeds so each layer has its own visual identity. */
static float stack_high_density(float x, float y, float wind_x)
{
    return fbm2_n((x + wind_x * 1.4f) * STK_HIGH_SCALE_X,
                  y * STK_HIGH_SCALE_Y, OCT_STACK_HIGH);
}
static float stack_mid_density(float x, float y, float wind_x, float wind_y, float warp_t)
{
    float qx = fbm2_n(x * CUMULUS_WARP_SCALE,
                      y * CUMULUS_WARP_SCALE + warp_t, OCT_WARP);
    float qy = fbm2_n((x + 5.2f) * CUMULUS_WARP_SCALE,
                      (y + 1.3f) * CUMULUS_WARP_SCALE + warp_t, OCT_WARP);
    return fbm2_n((x + qx * CUMULUS_WARP_AMT + wind_x) * STK_MID_SCALE,
                  (y + qy * CUMULUS_WARP_AMT + wind_y) * STK_MID_SCALE * ASPECT_Y_F,
                  OCT_STACK_MID);
}
static float stack_low_density(float x, float y, float wind_x, float wind_y)
{
    return fbm2_n((x + wind_x * 0.7f) * STK_LOW_SCALE_X,
                  (y + wind_y       ) * STK_LOW_SCALE_Y, OCT_STACK_LOW);
}

/* ----------------------------------------------------------------------- *
 * Density → level conversion.                                             *
 * ----------------------------------------------------------------------- */

/*
 * density_to_level — bin a density value (in roughly [0, 1]) into one
 * of five levels using the active pattern's thresholds. Level 0 is
 * "clear sky"; level 4 is "bright core".
 */
static inline int density_to_level(float d, const CloudPattern *cp)
{
    if (d < cp->thresholds[0]) return 0;
    if (d < cp->thresholds[1]) return 1;
    if (d < cp->thresholds[2]) return 2;
    if (d < cp->thresholds[3]) return 3;
    return 4;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   wind_x;
    float   wind_y;
    float   warp_t;
    float   flash_t;
} Scene;

static void scene_reseed(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          (int)(s->wind_x * 100.0f), 0xC0FFEE);
    perm_shuffle(seed);
    s->flash_t = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_CUMULUS;
    perm_shuffle(0xC0FFEE);
    s->flash_t         = 1.0f;
}

/*
 * scene_tick — advance time and wind. No regen; clouds drift forever.
 * The user can press 'r' to reseed the perm table for a new layout.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->wind_x += WIND_X_BASE * speed_mul * dt;
    s->wind_y += WIND_Y_BASE * speed_mul * dt;
    s->warp_t += WARP_DRIFT_BASE * speed_mul * dt;
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols, rows;
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

/*
 * draw_cell — common path. Pick attr from level (DIM/NORMAL/BOLD).
 */
static inline void draw_cell(int sy, int sx, char glyph, int pair, int level)
{
    int attr;
    if      (level >= 4) attr = A_BOLD;
    else if (level >= 3) attr = A_BOLD;
    else if (level >= 2) attr = A_NORMAL;
    else                 attr = A_DIM;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* ----------------------------------------------------------------------- *
 * Pattern renderers.                                                       *
 * ----------------------------------------------------------------------- */

static void render_simple(const Screen *sc, const Scene *s,
                          float (*density_fn)(float, float, float, float),
                          const CloudPattern *cp)
{
    int top = 2, bottom = sc->rows - 1;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            float d = density_fn((float)sx, (float)sy, s->wind_x, s->wind_y);
            int level = density_to_level(d, cp);
            if (level == 0) continue;
            char glyph = cp->glyphs[level - 1];
            int  pair  = PAIR_RAMP_BASE + cp->ramp[level - 1];
            draw_cell(sy, sx, glyph, pair, level);
        }
    }
}

static void render_cumulus(const Screen *sc, const Scene *s)
{
    const CloudPattern *cp = &PATTERNS[PATTERN_CUMULUS];
    int top = 2, bottom = sc->rows - 1;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            float d = cumulus_density((float)sx, (float)sy,
                                      s->wind_x, s->wind_y, s->warp_t);
            int level = density_to_level(d, cp);
            if (level == 0) continue;
            char glyph = cp->glyphs[level - 1];
            int  pair  = PAIR_RAMP_BASE + cp->ramp[level - 1];
            draw_cell(sy, sx, glyph, pair, level);
        }
    }
}

static void render_cirrus(const Screen *sc, const Scene *s)
{
    render_simple(sc, s, cirrus_density, &PATTERNS[PATTERN_CIRRUS]);
}

static void render_stratus(const Screen *sc, const Scene *s)
{
    render_simple(sc, s, stratus_density, &PATTERNS[PATTERN_STRATUS]);
}

/*
 * render_stacked — three layers in altitude order. Cirrus drawn first
 * (high, faint), cumulus on top (mid), stratus last (low, opaque) so
 * the visual depth matches real sky stratification.
 *
 * Each layer uses its own fixed thresholds + glyph + ramp slot so all
 * three are visually distinct in the same view.
 */
static void render_stacked(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    int lightning_t = (int)(s->time_secs * 30.0f);

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            float d_high = stack_high_density((float)sx, (float)sy, s->wind_x);
            float d_mid  = stack_mid_density ((float)sx, (float)sy,
                                              s->wind_x, s->wind_y, s->warp_t);
            float d_low  = stack_low_density ((float)sx, (float)sy,
                                              s->wind_x, s->wind_y);

            char glyph;
            int  pair;
            int  level;
            bool drew = false;

            /* Stratus first if dense enough (lowest layer, frontmost). */
            if (d_low > 0.62f) {
                glyph = (d_low > 0.78f) ? '#' : (d_low > 0.70f) ? '~' : '_';
                level = (d_low > 0.78f) ? 4 : (d_low > 0.70f) ? 3 : 2;
                pair  = PAIR_RAMP_BASE + 4 + (level - 2);
                drew  = true;
            }
            /* Else cumulus. */
            else if (d_mid > 0.55f) {
                glyph = (d_mid > 0.78f) ? '@' : (d_mid > 0.68f) ? 'O' : 'o';
                level = (d_mid > 0.78f) ? 4 : (d_mid > 0.68f) ? 3 : 2;
                pair  = PAIR_RAMP_BASE + 5 + (level - 2);
                drew  = true;
            }
            /* Else cirrus (high, faintest). */
            else if (d_high > 0.55f) {
                glyph = (d_high > 0.70f) ? '-' : '~';
                level = (d_high > 0.70f) ? 2 : 1;
                pair  = PAIR_RAMP_BASE + 3 + level;
                drew  = true;
            } else {
                continue;
            }

            /* Storm-cell lightning: cumulus AND stratus both dense
             * → sparse hash-gated '/' bolt in PAIR_HOT. */
            if (d_mid > STORM_THRESH && d_low > STORM_THRESH) {
                if ((hash3(sx, sy, lightning_t) % LIGHTNING_HASH_MOD) == 0u) {
                    glyph = '/';
                    pair  = PAIR_HOT;
                    level = 4;
                }
            }

            if (drew) draw_cell(sy, sx, glyph, pair, level);
        }
    }
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    switch (s->current_pattern) {
    case PATTERN_CUMULUS: render_cumulus(sc, s); break;
    case PATTERN_CIRRUS:  render_cirrus (sc, s); break;
    case PATTERN_STRATUS: render_stratus(sc, s); break;
    case PATTERN_STACKED: render_stacked(sc, s); break;
    case N_PATTERNS:      break;        /* unreachable sentinel */
    }

    /* Reseed flash. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = 2; sy < sc->rows - 1; sy += 2) {
            for (int sx = 0; sx < sc->cols; sx += 2) {
                if (((sx ^ sy ^ seed) & 7) == 0)
                    mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    /* Row 0 right — primary status. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " CLOUD LAYERS (fBm) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + theme + ramp swatch + wind. */
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
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    static const char ramp_glyphs[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ramp_glyphs[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  wind:%6.1f,%5.1f  warp:%5.2f ",
             (double)s->wind_x, (double)s->wind_y, (double)s->warp_t);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:wind  ]/[:tickHz  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
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
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

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
    scene_init(&app->scene);

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
