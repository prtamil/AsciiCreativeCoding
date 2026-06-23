/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * domain_warped_noise_iq_style.c
 *
 * A noise field you can watch swirl. The trick (from Inigo Quilez) is to
 * let noise distort its own input coordinates: feed noise(x) back in as
 * noise(x + noise(x)). Do that once, twice, three times and smooth blobs
 * turn into marble and smoke. n/p cycles five patterns showing the build-up.
 *
 * The domain-warping idea and the magic offset constants come from:
 *   Inigo Quilez, "Domain warping" — https://iquilezles.org/articles/warp/
 * Sister demo using the same noise for particle motion instead of a field:
 *   ./perin_noise_flow_showcase.c
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

    /* Colour slots. The HUD reserves the first two by project convention. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* first of N_BANDS in-a-row palette slots */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* The white flash on reset fades out this fast (bigger = quicker). */
#define SUPERNOVA_DECAY     4.0f
/* Below this brightness a cell is treated as empty. */
#define GLOW_THRESHOLD      0.05f

/* How fast we step through noise-space from one cell to the next. Small,
 * so a few wide features span the whole grid instead of looking grainy. */
#define NOISE_SCALE         0.04f

/* How fast the pattern drifts on its own, in noise-units per second.
 * Slow enough to watch evolve, not so fast it churns. */
#define FIELD_DRIFT         0.10f

/* The +/- keys scale drift up and down between these limits. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* fBm = stacking several copies of noise at different scales. Each layer
 * is half as strong (persistence) and twice as fine (lacunarity) as the
 * one before. 0.5 and 2.0 are the classic values for natural-looking noise. */
#define FBM_OCTAVES         4
#define FBM_PERSISTENCE     0.5f
#define FBM_LACUNARITY      2.0f

/* How hard the warp shoves the input coordinates. Bigger = wilder. */
#define WARP_AMOUNT         4.0f

/* When the warp builds a 2-D shove vector, it reads two noise samples,
 * one per axis. These offsets push those two reads apart so the axes
 * don't end up identical (which would flatten the warp to a diagonal
 * slide). The exact numbers are Quilez's; any non-zero, non-equal pair
 * works, so don't fuss over them. */
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

/* One-off tunables, grouped so §1 lists every dial in one place. */

/* How bright the reset flash starts; it fades from here. */
#define SUPERNOVA_FLASH_INIT     1.0f

/* We sort each cell's colour into one of N_BANDS buckets. The scale is a
 * hair under N_BANDS so a colour of exactly 1.0 lands in the top bucket
 * instead of falling off the end. */
#define N_BANDS                  4
#define BAND_QUANTIZE_SCALE      3.999f

/* For RAW, we want the colour to look unrelated to the brightness, so we
 * read it from a spot in noise-space far away from the brightness sample. */
#define RAW_COLOR_DECOR_FACTOR   4.0f

/* The reset flash sparkles instead of whiting out the whole screen: this
 * mask lights roughly one cell in four, like scattered stars. */
#define SUPERNOVA_SPARSE_MASK    3

/* Same bucketing trick as the colour bands, but for the brightness ramp:
 * just under N_GLYPHS so brightness 1.0 picks the last glyph, not past it. */
#define GLYPH_QUANTIZE_SCALE     (N_GLYPHS - 0.001f)

/* The brightness ramp: blank for empty, '@' for fully lit, ASCII shades
 * in between (from Paul Bourke's grey-scale character set). Ten steps is
 * about the fewest that still looks smoothly shaded at terminal size. */
static const char glyph_ramp[] = " .:-=+*#%@";
#define N_GLYPHS         10
#define GLYPH_BOLD_FROM   5    /* glyphs this bright and up draw in bold */

/* Brightness curve. Raw noise sits mostly in the dull middle and reads as
 * muddy; this lifts the mid-tones so a half-bright value looks genuinely
 * lit. 0.65 is a hand-tuned sweet spot between flat and washed-out. */
#define INTENSITY_GAMMA  0.65f

/* HUD layout. Top three rows show info (state, parameters, ramp legend);
 * the field fills the middle; the bottom row lists the keys. */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      18    /* width of the " pattern:XXX " box */
#define HUD_THEME_FIELD_W        17    /* width of the " theme:XXX " box   */
#define HUD_PALETTE_LABEL_W       9    /* width of the " palette:" label   */
#define HUD_PALETTE_SWATCH_N      4    /* one '#' shown per band colour    */

/*
 * Pattern — which of the five looks is on screen; cycle with n/p.
 * Each step adds one more layer of warping, so you can see the effect grow.
 *
 *   RAW    : plain noise, no warp at all — the baseline to compare against
 *   WARP1  : warped once
 *   WARP2  : warped twice (Quilez's classic marble look)
 *   WARP3  : warped three times — the busiest
 *   RIDGE  : WARP2 reshaped into sharp veins instead of soft blobs
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

/* If a frame stalls badly (over 100 ms), pretend it was only 100 ms so the
 * sim doesn't try to fast-forward a huge backlog of catch-up ticks. */
#define DT_MAX_NS                (100 * NS_PER_MS)
#define FRAME_CAP_FPS            60

/*
 * Theme — one named colour scheme. t/T cycles the ten in themes[].
 * Holds exactly what's needed to recolour the screen: the band colours
 * (a low-to-high gradient) plus the colour of the reset flash.
 *
 * Each cell carries a band number 0..3 (picked from its colour channel,
 * which flows separately from brightness — see §6); band[] turns that
 * number into an actual colour. Keeping it to four nudges theme authors
 * toward a clean dark-to-bright ramp instead of a random pile of colours.
 *
 * The HUD's own colours are kept separate and never themed, so it stays
 * readable over any palette. Values are xterm-256 colour numbers, not RGB;
 * on terminals with fewer colours theme_apply() falls back to a basic set.
 */
typedef struct {
    const char *name;            /* short label shown in the HUD             */
    short       band[N_BANDS];   /* the colour ramp: 0 = darkest, 3 = brightest */
    short       flash;           /* colour of the reset flash                */
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
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
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
/* §5  noise — Noise context + Perlin 2-D + FBM                           */
/* ===================================================================== */

/*
 * Noise — the randomness behind the whole field. It's just a shuffled
 * list of 0..255 (a "permutation table") that Perlin noise uses to pick
 * a pseudo-random direction at each grid point. Re-shuffling it gives a
 * brand-new field, which is what the 'r' key does. Every sampler takes
 * it as const, so it's clear that reading noise never changes anything.
 *
 * This is Ken Perlin's classic gradient noise (2002 "Improving Noise"
 * version, which smooths out artefacts the 1985 original had).
 */
typedef struct {
    /* The shuffled table, stored twice back-to-back. Storing two copies
     * lets the sampler read perm[X+1] without worrying about running off
     * the end — a small, well-known speed trick from Perlin's code. */
    uint8_t perm[512];
} Noise;

/* Shuffle 0..255 into random order, every arrangement equally likely
 * (standard Fisher-Yates). */
static void fisher_yates_shuffle_256(uint8_t base[256])
{
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
}

/* Copy the shuffled table into the doubled buffer twice (see Noise). */
static void mirror_perm_for_double_lookup(uint8_t perm[512],
                                          const uint8_t base[256])
{
    for (int i = 0; i < 256; i++) {
        perm[i]       = base[i];
        perm[i + 256] = base[i];
    }
}

static void noise_shuffle(Noise *n)
{
    uint8_t base[256];
    fisher_yates_shuffle_256      (base);
    mirror_perm_for_double_lookup (n->perm, base);
}

/* Smooth S-shaped blend curve. Using this instead of a straight blend is
 * what keeps the noise from showing seams where grid cells meet. */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

/* Picks one of 8 fixed directions (from the low bits of the hash) and
 * measures how far the sample point lies along it. This is the per-corner
 * value that the noise blends together. */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * One sample of Perlin noise at (x, y); result lands roughly in [-1, 1].
 * The idea: find which grid square the point is in, give each of its four
 * corners a pseudo-random slope, then smoothly blend the corners together
 * weighted by how close the point is to each.
 */
static float noise_perlin2d(const Noise *n, float x, float y)
{
    /* Which grid square we're in, and where inside it. The & 255 just
     * wraps the grid coordinates so they stay in table range. */
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    /* Smooth blend weights for the position inside the square. */
    float u = fade(x);
    float v = fade(y);

    /* Look up a pseudo-random value for each side of the square. */
    int A = n->perm[X    ] + Y;
    int B = n->perm[X + 1] + Y;

    /* The four corners' contributions. */
    float n00 = grad(n->perm[A    ], x,        y       );   /* corner (0,0) */
    float n10 = grad(n->perm[B    ], x - 1.0f, y       );   /* corner (1,0) */
    float n01 = grad(n->perm[A + 1], x,        y - 1.0f);   /* corner (0,1) */
    float n11 = grad(n->perm[B + 1], x - 1.0f, y - 1.0f);   /* corner (1,1) */

    /* Blend across, then down. */
    return lerp_f(lerp_f(n00, n10, u),
                  lerp_f(n01, n11, u), v);
}

/*
 * fBm: layered noise. Adds several copies of the noise above, each finer
 * and fainter than the last, which gives both broad shapes and fine detail
 * at once. We divide by the total strength so the result stays near [-1, 1].
 */
static float noise_fbm(const Noise *n, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * noise_perlin2d(n, x * freq, y * freq);
        max_amp += amp;
        amp     *= FBM_PERSISTENCE;
        freq    *= FBM_LACUNARITY;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — 5 domain-warp variants                                  */
/* ===================================================================== */

/*
 * Sample — what each pattern hands back for one cell: two numbers in
 * [0, 1], computed from two *different* noise reads.
 *
 *   intensity → how bright the cell is (picks a glyph from the ramp)
 *   color     → which palette colour the cell uses
 *
 * Driving brightness and colour from separate noise is the whole visual
 * trick. Tie them to one value and the screen looks like a flat shaded
 * heightmap; split them and you see two patterns flowing over each other.
 *
 * Note: `intensity` is already run through the brightness curve by the
 * pattern that produced it, so the renderer can use it as-is. `color` is
 * left as plain noise — it gets sorted into a few buckets anyway, so a
 * curve wouldn't change the result.
 */
typedef struct {
    float intensity;   /* final brightness, 0..1, curve already applied */
    float color;       /* plain colour noise, 0..1                      */
} Sample;

/* Lifts dull mid-tones toward "lit" so the field looks illuminated rather
 * than muddy. See INTENSITY_GAMMA. */
static inline float gamma_boost(float v)
{
    return powf(v, INTENSITY_GAMMA);
}

/* Noise comes out roughly -1..1; this slides it into 0..1 for display. */
static inline float remap_signed_to_unit(float v)
{
    return v * 0.5f + 0.5f;
}

/* A plain (x, y) pair. Used so the warp steps read as point math rather
 * than juggling loose floats. */
typedef struct { float x, y; } Vec2;

/*
 * WarpOffsets — the four numbers that set up one warp layer. A warp layer
 * builds a 2-D shove vector by reading noise twice, once for each axis;
 * these offsets move those two reads apart so the axes don't come out
 * identical. (dx_*, the x-axis read; dy_*, the y-axis read.) The actual
 * values are Quilez's — any non-zero, non-equal pair does the job.
 */
typedef struct {
    float dx_x, dx_y;
    float dy_x, dy_y;
} WarpOffsets;

/* The three warp layers, each with its own offsets. The very first read
 * (Q's x-axis) uses zero offset, so it samples right at the input point;
 * the rest are nudged away to stay independent of each other. */
static const WarpOffsets Q_OFFSETS = { 0.0f,       0.0f,       DECOR_QY_X, DECOR_QY_Y };
static const WarpOffsets R_OFFSETS = { DECOR_RX_X, DECOR_RX_Y, DECOR_RY_X, DECOR_RY_Y };
static const WarpOffsets S_OFFSETS = { DECOR_SX_X, DECOR_SX_Y, DECOR_SY_X, DECOR_SY_Y };

/* Build one warp's 2-D shove vector at `base`, reading noise once per axis
 * (offset apart so x and y differ). `t` slides the reads over time so the
 * warp animates. */
static Vec2 sample_warp_layer(const Noise *n, Vec2 base, WarpOffsets o, float t)
{
    return (Vec2){
        .x = noise_fbm(n, base.x + o.dx_x, base.y + o.dx_y + t),
        .y = noise_fbm(n, base.x + o.dy_x, base.y + o.dy_y + t),
    };
}

/* The warp itself: nudge a point by a shove vector. This one line is the
 * heart of the whole demo — everything else just feeds it. */
static inline Vec2 warp_position(Vec2 base, Vec2 v)
{
    return (Vec2){
        base.x + WARP_AMOUNT * v.x,
        base.y + WARP_AMOUNT * v.y,
    };
}

/* The last noise read in the chain, after all the warping is done. Its
 * value becomes the cell's brightness. `t` drifts it over time. */
static inline float sample_final_intensity(const Noise *n, Vec2 p, float t)
{
    return noise_fbm(n, p.x, p.y + t);
}

/* RAW: no warping at all — the baseline. Brightness is plain noise; colour
 * is noise read from a far-off spot so it looks like an unrelated pattern. */
static Sample pattern_raw(const Noise *n, float x, float y, float t)
{
    Vec2 intensity_pos = { x,                                       y };
    Vec2 color_pos     = { x + RAW_COLOR_DECOR_FACTOR * DECOR_QY_X,
                           y + RAW_COLOR_DECOR_FACTOR * DECOR_QY_Y };

    float intensity = sample_final_intensity(n, intensity_pos, t);
    float color     = sample_final_intensity(n, color_pos,     t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(color),
    };
}

/* WARP1: warp once. Read a shove vector, nudge the point, sample there for
 * brightness. Colour comes from the shove's x-component, so it drifts on
 * its own track. */
static Sample pattern_warp1(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base, Q_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, q), t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(q.x),
    };
}

/* Warp twice — shared by WARP2 and RIDGE. Shove once, shove the result
 * again, then read brightness at the doubly-shoved point. Returns
 * brightness *before* the curve so RIDGE can apply its own. Colour comes
 * from the first shove, the broadest and clearest layer to colour by. */
static Sample warp2_raw(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base,                       Q_OFFSETS, t);
    Vec2 r          = sample_warp_layer    (n, warp_position(base, q),     R_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, r), t);

    return (Sample){
        .intensity = remap_signed_to_unit(intensity),
        .color     = remap_signed_to_unit(q.x),
    };
}

/* WARP2: Quilez's classic marble look — just warp2_raw plus the
 * brightness curve. */
static Sample pattern_warp2(const Noise *n, float x, float y, float t)
{
    Sample s = warp2_raw(n, x, y, t);
    s.intensity = gamma_boost(s.intensity);
    return s;
}

/* WARP3: warp three times — same idea as WARP2 with one more shove on top.
 * Colour comes from the last (busiest) shove this time, which makes WARP3's
 * colours noticeably more frantic. */
static Sample pattern_warp3(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base,                       Q_OFFSETS, t);
    Vec2 r          = sample_warp_layer    (n, warp_position(base, q),     R_OFFSETS, t);
    Vec2 s          = sample_warp_layer    (n, warp_position(base, r),     S_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, s), t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(s.x),
    };
}

/* RIDGE: turn the soft WARP2 blobs into sharp veins. Make the brightest
 * point be wherever the noise crosses its mid-value, then square it to
 * pinch those bright spots into thin lines — the marble / cracked-stone
 * look. No brightness curve here; the squaring already shapes it, and a
 * curve on top would just grey out the dark background. */
static Sample pattern_ridge(const Noise *n, float x, float y, float t)
{
    Sample s = warp2_raw(n, x, y, t);    /* brightness before any curve */
    float v = s.intensity - 0.5f;
    float ridge = 1.0f - 2.0f * fabsf(v);
    ridge = ridge * ridge;                /* pinch into thin veins */
    return (Sample){
        .intensity = ridge,
        .color     = s.color,
    };
}

/* Pick the active pattern. The only place a Pattern turns into a Sample,
 * so the grid loop in §7 never has to know which one is running. */
static Sample sample_pattern(const Noise *n, Pattern p,
                              float fx, float fy, float t)
{
    switch (p) {
    case PATTERN_RAW:   return pattern_raw  (n, fx, fy, t);
    case PATTERN_WARP1: return pattern_warp1(n, fx, fy, t);
    case PATTERN_WARP2: return pattern_warp2(n, fx, fy, t);
    case PATTERN_WARP3: return pattern_warp3(n, fx, fy, t);
    case PATTERN_RIDGE: return pattern_ridge(n, fx, fy, t);
    default:            return (Sample){ 0.0f, 0.0f };
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is five small structs, each owning one job. Splitting them
 * this way makes function signatures honest: a function taking const Grid *
 * plainly can't change the buffers, and so on.
 */

/*
 * Grid — how big the field is, in cells. Just dimensions, no storage.
 * Cells are numbered row by row: (x, y) lives at y*w + x, which matches how
 * the render buffers are laid out, so looping row-by-row stays cache-friendly.
 * w * h never exceeds CELLS_MAX — app_pick_map_size() clamps it on resize,
 * and everything downstream relies on that.
 */
typedef struct {
    int w, h;         /* field width / height in cells       */
    int total_cells;  /* w * h, kept around to skip the multiply in hot loops */
} Grid;

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }

/*
 * RenderBuffers — the finished picture, one entry per cell. This is the
 * only thing the screen reads: the patterns fill it in, the screen draws
 * it, and nothing else passes between them. Two side-by-side arrays (rather
 * than one array of structs) because the screen reads brightness far more
 * than colour, and clearing one without the other is easy.
 */
typedef struct {
    /* How bright each cell is, 0..1. Below GLOW_THRESHOLD it draws blank.
     * Rewritten every frame — no fade-out, since each frame repaints fully. */
    float   glow [CELLS_MAX];

    /* Which colour band each cell uses, 0..N_BANDS-1. Masked in the draw
     * loop so a stray value can never pick a colour slot that isn't set. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->color[i] = 0;
    }
}

/*
 * SimState — what the animation changes on its own each tick, kept apart
 * from Controls (the user's knobs) so it's clear who changes what.
 *   field_time       → slowly slides where we sample the noise, which is
 *                      what makes the pattern drift and evolve over time.
 *   supernova_glow_t → the brief white flash after a reset; fades to zero.
 */
typedef struct {
    /* Added to the noise y-coordinate before sampling. Grows a little each
     * tick (faster with drift_mult), which is what makes the field move.
     * Reset to 0 by scene_reset(). */
    float field_time;

    /* How strong the reset flash is right now. Starts at SUPERNOVA_FLASH_INIT
     * and fades each tick; while it's above GLOW_THRESHOLD the screen shows
     * scattered sparkles. */
    float supernova_glow_t;
} SimState;

/*
 * Controls — everything the keyboard sets. The handler writes these and
 * the rest of the program only reads them; keeping them in their own struct
 * is what lets the key handler stay a tidy list of one-line changes.
 */
typedef struct {
    /* When true the field freezes, but drawing keeps going so the HUD
     * still responds. */
    bool    paused;

    /* How fast the field drifts. +/- double and halve it (so each press
     * feels like the same size step), clamped to [DRIFT_MULT_MIN, MAX]. */
    int     drift_mult;

    /* Which colour scheme is active; t/T step through themes[]. */
    int     current_theme;

    /* Which pattern is on screen; n/p step through them. No ghosting to
     * worry about — each frame repaints the whole field. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — the whole program state in one place. Reading the fields top to
 * bottom is the quickest tour of how it works:
 *   grid   → how big the field is
 *   noise  → the randomness it's drawn from
 *   buf    → the finished picture
 *   sim    → what the animation changes by itself
 *   ctrl   → what the user changes
 * The order is deliberate: each field only leans on the ones above it.
 */
typedef struct {
    Grid          grid;       /* only resize changes this           */
    Noise         noise;      /* only a reset (reshuffle) changes this */
    RenderBuffers buf;        /* patterns write it, the screen reads it */
    SimState      sim;        /* only scene_tick changes this       */
    Controls      ctrl;       /* only the key handler changes this  */
} Scene;

/* Repaint the whole field: for each cell, sample the active pattern at that
 * spot (scaled into noise-space and offset by drift time) and store the
 * brightness and colour. */

static inline float clamp_unit(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

/* Sort a colour value 0..1 into one of the palette buckets. */
static inline int quantize_color_to_band(float color)
{
    return (int)(color * BAND_QUANTIZE_SCALE) & (N_BANDS - 1);
}

/* Store one cell's sample into the picture. */
static inline void write_cell(RenderBuffers *b, int idx, Sample s)
{
    b->glow [idx] = clamp_unit(s.intensity);
    b->color[idx] = (uint8_t)quantize_color_to_band(clamp_unit(s.color));
}

static void scene_update_grid(Scene *s)
{
    const Grid    *g  = &s->grid;
    const Noise   *n  = &s->noise;
    RenderBuffers *b  = &s->buf;
    Pattern        p  = s->ctrl.current_pattern;
    float          t  = s->sim.field_time;

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            Sample sample = sample_pattern(n, p, fx, fy, t);
            write_cell(b, grid_idx(g, x, y), sample);
        }
    }
}

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time       = 0.0f;
    sim->supernova_glow_t = SUPERNOVA_FLASH_INIT;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions(&s->grid, w, h);
    reset_sim_state      (&s->sim);
    buffers_clear        (&s->buf, s->grid.total_cells);
    noise_shuffle        (&s->noise);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused          = false;
    s->ctrl.drift_mult      = DRIFT_MULT_DEF;
    s->ctrl.current_theme   = 0;
    s->ctrl.current_pattern = PATTERN_WARP2;   /* start on the classic look */
    scene_reset(s, w, h);
}

/* One simulation step: fade the reset flash, nudge the drift forward, and
 * repaint the field. Does nothing while paused. The noise is only ever
 * reshuffled by a manual reset, never here. */

static void decay_supernova_flash(Scene *s, float dt)
{
    s->sim.supernova_glow_t *= expf(-SUPERNOVA_DECAY * dt);
}

static void advance_field_time(Scene *s, float dt)
{
    s->sim.field_time += FIELD_DRIFT * (float)s->ctrl.drift_mult * dt;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_supernova_flash (s, dt);
    advance_field_time    (s, dt);
    scene_update_grid     (s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, refreshed whenever the window
 * resizes. We only keep the size; ncurses tracks everything else. Used to
 * place the HUD and centre the field.
 */
typedef struct {
    int cols;   /* terminal width,  in characters */
    int rows;   /* terminal height, in characters */
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
 * CellDraw — a little "here's what to paint at one spot" note: which
 * colour, which attributes, which character, or skip it entirely. The
 * per-cell deciders fill one of these out, and paint_cell is the single
 * place that actually talks to ncurses. Splitting the decision from the
 * drawing keeps all the terminal calls in one spot.
 *
 * `skip` means leave the cell untouched, which is different from drawing a
 * blank space: untouched lets the previous frame show through. That's what
 * makes the reset sparkles look like scattered stars instead of a wash.
 */
typedef struct {
    int  pair;   /* which colour slot to use   */
    int  attr;   /* bold or normal             */
    char glyph;  /* the character to draw       */
    bool skip;   /* true = draw nothing here    */
} CellDraw;

/* Find the top-left corner so the field sits centred, with room left for
 * the HUD rows above and below. Clamped so it never overlaps the HUD on a
 * short terminal. */
static void compute_centred_origin(const Grid *g, int cols, int rows,
                                    int *out_gx0, int *out_gy0)
{
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;
    *out_gx0 = gx0;
    *out_gy0 = gy0;
}

/* During the reset flash, light a scattered subset of cells as sparkles,
 * plus any cell that's already bright so the field underneath stays visible. */
static CellDraw cell_supernova_sparkle(int x, int y, float trail_glow)
{
    bool sparkle_lit = ((x ^ y) & SUPERNOVA_SPARSE_MASK) == 0;
    if (!sparkle_lit && trail_glow <= GLOW_THRESHOLD)
        return (CellDraw){ .skip = true };
    return (CellDraw){ .pair = PAIR_SUPERNOVA, .attr = A_BOLD, .glyph = '*' };
}

/* Turn a brightness 0..1 into a position on the glyph ramp, clamped. */
static inline int quantize_glow_to_glyph_index(float glow)
{
    int gi = (int)(glow * GLYPH_QUANTIZE_SCALE);
    if (gi < 0)         gi = 0;
    if (gi >= N_GLYPHS) gi = N_GLYPHS - 1;
    return gi;
}

/* The normal case: pick a glyph from the cell's brightness, in its band
 * colour, bold once it's bright enough. Blank cells are skipped, not drawn. */
static CellDraw cell_density_band(uint8_t band, float glow)
{
    int gi = quantize_glow_to_glyph_index(glow);
    if (gi == 0) return (CellDraw){ .skip = true };
    return (CellDraw){
        .pair  = PAIR_BAND_BASE + (band & (N_BANDS - 1)),
        .attr  = (gi >= GLYPH_BOLD_FROM) ? A_BOLD : A_NORMAL,
        .glyph = glyph_ramp[gi],
    };
}

/* Decide what to draw at one cell: a reset sparkle if the flash is going,
 * otherwise the normal brightness glyph. */
static CellDraw pick_cell(const Scene *s, int x, int y)
{
    int   idx        = grid_idx(&s->grid, x, y);
    float trail_glow = s->buf.glow[idx];

    if (s->sim.supernova_glow_t > GLOW_THRESHOLD)
        return cell_supernova_sparkle(x, y, trail_glow);

    return cell_density_band(s->buf.color[idx], trail_glow);
}

/* The one place that actually draws a field cell to the terminal. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);

    for (int y = 0; y < s->grid.h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < s->grid.w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            paint_cell(sy, sx, pick_cell(s, x, y));
        }
    }
}

/* The HUD: top rows show what's going on (state, settings, the ramp key);
 * the bottom row lists the keys. Each piece has its own small drawer. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED "
                                      : pattern_name(c->current_pattern);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  drift:x%-2d ",
             fps, sim_fps, state_str, c->drift_mult);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DOMAIN WARP (IQ STYLE) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Show the current palette as one '#' per band colour. Returns the column
 * just after it so the caller can keep laying out the row. */
static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < HUD_PALETTE_SWATCH_N; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* The settings row is laid out left to right: each drawer paints its bit
 * starting at column x and returns where the next one should start. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-7s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_PATTERN_FIELD_W;
}

static int draw_status_theme_field(int row, int x, int theme_idx)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_THEME_FIELD_W;
}

static int draw_status_palette_label(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + HUD_PALETTE_LABEL_W;
}

/* Last bit of the row: the noise settings and current field size. */
static void draw_status_noise_params(int row, int x, const Grid *g)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x,
             "  scale:%.2f  warp:%.1f  oct:%d  map:%dx%d ",
             NOISE_SCALE, WARP_AMOUNT, FBM_OCTAVES, g->w, g->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    int x = HUD_LEFT_MARGIN;
    x = draw_status_pattern_field (1, x, c->current_pattern);
    x = draw_status_theme_field   (1, x, c->current_theme);
    x = draw_status_palette_label (1, x);
    x = draw_palette_swatch       (1, x);
        draw_status_noise_params  (1, x, &s->grid);
}

/* Show the dim-to-bright character key so the viewer can read the field. */
static void draw_hud_ramp_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " ramp:  .  :  -  =  +  *  #  %%  @   dim -> bright ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the list of keys. */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                         double fps, int sim_fps)
{
    erase();
    scene_draw            (s, sc->cols, sc->rows);   /* the field, drawn first */
    draw_hud_state_bar    (sc, s, fps, sim_fps);     /* HUD painted on top */
    draw_hud_title        ();
    draw_hud_status_line  (s);
    draw_hud_ramp_legend  ();
    draw_bottom_hint      (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — everything the program owns: the scene, the screen, the tick rate
 * and chosen field size, plus two flags the OS signal handlers set. There's
 * a single instance (g_app) so the handlers can reach it. The handlers only
 * flip a flag; the main loop notices and does the real work in its own time,
 * which is the safe way to handle signals.
 */
typedef struct {
    Scene                 scene;   /* the simulation              */
    Screen                screen;  /* current terminal size       */

    int                   sim_fps; /* tick rate; [ and ] change it */
    int                   map_w;   /* field width,  capped at MAP_W_MAX */
    int                   map_h;   /* field height, capped at MAP_H_MAX */

    /* Set by signal handlers, read by the main loop. The qualifiers make
     * that hand-off safe across the two contexts. */
    volatile sig_atomic_t running;       /* set to 0 to quit            */
    volatile sig_atomic_t need_resize;   /* set to 1 when the window resized */
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
    screen_resize    (&app->screen);
    app_pick_map_size(app);
    scene_reset      (&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* Speed the drift up or down. Doubling/halving rather than +/-1 so each
 * press feels like the same size change whatever the current speed. */
static void bump_drift_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->drift_mult < DRIFT_MULT_MAX) c->drift_mult *= 2;
        if (c->drift_mult > DRIFT_MULT_MAX) c->drift_mult = DRIFT_MULT_MAX;
    } else {
        c->drift_mult /= 2;
        if (c->drift_mult < DRIFT_MULT_MIN) c->drift_mult = DRIFT_MULT_MIN;
    }
}

/* Nudge the tick rate up or down, kept within limits. */
static void bump_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void cycle_theme(Controls *c, int dir)
{
    c->current_theme = (c->current_theme + dir + N_THEMES) % N_THEMES;
    theme_apply(c->current_theme);
}

static void cycle_pattern(Controls *c, int dir)
{
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                            break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);  break;
    case '=': case '+': bump_drift_geometric(c, +1);                       break;
    case '-':           bump_drift_geometric(c, -1);                       break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                  break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                  break;
    case 't':           cycle_theme  (c, +1);                              break;
    case 'T':           cycle_theme  (c, -1);                              break;
    case 'n': case 'N': cycle_pattern(c, +1);                              break;
    case 'p': case 'P': cycle_pattern(c, -1);                              break;
    default: break;
    }
    return true;
}

static void install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* How long since the last frame, capped so one slow frame can't snowball
 * (see DT_MAX_NS). Also updates the stored "last frame" time. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned. Keeping
 * each step the same length makes the animation behave the same at any
 * frame rate. */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the fps reading once every FPS_UPDATE_MS; otherwise keep the
 * last one so the HUD number doesn't flicker. */
static double maybe_update_fps_counter(int64_t *fps_accum,
                                        int *frame_count,
                                        double previous)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return previous;
    double fps = (double)(*frame_count) /
                  ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
    return fps;
}

/* Sleep off whatever's left of this frame's time budget so we don't run
 * faster than target_fps. */
static void cap_frame_rate(int64_t work_done_ns, int target_fps)
{
    int64_t budget = NS_PER_SEC / target_fps;
    clock_sleep_ns(budget - work_done_ns);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init      (&app->screen);
    app_pick_map_size(app);
    scene_init       (&app->scene, app->map_w, app->map_h);

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

        int64_t dt      = advance_frame_clock(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        simulate_pending_ticks(app, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum  += dt;
        fps_display = maybe_update_fps_counter(&fps_accum, &frame_count, fps_display);

        cap_frame_rate((clock_ns() - frame_time) + dt, FRAME_CAP_FPS);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
