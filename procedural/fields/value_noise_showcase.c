/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * value_noise_showcase.c — value noise, the 1980s ancestor of Perlin noise.
 *
 * Drop a random number at every grid corner, then blend between corners to
 * fill in the spaces. That blending is the whole trick. The demo runs 30
 * patterns built on top of it (cycle with n/p) so you can watch what plain
 * "random numbers + blending" can make, and where it shows its seams: value
 * noise leaks faint grid lines along the corners that Perlin's later gradient
 * noise gets rid of.
 *
 * Sister files that show the next steps:
 *   ./perin_noise_flow_showcase.c — Perlin 1985 gradient noise, the direct
 *       successor. It fixes the grid bias you can see in this file's SMOOTH /
 *       LINEAR / BLOCKY patterns.
 *   ./simplex_noise_clouds.c — simplex noise (Perlin 2001), a triangular grid.
 *
 * Reference: Williams 1983 "Pyramidal parametrics" and Perlin 1985 "An Image
 *   Synthesizer" — the value-noise / gradient-noise pair. Quintic blend curve
 *   from Perlin 2002 "Improving Noise". Domain-warp pattern from Inigo Quilez,
 *   https://iquilezles.org/articles/warp/ . Glyph ramp from Paul Bourke,
 *   http://paulbourke.net/dataformats/asciiart/ .
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

    /* Slots in the ncurses colour table. The two HUD slots are fixed
     * across every demo in this project; the four band slots hold the
     * current theme's colours. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* first of 4 in-a-row theme colours */
    PAIR_FLASH        =   7,    /* unused here; kept so themes match sibling files */
};

#define GLOW_THRESHOLD      0.05f

/* HUD reserves rows at the top (status) and bottom (key hints). */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* How zoomed-in the noise is. At 0.10 there are about 10 screen cells
 * between grid corners — close enough that value noise's faint grid
 * seams are actually visible (which is the whole point of this demo). */
#define NOISE_SCALE         0.10f

/* How fast the field scrolls past, in noise units per second. */
#define FIELD_DRIFT         0.10f

/* Drift speed multiplier the user bumps with +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* How many layers of detail the fBm patterns stack up. */
#define FBM_OCTAVES         4

/* Brightness cut-offs for the three glyphs in the ASCII ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Turning a brightness value (0..1) into one of 4 colour tiers.
 *   N_PALETTE_BANDS    — how many tiers (matches Theme.band[]).
 *   PALETTE_BAND_MASK  — lets us do the wrap with a cheap bit-mask
 *                        instead of %, which works only because 4 is
 *                        a power of two.
 *   GLOW_TO_BAND_GAIN  — multiply brightness by this, then chop to an
 *                        int. Just under 4 so that a brightness of
 *                        exactly 1.0 lands on tier 3, not a phantom 4.
 */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3
#define GLOW_TO_BAND_GAIN   3.999f

/*
 * Numbers that turn a grid corner's coordinates into a random value.
 *   LATTICE_HASH_MUL_X / Y — two big odd primes, one per axis. Using a
 *       different one for each axis keeps corner (1,0) and corner (0,1)
 *       from accidentally getting the same random value.
 *   HASH_TOP24_TO_UNIT     — scales the hash down into the 0..1 range.
 *       We keep the top 24 bits because the high bits of this kind of
 *       hash are the well-mixed ones; 16777215 is the largest 24-bit
 *       value.
 */
#define LATTICE_HASH_MUL_X  374761393u
#define LATTICE_HASH_MUL_Y  668265263u
#define HASH_TOP24_TO_UNIT  (1.0f / 16777215.0f)

/* 0.7071 is both the sine and cosine of 45 degrees; the ROTATE pattern
 * uses it to sample the noise turned a quarter-turn off the screen axes. */
#define ROTATE_COS_45       0.7071068f

/*
 * Pattern — the 30 things this demo can draw, in six difficulty tiers
 * from "just the raw noise" up to "full visual effects". n/p steps
 * through them. This order must line up with noise_patterns[] in §6;
 * the fixed-size array there makes the compiler catch any mismatch.
 */
typedef enum {
    /* Tier 1 — the raw noise, drawn with five different blend curves */
    PATTERN_SMOOTH = 0,
    PATTERN_LINEAR,
    PATTERN_BLOCKY,
    PATTERN_QUINTIC,
    PATTERN_COSINE,
    /* Tier 2 — stacking the noise at several scales (fBm and friends) */
    PATTERN_FBM_2,
    PATTERN_FBM,
    PATTERN_FBM_6,
    PATTERN_TURBULENCE,
    PATTERN_RIDGED,
    /* Tier 3 — one noise field reshaped by a math function */
    PATTERN_CONTOURS,
    PATTERN_THRESHOLD,
    PATTERN_WAVES,
    PATTERN_POW2,
    PATTERN_SQRT,
    /* Tier 4 — sampling the noise at bent or shifted coordinates */
    PATTERN_WARP,
    PATTERN_WISPS,
    PATTERN_ROTATE,
    PATTERN_SHIFTED,
    PATTERN_ZOOMED,
    /* Tier 5 — two noise fields combined into one */
    PATTERN_MULT,
    PATTERN_ADD,
    PATTERN_DIFF,
    PATTERN_MARBLE,
    PATTERN_NEBULA,
    /* Tier 6 — bigger recipes that read as a finished visual effect */
    PATTERN_PLASMA,
    PATTERN_STORM,
    PATTERN_LIGHTNING,
    PATTERN_STARS,
    PATTERN_AURORA,
    N_PATTERNS,
} Pattern;

/* Defined in §6 next to the table they read from. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * MAX_FRAME_DT_NS — never let one frame count as more than 100 ms of
 *   elapsed time. If the program is paused (debugger, swapped out) and
 *   then resumes, we don't want it to think hours passed and try to
 *   replay thousands of sim steps all at once. See Glenn Fiedler,
 *   "Fix Your Timestep!".
 * RENDER_FPS_TARGET — how often we redraw the screen, even if the
 *   simulation is ticking faster.
 */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — one named colour scheme. Picking a theme is kept separate
 * from how things get drawn, so switching the look is just swapping
 * which colours the four brightness tiers point at (t / T cycles them;
 * theme_apply() in §3 does the swap).
 *
 *   name   : short label shown in the HUD. Keep it 7 chars or fewer so
 *            it fits the "theme:%-8s" slot.
 *   band[] : four xterm-256 colour numbers, dimmest to brightest, one
 *            for each brightness tier. Every one must be a bright-half
 *            colour (project rule) or it vanishes on a black terminal.
 *   flash  : an accent colour. Not used in this file, but kept so the
 *            theme table matches the sibling demos that do use it.
 */
typedef struct {
    const char *name;        /* HUD label, 7 chars or fewer            */
    short       band[4];     /* one colour per brightness tier, dim→bright */
    short       flash;       /* accent colour (unused here)            */
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
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  noise — the value-noise generator and the fBm helpers on top of it */
/* ===================================================================== */

/*
 * ValueNoise — all the state the generator needs, which is just one
 * random seed. Everything else (the math that turns coordinates into
 * random values, the blend curves) is fixed.
 *
 * Patterns get this as a const pointer so the call sites read as
 * "ask the noise for a value, don't change it". It's only changed by
 * value_noise_reseed() when the user hits reset (r).
 *
 *   seed : the per-run random number. Mixed into every corner's value
 *          so each run of the program gets a different random field.
 */
typedef struct {
    uint32_t seed;
} ValueNoise;

/* Scrambles an integer into a well-spread random-looking one. Good
 * enough for visuals, not for security. Same mixer as the sibling files. */
static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/*
 * The random value (0..1) sitting at one grid corner. Same corner
 * always gives the same value within a run, but neighbouring corners
 * give unrelated values. This IS value noise's data — there's no grid
 * stored in memory; we just re-hash the corner's coordinates whenever
 * we need it.
 */
static inline float lattice_scalar(const ValueNoise *vn, int xi, int yi)
{
    uint32_t h = (uint32_t)xi * LATTICE_HASH_MUL_X
               + (uint32_t)yi * LATTICE_HASH_MUL_Y
               + vn->seed;
    h = hash32(h);
    /* Keep the well-mixed top 24 bits and scale into 0..1. */
    return (float)(h >> 8) * HASH_TOP24_TO_UNIT;
}

/* Roll a new random seed so reset (r) gives a fresh-looking field. */
static void value_noise_reseed(ValueNoise *vn)
{
    vn->seed = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/* The blend curves below all take a 0..1 fraction and bend it so the
 * blend eases in and out instead of going in a straight line. A
 * straight blend leaves visible creases at the grid corners; an eased
 * one hides them. They differ in how smooth the easing is. */

/* The standard ease. Flat at both ends, so no sudden change in slope. */
static inline float smoothstep01(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

/* Perlin's smoother ease (from "Improving Noise"). Flat at the ends in
 * an even gentler way than smoothstep, which kills a faint ridge that
 * smoothstep leaves along the grid lines. The QUINTIC pattern uses it. */
static inline float quintic01(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

/* An old-school ease built from a cosine. Looks like smoothstep but is
 * slower because of the cosine call. The COSINE pattern uses it. */
static inline float cosine01(float t)
{
    return (1.0f - cosf(t * (float)M_PI)) * 0.5f;
}

/* Blend from a to b by fraction t. */
static inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* ───── looking up a noise value, step by step ──────────────────────── *
 *
 * Asking for the noise at a point breaks into a few small steps, each
 * with its own helper below; value_noise() then just calls them in order.
 *   1. Find which grid square the point is in, and where inside it.
 *   2. (BLOCKY shortcut) if we're not blending at all, grab the nearest
 *      corner's value and stop.
 *   3. Look up the four corner values of that square.
 *   4. Bend the position fractions through the chosen ease curve.
 *   5. Blend the four corners using those bent fractions.
 */

/*
 * Where a sample point landed: which grid square, and how far into it.
 *   cell_x, cell_y : the square's bottom-left corner (whole numbers).
 *   frac_x, frac_y : how far across the square the point is, 0..1.
 */
typedef struct {
    int   cell_x, cell_y;
    float frac_x, frac_y;
} CellQuery;

/* Step 1: round down to find the grid square, and keep the leftover as
 * the position inside it. */
static inline CellQuery value_locate_cell(float x, float y)
{
    CellQuery q;
    q.cell_x = (int)floorf(x);
    q.cell_y = (int)floorf(y);
    q.frac_x = x - (float)q.cell_x;
    q.frac_y = y - (float)q.cell_y;
    return q;
}

/*
 * Step 4: bend one position fraction through the chosen ease curve.
 * This curve is the only difference between the LINEAR, SMOOTH, QUINTIC
 * and COSINE looks. (BLOCKY never gets here — it stopped at step 2.)
 *   1 LINEAR  — no easing, leave it straight
 *   2 SMOOTH  — the standard ease
 *   3 QUINTIC — Perlin's smoother ease
 *   4 COSINE  — the cosine-based ease
 */
static inline float apply_interp_curve(float t, int interp_mode)
{
    switch (interp_mode) {
    case 2:  return smoothstep01(t);
    case 3:  return quintic01(t);
    case 4:  return cosine01(t);
    default: return t;                  /* LINEAR — no easing */
    }
}

/*
 * Step 5: blend the four corner values into one. First blend the two
 * bottom corners and the two top corners left-to-right, then blend
 * those two results top-to-bottom. (bx, by) are the eased fractions.
 */
static inline float bilerp_corners(float corner_00, float corner_10,
                                   float corner_01, float corner_11,
                                   float bx, float by)
{
    float bottom_edge = lerpf(corner_00, corner_10, bx);
    float top_edge    = lerpf(corner_01, corner_11, bx);
    return lerpf(bottom_edge, top_edge, by);
}

/*
 * The core lookup: noise value (0..1) at point (x, y). interp_mode
 * picks how the corners blend:
 *   0 BLOCKY  — no blend, snap to the nearest corner
 *   1 LINEAR  — straight blend, no easing
 *   2 SMOOTH  — standard ease
 *   3 QUINTIC — Perlin's smoother ease
 *   4 COSINE  — cosine ease
 */
static float value_noise(const ValueNoise *vn,
                         float x, float y, int interp_mode)
{
    CellQuery q = value_locate_cell(x, y);

    /* BLOCKY: skip blending, just take the closest corner. */
    if (interp_mode == 0) {
        int corner_x = (q.frac_x < 0.5f) ? q.cell_x : q.cell_x + 1;
        int corner_y = (q.frac_y < 0.5f) ? q.cell_y : q.cell_y + 1;
        return lattice_scalar(vn, corner_x, corner_y);
    }

    float corner_00 = lattice_scalar(vn, q.cell_x,     q.cell_y);
    float corner_10 = lattice_scalar(vn, q.cell_x + 1, q.cell_y);
    float corner_01 = lattice_scalar(vn, q.cell_x,     q.cell_y + 1);
    float corner_11 = lattice_scalar(vn, q.cell_x + 1, q.cell_y + 1);

    float blend_x = apply_interp_curve(q.frac_x, interp_mode);
    float blend_y = apply_interp_curve(q.frac_y, interp_mode);

    return bilerp_corners(corner_00, corner_10,
                          corner_01, corner_11,
                          blend_x, blend_y);
}

/*
 * fBm — "fractional Brownian motion", the recipe behind natural-looking
 * clouds and terrain. Add up several copies of the noise: each copy
 * (an "octave") is twice as fine-grained and half as strong as the one
 * before. The big copies give the overall shape, the small ones add
 * crisp detail. Caller picks how many copies. Result is rescaled back
 * into 0..1.
 */
static float value_fbm_n(const ValueNoise *vn,
                         float x, float y, int n_octaves)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < n_octaves; o++) {
        /* Shift each copy from 0..1 to -1..1 so they can cancel as well
         * as add when summed. */
        float v = value_noise(vn, x * freq, y * freq, 2) * 2.0f - 1.0f;
        total   += amp * v;
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    /* Back to 0..1 for the renderer. */
    return total / max_amp * 0.5f + 0.5f;
}

/* fBm with the usual number of octaves — the everyday version. */
static float value_fbm(const ValueNoise *vn, float x, float y)
{
    return value_fbm_n(vn, x, y, FBM_OCTAVES);
}

/* "Turbulence": same as fBm but each copy's value is made positive
 * first. Folding the negatives up creates sharp creases — good for
 * billowing smoke and storm clouds. */
static float value_fbm_abs(const ValueNoise *vn, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        float v = value_noise(vn, x * freq, y * freq, 2) * 2.0f - 1.0f;
        total   += amp * fabsf(v);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — the 30 looks, plus the table that names them            */
/* ===================================================================== */

/*
 * Every pattern below takes the noise, a point (x, y), and the drift
 * time t, and returns one brightness value from 0 to 1. The drift time
 * is added to y so the whole field slides upward as the clock runs.
 * They're grouped by the six tiers from the Pattern enum.
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Brightness (0..1) to colour tier (0..3). See GLOW_TO_BAND_GAIN in §1
 * for why the gain is just-under-4. */
static inline int band_from_glow(float g)
{
    return (int)(g * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK;
}

/* ---------- Tier 1 — the raw noise, five blend curves ---------------- */

/* The plain, everyday value-noise look. */
static float pattern_smooth(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_noise(vn, x, y + t, 2), 0.0f, 1.0f);
}

/* No easing — you can see little pyramid shapes peaking at each corner. */
static float pattern_linear(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_noise(vn, x, y + t, 1), 0.0f, 1.0f);
}

/* No blending at all — chunky square tiles, one colour per grid cell. */
static float pattern_blocky(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_noise(vn, x, y + t, 0), 0.0f, 1.0f);
}

/* Like SMOOTH but with Perlin's gentler ease — a touch cleaner. */
static float pattern_quintic(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_noise(vn, x, y + t, 3), 0.0f, 1.0f);
}

/* The cosine ease. Looks almost the same as SMOOTH. */
static float pattern_cosine(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_noise(vn, x, y + t, 4), 0.0f, 1.0f);
}

/* ---------- Tier 2 — stacked noise (fBm and friends) ----------------- */

/* Just two layers: big rolling blobs with one layer of fine detail. */
static float pattern_fbm_2(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_fbm_n(vn, x, y + t, 2), 0.0f, 1.0f);
}

/* The default cloud look. */
static float pattern_fbm(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_fbm(vn, x, y + t), 0.0f, 1.0f);
}

/* Six layers — extra-fine detail (the finest layers start to sparkle). */
static float pattern_fbm_6(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_fbm_n(vn, x, y + t, 6), 0.0f, 1.0f);
}

/* Turbulence — billowing, sharp-edged storm clouds. */
static float pattern_turbulence(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_fbm_abs(vn, x, y + t), 0.0f, 1.0f);
}

/* Bright thin ridges wherever the cloud crosses its midpoint — wispy
 * cirrus streaks. */
static float pattern_ridged(const ValueNoise *vn, float x, float y, float t)
{
    /* Re-centre the cloud around zero so its midline becomes the ridge. */
    float v = (value_fbm(vn, x, y + t) - 0.5f) * 2.0f;
    return clampf(1.0f - fabsf(v), 0.0f, 1.0f);
}

/* ---------- Tier 3 — one cloud, reshaped by a math function --------- */

/* Stacked contour lines, like a topographic map. */
static float pattern_contours(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm(vn, x, y + t) * 8.0f;
    return v - floorf(v);
}

/* Snap to either dark or bright around the midpoint, with a soft edge in
 * between — looks like patchy cloud cover. */
static float pattern_threshold(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm(vn, x, y + t);
    if (v < 0.45f) return 0.0f;
    if (v > 0.55f) return 1.0f;
    float s = (v - 0.45f) * 10.0f;
    return s * s * (3.0f - 2.0f * s);
}

/* Ripple it through a sine — concentric rings, like raindrops on a pond. */
static float pattern_waves(const ValueNoise *vn, float x, float y, float t)
{
    return sinf(value_fbm(vn, x, y + t) * 12.0f) * 0.5f + 0.5f;
}

/* Square it — pushes the bright spots brighter and the dim spots darker. */
static float pattern_pow2(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm(vn, x, y + t);
    return v * v;
}

/* Square-root it — lifts the dim spots, giving a flat low-contrast haze. */
static float pattern_sqrt(const ValueNoise *vn, float x, float y, float t)
{
    return sqrtf(clampf(value_fbm(vn, x, y + t), 0.0f, 1.0f));
}

/* ---------- Tier 4 — sample at bent or shifted coordinates ----------- */

/* Domain warp: look up the cloud at points that a second cloud has
 * nudged around. Looks like currents dragging the clouds. (Inigo Quilez.) */
static float pattern_warp(const ValueNoise *vn, float x, float y, float t)
{
    float wx = (value_fbm(vn, x,        y + t)        - 0.5f) * 2.0f;
    float wy = (value_fbm(vn, x + 5.2f, y + 1.3f + t) - 0.5f) * 2.0f;
    return clampf(value_fbm(vn, x + wx, y + wy + t), 0.0f, 1.0f);
}

/* Squash one axis so round puffs stretch into horizontal streaks. */
static float pattern_wisps(const ValueNoise *vn, float x, float y, float t)
{
    return clampf(value_fbm(vn, x * 0.4f, y + t * 1.5f), 0.0f, 1.0f);
}

/* Sample the cloud turned 45 degrees, so its grid seams run diagonally
 * instead of straight up and down. */
static float pattern_rotate(const ValueNoise *vn, float x, float y, float t)
{
    const float ca = ROTATE_COS_45;
    const float sa = ROTATE_COS_45;         /* sin and cos of 45 are equal */
    float rx = x * ca - y * sa;
    float ry = x * sa + y * ca;
    return clampf(value_fbm(vn, rx, ry + t), 0.0f, 1.0f);
}

/* Average four samples nudged half a cell apart — softens the cloud. */
static float pattern_shifted(const ValueNoise *vn, float x, float y, float t)
{
    float a = value_fbm(vn, x,        y + t);
    float b = value_fbm(vn, x + 0.5f, y + t);
    float c = value_fbm(vn, x,        y + 0.5f + t);
    float d = value_fbm(vn, x + 0.5f, y + 0.5f + t);
    return clampf(0.25f * (a + b + c + d), 0.0f, 1.0f);
}

/* Mix a coarse cloud with a 3x-finer one — big blobs with detail on top. */
static float pattern_zoomed(const ValueNoise *vn, float x, float y, float t)
{
    float lo = value_fbm(vn, x,        y + t);
    float hi = value_fbm(vn, x * 3.0f, y * 3.0f + t);
    return clampf(0.6f * lo + 0.4f * hi, 0.0f, 1.0f);
}

/* ---------- Tier 5 — two clouds combined ---------------------------- */

/* Multiply two clouds — only stays bright where BOTH are bright. */
static float pattern_mult(const ValueNoise *vn, float x, float y, float t)
{
    float a = value_fbm(vn, x, y + t);
    float b = value_fbm(vn, x * 2.0f + 5.0f, y * 2.0f + 3.0f + t);
    return clampf(a * b * 2.0f, 0.0f, 1.0f);
}

/* Average two clouds at different scales together. */
static float pattern_add(const ValueNoise *vn, float x, float y, float t)
{
    float a = value_fbm(vn, x,        y + t);
    float b = value_fbm(vn, x * 3.0f, y * 3.0f + t);
    return clampf((a + b) * 0.5f, 0.0f, 1.0f);
}

/* Subtract a shifted copy from the cloud — lights up only the edges. */
static float pattern_diff(const ValueNoise *vn, float x, float y, float t)
{
    float a = value_fbm(vn, x,        y + t);
    float b = value_fbm(vn, x + 0.5f, y + 0.5f + t);
    return clampf(fabsf(a - b) * 4.0f, 0.0f, 1.0f);
}

/* Even sine stripes, with the cloud wobbling them so they bend like the
 * veins in marble. */
static float pattern_marble(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm(vn, x, y + t);
    return sinf(x * 3.0f + v * 6.0f) * 0.5f + 0.5f;
}

/* A cloud times some turbulence — bright cores in a dim gassy haze. */
static float pattern_nebula(const ValueNoise *vn, float x, float y, float t)
{
    float a = value_fbm(vn, x, y + t);
    float b = value_fbm_abs(vn, x * 1.5f + 7.0f, y * 1.5f + 3.0f + t);
    return clampf(a * b * 1.8f, 0.0f, 1.0f);
}

/* ---------- Tier 6 — full visual effects ----------------------------- */

/* The old demoscene plasma: three sine waves added together, with a
 * cloud jiggling each one's phase. */
static float pattern_plasma(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm(vn, x, y + t);
    float s = sinf(x * 2.0f + v * 3.0f)
            + cosf(y * 2.5f + v * 3.0f)
            + sinf((x + y) * 1.5f + v * 2.0f);
    return (s + 3.0f) / 6.0f;
}

/* Turbulence squared and brightened — angrier, higher-contrast clouds. */
static float pattern_storm(const ValueNoise *vn, float x, float y, float t)
{
    float v = value_fbm_abs(vn, x, y + t);
    return clampf(v * v * 1.5f, 0.0f, 1.0f);
}

/* Take the ridge look and raise it to a high power so only the very
 * brightest threads survive — they read as lightning bolts. */
static float pattern_lightning(const ValueNoise *vn, float x, float y, float t)
{
    float v = (value_fbm(vn, x, y + t) - 0.5f) * 2.0f;   /* re-centre */
    float r = 1.0f - fabsf(v);                           /* the ridge */
    float p = r * r;                                     /* build up to */
    p     *= p;                                          /* the 12th    */
    p     *= p;                                          /* power, which */
    p     *= r * r * r * r;                              /* leaves only the peaks */
    return clampf(p * 1.8f, 0.0f, 1.0f);
}

/* Sparse bright pinpoints (a fine, high-contrast noise) over a dim
 * background glow — a starfield. */
static float pattern_stars(const ValueNoise *vn, float x, float y, float t)
{
    float bg    = value_noise(vn, x, y + t * 0.3f, 2) * 0.3f;
    float stars = value_noise(vn, x * 6.0f + 13.0f, y * 6.0f + 7.0f + t * 0.5f, 2);
    if (stars > 0.7f) bg += (stars - 0.7f) * 3.0f;
    return clampf(bg, 0.0f, 1.0f);
}

/* Shimmering horizontal curtains that fade out toward the top and
 * bottom — northern lights. */
static float pattern_aurora(const ValueNoise *vn, float x, float y, float t)
{
    float band = sinf(x * 1.5f + value_fbm(vn, x, y + t) * 4.0f) * 0.5f + 0.5f;
    float vert = sinf(y * 0.4f + 1.0f);
    if (vert < 0.0f) vert = 0.0f;
    return clampf(band * vert * 1.2f, 0.0f, 1.0f);
}

/* ---------- the lookup table --------------------------------------- */

/* Shape every pattern function shares (see the §6 intro for the args). */
typedef float (*ValuePatternFn)(const ValueNoise *vn,
                                float x, float y, float t);

/*
 * NoisePattern — one row of the table that maps a Pattern enum value to
 * its name, tier label, and the function that draws it. Using a table
 * instead of a 30-case switch means adding a pattern is just an enum
 * value, a table row, and a function — and because the array is sized
 * exactly [N_PATTERNS], the compiler complains if a row is missing.
 *
 *   name   : shown in the HUD, padded to a fixed 10 chars so the HUD
 *            doesn't jitter as you flip through patterns.
 *   tier   : the "1-INTRP" / "6-FX" style label, fixed at 7 chars for
 *            the same reason.
 *   sample : the pattern's drawing function.
 */
typedef struct {
    const char     *name;      /* HUD name, padded to 10 chars   */
    const char     *tier;      /* HUD tier label, padded to 7    */
    ValuePatternFn  sample;    /* the function that draws it      */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — INTERPOLATION */
    [PATTERN_SMOOTH]     = { "SMOOTH    ", "1-INTRP", pattern_smooth     },
    [PATTERN_LINEAR]     = { "LINEAR    ", "1-INTRP", pattern_linear     },
    [PATTERN_BLOCKY]     = { "BLOCKY    ", "1-INTRP", pattern_blocky     },
    [PATTERN_QUINTIC]    = { "QUINTIC   ", "1-INTRP", pattern_quintic    },
    [PATTERN_COSINE]     = { "COSINE    ", "1-INTRP", pattern_cosine     },
    /* Tier 2 — STACKS */
    [PATTERN_FBM_2]      = { "FBM_2     ", "2-STACK", pattern_fbm_2      },
    [PATTERN_FBM]        = { "FBM       ", "2-STACK", pattern_fbm        },
    [PATTERN_FBM_6]      = { "FBM_6     ", "2-STACK", pattern_fbm_6      },
    [PATTERN_TURBULENCE] = { "TURBULENCE", "2-STACK", pattern_turbulence },
    [PATTERN_RIDGED]     = { "RIDGED    ", "2-STACK", pattern_ridged     },
    /* Tier 3 — MAPPED */
    [PATTERN_CONTOURS]   = { "CONTOURS  ", "3-MAP  ", pattern_contours   },
    [PATTERN_THRESHOLD]  = { "THRESHOLD ", "3-MAP  ", pattern_threshold  },
    [PATTERN_WAVES]      = { "WAVES     ", "3-MAP  ", pattern_waves      },
    [PATTERN_POW2]       = { "POW2      ", "3-MAP  ", pattern_pow2       },
    [PATTERN_SQRT]       = { "SQRT      ", "3-MAP  ", pattern_sqrt       },
    /* Tier 4 — WARPED */
    [PATTERN_WARP]       = { "WARP      ", "4-WARP ", pattern_warp       },
    [PATTERN_WISPS]      = { "WISPS     ", "4-WARP ", pattern_wisps      },
    [PATTERN_ROTATE]     = { "ROTATE    ", "4-WARP ", pattern_rotate     },
    [PATTERN_SHIFTED]    = { "SHIFTED   ", "4-WARP ", pattern_shifted    },
    [PATTERN_ZOOMED]     = { "ZOOMED    ", "4-WARP ", pattern_zoomed     },
    /* Tier 5 — COMPOSITES */
    [PATTERN_MULT]       = { "MULT      ", "5-COMP ", pattern_mult       },
    [PATTERN_ADD]        = { "ADD       ", "5-COMP ", pattern_add        },
    [PATTERN_DIFF]       = { "DIFF      ", "5-COMP ", pattern_diff       },
    [PATTERN_MARBLE]     = { "MARBLE    ", "5-COMP ", pattern_marble     },
    [PATTERN_NEBULA]     = { "NEBULA    ", "5-COMP ", pattern_nebula     },
    /* Tier 6 — EFFECTS */
    [PATTERN_PLASMA]     = { "PLASMA    ", "6-FX   ", pattern_plasma     },
    [PATTERN_STORM]      = { "STORM     ", "6-FX   ", pattern_storm      },
    [PATTERN_LIGHTNING]  = { "LIGHTNING ", "6-FX   ", pattern_lightning  },
    [PATTERN_STARS]      = { "STARS     ", "6-FX   ", pattern_stars      },
    [PATTERN_AURORA]     = { "AURORA    ", "6-FX   ", pattern_aurora     },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return noise_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return noise_patterns[p].tier;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * GlowField — the finished grid of values, one entry per screen cell.
 * It's the hand-off point: the simulation fills it in each tick, and the
 * renderer reads it each frame, so the two don't have to run in lockstep.
 * Both arrays are full-size up front, so drawing never has to allocate.
 * Find a cell with glow_field_idx(gf, x, y).
 *
 *   w, h    : grid size in cells, set when the screen size is chosen.
 *   count   : w * h, kept around for the clear loop.
 *   glow[]  : each cell's brightness, 0..1 (the pattern writes this).
 *   band[]  : each cell's colour tier, 0..3 (worked out from glow).
 */
typedef struct {
    int      w, h;                   /* grid size in cells            */
    int      count;                  /* w * h                         */
    float    glow[CELLS_MAX];        /* per-cell brightness, 0..1     */
    uint8_t  band[CELLS_MAX];        /* per-cell colour tier, 0..3    */
} GlowField;

static inline int glow_field_idx(const GlowField *gf, int x, int y)
{
    return y * gf->w + x;
}

static void glow_field_reset(GlowField *gf, int w, int h)
{
    gf->w     = w;
    gf->h     = h;
    gf->count = w * h;
    for (int i = 0; i < gf->count; i++) {
        gf->glow[i] = 0.0f;
        gf->band[i] = 0;
    }
}

/*
 * GlyphRamp — the rule for turning a brightness into a character.
 * Brighter cells get a denser glyph; the dimmest cells are left blank
 * so the background shows through. Keeping it as data (not an if/else
 * buried in the renderer) makes the cut-offs easy to see and tweak.
 *
 *      brightness above thresh_high → glyph_high, bold
 *      above thresh_mid             → glyph_mid,  bold
 *      above thresh_low             → glyph_low,  normal
 *      below that                   → nothing drawn
 *
 * Defaults are '#' / '*' / '.', a coarse slice of Paul Bourke's
 * brightness-to-character ramp.
 */
typedef struct {
    float thresh_high;   /* above this → the densest glyph    */
    float thresh_mid;    /* above this → the middle glyph     */
    float thresh_low;    /* above this → the faint glyph      */
    char  glyph_high;    /* densest glyph                     */
    char  glyph_mid;     /* middle glyph                      */
    char  glyph_low;     /* faint glyph                       */
} GlyphRamp;

/*
 * GlyphChoice — what glyph_ramp_pick() hands back: the character to
 * draw, its bold/normal style, and whether to draw anything at all
 * (a too-dim cell is left blank).
 */
typedef struct {
    char glyph;         /* the character (ignore if not visible) */
    int  attr;          /* bold or normal                        */
    bool visible;       /* false → leave the cell blank          */
} GlyphChoice;

static void glyph_ramp_init(GlyphRamp *gr)
{
    gr->thresh_high = GLYPH_HIGH_THRESH;
    gr->thresh_mid  = GLYPH_MID_THRESH;
    gr->thresh_low  = GLOW_THRESHOLD;
    gr->glyph_high  = '#';
    gr->glyph_mid   = '*';
    gr->glyph_low   = '.';
}

static GlyphChoice glyph_ramp_pick(const GlyphRamp *gr, float glow)
{
    GlyphChoice c = { .glyph = ' ', .attr = A_NORMAL, .visible = false };
    if      (glow > gr->thresh_high) { c.glyph = gr->glyph_high; c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_mid)  { c.glyph = gr->glyph_mid;  c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_low)  { c.glyph = gr->glyph_low;  c.attr = A_NORMAL; c.visible = true; }
    return c;
}

/*
 * PatternState — which pattern is showing and how it's moving.
 *
 *   current    : the pattern on screen (n/p cycle through, wrapping).
 *                Starts on SMOOTH.
 *   field_time : a running clock that's added to every sample's y, which
 *                makes the field scroll upward. Only the reset key
 *                rewinds it.
 *   drift_mult : how fast it scrolls; +/- double or halve it, 1 up to 32.
 */
typedef struct {
    Pattern current;          /* the pattern on screen          */
    float   field_time;       /* scroll clock                   */
    int     drift_mult;       /* scroll-speed multiplier, 1..32 */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current    = PATTERN_SMOOTH;
    ps->field_time = 0.0f;
    ps->drift_mult = DRIFT_MULT_DEF;
}

/*
 * PaletteState — which theme is showing. Just one number today, but kept
 * as its own struct so "the current colour scheme" has a clear home.
 *
 *   current : which entry of themes[] (§1) is active, 0 up to N_THEMES-1.
 */
typedef struct {
    int current;              /* index into themes[] */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — everything the running demo can change, in one place. The
 * fields are listed in the order the data flows through them each tick:
 *   noise   — the random seed everything is built from
 *   pattern — which look is showing and how it's scrolling
 *   field   — the finished grid of brightness/colour values
 *   ramp    — how a brightness becomes a character
 *   palette — which colour theme is active
 *   paused  — when true, the simulation holds still
 */
typedef struct {
    ValueNoise    noise;     /* the random seed                */
    GlowField     field;     /* finished grid (biggest member) */
    GlyphRamp     ramp;      /* brightness → character rule    */
    PatternState  pattern;   /* current look + scrolling       */
    PaletteState  palette;   /* current colour theme           */
    bool          paused;    /* freeze the simulation          */
} Scene;

/* Turn a cell number into the coordinate handed to a pattern. */
static inline float cell_to_noise_coord(int cell)
{
    return (float)cell * NOISE_SCALE;
}

/* Fill the whole grid for one simulation tick: run the current pattern
 * at every cell and store its brightness and colour tier. */
static void scene_evaluate_field(Scene *s)
{
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    ValuePatternFn      sample_pattern = noise_patterns[active].sample;
    const ValueNoise   *noise          = &s->noise;
    GlowField          *field          = &s->field;
    float               drift          = s->pattern.field_time;

    for (int y = 0; y < field->h; y++) {
        float fy = cell_to_noise_coord(y);
        for (int x = 0; x < field->w; x++) {
            float fx   = cell_to_noise_coord(x);
            float glow = clampf(sample_pattern(noise, fx, fy, drift),
                                0.0f, 1.0f);

            int idx          = glow_field_idx(field, x, y);
            field->glow[idx] = glow;
            field->band[idx] = (uint8_t)band_from_glow(glow);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->pattern.field_time = 0.0f;
    value_noise_reseed(&s->noise);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    glyph_ramp_init   (&s->ramp);
    pattern_state_init(&s->pattern);
    palette_state_init(&s->palette);
    s->paused = false;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->pattern.field_time += FIELD_DRIFT * (float)s->pattern.drift_mult * dt;
    scene_evaluate_field(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal's current width and height, remembered so we
 * don't have to ask ncurses for them on every cell. Refreshed whenever
 * the terminal is resized.
 *
 *   cols : width in characters.
 *   rows : height in characters.
 * (rows-first ordering matches ncurses' getmaxyx.)
 */
typedef struct {
    int cols;     /* terminal width  in characters  */
    int rows;     /* terminal height in characters  */
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

/* ---------- drawing the grid ----------------------------------------- */

/*
 * GridPlacement — the top-left spot on screen where the grid starts,
 * once it's been centred with the HUD rows kept clear. Worked out once
 * per frame so the per-cell loop just adds x and y to it instead of
 * redoing the centring math for every cell.
 *
 *   origin_x : starting column (0 if the grid is wider than the screen).
 *   origin_y : starting row (never above the top HUD rows).
 *
 * If the grid is bigger than the screen these are clamped to the edge,
 * so the grid is cropped rather than wrapping around.
 */
typedef struct {
    int origin_x;   /* starting column */
    int origin_y;   /* starting row    */
} GridPlacement;

/* Centre the grid on screen, keeping the HUD rows free. Clamps to the
 * edge so an oversized grid is cropped, not wrapped. */
static GridPlacement compute_grid_placement(int field_w, int field_h,
                                            int cols, int rows)
{
    GridPlacement p;
    p.origin_x = (cols - field_w) / 2;
    p.origin_y = ((rows - HUD_BAND_RESERVED_ROWS) - field_h) / 2
                 + HUD_TOP_ROWS;
    if (p.origin_x < 0)            p.origin_x = 0;
    if (p.origin_y < HUD_TOP_ROWS) p.origin_y = HUD_TOP_ROWS;
    return p;
}

/* Draw one character in the given colour and style. */
static inline void draw_glyph_at(int screen_y, int screen_x,
                                 char glyph, int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(screen_y, screen_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* Paint the whole grid: centre it, then for each cell pick a character
 * from its brightness and draw it in that cell's colour. Dim cells are
 * skipped so the background shows through. Only reads state. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const GlowField *field = &s->field;
    const GlyphRamp *ramp  = &s->ramp;
    GridPlacement    place = compute_grid_placement(field->w, field->h,
                                                    cols, rows);

    for (int y = 0; y < field->h; y++) {
        int screen_y = place.origin_y + y;
        if (screen_y < 0 || screen_y >= rows) continue;
        for (int x = 0; x < field->w; x++) {
            int screen_x = place.origin_x + x;
            if (screen_x < 0 || screen_x >= cols) continue;

            int   cell_idx = glow_field_idx(field, x, y);
            float glow     = field->glow[cell_idx];

            GlyphChoice pick = glyph_ramp_pick(ramp, glow);
            if (!pick.visible) continue;

            int color_pair = PAIR_BAND_BASE + (field->band[cell_idx] & PALETTE_BAND_MASK);
            draw_glyph_at(screen_y, screen_x,
                          pick.glyph, color_pair, pick.attr);
        }
    }
}

/* ---------- HUD layout ----------------------------------------------- *
 *
 * The status row is built left to right from labelled chunks. Each
 * chunk's width below is how far it pushes the cursor along.
 */
#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " VALUE NOISE "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* ---------- HUD pieces ------------------------------------------------ *
 *
 * Each piece draws at (row, x) and returns the next x, so they chain.
 */

/* The title at the far left. */
static int hud_draw_title_chip(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, "%s", HUD_TITLE_TEXT);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + (int)strlen(HUD_TITLE_TEXT);
}

/* Top-right readout: fps, tick rate, state, pattern number, drift speed. */
static void hud_draw_state_bar(int row, int cols,
                               double fps, int sim_fps,
                               const PatternState *ps, bool paused)
{
    const char *state_text = paused ? "PAUSED    " : pattern_name(ps->current);
    char        buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d ",
             fps, sim_fps, state_text,
             (int)ps->current + 1, N_PATTERNS,
             ps->drift_mult);
    int right_aligned_x = cols - (int)strlen(buf);
    if (right_aligned_x < 0) right_aligned_x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, right_aligned_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Status row — the current pattern's name. */
static int hud_draw_pattern_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-10s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_PATTERN_FIELD;
}

/* Status row — the current pattern's tier label. */
static int hud_draw_tier_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " tier:%-7s ", pattern_tier(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_TIER_FIELD;
}

/* Status row — the current theme's name. */
static int hud_draw_theme_field(int row, int x, int theme_idx)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_THEME_FIELD;
}

/* Status row — four little coloured blocks showing the theme's four
 * brightness-tier colours, so you can see the palette at a glance. */
static int hud_draw_palette_swatches(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_W_PALETTE_LABEL;
    for (int band = 0; band < N_PALETTE_BANDS; band++) {
        int color_pair = PAIR_BAND_BASE + band;
        attron(COLOR_PAIR(color_pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(color_pair) | A_BOLD);
        x += 1;
    }
    return x;
}

/* Status row tail — the noise settings: zoom, layer count, grid size. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  oct:%d  map:%dx%d ",
             NOISE_SCALE, FBM_OCTAVES, gf->w, gf->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the list of keys you can press. */
static void hud_draw_action_hint(int row)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(row, 0, "%s", HUD_BOTTOM_HINT_TEXT);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ---------- the full frame ------------------------------------------- */

/* Draw one whole frame: clear, paint the field, then lay down the HUD —
 * title and stats up top, status chunks below, key hints at the bottom. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    hud_draw_title_chip(HUD_TITLE_ROW, HUD_LEFT_MARGIN);
    hud_draw_state_bar (HUD_TITLE_ROW, sc->cols, fps, sim_fps,
                        &s->pattern, s->paused);

    int x = HUD_LEFT_MARGIN;
    x = hud_draw_pattern_field   (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_tier_field      (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_theme_field     (HUD_STATUS_ROW, x, s->palette.current);
    x = hud_draw_palette_swatches(HUD_STATUS_ROW, x);
    hud_draw_stats_field         (HUD_STATUS_ROW, x, &s->field);

    hud_draw_action_hint(sc->rows - 1);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the whole program's state in one struct. main() works through
 * this and nothing else; helpers take a pointer to it (or to a part of
 * it). There's a single global instance because signal handlers have to
 * reach the two flags below and can't be handed a pointer.
 *
 *   scene       : the simulation (see Scene).
 *   screen      : terminal size, refreshed when the window resizes.
 *   sim_fps     : how many times a second the simulation steps; ]/[
 *                 raise and lower it, capped to 10..240.
 *   map_w,      : the grid size, derived from the terminal size minus
 *   map_h         the HUD rows.
 *   running     : set to 0 to quit (by Ctrl-C, kill, or the q key). The
 *                 odd type is what's safe to touch from a signal handler,
 *                 and volatile stops the compiler caching it in the loop.
 *   need_resize : set to 1 when the window resizes; the loop notices,
 *                 resizes, and clears it. Same special type as running.
 */
typedef struct {
    Scene                 scene;        /* the simulation                       */
    Screen                screen;       /* terminal size                        */
    int                   sim_fps;      /* simulation steps per second          */
    int                   map_w;        /* grid width  (cells)                  */
    int                   map_h;        /* grid height (cells)                  */
    volatile sig_atomic_t running;      /* 0 → quit                             */
    volatile sig_atomic_t need_resize;  /* 1 → window resized, loop will handle */
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

/* ---------- what each key does --------------------------------------- *
 *
 * One small function per key action, so app_handle_key reads as a clean
 * list of "key → action". +1 means next, -1 means previous.
 */

/* Step to the next/previous pattern, wrapping around the ends. */
static void scene_cycle_pattern(Scene *s, int direction)
{
    int next = ((int)s->pattern.current + direction + N_PATTERNS) % N_PATTERNS;
    s->pattern.current = (Pattern)next;
}

/* Step to the next/previous theme and load its colours into ncurses. */
static void scene_cycle_theme(Scene *s, int direction)
{
    s->palette.current = (s->palette.current + direction + N_THEMES)
                         % N_THEMES;
    theme_apply(s->palette.current);
}

/* Drift speed doubles or halves so the steps feel distinct, not a slider. */
static void scene_drift_double(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult = DRIFT_MULT_MAX;
}
static void scene_drift_halve(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/* Speed up / slow down the simulation, kept within its limits. */
static void app_adjust_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Handle one keypress. Returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            s->paused = !s->paused;                 break;
    case 'r': case 'R':  scene_reset(s, app->map_w, app->map_h); break;

    case '=': case '+':  scene_drift_double(&s->pattern);        break;
    case '-':            scene_drift_halve (&s->pattern);        break;

    case ']':            app_adjust_sim_fps(app, +SIM_FPS_STEP); break;
    case '[':            app_adjust_sim_fps(app, -SIM_FPS_STEP); break;

    case 't':            scene_cycle_theme  (s, +1);             break;
    case 'T':            scene_cycle_theme  (s, -1);             break;

    case 'n': case 'N':  scene_cycle_pattern(s, +1);             break;
    case 'p': case 'P':  scene_cycle_pattern(s, -1);             break;

    default:                                                     break;
    }
    return true;
}

/* ---------- the main loop's timekeeping ------------------------------ */

/*
 * FrameClock — the running totals the main loop uses to keep time, kept
 * together so the loop body reads as named steps. It tracks two separate
 * things that both feed off each frame's elapsed time:
 *   - a leftover-time bucket that decides how many simulation steps to
 *     run, so the sim runs at a steady rate no matter how jittery the
 *     frame rate is (Glenn Fiedler, "Fix Your Timestep!");
 *   - an fps counter that updates the on-screen number now and then.
 *
 *   frame_time_ns   : when the current frame started.
 *   sim_accum_ns    : elapsed time waiting to be spent on sim steps.
 *   fps_accum_ns    : time piled up since the last fps update.
 *   fps_frame_count : frames counted since the last fps update.
 *   fps_display     : the fps number the HUD shows.
 */
typedef struct {
    int64_t frame_time_ns;     /* start time of the current frame  */
    int64_t sim_accum_ns;      /* time waiting to become sim steps */
    int64_t fps_accum_ns;      /* time since last fps update        */
    int     fps_frame_count;   /* frames since last fps update      */
    double  fps_display;       /* fps number shown in the HUD       */
} FrameClock;

static void frame_clock_init(FrameClock *fc)
{
    fc->frame_time_ns   = clock_ns();
    fc->sim_accum_ns    = 0;
    fc->fps_accum_ns    = 0;
    fc->fps_frame_count = 0;
    fc->fps_display     = 0.0;
}

/* Reset the clock after a pause (like a resize) so the sim doesn't try
 * to fast-forward through time the user never actually saw. */
static void frame_clock_resync(FrameClock *fc)
{
    fc->frame_time_ns = clock_ns();
    fc->sim_accum_ns  = 0;
}

/* Start a new frame; return how long since the last one, capped so a long
 * stall can't make the sim try to catch up all at once. */
static int64_t frame_clock_advance(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->frame_time_ns;
    fc->frame_time_ns = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* Count this frame toward the fps number, and recompute it occasionally. */
static void fps_meter_observe(FrameClock *fc, int64_t frame_dt_ns)
{
    fc->fps_frame_count++;
    fc->fps_accum_ns += frame_dt_ns;
    if (fc->fps_accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        double elapsed_sec = (double)fc->fps_accum_ns / (double)NS_PER_SEC;
        fc->fps_display     = (double)fc->fps_frame_count / elapsed_sec;
        fc->fps_frame_count = 0;
        fc->fps_accum_ns    = 0;
    }
}

/* ---------- the steps the main loop runs each frame ------------------ */

/* Add this frame's elapsed time to the bucket, then step the simulation
 * one fixed slice at a time until the bucket runs low. Each step always
 * gets the same slice of time, so the sim runs steadily even when the
 * frame rate wobbles (Glenn Fiedler, "Fix Your Timestep!"). */
static void app_run_fixed_step_ticks(App *app, FrameClock *fc,
                                     int64_t frame_dt_ns)
{
    int64_t tick_ns     = TICK_NS(app->sim_fps);
    float   tick_dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    fc->sim_accum_ns += frame_dt_ns;
    while (fc->sim_accum_ns >= tick_ns) {
        scene_tick(&app->scene, tick_dt_sec);
        fc->sim_accum_ns -= tick_ns;
    }
}

/* Wait out the rest of the frame so we redraw at roughly 60 per second:
 * see how much of the frame's time budget is already gone, sleep the rest. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Grab a keypress if there is one and act on it; quit if it says so. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Hook up Ctrl-C / kill to the quit flag and window-resize to the resize
 * flag. The handlers only flip one flag each, which is all that's safe to
 * do inside a signal handler. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/*
 * main — set things up, then loop: handle any resize, work out elapsed
 * time, step the sim, update fps, wait out the frame, draw, read input.
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    FrameClock clock;
    frame_clock_init(&clock);

    while (app->running) {
        if (app->need_resize) {
            app_do_resize(app);
            frame_clock_resync(&clock);
        }

        int64_t frame_dt_ns = frame_clock_advance(&clock);
        app_run_fixed_step_ticks   (app, &clock, frame_dt_ns);
        fps_meter_observe          (&clock, frame_dt_ns);
        app_throttle_to_render_rate(clock.frame_time_ns, frame_dt_ns);

        screen_draw(&app->screen, &app->scene,
                    clock.fps_display, app->sim_fps);
        screen_present();

        app_pump_input(app);
    }

    screen_free(&app->screen);
    return 0;
}
