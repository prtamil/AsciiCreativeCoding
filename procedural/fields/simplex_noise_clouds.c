/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * simplex_noise_clouds.c — drifting clouds drawn in ASCII from simplex noise.
 *
 * Simplex noise is a way to make smooth, natural-looking random fields
 * (think clouds, smoke, terrain). It's the 2001 follow-up to classic
 * Perlin noise, and looks more even because it has no built-in "grain"
 * along the screen axes. We compute one noise value per terminal cell,
 * let the field slowly drift, and draw it with three density characters.
 * There are 30 different "looks" (patterns) you can flip through; they
 * all start from the same noise and just transform it in different ways.
 *
 * Sister files worth a look:
 *   ./perin_noise_flow_showcase.c   — the older Perlin noise this improves on
 *   ./domain_warped_noise_iq_style.c — a deep dive on the warping trick (tier 4)
 *   ./worley_cellular_noise.c        — a different kind of noise (cells, not blobs)
 *
 * The math here follows two references the code can't restate:
 *   Perlin, K. (2001) "Noise hardware" — the simplex idea.
 *   Gustavson, S. (2005) "Simplex Noise Demystified" — the implementation
 *     this file mirrors: https://weber.itn.liu.se/~stegu/simplexnoise/simplexnoise.pdf
 *   Bourke, P. — the ASCII brightness ramp the '.', '*', '#' glyphs come from.
 *
 * Keys: q/ESC quit  space pause  r reset(new field)  n/p pattern
 *       t/T theme  +/- drift speed  ]/[ tick rate
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra simplex_noise_clouds.c -o simplex_clouds -lncurses -lm
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

    /* ncurses color-pair slots. 1 and 2 are the HUD colors; 3..6 are the
     * four cloud shades; 7 is an unused accent kept for parity with sister files. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
    PAIR_FLASH        =   7,
};

#define GLOW_THRESHOLD      0.05f

/* Two rows reserved at the top for the HUD (title + readouts), one at the
 * bottom for the key hints. The cloud field fills everything in between. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* How fast we move through "noise space" as we step from cell to cell.
 * Smaller = larger, smoother cloud shapes spread across the screen. */
#define NOISE_SCALE         0.04f

/* How far the whole field slides per second, so the clouds keep moving. */
#define FIELD_DRIFT         0.10f

/* Drift can be sped up or slowed with +/-, doubling/halving between these. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* How many layers of noise we stack: each layer is finer and fainter than
 * the last, which is what gives clouds both big shapes and small detail. */
#define FBM_OCTAVES         4

/* WISPS stretches the noise sideways so clouds become long streaks
 * instead of round puffs (smaller number = longer streaks). */
#define WISPS_X_SCALE       0.40f

/* A cell brighter than HIGH gets the densest character, brighter than MID
 * the medium one; dimmer cells get a faint dot or nothing. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* Each cell's brightness (0..1) is also sorted into one of four color
 * shades. We multiply by "almost 4" and chop to a whole number: using
 * exactly 4 would let a fully-bright cell (1.0) spill into a 5th shade
 * that doesn't exist, so we stay a hair below 4. */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3
#define GLOW_TO_BAND_GAIN   3.999f

/* Simplex noise lives on a grid of triangles, not squares. To get there
 * we squash the normal x/y space a bit (F2) before snapping to a cell,
 * then un-squash (G2) to measure where the point really sits. These exact
 * numbers come from Gustavson's reference; rounding them causes a point to
 * occasionally land in the wrong triangle, which shows up as visible seams.
 * The final gain (70) just stretches the raw result back into about -1..1. */
#define SIMPLEX_F2            0.36602540378443864f /* squash  = (sqrt(3) - 1) / 2 */
#define SIMPLEX_G2            0.21132486540518713f /* unsquash = (3 - sqrt(3)) / 6 */
#define SIMPLEX_OUTPUT_GAIN   70.0f

/* A 2-D simplex (triangle) has 3 corners; we have 12 fixed gradient
 * directions to pick from, and a shuffled lookup table of 256 values
 * (stored twice, 512 long, so an index that overshoots still lands safely).
 * The "& 255" is just a fast way to keep an index inside 0..255. */
#define N_SIMPLEX_CORNERS     3
#define N_GRADIENT_DIRS       12
#define PERM_TABLE_SIZE       256
#define PERM_TABLE_INDEX_MASK 255

/*
 * The 30 "looks" you can flip through with n/p. They're grouped into six
 * tiers, roughly simple to fancy: each tier is a different trick for turning
 * the same noise into a different image, and the five patterns in a tier are
 * variations on that one trick.
 *
 * The order here must line up with the noise_patterns[] table in §6 — the
 * fixed-size array there means the compiler complains if they ever drift
 * apart, so they can't silently get out of sync.
 */
typedef enum {
    /* Tier 1 RAW — show the noise more or less straight */
    PATTERN_CLOUDS = 0,
    PATTERN_BILLOW,
    PATTERN_RIDGED,
    PATTERN_WISPS,
    PATTERN_CRESTS,
    /* Tier 2 MAPPED — bend the noise into stripes, bands, marble */
    PATTERN_CONTOURS,
    PATTERN_MARBLE,
    PATTERN_ZEBRA,
    PATTERN_RIPPLES,
    PATTERN_THRESHOLD,
    /* Tier 3 TURBULENCE — folded noise for churning, fiery looks */
    PATTERN_TURBULENCE,
    PATTERN_STORM,
    PATTERN_INFERNO,
    PATTERN_VEINS,
    PATTERN_EMBERS,
    /* Tier 4 WARPED — push the clouds around with more noise */
    PATTERN_WARP,
    PATTERN_WHIRL,
    PATTERN_DUNES,
    PATTERN_CURRENTS,
    PATTERN_FRACTAL,
    /* Tier 5 COMPOSITE — mix two or more fields together */
    PATTERN_NEBULA,
    PATTERN_AURORA,
    PATTERN_PLASMA,
    PATTERN_LIGHTNING,
    PATTERN_GALAXY,
    /* Tier 6 MASKED — fade the clouds by where they are on screen */
    PATTERN_SOLAR,
    PATTERN_MOSAIC,
    PATTERN_VIGNETTE,
    PATTERN_COSMOS,
    PATTERN_SUPERNOVA,
    N_PATTERNS,
} Pattern;

/* Both defined in §6 next to the pattern table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If the program ever freezes for a moment (debugger, heavy load), we don't
 * want it to "catch up" by running thousands of sim steps at once. Capping a
 * single frame's elapsed time at 100 ms prevents that runaway. RENDER_FPS_TARGET
 * is how often we actually redraw, even if the sim ticks faster. */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — one named color scheme for the clouds (10 of them in themes[] below).
 *
 * The renderer never knows colors directly; it just asks for "shade 0..3" and a
 * theme decides what those shades actually are. Switching themes with t/T simply
 * re-points those four shades, so the same clouds take on a totally different mood.
 *
 *   name   — short label shown in the HUD. Keep to 7 chars or fewer so the
 *            "theme:%-8s" slot doesn't overflow.
 *   band[] — the four cloud colors, dimmest first, brightest last. These are
 *            xterm 256-color numbers. Keep every one in the bright half of the
 *            palette: dark colors vanish against the black terminal background.
 *   flash  — a spare accent color. Not used here; kept so the layout matches the
 *            sister files and a future "flash on bright cells" feature can drop in.
 */
typedef struct {
    const char *name;        /* HUD label, 7 chars or fewer               */
    short       band[4];     /* the four cloud colors, dim to bright       */
    short       flash;       /* spare accent, unused here                  */
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
/* §5  noise — simplex noise plus the layered-noise (fBm) stacks          */
/* ===================================================================== */

/*
 * SimplexNoise — the only thing the noise needs to remember between calls:
 * a shuffled lookup table. Everything else (the 12 directions, the squash
 * constants) is fixed math, so it lives as file-scope constants, not here.
 *
 * Pattern functions take this as a const pointer — they read the noise but
 * never change it. The only time it changes is when the user presses 'r',
 * which reshuffles the table for a fresh-looking field.
 *
 *   perm[512] — a random shuffle of the numbers 0..255, then the same 256
 *               values copied a second time. Why store it twice? When we look
 *               up a gradient we add two table values together, and the sum can
 *               run past 255; the doubled length means that overshoot still
 *               lands on a valid slot, so we skip any wrap-around bookkeeping.
 *               uint8_t fits 0..255 exactly — 512 bytes total.
 *
 * The doubled-table trick is from Gustavson's reference implementation.
 */
typedef struct {
    uint8_t perm[512];
} SimplexNoise;

/*
 * The 12 directions a corner's gradient can point. These are the standard
 * balanced set used by Perlin/Gustavson (the duplicates pad the count to 12
 * so the lookup divides evenly). It's pure math, shared by all noise, so it's
 * a file-scope constant rather than per-instance state.
 */
static const int8_t grad2[N_GRADIENT_DIRS][2] = {
    {  1,  1 }, { -1,  1 }, {  1, -1 }, { -1, -1 },
    {  1,  0 }, { -1,  0 }, {  1,  0 }, { -1,  0 },
    {  0,  1 }, {  0, -1 }, {  0,  1 }, {  0, -1 },
};

/* Build a fresh random shuffle of the lookup table. Run on 'r' so each
 * reset gives a different-looking cloud field. (Shuffle method: Fisher-Yates.) */
static void simplex_reshuffle(SimplexNoise *sn)
{
    uint8_t base[PERM_TABLE_SIZE];
    for (int i = 0; i < PERM_TABLE_SIZE; i++) base[i] = (uint8_t)i;

    for (int i = PERM_TABLE_SIZE - 1; i > 0; i--) {
        int     j         = rand() % (i + 1);
        uint8_t swap_tmp  = base[i];
        base[i]           = base[j];
        base[j]           = swap_tmp;
    }

    /* Store the shuffle twice back-to-back; see the perm[] note above. */
    for (int i = 0; i < PERM_TABLE_SIZE; i++) {
        sn->perm[i]                    = base[i];
        sn->perm[i + PERM_TABLE_SIZE]  = base[i];
    }
}

/* ───── how one simplex noise value gets computed ────────────────────── *
 *
 * To get the noise at a point, we do four steps, one helper each:
 *   1. LOCATE   — figure out which triangle the point falls in, and where
 *                 inside it the point sits.            (simplex_locate)
 *   2. PICK     — a square holds two triangles; choose the right one and
 *                 list its 3 corners.            (simplex_pick_corners)
 *   3. EVALUATE — for each corner, measure how much it pulls on the point.
 *                                            (simplex_evaluate_corner)
 *   4. ADD UP   — sum the 3 corner pulls and scale to about -1..1.
 *                                                    (simplex_sample)
 * Two tiny helpers handle the smallest reused bits: looking up a corner's
 * gradient direction, and the per-corner pull formula.
 */

/*
 * SimplexCornerOffset — which corner of the cell we mean, as a (0/1, 0/1)
 * step from the cell's first corner.
 *
 * Making the three corners into a little array lets the main routine loop
 * over them instead of spelling out the same math three times.
 *
 *   Corner 0 is always (0, 0) — the cell's origin.
 *   Corner 1 is (1, 0) or (0, 1) depending on which of the cell's two
 *            triangles the point landed in.
 *   Corner 2 is always (1, 1) — the far corner.
 *
 *   di, dj — the step along each axis, each 0 or 1.
 */
typedef struct {
    int di;
    int dj;
} SimplexCornerOffset;

/*
 * SimplexQuery — the result of step 1: everything later steps need to know
 * about where the point landed. Bundling it means the other helpers take one
 * pointer instead of four loose numbers.
 *
 *   cell_i, cell_j        — which cell of the (squashed) triangle grid the
 *                           point is in. Also seeds the gradient lookup.
 *   corner0_dx, corner0_dy — how far the point sits from the cell's first
 *                            corner, measured in normal x/y space. Used both
 *                            to pick the triangle (point is in the lower one
 *                            when dx > dy) and as the starting offset for
 *                            every corner's pull.
 */
typedef struct {
    int   cell_i, cell_j;
    float corner0_dx, corner0_dy;
} SimplexQuery;

/* Pick a gradient direction for a corner. The double table lookup
 * (look up one number, use it to look up another) gives noise its
 * "random but repeatable" feel: the same corner always gets the same
 * direction, but neighbours look unrelated. */
static inline int simplex_hash_gradient_at(const SimplexNoise *sn,
                                           int corner_hash_i,
                                           int corner_hash_j)
{
    int row_offset = sn->perm[corner_hash_j];
    return sn->perm[corner_hash_i + row_offset] % N_GRADIENT_DIRS;
}

/* How hard one corner pulls on the point. A corner's influence fades
 * smoothly to zero as the point moves away, and is exactly zero once the
 * point is too far. Within range, the pull also depends on whether the
 * corner's gradient direction points toward or away from the point. */
static inline float simplex_corner_contribution(int gradient_idx,
                                                float dx, float dy)
{
    float radial_falloff = 0.5f - dx * dx - dy * dy;
    if (radial_falloff <= 0.0f) return 0.0f;   /* too far away to matter */
    radial_falloff *= radial_falloff;
    radial_falloff *= radial_falloff;          /* raise to the 4th power */
    float gradient_dot_displacement =
        (float)grad2[gradient_idx][0] * dx +
        (float)grad2[gradient_idx][1] * dy;
    return radial_falloff * gradient_dot_displacement;
}

/* Step 1: find which triangle cell the point is in and where inside it.
 * We squash x/y so triangles become easy to snap to, round down to get the
 * cell, then un-squash to measure the point's offset from the cell corner. */
static inline SimplexQuery simplex_locate(float xin, float yin)
{
    float skew_offset = (xin + yin) * SIMPLEX_F2;
    int   cell_i      = (int)floorf(xin + skew_offset);
    int   cell_j      = (int)floorf(yin + skew_offset);

    float unskew_offset = (float)(cell_i + cell_j) * SIMPLEX_G2;
    SimplexQuery q;
    q.cell_i     = cell_i;
    q.cell_j     = cell_j;
    q.corner0_dx = xin - ((float)cell_i - unskew_offset);
    q.corner0_dy = yin - ((float)cell_j - unskew_offset);
    return q;
}

/* Step 2: a square cell is split into two triangles by its diagonal. The
 * point is in the lower one if it's to the right of the diagonal (dx > dy),
 * the upper one otherwise. Only the middle corner differs between the two;
 * the first and last corners are always the same. */
static inline void simplex_pick_corners(float corner0_dx, float corner0_dy,
                                        SimplexCornerOffset out[N_SIMPLEX_CORNERS])
{
    bool is_lower_triangle = (corner0_dx > corner0_dy);
    out[0] = (SimplexCornerOffset){ 0, 0 };
    out[1] = is_lower_triangle
           ? (SimplexCornerOffset){ 1, 0 }
           : (SimplexCornerOffset){ 0, 1 };
    out[2] = (SimplexCornerOffset){ 1, 1 };
}

/* Step 3: one corner's contribution. Work out how far the point is from
 * this corner (in normal x/y space), look up the corner's gradient
 * direction, and feed both into the pull formula above. */
static inline float simplex_evaluate_corner(const SimplexNoise *sn,
                                            const SimplexQuery *q,
                                            SimplexCornerOffset corner)
{
    int   di            = corner.di;
    int   dj            = corner.dj;
    float corner_unskew = (float)(di + dj) * SIMPLEX_G2;
    float dx            = q->corner0_dx - (float)di + corner_unskew;
    float dy            = q->corner0_dy - (float)dj + corner_unskew;
    int   hash_i        = (q->cell_i + di) & PERM_TABLE_INDEX_MASK;
    int   hash_j        = (q->cell_j + dj) & PERM_TABLE_INDEX_MASK;
    int   gradient_idx  = simplex_hash_gradient_at(sn, hash_i, hash_j);
    return simplex_corner_contribution(gradient_idx, dx, dy);
}

/* One 2-D simplex noise value, roughly in -1..1. Just runs the four steps:
 * find the triangle, pick its corners, add up their pulls, scale the result.
 * Follows Gustavson's "Simplex Noise Demystified". */
static float simplex_sample(const SimplexNoise *sn, float xin, float yin)
{
    SimplexQuery q = simplex_locate(xin, yin);

    SimplexCornerOffset corners[N_SIMPLEX_CORNERS];
    simplex_pick_corners(q.corner0_dx, q.corner0_dy, corners);

    float corner_sum = 0.0f;
    for (int k = 0; k < N_SIMPLEX_CORNERS; k++)
        corner_sum += simplex_evaluate_corner(sn, &q, corners[k]);

    return SIMPLEX_OUTPUT_GAIN * corner_sum;
}

/* Stack several copies of the noise, each half as strong and twice as fine
 * as the last, and average them. This is what makes clouds look natural:
 * big rolling shapes with smaller wisps layered on top. (The technique is
 * called fBm; Perlin introduced it in 1985.) */
static float simplex_fbm(const SimplexNoise *sn, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * simplex_sample(sn, x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/* Same stack as simplex_fbm, but folding each layer's negative values up to
 * positive first. That fold creates sharp creases wherever the noise crosses
 * zero, giving a rougher, more billowing "storm cloud" texture. (Perlin
 * called this turbulence; it's the basis of his marble look.) */
static float simplex_fbm_abs(const SimplexNoise *sn, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * fabsf(simplex_sample(sn, x * freq, y * freq));
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — the 30 "looks", in 6 tiers, plus the lookup table       */
/* ===================================================================== */

/*
 * Every pattern function has the same shape: (sn, fx, fy, t, nx, ny).
 *   sn      — the noise to sample (read-only).
 *   fx, fy  — where to sample in noise space (the cell position scaled down).
 *   t       — the drift amount, so the field keeps moving over time.
 *   nx, ny  — the cell's position on screen as -1..1 with the centre at (0,0).
 *             Only the patterns that care about "where on screen" (galaxy,
 *             solar, aurora, vignette, supernova, inferno) use these; the
 *             rest cast them to (void) to say "ignored".
 * Each returns a brightness from 0 (dark) to 1 (bright).
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline float smoothstepf(float e0, float e1, float x)
{
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* ---------- Tier 1  RAW — the noise shown nearly straight ------------- */

/*
 * CLOUDS — the plain layered noise, shifted into 0..1. Soft puffy clouds;
 * the starting point every other pattern builds on.
 */
static float pattern_clouds(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
}

/*
 * BILLOW — fold the dark (negative) parts up to bright. Bumpy puffs with
 * dark creases between them.
 */
static float pattern_billow(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(fabsf(simplex_fbm(sn, x, y + t)), 0.0f, 1.0f);
}

/*
 * RIDGED — BILLOW flipped upside down: bright thin ridges with dark valleys.
 * Looks like wispy cirrus cloud. (Known as ridged multifractal, from Musgrave.)
 */
static float pattern_ridged(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(1.0f - fabsf(simplex_fbm(sn, x, y + t)), 0.0f, 1.0f);
}

/*
 * WISPS — stretch the clouds sideways so they read as long streaks rather
 * than round puffs, and drift a bit faster so they seem to fly past.
 */
static float pattern_wisps(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    return simplex_fbm(sn, x * WISPS_X_SCALE, y + t * 1.5f) * 0.5f + 0.5f;
}

/*
 * CRESTS — RIDGED pushed harder (raised to a power) so the ridges turn
 * razor-thin and the gaps go nearly black, like windblown snow crests.
 */
static float pattern_crests(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float r = 1.0f - fabsf(simplex_fbm(sn, x, y + t));
    r *= r; r *= r;                          /* pow 4 */
    return clampf(r, 0.0f, 1.0f);
}

/* ---------- Tier 2  MAPPED — bent into stripes, bands, marble --------- */

/*
 * CONTOURS — treat the noise as terrain height and draw repeating
 * "elevation lines", like the rings on a topographic map.
 */
static float pattern_contours(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 8.0f;
    return v - floorf(v);
}

/*
 * MARBLE — start with clean vertical stripes, then let the noise nudge them
 * sideways so they bend and swirl like the veins in polished marble.
 */
static float pattern_marble(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t);
    return sinf(x * 3.0f + v * 6.0f) * 0.5f + 0.5f;
}

/*
 * ZEBRA — like MARBLE but with no in-between: every cell is forced to
 * either bright or dark, giving hard stripes that twist with the noise.
 */
static float pattern_zebra(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = sinf(simplex_fbm(sn, x, y + t) * 12.0f);
    return v > 0.0f ? 0.90f : 0.10f;
}

/*
 * RIPPLES — many tightly-packed smooth rings, like raindrops spreading on a
 * pond. Same idea as CONTOURS but with soft bands instead of sharp lines.
 */
static float pattern_ripples(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx; (void)ny;
    return sinf(simplex_fbm(sn, x, y + t) * 16.0f) * 0.5f + 0.5f;
}

/*
 * THRESHOLD — cut the clouds into solid "islands": anything above a brightness
 * cutoff is on, below is off, with a soft edge. The islands morph as it drifts.
 */
static float pattern_threshold(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return smoothstepf(0.45f, 0.55f, v);
}

/* ---------- Tier 3  TURBULENCE — folded noise, churning looks --------- */

/*
 * TURBULENCE — the folded noise stack shown straight: the creases give it a
 * churning, sharp-edged "storm cloud" look.
 */
static float pattern_turbulence(const SimplexNoise *sn, float x, float y, float t,
                                float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(simplex_fbm_abs(sn, x, y + t), 0.0f, 1.0f);
}

/*
 * STORM — turbulence with the contrast cranked up: mid-tones drop away and
 * the bright cores pop, for a darker, more violent storm.
 */
static float pattern_storm(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t);
    return clampf(v * v * 1.5f, 0.0f, 1.0f);
}

/*
 * INFERNO — turbulence that's brightest near the bottom of the screen and
 * drifts upward fast, so it reads as rising flames and heat.
 */
static float pattern_inferno(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx;
    float turb = simplex_fbm_abs(sn, x, y + t * 2.0f);
    float rise = (1.0f - ny) * 0.5f + 0.25f; /* brighter toward the bottom */
    return clampf(turb * rise * 1.3f, 0.0f, 1.0f);
}

/*
 * VEINS — flip turbulence so its creases become thin dark lines on a bright
 * field — like the veins of a leaf or branching rivers.
 */
static float pattern_veins(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t);
    return clampf(1.0f - sqrtf(v + 0.05f), 0.0f, 1.0f);
}

/*
 * EMBERS — dim almost everything down, but let the brightest bits flare up,
 * so a few glowing embers stand out over a dark field.
 */
static float pattern_embers(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t * 1.5f);
    if (v > 0.55f) return clampf((v - 0.55f) * 3.0f + 0.5f, 0.0f, 1.0f);
    return v * 0.4f;
}

/* ---------- Tier 4  WARPED — push the lookup around with noise -------- */

/*
 * WARP — domain warping: instead of asking "what's the cloud here", we first
 * use noise to nudge the lookup spot, then sample there. The clouds look like
 * they're being pushed around by invisible currents. (IQ's technique.)
 */
static float pattern_warp(const SimplexNoise *sn, float x, float y, float t,
                          float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y + t);
    float qy = simplex_sample(sn, x + 5.2f, y + 1.3f + t);
    return simplex_fbm(sn, x + qx, y + qy) * 0.5f + 0.5f;
}

/*
 * WHIRL — let noise decide how much to spin each spot before sampling. The
 * varying spin creates big readable swirls and whirlpools.
 */
static float pattern_whirl(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float a  = simplex_sample(sn, x * 0.5f, y * 0.5f + t) * 2.0f * (float)M_PI;
    float ca = cosf(a), sa = sinf(a);
    float wx = x * ca - y * sa;
    float wy = x * sa + y * ca;
    return simplex_fbm(sn, wx, wy + t) * 0.5f + 0.5f;
}

/*
 * DUNES — warp strongly sideways only, and stretch vertically, so you get
 * long curved ridges nesting into each other like sand dunes from a low angle.
 */
static float pattern_dunes(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y) * 2.0f;
    return simplex_fbm(sn, x + qx + t, y * 0.5f) * 0.5f + 0.5f;
}

/*
 * CURRENTS — a gentle warp in both directions, so the field flows slowly
 * rather than churning. Reads like a slow river delta or ocean current.
 */
static float pattern_currents(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x * 2.0f, y * 2.0f + t);
    float qy = simplex_sample(sn, x * 2.0f + 5.0f, y * 2.0f + 5.0f + t);
    return simplex_fbm(sn, x + 0.4f * qx, y + 0.4f * qy + t) * 0.5f + 0.5f;
}

/*
 * FRACTAL — warp the warp: do WARP's nudge twice over, which produces deeply
 * nested swirls. It's the most expensive pattern (the most noise lookups per
 * cell), but still fast enough to stay smooth.
 */
static float pattern_fractal(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y + t);
    float qy = simplex_sample(sn, x + 5.2f, y + 1.3f + t);
    float rx = simplex_sample(sn, x + 2.0f * qx + 1.7f, y + 2.0f * qy + 9.2f + t);
    float ry = simplex_sample(sn, x + 2.0f * qx + 8.3f, y + 2.0f * qy + 2.8f + t);
    return simplex_fbm(sn, x + 2.0f * rx, y + 2.0f * ry + t) * 0.5f + 0.5f;
}

/* ---------- Tier 5  COMPOSITE — several fields mixed together --------- */

/*
 * NEBULA — multiply two separate cloud fields together. A spot is only bright
 * where both happen to be bright, so you get sparse glowing cores in dark space.
 */
static float pattern_nebula(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float a = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    float b = simplex_fbm(sn, x * 2.0f + 7.0f, y * 2.0f + 3.0f + t) * 0.5f + 0.5f;
    return clampf(a * b * 2.0f, 0.0f, 1.0f);
}

/*
 * AURORA — wavy horizontal bands of light, kept brightest across the middle of
 * the screen and fading top and bottom — curtains of light over a night sky.
 */
static float pattern_aurora(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx;
    float band = sinf(x * 1.5f + simplex_fbm(sn, x, y + t) * 4.0f) * 0.5f + 0.5f;
    float vert = expf(-ny * ny * 1.5f);
    return clampf(band * vert * 1.2f, 0.0f, 1.0f);
}

/*
 * PLASMA — the classic demoscene plasma effect: overlap three wave patterns,
 * wobbled by noise, so they interfere into soft shifting blobs of color.
 */
static float pattern_plasma(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t);
    float s = sinf(x * 2.0f + v * 3.0f)
            + cosf(y * 2.5f + v * 3.0f)
            + sinf((x + y) * 1.5f + v * 2.0f);
    return (s + 3.0f) / 6.0f;
}

/*
 * LIGHTNING — RIDGED pushed to an extreme: everything but the very sharpest
 * ridges collapses to black, leaving thin branching streaks like lightning.
 */
static float pattern_lightning(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    (void)nx; (void)ny;
    float r = 1.0f - fabsf(simplex_fbm(sn, x, y + t));
    float p = r * r;     /*  ^2  */
    p *= p;              /*  ^4  */
    p *= p;              /*  ^8  */
    p *= r * r * r * r;  /* ^12  */
    return clampf(p * 1.8f, 0.0f, 1.0f);
}

/*
 * GALAXY — think in terms of distance and angle from the screen centre, and
 * draw spiral arms that wind outward, fading toward the edges.
 */
static float pattern_galaxy(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    float r      = sqrtf(nx * nx + ny * ny);
    float a      = atan2f(ny, nx);
    float spiral = sinf(a * 2.0f + r * 6.0f + simplex_fbm(sn, x, y + t) * 3.0f);
    return clampf((spiral * 0.5f + 0.5f) * expf(-r * 0.5f) * 1.4f,
                  0.0f, 1.0f);
}

/* ---------- Tier 6  MASKED — faded by position on screen ------------- */

/*
 * SOLAR — churning turbulence kept bright at the centre and fading to dark at
 * the edges, drifting fast so it boils like the surface of the sun.
 */
static float pattern_solar(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    float r      = sqrtf(nx * nx + ny * ny);
    float corona = simplex_fbm_abs(sn, x, y + t * 2.0f);
    float mask   = expf(-r * 1.2f);
    return clampf(corona * mask * 1.8f, 0.0f, 1.0f);
}

/*
 * MOSAIC — round the cloud brightness to just 6 fixed levels, so smooth
 * shading becomes flat patches with hard edges, like stained glass.
 */
static float pattern_mosaic(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return floorf(v * 6.0f) / 5.0f;
}

/*
 * VIGNETTE — plain clouds, but darkened toward the corners, the way a camera
 * lens dims the edges of a photo. Subtle; a nice baseline to compare against.
 */
static float pattern_vignette(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    float r2   = nx * nx + ny * ny;
    float v    = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    float mask = clampf(1.0f - r2 * 0.8f, 0.0f, 1.0f);
    return v * mask;
}

/*
 * COSMOS — a faint cloud glow in the background, with sparse bright "stars"
 * lit only where a fine, fast noise happens to spike. A starfield over a nebula.
 */
static float pattern_cosmos(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float bg    = simplex_fbm(sn, x, y + t * 0.3f) * 0.5f + 0.5f;
    float stars = simplex_sample(sn, x * 6.0f + 13.0f, y * 6.0f + 7.0f + t * 0.5f);
    bg *= 0.3f;
    if (stars > 0.6f) bg += (stars - 0.6f) * 2.5f;
    return clampf(bg, 0.0f, 1.0f);
}

/*
 * SUPERNOVA — eight bright spokes radiating from the centre, with the whole
 * thing pulsing brighter and dimmer over time and a little noise mixed in.
 */
static float pattern_supernova(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    float r     = sqrtf(nx * nx + ny * ny);
    float a     = atan2f(ny, nx);
    float rays  = sinf(a * 8.0f) * 0.5f + 0.5f;
    float pulse = sinf(t * 2.0f) * 0.3f + 0.7f;
    float n     = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return clampf((rays * pulse + n * 0.4f) * expf(-r * 0.8f) * 2.0f,
                  0.0f, 1.0f);
}

/* ---------- pattern lookup table -------------------------------------- */
/* The shared shape of every pattern function (see the note above clampf). */
typedef float (*NoisePatternFn)(const SimplexNoise *sn,
                                float fx, float fy, float t,
                                float nx, float ny);

/*
 * NoisePattern — one row of the pattern table: a display name, a tier
 * label, and the function that draws it. Using a table instead of a giant
 * switch means each pattern's name, tier, and code sit together, and the
 * Pattern enum just indexes into it.
 *
 *   name   — shown in the HUD. Padded to a fixed 10 characters so the HUD
 *            doesn't jiggle as you cycle through names of different lengths.
 *   tier   — short "N-LABEL" tag, same fixed-width reason.
 *   sample — the function that turns a cell position into a brightness.
 */
typedef struct {
    const char     *name;
    const char     *tier;
    NoisePatternFn  sample;
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — RAW */
    [PATTERN_CLOUDS]     = { "CLOUDS    ", "1-RAW  ", pattern_clouds     },
    [PATTERN_BILLOW]     = { "BILLOW    ", "1-RAW  ", pattern_billow     },
    [PATTERN_RIDGED]     = { "RIDGED    ", "1-RAW  ", pattern_ridged     },
    [PATTERN_WISPS]      = { "WISPS     ", "1-RAW  ", pattern_wisps      },
    [PATTERN_CRESTS]     = { "CRESTS    ", "1-RAW  ", pattern_crests     },
    /* Tier 2 — MAPPED */
    [PATTERN_CONTOURS]   = { "CONTOURS  ", "2-MAP  ", pattern_contours   },
    [PATTERN_MARBLE]     = { "MARBLE    ", "2-MAP  ", pattern_marble     },
    [PATTERN_ZEBRA]      = { "ZEBRA     ", "2-MAP  ", pattern_zebra      },
    [PATTERN_RIPPLES]    = { "RIPPLES   ", "2-MAP  ", pattern_ripples    },
    [PATTERN_THRESHOLD]  = { "THRESHOLD ", "2-MAP  ", pattern_threshold  },
    /* Tier 3 — TURBULENCE */
    [PATTERN_TURBULENCE] = { "TURBULENCE", "3-TURB ", pattern_turbulence },
    [PATTERN_STORM]      = { "STORM     ", "3-TURB ", pattern_storm      },
    [PATTERN_INFERNO]    = { "INFERNO   ", "3-TURB ", pattern_inferno    },
    [PATTERN_VEINS]      = { "VEINS     ", "3-TURB ", pattern_veins      },
    [PATTERN_EMBERS]     = { "EMBERS    ", "3-TURB ", pattern_embers     },
    /* Tier 4 — WARPED */
    [PATTERN_WARP]       = { "WARP      ", "4-WARP ", pattern_warp       },
    [PATTERN_WHIRL]      = { "WHIRL     ", "4-WARP ", pattern_whirl      },
    [PATTERN_DUNES]      = { "DUNES     ", "4-WARP ", pattern_dunes      },
    [PATTERN_CURRENTS]   = { "CURRENTS  ", "4-WARP ", pattern_currents   },
    [PATTERN_FRACTAL]    = { "FRACTAL   ", "4-WARP ", pattern_fractal    },
    /* Tier 5 — COMPOSITE */
    [PATTERN_NEBULA]     = { "NEBULA    ", "5-COMP ", pattern_nebula     },
    [PATTERN_AURORA]     = { "AURORA    ", "5-COMP ", pattern_aurora     },
    [PATTERN_PLASMA]     = { "PLASMA    ", "5-COMP ", pattern_plasma     },
    [PATTERN_LIGHTNING]  = { "LIGHTNING ", "5-COMP ", pattern_lightning  },
    [PATTERN_GALAXY]     = { "GALAXY    ", "5-COMP ", pattern_galaxy     },
    /* Tier 6 — MASKED */
    [PATTERN_SOLAR]      = { "SOLAR     ", "6-MASK ", pattern_solar      },
    [PATTERN_MOSAIC]     = { "MOSAIC    ", "6-MASK ", pattern_mosaic     },
    [PATTERN_VIGNETTE]   = { "VIGNETTE  ", "6-MASK ", pattern_vignette   },
    [PATTERN_COSMOS]     = { "COSMOS    ", "6-MASK ", pattern_cosmos     },
    [PATTERN_SUPERNOVA]  = { "SUPERNOVA ", "6-MASK ", pattern_supernova  },
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
/* §7  scene — the simulation state and the per-tick field update          */
/* ===================================================================== */

/*
 * GlowField — the grid of computed brightness, one value per terminal cell.
 * The sim writes it; the renderer reads it. Keeping it as a middle layer means
 * "compute the clouds" and "draw the clouds" stay separate jobs.
 *
 * The buffers are sized for the largest possible grid and live in fixed
 * storage, so we never allocate memory while running (about 88 KB worst case).
 *
 *   w, h   — current grid size in cells. Set at startup and on every resize.
 *   count  — w * h, kept around so loops don't keep recomputing it.
 *   glow[] — each cell's brightness, 0 (dark) to 1 (bright). Index a cell with
 *            glow_field_idx(gf, x, y).
 *   band[] — each cell's color shade (0..3), worked out once here so the
 *            renderer doesn't have to redo it. One byte each is plenty.
 */
typedef struct {
    int      w, h;
    int      count;
    float    glow[CELLS_MAX];
    uint8_t  band[CELLS_MAX];
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
 * GlyphRamp — the rule for turning a cell's brightness into a character.
 * Pulling it into a named thing keeps the cutoff values out in the open and
 * lets a future theme swap in different characters.
 *
 * Brightest cells get '#', medium get '*', faint get '.', and anything below
 * the lowest cutoff is left blank so the background shows through (that's why
 * patterns like NEBULA have visible empty space). The cutoffs must stay in
 * order, brightest first.
 *
 * The three characters are a coarse slice of Paul Bourke's brightness ramp.
 * Three steps is enough to read clearly in a terminal; more wouldn't show.
 */
typedef struct {
    float thresh_high;    /* above this -> '#' */
    float thresh_mid;     /* above this -> '*' */
    float thresh_low;     /* above this -> '.' */
    char  glyph_high;
    char  glyph_mid;
    char  glyph_low;
} GlyphRamp;

/*
 * GlyphChoice — what glyph_ramp_pick() hands back: which character to draw,
 * whether to draw it bold, and whether to draw it at all. Returning a little
 * struct lets the caller read it like a plain answer. Note it carries no
 * color: the ramp only decides the character, color comes from the theme.
 */
typedef struct {
    char glyph;         /* the character to draw (only if visible)    */
    int  attr;          /* bold or normal                             */
    bool visible;       /* false -> leave the cell blank              */
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
 * PatternState — which pattern is showing and how it's moving. These three
 * values change together as you interact, so they live together.
 *
 *   current    — the pattern on screen; cycled with n/p.
 *   field_time — how far the field has drifted so far. Each tick we add a bit
 *                and use it to shift where we sample, which makes the clouds
 *                scroll. It only grows; 'r' resets it to zero. A float is fine
 *                even after hours, because the noise repeats every so often,
 *                so a big value just wraps harmlessly.
 *   drift_mult — the speed dial, doubled/halved with +/- between 1 and 32.
 *                Stepping by doubling makes the speed changes feel distinct.
 */
typedef struct {
    Pattern current;
    float   field_time;
    int     drift_mult;
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current    = PATTERN_CLOUDS;
    ps->field_time = 0.0f;
    ps->drift_mult = DRIFT_MULT_DEF;
}

/*
 * PaletteState — which theme is showing, as an index into themes[]. It's just
 * one number today, but it's kept as its own little struct so "the current
 * color scheme" has a clear home (and room to grow, e.g. theme crossfades).
 * Cycled with t/T; changing it re-binds the ncurses colors via theme_apply().
 */
typedef struct {
    int current;          /* index into themes[] */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — one struct holding everything the running simulation cares about.
 * Functions take a Scene* instead of a long list of arguments. Reading the
 * fields top to bottom traces the flow of work:
 *
 *   noise   — the random source the patterns sample
 *   pattern — which pattern is showing and how fast it drifts
 *   field   — the brightness computed for every cell
 *   ramp    — how brightness becomes a character
 *   palette — which color theme is in effect
 *   paused  — when true, the field stops updating
 *
 * The field grid is by far the biggest part; it sits early in the struct so
 * the small parts the inner loop reads first stay together in cache. Nothing
 * here is allocated at runtime; it all lives inside the App in §9.
 */
typedef struct {
    SimplexNoise  noise;
    GlowField     field;
    GlyphRamp     ramp;
    PatternState  pattern;
    PaletteState  palette;
    bool          paused;
} Scene;

/* Turn a cell number into the matching spot in noise space — this is the
 * (x, y) we hand to a pattern function. */
static inline float cell_to_noise_coord(int cell)
{
    return (float)cell * NOISE_SCALE;
}

/* Turn a cell number into a -1..1 position on screen with 0 at the centre.
 * Patterns that care about "where on screen" (galaxy, solar, etc.) use this. */
static inline float cell_to_normalized_coord(int cell, int n_cells)
{
    if (n_cells <= 1) return 0.0f;
    return (float)cell * (2.0f / (float)(n_cells - 1)) - 1.0f;
}

/* Turn a brightness (0..1) into one of the four color shades. See the
 * "almost 4" note back in §1 for why the constant isn't exactly 4. */
static inline uint8_t glow_to_palette_band(float glow)
{
    return (uint8_t)((int)(glow * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK);
}

/* Recompute the whole cloud field for one sim step: for every cell, ask the
 * current pattern how bright it is, clamp it, sort it into a color shade, and
 * store both. This runs over every cell each tick, so it's kept tight. */
static void scene_evaluate_field(Scene *s)
{
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    NoisePatternFn      sample_pattern = noise_patterns[active].sample;
    const SimplexNoise *noise          = &s->noise;
    GlowField          *field          = &s->field;
    float               drift          = s->pattern.field_time;

    for (int y = 0; y < field->h; y++) {
        float ny = cell_to_normalized_coord(y, field->h);
        float fy = cell_to_noise_coord(y);
        for (int x = 0; x < field->w; x++) {
            float nx   = cell_to_normalized_coord(x, field->w);
            float fx   = cell_to_noise_coord(x);
            float glow = clampf(sample_pattern(noise, fx, fy, drift, nx, ny),
                                0.0f, 1.0f);

            int idx          = glow_field_idx(field, x, y);
            field->glow[idx] = glow;
            field->band[idx] = glow_to_palette_band(glow);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->pattern.field_time = 0.0f;
    simplex_reshuffle(&s->noise);
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
 * Screen — just the terminal's current width and height, remembered so we
 * don't ask ncurses for them once per cell. ncurses itself owns the actual
 * drawing buffer and colors; this is only the size.
 *
 *   cols — width in characters (kept at least 16, or the HUD won't fit).
 *   rows — height in characters (kept at least 8).
 *
 * Refreshed on every terminal resize.
 */
typedef struct {
    int cols;
    int rows;
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

/* ---------- drawing the cloud field ----------------------------------- */

/*
 * GridPlacement — the top-left corner on screen where the field starts, after
 * centering it and leaving the HUD rows clear. Working it out once (instead of
 * per cell) keeps the draw loop simple: a cell's screen row is just origin_y + y.
 *
 *   origin_x — leftmost column the field starts at (0 if it's wider than screen).
 *   origin_y — topmost row (never above the title bar).
 *
 * If the field is bigger than the terminal, these clamp to the edge and the
 * field gets cropped rather than wrapping around.
 */
typedef struct {
    int origin_x;
    int origin_y;
} GridPlacement;

/* Centre the field in the terminal, keeping the HUD rows clear. An over-large
 * field is pinned to the edge and cropped, not wrapped. */
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

/* Draw a single character in a given color. */
static inline void draw_glyph_at(int screen_y, int screen_x,
                                 char glyph, int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(screen_y, screen_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* Paint the cloud field into the terminal: centre it, then for each cell pick a
 * character from its brightness and draw it in its color shade (skipping the
 * dim cells the ramp marks invisible). Read-only; no simulation happens here. */
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

            int color_pair = PAIR_BAND_BASE
                           + (field->band[cell_idx] & PALETTE_BAND_MASK);
            draw_glyph_at(screen_y, screen_x,
                          pick.glyph, color_pair, pick.attr);
        }
    }
}

/* The status row is drawn left to right in chunks; each number is how wide its
 * chunk is, so the next one starts in the right place. Named so a wording tweak
 * is a one-line change. */
#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " SIMPLEX NOISE CLOUDS "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* ---------- HUD: per-segment drawers ---------------------------------- *
 * Each one draws its chunk at (row, x) and returns the x to continue from,
 * so the caller can chain them across the row.
 */

/* Row 0 left — the program title. */
static int hud_draw_title_chip(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, "%s", HUD_TITLE_TEXT);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + (int)strlen(HUD_TITLE_TEXT);
}

/* Row 0 right — frame rate, tick rate, current pattern, and drift speed. */
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

/* Row 1 segment — "pattern:<NAME>" */
static int hud_draw_pattern_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-10s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_PATTERN_FIELD;
}

/* Row 1 segment — "tier:<N-LABEL>" */
static int hud_draw_tier_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " tier:%-7s ", pattern_tier(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_TIER_FIELD;
}

/* Row 1 segment — "theme:<NAME>" */
static int hud_draw_theme_field(int row, int x, int theme_idx)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_THEME_FIELD;
}

/* Row 1 segment — "palette:" plus one '#' per color shade, each in its own
 * color, so you can see at a glance what the four cloud shades look like now. */
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

/* Row 1 tail — a readout of the noise settings: scale, layer count, grid size. */
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

/* ---------- screen_draw: scene + full HUD ----------------------------- */

/* Draw one whole frame: clear, paint the clouds, then lay the HUD on top
 * (title and status up top, key hints along the bottom). Read-only. */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    /* Row 0 — title + state bar */
    hud_draw_title_chip(HUD_TITLE_ROW, HUD_LEFT_MARGIN);
    hud_draw_state_bar (HUD_TITLE_ROW, sc->cols, fps, sim_fps,
                        &s->pattern, s->paused);

    /* Row 1 — chained left-to-right segments */
    int x = HUD_LEFT_MARGIN;
    x = hud_draw_pattern_field   (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_tier_field      (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_theme_field     (HUD_STATUS_ROW, x, s->palette.current);
    x = hud_draw_palette_swatches(HUD_STATUS_ROW, x);
    hud_draw_stats_field         (HUD_STATUS_ROW, x, &s->field);

    /* Bottom row — keymap hint */
    hud_draw_action_hint(sc->rows - 1);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the one struct that holds everything: the simulation, the terminal
 * size, the tick rate, the chosen grid size, and two flags the main loop
 * watches. main() only ever touches this.
 *
 * There's a single file-scope instance (g_app below). It has to be file-scope
 * because the signal handlers need to reach the two flags, and signal handlers
 * can't be handed a pointer.
 *
 *   scene       — the simulation (see Scene).
 *   screen      — terminal width/height; refreshed when the terminal resizes.
 *   sim_fps     — how many times per second the sim updates; ]/[ change it.
 *   map_w/map_h — the grid size, derived from the terminal minus the HUD rows.
 *   running     — set to 0 to quit (by Ctrl-C, the q key, etc).
 *   need_resize — set to 1 when the terminal is resized; the loop acts on it.
 *
 * The two flags are volatile sig_atomic_t because a signal handler can flip
 * them at any moment, and that type is the only one safe to write from a
 * handler; volatile stops the loop from caching a stale copy.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w;
    int                   map_h;
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

/* ---------- key actions ----------------------------------------------- *
 * One small function per thing a key does, so the key handler below reads as a
 * plain list. The +1/-1 direction argument means "next" or "previous".
 */

/* Step to the next/previous pattern, wrapping around the ends. */
static void scene_cycle_pattern(Scene *s, int direction)
{
    int next = ((int)s->pattern.current + direction + N_PATTERNS) % N_PATTERNS;
    s->pattern.current = (Pattern)next;
}

/* Step to the next/previous theme and apply its colors. */
static void scene_cycle_theme(Scene *s, int direction)
{
    s->palette.current = (s->palette.current + direction + N_THEMES)
                         % N_THEMES;
    theme_apply(s->palette.current);
}

/* Drift speed doubles or halves, so each press is a clear step, not a nudge. */
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

/* Nudge the sim's update rate up or down, kept within sane limits. */
static void app_adjust_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Route a keypress to its action. Returns false only when the user wants out. */
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

/* ---------- the main loop's timekeeping ------------------------------- */

/*
 * FrameClock — the handful of timing values the main loop keeps between
 * frames, bundled together so the loop can advance them with named steps.
 *
 * It does two separate jobs that both feed off "how long the last frame took":
 *
 *   1. Keep the sim updating at a steady pace. We pour each frame's elapsed
 *      time into a bucket and spend it one fixed tick at a time, so the sim
 *      always sees an even step even when frames are uneven. (This is the
 *      well-known "fix your timestep" pattern from Glenn Fiedler.)
 *   2. Measure the frame rate to show in the HUD.
 *
 *   frame_time_ns   — the clock reading at the start of this frame; next
 *                     frame subtracts it to learn how long this one took.
 *   sim_accum_ns    — leftover time waiting to be spent on sim ticks.
 *   fps_accum_ns    — time piled up since we last recomputed the frame rate.
 *   fps_frame_count — frames counted since then.
 *   fps_display     — the latest frame rate; the only field the HUD reads.
 */
typedef struct {
    int64_t frame_time_ns;
    int64_t sim_accum_ns;
    int64_t fps_accum_ns;
    int     fps_frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *fc)
{
    fc->frame_time_ns   = clock_ns();
    fc->sim_accum_ns    = 0;
    fc->fps_accum_ns    = 0;
    fc->fps_frame_count = 0;
    fc->fps_display     = 0.0;
}

/* Reset the clock after a pause (like a resize) so we don't try to "catch up"
 * on all the time that passed while nothing was being shown. */
static void frame_clock_resync(FrameClock *fc)
{
    fc->frame_time_ns = clock_ns();
    fc->sim_accum_ns  = 0;
}

/* Start a new frame; return how long since the last one, capped so a long
 * freeze can't make the sim try to catch up all at once. */
static int64_t frame_clock_advance(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->frame_time_ns;
    fc->frame_time_ns = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* Count frames and elapsed time; every half second or so, work out the frame
 * rate from them and refresh the number the HUD shows. */
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

/* ---------- main-loop steps ------------------------------------------- */

/* Spend this frame's elapsed time on sim updates, one fixed-size tick at a
 * time, so the sim always advances by an even step no matter how jumpy the
 * frame rate is. */
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

/* Sleep just long enough to hold the redraw rate steady: figure out how much
 * of this frame's time budget is already used up and wait out the rest. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Grab a key if one's waiting and act on it; quitting stops the main loop. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Hook up Ctrl-C / kill to quit, and terminal-resize to the resize flag.
 * The handlers only flip one flag each, which is all that's safe to do here. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/*
 * main — set things up, then loop: handle any resize, update the sim by the
 * right amount, draw a frame, read input, repeat until the user quits.
 */
int main(void)
{
    /* Seed randomness, make sure the terminal gets restored on exit, listen
     * for signals. */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    /* Start the terminal, size the grid to fit, build the simulation. */
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
