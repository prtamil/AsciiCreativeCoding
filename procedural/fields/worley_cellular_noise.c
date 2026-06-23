/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * worley_cellular_noise.c — Steven Worley's cellular noise, the "scatter
 * random dots and shade every point by how far it is from the nearest dot"
 * texture. One hidden dot per grid tile; 30 different ways to turn those
 * distances into a picture, in 6 groups you cycle with n/p. The dots drift
 * slowly so the whole thing breathes instead of freezing.
 *
 * Sister files (the comparison the code can't show you):
 *   ./domain_warped_noise_iq_style.c — Perlin noise: smooth rolling hills
 *       instead of these hard-edged cells. Same "shade every cell" loop.
 *   ../generational/voronoi_region_map.c — the same cell idea, but
 *       precomputed once into a static map; this file computes on the fly.
 *
 * Reference: Worley, S. (1996) "A cellular texture basis function",
 *   SIGGRAPH'96 — the original paper. The F1/F2/F3 naming and the
 *   3x3-tile search both come from here.
 *   https://dl.acm.org/doi/10.1145/237170.237267
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra worley_cellular_noise.c \
 *       -o worley -lncurses -lm
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

    /* ncurses colour-pair slots. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* the 4 palette colours live in slots 3..6 */
    PAIR_FLASH        =   7,    /* unused here; kept so themes match sibling demos */
};

#define GLOW_THRESHOLD      0.05f

/* The HUD reserves two rows at the top (title + status) and one at the
 * bottom (key hints); the noise fills everything in between. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* How zoomed-in the noise is: one tile (one hidden dot) every ~10 cells. */
#define NOISE_SCALE         0.10f

/* How fast the dots drift over time (the "breathing" speed). */
#define FIELD_DRIFT         0.40f

/* How far each dot wanders from its home spot. Keep under ~0.4 so dots
 * stay roughly inside their own tile — that's what lets the 3x3 search
 * below find the true nearest dot. */
#define WOBBLE_AMOUNT       0.20f

/* Drift-speed multiplier, nudged by +/-. Capped low so cells don't boil. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      16

/* Brightness cutoffs for the three ASCII glyphs (faint/medium/dense). */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* Turning a brightness in [0,1] into one of 4 colour tiers. The gain is
 * just under 4 so a brightness of exactly 1.0 lands in tier 3, not 4.
 * The mask is a fast stand-in for "mod 4" (works because 4 is a power
 * of two). */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3       /* N_PALETTE_BANDS - 1 */
#define GLOW_TO_BAND_GAIN   3.999f

/* A starting "distance" so big that the first real dot always beats it
 * (any real distance here is under 5). The search looks at the 3x3 block
 * of tiles around a point — the nearest dot can't be further out than
 * that as long as the wobble stays small. */
#define WORLEY_DISTANCE_INF      1.0e10f
#define WORLEY_NEIGHBOUR_RADIUS  1   /* 1 tile in each direction = 3x3 block */
#define N_WORLEY_NEIGHBOURS      9   /* (2*1+1)^2 tiles checked per point */

/* Pulling small random numbers out of a 32-bit hash:
 *   FROM_16BIT — a 16-bit chunk (0..65535) becomes a float in [0,1].
 *   FROM_BYTE  — one byte (0..255) becomes a float in [0,1].
 *   PHASE_FROM_BYTE — one byte becomes an angle, so different dots wobble
 *                     out of step with each other.
 *   PERSONALITY_SALT — mixed into a tile's id before re-hashing, so two
 *                     neighbouring cells don't end up with similar values. */
#define HASH_UNIT_FROM_16BIT    (1.0f / 65535.0f)
#define HASH_UNIT_FROM_BYTE     (1.0f / 255.0f)
#define HASH_PHASE_FROM_BYTE    (1.0f / 40.0f)
#define HASH_PERSONALITY_SALT   0xa5a5a5a5u

/* Weighted variant (Tier 4 WEIGHTED): each dot gets a "pull" between 0.5
 * and 1.5 from its hash. Dots with a stronger pull grab more territory,
 * so cells come out unevenly sized instead of all about the same. */
#define WEIGHTED_W_MIN          0.5f
#define WEIGHTED_W_RANGE        1.0f   /* so the max pull is 1.5 */

/* Stretched variant (Tier 3 STRETCHED): horizontal steps count double,
 * which squashes the cells taller and thinner. */
#define STRETCHED_X_WEIGHT      2.0f

/*
 * Pattern — the 30 ways to turn the distances into a picture, in 6
 * groups. Cycle with n/p. This list MUST stay in the same order as the
 * noise_patterns[] table in §6 — the compiler catches a mismatch because
 * that table is a fixed [N_PATTERNS] array indexed by these names.
 */
typedef enum {
    /* Tier 1 — the raw distances: nearest dot, 2nd-nearest, 3rd, and
     * a couple of simple combinations of them. */
    PATTERN_F1 = 0,
    PATTERN_F2,
    PATTERN_F3,
    PATTERN_F2_F1,
    PATTERN_F1_OVER_F2,
    /* Tier 2 — more arithmetic on those same distances. */
    PATTERN_F1_PLUS_F2,
    PATTERN_F1_TIMES_F2,
    PATTERN_F3_F2,
    PATTERN_F2_F1_INV,
    PATTERN_F1_INV,
    /* Tier 3 — same idea, different way of measuring distance, which
     * changes the cell shape (diamonds, squares, stars...). */
    PATTERN_MANHATTAN,
    PATTERN_CHEBYSHEV,
    PATTERN_SUPERELLIPSE,
    PATTERN_STAR,
    PATTERN_STRETCHED,
    /* Tier 4 — each cell's own random id picks its look (a fixed colour,
     * its own flicker, etc.). */
    PATTERN_CELL_ID,
    PATTERN_WEIGHTED,
    PATTERN_TWINKLE,
    PATTERN_RANDOM_GLOW,
    PATTERN_CRACKLE,
    /* Tier 5 — stack the noise at several zoom levels and add it up. */
    PATTERN_WORLEY_FBM,
    PATTERN_WORLEY_TURBULENCE,
    PATTERN_WORLEY_RIDGED,
    PATTERN_NESTED,
    PATTERN_CHECKER,
    /* Tier 6 — feed one noise field into another to smear or twist it. */
    PATTERN_DOMAIN_WARP,
    PATTERN_METRIC_BLEND,
    PATTERN_HALO,
    PATTERN_ENERGY,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* Defined in §6 next to the table they read from. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

/*
 * Metric — the rule for "how far apart are two points", which decides
 * what shape the cells come out. Passed into the search; most patterns
 * use plain straight-line distance, the Tier-3 ones swap it out.
 *
 *   EUCLIDEAN    straight-line distance — round cells (the default)
 *   MANHATTAN    only-horizontal-and-vertical steps — diamond cells
 *   CHEBYSHEV    the bigger of the two gaps — square cells
 *   SUPERELLIPSE between round and square — rounded squares
 *   STAR         a quirky measure that pinches inward — pointy stars
 *   STRETCHED    Manhattan but horizontal counts double — tall cells
 */
typedef enum {
    METRIC_EUCLIDEAN    = 0,
    METRIC_MANHATTAN    = 1,
    METRIC_CHEBYSHEV    = 2,
    METRIC_SUPERELLIPSE = 3,
    METRIC_STAR         = 4,
    METRIC_STRETCHED    = 5,
} Metric;

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* MAX_FRAME_DT_NS — if a frame takes longer than 100 ms (a debugger
 * pause, the laptop sleeping), pretend only 100 ms passed. Otherwise the
 * sim would try to "catch up" all at once and lock up.
 * RENDER_FPS_TARGET — redraw the screen about 60 times a second even if
 * the sim is set to tick faster. */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — one named colour scheme. Ten of these let you re-skin the whole
 * picture (t/T to cycle) without touching any of the drawing code.
 *
 *   name   : short label shown in the HUD (kept to 7 chars so it fits).
 *   band[] : four colours, dim to bright, one per brightness tier the
 *            renderer uses. These are xterm 256-colour numbers; all of
 *            them sit in the bright half of the palette on purpose, so
 *            even the "dimmest" one is still visible on a black terminal.
 *   flash  : an accent colour. Unused here, but kept so this table stays
 *            interchangeable with the sibling demos that do use it.
 */
typedef struct {
    const char *name;        /* HUD label, <=7 chars                        */
    short       band[4];     /* one colour per brightness tier, dim to bright */
    short       flash;       /* accent (reserved, unused here)              */
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
/* §5  noise — hash, feature points, F1 / F2 / F3 query under N metrics   */
/* ===================================================================== */

/*
 * WorleyNoise — the entire state of the noise: a single seed. We never
 * store the dots themselves. Each dot's position is computed fresh from
 * its tile coordinates and this seed whenever we need it, so the noise
 * covers infinite space for free. Reset (r key) rolls a new seed for a
 * fresh layout.
 *
 *   seed : changes the whole dot pattern; same seed always gives the
 *          same dots.
 */
typedef struct {
    uint32_t seed;
} WorleyNoise;

/* Scrambles an integer into a well-mixed 32-bit number. Not for security,
 * just to turn tile coordinates into believable randomness. */
static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/* The random fingerprint of one tile. Same tile always gives the same
 * number; neighbouring tiles get unrelated ones. */
static inline uint32_t tile_hash(const WorleyNoise *wn, int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + wn->seed;
    return hash32(h);
}

/* Pull one byte (0..3) or one 16-bit half (0 or 1) out of a hash, so the
 * code can say "byte 1 of the hash" instead of bit-shuffling inline. One
 * hash gives several independent random values this way. */
static inline uint32_t hash_byte(uint32_t h, int byte_idx)
{
    return (h >> (byte_idx * 8)) & 0xFFu;
}

static inline uint32_t hash_halfword(uint32_t h, int half_idx)
{
    return (h >> (half_idx * 16)) & 0xFFFFu;
}

/* Turn a tile's hash into where its dot sits inside the tile: an x and y
 * offset, each somewhere in 0..1. */
static inline void hash_to_offset(uint32_t h, float *ox, float *oy)
{
    *ox = (float)hash_halfword(h, 1) * HASH_UNIT_FROM_16BIT;
    *oy = (float)hash_halfword(h, 0) * HASH_UNIT_FROM_16BIT;
}

/* Roll a brand-new seed. Called on reset (r) so each run looks different. */
static void worley_reseed(WorleyNoise *wn)
{
    wn->seed = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/* How far apart two points are, measured the way the chosen metric says.
 * Swapping the rule here is what gives each Tier-3 pattern its cell shape;
 * everything else stays the same. (ex, ey) is the gap between the points. */
static inline float metric_distance(Metric metric, float ex, float ey)
{
    float ax = fabsf(ex);
    float ay = fabsf(ey);
    switch (metric) {
    case METRIC_MANHATTAN:    return ax + ay;
    case METRIC_CHEBYSHEV:    return ax > ay ? ax : ay;
    case METRIC_SUPERELLIPSE: return cbrtf(ax*ax*ax + ay*ay*ay);
    case METRIC_STAR: {
        float sx = sqrtf(ax);
        float sy = sqrtf(ay);
        float s  = sx + sy;
        return s * s;
    }
    case METRIC_STRETCHED:    return STRETCHED_X_WEIGHT * ax + ay;
    case METRIC_EUCLIDEAN:
    default:                  return sqrtf(ex*ex + ey*ey);
    }
}

/*
 * WorleyResult — everything one lookup tells us about a point. We always
 * find the three nearest dots in one pass, so any pattern can build its
 * look from these without re-running the search.
 *
 *   f1 : distance to the nearest dot.
 *   f2 : distance to the second-nearest (always >= f1). Where f1 and f2
 *        are nearly equal, you're on the border between two cells.
 *   f3 : distance to the third-nearest. Some patterns want it; tracking
 *        it costs almost nothing.
 *   cell_id : the fingerprint of the tile that owns the nearest dot —
 *        i.e. which cell you're standing in. Tier-4 patterns use it to
 *        give each cell its own colour, flicker, etc.
 */
typedef struct {
    float    f1;        /* distance to nearest dot       */
    float    f2;        /* distance to second-nearest    */
    float    f3;        /* distance to third-nearest     */
    uint32_t cell_id;   /* which cell we're in           */
} WorleyResult;

/*
 * WorleyFeaturePoint — one dot, ready to measure against: where it ended
 * up after wobbling, plus the fingerprint of the tile it came from.
 *
 *   px, py : the dot's position (after wobble), in the same coordinate
 *            space as the point we're shading.
 *   hash   : the owning tile's fingerprint, carried along so the search
 *            can record which cell won.
 */
typedef struct {
    float    px, py;    /* dot position after wobble  */
    uint32_t hash;      /* owning tile's fingerprint  */
} WorleyFeaturePoint;

/* A fresh result with all distances set huge, so the first real dot
 * always replaces them. */
static inline WorleyResult worley_result_empty(void)
{
    WorleyResult r = {
        .f1      = WORLEY_DISTANCE_INF,
        .f2      = WORLEY_DISTANCE_INF,
        .f3      = WORLEY_DISTANCE_INF,
        .cell_id = 0,
    };
    return r;
}

/* Find where tile (cx, cy)'s dot is right now: start from its fixed spot
 * inside the tile, then nudge it with a slow wobble. Each dot wobbles on
 * its own schedule so they don't all sway together. Time t drives the
 * wobble. */
static inline WorleyFeaturePoint worley_feature_point_at(
    const WorleyNoise *wn, int cx, int cy, float t)
{
    WorleyFeaturePoint fp;
    fp.hash = tile_hash(wn, cx, cy);

    /* The dot's resting spot inside the tile. */
    float ox, oy;
    hash_to_offset(fp.hash, &ox, &oy);

    /* Each dot starts its sway at a different point in the cycle. */
    float phase_x = (float)hash_byte(fp.hash, 1) * HASH_PHASE_FROM_BYTE;
    float phase_y = (float)hash_byte(fp.hash, 3) * HASH_PHASE_FROM_BYTE;
    ox += WOBBLE_AMOUNT * sinf(t + phase_x);
    oy += WOBBLE_AMOUNT * cosf(t + phase_y);

    fp.px = (float)cx + ox;
    fp.py = (float)cy + oy;
    return fp;
}

/* Slot one more dot's distance into the top-three. If it's the new
 * closest, also note which cell that dot belongs to. */
static inline void worley_consider_distance(WorleyResult *r,
                                            float d, uint32_t source_hash)
{
    if (d < r->f1) {
        r->f3      = r->f2;
        r->f2      = r->f1;
        r->f1      = d;
        r->cell_id = source_hash;
    } else if (d < r->f2) {
        r->f3 = r->f2;
        r->f2 = d;
    } else if (d < r->f3) {
        r->f3 = d;
    }
}

/*
 * The heart of the whole file: for a point (fx, fy), find how far the
 * three nearest dots are and which cell it's in. Look at the 3x3 block of
 * tiles around the point, check each tile's dot, and keep the closest few.
 * Because dots stay near their home tile, the nearest one is always inside
 * this block. The metric argument decides what "near" means.
 */
static WorleyResult worley_query(const WorleyNoise *wn,
                                 float fx, float fy,
                                 float t, Metric metric)
{
    /* Which tile the point sits in. */
    int xi = (int)floorf(fx);
    int yi = (int)floorf(fy);

    WorleyResult r = worley_result_empty();

    /* Check the dot in this tile and its 8 neighbours. */
    for (int dy = -WORLEY_NEIGHBOUR_RADIUS; dy <= WORLEY_NEIGHBOUR_RADIUS; dy++) {
        for (int dx = -WORLEY_NEIGHBOUR_RADIUS; dx <= WORLEY_NEIGHBOUR_RADIUS; dx++) {
            WorleyFeaturePoint fp = worley_feature_point_at(wn, xi + dx, yi + dy, t);
            float d = metric_distance(metric, fp.px - fx, fp.py - fy);
            worley_consider_distance(&r, d, fp.hash);
        }
    }
    return r;
}

/* Same search, but each dot has a "pull" that shrinks its measured
 * distance, so stronger dots win more ground and cells come out uneven.
 * Used only by the WEIGHTED pattern. */
static WorleyResult worley_query_weighted(const WorleyNoise *wn,
                                          float fx, float fy, float t)
{
    int xi = (int)floorf(fx);
    int yi = (int)floorf(fy);

    WorleyResult r = worley_result_empty();

    for (int dy = -WORLEY_NEIGHBOUR_RADIUS; dy <= WORLEY_NEIGHBOUR_RADIUS; dy++) {
        for (int dx = -WORLEY_NEIGHBOUR_RADIUS; dx <= WORLEY_NEIGHBOUR_RADIUS; dx++) {
            WorleyFeaturePoint fp = worley_feature_point_at(wn, xi + dx, yi + dy, t);

            float ex    = fp.px - fx;
            float ey    = fp.py - fy;
            float d_raw = sqrtf(ex * ex + ey * ey);

            /* This dot's pull (0.5..1.5); dividing by it lets strong dots
             * reach further. */
            float weight = WEIGHTED_W_MIN
                         + (float)hash_byte(fp.hash, 2) * HASH_UNIT_FROM_BYTE
                                                        * WEIGHTED_W_RANGE;
            worley_consider_distance(&r, d_raw / weight, fp.hash);
        }
    }
    return r;
}

/* ===================================================================== */
/* §6  patterns — 30 noise mappings in 6 tiers + dispatch table           */
/* ===================================================================== */

/*
 * Every pattern below has the same shape. It gets the noise, a point
 * (fx, fy), the current time t, and writes back two things:
 *   out_glow — the cell's brightness, 0 to 1.
 *   out_band — which of the 4 colours to use.
 * Most patterns pick the colour from the brightness; the Tier-4 ones pick
 * it from the cell's id instead.
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Brightness (0..1) to one of the 4 colour tiers. */
static inline int band_from_glow(float g)
{
    return (int)(g * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK;
}

/* Give a cell its own fixed random number (0..1) from its id, so each
 * cell can have its own brightness, flicker rate, and so on without us
 * storing anything per cell. */
static inline float cell_hash_to_unit(uint32_t cell_id)
{
    uint32_t remixed = hash32(cell_id ^ HASH_PERSONALITY_SALT);
    return (float)hash_halfword(remixed, 0) * HASH_UNIT_FROM_16BIT;
}

/* ---------- Tier 1 — the raw distances -------------------------------- */

/* Brightness from distance to the nearest dot: dark at cell centres,
 * bright at the edges. The plain cellular noise. */
static void pattern_f1(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Distance to the second-nearest dot: smoother, with bigger blobs. */
static void pattern_f2(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f2 - 0.3f) / 1.6f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Distance to the third-nearest dot: smoother still, a faint structure. */
static void pattern_f3(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f3 - 0.5f) / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The gap between the two nearest dots. It shrinks to zero right on a
 * cell border, so flipping it makes the borders glow — crisp cell outlines.
 * The classic look, and the default on startup. */
static void pattern_f2_f1(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(clampf(r.f1 / 1.414f, 0.0f, 1.0f));
}

/* The nearest distance as a fraction of the second: 0 at a cell's centre,
 * 1 at its edge. Like the F1 look but with a different falloff. */
static void pattern_f1_over_f2(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / (r.f2 + 0.001f), 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 2 — more arithmetic on the distances ----------------- */

/* The two nearest distances added together: bright everywhere except the
 * cell centres, a soft cushiony field. */
static void pattern_f1_plus_f2(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f1 + r.f2 - 0.3f) / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The two nearest distances multiplied: very dark cores, bright outlines. */
static void pattern_f1_times_f2(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 * r.f2 * 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The gap between the third- and second-nearest dots: thinner, more
 * branch-like lines than the F2-F1 edges. */
static void pattern_f3_f2(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f3 - r.f2) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The F2-F1 edges, but not flipped: dark borders, bright cell interiors. */
static void pattern_f2_f1_inv(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The F1 look flipped: each cell has a bright core fading to dark edges. */
static void pattern_f1_inv(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 3 — different ways to measure distance --------------- */

/* Distance measured in only-horizontal-and-vertical steps: diamond cells. */
static void pattern_manhattan(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_MANHATTAN);
    float g = clampf(r.f1 / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Distance = the bigger of the horizontal and vertical gaps: square cells. */
static void pattern_chebyshev(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_CHEBYSHEV);
    float g = clampf(r.f1, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* A distance halfway between round and square: rounded-square cells. */
static void pattern_superellipse(const WorleyNoise *wn, float fx, float fy, float t,
                                 float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_SUPERELLIPSE);
    float g = clampf(r.f1 / 1.4f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* A quirky distance that pinches the cells inward into 4-pointed stars.
 * Not a "real" distance rule, but it looks striking. */
static void pattern_star(const WorleyNoise *wn, float fx, float fy, float t,
                         float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_STAR);
    float g = clampf(r.f1 / 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Like Manhattan but horizontal steps count double, so cells come out
 * tall and thin. The dots don't move; only the measuring rule changes. */
static void pattern_stretched(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_STRETCHED);
    float g = clampf(r.f1 / 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 4 — each cell's id drives its look ------------------- */

/* Each cell gets one solid colour chosen from its id, with a gentle glow
 * toward its centre. The classic patchwork-of-regions look. */
static void pattern_cell_id(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    *out_glow = 1.0f - clampf(r.f1 * 0.4f, 0.0f, 0.4f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* Patchwork cells, but some dots pull harder than others (see the weighted
 * search above), so the cells come out unevenly sized. */
static void pattern_weighted(const WorleyNoise *wn, float fx, float fy, float t,
                             float *out_glow, int *out_band)
{
    WorleyResult r = worley_query_weighted(wn, fx, fy, t);
    *out_glow = 1.0f - clampf(r.f1 * 0.5f, 0.0f, 0.5f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* Each cell pulses brighter and dimmer at its own speed, set by its id. */
static void pattern_twinkle(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float rate    = 0.5f + cell_hash_to_unit(r.cell_id) * 3.0f;
    float pulse   = 0.5f + 0.5f * sinf(t * rate);
    float g_base  = 1.0f - clampf(r.f1 * 0.4f, 0.0f, 0.4f);
    *out_glow = clampf(g_base * pulse, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* Each cell gets one fixed random brightness from its id: a flat mosaic
 * where every cell is its own shade. */
static void pattern_random_glow(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    *out_glow = cell_hash_to_unit(r.cell_id);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* Crisp cell borders, but each cell's interior is a different shade.
 * Looks like cracked dried mud or cooling lava. */
static void pattern_crackle(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r       = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float        edges   = 1.0f - clampf((r.f2 - r.f1) * 3.0f, 0.0f, 1.0f);
    float        cell_lo = 0.3f + 0.7f * cell_hash_to_unit(r.cell_id);
    *out_glow = clampf(edges * cell_lo, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* ---------- Tier 5 — stack the noise at several zoom levels ----------- *
 *
 * Add up three copies of the noise: big cells at full strength, plus
 * half-size cells at half strength, plus quarter-size at a quarter. The
 * 1.75 below is the total of those strengths, used to keep the result in
 * roughly 0..1.
 */
#define WORLEY_FBM_OCTAVES   3
#define WORLEY_FBM_AMP_SUM   1.75f   /* 1 + 0.5 + 0.25 */

/* Three zoom levels of the basic F1 noise added together: clusters of
 * cells in mixed sizes. */
static void pattern_worley_fbm(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        total += amp * r.f1 / 1.414f;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The edge look stacked at three zoom levels: fine edges over coarse ones. */
static void pattern_worley_turbulence(const WorleyNoise *wn, float fx, float fy, float t,
                                      float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        float edges    = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
        total += amp * edges;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* The bright-centres look stacked at three zoom levels: looks like the
 * tops of clouds seen from above. */
static void pattern_worley_ridged(const WorleyNoise *wn, float fx, float fy, float t,
                                  float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        float ridged   = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
        total += amp * ridged;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Big cells with smaller cells laid over them: a two-level pattern,
 * cheaper than the full stacks above. */
static void pattern_nested(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult rc = worley_query(wn, fx,        fy,        t, METRIC_EUCLIDEAN);
    WorleyResult rf = worley_query(wn, fx * 2.0f, fy * 2.0f, t, METRIC_EUCLIDEAN);
    float g = clampf((rc.f1 + rf.f1 * 0.5f) / 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Like a checkerboard: tiles alternate between fine and coarse noise,
 * making a patchwork of two cell sizes. */
static void pattern_checker(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    int parity = ((int)floorf(fx) + (int)floorf(fy)) & 1;
    float scale = parity ? 2.0f : 0.5f;
    WorleyResult r = worley_query(wn, fx * scale, fy * scale, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 6 — feed one noise field into another --------------- */

/* Before shading a point, shove it sideways by an amount read from a
 * second, coarser noise field. The cells come out smudged and flowing. */
static void pattern_domain_warp(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult warp_x = worley_query(wn, fx * 0.5f,         fy * 0.5f,         t, METRIC_EUCLIDEAN);
    WorleyResult warp_y = worley_query(wn, fx * 0.5f + 5.2f,  fy * 0.5f + 1.3f,  t, METRIC_EUCLIDEAN);
    float wx = (warp_x.f1 - 0.5f) * 1.5f;
    float wy = (warp_y.f1 - 0.5f) * 1.5f;
    WorleyResult r = worley_query(wn, fx + wx, fy + wy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* Average the round-cell and diamond-cell looks: a halfway shape you
 * can't get from either measuring rule on its own. */
static void pattern_metric_blend(const WorleyNoise *wn, float fx, float fy, float t,
                                 float *out_glow, int *out_band)
{
    WorleyResult re = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    WorleyResult rm = worley_query(wn, fx, fy, t, METRIC_MANHATTAN);
    float g = clampf(0.5f * re.f1 / 1.414f + 0.5f * rm.f1 / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* A bright glowing dot at each cell's centre, fading out, each one a
 * different peak brightness. Looks like fireflies hovering in the cells. */
static void pattern_halo(const WorleyNoise *wn, float fx, float fy, float t,
                         float *out_glow, int *out_band)
{
    WorleyResult r        = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float        peak     = 0.5f + 0.5f * cell_hash_to_unit(r.cell_id);
    float        falloff  = expf(-r.f1 * 4.0f);          /* drops off fast away from the dot */
    *out_glow = clampf(peak * falloff, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* The bright-centres look pushed to an extreme: tiny razor-sharp sparks
 * at the cell centres, everything else dark. */
static void pattern_energy(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float v = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    /* Square it three times over to raise it to the 8th power — that's
     * what turns the gentle glow into a sharp pinpoint. */
    float g = v * v;
    g     *= g;
    g     *= g;
    *out_glow = clampf(g * 1.5f, 0.0f, 1.0f);
    *out_band = band_from_glow(*out_glow);
}

/* The works: shove the point around using a coarse field, then read the
 * fine edges. Looks like turbulent, swirling fluid. */
static void pattern_chaos(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult warp = worley_query(wn, fx * 0.5f, fy * 0.5f, t, METRIC_EUCLIDEAN);
    float        wx   = (warp.f1 - 0.5f) * 2.0f;
    float        wy   = (warp.cell_id & 1u) ? wx : -wx;    /* push x and y differently for a lopsided swirl */
    WorleyResult r    = worley_query(wn, fx + wx, fy + wy, t, METRIC_EUCLIDEAN);
    float        edges = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = clampf(edges, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* ---------- Dispatch table -------------------------------------------- */

/* The shared shape of all 30 pattern functions. */
typedef void (*NoisePatternFn)(const WorleyNoise *wn,
                               float fx, float fy, float t,
                               float *out_glow, int *out_band);

/*
 * NoisePattern — one row of the lookup table: a pattern's name, its group
 * label, and the function that draws it. The table replaces a giant
 * switch; pick a pattern by enum and call its function. The names are
 * padded to a fixed width so the HUD doesn't jiggle as you cycle through.
 *
 *   name   : display name shown in the HUD (padded to 10 chars).
 *   tier   : group label, like "1-FN" or "6-HYB" (padded to 7 chars).
 *   sample : the function that shades one point for this pattern.
 */
typedef struct {
    const char     *name;      /* HUD name, padded to 10 chars */
    const char     *tier;      /* HUD group label, padded to 7 */
    NoisePatternFn  sample;    /* the pattern's draw function   */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — F-FUNCTIONS */
    [PATTERN_F1]                = { "F1        ", "1-FN   ", pattern_f1                },
    [PATTERN_F2]                = { "F2        ", "1-FN   ", pattern_f2                },
    [PATTERN_F3]                = { "F3        ", "1-FN   ", pattern_f3                },
    [PATTERN_F2_F1]             = { "F2-F1     ", "1-FN   ", pattern_f2_f1             },
    [PATTERN_F1_OVER_F2]        = { "F1/F2     ", "1-FN   ", pattern_f1_over_f2        },
    /* Tier 2 — F-COMBOS */
    [PATTERN_F1_PLUS_F2]        = { "F1+F2     ", "2-COMBO", pattern_f1_plus_f2        },
    [PATTERN_F1_TIMES_F2]       = { "F1*F2     ", "2-COMBO", pattern_f1_times_f2       },
    [PATTERN_F3_F2]             = { "F3-F2     ", "2-COMBO", pattern_f3_f2             },
    [PATTERN_F2_F1_INV]         = { "F2-F1 INV ", "2-COMBO", pattern_f2_f1_inv         },
    [PATTERN_F1_INV]            = { "F1 INVERT ", "2-COMBO", pattern_f1_inv            },
    /* Tier 3 — METRICS */
    [PATTERN_MANHATTAN]         = { "MANHATTAN ", "3-METR ", pattern_manhattan         },
    [PATTERN_CHEBYSHEV]         = { "CHEBYSHEV ", "3-METR ", pattern_chebyshev         },
    [PATTERN_SUPERELLIPSE]      = { "SUPRELLIPS", "3-METR ", pattern_superellipse      },
    [PATTERN_STAR]              = { "STAR      ", "3-METR ", pattern_star              },
    [PATTERN_STRETCHED]         = { "STRETCHED ", "3-METR ", pattern_stretched         },
    /* Tier 4 — IDENTITY */
    [PATTERN_CELL_ID]           = { "CELL_ID   ", "4-IDENT", pattern_cell_id           },
    [PATTERN_WEIGHTED]          = { "WEIGHTED  ", "4-IDENT", pattern_weighted          },
    [PATTERN_TWINKLE]           = { "TWINKLE   ", "4-IDENT", pattern_twinkle           },
    [PATTERN_RANDOM_GLOW]       = { "RND_GLOW  ", "4-IDENT", pattern_random_glow       },
    [PATTERN_CRACKLE]           = { "CRACKLE   ", "4-IDENT", pattern_crackle           },
    /* Tier 5 — MULTI-SCALE */
    [PATTERN_WORLEY_FBM]        = { "WORLEY_FBM", "5-MULTI", pattern_worley_fbm        },
    [PATTERN_WORLEY_TURBULENCE] = { "TURBULENCE", "5-MULTI", pattern_worley_turbulence },
    [PATTERN_WORLEY_RIDGED]     = { "RIDGED    ", "5-MULTI", pattern_worley_ridged     },
    [PATTERN_NESTED]            = { "NESTED    ", "5-MULTI", pattern_nested            },
    [PATTERN_CHECKER]           = { "CHECKER   ", "5-MULTI", pattern_checker           },
    /* Tier 6 — HYBRIDS */
    [PATTERN_DOMAIN_WARP]       = { "DOM_WARP  ", "6-HYB  ", pattern_domain_warp       },
    [PATTERN_METRIC_BLEND]      = { "MTRC_BLEND", "6-HYB  ", pattern_metric_blend      },
    [PATTERN_HALO]              = { "HALO      ", "6-HYB  ", pattern_halo              },
    [PATTERN_ENERGY]            = { "ENERGY    ", "6-HYB  ", pattern_energy            },
    [PATTERN_CHAOS]             = { "CHAOS     ", "6-HYB  ", pattern_chaos             },
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
/* §7  scene — GlowField + GlyphRamp + PatternState + PaletteState + Scene */
/* ===================================================================== */

/*
 * GlowField — the finished picture as plain numbers, one entry per screen
 * cell. The sim fills it in once per tick; the renderer reads it to draw.
 * Keeping them separate means drawing can happen between sim ticks. The
 * buffers are sized for the biggest possible grid up front, so the busy
 * loop never has to allocate memory.
 *
 *   w, h   : grid size in cells, set at startup and on every resize.
 *   count  : w * h, kept around so loops don't recompute it.
 *   glow[] : each cell's brightness, 0 to 1.
 *   band[] : each cell's colour, 0 to 3.
 * Find a cell with glow_field_idx(gf, x, y).
 */
typedef struct {
    int      w, h;                   /* grid size in cells          */
    int      count;                  /* w * h                       */
    float    glow[CELLS_MAX];        /* per-cell brightness 0..1    */
    uint8_t  band[CELLS_MAX];        /* per-cell colour 0..3        */
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
 * GlyphRamp — the rule for turning a brightness into a character. Brighter
 * cells get a denser character; the dimmest are left blank. Keeping it as
 * data (rather than if/else in the drawer) makes the cutoffs easy to see
 * and would let a future theme swap in different characters.
 *
 *   thresh_high/mid/low : the brightness cutoffs, must go high > mid > low.
 *   glyph_high/mid/low  : the character drawn at each level.
 * The three characters '#', '*', '.' are a coarse slice of Paul Bourke's
 * dark-to-light character ramp.
 */
typedef struct {
    float thresh_high;   /* above this -> densest char  (default 0.65) */
    float thresh_mid;    /* above this -> medium char   (default 0.30) */
    float thresh_low;    /* above this -> faintest char (default 0.05) */
    char  glyph_high;    /* densest char  */
    char  glyph_mid;     /* medium char   */
    char  glyph_low;     /* faintest char */
} GlyphRamp;

/*
 * GlyphChoice — what glyph_ramp_pick() decided for one cell: which
 * character, whether to bold it, and whether to draw it at all. The colour
 * is decided separately, so this doesn't carry one.
 *
 *   glyph   : the character to draw (only meaningful if visible).
 *   attr    : bold or normal.
 *   visible : false means leave the cell blank.
 */
typedef struct {
    char glyph;         /* char to draw (only if visible)   */
    int  attr;          /* A_BOLD or A_NORMAL               */
    bool visible;       /* false -> leave the cell blank    */
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
 * PatternState — which pattern is showing and how it's animating.
 *
 *   current    : the active pattern. Starts on the crisp-edge one, which
 *                makes the best first impression. Cycled with n/p.
 *   field_time : a clock that keeps climbing; it's what makes the dots
 *                wobble. Only resets on r.
 *   drift_mult : how fast that clock runs (1, 2, 4 ... up to 16), set by +/-.
 */
typedef struct {
    Pattern current;          /* active pattern               */
    float   field_time;       /* drift clock                  */
    int     drift_mult;       /* drift speed, 1..16           */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current    = PATTERN_F2_F1;   /* crisp-edge pattern by default */
    ps->field_time = 0.0f;
    ps->drift_mult = DRIFT_MULT_DEF;
}

/*
 * PaletteState — which colour theme is showing. Just an index for now,
 * but kept as its own struct so the "current colours" idea has a home if
 * theming grows later.
 *
 *   current : index into themes[]; the key handler wraps it around.
 */
typedef struct {
    int current;              /* index into themes[]  */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — holds everything that changes while the program runs. The fields
 * are listed in the order data flows: the noise seed feeds the active
 * pattern, which fills the field, which the ramp and theme turn into
 * pictures on screen.
 *
 *   noise   : the seed the dots come from.
 *   field   : the finished brightness/colour grid.
 *   ramp    : brightness-to-character rule.
 *   pattern : which pattern and how it's animating.
 *   palette : which colour theme.
 *   paused  : when true, the sim stops updating.
 */
typedef struct {
    WorleyNoise   noise;     /* the dot field's seed              */
    GlowField     field;     /* finished grid (biggest member)    */
    GlyphRamp     ramp;      /* brightness -> character rule      */
    PatternState  pattern;   /* active pattern + animation        */
    PaletteState  palette;   /* active colour theme               */
    bool          paused;    /* true -> sim frozen                */
} Scene;

/* Turn a cell's grid position into the coordinate the patterns work in. */
static inline float cell_to_noise_coord(int cell)
{
    return (float)cell * NOISE_SCALE;
}

/* Fill the whole grid for one sim tick: run the active pattern at every
 * cell and store the brightness and colour it returns. This is the busy
 * part of the program — it runs the lookup for thousands of cells, many
 * times a second. */
static void scene_evaluate_field(Scene *s)
{
    /* Look up the active pattern's draw function once, up front. */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    NoisePatternFn      sample_pattern = noise_patterns[active].sample;
    const WorleyNoise  *noise          = &s->noise;
    GlowField          *field          = &s->field;
    float               drift          = s->pattern.field_time;

    for (int y = 0; y < field->h; y++) {
        float fy = cell_to_noise_coord(y);
        for (int x = 0; x < field->w; x++) {
            float fx   = cell_to_noise_coord(x);
            float glow = 0.0f;
            int   band = 0;
            sample_pattern(noise, fx, fy, drift, &glow, &band);

            int idx          = glow_field_idx(field, x, y);
            field->glow[idx] = glow;
            field->band[idx] = (uint8_t)(band & PALETTE_BAND_MASK);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->pattern.field_time = 0.0f;
    worley_reseed(&s->noise);
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
    /* Advance the wobble clock, faster or slower per the drift setting. */
    s->pattern.field_time += FIELD_DRIFT * (float)s->pattern.drift_mult * dt;
    scene_evaluate_field(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — just the terminal's current width and height, remembered so we
 * don't ask ncurses for them on every cell. Set up at startup, refreshed
 * whenever the window resizes, and torn down at exit (which restores the
 * terminal).
 *
 *   cols : terminal width in cells.
 *   rows : terminal height in cells.
 */
typedef struct {
    int cols;     /* terminal width in cells   */
    int rows;     /* terminal height in cells  */
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

/* ---------- scene_draw: grid → glyphs ---------------------------------- */

/*
 * GridPlacement — where on screen the top-left of the grid goes, worked
 * out once so the drawing loop doesn't redo the centering math per cell.
 * If the grid is bigger than the window, it's clamped to the edge and
 * cropped (never wrapped). It never overlaps the title rows.
 *
 *   origin_x : screen column of the grid's left edge.
 *   origin_y : screen row of the grid's top edge.
 */
typedef struct {
    int origin_x;   /* grid's left edge, in screen columns */
    int origin_y;   /* grid's top edge, in screen rows     */
} GridPlacement;

/* Centre the grid in the window, leaving the HUD rows clear. */
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

/* Draw one character at one spot in the chosen colour. */
static inline void draw_glyph_at(int screen_y, int screen_x,
                                 char glyph, int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(screen_y, screen_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* Draw the finished grid: for each cell, pick a character from its
 * brightness and a colour from its tier, and place it. Cells off the edge
 * or too dim to show are skipped. Only reads state, never changes it. */
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

/* How wide each piece of the status row is, so the drawers below can sit
 * side by side. */
#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " WORLEY CELLULAR NOISE "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* Each drawer below paints one piece of a HUD row and returns where the
 * next piece should start, so they can be chained left to right. */

/* Top-left: the program title. */
static int hud_draw_title_chip(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, "%s", HUD_TITLE_TEXT);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + (int)strlen(HUD_TITLE_TEXT);
}

/* Top-right: frame rate, tick rate, current pattern, and drift speed. */
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

/* Status row: the pattern's name. */
static int hud_draw_pattern_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-10s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_PATTERN_FIELD;
}

/* Status row: which group the pattern is in. */
static int hud_draw_tier_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " tier:%-7s ", pattern_tier(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_TIER_FIELD;
}

/* Status row: the colour theme's name. */
static int hud_draw_theme_field(int row, int x, int theme_idx)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_THEME_FIELD;
}

/* Status row: little coloured blocks showing the theme's four colours. */
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

/* Status row tail: zoom, wobble amount, and grid size. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  wobble:%.2f  map:%dx%d ",
             NOISE_SCALE, WOBBLE_AMOUNT, gf->w, gf->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the list of keys you can press. */
static void hud_draw_action_hint(int row)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(row, 0, "%s", HUD_BOTTOM_HINT_TEXT);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ---------- screen_draw: scene + full HUD ----------------------------- */

/* Draw one full frame: clear, paint the noise, then lay the HUD on top. */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    /* Top row: title on the left, live stats on the right. */
    hud_draw_title_chip(HUD_TITLE_ROW, HUD_LEFT_MARGIN);
    hud_draw_state_bar (HUD_TITLE_ROW, sc->cols, fps, sim_fps,
                        &s->pattern, s->paused);

    /* Status row: each piece picks up where the last left off. */
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
 * App — the one struct that owns everything. main() only ever touches
 * this; other functions get it or a piece of it. There's a single global
 * copy (g_app) because signal handlers have to reach the two flags below,
 * and signal handlers can't be handed a pointer.
 *
 *   scene       : the simulation.
 *   screen      : terminal size, refreshed on resize.
 *   sim_fps     : how many times a second the sim updates (]/[ to change).
 *   map_w,
 *   map_h       : the grid size, picked from the terminal size.
 *   running     : the main loop runs while this is true; q, ESC, or a
 *                 kill signal clears it.
 *   need_resize : a resize signal sets this; the main loop notices and
 *                 re-fits the grid.
 * The two flags are volatile sig_atomic_t: that's the only kind of
 * variable a signal handler is allowed to set safely, and volatile stops
 * the loop from caching a stale value.
 */
typedef struct {
    Scene                 scene;        /* the simulation                  */
    Screen                screen;       /* terminal size                   */
    int                   sim_fps;      /* sim updates per second          */
    int                   map_w;        /* grid width  (cells)             */
    int                   map_h;        /* grid height (cells)             */
    volatile sig_atomic_t running;      /* main loop runs while true       */
    volatile sig_atomic_t need_resize;  /* set by a resize signal          */
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

/* Each helper below is one thing a keypress does, so the key handler reads
 * as a tidy list. The +1/-1 direction means next/previous. */

/* Move to the next or previous pattern, wrapping around the ends. */
static void scene_cycle_pattern(Scene *s, int direction)
{
    int next = ((int)s->pattern.current + direction + N_PATTERNS) % N_PATTERNS;
    s->pattern.current = (Pattern)next;
}

/* Move to the next or previous theme, then load its colours. */
static void scene_cycle_theme(Scene *s, int direction)
{
    s->palette.current = (s->palette.current + direction + N_THEMES)
                         % N_THEMES;
    theme_apply(s->palette.current);
}

/* Drift speed doubles or halves so the change feels like clear steps. */
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

/* Speed the sim up or slow it down, kept within sane limits. */
static void app_adjust_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Do what one keypress asks. Returns false only when the user wants to quit. */
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

/*
 * FrameClock — the timekeeping the main loop needs, in one place. It does
 * two jobs that both start from "how long did this frame take":
 *   - It keeps the sim ticking at a steady rate no matter how choppy the
 *     drawing is: bank up elapsed time and spend it one fixed tick at a
 *     time (the classic "fix your timestep" approach).
 *   - It works out the frame rate to show in the HUD.
 *
 *   frame_time_ns   : when the current frame started.
 *   sim_accum_ns    : banked-up time still waiting to be turned into ticks.
 *   fps_accum_ns,
 *   fps_frame_count : time and frames counted toward the next fps figure.
 *   fps_display     : the latest frame rate, which the HUD reads.
 */
typedef struct {
    int64_t frame_time_ns;     /* when this frame started              */
    int64_t sim_accum_ns;      /* time banked for sim ticks            */
    int64_t fps_accum_ns;      /* time counted toward the fps figure   */
    int     fps_frame_count;   /* frames counted toward the fps figure */
    double  fps_display;       /* latest fps, shown in the HUD         */
} FrameClock;

static void frame_clock_init(FrameClock *fc)
{
    fc->frame_time_ns   = clock_ns();
    fc->sim_accum_ns    = 0;
    fc->fps_accum_ns    = 0;
    fc->fps_frame_count = 0;
    fc->fps_display     = 0.0;
}

/* After a pause (like a window resize), reset the clock so the sim doesn't
 * try to fast-forward through the time the user wasn't watching. */
static void frame_clock_resync(FrameClock *fc)
{
    fc->frame_time_ns = clock_ns();
    fc->sim_accum_ns  = 0;
}

/* Tick the clock over to now and report how long the last frame took,
 * capped so a long stall can't make the sim try to catch up all at once. */
static int64_t frame_clock_advance(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->frame_time_ns;
    fc->frame_time_ns = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* Count this frame; every half-second or so, recompute the frame rate. */
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

/* Bank this frame's time, then spend it on sim ticks in fixed chunks. Each
 * tick advances the sim by the same amount, so it behaves the same whether
 * the screen is drawing smoothly or stuttering. */
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

/* Sleep off whatever's left of this frame's time budget so the screen
 * refreshes around 60 times a second instead of as fast as possible. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Grab a keypress if there is one and act on it; quit clears running. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Hook up the signals: Ctrl-C / kill end the program, a resize re-fits
 * the grid. The handlers only flip a flag, which is all they're allowed
 * to do safely. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/*
 * main — set things up, then loop: handle any resize, advance the clock,
 * run the sim, pace the frame rate, draw, and read input, until the user
 * quits. Then put the terminal back the way it was.
 */
int main(void)
{
    /* Seed randomness, make sure the terminal gets restored on exit. */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    /* Start the terminal and the simulation. */
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
