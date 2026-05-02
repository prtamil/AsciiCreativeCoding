/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * worley_cellular_noise.c
 *   — Steven Worley's cellular / Voronoi-style noise, 5 patterns.
 *
 * DEMO: A scattered field of "feature points" — one per integer tile
 *       of noise space — drives five visually distinct patterns. For
 *       any cell on screen the algorithm finds the nearest feature
 *       points among the 3×3 surrounding tiles, then the active
 *       pattern decides what to render with those distances:
 *         F1        — distance to nearest point: cellular blobs,
 *                     dark feature-point centres, bright boundaries
 *         F2_F1     — second-min minus first-min: thin bright LINES
 *                     along every cell boundary (Voronoi edges)
 *         F2        — distance to second-nearest: smoother, larger blobs
 *         MANHATTAN — F1 but with |Δx|+|Δy| metric — sharp 45°-ish
 *                     diamond cells instead of round blobs
 *         CELL_ID   — colour each cell by the hash of its nearest
 *                     feature point — solid Voronoi-style regions
 *       Feature points wobble slightly with time, so all patterns
 *       evolve organically rather than freezing.
 *
 * Study alongside:
 *   ./domain_warped_noise_iq_style.c — Perlin-based field showcases.
 *       Worley vs Perlin: same "evaluate a noise field at every cell"
 *       skeleton, but Worley is DISCRETE (distance to feature points)
 *       where Perlin is CONTINUOUS (smooth gradient noise). Perlin
 *       gives rolling hills; Worley gives cells with hard or soft
 *       edges depending on the chosen distance function.
 *   ../generational/voronoi_region_map.c — Worley's CELL_ID pattern
 *       is essentially a streaming Voronoi diagram. The voronoi file
 *       precomputes everything; this one evaluates on-demand and
 *       lets every cell's owner change as feature points wobble.
 *
 * Section map:
 *   §1 config   — grid, patterns, scale, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 palette colours each)
 *   §5 noise    — hash, feature-point sampling, F1/F2 query
 *   §6 patterns — per-pattern noise → glow + colour
 *   §7 scene    — Field, scene state, per-frame grid update
 *   §8 screen   — ASCII render: density-graded glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new feature-point hash seed)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster wobble drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra worley_cellular_noise.c \
 *       -o worley -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Worley noise (Worley 1996, "A cellular texture basis
 *                  function"). Divide the plane into integer-aligned
 *                  unit tiles. Each tile contains exactly one "feature
 *                  point" whose sub-tile position is determined by a
 *                  deterministic hash of the tile coordinates plus a
 *                  per-run seed. To evaluate the noise at a query
 *                  point (x, y):
 *                    1. Find the integer tile (xi, yi) containing it.
 *                    2. For each of the 3×3 neighbouring tiles
 *                       (xi+dx, yi+dy), dx, dy ∈ {-1, 0, 1}:
 *                       a. Hash the tile coords to get the feature
 *                          point's position in [0, 1)² inside the tile.
 *                       b. Compute the Euclidean (or Manhattan, or
 *                          Chebyshev) distance from (x, y) to that
 *                          point.
 *                    3. Track F1 = nearest, F2 = second-nearest.
 *                  The 3×3 lookup is sufficient: any feature point in
 *                  a tile further than 1 cell away is necessarily
 *                  beyond distance √2, larger than any in-bounds F1
 *                  candidate. The function is fast (constant work per
 *                  query) and produces patterns whose CELL STRUCTURE
 *                  matches a Voronoi diagram of the feature points.
 *
 *                  Five patterns are derived from F1, F2, and the cell
 *                  identity. They share the same query loop; only the
 *                  metric and the post-query mapping differ.
 *
 * Data-structure : One uint32_t hash seed (re-rolled at reset) and a
 *                  hash function — no pre-allocated point list. Every
 *                  feature point is implicit: (cx, cy) → hash → (ox, oy).
 *                  That makes Worley noise infinite-extent and
 *                  zero-memory; we don't store any of it.
 *
 *                  Per-cell glow + colour buffers as in the other
 *                  field showcases. Field overwritten each frame.
 *
 * Rendering      : ASCII only. Density-graded glyphs ('.', '*', '#')
 *                  in 4 theme palette colours. Pattern picks both glow
 *                  and colour band from F1/F2 and the cell identity.
 *                  Animation: each feature point wobbles with a phase
 *                  derived from its hash + a time accumulator, so cell
 *                  boundaries breathe and shift slowly.
 *
 * Performance    : 1 query per cell per frame. Each query examines 9
 *                  tiles → 9 hashes + 9 distance computations per
 *                  query. On a 200×56 grid at 60 Hz that's ~6 M
 *                  hashes/sec — completely trivial. The hash itself
 *                  is a 4-mul integer mixer; no float math beyond the
 *                  distance calculation.
 *
 * References     : • Worley, S. (1996) — "A cellular texture basis
 *                    function", SIGGRAPH:
 *                    https://dl.acm.org/doi/10.1145/237170.237267
 *                  • Inigo Quilez — "Voronoise" (variant on Worley):
 *                    https://iquilezles.org/articles/voronoise/
 *                  • The Book of Shaders — chapter on cellular noise:
 *                    https://thebookofshaders.com/12/
 *                  • Wikipedia — "Worley noise":
 *                    https://en.wikipedia.org/wiki/Worley_noise
 *                  • Compare — ../generational/voronoi_region_map.c
 *                    (precomputed) and ./perin_noise_flow_showcase.c
 *                    (Perlin instead of Worley).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Scatter random "stars" across a grid, one per square tile. For any
 * point in space, ask "how far am I from the closest star?". That
 * distance is your noise value. Repeat for every screen cell and you
 * get a cellular pattern — dark near each star, brighter as you move
 * away from any star, brightest along the lines exactly between two
 * stars. Different distance metrics (Euclidean / Manhattan /
 * Chebyshev) produce different cell shapes (circles / diamonds /
 * squares). Different functions of the distances (F1 / F2 / F2-F1)
 * highlight different features (centres / edges / blobs).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each tile of the grid harbours one hidden firefly. The firefly's
 * exact position in the tile is determined by a HASH — the same tile
 * coordinates always give the same firefly position, but two adjacent
 * tiles see un-correlated positions. To shade any pixel, ask: "of
 * the 9 nearby fireflies (this tile + 8 neighbours), which is closest?"
 * Use that distance as the brightness. The result is a pattern where
 * every firefly is at a dark centre, bordered by bright "moats"
 * exactly halfway to its neighbours.
 *
 * Visible layers:
 *   1. The CELL STRUCTURE — visible in every pattern as the locus of
 *      brightest cells, which traces the Voronoi diagram of the
 *      hidden feature points.
 *   2. The PATTERN-SPECIFIC mapping — what we DO with F1/F2/cell-id
 *      determines whether centres or edges are bright, whether cells
 *      are circles or diamonds, whether each cell has its own colour
 *      or fades smoothly into the next.
 *   3. The TIME WOBBLE — feature points oscillate slightly within
 *      their tiles, so cell boundaries shift continuously. The Voronoi
 *      structure is visible but never frozen.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Convert cell coordinate to noise space: (fx, fy) = (x, y) ·
 *     NOISE_SCALE.
 *  2. Find integer tile: (xi, yi) = (⌊fx⌋, ⌊fy⌋).
 *  3. Initialise F1 = F2 = +∞, cell_id = 0.
 *  4. For each of the 9 tiles around (xi, yi):
 *     a. Hash tile coords + run-seed → 32-bit value h.
 *     b. Extract (ox, oy) ∈ [0, 1)² from h's bits.
 *     c. Wobble: ox += A · sin(t + phase_x); oy += A · cos(t + phase_y).
 *        Phases come from other bits of h so neighbouring fireflies
 *        wobble at different rates.
 *     d. Feature point position: (cx + ox, cy + oy).
 *     e. Distance d to (fx, fy) under the active metric.
 *     f. If d < F1: F2 = F1; F1 = d; cell_id = h.
 *        Else if d < F2: F2 = d.
 *  5. Pattern function: turn (F1, F2, cell_id) into (glow, colour band).
 *  6. Write to trail_glow[cell] and trail_color[cell].
 *
 * KEY FORMULAS
 * ────────────
 *  Hash mixer (32-bit)         : h = mix(xi · 374761393 + yi · 668265263 + seed)
 *  Feature pos in tile         : (ox, oy) = (high_16(h), low_16(h)) / 2¹⁶
 *  Wobble                      : ox' = ox + W · sin(t + phase),
 *                                oy' = oy + W · cos(t + phase)
 *  Euclidean distance          : √(Δx² + Δy²)
 *  Manhattan distance          : |Δx| + |Δy|
 *  F2 − F1 (edge highlight)    : near-zero on cell boundaries
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • 3×3 LOOKUP IS SUFFICIENT only if the maximum distance to F1
 *    inside the centre tile is ≤ √2 (≈ 1.414). The diagonal of a
 *    1×1 tile is exactly √2, so any feature point in a tile beyond
 *    the 3×3 cannot be closer than 1 in some axis → ≥ 1 distance.
 *    Don't shrink to 2×2 — you'll miss F1 candidates near tile
 *    boundaries.
 *
 *  • WOBBLE AMPLITUDE. Keep WOBBLE_AMOUNT < ~0.4 so feature points
 *    stay roughly inside their original tiles. Large wobble lets
 *    them cross into neighbouring tiles, breaking the 3×3 lookup
 *    completeness guarantee — you might miss F1 entirely.
 *
 *  • F2 − F1 NORMALISATION. Raw F2−F1 is in [0, ~1.4] but most
 *    cells have it in [0, 0.5]. We multiply by 2.0 and clamp; that
 *    gives bright lines exactly at the boundaries, fading inward
 *    over a few cells.
 *
 *  • CELL_ID COLOURING. We mod the hash by 4 for the colour band.
 *    Adjacent cells COULD pick the same colour — that's a known
 *    visual artefact called "the four-colour problem" (only 4
 *    colours are used so collisions are statistically common).
 *    Doesn't matter for showcase quality.
 *
 *  • DISTANCE METRIC. The Manhattan metric produces diamond-shaped
 *    cells; Chebyshev (max(|Δx|, |Δy|)) produces axis-aligned
 *    squares. Both are valid Worley variants. We expose Manhattan
 *    as a separate pattern and use Euclidean for the rest.
 *
 *  • HASH QUALITY. The mixer here is good enough for visual noise
 *    but is not cryptographically random. For physical simulations
 *    or scientific work use a stronger PRNG.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • F1 pattern: dark spots at feature points, brightening outward.
 *    If you see the opposite (bright at points), the glow mapping is
 *    inverted.
 *  • F2_F1: bright thin lines at every cell boundary. If you see no
 *    lines, F2 is being read as F1 (the if/else chain is wrong).
 *  • MANHATTAN: cells have STRAIGHT 45°-rotated boundaries (diamonds
 *    aligned with the grid axes). Round cells = wrong metric.
 *  • CELL_ID: cells have flat solid colour with sharp boundaries. If
 *    you see a smooth gradient inside cells, the constant glow isn't
 *    being applied.
 *  • Wobble visible: pause briefly, then unpause; cell boundaries
 *    should drift slowly. If frozen, FIELD_DRIFT is 0 or t isn't
 *    advancing.
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

    /* How often to reroll the hash seed. ~12 s lets each cellular
     * arrangement breathe before the next arrives. */
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

/* Noise scale: ~0.1 noise-units per cell → 1 tile every 10 cells →
 * about 20 × 6 = 120 cells across a 200 × 56 grid. */
#define NOISE_SCALE         0.10f

/* Field drift in noise-coord units per second (drives the wobble's
 * time component). */
#define FIELD_DRIFT         0.40f

/* Wobble amplitude. Keep < 0.4 to ensure feature points stay roughly
 * inside their original tiles (preserves the 3×3 lookup invariant). */
#define WOBBLE_AMOUNT       0.20f

/* Drift multiplier — scaled by +/-, capped low so the cells don't
 * boil too aggressively. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      16

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — five Worley variants, cycle with n/p.
 *
 *   F1        : distance to nearest feature point — cellular blobs
 *   F2_F1     : F2 − F1 — bright lines at cell boundaries
 *   F2        : distance to second-nearest — smoother / larger blobs
 *   MANHATTAN : F1 with the Manhattan metric — diamond cells
 *   CELL_ID   : flat colour per cell — Voronoi-style regions
 */
typedef enum {
    PATTERN_F1        = 0,
    PATTERN_F2_F1     = 1,
    PATTERN_F2        = 2,
    PATTERN_MANHATTAN = 3,
    PATTERN_CELL_ID   = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_F1:        return "F1     ";
    case PATTERN_F2_F1:     return "F2-F1  ";
    case PATTERN_F2:        return "F2     ";
    case PATTERN_MANHATTAN: return "MANHATTAN";
    case PATTERN_CELL_ID:   return "CELL_ID";
    default:                return "?      ";
    }
}

/* Distance metrics. Used by patterns that take a metric parameter. */
typedef enum {
    METRIC_EUCLIDEAN = 0,
    METRIC_MANHATTAN = 1,
} Metric;

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names. Each theme defines 4 band colours (used by
 * patterns F1/F2/MANHATTAN/CELL_ID for colour selection) plus a
 * flash accent. PAIR_HUD/HINT/SUPERNOVA stay theme-independent.
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
/* §5  noise — hash, feature points, F1 / F2 query                        */
/* ===================================================================== */

/*
 * Per-run hash seed. Re-rolled at every reset so each run gets a
 * different feature-point arrangement. Pure scalar — no permutation
 * table needed for cellular noise.
 */
static uint32_t worley_seed = 0;

/*
 * hash32 — small integer mixer. 4 multiplies + 4 shifts. Output is a
 * decent-quality 32-bit hash for visual noise (passes basic chi-square
 * but not crypto-grade). Good enough for picking feature-point
 * positions deterministically.
 */
static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/*
 * tile_hash — 32-bit hash of (xi, yi, worley_seed). Different (xi, yi)
 * pairs almost always give different hash values; the seed re-rolls
 * the entire space for variety across runs.
 */
static inline uint32_t tile_hash(int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + worley_seed;
    return hash32(h);
}

/*
 * Float-in-[0, 1) extraction from two halves of a 32-bit hash.
 * High 16 bits → ox, low 16 bits → oy. Independent enough that ox/oy
 * are statistically uncorrelated.
 */
static inline void hash_to_offset(uint32_t h, float *ox, float *oy)
{
    *ox = (float)((h >> 16) & 0xFFFFu) * (1.0f / 65535.0f);
    *oy = (float)(h        & 0xFFFFu) * (1.0f / 65535.0f);
}

/*
 * WorleyResult — what the per-cell query returns. F1 = nearest
 * distance, F2 = second-nearest, cell_id = the hash that owns F1.
 * cell_id is used by the CELL_ID pattern for solid colouring.
 */
typedef struct {
    float    f1, f2;
    uint32_t cell_id;
} WorleyResult;

/*
 * worley_query — for the noise-space point (fx, fy), find the F1
 * (and F2) feature point distance under the requested metric, plus
 * the hash of the F1-owning tile.
 *
 * Examines the 3×3 neighbourhood of the integer tile containing
 * (fx, fy). With WOBBLE_AMOUNT < 0.4 every feature point stays
 * roughly inside its tile, so the 3×3 lookup is provably complete.
 *
 * t is the field time used for the wobble phase.
 */
static WorleyResult worley_query(float fx, float fy, float t, Metric metric)
{
    int xi = (int)floorf(fx);
    int yi = (int)floorf(fy);

    WorleyResult r = { .f1 = 1e10f, .f2 = 1e10f, .cell_id = 0 };

    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int cx = xi + dx;
            int cy = yi + dy;

            uint32_t h = tile_hash(cx, cy);
            float ox, oy;
            hash_to_offset(h, &ox, &oy);

            /* Wobble — phases derived from other bits of the hash so
             * neighbouring fireflies wobble out of sync. */
            float phase_x = (float)((h >>  8) & 0xFFu) * (1.0f / 40.0f);
            float phase_y = (float)((h >> 24) & 0xFFu) * (1.0f / 40.0f);
            ox += WOBBLE_AMOUNT * sinf(t + phase_x);
            oy += WOBBLE_AMOUNT * cosf(t + phase_y);

            float px = (float)cx + ox;
            float py = (float)cy + oy;
            float ex = px - fx;
            float ey = py - fy;

            float d;
            if (metric == METRIC_MANHATTAN) {
                d = fabsf(ex) + fabsf(ey);
            } else {
                d = sqrtf(ex * ex + ey * ey);
            }

            if (d < r.f1) {
                r.f2 = r.f1;
                r.f1 = d;
                r.cell_id = h;
            } else if (d < r.f2) {
                r.f2 = d;
            }
        }
    }
    return r;
}

/* ===================================================================== */
/* §6  patterns — 5 derived visualisations                                */
/* ===================================================================== */

/*
 * Pattern functions return both glow ([0, 1]) and a band index ([0, 3])
 * via out-pointers. Every pattern shares the worley_query as its
 * compute kernel; only the post-processing differs.
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * pattern_f1 — distance to nearest feature point. F1 is ~0 at each
 * feature point and approaches √2 at the farthest cell boundaries.
 * Glow rises with distance, so cell CENTRES are dark and cell EDGES
 * are bright. Band index follows the glow quartile.
 */
static void pattern_f1(float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = (int)(g * 3.999f) & 3;
}

/*
 * pattern_f2_f1 — F2 minus F1. Tiny near cell boundaries (where two
 * distances are nearly equal) and larger inside cells. We invert and
 * scale so BOUNDARIES are bright. Result is a fine-line Voronoi
 * diagram — the sharpest of the five patterns.
 */
static void pattern_f2_f1(float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(fx, fy, t, METRIC_EUCLIDEAN);
    float diff = r.f2 - r.f1;
    /* Sharpen — multiplier controls line thickness; larger = thinner. */
    float g = 1.0f - clampf(diff * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = (int)((r.f1 / 1.414f) * 3.999f) & 3;
}

/*
 * pattern_f2 — distance to second-nearest feature point. Always >
 * F1; produces smoother, larger blobs because every cell is "inside"
 * the F2-region of a wider neighbourhood.
 */
static void pattern_f2(float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(fx, fy, t, METRIC_EUCLIDEAN);
    /* F2 typical range ~ [0.5, 2.0]; map to [0, 1]. */
    float g = clampf((r.f2 - 0.3f) / 1.6f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = (int)(g * 3.999f) & 3;
}

/*
 * pattern_manhattan — F1 under the Manhattan |Δx|+|Δy| metric.
 * Feature points are still at the same hashed positions, but the
 * "nearest" relationship now produces 45°-rotated diamond cells
 * instead of the round Euclidean blobs.
 */
static void pattern_manhattan(float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(fx, fy, t, METRIC_MANHATTAN);
    /* Manhattan distance can reach 2.0 in the centre tile (the
     * far corner of a unit square is at Manhattan distance 2). */
    float g = clampf(r.f1 / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = (int)(g * 3.999f) & 3;
}

/*
 * pattern_cell_id — solid-colour each cell by the F1 owner. We pick
 * a constant glow with a slight inward gradient (so each cell has a
 * subtle "focal point" feel rather than being completely flat) and
 * the band index is the hash mod 4.
 *
 * Result reads as a Voronoi diagram of the wobbling feature points.
 */
static void pattern_cell_id(float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(fx, fy, t, METRIC_EUCLIDEAN);
    /* Slight inward gradient: brighter near centre, dimmer at edges,
     * but always above the "solid block" threshold. */
    float g = 1.0f - clampf(r.f1 * 0.4f, 0.0f, 0.4f);
    *out_glow = g;
    *out_band = (int)(r.cell_id & 3u);
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
    /* Reroll the worley_seed so feature-point positions are fresh. */
    worley_seed = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
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

            float g    = 0.0f;
            int   band = 0;

            switch (p) {
            case PATTERN_F1:        pattern_f1       (fx, fy, t, &g, &band); break;
            case PATTERN_F2_F1:     pattern_f2_f1    (fx, fy, t, &g, &band); break;
            case PATTERN_F2:        pattern_f2       (fx, fy, t, &g, &band); break;
            case PATTERN_MANHATTAN: pattern_manhattan(fx, fy, t, &g, &band); break;
            case PATTERN_CELL_ID:   pattern_cell_id  (fx, fy, t, &g, &band); break;
            default:                                                          break;
            }
            int idx = field_idx(f, x, y);
            f->trail_glow[idx]  = g;
            f->trail_color[idx] = (uint8_t)(band & 3);
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
    s->current_pattern = PATTERN_F2_F1;     /* boundary-line pattern by default */
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    /* Supernova flash decays. */
    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Drive the wobble. */
    f->field_time += FIELD_DRIFT * (float)s->drift_mult * dt;

    /* Repaint the entire grid from the active pattern. */
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
                          ? "PAUSED   "
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
    mvprintw(0, 1, " WORLEY CELLULAR NOISE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + colour swatches. */
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
             "  scale:%.2f  wobble:%.2f  map:%dx%d ",
             NOISE_SCALE, WOBBLE_AMOUNT, f->w, f->h);
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
