/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diamond_square_heightmap_showcase.c
 *   — Diamond-Square fractal heightmap, animated.
 *
 * DEMO: Watch a fractal terrain grow from four random corners. The
 *       grid starts blue (uncomputed = water level) with four bright
 *       seeded corners. Each tick, a few more cells are computed via
 *       alternating "diamond" and "square" passes — the standard
 *       midpoint-displacement recurrence. Newly-set cells flash gold,
 *       then settle to a colour band by height: water → beach → grass
 *       → hills → mountain → snow. Heights are normalised at render
 *       time so all six bands are always represented. After every cell
 *       is computed, HOLD on the terrain; supernova reset; new
 *       continent forever.
 *
 * Study alongside: ./bsp_dungeon_showcase.c — another "structured
 *       partition" generator. BSP partitions space recursively for
 *       discrete rooms; Diamond-Square recursively interpolates a
 *       continuous height field. Same recursive-halving philosophy,
 *       very different output.
 *
 * Section map:
 *   §1 config   — grid size auto-detect, palette, glow rates
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 7 terrain bands + flash + supernova
 *   §5 ds       — heightmap state, schedule_phase, diamond/square step
 *   §6 scene    — COMPUTING / HOLD state machine
 *   §7 screen   — terrain band rendering, compute glow flash
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate restart with new seed)
 *   + / =      faster (more cells per tick)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra diamond_square_heightmap_showcase.c \
 *       -o diamond_square -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Diamond-Square (Fournier, Fussell & Carpenter 1982).
 *                  Generates a fractal 2D height field on a
 *                  (2^n + 1) × (2^n + 1) grid by midpoint displacement:
 *                    1. Seed the four corners with random heights.
 *                    2. DIAMOND step. For every square sub-region of the
 *                       current step size, the centre cell becomes the
 *                       AVERAGE of the four corners plus a random offset.
 *                    3. SQUARE step. For every diamond sub-region just
 *                       formed, each of its four edge midpoints becomes
 *                       the average of its (up to four) cardinal
 *                       neighbours plus a random offset.
 *                    4. Halve the step size; halve the random magnitude
 *                       (the "roughness"); repeat from 2.
 *                  Stops when step size = 1 — every cell has a value.
 *                  The result is a self-similar fractal — at any zoom
 *                  level the terrain looks statistically the same.
 *
 *                  The roughness decay (R *= persistence each level,
 *                  typically persistence = 0.5) is what makes the
 *                  output look like terrain rather than noise: large
 *                  features dominate, with progressively smaller
 *                  features layered on top.
 *
 * Data-structure : Single flat float[] heightmap, [0..1] range. A
 *                  parallel computed[] bool tracks which cells have
 *                  been set so the renderer can paint uncomputed cells
 *                  as flat water. A queue of (x, y, kind) operations
 *                  drives the animated reveal — each scene_tick pops
 *                  a few cells off the queue and computes them.
 *
 * Rendering      : Cells map to ASCII terrain glyphs by height (after
 *                  normalisation against the per-frame min/max):
 *                    h < 0.30 water       '~'  (sky blue)
 *                    h < 0.40 beach       '.'  (yellow)
 *                    h < 0.55 grass       ','  (green)
 *                    h < 0.72 hills       ';'  (forest green)
 *                    h < 0.88 mountain    '#'  (grey)
 *                    else     snow        '@'  (white)
 *                  Without normalisation, heights would drift outside
 *                  [0, 1] from accumulated jitter, clamp at the extremes,
 *                  and the middle bands would rarely render. A per-cell
 *                  compute_glow flashes gold the moment the cell is set;
 *                  decays to the resting band colour.
 *
 * Performance    : O(N²) where N = 2^n + 1. The work per phase grows
 *                  geometrically (1, 4, 4, 16, 16, …, 4ⁿ) but the inner
 *                  per-cell work is constant (3 adds + a divide + a
 *                  random). We throttle via ops_per_tick so the
 *                  spectacle plays out over ~6–10 seconds regardless
 *                  of grid size.
 *
 * References     : • Fournier, Fussell & Carpenter (1982) —
 *                    "Computer rendering of stochastic models",
 *                    CACM 25(6) — the original paper.
 *                  • Lighthouse3D — "Diamond Square Algorithm" tutorial:
 *                    http://www.lighthouse3d.com/opengl/terrain/index.php3?diamond
 *                  • Wikipedia — "Diamond-square algorithm":
 *                    https://en.wikipedia.org/wiki/Diamond-square_algorithm
 *                  • Hunter Loftis — "Playfuljs: Realistic terrain in 130
 *                    lines" (a good visual walkthrough):
 *                    https://www.playfuljs.com/realistic-terrain-in-130-lines/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a height field that looks like real terrain, don't sample
 * random heights independently — neighbouring points must be similar.
 * Diamond-Square gets that by INTERPOLATING between known points and
 * adding only a small RANDOM jitter. The jitter shrinks at every
 * recursion depth, so big features (continents, mountain ranges) get
 * shaped first, and tiny details (boulders, ripples) get layered on
 * later. The whole thing is just "average your neighbours, jitter a
 * bit, recurse smaller".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a stretched rubber sheet pinned at the four corners. Pinch
 * the centre upward with a small random offset — that's the first
 * DIAMOND step. Now pinch the four edge-midpoints (the points at the
 * middle of each side) — that's the first SQUARE step. The sheet now
 * has 9 fixed points: 4 corners, 4 edge-midpoints, 1 centre. Subdivide
 * each of the 4 quadrants and repeat (with HALF the pinch strength).
 * Eventually every grid point is pinned at a height.
 *
 * Two key observations:
 *   • DIAMOND points see four corners arranged in a SQUARE shape
 *     around them. SQUARE points see (up to) four neighbours arranged
 *     in a DIAMOND shape (cross / +). Hence the names.
 *   • Halving the random magnitude every level is what makes the
 *     output FRACTAL — without that decay you just get noise. The
 *     ratio of decay is called "persistence"; 0.5 is the textbook
 *     value, lower → smoother, higher → noisier.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Allocate a (2^n+1) × (2^n+1) grid of floats. Mark all
 *     cells uncomputed. Set the 4 CORNERS — (0,0), (N,0), (0,N), (N,N)
 *     where N = 2^n — to random values in [0, 1]; mark them computed.
 *  2. step = N, roughness = R0 (≈ 0.5).
 *  3. DIAMOND PHASE at this step:
 *     for y in [step/2, step/2 + step, step/2 + 2·step, …, N - step/2]:
 *       for x in same range:
 *         h = mean(grid[x±step/2][y±step/2]) + uniform(-R, +R)
 *         set grid[x][y] = h
 *  4. SQUARE PHASE at this step:
 *     for every "edge midpoint" (cells where exactly one of x, y is a
 *     multiple of step and the other is offset by step/2):
 *       h = mean of (up to four) cardinal neighbours at distance step/2
 *           + uniform(-R, +R)
 *       set grid[x][y] = h
 *  5. step ← step / 2; R ← R · 0.5; if step ≥ 1 goto 3.
 *  6. Done. Every cell has a height; clamp to [0, 1] for rendering.
 *
 * KEY FORMULAS
 * ────────────
 *  N (grid side − 1)             : N = 2^n
 *  Number of levels              : log₂(N)
 *  Diamond cell coords           : (i + ½)·step, (j + ½)·step  ∈ {1..⌊N/step⌋}²
 *  Square cell coords            : where (x ÷ half + y ÷ half) is odd,
 *                                  with both x, y multiples of half.
 *                                  Equivalently: every other lattice
 *                                  point on the half-step grid that is
 *                                  NOT a diamond point.
 *  Diamond average               : (NW + NE + SW + SE) / 4
 *  Square average                : (N + E + S + W) / count_in_grid
 *                                  (omit out-of-grid neighbours)
 *  Random offset                 : uniform(-R, +R)
 *  Persistence (roughness decay) : R_{level+1} = R_level · 0.5
 *  Total ops                     : N² − 4 (every cell except corners)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • OUT-OF-GRID NEIGHBOURS in the SQUARE step. Edge cells along the
 *    map borders only have 3 valid cardinal neighbours (corners only
 *    have 2). The average must use only the in-bounds neighbours,
 *    NOT pretend missing ones are 0 — using 0 makes the entire border
 *    drop into "deep water" regardless of corner seeds. Always count
 *    contributors and divide by the actual count.
 *
 *  • DIAMOND BEFORE SQUARE. The order matters: every level's SQUARE
 *    phase reads the DIAMOND points it just created. Running square
 *    before diamond at the same level produces noise (the square
 *    averages have nothing to average).
 *
 *  • SEAM AT THE WRAP. If you accidentally treat the grid as periodic
 *    you'll see a sharp seam at the boundary because the corners are
 *    independent random samples. The standard algorithm is NON-periodic.
 *    For a wrap-around terrain (e.g. a sphere), seed corners equal
 *    pairwise — but that's a different algorithm.
 *
 *  • CLAMPING THE OUTPUT. Heights drift outside [0, 1] because of the
 *    accumulated random offsets. For rendering we clamp on read, but
 *    the underlying float can go negative or > 1. Don't clamp during
 *    the algorithm — clipping mid-computation flattens features.
 *
 *  • POWER-OF-TWO PLUS ONE. The grid MUST be (2^n + 1) wide. 33, 65,
 *    129, 257 are the practical sizes; non-power-of-2 widths break
 *    the recursion (step halves and never lands on integer cells).
 *
 *  • PHASE TRANSITION. The animation queue must be fully drained
 *    BEFORE moving to the next phase. If you advance to the next
 *    halving while DIAMOND cells from this level haven't been
 *    computed yet, the SQUARE step reads zeros and produces flat
 *    triangles instead of fractal noise.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At init, exactly 4 cells are computed (the corners). If 0 or 5+,
 *    the seed loop is wrong.
 *  • After every DIAMOND phase, the count of computed cells equals
 *    the previous count + (N/step)². For step=N (level 0) that's +1.
 *  • The final terrain has features at multiple scales — if it looks
 *    like a single big triangle/cone, the random offsets are too
 *    small or the persistence is too aggressive.
 *  • Histogram: a healthy run produces a roughly bell-shaped height
 *    distribution centered near the mean of the corner seeds. A flat
 *    or U-shaped histogram means roughness is wrong.
 *  • Visual: continents, coastlines, mountain interiors should be
 *    obviously distinguishable. If everything is one colour band the
 *    height range is collapsed (corners too similar OR roughness
 *    decay too steep).
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    /* Maximum grid side − 1. We support (2^n + 1) sizes up to 129 — at
     * 129×129 the heightmap holds 16641 cells, which fits a wide
     * terminal and stays under 100KB of state. */
    GRID_N_MAX        = 128,        /* N = 2^n; grid is (N+1) × (N+1) */
    GRID_N_MIN        =   8,        /* minimum useful — gives a 9×9   */

    GRID_W_MAX        = GRID_N_MAX + 1,
    CELLS_MAX         = GRID_W_MAX * GRID_W_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =   8,        /* cells computed per scene_tick */
    OPS_PER_TICK_MAX  = 512,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WATER        =   3,        /* water — sky blue              */
    PAIR_BEACH        =   4,        /* beach — sand yellow           */
    PAIR_GRASS        =   5,        /* grass — green                 */
    PAIR_HILL         =   6,        /* hills — forest green          */
    PAIR_MOUNTAIN     =   7,        /* mountain — grey               */
    PAIR_SNOW         =   8,        /* snow — bright white           */
    PAIR_FLASH        =   9,        /* freshly-computed flash — gold */
    PAIR_SUPERNOVA    =  10,        /* reset flash — yellow          */
};

/* Glow decay rates. */
#define COMPUTE_GLOW_DECAY  3.0f    /* gold flash on freshly-set cell */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* Diamond-Square parameters. */
#define INITIAL_ROUGHNESS   0.55f   /* initial random offset magnitude */
#define PERSISTENCE         0.5f    /* roughness *= persistence per level */

/* Terrain band thresholds — applied to NORMALIZED heights ([0, 1] after
 * scaling by min/max of the computed grid) so every band is exercised
 * regardless of where the raw float values landed. */
#define BAND_WATER       0.30f
#define BAND_BEACH       0.40f
#define BAND_GRASS       0.55f
#define BAND_HILL        0.72f
#define BAND_MOUNTAIN    0.88f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Operation kinds for the work queue. */
enum { OP_DIAMOND = 0, OP_SQUARE = 1 };

/*
 * Themes — 10 named colour palettes for the 6 terrain bands. Selected
 * via the 't' key (forward) or 'T' (backward). Each theme is a small
 * 256-colour table; PAIR_HUD/HINT/FLASH/SUPERNOVA stay theme-independent
 * so the UI remains readable across all themes.
 */
typedef struct {
    const char *name;
    short       colors[6];   /* [WATER, BEACH, GRASS, HILL, MOUNTAIN, SNOW] */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* Tuned by eye — each theme runs a coherent gradient from the
     * "deepest" band (water) to the "highest" (snow). */
    { "DEFAULT", {  33, 221,  34,  22, 244, 231 } },  /* sky/sand/green/forest/grey/white */
    { "MATRIX",  {  22,  28,  34,  40,  46, 118 } },  /* shades of green */
    { "NOVA",    {  53,  92, 129, 165, 201, 219 } },  /* purple → pink */
    { "MONO",    { 234, 239, 243, 246, 250, 254 } },  /* greyscale */
    { "OCEAN",   {  17,  20,  27,  33,  39,  51 } },  /* navy → cyan */
    { "FIRE",    {  52, 124, 160, 196, 208, 226 } },  /* dark red → yellow */
    { "EARTH",   {  58, 180,  64,  94, 137, 187 } },  /* dark brown → cream */
    { "FOREST",  {  23,  22,  28,  34, 130, 144 } },  /* dark green → brown → tan */
    { "DESERT",  {  24, 222, 178, 137,  94, 230 } },  /* warm sandy */
    { "ARCTIC",  {  18,  24,  39,  51, 189, 231 } },  /* navy → ice → white */
};

/* Biome filter — index into band_counts[] and the filter set by the
 * w/b/g/h/m/s keys. 0 means "show everything". */
enum {
    FILTER_ALL      = 0,
    FILTER_WATER    = 1,
    FILTER_BEACH    = 2,
    FILTER_GRASS    = 3,
    FILTER_HILLS    = 4,
    FILTER_MOUNTAIN = 5,
    FILTER_SNOW     = 6,
    N_BANDS         = 7,
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

/*
 * theme_apply — install one of the 10 named palettes. Re-initialises
 * the 6 terrain colour pairs only; HUD/HINT/FLASH/SUPERNOVA pairs stay
 * the same across themes so the UI remains legible.
 *
 * Safe to call any time; ncurses' init_pair updates the live pair
 * definition, so cells already on screen redraw with the new colours
 * on the next refresh.
 *
 * On 8-colour terminals all themes degrade to the same fallback set —
 * 256-colour palettes can't render with 8 colours, and remapping each
 * theme to ANSI primaries would just produce 10 indistinguishable
 * variants.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        init_pair(PAIR_WATER,    t->colors[0], -1);
        init_pair(PAIR_BEACH,    t->colors[1], -1);
        init_pair(PAIR_GRASS,    t->colors[2], -1);
        init_pair(PAIR_HILL,     t->colors[3], -1);
        init_pair(PAIR_MOUNTAIN, t->colors[4], -1);
        init_pair(PAIR_SNOW,     t->colors[5], -1);
    } else {
        init_pair(PAIR_WATER,    COLOR_BLUE,    -1);
        init_pair(PAIR_BEACH,    COLOR_YELLOW,  -1);
        init_pair(PAIR_GRASS,    COLOR_GREEN,   -1);
        init_pair(PAIR_HILL,     COLOR_GREEN,   -1);
        init_pair(PAIR_MOUNTAIN, COLOR_WHITE,   -1);
        init_pair(PAIR_SNOW,     COLOR_WHITE,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);   /* reserved bright yellow */
        init_pair(PAIR_HINT,        51, -1);   /* reserved bright cyan   */
        init_pair(PAIR_FLASH,      220, -1);   /* gold                   */
        init_pair(PAIR_SUPERNOVA,  226, -1);   /* yellow                 */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_FLASH,     COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);   /* DEFAULT theme on startup */
}

/* ===================================================================== */
/* §5  ds — Diamond-Square heightmap                                      */
/* ===================================================================== */

/*
 * Op — one cell to compute, on the work queue. The (x, y) is grid
 * coordinates; `kind` is OP_DIAMOND or OP_SQUARE; `step` and `rough`
 * carry the level-specific parameters at the moment this op was
 * scheduled (so each op is self-contained and the level state can
 * advance while ops are still pending).
 */
typedef struct {
    int     x, y;
    uint8_t kind;     /* OP_DIAMOND / OP_SQUARE */
    int     step;     /* full step size at scheduling time */
    float   rough;    /* random-offset magnitude at this level */
} Op;

/*
 * Heightmap — the simulation heart.
 *
 *   w, h          : grid width / height (= N+1 for both, square)
 *   N             : N (= w-1 = h-1, must be power of 2)
 *   height[]      : per-cell height, [0, 1] after clamping
 *   computed[]    : per-cell flag — true once height is set
 *   compute_glow[]: per-cell gold flash magnitude
 *   supernova_glow[]: per-cell reset flash
 *
 *   queue[]       : pending Ops in execution order (one phase fully
 *                   queued before scheduling the next)
 *   qhead, qtail  : queue cursors
 *   level_step    : current step size; halves each level
 *   level_rough   : current random magnitude; *= 0.5 each level
 *   done          : true when level_step would drop below 1
 */
typedef struct {
    int     w, h;
    int     N;

    float   height        [CELLS_MAX];
    bool    computed      [CELLS_MAX];
    float   compute_glow  [CELLS_MAX];
    float   supernova_glow[CELLS_MAX];

    Op      queue[CELLS_MAX];
    int     qhead, qtail;
    int     level_step;
    float   level_rough;
    int     phase;               /* OP_DIAMOND or OP_SQUARE — current sub-phase */
    bool    done;

    int     computed_count;
} Heightmap;

static inline int hm_idx(const Heightmap *h, int x, int y) { return y * h->w + x; }
static inline bool hm_in_bounds(const Heightmap *h, int x, int y)
{
    return x >= 0 && x < h->w && y >= 0 && y < h->h;
}

/*
 * hm_rand — uniform float in [-1, 1]. Called per cell in DIAMOND/SQUARE
 * to produce the random offset (which is then scaled by the level's
 * roughness magnitude). Quality of rand() is fine for this — visual
 * artifacts from poor RNG would require statistical analysis to spot.
 */
static inline float hm_rand_signed(void)
{
    return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

/*
 * hm_schedule_diamond_phase — append every diamond cell of the current
 * level to the queue. Ordering: row-major; sub-tick visual is the
 * "raster sweep through the grid at this level".
 *
 * Diamond cells sit at the centres of squares of the current step.
 * For step S and N = grid_size - 1, the centres are at:
 *   (S/2 + i*S, S/2 + j*S)  for i, j = 0 .. (N/S) - 1
 */
static void hm_schedule_diamond_phase(Heightmap *h)
{
    h->phase = OP_DIAMOND;
    int S = h->level_step;
    int half = S / 2;
    for (int y = half; y < h->N; y += S) {
        for (int x = half; x < h->N; x += S) {
            h->queue[h->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_DIAMOND,
                .step = S, .rough = h->level_rough
            };
        }
    }
}

/*
 * hm_schedule_square_phase — append every square (cross-midpoint) cell
 * of the current level to the queue.
 *
 * Square cells are the "edge midpoints" of the diamond regions. They
 * sit at lattice points on the half-step grid where exactly one of
 * (x, y) is on the full-step grid and the other is offset by half.
 *
 * Equivalent characterisation: every cell at half-step coords whose
 * (x/half + y/half) is ODD. We iterate row-by-row at half-step and
 * pick the right starting offset per row.
 */
static void hm_schedule_square_phase(Heightmap *h)
{
    h->phase = OP_SQUARE;
    int S = h->level_step;
    int half = S / 2;
    for (int y = 0; y <= h->N; y += half) {
        /* On row y: if y/half is even, x_start = half (offset);
         * if odd, x_start = 0 (aligned). Both sequences step by S. */
        int x_start = ((y / half) & 1) ? 0 : half;
        for (int x = x_start; x <= h->N; x += S) {
            h->queue[h->qtail++] = (Op){
                .x = x, .y = y, .kind = OP_SQUARE,
                .step = S, .rough = h->level_rough
            };
        }
    }
}

/*
 * hm_compute_diamond — set grid[x][y] from the four square corners at
 * (x ± half, y ± half).
 */
static void hm_compute_diamond(Heightmap *h, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;
    float a = h->height[hm_idx(h, x - half, y - half)];
    float b = h->height[hm_idx(h, x + half, y - half)];
    float c = h->height[hm_idx(h, x - half, y + half)];
    float d = h->height[hm_idx(h, x + half, y + half)];
    float v = (a + b + c + d) * 0.25f + hm_rand_signed() * op->rough;

    int idx = hm_idx(h, x, y);
    if (!h->computed[idx]) h->computed_count++;
    h->height[idx]       = v;
    h->computed[idx]     = true;
    h->compute_glow[idx] = 1.0f;
}

/*
 * hm_compute_square — set grid[x][y] from up to 4 cardinal neighbours
 * at distance half. Cells along the map borders see only 3 (edges) or
 * 2 (corners — but corners are seeded, never reached). Average uses
 * ONLY in-bounds contributors; out-of-grid neighbours are not counted.
 */
static void hm_compute_square(Heightmap *h, const Op *op)
{
    int half = op->step / 2;
    int x = op->x, y = op->y;
    float sum = 0.0f;
    int count = 0;
    if (hm_in_bounds(h, x - half, y)) { sum += h->height[hm_idx(h, x - half, y)]; count++; }
    if (hm_in_bounds(h, x + half, y)) { sum += h->height[hm_idx(h, x + half, y)]; count++; }
    if (hm_in_bounds(h, x, y - half)) { sum += h->height[hm_idx(h, x, y - half)]; count++; }
    if (hm_in_bounds(h, x, y + half)) { sum += h->height[hm_idx(h, x, y + half)]; count++; }
    if (count == 0) return;             /* shouldn't happen — defensive */
    float v = sum / (float)count + hm_rand_signed() * op->rough;

    int idx = hm_idx(h, x, y);
    if (!h->computed[idx]) h->computed_count++;
    h->height[idx]       = v;
    h->computed[idx]     = true;
    h->compute_glow[idx] = 1.0f;
}

/*
 * hm_advance_level — switch to the next level: halve step, halve
 * roughness, schedule the next DIAMOND phase. The smallest useful
 * step is 2 (half=1); at step=1 the half would be 0 and the
 * schedule_square loop would divide by zero AND infinite-loop on
 * `y += half=0`. So we stop when the next step would be < 2.
 */
static void hm_advance_level(Heightmap *h)
{
    int next = h->level_step / 2;
    if (next < 2) {
        h->done = true;
        return;
    }
    h->level_step  = next;
    h->level_rough *= PERSISTENCE;
    hm_schedule_diamond_phase(h);
}

/*
 * hm_step — perform one cell-computation operation. Returns true if a
 * cell was set, false if the algorithm is done.
 *
 * Phase transitions live here:
 *   • If queue empty AND last op kind was DIAMOND → schedule SQUARE
 *     for the same step.
 *   • If queue empty AND last op kind was SQUARE  → advance to the
 *     next level (halve step, schedule next DIAMOND).
 *
 * We track the "last popped kind" via h->queue[h->qhead-1]. When the
 * queue is empty and qhead > 0, look back at the last entry. (The
 * queue is never re-shrunk; we just advance qhead.)
 */
static bool hm_step(Heightmap *h)
{
    if (h->done) return false;

    /* If queue is empty, advance to the next phase / level. */
    if (h->qhead >= h->qtail) {
        /* No way to know the previous phase if we never had any —
         * but that only happens before init. By construction, after
         * the first hm_schedule_diamond_phase, qhead < qtail. */
        if (h->qhead == 0) return false;

        uint8_t last_kind = h->queue[h->qhead - 1].kind;
        if (last_kind == OP_DIAMOND) {
            hm_schedule_square_phase(h);
        } else {
            hm_advance_level(h);
        }
        if (h->done) return false;
        if (h->qhead >= h->qtail) return false;   /* nothing scheduled */
    }

    Op op = h->queue[h->qhead++];
    if (op.kind == OP_DIAMOND) hm_compute_diamond(h, &op);
    else                       hm_compute_square (h, &op);
    return true;
}

/*
 * hm_seed_corners — reset the grid and seed the four corners with
 * random heights; queue the first DIAMOND phase.
 *
 * w MUST equal h->N + 1; the caller picks N as a power of 2.
 */
static void hm_reset(Heightmap *h, int N)
{
    h->N = N;
    h->w = N + 1;
    h->h = N + 1;

    int n = h->w * h->h;
    for (int i = 0; i < n; i++) {
        h->height[i]         = 0.0f;
        h->computed[i]       = false;
        h->compute_glow[i]   = 0.0f;
        h->supernova_glow[i] = 1.0f;
    }

    h->qhead = 0;
    h->qtail = 0;
    h->level_step  = N;
    h->level_rough = INITIAL_ROUGHNESS;
    h->done = false;
    h->computed_count = 0;

    /* Seed corners — random in [0.2, 0.8] so the initial range is
     * neither trivially flat nor saturated. The roughness then perturbs
     * outward from these. */
    int corners[4][2] = {{0, 0}, {N, 0}, {0, N}, {N, N}};
    for (int i = 0; i < 4; i++) {
        int x = corners[i][0], y = corners[i][1];
        int idx = hm_idx(h, x, y);
        float v = 0.2f + ((float)rand() / (float)RAND_MAX) * 0.6f;
        h->height[idx]       = v;
        h->computed[idx]     = true;
        h->compute_glow[idx] = 1.0f;
        h->computed_count++;
    }

    /* Schedule the first DIAMOND phase. */
    hm_schedule_diamond_phase(h);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   COMPUTING — each tick computes ops_per_tick cells. Transitions to
 *               HOLD when hm_step returns false (algorithm done).
 *   HOLD      — wait HOLD_SECONDS, then hm_reset and back to COMPUTING.
 */
typedef enum {
    SCENE_COMPUTING = 0,
    SCENE_HOLD      = 1,
} SceneState;

typedef struct {
    Heightmap   h;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         ops_per_tick;
    int         grid_N;          /* the N for this run; derived from terminal */
    int         filter_band;     /* FILTER_ALL or FILTER_WATER..SNOW */
    int         band_counts[N_BANDS];  /* [1..6] per band; updated by scene_draw */
    int         current_theme;   /* index into themes[]; 0 = DEFAULT */
} Scene;

/*
 * band_index_for_norm — given a normalised height in [0, 1], return a
 * band index 1..6 (FILTER_WATER..FILTER_SNOW). Same thresholds as
 * band_for_height — kept in lock-step. Used both for filtering and
 * for counting.
 */
static inline int band_index_for_norm(float v)
{
    if (v < BAND_WATER)    return FILTER_WATER;
    if (v < BAND_BEACH)    return FILTER_BEACH;
    if (v < BAND_GRASS)    return FILTER_GRASS;
    if (v < BAND_HILL)     return FILTER_HILLS;
    if (v < BAND_MOUNTAIN) return FILTER_MOUNTAIN;
    return FILTER_SNOW;
}

static const char *band_name(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return "WATER";
    case FILTER_BEACH:    return "BEACH";
    case FILTER_GRASS:    return "GRASS";
    case FILTER_HILLS:    return "HILLS";
    case FILTER_MOUNTAIN: return "MOUNTAIN";
    case FILTER_SNOW:     return "SNOW";
    default:              return "ALL";
    }
}

static int band_color_pair(int filter)
{
    switch (filter) {
    case FILTER_WATER:    return PAIR_WATER;
    case FILTER_BEACH:    return PAIR_BEACH;
    case FILTER_GRASS:    return PAIR_GRASS;
    case FILTER_HILLS:    return PAIR_HILL;
    case FILTER_MOUNTAIN: return PAIR_MOUNTAIN;
    case FILTER_SNOW:     return PAIR_SNOW;
    default:              return PAIR_HUD;
    }
}

static void scene_reset(Scene *s)
{
    hm_reset(&s->h, s->grid_N);
    s->state      = SCENE_COMPUTING;
    s->hold_timer = 0.0f;
    /* filter_band is preserved across resets — pressing 'r' or a
     * filter key gives a fresh heightmap with the same filter still
     * applied. Counts must reset because the new heightmap will
     * accumulate them from scratch. */
    for (int i = 0; i < N_BANDS; i++) s->band_counts[i] = 0;
}

static void scene_init(Scene *s, int grid_N)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    s->grid_N        = grid_N;
    s->filter_band   = FILTER_ALL;
    s->current_theme = 0;
    scene_reset(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay glows. */
    float compute_d = expf(-COMPUTE_GLOW_DECAY * dt);
    float nova_d    = expf(-SUPERNOVA_DECAY    * dt);
    int n = s->h.w * s->h.h;
    for (int i = 0; i < n; i++) {
        s->h.compute_glow[i]   *= compute_d;
        s->h.supernova_glow[i] *= nova_d;
    }

    switch (s->state) {

    case SCENE_COMPUTING:
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!hm_step(&s->h)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Same canonical pattern as other procedural files.
 *
 * Per-cell render:
 *   supernova_glow > thresh → yellow '*'
 *   compute_glow   > thresh → gold '*' bold (recently computed)
 *   computed                → terrain band (colour + glyph by height)
 *   else                    → blank (uncomputed; treated as deep void)
 *
 * The terrain band classifier is `band_for_height` below.
 */
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

/*
 * pick_grid_N — choose the largest N=2^n such that the resulting
 * (N+1) × (N+1) grid fits inside the terminal with HUD margins.
 *
 * Falls back to GRID_N_MIN if the terminal is too small.
 */
static int pick_grid_N(int cols, int rows)
{
    int avail_w = cols;
    int avail_h = rows - 2;
    int min_dim = (avail_w < avail_h) ? avail_w : avail_h;
    int N = 1;
    while ((N * 2) + 1 <= min_dim && (N * 2) <= GRID_N_MAX) {
        N *= 2;
    }
    if (N < GRID_N_MIN) N = GRID_N_MIN;
    if (N > GRID_N_MAX) N = GRID_N_MAX;
    return N;
}

/*
 * band_for_height — classify a (clamped) [0,1] height into a terrain
 * band, returning the colour pair, attribute, and ASCII glyph.
 *
 * Bands are ordered by altitude. Boundary values (BAND_*) are tunable
 * in §1; tighter bands give a more "ribbon" look, wider bands smoother.
 */
static void band_for_height(float h, int *pair, int *attr, char *glyph)
{
    *attr = A_NORMAL;
    if (h < BAND_WATER) {
        *pair = PAIR_WATER;    *glyph = '~';
    } else if (h < BAND_BEACH) {
        *pair = PAIR_BEACH;    *glyph = '.';
    } else if (h < BAND_GRASS) {
        *pair = PAIR_GRASS;    *glyph = ',';
    } else if (h < BAND_HILL) {
        *pair = PAIR_HILL;     *glyph = ';';
    } else if (h < BAND_MOUNTAIN) {
        *pair = PAIR_MOUNTAIN; *glyph = '#';
    } else {
        *pair = PAIR_SNOW;     *glyph = '@';  *attr = A_BOLD;
    }
}

static void scene_draw(Scene *s, int cols, int rows)
{
    Heightmap *h = &s->h;

    /* Reset per-frame band counts — repopulated as we classify cells. */
    for (int i = 0; i < N_BANDS; i++) s->band_counts[i] = 0;

    int gx0 = (cols - h->w) / 2;
    /* Reserve row 0 (primary HUD), row 1 (secondary HUD), and the last
     * row (hint strip). Map gets the (rows - 3) middle rows. */
    int gy0 = ((rows - 3) - h->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* Find min/max of computed heights so we can normalise to [0, 1].
     * Without this, heights drift outside [0, 1] from the accumulated
     * random offsets, get clamped at the extremes, and most cells end
     * up squashed into either the deepest or highest band — only ' '
     * and '@' would render. Normalising spreads the distribution
     * evenly across all six band glyphs (~ . , ; # @). */
    float hmin = 1.0f, hmax = 0.0f;
    int n_total = h->w * h->h;
    bool any_computed = false;
    for (int i = 0; i < n_total; i++) {
        if (!h->computed[i]) continue;
        float v = h->height[i];
        if (!any_computed) { hmin = v; hmax = v; any_computed = true; }
        else { if (v < hmin) hmin = v; if (v > hmax) hmax = v; }
    }
    float hspan = (hmax - hmin > 1e-6f) ? (hmax - hmin) : 1.0f;

    for (int y = 0; y < h->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < h->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = hm_idx(h, x, y);
            float ng = h->supernova_glow[idx];
            float cg = h->compute_glow[idx];

            int  pair = -1, attr = A_NORMAL;
            char glyph = ' ';

            if (ng > GLOW_THRESHOLD) {
                pair  = PAIR_SUPERNOVA;
                attr  = A_BOLD;
                glyph = '*';
            } else if (cg > GLOW_THRESHOLD) {
                pair  = PAIR_FLASH;
                attr  = A_BOLD;
                glyph = '*';
            } else if (h->computed[idx]) {
                /* Normalise to [0, 1] using the per-frame min/max so
                 * every band gets used regardless of where the raw
                 * float values landed. */
                float v = (h->height[idx] - hmin) / hspan;
                if (v < 0.0f) v = 0.0f;
                if (v > 1.0f) v = 1.0f;
                int band = band_index_for_norm(v);
                s->band_counts[band]++;

                /* Filter mode: only render the selected band; everything
                 * else stays blank so the user sees JUST that biome. */
                if (s->filter_band != FILTER_ALL && band != s->filter_band)
                    continue;

                band_for_height(v, &pair, &attr, &glyph);
            } else {
                /* Uncomputed — blank. */
                continue;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Heightmap *h = &s->h;

    /* Row 0: title + primary state. The phase/step belong on row 1 so
     * the primary status (PAUSED / running) stays the dominant line. */
    const char *state_str =
        s->paused                     ? "PAUSED   " :
        (s->state == SCENE_COMPUTING) ? "COMPUTING" :
                                        "HOLD     ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops:%-3d  %s  %5d/%-5d ",
             fps, sim_fps, s->ops_per_tick, state_str,
             h->computed_count, h->w * h->h);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title + active filter (if any). When a filter is
     * active, the band name is colour-coded to match its tile colour
     * so the user can see at a glance which biome is on screen. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " DIAMOND-SQUARE  show:");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int show_pair = band_color_pair(s->filter_band);
    attron(COLOR_PAIR(show_pair) | A_BOLD);
    /* %-8s width keeps the field stable as the name length changes
     * (WATER/BEACH/GRASS/HILLS/SNOW are 5 chars; MOUNTAIN is 8). */
    mvprintw(0, 24, "%-9s", band_name(s->filter_band));
    attroff(COLOR_PAIR(show_pair) | A_BOLD);

    /* Row 1: algorithm detail — current step (lattice spacing, halves
     * each level), current phase (DIAMOND / SQUARE), random magnitude
     * (roughness, halves each level), grid side N. Per the CLAUDE.md
     * HUD Standard, the secondary line uses PAIR_HUD without A_BOLD so
     * row 0 stays the dominant element. */
    const char *phase_str =
        s->paused                     ? "(paused)" :
        (s->state == SCENE_HOLD)      ? "(done)  " :
        (h->phase == OP_DIAMOND)      ? "DIAMOND " :
                                        "SQUARE  ";
    /* Compute level number: log2(N) total levels; current = 1 + how
     * many halvings we've done. Cheap because both N and step are
     * powers of 2, so we use straight loops. */
    int total_levels = 0;
    for (int v = h->N; v > 1; v >>= 1) total_levels++;
    int curr_level   = 0;
    for (int v = h->N; v > h->level_step && v > 1; v >>= 1) curr_level++;
    curr_level += 1;
    if (curr_level > total_levels) curr_level = total_levels;

    char row1[160];
    snprintf(row1, sizeof row1,
             " level %d/%d  step:%-3d  phase:%s  rough:%.3f  N=%d ",
             curr_level, total_levels,
             h->level_step, phase_str, h->level_rough, h->N);
    int rx = sc->cols - (int)strlen(row1);
    if (rx < 0) rx = 0;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, rx, "%s", row1);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Row 1 left — theme name + biome cell counts. Each W/B/G/H/M/S
     * letter is colour-coded to its band so the legend doubles as a
     * swatch (and lets you see the active theme's palette at a glance —
     * cycle themes with 't' / 'T' and watch the swatches change). */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-7s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 16;

    static const struct { int filter; const char *letter; } items[] = {
        { FILTER_WATER,    "W" },
        { FILTER_BEACH,    "B" },
        { FILTER_GRASS,    "G" },
        { FILTER_HILLS,    "H" },
        { FILTER_MOUNTAIN, "M" },
        { FILTER_SNOW,     "S" },
    };
    for (size_t i = 0; i < sizeof items / sizeof items[0]; i++) {
        int p = band_color_pair(items[i].filter);
        attron(COLOR_PAIR(p) | A_BOLD);
        mvprintw(1, x, "%s", items[i].letter);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, x, ":%-4d ", s->band_counts[items[i].filter]);
        attroff(COLOR_PAIR(PAIR_HUD));
        x += 6;
    }

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " filter:w/b/g/h/m/s a:all  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
    int                   grid_N;
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
    app->grid_N = pick_grid_N(app->screen.cols, app->screen.rows);
    app->scene.grid_N = app->grid_N;
    scene_reset(&app->scene);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s);
        break;

    /* Biome filter keys — set the filter to one band and reset the
     * heightmap so the user sees a fresh reveal showing only that
     * biome's cells. 'a' clears the filter back to "show all". */
    case 'w': case 'W': s->filter_band = FILTER_WATER;    scene_reset(s); break;
    case 'b': case 'B': s->filter_band = FILTER_BEACH;    scene_reset(s); break;
    case 'g': case 'G': s->filter_band = FILTER_GRASS;    scene_reset(s); break;
    case 'h': case 'H': s->filter_band = FILTER_HILLS;    scene_reset(s); break;
    case 'm': case 'M': s->filter_band = FILTER_MOUNTAIN; scene_reset(s); break;
    case 's': case 'S': s->filter_band = FILTER_SNOW;     scene_reset(s); break;
    case 'a': case 'A': s->filter_band = FILTER_ALL;      scene_reset(s); break;

    /* Theme cycling — 't' next, 'T' previous. Doesn't reset the
     * heightmap; just re-installs the colour pairs and the next
     * frame redraws with the new palette. */
    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    app->grid_N = pick_grid_N(app->screen.cols, app->screen.rows);
    scene_init(&app->scene, app->grid_N);

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
