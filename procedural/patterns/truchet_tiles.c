/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * truchet_tiles.c
 *   — Truchet tilings: take ONE simple tile (e.g., a square split
 *     diagonally), randomly rotate it at every cell of a grid, and
 *     watch a complex emergent pattern arise from a single primitive.
 *     Sébastien Truchet's 1704 observation, in ASCII.
 *
 * DEMO: A grid of '/' and '\' — randomly per cell — fills the
 *       screen. The diagonals join across cell boundaries to form
 *       swirls, mazes, and meandering ribbons that look anything
 *       but random; that emergent structure is the Truchet effect.
 *       A slow fractal-noise "light field" drifts across the screen
 *       making different regions of the static pattern brighten and
 *       dim, as if a spotlight sweeps the surface.
 *
 *       PATTERN and GLYPH SET are independent axes.
 *
 *       PATTERN controls HOW the orientation field is DISTRIBUTED
 *       across the grid — i.e., the "noise" the tiling is sampled
 *       from. Cycle with n / p:
 *
 *         RANDOM   uniform per-cell hash. Adjacent cells uncorrelated.
 *                  The classic Truchet "white-noise" look.
 *         NOISE    fBm-correlated. Smooth flowing regions of same
 *                  orientation, like clouds carved into Truchet.
 *         BANDS    sinusoidal field — diagonal stripes alternate
 *                  between orientations.
 *         VORONOI  jittered-seed regions. Irregular blocks share
 *                  one orientation; boundaries follow the Voronoi
 *                  diagram of the random seeds.
 *
 *       GLYPH SET controls WHICH characters represent each
 *       orientation, and how many orientations are available. Cycle
 *       with g / G — 12 sets across three structural families:
 *
 *         2-orient × 1-cell : diag(/\), lens(()), brkt([]), wave(~-)
 *         4-orient × 1-cell : axis(/\_|), cross(/\+X),
 *                             arrow(<>^v), dots(oO#@)
 *         2-orient × 2-cell : slope(,'), tri(<>),
 *                             wcurv(()), wbrkt([])
 *
 *       'r' reseeds — same algorithm, fresh arrangement of the
 *       chosen pattern.
 *
 * Study alongside:
 *   ../worldgen/cloud.c
 *      — same fBm-modulated-brightness trick used to add a flowing
 *        light field on top of an underlying pattern.
 *   ../fractal_random/automaton_2d.c
 *      — also "complexity from a simple per-cell rule applied
 *        uniformly". Truchet is the static analogue: no time
 *        evolution, just per-cell random orientation.
 *
 * Section map:
 *   §1 config    — glyph tables, themes, animation parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 truchet   — hash, perlin/fbm, per-pattern glyph picker
 *   §6 scene     — state, scene_tick (drifts light field over time)
 *   §7 screen    — render with glyph (hash) + brightness (fBm)
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume light-field drift
 *   r          reseed (new tile arrangement)
 *   n / N      next pattern  (CLASSIC → QUAD → WAVE)
 *   p / P      previous pattern
 *   g / G      next / previous glyph set within the current pattern
 *   t / T      next / previous theme
 *   + / =      faster light drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/truchet_tiles.c \
 *     -o truchet -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *
 *                  (1) TRUCHET TILING — Sébastien Truchet noticed
 *                      (1704) that a single decorative tile can
 *                      generate effectively unlimited PATTERNS just
 *                      by being randomly rotated at every cell of a
 *                      grid. Pick a tile (a square divided diagonally
 *                      black/white). Pick a per-cell rotation from
 *                      its symmetry group (C4 for a 4-rotation tile,
 *                      or just the 2-rotation flip for the diagonal-
 *                      split version). The resulting tiling has
 *                      structure at every scale — no two cells
 *                      decide the global pattern, but their
 *                      collective choices reveal swirls, mazes,
 *                      labyrinths and Voronoi-like blobs.
 *
 *                      For ASCII: a "tile" is one or more screen
 *                      cells; the rotation is encoded in WHICH
 *                      glyph(s) we draw there. Per-cell rotation
 *                      is picked deterministically from a hash of
 *                      (cell_x, cell_y, seed) — same seed always
 *                      reproduces the same pattern; reseed = new
 *                      pattern.
 *
 *                  (2) FBM BRIGHTNESS FIELD — to give the static
 *                      pattern a sense of motion without changing
 *                      it, we modulate per-cell BRIGHTNESS by a
 *                      slow-drifting fractional-Brownian noise
 *                      sampled at (x + wind·t, y). The Truchet
 *                      glyph at each cell stays the same, but its
 *                      colour-ramp index changes as the fBm field
 *                      sweeps overhead. The visual is a static
 *                      pattern lit by a slowly moving spotlight —
 *                      the same trick demosceners use to "animate"
 *                      texture rendering without redrawing it.
 *
 *                  Together: a pure-function pattern (Truchet glyph
 *                  from hash) overlaid with a smoothly-evolving
 *                  field (fBm brightness). No allocation; the only
 *                  state is a few floats.
 *
 * Data-structure : NONE. Like the cloud and parallax-star showcases,
 *                  every render is a pure function of (cell_x,
 *                  cell_y, seed, time). No grid is stored.
 *
 * Rendering      : ASCII only. For each cell:
 *                    glyph = truchet_glyph(sx, sy, seed, pattern)
 *                    bright = fbm2(sx · S + wind·t, sy · S · ASPECT)
 *                    level  = floor(bright · 8)            // 0..7
 *                    attr   = level >= 6 ? A_BOLD
 *                           : level <= 1 ? A_DIM
 *                           : A_NORMAL
 *                    render at PAIR_RAMP_BASE + level
 *
 * Performance    : One Perlin-fBm call per cell per frame (3 octaves)
 *                  plus a single hash. ~150 ns/cell × 19 200 × 60 fps
 *                  ≈ 175 ms/sec, comfortable on any modern CPU. No
 *                  allocations.
 *
 * References     :
 *   • Truchet, S. (1704) — "Mémoire sur les Combinaisons", Mémoires
 *     de l'Académie Royale des Sciences. The original paper.
 *   • Smith, C. (1987) — "The tiling patterns of Sébastien Truchet
 *     and the topology of structural hierarchy", Leonardo 20(4).
 *     The modern revival; introduces the quarter-circle variant.
 *   • Browne, C. (2008) — "Truchet curves and surfaces", Computers
 *     & Graphics 32(2):268-281.
 *   • Wikipedia — Truchet tiles
 *     https://en.wikipedia.org/wiki/Truchet_tiles
 *   • Inigo Quilez — Truchet shader article
 *     https://iquilezles.org/articles/multiresolution/
 *   • Bridges Math/Art conference — many Truchet papers
 *     https://archive.bridgesmathart.org/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A complex pattern need not come from a complex rule. Take ONE tiny
 * design — half a square shaded one way, half the other. Cover a
 * grid with copies of it, rotating each one at random. The resulting
 * surface is anything but trivial: paths, swirls, mazes, knots all
 * appear out of pure local randomness. The cells don't communicate;
 * they each just flip a coin. The pattern is in the EYE — your
 * visual cortex stitches the per-cell diagonals into globally
 * meaningful curves. That's the Truchet effect.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a stack of identical playing cards, each one bearing a
 * single diagonal line from corner to corner. Spread them on a
 * checkerboard, randomly choosing which corner each card's line
 * starts from. Step back. Where two adjacent cards have lines that
 * share an edge endpoint, those lines APPEAR continuous to the eye.
 * Long curves emerge — sometimes closing into circles, sometimes
 * meandering across the whole grid. Variations:
 *   - Cards with TWO diagonals (an X) on some flipped orientation
 *     → cross-junctions in the pattern
 *   - Multi-cell cards (one card spans 2 squares)
 *     → patterns at multiple scales
 *
 * Add LIGHT: shine a slowly-moving spotlight across the pattern.
 * The pattern itself doesn't change; only the BRIGHTNESS shifts.
 * That is the fBm field — a smooth, drifting "luminance" texture
 * we paint OVER the Truchet to bring it to life.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SEED. Pick a 32-bit hash seed.  Reseed reshuffles all tiles.
 *  2. EACH FRAME:
 *     a. Advance the wind: wind_x += WIND·dt·speed.
 *     b. For every screen cell (sx, sy):
 *        i.  Get glyph from TRUCHET — a pure function of (sx, sy,
 *            seed, pattern):
 *              h     = hash3(tile_x(sx), tile_y(sy), seed)
 *              orient = h % N_orientations[pattern]
 *              glyph = GLYPH_TABLE[pattern][orient][sub_position]
 *            For 1-cell tiles, sub_position is always 0.
 *            For 2-cell-wide WAVE, sub_position = sx mod 2.
 *        ii. Get brightness from fBm:
 *              b = fbm2(sx · BSC_X + wind_x,
 *                       sy · BSC_Y · ASPECT_Y)
 *        iii. Map b → level (0..7), pick PAIR_RAMP_BASE + level,
 *             pick attr (A_DIM / A_NORMAL / A_BOLD).
 *        iv.  mvaddch(sy, sx, glyph).
 *  3. HUD on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Tile-cell coordinate:
 *     tile_x = sx / TILE_W      // for 2-cell-wide WAVE, TILE_W=2
 *     tile_y = sy / TILE_H      // 1 for all current patterns
 *     sub    = sx mod TILE_W
 *
 *  Truchet orientation:
 *     orient = hash3(tile_x, tile_y, seed) mod N_orientations
 *
 *  Brightness field:
 *     b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
 *
 *  Brightness → level → attr:
 *     level = clamp(⌊b · 8⌋, 0, 7)
 *     attr  = level >= 6 ? A_BOLD : level <= 1 ? A_DIM : A_NORMAL
 *
 *  Wind:
 *     wind_x' = wind_x + WIND_X_BASE · dt · (speed / SPEED_DEF)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • TILE_W != 1 INDEXING. WAVE has TILE_W = 2; the LEFT cell of a
 *    tile uses sub = 0, the RIGHT cell uses sub = 1. With negative
 *    sx, sx % 2 may give -1 in C; bias by adding TILE_W and taking
 *    modulo again. We render starting at sx = 0 only, so this is
 *    moot in this file; flag it if you ever scroll into negatives.
 *
 *  • HASH STABILITY. The orientation hash uses (tile_x, tile_y,
 *    seed). seed is fixed until 'r' reseeds. tile_x, tile_y are
 *    integer screen coordinates — they DO NOT include wind_x.
 *    The Truchet pattern is therefore STATIC; only the brightness
 *    field drifts. Adding wind_x to the hash inputs would cause the
 *    pattern itself to scroll, which IS a valid look but loses the
 *    "static art with moving light" effect we're going for.
 *
 *  • FBM BRIGHTNESS NEEDS NORMALISATION. fbm2 returns roughly
 *    [0, 1]; we use it directly. If the noise occasionally exceeds
 *    1 due to the Perlin gradient bound (technically possible),
 *    clamp before mapping to level — otherwise level can wrap
 *    past 7.
 *
 *  • ASPECT IN BRIGHTNESS. Multiply sy by ASPECT_Y when sampling
 *    fBm so the "spotlight" appears as a circular blob, not a
 *    horizontal oval. The Truchet glyph itself does NOT need
 *    aspect correction — it tiles on the discrete cell grid.
 *
 *  • DON'T USE A_DIM FOR LEVEL 0. With dark theme entries plus
 *    A_DIM, level-0 cells become invisible — the user sees the
 *    Truchet pattern with random GAPS where the spotlight is
 *    weakest. Keep level 0 at the brightest legible tier (A_DIM
 *    over a still-bright ramp slot is OK; A_DIM over a near-black
 *    ramp slot is not). The bright-half theme palette enforces this.
 *
 *  • WIND CONTINUITY. wind_x grows monotonically; after ~10^7
 *    cells of drift, single-precision Perlin starts binning the
 *    same integer cells. For an interactive showcase that runs
 *    minutes at most, ignore.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). The fBm field freezes; the bright "spotlight"
 *    stops moving; the Truchet pattern was already static. Resume:
 *    spotlight slides from exactly where it stopped.
 *
 *  • Press 'r'. Sky-flash, then the tile arrangement REORGANISES —
 *    same algorithm, new orientations. The brightness field is
 *    unchanged (only the seed for orientation changes).
 *
 *  • CLASSIC pattern. Trace a continuous diagonal line across the
 *    screen — it should connect across multiple cells, occasionally
 *    bending where neighbouring cells happen to flip orientation.
 *    Long ribbons and closed loops are common; that's the Truchet
 *    signature.
 *
 *  • QUAD pattern. Each cell is one of '/' '\\' '_' '|' — half are
 *    DIAGONAL, half are AXIS-ALIGNED. Lines now run horizontally
 *    and vertically too, creating maze-like channels.
 *
 *  • CROSSED pattern. Watch for '+' and 'X' cells — those are
 *    intersections. They appear at ~50 % of cells (4-orientation
 *    set with 2 of 4 being intersection glyphs).
 *
 *  • WAVE pattern. Adjacent COLUMN PAIRS form ",'" or "',"
 *    — try to spot the pair-pair structure. Random pair choice
 *    yields a saw-tooth wave look across rows.
 *
 *  • Theme cycle (t / T). Spotlight intensity should be visible
 *    against EVERY theme — even MATRIX (greens) and OCEAN (blues)
 *    should show clear contrast between the dim and bright
 *    spotlight zones.
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
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 brightness tints      */
    PAIR_HOT            =  11,    /* hot accent (unused by default)   */
    PAIR_COLD           =  12,    /* cold accent (unused by default)  */
    PAIR_FLASH          =  13,    /* reseed flash                     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/* Brightness fBm parameters — slow, smooth field that drifts left to
 * right over the static Truchet pattern. */
#define BRIGHT_SCALE_X       0.040f
#define BRIGHT_SCALE_Y       0.080f
#define WIND_X_BASE          3.0f      /* cells / sec at SPEED_DEF */
#define FBM_OCTAVES          3

/*
 * Pattern enum — four UNDERLYING DISTRIBUTIONS. Each pattern is a
 * different way of mapping (tile_x, tile_y) → an orientation value
 * in [0, 1]. The orientation COUNT is determined by the active
 * GLYPH SET (cycled with g / G), not the pattern.  This is the
 * key separation: PATTERN says HOW orientations are distributed
 * across the grid (random, noise-correlated, banded, region-based);
 * GLYPH SET says WHICH characters each orientation maps to and how
 * many orientations are available.
 */
typedef enum {
    PATTERN_RANDOM  = 0,    /* uniform per-cell hash (classic Truchet) */
    PATTERN_NOISE   = 1,    /* fBm — smooth flowing regions            */
    PATTERN_BANDS   = 2,    /* sinusoidal — diagonal stripes           */
    PATTERN_VORONOI = 3,    /* jittered seeds — irregular blocks       */
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_BANDS:   return "BANDS  ";
    case PATTERN_VORONOI: return "VORONOI";
    default:              return "?      ";
    }
}

/* Pattern-specific tuning constants. */
#define NOISE_SCALE_X        0.045f     /* fBm spatial scale          */
#define NOISE_SCALE_Y        0.090f
#define BANDS_FREQ_X         0.25f      /* sinusoid spatial frequency */
#define BANDS_FREQ_Y         0.45f
#define VORONOI_GRID_X       6          /* coarse-cell width  (cols)  */
#define VORONOI_GRID_Y       3          /* coarse-cell height (rows)  */

/*
 * Glyph set — pairs a glyph collection with the tile metadata needed
 * to render it. n_orient = number of orientations the set supports;
 * tile_w = tile width in screen cells. The pattern's [0, 1] value is
 * quantised into n_orient buckets to pick the rotation; the chosen
 * row of glyphs (orient × tile_w bytes) is then read to render the
 * tile_w cells of the tile.
 *
 * 12 sets covering three structural families:
 *   2-orient × 1-cell : diag, lens, brkt, wave
 *   4-orient × 1-cell : axis, cross, arrow, dots
 *   2-orient × 2-cell : slope, tri, wcurv, wbrkt
 */
typedef struct {
    const char *name;
    int  n_orient;
    int  tile_w;
    char glyphs[8];     /* n_orient * tile_w characters in order      */
} GlyphSet;

static const GlyphSet GLYPH_SETS[] = {
    /*  name      orient tile glyphs                                       */
    {  "diag ",    2,    1,   { '/', '\\',                  0, 0, 0, 0, 0, 0 } },
    {  "lens ",    2,    1,   { '(', ')',                   0, 0, 0, 0, 0, 0 } },
    {  "brkt ",    2,    1,   { '[', ']',                   0, 0, 0, 0, 0, 0 } },
    {  "wave ",    2,    1,   { '~', '-',                   0, 0, 0, 0, 0, 0 } },
    {  "axis ",    4,    1,   { '/', '\\', '_', '|',        0, 0, 0, 0 } },
    {  "cross",    4,    1,   { '/', '\\', '+', 'X',        0, 0, 0, 0 } },
    {  "arrow",    4,    1,   { '<', '>',  '^', 'v',        0, 0, 0, 0 } },
    {  "dots ",    4,    1,   { 'o', 'O',  '#', '@',        0, 0, 0, 0 } },
    {  "slope",    2,    2,   { ',', '\'', '\'', ',',       0, 0, 0, 0 } },
    {  "tri  ",    2,    2,   { '<', '>',  '>',  '<',       0, 0, 0, 0 } },
    {  "wcurv",    2,    2,   { '(', ')',  ')',  '(',       0, 0, 0, 0 } },
    {  "wbrkt",    2,    2,   { '[', ']',  ']',  '[',       0, 0, 0, 0 } },
};
#define N_GLYPH_SETS ((int)(sizeof GLYPH_SETS / sizeof GLYPH_SETS[0]))

/*
 * Themes — same 10-name menu as the rest of the showcases. Every
 * entry sits in the bright half of the 256-colour space so even
 * A_DIM cells stay legible against a default-black terminal.
 * See "Theme Palette Brightness" in /CLAUDE.md.
 *
 * The 8-step ramp is read directly by the brightness mapper:
 *   level 0 = ramp[0] (faintest), level 7 = ramp[7] (peak spotlight).
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
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 }, 196,  39 },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 }, 226,  39 },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 }, 196,  39 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226,  39 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 196,  21 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226,  21 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 196,  39 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 196,  39 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 196,  21 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 196,  39 },
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
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,  COLOR_RED,    -1);
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
/* §5  truchet — hash, perlin, fbm, glyph picker                          */
/* ===================================================================== */

/* hash3 — same routine as other showcases. */
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

/* Perlin scaffold — copied inline per the self-contained-file rule. */
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

static float fbm2(float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;     /* → [0, 1] */
}

/* ----------------------------------------------------------------------- *
 * Truchet glyph picker.                                                   *
 * ----------------------------------------------------------------------- */

/*
 * pattern_value — map (tile_x, tile_y, seed) to a value in [0, 1].
 * Each pattern is a different DISTRIBUTION of that value across the
 * grid. The caller quantises it into the active glyph set's
 * orientation count to pick the rotation.
 *
 *   RANDOM   — pure white-noise hash. Adjacent tiles uncorrelated.
 *   NOISE    — fBm sampled at the tile coords; smooth blobs of
 *              correlated orientation.
 *   BANDS    — 0.5 + 0.5·sin(x·fx + y·fy); diagonal stripes.
 *   VORONOI  — find the nearest jittered seed in the 3×3 neighbourhood
 *              of the tile's coarse-grid cell; the orientation comes
 *              from that seed's hash. Result is irregular regions of
 *              uniform orientation.
 */
static float pattern_value(Pattern p, int tx, int ty, int seed)
{
    switch (p) {
    case PATTERN_RANDOM: {
        uint32_t h = hash3(tx, ty, seed);
        return (float)(h & 0xFFFFFFu) / 16777216.0f;
    }
    case PATTERN_NOISE: {
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        return fbm2((float)tx * NOISE_SCALE_X + ox,
                    (float)ty * NOISE_SCALE_Y + oy);
    }
    case PATTERN_BANDS: {
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float a     = (float)tx * BANDS_FREQ_X
                    + (float)ty * BANDS_FREQ_Y
                    + phase;
        return 0.5f + 0.5f * sinf(a);
    }
    case PATTERN_VORONOI: {
        int cx = tx / VORONOI_GRID_X;
        int cy = ty / VORONOI_GRID_Y;
        int best_cx = cx, best_cy = cy;
        long best_d = (long)1 << 60;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int gcx = cx + dx, gcy = cy + dy;
                uint32_t h = hash3(gcx, gcy, seed);
                int sx = gcx * VORONOI_GRID_X
                       + (int)(h % (uint32_t)VORONOI_GRID_X);
                int sy = gcy * VORONOI_GRID_Y
                       + (int)((h >> 8) % (uint32_t)VORONOI_GRID_Y);
                long ddx = (long)(sx - tx);
                long ddy = (long)(sy - ty) * 2;     /* aspect */
                long d   = ddx * ddx + ddy * ddy;
                if (d < best_d) { best_d = d; best_cx = gcx; best_cy = gcy; }
            }
        }
        uint32_t h = hash3(best_cx, best_cy, seed ^ 0x9E3779B9);
        return (float)(h & 0xFFFFFFu) / 16777216.0f;
    }
    case N_PATTERNS: break;
    }
    return 0.5f;
}

/*
 * truchet_glyph — deterministic glyph for one screen cell.
 *
 * 1. Resolve the active glyph set: gs->n_orient is the orientation
 *    count, gs->tile_w is the tile width.
 * 2. Convert screen x to (tile_x, sub_x) using gs->tile_w.
 * 3. Sample the pattern field at (tile_x, sy) → v ∈ [0, 1].
 * 4. Quantise v → orient ∈ [0, n_orient).
 * 5. Look up gs->glyphs[orient · tile_w + sub_x].
 *
 * Pure function of (sx, sy, seed, pattern, set_idx); no time
 * dependence. The static Truchet layout is reseeded by 'r'.
 */
static char truchet_glyph(int sx, int sy, int seed, Pattern p, int set_idx)
{
    if (set_idx < 0 || set_idx >= N_GLYPH_SETS) set_idx = 0;
    const GlyphSet *gs = &GLYPH_SETS[set_idx];
    int tile_w = gs->tile_w;
    int tile_x = (sx >= 0) ? (sx / tile_w) : -((-sx + tile_w - 1) / tile_w);
    int sub_x  = sx - tile_x * tile_w;

    float v = pattern_value(p, tile_x, sy, seed);
    int orient = (int)(v * (float)gs->n_orient);
    if (orient < 0)              orient = 0;
    if (orient >= gs->n_orient)  orient = gs->n_orient - 1;

    return gs->glyphs[orient * tile_w + sub_x];
}

/* ----------------------------------------------------------------------- *
 * Brightness field.                                                        *
 * ----------------------------------------------------------------------- */

/*
 * bright_at — sample the slow drifting fBm "spotlight" at a screen
 * cell. Returned value is in [0, 1] (clamped).
 */
static inline float bright_at(int sx, int sy, float wind_x)
{
    float nx = (float)sx * BRIGHT_SCALE_X + wind_x;
    float ny = (float)sy * BRIGHT_SCALE_Y * ASPECT_Y_F;
    float b  = fbm2(nx, ny);
    if (b < 0.0f) b = 0.0f;
    if (b > 1.0f) b = 1.0f;
    return b;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    int     current_glyph_set;    /* 0..N_GLYPH_SETS-1, cycled with g/G */
    int     seed;                 /* drives Truchet orientation         */
    float   time_secs;
    float   wind_x;
    float   flash_t;
} Scene;

/*
 * apply_perm — reshuffle the Perlin perm[] table for the current
 * (seed, pattern) pair. Each pattern gets its own perm view so:
 *   - the BRIGHTNESS spotlight field looks different per pattern
 *   - the NOISE pattern's orientation field also looks different
 * The pattern index is XOR-mixed into the seed so the same (seed,
 * pattern) always produces the same shuffle (deterministic), but
 * cycling patterns gives a fresh field each time.
 */
static void apply_perm(const Scene *s)
{
    perm_shuffle(s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_reseed(Scene *s)
{
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f),
                         (int)(s->wind_x * 100.0f), 0xC0FFEE);
    apply_perm(s);
    s->flash_t = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused            = false;
    s->speed             = SPEED_DEF;
    s->current_theme     = 0;
    s->current_pattern   = PATTERN_RANDOM;
    s->current_glyph_set = 0;
    s->seed              = 0xC0FFEE;
    apply_perm(s);
    s->flash_t           = 1.0f;
}

/*
 * scene_tick — advance time and wind. Truchet pattern is static; only
 * the brightness field drifts. Pause halts wind but not the
 * flash-decay (so the post-reseed flash always fades out).
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->wind_x += WIND_X_BASE * speed_mul * dt;
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
 * scene_draw — for every cell, compute the static Truchet glyph and
 * the dynamic brightness level, then render. The two are independent
 * computations: the glyph is a hash of (sx, sy, seed), the brightness
 * is a Perlin-fBm sample at (sx, sy) shifted by wind_x.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            char glyph = truchet_glyph(sx, sy, s->seed,
                                       s->current_pattern,
                                       s->current_glyph_set);
            float b    = bright_at(sx, sy, s->wind_x);

            int level = (int)(b * 8.0f);
            if (level < 0) level = 0;
            if (level > 7) level = 7;

            int attr;
            if      (level >= 6) attr = A_BOLD;
            else if (level <= 1) attr = A_DIM;
            else                 attr = A_NORMAL;

            int pair = PAIR_RAMP_BASE + level;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Reseed flash overlay. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = top; sy < bottom; sy += 2) {
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

    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";

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
    mvprintw(0, 1, " TRUCHET TILES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + glyph set + theme + ramp swatch + seed. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ",
             GLYPH_SETS[s->current_glyph_set].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

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
             "  seed:%08x  drift:%6.1f ",
             (unsigned)s->seed, (double)s->wind_x);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:drift  spc:pause  r:reseed  q:quit ");
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
        apply_perm(s);                 /* fresh brightness/noise field */
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        apply_perm(s);
        break;

    case 'g':
        s->current_glyph_set = (s->current_glyph_set + 1) % N_GLYPH_SETS;
        break;
    case 'G':
        s->current_glyph_set = (s->current_glyph_set + N_GLYPH_SETS - 1) % N_GLYPH_SETS;
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
