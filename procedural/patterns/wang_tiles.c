/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wang_tiles.c
 *   — Wang tilings: a finite set of square tiles with coloured edges
 *     are placed on a grid such that adjacent tiles share matching
 *     edge colours.  Hao Wang's 1961 construction; the constraint
 *     graph forces continuous "veins" of colour to flow across
 *     tile boundaries.  16-tile complete set with 2 edge colours.
 *
 * DEMO: A grid of multi-cell tiles fills the screen.  Each tile is
 *       a small rectangle whose four edges (north/east/south/west)
 *       carry one of two colours.  Where two tiles meet, the
 *       SHARED edge has the same colour on both sides — that's the
 *       Wang constraint.  The eye reads adjacent same-colour edges
 *       as continuous BANDS flowing across the screen, even though
 *       each tile chose its colour independently.
 *
 *       PATTERN (n / N) — what BIASES the per-cell tile choice
 *       beyond the Wang constraint:
 *
 *         RANDOM    uniform over all valid tiles → maximally varied
 *         NOISE     fBm bias → large blobby same-colour regions
 *         STRIPES   sinusoidal y-bias → horizontal coloured bands
 *         SWIRL     polar bias around screen centre → quartered
 *                   colour map
 *
 *       GLYPH (g / G) — how each tile is rendered:
 *
 *         EDGES     thin '-' '_' '|' borders only
 *         BLOCKS    bold '#' borders + softly-tinted interior
 *         WIRES     through-lines connecting parallel matching
 *                   edges (vertical when N=S, horizontal when E=W)
 *
 *       'r' reseeds the tile placement; pattern and glyph cycling
 *       reshuffle the underlying noise too so the screen feels
 *       genuinely different per pattern.
 *
 * Study alongside:
 *   ../patterns/truchet_tiles.c
 *      — same idea (per-cell rotation of a tile family) but with no
 *        edge constraint.  Wang tiles are Truchet's with the
 *        adjacency rule turned on.
 *   ../generational/wfc_showcase.c
 *      — wave-function collapse generalises Wang-tile constraint
 *        propagation to arbitrary tile sets and supports backtracking;
 *        this file is the simpler "pick first valid candidate"
 *        version.
 *
 * Section map:
 *   §1 config     — glyph tables, themes, animation parameters
 *   §2 clock      — monotonic timer + sleep
 *   §3 color      — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 wang       — tile-set, hash + perlin/fBm, constraint solver,
 *                   per-pattern selector, per-glyph renderer
 *   §6 scene      — state, regenerate on pattern change / reseed
 *   §7 screen     — render whole tile grid + HUD
 *   §8 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume light-field drift
 *   r          reseed tile arrangement
 *   n / N      next pattern  (RANDOM → NOISE → STRIPES → SWIRL)
 *   p / P      previous pattern
 *   g / G      next / previous glyph set (EDGES → BLOCKS → WIRES)
 *   t / T      next / previous theme
 *   + / =      faster light drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/wang_tiles.c \
 *     -o wang -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three stages.
 *
 *                  (1) TILE SET. With 2 edge colours per axis there
 *                      are 2⁴ = 16 distinct tiles — every combination
 *                      of (N, E, S, W) ∈ {0, 1}⁴.  This is a
 *                      "complete" set: for any pair of (W-constraint,
 *                      N-constraint) at least one tile is valid, so
 *                      placement never gets stuck.
 *
 *                  (2) PLACEMENT (constraint solver).  Iterate row-
 *                      major over the grid.  For each cell:
 *                          expected_w = (left  cell exists) ? left.E  : ANY
 *                          expected_n = (above cell exists) ? above.S : ANY
 *                          valid     = filter(tile_set, by W & N)
 *                          chosen    = pick(valid, x, y, seed, pattern)
 *                          grid[x,y] = chosen
 *                      The PATTERN influences which of several valid
 *                      tiles to pick — RANDOM is uniform; NOISE biases
 *                      toward whichever colour an fBm field prefers
 *                      at this point; STRIPES biases by sin(y);
 *                      SWIRL biases by atan2 around the centre.
 *                      The Wang constraint itself is identical
 *                      across patterns; only the bias differs.
 *
 *                  (3) RENDERING.  Each tile is TILE_W × TILE_H
 *                      cells (default 6×3 → roughly square on
 *                      terminals where cells are 2× taller than
 *                      wide).  The active GLYPH set determines how
 *                      to draw each cell within the tile:
 *                        EDGES  : '-' top, '_' bottom, '|' sides,
 *                                 blank interior — the minimum
 *                                 visualisation of the constraint.
 *                        BLOCKS : '#' on edges (BOLD), '.' tinted
 *                                 interior — heavier visual weight.
 *                        WIRES  : draw through-lines through the
 *                                 tile centre when parallel edges
 *                                 share a colour (vertical when
 *                                 N=S; horizontal when E=W).
 *                                 Other edges fall back to thin
 *                                 dim border stubs.
 *
 *                  Plus a slow-drifting fBm BRIGHTNESS field
 *                  (same trick as truchet_tiles.c) modulates per-
 *                  cell A_DIM / A_NORMAL / A_BOLD over the static
 *                  tile placement, so the visual is alive even
 *                  without re-tiling.
 *
 * Data-structure : 16-entry tile_set[] plus a flat WangGrid of
 *                  uint8_t tile-indices, one per tile cell. For a
 *                  240×80 screen with 6×3 tiles, the grid is at
 *                  most 40×26 = 1 040 bytes. No allocation.
 *
 * Rendering      : ASCII only. Edge colours map to two ramp slots
 *                  in the active theme (mid + bright). Brightness
 *                  modulation overlays via attr; the tile colour
 *                  itself never changes mid-frame.
 *
 * Performance    : Generation is O(grid_w · grid_h · N_TILES) — about
 *                  1 040 × 16 = 17 000 ops, microseconds. Run only
 *                  on reseed / pattern change. Per-frame render is
 *                  one fBm call per cell for the brightness field;
 *                  ~100 ms/sec on a 240×80 screen. Trivial.
 *
 * References     :
 *   • Wang, H. (1961) — "Proving theorems by pattern recognition II",
 *     Bell System Technical Journal 40:1-41.
 *     The original Wang tile paper.
 *   • Berger, R. (1966) — "The Undecidability of the Domino Problem".
 *     Disproved Wang's "every tileable set is periodically tileable"
 *     conjecture by exhibiting an aperiodic 20 426-tile set.
 *   • Cohen, M. F. et al. (2003) — "Wang Tiles for Image and Texture
 *     Generation", SIGGRAPH 2003.
 *     The modern computer-graphics revival.
 *   • Wikipedia — Wang tile
 *     https://en.wikipedia.org/wiki/Wang_tile
 *   • Inigo Quilez — "Wang tiles" article
 *     https://iquilezles.org/articles/voronoise/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a small library of square tiles. Each tile has FOUR edges,
 * each painted in one of a few colours. Rule: when you place tile
 * A next to tile B, the edge they SHARE must match in colour. Now
 * fill a grid by repeatedly picking any tile that satisfies the rule
 * at this position. The rule is local — only the immediate north
 * and west neighbours influence the choice — but the consequence is
 * GLOBAL: same-colour edges line up across many tile boundaries to
 * form continuous bands of colour flowing through the picture. The
 * pattern looks designed; nobody designed it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a child's wooden block set. Each block is a cube with
 * coloured stripes painted on its four side faces. You stack the
 * blocks on a table such that any TWO TOUCHING SIDE FACES wear the
 * same paint colour. Doing this strictly forces stripes that span
 * many blocks — a red stripe carrying through five blocks,
 * separated from a blue stripe by a clean dividing line. Step back:
 * what looks like one elaborate continuous painting is actually 25
 * independently-chosen blocks held together by a single matching
 * rule.
 *
 * The PATTERN (n / N) is what makes the random tile choice prefer
 * red OR blue at each spot — but the matching rule is the same.
 * Pattern bias produces large red regions vs random patches vs
 * banded stripes; the underlying machinery is identical.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Build the 16-tile complete set. tile_set[i] has edges
 *     (N, E, S, W) = (i bits 3..0).
 *  2. GENERATE the grid (whenever seed or pattern changes):
 *     for ty in 0..H:
 *       for tx in 0..W:
 *         if (tx > 0): expected_w = grid[ty][tx-1].E
 *         if (ty > 0): expected_n = grid[ty-1][tx].S
 *         valid = { tile : tile.W matches and tile.N matches }
 *         scored = score each valid tile by pattern's preference
 *                  (RANDOM: 0 for all; NOISE: prefer if E,S match
 *                  fBm-biased colour; STRIPES: prefer if S matches
 *                  sin(y)-biased colour; SWIRL: prefer if E,S match
 *                  atan2(y - cy, x - cx)-biased colour)
 *         chosen = highest-scored, with hash-based tiebreak
 *         grid[ty][tx] = chosen
 *  3. RENDER each frame:
 *     for sy in screen_rows:
 *       for sx in screen_cols:
 *         tx = (sx − gx0) / TILE_W
 *         ty = (sy − gy0) / TILE_H
 *         dx = sx mod TILE_W, dy = sy mod TILE_H
 *         tile = grid[ty][tx]
 *         (glyph, pair) = glyph_set_render(tile, dx, dy)
 *         brightness    = fbm2(sx · BSC_X + wind, sy · BSC_Y · ASPECT)
 *         attr          = level_to_attr(brightness)
 *         mvaddch(sy, sx, glyph) in pair + attr
 *
 * KEY FORMULAS
 * ────────────
 *  Tile index encoding:
 *    tile_set[i].n = (i >> 3) & 1
 *    tile_set[i].e = (i >> 2) & 1
 *    tile_set[i].s = (i >> 1) & 1
 *    tile_set[i].w = (i >> 0) & 1
 *
 *  Wang constraint at cell (tx, ty):
 *    valid = { tile : (tx == 0 OR tile.W = grid[ty][tx-1].E) AND
 *                     (ty == 0 OR tile.N = grid[ty-1][tx].S) }
 *
 *  Pattern preference for edges to bias:
 *    RANDOM   : no preference
 *    NOISE    : prefer_e = prefer_s = (fbm(tx·s, ty·s) > 0.5)
 *    STRIPES  : prefer_s = (sin(ty·k) > 0)
 *    SWIRL    : θ        = atan2((ty − cy)·2, tx − cx)
 *               prefer_e = prefer_s = (θ > 0)
 *
 *  Score and pick:
 *    score(tile)     = (tile.E matches prefer_e ? 1 : 0)
 *                    + (tile.S matches prefer_s ? 1 : 0)
 *    chosen = uniform random over { valid : score = max_score }
 *
 *  Brightness modulation:
 *    b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
 *    attr = b > 0.65 → A_BOLD ; b < 0.35 → A_DIM ; else A_NORMAL
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • COMPLETE SET. With 2 colours per axis and 16 tiles, EVERY
 *    (W, N) constraint pair has exactly 4 valid tiles (free
 *    choice over E and S). Placement never gets stuck. If you cut
 *    the set down to fewer than 16 tiles — to make the visual
 *    more constrained — you must verify that for ALL 4 possible
 *    (W, N) pairs, at least one tile remains valid.
 *
 *  • EDGE OF GRID. The leftmost column has no W neighbour; the
 *    top row has no N neighbour. Treat the missing constraint as
 *    "any" (tile can have either edge value). The corner tile
 *    (0, 0) is unconstrained and picks freely from all 16.
 *
 *  • PREFERENCE ≠ HARD CONSTRAINT. The pattern's preference biases
 *    the choice but never overrides the Wang constraint. NOISE
 *    might want a tile with E = 1, but if no valid tile has E = 1
 *    given the W/N constraint, the highest-scoring valid tile (E = 0)
 *    is taken instead. This keeps placement always succeeding.
 *
 *  • GRID SIZE VS SCREEN SIZE. The grid is rounded down to whole
 *    tiles: grid_w = screen_w / TILE_W, grid_h = (screen_h − HUD)
 *    / TILE_H. Cells at the screen edge that fall outside the
 *    last tile boundary are not drawn (left blank).
 *
 *  • TILE_W / TILE_H ASPECT. With cells 2× taller than wide, a 6×3
 *    tile is 6 cells horizontally and 6 "cell-widths" vertically
 *    (3 cells × 2) — visually square. Smaller tiles look
 *    horizontally squashed.
 *
 *  • BRIGHTNESS DRIFT TICK-COUPLED. wind_x advances per second of
 *    sim time; on slow ticks the visual drift slows. For
 *    consistent visual speed across tick rates, scale by dt
 *    (already done) and rely on the fixed-step accumulator.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • EDGES pattern. Trace any vertical line where two tiles meet.
 *    The right border ('|') of one tile and the left border ('|')
 *    of the next tile are adjacent CELLS on screen — and by Wang
 *    constraint they're the same colour. You should never see a
 *    sharp colour change at a tile boundary; only at the
 *    ENCAPSULATED edges between tiles where the colour MATCHES.
 *
 *  • RANDOM pattern. Tile diversity is maximum; no large regions
 *    of same colour. The whole grid is uniformly speckled.
 *
 *  • NOISE pattern. Same constraint, different bias: now you should
 *    see large continuous regions of "mostly blue" and "mostly
 *    yellow" (or whatever the theme calls them) separated by
 *    irregular boundaries. Like fBm clouds.
 *
 *  • STRIPES pattern. The S edge bias creates horizontal BANDS:
 *    rows alternate between mostly-colour-0 and mostly-colour-1.
 *    The bands move slightly when you reseed because the sin
 *    phase is seeded.
 *
 *  • SWIRL pattern. Bias angles around the centre into 4 quadrants;
 *    you should see two halves of the grid favour different colours,
 *    splitting roughly along a horizontal-ish line through the
 *    middle.
 *
 *  • Switch GLYPH (g) within a fixed pattern. The TILE LAYOUT does
 *    not change; only the rendering (glyphs and interior fills) does.
 *    Tile boundaries are at the same positions; constraint matching
 *    is still visible (just with different glyphs).
 *
 *  • Pause (space). Brightness wave freezes; tiles stay where they
 *    are. Resume: drift continues from where it stopped.
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

    /* Tile dimensions in screen cells. 6 × 3 → ~square on terminals
     * with 2:1 cell aspect. */
    TILE_W              =   6,
    TILE_H              =   3,

    /* 16-tile complete set: 2 edge colours per axis. */
    N_EDGE_COLORS       =   2,
    N_TILES             =  16,

    /* Hard upper bounds on the tile grid. With TILE_W=6 and TILE_H=3
     * across a 240×80 screen, max grid is 40 × 26. */
    MAX_SCREEN_W        = 256,
    MAX_SCREEN_H        =  96,
    MAX_GRID_W          = MAX_SCREEN_W / TILE_W,
    MAX_GRID_H          = MAX_SCREEN_H / TILE_H,
    MAX_GRID_CELLS      = MAX_GRID_W * MAX_GRID_H,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 brightness/colour tints */
    PAIR_HOT            =  11,
    PAIR_COLD           =  12,
    PAIR_FLASH          =  13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/* Brightness fBm parameters — slow drift, same trick as truchet. */
#define BRIGHT_SCALE_X       0.040f
#define BRIGHT_SCALE_Y       0.080f
#define WIND_X_BASE          3.0f
#define FBM_OCTAVES          3

/* Edge-color → ramp-slot mapping. With 2 edge colours and a 0..7
 * ramp, pick two well-separated slots so the colours read distinctly:
 *   colour 0 → ramp[3] (mid-bright)
 *   colour 1 → ramp[6] (highlight)
 */
static const int EDGE_RAMP[N_EDGE_COLORS] = { 3, 6 };

/* Pattern enum — what BIASES the random choice of valid tile. */
typedef enum {
    PATTERN_RANDOM  = 0,
    PATTERN_NOISE   = 1,
    PATTERN_STRIPES = 2,
    PATTERN_SWIRL   = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_STRIPES: return "STRIPES";
    case PATTERN_SWIRL:   return "SWIRL  ";
    default:              return "?      ";
    }
}

/* Glyph enum — how each tile cell is RENDERED. */
typedef enum {
    GLYPH_EDGES  = 0,
    GLYPH_BLOCKS = 1,
    GLYPH_WIRES  = 2,
    N_GLYPH_SETS = 3,
} GlyphSet;

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_EDGES:  return "edge ";
    case GLYPH_BLOCKS: return "block";
    case GLYPH_WIRES:  return "wire ";
    default:           return "?    ";
    }
}

/* Pattern bias parameters. */
#define NOISE_SCALE_X        0.06f
#define NOISE_SCALE_Y        0.12f
#define STRIPES_FREQ_Y       0.45f

/*
 * Themes — every entry sits in the bright half of the 256-colour
 * cube so even A_DIM cells stay legible against a default-black
 * terminal.  See "Theme Palette Brightness" in /CLAUDE.md.
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
        init_pair(PAIR_HOT,  COLOR_RED,  -1);
        init_pair(PAIR_COLD, COLOR_CYAN, -1);
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
/* §5  wang — tile set, perlin/fbm, constraint solver, picker             */
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
 * Tile set + grid.                                                         *
 * ----------------------------------------------------------------------- */

typedef struct {
    uint8_t n, e, s, w;
} WangTile;

static WangTile tile_set[N_TILES];

static void tile_set_init(void)
{
    for (int i = 0; i < N_TILES; i++) {
        tile_set[i].n = (uint8_t)((i >> 3) & 1);
        tile_set[i].e = (uint8_t)((i >> 2) & 1);
        tile_set[i].s = (uint8_t)((i >> 1) & 1);
        tile_set[i].w = (uint8_t)((i >> 0) & 1);
    }
}

typedef struct {
    int     w, h;     /* grid dimensions in TILES (not cells) */
    uint8_t cells[MAX_GRID_CELLS];
} WangGrid;

/*
 * grid_pick — pick one valid tile index given the per-pattern bias.
 *
 *   - RANDOM   : uniform over `valid` (no bias score).
 *   - NOISE    : prefer tiles whose E and S edges match an fBm-biased
 *                colour, so neighbouring cells inherit the same
 *                preferred colour and large blobs form.
 *   - STRIPES  : prefer tiles whose S edge matches a sin-biased
 *                colour, so horizontal bands of S colour stack up.
 *   - SWIRL    : prefer tiles whose E and S match an angle-biased
 *                colour around the screen centre.
 *
 * The Wang constraint (already applied to `valid`) is never violated;
 * pattern only chooses among the candidates that are already legal.
 */
static int grid_pick(int tx, int ty, int seed, Pattern p,
                     const uint8_t *valid, int n_valid,
                     int grid_w, int grid_h)
{
    if (n_valid <= 0) return 0;

    int prefer_e = -1, prefer_s = -1;

    switch (p) {
    case PATTERN_RANDOM:
        return valid[hash3(tx, ty, seed) % (uint32_t)n_valid];
    case PATTERN_NOISE: {
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        float v  = fbm2((float)tx * NOISE_SCALE_X + ox,
                        (float)ty * NOISE_SCALE_Y + oy);
        int prefer = (v > 0.5f) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case PATTERN_STRIPES: {
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float v     = 0.5f + 0.5f * sinf((float)ty * STRIPES_FREQ_Y + phase);
        prefer_s = (v > 0.5f) ? 1 : 0;
        break;
    }
    case PATTERN_SWIRL: {
        float cx = (float)grid_w * 0.5f;
        float cy = (float)grid_h * 0.5f;
        float dx = (float)tx - cx;
        float dy = ((float)ty - cy) * ASPECT_Y_F;
        float a  = atan2f(dy, dx);
        int prefer = (a > 0) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case N_PATTERNS: break;
    }

    /* Score and pick the highest-scoring candidate(s) with hash tiebreak. */
    uint8_t best[N_TILES];
    int best_score = -1;
    int n_best     =  0;
    for (int i = 0; i < n_valid; i++) {
        const WangTile *t = &tile_set[valid[i]];
        int score = 0;
        if (prefer_e >= 0 && (int)t->e == prefer_e) score++;
        if (prefer_s >= 0 && (int)t->s == prefer_s) score++;
        if (score > best_score) {
            best_score = score;
            best[0]    = valid[i];
            n_best     = 1;
        } else if (score == best_score) {
            best[n_best++] = valid[i];
        }
    }
    return best[hash3(tx, ty, seed ^ 0x9E3779B9) % (uint32_t)n_best];
}

/*
 * grid_generate — fill the grid cell-by-cell, respecting the Wang
 * constraint between adjacent tiles. Iterate row-major; left + above
 * neighbour determine the W and N edge constraints respectively.
 * The first column drops the W constraint; the first row drops N.
 *
 * Time complexity: O(grid_w · grid_h · N_TILES) where N_TILES = 16 —
 * trivial. Run on reseed and on pattern change.
 */
static void grid_generate(WangGrid *g, int seed, Pattern p,
                          int grid_w, int grid_h)
{
    if (grid_w > MAX_GRID_W) grid_w = MAX_GRID_W;
    if (grid_h > MAX_GRID_H) grid_h = MAX_GRID_H;
    g->w = grid_w;
    g->h = grid_h;

    for (int ty = 0; ty < grid_h; ty++) {
        for (int tx = 0; tx < grid_w; tx++) {
            int has_w = (tx > 0);
            int has_n = (ty > 0);
            int expected_w = has_w
                           ? tile_set[g->cells[ty * grid_w + (tx - 1)]].e
                           : -1;
            int expected_n = has_n
                           ? tile_set[g->cells[(ty - 1) * grid_w + tx]].s
                           : -1;

            uint8_t valid[N_TILES];
            int     n_valid = 0;
            for (int i = 0; i < N_TILES; i++) {
                if (expected_w >= 0 && (int)tile_set[i].w != expected_w) continue;
                if (expected_n >= 0 && (int)tile_set[i].n != expected_n) continue;
                valid[n_valid++] = (uint8_t)i;
            }

            int chosen = grid_pick(tx, ty, seed, p, valid, n_valid,
                                   grid_w, grid_h);
            g->cells[ty * grid_w + tx] = (uint8_t)chosen;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Per-glyph tile-cell renderer.                                           *
 * ----------------------------------------------------------------------- */

/*
 * tile_cell_render — return (glyph, ramp_idx, *visible) for one cell
 * within a tile, given the active glyph set and the cell's local
 * (dx, dy) coordinates inside the tile [0..TILE_W) × [0..TILE_H).
 *
 * For invisible cells (e.g., interior of EDGES set), `visible` is
 * set to false; the caller skips drawing.
 */
static int tile_cell_render(const WangTile *t, int dx, int dy,
                            GlyphSet g, char *out_glyph, bool *visible)
{
    *visible = true;
    int ramp_idx = 0;
    char glyph = ' ';

    bool is_top    = (dy == 0);
    bool is_bot    = (dy == TILE_H - 1);
    bool is_left   = (dx == 0);
    bool is_right  = (dx == TILE_W - 1);
    bool is_mid_y  = (dy == TILE_H / 2);
    bool is_mid_x  = (dx == TILE_W / 2);

    if (g == GLYPH_EDGES) {
        if (is_top) {
            glyph = '-'; ramp_idx = EDGE_RAMP[t->n];
        } else if (is_bot) {
            glyph = '_'; ramp_idx = EDGE_RAMP[t->s];
        } else if (is_left) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->w];
        } else if (is_right) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->e];
        } else {
            *visible = false;
        }
    }
    else if (g == GLYPH_BLOCKS) {
        if (is_top || is_bot || is_left || is_right) {
            glyph = '#';
            ramp_idx = is_top  ? EDGE_RAMP[t->n]
                     : is_bot  ? EDGE_RAMP[t->s]
                     : is_left ? EDGE_RAMP[t->w]
                     :           EDGE_RAMP[t->e];
        } else {
            /* Interior — softly tinted by the parity of the four
             * edge colours so each tile has a recognisable shade. */
            int parity = (t->n ^ t->e ^ t->s ^ t->w) & 1;
            glyph = '.';
            ramp_idx = parity ? 5 : 4;
        }
    }
    else if (g == GLYPH_WIRES) {
        bool ns_match = (t->n == t->s);
        bool ew_match = (t->e == t->w);
        bool drew = false;

        /* Vertical through-line at the tile's middle column when N=S. */
        if (is_mid_x && ns_match) {
            glyph = '|';
            ramp_idx = EDGE_RAMP[t->n];
            drew = true;
        }
        /* Horizontal through-line at the middle row when E=W. */
        if (is_mid_y && ew_match) {
            /* Horizontal wins at the cross when both lines pass — '+'. */
            if (drew) {
                glyph = '+';
                /* Mix: take whichever line we'd label as primary. */
                ramp_idx = EDGE_RAMP[t->n];
            } else {
                glyph = '-';
                ramp_idx = EDGE_RAMP[t->w];
                drew = true;
            }
        }

        /* Edge stubs always rendered, dimmed when no through-line. */
        if (!drew) {
            if (is_top) {
                glyph = '-'; ramp_idx = EDGE_RAMP[t->n];
            } else if (is_bot) {
                glyph = '_'; ramp_idx = EDGE_RAMP[t->s];
            } else if (is_left) {
                glyph = '|'; ramp_idx = EDGE_RAMP[t->w];
            } else if (is_right) {
                glyph = '|'; ramp_idx = EDGE_RAMP[t->e];
            } else {
                *visible = false;
            }
        }
    }

    *out_glyph = glyph;
    return ramp_idx;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    WangGrid grid;
    bool     paused;
    int      speed;
    int      current_theme;
    Pattern  current_pattern;
    GlyphSet current_glyph;
    int      seed;
    float    time_secs;
    float    wind_x;
    float    flash_t;
    int      grid_w_cap;     /* most recent screen-derived bounds */
    int      grid_h_cap;
} Scene;

/*
 * apply_perm — reshuffle the Perlin perm[] for the current
 * (seed, pattern) pair so brightness AND the NOISE-pattern bias both
 * change when the user cycles patterns. Same trick as truchet_tiles.c.
 */
static void apply_perm(const Scene *s)
{
    perm_shuffle(s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_regenerate(Scene *s)
{
    apply_perm(s);
    grid_generate(&s->grid, s->seed, s->current_pattern,
                  s->grid_w_cap, s->grid_h_cap);
    s->flash_t = 1.0f;
}

static void scene_reseed(Scene *s)
{
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f),
                         (int)(s->wind_x * 100.0f), 0xABCDEF);
    scene_regenerate(s);
}

static void scene_init(Scene *s, int grid_w, int grid_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_RANDOM;
    s->current_glyph   = GLYPH_EDGES;
    s->seed            = 0xDEADBEEF;
    s->grid_w_cap      = grid_w;
    s->grid_h_cap      = grid_h;
    tile_set_init();
    scene_regenerate(s);
}

static void scene_resize_to(Scene *s, int grid_w, int grid_h)
{
    s->grid_w_cap = grid_w;
    s->grid_h_cap = grid_h;
    scene_regenerate(s);
}

/*
 * scene_tick — advance brightness wind. Tile placement is static
 * until the user reseeds or changes pattern; only the brightness
 * field drifts.
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
    int gx0, gy0;        /* top-left of the tile grid in screen coords */
    int grid_w, grid_h;  /* grid dimensions in TILES                   */
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int gw = avail_w / TILE_W;
    int gh = avail_h / TILE_H;
    if (gw > MAX_GRID_W) gw = MAX_GRID_W;
    if (gh > MAX_GRID_H) gh = MAX_GRID_H;
    if (gw < 4) gw = 4;
    if (gh < 4) gh = 4;

    s->grid_w = gw;
    s->grid_h = gh;
    /* Centre the grid on screen so partial tiles don't leak at edges. */
    s->gx0 = (avail_w - gw * TILE_W) / 2;
    s->gy0 = top + (avail_h - gh * TILE_H) / 2;
    if (s->gx0 < 0)   s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
}

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
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/*
 * bright_at — slow drifting fBm spotlight value at a screen cell.
 * Returns [0, 1]. Used to modulate per-cell A_DIM / A_NORMAL / A_BOLD
 * independently of the tile's intrinsic colour.
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

static void scene_draw(const Screen *sc, const Scene *s)
{
    const WangGrid *g = &s->grid;
    int gx0 = sc->gx0, gy0 = sc->gy0;

    for (int ty = 0; ty < g->h; ty++) {
        const WangTile *row = NULL;
        (void)row;
        for (int tx = 0; tx < g->w; tx++) {
            const WangTile *t = &tile_set[g->cells[ty * g->w + tx]];
            for (int dy = 0; dy < TILE_H; dy++) {
                int sy = gy0 + ty * TILE_H + dy;
                if (sy < 2 || sy >= sc->rows - 1) continue;
                for (int dx = 0; dx < TILE_W; dx++) {
                    int sx = gx0 + tx * TILE_W + dx;
                    if (sx < 0 || sx >= sc->cols) continue;

                    char glyph;
                    bool visible;
                    int ramp_idx = tile_cell_render(t, dx, dy,
                                                    s->current_glyph,
                                                    &glyph, &visible);
                    if (!visible) continue;

                    float b = bright_at(sx, sy, s->wind_x);
                    int attr;
                    if      (b > 0.65f) attr = A_BOLD;
                    else if (b < 0.35f) attr = A_DIM;
                    else                attr = A_NORMAL;

                    int pair = PAIR_RAMP_BASE + ramp_idx;
                    attron(COLOR_PAIR(pair) | attr);
                    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                    attroff(COLOR_PAIR(pair) | attr);
                }
            }
        }
    }

    /* Reseed flash overlay. */
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
    mvprintw(0, 1, " WANG TILES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + glyph + theme + ramp swatch + grid size. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
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
             "  grid:%dx%d  tiles:%d ",
             s->grid.w, s->grid.h, s->grid.w * s->grid.h);
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
    scene_resize_to(&app->scene, app->screen.grid_w, app->screen.grid_h);
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
        scene_regenerate(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_regenerate(s);
        break;

    /* Glyph cycling — pure rendering change, no regen. */
    case 'g':
        s->current_glyph = (GlyphSet)(((int)s->current_glyph + 1) % N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)(((int)s->current_glyph + N_GLYPH_SETS - 1) % N_GLYPH_SETS);
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
    scene_init(&app->scene, app->screen.grid_w, app->screen.grid_h);

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
