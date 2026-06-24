/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze_of_maze.c — a maze where every room holds its own smaller maze.
 *
 * Two mazes drawn at once: a big "outer" maze whose cells are rooms, and
 * a separate small maze carved inside each room. Both are made the same
 * way (a random depth-first walk that knocks down walls), so the whole
 * thing looks self-similar — zoom into any room and there's more maze.
 * A slow drifting light wash shimmers over the otherwise still picture.
 *
 * Keys: q/ESC quit · space pause · r reseed · n/p pattern · g/G glyph
 *       · t/T theme · +/- drift speed · ]/[ tick rate.
 *
 * Sister files: ../generational/wfc_showcase.c (constraint-based layouts),
 *               ../patterns/wang_tiles.c (edge-colour grid tiling).
 * Maze method: Buck, "Mazes for Programmers" ch.5 (recursive backtracker).
 * Light wash: Perlin, "Improving Noise" + Ebert et al., "Texturing &
 *             Modeling" (fractal noise = sum of noise layers).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/maze_of_maze.c \
 *     -o maze_of_maze -lncurses -lm
 */

/* ── HOW IT WORKS ─────────────────────────────────────────────────────── *
 *
 * THE MAZE. Each maze starts as a grid of rooms with every wall standing.
 * To carve it, walk randomly: from the cell you're in, step to a random
 * neighbour you haven't visited, knocking down the wall between them; when
 * you're boxed in, back up and try elsewhere. Keep going until every cell
 * has been visited. The result is a "perfect" maze — every cell reachable,
 * exactly one path between any two, no loops. (This is the recursive
 * backtracker; dfs_carve below.)
 *
 * THE NESTING. Carve the big outer maze once. Then carve one small maze
 * inside each of its rooms, each from its own seed, so no two rooms look
 * alike. Draw the inner mazes first and the outer walls on top, so the
 * room boundaries always stay crisp.
 *
 * THE SHIMMER. The maze never moves once carved. To give it life, a slow
 * drifting cloud of fractal noise picks a brightness for each cell each
 * frame — dim, normal, or bold — so light seems to wash across the walls.
 *
 * THE PRESETS. A "pattern" picks how many rooms tile the screen and how
 * many cells fill each room. On a fixed screen those trade off: more rooms
 * means each room (and its inner maze) is smaller. CLASSIC is balanced,
 * WIDE has few big detailed rooms, DENSE has many small coarse ones, EVEN
 * splits the difference. A "glyph" set picks the wall characters.
 *
 * GOTCHAS WORTH KNOWING.
 *  • A wall between two cells is stored in BOTH cells, and both get drawn,
 *    so it's painted twice — harmless, same colour, lets each cell draw
 *    on its own without peeking at neighbours.
 *  • If a room gets too small, its inner cells round down below 2 chars
 *    and the inner maze would vanish; the size is floored so it stays.
 *  • Inner mazes are independent — their walls do NOT line up across room
 *    boundaries. That's on purpose; it's what makes the two scales read
 *    as separate.
 *
 * References:
 *   • Buck — "Mazes for Programmers" (2015), ch.5: recursive backtracker.
 *   • Cormen et al. — "Introduction to Algorithms" (4th ed.): a perfect
 *     maze is a spanning tree of the grid; DFS carving is a random walk
 *     of one.
 *   • Perlin — "Improving Noise" (SIGGRAPH 2002): the gradient noise in
 *     perlin2d/grad2.
 *   • Ebert et al. — "Texturing & Modeling" (3rd ed.): fractal noise as
 *     a sum of noise layers (fbm2).
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
/* §1  config       — immutable data tables & constants (no functions) */
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

    /* Biggest maze we'll ever hold: 16 x 8 cells. Roomy; presets use less. */
    MAX_MAZE_W          =  16,
    MAX_MAZE_H          =   8,
    MAX_MAZE_CELLS      = MAX_MAZE_W * MAX_MAZE_H,

    /* How many inner mazes we keep room for — one per outer cell, so as
     * many cells as the biggest outer maze can have. */
    MAX_OUTER_CELLS     = MAX_MAZE_W * MAX_MAZE_H,

    /* Colour-pair slot numbers. HUD/HINT slots are the project-wide reserved
     * ones (see CLAUDE.md). */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* slots 3..10 = the 8 colour-ramp tints */
    PAIR_FLASH          =  11,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/* Knobs for the drifting light wash: how stretched the noise cloud is in
 * each direction, how fast it blows sideways, how many noise layers stack. */
#define BRIGHT_SCALE_X       0.040f
#define BRIGHT_SCALE_Y       0.080f
#define WIND_X_BASE          3.0f
#define FBM_OCTAVES          3

/* Which bit in a cell's wall byte means which wall. */
#define WALL_N               0x1u
#define WALL_E               0x2u
#define WALL_S               0x4u
#define WALL_W               0x8u

/* Which rung of the colour ramp the outer and inner walls draw from. Both
 * sit in the bright half so the inner walls stay readable even when the
 * light wash is at its dimmest. Outer sits higher so it stays dominant.
 * (Inner walls are also pinned to at least normal brightness — see no_dim.) */
#define OUTER_RAMP           7
#define INNER_RAMP           5

/* Layout sizes, all counted in terminal character cells. */
enum {
    HUD_TOP_ROWS   = 2,   /* top two rows belong to the HUD; the maze starts below */
    MIN_OUTER_CW   = 6,   /* smallest a room may shrink to, across                  */
    MIN_OUTER_CH   = 4,   /* smallest a room may shrink to, down                    */
    MIN_INNER_CELL = 2,   /* smallest an inner cell may shrink to; below 2 it vanishes */
};

/* Where the light wash flips a wall to bold or dim, on its 0..1 reading. */
#define BRIGHT_HI            0.65f   /* brighter than this -> bold               */
#define BRIGHT_LO            0.35f   /* dimmer than this  -> dim (unless no_dim) */

/* The brief sparkle that flashes when you reseed. */
#define FLASH_VISIBLE_MIN    0.05f   /* once the flash has faded past this, stop drawing it */
#define FLASH_PHASE_RATE  1000.0f    /* how fast the sparkle pattern shifts, per second      */
#define FLASH_GLYPH          '*'
enum { FLASH_SPARSITY_MASK = 7 };    /* keeps roughly 1 cell in 8 sparkling                  */

/* Maze carving. */
#define ALL_WALLS            0xFu        /* all four walls up — how a cell starts before carving */
#define KNUTH_HASH32  2654435761u        /* a good odd multiplier for scrambling a seed          */
#define INNER_SEED_SALT      0xBEEF      /* stirred into each room's seed so rooms differ        */

/* The little random-number generator used to shuffle carving directions.
 * Each step is next = state*LCG_MUL + LCG_ADD (classic Numerical Recipes). */
#define LCG_MUL       1664525u
#define LCG_ADD    1013904223u

/* Main-loop timing. */
enum {
    MAX_FRAME_MS = 100,   /* longest frame gap we'll trust, so a stall can't trigger a catch-up spiral */
    RENDER_FPS   =  60,   /* how often we redraw — separate from how often the sim ticks               */
};

/*
 * Pattern — a named preset for the two size choices.
 *
 * There are two dials: how many rooms tile the screen, and how many cells
 * fill each room. They pull against each other — on a fixed screen, more
 * rooms means each room (and its inner maze) is smaller. These four presets
 * are handy spots along that range, so you flip between "a few big detailed
 * rooms" and "lots of small coarse ones" with one key instead of fiddling
 * with four numbers. The value is an index into PATTERN_CFG[].
 */
typedef enum {
    PATTERN_CLASSIC = 0,   /* 6x3 rooms / 6x4 inner — balanced default         */
    PATTERN_WIDE    = 1,   /* 4x2 rooms / 10x6 inner — most detail per room    */
    PATTERN_DENSE   = 2,   /* 8x4 rooms / 4x3 inner — most rooms, sparse inner */
    PATTERN_EVEN    = 3,   /* 5x3 rooms / 8x5 inner — even split of both       */
    N_PATTERNS      = 4,   /* how many there are; also the wrap-around for n/p  */
} Pattern;

/*
 * PatternCfg — the actual cell counts a Pattern stands for (one row of
 * PATTERN_CFG[]). All four numbers are counted in MAZE CELLS, not screen
 * characters: the on-screen size of a cell is worked out fresh each frame
 * from the live window size, so one preset reads right at any window size.
 * Each count is clamped down to the maze capacity before it's used.
 */
typedef struct {
    int outer_w, outer_h;   /* rooms across / down                            */
    int inner_w, inner_h;   /* cells across / down inside each room           */
} PatternCfg;

static const PatternCfg PATTERN_CFG[N_PATTERNS] = {
    /*               outer  inner */
    [PATTERN_CLASSIC] = { 6, 3,   6, 4 },
    [PATTERN_WIDE   ] = { 4, 2,  10, 6 },
    [PATTERN_DENSE  ] = { 8, 4,   4, 3 },
    [PATTERN_EVEN   ] = { 5, 3,   8, 5 },
};


/*
 * GlyphSet — which characters to draw the walls with. This is looks only:
 * the carved maze is exactly the same in all three; the glyph choice just
 * changes how it's painted. MIXED draws the outer walls thick and the inner
 * walls thin on purpose, to make the two scales stand apart. All ASCII (no
 * fancy box-drawing) so it looks the same on every terminal.
 */
typedef enum {
    GLYPH_LINES  = 0,   /* '-' '|' '+' at both levels — clean line art       */
    GLYPH_BLOCKS = 1,   /* '#' everywhere — chunky / solid                   */
    GLYPH_MIXED  = 2,   /* outer '#', inner '-|+' — strongest scale contrast */
    N_GLYPH_SETS = 3,   /* how many there are; the wrap-around for g/G        */
} GlyphSet;

/*
 * WallStyle — the three characters one drawing pass uses for one level
 * (outer or inner). A GlyphSet is boiled down to this once, so the drawing
 * code never has to branch on the mode — it just stamps these three. Three
 * is all a grid needs: a run along each axis, plus the dot at the crossings.
 */
typedef struct {
    char h_wall;     /* the horizontal run (a cell's top / bottom edge)      */
    char v_wall;     /* the vertical run (a cell's left / right edge)        */
    char corner;     /* the dot at every grid crossing (drawn last, on top)  */
} WallStyle;

static const WallStyle STYLE_LINES  = { '-', '|', '+' };
static const WallStyle STYLE_BLOCKS = { '#', '#', '#' };

/*
 * Theme — one named colour palette. The drawing code never picks a raw
 * colour; it goes through these slots, so re-tinting the whole demo is just
 * swapping one theme in (t/T reloads the colour slots).
 *
 * Every colour here is kept in the bright half of the 256-colour space on
 * purpose: a dim wall on a near-black colour just disappears on a black
 * terminal. What gives a theme its look is the step from dark to light, not
 * how dark the darkest one is. See "Theme Palette Brightness" / COLOR.md.
 */
typedef struct {
    const char *name;     /* the label shown on the HUD while cycling with t/T */
    short       ramp[8];  /* eight colours, dark ramp[0] up to bright ramp[7]. */
                          /*   Walls take fixed rungs (INNER_RAMP/OUTER_RAMP); */
                          /*   the light wash only nudges them dim/bold on top.*/
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7 */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* ===================================================================== */
/* §2  logic        — pure decisions: no mutation, no I/O; readable alone */
/* ===================================================================== */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_CLASSIC: return "CLASSIC";
    case PATTERN_WIDE:    return "WIDE   ";
    case PATTERN_DENSE:   return "DENSE  ";
    case PATTERN_EVEN:    return "EVEN   ";
    default:              return "?      ";
    }
}

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_LINES:  return "lines";
    case GLYPH_BLOCKS: return "block";
    case GLYPH_MIXED:  return "mixed";
    default:           return "?    ";
    }
}

static WallStyle outer_style_for(GlyphSet g)
{
    switch (g) {
    case GLYPH_LINES:  return STYLE_LINES;
    case GLYPH_BLOCKS: return STYLE_BLOCKS;
    case GLYPH_MIXED:  return STYLE_BLOCKS;
    default:           return STYLE_LINES;
    }
}

static WallStyle inner_style_for(GlyphSet g)
{
    switch (g) {
    case GLYPH_LINES:  return STYLE_LINES;
    case GLYPH_BLOCKS: return STYLE_BLOCKS;
    case GLYPH_MIXED:  return STYLE_LINES;
    default:           return STYLE_LINES;
    }
}

/* Scrambles three numbers into one well-mixed number — same hash used by
 * the other showcases, here to turn a room's (x,y) into its own seed. */
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

/* The next few helpers are Perlin noise — the smooth, natural-looking
 * randomness behind the drifting light wash. Standard implementation,
 * copied in so this file stands alone. perm[] is its scrambled lookup
 * table, filled by perm_shuffle() in §5. */
static uint8_t perm[512];

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
    return (total / max_amp) * 0.5f + 0.5f;     /* rescale to land in 0..1 */
}

/*
 * How bright the light wash is at one screen cell, from 0 (dark) to 1
 * (bright). Stretches the cell's position to suit the noise and slides it
 * by the wind so the wash appears to drift. The drawing code turns this
 * number into dim / normal / bold.
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

/* Turn a 0..1 brightness into dim / normal / bold. no_dim lifts the floor to
 * normal — used for inner walls so they never disappear into the dark. */
static inline int brightness_attr(float b, bool no_dim)
{
    if (b > BRIGHT_HI)            return A_BOLD;
    if (b < BRIGHT_LO && !no_dim) return A_DIM;
    return A_NORMAL;
}

/* Is the reseed sparkle still bright enough to bother drawing? */
static inline bool flash_active(float flash_t) { return flash_t > FLASH_VISIBLE_MIN; }

/* The seed for the inner maze of room (ox,oy): the base seed mixed with the
 * room's position, so one base seed rebuilds the whole nest exactly yet no
 * two rooms come out the same. */
static inline int inner_seed(int base_seed, int ox, int oy)
{
    return base_seed ^ (int)hash3(ox, oy, INNER_SEED_SALT);
}

static inline int imin(int a, int b) { return a < b ? a : b; }

/* Step an option index forward / back, wrapping around — for the t/n/g keys. */
static inline int cycle_next(int i, int n) { return (i + 1) % n; }
static inline int cycle_prev(int i, int n) { return (i + n - 1) % n; }

/* Biggest cell size (in chars) so n_cells of them plus the closing border
 * fit inside span, never smaller than min_cell. */
static inline int fit_cell_size(int span, int n_cells, int min_cell)
{
    int size = (span - 1) / n_cells;     /* the -1 leaves room for the far border */
    return size < min_cell ? min_cell : size;
}

/* How wide a grid of n_cells ends up on screen: the cells plus the one
 * closing border line. */
static inline int grid_span(int cell_size, int n_cells) { return cell_size * n_cells + 1; }

/* Where to place `content` so it sits centred inside `avail`, but never
 * before `origin` — an oversized grid clips at the start, not off-screen. */
static inline int center_offset(int avail, int content, int origin)
{
    int off = origin + (avail - content) / 2;
    return off < origin ? origin : off;
}

/* ===================================================================== */
/* §3  performance  — wall-clock read + sleep (frame pacing: §8 loop) */
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
/* §4  platform     — ncurses colour setup (init + theme key)         */
/* ===================================================================== */

/* Point the 8 ramp slots at a theme's colours. */
static void load_theme_pairs(const Theme *t)
{
    for (int i = 0; i < 8; i++)
        init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
}

/* A plain 8-colour ramp for terminals that don't have the full 256. */
static void load_fallback_pairs(void)
{
    static const short fb[8] = {
        COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
        COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
        init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) load_theme_pairs(&themes[idx]);
    else               load_fallback_pairs();
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
/* §5  generation   — builds static maze + noise basis; EVENT/INIT only */
/* ===================================================================== */

/* Fill the Perlin lookup table perm[] from a seed, so a given seed always
 * gives the same noise (and reseeding changes the light wash too). */
static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * KNUTH_HASH32;
    for (int i = 255; i > 0; i--) {
        st = st * LCG_MUL + LCG_ADD;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    /* Copy the 256 entries again into the top half so perlin2d can read an
     * index up to 511 without ever needing to wrap. */
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/* ── one maze + how it's carved ──────────────────────────────────────── */

/*
 * Maze — one "perfect" maze: a grid where every cell is reachable and there's
 * exactly one path between any two, with no loops. maze_generate() carves it
 * by random depth-first walk (the recursive backtracker): start with every
 * wall up, keep stepping to a random unvisited neighbour and knocking down
 * the wall between, and back up at dead ends until every cell is visited.
 *   refs: Buck, "Mazes for Programmers" ch.5; a perfect maze is a spanning
 *         tree of the grid (CLRS).
 *
 * How it's stored — one byte per cell, four bits saying which walls stand
 * (N=1 E=2 S=4 W=8); 0xF means all four up, the state before carving. A wall
 * between two cells belongs to both, and is stored in both — carving clears
 * the bit on each side. That doubling is on purpose: it lets the drawing code
 * read any cell on its own without peeking at its neighbours. Laid out
 * row by row: cell (x,y) lives at walls[y*w + x].
 */
typedef struct {
    int     w, h;                   /* size in cells, each within the max */
    uint8_t walls[MAX_MAZE_CELLS];  /* one wall-bit byte per cell, row by row */
} Maze;

/* The four directions (N, E, S, W). DX/DY is the step to take; OPP is the
 * opposite direction; WALL_BIT is which wall bit that direction means. */
static const int DX[4] = {  0,  1,  0, -1 };
static const int DY[4] = { -1,  0,  1,  0 };
static const int OPP[4] = { 2,  3,  0,  1 };
static const uint8_t WALL_BIT[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

/* One step of the carving random-number generator. Seeded from the maze's
 * seed, so the same seed always carves the same maze. */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = (*st) * LCG_MUL + LCG_ADD;
    return *st;
}

static void shuffle4(int *a, uint32_t *st)
{
    for (int i = 3; i > 0; i--) {
        int j = (int)(lcg_next(st) >> 16) % (i + 1);
        int t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

static inline int cell_idx(const Maze *m, int x, int y) { return y * m->w + x; }

static inline bool in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

/* Knock down the wall between cell (x,y) and its neighbour in direction d.
 * The wall lives in both cells, so clear it on both sides. */
static inline void remove_wall_between(Maze *m, int x, int y, int d)
{
    int nx = x + DX[d], ny = y + DY[d];
    m->walls[cell_idx(m, x,  y )] &= (uint8_t)~WALL_BIT[d];
    m->walls[cell_idx(m, nx, ny)] &= (uint8_t)~WALL_BIT[OPP[d]];
}

/*
 * The carving walk itself. Mark this cell visited, try the four directions
 * in random order, and for each neighbour that's on the grid and not yet
 * visited, open the wall to it and walk on into it. When none are left, this
 * call returns — which is the "back up" step.
 *
 * `visited` is the caller's array (one bool per cell); `st` is the random
 * generator's state, passed down so every step in the walk shares it.
 */
static void dfs_carve(Maze *m, int x, int y, bool *visited, uint32_t *st)
{
    visited[cell_idx(m, x, y)] = true;
    int order[4] = { 0, 1, 2, 3 };
    shuffle4(order, st);
    for (int i = 0; i < 4; i++) {
        int d  = order[i];
        int nx = x + DX[d];
        int ny = y + DY[d];
        if (!in_bounds(m, nx, ny))           continue;
        if (visited[cell_idx(m, nx, ny)])    continue;
        remove_wall_between(m, x, y, d);
        dfs_carve(m, nx, ny, visited, st);
    }
}

/* Turn a maze seed into a starting state for the carving generator, forced
 * to be non-zero — this generator gets stuck on all-zeros if you ever feed
 * it zero. */
static inline uint32_t lcg_seed_from(int seed)
{
    uint32_t st = (uint32_t)seed * KNUTH_HASH32 + 1u;
    return st ? st : 1u;
}

static void maze_generate(Maze *m, int seed)
{
    int total = m->w * m->h;
    if (total > MAX_MAZE_CELLS) return;     /* can't happen, but stay safe */

    /* start over: every wall up, nothing visited yet */
    bool visited[MAX_MAZE_CELLS];
    for (int i = 0; i < total; i++) {
        m->walls[i] = ALL_WALLS;
        visited[i]  = false;
    }

    uint32_t st = lcg_seed_from(seed);
    dfs_carve(m, 0, 0, visited, &st);
}

/* ── the nested maze: a maze inside every room ───────────────────────── */

/*
 * MazeOfMaze — the whole nested thing: one big outer maze, plus a separate
 * small maze carved inside each of its rooms. Two perfect mazes at two scales
 * on screen at once, which is what gives the self-similar, fractal look.
 * mom_generate() seeds each room's inner maze from the room's position, so
 * the entire nest rebuilds exactly from one seed yet no two rooms match.
 * The two levels are kept independent on purpose — inner walls don't line up
 * across room edges — which is what lets the eye see two distinct scales.
 */
typedef struct {
    Maze outer;                    /* the big maze; its cells are the rooms    */
    Maze inner[MAX_OUTER_CELLS];   /* one small maze per room, room by room    */
    int  outer_count;              /* how many rooms are actually live, so the */
                                   /*   draw loop never touches leftover slots */
} MazeOfMaze;

static void mom_generate(MazeOfMaze *mom, const PatternCfg *cfg, int seed)
{
    int ow = imin(cfg->outer_w, MAX_MAZE_W);   /* clamp the request to capacity */
    int oh = imin(cfg->outer_h, MAX_MAZE_H);
    int iw = imin(cfg->inner_w, MAX_MAZE_W);
    int ih = imin(cfg->inner_h, MAX_MAZE_H);

    /* 1. carve the big outer maze — its cells become the rooms */
    mom->outer.w = ow;
    mom->outer.h = oh;
    maze_generate(&mom->outer, seed);

    /* 2. carve one small maze inside each room, seeded from the room's spot
     *    so the nest still rebuilds from `seed` but every room comes out different */
    mom->outer_count = ow * oh;
    for (int oy = 0; oy < oh; oy++) {
        for (int ox = 0; ox < ow; ox++) {
            Maze *room = &mom->inner[oy * ow + ox];
            room->w = iw;
            room->h = ih;
            maze_generate(room, inner_seed(seed, ox, oy));
        }
    }
}

/*
 * Brightness — the drifting light wash that plays over the still maze, giving
 * it life without moving a single wall. bright_at() reads a 0..1 value from
 * this state for each screen cell, and the drawing turns that into dim /
 * normal / bold. The light cloud itself is never stored — only the few
 * numbers below that say where it has drifted to.
 *   refs: Perlin, "Improving Noise"; Ebert et al., "Texturing & Modeling".
 */
typedef struct {
    float time_secs;   /* seconds elapsed; also feeds the reseed and the      */
                       /*   sparkle's shifting pattern                        */
    float wind_x;      /* how far the wash has blown sideways — the one thing */
                       /*   that makes the still maze shimmer                 */
    float flash_t;     /* the reseed sparkle's fade: 1.0 right after a reseed,*/
                       /*   then decays toward 0; gates the '*' overlay       */
    int   speed;       /* how fast the wash drifts (the +/- keys halve/double)*/
} Brightness;

/*
 * Scene — everything the demo is showing right now: the maze itself, the few
 * choices that pick and paint it, and the light drifting over it. Grouped by
 * idea, not by which key changes them. One thing to remember: changing the
 * seed or the pattern means re-carving the mazes (scene_regenerate), while
 * the glyph and theme are paint-only and need no rebuild.
 */
typedef struct {
    MazeOfMaze mom;             /* the nested maze itself (still once carved)   */
    Pattern    current_pattern; /* which size preset is in use                  */
    int        seed;            /* which random maze; reseeding picks a new one */
    GlyphSet   current_glyph;   /* which wall characters to draw with           */
    int        current_theme;   /* which colour palette (index into themes[])   */
    Brightness bright;          /* the drifting light wash + reseed sparkle     */
    bool       paused;          /* when true, the drift is frozen               */
} Scene;

/* Re-roll the Perlin table for this (seed, pattern), so each pattern gets its
 * own light wash. Same trick as truchet_tiles.c / wang_tiles.c. */
static void apply_perm(int seed, Pattern pattern)
{
    perm_shuffle(seed ^ ((int)pattern * 0xA5A5A5));
}

static void scene_regenerate(Scene *s)
{
    apply_perm(s->seed, s->current_pattern);
    mom_generate(&s->mom, &PATTERN_CFG[s->current_pattern], s->seed);
    s->bright.flash_t = 1.0f;
}

static void scene_reseed(Scene *s)
{
    s->seed = (int)hash3((int)(s->bright.time_secs * 1000.0f),
                         (int)(s->bright.wind_x * 100.0f), 0xC0FFEE);
    scene_regenerate(s);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->bright.speed    = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_CLASSIC;
    s->current_glyph   = GLYPH_LINES;
    s->seed            = 0xCAFE;
    scene_regenerate(s);
}

/* ===================================================================== */
/* §6  effects      — cosmetic per-tick state advance (only simulation) */
/* ===================================================================== */

/* Move the light wash on by one tick (and fade any reseed sparkle). The maze
 * itself doesn't change here — only when the user reseeds or switches pattern. */
static void scene_tick(Scene *s, float dt)
{
    s->bright.time_secs += dt;
    s->bright.flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->bright.speed / (float)SPEED_DEF;
    s->bright.wind_x += WIND_X_BASE * speed_mul * dt;
}

/* ===================================================================== */
/* §7  render       — state -> screen; reads only, never mutates Scene */
/* ===================================================================== */

/*
 * Screen — the terminal window, measured in characters. Re-read at start and
 * on every resize, so all the drawing checks against the current size and a
 * resize can't garble the display. Handed to draw functions read-only;
 * scene_draw() is the one place that turns this size into per-cell sizes.
 */
typedef struct {
    int cols, rows;   /* window width and height in characters (cols=x, rows=y) */
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
 * Draw one wall character at (sx, sy), brightened or dimmed by the light
 * wash, skipping anything outside the maze area (top rows and bottom row are
 * the HUD's). no_dim is passed straight through to keep inner walls readable
 * even in the wash's dark patches.
 */
static inline void draw_wall_cell(const Screen *sc, const Brightness *br,
                                  int sx, int sy, char glyph,
                                  int pair, int extra_attr, bool no_dim)
{
    if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) return;
    if (sx < 0 || sx >= sc->cols)                return;
    int attr = brightness_attr(bright_at(sx, sy, br->wind_x), no_dim) | extra_attr;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * Draw the standing walls of one cell, whose top-left is at (sx, sy) and which
 * is cw by ch characters. The very ends of each run are left out — the corner
 * dots are stamped afterwards so they always sit on top.
 */
static void draw_cell_walls(const Screen *sc, const Brightness *br,
                            int sx, int sy, int cw, int ch, uint8_t walls,
                            WallStyle style, int pair, int extra_attr, bool no_dim)
{
    if (walls & WALL_N)
        for (int dx = 1; dx < cw; dx++)
            draw_wall_cell(sc, br, sx + dx, sy, style.h_wall, pair, extra_attr, no_dim);
    if (walls & WALL_E)
        for (int dy = 1; dy < ch; dy++)
            draw_wall_cell(sc, br, sx + cw, sy + dy, style.v_wall, pair, extra_attr, no_dim);
    if (walls & WALL_S)
        for (int dx = 1; dx < cw; dx++)
            draw_wall_cell(sc, br, sx + dx, sy + ch, style.h_wall, pair, extra_attr, no_dim);
    if (walls & WALL_W)
        for (int dy = 1; dy < ch; dy++)
            draw_wall_cell(sc, br, sx, sy + dy, style.v_wall, pair, extra_attr, no_dim);
}

/* Stamp the corner dot at every grid crossing, after the walls, so a wall
 * run never paints over a corner. */
static void draw_grid_corners(const Screen *sc, const Brightness *br,
                              const Maze *m, int gx, int gy, int cw, int ch,
                              WallStyle style, int pair, int extra_attr, bool no_dim)
{
    for (int my = 0; my <= m->h; my++)
        for (int mx = 0; mx <= m->w; mx++)
            draw_wall_cell(sc, br, gx + mx * cw, gy + my * ch,
                           style.corner, pair, extra_attr, no_dim);
}

/* Draw one whole maze at (gx, gy) with cells cw by ch: walls first, then all
 * the corner dots on top. (A shared wall gets drawn by both its cells, which
 * is harmless — same character, same colour.) */
static void render_maze(const Screen *sc, const Brightness *br,
                        const Maze *m, int gx, int gy,
                        int cw, int ch,
                        WallStyle style, int pair, int extra_attr,
                        bool no_dim)
{
    for (int my = 0; my < m->h; my++)
        for (int mx = 0; mx < m->w; mx++)
            draw_cell_walls(sc, br, gx + mx * cw, gy + my * ch, cw, ch,
                            m->walls[my * m->w + mx],
                            style, pair, extra_attr, no_dim);

    draw_grid_corners(sc, br, m, gx, gy, cw, ch, style, pair, extra_attr, no_dim);
}

/*
 * MazeLayout — where everything lands on screen this frame, all in characters.
 * layout_fit() works it out once per frame from the live window size and the
 * pattern, so scene_draw and its helpers all place things the same way.
 */
typedef struct {
    int gx, gy;                         /* top-left corner of the whole grid  */
    int cw_outer, ch_outer;             /* size of one room, in chars         */
    int cw_inner, ch_inner;             /* size of one inner cell, in chars   */
    int inner_margin_x, inner_margin_y; /* how far an inner maze is inset in its room */
} MazeLayout;

/* Work out the sizes and centring for the current window. Rooms fill the
 * space below the HUD; each inner maze fills the room minus its 1-char wall
 * border, and both are centred so nothing jams against a corner. */
static MazeLayout layout_fit(const Screen *sc, const PatternCfg *cfg)
{
    int avail_w = sc->cols;
    int avail_h = sc->rows - 1 - HUD_TOP_ROWS;   /* the room between HUD top and hint row */

    MazeLayout L;
    L.cw_outer = fit_cell_size(avail_w, cfg->outer_w, MIN_OUTER_CW);
    L.ch_outer = fit_cell_size(avail_h, cfg->outer_h, MIN_OUTER_CH);
    L.gx = center_offset(avail_w, grid_span(L.cw_outer, cfg->outer_w), 0);
    L.gy = center_offset(avail_h, grid_span(L.ch_outer, cfg->outer_h), HUD_TOP_ROWS);

    int interior_w = L.cw_outer - 1;             /* the room minus its outer */
    int interior_h = L.ch_outer - 1;             /*   wall border            */
    L.cw_inner = fit_cell_size(interior_w, cfg->inner_w, MIN_INNER_CELL);
    L.ch_inner = fit_cell_size(interior_h, cfg->inner_h, MIN_INNER_CELL);
    L.inner_margin_x = center_offset(interior_w, grid_span(L.cw_inner, cfg->inner_w), 0);
    L.inner_margin_y = center_offset(interior_h, grid_span(L.ch_inner, cfg->inner_h), 0);
    return L;
}

/* Draw every room's inner maze, tucked inside the room. Done FIRST so the
 * outer walls drawn next can paint over any inner cell that reaches a room
 * edge. The last argument keeps inner walls from ever going dim, so they stay
 * readable in the dark patches of the light wash. */
static void draw_inner_mazes(const Screen *sc, const Scene *s,
                             const PatternCfg *cfg, const MazeLayout *L)
{
    WallStyle style = inner_style_for(s->current_glyph);
    int pair = PAIR_RAMP_BASE + INNER_RAMP;
    for (int oy = 0; oy < cfg->outer_h; oy++) {
        for (int ox = 0; ox < cfg->outer_w; ox++) {
            int idx = oy * cfg->outer_w + ox;
            if (idx >= s->mom.outer_count) break;
            int igx = L->gx + ox * L->cw_outer + 1 + L->inner_margin_x;
            int igy = L->gy + oy * L->ch_outer + 1 + L->inner_margin_y;
            render_maze(sc, &s->bright, &s->mom.inner[idx], igx, igy,
                        L->cw_inner, L->ch_inner, style, pair, A_NORMAL, true);
        }
    }
}

/* Draw the big outer maze over the inner ones, in bold and with the full
 * dim/bold range, so the large scale clearly dominates. */
static void draw_outer_maze(const Screen *sc, const Scene *s, const MazeLayout *L)
{
    WallStyle style = outer_style_for(s->current_glyph);
    int pair = PAIR_RAMP_BASE + OUTER_RAMP;
    render_maze(sc, &s->bright, &s->mom.outer, L->gx, L->gy,
                L->cw_outer, L->ch_outer, style, pair, A_BOLD, false);
}

/* The brief sparkle that washes over the screen right after a reseed. Lights
 * up about 1 cell in 8, in a pattern that shifts over time so it twinkles. */
static void draw_reseed_flash(const Screen *sc, const Scene *s)
{
    int phase = (int)(s->bright.time_secs * FLASH_PHASE_RATE);
    attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    for (int sy = HUD_TOP_ROWS; sy < sc->rows - 1; sy += 2)
        for (int sx = 0; sx < sc->cols; sx += 2)
            if (((sx ^ sy ^ phase) & FLASH_SPARSITY_MASK) == 0)
                mvaddch(sy, sx, FLASH_GLYPH);
    attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
}

/* Draw one frame: inner mazes first, outer walls over them, then the reseed
 * sparkle if one is still fading. */
static void scene_draw(const Screen *sc, const Scene *s)
{
    const PatternCfg *cfg = &PATTERN_CFG[s->current_pattern];
    MazeLayout layout = layout_fit(sc, cfg);

    draw_inner_mazes(sc, s, cfg, &layout);
    draw_outer_maze(sc, s, &layout);
    if (flash_active(s->bright.flash_t))
        draw_reseed_flash(sc, s);
}

/* Draw the theme's 8 colours as a little dark-to-bright strip in the HUD,
 * starting at column x; returns the column just after it. */
static int draw_ramp_swatch(int x)
{
    static const char ramp_glyphs[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };
    for (int i = 0; i < 8; i++) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ramp_glyphs[i]);
        attroff(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        x++;
    }
    return x;
}

/* Top HUD row: fps, tick rate, run state and drift speed on the right; the
 * title on the left. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->bright.speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " MAZE OF MAZE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second HUD row: pattern, glyph, theme, the colour strip, and the sizes.
 * x just walks along, one field's width at a time. */
static void draw_hud_params(const Scene *s)
{
    const PatternCfg *cfg = &PATTERN_CFG[s->current_pattern];
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

    x = draw_ramp_swatch(x);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  outer:%dx%d  inner:%dx%d ",
             cfg->outer_w, cfg->outer_h, cfg->inner_w, cfg->inner_h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom HUD row: the full list of keys. */
static void draw_hud_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:drift  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    draw_hud_status(sc, s, fps, sim_fps);
    draw_hud_params(s);
    draw_hud_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app          — events + the one tick-combine point + main loop */
/* ===================================================================== */

/*
 * App — everything the program runs on: the demo (Scene), the window (Screen),
 * and the few flags the main loop watches. There's one global (g_app) for the
 * single reason that signal handlers have no other way to reach it. The two
 * flags are set inside signal handlers and read in the loop, so they're marked
 * the special signal-safe way that guarantees a clean read without locks.
 * sim_fps is how often the demo ticks, kept separate from the ~60 fps redraw
 * (the main loop bridges the two).
 */
typedef struct {
    Scene                 scene;        /* the demo state                       */
    Screen                screen;       /* the terminal window                  */
    int                   sim_fps;      /* tick rate (]/[ keys), not the redraw */
    volatile sig_atomic_t running;      /* a quit signal clears it -> loop ends */
    volatile sig_atomic_t need_resize;  /* a resize signal sets it -> re-measure*/
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
        if (s->bright.speed < SPEED_MAX) s->bright.speed *= 2;
        if (s->bright.speed > SPEED_MAX) s->bright.speed  = SPEED_MAX;
        break;
    case '-':
        s->bright.speed /= 2;
        if (s->bright.speed < SPEED_MIN) s->bright.speed  = SPEED_MIN;
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
        s->current_theme = cycle_next(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = cycle_prev(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)cycle_next(s->current_pattern, N_PATTERNS);
        scene_regenerate(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)cycle_prev(s->current_pattern, N_PATTERNS);
        scene_regenerate(s);
        break;

    case 'g':
        s->current_glyph = (GlyphSet)cycle_next(s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)cycle_prev(s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/*
 * How long since the last frame, and move the clock to now. The gap is capped
 * so that if the program stalls (a debugger, a hung terminal), the sim isn't
 * handed one giant gap and forced into a catch-up spiral. On return,
 * *frame_time marks this frame's start.
 */
static int64_t frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;
    return dt;
}

/*
 * Sleep off whatever time is left in this frame's budget, to hold the redraw
 * rate steady. Doing the sleep BEFORE writing to the terminal keeps the pace
 * even no matter how long the write itself takes.
 */
static void frame_sleep(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(NS_PER_SEC / RENDER_FPS - elapsed);
}

/*
 * FpsCounter — a smoothed frames-per-second reading for the HUD. It counts
 * frames and time over a short window, then publishes a fresh number and
 * starts over — so the displayed rate holds steady instead of jumping around
 * every frame.
 */
typedef struct {
    int     count;     /* frames since the last published number   */
    int64_t accum;     /* nanoseconds since the last published one */
    double  value;     /* the last published frames-per-second     */
} FpsCounter;

/* Fold one frame into the counter, and publish a new rate once the window
 * fills up. */
static void fps_tick(FpsCounter *f, int64_t dt)
{
    f->count++;
    f->accum += dt;
    if (f->accum >= FPS_UPDATE_MS * NS_PER_MS) {
        f->value = (double)f->count / ((double)f->accum / (double)NS_PER_SEC);
        f->count = 0;
        f->accum = 0;
    }
}

/*
 * Spend the saved-up time in fixed ticks, so the demo runs at exactly its
 * tick rate no matter how fast or slow we're redrawing. `accum` carries any
 * leftover time over to the next frame.
 */
static void advance_sim(Scene *s, int64_t *accum, int64_t tick_ns, float dt_sec)
{
    while (*accum >= tick_ns) {
        scene_tick(s, dt_sec);
        *accum -= tick_ns;
    }
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

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;
    FpsCounter fps        = { 0, 0, 0.0 };

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t dt      = frame_dt(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        advance_sim(&app->scene, &sim_accum, tick_ns, dt_sec);

        fps_tick(&fps, dt);
        frame_sleep(frame_time, dt);

        screen_draw(&app->screen, &app->scene, fps.value, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}

