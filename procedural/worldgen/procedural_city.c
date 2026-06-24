/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_city.c
 *   Grows a fake city in the terminal. We start with one big block,
 *   keep slicing it in half with roads until the pieces are small
 *   enough to be a single lot, then fill each lot with buildings or a
 *   park. The whole thing is animated growing in, held a few seconds,
 *   then torn down and re-grown from a fresh random seed.
 *
 * The slicing is the formal idea called an "L-system" / binary space
 * partition; the building zoning is from the "shape grammar" idea. If
 * you want the theory:
 *   Prusinkiewicz & Lindenmayer (1990), The Algorithmic Beauty of Plants
 *     https://algorithmicbotany.org/papers/abop/abop.pdf
 *   Parish & Müller (2001), "Procedural Modeling of Cities", SIGGRAPH
 *   Bourke (1997), ASCII grey-scale ramp (the h/H/#/@ density glyphs)
 *     http://paulbourke.net/dataformats/asciiart/
 *
 * Sister files: ../generational/bsp_dungeon_showcase.c (same slicing,
 * dungeon rooms instead of lots) and ../worldgen/procedural_galaxy.c
 * (a "world from a function" using circles instead of rectangles).
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

/* §1  CONFIG -- constants, data tables, core state types */

enum {
    /* Biggest city we'll ever hold; a larger terminal just gets clipped. */
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CITY_CELLS_MAX      = MAP_W_MAX * MAP_H_MAX,

    /* How far the slicing can go, and how small a lot is allowed to get.
     * We usually stop short of MAX_DEPTH because a lot hits the minimum
     * size first. */
    MAX_DEPTH           =   9,
    MIN_LOT_W           =   4,
    MIN_LOT_H           =   3,

    /* How fast the city grows in: step-units added per tick at normal speed. */
    BUILD_RATE_DEFAULT  =  28,

    /* Show the finished city for ~6 seconds before tearing it down. */
    HOLD_TICKS_DEF      = 6 * 60,

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* ncurses colour-pair slots. PAIR_HUD/PAIR_HINT are reserved across
     * all demos in this project. The _BASE pairs each cover four slots. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_ROAD           =   3,
    PAIR_BUILDING_BASE  =   4,    /* +0..+3 = the four building tints */
    PAIR_PARK_BASE      =   8,    /* +0..+3 = the four park tints     */
    PAIR_CAR            =  13,

    /* Traffic: this many cars, each moving once every CAR_STEP_TICKS ticks
     * so they crawl like real traffic instead of teleporting. */
    N_CARS              =  28,
    CAR_STEP_TICKS      =   4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Terminal characters are about twice as tall as they are wide, so we
 * stretch every vertical measurement by this to keep blocks looking square. */
#define ASPECT_Y_F      2.0f

/* Each slicing level gets its own band of "appear times" so deeper roads
 * and lots show up after shallower ones during the grow-in animation. */
#define DEPTH_STEP      1000

/* How far a road can wander off-centre in the non-grid patterns: up to a
 * third of the way toward either edge. */
#define SPLIT_JITTER    0.33f

/* When deciding which way to slice: clearly-tall blocks slice across,
 * clearly-wide ones slice down, and anything in between flips a coin. */
#define SPLIT_ASPECT_HI 1.4f
#define SPLIT_ASPECT_LO 0.7f

/* PARKS pattern: roughly 1 lot in this many becomes a park... */
#define PARK_DENOM      4
/* ...but never this close to the centre, so downtown stays built up. */
#define PARK_MIN_RADIUS 0.30f

/* The first few slicing levels are the big main roads; draw them bold. */
#define MAJOR_ROAD_MAX_DEPTH 2
/* Twinkling windows: roughly 1 lit window in this many blinks each second. */
#define TWINKLE_1_IN    30
/* Spawning a car: give up after this many random tries (matters on tiny maps). */
#define CAR_SPAWN_TRIES 200

/* The glyphs a building is drawn from. A building is a little framed box,
 * not a solid blob, so it reads as a real structure:
 *
 *       _____
 *      |HH"H|     top
 *      |HHHH|     interior body, with the odd '"' lit window
 *      |____|     foundation row and '|' side walls
 *
 * All four arrays line up by the same index 0..3 = the building's "size tier"
 * (house, apartment, office, skyscraper). That tier is stored in
 * Cell.color_idx and also picks the colour, so one small number per cell
 * drives the whole look. building_glyph_at (§3) looks at neighbours to decide
 * wall vs base vs interior, then picks from these by tier.
 *
 *   BODY    the fill glyph: n / H / # / @, getting visually denser with size.
 *   WINDOW  the glyph for a lit interior cell. Slot 0 ('m') is never used —
 *           houses skip the frame entirely; it's only here to keep the four
 *           arrays index-aligned.
 *   BASE    the foundation row: '_', except the skyscraper sits on a heavier '='.
 *   WINDOW_FREQ  how often an interior cell is a lit window, as "1 in N".
 *           Smaller = more windows, so towers glitter and townhouses stay solid.
 *           Slot 0 is unused for the same reason WINDOW[0] is. */
static const char BUILDING_BODY  [4] = { 'n', 'H', '#', '@' };
static const char BUILDING_WINDOW[4] = { 'm', '"', '*', '*' };
static const char BUILDING_BASE  [4] = { 'n', '_', '_', '=' };
static const int  WINDOW_FREQ    [4] = {   2,   8,   4,   3 };

/* The one most recognisable glyph per park tier, used ONLY for the little HUD
 * legend swatch. The actual park cells are drawn by park_glyph (§3), which
 * mixes several glyphs per tier so a park reads as a textured patch of green
 * rather than a flat fill. */
static const char PARK_HUD_GLYPH[4] = { ',', '.', 'Y', 'T' };

/* The four city "styles". These aren't four different algorithms — they're the
 * same slicing with a couple of knobs turned, and each one builds on the one
 * before it. The chosen style is passed around as `p`.
 *   GRID      slices exactly down the middle every time -> a tidy Manhattan grid.
 *   ORGANIC   lets each slice wander off-centre -> uneven, old-European blocks;
 *             building colours are random.
 *   DISTRICTS like ORGANIC, but now the building type depends on where the lot
 *             is: skyscrapers downtown fading to houses at the edges.
 *   PARKS     like DISTRICTS, plus some of the outer lots become green parks. */
typedef enum {
    PATTERN_GRID      = 0,
    PATTERN_ORGANIC   = 1,
    PATTERN_DISTRICTS = 2,
    PATTERN_PARKS     = 3,
    N_PATTERNS        = 4,
} Pattern;

/* One named colour palette; the 't' key cycles through ten of them.
 * The building colours run from house to skyscraper, usually dim-to-bright so
 * the skyline reads as density at a glance; park colours are the green version
 * of the same idea. All are 256-colour codes on the terminal's own background. */
typedef struct {
    const char *name;       /* shown in the HUD                     */
    short       road;       /* the single asphalt colour            */
    short       building[4];/* colour per size tier (house..tower)  */
    short       park    [4];/* colour per park type                 */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name      road  building{0..3}            park{0..3}             */
    { "DEFAULT", 244, { 110, 109, 215, 226 }, {  28,  34, 118, 120 } },
    { "MATRIX",  236, {  22,  28,  34,  46 }, {  10,  16,  22,  28 } },
    { "NOVA",    240, {  53,  91, 165, 207 }, {  35,  41,  77, 113 } },
    { "MONO",    240, { 240, 244, 248, 252 }, { 234, 236, 238, 240 } },
    { "OCEAN",   239, {  17,  18,  31,  39 }, {  22,  29,  79, 116 } },
    { "FIRE",    240, {  88, 124, 208, 226 }, {  64, 100, 142, 178 } },
    { "EARTH",   239, {  58, 100, 137, 173 }, {  22,  28,  64, 100 } },
    { "FOREST",  240, {  58,  64, 100, 136 }, {  22,  28,  29,  65 } },
    { "DESERT",  239, {  94, 130, 173, 222 }, {  58,  64, 100, 142 } },
    { "ARCTIC",  240, {  19,  24, 110, 195 }, {  17,  18,  24,  31 } },
};

/* What a single map square is. EMPTY means "nothing here yet / outside the
 * city"; the other three are what the slicing ends up producing. */
typedef enum {
    CELL_EMPTY    = 0,
    CELL_ROAD     = 1,
    CELL_BUILDING = 2,
    CELL_PARK     = 3,
} CellType;

/* One map square — the smallest piece of the city, and the only place we store
 * anything. There's no separate list of roads or lots: every question about a
 * spot (road? what colour? has it appeared yet?) is answered by reading just
 * this one struct, which is why generation never needs to allocate memory.
 * Kept to 4 bytes so the whole 240x80 map stays small and cache-friendly while
 * the draw loop sweeps every square each frame.
 *
 *   type       road, building, park, or empty.
 *   color_idx  0..3 sub-shade into the theme; for buildings it's also the
 *              size tier (house..skyscraper).
 *   step       the "appear time". The cell is only drawn once City.build_step
 *              has reached this value. Each square is stamped with a time during
 *              slicing, so just ramping build_step up replays the whole build as
 *              a grow-in animation, with no need to remember past frames. */
typedef struct {
    uint8_t  type;       /* CellType                              */
    uint8_t  color_idx;  /* 0..3 shade / building size tier       */
    uint16_t step;       /* drawn once build_step reaches this    */
} Cell;

/* The whole city: a w*h grid of Cells (stored row by row) plus the bookkeeping
 * for the grow-in animation. This is the one big piece of state; only §4 ever
 * writes it. The grid is a fixed-size array, reused on every rebuild, so we
 * never allocate while running.
 *
 *   w, h            size in cells.
 *   map[]           the cells themselves.
 *   max_step        the largest appear-time any cell got — i.e. the finish line.
 *   build_step      how far the grow-in animation has reached. 0 = blank,
 *                   max_step = fully built.
 *   hold_countdown  ticks left to admire the finished city before rebuilding.
 *   seed            the random seed this layout grew from (also keeps the window
 *                   twinkle stable for a given city).
 *   pattern_built   which style produced THIS layout, so the drawing always
 *                   matches the shape on screen. */
typedef struct {
    int   w, h;
    Cell  map[CITY_CELLS_MAX];
    int   max_step;
    int   build_step;
    int   hold_countdown;
    int   seed;
    Pattern pattern_built;
} City;

/* One little car. Once the city is built, a handful of these wander the roads
 * just to make the place feel alive. They only read the roads, never change
 * them. Each car follows a simple local rule (see car_step): mostly drive
 * straight, occasionally turn at a junction, and if it gets boxed in, it
 * disappears and respawns somewhere else.
 *
 *   x, y    where it is on the grid.
 *   dx, dy  which way it's pointing — one of these is +/-1, the other 0.
 *   active  false means "I'm stuck, respawn me next tick". */
typedef struct {
    int  x, y;
    int  dx, dy;
    bool active;
} Car;

/* §2  PERFORMANCE -- the two clock helpers (the actual pacing lives in main) */

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

/* §3  LOGIC -- pure look-it-up functions: they only read, never change anything.
 * Two groups: the ones that pick a glyph/colour for drawing, and the ones that
 * make slicing decisions for generation. */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_GRID:      return "GRID     ";
    case PATTERN_ORGANIC:   return "ORGANIC  ";
    case PATTERN_DISTRICTS: return "DISTRICTS";
    case PATTERN_PARKS:     return "PARKS    ";
    default:                return "?        ";
    }
}

/* Turns three numbers into one well-scrambled "random-looking" number. Same
 * inputs always give the same output, so it's our source of stable randomness
 * for jittered roads, building colours, and twinkling windows. */
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

static inline int cidx(const City *c, int x, int y) { return y * c->w + x; }

/* Decides what kind of building goes on a lot. In GRID/ORGANIC it's just
 * random. In DISTRICTS/PARKS it depends on how far the lot is from the city
 * centre: towers downtown, houses at the edges. */
static uint8_t lot_color_idx(int lcx, int lcy, int cx, int cy,
                             int max_dist, uint32_t lot_hash, Pattern p)
{
    if (p == PATTERN_GRID || p == PATTERN_ORGANIC) {
        return (uint8_t)((lot_hash >> 8) & 3u);
    }

    /* How far from centre, as a fraction (0 at the middle, 1 at the corner),
     * stretching the vertical so distance feels even on screen. */
    float dx = (float)(lcx - cx);
    float dy = (float)(lcy - cy) * ASPECT_Y_F;
    float d  = sqrtf(dx * dx + dy * dy) / (float)max_dist;
    if      (d < 0.25f) return 3;        /* skyscraper */
    else if (d < 0.50f) return 2;        /* office     */
    else if (d < 0.75f) return 1;        /* apartment  */
    else                return 0;        /* house      */
}

static inline bool road_at(const City *c, int x, int y)
{
    if (x < 0 || x >= c->w || y < 0 || y >= c->h) return false;
    return c->map[cidx(c, x, y)].type == CELL_ROAD;
}

/* An arrow pointing the way the car is going. */
static inline char car_glyph(const Car *car)
{
    if (car->dx > 0) return '>';
    if (car->dx < 0) return '<';
    if (car->dy > 0) return 'v';
    if (car->dy < 0) return '^';
    return 'o';
}

/* Picks the road character ('-', '|' or '+') by looking at the four
 * neighbours: a '+' where vertical road meets horizontal. Only roads that have
 * already grown in count, so a fresh road stub looks like a straight line and
 * turns into a junction the moment the crossing road catches up. */
static char road_glyph_at(const City *c, int x, int y, int build_step)
{
    bool n_v = false, s_v = false, e_v = false, w_v = false;
    if (y > 0) {
        const Cell *cl = &c->map[cidx(c, x, y - 1)];
        n_v = (cl->type == CELL_ROAD && cl->step <= build_step);
    }
    if (y < c->h - 1) {
        const Cell *cl = &c->map[cidx(c, x, y + 1)];
        s_v = (cl->type == CELL_ROAD && cl->step <= build_step);
    }
    if (x > 0) {
        const Cell *cl = &c->map[cidx(c, x - 1, y)];
        w_v = (cl->type == CELL_ROAD && cl->step <= build_step);
    }
    if (x < c->w - 1) {
        const Cell *cl = &c->map[cidx(c, x + 1, y)];
        e_v = (cl->type == CELL_ROAD && cl->step <= build_step);
    }
    bool vert  = n_v || s_v;
    bool horiz = e_v || w_v;
    if (vert && horiz) return '+';
    if (vert)          return '|';
    if (horiz)         return '-';
    /* No road neighbours yet — happens only at the very first cell of a road. */
    return '+';
}

/* True if the neighbour at (x,y) exists, is the type we want, and has already
 * grown in. Building edges use this so the frame fills in correctly as the
 * building appears. */
static inline bool nb_visible(const City *c, int x, int y,
                              int build_step, CellType want)
{
    if (x < 0 || x >= c->w || y < 0 || y >= c->h) return false;
    const Cell *n = &c->map[cidx(c, x, y)];
    return n->type == want && n->step <= build_step;
}

/* Picks one of a few green glyphs at random for a park cell, so a park looks
 * like a speckled patch of grass and trees instead of a flat block. */
static char park_glyph(int ci, uint32_t h)
{
    int b = (h >> 4) & 7;
    switch (ci & 3) {
    case 0: return (b < 5) ? ',' : '.';
    case 1: return (b < 3) ? ',' : (b < 6) ? '.' : '\'';
    case 2: return (b < 4) ? ',' : (b < 6) ? 'Y' : '.';
    default:return (b < 4) ? 'Y' : (b < 6) ? 'T' : '*';
    }
}

/* Picks the character for one building cell by looking at where it sits in the
 * building: the bottom row is the foundation, the left/right edges are walls,
 * and the inside is body fill with the occasional lit window. Houses (the
 * smallest tier) skip the frame entirely and are just a speckle of tiny
 * dwellings. Sets *lit when this cell turned out to be a window, so the caller
 * can make it glow. */
static char building_glyph_at(const City *c, int x, int y,
                              int build_step, uint32_t h, bool *lit)
{
    *lit = false;
    int ci = c->map[cidx(c, x, y)].color_idx & 3;

    /* Houses skip the frame — just alternate two little glyphs. */
    if (ci == 0) {
        return ((x + y) & 1) ? 'n' : 'm';
    }

    bool top_v = nb_visible(c, x, y - 1, build_step, CELL_BUILDING);
    bool bot_v = nb_visible(c, x, y + 1, build_step, CELL_BUILDING);
    bool lft_v = nb_visible(c, x - 1, y, build_step, CELL_BUILDING);
    bool rgt_v = nb_visible(c, x + 1, y, build_step, CELL_BUILDING);

    bool bot_edge  = !bot_v;
    bool side_edge = !lft_v || !rgt_v;
    /* We deliberately don't special-case the top edge: letting body fill run
     * right up to the road above gives the building a clean rooftop line.
     * (top_v is computed just for symmetry, otherwise unused.) */
    (void)top_v;

    if (bot_edge)  return BUILDING_BASE[ci];
    if (side_edge) return '|';

    /* Inside: usually body fill, occasionally a lit window. */
    int freq = WINDOW_FREQ[ci];
    if (freq > 0 && (int)((h >> 8) % (uint32_t)freq) == 0) {
        *lit = true;
        return BUILDING_WINDOW[ci];
    }
    return BUILDING_BODY[ci];
}

/* Decides whether to slice a block across (true) or down (false). We slice the
 * longer side so the two halves stay roughly square; if it's already about
 * square, flip a coin. */
static bool choose_split_axis(bool can_h, bool can_v, float aspect, uint32_t hash)
{
    if (!can_h)                   return false;
    if (!can_v)                   return true;
    if (aspect > SPLIT_ASPECT_HI) return true;    /* clearly tall -> cut across */
    if (aspect < SPLIT_ASPECT_LO) return false;   /* clearly wide -> cut down   */
    return (hash & 1u) != 0u;                      /* about square -> coin flip  */
}

/* Decides where to put the dividing road between lo and hi. GRID puts it dead
 * centre; the other styles nudge it off-centre by a random amount, but never so
 * far that a half gets too small for a lot. */
static int pick_split_pos(int lo, int hi, int mid, Pattern p, uint32_t hash)
{
    int split_pos;
    if (p == PATTERN_GRID || hi <= lo) {
        split_pos = mid;
    } else {
        int range = (int)((float)(hi - lo) * SPLIT_JITTER);
        if (range < 1) range = 1;
        int jit = (int)((hash >> 8) % (uint32_t)(2 * range + 1)) - range;
        split_pos = mid + jit;
    }
    if (split_pos < lo) split_pos = lo;
    if (split_pos > hi) split_pos = hi;
    return split_pos;
}

/* Works out how to draw one already-grown, non-empty cell: returns its glyph
 * and hands back the colour pair and bold/dim setting through *pair and *attr.
 * Pulled out of the draw loop so that loop stays short and readable. */
static chtype cell_appearance(const City *c, int x, int y, int build_step,
                              bool fully_built, int twinkle_t,
                              int *pair, int *attr)
{
    const Cell *cell = &c->map[cidx(c, x, y)];
    uint32_t cell_hash = hash3(x, y, c->seed);
    char glyph;
    *attr = A_NORMAL;

    if (cell->type == CELL_ROAD) {
        glyph = road_glyph_at(c, x, y, build_step);
        *pair = PAIR_ROAD;
        /* Make the big main roads bold so the layout reads at a glance. */
        int road_depth = cell->step / DEPTH_STEP;
        if (road_depth <= MAJOR_ROAD_MAX_DEPTH) *attr = A_BOLD;
    } else if (cell->type == CELL_BUILDING) {
        int ci = cell->color_idx & 3;
        *pair = PAIR_BUILDING_BASE + ci;
        bool lit;
        glyph = building_glyph_at(c, x, y, build_step, cell_hash, &lit);
        if (ci >= 2 || lit) *attr = A_BOLD;
        /* Once built, blink a few lit windows on and off each second. */
        if (fully_built && lit) {
            uint32_t h2 = hash3(x, y, twinkle_t);
            if ((h2 % TWINKLE_1_IN) == 0u) *attr |= A_BOLD;
        }
    } else {  /* CELL_PARK */
        int ci = cell->color_idx & 3;
        *pair = PAIR_PARK_BASE + ci;
        glyph = park_glyph(ci, cell_hash);
        *attr = (ci <= 1) ? A_DIM : A_NORMAL;
    }
    return (chtype)(unsigned char)glyph;
}

/* §4  SIMULATION -- the only code that changes the city. Two things happen
 * here: building the city (lay it out once, then animate it growing in) and
 * moving the cars around once it's done. scene_tick() is the single heartbeat. */

static void city_clear(City *c)
{
    int total = c->w * c->h;
    for (int i = 0; i < total; i++) {
        c->map[i].type      = CELL_EMPTY;
        c->map[i].color_idx = 0;
        c->map[i].step      = 0;
    }
    c->max_step       = 0;
    c->build_step     = 0;
    c->hold_countdown = 0;
}

/* Lays a horizontal road across one row. The appear-times step up along the
 * road so it draws itself in left to right during the animation. */
static void place_road_h(City *c, int row, int x0, int x1, int depth)
{
    int span = x1 - x0 + 1;
    if (span < 1) span = 1;
    int base = depth * DEPTH_STEP;
    for (int x = x0; x <= x1; x++) {
        int idx = cidx(c, x, row);
        c->map[idx].type      = CELL_ROAD;
        c->map[idx].color_idx = 0;
        int s = base + ((x - x0) * (DEPTH_STEP / 2)) / span;
        c->map[idx].step = (uint16_t)s;
        if (s > c->max_step) c->max_step = s;
    }
}

static void place_road_v(City *c, int col, int y0, int y1, int depth)
{
    int span = y1 - y0 + 1;
    if (span < 1) span = 1;
    int base = depth * DEPTH_STEP;
    for (int y = y0; y <= y1; y++) {
        int idx = cidx(c, col, y);
        c->map[idx].type      = CELL_ROAD;
        c->map[idx].color_idx = 0;
        int s = base + ((y - y0) * (DEPTH_STEP / 2)) / span;
        c->map[idx].step = (uint16_t)s;
        if (s > c->max_step) c->max_step = s;
    }
}

/* Fills one rectangle as a single lot — buildings, or (in PARKS) sometimes a
 * park if it's far enough from downtown. Appear-times ramp diagonally so the
 * lot draws itself in from one corner. */
static void place_lot(City *c, int x0, int y0, int x1, int y1,
                      int depth, int seed, Pattern p)
{
    int lcx = (x0 + x1) / 2;
    int lcy = (y0 + y1) / 2;
    int cx  = c->w / 2;
    int cy  = c->h / 2;
    /* Distance from the centre out to a corner — our "1.0" yardstick. */
    float mdx = (float)cx;
    float mdy = (float)cy * ASPECT_Y_F;
    int max_dist = (int)sqrtf(mdx * mdx + mdy * mdy);
    if (max_dist < 1) max_dist = 1;

    uint32_t lot_hash = hash3(x0 ^ (x1 << 8), y0 ^ (y1 << 8), seed + depth);
    uint8_t  ci = lot_color_idx(lcx, lcy, cx, cy, max_dist, lot_hash, p);

    /* In PARKS, randomly make this a park instead — but keep downtown built up. */
    bool is_park = false;
    if (p == PATTERN_PARKS) {
        float dx = (float)(lcx - cx);
        float dy = (float)(lcy - cy) * ASPECT_Y_F;
        float d  = sqrtf(dx * dx + dy * dy) / (float)max_dist;
        if (d > PARK_MIN_RADIUS && (lot_hash % PARK_DENOM) == 0) {
            is_park = true;
            ci = (uint8_t)((lot_hash >> 16) & 3u);
        }
    }

    int span = (x1 - x0 + 1) + (y1 - y0 + 1);
    int base = depth * DEPTH_STEP + DEPTH_STEP / 2;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            int idx = cidx(c, x, y);
            c->map[idx].type      = is_park ? CELL_PARK : CELL_BUILDING;
            c->map[idx].color_idx = ci;
            int rel = (x - x0) + (y - y0);
            int s   = base + (rel * (DEPTH_STEP / 2)) / (span > 0 ? span : 1);
            c->map[idx].step = (uint16_t)s;
            if (s > c->max_step) c->max_step = s;
        }
    }
}

/* The heart of generation, called on itself. Take a rectangle: if it's small
 * enough (or we've recursed too deep), turn it into a lot and stop. Otherwise
 * lay one road through it, splitting it into two smaller rectangles, and do the
 * same thing to each half. */
static void subdivide(City *c, int x0, int y0, int x1, int y1,
                      int depth, int seed, Pattern p)
{
    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;
    bool can_split_h = (h >= 2 * MIN_LOT_H + 1);   /* room for 2 lots + a road */
    bool can_split_v = (w >= 2 * MIN_LOT_W + 1);

    /* Too deep, or too small to split again -> this is a lot. */
    if (depth >= MAX_DEPTH || (!can_split_h && !can_split_v)) {
        place_lot(c, x0, y0, x1, y1, depth, seed, p);
        return;
    }

    /* Otherwise: pick which way to slice and where, lay the road, recurse. */
    uint32_t hh     = hash3(x0 + (x1 << 4), y0 + (y1 << 4), depth + seed);
    float    aspect = ((float)h * ASPECT_Y_F) / (float)w;
    bool     split_h = choose_split_axis(can_split_h, can_split_v, aspect, hh);

    if (split_h) {
        int row = pick_split_pos(y0 + MIN_LOT_H, y1 - MIN_LOT_H, (y0 + y1) / 2, p, hh);
        place_road_h(c, row, x0, x1, depth);
        subdivide(c, x0, y0,      x1, row - 1, depth + 1, seed, p);
        subdivide(c, x0, row + 1, x1, y1,      depth + 1, seed, p);
    } else {
        int col = pick_split_pos(x0 + MIN_LOT_W, x1 - MIN_LOT_W, (x0 + x1) / 2, p, hh);
        place_road_v(c, col, y0, y1, depth);
        subdivide(c, x0,      y0, col - 1, y1, depth + 1, seed, p);
        subdivide(c, col + 1, y0, x1,      y1, depth + 1, seed, p);
    }
}

/* Builds a fresh city: wipe the grid, slice it all up, remember the seed and
 * style. When this returns, max_step is the animation's finish line. */
static void city_build(City *c, int seed, Pattern p)
{
    city_clear(c);
    c->seed          = seed;
    c->pattern_built = p;
    subdivide(c, 0, 0, c->w - 1, c->h - 1, 0, seed, p);
}

/* Drops a car on a random road, facing a random way. If it can't find a road
 * in a reasonable number of tries (a nearly road-less map), it leaves the car
 * inactive to try again next tick. */
static void car_spawn(const City *c, Car *car)
{
    for (int t = 0; t < CAR_SPAWN_TRIES; t++) {
        int x = rand() % c->w;
        int y = rand() % c->h;
        if (c->map[cidx(c, x, y)].type != CELL_ROAD) continue;
        int dir = rand() & 3;
        car->x  = x;  car->y  = y;
        car->dx = (dir == 0) ?  1 : (dir == 1) ? -1 : 0;
        car->dy = (dir == 2) ?  1 : (dir == 3) ? -1 : 0;
        car->active = true;
        return;
    }
    car->active = false;
}

static void cars_spawn_all(const City *c, Car *cars)
{
    for (int i = 0; i < N_CARS; i++) car_spawn(c, &cars[i]);
}

/* Moves one car a step. It likes to keep going straight, turns now and then at
 * junctions, and if the road ahead ends it takes any other road it can find. A
 * car with no road around it at all gives up and respawns elsewhere. */
static void car_step(const City *c, Car *car)
{
    int nx = car->x + car->dx;          /* the cell straight ahead */
    int ny = car->y + car->dy;

    /* Now and then, if there's road ahead, turn left or right instead. */
    if (road_at(c, nx, ny) && (rand() & 7) == 0) {
        int turn_dx[2] = { -car->dy,  car->dy };   /* left, right turn */
        int turn_dy[2] = {  car->dx, -car->dx };
        int pick = rand() & 1;
        int tx   = car->x + turn_dx[pick];
        int ty   = car->y + turn_dy[pick];
        if (road_at(c, tx, ty)) {
            car->dx = turn_dx[pick]; car->dy = turn_dy[pick];
            nx = tx; ny = ty;
        }
    }

    /* Keep driving straight if the road continues. */
    if (road_at(c, nx, ny)) {
        car->x = nx; car->y = ny;
        return;
    }

    /* Road ahead is blocked: take any other road, but avoid an ugly U-turn.
     * If there's truly nowhere to go, mark the car for respawn. */
    static const int dirs[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
    int order = rand() & 3;
    for (int i = 0; i < 4; i++) {
        int idx = (order + i) & 3;
        int tdx = dirs[idx][0], tdy = dirs[idx][1];
        if (tdx == -car->dx && tdy == -car->dy && i < 3) continue;   /* skip U-turn */
        int tx = car->x + tdx, ty = car->y + tdy;
        if (road_at(c, tx, ty)) {
            car->dx = tdx; car->dy = tdy;
            car->x  = tx;  car->y  = ty;
            return;
        }
    }
    car->active = false;
}

static void cars_step_all(const City *c, Car *cars)
{
    for (int i = 0; i < N_CARS; i++) {
        if (cars[i].active) car_step(c, &cars[i]);
        else                car_spawn(c, &cars[i]);
    }
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * WHAT  The whole simulated-and-shown world, as a table of contents: the
 *       domain objects (the city + its traffic), the user-tunable knobs, the
 *       view selections, and the run clock. scene_tick() (§4) is its only
 *       per-tick writer; user events (keys) also set the knobs/selections.
 *
 * VALUE LOGIC (grouped by concept, not by which key changes them):
 *   WHAT is simulated —
 *     city            the terrain + build animation (see City).
 *     cars            N_CARS traffic agents (see Car).
 *     car_step_count  sub-tick divider: cars move once every CAR_STEP_TICKS
 *                     ticks so traffic crawls rather than teleports.
 *   HOW the user drives it —
 *     speed           build pacing, 1..SPEED_MAX; scales BUILD_RATE_DEFAULT.
 *   WHAT we are looking at (RENDER selections, not simulation) —
 *     current_theme   index into themes[] (t/T).
 *     current_pattern the active Pattern (n/p); a switch rebuilds the city.
 *   WHEN / run-state —
 *     paused          freezes scene_tick (rendering continues).
 *     time_secs       simulated seconds; reseeds each rebuild and drives the
 *                     window-twinkle hash so it varies frame to frame. */
typedef struct {
    /* WHAT is simulated — the domain objects. */
    City    city;
    Car     cars[N_CARS];
    int     car_step_count;   /* sub-tick divider: cars move 1/CAR_STEP_TICKS */
    /* HOW the user drives it — the tunable simulation knob. */
    int     speed;            /* build pacing, 1..SPEED_MAX                   */
    /* WHAT we are looking at — RENDER selections, not simulation. */
    int     current_theme;    /* index into themes[] (t/T)                   */
    Pattern current_pattern;  /* active style (n/p); a switch rebuilds       */
    /* WHEN / run-state. */
    bool    paused;           /* freeze scene_tick (rendering continues)     */
    float   time_secs;        /* simulated seconds; reseed + twinkle clock   */
} Scene;

/*
 * scene_rebuild — pick a new seed, run the L-system, restart the
 * build-step animation. Called from scene_init, scene_reset, the 'r'
 * key, and after the hold-phase countdown reaches zero.
 */
static void scene_rebuild(Scene *s)
{
    int seed = (int)(hash3((int)(s->time_secs * 1000.0f),
                           s->city.w, s->city.h)
                     ^ (uint32_t)s->current_pattern * 0xA5A5A5u);
    city_build(&s->city, seed, s->current_pattern);
    s->city.build_step     = 0;
    s->city.hold_countdown = 0;
    cars_spawn_all(&s->city, s->cars);
    s->car_step_count      = 0;
}

static void scene_reset(Scene *s) { scene_rebuild(s); }

static void scene_init(Scene *s, int city_w, int city_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_GRID;
    s->city.w = city_w;
    s->city.h = city_h;
    scene_rebuild(s);
}

/*
 * scene_resize_to — change the city dimensions and rebuild. Called
 * by the screen-resize handler in §7.
 */
static void scene_resize_to(Scene *s, int city_w, int city_h)
{
    s->city.w = city_w;
    s->city.h = city_h;
    scene_rebuild(s);
}

/*
 * scene_tick — advance the build-step counter. Three phases:
 *   1. building  : build_step < max_step
 *   2. completing: build_step just reached max_step → arm the hold
 *   3. holding   : count down, then rebuild
 *
 * Pause halts everything.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    if (s->paused) return;

    City *c = &s->city;

    if (c->build_step < c->max_step) {
        float speed_mul = (float)s->speed / (float)SPEED_DEF;
        int   delta     = (int)((float)BUILD_RATE_DEFAULT * speed_mul);
        if (delta < 1) delta = 1;
        c->build_step += delta;
        if (c->build_step >= c->max_step) {
            c->build_step     = c->max_step;
            c->hold_countdown = HOLD_TICKS_DEF;
        }
    } else {
        /* Traffic only moves while the city is BUILT — during the
         * subdivision animation cars would race over yet-unbuilt
         * road segments. Step them at CAR_STEP_TICKS / sim_fps Hz. */
        s->car_step_count++;
        if (s->car_step_count >= CAR_STEP_TICKS) {
            s->car_step_count = 0;
            cars_step_all(c, s->cars);
        }

        if (c->hold_countdown > 0) {
            c->hold_countdown--;
        } else {
            scene_rebuild(s);
        }
    }
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. There is no stored cosmetic buffer: the window-light
 * twinkle, the bold major-arteries and the lit-windows are all derived at draw
 * time by cell_appearance (§3) from a hash on (x,y,time) — nothing is kept
 * between frames to glow, trail or fade. The moving cars ARE persistent state,
 * but state that advances → SIMULATION (§4), not a cosmetic overlay. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer -- one line. The post-build HOLD before the city is torn
 * down is a single int, City.hold_countdown: armed to HOLD_TICKS_DEF when the
 * build animation completes and counted down inside scene_tick() (§4); at 0
 * the scene rebuilds. The pause toggle (Scene.paused) early-returns
 * scene_tick(). Both are trivial and woven into the simulation tick. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Reads Scene / City / Screen and the §3 glyph deciders;
 * writes ONLY the ncurses back buffer and the colour-pair table (theme_apply
 * / color_init at init). Never touches simulation state, so a frame can be
 * dropped or re-ordered with no effect on the sim. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        init_pair(PAIR_ROAD, t->road, -1);
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_BUILDING_BASE + i), t->building[i], -1);
            init_pair((short)(PAIR_PARK_BASE     + i), t->park    [i], -1);
        }
    } else {
        static const short fb_b[4] = { COLOR_WHITE, COLOR_YELLOW,
                                       COLOR_CYAN,  COLOR_MAGENTA };
        static const short fb_p[4] = { COLOR_GREEN, COLOR_GREEN,
                                       COLOR_GREEN, COLOR_CYAN   };
        init_pair(PAIR_ROAD, COLOR_WHITE, -1);
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_BUILDING_BASE + i), fb_b[i], -1);
            init_pair((short)(PAIR_PARK_BASE     + i), fb_p[i], -1);
        }
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_CAR,   231, -1);   /* bright white headlights */
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_CAR,   COLOR_WHITE, -1);
    }
    theme_apply(0);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport: its full size, the city rectangle that fits inside
 * it, and the offset where that rectangle is centred. Pure presentation
 * geometry — holds NO simulation state — recomputed by screen_layout() at
 * startup and on every SIGWINCH. Map cell (x,y) draws at terminal (gy0+y,
 * gx0+x); city_w/city_h are clamped to [16..MAP_W_MAX]×[8..MAP_H_MAX] so a huge
 * terminal never exceeds CITY_CELLS_MAX and a tiny one stays usable. Rows 0–1
 * are the HUD and the last row the hint line, so the city is inset to avoid
 * overwriting them. */
typedef struct {
    int cols, rows;         /* full terminal size (getmaxyx)        */
    int city_w, city_h;     /* city rectangle that fits inside it   */
    int gx0, gy0;           /* top-left of city on screen           */
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;

    int cw = s->cols;
    int ch = avail_h;
    if (cw > MAP_W_MAX) cw = MAP_W_MAX;
    if (ch > MAP_H_MAX) ch = MAP_H_MAX;
    if (cw < 16) cw = 16;
    if (ch < 8)  ch = 8;

    s->city_w = cw;
    s->city_h = ch;
    s->gx0 = (s->cols  - cw) / 2;
    s->gy0 = top + (avail_h - ch) / 2;
    if (s->gx0 < 0) s->gx0 = 0;
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

/* Overlay the live traffic — one directional arrow per active car over its
 * current road cell. Drawn after the city so cars sit on top. */
static void draw_traffic(const Screen *sc, const Car *cars)
{
    for (int i = 0; i < N_CARS; i++) {
        const Car *car = &cars[i];
        if (!car->active) continue;
        int sy = sc->gy0 + car->y;
        int sx = sc->gx0 + car->x;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        if (sx < 0 || sx >= sc->cols)     continue;
        attron(COLOR_PAIR(PAIR_CAR) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)car_glyph(car));
        attroff(COLOR_PAIR(PAIR_CAR) | A_BOLD);
    }
}

/* Paint the current frame: every constructed, non-empty cell gets the glyph +
 * colour cell_appearance() decides for it; then live traffic is overlaid once
 * the city is fully built. Render leaf — takes only what it draws (grid,
 * traffic, the twinkle clock), never the whole Scene. */
static void scene_draw(const Screen *sc, const City *c, const Car *cars,
                       float time_secs)
{
    int  build_step  = c->build_step;
    bool fully_built = (build_step >= c->max_step);
    int  twinkle_t   = (int)time_secs;

    for (int y = 0; y < c->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < c->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell *cell = &c->map[cidx(c, x, y)];
            if (cell->step > build_step)  continue;   /* not yet built */
            if (cell->type == CELL_EMPTY) continue;

            int pair, attr;
            chtype glyph = cell_appearance(c, x, y, build_step,
                                           fully_built, twinkle_t, &pair, &attr);
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    if (fully_built) draw_traffic(sc, cars);
}

/* Draw a 4-glyph colour-ramp swatch at (row,x): glyph i in pair pair_base+i.
 * Returns the x just past it. Shared by the building and park legends. */
static int draw_swatch(int row, int x, int pair_base, const char glyphs[4], int attr)
{
    for (int i = 0; i < 4; i++) {
        attron(COLOR_PAIR(pair_base + i) | attr);
        mvaddch(row, x, (chtype)(unsigned char)glyphs[i]);
        attroff(COLOR_PAIR(pair_base + i) | attr);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, build phase, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    bool fully_built = (s->city.build_step >= s->city.max_step);
    const char *phase = s->paused     ? "PAUSED   "
                      : fully_built   ? "BUILT    "
                                      : "BUILDING ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, phase, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern, theme, the building + park colour swatches, a car icon,
 * and the city size + build progress. Fixed left-aligned layout. */
static void draw_param_line(const Scene *s)
{
    const City *c = &s->city;
    int x = 1;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " bld:");  attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    x = draw_swatch(1, x, PAIR_BUILDING_BASE, BUILDING_BODY, A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " park:");  attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    x = draw_swatch(1, x, PAIR_PARK_BASE, PARK_HUD_GLYPH, A_NORMAL);

    /* Car indicator — a bright arrow showing traffic exists. */
    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " car:");  attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    attron(COLOR_PAIR(PAIR_CAR) | A_BOLD);
    mvaddch(1, x, '>');
    attroff(COLOR_PAIR(PAIR_CAR) | A_BOLD);
    x++;

    int pct = (c->max_step > 0) ? (c->build_step * 100) / c->max_step : 100;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  %dx%d  build:%3d%% ", c->w, c->h, pct);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:rebuild  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, progress, run-state —
 * so narrowing it to a sub-type would mean passing half a dozen scalars. A
 * const read can't re-couple the layers; scene_draw and the leaf deciders stay
 * narrow. Reads as: draw the city, then lay the HUD over it. */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, &s->city, s->cars, s->time_secs);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " PROCEDURAL CITY ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM + DELAYS; twinkle is render-derived) -> screen_draw
 * (RENDER) -> screen_present -> input. app_handle_key() / app_do_resize()
 * mutate state on USER EVENTS and are deliberately OUTSIDE the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * Top-level harness binding the simulation (scene) to the terminal (screen),
 * plus the loop's PERFORMANCE knob and the async signal flags. A single static
 * instance (g_app) exists ONLY so the signal handlers — which may fire between
 * any two instructions — can reach the flags; everything else passes App
 * explicitly. Only init + the main loop touch it whole; other functions take a
 * sub-object. The fixed-timestep rate (sim_fps) decouples simulation speed from
 * frame rate (Fiedler, "Fix Your Timestep!"; §8 main()). */
typedef struct {
    /* the two worlds it binds */
    Scene                 scene;        /* WHAT is simulated + shown        */
    Screen                screen;       /* WHERE it is drawn                */
    /* loop control */
    int                   sim_fps;      /* fixed-timestep rate, SIM_FPS_* Hz*/
    /* volatile sig_atomic_t: written from signal handlers, so the compiler
     * must re-read them each loop and the write is atomic w.r.t. the
     * interrupted code. */
    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM → exit */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, served next loop*/
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize_to(&app->scene, app->screen.city_w, app->screen.city_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                                break;

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

    /* Pattern switch — rebuild immediately so the user sees the new
     * style instead of the leftover layout. */
    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_rebuild(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_rebuild(s);
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
    scene_init(&app->scene, app->screen.city_w, app->screen.city_h);

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
