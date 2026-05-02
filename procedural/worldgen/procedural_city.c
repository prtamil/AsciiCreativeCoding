/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_city.c
 *   — A procedural city plan: a recursive L-system rewrites a single
 *     "city block" into roads and ever-smaller sub-blocks; the leaves
 *     of the recursion are lots that get assigned building types.
 *     The city is drawn growing in real time, depth-by-depth, then
 *     held briefly before being torn down and re-grown from a new
 *     seed.
 *
 * DEMO: A blank rectangle. A single road slices it in half. Each
 *       half gets its own road. Each quarter gets another. The
 *       pattern of streets fans out in ~5 seconds, then BUILDINGS
 *       rise in the lots — each one rendered as a FRAMED BOX with
 *       side walls '|', a foundation '_' (or '=' for skyscrapers),
 *       and an interior dotted with lit windows ('"' on apartments,
 *       '*' on offices and skyscrapers). Low-density houses skip
 *       the frame and read as a 'n'/'m' textured row. Once the city
 *       is BUILT, ~28 traffic dots — directional arrows '>' '<' '^'
 *       'v' — start cruising the road network, occasionally turning
 *       at intersections. Major arteries are rendered in bold so
 *       the road hierarchy is visible at a glance. After a few
 *       seconds the city is bulldozed and a new one grows in its
 *       place — same algorithm, fresh random seed, completely
 *       different layout.
 *
 *       Cycle four city styles with n / p:
 *
 *         GRID       every split lands at exact centre — strict
 *                    Manhattan / Cartesian grid
 *         ORGANIC    splits jitter ±33% from centre — irregular
 *                    block sizes, more "old European"
 *         DISTRICTS  ORGANIC layout, but building types follow zones
 *                    (skyscrapers downtown, suburbs at the edges)
 *         PARKS      DISTRICTS layout, but ~25 % of lots are parks
 *                    instead of buildings — visible green spaces
 *
 * Study alongside:
 *   ../generational/bsp_dungeon_showcase.c
 *      — the same BSP recursion, applied to dungeon rooms instead
 *        of city lots. The recursion engine is identical; only the
 *        leaf interpretation differs.
 *   ../worldgen/procedural_galaxy.c
 *      — also a "world from a function" file, but using radial /
 *        polar geometry (continuous) rather than Cartesian recursion
 *        (discrete). Two complementary strategies for procedural
 *        place-making.
 *
 * Section map:
 *   §1 config    — geometry, depth limits, themes, pattern enum
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (road + 4 buildings + 4 parks)
 *   §5 city      — hash, recursive subdivide, place_road / place_lot
 *   §6 scene     — Scene state, build animation, hold + rebuild
 *   §7 screen    — neighbour-aware road glyphs, twinkle, HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the build animation
 *   r          tear down and rebuild with a new seed
 *   n / N      next pattern  (GRID → ORGANIC → DISTRICTS → PARKS → ...)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster build
 *   -          slower build
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/procedural_city.c \
 *     -o city -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two ideas composed together.
 *
 *                  (1) An L-SYSTEM (Lindenmayer system) over rectangu-
 *                      lar blocks. The alphabet has two non-terminal
 *                      symbols { Block(w,h) } and three terminals
 *                      { HSplit, VSplit, Lot }. The productions are
 *                      stochastic and parametric:
 *
 *                        Block(w, h) → HSplit  Block(w, ½h) Block(w, ½h)
 *                                         when h > 2·MIN_LOT_H + 1
 *                        Block(w, h) → VSplit  Block(½w, h) Block(½w, h)
 *                                         when w > 2·MIN_LOT_W + 1
 *                        Block(w, h) → Lot
 *                                         when both above fail
 *                                         OR when depth = MAX_DEPTH
 *
 *                      Choice between HSplit / VSplit is biased by the
 *                      block's aspect ratio (always split the longer
 *                      side) with a hash-driven tie-breaker. The
 *                      *position* of the split is exact-centre in the
 *                      GRID pattern and jittered ±33 % in the others.
 *                      The L-system is iterated to fixed-point — each
 *                      Block keeps rewriting until it becomes a Lot.
 *
 *                      Geometrically this is a binary space partition
 *                      (BSP) of the rectangle; "L-system" is the
 *                      formal-language framing — the city plan is a
 *                      derivation tree of the grammar.
 *
 *                  (2) PER-LOT CONTENT. Each Lot leaf is filled with
 *                      ONE building type, chosen as a function of:
 *                        - the lot's centre position vs the city centre
 *                          (zoning gradient: skyscrapers downtown,
 *                           houses at the edges, in DISTRICTS / PARKS
 *                           patterns)
 *                        - a hash of the lot's bounds (random tint
 *                          variation)
 *                        - one bit of the same hash (PARKS pattern:
 *                          ~25 % of lots become green spaces)
 *                      Building "height" is encoded by glyph: h, H, #,
 *                      @ — visually progressive density.
 *
 *                  Plus an ANIMATION PASS — every cell is stamped with
 *                  a "creation step" computed during subdivision, then
 *                  rendered only when the global build_step counter
 *                  has caught up. This visualises the L-system
 *                  derivation in time.
 *
 * Data-structure : A flat 2-D Cell array, one per terminal cell:
 *                    Cell { type:1, color:1, step:2 } = 4 bytes
 *                  No lot list, no road graph — every per-cell
 *                  question (is this a road? what colour? when does
 *                  it appear?) reads exactly one Cell. The recursion
 *                  itself only writes — it allocates nothing.
 *
 * Rendering      : ASCII only. Roads are drawn with a NEIGHBOUR-AWARE
 *                  glyph chooser: at each road cell, look N/S/E/W; if
 *                  any vertical road neighbour AND any horizontal road
 *                  neighbour, draw '+'; otherwise draw '|' or '-'. The
 *                  same logic naturally produces T-junctions and
 *                  corners without explicit case enumeration. After
 *                  the build completes, building cells get a subtle
 *                  per-second "window-light" twinkle: a cheap hash on
 *                  (x, y, time) flips ~2 % of cells to A_BOLD,
 *                  imitating the random pattern of lit windows in a
 *                  night skyline.
 *
 * Performance    : The whole subdivision finishes in microseconds —
 *                  ~20 000 cells written by ~500 recursive calls.
 *                  All per-frame work is the render loop:
 *                  O(W·H) cell visits with one neighbour lookup and
 *                  one mvaddch each — well under 1 ms for the
 *                  full 240×80 grid. No allocation in steady state;
 *                  rebuild reuses the same buffer.
 *
 * References     :
 *   • Lindenmayer, A. (1968) — "Mathematical models for cellular
 *     interactions in development", J. Theor. Biol. 18.
 *     The original L-system paper.
 *   • Prusinkiewicz & Lindenmayer (1990) — The Algorithmic Beauty of
 *     Plants  https://algorithmicbotany.org/papers/abop/abop.pdf
 *     The canonical reference for L-system formalisms.
 *   • Parish, Y. & Müller, P. (2001) — "Procedural Modeling of Cities",
 *     SIGGRAPH 2001.  https://www.researchgate.net/publication/2557915
 *     The CityEngine paper — L-system roads on a real-world scale.
 *   • Wikipedia — Binary space partitioning
 *     https://en.wikipedia.org/wiki/Binary_space_partitioning
 *   • Red Blob Games — Dungeon generation with BSP trees
 *     https://www.redblobgames.com/articles/dungeon-generation/
 *   • Inigo Quilez — "City stamping" technique notes
 *     https://iquilezles.org/articles/menger/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a city, do not build it forward (lay one road, then another,
 * then place buildings); build it BACKWARD. Start with the answer —
 * "this whole rectangle is one city block" — and rewrite that one
 * block into smaller blocks separated by roads. Repeat until the
 * blocks are too small to subdivide further; those terminal blocks
 * are the lots. The road network and the lot grid emerge for free
 * from the subdivision tree.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of a sheet of paper and a ruler. You draw ONE line that splits
 * the page in half — that is your first street. Now you have two
 * smaller pages. On each of them you draw another line, splitting
 * them again. Now four. Each new line is a street; the regions they
 * bound are "blocks". Keep recursing on each block until a block is
 * smaller than a couple of buildings — at that point you stop, and
 * call that block a lot.
 *
 * Equivalently — and this is the L-system view — write a single
 * letter B on the page, meaning "the whole city is one Block". Apply
 * a rewriting rule: replace B with R B B (a Road and two smaller
 * Blocks). Apply the rule again to each of the new B's. Eventually
 * you stop rewriting (the B is too small) and replace it with L (a
 * Lot). The string of letters R, R, R, ..., L, L, L, ... — read in
 * order — describes the entire city plan.
 *
 * The recursion tree IS the city. The depth of the tree IS the
 * road hierarchy: depth-0 splits become arterial roads, depth-3 splits
 * become side streets, depth-7 splits become alleys.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT.   Clear all cells to EMPTY. Pick a random seed.
 *  2. SUBDIVIDE(rect, depth):
 *      a. If depth ≥ MAX_DEPTH, OR rect cannot fit two min-sized
 *         lots in either axis: stamp every cell of rect with type
 *         BUILDING (or PARK in some patterns) and return.
 *      b. Choose split axis:
 *           - if rect.h × ASPECT_Y > 1.4 × rect.w  → split horizontally
 *           - else if rect.w > 1.4 × rect.h × ASPECT_Y → split vertically
 *           - else: hash(rect) low bit picks one
 *      c. Choose split position:
 *           - GRID    pattern: exact midpoint
 *           - others : midpoint + jitter, where jitter ∝ hash(rect),
 *                      clipped so each half has room for ≥ MIN_LOT
 *      d. Stamp the split line as ROAD cells (one row or one column).
 *      e. Recurse on the two halves with depth+1.
 *  3. STEP TAGGING. As each cell is stamped, give it a "creation
 *     step" that depends on (depth, kind). Lower depth → smaller step.
 *     Roads stamp first; lots within the same depth stamp after.
 *  4. ANIMATE. Each frame increment build_step toward max_step.
 *     Render only cells with cell.step ≤ build_step. The viewer sees
 *     the recursion unfold.
 *  5. HOLD then REBUILD. After build_step reaches max_step, hold the
 *     finished city for HOLD_TICKS, then re-seed and goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Aspect-corrected ratio of a block (cells are 2× taller than wide):
 *     aspect = (h · ASPECT_Y) / w
 *     split_horizontally = (aspect > 1.4) — tall block: cut the long way
 *
 *  Split position (ORGANIC family):
 *     mid    = (lo + hi) / 2
 *     range  = (hi − lo) / 3
 *     jitter = (hash(rect) mod (2·range+1)) − range
 *     split  = clamp(mid + jitter, lo + MIN_LOT, hi − MIN_LOT)
 *
 *  Step tag for a cell stamped during depth d:
 *     step(road) = d · 1000 + ((x − x0) · 500) / span_x
 *     step(lot)  = d · 1000 + 500 + ((x + y) · 500) / (lot_w + lot_h)
 *
 *  Building type by zoning (DISTRICTS / PARKS):
 *     dist    = √(((cx − Cx)·1)² + ((cy − Cy)·ASPECT_Y)²) / D_max
 *     type    = dist < 0.25 → 3 (skyscraper)
 *               dist < 0.50 → 2 (office)
 *               dist < 0.75 → 1 (apartment)
 *               else        → 0 (house)
 *
 *  Window-light twinkle (after build completes):
 *     bold = (hash(x, y, ⌊t⌋) mod 50 == 0)
 *     ~2 % of building cells light up; pattern shuffles each second.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • MIN-LOT ENFORCEMENT. If the split position is unconstrained, the
 *    recursion can produce 1×1 "lots" with no room for a building.
 *    Always clip split to [lo + MIN_LOT, hi − MIN_LOT]; if that
 *    interval is empty, terminate the recursion at the current rect.
 *
 *  • ASPECT BIAS. Without the ASPECT_Y multiplier, the algorithm
 *    sees a 60×30 cell rect as "wider than tall" and prefers vertical
 *    splits, but on a screen 60 cells wide × 30 cells tall is nearly
 *    SQUARE (because cells are 2× taller). Always multiply the row
 *    extent by ASPECT_Y when comparing for the split-axis decision —
 *    otherwise blocks look extremely flat.
 *
 *  • RECURSION DEPTH BOUND. Without a depth cap, a 240×80 city with a
 *    very small MIN_LOT could recurse ~20 levels deep, generating
 *    half a million lots and choking the per-frame render. MAX_DEPTH
 *    keeps the tree shallow enough that the lot count stays under
 *    ~2 000 — comfortable for ASCII rendering.
 *
 *  • ROAD CELL IDENTITY. After subdivision a cell is either ROAD,
 *    BUILDING, PARK, or EMPTY. Roads get a special drawing path that
 *    looks at four neighbours; buildings/parks just print their glyph.
 *    Don't try to decide in advance whether a road cell is a "+",
 *    "-" or "|" — you can't, because it depends on neighbours that
 *    haven't been placed yet at the time you're recursing. Decide at
 *    render time.
 *
 *  • STEP MONOTONICITY. The cell.step values must be a 16-bit
 *    quantity that grows with depth (so the build animation reveals
 *    things in roughly tree-order). With depth_factor = 1000 and
 *    MAX_DEPTH = 9, max_step ≈ 10 000 — well under the 65 535 limit
 *    of uint16_t. If you raise MAX_DEPTH past 50, change cell.step
 *    to uint32_t.
 *
 *  • RESIZE = REBUILD. The city's grid size is fixed at scene init;
 *    on terminal resize we MUST throw away the existing layout and
 *    re-subdivide at the new dimensions. There is no incremental
 *    "reflow" of a BSP tree; just rebuild.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE during the build (space). The wavefront freezes —
 *    cells with step ≤ build_step are visible, the rest are blank.
 *    Resume: build continues from exactly where it stopped.
 *
 *  • RESET (r). The city collapses to empty and a NEW layout grows
 *    in. The road structure should be visibly different — same
 *    pattern (e.g. GRID), but the seed-driven choices differ.
 *
 *  • GRID pattern. Every road runs through the EXACT middle of its
 *    block. The result is a perfectly regular grid: rows of roads at
 *    powers-of-2 offsets, columns at powers-of-2 offsets. If the
 *    splits are off-centre, GRID has accidentally inherited the
 *    jitter from another pattern.
 *
 *  • ORGANIC pattern. Block sizes visibly vary; some lots are 6×3,
 *    others 12×8. Roads do not line up across blocks (compare to
 *    GRID where they do). If everything still looks like a grid,
 *    the jitter is being clipped to zero.
 *
 *  • DISTRICTS pattern. Walk visually outward from the city centre:
 *    you should see a CLEAR colour progression from one tint to
 *    another over the four building palettes. Framed skyscraper
 *    boxes (interior '@', foundation '=') cluster at the centre;
 *    bumpy 'n'/'m' houses with no frame dominate the outer ring.
 *
 *  • PARKS pattern. Visible green park lots scattered through the
 *    city — light grass ',' '.', wooded park 'Y', dense forest 'T'.
 *    None in the central skyscraper cluster, more toward the edges.
 *
 *  • TWINKLE. After the build completes (HUD says "BUILT"), watch
 *    a particular WINDOW cell. About once every few seconds it
 *    should flicker brighter for one tick.
 *
 *  • TRAFFIC. After BUILT, ~28 directional arrows ('>' '<' '^' 'v')
 *    appear on the road network and step every few frames. They
 *    follow forward direction until the road ends, then pick a new
 *    direction; at intersections they occasionally turn. They never
 *    leave the road or appear during the build phase.
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
    /* Hard upper bound on the city grid. Anything larger is clipped. */
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CITY_CELLS_MAX      = MAP_W_MAX * MAP_H_MAX,

    /* Subdivision controls. MAX_DEPTH bounds the recursion; the actual
     * depth reached is usually less because MIN_LOT_W/H halts splitting
     * sooner. With MIN_LOT 4×3, a 240×80 grid bottoms out around
     * depth 8 — comfortable for visual reading. */
    MAX_DEPTH           =   9,
    MIN_LOT_W           =   4,
    MIN_LOT_H           =   3,

    /* Build animation pacing — how many step-units to advance per
     * tick at default speed. ~28 steps/tick × 60 ticks/s × 6 s = 10 080,
     * which matches max_step = (MAX_DEPTH+1) * 1000 = 10 000. */
    BUILD_RATE_DEFAULT  =  28,

    /* Hold the finished city for ~6 s before tearing down. */
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

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_ROAD           =   3,
    PAIR_BUILDING_BASE  =   4,    /* +0..+3 = 4 building tints       */
    PAIR_PARK_BASE      =   8,    /* +0..+3 = 4 park tints           */
    PAIR_FLASH          =  12,    /* reset flash                     */
    PAIR_CAR            =  13,    /* moving traffic dots             */

    /* Traffic.  N_CARS dots step every CAR_STEP_TICKS ticks during
     * the BUILT phase — at 60 fps with CAR_STEP_TICKS=4 they move
     * ~15 cells/sec, a comfortable scrolling-traffic feel. */
    N_CARS              =  28,
    CAR_STEP_TICKS      =   4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Cell aspect — terminal cells are ~2× taller than wide, so we
 * multiply the row extent by this when comparing aspect ratios. */
#define ASPECT_Y_F      2.0f

/* Step-tag spacing per depth level. Roads at depth d occupy steps
 * [d·DEPTH_STEP, d·DEPTH_STEP + DEPTH_STEP/2); lots occupy the upper
 * half. With DEPTH_STEP = 1000 and MAX_DEPTH = 9, max_step ≈ 10 000. */
#define DEPTH_STEP      1000

/* ORGANIC / DISTRICTS / PARKS jitter — split position offset from
 * mid as a fraction of the half-extent. 0.33 = ±33 %. */
#define SPLIT_JITTER    0.33f

/* PARKS pattern: probability that a lot becomes a park rather than
 * a building, expressed as "1 in N". */
#define PARK_DENOM      4

/*
 * Building glyph encoding — buildings are drawn as FRAMED BOXES, not
 * solid fills, so the eye sees actual structures with side walls,
 * a foundation, and an interior dotted with lit windows:
 *
 *       _____
 *      |HH"H|     ← top row: just below the road
 *      |HHHH|     ← interior body, occasional '"' = lit window
 *      |____|     ← bottom row foundation, side walls '|'
 *
 * The four index slots are the four building "heights":
 *   0 — low-rise houses        body 'n' / 'm'   (no frame; too small)
 *   1 — apartments / townhouse body 'H'  windows '"'   base '_'
 *   2 — offices / commercial   body '#'  windows '*'   base '_'
 *   3 — skyscraper / landmark  body '@'  windows '*'   base '='
 *
 * Lot-edge detection at render time picks frame vs body — the same
 * cell is "interior" in a 12×8 lot but "edge" in a 4×3 lot. No
 * per-cell role is stored; everything is derived from neighbours.
 */
static const char BUILDING_BODY  [4] = { 'n', 'H', '#', '@' };
static const char BUILDING_WINDOW[4] = { 'm', '"', '*', '*' };
static const char BUILDING_BASE  [4] = { 'n', '_', '_', '=' };

/*
 * WINDOW_FREQ[i] — for the i-th building type, 1 in N interior cells
 * is rendered as a window (BUILDING_WINDOW glyph) instead of a wall.
 * Denser types get more windows so a skyscraper visibly glitters
 * with bright cells while a townhouse stays mostly solid.
 */
static const int  WINDOW_FREQ   [4] = {   2,   8,   4,   3 };

/*
 * Park glyph palette — each park type has its own glyph mix, cycled
 * per cell by a hash so the lot reads as a textured natural patch
 * (grass + flowers + trees) rather than a flat fill.
 *
 *   0 light grass : ',' '.'
 *   1 garden      : ',' '.' '\''
 *   2 wooded park : ',' 'Y' '.'        (Y = stylised tree)
 *   3 dense forest: 'Y' 'T' '*'        (T = trunk)
 *
 * PARK_HUD_GLYPH is the single most-distinctive glyph per type — used
 * for the HUD palette swatch only, not for in-city rendering.
 */
static const char PARK_HUD_GLYPH[4] = { ',', '.', 'Y', 'T' };

/*
 * Pattern — four ways the same recursive subdivision is parameterised.
 */
typedef enum {
    PATTERN_GRID      = 0,
    PATTERN_ORGANIC   = 1,
    PATTERN_DISTRICTS = 2,
    PATTERN_PARKS     = 3,
    N_PATTERNS        = 4,
} Pattern;

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

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Each theme provides:
 *   - 1 road tint (gray-ish for the asphalt)
 *   - 4 building tints (houses → skyscrapers, ordered cool-to-warm)
 *   - 4 park tints (greens / accents)
 */
typedef struct {
    const char *name;
    short       road;
    short       building[4];
    short       park    [4];
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
        init_pair(PAIR_FLASH, 226, -1);
        init_pair(PAIR_CAR,   231, -1);   /* bright white headlights */
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
        init_pair(PAIR_CAR,   COLOR_WHITE, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  city — hash, recursive subdivide, lot placement                    */
/* ===================================================================== */

/*
 * hash3 — stateless 3-int avalanche hash. Same routine as the other
 * procedural showcases. Used to drive split-position jitter, building
 * tint selection, and the per-window twinkle.
 */
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

/* Per-cell entry. 4 bytes — kept tight so the whole map fits in L2
 * cache. type: enum below; color_idx: 0..3 sub-tint; step: animation
 * threshold (cell visible iff cell.step ≤ build_step). */
typedef enum {
    CELL_EMPTY    = 0,
    CELL_ROAD     = 1,
    CELL_BUILDING = 2,
    CELL_PARK     = 3,
} CellType;

typedef struct {
    uint8_t  type;
    uint8_t  color_idx;
    uint16_t step;
} Cell;

typedef struct {
    int   w, h;
    Cell  map[CITY_CELLS_MAX];
    int   max_step;
    int   build_step;
    int   hold_countdown;     /* ticks remaining once build completes */
    int   seed;
    Pattern pattern_built;    /* the pattern THIS layout was generated for */
} City;

static inline int cidx(const City *c, int x, int y) { return y * c->w + x; }

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

/*
 * place_road_h — stamp a horizontal road across a row, between
 * inclusive x bounds. Step values increment along x so the road
 * "draws" left to right within its depth band.
 */
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

/*
 * lot_color_idx — pick a building tint for a lot.
 *
 *   GRID, ORGANIC : random tint via hash, no spatial structure.
 *   DISTRICTS     : zoning gradient — radius from city centre maps
 *                   to building "height". '@'/'#'/'H'/'h' from
 *                   downtown out to suburbs.
 *   PARKS         : same as DISTRICTS for buildings; parks pick
 *                   their own tint via the same hash for variety.
 *
 * lcx, lcy : lot centre in cell coords
 * cx,  cy  : city centre
 */
static uint8_t lot_color_idx(int lcx, int lcy, int cx, int cy,
                             int max_dist, uint32_t lot_hash, Pattern p)
{
    if (p == PATTERN_GRID || p == PATTERN_ORGANIC) {
        return (uint8_t)((lot_hash >> 8) & 3u);
    }

    /* Aspect-corrected radial distance from city centre. */
    float dx = (float)(lcx - cx);
    float dy = (float)(lcy - cy) * ASPECT_Y_F;
    float d  = sqrtf(dx * dx + dy * dy) / (float)max_dist;
    if      (d < 0.25f) return 3;        /* skyscraper           */
    else if (d < 0.50f) return 2;        /* office               */
    else if (d < 0.75f) return 1;        /* apartment            */
    else                return 0;        /* house                */
}

/*
 * place_lot — stamp every cell of an inclusive rectangle as a single
 * lot. In PARKS pattern, ~1/PARK_DENOM lots become parks (CELL_PARK)
 * — but only outside the densest centre zone, where downtown should
 * stay built up.
 *
 * The cells get a stepwise creation step so the lot "draws in"
 * diagonally during the build animation.
 */
static void place_lot(City *c, int x0, int y0, int x1, int y1,
                      int depth, int seed, Pattern p)
{
    int lcx = (x0 + x1) / 2;
    int lcy = (y0 + y1) / 2;
    int cx  = c->w / 2;
    int cy  = c->h / 2;
    /* Aspect-corrected max distance — corner of city. */
    float mdx = (float)cx;
    float mdy = (float)cy * ASPECT_Y_F;
    int max_dist = (int)sqrtf(mdx * mdx + mdy * mdy);
    if (max_dist < 1) max_dist = 1;

    uint32_t lot_hash = hash3(x0 ^ (x1 << 8), y0 ^ (y1 << 8), seed + depth);
    uint8_t  ci = lot_color_idx(lcx, lcy, cx, cy, max_dist, lot_hash, p);

    /* PARKS pattern — drop a park here if hash says so, but never in
     * the central skyscraper zone (we want a downtown). */
    bool is_park = false;
    if (p == PATTERN_PARKS) {
        float dx = (float)(lcx - cx);
        float dy = (float)(lcy - cy) * ASPECT_Y_F;
        float d  = sqrtf(dx * dx + dy * dy) / (float)max_dist;
        if (d > 0.30f && (lot_hash % PARK_DENOM) == 0) {
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

/*
 * subdivide — the L-system production step.
 *
 *   B(x0,y0,x1,y1) →  HSplit(row)  B(x0,y0,x1,row-1)  B(x0,row+1,x1,y1)
 *                  |  VSplit(col)  B(x0,y0,col-1,y1)  B(col+1,y0,x1,y1)
 *                  |  Lot(rect)
 *
 * The third rule fires when (a) recursion depth has hit MAX_DEPTH or
 * (b) neither axis can host two sub-blocks of MIN_LOT size plus a road.
 *
 * Split-axis choice is biased by aspect (always cut the longer side);
 * a hash low-bit picks when the block is roughly square. Split position
 * is exact-centre under PATTERN_GRID, otherwise jittered by ±SPLIT_JITTER.
 */
static void subdivide(City *c, int x0, int y0, int x1, int y1,
                      int depth, int seed, Pattern p)
{
    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;

    bool can_split_h = (h >= 2 * MIN_LOT_H + 1);   /* two lots + 1 road row */
    bool can_split_v = (w >= 2 * MIN_LOT_W + 1);

    if (depth >= MAX_DEPTH || (!can_split_h && !can_split_v)) {
        place_lot(c, x0, y0, x1, y1, depth, seed, p);
        return;
    }

    /* Decide axis — aspect first, hash second. ASPECT_Y_F so blocks
     * look square on the (taller) terminal cells. */
    bool split_h;
    float aspect = ((float)h * ASPECT_Y_F) / (float)w;
    uint32_t hh  = hash3(x0 + (x1 << 4), y0 + (y1 << 4), depth + seed);

    if (!can_split_h)        split_h = false;
    else if (!can_split_v)   split_h = true;
    else if (aspect > 1.4f)  split_h = true;
    else if (aspect < 0.7f)  split_h = false;
    else                     split_h = (hh & 1u) != 0u;

    /* Pick split position, clipped so each half can host MIN_LOT. */
    int split_pos;
    if (split_h) {
        int lo  = y0 + MIN_LOT_H;
        int hi  = y1 - MIN_LOT_H;
        int mid = (y0 + y1) / 2;
        if (p == PATTERN_GRID || hi <= lo) {
            split_pos = mid;
        } else {
            int range = (int)((float)(hi - lo) * SPLIT_JITTER);
            if (range < 1) range = 1;
            int jit = (int)((hh >> 8) % (uint32_t)(2 * range + 1)) - range;
            split_pos = mid + jit;
        }
        if (split_pos < lo) split_pos = lo;
        if (split_pos > hi) split_pos = hi;
        place_road_h(c, split_pos, x0, x1, depth);
        subdivide(c, x0, y0,            x1, split_pos - 1, depth + 1, seed, p);
        subdivide(c, x0, split_pos + 1, x1, y1,            depth + 1, seed, p);
    } else {
        int lo  = x0 + MIN_LOT_W;
        int hi  = x1 - MIN_LOT_W;
        int mid = (x0 + x1) / 2;
        if (p == PATTERN_GRID || hi <= lo) {
            split_pos = mid;
        } else {
            int range = (int)((float)(hi - lo) * SPLIT_JITTER);
            if (range < 1) range = 1;
            int jit = (int)((hh >> 8) % (uint32_t)(2 * range + 1)) - range;
            split_pos = mid + jit;
        }
        if (split_pos < lo) split_pos = lo;
        if (split_pos > hi) split_pos = hi;
        place_road_v(c, split_pos, y0, y1, depth);
        subdivide(c, x0,            y0, split_pos - 1, y1, depth + 1, seed, p);
        subdivide(c, split_pos + 1, y0, x1,            y1, depth + 1, seed, p);
    }
}

/*
 * city_build — the entry point. Clears the map, runs subdivision,
 * remembers the seed and pattern. After returning, max_step holds the
 * highest step value any cell received, which the animation will ramp
 * to.
 */
static void city_build(City *c, int seed, Pattern p)
{
    city_clear(c);
    c->seed          = seed;
    c->pattern_built = p;
    subdivide(c, 0, 0, c->w - 1, c->h - 1, 0, seed, p);
}

/* ===================================================================== */
/* §6  scene — state, build animation, hold + rebuild                     */
/* ===================================================================== */

/*
 * Car — one traffic dot. Position is integer cell coords; direction
 * is one of the four cardinals. A car is "active" iff it currently
 * sits on a road cell with a valid forward path; a stranded car is
 * marked inactive and respawned on the next tick.
 */
typedef struct {
    int  x, y;
    int  dx, dy;
    bool active;
} Car;

static inline bool road_at(const City *c, int x, int y)
{
    if (x < 0 || x >= c->w || y < 0 || y >= c->h) return false;
    return c->map[cidx(c, x, y)].type == CELL_ROAD;
}

/*
 * car_spawn — place a car at a random road cell with a random
 * direction. If no road cell is found in 200 tries (very small map),
 * the car is left inactive — it will respawn next tick.
 */
static void car_spawn(const City *c, Car *car)
{
    for (int t = 0; t < 200; t++) {
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

/*
 * car_step — advance one car along the road network. The driver
 * prefers to keep going forward; at intersections it occasionally
 * turns; at dead ends it picks any valid neighbour. Stranded cars
 * (no road neighbour at all) deactivate and respawn next tick.
 */
static void car_step(const City *c, Car *car)
{
    int nx = car->x + car->dx;
    int ny = car->y + car->dy;

    /* Sometimes turn at intersections — 1 in 8 chance per move. */
    if (road_at(c, nx, ny) && (rand() & 7) == 0) {
        int pdx[2] = { -car->dy,  car->dy };
        int pdy[2] = {  car->dx, -car->dx };
        int pick   = rand() & 1;
        int tx     = car->x + pdx[pick];
        int ty     = car->y + pdy[pick];
        if (road_at(c, tx, ty)) {
            car->dx = pdx[pick]; car->dy = pdy[pick];
            nx = tx; ny = ty;
        }
    }

    if (road_at(c, nx, ny)) {
        car->x = nx; car->y = ny;
        return;
    }

    /* Forward blocked — pick any valid direction (skip reversing if
     * possible; it looks better when cars don't yo-yo). */
    static const int dirs[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
    int order = rand() & 3;
    for (int i = 0; i < 4; i++) {
        int idx = (order + i) & 3;
        int tdx = dirs[idx][0], tdy = dirs[idx][1];
        if (tdx == -car->dx && tdy == -car->dy && i < 3) continue;
        int tx = car->x + tdx, ty = car->y + tdy;
        if (road_at(c, tx, ty)) {
            car->dx = tdx; car->dy = tdy;
            car->x  = tx;  car->y  = ty;
            return;
        }
    }
    /* Truly stranded — reseed on next tick. */
    car->active = false;
}

static void cars_step_all(const City *c, Car *cars)
{
    for (int i = 0; i < N_CARS; i++) {
        if (cars[i].active) car_step(c, &cars[i]);
        else                car_spawn(c, &cars[i]);
    }
}

/*
 * car_glyph — directional arrow that visibly indicates which way the
 * car is heading. Stationary cars (shouldn't really happen, but for
 * completeness) get 'o'.
 */
static inline char car_glyph(const Car *car)
{
    if (car->dx > 0) return '>';
    if (car->dx < 0) return '<';
    if (car->dy > 0) return 'v';
    if (car->dy < 0) return '^';
    return 'o';
}

typedef struct {
    City    city;
    Car     cars[N_CARS];
    int     car_step_count;
    bool    paused;
    int     speed;            /* 1..SPEED_MAX                          */
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   flash_t;
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
    s->flash_t             = 1.0f;
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
 * The flash decays regardless of phase. Pause halts everything.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);
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
/* §7  screen — neighbour-aware road glyphs, twinkle, HUD                 */
/* ===================================================================== */

typedef struct {
    int cols, rows;
    int city_w, city_h;
    int gx0, gy0;            /* top-left of city on screen */
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

/*
 * road_glyph_at — pick '+' / '-' / '|' for a road cell from its
 * neighbours. Only neighbours that are ROAD AND ALREADY VISIBLE
 * (step ≤ build_step) count, so the road glyph correctly evolves
 * during the build animation: a fresh stub starts as '|' and "joins"
 * (turns to '+') once the perpendicular road catches up.
 */
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
    /* Solitary cell — happens at the very start of a road draw. */
    return '+';
}

/*
 * nb_visible — neighbour visibility check. The neighbour exists, has
 * the requested type, AND has step ≤ build_step (so it has already
 * been "built" in the animation). Used by edge detection so building
 * frames evolve correctly during the build wavefront.
 */
static inline bool nb_visible(const City *c, int x, int y,
                              int build_step, CellType want)
{
    if (x < 0 || x >= c->w || y < 0 || y >= c->h) return false;
    const Cell *n = &c->map[cidx(c, x, y)];
    return n->type == want && n->step <= build_step;
}

/*
 * park_glyph — pick one of a small per-type glyph pool by hash, so
 * a park lot reads as a textured natural patch (grass / flowers /
 * trees) instead of a flat fill.
 */
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

/*
 * building_glyph_at — pick the glyph for ONE building cell from its
 * surroundings. The four edge bits drive the choice:
 *
 *   bot_edge          → BUILDING_BASE[ci]   ('_' / '_' / '_' / '=')
 *   side_edge & !bot  → '|'
 *   interior          → either body or window glyph (per WINDOW_FREQ)
 *
 * Type 0 (low-rise houses) is rendered as a flat mix of 'n' / 'm'
 * so it reads as a row of tiny dwellings, not a solid block. Bigger
 * buildings get the framed-box treatment.
 *
 * `*lit` is set to true when the chosen glyph is a window — the
 * caller upgrades to A_BOLD so windows visibly pop against walls.
 */
static char building_glyph_at(const City *c, int x, int y,
                              int build_step, uint32_t h, bool *lit)
{
    *lit = false;
    int ci = c->map[cidx(c, x, y)].color_idx & 3;

    /* Houses (type 0) skip the frame — alternate two glyphs. */
    if (ci == 0) {
        return ((x + y) & 1) ? 'n' : 'm';
    }

    bool top_v = nb_visible(c, x, y - 1, build_step, CELL_BUILDING);
    bool bot_v = nb_visible(c, x, y + 1, build_step, CELL_BUILDING);
    bool lft_v = nb_visible(c, x - 1, y, build_step, CELL_BUILDING);
    bool rgt_v = nb_visible(c, x + 1, y, build_step, CELL_BUILDING);

    bool bot_edge  = !bot_v;
    bool side_edge = !lft_v || !rgt_v;
    /* top_edge isn't drawn specially — letting the body glyph hit the
     * road row above gives the building a clean top profile against
     * the road. (top_v is read above for symmetry; unused otherwise.) */
    (void)top_v;

    if (bot_edge)  return BUILDING_BASE[ci];
    if (side_edge) return '|';

    /* Interior — body or window. WINDOW_FREQ[ci] gives 1-in-N windows. */
    int freq = WINDOW_FREQ[ci];
    if (freq > 0 && (int)((h >> 8) % (uint32_t)freq) == 0) {
        *lit = true;
        return BUILDING_WINDOW[ci];
    }
    return BUILDING_BODY[ci];
}

/*
 * scene_draw — paint the current frame.
 *
 *   For every cell:
 *     1. Skip if step > build_step (not yet "constructed").
 *     2. Skip if EMPTY (outside the city, or pre-construction).
 *     3. ROAD cells: neighbour-aware '+' / '-' / '|', BOLD for
 *        depth ≤ 2 so the major arteries stand out.
 *     4. BUILDING cells: framed-box rendering via building_glyph_at;
 *        windows get A_BOLD; once fully built, ~3 % of windows
 *        further pulse on a per-second hash (lit-window twinkle).
 *     5. PARK cells: textured glyph pool via park_glyph.
 *   Then:
 *     6. TRAFFIC overlay (only after fully_built): each car prints
 *        a directional arrow over its current road cell.
 *     7. RESET FLASH overlay: sparse '*' fading after a rebuild.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    const City *c = &s->city;
    int  build_step  = c->build_step;
    bool fully_built = (build_step >= c->max_step);
    int  twinkle_t   = (int)s->time_secs;

    for (int y = 0; y < c->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;

        for (int x = 0; x < c->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell *cell = &c->map[cidx(c, x, y)];
            if (cell->step > build_step) continue;
            if (cell->type == CELL_EMPTY) continue;

            char glyph;
            int  attr = A_NORMAL;
            int  pair;
            uint32_t cell_hash = hash3(x, y, c->seed);

            if (cell->type == CELL_ROAD) {
                glyph = road_glyph_at(c, x, y, build_step);
                pair  = PAIR_ROAD;
                /* Bold the major arteries — depth 0..2 — so the city's
                 * primary road structure reads at a glance. */
                int road_depth = cell->step / DEPTH_STEP;
                if (road_depth <= 2) attr = A_BOLD;
            } else if (cell->type == CELL_BUILDING) {
                int ci = cell->color_idx & 3;
                pair = PAIR_BUILDING_BASE + ci;
                bool lit;
                glyph = building_glyph_at(c, x, y, build_step, cell_hash, &lit);
                if (ci >= 2 || lit) attr = A_BOLD;
                /* Window-light twinkle on lit windows only — keeps the
                 * effect localised to the sparse window cells. */
                if (fully_built && lit) {
                    uint32_t h2 = hash3(x, y, twinkle_t);
                    if ((h2 % 30u) == 0u) attr |= A_BOLD;
                }
            } else {  /* CELL_PARK */
                int ci = cell->color_idx & 3;
                pair  = PAIR_PARK_BASE + ci;
                glyph = park_glyph(ci, cell_hash);
                attr  = (ci <= 1) ? A_DIM : A_NORMAL;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* TRAFFIC overlay — only after the city is built. Each car paints
     * one directional arrow over its current road cell. */
    if (fully_built) {
        for (int i = 0; i < N_CARS; i++) {
            const Car *car = &s->cars[i];
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

    /* Reset / rebuild flash overlay — sparse '*' that fades. */
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

/*
 * screen_draw — clear, draw the city, draw the HUD.
 *   row 0 : title (left) + fps/Hz/state/speed (right)
 *   row 1 : pattern + theme + colour palettes + city size + progress
 *   bottom: key-hint strip (PAIR_HINT, A_BOLD)
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const City *c = &s->city;
    bool fully_built = (c->build_step >= c->max_step);
    const char *state_str;
    if (s->paused)            state_str = "PAUSED   ";
    else if (fully_built)     state_str = "BUILT    ";
    else                       state_str = "BUILDING ";

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
    mvprintw(0, 1, " PROCEDURAL CITY ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + palettes + size + progress. */
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
    mvprintw(1, x, " bld:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_BUILDING_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)BUILDING_BODY[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " park:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_PARK_BASE + i;
        attron(COLOR_PAIR(p));
        mvaddch(1, x, (chtype)(unsigned char)PARK_HUD_GLYPH[i]);
        attroff(COLOR_PAIR(p));
        x++;
    }
    /* Car indicator — bright white arrow showing traffic exists. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " car:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    attron(COLOR_PAIR(PAIR_CAR) | A_BOLD);
    mvaddch(1, x, '>');
    attroff(COLOR_PAIR(PAIR_CAR) | A_BOLD);
    x++;

    int pct = (c->max_step > 0)
            ? (c->build_step * 100) / c->max_step
            : 100;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  %dx%d  build:%3d%% ",
             c->w, c->h, pct);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:rebuild  q:quit ");
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
