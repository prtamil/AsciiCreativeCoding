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
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, data tables, core state types
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure decisions: no mutation, no I/O (glyph deciders)
 *   §4 SIMULATION   — advances state: the city build + the car traffic
 *   §5 EFFECTS      — cosmetic-only state (one-line note: none stored)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: woven in)
 *   §7 RENDER       — state → screen; reads only, never mutates
 *   §8 APP          — user events + per-tick combine + main loop
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
 *
 *   Generation & layout —
 *   • Lindenmayer, A. (1968) — "Mathematical models for cellular
 *     interactions in development", J. Theor. Biol. 18. The original
 *     L-system paper.
 *   • Prusinkiewicz & Lindenmayer (1990) — The Algorithmic Beauty of
 *     Plants. The canonical reference for L-system formalisms.
 *     https://algorithmicbotany.org/papers/abop/abop.pdf
 *   • Parish, Y. & Müller, P. (2001) — "Procedural Modeling of Cities",
 *     SIGGRAPH 2001. The CityEngine paper — L-system roads at scale.
 *     https://www.researchgate.net/publication/2557915
 *   • Müller, Wonka, Haegler, Ulmer & Van Gool (2006) — "Procedural
 *     Modeling of Buildings", SIGGRAPH 2006. CGA Shape grammar — the
 *     per-lot "what goes on this leaf" zoning/building-type step here.
 *   • Recursive rectangle subdivision (the BSP under the L-system framing):
 *     Wikipedia, "Binary space partitioning"
 *       https://en.wikipedia.org/wiki/Binary_space_partitioning
 *     Red Blob Games, "Dungeon generation with BSP trees" (practical)
 *       https://www.redblobgames.com/articles/dungeon-generation/
 *
 *   Rendering —
 *   • Auto-tiling by neighbour bitmask: the road glyph chooser reads
 *     N/S/E/W and draws the matching junction (+ - |) with no explicit
 *     case table. See boristhebrave.com's tileset / auto-tiling articles.
 *   • Bourke, P. (1997) — "Character representation of grey scale images".
 *     The density-ramp idea behind the h/H/#/@ building-height glyphs.
 *     http://paulbourke.net/dataformats/asciiart/
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

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants + const data tables
 *                    + the core state types (Cell/City/Car), relocated up so
 *                    the pure §3 helpers can see them.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — pure reads/decisions: the hash + index, the
 *                    GLYPH DECIDERS (road_glyph_at, building_glyph_at,
 *                    park_glyph, car_glyph, nb_visible, cell_appearance), and
 *                    the SUBDIVISION DECIDERS (choose_split_axis,
 *                    pick_split_pos, lot_color_idx, pattern_name). They only
 *                    READ (the City grid or plain args), so they live here, not
 *                    in §4/§7, and no RENDER/EFFECTS reordering can corrupt them.
 *   SIMULATION   §4  City.{map,max_step,build_step,hold_countdown,seed,
 *                    pattern_built} and Scene.{cars,car_step_count,time_secs,
 *                    paused,speed,current_theme,current_pattern}. Two advancing
 *                    sub-systems: the city build (subdivide → animate) and the
 *                    car traffic (a cosmetic-but-stateful sim). Only writers.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; the window twinkle is
 *                    render-derived. One-line section, not a real layer.
 *   DELAYS       §6  (none) — only City.hold_countdown + Scene.paused, handled
 *                    inside scene_tick (§4). One-line section.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (theme_apply
 *                    / color_init, scene_draw + draw_traffic, and screen_draw
 *                    with its draw_status_line / draw_param_line / draw_swatch
 *                    / draw_hint helpers). Reads §4 state via the §3 deciders;
 *                    never writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (city + cars) + DELAYS
 *     screen_draw() ; screen_present()    // RENDER (reads only; twinkle derived)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state. User events
 * (app_handle_key / app_do_resize) mutate Scene/Screen on a keypress or
 * SIGWINCH, but run once per frame OUTSIDE the accumulator loop.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, data tables, core state types               */
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

/* Split-axis aspect thresholds (height·ASPECT_Y_F / width). Above HI the block
 * is clearly tall → split horizontally; below LO clearly wide → split
 * vertically; between, a hash bit breaks the tie. */
#define SPLIT_ASPECT_HI 1.4f
#define SPLIT_ASPECT_LO 0.7f

/* PARKS pattern: probability that a lot becomes a park rather than
 * a building, expressed as "1 in N". */
#define PARK_DENOM      4
/* …but never inside this radius of the centre (fraction of max radius) — keep
 * a built-up downtown. */
#define PARK_MIN_RADIUS 0.30f

/* Roads at subdivision depth <= this are "major arteries", drawn bold. */
#define MAJOR_ROAD_MAX_DEPTH 2
/* Window-light twinkle: 1 in N lit windows flips bold on a per-second hash. */
#define TWINKLE_1_IN    30
/* Car spawn: give up after this many random-cell tries (tiny maps). */
#define CAR_SPAWN_TRIES 200

/* ── Building glyph tables ─────────────────────────────────────────────── *
 * Buildings are drawn as FRAMED BOXES, not solid fills, so the eye reads
 * actual structures — side walls, a foundation, an interior dotted with lit
 * windows:
 *
 *       _____
 *      |HH"H|     ← top row: just below the road
 *      |HHHH|     ← interior body, occasional '"' = lit window
 *      |____|     ← bottom row foundation, side walls '|'
 *
 * KEY IDEA  These three arrays are PARALLEL and share ONE index: the building
 * "height tier" 0..3. That index is exactly Cell.color_idx — the same number
 * also picks the colour (Theme.building[i]). So one small int, stamped at
 * generation, drives the whole look: tier → {body, window, base} glyph + tint.
 * building_glyph_at (§3) reads neighbours to choose base vs side-wall vs
 * interior, then indexes these by tier; no per-cell role is stored.
 *
 * VALUE LOGIC (per tier i):
 *   BODY    n < H < # < @  — a visual DENSITY ramp (sparse house → solid tower).
 *   WINDOW  the glyph used for a lit interior cell ('"' on apartments, '*' on
 *           the brighter office/skyscraper). BODY[0]/WINDOW[0] ('n'/'m') are the
 *           house mix — houses (tier 0) skip the frame entirely, so they render
 *           straight from BODY/('n'/'m'); WINDOW[0]='m' is therefore NEVER read
 *           (kept only to keep the arrays index-aligned).
 *   BASE    foundation row: '_' for everything except the skyscraper's heavier
 *           '=' — a tiny landmark cue that the tallest tier sits on a slab. */
static const char BUILDING_BODY  [4] = { 'n', 'H', '#', '@' };
static const char BUILDING_WINDOW[4] = { 'm', '"', '*', '*' };
static const char BUILDING_BASE  [4] = { 'n', '_', '_', '=' };

/* WINDOW_FREQ[i] — window density per tier: 1 interior cell in N becomes a lit
 * window. SMALLER N = MORE windows, so the values invert the build order on
 * purpose: skyscraper(3) and office(4) glitter, apartment(8) stays mostly
 * solid like a townhouse. Slot 0 (=2) is DEAD — houses never reach the window
 * branch (see BODY note above); it exists only to keep the index aligned. */
static const int  WINDOW_FREQ   [4] = {   2,   8,   4,   3 };

/* ── Park glyph palette ────────────────────────────────────────────────── *
 * Parks reuse the SAME tier index (0..3 = Cell.color_idx = Theme.park[i]) as
 * buildings, but here the index means a vegetation density, not a height. The
 * actual per-cell glyph is NOT a table lookup — park_glyph (§3) cycles a small
 * pool by a per-cell hash so a lot reads as a textured natural patch (grass +
 * flowers + trees) instead of a flat fill; the pools are, by tier:
 *
 *   0 light grass : ',' '.'
 *   1 garden      : ',' '.' '\''
 *   2 wooded park : ',' 'Y' '.'        (Y = stylised tree)
 *   3 dense forest: 'Y' 'T' '*'        (T = trunk)            sparser → denser
 *
 * PARK_HUD_GLYPH below holds just the single most-distinctive glyph per tier —
 * used ONLY for the HUD palette swatch, never for in-city rendering (that's
 * park_glyph's job). It exists so the legend shows one recognisable icon. */
static const char PARK_HUD_GLYPH[4] = { ',', '.', 'Y', 'T' };

/* ── Pattern ───────────────────────────────────────────────────────────── *
 * The four city "styles". They are NOT four algorithms — they are four
 * parameter sets for the SAME recursive subdivision (subdivide, §4). Each one
 * flips a couple of knobs, so the enum value threads through subdivide() and
 * place_lot() as `p` and selects behaviour at the decision points:
 *   GRID      splits land at exact centre (split_pos = mid) → a strict
 *             Manhattan grid of equal blocks.
 *   ORGANIC   split position jitters ±SPLIT_JITTER from centre → irregular
 *             block sizes, an "old-European" feel; building tints are random.
 *   DISTRICTS ORGANIC layout, but building TYPE follows a zoning gradient
 *             (lot_color_idx maps distance-from-centre → skyscraper..house).
 *   PARKS     DISTRICTS layout, plus ~1/PARK_DENOM of the outer lots become
 *             green spaces instead of buildings.
 * The order is cumulative: each adds a rule on top of the previous, which is
 * why DISTRICTS/PARKS share code paths. REFS: file-header References block. */
typedef enum {
    PATTERN_GRID      = 0,
    PATTERN_ORGANIC   = 1,
    PATTERN_DISTRICTS = 2,
    PATTERN_PARKS     = 3,
    N_PATTERNS        = 4,
} Pattern;

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * WHAT  One named colour palette (10 ship; t cycles). theme_apply() loads it
 *       into the ncurses pairs PAIR_ROAD / PAIR_BUILDING_BASE+i / PAIR_PARK_BASE+i.
 *       Same 10-name menu as the sibling procedural showcases, for muscle memory.
 *
 * VALUE LOGIC  building[0..3] is a RAMP ordered by building "height" (house →
 *       apartment → office → skyscraper), conventionally cool→warm/dim→bright so
 *       the skyline reads as density; the index i is exactly Cell.color_idx, so
 *       a denser lot picks a brighter tint automatically. park[0..3] is the
 *       parallel green/accent ramp for park types. road is a single gray-ish
 *       asphalt tint. All are 256-colour codes; background is the terminal
 *       default (-1). REFS: project palette notes — documentation/COLOR.md. */
typedef struct {
    const char *name;       /* HUD label (t cycles)                          */
    short       road;       /* asphalt tint (one, shared by all roads)       */
    short       building[4];/* tint per height tier, indexed by Cell.color_idx*/
    short       park    [4];/* tint per park type, same indexing             */
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

/* CellType — what a map cell IS. EMPTY is the pre-build / outside-the-city
 * default; the other three are the terminal-block interpretations the L-system
 * produces (a road line, or a lot filled with buildings or a park). */
typedef enum {
    CELL_EMPTY    = 0,
    CELL_ROAD     = 1,
    CELL_BUILDING = 2,
    CELL_PARK     = 3,
} CellType;

/* ── Cell ──────────────────────────────────────────────────────────────── *
 * WHAT  One map square — the leaf of the whole design. The subdivision writes
 *       Cells; render reads them. There is NO road graph and NO lot list: every
 *       per-cell question (road? what tint? when does it appear?) is answered
 *       by reading exactly one Cell, so the generator allocates nothing.
 *
 * WHY 4 BYTES  Packed to type(1)+color_idx(1)+step(2) = 4 bytes so the full
 *       240×80 map is ~75 KB and stays in L2 cache — the render loop sweeps
 *       every cell each frame, so locality matters more than convenience.
 *
 * VALUE LOGIC
 *   type       a CellType (stored as uint8_t to keep the struct tight).
 *   color_idx  0..3 sub-tint, indexing Theme.building[] or Theme.park[]; for
 *              buildings it doubles as the height tier (house..skyscraper).
 *   step       the ANIMATION THRESHOLD — the cell is drawn iff step <=
 *              City.build_step. subdivide() stamps each cell with the build
 *              "time" at which it should appear (roads of depth d get the low
 *              half of [d·DEPTH_STEP, …], lots the upper half), so simply
 *              ramping build_step replays the L-system derivation as a growth
 *              animation — no frame buffer of past states needed. uint16_t
 *              holds the max ~10 000 comfortably. */
typedef struct {
    uint8_t  type;       /* CellType                                       */
    uint8_t  color_idx;  /* 0..3 tint / building-height tier               */
    uint16_t step;       /* visible iff step <= City.build_step (wavefront)*/
} Cell;

/* ── City ──────────────────────────────────────────────────────────────── *
 * WHAT  The terrain: a w×h grid of Cells (row-major, index via cidx) PLUS the
 *       state of the build animation playing over it. This is the one domain
 *       object — SIMULATION (§4) is its only writer, LOGIC/RENDER read it.
 *
 * WHY A FLAT ARRAY  Static CITY_CELLS_MAX backing store, never malloc'd: the
 *       L-system writes into it and the next rebuild overwrites in place (the
 *       "no allocation in steady state" rule).
 *
 * VALUE LOGIC
 *   w, h            grid extent (cells); clamped to MAP_W_MAX×MAP_H_MAX.
 *   map[]           the cells (see Cell); the derivation tree's leaves.
 *   max_step        highest Cell.step any cell got — the finish line the build
 *                   animation ramps build_step up to.
 *   build_step      the animation wavefront: cells with step <= build_step are
 *                   "constructed" and drawn. 0 = blank, max_step = fully built.
 *   hold_countdown  ticks left to display the finished city before rebuilding
 *                   (DELAYS, §6); armed to HOLD_TICKS_DEF on completion.
 *   seed            the RNG seed this layout was grown from (also drives the
 *                   per-cell twinkle hash so it's stable within a generation).
 *   pattern_built   which Pattern produced THIS layout — recorded so a render
 *                   never disagrees with the geometry on screen.
 *
 * REFS  L-system framing — Lindenmayer (1968), Prusinkiewicz & Lindenmayer
 *       (1990); city road subdivision — Parish & Müller (2001). The geometry is
 *       a binary space partition (BSP) of the rectangle. See file-header refs. */
typedef struct {
    int   w, h;               /* grid extent (cells)                        */
    Cell  map[CITY_CELLS_MAX];/* the cell grid, row-major                   */
    int   max_step;           /* highest Cell.step — the build finish line  */
    int   build_step;         /* animation wavefront; cells <= it are drawn */
    int   hold_countdown;     /* ticks remaining once build completes       */
    int   seed;               /* RNG seed this layout was grown from        */
    Pattern pattern_built;    /* the pattern THIS layout was generated for  */
} City;

/* ── Car ───────────────────────────────────────────────────────────────── *
 * WHAT  One traffic dot — a minimal agent that walks the road network for
 *       decoration once the city is BUILT. N_CARS of them give the static plan
 *       a sense of life. They read the City (road_at) but never modify it; they
 *       are the cosmetic side of SIMULATION (§4), not part of the city geometry.
 *
 * BEHAVIOUR  Greedy road-follower (car_step): keep the current heading if the
 *       next cell is road; ~1/8 of the time take a perpendicular turn at an
 *       intersection; if blocked, pick any non-reversing road neighbour; if
 *       truly boxed in, deactivate and respawn elsewhere next tick. No global
 *       pathfinding — emergent traffic from a per-car local rule.
 *
 * VALUE LOGIC
 *   x, y    integer cell position on the City grid.
 *   dx, dy  heading: exactly one is ±1, the other 0 (a cardinal direction);
 *           car_glyph maps it to > < ^ v.
 *   active  true while on a valid road path; false marks "respawn me" so a
 *           stranded car never freezes a glyph on screen. */
typedef struct {
    int  x, y;     /* cell position                                 */
    int  dx, dy;   /* cardinal heading (one is ±1, the other 0)     */
    bool active;   /* false → stranded, respawn next tick           */
} Car;

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The 60 fps frame cap and the fixed-timestep
 * accumulator that decides how many scene_tick()s run per frame are POLICY,
 * applied in main() (§8). */

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
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Pure reads / decisions: each returns a value from its arguments (and the
 * read-only City grid + const tables) with NO mutation and NO I/O. Deleting
 * or reordering any RENDER/EFFECTS code cannot change what these return, so
 * they are corruption-proof by construction. Two families live here because
 * they only READ: the GLYPH deciders that §7 render uses (road_glyph_at,
 * building_glyph_at, cell_appearance, ...) and the SUBDIVISION deciders that
 * §4 generation uses (choose_split_axis, pick_split_pos, lot_color_idx). */

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

static inline int cidx(const City *c, int x, int y) { return y * c->w + x; }

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

static inline bool road_at(const City *c, int x, int y)
{
    if (x < 0 || x >= c->w || y < 0 || y >= c->h) return false;
    return c->map[cidx(c, x, y)].type == CELL_ROAD;
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

/* Which axis to cut a block on: always split the LONGER side so sub-blocks stay
 * roughly square on the aspect-corrected grid; when neither side dominates, a
 * hash bit breaks the tie. Returns true to split horizontally (a road row). */
static bool choose_split_axis(bool can_h, bool can_v, float aspect, uint32_t hash)
{
    if (!can_h)                   return false;
    if (!can_v)                   return true;
    if (aspect > SPLIT_ASPECT_HI) return true;    /* clearly tall → cut across */
    if (aspect < SPLIT_ASPECT_LO) return false;   /* clearly wide → cut down   */
    return (hash & 1u) != 0u;                      /* ~square → coin flip       */
}

/* Where along [lo,hi] to place the dividing road. Exact centre for GRID (and
 * whenever the band is too thin to jitter); otherwise offset from centre by a
 * hash-driven amount up to ±SPLIT_JITTER of the band, then clamped into [lo,hi]
 * so each half can still host a MIN_LOT. */
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

/* The glyph + colour pair + attribute for ONE already-built, non-empty cell,
 * decided purely from its type and neighbours. Returns the glyph and writes the
 * pair / attribute through *pair / *attr. Pure read — the per-cell appearance
 * rule, lifted out of the scene_draw loop so that loop reads as pseudocode. */
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
        /* Bold the major arteries so the primary road structure reads. */
        int road_depth = cell->step / DEPTH_STEP;
        if (road_depth <= MAJOR_ROAD_MAX_DEPTH) *attr = A_BOLD;
    } else if (cell->type == CELL_BUILDING) {
        int ci = cell->color_idx & 3;
        *pair = PAIR_BUILDING_BASE + ci;
        bool lit;
        glyph = building_glyph_at(c, x, y, build_step, cell_hash, &lit);
        if (ci >= 2 || lit) *attr = A_BOLD;
        /* Window-light twinkle on lit windows only (sparse, localised). */
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

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (only writers of sim state)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. Two sub-systems advance here:
 *  • the CITY build — city_build/subdivide/place_* write the Cell grid once,
 *    then scene_tick ramps City.build_step to animate it appearing;
 *  • the CARS — a cosmetic-but-STATEFUL traffic sim (car_step walks each car
 *    along the road graph). Cars are decorative, but they ADVANCE STATE, so
 *    they belong here, not in EFFECTS (§5).
 * Mutates: City.{map,max_step,build_step,hold_countdown,seed,pattern_built};
 * Scene.{cars,car_step_count,time_secs,paused,speed,current_theme,
 * current_pattern}. scene_tick() is the single per-tick entry point (called
 * only from main, §8); user events (key/resize) also mutate Scene but are NOT
 * part of the tick -- see §8. */

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

    /* Terminal production  B → Lot:  too deep, or too small to split. */
    if (depth >= MAX_DEPTH || (!can_split_h && !can_split_v)) {
        place_lot(c, x0, y0, x1, y1, depth, seed, p);
        return;
    }

    /* Split production  B → road + two sub-blocks: choose the axis (longer
     * side; aspect corrected for tall cells), then the road position. */
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

/*
 * car_spawn — place a car at a random road cell with a random
 * direction. If no road cell is found in CAR_SPAWN_TRIES tries (very small
 * map), the car is left inactive — it will respawn next tick.
 */
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

/*
 * car_step — advance one car along the road network. The driver
 * prefers to keep going forward; at intersections it occasionally
 * turns; at dead ends it picks any valid neighbour. Stranded cars
 * (no road neighbour at all) deactivate and respawn next tick.
 */
static void car_step(const City *c, Car *car)
{
    int nx = car->x + car->dx;          /* the cell straight ahead */
    int ny = car->y + car->dy;

    /* 1. At an intersection, occasionally turn (1 in 8). The two candidates
     *    are the left/right perpendiculars to the current heading. */
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

    /* 2. Drive straight if the road continues. */
    if (road_at(c, nx, ny)) {
        car->x = nx; car->y = ny;
        return;
    }

    /* 3. Forward blocked → take any road neighbour, preferring not to reverse
     *    (yo-yoing cars look bad); if none exists, mark stranded to respawn. */
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
