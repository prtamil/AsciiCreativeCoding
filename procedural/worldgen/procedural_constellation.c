/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_constellation.c
 *   — Procedural constellation generator: place anchor stars in the
 *     sky, connect them with one of four canonical graph topologies,
 *     give the result a Latin-flavoured procedural name, and animate
 *     the "discovery" star-by-star and line-by-line.
 *
 * DEMO: A faint dust of background stars fills the screen. After a
 *       moment, brighter "anchor" stars wink into existence in a
 *       loose cluster. Once they are all lit, glowing dotted LINES
 *       trace out between them — connecting the stars into a single
 *       graph. The shape of that graph depends on the active pattern:
 *
 *         TREE    minimum-spanning-tree on the anchor stars
 *                 (organic branching shape — Orion-like)
 *         CHAIN   anchor stars sorted left-to-right and connected
 *                 sequentially (Cassiopeia / Big Dipper-like)
 *         LOOP    polar-sorted around the cluster centroid and
 *                 connected in a cycle (Pegasus / Crown-like)
 *         SPOKE   one central star plus radial spokes to all others
 *                 (Crux / simple-cross-like)
 *
 *       When the trace finishes, a procedurally-generated name
 *       ("Aurelia Major", "Lyrenor", "Cygnara Borealis", ...) fades
 *       in below the figure. After a brief HOLD, a flash sweeps the
 *       sky and the next constellation begins to discover itself.
 *
 * Study alongside:
 *   ../worldgen/procedural_galaxy.c
 *      — also stars, but a dense FIELD of them rendered from a
 *        density function. This file is the opposite: a SPARSE GRAPH
 *        of named anchor stars connected by deliberate edges.
 *   ../fractal_random/lightning.c
 *      — same Bresenham line-rasterisation idea applied to a single
 *        animated bolt of lightning instead of a static graph.
 *
 * Section map:
 *   §1 config        — themes, name fragments, animation timings
 *   §2 clock         — monotonic timer + sleep
 *   §3 color         — HUD reserved + 10 themes (4 star tints + line)
 *   §5 constellation — hash, point placement, topology builders,
 *                      MST (Prim's), Bresenham, name generator
 *   §6 scene         — phase machine, reveal animation
 *   §7 screen        — backdrop sky, anchor stars, edges, name, HUD
 *   §8 app           — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the discovery animation
 *   r          regenerate constellation with a new seed
 *   n / N      next pattern  (TREE → CHAIN → LOOP → SPOKE → ...)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster reveal
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/procedural_constellation.c \
 *     -o constellation -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three pieces composed together.
 *
 *                  (1) POINT PLACEMENT — a Poisson-ish jittered grid.
 *                      Subdivide a rectangular sky region into roughly
 *                      √N × √N cells; place ONE star in each cell at a
 *                      hash-driven offset from the cell centre. Result:
 *                      stars look organically scattered (no obvious
 *                      grid) but the minimum inter-star distance is
 *                      bounded below — none of the stars cluster in one
 *                      corner. This is the same trick stippling
 *                      algorithms use to avoid white-noise clumping.
 *
 *                  (2) GRAPH TOPOLOGY — the constellation "shape" is
 *                      really just an edge set over the anchor stars.
 *                      Four canonical edge-selection rules, each
 *                      corresponding to a real-world constellation
 *                      morphology:
 *
 *                        TREE  : Minimum Spanning Tree via Prim's
 *                                algorithm. N-1 edges, connected,
 *                                no cycles. Mimics branched
 *                                constellations (Orion, Hercules).
 *                        CHAIN : sort by x, connect sequentially.
 *                                N-1 edges arranged left-to-right.
 *                                Mimics Cassiopeia / Big Dipper.
 *                        LOOP  : sort by polar angle around the
 *                                centroid, connect into a cycle.
 *                                N edges forming a closed polygon.
 *                                Mimics Pegasus / Corona.
 *                        SPOKE : pick the star nearest the centroid;
 *                                connect every other star to it.
 *                                N-1 edges all meeting at one hub.
 *                                Mimics Crux / simple-cross figures.
 *
 *                  (3) BRESENHAM LINE RASTERISATION — to draw an edge
 *                      between two integer cells we walk every cell
 *                      that lies along the line using Jack Bresenham's
 *                      1965 algorithm. The animated "trace" effect is
 *                      just a partial Bresenham: at reveal_t ∈ [0,1]
 *                      we plot the first ⌈reveal_t · steps⌉ cells.
 *                      The glyph at each cell is chosen from the
 *                      segment's overall direction:
 *                         dx ≫ dy → '-'    horizontal segment
 *                         dy ≫ dx → '|'    vertical segment
 *                         same-sign dx, dy → '\\'   down-right or up-left
 *                         opp-sign         → '/'    down-left or up-right
 *
 *                  Plus PROCEDURAL NAMING — pick one of N prefix roots
 *                  ("Auri", "Lyr", "Cygn"...) plus one of M suffixes
 *                  ("us", "ina", "ax"...) plus one of K modifiers
 *                  ("", "Major", "Borealis"...) all from the same
 *                  hash. Yields ~5000 unique names with constellation
 *                  flavour without any list of full names.
 *
 * Data-structure : Tiny — at most 12 stars and 24 edges in fixed
 *                  arrays. Stars have (x, y, twinkle_phase, reveal_t).
 *                  Edges have (from, to, reveal_t). Plus a 32-byte
 *                  name buffer. Total state per constellation: a few
 *                  hundred bytes. No allocations, no heap.
 *
 * Rendering      : ASCII only. Two layers:
 *                    - BACKDROP : every screen cell rolls a hash; if
 *                                 mod BG_DENSITY == 0 paint a faint
 *                                 '.' or '`' star. Twinkle modulated
 *                                 per-cell by time so the field shimmers.
 *                    - CONSTELLATION : revealed anchor stars as bright
 *                                 '*' / 'O' (BOLD); revealed edges as
 *                                 a Bresenham trail of '-' '|' '/' '\\'
 *                                 in the line tint (DIM); name label
 *                                 in a dedicated bright tint.
 *
 * Performance    : The whole rendering is O(W·H) backdrop hashing +
 *                  O(N²) MST + a few hundred Bresenham steps per
 *                  edge. Trivial — <1 ms / frame at 240×80×60 fps.
 *                  No per-tick allocation.
 *
 * References     :
 *   • Bresenham, J. (1965) — "Algorithm for computer control of a
 *     digital plotter", IBM Systems Journal 4(1):25-30.
 *     https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *   • Prim, R. (1957) — "Shortest connection networks and some
 *     generalizations", Bell System Tech. J. 36(6).
 *     https://en.wikipedia.org/wiki/Prim%27s_algorithm
 *   • Wikipedia — Constellation
 *     https://en.wikipedia.org/wiki/Constellation
 *   • Wikipedia — Poisson-disk sampling (point-placement intuition)
 *     https://en.wikipedia.org/wiki/Supersampling#Poisson_disc
 *   • Lloyd, S. (1982) — Least squares quantization (the dual of
 *     jittered-grid placement).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A constellation is two things and only two things: a small set of
 * points scattered in the sky, and a set of edges that someone — long
 * ago, around a fire — drew between those points to make a picture.
 * The points come first; the edges come second; the picture is
 * imaginary. Procedurally, we generate the points (jittered grid),
 * pick the edges (one of four rules), draw the lines (Bresenham),
 * and slap on a name. There is nothing else.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine punching ten holes through a sheet of black paper, then
 * drawing pencil lines BETWEEN selected pairs of holes. The holes are
 * the stars; the pencil lines are the constellation. Different
 * constellations differ ONLY in (a) where you punched the holes and
 * (b) which pairs you chose to connect with pencil. Both are choices;
 * neither is intrinsic to the stars.
 *
 * Now: replace "punch holes" with "place jittered points in a grid",
 * and replace "draw pencil lines" with one of four small algorithms.
 * That is the entire generator. The reason the result LOOKS like a
 * real constellation is that real constellations were drawn by humans
 * subject to the same constraints: stars roughly evenly distributed
 * across the visible sky, and edges chosen for visual simplicity
 * (no crossings, no long jumps, every star connected).
 *
 * Why FOUR topologies? Real constellations cluster into morphological
 * families:
 *   - branchy ones (Orion, Hercules)            → MST
 *   - linear ones (Big Dipper, Cassiopeia's W)  → CHAIN
 *   - closed ones (Pegasus square, Corona)      → LOOP
 *   - radial ones (Crux, the Cross)             → SPOKE
 * Picking one and applying it consistently produces a recognisable
 * constellation flavour every time.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. PICK N. Random integer in [5, 10] depending on pattern.
 *  2. PLACE N STARS in a region of size REG_W × REG_H:
 *     a. cols = round(√N), rows = ⌈N / cols⌉
 *     b. cell_w = REG_W / cols,  cell_h = REG_H / rows
 *     c. for each cell (c, r) in row-major order, until N stars:
 *           h = hash3(c, r, seed)
 *           jitter_x = ((h        & 0x3FF) − 512) · cell_w / 2048
 *           jitter_y = ((h >> 10) & 0x3FF) − 512) · cell_h / 2048
 *           star.x = c·cell_w + cell_w/2 + jitter_x
 *           star.y = r·cell_h + cell_h/2 + jitter_y
 *  3. BUILD EDGES per pattern:
 *       TREE  : Prim's. Maintain a "in_tree[]" bitset; repeatedly add
 *               the cheapest cross-edge until N-1 edges placed.
 *       CHAIN : sort stars by x; emit edges 0-1, 1-2, …, (N-2)-(N-1).
 *       LOOP  : compute centroid; sort stars by atan2(y-cy, x-cx);
 *               emit edges 0-1, 1-2, …, (N-1)-0.
 *       SPOKE : compute centroid; pick hub = nearest star to centroid;
 *               emit one edge from hub to every other star.
 *  4. NAME the constellation:
 *       prefix    = PREFIXES[hash(seed, 1) mod N_P]
 *       suffix    = SUFFIXES[hash(seed, 2) mod N_S]
 *       modifier  = MODIFIERS[hash(seed, 3) mod N_M]
 *       name      = prefix ++ suffix ++ (" " ++ modifier  if non-empty)
 *  5. ANIMATE in four phases:
 *       PHASE_DRAW_STARS : reveal stars one by one, fade-in over
 *                          PHASE_STARS_TOTAL seconds.
 *       PHASE_DRAW_EDGES : reveal edges in order, each over
 *                          PHASE_EDGE_TIME seconds. Per-edge
 *                          reveal_t ∈ [0,1] drives the partial
 *                          Bresenham plot.
 *       PHASE_HOLD       : freeze the figure, fade in the name.
 *       PHASE_FADE       : sky-wide flash, then regenerate.
 *
 * KEY FORMULAS
 * ────────────
 *  Bresenham (one cell per step, integer-only):
 *     dx, dy = |x1-x0|, |y1-y0|
 *     sx, sy = sign(x1-x0), sign(y1-y0)
 *     err    = dx − dy
 *     each step:
 *       plot(x, y)
 *       if 2·err >= -dy: err -= dy; x += sx
 *       if 2·err <=  dx: err += dx; y += sy
 *     stop when (x, y) == (x1, y1)
 *
 *  Animated partial Bresenham:
 *     total_steps = max(|dx|, |dy|) + 1
 *     step_limit  = ⌈reveal_t · total_steps⌉
 *
 *  Line-segment glyph from segment direction (dx, dy):
 *       |dx| ≥ 2·|dy|              → '-'
 *       |dy| ≥ 2·|dx|              → '|'
 *       sign(dx) == sign(dy)       → '\\'
 *       sign(dx) != sign(dy)       → '/'
 *
 *  Prim's MST (best-cross-edge until tree spans all N):
 *     while tree.size < N:
 *       pick (i, j) with i in tree, j not in tree, minimising d(i,j)²
 *       add j to tree; emit edge (i, j)
 *
 *  Polar sort key for LOOP topology:
 *     θ_i = atan2(y_i − cy, x_i − cx)         // centroid (cx, cy)
 *     sort stars[] ascending by θ_i           // counterclockwise cycle
 *
 *  Procedural name:
 *     name = prefix[hash%N_P]  ⊕  suffix[hash%N_S]  ⊕  modifier[hash%N_M]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • REGION TOO SMALL. If the constellation region is narrower than
 *    cols, the jittered-grid placement collapses (cell_w < 1) and
 *    multiple stars overlap. Clamp REG_W ≥ 16 and REG_H ≥ 8 in the
 *    layout step; on tiny terminals this means the constellation
 *    region grows to fill the whole sky, which is fine.
 *
 *  • DUPLICATE STAR POSITIONS. With heavy jitter two stars in
 *    adjacent cells COULD land on the same (x, y). The MST is robust
 *    to that (zero-distance edges just fold the duplicates), but the
 *    rendering double-draws which looks weird. Reduce JITTER_FRAC
 *    below 0.5 so adjacent cells don't overlap.
 *
 *  • PRIM'S TIE-BREAKING. When two cross-edges have identical
 *    distance², the loop picks the first one found. That makes the
 *    MST seed-dependent in subtle ways (insertion order). It's
 *    deterministic — same seed, same MST — and that's all we need.
 *
 *  • LOOP WITH N=2. The polygon collapses to two points joined by
 *    two coincident edges. Skip LOOP for very small N.
 *
 *  • SORTING IN PLACE. CHAIN sorts stars[] by x, LOOP sorts by angle.
 *    The edge indices reference positions IN THE SORTED array, so
 *    the order matters. Don't re-sort after edge construction.
 *
 *  • LINE OFF-SCREEN. Bresenham can step into cells that are out of
 *    the renderable region (HUD rows, off-screen). Always bounds-
 *    check before mvaddch — the algorithm doesn't know about the HUD.
 *
 *  • REVEAL_T = 1.0 OFF-BY-ONE. ⌈1·total_steps⌉ might be one fewer
 *    than total_steps when total_steps is small and reveal_t is very
 *    slightly < 1. Let the fully-reached state plot ALL cells by
 *    using > 0.999 as the "complete" guard.
 *
 *  • NAME OVERFLOW. snprintf truncates if prefix+suffix+modifier is
 *    longer than the name buffer. Size the buffer to comfortably hold
 *    the longest combination of the largest fragment lengths.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space) during DRAW_EDGES: a partially-traced edge freezes
 *    mid-draw. Resume: the edge continues from exactly where it
 *    paused. Verifies the fixed-step accumulator + reveal_t logic.
 *
 *  • Press 'r': sky flashes, regenerate. The new constellation has a
 *    DIFFERENT name and DIFFERENT star arrangement, but the topology
 *    family is the same as the active pattern.
 *
 *  • TREE pattern: count edges. Should be exactly (n_stars − 1). The
 *    edge graph should be CONNECTED (every star reachable from every
 *    other via edges) but ACYCLIC (no closed loops).
 *
 *  • CHAIN pattern: edges should run left-to-right with no crossings,
 *    each edge connecting horizontally-adjacent stars in the sort order.
 *
 *  • LOOP pattern: count edges = n_stars exactly. The edges should
 *    form a closed polygon (no break, no fork, every star is endpoint
 *    of exactly two edges).
 *
 *  • SPOKE pattern: one star is the endpoint of EVERY edge. That hub
 *    star should be visually near the centroid of the cluster.
 *
 *  • NAMING: trying many seeds should produce many different names
 *    with constellation flavour ("Aurelia", "Lyrenus", "Pegonor")
 *    rather than gibberish or repeats. With ~30 prefixes × 16
 *    suffixes × 10 modifiers, expect <0.05% repeat rate over 100 trials.
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
    /* Maximum number of anchor stars in a constellation. 12 covers
     * even Orion's 17-star canonical figure compactly enough for
     * an ASCII showcase. Larger N starts to look noisy. */
    N_STARS_MAX         =  12,
    N_EDGES_MAX         =  24,

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Background-star density: 1 in N cells in the sky carries a
     * faint star. 60 → ~1.7 % of the sky; subtle sprinkle. */
    BG_STAR_DENSITY     =  60,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_BG_STAR        =   3,    /* dim background sprinkle         */
    PAIR_LINE           =   4,    /* constellation lines             */
    PAIR_STAR_BASE      =   5,    /* +0..+3 = 4 anchor-star tints    */
    PAIR_NAME           =   9,    /* constellation name              */
    PAIR_FLASH          =  10,    /* phase-transition flash          */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * Animation timings (in seconds). The "speed" knob multiplies these
 * inversely — speed=16 doubles the rate, speed=4 halves it.
 */
#define PHASE_STARS_TOTAL    1.6f      /* full reveal of all anchor stars */
#define PHASE_EDGE_TIME      0.30f     /* time to draw one edge           */
#define PHASE_HOLD_TIME      6.0f      /* dwell once finished             */
#define PHASE_FADE_TIME      0.6f      /* flash transition before regen   */

/*
 * Jittered-grid star placement: maximum offset from cell centre as a
 * fraction of cell size. 0.40 keeps adjacent cells from overlapping
 * but feels organic.
 */
#define JITTER_FRAC          0.40f

/*
 * Pattern — four edge-selection rules over the same anchor-star set.
 */
typedef enum {
    PATTERN_TREE  = 0,
    PATTERN_CHAIN = 1,
    PATTERN_LOOP  = 2,
    PATTERN_SPOKE = 3,
    N_PATTERNS    = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_TREE:  return "TREE ";
    case PATTERN_CHAIN: return "CHAIN";
    case PATTERN_LOOP:  return "LOOP ";
    case PATTERN_SPOKE: return "SPOKE";
    default:            return "?    ";
    }
}

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 *   bg_star : tint of the faint sky-sprinkle
 *   line    : tint of the constellation lines
 *   star[4] : four anchor-star tints (cycled per-star by hash)
 *   name    : tint of the constellation name label
 */
typedef struct {
    const char *name;
    short       bg_star;
    short       line;
    short       star[4];
    short       name_color;
} Theme;

#define N_THEMES 10

/*
 * Tuned for legibility on dark terminals: every entry sits in the
 * brighter half of the 256-colour cube so even A_DIM cells stay
 * readable.  bg_star is theme-tinted (not generic dark gray) so
 * each backdrop carries its theme's character — green dust for
 * MATRIX, purple haze for NOVA, etc.  star[3] (the dimmest anchor
 * tint) avoids the deep-shadow indices that previously vanished
 * against a default-black terminal background.
 */
static const Theme themes[N_THEMES] = {
    /* name        bg   line  star{0,1,2,3}              name_col */
    { "DEFAULT",   245, 252, { 226, 230, 159, 117 }, 226 },
    { "MATRIX",     65, 118, { 230, 226, 154, 118 }, 154 },
    { "NOVA",       97, 213, { 231, 219, 207, 177 }, 219 },
    { "MONO",      247, 253, { 255, 252, 248, 244 }, 254 },
    { "OCEAN",      67, 117, { 231, 195, 159,  87 }, 159 },
    { "FIRE",      131, 214, { 231, 226, 215, 209 }, 226 },
    { "EARTH",     137, 223, { 231, 230, 222, 179 }, 230 },
    { "FOREST",     66, 114, { 231, 192, 156, 120 }, 156 },
    { "DESERT",    180, 223, { 231, 230, 223, 215 }, 230 },
    { "ARCTIC",    153, 195, { 231, 219, 195, 159 }, 195 },
};

/*
 * Procedural name fragments. The combinatorial space —
 *   ~33 prefixes × 16 suffixes × 10 modifiers  ≈ 5 280 combinations
 * — is large enough that consecutive seeds rarely collide.
 *
 * Roots are clipped so the prefix+suffix concatenation reads as a
 * single Latin-flavoured word (e.g., "Lyr" + "ax" → "Lyrax",
 * "Cygn" + "us" → "Cygnus").
 */
static const char *PREFIXES[] = {
    "Auri", "Lyr",   "Dracon","Cygn",  "Pegas", "Hydr",  "Casso",
    "Cepheu","Andro","Boote", "Virg",  "Aquil", "Sagit", "Capric",
    "Corv", "Lupin", "Perse", "Trian", "Erid",  "Phen",  "Cete",
    "Tauri","Gemin", "Scorpi","Libr",  "Indus", "Phoen", "Lacert",
    "Vulpe","Centau","Lepor", "Ursar", "Vegan",
};

static const char *SUFFIXES[] = {
    "a",   "us",  "is",  "es",  "ax",   "or",  "on",
    "ula", "ina", "as",  "ea",  "ona",  "ena", "alia",
    "ius", "yra",
};

static const char *MODIFIERS[] = {
    "", "", "", "", "",                     /* most names are unmodified  */
    " Major", " Minor", " Borealis", " Australis", " Magnus",
};

static const int N_PREFIXES  = (int)(sizeof PREFIXES  / sizeof PREFIXES[0]);
static const int N_SUFFIXES  = (int)(sizeof SUFFIXES  / sizeof SUFFIXES[0]);
static const int N_MODIFIERS = (int)(sizeof MODIFIERS / sizeof MODIFIERS[0]);

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
        init_pair(PAIR_BG_STAR, t->bg_star, -1);
        init_pair(PAIR_LINE,    t->line,    -1);
        init_pair(PAIR_NAME,    t->name_color, -1);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_STAR_BASE + i), t->star[i], -1);
    } else {
        static const short fb_star[4] = { COLOR_WHITE, COLOR_YELLOW,
                                          COLOR_CYAN,  COLOR_BLUE };
        init_pair(PAIR_BG_STAR, COLOR_WHITE,  -1);
        init_pair(PAIR_LINE,    COLOR_WHITE,  -1);
        init_pair(PAIR_NAME,    COLOR_YELLOW, -1);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_STAR_BASE + i), fb_star[i], -1);
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
/* §5  constellation — placement, topology, Bresenham, naming             */
/* ===================================================================== */

/*
 * hash3 — stateless 3-int avalanche hash. Same routine as the other
 * procedural showcases. Drives jitter, name selection, and the
 * background-star sprinkle.
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

typedef struct {
    int      x, y;            /* position in region cell coords        */
    uint8_t  twinkle_phase;   /* 0..255, fed to sin for per-star pulse */
    uint8_t  color_idx;       /* 0..3, picks a star tint               */
    float    reveal_t;        /* 0..1, fade-in during DRAW_STARS phase */
} Star;

typedef struct {
    int   from, to;           /* indices into stars[]                  */
    float reveal_t;           /* 0..1, partial Bresenham progress      */
} Edge;

typedef struct {
    Star  stars[N_STARS_MAX];
    int   n_stars;
    Edge  edges[N_EDGES_MAX];
    int   n_edges;
    char  name[40];
    int   seed;
    int   region_w, region_h;
} Constellation;

/* ----------------------------------------------------------------------- *
 * Star placement — jittered grid.                                         *
 * ----------------------------------------------------------------------- */

/*
 * place_stars — drop n stars into a region using a jittered-grid
 * scheme. Region is divided into ~√n × ⌈n/√n⌉ cells; each cell hosts
 * ONE star at a hash-driven offset from the cell centre. The result
 * looks organically scattered without clusters or gaps.
 */
static void place_stars(Star *stars, int n, int region_w, int region_h, int seed)
{
    int cols = (int)ceilf(sqrtf((float)n));
    if (cols < 1) cols = 1;
    int rows = (n + cols - 1) / cols;

    int cell_w = region_w / cols;
    int cell_h = region_h / rows;
    if (cell_w < 1) cell_w = 1;
    if (cell_h < 1) cell_h = 1;

    int idx = 0;
    for (int r = 0; r < rows && idx < n; r++) {
        for (int c = 0; c < cols && idx < n; c++) {
            uint32_t h = hash3(c, r, seed);
            int cx = c * cell_w + cell_w / 2;
            int cy = r * cell_h + cell_h / 2;
            int jit_x = (int)(((int)(h        & 0x3FFu) - 512)
                              * cell_w * (int)(JITTER_FRAC * 1024) / (512 * 1024));
            int jit_y = (int)(((int)((h >> 10) & 0x3FFu) - 512)
                              * cell_h * (int)(JITTER_FRAC * 1024) / (512 * 1024));
            stars[idx].x = cx + jit_x;
            stars[idx].y = cy + jit_y;
            if (stars[idx].x < 1)              stars[idx].x = 1;
            if (stars[idx].x > region_w - 2)   stars[idx].x = region_w - 2;
            if (stars[idx].y < 1)              stars[idx].y = 1;
            if (stars[idx].y > region_h - 2)   stars[idx].y = region_h - 2;
            stars[idx].twinkle_phase = (uint8_t)((h >> 24) & 0xFFu);
            stars[idx].color_idx     = (uint8_t)((h >> 20) & 3u);
            stars[idx].reveal_t      = 0.0f;
            idx++;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Topology builders — four ways to pick edges over the same star set.     *
 * ----------------------------------------------------------------------- */

/*
 * make_tree — Prim's minimum-spanning-tree algorithm. Repeatedly
 * adds the cheapest cross-edge between the current tree and the
 * unvisited stars. Distance metric is squared Euclidean (no sqrt).
 *
 * Time complexity: O(N²); for N ≤ 12 this is ~144 ops. Trivial.
 */
static void make_tree(const Star *stars, int n, Edge *edges, int *n_edges)
{
    *n_edges = 0;
    if (n < 2) return;

    bool in_tree[N_STARS_MAX] = { false };
    in_tree[0] = true;

    for (int iter = 1; iter < n; iter++) {
        int best_from = -1, best_to = -1;
        long best_d2  = (long)1 << 60;
        for (int i = 0; i < n; i++) {
            if (!in_tree[i]) continue;
            for (int j = 0; j < n; j++) {
                if (in_tree[j]) continue;
                long dx = stars[i].x - stars[j].x;
                long dy = stars[i].y - stars[j].y;
                long d2 = dx * dx + dy * dy;
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_from = i;
                    best_to   = j;
                }
            }
        }
        if (best_to < 0) break;
        in_tree[best_to] = true;
        edges[*n_edges].from     = best_from;
        edges[*n_edges].to       = best_to;
        edges[*n_edges].reveal_t = 0.0f;
        (*n_edges)++;
        if (*n_edges >= N_EDGES_MAX) break;
    }
}

/*
 * sort_by_x — bubble-sort stars[] in place by x. N is tiny (≤ 12)
 * so the O(N²) cost is negligible and we avoid qsort's static-state
 * comparator awkwardness.
 */
static void sort_by_x(Star *stars, int n)
{
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (stars[j].x > stars[j + 1].x) {
                Star t = stars[j]; stars[j] = stars[j + 1]; stars[j + 1] = t;
            }
        }
    }
}

/*
 * make_chain — sort stars left-to-right and connect (0)-(1)-(2)-…-(N-1).
 * Produces N-1 edges arranged in a non-crossing left-to-right path.
 */
static void make_chain(Star *stars, int n, Edge *edges, int *n_edges)
{
    *n_edges = 0;
    if (n < 2) return;
    sort_by_x(stars, n);
    for (int i = 0; i < n - 1; i++) {
        edges[i].from     = i;
        edges[i].to       = i + 1;
        edges[i].reveal_t = 0.0f;
    }
    *n_edges = n - 1;
}

/*
 * sort_by_angle — bubble-sort stars[] in place by polar angle around
 * (cx, cy). After this, neighbouring entries in the array sit next
 * to each other on the polygonal cycle around the centroid.
 */
static void sort_by_angle(Star *stars, int n, int cx, int cy)
{
    float ang[N_STARS_MAX];
    for (int i = 0; i < n; i++)
        ang[i] = atan2f((float)(stars[i].y - cy), (float)(stars[i].x - cx));
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (ang[j] > ang[j + 1]) {
                float ta = ang[j];   ang[j]   = ang[j + 1];   ang[j + 1]   = ta;
                Star  ts = stars[j]; stars[j] = stars[j + 1]; stars[j + 1] = ts;
            }
        }
    }
}

/*
 * make_loop — angular-sort around the centroid, then connect the
 * stars into a closed cycle. N edges (vs N-1 for the other patterns).
 */
static void make_loop(Star *stars, int n, Edge *edges, int *n_edges)
{
    *n_edges = 0;
    if (n < 3) return;
    int cx = 0, cy = 0;
    for (int i = 0; i < n; i++) { cx += stars[i].x; cy += stars[i].y; }
    cx /= n; cy /= n;
    sort_by_angle(stars, n, cx, cy);
    for (int i = 0; i < n; i++) {
        edges[i].from     = i;
        edges[i].to       = (i + 1) % n;
        edges[i].reveal_t = 0.0f;
    }
    *n_edges = n;
}

/*
 * make_spoke — pick the star nearest the centroid as the hub; emit
 * one edge from hub to every other star. N-1 edges all sharing one
 * vertex.
 */
static void make_spoke(const Star *stars, int n, Edge *edges, int *n_edges)
{
    *n_edges = 0;
    if (n < 2) return;
    int cx = 0, cy = 0;
    for (int i = 0; i < n; i++) { cx += stars[i].x; cy += stars[i].y; }
    cx /= n; cy /= n;
    int hub  = 0;
    long bd2 = (long)1 << 60;
    for (int i = 0; i < n; i++) {
        long dx = stars[i].x - cx, dy = stars[i].y - cy;
        long d2 = dx * dx + dy * dy;
        if (d2 < bd2) { bd2 = d2; hub = i; }
    }
    int eidx = 0;
    for (int i = 0; i < n; i++) {
        if (i == hub) continue;
        edges[eidx].from     = hub;
        edges[eidx].to       = i;
        edges[eidx].reveal_t = 0.0f;
        eidx++;
        if (eidx >= N_EDGES_MAX) break;
    }
    *n_edges = eidx;
}

/* ----------------------------------------------------------------------- *
 * Procedural naming.                                                      *
 * ----------------------------------------------------------------------- */

/*
 * gen_name — concatenate three random fragments. The hash bits used
 * are far apart so prefix / suffix / modifier choices are
 * uncorrelated.
 */
static void gen_name(uint32_t hash, char *out, size_t len)
{
    int p = (int)( hash         % (uint32_t)N_PREFIXES);
    int s = (int)((hash >>  8u) % (uint32_t)N_SUFFIXES);
    int m = (int)((hash >> 16u) % (uint32_t)N_MODIFIERS);
    snprintf(out, len, "%s%s%s", PREFIXES[p], SUFFIXES[s], MODIFIERS[m]);
}

/* ----------------------------------------------------------------------- *
 * Bresenham — total step count.                                           *
 * ----------------------------------------------------------------------- */

/*
 * bres_steps — how many cells the line from (x0,y0) to (x1,y1) will
 * occupy when rendered. Equals max(|dx|, |dy|) + 1 (one start cell
 * plus that many movement steps).
 */
static inline int bres_steps(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    return (dx > dy ? dx : dy) + 1;
}

/* line_glyph_for — pick '-' / '|' / '/' / '\' from segment direction. */
static inline char line_glyph_for(int dx, int dy)
{
    int adx = abs(dx), ady = abs(dy);
    if (adx >= 2 * ady) return '-';
    if (ady >= 2 * adx) return '|';
    if ((dx > 0) == (dy > 0)) return '\\';
    return '/';
}

/* ----------------------------------------------------------------------- *
 * Top-level: regenerate everything for a new constellation.               *
 * ----------------------------------------------------------------------- */

/*
 * constellation_gen — regenerate the entire constellation from a
 * (pattern, seed) pair. The caller passes the desired star-region
 * dimensions; star positions land in [0, region_w) × [0, region_h)
 * and are rendered at gx0+x, gy0+y in the screen layout step.
 */
static void constellation_gen(Constellation *con, Pattern p, int seed,
                              int region_w, int region_h)
{
    /* Pick N — slightly different per pattern so each topology
     * displays with the count it looks best at. */
    uint32_t h = hash3(seed, (int)p, 0);
    int n;
    switch (p) {
    case PATTERN_TREE:  n = 7 + (int)(h % 4u); break;     /* 7..10 */
    case PATTERN_CHAIN: n = 5 + (int)(h % 4u); break;     /* 5..8  */
    case PATTERN_LOOP:  n = 5 + (int)(h % 4u); break;     /* 5..8  */
    case PATTERN_SPOKE: n = 5 + (int)(h % 5u); break;     /* 5..9  */
    default:            n = 6;
    }
    if (n > N_STARS_MAX) n = N_STARS_MAX;

    place_stars(con->stars, n, region_w, region_h, seed);
    con->n_stars   = n;
    con->region_w  = region_w;
    con->region_h  = region_h;
    con->seed      = seed;

    switch (p) {
    case PATTERN_TREE:  make_tree (con->stars, n, con->edges, &con->n_edges); break;
    case PATTERN_CHAIN: make_chain(con->stars, n, con->edges, &con->n_edges); break;
    case PATTERN_LOOP:  make_loop (con->stars, n, con->edges, &con->n_edges); break;
    case PATTERN_SPOKE: make_spoke(con->stars, n, con->edges, &con->n_edges); break;
    default:            con->n_edges = 0;
    }

    gen_name(hash3(seed, 42, 17), con->name, sizeof con->name);
}

/* ===================================================================== */
/* §6  scene — phase machine, reveal animation                            */
/* ===================================================================== */

typedef enum {
    PHASE_DRAW_STARS = 0,
    PHASE_DRAW_EDGES = 1,
    PHASE_HOLD       = 2,
    PHASE_FADE       = 3,
} Phase;

typedef struct {
    Constellation con;
    Phase   phase;
    float   phase_t;          /* time since entering current phase     */
    float   total_t;          /* wall-clock accumulator                */
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   flash_t;          /* phase-fade flash intensity            */
    int     sky_seed;         /* changes when the constellation does   */
} Scene;

/*
 * scene_rebuild — pick a fresh seed and build a new constellation in
 * the current pattern. Resets the phase machine to DRAW_STARS.
 */
static void scene_rebuild(Scene *s, int region_w, int region_h)
{
    int seed = (int)hash3((int)(s->total_t * 1000.0f),
                          region_w, region_h ^ ((int)s->current_pattern << 8));
    constellation_gen(&s->con, s->current_pattern, seed, region_w, region_h);
    s->phase    = PHASE_DRAW_STARS;
    s->phase_t  = 0.0f;
    s->flash_t  = 1.0f;
    s->sky_seed = seed ^ 0x5A5A5A5A;
}

static void scene_init(Scene *s, int region_w, int region_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_TREE;
    scene_rebuild(s, region_w, region_h);
}

/*
 * scene_resize_to — region size has changed; throw away the current
 * layout and regenerate so positions are valid for the new region.
 */
static void scene_resize_to(Scene *s, int region_w, int region_h)
{
    scene_rebuild(s, region_w, region_h);
}

/*
 * scene_tick — phase-machine update. The reveal_t fields on stars and
 * edges are computed from phase_t / phase_total so pause + resume keep
 * the animation perfectly aligned.
 */
static void scene_tick(Scene *s, float dt)
{
    s->total_t += dt;
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->phase_t     += dt * speed_mul;

    Constellation *c = &s->con;

    switch (s->phase) {
    case PHASE_DRAW_STARS: {
        /* Each star occupies a slice of duration  STARS_TOTAL/N. */
        float slice = (c->n_stars > 0)
                    ? (PHASE_STARS_TOTAL / (float)c->n_stars) : PHASE_STARS_TOTAL;
        for (int i = 0; i < c->n_stars; i++) {
            float t0 = i * slice;
            float t1 = t0 + slice;
            if (s->phase_t >= t1)        c->stars[i].reveal_t = 1.0f;
            else if (s->phase_t > t0)
                c->stars[i].reveal_t = (s->phase_t - t0) / slice;
            else
                c->stars[i].reveal_t = 0.0f;
        }
        if (s->phase_t >= PHASE_STARS_TOTAL) {
            s->phase   = PHASE_DRAW_EDGES;
            s->phase_t = 0.0f;
        }
        break;
    }

    case PHASE_DRAW_EDGES: {
        /* Each edge animates for PHASE_EDGE_TIME. */
        for (int i = 0; i < c->n_edges; i++) {
            float t0 = i * PHASE_EDGE_TIME;
            float t1 = t0 + PHASE_EDGE_TIME;
            if (s->phase_t >= t1)        c->edges[i].reveal_t = 1.0f;
            else if (s->phase_t > t0)
                c->edges[i].reveal_t = (s->phase_t - t0) / PHASE_EDGE_TIME;
            else
                c->edges[i].reveal_t = 0.0f;
        }
        float total = c->n_edges * PHASE_EDGE_TIME;
        if (s->phase_t >= total) {
            s->phase   = PHASE_HOLD;
            s->phase_t = 0.0f;
        }
        break;
    }

    case PHASE_HOLD:
        if (s->phase_t >= PHASE_HOLD_TIME) {
            s->phase   = PHASE_FADE;
            s->phase_t = 0.0f;
            s->flash_t = 1.0f;
        }
        break;

    case PHASE_FADE:
        if (s->phase_t >= PHASE_FADE_TIME) {
            scene_rebuild(s, c->region_w, c->region_h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen — backdrop sky, anchor stars, edges, name, HUD              */
/* ===================================================================== */

typedef struct {
    int cols, rows;
    int region_w, region_h;
    int gx0, gy0;
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    /* Constellation region — 80% wide × 75% tall, centred. Leaves
     * room for the name caption below. */
    int rw = (avail_w * 80) / 100;
    int rh = (avail_h * 75) / 100;
    if (rw < 16) rw = (avail_w < 16) ? avail_w : 16;
    if (rh < 8)  rh = (avail_h < 8)  ? avail_h : 8;
    if (rw > avail_w) rw = avail_w;
    if (rh > avail_h - 2) rh = avail_h - 2;
    if (rh < 4) rh = 4;

    s->region_w = rw;
    s->region_h = rh;
    s->gx0 = (avail_w - rw) / 2;
    s->gy0 = top + (avail_h - rh) / 2 - 1;
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
 * draw_backdrop_sky — faint, twinkling sprinkle of dots throughout
 * the renderable region (excluding HUD rows). A per-cell hash gates
 * placement; a slow-time hash modulates intensity for the shimmer.
 */
static void draw_backdrop_sky(const Screen *sc, int sky_seed,
                              float total_t)
{
    int top = 2, bottom = sc->rows - 1;
    int twinkle_t = (int)(total_t * 1.5f);
    for (int y = top; y < bottom; y++) {
        for (int x = 0; x < sc->cols; x++) {
            uint32_t h = hash3(x, y, sky_seed);
            if ((h % BG_STAR_DENSITY) != 0) continue;
            char glyph = ((h >> 17) & 1u) ? '.' : '`';
            uint32_t h2 = hash3(x, y, sky_seed ^ twinkle_t);
            int attr = ((h2 % 7u) == 0u) ? A_NORMAL : A_DIM;
            attron(COLOR_PAIR(PAIR_BG_STAR) | attr);
            mvaddch(y, x, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(PAIR_BG_STAR) | attr);
        }
    }
}

/*
 * draw_partial_line — partial Bresenham from (x0,y0) to (x1,y1).
 * Plots the first ⌈reveal_t · total_steps⌉ cells; skips the two
 * endpoints (anchor stars draw themselves and look better undimmed).
 *
 * Coordinates are in REGION space; we add gx0/gy0 to land on screen.
 */
static void draw_partial_line(int x0, int y0, int x1, int y1,
                              float reveal_t,
                              int gx0, int gy0,
                              int rows, int cols,
                              int pair, int attr, char glyph)
{
    int dx_abs = abs(x1 - x0);
    int dy_abs = abs(y1 - y0);
    int sx     = (x0 < x1) ?  1 : -1;
    int sy     = (y0 < y1) ?  1 : -1;
    int dx     =  dx_abs;
    int dy     = -dy_abs;
    int err    = dx + dy;

    int total  = (dx_abs > dy_abs ? dx_abs : dy_abs) + 1;
    int limit  = (reveal_t > 0.999f) ? total : (int)ceilf(reveal_t * (float)total);
    if (limit > total) limit = total;

    int x = x0, y = y0;
    int step = 0;
    while (step < limit) {
        bool is_endpoint = (x == x0 && y == y0) || (x == x1 && y == y1);
        if (!is_endpoint) {
            int sy_screen = gy0 + y;
            int sx_screen = gx0 + x;
            if (sy_screen >= 2 && sy_screen < rows - 1 &&
                sx_screen >= 0 && sx_screen < cols) {
                attron(COLOR_PAIR(pair) | attr);
                mvaddch(sy_screen, sx_screen, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | attr);
            }
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
        step++;
    }
}

/*
 * draw_anchor_star — bright '*' or 'O' at the star's position. The
 * star reveal_t fades the brightness in: 0 = invisible, 0.5 = dim,
 * 1.0 = full brightness with twinkle.
 */
static void draw_anchor_star(const Screen *sc, const Star *star,
                              float total_t)
{
    if (star->reveal_t <= 0.05f) return;
    int sy = sc->gy0 + star->y;
    int sx = sc->gx0 + star->x;
    if (sy < 2 || sy >= sc->rows - 1)  return;
    if (sx < 0 || sx >= sc->cols)      return;

    int  pair = PAIR_STAR_BASE + (star->color_idx & 3);
    int  attr;
    char glyph;

    if (star->reveal_t < 0.40f) {
        attr  = A_DIM;
        glyph = '.';
    } else if (star->reveal_t < 0.85f) {
        attr  = A_NORMAL;
        glyph = '*';
    } else {
        /* Fully revealed — twinkle. The star pulses with a
         * sinusoid keyed to its individual phase so neighbours
         * twinkle out of sync. */
        float phase = (float)star->twinkle_phase / 255.0f * 2.0f * (float)M_PI;
        float b     = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * 0.6f * total_t + phase);
        attr  = A_BOLD;
        glyph = (b > 0.65f) ? 'O' : '*';
    }

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_name_label — render the constellation name centred below the
 * region, with a fade-in keyed to the phase. A tiny underline made
 * of '-' grows under it during the HOLD phase.
 */
static void draw_name_label(const Screen *sc, const Scene *s)
{
    if (s->phase != PHASE_HOLD && s->phase != PHASE_FADE) return;
    const Constellation *c = &s->con;

    int len = (int)strlen(c->name);
    int row = sc->gy0 + sc->region_h + 0;
    if (row >= sc->rows - 1) row = sc->rows - 2;
    int col = sc->gx0 + (sc->region_w - len) / 2;
    if (col < 0) col = 0;

    /* Fade in over first second of HOLD; fade out during FADE. */
    float intensity;
    if (s->phase == PHASE_HOLD) {
        intensity = (s->phase_t < 1.0f) ? s->phase_t : 1.0f;
    } else {
        intensity = 1.0f - (s->phase_t / PHASE_FADE_TIME);
        if (intensity < 0.0f) intensity = 0.0f;
    }
    if (intensity < 0.05f) return;

    int attr = (intensity > 0.7f) ? A_BOLD : A_NORMAL;
    attron(COLOR_PAIR(PAIR_NAME) | attr);
    mvprintw(row, col, "%s", c->name);
    attroff(COLOR_PAIR(PAIR_NAME) | attr);

    /* Underline that grows in during HOLD. */
    if (s->phase == PHASE_HOLD) {
        float ut = s->phase_t / 1.5f;
        if (ut > 1.0f) ut = 1.0f;
        int u_len = (int)(ut * (float)len);
        int u_row = row + 1;
        if (u_row < sc->rows - 1) {
            attron(COLOR_PAIR(PAIR_NAME) | A_DIM);
            for (int i = 0; i < u_len; i++) {
                int u_col = col + i;
                if (u_col >= 0 && u_col < sc->cols)
                    mvaddch(u_row, u_col, '-');
            }
            attroff(COLOR_PAIR(PAIR_NAME) | A_DIM);
        }
    }
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    /* Backdrop first — faint sky everywhere. */
    draw_backdrop_sky(sc, s->sky_seed, s->total_t);

    const Constellation *c = &s->con;

    /* Edges first (so anchor stars overdraw at vertices). */
    for (int i = 0; i < c->n_edges; i++) {
        const Edge *e = &c->edges[i];
        if (e->reveal_t <= 0.001f) continue;
        const Star *a = &c->stars[e->from];
        const Star *b = &c->stars[e->to];
        char glyph = line_glyph_for(b->x - a->x, b->y - a->y);
        int  attr  = (e->reveal_t > 0.99f) ? A_DIM : A_NORMAL;
        draw_partial_line(a->x, a->y, b->x, b->y,
                          e->reveal_t,
                          sc->gx0, sc->gy0,
                          sc->rows, sc->cols,
                          PAIR_LINE, attr, glyph);
    }

    /* Anchor stars (overdrawing edge tails at vertices). */
    for (int i = 0; i < c->n_stars; i++) {
        draw_anchor_star(sc, &c->stars[i], s->total_t);
    }

    /* Name caption during HOLD / FADE. */
    draw_name_label(sc, s);

    /* Phase-transition flash. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->total_t * 1000.0f);
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

static const char *phase_name(Phase p)
{
    switch (p) {
    case PHASE_DRAW_STARS: return "DISCOVER ";
    case PHASE_DRAW_EDGES: return "TRACE    ";
    case PHASE_HOLD:       return "NAMED    ";
    case PHASE_FADE:       return "FADING   ";
    default:               return "?        ";
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const Constellation *c = &s->con;
    const char *state_str = s->paused ? "PAUSED   " : phase_name(s->phase);

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
    mvprintw(0, 1, " PROCEDURAL CONSTELLATION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + theme + palette + stats. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-5s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 16;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " stars:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 7;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_STAR_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '*');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  N=%d  E=%d  name:\"%s\" ",
             c->n_stars, c->n_edges, c->name);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
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
    scene_resize_to(&app->scene, app->screen.region_w, app->screen.region_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                         break;
    case 'r': case 'R':
        scene_rebuild(s, app->screen.region_w, app->screen.region_h);
        break;

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

    /* Pattern switch — regenerate so the user sees the new topology
     * applied to a fresh constellation. */
    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_rebuild(s, app->screen.region_w, app->screen.region_h);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_rebuild(s, app->screen.region_w, app->screen.region_h);
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
    scene_init(&app->scene, app->screen.region_w, app->screen.region_h);

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
