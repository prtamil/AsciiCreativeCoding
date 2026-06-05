/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_constellation.c
 *   — Procedural constellation generator: place anchor stars in the
 *     sky, connect them with one of fifteen graph topologies,
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
 *       in below the figure. After a brief HOLD, the sky clears and the
 *       next constellation begins to discover itself.
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
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, data tables, core state types
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure decisions: no mutation, no I/O
 *   §4 SIMULATION   — generation (place/topology/name) + the reveal tick
 *   §5 EFFECTS      — cosmetic-only state (one-line note: none stored)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: phase machine)
 *   §7 RENDER       — state → screen; reads only, never mutates
 *   §8 APP          — user events + per-tick combine + main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the discovery animation
 *   r          regenerate constellation with a new seed
 *   n / N      next pattern  (cycles all 15: TREE, CHAIN, LOOP, SPOKE,
 *              WHEEL, FAN, STARPOLY, NEAREST, MESH, ARCH, BINARY, CROSS,
 *              TRIANGLES, SPIRAL, CATERPILLAR)
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
 *                      Fifteen edge-selection rules; the four canonical
 *                      ones below each correspond to a real-world
 *                      constellation morphology:
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
 *                      The other eleven (WHEEL, FAN, STARPOLY, NEAREST,
 *                      MESH, ARCH, BINARY, CROSS, TRIANGLES, SPIRAL,
 *                      CATERPILLAR) layer more graph shapes — cycle+hub,
 *                      crossing star-polygons, k-nearest webs, two-cluster
 *                      binaries, spirals — over the same star set.
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
 *
 *   Graph topologies (the 15 edge-selection rules) —
 *   • West, D.B. — "Introduction to Graph Theory" (2nd ed., 2001). The
 *     named families used here: path (CHAIN), cycle (LOOP), star (SPOKE),
 *     wheel (WHEEL), tree / spanning tree (TREE), etc.
 *   • Prim, R. (1957) — "Shortest connection networks and some
 *     generalizations", Bell System Tech. J. 36(6). The MST → TREE.
 *     https://en.wikipedia.org/wiki/Prim%27s_algorithm
 *   • Jaromczyk, J.W. & Toussaint, G.T. (1992) — "Relative neighborhood
 *     graphs and their relatives", Proc. IEEE 80(9). Proximity graphs —
 *     the 1- and 2-nearest-neighbour graphs behind NEAREST / MESH, and how
 *     the NN-graph, RNG and MST nest.
 *   • Wikipedia — "Star polygon" (the {n/2} crossing construction → STARPOLY)
 *     https://en.wikipedia.org/wiki/Star_polygon
 *
 *   Point placement & rendering —
 *   • Lloyd, S. (1982) — "Least squares quantization in PCM". The relaxation
 *     dual of the jittered-grid star scatter.
 *   • Wikipedia — "Poisson-disk sampling" (even-but-organic point spacing)
 *     https://en.wikipedia.org/wiki/Supersampling#Poisson_disc
 *   • Bresenham, J. (1965) — "Algorithm for computer control of a digital
 *     plotter", IBM Systems Journal 4(1):25-30. Integer line rasterisation
 *     — drawn partially (reveal_t) for the animated edge "trace".
 *     https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *
 *   Domain —
 *   • Wikipedia — "Constellation"  https://en.wikipedia.org/wiki/Constellation
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
 * pick the edges (one of fifteen rules), draw the lines (Bresenham),
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
 * and replace "draw pencil lines" with one of fifteen small algorithms.
 * That is the entire generator. The reason the result LOOKS like a
 * real constellation is that real constellations were drawn by humans
 * subject to the same constraints: stars roughly evenly distributed
 * across the visible sky, and edges chosen for a clear shape (mostly
 * short hops, every star reached — though a few patterns bend this on
 * purpose: STARPOLY crosses its own lines, BINARY leaves two clusters).
 *
 * The first FOUR topologies mirror the morphological families real
 * constellations fall into:
 *   - branchy ones (Orion, Hercules)            → MST  (TREE)
 *   - linear ones (Big Dipper, Cassiopeia's W)  → CHAIN
 *   - closed ones (Pegasus square, Corona)      → LOOP
 *   - radial ones (Crux, the Cross)             → SPOKE
 * The remaining eleven extend the catalogue with more graph shapes —
 * wheel, fan, star-polygon, nearest-neighbour mesh, arch, binary, cross,
 * triangulation, spiral, caterpillar — for variety as the sky cycles.
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
 *       PHASE_FADE       : brief dwell, then regenerate.
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
 *  • Press 'r': regenerate. The new constellation has a
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

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants + const data tables
 *                    (themes, name fragments) + the core state types (Star,
 *                    Edge, Constellation, Phase), relocated up so the pure §3
 *                    helpers can see them.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — pure reads/maps: hash3, pattern_name, phase_name,
 *                    star_centroid, nearest_to, has_edge, bres_steps,
 *                    line_glyph_for, reveal_at (the staggered fade ramp),
 *                    jitter_offset. They only READ, so no RENDER/EFFECTS
 *                    reordering can corrupt them.
 *   SIMULATION   §4  Constellation.{stars,edges,n_*,name,seed,region_*} and
 *                    Scene.{con,phase,phase_t,total_t,paused,speed,
 *                    current_theme,current_pattern,sky_seed}. Generation
 *                    (constellation_gen → place_stars + make_* + gen_name) and
 *                    the per-tick reveal advance (scene_tick). Only writers.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; the twinkle is
 *                    render-derived. One-line section, not a real layer.
 *   DELAYS       §6  (none) — the HOLD + FADE dwells are phases of the scene
 *                    machine, timed inside scene_tick (§4). One-line section.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (theme_apply,
 *                    color_init, draw_*, scene_draw, screen_draw). Reads §4
 *                    state via the §3 deciders; never writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (reveal) + DELAYS
 *     screen_draw() ; screen_present()    // RENDER (reads only; twinkle
 * derived) getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state. User events
 * (app_handle_key / app_do_resize) mutate Scene/Screen on a keypress or
 * SIGWINCH — and a key may trigger a full regeneration — but they run once per
 * frame OUTSIDE the accumulator loop, not as part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, data tables, core state types               */
/* ===================================================================== */

enum {
  /* Maximum number of anchor stars in a constellation. 12 covers
   * even Orion's 17-star canonical figure compactly enough for
   * an ASCII showcase. Larger N starts to look noisy. */
  N_STARS_MAX = 12,
  N_EDGES_MAX = 24,

  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 240,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Background-star density: 1 in N cells in the sky carries a
   * faint star. 60 → ~1.7 % of the sky; subtle sprinkle. */
  BG_STAR_DENSITY = 60,

  /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_BG_STAR = 3,   /* dim background sprinkle         */
  PAIR_LINE = 4,      /* constellation lines             */
  PAIR_STAR_BASE = 5, /* +0..+3 = 4 anchor-star tints    */
  PAIR_NAME = 9,      /* constellation name              */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/*
 * Animation timings (in seconds). The "speed" knob multiplies these
 * inversely — speed=16 doubles the rate, speed=4 halves it.
 */
#define PHASE_STARS_TOTAL 1.6f /* full reveal of all anchor stars */
#define PHASE_EDGE_TIME 0.30f  /* time to draw one edge           */
#define PHASE_HOLD_TIME 6.0f   /* dwell once finished             */
#define PHASE_FADE_TIME 0.6f   /* brief dwell before regenerating */

/* Anchor-star twinkle frequency (Hz): how fast a fully-revealed star pulses. */
#define TWINKLE_HZ 0.6f

/*
 * Jittered-grid star placement: maximum offset from cell centre as a
 * fraction of cell size. 0.40 keeps adjacent cells from overlapping
 * but feels organic.
 */
#define JITTER_FRAC 0.40f

/*
 * ── Pattern ───────────────────────────────────────────────────────────── *
 * Fifteen edge-selection rules over the SAME anchor-star set. The intent: the
 * "shape" of a constellation is not the stars (those are placed once, the same
 * way for every pattern) but the GRAPH drawn over them — so a constellation
 * family is just a graph topology. Each enum value selects one make_* builder
 * (§4); none move the stars. The first four mirror the morphological families
 * real constellations fall into (branchy / linear / closed / radial); the rest
 * extend the catalogue (wheel, fan, crossing star-polygon, k-nearest mesh, …).
 * REFS: graph families — West, "Introduction to Graph Theory"; see file header.
 */
typedef enum {
  PATTERN_TREE = 0,    /* MST — sparse organic branching         */
  PATTERN_CHAIN,       /* left→right path                        */
  PATTERN_LOOP,        /* convex cycle around the centroid       */
  PATTERN_SPOKE,       /* central hub + spokes to all            */
  PATTERN_WHEEL,       /* rim cycle + hub spokes (a wheel)        */
  PATTERN_FAN,         /* one corner apex, rays to all (a fan)    */
  PATTERN_STARPOLY,    /* star polygon — edges CROSS (pentagram)  */
  PATTERN_NEAREST,     /* each star to its 1 nearest (scatter)    */
  PATTERN_MESH,        /* each star to its 2 nearest (a web)      */
  PATTERN_ARCH,        /* open angular arc (a crown)              */
  PATTERN_BINARY,      /* two separate clusters (a binary)        */
  PATTERN_CROSS,       /* two interleaved chains (a weave)        */
  PATTERN_TRIANGLES,   /* apex fan + arc → tiled triangles        */
  PATTERN_SPIRAL,      /* inner→outer by radius (an arm)          */
  PATTERN_CATERPILLAR, /* x-sorted spine + leaves                 */
  N_PATTERNS,
} Pattern;

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * WHAT  One named colour palette (10 ship; t/T cycles). A theme assigns a
 *       256-colour code to every drawable element; theme_apply() loads them
 *       into the ncurses pairs. Same 10-name menu as the sibling showcases.
 *
 * VALUE LOGIC  Five roles, each a separate slot so the backdrop, lines, stars
 *       and caption can be tinted independently: bg_star (the faint sky
 *       sprinkle — theme-tinted, not gray, so each sky has character), line
 *       (the constellation edges), star[4] (four anchor-star tints, picked
 *       per star by Star.color_idx so neighbours differ), name_color (the
 *       caption). Every code sits in the BRIGHT half of the cube so even A_DIM
 *       cells stay legible on default-black. REFS: documentation/COLOR.md. */
typedef struct {
  const char *name; /* HUD label (t/T cycles)                       */
  short bg_star;    /* faint backdrop-sprinkle tint                 */
  short line;       /* constellation edge tint                      */
  short star[4];    /* 4 anchor-star tints, indexed by Star.color_idx*/
  short name_color; /* caption tint                                 */
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
    {"DEFAULT", 245, 252, {226, 230, 159, 117}, 226},
    {"MATRIX", 65, 118, {230, 226, 154, 118}, 154},
    {"NOVA", 97, 213, {231, 219, 207, 177}, 219},
    {"MONO", 247, 253, {255, 252, 248, 244}, 254},
    {"OCEAN", 67, 117, {231, 195, 159, 87}, 159},
    {"FIRE", 131, 214, {231, 226, 215, 209}, 226},
    {"EARTH", 137, 223, {231, 230, 222, 179}, 230},
    {"FOREST", 66, 114, {231, 192, 156, 120}, 156},
    {"DESERT", 180, 223, {231, 230, 223, 215}, 230},
    {"ARCTIC", 153, 195, {231, 219, 195, 159}, 195},
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
    "Auri",   "Lyr",    "Dracon", "Cygn",  "Pegas", "Hydr",  "Casso",
    "Cepheu", "Andro",  "Boote",  "Virg",  "Aquil", "Sagit", "Capric",
    "Corv",   "Lupin",  "Perse",  "Trian", "Erid",  "Phen",  "Cete",
    "Tauri",  "Gemin",  "Scorpi", "Libr",  "Indus", "Phoen", "Lacert",
    "Vulpe",  "Centau", "Lepor",  "Ursar", "Vegan",
};

static const char *SUFFIXES[] = {
    "a",   "us", "is", "es",  "ax",  "or",   "on",  "ula",
    "ina", "as", "ea", "ona", "ena", "alia", "ius", "yra",
};

static const char *MODIFIERS[] = {
    "",           "",        "",
    "",           "", /* most names are unmodified  */
    " Major",     " Minor",  " Borealis",
    " Australis", " Magnus",
};

static const int N_PREFIXES = (int)(sizeof PREFIXES / sizeof PREFIXES[0]);
static const int N_SUFFIXES = (int)(sizeof SUFFIXES / sizeof SUFFIXES[0]);
static const int N_MODIFIERS = (int)(sizeof MODIFIERS / sizeof MODIFIERS[0]);

/* ── Star ──────────────────────────────────────────────────────────────── *
 * One anchor star — a graph VERTEX, the node-link diagram's node. Placed once
 * by place_stars (a jittered grid) and never moved; the topology builders only
 * read positions and emit edges between them.
 *   x,y           position in REGION cell coords (0..region_w/h); rendered at
 *                 gx0+x, gy0+y. Some make_* sort the array in place, so an
 *                 index is only meaningful relative to the current order.
 *   twinkle_phase 0..255 → an angle fed to sinf so each star pulses out of
 *                 phase with its neighbours (no shared, lock-step flicker).
 *   color_idx     0..3, indexes Theme.star[] — a per-star tint, hash-chosen.
 *   reveal_t      0..1 fade-in progress during the DISCOVER phase; SIMULATION
 *                 state written by scene_tick, read by draw_anchor_star. */
typedef struct {
  int x, y;
  uint8_t twinkle_phase;
  uint8_t color_idx;
  float reveal_t;
} Star;

/* ── Edge ──────────────────────────────────────────────────────────────── *
 * One constellation line — a graph EDGE between two stars (indices into the
 * Constellation's stars[]). The topology IS the edge set; the 15 patterns
 * differ only in which edges they emit.
 *   reveal_t  0..1 partial-Bresenham progress during the TRACE phase: the line
 *             is drawn for its first ⌈reveal_t·steps⌉ cells, so it appears to
 *             grow from one endpoint. SIMULATION state (scene_tick writes it).
 */
typedef struct {
  int from, to;
  float reveal_t;
} Edge;

/* ── Constellation ─────────────────────────────────────────────────────── *
 * WHAT  The figure: a small node-link graph (anchor stars + the edges over
 *       them) plus its procedural name. This is the domain object every render
 *       reads; SIMULATION (§4) is its only writer, rebuilt wholesale per sky.
 *
 * WHY FIXED ARRAYS  stars[]/edges[] are static (N_STARS_MAX=12, N_EDGES_MAX=24)
 *       — tiny and never malloc'd; a rebuild overwrites in place.
 *
 * VALUE LOGIC  n_stars / n_edges are the live counts (≤ the array caps); name
 *       is the generated label; seed is what this figure was grown from (also
 *       reused by the backdrop hash); region_w/h record the area the stars were
 *       placed into, so a resize knows to regenerate. REFS: node-link graph
 *       drawing + the proximity/spanning graphs behind the patterns — see the
 *       file-header References block. */
typedef struct {
  Star stars[N_STARS_MAX]; /* the vertices (graph nodes)              */
  int n_stars;
  Edge edges[N_EDGES_MAX]; /* the lines (graph edges) = the topology  */
  int n_edges;
  char name[40];          /* procedural Latin-flavoured name         */
  int seed;               /* what this figure was generated from     */
  int region_w, region_h; /* the cell area the stars were placed in  */
} Constellation;

/* ── Phase ─────────────────────────────────────────────────────────────── *
 * The reveal animation's state machine, advanced by scene_tick (§4):
 *   DRAW_STARS — stars fade in one slice at a time ("discovery").
 *   DRAW_EDGES — edges trace in via partial Bresenham.
 *   HOLD       — figure frozen, name caption fades in (the DELAYS dwell).
 *   FADE       — a brief dwell, then scene_rebuild regenerates a new sky.
 * The order is the lifecycle; phase_t (in Scene) measures time within the
 * current phase, and the thresholds are PHASE_*_TIME (§1). */
typedef enum {
  PHASE_DRAW_STARS = 0,
  PHASE_DRAW_EDGES = 1,
  PHASE_HOLD = 2,
  PHASE_FADE = 3,
} Phase;

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The 60 fps frame cap and the fixed-timestep
 * accumulator that decides how many scene_tick()s run per frame are POLICY,
 * applied in main() (§8). */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Pure reads / decisions: each returns a value from its arguments (or the
 * read-only star/edge arrays + name tables) with NO mutation and NO I/O. The
 * topology builders that MUTATE (sort stars, append edges) live in §4; only
 * the pure deciders are here — the hash, the centroid / nearest / has-edge
 * queries, the Bresenham step-count + line glyph, the enum→string mappings, and
 * the small math (reveal_at staggered ramp, jitter_offset) — so no
 * RENDER/EFFECTS reordering can corrupt them. */

/* 5-char padded names so the HUD field width stays fixed. */
static const char *pattern_name(Pattern p) {
  static const char *names[N_PATTERNS] = {
      "TREE ", "CHAIN", "LOOP ", "SPOKE", "WHEEL", "FAN  ", "POLY*", "NEAR ",
      "MESH ", "ARCH ", "TWINS", "CROSS", "TRIAD", "SPIRL", "SPINE",
  };
  return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?    ";
}

/*
 * hash3 — stateless 3-int avalanche hash. Same routine as the other
 * procedural showcases. Drives jitter, name selection, and the
 * background-star sprinkle.
 */
static inline uint32_t hash3(int wx, int wy, int wz) {
  uint32_t h = (uint32_t)wx * 73856093u ^ (uint32_t)wy * 19349663u ^
               (uint32_t)wz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

/* ----------------------------------------------------------------------- *
 * Shared helpers for the extra topologies (below).                        *
 * ----------------------------------------------------------------------- */

/* Integer centroid of the star set. */
static void star_centroid(const Star *stars, int n, int *cx, int *cy) {
  long sx = 0, sy = 0;
  for (int i = 0; i < n; i++) {
    sx += stars[i].x;
    sy += stars[i].y;
  }
  *cx = (int)(sx / n);
  *cy = (int)(sy / n);
}

/* Index of the star nearest (px,py), skipping `except` (-1 = none). */
static int nearest_to(const Star *stars, int n, int px, int py, int except) {
  int best = -1;
  long bd2 = (long)1 << 60;
  for (int i = 0; i < n; i++) {
    if (i == except)
      continue;
    long dx = stars[i].x - px, dy = stars[i].y - py;
    long d2 = dx * dx + dy * dy;
    if (d2 < bd2) {
      bd2 = d2;
      best = i;
    }
  }
  return best;
}

/* True if undirected edge {a,b} is already present. */
static bool has_edge(const Edge *edges, int n_edges, int a, int b) {
  for (int i = 0; i < n_edges; i++)
    if ((edges[i].from == a && edges[i].to == b) ||
        (edges[i].from == b && edges[i].to == a))
      return true;
  return false;
}

/* ----------------------------------------------------------------------- *
 * Bresenham — total step count.                                           *
 * ----------------------------------------------------------------------- */

/*
 * bres_steps — how many cells the line from (x0,y0) to (x1,y1) will
 * occupy when rendered. Equals max(|dx|, |dy|) + 1 (one start cell
 * plus that many movement steps).
 */
static inline int bres_steps(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  return (dx > dy ? dx : dy) + 1;
}

/* line_glyph_for — pick '-' / '|' / '/' / '\' from segment direction. */
static inline char line_glyph_for(int dx, int dy) {
  int adx = abs(dx), ady = abs(dy);
  if (adx >= 2 * ady)
    return '-';
  if (ady >= 2 * adx)
    return '|';
  if ((dx > 0) == (dy > 0))
    return '\\';
  return '/';
}

static const char *phase_name(Phase p) {
  switch (p) {
  case PHASE_DRAW_STARS:
    return "DISCOVER ";
  case PHASE_DRAW_EDGES:
    return "TRACE    ";
  case PHASE_HOLD:
    return "NAMED    ";
  case PHASE_FADE:
    return "FADING   ";
  default:
    return "?        ";
  }
}

/* Staggered fade-in value at `now` for an item whose animation window is
 * [start, start+slice]: 0 before it starts, a linear ramp to 1 across the
 * window, 1 after. The "reveal wavefront" that sweeps the stars then the edges
 * in turn is just this evaluated with start = i·slice for item i. */
static inline float reveal_at(float now, float start, float slice) {
  if (now >= start + slice)
    return 1.0f;
  if (now > start)
    return (now - start) / slice;
  return 0.0f;
}

/* Hash-driven jitter offset for jittered-grid placement: maps the low 10 bits
 * of `hbits` (0..1023, recentred to ±512) to an offset within ±JITTER_FRAC of
 * a cell's size. Integer fixed-point (×1024 then /1024) avoids floats. */
static inline int jitter_offset(uint32_t hbits, int cell_size) {
  return ((int)(hbits & 0x3FFu) - 512) * cell_size * (int)(JITTER_FRAC * 1024) /
         (512 * 1024);
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (only writers of sim state)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. Two phases of work:
 *  • GENERATION (once per new sky) — constellation_gen runs place_stars, one
 *    of the 15 make_* topology builders (some sort the star array in place,
 *    all append to edges via add_edge), and gen_name.
 *  • the per-tick ADVANCE — scene_tick drives the phase machine and writes the
 *    reveal_t fade/trace progress onto every star and edge.
 * Mutates: Constellation.{stars,edges,n_stars,n_edges,name,seed,region_*};
 * Scene.{con,phase,phase_t,total_t,paused,speed,current_theme,
 * current_pattern,sky_seed}. scene_tick() is the single per-tick entry point
 * (called only from main, §8); user events (key/resize) also mutate Scene but
 * are NOT part of the tick -- see §8. */

/* ----------------------------------------------------------------------- *
 * Star placement — jittered grid.                                         *
 * ----------------------------------------------------------------------- */

/*
 * place_stars — drop n stars into a region using a jittered-grid
 * scheme. Region is divided into ~√n × ⌈n/√n⌉ cells; each cell hosts
 * ONE star at a hash-driven offset from the cell centre. The result
 * looks organically scattered without clusters or gaps.
 */
static void place_stars(Star *stars, int n, int region_w, int region_h,
                        int seed) {
  /* Lay a √n × ⌈n/√n⌉ grid of cells over the region. */
  int cols = (int)ceilf(sqrtf((float)n));
  if (cols < 1)
    cols = 1;
  int rows = (n + cols - 1) / cols;

  int cell_w = region_w / cols;
  int cell_h = region_h / rows;
  if (cell_w < 1)
    cell_w = 1;
  if (cell_h < 1)
    cell_h = 1;

  /* One star per cell, hash-jittered off the cell centre (so it scatters
   * organically) plus its per-star twinkle phase and tint. */
  int idx = 0;
  for (int r = 0; r < rows && idx < n; r++) {
    for (int c = 0; c < cols && idx < n; c++) {
      uint32_t h = hash3(c, r, seed);
      int cx = c * cell_w + cell_w / 2;
      int cy = r * cell_h + cell_h / 2;
      stars[idx].x = cx + jitter_offset(h, cell_w);
      stars[idx].y = cy + jitter_offset(h >> 10, cell_h);
      if (stars[idx].x < 1)
        stars[idx].x = 1;
      if (stars[idx].x > region_w - 2)
        stars[idx].x = region_w - 2;
      if (stars[idx].y < 1)
        stars[idx].y = 1;
      if (stars[idx].y > region_h - 2)
        stars[idx].y = region_h - 2;
      stars[idx].twinkle_phase = (uint8_t)((h >> 24) & 0xFFu);
      stars[idx].color_idx = (uint8_t)((h >> 20) & 3u);
      stars[idx].reveal_t = 0.0f;
      idx++;
    }
  }
}

/* ----------------------------------------------------------------------- *
 * Topology builders — fifteen ways to pick edges over the same star set.  *
 * ----------------------------------------------------------------------- */

/*
 * make_tree — Prim's minimum-spanning-tree algorithm. Repeatedly
 * adds the cheapest cross-edge between the current tree and the
 * unvisited stars. Distance metric is squared Euclidean (no sqrt).
 *
 * Time complexity: O(N²); for N ≤ 12 this is ~144 ops. Trivial.
 */
static void make_tree(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;

  bool in_tree[N_STARS_MAX] = {false};
  in_tree[0] = true;

  for (int iter = 1; iter < n; iter++) {
    int best_from = -1, best_to = -1;
    long best_d2 = (long)1 << 60;
    for (int i = 0; i < n; i++) {
      if (!in_tree[i])
        continue;
      for (int j = 0; j < n; j++) {
        if (in_tree[j])
          continue;
        long dx = stars[i].x - stars[j].x;
        long dy = stars[i].y - stars[j].y;
        long d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_from = i;
          best_to = j;
        }
      }
    }
    if (best_to < 0)
      break;
    in_tree[best_to] = true;
    edges[*n_edges].from = best_from;
    edges[*n_edges].to = best_to;
    edges[*n_edges].reveal_t = 0.0f;
    (*n_edges)++;
    if (*n_edges >= N_EDGES_MAX)
      break;
  }
}

/*
 * sort_by_x — bubble-sort stars[] in place by x. N is tiny (≤ 12)
 * so the O(N²) cost is negligible and we avoid qsort's static-state
 * comparator awkwardness.
 */
static void sort_by_x(Star *stars, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (stars[j].x > stars[j + 1].x) {
        Star t = stars[j];
        stars[j] = stars[j + 1];
        stars[j + 1] = t;
      }
    }
  }
}

/*
 * make_chain — sort stars left-to-right and connect (0)-(1)-(2)-…-(N-1).
 * Produces N-1 edges arranged in a non-crossing left-to-right path.
 */
static void make_chain(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  sort_by_x(stars, n);
  for (int i = 0; i < n - 1; i++) {
    edges[i].from = i;
    edges[i].to = i + 1;
    edges[i].reveal_t = 0.0f;
  }
  *n_edges = n - 1;
}

/*
 * sort_by_angle — bubble-sort stars[] in place by polar angle around
 * (cx, cy). After this, neighbouring entries in the array sit next
 * to each other on the polygonal cycle around the centroid.
 */
static void sort_by_angle(Star *stars, int n, int cx, int cy) {
  float ang[N_STARS_MAX];
  for (int i = 0; i < n; i++)
    ang[i] = atan2f((float)(stars[i].y - cy), (float)(stars[i].x - cx));
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (ang[j] > ang[j + 1]) {
        float ta = ang[j];
        ang[j] = ang[j + 1];
        ang[j + 1] = ta;
        Star ts = stars[j];
        stars[j] = stars[j + 1];
        stars[j + 1] = ts;
      }
    }
  }
}

/*
 * make_loop — angular-sort around the centroid, then connect the
 * stars into a closed cycle. N edges (vs N-1 for the other patterns).
 */
static void make_loop(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3)
    return;
  int cx = 0, cy = 0;
  for (int i = 0; i < n; i++) {
    cx += stars[i].x;
    cy += stars[i].y;
  }
  cx /= n;
  cy /= n;
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n; i++) {
    edges[i].from = i;
    edges[i].to = (i + 1) % n;
    edges[i].reveal_t = 0.0f;
  }
  *n_edges = n;
}

/*
 * make_spoke — pick the star nearest the centroid as the hub; emit
 * one edge from hub to every other star. N-1 edges all sharing one
 * vertex.
 */
static void make_spoke(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx = 0, cy = 0;
  for (int i = 0; i < n; i++) {
    cx += stars[i].x;
    cy += stars[i].y;
  }
  cx /= n;
  cy /= n;
  int hub = 0;
  long bd2 = (long)1 << 60;
  for (int i = 0; i < n; i++) {
    long dx = stars[i].x - cx, dy = stars[i].y - cy;
    long d2 = dx * dx + dy * dy;
    if (d2 < bd2) {
      bd2 = d2;
      hub = i;
    }
  }
  int eidx = 0;
  for (int i = 0; i < n; i++) {
    if (i == hub)
      continue;
    edges[eidx].from = hub;
    edges[eidx].to = i;
    edges[eidx].reveal_t = 0.0f;
    eidx++;
    if (eidx >= N_EDGES_MAX)
      break;
  }
  *n_edges = eidx;
}

/* Append edge {from,to} (skips self-loops and respects N_EDGES_MAX). */
static void add_edge(Edge *edges, int *n_edges, int from, int to) {
  if (*n_edges >= N_EDGES_MAX || from == to)
    return;
  edges[*n_edges].from = from;
  edges[*n_edges].to = to;
  edges[*n_edges].reveal_t = 0.0f;
  (*n_edges)++;
}

/* make_wheel — a rim cycle (angular order) plus spokes from the most-central
 * star to all others: cycle + hub = a wheel. */
static void make_wheel(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  int hub = nearest_to(stars, n, cx, cy, -1);
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, i, (i + 1) % n); /* rim */
  for (int i = 0; i < n; i++)
    if (i != hub && !has_edge(edges, *n_edges, hub, i))
      add_edge(edges, n_edges, hub, i); /* spokes */
}

/* make_fan — lowest star is the apex; rays to every other star (a hand-fan
 * fanning out from one corner; unlike SPOKE the apex is on the edge). */
static void make_fan(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int apex = 0;
  for (int i = 1; i < n; i++)
    if (stars[i].y > stars[apex].y)
      apex = i;
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, apex, i);
}

/* make_starpoly — angular order, but connect every-other vertex (i → i+2). For
 * an ODD star count this traces one closed path whose edges CROSS the middle —
 * a pentagram / heptagram. (constellation_gen feeds it an odd n.) */
static void make_starpoly(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 5) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, i, (i + 2) % n);
}

/* make_nearest — each star to its single nearest neighbour (deduped): a sparse
 * organic scatter of short links. */
static void make_nearest(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  for (int i = 0; i < n; i++) {
    int j = nearest_to(stars, n, stars[i].x, stars[i].y, i);
    if (j >= 0 && !has_edge(edges, *n_edges, i, j))
      add_edge(edges, n_edges, i, j);
  }
}

/* make_mesh — each star to its TWO nearest neighbours (deduped): a fuller
 * organic web than NEAREST. */
static void make_mesh(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  for (int i = 0; i < n; i++) {
    int n1 = -1, n2 = -1;
    long d1 = (long)1 << 60, d2 = (long)1 << 60;
    for (int j = 0; j < n; j++) {
      if (j == i)
        continue;
      long dx = stars[i].x - stars[j].x, dy = stars[i].y - stars[j].y;
      long dd = dx * dx + dy * dy;
      if (dd < d1) {
        d2 = d1;
        n2 = n1;
        d1 = dd;
        n1 = j;
      } else if (dd < d2) {
        d2 = dd;
        n2 = j;
      }
    }
    if (n1 >= 0 && !has_edge(edges, *n_edges, i, n1))
      add_edge(edges, n_edges, i, n1);
    if (n2 >= 0 && !has_edge(edges, *n_edges, i, n2))
      add_edge(edges, n_edges, i, n2);
  }
}

/* make_arch — angular order but an OPEN path (no closing edge): a graceful arc
 * or crown across the sky. */
static void make_arch(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1);
}

/* make_binary — split the field left/right and loop each half: two distinct
 * sub-constellations (a binary system). */
static void make_binary(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  int mid = n / 2;
  for (int i = 0; i < mid - 1; i++)
    add_edge(edges, n_edges, i, i + 1); /* left chain  */
  for (int i = mid; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1); /* right chain */
  if (mid >= 3)
    add_edge(edges, n_edges, 0, mid - 1); /* close left  */
  if (n - mid >= 3)
    add_edge(edges, n_edges, mid, n - 1); /* close right */
}

/* make_cross — two interleaved chains (even- vs odd-ranked after x-sort) that
 * weave across each other, joined at the ends: a cross-hatch / lattice. */
static void make_cross(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  for (int i = 0; i + 2 < n; i += 2)
    add_edge(edges, n_edges, i, i + 2);
  for (int i = 1; i + 2 < n; i += 2)
    add_edge(edges, n_edges, i, i + 2);
  add_edge(edges, n_edges, 0, 1);
  add_edge(edges, n_edges, n - 2, n - 1);
}

/* make_triangles — apex = lowest star; angular-sort all, fan apex→each and
 * chain the rest, tiling the figure with triangles between adjacent rays. */
static void make_triangles(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  int apex = 0;
  for (int i = 1; i < n; i++)
    if (stars[i].y > stars[apex].y)
      apex = i;
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, apex, i); /* fan */
  for (int i = 0; i < n - 1; i++)
    if (i != apex && i + 1 != apex)
      add_edge(edges, n_edges, i, i + 1); /* arc → triangles */
}

/* make_spiral — order stars by radius from the centroid and connect inner →
 * outer: a single arm winding outward. */
static void make_spiral(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  for (int i = 1; i < n; i++) { /* insertion-sort by radius² */
    Star key = stars[i];
    long kr =
        (long)(key.x - cx) * (key.x - cx) + (long)(key.y - cy) * (key.y - cy);
    int j = i - 1;
    while (j >= 0) {
      long jr = (long)(stars[j].x - cx) * (stars[j].x - cx) +
                (long)(stars[j].y - cy) * (stars[j].y - cy);
      if (jr <= kr)
        break;
      stars[j + 1] = stars[j];
      j--;
    }
    stars[j + 1] = key;
  }
  for (int i = 0; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1);
}

/* make_caterpillar — x-sorted spine of the even-ranked stars, with each
 * odd-ranked star hung off its nearest spine star as a leaf. */
static void make_caterpillar(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  int prev = -1;
  for (int i = 0; i < n; i += 2) { /* spine */
    if (prev >= 0)
      add_edge(edges, n_edges, prev, i);
    prev = i;
  }
  for (int i = 1; i < n; i += 2) { /* leaves → nearest spine star */
    int best = -1;
    long bd2 = (long)1 << 60;
    for (int j = 0; j < n; j += 2) {
      long dx = stars[i].x - stars[j].x, dy = stars[i].y - stars[j].y;
      long d2 = dx * dx + dy * dy;
      if (d2 < bd2) {
        bd2 = d2;
        best = j;
      }
    }
    if (best >= 0)
      add_edge(edges, n_edges, i, best);
  }
}

/* ----------------------------------------------------------------------- *
 * Procedural naming.                                                      *
 * ----------------------------------------------------------------------- */

/*
 * gen_name — concatenate three random fragments. The hash bits used
 * are far apart so prefix / suffix / modifier choices are
 * uncorrelated.
 */
static void gen_name(uint32_t hash, char *out, size_t len) {
  int p = (int)(hash % (uint32_t)N_PREFIXES);
  int s = (int)((hash >> 8u) % (uint32_t)N_SUFFIXES);
  int m = (int)((hash >> 16u) % (uint32_t)N_MODIFIERS);
  snprintf(out, len, "%s%s%s", PREFIXES[p], SUFFIXES[s], MODIFIERS[m]);
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
                              int region_w, int region_h) {
  /* Pick N — slightly different per pattern so each topology
   * displays with the count it looks best at. */
  uint32_t h = hash3(seed, (int)p, 0);
  int n;
  switch (p) {
  case PATTERN_CHAIN:
  case PATTERN_LOOP:
    n = 5 + (int)(h % 4u);
    break; /* 5..8  */
  case PATTERN_SPOKE:
    n = 5 + (int)(h % 5u);
    break; /* 5..9  */
  case PATTERN_STARPOLY:
    n = 5 + 2 * (int)(h % 3u);
    break; /* 5,7,9 (odd) */
  case PATTERN_WHEEL:
  case PATTERN_MESH:
  case PATTERN_TRIANGLES:
    n = 6 + (int)(h % 3u);
    break; /* 6..8 (denser) */
  case PATTERN_BINARY:
  case PATTERN_CROSS:
    n = 8 + (int)(h % 4u);
    break; /* 8..11 (two parts) */
  default:
    n = 7 + (int)(h % 4u); /* 7..10 */
  }
  if (n > N_STARS_MAX)
    n = N_STARS_MAX;

  place_stars(con->stars, n, region_w, region_h, seed);
  con->n_stars = n;
  con->region_w = region_w;
  con->region_h = region_h;
  con->seed = seed;

  switch (p) {
  case PATTERN_TREE:
    make_tree(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CHAIN:
    make_chain(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_LOOP:
    make_loop(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_SPOKE:
    make_spoke(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_WHEEL:
    make_wheel(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_FAN:
    make_fan(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_STARPOLY:
    make_starpoly(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_NEAREST:
    make_nearest(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_MESH:
    make_mesh(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_ARCH:
    make_arch(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_BINARY:
    make_binary(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CROSS:
    make_cross(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_TRIANGLES:
    make_triangles(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_SPIRAL:
    make_spiral(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CATERPILLAR:
    make_caterpillar(con->stars, n, con->edges, &con->n_edges);
    break;
  default:
    con->n_edges = 0;
  }

  gen_name(hash3(seed, 42, 17), con->name, sizeof con->name);
}

/* Scene — the whole simulated-and-shown sky, as a table of contents: the
 * figure being revealed plus the knobs, view selections and animation clocks
 * that drive it. scene_tick() (§4) is its only per-tick writer; user events set
 * the knobs/selections. Fields group by concept, not by which key changes them.
 */
typedef struct {
  /* WHAT is simulated / shown */
  Constellation con; /* the figure: stars + edges + name          */
  int sky_seed;      /* seed for the faint backdrop sprinkle      */
  /* HOW the user drives it — the tunable simulation knob */
  int speed; /* reveal-animation pace, 1..SPEED_MAX        */
  /* WHAT we are looking at — RENDER selections, not simulation */
  int current_theme;       /* index into themes[] (t/T)                 */
  Pattern current_pattern; /* active topology (n/p); a switch rebuilds  */
  /* WHEN we are — the phase-machine clock + run-state */
  Phase phase;   /* DISCOVER / TRACE / HOLD / FADE            */
  float phase_t; /* seconds into the current phase            */
  float total_t; /* wall-clock accumulator (drives twinkle)   */
  bool paused;   /* freeze the reveal; rendering continues    */
} Scene;

/*
 * scene_rebuild — pick a fresh seed and build a new constellation in
 * the current pattern. Resets the phase machine to DRAW_STARS.
 */
static void scene_rebuild(Scene *s, int region_w, int region_h) {
  int seed = (int)hash3((int)(s->total_t * 1000.0f), region_w,
                        region_h ^ ((int)s->current_pattern << 8));
  constellation_gen(&s->con, s->current_pattern, seed, region_w, region_h);
  s->phase = PHASE_DRAW_STARS;
  s->phase_t = 0.0f;
  s->sky_seed = seed ^ 0x5A5A5A5A;
}

static void scene_init(Scene *s, int region_w, int region_h) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_TREE;
  scene_rebuild(s, region_w, region_h);
}

/*
 * scene_resize_to — region size has changed; throw away the current
 * layout and regenerate so positions are valid for the new region.
 */
static void scene_resize_to(Scene *s, int region_w, int region_h) {
  scene_rebuild(s, region_w, region_h);
}

/*
 * scene_tick — phase-machine update. The reveal_t fields on stars and
 * edges are computed from phase_t / phase_total so pause + resume keep
 * the animation perfectly aligned.
 */

/* Transition the phase machine to `next`, restarting the per-phase clock. */
static void enter_phase(Scene *s, Phase next) {
  s->phase = next;
  s->phase_t = 0.0f;
}

static void scene_tick(Scene *s, float dt) {
  s->total_t += dt; /* wall clock runs even when paused (twinkle) */
  if (s->paused)
    return;

  float speed_mul = (float)s->speed / (float)SPEED_DEF; /* speed knob → rate */
  s->phase_t += dt * speed_mul;

  Constellation *c = &s->con;

  switch (s->phase) {
  case PHASE_DRAW_STARS: { /* stars fade in, one staggered slice each */
    float slice = (c->n_stars > 0) ? (PHASE_STARS_TOTAL / (float)c->n_stars)
                                   : PHASE_STARS_TOTAL;
    for (int i = 0; i < c->n_stars; i++)
      c->stars[i].reveal_t = reveal_at(s->phase_t, i * slice, slice);
    if (s->phase_t >= PHASE_STARS_TOTAL)
      enter_phase(s, PHASE_DRAW_EDGES);
    break;
  }

  case PHASE_DRAW_EDGES: /* edges trace in, one after another */
    for (int i = 0; i < c->n_edges; i++)
      c->edges[i].reveal_t =
          reveal_at(s->phase_t, i * PHASE_EDGE_TIME, PHASE_EDGE_TIME);
    if (s->phase_t >= c->n_edges * PHASE_EDGE_TIME)
      enter_phase(s, PHASE_HOLD);
    break;

  case PHASE_HOLD: /* dwell with the name caption, then fade */
    if (s->phase_t >= PHASE_HOLD_TIME)
      enter_phase(s, PHASE_FADE);
    break;

  case PHASE_FADE: /* brief dwell, then a whole new sky */
    if (s->phase_t >= PHASE_FADE_TIME)
      scene_rebuild(s, c->region_w, c->region_h);
    break;
  }
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. There is no stored cosmetic buffer: the star twinkle is
 * derived at render time from sinf(twinkle_phase + total_t) inside
 * draw_anchor_star (§7), and reveal_t (the fade-in / Bresenham trace progress)
 * is SIMULATION state — it decides WHAT is drawn, not a glow/trail on top. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The post-trace HOLD (the name dwell) and the brief FADE
 * before regenerating are two phases of the scene phase machine, timed by
 * phase_t against PHASE_HOLD_TIME / PHASE_FADE_TIME inside scene_tick() (§4).
 * The pause toggle (Scene.paused) early-returns scene_tick(). Both are woven
 * into the simulation tick. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Reads Scene / Constellation / Screen and the §3 deciders;
 * writes ONLY the ncurses back buffer and the colour-pair table (theme_apply
 * / color_init at init). Never touches simulation state, so a frame can be
 * dropped or re-ordered with no effect on the sim. */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    init_pair(PAIR_BG_STAR, t->bg_star, -1);
    init_pair(PAIR_LINE, t->line, -1);
    init_pair(PAIR_NAME, t->name_color, -1);
    for (int i = 0; i < 4; i++)
      init_pair((short)(PAIR_STAR_BASE + i), t->star[i], -1);
  } else {
    static const short fb_star[4] = {COLOR_WHITE, COLOR_YELLOW, COLOR_CYAN,
                                     COLOR_BLUE};
    init_pair(PAIR_BG_STAR, COLOR_WHITE, -1);
    init_pair(PAIR_LINE, COLOR_WHITE, -1);
    init_pair(PAIR_NAME, COLOR_YELLOW, -1);
    for (int i = 0; i < 4; i++)
      init_pair((short)(PAIR_STAR_BASE + i), fb_star[i], -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport: its full size, the constellation region carved out of
 * it, and the offset where that region sits. Pure presentation geometry — holds
 * NO simulation state — recomputed by screen_layout() at startup and on every
 * SIGWINCH. A star at region cell (x,y) draws at terminal (gy0+y, gx0+x). The
 * region is ~80%×75% of the usable rows (rows 0-1 are the HUD, the last row the
 * hint line) so the name caption has room below it. */
typedef struct {
  int cols, rows;         /* full terminal size (getmaxyx)            */
  int region_w, region_h; /* constellation area carved out of it      */
  int gx0, gy0;           /* its top-left origin on screen            */
} Screen;

static void screen_layout(Screen *s) {
  int top = 2, bottom = s->rows - 1;
  int avail_h = bottom - top;
  int avail_w = s->cols;

  /* Constellation region — 80% wide × 75% tall, centred. Leaves
   * room for the name caption below. */
  int rw = (avail_w * 80) / 100;
  int rh = (avail_h * 75) / 100;
  if (rw < 16)
    rw = (avail_w < 16) ? avail_w : 16;
  if (rh < 8)
    rh = (avail_h < 8) ? avail_h : 8;
  if (rw > avail_w)
    rw = avail_w;
  if (rh > avail_h - 2)
    rh = avail_h - 2;
  if (rh < 4)
    rh = 4;

  s->region_w = rw;
  s->region_h = rh;
  s->gx0 = (avail_w - rw) / 2;
  s->gy0 = top + (avail_h - rh) / 2 - 1;
  if (s->gy0 < top)
    s->gy0 = top;
}

static void screen_init(Screen *s) {
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

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
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
static void draw_backdrop_sky(const Screen *sc, int sky_seed, float total_t) {
  int top = 2, bottom = sc->rows - 1;
  int twinkle_t = (int)(total_t * 1.5f);
  for (int y = top; y < bottom; y++) {
    for (int x = 0; x < sc->cols; x++) {
      uint32_t h = hash3(x, y, sky_seed);
      if ((h % BG_STAR_DENSITY) != 0)
        continue;
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
static void draw_partial_line(int x0, int y0, int x1, int y1, float reveal_t,
                              int gx0, int gy0, int rows, int cols, int pair,
                              int attr, char glyph) {
  int dx_abs = abs(x1 - x0);
  int dy_abs = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int dx = dx_abs;
  int dy = -dy_abs;
  int err = dx + dy;

  /* Of the line's `total_steps` cells, plot only the first `reveal_cells` —
   * the leading fraction reveal_t — so the line appears to grow from (x0,y0).
   */
  int total_steps = (dx_abs > dy_abs ? dx_abs : dy_abs) + 1;
  int reveal_cells = (reveal_t > 0.999f)
                         ? total_steps
                         : (int)ceilf(reveal_t * (float)total_steps);
  if (reveal_cells > total_steps)
    reveal_cells = total_steps;

  int x = x0, y = y0;
  int step = 0;
  while (step < reveal_cells) {
    bool is_endpoint = (x == x0 && y == y0) || (x == x1 && y == y1);
    if (!is_endpoint) {
      int sy_screen = gy0 + y;
      int sx_screen = gx0 + x;
      if (sy_screen >= 2 && sy_screen < rows - 1 && sx_screen >= 0 &&
          sx_screen < cols) {
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy_screen, sx_screen, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
      }
    }
    if (x == x1 && y == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
    step++;
  }
}

/*
 * draw_anchor_star — bright '*' or 'O' at the star's position. The
 * star reveal_t fades the brightness in: 0 = invisible, 0.5 = dim,
 * 1.0 = full brightness with twinkle.
 */
static void draw_anchor_star(const Screen *sc, const Star *star,
                             float total_t) {
  if (star->reveal_t <= 0.05f)
    return;
  int sy = sc->gy0 + star->y;
  int sx = sc->gx0 + star->x;
  if (sy < 2 || sy >= sc->rows - 1)
    return;
  if (sx < 0 || sx >= sc->cols)
    return;

  int pair = PAIR_STAR_BASE + (star->color_idx & 3);
  int attr;
  char glyph;

  if (star->reveal_t < 0.40f) {
    attr = A_DIM;
    glyph = '.';
  } else if (star->reveal_t < 0.85f) {
    attr = A_NORMAL;
    glyph = '*';
  } else {
    /* Fully revealed — twinkle. The star pulses with a
     * sinusoid keyed to its individual phase so neighbours
     * twinkle out of sync. */
    float phase = (float)star->twinkle_phase / 255.0f * 2.0f * (float)M_PI;
    float b =
        0.5f + 0.5f * sinf(2.0f * (float)M_PI * TWINKLE_HZ * total_t + phase);
    attr = A_BOLD;
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
static void draw_name_label(const Screen *sc, const Constellation *con,
                            Phase phase, float phase_t) {
  if (phase != PHASE_HOLD && phase != PHASE_FADE)
    return;

  int len = (int)strlen(con->name);
  int row = sc->gy0 + sc->region_h + 0;
  if (row >= sc->rows - 1)
    row = sc->rows - 2;
  int col = sc->gx0 + (sc->region_w - len) / 2;
  if (col < 0)
    col = 0;

  /* Fade in over first second of HOLD; fade out during FADE. */
  float intensity;
  if (phase == PHASE_HOLD) {
    intensity = (phase_t < 1.0f) ? phase_t : 1.0f;
  } else {
    intensity = 1.0f - (phase_t / PHASE_FADE_TIME);
    if (intensity < 0.0f)
      intensity = 0.0f;
  }
  if (intensity < 0.05f)
    return;

  int attr = (intensity > 0.7f) ? A_BOLD : A_NORMAL;
  attron(COLOR_PAIR(PAIR_NAME) | attr);
  mvprintw(row, col, "%s", con->name);
  attroff(COLOR_PAIR(PAIR_NAME) | attr);

  /* Underline that grows in during HOLD. */
  if (phase == PHASE_HOLD) {
    float ut = phase_t / 1.5f;
    if (ut > 1.0f)
      ut = 1.0f;
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

/* Render leaf, NOT a tick orchestrator: takes only what it draws — the figure
 * (Constellation), the sky seed, and the animation clocks/phase — never the
 * whole Scene. */
static void scene_draw(const Screen *sc, const Constellation *con, int sky_seed,
                       float total_t, Phase phase, float phase_t) {
  /* Backdrop first — faint sky everywhere. */
  draw_backdrop_sky(sc, sky_seed, total_t);

  /* Edges first (so anchor stars overdraw at vertices). */
  for (int i = 0; i < con->n_edges; i++) {
    const Edge *e = &con->edges[i];
    if (e->reveal_t <= 0.001f)
      continue;
    const Star *a = &con->stars[e->from];
    const Star *b = &con->stars[e->to];
    char glyph = line_glyph_for(b->x - a->x, b->y - a->y);
    int attr = (e->reveal_t > 0.99f) ? A_DIM : A_NORMAL;
    draw_partial_line(a->x, a->y, b->x, b->y, e->reveal_t, sc->gx0, sc->gy0,
                      sc->rows, sc->cols, PAIR_LINE, attr, glyph);
  }

  /* Anchor stars (overdrawing edge tails at vertices). */
  for (int i = 0; i < con->n_stars; i++) {
    draw_anchor_star(sc, &con->stars[i], total_t);
  }

  /* Name caption during HOLD / FADE. */
  draw_name_label(sc, con, phase, phase_t);
}

/* Row 0 right — primary status: fps, sim Hz, phase/pause, speed. Right-aligned.
 */
static void draw_status_line(const Screen *sc, const Scene *s, double fps,
                             int sim_fps) {
  const char *phase = s->paused ? "PAUSED   " : phase_name(s->phase);
  char buf[HUD_COLS + 1];
  snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ", fps, sim_fps,
           phase, s->speed);
  int hx = sc->cols - (int)strlen(buf);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (with n/N counter), theme, the 4 anchor-star tint swatch, and
 * the constellation stats (N stars, E edges, name). Fixed left-aligned layout.
 */
static void draw_param_line(const Scene *s) {
  const Constellation *c = &s->con;
  int x = 1;

  char pbuf[40];
  snprintf(pbuf, sizeof pbuf, " pattern:%s %2d/%-2d ",
           pattern_name(s->current_pattern), (int)s->current_pattern + 1,
           (int)N_PATTERNS);
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(1, x, "%s", pbuf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  x += (int)strlen(pbuf);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  x += 17;

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, x, " stars:");
  attroff(COLOR_PAIR(PAIR_HUD));
  x += 7;
  for (int i = 0; i < 4; i++) { /* anchor-star tint swatch */
    attron(COLOR_PAIR(PAIR_STAR_BASE + i) | A_BOLD);
    mvaddch(1, x, '*');
    attroff(COLOR_PAIR(PAIR_STAR_BASE + i) | A_BOLD);
    x++;
  }
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, x, "  N=%d  E=%d  name:\"%s\" ", c->n_stars, c->n_edges, c->name);
  attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen "
           " q:quit ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, phase, run-state — so
 * narrowing it would mean passing half a dozen scalars. A const read can't
 * re-couple the layers; scene_draw and the leaf draws stay narrow.
 * Reads as: draw the sky, then lay the HUD over it. */
static void screen_draw(const Screen *sc, const Scene *s, double fps,
                        int sim_fps) {
  erase();
  scene_draw(sc, &s->con, s->sky_seed, s->total_t, s->phase, s->phase_t);

  draw_status_line(sc, s, fps, sim_fps);

  /* Row 0 left — title. */
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 1, " PROCEDURAL CONSTELLATION ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  draw_param_line(s);
  draw_hint(sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

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
 * explicitly. Only init + the main loop touch it whole. The fixed-timestep rate
 * (sim_fps) decouples animation speed from frame rate (Fiedler, "Fix Your
 * Timestep!"; §8 main()). */
typedef struct {
  /* the two worlds it binds */
  Scene scene;   /* WHAT is simulated + shown        */
  Screen screen; /* WHERE it is drawn                */
  /* loop control */
  int sim_fps; /* fixed-timestep rate, SIM_FPS_* Hz*/
  /* volatile sig_atomic_t: written from signal handlers, so the compiler
   * must re-read them each loop and the write is atomic w.r.t. the
   * interrupted code. */
  volatile sig_atomic_t running;     /* cleared by SIGINT/SIGTERM → exit */
  volatile sig_atomic_t need_resize; /* set by SIGWINCH, served next loop*/
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize_to(&app->scene, app->screen.region_w, app->screen.region_h);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
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
  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.region_w, app->screen.region_h);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
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
