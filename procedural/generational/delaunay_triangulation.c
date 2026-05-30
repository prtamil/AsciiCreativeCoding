/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * delaunay_triangulation.c — Bowyer-Watson Delaunay, animated.
 *
 * DEMO: Seed points drop onto the map one at a time (6–30, set by the
 *       active preset). Each is INSERTED into a growing Delaunay
 *       triangulation by Bowyer-Watson's incremental method: the new
 *       point invalidates every triangle whose circumscribed circle
 *       contains it; the resulting "cavity" is re-triangulated by
 *       joining the new point to every boundary edge. New triangles
 *       flash their edges briefly before settling. After all points are
 *       inserted, the super-triangle scaffolding is hidden, leaving a
 *       clean Delaunay mesh — every triangle's circumcircle contains no
 *       other vertex. The build then HOLDS on the finished mesh: no
 *       auto-restart, no auto style switch. n/p switches preset (15
 *       styles, varying palette + density); r rebuilds the same one.
 *
 * Study alongside: ./voronoi_region_map.c — the GEOMETRIC DUAL of
 *       Delaunay. Voronoi assigns each cell to its nearest seed;
 *       Delaunay connects seeds whose Voronoi regions touch. The
 *       same point set produces both — try the same seed positions
 *       and overlay the diagrams in your head.
 *
 *       geometry/delaunay_triangulation.c — same Bowyer-Watson, but
 *       framed as an algorithm showcase: random points + an explicit
 *       SHOWCASE phase that draws every triangle's circumcircle and
 *       verifies the empty-circumcircle property.  This file tells
 *       the story; that one proves the invariant.
 *
 * Section map (layered — see ARCHITECTURE below):
 *   §1 config   — map size, storage caps, 15 presets
 *   §2 clock    — monotonic timer + sleep (PERFORMANCE / DELAYS infra)
 *   §3 color    — HUD pairs + preset palette (RENDER setup)
 *   §5 dt       — STATE · pure LOGIC (geometry) · EFFECTS · SIMULATION
 *   §6 scene    — orchestration: the one place the layers combine + DONE
 *   §7 render   — state → ASCII edges/points (pure reads), HUD
 *   §8 app      — PERFORMANCE loop, signals, resize, input
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   n / p      next / previous preset (15 styles; each builds then holds)
 *   r          rebuild (same preset)
 *   + / =      faster (one point inserted sooner)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra delaunay_triangulation.c \
 *       -o delaunay -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Bowyer-Watson incremental Delaunay triangulation
 *                  (Bowyer 1981, Watson 1981). Starts with a single
 *                  "super-triangle" large enough to enclose every
 *                  input point. For each point P:
 *                    1. Find every triangle whose CIRCUMSCRIBED
 *                       CIRCLE contains P — these are the "bad"
 *                       triangles.
 *                    2. Collect the edges of the bad triangles. An
 *                       edge shared by two bad triangles is INTERIOR
 *                       to the cavity — discard it. An edge shared
 *                       by exactly one bad triangle is on the cavity
 *                       BOUNDARY — keep it.
 *                    3. Remove all bad triangles.
 *                    4. For every boundary edge, create a new
 *                       triangle (boundary edge endpoints + P).
 *                  At the end, remove every triangle that touches a
 *                  super-triangle vertex. The remaining triangles are
 *                  the Delaunay triangulation of the input.
 *
 *                  The Delaunay property: for every triangle T, T's
 *                  circumscribed circle contains NO other input
 *                  point. Bowyer-Watson maintains this invariant
 *                  because step 1 finds exactly the triangles
 *                  violating it for the new point, and step 4
 *                  replaces them with triangles that don't.
 *
 *                  Complexity: O(N · K) per insertion where K is the
 *                  number of bad triangles (typically O(1) on average,
 *                  O(N) in pathological cases). Total: O(N² log N) in
 *                  the worst case, O(N log N) on random inputs.
 *
 * Data-structure : Static arrays of points and triangles. Each
 *                  triangle stores 3 vertex indices into points[]
 *                  plus a "valid" flag (a triangle is logically
 *                  removed by clearing the flag, not erasing the
 *                  slot, so indices stay stable). New triangles get
 *                  appended to the end. Edge tracking uses a tiny
 *                  scratch buffer per insertion.
 *
 * Rendering      : ASCII only. Edges are drawn cell-by-cell with
 *                  Bresenham's line algorithm; the per-edge glyph
 *                  is chosen by the line's dominant slope:
 *                    |dx| ≫ |dy|  →  '-'  (mostly horizontal)
 *                    |dy| ≫ |dx|  →  '|'  (mostly vertical)
 *                    dx·dy > 0    →  '\\' (down-right diagonal)
 *                    dx·dy < 0    →  '/'  (up-right diagonal)
 *                  Points draw as '@' BOLD on top. Newly-added
 *                  triangles paint a per-edge new_glow that fades
 *                  over ~0.5 s — accent colour pops on every insert.
 *
 * Performance    : 12 points, ~30 triangles, edges ~30 cells avg.
 *                  Per-frame edge render ≈ 1000 cell paints — trivial.
 *                  Insertion is O(1) per bad triangle plus a 30-entry
 *                  edge scan, ≈ 1 ms per point on modern hardware.
 *                  No allocation post-init.
 *
 * References     : ALGORITHM — Delaunay, Bowyer-Watson, the geometric
 *                  predicates this code rests on
 *                  • Bowyer, A. (1981) — "Computing Dirichlet
 *                    tessellations", The Computer Journal 24(2). One of
 *                    the two namesake papers for the incremental method.
 *                  • Watson, D.F. (1981) — "Computing the n-dimensional
 *                    Delaunay tessellation with application to Voronoi
 *                    polytopes", The Computer Journal 24(2). The other.
 *                  • de Berg, Cheong, van Kreveld & Overmars —
 *                    "Computational Geometry: Algorithms and Applications"
 *                    (3rd ed., Springer 2008), ch. 9. The standard
 *                    textbook treatment of Delaunay/Voronoi and the
 *                    empty-circumcircle property.
 *                  • Guibas & Stolfi (1985) — "Primitives for the
 *                    Manipulation of General Subdivisions and the
 *                    Computation of Voronoi Diagrams", ACM TOG 4(2).
 *                    Origin of the InCircle / orientation predicates that
 *                    in_circumcircle() implements as a determinant.
 *                  • Shewchuk, J.R. (1997) — "Adaptive Precision
 *                    Floating-Point Arithmetic and Fast Robust Geometric
 *                    Predicates". Why those determinants are sign-fragile
 *                    and how production code keeps them exact (we sidestep
 *                    it with 64-bit integer coordinates).
 *                  • Wikipedia — "Bowyer–Watson algorithm" — concise
 *                    pseudocode mirror of the insertion loop here:
 *                    https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
 *                  • Inigo Quilez — "Voronoi/Delaunay duality":
 *                    https://iquilezles.org/articles/voronoilines/
 *
 *                  RENDERING — lines + terminal output
 *                  • Bresenham, J.E. (1965) — "Algorithm for computer
 *                    control of a digital plotter", IBM Systems Journal
 *                    4(1). The integer line-stepping used per edge.
 *                  • Padala, Pradeep — "NCURSES Programming HOWTO" (TLDP)
 *                    — the erase→draw→doupdate frame model, colour pairs,
 *                    and non-blocking input this file uses.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Connect points into triangles such that no triangle's circumscribed
 * circle contains any other point. That single rule produces the
 * "fattest possible" triangles — the Delaunay triangulation maximises
 * the minimum angle and avoids skinny slivers. It's the most useful
 * triangulation in graphics, finite-element analysis, terrain
 * meshing, anywhere triangles need to be well-shaped.
 *
 * The Bowyer-Watson trick: build it incrementally. Each new point
 * BREAKS a few existing triangles (those whose circumcircle contains
 * the point). Demolish them, then re-fill the gap with triangles
 * that all touch the new point.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you're a building inspector. Every triangle has a circular
 * "no-other-point" zone (its circumcircle). When a new building (a
 * new point) is added that lands inside someone's zone, those
 * triangles are condemned. You demolish them, leaving a polygonal
 * empty lot. Then you re-triangulate that lot by connecting the new
 * building to each corner of the lot. Every new triangle has the
 * new building as one of its vertices; the triangulation invariant
 * is restored.
 *
 * Visible layers:
 *   1. POINTS '@' (preset point colour, bold) — the seed set, growing
 *      to the preset's target count over the BUILDING phase.
 *   2. EDGES (preset edge colour, slope-aware glyphs '-', '|', '/',
 *      '\\') — each triangle's three sides drawn cell-by-cell.
 *   3. NEW EDGE FLASH (preset flash colour) — edges that were just
 *      created during the most recent insertion glow brightly for
 *      ~0.5 s before fading.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Place 3 super-triangle vertices FAR outside the map
 *     (-5W, -5H), (5W, -5H), (W/2, 5H). One initial triangle joins
 *     them. n_points = 3, n_tris = 1, n_real_points = 0.
 *  2. INSERT NEXT POINT P (one per scene_tick after a cooldown):
 *     a. Pick a random in-bounds (x, y) — with min-spacing rejection
 *        against existing real points.
 *     b. For each valid triangle T: if P lies inside T's
 *        circumcircle, mark T as "bad".
 *     c. Walk every edge of every bad triangle. An edge that appears
 *        twice (shared by two bad triangles) is INTERIOR — skip it.
 *        An edge that appears once is on the BOUNDARY — record it.
 *     d. Mark all bad triangles invalid.
 *     e. For every boundary edge (e0, e1), append a new triangle
 *        (e0, e1, P_idx) with new_glow = 1.0.
 *  3. Repeat until n_real_points reaches the preset's target count.
 *  4. DONE: hold on the finished triangulation. No auto-reset and no auto
 *     style switch — n/p (switch preset) or r (same preset) rebuilds.
 *
 *  Rendering simply iterates valid triangles whose all 3 vertices
 *  are real (index ≥ 3). Triangles touching a super-triangle vertex
 *  exist but are FILTERED at render time. After full insertion,
 *  every renderable triangle is part of the Delaunay triangulation
 *  of the real points.
 *
 * KEY FORMULAS
 * ────────────
 *  Triangle area sign            : S = (Bx−Ax)(Cy−Ay) − (Cx−Ax)(By−Ay)
 *                                  (S > 0 ⇒ CCW in math coords)
 *  In-circumcircle predicate     : 3×3 determinant (orientation-aware):
 *                                    | ax  ay  ax²+ay² |
 *                                    | bx  by  bx²+by² |
 *                                    | cx  cy  cx²+cy² |
 *                                  with (ax, ay) = (A.x−P.x, A.y−P.y),
 *                                  etc. Sign convention: same as S.
 *  Edge identity                 : (min(p1,p2), max(p1,p2)) — pair
 *                                  of point indices, order-normalised
 *                                  so duplicate-detection works.
 *  Slope glyph (dominant axis)   : see Rendering above.
 *  Bresenham step                : standard integer DDA.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SUPER-TRIANGLE PLACEMENT. Vertices must be far enough that
 *    EVERY input point is strictly inside. Too small → the in-circle
 *    test fails for some valid input. Too large → numerical
 *    overflow in the determinant. (-5W, -5H) etc. is a safe choice
 *    that easily fits int math up to W,H = 200.
 *
 *  • TIE-BREAKING. When P lands exactly on a circumcircle, the
 *    determinant is zero — the algorithm has a choice. Strictly we
 *    should treat zero as "not inside" (so the existing triangle is
 *    preferred), but for non-degenerate random input, ties are
 *    vanishingly rare. We treat det > 0 as "inside" to break ties
 *    consistently.
 *
 *  • ORIENTATION. The in-circle determinant assumes the triangle is
 *    CCW (math, y-up). With screen coords (y-down) "CCW math" is
 *    "CW screen". Easiest: detect the triangle's orientation each
 *    test and flip the predicate. We do that — see in_circumcircle.
 *
 *  • EDGE NORMALISATION. When matching edges across bad triangles,
 *    use min(p1, p2)/max(p1, p2) as the canonical key. Otherwise
 *    edge (5, 7) of one triangle won't match edge (7, 5) of another.
 *
 *  • SUPER-TRIANGLE FILTERING. Triangles touching any of indices
 *    0, 1, 2 are "scaffolding" and should not render. If they
 *    accidentally do (because filter is missing), you'll see big
 *    triangles with edges leaving the map — a clear visual bug.
 *
 *  • STORAGE OF REMOVED TRIANGLES. We keep them around with valid=
 *    false rather than compacting the array. This keeps indices
 *    stable across insertions (important if anything else holds an
 *    index — though we don't here, it's good hygiene).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial state: 3 super-triangle points (offscreen) and 1
 *    triangle. 0 real points visible.
 *  • After K insertions: between 2K-1 and 2K+1 valid triangles
 *    typically (depends on point positions). Visible triangles ≤
 *    valid triangles since super-tri triangles are filtered.
 *  • Final triangulation is convex: the boundary edges form a
 *    convex polygon (the convex hull of the input). If you see
 *    concave edges or holes, the cavity-walking logic is buggy.
 *  • Delaunay property — sanity test: pick any visible triangle's
 *    three vertices, compute its circumcircle, verify NO other
 *    input point is inside it. (We don't run this check, but you
 *    could add it as a debug pass.)
 *  • Edge count: a Delaunay triangulation of n points in general
 *    position has between 2n−5 and 3n−6 edges. For n=12 that's
 *    19 to 30 edges. Wildly outside that range = bug.
 *  • A preset's palette and point glyph are pure presentation: for a
 *    fixed point count the algorithm is colour-independent. (Presets
 *    also vary the point count, which of course changes the mesh.)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The program is built from SIX layers, kept in separate labelled places so
 * each can be read without the others — and so a change to one (say the
 * glow) provably cannot break another (the mesh).
 *
 *   LAYER         WHAT IT IS                          WHERE       MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   LOGIC         pure geometry predicates (signed    §5.2        nothing
 *                 area, in-circumcircle) + edge_glyph             (by value)
 *   SIMULATION    Bowyer-Watson: seed, pick a point,  §5.4 §6     Delaunay
 *                 insert (find cavity, retriangulate)             points/tris
 *   EFFECTS       per-triangle new_glow flash; its    §5.3 §6     new_glow only
 *                 exponential fade
 *   RENDER        mesh → ASCII edges/points → ncurses §3 §7       screen only
 *                 (Bresenham); pure reads of state
 *   DELAYS        pause, the insert cooldown, and the §6 §8       timers /
 *                 BUILDING→DONE hold                              state
 *   PERFORMANCE   fixed-timestep accumulator + dt cap §8          —
 *                 + frame-rate cap
 *
 * SIGNATURE CONVENTION — a call site tells you whether state changes:
 *   • const Delaunay * / const Scene * (or Point by value) → a pure read
 *     (LOGIC or RENDER).
 *   • non-const pointer → an EFFECT: the function mutates what it points at.
 *
 * THE ONE COMBINING POINT is scene_tick() (§6): every per-tick concern
 * appears there once, in order — DELAY guard → EFFECTS fade → SIMULATION
 * step → DELAY state transition. Nothing else advances state; the render
 * path (§7) only reads.
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    /* Max real points across all presets, + 3 super-triangle scaffold
     * points. The per-preset point count (mesh density) lives in
     * presets[]; this only bounds the storage arrays. */
    N_REAL_MAX        =  32,
    SUPER_TRI_VERTS   =   3,
    MAX_POINTS        = N_REAL_MAX + SUPER_TRI_VERTS,

    /* Triangle + scratch storage, sized for the densest preset. Triangles
     * are never compacted (removed = valid flag cleared, slot kept for
     * index stability), so MAX_TRIS bounds the TOTAL ever created — about
     * 3–6× the point count for random inputs, so 512 is ample for 32. */
    MAX_TRIS          = 512,

    /* Per-insertion scratch: bad-triangle cap + edge dedup buffer. Both
     * sized so neither cap is ever hit at N_REAL_MAX (hitting one would
     * silently drop cavity edges and corrupt the mesh). */
    MAX_BAD_TRIS      = 128,
    MAX_EDGE_BUF      = 256,

    /* Min spacing between real points for visual balance. */
    MIN_POINT_DIST    =  10,
    POINT_PLACE_TRIES = 100,

    /* Ticks between successive insertions during BUILDING. */
    INSERT_HOLD_TICKS_DEF = 18,     /* 18/60 ≈ 0.3 s @ 60 Hz */
    INSERT_HOLD_TICKS_MIN =   1,
    INSERT_HOLD_TICKS_MAX = 240,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md.
     * (3 is intentionally skipped to keep edge/point/flash on 4..6.) */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_EDGE         =   4,        /* triangle edges                  */
    PAIR_POINT        =   5,        /* '@' point glyph                 */
    PAIR_FLASH        =   6,        /* new edge flash                  */
};

/* Glow decay rates. */
#define EDGE_GLOW_DECAY     2.5f    /* new-edge flash fade ~0.7 s */
#define GLOW_THRESHOLD      0.05f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Preset — one complete visual style. Switching preset (n/p) changes how
 * the mesh LOOKS (palette, point glyph) and its DENSITY (how many points
 * are inserted) — but not the Bowyer-Watson algorithm. Each preset builds
 * once and then holds; there is no automatic switching.
 *
 *   edge/point/flash  xterm-256 colours for the three drawn layers, all in
 *                     the bright half so they read on the default bg
 *   n_points          real points to insert (mesh density); ≤ N_REAL_MAX
 *   point_glyph       ASCII char drawn at each seed point
 */
typedef struct {
    const char *name;
    short       edge, point, flash;
    int         n_points;
    char        point_glyph;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
  /*  name        edge point flash  pts glyph */
  { "CLASSIC",   51, 231, 220,  12, '@' },   /* cyan / white / gold        */
  { "MATRIX",    46, 118, 226,  16, '#' },   /* greens                     */
  { "NOVA",     201, 219, 226,  10, '*' },   /* magenta / pink / yellow    */
  { "MONO",     250, 255, 244,  14, 'o' },   /* greyscale                  */
  { "OCEAN",     39, 159,  51,  18, '@' },   /* navy / ice / cyan          */
  { "FIRE",     208, 226, 196,  12, '*' },   /* orange / yellow / red      */
  { "FOREST",    82, 154, 226,  20, '+' },   /* greens                     */
  { "DESERT",   178, 230, 220,  14, 'o' },   /* sand / cream / gold        */
  { "ARCTIC",   159, 231,  87,  16, '@' },   /* ice / white / cyan         */
  { "AMETHYST", 141, 219, 201,  22, '*' },   /* violet / pink / magenta    */
  { "EMBER",    166, 214, 202,  10, '#' },   /* deep orange embers         */
  { "NEON",      51, 201, 226,  24, '@' },   /* cyan / magenta / yellow    */
  { "SPARSE",   244, 252, 220,   6, 'O' },   /* few points, big triangles  */
  { "DENSE",     45, 195,  51,  30, '.' },   /* many points, fine mesh     */
  { "REEF",     211, 230, 207,  18, '@' },   /* coral pink / cream         */
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

/* preset_apply — bind the preset's palette into the edge/point/flash pairs. */
static void preset_apply(int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    if (COLORS >= 256) {
        const Preset *p = &presets[idx];
        init_pair(PAIR_EDGE,  p->edge,  -1);
        init_pair(PAIR_POINT, p->point, -1);
        init_pair(PAIR_FLASH, p->flash, -1);
    } else {
        init_pair(PAIR_EDGE,  COLOR_CYAN,    -1);
        init_pair(PAIR_POINT, COLOR_WHITE,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW,  -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    preset_apply(0);
}

/* ===================================================================== */
/* §5  dt — STATE · LOGIC · EFFECTS · SIMULATION                          */
/* ===================================================================== */
/*
 * The heart of the program, split into four layers (see ARCHITECTURE).
 * Reading order is dependency order: each layer uses only the ones above.
 *   5.1 STATE       the data the triangulation lives in
 *   5.2 LOGIC       pure geometry — no state, no side effects
 *   5.3 EFFECTS     the cosmetic glow fade — never read by LOGIC
 *   5.4 SIMULATION  Bowyer-Watson; advances STATE using LOGIC, sets EFFECTS
 */

/* ── 5.1 STATE ───────────────────────────────────────────────────────── */

/*
 * Point — a vertex, as INTEGER cell coordinates. WHY integer (not float):
 * the in-circumcircle test is a sign-of-determinant decision (§5.2); with
 * integer inputs and 64-bit intermediates that sign is computed EXACTLY,
 * so the algorithm can't flip on floating-point round-off near a
 * cocircular configuration. Index convention: 0..2 are the super-triangle
 * scaffold (placed far off-screen), 3..MAX_POINTS-1 are real input points —
 * the "≥ 3" test is how the renderer tells real from scaffold.
 */
typedef struct {
    int x, y;          /* cell column, row (grid space)                    */
} Point;

/*
 * Triangle — one face of the mesh: three vertices stored as INDICES into
 * points[], not coordinates. WHY indices: an edge shared by two triangles
 * is then the same pair of ints in both, so the cavity walk in
 * dt_insert_point can match shared edges by value (and a point moving would
 * update everywhere at once — though here points never move).
 *
 * WHY a valid flag instead of deleting: removed triangles keep their slot
 * (valid=false) so every other triangle's index stays put across an
 * insertion — no array compaction, no dangling indices. n_tris therefore
 * counts slots EVER used, not live triangles (the HUD counts live ones).
 */
typedef struct {
    int   p[3];        /* SIMULATION: vertex indices into points[]          */
    bool  valid;       /* SIMULATION: false = logically removed (slot kept) */
    float new_glow;    /* EFFECTS: 1.0 on creation, faded by fx_decay (~0.7s)*/
} Triangle;

/* triangle_is_real — true when all three vertices are real input points
 * (index ≥ SUPER_TRI_VERTS); false for any triangle still touching the
 * super-triangle scaffold. This is the filter that hides the scaffold —
 * named once here, reused by every render and HUD-count pass. */
static inline bool triangle_is_real(const Triangle *t)
{
    return t->p[0] >= SUPER_TRI_VERTS
        && t->p[1] >= SUPER_TRI_VERTS
        && t->p[2] >= SUPER_TRI_VERTS;
}

/* triangle_is_flashing — true while a freshly-created triangle's new-edge
 * glow is still above the visibility threshold (drawn in the accent colour). */
static inline bool triangle_is_flashing(const Triangle *t)
{
    return t->new_glow > GLOW_THRESHOLD;
}

/*
 * Delaunay — the whole triangulation being grown by Bowyer-Watson.
 *
 * WHY a super-triangle (points 0..2): the incremental method needs an
 * existing triangulation to insert INTO, so it begins with one huge
 * triangle that encloses the entire map. Every real point lands inside it;
 * at the end, triangles touching a scaffold vertex are filtered out at
 * render time, leaving the Delaunay triangulation of just the real points.
 *
 * WHY points[] and triangles[] are append-only (never compacted): indices
 * must stay stable across an insertion (triangles reference vertices by
 * index, edges are matched by index), so removal clears a flag rather than
 * erasing a slot. n_points / n_tris therefore only ever grow within a run.
 *
 * Ref: Bowyer (1981), Watson (1981); see the References block.
 */
typedef struct {
    /* SIMULATION state — the mesh itself (the only fields LOGIC reads). */
    int       w, h;                /* map size in cells                      */
    Point     points[MAX_POINTS];  /* [0..2] super-tri scaffold, then real   */
    int       n_points;            /* points placed so far (3 + n_real)      */
    int       n_real;              /* real points inserted (0..target)       */
    int       target;             /* real points this run will insert (preset)*/
    Triangle  triangles[MAX_TRIS]; /* every triangle ever created; see valid */
    int       n_tris;              /* slots used (live + removed)            */

    int       insert_cooldown;     /* DELAYS: ticks until the next insertion */
} Delaunay;

/* ── 5.2 LOGIC (pure — Point by value, mutates nothing) ──────────────── *
 * Integer geometry predicates. 64-bit intermediates throughout so
 * int*int*int can't overflow on a 200×200 grid. These take their inputs
 * by value and touch no state — the part of the program you can verify in
 * isolation, and that no EFFECT or RENDER bug can reach.
 * ── ─────────────────────────────────────────────────────────────────── */

/* clamp_int — confine v to [lo, hi]. Names the bound-application the keyboard
 * handler does to every tunable knob. */
static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * tri_signed_area_2x — returns 2 × signed area of triangle (a,b,c).
 *   > 0  : CCW in math (y-up) coords    — equivalently CW in screen
 *   < 0  : CW in math                   — equivalently CCW in screen
 *   = 0  : degenerate (colinear)
 */
static long long tri_signed_area_2x(Point a, Point b, Point c)
{
    return (long long)(b.x - a.x) * (c.y - a.y)
         - (long long)(c.x - a.x) * (b.y - a.y);
}

/*
 * in_circumcircle — does point P lie strictly inside the circumcircle
 * of triangle (a, b, c)?
 *
 * Computed as a 3×3 determinant of (vertex relative to P, plus its
 * squared length). The sign convention depends on the triangle's
 * orientation, so we compute the area first and flip the sign of the
 * predicate accordingly. Robust for integer inputs up to ~10⁵.
 */
static bool in_circumcircle(Point p, Point a, Point b, Point c)
{
    long long sign_area = tri_signed_area_2x(a, b, c);
    if (sign_area == 0) return false;       /* degenerate */

    long long ax = a.x - p.x, ay = a.y - p.y;
    long long bx = b.x - p.x, by = b.y - p.y;
    long long cx = c.x - p.x, cy = c.y - p.y;
    long long as_ = ax * ax + ay * ay;
    long long bs_ = bx * bx + by * by;
    long long cs_ = cx * cx + cy * cy;

    /* 3×3 determinant expansion along the first column. */
    long long det = ax * (by * cs_ - bs_ * cy)
                  - ay * (bx * cs_ - bs_ * cx)
                  + as_ * (bx * cy - by * cx);

    return (sign_area > 0) ? (det > 0) : (det < 0);
}

/* ── 5.3 EFFECTS (cosmetic glow — writes ONLY the new_glow field) ─────── *
 * The flash STATE is set inline where the SIMULATION creates a triangle
 * (Triangle.new_glow = 1); this is the one place it FADES. LOGIC never reads
 * a glow, so a bug here can dim or brighten the screen but can never corrupt
 * the mesh.
 * ── ─────────────────────────────────────────────────────────────────── */

/* fx_decay — exponential fade of the per-triangle new-edge glow, once per sim
 * tick. Pure cosmetic time-evolution; dt is the fixed timestep. */
static void fx_decay(Delaunay *d, float dt)
{
    float edge_k = expf(-EDGE_GLOW_DECAY * dt);
    for (int i = 0; i < d->n_tris; i++)
        d->triangles[i].new_glow *= edge_k;
}

/* ── 5.4 SIMULATION (Bowyer-Watson — mutates points/triangles) ───────── */

/*
 * CavityEdge — one edge of the bad-triangle cavity: the normalised pair
 * (a ≤ b) of point indices plus how many bad triangles share it. After the
 * cavity is walked, count == 1 ⇒ BOUNDARY edge (the cavity outline, kept),
 * count == 2 ⇒ INTERIOR edge (shared by two bad triangles, discarded).
 */
typedef struct { int a, b, count; } CavityEdge;

/* place_super_triangle — scaffold vertices 0..2, placed far outside the map
 * so every in-bounds point lands strictly inside. Far enough to enclose,
 * small enough that the int64 in-circle determinant cannot overflow. */
static void place_super_triangle(Delaunay *d)
{
    d->points[0] = (Point){ -5 * d->w,     -5 * d->h };
    d->points[1] = (Point){  5 * d->w,     -5 * d->h };
    d->points[2] = (Point){      d->w / 2,  5 * d->h };
    d->n_points = SUPER_TRI_VERTS;
}

/* seed_initial_triangle — the single triangle (0,1,2) Bowyer-Watson inserts
 * INTO. Vertex order is CCW in y-up math coords (CW on screen, but the
 * scaffold verts are off-screen so on-screen orientation doesn't matter). */
static void seed_initial_triangle(Delaunay *d)
{
    d->triangles[0] = (Triangle){
        .p = {0, 1, 2},
        .valid = true,
        .new_glow = 0.0f,
    };
    d->n_tris = 1;
}

/*
 * dt_reset — clear everything, place super-triangle, no real points.
 *
 * Super-triangle vertices sit at (−5W, −5H), (5W, −5H), (W/2, 5H) —
 * far enough that any in-bounds point is well inside, small enough
 * that the determinant doesn't overflow int64.
 */
static void dt_reset(Delaunay *d, int w, int h, int target)
{
    d->w = w;
    d->h = h;
    d->n_points = 0;
    d->n_real   = 0;
    d->target   = target;
    d->n_tris   = 0;
    d->insert_cooldown = 0;

    place_super_triangle(d);             /* scaffold vertices 0..2            */
    seed_initial_triangle(d);            /* the one triangle to insert into   */
}

/* too_close_to_existing — true if (x, y) lies within MIN_POINT_DIST of any
 * real point already placed. The min-spacing rejection test, in squared
 * distance so no sqrt is needed. */
static bool too_close_to_existing(const Delaunay *d, int x, int y)
{
    for (int i = SUPER_TRI_VERTS; i < d->n_points; i++) {
        int dx = x - d->points[i].x;
        int dy = y - d->points[i].y;
        if (dx * dx + dy * dy < MIN_POINT_DIST * MIN_POINT_DIST)
            return true;
    }
    return false;
}

/*
 * dt_pick_real_point — pick a random in-bounds (x, y) with min-distance
 * rejection against existing real points.
 *
 * Falls back to the last candidate even if it's too close — for our
 * map sizes and preset point counts we always find a good spot, but
 * the fallback keeps the algorithm running on extreme inputs.
 */
static void dt_pick_real_point(const Delaunay *d, int *out_x, int *out_y)
{
    int best_x = 0, best_y = 0;
    int margin = 2;     /* keep points off the very edge */

    for (int attempt = 0; attempt < POINT_PLACE_TRIES; attempt++) {
        int x = margin + (rand() % (d->w - 2 * margin));
        int y = margin + (rand() % (d->h - 2 * margin));
        best_x = x; best_y = y;
        if (!too_close_to_existing(d, x, y)) break;
    }
    *out_x = best_x;
    *out_y = best_y;
}

/*
 * find_bad_triangles — Bowyer-Watson step 1: flag every valid triangle whose
 * circumcircle contains P. Together they form the "cavity" P will carve out.
 * is_bad[] is caller-zeroed scratch indexed by triangle slot. The
 * MAX_BAD_TRIS cap is defensive — at our point counts it is never reached.
 */
static void find_bad_triangles(const Delaunay *d, Point p, bool is_bad[])
{
    int n_bad = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!d->triangles[i].valid) continue;
        const Triangle *t = &d->triangles[i];
        if (in_circumcircle(p,
                            d->points[t->p[0]],
                            d->points[t->p[1]],
                            d->points[t->p[2]])) {
            is_bad[i] = true;
            if (++n_bad >= MAX_BAD_TRIS) break;   /* defensive */
        }
    }
}

/*
 * cavity_edge_add — register one undirected edge (a,b) into the dedup buffer:
 * normalise to the canonical key a ≤ b, then bump its occurrence count if the
 * edge is already present, else append it. Counting is how boundary (count 1)
 * and interior (count 2) edges are later told apart.
 */
static void cavity_edge_add(CavityEdge edges[], int *n_edges, int a, int b)
{
    if (a > b) { int tmp = a; a = b; b = tmp; }   /* canonical edge key */

    for (int j = 0; j < *n_edges; j++)
        if (edges[j].a == a && edges[j].b == b) { edges[j].count++; return; }

    if (*n_edges < MAX_EDGE_BUF) {
        edges[*n_edges].a     = a;
        edges[*n_edges].b     = b;
        edges[*n_edges].count = 1;
        (*n_edges)++;
    }
}

/*
 * collect_cavity_edges — Bowyer-Watson step 2: walk all three edges of every
 * bad triangle through cavity_edge_add, producing the deduped edge list.
 * Returns the edge count; each edge's .count classifies it (1 = boundary).
 */
static int collect_cavity_edges(const Delaunay *d, const bool is_bad[],
                                CavityEdge edges[])
{
    int n_edges = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!is_bad[i]) continue;
        for (int e = 0; e < 3; e++) {
            int a = d->triangles[i].p[e];
            int b = d->triangles[i].p[(e + 1) % 3];
            cavity_edge_add(edges, &n_edges, a, b);
        }
    }
    return n_edges;
}

/*
 * invalidate_triangles — Bowyer-Watson step 3: logically remove every bad
 * triangle by clearing its valid flag (the slot is kept so indices stay
 * stable across the insertion).
 */
static void invalidate_triangles(Delaunay *d, const bool is_bad[])
{
    for (int i = 0; i < d->n_tris; i++)
        if (is_bad[i]) d->triangles[i].valid = false;
}

/*
 * retriangulate_cavity — Bowyer-Watson step 4: fill the cavity by joining the
 * new point p_idx to every BOUNDARY edge (count == 1), appending one fresh
 * (glowing) triangle per boundary edge. Returns false if MAX_TRIS would be
 * exceeded (defensive — shouldn't happen with our caps).
 */
static bool retriangulate_cavity(Delaunay *d, const CavityEdge edges[],
                                 int n_edges, int p_idx)
{
    for (int j = 0; j < n_edges; j++) {
        if (edges[j].count != 1) continue;          /* interior edge — skip */
        if (d->n_tris >= MAX_TRIS) return false;
        d->triangles[d->n_tris++] = (Triangle){
            .p = { edges[j].a, edges[j].b, p_idx },
            .valid = true,
            .new_glow = 1.0f,          /* EFFECT: flash this fresh edge */
        };
    }
    return true;
}

/*
 * dt_insert_point — Bowyer-Watson insertion of one new point, read as its
 * four textbook steps. Returns false if storage would overflow (defensive).
 */
static bool dt_insert_point(Delaunay *d, int x, int y)
{
    if (d->n_points >= MAX_POINTS) return false;

    int p_idx = d->n_points;                     /* append P to points[] */
    d->points[p_idx] = (Point){ x, y };
    d->n_points++;
    Point P = d->points[p_idx];

    bool is_bad[MAX_TRIS] = { false };
    find_bad_triangles(d, P, is_bad);            /* 1. cavity = circumcircle hits */

    CavityEdge edges[MAX_EDGE_BUF];
    int n_edges = collect_cavity_edges(d, is_bad, edges);  /* 2. cavity edges */

    invalidate_triangles(d, is_bad);             /* 3. demolish bad triangles */

    if (!retriangulate_cavity(d, edges, n_edges, p_idx))   /* 4. refill cavity */
        return false;

    d->n_real++;
    return true;
}

/*
 * dt_step — performs one Bowyer-Watson insertion (after the cooldown
 * has elapsed). Returns true if a real point was inserted, false if
 * either still cooling down OR all real points already in.
 */
static bool dt_step(Delaunay *d)
{
    if (d->n_real >= d->target) return false;
    if (d->insert_cooldown > 0) {
        d->insert_cooldown--;
        return false;
    }
    int x, y;
    dt_pick_real_point(d, &x, &y);
    bool ok = dt_insert_point(d, x, y);
    d->insert_cooldown = INSERT_HOLD_TICKS_DEF;
    return ok;
}

/* ===================================================================== */
/* §6  scene — orchestration: the one place the layers combine            */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   BUILDING — insert one point per cooldown until the preset's target
 *              count is reached, then transition to DONE.
 *   DONE     — the triangulation is complete; HOLD on it forever. No
 *              auto-reset and no auto preset switch — the user starts a
 *              fresh build with r (same preset) or n/p (switch preset).
 */
typedef enum {
    SCENE_BUILDING = 0,
    SCENE_DONE     = 1,
} SceneState;

/*
 * Control — the user-tunable knobs, gathered in one place: every field is
 * something a key changes, nothing the algorithm decides on its own. Bounds
 * for each live in §1 config; app_handle_key clamps to them.
 */
typedef struct {
    int  preset_idx;          /* n/p  visual style, index into presets[]      */
    int  insert_hold_ticks;   /* +/-  build speed (ticks per point inserted)  */
    int  sim_fps;             /* [ ]  simulation tick rate (Hz)              */
    bool paused;              /* space freeze the build                       */
} Control;

/*
 * Scene — the whole showcase in one structure, three concerns kept apart:
 *   d      WHAT is simulated (the Delaunay mesh + its build progress)
 *   ctrl   HOW the user drives it (the knobs)
 *   state  WHERE in the lifecycle (BUILDING or DONE)
 */
typedef struct {
    Delaunay    d;
    Control     ctrl;
    SceneState  state;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    const Preset *p = &presets[s->ctrl.preset_idx];
    preset_apply(s->ctrl.preset_idx);             /* palette → colour pairs    */
    dt_reset(&s->d, mw, mh, p->n_points);         /* fresh mesh at preset size */
    s->d.insert_cooldown = s->ctrl.insert_hold_ticks;
    s->state = SCENE_BUILDING;
}

/* scene_init — full setup for startup. Sets the knob defaults once; resize
 * and rebuilds go through scene_reset, which preserves them. */
static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->ctrl.preset_idx        = 0;
    s->ctrl.insert_hold_ticks = INSERT_HOLD_TICKS_DEF;
    s->ctrl.sim_fps           = SIM_FPS_DEFAULT;
    s->ctrl.paused            = false;
    scene_reset(s, mw, mh);
}

/* scene_regrow — rebuild at the current size, keeping the chosen preset and
 * build speed. Triggered only by the user: r (same preset) or n/p. */
static void scene_regrow(Scene *s)
{
    scene_reset(s, s->d.w, s->d.h);
}

/*
 * scene_tick — THE one place the layers combine each tick, in fixed order:
 * DELAY guard → EFFECTS fade → SIMULATION step → DELAY transition. Nothing
 * else advances state. dt is the fixed sim timestep.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;            /* DELAY: frozen — nothing advances */

    fx_decay(&s->d, dt);                   /* EFFECTS: fade glows (even when DONE,
                                            * so the last flash settles)        */

    if (s->state == SCENE_DONE) return;    /* DELAY: built — hold, no inserts   */

    /* SIMULATION, paced by the insert cooldown (a DELAY): re-clamp the
     * cooldown so a +/- speed change takes effect on the next insertion. */
    if (s->d.insert_cooldown > s->ctrl.insert_hold_ticks)
        s->d.insert_cooldown = s->ctrl.insert_hold_ticks;
    dt_step(&s->d);

    if (s->d.n_real >= s->d.target)        /* DELAY: build complete → hold */
        s->state = SCENE_DONE;
}

/* ===================================================================== */
/* §7  render — state → ASCII (pure reads of the simulation)              */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, cached from getmaxyx(). WHY cache
 * it: the dimensions change only on a resize (SIGWINCH → screen_resize
 * re-reads them), yet every frame needs them to centre the mesh and pin the
 * HUD bars. Units are character cells — this is cell-space rendering.
 */
typedef struct {
    int cols;   /* terminal width  in character columns */
    int rows;   /* terminal height in character rows    */
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
 * edge_glyph — pick the ASCII character for an edge based on its
 * dominant slope. Uses the line's overall (Δx, Δy) — every cell on
 * the edge gets the same glyph for visual coherence.
 */
static char edge_glyph(int dx, int dy)
{
    int adx = dx < 0 ? -dx : dx;
    int ady = dy < 0 ? -dy : dy;
    if (adx > 2 * ady) return '-';
    if (ady > 2 * adx) return '|';
    /* (dx > 0 && dy > 0) or (dx < 0 && dy < 0) → top-left to bottom-right
     * which in screen coords looks like '\\'. */
    return ((long)dx * (long)dy > 0) ? '\\' : '/';
}

/*
 * draw_edge_segment — Bresenham line draw. Iterates cells from
 * (x0, y0) to (x1, y1) and calls mvaddch with the chosen glyph on
 * each visible cell. attron/attroff is the caller's responsibility
 * (so we only set colour once per edge, not once per cell).
 */
static void draw_edge_segment(int x0, int y0, int x1, int y1,
                              int gx0, int gy0, int cols, int rows,
                              char glyph)
{
    int dx =  abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (1) {
        int sx_screen = gx0 + x;
        int sy_screen = gy0 + y;
        if (sx_screen >= 0 && sx_screen < cols
            && sy_screen >= 0 && sy_screen < rows) {
            mvaddch(sy_screen, sx_screen, (chtype)(unsigned char)glyph);
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

/*
 * mesh_origin — top-left screen cell the map's (0,0) maps to, centring the
 * w×h mesh in the area between the row-0 data bar and the last-row hint.
 * Clamped so the mesh never overdraws the bars. Written through gx0/gy0.
 */
static void mesh_origin(const Delaunay *d, int cols, int rows,
                        int *gx0, int *gy0)
{
    *gx0 = (cols - d->w) / 2;
    *gy0 = ((rows - 2) - d->h) / 2 + 1;   /* row 0 = data bar, last = hint */
    if (*gx0 < 0) *gx0 = 0;
    if (*gy0 < 1) *gy0 = 1;
}

/*
 * draw_mesh_edges — pass 1: every real triangle's three sides, drawn cell by
 * cell with the slope-aware glyph. Scaffold triangles are filtered by
 * triangle_is_real; flashing ones use the accent colour + bold. Edges shared
 * by two triangles are drawn twice — harmless overdraw.
 */
static void draw_mesh_edges(const Scene *s, int gx0, int gy0,
                            int cols, int rows)
{
    const Delaunay *d = &s->d;
    for (int i = 0; i < d->n_tris; i++) {
        const Triangle *t = &d->triangles[i];
        if (!t->valid || !triangle_is_real(t)) continue;

        int pair = triangle_is_flashing(t) ? PAIR_FLASH : PAIR_EDGE;
        int attr = triangle_is_flashing(t) ? A_BOLD     : A_NORMAL;

        attron(COLOR_PAIR(pair) | attr);
        for (int e = 0; e < 3; e++) {
            const Point *a = &d->points[t->p[e]];
            const Point *b = &d->points[t->p[(e + 1) % 3]];
            char g = edge_glyph(b->x - a->x, b->y - a->y);
            draw_edge_segment(a->x, a->y, b->x, b->y,
                              gx0, gy0, cols, rows, g);
        }
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* draw_seed_points — pass 2: the real seed points' glyph on top of the edges,
 * in the preset's point colour. */
static void draw_seed_points(const Scene *s, int gx0, int gy0,
                             int cols, int rows)
{
    const Delaunay *d = &s->d;
    char point_glyph = presets[s->ctrl.preset_idx].point_glyph;

    attron(COLOR_PAIR(PAIR_POINT) | A_BOLD);
    for (int i = SUPER_TRI_VERTS; i < d->n_points; i++) {
        int sx = gx0 + d->points[i].x;
        int sy = gy0 + d->points[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)point_glyph);
    }
    attroff(COLOR_PAIR(PAIR_POINT) | A_BOLD);
}

/*
 * scene_draw — the mesh, as two named passes over a centred origin:
 * edges (background) → seed points (foreground).
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    mesh_origin(&s->d, cols, rows, &gx0, &gy0);

    draw_mesh_edges(s, gx0, gy0, cols, rows);
    draw_seed_points(s, gx0, gy0, cols, rows);
}

/*
 * hud_bar — paint one full-width status bar on `row`: fill the row with
 * spaces in `pair`, then write `buf` clipped to the terminal width with
 * mvaddnstr. Clipping (not mvprintw) is what keeps an over-long string from
 * wrapping down onto the mesh. Drives both the data bar and the action bar.
 */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* count_real_triangles — live (valid) triangles that belong to the mesh,
 * scaffold-touching ones filtered out. The HUD's tris:N figure. */
static int count_real_triangles(const Delaunay *d)
{
    int n = 0;
    for (int i = 0; i < d->n_tris; i++)
        if (d->triangles[i].valid && triangle_is_real(&d->triangles[i]))
            n++;
    return n;
}

static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Delaunay *d = &s->d;
    const Control  *c = &s->ctrl;
    const char *state_str =
        c->paused                    ? "PAUSED"   :
        (s->state == SCENE_BUILDING) ? "BUILDING" :
                                       "DONE";

    /* Visible (real) triangle count — scaffold triangles are filtered. */
    int n_visible = count_real_triangles(d);

    /* Row 0 — DATA: identity, active preset, live state, one clipped line. */
    char data[200];
    snprintf(data, sizeof data,
             " DELAUNAY B-W  %-8s  %s (%d/%d)  pts:%d/%d  tris:%d  "
             "hold:%-3d  %5.1f fps  %3d Hz ",
             state_str, presets[c->preset_idx].name,
             c->preset_idx + 1, N_PRESETS,
             d->n_real, d->target, n_visible,
             c->insert_hold_ticks, fps, c->sim_fps);

    /* Last row — ACTIONS only: every interactive key, nothing else. */
    static const char *keys =
        " q:quit  spc:pause  n/p:preset  r:reset  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app — PERFORMANCE loop (fixed timestep) + signals + input          */
/* ===================================================================== */

/*
 * App — the top-level program object: the showcase, the screen, the derived
 * map size, and the two signal flags. WHY a single g_app global: POSIX
 * signal handlers take no user pointer, so the handlers below reach the
 * program through this one well-known instance — the only global, by
 * design; everything else is passed by pointer.
 */
typedef struct {
    Scene                 scene;          /* the showcase (mesh + control)    */
    Screen                screen;         /* cached terminal size             */
    int                   map_w, map_h;   /* mesh size chosen to fit the
                                           * screen, clamped to MAP_W/H_MAX    */
    volatile sig_atomic_t running;        /* 0 ⇒ exit main loop (SIGINT/TERM).
                                           * volatile sig_atomic_t: the only
                                           * type safe to touch in a handler   */
    volatile sig_atomic_t need_resize;    /* 1 ⇒ re-read size (SIGWINCH)        */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 2;
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
    Scene   *s = &app->scene;
    Control *c = &s->ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     c->paused = !c->paused; break;

    case 'n': case 'N':   /* next preset */
        c->preset_idx = (c->preset_idx + 1) % N_PRESETS;
        scene_regrow(s);
        break;
    case 'p': case 'P':   /* previous preset */
        c->preset_idx = (c->preset_idx + N_PRESETS - 1) % N_PRESETS;
        scene_regrow(s);
        break;

    case 'r': case 'R':   /* rebuild, same preset */
        scene_regrow(s);
        break;
    case '=': case '+':   /* faster = SHORTER cooldown (halve, clamped) */
        c->insert_hold_ticks = clamp_int(c->insert_hold_ticks / 2,
                                         INSERT_HOLD_TICKS_MIN,
                                         INSERT_HOLD_TICKS_MAX);
        break;
    case '-':             /* slower = LONGER cooldown (double, clamped) */
        c->insert_hold_ticks = clamp_int(c->insert_hold_ticks * 2,
                                         INSERT_HOLD_TICKS_MIN,
                                         INSERT_HOLD_TICKS_MAX);
        break;
    case ']':
        c->sim_fps = clamp_int(c->sim_fps + SIM_FPS_STEP,
                               SIM_FPS_MIN, SIM_FPS_MAX);
        break;
    case '[':
        c->sim_fps = clamp_int(c->sim_fps - SIM_FPS_STEP,
                               SIM_FPS_MIN, SIM_FPS_MAX);
        break;

    default: break;
    }
    return true;
}

/* install_signals — clean exit on SIGINT/SIGTERM, resize flag on SIGWINCH,
 * and endwin() via atexit so even a crash restores the terminal. */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    install_signals();

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);   /* sets ctrl.sim_fps */

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

        int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);
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

        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
