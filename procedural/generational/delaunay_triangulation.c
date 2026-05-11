/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * delaunay_triangulation.c — Bowyer-Watson Delaunay, animated.
 *
 * DEMO: Twelve seed points drop onto the map one at a time. Each is
 *       INSERTED into a growing Delaunay triangulation by Bowyer-
 *       Watson's incremental method: the new point invalidates every
 *       triangle whose circumscribed circle contains it; the
 *       resulting "cavity" is re-triangulated by joining the new
 *       point to every boundary edge. New triangles flash their
 *       edges briefly before settling. After all points are
 *       inserted, the super-triangle scaffolding is hidden, leaving
 *       a clean Delaunay mesh — every triangle's circumcircle
 *       contains no other vertex. HOLD; supernova reset; loop.
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
 * Section map:
 *   §1 config   — map size, point count, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (edge/point/flash)
 *   §5 dt       — Point, Triangle, in-circumcircle, Bowyer-Watson
 *   §6 scene    — BUILDING / HOLD state machine
 *   §7 screen   — ASCII line rendering: '-' '|' '/' '\\' edges
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (preserves theme)
 *   t / T      next / previous theme
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
 * References     : • Bowyer, A. (1981) — "Computing Dirichlet
 *                    tessellations", The Computer Journal.
 *                  • Watson, D.F. (1981) — "Computing the
 *                    n-dimensional Delaunay tessellation".
 *                  • Wikipedia — "Bowyer–Watson algorithm":
 *                    https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
 *                  • Inigo Quilez — "Voronoi/Delaunay duality":
 *                    https://iquilezles.org/articles/voronoilines/
 *                  • Bresenham, J. (1965) — line-drawing algorithm
 *                    used for the edge rendering.
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
 *   1. POINTS '@' (theme point colour, bold) — the seed set, growing
 *      from 0 to 12 over the BUILDING phase.
 *   2. EDGES (theme edge colour, slope-aware glyphs '-', '|', '/',
 *      '\\') — each triangle's three sides drawn cell-by-cell.
 *   3. NEW EDGE FLASH (theme flash colour) — edges that were just
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
 *  3. Repeat until n_real_points = N_POINTS.
 *  4. HOLD on the triangulation; reset; loop.
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
 *  • Different themes change colours but the triangulation is
 *    identical for the same RNG seed (algorithm is theme-independent).
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

    /* Real point count + 3 super-triangle scaffold points. */
    N_REAL_POINTS     =  12,
    SUPER_TRI_VERTS   =   3,
    MAX_POINTS        = N_REAL_POINTS + SUPER_TRI_VERTS,

    /* Triangle storage. Bound: a Delaunay triangulation of n points
     * has ≤ 2n+1 triangles in general position; with super-triangle
     * we add a few more during construction. 256 is comfortable. */
    MAX_TRIS          = 256,

    /* Per-insertion scratch: bad triangle indices + edge dedup buffer. */
    MAX_BAD_TRIS      =  64,
    MAX_EDGE_BUF      = 192,

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

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BG           =   3,        /* unused — reserved for theme parity */
    PAIR_EDGE         =   4,        /* triangle edges                  */
    PAIR_POINT        =   5,        /* '@' point glyph                 */
    PAIR_FLASH        =   6,        /* new edge flash                  */
    PAIR_SUPERNOVA    =   7,        /* yellow reset flash              */
};

/* Glow decay rates. */
#define EDGE_GLOW_DECAY     2.5f    /* new-edge flash fade ~0.7 s */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the rest of the procedural showcases.
 * Each theme defines four colours: bg/edge/point/flash. PAIR_BG is
 * reserved (no background pattern is drawn).
 */
typedef struct {
    const char *name;
    short       bg, edge, point, flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      bg edge point flash */
    { "DEFAULT",  240,   51,  231,  220 },   /* grey / cyan / white / gold */
    { "MATRIX",    22,   46,  118,  226 },   /* greens                     */
    { "NOVA",      53,  201,  219,  226 },   /* magenta / pink / yellow    */
    { "MONO",     234,  244,  254,  226 },   /* greyscale + yellow accent  */
    { "OCEAN",     17,   39,  159,  226 },   /* navy / cyan / ice          */
    { "FIRE",      52,  208,  226,  196 },   /* orange / yellow / red      */
    { "EARTH",     58,  173,  230,  220 },   /* brown / cream / gold       */
    { "FOREST",    22,   82,  154,  226 },   /* greens                     */
    { "DESERT",    94,  178,  230,  220 },   /* sandy                      */
    { "ARCTIC",    18,  159,  231,  226 },   /* navy / ice / white         */
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
        init_pair(PAIR_BG,    t->bg,    -1);
        init_pair(PAIR_EDGE,  t->edge,  -1);
        init_pair(PAIR_POINT, t->point, -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        init_pair(PAIR_BG,    COLOR_WHITE,   -1);
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
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  dt — Delaunay triangulation                                        */
/* ===================================================================== */

/*
 * Point — integer cell position. Indices 0..2 are the super-triangle
 * scaffold; indices 3..MAX_POINTS-1 are real input points.
 */
typedef struct {
    int x, y;
} Point;

/*
 * Triangle — three vertex indices into points[], plus state.
 *
 *   p[0..2]   : vertex indices
 *   valid     : false if logically removed (kept in array for index
 *               stability)
 *   new_glow  : 1.0 when freshly created, decays over ~0.7 s
 */
typedef struct {
    int   p[3];
    bool  valid;
    float new_glow;
} Triangle;

/*
 * Delaunay — the simulation heart.
 *
 *   w, h           : map dims
 *   points[]       : 3 super-tri verts followed by real points
 *   n_points       : current point count (3 + n_real)
 *   n_real         : real points already inserted (0..N_REAL_POINTS)
 *
 *   triangles[]    : all triangles ever created; valid-flag tells
 *                    which are alive
 *   n_tris         : total slots used
 *
 *   insert_cooldown: ticks until next insertion fires
 *
 *   supernova_glow_t : single global supernova fade
 */
typedef struct {
    int       w, h;
    Point     points[MAX_POINTS];
    int       n_points;
    int       n_real;

    Triangle  triangles[MAX_TRIS];
    int       n_tris;

    int       insert_cooldown;

    float     supernova_glow_t;
} Delaunay;

/* ── ─────────────────────────────────────────────────────────────────── *
 * Geometry helpers — operate on integer Point coordinates. Use 64-bit
 * intermediates throughout so int*int*int doesn't overflow on a
 * 200×200 grid.
 * ── ─────────────────────────────────────────────────────────────────── */

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

/* ── ─────────────────────────────────────────────────────────────────── *
 * Delaunay state management.
 * ── ─────────────────────────────────────────────────────────────────── */

/*
 * dt_reset — clear everything, place super-triangle, no real points.
 *
 * Super-triangle vertices sit at (−5W, −5H), (5W, −5H), (W/2, 5H) —
 * far enough that any in-bounds point is well inside, small enough
 * that the determinant doesn't overflow int64.
 */
static void dt_reset(Delaunay *d, int w, int h)
{
    d->w = w;
    d->h = h;
    d->n_points = 0;
    d->n_real   = 0;
    d->n_tris   = 0;
    d->insert_cooldown = 0;
    d->supernova_glow_t = 1.0f;

    /* Super-triangle vertices (indices 0..2). */
    d->points[0] = (Point){ -5 * w,        -5 * h };
    d->points[1] = (Point){  5 * w,        -5 * h };
    d->points[2] = (Point){      w / 2,     5 * h };
    d->n_points = 3;

    /* The single seed triangle. Order picked so it's CCW under the
     * y-up math convention (which is CW in screen coords — we don't
     * care about the on-screen orientation since super-tri verts are
     * off-screen). */
    d->triangles[0] = (Triangle){
        .p = {0, 1, 2},
        .valid = true,
        .new_glow = 0.0f,
    };
    d->n_tris = 1;
}

/*
 * dt_pick_real_point — pick a random in-bounds (x, y) with min-distance
 * rejection against existing real points.
 *
 * Falls back to the last candidate even if it's too close — for our
 * 200×56 map and N_REAL=12 we always find a good spot, but the
 * fallback keeps the algorithm running on extreme inputs.
 */
static void dt_pick_real_point(const Delaunay *d, int *out_x, int *out_y)
{
    int best_x = 0, best_y = 0;
    int margin = 2;     /* keep points off the very edge */

    for (int attempt = 0; attempt < POINT_PLACE_TRIES; attempt++) {
        int x = margin + (rand() % (d->w - 2 * margin));
        int y = margin + (rand() % (d->h - 2 * margin));
        bool ok = true;
        for (int i = 3; i < d->n_points; i++) {
            int dx = x - d->points[i].x;
            int dy = y - d->points[i].y;
            if (dx * dx + dy * dy < MIN_POINT_DIST * MIN_POINT_DIST) {
                ok = false;
                break;
            }
        }
        best_x = x; best_y = y;
        if (ok) break;
    }
    *out_x = best_x;
    *out_y = best_y;
}

/*
 * dt_insert_point — Bowyer-Watson insertion of one new point.
 *
 *   1. Append to points[].
 *   2. Find bad triangles (P in their circumcircle) → mark + collect.
 *   3. Walk every edge of every bad triangle; classify as boundary
 *      (count == 1) or interior (count > 1) using min/max-normalised
 *      edge keys.
 *   4. Invalidate bad triangles.
 *   5. For each boundary edge, append a new triangle (e0, e1, P).
 *
 * Returns true if the point was inserted, false if MAX_TRIS would be
 * exceeded (defensive — shouldn't happen with our caps).
 */
static bool dt_insert_point(Delaunay *d, int x, int y)
{
    if (d->n_points >= MAX_POINTS) return false;
    int p_idx = d->n_points;
    d->points[p_idx] = (Point){ x, y };
    d->n_points++;

    Point P = d->points[p_idx];

    /* Find bad triangles. */
    bool is_bad[MAX_TRIS] = { false };
    int n_bad = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!d->triangles[i].valid) continue;
        Triangle *t = &d->triangles[i];
        if (in_circumcircle(P,
                            d->points[t->p[0]],
                            d->points[t->p[1]],
                            d->points[t->p[2]])) {
            is_bad[i] = true;
            n_bad++;
            if (n_bad >= MAX_BAD_TRIS) break;   /* defensive */
        }
    }

    /* Collect edges with occurrence counts. We use a flat array of
     * (p_low, p_high, count) — linear scan is fine, n_bad·3 entries
     * is small (typically < 50). */
    typedef struct { int a, b, count; } Edge;
    Edge edges[MAX_EDGE_BUF];
    int n_edges = 0;

    for (int i = 0; i < d->n_tris; i++) {
        if (!is_bad[i]) continue;
        for (int e = 0; e < 3; e++) {
            int a = d->triangles[i].p[e];
            int b = d->triangles[i].p[(e + 1) % 3];
            if (a > b) { int tmp = a; a = b; b = tmp; }

            int found = -1;
            for (int j = 0; j < n_edges; j++) {
                if (edges[j].a == a && edges[j].b == b) { found = j; break; }
            }
            if (found >= 0) {
                edges[found].count++;
            } else if (n_edges < MAX_EDGE_BUF) {
                edges[n_edges].a = a;
                edges[n_edges].b = b;
                edges[n_edges].count = 1;
                n_edges++;
            }
        }
    }

    /* Invalidate bad triangles. */
    for (int i = 0; i < d->n_tris; i++) {
        if (is_bad[i]) d->triangles[i].valid = false;
    }

    /* Add new triangles for every boundary edge (count == 1). */
    for (int j = 0; j < n_edges; j++) {
        if (edges[j].count != 1) continue;
        if (d->n_tris >= MAX_TRIS) return false;
        d->triangles[d->n_tris++] = (Triangle){
            .p = { edges[j].a, edges[j].b, p_idx },
            .valid = true,
            .new_glow = 1.0f,
        };
    }
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
    if (d->n_real >= N_REAL_POINTS) return false;
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
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   BUILDING — drop+insert one point per cooldown. When N_REAL points
 *              are inserted, transition to HOLD.
 *   HOLD     — wait HOLD_SECONDS, then dt_reset and back to BUILDING.
 */
typedef enum {
    SCENE_BUILDING = 0,
    SCENE_HOLD     = 1,
} SceneState;

typedef struct {
    Delaunay    d;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         insert_hold_ticks;     /* exposed to keyboard via +/- */
    int         current_theme;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    dt_reset(&s->d, mw, mh);
    s->d.insert_cooldown = s->insert_hold_ticks;
    s->state      = SCENE_BUILDING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused            = false;
    s->insert_hold_ticks = INSERT_HOLD_TICKS_DEF;
    s->current_theme     = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay glows on triangles + global supernova. */
    float edge_d = expf(-EDGE_GLOW_DECAY * dt);
    float nova_d = expf(-SUPERNOVA_DECAY * dt);
    for (int i = 0; i < s->d.n_tris; i++) {
        s->d.triangles[i].new_glow *= edge_d;
    }
    s->d.supernova_glow_t *= nova_d;

    switch (s->state) {

    case SCENE_BUILDING:
        /* Override the cooldown each tick so the user's adjustment
         * via +/- takes effect for the NEXT insertion. */
        if (s->d.insert_cooldown > s->insert_hold_ticks)
            s->d.insert_cooldown = s->insert_hold_ticks;
        dt_step(&s->d);
        if (s->d.n_real >= N_REAL_POINTS) {
            s->state = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->d.w, s->d.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Delaunay *d = &s->d;

    int gx0 = (cols - d->w) / 2;
    int gy0 = ((rows - 3) - d->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* Sparse supernova flash — only paint when active so we don't
     * iterate the full screen most frames. */
    if (d->supernova_glow_t > GLOW_THRESHOLD) {
        attron(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
        for (int y = 0; y < d->h; y++) {
            int sy = gy0 + y;
            if (sy < 0 || sy >= rows) continue;
            for (int x = 0; x < d->w; x++) {
                int sx = gx0 + x;
                if (sx < 0 || sx >= cols) continue;
                if (((x ^ y) & 3) == 0) mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
    }

    /* Pass 1 — draw EDGES. We iterate triangles, drawing each of the
     * 3 edges. Edges shared by two triangles get drawn twice; that's
     * fine (overdraw is cheap). Triangles touching super-tri vertices
     * (indices < 3) are skipped — that's the on-the-fly filtering
     * that hides the scaffold. */
    for (int i = 0; i < d->n_tris; i++) {
        const Triangle *t = &d->triangles[i];
        if (!t->valid) continue;
        if (t->p[0] < 3 || t->p[1] < 3 || t->p[2] < 3) continue;

        int pair = (t->new_glow > GLOW_THRESHOLD) ? PAIR_FLASH : PAIR_EDGE;
        int attr = (t->new_glow > GLOW_THRESHOLD) ? A_BOLD : A_NORMAL;

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

    /* Pass 2 — points '@' on top, in theme point colour. */
    attron(COLOR_PAIR(PAIR_POINT) | A_BOLD);
    for (int i = 3; i < d->n_points; i++) {
        int sx = gx0 + d->points[i].x;
        int sy = gy0 + d->points[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
    }
    attroff(COLOR_PAIR(PAIR_POINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Delaunay *d = &s->d;
    const char *state_str =
        s->paused                       ? "PAUSED   " :
        (s->state == SCENE_BUILDING)    ? "BUILDING " :
                                          "HOLD     ";

    /* Visible triangle count — for HUD, only count real ones. */
    int n_visible = 0;
    int n_valid   = 0;
    for (int i = 0; i < d->n_tris; i++) {
        if (!d->triangles[i].valid) continue;
        n_valid++;
        if (d->triangles[i].p[0] >= 3
            && d->triangles[i].p[1] >= 3
            && d->triangles[i].p[2] >= 3) n_visible++;
    }

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  pts:%d/%d  tris:%d ",
             fps, sim_fps, state_str,
             d->n_real, N_REAL_POINTS, n_visible);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " DELAUNAY TRIANGULATION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme name + algorithm parameters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             " bowyer-watson  hold-ticks:%-3d  valid-tris:%d  map:%dx%d ",
             s->insert_hold_ticks, n_valid, d->w, d->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " @:point  -|/\\:edge  *:flash | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
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
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        /* Faster = SHORTER cooldown. */
        if (s->insert_hold_ticks > INSERT_HOLD_TICKS_MIN)
            s->insert_hold_ticks /= 2;
        if (s->insert_hold_ticks < INSERT_HOLD_TICKS_MIN)
            s->insert_hold_ticks = INSERT_HOLD_TICKS_MIN;
        break;
    case '-':
        if (s->insert_hold_ticks < INSERT_HOLD_TICKS_MAX)
            s->insert_hold_ticks *= 2;
        if (s->insert_hold_ticks > INSERT_HOLD_TICKS_MAX)
            s->insert_hold_ticks = INSERT_HOLD_TICKS_MAX;
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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
