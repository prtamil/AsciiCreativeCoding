/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * geometry/delaunay_triangulation.c -- Delaunay Triangulation Visualizer
 *
 * Incremental Bowyer-Watson algorithm: inserts N_POINTS random points one
 * by one, maintaining the Delaunay property after each insertion.
 *
 * After insertion, a SHOWCASE phase cycles through every triangle, draws
 * its circumcircle, and verifies no other point lies inside it (the empty
 * circumcircle property).
 *
 * Study alongside: procedural/generational/delaunay_triangulation.c --
 *       same Bowyer-Watson algorithm, framed as a generative-art demo
 *       (12 seeds drop in, edges flash on insert, super-triangle hidden
 *       at the end).  This file is the algorithmic showcase (random
 *       points + circumcircle verification phase); that one is the
 *       generative narrative.
 *
 * -------------------------------------------------------------------------
 *  Section map
 * -------------------------------------------------------------------------
 *  S1  config       -- sizes, timing, aspect ratio
 *  S2  clock        -- monotonic ns clock + sleep
 *  S3  color        -- palette
 *  S4  geometry     -- Pt/Tri/HEdge types, circumcircle math, line/ellipse draw
 *  S5  mesh         -- Mesh + Scratch structs, Bowyer-Watson in four named steps
 *  S6  scene        -- Insertion/Showcase/Scene structs, phase machine
 *  S7  render       -- per-layer renderers + render_scene driver + HUD
 *  S8  screen       -- ncurses layer
 *  S9  app          -- signals, resize, input, main loop
 * -------------------------------------------------------------------------
 *
 * Keys:  SPACE pause/resume   s single-step   r reset (new points)
 *        +/-   speed          q quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       geometry/delaunay_triangulation.c \
 *       -o dt -lncurses -lm
 */

/* -- CONCEPTS -----------------------------------------------------------  *
 *
 * Empty circumcircle property (S4 in_circumcircle):
 *   A triangulation is Delaunay iff for every triangle no other point in
 *   the set lies strictly inside its circumscribed circle.
 *   Equivalently, among all triangulations of the same point set, Delaunay
 *   maximises the minimum interior angle -- it avoids thin "sliver" triangles.
 *
 * Bowyer-Watson incremental insertion (S5 mesh_insert_point):
 *   To insert point P into an existing Delaunay triangulation:
 *     1. Find all "bad" triangles whose circumcircle contains P.    bw_find_bad
 *     2. Trace the boundary of the bad-triangle union -- the "hole".bw_collect_hole
 *     3. Remove bad triangles.                                      bw_remove_bad
 *     4. Connect P to every boundary edge to fill the hole.         bw_fill_hole
 *   The result is again a valid Delaunay triangulation.  Each insertion
 *   touches only O(log N) triangles on average, giving O(N log N) total.
 *
 * Mesh quality benefit (S7 render_overlay):
 *   Finite element methods and scattered-data interpolation require meshes
 *   with well-shaped elements.  Delaunay guarantees no angle < the smallest
 *   angle of the point set's convex hull -- far better than naive meshing.
 *   This is why Delaunay is the default mesh generator in physics simulators,
 *   terrain engines, and medical imaging.
 *
 * Aspect ratio correction (S4):
 *   Terminal cells are ~2x taller than wide (CELL_H/CELL_W = 2).  All
 *   circumcircle geometry is computed in "geo space" where y is doubled,
 *   then converted back for display as an aspect-corrected ellipse.
 *
 * ----------------------------------------------------------------------- */

/* -- REFERENCES ---------------------------------------------------------  *
 *
 * Algorithm & theory:
 *   [1] Bowyer, A. (1981). "Computing Dirichlet tessellations."
 *       The Computer Journal 24(2), 162-166.
 *       -- one of the two namesake papers; introduces the incremental
 *          insertion idea via the dual Voronoi diagram.
 *
 *   [2] Watson, D. F. (1981). "Computing the n-dimensional Delaunay
 *       tessellation with application to Voronoi polytopes."
 *       The Computer Journal 24(2), 167-172.
 *       -- the companion paper (same issue) that gives the algorithm
 *          we implement: find bad triangles, remove, retriangulate the hole.
 *
 *   [3] Sloan, S. W. (1987). "A fast algorithm for constructing Delaunay
 *       triangulations in the plane." Advances in Engineering Software,
 *       9(1), 34-55.
 *       -- the practical engineering reference for the "huge super-
 *          triangle" trick used by S5 mesh_init_super().
 *
 *   [4] de Berg, Cheong, van Kreveld, Overmars (2008).
 *       "Computational Geometry: Algorithms and Applications" (3rd ed.),
 *       chapter 9. Springer.
 *       -- the canonical textbook treatment; best starting point if you
 *          want one source.  Proves the empty-circumcircle property and
 *          the max-min-angle theorem.
 *
 * Implementation & numerics:
 *   [5] Shewchuk, J. R. (1996). "Triangle: Engineering a 2D Quality Mesh
 *       Generator and Delaunay Triangulator." Applied Computational
 *       Geometry: Towards Geometric Engineering, LNCS 1148, 203-222.
 *       -- the most-cited practical reference; covers degeneracies,
 *          ghost triangles, and how to make Bowyer-Watson robust at scale.
 *
 *   [6] Shewchuk, J. R. (1997). "Adaptive Precision Floating-Point
 *       Arithmetic and Fast Robust Geometric Predicates." Discrete &
 *       Computational Geometry 18(3), 305-363.
 *       -- the gold standard for an exact in-circumcircle test.  Our
 *          float version (S4 in_circumcircle) is fine for 40 random
 *          points; production code uses these adaptive predicates.
 *
 * Rendering:
 *   [7] Bresenham, J. E. (1965). "Algorithm for computer control of a
 *       digital plotter." IBM Systems Journal 4(1), 25-30.
 *       -- the integer DDA line algorithm in S4 draw_line().
 *
 *   [8] Foley, van Dam, Feiner, Hughes (1995). "Computer Graphics:
 *       Principles and Practice" (2nd ed. in C), chapters 3 & 19.
 *       Addison-Wesley.
 *       -- midpoint ellipse rasterization, scan conversion theory, and
 *          background for the aspect-correction trick used in S7
 *          render_circumcircle().
 *
 * ----------------------------------------------------------------------- */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* S1  config                                                             */
/* ===================================================================== */

#define SIM_FPS_DEFAULT   8
#define SIM_FPS_MIN       1
#define SIM_FPS_MAX      30
#define TARGET_FPS       60
#define FPS_UPDATE_MS    500

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ASPECT_Y: terminal cell height / width.
 * CELL_H=16, CELL_W=8 => aspect = 2.0.
 * Multiply cell-space y by ASPECT_Y before any distance/circumcircle
 * computation so that "circles" are metrically correct.
 */
#define ASPECT_Y  2.0f

#define N_POINTS         40     /* random points to triangulate           */
#define SUPER_COUNT       3     /* super-triangle vertex indices 0,1,2    */
#define MAX_PTS          (N_POINTS + SUPER_COUNT + 2)
#define MAX_TRIS         (N_POINTS * 8 + 10)
#define MAX_HOLE         (MAX_TRIS * 3)

#define HUD_TOP_ROWS      4     /* rows reserved at top for data overlay  */
#define HUD_BOTTOM_ROWS   1     /* rows reserved at bottom for actions    */
#define MARGIN_X          4
#define MARGIN_Y          3

#define SHOWCASE_HOLD     1     /* sim-ticks per triangle in showcase     */
#define SHOWCASE_CYCLES   1
#define DONE_TICKS       10

/* Frame pacing (S9 main loop). */
#define DT_CAP_NS         (100 * NS_PER_MS)     /* spiral-of-death guard:
                                                   if dt exceeds this (e.g. after
                                                   debugger pause), absorb only
                                                   this much so the sim doesn't
                                                   try to "catch up" with a flood
                                                   of ticks. */
#define FRAME_PERIOD_NS   (NS_PER_SEC / TARGET_FPS)  /* nominal render budget */

/* Numerical robustness epsilons.  Floats are good to ~6 decimal digits;
 * these tolerances live above that noise floor.  See Shewchuk 1997 [6]
 * for why exact predicates would replace these in production code. */
#define DEGEN_TRI_EPS     1e-7f   /* twice-signed-area magnitude below which
                                     three points are treated as colinear --
                                     no circumcircle defined */
#define INCIRCLE_EPS      1e-5f   /* slack on the "strictly inside" test
                                     used by bw_find_bad */
#define VIOLATION_EPS     1e-4f   /* same test for HUD verdict; deliberately
                                     looser so a near-boundary point isn't
                                     flagged as a Delaunay violator */

/* Math literals named for their geometric meaning. */
#define TAU               (2.0f * (float)M_PI)  /* one full turn in radians */

/* Ellipse rasterization (draw_ellipse).  Sample count tracks perimeter
 * length: a coarse approximation of the Ramanujan perimeter formula. */
#define ELLIPSE_PERIM_FACTOR  0.75f             /* empirical scaling          */
#define ELLIPSE_MIN_STEPS     32                /* minimum samples per ring   */
#define ELLIPSE_MAX_STEPS    512                /* avoid pathological huge n  */

/* Seed bumps for restart -- coprime offsets so successive runs traverse
 * different RNG sub-sequences instead of nearby states. */
#define RESET_SEED_BUMP    9973                 /* user 'r' key            */
#define RESTART_SEED_BUMP  7919                 /* auto-restart from DONE  */

/* HUD layout. */
#define HUD_VERDICT_RIGHT_OFFSET  40            /* cols from right edge for
                                                   the "empty circumcircle: YES/NO"
                                                   verdict text */
#define HUD_SEPARATOR_ROW          3            /* the dashed divider line */

/* ===================================================================== */
/* S2  clock                                                              */
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
    struct timespec r = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* S3  color                                                              */
/* ===================================================================== */

enum {
    CP_DEFAULT = 0,
    CP_EDGE,         /* normal triangle edges (dim blue)       */
    CP_EDGE_HI,      /* highlighted triangle (bright yellow)   */
    CP_CIRC,         /* circumcircle ring (magenta)            */
    CP_CIRC_CTR,     /* circumcenter dot (bright magenta)      */
    CP_POINT,        /* regular points (white)                 */
    CP_POINT_NEW,    /* most-recently inserted point (yellow)  */
    CP_POINT_IN,     /* point inside circumcircle (red)        */
    CP_HUD,
    CP_HEADER,
    CP_LABEL,
    CP_EXPLAIN,
    CP_OK,           /* "empty: YES" green                     */
    CP_WARN,         /* "empty: NO"  red                       */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_EDGE,      33,  -1);
        init_pair(CP_EDGE_HI,  226,  -1);
        init_pair(CP_CIRC,     201,  -1);
        init_pair(CP_CIRC_CTR, 207,  -1);
        init_pair(CP_POINT,    255,  -1);
        init_pair(CP_POINT_NEW,226,  -1);
        init_pair(CP_POINT_IN, 196,  -1);
        init_pair(CP_HUD,      252,  -1);
        init_pair(CP_HEADER,    51,  -1);
        init_pair(CP_LABEL,    244,  -1);
        init_pair(CP_EXPLAIN,  227,  -1);
        init_pair(CP_OK,        46,  -1);
        init_pair(CP_WARN,     196,  -1);
    } else {
        init_pair(CP_EDGE,      COLOR_CYAN,    -1);
        init_pair(CP_EDGE_HI,   COLOR_YELLOW,  -1);
        init_pair(CP_CIRC,      COLOR_MAGENTA, -1);
        init_pair(CP_CIRC_CTR,  COLOR_MAGENTA, -1);
        init_pair(CP_POINT,     COLOR_WHITE,   -1);
        init_pair(CP_POINT_NEW, COLOR_YELLOW,  -1);
        init_pair(CP_POINT_IN,  COLOR_RED,     -1);
        init_pair(CP_HUD,       COLOR_WHITE,   -1);
        init_pair(CP_HEADER,    COLOR_CYAN,    -1);
        init_pair(CP_LABEL,     COLOR_WHITE,   -1);
        init_pair(CP_EXPLAIN,   COLOR_YELLOW,  -1);
        init_pair(CP_OK,        COLOR_GREEN,   -1);
        init_pair(CP_WARN,      COLOR_RED,     -1);
    }
}

/* ===================================================================== */
/* S4  geometry primitives                                                */
/* ===================================================================== */

/*
 * Pt -- a 2D point.
 *
 * WHY a separate type:  Every coordinate in the program travels through this
 *      type, so the meaning of (x,y) is documented in exactly one place.
 *
 * Units / space:
 *      x  is in CELL columns; one unit = one character cell.
 *      y  is in CELL rows;    one unit = one character row (which is
 *         visually ~2x taller than one column -- see ASPECT_Y in S1).
 *
 * For Delaunay math we must work in metric "geometry space" where x and y
 * have the same physical scale.  Conversion is one-way and happens only at
 * the math boundary (gy() in S4, called by bw_find_bad, tri_inside_count,
 * render_circumcircle).  Pt itself always holds the cell-space form.
 *
 * float (not double):  precision is ample for ~40 points on an 80x24
 *      terminal; doubles would just double the cache footprint of Mesh.pts.
 */
typedef struct { float x, y; } Pt;

/*
 * Tri -- a triangle, stored as three indices into Mesh.pts (NOT three Pts).
 *
 * WHY indices, not coordinates:
 *      1. A vertex is typically shared by ~6 triangles; storing indices
 *         lets all of them refer to the same Pt and stay in sync if it
 *         ever moves.  (Not needed for static input, but a one-line
 *         habit that scales to dynamic meshes.)
 *      2. The Bowyer-Watson edge-matching predicate (bw_edge_is_shared
 *         in S5) decides whether two triangles share an edge by integer
 *         comparison -- same vertex indices, either direction.  Comparing
 *         floats would need an epsilon and could miss matches due to
 *         round-off.
 *      3. Indices 0..SUPER_COUNT-1 are reserved for the super-triangle,
 *         which lets every "is this a real triangle?" test become a
 *         single ">= SUPER_COUNT" comparison.
 *
 * Vertex order (a, b, c) is NOT canonicalised; Bowyer-Watson does not
 * rely on CCW/CW orientation -- only on edge adjacency.  Renderers treat
 * the three edges symmetrically.
 *
 * Reference:  Shewchuk 1996 (Triangle) [5] §3 discusses the trade-offs of
 *      index-based vs. pointer-based vs. quad-edge mesh representations.
 *      Guibas & Stolfi 1985 is the alternative we are NOT using.
 */
typedef struct { int a, b, c; } Tri;

/*
 * HEdge -- a half-edge / directed boundary edge, holding two Mesh.pts indices.
 *
 * WHY this exists at all:
 *      Bowyer-Watson step 2 (bw_collect_hole in S5) walks every edge of
 *      every bad triangle and records the ones that do NOT have a matching
 *      mate among the other bad triangles.  Those unmatched edges, taken
 *      together, form the star-shaped boundary of the "hole" left after
 *      bad triangles are removed.  Step 4 (bw_fill_hole) then fans the
 *      new point P into a triangle against each of those edges.
 *
 * WHY directed (a -> b, not unordered):
 *      We never need to ask "is edge {a,b} present?" in a way that cares
 *      about direction -- but recording the source triangle's CCW/CW
 *      traversal order means the fill step can write Tri{ a, b, P } and
 *      get a consistent winding without an extra orientation check.
 *
 * NOT a general mesh half-edge:  this is the minimum data needed for one
 *      Bowyer-Watson hole.  A full half-edge data structure (Mantyla 1988;
 *      OpenMesh) carries next/prev/twin pointers; we do not.
 */
typedef struct { int a, b; } HEdge;

/* ---- coordinate-space conversion ------------------------------------- */

/* Convert cell-space y to GEOMETRY space (aspect-corrected Euclidean).
 * Every circumcircle calculation must be done in geometry space; cells
 * are ~2x taller than wide, so a Euclidean circle would render as a
 * stretched egg if we ignored ASPECT_Y. */
static inline float gy(float cy) { return cy * ASPECT_Y; }

/* ---- 2-D math primitives --------------------------------------------- */

/* |(x,y)|^2 -- squared magnitude of a 2-vector.  Squared because every
 * geometric predicate in this file ("inside circle?", "below epsilon?")
 * can be answered without taking sqrt; sqrt is reserved for the one
 * radius-display path in S7. */
static inline float vec2_sqr_mag(float x, float y) { return x*x + y*y; }

/* 2 * SignedArea(A, B, C) -- the cross-product shoelace expression.
 *   sign:      > 0 if A,B,C are counter-clockwise, < 0 if clockwise.
 *   magnitude: twice the triangle's area; near zero iff colinear.
 * The doubled form avoids a divide-by-2 in callers that only inspect
 * sign/magnitude relative to an epsilon. */
static inline float tri_signed_area_doubled(float ax, float ay,
                                            float bx, float by,
                                            float cx, float cy)
{
    return ax*(by-cy) + bx*(cy-ay) + cx*(ay-by);
}

/* ---- ncurses cell plotting ------------------------------------------- */

/* plot_cell -- write a glyph at (x,y) in color pair cp, clipped to the
 * screen.  Every line/ellipse/dot in this file paints through this one
 * helper so the bounds check lives in one place. */
static inline void plot_cell(int y, int x, int cp, chtype ch,
                             int rows, int cols)
{
    if (x < 0 || x >= cols || y < 0 || y >= rows) return;
    attron(COLOR_PAIR(cp));
    mvaddch(y, x, ch);
    attroff(COLOR_PAIR(cp));
}

/* ---- circumcircle of a triangle -------------------------------------- */

/*
 * circumcircle_geo() -- circumcircle of triangle in GEOMETRY space.
 *
 * Inputs are already aspect-corrected (y = cell_y * ASPECT_Y).  Outputs
 * center (ocx, ocy) and squared radius r2 in the same space.  Returns
 * false on near-colinear input (no unique circle exists).
 *
 * Derivation: the circumcenter is equidistant from A, B, C, so it lies
 * at the intersection of any two perpendicular bisectors.  Solving the
 * 2x2 linear system gives Cramer-rule formulas in terms of |A|^2, |B|^2,
 * |C|^2 and twice the signed area D:
 *
 *   D  = 2 * SignedArea(A,B,C)
 *   Ox = [|A|^2(By-Cy) + |B|^2(Cy-Ay) + |C|^2(Ay-By)] / D
 *   Oy = [|A|^2(Cx-Bx) + |B|^2(Ax-Cx) + |C|^2(Bx-Ax)] / D
 *
 * Reference: de Berg ch. 9 [4]; Shewchuk 1997 [6] for the robust version.
 */
static bool circumcircle_geo(float ax, float ay, float bx, float by,
                              float cx, float cy,
                              float *ocx, float *ocy, float *r2)
{
    /* Step 1 -- twice the signed area; near-zero means colinear vertices. */
    float two_area = 2.0f * tri_signed_area_doubled(ax,ay, bx,by, cx,cy);
    if (fabsf(two_area) < DEGEN_TRI_EPS) return false;

    /* Step 2 -- |A|^2, |B|^2, |C|^2 (Cramer numerators reuse these). */
    float ma2 = vec2_sqr_mag(ax, ay);
    float mb2 = vec2_sqr_mag(bx, by);
    float mc2 = vec2_sqr_mag(cx, cy);

    /* Step 3 -- center via Cramer's rule (perpendicular-bisector solve). */
    *ocx = (ma2*(by-cy) + mb2*(cy-ay) + mc2*(ay-by)) / two_area;
    *ocy = (ma2*(cx-bx) + mb2*(ax-cx) + mc2*(bx-ax)) / two_area;

    /* Step 4 -- squared radius from center to any vertex (use A). */
    *r2  = vec2_sqr_mag(*ocx - ax, *ocy - ay);
    return true;
}

/*
 * in_circumcircle() -- true iff point P lies STRICTLY inside triangle
 * (A,B,C)'s circumcircle.  Uses INCIRCLE_EPS to keep boundary cases from
 * oscillating during incremental insertion (Bowyer-Watson is sensitive
 * to ties).
 */
static bool in_circumcircle(float ax, float ay, float bx, float by,
                             float cx, float cy, float px, float py)
{
    float ocx, ocy, r2;
    if (!circumcircle_geo(ax, ay, bx, by, cx, cy, &ocx, &ocy, &r2))
        return false;
    return vec2_sqr_mag(px - ocx, py - ocy) < r2 - INCIRCLE_EPS;
}

/* ---- line and ellipse rasterization ---------------------------------- */

/*
 * draw_line() -- Bresenham's integer-only line algorithm in cell space.
 *
 * Reads as pseudocode:
 *   start at (x0,y0); each iteration plot the current cell, then advance
 *   along whichever axis is more "behind" (tracked by err) until we land
 *   on (x1,y1).
 *
 * Reference: Bresenham 1965 [7].
 */
static void draw_line(int x0, int y0, int x1, int y1,
                      int cp, chtype ch, int rows, int cols)
{
    int dx = abs(x1 - x0);              /* magnitude of x-extent       */
    int dy = abs(y1 - y0);              /* magnitude of y-extent       */
    int step_x = (x0 < x1) ? 1 : -1;    /* unit direction in x         */
    int step_y = (y0 < y1) ? 1 : -1;    /* unit direction in y         */
    int err    = dx - dy;               /* deviation accumulator       */

    for (;;) {
        plot_cell(y0, x0, cp, ch, rows, cols);
        if (x0 == x1 && y0 == y1) break;

        /* Bresenham step: whichever axis is more "behind" advances. */
        int err2 = 2 * err;
        if (err2 > -dy) { err -= dy; x0 += step_x; }   /* step in x */
        if (err2 <  dx) { err += dx; y0 += step_y; }   /* step in y */
    }
}

/* How many samples to draw around an ellipse perimeter.  Crude
 * Ramanujan-style perimeter scaling (~ pi * (rx + ry)) with a floor and
 * ceiling so tiny circles still close cleanly and huge ones don't melt
 * the renderer. */
static int ellipse_sample_count(float rx, float ry)
{
    int n = (int)(TAU * (rx + ry) * ELLIPSE_PERIM_FACTOR) + ELLIPSE_MIN_STEPS;
    if (n > ELLIPSE_MAX_STEPS) n = ELLIPSE_MAX_STEPS;
    return n;
}

/*
 * draw_ellipse() -- parametric ellipse rasterizer in cell space.
 *
 * Samples theta uniformly around [0, TAU), plots one cell per sample.
 * A geometric circumcircle of radius r in geo-space becomes an ellipse
 * with rx = r, ry = r / ASPECT_Y when projected back to cells -- the
 * caller (render_circumcircle) applies that aspect correction.
 */
static void draw_ellipse(float cx, float cy, float rx, float ry,
                         int cp, chtype ch, int rows, int cols)
{
    int n_samples = ellipse_sample_count(rx, ry);
    for (int i = 0; i < n_samples; i++) {
        float theta = TAU * (float)i / (float)n_samples;
        int   x     = (int)(cx + rx * cosf(theta) + 0.5f);
        int   y     = (int)(cy + ry * sinf(theta) + 0.5f);
        plot_cell(y, x, cp, ch, rows, cols);
    }
}

/* ===================================================================== */
/* S5  mesh + Bowyer-Watson                                               */
/* ===================================================================== */

/*
 * Mesh -- the triangulation under construction.
 *
 * WHY a struct (and not naked globals like the first draft had):
 *      Putting pts + tris + their counts in one struct makes "the
 *      triangulation" a first-class value.  Every Bowyer-Watson step
 *      takes (Mesh *m, ...), so the algorithm is decoupled from the
 *      enclosing demo and could be lifted into another program with
 *      no edits to the math.
 *
 * Invariants maintained at every external call site:
 *      I1.  pts[0..SUPER_COUNT-1] are the SUPER-TRIANGLE vertices.
 *           Placed far outside the screen by mesh_init_super so that
 *           every real point falls inside that giant triangle.  This
 *           lets Bowyer-Watson start from a valid one-triangle Delaunay
 *           triangulation and grow it incrementally.  See Sloan 1987 [3].
 *
 *      I2.  pts[SUPER_COUNT .. npts-1] are the real input points,
 *           filled in by mesh_gen_points and revealed one at a time by
 *           the INSERT phase.
 *
 *      I3.  tris[0..ntris-1] is a Delaunay triangulation of
 *           pts[0..npts-1].  After every mesh_insert_point() call this
 *           holds again -- that is the entire correctness claim.  See
 *           Bowyer 1981 [1], Watson 1981 [2], de Berg ch. 9 [4].
 *
 *      I4.  Triangles that touch any super-vertex (index < SUPER_COUNT)
 *           are "virtual" -- they exist so the algorithm has somewhere
 *           to anchor edges along the convex hull.  They are filtered
 *           out by mesh_real_tri_count and showcase_init.
 *
 * Fields:
 *      pts     vertex pool.  Capacity MAX_PTS = N_POINTS + SUPER_COUNT + 2
 *              -- the +2 is slack for any future "bounding rectangle"
 *              technique that could replace the super-triangle.
 *      npts    number of valid entries in pts[].
 *      tris    triangle list.  Capacity MAX_TRIS = N_POINTS*8 + 10.
 *              The 8x factor is the loose worst-case for the planar
 *              triangulation of N points (Euler's formula gives ~2N-5
 *              triangles for points in general position; the 8x leaves
 *              ample headroom for intermediate states during insertion
 *              where tris[] briefly contains both old and new entries).
 *      ntris   number of valid entries in tris[].
 *
 * Memory:  ~4 KB at MAX_PTS=45, MAX_TRIS=330.  Sits inside Scene which
 *      sits inside the static g_app -- no heap, no per-frame allocation.
 */
typedef struct {
    Pt   pts [MAX_PTS];    /* I1: 0..2 super-triangle; I2: 3..npts-1 real */
    int  npts;             /* count of valid entries in pts[]             */
    Tri  tris[MAX_TRIS];   /* I3: a Delaunay triangulation of pts[0..npts-1] */
    int  ntris;            /* count of valid entries in tris[]            */
} Mesh;

/*
 * Scratch -- per-insertion working memory for Bowyer-Watson.
 *
 * WHY this is its own struct:
 *      mesh_insert_point() needs two temporary arrays whose size depends
 *      on Mesh capacity (one bool per triangle, up to MAX_HOLE boundary
 *      edges).  Three options were considered:
 *
 *        a) `static` locals inside mesh_insert_point  -- works but hides
 *           the dependency, breaks reentrancy, and prevents the demo
 *           from running two triangulations in parallel.
 *        b) heap allocation per call  -- forbidden by the "no malloc
 *           after init" rule (see CLAUDE.md).
 *        c) caller-owned struct  -- chosen.  The hot path performs zero
 *           allocation, and the cost (~8 KB sitting idle in Scene) is
 *           visible in one place.
 *
 * WHY two arrays, not one:
 *      bad[]  is indexed by triangle id, must be sized MAX_TRIS.
 *      hole[] is indexed by edge id, can hold up to 3 * MAX_TRIS edges
 *             in the pathological case (every bad triangle contributes
 *             all three of its edges as boundary).
 *      Different shapes => different types => different arrays.
 *
 * Lifecycle within one mesh_insert_point() call:
 *      bw_find_bad      writes bad[]                  (memset + flag)
 *      bw_collect_hole  reads  bad[],  writes hole[]  (returns nhole)
 *      bw_remove_bad    reads  bad[]                  (compacts tris[])
 *      bw_fill_hole     reads  hole[]                 (appends to tris[])
 *      -- both arrays become garbage between calls; no cross-call state.
 */
typedef struct {
    bool  bad [MAX_TRIS];  /* bad[i] iff tris[i]'s circumcircle contains P */
    HEdge hole[MAX_HOLE];  /* boundary of the union of bad triangles       */
} Scratch;

/* ---- Mesh accessors / queries ---------------------------------------- */

/* Read out triangle t's three vertices already converted to GEOMETRY space.
 * Every Bowyer-Watson predicate and circumcircle renderer needs exactly
 * this -- so the gy() aspect correction lives in one place. */
static void tri_geo_vertices(const Mesh *m, const Tri *t,
                             float *ax, float *ay,
                             float *bx, float *by,
                             float *cx, float *cy)
{
    *ax = m->pts[t->a].x;   *ay = gy(m->pts[t->a].y);
    *bx = m->pts[t->b].x;   *by = gy(m->pts[t->b].y);
    *cx = m->pts[t->c].x;   *cy = gy(m->pts[t->c].y);
}

/* True iff all three vertices are real points (none is a super-vertex).
 * The "ghost triangles" anchored on the super-triangle exist for the
 * algorithm's bookkeeping but must not appear in user-visible counts. */
static inline bool tri_is_real(const Tri *t)
{
    return t->a >= SUPER_COUNT && t->b >= SUPER_COUNT && t->c >= SUPER_COUNT;
}

/* The rectangle in which random points may spawn -- shrunk away from
 * screen edges (MARGIN_X) and away from the HUD bands (HUD_TOP_ROWS,
 * HUD_BOTTOM_ROWS).  A degenerate terminal (too small) is clamped to a
 * 1-cell range so rand() % range never divides by zero. */
static void mesh_spawn_area(int cols, int rows,
                            int *xlo, int *xhi, int *ylo, int *yhi)
{
    *xlo = MARGIN_X;
    *xhi = cols - MARGIN_X;
    *ylo = HUD_TOP_ROWS    + MARGIN_Y;
    *yhi = rows - HUD_BOTTOM_ROWS - MARGIN_Y;
    if (*xhi <= *xlo) *xhi = *xlo + 1;
    if (*yhi <= *ylo) *yhi = *ylo + 1;
}

/* ---- Mesh constructors ---------------------------------------------- */

/*
 * mesh_init_super() -- seed the triangulation with one giant "super-
 * triangle" placed far enough outside the screen that every future real
 * point will fall strictly inside it.
 *
 * Why this is necessary: Bowyer-Watson is INCREMENTAL -- it needs a
 * starting Delaunay triangulation to insert into.  The smallest such
 * triangulation is a single triangle, and the simplest way to guarantee
 * that a stream of future points all land inside it is to make it
 * absurdly large.  See Sloan 1987 [3].
 *
 * The 3x/4x screen-size offsets are loose-but-safe; precise minimal
 * super-triangles exist but are pointless at this scale.
 */
static void mesh_init_super(Mesh *m, int cols, int rows)
{
    float W = (float)cols, H = (float)rows;

    /* Vertices far outside the screen, indices 0/1/2 by convention. */
    m->pts[0] = (Pt){  W * 0.5f,   -H * 3.0f };   /* far above center */
    m->pts[1] = (Pt){ -W * 3.0f,    H * 4.0f };   /* far below-left   */
    m->pts[2] = (Pt){  W * 4.0f,    H * 4.0f };   /* far below-right  */

    /* The whole triangulation starts as one triangle: the super-tri. */
    m->tris[0] = (Tri){ 0, 1, 2 };
    m->npts    = SUPER_COUNT;
    m->ntris   = 1;
}

/* Fill m->pts[SUPER_COUNT .. SUPER_COUNT+N_POINTS-1] with random spawn
 * positions.  Layout-aware -- avoids the HUD bands so no point spawns
 * behind a label. */
static void mesh_gen_points(Mesh *m, int cols, int rows, unsigned seed)
{
    srand(seed);
    int xlo, xhi, ylo, yhi;
    mesh_spawn_area(cols, rows, &xlo, &xhi, &ylo, &yhi);
    for (int i = 0; i < N_POINTS; i++) {
        m->pts[SUPER_COUNT + i].x = (float)(xlo + rand() % (xhi - xlo));
        m->pts[SUPER_COUNT + i].y = (float)(ylo + rand() % (yhi - ylo));
    }
}

/* Count triangles that do not touch any super-vertex. */
static int mesh_real_tri_count(const Mesh *m)
{
    int n = 0;
    for (int i = 0; i < m->ntris; i++)
        if (tri_is_real(&m->tris[i])) n++;
    return n;
}

/* --- Bowyer-Watson: the algorithm split into four named steps --------- */

/*
 * Step 1 -- mark every triangle whose circumcircle contains P (geo space).
 * Such triangles violate Delaunay once P is added; they will be removed
 * and the cavity left behind will be re-triangulated against P.
 */
static void bw_find_bad(const Mesh *m, float px, float py, bool *bad)
{
    memset(bad, 0, (size_t)m->ntris * sizeof(bool));
    for (int i = 0; i < m->ntris; i++) {
        float ax,ay, bx,by, cx,cy;
        tri_geo_vertices(m, &m->tris[i], &ax,&ay, &bx,&by, &cx,&cy);
        if (in_circumcircle(ax,ay, bx,by, cx,cy, px,py))
            bad[i] = true;
    }
}

/*
 * Does directed edge (va -> vb) appear in any OTHER bad triangle?
 *
 * The hole boundary (step 2) is exactly the set of bad-triangle edges
 * that have NO mate among the other bad triangles.  Edges shared by
 * two bad triangles are interior to the cavity and discarded; edges
 * shared by exactly one bad triangle survive and form the hole's
 * boundary (a star-shaped polygon visible from P).
 *
 * Matching is direction-agnostic: an edge {a,b} in one triangle pairs
 * with edge {b,a} traversed the opposite way by its neighbour.
 */
static bool bw_edge_is_shared(const Mesh *m, const bool *bad,
                              int self_idx, int va, int vb)
{
    for (int j = 0; j < m->ntris; j++) {
        if (j == self_idx || !bad[j]) continue;
        int ws[3] = { m->tris[j].a, m->tris[j].b, m->tris[j].c };
        for (int f = 0; f < 3; f++) {
            int wa = ws[f], wb = ws[(f + 1) % 3];
            bool same_dir     = (va == wa && vb == wb);
            bool opposite_dir = (va == wb && vb == wa);
            if (same_dir || opposite_dir) return true;
        }
    }
    return false;
}

/*
 * Step 2 -- walk every edge of every bad triangle; keep only those that
 * are NOT shared with another bad triangle.  Those unmatched edges form
 * the cavity boundary that step 4 will fan-triangulate against P.
 */
static int bw_collect_hole(const Mesh *m, const bool *bad, HEdge *hole)
{
    int nhole = 0;
    for (int i = 0; i < m->ntris; i++) {
        if (!bad[i]) continue;
        int vs[3] = { m->tris[i].a, m->tris[i].b, m->tris[i].c };
        for (int e = 0; e < 3; e++) {
            int va = vs[e];
            int vb = vs[(e + 1) % 3];
            if (bw_edge_is_shared(m, bad, i, va, vb)) continue;   /* interior */
            if (nhole >= MAX_HOLE) continue;                      /* safety   */
            hole[nhole++] = (HEdge){ va, vb };                    /* boundary */
        }
    }
    return nhole;
}

/* Step 3 -- compact the triangle list, dropping every flagged triangle. */
static void bw_remove_bad(Mesh *m, const bool *bad)
{
    int w = 0;
    for (int i = 0; i < m->ntris; i++)
        if (!bad[i]) m->tris[w++] = m->tris[i];
    m->ntris = w;
}

/* Step 4 -- fan-triangulate the hole: connect P to each boundary edge. */
static void bw_fill_hole(Mesh *m, const HEdge *hole, int nhole, int idx)
{
    for (int e = 0; e < nhole && m->ntris < MAX_TRIS; e++)
        m->tris[m->ntris++] = (Tri){ hole[e].a, hole[e].b, idx };
}

/*
 * mesh_insert_point() -- Bowyer-Watson driver.
 * Adds the point already stored at m->pts[idx] to the triangulation.
 */
static void mesh_insert_point(Mesh *m, Scratch *s, int idx)
{
    float px = m->pts[idx].x;
    float py = gy(m->pts[idx].y);
    bw_find_bad     (m, px, py, s->bad);
    int nhole =
    bw_collect_hole (m, s->bad, s->hole);
    bw_remove_bad   (m, s->bad);
    bw_fill_hole    (m, s->hole, nhole, idx);
}

/* ===================================================================== */
/* S6  scene                                                              */
/* ===================================================================== */

/*
 * Phase -- which of the three sequential acts the demo is playing.
 *
 *      PHASE_INSERT     "the build": reveal one random point per sim tick,
 *                       call mesh_insert_point, draw the growing mesh.
 *                       Lasts N_POINTS ticks.
 *      PHASE_SHOWCASE   "the proof": iterate over every real triangle,
 *                       draw its circumcircle, count points inside.
 *                       For a correct implementation the count is 0
 *                       (the empty-circumcircle property -- de Berg ch. 9 [4]).
 *                       Lasts nrtris * SHOWCASE_HOLD * SHOWCASE_CYCLES ticks.
 *      PHASE_DONE       brief pause, then loop with a new seed.
 *                       Lasts DONE_TICKS ticks.
 *
 * The state machine lives in scene_tick (S6) and only that one function
 * is allowed to mutate Scene.phase.
 */
typedef enum { PHASE_INSERT, PHASE_SHOWCASE, PHASE_DONE } Phase;

/* Human-readable name for the HUD.  Lives next to Phase so adding a new
 * phase forces the compiler to remind us to add a label here too. */
static const char *phase_label(Phase p)
{
    switch (p) {
    case PHASE_INSERT:   return "INSERTING";
    case PHASE_SHOWCASE: return "SHOWCASE";
    case PHASE_DONE:     return "RESTARTING";
    }
    return "?";
}

/*
 * Insertion -- progress state for the INSERT phase.
 *
 * WHY a struct of one field:
 *      Currently there is only one cursor to track, but giving it its
 *      own struct (rather than a loose Scene.insert_idx) does two things:
 *        1. groups any future insert-phase state (e.g. interpolation
 *           progress for animated point arrival) in one place;
 *        2. makes call sites self-documenting: sc->insert.next_idx
 *           reads as "the insertion cursor", not "an int on Scene".
 *
 * Semantics:
 *      next_idx is the index in Mesh.pts of the NEXT real point to be
 *      revealed and inserted by insertion_step.  It progresses through
 *      the half-open range [SUPER_COUNT, SUPER_COUNT + N_POINTS).
 *      When it reaches SUPER_COUNT + N_POINTS the phase transitions
 *      to SHOWCASE.
 *
 *      Convention: at any point, Mesh.npts == next_idx (the prefix of
 *      pts[] revealed so far equals the prefix processed so far).
 */
typedef struct {
    int next_idx;    /* next point to insert; range [SUPER_COUNT, SUPER_COUNT+N_POINTS) */
} Insertion;

/*
 * Showcase -- iterator state for the SHOWCASE phase.
 *
 * WHY rtris[] is a separate array (not just "skip super triangles inline"):
 *      The showcase cursor advances by one PER TICK across the real
 *      triangles only.  If we filtered inline ("skip if any vertex
 *      index < SUPER_COUNT") the cursor arithmetic would have to walk
 *      Mesh.tris[] from the start every tick to find the i-th real
 *      triangle.  Pre-computing the index list once (showcase_init) at
 *      INSERT->SHOWCASE transition makes per-tick access O(1).
 *
 *      The cost is MAX_TRIS ints (~1.3 KB) sitting on Scene -- cheap.
 *
 * WHY (idx, ticks, pass) and not a single counter:
 *      idx     advances per triangle, used by the renderer to pick which
 *              triangle to highlight.
 *      ticks   counts sim-ticks spent on the current triangle; lets the
 *              demo "dwell" on each one for SHOWCASE_HOLD frames so a
 *              human can see it (decoupled from sim_fps speed control).
 *      pass    counts completed sweeps through rtris[]; SHOWCASE_CYCLES
 *              full passes end the phase.  Useful if we ever raise the
 *              cycle count to let the viewer see the verification twice.
 *
 *      Three separate counters mean each can change independently for
 *      reasons documented by its name.  A single packed counter would
 *      hide those three independent rates of change.
 */
typedef struct {
    int rtris[MAX_TRIS];  /* pre-filtered indices into Mesh.tris (real tris only) */
    int nrtris;           /* count of valid entries in rtris[]                    */
    int idx;              /* cursor into rtris[]; advances when ticks >= HOLD     */
    int ticks;            /* sim-ticks spent on rtris[idx]; resets at HOLD        */
    int pass;             /* full sweeps through rtris[] so far; ends at CYCLES   */
} Showcase;

/*
 * Scene -- the entire simulation state in one struct.
 *
 * WHY no globals:
 *      First-draft code used a fistful of g_pts/g_ntris/g_rtris globals.
 *      Folding them onto Scene buys three things:
 *        1. The function signature `scene_tick(Scene *)` documents what
 *           the simulation depends on; you cannot accidentally read
 *           triangulation state from somewhere else.
 *        2. Multiple Scenes could coexist (a future split-screen demo,
 *           or a "before/after" comparison view) with no code changes
 *           in the algorithm layer.
 *        3. Memory ownership is one variable: g_app contains the App
 *           which contains the Scene which contains everything.
 *
 * Composition (each substruct's own block-comment has the details):
 *      phase     which of the three acts is running (Phase enum above)
 *      mesh      the triangulation itself: points + triangles + counts
 *      scratch   per-insertion working memory for Bowyer-Watson
 *      insert    progress cursor for PHASE_INSERT
 *      show      iterator state for PHASE_SHOWCASE
 *
 * Loose Scene-level fields (don't belong to any sub-act):
 *      done_ticks  countdown during PHASE_DONE before scene_start_insert
 *      seed        RNG seed; bumped on reset (r key) and on auto-restart
 *      cols, rows  cached terminal dimensions; updated on SIGWINCH
 *      paused      user toggle (SPACE); freezes scene_tick
 *      step_req    user request (s key) for a single tick while paused
 */
typedef struct {
    Phase     phase;       /* state-machine cursor; only scene_tick writes it */
    Mesh      mesh;        /* the triangulation under construction            */
    Scratch   scratch;     /* Bowyer-Watson per-call working memory           */
    Insertion insert;      /* PHASE_INSERT progress                           */
    Showcase  show;        /* PHASE_SHOWCASE iterator                         */
    int       done_ticks;  /* PHASE_DONE countdown to auto-restart            */
    unsigned  seed;        /* RNG seed; r-key adds RESET_SEED_BUMP, auto-restart adds RESTART_SEED_BUMP */
    int       cols, rows;  /* terminal size cached from screen_init / resize  */
    bool      paused;      /* user pause toggle (SPACE)                       */
    bool      step_req;    /* one-shot tick request while paused (s)          */
} Scene;

/* Build Showcase.rtris[] -- the subset of tris that excludes the super-triangle. */
static void showcase_init(Showcase *sh, const Mesh *m)
{
    sh->nrtris = 0;
    for (int i = 0; i < m->ntris; i++) {
        Tri t = m->tris[i];
        if (t.a >= SUPER_COUNT && t.b >= SUPER_COUNT && t.c >= SUPER_COUNT)
            sh->rtris[sh->nrtris++] = i;
    }
}

/* Index of the triangle currently being showcased, or -1 if none. */
static int showcase_current_tri(const Showcase *sh)
{
    if (sh->nrtris <= 0) return -1;
    return sh->rtris[sh->idx % sh->nrtris];
}

/*
 * insertion_step() -- one sim-tick of the INSERT phase.
 * Reveals one new point and re-triangulates; when all are in, transitions
 * to SHOWCASE.
 */
static void insertion_step(Scene *sc)
{
    if (sc->insert.next_idx < SUPER_COUNT + N_POINTS) {
        sc->mesh.npts = sc->insert.next_idx + 1;
        mesh_insert_point(&sc->mesh, &sc->scratch, sc->insert.next_idx);
        sc->insert.next_idx++;
    } else {
        showcase_init(&sc->show, &sc->mesh);
        sc->phase    = PHASE_SHOWCASE;
        sc->show.idx = 0;
    }
}

/*
 * showcase_step() -- one sim-tick of the SHOWCASE phase.
 * Advances to the next triangle every SHOWCASE_HOLD ticks; after
 * SHOWCASE_CYCLES full passes, transitions to DONE.
 */
static void showcase_step(Scene *sc)
{
    sc->show.ticks++;
    if (sc->show.ticks < SHOWCASE_HOLD) return;

    sc->show.ticks = 0;
    sc->show.idx++;
    if (sc->show.idx < sc->show.nrtris) return;

    sc->show.pass++;
    sc->show.idx = 0;
    if (sc->show.pass >= SHOWCASE_CYCLES) {
        sc->phase      = PHASE_DONE;
        sc->done_ticks = 0;
    }
}

/* Begin a fresh insertion run with the current seed. */
static void scene_start_insert(Scene *sc)
{
    sc->phase           = PHASE_INSERT;
    sc->insert.next_idx = SUPER_COUNT;
    sc->show.idx        = 0;
    sc->show.ticks      = 0;
    sc->show.pass       = 0;
    sc->show.nrtris     = 0;
    sc->done_ticks      = 0;
    mesh_init_super (&sc->mesh, sc->cols, sc->rows);
    mesh_gen_points (&sc->mesh, sc->cols, sc->rows, sc->seed);
}

static void scene_init(Scene *sc, int cols, int rows)
{
    sc->cols     = cols;
    sc->rows     = rows;
    sc->seed     = (unsigned)time(NULL);
    sc->paused   = false;
    sc->step_req = false;
    scene_start_insert(sc);
}

/*
 * done_step() -- one sim-tick of the DONE phase.
 *
 * The DONE phase exists only to give the viewer a brief pause after the
 * SHOWCASE verifies the property.  After DONE_TICKS we bump the seed by
 * a coprime offset (so we visit a fresh RNG sub-sequence rather than a
 * neighbouring one) and start over.
 */
static void done_step(Scene *sc)
{
    sc->done_ticks++;
    if (sc->done_ticks < DONE_TICKS) return;
    sc->seed += RESTART_SEED_BUMP;
    scene_start_insert(sc);
}

/*
 * scene_tick() -- one sim-tick of whichever phase is currently active.
 *
 * Reads as pseudocode:
 *   if paused (and no single-step request)  -> do nothing
 *   else                                    -> dispatch on current phase
 *
 * Only this function may transition between phases.
 */
static void scene_tick(Scene *sc)
{
    if (sc->paused && !sc->step_req) return;
    sc->step_req = false;

    switch (sc->phase) {
    case PHASE_INSERT:    insertion_step(sc);  break;
    case PHASE_SHOWCASE:  showcase_step(sc);   break;
    case PHASE_DONE:      done_step(sc);       break;
    }
}

/* ===================================================================== */
/* S7  render                                                             */
/* ===================================================================== */

/*
 * TriCells -- the three vertices of a Tri, snapped to integer cells.
 *
 * WHY a struct (not just six locals at each call site):
 *      Two renderers (render_mesh_edges, render_highlight) need exactly
 *      the same six numbers, computed exactly the same way.  Bundling
 *      them as TriCells lets one tri_cells() call replace twelve casts.
 *      The actual round-to-nearest convention is one level deeper, in
 *      cell_round() -- switching to floor()/lroundf() would change that
 *      one helper and propagate everywhere automatically.
 *
 * Coordinate space:
 *      Cell space (NOT geometry space).  Bresenham/draw_line work in
 *      whole cell units; aspect correction is the renderer's job for
 *      the circle path only (render_circumcircle), not the line path.
 *      See Bresenham 1965 [7].
 *
 * Naming: ax,ay are vertex a's column,row; bx,by vertex b; cx,cy vertex c.
 *      Two-letter names beat aliased locals because Bresenham takes
 *      pairs and the call site reads naturally as
 *          draw_line(c.ax,c.ay, c.bx,c.by, ...);
 */
typedef struct { int ax, ay, bx, by, cx, cy; } TriCells;

/* ---- coordinate helpers --------------------------------------------- */

/* Round-to-nearest cell coordinates of a single Pt. */
static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* Cell-space (integer) coordinates of a triangle's three vertices.
 * The one place rounding happens for rendering. */
static TriCells tri_cells(const Mesh *m, const Tri *t)
{
    return (TriCells){
        .ax = cell_round(m->pts[t->a].x),  .ay = cell_round(m->pts[t->a].y),
        .bx = cell_round(m->pts[t->b].x),  .by = cell_round(m->pts[t->b].y),
        .cx = cell_round(m->pts[t->c].x),  .cy = cell_round(m->pts[t->c].y),
    };
}

/* ---- circle-geometry helpers ---------------------------------------- */

/* Is geo-space point (px,py) inside a circle with squared radius r2?
 * Uses VIOLATION_EPS slack so points sitting exactly on the boundary
 * aren't flagged.  Squared form -- no sqrt needed for the predicate. */
static inline bool pt_in_circle_sqr(float px, float py,
                                    float ocx, float ocy, float r2)
{
    return vec2_sqr_mag(px - ocx, py - ocy) < r2 - VIOLATION_EPS;
}

/* The aspect-corrected projection of a geometry-space circle into the
 * cell-space ellipse that draw_ellipse() actually rasterises.  X is
 * unchanged (cells and geo agree on width); Y compresses by ASPECT_Y. */
typedef struct { float cx, cy, rx, ry; } CellEllipse;

static CellEllipse circle_to_cell_ellipse(float ocx_geo, float ocy_geo, float r_geo)
{
    return (CellEllipse){
        .cx = ocx_geo,                   /* geo-x == cell-x       */
        .cy = ocy_geo / ASPECT_Y,        /* compress y to cells   */
        .rx = r_geo,
        .ry = r_geo / ASPECT_Y,
    };
}

/* ---- single-cell renderers ------------------------------------------ */

/* Mark the circumcenter as a bright '+' dot. */
static void plot_circumcenter(float cx_f, float cy_f, int rows, int cols)
{
    int x = cell_round(cx_f), y = cell_round(cy_f);
    attron(COLOR_PAIR(CP_CIRC_CTR) | A_BOLD);
    plot_cell(y, x, CP_CIRC_CTR, '+', rows, cols);
    attroff(COLOR_PAIR(CP_CIRC_CTR) | A_BOLD);
}

/* Mark a single input point. */
static void plot_point(const Pt *p, int cp, int rows, int cols)
{
    int x = cell_round(p->x), y = cell_round(p->y);
    attron(COLOR_PAIR(cp) | A_BOLD);
    plot_cell(y, x, cp, '@', rows, cols);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* Mark a Delaunay-property violator as a red 'X'. */
static void plot_violation(const Pt *p, int rows, int cols)
{
    int x = cell_round(p->x), y = cell_round(p->y);
    attron(COLOR_PAIR(CP_POINT_IN) | A_BOLD);
    plot_cell(y, x, CP_POINT_IN, 'X', rows, cols);
    attroff(COLOR_PAIR(CP_POINT_IN) | A_BOLD);
}

/* ---- triangle-level geometry query ---------------------------------- */

/*
 * tri_inside_count() -- count points inside triangle t's circumcircle
 * (excluding t's own vertices).  This is the Delaunay verification
 * predicate played out at the visualisation layer: zero means the
 * empty-circumcircle property holds for this triangle.
 *
 * Out-parameters expose the circle (center + radius) for HUD display;
 * pass NULL if uninterested.
 */
static int tri_inside_count(const Mesh *m, const Tri *t,
                            float *out_ocx, float *out_ocy, float *out_r)
{
    float ax,ay, bx,by, cx,cy;
    tri_geo_vertices(m, t, &ax,&ay, &bx,&by, &cx,&cy);

    float ocx, ocy, r2;
    if (!circumcircle_geo(ax,ay, bx,by, cx,cy, &ocx,&ocy,&r2)) {
        if (out_ocx) *out_ocx = 0;
        if (out_ocy) *out_ocy = 0;
        if (out_r)   *out_r   = 0;
        return 0;
    }

    int inside = 0;
    for (int i = SUPER_COUNT; i < m->npts; i++) {
        if (i == t->a || i == t->b || i == t->c) continue;       /* skip own vertices */
        float px = m->pts[i].x, py = gy(m->pts[i].y);
        if (pt_in_circle_sqr(px, py, ocx, ocy, r2)) inside++;
    }

    if (out_ocx) *out_ocx = ocx;
    if (out_ocy) *out_ocy = ocy;
    if (out_r)   *out_r   = sqrtf(r2);
    return inside;
}

/* ---- per-layer renderers -------------------------------------------- */

/* Layer 1 -- every real triangle's three edges as dim blue dots. */
static void render_mesh_edges(const Mesh *m, int rows, int cols)
{
    for (int i = 0; i < m->ntris; i++) {
        const Tri *t = &m->tris[i];
        if (!tri_is_real(t)) continue;
        TriCells c = tri_cells(m, t);
        draw_line(c.ax,c.ay, c.bx,c.by, CP_EDGE, '.', rows,cols);
        draw_line(c.bx,c.by, c.cx,c.cy, CP_EDGE, '.', rows,cols);
        draw_line(c.cx,c.cy, c.ax,c.ay, CP_EDGE, '.', rows,cols);
    }
}

/* Layer 2 -- the showcase triangle's edges in bright yellow. */
static void render_highlight(const Mesh *m, const Tri *t, int rows, int cols)
{
    TriCells c = tri_cells(m, t);
    draw_line(c.ax,c.ay, c.bx,c.by, CP_EDGE_HI, '+', rows,cols);
    draw_line(c.bx,c.by, c.cx,c.cy, CP_EDGE_HI, '+', rows,cols);
    draw_line(c.cx,c.cy, c.ax,c.ay, CP_EDGE_HI, '+', rows,cols);
}

/* Layer 3-4 -- circumcircle ring + circumcenter dot. */
static void render_circumcircle(const Mesh *m, const Tri *t, int rows, int cols)
{
    float ax,ay, bx,by, cx,cy;
    tri_geo_vertices(m, t, &ax,&ay, &bx,&by, &cx,&cy);

    float ocx, ocy, r2;
    if (!circumcircle_geo(ax,ay, bx,by, cx,cy, &ocx,&ocy,&r2)) return;

    CellEllipse e = circle_to_cell_ellipse(ocx, ocy, sqrtf(r2));
    draw_ellipse(e.cx, e.cy, e.rx, e.ry, CP_CIRC, 'o', rows, cols);
    plot_circumcenter(e.cx, e.cy, rows, cols);
}

/*
 * Layer 5 -- every revealed real point as '@'.  In INSERT, the most-
 * recently inserted point is yellow; everything else white.
 */
static void render_points(const Mesh *m, Phase phase, int newest,
                          int rows, int cols)
{
    for (int i = SUPER_COUNT; i < m->npts; i++) {
        bool is_newest = (i == newest && phase == PHASE_INSERT);
        plot_point(&m->pts[i], is_newest ? CP_POINT_NEW : CP_POINT, rows, cols);
    }
}

/*
 * Layer 6 -- red 'X' overlay on any point that violates the empty-
 * circumcircle property for the showcased triangle.  In a correct
 * Bowyer-Watson implementation this layer always paints zero pixels --
 * its presence on screen would mean a bug.
 */
static void render_violations(const Mesh *m, const Tri *t, int rows, int cols)
{
    float ax,ay, bx,by, cx,cy;
    tri_geo_vertices(m, t, &ax,&ay, &bx,&by, &cx,&cy);

    float ocx, ocy, r2;
    if (!circumcircle_geo(ax,ay, bx,by, cx,cy, &ocx,&ocy,&r2)) return;

    for (int i = SUPER_COUNT; i < m->npts; i++) {
        if (i == t->a || i == t->b || i == t->c) continue;       /* skip own vertices */
        float px = m->pts[i].x, py = gy(m->pts[i].y);
        if (pt_in_circle_sqr(px, py, ocx, ocy, r2))
            plot_violation(&m->pts[i], rows, cols);
    }
}

/* render_scene() -- composite the six visualisation layers in back-to-front order. */
static void render_scene(const Scene *sc, int rows, int cols)
{
    const Mesh *m = &sc->mesh;

    render_mesh_edges(m, rows, cols);

    int ti = (sc->phase == PHASE_SHOWCASE) ? showcase_current_tri(&sc->show) : -1;
    if (ti >= 0) {
        const Tri *t = &m->tris[ti];
        render_highlight   (m, t, rows, cols);
        render_circumcircle(m, t, rows, cols);
    }

    int newest = sc->insert.next_idx - 1;
    render_points(m, sc->phase, newest, rows, cols);

    if (ti >= 0) render_violations(m, &m->tris[ti], rows, cols);
}

/* ---- HUD row helpers ------------------------------------------------- */

/* Row 0 left -- the title block. */
static void hud_draw_title(void)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(0, 0,
        " Delaunay Triangulation  [Bowyer-Watson incremental]   N=%d points",
        N_POINTS);
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
}

/* Row 0 right -- live engine stats: render fps, sim Hz, paused/running. */
static void hud_draw_engine_stats(const Scene *sc, double fps, int sim_fps,
                                  int cols)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%2d Hz  %s ",
             fps, sim_fps, sc->paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len >= cols) return;                       /* no room */

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, cols - len, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Row 1 -- triangle count, point count, phase name. */
static void hud_draw_counts(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(1, 1,
        " triangles: %3d   points: %3d/%d   phase: %s",
        mesh_real_tri_count(&sc->mesh),
        sc->insert.next_idx - SUPER_COUNT, N_POINTS,
        phase_label(sc->phase));
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Row 2 (SHOWCASE) left -- which triangle, its circumradius, its center. */
static void hud_draw_circle_stats(const Scene *sc, float r, float ocx, float ocy)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(2, 1,
        " tri %d/%d   radius: %.1f   center: (%.1f, %.1f)",
        sc->show.idx % sc->show.nrtris + 1, sc->show.nrtris,
        (double)r, (double)ocx, (double)(ocy / ASPECT_Y));
    attroff(COLOR_PAIR(CP_HUD));
}

/* Row 2 (SHOWCASE) right -- the empty-circumcircle verdict, green/red. */
static void hud_draw_verdict(int inside, int cols)
{
    int verdict_color = (inside == 0) ? CP_OK : CP_WARN;
    int xcol          = cols - HUD_VERDICT_RIGHT_OFFSET;
    if (xcol < 1) xcol = 1;

    attron(COLOR_PAIR(verdict_color) | A_BOLD);
    mvprintw(2, xcol,
        " empty circumcircle: %s   (%d inside) ",
        inside == 0 ? "YES" : "NO!", inside);
    attroff(COLOR_PAIR(verdict_color) | A_BOLD);
}

/* Row 2 (SHOWCASE) -- compose the stats + verdict for the current triangle. */
static void hud_draw_showcase_detail(const Scene *sc, int cols)
{
    int ti = showcase_current_tri(&sc->show);
    if (ti < 0) return;
    const Tri *t = &sc->mesh.tris[ti];

    float ocx, ocy, r;
    int inside = tri_inside_count(&sc->mesh, t, &ocx, &ocy, &r);

    hud_draw_circle_stats(sc, r, ocx, ocy);
    hud_draw_verdict(inside, cols);
}

/* Row 2 (INSERT) -- one-line progress tip. */
static void hud_draw_insert_progress(const Scene *sc)
{
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(2, 1,
        " inserting point %d/%d -- each insertion repairs circumcircle property",
        sc->insert.next_idx - SUPER_COUNT, N_POINTS);
    attroff(COLOR_PAIR(CP_LABEL));
}

/* Dashed divider line. */
static void hud_draw_separator(int row, int cols)
{
    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(row, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));
}

/*
 * render_header() -- top HUD (rows 0..HUD_TOP_ROWS-1) carries the data.
 *
 *   Row 0  title (left)                +  fps / sim Hz / paused (right)
 *   Row 1  triangles, points, phase
 *   Row 2  showcase circumcircle stats OR insertion progress
 *   Row 3  separator
 */
static void render_header(const Scene *sc, double fps, int sim_fps,
                          int rows, int cols)
{
    (void)rows;

    hud_draw_title();
    hud_draw_engine_stats(sc, fps, sim_fps, cols);
    hud_draw_counts(sc);

    if (sc->phase == PHASE_SHOWCASE && sc->show.nrtris > 0)
        hud_draw_showcase_detail(sc, cols);
    else if (sc->phase == PHASE_INSERT)
        hud_draw_insert_progress(sc);

    hud_draw_separator(HUD_SEPARATOR_ROW, cols);
}

/* render_overlay() -- bottom HUD (row rows-1) carries the action keys. */
static void render_overlay(int rows, int cols)
{
    (void)cols;
    attron(COLOR_PAIR(CP_HEADER)|A_BOLD);
    mvprintw(rows-1, 1,
        " q:quit   SPACE:pause   s:step   r:reset   +/-:speed ");
    attroff(COLOR_PAIR(CP_HEADER)|A_BOLD);
}

/* ===================================================================== */
/* S8  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal geometry.
 *
 * WHY cache cols/rows on a struct instead of calling getmaxyx() everywhere:
 *      getmaxyx is cheap but not free, and -- more importantly -- many
 *      functions in the render layer take rows/cols as plain ints.
 *      Passing one Screen* would conflate "I need the size" with "I need
 *      to talk to ncurses".  Holding the size as a value lets the math
 *      layer stay terminal-agnostic; only screen_init / screen_resize /
 *      screen_draw ever touch ncurses.
 *
 * Update points:
 *      screen_init      -- once at startup
 *      screen_resize    -- after a SIGWINCH (raised by the kernel on
 *                          terminal resize; handled in S9 app)
 *
 * Fields are in ncurses convention: cols = X-extent, rows = Y-extent.
 * Both are 1-based counts, so valid coordinates are [0, cols-1] x [0, rows-1].
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); cbreak(); noecho();
    curs_set(0); keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    render_header(sc, fps, sim_fps, s->rows, s->cols);
    render_scene(sc, s->rows, s->cols);
    render_overlay(s->rows, s->cols);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* S9  app                                                                */
/* ===================================================================== */

/*
 * App -- the top-level container.  Exists as a single static instance
 * (g_app, below) so the signal handlers can poke flags on it without
 * needing globals scattered across the file.
 *
 * WHY signal handler flags are on App, not Scene:
 *      Signals interrupt the main loop, not the simulation.  "Should I
 *      quit?" and "did the terminal change size?" are concerns of the
 *      run loop in main(), not of Bowyer-Watson.  Keeping them out of
 *      Scene means Scene stays pure simulation state -- snapshottable,
 *      restartable, testable in isolation.
 *
 * WHY volatile sig_atomic_t (and not bool):
 *      Per POSIX, sig_atomic_t is the only integer type guaranteed
 *      atomic with respect to async signal delivery, and `volatile`
 *      forces the main loop to re-read from memory each iteration
 *      rather than caching in a register.  Without both qualifiers
 *      the compiler may legally hoist the flag read out of the loop
 *      and the program would never see the signal-set value.
 *
 * Fields:
 *      scene        the simulation itself; everything algorithm-related
 *      screen       cached terminal dimensions (see Screen above)
 *      sim_fps      tick rate of scene_tick; +/- keys adjust it within
 *                   [SIM_FPS_MIN, SIM_FPS_MAX].  Independent of render
 *                   fps (TARGET_FPS), which is fixed at 60.
 *      running      set to 0 by SIGINT/SIGTERM handler; main loop
 *                   checks each iteration and exits cleanly via atexit.
 *      need_resize  set to 1 by SIGWINCH handler; main loop calls
 *                   screen_resize + scene_init on next iteration.
 */
typedef struct {
    Scene                 scene;       /* simulation state                    */
    Screen                screen;      /* terminal size cache                 */
    int                   sim_fps;     /* sim-tick rate; +/- adjust at runtime */
    volatile sig_atomic_t running;     /* 0 = main loop should exit (SIGINT)  */
    volatile sig_atomic_t need_resize; /* 1 = main loop should re-init screen */
} App;

static App g_app;

static void cleanup(void)   { endwin(); }
static void on_exit(int s)  { (void)s; g_app.running    = 0; }
static void on_resize(int s){ (void)s; g_app.need_resize = 1; }

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':
        app->scene.paused = !app->scene.paused;
        break;
    case 's': case 'S':
        app->scene.paused   = true;
        app->scene.step_req = true;
        break;
    case 'r': case 'R':
        app->scene.seed += RESET_SEED_BUMP;
        scene_start_insert(&app->scene);
        break;
    case '+': case '=':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps++;
        break;
    case '-': case '_':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps--;
        break;
    default: break;
    }
    return true;
}

/* ---- main-loop step helpers ----------------------------------------- */

/*
 * Compute wallclock dt since the previous frame, capped at DT_CAP_NS so
 * a long pause (debugger, terminal suspend) doesn't unleash a flood of
 * sim ticks trying to catch up.
 */
static int64_t frame_dt_clamped(int64_t *last_ns)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *last_ns;
    *last_ns    = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/*
 * Fixed-timestep accumulator: drain whole sim ticks at the current
 * sim_fps rate.  Decouples simulation rate from render fps so visual
 * speed is the same on a 30 fps and a 144 fps terminal.
 *
 * Reference for this pattern: "Fix Your Timestep!" -- Glenn Fiedler.
 */
static void frame_drain_sim_ticks(Scene *sc, int sim_fps, int64_t *accum)
{
    int64_t tick_ns = TICK_NS(sim_fps);
    while (*accum >= tick_ns) {
        scene_tick(sc);
        *accum -= tick_ns;
    }
}

/* Rolling FPS counter -- average frames per second over the most recent
 * FPS_UPDATE_MS window, then reset. */
static void fps_counter_update(int64_t dt, int64_t *accum_ns,
                               int *frame_count, double *fps_out)
{
    *accum_ns += dt;
    (*frame_count)++;
    if (*accum_ns < FPS_UPDATE_MS * NS_PER_MS) return;

    double seconds = (double)(*accum_ns) / (double)NS_PER_SEC;
    *fps_out      = (double)(*frame_count) / seconds;
    *frame_count  = 0;
    *accum_ns     = 0;
}

/* Sleep just long enough to maintain TARGET_FPS, given how much wall time
 * this frame already spent (work_so_far covers everything since
 * frame_start_ns). */
static void frame_sleep_to_target(int64_t frame_start_ns, int64_t work_so_far)
{
    int64_t spent = clock_ns() - frame_start_ns + work_so_far;
    clock_sleep_ns(FRAME_PERIOD_NS - spent);
}

/* Pull pending keystroke; dispatch to app_handle_key.  Returns false
 * only when the user pressed a quit key (caller should exit). */
static bool drain_input(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Respond to SIGWINCH: re-query terminal size, rebuild Scene, reset
 * timing so the next dt isn't enormous. */
static void apply_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
    *frame_time      = clock_ns();
    *sim_accum       = 0;
}

/* Install handlers, wire atexit cleanup. */
static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);
}

/* Bring the App up: defaults, screen, scene. */
static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

/*
 * main() -- the orchestrator.  Each line is one named phase of a frame.
 */
int main(void)
{
    install_signal_handlers();
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) apply_resize(app, &frame_time, &sim_accum);

        int64_t dt = frame_dt_clamped(&frame_time);

        sim_accum += dt;
        frame_drain_sim_ticks(&app->scene, app->sim_fps, &sim_accum);

        fps_counter_update(dt, &fps_accum, &frame_count, &fps_display);

        frame_sleep_to_target(frame_time, dt);
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);

        if (!drain_input(app)) app->running = 0;
    }
    return 0;
}
