/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * Delaunay triangulation, watched as it grows.  Random points drop in one
 * at a time and the mesh keeps the Delaunay property after each one; then a
 * showcase pass draws each triangle's circumcircle and checks that no other
 * point sits inside it.
 *
 * Algorithm: Bowyer-Watson incremental insertion (Bowyer 1981; Watson 1981;
 *   de Berg et al., "Computational Geometry", ch. 9).
 * Sister file: procedural/generational/delaunay_triangulation.c -- same
 *   algorithm framed as a generative-art demo rather than this showcase.
 */

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

/* -- S1 config -- */

#define SIM_FPS_DEFAULT   8
#define SIM_FPS_MIN       1
#define SIM_FPS_MAX      30
#define TARGET_FPS       60
#define FPS_UPDATE_MS    500

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* A character cell is about twice as tall as it is wide.  Stretch y by this
 * factor before any distance or circle math, so circles come out round
 * instead of egg-shaped. */
#define ASPECT_Y  2.0f

#define N_POINTS         40     /* how many random points to triangulate  */
#define SUPER_COUNT       3     /* corners of the giant starter triangle  */
#define MAX_PTS          (N_POINTS + SUPER_COUNT + 2)
#define MAX_TRIS         (N_POINTS * 8 + 10)
#define MAX_HOLE         (MAX_TRIS * 3)

#define HUD_TOP_ROWS      4     /* top rows kept for the stats display     */
#define HUD_BOTTOM_ROWS   1     /* bottom row kept for the key hints       */
#define MARGIN_X          4
#define MARGIN_Y          3

#define SHOWCASE_HOLD     1     /* ticks to linger on each triangle        */
#define SHOWCASE_CYCLES   1
#define DONE_TICKS       10

/* If a frame's elapsed time blows past this (e.g. the program was paused in
 * a debugger), pretend only this much passed -- otherwise the sim would try
 * to catch up with a flood of ticks. */
#define DT_CAP_NS         (100 * NS_PER_MS)
#define FRAME_PERIOD_NS   (NS_PER_SEC / TARGET_FPS)

/* Wiggle room for the geometry tests.  Floats are only good to ~6 digits,
 * so we leave a little slack above that noise rather than trust exact ties. */
#define DEGEN_TRI_EPS     1e-7f   /* below this the three points are basically
                                     in a line, so no circle through them    */
#define INCIRCLE_EPS      1e-5f   /* slack on "is the new point inside?"      */
#define VIOLATION_EPS     1e-4f   /* same test for the HUD verdict, kept a bit
                                     looser so a point right on the edge isn't
                                     wrongly flagged                          */

#define TAU               (2.0f * (float)M_PI)  /* one full turn, in radians */

/* How many dots to walk around a circle's outline -- more for bigger
 * circles, with a floor so tiny ones still close up and a ceiling so huge
 * ones don't bog down. */
#define ELLIPSE_PERIM_FACTOR  0.75f
#define ELLIPSE_MIN_STEPS     32
#define ELLIPSE_MAX_STEPS    512

/* Jumps added to the random seed on restart.  These are picked to land far
 * from the current seed, so each run looks clearly different. */
#define RESET_SEED_BUMP    9973                 /* user presses 'r'        */
#define RESTART_SEED_BUMP  7919                 /* auto-restart after DONE */

#define HUD_VERDICT_RIGHT_OFFSET  40            /* where the YES/NO verdict sits, from the right edge */
#define HUD_SEPARATOR_ROW          3            /* the dashed divider line */

/* -- S2 clock -- */

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

/* -- S3 color -- */

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

/* -- S4 geometry primitives -- */

/*
 * Pt -- a 2D point.  Every coordinate in the program is one of these, so
 * the meaning of (x, y) lives in one place.
 *
 *   x  column on screen; one unit = one character wide.
 *   y  row on screen;    one unit = one character tall (and a character is
 *      about twice as tall as it is wide -- see ASPECT_Y).
 *
 * A Pt is always in this screen form.  The circle math needs x and y at the
 * same real-world scale, so it stretches y on the fly via gy() -- the point
 * itself is never changed.  float is plenty for ~40 points on a terminal.
 */
typedef struct { float x, y; } Pt;

/*
 * Tri -- a triangle, kept as three slot numbers into Mesh.pts, not three
 * copies of the points.
 *
 * Why slot numbers instead of coordinates:
 *   - One point is usually a corner of several triangles; everyone pointing
 *     at the same slot keeps them in sync.
 *   - Checking whether two triangles share an edge is then a plain
 *     whole-number compare (bw_edge_is_shared) -- no fuzzy float matching.
 *   - Slots 0..SUPER_COUNT-1 are reserved for the giant starter triangle,
 *     so "is this a real triangle?" is just "are all three slots >= that?".
 *
 * The corner order (a, b, c) doesn't matter here -- the algorithm only cares
 * which corners an edge connects, not which way the triangle winds.
 */
typedef struct { int a, b, c; } Tri;

/*
 * HEdge -- one edge of the hole boundary, as the two corner slots a and b
 * it runs between.
 *
 * When we add a point, some triangles have to be torn out, leaving a hole.
 * The hole's outline is the set of edges that belonged to exactly one of
 * the torn-out triangles (bw_collect_hole gathers them).  We then sew the
 * new point onto each of these edges to fill the hole (bw_fill_hole).
 *
 * The order a -> b is kept, not sorted, so the new triangles come out wound
 * the same way without an extra check.  This is just the bare minimum for
 * one hole -- not a full mesh edge structure with neighbour links.
 */
typedef struct { int a, b; } HEdge;

/* Stretch a screen-row value into the same scale as columns, so circles
 * come out round.  Every circle calculation goes through this. */
static inline float gy(float cy) { return cy * ASPECT_Y; }

/* How far (x, y) is from the origin, but left squared.  Most of our tests
 * ("inside the circle?", "too small?") work fine on squared distances, and
 * skipping the square root keeps them cheap; we only take the real root once,
 * to print a radius. */
static inline float vec2_sqr_mag(float x, float y) { return x*x + y*y; }

/* A number whose sign tells which way the corners A, B, C wind (positive =
 * counter-clockwise) and whose size is twice the triangle's area -- near
 * zero means the three points are nearly in a straight line. */
static inline float tri_signed_area_doubled(float ax, float ay,
                                            float bx, float by,
                                            float cx, float cy)
{
    return ax*(by-cy) + bx*(cy-ay) + cx*(ay-by);
}

/* Draw one character on screen, ignoring anything off the edges.  Every dot,
 * line, and circle goes through here, so the off-screen check lives in just
 * one spot. */
static inline void plot_cell(int y, int x, int cp, chtype ch,
                             int rows, int cols)
{
    if (x < 0 || x >= cols || y < 0 || y >= rows) return;
    attron(COLOR_PAIR(cp));
    mvaddch(y, x, ch);
    attroff(COLOR_PAIR(cp));
}

/*
 * Find the one circle that passes through all three corners of a triangle.
 * Hands back its center (ocx, ocy) and its radius-squared (r2), all in the
 * stretched circle-math scale the inputs are already in.  Returns false if
 * the corners are nearly in a line, since then no such circle exists.
 *
 * The center is the spot equally far from all three corners; the algebra
 * below is the standard closed-form solution for it (de Berg, ch. 9).
 */
static bool circumcircle_geo(float ax, float ay, float bx, float by,
                              float cx, float cy,
                              float *ocx, float *ocy, float *r2)
{
    float two_area = 2.0f * tri_signed_area_doubled(ax,ay, bx,by, cx,cy);
    if (fabsf(two_area) < DEGEN_TRI_EPS) return false;   /* corners in a line */

    float ma2 = vec2_sqr_mag(ax, ay);
    float mb2 = vec2_sqr_mag(bx, by);
    float mc2 = vec2_sqr_mag(cx, cy);

    *ocx = (ma2*(by-cy) + mb2*(cy-ay) + mc2*(ay-by)) / two_area;
    *ocy = (ma2*(cx-bx) + mb2*(ax-cx) + mc2*(bx-ax)) / two_area;

    *r2  = vec2_sqr_mag(*ocx - ax, *ocy - ay);
    return true;
}

/* Is point P inside the triangle's circle?  The little slack (INCIRCLE_EPS)
 * keeps points sitting right on the rim from flickering in and out as we
 * build the mesh. */
static bool in_circumcircle(float ax, float ay, float bx, float by,
                             float cx, float cy, float px, float py)
{
    float ocx, ocy, r2;
    if (!circumcircle_geo(ax, ay, bx, by, cx, cy, &ocx, &ocy, &r2))
        return false;
    return vec2_sqr_mag(px - ocx, py - ocy) < r2 - INCIRCLE_EPS;
}

/* Draw a straight line of characters between two cells using only integer
 * steps (Bresenham's classic line algorithm). */
static void draw_line(int x0, int y0, int x1, int y1,
                      int cp, chtype ch, int rows, int cols)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int step_x = (x0 < x1) ? 1 : -1;
    int step_y = (y0 < y1) ? 1 : -1;
    int err    = dx - dy;              /* tracks which way we've drifted */

    for (;;) {
        plot_cell(y0, x0, cp, ch, rows, cols);
        if (x0 == x1 && y0 == y1) break;

        int err2 = 2 * err;
        if (err2 > -dy) { err -= dy; x0 += step_x; }
        if (err2 <  dx) { err += dx; y0 += step_y; }
    }
}

/* How many dots to space around a circle's outline -- bigger circles get
 * more, kept between a floor (so small ones still close up) and a ceiling
 * (so big ones stay fast). */
static int ellipse_sample_count(float rx, float ry)
{
    int n = (int)(TAU * (rx + ry) * ELLIPSE_PERIM_FACTOR) + ELLIPSE_MIN_STEPS;
    if (n > ELLIPSE_MAX_STEPS) n = ELLIPSE_MAX_STEPS;
    return n;
}

/* Draw an oval by stepping a dot all the way around it.  Drawing the circle
 * as an oval (taller radius squashed) cancels out the cell aspect ratio, so
 * it reads as a real circle on screen; the caller works out the radii. */
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

/* -- S5 mesh + Bowyer-Watson -- */

/*
 * Mesh -- the triangulation as it's being built: the points, the triangles,
 * and how many of each are in use.  Bundling them lets every step of the
 * algorithm take one Mesh* and stay self-contained.
 *
 * A few things stay true the whole time:
 *   - The first SUPER_COUNT points are the corners of the giant starter
 *     triangle (set by mesh_init_super), placed way off-screen so every real
 *     point lands inside it.  Bowyer-Watson needs some valid triangle to
 *     start from, and one this big swallows everything.
 *   - The points after that are the real ones, revealed one at a time.
 *   - After each point is added, the triangle list is once again a proper
 *     Delaunay triangulation of everything revealed so far -- that's the
 *     whole promise of the algorithm.
 *   - Triangles touching a starter corner are "ghosts": scaffolding the
 *     algorithm needs but the user never sees (filtered out for counts and
 *     the showcase).
 *
 * The array sizes are loose over-estimates with room to spare, so the lists
 * never overflow.  It all lives in static storage -- no heap, no per-frame
 * allocation.
 */
typedef struct {
    Pt   pts [MAX_PTS];    /* first SUPER_COUNT are the starter corners, rest real */
    int  npts;             /* how many points are in use                  */
    Tri  tris[MAX_TRIS];   /* the current triangulation                   */
    int  ntris;            /* how many triangles are in use               */
} Mesh;

/*
 * Scratch -- a notepad for one point insertion.  Holds the two temporary
 * lists Bowyer-Watson needs while it works.  It belongs to the caller (not
 * hidden statics, not a fresh malloc) so the hot path never allocates and
 * the memory cost is plain to see.
 *
 *   bad   one flag per triangle: does this triangle's circle swallow the
 *         new point, so it must be torn out?
 *   hole  the edges left around the resulting hole, to sew the new point onto.
 *
 * Both are filled fresh each insertion and mean nothing between calls.
 */
typedef struct {
    bool  bad [MAX_TRIS];
    HEdge hole[MAX_HOLE];
} Scratch;

/* Hand back a triangle's three corners already stretched into circle-math
 * scale -- the form every circle test and circle drawer wants. */
static void tri_geo_vertices(const Mesh *m, const Tri *t,
                             float *ax, float *ay,
                             float *bx, float *by,
                             float *cx, float *cy)
{
    *ax = m->pts[t->a].x;   *ay = gy(m->pts[t->a].y);
    *bx = m->pts[t->b].x;   *by = gy(m->pts[t->b].y);
    *cx = m->pts[t->c].x;   *cy = gy(m->pts[t->c].y);
}

/* A real triangle is one with no corner on the starter triangle -- those
 * ghost triangles are scaffolding and shouldn't show up in counts. */
static inline bool tri_is_real(const Tri *t)
{
    return t->a >= SUPER_COUNT && t->b >= SUPER_COUNT && t->c >= SUPER_COUNT;
}

/* The box random points may land in, pulled in from the screen edges and
 * away from the HUD strips so nothing spawns under a label.  A tiny terminal
 * is clamped to at least 1 cell wide so the random pick never divides by zero. */
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

/*
 * Start the mesh off as one enormous triangle drawn far off-screen, big
 * enough that every real point to come will sit inside it.  Bowyer-Watson
 * has to grow from some valid triangle, and one this big is the easy way to
 * be sure nothing ever falls outside.
 */
static void mesh_init_super(Mesh *m, int cols, int rows)
{
    float W = (float)cols, H = (float)rows;

    m->pts[0] = (Pt){  W * 0.5f,   -H * 3.0f };   /* far above center */
    m->pts[1] = (Pt){ -W * 3.0f,    H * 4.0f };   /* far below-left   */
    m->pts[2] = (Pt){  W * 4.0f,    H * 4.0f };   /* far below-right  */

    m->tris[0] = (Tri){ 0, 1, 2 };
    m->npts    = SUPER_COUNT;
    m->ntris   = 1;
}

/* Scatter the real points at random, keeping clear of the HUD strips so none
 * lands behind a label. */
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

/* Count the triangles the user actually sees (ghosts excluded). */
static int mesh_real_tri_count(const Mesh *m)
{
    int n = 0;
    for (int i = 0; i < m->ntris; i++)
        if (tri_is_real(&m->tris[i])) n++;
    return n;
}

/* --- Bowyer-Watson, in four named steps --- */

/* Step 1 -- flag every triangle whose circle swallows the new point P.
 * Those are the ones that go wrong once P joins, so they get torn out and
 * the gap is re-stitched around P. */
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

/* Does this edge also belong to some other torn-out triangle?  If two
 * torn-out triangles share an edge, that edge is inside the hole and we drop
 * it; an edge owned by only one survives as part of the hole's rim.  The two
 * corners can be listed in either order and still count as the same edge. */
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

/* Step 2 -- gather the rim of the hole: every edge of a torn-out triangle
 * that no other torn-out triangle shares.  Step 4 fans P out to these. */
static int bw_collect_hole(const Mesh *m, const bool *bad, HEdge *hole)
{
    int nhole = 0;
    for (int i = 0; i < m->ntris; i++) {
        if (!bad[i]) continue;
        int vs[3] = { m->tris[i].a, m->tris[i].b, m->tris[i].c };
        for (int e = 0; e < 3; e++) {
            int va = vs[e];
            int vb = vs[(e + 1) % 3];
            if (bw_edge_is_shared(m, bad, i, va, vb)) continue;   /* inside the hole */
            if (nhole >= MAX_HOLE) continue;                      /* don't overflow  */
            hole[nhole++] = (HEdge){ va, vb };                    /* rim edge        */
        }
    }
    return nhole;
}

/* Step 3 -- drop the flagged triangles, closing the gaps in the list. */
static void bw_remove_bad(Mesh *m, const bool *bad)
{
    int w = 0;
    for (int i = 0; i < m->ntris; i++)
        if (!bad[i]) m->tris[w++] = m->tris[i];
    m->ntris = w;
}

/* Step 4 -- fill the hole: make a new triangle from P to each rim edge. */
static void bw_fill_hole(Mesh *m, const HEdge *hole, int nhole, int idx)
{
    for (int e = 0; e < nhole && m->ntris < MAX_TRIS; e++)
        m->tris[m->ntris++] = (Tri){ hole[e].a, hole[e].b, idx };
}

/* Add the point sitting at pts[idx] to the mesh -- the four steps in order. */
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

/* -- S6 scene -- */

/*
 * Phase -- which of the three acts the demo is in:
 *   PHASE_INSERT    the build -- drop in one point per tick and re-stitch.
 *   PHASE_SHOWCASE  the proof -- walk each triangle, draw its circle, and
 *                   check no other point is inside it (for correct output
 *                   the count is always zero).
 *   PHASE_DONE      a short pause, then start over with a fresh seed.
 *
 * Only scene_tick is allowed to switch acts.
 */
typedef enum { PHASE_INSERT, PHASE_SHOWCASE, PHASE_DONE } Phase;

/* Name for the HUD.  Kept next to Phase so adding an act makes the compiler
 * nag us to add its label too. */
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
 * Insertion -- where the build phase is up to.
 *
 *   next_idx  the slot of the next real point to reveal and add.  It walks
 *             from the first real point to the last; once past the last,
 *             the demo moves on to the showcase.
 *
 * It's its own little struct (not a loose field on Scene) so the call sites
 * read clearly and any future build-phase state has a home.
 */
typedef struct {
    int next_idx;
} Insertion;

/*
 * Showcase -- where the proof phase is up to as it visits each triangle.
 *
 *   rtris   the list of real triangles to walk, worked out once up front so
 *           that stepping to the next one each tick is instant (rather than
 *           re-scanning past the ghosts every time).
 *   nrtris  how many entries rtris holds.
 *   idx     which triangle in rtris we're showing now.
 *   ticks   how long we've lingered on this triangle; we hold each one for a
 *           few ticks so it's actually watchable.
 *   pass    how many full sweeps we've made; after enough, the phase ends.
 *
 * Three separate counters (idx, ticks, pass) because each ticks at its own
 * rate for its own reason.
 */
typedef struct {
    int rtris[MAX_TRIS];
    int nrtris;
    int idx;
    int ticks;
    int pass;
} Showcase;

/*
 * Scene -- everything the simulation needs, in one place (no globals).
 * Keeping it together means scene_tick(Scene*) plainly states what it
 * touches, and the whole thing can be reset or copied as a unit.
 *
 *   phase       which act is playing (only scene_tick changes it)
 *   mesh        the triangulation itself
 *   scratch     Bowyer-Watson's notepad for the current insertion
 *   insert      build-phase progress
 *   show        proof-phase progress
 *   done_ticks  countdown during the pause before restarting
 *   seed        the random seed; bumped on reset and on auto-restart
 *   cols, rows  current terminal size, refreshed on resize
 *   paused      the SPACE toggle that freezes the sim
 *   step_req    the 's' request for a single tick while paused
 */
typedef struct {
    Phase     phase;
    Mesh      mesh;
    Scratch   scratch;
    Insertion insert;
    Showcase  show;
    int       done_ticks;
    unsigned  seed;
    int       cols, rows;
    bool      paused;
    bool      step_req;
} Scene;

/* Collect the real triangles (skipping ghosts) into rtris so the showcase
 * can step through them. */
static void showcase_init(Showcase *sh, const Mesh *m)
{
    sh->nrtris = 0;
    for (int i = 0; i < m->ntris; i++) {
        Tri t = m->tris[i];
        if (t.a >= SUPER_COUNT && t.b >= SUPER_COUNT && t.c >= SUPER_COUNT)
            sh->rtris[sh->nrtris++] = i;
    }
}

/* Which triangle is on display right now, or -1 if there are none. */
static int showcase_current_tri(const Showcase *sh)
{
    if (sh->nrtris <= 0) return -1;
    return sh->rtris[sh->idx % sh->nrtris];
}

/* One tick of the build: reveal the next point and re-stitch; once they're
 * all in, switch to the showcase. */
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

/* One tick of the proof: linger on each triangle a moment, move to the next,
 * and after enough full sweeps move on to the pause. */
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

/* Start a fresh build from scratch with the current seed. */
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

/* One tick of the pause after the proof.  Once it's held long enough, jump
 * the seed to a clearly different one and start the whole thing over. */
static void done_step(Scene *sc)
{
    sc->done_ticks++;
    if (sc->done_ticks < DONE_TICKS) return;
    sc->seed += RESTART_SEED_BUMP;
    scene_start_insert(sc);
}

/* Advance whichever act is playing by one tick (unless paused).  This is the
 * only place acts switch. */
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

/* -- S7 render -- */

/*
 * TriCells -- a triangle's three corners rounded to whole screen cells, ready
 * for the line drawer.  ax,ay is corner a's column and row, and so on.  We
 * bundle the six numbers so the two edge-drawing functions share one rounding
 * step instead of repeating it.  These are plain screen cells -- the round
 * circle is the only thing that needs the aspect stretch, not the edges.
 */
typedef struct { int ax, ay, bx, by, cx, cy; } TriCells;

/* Round a coordinate to the nearest whole cell. */
static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* A triangle's three corners as whole cells -- the one spot rounding happens
 * for drawing. */
static TriCells tri_cells(const Mesh *m, const Tri *t)
{
    return (TriCells){
        .ax = cell_round(m->pts[t->a].x),  .ay = cell_round(m->pts[t->a].y),
        .bx = cell_round(m->pts[t->b].x),  .by = cell_round(m->pts[t->b].y),
        .cx = cell_round(m->pts[t->c].x),  .cy = cell_round(m->pts[t->c].y),
    };
}

/* Is the point inside the circle (squared radius r2)?  A touch of slack
 * (VIOLATION_EPS) so a point sitting right on the rim isn't counted in. */
static inline bool pt_in_circle_sqr(float px, float py,
                                    float ocx, float ocy, float r2)
{
    return vec2_sqr_mag(px - ocx, py - ocy) < r2 - VIOLATION_EPS;
}

/* A circle from the math turned into the squashed oval we actually draw on
 * screen.  Width stays as is; height is compressed to undo the cell stretch. */
typedef struct { float cx, cy, rx, ry; } CellEllipse;

static CellEllipse circle_to_cell_ellipse(float ocx_geo, float ocy_geo, float r_geo)
{
    return (CellEllipse){
        .cx = ocx_geo,
        .cy = ocy_geo / ASPECT_Y,
        .rx = r_geo,
        .ry = r_geo / ASPECT_Y,
    };
}

/* Mark the circle's center with a bright '+'. */
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

/* Mark a point that broke the empty-circle rule with a red 'X'. */
static void plot_violation(const Pt *p, int rows, int cols)
{
    int x = cell_round(p->x), y = cell_round(p->y);
    attron(COLOR_PAIR(CP_POINT_IN) | A_BOLD);
    plot_cell(y, x, CP_POINT_IN, 'X', rows, cols);
    attroff(COLOR_PAIR(CP_POINT_IN) | A_BOLD);
}

/*
 * Count how many other points fall inside this triangle's circle (its own
 * three corners don't count).  Zero is the goal -- it means the triangle
 * obeys the empty-circle rule that makes the mesh Delaunay.  The circle's
 * center and radius come back through the out-params for the HUD; pass NULL
 * to skip them.
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

/* Draw every real triangle's three edges as dim blue dots. */
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

/* Draw the triangle on display in bright yellow. */
static void render_highlight(const Mesh *m, const Tri *t, int rows, int cols)
{
    TriCells c = tri_cells(m, t);
    draw_line(c.ax,c.ay, c.bx,c.by, CP_EDGE_HI, '+', rows,cols);
    draw_line(c.bx,c.by, c.cx,c.cy, CP_EDGE_HI, '+', rows,cols);
    draw_line(c.cx,c.cy, c.ax,c.ay, CP_EDGE_HI, '+', rows,cols);
}

/* Draw the triangle's circle plus a dot at its center. */
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

/* Draw every revealed point as '@'.  During the build, the one just added is
 * yellow and the rest are white. */
static void render_points(const Mesh *m, Phase phase, int newest,
                          int rows, int cols)
{
    for (int i = SUPER_COUNT; i < m->npts; i++) {
        bool is_newest = (i == newest && phase == PHASE_INSERT);
        plot_point(&m->pts[i], is_newest ? CP_POINT_NEW : CP_POINT, rows, cols);
    }
}

/* Put a red 'X' on any point caught inside the displayed triangle's circle.
 * If the algorithm is right this never draws anything -- seeing one means a bug. */
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

/* Draw the whole picture, back layer to front. */
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

/* The title, top-left. */
static void hud_draw_title(void)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(0, 0,
        " Delaunay Triangulation  [Bowyer-Watson incremental]   N=%d points",
        N_POINTS);
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
}

/* Live stats top-right: frames per second, sim speed, and paused/running. */
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

/* Triangle count, point count, and current phase. */
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

/* During the proof: which triangle, and its circle's radius and center. */
static void hud_draw_circle_stats(const Scene *sc, float r, float ocx, float ocy)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(2, 1,
        " tri %d/%d   radius: %.1f   center: (%.1f, %.1f)",
        sc->show.idx % sc->show.nrtris + 1, sc->show.nrtris,
        (double)r, (double)ocx, (double)(ocy / ASPECT_Y));
    attroff(COLOR_PAIR(CP_HUD));
}

/* The verdict for the current triangle: green if its circle is empty, red if not. */
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

/* Pull the current triangle's circle stats and verdict together onto one row. */
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

/* During the build: a one-line progress note. */
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

/* Draw the top HUD strip: title and stats, the counts, the phase-specific
 * line, and the divider under them. */
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

/* Draw the bottom strip listing the keys you can press. */
static void render_overlay(int rows, int cols)
{
    (void)cols;
    attron(COLOR_PAIR(CP_HEADER)|A_BOLD);
    mvprintw(rows-1, 1,
        " q:quit   SPACE:pause   s:step   r:reset   +/-:speed ");
    attroff(COLOR_PAIR(CP_HEADER)|A_BOLD);
}

/* -- S8 screen -- */

/*
 * Screen -- the terminal's current width and height, remembered so the
 * drawing code can take plain numbers and never has to ask ncurses itself.
 * Only the three functions below touch ncurses; everyone else just gets the
 * size.  It's refreshed at startup and again whenever the window resizes.
 * cols is the width, rows the height, so valid spots run 0..cols-1 across
 * and 0..rows-1 down.
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

/* -- S9 app -- */

/*
 * App -- the whole program in one box, kept as a single static instance so
 * the signal handlers have something to flip flags on without scattered
 * globals.  Those flags live here, not on Scene, because quitting and
 * resizing are the run loop's business, leaving Scene as pure sim state.
 *
 *   scene        the simulation
 *   screen       the terminal size
 *   sim_fps      how fast the sim ticks; the +/- keys nudge it, separate
 *                from how fast we redraw
 *   running      the quit signal sets this to 0; the loop then exits cleanly
 *   need_resize  the resize signal sets this to 1; the loop rebuilds next time
 *
 * running and need_resize are volatile sig_atomic_t -- the one type a signal
 * can safely set, with volatile so the loop actually re-reads it each pass
 * instead of caching a stale copy.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
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

/* How much real time passed since last frame, capped so a long stall doesn't
 * make the sim try to catch up all at once. */
static int64_t frame_dt_clamped(int64_t *last_ns)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *last_ns;
    *last_ns    = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned.  This
 * keeps the sim's pace the same whatever the redraw rate.  (Glenn Fiedler,
 * "Fix Your Timestep!") */
static void frame_drain_sim_ticks(Scene *sc, int sim_fps, int64_t *accum)
{
    int64_t tick_ns = TICK_NS(sim_fps);
    while (*accum >= tick_ns) {
        scene_tick(sc);
        *accum -= tick_ns;
    }
}

/* Work out the average frame rate over the last little while, then start a
 * fresh window. */
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

/* Nap for whatever's left of this frame's time budget, so we hold a steady
 * frame rate instead of running flat out. */
static void frame_sleep_to_target(int64_t frame_start_ns, int64_t work_so_far)
{
    int64_t spent = clock_ns() - frame_start_ns + work_so_far;
    clock_sleep_ns(FRAME_PERIOD_NS - spent);
}

/* Handle one pending keypress.  Returns false only if it was a quit key. */
static bool drain_input(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Handle a window resize: get the new size, rebuild the scene to fit, and
 * reset the clock so the next frame's elapsed time isn't huge. */
static void apply_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
    *frame_time      = clock_ns();
    *sim_accum       = 0;
}

/* Wire up the quit/resize signals and make sure the terminal is restored on exit. */
static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);
}

/* Get everything ready to run: defaults, the screen, and the scene. */
static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

/* The main loop: each pass measures time, runs the sim, draws, and reads input. */
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
