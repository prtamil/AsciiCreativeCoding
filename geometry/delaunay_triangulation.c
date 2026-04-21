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
 * -------------------------------------------------------------------------
 *  Section map
 * -------------------------------------------------------------------------
 *  S1  config       -- sizes, timing, aspect ratio
 *  S2  clock        -- monotonic ns clock + sleep
 *  S3  color        -- palette
 *  S4  geometry     -- Pt/Tri/Edge types, circumcircle math, line/ellipse draw
 *  S5  triangulation -- insert_point(), update_mesh(), Bowyer-Watson core
 *  S6  scene        -- state machine (INSERT -> SHOWCASE -> DONE -> restart)
 *  S7  render       -- render_scene(), render_overlay()
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
 * Empty circumcircle property (S5 in_circumcircle):
 *   A triangulation is Delaunay iff for every triangle no other point in
 *   the set lies strictly inside its circumscribed circle.
 *   Equivalently, among all triangulations of the same point set, Delaunay
 *   maximises the minimum interior angle -- it avoids thin "sliver" triangles.
 *
 * Bowyer-Watson incremental insertion (S5 insert_point):
 *   To insert point P into an existing Delaunay triangulation:
 *     1. Find all "bad" triangles whose circumcircle contains P.
 *     2. Remove them; their union forms a star-shaped "hole".
 *     3. Re-triangulate the hole by connecting P to every boundary edge.
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

#define HUD_ROWS          8     /* rows reserved at bottom for overlay    */
#define MARGIN_X          4
#define MARGIN_Y          3

#define SHOWCASE_HOLD     1     /* sim-ticks per triangle in showcase     */
#define SHOWCASE_CYCLES   1
#define DONE_TICKS       10

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
/* S4  geometry                                                           */
/* ===================================================================== */

typedef struct { float x, y; } Pt;
typedef struct { int a, b, c; } Tri;
typedef struct { int a, b;    } HEdge;   /* directed boundary edge */

/* Convert cell-space y to geometry space (aspect-corrected Euclidean). */
static inline float gy(float cy) { return cy * ASPECT_Y; }

/*
 * circumcircle_geo() -- circumcircle of triangle in GEOMETRY space.
 *
 * All three input vertices must already be in geometry space (x unchanged,
 * y = cell_y * ASPECT_Y).  Outputs center (ocx, ocy) and squared radius r2,
 * also in geometry space.
 *
 * Standard formula via perpendicular bisectors:
 *   D  = 2[Ax(By-Cy) + Bx(Cy-Ay) + Cx(Ay-By)]
 *   Ox = [|A|^2(By-Cy) + |B|^2(Cy-Ay) + |C|^2(Ay-By)] / D
 *   Oy = [|A|^2(Cx-Bx) + |B|^2(Ax-Cx) + |C|^2(Bx-Ax)] / D
 */
static bool circumcircle_geo(float ax, float ay, float bx, float by,
                              float cx, float cy,
                              float *ocx, float *ocy, float *r2)
{
    float D = 2.0f * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by));
    if (fabsf(D) < 1e-7f) return false;
    float a2 = ax*ax + ay*ay;
    float b2 = bx*bx + by*by;
    float c2 = cx*cx + cy*cy;
    *ocx = (a2*(by-cy) + b2*(cy-ay) + c2*(ay-by)) / D;
    *ocy = (a2*(cx-bx) + b2*(ax-cx) + c2*(bx-ax)) / D;
    float dx = *ocx - ax, dy = *ocy - ay;
    *r2 = dx*dx + dy*dy;
    return true;
}

/*
 * in_circumcircle() -- true iff point P (geo space) is strictly inside
 * the circumcircle of triangle (A,B,C) (geo space).
 * A small epsilon handles numerical boundary cases.
 */
static bool in_circumcircle(float ax, float ay, float bx, float by,
                             float cx, float cy, float px, float py)
{
    float ocx, ocy, r2;
    if (!circumcircle_geo(ax, ay, bx, by, cx, cy, &ocx, &ocy, &r2))
        return false;
    float dx = px - ocx, dy = py - ocy;
    return (dx*dx + dy*dy) < r2 - 1e-5f;
}

/* Bresenham line draw in cell space. */
static void draw_line(int x0, int y0, int x1, int y1,
                      int cp, chtype ch, int rows, int cols)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = (x0<x1)?1:-1, sy = (y0<y1)?1:-1;
    int err = dx - dy;
    for (;;) {
        if (x0>=0 && x0<cols && y0>=0 && y0<rows) {
            attron(COLOR_PAIR(cp));
            mvaddch(y0, x0, ch);
            attroff(COLOR_PAIR(cp));
        }
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_ellipse() -- parametric ellipse at (cx,cy) with semi-axes (rx,ry)
 * in cell space.
 *
 * A circumcircle of geo-radius r appears as an ellipse with
 *   rx = r           (cell units, x unchanged)
 *   ry = r/ASPECT_Y  (cell units, y compressed by aspect ratio)
 */
static void draw_ellipse(float cx, float cy, float rx, float ry,
                         int cp, chtype ch, int rows, int cols)
{
    int n = (int)(2.0f * (float)M_PI * (rx + ry) * 0.75f) + 32;
    if (n > 512) n = 512;
    for (int i = 0; i < n; i++) {
        float t = 2.0f * (float)M_PI * (float)i / (float)n;
        int x = (int)(cx + rx * cosf(t) + 0.5f);
        int y = (int)(cy + ry * sinf(t) + 0.5f);
        if (x>=0 && x<cols && y>=0 && y<rows) {
            attron(COLOR_PAIR(cp));
            mvaddch(y, x, ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* ===================================================================== */
/* S5  triangulation (Bowyer-Watson)                                      */
/* ===================================================================== */

static Pt  g_pts[MAX_PTS];
static Tri g_tris[MAX_TRIS];
static int g_npts;
static int g_ntris;

/*
 * init_super() -- create the super-triangle that contains all future points.
 *
 * Vertices are placed well outside [0,cols] x [0,rows] in cell space.
 * Indices 0, 1, 2 are always the super-triangle; real points start at 3.
 */
static void init_super(int cols, int rows)
{
    float W = (float)cols, H = (float)rows;
    g_pts[0] = (Pt){  W * 0.5f,     -H * 3.0f };
    g_pts[1] = (Pt){ -W * 3.0f,      H * 4.0f };
    g_pts[2] = (Pt){  W * 4.0f,      H * 4.0f };
    g_tris[0] = (Tri){ 0, 1, 2 };
    g_npts  = SUPER_COUNT;
    g_ntris = 1;
}

/* Pre-fill N_POINTS random positions into g_pts[3..3+N_POINTS-1]. */
static void gen_points(int cols, int rows, unsigned seed)
{
    srand(seed);
    int xlo = MARGIN_X,          xhi = cols - MARGIN_X;
    int ylo = MARGIN_Y + 2,      yhi = rows - HUD_ROWS - MARGIN_Y;
    if (xhi <= xlo) xhi = xlo + 1;
    if (yhi <= ylo) yhi = ylo + 1;
    for (int i = 0; i < N_POINTS; i++) {
        g_pts[SUPER_COUNT + i].x = (float)(xlo + rand() % (xhi - xlo));
        g_pts[SUPER_COUNT + i].y = (float)(ylo + rand() % (yhi - ylo));
    }
}

/*
 * insert_point() -- Bowyer-Watson incremental step.
 *
 * g_pts[idx] must already be filled in.  Adds it to the triangulation:
 *   1. Mark every triangle whose circumcircle (geo space) contains idx.
 *   2. Collect boundary edges of the "hole" -- edges belonging to exactly
 *      one bad triangle.
 *   3. Compact away bad triangles.
 *   4. Connect idx to every boundary edge to fill the hole.
 */
static void insert_point(int idx)
{
    float px = g_pts[idx].x;
    float py = gy(g_pts[idx].y);

    /* Step 1 -- find bad triangles */
    static bool bad[MAX_TRIS];
    memset(bad, 0, (size_t)g_ntris * sizeof(bool));

    for (int i = 0; i < g_ntris; i++) {
        float ax = g_pts[g_tris[i].a].x,  ay = gy(g_pts[g_tris[i].a].y);
        float bx = g_pts[g_tris[i].b].x,  by = gy(g_pts[g_tris[i].b].y);
        float cx = g_pts[g_tris[i].c].x,  cy = gy(g_pts[g_tris[i].c].y);
        if (in_circumcircle(ax, ay, bx, by, cx, cy, px, py))
            bad[i] = true;
    }

    /* Step 2 -- collect hole boundary edges */
    static HEdge hole[MAX_HOLE];
    int nhole = 0;

    for (int i = 0; i < g_ntris; i++) {
        if (!bad[i]) continue;
        int vs[3] = { g_tris[i].a, g_tris[i].b, g_tris[i].c };
        for (int e = 0; e < 3; e++) {
            int va = vs[e], vb = vs[(e+1)%3];
            bool shared = false;
            for (int j = 0; j < g_ntris && !shared; j++) {
                if (j == i || !bad[j]) continue;
                int ws[3] = { g_tris[j].a, g_tris[j].b, g_tris[j].c };
                for (int f = 0; f < 3; f++) {
                    int wa = ws[f], wb = ws[(f+1)%3];
                    if ((va==wa && vb==wb) || (va==wb && vb==wa)) {
                        shared = true; break;
                    }
                }
            }
            if (!shared && nhole < MAX_HOLE)
                hole[nhole++] = (HEdge){ va, vb };
        }
    }

    /* Step 3 -- remove bad triangles */
    int w = 0;
    for (int i = 0; i < g_ntris; i++)
        if (!bad[i]) g_tris[w++] = g_tris[i];
    g_ntris = w;

    /* Step 4 -- fill hole */
    for (int e = 0; e < nhole && g_ntris < MAX_TRIS; e++)
        g_tris[g_ntris++] = (Tri){ hole[e].a, hole[e].b, idx };
}

/*
 * update_mesh() -- build triangulation from scratch (used on reset).
 * During animation, insert_point() is called one step at a time instead.
 */
/* update_mesh() builds the full triangulation in one shot (used externally
 * if the caller wants the final mesh without animation). */
static void update_mesh(int cols, int rows, unsigned seed)
    __attribute__((unused));
static void update_mesh(int cols, int rows, unsigned seed)
{
    init_super(cols, rows);
    gen_points(cols, rows, seed);
    g_npts = SUPER_COUNT + N_POINTS;
    for (int i = 0; i < N_POINTS; i++)
        insert_point(SUPER_COUNT + i);
}

/* Count triangles that do not touch super-triangle vertices. */
static int real_tri_count(void)
{
    int n = 0;
    for (int i = 0; i < g_ntris; i++) {
        Tri *t = &g_tris[i];
        if (t->a >= SUPER_COUNT && t->b >= SUPER_COUNT && t->c >= SUPER_COUNT)
            n++;
    }
    return n;
}

/* ===================================================================== */
/* S6  scene                                                              */
/* ===================================================================== */

typedef enum { PHASE_INSERT, PHASE_SHOWCASE, PHASE_DONE } Phase;

/* Indices into g_tris for "real" triangles (no super-triangle vertices). */
static int  g_rtris[MAX_TRIS];
static int  g_nrtris;

static void collect_real_tris(void)
{
    g_nrtris = 0;
    for (int i = 0; i < g_ntris; i++) {
        Tri *t = &g_tris[i];
        if (t->a >= SUPER_COUNT && t->b >= SUPER_COUNT && t->c >= SUPER_COUNT)
            g_rtris[g_nrtris++] = i;
    }
}

typedef struct {
    Phase    phase;
    int      insert_idx;     /* next point to insert: SUPER_COUNT .. SUPER_COUNT+N_POINTS-1 */
    int      showcase_idx;   /* index into g_rtris[]  */
    int      showcase_ticks;
    int      showcase_pass;
    int      done_ticks;
    unsigned seed;
    int      cols, rows;
    bool     paused;
    bool     step_req;
} Scene;

static void scene_start_insert(Scene *sc)
{
    sc->phase           = PHASE_INSERT;
    sc->insert_idx      = SUPER_COUNT;
    sc->showcase_idx    = 0;
    sc->showcase_ticks  = 0;
    sc->showcase_pass   = 0;
    sc->done_ticks      = 0;
    init_super(sc->cols, sc->rows);
    gen_points(sc->cols, sc->rows, sc->seed);
    g_nrtris = 0;
}

static void scene_init(Scene *sc, int cols, int rows)
{
    sc->cols   = cols;
    sc->rows   = rows;
    sc->seed   = (unsigned)time(NULL);
    sc->paused = false;
    scene_start_insert(sc);
}

static void scene_tick(Scene *sc)
{
    if (sc->paused && !sc->step_req) return;
    sc->step_req = false;

    switch (sc->phase) {

    case PHASE_INSERT:
        if (sc->insert_idx < SUPER_COUNT + N_POINTS) {
            /* Reveal and insert one point */
            g_npts = sc->insert_idx + 1;
            insert_point(sc->insert_idx);
            sc->insert_idx++;
        } else {
            collect_real_tris();
            sc->phase       = PHASE_SHOWCASE;
            sc->showcase_idx = 0;
        }
        break;

    case PHASE_SHOWCASE:
        sc->showcase_ticks++;
        if (sc->showcase_ticks >= SHOWCASE_HOLD) {
            sc->showcase_ticks = 0;
            sc->showcase_idx++;
            if (sc->showcase_idx >= g_nrtris) {
                sc->showcase_pass++;
                sc->showcase_idx = 0;
                if (sc->showcase_pass >= SHOWCASE_CYCLES) {
                    sc->phase      = PHASE_DONE;
                    sc->done_ticks = 0;
                }
            }
        }
        break;

    case PHASE_DONE:
        sc->done_ticks++;
        if (sc->done_ticks >= DONE_TICKS) {
            sc->seed += 7919;
            scene_start_insert(sc);
        }
        break;
    }
}

/* ===================================================================== */
/* S7  render                                                             */
/* ===================================================================== */

/*
 * render_scene() -- draw the mesh, and in SHOWCASE the highlighted triangle
 * and its circumcircle.
 *
 * Draw order (back to front):
 *   1. All real triangle edges      -- dim blue dots
 *   2. Highlighted triangle edges   -- bright yellow lines
 *   3. Circumcircle of highlighted  -- magenta ellipse
 *   4. Circumcenter                 -- magenta '+'
 *   5. All inserted real points     -- white '@'
 *   6. Most-recently inserted point -- yellow '@' (INSERT phase)
 *   7. Points inside circumcircle   -- red '@' (should always be 0)
 */
static void render_scene(const Scene *sc, int rows, int cols)
{
    /* Layer 1: normal edges */
    for (int i = 0; i < g_ntris; i++) {
        Tri *t = &g_tris[i];
        if (t->a < SUPER_COUNT || t->b < SUPER_COUNT || t->c < SUPER_COUNT)
            continue;
        int ax=(int)(g_pts[t->a].x+.5f), ay=(int)(g_pts[t->a].y+.5f);
        int bx=(int)(g_pts[t->b].x+.5f), by=(int)(g_pts[t->b].y+.5f);
        int cx=(int)(g_pts[t->c].x+.5f), cy=(int)(g_pts[t->c].y+.5f);
        draw_line(ax,ay, bx,by, CP_EDGE, '.', rows,cols);
        draw_line(bx,by, cx,cy, CP_EDGE, '.', rows,cols);
        draw_line(cx,cy, ax,ay, CP_EDGE, '.', rows,cols);
    }

    /* Layers 2-4: showcase circumcircle */
    if (sc->phase == PHASE_SHOWCASE && g_nrtris > 0) {
        int ti = g_rtris[sc->showcase_idx % g_nrtris];
        Tri *t = &g_tris[ti];

        int ax=(int)(g_pts[t->a].x+.5f), ay=(int)(g_pts[t->a].y+.5f);
        int bx=(int)(g_pts[t->b].x+.5f), by=(int)(g_pts[t->b].y+.5f);
        int cx=(int)(g_pts[t->c].x+.5f), cy=(int)(g_pts[t->c].y+.5f);

        /* Layer 2: highlighted triangle edges */
        draw_line(ax,ay, bx,by, CP_EDGE_HI, '+', rows,cols);
        draw_line(bx,by, cx,cy, CP_EDGE_HI, '+', rows,cols);
        draw_line(cx,cy, ax,ay, CP_EDGE_HI, '+', rows,cols);

        /* Layer 3+4: circumcircle */
        float ga = g_pts[t->a].x,  gA = gy(g_pts[t->a].y);
        float gb = g_pts[t->b].x,  gB = gy(g_pts[t->b].y);
        float gc = g_pts[t->c].x,  gC = gy(g_pts[t->c].y);
        float ocx, ocy, r2;
        if (circumcircle_geo(ga,gA, gb,gB, gc,gC, &ocx,&ocy,&r2)) {
            float r   = sqrtf(r2);
            float dcx = ocx;           /* geo x == cell x */
            float dcy = ocy / ASPECT_Y;
            float rx  = r;
            float ry  = r / ASPECT_Y;
            draw_ellipse(dcx, dcy, rx, ry, CP_CIRC, 'o', rows, cols);
            int ex=(int)(dcx+.5f), ey=(int)(dcy+.5f);
            if (ex>=0&&ex<cols&&ey>=0&&ey<rows) {
                attron(COLOR_PAIR(CP_CIRC_CTR)|A_BOLD);
                mvaddch(ey, ex, '+');
                attroff(COLOR_PAIR(CP_CIRC_CTR)|A_BOLD);
            }
        }
    }

    /* Layer 5: all inserted real points */
    int newest = sc->insert_idx - 1;
    for (int i = SUPER_COUNT; i < g_npts; i++) {
        int x=(int)(g_pts[i].x+.5f), y=(int)(g_pts[i].y+.5f);
        if (x<0||x>=cols||y<0||y>=rows) continue;
        int cp = (i == newest && sc->phase == PHASE_INSERT)
               ? CP_POINT_NEW : CP_POINT;
        attron(COLOR_PAIR(cp)|A_BOLD);
        mvaddch(y, x, '@');
        attroff(COLOR_PAIR(cp)|A_BOLD);
    }

    /* Layer 6: points inside current circumcircle (should always be 0) */
    if (sc->phase == PHASE_SHOWCASE && g_nrtris > 0) {
        int ti = g_rtris[sc->showcase_idx % g_nrtris];
        Tri *t = &g_tris[ti];
        float ga=g_pts[t->a].x, gA=gy(g_pts[t->a].y);
        float gb=g_pts[t->b].x, gB=gy(g_pts[t->b].y);
        float gc=g_pts[t->c].x, gC=gy(g_pts[t->c].y);
        float ocx, ocy, r2;
        if (circumcircle_geo(ga,gA, gb,gB, gc,gC, &ocx,&ocy,&r2)) {
            for (int i = SUPER_COUNT; i < g_npts; i++) {
                if (i==t->a||i==t->b||i==t->c) continue;
                float px=g_pts[i].x, py=gy(g_pts[i].y);
                float dx=px-ocx, dy=py-ocy;
                if (dx*dx+dy*dy < r2-1e-4f) {
                    int x=(int)(g_pts[i].x+.5f), y=(int)(g_pts[i].y+.5f);
                    if (x>=0&&x<cols&&y>=0&&y<rows) {
                        attron(COLOR_PAIR(CP_POINT_IN)|A_BOLD);
                        mvaddch(y, x, 'X');
                        attroff(COLOR_PAIR(CP_POINT_IN)|A_BOLD);
                    }
                }
            }
        }
    }
}

/*
 * render_overlay() -- bottom HUD.
 *
 * Shows: triangle/point counts, phase, showcase circumcircle stats,
 * empty-circumcircle verification, and two explanation lines.
 */
static void render_overlay(const Scene *sc, int rows, int cols)
{
    int hud = rows - HUD_ROWS;

    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(hud, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));

    /* Line 1: counts */
    const char *phase_str = (sc->phase==PHASE_INSERT)   ? "INSERTING" :
                            (sc->phase==PHASE_SHOWCASE)  ? "SHOWCASE " :
                                                           "RESTARTING";
    attron(COLOR_PAIR(CP_HUD)|A_BOLD);
    mvprintw(hud+1, 1,
        " triangles: %3d   points: %3d/%d   phase: %s",
        real_tri_count(),
        sc->insert_idx - SUPER_COUNT, N_POINTS,
        phase_str);
    attroff(COLOR_PAIR(CP_HUD)|A_BOLD);

    /* Line 2: showcase detail or insertion progress */
    if (sc->phase == PHASE_SHOWCASE && g_nrtris > 0) {
        int ti = g_rtris[sc->showcase_idx % g_nrtris];
        Tri *t = &g_tris[ti];
        float ga=g_pts[t->a].x, gA=gy(g_pts[t->a].y);
        float gb=g_pts[t->b].x, gB=gy(g_pts[t->b].y);
        float gc=g_pts[t->c].x, gC=gy(g_pts[t->c].y);
        float ocx, ocy, r2;
        bool ok = circumcircle_geo(ga,gA, gb,gB, gc,gC, &ocx,&ocy,&r2);
        float r  = ok ? sqrtf(r2) : 0.0f;

        /* Count points inside */
        int inside = 0;
        if (ok) {
            for (int i = SUPER_COUNT; i < g_npts; i++) {
                if (i==t->a||i==t->b||i==t->c) continue;
                float px=g_pts[i].x, py=gy(g_pts[i].y);
                float dx=px-ocx, dy=py-ocy;
                if (dx*dx+dy*dy < r2-1e-4f) inside++;
            }
        }

        attron(COLOR_PAIR(CP_HUD));
        mvprintw(hud+2, 1,
            " tri %d/%d   circ_radius: %.1f cells   center: (%.1f, %.1f)",
            sc->showcase_idx % g_nrtris + 1, g_nrtris,
            (double)r, (double)ocx, (double)(ocy/ASPECT_Y));
        attroff(COLOR_PAIR(CP_HUD));

        int cp_res = (inside == 0) ? CP_OK : CP_WARN;
        attron(COLOR_PAIR(cp_res)|A_BOLD);
        mvprintw(hud+3, 1,
            " empty circumcircle: %s   (%d point%s inside  --  %s)",
            inside==0 ? "YES" : "NO!",
            inside, inside==1?"":"s",
            inside==0 ? "Delaunay property holds" : "BUG: property violated");
        attroff(COLOR_PAIR(cp_res)|A_BOLD);

    } else {
        attron(COLOR_PAIR(CP_LABEL));
        mvprintw(hud+2, 1,
            " inserting point %d/%d -- each insertion repairs circumcircle property",
            sc->insert_idx - SUPER_COUNT, N_POINTS);
        mvprintw(hud+3, 1, " '@'=point  '.'=edge  'o'=circumcircle  '+'=circumcenter  [+]=highlighted tri");
        attroff(COLOR_PAIR(CP_LABEL));
    }

    /* Lines 4-5: explanation */
    attron(COLOR_PAIR(CP_EXPLAIN));
    mvprintw(hud+4, 1,
        " Empty circumcircle: no point lies inside any triangle's circumcircle."
        "  This uniquely defines Delaunay.");
    mvprintw(hud+5, 1,
        " Mesh quality: Delaunay maximises the minimum angle across all triangles,"
        " avoiding thin slivers.");
    mvprintw(hud+6, 1,
        " Why it matters: FEM solvers, terrain meshing, interpolation -- all need"
        " well-shaped triangles for accuracy.");
    attroff(COLOR_PAIR(CP_EXPLAIN));

    /* Controls */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(hud+7, 1,
        " SPACE pause   s step   r reset   +/- speed   q quit");
    attroff(COLOR_PAIR(CP_LABEL));
}

static void render_header(const Scene *sc, int cols)
{
    (void)sc;
    attron(COLOR_PAIR(CP_HEADER)|A_BOLD);
    mvprintw(0, 0,
        " Delaunay Triangulation  [Bowyer-Watson incremental]"
        "   N=%d points   aspect-corrected circumcircles", N_POINTS);
    attroff(COLOR_PAIR(CP_HEADER)|A_BOLD);
    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(1, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));
}

/* ===================================================================== */
/* S8  screen                                                             */
/* ===================================================================== */

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

static void screen_draw(const Screen *s, const Scene *sc)
{
    erase();
    render_header(sc, s->cols);
    render_scene(sc, s->rows, s->cols);
    render_overlay(sc, s->rows, s->cols);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* S9  app                                                                */
/* ===================================================================== */

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
        app->scene.seed += 9973;
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

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_init(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time       = clock_ns();
            sim_accum        = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count /
                          ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }
        (void)fps_display;

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        screen_draw(&app->screen, &app->scene);

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }
    return 0;
}
