/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 11_delaunay.c — Delaunay triangulation of random points (Bowyer-Watson)
 *
 * DEMO: N random points are scattered across the screen and triangulated
 *       with the Bowyer-Watson incremental algorithm. The cursor selects
 *       one input point and highlights every triangle incident to it;
 *       ',' / '.' cycle the cursor through the points list. Press 'r' to
 *       reseed with new random points.
 *
 * Study alongside: 01_equilateral.c — regular triangle tiling. This file
 *                  is the IRREGULAR counterpart: any point cloud can be
 *                  turned into a triangulation that maximises the minimum
 *                  interior angle.
 *                  geometry/delaunay_triangulation.c — fuller reference
 *                  with circumcircle visualisation and step-by-step
 *                  insertion.
 *                  ../README.md — GridCtx primitive (this file builds a
 *                  mesh from a point set).
 *
 * Section map:
 *   §1 config   — N points, screen margins, EWMA constant
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: edge / point / cursor / HUD / hint
 *   §4 formula  — GridCtx + ctx_init + orient2d + circumcircle predicate
 *   §5 mesh     — Bowyer-Watson insertion + super-triangle cleanup
 *   §5b cursor  — Cursor { int point_idx } + cursor_advance / cursor_draw
 *   §5c draw    — slope_char + Bresenham line_draw
 *   §6 scene    — hud_draw + scene_draw (seed_random + render)
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  ,/. cursor   r reseed   t theme   p pause
 *        +/- N points   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/11_delaunay.c \
 *       -o 11_delaunay -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Bowyer-Watson (1981). Incremental Delaunay:
 *                    1. Start with a "super-triangle" containing every
 *                       input point.
 *                    2. For each input point P:
 *                       a. Find every triangle whose circumcircle
 *                          contains P ("bad" triangles).
 *                       b. The bad triangles form a star-shaped polygon
 *                          (a "hole"); delete them.
 *                       c. Connect P to every boundary edge of the hole.
 *                    3. Remove every triangle that still touches a super-
 *                       triangle vertex; the rest is the Delaunay
 *                       triangulation of the actual input points.
 *
 * Data-structure : GridCtx carries screen extent and the input point
 *                  count. A separate static mesh (g_pts, g_tris) holds
 *                  the actual points and triangles — building a Delaunay
 *                  mesh requires global state (the in-circle predicate
 *                  consults all triangles), which is the documented
 *                  exception to "no malloc in hot path" (here static
 *                  arrays, sized at compile time).
 *
 * Formula        : Empty-circumcircle predicate (in_circumcircle):
 *                    For triangle ABC (CCW) and point P:
 *                      | A.x−P.x  A.y−P.y  (A.x²+A.y²−P.x²−P.y²) |
 *                      | B.x−P.x  B.y−P.y  (B.x²+B.y²−P.x²−P.y²) | > 0
 *                      | C.x−P.x  C.y−P.y  (C.x²+C.y²−P.x²−P.y²) |
 *                    holds iff P is strictly inside the circumcircle of ABC.
 *                  Orientation predicate (orient2d):
 *                    sgn((B−A) × (C−A)) — positive ⇔ CCW.
 *
 * Edge chars     : Each leaf triangle draws its 3 edges via Bresenham.
 *                  Slope→character classification (same as 07-10).
 *                  Points draw as '*' in PAIR_POINT.
 *
 * Movement       : ',' / '.' cycle the cursor through the input points
 *                  list (N_SUPER..g_n_pts-1). The selected point is drawn
 *                  with '@' in PAIR_CURSOR; every triangle that contains
 *                  the selected point as a vertex is highlighted bold.
 *                  'r' reseeds with new random points.
 *
 * References     :
 *   Bowyer, "Computing Dirichlet Tessellations" (1981)
 *   Watson, "Computing the n-dimensional Delaunay tessellation..." (1981)
 *   de Berg et al., "Computational Geometry" (3e), §9
 *   InCircle predicate — Shewchuk, "Robust Adaptive Floating-Point
 *                        Geometric Predicates" (1996)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Given a scattered set of points, draw triangle edges between them so
 * that no triangle has any other point inside its circumscribed circle.
 * That single property — "empty circumcircle" — uniquely determines the
 * triangulation (up to ties on cocircular point sets), and equivalently
 * MAXIMISES the minimum interior angle across the whole mesh. Thin
 * sliver triangles are punished; well-shaped triangles are rewarded.
 *
 * Bowyer-Watson achieves this incrementally. Start with a giant triangle
 * that contains every input point. Insert points one by one. After each
 * insertion, repair any triangles that violate the empty-circumcircle
 * property by deleting them and re-triangulating the resulting hole
 * around the new point.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine each triangle has a circle drawn through its three vertices —
 * the CIRCUMCIRCLE. A triangulation is "Delaunay" if every triangle's
 * circumcircle has no points inside it (no other input points, that is).
 * When you add a new point P, some existing circumcircles will end up
 * containing P. Those triangles are "bad" and must die. Together they
 * form a star-shaped polygon (a hole). Re-fill the hole by drawing
 * edges from P to each boundary edge of the hole — voila, P is part of
 * the new Delaunay triangulation.
 *
 * The "super-triangle" is just a scaffold: a giant triangle containing
 * everything. Every real point sits inside it, so the algorithm always
 * has SOME triangle to work with. After insertion is done, we delete
 * every triangle that still touches a super-triangle vertex.
 *
 * DRAWING METHOD  (one-shot pipeline)
 * ──────────────
 *  1. Pick N points; reseed at random within screen margins.
 *  2. Install super-triangle (3 vertices, 1 triangle).
 *  3. For each input point P:  mesh_insert(P).
 *  4. mesh_strip_super(): mark every triangle touching a super vertex as
 *     invalid.
 *  5. Draw every valid triangle's 3 edges with Bresenham + slope_char.
 *     Bold-highlight every triangle incident to the cursor point.
 *  6. Draw every input point as '*'; the cursor point as '@'.
 *  7. ',' / '.' move the cursor index through the input points.
 *
 * KEY FORMULAS
 * ────────────
 *  Signed area (× 2) — also called orient2d:
 *    orient2d(A, B, C) = (B.x − A.x)·(C.y − A.y) − (B.y − A.y)·(C.x − A.x)
 *    sign +  ⇒ CCW
 *    sign −  ⇒ CW
 *    sign 0  ⇒ collinear
 *
 *  In-circle predicate:
 *    For triangle ABC (CCW) and point P:
 *      let ax = A.x − P.x, ay = A.y − P.y, a² = ax² + ay²
 *      similarly bx, by, b², cx, cy, c²
 *      det = a² (bx·cy − cx·by) − b² (ax·cy − cx·ay) + c² (ax·by − bx·ay)
 *      det > 0  ⇔  P inside circumcircle of ABC
 *
 *  Boundary edge of the hole: edge (a, b) shared by exactly one bad
 *  triangle (i.e., not also an edge of another bad triangle).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The super-triangle MUST contain every input point or insertion fails.
 *    We make it 50× the larger screen dimension — way larger than any
 *    real input could need.
 *  • Co-circular points (4+ points on the same circle) leave the empty-
 *    circumcircle predicate ambiguous. Bowyer-Watson breaks ties
 *    arbitrarily, which is correct — both possible triangulations are
 *    valid Delaunay.
 *  • Floating-point precision: the in_circumcircle determinant can be
 *    tiny near borderline cases. We use an epsilon of 1e-9. For exact
 *    robustness, Shewchuk's adaptive predicates would be the upgrade.
 *  • CCW orientation must hold for the in-circle test. We force CCW by
 *    swapping vertices in tri_add() when orient2d is negative.
 *  • N=80 gives ~150 triangles — interactive at 60 fps.
 *  • Screen resize: re-seed entirely (the random points need to be
 *    re-placed within the new bounds anyway).
 *
 * HOW TO VERIFY
 * ─────────────
 *  Visual check: NO triangle should look "needle thin" or have a tiny
 *  angle. The Delaunay property guarantees the smallest angle is as
 *  large as possible.
 *
 *  Programmatic check: for each triangle (A, B, C), iterate over all
 *  other input points P and confirm in_circumcircle(A, B, C, P) == 0.
 *  (Not done in this demo, but easy to add.)
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define N_POINTS_DEFAULT 30
#define N_POINTS_MIN      6
#define N_POINTS_MAX     80

/* Super-triangle — 100× larger than the screen so every input lands inside.
 * Indices 0..2 in the points array are reserved for the super vertices. */
#define N_SUPER          3
#define MAX_POINTS      (N_POINTS_MAX + N_SUPER)
#define MAX_TRIS        (MAX_POINTS * 4)
#define MAX_BOUNDARY    (MAX_TRIS * 3)

#define MARGIN_FRAC      0.08
#define N_THEMES         3

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA 0.05

#define PAIR_EDGE   1
#define PAIR_POINT  2
#define PAIR_CURSOR 3
#define PAIR_HUD    4
#define PAIR_HINT   5

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES][2] = {
    /* edge,  point */
    {  75, 226 },
    {  82, 196 },
    {  15,  39 },
};
static const short THEME_FG_8[N_THEMES][2] = {
    { COLOR_CYAN,  COLOR_YELLOW },
    { COLOR_GREEN, COLOR_RED    },
    { COLOR_WHITE, COLOR_CYAN   },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg_e, fg_p;
    if (COLORS >= 256) { fg_e = THEME_FG[theme][0];   fg_p = THEME_FG[theme][1];   }
    else               { fg_e = THEME_FG_8[theme][0]; fg_p = THEME_FG_8[theme][1]; }
    init_pair(PAIR_EDGE,   fg_e, -1);
    init_pair(PAIR_POINT,  fg_p, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx + orient2d + in_circumcircle                      */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { double x, y; } Pt;
typedef struct { int v[3]; int valid; } Tri;

/*
 * GridCtx — geometry + scene parameters for the Delaunay mesh.
 *
 * Mirrors the canonical GridCtx (rect_grids/01_uniform_rect.c) but
 * carries the Delaunay-specific n_pts. The persistent mesh itself
 * (g_pts, g_tris) lives in §5 statics — keeping that out of GridCtx
 * preserves GridCtx as a thin geometry descriptor.
 */
typedef struct {
    int rows, cols;
    int cw, ch;
    int ox, oy;          /* screen origin (centre) — used by some helpers */
    int n_pts;           /* input point count (excludes the 3 super verts) */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols, int n_pts)
{
    g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = (cols * CELL_W) / 2;
    g->oy = ((rows - 1) * CELL_H) / 2;
    g->n_pts = n_pts;
}

/*
 * orient2d — twice the signed area of triangle ABC. Positive ⇔ CCW.
 *
 * THE FORMULA:
 *    (B.x − A.x)·(C.y − A.y) − (B.y − A.y)·(C.x − A.x)
 */
static double orient2d(Pt A, Pt B, Pt C)
{
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

/*
 * in_circumcircle — non-zero if P is strictly inside the circumcircle
 * of triangle ABC. Assumes ABC is CCW (caller responsibility).
 *
 * THE FORMULA (3×3 determinant):
 *
 *   det = | A.x−P.x  A.y−P.y  (A.x²+A.y²−P.x²−P.y²) |
 *         | B.x−P.x  B.y−P.y  (B.x²+B.y²−P.x²−P.y²) |
 *         | C.x−P.x  C.y−P.y  (C.x²+C.y²−P.x²−P.y²) |
 *
 *   det > 0  ⇔  P inside circumcircle of CCW triangle ABC
 */
static int in_circumcircle(Pt A, Pt B, Pt C, Pt P)
{
    double ax = A.x - P.x, ay = A.y - P.y;
    double bx = B.x - P.x, by = B.y - P.y;
    double cx = C.x - P.x, cy = C.y - P.y;
    double a2 = ax * ax + ay * ay;
    double b2 = bx * bx + by * by;
    double c2 = cx * cx + cy * cy;
    double det = a2 * (bx * cy - cx * by)
               - b2 * (ax * cy - cx * ay)
               + c2 * (ax * by - bx * ay);
    return det > 1e-9;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  mesh — Bowyer-Watson                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

static Pt   g_pts[MAX_POINTS];
static int  g_n_pts;
static Tri  g_tris[MAX_TRIS];
static int  g_n_tris;

static void mesh_clear(void) { g_n_pts = 0; g_n_tris = 0; }

/*
 * tri_add — append a triangle, force CCW orientation by swapping if needed.
 * The in_circumcircle predicate requires CCW.
 */
static void tri_add(int v0, int v1, int v2)
{
    if (g_n_tris >= MAX_TRIS) return;
    if (orient2d(g_pts[v0], g_pts[v1], g_pts[v2]) < 0.0) {
        int tmp = v1; v1 = v2; v2 = tmp;
    }
    g_tris[g_n_tris++] = (Tri){ {v0, v1, v2}, 1 };
}

/*
 * mesh_seed_super — install a giant super-triangle as points 0..2 and the
 * only triangle. The super-triangle contains every screen pixel.
 */
static void mesh_seed_super(double pw, double ph)
{
    double D = (pw > ph ? pw : ph) * 50.0;
    g_pts[0] = (Pt){ pw * 0.5,         ph * 0.5 - D };
    g_pts[1] = (Pt){ pw * 0.5 - D,     ph * 0.5 + D };
    g_pts[2] = (Pt){ pw * 0.5 + D,     ph * 0.5 + D };
    g_n_pts  = N_SUPER;
    g_n_tris = 0;
    tri_add(0, 1, 2);
}

/*
 * mesh_insert — Bowyer-Watson incremental insertion of a single point.
 *
 * Steps:
 *   1. Append P to the points array.
 *   2. Mark every triangle whose circumcircle contains P as bad
 *      (set valid=0; record index in bad[]).
 *   3. Walk bad triangles' edges; an edge is on the boundary if it
 *      appears in EXACTLY ONE bad triangle (not shared with another
 *      bad one).
 *   4. Spawn a new triangle for each boundary edge, connecting it to P.
 */
static void mesh_insert(Pt P)
{
    if (g_n_pts >= MAX_POINTS) return;
    g_pts[g_n_pts] = P;
    int pid = g_n_pts;
    g_n_pts++;

    int bad[MAX_TRIS];
    int n_bad = 0;
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        Pt A = g_pts[g_tris[i].v[0]];
        Pt B = g_pts[g_tris[i].v[1]];
        Pt C = g_pts[g_tris[i].v[2]];
        if (in_circumcircle(A, B, C, P)) {
            g_tris[i].valid = 0;
            bad[n_bad++] = i;
        }
    }

    int boundary[MAX_BOUNDARY][2];
    int n_b = 0;
    for (int bi = 0; bi < n_bad; bi++) {
        int ti = bad[bi];
        for (int e = 0; e < 3; e++) {
            int a = g_tris[ti].v[e];
            int b = g_tris[ti].v[(e + 1) % 3];
            int shared = 0;
            for (int bj = 0; bj < n_bad; bj++) {
                if (bj == bi) continue;
                int tj = bad[bj];
                for (int f = 0; f < 3; f++) {
                    int aa = g_tris[tj].v[f];
                    int bb = g_tris[tj].v[(f + 1) % 3];
                    if ((aa == a && bb == b) || (aa == b && bb == a)) {
                        shared = 1; break;
                    }
                }
                if (shared) break;
            }
            if (!shared && n_b < MAX_BOUNDARY) {
                boundary[n_b][0] = a;
                boundary[n_b][1] = b;
                n_b++;
            }
        }
    }

    for (int i = 0; i < n_b; i++)
        tri_add(boundary[i][0], boundary[i][1], pid);
}

/*
 * mesh_strip_super — invalidate every triangle that touches a super-tri
 * vertex (index < N_SUPER).
 */
static void mesh_strip_super(void)
{
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        for (int j = 0; j < 3; j++) {
            if (g_tris[i].v[j] < N_SUPER) { g_tris[i].valid = 0; break; }
        }
    }
}

static int count_valid_tris(void)
{
    int n = 0;
    for (int i = 0; i < g_n_tris; i++) if (g_tris[i].valid) n++;
    return n;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5b  cursor — selection index into the input points list                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — index into g_pts[N_SUPER..g_n_pts-1].
 *
 * Aperiodic / random meshes have no natural "neighbour" relation in cell
 * space, so we cycle through the points list with ',' / '.' instead of
 * arrow-key cell-stepping. The selected point is drawn '@' and every
 * triangle incident to it is bold-highlighted.
 */
typedef struct { int point_idx; } Cursor;

static void cursor_reset(Cursor *cur)
{
    cur->point_idx = (g_n_pts > N_SUPER) ? N_SUPER : -1;
}

static void cursor_advance(Cursor *cur, int dir)
{
    int n_input = g_n_pts - N_SUPER;
    if (n_input <= 0) { cur->point_idx = -1; return; }
    int rel = cur->point_idx - N_SUPER;
    rel = (rel + dir + n_input) % n_input;
    cur->point_idx = N_SUPER + rel;
}

/* True if triangle ti has the cursor point as one of its vertices. */
static int tri_has_point(int ti, int point_idx)
{
    if (point_idx < 0) return 0;
    return g_tris[ti].v[0] == point_idx
        || g_tris[ti].v[1] == point_idx
        || g_tris[ti].v[2] == point_idx;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5c draw — slope_char + Bresenham line                                  */
/* ═══════════════════════════════════════════════════════════════════════ */

static char slope_char(double dx, double dy)
{
    double ax = fabs(dx) * (1.0 / CELL_W);
    double ay = fabs(dy) * (1.0 / CELL_H);
    double t  = atan2(ay, ax);
    if (t < M_PI / 8.0)         return '-';
    if (t > 3.0 * M_PI / 8.0)   return '|';
    return ((dx >= 0) == (dy >= 0)) ? '\\' : '/';
}

static void line_draw(const GridCtx *g, double px0, double py0,
                      double px1, double py1, int attr)
{
    char ch = slope_char(px1 - px0, py1 - py0);
    int x0 = (int)(px0 / CELL_W), y0 = (int)(py0 / CELL_H);
    int x1 = (int)(px1 / CELL_W), y1 = (int)(py1 / CELL_H);
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    attron(attr);
    for (;;) {
        if (x0 >= 0 && x0 < g->cols && y0 >= 0 && y0 < g->rows - 1)
            mvaddch(y0, x0, (chtype)(unsigned char)ch);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    attroff(attr);
}

/*
 * ctx_draw_bg — render every valid mesh edge.
 *
 * Triangles incident to the cursor point are drawn with PAIR_CURSOR + bold;
 * the rest with PAIR_EDGE.
 */
static void ctx_draw_bg(const GridCtx *g, const Cursor *cur)
{
    for (int i = 0; i < g_n_tris; i++) {
        if (!g_tris[i].valid) continue;
        int hot  = tri_has_point(i, cur->point_idx);
        int attr = hot ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                       : COLOR_PAIR(PAIR_EDGE);
        Pt A = g_pts[g_tris[i].v[0]];
        Pt B = g_pts[g_tris[i].v[1]];
        Pt C = g_pts[g_tris[i].v[2]];
        line_draw(g, A.x, A.y, B.x, B.y, attr);
        line_draw(g, B.x, B.y, C.x, C.y, attr);
        line_draw(g, C.x, C.y, A.x, A.y, attr);
    }
}

static void cursor_draw(const GridCtx *g, const Cursor *cur)
{
    /* All input points as '*' */
    attron(COLOR_PAIR(PAIR_POINT) | A_BOLD);
    for (int i = N_SUPER; i < g_n_pts; i++) {
        int col = (int)(g_pts[i].x / CELL_W);
        int row = (int)(g_pts[i].y / CELL_H);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1)
            mvaddch(row, col, '*');
    }
    attroff(COLOR_PAIR(PAIR_POINT) | A_BOLD);

    /* Selected point as '@' on top */
    if (cur->point_idx >= N_SUPER && cur->point_idx < g_n_pts) {
        int col = (int)(g_pts[cur->point_idx].x / CELL_W);
        int row = (int)(g_pts[cur->point_idx].y / CELL_H);
        if (col >= 0 && col < g->cols && row >= 0 && row < g->rows - 1) {
            attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
            mvaddch(row, col, '@');
            attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int          n_pts;
    int          theme;
    int          paused;
    unsigned int seed;
} Scene;

static double frand(unsigned int *s)
{
    *s = *s * 1103515245u + 12345u;
    return ((double)((*s >> 16) & 0x7FFF)) / 32767.0;
}

static void scene_seed_random(Scene *s, Cursor *cur, int rows, int cols)
{
    double pw = (double)cols * CELL_W;
    double ph = (double)(rows - 1) * CELL_H;
    double mx = pw * MARGIN_FRAC;
    double my = ph * MARGIN_FRAC;

    mesh_clear();
    mesh_seed_super(pw, ph);
    for (int i = 0; i < s->n_pts; i++) {
        Pt p = { mx + frand(&s->seed) * (pw - 2 * mx),
                 my + frand(&s->seed) * (ph - 2 * my) };
        mesh_insert(p);
    }
    mesh_strip_super();
    cursor_reset(cur);
}

static void scene_reset(Scene *s, Cursor *cur, int rows, int cols)
{
    s->n_pts  = N_POINTS_DEFAULT;
    s->theme  = 0;
    s->paused = 0;
    s->seed   = (unsigned int)time(NULL);
    scene_seed_random(s, cur, rows, cols);
}

static void hud_draw(const GridCtx *g, const Scene *s, const Cursor *cur, double fps)
{
    char buf[128];
    snprintf(buf, sizeof buf,
             " pts:%d  tris:%d  cursor:%d  theme:%d  %5.1f fps  %s ",
             s->n_pts, count_valid_tris(),
             cur->point_idx >= 0 ? cur->point_idx - N_SUPER : -1,
             s->theme, fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " ,/.:cursor  +/-:N  r:reseed  t:theme  p:pause  q:quit  [11 delaunay] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Scene *s, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g, cur);
    cursor_draw(g, cur);
    hud_draw(g, s, cur, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    Scene  sc  = { 0 };
    Cursor cur = { -1 };
    sc.theme = 0;
    screen_init(sc.theme);

    GridCtx g;
    ctx_init(&g, LINES, COLS, N_POINTS_DEFAULT);
    scene_reset(&sc, &cur, LINES, COLS);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            scene_seed_random(&sc, &cur, LINES, COLS);
        }
        ctx_init(&g, LINES, COLS, sc.n_pts);

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           sc.paused ^= 1; break;
                case 'r':
                    sc.seed = (unsigned int)clock_ns();
                    scene_seed_random(&sc, &cur, g.rows, g.cols);
                    break;
                case 't':
                    sc.theme = (sc.theme + 1) % N_THEMES;
                    color_init(sc.theme);
                    break;
                case ',': case '<': cursor_advance(&cur, -1); break;
                case '.': case '>': cursor_advance(&cur, +1); break;
                case '+': case '=':
                    if (sc.n_pts < N_POINTS_MAX) {
                        sc.n_pts += 2; scene_seed_random(&sc, &cur, g.rows, g.cols);
                    } break;
                case '-':
                    if (sc.n_pts > N_POINTS_MIN) {
                        sc.n_pts -= 2; scene_seed_random(&sc, &cur, g.rows, g.cols);
                    } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;

        scene_draw(&g, &sc, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
