/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * convex_hull.c — Animated Convex Hull Algorithms
 *
 * 40 random points on screen.  Two panels show Graham scan and Jarvis march
 * (gift-wrapping) running simultaneously at the same step rate.
 *
 *   Graham scan:  O(n log n) — sort by polar angle, then stack sweep
 *   Jarvis march: O(n·h)    — repeatedly find most counter-clockwise point
 *
 * Left panel = Graham scan (cyan).  Right panel = Jarvis march (green).
 * Hull edges appear one by one; HUD shows step count and comparison count.
 *
 * Keys: q quit  SPACE new points  p pause  +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/convex_hull.c \
 *       -o convex_hull -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 algorithms  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two classic convex hull algorithms compared side-by-side:
 *
 *                  Graham scan — O(N log N):
 *                    1. Find the lowest-y (rightmost if tie) point as pivot.
 *                    2. Sort remaining points by polar angle from pivot.
 *                    3. Sweep: push points onto a stack; pop when the last
 *                       three points make a clockwise (non-left) turn.
 *                    Cross-product sign determines turn direction:
 *                    (B−A) × (C−A) < 0 → clockwise (not on hull).
 *
 *                  Jarvis march (gift-wrapping) — O(N·h), h=hull points:
 *                    Repeatedly find the most counter-clockwise point from
 *                    the current hull point until we return to the start.
 *                    Faster than Graham scan when h ≪ N (few hull points).
 *
 * Math           : The cross product (A→B) × (A→C) = (Bx−Ax)(Cy−Ay) −
 *                  (By−Ay)(Cx−Ax) determines winding direction and is the
 *                  foundation of all computational geometry primitives.
 *
 * Performance    : Graham scan is O(N log N) due to the sort step.
 *                  The stack sweep is O(N) — each point is pushed/popped ≤ once.
 *                  Jarvis march O(N·h): at worst O(N²) but optimal for small h.
 *
 * References     :
 *   Graham, "An efficient algorithm for determining the convex hull
 *     of a finite planar set," Inf. Proc. Lett. 1 (1972).
 *   Jarvis, "On the identification of the convex hull of a finite
 *     set of points in the plane," Inf. Proc. Lett. 2 (1973).
 *   de Berg et al., "Computational Geometry: Algorithms and
 *     Applications" (3rd ed., 2008) ch. 1.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A convex hull is the smallest convex polygon containing every
 * given point.  Equivalently: stretch a rubber band around all
 * the points and let it snap tight.  The polygon it forms IS
 * the convex hull.  Two algorithms compute it differently:
 * Graham SORTS by angle then sweeps; Jarvis WRAPS by repeatedly
 * picking the next-most-counter-clockwise neighbour.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *   GRAHAM SCAN:  imagine a bottom-anchored radar dish sweeping
 *     anti-clockwise.  As it hits each point in angular order,
 *     decide: does adding this point keep my hull convex?  If
 *     not, BACK UP — pop the previous point off the stack — and
 *     re-check.  After one full sweep the stack holds the hull.
 *
 *   JARVIS MARCH:  imagine wrapping a string around the points.
 *     Start at the lowest point; the next hull point is the one
 *     most COUNTER-CLOCKWISE from your current hull point (the
 *     one that's "leftmost as seen from the current direction").
 *     Repeat until you wrap back to start.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  GRAHAM:
 *   1. Find the LOWEST point (smallest y, leftmost on tie) — it's
 *      definitely on the hull.  Call it P0.
 *   2. Sort all other points by POLAR ANGLE from P0.
 *   3. Initialize stack with P0, P1.
 *   4. For each remaining point P:
 *        while stack has ≥2 points AND turning at the top is NOT
 *              counter-clockwise:
 *          stack.pop()
 *        stack.push(P)
 *   5. Stack contains the hull (in counter-clockwise order).
 *
 *  JARVIS:
 *   1. current = lowest point.  hull = [current].
 *   2. Repeat:
 *        next = current
 *        for each candidate C ≠ current:
 *          if next == current OR (next is to RIGHT of current→C line):
 *            next = C
 *        if next == hull[0]: done
 *        else: hull.push(next); current = next.
 *
 * KEY FORMULA
 * ───────────
 *  Cross product (winding direction):
 *    cross(A, B, C) = (Bx - Ax) · (Cy - Ay) - (By - Ay) · (Cx - Ax)
 *
 *      > 0   →  A → B → C makes a LEFT (counter-clockwise) turn
 *      < 0   →  RIGHT (clockwise) turn — not on hull
 *      = 0   →  collinear (depends on tie-break policy)
 *
 *  This single primitive drives BOTH algorithms; computational
 *  geometry runs on cross-product signs.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Colinear points: cross == 0.  Graham keeps the FARTHEST
 *    one from P0 (handle in the angle-sort comparator); Jarvis
 *    treats them as "same direction, pick farthest."
 *  • Duplicate points: handled by the sort step in Graham
 *    (duplicates produce stable order); Jarvis sees them as
 *    candidates with the same angle and picks one consistently.
 *  • All points collinear: hull is a degenerate line segment
 *    (2 endpoints).  Graham's sweep still produces it; Jarvis
 *    walks back and forth.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At default 40 points, Graham finishes in ~40 steps;
 *    Jarvis in ~h steps where h is the hull size (typically
 *    8-15).  HUD shows step counts.
 *  • Both algorithms produce the IDENTICAL hull when finished
 *    (same set of points).  Order may differ.
 *  • Press SPACE for new points: many random distributions
 *    cluster the hull at the boundary; both algorithms find
 *    the same extreme points.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Read sort_vis.c first if state-machine algorithm animation
 *      is new.  Same coroutine pattern: each algorithm step does
 *      ONE comparison/operation per tick.
 *   2. §4 algorithms — graham_scan_step + jarvis_march_step.
 *      Read AFTER tutorials T1-T4 below.
 *   3. §5 draw — split-panel renderer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pts[N_POINTS]              the input point set (random).
 *   gr_stack[]                 Graham's stack (active hull
 *                              candidates).
 *   ja_hull[]                  Jarvis's hull (committed points).
 *   pivot                      lowest-y point — anchor for both
 *                              algorithms.
 *   cross(A, B, C)             winding determinant (T2).
 *
 * Background you need
 * ───────────────────
 *   - Sorting (Graham uses qsort).
 *   - Stack as last-in-first-out array.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Quickhull, chan's algorithm, monotone chain — other hull
 *     algorithms.  Two basic ones suffice for the lesson.
 *   - 3-D convex hulls (much harder problem).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Four tutorials that build two convex-hull algorithms.
 *
 *   T1  What IS a convex hull, and why care?
 *   T2  The cross product as winding determinant
 *   T3  Graham scan — sort, then sweep
 *   T4  Jarvis march — wrap by always turning left
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS A CONVEX HULL, AND WHY CARE?
 * ────────────────────────────────────────
 * A POLYGON is CONVEX if every line segment between two of
 * its points stays INSIDE the polygon.  The CONVEX HULL of a
 * set of points is the smallest convex polygon enclosing all
 * of them.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │           ●                                      │
 *      │      ●         ●                                 │
 *      │   ●        ●         ●                           │
 *      │              ●                                   │
 *      │   ●               ●                              │
 *      │      ●        ●                                  │
 *      │                                                  │
 *      │   the rubber-band polygon enclosing all dots     │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Use cases:
 *   - Collision detection (bounding hulls in games / physics)
 *   - Pattern recognition (shape descriptors)
 *   - Computational geometry primitives (used inside Voronoi
 *     diagrams, Delaunay triangulation, etc.)
 *   - Quality control (find the extreme outliers in a sample)
 *
 * Both algorithms in this file solve the same problem; we
 * compare them side-by-side because their TRADE-OFFS differ:
 *   - Graham:  always O(N log N), good for large N.
 *   - Jarvis:  O(N · h), beats Graham when h ≪ N.
 *
 * T2  THE CROSS PRODUCT AS WINDING DETERMINANT
 * ────────────────────────────────────────────
 * The most important primitive in computational geometry:
 *
 *     cross(A, B, C) = (Bx - Ax)(Cy - Ay) - (By - Ay)(Cx - Ax)
 *
 * Geometrically this is the SIGNED AREA of the parallelogram
 * spanned by vectors (B - A) and (C - A).  Sign tells you
 * the WINDING:
 *
 *      cross > 0       cross < 0       cross = 0
 *
 *           C              C
 *            \            /
 *      A ─── B        A ─── B          A ── B ── C
 *      (LEFT turn)    (RIGHT turn)     (collinear)
 *
 * Equivalently: "is C to the LEFT of the line A→B?"
 *
 * Both algorithms answer that one question many times:
 *   Graham: "is the new point a left turn from the previous
 *            two on the stack?"
 *   Jarvis: "is candidate C to the left of the current ray?"
 *
 * The cross product is one MULTIPLY + one MULTIPLY + one
 * SUBTRACT.  No trig, no division, no floating-point catastrophe
 * (when inputs are bounded floats).
 *
 * T3  GRAHAM SCAN — SORT, THEN SWEEP
 * ──────────────────────────────────
 * Graham (1972) observed: if you process points in ANGULAR
 * ORDER from a fixed pivot, the convex hull can be assembled
 * in ONE LINEAR SWEEP using a stack.
 *
 * Step-by-step:
 *
 *   1. PIVOT: find the lowest point P0.  It's on the hull
 *      (extreme in y).
 *
 *   2. SORT: order all other points by their polar angle from
 *      P0.  Now we walk them in counter-clockwise order.
 *
 *   3. SWEEP:
 *      stack = [P0, P1]
 *      for each subsequent P:
 *        while stack has ≥ 2 elements AND
 *              cross(stack[-2], stack[-1], P) ≤ 0:
 *          stack.pop()       ← stack[-1] is NOT on the hull
 *        stack.push(P)
 *
 * The "while pop" step is the magic: when adding P would
 * create a non-LEFT turn at stack[-1], that means stack[-1]
 * is INSIDE the hull (some triangle with P excludes it).
 * Pop it and re-check.
 *
 * Cost: sort = O(N log N).  Sweep = O(N) (every point pushed
 * once, popped at most once).  Total: O(N log N).
 *
 * Watch the demo: the sweep is animated, so you SEE the pop
 * step retract the stack at each non-convex turn.
 *
 * T4  JARVIS MARCH — WRAP BY ALWAYS TURNING LEFT
 * ──────────────────────────────────────────────
 * Jarvis (1973) — also known as GIFT WRAPPING — produces the
 * hull one vertex at a time:
 *
 *   current = lowest point P0   (definitely on hull)
 *   hull = [P0]
 *
 *   repeat:
 *     next = some point ≠ current
 *     for each candidate C ≠ current:
 *       if cross(current, next, C) < 0:
 *         next = C           ← C is more counter-clockwise
 *     if next == P0: break    ← wrapped back to start
 *     else: hull.push(next); current = next
 *
 * "Always pick the most LEFT point relative to my current
 * heading" — exactly what wrapping a string around the
 * points would do.
 *
 * Cost: each hull vertex requires O(N) candidate checks.
 * If the hull has h vertices, total is O(N · h).
 *
 * COMPARISON:
 *   - h = 3 (triangle): Jarvis is O(3N), Graham is O(N log N).
 *     Jarvis wins for small h.
 *   - h = N (all points on hull, e.g. circle): Jarvis is
 *     O(N²), Graham is O(N log N).  Graham wins.
 *   - Random points in the plane: h grows as O(log N) on
 *     average, so Jarvis is ~O(N log N) on random input.
 *     Wash.
 *
 * The animation lets you SEE Jarvis "wrap" — it cleanly walks
 * one hull edge per outer-loop tick.  Watching the two
 * algorithms run side-by-side shows that their visual
 * "personality" differs (sweep vs march) even though the
 * RESULT is identical.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_POINTS     40
#define HUD_ROWS      2
#define RENDER_NS    (1000000000LL / 30)
#define STEP_NS      (1000000000LL / 6)   /* 6 steps per second */

enum { CP_PTS=1, CP_GR, CP_JA, CP_CUR, CP_HUD };

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_PTS, 244, -1);   /* grey  — points */
        init_pair(CP_GR,   51, -1);   /* cyan  — Graham */
        init_pair(CP_JA,   46, -1);   /* green — Jarvis */
        init_pair(CP_CUR, 226, -1);   /* yellow — current */
        init_pair(CP_HUD, 244, -1);
    } else {
        init_pair(CP_PTS, COLOR_WHITE,  -1);
        init_pair(CP_GR,  COLOR_CYAN,   -1);
        init_pair(CP_JA,  COLOR_GREEN,  -1);
        init_pair(CP_CUR, COLOR_YELLOW, -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  algorithms                                                         */
/* ===================================================================== */

typedef struct { float x, y; } Point;

static Point g_pts[N_POINTS];
static int g_rows, g_cols;

/* Graham scan state */
static int   g_gs_sorted[N_POINTS];   /* sorted point indices */
static int   g_gs_stack[N_POINTS];    /* convex hull stack    */
static int   g_gs_sp;                  /* stack pointer        */
static int   g_gs_idx;                 /* current scan index   */
static bool  g_gs_done;
static long long g_gs_steps;

/* Jarvis march state */
static int   g_jv_hull[N_POINTS+1];   /* hull point indices   */
static int   g_jv_n;                   /* hull size            */
static int   g_jv_cur;                 /* current hull tip     */
static int   g_jv_cand;                /* current candidate being tested */
static int   g_jv_best;                /* best (most CCW) candidate so far */
static bool  g_jv_done;
static long long g_jv_steps;

static float cross2(const Point *O, const Point *A, const Point *B)
{
    return (A->x - O->x)*(B->y - O->y) - (A->y - O->y)*(B->x - O->x);
}

static float polar_angle(const Point *ref, const Point *p)
{
    return atan2f(p->y - ref->y, p->x - ref->x);
}

static int g_pivot;   /* lowest-leftmost point for Graham */

static int cmp_angle(const void *a, const void *b)
{
    int ia = *(const int*)a, ib = *(const int*)b;
    float da = polar_angle(&g_pts[g_pivot], &g_pts[ia]);
    float db = polar_angle(&g_pts[g_pivot], &g_pts[ib]);
    if (da < db) return -1;
    if (da > db) return  1;
    /* tie: sort by distance */
    float la = (g_pts[ia].x-g_pts[g_pivot].x)*(g_pts[ia].x-g_pts[g_pivot].x)
             + (g_pts[ia].y-g_pts[g_pivot].y)*(g_pts[ia].y-g_pts[g_pivot].y);
    float lb = (g_pts[ib].x-g_pts[g_pivot].x)*(g_pts[ib].x-g_pts[g_pivot].x)
             + (g_pts[ib].y-g_pts[g_pivot].y)*(g_pts[ib].y-g_pts[g_pivot].y);
    return (la < lb) ? -1 : (la > lb) ? 1 : 0;
}

static void graham_init(void)
{
    /* find lowest (then leftmost) point */
    g_pivot = 0;
    for (int i = 1; i < N_POINTS; i++) {
        if (g_pts[i].y < g_pts[g_pivot].y ||
            (g_pts[i].y == g_pts[g_pivot].y && g_pts[i].x < g_pts[g_pivot].x))
            g_pivot = i;
    }
    for (int i = 0; i < N_POINTS; i++) g_gs_sorted[i] = i;
    qsort(g_gs_sorted, N_POINTS, sizeof(int), cmp_angle);

    g_gs_stack[0] = g_gs_sorted[0];
    g_gs_stack[1] = g_gs_sorted[1];
    g_gs_sp  = 2;
    g_gs_idx = 2;
    g_gs_done  = false;
    g_gs_steps = 0;
}

/* One Graham scan step: process g_gs_idx */
static void graham_step(void)
{
    if (g_gs_done) return;
    if (g_gs_idx >= N_POINTS) { g_gs_done = true; return; }

    while (g_gs_sp >= 2 &&
           cross2(&g_pts[g_gs_stack[g_gs_sp-2]],
                  &g_pts[g_gs_stack[g_gs_sp-1]],
                  &g_pts[g_gs_sorted[g_gs_idx]]) <= 0.f)
        g_gs_sp--;

    g_gs_stack[g_gs_sp++] = g_gs_sorted[g_gs_idx++];
    g_gs_steps++;
}

/* Jarvis march init */
static int g_jv_start;

static void jarvis_init(void)
{
    /* start from leftmost point */
    g_jv_start = 0;
    for (int i = 1; i < N_POINTS; i++)
        if (g_pts[i].x < g_pts[g_jv_start].x) g_jv_start = i;

    g_jv_hull[0] = g_jv_start;
    g_jv_n    = 1;
    g_jv_cur  = g_jv_start;
    g_jv_cand = 0;
    g_jv_best = (g_jv_start == 0) ? 1 : 0;  /* initial best: first point != cur */
    g_jv_done = false;
    g_jv_steps = 0;
}

/* One Jarvis step: compare one candidate against current best */
static void jarvis_step(void)
{
    if (g_jv_done) return;

    /* Skip self */
    if (g_jv_cand == g_jv_cur) { g_jv_cand++; return; }

    /* All candidates tested — confirm best as next hull vertex */
    if (g_jv_cand >= N_POINTS) {
        if (g_jv_best == g_jv_start && g_jv_n > 1) {
            g_jv_done = true;
            return;
        }
        g_jv_hull[g_jv_n++] = g_jv_best;
        g_jv_cur  = g_jv_best;
        g_jv_cand = 0;
        g_jv_best = (g_jv_cur == 0) ? 1 : 0;  /* reset best to first point != new cur */
        return;
    }

    /* Compare candidate vs current best: is cand more CCW from cur? */
    float c = cross2(&g_pts[g_jv_cur], &g_pts[g_jv_best], &g_pts[g_jv_cand]);
    if (c < 0.f)
        g_jv_best = g_jv_cand;

    g_jv_cand++;
    g_jv_steps++;
}

static void new_points(int rows, int cols)
{
    int pw = cols / 2 - 4;
    int ph = rows - HUD_ROWS - 2;
    for (int i = 0; i < N_POINTS; i++) {
        g_pts[i].x = 2 + (float)rand() / RAND_MAX * pw;
        g_pts[i].y = 1 + (float)rand() / RAND_MAX * ph;
    }
    graham_init();
    jarvis_init();
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static bool g_paused;
static long long g_step_accum;
static long long g_step_ns = STEP_NS;

/* Draw a line between two points using mvaddch (Bresenham) */
static void draw_line(float x0, float y0, float x1, float y1, int cp, int col_offset)
{
    int dx = (int)fabsf(x1 - x0), dy = (int)fabsf(y1 - y0);
    int steps = dx > dy ? dx : dy;
    if (steps == 0) steps = 1;
    attron(COLOR_PAIR(cp) | A_BOLD);
    for (int s = 0; s <= steps; s++) {
        float t = (float)s / (float)steps;
        int col = (int)(x0 + t*(x1-x0)) + col_offset;
        int row = (int)(y0 + t*(y1-y0)) + HUD_ROWS;
        if (row >= HUD_ROWS && row < g_rows && col >= 0 && col < g_cols)
            mvaddch(row, col, '-');
    }
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void scene_draw(void)
{
    int half = g_cols / 2;

    /* draw divider */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    for (int r = HUD_ROWS; r < g_rows; r++) mvaddch(r, half, '|');
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    /* draw points in both panels */
    for (int i = 0; i < N_POINTS; i++) {
        int col = (int)g_pts[i].x;
        int row = (int)g_pts[i].y + HUD_ROWS;
        if (row < HUD_ROWS || row >= g_rows) continue;
        attron(COLOR_PAIR(CP_PTS));
        if (col >= 0 && col < g_cols)          mvaddch(row, col, '*');
        if (col+half >= 0 && col+half < g_cols) mvaddch(row, col+half, '*');
        attroff(COLOR_PAIR(CP_PTS));
    }

    /* Graham scan hull edges */
    for (int i = 0; i < g_gs_sp - 1; i++) {
        draw_line(g_pts[g_gs_stack[i]].x,   g_pts[g_gs_stack[i]].y,
                  g_pts[g_gs_stack[i+1]].x, g_pts[g_gs_stack[i+1]].y,
                  CP_GR, 0);
    }
    /* closing edge when done */
    if (g_gs_done && g_gs_sp >= 3)
        draw_line(g_pts[g_gs_stack[g_gs_sp-1]].x, g_pts[g_gs_stack[g_gs_sp-1]].y,
                  g_pts[g_gs_stack[0]].x,          g_pts[g_gs_stack[0]].y,
                  CP_GR, 0);

    /* Jarvis march hull edges (confirmed) */
    for (int i = 0; i < g_jv_n - 1; i++) {
        draw_line(g_pts[g_jv_hull[i]].x,   g_pts[g_jv_hull[i]].y,
                  g_pts[g_jv_hull[i+1]].x, g_pts[g_jv_hull[i+1]].y,
                  CP_JA, half);
    }
    if (g_jv_done && g_jv_n >= 3)
        draw_line(g_pts[g_jv_hull[g_jv_n-1]].x, g_pts[g_jv_hull[g_jv_n-1]].y,
                  g_pts[g_jv_hull[0]].x,          g_pts[g_jv_hull[0]].y,
                  CP_JA, half);

    /* Jarvis in-progress edge: show current best candidate */
    if (!g_jv_done)
        draw_line(g_pts[g_jv_cur].x,  g_pts[g_jv_cur].y,
                  g_pts[g_jv_best].x, g_pts[g_jv_best].y,
                  CP_CUR, half);

    /* labels */
    attron(COLOR_PAIR(CP_GR) | A_BOLD);
    mvprintw(HUD_ROWS, 1, "Graham scan O(n log n)");
    attroff(COLOR_PAIR(CP_GR) | A_BOLD);
    attron(COLOR_PAIR(CP_JA) | A_BOLD);
    mvprintw(HUD_ROWS, half + 1, "Jarvis march O(n*h)");
    attroff(COLOR_PAIR(CP_JA) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " ConvexHull  q:quit  spc:new points  p:pause  +/-:speed  N=%d", N_POINTS);
    mvprintw(1, 0,
        " Graham: steps=%lld hull=%d %s   Jarvis: steps=%lld hull=%d %s",
        g_gs_steps, g_gs_sp, g_gs_done ? "[DONE]" : "",
        g_jv_steps, g_jv_n,  g_jv_done ? "[DONE]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);
    new_points(g_rows, g_cols);

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            new_points(g_rows, g_cols);
            last = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': new_points(g_rows, g_cols); break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case '+': case '=': g_step_ns /= 2; if (g_step_ns < 10000000LL) g_step_ns = 10000000LL; break;
        case '-': g_step_ns *= 2; if (g_step_ns > 2000000000LL) g_step_ns = 2000000000LL; break;
        default: break;
        }

        long long now = clock_ns();
        long long dt  = now - last;
        last = now;

        if (!g_paused) {
            g_step_accum += dt;
            while (g_step_accum >= g_step_ns) {
                g_step_accum -= g_step_ns;
                if (!g_gs_done) graham_step();
                if (!g_jv_done) jarvis_step();
            }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
