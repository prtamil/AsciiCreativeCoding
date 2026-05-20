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
 * References
 * ──────────
 *   ── The two original algorithms ──────────────────────────────────
 *   [1] Graham, R. L. (1972), "An efficient algorithm for determining
 *       the convex hull of a finite planar set", Information
 *       Processing Letters 1(4), pp. 132-133 — the polar-sort + stack-
 *       sweep algorithm implemented by graham_init/graham_step.
 *   [2] Jarvis, R. A. (1973), "On the identification of the convex
 *       hull of a finite set of points in the plane", Information
 *       Processing Letters 2(1), pp. 18-21 — the gift-wrap algorithm
 *       implemented by jarvis_init/jarvis_step.
 *   [3] Chand, D. R. & Kapur, S. S. (1970), "An algorithm for convex
 *       polytopes", J. ACM 17(1) — the n-dimensional gift-wrap that
 *       Jarvis specialised to 2-D three years later.
 *
 *   ── Output-sensitive + asymptotically better hulls ──────────────
 *   [4] Chan, T. M. (1996), "Optimal output-sensitive convex hull
 *       algorithms in two and three dimensions", Discrete & Comput.
 *       Geom. 16(4), pp. 361-368 — O(N log h) hybrid that combines
 *       Graham and gift-wrap; the modern state-of-the-art. Worth
 *       reading once you know both algorithms in this file.
 *   [5] Kirkpatrick, D. G. & Seidel, R. (1986), "The ultimate planar
 *       convex hull algorithm?", SIAM J. Computing 15(1), pp. 287-299
 *       — earlier O(N log h) algorithm; explains why output-sensitive
 *       bounds matter.
 *
 *   ── Canonical computational-geometry textbooks ──────────────────
 *   [6] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 1 introduces convex hulls;
 *       Theorem 1.1 proves Graham's correctness.
 *   [7] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge — §3.5 Graham scan with full C implementation;
 *       §1.5 the signed-area / orientation predicate cross2 uses.
 *   [8] Preparata, F. P. & Shamos, M. I. (1985), "Computational
 *       Geometry: An Introduction", Springer — the field's
 *       foundational textbook; §3.3 surveys hull algorithms.
 *
 *   ── Numerical robustness (the FP gotcha behind cross2) ──────────
 *   [9] Shewchuk, J. R. (1997), "Adaptive precision floating-point
 *       arithmetic and fast robust geometric predicates", Discrete
 *       & Comput. Geom. 18(3), pp. 305-363 — why naive cross2 can
 *       MISCLASSIFY near-collinear points and how to fix it. This
 *       demo uses naive float; production code should not.
 *
 *   ── Online quick reference ──────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Convex_hull_algorithms
 *  [11] https://en.wikipedia.org/wiki/Graham_scan
 *  [12] https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
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

/*
 * Point — a 2-D input point in screen-pixel coordinates.
 *
 * Intent
 *   The convex-hull algorithms operate on an INDEXED ARRAY of these.
 *   Both Graham and Jarvis store INDICES into Scene.pts[] (not Point
 *   values) in their working arrays — copying a Point is cheap (8
 *   bytes) but indices stay smaller and let multiple algorithms
 *   share one canonical point list without aliasing problems.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right).
 *   y : SOUTHWARD pixel coordinate (positive → DOWN the screen).
 *
 *   The y-down convention matters for ORIENTATION semantics:
 *   cross2(O, A, B) > 0 is mathematically a CCW turn (positive
 *   angular sweep around O), but on screen it APPEARS CW because
 *   y is flipped. graham_step + jarvis_step still produce a hull
 *   that LOOKS correct because the comparison is symmetric — what
 *   matters is the SIGN consistency, not the absolute orientation
 *   convention.
 *
 * Why float (not int)
 *   new_points() places points with floating-point arithmetic
 *   (jittered grid). Storing them as floats avoids a quantisation
 *   pass between generation and the algorithm — cross2's near-
 *   collinear test stays as accurate as the generator allowed.
 *   Float also matches O'Rourke's reference C implementation [7].
 *
 *   Caveat: naive float cross-products can MISCLASSIFY near-collinear
 *   points (the FP rounding can flip the sign of a true-zero
 *   determinant). Production code uses exact arithmetic or robust
 *   predicates — see [9] Shewchuk. This demo accepts the risk.
 *
 * Why a value type
 *   8 bytes; passes in registers. cross2 takes three Point pointers
 *   not values, but only because cross2 was written for the pointer
 *   convention used in O'Rourke's reference code; by-value would
 *   work equivalently.
 *
 * References [7] O'Rourke §1.5 for the orientation primitive that
 *   makes Point useful; [9] Shewchuk for the FP-robustness caveat.
 */
typedef struct {
    float x;   /* eastward  pixel coordinate */
    float y;   /* southward pixel coordinate */
} Point;

/*
 * ─────────────────────────────────────────────────────────────────────
 *  Scene — owns all simulation + render state for one run.
 * ─────────────────────────────────────────────────────────────────────
 *
 * One instance lives in main() and is passed by pointer to every
 * algorithm / draw / init function. The fields group by concern:
 *
 *   ── Shared simulation state ── read by BOTH algorithms
 *      pts[]            input point cloud (the algorithms' input)
 *      rows, cols       terminal extent in CHARACTER CELLS
 *                       (needed by new_points + the rasteriser)
 *
 *   ── Graham scan state (§4 graham_init / graham_step) ──
 *      gs_sorted[]      input indices ordered by polar angle around
 *                       pivot — the scan visits them in this order.
 *      gs_stack[]       hull-vertex stack; final hull = stack[0..sp-1].
 *      gs_sp            stack pointer (number of valid entries).
 *      gs_idx           next sorted-index to process.
 *      gs_done          true once the scan has consumed all points.
 *      gs_steps         visualiser counter (push events).
 *      pivot            lowest-leftmost point — Graham's anchor.
 *
 *   ── Jarvis march state (§4 jarvis_init / jarvis_step) ──
 *      jv_hull[]        hull vertices in emission order; +1 slot
 *                       for the closing wrap-back.
 *      jv_n             hull size so far (vertices emitted).
 *      jv_cur           current hull tip — last emitted vertex.
 *      jv_cand          candidate index this comparison.
 *      jv_best          best (most-CCW) candidate found so far.
 *      jv_done          true once wrapping returns to start.
 *      jv_steps         visualiser counter (comparisons).
 *      jv_start         first hull vertex (leftmost point).
 *
 *   ── Render / app state ──
 *      paused           pause flag (toggled by 'p')
 *      step_accum       fixed-step accumulator (ns)
 *      step_ns          per-step interval (ns), adjusted by +/-
 *
 * Why a STRUCT (not file-scope globals)
 *   Keeps the algorithm/render concerns from leaking into module
 *   scope, makes the "what does this function read/write" question
 *   answerable from the signature, and leaves the door open to e.g.
 *   two side-by-side runs without aliasing.
 *
 * The signal-handler flags (g_quit, g_resize) below §6 stay as
 * globals because POSIX signal handlers can't be passed a pointer.
 * ───────────────────────────────────────────────────────────────────── */

typedef struct {
    /* Shared simulation state */
    Point     pts[N_POINTS];
    int       rows;
    int       cols;

    /* Graham scan state */
    int       gs_sorted[N_POINTS];
    int       gs_stack [N_POINTS];
    int       gs_sp;
    int       gs_idx;
    bool      gs_done;
    long long gs_steps;
    int       pivot;

    /* Jarvis march state */
    int       jv_hull[N_POINTS + 1];
    int       jv_n;
    int       jv_cur;
    int       jv_cand;
    int       jv_best;
    bool      jv_done;
    long long jv_steps;
    int       jv_start;

    /* Render / app state */
    bool      paused;
    long long step_accum;
    long long step_ns;
} Scene;

/*
 * cross2 — the 2-D cross product of vectors OA and OB.
 *
 *     cross2(O, A, B) = (A − O) × (B − O)
 *                     = (Aₓ − Oₓ)(Bᵧ − Oᵧ) − (Aᵧ − Oᵧ)(Bₓ − Oₓ)
 *
 *   Geometric interpretation: this scalar EQUALS TWICE THE SIGNED
 *   AREA of the triangle OAB. The SIGN encodes the orientation of
 *   the triple (O, A, B):
 *
 *       cross2 > 0   →  CCW (left turn) — A→B sweeps counter-clockwise around O
 *       cross2 < 0   →  CW  (right turn) — A→B sweeps clockwise around O
 *       cross2 = 0   →  COLLINEAR — O, A, B lie on one line
 *
 *   This is the most-used primitive in computational geometry; both
 *   Graham scan and Jarvis march reduce to "is this a left turn?"
 *   tests against cross2.
 *
 *   Reference: O'Rourke (1998) "Computational Geometry in C" §1.5
 *   for the signed-area derivation; de Berg et al. (2008)
 *   "Computational Geometry: Algorithms and Applications" §1.1 for
 *   the orientation predicate.
 */
static float cross2(const Point *O, const Point *A, const Point *B)
{
    return (A->x - O->x)*(B->y - O->y) - (A->y - O->y)*(B->x - O->x);
}

/*
 * is_strict_left_turn — true iff (O, A, B) is a STRICT CCW turn.
 *
 *   Used to validate hull invariants: a point B is on the upper hull
 *   relative to the previous edge OA if and only if the turn is
 *   strictly counter-clockwise. Collinear points (cross2 == 0) FAIL
 *   this test — that's intentional: a hull edge should not pass
 *   through an interior collinear point.
 */
static inline bool is_strict_left_turn(const Point *O, const Point *A,
                                       const Point *B)
{
    return cross2(O, A, B) > 0.0f;
}

/*
 * is_not_strict_left_turn — Graham scan's pop predicate.
 *
 *   The Graham inner loop pops the top of the stack while the top
 *   two points + the next candidate fail to make a strict left turn.
 *   "Fail" means right turn OR collinear (cross2 ≤ 0), so we use the
 *   non-strict negation rather than is_right_turn.
 *
 *   Why include COLLINEAR in the pop set? Because a collinear vertex
 *   is REDUNDANT — three collinear points contribute only two hull
 *   edges; keeping the middle one would clutter the hull without
 *   changing its shape.
 */
static inline bool is_not_strict_left_turn(const Point *O, const Point *A,
                                           const Point *B)
{
    return cross2(O, A, B) <= 0.0f;
}

/*
 * polar_angle — atan2 of (p − ref), in (−π, π].
 *
 *   Used as the SORT KEY in Graham scan after the pivot is found.
 *   atan2 unambiguously assigns an angle to every (Δx, Δy) ≠ 0,
 *   handles all four quadrants, and is monotonic with rotation
 *   direction — properties qsort needs to produce a clean CCW order.
 *
 *   Reference: O'Rourke (1998) §3.5 for the polar-sort step of
 *   Graham scan.
 */
static float polar_angle(const Point *ref, const Point *p)
{
    return atan2f(p->y - ref->y, p->x - ref->x);
}

/*
 * squared_distance — Euclidean distance² from a to b (no sqrt).
 *
 *   For ORDERING by distance we don't need the actual distance —
 *   only its order, which sqrt preserves monotonically. Skipping
 *   sqrt saves one transcendental per comparison without changing
 *   the sort result. This is the standard CG trick (ref O'Rourke
 *   §3 "Avoid square roots in comparisons").
 */
static inline float squared_distance(const Point *a, const Point *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return dx*dx + dy*dy;
}

/*
 * cmp_angle_ctx — transient context for cmp_angle.
 *
 *   ISO C qsort has no user-data parameter, so the comparator can't
 *   reach the Scene through its arguments. We set this pointer just
 *   before the qsort call (see sort_indices_by_polar_angle_around_pivot)
 *   and clear it after. Not thread-safe, doesn't need to be — the
 *   demo is single-threaded and qsort runs to completion.
 */
static const Scene *cmp_angle_ctx;

/*
 * cmp_angle — qsort comparator: orders points by polar angle around
 * the pivot, breaking ties by SQUARED DISTANCE.
 *
 *   PRIMARY KEY:    polar_angle(pivot, P)    (ascending → CCW order)
 *   TIE BREAKER:    squared_distance(pivot, P)  (ascending — NEAR first)
 *
 *   Why NEAR-first on ties: collinear points at equal angle from the
 *   pivot are all on the same RAY. Graham scan will discard the
 *   inner ones when the pop loop runs anyway, but ordering them
 *   near-first keeps the stack walk predictable.
 *
 *   (Some Graham variants order COLLINEAR ties FAR-first on the
 *   final edge to handle the wrap-around correctly. We don't here
 *   because the demo's random points rarely produce exact ties.)
 */
static int cmp_angle(const void *a, const void *b)
{
    const Scene *sc = cmp_angle_ctx;
    int ia = *(const int*)a, ib = *(const int*)b;
    const Point *pivot = &sc->pts[sc->pivot];

    float da = polar_angle(pivot, &sc->pts[ia]);
    float db = polar_angle(pivot, &sc->pts[ib]);
    if (da < db) return -1;
    if (da > db) return  1;

    /* TIE: same polar angle — order by distance from pivot */
    float la = squared_distance(pivot, &sc->pts[ia]);
    float lb = squared_distance(pivot, &sc->pts[ib]);
    return (la < lb) ? -1 : (la > lb) ? 1 : 0;
}

/*
 * find_lowest_leftmost_point — Graham scan's pivot.
 *
 *   The pivot is the point with MINIMUM y, breaking ties with
 *   MINIMUM x. Geometrically: the bottom-most, then leftmost
 *   point of the set.
 *
 *   Why this point: it is GUARANTEED to be on the convex hull (a
 *   bottom-most point of any set is always extreme in the −y
 *   direction). Polar-sorting the remaining points AROUND this
 *   pivot then gives a CCW traversal of the hull boundary starting
 *   at a known hull vertex.
 *
 *   Reference: Graham (1972) "An efficient algorithm for determining
 *   the convex hull of a finite planar set", Information Processing
 *   Letters 1(4) — original paper specifies this exact pivot rule.
 */
static int find_lowest_leftmost_point(const Point *pts, int n)
{
    int idx = 0;
    for (int i = 1; i < n; i++) {
        bool lower      = pts[i].y < pts[idx].y;
        bool same_y_lft = pts[i].y == pts[idx].y && pts[i].x < pts[idx].x;
        if (lower || same_y_lft) idx = i;
    }
    return idx;
}

/*
 * sort_indices_by_polar_angle_around_pivot — Graham's polar sort.
 *
 *   Fills idx_out[] with 0..n−1 then qsorts it so that walking
 *   idx_out[] visits the points in CCW polar order around sc->pivot.
 *   cmp_angle is the comparator (defined above); it reads the scene
 *   through cmp_angle_ctx, set here for the duration of the qsort.
 *
 *   Complexity: O(n log n) — the qsort dominates Graham's total
 *   complexity. Once sorted, the scan is O(n).
 */
static void sort_indices_by_polar_angle_around_pivot(const Scene *sc,
                                                     int *idx_out, int n)
{
    for (int i = 0; i < n; i++) idx_out[i] = i;
    cmp_angle_ctx = sc;
    qsort(idx_out, (size_t)n, sizeof(int), cmp_angle);
    cmp_angle_ctx = NULL;
}

/*
 * graham_init — set up the Graham scan state for a step-driven run.
 *
 *   Pseudocode:
 *     1. PIVOT      = find_lowest_leftmost_point(pts)
 *                     — guaranteed-extreme point that anchors the sort.
 *     2. SORT       sc->gs_sorted ← polar-angle order around pivot
 *                     — visiting order for the scan.
 *     3. SEED STACK with the first TWO sorted points.
 *                     The scan keeps a stack of "hull candidates"; we
 *                     start with the pivot (sorted[0]) and the next
 *                     point in CCW order (sorted[1]). These two are
 *                     trivially CCW and form the first hull edge.
 *     4. SCAN INDEX = 2  — graham_step will process sorted[2..n−1].
 *     5. Reset done flag + step counter for the visualiser.
 *
 *   Reference: Graham (1972); de Berg et al. (2008) §1.1.
 */
static void graham_init(Scene *sc)
{
    /* (1) anchor pivot */
    sc->pivot = find_lowest_leftmost_point(sc->pts, N_POINTS);

    /* (2) polar-angle CCW order */
    sort_indices_by_polar_angle_around_pivot(sc, sc->gs_sorted, N_POINTS);

    /* (3) seed stack with first two sorted points */
    sc->gs_stack[0] = sc->gs_sorted[0];
    sc->gs_stack[1] = sc->gs_sorted[1];
    sc->gs_sp       = 2;

    /* (4) scan starts at index 2 (third point in CCW order) */
    sc->gs_idx = 2;

    /* (5) reset run-state */
    sc->gs_done  = false;
    sc->gs_steps = 0;
}

/*
 * pop_while_not_strict_left_turn — Graham's hull-maintenance loop.
 *
 *   While the top of the stack and the incoming candidate fail to
 *   form a strict CCW turn together with the second-from-top point,
 *   POP the top.
 *
 *   Geometrically: the top of the stack would lie ON or INSIDE the
 *   edge from second-top → candidate, so it's not a hull vertex.
 *   Discarding it lets the candidate become the new top.
 *
 *   The loop body is the key insight of Graham: by polar-sorting
 *   first, we know the candidate is FURTHER CCW than anything on
 *   the stack, so the only way to keep convexity is to pop interior
 *   points until the remaining stack top forms a strict left turn
 *   with (second-top → candidate).
 *
 *   Each point can be pushed at most once and popped at most once,
 *   so the AMORTISED cost is O(1) per point even though a single
 *   call can pop many in a row. Total scan is O(n).
 */
static void pop_while_not_strict_left_turn(const Point *pts,
                                           int *stack, int *sp,
                                           int candidate_idx)
{
    while (*sp >= 2 &&
           is_not_strict_left_turn(&pts[stack[*sp - 2]],
                                   &pts[stack[*sp - 1]],
                                   &pts[candidate_idx]))
        (*sp)--;
}

/*
 * graham_step — process ONE candidate point per call (animation step).
 *
 *   Pseudocode:
 *     1. early-out if done or no more candidates.
 *     2. pop_while_not_strict_left_turn — discard hull-interior
 *        points from the stack until the top forms a strict CCW
 *        turn with the candidate.
 *     3. push the candidate onto the stack.
 *     4. advance to the next candidate.
 *
 *   When all candidates have been processed the stack holds the
 *   hull in CCW order (pivot first, then bottom→right→top→left).
 *
 *   This function is called once per frame by the visualiser so
 *   the user can watch the scan advance one point at a time.
 */
static void graham_step(Scene *sc)
{
    if (sc->gs_done) return;
    if (sc->gs_idx >= N_POINTS) { sc->gs_done = true; return; }

    int candidate = sc->gs_sorted[sc->gs_idx];
    pop_while_not_strict_left_turn(sc->pts, sc->gs_stack, &sc->gs_sp,
                                   candidate);
    sc->gs_stack[sc->gs_sp++] = candidate;
    sc->gs_idx++;
    sc->gs_steps++;
}

/*
 * find_leftmost_point — Jarvis march's start vertex.
 *
 *   The leftmost point has minimum x; it is GUARANTEED to be on the
 *   convex hull (extreme in the −x direction). Like Graham's
 *   lowest-leftmost pivot, this is the standard anchor that lets
 *   the algorithm produce a deterministic starting point and a
 *   well-defined termination condition (wrap back to start).
 *
 *   Reference: Jarvis (1973) "On the identification of the convex
 *   hull of a finite set of points in the plane", Information
 *   Processing Letters 2(1) — original paper.
 */
static int find_leftmost_point(const Point *pts, int n)
{
    int idx = 0;
    for (int i = 1; i < n; i++)
        if (pts[i].x < pts[idx].x) idx = i;
    return idx;
}

/*
 * first_index_not_equal_to — pick a "best so far" candidate that
 * isn't the current hull tip.
 *
 *   Jarvis needs a non-self initial guess for `best` so the
 *   subsequent loop has something to compare against. Picking the
 *   first index ≠ cur works for any input (cur ∈ {0, 1, …, n−1}
 *   always has a valid neighbour with a different index).
 */
static inline int first_index_not_equal_to(int cur)
{
    return (cur == 0) ? 1 : 0;
}

/*
 * is_more_counterclockwise — Jarvis's candidate-selection predicate.
 *
 *   "Is `cand` more CCW from `cur` than `best`?"
 *
 *   Geometrically: standing at cur, looking toward best, is cand
 *   to the RIGHT of that ray? If yes, the ray to cand sweeps
 *   FARTHER CLOCKWISE in screen space — but the hull is traced
 *   CW in screen-y-down terms, so this is in fact the CCW direction
 *   we want for the gift-wrap step.
 *
 *   Implementation: cross2(cur, best, cand) < 0 means cand sits on
 *   the CW side of the line cur→best (in math convention; CCW on
 *   screen because y is inverted). Either way, that's the side
 *   that moves the wrapping "string" further around the hull.
 */
static inline bool is_more_counterclockwise(const Point *pts,
                                            int cur, int best, int cand)
{
    return cross2(&pts[cur], &pts[best], &pts[cand]) < 0.0f;
}

/*
 * jarvis_init — gift-wrap start state.
 *
 *   1. start = find_leftmost_point   — anchor on a known hull vertex.
 *   2. hull[0] = start; n = 1         — emit the first hull point.
 *   3. cur = start                    — current "tip" of the partial hull.
 *   4. cand, best                     — reset to scan candidates from 0.
 *   5. done, steps                    — visualiser counters.
 *
 *   After init, repeatedly calling jarvis_step() will sweep one
 *   candidate per call until the wrapping string returns to `start`.
 */
static void jarvis_init(Scene *sc)
{
    sc->jv_start = find_leftmost_point(sc->pts, N_POINTS);

    sc->jv_hull[0] = sc->jv_start;
    sc->jv_n       = 1;
    sc->jv_cur     = sc->jv_start;
    sc->jv_cand    = 0;
    sc->jv_best    = first_index_not_equal_to(sc->jv_cur);

    sc->jv_done  = false;
    sc->jv_steps = 0;
}

/*
 * commit_best_as_next_hull_vertex — advance the gift-wrap one tip.
 *
 *   Called after every candidate has been compared against `best`.
 *   `best` is now the next CCW-most point from `cur`; append it
 *   to the hull, slide the tip forward, and reset the candidate
 *   scanner for the next pass.
 */
static inline void commit_best_as_next_hull_vertex(Scene *sc)
{
    sc->jv_hull[sc->jv_n++] = sc->jv_best;
    sc->jv_cur              = sc->jv_best;
    sc->jv_cand             = 0;
    sc->jv_best             = first_index_not_equal_to(sc->jv_cur);
}

/*
 * wraps_back_to_start — Jarvis termination check.
 *
 *   The gift-wrap terminates when the next-chosen vertex equals the
 *   start vertex AND we've already accumulated at least one edge
 *   (n > 1). The n > 1 guard prevents an immediate termination on
 *   the first vertex (where best could spuriously match start
 *   before any wrapping has happened).
 */
static inline bool wraps_back_to_start(const Scene *sc)
{
    return sc->jv_best == sc->jv_start && sc->jv_n > 1;
}

/*
 * jarvis_step — one comparison or one vertex-commit per call.
 *
 *   This is Jarvis march's gift-wrapping algorithm broken into the
 *   smallest visible step: each call advances the state by EITHER
 *   one candidate comparison OR one hull-vertex commit. The
 *   visualiser drives it at one step per frame so the user can
 *   watch the wrapping ray sweep around the point set.
 *
 *   Pseudocode:
 *     1. early-out if done.
 *     2. if cand == cur: skip self, advance cand.
 *     3. if all candidates compared (cand >= N_POINTS):
 *          a. if wraps_back_to_start → terminate.
 *          b. else commit_best_as_next_hull_vertex.
 *     4. otherwise compare:
 *          if is_more_counterclockwise(cur, best, cand): best ← cand
 *          advance cand.
 *
 *   Complexity per hull vertex: O(N) comparisons (one full sweep
 *   over the input). Total: O(h·N) where h is the hull size.
 *   Worse than Graham's O(N log N) when h ≈ N, but BETTER when
 *   h is small (the hull is "output-sensitive"). For h ≪ N this is
 *   actually the asymptotically optimal output-sensitive algorithm.
 *
 *   Reference: Jarvis (1973); Chand & Kapur (1970) gift-wrap in nD.
 */
static void jarvis_step(Scene *sc)
{
    if (sc->jv_done) return;

    /* (2) skip self */
    if (sc->jv_cand == sc->jv_cur) { sc->jv_cand++; return; }

    /* (3) all candidates tested — either terminate or commit */
    if (sc->jv_cand >= N_POINTS) {
        if (wraps_back_to_start(sc)) {
            sc->jv_done = true;
            return;
        }
        commit_best_as_next_hull_vertex(sc);
        return;
    }

    /* (4) compare candidate vs current best */
    if (is_more_counterclockwise(sc->pts, sc->jv_cur, sc->jv_best, sc->jv_cand))
        sc->jv_best = sc->jv_cand;

    sc->jv_cand++;
    sc->jv_steps++;
}

/*
 * new_points — generate a fresh random input set and reset BOTH algorithms.
 *
 *   Pseudocode:
 *     pw, ph := LEFT-panel pixel extent (cols/2 − 4 by rows − HUD − 2)
 *     for i in 0..N_POINTS−1:
 *         pts[i] := uniform random point in [(2, 1), (2 + pw, 1 + ph))
 *     graham_init(sc)
 *     jarvis_init(sc)
 *
 *   Why LEFT-panel coordinates only: scene_draw renders the same point
 *   set TWICE — once at its native x, once mirrored at x + half — so
 *   both panels show identical inputs. Storing in left-panel coords
 *   keeps the data canonical; the rasteriser adds the offset.
 *
 *   The (2, 1) leading margin and the (−4, −2) trailing slack in pw/ph
 *   keep points clear of the central divider and the HUD rows.
 *
 *   Bound to SPACE in the input handler; also called on resize so the
 *   point cloud rescales to the new terminal extent.
 */
static void new_points(Scene *sc)
{
    int pw = sc->cols / 2 - 4;
    int ph = sc->rows - HUD_ROWS - 2;
    for (int i = 0; i < N_POINTS; i++) {
        sc->pts[i].x = 2 + (float)rand() / RAND_MAX * pw;
        sc->pts[i].y = 1 + (float)rand() / RAND_MAX * ph;
    }
    graham_init(sc);
    jarvis_init(sc);
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

/*
 * draw_hull_edge — rasterise one line segment between two pixel points.
 *
 *   Pseudocode (DDA — Digital Differential Analyser):
 *     steps := max(|dx|, |dy|)              ← cells on the long axis
 *     for s in 0..steps:
 *         t := s / steps                    ∈ [0, 1]
 *         (x, y) := lerp((x0, y0), (x1, y1), t)
 *         plot '-' at (x + col_offset, y + HUD_ROWS)
 *
 *   DDA samples `steps + 1` points along the segment, picking the axis
 *   with the larger delta so every cell on the long axis is visited
 *   exactly once. Bresenham is the integer-only variant; we use float
 *   lerp because the endpoints are already floats and the savings don't
 *   matter at ~30 edges per frame.
 *
 *   col_offset shifts the segment into the right panel (= sc->cols/2)
 *   when drawing Jarvis's hull; it's 0 for Graham (left panel).
 *
 *   Reference: Foley et al. (1995) "Computer Graphics: Principles and
 *   Practice" §3.2 — DDA and Bresenham line rasterisation.
 */
static void draw_hull_edge(const Scene *sc,
                           float x0, float y0, float x1, float y1,
                           int cp, int col_offset)
{
    int dx = (int)fabsf(x1 - x0), dy = (int)fabsf(y1 - y0);
    int steps = dx > dy ? dx : dy;
    if (steps == 0) steps = 1;
    attron(COLOR_PAIR(cp) | A_BOLD);
    for (int s = 0; s <= steps; s++) {
        float t = (float)s / (float)steps;
        int col = (int)(x0 + t*(x1-x0)) + col_offset;
        int row = (int)(y0 + t*(y1-y0)) + HUD_ROWS;
        if (row >= HUD_ROWS && row < sc->rows && col >= 0 && col < sc->cols)
            mvaddch(row, col, '-');
    }
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * draw_hull_polyline — render a chain of hull edges, optionally closed.
 *
 *   Pseudocode:
 *     for i in 0..n−2:
 *         draw_hull_edge(pts[indices[i]] → pts[indices[i+1]])
 *     if closed and n ≥ 3:
 *         draw_hull_edge(pts[indices[n−1]] → pts[indices[0]])   ← wrap
 *
 *   Both algorithms store their partial/complete hull as an ordered
 *   index list:
 *     Graham → sc->gs_stack[0 .. gs_sp−1]   (close when gs_done)
 *     Jarvis → sc->jv_hull [0 .. jv_n−1]    (close when jv_done)
 *
 *   While the algorithm is still running, the chain is an OPEN POLYLINE
 *   (the edge from the last vertex back to the first hasn't been
 *   emitted yet). Once done, the closing edge promotes the polyline
 *   to a closed POLYGON — the finished convex hull.
 */
static void draw_hull_polyline(const Scene *sc, const int *indices, int n,
                               bool closed, int cp, int col_offset)
{
    for (int i = 0; i < n - 1; i++) {
        draw_hull_edge(sc,
                       sc->pts[indices[i]].x,   sc->pts[indices[i]].y,
                       sc->pts[indices[i+1]].x, sc->pts[indices[i+1]].y,
                       cp, col_offset);
    }
    if (closed && n >= 3) {
        draw_hull_edge(sc,
                       sc->pts[indices[n-1]].x, sc->pts[indices[n-1]].y,
                       sc->pts[indices[0]].x,   sc->pts[indices[0]].y,
                       cp, col_offset);
    }
}

/*
 * draw_active_wrapping_ray — the moving "gift-wrap string" in Jarvis.
 *
 *   While Jarvis sweeps candidates, sc->jv_best holds the most-CCW
 *   candidate found SO FAR for the next hull vertex. Drawing the
 *   edge (cur → best) shows the wrapping string pivoting between
 *   competitors; when a more counter-clockwise candidate is found,
 *   jv_best updates and this ray visibly snaps to it.
 *
 *   Only meaningful while the algorithm is still running; the caller
 *   guards with !sc->jv_done.
 */
static void draw_active_wrapping_ray(const Scene *sc, int half)
{
    draw_hull_edge(sc,
                   sc->pts[sc->jv_cur ].x, sc->pts[sc->jv_cur ].y,
                   sc->pts[sc->jv_best].x, sc->pts[sc->jv_best].y,
                   CP_CUR, half);
}

/*
 * draw_panel_divider — vertical '|' rule between the two algorithm panels.
 *   Marks the visual symmetry: same point cloud either side, different
 *   algorithm. Drawn DIM so it never competes with the hull edges.
 */
static void draw_panel_divider(const Scene *sc, int half)
{
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    for (int r = HUD_ROWS; r < sc->rows; r++) mvaddch(r, half, '|');
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);
}

/*
 * draw_point_cloud — render every input point as '*' in BOTH panels.
 *
 *   Each point is plotted twice — once at its canonical x (left panel),
 *   once mirrored at x + half (right panel) — so the two algorithms
 *   are visibly working on the same input. Per-axis bounds checks gate
 *   the writes so an off-screen point can't corrupt the terminal.
 */
static void draw_point_cloud(const Scene *sc, int half)
{
    attron(COLOR_PAIR(CP_PTS));
    for (int i = 0; i < N_POINTS; i++) {
        int col = (int)sc->pts[i].x;
        int row = (int)sc->pts[i].y + HUD_ROWS;
        if (row < HUD_ROWS || row >= sc->rows) continue;
        if (col >= 0      && col      < sc->cols) mvaddch(row, col,      '*');
        if (col+half >= 0 && col+half < sc->cols) mvaddch(row, col+half, '*');
    }
    attroff(COLOR_PAIR(CP_PTS));
}

/*
 * draw_panel_labels — algorithm name + complexity above each panel.
 *   "Graham scan O(n log n)" (left, cyan) and "Jarvis march O(n*h)"
 *   (right, green). Colours match the hull edges so the eye can map
 *   label → algorithm at a glance.
 */
static void draw_panel_labels(int half)
{
    attron(COLOR_PAIR(CP_GR) | A_BOLD);
    mvprintw(HUD_ROWS, 1, "Graham scan O(n log n)");
    attroff(COLOR_PAIR(CP_GR) | A_BOLD);
    attron(COLOR_PAIR(CP_JA) | A_BOLD);
    mvprintw(HUD_ROWS, half + 1, "Jarvis march O(n*h)");
    attroff(COLOR_PAIR(CP_JA) | A_BOLD);
}

/*
 * draw_hud — top two status lines: controls + per-algorithm progress.
 *
 *   Row 0: title, key hints, N.
 *   Row 1: live step / hull-size counters for both algorithms, with
 *          [DONE] markers once each finishes. Reading both counters
 *          side-by-side is the whole point of the demo: you can watch
 *          Graham's O(N log N) versus Jarvis's O(N · h) play out in
 *          step counts as the hulls converge.
 */
static void draw_hud(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " ConvexHull  q:quit  spc:new points  p:pause  +/-:speed  N=%d",
        N_POINTS);
    mvprintw(1, 0,
        " Graham: steps=%lld hull=%d %s   Jarvis: steps=%lld hull=%d %s",
        sc->gs_steps, sc->gs_sp, sc->gs_done ? "[DONE]" : "",
        sc->jv_steps, sc->jv_n,  sc->jv_done ? "[DONE]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/*
 * scene_draw — render one frame.
 *
 *   Pseudocode (layered draw, painter's algorithm — later layers cover
 *   earlier ones at any overlap):
 *     1. panel divider           ─ vertical rule between the two panels
 *     2. point cloud             ─ '*' at every input point, both sides
 *     3. Graham polyline         ─ left panel; closed once gs_done
 *     4. Jarvis polyline         ─ right panel; closed once jv_done
 *     5. Jarvis active wrapping  ─ pivoting cur→best edge, while running
 *     6. panel labels            ─ algorithm names above each panel
 *     7. HUD                     ─ top two lines: keys + step counters
 *
 *   Order matters: HUD goes LAST so its glyphs overlay any hull edges
 *   that might have intruded into the top rows.
 */
static void scene_draw(const Scene *sc)
{
    int half = sc->cols / 2;

    draw_panel_divider(sc, half);
    draw_point_cloud  (sc, half);

    /* Graham hull on the LEFT (offset 0); Jarvis hull on the RIGHT (offset half) */
    draw_hull_polyline(sc, sc->gs_stack, sc->gs_sp, sc->gs_done, CP_GR, 0);
    draw_hull_polyline(sc, sc->jv_hull,  sc->jv_n,  sc->jv_done, CP_JA, half);

    if (!sc->jv_done) draw_active_wrapping_ray(sc, half);

    draw_panel_labels(half);
    draw_hud         (sc);
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

    Scene scene = {0};
    scene.step_ns = STEP_NS;
    getmaxyx(stdscr, scene.rows, scene.cols);
    new_points(&scene);

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, scene.rows, scene.cols);
            new_points(&scene);
            last = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': new_points(&scene); break;
        case 'p': case 'P': scene.paused = !scene.paused; break;
        case '+': case '=': scene.step_ns /= 2; if (scene.step_ns < 10000000LL) scene.step_ns = 10000000LL; break;
        case '-': scene.step_ns *= 2; if (scene.step_ns > 2000000000LL) scene.step_ns = 2000000000LL; break;
        default: break;
        }

        long long now = clock_ns();
        long long dt  = now - last;
        last = now;

        if (!scene.paused) {
            scene.step_accum += dt;
            while (scene.step_accum >= scene.step_ns) {
                scene.step_accum -= scene.step_ns;
                if (!scene.gs_done) graham_step(&scene);
                if (!scene.jv_done) jarvis_step(&scene);
            }
        }

        erase();
        scene_draw(&scene);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
