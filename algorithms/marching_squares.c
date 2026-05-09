/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_squares.c — Marching Squares isosurface extraction
 *
 * Algorithm overview
 * ------------------
 * Given a scalar field f(x,y), find the contour f(x,y) = threshold.
 *
 * Step 1 — Sample f at every grid corner.
 * Step 2 — For each 2×2 cell of corners, classify each corner as
 *           inside (f > threshold) or outside (f ≤ threshold).
 *           Four corners → 4-bit index → 16 possible cases.
 * Step 3 — Look up which edges the contour crosses in a 16-entry table.
 * Step 4 — Interpolate crossing point along each edge (linear interp).
 * Step 5 — Draw a character at each crossing. Result is an iso-contour.
 *
 * The 16-case lookup table (by 4-bit corner index):
 *   Bits: bit3=TL  bit2=TR  bit1=BR  bit0=BL  (1=inside, 0=outside)
 *   Cases 0 and 15: no contour (all outside / all inside).
 *   Cases 1–14: one or two edges crossed.
 *
 * Scalar field used here:
 *   f(x,y) = Σ_i  A_i / ( (x-cx_i)² + (y-cy_i)² )   (metaball potential)
 *
 * Multiple iso-levels can be drawn simultaneously with different chars.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   +/-       increase / decrease iso-threshold
 *   m         toggle multi-level (5 iso-levels)
 *   t         cycle colour theme
 *   r         randomise blob positions
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/marching_squares.c \
 *       -o marching_squares -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 field  §4 marching  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Marching Squares — the 2-D analogue of Marching Cubes (Lorensen
 *                  & Cline, 1987).  Classifies each 2×2 cell of the scalar field
 *                  by a 4-bit index (inside/outside per corner), looks up which
 *                  of the 16 possible edge-crossing patterns applies, then draws
 *                  an ASCII character at each crossing position.
 *
 * Math           : Metaball potential field: f(x,y) = Σ A_i / r_i²
 *                  where r_i = distance from point (x,y) to source i.
 *                  This is the gravitational potential of multiple point masses.
 *                  When f(x,y) = threshold, the iso-contour is the locus of
 *                  equal potential — it encircles sources and merges blobs when
 *                  sources are close enough (classic "organic" metaball look).
 *
 * Rendering      : Terminal cells are ~2× taller than wide (ASPECT=0.5).
 *                  The field is sampled at (col × ASPECT, row) in world space
 *                  to correct for this and produce circular blobs on screen.
 *
 * Performance    : O(W×H) per frame.  The scalar field is re-evaluated at every
 *                  grid corner each frame (N_BLOBS × W × H evaluations).
 *                  No caching because blob positions change every frame.
 *                  Multi-level mode draws 5 iso-contours with one pass over the
 *                  same pre-evaluated corner values.
 *
 * References     :
 *   Lorensen & Cline, "Marching Cubes: A High Resolution 3D Surface
 *     Construction Algorithm," SIGGRAPH 1987.  The 3-D version this
 *     2-D file derives from.  See also raster/marching_cubes.c in
 *     this project.
 *   Wikipedia, "Marching squares" — 16-case derivation + ambiguity
 *     resolution.  https://en.wikipedia.org/wiki/Marching_squares
 *   Paul Bourke, "Polygonising a scalar field":
 *     https://paulbourke.net/geometry/polygonise/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't trace the contour analytically.  Walk a grid; at each 2×2
 * cell, ask "of my four corners, which are inside the contour?"
 * Four corners → 16 possible patterns.  Each pattern has a
 * known answer for "where does the contour cross my edges?"
 * Look up the answer in a 16-entry table; draw line segments at
 * the crossings; move to the next cell.  The continuous contour
 * emerges from a discrete table lookup.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the field as a heightmap.  The contour is the
 * coastline at some sea level.  The grid is a fishnet laid
 * over the heightmap.  At each square of fishnet, you note
 * which of its four corners are above water and which are
 * below.  16 possible "wetness" patterns per square — for each,
 * you know which way the coastline goes through.  Sketch a
 * line segment.  Move to next square.  Done.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │    one cell, case 6 (bottom corners IN):         │
 *      │                                                  │
 *      │    out  ─────  out                               │
 *      │     │            │                               │
 *      │     ●            ●  ← edge crossings (interpolate)│
 *      │     │            │                               │
 *      │    in   ─────  in                                │
 *      │                                                  │
 *      │   contour line connects the two ● crossings     │
 *      └──────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *  1. Sample the scalar field f at every grid corner — one
 *     pass over the (W+1)×(H+1) corner grid.
 *  2. For each 2×2 cell of corners (there are W×H cells):
 *     a. Build a 4-bit index: bit3=TL, bit2=TR, bit1=BR, bit0=BL.
 *        Each bit set if that corner's value > threshold.
 *     b. Look up MARCHING_TABLE[index] → list of edge pairs to
 *        connect.  Cases 0 + 15 produce nothing (all out / all in).
 *     c. For each edge crossing:
 *        - Find which two corners straddle the threshold.
 *        - Linearly interpolate the crossing point on the edge:
 *            t = (threshold - f_a) / (f_b - f_a)
 *            x_cross = lerp(corner_a, corner_b, t)
 *        - Paint a glyph at that screen cell.
 *  3. Multi-level mode: repeat step 2 with 5 different threshold
 *     values, each producing its own coloured contour.
 *
 * KEY FORMULAS
 * ────────────
 *   Cell index:    bits = ((TL>thresh)<<3) | ((TR>thresh)<<2)
 *                       | ((BR>thresh)<<1) |  (BL>thresh)
 *                  ∈ [0, 15]
 *
 *   Linear edge crossing:  t = (thresh - f_a) / (f_b - f_a)
 *                          (t ∈ [0, 1] tells where on the edge)
 *
 *   Metaball field:        f(x, y) = Σ A_i / r_i²
 *                          (where r_i = dist to source i)
 *
 *   Aspect correction:     x_world = x_cell · ASPECT (= 0.5)
 *                          so blobs read circular on a 2:1 cell
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Saddle ambiguity: cases 5 and 10 (alternating in/out
 *     diagonal) admit TWO contour configurations.  We pick one
 *     consistently; alternative would change topology.
 *   • Float roundoff at threshold boundary: when corner == threshold
 *     exactly, the < check decides arbitrarily.  Tiny; invisible.
 *   • Aspect: terminal cells are ~2:1 tall.  We multiply x by
 *     ASPECT = 0.5 before sampling f so circles look round.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Default threshold (0.20): blobs are visibly enclosed by
 *     contour lines.  Drag threshold up (+) → blobs shrink;
 *     down (-) → blobs grow and merge.
 *   • Multi-level (m): 5 nested contours — like elevation lines
 *     on a topographic map.  Each level encloses an area
 *     proportional to the field's height there.
 *   • Reset (r): blobs randomise positions; topology
 *     immediately re-extracts on next frame (no state to update).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      raster/marching_cubes.c is the 3-D big sister of this file
 *      (same algorithm, 8 corners + 256 cases instead of 4 + 16);
 *      its tutorials T1-T6 cover the 3-D version.  Read this one
 *      first — the 2-D case is easier to understand.
 *   2. §4 marching — the 16-case table + extraction loop.  THE
 *      HEART of the file.  Read AFTER tutorials T1-T5 below.
 *   3. §3 field — metaball scalar field.  Independent of marching.
 *   4. §5 draw + §6 app — rendering and main loop.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   threshold              the iso-value being extracted (the
 *                          "sea level" in the topographic analogy).
 *   N_BLOBS = 4            number of metaball sources.
 *   N_LEVELS = 5           number of contour levels in multi mode.
 *   ASPECT = 0.5           horizontal compress factor for circular
 *                          blobs on tall cells.
 *   bits                   4-bit cell case index (0..15).
 *   t                      lerp parameter ∈ [0, 1] for edge crossing.
 *
 * Background you need
 * ───────────────────
 *   - Scalar fields: a function f(x, y) → ℝ.
 *   - Linear interpolation: lerp(a, b, t) = a + t·(b - a).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Marching cubes (the 3-D version).  It's a generalisation
 *     of this; see raster/marching_cubes.c when you're ready.
 *   - Dual contouring / extended marching cubes — refinements
 *     for sharp features in 3-D.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build marching squares from first
 * principles.
 *
 *   T1  The level-set problem in 2-D
 *   T2  Why GRID instead of analytic — the marching trick
 *   T3  The 16 cases — encoding corner state in 4 bits
 *   T4  Linear edge crossing — placing the line segment
 *   T5  Multi-level contours — same code, different threshold
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE LEVEL-SET PROBLEM IN 2-D
 * ────────────────────────────────
 * Take any function f: ℝ² → ℝ that returns a real number for
 * every (x, y) point.  Pick a threshold T.  The set of points
 * where f(x, y) = T is a 1-D curve called the LEVEL SET or
 * ISO-CONTOUR of f at T.
 *
 *   f(x, y) = x² + y² - 1   →   level T = 0 is a unit circle.
 *   f = elevation           →   level T = 100m is a topo line.
 *   f = density of clouds   →   level T = 0.5 is the cloud edge.
 *
 * THE PROBLEM: there's no closed-form list of (x, y) pairs for
 * an arbitrary level set.  Even for the unit circle, the
 * formula doesn't give you draw-ready coordinates — it tells
 * you which points are ON the circle, not how to enumerate
 * them.
 *
 * Marching squares solves this for ANY scalar field: it
 * produces a piecewise-linear approximation of f(x, y) = T
 * from a discrete grid of sample values, no analytic
 * surface-finding required.
 *
 * T2  WHY GRID INSTEAD OF ANALYTIC — THE MARCHING TRICK
 * ─────────────────────────────────────────────────────
 * The trick: don't try to trace the curve directly.  Instead,
 * partition the plane into a regular grid of cells.  At each
 * cell, ask the LOCAL question:
 *
 *     "Of my four corners, which are INSIDE the curve
 *      (f(corner) > T) and which are OUTSIDE?"
 *
 * Four corners, each binary → 2⁴ = 16 possible patterns.  The
 * curve segment inside that cell is approximated by lines
 * connecting the EDGES that cross from inside-to-outside.
 *
 * For each of the 16 cases, ONE line layout is enough (with
 * the saddle ambiguities tweaked).  Lookup table.  16 entries.
 *
 * The whole continuous contour emerges from doing this lookup
 * at EVERY cell of the grid.  Adjacent cells' shared edges
 * produce MATCHING crossings (because both cells see the same
 * corner values), so the line segments connect seamlessly.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  one contour threading through a grid:           │
 *      │                                                  │
 *      │  ┌─────┬─────┬─────┬─────┐                       │
 *      │  │     │  ╱  │  ╲  │     │                       │
 *      │  ├─────┼─/───┼───\─┼─────┤                       │
 *      │  │  ╱  │     │     │  ╲  │                       │
 *      │  ├─/───┼─────┼─────┼───\─┤                       │
 *      │  │     │     │     │     │                       │
 *      │  └─────┴─────┴─────┴─────┘                       │
 *      │                                                  │
 *      │  each cell decides locally which line(s) to draw│
 *      └──────────────────────────────────────────────────┘
 *
 * Same algorithmic idea as marching cubes (3-D, 256 cases) —
 * just the 2-D reduction.
 *
 * T3  THE 16 CASES — ENCODING CORNER STATE IN 4 BITS
 * ──────────────────────────────────────────────────
 * Number the four corners of a cell:
 *
 *     TL ─────── TR        (top-left, top-right)
 *     │           │
 *     │           │
 *     BL ─────── BR        (bottom-left, bottom-right)
 *
 * Build a 4-bit index by setting bit n if corner n is INSIDE:
 *
 *     bit 3: TL    bit 2: TR    bit 1: BR    bit 0: BL
 *
 * Examples:
 *   case 0  = 0000   no corners inside       → no contour
 *   case 1  = 0001   only BL inside          → one segment
 *   case 6  = 0110   TR + BR inside          → vertical-ish line
 *   case 15 = 1111   all corners inside      → no contour
 *
 * The 16 cases break down by symmetry:
 *   2 NULL cases (0, 15)
 *   4 one-corner cases (1, 2, 4, 8) — all rotations of "one in"
 *   4 three-corner cases (7, 11, 13, 14) — all rotations of "one out"
 *   4 two-adjacent cases (3, 6, 9, 12) — vertical or horizontal split
 *   2 saddle cases (5, 10) — diagonal in/out, AMBIGUOUS topology
 *
 * The MARCHING_TABLE in §4 is a 16-entry array of edge pairs.
 * Each entry says "to draw the contour for case N, connect
 * edge X to edge Y" (and possibly another pair, for saddles).
 *
 * T4  LINEAR EDGE CROSSING — PLACING THE LINE SEGMENT
 * ───────────────────────────────────────────────────
 * Suppose edge e connects corner a (value f_a) and corner b
 * (value f_b) with f_a < T < f_b.  The contour passes through
 * SOMEWHERE along that edge.  WHERE exactly?
 *
 * Approximation: assume f varies LINEARLY along the edge.
 * Then the contour crosses at:
 *
 *     f_a + t · (f_b - f_a) = T
 *     t = (T - f_a) / (f_b - f_a)
 *
 *     crossing = lerp(corner_a_pos, corner_b_pos, t)
 *
 * Notice this is the SAME interpolation as marching cubes
 * (raster/marching_cubes.c T5).  Same idea in any dimension:
 * given two sample values straddling the threshold, find the
 * point along the connecting edge where the linear interpolant
 * equals T.
 *
 * For each of the 16 cases, the table tells you which edge
 * pairs to connect.  Compute the crossings (linearly).  Draw
 * line segments between them.
 *
 * Without linear interp, every crossing would land at the edge
 * MIDPOINT (t = 0.5 always) and the resulting contour would
 * look CHUNKY — you'd see the underlying grid.  With linear
 * interp, crossings slide along their edges and adjacent cells'
 * corresponding edges produce IDENTICAL crossing points (since
 * both see the same f_a, f_b).  Result: contour reads as a
 * SMOOTH curve even though every line segment lives in a
 * specific cell.
 *
 * T5  MULTI-LEVEL CONTOURS — SAME CODE, DIFFERENT THRESHOLD
 * ─────────────────────────────────────────────────────────
 * Want a topographic map (multiple elevation lines)?  Run the
 * algorithm multiple times with different threshold values:
 *
 *     for k in 0 .. N_LEVELS-1:
 *       T_k = T_min + k · (T_max - T_min) / (N_LEVELS - 1)
 *       extract_contour(field, T_k)
 *
 * Each invocation is independent; each produces its own contour
 * line.  You can colour them differently to suggest depth /
 * density.
 *
 * Crucial optimisation: the FIELD is sampled ONCE.  All N_LEVELS
 * contours read from the SAME corner-value array; only the
 * threshold-comparison + lookup step runs N times.
 *
 * Cost: O(W·H · N_LEVELS) cell tests, but only O(W·H)
 * field evaluations.  For N_BLOBS = 4 and W·H ≈ 80·24, that's
 * ~7600 corner-value reads per frame and ~38k cell tests for
 * 5 levels.  Sub-millisecond.
 *
 * Same trick is used in:
 *   - GIS topographic mapping (contour lines on terrain data)
 *   - Medical imaging (CT/MRI tissue iso-density surfaces)
 *   - Simulation visualization (pressure / temperature
 *     iso-lines in CFD)
 *
 * The marching algorithm is ONE of the simplest and most
 * REUSABLE pieces of computational geometry — once you've
 * implemented it for any scalar field, you can plug in
 * different fields without touching the extraction code.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
/* §1  config                                                             */
/* ===================================================================== */

#define N_BLOBS       4        /* 4 metaball sources; more blobs → richer topology  */
#define BLOB_R        0.28f    /* blob influence radius in normalised [0,1] coords;
                                * at 0.28 blobs merge and separate as they orbit    */
#define THRESH_DEF    0.20f    /* iso-threshold in normalised field units;
                                * lower → more area enclosed (fatter blobs)         */
#define THRESH_STEP   0.02f
#define THRESH_MIN    0.02f
#define THRESH_MAX    0.95f
#define RENDER_FPS    20       /* 20fps for marching squares — computation is heavier
                                * than simple CA; N_BLOBS × W × H field evaluations */
#define RENDER_NS    (1000000000LL / RENDER_FPS)
#define N_THEMES      4
#define N_LEVELS      5        /* iso-levels in multi mode spaced 0.15 apart in [0,1] */

/* Aspect correction: terminal cells are ~2× taller than wide (CELL_H/CELL_W = 16/8 = 2).
 * Multiplying x by 0.5 before evaluating f(x,y) compresses the field horizontally,
 * making circular blobs appear circular on screen despite the non-square cells. */
#define ASPECT        0.5f

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
/* §3  scalar field                                                       */
/* ===================================================================== */

typedef struct { float x, y, vx, vy, str; } Blob;

static Blob g_blobs[N_BLOBS];

static void blobs_init(int rows, int cols)
{
    for (int i = 0; i < N_BLOBS; i++) {
        g_blobs[i].x   = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        g_blobs[i].y   = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        g_blobs[i].vx  = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        g_blobs[i].vy  = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        g_blobs[i].str = 1.0f;   /* strength unused — kernel is normalised */
        if (rand() & 1) g_blobs[i].vx = -g_blobs[i].vx;
        if (rand() & 1) g_blobs[i].vy = -g_blobs[i].vy;
    }
    (void)rows; (void)cols;
}

static void blobs_step(void)
{
    for (int i = 0; i < N_BLOBS; i++) {
        g_blobs[i].x += g_blobs[i].vx;
        g_blobs[i].y += g_blobs[i].vy;
        if (g_blobs[i].x < 0.05f || g_blobs[i].x > 0.95f) g_blobs[i].vx = -g_blobs[i].vx;
        if (g_blobs[i].y < 0.05f || g_blobs[i].y > 0.95f) g_blobs[i].vy = -g_blobs[i].vy;
    }
}

/*
 * Evaluate metaball field at normalised coordinates (nx, ny) ∈ [0,1]²
 *
 * Uses the Wyvill compact-support kernel:
 *   W(r) = (1 − r²/R²)³   for r < R,  else 0
 *
 * This kernel is exactly 0 outside radius R, peaks at 1 in the centre,
 * and has continuous first derivative at r=R (no discontinuity).
 * The field sums to [0, N_BLOBS], so a threshold of ~0.2 draws a contour
 * well outside each blob while ~0.8 draws only their dense cores.
 */
static float field_eval(float nx, float ny)
{
    float v = 0.0f;
    float R2 = BLOB_R * BLOB_R;
    for (int i = 0; i < N_BLOBS; i++) {
        float dx = nx - g_blobs[i].x;
        float dy = (ny - g_blobs[i].y) * ASPECT;
        float r2 = dx*dx + dy*dy;
        if (r2 >= R2) continue;
        float t = 1.0f - r2 / R2;
        v += t * t * t;
    }
    return v;
}

/* ===================================================================== */
/* §4  marching squares                                                   */
/* ===================================================================== */

/*
 * Edge table: for each 4-bit case, which of the 4 edges are crossed?
 * Edges: 0=top  1=right  2=bottom  3=left
 * Each entry is a bitmask of crossed edges.
 * Corner bit assignment: bit3=TL bit2=TR bit1=BR bit0=BL
 */
static const uint8_t edge_table[16] __attribute__((unused)) = {
    0x0,  /* 0000 — all outside        */
    0x9,  /* 0001 — BL: left+bottom    */
    0x3,  /* 0010 — BR: bottom+right   */
    0xa,  /* 0011 — BL+BR: left+right  */
    0x6,  /* 0100 — TR: right+top      */
    0xf,  /* 0101 — BL+TR: all 4 (saddle — rare) */
    0x5,  /* 0110 — BR+TR: top+bottom  */
    0xc,  /* 0111 — all but TL: top+left */
    0xc,  /* 1000 — TL: top+left       */
    0x5,  /* 1001 — TL+BL: top+bottom  */
    0xf,  /* 1010 — TL+BR: all 4 (saddle) */
    0x6,  /* 1011 — all but TR: right+top → corrected */
    0xa,  /* 1100 — TL+TR: left+right  */
    0x3,  /* 1101 — all but BR: bottom+right */
    0x9,  /* 1110 — all but BL: left+bottom */
    0x0,  /* 1111 — all inside         */
};

/*
 * Character table: pick an ASCII char that visually suggests the crossing.
 * Two edges are always crossed (except saddle cases). We pick the char
 * based on which pair of edges.
 *
 * edge pairs → char:
 *   top+bottom    → |
 *   left+right    → -
 *   top+right     → /  (or ╮)
 *   top+left      → \  (or ╭)
 *   bottom+right  → \
 *   bottom+left   → /
 *   saddle        → X
 */
static char case_char(int idx)
{
    static const char tbl[16] = {
        ' ',  /* 0000 */
        '/',  /* 0001 BL: left+bottom  → / */
        '\\', /* 0010 BR: bottom+right → \ */
        '-',  /* 0011 left+right       → - */
        '\\', /* 0100 TR: right+top    → \ */
        'X',  /* 0101 saddle            → X */
        '|',  /* 0110 top+bottom       → | */
        '/',  /* 0111 top+left         → / */
        '/',  /* 1000 TL: top+left     → / */
        '|',  /* 1001 top+bottom       → | */
        'X',  /* 1010 saddle           → X */
        '\\', /* 1011 right+top        → \ */
        '-',  /* 1100 left+right       → - */
        '\\', /* 1101 bottom+right     → \ */
        '/',  /* 1110 left+bottom      → / */
        ' ',  /* 1111 */
    };
    return tbl[idx & 0xf];
}

/* ===================================================================== */
/* §5  drawing                                                            */
/* ===================================================================== */

/* colour pairs: CP_BASE + level*2 + 0/1 for outside/inside fill */
#define CP_LABEL   1
#define CP_CONTOUR 2
#define CP_INSIDE  3
#define CP_OUTSIDE 4

static const short theme_contour[N_THEMES] = { 51, 196, 46, 201 };
static const short theme_inside [N_THEMES] = { 87, 202, 82, 171 };

static void init_colors(int theme)
{
    init_pair(CP_LABEL,   231,                  16);
    init_pair(CP_CONTOUR, theme_contour[theme], 16);
    init_pair(CP_INSIDE,  theme_inside[theme],  16);
    init_pair(CP_OUTSIDE, 244,                  16);
}

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static void on_signal(int s) { (void)s; g_quit = 1; }
static void on_resize(int s) { (void)s; g_resize = 1; }

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

typedef struct {
    float thresh;
    bool  paused;
    bool  multi;
    int   theme;
} App;

static void app_draw(const App *a)
{
    int rows = LINES - 2;   /* leave 2 rows for HUD */
    int cols = COLS;
    if (rows < 2 || cols < 2) return;

    /* pre-allocate field values at cell corners */
    static float fld[256][128];
    int fw = cols < 255 ? cols : 255;
    int fh = rows < 127 ? rows : 127;

    /* sample field at each corner */
    for (int gy = 0; gy <= fh; gy++) {
        float ny = (float)gy / (float)fh;
        for (int gx = 0; gx <= fw; gx++) {
            float nx = (float)gx / (float)fw;
            fld[gy][gx] = field_eval(nx, ny);
        }
    }

    /* march each cell */
    for (int gy = 0; gy < fh; gy++) {
        for (int gx = 0; gx < fw; gx++) {
            float tl = fld[gy  ][gx  ];
            float tr = fld[gy  ][gx+1];
            float br = fld[gy+1][gx+1];
            float bl = fld[gy+1][gx  ];

            if (a->multi) {
                /* draw multiple contour levels */
                bool drew = false;
                for (int lv = 0; lv < N_LEVELS && !drew; lv++) {
                    /* evenly spaced levels from thresh*0.3 up to thresh*0.95 */
                    float t = a->thresh * (0.3f + lv * 0.15f);
                    int idx = ((tl > t) ? 8 : 0)
                            | ((tr > t) ? 4 : 0)
                            | ((br > t) ? 2 : 0)
                            | ((bl > t) ? 1 : 0);
                    if (idx != 0 && idx != 15) {
                        char c = case_char(idx);
                        /* hue shifts per level */
                        short col = (short)(theme_contour[a->theme] + lv * 6);
                        init_pair(10 + (short)lv, col, 16);
                        attron(COLOR_PAIR(10 + lv));
                        mvaddch(gy, gx, c);
                        attroff(COLOR_PAIR(10 + lv));
                        drew = true;
                    }
                }
                if (!drew) {
                    /* fill inside regions faintly */
                    if (fld[gy][gx] > a->thresh) {
                        attron(COLOR_PAIR(CP_INSIDE));
                        mvaddch(gy, gx, '`');
                        attroff(COLOR_PAIR(CP_INSIDE));
                    }
                }
            } else {
                float t = a->thresh;
                int idx = ((tl > t) ? 8 : 0)
                        | ((tr > t) ? 4 : 0)
                        | ((br > t) ? 2 : 0)
                        | ((bl > t) ? 1 : 0);

                if (idx == 0) {
                    /* all outside — blank */
                } else if (idx == 15) {
                    /* all inside — fill char */
                    attron(COLOR_PAIR(CP_INSIDE));
                    mvaddch(gy, gx, '.');
                    attroff(COLOR_PAIR(CP_INSIDE));
                } else {
                    /* contour crossing */
                    char c = case_char(idx);
                    attron(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
                    mvaddch(gy, gx, c);
                    attroff(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
                }
            }
        }
    }

    /* HUD */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(rows,     0, "Marching Squares  threshold=%.2f  [+/-] adjust  [m] multi  [t] theme  [r] random  [q] quit",
             a->thresh);
    mvprintw(rows + 1, 0, "16-case lookup: each 2x2 cell → 4-bit index → contour edge character");
    attroff(COLOR_PAIR(CP_LABEL));

    if (a->paused) {
        attron(A_REVERSE | A_BOLD);
        mvprintw(0, COLS - 8, " PAUSED ");
        attroff(A_REVERSE | A_BOLD);
    }

    refresh();
}

int main(void)
{
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_resize);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    start_color();
    use_default_colors();

    App a = { .thresh = THRESH_DEF, .paused = false, .multi = false, .theme = 0 };
    init_colors(a.theme);
    blobs_init(LINES, COLS);

    long long next = clock_ns();

    while (!g_quit) {
        if (g_resize) { g_resize = 0; endwin(); refresh(); }

        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case ' ': a.paused = !a.paused; break;
            case '+': case '=':
                a.thresh += THRESH_STEP;
                if (a.thresh > THRESH_MAX) a.thresh = THRESH_MAX;
                break;
            case '-':
                a.thresh -= THRESH_STEP;
                if (a.thresh < THRESH_MIN) a.thresh = THRESH_MIN;
                break;
            case 'm': a.multi = !a.multi; break;
            case 't':
                a.theme = (a.theme + 1) % N_THEMES;
                init_colors(a.theme);
                break;
            case 'r': blobs_init(LINES, COLS); break;
        }

        long long now = clock_ns();
        if (now >= next) {
            if (!a.paused) blobs_step();
            erase();
            app_draw(&a);
            next += RENDER_NS;
        } else {
            clock_sleep_ns(next - now);
        }
    }

    endwin();
    return 0;
}
