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
 * Sections: §1 config  §2 clock  §3 types  §4 field  §5 marching
 *           §6 draw    §7 app
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

/* FieldGrid size cap — bounds the per-frame scratch sample buffer.
 * Covers any reasonable terminal up to 256 cols × 128 rows.  The field
 * is sampled at (cols+1) × (rows+1) corners, so the actual storage is
 * (FIELD_W_MAX+1) × (FIELD_H_MAX+1) floats ≈ 128 KB.                */
#define FIELD_W_MAX   255
#define FIELD_H_MAX   127

/* HUD layout — rows reserved at top/bottom of the screen for status bars.
 * Top bar holds DATA (parameter readout); bottom bar holds ACTIONS (key
 * hints).  field_sample_grid subtracts these from sc->rows; march_cells
 * offsets every drawn cell by HUD_TOP_ROWS so the field sits between
 * the two bars. */
#define HUD_TOP_ROWS  1
#define HUD_BOT_ROWS  1

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
/* §3  data types                                                         */
/* ===================================================================== */

/*
 * Blob — one moving metaball source.
 *
 * Intent
 *   The N_BLOBS sources are the INPUT to the scalar field.  Each blob
 *   contributes a Wyvill-kernel bump centred at (x, y); their sum at
 *   every grid corner is the field f(nx, ny) the marching squares
 *   algorithm then iso-extracts.
 *
 * Coordinate convention
 *   x, y are NORMALISED to [0, 1] — independent of terminal size.
 *   field_eval_at converts back to screen-aspect-corrected pixel
 *   distance by multiplying dy by ASPECT (=0.5), so circular blobs
 *   read circular on the ~2:1 terminal cell.
 *
 * Why normalised coords (not pixel)
 *   The field is sampled at corners (nx, ny) = (gx/W, gy/H); using
 *   normalised blob positions means resizing the terminal just
 *   re-buckets the same field — blob motion looks identical at any
 *   window size.
 *
 * Members
 *   x, y       position in normalised [0, 1]²
 *   vx, vy     per-frame velocity (also normalised; ~0.005)
 *   strength   currently unused — the Wyvill kernel is hard-coded
 *              to peak at 1; left as a hook for per-blob weighting
 */
typedef struct {
    float x, y;        /* position in normalised [0, 1] coords  */
    float vx, vy;      /* per-frame velocity (normalised units) */
    float strength;    /* reserved for per-blob weight (unused) */
} Blob;

/*
 * FieldGrid — scratch buffer of scalar-field samples at the corner lattice.
 *
 * Intent
 *   Cache of f(nx, ny) at every grid corner, computed ONCE per frame
 *   by field_sample_grid.  The marching loop then reads sample[gy][gx]
 *   in tight inner loops; in multi-level mode, ALL N_LEVELS contours
 *   read from the same sample[] without re-evaluating the field.
 *   Saves ~N_LEVELS× field evaluations per frame.
 *
 * Storage layout
 *   sample[row][col] — row-major.  Used region is sample[0..rows][0..cols];
 *   rows/cols track the per-frame window (clamped to FIELD_*_MAX).
 *
 * Why a SEPARATE struct (not embedded in Scene)
 *   FieldGrid is per-frame SCRATCH STORAGE — its contents have no
 *   meaning between frames.  Keeping it out of Scene clarifies the
 *   data-flow: main owns one FieldGrid in BSS, scene_draw fills it
 *   then immediately reads from it within a single call.  No
 *   cross-frame state lives here.
 *
 * Why a 128 KB static, not a malloc
 *   The maximum size is bounded by FIELD_*_MAX, so a single BSS
 *   allocation is exactly right — zero allocator overhead, no
 *   per-frame churn.  128 KB is too big for the stack but trivial
 *   for BSS on any modern system.
 *
 * Members
 *   cols     number of CELL columns this frame (corners are cols+1 wide)
 *   rows     number of CELL rows    this frame (corners are rows+1 tall)
 *   sample   per-corner f values; sample[0..rows][0..cols] valid
 *
 * Invariants
 *   0 ≤ cols ≤ FIELD_W_MAX   and   0 ≤ rows ≤ FIELD_H_MAX
 *   After field_sample_grid returns, sample[gy][gx] for gy ∈ [0,rows]
 *   and gx ∈ [0,cols] holds f(gx/cols, gy/rows).  Other entries are
 *   stale and must not be read.
 */
typedef struct {
    int   cols;
    int   rows;
    float sample[FIELD_H_MAX + 1][FIELD_W_MAX + 1];
} FieldGrid;

/*
 * Scene — owns all PERSISTENT simulation + UI state for one run.
 *
 * Intent
 *   One instance lives in main() and is passed by pointer to every
 *   simulation, draw, and input function.  No file-scope mutables
 *   for the simulation; signal-handler flags (g_quit, g_resize) in §7
 *   stay as globals because POSIX signal handlers can't be passed a
 *   pointer.
 *
 *   Scratch buffers (FieldGrid above) live OUTSIDE Scene — they
 *   are not persistent state, just per-frame intermediate.
 *
 * What lives here vs in FieldGrid
 *   Scene    = data that survives between frames (blob positions,
 *              threshold, paused flag, theme choice)
 *   FieldGrid = data that is fully recomputed every frame
 *
 * Members
 *   blobs[N_BLOBS]   moving metaball sources (the simulation state)
 *   rows, cols       terminal extent in cells (refreshed each frame
 *                    from ncurses' LINES/COLS so resize is picked up)
 *   threshold        current iso-value being extracted from the field
 *   paused           true → freeze blob motion (toggle: SPACE)
 *   multi            true → draw N_LEVELS contours at once (toggle: 'm')
 *   theme            colour theme index ∈ [0, N_THEMES) (cycle: 't')
 */
typedef struct {
    Blob   blobs[N_BLOBS];
    int    rows, cols;
    float  threshold;
    bool   paused;
    bool   multi;
    int    theme;
} Scene;

/* ===================================================================== */
/* §4  scalar field — blob motion + field sampling                        */
/* ===================================================================== */

/*
 * blobs_init — scatter N_BLOBS sources at random positions with
 * random velocities.  Called at startup and on 'r' (reset).
 *
 *   Positions:  uniform in [0.2, 0.8]² (margin keeps blobs visible
 *               on first frame, away from the bounce walls)
 *   Velocities: uniform magnitude in [0.003, 0.007], random sign per axis
 */
static void blobs_init(Scene *sc)
{
    for (int i = 0; i < N_BLOBS; i++) {
        sc->blobs[i].x        = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        sc->blobs[i].y        = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        sc->blobs[i].vx       = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        sc->blobs[i].vy       = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        sc->blobs[i].strength = 1.0f;
        if (rand() & 1) sc->blobs[i].vx = -sc->blobs[i].vx;
        if (rand() & 1) sc->blobs[i].vy = -sc->blobs[i].vy;
    }
}

/*
 * integrate_blob_euler — advance position by velocity for one tick.
 *
 *   Explicit Euler integration of a point mass:
 *       x_{t+1} := x_t + v · Δt           (Δt = 1 frame, implicit)
 *
 *   The kinematic primitive — every motion-of-mass-points solver
 *   reduces to this assignment.  No acceleration term: the blobs
 *   travel at constant velocity between wall reflections.
 */
static inline void integrate_blob_euler(Blob *b)
{
    b->x += b->vx;
    b->y += b->vy;
}

/*
 * bounce_blob_off_walls — elastic reflection against the unit box
 * [0.05, 0.95]² in normalised coords.
 *
 *   When a position component crosses a wall, the corresponding
 *   velocity component flips sign:
 *       if x ∉ [0.05, 0.95]:  vx ← −vx     (perfectly elastic)
 *       if y ∉ [0.05, 0.95]:  vy ← −vy
 *
 *   Simplest possible boundary condition for a particle in a box —
 *   no restitution coefficient, no friction, no position correction.
 *   Sufficient because the per-tick step (~0.005) is small relative
 *   to the box, so the position never overshoots the wall by much.
 */
static inline void bounce_blob_off_walls(Blob *b)
{
    if (b->x < 0.05f || b->x > 0.95f) b->vx = -b->vx;
    if (b->y < 0.05f || b->y > 0.95f) b->vy = -b->vy;
}

/*
 * blobs_step — advance the entire blob system by one frame.
 *
 *   Pseudocode:
 *     for each blob b:
 *       integrate_blob_euler(b)        ─ kinematic step (x += v)
 *       bounce_blob_off_walls(b)       ─ elastic reflection at the box
 */
static void blobs_step(Scene *sc)
{
    for (int i = 0; i < N_BLOBS; i++) {
        integrate_blob_euler(&sc->blobs[i]);
        bounce_blob_off_walls(&sc->blobs[i]);
    }
}

/*
 * wyvill_kernel — compact-support potential kernel.
 *
 *   W(r²) = (1 − r²/R²)³    for r² < R²,    else 0
 *
 *   Why this kernel:
 *     - EXACTLY zero outside R: clean cutoff, no faint tails on the
 *       contour, no need for "ignore values below ε" hacks.
 *     - Peaks at 1 when r = 0: bounded field sum ∈ [0, N_BLOBS],
 *       so threshold values fall into an easy [0, 1]-ish range.
 *     - C¹ at r = R: smooth contour at the kernel edge (no kink).
 *
 *   Takes r² (already squared) so the caller can skip a sqrt — the
 *   kernel never needs r itself, only r² compared against R².
 *
 *   Reference: Wyvill, McPheeters & Wyvill (1986), "Data structure
 *   for soft objects", The Visual Computer 2(4), pp. 227-234.
 */
static inline float wyvill_kernel(float r2, float R2)
{
    if (r2 >= R2) return 0.0f;
    float t = 1.0f - r2 / R2;
    return t * t * t;
}

/*
 * blob_contribution_at — ONE blob's contribution to the field at (nx, ny).
 *
 *   Pseudocode:
 *     dx   := nx − b.x
 *     dy   := (ny − b.y) · ASPECT          ← aspect-corrected so circles
 *                                            look circular on a 2:1 cell
 *     r²   := dx² + dy²                    ← squared distance, no sqrt
 *     return wyvill_kernel(r², BLOB_R²)
 *
 *   This is the only place where SCREEN ASPECT leaks into the field
 *   math.  Blobs live in normalised [0,1]² where pixel cells are 2:1
 *   tall; multiplying dy by ASPECT (=0.5) compresses the field
 *   vertically so a Euclidean "circle" of constant r in the (dx, dy/2)
 *   space looks circular on screen.
 */
static inline float blob_contribution_at(const Blob *b, float nx, float ny)
{
    float dx = nx - b->x;
    float dy = (ny - b->y) * ASPECT;
    float r2 = dx*dx + dy*dy;
    return wyvill_kernel(r2, BLOB_R * BLOB_R);
}

/*
 * field_eval_at — superpose all blob contributions at (nx, ny).
 *
 *   f(nx, ny) = Σ_i  blob_contribution_at(blob_i, nx, ny)
 *
 *   The scalar field IS a sum of per-source contributions; this
 *   function is one for-loop expressing that sum.  PURE — depends
 *   only on its arguments.
 *
 *   Called once per grid corner per frame (~ W·H times); the inner
 *   helpers are marked inline so the loop body collapses to roughly
 *   the same assembly the original open-coded version produced.
 */
static float field_eval_at(const Blob *blobs, int n, float nx, float ny)
{
    float v = 0.0f;
    for (int i = 0; i < n; i++)
        v += blob_contribution_at(&blobs[i], nx, ny);
    return v;
}

/*
 * init_field_extent — decide the per-frame field dimensions.
 *
 *   Pseudocode:
 *     cols := sc->cols                                ← terminal width
 *     rows := sc->rows − HUD_TOP_ROWS − HUD_BOT_ROWS  ← reserve HUD bars
 *     clamp cols to FIELD_W_MAX                       ← scratch buf cap
 *     clamp rows to FIELD_H_MAX
 *     if too small to march: zero the extent and bail
 *     else: write back into field->{cols, rows}
 *
 *   Two clamps in one place: HUD reservation is the ALGORITHM side
 *   (we need space for the status bars); FIELD_*_MAX is the STORAGE
 *   side (the scratch sample buffer is fixed-size).  Returning
 *   zero-extent makes downstream march_cells trivially skip.
 */
static void init_field_extent(const Scene *sc, FieldGrid *field)
{
    int cols = sc->cols;
    int rows = sc->rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
    if (cols > FIELD_W_MAX) cols = FIELD_W_MAX;
    if (rows > FIELD_H_MAX) rows = FIELD_H_MAX;
    if (cols < 2 || rows < 2) { field->cols = 0; field->rows = 0; return; }
    field->cols = cols;
    field->rows = rows;
}

/*
 * sample_all_corners — walk the (rows+1) × (cols+1) corner lattice,
 * storing f at each corner into field->sample[][].
 *
 *   Pseudocode:
 *     for gy in 0..rows:
 *       ny := gy / rows                        ← normalise to [0, 1]
 *       for gx in 0..cols:
 *         nx := gx / cols
 *         field->sample[gy][gx] := field_eval_at(blobs, nx, ny)
 *
 *   Normalised coords mean the grid spans the full unit box no matter
 *   the terminal size — resizing the window just re-buckets the same
 *   field.  Blob motion looks identical at any resolution.
 *
 *   This is the EXPENSIVE step of the frame — O(W·H·N_BLOBS) kernel
 *   evaluations.  Everything downstream (march_cells, draw_cell_*)
 *   reads field->sample[][] in O(1) per access.
 */
static void sample_all_corners(const Scene *sc, FieldGrid *field)
{
    for (int gy = 0; gy <= field->rows; gy++) {
        float ny = (float)gy / (float)field->rows;
        for (int gx = 0; gx <= field->cols; gx++) {
            float nx = (float)gx / (float)field->cols;
            field->sample[gy][gx] = field_eval_at(sc->blobs, N_BLOBS, nx, ny);
        }
    }
}

/*
 * field_sample_grid — orchestrator: pick dimensions, then sample.
 *
 *   Pseudocode:
 *     1. init_field_extent     ─ HUD-aware + storage-cap clamp
 *     2. early-exit if extent is zero (window too small)
 *     3. sample_all_corners    ─ the expensive O(W·H·N) lattice pass
 */
static void field_sample_grid(const Scene *sc, FieldGrid *field)
{
    init_field_extent(sc, field);
    if (field->cols == 0 || field->rows == 0) return;
    sample_all_corners(sc, field);
}

/* ===================================================================== */
/* §5  marching squares — 4-bit case classification + glyph lookup table  */
/* ===================================================================== */

/*
 * Corner-state encoding (the 4-bit cell case index)
 * ─────────────────────────────────────────────────
 *
 *   TL ─────── TR        bit 3 = (TL > threshold) ? 1 : 0
 *    │          │        bit 2 = (TR > threshold) ? 1 : 0
 *    │          │        bit 1 = (BR > threshold) ? 1 : 0
 *   BL ─────── BR        bit 0 = (BL > threshold) ? 1 : 0
 *
 *   "Inside" means corner_value > threshold.  The 4 bits pack into a
 *   single index ∈ [0, 15] — naming one of the 16 algorithm cases.
 *
 *   Trivial cases:  0 (0000, all outside)  and  15 (1111, all inside)
 *                   — no contour passes through the cell.
 *   Saddle cases:   5 (0101)  and  10 (1010) — diagonal in/out admits
 *                   TWO topologies; we pick one consistently below.
 *
 *   This encoding is the 2-D reduction of Lorensen & Cline's 8-bit
 *   cube-corner encoding (Marching Cubes, SIGGRAPH 1987).
 */

/*
 * case_glyph[16] — the algorithm's OUTPUT TABLE.
 *
 *   For each cell case index, the single ASCII glyph that best suggests
 *   the contour crossing through the cell.  This file draws ONE GLYPH
 *   PER CROSSED CELL rather than interpolating line endpoints — the
 *   glyph SHAPE encodes the edge-pair direction implicitly:
 *
 *     '-'   cuts left ↔ right          (horizontal stripe)
 *     '|'   cuts top  ↔ bottom         (vertical stripe)
 *     '/'   cuts top  ↔ left   OR  bottom ↔ right
 *     '\'   cuts top  ↔ right  OR  bottom ↔ left
 *     'X'   saddle (two crossings — single glyph for legibility)
 *     ' '   no crossing (cases 0, 15)
 *
 *   This is the "algorithm IS a table" formulation — every cell of
 *   the marching loop reduces to ONE indexed read into case_glyph[].
 *   A higher-fidelity renderer would instead store an EDGE-PAIR list
 *   per case and interpolate the crossing points along those edges
 *   (see raster/marching_cubes.c for the line-segment form).
 */
static const char case_glyph[16] = {
    ' ',    /* 0000 — all outside              — no contour              */
    '/',    /* 0001  BL inside                  — left + bottom edges    */
    '\\',   /* 0010  BR inside                  — bottom + right edges   */
    '-',    /* 0011  BL + BR inside             — left + right edges     */
    '\\',   /* 0100  TR inside                  — right + top edges      */
    'X',    /* 0101  BL + TR (saddle)           — two crossings          */
    '|',    /* 0110  TR + BR inside             — top + bottom edges     */
    '/',    /* 0111  all but TL                 — top + left edges       */
    '/',    /* 1000  TL inside                  — top + left edges       */
    '|',    /* 1001  TL + BL inside             — top + bottom edges     */
    'X',    /* 1010  TL + BR (saddle)           — two crossings          */
    '\\',   /* 1011  all but TR                 — right + top edges      */
    '-',    /* 1100  TL + TR inside             — left + right edges     */
    '\\',   /* 1101  all but BR                 — bottom + right edges   */
    '/',    /* 1110  all but BL                 — left + bottom edges    */
    ' ',    /* 1111 — all inside                — no contour             */
};

/*
 * CellCorners — the four scalar samples that classify ONE 2×2 cell.
 *
 *   The atomic input to the marching-squares case lookup: four field
 *   values at the cell's corners, in a fixed naming order that the
 *   case-index encoding (cell_case_index) depends on.
 *
 *     TL ─────── TR        bit 3 of the case index → TL inside?
 *      │          │        bit 2 of the case index → TR inside?
 *      │          │        bit 1 of the case index → BR inside?
 *     BL ─────── BR        bit 0 of the case index → BL inside?
 *
 *   Naming this group as a struct makes the per-cell data flow
 *   explicit:  read_cell_corners → cell_case_index → case_glyph[idx].
 *   Three named steps instead of one anonymous 4-float bundle.
 */
typedef struct { float tl, tr, br, bl; } CellCorners;

/*
 * read_cell_corners — extract the 2×2 sample window at cell (gx, gy).
 *
 *   Pseudocode:
 *     return ( sample[gy  ][gx  ],   ← TL
 *              sample[gy  ][gx+1],   ← TR
 *              sample[gy+1][gx+1],   ← BR
 *              sample[gy+1][gx  ] )  ← BL
 *
 *   Pure pointer-shuffle on the cached field grid.  The whole purpose
 *   is to LABEL the four reads with their geometric role (TL/TR/BR/BL)
 *   so the downstream classifier doesn't have to repeat the unpacking.
 */
static inline CellCorners read_cell_corners(const FieldGrid *field,
                                            int gx, int gy)
{
    return (CellCorners){
        .tl = field->sample[gy  ][gx  ],
        .tr = field->sample[gy  ][gx+1],
        .br = field->sample[gy+1][gx+1],
        .bl = field->sample[gy+1][gx  ],
    };
}

/*
 * cell_case_index — pack the 4 corner samples into the 4-bit case index.
 *
 *   Pseudocode:
 *     return ((tl > t) << 3) | ((tr > t) << 2) | ((br > t) << 1) | (bl > t)
 *
 *   The whole marching-squares algorithm flows through this one line:
 *   four comparisons, one bitwise OR, one table lookup.  Result ∈ [0, 15]
 *   indexes case_glyph[] (or any other 16-entry table you want to drive
 *   from the same classification — e.g. an edge-pair list for an
 *   interpolated-line renderer).
 */
static inline int cell_case_index(CellCorners c, float thresh)
{
    return ((c.tl > thresh) ? 8 : 0)
         | ((c.tr > thresh) ? 4 : 0)
         | ((c.br > thresh) ? 2 : 0)
         | ((c.bl > thresh) ? 1 : 0);
}

/* ===================================================================== */
/* §6  drawing — colour init + per-frame render pipeline                  */
/* ===================================================================== */

/*
 * Color-pair allocation
 * ─────────────────────
 *   CP_HUD / CP_HINT      fixed HUD pairs — bright yellow / bright cyan
 *                         per the project "HUD Standard" in CLAUDE.md.
 *                         Both are used with A_BOLD so the bars remain
 *                         legible against any field cell that intrudes.
 *   CP_CONTOUR / _INSIDE  primary theme colours for the contour line and
 *                         the "inside" fill in single-level mode.
 *   CP_OUTSIDE            unused background-grey pair, kept for symmetry.
 *   CP_LEVEL_BASE .. +N   the multi-mode contour colours, derived from
 *                         the active theme by hue shift.
 *   ALL pairs are initialised once per theme change in init_theme_colors —
 *   never inside the per-frame draw loop.
 *
 *   (The previous implementation re-init'd the multi-level pairs from
 *   inside the cell loop, doing thousands of redundant init_pair() calls
 *   per frame; the precompute here removes that hot-path overhead.)
 */
enum {
    CP_HUD        = 1,     /* top data bar       — bright yellow + A_BOLD */
    CP_HINT       = 2,     /* bottom action bar  — bright cyan   + A_BOLD */
    CP_CONTOUR    = 3,
    CP_INSIDE     = 4,
    CP_OUTSIDE    = 5,
    CP_LEVEL_BASE = 10,    /* CP_LEVEL_BASE + i  for i ∈ [0, N_LEVELS)    */
};

/*
 * Theme tables — one row per theme; columns are colour-cube indices
 *   theme_contour[t]   primary contour line colour for theme t
 *   theme_inside [t]   "inside" fill / accent colour for theme t
 *   Multi-mode hue shift: theme_contour[t] + lv * 6 for level lv.
 */
static const short theme_contour[N_THEMES] = { 51, 196, 46, 201 };
static const short theme_inside [N_THEMES] = { 87, 202, 82, 171 };

/*
 * init_theme_colors — set every colour pair for the chosen theme in
 * ONE call.  Includes the two HUD pairs (fixed across themes) and the
 * N_LEVELS multi-mode pairs.  Called at startup and on every 't'
 * (theme cycle); never inside the cell loop.
 */
static void init_theme_colors(int theme)
{
    init_pair(CP_HUD,     226,                  16);  /* bright yellow */
    init_pair(CP_HINT,     51,                  16);  /* bright cyan   */
    init_pair(CP_CONTOUR, theme_contour[theme], 16);
    init_pair(CP_INSIDE,  theme_inside [theme], 16);
    init_pair(CP_OUTSIDE, 244,                  16);
    for (int lv = 0; lv < N_LEVELS; lv++) {
        short col = (short)(theme_contour[theme] + lv * 6);
        init_pair((short)(CP_LEVEL_BASE + lv), col, 16);
    }
}

/*
 * draw_cell_single — paint one cell against a SINGLE iso-threshold.
 *
 *   Pseudocode:
 *     idx := cell_case_index(tl, tr, br, bl, threshold)
 *     if idx == 0:   blank (no contour, all outside)
 *     if idx == 15:  paint '.' in INSIDE colour (all inside fill)
 *     else:          paint case_glyph[idx] in CONTOUR colour, bold
 *
 *   This is the body of the marching-squares loop in single-level mode.
 */
static void draw_cell_single(int gx, int gy, CellCorners c, float threshold)
{
    int idx = cell_case_index(c, threshold);
    if (idx == 0)  return;                       /* fully outside — blank */
    if (idx == 15) {                             /* fully inside — fill   */
        attron(COLOR_PAIR(CP_INSIDE));
        mvaddch(gy, gx, '.');
        attroff(COLOR_PAIR(CP_INSIDE));
    } else {                                     /* contour crossing      */
        attron(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
        mvaddch(gy, gx, case_glyph[idx]);
        attroff(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
    }
}

/*
 * draw_cell_multi — paint one cell against N_LEVELS evenly-spaced
 * iso-thresholds, each rendered in its own colour.
 *
 *   Pseudocode:
 *     for lv in 0..N_LEVELS−1:
 *       t := base_threshold × (0.3 + lv × 0.15)
 *       idx := cell_case_index(tl, tr, br, bl, t)
 *       if idx ∉ {0, 15}:
 *         paint case_glyph[idx] in CP_LEVEL_BASE + lv
 *         return       ← first hit wins; deeper levels masked
 *     if no level produced a contour AND centre sample is inside:
 *       paint '`' in CP_INSIDE   (faint interior fill)
 *
 *   First-hit-wins lets the OUTERMOST contour (lowest threshold) take
 *   priority — same visual effect as back-to-front draw without
 *   overdrawing.
 */
static void draw_cell_multi(int gx, int gy, CellCorners c, float base_threshold)
{
    for (int lv = 0; lv < N_LEVELS; lv++) {
        float t = base_threshold * (0.3f + lv * 0.15f);
        int idx = cell_case_index(c, t);
        if (idx != 0 && idx != 15) {
            short cp = (short)(CP_LEVEL_BASE + lv);
            attron(COLOR_PAIR(cp));
            mvaddch(gy, gx, case_glyph[idx]);
            attroff(COLOR_PAIR(cp));
            return;
        }
    }
    /* No level produced a contour here — faint fill if it's inside the
       OUTERMOST contour.  Use the TL corner as a representative sample;
       all four corners straddle the threshold the same way when the
       cell didn't trigger any level. */
    if (c.tl > base_threshold) {
        attron(COLOR_PAIR(CP_INSIDE));
        mvaddch(gy, gx, '`');
        attroff(COLOR_PAIR(CP_INSIDE));
    }
}

/*
 * march_cells — visit every cell, dispatch to draw_cell_{single,multi}.
 *
 *   The "marching" is just two nested for-loops over the field grid;
 *   the per-cell work is delegated.  Single-level vs multi-level mode
 *   is selected once per cell (and could be hoisted out of the loop if
 *   the branch ever shows up in a profile — currently negligible).
 */
static void march_cells(const Scene *sc, const FieldGrid *field)
{
    for (int gy = 0; gy < field->rows; gy++) {
        for (int gx = 0; gx < field->cols; gx++) {
            CellCorners c   = read_cell_corners(field, gx, gy);
            int  screen_row = gy + HUD_TOP_ROWS;       /* slip below top HUD */
            if (sc->multi)
                draw_cell_multi (gx, screen_row, c, sc->threshold);
            else
                draw_cell_single(gx, screen_row, c, sc->threshold);
        }
    }
}

/*
 * draw_hud_top — row 0: DATA readout for the current frame.
 *
 *   Layout (left-aligned):
 *     " Marching Squares  thresh=0.20  mode:single  theme:0/4  running "
 *
 *   Bright yellow + A_BOLD per the project HUD standard (CLAUDE.md
 *   "HUD Standard").  Shows every value the user can change via the
 *   bottom action bar, so they can confirm key presses had effect.
 *   The paused state is shown HERE (was a separate top-right badge in
 *   the previous version).
 */
static void draw_hud_top(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " Marching Squares  thresh=%.2f  mode:%-6s  theme:%d/%d  %s ",
             sc->threshold,
             sc->multi ? "multi" : "single",
             sc->theme, N_THEMES,
             sc->paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_bottom — last row: ACTION key hints.
 *
 *   Bright cyan + A_BOLD.  Lists every key bound in handle_input; each
 *   key here changes a value visible in the top HUD.  The "spc" mnemonic
 *   for space matches the project convention (CLAUDE.md HUD Standard).
 */
static void draw_hud_bottom(const Scene *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  +/-:thresh  m:multi  t:theme  r:random ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * scene_draw — render one frame.
 *
 *   Pseudocode (the entire frame pipeline):
 *     1. field_sample_grid     ─ sample f at every grid corner (one pass)
 *     2. march_cells           ─ classify + paint each 2x2 cell (offset
 *                                by HUD_TOP_ROWS so it lands BELOW the
 *                                top HUD bar)
 *     3. draw_hud_top          ─ row 0: data readout (yellow)
 *     4. draw_hud_bottom       ─ last row: action hints (cyan)
 *     5. refresh               ─ flush ncurses output
 *
 *   Steps 1-2 are the marching algorithm proper; 3-4 are UI chrome;
 *   5 is the I/O boundary.
 */
static void scene_draw(const Scene *sc, FieldGrid *field)
{
    field_sample_grid(sc, field);
    march_cells(sc, field);
    draw_hud_top(sc);
    draw_hud_bottom(sc);
    refresh();
}

/* ===================================================================== */
/* §7  app — signal handlers, input dispatch, main loop                   */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static void on_signal(int s) { (void)s; g_quit   = 1; }
static void on_resize(int s) { (void)s; g_resize = 1; }

/*
 * handle_input — dispatch one keypress against the scene.
 *
 *   Pure delegation: every key maps to a small mutation of sc plus
 *   (for theme cycling) one re-initialisation of the colour pairs.
 *   Quit signals go through g_quit so SIGINT/SIGTERM take the same
 *   exit path as the 'q' key.
 */
static void handle_input(Scene *sc, int ch)
{
    switch (ch) {
        case 'q': case 27: g_quit = 1; break;
        case ' ': sc->paused = !sc->paused; break;
        case '+': case '=':
            sc->threshold += THRESH_STEP;
            if (sc->threshold > THRESH_MAX) sc->threshold = THRESH_MAX;
            break;
        case '-':
            sc->threshold -= THRESH_STEP;
            if (sc->threshold < THRESH_MIN) sc->threshold = THRESH_MIN;
            break;
        case 'm': sc->multi = !sc->multi; break;
        case 't':
            sc->theme = (sc->theme + 1) % N_THEMES;
            init_theme_colors(sc->theme);
            break;
        case 'r': blobs_init(sc); break;
    }
}

/*
 * main — own the Scene + FieldGrid scratch, drive the fixed-rate loop.
 *
 *   The render loop runs at RENDER_FPS (~20 fps).  When woken early, we
 *   sleep until the next frame deadline rather than busy-poll keypresses.
 *
 *   Why a STATIC FieldGrid in main (rather than embedded in Scene)
 *     - It's per-frame scratch (~128 KB), not persistent state.
 *     - BSS allocation: zero alloc cost, never touches the stack.
 *     - main is the canonical owner; the loop passes a pointer.
 */
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

    Scene scene = {
        .threshold = THRESH_DEF,
        .paused    = false,
        .multi     = false,
        .theme     = 0,
        .rows      = LINES,
        .cols      = COLS,
    };
    static FieldGrid field;          /* BSS — ~128 KB per-frame scratch */

    init_theme_colors(scene.theme);
    blobs_init(&scene);

    long long next = clock_ns();

    while (!g_quit) {
        if (g_resize) { g_resize = 0; endwin(); refresh(); }

        scene.rows = LINES;
        scene.cols = COLS;

        int ch = getch();
        handle_input(&scene, ch);

        long long now = clock_ns();
        if (now >= next) {
            if (!scene.paused) blobs_step(&scene);
            erase();
            scene_draw(&scene, &field);
            next += RENDER_NS;
        } else {
            clock_sleep_ns(next - now);
        }
    }

    endwin();
    return 0;
}
