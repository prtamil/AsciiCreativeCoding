/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wireframe.c  —  ncurses full-screen 3-D ASCII wireframe shapes
 *
 * Four shapes cycle with Tab:  cube · sphere · pyramid · torus
 *
 * The shape always fills ~80 % of the terminal regardless of size.
 * No fixed-resolution virtual canvas — everything is projected directly
 * into terminal-cell coordinates at runtime.
 *
 * Aspect-ratio fix:
 *   Terminal cells are ~2× taller than wide (in physical pixels).
 *   project() compensates by halving the Y component of the projected
 *   point so vertical distances in world space map to the same physical
 *   size on screen as horizontal distances.
 *
 * Keys:
 *   q / ESC   quit
 *   Tab       cycle shapes  (cube → sphere → pyramid → torus → …)
 *   space     pause / resume
 *   ]  [      spin faster / slower
 *   z / Z     zoom in / out
 *   t / T     next / previous colour theme
 *               (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/wireframe.c -o wireframe -lncurses
 * -lm
 *
 * Sections
 * --------
 *   §1  config    — tunables
 *   §2  clock     — monotonic ns clock + sleep
 *   §3  color     — one hue per shape
 *   §4  vec3      — 3-D math, value types
 *   §5  project   — rotation + perspective + aspect correction
 *   §6  canvas    — heap-allocated terminal-size framebuffer + Bresenham
 *   §7  shapes    — vertex/edge tables for all four shapes
 *   §8  scene     — active shape, rotation, tick, render, draw
 *   §9  screen    — single stdscr, ncurses internal double buffer
 *   §10 app       — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wireframe rendering via vertex transformation and line
 * drawing. For each edge: transform both endpoints, project to screen, draw a
 * line using Bresenham's algorithm. No Z-buffer: edges are drawn regardless of
 * depth (wireframe). Back-face culling: for each face, if the normal dotted
 * with the view direction is positive → facing away → don't draw.
 *
 * Math           : Perspective projection from 3D camera space to 2D screen:
 *                    col = (x / z) × f_x + cx
 *                    row = (y / z) × f_y + cy
 *                  where f_x, f_y are focal lengths and (cx, cy) is the
 *                  principal point (screen centre).  Larger f → more telephoto.
 *                  Rotation applied via 3×3 matrix from two Euler angles.
 *                  Aspect correction: y component scaled by CELL_W/CELL_H so
 *                  circles appear circular on non-square cells.
 *
 * Rendering      : Bresenham line drawing: integer-only incremental line
 * algorithm. No floating point in the inner loop — increments dx/dy by the
 * dominant axis direction, accumulates error, and steps the minor axis when
 * error exceeds 0.5.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A wireframe is just two tables — a vertex list and an edge list — plus
 * one pipeline that does NOTHING except "rotate every vertex, project to
 * 2D, draw a line between every (a, b) edge".  No fills, no z-buffer, no
 * shading.  All four shapes (cube, sphere, pyramid, torus) share the
 * SAME pipeline; the only thing that differs is what the vertex/edge
 * tables contain.  Curved shapes are not "curved" — they are dense
 * polygons whose edges happen to approximate a curve.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a constellation: stars at fixed positions in space, lines
 * connecting predetermined pairs.  Spin the whole constellation, snap a
 * photograph through a pinhole camera, ink the lines onto the photo.
 * Every shape in this gallery is one such constellation — the cube has
 * 8 stars and 12 lines; a 6×8 sphere has 48 stars and 88 lines.  The
 * pipeline never asks "is this a sphere?" — it just walks the edge
 * list.  That is why adding a new shape costs only a build_X function.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Each tick (paused or not, framerate-independent):
 *       rx += rot_x · dt,  ry += rot_y · dt   (Euler accumulators)
 *  2. Each frame:
 *       fov = fov_from_screen(cols, rows, zoom)  — picks fov_px so the
 *       unit-radius shape fills FILL=82% of the smaller dimension.
 *  3. For every vertex i: rotate by (rx, ry) using rot_yx, then project
 *       through perspective division (fov / (CAM_DIST - z)) into screen
 *       cell coordinates (col, row, z_world).
 *  4. For every edge {a, b}:
 *       skip if either endpoint is behind the camera (z_world too low),
 *       call canvas_line() with Bresenham's algorithm.
 *  5. canvas_line picks the glyph: curved shapes use 'o' everywhere
 *       (varying slope across an arc would look noisy);
 *       flat shapes pick by slope (-, |, /, \).
 *  6. canvas_draw walks the framebuffer once and emits each non-zero
 *       cell with the active shape's color pair + A_BOLD.
 *
 * KEY FORMULAS
 * ────────────
 *  Y rotation     x' =  x·cosry + z·sinry,    z' = -x·sinry + z·cosry
 *  X rotation     y' =  y·cosrx - z·sinrx,    z' =  y·sinrx + z·cosrx
 *  Perspective    scale = fov_px / (CAM_DIST - p.z)
 *                 col = ox + p.x · scale
 *                 row = oy - p.y · scale / CELL_AR    (aspect fix)
 *  Auto-fit       fov_rows = FILL · (rows/2) · CAM_DIST
 *                 fov_cols = FILL · (cols/2) · CAM_DIST / CELL_AR
 *                 fov_px   = min(fov_rows, fov_cols) · zoom
 *  Sphere param   v(st, sl) = (sin φ·cos θ, cos φ, sin φ·sin θ)
 *                 with φ = π·st/STACKS, θ = 2π·sl/SLICES
 *  Torus param    v(i, j) = ((R+r·cos θ)cos φ, r·sin θ, (R+r·cos θ)sin φ)
 *  Bresenham      err = dx + dy where dy is negated; step x when 2err≥dy,
 *                 step y when 2err≤dx; both axes share one error term
 *  Slope to glyph |slope| < 0.5 → '-',  < 2 → '/' or '\\',  else '|'
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • If a vertex passes behind the camera (denom = CAM_DIST - z ≤ 0.01)
 *    project_to_screen returns z = -9999 and the edge is skipped.  At
 *    extreme zoom this can hide a face entirely; the shape "blinks".
 *  • CELL_AR=2.0 is hard-coded.  On a square-cell terminal (some font
 *    settings) shapes will look squashed vertically.
 *  • SPHERE_STACKS/SLICES=6/8 is INTENTIONALLY low — at 12/16 the lines
 *    overlap on screen and the ball looks filled instead of wireframe.
 *  • Increasing torus MAJOR/MINOR raises edge count quadratically; over
 *    MAX_EDGES=800 silently truncates.
 *  • Tab between shapes resets rx/ry to (0.4, 0.6) — accumulators do
 *    NOT carry across shapes.
 *  • Resize calls canvas_free + canvas_alloc; if calloc fails the
 *    canvas is zeroed, which canvas_set then tolerates (NULL ptr deref
 *    avoided by bounds check on cols/rows already 0).
 *  • Bresenham draws in row-major; the corner pixel is shared between
 *    two edges and gets overwritten with the second edge's slope glyph.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Cube: count edges on screen — should be exactly 12 (4 front + 4
 *    back + 4 pillars).  Stop rotation with space and verify.
 *  • Sphere: count latitude rings — SPHERE_STACKS=6 means 5 visible
 *    rings (excluding poles).  Longitude lines: 8.
 *  • Pyramid: 8 edges (4 base + 4 lateral).  Apex sits above the centre
 *    of the base.
 *  • Torus: M·m·2 = 12·6·2 = 144 edges, each rendered as 'o' dots.
 *  • Resize the terminal — shape should always fill ~82% of the smaller
 *    dimension; if it drifts, fov_from_screen is broken.
 *  • Press = until ZOOM_MAX (3.5×): edges should stop scaling further.
 *  • Press ] until ROT_MAX caps at 8 rad/s (~76 RPM).  Beyond that key
 *    presses do nothing.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Wireframe is the SIMPLEST raster pipeline in this
 *      folder (no z-buffer, no triangle fill, no shading); read it
 *      first if you're new to the rasterisation family.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §5 project + §6 canvas — the two-step pipeline. Read AFTER
 *      tutorials T2, T3.
 *   4. §7 shapes — pure data tables (vertex + edge arrays for cube,
 *      sphere, pyramid, torus). Skim. Compare construction to
 *      tutorial T5.
 *   5. §8 scene + §10 app — orchestration; skip on a first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Vec3                 a 3-D point or vector by value
 *   rx / ry              euler accumulators around X / Y axes
 *   fov_px               focal length IN CELLS (after auto-fit)
 *   col / row            screen cell coords (top-left origin)
 *   x0/y0/x1/y1          line endpoints in cell coords (Bresenham)
 *   STACKS / SLICES      sphere tessellation grid (lat × lon)
 *   MAJOR / MINOR        torus tessellation grid (around-ring × around-tube)
 *
 * Background you need
 * ───────────────────
 *   - Basic 3-D rotation (sin/cos around an axis).
 *   - Perspective projection in one sentence: "divide x and y by z."
 *   - Bresenham's line algorithm (or willingness to learn from T3).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Triangle rasterisation (cube_raster.c covers that).
 *   - Z-buffer (no hidden surface removal here — every edge draws).
 *   - Lighting / shading models (wireframe is monochrome per shape).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Seven short tutorials that build the wireframe renderer from first
 * principles. Read in order; each builds on the previous.
 *
 *   T1  The wireframe pipeline — vertex transform → project → line
 *   T2  Perspective projection — divide by z, plus aspect correction
 *   T3  Bresenham's line algorithm — integer-only line drawing
 *   T4  Auto-fit FOV — making the shape fill ~82% regardless of size
 *   T5  Parametric tessellation — sphere & torus as 2-D vertex grids
 *   T6  Curve approximation — why a "sphere" is just dense polygons
 *   T7  Slope-to-glyph — picking '─', '│', '/', '\\' from line direction
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE WIREFRAME PIPELINE — VERTEX TRANSFORM → PROJECT → LINE
 * ───────────────────────────────────────────────────────────────
 * Every wireframe renderer ever written runs the same three-stage
 * pipeline. Naming the stages explicitly helps you read any
 * wireframe code in any language:
 *
 *   STAGE 1  TRANSFORM    rotate every vertex by the current Euler
 *                         angles (rx, ry). Output: vertex in CAMERA
 *                         space.
 *   STAGE 2  PROJECT      divide x and y by z (the perspective
 *                         transform), scale by focal length, offset
 *                         to screen centre. Output: 2-D cell coords.
 *   STAGE 3  RASTER       for each edge {a, b} in the edge table,
 *                         draw a line from projected[a] to projected[b]
 *                         using Bresenham (T3) — output: glyph in cell.
 *
 * Pseudocode of one frame:
 *
 *   foreach vertex v_i:
 *     v_rot = rotate(v_i, rx, ry)             stage 1
 *     screen[i] = project(v_rot, fov_px)       stage 2
 *
 *   foreach edge {a, b} in edge_table:
 *     if either endpoint is behind camera: skip
 *     bresenham(screen[a], screen[b])           stage 3
 *
 * No z-buffer, no triangle fill, no shading. The "solid" appearance
 * comes from the EYE filling in the gaps between many close edges
 * (a sphere with 88 edges reads as a sphere even though every cell
 * between the lines is empty).
 *
 * Read §5 project + §6 canvas — that's the entire pipeline, ~50 lines.
 *
 * T2  PERSPECTIVE PROJECTION — DIVIDE BY Z, PLUS ASPECT CORRECTION
 * ────────────────────────────────────────────────────────────────
 * To project a 3-D point (x, y, z) onto a 2-D screen, do this:
 *
 *     col = ox + x · (fov_px / depth)
 *     row = oy - y · (fov_px / depth) / CELL_AR
 *
 * where:
 *   depth   = CAM_DIST - z   (camera at +Z looking toward origin)
 *   fov_px  = focal length in screen cells
 *   ox, oy  = screen centre (cols/2, rows/2)
 *   CELL_AR = cell height / cell width (≈ 2.0 on most terminals)
 *
 * The "divide by depth" is the entire perspective effect. Distant
 * vertices (large depth) get small (col, row) offsets — they cluster
 * near the centre. Near vertices get LARGE offsets — they spread
 * outward. That's why a cube looks like a cube and not a hex.
 *
 * The aspect correction `/ CELL_AR` is critical for ASCII rendering.
 * Terminal cells are ~2× taller than wide. Without correction, a
 * sphere would render as a vertically-squashed oval. Dividing the
 * row offset by 2 stretches it back so circles read as circles.
 *
 * Negation on the row term: screen y goes DOWN; world y goes UP. The
 * minus sign converts world-up to screen-down.
 *
 * Worked example:
 *   v = (0.5, 0.5, 0)   on a 60×80 terminal, fov_px = 30
 *   depth = CAM_DIST - 0 = 3.0
 *   col   = 30 + 0.5 · (30/3)    = 30 + 5 = 35
 *   row   = 15 - 0.5 · (30/3)/2  = 15 - 2.5 = 12.5 → 12
 *   → renders at (35, 12), upper-right quadrant ✓
 *
 * Read §5 project_to_screen for the implementation.
 *
 * T3  BRESENHAM'S LINE ALGORITHM — INTEGER-ONLY LINE DRAWING
 * ───────────────────────────────────────────────────────────
 * Goal: given two screen-cell endpoints (x0, y0) and (x1, y1), light
 * up every cell along the line between them. Naïvely:
 *
 *   for x = x0 .. x1:
 *     y = y0 + (y1 - y0) · (x - x0) / (x1 - x0)    floating point!
 *     paint(x, round(y))
 *
 * Bresenham (1962) eliminates the floating point. It uses a single
 * INTEGER ERROR ACCUMULATOR that tracks how far the "true" line
 * deviates from the cells we've actually painted; when the error
 * exceeds half a cell, step the minor axis and reset.
 *
 * Pseudocode (matches §6 canvas_line):
 *
 *   dx = |x1 - x0|;  sx = (x0 < x1) ? +1 : -1
 *   dy = -|y1 - y0|; sy = (y0 < y1) ? +1 : -1   (note: dy negated)
 *   err = dx + dy
 *   loop:
 *     paint(x0, y0)
 *     if x0 == x1 and y0 == y1: break
 *     e2 = 2 · err
 *     if e2 >= dy:    err += dy;  x0 += sx        step in x
 *     if e2 <= dx:    err += dx;  y0 += sy        step in y
 *
 * The trick: `err += dy` (negative) when stepping x, `err += dx`
 * (positive) when stepping y. The error oscillates around zero,
 * which is the geometric distance from the true line to the painted
 * cells. Both axes can step in the same iteration on diagonals.
 *
 * Why Bresenham over float interpolation:
 *   - Integer ops only — fast, no rounding error.
 *   - Identical results regardless of CPU float behaviour.
 *   - The standard "rasterise a line" algorithm in every graphics
 *     library since 1962.
 *
 * T4  AUTO-FIT FOV — MAKING THE SHAPE FILL ~82% REGARDLESS OF SIZE
 * ────────────────────────────────────────────────────────────────
 * The shape is unit-radius (centred at origin, max distance from
 * origin = 1). We want it to fill FILL=0.82 of the SMALLER terminal
 * dimension regardless of whether the user has a 40×80 or 200×600
 * terminal.
 *
 * Solve for fov_px (focal length in cells) such that a vertex at
 * radius 1 in world space projects to FILL · (rows/2) cells from
 * centre:
 *
 *     FILL · (rows/2) = 1 · fov_px / CAM_DIST
 *     fov_rows        = FILL · (rows/2) · CAM_DIST
 *
 * Same calculation for cols, with aspect correction:
 *
 *     fov_cols = FILL · (cols/2) · CAM_DIST / CELL_AR
 *
 * Pick the smaller — that's the dimension that constrains us:
 *
 *     fov_px = min(fov_rows, fov_cols) · zoom
 *
 * Recomputed each frame so resize and zoom both work seamlessly.
 *
 * T5  PARAMETRIC TESSELLATION — SPHERE & TORUS AS 2-D VERTEX GRIDS
 * ────────────────────────────────────────────────────────────────
 * Cube and pyramid have hand-listed vertex tables (8 corners, 5
 * vertices). Sphere and torus are TESSELLATED — built procedurally
 * from a 2-D parameter grid:
 *
 *   SPHERE  parameters (st, sl) ∈ [0, STACKS] × [0, SLICES]
 *           φ (latitude)  = π · st / STACKS
 *           θ (longitude) = 2π · sl / SLICES
 *           v(st, sl) = (sin φ · cos θ, cos φ, sin φ · sin θ)
 *           Edges: vertical "longitude lines" + horizontal "latitude
 *           rings". Two stacks share an edge → ring; two slices
 *           share an edge → meridian.
 *
 *   TORUS   parameters (i, j) ∈ [0, MAJOR] × [0, MINOR]
 *           φ = 2π · i / MAJOR    (around the donut hole)
 *           θ = 2π · j / MINOR    (around the tube)
 *           v(i, j) = ((R + r·cos θ) cos φ, r·sin θ, (R + r·cos θ) sin φ)
 *           Edges: 2-D wraparound grid — every (i, j) connects to
 *           (i+1, j) and (i, j+1), with both indices wrapping mod
 *           their respective dimensions.
 *
 * The 2-D grid is the KEY abstraction. Both sphere and torus are
 * parameterised by two angles → 2-D grid → vertex array via lookup
 * → edge array via "neighbour pairs in the grid."
 *
 * SPHERE_STACKS = 6, SPHERE_SLICES = 8 → 7×9 = 63 vertices, ~88 edges.
 * TORUS  MAJOR = 12, MINOR = 6           → 12×6 = 72 vertices, ~144 edges.
 *
 * T6  CURVE APPROXIMATION — WHY A "SPHERE" IS JUST DENSE POLYGONS
 * ───────────────────────────────────────────────────────────────
 * No real spheres or torii in this file. Both are POLYGONAL
 * APPROXIMATIONS — a sphere with STACKS·SLICES edge cells, a torus
 * with MAJOR·MINOR. Larger numbers → smoother appearance, but more
 * edges to render.
 *
 * SPHERE_STACKS = 6 is INTENTIONALLY low. At 12 stacks the latitude
 * rings are dense enough that they overlap on screen and the ball
 * looks SOLID — defeating the wireframe aesthetic. The eye should
 * see distinct rings; that requires sparse-enough tessellation.
 *
 * Trade-off:
 *   too sparse (e.g. 4×6)  → visibly hexagonal silhouette, not round
 *   too dense (e.g. 16×24) → solid filled ball, no wireframe feel
 *   sweet spot (6×8)        → readable as sphere, lines stay distinct
 *
 * Same logic for torus: MAJOR=12, MINOR=6 gives 144 edges that
 * collectively suggest a donut without filling it.
 *
 * T7  SLOPE-TO-GLYPH — PICKING '─', '│', '/', '\\' FROM LINE DIRECTION
 * ───────────────────────────────────────────────────────────────────
 * For flat shapes (cube, pyramid), edges have a clear direction —
 * mostly horizontal, vertical, or diagonal. We pick the glyph that
 * matches:
 *
 *   |slope| < 0.5     mostly horizontal → '─'
 *   |slope| < 2       diagonal          → '/' or '\\' (sign of slope)
 *   else              mostly vertical   → '│'
 *
 * Slope = (dy · CELL_AR) / dx — the CELL_AR factor accounts for the
 * non-square cells (a 1:1 line in world space is steeper in cell
 * space because cells are tall).
 *
 * For curved shapes (sphere, torus), slope changes continuously
 * along an edge — picking glyph by slope would produce noisy mix of
 * '/', '\\', '|'. So we use 'o' EVERYWHERE on curved shapes,
 * sacrificing slope-information for visual consistency.
 *
 * Read §6 canvas_line() for the implementation — slope check at the
 * top, paint loop below.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 46,
  FPS_UPDATE_MS = 500,

  MAX_VERTS = 400,
  MAX_EDGES = 800,
  SHAPE_COUNT = 4,
};

/*
 * CAM_DIST  — camera distance along +Z.  The shape sits at the origin.
 * FILL      — fraction of the smaller screen dimension the shape fills.
 *             At 0.82 the shape just fits with a small margin.
 * CELL_AR   — physical cell height / cell width.
 *             Standard terminals: ~2.0.  Adjust if shape looks stretched.
 */
#define CAM_DIST 5.0f
#define FILL 0.82f
#define CELL_AR 2.0f /* cell aspect ratio: height / width        */

#define ROT_X_DEF 0.50f /* radians/sec                              */
#define ROT_Y_DEF 0.85f
#define ROT_STEP 1.35f
#define ROT_MIN 0.01f
#define ROT_MAX 8.0f

#define ZOOM_DEFAULT 1.0f
#define ZOOM_STEP 1.15f
#define ZOOM_MIN 0.4f
#define ZOOM_MAX 3.5f

/* Tessellation resolution — kept LOW intentionally.
 * A wireframe needs just enough lines to read as the shape.
 * Too many edges overlap at terminal resolution → looks filled/shaded.
 * Rule of thumb: circumference_in_chars / edges_per_ring >= 4 chars gap. */
#define SPHERE_STACKS 6 /* 5 visible latitude rings                */
#define SPHERE_SLICES 8 /* 8 longitude lines                       */
#define TORUS_MAJOR 12  /* 12 ring circles                         */
#define TORUS_MINOR 6   /* 6 tube circles                          */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color  — themes (4 shape pairs) + HUD/hint pairs                   */
/* ===================================================================== */

/*
 * Each shape gets one ncurses color pair (1..4).  A theme provides
 * those 4 colours; theme_apply re-inits the pairs.  HUD + HINT pairs
 * (5, 6) are theme-INDEPENDENT — yellow + cyan, both used with
 * A_BOLD per the CLAUDE.md HUD spec.
 *
 * 8-color fallback uses the original four basic-8 hues regardless
 * of theme (basic-8 has nowhere near the variety needed for themes).
 */
typedef enum {
  COL_CUBE = 1,
  COL_SPHERE = 2,
  COL_PYRAMID = 3,
  COL_TORUS = 4,
  PAIR_HUD = 5,
  PAIR_HINT = 6,
} ShapeColor;

#define THEME_COUNT 6

typedef struct {
  const char *display_name;
  /* one 256-colour code per shape: cube, sphere, pyramid, torus */
  short shape_256[4];
} Theme;

static const Theme THEMES[THEME_COUNT] = {
    /* CLASSIC — original four hues (cyan / green / yellow / magenta). */
    {"CLASSIC ", {51, 46, 226, 201}},
    /* AMBER   — warm phosphor monitor: bronze → gold → orange → amber. */
    {"AMBER   ", {130, 178, 208, 220}},
    /* MATRIX  — green data-stream: moss → emerald → lime → highlight. */
    {"MATRIX  ", {28, 46, 82, 154}},
    /* NEON    — synthwave magenta/pink ramp. */
    {"NEON    ", {165, 201, 207, 213}},
    /* ICE     — blues from teal → cyan → sky. */
    {"ICE     ", {39, 51, 87, 159}},
    /* COPPER  — bronze → copper-orange → gold. */
    {"COPPER  ", {130, 166, 208, 214}},
};

static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *t = &THEMES[theme_index];

  if (COLORS >= 256) {
    init_pair(COL_CUBE, t->shape_256[0], COLOR_BLACK);
    init_pair(COL_SPHERE, t->shape_256[1], COLOR_BLACK);
    init_pair(COL_PYRAMID, t->shape_256[2], COLOR_BLACK);
    init_pair(COL_TORUS, t->shape_256[3], COLOR_BLACK);
  } else {
    /* 8-colour fallback — basic hues, theme-independent. */
    init_pair(COL_CUBE, COLOR_CYAN, COLOR_BLACK);
    init_pair(COL_SPHERE, COLOR_GREEN, COLOR_BLACK);
    init_pair(COL_PYRAMID, COLOR_YELLOW, COLOR_BLACK);
    init_pair(COL_TORUS, COLOR_MAGENTA, COLOR_BLACK);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }

  theme_apply(0); /* default to CLASSIC */
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}

/* ===================================================================== */
/* §5  project                                                            */
/* ===================================================================== */

/*
 * rot_yx() — rotate p:  first Y by ry, then X by rx.
 */
static Vec3 rot_yx(Vec3 p, float rx, float ry) {
  float cy = cosf(ry), sy = sinf(ry);
  float x1 = p.x * cy + p.z * sy;
  float z1 = -p.x * sy + p.z * cy;
  p.x = x1;
  p.z = z1;

  float cx = cosf(rx), sx = sinf(rx);
  float y2 = p.y * cx - p.z * sx;
  float z2 = p.y * sx + p.z * cx;
  p.y = y2;
  p.z = z2;

  return p;
}

/*
 * project_to_screen() — perspective-project one 3-D world point to
 * terminal cell coordinates (col, row).
 *
 *   fov_px   half-height of the viewport in terminal rows, computed once
 *            per frame from the terminal size and the FILL constant so
 *            the shape always occupies FILL fraction of the screen.
 *
 *   ox, oy   screen origin (centre of terminal in cell coordinates).
 *
 * The shape vertex sits in world space scaled to unit size (radius ≈ 1).
 * Perspective division by (CAM_DIST - p.z) maps world units to pixels.
 * CELL_AR divides the Y result: terminal rows are taller than cols, so
 * without correction the shape would be stretched vertically.
 *
 * Returns (col, row, z_world).  If z_world is negative (behind cam) the
 * caller skips the edge.
 */
typedef struct {
  float col, row, z;
} P2;

static P2 project_to_screen(Vec3 p, float fov_px, float ox, float oy) {
  float denom = CAM_DIST - p.z;
  if (denom < 0.01f)
    return (P2){-1, -1, -9999};

  float scale = fov_px / denom;

  float col = ox + p.x * scale;
  float row = oy - p.y * scale / CELL_AR; /* /CELL_AR = aspect fix */

  return (P2){col, row, p.z};
}

/*
 * fov_from_screen() — compute the fov_px that makes the shape fill FILL
 * fraction of the screen.
 *
 * The shape has unit radius 1.  At distance CAM_DIST the projected radius
 * in pixels (rows) is:  fov_px / CAM_DIST * 1.
 * We want that to equal  FILL * (rows/2).
 * So: fov_px = FILL * (rows/2) * CAM_DIST.
 *
 * We also take the column dimension into account (accounting for CELL_AR)
 * and use the smaller of the two so the shape fits in both dimensions.
 */
static float fov_from_screen(int cols, int rows, float zoom) {
  float fov_rows = FILL * (float)rows * 0.5f * CAM_DIST;
  /* columns are CELL_AR times narrower visually than rows */
  float fov_cols = FILL * (float)cols * 0.5f * CAM_DIST / CELL_AR;
  float fov = fov_rows < fov_cols ? fov_rows : fov_cols;
  return fov * zoom;
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

/*
 * Canvas — heap-allocated terminal-size character grid.
 * Sized to exact terminal dimensions at runtime so shapes always fill
 * the whole screen with no fixed-resolution limitation.
 *
 * Each cell stores: ch (0 = empty) and ShapeColor.
 * canvas_draw() writes non-empty cells to the ncurses WINDOW.
 */
typedef struct {
  char *ch;        /* [rows * cols]                                  */
  ShapeColor *col; /* [rows * cols]                                  */
  int cols;
  int rows;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->cols = cols;
  c->rows = rows;
  c->ch = calloc((size_t)(cols * rows), sizeof(char));
  c->col = calloc((size_t)(cols * rows), sizeof(ShapeColor));
}

static void canvas_free(Canvas *c) {
  free(c->ch);
  free(c->col);
  *c = (Canvas){0};
}

static void canvas_clear(Canvas *c) {
  memset(c->ch, 0, sizeof(char) * (size_t)(c->cols * c->rows));
  memset(c->col, 0, sizeof(ShapeColor) * (size_t)(c->cols * c->rows));
}

static void canvas_set(Canvas *c, int x, int y, char ch, ShapeColor col) {
  if (x < 0 || x >= c->cols || y < 0 || y >= c->rows)
    return;
  int i = y * c->cols + x;
  c->ch[i] = ch;
  c->col[i] = col;
}

/*
 * canvas_line() — Bresenham integer line, all eight octants.
 *
 * ch_override:
 *   0   — pick char by slope: '-' '|' '/' '\' for straight edges
 *   'o' — draw 'o' at every pixel (for curved shapes like sphere/torus)
 *
 * Straight-edge shapes (cube, pyramid) use slope chars so the edges
 * read as clean geometric lines.  Curved shapes use a uniform dot so
 * the varying slope along each arc doesn't produce visual noise.
 */
static void canvas_line(Canvas *c, int x0, int y0, int x1, int y1,
                        char ch_override, ShapeColor col) {
  int dx = abs(x1 - x0), dy = -abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  /* Slope-based character (used only when ch_override == 0). */
  char slope_ch;
  if (ch_override != 0) {
    slope_ch = ch_override;
  } else {
    int adx = abs(x1 - x0), ady = abs(y1 - y0);
    if (adx == 0)
      slope_ch = '|';
    else if (ady == 0)
      slope_ch = '-';
    else {
      float slope = (float)ady / (float)adx;
      if (slope < 0.5f)
        slope_ch = '-';
      else if (slope < 2.0f)
        slope_ch = (sx == sy) ? '\\' : '/';
      else
        slope_ch = '|';
    }
  }

  for (;;) {
    canvas_set(c, x0, y0, slope_ch, col);
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

static void canvas_draw(const Canvas *c) {
  int total = c->cols * c->rows;
  for (int i = 0; i < total; i++) {
    char ch = c->ch[i];
    if (!ch)
      continue;

    int y = i / c->cols;
    int x = i % c->cols;

    attr_t attr = COLOR_PAIR(c->col[i]) | A_BOLD;
    attron(attr);
    mvaddch(y, x, (chtype)(unsigned char)ch);
    attroff(attr);
  }
}

/* ===================================================================== */
/* §7  shapes                                                             */
/* ===================================================================== */

typedef struct {
  int a, b;
} Edge;

typedef struct {
  const char *name;
  int nv;
  int ne;
  Vec3 verts[MAX_VERTS];
  Edge edges[MAX_EDGES];
  ShapeColor color;
  bool curved; /* true = dot rendering; false = line chars  */
} Shape;

/* ---- cube ---- */
static void shape_build_cube(Shape *s) {
  s->name = "cube";
  s->color = COL_CUBE;
  s->curved = false;
  s->nv = 8;
  s->ne = 12;

  for (int i = 0; i < 8; i++)
    s->verts[i] =
        v3((i & 1) ? 1.f : -1.f, (i & 2) ? 1.f : -1.f, (i & 4) ? 1.f : -1.f);

  Edge e[] = {
      {0, 1}, {1, 3}, {3, 2}, {2, 0}, /* front face  */
      {4, 5}, {5, 7}, {7, 6}, {6, 4}, /* back face   */
      {0, 4}, {1, 5}, {2, 6}, {3, 7}  /* pillars     */
  };
  memcpy(s->edges, e, sizeof e);
}

/* ---- sphere ---- */
static void shape_build_sphere(Shape *s) {
  s->name = "sphere";
  s->color = COL_SPHERE;
  s->curved = true;
  s->nv = 0;
  s->ne = 0;

  int ST = SPHERE_STACKS, SL = SPHERE_SLICES;

  for (int st = 0; st <= ST; st++) {
    float phi = (float)M_PI * st / ST;
    for (int sl = 0; sl < SL; sl++) {
      float th = 2.f * (float)M_PI * sl / SL;
      s->verts[s->nv++] =
          v3(sinf(phi) * cosf(th), cosf(phi), sinf(phi) * sinf(th));
    }
  }
  /* longitude lines */
  for (int sl = 0; sl < SL; sl++)
    for (int st = 0; st < ST; st++)
      s->edges[s->ne++] = (Edge){st * SL + sl, (st + 1) * SL + sl};
  /* latitude rings */
  for (int st = 1; st < ST; st++)
    for (int sl = 0; sl < SL; sl++)
      s->edges[s->ne++] = (Edge){st * SL + sl, st * SL + (sl + 1) % SL};
}

/* ---- pyramid ---- */
static void shape_build_pyramid(Shape *s) {
  s->name = "pyramid";
  s->color = COL_PYRAMID;
  s->curved = false;
  s->nv = 5;
  s->ne = 8;

  s->verts[0] = v3(-1, -1, -1);
  s->verts[1] = v3(1, -1, -1);
  s->verts[2] = v3(1, -1, 1);
  s->verts[3] = v3(-1, -1, 1);
  s->verts[4] = v3(0, 1, 0);

  Edge e[] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 4}, {2, 4}, {3, 4}};
  memcpy(s->edges, e, sizeof e);
}

/* ---- torus ---- */
static void shape_build_torus(Shape *s) {
  s->name = "torus";
  s->color = COL_TORUS;
  s->curved = true;
  s->nv = 0;
  s->ne = 0;

  int M = TORUS_MAJOR, m = TORUS_MINOR;
  float R = 0.65f, r = 0.28f;

  for (int i = 0; i < M; i++) {
    float phi = 2.f * (float)M_PI * i / M;
    float cp = cosf(phi), sp = sinf(phi);
    for (int j = 0; j < m; j++) {
      float th = 2.f * (float)M_PI * j / m;
      s->verts[s->nv++] =
          v3((R + r * cosf(th)) * cp, r * sinf(th), (R + r * cosf(th)) * sp);
    }
  }
  for (int i = 0; i < M; i++)
    for (int j = 0; j < m; j++) {
      s->edges[s->ne++] = (Edge){i * m + j, i * m + (j + 1) % m};   /* tube  */
      s->edges[s->ne++] = (Edge){i * m + j, ((i + 1) % M) * m + j}; /* ring  */
    }
}

static const char *const k_names[SHAPE_COUNT] = {"cube", "sphere", "pyramid",
                                                 "torus"};

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct {
  Shape shapes[SHAPE_COUNT];
  Canvas canvas;
  int active;
  float rx, ry;
  float rot_x, rot_y;
  float zoom;      /* fov-based zoom (z/Z keys)                */
  int theme_index; /* index into THEMES[] (t/T keys)           */
  bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  shape_build_cube(&s->shapes[0]);
  shape_build_sphere(&s->shapes[1]);
  shape_build_pyramid(&s->shapes[2]);
  shape_build_torus(&s->shapes[3]);

  s->active = 0;
  s->rx = 0.4f;
  s->ry = 0.6f;
  s->rot_x = ROT_X_DEF;
  s->rot_y = ROT_Y_DEF;
  s->zoom = ZOOM_DEFAULT;
  s->theme_index = 0; /* CLASSIC */
  s->paused = false;

  canvas_alloc(&s->canvas, cols, rows);
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt_sec) {
  if (s->paused)
    return;
  s->rx += s->rot_x * dt_sec;
  s->ry += s->rot_y * dt_sec;
}

/*
 * scene_render() — the core pipeline.
 *
 * 1. Compute fov_px once for this frame so the shape fills the screen.
 * 2. Transform all vertices: scale to unit → rotate → project.
 * 3. Draw each edge as a Bresenham line with slope-based character.
 *
 * The shape vertices are in unit coordinates (max radius ≈ 1).
 * fov_from_screen() maps that unit radius to FILL * screen_half_height
 * terminal rows, so the shape always fills the desired fraction of the
 * screen regardless of terminal size.
 */
static void scene_render(Scene *s) {
  canvas_clear(&s->canvas);

  int cols = s->canvas.cols;
  int rows = s->canvas.rows;
  float ox = (float)cols * 0.5f;
  float oy = (float)rows * 0.5f;
  float fov = fov_from_screen(cols, rows, s->zoom);

  const Shape *sh = &s->shapes[s->active];

  /* Pre-project all vertices. */
  P2 proj[MAX_VERTS];
  for (int i = 0; i < sh->nv; i++) {
    Vec3 v = rot_yx(sh->verts[i], s->rx, s->ry);
    proj[i] = project_to_screen(v, fov, ox, oy);
  }

  /* Draw edges — slope chars for flat shapes, dots for curved. */
  char ch_override = sh->curved ? 'o' : 0;

  for (int e = 0; e < sh->ne; e++) {
    P2 pa = proj[sh->edges[e].a];
    P2 pb = proj[sh->edges[e].b];

    if (pa.z < -CAM_DIST + 0.1f || pb.z < -CAM_DIST + 0.1f)
      continue;

    canvas_line(&s->canvas, (int)(pa.col + 0.5f), (int)(pa.row + 0.5f),
                (int)(pb.col + 0.5f), (int)(pb.row + 0.5f), ch_override,
                sh->color);
  }
}

static void scene_draw(const Scene *s) { canvas_draw(&s->canvas); }

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct {
  int cols;
  int rows;
} Screen;

static void screen_init(Screen *s) {
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

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps) {
  erase();
  scene_draw(sc);

  /* Top row — yellow status (right-aligned) + title (left). */
  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  %-7s  spd:%.2f  zoom:%.2f  theme:%s  %s ", fps,
           k_names[sc->active], sc->rot_y, sc->zoom,
           THEMES[sc->theme_index].display_name,
           sc->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " WIREFRAME ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  Tab:shape  ]/[:spin  z/Z:zoom  t/T:theme ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {

  case 'q':
  case 'Q':
  case 27:
    return false;

  case '\t':
    s->active = (s->active + 1) % SHAPE_COUNT;
    s->rx = 0.4f;
    s->ry = 0.6f;
    break;

  case ' ':
    s->paused = !s->paused;
    break;

  case ']':
    s->rot_x *= ROT_STEP;
    s->rot_y *= ROT_STEP;
    if (s->rot_x > ROT_MAX)
      s->rot_x = ROT_MAX;
    if (s->rot_y > ROT_MAX)
      s->rot_y = ROT_MAX;
    break;
  case '[':
    s->rot_x /= ROT_STEP;
    s->rot_y /= ROT_STEP;
    if (s->rot_x < ROT_MIN)
      s->rot_x = ROT_MIN;
    if (s->rot_y < ROT_MIN)
      s->rot_y = ROT_MIN;
    break;

  case 'z':
    /* zoom IN — bigger fov_px, shape fills more of the screen */
    s->zoom *= ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    /* zoom OUT — smaller fov_px, shape shrinks */
    s->zoom /= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* ── resize ──────────────────────────────────────────────── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── dt ──────────────────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* ── sim accumulator ─────────────────────────────────────── */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* ── render ──────────────────────────────────────────────── */
    scene_render(&app->scene);

    /* ── HUD counter ─────────────────────────────────────────── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* ── draw + present ──────────────────────────────────────── */
    screen_draw(&app->screen, &app->scene, fps_display);
    screen_present();

    /* ── input ───────────────────────────────────────────────── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
