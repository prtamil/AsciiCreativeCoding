/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * geometry/visibility_polygon.c -- Visibility Polygon Simulator
 *
 * An observer glides along a Lissajous figure path inside a room full of
 * wall obstacles.  At each frame the exact set of cells visible from the
 * observer is computed via the angular-sweep algorithm and drawn with
 * distance-coded fill characters.
 *
 * -------------------------------------------------------------------------
 *  Section map
 * -------------------------------------------------------------------------
 *  S1  config       -- sizes, timing, aspect ratio, algorithm constants
 *  S2  clock        -- monotonic ns clock + sleep
 *  S3  color        -- fill / walls / observer / HUD color pairs
 *  S4  geometry     -- Wall, WallSpec, SweepHit, VisibilityPolygon types;
 *                      ray_hits_wall(), aspect-ratio helpers
 *  S5  visibility   -- compute_visibility(), cast_ray_in_direction(),
 *                      find_angle_sector(), cell_is_visible()
 *  S6  scene        -- WallSet/Observer/SimControls/Scene structs +
 *                      scene_init / scene_tick / random-wall generator
 *  S7  render       -- scene_addch helper, render_scene(), draw_hud()
 *  S8  screen       -- ncurses init / teardown
 *  S9  app          -- signals, input, main loop
 * -------------------------------------------------------------------------
 *
 * Keys:  SPACE pause/resume   r reset   +/- speed   q quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       algorithms/visibility_polygon.c -o vp -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Exact angular-sweep visibility polygon.  From observer
 *                 O, only WALL ENDPOINT directions matter — between
 *                 two consecutive endpoint angles the nearest wall
 *                 cannot change, so the visibility boundary is a
 *                 straight segment between two ray-wall hits.  We
 *                 cast 3 rays per endpoint (θ - ε, θ, θ + ε) so
 *                 corners on either side are handled correctly, then
 *                 sort all hits by angle and join them as a polygon.
 *
 * Math          : Ray-segment intersection via 2-D cross product:
 *                   denom = D × (B - A)
 *                   t     = (A - O) × (B - A) / denom
 *                   s     = (A - O) × D       / denom
 *                 Hit is valid iff t > 0 AND 0 ≤ s ≤ 1.
 *                 Pick the wall with smallest t.
 *
 * Aspect fix    : Terminal cells are 2:1 tall.  Geometry done in
 *                 "geo space" where y is multiplied by ASPECT_Y to
 *                 make distances metrically isotropic.
 *
 * Performance   : O(E log E) for sorting endpoint angles, plus
 *                 O(E · W) for ray-vs-all-walls intersection.  With
 *                 MAX_WALLS = 13 walls × 2 endpoints × 3 ε-bracket rays
 *                 = 78 rays per frame; well under 1 ms per frame.
 *
 * References
 * ──────────
 *   ── Original algorithm papers ───────────────────────────────────
 *   [1] Asano, T., Asano, T., Guibas, L., Hershberger, J. & Imai, H.
 *       (1986), "Visibility of disjoint polygons", Algorithmica
 *       1(1), pp. 49-63 — the foundational angular-sweep paper;
 *       casting rays only at wall endpoints originates here.
 *   [2] ElGindy, H. & Avis, D. (1981), "A linear algorithm for
 *       computing the visibility polygon from a point", J. Algorithms
 *       2(2), pp. 186-197 — the O(n) visibility-from-a-point algorithm
 *       for an observer inside a simple polygon.
 *   [3] Lee, D. T. (1983), "Visibility of a simple polygon", Computer
 *       Vision, Graphics, and Image Processing 22(2), pp. 207-221 —
 *       predecessor to El Gindy & Avis; the classic stack-based sweep.
 *   [4] Suri, S. & O'Rourke, J. (1986), "Worst-case optimal algorithms
 *       for constructing visibility polygons with holes", SoCG '86 —
 *       generalises angular sweep to polygons-with-holes (the case
 *       this file uses, with isolated wall obstacles).
 *
 *   ── Canonical textbooks ────────────────────────────────────────
 *   [5] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 15 (art gallery + visibility),
 *       Ch. 2 (line-segment intersection via plane sweep).  The
 *       standard graduate text.
 *   [6] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge — Ch. 8 covers visibility polygons with full C code.
 *       The most accessible implementation-oriented reference.
 *   [7] Goodman, J. E., O'Rourke, J. & Tóth, C. D. (eds., 2017),
 *       "Handbook of Discrete and Computational Geometry" (3rd ed.),
 *       CRC — Ch. 33 (visibility) collects every modern result in
 *       one place.  Reference, not introduction.
 *
 *   ── Ray-segment intersection primitive (§4 math) ───────────────
 *   [8] Ericson, C. (2005), "Real-Time Collision Detection", Morgan
 *       Kaufmann — §5.1 ray vs segment, §5.3 ray vs polygon.  The
 *       practical-implementation reference; explains numerical
 *       robustness concerns the textbooks skip.
 *   [9] Bourke, P. (1989), "Intersection point of two lines in 2D" —
 *       http://paulbourke.net/geometry/pointlineplane/ — the same
 *       cross-product derivation used in ray_hits_wall(), in 3
 *       short pages with worked examples.
 *
 *   ── Algorithm visualisation ────────────────────────────────────
 *  [10] Patel, A., "2D Visibility", Red Blob Games —
 *       https://www.redblobgames.com/articles/visibility/ —
 *       interactive WebGL walkthrough of this exact angular-sweep
 *       algorithm.  The most pedagogically clear treatment of the
 *       ε-offset corner trick that §5 uses.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stand somewhere in a room full of walls.  The set of cells you
 * can see (no wall in the way) is the VISIBILITY POLYGON.  Compute
 * it: sweep a ray 360° around the observer; at each angle, record
 * where the ray first hits a wall; connect those hit points and
 * fill the interior.  Cleverness: only WALL CORNERS matter — at
 * angles between corners, the polygon edge is a straight line from
 * hit-at-corner-A to hit-at-corner-B.  So we cast O(E) rays
 * instead of "infinitely many."
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *  1. Move observer along Lissajous path.
 *  2. Collect all wall-endpoint angles relative to observer.
 *  3. For each endpoint at angle θ, cast THREE rays:
 *     θ - ε, θ, θ + ε.  The ε-offset rays disambiguate corners.
 *  4. For each ray:
 *     For each wall, compute ray-segment intersection (t, s).
 *     Valid iff t > 0 AND 0 ≤ s ≤ 1.  Keep min-t hit.
 *  5. Sort hits by angle.  The polygon is the closed list of hits
 *     in angular order.
 *  6. Render: cell is visible iff angularly-bracketing hits give
 *     a wall-distance ≥ cell-distance to observer.
 *
 * KEY FORMULAS
 * ────────────
 *   Ray-segment intersection (2-D cross product):
 *     denom = D.x · (B.y - A.y) - D.y · (B.x - A.x)
 *     if denom == 0: parallel, no hit
 *     t = ((A.x - O.x)·(B.y - A.y) - (A.y - O.y)·(B.x - A.x)) / denom
 *     s = ((A.x - O.x)·D.y       - (A.y - O.y)·D.x      ) / denom
 *     hit valid iff t > 0 AND 0 ≤ s ≤ 1
 *
 *   Polygon area (shoelace):
 *     area = ½ · |Σ (x_i · y_{i+1} - x_{i+1} · y_i)|
 *
 *   Aspect: y_geo = y_cell · ASPECT_Y
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first as prose.
 *   2. §4 data types — Wall, WallSpec, SweepHit, VisibilityPolygon.
 *      Sub-structs are referenced by every later step; understanding
 *      their fields makes the rest read like pseudocode.
 *   3. §5 visibility — compute_visibility orchestrator (collect
 *      endpoint angles → cast every ray → sort by angle → shoelace
 *      area).  THE HEART of this file.  Each step is one named helper.
 *   4. §4 geometry — ray-vs-segment intersection primitive.
 *      The single most important math; everything else is bookkeeping.
 *   5. §6 scene — WallSet / Observer / SimControls / Scene structs
 *      + the tick / init / random-wall generator built on top of them.
 *   6. §7 render — five-pass painter (paint_visible_cells →
 *      draw_sweep_ray → paint_interior_walls → paint_bounding_box →
 *      paint_observer_glyph) + draw_hud (top status / bottom hint).
 *   7. §9 app — main loop with handle_resize / advance_fixed_timestep /
 *      app_handle_input / frame_render / wait_until_next_frame.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Almost everything lives in `Scene scene` on main()'s stack and
 *   threads through every function as `Scene *s` (mutators) or
 *   `const Scene *s` (observers).  Only the POSIX signal flags
 *   (g_quit, g_resize_pending) remain global; signal handlers cannot
 *   receive a context pointer.
 *
 *   s->walls.list[]            line-segment obstacles in cell coords.
 *   s->walls.count             number of valid entries in list[].
 *   s->walls.rand_specs[]      randomised-layout fractions (use_rand=true).
 *   s->observer.x_cells/y_cells  current eye position (Lissajous-driven).
 *   s->observer.lissajous_t      drift-path phase (radians).
 *   s->observer.sweep_angle_rad  independent '*' overlay ray angle.
 *   s->vis.hits[]              ray-hit list (sorted by angle).
 *   s->vis.n_hits              valid hit count.
 *   s->vis.visible_area_cells  shoelace area, cached for HUD.
 *   s->sim.paused / s->sim.fps user-controlled run parameters.
 *   s->scene_cols / s->scene_rows  drawable extent (term minus HUD_ROWS).
 *
 *   theta, eps                 sweep angle + ε disambiguator (local to §5).
 *   ASPECT_Y                   y-stretch factor for geo space (§4 helper).
 *
 * Background you need
 * ───────────────────
 *   - 2-D cross product as winding determinant
 *     (algorithms/convex_hull.c T2 if new).
 *   - Linear ray equation P(t) = O + t·D, segment Q(s) = A + s·(B−A).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Computational geometry beyond this single primitive.
 *   - 3-D visibility (BSP trees, portal rendering).  This is
 *     2-D only.  See algorithms/bsp_tree.c for the 3-D
 *     analogue's data structure.
 *   - Real-time shadow casting in graphics engines.
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

/* ===================================================================== */
/* S1  config                                                             */
/* ===================================================================== */

/* Simulation speed: physics steps per second (user-adjustable). */
#define SIM_FPS_DEFAULT   30
#define SIM_FPS_MIN        5
#define SIM_FPS_MAX       60

/* Render loop target: runs independently of physics at a fixed budget. */
#define TARGET_FPS        60
#define FPS_UPDATE_MS    500

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(fps)    (NS_PER_SEC / (fps))

/*
 * ASPECT_Y: cell pixel height / cell pixel width.
 * Standard terminals use 8x16-pixel cells => ASPECT_Y = 16/8 = 2.0.
 * Multiply cell-space y by ASPECT_Y before any distance/angle math.
 */
#define ASPECT_Y  2.0f

/*
 * HUD reserves ONE row at the top (data: fps, visibility, rays, walls,
 * paused-state) and ONE row at the bottom (action hints: key bindings).
 * Scene rendering occupies the rows in between.
 */
#define HUD_TOP_ROWS     1
#define HUD_BOTTOM_ROWS  1
#define HUD_ROWS         (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)

/*
 * Wall counts: 4 outer bounding-box walls + interior obstacles.
 * MAX_WALLS must equal BOUNDING_WALL_COUNT + INTERIOR_WALL_COUNT.
 */
#define BOUNDING_WALL_COUNT   4
#define INTERIOR_WALL_COUNT   9
#define MAX_WALLS            (BOUNDING_WALL_COUNT + INTERIOR_WALL_COUNT)  /* 13 */

/*
 * Visibility sweep limits.
 * Each of MAX_WALLS walls contributes 2 endpoints => MAX_SWEEP_ANGLES angles.
 * Each endpoint spawns 3 rays (angle-eps, angle, angle+eps) => MAX_SWEEP_RAYS.
 */
#define MAX_SWEEP_ANGLES  (MAX_WALLS * 2)        /* 26 distinct endpoint angles */
#define MAX_SWEEP_RAYS    (MAX_SWEEP_ANGLES * 3) /* 78 rays in the sweep        */

/*
 * RAY_T_MIN: ignore intersection parameters smaller than this.
 * Prevents a ray from hitting the wall it originates from due to floating-point.
 *
 * ENDPOINT_ANGLE_EPS_RAD: tiny angular bracket around each endpoint angle.
 * Large enough to numerically separate the two sides of a corner;
 * small enough not to skip a nearby endpoint angle.
 */
#define RAY_T_MIN              1e-4f
#define ENDPOINT_ANGLE_EPS_RAD 1e-5f

/*
 * Observer Lissajous path: traces x = cx + rx*sin(FREQ_X * t),
 *                                  y = cy + ry*sin(FREQ_Y * t + PHASE).
 * The 3:2 frequency ratio produces a classic 6-lobed figure.
 * Radii are kept small (12%, 10% of scene) so the observer stays in the
 * open central area and does not clip through interior walls.
 */
#define OBS_CENTER_X_FRAC    0.50f   /* scene-fraction: horizontal center     */
#define OBS_CENTER_Y_FRAC    0.50f   /* scene-fraction: vertical center       */
#define OBS_RADIUS_X_FRAC    0.12f   /* scene-fraction: horizontal amplitude  */
#define OBS_RADIUS_Y_FRAC    0.10f   /* scene-fraction: vertical amplitude    */
#define LISSAJOUS_FREQ_X     3.0f    /* x-oscillation frequency multiplier    */
#define LISSAJOUS_FREQ_Y     2.0f    /* y-oscillation frequency multiplier    */
#define LISSAJOUS_PHASE_RAD  0.7854f /* pi/4 phase offset between x and y    */
#define LISSAJOUS_SPEED      0.015f  /* radians of t advanced per tick        */

/*
 * Sweep ray animation: a single rotating ray visualises the scan concept.
 * SWEEP_PERIOD_TICKS: ticks to complete one full 360-degree rotation.
 * SWEEP_SPEED_RAD:    angle advanced per tick.
 */
#define SWEEP_PERIOD_TICKS  80
#define SWEEP_SPEED_RAD     ((float)(2.0 * M_PI / SWEEP_PERIOD_TICKS))

/*
 * Ray-hit sentinel: the iterative min-t reduction starts at +∞ and is
 * lowered by each valid hit.  RAY_NO_HIT_T_SENTINEL is a "definitely
 * larger than any real hit" value; RAY_VALID_HIT_T_LIMIT is the
 * post-loop test "any wall actually got hit?".
 */
#define RAY_NO_HIT_T_SENTINEL     1e30f
#define RAY_VALID_HIT_T_LIMIT     1e29f

/*
 * cell_is_visible() tolerances — chosen empirically so the polygon
 * boundary renders without flicker on integer-rounded cell positions.
 *   OBSERVER_OWN_CELL_RADIUS_SQ : cells this close to the observer
 *       are always visible (avoid 0/0 angle computation at the eye).
 *   DEGENERATE_SECTOR_SLACK_CELLS : extra cells of slack used when the
 *       polygon boundary segment is degenerate (parallel hits).
 *   BOUNDARY_VISIBILITY_TOLERANCE_CELLS : cells riding the polygon
 *       boundary itself still light up (cosmetic).
 */
#define OBSERVER_OWN_CELL_RADIUS_SQ            0.01f
#define DEGENERATE_SECTOR_SLACK_CELLS          1.0f
#define BOUNDARY_VISIBILITY_TOLERANCE_CELLS    0.8f

/*
 * Wall glyph picker — slope thresholds that classify each wall as
 *   "nearly horizontal" → '-'
 *   "nearly vertical"   → '|'
 *   "diagonal"          → '/' or '\\'
 * WALL_AXIS_RATIO_THRESHOLD compares the short axis to the long axis;
 * smaller value = more walls classified as orthogonal.
 */
#define WALL_AXIS_RATIO_THRESHOLD   0.4f

/*
 * Distance-falloff fill: as a cell's distance to the observer grows,
 * the fill glyph fades from '@' (dense ring) → 'o' → '+' → '.' → ' '.
 * Cutoffs are expressed as fractions of the scene diagonal in geo space.
 * Tweak to grow / shrink the visible "light cone".
 */
#define FILL_DIST_FRAC_DENSE_LIMIT   0.10f   /* '@' below this fraction      */
#define FILL_DIST_FRAC_NEAR_LIMIT    0.25f   /* 'o' below this fraction      */
#define FILL_DIST_FRAC_MID_LIMIT     0.45f   /* '+' below this fraction      */
#define FILL_DIST_FRAC_FAR_LIMIT     0.65f   /* '.' below this fraction      */
                                             /* ' '  beyond                  */

/* Distance-falloff colour band boundaries (CP_FILL_NEAR / MID / FAR). */
#define COLOR_DIST_FRAC_NEAR_LIMIT   0.30f
#define COLOR_DIST_FRAC_MID_LIMIT    0.60f

/*
 * Cell-center sub-pixel offset: rays should test the CENTRE of a cell,
 * not its top-left corner, so visibility decisions match what the user
 * sees painted in the middle of the glyph.
 */
#define CELL_CENTER_OFFSET   0.5f

/*
 * Simulation speed adjustment step: +/- keys move sim.fps by this much.
 * Five Hz is small enough to feel like a knob, large enough to make
 * difference visible within a couple of presses.
 */
#define SIM_FPS_STEP   5

/* ESC key code — POSIX terminals send 27 for both ESC and the start of
 * many escape sequences; we treat it as quit because nodelay+typeahead
 * mean we'd never actually see a multi-byte sequence in time. */
#define KEY_ESC   27

/*
 * Random-walls generator — anchor placement + length bounds + clamp
 * margins.  All in scene-fraction space [0, 1].  See
 * scene_generate_random_walls() for usage.
 */
#define OBS_SAFE_HALF_X            0.20f   /* horizontal half-extent of "no walls" zone */
#define OBS_SAFE_HALF_Y            0.18f   /* vertical   half-extent of "no walls" zone */
#define RAND_WALL_LEN_MIN          0.07f   /* shortest random wall, scene fraction      */
#define RAND_WALL_LEN_MAX          0.28f   /* longest  random wall, scene fraction      */
#define MAX_PLACEMENT_TRIES        30      /* anchor-rejection retries before giving up */
#define ANCHOR_MARGIN_FRAC         0.04f   /* keep anchors off the very edge            */
#define ANCHOR_RANGE_FRAC          (1.0f - 2.0f * ANCHOR_MARGIN_FRAC)
#define INNER_SCENE_MIN_FRAC       0.02f   /* clamp wall endpoints inside outer wall    */
#define INNER_SCENE_MAX_FRAC       0.98f

/* ===================================================================== */
/* S2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long  )(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* S3  color                                                              */
/* ===================================================================== */

enum {
    CP_DEFAULT = 0,

    /* Visible-region fill -- brightness encodes distance from observer */
    CP_FILL_NEAR,    /* very close: near-white                           */
    CP_FILL_MID,     /* medium distance: yellow-green                    */
    CP_FILL_FAR,     /* far from observer: steel blue                    */

    /* Walls and bounding box */
    CP_WALL,         /* interior obstacle walls: bold white              */
    CP_BOUND,        /* bounding box frame: dim grey                     */

    /* Observer dot and animated sweep ray */
    CP_OBSERVER,     /* observer '@': bright magenta                     */
    CP_SWEEP_RAY,    /* sweep ray '*': bright yellow                     */

    /* HUD layers — canonical project standard (bright + A_BOLD) */
    CP_HUD,          /* top status row: bright yellow                    */
    CP_HINT,         /* bottom action hint row: bright cyan              */
};

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_FILL_NEAR,   231,  -1);
        init_pair(CP_FILL_MID,    190,  -1);
        init_pair(CP_FILL_FAR,     38,  -1);
        init_pair(CP_WALL,        255,  -1);
        init_pair(CP_BOUND,        59,  -1);
        init_pair(CP_OBSERVER,    201,  -1);
        init_pair(CP_SWEEP_RAY,   226,  -1);
        init_pair(CP_HUD,         226,  -1);   /* bright yellow */
        init_pair(CP_HINT,         51,  -1);   /* bright cyan   */
    } else {
        init_pair(CP_FILL_NEAR,  COLOR_WHITE,   -1);
        init_pair(CP_FILL_MID,   COLOR_YELLOW,  -1);
        init_pair(CP_FILL_FAR,   COLOR_CYAN,    -1);
        init_pair(CP_WALL,       COLOR_WHITE,   -1);
        init_pair(CP_BOUND,      COLOR_WHITE,   -1);
        init_pair(CP_OBSERVER,   COLOR_MAGENTA, -1);
        init_pair(CP_SWEEP_RAY,  COLOR_YELLOW,  -1);
        init_pair(CP_HUD,        COLOR_YELLOW,  -1);
        init_pair(CP_HINT,       COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* S4  geometry                                                           */
/* ===================================================================== */

/*
 * Wall — one line-segment obstacle in CELL space.
 *
 * Intent
 *   The atomic unit consumed by ray_hits_wall().  Each frame the sweep
 *   intersects every ray against every Wall in WallSet.list[].
 *
 * Why a 4-float record (rather than two Points)
 *   Ray-segment intersection (Bourke 1989 / Ericson 2005 §5.1) reads
 *   the endpoints as scalars (A.x, A.y, B.x, B.y); keeping them flat
 *   matches that math literally and avoids one layer of indirection
 *   in the hottest inner loop of the program.
 *
 * Members
 *   x0_cells, y0_cells   segment start in cell space (column, row).
 *   x1_cells, y1_cells   segment end   in cell space (column, row).
 *
 * Invariants
 *   The segment is UNDIRECTED — swapping (x0,y0) ↔ (x1,y1) yields the
 *   same occlusion behaviour.  cast_ray_in_direction() does not rely
 *   on a winding order.
 *   The _cells suffix is load-bearing: distance math must convert via
 *   geo_y() to compensate for the 2:1 terminal cell aspect ratio.
 *
 * References
 *   [8] Ericson 2005 §5.1 — ray vs segment, robust formulation.
 *   [9] Bourke 1989 — short derivation of the 2-D cross-product test
 *       used by ray_hits_wall().
 */
typedef struct {
    float x0_cells, y0_cells;   /* segment start: (col, row) */
    float x1_cells, y1_cells;   /* segment end:   (col, row) */
} Wall;

/*
 * WallSpec — a wall expressed as fractions of scene dimensions.
 *
 * Intent
 *   Resolution-independent obstacle definition.  scene_build_walls()
 *   instantiates each WallSpec into a cell-space Wall by multiplying
 *   the fractions by (scene_cols, scene_rows).
 *
 * Why fractions (not cell coords baked in)
 *   The terminal can resize at any moment.  Storing the LAYOUT as
 *   fractions and re-baking to Wall on every resize means the
 *   obstacle arrangement preserves its visual proportions across
 *   terminal sizes — a small 80×24 window and a wide 200×60 window
 *   show the SAME scene, just bigger / smaller.  Equivalent to
 *   normalized device coordinates in 3-D graphics.
 *
 * Members
 *   x0_frac, y0_frac   segment start as a fraction of (cols, rows).
 *   x1_frac, y1_frac   segment end   as a fraction of (cols, rows).
 *
 * Invariants
 *   All four fields lie in [0.0, 1.0] for default layouts.  The random
 *   generator may place endpoints anywhere in that closed unit square.
 *   0.0 = top/left edge; 1.0 = bottom/right edge.
 */
typedef struct {
    float x0_frac, y0_frac;  /* start: fraction of (scene_cols, scene_rows) */
    float x1_frac, y1_frac;  /* end:   fraction of (scene_cols, scene_rows) */
} WallSpec;

/*
 * SweepHit — one (angle, contact) sample from the radial visibility sweep.
 *
 * Intent
 *   The atomic OUTPUT unit of compute_visibility().  For each ray cast
 *   at an endpoint angle, the algorithm records WHICH direction the
 *   ray went (angle_rad) and WHERE it first hit a wall (hit_x/y_cells).
 *   Sorting hits by angle and joining them produces the boundary of
 *   the visibility polygon.
 *
 * Why bundle angle + hit point (not separate arrays)
 *   qsort must keep angle and hit_point paired during reordering.
 *   Bundling them into one struct lets us sort with a SINGLE cmp_by_angle
 *   function that just dereferences `angle_rad`; storing them in two
 *   parallel arrays would require a permutation index and a second
 *   sweep to reorder the second array.
 *
 * Members
 *   angle_rad      ray direction from the observer, in geo space.
 *                  SORT KEY — strictly ascending in the final polygon.
 *   hit_x_cells    x of the ray's first wall contact, in cell space.
 *   hit_y_cells    y of the ray's first wall contact, in cell space.
 *
 * Invariants
 *   angle_rad ∈ (−π, π].  We wrap into this canonical interval before
 *   sorting so the polygon vertices walk around the observer exactly
 *   once.
 *   The hit point lies on some Wall in WallSet.list[].
 *
 * References
 *   [1] Asano et al. 1986 — angular-sweep formulation; sort hits by
 *       angle, join consecutive hits with straight segments.
 *  [10] Patel (Red Blob Games) — interactive presentation of the
 *       hits-sorted-by-angle data layout.
 */
typedef struct {
    float angle_rad;      /* direction of this ray from observer, in geo space */
    float hit_x_cells;    /* x of the ray's first wall contact, in cell coords */
    float hit_y_cells;    /* y of the ray's first wall contact, in cell coords */
} SweepHit;

/*
 * VisibilityPolygon — complete output of one compute_visibility() call.
 *
 * Intent
 *   Captures the star-shaped polygon rooted at the observer in one
 *   compact bundle that downstream consumers (render_scene,
 *   cell_is_visible, draw_hud) can read freely.  Recomputed in full
 *   every frame; never updated incrementally.
 *
 * Why a STRUCT (not just a raw hits[] array)
 *   Three pieces of metadata travel together:
 *     • the boundary vertices,
 *     • how many of them are valid,
 *     • the polygon AREA in cell² (cached so the HUD doesn't
 *       re-shoelace every frame).
 *   Bundling them prevents the "two-out-of-three" bug where a caller
 *   reads stale n_hits against fresh hits[].
 *
 * Members
 *   hits[MAX_SWEEP_RAYS]   boundary vertices SORTED BY angle_rad.
 *                          Form the star-shaped visibility polygon
 *                          when joined cyclically.
 *   n_hits                 number of valid entries in hits[];
 *                          rays that missed all walls are dropped.
 *   visible_area_cells     polygon area in cell² (shoelace formula);
 *                          cached so draw_hud's percentage computation
 *                          is O(1) per frame instead of O(n_hits).
 *
 * Invariants
 *   0 ≤ n_hits ≤ MAX_SWEEP_RAYS.
 *   hits[i].angle_rad < hits[i+1].angle_rad  for i in [0, n_hits − 2].
 *   visible_area_cells ≥ 0 (sign-flipped via fabsf in the shoelace).
 *
 * References
 *   [6] O'Rourke 1998 §8 — visibility polygon shape (star-shaped, with
 *       the observer as the kernel point).
 *   [5] de Berg et al. 2008 Ch. 15 — visibility-polygon construction.
 *   Meister, A. L. F. (1769) / Gauss — shoelace area formula
 *       derivation; reproduced in any computational-geometry text.
 */
typedef struct {
    SweepHit hits[MAX_SWEEP_RAYS]; /* sorted boundary vertices              */
    int       n_hits;              /* number of valid sweep samples         */
    float     visible_area_cells;  /* polygon area in cell^2 (shoelace)     */
} VisibilityPolygon;

/*
 * geo_y / cell_y: convert between cell-space and geo-space y coordinates.
 * All angle/distance math must be done in geo space.
 * Only convert back to cell space for the final mvaddch() call.
 */
static inline float geo_y(float y_cells)  { return y_cells * ASPECT_Y; }
static inline float cell_y(float y_geo)   { return y_geo   / ASPECT_Y; }

/*
 * ray_hits_wall -- parametric ray vs. segment intersection (geo space).
 *
 * Ray:     P(t) = O + t*D              (want t >= RAY_T_MIN)
 * Segment: Q(s) = A + s*(B-A)          (want 0 <= s <= 1)
 *
 * Setting P(t) = Q(s):
 *   denom   = D x (B-A)                (2D cross product, scalar)
 *   t_ray   = (A-O) x (B-A) / denom   (ray parameter at intersection)
 *   s_seg   = (A-O) x D     / denom   (segment parameter at intersection)
 *
 * 2D cross product: (u x v) = u.x*v.y - u.y*v.x
 *
 * Returns t_ray on a valid hit; returns -1.0f otherwise.
 * All arguments must be in geo space (y already multiplied by ASPECT_Y).
 */
static float ray_hits_wall(
    float ox_geo,  float oy_geo,   /* ray origin      */
    float dx_geo,  float dy_geo,   /* ray direction   */
    float ax_geo,  float ay_geo,   /* segment start A */
    float bx_geo,  float by_geo    /* segment end   B */
) {
    float seg_dx = bx_geo - ax_geo;
    float seg_dy = by_geo - ay_geo;

    /* denom = D x (B-A) -- if near zero, ray and segment are parallel */
    float denom = dx_geo * seg_dy - dy_geo * seg_dx;
    if (fabsf(denom) < 1e-9f) return -1.0f;

    float ao_x = ax_geo - ox_geo;
    float ao_y = ay_geo - oy_geo;

    float t_ray = (ao_x * seg_dy  - ao_y * seg_dx)  / denom;
    float s_seg = (ao_x * dy_geo  - ao_y * dx_geo)  / denom;

    if (t_ray < RAY_T_MIN || s_seg < 0.0f || s_seg > 1.0f) return -1.0f;
    return t_ray;
}

/* qsort comparator: order SweepHits ascending by angle_rad */
static int cmp_sweep_hit_by_angle(const void *lhs, const void *rhs)
{
    float a = ((const SweepHit *)lhs)->angle_rad;
    float b = ((const SweepHit *)rhs)->angle_rad;
    return (a > b) - (a < b);
}

/* ===================================================================== */
/* S5  visibility                                                         */
/* ===================================================================== */

/*
 * cast_ray_in_direction -- find the nearest wall hit along one ray.
 *
 * obs_x/y_cells: observer position in cell space.
 * ray_angle_rad: direction to cast in geo space (y already corrected).
 * all_walls, n_walls: the complete wall list to test against.
 * out_hit: filled with angle_rad and hit_x/y_cells on success.
 *
 * Returns false if no wall was intersected (should not happen in a closed scene).
 */
/*
 * intersect_ray_with_wall_geo — thin wrapper around ray_hits_wall that
 * stretches a Wall's cell-space endpoints into geo space first.  Keeps
 * the call site in cast_ray_in_direction clean.
 */
static float intersect_ray_with_wall_geo(
    float ox_geo, float oy_geo,
    float dx_geo, float dy_geo,
    const Wall *w
) {
    return ray_hits_wall(
        ox_geo, oy_geo,
        dx_geo, dy_geo,
        w->x0_cells, geo_y(w->y0_cells),
        w->x1_cells, geo_y(w->y1_cells)
    );
}

/*
 * find_nearest_wall_t — min-t reduction over every wall.
 *
 *   Pseudocode:
 *     nearest := +∞
 *     for each wall w:
 *       t := intersect_ray_with_wall_geo(ray, w)
 *       if t > 0 and t < nearest:
 *         nearest := t
 *     return nearest
 *
 *   Returns RAY_NO_HIT_T_SENTINEL when no wall is hit.  cast_ray_in_direction
 *   compares the result against RAY_VALID_HIT_T_LIMIT to detect that case.
 */
static float find_nearest_wall_t(
    float ox_geo, float oy_geo,
    float dx_geo, float dy_geo,
    const Wall *walls, int n_walls
) {
    float nearest_t = RAY_NO_HIT_T_SENTINEL;
    for (int i = 0; i < n_walls; i++) {
        float t = intersect_ray_with_wall_geo(ox_geo, oy_geo,
                                              dx_geo, dy_geo,
                                              &walls[i]);
        if (t > 0.0f && t < nearest_t) nearest_t = t;
    }
    return nearest_t;
}

/* Write the (angle, hit_point) pair into a SweepHit.  Converts the
 * geo-space hit point back to cell space for the y component. */
static void record_sweep_hit(
    SweepHit *out, float ray_angle_rad,
    float ox_geo, float oy_geo, float dx_geo, float dy_geo, float t
) {
    out->angle_rad   = ray_angle_rad;
    out->hit_x_cells = ox_geo + t * dx_geo;                /* x same in both spaces */
    out->hit_y_cells = cell_y(oy_geo + t * dy_geo);        /* y unstretched         */
}

/*
 * cast_ray_in_direction — fire ONE ray from the observer, return the
 * nearest wall hit (if any).
 *
 *   Pseudocode:
 *     (ox, oy)  := observer in geo space
 *     (dx, dy)  := unit ray direction
 *     t         := find_nearest_wall_t over all walls
 *     if t indicates no hit: return false
 *     record (angle, hit point) into out_hit; return true
 */
static bool cast_ray_in_direction(
    float obs_x_cells, float obs_y_cells,
    float ray_angle_rad,
    const Wall *all_walls, int n_walls,
    SweepHit   *out_hit
) {
    float ox_geo = obs_x_cells;
    float oy_geo = geo_y(obs_y_cells);
    float dx_geo = cosf(ray_angle_rad);
    float dy_geo = sinf(ray_angle_rad);

    float t = find_nearest_wall_t(ox_geo, oy_geo, dx_geo, dy_geo,
                                  all_walls, n_walls);
    if (t >= RAY_VALID_HIT_T_LIMIT) return false;

    record_sweep_hit(out_hit, ray_angle_rad, ox_geo, oy_geo, dx_geo, dy_geo, t);
    return true;
}

/*
 * compute_visibility -- angular sweep producing the full visibility polygon.
 *
 * The algorithm (three steps):
 *   1. Collect endpoint angles: for every wall endpoint, compute the angle
 *      from the observer.  Emit angle-eps, angle, angle+eps to bracket
 *      each corner.
 *   2. Cast one ray per candidate angle; record the nearest wall hit.
 *   3. Sort all hits by angle_rad.  The ordered sequence of hit points is
 *      the star-shaped boundary polygon of visible space.
 *
 * The shoelace formula is applied to the sorted hits to get visible area.
 */
/*
 * angle_from_observer_to_endpoint — atan2 in geo space, with the
 * eye stretched + the endpoint stretched so the angle is metrically
 * correct on the 2:1 terminal grid.
 */
static float angle_from_observer_to_endpoint(
    float ox_geo, float oy_geo, float ex_cells, float ey_cells
) {
    float dx = ex_cells           - ox_geo;
    float dy = geo_y(ey_cells)    - oy_geo;
    return atan2f(dy, dx);
}

/*
 * push_endpoint_angle_triplet — emit (θ−ε, θ, θ+ε) into the candidate
 * angle buffer.  The ε-offset trick (see [10] Patel / Red Blob Games)
 * disambiguates which side of a wall corner the ray hits.
 *
 *   Skip if the buffer would overflow — `out_count` is treated as a
 *   bounded resource.
 */
static void push_endpoint_angle_triplet(
    float angles[], int *count, int capacity, float theta
) {
    if (*count + 3 > capacity) return;
    angles[(*count)++] = theta - ENDPOINT_ANGLE_EPS_RAD;
    angles[(*count)++] = theta;
    angles[(*count)++] = theta + ENDPOINT_ANGLE_EPS_RAD;
}

/*
 * collect_endpoint_candidate_angles — Step 1 of the sweep.
 *
 *   Pseudocode:
 *     for each wall w:
 *       for each endpoint e in (w.start, w.end):
 *         θ := angle_from_observer_to_endpoint(observer, e)
 *         push (θ − ε, θ, θ + ε) into the candidate buffer
 *
 *   The output is an unsorted list of angles; cast_all_candidate_rays
 *   fires one ray per angle, and the final qsort imposes order.
 */
static int collect_endpoint_candidate_angles(
    float ox_geo, float oy_geo,
    const Wall *walls, int n_walls,
    float out_angles[], int capacity
) {
    int n_candidates = 0;
    for (int i = 0; i < n_walls; i++) {
        const Wall *w = &walls[i];
        float t0 = angle_from_observer_to_endpoint(ox_geo, oy_geo,
                                                   w->x0_cells, w->y0_cells);
        float t1 = angle_from_observer_to_endpoint(ox_geo, oy_geo,
                                                   w->x1_cells, w->y1_cells);
        push_endpoint_angle_triplet(out_angles, &n_candidates, capacity, t0);
        push_endpoint_angle_triplet(out_angles, &n_candidates, capacity, t1);
    }
    return n_candidates;
}

/*
 * cast_all_candidate_rays — Step 2 of the sweep.
 *
 *   Pseudocode:
 *     out.n_hits := 0
 *     for each candidate angle θ:
 *       if cast_ray_in_direction(θ) hits a wall:
 *         out.hits[out.n_hits++] := hit
 *
 *   Rays that miss every wall (degenerate: open scene) are silently
 *   dropped.  In a bounded room the outer walls always catch a ray.
 */
static void cast_all_candidate_rays(
    float obs_x_cells, float obs_y_cells,
    const float angles[], int n_angles,
    const Wall *walls, int n_walls,
    VisibilityPolygon *out
) {
    out->n_hits = 0;
    for (int i = 0; i < n_angles; i++) {
        SweepHit hit;
        if (cast_ray_in_direction(obs_x_cells, obs_y_cells, angles[i],
                                  walls, n_walls, &hit)
            && out->n_hits < MAX_SWEEP_RAYS)
        {
            out->hits[out->n_hits++] = hit;
        }
    }
}

/* Step 3 wrapper — qsort hits by angle_rad so consecutive entries
 * form sectors of the visibility polygon. */
static void sort_hits_by_angle(VisibilityPolygon *vp)
{
    qsort(vp->hits, vp->n_hits, sizeof(SweepHit), cmp_sweep_hit_by_angle);
}

/*
 * polygon_shoelace_area_cells — Gauss/Meister 1769 shoelace formula
 * applied to the sorted hits, returning area in cell² units.
 *
 *   Pseudocode:
 *     sum := 0
 *     for i in 0..n−1:
 *       j := (i + 1) mod n
 *       sum += xi · yj − xj · yi              (in geo space)
 *     return ½ · |sum| / ASPECT_Y             (undo y-stretch)
 *
 *   The y-stretch undo step recovers cell² because cell-y was multiplied
 *   by ASPECT_Y throughout the angle math.
 */
static float polygon_shoelace_area_cells(const VisibilityPolygon *vp)
{
    float sum = 0.0f;
    int n = vp->n_hits;
    for (int i = 0; i < n; i++) {
        int   j  = (i + 1) % n;
        float xi = vp->hits[i].hit_x_cells;
        float yi = geo_y(vp->hits[i].hit_y_cells);
        float xj = vp->hits[j].hit_x_cells;
        float yj = geo_y(vp->hits[j].hit_y_cells);
        sum += xi * yj - xj * yi;
    }
    return 0.5f * fabsf(sum) / ASPECT_Y;
}

/*
 * compute_visibility — the angular-sweep orchestrator.
 *
 *   Pseudocode:
 *     Step 1: collect endpoint candidate angles (3 per endpoint, ±ε bracket).
 *     Step 2: cast one ray per angle, keep the nearest wall hit.
 *     Step 3: sort hits by angle → polygon boundary in CCW order.
 *     Step 4: shoelace area, cache into out.visible_area_cells.
 *
 *   Asano et al. 1986 / O'Rourke 1998 §8 — exact angular-sweep
 *   visibility polygon.
 */
static void compute_visibility(
    float obs_x_cells, float obs_y_cells,
    const Wall *all_walls, int n_walls,
    VisibilityPolygon *out_vp
) {
    float ox_geo = obs_x_cells;
    float oy_geo = geo_y(obs_y_cells);

    float candidates[MAX_SWEEP_RAYS];
    int   n_candidates = collect_endpoint_candidate_angles(
        ox_geo, oy_geo, all_walls, n_walls, candidates, MAX_SWEEP_RAYS);

    cast_all_candidate_rays(obs_x_cells, obs_y_cells,
                            candidates, n_candidates,
                            all_walls, n_walls, out_vp);

    sort_hits_by_angle(out_vp);

    out_vp->visible_area_cells = polygon_shoelace_area_cells(out_vp);
}

/*
 * find_angle_sector -- binary search for the largest i where hits[i].angle_rad
 * <= query_angle_rad.  Returns i; caller wraps to (i+1) % n_hits for the sector.
 *
 * Precondition: hits[] sorted ascending, n_hits >= 1.
 */
static int find_angle_sector(const VisibilityPolygon *vp, float query_angle_rad)
{
    int lo = 0;
    int hi = vp->n_hits - 1;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (vp->hits[mid].angle_rad <= query_angle_rad)
            lo = mid;
        else
            hi = mid - 1;
    }
    return lo;
}

/*
 * cell_is_visible -- test whether a terminal cell lies inside the visibility polygon.
 *
 * Algorithm (O(log N) per cell):
 *   1. Compute angle from observer to cell center in geo space.
 *   2. Identify which angular sector the cell falls in (binary search).
 *      The wrap-around sector [hits[n-1], hits[0]] needs a special check.
 *   3. The polygon boundary in that sector is segment hits[i] -> hits[next].
 *   4. Cast a ray from observer at cell_angle; intersect with that boundary.
 *   5. Cell is visible iff cell_dist <= boundary_t (with a small tolerance).
 */
/*
 * cell_displacement_from_observer_geo — write the (dx, dy) vector and
 * the squared length of the observer→cell displacement, in geo space.
 *   The squared distance is sufficient for the "own cell" early exit
 *   so we avoid a sqrtf in the hot path.
 */
static void cell_displacement_from_observer_geo(
    float cell_x_cells, float cell_y_cells,
    float obs_x_cells,  float obs_y_cells,
    float *out_dx, float *out_dy, float *out_dist_sq
) {
    float dx = cell_x_cells           - obs_x_cells;
    float dy = geo_y(cell_y_cells)    - geo_y(obs_y_cells);
    *out_dx      = dx;
    *out_dy      = dy;
    *out_dist_sq = dx * dx + dy * dy;
}

/*
 * locate_polygon_sector_for_angle — given a cell angle, return the
 * indices (sector, next) of the polygon boundary segment that brackets
 * that angle.  The wrap-around sector (between the LAST and FIRST hit)
 * is detected explicitly because find_angle_sector assumes interior.
 */
static void locate_polygon_sector_for_angle(
    const VisibilityPolygon *vp, float cell_angle_rad,
    int *out_sector, int *out_next
) {
    float first = vp->hits[0].angle_rad;
    float last  = vp->hits[vp->n_hits - 1].angle_rad;

    if (cell_angle_rad < first || cell_angle_rad >= last) {
        *out_sector = vp->n_hits - 1;
        *out_next   = 0;                              /* wrap-around */
    } else {
        *out_sector = find_angle_sector(vp, cell_angle_rad);
        *out_next   = *out_sector + 1;                /* in-range */
    }
}

/*
 * intersect_cell_ray_with_sector_boundary — fire a unit ray at
 * cell_angle_rad from the observer and intersect with the polygon
 * boundary segment hits[sector]→hits[next].  Returns the parameter t
 * along the ray, or a negative value if the boundary is degenerate.
 */
static float intersect_cell_ray_with_sector_boundary(
    const VisibilityPolygon *vp, int sector_idx, int next_idx,
    float obs_x_cells, float obs_y_cells, float cell_angle_rad
) {
    /* Boundary segment endpoints relative to observer, geo space */
    float bnd0_x = vp->hits[sector_idx].hit_x_cells - obs_x_cells;
    float bnd0_y = geo_y(vp->hits[sector_idx].hit_y_cells) - geo_y(obs_y_cells);
    float bnd1_x = vp->hits[next_idx].hit_x_cells   - obs_x_cells;
    float bnd1_y = geo_y(vp->hits[next_idx].hit_y_cells) - geo_y(obs_y_cells);
    float dir_x  = cosf(cell_angle_rad);
    float dir_y  = sinf(cell_angle_rad);
    return ray_hits_wall(0.0f, 0.0f, dir_x, dir_y,
                         bnd0_x, bnd0_y, bnd1_x, bnd1_y);
}

/* Fallback for the degenerate-sector case: visible iff cell sits within
 * one cell of the polygon-boundary first endpoint distance. */
static bool visible_via_degenerate_sector_fallback(
    const VisibilityPolygon *vp, int sector_idx,
    float obs_x_cells, float obs_y_cells, float cell_dist
) {
    float bnd0_x = vp->hits[sector_idx].hit_x_cells - obs_x_cells;
    float bnd0_y = geo_y(vp->hits[sector_idx].hit_y_cells) - geo_y(obs_y_cells);
    float bnd0_dist = sqrtf(bnd0_x * bnd0_x + bnd0_y * bnd0_y);
    return cell_dist <= bnd0_dist + DEGENERATE_SECTOR_SLACK_CELLS;
}

/*
 * cell_is_visible — does this cell sit inside the visibility polygon?
 *
 *   Pseudocode (O(log N) per cell):
 *     if polygon is degenerate (< 3 hits): false
 *     compute (dx, dy, dist²) from observer to cell
 *     if cell is on top of the observer:   true                    (early exit)
 *     θ := atan2(dy, dx)
 *     locate the angular sector hits[sector]→hits[next] containing θ
 *     t := intersect a unit ray at θ with that boundary segment
 *     if degenerate sector:  fall back to bnd0-distance check
 *     else:                  visible iff cell_dist ≤ t + tolerance
 *
 *   The tolerance (BOUNDARY_VISIBILITY_TOLERANCE_CELLS) lets cells
 *   that ride the polygon edge still light up — cosmetic.
 */
static bool cell_is_visible(
    float cell_x_cells, float cell_y_cells,
    float obs_x_cells,  float obs_y_cells,
    const VisibilityPolygon *vp
) {
    if (vp->n_hits < 3) return false;

    float dx, dy, dist_sq;
    cell_displacement_from_observer_geo(cell_x_cells, cell_y_cells,
                                        obs_x_cells, obs_y_cells,
                                        &dx, &dy, &dist_sq);
    if (dist_sq < OBSERVER_OWN_CELL_RADIUS_SQ) return true;

    float cell_angle = atan2f(dy, dx);
    int sector_idx, next_idx;
    locate_polygon_sector_for_angle(vp, cell_angle, &sector_idx, &next_idx);

    float boundary_t = intersect_cell_ray_with_sector_boundary(
        vp, sector_idx, next_idx, obs_x_cells, obs_y_cells, cell_angle);

    float cell_dist = sqrtf(dist_sq);
    if (boundary_t < 0.0f)
        return visible_via_degenerate_sector_fallback(
            vp, sector_idx, obs_x_cells, obs_y_cells, cell_dist);

    return cell_dist <= boundary_t + BOUNDARY_VISIBILITY_TOLERANCE_CELLS;
}

/* ===================================================================== */
/* S6  scene                                                              */
/* ===================================================================== */

/*
 * Bounding walls: the four outer edges of the room.
 * Fractions (0,0)-(1,0) etc. map to the full scene width/height after init.
 */
static const WallSpec BOUNDING_WALL_SPECS[BOUNDING_WALL_COUNT] = {
    { 0.00f, 0.00f,  1.00f, 0.00f },   /* top    edge */
    { 1.00f, 0.00f,  1.00f, 1.00f },   /* right  edge */
    { 1.00f, 1.00f,  0.00f, 1.00f },   /* bottom edge */
    { 0.00f, 1.00f,  0.00f, 0.00f },   /* left   edge */
};

/*
 * Interior obstacle walls: placed in the outer thirds of the scene so
 * the observer's Lissajous path (occupying the central ~25%) stays clear.
 *
 * Layout sketch (. = open space, # = wall, @ = observer path region):
 *
 *   |   |   |   | # |   |   |   |   |   |   |   |   |   |   |   |   | # |
 *   |   |   |   | # |                    @@@@@                         | # |
 *   | --+-- |   | # |           @@@@@@@@@@@@@@@@@@@@@                  | # |
 *   |       |   | # |         @@@@@@@@@@@@@@@@@@@@@@@@@                | # |
 *   |       |                 @@@@@@@@@@@@@@@@@@@@@@@@@                |   |
 *   |   ----+---|             @@@@@@@@@@@@@@@@@@@@@@@@@                |---|
 *   |           |             @@@@@@@@@@@@@@@@@@@@@@@@@                    |
 *   |           |              @@@@@@@@@@@@@@@@@@@@@@@                     |
 *   |           |               @@@@@@@@@@@@@@@@@@@@                   |   |
 *   | L-corner  |                                                       |   |
 *                                                           L-corner ---|   |
 */
static const WallSpec INTERIOR_WALL_SPECS[INTERIOR_WALL_COUNT] = {
    /* Left vertical pillar: tall, creates strong left-side shadow */
    { 0.15f, 0.05f,  0.15f, 0.60f },

    /* Right vertical pillar: symmetric, creates right-side shadow */
    { 0.85f, 0.05f,  0.85f, 0.60f },

    /* Upper-left notch: short horizontal shelf near top-left */
    { 0.05f, 0.28f,  0.22f, 0.28f },

    /* Upper-right notch: symmetric shelf near top-right */
    { 0.78f, 0.28f,  0.95f, 0.28f },

    /* Lower-left L -- horizontal part: creates bottom-left shadow pocket */
    { 0.05f, 0.72f,  0.30f, 0.72f },

    /* Lower-left L -- vertical part (forms the L with line above) */
    { 0.30f, 0.72f,  0.30f, 0.95f },

    /* Lower-right L -- horizontal part */
    { 0.70f, 0.72f,  0.95f, 0.72f },

    /* Lower-right L -- vertical part */
    { 0.70f, 0.72f,  0.70f, 0.95f },

    /* Central top post: short vertical blocker directly above path */
    { 0.50f, 0.04f,  0.50f, 0.22f },
};

/*
 * WallSet — every line-segment obstacle in the scene.
 *
 * Intent
 *   Owns the OBSTACLES seen by the visibility sweep.  Includes both
 *   the active wall list AND the random-pattern feature state, so
 *   pressing 'n' regenerates a fresh layout without touching the rest
 *   of the Scene.
 *
 * Why a BUNDLE (not three top-level Scene fields)
 *   The wall list, its count, and the random-pattern variant always
 *   move together — every place that reads `list[]` needs to know
 *   `count` to bound the loop, and choosing between the default and
 *   random layouts touches `use_rand` + `rand_specs[]`.  Grouping
 *   them keeps scene_build_walls() and the 'n' key handler reading
 *   one bundle instead of four scattered Scene fields.
 *
 * Why a fixed array (not malloc'd slice)
 *   Project rule: no dynamic allocation after init.  MAX_WALLS (13 =
 *   4 bounding + 9 interior) is the exact capacity used by every
 *   layout; the random generator overwrites only the interior slots.
 *
 * Members
 *   list[MAX_WALLS]    line-segment obstacles in cell coords.  First
 *                      BOUNDING_WALL_COUNT entries are the outer
 *                      bounding box (always present); the rest are
 *                      interior obstacles.
 *   count              number of valid entries in list[].
 *   rand_specs[]       fractions of a randomised interior layout;
 *                      re-instantiated to cell-space on each resize
 *                      via scene_build_walls().
 *   use_rand           false = use INTERIOR_WALL_SPECS default layout;
 *                      true after the first 'n' press = use rand_specs.
 *
 * Invariants
 *   BOUNDING_WALL_COUNT ≤ count ≤ MAX_WALLS.
 *   list[0..BOUNDING_WALL_COUNT−1] is always the outer bounding box.
 *   At any moment EITHER the default INTERIOR_WALL_SPECS or rand_specs
 *   has been baked into list[]; use_rand selects which.
 *
 * References
 *   [4] Suri & O'Rourke 1986 — visibility among disjoint line
 *       segments (the polygons-with-holes regime this WallSet models).
 */
typedef struct {
    Wall      list[MAX_WALLS];
    int       count;
    WallSpec  rand_specs[INTERIOR_WALL_COUNT];
    bool      use_rand;
} WallSet;

/*
 * Observer — the eye + the visualisation sweep ray.
 *
 * Intent
 *   Owns everything the visibility math + scene animation need to know
 *   about WHERE the viewer is and HOW the spinning teaching-aid ray is
 *   oriented.  The visibility sweep math reads x_cells / y_cells; the
 *   spinning sweep ray glyph is a SEPARATE teaching aid driven by
 *   sweep_angle_rad; lissajous_t parameterises the auto-drift path.
 *
 * Why bundle three state machines (eye pos, drift phase, sweep angle)
 *   They all share the SAME conceptual frame of reference — "the eye"
 *   — even though they evolve at different rates:
 *     • lissajous_t  → ticks forward each frame; deterministic curve.
 *     • x_cells/y_cells → derived from lissajous_t each tick.
 *     • sweep_angle_rad → rotates independently for the '*' overlay.
 *   Splitting them across three Scene fields would scatter "anything
 *   that the eye knows" across the file; bundling makes scene_tick()
 *   a single sub-struct mutation.
 *
 * Why TWO separate angles (lissajous_t and sweep_angle_rad)
 *   lissajous_t parameterises the *path* (Lissajous curve), driving
 *   where the eye is.  sweep_angle_rad drives the *spinning ray
 *   overlay*, an entirely separate teaching aid that exists only to
 *   visualise "what a single cast looks like".  They are independent
 *   so the user can pause the eye while the ray keeps sweeping, or
 *   vice versa, depending on UI design.
 *
 * Members
 *   x_cells, y_cells    eye position in CELL space (column, row).
 *                       Drives the angular-sweep computation; the
 *                       visibility polygon is rooted here.
 *   lissajous_t         Lissajous-path phase parameter (radians);
 *                       grows monotonically each tick.  The eye
 *                       position is then
 *                         x = cx + rx · sin(fx · t)
 *                         y = cy + ry · sin(fy · t + phase)
 *                       — a Lissajous figure with rational
 *                       frequency ratio (Bowditch 1815 / Lissajous 1857).
 *   sweep_angle_rad     direction of the animated '*' sweep ray drawn
 *                       in the scene; rotates independently of the
 *                       visibility polygon computation.
 *
 * Invariants
 *   0 < x_cells < scene_cols   AND   0 < y_cells < scene_rows
 *     (scene_update_observer keeps the eye inside the bounding box;
 *      OBS_RADIUS_*_FRAC are tuned to leave a margin).
 *   sweep_angle_rad ∈ (−π, π]  (cast_ray_in_direction is angle-agnostic
 *                               but we wrap to keep numbers small).
 *   lissajous_t is unbounded as written — sin/cos handle wraparound,
 *   so we accept the slow precision drift over ~10^8 frames.
 *
 * References
 *   Bowditch, N. (1815) / Lissajous, J. A. (1857) — the curve
 *       x = A·sin(at), y = B·sin(bt + δ) that traces compound
 *       harmonic motion when a/b is rational.  Used here purely
 *       as a deterministic non-trivial 2-D path for the demo.
 *   [10] Patel (Red Blob Games) — uses a similar drifting observer
 *       in the interactive 2-D visibility tutorial.
 */
typedef struct {
    float x_cells, y_cells;
    float lissajous_t;
    float sweep_angle_rad;
} Observer;

/*
 * SimControls — playback controls toggled / adjusted at runtime.
 *
 * Intent
 *   Two knobs the user adjusts via the input handler; kept together
 *   so handle_input writes ONE bundle rather than two scattered
 *   Scene fields.  Decouples the FIXED-TIMESTEP accumulator in
 *   main() from the underlying simulation rate.
 *
 * Why a sub-struct (two scalars is not much)
 *   The boundary "user-controlled run parameters" is conceptually
 *   distinct from "simulation state" — paused and fps are read by
 *   main()'s tick loop and the HUD; everything else in Scene is
 *   simulation output.  Naming them as a bundle signals that
 *   distinction to the reader.  Easy to extend later (target_fps,
 *   speed_multiplier) without churning Scene's top level.
 *
 * Members
 *   paused      true → scene_tick is a no-op (animation freezes).
 *               The HUD then displays "PAUSED" instead of "running".
 *   fps         simulation tick rate (Hz).  '+' / '-' adjust by 5,
 *               clamped to [SIM_FPS_MIN, SIM_FPS_MAX].  Drives the
 *               fixed-timestep accumulator's TICK_NS(fps) period.
 *
 * Invariants
 *   SIM_FPS_MIN ≤ fps ≤ SIM_FPS_MAX  (handle_input clamps).
 *
 * References
 *   Fiedler, G., "Fix Your Timestep!" — gafferongames.com/post/
 *       fix_your_timestep — the canonical "accumulator-driven discrete
 *       ticks" pattern that main()'s loop implements using SimControls.
 */
typedef struct {
    bool paused;
    int  fps;
} SimControls;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── walls    : WallSet            ← obstacles + random-layout state
 *       │     ├── list[MAX_WALLS]   : Wall[]        (cell-space segments)
 *       │     ├── count                                (≥ BOUNDING_WALL_COUNT)
 *       │     ├── rand_specs[]      : WallSpec[]   (resolution-independent)
 *       │     └── use_rand          : bool         (default vs 'n'-pressed)
 *       │
 *       ├── observer : Observer           ← eye + sweep-ray + path phase
 *       │     ├── x_cells, y_cells        (Lissajous-driven eye position)
 *       │     ├── lissajous_t             (path-curve parameter)
 *       │     └── sweep_angle_rad         (independent '*' overlay angle)
 *       │
 *       ├── vis      : VisibilityPolygon  ← per-frame angular-sweep output
 *       │     ├── hits[]           : SweepHit[]    (sorted boundary verts)
 *       │     ├── n_hits                            (valid entries)
 *       │     └── visible_area_cells                (cached shoelace area)
 *       │
 *       ├── scene_cols, scene_rows        ← drawable extent inside HUD frame
 *       │
 *       └── sim      : SimControls        ← paused + fps
 *
 *   Every persistent value the program needs is reachable from a
 *   single Scene*.  No file-scope simulation state exists; only the
 *   two POSIX signal flags (g_quit, g_resize_pending) remain global
 *   in §9 because signal handlers cannot receive a context pointer.
 *
 * Intent
 *   One instance lives on main()'s stack and is passed by pointer to
 *   every tick, renderer, and input handler.  Pure observers take
 *   `const Scene *`; mutators take `Scene *`.  The signature alone
 *   communicates which kind of access each callee performs.
 *
 * Why a SCENE struct (not file-scope globals)
 *   • Replaceability: future variants (split-screen, recorded replay)
 *     allocate multiple Scenes — impossible with globals.
 *   • Testability: scene_tick becomes a pure transformation s → s';
 *     no hidden inputs.
 *   • Reading aid: every helper begins with `Scene *s` (or const),
 *     giving the reader ONE place to look for "what state exists".
 *
 * Members
 *   walls           WallSet — obstacles + random-pattern state.
 *   observer        Observer — eye position + sweep ray + Lissajous phase.
 *   vis             VisibilityPolygon — sorted sweep hits + area.
 *   scene_cols      drawable columns (= terminal cols).
 *   scene_rows      drawable rows (= terminal rows − HUD_ROWS).
 *   sim             SimControls — paused + fps.
 *
 * Invariants
 *   scene_cols > 0  AND  scene_rows > 0  (set by scene_init).
 *   walls.count ≥ BOUNDING_WALL_COUNT   AND   observer inside scene area.
 *   g_quit / g_resize_pending are NOT here — POSIX signal handlers
 *   need sig_atomic_t globals, so they live in §9.
 *
 * References
 *   [10] Patel (Red Blob Games) — uses an equivalent single-Scene
 *       "world" object pattern in the interactive 2-D visibility demo.
 *   "Game Programming Patterns" (Nystrom, 2014) — Game Loop chapter
 *       motivates the one-Scene + fixed-timestep accumulator design
 *       that main() implements.
 */
typedef struct {
    WallSet           walls;
    Observer          observer;
    VisibilityPolygon vis;
    int               scene_cols;
    int               scene_rows;
    SimControls       sim;
} Scene;

/*
 * bake_wall_spec_to_cells — convert a single WallSpec (fractions) into
 * a cell-space Wall by scaling fractions against the scene extent.
 *
 *   Equivalent to "normalized device coordinates → device coordinates"
 *   in graphics pipelines.
 */
static void bake_wall_spec_to_cells(
    const WallSpec *spec, float scene_w, float scene_h, Wall *out
) {
    out->x0_cells = spec->x0_frac * scene_w;
    out->y0_cells = spec->y0_frac * scene_h;
    out->x1_cells = spec->x1_frac * scene_w;
    out->y1_cells = spec->y1_frac * scene_h;
}

/* Append `n` walls from `specs[]` to s->walls.list, baking each one. */
static void append_specs_to_wall_list(
    Scene *s, const WallSpec *specs, int n, float scene_w, float scene_h
) {
    for (int i = 0; i < n; i++)
        bake_wall_spec_to_cells(&specs[i], scene_w, scene_h,
                                &s->walls.list[s->walls.count++]);
}

/* Choose between the default layout and the random-generated layout
 * based on the user toggle (set by the 'n' key handler). */
static const WallSpec *interior_layout_to_use(const Scene *s)
{
    return s->walls.use_rand ? s->walls.rand_specs : INTERIOR_WALL_SPECS;
}

/*
 * scene_build_walls — bake the WallSpec layout into cell-space Walls.
 *
 *   Pseudocode:
 *     count := 0
 *     append BOUNDING_WALL_SPECS (the outer box)
 *     append the currently-selected interior layout (default or random)
 *
 *   Must be called after scene_cols / scene_rows are set (init / resize).
 */
static void scene_build_walls(Scene *s)
{
    float scene_w = (float)s->scene_cols;
    float scene_h = (float)s->scene_rows;
    s->walls.count = 0;

    append_specs_to_wall_list(s, BOUNDING_WALL_SPECS, BOUNDING_WALL_COUNT,
                              scene_w, scene_h);
    append_specs_to_wall_list(s, interior_layout_to_use(s), INTERIOR_WALL_COUNT,
                              scene_w, scene_h);
}

/* Uniform random in [0, 1] — wrapper around rand() / RAND_MAX. */
static float rand_frac(void) { return (float)rand() / (float)RAND_MAX; }

/* True when an anchor point lies inside the observer's "no walls" zone
 * — the rectangle centered on (OBS_CENTER_X_FRAC, OBS_CENTER_Y_FRAC)
 * that the Lissajous path traces, plus a margin.  Walls placed inside
 * this zone would clip through the observer mid-glide. */
static bool anchor_in_observer_safe_zone(float ax, float ay)
{
    return fabsf(ax - OBS_CENTER_X_FRAC) < OBS_SAFE_HALF_X
        && fabsf(ay - OBS_CENTER_Y_FRAC) < OBS_SAFE_HALF_Y;
}

/* Draw a uniform random anchor in [ANCHOR_MARGIN, 1 − ANCHOR_MARGIN]²
 * (keep anchors off the very edge of the scene). */
static void random_anchor_in_inner_scene(float *out_ax, float *out_ay)
{
    *out_ax = ANCHOR_MARGIN_FRAC + rand_frac() * ANCHOR_RANGE_FRAC;
    *out_ay = ANCHOR_MARGIN_FRAC + rand_frac() * ANCHOR_RANGE_FRAC;
}

/*
 * pick_anchor_outside_safe_zone — rejection-sample an anchor that
 * misses the observer safe zone.
 *
 *   Pseudocode:
 *     repeat up to MAX_PLACEMENT_TRIES:
 *       draw a uniform anchor
 *       if it is OUTSIDE the safe zone: return
 *     fall through with the last sample (best effort)
 */
static void pick_anchor_outside_safe_zone(float *out_ax, float *out_ay)
{
    *out_ax = OBS_CENTER_X_FRAC;
    *out_ay = OBS_CENTER_Y_FRAC;
    for (int attempt = 0; attempt < MAX_PLACEMENT_TRIES; attempt++) {
        random_anchor_in_inner_scene(out_ax, out_ay);
        if (!anchor_in_observer_safe_zone(*out_ax, *out_ay)) return;
    }
}

/* Uniform angle in [0, π) — undirected segments cover the full plane
 * with only a half-circle of angles. */
static float random_undirected_angle_rad(void)
{
    return rand_frac() * (float)M_PI;
}

/* Uniform half-length in [RAND_WALL_LEN_MIN, RAND_WALL_LEN_MAX]. */
static float random_wall_half_length(void)
{
    return RAND_WALL_LEN_MIN
         + rand_frac() * (RAND_WALL_LEN_MAX - RAND_WALL_LEN_MIN);
}

/* Clamp a scene fraction to the strict interior so random walls cannot
 * sit on top of (or past) the outer bounding box. */
static float clamp_to_inner_scene(float frac)
{
    return fmaxf(INNER_SCENE_MIN_FRAC, fminf(INNER_SCENE_MAX_FRAC, frac));
}

/*
 * write_wall_spec_from_anchor — build a WallSpec from
 * (anchor, angle, half-length).  Endpoints are (anchor ± half-length ·
 * (cos θ, sin θ)), then clamped to the strict interior.
 */
static void write_wall_spec_from_anchor(
    WallSpec *out, float ax, float ay, float angle, float half_len
) {
    float dx = cosf(angle) * half_len;
    float dy = sinf(angle) * half_len;
    out->x0_frac = clamp_to_inner_scene(ax - dx);
    out->y0_frac = clamp_to_inner_scene(ay - dy);
    out->x1_frac = clamp_to_inner_scene(ax + dx);
    out->y1_frac = clamp_to_inner_scene(ay + dy);
}

/*
 * scene_generate_random_walls — rebuild s->walls.rand_specs[] with a
 * fresh random layout, then bake + recompute visibility.
 *
 *   Pseudocode:
 *     for i in 0..INTERIOR_WALL_COUNT−1:
 *       (ax, ay)  := pick_anchor_outside_safe_zone()
 *       θ         := random_undirected_angle_rad()
 *       half_len  := random_wall_half_length()
 *       rand_specs[i] := write_wall_spec_from_anchor(ax, ay, θ, half_len)
 *     mark use_rand := true
 *     scene_build_walls(s)              ← bake new fractions to cells
 *     compute_visibility(observer, walls, &vis)   ← refresh polygon
 */
static void scene_generate_random_walls(Scene *s)
{
    for (int i = 0; i < INTERIOR_WALL_COUNT; i++) {
        float ax, ay;
        pick_anchor_outside_safe_zone(&ax, &ay);
        float angle    = random_undirected_angle_rad();
        float half_len = random_wall_half_length();
        write_wall_spec_from_anchor(&s->walls.rand_specs[i],
                                    ax, ay, angle, half_len);
    }

    s->walls.use_rand = true;
    scene_build_walls(s);
    compute_visibility(s->observer.x_cells, s->observer.y_cells,
                       s->walls.list, s->walls.count, &s->vis);
}

/*
 * lissajous_eval_at_phase — Lissajous curve evaluator in cell space.
 *
 *   Formula (Bowditch 1815 / Lissajous 1857):
 *     x(t) = cx + rx · sin(fx · t)
 *     y(t) = cy + ry · sin(fy · t + phase)
 *
 *   With fx:fy = 3:2 this traces the canonical 6-lobed figure-eight.
 *   Centre + radii are derived from scene extent so the curve scales
 *   automatically with terminal resize.
 */
static void lissajous_eval_at_phase(
    float t, float scene_w, float scene_h,
    float *out_x_cells, float *out_y_cells
) {
    float cx = scene_w * OBS_CENTER_X_FRAC;
    float cy = scene_h * OBS_CENTER_Y_FRAC;
    float rx = scene_w * OBS_RADIUS_X_FRAC;
    float ry = scene_h * OBS_RADIUS_Y_FRAC;
    *out_x_cells = cx + rx * sinf(LISSAJOUS_FREQ_X * t);
    *out_y_cells = cy + ry * sinf(LISSAJOUS_FREQ_Y * t + LISSAJOUS_PHASE_RAD);
}

/* Advance a Lissajous phase by one LISSAJOUS_SPEED tick, wrapping at 2π
 * so the float never drifts unbounded. */
static float lissajous_advance_phase(float t)
{
    t += LISSAJOUS_SPEED;
    if (t >= (float)(2.0 * M_PI)) t -= (float)(2.0 * M_PI);
    return t;
}

/*
 * scene_update_observer — move the eye one tick along the Lissajous
 * curve.
 *
 *   Pseudocode:
 *     (x, y) := lissajous_eval_at_phase(lissajous_t, scene_w, scene_h)
 *     observer.{x_cells, y_cells} := (x, y)
 *     lissajous_t := lissajous_advance_phase(lissajous_t)
 */
static void scene_update_observer(Scene *s)
{
    lissajous_eval_at_phase(
        s->observer.lissajous_t,
        (float)s->scene_cols, (float)s->scene_rows,
        &s->observer.x_cells, &s->observer.y_cells);
    s->observer.lissajous_t = lissajous_advance_phase(s->observer.lissajous_t);
}

/* Drawable extent = terminal extent minus the top + bottom HUD rows. */
static void compute_scene_extent_from_terminal(
    Scene *s, int term_cols, int term_rows
) {
    s->scene_cols = term_cols;
    s->scene_rows = term_rows - HUD_ROWS;
}

/* Seed defaults for SimControls (paused + fps).  Called once at init. */
static void sim_controls_seed_defaults(SimControls *sim)
{
    sim->paused = false;
    sim->fps    = SIM_FPS_DEFAULT;
}

/* Seed defaults for Observer (Lissajous phase + sweep ray angle +
 * eye at scene centre).  scene_update_observer will move the eye once
 * the first tick fires. */
static void observer_seed_at_center(Observer *o, float scene_w, float scene_h)
{
    o->lissajous_t     = 0.0f;
    o->sweep_angle_rad = -(float)M_PI;
    o->x_cells         = scene_w * OBS_CENTER_X_FRAC;
    o->y_cells         = scene_h * OBS_CENTER_Y_FRAC;
}

/*
 * scene_init — zero, seed defaults, build walls, compute first polygon.
 *
 *   Pseudocode:
 *     memset scene to zero
 *     compute drawable extent (term − HUD)
 *     seed sim controls (paused=false, fps=default)
 *     seed observer at scene centre with Lissajous phase = 0
 *     scene_build_walls(s)
 *     compute_visibility(observer, walls, &vis)   ← first frame ready
 */
static void scene_init(Scene *s, int term_cols, int term_rows)
{
    memset(s, 0, sizeof(*s));
    compute_scene_extent_from_terminal(s, term_cols, term_rows);
    sim_controls_seed_defaults(&s->sim);
    observer_seed_at_center(&s->observer,
                            (float)s->scene_cols, (float)s->scene_rows);
    scene_build_walls(s);
    compute_visibility(s->observer.x_cells, s->observer.y_cells,
                       s->walls.list, s->walls.count, &s->vis);
}

/* Advance the spinning '*' ray by one tick, wrapping into (−π, π]. */
static float sweep_ray_advance_angle(float angle_rad)
{
    angle_rad += SWEEP_SPEED_RAD;
    if (angle_rad > (float)M_PI) angle_rad -= (float)(2.0 * M_PI);
    return angle_rad;
}

/*
 * scene_tick — advance one simulation step.
 *
 *   Pseudocode:
 *     scene_update_observer(s)             ← Lissajous one step forward
 *     observer.sweep_angle_rad := sweep_ray_advance_angle(…)
 *     compute_visibility(observer, walls, &vis)   ← refresh polygon
 */
static void scene_tick(Scene *s)
{
    scene_update_observer(s);
    s->observer.sweep_angle_rad =
        sweep_ray_advance_angle(s->observer.sweep_angle_rad);
    compute_visibility(s->observer.x_cells, s->observer.y_cells,
                       s->walls.list, s->walls.count, &s->vis);
}

/* ===================================================================== */
/* S7  render                                                             */
/* ===================================================================== */

/*
 * fill_char_for_distance -- encode depth as character density.
 * A bright dense ring near the observer fades to sparse dots at the edges,
 * giving a "light cone emanating from the observer" feel.
 *
 * dist_fraction: cell_dist_geo / scene_diagonal_geo, clamped to [0,1].
 */
static char fill_char_for_distance(float dist_fraction)
{
    if (dist_fraction < FILL_DIST_FRAC_DENSE_LIMIT) return '@';  /* dense ring */
    if (dist_fraction < FILL_DIST_FRAC_NEAR_LIMIT)  return 'o';
    if (dist_fraction < FILL_DIST_FRAC_MID_LIMIT)   return '+';
    if (dist_fraction < FILL_DIST_FRAC_FAR_LIMIT)   return '.';
    return ' ';                                                  /* fade out  */
}

/* color_pair_for_distance -- complement fill_char_for_distance with color. */
static int color_pair_for_distance(float dist_fraction)
{
    if (dist_fraction < COLOR_DIST_FRAC_NEAR_LIMIT) return CP_FILL_NEAR;
    if (dist_fraction < COLOR_DIST_FRAC_MID_LIMIT)  return CP_FILL_MID;
    return CP_FILL_FAR;
}

/*
 * scene_addch -- bounds-checked mvaddch that translates scene-y into
 * terminal-y.  Scene coords are 0..scene_rows-1; HUD_TOP_ROWS reserves
 * row 0 for the top status bar, so the actual terminal row is
 * (scene_y + HUD_TOP_ROWS).  All four scene-coord draw sites route
 * through this helper.
 */
static void scene_addch(const Scene *s, int scene_y, int scene_x, char ch)
{
    if (scene_x < 0 || scene_x >= s->scene_cols) return;
    if (scene_y < 0 || scene_y >= s->scene_rows) return;
    mvaddch(scene_y + HUD_TOP_ROWS, scene_x, (chtype)ch);
}

/*
 * draw_line_bresenham -- draw character c from (x0,y0) to (x1,y1) using
 * Bresenham's algorithm in SCENE coordinates; scene_addch handles the
 * HUD_TOP_ROWS y-translation and clipping.
 */
static void draw_line_bresenham(
    const Scene *s,
    int x0, int y0, int x1, int y1,
    int color_pair, int attrs, char c
) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    attron(COLOR_PAIR(color_pair) | attrs);
    while (1) {
        scene_addch(s, y0, x0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
    attroff(COLOR_PAIR(color_pair) | attrs);
}

/*
 * wall_display_char -- choose a line character based on wall slope.
 * Nearly-horizontal walls get '-', nearly-vertical get '|',
 * diagonal walls get '/' or '\'.
 */
static char wall_display_char(const Wall *w)
{
    float dx  = w->x1_cells - w->x0_cells;
    float dy  = w->y1_cells - w->y0_cells;
    float adx = fabsf(dx), ady = fabsf(dy);
    if (ady < adx * WALL_AXIS_RATIO_THRESHOLD) return '-';   /* horizontal */
    if (adx < ady * WALL_AXIS_RATIO_THRESHOLD) return '|';   /* vertical   */
    return (dx * dy > 0.0f) ? '\\' : '/';                    /* diagonal   */
}

/*
 * draw_sweep_ray -- draw the animated rotating ray from observer to nearest wall.
 * This is a visual teaching aid showing "the scan is sweeping around 360 degrees".
 * It is computed independently of the visibility polygon.
 */
static void draw_sweep_ray(const Scene *s)
{
    SweepHit ray_hit;
    bool hit = cast_ray_in_direction(
        s->observer.x_cells, s->observer.y_cells,
        s->observer.sweep_angle_rad,
        s->walls.list, s->walls.count,
        &ray_hit
    );
    if (!hit) return;

    int x0 = (int)roundf(s->observer.x_cells);
    int y0 = (int)roundf(s->observer.y_cells);
    int x1 = (int)roundf(ray_hit.hit_x_cells);
    int y1 = (int)roundf(ray_hit.hit_y_cells);

    /* Draw ray, but skip the observer cell itself (drawn later as '@') */
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    attron(COLOR_PAIR(CP_SWEEP_RAY));
    while (1) {
        bool is_observer_cell = (x0 == (int)roundf(s->observer.x_cells) &&
                                 y0 == (int)roundf(s->observer.y_cells));
        if (!is_observer_cell) scene_addch(s, y0, x0, '*');
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
    attroff(COLOR_PAIR(CP_SWEEP_RAY));
}

/*
 * render_scene -- main per-frame draw.
 *
 * Pass 1: iterate every cell in the scene.  If visible, draw a fill
 *         character whose density and color encode the observer distance.
 * Pass 2: draw the animated sweep ray ('*').
 * Pass 3: draw interior walls on top of the fill.
 * Pass 4: draw bounding box (dim, behind everything visually).
 * Pass 5: draw observer '@' topmost.
 *
 * Per-cell cost: O(log N) binary search + O(1) intersection.
 * Total frame cost: O(scene_cells * log N).
 */
/* Scene diagonal in geo space — the normaliser for "fraction of the
 * farthest possible distance".  Used by paint_visible_cells to map
 * raw distance into [0, 1] for the fill_char + colour_pair pickers. */
static float scene_diagonal_geo(const Scene *s)
{
    float w = (float)s->scene_cols;
    float h = geo_y((float)s->scene_rows);
    return sqrtf(w * w + h * h);
}

/* Centre of cell (row, col) in cell space — the place the ray actually
 * tests, not the top-left corner. */
static void cell_center_in_cell_space(int row, int col,
                                      float *out_cx, float *out_cy)
{
    *out_cx = (float)col + CELL_CENTER_OFFSET;
    *out_cy = (float)row + CELL_CENTER_OFFSET;
}

/* Distance from observer to cell centre in geo space, normalised by
 * the scene diagonal (so 0.0 = observer cell, 1.0 = farthest cell). */
static float cell_distance_fraction(
    float cell_cx, float cell_cy,
    float obs_x_cells, float obs_y_cells, float scene_diag_geo
) {
    float dx = cell_cx - obs_x_cells;
    float dy = geo_y(cell_cy) - geo_y(obs_y_cells);
    float dist = sqrtf(dx * dx + dy * dy);
    return (scene_diag_geo > 0.0f) ? dist / scene_diag_geo : 0.0f;
}

/* Paint one cell in the appropriate fill glyph + colour pair derived
 * from its distance fraction.  Skips writes when the glyph is ' '. */
static void paint_cell_with_distance_falloff(
    const Scene *s, int row, int col, float dist_fraction
) {
    char fill = fill_char_for_distance(dist_fraction);
    if (fill == ' ') return;
    int cpair = color_pair_for_distance(dist_fraction);
    attron(COLOR_PAIR(cpair));
    scene_addch(s, row, col, fill);
    attroff(COLOR_PAIR(cpair));
}

/*
 * paint_visible_cells_with_distance_falloff — Pass 1 of render_scene.
 *
 *   Pseudocode (per cell of the scene grid):
 *     (cx, cy) := cell centre in cell space
 *     if not cell_is_visible(cx, cy):  skip
 *     frac     := cell_distance_fraction(...)
 *     paint cell with the falloff glyph + colour
 */
static void paint_visible_cells_with_distance_falloff(const Scene *s)
{
    float scene_diag = scene_diagonal_geo(s);
    for (int row = 0; row < s->scene_rows; row++) {
        for (int col = 0; col < s->scene_cols; col++) {
            float cx, cy;
            cell_center_in_cell_space(row, col, &cx, &cy);
            if (!cell_is_visible(cx, cy,
                                 s->observer.x_cells, s->observer.y_cells,
                                 &s->vis)) continue;
            float frac = cell_distance_fraction(cx, cy,
                                                s->observer.x_cells,
                                                s->observer.y_cells,
                                                scene_diag);
            paint_cell_with_distance_falloff(s, row, col, frac);
        }
    }
}

/* Paint one Wall as a Bresenham line in (color_pair, attr). */
static void paint_wall_line(
    const Scene *s, const Wall *w, int color_pair, int attr
) {
    char wc = wall_display_char(w);
    draw_line_bresenham(s,
        (int)roundf(w->x0_cells), (int)roundf(w->y0_cells),
        (int)roundf(w->x1_cells), (int)roundf(w->y1_cells),
        color_pair, attr, wc);
}

/* Pass 3 — interior walls (bold). */
static void paint_interior_walls(const Scene *s)
{
    for (int i = BOUNDING_WALL_COUNT; i < s->walls.count; i++)
        paint_wall_line(s, &s->walls.list[i], CP_WALL, A_BOLD);
}

/* Pass 4 — outer bounding box (dim, drawn AFTER interior so it doesn't
 * occlude interior junctions). */
static void paint_bounding_box(const Scene *s)
{
    for (int i = 0; i < BOUNDING_WALL_COUNT; i++)
        paint_wall_line(s, &s->walls.list[i], CP_BOUND, A_DIM);
}

/* Pass 5 — observer '@' glyph on top of everything else. */
static void paint_observer_glyph(const Scene *s)
{
    int obs_col = (int)roundf(s->observer.x_cells);
    int obs_row = (int)roundf(s->observer.y_cells);
    attron(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
    scene_addch(s, obs_row, obs_col, '@');
    attroff(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
}

/*
 * render_scene — main per-frame draw, five passes painter-order.
 *
 *   Pseudocode:
 *     Pass 1  paint_visible_cells_with_distance_falloff   (depth fill)
 *     Pass 2  draw_sweep_ray                              ('*' overlay)
 *     Pass 3  paint_interior_walls                        (bold)
 *     Pass 4  paint_bounding_box                          (dim outer box)
 *     Pass 5  paint_observer_glyph                        ('@' on top)
 *
 *   Cost per frame: O(scene_cells · log n_hits) + O(walls · cell_len).
 */
static void render_scene(const Scene *s)
{
    paint_visible_cells_with_distance_falloff(s);
    draw_sweep_ray(s);
    paint_interior_walls(s);
    paint_bounding_box(s);
    paint_observer_glyph(s);
}

/*
 * draw_hud -- canonical project HUD.
 *
 *   Top row 0       (CP_HUD,  A_BOLD)   right-aligned data line:
 *       visibility %, rays cast, wall count, fps, paused/running.
 *   Bottom row R-1  (CP_HINT, A_BOLD)   left-aligned key bindings.
 *
 *   Layout follows the project HUD Standard (CLAUDE.md): data on top,
 *   actions on bottom, both bright and bold so they remain legible
 *   over any animation cell underneath.
 */
/* visible_polygon_area / scene_area as a percentage; 0 if scene degenerate. */
static float visible_area_percent(const Scene *s)
{
    float scene_area = (float)s->scene_cols * (float)s->scene_rows;
    return (scene_area > 0.0f)
        ? 100.0f * s->vis.visible_area_cells / scene_area
        : 0.0f;
}

/* One-word run-state indicator used in the top status string. */
static const char *run_state_label(const Scene *s)
{
    return s->sim.paused ? "PAUSED " : "running";
}

/* Format the right-aligned top status line into `buf`. */
static void format_hud_status(const Scene *s, char *buf, size_t bufsz)
{
    snprintf(buf, bufsz,
        " vis:%4.0fc^2 (%4.1f%%)  rays:%3d  walls:%2d  fps:%2d  %s ",
        s->vis.visible_area_cells, visible_area_percent(s),
        s->vis.n_hits, s->walls.count, s->sim.fps, run_state_label(s));
}

/* Paint the right-aligned status line on row 0 (CP_HUD + A_BOLD). */
static void draw_top_status(const char *status, int term_cols)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    int pad = term_cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Paint the bottom action-hint strip listing every interactive key. */
static void draw_bottom_hint(int term_rows)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
        " q:quit  spc:pause  r:reset  n:new-walls  +/-:speed ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * draw_hud — canonical project HUD (top status + bottom hints).
 *
 *   Pseudocode:
 *     read terminal extent
 *     format right-aligned status line
 *     draw top status        (row 0,        CP_HUD,  A_BOLD)
 *     draw bottom action hint(row R−1,      CP_HINT, A_BOLD)
 */
static void draw_hud(const Scene *s)
{
    int term_rows, term_cols;
    getmaxyx(stdscr, term_rows, term_cols);

    char status[128];
    format_hud_status(s, status, sizeof status);
    draw_top_status(status, term_cols);
    draw_bottom_hint(term_rows);
}

/* ===================================================================== */
/* S8  screen                                                             */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    nodelay(stdscr, TRUE);
    color_init();
}

static void screen_teardown(void) { endwin(); }

/* ===================================================================== */
/* S9  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_resize_pending = 0;
static volatile sig_atomic_t g_quit           = 0;

static void handle_sigwinch(int sig) { (void)sig; g_resize_pending = 1; }
static void handle_sigint  (int sig) { (void)sig; g_quit           = 1; }

/* Toggle the simulation paused flag. */
static void sim_toggle_pause(Scene *s) { s->sim.paused = !s->sim.paused; }

/* Reset the entire scene at the current terminal extent. */
static void scene_reset_at_current_extent(Scene *s)
{
    scene_init(s, s->scene_cols, s->scene_rows + HUD_ROWS);
}

/* Bump simulation fps up by SIM_FPS_STEP, capped at SIM_FPS_MAX. */
static void sim_fps_increase(Scene *s)
{
    if (s->sim.fps < SIM_FPS_MAX) s->sim.fps += SIM_FPS_STEP;
}

/* Bump simulation fps down by SIM_FPS_STEP, floored at SIM_FPS_MIN. */
static void sim_fps_decrease(Scene *s)
{
    if (s->sim.fps > SIM_FPS_MIN) s->sim.fps -= SIM_FPS_STEP;
}

/*
 * route_key_to_action — translate one keystroke into a Scene mutation.
 *
 *   q / Q / ESC  → request quit (set g_quit)
 *   SPACE        → toggle pause
 *   r / R        → reset scene at current extent
 *   n / N        → regenerate random wall layout
 *   + / =        → faster simulation tempo
 *   - / _        → slower simulation tempo
 */
static void route_key_to_action(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: g_quit = 1;             break;
    case ' ':                         sim_toggle_pause(s);    break;
    case 'r': case 'R':               scene_reset_at_current_extent(s); break;
    case 'n': case 'N':               scene_generate_random_walls(s);   break;
    case '+': case '=':               sim_fps_increase(s);    break;
    case '-': case '_':               sim_fps_decrease(s);    break;
    default: break;
    }
}

/* Drain every pending key from ncurses and route each through
 * route_key_to_action.  ncurses is in non-blocking mode so getch()
 * returns ERR when the buffer is empty. */
static void app_handle_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR) route_key_to_action(s, ch);
}

/* ── one-shot init helpers for main() ────────────────────────────────── */

/* Install the POSIX signal handlers used by the loop. */
static void signals_install(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGINT,   handle_sigint);
}

/* If the SIGWINCH handler set g_resize_pending, tear ncurses down +
 * back up so stdscr matches the new terminal extent, then rebuild the
 * scene against the new size. */
static void handle_resize_if_pending(Scene *s,
                                     int64_t *sim_accum_ns,
                                     int64_t *frame_prev_ns)
{
    if (!g_resize_pending) return;
    g_resize_pending = 0;
    endwin();
    refresh();
    int term_cols, term_rows;
    getmaxyx(stdscr, term_rows, term_cols);
    scene_init(s, term_cols, term_rows);
    *frame_prev_ns = clock_ns();
    *sim_accum_ns  = 0;
}

/*
 * advance_fixed_timestep_sim — Fiedler's accumulator pattern.
 *
 *   Pseudocode:
 *     if paused: return
 *     accum += dt
 *     while accum ≥ tick_period:
 *       scene_tick(s)
 *       accum -= tick_period
 *
 *   Keeps physics deterministic regardless of render frame rate.
 */
static void advance_fixed_timestep_sim(Scene *s,
                                       int64_t *sim_accum_ns, int64_t dt_ns)
{
    if (s->sim.paused) return;
    *sim_accum_ns += dt_ns;
    int64_t tick_ns = TICK_NS(s->sim.fps);
    while (*sim_accum_ns >= tick_ns) {
        scene_tick(s);
        *sim_accum_ns -= tick_ns;
    }
}

/* Render one frame: erase, scene, HUD, mark dirty, single-write flush. */
static void frame_render(const Scene *s)
{
    erase();
    render_scene(s);
    draw_hud(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Sleep just long enough to keep this frame on its TARGET_FPS budget. */
static void wait_until_next_frame(int64_t frame_start_ns)
{
    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(TICK_NS(TARGET_FPS) - elapsed);
}

/*
 * main — wire signals + ncurses + Scene, then run the canonical loop:
 *
 *   Pseudocode:
 *     install signal handlers
 *     bring up terminal (ncurses + colour)
 *     allocate Scene on stack, seed at current terminal extent
 *
 *     while not quit:
 *       handle resize if pending
 *       compute dt since last frame
 *       advance the fixed-timestep simulation
 *       drain input
 *       render one frame
 *       wait until next frame boundary
 */
int main(void)
{
    signals_install();
    srand((unsigned)time(NULL));
    screen_init();

    Scene scene;
    int term_cols, term_rows;
    getmaxyx(stdscr, term_rows, term_cols);
    scene_init(&scene, term_cols, term_rows);

    int64_t sim_accum_ns  = 0;
    int64_t frame_prev_ns = clock_ns();

    while (!g_quit) {
        handle_resize_if_pending(&scene, &sim_accum_ns, &frame_prev_ns);

        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - frame_prev_ns;
        frame_prev_ns  = now_ns;

        advance_fixed_timestep_sim(&scene, &sim_accum_ns, dt_ns);
        app_handle_input(&scene);
        frame_render(&scene);
        wait_until_next_frame(now_ns);
    }

    screen_teardown();
    return 0;
}
