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
 *  S6  scene        -- obstacle layout, observer Lissajous path, scene_tick()
 *  S7  render       -- render_scene(), render_overlay()
 *  S8  screen       -- ncurses init / teardown
 *  S9  app          -- signals, input, main loop
 * -------------------------------------------------------------------------
 *
 * Keys:  SPACE pause/resume   r reset   +/- speed   q quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       geometry/visibility_polygon.c -o vp -lncurses -lm
 */

/* -- CONCEPTS -------------------------------------------------------------- *
 *
 * Angular sweep algorithm (S5 compute_visibility):
 *   The visibility polygon from observer O is the set of all points P such
 *   that the segment OP does not cross any wall.  It is computed by sweeping
 *   a ray 360 degrees and recording where each angle first hits a wall.
 *
 *   Naive approach: cast N uniformly-spaced rays (approximate; misses corners).
 *   Exact approach: only cast rays at wall-endpoint angles, plus a tiny eps
 *   on each side.  Between consecutive endpoint angles the nearest wall cannot
 *   change, so the boundary is always a straight line from hit[i] to hit[i+1].
 *   This gives the *exact* polygon in O(E log E) time (E = endpoint count).
 *
 * Epsilon trick at corners (S5 cast_ray_in_direction):
 *   For each endpoint angle theta, cast three rays: theta-eps, theta, theta+eps.
 *   The eps-offset rays handle both sides of a wall corner correctly:
 *   - theta-eps sees the wall *ending* at the corner
 *   - theta+eps sees the wall *beginning* past the corner (may differ)
 *   Without eps, a ray aimed exactly at a shared corner gives ambiguous results.
 *
 * Occlusion detection -- ray vs. segment (S4 ray_hits_wall):
 *   Ray:     P(t) = O + t*D          (parameter t, want t > 0)
 *   Segment: Q(s) = A + s*(B-A)      (parameter s, want 0 <= s <= 1)
 *   Solve O + t*D = A + s*(B-A):
 *     denom = D x (B-A)              (2D cross product = scalar)
 *     t     = (A-O) x (B-A) / denom
 *     s     = (A-O) x D     / denom
 *   The wall at minimum valid t is the occluder for that ray direction.
 *
 * Aspect ratio correction (S4 geo_y, cell_y):
 *   Terminal cells are ~2x taller than wide (CELL_H=16, CELL_W=8 pixels).
 *   Distances and angles computed with raw row/col numbers are metrically
 *   distorted.  All geometry here uses "geo space" where y is doubled:
 *     y_geo = y_cells * ASPECT_Y      (stretch to square pixels)
 *   Results are converted back to cell space only for rendering.
 *
 * Visible area (S7 render_overlay):
 *   Shoelace formula on the sorted hit points (in geo space):
 *     area_geo = 0.5 * |sum_i (x_i * y_{i+1} - x_{i+1} * y_i)|
 *   Cell-space area = area_geo / ASPECT_Y  (y was stretched, so divide back).
 * --------------------------------------------------------------------------- */

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

/* HUD at the bottom: this many terminal rows are reserved for the overlay. */
#define HUD_ROWS   8

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

    /* HUD text layers */
    CP_HUD,          /* normal metric labels: light grey                 */
    CP_HUD_VALUE,    /* numeric values: bright cyan                      */
    CP_HUD_HEADER,   /* section title: bright yellow                     */
    CP_HUD_EXPLAIN,  /* algorithm explanation: medium grey               */
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
        init_pair(CP_HUD,         252,  -1);
        init_pair(CP_HUD_VALUE,    51,  -1);
        init_pair(CP_HUD_HEADER,  226,  -1);
        init_pair(CP_HUD_EXPLAIN, 244,  -1);
    } else {
        init_pair(CP_FILL_NEAR,  COLOR_WHITE,   -1);
        init_pair(CP_FILL_MID,   COLOR_YELLOW,  -1);
        init_pair(CP_FILL_FAR,   COLOR_CYAN,    -1);
        init_pair(CP_WALL,       COLOR_WHITE,   -1);
        init_pair(CP_BOUND,      COLOR_WHITE,   -1);
        init_pair(CP_OBSERVER,   COLOR_MAGENTA, -1);
        init_pair(CP_SWEEP_RAY,  COLOR_YELLOW,  -1);
        init_pair(CP_HUD,        COLOR_WHITE,   -1);
        init_pair(CP_HUD_VALUE,  COLOR_CYAN,    -1);
        init_pair(CP_HUD_HEADER, COLOR_YELLOW,  -1);
        init_pair(CP_HUD_EXPLAIN,COLOR_WHITE,   -1);
    }
}

/* ===================================================================== */
/* S4  geometry                                                           */
/* ===================================================================== */

/*
 * Wall: a line segment in cell coordinates.
 * The _cells suffix reminds callers that (x, y) are terminal column/row.
 */
typedef struct {
    float x0_cells, y0_cells;   /* segment start: (col, row) */
    float x1_cells, y1_cells;   /* segment end:   (col, row) */
} Wall;

/*
 * WallSpec: wall expressed as fractions of scene dimensions [0.0, 1.0].
 * Decoupled from terminal size; instantiated to Wall after resize.
 * Using fractions makes the obstacle layout scale to any terminal.
 */
typedef struct {
    float x0_frac, y0_frac;  /* start: fraction of (scene_cols, scene_rows) */
    float x1_frac, y1_frac;  /* end:   fraction of (scene_cols, scene_rows) */
} WallSpec;

/*
 * SweepHit: one sample from the radial visibility sweep.
 * angle_rad is the key for sorting; hit_x/y_cells is where the ray landed.
 * Sorted hits in order form the boundary polygon of visible space.
 */
typedef struct {
    float angle_rad;      /* direction of this ray from observer, in geo space */
    float hit_x_cells;    /* x of the ray's first wall contact, in cell coords */
    float hit_y_cells;    /* y of the ray's first wall contact, in cell coords */
} SweepHit;

/*
 * VisibilityPolygon: complete result of one compute_visibility() call.
 * hits[] is sorted by angle_rad, forming the star-shaped boundary polygon.
 * visible_area_cells: polygon area in cell^2 units (via shoelace formula).
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
static bool cast_ray_in_direction(
    float obs_x_cells, float obs_y_cells,
    float ray_angle_rad,
    const Wall *all_walls, int n_walls,
    SweepHit   *out_hit
) {
    /* Observer in geo space: x unaffected, y stretched by ASPECT_Y */
    float obs_x_geo = obs_x_cells;
    float obs_y_geo = geo_y(obs_y_cells);

    float dir_x_geo = cosf(ray_angle_rad);
    float dir_y_geo = sinf(ray_angle_rad);

    /* Scan all walls; keep the smallest positive t (nearest occluder) */
    float nearest_t_geo = 1e30f;

    for (int wall_idx = 0; wall_idx < n_walls; wall_idx++) {
        const Wall *w = &all_walls[wall_idx];

        float ax_geo = w->x0_cells;
        float ay_geo = geo_y(w->y0_cells);
        float bx_geo = w->x1_cells;
        float by_geo = geo_y(w->y1_cells);

        float t_hit = ray_hits_wall(
            obs_x_geo, obs_y_geo,
            dir_x_geo, dir_y_geo,
            ax_geo, ay_geo,
            bx_geo, by_geo
        );

        if (t_hit > 0.0f && t_hit < nearest_t_geo)
            nearest_t_geo = t_hit;
    }

    if (nearest_t_geo >= 1e29f) return false;  /* no hit -- open scene? */

    /* Convert hit point from geo space back to cell space */
    float hit_x_geo = obs_x_geo + nearest_t_geo * dir_x_geo;
    float hit_y_geo = obs_y_geo + nearest_t_geo * dir_y_geo;

    out_hit->angle_rad   = ray_angle_rad;
    out_hit->hit_x_cells = hit_x_geo;           /* x is identical in both spaces */
    out_hit->hit_y_cells = cell_y(hit_y_geo);   /* y divided back to cell space  */
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
static void compute_visibility(
    float obs_x_cells, float obs_y_cells,
    const Wall *all_walls, int n_walls,
    VisibilityPolygon *out_vp
) {
    float obs_x_geo = obs_x_cells;
    float obs_y_geo = geo_y(obs_y_cells);

    /* --- Step 1: collect candidate ray angles at each endpoint --- */
    float candidate_angle_rad[MAX_SWEEP_RAYS];
    int   n_candidates = 0;
    int   max_candidates = MAX_SWEEP_RAYS;  /* static cap */

    for (int wall_idx = 0; wall_idx < n_walls; wall_idx++) {
        const Wall *w = &all_walls[wall_idx];

        /* Two endpoints per wall; compute angle from observer to each */
        float ep_x_geo[2] = { w->x0_cells, w->x1_cells };
        float ep_y_geo[2] = { geo_y(w->y0_cells), geo_y(w->y1_cells) };

        for (int ep = 0; ep < 2; ep++) {
            float endpoint_dx_geo = ep_x_geo[ep] - obs_x_geo;
            float endpoint_dy_geo = ep_y_geo[ep] - obs_y_geo;
            float endpoint_angle_rad = atan2f(endpoint_dy_geo, endpoint_dx_geo);

            /* Bracket the endpoint angle with -eps and +eps rays */
            if (n_candidates + 3 <= max_candidates) {
                candidate_angle_rad[n_candidates++] =
                    endpoint_angle_rad - ENDPOINT_ANGLE_EPS_RAD;
                candidate_angle_rad[n_candidates++] =
                    endpoint_angle_rad;
                candidate_angle_rad[n_candidates++] =
                    endpoint_angle_rad + ENDPOINT_ANGLE_EPS_RAD;
            }
        }
    }

    /* --- Step 2: cast each candidate ray and record hits --- */
    out_vp->n_hits = 0;

    for (int ray_idx = 0; ray_idx < n_candidates; ray_idx++) {
        SweepHit hit;
        if (cast_ray_in_direction(
                obs_x_cells, obs_y_cells,
                candidate_angle_rad[ray_idx],
                all_walls, n_walls,
                &hit)
            && out_vp->n_hits < MAX_SWEEP_RAYS)
        {
            out_vp->hits[out_vp->n_hits++] = hit;
        }
    }

    /* --- Step 3: sort by angle to form ordered polygon boundary --- */
    qsort(out_vp->hits, out_vp->n_hits,
          sizeof(SweepHit), cmp_sweep_hit_by_angle);

    /* --- Shoelace area in geo space, then convert to cell space --- */
    float shoelace_sum = 0.0f;
    int n = out_vp->n_hits;
    for (int i = 0; i < n; i++) {
        int   j  = (i + 1) % n;
        float xi = out_vp->hits[i].hit_x_cells;
        float yi = geo_y(out_vp->hits[i].hit_y_cells);
        float xj = out_vp->hits[j].hit_x_cells;
        float yj = geo_y(out_vp->hits[j].hit_y_cells);
        shoelace_sum += xi * yj - xj * yi;
    }
    /* area_geo / ASPECT_Y recovers cell-space area (y was stretched) */
    out_vp->visible_area_cells = 0.5f * fabsf(shoelace_sum) / ASPECT_Y;
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
static bool cell_is_visible(
    float cell_x_cells, float cell_y_cells,
    float obs_x_cells,  float obs_y_cells,
    const VisibilityPolygon *vp
) {
    if (vp->n_hits < 3) return false;

    /* Displacement and distance in geo space */
    float dx_geo        = cell_x_cells - obs_x_cells;
    float dy_geo        = geo_y(cell_y_cells) - geo_y(obs_y_cells);
    float cell_dist_sq  = dx_geo * dx_geo + dy_geo * dy_geo;
    if (cell_dist_sq < 0.01f) return true;   /* observer's own cell -- always visible */

    float cell_angle_rad = atan2f(dy_geo, dx_geo);

    /* Determine which sector the cell angle falls in */
    int sector_idx, next_idx;
    float first_angle = vp->hits[0].angle_rad;
    float last_angle  = vp->hits[vp->n_hits - 1].angle_rad;

    if (cell_angle_rad < first_angle || cell_angle_rad >= last_angle) {
        /* Wrap-around sector: between the highest angle and the lowest */
        sector_idx = vp->n_hits - 1;
        next_idx   = 0;
    } else {
        sector_idx = find_angle_sector(vp, cell_angle_rad);
        next_idx   = sector_idx + 1;          /* safe: angle < last_angle guarantees next exists */
    }

    /* Boundary segment endpoints relative to observer, in geo space */
    float bnd0_x = vp->hits[sector_idx].hit_x_cells - obs_x_cells;
    float bnd0_y = geo_y(vp->hits[sector_idx].hit_y_cells) - geo_y(obs_y_cells);
    float bnd1_x = vp->hits[next_idx].hit_x_cells   - obs_x_cells;
    float bnd1_y = geo_y(vp->hits[next_idx].hit_y_cells) - geo_y(obs_y_cells);

    float dir_x = cosf(cell_angle_rad);
    float dir_y = sinf(cell_angle_rad);

    /* Intersect ray with the polygon boundary segment */
    float boundary_t = ray_hits_wall(
        0.0f, 0.0f,
        dir_x, dir_y,
        bnd0_x, bnd0_y,
        bnd1_x, bnd1_y
    );

    if (boundary_t < 0.0f) {
        /* Degenerate sector -- fall back: visible if within bnd0 distance */
        float bnd0_dist = sqrtf(bnd0_x * bnd0_x + bnd0_y * bnd0_y);
        return sqrtf(cell_dist_sq) <= bnd0_dist + 1.0f;
    }

    /* 0.8 geo-unit tolerance so cells exactly on the polygon edge appear lit */
    return sqrtf(cell_dist_sq) <= boundary_t + 0.8f;
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
 * Scene: all mutable simulation state.
 * walls[]:    active Wall segments in cell coordinates.
 * obs_*_cells: observer position (updated each tick).
 * lissajous_t: phase parameter for the Lissajous path [0, 2*PI).
 * sweep_angle_rad: independent rotating ray angle [−PI, PI).
 * vis:         visibility polygon computed at the current observer position.
 */
typedef struct {
    Wall  walls[MAX_WALLS];
    int   n_walls;

    int   scene_cols;     /* drawable columns (terminal cols)            */
    int   scene_rows;     /* drawable rows (terminal rows minus HUD)     */

    float obs_x_cells;    /* observer column (cell space)                */
    float obs_y_cells;    /* observer row    (cell space)                */
    float lissajous_t;    /* Lissajous path phase parameter              */

    float sweep_angle_rad;/* animated sweep ray direction                */

    VisibilityPolygon vis; /* current visibility polygon                 */

    /* Random wall pattern generated by the 'n' key */
    WallSpec rand_wall_specs[INTERIOR_WALL_COUNT]; /* fractions, rebuilt on resize */
    bool     use_rand_walls;  /* true after first 'n' press; false = default layout */

    bool  paused;
    int   sim_fps;
} Scene;

static Scene g_scene;

/*
 * scene_build_walls -- instantiate WallSpec fractions into cell-space Walls.
 * Must be called after scene_cols / scene_rows are set (init or resize).
 */
static void scene_build_walls(Scene *s)
{
    float sw = (float)s->scene_cols;
    float sh = (float)s->scene_rows;
    s->n_walls = 0;

    for (int i = 0; i < BOUNDING_WALL_COUNT; i++) {
        const WallSpec *spec = &BOUNDING_WALL_SPECS[i];
        Wall *w = &s->walls[s->n_walls++];
        w->x0_cells = spec->x0_frac * sw;
        w->y0_cells = spec->y0_frac * sh;
        w->x1_cells = spec->x1_frac * sw;
        w->y1_cells = spec->y1_frac * sh;
    }

    /* Dispatch: use random specs if 'n' was pressed, else default layout */
    const WallSpec *interior = s->use_rand_walls
                               ? s->rand_wall_specs
                               : INTERIOR_WALL_SPECS;

    for (int i = 0; i < INTERIOR_WALL_COUNT; i++) {
        const WallSpec *spec = &interior[i];
        Wall *w = &s->walls[s->n_walls++];
        w->x0_cells = spec->x0_frac * sw;
        w->y0_cells = spec->y0_frac * sh;
        w->x1_cells = spec->x1_frac * sw;
        w->y1_cells = spec->y1_frac * sh;
    }
}

/*
 * scene_generate_random_walls -- fill rand_wall_specs[] with new random segments.
 *
 * Each wall is defined by a random anchor point and a random angle + half-length.
 * Anchors that fall inside the observer's Lissajous safe zone are rejected (up to
 * MAX_PLACEMENT_TRIES retries) so the observer can always move freely.
 *
 * Lengths are drawn uniformly from [RAND_WALL_LEN_MIN, RAND_WALL_LEN_MAX] as
 * fractions of the smaller scene dimension, giving walls that feel proportional
 * regardless of terminal size.
 */

/* Fraction-space rectangle centred at (0.5, 0.5) that must stay clear of walls */
#define OBS_SAFE_HALF_X   0.20f   /* slightly wider than OBS_RADIUS_X_FRAC       */
#define OBS_SAFE_HALF_Y   0.18f   /* slightly taller than OBS_RADIUS_Y_FRAC      */

#define RAND_WALL_LEN_MIN  0.07f  /* shortest random wall as fraction of scene   */
#define RAND_WALL_LEN_MAX  0.28f  /* longest  random wall as fraction of scene   */
#define MAX_PLACEMENT_TRIES 30    /* retries before giving up on safe placement  */

static float rand_frac(void) { return (float)rand() / (float)RAND_MAX; }

static void scene_generate_random_walls(Scene *s)
{
    for (int i = 0; i < INTERIOR_WALL_COUNT; i++) {
        WallSpec *spec = &s->rand_wall_specs[i];

        /* Find an anchor outside the observer's safe zone */
        float ax = 0.5f, ay = 0.5f;
        for (int attempt = 0; attempt < MAX_PLACEMENT_TRIES; attempt++) {
            ax = 0.04f + rand_frac() * 0.92f;
            ay = 0.04f + rand_frac() * 0.92f;
            bool in_safe_x = fabsf(ax - OBS_CENTER_X_FRAC) < OBS_SAFE_HALF_X;
            bool in_safe_y = fabsf(ay - OBS_CENTER_Y_FRAC) < OBS_SAFE_HALF_Y;
            if (!(in_safe_x && in_safe_y)) break;  /* outside safe zone: accept */
        }

        /* Random angle in [0, PI) -- half-circle covers all undirected directions */
        float angle_rad = rand_frac() * (float)M_PI;

        /* Random half-length: scale by the shorter scene dimension so walls
         * look similarly sized in both wide and tall terminals */
        float scene_short_side = (s->scene_cols < s->scene_rows * 2)
                                 ? 1.0f : (float)s->scene_rows / (float)s->scene_cols;
        (void)scene_short_side;  /* proportional; use raw fraction directly */
        float half_len = RAND_WALL_LEN_MIN
                         + rand_frac() * (RAND_WALL_LEN_MAX - RAND_WALL_LEN_MIN);

        /* Compute endpoints and clamp to inner scene (avoid the bounding wall) */
        float cos_a = cosf(angle_rad) * half_len;
        float sin_a = sinf(angle_rad) * half_len;

        spec->x0_frac = fmaxf(0.02f, fminf(0.98f, ax - cos_a));
        spec->y0_frac = fmaxf(0.02f, fminf(0.98f, ay - sin_a));
        spec->x1_frac = fmaxf(0.02f, fminf(0.98f, ax + cos_a));
        spec->y1_frac = fmaxf(0.02f, fminf(0.98f, ay + sin_a));
    }

    s->use_rand_walls = true;
    scene_build_walls(s);
    compute_visibility(s->obs_x_cells, s->obs_y_cells,
                       s->walls, s->n_walls, &s->vis);
}

/*
 * scene_update_observer -- advance the Lissajous phase and recompute obs position.
 *
 * x = center_x + radius_x * sin(FREQ_X * t)
 * y = center_y + radius_y * sin(FREQ_Y * t + PHASE)
 *
 * The 3:2 frequency ratio traces a 6-lobed figure.  The small radii (12%,10%)
 * keep the observer inside the open central region away from walls.
 */
static void scene_update_observer(Scene *s)
{
    float cx = (float)s->scene_cols * OBS_CENTER_X_FRAC;
    float cy = (float)s->scene_rows * OBS_CENTER_Y_FRAC;
    float rx = (float)s->scene_cols * OBS_RADIUS_X_FRAC;
    float ry = (float)s->scene_rows * OBS_RADIUS_Y_FRAC;

    s->obs_x_cells = cx + rx * sinf(LISSAJOUS_FREQ_X * s->lissajous_t);
    s->obs_y_cells = cy + ry * sinf(LISSAJOUS_FREQ_Y * s->lissajous_t
                                    + LISSAJOUS_PHASE_RAD);

    s->lissajous_t += LISSAJOUS_SPEED;
    if (s->lissajous_t >= (float)(2.0 * M_PI))
        s->lissajous_t -= (float)(2.0 * M_PI);
}

static void scene_init(int term_cols, int term_rows)
{
    Scene *s = &g_scene;
    memset(s, 0, sizeof(*s));

    s->scene_cols    = term_cols;
    s->scene_rows    = term_rows - HUD_ROWS;
    s->sim_fps       = SIM_FPS_DEFAULT;
    s->lissajous_t   = 0.0f;
    s->sweep_angle_rad = -(float)M_PI;
    s->paused        = false;

    scene_build_walls(s);

    s->obs_x_cells = (float)s->scene_cols * OBS_CENTER_X_FRAC;
    s->obs_y_cells = (float)s->scene_rows * OBS_CENTER_Y_FRAC;

    compute_visibility(s->obs_x_cells, s->obs_y_cells,
                       s->walls, s->n_walls, &s->vis);
}

static void scene_tick(void)
{
    Scene *s = &g_scene;

    scene_update_observer(s);

    /* Advance sweep ray for the animation */
    s->sweep_angle_rad += SWEEP_SPEED_RAD;
    if (s->sweep_angle_rad > (float)M_PI)
        s->sweep_angle_rad -= (float)(2.0 * M_PI);

    compute_visibility(s->obs_x_cells, s->obs_y_cells,
                       s->walls, s->n_walls, &s->vis);
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
    if (dist_fraction < 0.10f) return '@';  /* very close  -- dense ring */
    if (dist_fraction < 0.25f) return 'o';  /* near        -- medium     */
    if (dist_fraction < 0.45f) return '+';  /* mid-range                 */
    if (dist_fraction < 0.65f) return '.';  /* far                       */
    return ' ';                              /* very far    -- fade out   */
}

/* color_pair_for_distance -- complement fill_char_for_distance with color. */
static int color_pair_for_distance(float dist_fraction)
{
    if (dist_fraction < 0.30f) return CP_FILL_NEAR;
    if (dist_fraction < 0.60f) return CP_FILL_MID;
    return CP_FILL_FAR;
}

/*
 * draw_line_bresenham -- draw character c from (x0,y0) to (x1,y1) using
 * Bresenham's algorithm, clipped to the scene area.
 */
static void draw_line_bresenham(
    int x0, int y0, int x1, int y1,
    int scene_cols, int scene_rows,
    int color_pair, int attrs, char c
) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    attron(COLOR_PAIR(color_pair) | attrs);
    while (1) {
        if (x0 >= 0 && x0 < scene_cols && y0 >= 0 && y0 < scene_rows)
            mvaddch(y0, x0, (chtype)c);
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
    float dx = w->x1_cells - w->x0_cells;
    float dy = w->y1_cells - w->y0_cells;
    float adx = fabsf(dx), ady = fabsf(dy);
    if (ady < adx * 0.4f) return '-';
    if (adx < ady * 0.4f) return '|';
    return (dx * dy > 0.0f) ? '\\' : '/';
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
        s->obs_x_cells, s->obs_y_cells,
        s->sweep_angle_rad,
        s->walls, s->n_walls,
        &ray_hit
    );
    if (!hit) return;

    int x0 = (int)roundf(s->obs_x_cells);
    int y0 = (int)roundf(s->obs_y_cells);
    int x1 = (int)roundf(ray_hit.hit_x_cells);
    int y1 = (int)roundf(ray_hit.hit_y_cells);

    /* Draw ray, but skip the observer cell itself (drawn later as '@') */
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    attron(COLOR_PAIR(CP_SWEEP_RAY));
    while (1) {
        bool is_observer_cell = (x0 == (int)roundf(s->obs_x_cells) &&
                                 y0 == (int)roundf(s->obs_y_cells));
        if (!is_observer_cell &&
            x0 >= 0 && x0 < s->scene_cols && y0 >= 0 && y0 < s->scene_rows)
            mvaddch(y0, x0, '*');
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
static void render_scene(void)
{
    const Scene *s = &g_scene;

    float obs_x   = s->obs_x_cells;
    float obs_y   = s->obs_y_cells;
    float obs_y_g = geo_y(obs_y);

    /* Scene diagonal in geo space -- used to normalise distance to [0,1] */
    float scene_diag_geo = sqrtf(
        (float)s->scene_cols * (float)s->scene_cols +
        geo_y((float)s->scene_rows) * geo_y((float)s->scene_rows)
    );

    /* --- Pass 1: fill visible cells --- */
    for (int row = 0; row < s->scene_rows; row++) {
        for (int col = 0; col < s->scene_cols; col++) {
            float cell_cx = (float)col + 0.5f;
            float cell_cy = (float)row + 0.5f;

            if (!cell_is_visible(cell_cx, cell_cy, obs_x, obs_y, &s->vis))
                continue;

            float dx_geo   = cell_cx - obs_x;
            float dy_geo   = geo_y(cell_cy) - obs_y_g;
            float dist_geo = sqrtf(dx_geo * dx_geo + dy_geo * dy_geo);
            float fraction = (scene_diag_geo > 0.0f)
                             ? dist_geo / scene_diag_geo : 0.0f;

            char fill = fill_char_for_distance(fraction);
            if (fill == ' ') continue;

            int cpair = color_pair_for_distance(fraction);
            attron(COLOR_PAIR(cpair));
            mvaddch(row, col, (chtype)fill);
            attroff(COLOR_PAIR(cpair));
        }
    }

    /* --- Pass 2: animated sweep ray --- */
    draw_sweep_ray(s);

    /* --- Pass 3: interior walls (bold white '#', '|', '-', '/') --- */
    for (int i = BOUNDING_WALL_COUNT; i < s->n_walls; i++) {
        const Wall *w = &s->walls[i];
        char wc = wall_display_char(w);
        draw_line_bresenham(
            (int)roundf(w->x0_cells), (int)roundf(w->y0_cells),
            (int)roundf(w->x1_cells), (int)roundf(w->y1_cells),
            s->scene_cols, s->scene_rows,
            CP_WALL, A_BOLD, wc
        );
    }

    /* --- Pass 4: bounding box (dim '+' at corners, '-'/'|' on edges) --- */
    for (int i = 0; i < BOUNDING_WALL_COUNT; i++) {
        const Wall *w = &s->walls[i];
        char wc = wall_display_char(w);
        draw_line_bresenham(
            (int)roundf(w->x0_cells), (int)roundf(w->y0_cells),
            (int)roundf(w->x1_cells), (int)roundf(w->y1_cells),
            s->scene_cols, s->scene_rows,
            CP_BOUND, A_DIM, wc
        );
    }

    /* --- Pass 5: observer on top --- */
    int obs_col = (int)roundf(obs_x);
    int obs_row = (int)roundf(obs_y);
    if (obs_col >= 0 && obs_col < s->scene_cols &&
        obs_row >= 0 && obs_row < s->scene_rows)
    {
        attron(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
        mvaddch(obs_row, obs_col, '@');
        attroff(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
    }
}

/*
 * render_overlay -- draw the HUD panel below the scene.
 *
 * Rows:
 *   +0  Title + key bindings
 *   +1  Separator line
 *   +2  Visible area (cell^2 + % of scene) and ray count
 *   +3  Observer position and current sweep angle
 *   +4  Algorithm summary line 1
 *   +5  Algorithm summary line 2
 *   +6  Algorithm summary line 3
 *   +7  Fill legend
 */
static void render_overlay(void)
{
    const Scene *s     = &g_scene;
    int hud_row        = s->scene_rows;   /* first terminal row of the HUD */

    float scene_area_cells = (float)s->scene_cols * (float)s->scene_rows;
    float visible_pct      = (scene_area_cells > 0.0f)
                             ? 100.0f * s->vis.visible_area_cells / scene_area_cells
                             : 0.0f;

    /* Row +0: title */
    attron(COLOR_PAIR(CP_HUD_HEADER) | A_BOLD);
    mvprintw(hud_row + 0, 0,
             " Visibility Polygon  "
             "[SPACE pause | +/- speed | r reset | n new walls | q quit]");
    attroff(COLOR_PAIR(CP_HUD_HEADER) | A_BOLD);

    /* Row +1: separator */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud_row + 1, 0, " %.*s",
             s->scene_cols - 2,
             "--------------------------------------------------------------"
             "--------------------------------------------------------------");
    attroff(COLOR_PAIR(CP_HUD));

    /* Row +2: visibility metrics */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud_row + 2, 0, " Visible area:");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_HUD_VALUE) | A_BOLD);
    mvprintw(hud_row + 2, 15, "%6.0f cells^2 (%5.1f%%)",
             s->vis.visible_area_cells, visible_pct);
    attroff(COLOR_PAIR(CP_HUD_VALUE) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud_row + 2, 43, "  Rays cast:");
    attroff(COLOR_PAIR(CP_HUD));
    attron(COLOR_PAIR(CP_HUD_VALUE) | A_BOLD);
    mvprintw(hud_row + 2, 56, "%3d  FPS: %2d%s",
             s->vis.n_hits, s->sim_fps,
             s->paused ? "  [PAUSED]" : "          ");
    attroff(COLOR_PAIR(CP_HUD_VALUE) | A_BOLD);

    /* Row +3: observer state */
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud_row + 3, 0,
             " Observer: col=%.1f row=%.1f   "
             "sweep angle: %+.3f rad   walls: %d",
             s->obs_x_cells, s->obs_y_cells,
             s->sweep_angle_rad, s->n_walls);
    attroff(COLOR_PAIR(CP_HUD));

    /* Rows +4 to +7: algorithm explanation */
    attron(COLOR_PAIR(CP_HUD_EXPLAIN));
    mvprintw(hud_row + 4, 0,
        " Algo: sweep ray at each wall endpoint angle +/-eps -> nearest wall"
        " hit -> sort -> polygon.");
    mvprintw(hud_row + 5, 0,
        " Between consecutive endpoint angles the nearest occluder cannot"
        " change: boundary is linear.");
    mvprintw(hud_row + 6, 0,
        " Cell visibility: binary-search sector, intersect ray with boundary"
        " segment, compare dist.");
    mvprintw(hud_row + 7, 0,
        " Fill: '@' near  'o' close  '+' mid  '.' far | '@'=observer  '*'="
        "sweep ray  '#'=wall");
    attroff(COLOR_PAIR(CP_HUD_EXPLAIN));
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

static void app_handle_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR) {
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_quit = 1;
            break;
        case ' ':
            s->paused = !s->paused;
            break;
        case 'r': case 'R':
            scene_init(s->scene_cols, s->scene_rows + HUD_ROWS);
            break;
        case 'n': case 'N':
            scene_generate_random_walls(s);
            break;
        case '+': case '=':
            if (s->sim_fps < SIM_FPS_MAX) s->sim_fps += 5;
            break;
        case '-': case '_':
            if (s->sim_fps > SIM_FPS_MIN) s->sim_fps -= 5;
            break;
        default:
            break;
        }
    }
}

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGINT,   handle_sigint);

    srand((unsigned)time(NULL));

    screen_init();

    int term_cols, term_rows;
    getmaxyx(stdscr, term_rows, term_cols);
    scene_init(term_cols, term_rows);

    int64_t sim_accum_ns  = 0;
    int64_t frame_prev_ns = clock_ns();

    while (!g_quit) {
        /* Handle terminal resize */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, term_rows, term_cols);
            scene_init(term_cols, term_rows);
            frame_prev_ns = clock_ns();
            sim_accum_ns  = 0;
        }

        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - frame_prev_ns;
        frame_prev_ns  = now_ns;

        /* Fixed-timestep accumulator: advance physics in discrete ticks */
        if (!g_scene.paused) {
            sim_accum_ns += dt_ns;
            int64_t tick_ns = TICK_NS(g_scene.sim_fps);
            while (sim_accum_ns >= tick_ns) {
                scene_tick();
                sim_accum_ns -= tick_ns;
            }
        }

        app_handle_input(&g_scene);

        erase();
        render_scene();
        render_overlay();
        wnoutrefresh(stdscr);
        doupdate();

        /* Sleep for the remainder of the frame budget */
        int64_t budget_ns  = TICK_NS(TARGET_FPS);
        int64_t elapsed_ns = clock_ns() - now_ns;
        clock_sleep_ns(budget_ns - elapsed_ns);
    }

    screen_teardown();
    return 0;
}
