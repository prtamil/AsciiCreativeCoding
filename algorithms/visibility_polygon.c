/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * visibility_polygon.c -- what a light at a moving point can see inside a
 * room of walls.  Shoot rays at every wall corner, find where each ray first
 * hits a wall, connect the hit points into a polygon, then fill it.  The
 * observer drifts along a looping Lissajous path so the lit region animates.
 *
 * Algorithm: exact angular-sweep visibility polygon (Asano 1986).  Clearest
 * walkthrough: Red Blob Games, "2D Visibility"
 * (https://www.redblobgames.com/articles/visibility/).
 *
 * Keys: SPACE pause  r reset  +/- speed  q quit
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/visibility_polygon.c \
 *            -o vp -lncurses -lm
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

/* ── §1 config — sizes, timing, aspect ratio, sweep tuning ── */

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

/* ── §2 clock — monotonic nanosecond clock and sleep ── */

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

/* ── §3 color — color pairs for fill, walls, observer, HUD ── */

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

/* ── §4 geometry — wall/hit/polygon types and the ray-vs-wall test ── */

/*
 * Wall -- one wall, stored as a line segment from corner to corner.  Every
 * frame each ray is tested against every wall to see where it stops.  The
 * four coordinates are kept flat (not two points) so the intersection math
 * can read them straight as A.x, A.y, B.x, B.y.
 *
 *   x0_cells, y0_cells   one endpoint, in cells (column, row).
 *   x1_cells, y1_cells   the other endpoint, in cells.
 *
 * The segment has no direction -- swapping the two endpoints blocks sight
 * exactly the same.  "_cells" matters: y must be passed through geo_y()
 * before any distance/angle math because terminal cells are twice as tall
 * as they are wide.
 */
typedef struct {
    float x0_cells, y0_cells;   /* one endpoint (col, row)   */
    float x1_cells, y1_cells;   /* other endpoint (col, row) */
} Wall;

/*
 * WallSpec -- the same wall, but stored as fractions of the screen rather
 * than fixed cells, so the layout survives a terminal resize.  When the
 * window changes size we re-multiply the fractions by the new (cols, rows)
 * and the scene keeps the same proportions, just bigger or smaller.
 *
 *   x0_frac, y0_frac   one endpoint as a fraction of (cols, rows).
 *   x1_frac, y1_frac   other endpoint as a fraction of (cols, rows).
 *
 * 0.0 is the top/left edge, 1.0 the bottom/right edge.  Hand-placed layouts
 * stay inside [0, 1]; the random generator may use the full unit square.
 */
typedef struct {
    float x0_frac, y0_frac;  /* one endpoint, fraction of (cols, rows)   */
    float x1_frac, y1_frac;  /* other endpoint, fraction of (cols, rows) */
} WallSpec;

/*
 * SweepHit -- one ray's result: the direction it was fired, and the point
 * where it first hit a wall.  Sort a batch of these by angle and join them
 * up and you have traced the outline of everything the observer can see.
 * Angle and hit point ride together in one struct so a single sort by angle
 * keeps each direction matched to its hit point.
 *
 *   angle_rad     direction from the observer (radians, geo space).
 *                 This is the sort key; the final polygon is in angle order.
 *   hit_x_cells   x of the first wall hit, in cells.
 *   hit_y_cells   y of the first wall hit, in cells.
 *
 * angle_rad is wrapped into (-pi, pi] before sorting so the outline walks
 * around the observer exactly once.  The hit point always sits on some wall.
 */
typedef struct {
    float angle_rad;      /* ray direction from observer (radians, geo space) */
    float hit_x_cells;    /* x of first wall hit, cells                       */
    float hit_y_cells;    /* y of first wall hit, cells                       */
} SweepHit;

/*
 * VisibilityPolygon -- the whole result of one visibility computation: the
 * outline of what the observer can see, plus its area.  Rebuilt from scratch
 * every frame.  Outline, count, and area travel together so a reader can
 * never use a fresh outline with a stale count.
 *
 *   hits[MAX_SWEEP_RAYS]  outline corners, sorted by angle; join them in
 *                         order to trace the lit region.
 *   n_hits                how many entries in hits[] are real (rays that
 *                         missed every wall are dropped).  0..MAX_SWEEP_RAYS.
 *   visible_area_cells    area of that outline in cells^2 (shoelace formula);
 *                         cached here so the HUD's "% visible" is free to
 *                         read.  Always >= 0.
 */
typedef struct {
    SweepHit hits[MAX_SWEEP_RAYS]; /* outline corners, sorted by angle      */
    int       n_hits;              /* valid entries in hits[]               */
    float     visible_area_cells;  /* outline area, cells^2 (shoelace)      */
} VisibilityPolygon;

/* Stretch y so a cell is square for the math (geo space), and unstretch it
 * back for drawing.  Do every angle/distance calculation in geo space; only
 * go back to cell space for the final mvaddch(). */
static inline float geo_y(float y_cells)  { return y_cells * ASPECT_Y; }
static inline float cell_y(float y_geo)   { return y_geo   / ASPECT_Y; }

/*
 * ray_hits_wall -- where the sight line first crosses a wall, if at all.
 *
 * Walk the ray forward as O + t*D and the wall as A + s*(B-A), then solve
 * for the (t, s) where they meet.  A real hit needs t in front of the eye
 * (t >= RAY_T_MIN) and s between the wall's two ends (0 <= s <= 1).  The
 * shared "denom" is the 2-D cross product of the two directions: zero means
 * ray and wall are parallel and never meet.
 *
 * Returns t (distance along the ray) on a hit, or -1 on a miss.  All inputs
 * must already be in geo space.
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

/* ── §5 visibility — cast rays at every corner, sort, build the polygon ── */

/* Test one ray against one wall, stretching the wall's endpoints into geo
 * space first so the caller can stay in cell space. */
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

/* Walk every wall and keep the closest one this ray hits -- that's the wall
 * the sight line actually stops at.  Returns a huge sentinel value if the ray
 * somehow hits nothing; the caller checks for that. */
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

/* Fire one ray from the observer at the given angle and return its nearest
 * wall hit in out_hit; false if it somehow hits nothing. */
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

/* Angle from the observer to a wall corner.  Both points are stretched into
 * geo space first so the angle is correct on the 2:1 terminal grid. */
static float angle_from_observer_to_endpoint(
    float ox_geo, float oy_geo, float ex_cells, float ey_cells
) {
    float dx = ex_cells           - ox_geo;
    float dy = geo_y(ey_cells)    - oy_geo;
    return atan2f(dy, dx);
}

/* Push three angles for one corner: dead-on, plus a hair to each side.  A ray
 * aimed exactly at a corner is ambiguous about which side it passes; the two
 * nudged rays slip just past the corner so the sight line continues to the
 * wall behind it.  Skips silently if the buffer is full. */
static void push_endpoint_angle_triplet(
    float angles[], int *count, int capacity, float theta
) {
    if (*count + 3 > capacity) return;
    angles[(*count)++] = theta - ENDPOINT_ANGLE_EPS_RAD;
    angles[(*count)++] = theta;
    angles[(*count)++] = theta + ENDPOINT_ANGLE_EPS_RAD;
}

/* Step 1 of the sweep: gather the directions worth firing at.  Only wall
 * corners can change what's visible, so collect the angle to each corner
 * (with its two nudged neighbours).  Output is unsorted; it gets ordered
 * later. */
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

/* Step 2 of the sweep: fire one ray per collected angle, keeping each
 * nearest hit.  Rays that hit nothing are dropped, but the room's outer
 * walls always stop every ray, so that case doesn't happen here. */
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

/* Step 3 of the sweep: sort the hits by angle so neighbours in the list are
 * neighbours around the observer -- that's what makes them a polygon. */
static void sort_hits_by_angle(VisibilityPolygon *vp)
{
    qsort(vp->hits, vp->n_hits, sizeof(SweepHit), cmp_sweep_hit_by_angle);
}

/* Area of the lit region in cells, via the shoelace formula: walk the
 * sorted corners and sum each edge's cross product, then halve the absolute
 * total.  The final /ASPECT_Y undoes the y-stretch so the answer is in real
 * cells, not stretched ones. */
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

/* The heart of the file: build the whole "what can the observer see" polygon.
 * Aim rays at every wall corner, keep where each first hits a wall, sort the
 * hits into a loop around the observer, then measure the enclosed area. */
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

/* Binary-search the sorted hits for the last one at or before query_angle_rad
 * -- i.e. which slice of the polygon a given direction falls into.  The caller
 * pairs it with the next hit to get the slice's far edge.  Needs hits[] sorted
 * and non-empty. */
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

/* Vector and squared distance from observer to a cell, in geo space.  We
 * return the squared distance (no sqrt) because the "cell sits on the
 * observer" early-out only needs to compare against a small radius. */
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

/* For a given direction, return the two polygon corners whose edge that
 * direction crosses.  The slice that wraps past the last corner back to the
 * first is handled on its own, since the binary search only covers the
 * middle. */
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

/* Fire a ray toward a cell and find how far it reaches before crossing the
 * polygon edge for that slice -- i.e. how far the observer can see in that
 * direction.  Returns distance along the ray, or negative if that edge is
 * degenerate (its two corners coincide). */
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

/* Used when the slice's edge collapsed to a point: just compare the cell's
 * distance against that point's distance (with one cell of slack). */
static bool visible_via_degenerate_sector_fallback(
    const VisibilityPolygon *vp, int sector_idx,
    float obs_x_cells, float obs_y_cells, float cell_dist
) {
    float bnd0_x = vp->hits[sector_idx].hit_x_cells - obs_x_cells;
    float bnd0_y = geo_y(vp->hits[sector_idx].hit_y_cells) - geo_y(obs_y_cells);
    float bnd0_dist = sqrtf(bnd0_x * bnd0_x + bnd0_y * bnd0_y);
    return cell_dist <= bnd0_dist + DEGENERATE_SECTOR_SLACK_CELLS;
}

/* Can the observer see this cell?  Find the direction to the cell, look up
 * how far the lit region reaches that way, and check the cell isn't farther
 * than that.  Tiny edge cases: a too-small polygon is never visible, a cell
 * right on the observer always is, and a small tolerance lets cells sitting
 * exactly on the boundary still light up. */
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

/* ── §6 scene — walls, observer, controls, and per-tick update ── */

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
 * Interior obstacle walls: kept out toward the edges so the observer's drift
 * path through the middle never runs into one.
 *
 * Layout sketch (. = open space, # = wall, @ = where the observer roams):
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
 * WallSet -- every wall in the room, plus the state for the "new random
 * layout" feature ('n' key).  Fixed-size array, no allocation: the project
 * never mallocs after startup, and MAX_WALLS is the exact capacity used.
 *
 *   list[MAX_WALLS]   the actual walls in cells.  The first
 *                     BOUNDING_WALL_COUNT are the outer box (always there);
 *                     the rest are interior obstacles.
 *   count             how many entries of list[] are in use
 *                     (BOUNDING_WALL_COUNT..MAX_WALLS).
 *   rand_specs[]      a randomly generated interior layout, kept as
 *                     fractions so it can be re-baked on a resize.
 *   use_rand          false = use the hand-placed default interior layout;
 *                     true once 'n' is pressed = use rand_specs instead.
 */
typedef struct {
    Wall      list[MAX_WALLS];               /* walls in use, cell coords  */
    int       count;                         /* valid entries in list[]    */
    WallSpec  rand_specs[INTERIOR_WALL_COUNT]; /* random layout, fractions */
    bool      use_rand;                      /* default vs random layout   */
} WallSet;

/*
 * Observer -- the eye: where it is, where it's headed, and the lone spinning
 * ray drawn as a teaching aid.  The drift path and the spinning ray move at
 * their own rates, so pausing one doesn't pause the other.
 *
 *   x_cells, y_cells   eye position in cells (column, row).  The visibility
 *                      polygon is computed from here.
 *   lissajous_t        position along the looping drift path, in radians.
 *                      Feeding it through sin gives the eye's x and y, so a
 *                      3:2 frequency ratio traces a smooth figure-eight.
 *   sweep_angle_rad    direction of the animated '*' ray (radians).  Purely
 *                      decorative -- it just shows "one ray being cast" and
 *                      rotates on its own.
 *
 * The eye stays inside the room (the path radii leave a margin).  Both angles
 * are kept in (-pi, pi].  lissajous_t is wrapped each tick so it never drifts
 * off into large, imprecise floats.
 */
typedef struct {
    float x_cells, y_cells;   /* eye position, cells (col, row)        */
    float lissajous_t;        /* phase along the drift path, radians   */
    float sweep_angle_rad;    /* direction of the '*' teaching ray     */
} Observer;

/*
 * SimControls -- the two playback knobs the user can change while it runs.
 *
 *   paused   true freezes the animation; the HUD then reads "PAUSED".
 *   fps      how many simulation steps per second; the +/- keys nudge it,
 *            kept within [SIM_FPS_MIN, SIM_FPS_MAX].
 */
typedef struct {
    bool paused;   /* animation frozen?      */
    int  fps;      /* simulation steps/sec   */
} SimControls;

/*
 * Scene -- all the state for one run, in one place.  A single Scene lives on
 * main()'s stack and is threaded through every function: functions that only
 * read it take `const Scene *`, functions that change it take `Scene *`, so
 * the signature tells you whether a call mutates anything.  The only state
 * not in here is the two signal flags in §9, which have to be globals because
 * signal handlers can't be handed a pointer.
 *
 *   walls        the room's walls plus random-layout state.
 *   observer     the eye, its drift path, and the spinning ray.
 *   vis          the visibility polygon, recomputed every frame.
 *   scene_cols   drawable width  (= terminal columns).
 *   scene_rows   drawable height (= terminal rows minus the HUD rows).
 *   sim          paused flag and simulation speed.
 */
typedef struct {
    WallSet           walls;       /* the room's walls               */
    Observer          observer;    /* the eye                        */
    VisibilityPolygon vis;         /* what the eye can see this frame */
    int               scene_cols;  /* drawable width, cells          */
    int               scene_rows;  /* drawable height, cells         */
    SimControls       sim;         /* pause + speed                  */
} Scene;

/* Turn one fraction-based WallSpec into a concrete cell-space Wall by
 * multiplying its fractions by the scene's width and height. */
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

/* Rebuild the live wall list: the outer box first, then whichever interior
 * layout is selected.  Call after the scene size is known (init or resize). */
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

/* Pick a random anchor that avoids the observer's no-walls zone: keep
 * re-rolling until one lands outside it, giving up after a fixed number of
 * tries and using whatever it last drew. */
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

/* Build a wall centered on an anchor: step half its length each way along
 * the given angle to get the two endpoints, then pull them back inside the
 * room if they spilled past the edge. */
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

/* The 'n' key: roll a fresh random interior layout (each wall a random
 * anchor, angle, and length), switch to it, rebuild the walls, and recompute
 * what the observer can now see. */
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

/* Where on the drift path the eye sits at phase t.  Each axis is a sine wave
 * around the scene centre; the 3:2 frequency ratio makes the looping
 * figure-eight.  Centre and amplitude scale with the scene so it fits any
 * terminal size. */
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

/* Move the eye one step along its drift path and advance the path phase. */
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

/* Set up a fresh scene for the given terminal size: clear it, seed the
 * controls and observer, build the walls, and compute the first frame's
 * visibility so something is ready to draw immediately. */
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

/* One simulation step: nudge the eye and the spinning ray forward, then
 * recompute what the eye can see from its new spot. */
static void scene_tick(Scene *s)
{
    scene_update_observer(s);
    s->observer.sweep_angle_rad =
        sweep_ray_advance_angle(s->observer.sweep_angle_rad);
    compute_visibility(s->observer.x_cells, s->observer.y_cells,
                       s->walls.list, s->walls.count, &s->vis);
}

/* ── §7 render — paint the lit region, walls, ray, observer, and HUD ── */

/* Pick a fill glyph by distance: dense near the eye, fading to dots and then
 * blank far away, so the lit region looks like a glow around the observer.
 * dist_fraction is 0 at the eye, 1 at the far corner. */
static char fill_char_for_distance(float dist_fraction)
{
    if (dist_fraction < FILL_DIST_FRAC_DENSE_LIMIT) return '@';  /* dense ring */
    if (dist_fraction < FILL_DIST_FRAC_NEAR_LIMIT)  return 'o';
    if (dist_fraction < FILL_DIST_FRAC_MID_LIMIT)   return '+';
    if (dist_fraction < FILL_DIST_FRAC_FAR_LIMIT)   return '.';
    return ' ';                                                  /* fade out  */
}

/* The matching colour for a distance: brighter near the eye, cooler far. */
static int color_pair_for_distance(float dist_fraction)
{
    if (dist_fraction < COLOR_DIST_FRAC_NEAR_LIMIT) return CP_FILL_NEAR;
    if (dist_fraction < COLOR_DIST_FRAC_MID_LIMIT)  return CP_FILL_MID;
    return CP_FILL_FAR;
}

/* Draw one character at a scene cell, clipping off-screen writes.  Scene row
 * 0 sits below the top HUD row, so we shift down by HUD_TOP_ROWS.  Every
 * scene-space draw goes through here. */
static void scene_addch(const Scene *s, int scene_y, int scene_x, char ch)
{
    if (scene_x < 0 || scene_x >= s->scene_cols) return;
    if (scene_y < 0 || scene_y >= s->scene_rows) return;
    mvaddch(scene_y + HUD_TOP_ROWS, scene_x, (chtype)ch);
}

/* Draw a straight line of character c from one cell to another using
 * Bresenham's algorithm (integer stepping, no floats).  scene_addch handles
 * the HUD offset and clipping. */
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

/* Pick a glyph that matches the wall's slant: '-' for flat, '|' for upright,
 * '/' or '\' for diagonal. */
static char wall_display_char(const Wall *w)
{
    float dx  = w->x1_cells - w->x0_cells;
    float dy  = w->y1_cells - w->y0_cells;
    float adx = fabsf(dx), ady = fabsf(dy);
    if (ady < adx * WALL_AXIS_RATIO_THRESHOLD) return '-';   /* horizontal */
    if (adx < ady * WALL_AXIS_RATIO_THRESHOLD) return '|';   /* vertical   */
    return (dx * dy > 0.0f) ? '\\' : '/';                    /* diagonal   */
}

/* Draw the spinning '*' ray from the eye to the first wall it hits -- just a
 * teaching aid to show the sweep going around, separate from the real
 * visibility math. */
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

/* The farthest a cell can be from the eye -- the corner-to-corner distance in
 * geo space.  Used to scale raw distances into the 0..1 fraction the glyph
 * and colour pickers want. */
static float scene_diagonal_geo(const Scene *s)
{
    float w = (float)s->scene_cols;
    float h = geo_y((float)s->scene_rows);
    return sqrtf(w * w + h * h);
}

/* The middle of a cell -- we test visibility at the centre of the glyph, not
 * its corner, so the lit/unlit decision matches what the eye sees painted. */
static void cell_center_in_cell_space(int row, int col,
                                      float *out_cx, float *out_cy)
{
    *out_cx = (float)col + CELL_CENTER_OFFSET;
    *out_cy = (float)row + CELL_CENTER_OFFSET;
}

/* How far a cell is from the eye, as a fraction of the longest possible
 * distance (0 at the eye, 1 at the far corner). */
static float cell_distance_fraction(
    float cell_cx, float cell_cy,
    float obs_x_cells, float obs_y_cells, float scene_diag_geo
) {
    float dx = cell_cx - obs_x_cells;
    float dy = geo_y(cell_cy) - geo_y(obs_y_cells);
    float dist = sqrtf(dx * dx + dy * dy);
    return (scene_diag_geo > 0.0f) ? dist / scene_diag_geo : 0.0f;
}

/* Paint one cell with the glyph and colour for its distance; does nothing if
 * that distance maps to blank space. */
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

/* Pass 1: walk every cell, and for each one the eye can see, paint it with
 * the glyph and colour for its distance. */
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

/* Draw one wall as a line in the given colour and style. */
static void paint_wall_line(
    const Scene *s, const Wall *w, int color_pair, int attr
) {
    char wc = wall_display_char(w);
    draw_line_bresenham(s,
        (int)roundf(w->x0_cells), (int)roundf(w->y0_cells),
        (int)roundf(w->x1_cells), (int)roundf(w->y1_cells),
        color_pair, attr, wc);
}

/* Pass 3: the interior obstacle walls, in bold. */
static void paint_interior_walls(const Scene *s)
{
    for (int i = BOUNDING_WALL_COUNT; i < s->walls.count; i++)
        paint_wall_line(s, &s->walls.list[i], CP_WALL, A_BOLD);
}

/* Pass 4: the outer box, dim.  Drawn after the interior walls so it never
 * paints over where they meet. */
static void paint_bounding_box(const Scene *s)
{
    for (int i = 0; i < BOUNDING_WALL_COUNT; i++)
        paint_wall_line(s, &s->walls.list[i], CP_BOUND, A_DIM);
}

/* Pass 5: the observer '@', on top of everything. */
static void paint_observer_glyph(const Scene *s)
{
    int obs_col = (int)roundf(s->observer.x_cells);
    int obs_row = (int)roundf(s->observer.y_cells);
    attron(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
    scene_addch(s, obs_row, obs_col, '@');
    attroff(COLOR_PAIR(CP_OBSERVER) | A_BOLD);
}

/* Draw the whole frame back-to-front: lit fill, then the sweep ray, then
 * walls, the outer box, and finally the observer on top. */
static void render_scene(const Scene *s)
{
    paint_visible_cells_with_distance_falloff(s);
    draw_sweep_ray(s);
    paint_interior_walls(s);
    paint_bounding_box(s);
    paint_observer_glyph(s);
}

/* What fraction of the room the eye can see, as a percentage; 0 if the scene
 * has no area. */
static float visible_area_percent(const Scene *s)
{
    float scene_area = (float)s->scene_cols * (float)s->scene_rows;
    return (scene_area > 0.0f)
        ? 100.0f * s->vis.visible_area_cells / scene_area
        : 0.0f;
}

/* "PAUSED" or "running", for the status line. */
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

/* The HUD: a status line across the top and a key-hint strip along the
 * bottom, both bright and bold so they stay readable over the animation. */
static void draw_hud(const Scene *s)
{
    int term_rows, term_cols;
    getmaxyx(stdscr, term_rows, term_cols);

    char status[128];
    format_hud_status(s, status, sizeof status);
    draw_top_status(status, term_cols);
    draw_bottom_hint(term_rows);
}

/* ── §8 screen — ncurses bring-up and teardown ── */

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

/* ── §9 app — signals, input, and the main loop ── */

/* The only globals: signal handlers can't be handed a Scene pointer, so the
 * two flags they set have to live at file scope. */
static volatile sig_atomic_t g_resize_pending = 0;   /* terminal was resized */
static volatile sig_atomic_t g_quit           = 0;   /* time to exit         */

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

/* Act on one keystroke: q/ESC quit, space pauses, r resets, n re-rolls the
 * walls, +/- change speed. */
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

/* Handle every key waiting in the buffer.  getch() returns ERR when empty
 * because the terminal is in non-blocking mode. */
static void app_handle_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR) route_key_to_action(s, ch);
}

/* ── one-shot helpers for main() ── */

/* Install the POSIX signal handlers used by the loop. */
static void signals_install(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGINT,   handle_sigint);
}

/* If the terminal was resized, restart ncurses so it learns the new size,
 * then rebuild the scene to fit and reset the frame timers. */
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

/* Run as many fixed-size simulation steps as the elapsed time has earned.
 * Banking leftover time in an accumulator keeps the animation running at the
 * same speed whatever the actual frame rate. */
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

/* Draw one frame: clear, paint the scene and HUD, then flush in one write. */
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

/* Set up signals, the terminal, and the scene, then loop: handle any resize,
 * advance the simulation by the time elapsed, read input, draw, and wait for
 * the next frame -- until the user quits. */
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
