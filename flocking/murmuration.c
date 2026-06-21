/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * murmuration.c — a Reynolds boid flock drawn as a density cloud, with a
 * diving hawk.  Instead of one glyph per bird, birds are binned into a
 * grid and each cell shows a glyph from sparse '.' to dense '@', so the
 * flock reads like a real starling murmuration.
 * Boids: Reynolds 1987.  Spatial hash: Teschner et al. 2003.
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — counts, speeds, radii, weights, glyph ramp, hash dims ── */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,
    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /* How many birds. Default 800 reads as a clear dense core fading to a
     * sparse edge on a normal terminal. Max 1500 is the hard ceiling the
     * spatial hash keeps fast. +/- keys add or remove BOID_STEP at a time. */
    N_BOIDS_DEFAULT = 800,
    N_BOIDS_MAX     = 1500,
    N_BOIDS_MIN     = 100,
    BOID_STEP       = 100,

    /* Colour pair IDs. Pairs 1..N_COLORS tint the flock through the active
     * theme; PAIR_HAWK is a fixed red so the predator always stands out. */
    N_COLORS        =   7,
    PAIR_HAWK       =   8,
    PAIR_HUD        =   9,
    PAIR_HINT       =  10,

    N_THEMES        =  10,

    /* Density grid cap — big enough that even a huge terminal can't
     * overflow the buffer. Only the visible region is ever touched. */
    MAX_ROWS        =  80,
    MAX_COLS        = 256,

    /* Spatial-hash grid cap. Each hash cell is HASH_CELL_PX wide, set equal
     * to the largest neighbour radius so a bird's neighbours always sit in
     * the 3x3 block of cells around it. The grid grows with the world; these
     * caps cover any terminal we'd realistically see. */
    HASH_CELL_PX     = 80,         /* matches COHESION_RADIUS              */
    HASH_GRID_W_MAX  = 32,
    HASH_GRID_H_MAX  = 24,
};

/* Cell dimensions — physics in pixel space, draw in cells. */
#define CELL_W   8
#define CELL_H  16

/* ── boid speeds (px/s) ─────────────────────────────────────────────── */
#define BOID_SPEED       90.0f   /* cruise speed used by force formulas    */
#define BOID_MAX_SPEED  150.0f   /* hard cap on |vel| each tick            */
#define BOID_MIN_SPEED   30.0f   /* floor — boids never stall completely   */
#define FLEE_SPEED      180.0f   /* effective speed scaling on hawk flee   */

/* ── boid radii (px) ────────────────────────────────────────────────── */
#define SEP_RADIUS       14.0f   /* personal-space bubble                  */
#define ALIGN_RADIUS     50.0f   /* heading-match neighbourhood            */
#define COHESION_RADIUS  80.0f   /* centre-of-mass neighbourhood           */
#define HAWK_FLEE_RADIUS 110.0f  /* boids flee within this disc of hawk    */

/* ── boid force weights ─────────────────────────────────────────────── *
 *  Bigger weight = stronger pull. Fleeing the hawk wins over everything;
 *  separation is firm so birds don't overlap in a panic; cohesion is soft
 *  so the flock stays loose rather than collapsing to a dot. MAX_STEER caps
 *  how hard a bird can turn per tick, giving smooth curves not instant pivots. */
#define W_SEP      1.8f
#define W_ALIGN    1.0f
#define W_COHERE   0.7f
#define W_FLEE     4.5f
#define MAX_STEER 130.0f

/* ── hawk parameters ────────────────────────────────────────────────── *
 *  The hawk has three base modes (cycled with n/p) plus a DIVE overlay
 *  (fired by SPACE). PATROL circles the centre; HOVER drifts onto the flock
 *  and parks there; PURSUE chases at a touch above bird speed; DIVE is a
 *  fast straight sprint at the flock for a fixed time, then back to base.
 *  AUTO_DIVE_PERIOD is the gap between auto-dives when the 'h' toggle is on. */
#define HAWK_PATROL_FRAC      0.40f
#define HAWK_PATROL_OMEGA     0.4f
#define HAWK_DIVE_SPEED     250.0f
#define HAWK_DIVE_DURATION    1.5f
#define AUTO_DIVE_PERIOD      6.0f

/* HOVER drift speed, and the radius inside which it eases to a stop so the
 * hawk settles smoothly onto the flock instead of jerking to a halt. */
#define HAWK_HOVER_DRIFT        60.0f
#define HAWK_HOVER_HOLD_RADIUS  30.0f

/* PURSUE chase speed — a little faster than a bird, so the hawk slowly gains. */
#define HAWK_PURSUE_SPEED  110.0f

/* The glyph ramp: sparse cells get '.', the densest core gets '@'.
 * ASCII only (no Unicode blocks) so it renders the same everywhere. */
static const char DENSITY_RAMP[] = ".,:;oO*#@";
#define RAMP_LEN ((int)(sizeof DENSITY_RAMP - 1))

/* Timing helpers. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer + sleep ── */

/* Wall-clock in nanoseconds; never runs backward. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for the given nanoseconds; a non-positive value returns at once. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — flock themes plus fixed hawk and HUD colours ── */

/*
 * Theme — one named flock palette.
 *
 * The renderer spreads these colours across the cloud (it picks a pair per
 * cell from its position), so the flock gets a soft mottled tint rather than
 * one flat colour. That's why all seven entries are kept in the same
 * brightness band: a dim-to-bright spread would leave half the cloud looking
 * muddy. Everything stays bright (>= 80 on the xterm-256 scale) so nothing
 * vanishes if a dim tier is ever switched back on.
 *
 * Members
 *   name     label shown in the HUD (e.g. "Dusk"); points at a string literal.
 *   body[7]  seven xterm-256 colour indices (each 0..255), all shades of the
 *            same tint. Colour pair i+1 uses body[i].
 */
typedef struct {
    const char *name;
    int         body[N_COLORS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        bright cluster — all 7 entries are similar shades */
    {"Dusk",    { 180, 187, 188, 222, 223, 224, 230 }},  /* warm cream    */
    {"Sky",     {  81,  87, 111, 117, 123, 153, 159 }},  /* sky-blue      */
    {"Solar",   { 214, 220, 221, 222, 226, 227, 228 }},  /* warm yellow   */
    {"Aurora",  { 121, 122, 158, 159, 195, 207, 213 }},  /* mint to pink  */
    {"Ember",   { 196, 202, 208, 209, 214, 215, 220 }},  /* fire ramp     */
    {"Forest",  { 119, 120, 121, 154, 155, 156, 157 }},  /* leafy green   */
    {"Neon",    { 165, 171, 201, 207, 213, 219, 225 }},  /* hot magenta   */
    {"Sunset",  { 209, 210, 215, 216, 221, 222, 223 }},  /* warm orange   */
    {"Ghost",   { 247, 250, 252, 253, 254, 255, 231 }},  /* near-white    */
    {"Matrix",  {  46,  82, 118, 119, 120, 121, 156 }},  /* matrix green  */
};

/* Register the chosen flock palette with ncurses (background = terminal
 * default). Hawk and HUD colours don't change with the theme. */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_COLORS] = {
            COLOR_WHITE, COLOR_WHITE, COLOR_CYAN,
            COLOR_CYAN,  COLOR_BLUE,  COLOR_BLUE,  COLOR_MAGENTA
        };
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/* One-time colour setup: turn on colour, allow a default background, load the
 * starting flock palette, and fix the hawk (red) and HUD (yellow/cyan) pairs. */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HAWK, 196, -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HAWK, COLOR_RED,    -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ── §4 coords — pixel/cell bridge, Vec2, toroidal distance ── */

/* World size in pixels, from the terminal's cell counts. */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * World — the size of the simulation box, in pixels.
 *
 * The physics treats the world as a torus: a bird leaving one edge comes back
 * on the opposite edge, and "distance" between two birds is always the
 * shortest path, even if that path goes off one edge and back on the other.
 * Both rules need the world's width and height, so they're bundled here and
 * passed around as one value.
 *
 * Members
 *   width   x size in pixels = cols * CELL_W.
 *   height  y size in pixels = rows * CELL_H. (CELL_H is double CELL_W to
 *           offset the fact that terminal cells are taller than they are wide.)
 */
typedef struct {
    float width;
    float height;
} World;

/* Build the pixel-space World from a terminal (cols, rows). Called at startup
 * and after each resize so the physics box matches what's on screen. */
static inline World world_from_terminal(int cols, int rows)
{
    return (World){ .width = pw(cols), .height = ph(rows) };
}

/* Pixel position to the nearest cell. The +0.5-then-floor rounding avoids a
 * jitter that plain rounding causes right on cell boundaries. */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/*
 * Vec2 — a 2-D point or arrow (x, y) in pixels.
 *
 * Everything with an x and a y is a Vec2: positions, velocities, forces,
 * offsets. The v2* helpers below let you add and scale them so the steering
 * code reads close to the math it implements. Only scene_draw ever converts
 * these pixels into terminal cells; the physics never deals in cells.
 *
 * Members
 *   x, y   the two components, in pixels.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float v2len2(Vec2 v)               { return v.x*v.x + v.y*v.y; }

/* Direction of v as a length-1 arrow; zero if v is basically zero. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* Shorten v to at most max_len, keeping its direction. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/*
 * Shortest step from a to b along one wrapped axis of length `extent`.
 * On a torus there are two ways round; this picks the shorter one and keeps
 * the sign so you know which way. Example (extent 100): a=5, b=95 gives -10,
 * not +90 — going off the left edge is closer. Every steering force uses this
 * so birds near opposite edges still count as close neighbours.
 */
static inline float toroidal_delta(float a, float b, float extent)
{
    float d = b - a;
    if (d >  extent * 0.5f) d -= extent;
    if (d < -extent * 0.5f) d += extent;
    return d;
}

static inline float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ── §5 boid — one bird: state, spawn, speed clamp, wrap ── */

/*
 * Boid — one bird ("bird-oid", from Reynolds 1987).
 *
 * A bird is just a moving dot: it has a position and a velocity. Each tick the
 * steering rules nudge its velocity, then the velocity moves its position.
 * There's no mass (every bird weighs 1, so a force feeds straight into speed)
 * and no stored heading (the direction is implied by the velocity). The whole
 * flock is an array of these. Positions are kept in pixels, not cells, so
 * motion stays smooth instead of jumping cell to cell.
 *
 * Members
 *   pos          current position, in pixels. Always inside the world after
 *                wrapping. Binning turns this into a (col, row) for drawing.
 *   prev_pos     position at the start of the tick. Snapshotted but unused by
 *                the current renderer; kept for a possible per-bird draw mode.
 *   vel          velocity, in pixels per second. Kept between MIN and MAX
 *                speed each tick (a floor so birds never freeze, a ceiling so
 *                a hawk-flee can't fling them away).
 *   color_pair   a spawn-time colour, 1..N_COLORS. Cosmetic only — the density
 *                renderer colours by cell position, not per bird.
 */
typedef struct {
    Vec2 pos;
    Vec2 prev_pos;     /* snapshotted but unused by the current renderer */
    Vec2 vel;
    int  color_pair;   /* cosmetic; density renderer ignores it */
} Boid;

/* Place a bird at a random spot, moving in a random direction at half cruise
 * speed so the flock is already in motion on frame one. */
static void boid_spawn(Boid *b, int id, World world)
{
    b->pos      = v2(randf() * world.width, randf() * world.height);
    b->prev_pos = b->pos;

    float ang = randf() * 2.0f * (float)M_PI;
    b->vel    = v2(cosf(ang) * BOID_SPEED * 0.5f,
                   sinf(ang) * BOID_SPEED * 0.5f);

    b->color_pair = (id % N_COLORS) + 1;
}

/* Keep a bird's speed within sane limits. A floor stops it freezing when the
 * steering forces happen to cancel out; a ceiling stops a hawk-flee from
 * launching it off the screen. */
static void boid_clamp_speed(Boid *b)
{
    const float STALL_EPSILON = 0.001f;     /* below this = no direction left */
    float current_speed = v2len(b->vel);

    /* Dead stop: forces cancelled out. Kick off in a random direction so the
     * bird never just sits there frozen. */
    if (current_speed < STALL_EPSILON) {
        float random_heading_rad = randf() * 2.0f * (float)M_PI;
        b->vel = v2(cosf(random_heading_rad) * BOID_MIN_SPEED,
                    sinf(random_heading_rad) * BOID_MIN_SPEED);
        return;
    }

    /* Too slow / too fast: rescale to the limit, keeping direction. */
    if (current_speed < BOID_MIN_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MIN_SPEED);
        return;
    }

    if (current_speed > BOID_MAX_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MAX_SPEED);
    }
}

/* Wrap a bird that walked off an edge back onto the opposite edge, so the
 * flocking looks the same everywhere with no edge where birds pile up. */
static void boid_wrap(Boid *b, World world)
{
    if (b->pos.x <  0.0f)        b->pos.x += world.width;
    if (b->pos.x >= world.width) b->pos.x -= world.width;
    if (b->pos.y <  0.0f)        b->pos.y += world.height;
    if (b->pos.y >= world.height)b->pos.y -= world.height;
}

/* ── §6 forces — separation, alignment, cohesion, hawk flee ── */
/* These turn the per-rule sums gathered in §6.5 into steering forces.
 * The neighbour scan that builds those sums lives in §6.5. */

/* Alignment: steer toward the average heading of nearby birds. Given the sum
 * of their velocities and how many there were, aim for that average. Zero if
 * nobody was in range. (Reynolds rule 2.) */
static Vec2 align_force(Vec2 ali_vsum, int ali_n, Vec2 my_vel)
{
    if (ali_n == 0) return v2(0, 0);

    Vec2 mean_neighbour_velocity = v2scale(ali_vsum, 1.0f / (float)ali_n);
    return v2sub(mean_neighbour_velocity, my_vel);
}

/* Cohesion: steer toward the centre of the nearby group. We feed in the sum of
 * *offsets* to neighbours, not their raw positions, because averaging raw
 * positions on a wrapped world would point at the screen centre even when the
 * flock sits at one edge. Zero if nobody was in range. (Reynolds rule 3.) */
static Vec2 cohere_force(Vec2 coh_dsum, int coh_n, Vec2 my_vel)
{
    if (coh_n == 0) return v2(0, 0);

    Vec2 mean_offset_to_centre_of_mass = v2scale(coh_dsum, 1.0f / (float)coh_n);
    Vec2 toward_centre_dir             = v2norm(mean_offset_to_centre_of_mass);
    Vec2 desired_velocity              = v2scale(toward_centre_dir, BOID_SPEED);
    return v2sub(desired_velocity, my_vel);
}

/* Flee: push the bird away from the hawk, harder the closer it is, and nothing
 * once the hawk is outside the panic radius. Uses the wrapped distance so a
 * hawk near the far edge still scares birds near the near edge. */
static Vec2 hawk_flee_force(Vec2 me_pos, Vec2 hawk_pos, World world)
{
    const float DIST_EPSILON_SQ = 1e-6f;
    const float HAWK_FLEE_RADIUS_SQ = HAWK_FLEE_RADIUS * HAWK_FLEE_RADIUS;

    /* Compare squared distance first so we can bail without a square root when
     * the hawk is out of range — which is true for most birds most of the time. */
    float offset_x          = toroidal_delta(me_pos.x, hawk_pos.x, world.width);
    float offset_y          = toroidal_delta(me_pos.y, hawk_pos.y, world.height);
    float squared_distance  = offset_x*offset_x + offset_y*offset_y;

    bool inside_panic_zone = squared_distance < HAWK_FLEE_RADIUS_SQ;
    bool well_separated    = squared_distance > DIST_EPSILON_SQ;
    if (!inside_panic_zone || !well_separated) return v2(0, 0);

    /* In range: panic_intensity is near 1 right next to the hawk and fades to 0
     * at the edge of the radius; push directly away from the hawk by that much. */
    float distance_to_hawk = sqrtf(squared_distance);
    float panic_intensity  = (HAWK_FLEE_RADIUS - distance_to_hawk) / HAWK_FLEE_RADIUS;
    Vec2  away_from_hawk_dir = v2(-offset_x / distance_to_hawk,
                                  -offset_y / distance_to_hawk);
    return v2scale(away_from_hawk_dir, panic_intensity * FLEE_SPEED);
}

/* ── §6.5 hash — spatial hash for fast neighbour search ── */
/*
 * The slow way to find each bird's neighbours is to test it against every
 * other bird, which gets painfully slow at 1500 birds. Instead we drop every
 * bird into a coarse grid of cells, then a bird only has to look at the birds
 * in its own cell and the eight around it.
 *
 * The grid is stored as one linked list per cell, packed into two arrays:
 *
 *      head[gy][gx]   index of the first bird in that cell, or -1 if empty
 *      next[bird]     index of the next bird in the same cell, or -1 at the end
 *
 * Build: clear all the heads, then push each bird onto the front of its cell's
 * list. Query: for each bird, walk the 3x3 block of cells around it.
 *
 * The cell size matches the largest neighbour radius, so the 3x3 block is
 * guaranteed to contain every neighbour that could matter. One wrinkle: if the
 * world is so small it's only one or two cells wide, wrapping {-1,0,+1} would
 * visit the same cell twice and double-count, so along a tiny axis we just
 * sweep all of its cells once with no wrap.
 */

/*
 * SpatialHash — the grid of per-cell bird lists, rebuilt every tick.
 *
 * This is what makes neighbour search fast (see the note above). It lives on
 * the Scene so it travels with the simulation it indexes.
 *
 * Members
 *   head[gy][gx]   first bird in that cell, or -1 if the cell is empty.
 *   next[i]        next bird in the same cell as bird i, or -1 if i is last.
 *                  Indexed by bird, in step with the pool array.
 *   grid_w, grid_h this tick's grid size (depends on the world size), each
 *                  at most the matching HASH_GRID_*_MAX.
 *
 * It's cleared and refilled from scratch each tick, so there's nothing to
 * tear down. Reference: Teschner et al. 2003 (uniform spatial hashing).
 */
typedef struct {
    int head[HASH_GRID_H_MAX][HASH_GRID_W_MAX];
    int next[N_BOIDS_MAX];
    int grid_w;
    int grid_h;
} SpatialHash;

/* Fold a cell index that may be out of range back into [0, extent), wrapping. */
static inline int hash_wrap_cell(int idx, int extent)
{
    int r = idx % extent;
    if (r < 0) r += extent;
    return r;
}

/* Pixel position to grid cell (without wrapping; the caller wraps). */
static inline void hash_cell_for_pos(Vec2 pos, int *out_cx, int *out_cy)
{
    *out_cx = (int)floorf(pos.x / (float)HASH_CELL_PX);
    *out_cy = (int)floorf(pos.y / (float)HASH_CELL_PX);
}

/* Refill the hash from the current bird positions: size the grid to the world,
 * empty every cell, then drop each bird into its cell. Called once per tick
 * before steering so every bird sees the same neighbour grid. */
static void spatial_hash_build(SpatialHash *h, const Boid *pool, int n,
                               World world)
{
    /* (1) grid size = world rounded up to whole cells, capped */
    int gw = (int)ceilf(world.width  / (float)HASH_CELL_PX);
    int gh = (int)ceilf(world.height / (float)HASH_CELL_PX);
    if (gw < 1)               gw = 1;
    if (gh < 1)               gh = 1;
    if (gw > HASH_GRID_W_MAX) gw = HASH_GRID_W_MAX;
    if (gh > HASH_GRID_H_MAX) gh = HASH_GRID_H_MAX;
    h->grid_w = gw;
    h->grid_h = gh;

    /* (2) empty every cell */
    for (int gy = 0; gy < gh; gy++)
        for (int gx = 0; gx < gw; gx++)
            h->head[gy][gx] = -1;

    /* (3) push each bird onto the front of its cell's list */
    for (int i = 0; i < n; i++) {
        int cx, cy;
        hash_cell_for_pos(pool[i].pos, &cx, &cy);
        cx = hash_wrap_cell(cx, gw);
        cy = hash_wrap_cell(cy, gh);
        h->next[i]      = h->head[cy][cx];
        h->head[cy][cx] = i;
    }
}

/*
 * For one pair (this bird and a neighbour), add the neighbour's contribution
 * to the running totals for the three rules — but only for the rules whose
 * radius the neighbour is inside. Skips the bird itself and exact overlaps.
 * The caller has already narrowed things to nearby cells; this is per-pair.
 */
static inline void accumulate_pair_force(const Boid *me, const Boid *nb,
                                         int j, int self, World world,
                                         Vec2 *sep_force,
                                         Vec2 *ali_vsum, int *ali_n,
                                         Vec2 *coh_dsum, int *coh_n)
{
    const float DIST_EPSILON_SQ = 1e-6f;
    const float SEP_R2          = SEP_RADIUS      * SEP_RADIUS;
    const float ALIGN_R2        = ALIGN_RADIUS    * ALIGN_RADIUS;
    const float COH_R2          = COHESION_RADIUS * COHESION_RADIUS;
    if (j == self) return;

    /* Wrapped offset to the neighbour, and squared distance for cheap
     * comparisons before paying for a square root. */
    float offset_x         = toroidal_delta(me->pos.x, nb->pos.x, world.width);
    float offset_y         = toroidal_delta(me->pos.y, nb->pos.y, world.height);
    float squared_distance = offset_x*offset_x + offset_y*offset_y;

    bool inside_perception = squared_distance <= COH_R2;
    bool well_separated    = squared_distance >  DIST_EPSILON_SQ;
    if (!inside_perception || !well_separated) return;

    /* Cohesion: add the offset (not the raw position — see cohere_force). */
    coh_dsum->x += offset_x;
    coh_dsum->y += offset_y;
    (*coh_n)++;

    /* Alignment: add the neighbour's velocity, if it's in the closer align ring. */
    if (squared_distance < ALIGN_R2) {
        ali_vsum->x += nb->vel.x;
        ali_vsum->y += nb->vel.y;
        (*ali_n)++;
    }

    /* Separation: only for neighbours inside personal space. Push away from the
     * neighbour, harder the closer it is; the square root is paid only here, in
     * this rare branch. */
    if (squared_distance < SEP_R2) {
        float distance                 = sqrtf(squared_distance);
        float personal_space_intrusion = (SEP_RADIUS - distance) / SEP_RADIUS;
        float push_away_x              = -offset_x / distance;
        float push_away_y              = -offset_y / distance;
        sep_force->x += push_away_x * personal_space_intrusion * BOID_SPEED;
        sep_force->y += push_away_y * personal_space_intrusion * BOID_SPEED;
    }
}

/*
 * Total steering force for one bird, using the hash to find neighbours fast.
 * Finds the bird's cell, walks the 3x3 block of cells around it tallying the
 * three rules, then turns those tallies into forces, adds the hawk-flee, and
 * caps the result. Reads only — it never moves any bird, so scene_tick can
 * gather all the new velocities first and apply them together. */
static Vec2 boid_forces(const Boid *pool, const SpatialHash *hash, int self,
                        Vec2 hawk_pos, World world)
{
    const Boid *me = &pool[self];

    /* Running totals, one set per rule. coh_dsum holds offsets, not positions. */
    Vec2 sep_force = v2(0, 0);
    Vec2 ali_vsum  = v2(0, 0); int ali_n = 0;
    Vec2 coh_dsum  = v2(0, 0); int coh_n = 0;

    /* (1) this bird's cell */
    int self_cx, self_cy;
    hash_cell_for_pos(me->pos, &self_cx, &self_cy);
    self_cx = hash_wrap_cell(self_cx, hash->grid_w);
    self_cy = hash_wrap_cell(self_cy, hash->grid_h);

    /* (2) pick the sweep range for each axis: the normal {-1,0,+1} with wrap,
     *     or, if the grid is under 3 cells on that axis, every cell once with
     *     no wrap (so a tiny world doesn't double-count). */
    int dy_lo, dy_hi, dy_use_wrap;
    if (hash->grid_h >= 3) { dy_lo = -1;            dy_hi =  1;                 dy_use_wrap = 1; }
    else                   { dy_lo =  0;            dy_hi = hash->grid_h - 1;   dy_use_wrap = 0; }

    int dx_lo, dx_hi, dx_use_wrap;
    if (hash->grid_w >= 3) { dx_lo = -1;            dx_hi =  1;                 dx_use_wrap = 1; }
    else                   { dx_lo =  0;            dx_hi = hash->grid_w - 1;   dx_use_wrap = 0; }

    /* (3) walk those cells, tallying every bird in them */
    for (int dy = dy_lo; dy <= dy_hi; dy++) {
        int cy = dy_use_wrap ? hash_wrap_cell(self_cy + dy, hash->grid_h) : dy;
        for (int dx = dx_lo; dx <= dx_hi; dx++) {
            int cx = dx_use_wrap ? hash_wrap_cell(self_cx + dx, hash->grid_w) : dx;
            for (int j = hash->head[cy][cx]; j != -1; j = hash->next[j]) {
                accumulate_pair_force(me, &pool[j], j, self, world,
                                      &sep_force,
                                      &ali_vsum, &ali_n,
                                      &coh_dsum, &coh_n);
            }
        }
    }

    /* (4) turn the tallies into forces */
    Vec2 ali  = align_force    (ali_vsum, ali_n, me->vel);
    Vec2 coh  = cohere_force   (coh_dsum, coh_n, me->vel);
    Vec2 flee = hawk_flee_force(me->pos, hawk_pos, world);

    /* (5) blend by weight and cap the turn so the path stays smooth */
    Vec2 total = v2add(
        v2add(v2scale(sep_force, W_SEP),
              v2scale(ali,       W_ALIGN)),
        v2add(v2scale(coh,       W_COHERE),
              v2scale(flee,      W_FLEE))
    );
    return v2clamp_len(total, MAX_STEER);
}

/* ── §7 hawk — the predator and its PATROL/HOVER/PURSUE/DIVE modes ── */

/*
 * HawkMode — what the hawk is doing right now.
 *
 * There are three "base" modes the user cycles with n/p, plus DIVE, an overlay
 * fired by SPACE (or the auto-dive timer) on top of whatever base is active.
 * When a dive finishes, the hawk returns to its base mode rather than jumping
 * anywhere.
 *
 *   PATROL  circle the centre of the world at a steady angular speed; the hawk
 *           looks like it's watching, not chasing.
 *   HOVER   drift onto the flock's centre and park there, easing to a stop, so
 *           the flock has to bend around it (a moving doughnut shape).
 *   PURSUE  chase the flock's centre nonstop at just above bird speed, slowly
 *           gaining, so the flock keeps running.
 *   DIVE    a fast straight sprint at the flock for a fixed time — a raptor's
 *           stoop. The flock splits and reforms around it.
 *
 * Members
 *   HAWK_PATROL/HOVER/PURSUE  the three base modes (0..2).
 *   HAWK_DIVE                 the overlay; kept last, never in the n/p cycle.
 *   HAWK_MODE_COUNT_BASE      equals HAWK_DIVE, i.e. the count of base modes (3).
 *
 * References: Reynolds 1999 (steering), Couzin et al. 2002 (doughnut shapes),
 * Hildenbrandt et al. 2010 and Cavagna et al. 2010 (hawk dives on real flocks).
 */
typedef enum {
    HAWK_PATROL = 0,
    HAWK_HOVER,
    HAWK_PURSUE,
    HAWK_DIVE,                  /* always last — overlay, not in n/p cycle */
    HAWK_MODE_COUNT_BASE = HAWK_DIVE  /* number of cyclable base modes (3) */
} HawkMode;

static const char *HAWK_MODE_NAMES[] = {
    [HAWK_PATROL] = "PATROL",
    [HAWK_HOVER ] = "HOVER ",
    [HAWK_PURSUE] = "PURSUE",
    [HAWK_DIVE  ] = "DIVE  ",
};

/*
 * Hawk — the single predator and the state its modes need.
 *
 * One Hawk lives on the Scene. Each tick the stepper for its current mode moves
 * it, and every bird feels a flee force from it. All four modes share one flat
 * struct; a given mode only touches the fields it needs (PATROL uses
 * patrol_phase, DIVE uses dive_timer, and so on).
 *
 * It keeps two mode fields. `mode` is what's running now (which stepper to
 * call); `base_mode` is what to return to when a dive ends. Without the second
 * one, a dive launched from HOVER would wrongly snap back to PATROL.
 *
 * Members
 *   pos          position in pixels; the centre every bird flees from.
 *   vel          velocity in pixels/sec. Used by HOVER, PURSUE, DIVE. PATROL
 *                ignores it — it sets pos straight from its orbit equation.
 *   mode         the live mode. Also drives the HUD label and the hawk glyph.
 *   base_mode    the mode to return to after a dive (one of the three bases).
 *   patrol_phase the orbit angle, in radians, ticked forward while patrolling.
 *                When a dive ends back into PATROL, it's recomputed from the
 *                current position so the orbit picks up where the dive left off.
 *   dive_timer   seconds left in the current dive; ignored outside DIVE.
 *   auto_dive    when on ('h' key), dives fire on a timer with no keypresses.
 *   auto_dive_timer  seconds since the last dive; only ticks while auto_dive is
 *                    on and a dive isn't already running.
 *
 * References: Reynolds 1999, Hildenbrandt et al. 2010, Cavagna et al. 2010.
 */
typedef struct {
    Vec2 pos;
    Vec2 vel;        /* used by HOVER / PURSUE / DIVE; PATROL recomputes pos */

    HawkMode mode;            /* current mode — includes DIVE overlay   */
    HawkMode base_mode;       /* PATROL/HOVER/PURSUE — DIVE returns here */
    float    patrol_phase;    /* radians around world centre            */
    float    dive_timer;      /* seconds remaining in current dive      */

    bool     auto_dive;
    float    auto_dive_timer; /* seconds since last (auto-)fired dive */
} Hawk;

/* Step the base mode forward (n) or backward (p) through PATROL/HOVER/PURSUE.
 * Mid-dive this only changes what the hawk returns to afterward; otherwise it
 * switches the live mode too. */
static void hawk_cycle_base_mode(Hawk *h, int dir)
{
    int next = ((int)h->base_mode + dir + HAWK_MODE_COUNT_BASE)
             % HAWK_MODE_COUNT_BASE;
    h->base_mode = (HawkMode)next;
    if (h->mode != HAWK_DIVE) h->mode = h->base_mode;
}

/* Start a dive at `target` (usually the flock's centre). Does nothing if a dive
 * is already running. The dive flies in a straight line and ignores wrap, so if
 * the target is across an edge the hawk takes the long way — harmless, since it
 * resumes its base mode the moment the dive ends. */
static void hawk_dive(Hawk *h, Vec2 target)
{
    if (h->mode == HAWK_DIVE) return;
    Vec2 dir = v2norm(v2sub(target, h->pos));
    if (v2len(dir) < 0.001f) return;       /* hawk already on the target */

    h->vel        = v2scale(dir, HAWK_DIVE_SPEED);
    h->dive_timer = HAWK_DIVE_DURATION;
    h->mode       = HAWK_DIVE;
}

/* ── per-mode hawk steppers ─────────────────────────────────────────── */

/* PATROL: advance the orbit angle and place the hawk on the circle. No velocity
 * is used — the position comes straight from the angle. */
static void hawk_step_patrol(Hawk *h, Vec2 world_centre, float patrol_r, float dt)
{
    h->patrol_phase += HAWK_PATROL_OMEGA * dt;
    Vec2 orbit_offset = v2scale(v2(cosf(h->patrol_phase),
                                    sinf(h->patrol_phase)), patrol_r);
    h->pos = v2add(world_centre, orbit_offset);
}

/* HOVER: drift toward the flock's centre and ease to a stop near it. Outside
 * the hold radius it moves at full drift speed; inside, the speed shrinks with
 * the distance so it settles smoothly instead of stopping with a jerk. (Plain
 * line-of-sight, no wrap — hover is a close-range approach.) */
static void hawk_step_hover(Hawk *h, Vec2 centroid, float dt)
{
    Vec2  offset_to_centroid = v2sub(centroid, h->pos);
    float distance           = v2len(offset_to_centroid);
    if (distance < 0.001f) { h->vel = v2(0, 0); return; }

    float settle_fraction = (distance < HAWK_HOVER_HOLD_RADIUS)
                          ? (distance / HAWK_HOVER_HOLD_RADIUS)
                          : 1.0f;
    float drift_speed     = HAWK_HOVER_DRIFT * settle_fraction;

    Vec2 toward_centroid = v2scale(offset_to_centroid, 1.0f / distance);
    h->vel = v2scale(toward_centroid, drift_speed);
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
}

/* PURSUE: chase the flock's centre at full chase speed, no settling — the hawk
 * is actively trying to close the gap. */
static void hawk_step_pursue(Hawk *h, Vec2 centroid, float dt)
{
    Vec2  offset_to_centroid = v2sub(centroid, h->pos);
    float distance           = v2len(offset_to_centroid);
    if (distance < 0.001f) { h->vel = v2(0, 0); return; }

    Vec2 toward_centroid = v2scale(offset_to_centroid, 1.0f / distance);
    h->vel = v2scale(toward_centroid, HAWK_PURSUE_SPEED);
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
}

/* DIVE: fly straight at full dive speed, counting down the timer. When it runs
 * out, return to the base mode. If that's PATROL, recompute the orbit angle from
 * the current spot so the circle resumes here instead of snapping back. */
static void hawk_step_dive(Hawk *h, Vec2 world_centre, float dt)
{
    h->pos = v2add(h->pos, v2scale(h->vel, dt));
    h->dive_timer -= dt;
    if (h->dive_timer > 0.0f) return;

    if (h->base_mode == HAWK_PATROL) {
        h->patrol_phase = atan2f(h->pos.y - world_centre.y,
                                  h->pos.x - world_centre.x);
    }
    h->mode = h->base_mode;
}

/* Wrap the hawk back on-screen, so a long dive doesn't fly it off the edge. */
static void hawk_wrap(Hawk *h, World world)
{
    if (h->pos.x <  0.0f)         h->pos.x += world.width;
    if (h->pos.x >= world.width)  h->pos.x -= world.width;
    if (h->pos.y <  0.0f)         h->pos.y += world.height;
    if (h->pos.y >= world.height) h->pos.y -= world.height;
}

/* Run the stepper for the hawk's current mode, then wrap it on-screen. The
 * caller passes the flock centre (the target for HOVER/PURSUE/DIVE; PATROL
 * ignores it). */
static void hawk_step(Hawk *h, Vec2 world_centre, float patrol_r,
                      Vec2 flock_centroid, World world, float dt)
{
    switch (h->mode) {
        case HAWK_PATROL: hawk_step_patrol(h, world_centre, patrol_r,    dt); break;
        case HAWK_HOVER:  hawk_step_hover (h, flock_centroid,            dt); break;
        case HAWK_PURSUE: hawk_step_pursue(h, flock_centroid,            dt); break;
        case HAWK_DIVE:   hawk_step_dive  (h, world_centre,              dt); break;
        default: break;
    }
    hawk_wrap(h, world);
}

/* ── §8 scene — flock + hawk state, the tick, and the density renderer ── */

/*
 * SimControls — the playback knobs the user can change.
 *
 * Members
 *   paused      true means the simulation is frozen; the HUD shows PAUSED.
 *   theme_idx   which flock palette is active, 0..N_THEMES-1 (t/T cycles it).
 */
typedef struct {
    bool paused;
    int  theme_idx;
} SimControls;

/*
 * Scene — all of one run's simulation state in one place.
 *
 * Every persistent simulation value hangs off a single Scene: the flock, the
 * hawk, the user knobs, the world size, the flock centre, and the per-tick
 * neighbour grid. (Terminal size and ncurses live on Screen; signal flags on
 * App, since signal handlers can't be handed a pointer.) Keeping it in one
 * struct rather than loose globals means a helper that takes `Scene *` is the
 * one place to look for what state exists, and a future split-screen view could
 * just allocate a second Scene.
 *
 * Members
 *   pool[]          the bird pool; only the first n_birds are active.
 *   n_birds         how many birds are live right now.
 *   hawk            the single predator.
 *   sim             paused + theme.
 *   world           world size in pixels; refreshed on resize.
 *   flock_centroid  the flock's centre, recomputed each tick (a dive target).
 *   hash            the neighbour grid, rebuilt each tick.
 */
typedef struct {
    Boid  pool[N_BOIDS_MAX];
    int   n_birds;
    Hawk  hawk;
    SimControls sim;
    World world;
    Vec2  flock_centroid;
    SpatialHash hash;
} Scene;

/* The geometric centre of the world. Used as the patrol orbit centre, the
 * starting centroid, and a dive's return point. */
static inline Vec2 world_centre(World world)
{
    const float WORLD_CENTRE_FRAC = 0.5f;
    return v2(world.width  * WORLD_CENTRE_FRAC,
              world.height * WORLD_CENTRE_FRAC);
}

/* The hawk's orbit radius, scaled to the smaller of width/height so the circle
 * stays fully on screen even on a long-and-thin terminal. */
static inline float hawk_patrol_radius_for(World world)
{
    float min_dim = (world.width < world.height) ? world.width : world.height;
    return min_dim * HAWK_PATROL_FRAC;
}

/* Spawn the whole pool at once. The +/- keys just change how many are active,
 * so they reveal already-placed birds rather than re-randomising. */
static void spawn_boid_pool(Boid *pool, World world)
{
    for (int i = 0; i < N_BOIDS_MAX; i++)
        boid_spawn(&pool[i], i, world);
}

/* Put the hawk at the right-hand point of its orbit and reset its controller
 * state, so frame one shows it away from where the birds spawned. */
static void place_hawk_at_orbit_east(Hawk *h, World world)
{
    Vec2  centre = world_centre(world);
    float r      = hawk_patrol_radius_for(world);
    h->pos             = v2add(centre, v2(r, 0));
    h->vel             = v2(0, 0);
    h->mode            = HAWK_PATROL;
    h->base_mode       = HAWK_PATROL;
    h->patrol_phase    = 0.0f;
    h->dive_timer      = 0.0f;
    h->auto_dive       = false;
    h->auto_dive_timer = 0.0f;
}

/* Start (or reset) a scene: random birds, hawk on its orbit. The chosen theme
 * survives a reset (saved before the wipe, restored after). */
static void scene_init(Scene *s, int cols, int rows)
{
    int saved_theme = s->sim.theme_idx;
    memset(s, 0, sizeof *s);
    s->sim.theme_idx = saved_theme;
    s->sim.paused    = false;

    s->world   = world_from_terminal(cols, rows);
    s->n_birds = N_BOIDS_DEFAULT;

    spawn_boid_pool(s->pool, s->world);
    place_hawk_at_orbit_east(&s->hawk, s->world);

    /* No bird has moved yet, so start the centroid at the world centre. */
    s->flock_centroid = world_centre(s->world);
}

/* The flock's centre of mass, picked as the hawk's dive target. Computed in a
 * wrap-aware way: pick one bird as a reference and average the *offsets* to
 * every other bird, so a flock straddling an edge still gets the right centre
 * (same trick as cohere_force). */
static Vec2 scene_centroid(const Boid *pool, int n, World world)
{
    if (n <= 0) return v2(world.width * 0.5f, world.height * 0.5f);

    Vec2 reference_pos = pool[0].pos;
    Vec2 offset_sum    = v2(0, 0);
    for (int i = 0; i < n; i++) {
        offset_sum.x += toroidal_delta(reference_pos.x, pool[i].pos.x, world.width);
        offset_sum.y += toroidal_delta(reference_pos.y, pool[i].pos.y, world.height);
    }

    Vec2 mean_offset_from_reference = v2scale(offset_sum, 1.0f / (float)n);
    Vec2 centroid_pos = v2add(reference_pos, mean_offset_from_reference);

    /* Wrap the result back into the world so callers get a valid position. */
    if (centroid_pos.x <  0.0f)         centroid_pos.x += world.width;
    if (centroid_pos.x >= world.width)  centroid_pos.x -= world.width;
    if (centroid_pos.y <  0.0f)         centroid_pos.y += world.height;
    if (centroid_pos.y >= world.height) centroid_pos.y -= world.height;
    return centroid_pos;
}

/* When auto-dive is on, count up and fire a dive every AUTO_DIVE_PERIOD. Only
 * counts while the hawk isn't already diving, so dives don't stack up. */
static void tick_auto_dive_timer(Hawk *h, Vec2 flock_centroid, float dt)
{
    bool eligible = h->auto_dive && h->mode != HAWK_DIVE;
    if (!eligible) return;

    h->auto_dive_timer += dt;
    if (h->auto_dive_timer >= AUTO_DIVE_PERIOD) {
        h->auto_dive_timer = 0.0f;
        hawk_dive(h, flock_centroid);
    }
}

/* Step 1 of the two-step update: compute every bird's new velocity from the
 * current positions and write it into new_vel, without moving anything yet. So
 * no bird reacts to a neighbour that has already moved this tick. */
static void compute_new_velocities(const Boid *pool, const SpatialHash *hash,
                                   int n, Vec2 hawk_pos, World world,
                                   float dt, Vec2 *new_vel)
{
    for (int i = 0; i < n; i++) {
        Vec2 force      = boid_forces(pool, hash, i, hawk_pos, world);
        Vec2 raw_new_v  = v2add(pool[i].vel, v2scale(force, dt));
        new_vel[i]      = v2clamp_len(raw_new_v, BOID_MAX_SPEED);
    }
}

/* Step 2 of the two-step update: now that every new velocity is known, apply
 * them — store the velocity, save the old position, move, wrap, clamp speed. */
static void commit_step_and_wrap(Boid *pool, int n, const Vec2 *new_vel,
                                 World world, float dt)
{
    for (int i = 0; i < n; i++) {
        Boid *b      = &pool[i];
        b->vel       = new_vel[i];
        b->prev_pos  = b->pos;
        b->pos       = v2add(b->pos, v2scale(b->vel, dt));
        boid_wrap(b, world);
        boid_clamp_speed(b);
    }
}

/*
 * One physics tick: fire the auto-dive if it's due, move the hawk, rebuild the
 * neighbour hash, then run the two-step bird update (compute all new velocities,
 * then apply them), and finally refresh the flock centre. The two-step update
 * is what stops birds drifting because an earlier one in the array already
 * moved — see flocking.c for the same pattern explained in detail.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    World world = s->world;

    tick_auto_dive_timer(&s->hawk, s->flock_centroid, dt);

    hawk_step(&s->hawk,
              world_centre(world),
              hawk_patrol_radius_for(world),
              s->flock_centroid, world, dt);

    /* Build the neighbour hash once so every bird this tick sees the same one. */
    spatial_hash_build(&s->hash, s->pool, s->n_birds, world);

    static Vec2 new_vel[N_BOIDS_MAX];   /* static so it stays off the stack */
    compute_new_velocities(s->pool, &s->hash, s->n_birds,
                           s->hawk.pos, world, dt, new_vel);

    commit_step_and_wrap(s->pool, s->n_birds, new_vel, world, dt);

    s->flock_centroid = scene_centroid(s->pool, s->n_birds, world);
}

/* ── render ──────────────────────────────────────────────────────────── */

/* Draw one ASCII glyph at terminal cell (cx, cy), or drop it if off-screen.
 * The double cast stops a char above 127 turning negative and corrupting the
 * output (a known ncurses gotcha). */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* The per-frame grid of how many birds are in each cell. Kept static (not on
 * the stack, not re-allocated) and only the visible part is ever touched. */
static int g_density[MAX_ROWS][MAX_COLS];

/* Map a cell's bird count to a glyph and brightness: sparse cells get faint
 * dots from the ramp, dense cells get bold bright '@'. There's no dim tier on
 * purpose — most cells hold only one or two birds, and dimming them would make
 * the flock edges nearly invisible. The fade comes from the glyph ramp, not
 * from dimming. */
static void density_glyph(int density, char *out_ch, attr_t *out_attr)
{
    int idx = density - 1;
    if (idx < 0)            idx = 0;
    if (idx >= RAMP_LEN)    idx = RAMP_LEN - 1;
    *out_ch = DENSITY_RAMP[idx];

    *out_attr = (density >= 5) ? A_BOLD : A_NORMAL;
}

/* Clear the visible part of the density grid (no need to touch the rest). */
static void zero_density_region(int cap_cols, int cap_rows)
{
    for (int r = 0; r < cap_rows; r++)
        for (int c = 0; c < cap_cols; c++)
            g_density[r][c] = 0;
}

/* Tally the birds into the grid: for each bird, bump the count in its cell
 * (skipping any that fall off-screen). */
static void bin_boids_into_density(const Boid *pool, int n_birds,
                                   int cap_cols, int cap_rows)
{
    for (int i = 0; i < n_birds; i++) {
        int cx = px_to_cell_x(pool[i].pos.x);
        int cy = px_to_cell_y(pool[i].pos.y);
        if (cx < 0 || cx >= cap_cols || cy < 0 || cy >= cap_rows) continue;
        g_density[cy][cx]++;
    }
}

/* Pick a flock colour for a cell from its position, so neighbouring cells get
 * different colours and the cloud looks softly mottled instead of one flat
 * tint. The exact arithmetic doesn't matter much — any small odd multiplier
 * spreads the colours about evenly. */
static inline int tint_pair_for_cell(int cx, int cy)
{
    const int SPATIAL_HASH_PRIME = 7;
    return ((cy * SPATIAL_HASH_PRIME + cx) % N_COLORS) + 1;
}

/* Paint the cloud: for each non-empty cell, draw its density glyph in its tint.
 * This is the step that makes a thousand-plus birds read as one cloud. */
static void render_density_field(WINDOW *w, int cap_cols, int cap_rows,
                                 int cols, int rows)
{
    for (int cy = 0; cy < cap_rows; cy++) {
        for (int cx = 0; cx < cap_cols; cx++) {
            int density_in_cell = g_density[cy][cx];
            if (density_in_cell == 0) continue;

            char   glyph;
            attr_t brightness_tier;
            density_glyph(density_in_cell, &glyph, &brightness_tier);

            int tint_pair = tint_pair_for_cell(cx, cy);
            mark_cell(w, cx, cy, glyph, tint_pair, brightness_tier, cols, rows);
        }
    }
}

/* Draw the hawk on top of the flock: '!' while diving, 'X' otherwise, always in
 * bold red so it stands out against any theme. */
static void stamp_hawk_glyph(WINDOW *w, const Hawk *hawk, int cols, int rows)
{
    int  cx       = px_to_cell_x(hawk->pos.x);
    int  cy       = px_to_cell_y(hawk->pos.y);
    char hawk_glyph = (hawk->mode == HAWK_DIVE) ? '!' : 'X';
    mark_cell(w, cx, cy, hawk_glyph, PAIR_HAWK, A_BOLD, cols, rows);
}

/* Draw the whole scene: tally birds into the density grid, paint the cloud from
 * it, then stamp the hawk on top. */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    /* Don't draw past the density buffer on a very large terminal. */
    int cap_rows = (rows < MAX_ROWS) ? rows : MAX_ROWS;
    int cap_cols = (cols < MAX_COLS) ? cols : MAX_COLS;

    zero_density_region(cap_cols, cap_rows);
    bin_boids_into_density(s->pool, s->n_birds, cap_cols, cap_rows);
    render_density_field(w, cap_cols, cap_rows, cols, rows);
    stamp_hawk_glyph(w, &s->hawk, cols, rows);
}

/* ── §9 app — screen, signals, input, and the main loop ── */

/*
 * Screen — the terminal size in cells, and the ncurses wrapper.
 *
 * This is the terminal side of the world, kept separate from Scene.world (the
 * pixel side). The two stay in step (World.width = cols * CELL_W, and so on),
 * but keeping them apart means the physics only ever deals in pixels and the
 * renderer only ever deals in cells, which avoids a whole class of
 * aspect-ratio bugs.
 *
 * Members
 *   cols   terminal width in cells. Used for bounds checks and right-aligning
 *          the top HUD line.
 *   rows   terminal height in cells. The key-hint line sits at row rows-1.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Build the top HUD line. When a dive is running over a non-PATROL base, it
 * shows "hawk:DIVE->HOVER" so you can see where it'll return; otherwise just
 * the current mode. The suffix shows PAUSED, AUTO, or nothing. */
static void format_hud_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    const char *live_mode = HAWK_MODE_NAMES[sc->hawk.mode];
    const char *suffix    = sc->sim.paused      ? "  PAUSED"
                          : sc->hawk.auto_dive  ? "  AUTO"
                                                : "";
    bool dive_over_non_patrol = (sc->hawk.mode == HAWK_DIVE)
                              && (sc->hawk.base_mode != HAWK_PATROL);

    if (dive_over_non_patrol) {
        snprintf(buf, buflen,
                 " %5.1f fps  sim:%3d Hz  n:%d/%d  [%s]  hawk:%s->%s%s ",
                 fps, sim_fps, sc->n_birds, N_BOIDS_MAX,
                 THEMES[sc->sim.theme_idx].name,
                 live_mode, HAWK_MODE_NAMES[sc->hawk.base_mode], suffix);
    } else {
        snprintf(buf, buflen,
                 " %5.1f fps  sim:%3d Hz  n:%d/%d  [%s]  hawk:%s%s ",
                 fps, sim_fps, sc->n_birds, N_BOIDS_MAX,
                 THEMES[sc->sim.theme_idx].name, live_mode, suffix);
    }
}

/* Print one bold coloured string at (row, col). Shared by both HUD lines. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* The top-row status line (fps, mode, counts), right-aligned. */
static void draw_hud_status(const Screen *s, const Scene *sc,
                            double fps, int sim_fps)
{
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_COLS + 1];
    format_hud_status(sc, fps, sim_fps, buf, sizeof buf);

    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* The bottom-row strip of key bindings. */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:dive  n/p:mode  h:auto-dive  .:pause  r:reset  t/T:theme  +/-:birds ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* Compose one frame: clear, draw the scene, then the two HUD lines on top. */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App + signal handlers ──────────────────────────────────────────── */

/*
 * FpsCounter — a smoothed frame-rate readout.
 *
 * A raw per-frame fps number jumps around too much to read, so this tallies
 * frames and elapsed time over a half-second window and only updates the
 * displayed figure when the window fills.
 *
 * Members
 *   frame_count  frames counted so far in the current window.
 *   window_ns    nanoseconds counted so far in the current window.
 *   display      the latest smoothed fps the HUD shows.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

/* Count one frame; refresh the displayed fps when the window fills. */
static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display = (double)f->frame_count
               / ((double)f->window_ns / (double)NS_PER_SEC);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — everything the program keeps around, in one place.
 *
 * There's a single static `g_app`. Most code reaches state through an `App *`
 * argument; the signal handlers reach it through `&g_app` because a handler
 * can't be passed a pointer.
 *
 * Members
 *   scene        the simulation.
 *   screen       the terminal size.
 *   fps          the frame-rate readout.
 *   sim_fps      the target physics rate.
 *   running      cleared by a quit signal to end the loop.
 *   need_resize  set by a resize signal; handled at the top of the loop.
 *                (Both flags are sig_atomic_t because signal handlers set them.)
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc = &app->scene;

    /* Recompute the world size from the new terminal size. */
    sc->world = world_from_terminal(app->screen.cols, app->screen.rows);
    World world = sc->world;

    /* Pull any bird or the hawk that's now outside the smaller world back in. */
    for (int i = 0; i < sc->n_birds; i++) {
        Boid *b = &sc->pool[i];
        if (b->pos.x >= world.width)  b->pos.x = world.width  - 1.0f;
        if (b->pos.y >= world.height) b->pos.y = world.height - 1.0f;
        b->prev_pos = b->pos;
    }
    if (sc->hawk.pos.x >= world.width)  sc->hawk.pos.x = world.width  - 1.0f;
    if (sc->hawk.pos.y >= world.height) sc->hawk.pos.y = world.height - 1.0f;

    app->need_resize = 0;
}

/* Flip auto-dive on or off, resetting its timer. Resetting avoids a stray dive
 * firing right after you re-enable it from a half-elapsed timer. */
static void toggle_auto_dive(Hawk *h)
{
    h->auto_dive       = !h->auto_dive;
    h->auto_dive_timer = 0.0f;
}

/* Switch to the next (dir +1) or previous (-1) theme, wrapping around, and
 * load it so the change shows immediately. */
static void cycle_theme(SimControls *sim, int dir)
{
    sim->theme_idx = (sim->theme_idx + dir + N_THEMES) % N_THEMES;
    theme_apply(sim->theme_idx);
}

/* Add or remove birds, clamped to the min/max. The pool is already spawned, so
 * adding just reveals more of the existing birds. */
static void adjust_bird_count(Scene *sc, int delta)
{
    int next = sc->n_birds + delta;
    if (next < N_BOIDS_MIN) next = N_BOIDS_MIN;
    if (next > N_BOIDS_MAX) next = N_BOIDS_MAX;
    sc->n_birds = next;
}

/* Handle one keypress; return false only on quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':  hawk_dive(&sc->hawk, sc->flock_centroid);          break;

    case 'n':  hawk_cycle_base_mode(&sc->hawk, +1);               break;
    case 'p':  hawk_cycle_base_mode(&sc->hawk, -1);               break;

    case 'h': case 'H':  toggle_auto_dive(&sc->hawk);             break;

    case '.':            sc->sim.paused = !sc->sim.paused;        break;
    case 'r': case 'R':  scene_init(sc, app->screen.cols, app->screen.rows); break;
    case 't':            cycle_theme(&sc->sim, +1);               break;
    case 'T':            cycle_theme(&sc->sim, -1);               break;
    case '+': case '=':  adjust_bird_count(sc, +BOID_STEP);       break;
    case '-':            adjust_bird_count(sc, -BOID_STEP);       break;

    default: break;
    }
    return true;
}

/* Game loop: handle resizes, measure elapsed time, run fixed-size physics steps
 * until caught up, update the fps readout, sleep to hold the frame rate, draw,
 * then read input. */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    fps_counter_init(&app->fps);

    screen_init(&app->screen, 0 /* initial theme = Dusk */);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* DT_CAP_NS bounds one frame's time so a hiccup can't trigger a flood of
     * catch-up physics steps. */
    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* (1) handle a pending resize: reload sizes, reset timers */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) time since last frame, capped */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) run as many fixed-size physics steps as the elapsed time owes */
        const int64_t tick_ns = TICK_NS(app->sim_fps);
        const float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* (4) update the fps readout */
        fps_counter_tick(&app->fps, dt);

        /* (5) sleep before drawing so terminal output stays inside the budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* (6) draw */
        screen_draw(&app->screen, &app->scene, app->fps.display, app->sim_fps);
        screen_present();

        /* (7) read input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
