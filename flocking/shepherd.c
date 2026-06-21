/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shepherd.c — a border collie herds scattered sheep back into a pen.
 *
 * The dog never steers a sheep directly. It just stands on the far side
 * of a stray sheep, and the sheep — running away from the dog — happens
 * to run back toward the pen. The sheep are flocking particles; the dog
 * is a Strömbom-style controller (Strömbom et al. 2014, "Solving the
 * shepherding problem"). Sheep flocking follows Reynolds 1987.
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

/* ── §1 config — speeds, radii, force weights, pen + polygon shapes ── */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,

    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /* Colour pair IDs — see §3 color_init for the actual colour values. */
    PAIR_PEN_RING    = 1,   /* dashed pen boundary circle               */
    PAIR_SHEEP_CALM  = 2,   /* sheep grazing inside pen                 */
    PAIR_SHEEP_FLEE  = 3,   /* sheep panicking from dog or scatter      */
    PAIR_DOG         = 4,   /* the border collie                        */
    PAIR_HUD         = 8,   /* bright yellow — top status               */
    PAIR_HINT        = 9,   /* bright cyan   — bottom key hint          */

    SHEEP_MIN        = 5,
    SHEEP_MAX        = 80,
    SHEEP_DEFAULT    = 30,
    SHEEP_STEP       = 5,
};

/* The simulation runs in "pixel space": CELL_W x CELL_H invisible
 * sub-pixels per terminal cell. Terminal cells are taller than wide, so
 * splitting them this way keeps a circle from looking like an oval. */
#define CELL_W   8
#define CELL_H  16

/* Pen size as a fraction of the shorter screen dimension: roomy enough
 * for 30 sheep, with space for the dog to circle outside. */
#define PEN_RADIUS_FRAC  0.18f
/* Slack past the pen edge before a sheep counts as a stray worth
 * chasing — keeps a sheep right on the boundary from re-triggering the dog. */
#define PEN_TOLERANCE   16.0f

/*
 * The five pen shapes the u/v keys cycle through, in cycle order:
 * CIRCLE -> SQUARE -> TRIANGLE -> HEXAGON -> OCTAGON -> back to CIRCLE.
 * Each (except the circle) is a regular polygon. TRIANGLE is the hardest
 * to herd into (narrow corners trap sheep); the others are roomier. All
 * are convex, which is what lets the same inside/outside test handle them.
 */
typedef enum {
    COMPOUND_CIRCLE = 0,
    COMPOUND_SQUARE,
    COMPOUND_TRIANGLE,
    COMPOUND_HEXAGON,
    COMPOUND_OCTAGON,
    COMPOUND_COUNT
} CompoundShape;

/*
 * Recipe for one pen shape: how many corners it has and how it's turned.
 * From these two numbers the code computes each corner's position.
 *
 *   name                 short label shown in the HUD ("CIRCLE", ...).
 *   n_vertices           number of corners. 0 means "this is the circle"
 *                        (a special case with no corners); 3-8 otherwise.
 *   rotation_offset_rad  how far the shape is spun, in radians. Chosen
 *                        per shape so it sits upright on screen (square
 *                        flat, triangle/hexagon point-up, octagon edge-up).
 *
 * Polygon math follows O'Rourke, "Computational Geometry in C" §1.5.
 */
typedef struct {
    const char *name;
    int         n_vertices;
    float       rotation_offset_rad;
} CompoundDef;

/* The five shape recipes, indexed by CompoundShape. */
static const CompoundDef COMPOUND_DEFS[COMPOUND_COUNT] = {
    [COMPOUND_CIRCLE]   = { "CIRCLE",   0,  0.0f },
    [COMPOUND_SQUARE]   = { "SQUARE",   4,  (float)M_PI / 4.0f },
    [COMPOUND_TRIANGLE] = { "TRIANGLE", 3, -(float)M_PI / 2.0f },
    [COMPOUND_HEXAGON]  = { "HEXAGON",  6, -(float)M_PI / 2.0f },
    [COMPOUND_OCTAGON]  = { "OCTAGON",  8, -(float)M_PI / 2.0f + (float)M_PI / 8.0f },
};


/* Sheep speeds, in pixels per second.
 *   GRAZE — lazy milling speed when nothing is pushing them.
 *   FLEE  — how hard they run from the dog (a target, not a cap).
 *   MAX   — absolute speed cap. Above FLEE so panic still maxes out,
 *           below the dog's speed so the dog can get ahead of them. */
#define SHEEP_GRAZE_SPEED   20.0f
#define SHEEP_FLEE_SPEED   140.0f
#define SHEEP_MAX_SPEED    180.0f

/* How far each sheep can sense, in pixels.
 *   SEP — personal-space bubble; closer than this and sheep push apart.
 *   COH — how far a sheep looks to find flockmates to drift toward.
 *   DOG_FLEE — the dog's "scary aura"; sheep inside it run away. */
#define SHEEP_SEP_RADIUS    24.0f
#define SHEEP_COH_RADIUS    96.0f
#define DOG_FLEE_RADIUS    120.0f

/* How much each steering urge counts when they're blended together.
 * Fleeing the dog wins, so panic scatters instead of huddling. Keeping
 * apart is firm. Drifting together and the inward pen-pull are gentle —
 * the pen-pull is just a nudge so strays don't wander off forever. */
#define W_SHEEP_SEP    1.6f
#define W_SHEEP_COH    0.4f
#define W_SHEEP_FLEE   2.6f
#define W_PEN_PULL     0.3f

/* Drag applied each tick: a sheep with no forces on it slows and stops,
 * so it grazes in place instead of gliding forever. */
#define SHEEP_DAMPING  0.95f

/* The SPACE-bar panic: shove every sheep outward, plus a small random
 * angle per sheep so they fan out instead of firing along straight spokes. */
#define SCATTER_IMPULSE 220.0f
#define SCATTER_JITTER    0.6f

/* The dog.
 *   SPEED          — must beat the sheep's flee speed so it can get
 *                    ahead of a fleeing sheep instead of chasing its tail.
 *   APPROACH_OFFSET— how far past a stray the dog stands when herding it;
 *                    close enough to scare it, far enough not to overwhelm.
 *   PATROL_OFFSET  — gap between the pen edge and the dog's idle circuit.
 *   PATROL_OMEGA   — how fast it circles when idle (rad/s); ~12 s a lap. */
#define DOG_SPEED              180.0f
#define DOG_APPROACH_OFFSET     60.0f
#define DOG_PATROL_OFFSET       40.0f
#define DOG_PATROL_OMEGA         0.5f

/* Chaos mode ('c'): a tiny random shove to every sheep each tick so the
 * herd never quite settles and the dog stays busy. */
#define RANDOM_NUDGE  120.0f

/* Timing helpers: nanoseconds per second / millisecond, and the length
 * of one tick at f ticks per second. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer and sleep ── */

/* Current time in nanoseconds from a clock that only ever moves forward,
 * so the gap between two readings is never negative even if the system
 * clock is adjusted (NTP, daylight saving). */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for ns nanoseconds; a zero or negative request returns at once,
 * so the caller can pass a leftover time budget without checking it. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — pen / sheep / dog / HUD colour pairs ── */

/* The 256-colour palette. Background stays at the terminal default (-1)
 * so the demo blends with the user's theme; foregrounds are kept bright. */
static void palette_xterm256(void)
{
    init_pair(PAIR_PEN_RING,    220, -1);  /* warm yellow ring         */
    init_pair(PAIR_SHEEP_CALM,  255, -1);  /* near-white grazing sheep */
    init_pair(PAIR_SHEEP_FLEE,  196, -1);  /* pure red panicking sheep */
    init_pair(PAIR_DOG,          33, -1);  /* dodger blue border collie */
    init_pair(PAIR_HUD,         226, -1);  /* bright yellow — top bar  */
    init_pair(PAIR_HINT,         51, -1);  /* bright cyan   — key hint */
}

/* Fallback for terminals with only the 8 basic colours. */
static void palette_ansi8(void)
{
    init_pair(PAIR_PEN_RING,    COLOR_YELLOW, -1);
    init_pair(PAIR_SHEEP_CALM,  COLOR_WHITE,  -1);
    init_pair(PAIR_SHEEP_FLEE,  COLOR_RED,    -1);
    init_pair(PAIR_DOG,         COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,         COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,        COLOR_CYAN,   -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) palette_xterm256();
    else               palette_ansi8();
}

/* ── §4 coords — pixel <-> cell bridge and the Vec2 type ── */

/* Width/height of the terminal in pixels (cells x sub-pixels per cell). */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/* Snap a pixel coordinate to the nearest terminal cell. Adding 0.5 then
 * flooring rounds half-values up consistently; roundf() rounds half to
 * even, which can make a glyph flicker between two cells on the boundary. */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/*
 * A 2-D point or arrow with floating-point x and y, in pixel space.
 * Everything with an x and a y — a position, a velocity, a force — is a
 * Vec2. The v2* helpers below let force math read close to the math you'd
 * write on paper. It's a struct rather than two loose floats mainly so
 * functions can take and return whole vectors by name.
 *
 *   x, y   the two components, in pixels. Only the draw step turns these
 *          into terminal cells; the physics never deals in cells.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/* Unit-length version of v (same direction, length 1). Returns the zero
 * vector for a near-zero input so two points on top of each other don't
 * cause a divide-by-zero. */
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

/* Random float in [0, 1], used for spawn and scatter randomness. */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.5 world + compound — world size and shape-agnostic pen tests ── */

/*
 * The size of the playable box, in pixels. Sheep and the dog are kept
 * inside it; it's recomputed whenever the terminal is resized.
 *
 *   width, height   the box's dimensions in pixels. Both are positive.
 */
typedef struct {
    float width;
    float height;
} World;

/* Build a World from a terminal size. Called at startup and on every
 * resize so the physics box always matches the visible screen. */
static inline World world_from_terminal(int cols, int rows)
{
    return (World){ .width = pw(cols), .height = ph(rows) };
}

/*
 * The pen as it actually sits on screen: which shape, where, how big.
 * Every sheep, dog, and drawing routine takes one of these instead of
 * juggling three separate values.
 *
 *   shape           which of the five shapes is active.
 *   centre          the pen's centre point, in pixels.
 *   bounding_radius the pen's size in pixels — the radius for the circle,
 *                   the corner distance for a polygon. Always positive.
 *                   Also used as the dog's idle-circle radius.
 *
 * The dog herds sheep into this region. The math (Strömbom 2014) assumes
 * a convex target; all five shapes here are convex, so it just works.
 */
typedef struct {
    CompoundShape shape;
    Vec2          centre;
    float         bounding_radius;
} Compound;

/* Rescale a pixel position into a unit frame around the pen: the centre
 * becomes (0,0) and the pen edge becomes 1. Lets the polygon tests run on
 * fixed unit-circle corners regardless of the pen's actual size. */
static inline Vec2 compound_normalise(Vec2 pos, Vec2 centre, float scale)
{
    return v2((pos.x - centre.x) / scale, (pos.y - centre.y) / scale);
}

/* Position of corner k of a regular polygon, on the unit circle. The
 * corners are spread evenly around the circle starting from the shape's
 * rotation offset. */
static inline Vec2 polygon_vertex(const CompoundDef *def, int k)
{
    float angle = def->rotation_offset_rad
                + (float)k * 2.0f * (float)M_PI / (float)def->n_vertices;
    return v2(cosf(angle), sinf(angle));
}

/* Shortest distance from point p to the line segment a->b. Finds the
 * closest point on the segment (an endpoint, or a spot along it) and
 * returns the straight-line distance to it. */
static float point_to_segment_distance(Vec2 p, Vec2 a, Vec2 b)
{
    Vec2  ab          = v2sub(b, a);
    Vec2  ap          = v2sub(p, a);
    float ab_len_sq   = ab.x * ab.x + ab.y * ab.y;
    if (ab_len_sq < 1e-12f) return v2len(v2sub(p, a));      /* a and b coincide */
    float t = (ap.x * ab.x + ap.y * ab.y) / ab_len_sq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    Vec2 closest = v2add(a, v2scale(ab, t));
    return v2len(v2sub(p, closest));
}

/*
 * Is point pos inside the pen? For the circle it's a plain distance check.
 * For a polygon: a point is inside a convex shape only if it sits on the
 * same side of every edge — so we check each edge's turn direction and bail
 * out the moment one disagrees.
 */
static bool compound_is_inside(const Compound *c, Vec2 pos)
{
    const CompoundDef *def = &COMPOUND_DEFS[c->shape];

    /* circle: inside if closer to the centre than the radius */
    if (def->n_vertices == 0) {
        return v2len(v2sub(pos, c->centre)) < c->bounding_radius;
    }

    /* polygon: the sign of each edge's cross product tells which side the
     * point is on; one disagreement means it's outside. */
    Vec2 n = compound_normalise(pos, c->centre, c->bounding_radius);
    int  sign = 0;                     /* 0 = none seen yet, +1 / -1 = a side */
    for (int k = 0; k < def->n_vertices; k++) {
        Vec2 v0   = polygon_vertex(def, k);
        Vec2 v1   = polygon_vertex(def, (k + 1) % def->n_vertices);
        Vec2 edge = v2sub(v1, v0);
        Vec2 to_p = v2sub(n, v0);
        float cross = edge.x * to_p.y - edge.y * to_p.x;
        if      (cross > 1e-6f) { if (sign == -1) return false; sign = +1; }
        else if (cross < -1e-6f) { if (sign == +1) return false; sign = -1; }
    }
    return true;
}

/*
 * How far pos is outside the pen, in pixels — zero if it's inside.
 * For the circle, that's how far past the rim it sits; for a polygon,
 * the distance to the nearest edge. The dog uses this to pick which
 * stray sheep is worst.
 */
static float compound_distance_outside(const Compound *c, Vec2 pos)
{
    const CompoundDef *def = &COMPOUND_DEFS[c->shape];

    /* circle */
    if (def->n_vertices == 0) {
        float distance_from_centre = v2len(v2sub(pos, c->centre));
        return (distance_from_centre > c->bounding_radius)
             ? distance_from_centre - c->bounding_radius : 0.0f;
    }

    /* polygon: inside means zero; outside, the closest edge wins.
     * Distances run in the unit frame, then scale back to pixels. */
    if (compound_is_inside(c, pos)) return 0.0f;

    Vec2  n               = compound_normalise(pos, c->centre, c->bounding_radius);
    float best_norm_dist  = 1e9f;
    for (int k = 0; k < def->n_vertices; k++) {
        Vec2  v0 = polygon_vertex(def, k);
        Vec2  v1 = polygon_vertex(def, (k + 1) % def->n_vertices);
        float d  = point_to_segment_distance(n, v0, v1);
        if (d < best_norm_dist) best_norm_dist = d;
    }
    return best_norm_dist * c->bounding_radius;
}

/* The active shape's name, for the HUD. */
static inline const char *compound_name(const Compound *c)
{
    return COMPOUND_DEFS[c->shape].name;
}

/* Switch to the next (+1) or previous (-1) shape, wrapping around. */
static inline void compound_cycle_shape(Compound *c, int dir)
{
    int next = ((int)c->shape + dir + COMPOUND_COUNT) % COMPOUND_COUNT;
    c->shape = (CompoundShape)next;
}

/* ── §5 sheep — the herd and its four steering urges ── */

/*
 * One sheep: a moving dot with four urges blended each tick — keep apart
 * from close neighbours, drift toward the group, flee the dog, and a gentle
 * pull back inside if it's strayed out. There's no "line up with neighbours"
 * urge that boids have; real sheep huddle rather than stream, so it's left out.
 *
 *   pos       where it is now, in pixels. Always kept inside the world box;
 *             sheep can't run off-screen.
 *   prev_pos  where it was at the start of this tick. The renderer blends
 *             prev_pos -> pos for smooth motion between ticks.
 *   vel       velocity in pixels/second, capped at SHEEP_MAX_SPEED.
 *   fleeing   set each tick: true if the dog is close enough to scare it.
 *             Purely a drawing cue — the renderer uses it to pick the panic
 *             glyph and colour; the physics ignores it.
 *
 * Flocking follows Reynolds 1987; the huddle-under-threat behaviour matches
 * King et al. 2012.
 */
typedef struct {
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;
    bool fleeing;
} Sheep;

/*
 * Drop a sheep at a random spot inside the active pen shape. Throws darts at
 * a slightly-shrunk box around the pen and keeps the first one that lands
 * inside, so sheep start a little off the boundary. The try cap stops a
 * pathological shape from looping forever.
 */
static void sheep_spawn_in_pen(Sheep *s, const Compound *c)
{
    const float SPAWN_INSET_FRAC = 0.85f;     /* shrink so sheep start off the edge */
    const int   MAX_SPAWN_TRIES  = 256;
    const float r                = c->bounding_radius;

    Vec2 spawn_pos = c->centre;               /* used if every dart misses */
    for (int attempt = 0; attempt < MAX_SPAWN_TRIES; attempt++) {
        float dx = randf() * 2.0f - 1.0f;     /* uniform in [-1, 1] */
        float dy = randf() * 2.0f - 1.0f;
        Vec2 candidate = v2(c->centre.x + dx * r * SPAWN_INSET_FRAC,
                            c->centre.y + dy * r * SPAWN_INSET_FRAC);
        if (compound_is_inside(c, candidate)) {
            spawn_pos = candidate;
            break;
        }
    }

    s->pos      = spawn_pos;
    s->prev_pos = s->pos;

    /* a small nudge in a random direction so it isn't frozen on frame one */
    float ang = randf() * 2.0f * (float)M_PI;
    s->vel    = v2(cosf(ang) * SHEEP_GRAZE_SPEED * 0.5f,
                   sinf(ang) * SHEEP_GRAZE_SPEED * 0.5f);
    s->fleeing = false;
}

/* The four steering urges. Each returns a push as a Vec2; scene_tick
 * blends them with the W_* weights and turns the result into motion. */

/*
 * Push away from every neighbour that's crowding this sheep's personal
 * space. The push fades smoothly with distance — strongest when touching,
 * nothing at the edge of the bubble — so sheep settle into comfortable
 * spacing instead of bouncing off a hard wall.
 */
static Vec2 sheep_separate(const Sheep *flock, int n, int self)
{
    const float DIST_EPSILON = 0.001f;       /* guard against sitting exactly atop a neighbour */
    Vec2 force = v2(0, 0);
    Vec2 me    = flock[self].pos;

    for (int i = 0; i < n; i++) {
        if (i == self) continue;

        Vec2  away_offset = v2sub(me, flock[i].pos);   /* points from neighbour to me */
        float distance    = v2len(away_offset);

        bool inside_personal_space = distance < SHEEP_SEP_RADIUS;
        bool well_separated        = distance > DIST_EPSILON;
        if (!inside_personal_space || !well_separated) continue;

        /* closer crowding -> stronger push (1 when touching, 0 at the edge) */
        float personal_space_intrusion = (SHEEP_SEP_RADIUS - distance) / SHEEP_SEP_RADIUS;
        Vec2  push_dir     = v2norm(away_offset);
        Vec2  contribution = v2scale(push_dir,
                                     personal_space_intrusion * SHEEP_GRAZE_SPEED);
        force = v2add(force, contribution);
    }
    return force;
}

/*
 * Drift toward the average position of nearby flockmates — a gentle pull
 * that keeps the herd together. Returns nothing if no one's in range.
 */
static Vec2 sheep_cohere(const Sheep *flock, int n, int self)
{
    /* find the average position of neighbours in cohesion range */
    Vec2 position_sum  = v2(0, 0);
    int  n_neighbours  = 0;
    Vec2 me            = flock[self].pos;
    for (int i = 0; i < n; i++) {
        if (i == self) continue;
        bool inside_cohesion_radius = v2len(v2sub(flock[i].pos, me)) < SHEEP_COH_RADIUS;
        if (inside_cohesion_radius) {
            position_sum = v2add(position_sum, flock[i].pos);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* head toward that average at graze pace */
    Vec2 centroid_of_neighbours = v2scale(position_sum, 1.0f / (float)n_neighbours);
    Vec2 toward_centroid        = v2norm(v2sub(centroid_of_neighbours, me));
    return v2scale(toward_centroid, SHEEP_GRAZE_SPEED);
}

/*
 * Run away from the dog when it's close, harder the closer it is, nothing
 * once it's outside the scare radius. The caller checks whether this came
 * back non-zero to set the sheep's "panicking" drawing flag.
 */
static Vec2 sheep_flee_dog(Vec2 sheep_pos, Vec2 dog_pos)
{
    const float DIST_EPSILON = 0.001f;

    Vec2  away_from_dog    = v2sub(sheep_pos, dog_pos);   /* points away from the dog */
    float distance_to_dog  = v2len(away_from_dog);

    bool inside_panic_zone = distance_to_dog <= DOG_FLEE_RADIUS;
    bool well_separated    = distance_to_dog >  DIST_EPSILON;
    if (!inside_panic_zone || !well_separated) return v2(0, 0);

    /* closer dog -> stronger panic (1 right next to it, 0 at the edge) */
    float panic_intensity = (DOG_FLEE_RADIUS - distance_to_dog) / DOG_FLEE_RADIUS;
    Vec2  flee_dir        = v2norm(away_from_dog);
    return v2scale(flee_dir, panic_intensity * SHEEP_FLEE_SPEED);
}

/*
 * A faint tug back toward the pen centre, only when the sheep has strayed
 * out. Weak on purpose — it stops a stray drifting away forever, but never
 * fights the dog or pretends to route cleverly; the dog does the real work.
 */
static Vec2 sheep_pen_pull(Vec2 sheep_pos, const Compound *c)
{
    if (compound_is_inside(c, sheep_pos)) return v2(0, 0);

    Vec2 inward = v2sub(c->centre, sheep_pos);
    if (v2len(inward) < 0.001f) return v2(0, 0);
    return v2scale(v2norm(inward), SHEEP_GRAZE_SPEED);
}

/* ── §6 dog — the Strömbom-style shepherd controller ── */

/*
 * What the dog is doing right now. The whole controller is one yes/no
 * question per tick: is any sheep too far out?
 *
 *   DOG_PATROL   no — idle, circling the pen waiting for trouble.
 *   DOG_COLLECT  yes — herding the single worst stray back in.
 *
 * That's the entire trick (Strömbom et al. 2014): in COLLECT the dog just
 * stands on the far side of the stray, and the sheep fleeing it runs home.
 */
typedef enum {
    DOG_PATROL = 0,
    DOG_COLLECT,
} DogMode;

/*
 * The one border collie. Each tick its controller picks a spot to head for
 * and it walks there at a steady speed. The mode, the sheep it's chasing,
 * and the patrol angle are also shown in the HUD, but only the target spot
 * actually drives its movement.
 *
 *   pos       where it is now, in pixels. The sheep flee from this.
 *   prev_pos  where it was at tick start, for smooth drawing.
 *   vel       velocity, pixels/second. Always either still or moving at
 *             full DOG_SPEED -- there is no acceleration, it just snaps to
 *             face its target.
 *   target    the spot it is walking toward this tick.
 *   mode      PATROL or COLLECT (HUD display only).
 *   target_sheep  which sheep it is herding in COLLECT, or -1 in PATROL.
 *   patrol_phase  the angle around its idle circle, in radians. Stored as
 *             an angle (not a point) so the circle stays exact and survives
 *             a resize -- the position is recomputed from it each tick.
 *
 * Movement follows Stroembom 2014; the offset-pursuit idea is Reynolds 1999.
 */
typedef struct {
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;
    Vec2 target;       /* where the dog wants to be this tick */
    DogMode mode;
    int     target_sheep;   /* sheep index in COLLECT mode; -1 in PATROL */
    float   patrol_phase;   /* angle around the idle circle, radians     */
} Dog;

/*
 * The dog's brain. Each tick it finds the single sheep that has strayed
 * the furthest outside the pen. If that sheep is meaningfully out, it
 * COLLECTs: stand a little beyond the stray, on the line from the pen
 * centre out through it, so the stray fleeing the dog runs back inward.
 * Otherwise everyone's home, so it PATROLs: walk slowly around the pen.
 * Only reads the sheep; it doesn't move them.
 */
static void dog_decide_target(Dog *d, const Sheep *flock, int n,
                              const Compound *c, float dt)
{
    /* find the sheep that has strayed furthest past the pen edge */
    float worst_outside_dist = 0.0f;
    int   worst_i            = -1;
    for (int i = 0; i < n; i++) {
        float outside_dist = compound_distance_outside(c, flock[i].pos);
        if (outside_dist > worst_outside_dist) {
            worst_outside_dist = outside_dist;
            worst_i            = i;
        }
    }

    /* COLLECT: stand just past the stray, on the centre-through-sheep
     * line, so the sheep fleeing the dog runs back toward the pen. */
    if (worst_i >= 0 && worst_outside_dist > PEN_TOLERANCE) {
        d->mode         = DOG_COLLECT;
        d->target_sheep = worst_i;

        Vec2 centre_to_sheep = v2sub(flock[worst_i].pos, c->centre);
        Vec2 outward_dir     = v2norm(centre_to_sheep);
        Vec2 approach_offset = v2scale(outward_dir, DOG_APPROACH_OFFSET);
        d->target            = v2add(flock[worst_i].pos, approach_offset);
        return;
    }

    /* PATROL: everyone's home, so circle the pen waiting for the next stray */
    d->mode         = DOG_PATROL;
    d->target_sheep = -1;
    d->patrol_phase += DOG_PATROL_OMEGA * dt;

    /* a point on the idle circle at the current patrol angle */
    float orbit_radius      = c->bounding_radius + DOG_PATROL_OFFSET;
    Vec2  patrol_unit_dir   = v2(cosf(d->patrol_phase), sinf(d->patrol_phase));
    Vec2  orbit_offset      = v2scale(patrol_unit_dir, orbit_radius);
    d->target               = v2add(c->centre, orbit_offset);
}

/*
 * Walk the dog one tick straight toward its target at full speed. No
 * acceleration or momentum — it just turns to face the target and goes.
 * Smooth enough because the target itself moves smoothly.
 */
static void dog_step(Dog *d, float dt)
{
    const float ARRIVED_EPSILON = 0.001f;

    /* aim at the target; stop if we're already on it */
    Vec2  to_target          = v2sub(d->target, d->pos);
    float distance_to_target = v2len(to_target);

    Vec2 desired_velocity;
    if (distance_to_target > ARRIVED_EPSILON) {
        Vec2 toward_target_dir = v2norm(to_target);
        desired_velocity       = v2scale(toward_target_dir, DOG_SPEED);
    } else {
        desired_velocity       = v2(0, 0);
    }

    /* save the old spot for smooth drawing, then move */
    d->vel      = desired_velocity;
    d->prev_pos = d->pos;
    d->pos      = v2add(d->pos, v2scale(d->vel, dt));
}

/* ── §7 scene — owns all simulation state; runs ticks and draws ── */

/*
 * The two playback toggles.
 *   paused           true freezes the sim; HUD shows "PAUSED" (p key).
 *   continuous_chaos true keeps nudging the sheep every tick so they
 *                    never settle; HUD shows "CHAOS" (c key).
 */
typedef struct {
    bool paused;
    bool continuous_chaos;
} SimControls;

/*
 * Holds everything the simulation needs for one run: the sheep, the dog,
 * the pen, the world size, the playback toggles. Every helper takes a
 * Scene*, so this is the one place to find "what state exists". (The
 * terminal size and signal flags live elsewhere, on Screen and App.)
 *
 *   sheep / n_sheep  the pool of sheep; only the first n_sheep are active,
 *                    the rest pre-spawned so '+' can reveal them instantly.
 *   dog              the single shepherd.
 *   sim              the paused / chaos toggles.
 *   compound         the pen: shape, centre, size.
 *   world            the play area size in pixels; refreshed on resize.
 *   in_pen_count     how many sheep are currently inside, for the HUD;
 *                    recomputed every tick.
 */
typedef struct {
    Sheep sheep[SHEEP_MAX];
    int   n_sheep;
    Dog   dog;
    SimControls sim;
    Compound compound;
    World world;
    int in_pen_count;       /* sheep currently inside the pen (HUD) */
} Scene;

/*
 * Start (or reset) the scene: sheep scattered inside the pen, dog parked
 * just outside it. The pen sits at screen centre, sized from the shorter
 * screen dimension so it stays square. The chosen shape survives a reset
 * ('r' regathers into whatever shape you'd picked).
 */
static void scene_init(Scene *s, int cols, int rows)
{
    /* keep the chosen shape, wipe everything else */
    CompoundShape saved_shape = s->compound.shape;
    memset(s, 0, sizeof *s);
    s->compound.shape = saved_shape;

    /* world size from the terminal */
    s->world   = world_from_terminal(cols, rows);
    s->n_sheep = SHEEP_DEFAULT;

    /* pen at screen centre, sized off the shorter dimension to stay square */
    float min_dim = (s->world.width < s->world.height) ? s->world.width
                                                       : s->world.height;
    s->compound.centre          = v2(s->world.width  * 0.5f,
                                      s->world.height * 0.5f);
    s->compound.bounding_radius = min_dim * PEN_RADIUS_FRAC;

    /* spawn the whole pool now so '+' reveals ready-placed sheep instantly */
    for (int i = 0; i < SHEEP_MAX; i++)
        sheep_spawn_in_pen(&s->sheep[i], &s->compound);

    /* dog starts at the right edge of its idle circle */
    s->dog.pos          = v2(s->compound.centre.x
                              + s->compound.bounding_radius + DOG_PATROL_OFFSET,
                             s->compound.centre.y);
    s->dog.prev_pos     = s->dog.pos;
    s->dog.target       = s->dog.pos;
    s->dog.vel          = v2(0, 0);
    s->dog.mode         = DOG_PATROL;
    s->dog.target_sheep = -1;
    s->dog.patrol_phase = 0.0f;
}

/*
 * The panic button: shove every sheep outward from the pen centre, each at
 * a slightly randomised angle so the herd fans out instead of firing along
 * straight spokes. strength_mul is 1 for SPACE, 2 for the 'S' mega-scatter.
 */
static void scene_scatter(Scene *s, float strength_mul)
{
    const float DIST_EPSILON = 0.001f;

    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh = &s->sheep[i];

        /* point away from the centre; a sheep sitting exactly on the
         * centre has no outward direction, so pick a random one */
        Vec2  centre_to_sheep    = v2sub(sh->pos, s->compound.centre);
        float distance_to_centre = v2len(centre_to_sheep);
        Vec2  outward_dir;
        if (distance_to_centre > DIST_EPSILON) {
            outward_dir = v2norm(centre_to_sheep);
        } else {
            float random_heading_rad = randf() * 2.0f * (float)M_PI;
            outward_dir = v2(cosf(random_heading_rad), sinf(random_heading_rad));
        }

        /* Step 2: rotate outward_dir by a per-sheep random angle in
         * [−SCATTER_JITTER, +SCATTER_JITTER] using the 2-D rotation
         * matrix [[cos −sin], [sin cos]].  Without the jitter, sheep
         * would fly along radial spokes from the centre — the rotation
         * fans them out into a more natural panic spread. */
        float jitter_rad         = (randf() * 2.0f - 1.0f) * SCATTER_JITTER;
        float cos_j              = cosf(jitter_rad);
        float sin_j              = sinf(jitter_rad);
        Vec2  jittered_dir       = v2(outward_dir.x * cos_j - outward_dir.y * sin_j,
                                      outward_dir.x * sin_j + outward_dir.y * cos_j);

        /* Step 3: add SCATTER_IMPULSE · strength_mul · jittered_dir to vel.
         * strength_mul is 1.0 for SPACE, 2.0 for mega-scatter 'S'. */
        Vec2 impulse = v2scale(jittered_dir, SCATTER_IMPULSE * strength_mul);
        sh->vel      = v2add(sh->vel, impulse);
    }
}

/*
 * Chaos mode: give every sheep a tiny random shove this tick. One tick is
 * invisible, but it adds up so the herd never quite settles and the dog
 * stays busy herding — the whole point of the mode.
 */
static void scene_apply_chaos(Scene *s, float dt)
{
    for (int i = 0; i < s->n_sheep; i++) {
        float ang = randf() * 2.0f * (float)M_PI;
        float mag = RANDOM_NUDGE * dt;
        s->sheep[i].vel = v2add(s->sheep[i].vel,
                                v2(cosf(ang) * mag, sinf(ang) * mag));
    }
}

/*
 * Blend one sheep's four urges into a single push, weighting each by its
 * W_* constant. Also sets the sheep's "panicking" flag — the only place
 * that flag is written. Reads the old positions; the caller moves the sheep.
 */
static Vec2 sheep_total_force(Scene *s, int idx)
{
    const float FLEE_NONZERO_EPSILON = 0.001f;
    Sheep *sh = &s->sheep[idx];

    Vec2 sep_force  = sheep_separate (s->sheep, s->n_sheep, idx);
    Vec2 coh_force  = sheep_cohere   (s->sheep, s->n_sheep, idx);
    Vec2 flee_force = sheep_flee_dog (sh->pos, s->dog.pos);
    Vec2 pen_force  = sheep_pen_pull (sh->pos, &s->compound);

    /* panicking if the dog's flee force is non-zero */
    sh->fleeing = v2len(flee_force) > FLEE_NONZERO_EPSILON;

    /* weighted blend of the four urges */
    Vec2 force = v2(0, 0);
    force = v2add(force, v2scale(sep_force,  W_SHEEP_SEP));
    force = v2add(force, v2scale(coh_force,  W_SHEEP_COH));
    force = v2add(force, v2scale(flee_force, W_SHEEP_FLEE));
    force = v2add(force, v2scale(pen_force,  W_PEN_PULL));
    return force;
}

/*
 * Turn one sheep's force into a new velocity: add the push, apply drag (so
 * it can slow to a stop), and cap the top speed. Returns the new velocity;
 * the caller stores it.
 */
static Vec2 sheep_integrate_kinematics(Vec2 vel, Vec2 force, float dt)
{
    Vec2 accumulated_vel = v2add(vel, v2scale(force, dt));
    Vec2 damped_vel      = v2scale(accumulated_vel, SHEEP_DAMPING);
    Vec2 capped_vel      = v2clamp_len(damped_vel, SHEEP_MAX_SPEED);
    return capped_vel;
}

/*
 * Keep a position on screen. The world edge is a solid wall, not a wrap-
 * around — nothing ever runs off the side.
 */
static void clamp_pos_to_world(Vec2 *pos, World world)
{
    if (pos->x <  0.0f)         pos->x = 0.0f;
    if (pos->x >= world.width)  pos->x = world.width  - 1.0f;
    if (pos->y <  0.0f)         pos->y = 0.0f;
    if (pos->y >= world.height) pos->y = world.height - 1.0f;
}

static void scene_step_sheep(Scene *s, float dt)
{
    /* Two passes on purpose. First, compute every sheep's new velocity
     * from the current positions — so they all react to the same snapshot,
     * not to neighbours that already moved this tick. */
    Vec2 new_vel[SHEEP_MAX];
    for (int i = 0; i < s->n_sheep; i++) {
        Vec2 force = sheep_total_force(s, i);
        new_vel[i] = sheep_integrate_kinematics(s->sheep[i].vel, force, dt);
    }

    /* Then move everyone, keep them on screen, and tally who's in the pen. */
    int in_pen = 0;
    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh    = &s->sheep[i];
        sh->vel      = new_vel[i];
        sh->prev_pos = sh->pos;
        sh->pos      = v2add(sh->pos, v2scale(sh->vel, dt));
        clamp_pos_to_world(&sh->pos, s->world);

        if (compound_is_inside(&s->compound, sh->pos)) in_pen++;
    }
    s->in_pen_count = in_pen;
}

/*
 * Decide where the dog should go, move it there, keep it on screen. The
 * final clamp guards the rare case where one full-speed step would carry
 * it off the edge while chasing a sheep into a corner.
 */
static void scene_step_dog(Scene *s, float dt)
{
    dog_decide_target(&s->dog, s->sheep, s->n_sheep, &s->compound, dt);
    dog_step(&s->dog, dt);
    clamp_pos_to_world(&s->dog.pos, s->world);
}

/*
 * One physics step. The dog moves before the sheep so they flee from where
 * it actually is now, not where it was last tick.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    if (s->sim.continuous_chaos) scene_apply_chaos(s, dt);
    scene_step_dog  (s, dt);
    scene_step_sheep(s, dt);
}

/* ── §7 render — pen outline, then sheep, then dog on top ── */

/*
 * Draw one character at a terminal cell, clipping anything off-screen.
 * The double cast keeps high-bit characters from being sign-extended into
 * garbage — a standard ncurses gotcha.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * Draw the circular pen as a dashed ring of '*' and '.'. Walks evenly
 * around the circle in pixel space (not cells) so it stays round instead
 * of squashing into an oval. The number of dots scales with the radius.
 */
static void draw_pen(WINDOW *w, Vec2 c, float r, int cols, int rows)
{
    int n = (int)ceilf(2.0f * (float)M_PI * r / 4.0f);
    if (n < 16) n = 16;
    for (int i = 0; i < n; i++) {
        float a   = (float)i / (float)n * 2.0f * (float)M_PI;
        float px  = c.x + r * cosf(a);
        float py  = c.y + r * sinf(a);
        char  ch  = (i & 1) ? '.' : '*';   /* alternate for the dashed look */
        mark_cell(w, px_to_cell_x(px), px_to_cell_y(py),
                  ch, PAIR_PEN_RING, A_BOLD, cols, rows);
    }
}

/*
 * Draw a polygon pen's outline. Scans the cells around the pen and paints
 * any cell that sits on the edge — meaning at least one of its four
 * neighbours is on the other side of the boundary. Dashed like draw_pen.
 */
static void draw_compound(WINDOW *w, const Compound *cmp,
                          int cols, int rows)
{
    /* the cell range to scan, padded a little so we don't clip the edge */
    enum { BB_MARGIN_CELLS = 2 };
    const float r = cmp->bounding_radius;
    const Vec2  c = cmp->centre;
    int cx_min = px_to_cell_x(c.x - r) - BB_MARGIN_CELLS;
    int cx_max = px_to_cell_x(c.x + r) + BB_MARGIN_CELLS;
    int cy_min = px_to_cell_y(c.y - r) - BB_MARGIN_CELLS;
    int cy_max = px_to_cell_y(c.y + r) + BB_MARGIN_CELLS;
    if (cx_min < 0)         cx_min = 0;
    if (cy_min < 0)         cy_min = 0;
    if (cx_max > cols - 1)  cx_max = cols - 1;
    if (cy_max > rows - 1)  cy_max = rows - 1;

    /* test each cell at its centre point */
    int dash_step = 0;
    for (int cy = cy_min; cy <= cy_max; cy++) {
        for (int cx = cx_min; cx <= cx_max; cx++) {
            Vec2 here     = v2(cx * (float)CELL_W + CELL_W * 0.5f,
                               cy * (float)CELL_H + CELL_H * 0.5f);
            bool here_in  = compound_is_inside(cmp, here);

            /* edge cell if any neighbour is on the other side */
            static const int NB_DX[4] = {-1, +1,  0,  0};
            static const int NB_DY[4] = { 0,  0, -1, +1};
            bool is_boundary = false;
            for (int k = 0; k < 4 && !is_boundary; k++) {
                Vec2 nb = v2(here.x + NB_DX[k] * (float)CELL_W,
                             here.y + NB_DY[k] * (float)CELL_H);
                if (compound_is_inside(cmp, nb) != here_in)
                    is_boundary = true;
            }

            if (is_boundary) {
                char ch = (dash_step++ & 1) ? '.' : '*';
                mark_cell(w, cx, cy, ch, PAIR_PEN_RING, A_BOLD, cols, rows);
            }
        }
    }
}

/* Draw the pen: the round ring for a circle, the cell-scan for a polygon. */
static void draw_pen_or_compound(WINDOW *w, const Compound *cmp,
                                 int cols, int rows)
{
    if (cmp->shape == COMPOUND_CIRCLE)
        draw_pen(w, cmp->centre, cmp->bounding_radius, cols, rows);
    else
        draw_compound(w, cmp, cols, rows);
}

/*
 * Paint one frame back-to-front: pen outline, then the sheep ('o' calm,
 * 'O' panicking), then the dog ('&') on top. alpha is the fraction of a
 * tick we're between physics steps; blending prev_pos toward pos by it
 * keeps motion smooth even when drawing faster than the sim runs.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* pen outline */
    draw_pen_or_compound(w, &s->compound, cols, rows);

    /* sheep, drawn at their blended in-between position */
    for (int i = 0; i < s->n_sheep; i++) {
        const Sheep *sh = &s->sheep[i];
        Vec2 dp = v2add(sh->prev_pos,
                        v2scale(v2sub(sh->pos, sh->prev_pos), alpha));

        char   ch   = sh->fleeing ? 'O' : 'o';
        int    pair = sh->fleeing ? PAIR_SHEEP_FLEE : PAIR_SHEEP_CALM;
        attr_t attr = sh->fleeing ? A_BOLD : A_NORMAL;
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  ch, pair, attr, cols, rows);
    }

    /* dog last so it sits above the sheep */
    {
        const Dog *d = &s->dog;
        Vec2 dp = v2add(d->prev_pos,
                        v2scale(v2sub(d->pos, d->prev_pos), alpha));
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  '&', PAIR_DOG, A_BOLD, cols, rows);
    }
}

/* ── §8 app — terminal, signals, and the main loop ── */

/*
 * The terminal's size in cells. Scene.world is the same box measured in
 * pixels; this is the cell-grid version that ncurses actually paints onto.
 * Keeping the two apart is what stops the physics from ever drawing in
 * cells (which would squash circles).
 *
 *   cols, rows   terminal width and height in cells. Both positive. The
 *                bottom row is rows-1, where the key hint goes.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);    /* don't block waiting for a key */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* stop input checks from tearing the drawing */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();                 /* make ncurses re-read the new terminal size */
    getmaxyx(stdscr, s->rows, s->cols);
}

static const char *DOG_MODE_NAMES[] = { "PATROL ", "COLLECT" };

/* Build the top status line: fps, sim rate, sheep in/total, pen shape,
 * dog mode, and a PAUSED/CHAOS tag. */
static void format_hud_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    const char *mode_suffix = sc->sim.paused           ? "  PAUSED"
                            : sc->sim.continuous_chaos ? "  CHAOS"
                                                       : "";
    snprintf(buf, buflen,
             " %5.1f fps  sim:%3d Hz  sheep:%d/%d in [%s]  dog:%s%s ",
             fps, sim_fps,
             sc->in_pen_count, sc->n_sheep,
             compound_name(&sc->compound),
             DOG_MODE_NAMES[sc->dog.mode],
             mode_suffix);
}

/* Print text in a colour pair, bold, at one spot — shared by both HUD rows. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Right-align the status line on the top row. */
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

/* The key-binding strip along the bottom row. */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:scatter  S:mega-scatter  u/v:compound "
        "(CIRCLE->SQUARE->TRIANGLE->HEXAGON->OCTAGON)  c:chaos  "
        "p:pause  r:reset  +/-:sheep ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);
    /* HUD last so it sits on top of the animation */
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── app state, signal handlers, resize, input, main loop ── */

/*
 * Smooths the frame-rate reading. A per-frame number jumps around too much
 * to read, so this counts frames over a half-second window and only updates
 * the displayed figure when the window fills.
 *
 *   frame_count  frames seen so far this window.
 *   window_ns    time elapsed in this window, in nanoseconds.
 *   display      the last smoothed fps, shown in the HUD.
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

/* Call once per frame; refreshes the displayed fps when the window fills. */
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
 * Everything the program holds for its whole run: the simulation, the
 * terminal, the fps counter, the physics rate, and two flags the signal
 * handlers flip. It's a single file-scope g_app because signal handlers
 * can't be passed a pointer — they reach the flags through that global.
 *
 *   running      cleared when a quit signal arrives; ends the loop.
 *   need_resize  set when the terminal is resized; handled next frame.
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

/*
 * Handle a terminal resize: re-read the new size, rebuild the world box and
 * the pen for it, and pull any sheep or the dog back inside the new bounds.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc = &app->scene;

    /* new world size from the new terminal */
    sc->world = world_from_terminal(app->screen.cols, app->screen.rows);

    /* re-centre and re-size the pen (shape is unchanged) */
    float min_dim = (sc->world.width < sc->world.height) ? sc->world.width
                                                         : sc->world.height;
    sc->compound.centre          = v2(sc->world.width  * 0.5f,
                                       sc->world.height * 0.5f);
    sc->compound.bounding_radius = min_dim * PEN_RADIUS_FRAC;

    /* pull anything now off-screen back inside */
    World world = sc->world;
    for (int i = 0; i < sc->n_sheep; i++) {
        Sheep *sh = &sc->sheep[i];
        if (sh->pos.x >= world.width)  sh->pos.x = world.width  - 1.0f;
        if (sh->pos.y >= world.height) sh->pos.y = world.height - 1.0f;
        sh->prev_pos = sh->pos;
    }
    if (sc->dog.pos.x >= world.width)  sc->dog.pos.x = world.width  - 1.0f;
    if (sc->dog.pos.y >= world.height) sc->dog.pos.y = world.height - 1.0f;
    sc->dog.prev_pos = sc->dog.pos;

    app->need_resize = 0;
}

/* Run one keypress. Returns false on quit (q / Q / ESC); see the bottom
 * HUD strip for the full key list. */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':  scene_scatter(sc, 1.0f);                 break;
    case 'S':  scene_scatter(sc, 2.0f);                 break;
    /* sheep stay put; the dog re-herds whoever the new shape left outside */
    case 'u':  compound_cycle_shape(&sc->compound, +1); break;
    case 'v':  compound_cycle_shape(&sc->compound, -1); break;
    case 'c': case 'C': sc->sim.continuous_chaos = !sc->sim.continuous_chaos; break;
    case 'p': case 'P': sc->sim.paused           = !sc->sim.paused;           break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        sc->n_sheep += SHEEP_STEP;
        if (sc->n_sheep > SHEEP_MAX) sc->n_sheep = SHEEP_MAX;
        break;
    case '-':
        sc->n_sheep -= SHEEP_STEP;
        if (sc->n_sheep < SHEEP_MIN) sc->n_sheep = SHEEP_MIN;
        break;
    default: break;
    }
    return true;
}

/*
 * The game loop. Each frame: measure elapsed time, run as many fixed-size
 * physics steps as that time allows, sleep off the rest of the frame
 * budget, then draw and read input. The physics runs at a fixed rate no
 * matter how fast or slow the drawing is.
 */
int main(void)
{
    /* seed randomness, install signal handlers and the cleanup hook */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* bring up the terminal and the simulation */
    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* timing constants for the loop */
    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;             /* cap a long stall so physics doesn't avalanche */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;     /* time allotted per frame */
    const int64_t TICK_LEN_NS     = TICK_NS(app->sim_fps);
    const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* deal with a resize that happened since last frame */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* how long since the last frame, capped after a long stall */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* run whole physics steps until we've caught up to real time */
        sim_accum += dt;
        while (sim_accum >= TICK_LEN_NS) {
            scene_tick(&app->scene, TICK_LEN_SEC);
            sim_accum -= TICK_LEN_NS;
        }

        /* leftover fraction of a step, for smooth drawing */
        float alpha = (float)sim_accum / (float)TICK_LEN_NS;

        /* update the fps reading */
        fps_counter_tick(&app->fps, dt);

        /* sleep off the rest of the budget BEFORE drawing, so terminal
         * writes don't push us over the frame time */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* draw the frame */
        screen_draw(&app->screen, &app->scene,
                    app->fps.display, app->sim_fps, alpha);
        screen_present();

        /* read one key */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
