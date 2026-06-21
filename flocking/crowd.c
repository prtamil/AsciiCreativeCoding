/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * crowd.c — six switchable steering-behaviour crowds in ASCII.
 *
 * ASCII "people" move under one of six modes (wander, flock, panic, gather,
 * follow, queue), switchable live with keys 1-6. Companion: flocking/flocking.c
 * (the boids rules alone). Based on Reynolds, "Steering Behaviors for
 * Autonomous Characters" (GDC 1999, red3d.com/cwr/steer/) and his 1987 boids
 * paper.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable constants (speeds, radii, weights, layout) ── */

/*
 * Sim and display rates. SIM_FPS_DEFAULT is physics steps per second
 * (higher = more accurate, more CPU); TARGET_FPS is the render cap.
 */
enum {
    SIM_FPS_DEFAULT =  60,
    TARGET_FPS      =  60,

    HUD_COLS        =  72,   /* max chars in the status-bar string */
    FPS_UPDATE_MS   = 500,   /* how often the displayed fps refreshes */

    /* Colour-pair IDs. 1-7 paint agents (rainbow); HUD/HINT paint the
     * two status lines and stay readable against any backdrop. */
    N_COLORS        =   7,   /* agent palette: pairs 1-7 */
    PAIR_HUD        =   8,   /* bright yellow — top status bar  */
    PAIR_HINT       =   9,   /* bright cyan   — bottom key hint */

    CROWD_MIN       =   5,   /* fewest people allowed */
    CROWD_MAX       = 150,   /* pool size; also the hard cap */
    CROWD_DEFAULT   =  60,   /* people on startup */
    CROWD_STEP      =   5,   /* how many +/- adds or removes */
};

/*
 * Cell size in pixels. A terminal cell is roughly 8 wide x 16 tall.
 * Positions are kept in pixels so equal speed in X and Y looks equal
 * on screen. Change only for a terminal with a different cell ratio.
 */
#define CELL_W   8    /* pixels per terminal column */
#define CELL_H  16    /* pixels per terminal row    */

/*
 * How close a neighbour must be to matter (pixels).
 * SEP_RADIUS      — personal-space bubble; closer people get pushed apart.
 * ALIGN_RADIUS    — flock only: match the heading of people inside this.
 * COHESION_RADIUS — flock only: drift toward the average position inside this.
 *                   Kept larger than ALIGN_RADIUS so cohesion pulls strays back.
 */
#define SEP_RADIUS       40.0f
#define ALIGN_RADIUS     80.0f
#define COHESION_RADIUS 120.0f

/*
 * Speeds (pixels per second).
 * SPEED_BASE   — relaxed cruising speed (wander/flock/gather/follow).
 * SPEED_PANIC  — flee speed; higher feels more frantic.
 * SPEED_MAX    — hard cap so stacked forces can't run anyone away.
 * THREAT_SPEED — panic threat's speed; slower than people so they can escape.
 */
#define SPEED_BASE    80.0f
#define SPEED_PANIC  140.0f
#define SPEED_MAX    160.0f
#define THREAT_SPEED  55.0f

/*
 * How much each steering rule counts in the final blended force.
 * Bigger weight = that rule wins. Separation is highest so people never overlap.
 */
#define W_SEEK    1.0f   /* seek / flee strength                        */
#define W_FLEE    1.6f   /* flee higher so panic feels urgent           */
#define W_SEP     1.8f   /* separation highest — no overlap             */
#define W_ALIGN   1.0f   /* flock alignment                             */
#define W_COHERE  0.8f   /* flock cohesion (softer than alignment)      */

/*
 * Distances at which people decide they've "arrived" or should slow down.
 * ARRIVE_DIST        — within this of a target, count as arrived (pick a new one).
 * GATHER_SLOW_RADIUS — GATHER slows here so people mill instead of overshooting.
 * QUEUE_SLOW_RADIUS  — same slow-down near a queue slot.
 * QUEUE_IDLE_SPEED   — shuffle speed once nearly in the slot.
 */
#define ARRIVE_DIST         24.0f
#define GATHER_SLOW_RADIUS  (ARRIVE_DIST * 4.0f)   /* 96 px, ~12 cells  */
#define QUEUE_SLOW_RADIUS   (ARRIVE_DIST * 2.0f)   /* 48 px, ~6 cells   */
#define QUEUE_IDLE_SPEED    (SPEED_BASE  * 0.3f)   /* 24 px/s, shuffle  */

/*
 * Queue spacing. Slots march leftward from the counter in a 3-row zigzag.
 * QUEUE_SLOT_W — gap between slots; QUEUE_SLOT_H — vertical zigzag step.
 */
#define QUEUE_SLOT_W  ((float)CELL_W  * 1.5f)
#define QUEUE_SLOT_H  ((float)CELL_H  * 1.5f)

/* Timing helpers */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer + sleep ── */

/*
 * Nanosecond timer. Uses the monotonic clock because it never jumps
 * backward (a wall clock can, on NTP adjustments), so dt stays sane.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for ns nanoseconds. Called before drawing so terminal I/O
 * doesn't eat the frame budget (see the §8 main loop). */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — agent + HUD colour pairs ── */

/* 256-colour palette. Background -1 keeps the terminal's own; every
 * foreground sits in the bright half so it stays visible. */
static void palette_xterm256(void)
{
    init_pair(1, 196, -1);   /* red     */
    init_pair(2, 208, -1);   /* orange  */
    init_pair(3, 226, -1);   /* yellow  */
    init_pair(4,  46, -1);   /* green   */
    init_pair(5,  51, -1);   /* cyan    */
    init_pair(6,  33, -1);   /* blue    */
    init_pair(7, 201, -1);   /* magenta */
    init_pair(PAIR_HUD,  226, -1);   /* bright yellow — top status bar  */
    init_pair(PAIR_HINT,  51, -1);   /* bright cyan   — bottom key hint */
}

/* Fallback for plain 8-colour terminals; nearest equivalents. */
static void palette_ansi8(void)
{
    init_pair(1, COLOR_RED,     -1);
    init_pair(2, COLOR_RED,     -1);
    init_pair(3, COLOR_YELLOW,  -1);
    init_pair(4, COLOR_GREEN,   -1);
    init_pair(5, COLOR_CYAN,    -1);
    init_pair(6, COLOR_BLUE,    -1);
    init_pair(7, COLOR_MAGENTA, -1);
    init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN,   -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) palette_xterm256();
    else               palette_ansi8();
}

/* ── §4 coords & vec2 — pixel/cell bridge, Vec2 math, wall helpers ── */

/* World size in pixels, from the terminal's cell count. */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * Pixel coordinate to terminal cell index. Adds 0.5 then floors to round
 * to the nearest cell; plain truncation would bias toward zero and make
 * motion look like a staircase.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * A pair of numbers with an x and a y. Used for everything that has a
 * direction or location: position, velocity, force, a target offset.
 * Passing one Vec2 instead of two loose floats keeps function signatures
 * readable, e.g. steer_seek(pos, vel, target, speed).
 *
 *   x, y   the two components, in pixels. Converted to a cell index
 *          with px_to_cell_x/y only when drawing.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/* Direction of v, length 1. Returns zero if v is ~zero, which dodges a
 * divide-by-zero when two people land on the same pixel. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* Shorten v if it's longer than max_len, keeping its direction.
 * Used to hold speed at SPEED_MAX. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* ── wall handlers: three ways to treat the world edges ── */

/* Wrap around: walk off one edge, reappear on the opposite one.
 * Used by WANDER, FLOCK, GATHER, FOLLOW. */
static void wrap_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x += ww;
    if (pos->x >= ww)  pos->x -= ww;
    if (pos->y <  0)   pos->y += wh;
    if (pos->y >= wh)  pos->y -= wh;
}

/* Bounce off the wall: hitting an edge flips that velocity component.
 * Used by PANIC so the crowd stays trapped on screen. */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)   { pos->x = 0;      vel->x =  fabsf(vel->x); }
    if (pos->x >= ww)  { pos->x = ww-1;   vel->x = -fabsf(vel->x); }
    if (pos->y <  0)   { pos->y = 0;      vel->y =  fabsf(vel->y); }
    if (pos->y >= wh)  { pos->y = wh-1;   vel->y = -fabsf(vel->y); }
}

/* Pin to the edge: stop dead at the wall. Used by QUEUE. */
static void clamp_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x = 0;
    if (pos->x >= ww)  pos->x = ww-1;
    if (pos->y <  0)   pos->y = 0;
    if (pos->y >= wh)  pos->y = wh-1;
}

/* ── §5 entity — Person, spawn, integration step ── */

typedef enum {
    BEH_WANDER = 0,
    BEH_FLOCK,
    BEH_PANIC,
    BEH_GATHER,
    BEH_FOLLOW,
    BEH_QUEUE,
    BEH_COUNT
} Behaviour;

static const char *BEH_NAMES[BEH_COUNT] = {
    "WANDER", "FLOCK", "PANIC", "GATHER", "FOLLOW", "QUEUE"
};

/* Characters drawn for people, handed out in turn by index. */
static const char GLYPHS[] = "oO0abcdefghijklmnpqrstuvwxyz";
#define N_GLYPHS ((int)(sizeof(GLYPHS) - 1))

/*
 * One person in the crowd (a "boid", in flocking terms). Each tick a
 * person adds up the steering forces its current behaviour cares about,
 * turns that into a velocity (capped), and moves. All six behaviours
 * share this same struct; they only differ in which forces they apply.
 *
 * The fields split into two groups that never touch each other: the
 * motion fields, rewritten by the physics every tick, and the look
 * fields, set once at spawn and only ever read by the drawing code.
 *
 *   ── motion (rewritten every tick) ──
 *   pos        where the person is now, in pixels.
 *   prev_pos   where they were at the start of this tick. Saved before
 *              moving so the renderer can draw a point between the two
 *              and keep motion smooth even when draw rate and sim rate
 *              differ (the "Fix Your Timestep!" interpolation trick).
 *   vel        velocity in pixels/second; held under SPEED_MAX (or
 *              SPEED_PANIC in panic mode). Mass is taken as 1, so a
 *              force is just added straight onto velocity.
 *   target     this person's destination, read by steer_seek. The
 *              active behaviour decides what it is: a random point in
 *              WANDER, the person ahead in FOLLOW, a slot in QUEUE.
 *
 *   ── look (set once at spawn) ──
 *   glyph      the character drawn for this person, so the eye can
 *              follow one through the crowd.
 *   color      colour-pair index, 1..N_COLORS.
 */
typedef struct {
    /* motion — rewritten every tick */
    Vec2  pos;        /* current position, pixels                        */
    Vec2  prev_pos;   /* position at start of this tick (for smoothing)  */
    Vec2  vel;        /* velocity, pixels per second                     */
    Vec2  target;     /* where this person is heading                    */

    /* look — set once at spawn */
    char  glyph;      /* character drawn for this person                 */
    int   color;      /* colour-pair index, 1 .. N_COLORS                */
} Person;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* Drop one person at a random spot with random velocity and target. */
static void person_spawn(Person *p, int id, float ww, float wh)
{
    p->pos      = v2(randf()*ww, randf()*wh);
    p->prev_pos = p->pos;
    p->vel      = v2((randf()-0.5f)*SPEED_BASE, (randf()-0.5f)*SPEED_BASE);
    p->target   = v2(randf()*ww, randf()*wh);
    p->glyph    = GLYPHS[id % N_GLYPHS];
    p->color    = (id % N_COLORS) + 1;
}

/*
 * Move a person one step: nudge velocity by the force, cap it, remember
 * where it was, then advance. The same step is shared by every behaviour.
 * Wall handling is left to the caller since each behaviour treats edges
 * differently (wrap / bounce / clamp).
 */
static void person_step(Person *p, Vec2 accel, float dt)
{
    /* push velocity by force*dt, then cap it at SPEED_MAX */
    Vec2 vel_delta = v2scale(accel, dt);
    Vec2 new_vel   = v2add(p->vel, vel_delta);
    p->vel = v2clamp_len(new_vel, SPEED_MAX);

    /* remember where we were before moving, so drawing can interpolate */
    p->prev_pos = p->pos;

    /* advance by velocity*dt */
    Vec2 pos_delta = v2scale(p->vel, dt);
    p->pos = v2add(p->pos, pos_delta);
}

/* True once the person is within ARRIVE_DIST of its target. */
static inline bool person_at_target(const Person *p)
{
    return v2len(v2sub(p->target, p->pos)) < ARRIVE_DIST;
}

/* Pick a fresh random destination, so an arrived person keeps moving. */
static inline void person_pick_random_target(Person *p, float ww, float wh)
{
    p->target = v2(randf()*ww, randf()*wh);
}

/* ── §6 steering — the five force functions, each returns a Vec2 ── */

/*
 * Force that steers toward target at the given speed. The trick is the
 * last line: we work out the velocity we WANT, then return the
 * difference from our current velocity. When we're already cruising
 * straight at the target, that difference is zero, so a person eases in
 * and slows down on arrival with no special-case code.
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    /* offset      : how far and which way the target is
     * toward_dir  : just the direction
     * desired_vel : the velocity we wish we had right now
     * (returned)  : what to add to current velocity to get there */
    Vec2 offset      = v2sub(target, pos);
    Vec2 toward_dir  = v2norm(offset);
    Vec2 desired_vel = v2scale(toward_dir, speed);
    return v2sub(desired_vel, vel);
}

/* Steer straight away from the threat: just seek, then flip the result. */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed)
{
    return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * Push away from everyone who has crept inside SEP_RADIUS. The closer a
 * neighbour is, the harder the push (it fades to nothing right at the
 * edge of the radius), which gives a soft "personal space" bubble rather
 * than a hard wall.
 */
static Vec2 steer_separate(const Person *people, int count, int self)
{
    const float DIST_EPSILON = 0.001f;       /* avoid divide-by-zero */
    Vec2 force = v2(0, 0);
    Vec2 pos   = people[self].pos;

    for (int i = 0; i < count; i++) {
        if (i == self) continue;

        /* direction away from this neighbour, and how far they are */
        Vec2  away_offset = v2sub(pos, people[i].pos);
        float distance    = v2len(away_offset);

        bool inside_personal_space = distance < SEP_RADIUS;
        bool well_separated        = distance > DIST_EPSILON;
        if (!inside_personal_space || !well_separated) continue;

        /* push strength: 1 when touching, fading to 0 at the radius edge */
        float personal_space_intrusion = (SEP_RADIUS - distance) / SEP_RADIUS;
        Vec2  push_dir = v2norm(away_offset);
        Vec2  contribution = v2scale(push_dir, personal_space_intrusion * SPEED_BASE);
        force = v2add(force, contribution);
    }
    return force;
}

/*
 * Match the average heading of nearby neighbours (the second boids rule).
 * Average everyone's velocity inside ALIGN_RADIUS and nudge toward it, so
 * people heading roughly the same way end up heading exactly the same way
 * — the "flying together" look.
 */
static Vec2 steer_align(const Person *people, int count, int self)
{
    /* add up the velocities of neighbours inside ALIGN_RADIUS */
    Vec2 velocity_sum     = v2(0, 0);
    int  n_neighbours     = 0;
    Vec2 pos              = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        bool inside_align_radius = v2len(v2sub(people[i].pos, pos)) < ALIGN_RADIUS;
        if (inside_align_radius) {
            velocity_sum = v2add(velocity_sum, people[i].vel);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* steer from our heading toward the crowd's average heading */
    Vec2 mean_velocity = v2scale(velocity_sum, 1.0f / (float)n_neighbours);
    return v2sub(mean_velocity, people[self].vel);
}

/*
 * Drift toward the middle of the nearby group (the third boids rule).
 * Average everyone's position inside COHESION_RADIUS and seek that point,
 * which keeps the flock from drifting apart. Reusing steer_seek means
 * the group acts as a gentle magnet that people ease into.
 */
static Vec2 steer_cohere(const Person *people, int count, int self)
{
    /* add up the positions of neighbours inside COHESION_RADIUS */
    Vec2 position_sum = v2(0, 0);
    int  n_neighbours = 0;
    Vec2 pos          = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        bool inside_cohesion_radius = v2len(v2sub(people[i].pos, pos)) < COHESION_RADIUS;
        if (inside_cohesion_radius) {
            position_sum = v2add(position_sum, people[i].pos);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* seek the average neighbour position */
    Vec2 centre_of_mass = v2scale(position_sum, 1.0f / (float)n_neighbours);
    return steer_seek(pos, people[self].vel, centre_of_mass, SPEED_BASE);
}

/* ── §6.5 force blend — weighted sum of two or three forces ── */
/*
 * Add up forces with a weight on each. Every behaviour combines its
 * forces this way; naming it lets each tick read as a single line like
 *   force = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);
 */
static inline Vec2 blend2_forces(Vec2 a, float wa, Vec2 b, float wb)
{
    return v2add(v2scale(a, wa), v2scale(b, wb));
}
static inline Vec2 blend3_forces(Vec2 a, float wa,
                                  Vec2 b, float wb,
                                  Vec2 c, float wc)
{
    return v2add(v2add(v2scale(a, wa), v2scale(b, wb)), v2scale(c, wc));
}

/* ── §7 scene — scene state, behaviour ticks, drawing ── */

/*
 * The pool of people. Steering needs the whole neighbour list, so people
 * live in one flat array.
 *
 * The array is always full size; only the first `count` are "live" and
 * get simulated and drawn. The rest were already placed at startup, so
 * pressing '+' just bumps `count` and reveals ready-positioned people
 * instead of spawning fresh ones each keypress (the project forbids
 * allocating after init).
 *
 *   people[CROWD_MAX]   the whole pool, all placed at startup.
 *   count               how many are live right now, CROWD_MIN..CROWD_MAX.
 */
typedef struct {
    Person people[CROWD_MAX];
    int    count;
} Crowd;

/*
 * The roaming '!' marker that the crowd flees in PANIC mode. Everyone
 * runs away from threat.pos. The threat itself wanders slowly toward
 * random points, so it prowls the room instead of sitting still — it is
 * basically a Person with no look fields, hence the same pos/vel/target.
 * Its speed is capped below the crowd's panic speed so people can always
 * get away: it's a source of pressure, not a chaser.
 *
 *   pos      where it is; the point everyone flees.
 *   vel      velocity, held under THREAT_SPEED.
 *   target   the random point it's currently heading toward.
 */
typedef struct {
    Vec2 pos;
    Vec2 vel;
    Vec2 target;
} Threat;

/*
 * The knobs the user controls. The input handler writes these; the
 * tick and HUD read them. Grouped together so they're easy to find.
 *
 *   behaviour   which mode is running (keys 1..6); picks the tick
 *               function and which decoration scene_draw adds.
 *   paused      true freezes the sim; the HUD shows PAUSED. SPACE toggles.
 *   fps         physics ticks per second; must be > 0 (it's a divisor).
 */
typedef struct {
    Behaviour behaviour;
    bool      paused;
    int       fps;
} SimControls;

/*
 * All the state for one run, in one place. A single Scene lives on the
 * stack in main() and is passed by pointer everywhere; functions that
 * only read it take a const pointer, so the signature shows whether a
 * function changes anything. (The two signal flags in §8 can't live here
 * because signal handlers can't be handed a pointer.)
 *
 * The size is tracked twice in two different units, both kept in sync so
 * nothing has to keep re-multiplying by CELL_W/CELL_H:
 *   scene_cols/rows  the terminal grid in cells, for drawing.
 *   world_w/h        the same area in pixels, for the physics.
 *
 *   crowd            the pool of people.
 *   threat           the panic-mode repulsor.
 *   sim              the user's knobs (behaviour, pause, fps).
 *   world_w/h        world size in pixels; recomputed on resize.
 *   scene_cols/rows  terminal size in cells; updated on resize.
 */
typedef struct {
    Crowd       crowd;
    Threat      threat;
    SimControls sim;
    float       world_w, world_h;        /* world size in pixels        */
    int         scene_cols, scene_rows;  /* terminal size in cells       */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);

    /* size in both units: cells for drawing, pixels for physics */
    s->scene_cols = cols;
    s->scene_rows = rows;
    s->world_w    = pw(cols);
    s->world_h    = ph(rows);

    /* starting knobs */
    s->crowd.count   = CROWD_DEFAULT;
    s->sim.behaviour = BEH_WANDER;
    s->sim.paused    = false;
    s->sim.fps       = SIM_FPS_DEFAULT;

    /* place the whole pool now, so '+' later just reveals ready people */
    for (int i = 0; i < CROWD_MAX; i++)
        person_spawn(&s->crowd.people[i], i, s->world_w, s->world_h);

    /* start the threat at the centre, heading somewhere random */
    const float WORLD_CENTRE_FRAC = 0.5f;
    s->threat.pos    = v2(s->world_w * WORLD_CENTRE_FRAC,
                          s->world_h * WORLD_CENTRE_FRAC);
    s->threat.target = v2(randf()*s->world_w, randf()*s->world_h);
}

/* ── helpers shared by the behaviour ticks and the renderer ── */

/*
 * Pick a point partway between where a person was and where they are,
 * set by alpha (0 = old spot, 1 = new spot). The renderer uses this so
 * motion stays smooth even when frames and physics ticks don't line up.
 */
static inline Vec2 lerp_render_pos(Vec2 prev, Vec2 cur, float alpha)
{
    /* how far they moved last tick, times the fraction we want now */
    Vec2 tick_delta   = v2sub(cur, prev);
    Vec2 partial_step = v2scale(tick_delta, alpha);
    return v2add(prev, partial_step);
}

/*
 * GATHER approach speed: full speed until inside the slow ring, then
 * ramping down to a crawl right at the centre, so the crowd settles and
 * mills around instead of overshooting back and forth.
 */
static inline float gather_approach_speed(float dist_to_centre)
{
    if (dist_to_centre >= GATHER_SLOW_RADIUS) return SPEED_BASE;
    /* 0 at the centre, up to full speed at the edge of the ring */
    float slow_ramp_fraction = dist_to_centre / GATHER_SLOW_RADIUS;
    return SPEED_BASE * slow_ramp_fraction;
}

/*
 * QUEUE approach speed: brisk until nearly at the slot, then a sudden
 * shuffle. The abrupt drop (not a gentle ramp) makes the line look
 * patient — people walk up, then stop and wait.
 */
static inline float queue_approach_speed(float dist_to_slot)
{
    return (dist_to_slot < QUEUE_SLOW_RADIUS) ? QUEUE_IDLE_SPEED : SPEED_BASE;
}

/*
 * Where person number i stands in the queue. The counter sits a couple
 * of cells in from the right edge, vertically centred; person i stands i
 * gaps to its left, nudged onto one of three rows so the line zigzags
 * instead of being a dead-straight column. Far-back slots that would run
 * off the left edge get pinned just inside it.
 */
static Vec2 queue_slot_for_index(int i, float ww, float wh)
{
    enum { COUNTER_INSET_CELLS = 2 };
    enum { STAGGER_ROWS        = 3 };
    const float WORLD_VERTICAL_CENTRE_FRAC = 0.5f;
    const float LEFT_EDGE_INSET_PX         = (float)CELL_W;

    /* counter: in from the right edge, vertically centred */
    const float counter_x = ww - (float)CELL_W * (float)COUNTER_INSET_CELLS;
    const float counter_y = wh * WORLD_VERTICAL_CENTRE_FRAC;

    /* i gaps left of the counter, on row -1/0/+1 so the line zigzags */
    int   stagger_row_offset = (i % STAGGER_ROWS) - 1;       /* {-1, 0, +1} */
    float slot_x             = counter_x - (float)i * QUEUE_SLOT_W;
    float slot_y             = counter_y + (float)stagger_row_offset * QUEUE_SLOT_H;

    /* keep far-back slots from sliding off the left edge */
    if (slot_x < LEFT_EDGE_INSET_PX) slot_x = LEFT_EDGE_INSET_PX;
    return v2(slot_x, slot_y);
}

/*
 * Move the panic threat one step. It drifts toward its random target;
 * when it gets close it picks a new one, otherwise it accelerates toward
 * it (speed held below the crowd's so they can escape) and stays inside
 * the walls.
 */
static void threat_advance(Threat *t, float dt, float ww, float wh)
{
    const float THREAT_ARRIVE_RADIUS = ARRIVE_DIST * 2.0f;

    Vec2  offset_to_target   = v2sub(t->target, t->pos);
    float distance_to_target = v2len(offset_to_target);
    if (distance_to_target < THREAT_ARRIVE_RADIUS) {
        t->target = v2(randf()*ww, randf()*wh);   /* arrived: pick a new spot */
    } else {
        /* steer toward the target, capping speed at THREAT_SPEED */
        Vec2 toward_dir = v2norm(offset_to_target);
        Vec2 accel_v    = v2scale(toward_dir, THREAT_SPEED);
        Vec2 new_vel    = v2add(t->vel, v2scale(accel_v, dt));
        t->vel          = v2clamp_len(new_vel, THREAT_SPEED);
    }

    /* move, then keep it inside the walls */
    Vec2 step = v2scale(t->vel, dt);
    t->pos    = v2add(t->pos, step);
    clamp_pos(&t->pos, ww, wh);
}

/* ── the six behaviours: one tick function each ── */
/*
 * Each one runs its behaviour over all live people. They share the same
 * shape: maybe pick a new target, gather the forces this mode cares
 * about from §6, blend them (§6.5), move the person (§5), then handle the
 * walls (§4).
 */

/*
 * WANDER: everyone drifts to their own random target, picking a new one
 * on arrival, with separation keeping them apart. Edges wrap around so
 * nobody gets stuck.
 */
static void tick_wander(Scene *s, float dt)
{
    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        if (person_at_target(p))
            person_pick_random_target(p, s->world_w, s->world_h);

        /* seek the target, keep personal space */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, p->target, SPEED_BASE);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FLOCK: classic boids. No target — the three rules (keep apart, match
 * heading, stay together) make the group itself the attractor. Try
 * changing W_ALIGN and W_COHERE to see the flock loosen or tighten.
 */
static void tick_flock(Scene *s, float dt)
{
    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* the three boids rules */
        Vec2 sep_force    = steer_separate(crowd->people, crowd->count, i);
        Vec2 align_force  = steer_align   (crowd->people, crowd->count, i);
        Vec2 cohere_force = steer_cohere  (crowd->people, crowd->count, i);

        /* separation weighted highest so flockmates never overlap */
        Vec2 force = blend3_forces(sep_force,    W_SEP,
                                   align_force,  W_ALIGN,
                                   cohere_force, W_COHERE);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * PANIC: everyone flees the roaming '!'. The threat is slower than the
 * crowd so they can get away as long as they keep moving, and the walls
 * bounce so nobody escapes off-screen, which keeps the pressure on.
 */
static void tick_panic(Scene *s, float dt)
{
    /* move the threat first, so people flee its new position */
    threat_advance(&s->threat, dt, s->world_w, s->world_h);

    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* flee the threat, keep personal space */
        Vec2 flee_force = steer_flee    (p->pos, p->vel, s->threat.pos, SPEED_PANIC);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(flee_force, W_FLEE, sep_force, W_SEP);

        person_step(p, force, dt);
        bounce_pos(&p->pos, &p->vel, s->world_w, s->world_h);
    }
}

/*
 * GATHER: everyone heads for the centre, slowing down as they get close
 * (see gather_approach_speed) so they mill around instead of overshooting.
 */
static void tick_gather(Scene *s, float dt)
{
    const float WORLD_CENTRE_FRAC = 0.5f;
    const Vec2  centre = v2(s->world_w * WORLD_CENTRE_FRAC,
                            s->world_h * WORLD_CENTRE_FRAC);
    Crowd *crowd = &s->crowd;

    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        float dist_to_centre = v2len(v2sub(centre, p->pos));
        float approach_speed = gather_approach_speed(dist_to_centre);

        /* seek the centre at the eased speed, keep personal space */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, centre, approach_speed);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FOLLOW: a conga line. The first person leads, wandering freely (drawn
 * as '@'); everyone else chases the person just ahead of them, with
 * separation keeping the tail from piling onto the leader.
 */
static void tick_follow(Scene *s, float dt)
{
    enum { LEADER_INDEX = 0 };
    Crowd *crowd = &s->crowd;

    /* leader wanders, like WANDER but with no separation */
    Person *leader = &crowd->people[LEADER_INDEX];
    if (person_at_target(leader))
        person_pick_random_target(leader, s->world_w, s->world_h);

    Vec2 lead_force = steer_seek(leader->pos, leader->vel,
                                  leader->target, SPEED_BASE);
    person_step(leader, lead_force, dt);
    wrap_pos(&leader->pos, s->world_w, s->world_h);

    /* everyone else chases the person ahead of them */
    for (int i = LEADER_INDEX + 1; i < crowd->count; i++) {
        Person *p    = &crowd->people[i];
        Person *prev = &crowd->people[i - 1];

        Vec2 chase_force = steer_seek    (p->pos, p->vel, prev->pos, SPEED_BASE);
        Vec2 sep_force   = steer_separate(crowd->people, crowd->count, i);
        Vec2 force       = blend2_forces(chase_force, W_SEEK, sep_force, W_SEP);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * QUEUE: each person walks to their own reserved spot in a line running
 * left from the counter. They slow to a shuffle near their spot, and
 * separation is relaxed (halved) so they tolerate standing close together.
 */
static void tick_queue(Scene *s, float dt)
{
    const float W_SEP_PACKED = W_SEP * 0.5f;   /* relaxed so the line packs in */
    Crowd *crowd = &s->crowd;

    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        Vec2  slot           = queue_slot_for_index(i, s->world_w, s->world_h);
        float dist_to_slot   = v2len(v2sub(slot, p->pos));
        float approach_speed = queue_approach_speed(dist_to_slot);

        /* seek the slot with relaxed separation */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, slot, approach_speed);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP_PACKED);

        /* pin to the walls so the line stops at the counter */
        person_step(p, force, dt);
        clamp_pos(&p->pos, s->world_w, s->world_h);
    }
}

/* ── pick a tick, then draw the scene ── */

/*
 * Draw one character at cell (cx,cy) in a colour. One place for the
 * bounds-check, the colour on/off, and the double cast that stops
 * characters above 127 from being mangled (an ncurses gotcha).
 * Off-screen cells are quietly skipped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

static void scene_tick(Scene *s, float dt)
{
    /* keep pixel size in step with the cell size, in case of a resize */
    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);
    if (s->sim.paused) return;
    switch (s->sim.behaviour) {
    case BEH_WANDER: tick_wander(s, dt); break;
    case BEH_FLOCK:  tick_flock (s, dt); break;
    case BEH_PANIC:  tick_panic (s, dt); break;
    case BEH_GATHER: tick_gather(s, dt); break;
    case BEH_FOLLOW: tick_follow(s, dt); break;
    case BEH_QUEUE:  tick_queue (s, dt); break;
    default: break;
    }
}

/* Draw the panic threat as a blinking red '!'. */
static void draw_threat_marker(WINDOW *w, const Threat *t, int cols, int rows)
{
    enum { THREAT_COLOR = 1 };       /* red pair */
    int tx = px_to_cell_x(t->pos.x);
    int ty = px_to_cell_y(t->pos.y);
    mark_cell(w, tx, ty, '!', THREAT_COLOR, A_BOLD | A_BLINK, cols, rows);
}

/*
 * Draw the queue's service counter: a vertical '|' bar near the right
 * edge, with a '>>|' arrow pointing into it to show which way the line
 * is moving.
 */
static void draw_queue_counter(WINDOW *w, int cols, int rows)
{
    enum { COUNTER_COLOR        = 3 };       /* yellow */
    enum { COUNTER_INSET_CELLS  = 2 };       /* from right edge */
    enum { COUNTER_HALF_HEIGHT  = 3 };       /* cells above/below mid-row */
    enum { ARROW_TIP_OFFSET     = 1 };       /* '|' one cell left of counter */
    enum { ARROW_BODY_OFFSET    = 2 };       /* first '>' two cells left  */
    enum { ARROW_TAIL_OFFSET    = 3 };       /* second '>' three cells left */

    const int counter_col = cols - COUNTER_INSET_CELLS;
    const int counter_row = rows / 2;

    /* the counter bar */
    for (int dy = -COUNTER_HALF_HEIGHT; dy <= COUNTER_HALF_HEIGHT; dy++)
        mark_cell(w, counter_col, counter_row + dy, '|',
                  COUNTER_COLOR, A_BOLD, cols, rows);

    /* the arrow leading into it */
    mark_cell(w, counter_col - ARROW_TAIL_OFFSET, counter_row, '>',
              COUNTER_COLOR, A_BOLD, cols, rows);
    mark_cell(w, counter_col - ARROW_BODY_OFFSET, counter_row, '>',
              COUNTER_COLOR, A_BOLD, cols, rows);
    mark_cell(w, counter_col - ARROW_TIP_OFFSET,  counter_row, '|',
              COUNTER_COLOR, A_BOLD, cols, rows);
}

/* Draw the follow-mode leader as a bold underlined '@' so it stands out. */
static void draw_follow_leader(WINDOW *w, const Person *ldr, float alpha,
                                int cols, int rows)
{
    enum { LEADER_COLOR = 3 };       /* yellow */
    Vec2 dp = lerp_render_pos(ldr->prev_pos, ldr->pos, alpha);
    mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
              '@', LEADER_COLOR, A_BOLD | A_UNDERLINE, cols, rows);
}

/* Draw one person at their smoothed position, in their own glyph and colour. */
static void draw_person(WINDOW *w, const Person *p, float alpha,
                         int cols, int rows)
{
    Vec2 dp = lerp_render_pos(p->prev_pos, p->pos, alpha);
    mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
              p->glyph, p->color, A_NORMAL, cols, rows);
}

/*
 * Draw the whole scene. Mode-specific decoration (the threat, counter,
 * or leader) goes down first so the crowd lays on top of it; then every
 * live person, skipping the first one in FOLLOW since the leader was
 * already drawn.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    enum { LEADER_INDEX = 0 };

    /* decoration for the current mode */
    if (s->sim.behaviour == BEH_PANIC)
        draw_threat_marker(w, &s->threat, cols, rows);

    if (s->sim.behaviour == BEH_QUEUE)
        draw_queue_counter(w, cols, rows);

    if (s->sim.behaviour == BEH_FOLLOW && s->crowd.count > 0)
        draw_follow_leader(w, &s->crowd.people[LEADER_INDEX], alpha, cols, rows);

    /* the crowd, skipping the already-drawn leader in FOLLOW */
    int start = (s->sim.behaviour == BEH_FOLLOW) ? LEADER_INDEX + 1 : 0;
    for (int i = start; i < s->crowd.count; i++)
        draw_person(w, &s->crowd.people[i], alpha, cols, rows);
}

/* ── §8 app — screen, signals, input, main loop ── */

/*
 * Flags a signal handler can flip. They must be these special globals
 * because that's all a handler is allowed to touch safely; main() reads
 * and clears them.
 */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Start ncurses, then record the terminal size in the Scene. */
static void screen_init(Scene *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);   /* getch returns at once if no key is waiting */
    keypad(stdscr, TRUE);
    typeahead(-1);            /* stop ncurses pausing output to peek at input */
    color_init();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);
}

/* Print bold coloured text at (row, col). Shared by the two HUD lines. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the status line (top right) and the key hints (bottom). */
static void draw_hud(const Scene *s, double fps)
{
    enum { HUD_TOP_ROW = 0 };
    static const char *KEY_HINT =
        " q:quit  spc:pause  1:wander 2:flock 3:panic 4:gather 5:follow 6:queue  +/-:people  r:reset ";

    char status[HUD_COLS + 1];
    snprintf(status, sizeof status,
             " %5.1f fps  sim:%3d Hz  n:%3d  [%s]%s ",
             fps, s->sim.fps, s->crowd.count,
             BEH_NAMES[s->sim.behaviour],
             s->sim.paused ? "  PAUSED" : "");

    /* right-align the status on the top row */
    int right_col = s->scene_cols - (int)strlen(status);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, status);

    hud_paint_text(s->scene_rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* Draw one frame: scene first, HUD on top, then flush it all at once. */
static void frame_render(const Scene *s, double fps, float alpha)
{
    erase();
    scene_draw(s, stdscr, s->scene_cols, s->scene_rows, alpha);
    draw_hud(s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * After a shrink, pull anyone now outside the smaller world back in, and
 * reset prev_pos so the renderer doesn't draw a long snap-back streak.
 */
static void clamp_all_active_people(Scene *s)
{
    for (int i = 0; i < s->crowd.count; i++) {
        Person *p = &s->crowd.people[i];
        clamp_pos(&p->pos, s->world_w, s->world_h);
        p->prev_pos = p->pos;
    }
}

/*
 * After a terminal resize: restart ncurses so it sees the new size, then
 * recompute the world and pull stragglers back in. Does nothing if no
 * resize is pending.
 */
static void handle_resize_if_pending(Scene *s)
{
    if (!g_need_resize) return;
    g_need_resize = 0;

    /* restart ncurses so stdscr matches the new terminal size */
    endwin(); refresh();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);

    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);

    clamp_all_active_people(s);
}

/*
 * Change how many people are live by delta, kept within the limits.
 * The pool is already placed, so this just reveals or hides people.
 */
static void adjust_person_count(Crowd *crowd, int delta)
{
    int next = crowd->count + delta;
    if (next < CROWD_MIN) next = CROWD_MIN;
    if (next > CROWD_MAX) next = CROWD_MAX;
    crowd->count = next;
}

/* Act on one keypress. */
static void handle_input(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        g_running = 0;                                       break;

    case ' ':  s->sim.paused    = !s->sim.paused;            break;

    /* keys 1..6 pick a behaviour */
    case '1':  s->sim.behaviour = BEH_WANDER;                break;
    case '2':  s->sim.behaviour = BEH_FLOCK;                 break;
    case '3':  s->sim.behaviour = BEH_PANIC;                 break;
    case '4':  s->sim.behaviour = BEH_GATHER;                break;
    case '5':  s->sim.behaviour = BEH_FOLLOW;                break;
    case '6':  s->sim.behaviour = BEH_QUEUE;                 break;

    /* +/- show or hide people */
    case '+': case '=':
        adjust_person_count(&s->crowd, +CROWD_STEP);         break;
    case '-':
        adjust_person_count(&s->crowd, -CROWD_STEP);         break;

    /* r reshuffles the whole scene */
    case 'r': case 'R':
        scene_init(s, s->scene_cols, s->scene_rows);         break;

    default: break;
    }
}

/*
 * Counts frames over a short window to show a steady fps. Updating the
 * number only once per window stops the HUD reading from flickering.
 *
 *   frame_count   frames seen so far this window.
 *   window_ns     time elapsed this window, nanoseconds.
 *   display       the smoothed fps the HUD shows.
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

/* Count one frame; refresh the displayed fps once the window is full. */
static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;

    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;

    f->display     = (double)f->frame_count
                   / ((double)f->window_ns / (double)NS_PER_SEC);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * Run the physics in fixed-size steps. Time piles up in sim_accum; we
 * run one tick per whole step waiting there. This keeps the sim rate
 * steady no matter how fast the frames come.
 */
static void drain_sim_accumulator(Scene *s, int64_t *sim_accum,
                                   int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/*
 * The game loop. Each pass: handle any resize, measure elapsed time, run
 * the physics to catch up, work out how far into the next tick we are
 * (alpha, for smooth drawing), update the fps reading, sleep to hold the
 * frame rate, draw, and read a key.
 */
int main(void)
{
    /* seed RNG, clean up ncurses on exit, install signal handlers */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    Scene scene;
    screen_init(&scene);
    scene_init(&scene, scene.scene_cols, scene.scene_rows);

    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;             /* cap a long pause so physics can't avalanche */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;     /* time allowed per frame */

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;
    FpsCounter fps;
    fps_counter_init(&fps);

    while (g_running) {
        int64_t frame_start = clock_ns();

        if (g_need_resize) {
            handle_resize_if_pending(&scene);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* time since last frame, capped so a hiccup can't flood the sim */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* run fixed-size physics steps until caught up */
        const int64_t tick_ns = TICK_NS(scene.sim.fps);
        const float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        drain_sim_accumulator(&scene, &sim_accum, tick_ns, dt_sec);

        /* how far into the next step we are, for smooth drawing */
        float alpha = (float)sim_accum / (float)tick_ns;

        fps_counter_tick(&fps, dt);

        /* sleep before drawing so I/O time doesn't eat the frame budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        frame_render(&scene, fps.display, alpha);

        int key = getch();
        if (key != ERR) handle_input(&scene, key);
    }

    return 0;
}
