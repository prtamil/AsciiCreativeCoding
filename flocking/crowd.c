/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * crowd.c — ASCII Crowd Simulator
 *
 * N people (ASCII characters) move around the terminal under one of six
 * switchable crowd behaviours.  Press 1–6 to cycle between them live.
 *
 * This file follows the canonical 8-section framework pattern from
 * ncurses_basics/framework.c — study that file first for the game-loop.
 * The new concept introduced here is STEERING BEHAVIOURS (Reynolds 1987):
 * every entity accumulates a weighted sum of forces each physics tick.
 *
 * Keys
 * ────
 *   q / ESC    quit
 *   space      pause / resume
 *   1          WANDER  — random targets, personal space enforced
 *   2          FLOCK   — boids: separation + alignment + cohesion
 *   3          PANIC   — flee a roaming threat '!'
 *   4          GATHER  — converge to screen centre
 *   5          FOLLOW  — chain-follow the leader '@'
 *   6          QUEUE   — orderly line to right-edge counter
 *   + / =      add 5 people   (max 150)
 *   -          remove 5 people (min 5)
 *   r          reset positions
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/crowd.c -o crowd -lncurses -lm
 *
 * §1 config   §2 clock   §3 color   §4 coords & vec2
 * §5 entity   §6 steering §7 scene   §8 app
 */

/* ── STEERING BEHAVIOUR PRIMER ─────────────────────────────────────────
 *
 * HOW MOVEMENT WORKS (same pattern in every behaviour):
 *
 *   Each tick, a person computes a STEERING FORCE — a 2-D vector
 *   pointing where they "want" to go.  That force is added to velocity
 *   and velocity is capped so nobody goes hypersonic.  This gives
 *   smooth, momentum-based motion instead of instant teleporting.
 *
 *       force     = W_A * forceA  +  W_B * forceB  + ...
 *       velocity += force × dt
 *       velocity  = clamp(|velocity|, SPEED_MAX)
 *       position += velocity × dt
 *
 * THE SEEK FORCE (used by most behaviours):
 *
 *       desired   = normalise(target − pos) × desired_speed
 *       seek      = desired − current_velocity
 *
 *   This steers toward the target.  As the agent speeds up toward the
 *   target its current_velocity approaches desired, so the force naturally
 *   shrinks — producing smooth deceleration for free.
 *
 * FLEE = seek with the sign flipped (steer AWAY from threat).
 *
 * SEPARATION — prevents crowding:
 *   For every neighbour within SEP_RADIUS, apply a repulsion force.
 *   Strength ∝ (SEP_RADIUS − distance): touching → full push, edge → ~0.
 *
 * BOIDS (FLOCK mode) — three independent local rules:
 *   Separation  — avoid neighbours within SEP_RADIUS
 *   Alignment   — match average heading of neighbours in ALIGN_RADIUS
 *   Cohesion    — seek centre of mass of neighbours in COHESION_RADIUS
 *   Flocking emerges from purely local interaction, no central authority.
 *
 * TWO COORDINATE SPACES (why §4 matters):
 *   PIXEL space — physics lives here; one unit ≈ one physical screen pixel.
 *                 Square and isotropic: moving 1 unit in X travels the same
 *                 physical distance as 1 unit in Y.
 *   CELL space  — terminal rows/cols where drawing happens.
 *                 Cells are ~2× taller than wide, so if physics were done
 *                 in cell coords diagonal motion would look skewed.
 *   Only the draw step converts pixel → cell via px_to_cell_x/y.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — every tunable constant lives here, nowhere else          */
/* ===================================================================== */

/*
 * Simulation and display rates.
 * SIM_FPS_DEFAULT controls how many physics steps run per second.
 * Raising it makes physics more precise but costs more CPU.
 * TARGET_FPS is the render cap — 60 is plenty for a terminal.
 */
enum {
    SIM_FPS_DEFAULT =  60,
    TARGET_FPS      =  60,

    HUD_COLS        =  72,   /* max chars in the status-bar string */
    FPS_UPDATE_MS   = 500,   /* how often the displayed fps refreshes */

    N_COLORS        =   7,   /* color pairs defined in §3 */

    CROWD_MIN       =   5,   /* fewest people allowed */
    CROWD_MAX       = 150,   /* pool size; also the hard cap */
    CROWD_DEFAULT   =  60,   /* people on startup */
    CROWD_STEP      =   5,   /* how many +/- adds or removes */
};

/*
 * Cell dimensions in pixels.
 * A typical terminal cell is 8 px wide × 16 px tall (ratio 1:2).
 * Physics positions are stored in pixel units so that moving at
 * the same speed in X and Y covers equal physical distance.
 * Change these only if your terminal has a different cell ratio.
 */
#define CELL_W   8    /* pixels per terminal column */
#define CELL_H  16    /* pixels per terminal row    */

/*
 * Steering neighbourhood radii (pixels).
 *
 * SEP_RADIUS      — personal-space bubble; neighbours inside get pushed away.
 *                   Smaller → people cluster tighter; larger → roomier crowd.
 * ALIGN_RADIUS    — boids only: match heading of neighbours within this.
 *                   Larger → bigger "flock units"; smaller → chaotic swarms.
 * COHESION_RADIUS — boids only: seek average position of neighbours within.
 *                   Must be > ALIGN_RADIUS for cohesion to pull strays back.
 */
#define SEP_RADIUS       40.0f
#define ALIGN_RADIUS     80.0f
#define COHESION_RADIUS 120.0f

/*
 * Speeds (pixels per second).
 *
 * SPEED_BASE    — relaxed cruising speed for wander/flock/gather/follow.
 *                 At 60fps a person moves SPEED_BASE/60 px per frame (~1.3 px).
 * SPEED_PANIC   — speed used by flee force; higher = more frantic escape.
 * SPEED_MAX     — hard cap: no agent can ever exceed this regardless of forces.
 *                 Prevents runaway acceleration when many forces stack.
 * THREAT_SPEED  — how fast the panic threat moves; slower than people so
 *                 the crowd can escape but must keep moving.
 */
#define SPEED_BASE    80.0f
#define SPEED_PANIC  140.0f
#define SPEED_MAX    160.0f
#define THREAT_SPEED  55.0f

/*
 * Steering force weights.
 * The final force = W_X * forceX + W_Y * forceY + ...
 * Increasing a weight makes that rule dominate.
 * W_SEP > W_SEEK keeps separation from being overridden by targeting.
 */
#define W_SEEK    1.0f   /* seek / flee multiplier                     */
#define W_FLEE    1.6f   /* flee weighted higher so panic feels urgent  */
#define W_SEP     1.8f   /* separation weighted highest — no overlap    */
#define W_ALIGN   1.0f   /* boids alignment                             */
#define W_COHERE  0.8f   /* boids cohesion (softer than alignment)      */

/*
 * Arrival thresholds (pixels).
 *
 * ARRIVE_DIST        — a person is "close enough" to their target when
 *                      within this radius.  They pick a new target then.
 *                      ~3 cells wide at the default CELL_W=8.
 * GATHER_SLOW_RADIUS — GATHER mode slows people down when this close to
 *                      the centre so they mill around rather than overshooting.
 * QUEUE_SLOW_RADIUS  — similar slow-down when near a queue slot.
 * QUEUE_IDLE_SPEED   — shuffle speed once nearly at slot position.
 */
#define ARRIVE_DIST         24.0f
#define GATHER_SLOW_RADIUS  (ARRIVE_DIST * 4.0f)   /* 96 px ≈ 12 cells  */
#define QUEUE_SLOW_RADIUS   (ARRIVE_DIST * 2.0f)   /* 48 px ≈  6 cells  */
#define QUEUE_IDLE_SPEED    (SPEED_BASE  * 0.3f)   /* 24 px/s — shuffle */

/*
 * Queue layout geometry.
 * Slots extend leftward from the service counter in a 3-row stagger.
 * QUEUE_SLOT_W — horizontal gap between slots (1.5 cells apart).
 * QUEUE_SLOT_H — vertical stagger between the 3 rows (1.5 cells apart).
 */
#define QUEUE_SLOT_W  ((float)CELL_W  * 1.5f)
#define QUEUE_SLOT_H  ((float)CELL_H  * 1.5f)

/* Timing helpers */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                             */
/* ===================================================================== */

/*
 * clock_ns — monotonic wall-clock in nanoseconds.
 * CLOCK_MONOTONIC never jumps backward (unlike CLOCK_REALTIME which can
 * shift on NTP updates), making it safe for dt measurement.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns — sleep for exactly ns nanoseconds.
 * Called BEFORE the render step so that terminal I/O time does not
 * eat into the frame budget (see §8 main loop for the reasoning).
 */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                             */
/* ===================================================================== */

/*
 * color_init — define N_COLORS ncurses color pairs.
 * Pair IDs 1–7 map to the 7 visible colors used by people and UI.
 * 256-color path uses xterm-256 indices for vivid saturated hues.
 * 8-color fallback uses the basic ANSI colors on older terminals.
 *
 * Pair map:  1=red  2=orange  3=yellow  4=green  5=cyan  6=blue  7=magenta
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  21, COLOR_BLACK);
        init_pair(7, 201, COLOR_BLACK);
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords & vec2                                                     */
/* ===================================================================== */

/* pixel-space world dimensions from terminal cell counts */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell index.
 * Uses floorf(px/CELL + 0.5) — "round half up" — for stable rounding.
 * Plain truncation would round toward zero and cause staircase artefacts;
 * roundf() can oscillate at .5 boundaries due to banker's rounding.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── Vec2 — lightweight 2-D vector type ───────────────────────────── */
/*
 * All physics quantities (position, velocity, force) are Vec2.
 * Using a named type instead of bare float pairs makes function
 * signatures read clearly and eliminates double-pointer output params.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/*
 * v2norm — return unit vector; returns zero-vector if v is near-zero
 * to avoid divide-by-zero when two people sit on the same pixel.
 */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/*
 * v2clamp_len — cap the vector's magnitude to max_len.
 * Used after accumulating forces to enforce SPEED_MAX.
 */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* ── boundary helpers ──────────────────────────────────────────────── */

/*
 * wrap_pos — torus wrapping.  Walking off the right edge reappears
 * on the left.  Used by WANDER, FLOCK, GATHER, FOLLOW.
 */
static void wrap_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x += ww;
    if (pos->x >= ww)  pos->x -= ww;
    if (pos->y <  0)   pos->y += wh;
    if (pos->y >= wh)  pos->y -= wh;
}

/*
 * bounce_pos — elastic wall collision.  Hitting an edge flips the
 * matching velocity component.  Used by PANIC so the crowd is trapped.
 */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)   { pos->x = 0;      vel->x =  fabsf(vel->x); }
    if (pos->x >= ww)  { pos->x = ww-1;   vel->x = -fabsf(vel->x); }
    if (pos->y <  0)   { pos->y = 0;      vel->y =  fabsf(vel->y); }
    if (pos->y >= wh)  { pos->y = wh-1;   vel->y = -fabsf(vel->y); }
}

/* clamp_pos — hard-clip to world bounds.  Used by QUEUE. */
static void clamp_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x = 0;
    if (pos->x >= ww)  pos->x = ww-1;
    if (pos->y <  0)   pos->y = 0;
    if (pos->y >= wh)  pos->y = wh-1;
}

/* ===================================================================== */
/* §5  entity                                                            */
/* ===================================================================== */

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

/*
 * Glyphs assigned round-robin by person index.
 * Mix of round and angular ASCII chars that suggest human silhouettes.
 */
static const char GLYPHS[] = "oO0abcdefghijklmnpqrstuvwxyz";
#define N_GLYPHS ((int)(sizeof(GLYPHS) - 1))

/*
 * Person — all per-entity state.
 *
 * pos      — current position in pixel space.
 * prev_pos — position from the previous tick, kept so the renderer can
 *            lerp between prev and pos by alpha for smooth sub-tick motion.
 * vel      — velocity in pixels per second.
 * target   — personal wander/queue destination (reused per behaviour).
 * glyph    — the ASCII character drawn for this person.
 * color    — ncurses color pair index (1–N_COLORS).
 */
typedef struct {
    Vec2  pos;
    Vec2  prev_pos;
    Vec2  vel;
    Vec2  target;
    char  glyph;
    int   color;
} Person;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* person_spawn — randomise all fields of one person within the world. */
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
 * person_step — shared integration used by every behaviour.
 *
 * 1. Accumulate: add acceleration × dt to velocity.
 * 2. Clamp:      cap speed to SPEED_MAX.
 * 3. Snapshot:   save current pos as prev_pos for alpha interpolation.
 * 4. Integrate:  advance position by velocity × dt.
 *
 * Boundary handling (wrap / bounce / clamp) is left to the caller
 * because each behaviour uses a different boundary condition.
 */
static void person_step(Person *p, Vec2 accel, float dt)
{
    p->vel      = v2clamp_len(v2add(p->vel, v2scale(accel, dt)), SPEED_MAX);
    p->prev_pos = p->pos;
    p->pos      = v2add(p->pos, v2scale(p->vel, dt));
}

/* ===================================================================== */
/* §6  steering — force functions, all return Vec2                       */
/* ===================================================================== */

/*
 * steer_seek — generate a force that steers toward (target) at (speed).
 *
 *   desired = normalise(target − pos) × speed   ← where we want to go
 *   force   = desired − vel                      ← error correction term
 *
 * The subtraction of current velocity is the key insight: when we're
 * already going at full desired speed toward the target, force → 0.
 * This gives natural deceleration without any explicit slow-down code.
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
    return v2sub(desired, vel);
}

/*
 * steer_flee — steer directly AWAY from (threat).
 * Implemented as seek with the result negated; all the same math applies.
 */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed)
{
    return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * steer_separate — push away from all neighbours within SEP_RADIUS.
 *
 * For each neighbour closer than SEP_RADIUS:
 *   direction = normalise(my_pos − neighbour_pos)   ← point away
 *   strength  = (SEP_RADIUS − distance) / SEP_RADIUS ← closer = stronger
 *   force    += direction × strength × SPEED_BASE
 *
 * The linear falloff means neighbours at the edge of the radius
 * contribute almost nothing; only very close ones produce strong pushes.
 * This creates soft "personal space" rather than a hard exclusion zone.
 */
static Vec2 steer_separate(const Person *people, int count, int self)
{
    Vec2 force = v2(0, 0);
    Vec2 pos   = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        Vec2  away = v2sub(pos, people[i].pos);
        float d    = v2len(away);
        if (d < SEP_RADIUS && d > 0.001f) {
            float strength = (SEP_RADIUS - d) / SEP_RADIUS;
            force = v2add(force, v2scale(v2norm(away), strength * SPEED_BASE));
        }
    }
    return force;
}

/*
 * steer_align — match the average heading of nearby neighbours (boids rule 2).
 *
 * Computes mean velocity of everyone within ALIGN_RADIUS, then returns
 * a force that nudges this person's velocity toward that average.
 * Effect: birds that are going roughly the same direction converge
 * to exactly the same direction → the "flying together" look.
 */
static Vec2 steer_align(const Person *people, int count, int self)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    Vec2 pos = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        if (v2len(v2sub(people[i].pos, pos)) < ALIGN_RADIUS) {
            sum = v2add(sum, people[i].vel);
            n++;
        }
    }
    if (!n) return v2(0, 0);
    /* steer toward (mean_velocity − own_velocity) */
    return v2sub(v2scale(sum, 1.0f/n), people[self].vel);
}

/*
 * steer_cohere — seek the centre of mass of nearby neighbours (boids rule 3).
 *
 * Finds the average position of everyone within COHESION_RADIUS, then
 * seeks toward that point.  Prevents the flock from drifting apart.
 * Note: this reuses steer_seek, which already handles the "slow down
 * when arriving" behaviour — the group acts as a soft attractor.
 */
static Vec2 steer_cohere(const Person *people, int count, int self)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    Vec2 pos = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        if (v2len(v2sub(people[i].pos, pos)) < COHESION_RADIUS) {
            sum = v2add(sum, people[i].pos);
            n++;
        }
    }
    if (!n) return v2(0, 0);
    Vec2 centre = v2scale(sum, 1.0f/n);
    return steer_seek(pos, people[self].vel, centre, SPEED_BASE);
}

/* ===================================================================== */
/* §7  scene                                                             */
/* ===================================================================== */

/*
 * Scene — all simulation state in one struct.
 *
 * people[]       — pool of all agents; only [0 .. count-1] are active.
 *                  The pool is always fully initialised so that raising
 *                  count with '+' reveals already-placed people.
 * threat_*       — roaming threat used only in PANIC mode.
 * world_w/h      — pixel dimensions, recomputed from cols/rows each tick.
 */
typedef struct {
    Person    people[CROWD_MAX];
    int       count;
    Behaviour behaviour;
    bool      paused;
    Vec2      threat_pos;
    Vec2      threat_vel;
    Vec2      threat_target;
    float     world_w, world_h;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->world_w    = pw(cols);
    s->world_h    = ph(rows);
    s->count      = CROWD_DEFAULT;
    s->behaviour  = BEH_WANDER;
    for (int i = 0; i < CROWD_MAX; i++)
        person_spawn(&s->people[i], i, s->world_w, s->world_h);
    s->threat_pos    = v2(s->world_w * 0.5f, s->world_h * 0.5f);
    s->threat_target = v2(randf()*s->world_w, randf()*s->world_h);
}

/* ── behaviour tick functions ──────────────────────────────────────── */
/*
 * Each tick_* function applies one behaviour to all active people.
 * Pattern inside every loop body:
 *   1. Compute force  = weighted sum of relevant steering forces.
 *   2. Call person_step(p, force, dt)  → update vel + pos.
 *   3. Apply boundary condition.
 */

/*
 * WANDER — individuals drift to random targets; separation enforces
 * personal space.  On arrival each person picks a new random target.
 * The world wraps toroidally so nobody gets stuck at edges.
 */
static void tick_wander(Scene *s, float dt)
{
    for (int i = 0; i < s->count; i++) {
        Person *p = &s->people[i];

        if (v2len(v2sub(p->target, p->pos)) < ARRIVE_DIST)
            p->target = v2(randf()*s->world_w, randf()*s->world_h);

        Vec2 force = v2add(
            v2scale(steer_seek    (p->pos, p->vel, p->target, SPEED_BASE), W_SEEK),
            v2scale(steer_separate(s->people, s->count, i),                W_SEP)
        );
        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FLOCK — classic Reynolds boids with three weighted rules.
 * No explicit target: the group itself becomes the attractor.
 * Try raising/lowering W_ALIGN and W_COHERE to see the flock change.
 */
static void tick_flock(Scene *s, float dt)
{
    for (int i = 0; i < s->count; i++) {
        Person *p = &s->people[i];

        Vec2 force = v2(0, 0);
        force = v2add(force, v2scale(steer_separate(s->people, s->count, i), W_SEP));
        force = v2add(force, v2scale(steer_align   (s->people, s->count, i), W_ALIGN));
        force = v2add(force, v2scale(steer_cohere  (s->people, s->count, i), W_COHERE));

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * PANIC — everyone flees a slowly roaming threat '!'.
 * The threat is slower than the crowd (THREAT_SPEED < SPEED_PANIC)
 * so escape is possible, but only if people keep moving.
 * Walls bounce — people can't escape off-screen, adding pressure.
 */
static void tick_panic(Scene *s, float dt)
{
    /* advance threat toward its own wander target */
    Vec2  tdiff = v2sub(s->threat_target, s->threat_pos);
    float tdist = v2len(tdiff);
    if (tdist < ARRIVE_DIST * 2.0f) {
        s->threat_target = v2(randf()*s->world_w, randf()*s->world_h);
    } else {
        Vec2 t_accel = v2scale(v2norm(tdiff), THREAT_SPEED);
        s->threat_vel = v2clamp_len(v2add(s->threat_vel, v2scale(t_accel, dt)),
                                    THREAT_SPEED);
    }
    s->threat_pos = v2add(s->threat_pos, v2scale(s->threat_vel, dt));
    clamp_pos(&s->threat_pos, s->world_w, s->world_h);

    for (int i = 0; i < s->count; i++) {
        Person *p = &s->people[i];
        Vec2 force = v2add(
            v2scale(steer_flee    (p->pos, p->vel, s->threat_pos, SPEED_PANIC), W_FLEE),
            v2scale(steer_separate(s->people, s->count, i),                     W_SEP)
        );
        person_step(p, force, dt);
        bounce_pos(&p->pos, &p->vel, s->world_w, s->world_h);
    }
}

/*
 * GATHER — everyone seeks the screen centre.
 * Speed scales down linearly inside GATHER_SLOW_RADIUS so people
 * mill around once gathered rather than overshooting and oscillating.
 */
static void tick_gather(Scene *s, float dt)
{
    Vec2 centre = v2(s->world_w * 0.5f, s->world_h * 0.5f);
    for (int i = 0; i < s->count; i++) {
        Person *p = &s->people[i];
        float dist = v2len(v2sub(centre, p->pos));
        /* approach speed ramps from 0 at centre to SPEED_BASE at slow radius */
        float spd  = (dist < GATHER_SLOW_RADIUS)
                   ? SPEED_BASE * (dist / GATHER_SLOW_RADIUS)
                   : SPEED_BASE;
        Vec2 force = v2add(
            v2scale(steer_seek    (p->pos, p->vel, centre, spd), W_SEEK),
            v2scale(steer_separate(s->people, s->count, i),      W_SEP)
        );
        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FOLLOW — chain following: person[i] seeks person[i-1].
 * Person[0] is the leader (drawn as '@') and wanders freely.
 * The chain produces a snake-like procession; separation prevents
 * the tail from bunching up against the leader.
 */
static void tick_follow(Scene *s, float dt)
{
    /* leader wanders independently */
    Person *leader = &s->people[0];
    if (v2len(v2sub(leader->target, leader->pos)) < ARRIVE_DIST)
        leader->target = v2(randf()*s->world_w, randf()*s->world_h);

    Vec2 lead_force = steer_seek(leader->pos, leader->vel,
                                 leader->target, SPEED_BASE);
    person_step(leader, lead_force, dt);
    wrap_pos(&leader->pos, s->world_w, s->world_h);

    /* each follower seeks the person directly ahead */
    for (int i = 1; i < s->count; i++) {
        Person *p    = &s->people[i];
        Person *prev = &s->people[i-1];
        Vec2 force = v2add(
            steer_seek    (p->pos, p->vel, prev->pos, SPEED_BASE),
            v2scale(steer_separate(s->people, s->count, i), W_SEP)
        );
        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * QUEUE — each person is assigned a reserved slot in a line that
 * extends leftward from the right-edge service counter.
 * Slots stagger across 3 rows to prevent a dead-straight column.
 * Speed drops to QUEUE_IDLE_SPEED near the slot (patient shuffling).
 * Separation weight is halved — people tolerate being packed in line.
 */
static void tick_queue(Scene *s, float dt)
{
    /* service point: rightmost column, vertically centred */
    float sx = s->world_w - (float)CELL_W * 2.0f;
    float sy = s->world_h * 0.5f;

    for (int i = 0; i < s->count; i++) {
        Person *p = &s->people[i];

        /* slot position for this person */
        float slot_x = sx - (float)i * QUEUE_SLOT_W;
        float slot_y = sy + (float)((i % 3) - 1) * QUEUE_SLOT_H;
        /* clamp so distant slots don't fall off the left edge */
        if (slot_x < (float)CELL_W) slot_x = (float)CELL_W;

        Vec2  slot = v2(slot_x, slot_y);
        float dist = v2len(v2sub(slot, p->pos));
        float spd  = (dist < QUEUE_SLOW_RADIUS) ? QUEUE_IDLE_SPEED : SPEED_BASE;

        Vec2 force = v2add(
            v2scale(steer_seek    (p->pos, p->vel, slot, spd),  W_SEEK),
            v2scale(steer_separate(s->people, s->count, i),     W_SEP * 0.5f)
        );
        person_step(p, force, dt);
        clamp_pos(&p->pos, s->world_w, s->world_h);
    }
}

/* ── scene dispatch and draw ───────────────────────────────────────── */

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    s->world_w = pw(cols);
    s->world_h = ph(rows);
    if (s->paused) return;
    switch (s->behaviour) {
    case BEH_WANDER: tick_wander(s, dt); break;
    case BEH_FLOCK:  tick_flock (s, dt); break;
    case BEH_PANIC:  tick_panic (s, dt); break;
    case BEH_GATHER: tick_gather(s, dt); break;
    case BEH_FOLLOW: tick_follow(s, dt); break;
    case BEH_QUEUE:  tick_queue (s, dt); break;
    default: break;
    }
}

/*
 * scene_draw — render the current scene into WINDOW *w.
 *
 * alpha ∈ [0, 1) is the sub-tick interpolation factor from §8.
 * Each person's drawn position lerps between prev_pos and pos by alpha,
 * eliminating the micro-stutter that would appear if we always drew at
 * the last physics tick position.
 *
 *   draw_pos = prev_pos + (pos − prev_pos) × alpha
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* ── PANIC: roaming threat marker ── */
    if (s->behaviour == BEH_PANIC) {
        int tx = px_to_cell_x(s->threat_pos.x);
        int ty = px_to_cell_y(s->threat_pos.y);
        if (tx >= 0 && tx < cols && ty >= 0 && ty < rows) {
            wattron(w, COLOR_PAIR(1) | A_BOLD | A_BLINK);
            mvwaddch(w, ty, tx, '!');
            wattroff(w, COLOR_PAIR(1) | A_BOLD | A_BLINK);
        }
    }

    /* ── QUEUE: service counter on the right edge ── */
    if (s->behaviour == BEH_QUEUE) {
        int qcol = cols - 2;
        int qrow = rows / 2;
        wattron(w, COLOR_PAIR(3) | A_BOLD);
        for (int r = qrow-3; r <= qrow+3; r++)
            if (r >= 0 && r < rows) mvwaddch(w, r, qcol, '|');
        if (qrow >= 0 && qrow < rows && qcol >= 3)
            mvwprintw(w, qrow, qcol-3, ">>|");
        wattroff(w, COLOR_PAIR(3) | A_BOLD);
    }

    /* ── FOLLOW: leader '@' drawn with underline to stand out ── */
    if (s->behaviour == BEH_FOLLOW && s->count > 0) {
        const Person *ldr = &s->people[0];
        Vec2 dp = v2add(ldr->prev_pos,
                        v2scale(v2sub(ldr->pos, ldr->prev_pos), alpha));
        int lx = px_to_cell_x(dp.x), ly = px_to_cell_y(dp.y);
        if (lx >= 0 && lx < cols && ly >= 0 && ly < rows) {
            wattron(w, COLOR_PAIR(3) | A_BOLD | A_UNDERLINE);
            mvwaddch(w, ly, lx, '@');
            wattroff(w, COLOR_PAIR(3) | A_BOLD | A_UNDERLINE);
        }
    }

    /* ── all active people ── */
    int start = (s->behaviour == BEH_FOLLOW) ? 1 : 0; /* leader already drawn */
    for (int i = start; i < s->count; i++) {
        const Person *p = &s->people[i];
        Vec2 dp = v2add(p->prev_pos,
                        v2scale(v2sub(p->pos, p->prev_pos), alpha));
        int cx = px_to_cell_x(dp.x), cy = px_to_cell_y(dp.y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(p->color));
        mvwaddch(w, cy, cx, (chtype)(unsigned char)p->glyph);
        wattroff(w, COLOR_PAIR(p->color));
    }
}

/* ===================================================================== */
/* §8  app — screen, signals, input, main loop                          */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);   /* non-blocking getch */
    keypad(stdscr, TRUE);
    typeahead(-1);            /* don't let ncurses interrupt output to read ahead */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();     /* forces ncurses to re-read LINES/COLS from kernel */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — build a complete frame into stdscr (ncurses newscr buffer).
 *
 * Order matters:
 *   erase()      — blank the frame (stale content becomes spaces)
 *   scene_draw() — animation content
 *   HUD          — drawn last so it always appears on top of the scene
 *
 * Nothing reaches the terminal until screen_present() calls doupdate().
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    /* status bar — top right */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %.1f fps  sim:%d Hz  n:%d  [%s]%s ",
             fps, sim_fps, sc->count,
             BEH_NAMES[sc->behaviour],
             sc->paused ? "  PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* key hint bar — bottom */
    attron(COLOR_PAIR(3));
    mvprintw(s->rows-1, 0,
             " q:quit  spc:pause  1:wander 2:flock 3:panic 4:gather 5:follow 6:queue  +/-:people  r:reset ");
    attroff(COLOR_PAIR(3));
}

/*
 * screen_present — send only the changed cells to the terminal (one write).
 *
 * wnoutrefresh(stdscr) — copy stdscr content into ncurses' newscr model.
 * doupdate()           — diff newscr vs curscr, send only deltas to fd.
 * Never call refresh() instead: it flushes immediately and can produce
 * torn partial frames visible as flicker.
 */
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── App — top-level state ─────────────────────────────────────────── */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    float ww = pw(app->screen.cols), wh = ph(app->screen.rows);
    /* clamp all people into the new (possibly smaller) world */
    for (int i = 0; i < app->scene.count; i++) {
        Person *p = &app->scene.people[i];
        clamp_pos(&p->pos, ww, wh);
        p->prev_pos = p->pos;
    }
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  sc->paused    = !sc->paused;    break;
    case '1':  sc->behaviour = BEH_WANDER;     break;
    case '2':  sc->behaviour = BEH_FLOCK;      break;
    case '3':  sc->behaviour = BEH_PANIC;      break;
    case '4':  sc->behaviour = BEH_GATHER;     break;
    case '5':  sc->behaviour = BEH_FOLLOW;     break;
    case '6':  sc->behaviour = BEH_QUEUE;      break;
    case '+': case '=':
        sc->count += CROWD_STEP;
        if (sc->count > CROWD_MAX) sc->count = CROWD_MAX;
        break;
    case '-':
        sc->count -= CROWD_STEP;
        if (sc->count < CROWD_MIN) sc->count = CROWD_MIN;
        break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    default: break;
    }
    return true;
}

/*
 * main — the game loop (identical structure to framework.c).
 *
 * Every iteration:
 *   ① measure dt since last frame (capped at 100 ms to guard against
 *     physics avalanche after a debugger pause or Ctrl-Z)
 *   ② drain sim accumulator: add dt, fire fixed-step ticks until empty
 *   ③ compute alpha = leftover ns / tick_ns  ∈ [0, 1)
 *   ④ sleep to cap at TARGET_FPS BEFORE rendering (not after)
 *   ⑤ erase → draw scene → draw HUD → wnoutrefresh → doupdate
 *   ⑥ poll input (non-blocking getch)
 *
 * The fixed-step accumulator in ② decouples physics rate from render
 * rate.  Physics always advances at exactly sim_fps Hz regardless of
 * how fast or slow the terminal can render.
 */
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

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ① resize */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ② dt */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ③ fixed-step accumulator */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ④ alpha: how far into the next tick we are */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* smoothed fps counter (500 ms window) */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ⑤ sleep before render to keep the budget honest */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ⑦ input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
