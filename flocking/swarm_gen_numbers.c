/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * swarm_gen_numbers.c — a swarm of steered ASCII particles that flock
 * together to spell out the digits 0-9.  Each agent gets pulled toward a
 * target point in the digit's shape while ten switchable "strategies"
 * give that same formation ten different moods (drift, rush, flock, ...).
 *
 * Steering forces follow Reynolds, "Steering Behaviors for Autonomous
 * Characters" (red3d.com/cwr/steer/) and "Flocks, Herds, and Schools"
 * (SIGGRAPH 1987).
 */

#define _POSIX_C_SOURCE 200809L
#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable knobs: strategy presets, digit bitmaps, sizes ── */

/*
 * StrategyParams — one row of tuning numbers that defines a single
 * "strategy" (one steering personality).  Every strategy function reads
 * its coefficients from one of these; the scene caches a pointer to the
 * active one so switching strategies takes effect on the next tick.
 *
 * Fields:
 *   name            label shown in the HUD ("DRIFT", "RUSH", ...)
 *   max_speed       fastest an agent may move, in pixels per second
 *   arrive_speed    target speed when heading toward a slot (px/s)
 *   slow_radius     distance from the slot where agents start braking (px)
 *   slot_weight     how hard the pull-to-slot force is multiplied
 *   wander_strength size of the random drift force (0 = no wandering)
 *   sep_radius      personal-space bubble; agents inside it push apart (px)
 *   sep_weight      how hard the push-apart force is multiplied
 *   cohesion_weight FLOCK only: pull toward the average of nearby agents
 *   align_weight    FLOCK only: match the average heading of nearby agents
 *   neighbor_radius FLOCK only: how far "nearby" reaches, for the two above (px)
 */
typedef struct {
    const char *name;
    float max_speed;
    float arrive_speed;
    float slow_radius;
    float slot_weight;
    float wander_strength;
    float sep_radius;
    float sep_weight;
    float cohesion_weight;
    float align_weight;
    float neighbor_radius;
} StrategyParams;

/*
 * Ten presets, each a distinct look:
 *   DRIFT   wander gently, drift slowly into the digit (dreamy)
 *   RUSH    sprint straight to the slot, brake on arrival (snappy)
 *   FLOW    ride a rightward current, then settle when aligned (painting)
 *   ORBIT   spiral inward around the digit's centre (galaxy collapse)
 *   FLOCK   full boids (cohere + align + separate) morphing into shape
 *   PULSE   form the digit, then breathe in and out
 *   VORTEX  each agent spirals around its own slot (many whirlpools)
 *   GRAVITY constant downward pull; agents rain into place
 *   SPRING  bouncy springs that overshoot the slot before settling
 *   WAVE    snake side-to-side on the way in, settle straight at the slot
 *
 * Tuning rule of thumb: the pull-to-slot force (arrive_speed × slot_weight)
 * must clearly beat the sum of the others, or the digit won't hold its
 * shape in dense clusters.  Separation uses a fixed 60 px/s base so its
 * strength stays comparable across strategies regardless of arrive_speed.
 */
#define N_STRATEGIES  10

static const StrategyParams g_presets[N_STRATEGIES] = {
    /*          max   arr   slow  slotW wand  sepR  sepW  coh  aln  nbr  */

    /* DRIFT: meandering path, arrives slowly — wander is 10% of slot force */
    { "DRIFT",   70.0f, 70.0f, 55.0f, 2.5f, 18.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* RUSH: fast sprint, arrive deceleration prevents overshoot */
    { "RUSH",   200.0f,180.0f, 80.0f, 6.0f,  0.0f, 14.0f, 0.6f, 0.0f, 0.0f,  0.0f },

    /* FLOW: rightward stream; strong slot force takes over when aligned */
    { "FLOW",   120.0f,100.0f, 70.0f, 4.0f, 10.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* ORBIT: slot force (110×4.5=495) >> ORBIT_STRENGTH (40) → spiral in */
    { "ORBIT",  130.0f,110.0f, 90.0f, 4.5f,  0.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* FLOCK: boid forces shape the group; slot force (70×3.0=210) guides it */
    { "FLOCK",   90.0f, 70.0f, 80.0f, 3.0f, 12.0f, 18.0f, 1.2f, 0.9f, 0.5f, 80.0f },

    /* PULSE: strong slot keep (140×5.0=700) with oscillating target */
    { "PULSE",  160.0f,140.0f, 50.0f, 5.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* VORTEX: per-slot spiral (120×4.0=480) >> VORTEX_STRENGTH (35) */
    { "VORTEX", 150.0f,120.0f, 80.0f, 4.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* GRAVITY: slot force (150×5.0=750) >> GRAVITY_PULL (60) */
    { "GRAVITY",180.0f,150.0f, 60.0f, 5.0f,  0.0f, 14.0f, 0.6f, 0.0f, 0.0f,  0.0f },

    /* SPRING: spring stiffness/damping tuned to overshoot and bounce a bit */
    { "SPRING", 160.0f,130.0f, 70.0f, 4.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* WAVE: slot force (110×4.5=495) >> max wave amplitude (40) */
    { "WAVE",   140.0f,110.0f, 65.0f, 4.5f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },
};

/*
 * Digit bitmaps — small 5-wide by 7-tall pictures of each digit.
 * Each '#' becomes one target point (a Slot) in the formed digit; spaces
 * are ignored.  The '#' counts per digit (0->16 ... 7->11 ... 8->17) tell
 * you how many slots a digit needs, which is why N_AGENTS is sized to the
 * busiest one.
 */
#define DIGIT_NCOLS  5   /* template columns */
#define DIGIT_NROWS  7   /* template rows    */

static const char *const DIGIT_BITMAPS[10][DIGIT_NROWS] = {
    { " ### ", "#   #", "#   #", "#   #", "#   #", "#   #", " ### " }, /* 0 */
    { "  #  ", " ##  ", "  #  ", "  #  ", "  #  ", "  #  ", "#####" }, /* 1 */
    { " ### ", "#   #", "    #", "  ## ", " #   ", "#    ", "#####" }, /* 2 */
    { " ####", "    #", "    #", "  ###", "    #", "    #", " ####" }, /* 3 */
    { "#   #", "#   #", "#   #", "#####", "    #", "    #", "    #" }, /* 4 */
    { "#####", "#    ", "#    ", "#### ", "    #", "    #", "#### " }, /* 5 */
    { " ### ", "#    ", "#    ", "#### ", "#   #", "#   #", " ### " }, /* 6 */
    { "#####", "    #", "   # ", "  #  ", " #   ", "#    ", "#    " }, /* 7 */
    { " ### ", "#   #", "#   #", " ### ", "#   #", "#   #", " ### " }, /* 8 */
    { " ### ", "#   #", "#   #", " ####", "    #", "    #", " ### " }, /* 9 */
};

/*
 * Each '#' in the bitmap is blown up to a DIGIT_CELL_W by DIGIT_CELL_H
 * block of terminal cells.  The height is 3 (matching SLOT_GRID below) on
 * purpose: it gives each of the 3 sub-slot rows its own terminal row.  At
 * height 2 the third sub-row rounded into the next bitmap row and smeared
 * the digit vertically — the old "6 looks like 5" bug.
 */
#define DIGIT_CELL_W   5   /* terminal columns per template column */
#define DIGIT_CELL_H   3   /* terminal rows per template row       */

/*
 * Simulation sizes.
 *
 * SLOT_GRID  — each '#' is split into a 3x3 grid of finer target points,
 *              so the formed digit reads as a packed cluster of glyphs
 *              rather than a sparse outline (9 slots per '#').
 * SLOTS_MAX  — biggest possible slot count (busiest digit, 17 '#' x 9 = 153,
 *              rounded up); also the cap on N_AGENTS.
 * N_AGENTS   — total agents.  Set equal to SLOTS_MAX so even the busiest
 *              digit fills completely.  On sparser digits the leftover
 *              agents just wander dimly around the shape, which looks fine.
 * N_COLORS   — number of agent colour buckets; agent i uses bucket i % 7.
 */
enum {
    SLOT_GRID       =   3,   /* 3x3 sub-slot grid per '#' = 9 slots/cell */
    SLOTS_MAX       = 160,   /* 17 # × 9 sub-slots = 153, plus margin    */
    N_AGENTS        = 160,   /* matches SLOTS_MAX so every slot fills    */
    SIM_FPS_DEFAULT =  60,
    TARGET_FPS      =  60,
    FPS_UPDATE_MS   = 500,
    N_COLORS        =   7,
    AUTO_CYCLE_S    =   3,    /* seconds per digit in auto-cycle mode */

    PAIR_HUD        =   8,    /* bright yellow — top status bar  */
    PAIR_HINT       =   9,    /* bright cyan   — bottom key hint */

    N_THEMES        =  10,    /* see THEMES[] in §3 */
};

/* Cell dimensions — physics in px, draw in cells; convert only at render */
#define CELL_W   8
#define CELL_H  16

/* Agent is "at its slot" when closer than this; rendered bold */
#define AT_SLOT_DIST     14.0f

/* Wander: max angular change per second (radians); keeps turns smooth */
#define WANDER_TURN_MAX   5.0f

/* ORBIT: tangential speed around the digit centroid.
 * Must be weaker than the slot force (arrive_speed × slot_weight)
 * so agents eventually settle rather than orbiting forever. */
#define ORBIT_STRENGTH   40.0f

/* FLOW: rightward bias speed and x-distance threshold for switching
 * from flow to slot-arrive.  Once within FLOW_X_THRESH px of the
 * slot's x-coordinate, the agent switches to pure slot-arrive. */
#define FLOW_BIAS        50.0f
#define FLOW_X_THRESH    48.0f

/* PULSE: the digit breathes at PULSE_FREQ Hz with PULSE_AMPLITUDE px */
#define PULSE_FREQ        1.2f
#define PULSE_AMPLITUDE  55.0f
#define TWO_PI            6.28318530f

/* VORTEX: tangential speed around each agent's OWN slot.
 * Scaled by distance so vortex → 0 as agent reaches the slot. */
#define VORTEX_STRENGTH  35.0f
#define VORTEX_FADE_DIST 40.0f  /* px: full strength at this distance */

/* GRAVITY: constant downward acceleration (px/s²) */
#define GRAVITY_PULL     60.0f

/* SPRING: stiffness and damping of the bouncy spring.  These values are
 * "underdamped" — the agent overshoots its slot and rings back a couple
 * of times before settling, which is the springy look we want. */
#define SPRING_K          3.5f
#define SPRING_DAMP       2.0f

/* WAVE: lateral sinusoidal oscillation perpendicular to approach.
 * Amplitude fades linearly to 0 inside WAVE_FADE_DIST so agents
 * settle cleanly without perpetual wiggling at the slot. */
#define WAVE_AMPLITUDE   40.0f
#define WAVE_FREQ         2.0f
#define WAVE_FADE_DIST   50.0f  /* px: wave is zero below this distance */

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §2 clock — monotonic nanosecond timer and a sleep helper ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — agent palette themes plus fixed HUD colour pairs ── */

/*
 * Theme — one named colour palette for the swarm.  Agents are split into
 * N_COLORS "colour buckets" at spawn; switching themes just re-points
 * those buckets at a new set of foreground colours, so every agent
 * recolours instantly without being touched individually.
 *
 * Members:
 *   name    label shown in the HUD ("Rainbow", "Ocean", ...).
 *   body[]  one xterm-256 colour per bucket.  Every value is kept at 24
 *           or higher: lower colours are too dark to see when a wandering
 *           agent is drawn dim (per CLAUDE.md's palette-brightness rule).
 *
 * The HUD colours (PAIR_HUD / PAIR_HINT) live outside Theme on purpose so
 * the status bar stays a steady bright yellow/cyan no matter which theme
 * is active.
 */
typedef struct {
    const char *name;
    int         body[N_COLORS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name       colour for bucket 0 .. bucket 6 */
    {"Rainbow", { 196, 208, 226,  46,  51,  33, 201}},
    {"Solar",   { 226, 220, 214, 208, 202, 196, 160}},
    {"Ocean",   {  24,  27,  33,  38,  45,  51, 117}},
    {"Fire",    { 196, 202, 208, 214, 220, 226, 227}},
    {"Matrix",  {  28,  34,  40,  46,  76,  82, 118}},
    {"Aurora",  {  28,  34,  79, 122, 159, 165, 201}},
    {"Neon",    { 201, 165, 129,  93,  57,  51,  45}},
    {"Sunset",  {  54,  91, 128, 165, 202, 209, 208}},
    {"Toxic",   {  28,  58,  64,  70,  76,  82, 118}},
    {"Ghost",   { 244, 247, 250, 252, 253, 254, 255}},
};

/* Load theme `idx` into ncurses.  Background -1 means "keep the
 * terminal's own background".  On 8-colour terminals it falls back to a
 * rough warm-to-cool approximation. */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_COLORS] = {
            COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA
        };
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/* One-time colour setup: turn on ncurses colour, load the starting theme,
 * and fix the HUD pairs to bright yellow/cyan (theme-independent). */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ── §4 coords & vec2 — pixel/cell conversion and a 2-D vector type ── */

static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * Vec2 — a 2-D point or arrow (x, y), used for everything spatial:
 * positions, velocities, forces, offsets.  The little v2* helpers below
 * return new Vec2s by value so force math reads like a clean equation.
 *
 * Members:
 *   x, y    horizontal and vertical components, always in pixel units.
 *           Only the renderer ever converts these to terminal cells.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)      { return (Vec2){x, y};           }
static inline Vec2  v2add(Vec2 a, Vec2 b)      { return v2(a.x+b.x, a.y+b.y);  }
static inline Vec2  v2sub(Vec2 a, Vec2 b)      { return v2(a.x-b.x, a.y-b.y);  }
static inline Vec2  v2scale(Vec2 v, float s)   { return v2(v.x*s,   v.y*s);    }
static inline float v2len(Vec2 v)              { return sqrtf(v.x*v.x+v.y*v.y); }
static inline float v2len2(Vec2 v)             { return v.x*v.x + v.y*v.y;     }

static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* bounce_pos — elastic wall bounce; velocity component flips on contact */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)  { pos->x = 0;    vel->x =  fabsf(vel->x); }
    if (pos->x >= ww) { pos->x = ww-1; vel->x = -fabsf(vel->x); }
    if (pos->y <  0)  { pos->y = 0;    vel->y =  fabsf(vel->y); }
    if (pos->y >= wh) { pos->y = wh-1; vel->y = -fabsf(vel->y); }
}

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ── §5 entity — the Agent and Slot data, plus how an agent moves ── */

/*
 * Agent — one moving particle in the swarm.  Each tick it adds up the
 * active strategy's forces, speeds up or slows down, and moves.
 *
 * Members:
 *   pos          where it is now, in pixels.  Bounced off world edges.
 *   prev_pos     where it was at the start of this tick.  The renderer
 *                blends between prev_pos and pos so motion looks smooth
 *                even though physics only updates at a fixed rate.
 *   vel          how fast and which way it's moving, in pixels/second.
 *                Capped at the strategy's max_speed.
 *   wander_angle the direction it's currently drifting (radians).  It
 *                turns only a little each tick so wandering curves
 *                gently instead of jittering.
 *   slot_idx     which target slot it's heading for, or -1 if it has
 *                none (more agents than the digit needs) and just wanders.
 *   glyph        the character drawn for it; fixed at spawn so you can
 *                follow one agent across the screen.
 *   color_pair   its colour bucket (1..N_COLORS); fixed at spawn.
 */
typedef struct {
    /* kinematic state */
    Vec2  pos;
    Vec2  prev_pos;
    Vec2  vel;

    /* steering state */
    float wander_angle;

    /* assignment state */
    int   slot_idx;

    /* visual state (fixed at spawn) */
    char  glyph;
    int   color_pair;
} Agent;

/*
 * Slot — one target point an agent should fly to when forming the digit.
 * Picking a digit fills an array of these from its bitmap; then each
 * agent claims the nearest still-free slot.
 *
 * Members:
 *   pos        where this target sits on screen, in pixels.
 *   occupied   true once an agent has claimed it.  The claim flag is what
 *              stops two agents fighting over the same slot.  Cleared
 *              whenever the digit changes.
 */
typedef struct {
    Vec2 pos;
    bool occupied;
} Slot;

/* The characters agents are drawn with; each agent keeps one for life. */
static const char AGENT_GLYPHS[] = "*@+#oO0x~=";

/* Drop one agent at a random spot.  Its glyph and colour are picked here
 * and never change, so each agent stays recognisable. */
static void agent_spawn(Agent *a, int id, float ww, float wh)
{
    a->pos          = v2(randf() * ww, randf() * wh);
    a->prev_pos     = a->pos;
    a->vel          = v2(0, 0);
    a->wander_angle = randf() * 6.2832f;   /* random start direction */
    a->slot_idx     = -1;
    a->glyph        = AGENT_GLYPHS[id % (int)(sizeof(AGENT_GLYPHS) - 1)];
    a->color_pair   = (id % N_COLORS) + 1;
}

/* Apply one step of motion: nudge the velocity by the force (capped at
 * max_speed), remember the old position, then move. */
static void agent_step(Agent *a, Vec2 force, float max_speed, float dt)
{
    a->vel      = v2clamp_len(v2add(a->vel, v2scale(force, dt)), max_speed);
    a->prev_pos = a->pos;
    a->pos      = v2add(a->pos, v2scale(a->vel, dt));
}

/* Every strategy ends the same way: move the agent, then bounce it back
 * if it left the screen.  Bundled here so all ten strategies share one
 * line and any future change (say, wrap instead of bounce) is one edit. */
static void agent_commit_force(Agent *a, Vec2 force,
                                const StrategyParams *sp,
                                float dt, float ww, float wh)
{
    agent_step(a, force, sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
}

/* ── §6 steering forces — the building-block urges agents can feel ── */

/* Steer straight at a target at a chosen speed.  It returns the *change*
 * in velocity needed, so an agent already cruising the right way feels no
 * extra push. */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
    return v2sub(desired, vel);
}

/* Like seek, but ease off the gas once inside slow_radius so the agent
 * coasts to a stop on the target instead of overshooting and bouncing. */
static Vec2 steer_arrive(Vec2 pos, Vec2 vel, Vec2 target,
                          float max_speed, float slow_radius)
{
    Vec2  to_tgt       = v2sub(target, pos);
    float dist         = v2len(to_tgt);
    float desired_spd  = (dist < slow_radius)
                         ? max_speed * (dist / slow_radius)
                         : max_speed;
    Vec2  desired      = v2scale(v2norm(to_tgt), desired_spd);
    return v2sub(desired, vel);
}

/*
 * Push away from any agent inside the personal-space radius, harder the
 * closer they are.  The base push is a fixed 60 px/s on purpose: an
 * earlier version scaled it with each strategy's arrive_speed, which let
 * fast strategies blow the swarm apart.  A fixed base means sep_weight is
 * the only knob you ever need to touch.
 */
#define SEP_BASE_FORCE  60.0f   /* fixed repulsion magnitude (px/s) */

static Vec2 steer_separate(const Agent *agents, int n_agents, int self,
                            const StrategyParams *sp)
{
    Vec2 force = v2(0, 0);
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        Vec2  away = v2sub(agents[self].pos, agents[i].pos);
        float d    = v2len(away);
        if (d < sp->sep_radius && d > 0.001f) {
            float strength = (sp->sep_radius - d) / sp->sep_radius;
            force = v2add(force,
                          v2scale(v2norm(away), strength * SEP_BASE_FORCE));
        }
    }
    return force;
}

/* A gentle random drift.  The wander direction only turns a little each
 * tick, so the path curves naturally instead of twitching. */
static Vec2 steer_wander(Agent *a, float strength, float dt)
{
    a->wander_angle += (randf() - 0.5f) * WANDER_TURN_MAX * dt;
    return v2scale(v2(cosf(a->wander_angle), sinf(a->wander_angle)), strength);
}

/* Head toward the average position of nearby agents — the urge that keeps
 * a flock clumped together.  Used only by FLOCK. */
static Vec2 steer_cohesion(const Agent *agents, int n_agents, int self,
                             const StrategyParams *sp, float speed)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > sp->neighbor_radius)
            continue;
        sum = v2add(sum, agents[i].pos);
        n++;
    }
    if (n == 0) return v2(0, 0);
    Vec2 local_centre = v2scale(sum, 1.0f / n);
    return steer_seek(agents[self].pos, agents[self].vel, local_centre, speed);
}

/* Match the average heading of nearby agents — the urge that makes a
 * flock fly in the same direction.  Used only by FLOCK. */
static Vec2 steer_align(const Agent *agents, int n_agents, int self,
                          const StrategyParams *sp)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > sp->neighbor_radius)
            continue;
        sum = v2add(sum, agents[i].vel);
        n++;
    }
    if (n == 0) return v2(0, 0);
    Vec2 avg_vel = v2scale(sum, 1.0f / n);
    return v2sub(avg_vel, agents[self].vel);
}

/* A rubber-band pull toward the target, fought by a drag that grows with
 * speed.  With the file's stiffness/damping the agent overshoots and rings
 * back a couple of times before it settles (the springy look). */
static Vec2 steer_spring(Vec2 pos, Vec2 vel, Vec2 target, float k, float damping)
{
    Vec2 spring_pull = v2scale(v2sub(target, pos), k);
    Vec2 damp_drag   = v2scale(vel, damping);
    return v2sub(spring_pull, damp_drag);
}

/* ── §7 strategy tick functions — the ten formation personalities ── */

/*
 * Each strategy is one "personality": it blends the slot pull, the
 * push-apart force, and its own special ingredient, then hands the total
 * to agent_commit_force.  All ten read their numbers from the same
 * StrategyParams pointer, so the only difference between them is the
 * special ingredient.
 */

/* Return the pull-to-slot force (already weighted), or nothing if this
 * agent has no slot.  Nine of the ten strategies start with this; SPRING
 * is the exception, using its rubber-band force instead. */
static Vec2 agent_arrive_at_slot_if_assigned(const Agent *a,
                                              const Slot *slots,
                                              const StrategyParams *sp)
{
    if (a->slot_idx < 0) return v2(0, 0);

    Vec2 arrive = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                                sp->arrive_speed, sp->slow_radius);
    return v2scale(arrive, sp->slot_weight);
}

/* ── §7.1 DRIFT — wander gently while drifting into the digit ── */
/* The wandering force fades to nothing as the agent nears its slot.
 * Otherwise it would keep nudging arrived agents back out and the digit
 * would never sharpen up. */
#define WANDER_FADE_DIST  55.0f   /* px: wander is zero below this distance to slot */

/* Returns 1 when far from the slot and ramps down to 0 right at it, so
 * wandering quiets down once an agent has arrived.  Used by DRIFT and
 * FLOCK. */
static float wander_fade_scale_for_slot(const Agent *a, const Slot *slots)
{
    if (a->slot_idx < 0) return 1.0f;
    float d = v2len(v2sub(slots[a->slot_idx].pos, a->pos));
    return (d >= WANDER_FADE_DIST) ? 1.0f : d / WANDER_FADE_DIST;
}

static void strategy_drift(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    Vec2  sep_force    = steer_separate(agents, n_agents, self, sp);
    float wander_scale = wander_fade_scale_for_slot(a, slots);
    Vec2  wander_force = steer_wander(a, sp->wander_strength * wander_scale, dt);
    Vec2  slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);

    Vec2 force = v2add(v2add(wander_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.2 RUSH — sprint straight to the slot and brake on arrival ── */
#define RUSH_NO_SLOT_DAMPING  0.90f   /* per-tick velocity multiplier when unassigned */

static void strategy_rush(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Unassigned agents glide to a halt (no force will push them) */
    if (a->slot_idx < 0) a->vel = v2scale(a->vel, RUSH_NO_SLOT_DAMPING);

    Vec2 sep_force  = steer_separate(agents, n_agents, self, sp);
    Vec2 slot_force = agent_arrive_at_slot_if_assigned(a, slots, sp);

    Vec2 force = v2add(slot_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.3 FLOW — ride a rightward current, settle once aligned ── */
/* A rightward current carries agents across the screen; when one gets
 * close to its slot's column the current gives way to the slot pull, so
 * the digit "paints" itself left to right.
 *
 * These three scales tune the handoff: weaken the slot pull while still in
 * the current, weaken wander once aligned, and run the current at reduced
 * strength for agents that have no slot. */
#define FLOW_FAR_ARRIVE_SCALE   0.3f   /* slot pull while still in the current */
#define FLOW_NEAR_WANDER_SCALE  0.4f   /* wander once aligned with slot column */
#define FLOW_NO_SLOT_BIAS_SCALE 0.6f   /* current strength for unassigned agents */

static void strategy_flow(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2   task_force;

    if (a->slot_idx < 0) {
        /* (a) Unassigned: ride the current at reduced strength + wander */
        Vec2 stream       = v2(FLOW_BIAS * FLOW_NO_SLOT_BIAS_SCALE, 0.0f);
        Vec2 wander_force = steer_wander(a, sp->wander_strength, dt);
        task_force        = v2add(stream, wander_force);

    } else {
        Vec2  slot_pos = slots[a->slot_idx].pos;
        float dx_to_slot = slot_pos.x - a->pos.x;
        bool  still_in_current = dx_to_slot > FLOW_X_THRESH;

        if (still_in_current) {
            /* (b) Far left of slot: stream rightward + faint slot pull */
            Vec2 stream      = v2(FLOW_BIAS, 0.0f);
            Vec2 weak_arrive = steer_arrive(a->pos, a->vel, slot_pos,
                                             sp->arrive_speed * FLOW_FAR_ARRIVE_SCALE,
                                             sp->slow_radius);
            task_force       = v2add(stream, weak_arrive);
        } else {
            /* (c) Aligned with slot column: full slot-arrive + soft wander */
            Vec2 slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);
            Vec2 wander_force = steer_wander(a, sp->wander_strength * FLOW_NEAR_WANDER_SCALE, dt);
            task_force        = v2add(slot_force, wander_force);
        }
    }

    Vec2 force = v2add(task_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.4 ORBIT — agents spiral inward around the digit's centre ── */
/* A sideways spin around the digit's centre fights the inward slot pull,
 * so agents wind in like a collapsing galaxy. */
static void strategy_orbit(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt,
                            Vec2 digit_centroid)
{
    Agent *a = &agents[self];

    /* The orbit force points sideways around the digit's centre (at a right
     * angle to the line from centre to agent), so it spins the agent in a
     * circle while the slot pull drags it inward — a spiral collapse. */
    Vec2  radial         = v2sub(a->pos, digit_centroid);
    Vec2  tangent_dir    = v2norm(v2(-radial.y, radial.x));
    Vec2  orbit_force    = v2scale(tangent_dir, ORBIT_STRENGTH);

    Vec2  sep_force      = steer_separate(agents, n_agents, self, sp);
    Vec2  slot_force     = agent_arrive_at_slot_if_assigned(a, slots, sp);

    Vec2 force = v2add(v2add(orbit_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.5 FLOCK — real boids that morph the flock into the shape ── */
/* Cohesion, alignment and separation make the swarm move like one
 * organism; a deliberately weak slot pull lets that group motion show
 * before the digit finally takes form. */
static void strategy_flock(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    Vec2  sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2  coh_force = steer_cohesion(agents, n_agents, self, sp, sp->arrive_speed);
    Vec2  aln_force = steer_align   (agents, n_agents, self, sp);

    /* Wander quiets down near the slot so agents settle instead of drifting */
    float wander_scale = wander_fade_scale_for_slot(a, slots);
    Vec2  wander_force = steer_wander(a, sp->wander_strength * wander_scale, dt);

    Vec2  slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);

    Vec2 group_force = v2add(v2scale(coh_force, sp->cohesion_weight),
                             v2scale(aln_force, sp->align_weight));
    Vec2 personal_force = v2add(v2add(v2scale(sep_force, sp->sep_weight),
                                       slot_force),
                                wander_force);
    Vec2 force = v2add(group_force, personal_force);

    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.6 PULSE — the formed digit breathes in and out ── */
/* Each agent chases not its raw slot but a target that slides outward and
 * back on a steady rhythm, so the whole digit swells and shrinks. */
#define PULSE_NO_SLOT_DAMPING  0.94f   /* per-tick velocity decay when unassigned */
#define PULSE_DEGENERATE_EPS   0.001f  /* min |centroid→slot| before fallback dir */

static void strategy_pulse(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, int n_slots, float ww, float wh,
                            float dt, float sim_time, Vec2 digit_centroid)
{
    (void)n_slots;
    Agent *a = &agents[self];
    Vec2   sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2   task_force;

    if (a->slot_idx < 0) {
        /* (a) Unassigned: friction-only; agent drifts to rest */
        a->vel     = v2scale(a->vel, PULSE_NO_SLOT_DAMPING);
        task_force = v2(0, 0);
    } else {
        Vec2 slot_pos = slots[a->slot_idx].pos;

        /* (b) Push direction = centroid → slot (NOT agent → slot).
         *     Using the centroid-relative direction keeps it stable even
         *     when the agent sits exactly on its slot — where the
         *     agent→slot vector would be a degenerate zero. */
        Vec2 centroid_to_slot = v2sub(slot_pos, digit_centroid);
        Vec2 push_dir         = v2norm(centroid_to_slot);
        if (v2len(push_dir) < PULSE_DEGENERATE_EPS) push_dir = v2(1.0f, 0.0f);

        /* (c) Slide the target out past the slot and back on a sine wave,
         *     so it averages out to the slot itself. */
        float wave_phase     = sinf(TWO_PI * PULSE_FREQ * sim_time);
        Vec2  pulse_offset   = v2scale(push_dir, wave_phase * PULSE_AMPLITUDE);
        Vec2  oscillating_tgt = v2add(slot_pos, pulse_offset);

        /* (d) Chase that moving target.  The slot pull must out-muscle the
         *     swing or the agent lags behind and drifts off. */
        Vec2 arrive_force = steer_arrive(a->pos, a->vel, oscillating_tgt,
                                          sp->arrive_speed, sp->slow_radius);
        task_force = v2scale(arrive_force, sp->slot_weight);
    }

    Vec2 force = v2add(task_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.7 VORTEX — each agent spirals into its own slot ── */
/* Like ORBIT, but every agent spins around its own slot instead of the
 * shared centre — so the screen fills with many tiny whirlpools.  The spin
 * fades out as the agent closes in, or it would circle forever. */
static void strategy_vortex(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force    = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   vortex_force = v2(0, 0);

    if (a->slot_idx >= 0) {
        Vec2  slot_pos       = slots[a->slot_idx].pos;
        Vec2  radial         = v2sub(a->pos, slot_pos);
        float radius         = v2len(radial);

        /* Sideways direction (at a right angle to the line from slot to agent) */
        Vec2  tangent_dir    = v2norm(v2(-radial.y, radial.x));

        /* Spin at full strength far out, fading to nothing at the slot so
         * the agent stops circling and settles. */
        float fade_scale     = fminf(radius / VORTEX_FADE_DIST, 1.0f);
        vortex_force         = v2scale(tangent_dir, VORTEX_STRENGTH * fade_scale);
    }

    Vec2 force = v2add(v2add(vortex_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.8 GRAVITY — agents rain down into the digit ── */
/* A steady downward pull tilts every path.  The slot pull still wins, so
 * agents reach their slots, but they drop in from above and overshoot a
 * little, like raindrops landing. */
static void strategy_gravity(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                               const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force     = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force    = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   gravity_force = v2(0.0f, GRAVITY_PULL);     /* +y = screen-downward */

    Vec2 force = v2add(v2add(gravity_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.9 SPRING — bouncy springs snap agents into place ── */
/* A rubber-band pull replaces the usual slot steering, so agents overshoot
 * and ring back a couple of times before settling, like dropped balls. */
#define SPRING_NO_SLOT_DAMPING 0.92f   /* per-tick velocity decay when unassigned */

static void strategy_spring(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Unassigned agents have no spring to anchor them — damp to rest. */
    if (a->slot_idx < 0) a->vel = v2scale(a->vel, SPRING_NO_SLOT_DAMPING);

    Vec2 sep_force    = steer_separate(agents, n_agents, self, sp);

    /* Note SPRING ignores sp->slot_weight: the spring's stiffness and
     * damping are its tuning knobs instead. */
    Vec2 spring_force = (a->slot_idx >= 0)
        ? steer_spring(a->pos, a->vel, slots[a->slot_idx].pos, SPRING_K, SPRING_DAMP)
        : v2(0, 0);

    Vec2 force = v2add(spring_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.10 WAVE — agents snake side to side on the way in ── */
/* While heading for the slot, agents also swing left and right.  The swing
 * is small up close, grows with distance, and dies out right at the slot,
 * giving a snaking approach that settles cleanly. */
#define WAVE_RAMP_SLOPE  0.5f   /* px-of-amplitude per px-of-distance (ramp-up phase) */

static void strategy_wave(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh,
                            float dt, float sim_time)
{
    Agent *a = &agents[self];
    Vec2   sep_force  = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   wave_force = v2(0, 0);

    if (a->slot_idx >= 0) {
        /* lateral_dir is sideways: at a right angle to the line to the slot */
        Vec2  to_slot         = v2sub(slots[a->slot_idx].pos, a->pos);
        float dist_to_slot    = v2len(to_slot);
        Vec2  approach_dir    = v2norm(to_slot);
        Vec2  lateral_dir     = v2(-approach_dir.y, approach_dir.x);

        /* How wide the swing is.  Two effects multiply: it grows with
         * distance (capped) so it's gentle up close, and it's switched off
         * entirely right near the slot so the last stretch is straight. */
        float ramp_amplitude  = fminf(dist_to_slot * WAVE_RAMP_SLOPE, WAVE_AMPLITUDE);
        float fade_envelope   = fminf(dist_to_slot / WAVE_FADE_DIST, 1.0f);
        float lateral_amplitude = ramp_amplitude * fade_envelope;

        float wave_phase      = sinf(TWO_PI * WAVE_FREQ * sim_time);
        wave_force            = v2scale(lateral_dir, lateral_amplitude * wave_phase);
    }

    Vec2 force = v2add(v2add(slot_force, wave_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* Run one agent through whichever strategy is active.  centroid and
 * sim_time are passed to all of them but only a few actually use them. */
static void agent_tick(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                       const Slot *slots, int n_slots,
                       float ww, float wh, float dt,
                       int strategy, Vec2 digit_centroid, float sim_time)
{
    switch (strategy) {
    case 0: strategy_drift   (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 1: strategy_rush    (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 2: strategy_flow    (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 3: strategy_orbit   (agents, n_agents, self, sp, slots,          ww, wh, dt, digit_centroid); break;
    case 4: strategy_flock   (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 5: strategy_pulse   (agents, n_agents, self, sp, slots, n_slots, ww, wh, dt, sim_time, digit_centroid); break;
    case 6: strategy_vortex  (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 7: strategy_gravity (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 8: strategy_spring  (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 9: strategy_wave    (agents, n_agents, self, sp, slots,          ww, wh, dt, sim_time); break;
    }
}

/* ── §8 digit library — turn a digit's bitmap into a cloud of slots ── */

/*
 * DigitLayout — where the digit sits and how its sub-slots are spaced.
 * Worked out once per digit so the per-'#' loops stay cheap.
 *
 * Members:
 *   origin          top-left corner of the centred digit, in pixels.
 *   stride_x_cells  gap between sub-slots across, in terminal cells.
 *   stride_y_cells  gap between sub-slots down, in terminal cells.
 */
typedef struct {
    Vec2 origin;
    int  stride_x_cells;
    int  stride_y_cells;
} DigitLayout;

/* Work out where to place the digit so it sits centred on screen, plus the
 * spacing between sub-slots, for the current terminal size. */
static DigitLayout digit_layout_centred(int cols, int rows)
{
    const float bitmap_cell_px_w = (float)(DIGIT_CELL_W * CELL_W);
    const float bitmap_cell_px_h = (float)(DIGIT_CELL_H * CELL_H);
    const float bbox_px_w        = (float)DIGIT_NCOLS * bitmap_cell_px_w;
    const float bbox_px_h        = (float)DIGIT_NROWS * bitmap_cell_px_h;

    return (DigitLayout){
        .origin         = v2((pw(cols) - bbox_px_w) * 0.5f,
                             (ph(rows) - bbox_px_h) * 0.5f),
        .stride_x_cells = (DIGIT_CELL_W - 1) / (SLOT_GRID - 1),  /* = 2 */
        .stride_y_cells = (DIGIT_CELL_H - 1) / (SLOT_GRID - 1),  /* = 1 */
    };
}

/*
 * Lay down the 3x3 grid of sub-slots that fills one '#' of the bitmap,
 * appending them to slots[] and bumping the count.
 *
 * The sub-slots land on exact whole-cell positions on purpose.  If they
 * were placed at fractional spots, the later rounding from pixels to
 * terminal cells could bump the bottom sub-row into the next bitmap row
 * and smear the digit — the old "6 looks like 5" bug.
 */
static void emit_subslot_grid(Slot *slots, int *n, const DigitLayout *lay,
                              int cell_col0, int cell_row0)
{
    for (int sr = 0; sr < SLOT_GRID && *n < SLOTS_MAX; sr++) {
        for (int sc = 0; sc < SLOT_GRID && *n < SLOTS_MAX; sc++) {
            int sub_col = cell_col0 + sc * lay->stride_x_cells;
            int sub_row = cell_row0 + sr * lay->stride_y_cells;
            slots[*n].pos      = v2(lay->origin.x + (float)sub_col * (float)CELL_W,
                                     lay->origin.y + (float)sub_row * (float)CELL_H);
            slots[*n].occupied = false;
            (*n)++;
        }
    }
}

/* Build the full set of target slots for one digit and return how many.
 * Centre the digit, then drop a 3x3 grid of slots on every '#'. */
static int digit_load(Slot *slots, int digit, int cols, int rows)
{
    DigitLayout layout = digit_layout_centred(cols, rows);

    int n = 0;
    for (int r = 0; r < DIGIT_NROWS && n < SLOTS_MAX; r++) {
        const char *row = DIGIT_BITMAPS[digit][r];
        for (int c = 0; row[c] && c < DIGIT_NCOLS; c++) {
            if (row[c] != '#') continue;

            int cell_col0 = c * DIGIT_CELL_W;
            int cell_row0 = r * DIGIT_CELL_H;
            emit_subslot_grid(slots, &n, &layout, cell_col0, cell_row0);
        }
    }
    return n;
}

/* The middle of the formed digit (average of all its slots).  ORBIT and
 * PULSE spin/push around this point. */
static Vec2 digit_centroid(const Slot *slots, int n_slots)
{
    if (n_slots == 0) return v2(0, 0);
    Vec2 sum = v2(0, 0);
    for (int i = 0; i < n_slots; i++)
        sum = v2add(sum, slots[i].pos);
    return v2scale(sum, 1.0f / n_slots);
}

/* Hand each agent a slot to head for.  Going in order, every agent grabs
 * the nearest slot no one's taken yet; leftover agents get none (-1) and
 * just wander.  It's not the perfectly-fair pairing, but it's quick and
 * looks natural. */
static void assign_slots(Agent *agents, int n_agents,
                         Slot *slots, int n_slots)
{
    for (int s = 0; s < n_slots; s++) slots[s].occupied = false;

    for (int i = 0; i < n_agents; i++) {
        agents[i].slot_idx = -1;
        float best_d2 = FLT_MAX;
        int   best_s  = -1;
        for (int s = 0; s < n_slots; s++) {
            if (slots[s].occupied) continue;
            float d2 = v2len2(v2sub(slots[s].pos, agents[i].pos));
            if (d2 < best_d2) { best_d2 = d2; best_s = s; }
        }
        if (best_s >= 0) {
            agents[i].slot_idx    = best_s;
            slots[best_s].occupied = true;
        }
    }
}

/* ── §9 scene — all simulation state and how it advances + draws ── */

/*
 * World — the size of the play area in pixels.  Recomputed from the live
 * terminal size each tick, so a resize just flows through.
 *
 * Members:
 *   width    play-area width in pixels.
 *   height   play-area height in pixels.
 */
typedef struct {
    float width;
    float height;
} World;

/*
 * SimControls — the playback switches the keyboard toggles.
 *
 * Members:
 *   paused       when true the simulation freezes; the HUD shows PAUSED.
 *   auto_cycle   when true the demo steps to the next digit on its own.
 *   cycle_timer  seconds counted toward the next auto-advance; reset to 0
 *                whenever you pick a digit by hand.
 */
typedef struct {
    bool  paused;
    bool  auto_cycle;
    float cycle_timer;
} SimControls;

/*
 * Strategy — which steering personality is active.  It's kept as two
 * pieces that must always agree: a number (for cycling and the HUD) and a
 * pointer to that preset's tuning numbers.  They're set together so they
 * can never drift apart.
 *
 * Members:
 *   index    which preset (0..N_STRATEGIES-1).
 *   params   pointer to that preset's row in g_presets.
 */
typedef struct {
    int                          index;
    const StrategyParams        *params;
} Strategy;

/*
 * SwarmScene — the whole simulation in one struct.  Everything that
 * changes over a run lives here, so every helper just takes a SwarmScene*
 * and there's one place to look for "what state exists".
 *
 * Members:
 *   agents[]      the swarm (always all N_AGENTS of them).
 *   slots[]       the digit's target points; only the first n_slots used.
 *   n_slots       how many slots the current digit needs.
 *   current_digit which digit (0..9) is being formed.
 *   strategy      which steering personality is active.
 *   theme_idx     which colour theme is active.
 *   sim_time      seconds elapsed; drives the PULSE and WAVE rhythms.
 *   world         play-area size in pixels.
 *   sim           the pause / auto-cycle switches.
 */
typedef struct {
    Agent agents[N_AGENTS];
    Slot  slots[SLOTS_MAX];
    int   n_slots;

    int      current_digit;     /* 0..9                                 */
    Strategy strategy;          /* index + cached params pointer        */
    int      theme_idx;         /* 0..N_THEMES-1; t/T cycles            */

    float sim_time;             /* drives PULSE/WAVE oscillations       */
    World world;                /* pixel-space extent, refreshed/tick   */

    SimControls sim;
} SwarmScene;

/* Throw every agent to a random spot (the 'r' key and startup). */
static void scene_scatter(SwarmScene *s)
{
    for (int i = 0; i < N_AGENTS; i++) {
        s->agents[i].pos      = v2(randf() * s->world.width, randf() * s->world.height);
        s->agents[i].prev_pos = s->agents[i].pos;
        s->agents[i].vel      = v2(0, 0);
    }
}

/* Switch to a new digit: rebuild its slots and hand them out to agents. */
static void scene_set_digit(SwarmScene *s, int digit)
{
    s->current_digit = digit;
    s->n_slots = digit_load(s->slots, digit,
                             (int)(s->world.width / CELL_W),
                             (int)(s->world.height / CELL_H));
    assign_slots(s->agents, N_AGENTS, s->slots, s->n_slots);
}

/* Switch steering personality, updating both the index and the params
 * pointer together, and wrapping around the ends of the list. */
static void scene_set_strategy(SwarmScene *s, int new_index)
{
    new_index = ((new_index % N_STRATEGIES) + N_STRATEGIES) % N_STRATEGIES;
    s->strategy.index  = new_index;
    s->strategy.params = &g_presets[new_index];
}

/* Start fresh: clear state, size the world, pick strategy 0, scatter all
 * agents, and form digit 0. */
static void scene_init(SwarmScene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->world.width  = pw(cols);
    s->world.height = ph(rows);

    scene_set_strategy(s, 0);

    for (int i = 0; i < N_AGENTS; i++)
        agent_spawn(&s->agents[i], i, s->world.width, s->world.height);

    scene_set_digit(s, 0);
}

static void scene_tick(SwarmScene *s, float dt, int cols, int rows)
{
    s->world.width = pw(cols);
    s->world.height = ph(rows);
    if (s->sim.paused) return;

    s->sim_time    += dt;
    s->sim.cycle_timer += dt;

    /* Auto-cycle: advance to next digit every AUTO_CYCLE_S seconds */
    if (s->sim.auto_cycle && s->sim.cycle_timer >= (float)AUTO_CYCLE_S) {
        s->sim.cycle_timer = 0.0f;
        scene_set_digit(s, (s->current_digit + 1) % 10);
    }

    Vec2 centroid = digit_centroid(s->slots, s->n_slots);

    for (int i = 0; i < N_AGENTS; i++)
        agent_tick(s->agents, N_AGENTS, i, s->strategy.params,
                   s->slots, s->n_slots,
                   s->world.width, s->world.height, dt,
                   s->strategy.index, centroid, s->sim_time);
}

/* Draw one character at a terminal cell, skipping anything off-screen.
 * The double cast keeps high-byte characters from being mangled into
 * garbage by ncurses. */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* Draw every agent.  Brightness shows its state: bold once it has reached
 * its slot, dim while wandering with no slot, normal on the way there.
 * Each agent is drawn at a blended position (alpha) so motion stays smooth
 * between physics ticks. */
static void scene_draw(const SwarmScene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    for (int i = 0; i < N_AGENTS; i++) {
        const Agent *a = &s->agents[i];

        Vec2 dp = v2add(a->prev_pos,
                        v2scale(v2sub(a->pos, a->prev_pos), alpha));

        attr_t attr;
        if (a->slot_idx < 0) {
            attr = A_DIM;
        } else {
            float d = v2len(v2sub(s->slots[a->slot_idx].pos, a->pos));
            attr = (d < AT_SLOT_DIST) ? A_BOLD : A_NORMAL;
        }

        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  a->glyph, a->color_pair, attr, cols, rows);
    }
}

/* ── §10 app — ncurses setup, the HUD, input, and the main loop ── */

/*
 * Screen — the terminal's size in character cells, the rendering side of
 * the world (the simulation thinks in pixels; only the renderer converts).
 *
 * Members:
 *   cols   terminal width in cells.
 *   rows   terminal height in cells (bottom row holds the key hints).
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr(); noecho(); cbreak();
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
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* How many agents have reached their slot.  The HUD shows this as a
 * "N/total formed" progress reading for the digit. */
static int count_agents_at_slot(const SwarmScene *sc)
{
    int at_slot = 0;
    for (int i = 0; i < N_AGENTS; i++) {
        const Agent *a = &sc->agents[i];
        if (a->slot_idx < 0) continue;
        float distance_to_slot = v2len(v2sub(sc->slots[a->slot_idx].pos, a->pos));
        if (distance_to_slot < AT_SLOT_DIST) at_slot++;
    }
    return at_slot;
}

/* format_hud_status — write the top-row HUD line into `buf`.
 * Format: fps, sim rate, digit, strategy(idx/total), theme, formed-
 * counter, and an AUTO/PAUSED suffix when those modes are active. */
static void format_hud_status(const SwarmScene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    int   formed_count = count_agents_at_slot(sc);
    const char *auto_suffix   = sc->sim.auto_cycle ? "  AUTO"   : "";
    const char *paused_suffix = sc->sim.paused     ? "  PAUSED" : "";

    snprintf(buf, buflen,
             " %5.0f fps  sim:%3d Hz  digit:%d  %s (%d/%d)  [%s]  %d/%d formed%s%s ",
             fps, sim_fps,
             sc->current_digit,
             sc->strategy.params->name, sc->strategy.index + 1, N_STRATEGIES,
             THEMES[sc->theme_idx].name,
             formed_count, sc->n_slots,
             auto_suffix, paused_suffix);
}

/* hud_paint_text — attron / mvprintw / attroff sandwich shared by both
 * HUD rows so the colour-pair setup isn't duplicated per call. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_hud_status — right-aligned status on row 0 (PAIR_HUD yellow) */
static void draw_hud_status(const Screen *s, const SwarmScene *sc,
                             double fps, int sim_fps)
{
    enum { HUD_TOP_ROW = 0 };
    char buf[120];
    format_hud_status(sc, fps, sim_fps, buf, sizeof buf);

    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan) */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:pause  0-9:digit  n/p:strategy  t/T:theme  "
        "a:auto  r:scatter ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const SwarmScene *sc,
                        double fps, int sim_fps, float alpha)
{
    /* (1) clear offscreen buffer */
    erase();
    /* (2) paint the simulation (slots + agents) */
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);
    /* (3) HUD on top — data row + action row */
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * FpsCounter — a smoothed frames-per-second reading.  Counting per frame
 * would jitter, so it tallies frames over a short window and only updates
 * the displayed number when the window fills.
 *
 * Members:
 *   frame_count  frames seen so far this window.
 *   window_ns    nanoseconds elapsed so far this window.
 *   display      the last computed fps, shown in the HUD.
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

static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — the whole program in one struct.
 *
 * Members:
 *   scene        the simulation.
 *   screen       terminal size and ncurses setup.
 *   fps          the fps reading for the HUD.
 *   sim_fps      physics tick rate (fixed, not user-adjustable).
 *   running      cleared to stop the loop (by Ctrl-C, kill, or 'q').
 *   need_resize  set when the terminal is resized; handled next frame.
 */
typedef struct {
    SwarmScene            scene;
    Screen                screen;
    FpsCounter            fps;
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
    app->need_resize = 0;
    /* Reload digit slots to fit new terminal size */
    scene_set_digit(&app->scene, app->scene.current_digit);
}

/* Act on one keypress.  Returns false only for quit (q/ESC), which stops
 * the loop; every other key returns true. */
static bool app_handle_key(App *app, int ch)
{
    SwarmScene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sc->sim.paused = !sc->sim.paused;             break;
    case 'a': case 'A':
        sc->sim.auto_cycle  = !sc->sim.auto_cycle;
        sc->sim.cycle_timer = 0.0f;
        break;
    case 'r': case 'R':
        scene_scatter(sc);
        assign_slots(sc->agents, N_AGENTS, sc->slots, sc->n_slots);
        break;
    case 'n': case 'N':
        scene_set_strategy(sc, sc->strategy.index + 1);
        break;
    case 'p': case 'P':
        scene_set_strategy(sc, sc->strategy.index - 1);
        break;
    case 't':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        /* Step back a theme; add N-1 first so we never take a negative modulo */
        sc->theme_idx = (sc->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        scene_set_digit(sc, ch - '0');
        sc->sim.cycle_timer = 0.0f;
        break;
    default: break;
    }
    return true;
}

/* The main loop.  Physics runs in fixed-size steps from a time budget so
 * it stays the same speed on any machine, while the screen can be drawn
 * at whatever rate fits.  (Pattern: Fiedler, "Fix Your Timestep!".) */
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

    screen_init(&app->screen, 0 /* initial theme = Rainbow */);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;         /* avalanche guard */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS; /* render cadence  */
    const int64_t TICK_LEN_NS     = TICK_NS(app->sim_fps);
    const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* (1) handle SIGWINCH */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) measure dt, capped to avoid avalanche */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) drain accumulator: fixed-step physics until caught up */
        sim_accum += dt;
        while (sim_accum >= TICK_LEN_NS) {
            scene_tick(&app->scene, TICK_LEN_SEC,
                       app->screen.cols, app->screen.rows);
            sim_accum -= TICK_LEN_NS;
        }

        /* (4) sub-tick alpha for renderer */
        float alpha = (float)sim_accum / (float)TICK_LEN_NS;

        /* (5) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt);

        /* (6) sleep BEFORE render so terminal I/O stays inside budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* (7) draw + present */
        screen_draw(&app->screen, &app->scene,
                    app->fps.display, app->sim_fps, alpha);
        screen_present();

        /* (8) drain input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
