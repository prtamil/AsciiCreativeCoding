/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * moving_jump_spring_leg_robot.c — a one-legged pogo-stick robot that
 * loads a spring, launches into a parabolic hop, lands on procedural
 * terrain, and does it again forever while the camera follows along.
 *
 * The spring-mass hopping model comes from Raibert, "Legged Robots
 * That Balance" (MIT Press 1986); the rolling terrain uses Quilez's
 * value noise (https://iquilezles.org/articles/noise/).
 * Sister files: robots/diff_drive_robot.c (a wheeled robot for
 * contrast), animation/hexpod_tripod.c (six legs instead of one).
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — every tunable in one place ── */

enum { TARGET_FPS = 60 };

/* The physics runs in a fine "pixel" grid (8 across, 16 down per
 * terminal cell) so the math doesn't care that terminal cells are
 * about twice as tall as they are wide. We only convert to cells when
 * it's time to draw. */
#define CELL_W   8
#define CELL_H  16

/* The leg spring, in pixels. SPRING_REST is how tall the leg stands
 * unloaded; SPRING_COMPRESS_MAX is how far the body sinks when fully
 * loaded; SPRING_K is how stiff it is, which sets how fast we launch
 * (full load gives 48·√25 = 240 px/sec). BODY_HALF_H is half the body
 * glyph's height, used to decide when it touches the ground. */
#define SPRING_REST          80.0f
#define SPRING_COMPRESS_MAX  48.0f
#define SPRING_K             25.0f
#define BODY_HALF_H           8.0f

/* Which way we launch. LAUNCH_ANGLE is the flat-ground angle (50°,
 * giving a nicely balanced hop). On a slope we tilt that angle by
 * SLOPE_SCALE, then clamp it between LAUNCH_MIN and LAUNCH_MAX so a
 * steep patch of ground can't fling us straight up or straight out. */
#define LAUNCH_ANGLE  (50.0f * (float)M_PI / 180.0f)
#define SLOPE_SCALE    0.5f
#define LAUNCH_MIN    (25.0f * (float)M_PI / 180.0f)
#define LAUNCH_MAX    (82.0f * (float)M_PI / 180.0f)

/* Downward pull, in pixels per second². Tuned so a default hop peaks
 * about 5 cells up — easy to see, doesn't fill the screen. */
#define GRAVITY  200.0f

/* How long each grounded phase lasts. T_COMPRESS is the spring-loading
 * windup (long enough to watch the gauge fill); T_LAND is the brief
 * impact flash. */
#define T_COMPRESS  0.45f
#define T_LAND      0.14f

/* The ground is two waves of noise added together: one big slow wave
 * for rolling hills, one small fast wave for bumps on top. T_AMP is
 * how tall the hills get; the two T_FREQ values set the wavelengths;
 * the two weights (0.6 + 0.4) add to 1 so the result still lands in
 * [0,1] like a single noise sample. */
#define T_AMP            112.0f
#define T_FREQ_BIG       0.0028f
#define T_FREQ_SMALL     0.014f
#define T_WEIGHT_BIG     0.6f
#define T_WEIGHT_SMALL   0.4f

#define NOISE_N          512    /* must be a power of 2 — lets us wrap with & */

/* Follow-camera. It stays put until the robot reaches CAM_TRIGGER of
 * the screen width (80%), then slides after it at CAM_SPEED (a smooth
 * chase, never a snap). CAM_START_COL is where the robot spawns. */
#define CAM_TRIGGER     0.80f
#define CAM_SPEED       6.0f
#define CAM_START_COL   3

enum { TRAIL_CAP = 1500 };

/* The 'a' key picks one of these. It stretches the hop horizontally
 * only — the arc height stays the same so the jump still reads, the
 * robot just covers more ground. */
enum { SPEED_LEVELS = 5 };
static const float SPEED_MULTS[SPEED_LEVELS] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };

/* Cap on one frame's time step. If the program stalls (you dragged the
 * window, the laptop slept), this keeps it from trying to simulate one
 * giant leap and tearing the physics apart. */
#define DT_CAP_SEC  0.10f

/* ncurses colour-pair IDs. */
enum {
    /* 1..6 — robot & spring */
    CP_BODY      = 1,        /* '@' grounded                 white       */
    CP_SPRING_LO,            /* spring low compression       yellow      */
    CP_SPRING_MD,            /* spring medium                orange      */
    CP_SPRING_HI,            /* spring high (loaded)         red         */
    CP_FLIGHT,               /* 'O' airborne                 cyan        */
    CP_LAND,                 /* '*' impact                   magenta     */

    /* 7..8 — trail */
    CP_TRAIL,                /* fresh trail '.'              blue        */
    CP_TRAIL_OLD,            /* aged trail ':'               dim blue    */

    /* 9..11 — terrain & PE */
    CP_SURF,                 /* terrain surface              green       */
    CP_ROCK,                 /* terrain sub-surface          dim green   */
    CP_PE,                   /* PE bar fill                  yellow      */

    /* 12..13 — HUD spec, theme-independent */
    PAIR_HUD,                /* row 0 status                 yellow BOLD */
    PAIR_HINT,               /* bottom row hint              cyan BOLD   */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

#define HUD_BUF_LEN  192

/* ── §2 clock — monotonic timer + sleep ── */

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

/* ── §3 color — one fixed palette, no themes ── */

/* Each thing on screen gets its own colour so you can read the state
 * at a glance: the spring goes yellow → orange → red as it loads, the
 * body is white on the ground, cyan in the air, magenta on impact. */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,        255, COLOR_BLACK);    /* near-white   */
        init_pair(CP_SPRING_LO,   226, COLOR_BLACK);    /* yellow       */
        init_pair(CP_SPRING_MD,   208, COLOR_BLACK);    /* orange       */
        init_pair(CP_SPRING_HI,   196, COLOR_BLACK);    /* red          */
        init_pair(CP_FLIGHT,       51, COLOR_BLACK);    /* cyan         */
        init_pair(CP_LAND,        201, COLOR_BLACK);    /* magenta      */
        init_pair(CP_TRAIL,        27, COLOR_BLACK);    /* blue         */
        init_pair(CP_TRAIL_OLD,    25, COLOR_BLACK);    /* dim blue     */
        init_pair(CP_SURF,         46, COLOR_BLACK);    /* bright green */
        init_pair(CP_ROCK,         28, COLOR_BLACK);    /* dim green    */
        init_pair(CP_PE,          226, COLOR_BLACK);    /* yellow       */
        init_pair(PAIR_HUD,       226, -1);             /* yellow on bg */
        init_pair(PAIR_HINT,       51, -1);             /* cyan on bg   */
    } else {
        init_pair(CP_BODY,      COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_SPRING_LO, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_SPRING_MD, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_SPRING_HI, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_FLIGHT,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_LAND,      COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_TRAIL,     COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_SURF,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_ROCK,      COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_PE,        COLOR_YELLOW,  COLOR_BLACK);
        init_pair(PAIR_HUD,     COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,    COLOR_CYAN,    -1);
    }
}

/* ── §4 coords — convert between fine pixels and terminal cells ── */

/* pw/ph give the screen size in pixels; px_to_cx/cy round a pixel
 * position to the nearest cell for drawing. */
static inline int   pw       (int cols)  { return cols * CELL_W; }
static inline int   ph       (int rows)  { return rows * CELL_H; }
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── §5 noise — smooth random wiggle for the terrain ── */

/* Value noise: pick a random height at each whole-number point, then
 * blend smoothly between them. The blend uses a cosine curve so the
 * line glides through each point with no sharp kinks. Output stays in
 * [0,1] for any x. */
static float g_noise[NOISE_N];

static void noise_init(unsigned seed)
{
    srand(seed);
    for (int i = 0; i < NOISE_N; i++)
        g_noise[i] = (float)rand() / (float)RAND_MAX;
}

static float noise1d(float x)
{
    int   xi = (int)floorf(x) & (NOISE_N - 1);
    float xf = x - floorf(x);
    float a  = g_noise[xi];
    float b  = g_noise[(xi + 1) & (NOISE_N - 1)];
    float t  = (1.0f - cosf(xf * (float)M_PI)) * 0.5f;
    return a * (1.0f - t) + b * t;
}

/* One big slow wave plus one small fast wave — hills with bumps. */
static float terrain_noise(float world_x)
{
    return noise1d(world_x * T_FREQ_BIG  ) * T_WEIGHT_BIG
         + noise1d(world_x * T_FREQ_SMALL) * T_WEIGHT_SMALL;
}

/* ── §6 terrain — ground height, slope, and surface glyph ── */

/*
 * Which kind of ground to draw. The 'f' key flips between them live.
 *
 *   FLOOR_FLAT   = 0   A dead-level line. Slope is zero everywhere, so
 *                      the robot always hops at the same angle. Handy
 *                      for watching the spring/flight physics on their
 *                      own. (It's 0 on purpose: a freshly zeroed World
 *                      starts here, which keeps reset safe.)
 *   FLOOR_PERLIN = 1   Rolling hills from the noise above. The slope
 *                      changes as you move, so each hop tilts to match
 *                      the ground it's leaving.
 *   FLOOR_COUNT        Just a count, for wrapping with `% FLOOR_COUNT`.
 */
typedef enum { FLOOR_FLAT = 0, FLOOR_PERLIN, FLOOR_COUNT } FloorMode;
static const char *FLOOR_NAMES[FLOOR_COUNT] = { "FLAT  ", "PERLIN" };

/* Height of the ground at a given spot. Flat returns the baseline;
 * hilly swings the noise above and below it by up to T_AMP. */
static float floor_y_at(float world_x, FloorMode mode, float base_y)
{
    if (mode == FLOOR_FLAT) return base_y;
    return base_y + (terrain_noise(world_x) - 0.5f) * 2.0f * T_AMP;
}

/* How steep the ground is here, as an angle. We measure it by
 * comparing the height a little to the left and a little to the right
 * (sampling both sides keeps the estimate even-handed and smooth).
 * Positive means the ground slopes down to the right; negative, up. */
static float floor_slope(float world_x, FloorMode mode, float base_y)
{
    float dx = (float)(CELL_W * 2);
    float dy = floor_y_at(world_x + dx, mode, base_y)
             - floor_y_at(world_x - dx, mode, base_y);
    return atan2f(dy, 2.0f * dx);
}

/* Pick the surface character by whether the next column is higher,
 * lower, or about level: '/' going up, '\' going down, '_' flat. */
static chtype surface_glyph(float dy)
{
    if (dy < -CELL_H * 0.20f) return '/';
    if (dy >  CELL_H * 0.20f) return '\\';
    return '_';
}

/* ── §7 trail — the fading dots left behind in the air ── */

/*
 * A ring buffer of the body's recent positions, so we can draw the
 * dotted arc trailing the robot. Only flight positions get pushed
 * (the body barely moves on the ground), so the trail shows off the
 * shape of each hop and nothing else.
 *
 * A ring buffer means pushing is instant — write the newest slot and
 * move on, letting the oldest entry quietly get overwritten — instead
 * of shuffling the whole array every frame. Two separate x and y
 * arrays (rather than one array of points) because drawing walks them
 * one axis at a time.
 *
 * Positions are stored in world space, not screen space, so the trail
 * stays pinned to the ground as the camera scrolls past it.
 *
 *   wx, wy   the past positions (world pixels), kept in step.
 *   head     where the next position will be written; once full, this
 *            also points at the oldest entry (the one about to go).
 *   count    how many slots hold real data — climbs to TRAIL_CAP and
 *            then stays there.
 *
 * Same circular-buffer trick diff_drive_robot.c uses for its trail.
 */
typedef struct {
    float wx[TRAIL_CAP];
    float wy[TRAIL_CAP];
    int   head;
    int   count;
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = 0;
}

static void trail_push(Trail *t, float wx, float wy)
{
    t->wx[t->head] = wx;
    t->wy[t->head] = wy;
    t->head        = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ── §8 robot — the jump cycle, its physics, and the scene around it ── */

/*
 * The three stages of one hop, looping forever:
 *
 *   COMPRESS → FLIGHT → LAND → COMPRESS → ...
 *
 *   PHASE_COMPRESS — spring loading. The body sinks; when the spring
 *                    is full, we fire it and switch to FLIGHT.
 *   PHASE_FLIGHT   — sailing through the air. Gravity pulls the body
 *                    down along a parabola until the foot hits ground.
 *   PHASE_LAND     — a brief landing flash, body planted, then back to
 *                    COMPRESS for the next hop.
 *
 * (COMPRESS is 0 on purpose so a freshly zeroed Robot starts there.)
 */
typedef enum {
    PHASE_COMPRESS = 0,    /* spring loading, body sinking      */
    PHASE_FLIGHT,          /* projectile arc                    */
    PHASE_LAND,            /* impact flash, body locked         */
} Phase;

static const char *PHASE_NAMES[] = { "LOAD", "FLY ", "LAND" };

/*
 * Where the robot is. It has two anchor points: the foot (where it
 * touches the ground, which only moves when it lands) and the body
 * (its centre). On the ground the body's height is set by the spring;
 * in the air the body flies free and the foot stays frozen at the last
 * touchdown.
 *
 *   body_px, body_py   the body's centre, in world pixels.
 *   foot_px, foot_py   the ground-contact point, in world pixels.
 *
 * This body-plus-foot pair is the classic hopping-robot model from
 * Raibert's "Legged Robots That Balance".
 */
typedef struct {
    float body_px, body_py;
    float foot_px, foot_py;
} Pose;

/*
 * How fast the body is moving (pixels per second). Only matters in the
 * air: it's set once when the spring fires, then gravity bends it each
 * frame. On the ground both are zero. Remember +y points down.
 */
typedef struct {
    float vx, vy;
} Velocity;

/*
 * The leg spring, boiled down to one number: how far it's squashed
 * right now (0 unloaded, up to SPRING_COMPRESS_MAX fully loaded).
 * While loading, this ramps up; at launch, the energy it's holding
 * (½·K·squash²) becomes the body's speed.
 */
typedef struct {
    float compress;
} Spring;

/*
 * Which phase we're in, plus how long we've been in it. They're kept
 * together so the timer can never lag behind the phase — every phase
 * change resets phase_t to 0.
 *
 *   phase     COMPRESS, FLIGHT, or LAND.
 *   phase_t   seconds elapsed in the current phase.
 */
typedef struct {
    Phase phase;
    float phase_t;
} PhaseFSM;

/*
 * Where the next hop will aim. On flat ground it's just the default
 * angle; on a slope it tilts to follow the ground. We work it out once
 * while the spring is loading and stash it here, so the flight code
 * doesn't have to re-measure the (somewhat costly) slope mid-jump.
 *
 *   slope_angle   how the ground tilts under the foot (radians).
 *   eff_angle     the actual launch angle after the slope tilt.
 */
typedef struct {
    float slope_angle;
    float eff_angle;
} LaunchAim;

/*
 * The robot itself: everything about the hopping agent, grouped by job.
 *
 *   pose     where it is (body + foot).
 *   vel      how fast it's moving (only in the air).
 *   spring   how loaded the leg is.
 *   fsm      which phase, and how long it's been there.
 *   aim      which way the next hop will go.
 *   trail    the dotted arc it leaves behind.
 *
 * The split lets each helper take just the piece it needs, so a
 * function's parameters tell you what state it touches. The world,
 * camera, and controls live on Scene instead — this struct is only
 * about the robot's own state.
 */
typedef struct {
    Pose      pose;
    Velocity  vel;
    Spring    spring;
    PhaseFSM  fsm;
    LaunchAim aim;
    Trail     trail;
} Robot;

/* ── §8.1b the world around the robot ── */

/*
 * The camera. The world stretches far past the screen as the robot
 * keeps hopping forward, so this holds one number — the world x of the
 * left edge of the screen — and slides it smoothly to keep up. To draw
 * anything, subtract cam_x from its world x to get its screen column.
 * Never goes below 0 (we don't scroll past the start of the world).
 */
typedef struct {
    float cam_x;
} Camera;

/*
 * The ground setup: which kind of floor (flat or hilly) and the
 * baseline height the hills wobble around. The 'f' key swaps the kind;
 * base_y is recomputed when the terminal is resized so the ground
 * re-centres.
 *
 *   base_y       the floor's resting height, in pixels.
 *   floor_mode   FLAT or PERLIN.
 */
typedef struct {
    float     base_y;
    FloorMode floor_mode;
} World;

/*
 * The user-facing knobs, plus the jump tally. The keypress handler
 * writes these; the simulation and HUD read them. launch_count lives
 * here (not on the Robot) because it's a whole-session count, not part
 * of the robot's physical state.
 *
 *   paused         when true, physics freezes but drawing continues.
 *   speed_level    which entry of SPEED_MULTS scales the hop.
 *   launch_count   how many jumps since the last reset.
 */
typedef struct {
    bool paused;
    int  speed_level;
    int  launch_count;
} SimControls;

/*
 * Everything the simulation owns for one run, in four parts: the robot,
 * the world it hops over, the camera watching it, and the user
 * controls. Bundling them this way means a helper can take just the
 * part it cares about rather than the whole scene.
 */
typedef struct {
    Robot       robot;
    World       world;
    Camera      camera;
    SimControls sim;
} Scene;

/* ── §8.2 spring physics ── */

/* Energy stored in the spring right now. Grows with the square of the
 * squash, which is why the gauge crawls at first and races near full.
 * Only the gauge uses it. */
static inline float spring_energy(float compress)
{
    return 0.5f * SPRING_K * compress * compress;
}

/* How tall the leg stands at a given squash. */
static inline float leg_length(float compress)
{
    return SPRING_REST - compress;
}

/* Put the body where the foot and the current squash say it should be.
 * Used on the ground (COMPRESS and LAND); in the air the body moves on
 * its own instead. */
static void pose_from_spring(Robot *r)
{
    r->pose.body_px = r->pose.foot_px;
    r->pose.body_py = r->pose.foot_py - leg_length(r->spring.compress) - BODY_HALF_H;
}

/* ── §8.3 tilt the launch angle to match the ground ── */

/* Steepen the hop when climbing (so it clears the rise), flatten it
 * when descending (so it rides down), and clamp so a wild slope can't
 * make us fire straight up or straight ahead. */
static float effective_launch_angle(float slope)
{
    return clampf(LAUNCH_ANGLE - slope * SLOPE_SCALE,
                  LAUNCH_MIN, LAUNCH_MAX);
}

/* ── §8.4 the follow camera ── */

/* Once the robot passes 80% of the screen, start sliding the camera so
 * it stays pinned there. The camera eases toward its target rather than
 * jumping, and never scrolls back past the start of the world. */
static void cam_update(Scene *s, float dt, int cols)
{
    float scr_w   = (float)pw(cols);
    float bot_sx  = s->robot.pose.body_px - s->camera.cam_x;  /* screen x of robot */
    float trigger = scr_w * CAM_TRIGGER;

    if (bot_sx > trigger) {
        float target = s->robot.pose.body_px - trigger;
        s->camera.cam_x += (target - s->camera.cam_x) * CAM_SPEED * dt;
    }

    if (s->camera.cam_x < 0.0f) s->camera.cam_x = 0.0f;
}

/* ── §8.5 COMPRESS: load the spring, then fire ── */

/* How far through the loading windup we are, from 0 to 1. */
static inline float compress_progress_fraction(float phase_t) {
    return clampf(phase_t / T_COMPRESS, 0.0f, 1.0f);
}

/* Measure the ground slope under the foot now and stash the resulting
 * launch angle, so the flight code doesn't have to re-measure it. */
static inline void sample_terrain_aim(Scene *s) {
    Robot *r = &s->robot;
    r->aim.slope_angle = floor_slope(r->pose.foot_px,
                                     s->world.floor_mode, s->world.base_y);
    r->aim.eff_angle   = effective_launch_angle(r->aim.slope_angle);
}

/* Turn the stored spring energy into a launch speed: the more it's
 * squashed, the faster it goes (the spring's energy becomes the body's
 * motion). */
static inline float spring_pe_to_launch_speed(float compress) {
    return compress * sqrtf(SPRING_K);
}

/* Fire the spring: split the launch speed into forward and upward
 * parts along the aimed angle, apply the user's horizontal speed knob,
 * empty the spring, and switch to FLIGHT. (The speed knob stretches
 * the hop sideways only, leaving the height alone.) */
static inline void emit_launch_impulse(Scene *s) {
    Robot *r = &s->robot;
    float v_launch  = spring_pe_to_launch_speed(r->spring.compress);
    float speed_mult = SPEED_MULTS[s->sim.speed_level];

    r->vel.vx =  v_launch * cosf(r->aim.eff_angle) * speed_mult;
    r->vel.vy = -v_launch * sinf(r->aim.eff_angle);   /* y-down: up = negative */

    r->spring.compress = 0.0f;          /* leg now fully extended */
    r->fsm.phase       = PHASE_FLIGHT;
    r->fsm.phase_t     = 0.0f;
    s->sim.launch_count++;
}

/* One frame of loading: squash the spring a bit more, move the body to
 * match, re-check the slope, and fire once the spring is full. */
static void compress_tick(Scene *s)
{
    Robot *r = &s->robot;

    r->spring.compress = SPRING_COMPRESS_MAX
                       * compress_progress_fraction(r->fsm.phase_t);
    pose_from_spring(r);

    sample_terrain_aim(s);

    bool fully_loaded = (r->fsm.phase_t >= T_COMPRESS);
    if (fully_loaded) emit_launch_impulse(s);
}

/* ── §8.6 FLIGHT: fly through the air, watch for the ground ── */

/* Move the body one step under gravity: speed up the fall, then move.
 * It's the simple kind of step that slowly gains a hair of energy over
 * time, but a hop lasts under a second so the error never shows. */
static inline void integrate_projectile_step(Robot *r, float dt) {
    r->vel.vy       += GRAVITY * dt;
    r->pose.body_px += r->vel.vx * dt;
    r->pose.body_py += r->vel.vy * dt;
}

/* Where the foot would reach if the leg hung fully down from the body.
 * Once that dips to or below the ground, the robot has touched down. */
static inline float foot_projection_y(const Robot *r) {
    return r->pose.body_py + BODY_HALF_H + SPRING_REST;
}

/* Touch down: plant the foot on the ground right under the body, empty
 * the spring, stop moving, and switch to LAND. */
static inline void commit_landing(Robot *r, float ground_y) {
    r->pose.foot_px    = r->pose.body_px;
    r->pose.foot_py    = ground_y;
    r->spring.compress = 0.0f;
    pose_from_spring(r);
    r->vel.vx = r->vel.vy = 0.0f;

    r->fsm.phase   = PHASE_LAND;
    r->fsm.phase_t = 0.0f;
}

/* One frame in the air: take a gravity step, drop a dot on the trail,
 * and land if the foot has reached the ground below. */
static void flight_tick(Scene *s, float dt)
{
    Robot *r = &s->robot;

    integrate_projectile_step(r, dt);

    trail_push(&r->trail, r->pose.body_px, r->pose.body_py);

    float ground_y = floor_y_at(r->pose.body_px,
                                s->world.floor_mode, s->world.base_y);
    bool leg_punched_ground = (foot_projection_y(r) >= ground_y);
    if (leg_punched_ground) commit_landing(r, ground_y);
}

/* ── §8.7 LAND: a beat to rest, then load again ── */

/* Hold the landing pose for T_LAND seconds, then go back to loading.
 * Nothing moves; only the body glyph and the missing spring change. */
static void land_tick(Robot *r)
{
    pose_from_spring(r);
    if (r->fsm.phase_t >= T_LAND) {
        r->fsm.phase   = PHASE_COMPRESS;
        r->fsm.phase_t = 0.0f;
    }
}

/* ── §8.8 advance the whole scene by one frame ── */

/* When paused, skip the physics but keep drawing. Otherwise move the
 * camera, age the phase timer, and run whichever phase we're in. */
static void scene_tick(Scene *s, float dt, int cols)
{
    if (s->sim.paused) return;

    cam_update(s, dt, cols);
    s->robot.fsm.phase_t += dt;

    switch (s->robot.fsm.phase) {
    case PHASE_COMPRESS: compress_tick(s);     break;
    case PHASE_FLIGHT:   flight_tick(s, dt);   break;
    case PHASE_LAND:     land_tick(&s->robot); break;
    }
}

/* ── §8.9 set up / reset the scene ── */

static void scene_init(Scene *s, int rows)
{
    /* Keep the floor mode and speed the user picked across a reset. */
    FloorMode saved_floor = s->world.floor_mode;
    int       saved_speed = s->sim.speed_level;

    memset(s, 0, sizeof *s);
    s->world.floor_mode = saved_floor;
    s->sim.speed_level  = saved_speed;
    s->sim.paused       = false;

    /* Put the ground 72% down the screen — room above for the hops and
     * the gauge, room below for the dirt fill. */
    s->world.base_y = (float)ph(rows) * 0.72f;

    Robot *r = &s->robot;

    /* Drop the robot near the left edge, standing on the ground. */
    r->pose.foot_px    = (float)(CAM_START_COL * CELL_W);
    r->pose.foot_py    = floor_y_at(r->pose.foot_px,
                                    s->world.floor_mode, s->world.base_y);
    r->aim.slope_angle = floor_slope(r->pose.foot_px,
                                     s->world.floor_mode, s->world.base_y);
    r->aim.eff_angle   = effective_launch_angle(r->aim.slope_angle);

    r->fsm.phase   = PHASE_COMPRESS;
    r->fsm.phase_t = 0.0f;
    s->camera.cam_x = 0.0f;

    pose_from_spring(r);
}

/* The 'r' key: clear the trail and start over, keeping the user's
 * floor/speed choices. */
static void scene_reset(Scene *s, int rows)
{
    trail_clear(&s->robot.trail);
    scene_init(s, rows);
}

/* ── §9 render — paint one frame ── */

/* World x to the screen column it lands in, given the camera. */
static inline int scr_cx(float world_px, float cam_x)
{
    return px_to_cx(world_px - cam_x);
}

/* True if a cell is inside the playfield. Row 0 is the HUD and the
 * bottom row is the key hints, so drawing stays between them. */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* ── §9.2 the ground ── */

/* For each column, find the ground height and draw it as a stack: the
 * surface line on top, a fuzzy grass row below it, then solid dirt down
 * to the bottom. The checker pattern keys off the column so it doesn't
 * shimmer as the camera scrolls. */
static void render_terrain(FloorMode mode, float base_y, float cam_x,
                           int cols, int rows)
{
    for (int sc = 0; sc < cols; sc++) {
        float wx      = cam_x + (float)(sc * CELL_W);
        float fy      = floor_y_at(wx, mode, base_y);
        float fy_next = floor_y_at(wx + (float)CELL_W, mode, base_y);
        int   surf    = px_to_cy(fy);
        if (surf < 1)        surf = 1;
        if (surf > rows - 2) surf = rows - 2;

        chtype sg = (mode == FLOOR_PERLIN)
                  ? surface_glyph(fy_next - fy)
                  : '_';

        attron(COLOR_PAIR(CP_SURF) | A_BOLD);
        mvaddch(surf, sc, sg);
        attroff(COLOR_PAIR(CP_SURF) | A_BOLD);

        if (surf + 1 < rows - 1) {
            attron(COLOR_PAIR(CP_SURF) | A_DIM);
            mvaddch(surf + 1, sc, (sc % 2 == 0) ? ':' : '.');
            attroff(COLOR_PAIR(CP_SURF) | A_DIM);
        }

        attron(COLOR_PAIR(CP_ROCK) | A_DIM);
        for (int r = surf + 2; r < rows - 1; r++)
            mvaddch(r, sc, (sc % 2 == 0) ? '#' : ' ');
        attroff(COLOR_PAIR(CP_ROCK) | A_DIM);
    }
}

/* ── §9.3 the dotted trail ── */

/* Draw the trail newest-first so fresh dots sit on top of old ones.
 * Newer dots are brighter '.'; older ones fade to a dim ':'. */
static void render_trail(const Robot *r, float cam_x, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int cx  = scr_cx     (r->trail.wx[idx], cam_x);
        int cy  = px_to_cy   (r->trail.wy[idx]);
        if (!in_bounds(cx, cy, cols, rows)) continue;

        float  age = (r->trail.count > 1)
                   ? (float)k / (float)(r->trail.count - 1) : 0.0f;
        int    cp  = (age < 0.40f) ? CP_TRAIL : CP_TRAIL_OLD;
        attr_t at  = (age < 0.15f) ? A_BOLD
                   : (age < 0.40f) ? A_NORMAL : A_DIM;
        chtype ch  = (age < 0.40f) ? '.' : ':';

        attron(COLOR_PAIR(cp) | at);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/* ── §9.4 the spring leg ── */

/* Draw the coil from the bottom of the body down to the foot. It looks
 * like it scrunches up as the body sinks, and shifts yellow → orange →
 * red as the spring loads, so you can see how charged it is. */
static void render_spring(const Robot *r, float cam_x, int cols, int rows)
{
    int cx       = scr_cx(r->pose.body_px, cam_x);
    int body_bot = px_to_cy(r->pose.body_py + BODY_HALF_H);
    int foot_cy  = px_to_cy(r->pose.foot_py);

    if (cx < 0 || cx >= cols) return;

    float ratio = r->spring.compress / SPRING_COMPRESS_MAX;
    int    cp   = (ratio < 0.30f) ? CP_SPRING_LO
                : (ratio < 0.70f) ? CP_SPRING_MD : CP_SPRING_HI;
    attr_t at   = (ratio > 0.70f) ? A_BOLD : A_NORMAL;

    static const chtype coil[4] = { '(', '|', ')', '|' };
    for (int row = body_bot; row < foot_cy; row++) {
        if (row < 1 || row >= rows - 1) continue;
        attron(COLOR_PAIR(cp) | at);
        mvaddch(row, cx, coil[(row - body_bot) & 3]);
        attroff(COLOR_PAIR(cp) | at);
    }

    /* the little foot marker on the ground */
    if (foot_cy >= 1 && foot_cy < rows - 1) {
        attron(COLOR_PAIR(CP_ROCK));
        mvaddch(foot_cy, cx, 'v');
        attroff(COLOR_PAIR(CP_ROCK));
    }
}

/* ── §9.5 the body ── */

/* The body, drawn with a glyph that tells you the phase at a glance:
 * '@' loading, 'O' flying, '*' the moment it lands. */
static void render_body(const Robot *r, float cam_x, int cols, int rows)
{
    int cx = scr_cx(r->pose.body_px, cam_x);
    int cy = px_to_cy(r->pose.body_py);
    if (!in_bounds(cx, cy, cols, rows)) return;

    chtype ch; int cp;
    switch (r->fsm.phase) {
    case PHASE_COMPRESS: ch = '@'; cp = CP_BODY;   break;
    case PHASE_FLIGHT:   ch = 'O'; cp = CP_FLIGHT; break;
    default:             ch = '*'; cp = CP_LAND;   break;
    }

    attron(COLOR_PAIR(cp) | A_BOLD);
    mvaddch(cy, cx, ch);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* ── §9.6 the energy gauge (only while loading) ── */

/* Where the gauge sits and how big it gets. */
enum {
    PE_BAR_RIGHT_INSET    = 3,   /* columns in from the right edge       */
    PE_BAR_BOTTOM_INSET   = 2,   /* rows up from the bottom edge          */
    PE_BAR_VERTICAL_MARGIN = 10, /* rows kept clear above and below       */
    PE_BAR_MIN_HEIGHT     = 4,
    PE_BAR_WIDTH_CELLS    = 2,   /* two cells wide                        */
};
#define PE_BAR_HIGH_RATIO 0.80f  /* above this the bar turns red          */

/* The gauge's spot on screen, worked out once and passed around. */
typedef struct {
    int x;        /* its left column        */
    int top;      /* its top row            */
    int bottom;   /* its bottom row         */
    int height;   /* how many rows tall     */
} PeBarGeometry;

/* Work out where the gauge goes from the screen size. If the terminal
 * is too short for even a 4-row bar, the caller skips it. */
static inline PeBarGeometry compute_pe_bar_geometry(int cols, int rows) {
    PeBarGeometry g;
    g.x      = cols - PE_BAR_RIGHT_INSET;
    g.bottom = rows - PE_BAR_BOTTOM_INSET;
    g.height = (rows > PE_BAR_VERTICAL_MARGIN + 2)
             ? rows - PE_BAR_VERTICAL_MARGIN
             : PE_BAR_MIN_HEIGHT;
    g.top    = g.bottom - g.height;
    return g;
}

/* How full the gauge should be, from 0 to 1. It's the squash squared,
 * so the bar creeps up at first and then shoots up near the top — which
 * is exactly how the stored energy grows. */
static inline float pe_ratio_quadratic(float compress) {
    return spring_energy(compress) / spring_energy(SPRING_COMPRESS_MAX);
}

/* The little "PE" label above the gauge. */
static inline void paint_pe_label(const PeBarGeometry *g) {
    if (g->top - 1 < 1) return;
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(g->top - 1, g->x, "PE");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Paint one row of the bar: a lit one in the fill colour, or an empty
 * one shown as faint resting dots. */
static inline void paint_filled_cell(int row, int x, chtype glyph, int pair) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(row, x,     glyph);
    mvaddch(row, x + 1, glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}
static inline void paint_empty_cell(int row, int x) {
    attron(COLOR_PAIR(PAIR_HUD));
    mvaddch(row, x,     '.');
    mvaddch(row, x + 1, '.');
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Paint the whole gauge from the bottom up: the filled rows lit, the
 * rest as rest dots. When it's nearly full it turns red and the top lit
 * cell becomes '!' — a "ready to fire" cue. */
static inline void paint_pe_bar_column(const PeBarGeometry *g,
                                        int filled_cells, float ratio,
                                        int rows) {
    bool high_charge = (ratio > PE_BAR_HIGH_RATIO);
    int  fill_pair   = high_charge ? CP_SPRING_HI : CP_PE;

    for (int i = 0; i < g->height; i++) {
        int row = g->bottom - i;
        if (row < 1 || row >= rows - 1) continue;     /* stay off the HUD rows */

        bool   is_filled  = (i < filled_cells);
        bool   is_top_lit = (i == filled_cells - 1);
        chtype glyph      = (high_charge && is_top_lit) ? '!' : '|';

        if (is_filled) paint_filled_cell(row, g->x, glyph, fill_pair);
        else           paint_empty_cell (row, g->x);
    }
}

/* The energy gauge on the right, shown only while the spring is
 * loading. Its height tracks the stored energy. */
static void render_pe_bar(const Robot *r, int cols, int rows)
{
    if (r->fsm.phase != PHASE_COMPRESS) return;

    PeBarGeometry geom = compute_pe_bar_geometry(cols, rows);
    if (geom.x < 0 || geom.top < 1) return;

    float ratio        = pe_ratio_quadratic(r->spring.compress);
    int   filled_cells = (int)(ratio * (float)geom.height + 0.5f);

    paint_pe_label    (&geom);
    paint_pe_bar_column(&geom, filled_cells, ratio, rows);
}

/* ── §9.7 the HUD (status bar + key hints) ── */

/* Degrees for the HUD readout. */
static inline float radians_to_degrees(float radians) {
    return radians * (180.0f / (float)M_PI);
}

/* Print text, then fill the rest of the row with spaces so the
 * coloured bar runs edge to edge instead of stopping where the text
 * ends. */
static inline void hud_paint_text_padded(int row, int col, int pair,
                                          const char *text, int total_cols) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddnstr(row, col, text, total_cols - col);
    int used = (int)strlen(text);
    for (int x = col + used; x < total_cols; x++)
        mvaddch(row, x, ' ');
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Same, but no padding — for the key-hint strip, which needn't span
 * the full width. */
static inline void hud_paint_text(int row, int col, int pair,
                                   const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top status line: fps, floor mode, phase, spring load,
 * stored energy, slope and launch angles, speed, position, scroll,
 * jump count, and whether we're paused. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const Robot *r = &s->robot;
    int robot_scr_col = scr_cx(r->pose.body_px, s->camera.cam_x);

    snprintf(buf, buflen,
             " %5.1f fps  [%s] %-4s  cmp:%3.0f/%3.0f  PE:%5.0f  "
             "slope:%+5.1f°  launch:%4.1f°  spd:%.1fx  col:%-3d  "
             "cam:%5.0f  jumps:%-3d  %s ",
             fps, FLOOR_NAMES[s->world.floor_mode], PHASE_NAMES[r->fsm.phase],
             r->spring.compress, SPRING_COMPRESS_MAX,
             spring_energy(r->spring.compress),
             radians_to_degrees(r->aim.slope_angle),
             radians_to_degrees(r->aim.eff_angle),
             SPEED_MULTS[s->sim.speed_level],
             robot_scr_col, s->camera.cam_x, s->sim.launch_count,
             s->sim.paused ? "PAUSED" : "running");
}

/* The yellow status line across the top. */
static void draw_hud_status(const Scene *s, double fps, int cols) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    hud_paint_text_padded(HUD_TOP_ROW, 0, PAIR_HUD, buf, cols);
}

/* The cyan list of keys along the bottom. */
static void draw_hud_hint(int rows) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  f:floor  n:new-terrain  a:speed ";
    hud_paint_text(rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* The "PAUSED" sign in the middle when frozen. It borrows the bright
 * landing-flash colour rather than defining a new one. */
static void draw_paused_banner(int cols, int rows) {
    static const char *PAUSED_LABEL = " PAUSED ";
    int label_width = (int)strlen(PAUSED_LABEL);
    int banner_col  = cols / 2 - label_width / 2;
    if (banner_col < 0) return;
    hud_paint_text(rows / 2, banner_col, CP_LAND, PAUSED_LABEL);
}

/* The whole HUD: status line on top, key hints on the bottom, and the
 * PAUSED sign over the middle when frozen. */
static void render_hud(const Scene *s, double fps, int cols, int rows)
{
    draw_hud_status(s, fps, cols);
    draw_hud_hint  (rows);
    if (s->sim.paused) draw_paused_banner(cols, rows);
}

/* ── §9.8 paint the full frame ── */

/* Draw back to front so each layer covers the one before: trail, then
 * ground, then the leg, the body, the gauge, and finally the HUD. */
static void scene_draw(const Scene *s, double fps, int cols, int rows)
{
    erase();

    const Robot *r = &s->robot;
    float cam_x = s->camera.cam_x;

    render_trail   (r, cam_x, cols, rows);
    render_terrain (s->world.floor_mode, s->world.base_y, cam_x, cols, rows);

    if (r->fsm.phase == PHASE_COMPRESS || r->fsm.phase == PHASE_LAND)
        render_spring(r, cam_x, cols, rows);

    render_body    (r, cam_x, cols, rows);
    render_pe_bar  (r, cols, rows);
    render_hud     (s, fps, cols, rows);
}

/* ── §10 screen — start, resize, and flush the terminal ── */

/*
 * The size of the terminal in cells, kept up to date as it resizes.
 * Drawing and HUD placement read it; the ncurses startup/shutdown
 * calls are all kept here so the rest of the code doesn't touch them.
 *
 *   cols   width in cells.
 *   rows   height in cells. Row 0 is the status bar, the last row is
 *          the key hints, and the simulation fills everything between.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §11 app — signals, resize, and the main loop ── */

/*
 * A smoothed frame-rate readout. A per-frame number would flicker, so
 * this tallies frames over a half-second window and reports the average
 * for the HUD.
 *
 *   frame_count, window_ns   running tally for the current window.
 *   display                  the smoothed fps the HUD shows.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * Everything that lives for the whole program.
 *
 *   scene         the simulation.
 *   screen        the terminal size and lifecycle.
 *   fps           the smoothed frame-rate readout.
 *   running       cleared to stop the loop (by 'q' or a kill signal).
 *   need_resize   set when the window resizes; handled next frame.
 *
 * The two flags are sig_atomic_t because signal handlers set them.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Act on one keypress: quit, pause, reset, swap the floor, reseed the
 * terrain, or change the hop distance. */
static void app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27:
        app->running = 0;
        break;

    case ' ': case 'p': case 'P':
        s->sim.paused = !s->sim.paused;
        break;

    case 'r': case 'R':
        scene_reset(s, app->screen.rows);
        break;

    case 'f': case 'F':
        s->world.floor_mode =
            (FloorMode)((s->world.floor_mode + 1) % FLOOR_COUNT);
        break;

    case 'n': case 'N':
        noise_init((unsigned)clock_ns());
        break;

    case 'a': case 'A':
        s->sim.speed_level = (s->sim.speed_level + 1) % SPEED_LEVELS;
        break;

    default: break;
    }
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    noise_init((unsigned)(clock_ns() & 0xFFFFFFFF));

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);
    scene->world.floor_mode = FLOOR_PERLIN;   /* start on the hilly floor */

    screen_init(&app->screen);
    scene_init (scene, app->screen.rows);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* deal with a resize first so the rest of the frame sees the new size */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene->world.base_y = (float)ph(app->screen.rows) * 0.72f;
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* how much time passed, capped so a stall can't break the physics */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        int ch;
        while ((ch = getch()) != ERR) app_handle_key(app, ch);

        scene_tick(scene, dt, app->screen.cols);
        fps_counter_tick(&app->fps, dt_ns);

        scene_draw(scene, app->fps.display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* rest for what's left of the frame so we hold a steady rate */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
