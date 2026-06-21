/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hexpod_tripod.c — a six-legged robot walks across the terminal.
 *
 * It moves with a tripod gait: three legs plant on the ground while the
 * other three swing forward, then they swap — so there are always three
 * feet down forming a stable tripod and the body never tips. Each leg
 * uses inverse kinematics: you say where the foot should be and the code
 * works out the knee angle. Arrow keys steer; the body slides along its
 * heading.
 *
 * Sister demos (same 2-joint IK leg, different walkers):
 *   ik_spider.c, ik_scorpin.c
 *
 * Author : Tamilselvan R     License: MIT (see line 1)
 *
 * Build (needs -lm for the trig in the IK and rotations):
 *   gcc -std=c11 -O2 -Wall -Wextra animation/hexpod_tripod.c \
 *       -o hexpod_tripod -lncurses -lm
 *
 * Keys:  q / ESC quit   space pause   arrows steer   w/s speed
 *        t theme   [ / ] slow / speed up time
 */

#define _POSIX_C_SOURCE 200809L

/* M_PI is a POSIX extension, not standard C99/C11 — provide a fallback
 * so the build never fails on strict-conformance toolchains. */
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

/* ── §1 config — every tunable number, named in one place ── */

enum {
    /* Render frame-rate target. Variable-timestep simulation, so the
     * only thing this controls is the sleep cap at end of frame. */
    TARGET_FPS    = 60,

    /* HUD layout. */
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,

    /* ncurses pair IDs.
     *   1 body frame (rectangle + cross-braces + hip markers + center)
     *   2 femur (hip → knee)
     *   3 tibia (knee → foot)
     *   4 planted foot '*'  (bright accent)
     *   5 swinging foot 'o' (dim, in flight)
     *   6 knee joint 'o'    (bold)
     *   7 reserved
     *   8 PAIR_HUD  (status bar — bright yellow on default bg, A_BOLD)
     *   9 PAIR_HINT (key hint — bright cyan  on default bg, A_BOLD) */
    N_PAIRS  = 7,
    PAIR_HUD = 8,
    PAIR_HINT = 9,

    N_THEMES = 8,
    N_LEGS   = 6,
};

/* Body geometry (px). Body always faces +x in body-local space; the
 * heading rotates the whole frame about (body_x, body_y). */
#define BODY_LEN         80.0f   /* total length along body axis           */
#define BODY_HALF_W      20.0f   /* half-width — legs attach at ±this      */

/* Leg bone lengths (px). Their sum is the leg's max reach; every foot
 * target must stay inside it or the IK can't reach. */
#define UPPER_LEN        40.0f   /* femur (hip → knee)                     */
#define LOWER_LEN        36.0f   /* tibia (knee → foot)                    */

/* Walking speed (px/s). User-tunable in [MIN, MAX] via w/s. */
#define BODY_SPEED_DEFAULT  40.0f
#define BODY_SPEED_MIN      10.0f
#define BODY_SPEED_MAX     200.0f

/* Gait timing.
 *   PHASE_DURATION : minimum time one tripod stays in air before swap.
 *   STEP_DURATION  : single foot's swing arc duration.
 *   STEP_HEIGHT    : peak Y-lift during arc (pixels, +y = down).
 *   STEP_LOOKAHEAD : seconds-ahead foot lands; multiplied by body_speed
 *                    so faster walking → longer strides automatically. */
#define PHASE_DURATION    0.35f
#define STEP_DURATION     0.28f
#define STEP_HEIGHT       12.0f
#define STEP_LOOKAHEAD    0.18f

/* Heading interpolation rate (rad/s). 2.5 → 90° turn in ~0.6 s. */
#define TURN_RATE         2.5f

/* Direction-glyph line rasteriser step. Must be < CELL_W (8) so the
 * line never skips a column. */
#define DRAW_LEG_STEP_PX  8.0f

/* Time scale — user-controlled simulation speed multiplier.
 * 0.25× to 4×, default 1×. Stepped by ×/÷ TIME_SCALE_STEP via [/]. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions — the aspect-ratio bridge. */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic time + sleep ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);    /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                   /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — 8 robot palettes + fixed HUD colours ── */

/*
 * Theme — one robot colour palette (xterm-256 foreground indices).
 *
 * Picking a theme rebinds the seven body-rendering pairs (1..7) in one
 * shot; see theme_apply. The HUD/hint pairs are deliberately left out —
 * they stay bright yellow / bright cyan in every theme so the status bar
 * stays readable over any robot colour.
 *
 * col[0..6] map to pairs 1..7, in this order:
 *   [0] body box     [1] femur (hip→knee)   [2] tibia (knee→foot)
 *   [3] planted foot '*'  [4] swinging foot 'o'  [5] knee joint
 *   [6] reserved (unused; kept so adding an accent later won't renumber)
 *
 * Every index sits in the bright half of the palette so nothing vanishes
 * under A_DIM. The planted-foot slot is green 46 in every theme so
 * "ground contact" always reads the same colour.
 */
typedef struct {
    const char *name;          /* shown in the HUD                     */
    int         col[N_PAIRS];  /* xterm-256 fg per pair: pair p = col[p-1] */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      body  femur tibia plant step  knee   rsvd   */
    {"Steel",  {245,  67,  75,  46, 214, 231,  244}},
    {"Cobalt", {244,  27,  39,  46, 208, 231,  246}},
    {"Copper", {242, 130, 172,  46, 214, 231,  244}},
    {"Toxin",  {244,  34,  40,  46,  82, 231,  246}},
    {"Ember",  {244, 130, 136,  46, 208, 231,  246}},
    {"Ghost",  {248, 251, 254,  46, 252, 255,  246}},
    {"Neon",   {245,  93, 201,  46, 226, 255,  244}},
    {"Ocean",  {244,  27,  51,  46,  51, 231,  244}},
};

/* theme_apply — re-bind body pairs to the chosen theme.
 * HUD/HINT are NOT re-bound here because they're theme-independent. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_PAIRS; p++)
        init_pair(p + 1, t->col[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color fallback — coarser but directionally correct. */
        init_pair(1, COLOR_WHITE,   -1);
        init_pair(2, COLOR_CYAN,    -1);
        init_pair(3, COLOR_CYAN,    -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_YELLOW,  -1);
        init_pair(6, COLOR_WHITE,   -1);
        init_pair(7, COLOR_WHITE,   -1);
    }

    /* HUD pairs are theme-independent — bright yellow status, bright
     * cyan hint, both on default bg so they overlay any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — turn pixel positions into terminal cells ── */

/*
 * The robot lives in square "pixel" space (1 unit = 1 pixel). A terminal
 * cell is taller than it is wide (8 wide, 16 tall), so at draw time these
 * helpers divide by the cell size and round to the nearest cell. This is
 * the only place pixels become cells.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the Hexapod: body, legs, gait ── */

/* ── §5a vec math + the constant per-leg geometry tables ── */

/*
 * Vec2 — a 2-D point in pixel space.
 *
 * Two coordinate frames use this type:
 *   body-local : +x is forward (walk direction), +y is the body's right
 *                side, -y is its left. Used by the constant tables below.
 *   world      : +x is right of screen, +y is down. Used by foot/hip/knee
 *                positions. rotate2d turns a local point into world:
 *                world = body_center + R(heading) * local.
 *
 * Angles follow the math convention (positive = counter-clockwise from
 * +x). Because screen y points down, a math-CCW turn looks clockwise on
 * screen; only the left/right knee sign flip in solve_ik depends on this.
 */
typedef struct {
    float x;
    float y;
} Vec2;

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline Vec2 vec2_lerp(Vec2 a, Vec2 b, float t)
{
    return (Vec2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}

static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* rotate2d — turn vector v by `angle` radians (screen space, +y = down). */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/*
 * The six legs, by index (left side is even, right side is odd):
 *   0 left-front   1 right-front
 *   2 left-mid     3 right-mid
 *   4 left-rear    5 right-rear
 */

/* Where each hip attaches, measured from the body centre (body-local).
 * X is front/back along the body, Y is left/right. Constant — legs are
 * bolted to the body and never move relative to it. */
static const float HIP_LOCAL_X[N_LEGS] = {
    BODY_LEN * 0.40f,    /* 0 left-front  */
    BODY_LEN * 0.40f,    /* 1 right-front */
    0.0f,                /* 2 left-mid    */
    0.0f,                /* 3 right-mid   */
   -BODY_LEN * 0.40f,    /* 4 left-rear   */
   -BODY_LEN * 0.40f,    /* 5 right-rear  */
};
static const float HIP_LOCAL_Y[N_LEGS] = {
   -BODY_HALF_W,   /* 0 left  side */
    BODY_HALF_W,   /* 1 right side */
   -BODY_HALF_W,
    BODY_HALF_W,
   -BODY_HALF_W,
    BODY_HALF_W,
};

/*
 * Where each foot rests relative to its hip when standing still
 * (body-local). FORWARD spreads front legs ahead and rear legs behind;
 * SIDE pushes left legs out to -y and right legs out to +y.
 *
 * The farthest rest offset is about sqrt(32^2 + 50^2) ~= 60 px, comfortably
 * inside the leg's reach (femur + tibia = 76 px), so the IK can always
 * solve it.
 */
static const float REST_FORWARD[N_LEGS] = {
     32.0f, 32.0f,    /* front */
      0.0f,  0.0f,    /* mid   */
    -32.0f,-32.0f,    /* rear  */
};
static const float REST_SIDE[N_LEGS] = {
   -45.0f, 45.0f,     /* front: closer to body */
   -50.0f, 50.0f,     /* mid:   spread further */
   -45.0f, 45.0f,     /* rear */
};

/*
 * The two tripods. Each is three legs that form a triangle under the
 * body: one front, one mid, one rear, mixing left and right sides. While
 * one tripod swings forward, the other is planted and holds the robot up.
 *   A = {left-front, right-mid, left-rear}
 *   B = {right-front, left-mid, right-rear}
 */
static const int TRIPOD_A[3] = { 0, 3, 4 };
static const int TRIPOD_B[3] = { 1, 2, 5 };

/*
 * Hexapod — everything about the robot in one record.
 *
 * It holds three things that work together each frame:
 *   1. the body — slides along its heading at body_speed;
 *   2. the gait — which of the two tripods is currently swinging, and a
 *      timer that decides when to swap them;
 *   3. the legs — per-leg foot positions, plus the hip and knee points
 *      worked out from them each frame.
 *
 * The per-leg data is kept as parallel arrays (one entry per leg, indexed
 * 0..N_LEGS-1) rather than an array of structs — simple, and most loops
 * touch one field across all legs at once.
 *
 * hip[] and knee[] are derived: recomputed every tick by hexapod_tick and
 * then read many times by the renderer. Storing them once saves redoing
 * the rotation and IK math for every line and marker drawn.
 */
typedef struct {
    /* ── Body: a rigid block sliding through world pixel space ── */
    float body_x;             /* body centre x, world pixels             */
    float body_y;             /* body centre y, world pixels             */
    float body_speed;         /* walk speed along heading, px/s          */
    float heading;            /* way it's facing now, radians (0 = +x)   */
    float target_heading;     /* way you steered it to; heading eases to it */

    /* ── Per-leg gait state (one slot per leg) ──
     * A leg is either planted (stepping=false, foot_pos held fixed on the
     * ground) or swinging (stepping=true, foot_pos sweeps from foot_old to
     * step_target along an arc as step_t goes 0 -> 1). */
    Vec2  foot_pos   [N_LEGS]; /* where the foot is right now (the IK target) */
    Vec2  foot_old   [N_LEGS]; /* where this swing started (lerp start point) */
    Vec2  step_target[N_LEGS]; /* where this swing is aiming to land      */
    bool  stepping   [N_LEGS]; /* true while the foot is in the air       */
    float step_t     [N_LEGS]; /* swing progress, 0 = lifted .. 1 = landed */

    /* ── Per-leg derived (recomputed every tick, read by the drawer) ── */
    Vec2  hip [N_LEGS];        /* hip in world coords = body + rotated offset */
    Vec2  knee[N_LEGS];        /* knee from the IK solve                  */

    /* ── Gait clock ── */
    int   gait_phase;          /* which tripod swings: 0 = A, 1 = B       */
    float phase_timer;         /* seconds since the last tripod swap      */

    /* ── Controls ── */
    bool  paused;              /* true freezes the simulation (still draws) */
    int   theme_idx;           /* index into THEMES[]                     */
} Hexapod;

/* ── §5b solve_ik — given hip and foot, find the knee ── */

/*
 * Inverse kinematics for one 2-bone leg: you know where the hip is and
 * where the foot should be, and this finds where the knee must bend to.
 * It uses the law of cosines on the triangle hip-knee-foot.
 *
 * Edge case: if the foot is too far or too close, the angle math would
 * try acos() of a value outside [-1, 1] and produce NaN, making the leg
 * vanish. So the hip-to-foot distance is clamped to just inside the
 * reachable range first.
 *
 * A leg can bend two ways; left legs (even index) bend their knee toward
 * -y and right legs toward +y, so the two sides mirror each other.
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = sqrtf(dx * dx + dy * dy);

    dist = clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN - 1.0f);

    float base  = atan2f(dy, dx);
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN
                                - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    float ah    = acosf(clampf(cos_h, -1.0f, 1.0f));
    float ka    = is_left ? (base - ah) : (base + ah);

    knee_out->x = hip.x + UPPER_LEN * cosf(ka);
    knee_out->y = hip.y + UPPER_LEN * sinf(ka);
}

/* ── §5c hip_world_pos — body-local hip offset to world position ── */

/* Take leg i's fixed body-local hip offset, rotate it by the body's
 * heading, and add the body centre to get the hip's world position. */
static Vec2 hip_world_pos(const Hexapod *h, int i)
{
    Vec2 local   = { HIP_LOCAL_X[i], HIP_LOCAL_Y[i] };
    Vec2 rotated = rotate2d(local, h->heading);
    return (Vec2){ h->body_x + rotated.x, h->body_y + rotated.y };
}

/* ── §5d rest_target — where leg i's foot should aim to land ── */

/*
 * The foot's resting spot relative to the hip, pushed a little forward in
 * the walk direction. The push (body_speed * STEP_LOOKAHEAD) aims the foot
 * ahead of where the body is now, so by the time the swing finishes the
 * body has caught up and the foot lands underneath it. Faster walking ->
 * bigger push -> longer strides, automatically.
 */
static Vec2 rest_target(const Hexapod *h, int i)
{
    Vec2 hip     = hip_world_pos(h, i);
    Vec2 offset  = { REST_FORWARD[i] + h->body_speed * STEP_LOOKAHEAD,
                     REST_SIDE[i] };
    Vec2 rotated = rotate2d(offset, h->heading);
    return (Vec2){ hip.x + rotated.x, hip.y + rotated.y };
}

/* ── §5e gait_tick — the tripod state machine ── */

/* Move one swinging foot a step forward in time and place it. When the
 * swing finishes (step_t hits 1) the foot lands and goes back to planted. */
static void swing_foot(Hexapod *h, int i, float dt)
{
    h->step_t[i] += dt / STEP_DURATION;
    if (h->step_t[i] >= 1.0f) {
        h->step_t[i]   = 1.0f;
        h->foot_pos[i] = h->step_target[i];
        h->stepping[i] = false;
        return;
    }

    /* Horizontal: ease in and out so the foot starts and lands gently.
     * Vertical: a sine bump (raw step_t, so it peaks halfway through and
     * touches down softly at the end) lifts the foot off the ground. */
    float ease  = smoothstep(h->step_t[i]);
    Vec2  hz    = vec2_lerp(h->foot_old[i], h->step_target[i], ease);
    float arc_y = -STEP_HEIGHT * sinf((float)M_PI * h->step_t[i]);
    h->foot_pos[i] = (Vec2){ hz.x, hz.y + arc_y };
}

/* Start a fresh swing for all three legs in the given tripod: remember
 * where each foot is now and aim it at a new rest target. */
static void launch_tripod(Hexapod *h, const int group[3])
{
    for (int k = 0; k < 3; k++) {
        int i = group[k];
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = rest_target(h, i);
        h->stepping[i]    = true;
        h->step_t[i]      = 0.0f;
    }
}

/*
 * One tick of the gait: move the swinging tripod's feet, then decide
 * whether to hand off to the other tripod.
 *
 * The swap happens only when BOTH are true:
 *   - enough time has passed (phase_timer >= PHASE_DURATION), and
 *   - all three swinging feet have actually landed.
 * The second check matters: if we swapped while feet were still in the
 * air, both tripods would be up at once and the robot would have nothing
 * to stand on.
 */
static void gait_tick(Hexapod *h, float dt)
{
    const int *swing_grp = (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B;

    /* Advance any in-flight legs. */
    for (int k = 0; k < 3; k++) {
        int i = swing_grp[k];
        if (h->stepping[i]) swing_foot(h, i, dt);
    }

    h->phase_timer += dt;
    if (h->phase_timer < PHASE_DURATION) return;

    /* Wait until every swinging leg has landed before swapping. */
    for (int k = 0; k < 3; k++)
        if (h->stepping[swing_grp[k]]) return;

    /* Swap: the just-landed tripod plants, the other launches. */
    h->gait_phase  = (h->gait_phase + 1) % 2;
    h->phase_timer = 0.0f;
    launch_tripod(h, (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B);
}

/* ── §5f hexapod_tick — update everything for one frame ── */

/* Ease the current heading toward the steered target at TURN_RATE. The
 * angle difference is wrapped to [-pi, pi] so a near-180-degree turn goes
 * the short way around instead of spinning the long way. */
static void steer_heading(Hexapod *h, float dt)
{
    float diff = h->target_heading - h->heading;
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    float turn = clampf(diff, -TURN_RATE * dt, TURN_RATE * dt);
    h->heading += turn;
}

/* Slide the body forward along its heading, then wrap it around all four
 * screen edges so walking off one side reappears on the other. */
static void translate_body(Hexapod *h, float dt, int cols, int rows)
{
    h->body_x += h->body_speed * cosf(h->heading) * dt;
    h->body_y += h->body_speed * sinf(h->heading) * dt;

    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (h->body_x <  0.0f) h->body_x += wpx;
    if (h->body_x >= wpx)  h->body_x -= wpx;
    if (h->body_y <  0.0f) h->body_y += hpx;
    if (h->body_y >= hpx)  h->body_y -= hpx;
}

/*
 * Recover after a wrap or sharp turn. Planted feet stay where they were
 * placed, so when the body jumps to the other edge of the screen a foot
 * can end up impossibly far from its hip. The IK would clamp it, but the
 * leg would still look stretched across the screen. So any foot now out of
 * reach is snapped to its rest spot; the gait looks normal again next frame.
 */
static void stretch_snap(Hexapod *h)
{
    const float max_reach = UPPER_LEN + LOWER_LEN - 2.0f;
    for (int i = 0; i < N_LEGS; i++) {
        float dx = h->foot_pos[i].x - h->hip[i].x;
        float dy = h->foot_pos[i].y - h->hip[i].y;
        if (sqrtf(dx*dx + dy*dy) <= max_reach) continue;

        h->foot_pos[i]    = rest_target(h, i);
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = h->foot_pos[i];
        h->stepping[i]    = false;
        h->step_t[i]      = 0.0f;
    }
}

/* One full simulation step, in order: turn, move, refresh hips, fix any
 * over-stretched legs, advance the gait, then solve every knee. */
static void hexapod_tick(Hexapod *h, float dt, int cols, int rows)
{
    if (h->paused) return;

    steer_heading  (h, dt);
    translate_body (h, dt, cols, rows);

    /* Hips move with the body, so recompute them before anything reads them. */
    for (int i = 0; i < N_LEGS; i++)
        h->hip[i] = hip_world_pos(h, i);

    stretch_snap(h);
    gait_tick   (h, dt);

    for (int i = 0; i < N_LEGS; i++)
        solve_ik(h->hip[i], h->foot_pos[i], (i % 2 == 0), &h->knee[i]);
}

/* ── §5g rendering helpers ── */

/*
 * Pick the ASCII character that best shows the direction (dx, dy). A line
 * and its reverse look the same, so we fold the angle into a half-circle
 * and pick one of four glyphs by slice:
 *   '-' roughly horizontal   '\' down-right   '|' roughly vertical   '/' down-left
 * dy is negated first because terminal y grows downward but the angle math
 * assumes y grows up.
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg <  22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg <  67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                   return (chtype)(unsigned char)'|';
    return                              (chtype)(unsigned char)'/';
}

/*
 * Draw a straight line from a to b using direction glyphs. It walks the
 * line in small steps (DRAW_LEG_STEP_PX apart) and stamps a glyph in each
 * cell it crosses, skipping repeats so one cell isn't drawn twice.
 */
static void draw_leg_line(WINDOW *w,
                          Vec2 a, Vec2 b,
                          int pair, attr_t attr,
                          int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_LEG_STEP_PX) + 1;
    int    prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* Stamp one glyph at a pixel position, ignored if it falls off-screen. */
static void mark_cell(WINDOW *w, Vec2 p, chtype glyph,
                      int pair, attr_t attr, int cols, int rows)
{
    int cx = px_to_cell_x(p.x);
    int cy = px_to_cell_y(p.y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, glyph);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5h hexapod_draw — paint the whole robot ── */

/* Femur (hip->knee) and tibia (knee->foot) lines for all six legs. */
static void draw_legs(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, h->hip[i],  h->knee[i],     2, A_NORMAL, cols, rows);
        draw_leg_line(w, h->knee[i], h->foot_pos[i], 3, A_NORMAL, cols, rows);
    }
}

/* Foot markers: bold '*' where planted, dim 'o' while swinging. */
static void draw_feet(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        if (h->stepping[i])
            mark_cell(w, h->foot_pos[i], (chtype)(unsigned char)'o',
                      5, A_DIM, cols, rows);
        else
            mark_cell(w, h->foot_pos[i], (chtype)(unsigned char)'*',
                      4, A_BOLD, cols, rows);
    }
}

/* A bold 'o' at each knee joint. */
static void draw_knees(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++)
        mark_cell(w, h->knee[i], (chtype)(unsigned char)'o',
                  6, A_BOLD, cols, rows);
}

/* The body: a rotated rectangle with two diagonal cross-braces. */
static void draw_body_box(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    float bhl = BODY_LEN  * 0.5f;
    float bhw = BODY_HALF_W;

    /* Four corners, placed in body-local space then rotated into the world. */
    Vec2 tl_l = { -bhl, -bhw }, tr_l = {  bhl, -bhw };
    Vec2 bl_l = { -bhl,  bhw }, br_l = {  bhl,  bhw };
    Vec2 tl   = rotate2d(tl_l, h->heading);
    Vec2 tr   = rotate2d(tr_l, h->heading);
    Vec2 bl   = rotate2d(bl_l, h->heading);
    Vec2 br   = rotate2d(br_l, h->heading);
    tl.x += h->body_x;  tl.y += h->body_y;
    tr.x += h->body_x;  tr.y += h->body_y;
    bl.x += h->body_x;  bl.y += h->body_y;
    br.x += h->body_x;  br.y += h->body_y;

    /* Edges bold, the two diagonals dim, so the box reads as solid. */
    draw_leg_line(w, tl, tr, 1, A_BOLD, cols, rows);
    draw_leg_line(w, bl, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, bl, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tr, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, br, 1, A_DIM,  cols, rows);
    draw_leg_line(w, tr, bl, 1, A_DIM,  cols, rows);
}

/* A '+' where each leg joins the body, and an '@' at the body centre. */
static void draw_hips_and_center(const Hexapod *h, WINDOW *w,
                                 int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++)
        mark_cell(w, h->hip[i], (chtype)(unsigned char)'+',
                  1, A_NORMAL, cols, rows);

    Vec2 center = { h->body_x, h->body_y };
    mark_cell(w, center, (chtype)(unsigned char)'@', 1, A_BOLD, cols, rows);
}

/*
 * Draw the robot back-to-front so the body box and joint markers land on
 * top of the leg lines instead of being painted over by them.
 */
static void hexapod_draw(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    draw_legs           (h, w, cols, rows);
    draw_feet           (h, w, cols, rows);
    draw_knees          (h, w, cols, rows);
    draw_body_box       (h, w, cols, rows);
    draw_hips_and_center(h, w, cols, rows);
}

/* ── §6 scene — thin wrapper holding the one robot ── */

/*
 * Scene — the simulated world. Here it is just one hexapod, but it gets
 * its own struct so the loop (scene_init / scene_tick / scene_draw) looks
 * the same as in every other demo. Anything extra (obstacles, a goal to
 * walk toward) would sit alongside `hexapod` here.
 */
typedef struct {
    Hexapod hexapod;           /* the robot: body + 6 legs + gait     */
} Scene;

/*
 * Put the robot at the centre of the screen with every foot at its rest
 * spot, then start tripod A swinging right away so a step is visible on
 * the very first frame (otherwise the gait would sit still until the
 * phase timer first expires).
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Hexapod *h = &sc->hexapod;

    h->body_speed     = BODY_SPEED_DEFAULT;
    h->body_x         = (float)(cols * CELL_W) * 0.5f;
    h->body_y         = (float)(rows * CELL_H) * 0.5f;
    h->heading        = 0.0f;
    h->target_heading = 0.0f;
    h->gait_phase     = 0;
    h->phase_timer    = 0.0f;
    h->paused         = false;
    h->theme_idx      = 0;

    for (int i = 0; i < N_LEGS; i++)
        h->hip[i] = hip_world_pos(h, i);

    /* Plant every foot at its rest spot (no lookahead — it's standing still). */
    for (int i = 0; i < N_LEGS; i++) {
        Vec2 hip = h->hip[i];
        h->foot_pos[i]    = (Vec2){ hip.x + REST_FORWARD[i],
                                    hip.y + REST_SIDE[i] };
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = h->foot_pos[i];
        h->stepping[i]    = false;
        h->step_t[i]      = 0.0f;
        solve_ik(h->hip[i], h->foot_pos[i], (i % 2 == 0), &h->knee[i]);
    }

    launch_tripod(h, TRIPOD_A);
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    hexapod_tick(&sc->hexapod, dt, cols, rows);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    hexapod_draw(&sc->hexapod, w, cols, rows);
}

/* ── §7 screen — ncurses setup and the display layer ── */

/*
 * Screen — the current terminal size, in character cells. Cached here so
 * the rest of the code reads (cols, rows) as plain ints; refreshed only
 * when a resize (SIGWINCH) arrives.
 */
typedef struct {
    int cols;   /* terminal width  in cells */
    int rows;   /* terminal height in cells */
} Screen;

/* typeahead(-1) is the one non-obvious call: without it ncurses peeks at
 * stdin while writing output, which can tear a frame mid-update. */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* On resize, endwin()+refresh() makes ncurses re-read the new size. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* An arrow ('>' '<' '^' 'v') showing which way the robot faces, for the HUD. */
static const char *heading_arrow(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    if (deg <  45.0f || deg >= 315.0f) return ">";
    if (deg < 135.0f)                   return "v";
    if (deg < 225.0f)                   return "<";
    return                              "^";
}

/*
 * Draw one full frame: clear, draw the robot, then the status line
 * (top-right) and the key hints (bottom). The HUD colours stay bright
 * yellow / cyan so they read over the robot.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Hexapod *h = &sc->hexapod;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " HEXAPOD  dir:%s  spd:%.0f  phase:%s  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(h->heading), h->body_speed,
             h->gait_phase == 0 ? "A" : "B",
             THEMES[h->theme_idx].name,
             time_scale, fps,
             h->paused ? "PAUSED" : "walking");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  arrows:steer  w/s:speed  t:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, input, and the main loop ── */

/*
 * App — everything outside the simulated world, bundled so main() reads
 * as a few clear phases. It's a file-scope global (g_app) because signal
 * handlers can't take an argument and need to set the flags below.
 *
 * running and need_resize are volatile sig_atomic_t because a signal
 * handler writes them: volatile forces the loop to re-read them from
 * memory each pass, and sig_atomic_t guarantees the write can't be seen
 * half-finished.
 */
typedef struct {
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal size (§7)                   */

    float                 time_scale;   /* dt multiplier; 1.0 = realtime  */
    volatile sig_atomic_t running;      /* loop runs while true           */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, cleared on handle */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * Handle a pending resize: re-read the size, pull the body back on-screen
 * if the new bounds left it outside, and re-apply the theme (some
 * terminals reset their colours on resize).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Hexapod *h   = &app->scene.hexapod;
    float    wpx = (float)(app->screen.cols * CELL_W);
    float    hpx = (float)(app->screen.rows * CELL_H);
    if (h->body_x < 0.0f || h->body_x >= wpx) h->body_x = wpx * 0.5f;
    if (h->body_y < 0.0f || h->body_y >= hpx) h->body_y = hpx * 0.5f;
    theme_apply(h->theme_idx);
    app->need_resize = 0;
}

/*
 * Act on one keypress; return false to quit. Arrow keys only set the
 * target heading — the body turns toward it gradually in steer_heading.
 */
static bool app_handle_key(App *app, int ch)
{
    Hexapod *h = &app->scene.hexapod;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': h->paused = !h->paused; break;

    case KEY_RIGHT: h->target_heading =  0.0f;                break;
    case KEY_DOWN:  h->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  h->target_heading =  (float)M_PI;         break;
    case KEY_UP:    h->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        h->body_speed *= 1.25f;
        if (h->body_speed > BODY_SPEED_MAX) h->body_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        h->body_speed /= 1.25f;
        if (h->body_speed < BODY_SPEED_MIN) h->body_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        h->theme_idx = (h->theme_idx + 1) % N_THEMES;
        theme_apply(h->theme_idx);
        break;

    case ']':
        app->time_scale *= TIME_SCALE_STEP;
        if (app->time_scale > TIME_SCALE_MAX) app->time_scale = TIME_SCALE_MAX;
        break;
    case '[':
        app->time_scale /= TIME_SCALE_STEP;
        if (app->time_scale < TIME_SCALE_MIN) app->time_scale = TIME_SCALE_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * The main loop. Each frame, in order:
 *   1 read all pending keys
 *   2 handle a pending resize before touching ncurses
 *   3 measure real time since last frame (capped at 100 ms so the robot
 *     can't teleport after the machine was asleep)
 *   4 advance the simulation by that time, scaled by time_scale
 *   5 update the fps counter
 *   6 draw the frame
 *   7 sleep so the frame lasts about 1/TARGET_FPS
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFFU));
    atexit(cleanup);

    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->time_scale = TIME_SCALE_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t target_ns = NS_PER_SEC / TARGET_FPS;

    int64_t last_time   = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① drain input */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        /* ② resize */
        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();
        }

        /* ③ measure dt */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale,
                   app->screen.cols, app->screen.rows);

        /* ⑤ fps counter */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
