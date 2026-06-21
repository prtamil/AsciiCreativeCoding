/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_spider.c — a six-legged crawler you can steer around the terminal.
 *
 * The body is a snake-like chain of beads that follows the path the head
 * walks (forward kinematics). Each leg works out its own knee angle to
 * reach wherever its foot is planted (inverse kinematics): you say where
 * the foot should be, the math finds the joint angles. A leg swings
 * forward to a new foothold when the body has dragged it too far behind,
 * otherwise the foot stays planted — and no more than half the legs lift
 * at once, so it never tips. Sister demos: hexpod_tripod.c (rigid body,
 * lockstep gait), snake_forward_kinematics.c (the trail-buffer body).
 *
 * Keys:  q / ESC quit   space pause   arrows steer   w/s speed
 *        t cycle theme   [ / ] slow down / speed up time
 *
 * Build (needs -lm for the trig in the IK solver):
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_spider.c \
 *       -o ik_spider -lncurses -lm
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

/* ── §1 config — every tunable number, named once ── */

enum {
    /* Frame-rate cap. The sim uses real elapsed time, so this only sets
     * how long we sleep at the end of each frame. */
    TARGET_FPS    = 60,

    HUD_COLS      = 96,   /* max width of the status string */
    FPS_UPDATE_MS = 500,  /* how often the fps readout refreshes */

    /* ncurses colour-pair IDs. 1..N_PAIRS are the themed body/leg colours;
     * the HUD pairs stay fixed so the status bar reads against any theme. */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,    /* bright yellow status bar */
    PAIR_HINT     = 9,    /* bright cyan key hint     */

    N_THEMES      = 10,   /* colour palettes, cycled with `t` */

    /* Body has 3 leg pairs (front/mid/rear) and a 4-segment spine (so 5
     * joints, head to tail). The whole point vs hexpod_tripod is that this
     * spine CURVES instead of being one rigid block. */
    N_LEGS        = 6,
    N_BODY_SEGS   = 4,
    TRAIL_CAP     = 1024, /* how many past head positions we remember */
};

/* Body geometry, in pixels. */
#define BODY_SEG_LEN      20.0f   /* spacing between body beads             */
#define BODY_SPEED        45.0f   /* how fast the head walks, px/s          */
#define BODY_SPEED_MIN    10.0f
#define BODY_SPEED_MAX   200.0f
#define TURN_RATE          2.5f   /* how fast it can turn, radians/s        */

/* Leg bone lengths, in pixels. Long on purpose, to read as spindly. */
#define UPPER_LEN         56.0f   /* hip-to-knee bone                       */
#define LOWER_LEN         50.0f   /* knee-to-foot bone                      */

/* How far the hips sit out to the side of the spine, as a fraction of
 * screen height. Kept narrow so the legs, not the body, do the visual work. */
#define HIP_DIST_FACTOR   0.04f

/* Gait tuning. */
#define STEP_REACH_FACTOR 0.68f   /* a planted foot's distance out from the hip,
                                   * as a fraction of total leg length        */
#define STEP_TRIGGER_DIST 28.0f   /* foot drift (px) that starts a forward swing */
#define MAX_STRETCH       65.0f   /* hip-to-foot stretch (px) that forces a swing */
#define STEP_DURATION     0.22f   /* seconds for one swing to complete           */

/* Spacing when stamping glyphs along a line. Both stay under CELL_W (8 px)
 * so a line never skips a cell. */
#define DRAW_STEP_PX      5.0f    /* body beads */
#define DRAW_LEG_STEP_PX  8.0f    /* leg lines  */

/* How far the two eyes sit out to either side of the head arrow (px). */
#define EYE_OFFSET_PX    10.0f

/* Slow-motion / fast-forward multiplier, adjusted with `[` and `]`. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* One character cell is 8 wide by 16 tall in pixels. We do all the motion
 * math in square pixels so diagonals look right, then convert at draw time. */
#define CELL_W   8
#define CELL_H  16

/*
 * Which way each leg naturally points, as an angle (radians) measured from
 * the body's forward direction. Even legs are on the left and get a positive
 * angle; odd legs are the mirror on the right. The three pairs fan out at
 * roughly 60-degree spacing so they don't overlap.
 */
static const float LEG_ANGLE[N_LEGS] = {
     0.6f,    /* 0 front-left   ~34deg, forward and out */
    -0.6f,    /* 1 front-right  */
     1.57f,   /* 2 mid-left     90deg, straight out      */
    -1.57f,   /* 3 mid-right    */
     2.5f,    /* 4 rear-left    ~143deg, back and out     */
    -2.5f,    /* 5 rear-right   */
};

/*
 * Where each hip attaches along the spine: 0 = head, 1 = tail. The three
 * pairs are spread out so the front, middle, and rear legs hang from clearly
 * different points rather than bunching up.
 */
static const float HIP_BODY_T[N_LEGS] = {
    0.20f, 0.20f,   /* front pair — near the head */
    0.50f, 0.50f,   /* mid pair   — body centre   */
    0.80f, 0.80f,   /* rear pair  — near the tail */
};

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

/* ── §3 color — palettes + fixed HUD pairs ── */

/*
 * One colour palette for the spider. The seven colours fill ncurses pairs
 * 1..7, in this order:
 *   [0..2] body, dim tail to bright head (lets the eye trace the body)
 *   [3]    upper leg bone
 *   [4]    lower leg bone
 *   [5]    a planted foot '*' (bright, so it reads as "on the ground")
 *   [6]    a swinging foot '.' (dim, so it reads as "in the air")
 * Every colour sits in the bright half of the 256-colour space, so even the
 * darkest tier stays visible under A_DIM against a black terminal.
 */
typedef struct {
    const char *name;            /* shown in the HUD                  */
    int         col[N_PAIRS];    /* xterm-256 colour index per pair   */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        body gradient   legs      foot   ghost */
    {"Arachnid",{ 52,  88, 124,  58,  64,  70, 240}},
    {"Scarlet", { 88, 124, 160,  52,  88, 196, 240}},
    {"Toxic",   { 24,  28,  34,  28,  34,  82, 240}},
    {"Ocean",   { 24,  25,  27,  33,  39,  51, 240}},
    {"Nova",    { 54,  93, 129,  93, 129, 165, 240}},
    {"Ember",   { 52,  94, 130, 130, 130, 208, 240}},
    {"Aurora",  { 24,  29,  35,  35,  71, 221, 240}},
    {"Ghost",   {240, 244, 248, 244, 248, 254, 246}},
    {"Fire",    { 52,  88, 196,  88, 124, 226, 240}},
    {"Neon",    { 57,  93, 201,  93, 129, 201, 240}},
};

/* Point the body/leg colour pairs at a chosen palette. The HUD pairs are
 * left alone so the status bar colour never changes with the theme. */
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
        /* Fallback for terminals with only 8 colours. */
        init_pair(1, COLOR_RED,    -1);
        init_pair(2, COLOR_RED,    -1);
        init_pair(3, COLOR_RED,    -1);
        init_pair(4, COLOR_GREEN,  -1);
        init_pair(5, COLOR_GREEN,  -1);
        init_pair(6, COLOR_GREEN,  -1);
        init_pair(7, COLOR_WHITE,  -1);
    }

    /* Fixed HUD colours: yellow status bar, cyan key hint, both on the
     * default background so they stay legible over any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — pixel to cell conversion ── */

/*
 * Turn a pixel coordinate into a character-cell coordinate (rounding to the
 * nearest cell). These two functions are the only place pixel space meets
 * cell space — everything upstream stays in pixels.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the Spider: body, legs, gait ── */

/*
 * A 2-D point in pixel space. x runs right, y runs down (the terminal's own
 * convention). All the body and leg math works in these sub-cell pixels so
 * motion looks smooth; only the drawing code rounds to whole cells.
 *
 * Note on angles: positive rotation is counter-clockwise in the math, but
 * because y points down on screen it looks clockwise. That sign flip is why
 * the left/right legs use opposite signs later on.
 */
typedef struct {
    float x;   /* rightward pixel coordinate */
    float y;   /* downward  pixel coordinate */
} Vec2;

/* ── §5a vec2 helpers — small vector math ── */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline Vec2 vec2_add  (Vec2 a, Vec2 b)
{ return (Vec2){ a.x + b.x, a.y + b.y }; }

static inline Vec2 vec2_sub  (Vec2 a, Vec2 b)
{ return (Vec2){ a.x - b.x, a.y - b.y }; }

static inline Vec2 vec2_scale(Vec2 a, float s)
{ return (Vec2){ a.x * s, a.y * s }; }

static inline Vec2 vec2_lerp (Vec2 a, Vec2 b, float t)
{ return (Vec2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t }; }

static inline float vec2_len (Vec2 v)
{ return sqrtf(v.x * v.x + v.y * v.y); }

static inline float vec2_dist(Vec2 a, Vec2 b)
{ return vec2_len(vec2_sub(a, b)); }

/* Unit-length version of a vector. A zero vector has no direction, so we
 * return (1, 0) rather than divide by zero and spread NaNs. */
static inline Vec2 vec2_norm(Vec2 v)
{
    float len = vec2_len(v);
    if (len < 1e-6f) return (Vec2){ 1.0f, 0.0f };
    return (Vec2){ v.x / len, v.y / len };
}

/* Eases a 0..1 progress value so it starts and ends gently instead of at
 * constant speed. Used to make a swinging foot accelerate then settle. */
static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* Rotate a vector by an angle in radians. */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/*
 * The whole creature. It bundles three loosely-connected parts:
 *   - the body, which trails behind the head along the path it walked;
 *   - the six legs, each with its own foot, knee, and swing state;
 *   - steering, which turns the head toward whatever direction you press.
 * Each frame these run in order (body, then hips, then legs), with no
 * feedback between them — see scene_tick.
 *
 * The leg data is kept as one array per attribute (all hips together, all
 * feet together, ...) because the code almost always loops over "every leg's
 * foot" or "every leg's flag" at once.
 */
typedef struct {
    /* Body that follows the head's path.
     * trail is a ring of recent head positions; we read points out of it at
     * fixed distances behind the head to place the body beads each frame. */
    Vec2 trail[TRAIL_CAP];            /* recent head positions (newest at trail_head) */
    int  trail_head;                  /* index of the newest trail entry              */
    int  trail_count;                 /* how many entries are filled (caps at TRAIL_CAP) */
    Vec2 body_joint[N_BODY_SEGS + 1]; /* the body beads: [0] = head, last = tail tip  */

    /* Where the head is heading. */
    float heading;                    /* direction it's actually facing, radians  */
    float target_heading;             /* direction you asked for; heading eases toward it */
    float move_speed;                 /* walking speed along heading, px/s         */

    /* Per-leg state, one slot per leg. hip and knee are recomputed every
     * frame; foot_pos is the live foot, and the four swing fields below
     * describe a foot that is mid-step. */
    Vec2  hip        [N_LEGS];        /* where the leg attaches to the body   */
    Vec2  knee       [N_LEGS];        /* knee joint, found by the IK solver   */
    Vec2  foot_pos   [N_LEGS];        /* the foot right now (planted or moving) */
    Vec2  foot_old   [N_LEGS];        /* where this swing started from        */
    Vec2  step_target[N_LEGS];        /* where this swing is heading to       */
    bool  stepping   [N_LEGS];        /* true while the foot is in the air    */
    float step_t     [N_LEGS];        /* swing progress, 0 at lift-off to 1 at landing */

    /* How far out to the side the hips sit. Scaled to the screen height, so
     * it's recomputed at startup and on every resize. */
    float hip_dist;                   /* sideways hip offset from the spine, px */

    bool  paused;                     /* space toggles this; freezes the sim, not the render */
    int   theme_idx;                  /* which palette is active (cycled with `t`) */
} Spider;

/* ── §5b trail — the path the head has walked ── */

/* Record one new head position at the front of the ring. */
static void trail_push(Spider *sp, Vec2 pos)
{
    sp->trail_head = (sp->trail_head + 1) % TRAIL_CAP;
    sp->trail[sp->trail_head] = pos;
    if (sp->trail_count < TRAIL_CAP) sp->trail_count++;
}

/* The k-th most recent head position (k = 0 is the newest). */
static inline Vec2 trail_at(const Spider *sp, int k)
{
    return sp->trail[(sp->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * The point on the head's recorded path that is `dist` pixels back from the
 * head, measured along the path. We walk backward summing up the gaps between
 * recorded points until we've travelled far enough, then interpolate inside
 * the last gap. This is how each body bead finds its spot behind the head.
 */
static Vec2 trail_sample(const Spider *sp, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(sp, 0);

    for (int k = 1; k < sp->trail_count; k++) {
        Vec2  b   = trail_at(sp, k);
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx * dx + dy * dy);

        if (accum + seg >= dist) {
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx * t, a.y + dy * t };
        }
        accum += seg;
        a      = b;
    }
    return trail_at(sp, sp->trail_count - 1);
}

/* ── §5c body motion — steer and walk the head ── */

/*
 * Fold an angle difference into the range -pi..pi. Directions wrap around a
 * circle, so plain subtraction can claim you must turn most of the way
 * around when a small turn the other way is shorter. This picks the short way.
 */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/*
 * Limit how much the heading may change this frame. However far off the
 * target is, we only allow TURN_RATE radians per second, so a turn sweeps
 * smoothly instead of snapping instantly.
 */
static float clamp_to_max_turn_per_dt(float diff, float dt)
{
    float max_step = TURN_RATE * dt;
    return clampf(diff, -max_step, max_step);
}

/* Nudge the current heading toward the one you asked for: pick the short
 * way around, cap how fast it can turn, and apply the step. */
static void steer_heading(Spider *sp, float dt)
{
    float diff    = shortest_signed_angle(sp->target_heading - sp->heading);
    float clamped = clamp_to_max_turn_per_dt(diff, dt);
    sp->heading  += clamped;
}

/* Move the head forward by speed times elapsed time, in the heading's
 * direction. (cos, sin) of the heading gives that direction as a unit vector. */
static void integrate_position_along_heading(Spider *sp, float dt)
{
    sp->body_joint[0].x += sp->move_speed * cosf(sp->heading) * dt;
    sp->body_joint[0].y += sp->move_speed * sinf(sp->heading) * dt;
}

/* Wrap the head around the screen edges: walk off the right and you come
 * back on the left, off the bottom and you come back on the top. We add or
 * subtract one screen width/height, which is enough because the head can
 * only overshoot by one frame's worth of movement. */
static void wrap_position_to_toroidal_world(Spider *sp, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (sp->body_joint[0].x <  0.0f) sp->body_joint[0].x += wpx;
    if (sp->body_joint[0].x >= wpx)  sp->body_joint[0].x -= wpx;
    if (sp->body_joint[0].y <  0.0f) sp->body_joint[0].y += hpx;
    if (sp->body_joint[0].y >= hpx)  sp->body_joint[0].y -= hpx;
}

/* One frame of head movement: step forward, wrap at the edges, and record
 * the new head spot in the trail so the body has a path to follow. */
static void translate_body(Spider *sp, float dt, int cols, int rows)
{
    integrate_position_along_heading  (sp, dt);
    wrap_position_to_toroidal_world   (sp, cols, rows);
    trail_push                        (sp, sp->body_joint[0]);
}

/* ── §5d body joints — beads follow the head's path ── */

/* Place each body bead behind the head, evenly spaced along the recorded
 * path. Bead [0] (the head itself) was already moved by translate_body. */
static void compute_body_joints(Spider *sp)
{
    for (int i = 1; i <= N_BODY_SEGS; i++)
        sp->body_joint[i] = trail_sample(sp, (float)i * BODY_SEG_LEN);
}

/* ── §5e hip placement — attach legs to the curving body ── */

/* The direction the body is pointing at one of its segments. Used to work
 * out which way is "sideways" so the hips can hang off the body's edge. */
static Vec2 body_local_forward(const Spider *sp, int seg_idx)
{
    if (seg_idx + 1 > N_BODY_SEGS)
        return (Vec2){ cosf(sp->heading), sinf(sp->heading) };
    return vec2_norm(vec2_sub(sp->body_joint[seg_idx],
                              sp->body_joint[seg_idx + 1]));
}

/*
 * Find a point part-way along the body, given t_norm in 0..1 (0 = head,
 * 1 = tail). We scale t_norm up to which body segment it falls in, plus how
 * far along that segment, and blend between the two beads at its ends. Also
 * hands back which segment we landed on, so the caller can reuse it.
 */
static Vec2 attachment_point_along_spine(const Spider *sp, float t_norm,
                                         int *seg_idx_out)
{
    float t_body  = t_norm * (float)N_BODY_SEGS;
    int   seg_idx = (int)t_body;
    if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
    float frac    = t_body - (float)seg_idx;

    *seg_idx_out = seg_idx;
    return vec2_lerp(sp->body_joint[seg_idx],
                     sp->body_joint[seg_idx + 1], frac);
}

/*
 * The "sideways" direction at a body segment: take the forward direction and
 * turn it 90 degrees. Pass side = +1 for the left, -1 for the right. This is
 * how a hip steps out to the correct side of the spine.
 */
static Vec2 lateral_normal_at_spine(const Spider *sp, int seg_idx, float side)
{
    Vec2 fwd       = body_local_forward(sp, seg_idx);
    Vec2 left_norm = (Vec2){ -fwd.y, fwd.x };           /* forward turned 90 deg */
    return vec2_scale(left_norm, side);
}

/*
 * Work out where each leg's hip sits this frame: find its attach point on
 * the body, step sideways from there by hip_dist. Even legs go left, odd
 * legs go right. As the body curves, the sideways direction changes along
 * it, so the hips track the bend and the spider looks alive when it turns.
 */
static void compute_hips(Spider *sp)
{
    for (int i = 0; i < N_LEGS; i++) {
        int  seg_idx;
        Vec2 attach = attachment_point_along_spine(sp, HIP_BODY_T[i], &seg_idx);

        float side  = (i % 2 == 0) ? 1.0f : -1.0f;
        Vec2  out   = lateral_normal_at_spine(sp, seg_idx, side);

        sp->hip[i]  = vec2_add(attach, vec2_scale(out, sp->hip_dist));
    }
}

/* ── §5f leg IK — find the knee for a foot target ──
 *
 * Inverse kinematics: you say where the foot should be, and the math works
 * out the joint angles to put it there. A two-bone leg is the easy case —
 * the hip, knee, and foot form a triangle, and the law of cosines gives the
 * one angle we need directly, no guessing or iterating. */

/*
 * Clamp the hip-to-foot distance to what the leg can actually span. A leg
 * with bones U and L can only reach distances between |U-L| (folded shut)
 * and U+L (stretched straight). Outside that, the triangle doesn't exist and
 * the cosine math blows up; the 1-pixel margin keeps us safely inside.
 */
static float clamp_to_reachable_annulus(float dist)
{
    return clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN        - 1.0f);
}

/*
 * The angle at the hip corner of the (hip, knee, foot) triangle, given the
 * hip-to-foot distance. This is the law of cosines solved for that angle: it
 * tells us how far the upper bone bends away from the straight line to the
 * foot. Always comes out positive; the caller adds it for left legs and
 * subtracts it for right legs to bend the knee to the correct side.
 */
static float law_of_cosines_apex_angle(float dist)
{
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN
                                - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    return acosf(clampf(cos_h, -1.0f, 1.0f));
}

/* Step one upper-bone length out from the hip in a given direction to land
 * on the knee. The lower bone is then just the straight line from knee to
 * foot. */
static Vec2 place_knee_at_angle(Vec2 hip, float angle)
{
    return (Vec2){ hip.x + UPPER_LEN * cosf(angle),
                   hip.y + UPPER_LEN * sinf(angle) };
}

/*
 * Given a hip and a foot target, place the knee so the two bones reach the
 * foot. Steps: measure the hip-to-foot distance (clamped to reach), find the
 * straight-line direction to the foot, find how far the upper bone bends off
 * that line (law of cosines), then step out to the knee. is_left chooses
 * which way the knee bends so left and right knees splay outward.
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = clamp_to_reachable_annulus(sqrtf(dx * dx + dy * dy));

    float base       = atan2f(dy, dx);
    float alpha      = law_of_cosines_apex_angle(dist);
    float knee_angle = is_left ? (base + alpha) : (base - alpha);

    *knee_out = place_knee_at_angle(hip, knee_angle);
}

/* ── §5g step gait — when and how each foot moves ──
 *
 * Each leg decides for itself when to step. While the body walks on, a
 * planted foot stays put and falls behind; once it has drifted too far back
 * (or is being stretched past reach), the leg swings forward to a fresh
 * foothold. A cap of half the legs in the air at once keeps it from tipping. */

/* The direction leg i naturally points: the body's facing turned by that
 * leg's fixed angle, so front legs reach forward and rear legs reach back. */
static Vec2 leg_facing_direction(const Spider *sp, int i)
{
    Vec2 body_forward = (Vec2){ cosf(sp->heading), sinf(sp->heading) };
    return rotate2d(body_forward, LEG_ANGLE[i]);
}

/* How far out from the hip a comfortable foot lands: a bit less than full
 * leg length, so there's always slack and the foot never sits at the very
 * edge of what the leg can reach. */
static float rest_reach_distance(void)
{
    return (UPPER_LEN + LOWER_LEN) * STEP_REACH_FACTOR;
}

/* Where leg i's foot "wants" to be right now: out from the hip in the leg's
 * natural direction by the comfortable reach. The gait compares the real
 * foot against this and steps toward it when the gap grows too large. */
static Vec2 compute_ideal_foot(const Spider *sp, int i)
{
    Vec2  dir   = leg_facing_direction(sp, i);
    float reach = rest_reach_distance();
    return vec2_add(sp->hip[i], vec2_scale(dir, reach));
}

/* How many legs are mid-swing right now. */
static int count_airborne(const Spider *sp)
{
    int n = 0;
    for (int i = 0; i < N_LEGS; i++)
        if (sp->stepping[i]) n++;
    return n;
}

/* Recover a foot that got left too far behind, which happens when the body
 * wraps across a screen edge: a planted foot stays at its old coordinates
 * while the hip teleports. If it's now out of reach, snap it back to rest.
 * Returns true if the leg was mid-swing when snapped, so the caller can keep
 * its airborne count right. */
static bool snap_overstretched_foot(Spider *sp, int i)
{
    if (vec2_dist(sp->foot_pos[i], sp->hip[i]) <= UPPER_LEN + LOWER_LEN - 2.0f)
        return false;

    bool was_airborne   = sp->stepping[i];
    sp->foot_pos[i]     = compute_ideal_foot(sp, i);
    sp->foot_old[i]     = sp->foot_pos[i];
    sp->step_target[i]  = sp->foot_pos[i];
    sp->stepping[i]     = false;
    sp->step_t[i]       = 0.0f;
    return was_airborne;
}

/* For a planted leg, decide whether to start a forward swing. It steps if the
 * foot has drifted far from where it wants to be, or is being over-stretched
 * from the hip — but only while fewer than half the legs are already in the
 * air. Returns true if a swing was launched. */
static bool maybe_trigger_step(Spider *sp, int i, int n_air)
{
    if (sp->stepping[i] || n_air >= N_LEGS / 2) return false;

    Vec2  ideal   = compute_ideal_foot(sp, i);
    float drift   = vec2_dist(sp->foot_pos[i], ideal);
    float stretch = vec2_dist(sp->foot_pos[i], sp->hip[i]);
    if (drift <= STEP_TRIGGER_DIST && stretch <= MAX_STRETCH) return false;

    sp->stepping[i]    = true;
    sp->step_t[i]      = 0.0f;
    sp->foot_old[i]    = sp->foot_pos[i];
    sp->step_target[i] = ideal;
    return true;
}

/* Move a swinging foot a little further toward its target, easing in and out.
 * Returns true on the frame it lands (so the caller frees up an air slot). */
static bool advance_swing(Spider *sp, int i, float dt)
{
    sp->step_t[i] += dt / STEP_DURATION;
    if (sp->step_t[i] >= 1.0f) {
        sp->step_t[i]   = 1.0f;
        sp->foot_pos[i] = sp->step_target[i];
        sp->stepping[i] = false;
        return true;
    }
    float ease     = smoothstep(sp->step_t[i]);
    sp->foot_pos[i] = vec2_lerp(sp->foot_old[i], sp->step_target[i], ease);
    return false;
}

/*
 * Update the gait once per frame. Each leg is in exactly one of three states:
 * snap back if over-stretched, keep swinging if in the air, or maybe start a
 * swing if planted and drifting. We track how many are airborne so the
 * trigger can hold to the "at most half in the air" rule. After moving the
 * feet, re-run the IK so every knee matches its new foot.
 */
static void update_steps(Spider *sp, float dt)
{
    int n_air = count_airborne(sp);

    /* Advance each leg's state, keeping the airborne count current. */
    for (int i = 0; i < N_LEGS; i++) {
        if      (snap_overstretched_foot(sp, i))    n_air--;
        else if (sp->stepping[i]) {
            if (advance_swing(sp, i, dt))           n_air--;
        }
        else if (maybe_trigger_step(sp, i, n_air))  n_air++;
    }

    /* Now place every knee from its (possibly moved) foot. Even legs are
     * left, odd legs right — same convention solve_ik expects. */
    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
}

/* ── §5h drawing helpers — lines, beads, markers ── */

/* Pick the arrow glyph that points the way the head is facing. */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    if (deg <  45.0f || deg >= 315.0f) return (chtype)(unsigned char)'>';
    if (deg < 135.0f)                  return (chtype)(unsigned char)'v';
    if (deg < 225.0f)                  return (chtype)(unsigned char)'<';
    return                             (chtype)(unsigned char)'^';
}

/* Pick the ASCII glyph ('-', '\', '|', '/') closest to the slope of a line.
 * dy is flipped first because screen y grows downward. */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg < 67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                  return (chtype)(unsigned char)'|';
    return                             (chtype)(unsigned char)'/';
}

/* Draw one leg bone from a to b, alternating its slope glyph with '.' so it
 * reads as a jointed limb (e.g. "-.-.-") rather than a solid line. */
static void draw_leg_line(WINDOW *w, Vec2 a, Vec2 b,
                          int pair, attr_t attr, int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph   = seg_glyph(dx, dy);
    int    nsteps  = (int)ceilf(len / DRAW_LEG_STEP_PX) + 1;
    int    prev_cx = -9999, prev_cy = -9999;
    int    phase   = 0;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        chtype ch = (phase & 1) ? (chtype)(unsigned char)'.' : glyph;
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(pair) | attr);
        phase++;
    }
}

/* Stamp a row of 'o' between two body points to fill in the spine. The caller
 * passes in prev_cx/prev_cy so we don't restamp a cell already drawn — it
 * carries across all the body segments, not just this one. */
static void draw_body_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr, int cols, int rows,
                            int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx; *prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* Draw one glyph at a pixel position, skipping it if it's off-screen. */
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

/* ── §5i render — draw the whole spider, back to front ── */

/* Draw both bones of every leg. */
static void draw_legs(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, sp->hip[i],  sp->knee[i],     3, A_BOLD, cols, rows);
        draw_leg_line(w, sp->knee[i], sp->foot_pos[i], 3, A_BOLD, cols, rows);
    }
}

/* Mark each knee with 'o', and each foot with '*' if planted or '.' if in
 * the air — so you can read at a glance which feet are stepping. */
static void draw_leg_joints(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        mark_cell(w, sp->knee[i], (chtype)(unsigned char)'o',
                  3, A_BOLD, cols, rows);

        if (sp->stepping[i])
            mark_cell(w, sp->foot_pos[i], (chtype)(unsigned char)'.',
                      7, A_DIM, cols, rows);
        else
            mark_cell(w, sp->foot_pos[i], (chtype)(unsigned char)'*',
                      6, A_BOLD, cols, rows);
    }
}

/* Fill the body with beads, drawing tail-first so that where the body
 * crosses itself in a tight turn, the head end stays on top. */
static void draw_body_lines(const Spider *sp, WINDOW *w, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = N_BODY_SEGS - 1; i >= 0; i--) {
        draw_body_beads(w, sp->body_joint[i + 1], sp->body_joint[i],
                        3, A_BOLD, cols, rows, &prev_cx, &prev_cy);
    }
}

/* Mark the body joints: a big 'O' for the abdomen at the tail, smaller 'o'
 * for the joints in between. */
static void draw_body_nodes(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = N_BODY_SEGS; i >= 1; i--) {
        chtype glyph = (i == N_BODY_SEGS)
                     ? (chtype)(unsigned char)'O'   /* abdomen tip */
                     : (chtype)(unsigned char)'o';  /* mid-body    */
        mark_cell(w, sp->body_joint[i], glyph, 3, A_BOLD, cols, rows);
    }
}

/* Draw the head as ":" eyes flanking a direction arrow. The eyes sit off to
 * either side of the arrow, so whichever way the spider faces, they always
 * straddle it (above/below when going sideways, left/right when going up). */
static void draw_head(const Spider *sp, WINDOW *w, int cols, int rows)
{
    Vec2  head   = sp->body_joint[0];
    float perp_x = -sinf(sp->heading) * EYE_OFFSET_PX;
    float perp_y =  cosf(sp->heading) * EYE_OFFSET_PX;

    Vec2 eye_l = { head.x - perp_x, head.y - perp_y };
    Vec2 eye_r = { head.x + perp_x, head.y + perp_y };

    mark_cell(w, eye_l, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, eye_r, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);

    /* Arrow drawn last so it sits boldly between the eyes. */
    mark_cell(w, head, head_glyph(sp->heading), 3, A_BOLD, cols, rows);
}

/* Draw the spider in back-to-front order so the joint markers land on top of
 * the line glyphs: leg lines, then leg joints, then body fill, body joints,
 * and finally the head. */
static void render_spider(const Spider *sp, WINDOW *w, int cols, int rows)
{
    draw_legs       (sp, w, cols, rows);
    draw_leg_joints (sp, w, cols, rows);
    draw_body_lines (sp, w, cols, rows);
    draw_body_nodes (sp, w, cols, rows);
    draw_head       (sp, w, cols, rows);
}

/* ── §6 scene — the world (just one spider) ── */

/*
 * The whole simulated world. Here that's a single spider, but it's wrapped in
 * a Scene so the main loop talks to scene_init / scene_tick / scene_draw the
 * same way every demo in this project does. Anything new (prey, a web) would
 * be added here next to the spider.
 */
typedef struct {
    Spider spider;             /* the only thing in the world for now */
} Scene;

/*
 * Set up the spider at the centre of the screen. The trail is pre-filled with
 * a straight line stretching backward, so the body shows up fully extended on
 * the very first frame instead of growing out from a single point. All feet
 * start planted at their resting spots.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Spider *sp = &sc->spider;

    sp->move_speed     = BODY_SPEED;
    sp->heading        = 0.0f;
    sp->target_heading = 0.0f;
    sp->hip_dist       = (float)(rows * CELL_H) * HIP_DIST_FACTOR;
    sp->paused         = false;
    sp->theme_idx      = 0;

    sp->body_joint[0].x = (float)(cols * CELL_W) * 0.50f;
    sp->body_joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-fill trail one pixel at a time straight backward so the body
     * appears fully extended on the very first rendered frame. */
    float bx = cosf(sp->heading + (float)M_PI);
    float by = sinf(sp->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        sp->trail[k].x = sp->body_joint[0].x + (float)k * bx;
        sp->trail[k].y = sp->body_joint[0].y + (float)k * by;
    }
    sp->trail_head  = 0;
    sp->trail_count = TRAIL_CAP;

    compute_body_joints(sp);
    compute_hips(sp);

    /* Plant all feet at their ideal positions and solve IK. */
    for (int i = 0; i < N_LEGS; i++) {
        sp->foot_pos[i]    = compute_ideal_foot(sp, i);
        sp->foot_old[i]    = sp->foot_pos[i];
        sp->step_target[i] = sp->foot_pos[i];
        sp->stepping[i]    = false;
        sp->step_t[i]      = 0.0f;
        solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
    }
}

/*
 * Advance the world by dt seconds. The order matters: turn, then walk the
 * head, then re-lay the body behind it, then the hips on the body, then the
 * feet — each step builds on the one before it.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Spider *sp = &sc->spider;
    if (sp->paused) return;

    steer_heading      (sp, dt);
    translate_body     (sp, dt, cols, rows);
    compute_body_joints(sp);
    compute_hips       (sp);
    update_steps       (sp, dt);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_spider(&sc->spider, w, cols, rows);
}

/* ── §7 screen — ncurses setup and drawing ── */

/* The terminal's current size, in character cells. Cached here so the rest
 * of the code reads cols/rows without asking ncurses every frame; refreshed
 * on a resize. */
typedef struct {
    int cols;   /* width  in character cells */
    int rows;   /* height in character cells */
} Screen;

/* Start ncurses in raw, no-echo, hidden-cursor mode. typeahead(-1) is the
 * one non-obvious call: without it ncurses peeks at stdin mid-write, which
 * can tear a frame. */
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

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* The direction arrow as a string, for the HUD text. */
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

/* Draw one full frame: clear, draw the spider, then the status line at the
 * top-right and the key hints along the bottom. */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Spider *sp = &sc->spider;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-SPIDER  dir:%s  spd:%.0f  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(sp->heading), sp->move_speed,
             THEMES[sp->theme_idx].name,
             time_scale, fps,
             sp->paused ? "PAUSED" : "crawling");

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

/* ── §8 app — signals, resize, the main loop ── */

/*
 * Everything the program owns outside the world itself: the scene, the
 * terminal, and the few flags the loop watches. It lives at file scope as
 * g_app so the signal handlers (which can't be passed an argument) can set
 * `running` and `need_resize`.
 *
 * The two flags are volatile sig_atomic_t because a signal can change them
 * at any moment: volatile forces a fresh read each loop, and sig_atomic_t
 * guarantees the read/write can't be seen half-done.
 */
typedef struct {
    Scene  scene;              /* the world (§6)            */
    Screen screen;             /* the terminal it draws to  */

    float                 time_scale;   /* slow-mo / fast-forward factor   */
    volatile sig_atomic_t running;      /* loop keeps going while non-zero */
    volatile sig_atomic_t need_resize;  /* set by a resize signal          */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* Registered with atexit so the terminal is restored on any exit path. */
static void cleanup(void) { endwin(); }

/*
 * Handle a terminal resize: re-read the size, pull the spider back inside the
 * new bounds, and rescale the hip spread. The theme is re-applied because
 * some terminals reset their colours on resize.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Spider *sp  = &app->scene.spider;
    float   wpx = (float)(app->screen.cols * CELL_W);
    float   hpx = (float)(app->screen.rows * CELL_H);
    if (sp->body_joint[0].x >= wpx) sp->body_joint[0].x = wpx - 1.0f;
    if (sp->body_joint[0].y >= hpx) sp->body_joint[0].y = hpx - 1.0f;
    sp->hip_dist = hpx * HIP_DIST_FACTOR;
    theme_apply(sp->theme_idx);
    app->need_resize = 0;
}

/*
 * Act on one keypress; return false only for quit. Arrow keys set the
 * direction to steer toward (the turn itself happens gradually in
 * steer_heading), and the other keys adjust speed, theme, or time.
 */
static bool app_handle_key(App *app, int ch)
{
    Spider *sp = &app->scene.spider;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sp->paused = !sp->paused; break;

    case KEY_RIGHT: sp->target_heading =  0.0f;                break;
    case KEY_DOWN:  sp->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  sp->target_heading =  (float)M_PI;         break;
    case KEY_UP:    sp->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        sp->move_speed *= 1.25f;
        if (sp->move_speed > BODY_SPEED_MAX) sp->move_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        sp->move_speed /= 1.25f;
        if (sp->move_speed < BODY_SPEED_MIN) sp->move_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        sp->theme_idx = (sp->theme_idx + 1) % N_THEMES;
        theme_apply(sp->theme_idx);
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
 * main — variable-timestep render loop. Per-frame phases:
 *   ① INPUT      drain getch() — a press takes effect on this frame
 *   ② RESIZE     handle pending SIGWINCH before touching ncurses
 *   ③ MEASURE dt wall-clock ns since last frame; capped at 100 ms
 *   ④ TICK       one simulation step at exactly dt · time_scale
 *   ⑤ FPS        smoothed over a 500 ms window
 *   ⑥ RENDER     erase → draw → wnoutrefresh → doupdate
 *   ⑦ FRAME CAP  sleep so total frame ≈ 1/TARGET_FPS
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

        /* ⑦ frame cap — sleep off whatever time is left in the frame */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
