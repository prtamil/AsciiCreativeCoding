/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_scorpin.c — a scorpion crawls across the terminal: trail-following
 *   body, six IK legs that plant and swing as it walks, and a seven-segment
 *   tail that arches up over the back with a slow side-to-side sway.
 *
 * Inverse kinematics (the legs): you say where a foot should land and the
 * code works out the joint angles to reach it. Sister demos: ik_spider.c
 * (same body + legs, no tail), hexpod_tripod.c (rigid chassis contrast).
 *
 * Keys: q/ESC quit · space pause · arrows steer · w/s speed · t theme
 *       · [ / ] slower/faster time
 *
 * Build (needs -lm for the trig/sqrt):
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_scorpin.c \
 *       -o ik_scorpin -lncurses -lm
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

/* ── §1 config — all tunable numbers in one place ── */

enum {
    /* Frame-rate cap. The sim uses a variable timestep, so this only
     * sets how long we sleep at the end of each frame. */
    TARGET_FPS    = 60,

    /* HUD layout. */
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,

    /* ncurses pair IDs.
     *   1..N_PAIRS  body + legs + tail gradient (themed)
     *   8 PAIR_HUD   bright yellow status bar (theme-independent)
     *   9 PAIR_HINT  bright cyan key hint    (theme-independent) */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,
    PAIR_HINT     = 9,

    N_THEMES      = 10,    /* cycled with `t` */

    /* Anatomy.
     *   N_LEGS       = 6 (3 per side, 3 angular pairs at 60° spacing)
     *   N_BODY_SEGS  = 4 → 5 joints, 80 px end-to-end
     *   N_TAIL_SEGS  = 7 → 8 tail joints, ~77 px curving up over body */
    N_LEGS        = 6,
    N_BODY_SEGS   = 4,
    N_TAIL_SEGS   = 7,
    TRAIL_CAP     = 1024,
};

/* Body geometry (px). */
#define BODY_SEG_LEN      20.0f
#define BODY_SPEED        45.0f
#define BODY_SPEED_MIN    10.0f
#define BODY_SPEED_MAX   200.0f
#define TURN_RATE          2.5f

/* Leg geometry (px). */
#define UPPER_LEN         56.0f
#define LOWER_LEN         50.0f
#define HIP_DIST_FACTOR   0.04f

/* Step gait parameters. */
#define STEP_REACH_FACTOR 0.68f
#define STEP_TRIGGER_DIST 28.0f
#define MAX_STRETCH       65.0f
#define STEP_DURATION     0.22f

/*
 * Tail shape and sway. The tail is a chain of segments; each one bends a
 * little further than the last, so the chain arcs up and over the back.
 *
 *   TAIL_SEG_LEN       length of one tail segment (px).
 *   TAIL_BASE_CURL     fixed bend each segment adds. 7 segments x this
 *                      curls ~136deg total, sweeping the stinger over the
 *                      body — the classic scorpion shape.
 *   TAIL_SWAY_AMP      how far the bend wobbles on top of the base curl.
 *                      Kept well below TAIL_BASE_CURL (0.06 vs 0.34) so the
 *                      chain never bends backward and folds on itself.
 *   TAIL_FREQ          sway speed (rad/s); ~6 s per full sway cycle.
 *   TAIL_PHASE_PER_SEG phase shift per segment — makes the wobble travel
 *                      from the base out to the stinger instead of moving
 *                      as one rigid piece.
 */
/* FIXME: N_TAIL_SEGS_F is defined but never used (dead). */
#define N_TAIL_SEGS_F       7.0f
#define TAIL_SEG_LEN       11.0f
#define TAIL_BASE_CURL      0.34f
#define TAIL_SWAY_AMP       0.06f
#define TAIL_FREQ           1.0f
#define TAIL_PHASE_PER_SEG  0.5f

/* Direction-glyph step sizes. DRAW_STEP_PX < CELL_W (8) so the dense
 * stamping never skips a column. */
#define DRAW_STEP_PX      5.0f
#define DRAW_LEG_STEP_PX  8.0f

/* Eye cluster — perpendicular distance from head joint, in pixel space. */
#define EYE_OFFSET_PX    10.0f

/* Time scale — user-controlled simulation speed multiplier on `[/]`. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Nanosecond conversion factors for the clock. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions (aspect-ratio bridge). */
#define CELL_W   8
#define CELL_H  16

/*
 * LEG_ANGLE[i] — which way leg i reaches, as an angle (rad) measured from
 * the body's forward direction. Left legs reach left (positive), right legs
 * mirror them (negative). This sets where each foot wants to plant.
 */
static const float LEG_ANGLE[N_LEGS] = {
     0.6f,    /* 0 front-LEFT   ~ 34° forward + outward */
    -0.6f,    /* 1 front-RIGHT  */
     1.57f,   /* 2 mid-LEFT     90° straight out        */
    -1.57f,   /* 3 mid-RIGHT    */
     2.5f,    /* 4 rear-LEFT    ~143° rear + outward    */
    -2.5f,    /* 5 rear-RIGHT   */
};

/*
 * HIP_BODY_T[i] — where along the body each leg's hip attaches, as a
 * fraction from head (0.0) to abdomen (1.0). Legs come in left/right pairs,
 * so each value appears twice.
 */
static const float HIP_BODY_T[N_LEGS] = {
    0.20f, 0.20f,   /* front pair — near the head */
    0.50f, 0.50f,   /* mid pair   — body centre   */
    0.80f, 0.80f,   /* rear pair  — near the tail */
};

/* ── §2 clock — monotonic clock + sleep ── */

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

/* ── §3 color — 10 scorpion palettes + fixed HUD/hint pairs ── */

/*
 * Theme — one colour palette for the scorpion. Each of the 7 colours
 * (xterm-256 indices) paints a different part. theme_apply() loads a
 * palette into ncurses colour pairs 1..7; the HUD/hint pairs are kept
 * separate so the status bar stays readable over any theme.
 *
 * The 7 colours, in order (col[0]..col[6]):
 *   [0] body, tail-end (darkest)   [3] upper leg
 *   [1] body, middle               [4] lower leg
 *   [2] body, head-end (main hue)  [5] bright: tail + planted foot + stinger
 *                                  [6] dim: swinging foot (cue it's in flight)
 *
 * The body shades 0->1->2 from tail to head so the eye can trace it.
 * Planted feet ('*', bright) vs swinging feet ('.', dim) shows which feet
 * are on the ground. Every colour is kept in the bright half of the
 * palette so nothing vanishes under A_DIM.
 */
typedef struct {
    const char *name;            /* shown in the HUD            */
    int         col[N_PAIRS];    /* the 7 colours; col[p-1] -> pair p */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        body gradient   legs      foot   ghost */
    {"Desert",  {130, 136, 142,  58,  64,  46, 240}},
    {"Black",   { 52,  88, 124,  52,  88, 196, 240}},
    {"Toxic",   { 24,  28,  34,  28,  34,  82, 240}},
    {"Ocean",   { 24,  25,  27,  33,  39,  51, 240}},
    {"Nova",    { 54,  93, 129,  93, 129, 165, 240}},
    {"Ember",   { 52,  94, 130, 130, 130, 208, 240}},
    {"Aurora",  { 24,  29,  35,  35,  71, 221, 240}},
    {"Ghost",   {240, 244, 248, 244, 248, 254, 246}},
    {"Fire",    { 52,  88, 196,  88, 124, 226, 240}},
    {"Neon",    { 57,  93, 201,  93, 129, 201, 240}},
};

/* theme_apply — re-bind body/leg/tail pairs to chosen theme. HUD/HINT
 * pairs are NEVER touched — they're theme-independent. */
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
        /* 8-color fallback */
        init_pair(1, COLOR_RED,    -1);
        init_pair(2, COLOR_RED,    -1);
        init_pair(3, COLOR_RED,    -1);
        init_pair(4, COLOR_GREEN,  -1);
        init_pair(5, COLOR_GREEN,  -1);
        init_pair(6, COLOR_GREEN,  -1);
        init_pair(7, COLOR_WHITE,  -1);
    }

    /* HUD pairs are theme-independent — bright yellow status, bright
     * cyan hint, both on default bg so they overlay any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — pixel-to-cell rounding (terminal aspect ratio) ── */

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — Scorpion: body FK + IK legs + curving tail ── */

/*
 * Vec2 — a 2-D point in pixel space. Everything geometric (body, legs,
 * tail) works in pixels, which are finer than character cells (a cell is
 * 8x16 pixels), so motion looks smooth instead of jumping cell to cell.
 * The rendering helpers convert pixels to cells at the last moment.
 *
 * x grows rightward, y grows downward (the usual screen layout). Angles
 * are measured the math way (counter-clockwise from +x), but since y
 * points down, that shows on screen as clockwise — watch for this in the
 * left/right sign flips in solve_ik and compute_hips.
 */
typedef struct {
    float x;   /* pixels, positive = right */
    float y;   /* pixels, positive = down  */
} Vec2;

/* ── §5a  vec2 + small helpers ──────────────────────────────────────── */

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

static inline Vec2 vec2_norm(Vec2 v)
{
    float len = vec2_len(v);
    if (len < 1e-6f) return (Vec2){ 1.0f, 0.0f };
    return (Vec2){ v.x / len, v.y / len };
}

/* smoothstep — cubic ease-in/ease-out on [0, 1]. Used for foot swing. */
static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* rotate2d — rotate v by `angle` rad. */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ── Scorpion state ──────────────────────────────────────────────────── */

/*
 * Scorpion — all of the creature's state in one record. Four parts share
 * it, and each frame they run in order with no feedback between them:
 *
 *   BODY     A rolling log of where the head has been; the body joints are
 *            placed back along that trail, so the body follows the exact
 *            path the head walked (no per-joint angles needed).
 *   LEGS     Each leg is a 2-bone limb. Given a foot target, IK works out
 *            the knee. A simple gait decides when a foot has drifted too
 *            far and swings it to a fresh spot, keeping at least half the
 *            feet planted at all times.
 *   TAIL     A chain hanging off the abdomen that arches over the back and
 *            sways. Computed fresh each frame from the body pose + a clock.
 *   STEERING Heading turns toward target_heading, capped at TURN_RATE so
 *            turns are smooth rather than instant.
 *
 * A few layout choices worth knowing:
 *   - Leg data is kept as parallel arrays (one array per attribute) because
 *     most loops walk one attribute across all legs at once.
 *   - trail is a circular buffer: the head writes to it and the body reads
 *     from it, with no shifting of elements per frame.
 *   - knee[] is recomputed and stored each tick so the renderer can read it
 *     several times without re-running the IK.
 *   - hip_dist is computed from terminal height (not a fixed constant) so
 *     the legs spread proportionally; it is recomputed on resize.
 *   - wave_time is the only state the tail keeps. Same wave_time gives the
 *     same tail pose, so pausing it freezes the tail and resumes in phase.
 */
typedef struct {
    /* ── Body: trail buffer + the joints sampled from it ── */
    Vec2 trail[TRAIL_CAP];            /* circular log of past head positions  */
    int  trail_head;                  /* index of newest entry, mod TRAIL_CAP */
    int  trail_count;                 /* valid entries, caps at TRAIL_CAP     */
    Vec2 body_joint[N_BODY_SEGS + 1]; /* [0]=head, [N]=abdomen; rebuilt/tick  */

    /* ── Body heading and speed ── */
    float heading;                    /* current facing (rad), 0 = right      */
    float target_heading;             /* facing we're turning toward          */
    float move_speed;                 /* forward speed (px/s)                 */

    /* ── Per-leg state (one entry per leg) ── */
    Vec2  hip        [N_LEGS];        /* hip position, recomputed each tick   */
    Vec2  knee       [N_LEGS];        /* knee from IK, recomputed each tick   */
    Vec2  foot_pos   [N_LEGS];        /* where the foot is (planted or moving)*/
    Vec2  foot_old   [N_LEGS];        /* foot position when this step began   */
    Vec2  step_target[N_LEGS];        /* where the moving foot is heading     */
    bool  stepping   [N_LEGS];        /* true while this foot is in the air   */
    float step_t     [N_LEGS];        /* swing progress, 0..1                 */

    /* ── Tail ── */
    Vec2  tail[N_TAIL_SEGS + 1];      /* [0]=base at abdomen, [N]=stinger     */
    float wave_time;                  /* clock (s) driving the tail's sway    */

    /* ── Sized to the terminal ── */
    float hip_dist;                   /* how far hips sit out from the spine (px) */

    /* ── Controls ── */
    bool  paused;                     /* true = sim frozen (still draws)      */
    int   theme_idx;                  /* which §3 palette is active           */
} Scorpion;

/* ── §5b  trail (push / at / sample) ────────────────────────────────── */

static void trail_push(Scorpion *sc, Vec2 pos)
{
    sc->trail_head = (sc->trail_head + 1) % TRAIL_CAP;
    sc->trail[sc->trail_head] = pos;
    if (sc->trail_count < TRAIL_CAP) sc->trail_count++;
}

static inline Vec2 trail_at(const Scorpion *sc, int k)
{
    return sc->trail[(sc->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/* trail_sample — walk back along the trail until `dist` pixels of path
 * have been covered, and return the point there. This is how a body joint
 * finds its spot a fixed distance behind the head. */
static Vec2 trail_sample(const Scorpion *sc, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(sc, 0);

    for (int k = 1; k < sc->trail_count; k++) {
        Vec2  b   = trail_at(sc, k);
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
    return trail_at(sc, sc->trail_count - 1);
}

/* ── §5c  body motion ──────────────────────────────────────────────── */

/* shortest_signed_angle — fold an angle difference into [-pi, pi] so the
 * scorpion always turns the short way around the circle (e.g. turn -60
 * degrees instead of +300). */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/* clamp_to_max_turn_per_dt — limit how much the heading can change this
 * frame to TURN_RATE * dt, no matter how far the target is. Keeps turns
 * smooth instead of snapping instantly. */
static float clamp_to_max_turn_per_dt(float diff, float dt)
{
    float max_step = TURN_RATE * dt;
    return clampf(diff, -max_step, max_step);
}

/* steer_heading — nudge heading toward target_heading: pick the short way,
 * cap the step, apply it. */
static void steer_heading(Scorpion *sc, float dt)
{
    float diff    = shortest_signed_angle(sc->target_heading - sc->heading);
    float clamped = clamp_to_max_turn_per_dt(diff, dt);
    sc->heading  += clamped;
}

/* integrate_position_along_heading — move the head forward by speed * dt
 * in the heading direction. (cos, sin) of the heading is the unit forward
 * vector. */
static void integrate_position_along_heading(Scorpion *sc, float dt)
{
    sc->body_joint[0].x += sc->move_speed * cosf(sc->heading) * dt;
    sc->body_joint[0].y += sc->move_speed * sinf(sc->heading) * dt;
}

/* wrap_position_to_toroidal_world — wrap the head around screen edges:
 * off the right comes back on the left, off the bottom back at the top.
 * A single add/subtract works because the head never moves more than one
 * screen-width in a frame. */
static void wrap_position_to_toroidal_world(Scorpion *sc, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (sc->body_joint[0].x <  0.0f) sc->body_joint[0].x += wpx;
    if (sc->body_joint[0].x >= wpx)  sc->body_joint[0].x -= wpx;
    if (sc->body_joint[0].y <  0.0f) sc->body_joint[0].y += hpx;
    if (sc->body_joint[0].y >= hpx)  sc->body_joint[0].y -= hpx;
}

/* translate_body — one tick of the head: move forward, wrap at edges, then
 * record the new position in the trail (which the body joints follow). */
static void translate_body(Scorpion *sc, float dt, int cols, int rows)
{
    integrate_position_along_heading  (sc, dt);
    wrap_position_to_toroidal_world   (sc, cols, rows);
    trail_push                        (sc, sc->body_joint[0]);
}

/* ── §5d  body joints — place each joint a fixed distance back on the trail ── */

static void compute_body_joints(Scorpion *sc)
{
    for (int i = 1; i <= N_BODY_SEGS; i++)
        sc->body_joint[i] = trail_sample(sc, (float)i * BODY_SEG_LEN);
}

/* ── §5e  hip placement — find where each leg attaches to the body ── */

/* body_local_forward — unit vector pointing forward along the body at the
 * given segment (the last segment just uses the heading). */
static Vec2 body_local_forward(const Scorpion *sc, int seg_idx)
{
    if (seg_idx + 1 > N_BODY_SEGS)
        return (Vec2){ cosf(sc->heading), sinf(sc->heading) };
    return vec2_norm(vec2_sub(sc->body_joint[seg_idx],
                              sc->body_joint[seg_idx + 1]));
}

/*
 * attachment_point_along_spine — given a fraction t_norm from head (0) to
 * abdomen (1), find the point that far along the body and which segment it
 * falls in. Scale t_norm up to a segment index, take the whole part to pick
 * the segment and the fractional part to interpolate between its two joints.
 * The segment index is passed back so the caller can reuse it.
 */
static Vec2 attachment_point_along_spine(const Scorpion *sc, float t_norm,
                                         int *seg_idx_out)
{
    float t_body  = t_norm * (float)N_BODY_SEGS;
    int   seg_idx = (int)t_body;
    if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
    float frac    = t_body - (float)seg_idx;

    *seg_idx_out = seg_idx;
    return vec2_lerp(sc->body_joint[seg_idx],
                     sc->body_joint[seg_idx + 1], frac);
}

/* lateral_normal_at_spine — direction straight out the side of the body.
 * Rotating the forward vector 90 degrees gives the left side; `side` is
 * +1 for left legs, -1 for right. */
static Vec2 lateral_normal_at_spine(const Scorpion *sc, int seg_idx, float side)
{
    Vec2 fwd       = body_local_forward(sc, seg_idx);
    Vec2 left_norm = (Vec2){ -fwd.y, fwd.x };           /* forward rotated 90 deg */
    return vec2_scale(left_norm, side);
}

/*
 * compute_hips — set each leg's hip position. For every leg: find its
 * attach point on the body, step hip_dist pixels straight out the side
 * (left for even-numbered legs, right for odd), and that's the hip.
 */
static void compute_hips(Scorpion *sc)
{
    for (int i = 0; i < N_LEGS; i++) {
        int  seg_idx;
        Vec2 attach = attachment_point_along_spine(sc, HIP_BODY_T[i], &seg_idx);

        float side  = (i % 2 == 0) ? 1.0f : -1.0f;
        Vec2  out   = lateral_normal_at_spine(sc, seg_idx, side);

        sc->hip[i]  = vec2_add(attach, vec2_scale(out, sc->hip_dist));
    }
}

/* ── §5f  2-joint analytical IK ──────────────────────────────────── */

/*
 * clamp_to_reachable_annulus — pull the target distance into the range a
 * 2-bone leg can actually reach: no closer than the bones folded together
 * (|U-L|), no farther than fully extended (U+L). The +-1 px margin keeps
 * us just inside, so rounding can't push acos below in solve_ik out of
 * range.
 */
static float clamp_to_reachable_annulus(float dist)
{
    return clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN        - 1.0f);
}

/*
 * law_of_cosines_apex_angle — the bend angle at the hip. Given the three
 * side lengths of the (hip, knee, foot) triangle — upper bone U, lower
 * bone L, and hip-to-foot distance d — the law of cosines gives the angle
 * between the upper bone and the straight line to the foot:
 *
 *       cos(a) = (d*d + U*U - L*L) / (2*d*U)
 *
 * Always positive; the caller adds or subtracts it to pick which way the
 * knee bends.
 */
static float law_of_cosines_apex_angle(float dist)
{
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN
                                - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    return acosf(clampf(cos_h, -1.0f, 1.0f));
}

/* place_knee_at_angle — step one upper-bone length out from the hip in the
 * given direction; that point is the knee. The lower bone is then just the
 * straight line from knee to foot. */
static Vec2 place_knee_at_angle(Vec2 hip, float angle)
{
    return (Vec2){ hip.x + UPPER_LEN * cosf(angle),
                   hip.y + UPPER_LEN * sinf(angle) };
}

/*
 * solve_ik — the inverse-kinematics solve for one leg: given the hip and a
 * foot target, find the knee that reaches it. This is the heart of "you say
 * where the foot goes, the code works out the joints."
 *
 *       hip *---------- U ----------* knee
 *            \                      /
 *              d                  L
 *                \              /
 *                  *----------* foot (= target)
 *
 *   1. d    = distance hip to foot (clamped to what the leg can reach).
 *   2. base = direction from hip to foot.
 *   3. a    = bend angle at the hip (law of cosines).
 *   4. knee angle = base +- a; left legs bend one way, right legs mirror.
 *   5. step out one upper bone along that angle to get the knee.
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = clamp_to_reachable_annulus(sqrtf(dx * dx + dy * dy));

    float base        = atan2f(dy, dx);
    float alpha       = law_of_cosines_apex_angle(dist);
    float knee_angle  = is_left ? (base + alpha) : (base - alpha);

    *knee_out = place_knee_at_angle(hip, knee_angle);
}

/* ── §5g  step gait — decide when feet lift and where they land ── */

/* compute_ideal_foot — the resting spot leg i "wants" its foot: out from
 * the hip in the leg's reach direction, at a comfortable distance. */
static Vec2 compute_ideal_foot(const Scorpion *sc, int i)
{
    Vec2  fwd   = (Vec2){ cosf(sc->heading), sinf(sc->heading) };
    Vec2  dir   = rotate2d(fwd, LEG_ANGLE[i]);
    float reach = (UPPER_LEN + LOWER_LEN) * STEP_REACH_FACTOR;
    return vec2_add(sc->hip[i], vec2_scale(dir, reach));
}

static int count_airborne(const Scorpion *sc)
{
    int n = 0;
    for (int i = 0; i < N_LEGS; i++)
        if (sc->stepping[i]) n++;
    return n;
}

/* snap_overstretched_foot — safety net: if a foot ends up beyond what the
 * leg can reach (e.g. after a screen wrap), teleport it to the ideal spot
 * so the IK never gets an impossible target. Returns true if it was mid-air
 * so the caller can fix the airborne count. */
static bool snap_overstretched_foot(Scorpion *sc, int i)
{
    if (vec2_dist(sc->foot_pos[i], sc->hip[i]) <= UPPER_LEN + LOWER_LEN - 2.0f)
        return false;

    bool was_airborne   = sc->stepping[i];
    sc->foot_pos[i]     = compute_ideal_foot(sc, i);
    sc->foot_old[i]     = sc->foot_pos[i];
    sc->step_target[i]  = sc->foot_pos[i];
    sc->stepping[i]     = false;
    sc->step_t[i]       = 0.0f;
    return was_airborne;
}

/* maybe_trigger_step — start a new step if this foot has drifted too far
 * from its ideal spot (or stretched too far), but only while fewer than
 * half the legs are already in the air. Returns true if a step started. */
static bool maybe_trigger_step(Scorpion *sc, int i, int n_air)
{
    if (sc->stepping[i] || n_air >= N_LEGS / 2) return false;

    Vec2  ideal   = compute_ideal_foot(sc, i);
    float drift   = vec2_dist(sc->foot_pos[i], ideal);
    float stretch = vec2_dist(sc->foot_pos[i], sc->hip[i]);
    if (drift <= STEP_TRIGGER_DIST && stretch <= MAX_STRETCH) return false;

    sc->stepping[i]    = true;
    sc->step_t[i]      = 0.0f;
    sc->foot_old[i]    = sc->foot_pos[i];
    sc->step_target[i] = ideal;
    return true;
}

/* advance_swing — move a foot that's mid-step along its arc toward the
 * target, eased so it slows in and out. Returns true on the frame it
 * lands. */
static bool advance_swing(Scorpion *sc, int i, float dt)
{
    sc->step_t[i] += dt / STEP_DURATION;
    if (sc->step_t[i] >= 1.0f) {
        sc->step_t[i]   = 1.0f;
        sc->foot_pos[i] = sc->step_target[i];
        sc->stepping[i] = false;
        return true;
    }
    float ease     = smoothstep(sc->step_t[i]);
    sc->foot_pos[i] = vec2_lerp(sc->foot_old[i], sc->step_target[i], ease);
    return false;
}

/*
 * update_steps — run the gait for one frame. Each leg does exactly one of:
 * snap back if overstretched, keep swinging if mid-step, or start a new
 * step if it has drifted. n_air tracks how many feet are airborne so the
 * "no more than half off the ground at once" rule holds — that's what keeps
 * a walking insect from toppling. Then re-solve the IK for every leg so the
 * knees match the new foot positions.
 */
static void update_steps(Scorpion *sc, float dt)
{
    int n_air = count_airborne(sc);

    /* PHASE 1 — update each foot */
    for (int i = 0; i < N_LEGS; i++) {
        if      (snap_overstretched_foot(sc, i))    n_air--;
        else if (sc->stepping[i]) {
            if (advance_swing(sc, i, dt))           n_air--;
        }
        else if (maybe_trigger_step(sc, i, n_air))  n_air++;
    }

    /* PHASE 2 — recompute every knee from its (possibly moved) foot */
    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sc->hip[i], sc->foot_pos[i], (i % 2 == 0), &sc->knee[i]);
}

/* ── §5h  tail FK — build the arching, swaying tail each frame ── */

/* tail_bend_at_segment — how much tail segment i bends: a fixed curl (the
 * arch) plus a small sine wobble. The "i * phase" term offsets each
 * segment so the wobble looks like a wave rolling out toward the stinger
 * rather than the whole tail twitching at once. */
static float tail_bend_at_segment(const Scorpion *sc, int i)
{
    float wave = TAIL_SWAY_AMP * sinf(sc->wave_time * TAIL_FREQ
                                      + (float)i * TAIL_PHASE_PER_SEG);
    return TAIL_BASE_CURL + wave;
}

/*
 * compute_tail — lay out the tail joints for this frame. Start at the
 * abdomen pointing backward, then for each segment turn by its bend angle
 * and step one segment-length forward. The bends accumulate, so the chain
 * curves more and more — about 136 degrees total — arcing the tail up and
 * over the back. Rebuilt from scratch every frame; the only thing that
 * carries over is wave_time, so pausing freezes the tail mid-sway.
 */
static void compute_tail(Scorpion *sc)
{
    sc->tail[0] = sc->body_joint[N_BODY_SEGS];           /* base at abdomen */

    float cumulative_angle = sc->heading + (float)M_PI;  /* start pointing back */

    for (int i = 0; i < N_TAIL_SEGS; i++) {
        cumulative_angle += tail_bend_at_segment(sc, i);

        sc->tail[i + 1].x = sc->tail[i].x + TAIL_SEG_LEN * cosf(cumulative_angle);
        sc->tail[i + 1].y = sc->tail[i].y + TAIL_SEG_LEN * sinf(cumulative_angle);
    }
}

/* ── §5i  rendering helpers — turn pixel geometry into characters ── */

/* head_glyph — pick an arrow (> v < ^) for the way the head faces. */
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

/* seg_glyph — pick the ASCII line character (- \ | /) that best matches the
 * slope of a segment running (dx, dy). Note -dy: screen y grows downward, so
 * we flip it to think in normal upward angles. */
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

/* draw_chain_line — draw a line from a to b as alternating slope-glyph and
 * '.' characters, so it reads as a jointed limb. Used for both legs and the
 * tail. Skips cells already stamped and clips anything off-screen. */
static void draw_chain_line(WINDOW *w, Vec2 a, Vec2 b,
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

/* draw_body_beads — draw the body as a row of 'o' beads from a to b. The
 * caller threads prev_cx/prev_cy across segments so beads aren't doubled up
 * where two body segments meet. */
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

/* mark_cell — stamp a single glyph at pixel point p, ignoring it if it
 * falls off-screen. */
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

/* ── §5j  render_scorpion — draw the whole creature, back to front ── */

static void draw_legs(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_chain_line(w, sc->hip[i],  sc->knee[i],     3, A_BOLD, cols, rows);
        draw_chain_line(w, sc->knee[i], sc->foot_pos[i], 3, A_BOLD, cols, rows);
    }
}

static void draw_leg_joints(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        mark_cell(w, sc->knee[i], (chtype)(unsigned char)'o',
                  3, A_BOLD, cols, rows);

        /* '.' dim for a foot in the air, '*' bright for one on the ground */
        if (sc->stepping[i])
            mark_cell(w, sc->foot_pos[i], (chtype)(unsigned char)'.',
                      7, A_DIM, cols, rows);
        else
            mark_cell(w, sc->foot_pos[i], (chtype)(unsigned char)'*',
                      6, A_BOLD, cols, rows);
    }
}

static void draw_body_lines(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = N_BODY_SEGS - 1; i >= 0; i--) {
        draw_body_beads(w, sc->body_joint[i + 1], sc->body_joint[i],
                        3, A_BOLD, cols, rows, &prev_cx, &prev_cy);
    }
}

static void draw_body_nodes(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = N_BODY_SEGS; i >= 1; i--) {
        chtype glyph = (i == N_BODY_SEGS)
                     ? (chtype)(unsigned char)'O'   /* abdomen tip */
                     : (chtype)(unsigned char)'o';
        mark_cell(w, sc->body_joint[i], glyph, 3, A_BOLD, cols, rows);
    }
}

/* draw_tail — draw the tail as a chain (same style as the legs) in the
 * bright accent colour so it stands out where it crosses the body, with a
 * '#' stinger at the tip. */
static void draw_tail(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_TAIL_SEGS; i++) {
        draw_chain_line(w, sc->tail[i], sc->tail[i + 1],
                        6, A_BOLD, cols, rows);
    }
    /* Stinger — distinct glyph at the tail tip. */
    mark_cell(w, sc->tail[N_TAIL_SEGS], (chtype)(unsigned char)'#',
              6, A_BOLD, cols, rows);
}

/* draw_head — draw a ':' eye on each side of the head plus a facing arrow,
 * so it's obvious which way the scorpion is pointed. The eyes sit
 * perpendicular to the heading. */
static void draw_head(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    Vec2  head   = sc->body_joint[0];
    float perp_x = -sinf(sc->heading) * EYE_OFFSET_PX;
    float perp_y =  cosf(sc->heading) * EYE_OFFSET_PX;

    Vec2 eye_l = { head.x - perp_x, head.y - perp_y };
    Vec2 eye_r = { head.x + perp_x, head.y + perp_y };

    mark_cell(w, eye_l, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, eye_r, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, head, head_glyph(sc->heading), 3, A_BOLD, cols, rows);
}

/* render_scorpion — draw everything in order so the right parts end up on
 * top: legs, then body, then the tail (which arcs over the body), then the
 * head last so it's never hidden. */
static void render_scorpion(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    draw_legs       (sc, w, cols, rows);
    draw_leg_joints (sc, w, cols, rows);
    draw_body_lines (sc, w, cols, rows);
    draw_body_nodes (sc, w, cols, rows);
    draw_tail       (sc, w, cols, rows);
    draw_head       (sc, w, cols, rows);
}

/* ── §6 scene — thin wrapper holding the one scorpion ── */

/*
 * Scene — the simulated world. Here it's just one scorpion, but it's kept
 * as a wrapper so the main loop (scene_init / scene_tick / scene_draw) looks
 * the same as in every other demo. Prey, obstacles, or more scorpions would
 * be added as extra fields here.
 */
typedef struct {
    Scorpion scorpion;         /* body + 6 legs + tail */
} Scene;

/* scene_init — set up the scorpion at the centre of the screen. The trail
 * is pre-filled going straight backward so the body is already full-length
 * on the first frame instead of growing out from a point, and every foot
 * starts planted at its rest spot. */
static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    Scorpion *sc = &s->scorpion;

    sc->move_speed     = BODY_SPEED;
    sc->heading        = 0.0f;
    sc->target_heading = 0.0f;
    sc->hip_dist       = (float)(rows * CELL_H) * HIP_DIST_FACTOR;
    sc->wave_time      = 0.0f;
    sc->paused         = false;
    sc->theme_idx      = 0;

    sc->body_joint[0].x = (float)(cols * CELL_W) * 0.50f;
    sc->body_joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-fill trail backward so the body is fully extended on frame 1. */
    float bx = cosf(sc->heading + (float)M_PI);
    float by = sinf(sc->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        sc->trail[k].x = sc->body_joint[0].x + (float)k * bx;
        sc->trail[k].y = sc->body_joint[0].y + (float)k * by;
    }
    sc->trail_head  = 0;
    sc->trail_count = TRAIL_CAP;

    compute_body_joints(sc);
    compute_hips(sc);
    compute_tail(sc);

    for (int i = 0; i < N_LEGS; i++) {
        sc->foot_pos[i]    = compute_ideal_foot(sc, i);
        sc->foot_old[i]    = sc->foot_pos[i];
        sc->step_target[i] = sc->foot_pos[i];
        sc->stepping[i]    = false;
        sc->step_t[i]      = 0.0f;
        solve_ik(sc->hip[i], sc->foot_pos[i], (i % 2 == 0), &sc->knee[i]);
    }
}

/* scene_tick — advance the world by dt seconds (already scaled by the user's
 * time multiplier). Runs the subsystems in dependency order; the tail is last
 * so it reads the abdomen position this frame's body update just produced. */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    Scorpion *sc = &s->scorpion;
    if (sc->paused) return;

    sc->wave_time += dt;

    steer_heading      (sc, dt);
    translate_body     (sc, dt, cols, rows);
    compute_body_joints(sc);
    compute_hips       (sc);
    update_steps       (sc, dt);
    compute_tail       (sc);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    render_scorpion(&s->scorpion, w, cols, rows);
}

/* ── §7 screen — ncurses setup, HUD, double-buffered present ── */

/*
 * Screen — the current terminal size, in character cells. Cached here so
 * the rest of the code reads plain ints instead of asking ncurses each
 * frame; it's refreshed only on a resize. (Cells, not pixels — pixels stay
 * inside §5 and get converted at draw time.)
 */
typedef struct {
    int cols;   /* terminal width  in cells */
    int rows;   /* terminal height in cells */
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();                  /* don't echo typed keys */
    cbreak();                  /* deliver keys immediately, no line buffering */
    curs_set(0);               /* hide the cursor */
    nodelay(stdscr, TRUE);     /* getch() returns ERR instead of blocking */
    keypad(stdscr, TRUE);      /* report arrow keys as KEY_* codes */
    typeahead(-1);             /* don't let pending input interrupt our drawing */
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* screen_resize — re-read the terminal size after the window changed.
 * The endwin/refresh pair makes ncurses notice the new dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* heading_arrow — like head_glyph but returns a string for the HUD text.
 * (NOTE: duplicates head_glyph's direction buckets.) */
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

/* screen_draw — clear, draw the scorpion, then overlay the HUD (top-right
 * status) and the key-hint line (bottom). */
static void screen_draw(Screen *s, const Scene *sn,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sn, stdscr, s->cols, s->rows);

    const Scorpion *sc = &sn->scorpion;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-SCORPION  dir:%s  spd:%.0f  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(sc->heading), sc->move_speed,
             THEMES[sc->theme_idx].name,
             time_scale, fps,
             sc->paused ? "PAUSED" : "crawling");

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

/* ── §8 app — signals, resize, input, main loop ── */

/*
 * App — everything the program owns: the world, the terminal, and the
 * flags that control the main loop. It's a single file-scope global (g_app)
 * because signal handlers can't take an argument, so they need to reach the
 * flags directly.
 *
 * running and need_resize are volatile sig_atomic_t: volatile so the loop
 * re-reads them every iteration (a handler may have changed them), and
 * sig_atomic_t so a signal can never catch them half-written.
 */
typedef struct {
    Scene  scene;              /* the world (§6)         */
    Screen screen;             /* terminal size (§7)     */

    float                 time_scale;   /* dt multiplier; 1.0 = realtime    */
    volatile sig_atomic_t running;      /* loop keeps going while non-zero  */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, cleared on resize */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

static void cleanup(void) { endwin(); }

/* app_do_resize — handle a terminal resize: pick up the new size, pull the
 * scorpion back inside the new bounds, rescale how far the legs spread, and
 * reload the theme (colour pairs are lost on the ncurses restart). */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scorpion *sc = &app->scene.scorpion;
    float wpx = (float)(app->screen.cols * CELL_W);
    float hpx = (float)(app->screen.rows * CELL_H);
    if (sc->body_joint[0].x >= wpx) sc->body_joint[0].x = wpx - 1.0f;
    if (sc->body_joint[0].y >= hpx) sc->body_joint[0].y = hpx - 1.0f;
    sc->hip_dist = hpx * HIP_DIST_FACTOR;
    theme_apply(sc->theme_idx);
    app->need_resize = 0;
}

/* app_handle_key — apply one keypress. Returns false only for quit keys,
 * which tells the loop to stop. */
static bool app_handle_key(App *app, int ch)
{
    Scorpion *sc = &app->scene.scorpion;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sc->paused = !sc->paused; break;

    case KEY_RIGHT: sc->target_heading =  0.0f;                break;
    case KEY_DOWN:  sc->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  sc->target_heading =  (float)M_PI;         break;
    case KEY_UP:    sc->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        sc->move_speed *= 1.25f;
        if (sc->move_speed > BODY_SPEED_MAX) sc->move_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        sc->move_speed /= 1.25f;
        if (sc->move_speed < BODY_SPEED_MIN) sc->move_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
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

        /* ③ measure dt — time since last frame, capped at 100 ms so a long
         *    stall (e.g. laptop sleep) doesn't make the scorpion teleport */
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

        /* ⑦ frame cap */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
