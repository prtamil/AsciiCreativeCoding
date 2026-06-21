/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_tentacle_seek.c — a 16-segment tentacle that reaches for a moving dot.
 *
 * Inverse kinematics: you say where the tentacle TIP should reach, and the
 * solver works out all the segment angles to get there. The dot traces a
 * Lissajous figure; each joint has a max-bend limit so the chain curls
 * instead of kinking. Solver is FABRIK (Aristidou & Lasenby, 2011).
 *
 * Sister files: ik_arm_reach.c (plain FABRIK, no bend limits),
 *               ik_spider.c    (closed-form 2-joint IK for contrast).
 *
 * Keys:  q / ESC quit   space pause   w/s speed   t/T theme   [ / ] time scale
 *
 * Build (needs -lm for sin/cos/atan2):
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_tentacle_seek.c \
 *       -o ik_tentacle_seek -lncurses -lm
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
    TARGET_FPS    = 60,    /* frames per second the loop aims for */

    /* HUD layout. */
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,

    /* ncurses pair IDs.
     *   1..N_PAIRS  body gradient (root dark → tip bright), themed
     *   8 PAIR_HUD   bright yellow status (theme-independent)
     *   9 PAIR_HINT  bright cyan hint     (theme-independent) */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,
    PAIR_HINT     = 9,

    N_THEMES      = 10,    /* cycled with `t`/`T` */

    /* Joint count: 17 joints = 16 links. Tip = pos[16]; root = pos[0].
     * Longer chain than the canonical 12-link FABRIK demo so the joint
     * angle constraint visibly fires across more bends — easier to see
     * the curl-without-kink behaviour in action. */
    N_JOINTS      = 17,
    N_LINKS       = 16,

    /* MAX_ITER: FABRIK convergence cap. Typical 16-link chains converge
     * in 4–10 iterations; 20 is a safety ceiling. Cost: 20 · 17 · 2 = 680
     * vector ops per tick — negligible. */
    MAX_ITER      = 20,

    /* Trail buffer capacity. At ~60 fps this is ~2 s of target history;
     * every 3rd point rendered → 40 visible dots. */
    TRAIL_POINTS  = 120,
};

/* CONV_TOL — FABRIK convergence threshold (pixel space). 2.0 px is
 * sub-cell on both axes (CELL_W=8, CELL_H=16) — further iterations
 * produce no visible improvement. */
#define CONV_TOL        2.0f

/* MAX_JOINT_BEND — per-joint bend angle limit (radians). 1.1 rad ≈ 63°.
 * Below 45° the chain is too stiff to reach close targets. Above 90°
 * visible kinking appears at fast direction reversals. 1.1 is the
 * biological sweet spot. */
#define MAX_JOINT_BEND  1.1f

/*
 * Link geometry — tapered chain.
 *   link_len[i] = max(BASE_LINK_LEN − i · TAPER, 4 px)
 *   i=0  (root):  22.0 px,  i=8 (mid): 16.4 px,  i=15 (tip): 11.5 px
 *   Σ = 16 · 22 − 0.7 · 120 = 268 px total reach
 * Tapering is biomechanical: root link long (lever arm), tip short (nimble).
 * The 0.7 taper (gentler than the typical 0.8) keeps even tip links
 * substantial so the tip is articulated, not vestigial.
 */
#define BASE_LINK_LEN   22.0f
#define TAPER            0.7f

/* DRAW_STEP_PX — bead fill stride. < CELL_W (8) so dense stamping never
 * skips a column. */
#define DRAW_STEP_PX    5.0f

/*
 * Lissajous target parameters.
 *
 *   x(t) = anchor.x + LIS_AX · cos(LIS_OMEGA_X · scene_time)
 *   y(t) = anchor.y + LIS_AY · sin(LIS_OMEGA_Y · scene_time + LIS_PHASE_Y)
 *
 * LIS_OMEGA_X : LIS_OMEGA_Y = 1 : 1.7 = 10 : 17 — large LCM gives a
 * 62.8 s quasi-period with internal crossings that exercise the IK
 * solver with varied directional demands.
 *
 * LIS_AX/AY scaled to 55%/38% of screen — fits within tentacle reach.
 * LIS_PHASE_Y = π/3 avoids a tangent cusp at the figure's start.
 */
#define LIS_AX_FACTOR   0.55f
#define LIS_AY_FACTOR   0.38f
#define LIS_OMEGA_X     1.0f
#define LIS_OMEGA_Y     1.7f
#define LIS_PHASE_Y     ((float)M_PI / 3.0f)

/*
 * TARGET_SMOOTH — first-order low-pass rate (1/s). At 60 fps,
 * dt · TARGET_SMOOTH ≈ 0.133 → time constant τ = 1/8 s ≈ 125 ms.
 * Absorbs sudden target velocity reversals at Lissajous crossings so
 * the joint clamp never fires aggressively.
 */
#define TARGET_SMOOTH   8.0f

/* Speed scale — user-controlled Lissajous time multiplier on +/-/w/s. */
#define SPEED_DEFAULT   1.0f
#define SPEED_MIN       0.05f
#define SPEED_MAX       8.0f

/* Time scale — user-controlled simulation time multiplier on `[/]`. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions (aspect-ratio bridge). */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic time and sleep (shared framework code) ── */

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

/* ── §3 color — 10 body palettes plus fixed HUD/hint colors ── */

/*
 * Theme — one colour palette for the tentacle body.
 *
 * The body uses 7 colour pairs that run root (dim) → tip (bright), so
 * the brightening gradient lets your eye trace the chain and spot the
 * tip that is doing the reaching. Switching theme rebinds all 7 pairs
 * at once (theme_apply); the HUD/hint colours are left alone so the
 * status bar stays readable over any theme.
 *
 * All colour indices stay in the bright half of the xterm-256 space
 * (cube ≥ 24, grayscale ≥ 240); darker indices vanish under A_DIM. The
 * theme's character comes from the gradient, not from absolute darkness.
 */
typedef struct {
    const char *name;            /* shown in the HUD                       */
    int         body[N_PAIRS];   /* 7 xterm-256 fg colours, root → tip;    *
                                  * body[p-1] becomes colour pair p        */
} Theme;

static const Theme THEMES[N_THEMES] = {
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Matrix", { 24,  28,  34,  40,  46,  82, 118}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Ocean",  { 24,  25,  27,  33,  39,  45,  51}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Toxic",  { 24,  58,  64,  70,  76,  82, 118}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {240, 244, 246, 248, 250, 252, 255}},
    {"Aurora", { 24,  28,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/* theme_apply — re-bind body pairs to the chosen theme. HUD/HINT
 * pairs are NEVER touched here. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_PAIRS; p++)
        init_pair(p + 1, t->body[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color fallback — coarse purple→blue→cyan→white gradient. */
        init_pair(1, COLOR_MAGENTA, -1);
        init_pair(2, COLOR_MAGENTA, -1);
        init_pair(3, COLOR_BLUE,    -1);
        init_pair(4, COLOR_BLUE,    -1);
        init_pair(5, COLOR_CYAN,    -1);
        init_pair(6, COLOR_CYAN,    -1);
        init_pair(7, COLOR_WHITE,   -1);
    }

    /* HUD pairs are theme-independent — bright yellow status, bright
     * cyan hint, both on default bg so they overlay any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — turn smooth pixel positions into terminal cells ── */

/*
 * All entity positions live in square pixel space (1 unit = 1 px).
 * Only at draw time do these helpers convert to cell coordinates,
 * undoing the 8:16 cell aspect ratio.
 *   cell = floor(px / CELL_DIM + 0.5)    — nearest-integer rounding
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the Tentacle: solver, target, trail, drawing ── */

/*
 * Vec2 — a 2-D point in pixel space (finer than one cell).
 *
 * All the maths — the solver, the bend limits, the moving target —
 * runs in pixels, where each cell is 8 x 16 sub-pixels. That extra
 * resolution is why the tentacle glides instead of snapping cell to
 * cell. Positions become cells only at draw time (px_to_cell_x/y).
 *
 * Small value type, passed by copy: the solver does this arithmetic
 * thousands of times a frame and -O2 inlines it all, no allocation.
 */
typedef struct {
    float x;   /* pixels right of the left edge */
    float y;   /* pixels down from the top      */
} Vec2;

/* ── §5a  vec2 helpers ──────────────────────────────────────────────── */

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

static inline float vec2_len (Vec2 a)
{ return sqrtf(a.x * a.x + a.y * a.y); }

static inline float vec2_dist(Vec2 a, Vec2 b)
{ return vec2_len(vec2_sub(b, a)); }

/* vec2_norm — unit vector. Zero-length input returns (1, 0); the
 * specific direction is arbitrary but lets callers avoid NaN. */
static inline Vec2 vec2_norm(Vec2 a)
{
    float len = vec2_len(a);
    if (len < 1e-6f) return (Vec2){ 1.0f, 0.0f };
    return (Vec2){ a.x / len, a.y / len };
}

/* dot, cross — for unit vectors, cos(θ) and sin(θ). Together with
 * atan2 they give the signed angle in [−π, π], which is what the
 * joint-angle clamp needs (acos would lose the sign). */
static inline float vec2_dot  (Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
static inline float vec2_cross(Vec2 a, Vec2 b) { return a.x*b.y - a.y*b.x; }

/* vec2_rotate — rotate v by `angle` rad. Used by the joint clamp to
 * rotate the outgoing link direction by the correction. */
static inline Vec2 vec2_rotate(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ── Tentacle state ─────────────────────────────────────────────────── */

/*
 * Tentacle — everything the demo needs to track in one record.
 *
 * It holds three things that feed into one another each frame:
 *   1. the chain itself — joint positions and fixed segment lengths;
 *   2. the moving target — a Lissajous dot circling the anchor;
 *   3. a smoothed copy of that target, which is what the chain chases.
 *
 * The flow runs one way, no feedback: clock advances → Lissajous dot
 * moves → smoothing softens it → solver bends the chain to reach it →
 * renderer draws it. scene_tick walks that order.
 *
 * pos[] holds N_LINKS+1 joints (one extra for the root end). Get that
 * +1 wrong and the chain silently loses a segment.
 */
typedef struct {
    /* ── the chain; the solver rewrites pos[] from scratch each tick ── */
    Vec2  pos     [N_JOINTS];  /* joint positions; [0]=root, [N-1]=tip   */
    float link_len[N_LINKS];   /* segment lengths, set once, never change */

    /* Root anchor — the pinned base. Stored (not recomputed) so a
     * resize just reassigns it; lissajous_at re-centres around it. */
    Vec2  anchor;              /* tentacle root, pixels                  */

    /* The moving goal. scene_time drives the Lissajous dot; speed_scale
     * lets the user dilate that time. actual_target is the smoothed dot
     * the solver actually reaches for (see update_target). */
    Vec2  actual_target;       /* smoothed target the chain chases       */
    float scene_time;          /* drives the Lissajous curve, seconds    */
    float speed_scale;         /* user multiplier on how fast it traces  */

    /* Recent target positions, drawn as a dotted ghost trail. Ring
     * buffer: trail_write is the next slot (wraps mod TRAIL_POINTS),
     * trail_fill counts valid entries and stops growing once full so
     * the renderer never reads an uninitialised slot. */
    Vec2  trail_pts[TRAIL_POINTS];
    int   trail_write;         /* next slot to overwrite                 */
    int   trail_fill;          /* valid entries, caps at TRAIL_POINTS    */

    /* Per-tick results and user toggles. */
    int   last_iter;           /* solver loops the last solve took (HUD) */
    bool  at_limit;            /* true when target is beyond total reach */
    bool  paused;              /* freezes the clock, so the chain freezes */
    int   theme_idx;           /* which THEMES[] palette is active       */
} Tentacle;

/* ── §5b joint angle constraint — stop the chain from kinking ── */

/*
 * signed_angle_between_unit_vectors — angle to turn `a` onto `b`, in
 * radians, in [-π, π], positive counter-clockwise.
 *
 *   sin = a × b   (cross),  cos = a · b   (dot),  angle = atan2(sin, cos)
 *
 * We use atan2 rather than acos because we need the SIGN — which way the
 * joint bends — to know which way to rotate it back. acos only gives the
 * size of the angle, never its direction.
 */
static float signed_angle_between_unit_vectors(Vec2 a, Vec2 b)
{
    float sin_theta = vec2_cross(a, b);
    float cos_theta = vec2_dot  (a, b);
    return atan2f(sin_theta, cos_theta);
}

/*
 * place_child_along_direction — put the next joint exactly one segment
 * length away from joint i, in the given direction:
 *
 *   pos[i+1] = pos[i] + link_len[i] · direction
 *
 * Split out so the bend clamp can reuse it after it has rotated the
 * direction.
 */
static void place_child_along_direction(Tentacle *t, int i, Vec2 direction)
{
    t->pos[i + 1] = vec2_add(t->pos[i],
                             vec2_scale(direction, t->link_len[i]));
}

/*
 * apply_joint_constraint — don't let joint i bend more than
 * ±MAX_JOINT_BEND. This is what keeps the chain curling like a
 * tentacle instead of folding into a sharp hairpin.
 *
 * Look at the angle between the link arriving at the joint and the link
 * leaving it. If it is too sharp, rotate the outgoing link back to the
 * limit and move the next joint to match. Endpoints are skipped: the
 * root has no incoming link, the tip has no outgoing one.
 *
 * Joint limits like this are the Aristidou/Chrysanthou/Lasenby (2016)
 * extension to plain FABRIK.
 */
static void apply_joint_constraint(Tentacle *t, int i)
{
    if (i < 1 || i >= N_JOINTS - 1) return;

    Vec2 dir_in  = vec2_norm(vec2_sub(t->pos[i],     t->pos[i - 1]));
    Vec2 dir_out = vec2_norm(vec2_sub(t->pos[i + 1], t->pos[i]));

    float angle = signed_angle_between_unit_vectors(dir_in, dir_out);
    if (fabsf(angle) <= MAX_JOINT_BEND) return;

    float clamped_angle = clampf(angle, -MAX_JOINT_BEND, MAX_JOINT_BEND);
    float correction    = clamped_angle - angle;
    Vec2  new_dir       = vec2_rotate(dir_out, correction);

    place_child_along_direction(t, i, new_dir);
}

/* ── §5c FABRIK solver — work out the joint positions ── */

/* total_link_length — sum of all link lengths. Constant after init. */
static float total_link_length(const Tentacle *t)
{
    float total = 0.0f;
    for (int i = 0; i < N_LINKS; i++) total += t->link_len[i];
    return total;
}

/*
 * snap_to_link_length — the one move FABRIK is built from. Keep the
 * direction from `base` to `endpoint`, but pull `endpoint` in or out to
 * sit exactly `len` away. After this the segment is the right length
 * again. Both passes of the solver are just this move, applied joint by
 * joint, from one end or the other.
 */
static Vec2 snap_to_link_length(Vec2 base, Vec2 endpoint, float len)
{
    Vec2 dir = vec2_norm(vec2_sub(endpoint, base));
    return vec2_add(base, vec2_scale(dir, len));
}

/*
 * stretch_collinear — what to do when the target is too far away. If the
 * target is farther than the whole chain can reach, there is no way to
 * touch it, so just lay every segment out in a straight line aimed at it.
 * The tip ends up as close as the chain allows, pointing the right way.
 */
static void stretch_collinear(Tentacle *t, Vec2 anchor, Vec2 target)
{
    Vec2  dir   = vec2_norm(vec2_sub(target, anchor));
    float accum = 0.0f;
    t->pos[0]   = anchor;
    for (int i = 0; i < N_LINKS; i++) {
        accum += t->link_len[i];
        t->pos[i + 1] = vec2_add(anchor, vec2_scale(dir, accum));
    }
}

/*
 * fabrik_backward_pass — the "reach" half. Pin the tip onto the target,
 * then walk back toward the root, sliding each joint so its segment is
 * the right length again and clamping any joint that bends too far. When
 * done the tip is on target and every segment is correct, but the root
 * has drifted off its anchor — the forward pass fixes that next.
 */
static void fabrik_backward_pass(Tentacle *t, Vec2 target)
{
    t->pos[N_JOINTS - 1] = target;
    for (int i = N_JOINTS - 2; i >= 0; i--) {
        t->pos[i] = snap_to_link_length(t->pos[i + 1], t->pos[i],
                                        t->link_len[i]);
        if (i > 0) apply_joint_constraint(t, i);
    }
}

/*
 * fabrik_forward_pass — the "anchor" half. Pin the root back onto its
 * anchor, then walk out to the tip, sliding each joint to restore its
 * segment length. Now the root is anchored and the segments are correct,
 * but the tip has slipped off the target — by less than last time. Each
 * round of the two passes shrinks that gap, so a few rounds settle it.
 *
 * No bend clamp here: the backward pass already kept the bends in range,
 * and this pass only stretches along existing directions.
 */
static void fabrik_forward_pass(Tentacle *t, Vec2 anchor)
{
    t->pos[0] = anchor;
    for (int i = 0; i < N_JOINTS - 1; i++) {
        t->pos[i + 1] = snap_to_link_length(t->pos[i], t->pos[i + 1],
                                            t->link_len[i]);
    }
}

/*
 * tip_to_target_error — how far the tip still is from the target. This
 * shrinks every round, so once it drops below CONV_TOL we stop early.
 */
static float tip_to_target_error(const Tentacle *t, Vec2 target)
{
    return vec2_dist(t->pos[N_JOINTS - 1], target);
}

/*
 * fabrik_solve — the whole solve for one frame. First check reach: if
 * the target is too far, fall back to stretch_collinear and stop. Other-
 * wise alternate the reach pass and the anchor pass until the tip is
 * close enough or we hit the loop cap. A handful of rounds is normal.
 *
 * Returns how many rounds it used (the HUD shows it) and sets at_limit.
 */
static int fabrik_solve(Tentacle *t, Vec2 target, Vec2 anchor)
{
    /* PHASE 1 — reachability gate */
    float total = total_link_length(t);
    t->at_limit = (vec2_dist(anchor, target) >= total);
    if (t->at_limit) {
        stretch_collinear(t, anchor, target);
        return 1;
    }

    /* PHASE 2 — iterate until convergence */
    int iter = 0;
    for (iter = 0; iter < MAX_ITER; iter++) {
        fabrik_backward_pass(t, target);
        fabrik_forward_pass (t, anchor);
        if (tip_to_target_error(t, target) < CONV_TOL) {
            iter++;
            break;
        }
    }
    return iter;
}

/* ── §5d the moving target — Lissajous dot, smoothed ── */

/* trail_push — add one point to the ring buffer, dropping the oldest
 * once it is full. */
static void trail_push(Tentacle *t, Vec2 pos)
{
    t->trail_pts[t->trail_write] = pos;
    t->trail_write = (t->trail_write + 1) % TRAIL_POINTS;
    if (t->trail_fill < TRAIL_POINTS) t->trail_fill++;
}

/*
 * lissajous_at — where the target dot is at time `t`. A Lissajous figure
 * is just one sine across and another sine down; with the two frequencies
 * not a simple ratio (1 : 1.7 here) the path loops without ever quite
 * repeating, so it keeps crossing itself and giving the chain new shapes.
 */
static Vec2 lissajous_at(Vec2 anchor, int cols, int rows, float t)
{
    float ax = (float)(cols * CELL_W) * LIS_AX_FACTOR;
    float ay = (float)(rows * CELL_H) * LIS_AY_FACTOR;
    return (Vec2){
        anchor.x + ax * cosf(LIS_OMEGA_X * t),
        anchor.y + ay * sinf(LIS_OMEGA_Y * t + LIS_PHASE_Y),
    };
}

/*
 * advance_phase_clock — push the Lissajous clock forward. speed_scale
 * lets the user trace the same figure faster or slower without changing
 * its shape.
 */
static void advance_phase_clock(Tentacle *t, float dt)
{
    t->scene_time += dt * t->speed_scale;
}

/*
 * first_order_low_pass — ease one value toward another instead of
 * jumping. Each call moves `current` a fraction `rate` of the way to
 * `raw`: rate 0 means don't move, rate 1 means snap all the way. We use
 * it on the target so the chain follows a softened path and the bend
 * clamp doesn't fire on every sharp turn. rate is clamped to [0,1] so a
 * long frame can't overshoot.
 */
static Vec2 first_order_low_pass(Vec2 current, Vec2 raw, float rate)
{
    rate = clampf(rate, 0.0f, 1.0f);
    return vec2_lerp(current, raw, rate);
}

/*
 * update_target — move the goal forward one frame: tick the clock, read
 * the raw Lissajous dot, ease the smoothed target toward it, and log it
 * for the trail. The chain chases the smoothed target, and that smoothed
 * path is what the trail shows.
 */
static void update_target(Tentacle *t, float dt, int cols, int rows)
{
    advance_phase_clock(t, dt);

    Vec2  raw  = lissajous_at(t->anchor, cols, rows, t->scene_time);
    float rate = dt * TARGET_SMOOTH;
    t->actual_target = first_order_low_pass(t->actual_target, raw, rate);

    trail_push(t, t->actual_target);
}

/* ── §5e trail ring buffer — read back past target points ── */

/* trail_get — the trail point k steps before the newest. The
 * +TRAIL_POINTS before % keeps the index positive (C's % can go
 * negative). */
static inline Vec2 trail_get(const Tentacle *t, int k)
{
    int idx = (t->trail_write - 1 - k + TRAIL_POINTS) % TRAIL_POINTS;
    return t->trail_pts[idx];
}

/* ── §5f drawing helpers — colour, glyph, and stamping one cell ── */

/* joint_pair — colour pair for joint i, running root (pair 1, dim) to
 * tip (pair N_PAIRS, bright). */
static int joint_pair(int i)
{
    int p = 1 + (i * (N_PAIRS - 1)) / (N_JOINTS - 1);
    if (p < 1)       p = 1;
    if (p > N_PAIRS) p = N_PAIRS;
    return p;
}

/* joint_attr — depth-cued attribute for joint/link i.
 *   root third : A_DIM    (heavy, anchored)
 *   middle     : A_NORMAL
 *   tip third  : A_BOLD   (glowing, active)         */
static attr_t joint_attr(int i)
{
    if (i <= 2)              return A_DIM;
    if (i >= N_JOINTS - 4)   return A_BOLD;
    return A_NORMAL;
}

/* joint_marker — bead marker character by joint index.
 *   '0' root third, 'o' middle, '.' tip third — taper reinforces
 *   the visual hierarchy from anchor to seeking tip. */
static chtype joint_marker(int i)
{
    if (i <= (N_JOINTS - 1) / 3)        return (chtype)(unsigned char)'0';
    if (i >= (N_JOINTS - 1) * 2 / 3)    return (chtype)(unsigned char)'.';
    return                              (chtype)(unsigned char)'o';
}

/*
 * draw_link_beads — stamp 'o' every few pixels along the segment a→b so
 * the link reads as a solid string. prev_cx/cy skip a cell we just drew,
 * so overlapping steps don't restamp the same spot.
 */
static void draw_link_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr, int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* mark_cell — stamp one glyph at a pixel position with bounds check.
 * Pure helper used for trail dots, joint markers, target, anchor. */
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

/* ── §5g render_tentacle — draw the whole thing, back to front ── */

/* draw_trail — the dotted ghost path the target has traced. Every 3rd
 * stored point so it reads as dashes, in the bright tip colour. */
static void draw_trail(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int k = 0; k < t->trail_fill; k += 3) {
        mark_cell(w, trail_get(t, k), (chtype)(unsigned char)'.',
                  7, A_NORMAL, cols, rows);
    }
}

/* draw_link_fill — the string of beads along every segment, in the
 * root→tip gradient. Drawn root first so the brighter tip wins where the
 * chain folds over itself. */
static void draw_link_fill(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LINKS; i++) {
        draw_link_beads(w, t->pos[i], t->pos[i + 1],
                        joint_pair(i), joint_attr(i), cols, rows);
    }
}

/* draw_link_nodes — a marker at each joint, stamped over the fill; the
 * glyph (0/o/.) shows how far along the chain the joint sits. */
static void draw_link_nodes(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_JOINTS; i++) {
        mark_cell(w, t->pos[i], joint_marker(i),
                  joint_pair(i), joint_attr(i), cols, rows);
    }
}

/* draw_target_marker — the goal dot: '*' when the chain reached it,
 * '#' (different colour) when it is out of reach. */
static void draw_target_marker(const Tentacle *t, WINDOW *w,
                               int cols, int rows)
{
    if (t->at_limit) {
        mark_cell(w, t->actual_target, (chtype)(unsigned char)'#',
                  2, A_BOLD, cols, rows);
    } else {
        mark_cell(w, t->actual_target, (chtype)(unsigned char)'*',
                  6, A_BOLD, cols, rows);
    }
}

/* draw_anchor_marker — a '0' at the pinned base, drawn last so nothing
 * paints over it. */
static void draw_anchor_marker(const Tentacle *t, WINDOW *w,
                               int cols, int rows)
{
    mark_cell(w, t->anchor, (chtype)(unsigned char)'0',
              1, A_BOLD, cols, rows);
}

/*
 * render_tentacle — draw everything in back-to-front order so each layer
 * covers the one before: trail, then the links, the joints, the target,
 * and finally the anchor on top.
 */
static void render_tentacle(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    draw_trail        (t, w, cols, rows);
    draw_link_fill    (t, w, cols, rows);
    draw_link_nodes   (t, w, cols, rows);
    draw_target_marker(t, w, cols, rows);
    draw_anchor_marker(t, w, cols, rows);
}

/* ── §6 scene — thin wrapper holding the one Tentacle ── */

/*
 * Scene — the whole simulated world. Here that is just one tentacle,
 * but the wrapper keeps the main loop (scene_init / scene_tick /
 * scene_draw) reading the same as every other demo in the repo, and
 * gives a second tentacle or some obstacles an obvious place to go.
 */
typedef struct {
    Tentacle tentacle;         /* chain + target + trail, all in one */
} Scene;

/*
 * scene_init — place tentacle anchored at screen centre with all joints
 * laid out in a horizontal line. Pre-seed the trail with the initial
 * actual_target so the first frame already has a visible trail. Run
 * one FABRIK solve before the first render so HUD values are valid.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Tentacle *t = &sc->tentacle;

    t->anchor.x    = (float)(cols * CELL_W) * 0.50f;
    t->anchor.y    = (float)(rows * CELL_H) * 0.50f;
    t->speed_scale = SPEED_DEFAULT;
    t->scene_time  = 0.0f;
    t->paused      = false;
    t->theme_idx   = 0;

    /* Tapered link lengths, root longest. */
    for (int i = 0; i < N_LINKS; i++) {
        t->link_len[i] = BASE_LINK_LEN - (float)i * TAPER;
        if (t->link_len[i] < 4.0f) t->link_len[i] = 4.0f;
    }

    /* Initial pose: straight line right from anchor. */
    float x = t->anchor.x;
    t->pos[0] = t->anchor;
    for (int i = 0; i < N_LINKS; i++) {
        x += t->link_len[i];
        t->pos[i + 1] = (Vec2){ x, t->anchor.y };
    }

    /* Initial target = Lissajous at scene_time = 0 (max +x displacement). */
    t->actual_target = lissajous_at(t->anchor, cols, rows, 0.0f);

    /* Pre-seed the trail buffer so the first frame is fully populated. */
    for (int k = 0; k < TRAIL_POINTS; k++) t->trail_pts[k] = t->actual_target;
    t->trail_fill  = TRAIL_POINTS;
    t->trail_write = 0;

    /* One solve so HUD shows valid iteration count and at_limit flag. */
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);
}

/*
 * scene_tick — one variable-timestep simulation step. dt is wall-clock
 * delta scaled by the caller's time_scale.
 *
 * Order: target update first (so FABRIK sees the freshly smoothed
 * target), then FABRIK solve.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Tentacle *t = &sc->tentacle;
    if (t->paused) return;

    update_target(t, dt, cols, rows);
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_tentacle(&sc->tentacle, w, cols, rows);
}

/* ── §7 screen — ncurses setup, frame compose, HUD ── */

/*
 * Screen — the terminal size in character cells, cached so the rest of
 * the code reads plain ints instead of asking ncurses every frame. It is
 * refreshed only on a resize (SIGWINCH). Cells, not pixels: ncurses works
 * in cells, and the pixel maths stays inside §5.
 */
typedef struct {
    int cols;   /* terminal width  in cells */
    int rows;   /* terminal height in cells */
} Screen;

/* The non-obvious call here is typeahead(-1): without it, ncurses peeks
 * at stdin during output writes, which can tear frames mid-update. */
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

/*
 * screen_draw — compose one full frame:
 *   erase → tentacle → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any theme.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Tentacle *t = &sc->tentacle;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  N=%d  iter:%2d  %s  spd:%.2fx  theme:%s  %.2ftime  %.1ffps  %s ",
             N_LINKS, t->last_iter,
             t->at_limit ? "at-limit " : "converged",
             t->speed_scale,
             THEMES[t->theme_idx].name,
             time_scale, fps,
             t->paused ? "PAUSED" : "seeking");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:speed  t/T:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, keyboard, main loop ── */

/*
 * App — everything outside the simulated world: the world (Scene), the
 * terminal (Screen), and the few flags the loop checks each frame. It
 * lives at file scope (g_app) because signal handlers can't take an
 * argument, so they need a fixed place to set `running` and `need_resize`.
 *
 * Those two flags are volatile sig_atomic_t because a signal can change
 * them between any two instructions: volatile forces the loop to re-read
 * them from memory, and sig_atomic_t guarantees the read/write is not
 * torn in half.
 */
typedef struct {
    Scene  scene;              /* the simulated world (§6)             */
    Screen screen;             /* the terminal it draws to (§7)        */

    float                 time_scale;   /* speeds up / slows down time   */
    volatile sig_atomic_t running;      /* loop runs while non-zero      */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, handled next */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH. Re-centres the anchor to
 * the new screen dimensions; the chain follows on the next solve.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Tentacle *t = &app->scene.tentacle;
    t->anchor.x = (float)(app->screen.cols * CELL_W) * 0.50f;
    t->anchor.y = (float)(app->screen.rows * CELL_H) * 0.50f;
    t->pos[0]   = t->anchor;
    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Letter aliases (w/s for +/-) for ergonomics.
 */
static bool app_handle_key(App *app, int ch)
{
    Tentacle *t = &app->scene.tentacle;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': t->paused = !t->paused; break;

    case 'w': case '=': case '+':
        t->speed_scale *= 1.25f;
        if (t->speed_scale > SPEED_MAX) t->speed_scale = SPEED_MAX;
        break;
    case 's': case '-':
        t->speed_scale /= 1.25f;
        if (t->speed_scale < SPEED_MIN) t->speed_scale = SPEED_MIN;
        break;

    case 't':
        t->theme_idx = (t->theme_idx + 1) % N_THEMES;
        theme_apply(t->theme_idx);
        break;
    case 'T':
        t->theme_idx = (t->theme_idx - 1 + N_THEMES) % N_THEMES;
        theme_apply(t->theme_idx);
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

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
