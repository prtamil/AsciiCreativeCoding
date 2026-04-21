/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_spider.c — 6-Legged Spider with Procedural IK Leg Placement
 *
 * DEMO: A 6-legged spider crawls autonomously across the terminal.
 *       The body follows a sinusoidal swimming path (trail-buffer FK,
 *       identical to snake_forward_kinematics.c).  Each of the 6 legs
 *       uses 2-joint analytical inverse kinematics (law of cosines) to
 *       reach a computed step target.  Legs step in an alternating tripod
 *       gait when they become too stretched from their ideal position.
 *
 * STUDY ALONGSIDE: snake_forward_kinematics.c (trail-buffer FK body)
 *                  framework.c (canonical loop / timing template)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — arachnid palette (dark red + olive gradient)
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Spider: body FK + IK legs + step gait logic
 *       §5a  vec2 helpers
 *       §5b  trail helpers (push / at / sample — same as snake FK)
 *       §5c  body motion (heading integration + toroidal wrap)
 *       §5d  body joints (trail-buffer FK)
 *       §5e  hip placement (attach legs to body)
 *       §5f  2-joint analytical IK (law of cosines)
 *       §5g  step logic (trigger, alternating tripod gait, smoothstep)
 *       §5h  rendering (legs, feet, body, head)
 *   §6  scene         — scene_init / scene_tick / scene_draw
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, fixed-step main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW THE SPIDER WORKS
 * ────────────────────
 *
 * BODY (trail-buffer FK):
 *   The thorax/head position (body_joint[0]) integrates a sinusoidal
 *   turn rate each tick, identical to the snake FK approach.  Past
 *   positions are stored in a circular trail buffer.  Each body joint
 *   i is placed at arc-length i*BODY_SEG_LEN behind the head via
 *   trail_sample() — no per-joint angle formula needed.
 *
 * LEGS (2-joint analytical IK):
 *   Each leg has a hip (root, attached to the body), knee (mid-joint),
 *   and foot (end effector).  Given a hip position H and foot target T:
 *
 *     1. dist = |T - H|, clamped to [|UPPER-LOWER|+1, UPPER+LOWER-1]
 *     2. Law of cosines: cos_angle = (dist²+UPPER²-LOWER²)/(2·dist·UPPER)
 *     3. angle_at_hip = acos(cos_angle)
 *     4. base_angle   = atan2(dy, dx)
 *     5. Left legs:  knee_angle = base_angle + angle_at_hip  (knee out-left)
 *        Right legs: knee_angle = base_angle - angle_at_hip  (knee out-right)
 *     6. knee = hip + UPPER * (cos(knee_angle), sin(knee_angle))
 *
 *   The foot always reaches the target exactly — IK solves it analytically
 *   with no iteration needed for 2-joint chains.
 *
 * STEP GAIT (alternating tripod):
 *   Legs are grouped in two tripods: A={0,2,4} and B={1,3,5}.
 *   A leg triggers a step when its foot drifts > STEP_TRIGGER_DIST from
 *   its ideal position, or the hip-to-foot distance > MAX_STRETCH.
 *   Only one tripod steps at a time (while A steps, B is planted and
 *   vice-versa), giving the classic insect tripod gait.
 *   During a step, the foot smoothly lerps to its new target over
 *   STEP_DURATION seconds using a smoothstep ease-in/ease-out curve.
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   arrow keys    steer in 4 directions (gradual turn)
 *   w / s         speed faster / slower
 *   [/]           raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ik_spider.c -o ik_spider -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm 1 — Trail-buffer FK (body):
 *   The head position history is stored tick-by-tick in a circular buffer.
 *   Body joints are placed by measuring cumulative arc length backward along
 *   the trail and linearly interpolating.  Any path the head carves (curves,
 *   turns) propagates naturally to the body.
 *
 * Algorithm 2 — 2-Joint Analytical IK (legs):
 *   Given hip H and target T, the knee position is solved in closed form
 *   using the law of cosines.  No iteration, no singularity handling beyond
 *   distance clamping.  Left vs right legs choose opposite knee-bend
 *   directions so knees splay outward from the body center.
 *
 * Algorithm 3 — Alternating Tripod Gait:
 *   Insects use a tripod gait: legs {0,2,4} lift together while {1,3,5}
 *   are planted, then swap.  This guarantees 3-point ground contact at all
 *   times — statically stable.  Step timing is driven by foot drift from
 *   the ideal reach position, so the gait automatically adapts to speed.
 *
 * Data-structure:
 *   Circular trail buffer Vec2 trail[TRAIL_CAP] for body FK.
 *   Per-leg: foot_pos (planted), foot_old (start of step), step_target
 *   (where foot is heading), step_t (0→1 progress), stepping flag.
 *   Two snapshot arrays prev_body/prev_hip/prev_knee/prev_foot enable
 *   sub-tick alpha lerp for smooth rendering.
 *
 * Rendering:
 *   Legs drawn as segmented chain lines (hip→knee→foot): alternating
 *   direction char and '.' gives -.-.- / |.|.| / \.\.\  feel per angle.
 *   Knee: 'o' joint node.  Feet: '*' planted, '.' lifting.
 *   Body drawn as a dense bead chain, tail→head.  Head: directional arrow.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * Define our own if the toolchain omits it.
 */
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    HUD_COLS        =  96,
    FPS_UPDATE_MS   = 500,

    N_PAIRS         =   7,
    HUD_PAIR        =   8,
    N_THEMES        =  10,    /* selectable colour themes (cycle with 't') */

    N_LEGS          =   6,    /* 3 per side                                */
    N_BODY_SEGS     =   5,    /* body trail-buffer FK segments             */
    TRAIL_CAP       = 1024,   /* circular trail buffer capacity            */
};

/* Body motion */
#define BODY_SEG_LEN      18.0f   /* px between consecutive body joints    */
#define BODY_SPEED        45.0f   /* head translation speed, px/s          */
#define BODY_SPEED_MIN    10.0f
#define BODY_SPEED_MAX   200.0f
#define TURN_RATE          2.5f   /* rad/s — steering rate toward target heading */

/* IK leg geometry */
#define UPPER_LEN         55.0f   /* hip-to-knee segment length, px        */
#define LOWER_LEN         48.0f   /* knee-to-foot segment length, px       */

/*
 * HIP_DIST_FACTOR — hip offset from body centerline as fraction of screen
 * pixel height.
 */
#define HIP_DIST_FACTOR   0.07f

/* Step / gait parameters */
#define STEP_REACH_FACTOR 0.68f   /* ideal reach = (UPPER+LOWER)*factor    */
#define STEP_TRIGGER_DIST 28.0f   /* px drift before step triggers         */
#define MAX_STRETCH       65.0f   /* absolute max hip-to-foot before step  */
#define STEP_DURATION     0.22f   /* seconds for one complete step         */

/* Body bead fill step (px) */
#define DRAW_STEP_PX      5.0f
/*
 * DRAW_LEG_STEP_PX — step for leg direction-char lines.
 * Set to CELL_W (8 px) so each sample lands in a distinct terminal column:
 * exactly one char per cell traversed, no adjacent duplicates.
 */
#define DRAW_LEG_STEP_PX  8.0f

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cell dimensions (aspect-ratio bridge) */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/*
 * LEG_ANGLE[i] — angle (radians) added to body_forward to get the ideal
 * step direction for leg i, measured in body-local space.
 * Legs 0/1 = front pair, 2/3 = mid pair, 4/5 = rear pair.
 * Left legs (even): angle is positive (left side).
 * Right legs (odd): angle is negative (right side).
 */
static const float LEG_ANGLE[N_LEGS] = {
     0.55f,   /* leg 0 front-left  */
    -0.55f,   /* leg 1 front-right */
     1.57f,   /* leg 2 mid-left    (straight out) */
    -1.57f,   /* leg 3 mid-right   */
     2.60f,   /* leg 4 rear-left   */
    -2.60f,   /* leg 5 rear-right  */
};

/*
 * HIP_BODY_T[i] — parametric position (0=head, 1=tail) along the body
 * centerline where each hip is attached.
 */
static const float HIP_BODY_T[N_LEGS] = {
    0.15f,    /* leg 0 front-left  */
    0.15f,    /* leg 1 front-right */
    0.50f,    /* leg 2 mid-left    */
    0.50f,    /* leg 3 mid-right   */
    0.85f,    /* leg 4 rear-left   */
    0.85f,    /* leg 5 rear-right  */
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color — themed arachnid palette                                    */
/* ===================================================================== */

/*
 * Theme — one runtime colour scheme.
 *   col[0..6] → pairs 1–7:
 *     col[0-2]  body gradient (tail=1 → head=3)
 *     col[3-4]  leg segments  (upper=4, lower=5)
 *     col[5]    planted foot  (pair 6, bold '*')
 *     col[6]    stepping foot (pair 7, dim 'o')
 *   hud         pair 8 (status bars)
 */
typedef struct {
    const char *name;
    int col[N_PAIRS];   /* 7 entries: pairs 1–7 */
    int hud;
} Theme;

/*
 * THEMES[10] — ten selectable palettes.  Press 't' to cycle.
 *
 *  0  Arachnid — dark red body + olive legs (original)
 *  1  Scarlet  — crimson body + red legs
 *  2  Toxic    — dark green body + lime legs
 *  3  Ocean    — deep navy body + aqua legs
 *  4  Nova     — violet body + pink legs
 *  5  Ember    — dark amber body + orange legs
 *  6  Aurora   — teal body + gold legs
 *  7  Ghost    — charcoal body + white legs
 *  8  Fire     — red body + flame legs
 *  9  Neon     — violet body + hot-pink legs
 */
/*
 * Only col[2] (pair 3), col[5] (pair 6), col[6] (pair 7) and hud are
 * used by the renderer.  col[0/1/3/4] are set consistently but ignored.
 *
 *   col[2]  = main spider colour  (body + all legs)
 *   col[5]  = planted foot        (bright accent, should contrast well)
 *   col[6]  = stepping foot       (near-black / very dim ghost)
 */
static const Theme THEMES[N_THEMES] = {
    {"Arachnid",{52, 88,124, 58, 64, 70,236}, 226},  /* red + olive foot    */
    {"Scarlet", {88,124,160, 52, 88,196,240}, 208},  /* crimson + red foot  */
    {"Toxic",   {22, 28, 34, 28, 34, 82,236},  46},  /* green + lime foot   */
    {"Ocean",   {17, 18, 20, 18, 27, 51,236},  51},  /* navy + cyan foot    */
    {"Nova",    {54, 93,129, 93,129,165,240}, 213},  /* violet + pink foot  */
    {"Ember",   {52, 94,130,130,130,208,240}, 208},  /* amber + orange foot */
    {"Aurora",  {22, 28, 35, 28, 35,221,240}, 221},  /* teal + gold foot    */
    {"Ghost",   {234,238,242,238,242,254,240},252},  /* grey + white foot   */
    {"Fire",    {52, 88,196, 88,124,226,240}, 226},  /* red + yellow foot   */
    {"Neon",    {57, 93,201, 93,129,201,240}, 197},  /* violet + pink foot  */
};

/*
 * theme_apply() — rebind pairs 1–7 and HUD pair to the given theme index.
 * Safe to call after start_color() — ncurses picks up the new palette
 * on the next draw cycle without reinitialising the display.
 */
static void theme_apply(int idx)
{
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_PAIRS; p++)
        init_pair(p + 1, t->col[p], COLOR_BLACK);
    init_pair(HUD_PAIR, t->hud, COLOR_BLACK);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        init_pair(1, COLOR_RED,    COLOR_BLACK);
        init_pair(2, COLOR_RED,    COLOR_BLACK);
        init_pair(3, COLOR_RED,    COLOR_BLACK);
        init_pair(4, COLOR_GREEN,  COLOR_BLACK);
        init_pair(5, COLOR_GREEN,  COLOR_BLACK);
        init_pair(6, COLOR_GREEN,  COLOR_BLACK);
        init_pair(7, COLOR_BLACK,  COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio helpers                          */
/* ===================================================================== */

/*
 * All physics positions are in a square pixel space (1 unit = 1 physical px).
 * Only at draw time do we convert to terminal cell coordinates.
 * formula: cell = floor(px / CELL_DIM + 0.5)  (round to nearest)
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Spider                                                    */
/* ===================================================================== */

/* ── §5a  vec2 helpers ─────────────────────────────────────────────── */

typedef struct { float x, y; } Vec2;

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline Vec2 vec2_add(Vec2 a, Vec2 b)   { return (Vec2){ a.x+b.x, a.y+b.y }; }
static inline Vec2 vec2_sub(Vec2 a, Vec2 b)   { return (Vec2){ a.x-b.x, a.y-b.y }; }
static inline Vec2 vec2_scale(Vec2 a, float s) { return (Vec2){ a.x*s, a.y*s }; }
static inline Vec2 vec2_lerp(Vec2 a, Vec2 b, float t)
{
    return (Vec2){ a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t };
}

static inline float vec2_len(Vec2 a)
{
    return sqrtf(a.x*a.x + a.y*a.y);
}
static inline float vec2_dist(Vec2 a, Vec2 b)
{
    return vec2_len(vec2_sub(b, a));
}
static inline Vec2 vec2_norm(Vec2 a)
{
    float len = vec2_len(a);
    if (len < 1e-6f) return (Vec2){1.0f, 0.0f};
    return (Vec2){ a.x/len, a.y/len };
}

/*
 * smoothstep(t) — cubic ease-in/ease-out in [0,1].
 * smoothstep(0)=0, smoothstep(1)=1, derivative=0 at both ends.
 * Produces a smooth S-curve for leg step animation.
 */
static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/*
 * rotate2d() — rotate vector (x,y) by angle θ (radians) around origin.
 * Standard 2D rotation matrix: [cosθ -sinθ; sinθ cosθ].
 */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x*c - v.y*s, v.x*s + v.y*c };
}

/* ── §5b  trail helpers ─────────────────────────────────────────────── */

/*
 * Spider — complete simulation state.
 *
 * BODY FK:
 *   Same trail-buffer approach as snake_forward_kinematics.c.
 *   body_joint[0] = thorax/head; body_joint[1..N_BODY_SEGS] = body segments.
 *   trail[] records the per-tick position history of body_joint[0].
 *
 * LEGS:
 *   6 legs, each described by: hip (world-space root attached to body),
 *   knee (mid-joint, computed by IK), foot_pos (current planted target).
 *   During a step: foot animates from foot_old to step_target over STEP_DURATION s.
 *
 * PREV arrays: snapshot from start of tick for sub-tick alpha interpolation.
 */
typedef struct {
    /* ── body ── */
    Vec2  trail[TRAIL_CAP];
    int   trail_head, trail_count;
    Vec2  body_joint[N_BODY_SEGS + 1];
    Vec2  prev_body[N_BODY_SEGS + 1];
    float heading;          /* current direction (rad, 0=right)    */
    float target_heading;   /* desired direction set by arrow keys  */
    float move_speed;

    /* ── legs ── */
    Vec2  hip[N_LEGS];          /* world-space hip root (updated from body) */
    Vec2  knee[N_LEGS];         /* mid-joint computed by 2-joint IK          */
    Vec2  foot_pos[N_LEGS];     /* planted foot (IK target)                  */
    Vec2  foot_old[N_LEGS];     /* foot at step-start (for lerp)             */
    Vec2  step_target[N_LEGS];  /* where foot steps to                       */
    bool  stepping[N_LEGS];     /* is leg currently in swing phase?          */
    float step_t[N_LEGS];       /* step progress 0→1                         */

    /* ── render prev snapshots ── */
    Vec2  prev_hip[N_LEGS];
    Vec2  prev_knee[N_LEGS];
    Vec2  prev_foot[N_LEGS];

    float hip_dist;   /* hip offset from body centerline (px), set at init  */
    bool  paused;
    int   theme_idx;  /* current colour theme [0, N_THEMES)                 */
} Spider;

/* trail push/at/sample — identical logic to snake_forward_kinematics.c */

static void trail_push(Spider *sp, Vec2 pos)
{
    sp->trail_head = (sp->trail_head + 1) % TRAIL_CAP;
    sp->trail[sp->trail_head] = pos;
    if (sp->trail_count < TRAIL_CAP) sp->trail_count++;
}

static inline Vec2 trail_at(const Spider *sp, int k)
{
    return sp->trail[(sp->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample() — position at arc-length dist px behind the head.
 * Walks the trail accumulating segment lengths; interpolates the crossing.
 */
static Vec2 trail_sample(const Spider *sp, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(sp, 0);

    for (int k = 1; k < sp->trail_count; k++) {
        Vec2  b   = trail_at(sp, k);
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx*dx + dy*dy);

        if (accum + seg >= dist) {
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx*t, a.y + dy*t };
        }
        accum += seg;
        a      = b;
    }
    return trail_at(sp, sp->trail_count - 1);
}

/* ── §5c  body motion ──────────────────────────────────────────────── */

/*
 * move_body() — steer heading toward target_heading + translate body_joint[0].
 *
 *   1. Normalise angular diff to [−π, π] (short-arc) and clamp to TURN_RATE.
 *   2. Translate body_joint[0] along current heading at move_speed.
 *   3. Toroidal wrap at screen pixel boundaries.
 *   4. Push body_joint[0] into the trail.
 */
static void move_body(Spider *sp, float dt, int cols, int rows)
{
    float diff = sp->target_heading - sp->heading;
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    sp->heading += clampf(diff, -TURN_RATE * dt, TURN_RATE * dt);

    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    sp->body_joint[0].x += sp->move_speed * cosf(sp->heading) * dt;
    sp->body_joint[0].y += sp->move_speed * sinf(sp->heading) * dt;

    if (sp->body_joint[0].x <  0.0f) sp->body_joint[0].x += wpx;
    if (sp->body_joint[0].x >= wpx)  sp->body_joint[0].x -= wpx;
    if (sp->body_joint[0].y <  0.0f) sp->body_joint[0].y += hpx;
    if (sp->body_joint[0].y >= hpx)  sp->body_joint[0].y -= hpx;

    trail_push(sp, sp->body_joint[0]);
}

/* ── §5d  body joints ──────────────────────────────────────────────── */

/*
 * compute_body_joints() — place body_joint[1..N_BODY_SEGS] via trail sampling.
 * body_joint[0] is already updated by move_body().
 */
static void compute_body_joints(Spider *sp)
{
    for (int i = 1; i <= N_BODY_SEGS; i++) {
        sp->body_joint[i] = trail_sample(sp, (float)i * BODY_SEG_LEN);
    }
}

/* ── §5e  hip placement ─────────────────────────────────────────────── */

/*
 * compute_hips() — place each hip in world space.
 *
 * Each hip attaches to the body at parametric position HIP_BODY_T[i]
 * (0=head, 1=tail) and is offset laterally by hip_dist px perpendicular
 * to the body's local forward direction.
 *
 * Body forward at attachment point = normalise(body_joint[k] - body_joint[k+1]).
 * Body normal (perpendicular, pointing left) = rotate forward by +90°.
 * Left legs (even index): hip = attach_pt + normal * hip_dist
 * Right legs (odd index): hip = attach_pt - normal * hip_dist
 */
static void compute_hips(Spider *sp)
{
    for (int i = 0; i < N_LEGS; i++) {
        /* Find attachment point along body by interpolating body joints */
        float t_body   = HIP_BODY_T[i] * (float)N_BODY_SEGS;
        int   seg_idx  = (int)t_body;
        if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
        float frac     = t_body - (float)seg_idx;

        Vec2 attach = vec2_lerp(sp->body_joint[seg_idx],
                                sp->body_joint[seg_idx + 1], frac);

        /* Local forward direction at this body point */
        Vec2 fwd;
        if (seg_idx + 1 <= N_BODY_SEGS) {
            fwd = vec2_norm(vec2_sub(sp->body_joint[seg_idx],
                                     sp->body_joint[seg_idx + 1]));
        } else {
            fwd = (Vec2){ cosf(sp->heading), sinf(sp->heading) };
        }

        /* Left normal: rotate forward +90° → (-fy, fx) */
        Vec2 left_norm = (Vec2){ -fwd.y, fwd.x };

        /* Even legs = left side, odd legs = right side */
        float side = (i % 2 == 0) ? 1.0f : -1.0f;
        sp->hip[i] = vec2_add(attach, vec2_scale(left_norm, side * sp->hip_dist));
    }
}

/* ── §5f  2-joint analytical IK ────────────────────────────────────── */

/*
 * solve_ik() — 2-joint analytical IK for one leg.
 *
 * Given: hip position H, foot target T, upper/lower segment lengths,
 *        and whether this is a left or right leg (determines knee bend).
 *
 * ALGORITHM (law of cosines):
 *
 *  Step 1: Compute distance vector and clamped reach distance.
 *    dx = T.x - H.x,  dy = T.y - H.y
 *    dist = sqrtf(dx²+dy²)
 *    Clamp dist to [|UPPER-LOWER|+1, UPPER+LOWER-1] to avoid acos domain error
 *    (dist > UPPER+LOWER = over-extended, dist < |UPPER-LOWER| = folded in on itself).
 *
 *  Step 2: Law of cosines → angle at hip.
 *    In the triangle (hip, knee, foot):
 *      side_a = LOWER (knee→foot), side_b = UPPER (hip→knee), side_c = dist (hip→foot)
 *    cos(angle_at_hip) = (dist² + UPPER² - LOWER²) / (2 * dist * UPPER)
 *    angle_at_hip = acosf(clamped value)
 *
 *  Step 3: Base angle from hip toward target.
 *    base_angle = atan2f(dy, dx)
 *
 *  Step 4: Knee direction = base_angle ± angle_at_hip.
 *    Left legs: knee bends outward (left = positive normal side)
 *      → add angle_at_hip (rotates knee counter-clockwise, out to the left)
 *    Right legs: subtract angle_at_hip (rotates knee clockwise, out to the right)
 *
 *  Step 5: Place knee.
 *    knee.x = hip.x + UPPER * cos(knee_angle)
 *    knee.y = hip.y + UPPER * sin(knee_angle)
 *
 * Output: writes knee position into *knee_out.
 * The foot is always the target T (end-effector reaches target exactly).
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = sqrtf(dx*dx + dy*dy);

    /* Clamp dist to reachable range (avoid acos NaN) */
    float min_r = fabsf(UPPER_LEN - LOWER_LEN) + 1.0f;
    float max_r = UPPER_LEN + LOWER_LEN - 1.0f;
    dist = clampf(dist, min_r, max_r);

    /* Reconstruct a clamped target direction if dist changed */
    float base_angle = atan2f(dy, dx);

    /* Law of cosines: angle at the hip vertex */
    float cos_hip = (dist*dist + UPPER_LEN*UPPER_LEN - LOWER_LEN*LOWER_LEN)
                    / (2.0f * dist * UPPER_LEN);
    cos_hip = clampf(cos_hip, -1.0f, 1.0f);
    float angle_hip = acosf(cos_hip);

    /* Knee bends outward from body center */
    float knee_angle = is_left ? (base_angle + angle_hip)
                                : (base_angle - angle_hip);

    knee_out->x = hip.x + UPPER_LEN * cosf(knee_angle);
    knee_out->y = hip.y + UPPER_LEN * sinf(knee_angle);
}

/* ── §5g  step logic ────────────────────────────────────────────────── */

/*
 * compute_ideal_foot() — where leg i wants its foot to be when not stepping.
 *
 * ideal_foot = hip + STEP_REACH × (unit vector in step direction)
 * step direction = body forward direction rotated by LEG_ANGLE[i].
 * Body forward = (cos(heading), sin(heading)).
 */
static Vec2 compute_ideal_foot(const Spider *sp, int i)
{
    Vec2 fwd   = (Vec2){ cosf(sp->heading), sinf(sp->heading) };
    Vec2 dir   = rotate2d(fwd, LEG_ANGLE[i]);
    float reach = (UPPER_LEN + LOWER_LEN) * STEP_REACH_FACTOR;
    return vec2_add(sp->hip[i], vec2_scale(dir, reach));
}

/*
 * update_steps() — independent per-leg step scheduling.
 *
 * Each leg checks its own drift / stretch and steps as soon as it is ready.
 * Stability constraint: at most N_LEGS/2 (3) legs in the air simultaneously,
 * guaranteeing 3-point ground contact at all times.
 *
 * Contrast with the old alternating-tripod logic: that held the entire
 * opposite tripod (3 legs) frozen while one tripod stepped.  Here any leg
 * is free to swing the moment it is ready AND the in-air count is below 3.
 * The result is fluid, organic-looking movement: legs step at slightly
 * different times driven by their individual geometry, not a group timer.
 */
static void update_steps(Spider *sp, float dt)
{
    /* Count how many legs are currently airborne */
    int n_air = 0;
    for (int i = 0; i < N_LEGS; i++)
        if (sp->stepping[i]) n_air++;

    for (int i = 0; i < N_LEGS; i++) {

        /* ── Screen-wrap snap ────────────────────────────────────────── */
        float snap_stretch = vec2_dist(sp->foot_pos[i], sp->hip[i]);
        if (snap_stretch > UPPER_LEN + LOWER_LEN - 2.0f) {
            sp->foot_pos[i]    = compute_ideal_foot(sp, i);
            sp->foot_old[i]    = sp->foot_pos[i];
            sp->step_target[i] = sp->foot_pos[i];
            if (sp->stepping[i]) { sp->stepping[i] = false; n_air--; }
            sp->step_t[i]      = 0.0f;
            solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
            continue;
        }

        if (!sp->stepping[i]) {
            Vec2  ideal   = compute_ideal_foot(sp, i);
            float drift   = vec2_dist(sp->foot_pos[i], ideal);
            float stretch = vec2_dist(sp->foot_pos[i], sp->hip[i]);

            /* Step if foot has drifted too far OR leg is over-stretched,
             * but only when below the in-air cap (stability guarantee). */
            if ((drift > STEP_TRIGGER_DIST || stretch > MAX_STRETCH)
                    && n_air < N_LEGS / 2) {
                sp->stepping[i]    = true;
                sp->step_t[i]      = 0.0f;
                sp->foot_old[i]    = sp->foot_pos[i];
                sp->step_target[i] = ideal;
                n_air++;
            }
        } else {
            /* Advance step animation */
            sp->step_t[i] += dt / STEP_DURATION;
            if (sp->step_t[i] >= 1.0f) {
                sp->step_t[i]   = 1.0f;
                sp->foot_pos[i] = sp->step_target[i];
                sp->stepping[i] = false;
                n_air--;
            } else {
                float ease = smoothstep(sp->step_t[i]);
                sp->foot_pos[i] = vec2_lerp(sp->foot_old[i],
                                             sp->step_target[i], ease);
            }
        }
    }

    /* Recompute IK for all legs */
    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
}

/* ── §5h  rendering ─────────────────────────────────────────────────── */

/*
 * head_glyph() — directional arrow for spider thorax head.
 */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)'>';
    if (deg < 135.0f)                   return (chtype)'v';
    if (deg < 225.0f)                   return (chtype)'<';
    return                              (chtype)'^';
}

/*
 * seg_glyph() — direction char for a vector (dx,dy): - \ | /
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)'-';
    if (deg < 67.5f)                   return (chtype)'\\';
    if (deg < 112.5f)                  return (chtype)'|';
    return                             (chtype)'/';
}

/*
 * draw_leg_line() — segmented-limb line for a leg segment.
 *
 * Alternates direction char and '.' so each segment reads as a chain:
 *   horizontal  →  -.-.-.-
 *   vertical    →  |.|.|.|
 *   diagonal    →  \.\.\.\  or  /./././
 * This gives an organic arthropod-limb feel in pure ASCII.
 */
static void draw_leg_line(WINDOW *w,
                          Vec2 a, Vec2 b,
                          int pair, attr_t attr,
                          int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_LEG_STEP_PX) + 1;
    int    prev_cx = -9999, prev_cy = -9999;
    int    phase   = 0;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx*u);
        int   cy = px_to_cell_y(a.y + dy*u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        chtype ch = (phase & 1) ? (chtype)'.' : glyph;
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(pair) | attr);
        phase++;
    }
}

/*
 * draw_line_beads() — stamp bead char 'o' along a segment.
 * Used for the body centerline; joint node markers drawn on top.
 */
static void draw_line_beads(WINDOW *w,
                             Vec2 a, Vec2 b,
                             int pair, attr_t attr,
                             int cols, int rows,
                             int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx*u);
        int   cy = px_to_cell_y(a.y + dy*u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx; *prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/*
 * render_spider() — draw the complete spider frame.
 *
 * Two distinct rendering styles for visual clarity:
 *
 *   LEGS  — direction chars (\ | / -)  so each limb reads as a crisp angled
 *            line.  Knee node 'o' marks the articulation.
 *            Foot: '*' planted (pair 6 bold), 'o' stepping (pair 7 dim).
 *
 *   BODY  — bead chain (o 0 . fill + '0' node markers) identical to the
 *            other FK/IK demos.  Head gets a directional arrow glyph.
 *
 * Draw order (back to front):
 *   1. Legs  — direction-char lines: hip→knee, knee→foot
 *   2. Legs  — knee 'o' + foot '*'/'o' node markers
 *   3. Body  — bead fill tail→head
 *   4. Body  — '0' node markers
 *   5. Head  — directional arrow
 */
static void render_spider(const Spider *sp, WINDOW *w,
                          int cols, int rows, float alpha)
{
    /* Build alpha-interpolated positions */
    Vec2 r_body[N_BODY_SEGS + 1];
    for (int i = 0; i <= N_BODY_SEGS; i++) {
        r_body[i] = vec2_lerp(sp->prev_body[i], sp->body_joint[i], alpha);
    }

    Vec2 r_hip[N_LEGS], r_knee[N_LEGS], r_foot[N_LEGS];
    for (int i = 0; i < N_LEGS; i++) {
        r_hip[i]  = vec2_lerp(sp->prev_hip[i],  sp->hip[i],      alpha);
        r_knee[i] = vec2_lerp(sp->prev_knee[i], sp->knee[i],     alpha);
        r_foot[i] = vec2_lerp(sp->prev_foot[i], sp->foot_pos[i], alpha);
    }

    /* 1. Legs — Unicode box-drawing lines (─ ╲ │ ╱), bold for crispness */
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, r_hip[i],  r_knee[i], 3, A_BOLD, cols, rows);
        draw_leg_line(w, r_knee[i], r_foot[i], 3, A_BOLD, cols, rows);
    }

    /* 2. Leg joint nodes on top */
    for (int i = 0; i < N_LEGS; i++) {
        /* Knee: 'o' bold — joint node, contrasts with '.' beads on limb */
        int kx = px_to_cell_x(r_knee[i].x);
        int ky = px_to_cell_y(r_knee[i].y);
        if (kx >= 0 && kx < cols && ky >= 0 && ky < rows) {
            wattron(w, COLOR_PAIR(3) | A_BOLD);
            mvwaddch(w, ky, kx, (chtype)'o');
            wattroff(w, COLOR_PAIR(3) | A_BOLD);
        }

        /* Foot: '*' planted (pair 6 bold), '.' stepping (pair 7 dim) */
        int fx = px_to_cell_x(r_foot[i].x);
        int fy = px_to_cell_y(r_foot[i].y);
        if (fx >= 0 && fx < cols && fy >= 0 && fy < rows) {
            if (sp->stepping[i]) {
                wattron(w, COLOR_PAIR(7) | A_DIM);
                mvwaddch(w, fy, fx, (chtype)'.');
                wattroff(w, COLOR_PAIR(7) | A_DIM);
            } else {
                wattron(w, COLOR_PAIR(6) | A_BOLD);
                mvwaddch(w, fy, fx, (chtype)'*');
                wattroff(w, COLOR_PAIR(6) | A_BOLD);
            }
        }
    }

    /* 3. Body bead fill — tail→head, pair 3 A_BOLD */
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = N_BODY_SEGS - 1; i >= 0; i--) {
        draw_line_beads(w, r_body[i+1], r_body[i],
                        3, A_BOLD, cols, rows, &prev_cx, &prev_cy);
    }

    /* 4. Body node markers: '0' at each joint, pair 3 bold */
    for (int i = N_BODY_SEGS; i >= 1; i--) {
        int bx = px_to_cell_x(r_body[i].x);
        int by = px_to_cell_y(r_body[i].y);
        if (bx < 0 || bx >= cols || by < 0 || by >= rows) continue;
        wattron(w, COLOR_PAIR(3) | A_BOLD);
        mvwaddch(w, by, bx, (chtype)'0');
        wattroff(w, COLOR_PAIR(3) | A_BOLD);
    }

    /* 5. Head glyph — directional arrow, pair 3 A_BOLD */
    int hx = px_to_cell_x(r_body[0].x);
    int hy = px_to_cell_y(r_body[0].y);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        wattron(w, COLOR_PAIR(3) | A_BOLD);
        mvwaddch(w, hy, hx, head_glyph(sp->heading));
        wattroff(w, COLOR_PAIR(3) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Spider spider; } Scene;

/*
 * scene_init() — place spider at screen center with pre-populated trail.
 *
 * Body starts at 50% width, 50% height, heading slightly south-east.
 * Trail pre-filled straight backward (same technique as snake FK scene_init).
 * Feet placed at ideal positions around the initial body pose.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Spider *sp = &sc->spider;

    sp->move_speed     = BODY_SPEED;
    sp->heading        = 0.0f;   /* start facing right */
    sp->target_heading = 0.0f;
    sp->hip_dist       = (float)(rows * CELL_H) * HIP_DIST_FACTOR;
    sp->paused     = false;
    sp->theme_idx  = 0;

    /* Place head at screen center */
    sp->body_joint[0].x = (float)(cols * CELL_W) * 0.50f;
    sp->body_joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-fill trail straight backward */
    float bx = cosf(sp->heading + (float)M_PI);
    float by = sinf(sp->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        sp->trail[k].x = sp->body_joint[0].x + (float)k * bx;
        sp->trail[k].y = sp->body_joint[0].y + (float)k * by;
    }
    sp->trail_head  = 0;
    sp->trail_count = TRAIL_CAP;

    /* Compute initial body joints */
    compute_body_joints(sp);
    compute_hips(sp);

    /* Solve initial IK and place feet at ideal positions */
    for (int i = 0; i < N_LEGS; i++) {
        sp->foot_pos[i]    = compute_ideal_foot(sp, i);
        sp->foot_old[i]    = sp->foot_pos[i];
        sp->step_target[i] = sp->foot_pos[i];
        sp->stepping[i]    = false;
        sp->step_t[i]      = 0.0f;

        bool is_left = (i % 2 == 0);
        solve_ik(sp->hip[i], sp->foot_pos[i], is_left, &sp->knee[i]);
    }

    /* Snapshot prev arrays for alpha lerp (no-op on first frame) */
    memcpy(sp->prev_body, sp->body_joint, sizeof sp->body_joint);
    memcpy(sp->prev_hip,  sp->hip,        sizeof sp->hip);
    memcpy(sp->prev_knee, sp->knee,       sizeof sp->knee);
    memcpy(sp->prev_foot, sp->foot_pos,   sizeof sp->foot_pos);
}

/*
 * scene_tick() — one fixed-step physics update.
 *
 * ORDER:
 *   1. Save prev snapshots (before any physics).
 *   2. If paused, return early (lerp freezes cleanly).
 *   3. Move body (heading integration + wrap + trail push).
 *   4. Compute body joints from updated trail.
 *   5. Update hip positions from new body joints.
 *   6. Update step logic and advance step animations.
 *   7. Solve IK for all legs.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Spider *sp = &sc->spider;

    /* Step 1 — snapshot prev */
    memcpy(sp->prev_body, sp->body_joint, sizeof sp->body_joint);
    memcpy(sp->prev_hip,  sp->hip,        sizeof sp->hip);
    memcpy(sp->prev_knee, sp->knee,       sizeof sp->knee);
    memcpy(sp->prev_foot, sp->foot_pos,   sizeof sp->foot_pos);

    if (sp->paused) return;   /* Step 2 */

    move_body(sp, dt, cols, rows);         /* Step 3 */
    compute_body_joints(sp);               /* Step 4 */
    compute_hips(sp);                      /* Step 5 */
    update_steps(sp, dt);                  /* Steps 6 + 7 */
}

static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_spider(&sc->spider, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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
 * Count how many legs are currently in the stepping tripod A vs B,
 * used to compute step rate for the HUD display.
 */
static float count_step_hz(const Spider *sp, float sim_fps)
{
    int n_step = 0;
    for (int i = 0; i < N_LEGS; i++) {
        if (sp->stepping[i]) n_step++;
    }
    /* rough estimate: n_stepping / STEP_DURATION gives steps/sec */
    (void)sim_fps;
    return (float)n_step / STEP_DURATION;
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Spider *sp = &sc->spider;
    float step_hz = count_step_hz(sp, (float)sim_fps);

    char buf[HUD_COLS + 1];
    float deg = sp->heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    const char *dir_arrow =
        (deg < 45.0f || deg >= 315.0f) ? ">" :
        (deg < 135.0f)                  ? "v" :
        (deg < 225.0f)                  ? "<" : "^";

    snprintf(buf, sizeof buf,
             " IK-SPIDER  dir:%s  spd:%.0f  step:%.0fHz  [%s]  %s  %.1ffps  %dHz ",
             dir_arrow, sp->move_speed, step_hz,
             THEMES[sp->theme_idx].name,
             sp->paused ? "PAUSED" : "crawling",
             fps, sim_fps);

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  arrows:steer  w/s:speed  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
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
    Spider *sp  = &app->scene.spider;
    float   wpx = (float)(app->screen.cols * CELL_W);
    float   hpx = (float)(app->screen.rows * CELL_H);
    if (sp->body_joint[0].x >= wpx) sp->body_joint[0].x = wpx - 1.0f;
    if (sp->body_joint[0].y >= hpx) sp->body_joint[0].y = hpx - 1.0f;
    /* Recompute hip_dist for new terminal size; preserve theme */
    sp->hip_dist = hpx * HIP_DIST_FACTOR;
    app->need_resize = 0;
    /* Re-apply theme in case ncurses reset pairs during resize */
    theme_apply(sp->theme_idx);
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         pause / resume
 *   arrow keys    steer — set target_heading; body turns gradually
 *   w / s         speed ×1.25 / ÷1.25
 *   ] / [         sim Hz + / - SIM_FPS_STEP
 */
static bool app_handle_key(App *app, int ch)
{
    Spider *sp = &app->scene.spider;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sp->paused = !sp->paused; break;

    case KEY_RIGHT: sp->target_heading =  0.0f;                break;
    case KEY_DOWN:  sp->target_heading =  (float)M_PI * 0.5f; break;
    case KEY_LEFT:  sp->target_heading =  (float)M_PI;        break;
    case KEY_UP:    sp->target_heading = -(float)M_PI * 0.5f; break;

    case 'w':
        sp->move_speed *= 1.25f;
        if (sp->move_speed > BODY_SPEED_MAX) sp->move_speed = BODY_SPEED_MAX;
        break;
    case 's':
        sp->move_speed /= 1.25f;
        if (sp->move_speed < BODY_SPEED_MIN) sp->move_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        sp->theme_idx = (sp->theme_idx + 1) % N_THEMES;
        theme_apply(sp->theme_idx);
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — fixed-step accumulator game loop (identical structure to framework.c §8)
 *
 * ① RESIZE CHECK   — handle SIGWINCH before touching ncurses
 * ② MEASURE dt     — wall-clock since last frame, capped at 100 ms
 * ③ ACCUMULATOR    — fire scene_tick() at sim_fps Hz
 * ④ ALPHA          — sub-tick interpolation factor for smooth render
 * ⑤ FPS COUNTER    — smoothed over 500 ms windows
 * ⑥ FRAME CAP      — sleep to cap render at 60 fps
 * ⑦ DRAW + PRESENT — erase → scene_draw → HUD → doupdate
 * ⑧ DRAIN INPUT    — consume all queued keypresses
 * ───────────────────────────────────────────────────────────────────── */
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

        /* ── ① resize ─────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ─────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ fixed-step accumulator ─────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ──────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ───────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ─────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ───────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
