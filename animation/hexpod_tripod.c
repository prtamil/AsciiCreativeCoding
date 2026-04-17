/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hexpod_tripod.c — 2D Hexapod Robot Walker with Tripod Gait + 2-Joint IK
 *
 * DEMO: A 6-legged robot body (rigid rectangle) translates right across the
 *       terminal.  Alternating tripod gait keeps exactly 3 feet planted at
 *       all times while the other 3 swing forward.  Each leg is a 2-segment
 *       chain solved each tick with analytical (law-of-cosines) IK.
 *       The robot wraps at the right edge and re-enters from the left.
 *
 * STUDY ALONGSIDE: ik_spider.c     (2-joint IK + gait patterns)
 *                  ragdoll_figure.c (framework structure)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep
 *   §3  color         — 8 robot-themed palettes
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Hexapod struct + algorithms
 *       §5a  solve_ik       — 2-joint analytical IK (law of cosines)
 *       §5b  hip_world_pos  — body-local → world pixel coords
 *       §5c  rest_target    — ideal foot rest position per leg
 *       §5d  gait_tick      — tripod gait state machine
 *       §5e  hexapod_tick   — body translation + gait + IK per tick
 *       §5f  hexapod_draw   — render body rectangle + legs
 *   §6  scene         — scene_init / scene_tick / scene_draw
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW TRIPOD GAIT WORKS
 * ─────────────────────
 * Six legs split into two interlocked triangles that alternate stepping:
 *   Group A = {0 left-front, 3 right-mid, 4 left-rear}
 *   Group B = {1 right-front, 2 left-mid, 5 right-rear}
 *
 * While group A swings, B's 3 planted feet form a stable support triangle
 * under the body — the robot never tips.  After PHASE_DURATION seconds AND
 * all swinging feet have landed, the other group launches.  Each foot traces
 * a parabolic arc over STEP_DURATION seconds to rest_target (hip + fixed
 * offsets + forward lookahead proportional to speed).
 *
 * HOW 2-JOINT IK WORKS (law of cosines)
 * ──────────────────────────────────────
 * Given hip H, foot T, femur U, tibia L:
 *   dist      = clamp(|T−H|, |U−L|+1, U+L−1)
 *   base      = atan2(T.y−H.y, T.x−H.x)
 *   cos_h     = (dist²+U²−L²) / (2·dist·U)
 *   ah        = acos(cos_h)
 *   Left leg:   knee_angle = base − ah   (knee bends toward −y, outward ↑)
 *   Right leg:  knee_angle = base + ah   (knee bends toward +y, outward ↓)
 *   knee = H + U·(cos(knee_angle), sin(knee_angle))
 *
 * Sign convention is OPPOSITE to ik_spider.c: the spider's hips extend
 * along a sinusoidal heading; these hexapod hips extend ⊥ to motion (±y).
 *
 * Keys:
 *   q / ESC   quit        space  pause / resume
 *   w / s     speed ±     t      cycle colour theme
 *   [ / ]     sim Hz ±
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       hexpod_tripod.c -o hexpod_tripod -lncurses -lm
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    N_PAIRS          =   7,   /* body/femur/tibia/foot/step/knee/unused   */
    HUD_PAIR         =   8,
    N_THEMES         =   8,

    N_LEGS           =   6,
};

/* Body geometry (px) */
#define BODY_LEN         80.0f   /* total length, body faces +x           */
#define BODY_HALF_W      20.0f   /* half-width (legs attach at ±this)     */

/* Leg geometry (px) */
#define UPPER_LEN        40.0f   /* femur (hip → knee)                    */
#define LOWER_LEN        36.0f   /* tibia (knee → foot)                   */

/* Walking speed (px/s) */
#define BODY_SPEED_DEFAULT  40.0f
#define BODY_SPEED_MIN      10.0f
#define BODY_SPEED_MAX     200.0f

/* Gait timing (seconds) */
#define PHASE_DURATION    0.35f  /* one half-cycle (group-A or group-B in air) */
#define STEP_DURATION     0.28f  /* single foot swing time                     */
#define STEP_HEIGHT       12.0f  /* parabolic arc height (−y = upward)         */
#define STEP_LOOKAHEAD    0.18f  /* body_speed × this added to step target     */

/* Steering */
#define TURN_RATE         2.5f  /* rad/s — how fast heading rotates to target */

/* Rendering */
#define DRAW_LEG_STEP_PX  8.0f  /* step size for direction-char leg lines     */

/* Timing primitives */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cell dimensions — aspect-ratio bridge between physics and display */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

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
/* §3  color — 8 robot-themed palettes                                   */
/* ===================================================================== */

/*
 * Color pair semantics:
 *   pair 1  body frame (rectangle edges + cross-braces + hip markers)
 *   pair 2  femur segments (hip → knee)
 *   pair 3  tibia segments (knee → foot)
 *   pair 4  planted foot '*'  (bright accent)
 *   pair 5  stepping foot 'o' (dim, in-flight)
 *   pair 6  knee joint 'o'    (bold)
 *   pair 7  (reserved)
 *   pair 8  HUD status bars
 */
typedef struct {
    const char *name;
    int col[N_PAIRS];   /* pairs 1–7 */
    int hud;            /* pair 8    */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      body  femur  tibia plant step  knee  rsvd  hud  */
    {"Steel",  {245,  67,  75,  46, 214, 231, 238}, 226},
    {"Cobalt", {237,  27,  39,  46, 208, 231, 236}, 226},
    {"Copper", {242, 130, 172,  46, 214, 231, 238}, 220},
    {"Toxin",  {234,  34,  40,  46,  82, 231, 237},  46},
    {"Ember",  {239, 130, 136,  46, 208, 231, 238}, 208},
    {"Ghost",  {240, 251, 254,  46, 252, 255, 238}, 253},
    {"Neon",   {235,  93, 201,  46, 226, 255, 237}, 197},
    {"Ocean",  {234,  27,  51,  46,  51, 231, 237},  51},
};

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
        init_pair(1, COLOR_WHITE,  COLOR_BLACK);
        init_pair(2, COLOR_CYAN,   COLOR_BLACK);
        init_pair(3, COLOR_CYAN,   COLOR_BLACK);
        init_pair(4, COLOR_GREEN,  COLOR_BLACK);
        init_pair(5, COLOR_YELLOW, COLOR_BLACK);
        init_pair(6, COLOR_WHITE,  COLOR_BLACK);
        init_pair(7, COLOR_BLACK,  COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio helpers                          */
/* ===================================================================== */

/*
 * All physics positions are in square pixel space (1 unit = 1 physical px).
 * Only at draw time do we convert to terminal cell coordinates.
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
/* §5  entity — Hexapod                                                   */
/* ===================================================================== */

/* ── vec2 helpers ──────────────────────────────────────────────────── */

typedef struct { float x, y; } Vec2;

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

/* rotate2d — rotate vector v by angle radians (screen space: +y = down) */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ── Per-leg static tables ─────────────────────────────────────────── */

/*
 * Coordinate convention: body faces +x (walk direction).
 *   +y = down on screen.  Left side = −y (top).  Right side = +y (bottom).
 *
 * Leg index map:
 *   0 left-front   1 right-front
 *   2 left-mid     3 right-mid
 *   4 left-rear    5 right-rear
 *
 * Hip attachment offsets from body center.  Left legs sit on the top edge
 * (−y = −BODY_HALF_W), right legs on the bottom (+BODY_HALF_W).
 */
static const float HIP_LOCAL_X[N_LEGS] = {
    BODY_LEN * 0.40f,    /* 0 left-front  */
    BODY_LEN * 0.40f,    /* 1 right-front */
    0.0f,                /* 2 left-mid    */
    0.0f,                /* 3 right-mid   */
   -BODY_LEN * 0.40f,   /* 4 left-rear   */
   -BODY_LEN * 0.40f,   /* 5 right-rear  */
};
static const float HIP_LOCAL_Y[N_LEGS] = {
   -BODY_HALF_W,   /* 0 left-front  */
    BODY_HALF_W,   /* 1 right-front */
   -BODY_HALF_W,   /* 2 left-mid    */
    BODY_HALF_W,   /* 3 right-mid   */
   -BODY_HALF_W,   /* 4 left-rear   */
    BODY_HALF_W,   /* 5 right-rear  */
};

/*
 * Rest foot offsets from hip (body-space, body always faces +x).
 *   REST_FORWARD — along body axis; front legs reach forward, rear lean back.
 *   REST_SIDE    — lateral; left legs to −y (≤0), right legs to +y (≥0).
 *
 * Reach check: sqrt(32²+45²) ≈ 55 px < UPPER+LOWER=76 px  ✓
 */
static const float REST_FORWARD[N_LEGS] = {
    32.0f,   /* 0 left-front  */
    32.0f,   /* 1 right-front */
     0.0f,   /* 2 left-mid    */
     0.0f,   /* 3 right-mid   */
   -32.0f,   /* 4 left-rear   */
   -32.0f,   /* 5 right-rear  */
};
static const float REST_SIDE[N_LEGS] = {
   -45.0f,   /* 0 left-front  */
    45.0f,   /* 1 right-front */
   -50.0f,   /* 2 left-mid    */
    50.0f,   /* 3 right-mid   */
   -45.0f,   /* 4 left-rear   */
    45.0f,   /* 5 right-rear  */
};

/*
 * Tripod groups — two interlocked support triangles.
 *   A = {left-front(0), right-mid(3), left-rear(4)}
 *   B = {right-front(1), left-mid(2), right-rear(5)}
 * When A swings, B's three feet form a stable triangle (and vice versa).
 */
static const int TRIPOD_A[3] = {0, 3, 4};
static const int TRIPOD_B[3] = {1, 2, 5};

/* ── Hexapod struct ────────────────────────────────────────────────── */

typedef struct {
    /* body */
    float body_x, body_y;      /* center in pixel space                   */
    float body_speed;           /* px/s along heading                      */
    float heading;              /* current direction (rad, 0=right)        */
    float target_heading;       /* desired direction set by arrow keys     */

    /* per-leg */
    Vec2  foot_pos[N_LEGS];    /* current IK target (planted or lerping)  */
    Vec2  foot_old[N_LEGS];    /* foot at step-start (lerp anchor)        */
    Vec2  step_target[N_LEGS]; /* where this foot is stepping to          */
    bool  stepping[N_LEGS];    /* true while leg is swinging              */
    float step_t[N_LEGS];      /* swing progress 0→1                      */
    Vec2  hip[N_LEGS];         /* world hip (recomputed each tick)        */
    Vec2  knee[N_LEGS];        /* IK-computed mid-joint                   */

    /* snapshots for sub-tick alpha lerp */
    Vec2  prev_hip[N_LEGS];
    Vec2  prev_knee[N_LEGS];
    Vec2  prev_foot[N_LEGS];
    float prev_body_x, prev_body_y;
    float prev_heading;

    /* gait state */
    int   gait_phase;    /* 0 = group A steps, 1 = group B steps  */
    float phase_timer;   /* elapsed time in current half-cycle     */

    bool  paused;
    int   theme_idx;
} Hexapod;

/* ── §5a  solve_ik ─────────────────────────────────────────────────── */

/*
 * solve_ik() — 2-joint analytical IK via law of cosines.
 *
 * Left legs (even i): knee_angle = base − ah  → knee toward −y (top of screen)
 * Right legs (odd i): knee_angle = base + ah  → knee toward +y (bottom)
 *
 * This is OPPOSITE in sign to ik_spider.c because here the hips extend
 * perpendicular to the walk axis (±y), not along a sinusoidal heading.
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
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    float ah    = acosf(clampf(cos_h, -1.0f, 1.0f));

    float ka = is_left ? (base - ah) : (base + ah);

    knee_out->x = hip.x + UPPER_LEN * cosf(ka);
    knee_out->y = hip.y + UPPER_LEN * sinf(ka);
}

/* ── §5b  hip_world_pos ────────────────────────────────────────────── */

static Vec2 hip_world_pos(const Hexapod *h, int i)
{
    Vec2 local   = { HIP_LOCAL_X[i], HIP_LOCAL_Y[i] };
    Vec2 rotated = rotate2d(local, h->heading);
    return (Vec2){ h->body_x + rotated.x, h->body_y + rotated.y };
}

/* ── §5c  rest_target ──────────────────────────────────────────────── */

/*
 * rest_target() — ideal foot placement for leg i.
 * A lookahead offset (speed × STEP_LOOKAHEAD) plants the foot forward so
 * the leg is already reaching ahead when it lands.
 */
static Vec2 rest_target(const Hexapod *h, int i)
{
    Vec2 hip    = hip_world_pos(h, i);
    /* Rest offset is in body-local space; rotate to world by heading */
    Vec2 offset = { REST_FORWARD[i] + h->body_speed * STEP_LOOKAHEAD,
                    REST_SIDE[i] };
    Vec2 rotated = rotate2d(offset, h->heading);
    return (Vec2){ hip.x + rotated.x, hip.y + rotated.y };
}

/* ── §5d  gait_tick ────────────────────────────────────────────────── */

/*
 * gait_tick() — one tick of the tripod gait state machine.
 *
 * Advances swing animations for the current stepping group, then checks
 * whether the phase can transition:
 *   - phase_timer must exceed PHASE_DURATION
 *   - every leg in the stepping group must have landed
 * On transition: flip gait_phase, reset timer, launch the other group.
 */
static void gait_tick(Hexapod *h, float dt)
{
    const int *step_grp = (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B;

    /* Advance swing animations */
    for (int k = 0; k < 3; k++) {
        int i = step_grp[k];
        if (!h->stepping[i]) continue;

        h->step_t[i] += dt / STEP_DURATION;
        if (h->step_t[i] >= 1.0f) {
            h->step_t[i]   = 1.0f;
            h->foot_pos[i] = h->step_target[i];
            h->stepping[i] = false;
        } else {
            float ease = smoothstep(h->step_t[i]);
            Vec2  hz   = vec2_lerp(h->foot_old[i], h->step_target[i], ease);
            /* Parabolic arc: raw step_t (not eased) gives symmetric sin peak */
            float arc  = -STEP_HEIGHT * sinf((float)M_PI * h->step_t[i]);
            h->foot_pos[i] = (Vec2){ hz.x, hz.y + arc };
        }
    }

    h->phase_timer += dt;
    if (h->phase_timer < PHASE_DURATION) return;

    /* All stepping legs must be planted before we flip */
    for (int k = 0; k < 3; k++) {
        if (h->stepping[step_grp[k]]) return;
    }

    /* Transition: launch the other group */
    h->gait_phase  = (h->gait_phase + 1) % 2;
    h->phase_timer = 0.0f;

    const int *new_grp = (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B;
    for (int k = 0; k < 3; k++) {
        int i = new_grp[k];
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = rest_target(h, i);
        h->stepping[i]    = true;
        h->step_t[i]      = 0.0f;
    }
}

/* ── §5e  hexapod_tick ─────────────────────────────────────────────── */

static void hexapod_tick(Hexapod *h, float dt, int cols, int rows)
{
    /* Snapshot prev for alpha lerp */
    h->prev_body_x  = h->body_x;
    h->prev_body_y  = h->body_y;
    h->prev_heading = h->heading;
    memcpy(h->prev_hip,  h->hip,      sizeof h->hip);
    memcpy(h->prev_knee, h->knee,     sizeof h->knee);
    memcpy(h->prev_foot, h->foot_pos, sizeof h->foot_pos);

    if (h->paused) return;

    /*
     * Gradually steer heading toward target_heading.
     * Angular difference is normalised to [−π, π] so we always take
     * the short arc when target crosses ±π.
     */
    float diff = h->target_heading - h->heading;
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    float turn = clampf(diff, -TURN_RATE * dt, TURN_RATE * dt);
    h->heading += turn;

    /* Translate body along current heading */
    h->body_x += h->body_speed * cosf(h->heading) * dt;
    h->body_y += h->body_speed * sinf(h->heading) * dt;

    /* Toroidal wrap in all 4 directions (same as ik_spider.c) */
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (h->body_x <  0.0f) h->body_x += wpx;
    if (h->body_x >= wpx)  h->body_x -= wpx;
    if (h->body_y <  0.0f) h->body_y += hpx;
    if (h->body_y >= hpx)  h->body_y -= hpx;

    /* Recompute hips from new body position + heading */
    for (int i = 0; i < N_LEGS; i++) {
        h->hip[i] = hip_world_pos(h, i);
    }

    /*
     * Stretch-snap: after a toroidal wrap or a sharp turn the hip may
     * teleport away from the planted foot.  If the hip-to-foot distance
     * exceeds IK reach, snap the foot to its rest position immediately
     * (no step animation) so the IK solver is never given an impossible target.
     */
    for (int i = 0; i < N_LEGS; i++) {
        float dx = h->foot_pos[i].x - h->hip[i].x;
        float dy = h->foot_pos[i].y - h->hip[i].y;
        if (sqrtf(dx*dx + dy*dy) > UPPER_LEN + LOWER_LEN - 2.0f) {
            h->foot_pos[i]    = rest_target(h, i);
            h->foot_old[i]    = h->foot_pos[i];
            h->step_target[i] = h->foot_pos[i];
            h->stepping[i]    = false;
            h->step_t[i]      = 0.0f;
        }
    }

    /* Gait state machine */
    gait_tick(h, dt);

    /* Solve IK for all legs */
    for (int i = 0; i < N_LEGS; i++) {
        solve_ik(h->hip[i], h->foot_pos[i], (i % 2 == 0), &h->knee[i]);
    }
}

/* ── direction-char line renderer (verbatim from ik_spider.c §5h) ─── */

static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg <  22.5f || deg >= 157.5f) return (chtype)'-';
    if (deg <  67.5f)                   return (chtype)'\\';
    if (deg < 112.5f)                   return (chtype)'|';
    return                              (chtype)'/';
}

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

/* ── §5f  hexapod_draw ─────────────────────────────────────────────── */

/*
 * Draw order (back to front so joints appear on top of line chars):
 *   1. Leg direction-char lines  (femur pair 2, tibia pair 3)
 *   2. Foot markers              ('*' planted pair 4, 'o' stepping pair 5)
 *   3. Knee markers              ('o' pair 6)
 *   4. Body rectangle            (4 edges + 2 cross-braces, pair 1)
 *   5. Hip attachment markers    ('+' pair 1)
 *   6. Body center               ('@' pair 1)
 */
static void hexapod_draw(const Hexapod *h, WINDOW *w,
                         int cols, int rows, float alpha)
{
    /* Alpha-interpolated positions and heading */
    float bx = h->prev_body_x + (h->body_x - h->prev_body_x) * alpha;
    float by = h->prev_body_y + (h->body_y - h->prev_body_y) * alpha;

    /* Short-arc lerp for heading */
    float hdiff = h->heading - h->prev_heading;
    while (hdiff >  (float)M_PI) hdiff -= 2.0f * (float)M_PI;
    while (hdiff < -(float)M_PI) hdiff += 2.0f * (float)M_PI;
    float bh = h->prev_heading + hdiff * alpha;

    Vec2 r_hip[N_LEGS], r_knee[N_LEGS], r_foot[N_LEGS];
    for (int i = 0; i < N_LEGS; i++) {
        r_hip[i]  = vec2_lerp(h->prev_hip[i],  h->hip[i],      alpha);
        r_knee[i] = vec2_lerp(h->prev_knee[i], h->knee[i],     alpha);
        r_foot[i] = vec2_lerp(h->prev_foot[i], h->foot_pos[i], alpha);
    }

    /* 1. Leg lines */
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, r_hip[i],  r_knee[i], 2, A_NORMAL, cols, rows);
        draw_leg_line(w, r_knee[i], r_foot[i], 3, A_NORMAL, cols, rows);
    }

    /* 2. Foot markers */
    for (int i = 0; i < N_LEGS; i++) {
        int fx = px_to_cell_x(r_foot[i].x);
        int fy = px_to_cell_y(r_foot[i].y);
        if (fx < 0 || fx >= cols || fy < 0 || fy >= rows) continue;
        if (h->stepping[i]) {
            wattron(w, COLOR_PAIR(5) | A_DIM);
            mvwaddch(w, fy, fx, (chtype)'o');
            wattroff(w, COLOR_PAIR(5) | A_DIM);
        } else {
            wattron(w, COLOR_PAIR(4) | A_BOLD);
            mvwaddch(w, fy, fx, (chtype)'*');
            wattroff(w, COLOR_PAIR(4) | A_BOLD);
        }
    }

    /* 3. Knee markers */
    for (int i = 0; i < N_LEGS; i++) {
        int kx = px_to_cell_x(r_knee[i].x);
        int ky = px_to_cell_y(r_knee[i].y);
        if (kx < 0 || kx >= cols || ky < 0 || ky >= rows) continue;
        wattron(w, COLOR_PAIR(6) | A_BOLD);
        mvwaddch(w, ky, kx, (chtype)'o');
        wattroff(w, COLOR_PAIR(6) | A_BOLD);
    }

    /* 4. Body rectangle — corners rotated by interpolated heading */
    float bhl = BODY_LEN  * 0.5f;
    float bhw = BODY_HALF_W;
    Vec2 tl = { bx + rotate2d((Vec2){ -bhl, -bhw }, bh).x,
                by + rotate2d((Vec2){ -bhl, -bhw }, bh).y };
    Vec2 tr = { bx + rotate2d((Vec2){  bhl, -bhw }, bh).x,
                by + rotate2d((Vec2){  bhl, -bhw }, bh).y };
    Vec2 bl = { bx + rotate2d((Vec2){ -bhl,  bhw }, bh).x,
                by + rotate2d((Vec2){ -bhl,  bhw }, bh).y };
    Vec2 br = { bx + rotate2d((Vec2){  bhl,  bhw }, bh).x,
                by + rotate2d((Vec2){  bhl,  bhw }, bh).y };

    draw_leg_line(w, tl, tr, 1, A_BOLD, cols, rows);
    draw_leg_line(w, bl, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, bl, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tr, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, br, 1, A_DIM,  cols, rows);   /* cross-brace */
    draw_leg_line(w, tr, bl, 1, A_DIM,  cols, rows);   /* cross-brace */

    /* 5. Hip attachment markers */
    for (int i = 0; i < N_LEGS; i++) {
        int hx2 = px_to_cell_x(r_hip[i].x);
        int hy2 = px_to_cell_y(r_hip[i].y);
        if (hx2 < 0 || hx2 >= cols || hy2 < 0 || hy2 >= rows) continue;
        wattron(w, COLOR_PAIR(1) | A_NORMAL);
        mvwaddch(w, hy2, hx2, (chtype)'+');
        wattroff(w, COLOR_PAIR(1) | A_NORMAL);
    }

    /* 6. Body center */
    int cx = px_to_cell_x(bx);
    int cy = px_to_cell_y(by);
    if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
        wattron(w, COLOR_PAIR(1) | A_BOLD);
        mvwaddch(w, cy, cx, (chtype)'@');
        wattroff(w, COLOR_PAIR(1) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Hexapod hexapod; } Scene;

/*
 * scene_init() — place robot at screen center with all feet at rest.
 * Group A is immediately launched stepping so the gait begins from tick 1.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Hexapod *h = &sc->hexapod;

    h->body_speed      = BODY_SPEED_DEFAULT;
    h->body_x          = (float)(cols * CELL_W) * 0.50f;
    h->body_y          = (float)(rows * CELL_H) * 0.50f;
    h->heading         = 0.0f;   /* start facing right */
    h->target_heading  = 0.0f;
    h->prev_heading    = 0.0f;
    h->gait_phase      = 0;
    h->phase_timer     = 0.0f;
    h->paused          = false;
    h->theme_idx       = 0;

    /* Compute initial hips */
    for (int i = 0; i < N_LEGS; i++) {
        h->hip[i] = hip_world_pos(h, i);
    }

    /* Plant all feet at rest positions (no lookahead for initial static pose) */
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

    /* Launch group A immediately so the gait starts at frame 1 */
    for (int k = 0; k < 3; k++) {
        int i = TRIPOD_A[k];
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = rest_target(h, i);   /* includes lookahead */
        h->stepping[i]    = true;
        h->step_t[i]      = 0.0f;
    }

    /* Snapshot prev (no-op on first frame) */
    h->prev_body_x = h->body_x;
    h->prev_body_y = h->body_y;
    memcpy(h->prev_hip,  h->hip,      sizeof h->hip);
    memcpy(h->prev_knee, h->knee,     sizeof h->knee);
    memcpy(h->prev_foot, h->foot_pos, sizeof h->foot_pos);
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    hexapod_tick(&sc->hexapod, dt, cols, rows);
}

static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    hexapod_draw(&sc->hexapod, w, cols, rows, alpha);
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

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Hexapod *h = &sc->hexapod;
    char buf[HUD_COLS + 1];
    /* Direction arrow for current heading */
    float deg = h->heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    const char *dir_arrow =
        (deg < 45.0f || deg >= 315.0f) ? ">" :
        (deg < 135.0f)                  ? "v" :
        (deg < 225.0f)                  ? "<" : "^";

    snprintf(buf, sizeof buf,
             " HEXAPOD  dir:%s  spd:%.0f  phase:%s  [%s]  %s  %.1ffps  %dHz ",
             dir_arrow, h->body_speed,
             h->gait_phase == 0 ? "A" : "B",
             THEMES[h->theme_idx].name,
             h->paused ? "PAUSED" : "walking",
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
    Hexapod *h   = &app->scene.hexapod;
    float    wpx = (float)(app->screen.cols * CELL_W);
    float    hpx = (float)(app->screen.rows * CELL_H);
    if (h->body_x >= wpx) h->body_x = wpx * 0.5f;
    if (h->body_y >= hpx || h->body_y < 0.0f) h->body_y = hpx * 0.5f;
    app->need_resize = 0;
    theme_apply(h->theme_idx);
}

static bool app_handle_key(App *app, int ch)
{
    Hexapod *h = &app->scene.hexapod;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': h->paused = !h->paused; break;

    /* Arrow keys steer (set target_heading; body turns gradually) */
    case KEY_RIGHT: h->target_heading =  0.0f;                  break;
    case KEY_DOWN:  h->target_heading =  (float)M_PI * 0.5f;   break;
    case KEY_LEFT:  h->target_heading =  (float)M_PI;           break;
    case KEY_UP:    h->target_heading = -(float)M_PI * 0.5f;   break;

    case 'w':
        h->body_speed *= 1.25f;
        if (h->body_speed > BODY_SPEED_MAX) h->body_speed = BODY_SPEED_MAX;
        break;
    case 's':
        h->body_speed /= 1.25f;
        if (h->body_speed < BODY_SPEED_MIN) h->body_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        h->theme_idx = (h->theme_idx + 1) % N_THEMES;
        theme_apply(h->theme_idx);
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

/*
 * main() — fixed-step accumulator game loop (identical to ik_spider.c §8)
 *
 * ① resize check    — handle SIGWINCH before touching ncurses
 * ② measure dt      — wall-clock since last frame, capped at 100 ms
 * ③ accumulator     — fire scene_tick() at sim_fps Hz
 * ④ alpha           — sub-tick interpolation for smooth render
 * ⑤ fps counter     — smoothed over 500 ms windows
 * ⑥ frame cap       — sleep to cap render at ~60 fps
 * ⑦ draw + present  — erase → scene_draw → HUD → doupdate
 * ⑧ drain input     — consume all queued keypresses
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

        /* ── ⑥ frame cap ─────────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ─────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain input ────────────────────────────────────────── */
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
