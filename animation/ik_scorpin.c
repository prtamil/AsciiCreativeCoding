/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_scorpin.c — six-legged scorpion with curving FK tail
 *
 * DEMO: A scorpion crawls across the terminal under autonomous steering.
 *       Body uses trail-buffer FK; six legs use 2-joint analytical IK
 *       with a per-leg autonomous gait. The signature feature: a
 *       seven-segment tail anchored at the abdomen curves up and over
 *       the body in the iconic scorpion silhouette, with a sin-wave
 *       perturbation traveling along its length so the stinger visibly
 *       sways as the scorpion moves.
 *
 * Study alongside: ik_spider.c       (same body + leg model, no tail)
 *                  hexpod_tripod.c   (rigid-body chassis contrast)
 *
 * Section map:
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10 themes + spec HUD/hint pairs
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Scorpion: body FK + IK legs + curving FK tail
 *       §5a  vec2 + small helpers
 *       §5b  trail (push / at / sample)
 *       §5c  body motion (steer + translate + wrap)
 *       §5d  body joints (trail-buffer FK)
 *       §5e  hip placement
 *       §5f  2-joint analytical IK
 *       §5g  step gait
 *       §5h  tail FK (cumulative angle + sin wave)
 *       §5i  rendering helpers
 *       §5j  render_scorpion (orchestrator)
 *   §6  scene         — thin Scene wrapper
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 *
 * Keys:  q / ESC     quit                 space   pause / resume
 *        ↑ ↓ ← →     steer in 4 directions
 *        w / s       speed × / ÷ 1.25
 *        t           cycle theme          [ / ]   time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_scorpin.c \
 *       -o ik_scorpin -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Four sub-systems share one body.
 *
 *                 BODY uses path-following (trail-buffer) FK. Every
 *                 tick the head pushes its position into a circular
 *                 trail; each body joint is placed at arc-length
 *                 i*BODY_SEG_LEN backward along the trail. The body
 *                 curves naturally wherever the head went.
 *
 *                 LEGS use 2-joint analytical IK via law of cosines:
 *                     cos(θ_hip) = (d² + U² − L²) / (2·d·U)
 *                 with the hip→target distance d clamped into the
 *                 reachable annulus. Left and right legs use opposite
 *                 signs of θ_hip so knees splay outward.
 *
 *                 GAIT is per-leg autonomous: each leg watches its own
 *                 foot's drift from the ideal step target and triggers
 *                 a swing when ready, gated by an "n_air < N_LEGS/2"
 *                 stability cap so ≥3 feet are always planted.
 *
 *                 TAIL is a stateless FK chain anchored at the abdomen.
 *                 Cumulative angle starts at (heading + π) and adds
 *                 TAIL_BASE_CURL rad per segment so the tail sweeps up
 *                 and over the body in the classic scorpion silhouette.
 *                 A small sin perturbation per segment, with phase
 *                 offset i*TAIL_PHASE_PER_SEG, makes a wave roll along
 *                 the tail — the stinger visibly sways as wave_time
 *                 advances. No iteration, no contact constraints.
 *
 * Data-structure: Scorpion holds the trail buffer + body joints,
 *                 per-leg state, heading + steering, AND a tail joint
 *                 array `tail[N_TAIL_SEGS+1]` plus a `wave_time`
 *                 accumulator that drives the tail oscillation.
 *
 * Rendering     : Painter's order — leg lines → leg joints → body
 *                 fill → body markers → tail (with stinger) → head
 *                 cluster. The tail is drawn AFTER the body so where
 *                 the curve passes over the body, the tail wins. The
 *                 head cluster is drawn last so it always overlays.
 *
 * Performance   : Variable timestep at render rate. Per frame: body
 *                 trail push + 4 trail samples + 6 IK solves + 7 tail
 *                 FK iterations. Microseconds total.
 *
 * References    :
 *   Reynolds, "Steering Behaviors for Autonomous Characters" (1999) —
 *     framework for the heading-toward-target interpolation pattern.
 *     https://www.red3d.com/cwr/steer/
 *   Wikipedia, "Inverse kinematics" — derives the 2-joint
 *     law-of-cosines solver used in solve_ik().
 *   Wikipedia, "Scorpion" — anatomy reference for the curving tail
 *     geometry (cauda raised over the back).
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) — case
 *     for fixed-step (stiff sims); we don't qualify, hence variable.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The scorpion is a snake-like body with six pendulum legs and one
 * curving tail-pendulum. Body, legs, and gait are identical to
 * ik_spider.c — the tail is the new piece. The tail is just a chain
 * of stiff segments hinged end-to-end, anchored at the abdomen and
 * with each segment's angle = (previous angle) + (base curl + sin
 * wave). Cumulative angle accumulates the curl, so 7 segments × 0.34
 * rad each = 2.4 rad ≈ 136° total arch — the iconic scorpion shape.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Body : same trail-buffer chain as the spider — curves through space
 *        as the head walks.
 *
 * Legs : six 2-bar linkages, IK-solved, autonomous gait.
 *
 * Tail : one extra chain hinged at the abdomen. Imagine a snake
 *        attached to the back end of a flat lizard, but with each
 *        joint curling the same direction at the same rate — the
 *        whole snake naturally arcs over the lizard's back. Now add
 *        a slow sine wave to the angles and the snake breathes.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. wave_time += dt — single accumulator drives the tail wave.
 *  3. Steer: heading interpolates toward target_heading at TURN_RATE.
 *  4. Translate body, push trail, sample body joints.
 *  5. Compute hips along the body; run gait + IK for legs.
 *  6. Compute tail: starting at body_joint[N_BODY_SEGS] and angle
 *     (heading + π), accumulate angle += TAIL_BASE_CURL +
 *     TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ + i · TAIL_PHASE_PER_SEG)
 *     and place tail[i+1] from the new angle.
 *  7. Render painter's order: legs → body → tail → head.
 *
 * KEY FORMULAS
 * ────────────
 *  Tail FK     : ang_0   = heading + π        (rearward from abdomen)
 *                δᵢ      = TAIL_BASE_CURL
 *                        + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ
 *                                              + i · TAIL_PHASE_PER_SEG)
 *                ang_i+1 = ang_i + δᵢ
 *                tail_i+1 = tail_i + TAIL_SEG_LEN · (cos ang_i+1, sin ang_i+1)
 *
 *  Total curl  : N_TAIL_SEGS · TAIL_BASE_CURL ≈ 2.4 rad ≈ 136°
 *                stinger ends pointing forward-up (over the body).
 *
 *  All other formulas (trail_sample, hip placement, IK, ideal foot,
 *  step swing) are unchanged from ik_spider.c.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Tail base anchor moves with the body. Because body_joint[N] comes
 *    from trail_sample, the tail base curves naturally with the body.
 *    No extra work — the tail "just follows" the abdomen.
 *
 *  - Tail tip can poke off-screen. Body wraps toroidally; the tail
 *    extends from the wrapped abdomen and may extend past edges. The
 *    line-drawer clips out-of-bounds cells silently.
 *
 *  - Wave amplitude vs base curl. TAIL_SWAY_AMP must stay well below
 *    TAIL_BASE_CURL so the tail never reverses direction at any
 *    segment (which would cause the chain to fold back on itself).
 *    0.06 << 0.34 — safe by 5×.
 *
 *  - IK clamp / wrap recovery: same as the spider — over-stretched
 *    feet snap to ideal positions on the next tick.
 *
 *  - Suspend / lid-close: dt clamped to 100 ms in main().
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: scorpion crawls rightward at 45 px/s. The tail
 *    visibly arches up and over the body, ending in a '#' stinger
 *    that points forward-and-up. The stinger sways slowly side-to-side
 *    in a ~6 s cycle.
 *
 *  - Press arrows → body curves through the turn AND the tail follows,
 *    re-arching from the new body heading.
 *
 *  - Press space → tail freezes mid-curve. Un-pause → wave continues
 *    from the same phase (wave_time stops advancing during pause).
 *
 *  - Press `[` for slow time → tail wave slows visibly. Press `]` for
 *    fast — wave speeds up. Smooth at any time scale.
 *
 *  - Cycle themes with `t` → all parts (body / legs / tail / stinger)
 *    update colour together; HUD stays bright yellow.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* All magic numbers live here. Never scatter literals through the code. */
enum {
    /* Render frame-rate target. Variable-timestep simulation, so the
     * only thing this controls is the sleep cap at end of frame. */
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
 * Tail geometry and oscillation parameters.
 *
 *   TAIL_SEG_LEN      = 11 px per segment → 7×11 = 77 px ≈ 10 cells of tail.
 *
 *   TAIL_BASE_CURL    = 0.34 rad/seg. Cumulative: 7 × 0.34 = 2.38 rad ≈ 136°
 *                       total curl. Tail starts pointing rearward (heading
 *                       + π) and after the cumulative curl ends pointing
 *                       forward-and-up (over the body) — the iconic
 *                       scorpion silhouette.
 *
 *   TAIL_SWAY_AMP     = 0.06 rad — small per-segment perturbation. Must
 *                       stay well below TAIL_BASE_CURL so the chain never
 *                       folds back on itself (0.06 << 0.34, safe by 5×).
 *
 *   TAIL_FREQ         = 1.0 rad/s → period 2π/1.0 ≈ 6.3 s — slow elegant
 *                       sway. The stinger swings ±~10° over each cycle.
 *
 *   TAIL_PHASE_PER_SEG= 0.5 rad — wave traveling rate along the tail.
 *                       7 × 0.5 = 3.5 rad ≈ 0.56 of a full cycle, so a
 *                       half-wave is visible rolling from base to tip.
 */
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

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions (aspect-ratio bridge). */
#define CELL_W   8
#define CELL_H  16

/*
 * LEG_ANGLE[i] — angle (radians) added to body forward to give leg i's
 * ideal step direction in body-local space. Three pairs at 60° spacing.
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
 * HIP_BODY_T[i] — parametric position along the body where each hip
 * attaches (0 = head joint, 1 = tail joint).
 */
static const float HIP_BODY_T[N_LEGS] = {
    0.20f, 0.20f,   /* front pair — near the head */
    0.50f, 0.50f,   /* mid pair   — body centre   */
    0.80f, 0.80f,   /* rear pair  — near the tail */
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color — 10 themes + spec-fixed HUD pairs                          */
/* ===================================================================== */

/*
 * Per-theme palette. Pairs 1..7 set by the theme; HUD/HINT pairs are
 * theme-independent (CLAUDE.md HUD spec).
 *
 * Pair semantics:
 *   col[0..2] body gradient (tail → head)  — col[2] is the main scorpion colour
 *   col[3..4] leg segments (upper / lower)
 *   col[5]    tail + planted feet '*' + stinger '#'  (bright accent)
 *   col[6]    swinging foot '.'                       (dim trailing accent)
 *
 * All entries sit in the bright half of the 256-colour space:
 *   - cube colours: ≥ 24 (brightness rule)
 *   - grayscale:    ≥ 240 (the 232-239 zone vanishes under A_DIM)
 */
typedef struct {
    const char *name;
    int col[N_PAIRS];   /* pairs 1..7 */
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

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio helpers                          */
/* ===================================================================== */

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Scorpion                                                  */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention). */
typedef struct { float x, y; } Vec2;

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

typedef struct {
    /* body — trail-buffer FK */
    Vec2  trail[TRAIL_CAP];
    int   trail_head, trail_count;
    Vec2  body_joint[N_BODY_SEGS + 1];     /* [0]=head, [N]=tail joint    */

    /* body kinematics */
    float heading;
    float target_heading;
    float move_speed;

    /* per-leg state */
    Vec2  hip[N_LEGS];
    Vec2  knee[N_LEGS];
    Vec2  foot_pos[N_LEGS];
    Vec2  foot_old[N_LEGS];
    Vec2  step_target[N_LEGS];
    bool  stepping[N_LEGS];
    float step_t[N_LEGS];

    /* tail — stateless FK chain anchored at the abdomen */
    Vec2  tail[N_TAIL_SEGS + 1];
    float wave_time;          /* phase accumulator for tail oscillation */

    /* derived */
    float hip_dist;

    /* ui state */
    bool  paused;
    int   theme_idx;
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

static void steer_heading(Scorpion *sc, float dt)
{
    float diff = sc->target_heading - sc->heading;
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    sc->heading += clampf(diff, -TURN_RATE * dt, TURN_RATE * dt);
}

static void translate_body(Scorpion *sc, float dt, int cols, int rows)
{
    sc->body_joint[0].x += sc->move_speed * cosf(sc->heading) * dt;
    sc->body_joint[0].y += sc->move_speed * sinf(sc->heading) * dt;

    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (sc->body_joint[0].x <  0.0f) sc->body_joint[0].x += wpx;
    if (sc->body_joint[0].x >= wpx)  sc->body_joint[0].x -= wpx;
    if (sc->body_joint[0].y <  0.0f) sc->body_joint[0].y += hpx;
    if (sc->body_joint[0].y >= hpx)  sc->body_joint[0].y -= hpx;

    trail_push(sc, sc->body_joint[0]);
}

/* ── §5d  body joints (trail-buffer FK) ──────────────────────────── */

static void compute_body_joints(Scorpion *sc)
{
    for (int i = 1; i <= N_BODY_SEGS; i++)
        sc->body_joint[i] = trail_sample(sc, (float)i * BODY_SEG_LEN);
}

/* ── §5e  hip placement ──────────────────────────────────────────── */

static Vec2 body_local_forward(const Scorpion *sc, int seg_idx)
{
    if (seg_idx + 1 > N_BODY_SEGS)
        return (Vec2){ cosf(sc->heading), sinf(sc->heading) };
    return vec2_norm(vec2_sub(sc->body_joint[seg_idx],
                              sc->body_joint[seg_idx + 1]));
}

static void compute_hips(Scorpion *sc)
{
    for (int i = 0; i < N_LEGS; i++) {
        float t_body  = HIP_BODY_T[i] * (float)N_BODY_SEGS;
        int   seg_idx = (int)t_body;
        if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
        float frac    = t_body - (float)seg_idx;
        Vec2  attach  = vec2_lerp(sc->body_joint[seg_idx],
                                  sc->body_joint[seg_idx + 1], frac);

        Vec2  fwd       = body_local_forward(sc, seg_idx);
        Vec2  left_norm = (Vec2){ -fwd.y, fwd.x };
        float side      = (i % 2 == 0) ? 1.0f : -1.0f;

        sc->hip[i] = vec2_add(attach,
                              vec2_scale(left_norm, side * sc->hip_dist));
    }
}

/* ── §5f  2-joint analytical IK ──────────────────────────────────── */

static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = sqrtf(dx * dx + dy * dy);

    dist = clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN - 1.0f);

    float base    = atan2f(dy, dx);
    float cos_h   = (dist * dist + UPPER_LEN * UPPER_LEN
                                  - LOWER_LEN * LOWER_LEN)
                    / (2.0f * dist * UPPER_LEN);
    float ah      = acosf(clampf(cos_h, -1.0f, 1.0f));
    float ka      = is_left ? (base + ah) : (base - ah);

    knee_out->x = hip.x + UPPER_LEN * cosf(ka);
    knee_out->y = hip.y + UPPER_LEN * sinf(ka);
}

/* ── §5g  step gait ──────────────────────────────────────────────── */

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

static void update_steps(Scorpion *sc, float dt)
{
    int n_air = count_airborne(sc);

    for (int i = 0; i < N_LEGS; i++) {
        if (snap_overstretched_foot(sc, i)) n_air--;
        else if (sc->stepping[i]) {
            if (advance_swing(sc, i, dt))   n_air--;
        }
        else {
            if (maybe_trigger_step(sc, i, n_air)) n_air++;
        }
    }

    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sc->hip[i], sc->foot_pos[i], (i % 2 == 0), &sc->knee[i]);
}

/* ── §5h  tail FK ────────────────────────────────────────────────── */

/*
 * compute_tail — stateless FK chain for the curving tail.
 *
 * Tail is anchored at body_joint[N_BODY_SEGS] (the abdomen-end joint).
 * Cumulative angle starts at (heading + π) so the tail initially points
 * REARWARD from the abdomen. Each segment then adds:
 *
 *     δᵢ = TAIL_BASE_CURL + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ
 *                                                + i · TAIL_PHASE_PER_SEG)
 *
 * The cumulative effect of N_TAIL_SEGS · TAIL_BASE_CURL ≈ 2.4 rad ≈ 136°
 * sweeps the tail UP and OVER the body, ending with the stinger pointing
 * forward-and-up — the iconic scorpion silhouette.
 *
 * The sin perturbation makes a wave roll along the tail (phase offset
 * i · TAIL_PHASE_PER_SEG per segment), so the stinger visibly sways
 * over the wave_time period (~6.3 s at default).
 *
 * No iteration, no state across frames except wave_time. The tail is
 * a closed-form function of (body pose, wave_time).
 */
static void compute_tail(Scorpion *sc)
{
    sc->tail[0] = sc->body_joint[N_BODY_SEGS];   /* anchor at abdomen */

    float angle = sc->heading + (float)M_PI;     /* point rearward */

    for (int i = 0; i < N_TAIL_SEGS; i++) {
        float wave  = TAIL_SWAY_AMP * sinf(sc->wave_time * TAIL_FREQ
                                           + (float)i * TAIL_PHASE_PER_SEG);
        angle += TAIL_BASE_CURL + wave;

        sc->tail[i + 1].x = sc->tail[i].x + TAIL_SEG_LEN * cosf(angle);
        sc->tail[i + 1].y = sc->tail[i].y + TAIL_SEG_LEN * sinf(angle);
    }
}

/* ── §5i  rendering helpers ──────────────────────────────────────── */

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

/*
 * draw_chain_line — segmented chain line: alternating direction-glyph
 * and '.' so each segment reads as a chained limb. Used for legs AND
 * for the tail — both are articulated chains in this creature.
 */
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

/* ── §5j  render_scorpion ────────────────────────────────────────── */

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

/*
 * draw_tail — render the curving tail with stinger.
 * Same chained-line style as the legs (direction-glyph + '.'), drawn
 * in the brighter accent pair (6) so it visually pops over the body.
 * Stinger '#' at the tip — chunky barb, A_BOLD for emphasis.
 */
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

/*
 * draw_head — eye cluster ': arrow :' at the head joint, perpendicular
 * to heading. Same as ik_spider — a small visual signature so the
 * scorpion's facing direction is unambiguous.
 */
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

/*
 * render_scorpion — orchestrator. Painter's order back-to-front:
 *   legs (lines)  →  leg joints  →  body fill  →  body nodes
 *   tail (curves over body)  →  head cluster
 *
 * Tail is drawn AFTER body nodes so where it crosses over the body
 * (which it does at the tail's apex), the tail wins. Head is drawn
 * last so it always overlays everything else.
 */
static void render_scorpion(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    draw_legs       (sc, w, cols, rows);
    draw_leg_joints (sc, w, cols, rows);
    draw_body_lines (sc, w, cols, rows);
    draw_body_nodes (sc, w, cols, rows);
    draw_tail       (sc, w, cols, rows);
    draw_head       (sc, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Scorpion                              */
/* ===================================================================== */

typedef struct { Scorpion scorpion; } Scene;

/*
 * scene_init — place scorpion at screen centre with a pre-populated
 * trail (so the body is fully extended on frame 1) and feet planted
 * at their ideal rest positions.
 */
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

/*
 * scene_tick — one variable-timestep simulation step. dt is wall-clock
 * delta scaled by the caller's time_scale.
 *
 * Order: wave_time, steer, translate, body joints, hips, gait+IK, tail.
 * Tail comes LAST so it sees the new abdomen position from this tick's
 * body update.
 */
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

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    float                 time_scale;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

static void cleanup(void) { endwin(); }

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

        /* ⑦ frame cap */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
