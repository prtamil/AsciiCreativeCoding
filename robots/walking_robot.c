/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * walking_robot.c — Walking Stick Robot with Procedural Walk Cycle
 *
 * DEMO: A bipedal humanoid stick figure walks across the terminal driven
 *       by a sinusoidal procedural gait.  Swing legs use FK with a
 *       −cos-shaped thigh sweep and sinusoidal knee lift; stance legs use
 *       2-joint analytical IK from the locked ankle.  Foot contact locking
 *       keeps planted feet stationary while the body advances.  The torso
 *       bobs twice per stride and sways laterally over the stance leg.
 *       A scrolling ground grid, drop-shadow ellipse, center-of-mass
 *       projection line, head motion trail, and foot contact markers
 *       complete the visual.
 *
 * STUDY ALONGSIDE: hexpod_tripod.c  (2-joint IK + gait phases)
 *                  ragdoll_figure.c (framework + bone rendering)
 *                  fk_centipede.c   (sinusoidal FK legs)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config         — all tunables in one place
 *   §2  clock          — monotonic clock + sleep
 *   §3  color          — robot-themed palette + HUD pair
 *   §4  coords         — pixel↔cell helpers + bone_glyph
 *   §5  robot
 *       §5a  solve_ik      — 2-joint analytical IK (law of cosines)
 *       §5b  Robot struct
 *       §5c  compute_pose  — FK swing + IK stance → all joint positions
 *       §5d  robot_init    — initial pose + foot-lock seed
 *       §5e  robot_tick    — phase advance, foot lock, trail push
 *       §5f  draw helpers  — draw_bone, draw_ground, draw_shadow,
 *                            draw_trail, draw_com
 *       §5g  robot_draw    — full frame composition
 *   §6  scene          — scene_init / scene_tick / scene_draw
 *   §7  screen         — ncurses double-buffer display layer
 *   §8  app            — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * WALK CYCLE MECHANICS
 * ─────────────────────
 * A single walk phase φ advances each tick: φ += 2π·WALK_FREQ·dir·dt.
 * Legs are π out of phase:
 *   Left  leg: φ_L = φ          Right leg: φ_R = φ + π
 *
 * SWING (sin(φ_i) > 0, half-cycle 0 → π):
 *   Thigh angle (from vertical, + = forward/+x):
 *     θ_thigh = −THIGH_SWING · cos(φ_i)
 *     → −SWING at start (leg rear), +SWING at end (leg front)
 *   Knee lift:
 *     θ_shin = θ_thigh + KNEE_LIFT · sin(φ_i)   ← peak at mid-swing
 *   Joint positions via FK:
 *     knee = hip + U·(sin θ_t, cos θ_t)
 *     foot = knee + L·(sin θ_s, cos θ_s)
 *   At stance entry (sin → −), foot position is locked to ground.
 *
 * STANCE (sin(φ_i) ≤ 0, half-cycle π → 2π):
 *   foot = foot_lock[i]  (constant world position)
 *   knee = solve_ik(hip, foot, UPPER_LEG_LEN, LOWER_LEG_LEN)
 *
 * BODY:
 *   Bob:  hip_y  = base_hip_y  + BOB_AMP · sin(2φ)
 *   Sway: torso_x = hip_cx  + SWAY_AMP · cos(φ)
 *     (leans right when left leg swings, left when right leg swings)
 *
 * ARMS (counter-swing with contralateral leg):
 *   Arm_L: φ_aL = φ + π,   Arm_R: φ_aR = φ
 *   θ_upper = ARM_SWING · sin(φ_ai)
 *   θ_lower = θ_upper + ARM_BEND + 0.3·ARM_SWING · sin(φ_ai − 0.5)
 *
 * FOOT CONTACT LOCKING:
 *   When sin(φ_i) transitions from + to −, the foot is entering stance.
 *   We compute the FK landing position at that exact transition phase
 *   (thigh at max forward, knee straight) and lock foot_lock[i] to it.
 *   During stance the IK reconstructs the knee from hip → locked foot.
 *
 * CENTER OF MASS:
 *   com_x = (head.x + 2·torso_top.x + 2·hip_c.x) / 5
 *   Projected as a dotted vertical line to the ground with a '^' marker.
 *
 * Keys: q/ESC=quit  SPACE=pause  r=reverse  +/-=speed  .=step  g=grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       robots/walking_robot.c -o walking_robot -lncurses -lm
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
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    HUD_COLS        =  96,
    FPS_UPDATE_MS   = 500,

    N_PAIRS         =   8,
    HUD_PAIR        =   8,

    TRAIL_LEN       =  28,    /* head-position ring-buffer depth   */
    GRID_PERIOD     =   6,    /* columns between scrolling ticks   */
};

/* Terminal cell dimensions in pixels */
#define CELL_W  8
#define CELL_H  16

/* Draw step for bone scan-conversion */
#define DRAW_STEP_PX  2.0f

/* ── Robot proportions (pixel space) ─────────────────────────────────── */
#define HEAD_R          11.0f
#define NECK_LEN         8.0f
#define TORSO_LEN       52.0f
#define SHOULDER_W      18.0f
#define HIP_W           11.0f
#define UPPER_ARM_LEN   26.0f
#define LOWER_ARM_LEN   20.0f
#define ARM_BEND         0.28f   /* resting elbow forward-bend (rad)     */
#define ARM_SWING        0.45f   /* upper-arm swing amplitude (rad)      */
#define UPPER_LEG_LEN   38.0f
#define LOWER_LEG_LEN   32.0f
#define THIGH_SWING      0.52f   /* max thigh excursion (rad)            */
#define KNEE_LIFT        0.65f   /* extra knee bend at mid-swing (rad)   */

/* ── Gait dynamics ───────────────────────────────────────────────────── */
#define WALK_FREQ_DEFAULT  1.8f
#define WALK_FREQ_MIN      0.4f
#define WALK_FREQ_MAX      6.0f
#define WALK_FREQ_STEP     0.2f

#define WALK_SPEED_DEFAULT  55.0f   /* px/s */
#define WALK_SPEED_MIN      12.0f
#define WALK_SPEED_MAX     220.0f
#define WALK_SPEED_STEP     12.0f

/* ── Body oscillation ────────────────────────────────────────────────── */
#define BOB_AMP   4.0f    /* px vertical bob (twice per stride)          */
#define SWAY_AMP  5.5f    /* px lateral sway (once per stride)           */

/* ── Shadow ──────────────────────────────────────────────────────────── */
#define SHADOW_SEMI_X  22.0f   /* px half-width of drop-shadow ellipse   */

/* ── Timing ──────────────────────────────────────────────────────────── */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Robot palette:
 *   1  near-white  253  — head
 *   2  cyan         51  — neck / spine / collar
 *   3  steel-blue   75  — upper arms
 *   4  orange      214  — lower arms / hands
 *   5  chartreuse  154  — upper legs / thighs
 *   6  green        46  — lower legs / feet
 *   7  dark-grey   238  — ground, grid, shadow, COM
 *   8  yellow      226  — HUD
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(1, 253, -1);
        init_pair(2,  51, -1);
        init_pair(3,  75, -1);
        init_pair(4, 214, -1);
        init_pair(5, 154, -1);
        init_pair(6,  46, -1);
        init_pair(7, 238, -1);
        init_pair(8, 226, -1);
    } else {
        init_pair(1, COLOR_WHITE,  -1);
        init_pair(2, COLOR_CYAN,   -1);
        init_pair(3, COLOR_BLUE,   -1);
        init_pair(4, COLOR_YELLOW, -1);
        init_pair(5, COLOR_GREEN,  -1);
        init_pair(6, COLOR_GREEN,  -1);
        init_pair(7, COLOR_WHITE,  -1);
        init_pair(8, COLOR_YELLOW, -1);
    }
}

/* ===================================================================== */
/* §4  coords + bone_glyph                                               */
/* ===================================================================== */

/*
 * Pixel ↔ cell conversion.  All physics use square pixel space; only at
 * draw time are positions mapped to terminal cells via these helpers.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * bone_glyph — best ASCII char for a line oriented (dx, dy).
 * Maps direction angle to '-', '/', '|', or '\'.
 * dy is negated before atan2 to convert +y-down screen space to math space.
 */
static chtype bone_glyph(float dx, float dy)
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

/* ===================================================================== */
/* §5a  solve_ik                                                          */
/* ===================================================================== */

typedef struct { float x, y; } Vec2;

/*
 * 2-joint analytical IK (law of cosines).
 *
 * Given hip H, foot F, upper_len U, lower_len L:
 *   clamped = clamp(|F−H|, |U−L|+ε, U+L−ε)
 *   base    = atan2(F.y−H.y, F.x−H.x)
 *   cos_h   = (clamped² + U² − L²) / (2·clamped·U)
 *   ah      = acos(cos_h)
 *   knee    = H + U·(cos(base−ah), sin(base−ah))
 *
 * Sign convention `base − ah` places the knee toward +x (forward) in
 * 2-D side view with +y downward — verified for both swing and stance.
 */
static Vec2 solve_ik(Vec2 hip, Vec2 foot, float U, float L)
{
    float dx   = foot.x - hip.x;
    float dy   = foot.y - hip.y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1e-6f) dist = 1e-6f;

    float clamped = dist;
    float max_r = U + L - 0.5f;
    float min_r = fabsf(U - L) + 0.5f;
    if (clamped > max_r) clamped = max_r;
    if (clamped < min_r) clamped = min_r;

    float base  = atan2f(dy, dx);   /* original direction preserved */
    float cos_h = (clamped*clamped + U*U - L*L) / (2.0f * clamped * U);
    cos_h = fmaxf(-1.0f, fminf(1.0f, cos_h));
    float ah  = acosf(cos_h);
    float ang = base - ah;          /* knee bends forward (+x side) */

    return (Vec2){ hip.x + U * cosf(ang), hip.y + U * sinf(ang) };
}

/* ===================================================================== */
/* §5b  Robot struct                                                      */
/* ===================================================================== */

typedef struct {
    /* ── motion state ─────────────────────────────────────────────── */
    float x;               /* hip-centre x (pixel space)              */
    float phase;           /* walk phase φ (rad, unbounded)           */
    float walk_freq;       /* gait frequency (Hz)                     */
    float walk_speed_px;   /* forward translation speed (px/s)        */
    int   walk_dir;        /* +1 forward, -1 reverse                  */
    bool  paused;
    bool  show_grid;
    bool  step_once;       /* advance one tick while paused           */

    /* ── screen-dependent constants ──────────────────────────────── */
    float ground_y;
    float base_hip_y;

    /* ── foot contact locking ─────────────────────────────────────── */
    Vec2  foot_lock[2];       /* locked world foot positions           */
    bool  foot_on_ground[2];  /* is foot in stance this frame?        */

    /* ── computed joints (pixel space) ───────────────────────────── */
    Vec2  hip_c;              /* pelvis midpoint                      */
    Vec2  hip_j[2];           /* hip joints: 0=L, 1=R                */
    Vec2  torso_top;          /* top of torso (shoulder level)        */
    Vec2  neck;               /* neck base                            */
    Vec2  head_c;             /* head centre                          */
    Vec2  shoulder[2];
    Vec2  elbow[2];
    Vec2  wrist[2];
    Vec2  knee[2];
    Vec2  foot_j[2];          /* foot tip                             */
    float com_x;              /* centre-of-mass projected x           */

    /* ── head motion trail ────────────────────────────────────────── */
    Vec2  trail[TRAIL_LEN];
    int   trail_head;         /* ring-buffer write index              */
    int   trail_fill;         /* count of valid entries               */
} Robot;

/* ===================================================================== */
/* §5c  compute_pose                                                      */
/* ===================================================================== */

static void compute_pose(Robot *r)
{
    float phi = r->phase;
    float dir = (float)r->walk_dir;

    /* ── body ─────────────────────────────────────────────────────── */
    float bob  = BOB_AMP  * sinf(2.0f * phi);          /* 2× per stride */
    float sway = SWAY_AMP * cosf(phi) * dir;            /* lean over stance */

    r->hip_c     = (Vec2){ r->x,               r->base_hip_y + bob };
    r->hip_j[0]  = (Vec2){ r->hip_c.x - HIP_W, r->hip_c.y };
    r->hip_j[1]  = (Vec2){ r->hip_c.x + HIP_W, r->hip_c.y };

    r->torso_top = (Vec2){ r->hip_c.x + sway,  r->hip_c.y - TORSO_LEN };
    r->neck      = (Vec2){ r->torso_top.x,      r->torso_top.y - NECK_LEN };
    r->head_c    = (Vec2){ r->neck.x,           r->neck.y - HEAD_R };

    r->shoulder[0] = (Vec2){ r->torso_top.x - SHOULDER_W, r->torso_top.y };
    r->shoulder[1] = (Vec2){ r->torso_top.x + SHOULDER_W, r->torso_top.y };

    /* ── arms ─────────────────────────────────────────────────────── */
    /* Arm 0 (left)  phases with right leg → φ + π  (counter-swing)  */
    /* Arm 1 (right) phases with left  leg → φ      (counter-swing)  */
    for (int i = 0; i < 2; i++) {
        float ap = phi + (i == 0 ? (float)M_PI : 0.0f);
        float ua = ARM_SWING * sinf(ap) * dir;
        float la = ua + ARM_BEND
                 + 0.3f * ARM_SWING * sinf(ap - 0.5f) * dir;

        r->elbow[i].x = r->shoulder[i].x + UPPER_ARM_LEN * sinf(ua);
        r->elbow[i].y = r->shoulder[i].y + UPPER_ARM_LEN * cosf(ua);
        r->wrist[i].x = r->elbow[i].x   + LOWER_ARM_LEN * sinf(la);
        r->wrist[i].y = r->elbow[i].y   + LOWER_ARM_LEN * cosf(la);
    }

    /* ── legs ─────────────────────────────────────────────────────── */
    /* Leg 0 (left): φ_L = φ   Leg 1 (right): φ_R = φ + π           */
    for (int i = 0; i < 2; i++) {
        float lp  = phi + (i == 1 ? (float)M_PI : 0.0f);
        float sn  = sinf(lp);
        Vec2  hip = r->hip_j[i];

        if (sn > 0.0f) {
            /* swing phase ── FK */
            float ta = -THIGH_SWING * cosf(lp) * dir;
            float ke =  KNEE_LIFT   * sinf(lp);       /* always lifts up */
            float sa = ta + ke;

            Vec2 knee, foot;
            knee.x = hip.x + UPPER_LEG_LEN * sinf(ta);
            knee.y = hip.y + UPPER_LEG_LEN * cosf(ta);
            foot.x = knee.x + LOWER_LEG_LEN * sinf(sa);
            foot.y = knee.y + LOWER_LEG_LEN * cosf(sa);
            if (foot.y > r->ground_y) foot.y = r->ground_y;

            r->knee[i]           = knee;
            r->foot_j[i]         = foot;
            r->foot_on_ground[i] = false;
        } else {
            /* stance phase ── IK from locked foot */
            Vec2 foot = r->foot_lock[i];
            foot.y    = r->ground_y;

            r->knee[i]           = solve_ik(hip, foot, UPPER_LEG_LEN, LOWER_LEG_LEN);
            r->foot_j[i]         = foot;
            r->foot_on_ground[i] = true;
        }
    }

    /* COM: head×1 + torso_top×2 + hip_c×2 (top-heavy weighting) */
    r->com_x = (r->head_c.x * 1.0f
              + r->torso_top.x * 2.0f
              + r->hip_c.x    * 2.0f) / 5.0f;
}

/* ===================================================================== */
/* §5d  robot_init                                                        */
/* ===================================================================== */

static void robot_init(Robot *r, int cols, int rows)
{
    memset(r, 0, sizeof *r);

    r->walk_freq     = WALK_FREQ_DEFAULT;
    r->walk_speed_px = WALK_SPEED_DEFAULT;
    r->walk_dir      = 1;
    r->show_grid     = true;

    r->ground_y   = (float)((rows - 4) * CELL_H);
    r->base_hip_y = r->ground_y - (UPPER_LEG_LEN + LOWER_LEG_LEN) * 0.88f;
    r->x          = (float)(cols * CELL_W) * 0.35f;

    /* Seed foot locks: one stride-quarter apart, centred under body */
    float step = (r->walk_speed_px / r->walk_freq) * 0.25f;
    r->foot_lock[0] = (Vec2){ r->x - step, r->ground_y };
    r->foot_lock[1] = (Vec2){ r->x + step, r->ground_y };

    r->phase = 0.3f;

    compute_pose(r);

    for (int i = 0; i < TRAIL_LEN; i++)
        r->trail[i] = r->head_c;
    r->trail_fill = TRAIL_LEN;
    r->trail_head = 0;
}

/* ===================================================================== */
/* §5e  robot_tick                                                        */
/* ===================================================================== */

static void robot_tick(Robot *r, float dt, int cols, int rows)
{
    if (r->paused && !r->step_once) return;
    r->step_once = false;

    float old_phase = r->phase;
    float dir = (float)r->walk_dir;

    /* 1. Advance phase and position */
    r->phase += 2.0f * (float)M_PI * r->walk_freq * dir * dt;
    r->x     += r->walk_speed_px * dir * dt;

    /* 2. Update screen-dependent constants */
    r->ground_y   = (float)((rows - 4) * CELL_H);
    r->base_hip_y = r->ground_y - (UPPER_LEG_LEN + LOWER_LEG_LEN) * 0.88f;

    /* 3. Pre-compute hip position (needed for landing FK) */
    float bob = BOB_AMP * sinf(2.0f * r->phase);
    r->hip_c    = (Vec2){ r->x,               r->base_hip_y + bob };
    r->hip_j[0] = (Vec2){ r->hip_c.x - HIP_W, r->hip_c.y };
    r->hip_j[1] = (Vec2){ r->hip_c.x + HIP_W, r->hip_c.y };

    /* 4. Detect stance entry (sin: + → −) and lock foot at FK landing pos */
    for (int i = 0; i < 2; i++) {
        float old_lp = old_phase + (i == 1 ? (float)M_PI : 0.0f);
        float new_lp = r->phase  + (i == 1 ? (float)M_PI : 0.0f);
        if (sinf(old_lp) > 0.0f && sinf(new_lp) <= 0.0f) {
            /* At touchdown: thigh is at max excursion, shin nearly straight */
            float ta = -THIGH_SWING * cosf(new_lp) * dir;
            float sa = ta;   /* KNEE_LIFT·sin(≈0) ≈ 0 */
            Vec2  h  = r->hip_j[i];
            float fx = h.x
                     + UPPER_LEG_LEN * sinf(ta)
                     + LOWER_LEG_LEN * sinf(sa);
            r->foot_lock[i] = (Vec2){ fx, r->ground_y };
        }
    }

    /* 5. Full pose */
    compute_pose(r);

    /* 6. Push head position into trail ring buffer */
    r->trail[r->trail_head] = r->head_c;
    r->trail_head = (r->trail_head + 1) % TRAIL_LEN;
    if (r->trail_fill < TRAIL_LEN) r->trail_fill++;

    /* 7. Wrap robot at screen edges */
    float sw = (float)(cols * CELL_W);
    if (r->x > sw + 80.0f) {
        r->x -= sw + 160.0f;
        r->foot_lock[0].x -= sw + 160.0f;
        r->foot_lock[1].x -= sw + 160.0f;
        r->trail_fill = 0;
    }
    if (r->x < -80.0f) {
        r->x += sw + 160.0f;
        r->foot_lock[0].x += sw + 160.0f;
        r->foot_lock[1].x += sw + 160.0f;
        r->trail_fill = 0;
    }
}

/* ===================================================================== */
/* §5f  draw helpers                                                      */
/* ===================================================================== */

/*
 * draw_bone() — stamp ASCII glyphs along one bone from pixel a to b.
 * Walks the bone in DRAW_STEP_PX increments; deduplicated via prev_cx/cy.
 * Out-of-bounds cells silently skipped.
 */
static void draw_bone(WINDOW *w,
                      Vec2 a, Vec2 b,
                      int pair, attr_t attr,
                      int cols, int rows,
                      int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.1f) return;

    chtype glyph  = bone_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);
        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx; *prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/*
 * draw_ground() — horizontal ground line + optional scrolling grid.
 *
 *   gy    : ground line row  ( ─ × cols )
 *   gy+1  : tick marks scrolling left-to-right with robot
 *   gy+2  : perspective dots (wider spacing = distant)
 */
static void draw_ground(WINDOW *w, const Robot *r, int cols, int rows)
{
    int gy = px_to_cell_y(r->ground_y);
    if (gy < 0 || gy >= rows) return;

    wattron(w, COLOR_PAIR(7) | A_NORMAL);
    for (int cx = 0; cx < cols; cx++)
        mvwaddch(w, gy, cx, ACS_HLINE);
    wattroff(w, COLOR_PAIR(7) | A_NORMAL);

    if (!r->show_grid) return;

    /* Tick marks at gy+1 — offset scrolls with robot.x */
    if (gy + 1 >= rows - 1) return;
    int offset = (int)(r->x / (float)CELL_W) % GRID_PERIOD;
    if (offset < 0) offset += GRID_PERIOD;

    wattron(w, COLOR_PAIR(7) | A_DIM);
    for (int cx = 0; cx < cols; cx++) {
        chtype ch = ((cx + offset) % GRID_PERIOD == 0) ? '|' : ' ';
        mvwaddch(w, gy + 1, cx, ch);
    }

    /* Perspective dots at gy+2 (double period = simulates depth) */
    if (gy + 2 >= rows - 1) { wattroff(w, COLOR_PAIR(7) | A_DIM); return; }
    int off2   = (int)(r->x / (float)(CELL_W * 2)) % (GRID_PERIOD * 2);
    if (off2 < 0) off2 += GRID_PERIOD * 2;
    for (int cx = 0; cx < cols; cx++) {
        if ((cx + off2) % (GRID_PERIOD * 2) == 0)
            mvwaddch(w, gy + 2, cx, '.');
    }
    wattroff(w, COLOR_PAIR(7) | A_DIM);
}

/*
 * draw_shadow() — elliptical drop shadow on the ground line.
 *
 * Drawn at the ground row AFTER the ground line; overwrites '─' with
 * dim '~' (edges) and ':' (centre) to simulate a floor shadow.
 */
static void draw_shadow(WINDOW *w, const Robot *r, int cols, int rows)
{
    int gy     = px_to_cell_y(r->ground_y);
    int cx_mid = px_to_cell_x(r->com_x);
    int semi   = (int)(SHADOW_SEMI_X / (float)CELL_W + 0.5f);

    if (gy < 0 || gy >= rows) return;

    wattron(w, COLOR_PAIR(7) | A_DIM);
    for (int cx = cx_mid - semi; cx <= cx_mid + semi; cx++) {
        if (cx < 0 || cx >= cols) continue;
        float t  = fabsf((float)(cx - cx_mid) / (float)(semi + 1));
        chtype ch = (t < 0.35f) ? ':' : '~';
        mvwaddch(w, gy, cx, ch);
    }
    wattroff(w, COLOR_PAIR(7) | A_DIM);
}

/*
 * draw_trail() — fading head-position trail behind the robot.
 *
 * age 0–3   : 'o'  pair 2  A_NORMAL  (bright recent)
 * age 4–11  : '.'  pair 7  A_NORMAL  (mid-fade)
 * age 12+   : '.'  pair 7  A_DIM     (old, nearly invisible)
 */
static void draw_trail(WINDOW *w, const Robot *r, int cols, int rows)
{
    if (r->trail_fill < 2) return;

    for (int age = r->trail_fill - 1; age >= 1; age--) {
        int idx = ((r->trail_head - 1 - age) % TRAIL_LEN + TRAIL_LEN) % TRAIL_LEN;
        int cx  = px_to_cell_x(r->trail[idx].x);
        int cy  = px_to_cell_y(r->trail[idx].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        chtype ch;
        int    cp;
        attr_t at;

        if      (age < 4)  { ch = 'o'; cp = 2; at = A_NORMAL; }
        else if (age < 12) { ch = '.'; cp = 7; at = A_NORMAL; }
        else               { ch = '.'; cp = 7; at = A_DIM;    }

        wattron(w, COLOR_PAIR(cp) | at);
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(cp) | at);
    }
}

/*
 * draw_com() — centre-of-mass vertical projection + ground marker.
 *
 * Dotted ':' line from hip level to ground; '^' marker at ground contact.
 */
static void draw_com(WINDOW *w, const Robot *r, int cols, int rows)
{
    int gy     = px_to_cell_y(r->ground_y);
    int cx     = px_to_cell_x(r->com_x);
    int hip_cy = px_to_cell_y(r->hip_c.y);

    if (cx < 0 || cx >= cols) return;

    /* Vertical projection line */
    wattron(w, COLOR_PAIR(7) | A_DIM);
    for (int cy = hip_cy + 1; cy < gy; cy++) {
        if (cy < 0 || cy >= rows) continue;
        mvwaddch(w, cy, cx, ':');
    }
    wattroff(w, COLOR_PAIR(7) | A_DIM);

    /* Ground marker */
    if (gy >= 0 && gy < rows) {
        wattron(w, COLOR_PAIR(4) | A_BOLD);
        mvwaddch(w, gy, cx, '^');
        wattroff(w, COLOR_PAIR(4) | A_BOLD);
    }
}

/* ===================================================================== */
/* §5g  robot_draw                                                        */
/* ===================================================================== */

static void robot_draw(const Robot *r, WINDOW *w, int cols, int rows)
{
    int pcx, pcy;

    /* 1 ── Ground grid ──────────────────────────────────────────── */
    draw_ground(w, r, cols, rows);

    /* 2 ── Trail (oldest first so newer overwrites) ─────────────── */
    draw_trail(w, r, cols, rows);

    /* 3 ── Drop shadow ───────────────────────────────────────────── */
    draw_shadow(w, r, cols, rows);

    /* 4 ── COM projection ────────────────────────────────────────── */
    draw_com(w, r, cols, rows);

    /* 5 ── Bones ─────────────────────────────────────────────────── */

    /* Spine: hip_c → torso_top */
    pcx = -9999; pcy = -9999;
    draw_bone(w, r->hip_c, r->torso_top, 2, A_BOLD, cols, rows, &pcx, &pcy);

    /* Neck: torso_top → neck */
    pcx = -9999; pcy = -9999;
    draw_bone(w, r->torso_top, r->neck, 2, A_NORMAL, cols, rows, &pcx, &pcy);

    /* Collar: shoulder[0] → shoulder[1] */
    pcx = -9999; pcy = -9999;
    draw_bone(w, r->shoulder[0], r->shoulder[1], 2, A_DIM, cols, rows, &pcx, &pcy);

    /* Arms: shoulder → elbow (upper), elbow → wrist (lower) */
    for (int i = 0; i < 2; i++) {
        pcx = -9999; pcy = -9999;
        draw_bone(w, r->shoulder[i], r->elbow[i], 3, A_NORMAL, cols, rows, &pcx, &pcy);
        pcx = -9999; pcy = -9999;
        draw_bone(w, r->elbow[i],    r->wrist[i], 4, A_NORMAL, cols, rows, &pcx, &pcy);
    }

    /* Hip band: hip_j[0] → hip_j[1] */
    pcx = -9999; pcy = -9999;
    draw_bone(w, r->hip_j[0], r->hip_j[1], 2, A_DIM, cols, rows, &pcx, &pcy);

    /* Legs: hip_j → knee (thigh), knee → foot_j (shin) */
    for (int i = 0; i < 2; i++) {
        pcx = -9999; pcy = -9999;
        draw_bone(w, r->hip_j[i], r->knee[i],  5, A_NORMAL, cols, rows, &pcx, &pcy);
        pcx = -9999; pcy = -9999;
        draw_bone(w, r->knee[i],  r->foot_j[i], 6, A_NORMAL, cols, rows, &pcx, &pcy);
    }

    /* 6 ── Joint markers ─────────────────────────────────────────── */

    /* Head: '(O)' centred at head_c */
    {
        int hcx = px_to_cell_x(r->head_c.x);
        int hcy = px_to_cell_y(r->head_c.y);
        wattron(w, COLOR_PAIR(1) | A_BOLD);
        if (hcy >= 0 && hcy < rows) {
            if (hcx - 1 >= 0 && hcx - 1 < cols) mvwaddch(w, hcy, hcx - 1, '(');
            if (hcx     >= 0 && hcx     < cols) mvwaddch(w, hcy, hcx,     'O');
            if (hcx + 1 >= 0 && hcx + 1 < cols) mvwaddch(w, hcy, hcx + 1, ')');
        }
        wattroff(w, COLOR_PAIR(1) | A_BOLD);
    }

    /* Wrists: '*' */
    for (int i = 0; i < 2; i++) {
        int cx = px_to_cell_x(r->wrist[i].x);
        int cy = px_to_cell_y(r->wrist[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(4) | A_BOLD);
        mvwaddch(w, cy, cx, '*');
        wattroff(w, COLOR_PAIR(4) | A_BOLD);
    }

    /* Knee dots: 'o' dim */
    for (int i = 0; i < 2; i++) {
        int cx = px_to_cell_x(r->knee[i].x);
        int cy = px_to_cell_y(r->knee[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(5) | A_DIM);
        mvwaddch(w, cy, cx, 'o');
        wattroff(w, COLOR_PAIR(5) | A_DIM);
    }

    /* Feet + contact markers */
    for (int i = 0; i < 2; i++) {
        int cx = px_to_cell_x(r->foot_j[i].x);
        int cy = px_to_cell_y(r->foot_j[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        if (r->foot_on_ground[i]) {
            /* planted foot: bright '+' contact marker */
            wattron(w, COLOR_PAIR(4) | A_BOLD);
            mvwaddch(w, cy, cx, '+');
            wattroff(w, COLOR_PAIR(4) | A_BOLD);
        } else {
            /* swinging foot: 'v' tip in green */
            wattron(w, COLOR_PAIR(6) | A_NORMAL);
            mvwaddch(w, cy, cx, 'v');
            wattroff(w, COLOR_PAIR(6) | A_NORMAL);
        }
    }

    /* Elbow dots: dim small marker */
    for (int i = 0; i < 2; i++) {
        int cx = px_to_cell_x(r->elbow[i].x);
        int cy = px_to_cell_y(r->elbow[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(3) | A_DIM);
        mvwaddch(w, cy, cx, 'o');
        wattroff(w, COLOR_PAIR(3) | A_DIM);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Robot robot; } Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    robot_init(&sc->robot, cols, rows);
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    robot_tick(&sc->robot, dt, cols, rows);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    robot_draw(&sc->robot, w, cols, rows);
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

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    const Robot *r = &sc->robot;
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    /* HUD top-right: status */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " WALKING ROBOT  %.1fHz  %.0fpx/s  %s%s ",
             r->walk_freq,
             r->walk_speed_px,
             r->walk_dir == 1 ? ">" : "<",
             r->paused ? "  [PAUSED]" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    /* HUD bottom: keys */
    char buf2[HUD_COLS + 1];
    snprintf(buf2, sizeof buf2,
             " %.0ffps  %dHz  "
             "q:quit  spc:pause  r:reverse  +/-:speed  .:step  g:grid ",
             fps, sim_fps);
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0, "%s", buf2);
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
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Robot *r = &app->scene.robot;
    r->ground_y   = (float)((app->screen.rows - 4) * CELL_H);
    r->base_hip_y = r->ground_y - (UPPER_LEG_LEN + LOWER_LEG_LEN) * 0.88f;
    app->need_resize = 0;
}

/*
 * Hotkeys:
 *   q / ESC   quit
 *   SPACE     pause / resume
 *   r         reverse walk direction
 *   + / =     increase speed + frequency
 *   - / _     decrease speed + frequency
 *   .         step one frame (while paused)
 *   g         toggle ground grid
 *   [ / ]     sim Hz −/+
 */
static bool app_handle_key(App *app, int ch)
{
    Robot *r = &app->scene.robot;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        r->paused = !r->paused;
        break;

    case 'r': case 'R':
        r->walk_dir = -r->walk_dir;
        break;

    case '+': case '=': case KEY_UP:
        r->walk_speed_px += WALK_SPEED_STEP;
        r->walk_freq     += WALK_FREQ_STEP;
        if (r->walk_speed_px > WALK_SPEED_MAX) r->walk_speed_px = WALK_SPEED_MAX;
        if (r->walk_freq     > WALK_FREQ_MAX)  r->walk_freq     = WALK_FREQ_MAX;
        break;

    case '-': case '_': case KEY_DOWN:
        r->walk_speed_px -= WALK_SPEED_STEP;
        r->walk_freq     -= WALK_FREQ_STEP;
        if (r->walk_speed_px < WALK_SPEED_MIN) r->walk_speed_px = WALK_SPEED_MIN;
        if (r->walk_freq     < WALK_FREQ_MIN)  r->walk_freq     = WALK_FREQ_MIN;
        break;

    case '.':
        r->step_once = true;
        break;

    case 'g': case 'G':
        r->show_grid = !r->show_grid;
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

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app      = &g_app;
    app->sim_fps  = SIM_FPS_DEFAULT;
    app->running  = 1;
    app->need_resize = 0;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t t_now    = clock_ns();
    int64_t t_last   = t_now;
    int64_t accum_ns = 0;

    int64_t fps_last   = t_now;
    int     fps_frames = 0;
    double  fps_disp   = 0.0;

    while (app->running) {
        if (app->need_resize) app_do_resize(app);

        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        t_now = clock_ns();
        int64_t dt_ns = t_now - t_last;
        t_last = t_now;
        if (dt_ns > 200000000LL) dt_ns = 200000000LL;   /* cap at 0.2 s */
        accum_ns += dt_ns;

        int64_t tick_ns_val = TICK_NS(app->sim_fps);
        while (accum_ns >= tick_ns_val) {
            float dt = (float)tick_ns_val / (float)NS_PER_SEC;
            scene_tick(&app->scene, dt, app->screen.cols, app->screen.rows);
            accum_ns -= tick_ns_val;
        }

        screen_draw(&app->screen, &app->scene, fps_disp, app->sim_fps);
        screen_present();

        fps_frames++;
        if (t_now - fps_last >= (int64_t)FPS_UPDATE_MS * NS_PER_MS) {
            double elapsed = (double)(t_now - fps_last) / (double)NS_PER_SEC;
            fps_disp   = (double)fps_frames / elapsed;
            fps_frames = 0;
            fps_last   = t_now;
        }

        /* Sleep to ~60 render fps */
        int64_t frame_ns   = NS_PER_SEC / 60;
        int64_t elapsed_ns = clock_ns() - t_now;
        clock_sleep_ns(frame_ns - elapsed_ns);
    }

    screen_free(&app->screen);
    return 0;
}
