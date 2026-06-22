/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * walking_robot.c — an ASCII stick figure that walks across the screen.
 * One leg swings forward through the air while the other stays planted
 * and the body crosses over it; the two legs trade off forever.
 *
 * Sister files: animation/fk_helloworld.c (angles → joint positions, the
 * swing-leg math), animation/ik_helloworld.c (law of cosines, the
 * stance-leg math), animation/fk_ik_helloworld.c (both side by side).
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

/* ── §1 config ── */

enum { TARGET_FPS = 60 };

/* All the body math runs in "pixels": each terminal cell is 8 wide,
 * 16 tall. Keeping it in pixels means a knee rotation looks round
 * even though cells are twice as tall as they are wide. */
#define CELL_W   8
#define CELL_H  16

/* Body proportions, in pixels. Sized so each limb spans a few cells —
 * small enough and the line glyphs flicker. */
#define HEAD_R           8.0f       /* head size, used only for placement */
#define TORSO_LEN       72.0f       /* hip up to the top of the torso     */
#define SHOULDER_W      24.0f       /* half the shoulder width            */
#define HIP_W           14.0f       /* half the hip width                 */
#define UPPER_LEG_LEN   56.0f       /* hip to knee                        */
#define LOWER_LEG_LEN   48.0f       /* knee to foot                       */
#define ARM_LEN         60.0f       /* shoulder to hand                   */

/* How big each motion is. The angles are in radians. */
#define SWING_AMP    0.35f          /* how far the thigh swings (~20°)    */
#define LIFT_AMP     0.45f          /* how much the knee bends mid-step   */
#define ARM_SWING    0.30f          /* how far the arms swing             */
#define BOB_AMP      3.0f           /* how much the hips bob up and down  */
#define SWAY_AMP     4.0f           /* how much the body sways side to side */

/* Frequency is the only speed knob you turn; walk_speed is computed
 * from it (see robot_set_pace) so the body always moves exactly one
 * stride per step. Decouple them and the feet land too far ahead of
 * the body and both legs end up leaning forward the whole time. */
#define WALK_FREQ_DEFAULT   1.6f    /* steps per second, a relaxed pace   */
#define WALK_FREQ_MIN       0.4f
#define WALK_FREQ_MAX       4.5f
#define WALK_FREQ_STEP      0.2f

#define GRID_PERIOD     6           /* draw a ground tick every N columns */

/* Never advance the sim by more than this much in one frame, so a
 * stall (resize, swapped-out process) can't make everything jump. */
#define DT_CAP_SEC  0.10f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* Colour slots we hand to ncurses. */
enum {
    CP_BODY     = 1,        /* head, torso, hip band — white  */
    CP_BODY_DIM,            /* the faint connector lines      */
    CP_LEFT,                /* left arm + left leg — green    */
    CP_RIGHT,               /* right arm + right leg — magenta */
    CP_FOOT_PLANT,          /* a planted foot — yellow        */
    CP_FOOT_SWING,          /* a foot in the air — cyan       */
    CP_GROUND,              /* the ground line — dim grey     */
    PAIR_HUD,
    PAIR_HINT,
};

#define HUD_BUF_LEN  120

/* At rest the legs sit at 88% of full length, so the knees stay a
 * little bent. Two reasons: fully straight legs are a divide-by-zero
 * corner for the knee solver, and slightly-bent is how people
 * actually stand. */
#define STAND_LEG_RATIO         0.88f

/* Start the cycle slightly off zero. At exactly zero the touchdown
 * test would falsely fire on the very first frame. */
#define INITIAL_PHASE_RAD       0.30f

/* At boot, plant the two starting feet a quarter-stride either side
 * of the body so the first standing leg has somewhere to anchor. */
#define FOOT_LOCK_SEED_FRAC     0.25f

/* Start the body 30% of the way across the screen, not jammed against
 * an edge. */
#define INITIAL_X_FRAC          0.30f

/* Leave a few rows free at the bottom for the ground line, the ground
 * ticks, and the hint strip so they don't pile on top of each other. */
#define GROUND_BOTTOM_ROWS      4

/* When the robot walks off one side of the screen it pops back in on
 * the other. The two distances differ on purpose: it walks fully out
 * of sight before reappearing, so you never see it teleport. */
#define WRAP_MARGIN_PX         80.0f
#define WRAP_TOTAL_PX         160.0f

/* If the foot is almost on top of the hip, use this tiny distance
 * instead of zero so the knee solver doesn't divide by zero. */
#define IK_DIST_FLOOR          1e-6f

/* A little wiggle room kept away from full leg extension, so a
 * stretched-straight leg doesn't make the knee solver collapse. */
#define IK_REACH_SLACK          0.5f

/* ── §2 clock ── */

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

/* ── §3 color ── */

/*
 * One row per colour role: the slot to fill, plus the colour to use on
 * a fancy 256-colour terminal and the fallback for an 8-colour one.
 * Left and right limbs get different colours so you can follow each
 * leg through the step; feet change colour the moment they touch down.
 *   pair  — which slot from the §1 enum this fills
 *   fg256 — colour code on a 256-colour terminal
 *   fg8   — the closest match on a plain 8-colour terminal
 */
typedef struct {
    short pair;
    short fg256;
    short fg8;
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair             256    8-mode-fallback   role                */
    { CP_BODY,          255,   COLOR_WHITE   }, /* near-white         */
    { CP_BODY_DIM,      244,   COLOR_WHITE   }, /* dim grey-white     */
    { CP_LEFT,           82,   COLOR_GREEN   }, /* lime green         */
    { CP_RIGHT,         201,   COLOR_MAGENTA }, /* magenta            */
    { CP_FOOT_PLANT,    226,   COLOR_YELLOW  }, /* yellow             */
    { CP_FOOT_SWING,     51,   COLOR_CYAN    }, /* cyan               */
    { CP_GROUND,        240,   COLOR_WHITE   }, /* dim grey           */
    { PAIR_HUD,         226,   COLOR_YELLOW  }, /* yellow on bg       */
    { PAIR_HINT,         51,   COLOR_CYAN    }, /* cyan on bg         */
};
#define PALETTE_LEN  (int)(sizeof PALETTE / sizeof PALETTE[0])

static void color_init(void)
{
    start_color();
    use_default_colors();

    bool truecolor = (COLORS >= 256);
    for (int i = 0; i < PALETTE_LEN; i++) {
        short fg = truecolor ? PALETTE[i].fg256 : PALETTE[i].fg8;
        init_pair(PALETTE[i].pair, fg, -1);
    }
}

/* ── §4 coords — turn pixels into terminal cells ── */

/* The body math is in pixels; these turn a pixel position into a cell
 * row/column, and a cell count back into pixels. */
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_pw(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_ph(int rows){ return (float)rows * CELL_H; }
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── §5 robot — the figure, its walk cycle, and where its joints go ── */

/* A point in pixel space. x runs right, y runs DOWN (screen style, so
 * the top of the screen is y=0). */
typedef struct {
    float x;
    float y;
} Vec2;

/*
 * Gait — the walk cycle, the heartbeat that drives every limb.
 *
 *   A single number, phase, is really the whole state of the walk.
 *   Feed it through sin/cos and you get every joint angle, every bit
 *   of body bounce, and the moments each foot touches down. The other
 *   three fields just say how fast phase moves and which way.
 *
 *   phase      — where we are in the step, in radians. It just keeps
 *                growing; we never wrap it, sin/cos do that for us.
 *   walk_freq  — steps per second, the one speed knob you turn.
 *                Kept inside [0.4, 4.5].
 *   walk_speed — how fast the body slides forward, in pixels/sec.
 *                Never set this by hand — robot_set_pace() works it
 *                out from walk_freq so the feet and body stay in sync.
 *                (Set it freely and the feet land too far ahead and
 *                the legs lean forward the whole time.)
 *   direction  — +1 forward, -1 backward; the 'r' key flips it.
 *
 *   The single-clock idea comes from Boulic, Magnenat-Thalmann &
 *   Thalmann (1990), procedural human walking.
 */
typedef struct {
    float phase;
    float walk_freq;
    float walk_speed;
    int   direction;
} Gait;

/*
 * GroundAnchor — where the floor is and how high the hips rest.
 *
 *   The pose math never asks ncurses how big the screen is. Instead
 *   these two values are worked out from the screen size once at
 *   startup (and again whenever the window resizes), and every joint
 *   reads them from here. So handling a resize is just rewriting these
 *   two numbers; nothing else has to know the screen changed.
 *
 *   ground_y    — the floor line, in pixels. Sits a few rows up from
 *                 the bottom edge to leave room for the HUD and hints.
 *   base_hip_y  — the resting hip height, before any walk bounce is
 *                 added. Set so the legs rest at 88% extension (knees
 *                 slightly bent — see STAND_LEG_RATIO in §1).
 */
typedef struct {
    float ground_y;
    float base_hip_y;
} GroundAnchor;

/*
 * FootLocks — where each planted foot is pinned to the ground.
 *
 *   This is the trick that makes the walk look real. The moment a foot
 *   touches down, we remember exactly where it landed and pin it there.
 *   Until that foot lifts again, the body slides forward OVER a fixed
 *   foot, and the knee solver bends the leg to keep up. Skip this and
 *   the foot slides along the ground as the body moves — the robot
 *   would look like it's moonwalking.
 *
 *   Each leg is either swinging (foot in the air, driven from the hip)
 *   or planted (foot fixed here, knee solved to match). The swap is
 *   purely a function of where we are in the step.
 *
 *   pos[i]        — where leg i's foot is pinned, in pixels. Always
 *                   sits on the ground line.
 *   on_ground[i]  — true while leg i is planted. The renderer reads
 *                   this to pick the foot glyph and colour, and the
 *                   HUD reads it to show stance/swing — neither has to
 *                   redo the step-phase math.
 *
 *   The swing/stance split is the core idea of Raibert (1986),
 *   "Legged Robots That Balance".
 */
typedef struct {
    Vec2 pos[2];
    bool on_ground[2];
} FootLocks;

/*
 * Pose — every joint position for this frame, ready to draw.
 *
 *   compute_pose() fills this in once per frame; after that it's
 *   read-only. The renderer just draws a line between each connected
 *   pair of points here, so working everything out once up front
 *   beats redoing the trig for each line.
 *
 *   The body is a tree: each joint hangs off the one above it, so
 *   compute_pose() fills them in from the top down — a child can't be
 *   placed until its parent is.
 *
 *      hip_c (pelvis centre)
 *        ├─ hip_j[2]    one hip per leg, just left/right of centre
 *        │    └─ knee[2] / foot[2]   the legs
 *        └─ torso_top
 *             ├─ shoulder[2] → hand[2]   the arms
 *             └─ head_c   ('O')
 *
 *   The top-down tree of joints follows Parent (2012); the bent-knee
 *   case uses the solver from Buss (2009).
 */
typedef struct {
    Vec2 hip_c;              /* pelvis centre — the root of everything */
    Vec2 hip_j[2];           /* one hip joint per leg                   */
    Vec2 torso_top;          /* top of the torso; the side-sway lives
                              * here, which is what tips the upper body
                              * over the planted foot                   */
    Vec2 head_c;             /* head centre, drawn as 'O'               */

    Vec2 shoulder[2];        /* one shoulder per arm                    */
    Vec2 hand[2];            /* hand at the end of each arm; the arm
                              * swings opposite the leg on its side, the
                              * way people swing their arms to balance  */

    Vec2 knee[2];            /* one knee per leg                        */
    Vec2 foot[2];            /* one foot per leg; a swinging foot is
                              * stopped at the ground so it can't sink
                              * through the floor                       */
} Pose;

/*
 * RobotUI — the on/off switches the keyboard flips.
 *
 *   These have nothing to do with how walking works; they only exist
 *   because a person is watching. Kept apart from the gait state so
 *   that's clear. robot_reset() keeps these (and the speed/direction)
 *   across a resize, so a reset restarts the walk but not your settings.
 *
 *   paused     — spacebar; freezes the walk.
 *   step_once  — '.'; nudge one frame forward while paused, then it
 *                clears itself.
 *   show_grid  — 'g'; show the scrolling ground ticks.
 */
typedef struct {
    bool paused;
    bool step_once;
    bool show_grid;
} RobotUI;

/*
 * Robot — the whole figure, gathered from the five pieces above.
 *
 *   x is kept here at the top rather than inside Gait because it's the
 *   one anchor everything else is measured from: every joint and every
 *   planted foot is positioned relative to it. "Where is the robot" =
 *   r->x. Gait controls how x moves; x itself is just where it is.
 *
 *   x      — the body's left-right position, in pixels. Walks off one
 *            screen edge and wraps to the other.
 *   gait   — the walk cycle (the heartbeat).
 *   anchor — where the floor is and the resting hip height.
 *   locks  — where each planted foot is pinned.
 *   pose   — every joint position for this frame.
 *   ui     — the keyboard switches.
 */
typedef struct {
    float        x;
    Gait         gait;
    GroundAnchor anchor;
    FootLocks    locks;
    Pose         pose;
    RobotUI      ui;
} Robot;

/* ── §5.2 solve_ik2 — bend one leg to reach a fixed foot ── */

/*
 * The knee solver. We know where the hip is and where the foot is
 * pinned; this works out where the knee goes so the two leg segments
 * (lengths U and L) connect them. It's the law of cosines: from the
 * three side lengths of the hip-knee-foot triangle we get the angle at
 * the hip, then step that far along to place the knee.
 *
 * We always bend the knee toward the front (the natural walking way).
 * If the foot is pinned impossibly close or far, we quietly pretend
 * the leg is straight instead of returning garbage.
 *
 * animation/ik_helloworld.c teaches this same math from scratch.
 */
static Vec2 solve_ik2(Vec2 hip, Vec2 foot, float U, float L)
{
    float dx   = foot.x - hip.x;
    float dy   = foot.y - hip.y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < IK_DIST_FLOOR) dist = IK_DIST_FLOOR;

    /* keep the hip-to-foot distance within what the leg can actually
     * reach, staying clear of dead-straight where the math collapses */
    float min_reach = fabsf(U - L) + IK_REACH_SLACK;
    float max_reach = U + L        - IK_REACH_SLACK;
    float reach     = clampf(dist, min_reach, max_reach);

    float cos_alpha = (reach*reach + U*U - L*L) / (2.0f * reach * U);
    float alpha     = acosf(clampf(cos_alpha, -1.0f, 1.0f));

    float phi       = atan2f(dy, dx);
    float knee_ang  = phi - alpha;
    return (Vec2){ hip.x + U * cosf(knee_ang),
                   hip.y + U * sinf(knee_ang) };
}

/* ── §5.3 swing leg — foot in the air, driven from the hip ── */

/* Step one segment out from a joint at the given angle. (On screen, y
 * runs down, which is why down uses cos and forward uses sin.) */
static Vec2 fk_advance(Vec2 root, float angle, float len)
{
    return (Vec2){ root.x + len * sinf(angle),
                   root.y + len * cosf(angle) };
}

static void leg_swing_pose(Robot *r, int i, Vec2 hip, float phi_leg, float dir)
{
    /* thigh swings from back, through straight down, to front; the knee
     * bends most at mid-step then straightens to land. shin angle is
     * thigh minus the bend so the heel folds up behind, not forward. */
    float thigh_angle = -SWING_AMP * cosf(phi_leg) * dir;
    float knee_bend   =  LIFT_AMP  * sinf(phi_leg);
    float shin_angle  =  thigh_angle - knee_bend;

    Vec2 knee = fk_advance(hip,  thigh_angle, UPPER_LEG_LEN);
    Vec2 foot = fk_advance(knee, shin_angle,  LOWER_LEG_LEN);

    /* don't let a swinging foot dip below the floor */
    if (foot.y > r->anchor.ground_y) foot.y = r->anchor.ground_y;

    r->pose.knee[i]       = knee;
    r->pose.foot[i]       = foot;
    r->locks.on_ground[i] = false;
}

/* ── §5.4 stance leg — foot pinned, knee solved to match ── */

/* The foot stays where it was pinned; the hip has moved on, so we ask
 * the solver where the knee must go to connect them. */
static void leg_stance_pose(Robot *r, int i, Vec2 hip)
{
    Vec2 foot = r->locks.pos[i];
    foot.y    = r->anchor.ground_y;

    r->pose.knee[i]       = solve_ik2(hip, foot, UPPER_LEG_LEN, LOWER_LEG_LEN);
    r->pose.foot[i]       = foot;
    r->locks.on_ground[i] = true;
}

/* ── §5.5 arms ── */

/* Each arm is one segment from shoulder to hand. It swings opposite the
 * leg on its own side — left leg forward, left arm back — which is how
 * people keep their balance while walking. */
static void compute_arm(Robot *r, int i, float phase, float dir)
{
    /* shift the left arm's cycle by half so it opposes the left leg */
    float arm_phase = phase + (i == 0 ? (float)M_PI : 0.0f);
    float angle     = ARM_SWING * sinf(arm_phase) * dir;

    r->pose.hand[i].x = r->pose.shoulder[i].x + ARM_LEN * sinf(angle);
    r->pose.hand[i].y = r->pose.shoulder[i].y + ARM_LEN * cosf(angle);
}

/* ── §5.6 compute_pose — work out every joint for this frame ── */

/* How much the hips bounce up and down — twice per step, the bob you
 * see in a real walk. */
static inline float hip_bob_offset(float phi)
{
    return BOB_AMP * sinf(2.0f * phi);
}

/* How much the body leans side to side — once per step, tipping toward
 * whichever foot is planted so it stays balanced over its support. */
static inline float torso_sway_offset(float phi, float dir)
{
    return SWAY_AMP * cosf(phi) * dir;
}

/* The right leg runs half a cycle behind the left, so one is always in
 * the air while the other is planted. */
static inline float leg_phase(float phi, int i)
{
    return phi + (i == 1 ? (float)M_PI : 0.0f);
}

/* Is this leg in the air right now (rather than planted)? */
static inline bool leg_is_swinging(float phi_leg)
{
    return sinf(phi_leg) > 0.0f;
}

static void pose_place_pelvis(Pose *p, float x, float base_y, float bob)
{
    p->hip_c    = (Vec2){ x,                 base_y + bob };
    p->hip_j[0] = (Vec2){ p->hip_c.x - HIP_W, p->hip_c.y  };
    p->hip_j[1] = (Vec2){ p->hip_c.x + HIP_W, p->hip_c.y  };
}

static void pose_place_torso(Pose *p, float sway)
{
    p->torso_top = (Vec2){ p->hip_c.x + sway,   p->hip_c.y - TORSO_LEN };
    p->head_c    = (Vec2){ p->torso_top.x,      p->torso_top.y - HEAD_R };
}

static void pose_place_shoulders(Pose *p)
{
    p->shoulder[0] = (Vec2){ p->torso_top.x - SHOULDER_W, p->torso_top.y };
    p->shoulder[1] = (Vec2){ p->torso_top.x + SHOULDER_W, p->torso_top.y };
}

/* swing or stance for this leg, depending on where it is in the step */
static void pose_place_one_leg(Robot *r, int i, float phi, float dir)
{
    float phi_leg = leg_phase(phi, i);
    if (leg_is_swinging(phi_leg))
        leg_swing_pose (r, i, r->pose.hip_j[i], phi_leg, dir);
    else
        leg_stance_pose(r, i, r->pose.hip_j[i]);
}

/*
 * Build the whole figure for this frame. Works top-down — each part
 * hangs off the one above, so a part can't be placed until its parent
 * is: pelvis, then torso and head, then shoulders, arms, and legs.
 */
static void compute_pose(Robot *r)
{
    Pose *p   = &r->pose;
    float phi = r->gait.phase;
    float dir = (float)r->gait.direction;

    float bob  = hip_bob_offset(phi);
    float sway = torso_sway_offset(phi, dir);

    pose_place_pelvis   (p, r->x, r->anchor.base_hip_y, bob);
    pose_place_torso    (p, sway);
    pose_place_shoulders(p);

    for (int i = 0; i < 2; i++) compute_arm(r, i, phi, dir);
    for (int i = 0; i < 2; i++) pose_place_one_leg(r, i, phi, dir);
}

/* ── §5.7 setup ── */

/*
 * Set the walking pace. You only pick the frequency; the forward speed
 * is computed from it so the body moves exactly one stride per step.
 * Always change speed through here so the two never drift apart (set
 * them independently and the legs end up leaning forward the whole
 * time, because the feet land ahead of where the body catches up to).
 */
static void robot_set_pace(Robot *r, float freq)
{
    if (freq < WALK_FREQ_MIN) freq = WALK_FREQ_MIN;
    if (freq > WALK_FREQ_MAX) freq = WALK_FREQ_MAX;

    float stride       = 2.0f * (UPPER_LEG_LEN + LOWER_LEG_LEN) * sinf(SWING_AMP);
    r->gait.walk_freq  = freq;
    r->gait.walk_speed = freq * stride;
}

/* Recompute the floor line and resting hip height from the current
 * screen height. Re-run every frame so a window resize just works. */
static void anchor_update_from_screen(GroundAnchor *a, int rows)
{
    a->ground_y   = (float)((rows - GROUND_BOTTOM_ROWS) * CELL_H);
    a->base_hip_y = a->ground_y
                  - (UPPER_LEG_LEN + LOWER_LEG_LEN) * STAND_LEG_RATIO;
}

/* Plant the two starting feet, one ahead of the body and one behind,
 * so the first standing leg has somewhere to anchor. */
static void seed_foot_locks(Robot *r)
{
    float stride_len  = r->gait.walk_speed / r->gait.walk_freq;
    float seed_offset = stride_len * FOOT_LOCK_SEED_FRAC;
    r->locks.pos[0] = (Vec2){ r->x - seed_offset, r->anchor.ground_y };
    r->locks.pos[1] = (Vec2){ r->x + seed_offset, r->anchor.ground_y };
}

/* First-time setup: clear everything, pick the pace and direction,
 * place the body and feet, and build frame 0's pose so the very first
 * frame already looks right. */
static void robot_init(Robot *r, int cols, int rows)
{
    memset(r, 0, sizeof *r);

    robot_set_pace(r, WALK_FREQ_DEFAULT);
    r->gait.direction = +1;
    r->gait.phase     = INITIAL_PHASE_RAD;
    r->ui.show_grid   = true;

    anchor_update_from_screen(&r->anchor, rows);

    r->x = (float)(cols * CELL_W) * INITIAL_X_FRAC;

    seed_foot_locks(r);

    compute_pose(r);
}

static void robot_reset(Robot *r, int cols, int rows)
{
    /* hold on to the settings the user picked, then restart the walk */
    float freq  = r->gait.walk_freq;
    int   dir   = r->gait.direction;
    bool  grid  = r->ui.show_grid;

    robot_init(r, cols, rows);

    robot_set_pace(r, freq);
    r->gait.direction = dir;
    r->ui.show_grid   = grid;
}

/* ── §5.8 robot_tick — advance the walk by one frame ── */

/* Where the hips are at the new phase. We need them before the full
 * pose is built, just to figure out where a foot is landing. */
static void compute_provisional_hips(const Robot *r, Vec2 hip_j[2])
{
    float bob = hip_bob_offset(r->gait.phase);
    hip_j[0] = (Vec2){ r->x - HIP_W, r->anchor.base_hip_y + bob };
    hip_j[1] = (Vec2){ r->x + HIP_W, r->anchor.base_hip_y + bob };
}

/* Did this leg just go from swinging to planted this frame? That's the
 * moment the foot first touches the ground. */
static inline bool leg_just_touched_down(float phi_old, float phi_new)
{
    return sinf(phi_old) > 0.0f && sinf(phi_new) <= 0.0f;
}

/* Where a foot lands. At touchdown the knee is straight, so both leg
 * segments point the same way and the foot ends up straight under the
 * thigh's reach. It always lands on the ground line. */
static Vec2 fk_landing_foot(Vec2 hip, float thigh_angle, float ground_y)
{
    float foot_x = hip.x
                 + UPPER_LEG_LEN * sinf(thigh_angle)
                 + LOWER_LEG_LEN * sinf(thigh_angle);
    return (Vec2){ foot_x, ground_y };
}

/* If this leg just touched down, pin its foot where it landed. */
static void leg_detect_touchdown(Robot *r, int i,
                                 float phi_old, Vec2 prov_hip)
{
    float dir       = (float)r->gait.direction;
    float phi_old_l = leg_phase(phi_old,       i);
    float phi_new_l = leg_phase(r->gait.phase, i);

    if (!leg_just_touched_down(phi_old_l, phi_new_l)) return;

    float thigh_angle = -SWING_AMP * cosf(phi_new_l) * dir;
    r->locks.pos[i]   = fk_landing_foot(prov_hip, thigh_angle,
                                        r->anchor.ground_y);
}

/* When the body walks off one edge (and fully out of sight), bring it
 * back in on the other side. The pinned feet move along with it so the
 * legs don't suddenly jump. */
static void wrap_world_position(Robot *r, int cols)
{
    float screen_w = (float)(cols * CELL_W);
    float teleport = screen_w + WRAP_TOTAL_PX;

    if (r->x > screen_w + WRAP_MARGIN_PX) {
        r->x              -= teleport;
        r->locks.pos[0].x -= teleport;
        r->locks.pos[1].x -= teleport;
    }
    if (r->x < -WRAP_MARGIN_PX) {
        r->x              += teleport;
        r->locks.pos[0].x += teleport;
        r->locks.pos[1].x += teleport;
    }
}

/*
 * One frame of walking. Advance the step and the body, handle a resize,
 * pin any foot that just landed, build the new pose, and wrap the body
 * around the screen edge. Skips everything while paused (unless we were
 * asked to step a single frame).
 */
static void robot_tick(Robot *r, float dt, int cols, int rows)
{
    if (r->ui.paused && !r->ui.step_once) return;
    r->ui.step_once = false;

    /* advance the step and the body */
    float phi_old = r->gait.phase;
    float dir     = (float)r->gait.direction;
    r->gait.phase += 2.0f * (float)M_PI * r->gait.walk_freq * dir * dt;
    r->x          += r->gait.walk_speed * dir * dt;

    anchor_update_from_screen(&r->anchor, rows);

    Vec2 prov_hip[2];
    compute_provisional_hips(r, prov_hip);
    for (int i = 0; i < 2; i++)
        leg_detect_touchdown(r, i, phi_old, prov_hip[i]);

    compute_pose(r);

    wrap_world_position(r, cols);
}

/* ── §6 render — draw the bones, the ground, and the HUD ── */

static inline bool in_screen(int r, int c, int rows, int cols)
{
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

/*
 * Pick the ASCII character that best matches a line's direction:
 * '-' for mostly flat, '|' for mostly upright, '/' or '\' for the two
 * diagonals. (Direction has no front or back, so we fold opposite
 * directions together.)
 */
static chtype bone_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx) * (180.0f / (float)M_PI);
    if (ang <    0.0f) ang += 360.0f;
    if (ang >= 180.0f) ang -= 180.0f;
    if (ang <  22.5f || ang >= 157.5f) return '-';
    if (ang <  67.5f )                 return '/';
    if (ang < 112.5f )                 return '|';
    return                                  '\\';
}

/* ── §6.2 draw_bone — draw one limb as a straight line of glyphs ── */

/* Don't bother drawing a limb shorter than this; it would just leave a
 * stray mark when two joints momentarily land on the same spot. */
#define BONE_MIN_LEN_PX  0.5f

/*
 * Draw a straight line of characters from point a to point b, all the
 * same glyph (chosen from the line's overall slope). This is
 * Bresenham's line algorithm (1965): it steps one cell at a time and
 * uses a running tally to decide when to move sideways versus up/down,
 * so it stays on the straightest path using only integer steps.
 */
static void draw_bone(Vec2 a, Vec2 b, int cp, attr_t at,
                      int rows, int cols)
{
    float dxf = b.x - a.x;
    float dyf = b.y - a.y;
    if (sqrtf(dxf*dxf + dyf*dyf) < BONE_MIN_LEN_PX) return;

    chtype glyph = bone_glyph(dxf, dyf);

    /* endpoints as cell row/col, plus how far and which way to step */
    int r0 = px_to_cy(a.y), c0 = px_to_cx(a.x);
    int r1 = px_to_cy(b.y), c1 = px_to_cx(b.x);
    int dr = r1 - r0, dc = c1 - c0;
    int step_r = (r0 < r1) ? 1 : -1;
    int step_c = (c0 < c1) ? 1 : -1;
    int abs_dr = abs(dr), abs_dc = abs(dc);

    /* err is the running tally that keeps us on the straightest path */
    int err = abs_dr - abs_dc;
    int r = r0, c = c0;
    for (;;) {
        if (in_screen(r, c, rows, cols)) {
            attron (COLOR_PAIR(cp) | at);
            mvaddch(r, c, glyph);
            attroff(COLOR_PAIR(cp) | at);
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -abs_dc) { err -= abs_dc; r += step_r; }
        if (e2 <  abs_dr) { err += abs_dr; c += step_c; }
    }
}

/* ── §6.3 render_ground — the floor line and its scrolling ticks ── */

/* The solid floor line. */
static void ground_paint_line(int gy, int cols)
{
    attron (COLOR_PAIR(CP_GROUND) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(gy, c, '_');
    attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);
}

/* How far to shift the ground ticks so they scroll along with the body.
 * The +GRID_PERIOD keeps the result positive when walking backward. */
static int grid_scroll_offset(float x)
{
    int off = (int)(x / (float)CELL_W) % GRID_PERIOD;
    return off < 0 ? off + GRID_PERIOD : off;
}

/* Tick marks just below the floor line, scrolling with the body so the
 * world looks like it's sliding past. */
static void ground_paint_grid(int gy, int cols, float x, int rows)
{
    if (gy + 1 >= rows - 1) return;        /* no room left above the hint strip */
    int offset = grid_scroll_offset(x);

    attron(COLOR_PAIR(CP_GROUND) | A_DIM);
    for (int c = 0; c < cols; c++) {
        chtype ch = ((c + offset) % GRID_PERIOD == 0) ? '|' : ' ';
        mvaddch(gy + 1, c, ch);
    }
    attroff(COLOR_PAIR(CP_GROUND) | A_DIM);
}

static void render_ground(const Robot *r, int rows, int cols)
{
    int gy = px_to_cy(r->anchor.ground_y);
    if (gy < 0 || gy >= rows) return;

    ground_paint_line(gy, cols);
    if (r->ui.show_grid) ground_paint_grid(gy, cols, r->x, rows);
}

/* ── §6.4 render_robot — draw all the body parts ── */

/* Stamp one character at a pixel position, if it's on screen. */
static void put_glyph(Vec2 pos, chtype glyph, int cp, attr_t at,
                      int rows, int cols)
{
    int cx = px_to_cx(pos.x);
    int cy = px_to_cy(pos.y);
    if (!in_screen(cy, cx, rows, cols)) return;
    attron (COLOR_PAIR(cp) | at);
    mvaddch(cy, cx, glyph);
    attroff(COLOR_PAIR(cp) | at);
}

static void paint_spine(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_c, p->torso_top, CP_BODY, A_BOLD, rows, cols);
}

static void paint_arms(const Pose *p, int rows, int cols)
{
    draw_bone(p->shoulder[0], p->hand[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->shoulder[1], p->hand[1], CP_RIGHT, A_BOLD, rows, cols);
}

/* Thigh and shin are drawn as two separate lines so each can pick the
 * glyph that matches its own angle. */
static void paint_legs(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_j[0], p->knee[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->knee[0],  p->foot[0], CP_LEFT,  A_BOLD, rows, cols);
    draw_bone(p->hip_j[1], p->knee[1], CP_RIGHT, A_BOLD, rows, cols);
    draw_bone(p->knee[1],  p->foot[1], CP_RIGHT, A_BOLD, rows, cols);
}

static void paint_hip_band(const Pose *p, int rows, int cols)
{
    draw_bone(p->hip_j[0], p->hip_j[1], CP_BODY_DIM, A_DIM, rows, cols);
}

static void paint_shoulder_band(const Pose *p, int rows, int cols)
{
    draw_bone(p->shoulder[0], p->shoulder[1], CP_BODY_DIM, A_DIM, rows, cols);
}

static void paint_head(const Pose *p, int rows, int cols)
{
    put_glyph(p->head_c, 'O', CP_BODY, A_BOLD, rows, cols);
}

/* A planted foot is a yellow '#', a foot in the air a cyan '*'. We
 * trust the planted flag the walk math already set rather than redoing it. */
static void paint_feet(const Robot *r, int rows, int cols)
{
    for (int i = 0; i < 2; i++) {
        bool   planted = r->locks.on_ground[i];
        int    cp      = planted ? CP_FOOT_PLANT : CP_FOOT_SWING;
        chtype glyph   = planted ? '#'           : '*';
        put_glyph(r->pose.foot[i], glyph, cp, A_BOLD, rows, cols);
    }
}

/* Draw the figure back-to-front: the spine first so the arms and legs
 * paint cleanly over it, then the limbs and connectors, and finally the
 * head and feet on top. */
static void render_robot(const Robot *r, int rows, int cols)
{
    const Pose *p = &r->pose;
    paint_spine         (p,    rows, cols);
    paint_arms          (p,    rows, cols);
    paint_legs          (p,    rows, cols);
    paint_hip_band      (p,    rows, cols);
    paint_shoulder_band (p,    rows, cols);
    paint_head          (p,    rows, cols);
    paint_feet          (r,    rows, cols);
}

/* ── §6.5 render_hud — the status line and the key hints ── */

/* Build the top status line: frame rate, then the live walk numbers,
 * then what each leg is doing. */
static void hud_format_status(const Robot *r, double fps,
                              char *buf, size_t n)
{
    const char *dir_str  = r->gait.direction == 1 ? "→ forward" : "← reverse";
    const char *legL_str = r->locks.on_ground[0] ? "STANCE" : "swing ";
    const char *legR_str = r->locks.on_ground[1] ? "STANCE" : "swing ";
    const char *run_str  = r->ui.paused ? "PAUSED" : "running";

    snprintf(buf, n,
             " %5.1f fps  freq:%.2f Hz  speed:%5.0f px/s  "
             "dir:%s  legL:%s  legR:%s  %s ",
             fps, r->gait.walk_freq, r->gait.walk_speed,
             dir_str, legL_str, legR_str, run_str);
}

/* Paint the status line, padding with spaces to the right edge so the
 * yellow band runs the full width. */
static void hud_paint_status_row(const char *buf, int cols)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    int used = (int)strlen(buf);
    for (int x = used; x < cols; x++) mvaddch(0, x, ' ');
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The bottom strip listing every key you can press. */
static void hud_paint_hint_strip(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reverse  ↑/↓:speed  .:step  g:grid ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_hud(const Robot *r, double fps, int rows, int cols)
{
    char buf[HUD_BUF_LEN];
    hud_format_status   (r, fps, buf, sizeof buf);
    hud_paint_status_row(buf, cols);
    hud_paint_hint_strip(rows);
}

/* ── §6.6 scene_draw — paint a whole frame ── */

static void scene_draw(const Robot *r, double fps, int rows, int cols)
{
    erase();
    render_ground(r, rows, cols);
    render_robot (r, rows, cols);
    render_hud   (r, fps, rows, cols);
}

/* ── §7 screen — bring ncurses up and push frames out ── */

/*
 * Screen — how big the terminal is right now, in character cells.
 *
 *   The one place that answers "how big is the window". Every drawing
 *   function and the wrap-around math read it from here. Only
 *   screen_init() and screen_resize() ever change it, and never in the
 *   middle of a frame, so everything else can treat it as fixed.
 */
typedef struct {
    int cols;                /* width in cells  */
    int rows;                /* height in cells */
} Screen;

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

/* ── §8 scene — timing, signals, and the main loop ── */

/*
 * FrameTimer — the clock-watching the main loop needs.
 *
 *   Two jobs: measure how long the last frame took, so the walk
 *   advances by real wall-clock time; and work out a frame rate to
 *   show that's steady enough to read.
 *
 *   last_ns      — when the previous frame started. We reset it after
 *                  a resize so the stalled frame doesn't look huge.
 *   fps_accum_ns,
 *   fps_frames   — time and frame count piling up toward the next
 *                  frame-rate reading.
 *   fps_display  — the frame rate shown in the HUD. Refreshed about
 *                  twice a second: raw per-frame numbers jump around
 *                  too much to read, so we average over a window.
 *
 *   The measure-and-cap timing idea is from Glenn Fiedler's "Fix Your
 *   Timestep!" (2004). The walk is just sin/cos of the current phase,
 *   so a big time step is harmless — it simply advances the phase more.
 */
typedef struct {
    int64_t last_ns;
    int64_t fps_accum_ns;
    int     fps_frames;
    double  fps_display;
} FrameTimer;

/*
 * Scene — everything the program keeps around, in one struct.
 *
 *   There's a single global one (g_scene) for one reason: signal
 *   handlers can't take arguments, so they need a fixed place to set
 *   the "should quit" and "should resize" flags. Everything else is
 *   passed by pointer so you can see what each function touches.
 *
 *   robot       — the walking figure and its state.
 *   screen      — current window size.
 *   timer       — frame timing and frame-rate.
 *   running     — clear this to quit (a signal or the q/ESC key does).
 *   need_resize — the resize signal sets this; the loop notices next
 *                 frame and re-fits to the new window size.
 *
 *   Startup order (see main): screen first (to learn the size), then
 *   the robot (which needs the size), then seed the timer.
 */
typedef struct {
    Robot                 robot;
    Screen                screen;
    FrameTimer            timer;
    volatile sig_atomic_t running;     /* volatile sig_atomic_t: safe to
                                         * touch from a signal handler */
    volatile sig_atomic_t need_resize;
} Scene;

static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void key_pause_toggle(Robot *r) { r->ui.paused = !r->ui.paused; }

/* Flip direction without resetting the step, so the walk just runs
 * backward from wherever it was. */
static void key_reverse_direction(Robot *r)
{
    r->gait.direction = -r->gait.direction;
}

/* Change the pace through robot_set_pace so the forward speed is
 * recomputed to match. */
static void key_pace_nudge(Robot *r, float delta)
{
    robot_set_pace(r, r->gait.walk_freq + delta);
}

static void key_step_once(Robot *r) { r->ui.step_once = true; }

static void key_grid_toggle(Robot *r) { r->ui.show_grid = !r->ui.show_grid; }

/* Map one keystroke to its action; the switch reads as the keymap. */
static void scene_handle_key(Scene *scene, int ch)
{
    Robot *r = &scene->robot;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:  scene->running = 0;            break;
    case ' ':                                key_pause_toggle(r);          break;
    case 'r': case 'R':                      key_reverse_direction(r);     break;
    case '+': case '=': case KEY_UP:         key_pace_nudge(r, +WALK_FREQ_STEP); break;
    case '-': case '_': case KEY_DOWN:       key_pace_nudge(r, -WALK_FREQ_STEP); break;
    case '.':                                key_step_once(r);             break;
    case 'g': case 'G':                      key_grid_toggle(r);           break;
    default:                                                               break;
    }
}

/* Refit everything to the new window size after a resize. */
static void scene_handle_resize(Scene *scene)
{
    screen_resize(&scene->screen);
    robot_reset(&scene->robot, scene->screen.cols, scene->screen.rows);
    scene->need_resize   = 0;
    scene->timer.last_ns = clock_ns();   /* so the stalled frame doesn't count */
}

/* How long since the last frame, in seconds. Capped at DT_CAP_SEC so a
 * stall doesn't make everything lurch. Also hands back the raw time for
 * the frame-rate counter. */
static float frame_measure_dt(FrameTimer *tm, int64_t now_ns,
                              int64_t *out_dt_ns)
{
    int64_t dt_ns = now_ns - tm->last_ns;
    tm->last_ns   = now_ns;
    *out_dt_ns    = dt_ns;
    float dt = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;
    return dt;
}

/* Add this frame to the running count and, twice a second, work out a
 * fresh frame-rate to display. */
static void frame_tick_fps(FrameTimer *tm, int64_t dt_ns)
{
    tm->fps_accum_ns += dt_ns;
    tm->fps_frames++;
    if (tm->fps_accum_ns >= NS_PER_SEC / 2) {
        tm->fps_display  = (double)tm->fps_frames * 1e9
                         / (double)tm->fps_accum_ns;
        tm->fps_accum_ns = 0;
        tm->fps_frames   = 0;
    }
}

/* Handle every keystroke waiting in the queue. */
static void scene_drain_input(Scene *scene)
{
    int ch;
    while ((ch = getch()) != ERR) scene_handle_key(scene, ch);
}

/* Sleep off whatever's left of this frame's time budget so we hold a
 * steady frame rate no matter how quick the frame was. */
static void frame_cap_to_target_fps(int64_t now_ns, int64_t tick_ns)
{
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(tick_ns - elapsed);
}

/*
 * Set up, then loop every frame: handle a pending resize, measure the
 * elapsed time, read the keyboard, advance the walk, draw, and sleep to
 * hold the frame rate. Setup order matters — the screen comes up first
 * so the robot knows how big the world is.
 */
int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    Scene *scene   = &g_scene;
    scene->running = 1;
    screen_init(&scene->screen);
    robot_init (&scene->robot, scene->screen.cols, scene->screen.rows);
    scene->timer.last_ns = clock_ns();

    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (scene->running) {
        /* resize first, so the rest of the frame sees the new size */
        if (scene->need_resize) scene_handle_resize(scene);

        int64_t now_ns = clock_ns();
        int64_t dt_ns;
        float dt = frame_measure_dt(&scene->timer, now_ns, &dt_ns);
        frame_tick_fps(&scene->timer, dt_ns);

        scene_drain_input(scene);

        robot_tick(&scene->robot, dt,
                   scene->screen.cols, scene->screen.rows);

        scene_draw(&scene->robot, scene->timer.fps_display,
                   scene->screen.rows, scene->screen.cols);
        screen_present();

        frame_cap_to_target_fps(now_ns, TICK_NS);
    }

    screen_free(&scene->screen);
    return 0;
}
