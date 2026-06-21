/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_centipede.c — a 24-segment centipede crawls and swings its legs.
 *
 * The body follows the head's trail (forward kinematics: set the head,
 * the segments fall in behind it). The legs and antennae are driven by
 * sine waves, so a ripple of footsteps runs down the body.
 *
 * Sister files: snake_forward_kinematics.c (same trail-following body),
 *               fk_tentacle_forest.c (same line-drawing renderer).
 * Leg-phase pattern from real centipede gaits: Manton (1952), Full & Tu
 * (1991). Edge-avoidance steering from Reynolds (1999).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra animation/fk_centipede.c \
 *            -o fk_centipede -lncurses -lm
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
    TARGET_FPS       =  60,    /* frame-rate cap (only gates the end-of-frame sleep) */

    HUD_COLS         =  96,    /* width of the status-line buffer */
    FPS_UPDATE_MS    = 500,    /* how often the on-screen fps number refreshes */

    /* ncurses color-pair IDs. Pairs 1..N_PAIRS are the body's tail→head
     * color gradient; 8 and 9 are the fixed HUD/hint colors. */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,
    PAIR_HINT        =   9,

    BODY_SEGS        =  24,    /* number of body segments (24 × 20px ≈ 30 rows long) */

    N_LEGS           =  10,    /* leg pairs, spread along the body */

    ANT_SEGS         =   4,    /* segments in each antenna */

    /* Ring buffer of past head positions. Sized way past the worst case:
     * even at the slowest crawl the body needs ~720 frames of history, so
     * 4096 is generous. */
    TRAIL_CAP        = 4096,

    N_THEMES         =  10,    /* color themes, cycled with `t` */
};

/* SEG_LEN_PX: spacing between body joints, in pixels.
 * DRAW_STEP_PX: how far apart we stamp glyphs when drawing a line; must
 * be smaller than a cell so no cell gets skipped (see draw_line_dense). */
#define SEG_LEN_PX     20.0f
#define DRAW_STEP_PX    5.0f

/*
 * Leg shape. Each leg is two segments: hip→knee→foot, swinging out to
 * the side of the body. All lengths in pixels.
 *
 * SWING_AMP and GAIT_FREQ are deliberately large so the foot moves about
 * a full terminal cell per swing — smaller motion looks jittery because
 * a cell is the smallest thing the terminal can draw.
 */
#define UPPER_LEN      14.0f   /* hip → knee length (px)                */
#define LOWER_LEN      12.0f   /* knee → foot length (px)               */
#define LEG_SPLAY       1.2f   /* how far the leg sticks out sideways (rad) */
#define SWING_AMP       0.5f   /* how far the upper leg swings (rad)    */
#define LOWER_SWING     0.35f  /* how far the lower leg swings (rad)    */
#define LEG_BEND       -0.5f   /* fixed bend at the knee (rad)          */
#define BODY_OFFSET     8.0f   /* sideways distance from spine to hip (px) */

/*
 * Antennae: two short feelers off the head. They wave faster than the
 * legs (ANT_FREQ) so the creature looks alert, and splay forward-and-out
 * by ANT_SPLAY (about ±32°).
 */
#define ANT_SEG_LEN     7.0f
#define ANT_SPLAY       0.55f
#define ANT_AMP         0.45f   /* how far each antenna segment waves (rad) */
#define ANT_FREQ        4.5f

/*
 * Crawl and steering speeds. TURN_AMP/TURN_FREQ make the body weave a
 * gentle S-curve (about ±31° of swing, one full S every ~6.6 s).
 * GAIT_FREQ sets the leg cadence; MOVE_SPEED the crawl speed.
 */
#define GAIT_FREQ       2.5f
#define MOVE_SPEED     65.0f
#define MOVE_SPEED_MIN 10.0f
#define MOVE_SPEED_MAX 300.0f
#define TURN_AMP        0.52f
#define TURN_AMP_MIN    0.0f
#define TURN_AMP_MAX    3.0f
#define TURN_FREQ       0.95f
#define TURN_FREQ_MIN   0.05f
#define TURN_FREQ_MAX   5.0f

/*
 * A "soft fence" that keeps the head on screen. When the head comes
 * within REPEL_MARGIN of an edge it gets a turn-away nudge; the nudge
 * grows the closer it gets (quadratically), so the body curves back
 * smoothly instead of slamming into the wall. REPEL_GAIN is its top
 * strength, set high enough to always beat the normal weaving turn.
 */
#define REPEL_MARGIN    64.0f
#define REPEL_GAIN       4.0f

/* Playback speed multiplier the user controls with [ and ]: 0.25× to 4×. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* How many pixels make up one terminal cell. A cell is taller than it is
 * wide, so pixel space stays square and distance math behaves normally;
 * we squash back to cells only at draw time. */
#define CELL_W   8    /* pixels per column */
#define CELL_H  16    /* pixels per row    */

/* ── §2 clock — monotonic time + sleep ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);   /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                  /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — body-gradient themes plus the fixed HUD colors ── */

/*
 * Theme — one named color look for the body (Amber, Matrix, ...).
 *
 * The body is drawn as a color gradient running tail-to-head, so each
 * theme is just an ordered list of N_PAIRS colors: body[0] is the
 * tail color, body[N_PAIRS-1] the head color. They're stored tail-first
 * because ncurses color pairs are numbered from 1 up, so pair (i+1) maps
 * straight to body[i] in one loop — no index flipping.
 *
 * Every color is kept in the bright half of the 256-color palette; the
 * dark end (cube 16-23, gray 232-239) renders as black on the default
 * background and would make the tail vanish.
 */
typedef struct {
    const char *name;            /* short label shown in the HUD            */
    int         body[N_PAIRS];   /* tail→head colors; pair (i+1) uses body[i] */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* 0 Amber  */ { "Amber",  {130, 136, 142, 148, 154, 160, 196} },
    /* 1 Matrix */ { "Matrix", { 24,  28,  34,  40,  46,  82, 118} },
    /* 2 Fire   */ { "Fire",   { 52,  88, 124, 160, 196, 208, 226} },
    /* 3 Ocean  */ { "Ocean",  { 24,  25,  27,  33,  39,  45,  51} },
    /* 4 Nova   */ { "Nova",   { 54,  93, 129, 165, 201, 213, 225} },
    /* 5 Toxic  */ { "Toxic",  { 58,  64,  70,  76,  82, 118, 154} },
    /* 6 Lava   */ { "Lava",   { 52,  58,  94, 130, 166, 202, 208} },
    /* 7 Ghost  */ { "Ghost",  {240, 244, 246, 250, 252, 254, 231} },
    /* 8 Aurora */ { "Aurora", { 24,  29,  35,  71, 107, 143, 221} },
    /* 9 Neon   */ { "Neon",   { 57,  93, 129, 165, 201, 199, 197} },
};

/* Point the ncurses color pairs at theme idx. Safe to call at runtime:
 * redefining a pair takes effect on the next frame's draws. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *th = &THEMES[idx];

    if (COLORS >= 256) {
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, th->body[i], COLOR_BLACK);
    } else {
        int basic[N_PAIRS] = {
            COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
            COLOR_YELLOW, COLOR_GREEN,  COLOR_GREEN,  COLOR_WHITE
        };
        for (int i = 0; i < N_PAIRS; i++)
            init_pair(i + 1, basic[i], COLOR_BLACK);
    }

    /* HUD colors never change with the theme; -1 background lets them
     * sit cleanly on top of whatever is animating behind them. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ── §4 coords — turn pixel positions into terminal cells ── */

/*
 * The whole simulation works in square pixel space. These two helpers are
 * the single place we convert to terminal cells (rounding to the nearest
 * cell), undoing the fact that a cell is wider in pixels vertically.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the centipede: body, legs, motion ── */

/*
 * Vec2 — a 2-D point or direction, in pixels.
 *
 * x grows to the right, y grows downward (the usual terminal layout).
 * Angles are measured the math way (counterclockwise from +x), so a
 * heading of 0 points right/east and π/2 points down/south — "down"
 * because y is flipped from school graph paper.
 */
typedef struct {
    float x;   /* rightward pixel position  */
    float y;   /* downward pixel position   */
} Vec2;

static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){0.0f, 0.0f};
    return (Vec2){v.x / l, v.y / l};
}

/*
 * Centipede — everything about the one creature on screen.
 *
 * Three parts share this struct:
 *   - the HEAD moves on its own and drops a breadcrumb every frame;
 *   - the BODY follows that breadcrumb trail — this is forward
 *     kinematics: we already know where the head has been, so each
 *     segment just looks up the right past position. (The hard
 *     alternative, inverse kinematics, would instead solve for joint
 *     angles to reach a target; we don't need that here.)
 *   - the LEGS hang off body segments and swing on a shared clock,
 *     offset along the body so a ripple of footsteps runs down it —
 *     the way a real centipede walks (Manton 1952, Full & Tu 1991).
 *
 * It's one struct because the parts read each other's data constantly;
 * splitting them would just mean passing three pointers everywhere.
 */
typedef struct {
    /* ── Trail: the head's breadcrumb history ──────────────────────
     * A ring buffer of recent head positions. trail_push adds the
     * newest; the body reads it in compute_joints to place segments.
     * A ring buffer means adding a point is cheap (no shifting). */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;           /* index of the newest entry             */
    int  trail_count;          /* entries filled so far, caps at TRAIL_CAP */

    /* ── Body joints: where each segment sits this frame ───────────
     * joint[0] is the head; joint[1..BODY_SEGS] are the segments,
     * recomputed every frame by walking back along the trail. */
    Vec2 joint[BODY_SEGS + 1];

    /* ── Legs: hip/knee/foot point for each pair ───────────────────
     * leg_left[i] and leg_right[i] each hold [0]=hip, [1]=knee,
     * [2]=foot. Recomputed every frame from the body + the clock. */
    Vec2 leg_left [N_LEGS][3];
    Vec2 leg_right[N_LEGS][3];

    /* ── Motion: the values stepped forward each frame ─────────────
     * heading is where the head points. wave_time is the shared clock
     * that drives the legs and antennae. turn_phase is kept separate
     * so the user can change turn_freq without the body lurching. */
    float heading;             /* facing direction (rad), 0 = right     */
    float wave_time;           /* shared animation clock (seconds)      */
    float turn_phase;          /* running phase of the weaving turn     */

    /* ── Dials the user can change with keys (reset from §1) ──────── */
    float move_speed;          /* crawl speed (px/s)                    */
    float turn_amp;            /* how hard it weaves (rad/s)            */
    float turn_freq;           /* how fast it weaves (rad/s)            */

    /* ── UI state ──────────────────────────────────────────────────
     * theme_idx only affects color; paused freezes the simulation. */
    int  theme_idx;            /* 0..N_THEMES-1                         */
    bool paused;
} Centipede;

/* ── §5a trail helpers — record and read the head's path ── */

/* Drop a new breadcrumb at the head's position. */
static void trail_push(Centipede *c, Vec2 pos)
{
    c->trail_head = (c->trail_head + 1) % TRAIL_CAP;
    c->trail[c->trail_head] = pos;
    if (c->trail_count < TRAIL_CAP) c->trail_count++;
}

/* Get a past head position: k=0 is the newest, k=1 one older, and so on.
 * The +TRAIL_CAP dodges C giving a negative result from % on small indexes. */
static inline Vec2 trail_at(const Centipede *c, int k)
{
    return c->trail[(c->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * Find the point that is `dist` pixels back along the head's path.
 *
 * Walk the breadcrumbs from the head backward, adding up the distance
 * between them, until the total passes `dist`, then land partway into the
 * last step. Measuring by distance (not by "how many frames ago") keeps
 * the body the same length no matter how fast the head was moving.
 */
static Vec2 trail_sample(const Centipede *c, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(c, 0);   /* current head */

    for (int k = 1; k < c->trail_count; k++) {
        Vec2  b   = trail_at(c, k);
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
    return trail_at(c, c->trail_count - 1);
}

/* ── §5b move the head — weave, avoid the edges, step forward ── */

/*
 * Build a "push away from the walls" vector for the head.
 *
 * For each edge the head is near (within REPEL_MARGIN), add a push
 * pointing inward whose strength grows as it gets closer — gently at the
 * margin, hard at the wall. The result points toward open space; an empty
 * (zero) vector means the head is clear of every edge.
 */
static Vec2 quadratic_repulsion_from_walls(const Centipede *c,
                                           int cols, int rows)
{
    float wpx = (float)cols * (float)CELL_W;
    float hpx = (float)rows * (float)CELL_H;
    float x = c->joint[0].x, y = c->joint[0].y;

    float d_left   = x;
    float d_right  = wpx - x;
    float d_top    = y;
    float d_bottom = hpx - y;

    Vec2 push = { 0.0f, 0.0f };
    if (d_left   < REPEL_MARGIN) { float u = 1.0f - d_left   / REPEL_MARGIN; push.x += u * u; }
    if (d_right  < REPEL_MARGIN) { float u = 1.0f - d_right  / REPEL_MARGIN; push.x -= u * u; }
    if (d_top    < REPEL_MARGIN) { float u = 1.0f - d_top    / REPEL_MARGIN; push.y += u * u; }
    if (d_bottom < REPEL_MARGIN) { float u = 1.0f - d_bottom / REPEL_MARGIN; push.y -= u * u; }
    return push;
}

/*
 * Fold an angle difference into [-π, π] so we always turn the short way.
 * Angles wrap around, so a raw subtraction can say "turn 300°" when the
 * real answer is "turn -60°"; this fixes that.
 */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/*
 * Turn the wall-push vector into a steering amount (how fast to turn).
 * Find the direction toward open space, see how far the head must turn to
 * face it, and scale that by how strong the push is. Returns a turn rate
 * (rad/s) to add to the head's turning; 0 when clear of the walls.
 */
static float edge_bias(const Centipede *c, int cols, int rows)
{
    Vec2 push = quadratic_repulsion_from_walls(c, cols, rows);

    float mag2 = push.x * push.x + push.y * push.y;
    if (mag2 < 1e-8f) return 0.0f;            /* clear of every wall */

    float desired = atan2f(push.y, push.x);
    float diff    = shortest_signed_angle(desired - c->heading);

    float strength = sqrtf(mag2);
    if (strength > 1.0f) strength = 1.0f;
    return diff * REPEL_GAIN * strength;
}

/*
 * Advance the two internal clocks by one frame. wave_time drives the legs
 * and antennae. turn_phase tracks the weaving turn separately so the user
 * can change turn_freq without the body suddenly jerking. (These clocks
 * are the creature's built-in "rhythm generator" — the gait runs on a
 * timer, not on sensing the ground; Manton 1952, Full & Tu 1991.)
 */
static void advance_locomotion_clocks(Centipede *c, float dt)
{
    c->wave_time  += dt;
    c->turn_phase += c->turn_freq * dt;
}

/* The weaving turn: a sine wave on turn_phase, scaled by how hard the
 * user wants it to weave. Returns a turn rate (rad/s). */
static float cpg_turn_rate(const Centipede *c)
{
    return c->turn_amp * sinf(c->turn_phase);
}

/* Turn the head, then slide it forward in the new direction. Turning
 * before moving (rather than after) reads as the natural "steer then go". */
static void integrate_heading_then_position(Centipede *c,
                                            float omega, float dt)
{
    c->heading    += omega * dt;
    c->joint[0].x += c->move_speed * cosf(c->heading) * dt;
    c->joint[0].y += c->move_speed * sinf(c->heading) * dt;
}

/* Hard backstop that pins the head on screen. The soft fence normally
 * handles this, but a window resize can shrink the screen mid-frame and
 * leave the head outside before the fence gets a chance to react. */
static void clamp_head_to_pixel_bounds(Centipede *c, int cols, int rows)
{
    float wpx = (float)cols * (float)CELL_W;
    float hpx = (float)rows * (float)CELL_H;
    if (c->joint[0].x < 0.0f) c->joint[0].x = 0.0f;
    if (c->joint[0].x >= wpx) c->joint[0].x = wpx - 1.0f;
    if (c->joint[0].y < 0.0f) c->joint[0].y = 0.0f;
    if (c->joint[0].y >= hpx) c->joint[0].y = hpx - 1.0f;
}

/*
 * Move the head one frame: tick the clocks, work out how much to turn
 * (the weave plus any edge-avoidance), step the head forward, keep it on
 * screen, and record the new spot so the body can follow.
 */
static void move_head(Centipede *c, float dt, int cols, int rows)
{
    advance_locomotion_clocks(c, dt);

    float omega = cpg_turn_rate(c) + edge_bias(c, cols, rows);
    integrate_heading_then_position(c, omega, dt);

    clamp_head_to_pixel_bounds(c, cols, rows);
    trail_push(c, c->joint[0]);
}

/* ── §5c place the body segments along the trail ── */

/* Put each segment its fixed distance back along the head's path. The
 * head (joint[0]) was already placed by move_head. */
static void compute_joints(Centipede *c)
{
    for (int i = 1; i <= BODY_SEGS; i++)
        c->joint[i] = trail_sample(c, (float)i * SEG_LEN_PX);
}

/* ── §5d place the legs from the body + the gait clock ── */

/* Given a hip and the two leg angles, walk out hip → knee → foot and
 * write the three points. This is forward kinematics: angle in, points out. */
static void leg_fk(Vec2 hip, float upper_ang, float lower_ang,
                   Vec2 out[3])
{
    Vec2 knee = { hip.x + UPPER_LEN * cosf(upper_ang),
                  hip.y + UPPER_LEN * sinf(upper_ang) };
    Vec2 foot = { knee.x + LOWER_LEN * cosf(lower_ang),
                  knee.y + LOWER_LEN * sinf(lower_ang) };
    out[0] = hip;
    out[1] = knee;
    out[2] = foot;
}

/* Which way the body points at segment j. Measured across the segments on
 * either side (j-1 to j+1) so the direction stays steady even when the head
 * is creeping and neighbouring segments nearly overlap. */
static float body_dir_at(const Centipede *c, int j)
{
    Vec2 fwd = vec2_norm((Vec2){
        c->joint[j - 1].x - c->joint[j + 1].x,
        c->joint[j - 1].y - c->joint[j + 1].y
    });
    return atan2f(fwd.y, fwd.x);
}

/*
 * Which body segment leg-pair i hangs from. The pairs are spread evenly
 * over the middle segments; the head and the very tip are skipped so the
 * head stays a clean "feeler" end and the tail tapers off.
 */
static int attachment_joint_for_pair(int i)
{
    return 1 + i * (BODY_SEGS - 2) / (N_LEGS - 1);
}

/*
 * Place the left and right hips by stepping out to each side of the body.
 * "To the side" is the body direction turned 90° (± π/2), one way for the
 * left hip and the other for the right.
 */
static void hips_from_body_normal(Vec2 spine, float body_dir, float offset,
                                  Vec2 *hip_L, Vec2 *hip_R)
{
    const float half_pi = (float)M_PI * 0.5f;
    float n_L = body_dir + half_pi;
    float n_R = body_dir - half_pi;
    hip_L->x = spine.x + offset * cosf(n_L);
    hip_L->y = spine.y + offset * sinf(n_L);
    hip_R->x = spine.x + offset * cosf(n_R);
    hip_R->y = spine.y + offset * sinf(n_R);
}

/*
 * Work out where leg pair i is in its swing right now.
 *
 * Each pair down the body starts a little later than the one in front
 * (the i·π/N term), which makes a ripple of footsteps run head-to-tail,
 * like a real centipede. The right leg is set half a cycle behind the left
 * (the + π), so when one pushes the other is lifting. (Manton 1952;
 * Full & Tu 1991.)
 */
static void metachronal_gait_phases(int i, float wave_time,
                                    float *phi_L, float *phi_R)
{
    const float pi_f = (float)M_PI;
    *phi_L = wave_time * GAIT_FREQ + (float)i * (pi_f / (float)N_LEGS);
    *phi_R = *phi_L + pi_f;
}

/*
 * Turn a swing phase into the two leg angles. The upper leg sticks out to
 * the side and rocks back and forth; the lower leg follows it but lags
 * slightly (the + π/4), giving the natural "knee leads, foot trails" look.
 * `side` is +1 for the left leg, -1 for the right, which mirrors the splay.
 */
static void swing_angles_from_phase(float body_dir, float side, float phi,
                                    float *upper, float *lower)
{
    const float quarter_pi = (float)M_PI * 0.25f;
    *upper = body_dir + side * LEG_SPLAY + SWING_AMP   * sinf(phi);
    *lower = *upper  + LEG_BEND          + LOWER_SWING * sinf(phi + quarter_pi);
}

/*
 * Recompute every leg from scratch this frame. For each pair: find which
 * segment it hangs off, which way that segment points, where the two hips
 * sit, how far along the swing each leg is, the resulting leg angles, and
 * finally the knee and foot points. There's no carried-over leg state —
 * the same body and clock always give the same pose.
 */
static void compute_legs(Centipede *c)
{
    for (int i = 0; i < N_LEGS; i++) {
        int   j        = attachment_joint_for_pair(i);
        float body_dir = body_dir_at(c, j);

        Vec2 hip_L, hip_R;
        hips_from_body_normal(c->joint[j], body_dir, BODY_OFFSET,
                              &hip_L, &hip_R);

        float phi_L, phi_R;
        metachronal_gait_phases(i, c->wave_time, &phi_L, &phi_R);

        float upper_L, lower_L, upper_R, lower_R;
        swing_angles_from_phase(body_dir, +1.0f, phi_L, &upper_L, &lower_L);
        swing_angles_from_phase(body_dir, -1.0f, phi_R, &upper_R, &lower_R);

        leg_fk(hip_L, upper_L, lower_L, c->leg_left [i]);
        leg_fk(hip_R, upper_R, lower_R, c->leg_right[i]);
    }
}

/* ── §5e rendering helpers — pick glyphs and colors ── */

/* Color pair for body segment i: brightest at the head (i=0), dimmest at
 * the tail, blended evenly in between. Clamped so the very tip can't slip
 * out of range. */
static int body_seg_pair(int i)
{
    int p = N_PAIRS - (i * (N_PAIRS - 1)) / (BODY_SEGS - 1);
    if (p < 1)       p = 1;
    if (p > N_PAIRS) p = N_PAIRS;
    return p;
}

/* Brightness for segment i: bold near the head, normal through the
 * middle, dim toward the tail, so the body looks like it fades away. */
static attr_t body_seg_attr(int i)
{
    if (i <     BODY_SEGS / 4) return A_BOLD;
    if (i > 3 * BODY_SEGS / 4) return A_DIM;
    return A_NORMAL;
}

/*
 * Pick the ASCII character that best matches a line's slope: '-' for
 * roughly flat, '|' for roughly vertical, '/' and '\' for the diagonals.
 * Because each of those characters looks the same upside-down, we only
 * care about the angle within a half-turn. dy is flipped first because
 * the terminal's y grows downward, opposite to normal angle math.
 */
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

/* Arrow ('>' '<' '^' 'v') showing which way the head is facing. */
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

/*
 * How many points to step along a line so we don't skip any cell it
 * crosses. We step every DRAW_STEP_PX pixels, which is less than a cell
 * width, so each cell on the line gets hit; the +1 includes the far end.
 */
static int sample_count_for_dense_raster(float length_px)
{
    return (int)ceilf(length_px / DRAW_STEP_PX) + 1;
}

/*
 * Stamp one glyph in the cell that holds pixel (px, py), skipping it if
 * it's off screen or lands in the same cell as the previous stamp.
 *
 * The caller keeps prev_cx/prev_cy across calls so this skip carries over
 * between draw_line_dense calls — that stops a leg drawn as two lines
 * (hip→knee then knee→foot) from stamping the shared knee cell twice.
 */
static void stamp_glyph_at_pixel(WINDOW *w, float px, float py,
                                 chtype glyph,
                                 int pair, attr_t attr,
                                 int cols, int rows,
                                 int *prev_cx, int *prev_cy)
{
    int cx = px_to_cell_x(px);
    int cy = px_to_cell_y(py);

    if (cx == *prev_cx && cy == *prev_cy) return;     /* same cell as last time */
    *prev_cx = cx;
    *prev_cy = cy;

    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, glyph);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * Draw a straight line from a to b out of one repeated slope character.
 * Picks the character once from the line's angle, then stamps it cell by
 * cell along the way (skipping repeats). The caller seeds prev_cx/prev_cy
 * with -9999 the first time so the very first cell always gets drawn.
 * Used to draw the body, legs, and antennae.
 */
static void draw_line_dense(WINDOW *w,
                            Vec2 a, Vec2 b,
                            int pair, attr_t attr,
                            int cols, int rows,
                            int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = sample_count_for_dense_raster(len);

    for (int t = 0; t <= nsteps; t++) {
        float u = (float)t / (float)nsteps;
        stamp_glyph_at_pixel(w, a.x + dx * u, a.y + dy * u,
                             glyph, pair, attr,
                             cols, rows, prev_cx, prev_cy);
    }
}

/* ── §5f draw the centipede body ── */

/* The marker drawn at joint j. The shape changes from head to tail so you
 * can tell at a glance which end is which. */
static chtype node_marker(int j)
{
    if (j == BODY_SEGS)        return (chtype)(unsigned char)'*'; /* tail tip   */
    if (j > 3 * BODY_SEGS / 4) return (chtype)(unsigned char)'.'; /* near tail  */
    if (j >     BODY_SEGS / 4) return (chtype)(unsigned char)'o'; /* mid body   */
    if (j > 0)                 return (chtype)(unsigned char)'O'; /* near head  */
    return                            (chtype)(unsigned char)'@'; /* head joint */
}

/* Draw one leg: the hip→knee→foot line plus a bright marker at the foot. */
static void draw_leg(WINDOW *w, const Vec2 leg[3],
                     int line_pair, chtype foot_glyph,
                     int cols, int rows)
{
    int cx = -9999, cy = -9999;
    draw_line_dense(w, leg[0], leg[1], line_pair, A_DIM, cols, rows, &cx, &cy);
    draw_line_dense(w, leg[1], leg[2], line_pair, A_DIM, cols, rows, &cx, &cy);

    int fx = px_to_cell_x(leg[2].x);
    int fy = px_to_cell_y(leg[2].y);
    if (fx < 0 || fx >= cols || fy < 0 || fy >= rows) return;
    wattron(w, COLOR_PAIR(5) | A_BOLD);
    mvwaddch(w, fy, fx, foot_glyph);
    wattroff(w, COLOR_PAIR(5) | A_BOLD);
}

/* Draw all the legs (left then right). They're drawn first so the body
 * sits on top; left and right use different colors so you can tell them
 * apart where they meet at the hip. */
static void draw_legs(WINDOW *w, const Centipede *c, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg(w, c->leg_left [i], 3, (chtype)(unsigned char)'*', cols, rows);
        draw_leg(w, c->leg_right[i], 4, (chtype)(unsigned char)'x', cols, rows);
    }
}

/* Draw the body as joined line segments, from tail to head. Drawing toward
 * the head last means the head's colors win wherever the body crosses
 * itself in a tight curl. */
static void draw_body_lines(WINDOW *w, const Centipede *c, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = BODY_SEGS - 1; i >= 0; i--) {
        draw_line_dense(w, c->joint[i + 1], c->joint[i],
                        body_seg_pair(i), body_seg_attr(i),
                        cols, rows, &prev_cx, &prev_cy);
    }
}

/* Draw a bright marker at every joint, on top of the body lines. The
 * index is clamped at BODY_SEGS-1 so the extra tail-tip joint just borrows
 * the dimmest body color. */
static void draw_body_nodes(WINDOW *w, const Centipede *c, int cols, int rows)
{
    for (int j = BODY_SEGS; j >= 0; j--) {
        int cx = px_to_cell_x(c->joint[j].x);
        int cy = px_to_cell_y(c->joint[j].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        int    pair  = body_seg_pair(j < BODY_SEGS ? j : BODY_SEGS - 1);
        chtype glyph = node_marker(j);

        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }
}

/*
 * Draw the centipede back to front so the right things end up on top:
 * legs first, then the body lines, then the joint markers. The antennae
 * and head arrow are added afterward by scene_draw.
 */
static void render_centipede(const Centipede *c, WINDOW *w,
                              int cols, int rows)
{
    draw_legs      (w, c, cols, rows);
    draw_body_lines(w, c, cols, rows);
    draw_body_nodes(w, c, cols, rows);
}

/* ── §5g draw the antennae ── */

/*
 * Draw the two feelers off the head. Each is a short chain of segments
 * that waves on its own faster clock. The two are kept half a cycle out of
 * phase (the side*π/2 term) so they sway independently and look like
 * they're feeling around, not flapping in lockstep.
 */
static void render_antennae(const Centipede *c, WINDOW *w, int cols, int rows)
{
    const float pi_f = (float)M_PI;

    for (int i = 0; i < 2; i++) {
        float side = (i == 0) ? +1.0f : -1.0f;
        float base = c->heading + side * ANT_SPLAY;
        Vec2  prev = c->joint[0];
        int   pcx  = -9999, pcy = -9999;

        for (int s = 1; s <= ANT_SEGS; s++) {
            float wobble = ANT_AMP * sinf(c->wave_time * ANT_FREQ
                                          + (float)s * 0.4f
                                          + side * pi_f * 0.5f);
            float ang = base + wobble;
            Vec2  cur = { prev.x + ANT_SEG_LEN * cosf(ang),
                          prev.y + ANT_SEG_LEN * sinf(ang) };
            draw_line_dense(w, prev, cur, 7, A_DIM, cols, rows, &pcx, &pcy);
            prev = cur;
        }
    }
}

/* ── §6 scene — set up, step, and draw the world ── */

/*
 * Scene — the whole world. Here that's just the one centipede, but it's
 * wrapped in a Scene so the main loop (scene_init / scene_tick /
 * scene_draw) looks the same as every other demo in the project, and so a
 * future variant could add things like obstacles or food alongside it.
 */
typedef struct {
    Centipede centipede;       /* the creature: trail, joints, legs, motion */
} Scene;

/* Set the user-adjustable dials to their starting values from §1. These
 * are also what a reset would go back to. */
static void apply_default_tunables(Centipede *c)
{
    c->move_speed = MOVE_SPEED;
    c->turn_amp   = TURN_AMP;
    c->turn_freq  = TURN_FREQ;
    c->theme_idx  = 0;
    c->paused     = false;
}

/*
 * Set the starting clocks and heading so the centipede looks alive
 * immediately. turn_phase starts at π/2 — the peak of the turn sine — so
 * it begins curving on the very first frame instead of crawling straight
 * for a few seconds first. The slight downward-right heading aims its
 * first curve across the screen rather than along an edge.
 */
static void seed_central_pattern_generator(Centipede *c)
{
    c->wave_time  = 0.0f;
    c->turn_phase = (float)M_PI * 0.5f;
    c->heading    = (float)M_PI / 8.0f;
}

/* Start the head a bit left of centre (38% across, halfway down) so its
 * first curve has room to play out before it nears an edge. */
static void place_head_off_centre(Centipede *c, int cols, int rows)
{
    c->joint[0].x = (float)(cols * CELL_W) * 0.38f;
    c->joint[0].y = (float)(rows * CELL_H) * 0.50f;
}

/*
 * Fake a trail of breadcrumbs leading straight back from the head, as if
 * it had been walking in a line to get here. Without this the trail would
 * be empty on frame 1 and the body would start as a single dot and slowly
 * grow out its tail over the next few seconds.
 */
static void prefill_trail_behind_heading(Centipede *c)
{
    float back_x = cosf(c->heading + (float)M_PI);
    float back_y = sinf(c->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        c->trail[k].x = c->joint[0].x + (float)k * back_x;
        c->trail[k].y = c->joint[0].y + (float)k * back_y;
    }
    c->trail_head  = 0;
    c->trail_count = TRAIL_CAP;
}

/*
 * Build the starting state: zero everything, load the default dials, set
 * the clocks/heading/position, fake a trail so the body starts full
 * length, then compute the first body and leg pose ready to draw.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Centipede *c = &sc->centipede;

    apply_default_tunables(c);
    seed_central_pattern_generator(c);
    place_head_off_centre(c, cols, rows);
    prefill_trail_behind_heading(c);

    compute_joints(c);
    compute_legs(c);
}

/*
 * Advance the world by one frame's worth of time (dt seconds). When
 * paused it does nothing, which freezes everything cleanly since nothing
 * carries hidden momentum.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Centipede *c = &sc->centipede;
    if (c->paused) return;
    move_head(c, dt, cols, rows);
    compute_joints(c);
    compute_legs(c);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    const Centipede *c = &sc->centipede;
    render_centipede(c, w, cols, rows);
    render_antennae (c, w, cols, rows);

    /* Head arrow — drawn last so antennae cannot occlude the head. */
    int hx = px_to_cell_x(c->joint[0].x);
    int hy = px_to_cell_y(c->joint[0].y);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        wattron(w, COLOR_PAIR(7) | A_BOLD);
        mvwaddch(w, hy, hx, head_glyph(c->heading));
        wattroff(w, COLOR_PAIR(7) | A_BOLD);
    }
}

/* ── §7 screen — the ncurses display layer ── */

/*
 * Screen — the current terminal size, in character cells. Cached here so
 * the drawing code can take plain (cols, rows) instead of asking ncurses
 * every frame; refreshed only when the window is resized.
 */
typedef struct {
    int cols;                  /* terminal width in cells   */
    int rows;                  /* terminal height in cells  */
} Screen;

/*
 * Put the terminal into animation mode. The one non-obvious call is
 * typeahead(-1): without it, ncurses pauses to check the keyboard while
 * writing the screen, which can tear a frame mid-draw.
 */
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

/* Re-read the terminal size after a resize. The endwin()+refresh() dance
 * is what makes ncurses pick up the new width and height. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * Draw one full frame: clear, draw the centipede, then the status line
 * (top right) and the key hints (bottom). The HUD uses bright bold colors
 * so it stays readable over the animation — never dim it. */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Centipede *c = &sc->centipede;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %.2fx  spd:%.0f  amp:%.2f  freq:%.2f  theme:%s  %s ",
             fps, time_scale,
             c->move_speed, c->turn_amp, c->turn_freq,
             THEMES[c->theme_idx].name,
             c->paused ? "PAUSED" : "crawling");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  ^v:spd  a/d:freq  w/s:amp  t:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Push the staged frame out to the terminal (ncurses sends only the cells
 * that actually changed). */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, input, and the main loop ── */

/*
 * App — everything outside the simulated world: the world itself (scene),
 * the terminal (screen), and the loop's control flags. It lives at file
 * scope (g_app) so the signal handlers, which can't be passed arguments,
 * can flip `running` and `need_resize`.
 *
 * running and need_resize are volatile sig_atomic_t because a signal
 * handler writes them at any moment: volatile forces the loop to re-read
 * them from memory each pass, and sig_atomic_t guarantees the read isn't
 * caught half-written.
 */
typedef struct {
    Scene  scene;              /* the world (§6)            */
    Screen screen;             /* the terminal (§7)        */

    float                 time_scale;   /* playback speed; 1.0 = real time */
    volatile sig_atomic_t running;      /* loop runs while this is set     */
    volatile sig_atomic_t need_resize;  /* a resize is waiting to be handled */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* Safety net registered with atexit: make sure endwin() runs no matter
 * how we exit, so the terminal is always restored. */
static void cleanup(void) { endwin(); }

/*
 * Handle a window resize: pick up the new size and pull the head back
 * inside it; the edge-avoidance will smoothly recenter it from there.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Centipede *c = &app->scene.centipede;
    float wpx    = (float)(app->screen.cols * CELL_W);
    float hpx    = (float)(app->screen.rows * CELL_H);
    if (c->joint[0].x >= wpx) c->joint[0].x = wpx - 1.0f;
    if (c->joint[0].y >= hpx) c->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * Act on one keypress; return false to quit. The letter aliases (a/d,
 * w/s) exist because some terminals eat the arrow-key escape sequences.
 */
static bool app_handle_key(App *app, int ch)
{
    Centipede *c = &app->scene.centipede;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': c->paused = !c->paused; break;

    case 't': case 'T':
        c->theme_idx = (c->theme_idx + 1) % N_THEMES;
        theme_apply(c->theme_idx);
        break;

    case KEY_UP:
        c->move_speed *= 1.25f;
        if (c->move_speed > MOVE_SPEED_MAX) c->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN:
        c->move_speed /= 1.25f;
        if (c->move_speed < MOVE_SPEED_MIN) c->move_speed = MOVE_SPEED_MIN;
        break;

    case KEY_LEFT:  case 'a': case 'A':
        c->turn_freq -= 0.15f;
        if (c->turn_freq < TURN_FREQ_MIN) c->turn_freq = TURN_FREQ_MIN;
        break;
    case KEY_RIGHT: case 'd': case 'D':
        c->turn_freq += 0.15f;
        if (c->turn_freq > TURN_FREQ_MAX) c->turn_freq = TURN_FREQ_MAX;
        break;

    case 'w': case 'W':
        c->turn_amp += 0.10f;
        if (c->turn_amp > TURN_AMP_MAX) c->turn_amp = TURN_AMP_MAX;
        break;
    case 's': case 'S':
        c->turn_amp -= 0.10f;
        if (c->turn_amp < TURN_AMP_MIN) c->turn_amp = TURN_AMP_MIN;
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
 * The main loop. Each frame: read any keys, handle a pending resize,
 * measure how long the last frame took, advance the simulation by that
 * much, update the fps readout, draw, then sleep to hold the frame rate.
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

        /* Read keys first so a press takes effect this same frame. */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();    /* don't count the resize pause as frame time */
        }

        /* How long since the last frame. Capped at 100 ms so a long pause
         * (e.g. laptop sleep) doesn't make the centipede teleport. */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt * app->time_scale,
                   app->screen.cols, app->screen.rows);

        /* fps readout, averaged over a short window */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* Sleep off the rest of the frame's time budget to hold the cap. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
