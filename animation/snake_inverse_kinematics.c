/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snake_inverse_kinematics.c — a snake whose head chases a wandering target.
 *
 * Inverse kinematics: you point the head at a target and the whole body works
 * out where to be to reach it. Here the head takes one small step toward the
 * target each tick; the 32-segment body just follows the head's recorded path.
 * Sister file snake_forward_kinematics.c is the same body with no target — it
 * autopilots a wave instead of chasing. Wander/bounce idea after Reynolds'
 * steering behaviours; the FABRIK iterative IK solver (Aristidou & Lasenby) is
 * the heavyweight alternative this file deliberately does NOT need.
 *
 * Build (needs -lm for the trig in the wander + IK math):
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       snake_inverse_kinematics.c -o snake_ik -lncurses -lm
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

/* ── §1 config — tunables: speeds, wander harmonics, edge bounce, geometry ── */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    N_PAIRS          =   7,   /* gradient color pairs for snake body        */
    PAIR_HUD         =   8,   /* bright yellow — top status bar             */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint            */
    PAIR_GHOST       =  10,   /* dim grey      — wander-target ghost trail  */
    N_THEMES         =  10,

    N_SEGS           =  32,   /* rigid body segments                        */
    TRAIL_CAP        = 4096,  /* circular head-position history capacity    */

    /* How many recent target positions to keep for the faint ghost trail.
     * 200 ticks is roughly a few seconds of path — enough to see the curve. */
    TARGET_TRAIL_CAP = 200,
};

/* Segment and bead step dimensions */
#define SEG_LEN_PX     18.0f   /* pixel length of each rigid body segment   */
#define DRAW_STEP_PX    5.0f   /* bead fill step (larger = sparser beads)   */

/* Head translation speed (px/s) */
#define MOVE_SPEED_DEFAULT  150.0f
#define MOVE_SPEED_MIN       20.0f
#define MOVE_SPEED_MAX      600.0f

/*
 * Wandering-target parameters.
 *
 * The target steers itself: three sine waves are added together and used to
 * keep turning its heading, which traces a smooth, river-like path that never
 * quite repeats. The three frequencies (0.29, 0.71, 1.13) have no common
 * multiple, so the combined wiggle never lines back up.
 *
 *   AMP1/FREQ1 — big slow sweeps.   AMP2/FREQ2 — medium wiggles.
 *   AMP3/FREQ3 — small fast tremors. PHASE2/3 — start the waves out of sync.
 *   SMOOTH_RATE — how fast the snake's chase point catches up to the target
 *                 (higher = tighter chase, lower = lazier corner-rounding).
 */
#define TGT_WANDER_SPEED_DEFAULT  80.0f
#define TGT_WANDER_SPEED_MIN       5.0f
#define TGT_WANDER_SPEED_MAX     500.0f

#define TGT_TURN_AMP1   1.40f   /* wide sweeping curves (rad/s)            */
#define TGT_TURN_FREQ1  0.29f   /* ~21 s period                            */
#define TGT_TURN_AMP2   0.80f   /* medium wiggles                          */
#define TGT_TURN_FREQ2  0.71f   /* ~8.9 s period                           */
#define TGT_TURN_AMP3   0.40f   /* fine tremors                            */
#define TGT_TURN_FREQ3  1.13f   /* ~5.6 s period                           */
#define TGT_TURN_PHASE2 1.10f   /* phase offset for harmonic 2 (radians)   */
#define TGT_TURN_PHASE3 2.40f   /* phase offset for harmonic 3             */

#define TGT_SMOOTH_RATE  8.00f  /* actual_target lerp rate toward tgt_pos  */

/* The target bounces off a wall set this far inside the screen edge, so it
 * (and the head chasing it) never disappear into the corners or behind the
 * HUD bars. 64 px is about 8 columns / 4 rows of breathing room. */
#define EDGE_MARGIN_PX   64.0f

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cell dimensions (physics↔display bridge, see §4) */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic timer + sleep ── */

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

/* ── §3 color — 10 body palettes plus fixed HUD / hint / ghost pairs ── */

/*
 * Theme — one named colour palette for the snake body.
 *
 *   name   — shown in the HUD status bar; t/T cycles through THEMES[].
 *   body[] — 7 xterm-256 colour indices, head (brightest) to tail tip.
 *            Slot 0 colours the head, slots 1..5 the mid-body, slot 6 the
 *            tail. The bright-to-dim ramp lets the eye see which way the
 *            snake is swimming.
 *
 * Every entry stays in the bright half of the palette (cube >= 24, grays
 * >= 240). The tail quarter is drawn with A_DIM, and darker indices than
 * that would vanish on a dark terminal.
 *
 * The HUD, hint, and ghost-trail pairs are NOT in here — they keep fixed
 * bright colours across all themes (set once in color_init).
 */
typedef struct {
    const char *name;            /* theme name shown in the HUD               */
    int         body[N_PAIRS];   /* head->tail colours; pair p uses body[p-1] */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      head ←─────────────────────→ tail */
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Matrix", { 28,  34,  40,  76,  46,  82, 118}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Ocean",  { 24,  25,  31,  33,  39,  45,  51}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Toxic",  { 28,  58,  64,  70,  76,  82, 118}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {244, 245, 247, 249, 251, 253, 255}},
    {"Aurora", { 28,  34,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/*
 * Bind the 7 body colour pairs to theme `idx`.
 * Background -1 means "leave the terminal's own background", so the demo
 * blends with whatever colour scheme the user runs. On terminals without
 * 256 colours, fall back to a coarse warm-to-cool ramp.
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_PAIRS] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
            COLOR_GREEN,  COLOR_CYAN,   COLOR_CYAN, COLOR_BLUE
        };
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * One-time colour setup: turn on colour, allow -1 = terminal default, bind
 * the starting theme, then bind the three fixed pairs (HUD yellow, hint cyan,
 * ghost-trail grey) that never change when the theme cycles.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_GHOST, 244, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_GHOST, COLOR_WHITE,  -1);
    }
}

/* ── §4 coords — pixel space to terminal cells ── */

/*
 * The simulation works in square pixels (same scale on both axes). Terminal
 * cells are taller than wide (CELL_H = 2 x CELL_W), so we divide x and y by
 * different amounts when drawing; that keeps the snake from looking squashed.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — Snake: trail buffer, IK head, bead renderer ── */

/*
 * Vec2 — a point in pixel space.
 *
 * Everything the simulation tracks (target, head, trail, body joints) is a
 * Vec2. Pixels are finer than terminal cells (a cell is 8 x 16 pixels), so
 * the snake moves smoothly instead of jumping cell to cell. The only place
 * pixels turn into cell coordinates is the drawing helpers, via px_to_cell_*.
 *
 *   x : pixels right of the left edge (positive = right).
 *   y : pixels down from the top      (positive = down).
 *
 * Note y points DOWN, the opposite of school graph paper. Headings still use
 * the usual math convention (0 = east, angle grows counter-clockwise on
 * paper), which means it grows clockwise on screen because the y-axis is
 * flipped. The wall-reflection helpers below are written so this flip works
 * out correctly without any special-casing.
 */
typedef struct {
    float x;   /* pixels from left edge (positive = right) */
    float y;   /* pixels from top edge  (positive = down)  */
} Vec2;

/*
 * Snake — all state for the demo. Two things live on screen, both kept here:
 *
 *   1. The TARGET: a dot that drifts around on its own (sine-wave steering)
 *      and bounces off the walls. Nothing chases it back; it just wanders.
 *
 *   2. The SNAKE: its head chases the target; its body trails behind by
 *      following the head's recorded path.
 *
 * Each tick the data flows one way, no loops:
 *
 *   target wanders + bounces      -> tgt_pos       (sharp turns at walls)
 *   smooth it                     -> actual_target (what the head aims at)
 *   head steps toward it          -> joint[0] + heading
 *   record head into trail        -> trail
 *   sample trail at fixed spacing -> joint[1..N_SEGS] (the body)
 *
 * The smoothing step in the middle is the trick that makes it look good: the
 * raw target jerks sideways the instant it hits a wall, so if the head aimed
 * straight at it the snake would visibly snap. Aiming at the smoothed point
 * turns each bounce into a gentle rounded turn instead.
 */
typedef struct {
    /* Trail: a ring of recent head positions. New positions are written at
     * trail_head and wrap around, so the oldest are silently overwritten
     * once full — no shifting of the array. The body is built by walking
     * back along this trail. */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;                 /* index of the newest entry          */
    int  trail_count;                /* entries written so far, caps at TRAIL_CAP */

    /* Body joints. joint[0] is the head (set by the IK step); joint[1..N]
     * are the body and tail, placed by sampling the trail. prev_joint is
     * last tick's pose, kept so drawing can blend between ticks for smooth
     * motion even when the draw rate differs from the physics rate. */
    Vec2 joint     [N_SEGS + 1];
    Vec2 prev_joint[N_SEGS + 1];

    /* Head steering. heading is the direction the head is facing (radians,
     * 0 = east), used to pick its arrow glyph. move_speed is how fast the
     * head chases, in pixels/sec (tuned with the speed keys). */
    float heading;
    float move_speed;

    /* The wandering target. tgt_pos is where it is; tgt_dir is which way
     * it's heading (radians); tgt_speed is its pace (px/s); tgt_time is its
     * own clock, feeding the steering sine waves. tgt_pos jerks at bounces. */
    Vec2  tgt_pos;
    float tgt_time;                  /* target's clock, seconds            */
    float tgt_speed;                 /* target speed, px/s                 */
    float tgt_dir;                   /* target heading, radians            */

    /* The smoothed target the head actually chases. Lags tgt_pos slightly
     * so wall bounces read as rounded turns instead of snaps. */
    Vec2  actual_target;

    /* Faint breadcrumb trail showing where the target has been (recent
     * actual_target positions). Separate, shorter ring than the body trail
     * since a few seconds of dots is all the hint we want. */
    Vec2 tgt_trail[TARGET_TRAIL_CAP];
    int  tgt_head;                   /* index of the newest entry          */
    int  tgt_count;                  /* entries written, caps at TARGET_TRAIL_CAP */

    /* UI state. paused freezes the physics (drawing keeps running);
     * theme_idx picks the colour palette and is cycled with t/T. */
    int  theme_idx;                  /* index into THEMES[]                 */
    bool paused;
} Snake;

/* ── §5a trail helpers — record head positions, read them back by age ── */

/* Add the newest head position to the trail ring. */
static void trail_push(Snake *s, Vec2 pos)
{
    s->trail_head = (s->trail_head + 1) % TRAIL_CAP;
    s->trail[s->trail_head] = pos;
    if (s->trail_count < TRAIL_CAP) s->trail_count++;
}

/* The trail position from k ticks ago (k = 0 is the current head). */
static inline Vec2 trail_at(const Snake *s, int k)
{
    return s->trail[(s->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/* Straight-line distance between two points. */
static inline float polyline_segment_length(Vec2 a, Vec2 b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

/* A point t of the way from a to b (t = 0 is a, t = 1 is b). Lets a body
 * joint land at the exact spacing wanted, not just on the nearest stored
 * trail point, which is what keeps the body looking smooth. */
static inline Vec2 lerp_between_points(Vec2 a, Vec2 b, float t)
{
    return (Vec2){
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
    };
}

/*
 * Find the point `dist` pixels back along the trail, measured by walking the
 * actual path the head took. Step from newest to oldest, adding up segment
 * lengths until the running total passes `dist`, then interpolate inside the
 * segment that crossed it. This spaces body joints evenly along the curve
 * regardless of how fast the head was moving when each point was recorded.
 * If the trail isn't long enough, return its oldest point.
 *
 * The `seg > 1e-4f` guard avoids dividing by zero when two trail points sit
 * on top of each other (the head barely moved).
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);              /* newest = current head */

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);            /* one tick older than a */
        float seg = polyline_segment_length(a, b);

        if (accum + seg >= dist) {             /* target inside [a, b] */
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return lerp_between_points(a, b, t);
        }

        accum += seg;                          /* keep walking back    */
        a      = b;
    }

    return trail_at(s, s->trail_count - 1);    /* trail exhausted      */
}

/* ── §5b move_head — wander the target, then chase it with the head ── */

/* Add a point to the target's faint breadcrumb trail. */
static void tgt_push(Snake *s, Vec2 pos)
{
    s->tgt_head = (s->tgt_head + 1) % TARGET_TRAIL_CAP;
    s->tgt_trail[s->tgt_head] = pos;
    if (s->tgt_count < TARGET_TRAIL_CAP) s->tgt_count++;
}

/* Bounce off a left/right wall: keep the up/down motion, flip left<->right.
 * In heading terms that's dir -> pi - dir. */
static inline float reflect_direction_about_vertical_wall(float dir)
{
    return (float)M_PI - dir;
}

/* Bounce off a top/bottom wall: keep left/right, flip up<->down.
 * In heading terms that's dir -> -dir. */
static inline float reflect_direction_about_horizontal_wall(float dir)
{
    return -dir;
}

/*
 * Keep the target inside its box: if it has crossed a wall this tick, snap
 * its position back onto the wall and flip its heading so it bounces away.
 * The box sits EDGE_MARGIN_PX inside the screen.
 *
 * Snapping the position back matters: at high speed one tick can carry the
 * target well past the wall, and without the snap the next tick would still
 * see it outside, flip again, and leave it shuddering against the wall.
 */
static void bounce_target(Snake *s, float wpx, float hpx)
{
    float m    = EDGE_MARGIN_PX;
    float lo_x = m, hi_x = wpx - m;
    float lo_y = m, hi_y = hpx - m;

    if (s->tgt_pos.x < lo_x) {
        s->tgt_pos.x = lo_x;
        s->tgt_dir   = reflect_direction_about_vertical_wall(s->tgt_dir);
    } else if (s->tgt_pos.x > hi_x) {
        s->tgt_pos.x = hi_x;
        s->tgt_dir   = reflect_direction_about_vertical_wall(s->tgt_dir);
    }
    if (s->tgt_pos.y < lo_y) {
        s->tgt_pos.y = lo_y;
        s->tgt_dir   = reflect_direction_about_horizontal_wall(s->tgt_dir);
    } else if (s->tgt_pos.y > hi_y) {
        s->tgt_pos.y = hi_y;
        s->tgt_dir   = reflect_direction_about_horizontal_wall(s->tgt_dir);
    }
}

/*
 * How fast the target should turn right now (radians/sec). It's three sine
 * waves added together. One sine alone would trace the same loop forever and
 * look mechanical; because these three frequencies share no common multiple,
 * their sum never quite repeats, so the path looks organic and unpredictable.
 */
static inline float multi_harmonic_steering_turn(float t)
{
    return TGT_TURN_AMP1 * sinf(TGT_TURN_FREQ1 * t)
         + TGT_TURN_AMP2 * sinf(TGT_TURN_FREQ2 * t + TGT_TURN_PHASE2)
         + TGT_TURN_AMP3 * sinf(TGT_TURN_FREQ3 * t + TGT_TURN_PHASE3);
}

/* Move the target one tick: turn its heading by `turn`, then walk forward in
 * the new direction at its current speed. Turn first, then step. */
static inline void advance_wander_target(Snake *s, float turn, float dt)
{
    s->tgt_dir += turn * dt;
    s->tgt_pos.x += s->tgt_speed * cosf(s->tgt_dir) * dt;
    s->tgt_pos.y += s->tgt_speed * sinf(s->tgt_dir) * dt;
}

/*
 * Ease `current` a fraction `alpha` of the way toward `raw`. Call this every
 * tick and `current` chases `raw` but lags behind, smoothing out sudden jumps
 * (here: the target's sharp turns at wall bounces). alpha near 0 = very lazy
 * follow, alpha near 1 = snap to raw. Clamped so a big dt can't overshoot.
 */
static inline Vec2 first_order_low_pass(Vec2 current, Vec2 raw, float alpha)
{
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    return (Vec2){
        current.x + (raw.x - current.x) * alpha,
        current.y + (raw.y - current.y) * alpha,
    };
}

/*
 * The whole "inverse kinematics" of this demo, for a single point (the head):
 * face the target and step toward it. Point at it (atan2 gives the heading),
 * then move up to max_step pixels in that direction. The step is capped at
 * the remaining distance so the head lands on the target instead of
 * overshooting and jittering back and forth once it catches up. If it's
 * already basically there (within half a pixel), do nothing — that also keeps
 * the (dx/dist) division from blowing up when distance is zero.
 */
static inline void analytic_one_link_ik_step(Snake *s, Vec2 target,
                                             float max_step)
{
    float dx   = target.x - s->joint[0].x;
    float dy   = target.y - s->joint[0].y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= 0.5f) return;             /* already on the target */

    s->heading = atan2f(dy, dx);
    float step = (max_step < dist) ? max_step : dist;
    s->joint[0].x += (dx / dist) * step;
    s->joint[0].y += (dy / dist) * step;
}

/* Safety net: pin the head inside the screen. The chasing logic normally
 * keeps it there, but one unusually long frame (e.g. resume after suspend)
 * could fling it out before the target redirects it. */
static inline void clamp_head_to_pixel_bounds(Snake *s, float wpx, float hpx)
{
    if (s->joint[0].x < 0.0f) s->joint[0].x = 0.0f;
    if (s->joint[0].x > wpx)  s->joint[0].x = wpx;
    if (s->joint[0].y < 0.0f) s->joint[0].y = 0.0f;
    if (s->joint[0].y > hpx)  s->joint[0].y = hpx;
}

/*
 * One physics tick for the head and its target. In order: let the target
 * wander and bounce off the walls; smooth its position into the point the
 * head will actually aim at; step the head toward that point and pin it
 * on-screen; finally record both into their trails (the body is built from
 * the head trail; the target trail is just the on-screen breadcrumbs).
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    /* Target wanders and bounces */
    s->tgt_time += dt;
    float turn = multi_harmonic_steering_turn(s->tgt_time);
    advance_wander_target(s, turn, dt);
    bounce_target(s, wpx, hpx);

    /* Smooth it so the head doesn't snap at bounces */
    s->actual_target = first_order_low_pass(s->actual_target, s->tgt_pos,
                                            dt * TGT_SMOOTH_RATE);

    /* Head steps toward the smoothed target, then stays on-screen */
    analytic_one_link_ik_step(s, s->actual_target, s->move_speed * dt);
    clamp_head_to_pixel_bounds(s, wpx, hpx);

    /* Record both positions for their trails */
    trail_push(s, s->joint[0]);
    tgt_push  (s, s->actual_target);
}

/* ── §5c compute_joints — place the body along the head's path ── */

/* Put each body joint one segment-length further back along the trail, so the
 * body traces exactly where the head has already been. */
static void compute_joints(Snake *s)
{
    for (int i = 1; i <= N_SEGS; i++)
        s->joint[i] = trail_sample(s, (float)i * SEG_LEN_PX);
}

/* ── §5d render helpers — pick colour, attribute, and glyph per segment ── */

/* Colour pair for segment i, ramping from pair 1 (head) to N_PAIRS (tail). */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/* Brightness for segment i: bold near the head, dim near the tail, so the
 * snake fades out toward its tail. */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_BOLD;
    if (i > 3 * N_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/* The bead character for joint i: big '0' near the head, plain 'o' in the
 * middle, small '.' near the tail. */
static chtype joint_node_char(int i)
{
    if (i <= (N_SEGS - 1) / 3)    return '0';
    if (i >= (N_SEGS - 1) * 2 / 3) return '.';
    return 'o';
}

/* An arrow (> v < ^) pointing whichever way the head is facing. */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)'>';
    if (deg < 135.0f)                  return (chtype)'v';
    if (deg < 225.0f)                  return (chtype)'<';
    return                             (chtype)'^';
}

/* ── §5e mark_cell — draw one glyph at a cell ── */

/* Draw one character at cell (cx, cy) with the given colour and attribute.
 * Off-screen cells are dropped. The double cast keeps a high-bit char from
 * being sign-extended into a garbage glyph. */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f draw_segment_beads — fill the gap between two joints ── */

/* Draw 'o' beads along the line from a to b so neighbouring joints connect
 * into a continuous body. Skips repeats of the same cell to avoid flicker. */
static void draw_segment_beads(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx;  prev_cy = cy;
        mark_cell(w, cx, cy, 'o', pair, attr, cols, rows);
    }
}

/* ── §5g render_chain — compose one frame ── */

/* Blend each joint between last tick's pose and this tick's by alpha, giving
 * the positions to actually draw. This is what keeps motion smooth when the
 * draw rate and physics rate don't line up. */
static void lerp_joints(const Snake *s, float alpha, Vec2 rj[N_SEGS + 1])
{
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = s->prev_joint[i].x
                + (s->joint[i].x - s->prev_joint[i].x) * alpha;
        rj[i].y = s->prev_joint[i].y
                + (s->joint[i].y - s->prev_joint[i].y) * alpha;
    }
}

/* Draw the target's recent path as faint grey dots, oldest first so newer
 * dots win where they overlap. */
static void draw_target_ghost(WINDOW *w, const Snake *s, int cols, int rows)
{
    int n = s->tgt_count;
    for (int k = n - 1; k >= 1; k--) {
        int idx = (s->tgt_head + TARGET_TRAIL_CAP - k) % TARGET_TRAIL_CAP;
        int cx  = px_to_cell_x(s->tgt_trail[idx].x);
        int cy  = px_to_cell_y(s->tgt_trail[idx].y);
        mark_cell(w, cx, cy, '.', PAIR_GHOST, A_DIM, cols, rows);
    }
}

/* Draw a bright '+' where the head is aiming, so you can see what it chases.
 * Drawn before the body so the snake covers it once the head arrives. */
static void draw_target_cursor(WINDOW *w, const Snake *s, int cols, int rows)
{
    int cx = px_to_cell_x(s->actual_target.x);
    int cy = px_to_cell_y(s->actual_target.y);
    mark_cell(w, cx, cy, '+', PAIR_HUD, A_BOLD, cols, rows);
}

/* Fill every segment with beads, tail first so the brighter head colours win
 * wherever the body crosses itself. */
static void draw_body_fill(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                           int cols, int rows)
{
    for (int i = N_SEGS - 1; i >= 0; i--) {
        draw_segment_beads(w,
                           rj[i + 1], rj[i],
                           seg_pair(i), seg_attr(i),
                           cols, rows);
    }
}

/* Stamp the larger '0'/'o'/'.' bead on each joint over the fill, tail first
 * for the same head-wins reason. */
static void draw_body_nodes(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                            int cols, int rows)
{
    for (int i = N_SEGS; i >= 1; i--) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        char ch = (char)joint_node_char(i);
        mark_cell(w, cx, cy, ch,
                  seg_pair(i - 1), seg_attr(i - 1),
                  cols, rows);
    }
}

/* Draw the head arrow, drawn last so it always sits on top of the body. */
static void draw_head(WINDOW *w, const Vec2 *head_pos, float heading,
                      int cols, int rows)
{
    int cx = px_to_cell_x(head_pos->x);
    int cy = px_to_cell_y(head_pos->y);
    char ch = (char)head_glyph(heading);
    mark_cell(w, cx, cy, ch, 1 /* PAIR_HEAD */, A_BOLD, cols, rows);
}

/* Draw one frame back to front: blend joint positions, then the target's
 * ghost trail and cursor, then the body fill and beads, then the head arrow
 * on top. */
static void render_chain(const Snake *s, WINDOW *w,
                          int cols, int rows, float alpha)
{
    Vec2 rj[N_SEGS + 1];
    lerp_joints(s, alpha, rj);

    draw_target_ghost (w, s,         cols, rows);
    draw_target_cursor(w, s,         cols, rows);
    draw_body_fill    (w, rj,        cols, rows);
    draw_body_nodes   (w, rj,        cols, rows);
    draw_head         (w, &rj[0], s->heading, cols, rows);
}

/* ── §6 scene — init / tick / draw orchestration ── */

/*
 * Scene — the whole simulated world. Here that's just the one snake (which
 * already holds its target inside it). It's wrapped in a Scene struct only so
 * the loop (scene_init / scene_tick / scene_draw) looks the same as every
 * other demo in the project; a future variant could add obstacles or more
 * snakes here without touching the loop.
 */
typedef struct {
    Snake snake;               /* the snake and its target */
} Scene;

/*
 * Set up (or reset) the snake. Head starts at screen centre; the target
 * starts on top of it so there's no jump on the first frame, then drifts off
 * on its own. The trail is pre-filled with a straight tail behind the head so
 * the body is the right length immediately. The current theme is kept across
 * a reset (r/R shouldn't jump back to theme 0).
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->snake.theme_idx;
    memset(sc, 0, sizeof *sc);
    Snake *s = &sc->snake;
    s->theme_idx = saved_theme;

    s->move_speed = MOVE_SPEED_DEFAULT;
    s->tgt_speed  = TGT_WANDER_SPEED_DEFAULT;
    s->tgt_time   = 0.0f;
    s->tgt_dir    = (float)M_PI / 6.0f;   /* start the target heading down-right */
    s->heading    = 0.0f;
    s->paused     = false;

    /* Head at screen centre */
    s->joint[0].x = (float)(cols * CELL_W) * 0.5f;
    s->joint[0].y = (float)(rows * CELL_H) * 0.5f;

    /* Target starts exactly on the head so nothing jumps on frame one */
    s->tgt_pos       = s->joint[0];
    s->actual_target = s->joint[0];

    /* Lay a straight starting trail running left (west) from the head, one
     * pixel apart, so the body has somewhere to sit before the head moves. */
    float bx = -1.0f;   /* step west */
    float by =  0.0f;
    for (int k = 0; k < TRAIL_CAP; k++) {
        s->trail[k].x = s->joint[0].x + (float)k * bx;
        s->trail[k].y = s->joint[0].y + (float)k * by;
    }
    s->trail_head  = 0;
    s->trail_count = TRAIL_CAP;

    compute_joints(s);
    memcpy(s->prev_joint, s->joint, sizeof s->joint);

    /* Seed target ghost trail with head position */
    for (int k = 0; k < TARGET_TRAIL_CAP; k++)
        s->tgt_trail[k] = s->joint[0];
    s->tgt_head  = 0;
    s->tgt_count = TARGET_TRAIL_CAP;
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Snake *s = &sc->snake;
    memcpy(s->prev_joint, s->joint, sizeof s->joint);   /* keep last pose to blend from */
    if (s->paused) return;
    move_head(s, dt, cols, rows);
    compute_joints(s);
}

static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_chain(&sc->snake, w, cols, rows, alpha);
}

/* ── §7 screen — ncurses double-buffered display layer ── */

/*
 * Screen — the terminal size in character cells. Cached so drawing code can
 * read cols/rows as plain ints; refreshed only when the terminal is resized.
 */
typedef struct {
    int cols;   /* terminal width  in cells */
    int rows;   /* terminal height in cells */
} Screen;

/* Start ncurses: no echo, no line buffering, hidden cursor, non-blocking
 * input, arrow keys decoded. typeahead(-1) stops ncurses peeking at input
 * mid-draw, which would otherwise tear the picture. */
static void screen_init(Screen *s, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* Re-read the terminal size after a resize. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Draw the whole frame: the snake, the status line top-right, and the key
 * hint along the bottom. */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    const Snake *sn = &sc->snake;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  spd:%.0f  tgt:%.0f  [%s]  %s ",
             fps, sim_fps,
             sn->move_speed,
             sn->tgt_speed,
             THEMES[sn->theme_idx].name,
             sn->paused ? "PAUSED " : "chasing");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  UD/ws:spd  +/-:tgt-spd  t/T:theme  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, main game loop ── */

/*
 * App — everything the program owns: the world, the terminal, and the loop
 * flags. It's a single file-scope global (g_app) so the signal handlers,
 * which take no argument, can set running/need_resize.
 *
 * running and need_resize are volatile sig_atomic_t because a signal handler
 * writes them at any moment: volatile forces the loop to re-read them each
 * pass, and sig_atomic_t guarantees the read/write can't be seen half-done.
 */
typedef struct {
    Scene  scene;              /* the world (§6)            */
    Screen screen;             /* the terminal (§7)         */

    int    sim_fps;            /* physics ticks per second  */

    volatile sig_atomic_t running;      /* cleared to make the loop exit  */
    volatile sig_atomic_t need_resize;  /* set when the terminal resizes  */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Handle a terminal resize: re-read the size and pull the head back inside if
 * the window shrank under it. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Snake *s   = &app->scene.snake;
    float  wpx = (float)(app->screen.cols * CELL_W);
    float  hpx = (float)(app->screen.rows * CELL_H);
    if (s->joint[0].x >= wpx) s->joint[0].x = wpx - 1.0f;
    if (s->joint[0].y >= hpx) s->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * Handle one keypress; return false only on quit. Keys:
 *   q / Q / ESC   quit
 *   space         pause / resume
 *   up / w        head faster        down / s   head slower
 *   + / =         target faster      -          target slower
 *   t / T         next / previous theme
 *   r / R         reset (theme kept)
 *   ] / [         physics rate up / down
 * Speeds are scaled by a fixed factor each press and clamped to their limits.
 */
static bool app_handle_key(App *app, int ch)
{
    Snake *s = &app->scene.snake;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case KEY_UP: case 'w': case 'W':
        s->move_speed *= 1.20f;
        if (s->move_speed > MOVE_SPEED_MAX) s->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN: case 's': case 'S':
        s->move_speed /= 1.20f;
        if (s->move_speed < MOVE_SPEED_MIN) s->move_speed = MOVE_SPEED_MIN;
        break;

    case '+': case '=':
        s->tgt_speed *= 1.25f;
        if (s->tgt_speed > TGT_WANDER_SPEED_MAX) s->tgt_speed = TGT_WANDER_SPEED_MAX;
        break;
    case '-':
        s->tgt_speed /= 1.25f;
        if (s->tgt_speed < TGT_WANDER_SPEED_MIN) s->tgt_speed = TGT_WANDER_SPEED_MIN;
        break;

    case 't':
        s->theme_idx = (s->theme_idx + 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;
    case 'T':
        s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
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

/* ── main — the game loop: step physics on a fixed clock, draw, read keys ── */
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

    screen_init(&app->screen, 0);   /* theme 0 = Medusa on startup */
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② time since last frame, capped so a long stall (e.g. the
         *    program was paused by the OS) can't trigger a flood of
         *    catch-up ticks ──────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ run physics in fixed-size ticks: bank the elapsed time and
         *    spend it one tick at a time, so the simulation behaves the
         *    same regardless of frame rate ──────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ how far we are into the next tick (0..1), used to blend
         *    the drawn pose between ticks ────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain input ───────────────────────────────────────── */
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
