/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diff_drive_robot.c — a two-wheeled robot you steer by setting the
 * two wheels to different speeds (no steering wheel). Drive it with
 * the keyboard and watch it move, like a Roomba or a tank.
 *
 * The math (turning the two wheel speeds into motion, and back) follows
 * Siegwart, Nourbakhsh & Scaramuzza, "Introduction to Autonomous Mobile
 * Robots" (2nd ed.), MIT Press, §3.2.
 * Sister file: robots/walking_robot.c — the same idea with legs.
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

/* ── §1 config — every tunable in one place ──────────────────────── */

enum { TARGET_FPS = 60 };

/*
 * We do all the motion math in "pixels", a finer grid than the
 * character cells: 8 pixels across one cell, 16 down. Terminal cells
 * are about twice as tall as wide, so working in this even grid keeps
 * circles round instead of stretched.
 */
#define CELL_W   8
#define CELL_H  16

/* AXLE_PX: gap between the two wheels (a wider gap means lazier turns).
 * ARROW_PX: length of the heading arrow. VEL_ARROW_PX: length of a
 * wheel's speed arrow when that wheel is going flat out. */
#define AXLE_PX        36.0f
#define ARROW_PX       34.0f
#define VEL_ARROW_PX   28.0f

/* Top speeds, and how quickly a held key ramps toward them.
 * V_DECAY/W_DECAY are the per-second shrink factors once you let go:
 * 1.0 = coast forever, 0.001 = the turn dies away almost at once so a
 * quick steer doesn't leave the robot drifting in a curve. */
#define V_MAX            180.0f
#define W_MAX              3.0f
#define V_RATE          (V_MAX * 12.0f)
#define W_RATE          (W_MAX * 15.0f)
#define V_DECAY_PER_SEC    1.000f
#define W_DECAY_PER_SEC    0.001f

/* We only save every 2nd frame's position into the trail, so the same
 * buffer covers twice as much time on screen. */
enum {
    TRAIL_CAP             = 600,
    TRAIL_SAMPLE_STEP     = 2,
};

/* Longest frame we'll believe. A long pause (window dragged, laptop
 * asleep) would otherwise jump the robot across the screen at once. */
#define DT_CAP_SEC  0.10f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* One fixed colour per thing on screen, so each part is easy to pick
 * out. The last two are the standard HUD colours used across the project. */
enum {
    CP_BODY      = 1,           /* '@' body          white   */
    CP_HEAD,                    /* heading arrow     yellow  */
    CP_WHL_L,                   /* 'L' wheel         green   */
    CP_WHL_R,                   /* 'R' wheel         magenta */
    CP_VEL_FWD,                 /* forward arrow     lime    */
    CP_VEL_REV,                 /* reverse arrow     red     */
    CP_TRAIL_NEW,               /* fresh trail dots  cyan    */
    CP_TRAIL_OLD,               /* aged trail dots   blue    */
    PAIR_HUD,                   /* top status row    yellow  */
    PAIR_HINT,                  /* bottom hint row   cyan    */
};

#define HUD_BUF_LEN  120

/* ── §2 clock — monotonic timer + sleep ──────────────────────────── */

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

/* ── §3 color — bind each pair to a colour ───────────────────────── */

/* The 256-colour branch picks nicer shades; the fallback keeps it
 * working on plain 8-colour terminals. HUD pairs sit on the terminal's
 * own background (-1) so they stay readable over anything drawn behind. */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_BODY,       255, COLOR_BLACK);    /* near-white     */
        init_pair(CP_HEAD,       226, COLOR_BLACK);    /* yellow         */
        init_pair(CP_WHL_L,       46, COLOR_BLACK);    /* green          */
        init_pair(CP_WHL_R,      201, COLOR_BLACK);    /* magenta        */
        init_pair(CP_VEL_FWD,     82, COLOR_BLACK);    /* lime           */
        init_pair(CP_VEL_REV,    196, COLOR_BLACK);    /* red            */
        init_pair(CP_TRAIL_NEW,   51, COLOR_BLACK);    /* cyan           */
        init_pair(CP_TRAIL_OLD,   25, COLOR_BLACK);    /* dim blue       */
        init_pair(PAIR_HUD,      226, -1);             /* yellow on bg   */
        init_pair(PAIR_HINT,      51, -1);             /* cyan on bg     */
    } else {
        init_pair(CP_BODY,     COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HEAD,     COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_WHL_L,    COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_WHL_R,    COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_VEL_FWD,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_VEL_REV,  COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TRAIL_NEW,COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_TRAIL_OLD,COLOR_BLUE,    COLOR_BLACK);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

/* ── §4 coords — convert between pixels and cells ────────────────── */

/* pw/ph give the world size in pixels; px_to_cx/cy turn a pixel
 * position back into a screen cell. This is the only place the two
 * grids meet — everything else stays in pixels. */
static inline int   pw       (int cols)  { return cols * CELL_W; }
static inline int   ph       (int rows)  { return rows * CELL_H; }
static inline int   px_to_cx (float px)  { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cy (float py)  { return (int)floorf(py / (float)CELL_H + 0.5f); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── §5 trail — remember where the robot has been ────────────────── */

/*
 * Trail — the robot's recent positions, drawn as fading dots behind it.
 *
 * It's a ring buffer: a fixed-size set of slots that we keep writing
 * into, looping back to the start when we reach the end so the oldest
 * dot is quietly overwritten by the newest. That way it never grows and
 * we never have to shuffle anything — just write one new spot per sample.
 *
 * Members
 *   px[], py[]  Past positions in pixel space. Kept as two separate
 *               arrays (not pairs) because the drawing loop reads each
 *               axis on its own.
 *   head        Where the next position will be written. Once the
 *               buffer is full, this slot holds the oldest dot.
 *   count       How many slots are filled so far (0 up to TRAIL_CAP,
 *               then it stays full).
 *   skip        Frames counted since the last saved position; once it
 *               hits TRAIL_SAMPLE_STEP we save again and reset it.
 */
typedef struct {
    float px[TRAIL_CAP];
    float py[TRAIL_CAP];
    int   head;
    int   count;
    int   skip;
} Trail;

static void trail_clear(Trail *t)
{
    t->head = t->count = t->skip = 0;
}

static void trail_push(Trail *t, float px, float py)
{
    if (++t->skip < TRAIL_SAMPLE_STEP) return;
    t->skip          = 0;
    t->px[t->head]   = px;
    t->py[t->head]   = py;
    t->head          = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ── §6 robot — state, kinematics, one tick of physics ───────────── */

/*
 * Pose — where the robot is and which way it faces.
 *
 * px, py is its position on screen (in pixels, top-left is 0,0), and
 * theta is its heading in radians. Note the screen's y points DOWN, so
 * theta = 0 faces right and theta = pi/2 faces down — worth remembering,
 * because it flips the sign on the wheel-placement math later.
 *
 * Members
 *   px, py   Position in pixels; kept inside the screen by wrap_position
 *            (drive off one edge, reappear on the opposite one).
 *   theta    Heading in radians, kept in (-pi, pi] so the printed angle
 *            stays sensible after lots of spinning.
 */
typedef struct {
    float px, py, theta;
} Pose;

/*
 * WheelSpeeds — how fast each wheel is turning, in pixels/sec.
 *
 * This is the middle step: the keys set a wanted speed-and-turn, that
 * becomes these two wheel speeds, and these two speeds then move the
 * robot. We keep them around because both the motion math and the
 * green/red speed arrows on screen need them.
 *
 * Members
 *   vL, vR   Left and right wheel speed (pixels/sec). Positive = forward.
 *            Both held within +/-V_MAX.
 */
typedef struct {
    float vL, vR;
} WheelSpeeds;

/*
 * Commands — what the driver is asking for: a forward speed and a turn
 * rate. This is the keyboard's view of things; each frame the robot
 * works out the wheel speeds that satisfy it. Keys ramp these up
 * smoothly so a tap doesn't slam the robot to full speed at once.
 *
 * Members
 *   v_cmd    Wanted forward speed (pixels/sec); W/S keys.
 *   w_cmd    Wanted turn rate (radians/sec); A/D keys.
 *            Both held within their V_MAX / W_MAX limits.
 */
typedef struct {
    float v_cmd, w_cmd;
} Commands;

/*
 * Robot — everything about the robot itself, bundled together. The
 * field order tells the story of one frame: keys fill in cmd, cmd
 * becomes wheels, wheels move the pose, and the new spot goes into the
 * trail. axle is the fixed wheel gap, set once at startup.
 * (Pause isn't here — that's a whole-scene thing, kept on Scene.)
 *
 * Members
 *   pose     Where it is and which way it faces.
 *   wheels   The two current wheel speeds.
 *   cmd      What the driver asked for.
 *   trail    Recent positions, for the fading dot trail.
 *   axle     Distance between the wheels (constant).
 */
typedef struct {
    Pose        pose;
    WheelSpeeds wheels;
    Commands    cmd;
    Trail       trail;
    float       axle;
} Robot;

/*
 * Turn "go forward this fast while turning this hard" into two wheel
 * speeds. To turn, one wheel has to outrun the other: the outer wheel
 * speeds up and the inner one slows by the same amount. That's the
 * whole trick — the turn comes from the difference between the wheels,
 * not a separate steering input. Each wheel is capped at V_MAX; if the
 * ask is too much the robot just moves a bit slower than requested.
 */
static void compute_wheels(Robot *r)
{
    float half = r->axle * 0.5f;
    r->wheels.vL = clampf(r->cmd.v_cmd - r->cmd.w_cmd * half, -V_MAX, V_MAX);
    r->wheels.vR = clampf(r->cmd.v_cmd + r->cmd.w_cmd * half, -V_MAX, V_MAX);
}

/*
 * Move the robot for one frame. Read the two wheel speeds back into a
 * forward speed (their average) and a turn rate (their difference over
 * the axle), then nudge the position and heading by that much times the
 * time elapsed. This straight-line nudge is a touch off on a tight
 * curve, but at 60 fps each step is tiny so the error is far under a
 * pixel. Finally fold the heading back into (-pi, pi] so it doesn't
 * grow without bound over a long spin.
 */
static void step_pose(Robot *r, float dt)
{
    float v     = (r->wheels.vL + r->wheels.vR) * 0.5f;
    float omega = (r->wheels.vR - r->wheels.vL) / r->axle;

    r->pose.px    += v     * cosf(r->pose.theta) * dt;
    r->pose.py    += v     * sinf(r->pose.theta) * dt;
    r->pose.theta += omega * dt;

    /* Wrap heading into (−π, π]. */
    while (r->pose.theta >  (float)M_PI) r->pose.theta -= 2.0f * (float)M_PI;
    while (r->pose.theta < -(float)M_PI) r->pose.theta += 2.0f * (float)M_PI;
}

/* Drive off one edge and reappear on the opposite one, so the robot is
 * always on screen. Not realistic, but handy for a demo. */
static void wrap_position(Robot *r, int wpx, int hpx)
{
    if (r->pose.px <  0.0f)         r->pose.px += (float)wpx;
    if (r->pose.px >= (float)wpx)   r->pose.px -= (float)wpx;
    if (r->pose.py <  0.0f)         r->pose.py += (float)hpx;
    if (r->pose.py >= (float)hpx)   r->pose.py -= (float)hpx;
}

static void robot_init(Robot *r, int wpx, int hpx)
{
    memset(r, 0, sizeof *r);
    r->axle  = AXLE_PX;
    r->pose.px    = (float)wpx * 0.5f;
    r->pose.py    = (float)hpx * 0.5f;
    r->pose.theta = 0.0f;
}

static void robot_reset(Robot *r, int wpx, int hpx)
{
    /* Keep the wheel gap; clear everything else back to the start. */
    float axle = r->axle;
    memset(r, 0, sizeof *r);
    r->axle  = axle;
    r->pose.px    = (float)wpx * 0.5f;
    r->pose.py    = (float)hpx * 0.5f;
    r->pose.theta = 0.0f;
}

/* One frame of motion, start to finish: command -> wheels -> move ->
 * keep on screen -> record the spot. Pausing is the caller's job, so
 * this stays pure motion. */
static void robot_tick(Robot *r, float dt, int wpx, int hpx)
{
    compute_wheels(r);
    step_pose     (r, dt);
    wrap_position (r, wpx, hpx);
    trail_push    (&r->trail, r->pose.px, r->pose.py);
}

/* ── §7 scene — keys to commands, and one tick of the world ──────── */

/*
 * Keys — which keys are pressed this frame, one flag each. Cleared and
 * refilled every frame, so several can be on at once (e.g. forward and
 * left together to drive in a curve).
 *
 * Members
 *   fwd, rev        W/S — speed up / back up.
 *   left, right     A/D — steer.
 *   spin_l, spin_r  Z/E — spin on the spot, full tilt.
 *   stop            Space — halt at once.
 */
typedef struct {
    bool fwd, rev, left, right, spin_l, spin_r, stop;
} Keys;

/* InputState — the input side of the scene; for now just the keys. */
typedef struct {
    Keys keys;
} InputState;

/*
 * World — the play area's size in pixels. Taken from the terminal size
 * minus the two rows the HUD reserves (top and bottom), and recomputed
 * whenever the window resizes.
 *
 * Members
 *   wpx, hpx   Width and height of the world, in pixels.
 */
typedef struct {
    int wpx, hpx;
} World;

/*
 * SimControls — the playback switch. When paused, the world stops
 * moving but drawing carries on, so the screen and HUD stay live.
 *
 * Members
 *   paused   true freezes the motion; toggled by the 'p' key.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — holds everything for one run: the play area, the robot, the
 * current keys, and whether we're paused.
 *
 * Members
 *   world    Size of the play area.
 *   robot    The robot and all its state.
 *   input    This frame's key presses.
 *   sim      Paused or running.
 */
typedef struct {
    World       world;
    Robot       robot;
    InputState  input;
    SimControls sim;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->world.wpx = pw(cols);
    /* Reserve top row for HUD, bottom row for hint strip. */
    s->world.hpx = ph(rows - 2);
    memset(&s->input.keys, 0, sizeof s->input.keys);
    s->sim.paused = false;
    robot_init(&s->robot, s->world.wpx, s->world.hpx);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->world.wpx = pw(cols);
    s->world.hpx = ph(rows - 2);
}

/* Space: stop dead right now. */
static inline void apply_full_stop(Commands *cmd) {
    cmd->v_cmd = 0.0f;
    cmd->w_cmd = 0.0f;
}

/* W/S: while held, ease the speed up or down; once released, let it
 * coast. The decay is raised to the dt power so it fades at the same
 * real-time rate no matter the frame rate. */
static inline void apply_throttle_ramp_or_decay(Commands *cmd, const Keys *k,
                                                 float dt) {
    if (k->fwd) cmd->v_cmd += V_RATE * dt;
    if (k->rev) cmd->v_cmd -= V_RATE * dt;
    if (!k->fwd && !k->rev) cmd->v_cmd *= powf(V_DECAY_PER_SEC, dt);
}

/* A/D: same idea as the throttle, but for the turn rate. */
static inline void apply_turn_ramp_or_decay(Commands *cmd, const Keys *k,
                                             float dt) {
    if (k->right) cmd->w_cmd += W_RATE * dt;
    if (k->left)  cmd->w_cmd -= W_RATE * dt;
    if (!k->right && !k->left) cmd->w_cmd *= powf(W_DECAY_PER_SEC, dt);
}

/* Z/E: snap straight to spinning on the spot at full rate, no ramp. */
static inline void apply_spin_in_place_override(Commands *cmd, const Keys *k) {
    if (k->spin_r) { cmd->v_cmd = 0.0f; cmd->w_cmd =  W_MAX; }
    if (k->spin_l) { cmd->v_cmd = 0.0f; cmd->w_cmd = -W_MAX; }
}

/* Keep both commands within what the robot can actually do. */
static inline void clamp_commands_to_physical_limits(Commands *cmd) {
    cmd->v_cmd = clampf(cmd->v_cmd, -V_MAX, V_MAX);
    cmd->w_cmd = clampf(cmd->w_cmd, -W_MAX, W_MAX);
}

/* Turn this frame's keys into the robot's commands. Order matters:
 * stop and spin run after the gentle ramps so they always win — hit
 * space and the robot halts no matter what else is held. */
static void scene_apply_keys(Scene *s, float dt)
{
    Commands  *cmd = &s->robot.cmd;
    const Keys *k  = &s->input.keys;

    if (k->stop) apply_full_stop(cmd);
    apply_throttle_ramp_or_decay  (cmd, k, dt);
    apply_turn_ramp_or_decay      (cmd, k, dt);
    apply_spin_in_place_override  (cmd, k);
    clamp_commands_to_physical_limits(cmd);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    scene_apply_keys(s, dt);
    robot_tick(&s->robot, dt, s->world.wpx, s->world.hpx);
}

/* ── §8 render — draw the robot, its arrows, the trail, the HUD ──── */

/* On-screen and clear of the two HUD rows (top and bottom)? Only draw
 * if so. */
static inline bool in_bounds(int cx, int cy, int cols, int rows)
{
    return cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1;
}

/* Draw a dotted line from one point to another, stepping along it in
 * pixels and skipping repeats. The dots get heavier toward the end
 * ('.' then 'o' then '0') so the line clearly points from start to tip
 * without needing slanted characters. */
static void draw_line_dotted(float x0, float y0, float x1, float y1,
                             int cp, attr_t extra, int cols, int rows)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.5f) return;

    int   steps   = (int)(len / (float)CELL_W) + 1;
    int   prev_cx = -9999, prev_cy = -9999;

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)(steps > 0 ? steps : 1);
        int   cx = px_to_cx(x0 + dx * t);
        int   cy = px_to_cy(y0 + dy * t);
        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (!in_bounds(cx, cy, cols, rows)) continue;

        chtype ch = (t < 0.40f) ? '.' : (t < 0.75f) ? 'o' : '0';
        mvaddch(cy, cx, ch | (chtype)COLOR_PAIR(cp) | (chtype)extra);
    }
}

/* Pick an arrowhead for a heading. The four straight directions get
 * sharp arrows (> v < ^); the in-between diagonals get a round 'o',
 * since a slash would lie about the angle on these stretched cells. */
static chtype tip_char(float theta)
{
    float deg = fmodf(theta * (180.0f / (float)M_PI) + 360.0f, 360.0f);
    if (deg < 22.5f  || deg >= 337.5f) return '>';   /* east       */
    if (deg < 67.5f )                  return 'o';   /* south-east */
    if (deg < 112.5f)                  return 'v';   /* south      */
    if (deg < 157.5f)                  return 'o';   /* south-west */
    if (deg < 202.5f)                  return '<';   /* west       */
    if (deg < 247.5f)                  return 'o';   /* north-west */
    if (deg < 292.5f)                  return '^';   /* north      */
    return                                    'o';   /* north-east */
}

/* Draw a wheel's speed arrow: it starts at the wheel and reaches out
 * along the heading, longer the faster the wheel turns. Green for
 * forward, red for reverse. A wheel that's basically stopped gets no
 * arrow — the gap tells you it's stopped. */
static void draw_wheel_arrow(float wx_px, float wy_px,
                             float v_wheel, float theta,
                             int cols, int rows)
{
    float alen = fabsf(v_wheel / V_MAX) * VEL_ARROW_PX;
    if (alen < 1.0f) return;

    float sign = (v_wheel >= 0.0f) ? 1.0f : -1.0f;
    float ex   = wx_px + cosf(theta) * sign * alen;
    float ey   = wy_px + sinf(theta) * sign * alen;
    int   cp   = (v_wheel >= 0.0f) ? CP_VEL_FWD : CP_VEL_REV;

    draw_line_dotted(wx_px, wy_px, ex, ey, cp, A_BOLD, cols, rows);

    int eax = px_to_cx(ex), eay = px_to_cy(ey);
    if (in_bounds(eax, eay, cols, rows)) {
        float ta = (v_wheel >= 0.0f) ? theta : theta + (float)M_PI;
        mvaddch(eay, eax, tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
    }
}

/* Draw the trail newest-first, fading each dot by how old it is: the
 * freshest are bright cyan, then they dim, and the oldest drop to a
 * faint blue ':' so the path looks like it's receding. */
static void render_trail(const Robot *r, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = (r->trail.head - 1 - k + TRAIL_CAP) % TRAIL_CAP;
        int tc  = px_to_cx(r->trail.px[idx]);
        int tr  = px_to_cy(r->trail.py[idx]);
        if (!in_bounds(tc, tr, cols, rows)) continue;

        float  age = (float)k / (float)(r->trail.count > 1 ? r->trail.count - 1 : 1);
        int    cp  = (age < 0.35f) ? CP_TRAIL_NEW : CP_TRAIL_OLD;
        attr_t at  = (age < 0.12f) ? A_BOLD       : A_DIM;
        char   ch  = (age < 0.35f) ? '.'          : ':';

        attron (COLOR_PAIR(cp) | at);
        mvaddch(tr, tc, (chtype)ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/*
 * WheelPositions — where the two wheels sit on screen, in pixels. They
 * straddle the body, one out to each side, square across the heading.
 *
 * Members
 *   lx, ly   Left wheel.
 *   rx, ry   Right wheel.
 */
typedef struct {
    float lx, ly;
    float rx, ry;
} WheelPositions;

/* Work out where the wheels are right now. They're bolted to the body,
 * one on each side, so they swing around as the robot turns — hence
 * recomputed each frame from the heading. */
static inline WheelPositions compute_wheel_positions(const Robot *r) {
    float sinT = sinf(r->pose.theta);
    float cosT = cosf(r->pose.theta);
    float half_axle = r->axle * 0.5f;

    WheelPositions wp;
    /* Step out sideways from the body by half the axle to each wheel.
     * The signs look flipped from a textbook because screen y points down. */
    wp.lx = r->pose.px + sinT * half_axle;
    wp.ly = r->pose.py - cosT * half_axle;
    wp.rx = r->pose.px - sinT * half_axle;
    wp.ry = r->pose.py + cosT * half_axle;
    return wp;
}

/* Paint one character at one cell in a given colour, but only if it's
 * on screen. Every per-cell draw goes through here. */
static inline void paint_glyph_at_cell(int cx, int cy, chtype glyph,
                                        int color_pair, attr_t extra_attrs,
                                        int cols, int rows) {
    if (!in_bounds(cx, cy, cols, rows)) return;
    attron (COLOR_PAIR(color_pair) | extra_attrs);
    mvaddch(cy, cx, glyph | (chtype)COLOR_PAIR(color_pair) | (chtype)extra_attrs);
    attroff(COLOR_PAIR(color_pair) | extra_attrs);
}

/* Draw the yellow arrow showing which way the robot faces: a dotted
 * shaft from the body out along the heading, capped with an arrowhead. */
static inline void paint_heading_arrow(const Robot *r, int cols, int rows) {
    float cosT = cosf(r->pose.theta), sinT = sinf(r->pose.theta);
    float tip_x_px = r->pose.px + cosT * ARROW_PX;
    float tip_y_px = r->pose.py + sinT * ARROW_PX;

    draw_line_dotted(r->pose.px, r->pose.py, tip_x_px, tip_y_px,
                     CP_HEAD, A_BOLD, cols, rows);

    int tip_cx = px_to_cx(tip_x_px), tip_cy = px_to_cy(tip_y_px);
    paint_glyph_at_cell(tip_cx, tip_cy, tip_char(r->pose.theta),
                        CP_HEAD, A_BOLD, cols, rows);
}

/* Stamp an 'L' or 'R' on a wheel. */
static inline void paint_wheel_label(float wx_px, float wy_px,
                                      chtype label_glyph, int color_pair,
                                      int cols, int rows) {
    paint_glyph_at_cell(px_to_cx(wx_px), px_to_cy(wy_px),
                        label_glyph, color_pair, A_BOLD, cols, rows);
}

/* Draw the whole robot. Order is back-to-front so nearer things cover
 * farther ones: trail first, then wheel arrows, the heading arrow, the
 * 'L'/'R' labels, and the '@' body last so it's never hidden. */
static void render_robot(const Robot *r, int cols, int rows)
{
    WheelPositions wp = compute_wheel_positions(r);

    render_trail(r, cols, rows);

    draw_wheel_arrow(wp.lx, wp.ly, r->wheels.vL, r->pose.theta, cols, rows);
    draw_wheel_arrow(wp.rx, wp.ry, r->wheels.vR, r->pose.theta, cols, rows);

    paint_heading_arrow(r, cols, rows);

    paint_wheel_label(wp.lx, wp.ly, 'L', CP_WHL_L, cols, rows);
    paint_wheel_label(wp.rx, wp.ry, 'R', CP_WHL_R, cols, rows);

    paint_glyph_at_cell(px_to_cx(r->pose.px), px_to_cy(r->pose.py),
                        '@', CP_BODY, A_BOLD, cols, rows);
}

/* ── §8.7 render_hud — status row up top, key hints along the bottom ── */

/* When the turn rate is this close to zero, the robot is going dead
 * straight, so its turn circle is effectively infinite. We treat
 * anything under this as straight and show "INF" instead of a silly
 * huge radius. (The empty enum is just a placeholder — a C enum can't
 * hold a float, so the actual values are #defines.) */
enum { HUD_FLOAT_NOTES = 0 };
#define OMEGA_INFINITESIMAL_THRESHOLD 1e-3f
#define R_INFINITE_DISPLAY_THRESHOLD  9999.0f
enum { HUD_TOP_ROW = 0 };

/* Forward speed: just the average of the two wheel speeds. */
static inline float body_linear_velocity(const Robot *r) {
    return (r->wheels.vL + r->wheels.vR) * 0.5f;
}

/* Turn rate: how much the wheels differ, spread over the axle. */
static inline float body_angular_velocity(const Robot *r) {
    return (r->wheels.vR - r->wheels.vL) / r->axle;
}

/* Radius of the circle the robot is curving along. Going straight, we
 * hand back a huge number so the HUD can spot it and print "INF". */
static inline float instantaneous_turn_radius_px(float v, float omega) {
    if (fabsf(omega) <= OMEGA_INFINITESIMAL_THRESHOLD) return 1e9f;
    return v / omega;
}

static inline float radians_to_degrees(float radians) {
    return radians * (180.0f / (float)M_PI);
}

/* Print text, then fill the rest of the row with spaces so the row's
 * colour runs to the edge instead of stopping at the text. */
static inline void hud_paint_text_padded(int row, int col, int pair,
                                          const char *text,
                                          int total_cols) {
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    int used = (int)strlen(text);
    for (int x = col + used; x < total_cols; x++)
        mvaddch(row, x, ' ');
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Same, but no padding — the bottom hint strip doesn't need to fill
 * the row. */
static inline void hud_paint_text(int row, int col, int pair,
                                   const char *text) {
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top status line: frame rate, where the robot is and which
 * way it points, its speed and turn rate, the turn radius (or "INF"
 * going straight), both wheel speeds, and whether we're paused. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const Robot *r = &s->robot;
    float v     = body_linear_velocity (r);
    float omega = body_angular_velocity(r);
    float R     = instantaneous_turn_radius_px(v, omega);
    float theta_deg = radians_to_degrees(r->pose.theta);
    const char *state_label = s->sim.paused ? "PAUSED" : "running";

    bool render_R_as_infinity = (fabsf(R) > R_INFINITE_DISPLAY_THRESHOLD);
    if (render_R_as_infinity) {
        snprintf(buf, buflen,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:INF  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->pose.px, r->pose.py, theta_deg, v, omega,
                 r->wheels.vL, r->wheels.vR, state_label);
    } else {
        snprintf(buf, buflen,
                 " %5.1f fps  pose:(%.0f,%.0f) %+6.1f°  v:%+6.1fpx/s  "
                 "ω:%+5.2fr/s  R:%+6.0f  L:%+6.1f R:%+6.1f  %s ",
                 fps, r->pose.px, r->pose.py, theta_deg, v, omega, R,
                 r->wheels.vL, r->wheels.vR, state_label);
    }
}

static void draw_hud_status(const Scene *s, double fps, int cols) {
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    hud_paint_text_padded(HUD_TOP_ROW, 0, PAIR_HUD, buf, cols);
}

static void draw_hud_hint(int rows) {
    static const char *KEY_HINT =
        " q:quit  spc:stop  p:pause  r:reset  "
        "WS:throttle  AD:turn  ZE:spin in place ";
    hud_paint_text(rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* The HUD: a status line on top and a key-hint strip along the bottom. */
static void render_hud(const Scene *s, double fps, int cols, int rows)
{
    draw_hud_status(s, fps, cols);
    draw_hud_hint  (rows);
}

/* Draw one whole frame: clear, robot, HUD. */
static void scene_draw(const Scene *s, double fps, int cols, int rows)
{
    erase();
    render_robot(&s->robot, cols, rows);
    render_hud  (s, fps, cols, rows);
}

/* ── §9 screen — bring ncurses up, tear it down, push frames ─────── */

/*
 * Screen — the terminal's size in character cells. This is the cell
 * view of the world; Scene.world holds the matching pixel view. We
 * keep it in its own little struct so all the ncurses setup/teardown
 * calls share one handle and stay in this section.
 *
 * Members
 *   cols, rows   Terminal width and height in cells. Row 0 is the
 *                status line, the last row is the hints; the robot
 *                lives in the rows between. Refreshed on every resize.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* stop typed keys from breaking up a redraw */
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

/* ── §10 app — signals, resize, and the main loop ────────────────── */

/*
 * FpsCounter — a smoothed frame-rate readout. Measuring per frame would
 * jitter, so it tallies frames over a half-second window and only then
 * updates the number it shows.
 *
 * Members
 *   frame_count, window_ns   Running totals for the current window.
 *   display                  The smoothed rate shown in the HUD.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — the whole program in one struct: the world, the terminal, the
 * fps readout, and two flags the signal handlers flip.
 *
 * It lives in one global (g_app) only so the signal handlers can reach
 * those flags; everything else is passed around by pointer.
 *
 * Members
 *   scene         The simulation.
 *   screen        Terminal size + ncurses handle.
 *   fps           Smoothed frame-rate readout.
 *   running       Cleared on Ctrl-C, kill, or 'q' to end the loop.
 *   need_resize   Set when the window changes size; handled next frame.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Read every key waiting this frame. Held-down keys set their flags;
 * the one-shot keys (quit, pause, reset) act right here. Several can
 * land in the same frame, so flags add up rather than overwrite. */
static void collect_input(App *app)
{
    Keys *k = &app->scene.input.keys;
    memset(k, 0, sizeof *k);

    int ch;
    while ((ch = getch()) != ERR) {
        switch (ch) {
        case 'q': case 'Q': case 27:
            app->running = 0;
            break;

        case 'w': case 'W': case KEY_UP:    k->fwd    = true; break;
        case 's': case 'S': case KEY_DOWN:  k->rev    = true; break;
        case 'a': case 'A': case KEY_LEFT:  k->left   = true; break;
        case 'd': case 'D': case KEY_RIGHT: k->right  = true; break;
        case 'e': case 'E':                 k->spin_r = true; break;
        case 'z': case 'Z':                 k->spin_l = true; break;
        case ' ':                           k->stop   = true; break;

        case 'p': case 'P':
            app->scene.sim.paused = !app->scene.sim.paused;
            break;

        case 'r': case 'R':
            robot_reset(&app->scene.robot,
                        app->scene.world.wpx, app->scene.world.hpx);
            trail_clear(&app->scene.robot.trail);
            break;

        default: break;
        }
    }
}

/* The main loop. Each pass: react to any resize, measure how long the
 * last frame took, read input, move the world, redraw, then sleep to
 * hold a steady frame rate. */
int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* Deal with a resize before anything reads the new size. */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize (&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            last_ns = clock_ns();
        }

        /* How long since last frame, capped so a long stall can't fling
         * the robot across the screen in one jump. */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        collect_input(app);
        scene_tick(&app->scene, dt);
        fps_counter_tick(&app->fps, dt_ns);

        scene_draw(&app->scene, app->fps.display,
                   app->screen.cols, app->screen.rows);
        screen_present();

        /* Sleep off whatever's left of this frame's time budget. */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
