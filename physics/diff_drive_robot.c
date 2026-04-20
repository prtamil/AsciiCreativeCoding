/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diff_drive_robot.c — Differential Drive Robot Simulator
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  KINEMATICS OVERVIEW
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A differential drive robot has two driven wheels on a shared axle.
 *  Steering is achieved by spinning the wheels at different speeds.
 *
 *  State:  pose = (x, y, θ)   in pixel space (y-down, θ=0 → east)
 *  Inputs: vL, vR             individual wheel speeds [px/s]
 *
 *  Kinematics:
 *    v  = (vL + vR) / 2         linear velocity of body centre
 *    ω  = (vR − vL) / L         angular velocity  (L = axle width)
 *    ẋ  = v · cos θ
 *    ẏ  = v · sin θ             (y-down: positive θ rotates clockwise)
 *    θ̇  = ω
 *
 *  Key behaviours:
 *    vL = vR  → ω = 0 → straight line (R = v/ω → ∞)
 *    vL = −vR → v = 0 → spin in place (R = 0)
 *    vL = 0   → pivot about left wheel (R = L/2)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  NONHOLONOMIC CONSTRAINT
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The robot cannot slide sideways.  In vector form:
 *    ẋ·sin θ − ẏ·cos θ = 0    (lateral velocity = 0)
 *
 *  This constraint is automatically satisfied by the kinematic equations
 *  above: velocity is always (v·cosθ, v·sinθ) — along the heading.
 *  No penalty term or projection step is needed.
 *
 *  Compare to a holonomic robot (e.g. mecanum/omni wheels), which CAN
 *  move sideways — its state equations have no such restriction.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  INSTANTANEOUS CENTRE OF CURVATURE (ICC)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  When ω ≠ 0 the robot follows a circular arc.  The ICC lies on the
 *  axle extension at distance R from the body centre:
 *
 *    R = v/ω = (L/2)·(vR+vL)/(vR−vL)
 *
 *  ICC is always perpendicular to the heading direction and on the side
 *  of the slower wheel.  This gives a geometric intuition: "the slow
 *  wheel is the pivot, the fast wheel is the outside of the curve."
 *
 *  Exact arc integration (ICC method) for a step dt:
 *    θ_new = θ + ω·dt
 *    x_new = x + R·(sin θ_new − sin θ)
 *    y_new = y − R·(cos θ_new − cos θ)
 *
 *  We use simpler Euler integration instead — at 60 Hz the arc error
 *  per step is O((ω·dt)²) ≈ 0.025%·L, invisible at terminal resolution.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  COORDINATE SYSTEM
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Physics lives in PIXEL SPACE (CELL_W=8, CELL_H=16 per terminal cell).
 *  This decouples the simulation from terminal size and handles the 2:1
 *  cell aspect ratio automatically — only `px_to_cx/cy` bridges spaces.
 *
 *  y-down convention (standard screen layout):
 *    θ = 0    → faces right (+x)
 *    θ = π/2  → faces down  (+y)
 *    ω > 0    → clockwise rotation (rightward turn)
 *
 *  Perpendicular to heading in y-down:
 *    left  = (+sinθ, −cosθ)    [north of body when heading east]
 *    right = (−sinθ, +cosθ)    [south of body when heading east]
 *
 *  Common mistake: using math-convention perpendicular (−sinθ, cosθ)
 *  which is left in y-up but RIGHT in y-down.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  VISUALIZATION DESIGN
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  '@' body — robot centre
 *  'L'/'R'  — wheel positions, color-coded (green/magenta)
 *  Yellow heading arrow — direction the robot will travel next
 *  Green/red wheel velocity arrows — green=forward, red=reverse;
 *    length proportional to |vL| or |vR|; drawn with '.'→'o'→'0'
 *    progression (no ugly Bresenham diagonals)
 *  Trail — recent '.' bright, older ':' dim; ages via ring buffer
 *
 *  HUD: x, y, heading, v, ω, R (turn radius), vL, vR, fps
 *
 * Controls:
 *   W/↑  forward      S/↓  brake (stop v_cmd)
 *   A/←  turn left    D/→  turn right
 *   Z    spin left     E    spin right
 *   Space  full stop   p    pause/resume
 *   r    reset         q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/diff_drive_robot.c \
 *       -o diff_drive_robot -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config   §2 clock   §3 color
 *  §4 coords   §5 trail   §6 robot   §7 scene
 *  §8 render   §9 screen  §10 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define SIM_FPS      60
#define TARGET_FPS   60

/*
 * Pixel grid — terminal cells are roughly 2× taller than wide.
 * CELL_W and CELL_H are the nominal pixel dimensions of one cell.
 * All physics coordinates are in pixels; only px_to_cx/cy converts.
 * This keeps all velocity and distance arithmetic in consistent units
 * regardless of terminal size or aspect ratio.
 */
#define CELL_W        8
#define CELL_H       16

/* Robot geometry in pixel space */
#define AXLE_PX      36.0f   /* wheel separation: roughly 4.5 cells wide  */
#define ARROW_PX     34.0f   /* heading arrow length                       */
#define VEL_ARROW_PX 28.0f   /* wheel velocity arrow at full V_MAX         */

/*
 * Dynamics parameters.
 *
 * V_DECAY = 1.00: no automatic linear slowdown.  A real wheeled robot
 *   on a flat surface has negligible rolling resistance — it keeps rolling
 *   until braked.  Setting this < 1.0 (e.g. 0.82) at 60 Hz gives
 *   0.82^60 ≈ 0.00013 after 1 s — essentially an instant stop which
 *   feels wrong and makes fine speed control impossible.
 *
 * W_DECAY = 0.88: angular velocity decays quickly when no turn key is
 *   held.  0.88^60 ≈ 0.0006, so turning stops within ~1 s.  This
 *   prevents the robot spinning endlessly while driving straight.
 */
#define V_MAX       180.0f   /* max linear speed  [px/s]                   */
#define W_MAX         3.0f   /* max angular speed [rad/s]                  */
#define V_ACCEL       0.20f  /* forward key: v_cmd += V_MAX × V_ACCEL/tick */
#define W_ACCEL       0.25f  /* turn key:    w_cmd += W_MAX × W_ACCEL/tick */
#define V_DECAY       1.00f  /* linear persistence (1.0 = no friction)     */
#define W_DECAY       0.88f  /* angular damping per tick                   */

/* Trail ring buffer size and sampling rate */
#define TRAIL_CAP   600
#define TRAIL_STEP    2      /* record position every N physics ticks       */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/*
 * Color pairs — role-named.
 * Add a new visual element? Add a new CP_ name here; never use a raw number.
 */
#define CP_BODY      1   /* robot body '@'                  — white        */
#define CP_HEAD      2   /* heading arrow                   — yellow       */
#define CP_WHL_L     3   /* left  wheel 'L'                 — green        */
#define CP_WHL_R     4   /* right wheel 'R'                 — magenta      */
#define CP_TR_NEW    5   /* fresh trail dots                — cyan         */
#define CP_TR_OLD    6   /* aged  trail dots                — dim blue     */
#define CP_HUD       7   /* HUD text                        — cyan         */
#define CP_VEL_P     8   /* velocity arrow, forward         — lime green   */
#define CP_VEL_N     9   /* velocity arrow, reverse         — red          */

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_BODY,  255, COLOR_BLACK);   /* bright white */
        init_pair(CP_HEAD,  226, COLOR_BLACK);   /* yellow       */
        init_pair(CP_WHL_L,  46, COLOR_BLACK);   /* bright green */
        init_pair(CP_WHL_R, 201, COLOR_BLACK);   /* magenta      */
        init_pair(CP_TR_NEW, 51, COLOR_BLACK);   /* cyan         */
        init_pair(CP_TR_OLD, 17, COLOR_BLACK);   /* dark blue    */
        init_pair(CP_HUD,    51, COLOR_BLACK);   /* cyan         */
        init_pair(CP_VEL_P,  82, COLOR_BLACK);   /* lime green   */
        init_pair(CP_VEL_N, 196, COLOR_BLACK);   /* red          */
    } else {
        init_pair(CP_BODY,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HEAD,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_WHL_L, COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_WHL_R, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_TR_NEW,COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_TR_OLD,COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_HUD,   COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_VEL_P, COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_VEL_N, COLOR_RED,     COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is corrected                  */
/* ===================================================================== */

/*
 * pw/ph — world size in pixels from terminal cell count.
 * Physics always uses pixel coordinates; these set the world boundary.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * px_to_cx / px_to_cy — pixel → cell coordinate conversion.
 *
 * Rounding with +0.5 gives round-half-up, which places the entity in
 * the cell whose centre is nearest to the pixel position.
 * This is the ONLY place CELL_W/CELL_H appear in rendering code.
 */
static inline int px_to_cx(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cy(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  trail — ring buffer of past centre positions                       */
/* ===================================================================== */

/*
 * Trail stores the last TRAIL_CAP positions, sampled every TRAIL_STEP
 * physics ticks.  The ring buffer wastes no memory on reallocation and
 * has O(1) push and O(N) traversal.
 *
 * head: index of the NEXT write slot (oldest entry is at head itself
 *   once the buffer is full, but we only read up to `count` entries).
 * skip: countdown to next sample; resets to 0 when TRAIL_STEP reached.
 */
typedef struct {
    float px[TRAIL_CAP];
    float py[TRAIL_CAP];
    int   head, count, skip;
} Trail;

static void trail_push(Trail *t, float px, float py)
{
    if (++t->skip < TRAIL_STEP) return;
    t->skip       = 0;
    t->px[t->head] = px;
    t->py[t->head] = py;
    t->head        = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §6  robot — physics state and integration                             */
/* ===================================================================== */

/*
 * Robot — full state of the differential drive robot.
 *
 * (px, py, theta): pose in pixel space.
 *   theta = 0 → east (+x),  theta = π/2 → south (+y) in y-down.
 *
 * (vL, vR): actual wheel speeds derived each tick from (v_cmd, w_cmd).
 *   They are the "current state" of the drive system.
 *
 * (v_cmd, w_cmd): control commands set by keyboard input.
 *   v_cmd: desired linear speed  (positive = forward)
 *   w_cmd: desired angular speed (positive = clockwise = right turn)
 *
 * (prev_px, prev_py, prev_theta): pose one tick ago, used for
 *   sub-tick render interpolation (alpha lerp).
 */
typedef struct {
    float px, py, theta;
    float vL, vR;
    float v_cmd, w_cmd;
    float axle;
    float prev_px, prev_py, prev_theta;
    Trail trail;
    bool  paused;
} Robot;

/*
 * integrate_motion — Euler integration of robot pose from wheel speeds.
 *
 * Euler approximation at 60 Hz:
 *   Arc error per step = O((ω·dt)²).  For ω_max = 3 rad/s and dt = 1/60:
 *   error ≈ (3/60)² = 0.0025 px on a ~36 px axle, i.e. 0.007% — invisible.
 *
 * For larger dt or higher ω use the exact ICC arc:
 *   R     = v / ω
 *   θ_new = θ + ω·dt
 *   x_new = x + R·(sin θ_new − sin θ)
 *   y_new = y − R·(cos θ_new − cos θ)
 * This is exact for constant v, ω over the interval.
 */
static void integrate_motion(Robot *r, float dt)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;

    /* Save current pose before update (needed by render interpolation) */
    r->prev_px    = r->px;
    r->prev_py    = r->py;
    r->prev_theta = r->theta;

    r->px    += v * cosf(r->theta) * dt;
    r->py    += v * sinf(r->theta) * dt;
    r->theta += omega * dt;

    /* Wrap heading to (-π, π] to keep atan2 / lerp well-behaved */
    while (r->theta >  (float)M_PI) r->theta -= 2.0f * (float)M_PI;
    while (r->theta < -(float)M_PI) r->theta += 2.0f * (float)M_PI;
}

/*
 * map_commands_to_wheels — convert (v_cmd, w_cmd) to individual wheel speeds.
 *
 * Derivation: for an axle of width L and body centre velocity v, angular ω:
 *   vL = v − ω·L/2    (left wheel travels smaller arc on a right turn)
 *   vR = v + ω·L/2    (right wheel travels larger arc)
 *
 * Clamp each wheel independently to ±V_MAX so neither motor saturates.
 * Note: this means actual v and ω may be less than commanded if a wheel
 * would otherwise exceed its limit.
 */
static void map_commands_to_wheels(Robot *r)
{
    float half = r->axle * 0.5f;
    r->vL = r->v_cmd - r->w_cmd * half;
    r->vR = r->v_cmd + r->w_cmd * half;
    if (r->vL >  V_MAX) r->vL =  V_MAX;
    if (r->vL < -V_MAX) r->vL = -V_MAX;
    if (r->vR >  V_MAX) r->vR =  V_MAX;
    if (r->vR < -V_MAX) r->vR = -V_MAX;
}

/*
 * wrap_position — toroidal boundary: robot reappears on the opposite side.
 *
 * This is not physically realistic but keeps the robot visible at all
 * times — useful for demonstrating continuous trajectories (spirals,
 * figure-eights) without hitting an invisible wall.
 */
static void wrap_position(Robot *r, int wpx, int hpx)
{
    float wx = (float)wpx, wy = (float)hpx;
    if (r->px <  0.0f) r->px += wx;
    if (r->px >= wx)   r->px -= wx;
    if (r->py <  0.0f) r->py += wy;
    if (r->py >= wy)   r->py -= wy;
}

static void robot_init(Robot *r, int wpx, int hpx)
{
    memset(r, 0, sizeof *r);
    r->axle = AXLE_PX;
    r->px   = (float)wpx * 0.5f;
    r->py   = (float)hpx * 0.5f;
    r->theta = 0.0f;
}

static void robot_reset(Robot *r, int wpx, int hpx)
{
    float axle = r->axle;      /* preserve geometry across reset */
    memset(r, 0, sizeof *r);
    r->axle  = axle;
    r->px    = (float)wpx * 0.5f;
    r->py    = (float)hpx * 0.5f;
    r->theta = 0.0f;
}

/* robot_step — one physics tick: apply commands → integrate → wrap → trail */
static void robot_step(Robot *r, float dt, int wpx, int hpx)
{
    map_commands_to_wheels(r);
    integrate_motion(r, dt);
    wrap_position(r, wpx, hpx);
    trail_push(&r->trail, r->px, r->py);
}

/* ===================================================================== */
/* §7  scene — input state + world size                                  */
/* ===================================================================== */

/*
 * Keys — one bool per action, set fresh each frame from getch().
 * Using a struct rather than a bitmask keeps the intent readable and
 * allows multiple keys to be active simultaneously without bitwise ops.
 */
typedef struct {
    bool fwd, rev, left, right, spin_l, spin_r, stop;
} Keys;

typedef struct {
    Robot robot;
    Keys  keys;
    int   wpx, hpx;   /* world size in pixels */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->wpx = pw(cols);
    s->hpx = ph(rows - 2);   /* rows-2: reserve top/bottom HUD rows */
    memset(&s->keys, 0, sizeof s->keys);
    robot_init(&s->robot, s->wpx, s->hpx);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->wpx = pw(cols);
    s->hpx = ph(rows - 2);
}

/*
 * scene_tick — apply keyboard input to commands then step physics.
 *
 * Command update logic:
 *   fwd/rev: additive ramp (V_ACCEL × V_MAX per tick).
 *   No key pressed: V_DECAY multiplied (=1.0 → no decay for linear).
 *   This asymmetry lets the user hold a key to accelerate smoothly, and
 *   release to coast at constant speed until explicitly braking with S.
 *
 *   turn: additive ramp with angular decay W_DECAY when released.
 *   spin: override both v_cmd and w_cmd for pure rotation in place.
 */
static void scene_tick(Scene *s, float dt)
{
    Robot *r = &s->robot;
    if (r->paused) return;

    Keys *k = &s->keys;

    if (k->stop) { r->v_cmd = 0.0f; r->w_cmd = 0.0f; }

    /* Linear command — accumulate or coast */
    if (k->fwd)  r->v_cmd += V_MAX * V_ACCEL;
    if (k->rev)  r->v_cmd -= V_MAX * V_ACCEL;
    if (!k->fwd && !k->rev) r->v_cmd *= V_DECAY;

    /* Angular command — accumulate or damp */
    if (k->right)  r->w_cmd += W_MAX * W_ACCEL;
    if (k->left)   r->w_cmd -= W_MAX * W_ACCEL;
    if (!k->right && !k->left) r->w_cmd *= W_DECAY;

    /* Spin in place: set commands directly (overrides ramp) */
    if (k->spin_r) { r->v_cmd = 0.0f; r->w_cmd =  W_MAX; }
    if (k->spin_l) { r->v_cmd = 0.0f; r->w_cmd = -W_MAX; }

    /* Clamp to physical limits */
    if (r->v_cmd >  V_MAX) r->v_cmd =  V_MAX;
    if (r->v_cmd < -V_MAX) r->v_cmd = -V_MAX;
    if (r->w_cmd >  W_MAX) r->w_cmd =  W_MAX;
    if (r->w_cmd < -W_MAX) r->w_cmd = -W_MAX;

    robot_step(r, dt, s->wpx, s->hpx);
}

/* ===================================================================== */
/* §8  render                                                             */
/* ===================================================================== */

/*
 * draw_dot_line — draw a directional arrow using '.'→'o'→'0' progression.
 *
 * WHY not Bresenham: Bresenham samples at cell boundaries, so for a
 * diagonal line it visits cells in a staircase pattern and maps each
 * step to '\' or '/' based on slope sign.  With a 2:1 cell aspect ratio
 * those diagonal chars look wrong — the visual angle doesn't match the
 * actual angle.
 *
 * This function instead steps uniformly in PIXEL SPACE (step = CELL_W)
 * and converts each sample to a cell coordinate.  Duplicate cell visits
 * are skipped.  The character at each cell is chosen by progress t ∈ [0,1]:
 *   t < 0.40 → '.'  (near, faint — "starting")
 *   t < 0.75 → 'o'  (mid  — "growing")
 *   t ≥ 0.75 → '0'  (far  — "tip, boldest")
 *
 * The progressive density gives a visual sense of direction without any
 * angle-to-char mapping, and it looks the same at all arrow angles.
 */
static void draw_dot_line(float x0, float y0, float x1, float y1,
                          int cp, attr_t extra, int cols, int rows)
{
    float dx  = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.5f) return;

    int steps = (int)(len / (float)CELL_W) + 1;

    int prev_cx = -9999, prev_cy = -9999;
    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)(steps > 0 ? steps : 1);
        int   cx = px_to_cx(x0 + dx * t);
        int   cy = px_to_cy(y0 + dy * t);
        if (cx == prev_cx && cy == prev_cy) continue;   /* same cell — skip */
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 2) continue;

        chtype ch = (t < 0.40f) ? '.' : (t < 0.75f) ? 'o' : '0';
        mvaddch(cy, cx, ch | (chtype)COLOR_PAIR(cp) | (chtype)extra);
    }
}

/*
 * tip_char — choose the arrowhead character for a given angle.
 *
 * Cardinal directions get sharp ASCII arrows (>, v, <, ^) that
 * immediately read as directional.  Diagonal octants get 'o' — a
 * round, neutral char that doesn't suggest a wrong direction.
 *
 * WHY not '\' and '/': on a 2:1 aspect ratio grid those chars look
 * steeper than intended — they read as near-vertical even for a true
 * 45° heading.  'o' makes no claim about angle.
 */
static chtype tip_char(float theta)
{
    float d = fmodf(theta * (180.0f / (float)M_PI) + 360.0f, 360.0f);
    if (d < 22.5f  || d >= 337.5f) return '>';   /* east       */
    if (d < 67.5f)                 return 'o';   /* south-east */
    if (d < 112.5f)                return 'v';   /* south      */
    if (d < 157.5f)                return 'o';   /* south-west */
    if (d < 202.5f)                return '<';   /* west       */
    if (d < 247.5f)                return 'o';   /* north-west */
    if (d < 292.5f)                return '^';   /* north      */
    return                                'o';   /* north-east */
}

/*
 * draw_wheel_arrow — draw velocity arrow for one wheel.
 *
 * Arrow starts at the wheel's pixel position (wx_px, wy_px) and extends
 * along ±heading direction by |v_wheel|/V_MAX × VEL_ARROW_PX pixels.
 * Green = forward, red = reverse.
 *
 * Abstracting this removes the duplicated left/right wheel code that
 * was in the original render function.
 */
static void draw_wheel_arrow(float wx_px, float wy_px,
                             float v_wheel, float theta,
                             int cols, int rows)
{
    float alen = fabsf(v_wheel / V_MAX) * VEL_ARROW_PX;
    if (alen < 1.0f) return;                    /* too slow — skip drawing */

    float sign = (v_wheel >= 0.0f) ? 1.0f : -1.0f;
    float ex   = wx_px + cosf(theta) * sign * alen;
    float ey   = wy_px + sinf(theta) * sign * alen;
    int   cp   = (v_wheel >= 0.0f) ? CP_VEL_P : CP_VEL_N;

    draw_dot_line(wx_px, wy_px, ex, ey, cp, A_BOLD, cols, rows);

    /* Tip character at arrow endpoint */
    int eax = px_to_cx(ex), eay = px_to_cy(ey);
    if (eax >= 0 && eax < cols && eay >= 1 && eay < rows - 2) {
        float ta = (v_wheel >= 0.0f) ? theta : theta + (float)M_PI;
        mvaddch(eay, eax, tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
    }
}

/*
 * interpolated_pose — compute the sub-tick render pose.
 *
 * Physics ticks at SIM_FPS; the render frame may fall between ticks.
 * alpha = sim_accum / tick_ns ∈ [0,1) tells how far into the current
 * tick we are.  Linear interpolation between prev and current gives
 * smooth motion at any render rate.
 *
 * Heading lerp uses short-arc normalization to avoid a visual snap when
 * theta crosses the ±π discontinuity.
 *
 * World-wrap suppression: if the robot wrapped a toroidal edge this
 * tick, the raw delta (px - prev_px) is large.  We clamp it to zero
 * so the interpolated draw position doesn't streak across the screen.
 */
static void interpolated_pose(const Robot *r, float alpha,
                              int cols, int rows,
                              float *ipx, float *ipy, float *itheta)
{
    float dth = r->theta - r->prev_theta;
    if (dth >  (float)M_PI) dth -= 2.0f * (float)M_PI;
    if (dth < -(float)M_PI) dth += 2.0f * (float)M_PI;

    float dpx = r->px - r->prev_px;
    float dpy = r->py - r->prev_py;
    if (fabsf(dpx) > (float)(cols * CELL_W) * 0.5f) dpx = 0.0f;
    if (fabsf(dpy) > (float)(rows * CELL_H) * 0.5f) dpy = 0.0f;

    *ipx    = r->prev_px    + dpx * alpha;
    *ipy    = r->prev_py    + dpy * alpha;
    *itheta = r->prev_theta + dth * alpha;
}

/*
 * render_trail — draw the position history ring buffer.
 *
 * Iterate newest-first (head−1, head−2, …).  Age fraction t ∈ [0,1]:
 *   t < 0.12 → A_BOLD + '.' + CP_TR_NEW  (very recent, vivid)
 *   t < 0.35 → A_DIM  + '.' + CP_TR_NEW  (recent, fading)
 *   t ≥ 0.35 → A_DIM  + ':' + CP_TR_OLD  (old, barely visible)
 *
 * Using ':' for old dots makes them visually recede — the two stacked
 * dots read as "faded" compared to the solid '.' of fresh dots.
 */
static void render_trail(const Robot *r, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int idx = ((r->trail.head - 1 - k) + TRAIL_CAP) % TRAIL_CAP;
        int tc  = px_to_cx(r->trail.px[idx]);
        int tr  = px_to_cy(r->trail.py[idx]);
        if (tc < 0 || tc >= cols || tr < 1 || tr >= rows - 2) continue;

        float age  = (float)k / (float)(r->trail.count > 1 ? r->trail.count - 1 : 1);
        int   cp   = (age < 0.35f) ? CP_TR_NEW : CP_TR_OLD;
        attr_t at  = (age < 0.12f) ? A_BOLD : A_DIM;
        char   ch  = (age < 0.35f) ? '.' : ':';

        attron(COLOR_PAIR(cp) | at);
        mvaddch(tr, tc, (chtype)ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/*
 * render_robot — compose all robot visual elements.
 *
 * Draw order (last write wins in ncurses):
 *   1. Trail dots     — background, drawn first so robot covers them
 *   2. Wheel arrows   — velocity indicators
 *   3. Heading arrow  — direction of travel
 *   4. Wheel labels   — 'L', 'R' mark physical wheel positions
 *   5. Body '@'       — drawn last, always on top
 */
static void render_robot(const Robot *r, float alpha, int cols, int rows)
{
    float ipx, ipy, itheta;
    interpolated_pose(r, alpha, cols, rows, &ipx, &ipy, &itheta);

    int   cx   = px_to_cx(ipx);
    int   cy   = px_to_cy(ipy);
    float sinT = sinf(itheta), cosT = cosf(itheta);
    float half = r->axle * 0.5f;

    /*
     * Wheel pixel positions: perpendicular to heading in y-down coords.
     *   left  = body + (+sinθ, −cosθ) · axle/2
     *   right = body + (−sinθ, +cosθ) · axle/2
     * Verify with θ=0 (east): left = (+0, −1)·half → north ✓
     *                          right = (−0, +1)·half → south ✓
     */
    float lx_px = ipx + sinT * half,  ly_px = ipy - cosT * half;
    float rx_px = ipx - sinT * half,  ry_px = ipy + cosT * half;

    /* 1. Trail */
    render_trail(r, cols, rows);

    /* 2. Wheel velocity arrows */
    draw_wheel_arrow(lx_px, ly_px, r->vL, itheta, cols, rows);
    draw_wheel_arrow(rx_px, ry_px, r->vR, itheta, cols, rows);

    /* 3. Heading arrow from body centre */
    {
        float ex = ipx + cosT * ARROW_PX;
        float ey = ipy + sinT * ARROW_PX;
        draw_dot_line(ipx, ipy, ex, ey, CP_HEAD, A_BOLD, cols, rows);
        int tx = px_to_cx(ex), ty = px_to_cy(ey);
        if (tx >= 0 && tx < cols && ty >= 1 && ty < rows - 2)
            mvaddch(ty, tx, tip_char(itheta) | (chtype)COLOR_PAIR(CP_HEAD) | (chtype)A_BOLD);
    }

    /* 4. Wheel labels */
    int lx = px_to_cx(lx_px), ly = px_to_cy(ly_px);
    int rx = px_to_cx(rx_px), ry = px_to_cy(ry_px);
    attron(COLOR_PAIR(CP_WHL_L) | A_BOLD);
    if (lx >= 0 && lx < cols && ly >= 1 && ly < rows - 2) mvaddch(ly, lx, 'L');
    attroff(COLOR_PAIR(CP_WHL_L) | A_BOLD);
    attron(COLOR_PAIR(CP_WHL_R) | A_BOLD);
    if (rx >= 0 && rx < cols && ry >= 1 && ry < rows - 2) mvaddch(ry, rx, 'R');
    attroff(COLOR_PAIR(CP_WHL_R) | A_BOLD);

    /* 5. Body — drawn last so it's never occluded */
    attron(COLOR_PAIR(CP_BODY) | A_BOLD);
    if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 2) mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(CP_BODY) | A_BOLD);
}

/*
 * render_overlay — HUD: pose, kinematics, wheel speeds, key reference.
 *
 * Displayed quantities:
 *   x, y    position in pixel space (0,0 = top-left)
 *   hdg     heading in degrees; 0=east, 90=south, ±180=west, −90=north
 *   v       linear body speed [px/s]
 *   w       angular speed [rad/s]; positive = clockwise
 *   R       instantaneous turn radius [px]; "INF" when ω≈0
 *   L/R     individual wheel speeds, color-coded green/red
 */
static void render_overlay(const Robot *r, int cols, int rows)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;
    float R_val = (fabsf(omega) > 1e-3f) ? (v / omega) : 1.0e6f;
    float deg   = r->theta * (180.0f / (float)M_PI);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
        " x:%6.0f  y:%6.0f  hdg:%+7.1fdeg  v:%+6.1f px/s"
        "  w:%+5.2f r/s  R:",
        r->px, r->py, deg, v, omega);
    if (fabsf(R_val) > 9999.0f)
        mvprintw(0, getcurx(stdscr), "  INF px ");
    else
        mvprintw(0, getcurx(stdscr), "%-5.0f px ", fabsf(R_val));
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Per-wheel speed display — green for forward, red for reverse */
    int hx = cols - 18;
    if (hx > 0) {
        int cp_L = (r->vL >= 0.0f) ? CP_VEL_P : CP_VEL_N;
        int cp_R = (r->vR >= 0.0f) ? CP_VEL_P : CP_VEL_N;
        attron(COLOR_PAIR(CP_WHL_L));  mvprintw(0, hx,   "L");      attroff(COLOR_PAIR(CP_WHL_L));
        attron(COLOR_PAIR(cp_L)|A_BOLD); mvprintw(0, hx+1,":%+5.0f", r->vL); attroff(COLOR_PAIR(cp_L)|A_BOLD);
        attron(COLOR_PAIR(CP_WHL_R));  mvprintw(0, hx+8, "R");      attroff(COLOR_PAIR(CP_WHL_R));
        attron(COLOR_PAIR(cp_R)|A_BOLD); mvprintw(0, hx+9,":%+5.0f", r->vR); attroff(COLOR_PAIR(cp_R)|A_BOLD);
    }

    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " [W]fwd [S]brake [AD]turn [E]spin-R [Z]spin-L [spc]stop [p]pause [r]reset [q]quit");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (r->paused) {
        attron(COLOR_PAIR(CP_VEL_N) | A_BOLD);
        mvprintw(rows - 1, cols / 2 - 4, " PAUSED ");
        attroff(COLOR_PAIR(CP_VEL_N) | A_BOLD);
    }
}

static void scene_draw(const Scene *s, float alpha, int cols, int rows)
{
    erase();
    render_robot(&s->robot, alpha, cols, rows);
    render_overlay(&s->robot, cols, rows);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak();
    curs_set(0); nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)   { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit(int s)   { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)    { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t tick_ns     = NS_PER_SEC / SIM_FPS;
    float   dt_sec      = 1.0f / (float)SIM_FPS;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ─────────────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── wall-clock dt, capped to prevent spiral-of-death ─ */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── drain input queue, set key flags for this frame ─── */
        Keys *k = &app->scene.keys;
        memset(k, 0, sizeof *k);
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: app->running = 0;             break;
            case 'w': case 'W': case KEY_UP:    k->fwd    = true;      break;
            case 's': case 'S': case KEY_DOWN:  k->rev    = true;      break;
            case 'a': case 'A': case KEY_LEFT:  k->left   = true;      break;
            case 'd': case 'D': case KEY_RIGHT: k->right  = true;      break;
            case 'e': case 'E':                 k->spin_r = true;      break;
            case 'z': case 'Z': case KEY_PPAGE: k->spin_l = true;      break;
            case ' ':                           k->stop   = true;      break;
            case 'p': case 'P':
                app->scene.robot.paused = !app->scene.robot.paused;    break;
            case 'r': case 'R':
                robot_reset(&app->scene.robot,
                            app->scene.wpx, app->scene.hpx);           break;
            default: break;
            }
        }

        /* ── fixed-timestep physics accumulator ─────────────── */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ── alpha for render interpolation ─────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── fps counter (rolling 0.5 s window) ─────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap: sleep before render ─────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ── render ──────────────────────────────────────────── */
        scene_draw(&app->scene, alpha, app->screen.cols, app->screen.rows);

        attron(COLOR_PAIR(CP_HUD) | A_DIM);
        mvprintw(0, app->screen.cols - 10, " %.0ffps ", fps_display);
        attroff(COLOR_PAIR(CP_HUD) | A_DIM);

        wnoutrefresh(stdscr);
        doupdate();
    }

    screen_free(&app->screen);
    return 0;
}
