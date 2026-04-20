/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diff_drive_robot.c — Differential Drive Robot Simulator
 *
 * Physics:
 *   v = (vL + vR) / 2          linear velocity
 *   ω = (vR − vL) / L          angular velocity  (CW positive in y-down)
 *   ẋ = v·cos θ                pose update
 *   ẏ = v·sin θ
 *   θ̇ = ω
 *
 * Visualization:
 *   '@' body · Bresenham heading arrow · 'L'/'R' wheels
 *   wheel velocity arrows (length ∝ speed) · '.' trail
 *
 * Controls:
 *   W/↑  forward      S/↓  reverse
 *   A/←  turn left    D/→  turn right
 *   Q    spin left     E    spin right
 *   Space  full stop   p    pause
 *   r    reset         q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/diff_drive_robot.c \
 *       -o diff_drive_robot -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config  §2 clock  §3 color  §4 coords
 *  §5 robot   §6 scene  §7 render §8 screen  §9 app
 * ─────────────────────────────────────────────────────────────────────
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Nonholonomic constraint:
 *   A differential drive robot cannot slide sideways.  Its instantaneous
 *   velocity must lie along its heading: ẋ·sin θ − ẏ·cos θ = 0.
 *   Any lateral displacement requires first rotating (spending angular
 *   motion budget), then translating.  This distinguishes it from a
 *   holonomic platform (e.g. omni wheels) which can move in any direction.
 *
 * Wheel velocity mapping:
 *   Commands (v, ω) map to individual wheels as:
 *     vL = v − ω·L/2          vR = v + ω·L/2
 *   Inverse: v = (vR+vL)/2,   ω = (vR−vL)/L.
 *   In y-down screen coordinates, ω > 0 rotates θ clockwise (rightward).
 *   vR > vL → ω > 0 → robot turns right.
 *   vL > vR → ω < 0 → robot turns left.
 *
 * Turn radius relation:
 *   R = v/ω = L/2 · (vR+vL)/(vR−vL)
 *   R → ∞  when vL = vR  (straight line).
 *   R = 0   when vL = −vR (spin in place).
 *   R = L/2 when one wheel is stationary (pivot about that wheel).
 *   The Instantaneous Centre of Curvature (ICC) lies on the robot's axle
 *   at distance |R| from the body centre, on the slower wheel's side.
 * ─────────────────────────────────────────────────────────────────────── */

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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define SIM_FPS      60
#define TARGET_FPS   60

/* Aspect-ratio pixel grid (must match CELL_H/CELL_W ≈ 2) */
#define CELL_W        8
#define CELL_H       16

/* Robot geometry in pixel space */
#define AXLE_PX      36.0f   /* wheel separation                  */
#define ARROW_PX     34.0f   /* heading arrow length              */
#define VEL_ARROW_PX 28.0f   /* max wheel-velocity arrow length   */

/* Dynamics */
#define V_MAX       180.0f   /* max linear speed  (pixels/sec)    */
#define W_MAX         3.0f   /* max angular speed (rad/sec)       */
#define V_ACCEL       0.20f  /* v ramp per tick (fraction of V_MAX) */
#define W_ACCEL       0.25f  /* w ramp per tick                     */
#define V_DECAY       1.00f  /* no linear decay: robot keeps speed  */
#define W_DECAY       0.88f  /* angular decay: stops spinning on own */

/* Trail */
#define TRAIL_CAP   600
#define TRAIL_STEP    2      /* record every N ticks              */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Colour pairs */
#define CP_BODY      1
#define CP_HEAD      2
#define CP_WHL_L     3
#define CP_WHL_R     4
#define CP_TR_NEW    5
#define CP_TR_OLD    6
#define CP_HUD       7
#define CP_VEL_P     8
#define CP_VEL_N     9

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
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
        init_pair(CP_BODY,  255, COLOR_BLACK);
        init_pair(CP_HEAD,  226, COLOR_BLACK);
        init_pair(CP_WHL_L,  46, COLOR_BLACK);
        init_pair(CP_WHL_R, 201, COLOR_BLACK);
        init_pair(CP_TR_NEW, 51, COLOR_BLACK);
        init_pair(CP_TR_OLD, 17, COLOR_BLACK);
        init_pair(CP_HUD,    51, COLOR_BLACK);
        init_pair(CP_VEL_P,  82, COLOR_BLACK);
        init_pair(CP_VEL_N, 196, COLOR_BLACK);
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
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cx(float px)
{ return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cy(float py)
{ return (int)floorf(py / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  robot entity                                                       */
/* ===================================================================== */

/* Ring-buffer of past pixel positions for the trail */
typedef struct {
    float px[TRAIL_CAP], py[TRAIL_CAP];
    int   head, count, skip;
} Trail;

static void trail_push(Trail *t, float px, float py)
{
    if (++t->skip < TRAIL_STEP) return;
    t->skip = 0;
    t->px[t->head] = px;
    t->py[t->head] = py;
    t->head = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/*
 * Robot state.
 *
 * Pose (px, py, theta) is in pixel space.  theta=0 faces east; positive
 * theta rotates clockwise on screen (y-down convention).
 *
 * Control commands (v_cmd, w_cmd) are set by keyboard and decay each tick.
 * They map to wheel speeds: vL = v_cmd − w_cmd·axle/2
 *                           vR = v_cmd + w_cmd·axle/2
 */
typedef struct {
    float px, py, theta;         /* pose (pixels, radians)        */
    float vL, vR;                /* current wheel speeds (px/s)   */
    float v_cmd, w_cmd;          /* control inputs                */
    float axle;                  /* wheel separation (pixels)     */
    float prev_px, prev_py, prev_theta;  /* for render interpolation */
    Trail trail;
    bool  paused;
} Robot;

static void robot_reset(Robot *r, int wpx, int hpx)
{
    float px = r->px, py = r->py, theta = r->theta;
    float axle = r->axle;
    memset(r, 0, sizeof *r);
    r->axle  = axle;
    r->px    = (float)wpx * 0.5f;
    r->py    = (float)hpx * 0.5f;
    r->theta = 0.0f;
    (void)px; (void)py; (void)theta;
}

static void robot_init(Robot *r, int wpx, int hpx)
{
    memset(r, 0, sizeof *r);
    r->axle = AXLE_PX;
    robot_reset(r, wpx, hpx);
}

/*
 * integrate_motion — Euler pose integration from wheel speeds.
 *
 *   v     = (vL + vR) / 2
 *   ω     = (vR − vL) / L       [y-down: CW = positive]
 *   x    += v·cos(θ)·dt
 *   y    += v·sin(θ)·dt
 *   θ    += ω·dt
 *
 * Simple Euler is accurate enough at 60 Hz for a visualisation.
 * For smoother arcs (large dt or high ω) replace with ICC exact-arc:
 *   R = v/ω;  θ_new = θ + ω·dt
 *   x_new = x + R·(sin(θ_new) − sin(θ))
 *   y_new = y − R·(cos(θ_new) − cos(θ))
 */
static void integrate_motion(Robot *r, float dt)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;

    r->prev_px    = r->px;
    r->prev_py    = r->py;
    r->prev_theta = r->theta;

    r->px    += v * cosf(r->theta) * dt;
    r->py    += v * sinf(r->theta) * dt;
    r->theta += omega * dt;

    while (r->theta >  (float)M_PI) r->theta -= 2.0f * (float)M_PI;
    while (r->theta < -(float)M_PI) r->theta += 2.0f * (float)M_PI;
}

/*
 * update_pose — map (v_cmd, w_cmd) → wheel speeds, integrate, wrap world.
 *
 * Wheel speed derivation:
 *   vL = v_cmd − w_cmd·L/2
 *   vR = v_cmd + w_cmd·L/2
 * Clamped to ±V_MAX so neither wheel spins faster than its physical limit.
 */
static void update_pose(Robot *r, float dt, int wpx, int hpx)
{
    float half = r->axle * 0.5f;
    r->vL = r->v_cmd - r->w_cmd * half;
    r->vR = r->v_cmd + r->w_cmd * half;

    if (r->vL >  V_MAX) r->vL =  V_MAX;
    if (r->vL < -V_MAX) r->vL = -V_MAX;
    if (r->vR >  V_MAX) r->vR =  V_MAX;
    if (r->vR < -V_MAX) r->vR = -V_MAX;

    integrate_motion(r, dt);

    /* Toroidal wrap-around */
    float wx = (float)wpx, wy = (float)hpx;
    if (r->px <   0.0f) r->px += wx;
    if (r->px >= wx)    r->px -= wx;
    if (r->py <   0.0f) r->py += wy;
    if (r->py >= wy)    r->py -= wy;

    trail_push(&r->trail, r->px, r->py);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

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
    s->hpx = ph(rows - 2);
    memset(&s->keys, 0, sizeof s->keys);
    robot_init(&s->robot, s->wpx, s->hpx);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->wpx = pw(cols);
    s->hpx = ph(rows - 2);
}

static void scene_tick(Scene *s, float dt)
{
    Robot *r = &s->robot;
    if (r->paused) return;

    Keys *k = &s->keys;

    if (k->stop) { r->v_cmd = 0.0f; r->w_cmd = 0.0f; }

    if (k->fwd)  r->v_cmd += V_MAX * V_ACCEL;
    if (k->rev)  r->v_cmd -= V_MAX * V_ACCEL;
    if (!k->fwd && !k->rev) r->v_cmd *= V_DECAY;

    if (k->right)  r->w_cmd += W_MAX * W_ACCEL;
    if (k->left)   r->w_cmd -= W_MAX * W_ACCEL;
    if (!k->right && !k->left) r->w_cmd *= W_DECAY;

    if (k->spin_r) { r->v_cmd = 0.0f; r->w_cmd =  W_MAX; }
    if (k->spin_l) { r->v_cmd = 0.0f; r->w_cmd = -W_MAX; }

    if (r->v_cmd >  V_MAX) r->v_cmd =  V_MAX;
    if (r->v_cmd < -V_MAX) r->v_cmd = -V_MAX;
    if (r->w_cmd >  W_MAX) r->w_cmd =  W_MAX;
    if (r->w_cmd < -W_MAX) r->w_cmd = -W_MAX;

    update_pose(r, dt, s->wpx, s->hpx);
}

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* Step in pixel space placing . o 0 chars — no ugly diagonals */
static void draw_dot_line(float x0, float y0, float x1, float y1,
                          int cp, attr_t extra, int cols, int rows)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.5f) return;
    int steps = (int)(len / (float)CELL_W) + 1;
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)(steps > 0 ? steps : 1);
        int   cx = px_to_cx(x0 + dx * t);
        int   cy = px_to_cy(y0 + dy * t);
        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx<0||cx>=cols||cy<1||cy>=rows-2) continue;
        chtype ch = (t < 0.40f) ? '.' : (t < 0.75f) ? 'o' : '0';
        mvaddch(cy, cx, ch | (chtype)COLOR_PAIR(cp) | (chtype)extra);
    }
}

/* Arrow-tip char for angle theta — cardinals sharp, diagonals soft */
static chtype tip_char(float theta)
{
    float d = fmodf(theta * (180.0f/(float)M_PI) + 360.0f, 360.0f);
    if (d < 22.5f  || d >= 337.5f) return '>';
    if (d < 67.5f)  return 'o';
    if (d < 112.5f) return 'v';
    if (d < 157.5f) return 'o';
    if (d < 202.5f) return '<';
    if (d < 247.5f) return 'o';
    if (d < 292.5f) return '^';
    return 'o';
}

/*
 * render_robot — draw all robot elements in cell space.
 *
 * Wheel perpendicular offsets (y-down, theta=0 faces east):
 *   Left  wheel offset: (+sin θ, −cos θ) · axle/2   [north of body when east]
 *   Right wheel offset: (−sin θ, +cos θ) · axle/2   [south of body when east]
 *
 * Velocity arrows extend from each wheel along ±heading direction,
 * length proportional to |vL| or |vR|.
 */
static void render_robot(const Robot *r, float alpha, int cols, int rows)
{
    /* Sub-tick interpolated pose */
    float dth = r->theta - r->prev_theta;
    if (dth >  (float)M_PI) dth -= 2.0f*(float)M_PI;
    if (dth < -(float)M_PI) dth += 2.0f*(float)M_PI;

    float dpx = r->px - r->prev_px;
    float dpy = r->py - r->prev_py;
    /* Suppress interpolation across world-wrap */
    if (fabsf(dpx) > (float)(cols * CELL_W) * 0.5f) dpx = 0.0f;
    if (fabsf(dpy) > (float)(rows * CELL_H) * 0.5f) dpy = 0.0f;

    float ipx   = r->prev_px    + dpx * alpha;
    float ipy   = r->prev_py    + dpy * alpha;
    float itheta = r->prev_theta + dth * alpha;

    int cx = px_to_cx(ipx);
    int cy = px_to_cy(ipy);

    float sinT = sinf(itheta), cosT = cosf(itheta);
    float half  = r->axle * 0.5f;

    /* Wheel pixel positions */
    float lx_px = ipx + sinT * half,   ly_px = ipy - cosT * half;
    float rx_px = ipx - sinT * half,   ry_px = ipy + cosT * half;
    int   lx = px_to_cx(lx_px), ly = px_to_cy(ly_px);
    int   rx = px_to_cx(rx_px), ry = px_to_cy(ry_px);

    /* ── Trail ────────────────────────────────────────────────── */
    for (int k = 0; k < r->trail.count; k++) {
        int idx = ((r->trail.head - 1 - k) + TRAIL_CAP) % TRAIL_CAP;
        int tc  = px_to_cx(r->trail.px[idx]);
        int tr  = px_to_cy(r->trail.py[idx]);
        if (tc<0||tc>=cols||tr<1||tr>=rows-2) continue;
        float age = (float)k / (float)(r->trail.count > 1 ? r->trail.count-1 : 1);
        int cp  = (age < 0.35f) ? CP_TR_NEW : CP_TR_OLD;
        attr_t at = (age < 0.12f) ? A_BOLD : A_DIM;
        attron(COLOR_PAIR(cp) | at);
        mvaddch(tr, tc, (age < 0.35f) ? '.' : ':');
        attroff(COLOR_PAIR(cp) | at);
    }

    /* ── Wheel velocity arrows ────────────────────────────────── */
    /* Arrow from each wheel extending forward (positive v) or backward */
    float vL_frac = r->vL / V_MAX;
    float vR_frac = r->vR / V_MAX;

    /* Left wheel arrow */
    {
        float alen = fabsf(vL_frac) * VEL_ARROW_PX;
        float sign = (r->vL >= 0.0f) ? 1.0f : -1.0f;
        float ex   = lx_px + cosT * sign * alen;
        float ey   = ly_px + sinT * sign * alen;
        int   eax  = px_to_cx(ex), eay = px_to_cy(ey);
        int   cp   = (r->vL >= 0.0f) ? CP_VEL_P : CP_VEL_N;
        if (alen > 1.0f) draw_dot_line(lx_px, ly_px, ex, ey, cp, A_BOLD, cols, rows);
        if (eax>=0&&eax<cols&&eay>=1&&eay<rows-2) {
            float ta = (r->vL >= 0.0f) ? itheta : itheta + (float)M_PI;
            mvaddch(eay, eax,
                    tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
        }
    }

    /* Right wheel arrow */
    {
        float alen = fabsf(vR_frac) * VEL_ARROW_PX;
        float sign = (r->vR >= 0.0f) ? 1.0f : -1.0f;
        float ex   = rx_px + cosT * sign * alen;
        float ey   = ry_px + sinT * sign * alen;
        int   eax  = px_to_cx(ex), eay = px_to_cy(ey);
        int   cp   = (r->vR >= 0.0f) ? CP_VEL_P : CP_VEL_N;
        if (alen > 1.0f) draw_dot_line(rx_px, ry_px, ex, ey, cp, A_BOLD, cols, rows);
        if (eax>=0&&eax<cols&&eay>=1&&eay<rows-2) {
            float ta = (r->vR >= 0.0f) ? itheta : itheta + (float)M_PI;
            mvaddch(eay, eax,
                    tip_char(ta) | (chtype)COLOR_PAIR(cp) | (chtype)A_BOLD);
        }
    }

    /* ── Heading arrow ────────────────────────────────────────── */
    {
        float ex = ipx + cosT * ARROW_PX;
        float ey = ipy + sinT * ARROW_PX;
        int   tx = px_to_cx(ex), ty = px_to_cy(ey);
        draw_dot_line(ipx, ipy, ex, ey, CP_HEAD, A_BOLD, cols, rows);
        if (tx>=0&&tx<cols&&ty>=1&&ty<rows-2)
            mvaddch(ty, tx, tip_char(itheta) | (chtype)COLOR_PAIR(CP_HEAD) | (chtype)A_BOLD);
    }

    /* ── Wheel labels ─────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_WHL_L) | A_BOLD);
    if (lx>=0&&lx<cols&&ly>=1&&ly<rows-2) mvaddch(ly, lx, 'L');
    attroff(COLOR_PAIR(CP_WHL_L) | A_BOLD);

    attron(COLOR_PAIR(CP_WHL_R) | A_BOLD);
    if (rx>=0&&rx<cols&&ry>=1&&ry<rows-2) mvaddch(ry, rx, 'R');
    attroff(COLOR_PAIR(CP_WHL_R) | A_BOLD);

    /* ── Robot body ───────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_BODY) | A_BOLD);
    if (cx>=0&&cx<cols&&cy>=1&&cy<rows-2) mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(CP_BODY) | A_BOLD);
}

/*
 * render_overlay — HUD: pose, wheel speeds, turn radius, key hints.
 *
 * Overlay items:
 *   x, y   - robot position in pixel space
 *   hdg    - heading in degrees (0=east, 90=south, +/-180=west)
 *   v      - linear speed (px/s)
 *   w      - angular speed (rad/s)
 *   R      - instantaneous turn radius (px); INF when going straight
 *   vL,vR  - individual wheel speeds (green=fwd, red=rev)
 */
static void render_overlay(const Robot *r, int cols, int rows)
{
    float v     = (r->vL + r->vR) * 0.5f;
    float omega = (r->vR - r->vL) / r->axle;
    float R_val = (fabsf(omega) > 1e-3f) ? (v / omega) : 1e6f;
    float deg   = r->theta * (180.0f / (float)M_PI);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
        " x:%6.0f  y:%6.0f  hdg:%+7.1fdeg  v:%+6.1f px/s"
        "  w:%+5.2f r/s  R:%s px ",
        r->px, r->py, deg, v, omega,
        (fabsf(R_val) > 9999.0f) ? "  INF" : "");
    if (fabsf(R_val) <= 9999.0f)
        mvprintw(0, getcurx(stdscr), "%-5.0f px ", fabsf(R_val));
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Wheel speed indicators (colour-coded) */
    int cp_L = (r->vL >= 0.0f) ? CP_VEL_P : CP_VEL_N;
    int cp_R = (r->vR >= 0.0f) ? CP_VEL_P : CP_VEL_N;
    int hx = cols - 18;
    if (hx > 0) {
        attron(COLOR_PAIR(CP_WHL_L));  mvprintw(0, hx,     "L");  attroff(COLOR_PAIR(CP_WHL_L));
        attron(COLOR_PAIR(cp_L)|A_BOLD); mvprintw(0, hx+1, ":%+5.0f", r->vL); attroff(COLOR_PAIR(cp_L)|A_BOLD);
        attron(COLOR_PAIR(CP_WHL_R));  mvprintw(0, hx+8,   "R");  attroff(COLOR_PAIR(CP_WHL_R));
        attron(COLOR_PAIR(cp_R)|A_BOLD); mvprintw(0, hx+9, ":%+5.0f", r->vR); attroff(COLOR_PAIR(cp_R)|A_BOLD);
    }

    /* Key hints */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows-1, 0,
        " [W]fwd [S]brake [AD]turn [E]spin-R [Z]spin-L [spc]stop [p]pause [r]reset [q]quit");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (r->paused) {
        attron(COLOR_PAIR(CP_VEL_N) | A_BOLD);
        mvprintw(rows-1, cols/2 - 4, " PAUSED ");
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
/* §8  screen                                                             */
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
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

/* ===================================================================== */
/* §9  app                                                                */
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

        /* ── dt ─────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── input (drain queue; set key flags this frame) ───── */
        Keys *k = &app->scene.keys;
        memset(k, 0, sizeof *k);
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: app->running = 0; break;
            case 'w': case 'W': case KEY_UP:    k->fwd    = true; break;
            case 's': case 'S': case KEY_DOWN:  k->rev    = true; break;
            case 'a': case 'A': case KEY_LEFT:  k->left   = true; break;
            case 'd': case 'D': case KEY_RIGHT: k->right  = true; break;
            case 'e': case 'E':                 k->spin_r = true; break;
            case KEY_PPAGE: /* pgup — map to spin left */
            case 'z': case 'Z':                 k->spin_l = true; break;
            case ' ':                           k->stop   = true; break;
            case 'p': case 'P':
                app->scene.robot.paused = !app->scene.robot.paused; break;
            case 'r': case 'R':
                robot_reset(&app->scene.robot,
                            app->scene.wpx, app->scene.hpx); break;
            default: break;
            }
        }

        /* ── sim accumulator (fixed timestep) ───────────────── */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ── alpha (render interpolation) ───────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── fps counter ─────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0; fps_accum = 0;
        }

        /* ── frame cap ───────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ── draw ────────────────────────────────────────────── */
        scene_draw(&app->scene, alpha,
                   app->screen.cols, app->screen.rows);

        /* fps in top-right */
        attron(COLOR_PAIR(CP_HUD) | A_DIM);
        mvprintw(0, app->screen.cols - 10, " %.0ffps ", fps_display);
        attroff(COLOR_PAIR(CP_HUD) | A_DIM);

        wnoutrefresh(stdscr);
        doupdate();
    }

    screen_free(&app->screen);
    return 0;
}
