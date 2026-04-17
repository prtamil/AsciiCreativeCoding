/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snake_inverse_kinematics.c — 2-D IK Snake Chasing a Wandering Target
 *
 * DEMO: A 32-segment snake chases an organically wandering target that carves
 *       terrain-like paths across the screen.  The target steers via a sum of
 *       three incommensurable sine waves — producing irregular hills and valleys
 *       that never exactly repeat.  The head is goal-driven; the body follows
 *       via trail-buffer FK.  Ten color themes, bead rendering.
 *
 * CONTRAST WITH: snake_forward_kinematics.c — autonomous sinusoidal steering
 *               ik_tentacle_seek.c          — FABRIK IK with joint constraints
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color/themes  — 10 selectable palettes + dedicated HUD pair
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Snake: trail buffer, IK head, bead renderer
 *       §5a  trail helpers   — push, index, arc-length sampler
 *       §5b  move_head       — IK goal-seek + translate + wrap + record
 *       §5c  compute_joints  — body placement from trail
 *       §5d  bead rendering helpers
 *       §5e  draw_segment_beads — bead-fill for one segment
 *       §5f  render_chain    — full frame composition
 *   §6  scene         — scene_init / scene_tick / scene_draw wrappers
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW THIS SNAKE WORKS — IK GOAL-SEEKING + PATH-FOLLOWING FK
 * ────────────────────────────────────────────────────────────
 *
 * The body uses the same trail-buffer path-following FK as the forward
 * kinematics snake: each tick the head position is pushed into a circular
 * buffer, and body joints are placed at fixed arc-length intervals along
 * that buffer.  The body naturally traces every curve the head carves.
 *
 * The "IK" is in the HEAD STEERING:
 *   A multi-harmonic wandering target roams the screen along organic curves.
 *   actual_target lerps toward the wander point at 8× speed (smoothing).
 *   The head steers directly toward actual_target each tick:
 *
 *     turn = A1·sin(f1·t) + A2·sin(f2·t+φ2) + A3·sin(f3·t+φ3)
 *     tgt_dir   += turn × dt
 *     tgt_pos   += tgt_speed × (cos(tgt_dir), sin(tgt_dir)) × dt
 *     actual_target += (tgt_pos − actual_target) × min(dt × 8, 1)
 *     heading = atan2(actual_target.y − head.y, actual_target.x − head.x)
 *     head += move_speed × (cos(heading), sin(heading)) × dt
 *
 *   Three sine harmonics with incommensurable frequencies create a path that
 *   is smooth but never periodic — the target winds like river terrain, not
 *   a perfect mathematical figure.  Screen edges wrap toroidally.
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   ↑ / w         move speed faster
 *   ↓ / s         move speed slower
 *   + / =         target wander speed faster
 *   -             target wander speed slower
 *   t             next color theme
 *   T             previous color theme
 *   r / R         reset simulation
 *   ] / [         raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       snake_inverse_kinematics.c -o snake_ik -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : IK goal-seeking head + path-following FK body.
 *                  Head steers toward actual_target (a lerp-smoothed
 *                  wander position) at move_speed px/s.  Body joints
 *                  are placed by arc-length sampling of the head's recorded
 *                  trail — no per-joint angle formula required.
 *
 * Wander target  : The target drives itself like a mini-snake:
 *                  turn_rate = A1·sin(f1·t) + A2·sin(f2·t+φ2) + A3·sin(f3·t+φ3)
 *                  tgt_dir  += turn_rate × dt
 *                  tgt_pos  += tgt_speed × (cos, sin)(tgt_dir) × dt
 *                  Three frequencies (0.29, 0.71, 1.13 rad/s) are mutually
 *                  irrational → the path never repeats.  Amplitudes are tuned
 *                  so the target carves wide sweeping hills at low freq,
 *                  medium wiggles at mid, and small tremors at high.
 *
 * Smooth target  : actual_target lerps toward tgt_pos at rate dt×8.
 *                  This low-pass filter removes any residual discontinuities
 *                  (e.g. wrap edges) and keeps body curves gentle.
 *
 * Rendering      : Two-pass bead style.  Pass 1: each segment filled with
 *                  'o' at DRAW_STEP_PX intervals using a gradient pair.
 *                  Pass 2: joint markers ('0' head-quarter, 'o' mid, '.'
 *                  tail-quarter) stamped on top.  Head arrow drawn last.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

    N_PAIRS          =   7,   /* gradient color pairs for snake body        */
    HUD_PAIR         =   8,   /* dedicated HUD pair, never part of gradient */
    N_THEMES         =  10,

    N_SEGS           =  32,   /* rigid body segments                        */
    TRAIL_CAP        = 4096,  /* circular head-position history capacity    */

    /*
     * TARGET_TRAIL_CAP — ghost trail length for the wandering target.
     * The last 200 actual_target positions are drawn as dim dots, showing
     * enough of the winding path that the terrain-like character is visible.
     * At tgt_speed=80 px/s and 60 Hz: 200 ticks × 80/60 ≈ 267 px of trail
     * = ~33 columns — enough to see several hills and valleys.
     */
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
 * Wandering target parameters.
 *
 * The target steers itself via a superposition of three sine waves applied
 * to its heading (tgt_dir).  This models the kind of curvature a river or
 * mountain path makes — mostly smooth, with irregular undulations.
 *
 * TGT_WANDER_SPEED — target translation speed in px/s.
 *   At ~80 px/s the target crosses a 640 px screen in 8 s.  Slow enough
 *   for the snake to chase; fast enough to stay ahead of it.
 *   +/- keys adjust this at runtime in ×/÷ 1.25 steps.
 *
 * TGT_TURN_AMPn / TGT_TURN_FREQn — amplitude and angular frequency for
 *   each of three sine harmonics.
 *   Frequencies are mutually irrational (0.29, 0.71, 1.13) so the combined
 *   turn-rate waveform never exactly repeats.
 *   Amp1 (large, slow) → wide sweeping hills.
 *   Amp2 (medium)      → mid-scale wiggles overlaid on the hills.
 *   Amp3 (small, fast) → fine tremors for organic texture.
 *
 * TGT_TURN_PHASEn — initial phase offsets so the three waves start
 *   out of sync and the path is interesting from frame one.
 *
 * TGT_SMOOTH_RATE — lerp coefficient for actual_target → tgt_pos.
 *   Filters out wrap discontinuities; 8×/s is fast enough to track the
 *   target closely while still softening sharp turns.
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

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cell dimensions (physics↔display bridge, see §4) */
#define CELL_W   8
#define CELL_H  16

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
/* §3  color / themes                                                     */
/* ===================================================================== */

/*
 * Theme — one named color palette for the snake body.
 *
 * body[N_PAIRS]: xterm-256 foreground indices for pairs 1..N_PAIRS,
 *   mapped head (pair 1) → tail (pair 7) by seg_pair().
 * hud: foreground index for HUD_PAIR (pair 8).
 *
 * theme_apply() calls init_pair() live so switching themes takes effect on
 * the very next frame without restarting ncurses.
 */
typedef struct {
    const char *name;
    int         body[N_PAIRS];
    int         hud;
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      head←────────────────────────────→tail  hud  */
    {"Medusa", {57,  63,  93,  99,  105, 111, 159}, 226},
    {"Matrix", {22,  28,  34,  40,   46,  82, 118},  46},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}, 226},
    {"Ocean",  {17,  18,  19,  20,   21,  27,  51}, 123},
    {"Nova",   {54,  55,  56,  57,   93, 129, 165}, 201},
    {"Toxic",  {22,  58,  64,  70,   76,  82, 118},  82},
    {"Lava",   {52,  88, 124, 160,  196, 202, 208}, 196},
    {"Ghost",  {237, 238, 239, 240, 241, 250, 255}, 255},
    {"Aurora", {22,  28,  64,  71,   78, 121, 159}, 159},
    {"Neon",   {201, 165, 129,  93,   57,  51,  45}, 201},
};

/*
 * theme_apply() — (re-)initialise ncurses color pairs from THEMES[idx].
 * Safe to call at any time; ncurses updates pairs lazily on next draw.
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, th->body[p], COLOR_BLACK);
        init_pair(HUD_PAIR, th->hud, COLOR_BLACK);
    } else {
        /* 8-colour fallback: approximate with basic colours */
        static const int fb8[N_PAIRS] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
            COLOR_GREEN,  COLOR_CYAN,   COLOR_CYAN, COLOR_BLUE
        };
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, fb8[p], COLOR_BLACK);
        init_pair(HUD_PAIR, COLOR_YELLOW, COLOR_BLACK);
    }
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the aspect-ratio bridge                      */
/* ===================================================================== */

/*
 * All physics positions are in square pixel space (1 px = 1 physical pixel
 * in both axes).  Drawing converts to cell coordinates so the snake looks
 * isotropic regardless of terminal cell aspect ratio (CELL_H = 2×CELL_W).
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
/* §5  entity — Snake: trail buffer + IK head + bead renderer            */
/* ===================================================================== */

typedef struct { float x, y; } Vec2;

/*
 * Snake — complete simulation state.
 *
 * TRAIL BUFFER — same as snake_forward_kinematics.c:
 *   Circular buffer of head positions; trail_at(k) returns the entry
 *   k ticks back from newest.  Body joints sampled from this by
 *   arc-length (see trail_sample, compute_joints).
 *
 * IK FIELDS:
 *   tgt_time       simulation time accumulator for the wander harmonics
 *   tgt_speed      wander target translation speed (px/s); adjusted by +/-
 *   tgt_dir        current heading of the wander target (radians)
 *   tgt_pos        current pixel position of the wander target
 *   actual_target  smoothly-tracked position (lerp toward tgt_pos)
 *   heading        head's current travel direction (radians); head arrow
 *
 * TARGET TRAIL:
 *   tgt_trail[]    circular buffer of recent actual_target positions
 *   tgt_head       write pointer
 *   tgt_count      valid entries
 *
 * theme_idx — index into THEMES[]; adjusted by t/T keys.
 */
typedef struct {
    /* Trail buffer */
    Vec2  trail[TRAIL_CAP];
    int   trail_head;
    int   trail_count;

    /* Body joints */
    Vec2  joint[N_SEGS + 1];
    Vec2  prev_joint[N_SEGS + 1];

    /* IK / wander target */
    Vec2  actual_target;
    Vec2  tgt_pos;
    float tgt_time;
    float tgt_speed;
    float tgt_dir;
    float heading;
    float move_speed;

    /* Target ghost trail */
    Vec2  tgt_trail[TARGET_TRAIL_CAP];
    int   tgt_head;
    int   tgt_count;

    int   theme_idx;
    bool  paused;
} Snake;

/* ── §5a  trail helpers ─────────────────────────────────────────────── */

static void trail_push(Snake *s, Vec2 pos)
{
    s->trail_head = (s->trail_head + 1) % TRAIL_CAP;
    s->trail[s->trail_head] = pos;
    if (s->trail_count < TRAIL_CAP) s->trail_count++;
}

static inline Vec2 trail_at(const Snake *s, int k)
{
    return s->trail[(s->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample() — interpolated position at arc-length dist from head.
 * Walks the trail from newest entry, accumulating distances, until the
 * cumulative arc equals dist, then linearly interpolates.
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);
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

    return trail_at(s, s->trail_count - 1);
}

/* ── §5b  move_head — IK goal-seeking ──────────────────────────────── */

/*
 * tgt_push() — record actual_target into the ghost trail buffer.
 */
static void tgt_push(Snake *s, Vec2 pos)
{
    s->tgt_head = (s->tgt_head + 1) % TARGET_TRAIL_CAP;
    s->tgt_trail[s->tgt_head] = pos;
    if (s->tgt_count < TARGET_TRAIL_CAP) s->tgt_count++;
}

/*
 * move_head() — IK goal-seek: steer head toward actual_target.
 *
 * STEP 1 — advance wander target.
 *   tgt_time advances at 1 s/s (real time, not scaled).
 *   turn_rate = A1·sin(f1·t) + A2·sin(f2·t+φ2) + A3·sin(f3·t+φ3)
 *   tgt_dir  += turn_rate × dt      (integrating curvature → heading)
 *   tgt_pos  += tgt_speed × (cos, sin)(tgt_dir) × dt
 *   tgt_pos wraps toroidally so the target never stops or gets stuck at edges.
 *
 * STEP 2 — smooth actual_target toward tgt_pos.
 *   Low-pass lerp prevents snapping on wrap; keeps body curves gentle.
 *
 * STEP 3 — steer head toward actual_target.
 *   heading = atan2(dy, dx) where (dx,dy) = actual_target − head.
 *   Move at move_speed px/s, clamped so the head cannot overshoot.
 *
 * STEP 4 — toroidal screen wrap for head.
 *
 * STEP 5 — push new head position into trail.
 *   Also records actual_target in the ghost trail for rendering.
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    /* Step 1: advance wander target via multi-harmonic steering */
    s->tgt_time += dt;

    float turn = TGT_TURN_AMP1 * sinf(TGT_TURN_FREQ1 * s->tgt_time)
               + TGT_TURN_AMP2 * sinf(TGT_TURN_FREQ2 * s->tgt_time + TGT_TURN_PHASE2)
               + TGT_TURN_AMP3 * sinf(TGT_TURN_FREQ3 * s->tgt_time + TGT_TURN_PHASE3);
    s->tgt_dir += turn * dt;

    s->tgt_pos.x += s->tgt_speed * cosf(s->tgt_dir) * dt;
    s->tgt_pos.y += s->tgt_speed * sinf(s->tgt_dir) * dt;

    /* Toroidal wrap for wander target */
    if (s->tgt_pos.x <  0.0f) s->tgt_pos.x += wpx;
    if (s->tgt_pos.x >= wpx)  s->tgt_pos.x -= wpx;
    if (s->tgt_pos.y <  0.0f) s->tgt_pos.y += hpx;
    if (s->tgt_pos.y >= hpx)  s->tgt_pos.y -= hpx;

    /* Step 2: smooth actual_target toward tgt_pos */
    float k = dt * TGT_SMOOTH_RATE;
    if (k > 1.0f) k = 1.0f;
    s->actual_target.x += (s->tgt_pos.x - s->actual_target.x) * k;
    s->actual_target.y += (s->tgt_pos.y - s->actual_target.y) * k;

    /* Step 3: steer head toward actual_target */
    float dx   = s->actual_target.x - s->joint[0].x;
    float dy   = s->actual_target.y - s->joint[0].y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > 0.5f) {
        s->heading  = atan2f(dy, dx);
        float step  = s->move_speed * dt;
        if (step > dist) step = dist;   /* don't overshoot */
        s->joint[0].x += (dx / dist) * step;
        s->joint[0].y += (dy / dist) * step;
    }

    /* Step 4: toroidal wrap */
    if (s->joint[0].x <  0.0f) s->joint[0].x += wpx;
    if (s->joint[0].x >= wpx)  s->joint[0].x -= wpx;
    if (s->joint[0].y <  0.0f) s->joint[0].y += hpx;
    if (s->joint[0].y >= hpx)  s->joint[0].y -= hpx;

    /* Step 5: record into trail and target ghost trail */
    trail_push(s, s->joint[0]);
    tgt_push(s, s->actual_target);
}

/* ── §5c  compute_joints ────────────────────────────────────────────── */

/*
 * compute_joints() — place body joints by arc-length sampling of the trail.
 * Identical to snake_forward_kinematics.c: joint[i] is placed at distance
 * i × SEG_LEN_PX behind the head, measured along the actual path taken.
 */
static void compute_joints(Snake *s)
{
    for (int i = 1; i <= N_SEGS; i++)
        s->joint[i] = trail_sample(s, (float)i * SEG_LEN_PX);
}

/* ── §5d  bead rendering helpers ────────────────────────────────────── */

/*
 * seg_pair() — color pair index for body segment i (head=1, tail=N_PAIRS).
 * Linear interpolation: pair 1 (head) → pair N_PAIRS (tail tip).
 */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/*
 * seg_attr() — ncurses attribute for body segment i.
 * Head quarter: A_BOLD (bright, draws the eye).
 * Tail quarter: A_DIM (fades into background, emphasises tail-end).
 */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_BOLD;
    if (i > 3 * N_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/*
 * joint_node_char() — bead marker at joint position i.
 * Head third:  '0' (thick, prominent node)
 * Middle:      'o' (standard bead)
 * Tail third:  '.' (small, receding)
 */
static chtype joint_node_char(int i)
{
    if (i <= (N_SEGS - 1) / 3)    return '0';
    if (i >= (N_SEGS - 1) * 2 / 3) return '.';
    return 'o';
}

/*
 * head_glyph() — directional arrow at the snake's head.
 * Maps heading (radians) to >, v, <, ^ based on 90° quadrants.
 */
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

/* ── §5e  draw_segment_beads ────────────────────────────────────────── */

/*
 * draw_segment_beads() — fill segment a→b with 'o' at DRAW_STEP_PX steps.
 *
 * Pass 1 of the bead render.  Each cell touched by the walk receives an 'o'
 * character using the gradient pair for segment i.  Pass 2 (in render_chain)
 * then overwrites joint positions with '0' / 'o' / '.' node markers.
 *
 * No inter-segment dedup cursor is needed here because 'o' everywhere is
 * correct; the node markers in pass 2 provide the visual articulation.
 */
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
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, 'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5f  render_chain ──────────────────────────────────────────────── */

/*
 * render_chain() — compose the full snake frame (bead style).
 *
 * STEP 1 — sub-tick alpha interpolation.
 *   rj[i] = lerp(prev_joint[i], joint[i], alpha) for smooth motion.
 *
 * STEP 2 — draw target ghost trail.
 *   Recent actual_target positions rendered as dim '.' dots (HUD_PAIR | A_DIM).
 *   Oldest entries are drawn first so newer ones win on overlap.
 *
 * STEP 3 — draw Lissajous target cursor.
 *   actual_target drawn as bright '+' so the user can see what the head
 *   is chasing.
 *
 * STEP 4 — body segments, bead fill, drawn tail → head (pass 1).
 *   Tail-first ordering: head segments drawn last win any overlap.
 *
 * STEP 5 — joint node markers (pass 2).
 *   '0' / 'o' / '.' stamped at each rj[i]; head arrow at rj[0] last.
 */
static void render_chain(const Snake *s, WINDOW *w,
                          int cols, int rows, float alpha)
{
    /* Step 1 — interpolated render positions */
    Vec2 rj[N_SEGS + 1];
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = s->prev_joint[i].x
                + (s->joint[i].x - s->prev_joint[i].x) * alpha;
        rj[i].y = s->prev_joint[i].y
                + (s->joint[i].y - s->prev_joint[i].y) * alpha;
    }

    /* Step 2 — target ghost trail (oldest first) */
    wattron(w, COLOR_PAIR(HUD_PAIR) | A_DIM);
    int n = s->tgt_count;
    for (int k = n - 1; k >= 1; k--) {
        int idx = (s->tgt_head + TARGET_TRAIL_CAP - k) % TARGET_TRAIL_CAP;
        int cx = px_to_cell_x(s->tgt_trail[idx].x);
        int cy = px_to_cell_y(s->tgt_trail[idx].y);
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows)
            mvwaddch(w, cy, cx, '.');
    }
    wattroff(w, COLOR_PAIR(HUD_PAIR) | A_DIM);

    /* Step 3 — Lissajous target cursor */
    {
        int cx = px_to_cell_x(s->actual_target.x);
        int cy = px_to_cell_y(s->actual_target.y);
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            wattron(w, COLOR_PAIR(HUD_PAIR) | A_BOLD);
            mvwaddch(w, cy, cx, '+');
            wattroff(w, COLOR_PAIR(HUD_PAIR) | A_BOLD);
        }
    }

    /* Step 4 — bead fill: tail → head so head-end wins overlaps */
    for (int i = N_SEGS - 1; i >= 0; i--) {
        draw_segment_beads(w,
                           rj[i + 1], rj[i],
                           seg_pair(i), seg_attr(i),
                           cols, rows);
    }

    /* Step 5 — joint node markers: tail → head (overwrite fill beads) */
    for (int i = N_SEGS; i >= 1; i--) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        int   p  = seg_pair(i - 1);   /* use segment i-1 colour at joint i */
        attr_t a = seg_attr(i - 1);
        wattron(w, COLOR_PAIR(p) | a);
        mvwaddch(w, cy, cx, joint_node_char(i));
        wattroff(w, COLOR_PAIR(p) | a);
    }

    /* Head arrow drawn last — always on top */
    {
        int cx = px_to_cell_x(rj[0].x);
        int cy = px_to_cell_y(rj[0].y);
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            wattron(w, COLOR_PAIR(1) | A_BOLD);
            mvwaddch(w, cy, cx, head_glyph(s->heading));
            wattroff(w, COLOR_PAIR(1) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Snake snake; } Scene;

/*
 * scene_init() — initialise snake to a clean, immediately-animated state.
 *
 * theme_idx is preserved across reset: saved before memset, restored after,
 * so r/R doesn't jump back to theme 0 unexpectedly.
 *
 * The trail is pre-populated (TRAIL_CAP entries, 1 px apart, extending
 * behind the head) so body joints are valid from frame one.
 *
 * actual_target and tgt_pos are initialised to the head position so
 * there is no initial snap; the wander target diverges naturally from
 * frame one as the harmonic steering accumulates.
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
    s->tgt_dir    = (float)M_PI / 6.0f;   /* start heading slightly SE */
    s->heading    = 0.0f;
    s->paused     = false;

    /* Head at screen centre */
    s->joint[0].x = (float)(cols * CELL_W) * 0.5f;
    s->joint[0].y = (float)(rows * CELL_H) * 0.5f;

    /* Wander target starts near head (offset slightly so it leads) */
    s->tgt_pos       = s->joint[0];
    s->actual_target = s->joint[0];

    /*
     * Pre-populate trail: extend behind head pointing west (heading 0 → east,
     * so backward is west: bx = cos(π) = -1, by = 0).
     * 1 px spacing covers the full snake body length from frame one.
     */
    float bx = -1.0f;   /* unit vector pointing west */
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
    memcpy(s->prev_joint, s->joint, sizeof s->joint);   /* save for α lerp */
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

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame: scene + HUD bars.
 *
 * Top-right HUD:  fps · simHz · speed · liss_speed · theme · state
 * Bottom hint:    keyboard reference for all controls
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Snake *sn = &sc->snake;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %dHz  spd:%.0f  tgt:%.0f  [%s]  %s ",
             fps, sim_fps,
             sn->move_speed,
             sn->tgt_speed,
             THEMES[sn->theme_idx].name,
             sn->paused ? "PAUSED" : "chasing");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  UD/ws:spd  +/-:tgt-spd  t/T:theme  r:reset  [/]:Hz ");
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
    Snake *s   = &app->scene.snake;
    float  wpx = (float)(app->screen.cols * CELL_W);
    float  hpx = (float)(app->screen.rows * CELL_H);
    if (s->joint[0].x >= wpx) s->joint[0].x = wpx - 1.0f;
    if (s->joint[0].y >= hpx) s->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle paused
 *   ↑ / w / W     move_speed × 1.20
 *   ↓ / s / S     move_speed ÷ 1.20  (clamped [MOVE_SPEED_MIN, MAX])
 *   + / =         liss_speed × 1.25
 *   -             liss_speed ÷ 1.25  (clamped [LISS_SPEED_MIN, MAX])
 *   t             next theme (wraps)
 *   T             previous theme (wraps)
 *   r / R         reset simulation (theme preserved)
 *   ]             sim_fps + step     (clamped [SIM_FPS_MIN, MAX])
 *   [             sim_fps - step
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

/* ─────────────────────────────────────────────────────────────────────
 * main() — game loop (structure identical to framework.c §8)
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

    screen_init(&app->screen, 0);   /* theme 0 = Medusa on startup */
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
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

        /* ── ⑥ frame cap ─────────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
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
