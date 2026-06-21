/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_arm_reach.c — a 4-link arm that reaches for a moving dot.
 *
 * Inverse kinematics: you say where the hand should reach and the
 * solver works out the joint angles. Here a self-driving target traces
 * a figure-8 and the arm bends to keep its tip on it. The solver is
 * FABRIK: drag the chain so the tip lands on the target, then drag it
 * back so the shoulder stays put; repeat until it settles. When the
 * target drifts out of reach, the arm just points at it and a yellow
 * circle shows how far it can stretch.
 *
 * Sister files: ik_tentacle_seek.c, snake_inverse_kinematics.c (same
 *               FABRIK solver on longer chains).
 * FABRIK from: Aristidou & Lasenby (2011). License: MIT (see line 1).
 *
 * Keys:  q / ESC  quit     space  pause     +/-  target speed
 *        t  cycle theme     [ / ]  time scale (0.25x .. 4x)
 *
 * Build (-lm for the trig — cosf/sinf/sqrtf):
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_arm_reach.c \
 *       -o ik_arm_reach -lncurses -lm
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
    /* Frame-rate cap. Timestep is variable, so this only sets how long
     * we sleep at the end of each frame. */
    TARGET_FPS    = 60,

    HUD_COLS      = 96,    /* max status-bar width */
    FPS_UPDATE_MS = 500,   /* how often the fps readout refreshes */

    /* ncurses colour-pair IDs:
     *   1..N_ARM_COLORS  arm gradient (root dark to tip bright), themed
     *   6                target marker — always bright red
     *   7                reach circle  — always yellow
     *   8 PAIR_HUD       status bar    — bright yellow
     *   9 PAIR_HINT      key hint       — bright cyan */
    N_ARM_COLORS = 5,
    PAIR_HUD     = 8,
    PAIR_HINT    = 9,

    N_THEMES     = 10,    /* arm palettes, cycled with `t` */

    /* The chain: N_LINKS bones, N_JOINTS = N_LINKS + 1 joints between
     * them. pos[0] is the root (fixed); pos[N_JOINTS-1] is the tip that
     * chases the target; the joints between are free to bend. */
    N_JOINTS     = 5,
    N_LINKS      = 4,

    /* Most a 4-link chain needs is 3-5 FABRIK passes; this caps the
     * worst case (a tightly folded chain) so a frame can't run away. */
    MAX_ITER     = 15,

    /* How many past target positions to keep for the dotted trail.
     * At ~60 ticks/s this is about 1 second — enough to read the
     * figure-8 shape without cluttering the screen. */
    TRAIL_SIZE   = 60,

    /* Dots drawn around the reach circle. 48 reads as a dashed ring
     * without the dots overlapping at typical arm sizes. */
    REACH_CIRCLE_SAMPLES = 48,
};

/* "Close enough" for the tip: once it lands within 1.5 px of the
 * target the solver stops. That's under one cell, so more passes
 * would change nothing you can see. (cells are 8x16 px.) */
#define CONV_TOL    1.5f

/*
 * The target's figure-8 path (a Lissajous curve):
 *   x = root.x + lis_ax * cos(LIS_FX * t)
 *   y = root.y + lis_ay * sin(LIS_FY * t + LIS_PHASE)
 * A 1:2 frequency ratio gives one self-crossing — the figure-8. The
 * phase shift makes that crossing a clean X instead of a cusp. The
 * amplitudes lis_ax/lis_ay are set at scene_init from the screen size.
 */
#define LIS_FX               1.0f
#define LIS_FY               2.0f
#define LIS_PHASE            0.785f       /* about pi/4 */

/* Target speed (the +/- keys scale it).
 *   0.7 -> a leisurely ~9 s per figure-8 lap.
 *   clamped to [0.05, 5.0] -> ~125 s down to ~1.3 s per lap. */
#define LIS_SPEED_DEFAULT    0.7f
#define LIS_SPEED_MIN        0.05f
#define LIS_SPEED_MAX        5.0f

/* Spacing of the 'o' beads stamped along each bone when drawing. Kept
 * below CELL_W (8) so no cell is skipped; 5 px leaves small gaps so
 * the joint markers stay visible through the fill. */
#define DRAW_STEP_PX   5.0f

/* Time scale (the [ and ] keys): slows down or speeds up the whole
 * simulation, separate from the target's own speed. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Sub-pixels per character cell. The terminal is taller than wide per
 * cell, so positions are kept in square px and converted at draw time. */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic time + sleep ── */

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

/* ── §3 color — arm palettes + fixed HUD colours ── */

/*
 * Theme — one arm colour ramp (five xterm-256 indices, root to tip).
 *
 * Switching theme rebinds only the five arm pairs (1..5). The target
 * (red) and reach circle (yellow) keep their colours across every theme
 * because those colours mean something — you learn them once. The five
 * indices go dim->bright so the eye can trace the chain from shoulder to
 * tip without numbering the joints. Every index stays in the bright half
 * of the palette so nothing vanishes when drawn with A_DIM.
 */
typedef struct {
    const char *name;              /* shown in the HUD                  */
    int         arm[N_ARM_COLORS]; /* fg indices, root to tip;          *
                                    * arm[i] becomes colour-pair (i+1)  */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "Steel",  {240, 244, 248, 252,  51} },
    { "Matrix", { 24,  28,  34,  40,  46} },
    { "Fire",   { 52,  88, 124, 160, 196} },
    { "Ocean",  { 24,  25,  27,  33,  51} },
    { "Nova",   { 54,  93, 129, 165, 201} },
    { "Toxic",  { 58,  64,  70,  76,  82} },
    { "Lava",   { 52,  94, 130, 166, 202} },
    { "Ghost",  {240, 244, 246, 250, 254} },
    { "Aurora", { 24,  35,  71, 107, 143} },
    { "Neon",   { 57,  93, 129, 165, 201} },
};

/* Point the five arm colour-pairs at the chosen theme. The target,
 * reach, and HUD pairs are left alone — their colours never change. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_ARM_COLORS; p++)
        init_pair(p + 1, t->arm[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
        init_pair(6, 196, -1);   /* bright red — target marker */
        init_pair(7, 226, -1);   /* yellow     — reach circle  */
    } else {
        /* terminals with only 8 colours */
        init_pair(1, COLOR_WHITE,  -1);
        init_pair(2, COLOR_WHITE,  -1);
        init_pair(3, COLOR_WHITE,  -1);
        init_pair(4, COLOR_WHITE,  -1);
        init_pair(5, COLOR_CYAN,   -1);
        init_pair(6, COLOR_RED,    -1);
        init_pair(7, COLOR_YELLOW, -1);
    }

    /* Bright yellow status bar, bright cyan hint — fixed so they stay
     * readable over any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — pixel to cell conversion ── */

/*
 * Positions are stored in square pixels; the terminal's cells aren't
 * square (8 wide, 16 tall), so these turn a pixel position into the
 * cell that contains it, rounding to the nearest. This is the only
 * place that conversion happens.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the Arm (chain + target) ── */

/*
 * Vec2 — a point in pixel space.
 *   x grows rightward, y grows downward (screen convention).
 * The whole simulation runs in pixels and only converts to cells when
 * drawing. FABRIK never looks at directions as angles, only as
 * positions and distances, so y-pointing-down doesn't matter to it.
 */
typedef struct {
    float x;   /* pixels, positive = right */
    float y;   /* pixels, positive = down  */
} Vec2;

/* ── §5a  vec2 helpers ── */

static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

/*
 * Unit vector (same direction, length 1). If the input is (near) zero
 * there's no direction to return, so fall back to (1,0) — any direction
 * works, because the next FABRIK pass pulls coincident joints apart.
 */
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){1.0f, 0.0f};
    return (Vec2){v.x / l, v.y / l};
}

/*
 * Arm — everything about the arm and the dot it chases, in one record.
 * Two independent halves: the chain of joints (solved by FABRIK each
 * frame) and the self-driving target (a figure-8). They share nothing
 * but the target's position: the dot moves on its own, the chain just
 * reacts to wherever it ended up.
 */
typedef struct {
    /* ── The chain (FABRIK overwrites pos[] every frame) ── */
    Vec2  pos     [N_JOINTS];  /* joints: pos[0]=root (fixed), last=tip */
    float link_len[N_LINKS];   /* bone lengths, in px; set once at init */
    float total_len;           /* sum of link_len; the reach radius     */

    /* ── Root position (also the centre of the figure-8) ──
     * Recomputed on resize so the arm re-centres on the new screen. */
    float root_px;             /* root x, px */
    float root_py;             /* root y, px */

    /* ── The moving target ──
     * target is just a function of scene_time; pausing stops the clock
     * so the dot resumes exactly where it left off. */
    Vec2  target;              /* where the tip is trying to reach   */
    float scene_time;          /* the figure-8 clock, in seconds     */
    float speed_scale;         /* target speed, 0.05..5.0 (+/- keys)  */
    float lis_ax;              /* figure-8 half-width, px            */
    float lis_ay;              /* figure-8 half-height, px           */

    /* ── Trail: recent target positions, drawn as a faint dotted path.
     * A ring buffer: trail_head is the newest, older ones wrap around
     * and get overwritten. trail_count is how many are filled in so far
     * (so frame 1 doesn't draw garbage), maxing out at TRAIL_SIZE. */
    Vec2  trail[TRAIL_SIZE];
    int   trail_head;          /* index of newest entry */
    int   trail_count;         /* filled entries, up to TRAIL_SIZE */

    /* ── Flags ── */
    bool  at_limit;            /* target out of reach (recomputed each frame) */
    bool  paused;              /* spacebar: freeze the whole sim */
    int   theme_idx;           /* which THEMES[] palette is active */
} Arm;

/* ── §5b  FABRIK solver ── */

/*
 * Target's too far: lay all the joints in a straight line pointing at
 * it. The tip can't reach, but the arm at least points the right way.
 */
static void stretch_arm_straight(Arm *a, Vec2 root, Vec2 target)
{
    Vec2 dir   = vec2_norm((Vec2){ target.x - root.x, target.y - root.y });
    a->pos[0]  = root;
    for (int i = 0; i < N_LINKS; i++) {
        a->pos[i + 1].x = a->pos[i].x + dir.x * a->link_len[i];
        a->pos[i + 1].y = a->pos[i].y + dir.y * a->link_len[i];
    }
}

/*
 * Forward pass: snap the tip onto the target, then walk back toward the
 * root, pulling each joint into line so every bone keeps its length.
 * Afterward the tip is exactly on target, but the root has slid off its
 * anchor — the backward pass fixes that.
 */
static void fabrik_forward_pass(Arm *a, Vec2 target)
{
    a->pos[N_JOINTS - 1] = target;
    for (int i = N_JOINTS - 2; i >= 0; i--) {
        float fx   = a->pos[i].x - a->pos[i + 1].x;
        float fy   = a->pos[i].y - a->pos[i + 1].y;
        float flen = sqrtf(fx * fx + fy * fy);
        if (flen < 1e-6f) flen = 1e-6f;     /* two joints on top of each other */
        float r    = a->link_len[i] / flen;
        a->pos[i].x = a->pos[i + 1].x + fx * r;
        a->pos[i].y = a->pos[i + 1].y + fy * r;
    }
}

/*
 * Backward pass: snap the root back onto its anchor, then walk out to
 * the tip the same way. Now the root is right and the bones are right,
 * but the tip has drifted off the target — by less than last time.
 * Alternating the two passes closes that gap each round.
 */
static void fabrik_backward_pass(Arm *a, Vec2 root)
{
    a->pos[0] = root;
    for (int i = 0; i < N_JOINTS - 1; i++) {
        float bx   = a->pos[i + 1].x - a->pos[i].x;
        float by   = a->pos[i + 1].y - a->pos[i].y;
        float blen = sqrtf(bx * bx + by * by);
        if (blen < 1e-6f) blen = 1e-6f;
        float r    = a->link_len[i] / blen;
        a->pos[i + 1].x = a->pos[i].x + bx * r;
        a->pos[i + 1].y = a->pos[i].y + by * r;
    }
}

/* How far the tip still is from the target — the solver's stop test. */
static float tip_target_distance(const Arm *a, Vec2 target)
{
    float tdx = a->pos[N_JOINTS - 1].x - target.x;
    float tdy = a->pos[N_JOINTS - 1].y - target.y;
    return sqrtf(tdx * tdx + tdy * tdy);
}

/*
 * Solve the whole arm for this frame. If the target sits farther than
 * the arm can stretch, just point at it. Otherwise run forward/backward
 * passes until the tip is within CONV_TOL or we hit the pass cap.
 */
static void fabrik_solve(Arm *a)
{
    Vec2 root   = { a->root_px, a->root_py };
    Vec2 target = a->target;

    Vec2  drt   = { target.x - root.x, target.y - root.y };
    float dist  = vec2_len(drt);

    if (dist > a->total_len) {
        a->at_limit = true;
        stretch_arm_straight(a, root, target);
        return;
    }

    a->at_limit = false;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        fabrik_forward_pass (a, target);
        fabrik_backward_pass(a, root);
        if (tip_target_distance(a, target) < CONV_TOL) break;
    }
}

/* ── §5c  target motion — the figure-8 ── */

/* Record the latest target position in the trail ring, dropping the
 * oldest if the ring is full. */
static void trail_push(Arm *a, Vec2 pos)
{
    a->trail_head = (a->trail_head + 1) % TRAIL_SIZE;
    a->trail[a->trail_head] = pos;
    if (a->trail_count < TRAIL_SIZE) a->trail_count++;
}

/*
 * Move the target along its figure-8 for this frame: advance the clock,
 * plug it into the curve equations (see §1 LIS_*), and log the trail.
 */
static void update_target(Arm *a, float dt)
{
    a->scene_time += dt * a->speed_scale;

    a->target.x = a->root_px
                + a->lis_ax * cosf(LIS_FX * a->scene_time);
    a->target.y = a->root_py
                + a->lis_ay * sinf(LIS_FY * a->scene_time + LIS_PHASE);

    trail_push(a, a->target);
}

/* ── §5d  rendering helpers ── */

/*
 * Draw one bone as a line of 'o' beads from a to b. We step along it in
 * pixel-sized hops smaller than a cell so no cell along the line is
 * missed, skipping repeats when several hops land in the same cell.
 */
static void draw_link_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr,
                            int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* Draw one glyph at a pixel position, ignoring it if off-screen. */
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

/*
 * Pick a joint's glyph by where it sits in the chain. Big to small from
 * root to tip, so the chain reads as a hierarchy and the tip looks like
 * a fine gripper.
 */
static chtype joint_marker(int i)
{
    if (i == 0)              return (chtype)(unsigned char)'0';   /* root */
    if (i == N_JOINTS - 1)   return (chtype)(unsigned char)'.';   /* tip  */
    if (i <=     N_JOINTS / 2) return (chtype)(unsigned char)'0'; /* near root */
    return                          (chtype)(unsigned char)'o';   /* near tip  */
}

/* Pick a joint's colour so its marker roughly matches the bone it sits
 * on. The tip jumps to the brightest pair (5) for contrast. */
static int joint_pair(int i)
{
    if (i == 0)             return 1;   /* root */
    if (i == N_JOINTS - 1)  return 5;   /* tip  */
    return i + 1;                       /* interior joints */
}

/* ── §5e  render_arm — draw the whole arm ── */

/* The faint dotted figure-8 path behind the live target. */
static void draw_trail(const Arm *a, WINDOW *w, int cols, int rows)
{
    for (int k = 0; k < a->trail_count; k++) {
        int idx = (a->trail_head + TRAIL_SIZE - a->trail_count + 1 + k)
                  % TRAIL_SIZE;
        mark_cell(w, a->trail[idx], (chtype)(unsigned char)'.',
                  6, A_DIM, cols, rows);
    }
}

/* The dashed yellow ring showing how far the arm can reach — only drawn
 * when the target has escaped past it. */
static void draw_reach_circle(const Arm *a, WINDOW *w, int cols, int rows)
{
    if (!a->at_limit) return;

    Vec2  root = { a->root_px, a->root_py };
    float pi2  = 2.0f * (float)M_PI;

    for (int k = 0; k < REACH_CIRCLE_SAMPLES; k++) {
        float angle = (float)k * pi2 / (float)REACH_CIRCLE_SAMPLES;
        Vec2  p = { root.x + a->total_len * cosf(angle),
                    root.y + a->total_len * sinf(angle) };
        mark_cell(w, p, (chtype)(unsigned char)'.', 7, A_DIM, cols, rows);
    }
}

/* The four bones, each in its gradient colour. The last bone jumps
 * straight to the brightest pair (5) to set the tip apart. */
static void draw_links(const Arm *a, WINDOW *w, int cols, int rows)
{
    static const int link_pairs[N_LINKS] = { 1, 2, 3, 5 };
    for (int i = 0; i < N_LINKS; i++) {
        attr_t attr = (i == 0 || i == N_LINKS - 1) ? A_BOLD : A_NORMAL;
        draw_link_beads(w, a->pos[i], a->pos[i + 1],
                        link_pairs[i], attr, cols, rows);
    }
}

/* A bold node marker at every joint, drawn over the bones. */
static void draw_joints(const Arm *a, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_JOINTS; i++)
        mark_cell(w, a->pos[i], joint_marker(i),
                  joint_pair(i), A_BOLD, cols, rows);
}

/* The target marker on top of everything: '+' when the arm can reach
 * it, 'X' when it can't. Always bright red so it never gets lost. */
static void draw_target(const Arm *a, WINDOW *w, int cols, int rows)
{
    chtype glyph = a->at_limit
                 ? (chtype)(unsigned char)'X'
                 : (chtype)(unsigned char)'+';
    mark_cell(w, a->target, glyph, 6, A_BOLD, cols, rows);
}

/* Draw the whole arm, back to front, so later layers cover earlier
 * ones: trail, reach circle, bones, joints, then the target marker. */
static void render_arm(const Arm *a, WINDOW *w, int cols, int rows)
{
    draw_trail        (a, w, cols, rows);
    draw_reach_circle (a, w, cols, rows);
    draw_links        (a, w, cols, rows);
    draw_joints       (a, w, cols, rows);
    draw_target       (a, w, cols, rows);
}

/* ── §6 scene — the world (just the one arm) ── */

/*
 * Scene — the whole simulated world. Here that's just one Arm, but the
 * wrapper keeps init/tick/draw looking the same as every other demo in
 * the repo, and gives obstacles or a second target somewhere to go later.
 */
typedef struct {
    Arm arm;                   /* chain + target + trail */
} Scene;

/*
 * Build the arm centred on the screen, sized to the current terminal.
 *  - The four bone lengths shrink from root to tip and add up to the
 *    full reach; the taper mimics an upper-arm/forearm/hand and helps
 *    the eye read the chain.
 *  - The figure-8 is sized so its widest points sit just past the reach,
 *    so the arm hits its limit (and flashes the reach circle) each lap.
 *  - The arm starts laid out straight to the right; FABRIK reshapes it
 *    on the first frame.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Arm *a = &sc->arm;

    float sw = (float)(cols * CELL_W);
    float sh = (float)(rows * CELL_H);

    /* Full reach = 60% of the shorter screen side. The four weights
     * add to 1.0, so the bones add up to exactly that reach. */
    float arm_reach = (sw < sh ? sw : sh) * 0.60f;
    a->link_len[0]  = arm_reach * 0.32f;   /* longest — root bone */
    a->link_len[1]  = arm_reach * 0.27f;
    a->link_len[2]  = arm_reach * 0.23f;
    a->link_len[3]  = arm_reach * 0.18f;   /* shortest — tip bone */
    a->total_len    = arm_reach;

    a->root_px = sw * 0.50f;
    a->root_py = sh * 0.50f;

    /* Figure-8 size: 40% of each screen axis, but never more than 90%
     * of the reach (so the extremes poke just past what the arm can do). */
    float max_amp = a->total_len * 0.90f;
    a->lis_ax     = sw * 0.40f;
    a->lis_ay     = sh * 0.40f;
    if (a->lis_ax > max_amp) a->lis_ax = max_amp;
    if (a->lis_ay > max_amp) a->lis_ay = max_amp;

    a->scene_time  = 0.0f;
    a->speed_scale = LIS_SPEED_DEFAULT;

    /* Starting pose: joints in a straight line to the right of the root. */
    a->pos[0] = (Vec2){ a->root_px, a->root_py };
    for (int i = 1; i < N_JOINTS; i++) {
        a->pos[i].x = a->pos[i - 1].x + a->link_len[i - 1];
        a->pos[i].y = a->pos[i - 1].y;
    }

    /* Where the target starts, at time 0 on the figure-8. */
    a->target.x = a->root_px + a->lis_ax;
    a->target.y = a->root_py + a->lis_ay * sinf(LIS_PHASE);

    a->trail_head  = 0;
    a->trail_count = 0;
    a->at_limit    = false;
    a->paused      = false;
    a->theme_idx   = 0;
}

/* One frame of simulation: move the target, then solve the arm to it.
 * dt is seconds since the last frame (already time-scaled). */
static void scene_tick(Scene *sc, float dt)
{
    Arm *a = &sc->arm;
    if (a->paused) return;

    update_target(a, dt);    /* move the dot along its figure-8 */
    fabrik_solve (a);        /* bend the arm so the tip reaches it */
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_arm(&sc->arm, w, cols, rows);
}

/* ── §7 screen — ncurses display + HUD ── */

/*
 * Screen — the current terminal size, in cells. Cached here so the rest
 * of the code reads plain ints instead of querying ncurses every frame;
 * refreshed on resize. (Pixels are a §5 thing; ncurses works in cells.)
 */
typedef struct {
    int cols;   /* terminal width in cells  */
    int rows;   /* terminal height in cells */
} Screen;

/* typeahead(-1) is the one to know: without it ncurses checks stdin
 * mid-write and can tear the frame. */
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

/* On resize: endwin()+refresh() makes ncurses re-read the new size. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Paint one full frame: clear, draw the arm, then the status line
 * (top-right) and the key hint (bottom). */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Arm *a = &sc->arm;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  reach:%s  spd:%.2f  theme:%s  %.2fx  %.1ffps  %s ",
             a->at_limit ? "LIMIT" : "NEAR ",
             a->speed_scale,
             THEMES[a->theme_idx].name,
             time_scale, fps,
             a->paused ? "PAUSED" : "tracking");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  +/-:speed  t:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, main loop ── */

/*
 * App — everything outside the simulated world: the Scene, the Screen,
 * and the loop's control flags, in one record. It's a file-scope global
 * (g_app) so the signal handlers can reach it — they get no argument.
 *
 * running and need_resize are written from signal handlers, so they're
 * volatile sig_atomic_t: volatile forces a re-read each loop instead of
 * caching, and sig_atomic_t is the one int type guaranteed to be written
 * in one piece even when a signal interrupts.
 */
typedef struct {
    Scene  scene;              /* the world (§6)        */
    Screen screen;             /* terminal size (§7)    */

    float                 time_scale;   /* sim speed; 1.0 = realtime ([ ] keys) */
    volatile sig_atomic_t running;      /* loop runs while set; SIGINT/TERM clears */
    volatile sig_atomic_t need_resize;  /* SIGWINCH sets it; handled next frame */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * Rebuild the arm for the new terminal size after a resize. Geometry is
 * recomputed from scratch, but the theme, speed, and figure-8 clock are
 * saved and restored so the animation carries on where it was.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    int   saved_theme = app->scene.arm.theme_idx;
    float saved_speed = app->scene.arm.speed_scale;
    float saved_time  = app->scene.arm.scene_time;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.arm.theme_idx   = saved_theme;
    app->scene.arm.speed_scale = saved_speed;
    app->scene.arm.scene_time  = saved_time;

    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Arm *a = &app->scene.arm;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': a->paused = !a->paused; break;

    case '+': case '=':
        a->speed_scale *= 1.25f;
        if (a->speed_scale > LIS_SPEED_MAX) a->speed_scale = LIS_SPEED_MAX;
        break;
    case '-':
        a->speed_scale /= 1.25f;
        if (a->speed_scale < LIS_SPEED_MIN) a->speed_scale = LIS_SPEED_MIN;
        break;

    case 't': case 'T':
        a->theme_idx = (a->theme_idx + 1) % N_THEMES;
        theme_apply(a->theme_idx);
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
 * The render loop. Each frame: read keys, handle a pending resize,
 * measure how long the last frame took, advance the sim by that much,
 * update the fps readout, draw, then sleep to hold ~TARGET_FPS.
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

        /* ③ measure dt — capped at 100 ms so the arm doesn't lurch
         *    after the program was paused or the laptop slept */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale);

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

        /* ⑦ frame cap — sleep off whatever time is left in the budget */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
