/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * chain.c — Hanging Chain & Swinging Rope
 *
 * A chain of point masses connected by inextensible links, simulated with
 * Position-Based Dynamics (PBD):
 *
 *   1. Verlet predict:  x_pred = x + (x - x_old)*DAMP + g*dt²
 *   2. Project constraints (N_ITER times):
 *        for each link (a, b):
 *          d = x_pred[b] - x_pred[a]
 *          corr = (|d| - rest) / |d|
 *          x_pred[a] += 0.5 * corr * d   (split equally; skip if pinned)
 *          x_pred[b] -= 0.5 * corr * d
 *   3. Derive velocity: v ≈ (x_pred - x_old) / dt  (implicit in Verlet)
 *   4. x_old = x; x = x_pred
 *
 * PBD is unconditionally stable for any stiffness — no spring constant to
 * tune for stability. More iterations = stiffer chain.
 *
 * Presets:
 *   0  Hanging   — pinned at top, gravity + sinusoidal wind
 *   1  Pendulum  — pinned at top, released from 60° angle; swings freely
 *   2  Bridge    — pinned at both ends, catenary sag; gust from below
 *   3  Wave      — top node driven sinusoidally; standing waves emerge
 *
 * Keys:
 *   q / ESC    quit
 *   space / p  pause / resume
 *   r          reset current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   + / -      more / fewer constraint iterations (stiffness)
 *   w          toggle wind / gust
 *   ] / [      simulation FPS up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/chain.c -o chain -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 physics
 *           §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Position-Based Dynamics (PBD / XPBD)
 *                 Instead of integrating forces to get acceleration and then
 *                 velocity, PBD works directly on positions.  Each constraint
 *                 (link length) is projected to satisfaction by moving the
 *                 endpoints.  Repeating N_ITER times → stiffer chain.
 *
 * Physics        : Verlet integration + constraint projection
 *                 Velocity is implicit: v ≈ (x_current − x_prev) / dt.
 *                 There is no explicit velocity variable — damping multiplies
 *                 the positional delta before the predict step.
 *
 * Math           : Constraint projection formula (per link):
 *                 correction = (|d| − rest) / |d|  ·  d_vector
 *                 Each free endpoint absorbs half the correction (equal
 *                 mass assumption).  A pinned node absorbs none (infinite
 *                 mass).
 *
 * Performance    : Sub-stepping (SUB_STEPS=8) divides dt into 8 smaller
 *                 steps.  Each sub-step runs the full constraint loop.
 *                 This improves stability for stiff constraints without
 *                 shrinking the render frame rate or using an implicit
 *                 (matrix-solving) integrator.  Cost: O(N · I · S) per
 *                 frame where N=nodes, I=iterations, S=sub-steps.
 *
 * Stability      : PBD is unconditionally stable — no spring constant to
 *                 blow up.  Stiffness is purely iteration count, not a
 *                 numerical parameter.  More iterations → stiffer rope,
 *                 but each iteration is just a few multiplies.
 *
 * Data-structure : Ring-buffer trail (TRAIL_LEN entries) for the last
 *                 free node.  Oldest entry is trail_head wrapped by modulo
 *                 arithmetic — no shifts needed.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
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

enum {
    SIM_FPS_DEFAULT = 60,   /* target render/sim rate (frames per second)  */
    SIM_FPS_MIN     = 10,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,

    N_NODES_MAX     = 32,   /* max chain nodes (including anchor nodes)    */
    N_NODES_DEF     = 24,   /* default node count; more → smoother but     */
                            /* O(N) constraint passes per sub-step         */
    N_ITER_DEF      = 20,   /* constraint iterations per sub-step:         */
                            /* more → stiffer rope, no instability risk    */
    N_ITER_MIN      =  5,
    N_ITER_MAX      = 60,
    SUB_STEPS       =  8,   /* physics sub-steps per rendered frame:       */
                            /* effective dt = frame_dt / SUB_STEPS         */
                            /* smaller sub-dt → more accurate constraint   */
                            /* projection without changing display fps     */

    TRAIL_LEN       = 90,   /* ring-buffer trail depth for bottom node;    */
                            /* at 60 fps ≈ 1.5 s of history               */
    N_PRESETS       =  4,
    N_THEMES        =  5,
};

#define NS_PER_SEC  1000000000LL          /* nanoseconds in one second      */
#define TICK_NS(f)  (NS_PER_SEC / (f))    /* nanoseconds per frame at fps f */

/* Terminal cell dimensions in pixels (ncurses coordinate space).
 * CELL_W × CELL_H is the logical pixel size of one character cell.
 * CELL_H / CELL_W ≈ 2 because typical monospace fonts are taller than wide.
 * All physics runs in pixel-space; px_to_cx / px_to_cy convert to cells.*/
#define CELL_W   8
#define CELL_H  16

/* GRAVITY: downward acceleration in terminal pixel-space (px/s²).
 * Real gravity = 9.81 m/s².  If we define 1 m ≡ CELL_H px (≈ one cell
 * height), then a screen of 40 cells ≈ 40 m — too slow visually.
 * Chosen empirically so the chain sways at a pleasing rate on a typical
 * 80-row terminal.  Scale: ~38.7 px per "metre", giving 9.81×38.7 ≈ 380. */
#define GRAVITY     380.f

/* DAMP: fraction of velocity kept after each sub-step (dimensionless 0–1).
 * Applied as: velocity *= DAMP each sub-step (via positional delta).
 * At SUB_STEPS=8 and 60 fps: 0.997^(8×60) ≈ 0.30 per second — gentle
 * energy drain that prevents perpetual oscillation without overdamping. */
#define DAMP        0.997f

/* WIND_FREQ: frequency of the sinusoidal wind force (Hz).
 * 0.35 Hz ≈ one full left-right cycle every 2.9 s — slow enough to look
 * like natural wind gusts rather than rapid mechanical oscillation.       */
#define WIND_FREQ   0.35f

/* Wave preset driver parameters */
/* WAVE_FREQ: driven frequency of the top anchor node (Hz).
 * 1.8 Hz was tuned to fall near the second normal mode of a 24-node chain
 * at the default tension, producing visible standing-wave patterns.       */
#define WAVE_FREQ   1.8f

/* WAVE_AMPL: peak horizontal displacement of the driven anchor (pixels).
 * At CELL_W=8 this is ~2.75 character columns — large enough to see the
 * wave without the chain folding over itself.                             */
#define WAVE_AMPL  22.f

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
        .tv_nsec = (long) (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Color pair assignments:
 *   1   link — relaxed  (near rest length)
 *   2   link — stretched
 *   3   link — very stretched
 *   4   node — free
 *   5   node — pinned / driven anchor
 *   6   trail (bottom node)
 *   7   HUD
 */
enum {
    CP_LINK_LO  = 1,
    CP_LINK_MID = 2,
    CP_LINK_HI  = 3,
    CP_NODE     = 4,
    CP_ANCHOR   = 5,
    CP_TRAIL    = 6,
    CP_HUD      = 7,
};

typedef struct {
    short lo, mid, hi;        /* link colors: relaxed / stretched / taut   */
    short node, anchor;       /* node marker colors                         */
    short trail, hud;
    short lo8, mid8, hi8;     /* 8-color fallbacks                          */
    short node8, anchor8;
    short trail8, hud8;
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 Classic — cyan chain, yellow trail */
    { 51, 226, 196,  231, 220,  226, 244,
      COLOR_CYAN, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
      "Classic" },
    /* 1 Fire — orange/red chain */
    { 208, 196, 160,  231, 226,  208, 244,
      COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_RED, COLOR_WHITE,
      "Fire" },
    /* 2 Ice — blue/teal chain */
    { 39, 123, 231,  159, 231,  123, 244,
      COLOR_BLUE, COLOR_CYAN, COLOR_WHITE,
      COLOR_CYAN, COLOR_WHITE, COLOR_CYAN, COLOR_WHITE,
      "Ice" },
    /* 3 Neon — magenta/violet chain */
    { 201, 165, 93,   231, 226,  207, 244,
      COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_YELLOW, COLOR_MAGENTA, COLOR_WHITE,
      "Neon" },
    /* 4 Matrix — green chain */
    { 46, 118, 22,   231, 154,  118, 244,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
      "Matrix" },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_LINK_LO,  th->lo,     -1);
        init_pair(CP_LINK_MID, th->mid,    -1);
        init_pair(CP_LINK_HI,  th->hi,     -1);
        init_pair(CP_NODE,     th->node,   -1);
        init_pair(CP_ANCHOR,   th->anchor, -1);
        init_pair(CP_TRAIL,    th->trail,  -1);
        init_pair(CP_HUD,      th->hud,    -1);
    } else {
        init_pair(CP_LINK_LO,  th->lo8,     -1);
        init_pair(CP_LINK_MID, th->mid8,    -1);
        init_pair(CP_LINK_HI,  th->hi8,     -1);
        init_pair(CP_NODE,     th->node8,   -1);
        init_pair(CP_ANCHOR,   th->anchor8, -1);
        init_pair(CP_TRAIL,    th->trail8,  -1);
        init_pair(CP_HUD,      th->hud8,    -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

/* Convert pixel-space coordinates to terminal cell coordinates.
 * + 0.5f performs nearest-cell rounding (equivalent to floorf(x + 0.5)). */
static inline int px_to_cx(float px) { return (int)(px / CELL_W + 0.5f); }
static inline int px_to_cy(float py) { return (int)(py / CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  physics — Position-Based Dynamics (PBD) chain                    */
/*                                                                       */
/* Integration loop per frame:                                           */
/*   for each sub-step s in [0, SUB_STEPS):                              */
/*     1. Verlet predict  : x_pred = x + (x − x_old)·DAMP + g·dt²      */
/*     2. Project constraints (N_ITER passes over all links)             */
/*     3. Implicit velocity: v = (x_pred − x_old) / dt  (Verlet gives   */
/*        this for free — no explicit v variable needed)                 */
/*     4. Advance: x_old = x;  x = x_pred                               */
/*                                                                       */
/* Pinned nodes have infinite effective mass: their position is fixed    */
/* externally each tick; the projection step skips their correction.     */
/* ===================================================================== */

typedef struct {
    float x,  y;       /* current position (pixels)                       */
    float ox, oy;      /* previous position — velocity = (x-ox)/dt        */
    bool  pinned;      /* position controlled externally each tick         */
} ChainNode;

typedef struct {
    ChainNode nodes[N_NODES_MAX];
    int   n_nodes;
    float link_rest;   /* rest length of each link (pixels)               */
    float wind_phase;
    float wind_str;
    bool  wind_on;
    bool  paused;
    int   preset;
    int   n_iter;      /* constraint iterations per sub-step              */

    /* wave preset driver state */
    float wave_phase;
    float wave_ax;     /* x of anchor (fixed) for wave preset             */
    float wave_ay;     /* y of anchor                                      */

    /* trail ring-buffer for last free node */
    float trail_px[TRAIL_LEN];
    float trail_py[TRAIL_LEN];
    int   trail_head;
    int   trail_cnt;
} Chain;

/* ── Draw helper: DDA segment ─────────────────────────────────────── */

static void seg_draw(int x0, int y0, int x1, int y1,
                     int cols, int rows, chtype attr)
{
    int dx    = x1 - x0;
    int dy    = y1 - y0;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    if (steps == 0) {
        if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
            attron(attr); mvaddch(y0, x0, 'o'); attroff(attr);
        }
        return;
    }

    int adx = abs(dx), ady = abs(dy);
    char ch;
    if      (adx >= 2 * ady)  ch = '-';
    else if (ady >= 2 * adx)  ch = '|';
    else if (dx * dy > 0)     ch = '\\';
    else                      ch = '/';

    attron(attr);
    for (int k = 0; k <= steps; k++) {
        int cx = x0 + k * dx / steps;
        int cy = y0 + k * dy / steps;
        if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
            mvaddch(cy, cx, ch);
    }
    attroff(attr);
}

/* ── PBD sub-step ─────────────────────────────────────────────────── */

static void chain_pbd_step(Chain *c, float dt)
{
    int   N   = c->n_nodes;
    float dt2 = dt * dt;    /* dt² appears in the force→position term: Δx = a·dt² */
    float wind_force = c->wind_on
        ? c->wind_str * sinf(c->wind_phase)
        : 0.f;

    /* ── Step 1: Verlet predict ──────────────────────────────────────
     * x_pred = x + (x − x_old)·DAMP + a·dt²
     * (x − x_old) is the velocity from the last step, multiplied by DAMP
     * to introduce drag.  Gravity and wind are external accelerations.   */
    for (int i = 0; i < N; i++) {
        ChainNode *n = &c->nodes[i];
        if (n->pinned) continue;    /* pinned nodes are moved externally */

        float vx  = (n->x - n->ox) * DAMP;   /* damped positional velocity x */
        float vy  = (n->y - n->oy) * DAMP;   /* damped positional velocity y */
        float new_x = n->x + vx + wind_force * dt2;
        float new_y = n->y + vy + GRAVITY * dt2;   /* gravity is +y (downward) */

        n->ox = n->x;       /* save current as "old" for next step */
        n->oy = n->y;
        n->x  = new_x;
        n->y  = new_y;
    }

    /* ── Step 2: Constraint projection ──────────────────────────────
     * For each link (a, b):
     *   d     = b.pos − a.pos          (vector from a to b)
     *   dist  = |d|                    (current length)
     *   inv   = (dist − rest) / dist   (normalised violation; dimensionless)
     *   corr  = inv · d                (full correction vector)
     * Each free endpoint absorbs half (equal mass assumption).
     * Running this N_ITER times converges toward the exact constraint.    */
    for (int iter = 0; iter < c->n_iter; iter++) {
        for (int i = 0; i < N - 1; i++) {
            ChainNode *a = &c->nodes[i];
            ChainNode *b = &c->nodes[i + 1];

            float dx   = b->x - a->x;
            float dy   = b->y - a->y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < 1e-6f) continue;   /* skip degenerate zero-length links */

            /* inv: how much the link overshoots its rest length, per unit length.
             * Positive → too long; negative → compressed (rare with gravity).  */
            float inv  = (dist - c->link_rest) / dist;
            float cx   = inv * dx;    /* correction vector, x component */
            float cy   = inv * dy;    /* correction vector, y component */

            if (a->pinned && b->pinned) {
                continue;                           /* both fixed: nothing to do */
            } else if (a->pinned) {
                b->x -= cx; b->y -= cy;             /* b carries full correction */
            } else if (b->pinned) {
                a->x += cx; a->y += cy;             /* a carries full correction */
            } else {
                /* split equally: both nodes have the same mass (1) */
                a->x += 0.5f * cx; a->y += 0.5f * cy;
                b->x -= 0.5f * cx; b->y -= 0.5f * cy;
            }
        }
    }
}

/* ── Chain tick ───────────────────────────────────────────────────── */

static void chain_tick(Chain *c, float dt)
{
    if (c->paused) return;

    float sub_dt = dt / (float)SUB_STEPS;

    /* advance phase */
    c->wind_phase += WIND_FREQ * 2.f * (float)M_PI * dt;
    if (c->wind_phase > 2.f * (float)M_PI)
        c->wind_phase -= 2.f * (float)M_PI;

    c->wave_phase += WAVE_FREQ * 2.f * (float)M_PI * dt;
    if (c->wave_phase > 2.f * (float)M_PI)
        c->wave_phase -= 2.f * (float)M_PI;

    for (int s = 0; s < SUB_STEPS; s++) {
        /*
         * Wave preset: drive the top node sinusoidally in x.
         * Set both x and ox so the node "carries" its velocity
         * naturally into the constraint projection — the wave
         * propagates downward via the links.
         */
        if (c->preset == 3) {
            float sub_t = (float)s / (float)SUB_STEPS;
            float phase_now  = c->wave_phase
                             - WAVE_FREQ * 2.f*(float)M_PI * dt * (1.f - sub_t);
            float phase_prev = phase_now - WAVE_FREQ * 2.f*(float)M_PI * sub_dt;
            c->nodes[0].x  = c->wave_ax + WAVE_AMPL * sinf(phase_now);
            c->nodes[0].ox = c->wave_ax + WAVE_AMPL * sinf(phase_prev);
            c->nodes[0].y  = c->wave_ay;
            c->nodes[0].oy = c->wave_ay;
        }

        chain_pbd_step(c, sub_dt);
    }

    /* trail: record last free node position */
    int last = c->n_nodes - 1;
    if (!c->nodes[last].pinned) {
        c->trail_px[c->trail_head] = c->nodes[last].x;
        c->trail_py[c->trail_head] = c->nodes[last].y;
        c->trail_head = (c->trail_head + 1) % TRAIL_LEN;
        if (c->trail_cnt < TRAIL_LEN) c->trail_cnt++;
    }
}

/* ── Chain draw ───────────────────────────────────────────────────── */

static void chain_draw(const Chain *c, int cols, int rows)
{
    int N = c->n_nodes;

    /* trail (oldest to newest, fading) */
    int start = (c->trail_head - c->trail_cnt + TRAIL_LEN) % TRAIL_LEN;
    for (int i = 0; i < c->trail_cnt; i++) {
        int idx = (start + i) % TRAIL_LEN;
        int cx  = px_to_cx(c->trail_px[idx]);
        int cy  = px_to_cy(c->trail_py[idx]);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;
        /* age: 0 = oldest trail entry, 1 = most recent.
         * Entries in the newest 40% get a bold '*'; older ones get '.'.
         * This creates a fading tail effect without storing per-entry alpha. */
        float age  = (float)i / (float)(c->trail_cnt > 1 ? c->trail_cnt : 1);
        bool  new_  = (age > 0.6f);    /* newest 40% of trail is bright */
        attron(COLOR_PAIR(CP_TRAIL) | (new_ ? A_BOLD : 0));
        mvaddch(cy, cx, new_ ? '*' : '.');
        attroff(COLOR_PAIR(CP_TRAIL) | (new_ ? A_BOLD : 0));
    }

    /* links */
    for (int i = 0; i < N - 1; i++) {
        const ChainNode *a = &c->nodes[i];
        const ChainNode *b = &c->nodes[i + 1];

        int ax = px_to_cx(a->x), ay = px_to_cy(a->y);
        int bx = px_to_cx(b->x), by = px_to_cy(b->y);

        /* tension color: compare actual length to rest length.
         * stretch = |dist − rest| / rest  (dimensionless strain, 0–∞).
         * Thresholds tuned so < 4% strain looks relaxed, 4–12% is mid,
         * > 12% looks taut — mimics how a real rope changes colour under load.
         * +0.001f guards against division by zero for degenerate zero-length links. */
        float dx   = b->x - a->x, dy = b->y - a->y;
        float dist = sqrtf(dx*dx + dy*dy);
        float stretch = fabsf(dist - c->link_rest) / (c->link_rest + 0.001f);

        int cp = (stretch < 0.04f) ? CP_LINK_LO    /* < 4%  strain → relaxed   */
               : (stretch < 0.12f) ? CP_LINK_MID   /* 4–12% strain → stretched */
               :                     CP_LINK_HI;   /* > 12% strain → taut      */

        seg_draw(ax, ay, bx, by, cols, rows, COLOR_PAIR(cp) | A_BOLD);
    }

    /* nodes */
    for (int i = 0; i < N; i++) {
        const ChainNode *n = &c->nodes[i];
        int cx = px_to_cx(n->x), cy = px_to_cy(n->y);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        if (n->pinned) {
            attron(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
            mvaddch(cy, cx, (i == 0 || i == N-1) ? '#' : '*');
            attroff(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_NODE));
            mvaddch(cy, cx, 'o');
            attroff(COLOR_PAIR(CP_NODE));
        }
    }
}

/* ===================================================================== */
/* §6  scene — preset initialisation & lifecycle                         */
/*                                                                       */
/* Each preset places the chain nodes at specific initial positions and  */
/* pins selected nodes.  The physics §5 then evolves the system forward. */
/*                                                                       */
/* Preset geometry is expressed as fractions of screen dimensions so the */
/* simulation looks proportional regardless of terminal size.            */
/* ===================================================================== */

static int  g_rows, g_cols;
static Chain g_chain;
static int   g_sim_fps = SIM_FPS_DEFAULT;

static const char *k_preset_names[N_PRESETS] = {
    "Hanging", "Pendulum", "Bridge", "Wave"
};

/*
 * Common initialisation: zero the chain, set n_nodes, compute link_rest
 * so the full chain is ~55% of screen height.
 */
static void chain_init_common(Chain *c, int n_nodes, float link_rest)
{
    memset(c, 0, sizeof *c);
    c->n_nodes    = n_nodes;
    c->link_rest  = link_rest;
    c->n_iter     = N_ITER_DEF;
    c->wind_on    = true;
    c->trail_head = 0;
    c->trail_cnt  = 0;
}

/* ── Preset 0: Hanging ────────────────────────────────────────────── */
/*
 * Top node pinned at screen centre-top.
 * Nodes hang straight down initially.
 * Wind oscillates left-right; produces sway.
 */
static void preset_hanging(Chain *c)
{
    float anchor_x = (float)g_cols * CELL_W * 0.5f;
    float anchor_y = (float)g_rows * CELL_H * 0.07f;
    float total_len = (float)g_rows * CELL_H * 0.75f;
    int   N         = N_NODES_DEF;
    float rest      = total_len / (float)(N - 1);

    chain_init_common(c, N, rest);
    c->wind_str = 140.f;
    c->preset   = 0;

    for (int i = 0; i < N; i++) {
        c->nodes[i].x  = c->nodes[i].ox = anchor_x;
        c->nodes[i].y  = c->nodes[i].oy = anchor_y + (float)i * rest;
        c->nodes[i].pinned = (i == 0);
    }
}

/* ── Preset 1: Pendulum ───────────────────────────────────────────── */
/*
 * Top node pinned. Chain starts at 60° from vertical.
 * Released from rest — swings back and forth, wave patterns emerge.
 */
static void preset_pendulum(Chain *c)
{
    float anchor_x = (float)g_cols * CELL_W * 0.5f;
    float anchor_y = (float)g_rows * CELL_H * 0.07f;
    float total_len = (float)g_rows * CELL_H * 0.70f;
    int   N         = N_NODES_DEF;
    float rest      = total_len / (float)(N - 1);

    chain_init_common(c, N, rest);
    c->wind_on  = false;
    c->preset   = 1;

    float angle = (float)M_PI / 3.f;   /* 60° from vertical (π/3 radians)
                                         * Large enough angle for dramatic
                                         * swing without going past horizontal */
    float sin_a = sinf(angle), cos_a = cosf(angle);

    for (int i = 0; i < N; i++) {
        float t = (float)i * rest;
        c->nodes[i].x  = c->nodes[i].ox = anchor_x + sin_a * t;
        c->nodes[i].y  = c->nodes[i].oy = anchor_y + cos_a * t;
        c->nodes[i].pinned = (i == 0);
    }
}

/* ── Preset 2: Bridge ─────────────────────────────────────────────── */
/*
 * Two anchor nodes pinned at equal height, separated by ~65% screen width.
 * Link rest length is 1.25× the straight distance / (N-1) → natural sag.
 * Wind gusts upward (like a flag in the wind from below).
 */
static void preset_bridge(Chain *c)
{
    float span    = (float)g_cols * CELL_W * 0.65f;
    float ax      = (float)g_cols * CELL_W * 0.175f;
    float ay      = (float)g_rows * CELL_H * 0.28f;
    int   N       = N_NODES_DEF;
    /* rest 25% longer than the straight-line distance between anchors:
     * Catenary physics: a chain longer than the span must sag.  The
     * extra 25% of length has to go somewhere — it forms a parabolic
     * (approximately catenary) droop.  More excess → deeper sag.        */
    float rest    = span * 1.25f / (float)(N - 1);

    chain_init_common(c, N, rest);
    c->wind_str = 110.f;
    c->wind_on  = true;
    c->preset   = 2;

    /* initialise along straight line between the two anchors */
    for (int i = 0; i < N; i++) {
        float t = (float)i / (float)(N - 1);
        c->nodes[i].x  = c->nodes[i].ox = ax + t * span;
        c->nodes[i].y  = c->nodes[i].oy = ay;
        c->nodes[i].pinned = (i == 0 || i == N - 1);
    }
}

/* ── Preset 3: Wave ───────────────────────────────────────────────── */
/*
 * Top node driven with sinusoidal horizontal displacement.
 * Wave travels down the chain, reflects at the free bottom end, and
 * interferes with the incident wave — standing waves emerge at resonant
 * frequencies. The wave speed varies with tension (higher near the top).
 */
static void preset_wave(Chain *c)
{
    float anchor_x = (float)g_cols * CELL_W * 0.5f;
    float anchor_y = (float)g_rows * CELL_H * 0.06f;
    float total_len = (float)g_rows * CELL_H * 0.80f;
    int   N         = N_NODES_DEF;
    float rest      = total_len / (float)(N - 1);

    chain_init_common(c, N, rest);
    c->wind_on   = false;
    c->preset    = 3;
    c->wave_ax   = anchor_x;
    c->wave_ay   = anchor_y;
    c->wave_phase = 0.f;

    /* start hanging straight down */
    for (int i = 0; i < N; i++) {
        c->nodes[i].x  = c->nodes[i].ox = anchor_x;
        c->nodes[i].y  = c->nodes[i].oy = anchor_y + (float)i * rest;
        c->nodes[i].pinned = (i == 0);
    }
}

static void scene_init(int preset)
{
    g_chain.preset = preset;
    switch (preset) {
    case 0: preset_hanging (&g_chain); break;
    case 1: preset_pendulum(&g_chain); break;
    case 2: preset_bridge  (&g_chain); break;
    case 3: preset_wave    (&g_chain); break;
    }
}

/* ===================================================================== */
/* §7  screen / HUD                                                       */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void hud_draw(const Chain *c, int fps)
{
    move(g_rows - 1, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HUD));
    printw(" Chain  q:quit  n:%s  t:%s  +/-:iter(%d)  w:wind(%s)"
           "  p:pause  r:reset  %dfps%s",
           k_preset_names[c->preset],
           k_themes[g_theme].name,
           c->n_iter,
           c->wind_on ? "ON" : "OFF",
           fps,
           c->paused ? "  [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit_flag   = 0;
static volatile sig_atomic_t g_resize_flag = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit_flag   = 1;
    if (s == SIGWINCH)               g_resize_flag = 1;
}
static void do_cleanup(void) { endwin(); }

int main(void)
{
    atexit(do_cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();
    getmaxyx(stdscr, g_rows, g_cols);

    int cur_preset = 0;
    scene_init(cur_preset);

    int64_t t_last = clock_ns();

    /* FPS counter */
    int64_t fps_acc  = 0;
    int     fps_cnt  = 0;
    int     fps_disp = 0;

    while (!g_quit_flag) {

        /* ── resize ── */
        if (g_resize_flag) {
            g_resize_flag = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            scene_init(cur_preset);
            t_last = clock_ns();
            continue;
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_quit_flag = 1; break;

            case ' ': case 'p': case 'P':
                g_chain.paused = !g_chain.paused;
                break;

            case 'r': case 'R':
                scene_init(cur_preset);
                break;

            case 'n':
                cur_preset = (cur_preset + 1) % N_PRESETS;
                scene_init(cur_preset);
                break;

            case 'N':
                cur_preset = (cur_preset + N_PRESETS - 1) % N_PRESETS;
                scene_init(cur_preset);
                break;

            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;

            case 'T':
                g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_theme);
                break;

            case '+': case '=':
                if (g_chain.n_iter < N_ITER_MAX) g_chain.n_iter += 5;
                break;

            case '-':
                if (g_chain.n_iter > N_ITER_MIN) g_chain.n_iter -= 5;
                break;

            case 'w': case 'W':
                g_chain.wind_on = !g_chain.wind_on;
                break;

            case ']':
                if (g_sim_fps < SIM_FPS_MAX) g_sim_fps += SIM_FPS_STEP;
                break;

            case '[':
                if (g_sim_fps > SIM_FPS_MIN) g_sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── tick ── */
        int64_t t_now  = clock_ns();
        int64_t t_used = t_now - t_last;
        t_last = t_now;
        /* Cap dt at 100 ms (= 10 fps equivalent).
         * If the process was suspended (e.g., Ctrl-Z, terminal resize storm),
         * t_used could be seconds.  Feeding that into the physics would
         * launch nodes off-screen.  100 ms is a safe upper bound.          */
        if (t_used > 100000000LL) t_used = 100000000LL;  /* 100 ms cap */

        float dt = (float)t_used * 1e-9f;
        chain_tick(&g_chain, dt);

        /* ── draw ── */
        erase();
        chain_draw(&g_chain, g_cols, g_rows);
        hud_draw(&g_chain, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── FPS cap ── */
        int64_t t_frame = TICK_NS(g_sim_fps);
        int64_t t_sleep = t_frame - (clock_ns() - t_now);
        clock_sleep_ns(t_sleep);

        /* ── FPS counter ── */
        fps_acc += t_used;
        fps_cnt++;
        if (fps_acc >= NS_PER_SEC / 2) {
            fps_disp = fps_cnt * 2;
            fps_acc  = 0;
            fps_cnt  = 0;
        }
    }

    return 0;
}
