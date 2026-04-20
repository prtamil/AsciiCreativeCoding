/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/mass_spring_lattice.c — 2D Mass-Spring Lattice Simulator
 *
 * Models a rectangular grid of point masses connected by springs.
 * Each node obeys Newton's second law; springs obey Hooke's law:
 *   F = -k (|d| - L₀) d̂
 * Integration uses symplectic Euler for energy-conserving dynamics.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *  §1  config      — tunable constants
 *  §2  clock       — monotonic nanosecond clock + sleep
 *  §3  theme       — color pairs: spring stretch, node speed
 *  §4  lattice     — Node/Spring structs; static BSS arrays
 *  §5  solver      — init_lattice, compute_forces, integrate_nodes
 *  §6  presets     — cloth, net, wave, jelly
 *  §7  render      — render_lattice, render_overlay
 *  §8  scene       — Scene struct, scene_init/tick/draw
 *  §9  screen      — ncurses double-buffer display layer
 *  §10 app         — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q/ESC  quit          space  pause/resume    s    single step
 *   r      reset         i      impulse node    h    hammer node
 *   arrows move cursor   p      pin/unpin node  g/G  gravity±
 *   k/K    stiffness±    d/D    damping±        t    cycle theme
 *   o      overlay       1-4    preset
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/mass_spring_lattice.c \
 *       -o mass_spring -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────── *
 *
 * Hooke's law spring force (§5 compute_forces):
 *   For a spring connecting node a and node b:
 *
 *     d   = pos_b - pos_a          (displacement vector)
 *     |d| = sqrt(dx² + dy²)        (current spring length)
 *     L₀  = rest length
 *     d̂   = d / |d|               (unit vector a→b)
 *
 *     F_spring = k · (|d| - L₀) · d̂
 *
 *   Force F_spring acts on node a (toward b if extended).
 *   Reaction -F_spring acts on node b (Newton's 3rd law).
 *   When |d| > L₀: spring is extended, pulls nodes together.
 *   When |d| < L₀: spring is compressed, pushes nodes apart.
 *
 * Spring types (§4):
 *   SPRING_H — horizontal structural (rest = NODE_DX)
 *   SPRING_V — vertical structural   (rest = NODE_DY)
 *   SPRING_D — diagonal shear        (rest = √(DX²+DY²))
 *   Shear springs resist parallelogram deformation.
 *
 * Symplectic Euler integration (§5 integrate_nodes):
 *   v_new = v + a·dt                (velocity updated with old force)
 *   x_new = x + v_new·dt           (position uses NEW velocity)
 *
 *   Using v_new (not old v) conserves a shadow energy, preventing
 *   artificial growth that explicit Euler exhibits.  Cost is identical.
 *
 * Stability limit (§5):
 *   Highest mode frequency: ω_max ≈ √(N_springs · k / m)
 *   Explicit stability requires: dt < 2 / ω_max
 *
 *   With N=8 neighbors, k=40, m=1:
 *     dt_crit = 2 · √(m / (N·k)) = 2 · √(1/320) ≈ 0.112 s
 *   Default dt = 1/60 ≈ 0.0167 s ≪ 0.112 s  → comfortably stable.
 *   Raising k above ~1000 at 60 Hz approaches instability.
 *
 * Velocity damping (§5):
 *   F_damp = -c · v   (c = damping coefficient [N·s/m])
 *   Critical damping: c_crit = 2·√(k·m) ≈ 12.6 for k=40, m=1.
 *   Default c=2.0 → underdamped, oscillation decays slowly.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <signal.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ════════════════════════════════════════════════════════════════════
 * §1  CONFIG
 * ════════════════════════════════════════════════════════════════════ */
#define TARGET_FPS      60
#define SIM_HZ_DEFAULT  60

#define NODE_DX         3       /* cell columns between adjacent nodes */
#define NODE_DY         2       /* cell rows between adjacent nodes */
#define MAX_NX          52      /* max lattice columns */
#define MAX_NY          22      /* max lattice rows */
#define MAX_NODES       (MAX_NX * MAX_NY)
#define MAX_SPRINGS     (MAX_NX * MAX_NY * 6)

#define MASS_DEFAULT    1.0f
#define K_STRUCT_DEF    40.0f
#define K_SHEAR_DEF     20.0f
#define DAMPING_DEF     2.0f
#define GRAVITY_DEF     15.0f
#define IMPULSE_VEL     12.0f
#define HAMMER_VEL      30.0f

#define K_STEP          5.0f
#define K_MIN           5.0f
#define K_MAX           400.0f
#define DAMPING_STEP    0.5f
#define DAMPING_MIN     0.0f
#define DAMPING_MAX     20.0f
#define GRAVITY_STEP    2.0f
#define GRAVITY_MIN    -40.0f
#define GRAVITY_MAX     40.0f

#define OVERLAY_W       34
#define OVERLAY_H       13
#define STATUS_ROW_OFF  1       /* rows from bottom for status bar */

/* ════════════════════════════════════════════════════════════════════
 * §2  CLOCK
 * ════════════════════════════════════════════════════════════════════ */
typedef long long ns_t;

static ns_t clock_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ns_t)ts.tv_sec * 1000000000LL + (ns_t)ts.tv_nsec;
}

static void clock_sleep_ns(ns_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = {
        .tv_sec  = ns / 1000000000LL,
        .tv_nsec = ns % 1000000000LL
    };
    nanosleep(&ts, NULL);
}

/* ════════════════════════════════════════════════════════════════════
 * §3  THEME
 * ════════════════════════════════════════════════════════════════════ */
#define CP_VERY_COMP    1   /* heavily compressed spring */
#define CP_COMP         2   /* slightly compressed */
#define CP_NORMAL       3   /* near rest length */
#define CP_EXTEND       4   /* slightly extended */
#define CP_VERY_EXT     5   /* heavily extended */
#define CP_NODE_PIN     6   /* pinned node */
#define CP_NODE_SLOW    7   /* slow-moving node */
#define CP_NODE_FAST    8   /* fast-moving node */
#define CP_CURSOR       9   /* cursor selection */
#define CP_OVERLAY     10   /* overlay border + text */
#define CP_TITLE       11   /* overlay title */
#define CP_STATUS      12   /* bottom status bar */

#define THEME_CLASSIC   0
#define THEME_COLD      1
#define THEME_HOT       2
#define THEME_COUNT     3

static const char * const k_theme_names[THEME_COUNT] = {
    "Classic", "Cold", "Hot"
};

static void init_colors(int theme)
{
    start_color();
    use_default_colors();

    switch (theme) {
    case THEME_COLD:
        init_pair(CP_VERY_COMP, COLOR_WHITE,   -1);
        init_pair(CP_COMP,      COLOR_CYAN,    -1);
        init_pair(CP_NORMAL,    COLOR_BLUE,    -1);
        init_pair(CP_EXTEND,    COLOR_CYAN,    -1);
        init_pair(CP_VERY_EXT,  COLOR_WHITE,   -1);
        break;
    case THEME_HOT:
        init_pair(CP_VERY_COMP, COLOR_BLUE,    -1);
        init_pair(CP_COMP,      COLOR_CYAN,    -1);
        init_pair(CP_NORMAL,    COLOR_YELLOW,  -1);
        init_pair(CP_EXTEND,    COLOR_RED,     -1);
        init_pair(CP_VERY_EXT,  COLOR_MAGENTA, -1);
        break;
    default: /* THEME_CLASSIC */
        init_pair(CP_VERY_COMP, COLOR_BLUE,    -1);
        init_pair(CP_COMP,      COLOR_CYAN,    -1);
        init_pair(CP_NORMAL,    COLOR_WHITE,   -1);
        init_pair(CP_EXTEND,    COLOR_YELLOW,  -1);
        init_pair(CP_VERY_EXT,  COLOR_RED,     -1);
        break;
    }

    init_pair(CP_NODE_PIN,  COLOR_WHITE,  COLOR_BLUE);
    init_pair(CP_NODE_SLOW, COLOR_GREEN,  -1);
    init_pair(CP_NODE_FAST, COLOR_MAGENTA,-1);
    init_pair(CP_CURSOR,    COLOR_BLACK,  COLOR_YELLOW);
    init_pair(CP_OVERLAY,   COLOR_WHITE,  -1);
    init_pair(CP_TITLE,     COLOR_CYAN,   -1);
    init_pair(CP_STATUS,    COLOR_BLACK,  COLOR_WHITE);
}

static int stretch_color_pair(float ratio)
{
    if (ratio < 0.82f) return CP_VERY_COMP;
    if (ratio < 0.93f) return CP_COMP;
    if (ratio < 1.08f) return CP_NORMAL;
    if (ratio < 1.22f) return CP_EXTEND;
    return CP_VERY_EXT;
}

/* ════════════════════════════════════════════════════════════════════
 * §4  LATTICE  (static BSS — no malloc)
 * ════════════════════════════════════════════════════════════════════ */
typedef struct {
    float x,  y;    /* current position (cell coordinates) */
    float rx, ry;   /* rest position */
    float vx, vy;   /* velocity [cells/s] */
    float fx, fy;   /* accumulated force (zeroed each step) */
    bool  pinned;
} Node;

typedef enum { SPRING_H = 0, SPRING_V = 1, SPRING_D = 2 } SpringKind;

typedef struct {
    int        a, b;
    float      rest_len;
    float      stretch_ratio;   /* updated in compute_forces */
    SpringKind kind;
} Spring;

static Node   g_nodes  [MAX_NODES];
static Spring g_springs[MAX_SPRINGS];
static int    g_nn, g_ns;   /* node count, spring count */
static int    g_nx, g_ny;   /* lattice dimensions */
static int    g_x0, g_y0;   /* top-left cell offset */

/* ════════════════════════════════════════════════════════════════════
 * §5  SOLVER
 * ════════════════════════════════════════════════════════════════════ */
static void init_lattice(int nx, int ny, int cols, int rows)
{
    g_nx = nx;
    g_ny = ny;
    g_nn = nx * ny;
    g_ns = 0;

    int lat_w = (nx - 1) * NODE_DX;
    int lat_h = (ny - 1) * NODE_DY;
    g_x0 = (cols - lat_w) / 2;
    g_y0 = (rows - 1 - lat_h) / 3;
    if (g_y0 < 1) g_y0 = 1;

    for (int r = 0; r < ny; r++) {
        for (int c = 0; c < nx; c++) {
            int i = r * nx + c;
            g_nodes[i].rx = (float)(g_x0 + c * NODE_DX);
            g_nodes[i].ry = (float)(g_y0 + r * NODE_DY);
            g_nodes[i].x  = g_nodes[i].rx;
            g_nodes[i].y  = g_nodes[i].ry;
            g_nodes[i].vx = 0.0f;
            g_nodes[i].vy = 0.0f;
            g_nodes[i].fx = 0.0f;
            g_nodes[i].fy = 0.0f;
            g_nodes[i].pinned = false;
        }
    }
}

static void add_spring(int a, int b, float rest_len, SpringKind kind)
{
    if (g_ns >= MAX_SPRINGS) return;
    g_springs[g_ns].a            = a;
    g_springs[g_ns].b            = b;
    g_springs[g_ns].rest_len     = rest_len;
    g_springs[g_ns].stretch_ratio= 1.0f;
    g_springs[g_ns].kind         = kind;
    g_ns++;
}

static void build_springs(bool with_shear)
{
    g_ns = 0;
    float hr = (float)NODE_DX;
    float vr = (float)NODE_DY;
    float dr = sqrtf(hr * hr + vr * vr);

    for (int r = 0; r < g_ny; r++) {
        for (int c = 0; c < g_nx; c++) {
            int i = r * g_nx + c;
            if (c + 1 < g_nx)
                add_spring(i, i + 1,        hr, SPRING_H);
            if (r + 1 < g_ny)
                add_spring(i, i + g_nx,     vr, SPRING_V);
            if (with_shear) {
                if (r + 1 < g_ny && c + 1 < g_nx)
                    add_spring(i, i + g_nx + 1, dr, SPRING_D);
                if (r + 1 < g_ny && c - 1 >= 0)
                    add_spring(i, i + g_nx - 1, dr, SPRING_D);
            }
        }
    }
}

static void zero_forces(void)
{
    for (int i = 0; i < g_nn; i++) {
        g_nodes[i].fx = 0.0f;
        g_nodes[i].fy = 0.0f;
    }
}

static void compute_forces(float k_struct, float k_shear,
                            float damping, float gravity, float mass)
{
    zero_forces();

    /* Spring forces: F = k * stretch * unit_vector */
    for (int s = 0; s < g_ns; s++) {
        int   a  = g_springs[s].a;
        int   b  = g_springs[s].b;
        float dx = g_nodes[b].x - g_nodes[a].x;
        float dy = g_nodes[b].y - g_nodes[a].y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist < 0.01f) {
            g_springs[s].stretch_ratio = 1.0f;
            continue;
        }

        float rest = g_springs[s].rest_len;
        g_springs[s].stretch_ratio = dist / rest;

        float k       = (g_springs[s].kind == SPRING_D) ? k_shear : k_struct;
        float stretch = dist - rest;
        float inv_d   = 1.0f / dist;
        float fx      = k * stretch * dx * inv_d;
        float fy      = k * stretch * dy * inv_d;

        if (!g_nodes[a].pinned) { g_nodes[a].fx += fx; g_nodes[a].fy += fy; }
        if (!g_nodes[b].pinned) { g_nodes[b].fx -= fx; g_nodes[b].fy -= fy; }
    }

    /* External: gravity + velocity damping */
    for (int i = 0; i < g_nn; i++) {
        if (g_nodes[i].pinned) continue;
        g_nodes[i].fy += mass * gravity;
        g_nodes[i].fx -= damping * g_nodes[i].vx;
        g_nodes[i].fy -= damping * g_nodes[i].vy;
    }
}

static void integrate_nodes(float dt, float mass)
{
    float inv_m = 1.0f / mass;
    for (int i = 0; i < g_nn; i++) {
        if (g_nodes[i].pinned) continue;
        /* Symplectic Euler: velocity first, then position with new velocity */
        g_nodes[i].vx += g_nodes[i].fx * inv_m * dt;
        g_nodes[i].vy += g_nodes[i].fy * inv_m * dt;
        g_nodes[i].x  += g_nodes[i].vx * dt;
        g_nodes[i].y  += g_nodes[i].vy * dt;
    }
}

static void compute_stats(float k_struct, float mass,
                           float *ke, float *pe,
                           float *max_stretch, float *avg_vel)
{
    float sum_ke = 0.0f, sum_pe = 0.0f, mx = 1.0f, sum_v = 0.0f;

    for (int i = 0; i < g_nn; i++) {
        float v2 = g_nodes[i].vx * g_nodes[i].vx
                 + g_nodes[i].vy * g_nodes[i].vy;
        sum_ke += 0.5f * mass * v2;
        sum_v  += sqrtf(v2);
    }

    for (int s = 0; s < g_ns; s++) {
        float r = g_springs[s].stretch_ratio;
        if (r > mx) mx = r;
        float rest    = g_springs[s].rest_len;
        float stretch = (r - 1.0f) * rest;
        sum_pe += 0.5f * k_struct * stretch * stretch;
    }

    *ke          = sum_ke;
    *pe          = sum_pe;
    *max_stretch = mx;
    *avg_vel     = (g_nn > 0) ? sum_v / (float)g_nn : 0.0f;
}

/* ════════════════════════════════════════════════════════════════════
 * §6  PRESETS
 * ════════════════════════════════════════════════════════════════════ */
typedef enum {
    PRESET_CLOTH = 0,
    PRESET_NET,
    PRESET_WAVE,
    PRESET_JELLY,
    PRESET_COUNT
} PresetID;

static const char * const k_preset_names[PRESET_COUNT] = {
    "Cloth  (top pinned)",
    "Net    (border pinned)",
    "Wave   (left impulse)",
    "Jelly  (center impulse)"
};

typedef struct Scene Scene;

static void preset_apply(Scene *s, PresetID pid, int cols, int rows);

/* ════════════════════════════════════════════════════════════════════
 * §7  RENDER
 * ════════════════════════════════════════════════════════════════════ */

/* spring character based on displacement direction */
static chtype spring_char(float dx, float dy)
{
    float adx = fabsf(dx), ady = fabsf(dy);
    if (adx > 2.2f * ady) return (chtype)'-';
    if (ady > 2.0f * adx) return (chtype)'|';
    return (dx * dy > 0.0f) ? (chtype)'\\' : (chtype)'/';
}

/*
 * render_springs() — draw every spring as 1–2 characters coloured by stretch.
 *
 * WHY 1–2 chars per spring (not a full walk):
 *   Horizontal springs span NODE_DX=3 columns; vertical span NODE_DY=2 rows.
 *   Walking every interior cell would put 2–3 chars on screen per spring.
 *   But at deformed angles the walk diverges from the visual line, creating
 *   ragged gaps.  Two samples at t=0.33 and t=0.67 give a cleaner look and
 *   are O(1) per spring instead of O(N) for diagonal walks.
 *
 * WHY NaN/displaced guard:
 *   If k is raised past the stability limit, positions blow up to Inf/NaN.
 *   The scene_tick blow-up check resets the scene on the next tick, but
 *   this render call can happen BEFORE that check on the same frame.
 *   Without the guard, walking an Inf→Inf line wraps the loop counter
 *   and takes thousands of iterations — causing a visual hang.
 *
 * Colour mapping (stretch_color_pair):
 *   ratio < 0.82 → CP_VERY_COMP (blue)      — spring is tightly compressed
 *   ratio < 0.93 → CP_COMP      (cyan)       — slightly compressed
 *   ratio < 1.08 → CP_NORMAL    (white)      — near rest length
 *   ratio < 1.22 → CP_EXTEND    (yellow)     — slightly stretched
 *   ratio ≥ 1.22 → CP_VERY_EXT  (red)        — heavily stretched (alert)
 */
static void render_springs(WINDOW *w, int cols, int max_r)
{
    /* Guard: skip any spring that has blown past 6× rest displacement.
     * max_disp2 in cell² — if |ab|² > this, the spring is numerically broken. */
    float max_disp2 = (float)((NODE_DX * 6) * (NODE_DX * 6)
                             + (NODE_DY * 6) * (NODE_DY * 6));

    for (int s = 0; s < g_ns; s++) {
        int   a  = g_springs[s].a;
        int   b  = g_springs[s].b;
        float ax = g_nodes[a].x, ay = g_nodes[a].y;
        float bx = g_nodes[b].x, by = g_nodes[b].y;

        /* NaN/Inf guard — prevents infinite loop on a blown-up lattice */
        if (!isfinite(ax) || !isfinite(ay) || !isfinite(bx) || !isfinite(by))
            continue;
        float dx = bx - ax, dy = by - ay;
        if (dx * dx + dy * dy > max_disp2) continue;

        chtype ch = spring_char(dx, dy);
        int    cp = stretch_color_pair(g_springs[s].stretch_ratio);

        if (g_springs[s].kind == SPRING_D) {
            /* Diagonal shear spring: two samples at 1/3 and 2/3 of the span.
             * One character near each endpoint shows the shear direction clearly
             * without crowding the node positions at t=0 and t=1.            */
            int x1 = (int)roundf(ax + dx * 0.33f);
            int y1 = (int)roundf(ay + dy * 0.33f);
            int x2 = (int)roundf(ax + dx * 0.67f);
            int y2 = (int)roundf(ay + dy * 0.67f);
            if (x1 >= 0 && x1 < cols && y1 >= 0 && y1 < max_r)
                mvwaddch(w, y1, x1, ch | COLOR_PAIR(cp));
            if (x2 >= 0 && x2 < cols && y2 >= 0 && y2 < max_r)
                mvwaddch(w, y2, x2, ch | COLOR_PAIR(cp));
        } else {
            /* Structural H/V spring: walk interior cells one step at a time.
             * The hard cap (NODE_DX + NODE_DY + 2) prevents O(∞) loops when
             * the deformed length temporarily exceeds the rest length.        */
            int x0i = (int)roundf(ax), y0i = (int)roundf(ay);
            int x1i = (int)roundf(bx), y1i = (int)roundf(by);
            int sxi  = (x0i < x1i) ? 1 : (x0i > x1i) ? -1 : 0;
            int syi  = (y0i < y1i) ? 1 : (y0i > y1i) ? -1 : 0;
            int cxi  = x0i + sxi, cyi = y0i + syi;
            int lim  = NODE_DX + NODE_DY + 2;
            while ((cxi != x1i || cyi != y1i) && lim-- > 0) {
                if (cxi >= 0 && cxi < cols && cyi >= 0 && cyi < max_r)
                    mvwaddch(w, cyi, cxi, ch | COLOR_PAIR(cp));
                cxi += sxi; cyi += syi;
            }
        }
    }
}

/*
 * render_velocity_arrows() — place '>' 1.5 cells ahead of each moving node.
 *
 * WHY 1.5-cell scale:
 *   scale = 1.5 / speed → arrow tip is always 1.5 cells from the node.
 *   This makes all arrows the same visual length regardless of speed magnitude,
 *   showing DIRECTION without implying magnitude (speed is shown by node colour).
 *   Arrow tips are clipped to the lattice drawing region (max_r).
 *
 * WHY skip pinned nodes:
 *   Pinned nodes have non-zero velocity initialised by impulse/hammer but then
 *   immediately zeroed by the solver.  They never move, so any stored velocity
 *   is stale.  Skipping them avoids drawing misleading arrows at fixed anchors.
 */
static void render_velocity_arrows(WINDOW *w, int cols, int max_r)
{
    for (int i = 0; i < g_nn; i++) {
        if (g_nodes[i].pinned) continue;
        float speed = sqrtf(g_nodes[i].vx * g_nodes[i].vx
                          + g_nodes[i].vy * g_nodes[i].vy);
        if (speed < 0.5f) continue;   /* below noise threshold — skip */
        float scale = 1.5f / speed;
        int   ax    = (int)roundf(g_nodes[i].x + g_nodes[i].vx * scale);
        int   ay    = (int)roundf(g_nodes[i].y + g_nodes[i].vy * scale);
        if (ax >= 0 && ax < cols && ay >= 0 && ay < max_r)
            mvwaddch(w, ay, ax, '>' | COLOR_PAIR(CP_NODE_FAST));
    }
}

/*
 * render_nodes() — draw each node glyph ON TOP of springs.
 *
 * Nodes are drawn last so they are always visible even when the spring
 * character would occupy the same cell.  The glyph + colour encode speed:
 *   pinned → '@' CP_NODE_PIN  — anchored to world; never moves
 *   slow   → 'o' CP_NODE_SLOW — velocity < 1 cell/s; nearly at rest
 *   medium → '*' CP_NORMAL    — velocity 1–8 cell/s; actively oscillating
 *   fast   → '0' CP_NODE_FAST — velocity > 8 cell/s; just hit or impulsed
 *
 * WHY speed thresholds 1 and 8:
 *   With k=40 and m=1, the natural frequency is ~6.3 rad/s, giving a peak
 *   velocity of ~amplitude × ω.  A 1-cell impulse gives ~6 cells/s peak.
 *   "Fast" (>8) catches the first half-cycle of a fresh hammer strike.
 */
static void render_nodes(WINDOW *w, int cols, int max_r)
{
    for (int i = 0; i < g_nn; i++) {
        int cx = (int)roundf(g_nodes[i].x);
        int cy = (int)roundf(g_nodes[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= max_r) continue;

        float speed = sqrtf(g_nodes[i].vx * g_nodes[i].vx
                          + g_nodes[i].vy * g_nodes[i].vy);
        int    cp;
        chtype ch;

        if (g_nodes[i].pinned) {
            cp = CP_NODE_PIN;  ch = (chtype)'@';
        } else if (speed < 1.0f) {
            cp = CP_NODE_SLOW; ch = (chtype)'o';
        } else if (speed < 8.0f) {
            cp = CP_NORMAL;    ch = (chtype)'*';
        } else {
            cp = CP_NODE_FAST; ch = (chtype)'0';
        }
        mvwaddch(w, cy, cx, ch | COLOR_PAIR(cp));
    }
}

/*
 * render_lattice() — coordinate the three render layers in draw order.
 *
 * Draw order matters: springs first (background), optional velocity arrows
 * (mid), then nodes on top (foreground).  If nodes were drawn first, springs
 * would overwrite them.  Velocity arrows go between so the node glyph still
 * dominates visually at the node position.
 */
static void render_lattice(WINDOW *w, int cols, int rows, bool show_vel)
{
    int max_r = rows - (int)STATUS_ROW_OFF;

    render_springs(w, cols, max_r);
    if (show_vel)
        render_velocity_arrows(w, cols, max_r);
    render_nodes(w, cols, max_r);
}

static void render_cursor(WINDOW *w, int cols, int rows,
                          int cnx, int cny)
{
    int max_r = rows - (int)STATUS_ROW_OFF;
    int i  = cny * g_nx + cnx;
    int cx = (int)roundf(g_nodes[i].rx);
    int cy = (int)roundf(g_nodes[i].ry);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= max_r) return;

    chtype ch = g_nodes[i].pinned ? (chtype)'@' : (chtype)'*';
    mvwaddch(w, cy, cx, ch | COLOR_PAIR(CP_CURSOR) | A_BOLD);
}

static void render_overlay(WINDOW *w, int cols, int rows,
                           int preset_id, int theme, int ticks,
                           float ke, float pe,
                           float max_stretch, float avg_vel,
                           float k_struct, float damping, float gravity,
                           float k_max, float dt_crit)
{
    int ox = cols - OVERLAY_W - 1;
    int oy = 1;
    if (ox < 0) ox = 0;
    if (oy + OVERLAY_H >= rows) return;

    int w2 = OVERLAY_W;

    wattron(w, COLOR_PAIR(CP_OVERLAY));
    /* border */
    mvwprintw(w, oy,   ox, "┌");
    mvwprintw(w, oy,   ox + w2 - 1, "┐");
    mvwprintw(w, oy + OVERLAY_H - 1, ox, "└");
    mvwprintw(w, oy + OVERLAY_H - 1, ox + w2 - 1, "┘");
    for (int c = 1; c < w2 - 1; c++) {
        mvwprintw(w, oy,                 ox + c, "─");
        mvwprintw(w, oy + OVERLAY_H - 1, ox + c, "─");
    }
    for (int r = 1; r < OVERLAY_H - 1; r++) {
        mvwprintw(w, oy + r, ox,         "│");
        mvwprintw(w, oy + r, ox + w2 - 1,"│");
    }
    wattroff(w, COLOR_PAIR(CP_OVERLAY));

    wattron(w, COLOR_PAIR(CP_TITLE) | A_BOLD);
    mvwprintw(w, oy, ox + 2, " MASS-SPRING LATTICE ");
    wattroff(w, COLOR_PAIR(CP_TITLE) | A_BOLD);

    int r = oy + 1;
    wattron(w, COLOR_PAIR(CP_OVERLAY));
    mvwprintw(w, r++, ox + 2, "Preset : %-18s", k_preset_names[preset_id]);
    mvwprintw(w, r++, ox + 2, "Theme  : %-18s", k_theme_names[theme]);
    mvwprintw(w, r++, ox + 2, "Nodes  : %-4d  Springs: %-5d", g_nn, g_ns);
    mvwprintw(w, r++, ox + 2, "KE     : %-8.2f PE: %-8.2f", ke, pe);
    mvwprintw(w, r++, ox + 2, "Stretch: %-6.3fx             ", max_stretch);

    /* stretch bar */
    {
        int bar_w = w2 - 4;
        float fill = (max_stretch - 1.0f) * 1.5f;
        if (fill < 0.0f) fill = 0.0f;
        if (fill > 1.0f) fill = 1.0f;
        int filled = (int)(fill * bar_w);
        mvwprintw(w, r, ox + 2, "[");
        for (int c = 0; c < bar_w; c++) {
            int cp = (c < filled) ? stretch_color_pair(1.0f + (float)c / bar_w)
                                  : CP_OVERLAY;
            wattron(w, COLOR_PAIR(cp));
            waddch(w, (c < filled) ? '#' : '.');
            wattroff(w, COLOR_PAIR(cp));
        }
        wattron(w, COLOR_PAIR(CP_OVERLAY));
        waddch(w, ']');
        r++;
    }

    mvwprintw(w, r++, ox + 2, "AvgVel : %-6.2f cells/s       ", avg_vel);
    mvwprintw(w, r++, ox + 2, "k=%-5.0f  damp=%-4.1f grav=%-4.0f",
              k_struct, damping, gravity);
    mvwprintw(w, r++, ox + 2, "dt_crit=%.4fs  k_max≈%-5.0f",
              dt_crit, k_max);
    mvwprintw(w, r++, ox + 2, "ticks=%-7d               ", ticks);
    wattroff(w, COLOR_PAIR(CP_OVERLAY));
}

static void render_status(WINDOW *w, int cols, int rows,
                          double fps, bool paused, int sim_hz,
                          int cnx, int cny, bool node_pinned)
{
    int r = rows - 1;
    wattron(w, COLOR_PAIR(CP_STATUS));
    /* clear row */
    for (int c = 0; c < cols; c++) mvwaddch(w, r, c, ' ');

    mvwprintw(w, r, 1,
        "FPS:%-4.0f Hz:%-3d | Cursor:[%d,%d]%s | "
        "r=reset p=pin i=impulse h=hammer g/G=grav k/K=stiff d/D=damp "
        "1-4=preset t=theme o=overlay",
        fps, sim_hz,
        cnx, cny, node_pinned ? "(PIN)" : "     ");

    if (paused) {
        wattron(w, A_BOLD);
        mvwprintw(w, r, cols - 10, " PAUSED ");
        wattroff(w, A_BOLD);
    }
    wattroff(w, COLOR_PAIR(CP_STATUS));
}

/* ════════════════════════════════════════════════════════════════════
 * §8  SCENE
 * ════════════════════════════════════════════════════════════════════ */
struct Scene {
    int   nx, ny;
    float k_struct, k_shear;
    float damping, gravity, mass;
    float dt;
    float acc;      /* time accumulator for fixed-step integration */
    int   sim_hz;
    int   preset_id;
    int   theme;
    bool  paused;
    bool  show_overlay;
    bool  show_vel;
    int   cursor_nx, cursor_ny;
    int   ticks;
    float ke, pe, max_stretch, avg_vel;
    int   cols, rows;
};

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof(*s));
    s->k_struct    = K_STRUCT_DEF;
    s->k_shear     = K_SHEAR_DEF;
    s->damping     = DAMPING_DEF;
    s->gravity     = GRAVITY_DEF;
    s->mass        = MASS_DEFAULT;
    s->sim_hz      = SIM_HZ_DEFAULT;
    s->dt          = 1.0f / s->sim_hz;
    s->acc         = 0.0f;
    s->preset_id   = PRESET_CLOTH;
    s->theme       = THEME_CLASSIC;
    s->show_overlay= true;
    s->show_vel    = false;
    s->cols        = cols;
    s->rows        = rows;

    int nx = (cols - 4) / NODE_DX + 1;
    int ny = (rows - 4) / NODE_DY + 1;
    if (nx > MAX_NX) nx = MAX_NX;
    if (ny > MAX_NY) ny = MAX_NY;
    if (nx < 3) nx = 3;
    if (ny < 3) ny = 3;
    s->nx = nx;
    s->ny = ny;

    preset_apply(s, PRESET_CLOTH, cols, rows);
    /* start cursor at mid-cloth (row 1 avoids pinned top row) */
    s->cursor_nx = s->nx / 2;
    s->cursor_ny = s->ny / 2;
}

static void scene_tick(Scene *s, float frame_dt)
{
    if (s->paused) return;

    s->acc += frame_dt;
    /* drain stale accumulator to prevent spiral-of-death on slow frames */
    if (s->acc > s->dt * 3.0f) s->acc = s->dt;

    int max_steps = 2;
    while (s->acc >= s->dt && max_steps-- > 0) {
        compute_forces(s->k_struct, s->k_shear,
                       s->damping, s->gravity, s->mass);
        integrate_nodes(s->dt, s->mass);
        s->ticks++;
        s->acc -= s->dt;

        /* detect blow-up: any NaN/Inf position → reset to current preset */
        bool blown = false;
        for (int i = 0; i < g_nn && !blown; i++) {
            if (!isfinite(g_nodes[i].x) || !isfinite(g_nodes[i].y) ||
                !isfinite(g_nodes[i].vx) || !isfinite(g_nodes[i].vy))
                blown = true;
        }
        if (blown) {
            preset_apply(s, (PresetID)s->preset_id, s->cols, s->rows);
            s->acc = 0.0f;
            break;
        }
    }

    compute_stats(s->k_struct, s->mass,
                  &s->ke, &s->pe, &s->max_stretch, &s->avg_vel);
}

static void scene_draw(const Scene *s, WINDOW *w, double fps)
{
    werase(w);

    render_lattice(w, s->cols, s->rows, s->show_vel);
    render_cursor(w, s->cols, s->rows, s->cursor_nx, s->cursor_ny);

    if (s->show_overlay) {
        /* dt_crit = 2 * sqrt(m / (8 * k)) for 8-connected lattice */
        float k_max_stable = (float)M_PI * (float)M_PI * s->mass
                             / (8.0f * s->dt * s->dt);
        float dt_crit = 2.0f * sqrtf(s->mass / (8.0f * s->k_struct));

        render_overlay(w, s->cols, s->rows,
                       s->preset_id, s->theme, s->ticks,
                       s->ke, s->pe, s->max_stretch, s->avg_vel,
                       s->k_struct, s->damping, s->gravity,
                       k_max_stable, dt_crit);
    }

    int cn = s->cursor_ny * g_nx + s->cursor_nx;
    render_status(w, s->cols, s->rows, fps, s->paused, s->sim_hz,
                  s->cursor_nx, s->cursor_ny, g_nodes[cn].pinned);
}

/* ════════════════════════════════════════════════════════════════════
 * §6 (continued)  PRESET APPLY
 * ════════════════════════════════════════════════════════════════════ */
static void preset_apply(Scene *s, PresetID pid, int cols, int rows)
{
    s->preset_id = (int)pid;
    init_lattice(s->nx, s->ny, cols, rows);
    /* place cursor in the free interior so h/i keys work immediately */
    s->cursor_nx = s->nx / 2;
    s->cursor_ny = s->ny / 2;

    bool with_shear = (pid != PRESET_NET);
    build_springs(with_shear);

    switch (pid) {
    /* ── CLOTH: top row pinned, hangs under gravity ── */
    case PRESET_CLOTH:
        s->gravity = GRAVITY_DEF;
        s->damping = DAMPING_DEF;
        for (int c = 0; c < g_nx; c++)
            g_nodes[c].pinned = true;   /* top row */
        break;

    /* ── NET: entire border pinned, gravity off ── */
    case PRESET_NET:
        s->gravity = 0.0f;
        s->damping = DAMPING_DEF;
        for (int c = 0; c < g_nx; c++) {
            g_nodes[c].pinned = true;                          /* top */
            g_nodes[(g_ny-1)*g_nx + c].pinned = true;         /* bottom */
        }
        for (int r = 0; r < g_ny; r++) {
            g_nodes[r * g_nx].pinned = true;                   /* left */
            g_nodes[r * g_nx + g_nx - 1].pinned = true;       /* right */
        }
        /* central impulse */
        {
            int ci = (g_ny / 2) * g_nx + g_nx / 2;
            g_nodes[ci].vy = -IMPULSE_VEL * 3.0f;
        }
        break;

    /* ── WAVE: left column pinned, horizontal wave impulse ── */
    case PRESET_WAVE:
        s->gravity = 0.0f;
        s->damping = 1.0f;
        for (int r = 0; r < g_ny; r++)
            g_nodes[r * g_nx].pinned = true;   /* left column */
        /* sinusoidal velocity along left-quarter columns */
        for (int r = 0; r < g_ny; r++) {
            float phase = (float)r / (float)(g_ny - 1) * 2.0f * (float)M_PI;
            int   col   = g_nx / 4;
            g_nodes[r * g_nx + col].vx = IMPULSE_VEL * sinf(phase);
        }
        break;

    /* ── JELLY: no pins, central impact, low damping ── */
    case PRESET_JELLY:
        s->gravity = 0.0f;
        s->damping = 0.5f;
        {
            int cr = g_ny / 2, cc = g_nx / 2;
            int radius = (g_nx < g_ny ? g_nx : g_ny) / 5;
            for (int r = cr - radius; r <= cr + radius; r++) {
                for (int c = cc - radius; c <= cc + radius; c++) {
                    if (r < 0 || r >= g_ny || c < 0 || c >= g_nx) continue;
                    float dr = (float)(r - cr), dc = (float)(c - cc);
                    if (dr*dr + dc*dc <= (float)(radius*radius)) {
                        float angle = atan2f(dr, dc);
                        float mag   = HAMMER_VEL;
                        g_nodes[r * g_nx + c].vx = mag * cosf(angle);
                        g_nodes[r * g_nx + c].vy = mag * sinf(angle);
                    }
                }
            }
        }
        break;

    default:
        break;
    }
}

/* ════════════════════════════════════════════════════════════════════
 * §9  SCREEN
 * ════════════════════════════════════════════════════════════════════ */
static WINDOW *g_win;

static void screen_init(int theme)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    init_colors(theme);
    g_win = newwin(LINES, COLS, 0, 0);
    keypad(g_win, TRUE);
    nodelay(g_win, TRUE);
}

static void screen_resize(Scene *s)
{
    endwin();
    refresh();
    clear();
    int cols = COLS, rows = LINES;
    wresize(g_win, rows, cols);
    mvwin(g_win, 0, 0);
    init_colors(s->theme);
    s->cols = cols;
    s->rows = rows;

    int nx = (cols - 4) / NODE_DX + 1;
    int ny = (rows - 4) / NODE_DY + 1;
    if (nx > MAX_NX) nx = MAX_NX;
    if (ny > MAX_NY) ny = MAX_NY;
    if (nx < 3) nx = 3;
    if (ny < 3) ny = 3;
    s->nx = nx;
    s->ny = ny;

    preset_apply(s, (PresetID)s->preset_id, cols, rows);
}

static void screen_draw(Scene *s, double fps)
{
    scene_draw(s, g_win, fps);
    wnoutrefresh(g_win);
    doupdate();
}

static void screen_fini(void)
{
    delwin(g_win);
    endwin();
}

/* ════════════════════════════════════════════════════════════════════
 * §10  APP
 * ════════════════════════════════════════════════════════════════════ */
static volatile sig_atomic_t g_resize_flag = 0;
static volatile sig_atomic_t g_quit_flag   = 0;

static void handle_sigwinch(int sig) { (void)sig; g_resize_flag = 1; }
static void handle_sigint  (int sig) { (void)sig; g_quit_flag   = 1; }

/* i key: single-node upward impulse */
static void apply_impulse_to_cursor(Scene *s, float mag)
{
    int i = s->cursor_ny * g_nx + s->cursor_nx;
    if (g_nodes[i].pinned) return;
    g_nodes[i].vy -= mag;
}

/* h key: radial hammer — hits every node within radius r in grid units */
static void apply_hammer(Scene *s, float mag, int radius)
{
    int cx = s->cursor_nx, cy = s->cursor_ny;
    bool any = false;
    for (int dr = -radius; dr <= radius; dr++) {
        for (int dc = -radius; dc <= radius; dc++) {
            int nr = cy + dr, nc = cx + dc;
            if (nr < 0 || nr >= g_ny || nc < 0 || nc >= g_nx) continue;
            if (dr * dr + dc * dc > radius * radius) continue;
            int i = nr * g_nx + nc;
            if (g_nodes[i].pinned) continue;
            /* radial velocity: outward from cursor, scaled by distance */
            float dist = sqrtf((float)(dr*dr + dc*dc));
            float falloff = (dist < 0.5f) ? 1.0f : 1.0f / (dist * 0.6f + 0.4f);
            float vx = (dc == 0 && dr == 0) ? 0.0f : mag * (float)dc / (dist + 0.01f);
            float vy = (dc == 0 && dr == 0) ? -mag : mag * (float)dr / (dist + 0.01f) - mag * 0.5f;
            g_nodes[i].vx += vx * falloff;
            g_nodes[i].vy += vy * falloff;
            any = true;
        }
    }
    /* if cursor is on a pinned node, still hit the ring around it */
    (void)any;
}

static void handle_input(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  /* ESC */
        g_quit_flag = 1;
        break;

    case ' ':
        s->paused = !s->paused;
        break;

    case 's': case 'S':
        if (s->paused) {
            compute_forces(s->k_struct, s->k_shear,
                           s->damping, s->gravity, s->mass);
            integrate_nodes(s->dt, s->mass);
            s->ticks++;
            compute_stats(s->k_struct, s->mass,
                          &s->ke, &s->pe, &s->max_stretch, &s->avg_vel);
        }
        break;

    case 'r': case 'R':
        preset_apply(s, (PresetID)s->preset_id, s->cols, s->rows);
        s->ticks = 0;
        break;

    case 'o': case 'O':
        s->show_overlay = !s->show_overlay;
        break;

    case 'v': case 'V':
        s->show_vel = !s->show_vel;
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % THEME_COUNT;
        init_colors(s->theme);
        break;

    /* cursor movement */
    case KEY_UP:
        if (s->cursor_ny > 0) s->cursor_ny--;
        break;
    case KEY_DOWN:
        if (s->cursor_ny < g_ny - 1) s->cursor_ny++;
        break;
    case KEY_LEFT:
        if (s->cursor_nx > 0) s->cursor_nx--;
        break;
    case KEY_RIGHT:
        if (s->cursor_nx < g_nx - 1) s->cursor_nx++;
        break;

    case 'p': case 'P': {
        int i = s->cursor_ny * g_nx + s->cursor_nx;
        g_nodes[i].pinned = !g_nodes[i].pinned;
        if (g_nodes[i].pinned) {
            g_nodes[i].vx = 0; g_nodes[i].vy = 0;
        }
        break;
    }

    case 'i': case 'I':
        apply_impulse_to_cursor(s, IMPULSE_VEL);
        break;

    case 'h': case 'H':
        apply_hammer(s, HAMMER_VEL, 3);
        break;

    /* gravity */
    case 'g':
        s->gravity += GRAVITY_STEP;
        if (s->gravity > GRAVITY_MAX) s->gravity = GRAVITY_MAX;
        break;
    case 'G':
        s->gravity -= GRAVITY_STEP;
        if (s->gravity < GRAVITY_MIN) s->gravity = GRAVITY_MIN;
        break;

    /* stiffness */
    case 'k':
        s->k_struct += K_STEP;
        s->k_shear   = s->k_struct * 0.5f;
        if (s->k_struct > K_MAX) { s->k_struct = K_MAX; s->k_shear = K_MAX * 0.5f; }
        break;
    case 'K':
        s->k_struct -= K_STEP;
        s->k_shear   = s->k_struct * 0.5f;
        if (s->k_struct < K_MIN) { s->k_struct = K_MIN; s->k_shear = K_MIN * 0.5f; }
        break;

    /* damping */
    case 'd':
        s->damping += DAMPING_STEP;
        if (s->damping > DAMPING_MAX) s->damping = DAMPING_MAX;
        break;
    case 'D':
        s->damping -= DAMPING_STEP;
        if (s->damping < DAMPING_MIN) s->damping = DAMPING_MIN;
        break;

    /* presets */
    case '1': preset_apply(s, PRESET_CLOTH, s->cols, s->rows); s->ticks = 0; break;
    case '2': preset_apply(s, PRESET_NET,   s->cols, s->rows); s->ticks = 0; break;
    case '3': preset_apply(s, PRESET_WAVE,  s->cols, s->rows); s->ticks = 0; break;
    case '4': preset_apply(s, PRESET_JELLY, s->cols, s->rows); s->ticks = 0; break;

    default:
        break;
    }
}

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGINT,   handle_sigint);

    srand((unsigned)time(NULL));

    screen_init(THEME_CLASSIC);

    Scene scene;
    scene_init(&scene, COLS, LINES);

    ns_t   frame_ns  = 1000000000LL / TARGET_FPS;
    ns_t   prev_time = clock_now();
    double fps       = (double)TARGET_FPS;
    long   fps_count = 0;
    ns_t   fps_acc   = 0;

    while (!g_quit_flag) {
        if (g_resize_flag) {
            g_resize_flag = 0;
            screen_resize(&scene);
        }

        ns_t now  = clock_now();
        ns_t diff = now - prev_time;
        prev_time = now;

        float frame_dt = (float)diff * 1e-9f;
        if (frame_dt > 0.1f) frame_dt = 0.1f;

        fps_acc   += diff;
        fps_count++;
        if (fps_acc >= 500000000LL) {
            fps       = (double)fps_count / ((double)fps_acc * 1e-9);
            fps_count = 0;
            fps_acc   = 0;
        }

        scene_tick(&scene, frame_dt);
        screen_draw(&scene, fps);

        int ch;
        while ((ch = wgetch(g_win)) != ERR)
            handle_input(&scene, ch);

        ns_t elapsed = clock_now() - now;
        clock_sleep_ns(frame_ns - elapsed);
    }

    screen_fini();
    return 0;
}
