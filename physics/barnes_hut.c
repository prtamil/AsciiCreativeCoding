/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnes_hut.c — Barnes–Hut O(N log N) Gravity Simulation
 *
 * Up to 800 bodies in pixel space. A quadtree is rebuilt every tick;
 * each body's force is computed by the Barnes–Hut criterion:
 *   if  s / d  <  θ  →  treat node as a single point mass
 *   else             →  recurse into children
 *
 * Rendering: a brightness glow layer (decays over time) + direct body
 * glyphs drawn on top, coloured by speed.  Bodies are ALWAYS visible.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Quadtree node stores:
 *   total_mass, cx, cy  (centre of mass)
 *   child[4]            (NW=0, NE=1, SW=2, SE=3)
 *   body_idx            (≥0 if leaf with one body, else -1)
 *
 * Force criterion (s = node side, d = distance to node COM):
 *   s / d < θ  →  F = G · m_i · M_node / (d² + ε²)^(3/2)
 *   else       →  recurse children
 *
 * Integration: symplectic Euler (v += a·dt, x += v·dt)
 *
 * Galaxy:  body 0 = central massive anchor (mass = N×3).
 *          Others placed in Keplerian orbits: v = sqrt(G·M_c / r).
 *          Differential rotation shears the disk into spiral patterns.
 *
 * Cluster: cold uniform disc — self-gravity drives collapse into a
 *          tight core, then virialises and oscillates dramatically.
 *
 * Binary:  two rotating clusters approach and merge — tidal streams
 *          and chaotic ejections during the collision.
 *
 * Three presets:
 *   1  Galaxy  — central BH + Keplerian disk
 *   2  Cluster — cold collapse + virialisation
 *   3  Binary  — cluster merger
 *
 * Keys:
 *   q / ESC   quit
 *   p / space pause / resume
 *   r         reset current preset
 *   1 / 2 / 3 select preset
 *   t / T     theme next / prev
 *   o         toggle quadtree overlay
 *   f         toggle fast-forward (4× speed)
 *   + / -     add / remove bodies (±50)
 *   g / G     gravity constant up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/barnes_hut.c -o barnes_hut -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Barnes-Hut tree-code (Barnes & Hut, 1986).
 *                  Reduces N-body gravitational force from O(N²) to O(N log N)
 *                  by grouping distant bodies into their centre-of-mass using a
 *                  quadtree.  The approximation criterion θ (opening angle):
 *                    if s/d < θ → accept the node as a point mass
 *                    else       → recurse into children
 *                  Smaller θ → more accurate, more expensive; θ=0 → exact O(N²).
 *
 * Physics        : Softened Newtonian gravity:
 *                    F = G · mᵢ · Mⱼ / (d² + ε²)^(3/2)  (in 2-D pixel space)
 *                  Softening ε² prevents singularities at close approach (r→0).
 *                  Galaxy preset uses Keplerian IC: v_orbit = √(G·M_central / r)
 *                  so each body starts in a circular orbit — differential rotation
 *                  then winds the disk into spiral arms over time.
 *
 * Data-structure : Quadtree with a pre-allocated node pool (NODE_POOL_MAX nodes).
 *                  Pool avoids dynamic malloc per node; rebuilt each tick.
 *                  Leaves store one body index; internal nodes store aggregated
 *                  (total_mass, cx, cy) = centre of mass of all bodies below.
 *
 * Performance    : Typical cost at N=400: ~O(400 × log₂400 × θ_factor) ≈ 3600
 *                  force evaluations vs 400² / 2 = 80000 for brute force.
 *                  QT_MAX_DEPTH=32 caps recursion; NODE_POOL_MAX=16000 supports
 *                  up to 800 bodies in a well-distributed quadtree.
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
    RENDER_FPS       = 30,
    SIM_HZ           = 60,
    FPS_UPDATE_MS    = 500,

    N_BODIES_MAX     = 800,
    N_BODIES_DEF     = 400,
    N_BODIES_STEP    = 50,

    NODE_POOL_MAX    = 16000,
    QT_MAX_DEPTH     = 32,

    GRID_ROWS_MAX    = 120,
    GRID_COLS_MAX    = 400,

    N_PRESETS        = 3,
    N_THEMES         = 5,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_W          8
#define CELL_H          16

#define G_DEF           12.0f   /* gravitational constant in pixel² units; tuned so
                                  * galaxy-disk bodies orbit in ~10–20 seconds on screen */
#define G_STEP          2.0f
#define G_MIN           1.0f
#define G_MAX           200.0f

#define SOFTENING       10.0f
#define SOFT2           (SOFTENING * SOFTENING)

#define THETA_DEF       0.5f

/* Glow layer: slow decay so orbital paths stay visible */
#define DECAY           0.93f

/* Galaxy: central black hole mass = N_BODIES_DEF × BH_MASS_FACTOR */
#define BH_MASS_FACTOR  3.0f

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / themes                                                     */
/* ===================================================================== */

enum {
    CP_HUD  = 1,
    CP_L1   = 2,   /* dim  — slow/far glow     */
    CP_L2   = 3,
    CP_L3   = 4,
    CP_L4   = 5,
    CP_L5   = 6,   /* bright — fast/dense      */
    CP_TREE = 7,
    CP_BH   = 8,   /* central black hole       */
};

typedef struct {
    const char *name;
    int hi256[5];
    int hi8[5];
    int tree256;
    int tree8;
    int bh256;
    int bh8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    {   /* Galaxy — warm yellows/whites */
        "Galaxy",
        { 58, 136, 178, 220, 231 },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        240, COLOR_WHITE, 196, COLOR_RED,
    },
    {   /* Nebula — purples */
        "Nebula",
        { 54, 92, 129, 171, 213 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE },
        237, COLOR_WHITE, 226, COLOR_YELLOW,
    },
    {   /* Fire — reds/oranges */
        "Fire",
        { 52, 88, 160, 202, 226 },
        { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE },
        236, COLOR_RED, 51, COLOR_CYAN,
    },
    {   /* Ice — blues/cyans */
        "Ice",
        { 17, 27, 39, 75, 159 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE },
        237, COLOR_CYAN, 201, COLOR_MAGENTA,
    },
    {   /* Mono — grays */
        "Mono",
        { 235, 238, 242, 246, 252 },
        { COLOR_BLACK, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        234, COLOR_WHITE, 231, COLOR_WHITE,
    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    const Theme *th = &k_themes[theme];
    if (COLORS >= 256) {
        init_pair(CP_HUD,  231,          -1);
        init_pair(CP_TREE, th->tree256,  -1);
        init_pair(CP_BH,   th->bh256,    -1);
        for (int i = 0; i < 5; i++)
            init_pair(CP_L1 + i, th->hi256[i], -1);
    } else {
        init_pair(CP_HUD,  COLOR_WHITE,  -1);
        init_pair(CP_TREE, th->tree8,    -1);
        init_pair(CP_BH,   th->bh8,      -1);
        for (int i = 0; i < 5; i++)
            init_pair(CP_L1 + i, th->hi8[i], -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Body                                                      */
/* ===================================================================== */

typedef struct {
    float px, py;
    float vx, vy;
    float mass;
    bool  active;
    bool  anchor;   /* true = skip force integration (central BH) */
} Body;

static Body  g_bodies[N_BODIES_MAX];
static int   g_n_bodies  = N_BODIES_DEF;
static float g_G         = G_DEF;
static float g_v_max     = 1.0f;   /* rolling max speed for color mapping */

/* Simple LCG */
static uint32_t g_rng = 12345u;
static uint32_t rng_next(void)  { g_rng = g_rng * 1664525u + 1013904223u; return g_rng; }
static float    rng_f(void)     { return (float)(rng_next() & 0x7FFFFFFFu) / (float)0x80000000u; }
static float    rng_range(float lo, float hi) { return lo + rng_f() * (hi - lo); }

/* ===================================================================== */
/* §6  quadtree                                                           */
/* ===================================================================== */

typedef struct {
    float x0, y0, x1, y1;
    float total_mass, cx, cy;
    int   child[4];   /* NW=0, NE=1, SW=2, SE=3; -1 = absent */
    int   body_idx;   /* ≥0 if leaf with one body, else -1    */
    int   depth;
} QNode;

static QNode g_pool[NODE_POOL_MAX];
static int   g_pool_top = 0;

static int qt_alloc(float x0, float y0, float x1, float y1, int depth)
{
    if (g_pool_top >= NODE_POOL_MAX) return -1;
    int idx = g_pool_top++;
    QNode *n = &g_pool[idx];
    n->x0 = x0; n->y0 = y0; n->x1 = x1; n->y1 = y1;
    n->total_mass = 0.0f; n->cx = 0.0f; n->cy = 0.0f;
    n->child[0] = n->child[1] = n->child[2] = n->child[3] = -1;
    n->body_idx = -1;
    n->depth = depth;
    return idx;
}

static int qt_quadrant(const QNode *n, float px, float py)
{
    float mx = (n->x0 + n->x1) * 0.5f;
    float my = (n->y0 + n->y1) * 0.5f;
    return (px >= mx ? 1 : 0) | (py >= my ? 2 : 0);
}

static void qt_subdivide(QNode *n)
{
    float mx = (n->x0 + n->x1) * 0.5f;
    float my = (n->y0 + n->y1) * 0.5f;
    n->child[0] = qt_alloc(n->x0, n->y0, mx,    my,    n->depth + 1);
    n->child[1] = qt_alloc(mx,    n->y0, n->x1, my,    n->depth + 1);
    n->child[2] = qt_alloc(n->x0, my,    mx,    n->y1, n->depth + 1);
    n->child[3] = qt_alloc(mx,    my,    n->x1, n->y1, n->depth + 1);
}

static void qt_insert(int ni, int bi)
{
    if (ni < 0) return;
    QNode *n = &g_pool[ni];
    Body  *b = &g_bodies[bi];

    /* Incremental COM update */
    float new_mass = n->total_mass + b->mass;
    n->cx = (n->cx * n->total_mass + b->px * b->mass) / new_mass;
    n->cy = (n->cy * n->total_mass + b->py * b->mass) / new_mass;
    n->total_mass = new_mass;

    if (n->body_idx < 0 && n->child[0] < 0) {
        /* Empty leaf */
        n->body_idx = bi;
        return;
    }
    if (n->depth >= QT_MAX_DEPTH) return;

    if (n->body_idx >= 0) {
        /* Occupied leaf: push existing body down */
        int existing = n->body_idx;
        n->body_idx = -1;
        qt_subdivide(n);
        int q = qt_quadrant(n, g_bodies[existing].px, g_bodies[existing].py);
        qt_insert(n->child[q], existing);
    } else if (n->child[0] < 0) {
        qt_subdivide(n);
    }
    int q = qt_quadrant(n, b->px, b->py);
    qt_insert(n->child[q], bi);
}

static int qt_build(int cols, int rows)
{
    g_pool_top = 0;
    int root = qt_alloc(0.0f, 0.0f, (float)pw(cols), (float)ph(rows), 0);
    for (int i = 0; i < g_n_bodies; i++)
        if (g_bodies[i].active) qt_insert(root, i);
    return root;
}

static void qt_force(int ni, int bi, float *fx, float *fy)
{
    if (ni < 0) return;
    QNode *n = &g_pool[ni];
    if (n->total_mass == 0.0f) return;
    if (n->body_idx == bi) return;   /* skip self at leaf */

    Body  *b  = &g_bodies[bi];
    float  dx = n->cx - b->px;
    float  dy = n->cy - b->py;
    float  d2 = dx*dx + dy*dy + SOFT2;
    float  d  = sqrtf(d2);
    float  s  = n->x1 - n->x0;

    if ((s / d) < THETA_DEF || n->child[0] < 0) {
        float inv = g_G * n->total_mass / (d2 * d);
        *fx += inv * dx * b->mass;
        *fy += inv * dy * b->mass;
        return;
    }
    for (int c = 0; c < 4; c++)
        qt_force(n->child[c], bi, fx, fy);
}

/* Draw quadtree grid lines for depth ≤ 3 */
static void qt_draw_overlay(int ni, int rows, int cols)
{
    if (ni < 0) return;
    QNode *n = &g_pool[ni];
    if (n->total_mass == 0.0f || n->depth > 3) return;

    int cx0 = px_to_cell_x(n->x0), cx1 = px_to_cell_x(n->x1);
    int cy0 = px_to_cell_y(n->y0), cy1 = px_to_cell_y(n->y1);
    int mxc = px_to_cell_x((n->x0 + n->x1) * 0.5f);
    int myc = px_to_cell_y((n->y0 + n->y1) * 0.5f);

    if ((cx1 - cx0) < 2 || (cy1 - cy0) < 2) return;

    attron(COLOR_PAIR(CP_TREE));
    for (int c = cx0+1; c < cx1 && c < cols; c++)
        if (myc >= 0 && myc < rows) mvaddch(myc, c, ACS_HLINE);
    for (int r = cy0+1; r < cy1 && r < rows; r++)
        if (mxc >= 0 && mxc < cols) mvaddch(r, mxc, ACS_VLINE);
    if (myc >= 0 && myc < rows && mxc >= 0 && mxc < cols)
        mvaddch(myc, mxc, ACS_PLUS);
    attroff(COLOR_PAIR(CP_TREE));

    for (int c = 0; c < 4; c++)
        qt_draw_overlay(n->child[c], rows, cols);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static float g_bright[GRID_ROWS_MAX][GRID_COLS_MAX];
static bool  g_paused   = false;
static bool  g_overlay  = false;
static bool  g_fastfwd  = false;
static int   g_preset   = 0;
static int   g_theme    = 0;
static float g_sim_dt   = 1.0f / (float)SIM_HZ;
static int   g_qt_root  = 0;

/* ── Presets ─────────────────────────────────────────────────────────── */

/*
 * Preset 1 — Galaxy
 *
 * Body 0 = central black hole (anchor, mass = N×BH_MASS_FACTOR).
 * Remaining bodies in Keplerian circular orbits: v = sqrt(G·M_bh / r).
 * Differential rotation (inner orbits faster) shears the disk and
 * spontaneously generates spiral structure over a few orbital periods.
 */
static void preset_galaxy(int cols, int rows)
{
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;

    /* Choose disk radius to fit in the smaller screen dimension */
    float half = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.5f;
    float R    = half * 0.75f;

    float M_bh = (float)g_n_bodies * BH_MASS_FACTOR;

    /* Body 0 = central black hole (anchor — very heavy, not integrated) */
    g_bodies[0].px     = cx;
    g_bodies[0].py     = cy;
    g_bodies[0].vx     = 0.0f;
    g_bodies[0].vy     = 0.0f;
    g_bodies[0].mass   = M_bh;
    g_bodies[0].active = true;
    g_bodies[0].anchor = true;

    /* Disk bodies: uniform area distribution */
    for (int i = 1; i < g_n_bodies; i++) {
        /* sqrt gives uniform area density */
        float r     = R * (0.08f + 0.92f * sqrtf(rng_f()));
        float theta = rng_f() * 2.0f * (float)M_PI;

        float bx = cx + cosf(theta) * r;
        float by = cy + sinf(theta) * r;   /* circles in pixel space */

        /* Keplerian velocity: v = sqrt(G * M_bh / r) */
        float v_kep = sqrtf(g_G * M_bh / r);
        /* Small scatter keeps orbits slightly elliptical — more interesting */
        float v = v_kep * (1.0f + rng_range(-0.06f, 0.06f));

        /* Tangential direction (perpendicular to radius, CCW) */
        float nx = -(by - cy) / r;
        float ny =  (bx - cx) / r;

        g_bodies[i].px     = bx;
        g_bodies[i].py     = by;
        g_bodies[i].vx     = nx * v;
        g_bodies[i].vy     = ny * v;
        g_bodies[i].mass   = 1.0f;
        g_bodies[i].active = true;
        g_bodies[i].anchor = false;
    }
}

/*
 * Preset 2 — Cold Collapse
 *
 * Uniform disc, zero initial velocity.  Self-gravity drives rapid
 * collapse into a dense core → virialises → oscillates.  The
 * collapse happens fast enough to watch (a few seconds).
 */
static void preset_cluster(int cols, int rows)
{
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;
    /* Compact radius → strong gravity → fast collapse */
    float R  = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.28f;

    /* Slight spin so it doesn't just collapse to a point */
    float M_tot = (float)g_n_bodies;
    float v_spin = sqrtf(g_G * M_tot / R) * 0.12f;   /* 12% of virial speed */

    for (int i = 0; i < g_n_bodies; i++) {
        float r     = R * sqrtf(rng_f());
        float theta = rng_f() * 2.0f * (float)M_PI;
        float bx    = cx + cosf(theta) * r;
        float by    = cy + sinf(theta) * r;

        /* Tangential spin */
        float nx = -(by - cy) / (r + 1.0f);
        float ny =  (bx - cx) / (r + 1.0f);

        g_bodies[i].px     = bx;
        g_bodies[i].py     = by;
        g_bodies[i].vx     = nx * v_spin;
        g_bodies[i].vy     = ny * v_spin;
        g_bodies[i].mass   = 1.0f;
        g_bodies[i].active = true;
        g_bodies[i].anchor = false;
    }
}

/*
 * Preset 3 — Binary Merger
 *
 * Two counter-rotating clusters approach each other.  Tidal forces
 * strip outer bodies into long streams; the cores merge with a burst.
 */
static void preset_binary(int cols, int rows)
{
    float cx = (float)pw(cols) * 0.5f;
    float cy = (float)ph(rows) * 0.5f;
    float R  = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.18f;
    float sep = R * 2.4f;

    int half = g_n_bodies / 2;

    /* Approach velocity: clusters meet in ~5 seconds at default SIM_HZ */
    float approach = sep / (5.0f * (float)SIM_HZ);

    /* Internal spin speed */
    float M_half  = (float)half;
    float v_spin  = sqrtf(g_G * M_half / R) * 0.35f;

    for (int k = 0; k < 2; k++) {
        float ox = cx + (k == 0 ? -sep : sep);
        float oy = cy;
        float vx_drift = (k == 0 ? approach : -approach);
        float spin_dir = (k == 0 ? 1.0f : -1.0f);   /* counter-rotate */

        int start = k * half;
        int end   = (k == 1) ? g_n_bodies : half;

        for (int i = start; i < end; i++) {
            float r     = R * sqrtf(rng_f());
            float theta = rng_f() * 2.0f * (float)M_PI;
            float bx    = ox + cosf(theta) * r;
            float by    = oy + sinf(theta) * r;
            float nx    = -(by - oy) / (r + 1.0f);
            float ny    =  (bx - ox) / (r + 1.0f);

            g_bodies[i].px     = bx;
            g_bodies[i].py     = by;
            g_bodies[i].vx     = vx_drift + spin_dir * nx * v_spin;
            g_bodies[i].vy     = spin_dir * ny * v_spin;
            g_bodies[i].mass   = 1.0f;
            g_bodies[i].active = true;
            g_bodies[i].anchor = false;
        }
    }
}

static void scene_reset(int cols, int rows)
{
    memset(g_bright, 0, sizeof(g_bright));
    memset(g_bodies, 0, sizeof(Body) * N_BODIES_MAX);
    g_v_max = 1.0f;
    g_rng   = 99991u;

    switch (g_preset) {
        case 0: preset_galaxy (cols, rows); break;
        case 1: preset_cluster(cols, rows); break;
        case 2: preset_binary (cols, rows); break;
    }
}

/* ── scene_tick ──────────────────────────────────────────────────────── */

static void scene_tick(int cols, int rows)
{
    g_qt_root = qt_build(cols, rows);

    float W = (float)pw(cols);
    float H = (float)ph(rows);

    for (int i = 0; i < g_n_bodies; i++) {
        if (!g_bodies[i].active || g_bodies[i].anchor) continue;

        float fx = 0.0f, fy = 0.0f;
        qt_force(g_qt_root, i, &fx, &fy);

        float ax = fx / g_bodies[i].mass;
        float ay = fy / g_bodies[i].mass;

        g_bodies[i].vx += ax * g_sim_dt;
        g_bodies[i].vy += ay * g_sim_dt;
        g_bodies[i].px += g_bodies[i].vx * g_sim_dt;
        g_bodies[i].py += g_bodies[i].vy * g_sim_dt;

        /* Deactivate escapees */
        if (g_bodies[i].px < -W      || g_bodies[i].px > 2.0f * W ||
            g_bodies[i].py < -H * 2  || g_bodies[i].py > 3.0f * H)
            g_bodies[i].active = false;

        /* Track max speed for coloring */
        float spd = sqrtf(g_bodies[i].vx*g_bodies[i].vx
                        + g_bodies[i].vy*g_bodies[i].vy);
        if (spd > g_v_max) g_v_max = spd;
    }

    /* Slowly decay v_max so colormap adapts to current dynamics */
    g_v_max *= 0.9995f;
    if (g_v_max < 0.1f) g_v_max = 0.1f;
}

/* ── scene_draw ──────────────────────────────────────────────────────── */

static void scene_draw(int cols, int rows, float alpha)
{
    (void)alpha;

    /* ── 1. Accumulate glow for this frame ───────────────────────────── */
    for (int i = 0; i < g_n_bodies; i++) {
        if (!g_bodies[i].active) continue;
        int cr = px_to_cell_y(g_bodies[i].py);
        int cc = px_to_cell_x(g_bodies[i].px);
        if (cr >= 0 && cr < rows && cc >= 0 && cc < cols)
            g_bright[cr][cc] += g_bodies[i].mass > 1.0f ? 4.0f : 1.0f;
    }

    /* ── 2. Find max brightness for normalisation ────────────────────── */
    float b_max = 1.0f;
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            if (g_bright[r][c] > b_max) b_max = g_bright[r][c];

    /* ── 3. Render glow layer and decay ──────────────────────────────── */
    static const char k_glow[] = ".:+oO";
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float b = g_bright[r][c];
            g_bright[r][c] *= DECAY;
            if (b < 0.4f) continue;
            float norm = b / b_max;
            int   lvl  = (int)(norm * 4.9f);
            if (lvl > 4) lvl = 4;
            int    pair = CP_L1 + lvl;
            attr_t attr = COLOR_PAIR(pair);
            if (lvl >= 3) attr |= A_BOLD;
            mvaddch(r, c, (chtype)k_glow[lvl] | attr);
        }
    }

    /* ── 4. Draw each body directly on top (always visible) ─────────── */
    for (int i = 0; i < g_n_bodies; i++) {
        if (!g_bodies[i].active) continue;

        int cr = px_to_cell_y(g_bodies[i].py);
        int cc = px_to_cell_x(g_bodies[i].px);
        if (cr < 1 || cr >= rows - 1 || cc < 0 || cc >= cols) continue;

        if (g_bodies[i].anchor) {
            /* Central black hole — distinctive glyph */
            mvaddch(cr,   cc,   (chtype)'@' | COLOR_PAIR(CP_BH) | A_BOLD);
            if (cc > 0)   mvaddch(cr, cc-1, (chtype)'(' | COLOR_PAIR(CP_BH));
            if (cc < cols-1) mvaddch(cr, cc+1, (chtype)')' | COLOR_PAIR(CP_BH));
            continue;
        }

        /* Color by speed relative to current v_max */
        float spd  = sqrtf(g_bodies[i].vx*g_bodies[i].vx
                          + g_bodies[i].vy*g_bodies[i].vy);
        float norm = spd / g_v_max;

        int pair;
        chtype ch;
        if (norm > 0.80f) {
            pair = CP_L5; ch = '*';
        } else if (norm > 0.55f) {
            pair = CP_L4; ch = '+';
        } else if (norm > 0.30f) {
            pair = CP_L3; ch = 'o';
        } else if (norm > 0.10f) {
            pair = CP_L2; ch = '.';
        } else {
            pair = CP_L1; ch = ',';
        }

        mvaddch(cr, cc, ch | COLOR_PAIR(pair) | A_BOLD);
    }

    /* ── 5. Optional quadtree overlay ───────────────────────────────── */
    if (g_overlay)
        qt_draw_overlay(g_qt_root, rows, cols);
}

/* ===================================================================== */
/* §8  screen / HUD                                                       */
/* ===================================================================== */

static void hud_draw(int cols, int rows, double fps)
{
    /* Count active (non-anchor) bodies */
    int active = 0;
    for (int i = 0; i < g_n_bodies; i++)
        if (g_bodies[i].active && !g_bodies[i].anchor) active++;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);

    mvprintw(0, 0, "Barnes-Hut  N=%d(%d)  G=%.0f  %s%s",
             g_n_bodies, active, g_G,
             k_themes[g_theme].name,
             g_fastfwd ? "  [4x]" : "");

    char fps_buf[24];
    snprintf(fps_buf, sizeof(fps_buf), "%.0f fps", fps);
    mvprintw(0, cols - (int)strlen(fps_buf) - 1, "%s", fps_buf);

    /* Preset description */
    static const char *k_desc[N_PRESETS] = {
        "Galaxy: BH disk — watch spiral arms form",
        "Cluster: cold collapse — core bounce incoming",
        "Binary merger — tidal streams + chaotic ejections",
    };
    mvprintw(1, 0, "[%d] %s%s", g_preset + 1, k_desc[g_preset],
             g_paused ? "  [PAUSED]" : "");

    /* Bottom controls */
    const char *ctrl =
        "q:quit  p:pause  r:reset  1-3:preset  t/T:theme  o:overlay  f:4x  +/-:bodies  g/G:grav";
    if ((int)strlen(ctrl) < cols - 1)
        mvprintw(rows - 1, 0, "%s", ctrl);
    else
        mvprintw(rows - 1, 0, "q:quit  p/space:pause  r:reset  1-3:preset  f:4x  t:theme");

    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static void on_sigwinch(int sig) { (void)sig; g_resize = 1; }

int main(void)
{
    signal(SIGWINCH, on_sigwinch);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    color_init(g_theme);

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_reset(cols, rows);

    int64_t render_ns  = NS_PER_SEC / RENDER_FPS;
    int64_t sim_ns     = NS_PER_SEC / SIM_HZ;
    int64_t t_last_sim = clock_ns();
    int64_t sim_acc    = 0;
    int64_t fps_t0     = clock_ns();
    int     fps_frames = 0;
    double  fps_disp   = 0.0;

    for (;;) {
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            scene_reset(cols, rows);
            color_init(g_theme);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:
                    endwin(); return 0;

                case 'p': case ' ':
                    g_paused = !g_paused;
                    break;

                case 'r':
                    scene_reset(cols, rows);
                    break;

                case '1': case '2': case '3':
                    g_preset = ch - '1';
                    scene_reset(cols, rows);
                    break;

                case 't':
                    g_theme = (g_theme + 1) % N_THEMES;
                    color_init(g_theme);
                    break;

                case 'T':
                    g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
                    color_init(g_theme);
                    break;

                case 'o':
                    g_overlay = !g_overlay;
                    break;

                case 'f':
                    g_fastfwd = !g_fastfwd;
                    break;

                case '+': case '=':
                    g_n_bodies += N_BODIES_STEP;
                    if (g_n_bodies > N_BODIES_MAX) g_n_bodies = N_BODIES_MAX;
                    scene_reset(cols, rows);
                    break;

                case '-':
                    g_n_bodies -= N_BODIES_STEP;
                    if (g_n_bodies < N_BODIES_STEP) g_n_bodies = N_BODIES_STEP;
                    scene_reset(cols, rows);
                    break;

                case 'g':
                    g_G += G_STEP;
                    if (g_G > G_MAX) g_G = G_MAX;
                    break;

                case 'G':
                    g_G -= G_STEP;
                    if (g_G < G_MIN) g_G = G_MIN;
                    break;
            }
        }

        /* Physics accumulator */
        int64_t now = clock_ns();
        sim_acc += now - t_last_sim;
        t_last_sim = now;

        if (!g_paused) {
            /* Fast-forward: allow 4× the normal budget */
            int64_t cap = g_fastfwd ? sim_ns * 16 : sim_ns * 4;
            if (sim_acc > cap) sim_acc = cap;
            while (sim_acc >= sim_ns) {
                scene_tick(cols, rows);
                sim_acc -= sim_ns;
            }
        }

        float alpha = (float)sim_acc / (float)sim_ns;

        erase();
        scene_draw(cols, rows, alpha);
        hud_draw(cols, rows, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        fps_frames++;
        int64_t fps_elapsed = clock_ns() - fps_t0;
        if (fps_elapsed >= (int64_t)FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp   = (double)fps_frames / ((double)fps_elapsed / 1e9);
            fps_frames = 0;
            fps_t0     = clock_ns();
        }

        int64_t t_next = now + render_ns;
        clock_sleep_ns(t_next - clock_ns());
    }
}
