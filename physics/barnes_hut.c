/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnes_hut.c — Barnes–Hut O(N log N) Gravity Simulation
 *
 * Up to 800 bodies in pixel space. A quadtree is rebuilt every tick;
 * each body's force is computed by the Barnes–Hut criterion:
 *   if  s / d  <  θ  →  treat node as a single point mass
 *   else             →  recurse into children
 *
 * Rendering: layered for depth.  An additive brightness buffer captures
 * Bresenham streaks along each body's orbit arc and decays at 0.93/frame,
 * so paths persist as glowing trails.  Per-body glyphs sit on top,
 * coloured by speed with two glyph variants per band so the cloud reads
 * as textured rather than stencilled.  The central black hole pulses
 * with a sin() phase and is wreathed in a small aspect-corrected
 * accretion halo, so the mass anchor visibly acts the part.  Seven
 * themes recolour everything without touching the physics (t / T).
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
 *   t / T     theme next / prev (7 themes: Galaxy / Nebula / Fire / Ice /
 *                                 Mono / Aurora / Plasma)
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
 *
 * References     : 1. Barnes, J. & Hut, P. — "A hierarchical O(N log N)
 *                     force-calculation algorithm", Nature 324, pp. 446-
 *                     449 (1986).  The original paper.  Defines the
 *                     s/d < θ opening criterion the §6 quadtree
 *                     implements; θ = 0.5 is the value Barnes & Hut
 *                     recommended for general-purpose accuracy.
 *                  2. Aarseth, S. J. — "Gravitational N-Body Simulations:
 *                     Tools and Algorithms", Cambridge Univ. Press (2003).
 *                     Comprehensive textbook on N-body methods: softening
 *                     (our ε² in F = G·m·M/(d²+ε²)^(3/2)), time integration,
 *                     tree-code variants, energy diagnostics.
 *                  3. Binney, J. & Tremaine, S. — "Galactic Dynamics",
 *                     2nd ed., Princeton Univ. Press (2008).  The
 *                     galactic-dynamics standard.  Keplerian orbits,
 *                     virial theorem, spiral-structure formation through
 *                     differential rotation — directly justifies the
 *                     setup of all three presets (galaxy / cluster / binary).
 *                  4. Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes,
 *                     J. F. — "Computer Graphics: Principles and Practice",
 *                     3rd ed., Addison-Wesley (2013).  Ch. 2 covers
 *                     Bresenham's line algorithm, used by streak_glow()
 *                     to lay orbit arcs into the brightness buffer.
 *                  5. Ware, C. — "Information Visualization: Perception
 *                     for Design", 4th ed., Morgan Kaufmann (2020).
 *                     Perceptually-ordered colour and luminance ramps —
 *                     backs the 5-level brightness palette (CP_L1..CP_L5)
 *                     and the per-band glyph density ramp '.:*o@'.
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
    N_THEMES         = 7,
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
    CP_HUD  = 1,   /* top status bar (bright yellow)         */
    CP_L1   = 2,   /* dim  — slow/far glow                   */
    CP_L2   = 3,
    CP_L3   = 4,
    CP_L4   = 5,
    CP_L5   = 6,   /* bright — fast/dense                    */
    CP_TREE = 7,
    CP_BH   = 8,   /* central black hole                     */
    CP_HINT = 9,   /* bottom hint bar (bright cyan)          */
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
        { 24, 31, 39, 75, 159 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE },
        244, COLOR_CYAN, 201, COLOR_MAGENTA,
    },
    {   /* Mono — grays */
        "Mono",
        { 240, 244, 247, 250, 254 },
        { COLOR_BLACK, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        242, COLOR_WHITE, 231, COLOR_WHITE,
    },
    {   /* Aurora — green -> cyan -> white, magenta anchor */
        "Aurora",
        { 28, 36, 49, 87, 159 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE },
        244, COLOR_GREEN, 201, COLOR_MAGENTA,
    },
    {   /* Plasma — purple -> pink -> yellow, cyan anchor */
        "Plasma",
        { 53, 92, 165, 213, 226 },
        { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE },
        240, COLOR_MAGENTA, 51, COLOR_CYAN,
    },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    const Theme *th = &k_themes[theme];
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226,          -1);   /* bright yellow */
        init_pair(CP_HINT,  51,          -1);   /* bright cyan   */
        init_pair(CP_TREE, th->tree256,  -1);
        init_pair(CP_BH,   th->bh256,    -1);
        for (int i = 0; i < 5; i++)
            init_pair(CP_L1 + i, th->hi256[i], -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
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

/*
 * Body — one gravitational mass point in the N-body system.
 *
 * Lifecycle:
 *   - created by a preset (preset_galaxy / preset_cluster / preset_binary)
 *   - integrated each sim tick by scene_tick() unless anchor=true
 *   - may be deactivated if it escapes far outside the view bounds
 *   - rendered each draw frame in scene_draw()
 *
 * Why pixel-space (not cell-space) positions:
 *   The integrator advances positions smoothly between cells at sub-cell
 *   precision so orbits look continuous. px_to_cell_x/y rounds to a
 *   character cell only at render time. Storing px/py in cells would
 *   discretise every motion to integer steps — slow bodies would appear
 *   stationary for many frames, fast bodies would teleport.
 *
 * Why prev_px / prev_py:
 *   scene_tick snapshots the pre-step position into these before
 *   advancing px/py. streak_glow() then draws a Bresenham line from
 *   (prev_*, prev_*) to (px, py) into the brightness buffer, so fast
 *   bodies leave visible orbit arcs instead of flickering trails of
 *   isolated cells. Without this, a body moving 4 cells/frame would
 *   skip 3 cells between draws.
 *
 * Why both active and anchor flags (and not just one):
 *   active=false bodies are skipped entirely by both integrator and
 *   renderer; used to remove escapees (positions beyond an extended
 *   bound) cheaply without reallocating arrays.
 *   anchor=true bodies stay active and visible but skip force
 *   integration — they never move regardless of net force. Models a
 *   central black hole that is much heavier than the disk and (in our
 *   approximation) fixed at the centre. Galaxy preset uses one anchor;
 *   cluster and binary use none.
 *
 * Integration scheme — symplectic Euler:
 *   v += a · dt   (kick)
 *   x += v · dt   (drift)
 *   Symplectic schemes conserve a "shadow" Hamiltonian over long runs,
 *   so total energy stays bounded indefinitely — critical for galaxy
 *   stability over the thousands of ticks a long-running disc needs.
 *
 * Algorithm refs (header [n]):
 *   Softened gravity   F = G·mᵢ·M/(d²+ε²)^(3/2)   — Aarseth §2.4 [2]
 *   Symplectic Euler                              — Aarseth §2.2 [2]
 *   Keplerian initial conditions (galaxy preset)  — Binney&Tremaine §3.1 [3]
 *   Cold-collapse virialisation (cluster preset)  — Binney&Tremaine §8.5.4 [3]
 */
typedef struct {
    float px, py;             /* current position, pixel space             */
    float prev_px, prev_py;   /* position one sim step ago — for streak    */
    float vx, vy;              /* current velocity, pixel-units/sim-second  */
    float mass;                /* gravitational mass; 1.0 for ordinary
                                * bodies, ≫1 for the central anchor        */
    bool  active;              /* false = removed (escapee or unused slot)  */
    bool  anchor;              /* true = skip force integration entirely    */
} Body;

/* Body instances live in the §7 Scene struct (defined just below in
 * §6 for forward-declaration reasons). The deterministic RNG state
 * below is kept at file scope so the rng_*() pure utilities don't
 * need a Scene*. */

/* Simple LCG */
static uint32_t g_rng = 12345u;
static uint32_t rng_next(void)  { g_rng = g_rng * 1664525u + 1013904223u; return g_rng; }
static float    rng_f(void)     { return (float)(rng_next() & 0x7FFFFFFFu) / (float)0x80000000u; }
static float    rng_range(float lo, float hi) { return lo + rng_f() * (hi - lo); }

/* ===================================================================== */
/* §6  quadtree                                                           */
/* ===================================================================== */

/*
 * QNode — one node of the Barnes–Hut quadtree.
 *
 * Why a tree at all:
 *   Direct N-body force evaluation is O(N²): for N=400 that's 80 000
 *   pair computations per tick.  Barnes & Hut (1986) showed that a
 *   hierarchical spatial decomposition lets you approximate the
 *   gravitational influence of distant body clusters as a single point
 *   mass at their centre of mass, dropping the cost to O(N log N).
 *
 * Why quadtree specifically:
 *   We simulate in 2-D, so a quadtree (2-D space → 4 children per node)
 *   is the right shape.  3-D would use an octree with 8 children; the
 *   algorithm structure is identical.
 *
 * Node anatomy:
 *   x0, y0, x1, y1     Axis-aligned bounding box in pixel space.
 *                      Side length s = (x1 - x0) — the size used by
 *                      the BH opening criterion s/d < θ.
 *   total_mass         Σ mass of every body inside this subtree.
 *                      Updated incrementally during qt_insert so we
 *                      avoid a separate aggregation pass.
 *   cx, cy             Centre of mass of every body inside the subtree:
 *                        cx = Σ(mᵢ · pxᵢ) / Σ mᵢ
 *                        cy = Σ(mᵢ · pyᵢ) / Σ mᵢ
 *                      Also incremental — qt_insert does one weighted
 *                      average per inserted body.
 *   child[4]           Quadrant children: NW=0, NE=1, SW=2, SE=3.
 *                      -1 in any slot means "no body in that quadrant".
 *                      Leaves have all four slots = -1 (no children);
 *                      internal nodes have at least one ≥ 0.
 *   body_idx           ≥0 only for a leaf containing exactly one body
 *                      (the index into scene.bodies[]). -1 for empty leaves
 *                      and for all internal nodes. When a second body
 *                      arrives at a single-body leaf, qt_insert pushes
 *                      the resident body into a subdivided quadrant and
 *                      sets body_idx = -1.
 *   depth              Distance from the root (root = 0). Used to cap
 *                      recursion at QT_MAX_DEPTH so that two bodies at
 *                      numerically-identical positions don't subdivide
 *                      forever.
 *
 * Force evaluation — the Barnes–Hut opening criterion (see qt_force):
 *
 *     s = x1 - x0         (this node's side length)
 *     d = |p_body − COM|  (distance from query body to node COM)
 *
 *     if s / d < θ:
 *         treat node as a single point mass at (cx, cy)
 *         contribute F = G · m_body · total_mass / (d² + ε²)^(3/2)
 *     else:
 *         recurse into the four children
 *
 *   θ = 0.5 is Barnes & Hut's recommended value — small enough for
 *   ~1% RMS force error on a typical body distribution, large enough
 *   to keep the cost near O(N log N) in practice.  Smaller θ → more
 *   accurate, more expensive; θ = 0 → exact O(N²) brute force.
 *
 * Pool allocation:
 *   Every node lives in a static pool scene.pool[NODE_POOL_MAX]; qt_alloc()
 *   bumps an integer (no malloc/free in the hot path).  The whole tree
 *   is rebuilt every sim tick from scene.pool_top = 0; building from scratch
 *   is cheap and avoids the bookkeeping of incremental tree updates.
 *
 * Algorithm refs (header [n]):
 *   The original tree-code paper            — Barnes & Hut (1986) [1]
 *   Practical tree-code performance / θ     — Aarseth §6.2 [2]
 *   Incremental COM update                  — Aarseth §6.2 [2]
 */
typedef struct {
    float x0, y0, x1, y1;     /* bounding box, pixel space             */
    float total_mass;          /* Σ mass of all bodies in this subtree  */
    float cx, cy;              /* centre of mass of all bodies below    */
    int   child[4];            /* NW=0, NE=1, SW=2, SE=3; -1 = absent   */
    int   body_idx;            /* ≥0 if leaf with one body, else -1     */
    int   depth;               /* distance from root, capped at QT_MAX  */
} QNode;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — all state that persists between scene_tick() and scene_draw().
 *
 * Defined here (rather than in §7 with the scene_* functions) because §6
 * quadtree functions below reference scene.pool / scene.pool_top, so the
 * Scene type and instance must exist before they're used.  All scene_*
 * functions and presets that follow in §7 use this same `scene` global.
 *
 * Fields are split into two clearly-labelled groups:
 *
 *   Simulation params — advanced by scene_tick(); mutated only by
 *     physics-affecting input (pause / preset / reset / +/- bodies /
 *     g/G gravity / f fast-forward).
 *
 *   Rendering params  — read by scene_draw() and the HUD; do NOT alter
 *     physics in any way.  A theme change or overlay toggle drawn
 *     against the same Scene must produce identical body positions,
 *     velocities, and tree structure.
 *
 * Locality rationale:
 *   The split between sim and render fields exists for the READER, not
 *   the CPU.  A new sim flag accidentally placed in the rendering block
 *   would silently couple physics to display state — exactly the bug
 *   the separation prevents.  Future changes know which side to land on.
 *
 * One global instance (`static Scene scene`):
 *   The struct is large (Bodies + QNode pool + brightness grid ≈ 760 KB)
 *   so it lives in .bss as a single file-static, accessed as
 *   `scene.<field>` throughout.
 *
 * State kept OUTSIDE this struct (intentionally):
 *   g_rng     deterministic RNG state used by rng_*().  Kept separate
 *             so the pure-utility rng helpers don't need a Scene*.
 *   g_resize  signal-handler flag; must be a file-static volatile
 *             sig_atomic_t for async-signal safety.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Simulation parameters ─────────────────────────────────── */
    Body  bodies[N_BODIES_MAX];   /* all body slots (active + inactive) */
    int   n_bodies;               /* current body count (+/- adjustable) */
    float G;                       /* gravitational constant (g/G keys)  */
    bool  paused;                  /* if true, scene_tick is a no-op     */
    int   preset;                  /* index into preset_* dispatch       */
    bool  fastfwd;                 /* 4× sim time per render frame       */
    float sim_dt;                  /* fixed-timestep (sim seconds)       */

    /* Quadtree scratch — rebuilt from pool_top=0 every sim tick. */
    QNode pool[NODE_POOL_MAX];     /* pre-allocated node pool            */
    int   pool_top;                /* bump pointer into pool[]           */
    int   qt_root;                 /* root index into pool[]             */

    /* ── Rendering parameters ──────────────────────────────────── */
    float bright[GRID_ROWS_MAX][GRID_COLS_MAX];   /* additive glow buffer
                                                   * (streaks + body
                                                   * deposits + halo;
                                                   * decays 0.93/frame) */
    float bright_max;              /* EMA-smoothed max for colour norm   */
    float v_max;                   /* rolling max speed for colour bands;
                                    * tracked from physics, used only by
                                    * the renderer for colour mapping    */
    int   theme;                   /* index into k_themes[] (§3)         */
    bool  overlay;                 /* draw quadtree on top               */
    int   frame_tick;              /* monotonic counter; drives BH pulse
                                    * and accretion-halo phase           */
} Scene;

static Scene scene = {
    .n_bodies   = N_BODIES_DEF,
    .G          = G_DEF,
    .sim_dt     = 1.0f / (float)SIM_HZ,
    .bright_max = 1.0f,
    .v_max      = 1.0f,
};

static int qt_alloc(float x0, float y0, float x1, float y1, int depth)
{
    if (scene.pool_top >= NODE_POOL_MAX) return -1;
    int idx = scene.pool_top++;
    QNode *n = &scene.pool[idx];
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
    QNode *n = &scene.pool[ni];
    Body  *b = &scene.bodies[bi];

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
        int q = qt_quadrant(n, scene.bodies[existing].px, scene.bodies[existing].py);
        qt_insert(n->child[q], existing);
    } else if (n->child[0] < 0) {
        qt_subdivide(n);
    }
    int q = qt_quadrant(n, b->px, b->py);
    qt_insert(n->child[q], bi);
}

static int qt_build(int cols, int rows)
{
    scene.pool_top = 0;
    int root = qt_alloc(0.0f, 0.0f, (float)pw(cols), (float)ph(rows), 0);
    for (int i = 0; i < scene.n_bodies; i++)
        if (scene.bodies[i].active) qt_insert(root, i);
    return root;
}

static void qt_force(int ni, int bi, float *fx, float *fy)
{
    if (ni < 0) return;
    QNode *n = &scene.pool[ni];
    if (n->total_mass == 0.0f) return;
    if (n->body_idx == bi) return;   /* skip self at leaf */

    Body  *b  = &scene.bodies[bi];
    float  dx = n->cx - b->px;
    float  dy = n->cy - b->py;
    float  d2 = dx*dx + dy*dy + SOFT2;
    float  d  = sqrtf(d2);
    float  s  = n->x1 - n->x0;

    if ((s / d) < THETA_DEF || n->child[0] < 0) {
        float inv = scene.G * n->total_mass / (d2 * d);
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
    QNode *n = &scene.pool[ni];
    if (n->total_mass == 0.0f || n->depth > 3) return;

    int cx0 = px_to_cell_x(n->x0), cx1 = px_to_cell_x(n->x1);
    int cy0 = px_to_cell_y(n->y0), cy1 = px_to_cell_y(n->y1);
    int mxc = px_to_cell_x((n->x0 + n->x1) * 0.5f);
    int myc = px_to_cell_y((n->y0 + n->y1) * 0.5f);

    if ((cx1 - cx0) < 2 || (cy1 - cy0) < 2) return;

    attron(COLOR_PAIR(CP_TREE) | A_DIM);
    for (int c = cx0+1; c < cx1 && c < cols; c++)
        if (myc >= 0 && myc < rows) mvaddch(myc, c, '-');
    for (int r = cy0+1; r < cy1 && r < rows; r++)
        if (mxc >= 0 && mxc < cols) mvaddch(r, mxc, '|');
    if (myc >= 0 && myc < rows && mxc >= 0 && mxc < cols)
        mvaddch(myc, mxc, '+');
    attroff(COLOR_PAIR(CP_TREE) | A_DIM);

    for (int c = 0; c < 4; c++)
        qt_draw_overlay(n->child[c], rows, cols);
}

/* ===================================================================== */
/* §7  scene — state that spans across ticks and frames                   */
/* ===================================================================== */

/* The Scene struct is defined in §6 (just after QNode) because the
 * quadtree functions above reference scene.pool.  The presets and the
 * scene_* functions below all operate on the same `scene` instance. */

/* ── Presets ─────────────────────────────────────────────────────────── */

/*
 * Preset 1 — Galaxy
 *
 * Body 0 = central black hole (anchor, mass = N × BH_MASS_FACTOR).
 * Remaining bodies on Keplerian circular orbits: v = √(G · M_bh / r).
 * Differential rotation (inner orbits faster than outer) shears the
 * disk into spontaneous spiral structure over a few orbital periods.
 *
 * Pseudocode:
 *   compute disk geometry (centre, radius, M_bh)
 *   spawn the central black hole at body 0
 *   for each remaining body i:
 *       spawn one Keplerian orbiter at a uniform-area random radius
 */

/* Place a stationary anchor body of mass M at (cx, cy). Anchor means
 * the integrator will skip it — see Body.anchor field. */
static void spawn_central_bh(int idx, float cx, float cy, float M)
{
    scene.bodies[idx].px     = cx;
    scene.bodies[idx].py     = cy;
    scene.bodies[idx].vx     = 0.0f;
    scene.bodies[idx].vy     = 0.0f;
    scene.bodies[idx].mass   = M;
    scene.bodies[idx].active = true;
    scene.bodies[idx].anchor = true;
}

/*
 * sample_disk_radius_uniform_area — sample a radius r in [r_min·R, R]
 * such that the resulting points distribute UNIFORMLY by area (not by
 * radius). Picking r linearly would concentrate bodies near the centre
 * because the circle area at radius r grows as 2πr — equal-radius
 * shells have unequal area. The √(u) inverse-CDF gives equal area
 * per body, so the rendered disk looks uniformly dense.
 *
 * Mathematically:
 *   PDF over r ∝ r   (area weight)
 *   CDF: F(r) = (r/R)²
 *   Inverse: r = R · √u  where u ~ U(0,1)
 * We additionally clamp away from the very centre (0.08·R) so bodies
 * don't spawn inside the BH.
 */
static float sample_disk_radius_uniform_area(float R)
{
    return R * (0.08f + 0.92f * sqrtf(rng_f()));
}

/*
 * spawn_keplerian_body — place body idx at a random point on a disk of
 * radius R around (cx, cy), with a velocity that produces a circular
 * orbit around a central point mass M_bh.
 *
 *   v_kep = √(G · M_bh / r)         (Keplerian circular velocity)
 *   direction: tangent to the radial vector, CCW
 *   small ±6% scatter on |v| → slightly elliptical orbits so the disk
 *   looks dynamically rich rather than perfectly regular.
 *
 * Algorithm refs (header [n]):
 *   Keplerian initial conditions    — Binney & Tremaine §3.1 [3]
 *   Inverse-CDF sampling             — standard MC technique
 */
static void spawn_keplerian_body(int idx, float cx, float cy, float R, float M_bh)
{
    float r     = sample_disk_radius_uniform_area(R);
    float theta = rng_f() * 2.0f * (float)M_PI;

    float bx = cx + cosf(theta) * r;
    float by = cy + sinf(theta) * r;

    float v_kep = sqrtf(scene.G * M_bh / r);
    float v     = v_kep * (1.0f + rng_range(-0.06f, 0.06f));

    /* Unit tangent: perpendicular to radius, CCW (cross product with +z) */
    float tx = -(by - cy) / r;
    float ty =  (bx - cx) / r;

    scene.bodies[idx].px     = bx;
    scene.bodies[idx].py     = by;
    scene.bodies[idx].vx     = tx * v;
    scene.bodies[idx].vy     = ty * v;
    scene.bodies[idx].mass   = 1.0f;
    scene.bodies[idx].active = true;
    scene.bodies[idx].anchor = false;
}

static void preset_galaxy(int cols, int rows)
{
    /* 1. Disk geometry: centred on screen, radius fits the shorter axis. */
    float cx   = (float)pw(cols) * 0.5f;
    float cy   = (float)ph(rows) * 0.5f;
    float half = (float)pw(cols);
    if (ph(rows) < pw(cols)) half = (float)ph(rows);
    half *= 0.5f;
    float R    = half * 0.75f;

    /* 2. Central black hole mass scales with body count so the
     *    orbital periods stay roughly constant across +/- presses. */
    float M_bh = (float)scene.n_bodies * BH_MASS_FACTOR;

    /* 3. Spawn the anchor at body 0. */
    spawn_central_bh(0, cx, cy, M_bh);

    /* 4. Spawn N-1 disk orbiters with Keplerian velocities. */
    for (int i = 1; i < scene.n_bodies; i++)
        spawn_keplerian_body(i, cx, cy, R, M_bh);
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
    float M_tot = (float)scene.n_bodies;
    float v_spin = sqrtf(scene.G * M_tot / R) * 0.12f;   /* 12% of virial speed */

    for (int i = 0; i < scene.n_bodies; i++) {
        float r     = R * sqrtf(rng_f());
        float theta = rng_f() * 2.0f * (float)M_PI;
        float bx    = cx + cosf(theta) * r;
        float by    = cy + sinf(theta) * r;

        /* Tangential spin */
        float nx = -(by - cy) / (r + 1.0f);
        float ny =  (bx - cx) / (r + 1.0f);

        scene.bodies[i].px     = bx;
        scene.bodies[i].py     = by;
        scene.bodies[i].vx     = nx * v_spin;
        scene.bodies[i].vy     = ny * v_spin;
        scene.bodies[i].mass   = 1.0f;
        scene.bodies[i].active = true;
        scene.bodies[i].anchor = false;
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

    int half = scene.n_bodies / 2;

    /* Approach velocity: clusters meet in ~5 seconds at default SIM_HZ */
    float approach = sep / (5.0f * (float)SIM_HZ);

    /* Internal spin speed */
    float M_half  = (float)half;
    float v_spin  = sqrtf(scene.G * M_half / R) * 0.35f;

    for (int k = 0; k < 2; k++) {
        float ox = cx + (k == 0 ? -sep : sep);
        float oy = cy;
        float vx_drift = (k == 0 ? approach : -approach);
        float spin_dir = (k == 0 ? 1.0f : -1.0f);   /* counter-rotate */

        int start = k * half;
        int end   = (k == 1) ? scene.n_bodies : half;

        for (int i = start; i < end; i++) {
            float r     = R * sqrtf(rng_f());
            float theta = rng_f() * 2.0f * (float)M_PI;
            float bx    = ox + cosf(theta) * r;
            float by    = oy + sinf(theta) * r;
            float nx    = -(by - oy) / (r + 1.0f);
            float ny    =  (bx - ox) / (r + 1.0f);

            scene.bodies[i].px     = bx;
            scene.bodies[i].py     = by;
            scene.bodies[i].vx     = vx_drift + spin_dir * nx * v_spin;
            scene.bodies[i].vy     = spin_dir * ny * v_spin;
            scene.bodies[i].mass   = 1.0f;
            scene.bodies[i].active = true;
            scene.bodies[i].anchor = false;
        }
    }
}

static void scene_reset(int cols, int rows)
{
    memset(scene.bright, 0, sizeof(scene.bright));
    memset(scene.bodies, 0, sizeof(Body) * N_BODIES_MAX);
    scene.v_max = 1.0f;
    g_rng   = 99991u;

    switch (scene.preset) {
        case 0: preset_galaxy (cols, rows); break;
        case 1: preset_cluster(cols, rows); break;
        case 2: preset_binary (cols, rows); break;
    }

    /* Initialise prev position to current so the first frame draws a
     * single point per body, not a streak from (0,0). */
    for (int i = 0; i < scene.n_bodies; i++) {
        scene.bodies[i].prev_px = scene.bodies[i].px;
        scene.bodies[i].prev_py = scene.bodies[i].py;
    }
}

/* ── scene_tick ──────────────────────────────────────────────────────── */

/*
 * rebuild_force_tree — fresh O(N log N) Barnes–Hut tree over the active
 * bodies. The tree is thrown away and rebuilt every sim step (no
 * incremental updates) because that is cheaper than maintaining
 * positions inside a long-lived tree.
 */
static void rebuild_force_tree(int cols, int rows)
{
    scene.qt_root = qt_build(cols, rows);
}

/*
 * mark_inactive_if_escaped — bodies that have wandered far beyond the
 * visible region cannot meaningfully participate in the scene any more.
 * Setting active=false makes the integrator and renderer skip them on
 * subsequent ticks, which keeps the per-frame cost bounded.
 *
 * The slack zone (2× width horizontally, 3× height vertically) gives
 * room for hyperbolic trajectories to swing past the edge and come
 * back before we deactivate them.
 */
static void mark_inactive_if_escaped(Body *b, float W, float H)
{
    if (b->px < -W       || b->px > 2.0f * W) b->active = false;
    if (b->py < -H * 2.0f || b->py > 3.0f * H) b->active = false;
}

/*
 * integrate_body_symplectic_euler — one sim step for body i.
 *
 *   v += (F / m) · dt     (kick)
 *   x += v          · dt  (drift)
 *
 * Symplectic Euler conserves a "shadow" Hamiltonian over long runs so
 * orbital energy stays bounded — important for galaxy stability over
 * the thousands of ticks a long-running disc accumulates.  Force is
 * computed via the Barnes–Hut criterion in qt_force (§6).
 *
 * Side effects per call:
 *   - snapshot pre-step position for the renderer's streak pass
 *   - mark body inactive if it has escaped the slack zone
 *   - update scene.v_max for colour-band normalisation
 */
static void integrate_body_symplectic_euler(int i, float W, float H)
{
    Body *b = &scene.bodies[i];
    if (!b->active || b->anchor) return;

    /* Snapshot pre-step position for streak rendering. */
    b->prev_px = b->px;
    b->prev_py = b->py;

    /* Force from the Barnes–Hut tree (excludes self at leaf). */
    float fx = 0.0f, fy = 0.0f;
    qt_force(scene.qt_root, i, &fx, &fy);

    /* Kick — accelerate from current force. */
    float ax = fx / b->mass;
    float ay = fy / b->mass;
    b->vx += ax * scene.sim_dt;
    b->vy += ay * scene.sim_dt;

    /* Drift — advance position using the new velocity. */
    b->px += b->vx * scene.sim_dt;
    b->py += b->vy * scene.sim_dt;

    mark_inactive_if_escaped(b, W, H);

    /* Update rolling max speed for the renderer's colour bands. */
    float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (spd > scene.v_max) scene.v_max = spd;
}

/*
 * relax_speed_normalisation — slowly decay the rolling max speed so the
 * colour scale tracks the CURRENT dynamics rather than the fastest body
 * the simulation has ever produced (which would never come back down
 * after a single hot transient).  Decay factor and floor are tuned so
 * a galaxy disk settles within a few seconds.
 */
static void relax_speed_normalisation(void)
{
    scene.v_max *= 0.9995f;
    if (scene.v_max < 0.1f) scene.v_max = 0.1f;
}

/* Top-level: one fixed-timestep simulation step. */
static void scene_tick(int cols, int rows)
{
    /* 1. Rebuild the Barnes–Hut force tree for this step. */
    rebuild_force_tree(cols, rows);

    /* 2. Advance every active, non-anchor body one symplectic Euler step. */
    float W = (float)pw(cols);
    float H = (float)ph(rows);
    for (int i = 0; i < scene.n_bodies; i++)
        integrate_body_symplectic_euler(i, W, H);

    /* 3. Adapt the colour normalisation toward current dynamics. */
    relax_speed_normalisation();
}

/* ── scene_draw ──────────────────────────────────────────────────────── */

/*
 * streak_glow — additive Bresenham line from (x0,y0) to (x1,y1) in
 * pixel space, depositing `intensity` units of brightness into each
 * cell the line passes through. Skips the endpoint so the body's own
 * per-frame deposit isn't doubled. Hard cap on iterations prevents
 * runaway streaks if a body's position jumped wildly (e.g. resize).
 */
static void streak_glow(float x0, float y0, float x1, float y1,
                        float intensity, int rows, int cols)
{
    int r0 = px_to_cell_y(y0), c0 = px_to_cell_x(x0);
    int r1 = px_to_cell_y(y1), c1 = px_to_cell_x(x1);
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = -1; if (r0 < r1) sr = 1;
    int sc = -1; if (c0 < c1) sc = 1;
    int err = dr - dc;
    for (int i = 0; i < 32; i++) {
        if (r0 == r1 && c0 == c1) break;   /* endpoint = body draws there */
        if (r0 >= 0 && r0 < rows && c0 >= 0 && c0 < cols)
            scene.bright[r0][c0] += intensity;
        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r0 += sr; }
        if (e2 <  dr) { err += dr; c0 += sc; }
    }
}

/*
 * deposit_accretion_halo — bleed extra brightness into the cells around
 * the anchor, aspect-corrected so the halo appears circular on the
 * terminal grid (each row is ~2× as tall as wide, so column offsets
 * count quarter-weight in the distance norm: d² = dr² + (dc/2)²).
 *
 * Intensity falls as 1 / (1 + d²) so the central cell is brightest and
 * the halo fades by the edge. The pulse argument modulates total
 * brightness with the BH's sinusoidal phase.
 */
static void deposit_accretion_halo(int cr, int cc, int rows, int cols, float pulse)
{
    for (int dr = -2; dr <= 2; dr++) {
        for (int dc = -4; dc <= 4; dc++) {
            if (dr == 0 && dc == 0) continue;
            int rr = cr + dr;
            int ccc = cc + dc;
            if (rr < 0 || rr >= rows || ccc < 0 || ccc >= cols) continue;
            float d2 = (float)(dr * dr) + (float)(dc * dc) * 0.25f;
            scene.bright[rr][ccc] += 3.0f * pulse / (1.0f + d2);
        }
    }
}

/*
 * accumulate_glow_field — for each active body, deposit brightness at
 * its current cell plus a half-strength streak along its motion arc
 * from prev → current (Bresenham, via streak_glow). The anchor instead
 * gets an aspect-corrected accretion halo so it visibly pulls light.
 *
 * After this pass, scene.bright[][] holds the additive glow snapshot
 * for the frame and is ready for normalisation + painting.
 */
static void accumulate_glow_field(int cols, int rows)
{
    float halo_pulse = 1.0f + 0.4f * sinf((float)scene.frame_tick * 0.18f);

    for (int i = 0; i < scene.n_bodies; i++) {
        Body *b = &scene.bodies[i];
        if (!b->active) continue;

        int cr = px_to_cell_y(b->py);
        int cc = px_to_cell_x(b->px);
        float deposit = 1.0f;
        if (b->mass > 1.0f) deposit = 4.0f;   /* anchor brighter */

        if (cr >= 0 && cr < rows && cc >= 0 && cc < cols)
            scene.bright[cr][cc] += deposit;

        if (b->anchor) {
            deposit_accretion_halo(cr, cc, rows, cols, halo_pulse);
        } else {
            streak_glow(b->prev_px, b->prev_py, b->px, b->py,
                        deposit * 0.5f, rows, cols);
        }
    }
}

/*
 * update_brightness_norm — EMA-smooth the max of scene.bright[][] used
 * for colour normalisation. Plain frame-by-frame max would let a single
 * bright transient (close pass, flyby) rescale the whole field for ~30
 * frames as the decay slowly chews it down. 85/15 mix adapts over ~7
 * frames — fast enough to track real changes, slow enough to ignore
 * spikes.
 */
static void update_brightness_norm(int cols, int rows)
{
    float frame_max = 1.0f;
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            if (scene.bright[r][c] > frame_max) frame_max = scene.bright[r][c];
    scene.bright_max = scene.bright_max * 0.85f + frame_max * 0.15f;
    if (scene.bright_max < 1.0f) scene.bright_max = 1.0f;
}

/*
 * paint_glow_layer — render the brightness buffer to the screen, then
 * apply the per-frame decay multiplier. Density ramp '.' → '@'; top
 * two tiers get A_BOLD so hot cells punch through.
 */
static void paint_glow_layer(int cols, int rows)
{
    static const char k_glow[] = ".:*o@";
    float b_max = scene.bright_max;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float b = scene.bright[r][c];
            scene.bright[r][c] *= DECAY;
            if (b < 0.4f) continue;

            float norm = b / b_max;
            int   lvl  = (int)(norm * 4.9f);
            if (lvl > 4) lvl = 4;

            attr_t attr = COLOR_PAIR(CP_L1 + lvl);
            if (lvl >= 3) attr |= A_BOLD;
            mvaddch(r, c, (chtype)k_glow[lvl] | attr);
        }
    }
}

/*
 * paint_anchor_pulse — animated black-hole glyph. Phase from the same
 * frame_tick × 0.18 sin() used for the halo, so the core compresses
 * ('@') and brackets brighten in time with the halo's expansion, then
 * relaxes ('*') with dimmed brackets.
 */
static void paint_anchor_pulse(int cr, int cc, int cols)
{
    float ph = sinf((float)scene.frame_tick * 0.18f);

    chtype core_attr = COLOR_PAIR(CP_BH) | A_BOLD;
    chtype edge_attr = COLOR_PAIR(CP_BH);
    if      (ph >  0.6f) edge_attr |= A_BOLD;
    else if (ph < -0.4f) edge_attr |= A_DIM;

    char core = '*';
    if (ph > 0.0f) core = '@';

    mvaddch(cr, cc, (chtype)core | core_attr);
    if (cc > 0)        mvaddch(cr, cc - 1, (chtype)'(' | edge_attr);
    if (cc < cols - 1) mvaddch(cr, cc + 1, (chtype)')' | edge_attr);
}

/*
 * paint_field_body — non-anchor body. Speed-band colour with a two-
 * variant glyph picked by (i & 1) so a swarm of bodies at the same
 * speed reads as a textured cloud, not a stencil pattern.
 */
static void paint_field_body(int i, const Body *b, int cr, int cc)
{
    float spd  = sqrtf(b->vx * b->vx + b->vy * b->vy);
    float norm = spd / scene.v_max;

    int    pair;
    chtype ch;
    int    variant = i & 1;
    if (norm > 0.80f) {
        pair = CP_L5; ch = '*'; if (variant) ch = '#';
    } else if (norm > 0.55f) {
        pair = CP_L4; ch = '+'; if (variant) ch = 'x';
    } else if (norm > 0.30f) {
        pair = CP_L3; ch = 'o'; if (variant) ch = '0';
    } else if (norm > 0.10f) {
        pair = CP_L2; ch = '.'; if (variant) ch = ',';
    } else {
        pair = CP_L1; ch = ','; if (variant) ch = '`';
    }
    mvaddch(cr, cc, ch | COLOR_PAIR(pair) | A_BOLD);
}

/*
 * paint_bodies — render every active body's glyph on top of the glow
 * layer. The anchor uses its own pulsing renderer; everything else
 * uses the speed-banded body renderer.
 */
static void paint_bodies(int cols, int rows)
{
    for (int i = 0; i < scene.n_bodies; i++) {
        const Body *b = &scene.bodies[i];
        if (!b->active) continue;

        int cr = px_to_cell_y(b->py);
        int cc = px_to_cell_x(b->px);
        if (cr < 1 || cr >= rows - 1 || cc < 0 || cc >= cols) continue;

        if (b->anchor) paint_anchor_pulse(cr, cc, cols);
        else           paint_field_body(i, b, cr, cc);
    }
}

/* Top-level: one frame as five named passes. */
static void scene_draw(int cols, int rows, float alpha)
{
    (void)alpha;
    scene.frame_tick++;

    /* 1. Deposit body + streak + accretion-halo brightness for this frame. */
    accumulate_glow_field(cols, rows);

    /* 2. EMA-smooth the brightness normalisation against transients. */
    update_brightness_norm(cols, rows);

    /* 3. Paint the glow buffer (with per-frame decay multiplier). */
    paint_glow_layer(cols, rows);

    /* 4. Paint body glyphs on top of the glow (anchor pulses, others
     *    are speed-band coloured with two-variant glyphs). */
    paint_bodies(cols, rows);

    /* 5. Optional quadtree overlay for the Barnes–Hut decomposition. */
    if (scene.overlay)
        qt_draw_overlay(scene.qt_root, rows, cols);
}

/* ===================================================================== */
/* §8  screen / HUD                                                       */
/* ===================================================================== */

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0 right    — bright yellow + bold: live status (N, G, theme, fps, ...)
 *   row 1 left     — bright yellow:        preset name + description
 *   row rows-1     — bright cyan + bold:   key hints
 *
 * Paused indicator is folded into the top status string so it sits with
 * the other live state (rather than a separate inline tag).
 */
static void hud_draw(int cols, int rows, double fps)
{
    /* Count active (non-anchor) bodies. */
    int active = 0;
    for (int i = 0; i < scene.n_bodies; i++)
        if (scene.bodies[i].active && !scene.bodies[i].anchor) active++;

    /* Row 0 — right-aligned live status. */
    char top[160];
    const char *state = "running";
    if (scene.paused)       state = "PAUSED ";
    else if (scene.fastfwd) state = "4x     ";
    snprintf(top, sizeof top,
             " Barnes-Hut  N=%d(%d)  G=%.0f  theme:%s  %s  %.0f fps ",
             scene.n_bodies, active, scene.G, k_themes[scene.theme].name, state, fps);
    int top_len = (int)strlen(top);
    int top_col = cols - top_len;
    if (top_col < 0) top_col = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, top_col, top, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Row 1 — left-aligned preset name and tagline. */
    static const char *k_desc[N_PRESETS] = {
        "Galaxy: BH disk — watch spiral arms form",
        "Cluster: cold collapse — core bounce incoming",
        "Binary merger — tidal streams + chaotic ejections",
    };
    char sub[120];
    snprintf(sub, sizeof sub, " [%d] %s ", scene.preset + 1, k_desc[scene.preset]);
    attron(COLOR_PAIR(CP_HUD));
    mvaddnstr(1, 0, sub, cols);
    attroff(COLOR_PAIR(CP_HUD));

    /* Row rows-1 — bottom hint bar. */
    const char *hint_full =
        " q:quit  p:pause  r:reset  1-3:preset  t/T:theme  o:overlay  f:4x  +/-:bodies  g/G:grav ";
    const char *hint_short =
        " q:quit  p:pause  r:reset  1-3:preset  f:4x  t:theme ";
    const char *hint = hint_full;
    if ((int)strlen(hint_full) >= cols - 1) hint = hint_short;
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(rows - 1, 0, hint, cols);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
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

    color_init(scene.theme);

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
            color_init(scene.theme);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:
                    endwin(); return 0;

                case 'p': case ' ':
                    scene.paused = !scene.paused;
                    break;

                case 'r':
                    scene_reset(cols, rows);
                    break;

                case '1': case '2': case '3':
                    scene.preset = ch - '1';
                    scene_reset(cols, rows);
                    break;

                case 't':
                    scene.theme = (scene.theme + 1) % N_THEMES;
                    color_init(scene.theme);
                    break;

                case 'T':
                    scene.theme = (scene.theme + N_THEMES - 1) % N_THEMES;
                    color_init(scene.theme);
                    break;

                case 'o':
                    scene.overlay = !scene.overlay;
                    break;

                case 'f':
                    scene.fastfwd = !scene.fastfwd;
                    break;

                case '+': case '=':
                    scene.n_bodies += N_BODIES_STEP;
                    if (scene.n_bodies > N_BODIES_MAX) scene.n_bodies = N_BODIES_MAX;
                    scene_reset(cols, rows);
                    break;

                case '-':
                    scene.n_bodies -= N_BODIES_STEP;
                    if (scene.n_bodies < N_BODIES_STEP) scene.n_bodies = N_BODIES_STEP;
                    scene_reset(cols, rows);
                    break;

                case 'g':
                    scene.G += G_STEP;
                    if (scene.G > G_MAX) scene.G = G_MAX;
                    break;

                case 'G':
                    scene.G -= G_STEP;
                    if (scene.G < G_MIN) scene.G = G_MIN;
                    break;
            }
        }

        /* Physics accumulator */
        int64_t now = clock_ns();
        sim_acc += now - t_last_sim;
        t_last_sim = now;

        if (!scene.paused) {
            /* Fast-forward: allow 4× the normal budget */
            int64_t cap = scene.fastfwd ? sim_ns * 16 : sim_ns * 4;
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
