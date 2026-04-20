/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_systems/particles.c — 2-D Terminal Particle Simulation Engine
 *
 * Reusable foundation for fluid, smoke, fire, gravity, sand, and explosion
 * effects.  Every subsystem lives in its own numbered section and is written
 * to be extracted into a separate compilation unit as the project grows.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *  §1  config    — all tunable constants in one place
 *  §2  clock     — monotonic nanosecond clock + sleep
 *  §3  theme     — color palettes, ASCII ramps, visual LUT pipeline
 *  §4  structs   — Particle, Emitter data structures
 *  §5  pool      — static particle pool, O(1)-amortised slot allocator
 *  §6  forces    — gravity, drag; force-accumulation pattern
 *  §7  physics   — update_particles(dt), handle_collisions()
 *  §8  emitter   — continuous emission and burst spawning
 *  §9  render    — four render modes + overlay stats panel
 *  §10 presets   — fireworks, fountain, rainfall, explosion
 *  §11 scene     — top-level simulation state; tick + draw
 *  §12 screen    — ncurses double-buffer display layer
 *  §13 app       — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q / ESC    quit                 b     spawn burst
 *   space      pause / resume       e     toggle emitter
 *   g / G      gravity ±            d     toggle drag
 *   r          reset simulation     p/P   cycle presets
 *   t          cycle themes         v     cycle render modes
 *   ] / [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/particles.c \
 *       -o particles -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Particle integration (§7):
 *   Symplectic (semi-implicit) Euler integration is used:
 *     vel += accel * dt        ← velocity updated FIRST
 *     pos += vel   * dt        ← position updated with NEW velocity
 *   Unlike explicit Euler (pos updated with old vel), symplectic Euler is
 *   energy-conserving for harmonic oscillators and stays bounded for
 *   gravity wells.  For visual particle effects the difference is subtle,
 *   but the habit is important for any physics-accurate simulation.
 *
 * Force accumulation (§6):
 *   At the start of each tick, acceleration is cleared to zero.
 *   Each force function then ADDS its contribution to accel.
 *   This keeps forces completely modular: commenting out one function
 *   disables that force without touching anything else.
 *   F = ma → a = F/m.  Here 'density' plays the role of mass-per-volume.
 *   A denser particle (density > 1) responds less to the same force.
 *
 * Velocity update — drag (§7):
 *   Drag is NOT added as an acceleration; instead velocity is scaled:
 *     vx *= 1 - drag_coeff * density * dt
 *   This is the analytical solution of dv/dt = -k*v over interval dt.
 *   It never overshoots zero (no oscillation at large timesteps), whereas
 *   adding drag as an acceleration a = -k*v can flip sign if k*dt > 2.
 *
 * Boundary handling (§7):
 *   Three policies: BOUNCE (reflect + energy damping), KILL (particle dies),
 *   WRAP (teleport to opposite wall).  Each wall can have its own policy.
 *   Bounce damping < 1.0 removes energy on impact (inelastic collision).
 *   Kill policy is used for rain (drops vanish at floor) and sparks.
 *
 * Visual encoding (§9):
 *   Speed   → char density:  fast ↔ dense glyph ('O','@'), slow ↔ sparse ('.')
 *   Life    → brightness:    A_DIM when lifetime fraction < 0.3
 *   Angle   → arrow glyph:   8-way '>\/^<\/v/' encodes flow direction
 *   Trail   → spatial path:  ring buffer of past cell positions shows trajectory
 *   Heat    → density grid:  Gaussian splat accumulates per-cell brightness
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#define _USE_MATH_DEFINES

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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here.  Changing behaviour means editing this
 * block only — never scatter literals through implementation code.
 */

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    TARGET_FPS       =  60,   /* render cap                               */
    FPS_UPDATE_MS    = 500,   /* recalculate displayed fps every 500 ms   */
    HUD_COLS         =  72,   /* max width of any HUD string              */
};

/* ── particle pool ──────────────────────────────────────────────────── */
enum {
    MAX_PARTICLES    = 2048,  /* hard pool ceiling — never reallocated    */
    TRAIL_LEN        =    6,  /* positions kept in each particle's trail  */
    MAX_BURST        =  300,  /* hard cap: particles per single burst     */
};

/* ── physics ────────────────────────────────────────────────────────── */
/*
 * All physics values are in PIXEL SPACE (see §4 / CELL_W, CELL_H).
 * Typical terminal is ~200 px wide × 800 px tall in pixel space.
 */
#define GRAVITY_DEFAULT    200.0f   /* px / s²  downward                 */
#define GRAVITY_STEP        30.0f   /* change per 'g'/'G' keypress        */
#define GRAVITY_MIN          0.0f
#define GRAVITY_MAX        600.0f
#define DRAG_COEFF          0.80f   /* velocity fraction removed per s    */
#define BOUNCE_DAMPING      0.45f   /* kinetic energy fraction kept per bounce */
#define MAX_SPEED_NORM     650.0f   /* px/s mapped to highest ramp level  */

/* ── density grid (heatmap mode) ────────────────────────────────────── */
/*
 * Static grid sized for the largest terminal we expect to support.
 * 300 × 100 × 4 bytes = 120 KB in BSS — no heap needed.
 */
#define GRID_MAX_W   300
#define GRID_MAX_H   100

/* ── coordinate system ──────────────────────────────────────────────── */
/*
 * Terminal cells are ~2× taller than wide in physical pixels.
 * Physics runs in PIXEL SPACE; rendering converts to CELL SPACE once.
 * See §12 for the coordinate system explanation from framework.c.
 */
#define CELL_W   8    /* logical sub-pixel steps per column */
#define CELL_H  16    /* logical sub-pixel steps per row    */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }
static inline int px_to_cx(float px) { return (int)floorf(px/(float)CELL_W+0.5f); }
static inline int px_to_cy(float py) { return (int)floorf(py/(float)CELL_H+0.5f); }

/* ── render modes ───────────────────────────────────────────────────── */
enum {
    RENDER_GLYPH   = 0,  /* char density ∝ speed, brightness ∝ life  */
    RENDER_TRAIL   = 1,  /* glyph + dim spatial history               */
    RENDER_HEATMAP = 2,  /* Gaussian-splatted density grid + LUT      */
    RENDER_ARROW   = 3,  /* 8-way direction glyph per velocity angle  */
    RENDER_COUNT   = 4,
};

static const char *const k_render_names[RENDER_COUNT] = {
    "glyph", "trail", "heatmap", "arrow"
};

/* ── presets ────────────────────────────────────────────────────────── */
enum {
    PRESET_FOUNTAIN   = 0,
    PRESET_FIREWORKS  = 1,
    PRESET_RAINFALL   = 2,
    PRESET_EXPLOSION  = 3,
    PRESET_COUNT      = 4,
};

static const char *const k_preset_names[PRESET_COUNT] = {
    "fountain", "fireworks", "rainfall", "explosion"
};

/* ── timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
    struct timespec r = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme — color palettes, ASCII ramps, LUT pipeline                 */
/* ===================================================================== */

/*
 * ASCII ramp — characters ordered by visual ink density (sparse → dense).
 * These are the 9 display levels the rendering pipeline quantises to.
 * Chosen to match ncurses character-cell perception on dark terminals.
 *
 *   ' '  0%   cold / empty
 *   '.'  8%   trace / faint
 *   ':'  18%  low intensity
 *   '+'  29%  medium-low
 *   'x'  39%  medium
 *   '*'  50%  medium-high
 *   'X'  62%  high
 *   '#'  75%  very high
 *   '@'  90%  peak / core
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N  (int)(sizeof k_ramp - 1)   /* 9 levels */

/*
 * LUT breakpoints — gamma-corrected thresholds for each ramp level.
 * Gamma correction: raw_value → pow(v, 1/2.2) before lookup.
 * This maps physics-linear values to perceptually-uniform brightness
 * so mid-intensity particles aren't all the same dim character.
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f,
    0.500f, 0.620f, 0.750f, 0.900f,
};

static int lut_index(float v)
{
    /* Clamp, gamma-correct, then binary-scan for the highest level ≤ v */
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return RAMP_N - 1;
    float g = powf(v, 1.0f / 2.2f);
    int idx = 0;
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (g >= k_lut_breaks[i]) { idx = i; break; }
    return idx;
}

/*
 * Themes — four palettes, each mapping RAMP_N levels to colors.
 * We define all four simultaneously so switching themes at runtime
 * costs nothing (no init_pair calls, just change the color-pair base).
 *
 *   theme 0  fire    — red / orange / yellow   (fireworks, explosion)
 *   theme 1  ocean   — blue / cyan / white      (fountain, rainfall)
 *   theme 2  nature  — green / lime             (general, organic)
 *   theme 3  cosmic  — violet / magenta / white (cosmic, energy)
 *
 * Color pair layout:
 *   pairs [ CP_BASE + theme*RAMP_N .. CP_BASE + theme*RAMP_N + RAMP_N-1 ]
 *   Max pair id = CP_BASE + 4*9 - 1 = 36.  ncurses supports ≥ 256.
 */
#define CP_BASE  1   /* first color pair id we own */
#define N_THEMES 4

typedef struct {
    const char *name;
    int  fg256[RAMP_N];   /* xterm-256 foreground per level               */
    int  fg8  [RAMP_N];   /* 8-color fallback                             */
    attr_t attr8[RAMP_N]; /* A_BOLD / A_DIM / A_NORMAL per level (8-clr)  */
} PTheme;

static const PTheme k_themes[N_THEMES] = {
    {   /* 0  fire */
        "fire",
        {  88, 124, 160, 196, 202, 208, 214, 220, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
        {  A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM,
           A_NORMAL, A_BOLD, A_BOLD, A_BOLD },
    },
    {   /* 1  ocean */
        "ocean",
        {  17, 19, 25, 33, 39, 45, 51, 159, 231 },
        {  COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
           COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE },
        {  A_DIM, A_NORMAL, A_BOLD, A_NORMAL,
           A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD },
    },
    {   /* 2  nature */
        "nature",
        {  22, 28, 34, 40, 46, 82, 118, 154, 231 },
        {  COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
           COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        {  A_DIM, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD },
    },
    {   /* 3  cosmic */
        "cosmic",
        {  53, 91, 93, 129, 165, 201, 207, 213, 231 },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE },
        {  A_DIM, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD },
    },
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    for (int t = 0; t < N_THEMES; t++) {
        for (int i = 0; i < RAMP_N; i++) {
            int pair = CP_BASE + t * RAMP_N + i;
            if (COLORS >= 256)
                init_pair(pair, k_themes[t].fg256[i], COLOR_BLACK);
            else
                init_pair(pair, k_themes[t].fg8[i],   COLOR_BLACK);
        }
    }
}

/* Return ncurses attribute for (theme, ramp_level). */
static attr_t theme_attr(int theme, int level)
{
    int pair = CP_BASE + theme * RAMP_N + level;
    attr_t a = COLOR_PAIR(pair);
    if (COLORS >= 256) {
        if (level >= RAMP_N - 2) a |= A_BOLD;
    } else {
        a |= k_themes[theme].attr8[level];
    }
    return a;
}

/* ===================================================================== */
/* §4  structs — Particle and Emitter                                     */
/* ===================================================================== */

/*
 * Particle — one element in the simulation pool.
 *
 * Position and velocity live in PIXEL SPACE (see §1 CELL_W/CELL_H).
 * Acceleration is cleared and re-accumulated each physics tick (§6).
 *
 * density  — approximates mass per unit volume.
 *            density > 1: particle responds sluggishly to forces (heavy)
 *            density < 1: particle responds sharply (light, buoyant)
 *            This also scales drag: denser particles lose momentum faster.
 *
 * trail[]  — ring buffer of TRAIL_LEN past CELL positions.
 *            Stored as cells (not pixels) so the render step can use them
 *            directly without re-converting every frame.
 */
typedef struct {
    float  x,  y;             /* position   (pixel space)                 */
    float  vx, vy;            /* velocity   (px / s)                      */
    float  ax, ay;            /* acceleration — cleared every tick        */
    float  lifetime;          /* remaining life (seconds)                 */
    float  max_lifetime;      /* initial lifetime — for fade calculation  */
    float  density;           /* mass-like parameter [0.1, 3.0]           */
    int    color;             /* ramp level override (0 = auto)           */
    bool   alive;             /* false = pool slot is available           */
    /* trail ring buffer: last TRAIL_LEN cell positions */
    int    trail_cx[TRAIL_LEN];
    int    trail_cy[TRAIL_LEN];
    int    trail_head;        /* index of next write position             */
} Particle;

/*
 * Emitter — controls where and how new particles are born.
 *
 * angle / spread define the emission cone:
 *   angle = -π/2 → straight up (fountain)
 *   spread = π   → 180° fan
 *   spread = 2π  → full radial burst (explosion)
 *
 * rate / rate_accum — fractional particle accumulator.
 * Because rate may not be an integer, we accumulate the fractional part
 * and carry it between ticks.  This gives an exact average birth rate
 * without biasing toward int-ceiling counts each tick.
 *
 * spawn_width — x-axis scatter around emitter.x.
 * Set to pw(cols) for rainfall (particles spread across whole top edge),
 * 0 for a point source (fountain, explosion).
 */
typedef struct {
    float  x,   y;          /* emitter position in pixel space          */
    float  angle;           /* mean emission direction (radians)        */
    float  spread;          /* total angular spread (radians)           */
    float  speed_min;       /* birth speed range (px/s)                 */
    float  speed_max;
    float  life_min;        /* lifetime range (s)                       */
    float  life_max;
    float  density_min;     /* density range for new particles          */
    float  density_max;
    float  rate;            /* particles per second (continuous mode)   */
    float  rate_accum;      /* fractional carry-over between ticks      */
    float  spawn_width;     /* x-scatter width in pixels (0 = point)    */
    bool   active;          /* continuous emission enabled?             */
} Emitter;

/* ===================================================================== */
/* §5  pool — static particle pool, no-malloc allocator                  */
/* ===================================================================== */

/*
 * g_particles[] is a BSS-segment static array — allocated at program start,
 * never reallocated.  The "no heap allocation inside the update loop"
 * requirement is satisfied because spawn_particle() only sets fields inside
 * an already-allocated slot; it never calls malloc/realloc.
 *
 * Allocation strategy: linear cursor scan.
 *   The cursor advances from its last position, wrapping around.
 *   Because birth_rate ≈ death_rate in steady state, the cursor
 *   travels an expected O(N / free_fraction) before finding a free slot —
 *   essentially O(1) amortised when the pool is < 90% full.
 *
 * When the pool is completely full, pool_alloc() returns NULL and the
 * spawn is silently dropped.  The simulation degrades gracefully rather
 * than crashing or heap-allocating.
 */
static Particle g_particles[MAX_PARTICLES];
static int      g_cursor = 0;   /* allocation cursor */

static void pool_clear(void)
{
    memset(g_particles, 0, sizeof g_particles);
    g_cursor = 0;
}

/* Find a free slot; returns NULL if pool is full. */
static Particle *pool_alloc(void)
{
    for (int i = 0; i < MAX_PARTICLES; i++) {
        int idx = (g_cursor + i) % MAX_PARTICLES;
        if (!g_particles[idx].alive) {
            g_cursor = (idx + 1) % MAX_PARTICLES;
            return &g_particles[idx];
        }
    }
    return NULL;
}

/* Count alive particles (used for stats; called once per tick). */
static int pool_alive_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (g_particles[i].alive) n++;
    return n;
}

/* ===================================================================== */
/* §6  forces — gravity, drag; force-accumulation pattern                */
/* ===================================================================== */

/*
 * apply_forces() — accumulate all active forces into particle acceleration.
 *
 * PATTERN: This function is called FIRST each physics tick, before
 * velocity and position are integrated.  It clears accel then adds each
 * force separately.  To disable a force, simply comment out its line —
 * nothing else changes.
 *
 * Forces currently modelled:
 *   1. Gravity  — constant downward acceleration.
 *      In screen space +y points down, so gravity is positive ay.
 *      Galileo's principle: gravity is independent of mass/density.
 *      (To add buoyancy for smoke: ay += gravity * (1.0 - density) )
 *
 *   2. Drag     — velocity-proportional deceleration.
 *      Applied as a velocity scale in update_particles() (not here),
 *      because scaling is the exact analytical solution of dv/dt = -k*v
 *      and never goes unstable.  See §7.
 *
 * Extending: add wind, vortices, attractors, etc. as:
 *   p->ax += wind_force_x / p->density;
 *   p->ay += wind_force_y / p->density;
 */
static void apply_forces(Particle *p, float gravity)
{
    /* Clear: forces are re-evaluated fresh every tick */
    p->ax = 0.0f;
    p->ay = 0.0f;

    /* Gravity: uniform downward field.
     * Independent of density (equivalence principle). */
    p->ay += gravity;
}

/* ===================================================================== */
/* §7  physics — update_particles, handle_collisions                     */
/* ===================================================================== */

/*
 * update_particles() — advance all alive particles by one fixed timestep.
 *
 * Called from the scene_tick() accumulator loop; dt is always the fixed
 * tick duration (1 / sim_fps seconds).  Never called with variable dt.
 *
 * Per-particle steps (order matters):
 *   1. apply_forces()  — clear accel, re-accumulate forces
 *   2. Drag damping    — scale velocity before integration (exact solution)
 *   3. Symplectic Euler integration
 *                        vel += accel * dt   (velocity from NEW accel)
 *                        pos += vel   * dt   (position from NEW vel)
 *   4. Age the particle — decrease lifetime
 *   5. Update trail ring buffer
 *   6. Kill if lifetime expired
 *
 * The spawn_out / energy accumulators are for the overlay stats panel.
 */
static void update_particles(float dt, float gravity,
                             bool drag_on,
                             float *out_avg_vel, float *out_energy)
{
    double vel_sum    = 0.0;
    double energy_sum = 0.0;
    int    alive      = 0;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->alive) continue;

        /* ── 1. Force accumulation ───────────────────────────────── */
        apply_forces(p, gravity);

        /* ── 2. Drag (velocity scaling, not force) ───────────────── *
         * Drag formula: v(t+dt) = v(t) * e^(-k*dt) ≈ v(t) * (1 - k*dt)
         * We use the multiplicative form because:
         *   • It is the exact solution of dv/dt = -k*v over dt
         *   • It never overshoots zero no matter how large dt is
         *   • Adding drag as an acceleration (a -= k*v) can flip sign
         *     when k*dt > 2, causing explosive growth
         * density scales the drag so heavier particles slow down faster
         * (or lighter particles glide longer, depending on tuning).    */
        if (drag_on) {
            float damp = 1.0f - DRAG_COEFF * p->density * dt;
            if (damp < 0.0f) damp = 0.0f;
            p->vx *= damp;
            p->vy *= damp;
        }

        /* ── 3. Symplectic Euler integration ─────────────────────── *
         * Velocity is updated BEFORE position (symplectic / semi-implicit).
         * Explicit Euler:   pos += vel_old * dt; vel += accel * dt
         * Symplectic Euler: vel += accel * dt;   pos += vel_new * dt  ← this
         * The difference: symplectic uses the already-updated velocity for
         * the position step, giving better energy conservation.           */
        p->vx += p->ax * dt;
        p->vy += p->ay * dt;
        p->x  += p->vx * dt;
        p->y  += p->vy * dt;

        /* ── 4. Age ─────────────────────────────────────────────── */
        p->lifetime -= dt;
        if (p->lifetime <= 0.0f) {
            p->alive = false;
            continue;
        }

        /* ── 5. Trail ring buffer ────────────────────────────────── *
         * Store the CELL position (not pixel) so the render step can
         * use trail entries directly.  We write every tick; the ring
         * buffer automatically evicts the oldest entry.               */
        p->trail_cx[p->trail_head] = px_to_cx(p->x);
        p->trail_cy[p->trail_head] = px_to_cy(p->y);
        p->trail_head = (p->trail_head + 1) % TRAIL_LEN;

        /* ── 6. Stats accumulation ───────────────────────────────── */
        float speed = sqrtf(p->vx*p->vx + p->vy*p->vy);
        vel_sum    += speed;
        energy_sum += 0.5f * p->density * (p->vx*p->vx + p->vy*p->vy);
        alive++;
    }

    *out_avg_vel = (alive > 0) ? (float)(vel_sum / alive) : 0.0f;
    *out_energy  = (float)energy_sum;
}

/*
 * handle_collisions() — enforce world boundaries on all alive particles.
 *
 * World boundaries are 0 and pw(cols)-1 in x, 0 and ph(rows)-1 in y.
 * Three policies per wall:
 *   BOUNCE  — reflect the relevant velocity component and apply BOUNCE_DAMPING.
 *             Kinetic energy is reduced by (1 - BOUNCE_DAMPING²) per bounce.
 *   KILL    — mark particle dead.  Used for rainfall drops, spark embers.
 *   WRAP    — teleport to opposite side.  Used for wind-tunnel loops.
 *
 * The floor_kills / ceiling_kills / wall_kills booleans in Scene map to:
 *   true  → KILL policy
 *   false → BOUNCE policy
 * WRAP is not exposed as a preset option here but is trivial to add.
 */
static void handle_collisions(int cols, int rows,
                              bool floor_kills,
                              bool ceiling_kills,
                              bool wall_kills)
{
    float world_w = (float)pw(cols) - 1.0f;
    float world_h = (float)ph(rows) - 1.0f;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &g_particles[i];
        if (!p->alive) continue;

        /* ── Floor ──────────────────────────────────────────────── */
        if (p->y >= world_h) {
            if (floor_kills) {
                p->alive = false;
                continue;
            }
            /* Inelastic bounce: flip vy, lose BOUNCE_DAMPING fraction */
            p->y  = world_h;
            p->vy = -fabsf(p->vy) * BOUNCE_DAMPING;
            p->vx *= BOUNCE_DAMPING;   /* floor friction damps vx too */
        }

        /* ── Ceiling ─────────────────────────────────────────────── */
        if (p->y < 0.0f) {
            if (ceiling_kills) {
                p->alive = false;
                continue;
            }
            p->y  = 0.0f;
            p->vy = fabsf(p->vy) * BOUNCE_DAMPING;
        }

        /* ── Left / Right walls ──────────────────────────────────── */
        if (p->x < 0.0f) {
            if (wall_kills) { p->alive = false; continue; }
            p->x  = 0.0f;
            p->vx = fabsf(p->vx) * BOUNCE_DAMPING;
        }
        if (p->x >= world_w) {
            if (wall_kills) { p->alive = false; continue; }
            p->x  = world_w;
            p->vx = -fabsf(p->vx) * BOUNCE_DAMPING;
        }
    }
}

/* ===================================================================== */
/* §8  emitter — continuous emission and burst spawning                  */
/* ===================================================================== */

/*
 * rand_float() — uniform random float in [lo, hi).
 * Uses integer rand() to avoid floating-point random state issues.
 */
static float rand_float(float lo, float hi)
{
    return lo + ((float)(rand() % 100000) / 100000.0f) * (hi - lo);
}

/*
 * spawn_one() — fill a pool slot with one freshly born particle.
 *
 * Birth position is emitter.x ± spawn_width/2 (x) and emitter.y (y).
 * Birth velocity is emitter.speed in direction (angle ± spread/2).
 * Returns true if a slot was found, false if the pool was full.
 */
static bool spawn_one(const Emitter *em, int theme)
{
    Particle *p = pool_alloc();
    if (!p) return false;

    /* Scatter x for wide emitters (e.g. rainfall across full top edge) */
    float bx = em->x + rand_float(-em->spawn_width * 0.5f, em->spawn_width * 0.5f);
    float by = em->y;

    /* Random emission angle within the cone */
    float a = em->angle + rand_float(-em->spread * 0.5f, em->spread * 0.5f);
    float s = rand_float(em->speed_min, em->speed_max);

    float life    = rand_float(em->life_min,    em->life_max);
    float density = rand_float(em->density_min, em->density_max);

    p->x   = bx;  p->y   = by;
    p->vx  = cosf(a) * s;
    p->vy  = sinf(a) * s;
    p->ax  = 0.0f;  p->ay = 0.0f;
    p->lifetime     = life;
    p->max_lifetime = life;
    p->density      = density;
    p->color        = theme;    /* store theme so theme-change affects live particles */
    p->alive        = true;
    p->trail_head   = 0;

    /* Pre-fill trail with birth position so first TRAIL_LEN frames
     * don't show trails from (0,0) interpolating to birth pos.    */
    int cx = px_to_cx(bx), cy = px_to_cy(by);
    for (int t = 0; t < TRAIL_LEN; t++) {
        p->trail_cx[t] = cx;
        p->trail_cy[t] = cy;
    }
    return true;
}

/*
 * emitter_tick() — continuous emission for one physics tick.
 *
 * Fractional accumulator pattern: we want `rate` particles per SECOND,
 * but ticks are shorter than a second.  We accumulate rate*dt each tick
 * and spawn floor(accum) particles, keeping the fractional remainder.
 * This gives an accurate average birth rate with no systematic bias.
 *
 * Returns number of particles actually spawned this tick (for stats).
 */
static int emitter_tick(Emitter *em, float dt, int theme)
{
    if (!em->active) return 0;

    em->rate_accum += em->rate * dt;
    int to_spawn = (int)em->rate_accum;
    if (to_spawn > MAX_BURST) to_spawn = MAX_BURST;
    em->rate_accum -= (float)to_spawn;

    int spawned = 0;
    for (int i = 0; i < to_spawn; i++)
        if (spawn_one(em, theme)) spawned++;
    return spawned;
}

/*
 * spawn_burst() — immediately emit `count` particles.
 *
 * Used for: manual 'b' keypress, auto-burst timer, firework explosions.
 * Ignores emitter.active — bursts always fire regardless of toggle state.
 */
static int spawn_burst(const Emitter *em, int count, int theme)
{
    if (count > MAX_BURST) count = MAX_BURST;
    int spawned = 0;
    for (int i = 0; i < count; i++)
        if (spawn_one(em, theme)) spawned++;
    return spawned;
}

/* ===================================================================== */
/* §9  render — four render modes + overlay stats panel                  */
/* ===================================================================== */

/* ── density grid (heatmap mode only) ──────────────────────────────── */
/*
 * g_density is a GRID_MAX_W × GRID_MAX_H float grid cleared and rebuilt
 * every render frame.  It is in CELL SPACE (columns × rows), not pixel.
 * It is global/static (BSS) — no allocation in the render path.
 */
static float g_density[GRID_MAX_H][GRID_MAX_W];
/* previous density for borderless erase (only clear cells that changed) */
static float g_prev_density[GRID_MAX_H][GRID_MAX_W];

/*
 * splat_density() — deposit a Gaussian blob onto the density grid.
 *
 * A 3×3 kernel is used (identical to fire.c's particle splat):
 *   corners     : 0.0625  (1/16)
 *   edge-centres: 0.125   (2/16)
 *   centre      : 0.25    (4/16)
 *   sum = 4×(1/16) + 4×(2/16) + 4/16 = 1.0
 *
 * Weight by life_frac so particles fade out in the heatmap as they age.
 * Clamping to grid boundaries avoids out-of-bounds writes.
 */
static void splat_density(int cx, int cy, float weight,
                          int cols, int rows)
{
    static const float k[3][3] = {
        { 0.0625f, 0.125f, 0.0625f },
        { 0.125f,  0.25f,  0.125f  },
        { 0.0625f, 0.125f, 0.0625f },
    };
    for (int dy = -1; dy <= 1; dy++) {
        int gy = cy + dy;
        if (gy < 0 || gy >= rows || gy >= GRID_MAX_H) continue;
        for (int dx = -1; dx <= 1; dx++) {
            int gx = cx + dx;
            if (gx < 0 || gx >= cols || gx >= GRID_MAX_W) continue;
            g_density[gy][gx] += weight * k[dy+1][dx+1];
        }
    }
}

/*
 * 8-way direction arrows for RENDER_ARROW mode.
 *
 * Octant mapping (screen space: +x right, +y DOWN):
 *   angle = atan2(vy, vx)  — gives angle in [-π, π]
 *   We map this to 8 octants (0 = right, going clockwise).
 *
 * Characters '/' and '\' appear twice because they are symmetric —
 * the same slash encodes both (right-up) and (left-down) motion.
 * This is accurate: think of '/' as "the particle is going diagonally
 * along this stroke's direction", same for '\'.
 */
static const char k_arrows[8] = {
    '>',    /* 0: right        [−π/8 ,  π/8)  */
    '\\',   /* 1: right-down   [ π/8 , 3π/8)  */
    'v',    /* 2: down         [3π/8 , 5π/8)  */
    '/',    /* 3: left-down    [5π/8 , 7π/8)  */
    '<',    /* 4: left         [7π/8 ,−7π/8)  */
    '\\',   /* 5: left-up      [−7π/8,−5π/8)  */
    '^',    /* 6: up           [−5π/8,−3π/8)  */
    '/',    /* 7: right-up     [−3π/8,−π/8)   */
};

static char velocity_arrow(float vx, float vy)
{
    float angle = atan2f(vy, vx);                 /* [-π, π]          */
    float norm  = angle + (float)M_PI;             /* [0, 2π)          */
    int   oct   = (int)(norm / (float)M_PI * 4.0f) % 8;
    return k_arrows[oct];
}

/*
 * render_heatmap() — continuous density-field view of the particle cloud.
 *
 * This mode treats all particles as sources of a scalar density field rather
 * than discrete points.  The result looks like infrared heat or smoke density.
 *
 * Algorithm (three passes):
 *
 *   PASS 1 — erase stale cells:
 *     Cells that had density > threshold last frame but are now empty need
 *     explicit blanking.  We compare g_prev_density vs g_density (before
 *     rebuilding) instead of clearing the whole screen each frame.
 *     WHY: clearing all ROWS×COLS cells each frame causes terminal flicker
 *     because ncurses still has to repaint them all.  Differential erase
 *     only touches cells that actually changed.
 *
 *   PASS 2 — accumulate density:
 *     g_density is zeroed, then every alive particle splats a 3×3 Gaussian
 *     blob weighted by (speed / MAX_SPEED_NORM) × life_fraction.
 *     WHY weight by speed: fast particles carry more kinetic energy and should
 *     appear brighter (hotter).  A stationary particle contributes nothing.
 *     WHY weight by life_frac: dying particles fade out naturally in the field
 *     without needing per-particle dim logic.
 *
 *   PASS 3 — render density grid:
 *     Normalised density → lut_index → k_ramp character + theme colour pair.
 *     Cells below 0.01 are skipped (background stays dark).
 */
static void render_heatmap(WINDOW *w, int cols, int rows, int theme)
{
    /* PASS 1: erase cells that were occupied last frame but are now empty */
    for (int r = 0; r < rows && r < GRID_MAX_H; r++)
        for (int c = 0; c < cols && c < GRID_MAX_W; c++)
            if (g_prev_density[r][c] > 0.01f && g_density[r][c] < 0.01f)
                mvwaddch(w, r, c, ' ');

    /* Save snapshot of current density so next frame's PASS 1 can diff it */
    memcpy(g_prev_density, g_density, sizeof(float) * GRID_MAX_H * GRID_MAX_W);

    /* PASS 2: rebuild density from current particle positions */
    memset(g_density, 0, sizeof g_density);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &g_particles[i];
        if (!p->alive) continue;
        int   cx     = px_to_cx(p->x);
        int   cy     = px_to_cy(p->y);
        float lf     = p->lifetime / p->max_lifetime;
        float speed  = sqrtf(p->vx*p->vx + p->vy*p->vy);
        float weight = (speed / MAX_SPEED_NORM) * lf;
        if (weight > 1.0f) weight = 1.0f;
        splat_density(cx, cy, weight, cols, rows);
    }

    /* PASS 3: map density → character + colour */
    for (int r = 0; r < rows && r < GRID_MAX_H; r++) {
        for (int c = 0; c < cols && c < GRID_MAX_W; c++) {
            float d = g_density[r][c];
            if (d < 0.01f) continue;
            int    lvl  = lut_index(d);
            if (lvl == 0) continue;
            attr_t attr = theme_attr(theme, lvl);
            wattron(w, attr);
            mvwaddch(w, r, c, k_ramp[lvl]);
            wattroff(w, attr);
        }
    }
}

/*
 * render_per_particle() — GLYPH / TRAIL / ARROW modes.
 *
 * Every alive particle is drawn as a single character at its current cell
 * position.  The three modes share the same speed→level and life→dim logic;
 * they differ only in which character is placed:
 *
 *   GLYPH  — k_ramp[lvl]: density glyph encodes kinetic energy directly.
 *   TRAIL  — draws TRAIL_LEN-1 past cell positions first (fading), then the
 *             current glyph.  Older trail entries are dimmer (tlvl -= t/2).
 *             WHY draw trail before current: ensures the head glyph is on top
 *             and not overwritten by a trail entry from the next particle.
 *   ARROW  — velocity_arrow(vx, vy): 8-way direction char instead of density.
 *             Good for visualising flow fields and force alignment.
 *
 * Life-based dimming at 30%:
 *   At lifetime_fraction < 0.30, A_DIM replaces A_BOLD.  This creates a
 *   visible "dying" fade that doesn't require gradual alpha — just two
 *   brightness levels available in all ncurses terminals.
 */
static void render_per_particle(WINDOW *w, int cols, int rows,
                                int mode, int theme)
{
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &g_particles[i];
        if (!p->alive) continue;

        int   cx    = px_to_cx(p->x);
        int   cy    = px_to_cy(p->y);
        float lf    = p->lifetime / p->max_lifetime;
        float speed = sqrtf(p->vx*p->vx + p->vy*p->vy);

        /* Speed → ramp level: fast = visually dense, slow = sparse */
        float norm_spd = speed / MAX_SPEED_NORM;
        if (norm_spd > 1.0f) norm_spd = 1.0f;
        int lvl = lut_index(norm_spd);
        if (lvl < 1) lvl = 1;   /* always at least faintly visible */

        /* Life fraction < 30% → dim (dying visual cue) */
        attr_t base_attr = theme_attr(theme, lvl);
        if (lf < 0.30f) base_attr = (base_attr & ~A_BOLD) | A_DIM;

        /* TRAIL: render ring-buffer history first (older = dimmer) */
        if (mode == RENDER_TRAIL) {
            for (int t = 0; t < TRAIL_LEN - 1; t++) {
                /* Walk backwards through the ring buffer: index t=0 is newest
                 * trail entry (one tick ago), t=TRAIL_LEN-2 is oldest.       */
                int tidx = (p->trail_head - 1 - t + TRAIL_LEN) % TRAIL_LEN;
                int tx = p->trail_cx[tidx];
                int ty = p->trail_cy[tidx];
                if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;
                int tlvl = lvl - t / 2;   /* older entries drop one level per 2 steps */
                if (tlvl < 1) tlvl = 1;
                attr_t ta = theme_attr(theme, tlvl) | A_DIM;
                wattron(w, ta);
                mvwaddch(w, ty, tx, k_ramp[tlvl]);
                wattroff(w, ta);
            }
        }

        /* Current position: glyph, arrow, or density char */
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        char ch = (mode == RENDER_ARROW) ? velocity_arrow(p->vx, p->vy)
                                         : k_ramp[lvl];
        wattron(w, base_attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
        wattroff(w, base_attr);
    }
}

/*
 * render_particles() — dispatch to the active render mode.
 *
 * Visual encoding summary by mode:
 *   HEATMAP — continuous density field (Gaussian splat per particle).
 *             Best for fluid/smoke where individual identity is irrelevant.
 *   GLYPH   — per-particle: speed → character density.
 *   TRAIL   — per-particle: past positions fading behind each particle.
 *   ARROW   — per-particle: velocity angle → 8-way direction glyph.
 */
static void render_particles(WINDOW *w, int cols, int rows,
                             int mode, int theme)
{
    if (mode == RENDER_HEATMAP) {
        render_heatmap(w, cols, rows, theme);
    } else {
        render_per_particle(w, cols, rows, mode, theme);
    }
}

/*
 * render_overlay() — stats panel in the bottom-left corner.
 *
 * Displays the six key simulation metrics requested by the spec:
 *   particle_count, dt, simulation_time, spawn_rate, avg_velocity,
 *   energy_estimate.
 *
 * Also shows active preset, theme, render mode, and gravity so the
 * user always knows the current simulation state.
 *
 * Drawn LAST inside screen_draw so it appears on top of particles.
 */
static void render_overlay(WINDOW *w, int cols, int rows,
                           int particle_count, float dt_sec,
                           float sim_time, int spawn_rate,
                           float avg_vel, float energy,
                           float gravity, bool drag_on,
                           int preset_id, int theme_id,
                           int render_mode, bool paused)
{
    /* Choose a position that fits: bottom-left, 28 cols wide, 10 rows tall */
    int panel_w = 28;
    int panel_h = 10;
    int ox = 1;
    int oy = rows - panel_h - 1;
    if (oy < 0) oy = 0;
    if (ox + panel_w > cols) return;   /* terminal too narrow — skip */

    /* Box top */
    wattron(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
    mvwprintw(w, oy, ox, "+-- PARTICLE ENGINE ------+");

    /* Stats rows */
    mvwprintw(w, oy+1, ox, "| cnt  %5d / %-5d      |", particle_count, MAX_PARTICLES);
    mvwprintw(w, oy+2, ox, "| dt   %7.4f s          |", dt_sec);
    mvwprintw(w, oy+3, ox, "| time %7.2f s          |", sim_time);
    mvwprintw(w, oy+4, ox, "| rate %5d /tick        |", spawn_rate);
    mvwprintw(w, oy+5, ox, "| avgv %7.1f px/s       |", avg_vel);
    mvwprintw(w, oy+6, ox, "| nrg  %9.1f          |", energy);

    /* State row */
    mvwprintw(w, oy+7, ox, "| %-6s %-6s %-4s %4.0fg |",
              k_preset_names[preset_id],
              k_themes[theme_id].name,
              k_render_names[render_mode],
              gravity);

    /* Box bottom */
    mvwprintw(w, oy+8, ox, "| drag:%-3s  %s              |",
              drag_on ? "ON " : "OFF",
              paused  ? "PAUSED " : "running");
    mvwprintw(w, oy+9, ox, "+-------------------------+");
    wattroff(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
}

/* ===================================================================== */
/* §10 presets — fireworks, fountain, rainfall, explosion                 */
/* ===================================================================== */

/*
 * Each preset configures the Scene's emitter + physics parameters.
 * Presets require cols/rows because the emitter position is derived
 * from the terminal dimensions (pixel space).
 *
 * Design notes per preset:
 *
 *   FOUNTAIN  — classic upward spray.  Gravity curves arcs downward.
 *               Drag simulates air resistance making trajectories realistic.
 *               Bounce floor so drops accumulate then visually "roll off".
 *
 *   FIREWORKS — fast upward launches with wide radial spread simulate
 *               the burst seen at the peak of a rocket.  Low drag lets
 *               sparks travel far.  Kill at walls for clean look.
 *               Auto-burst every 2.5 s keeps the display active.
 *
 *   RAINFALL  — particles fill the entire top edge (spawn_width = pw(cols)).
 *               Strong gravity, narrow downward cone, kill at floor so drops
 *               don't bounce (realistic rain behaviour).
 *               Arrow render mode makes the fall direction clearly visible.
 *
 *   EXPLOSION — radial burst (spread = 2π) from screen centre.
 *               Strong drag dissipates the burst quickly.  Heatmap mode
 *               shows the density shockwave expanding outward.
 *               Auto-burst every 3 s with brief quiet gap.
 */

/* Forward declaration — Emitter lives in Scene defined in §11 */
typedef struct Scene Scene;

static void preset_fountain(Scene *s, int cols, int rows);
static void preset_fireworks(Scene *s, int cols, int rows);
static void preset_rainfall(Scene *s, int cols, int rows);
static void preset_explosion(Scene *s, int cols, int rows);

static void preset_apply(Scene *s, int id, int cols, int rows);

/* ===================================================================== */
/* §11 scene — top-level simulation state; tick + draw                   */
/* ===================================================================== */

/*
 * Scene — owns all mutable simulation state.
 *
 * The scene knows nothing about ncurses; it only updates physics and
 * writes to a WINDOW* (passed from screen_draw).  This separation means
 * the physics can be tested without a terminal.
 */
struct Scene {
    Emitter  emitter;

    /* physics parameters (can be modified at runtime) */
    float    gravity;
    bool     drag_on;

    /* boundary policies (set by preset_apply) */
    bool     floor_kills;
    bool     ceiling_kills;
    bool     wall_kills;

    /* visual state */
    int      preset_id;
    int      theme_id;
    int      render_mode;
    bool     paused;

    /* auto-burst */
    float    burst_interval;   /* seconds between auto-bursts (0 = off)   */
    float    burst_timer;      /* countdown to next auto-burst            */
    int      burst_count;      /* particles per auto-burst                */

    /* stats (recomputed each tick; read by render_overlay) */
    int      particle_count;
    float    dt_sec;
    float    simulation_time;
    int      spawn_rate_last;
    float    avg_velocity;
    float    energy_estimate;
};

/* ── preset implementations ────────────────────────────────────────── */

static void preset_fountain(Scene *s, int cols, int rows)
{
    Emitter *em = &s->emitter;
    em->x           = (float)pw(cols) * 0.5f;
    em->y           = (float)ph(rows) - 2.0f;
    em->angle       = -(float)M_PI * 0.5f;   /* straight up */
    em->spread      = (float)M_PI / 3.0f;    /* 60° cone    */
    em->speed_min   = 150.0f;
    em->speed_max   = 380.0f;
    em->life_min    = 2.5f;
    em->life_max    = 5.0f;
    em->density_min = 0.6f;
    em->density_max = 1.2f;
    em->rate        = 22.0f;
    em->rate_accum  = 0.0f;
    em->spawn_width = 0.0f;
    em->active      = true;

    s->gravity        = 200.0f;
    s->drag_on        = true;
    s->floor_kills    = false;   /* bounce at floor */
    s->ceiling_kills  = true;
    s->wall_kills     = false;
    s->burst_interval = 0.0f;
    s->burst_count    = 80;
    s->render_mode    = RENDER_TRAIL;
    s->theme_id       = 1;   /* ocean */
}

static void preset_fireworks(Scene *s, int cols, int rows)
{
    Emitter *em = &s->emitter;
    em->x           = (float)pw(cols) * 0.5f;
    em->y           = (float)ph(rows) - 2.0f;
    em->angle       = -(float)M_PI * 0.5f;   /* upward launch */
    em->spread      = 2.0f * (float)M_PI;    /* full radial   */
    em->speed_min   = 200.0f;
    em->speed_max   = 600.0f;
    em->life_min    = 1.5f;
    em->life_max    = 3.5f;
    em->density_min = 0.4f;
    em->density_max = 0.9f;
    em->rate        = 0.0f;    /* burst-only, no continuous */
    em->rate_accum  = 0.0f;
    em->spawn_width = 0.0f;
    em->active      = false;

    s->gravity        = 180.0f;
    s->drag_on        = false;
    s->floor_kills    = true;
    s->ceiling_kills  = false;
    s->wall_kills     = true;
    s->burst_interval = 2.5f;
    s->burst_count    = 150;
    s->burst_timer    = 0.5f;  /* first burst soon after switch */
    s->render_mode    = RENDER_GLYPH;
    s->theme_id       = 0;   /* fire */
}

static void preset_rainfall(Scene *s, int cols, int rows)
{
    (void)rows;
    Emitter *em = &s->emitter;
    em->x           = (float)pw(cols) * 0.5f;
    em->y           = 0.0f;
    em->angle       = (float)M_PI * 0.5f;    /* straight down  */
    em->spread      = 0.25f;                  /* narrow cone    */
    em->speed_min   = 180.0f;
    em->speed_max   = 380.0f;
    em->life_min    = 1.5f;
    em->life_max    = 3.5f;
    em->density_min = 0.5f;
    em->density_max = 1.0f;
    em->rate        = 45.0f;
    em->rate_accum  = 0.0f;
    em->spawn_width = (float)pw(cols);   /* scatter across full top */
    em->active      = true;

    s->gravity        = 280.0f;
    s->drag_on        = true;
    s->floor_kills    = true;
    s->ceiling_kills  = false;
    s->wall_kills     = false;
    s->burst_interval = 0.0f;
    s->burst_count    = 60;
    s->render_mode    = RENDER_ARROW;
    s->theme_id       = 1;   /* ocean */
}

static void preset_explosion(Scene *s, int cols, int rows)
{
    Emitter *em = &s->emitter;
    em->x           = (float)pw(cols) * 0.5f;
    em->y           = (float)ph(rows) * 0.5f;
    em->angle       = 0.0f;
    em->spread      = 2.0f * (float)M_PI;   /* full 360° */
    em->speed_min   = 40.0f;
    em->speed_max   = 480.0f;
    em->life_min    = 0.8f;
    em->life_max    = 2.8f;
    em->density_min = 0.3f;
    em->density_max = 1.5f;
    em->rate        = 0.0f;
    em->rate_accum  = 0.0f;
    em->spawn_width = 0.0f;
    em->active      = false;

    s->gravity        = 220.0f;
    s->drag_on        = true;
    s->floor_kills    = false;
    s->ceiling_kills  = false;
    s->wall_kills     = false;
    s->burst_interval = 3.5f;
    s->burst_count    = 200;
    s->burst_timer    = 0.2f;
    s->render_mode    = RENDER_HEATMAP;
    s->theme_id       = 0;   /* fire */
}

static void preset_apply(Scene *s, int id, int cols, int rows)
{
    /* Reset burst timer so presets don't inherit stale countdown */
    s->burst_timer = 9999.0f;

    switch (id) {
    case PRESET_FOUNTAIN:   preset_fountain(s, cols, rows);  break;
    case PRESET_FIREWORKS:  preset_fireworks(s, cols, rows); break;
    case PRESET_RAINFALL:   preset_rainfall(s, cols, rows);  break;
    case PRESET_EXPLOSION:  preset_explosion(s, cols, rows); break;
    default: break;
    }
    s->preset_id = id;
}

/* ── scene lifecycle ────────────────────────────────────────────────── */

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->gravity    = GRAVITY_DEFAULT;
    s->drag_on    = true;
    s->paused     = false;
    pool_clear();
    memset(g_density,      0, sizeof g_density);
    memset(g_prev_density, 0, sizeof g_prev_density);
    preset_apply(s, PRESET_FOUNTAIN, cols, rows);
}

static void scene_reset(Scene *s, int cols, int rows)
{
    pool_clear();
    memset(g_density,      0, sizeof g_density);
    memset(g_prev_density, 0, sizeof g_prev_density);
    s->simulation_time = 0.0f;
    /* Re-apply current preset to reset emitter state */
    preset_apply(s, s->preset_id, cols, rows);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep.
 *
 * Called from the accumulator loop in §13.  dt is always exactly
 * 1.0 / sim_fps seconds.  The order of operations is important:
 *
 *   1. Emitter tick       — spawn new particles (fractional accumulator)
 *   2. Auto-burst timer   — fire periodic bursts
 *   3. update_particles   — forces → drag → integrate → age → trail
 *   4. handle_collisions  — enforce world boundaries
 *   5. Update stats       — particle_count, avg_vel, energy (for overlay)
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused) return;

    s->dt_sec           = dt;
    s->simulation_time += dt;

    /* ── 1. Continuous emitter ───────────────────────────────────── */
    int spawned = emitter_tick(&s->emitter, dt, s->theme_id);

    /* ── 2. Auto-burst timer ─────────────────────────────────────── */
    if (s->burst_interval > 0.0f) {
        s->burst_timer -= dt;
        if (s->burst_timer <= 0.0f) {
            spawned += spawn_burst(&s->emitter, s->burst_count, s->theme_id);
            s->burst_timer = s->burst_interval;
        }
    }
    s->spawn_rate_last = spawned;

    /* ── 3. Physics step ─────────────────────────────────────────── */
    float avg_vel = 0.0f, energy = 0.0f;
    update_particles(dt, s->gravity, s->drag_on, &avg_vel, &energy);

    /* ── 4. Boundary collisions ──────────────────────────────────── */
    handle_collisions(cols, rows,
                      s->floor_kills, s->ceiling_kills, s->wall_kills);

    /* ── 5. Stats ────────────────────────────────────────────────── */
    s->particle_count = pool_alive_count();
    s->avg_velocity   = avg_vel;
    s->energy_estimate= energy;
}

/*
 * scene_draw() — render all particles and the overlay into WINDOW *w.
 *
 * alpha is the sub-tick interpolation factor ∈ [0,1).
 * For a pure particle system we could lerp positions by alpha * vel * dt,
 * but at 60 fps the visual difference is imperceptible for fast particles.
 * We accept alpha for API consistency and use it only to skip the render
 * while paused (alpha = 0 implies no progress to show).
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    render_particles(w, cols, rows, s->render_mode, s->theme_id);

    render_overlay(w, cols, rows,
                   s->particle_count, s->dt_sec,
                   s->simulation_time, s->spawn_rate_last,
                   s->avg_velocity, s->energy_estimate,
                   s->gravity, s->drag_on,
                   s->preset_id, s->theme_id,
                   s->render_mode, s->paused);
}

/* ===================================================================== */
/* §12 screen — ncurses double-buffer display layer                      */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer (identical to framework.c §7).
 *
 * FRAME SEQUENCE (one atomic write per frame):
 *   erase()              — blank newscr
 *   scene_draw()         — particle render + overlay
 *   key hint row         — always written last, always on top
 *   wnoutrefresh(stdscr) — mark newscr ready; no terminal I/O yet
 *   doupdate()           — ONE diff write to the terminal fd
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top-right */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  preset:%-10s theme:%-6s ",
             fps, sim_fps,
             k_preset_names[sc->preset_id],
             k_themes[sc->theme_id].name);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_BASE + sc->theme_id * RAMP_N + 6) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_BASE + sc->theme_id * RAMP_N + 6) | A_BOLD);

    /* Key hint — bottom row */
    attron(A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit spc:pause b:burst e:emit g/G:grav "
             "d:drag r:reset p:preset t:theme v:view [/]:Hz ");
    attroff(A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §13 app — signals, resize, input, main loop                           */
/* ===================================================================== */

typedef struct {
    Scene                  scene;
    Screen                 screen;
    int                    sim_fps;
    volatile sig_atomic_t  running;
    volatile sig_atomic_t  need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    /* Re-apply current preset so emitter positions use new dimensions */
    preset_apply(&app->scene, app->scene.preset_id,
                 app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — all user input in one place.
 *
 * Returns false to signal quit; true to continue.
 * Controls are grouped by category:
 *   navigation (q/ESC/space), spawning (b/e), physics (g/G/d),
 *   simulation (r/p/P), visual (t/v), Hz (]/[).
 */
static bool app_handle_key(App *app, int ch)
{
    Scene  *s  = &app->scene;
    Screen *sc = &app->screen;

    switch (ch) {
    /* ── quit / pause ──────────────────────────────────────────── */
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused = !s->paused; break;

    /* ── spawning ────────────────────────────────────────────────
     * 'b' triggers an immediate burst regardless of emitter state.
     * 'e' toggles the continuous emitter on/off.               */
    case 'b': case 'B':
        spawn_burst(&s->emitter, s->burst_count, s->theme_id);
        break;
    case 'e': case 'E':
        s->emitter.active = !s->emitter.active;
        break;

    /* ── gravity ─────────────────────────────────────────────────
     * 'g' increases gravity (heavier), 'G' decreases (lighter).
     * Gravity can go to zero for zero-g particle clouds.       */
    case 'g':
        s->gravity += GRAVITY_STEP;
        if (s->gravity > GRAVITY_MAX) s->gravity = GRAVITY_MAX;
        break;
    case 'G':
        s->gravity -= GRAVITY_STEP;
        if (s->gravity < GRAVITY_MIN) s->gravity = GRAVITY_MIN;
        break;

    /* ── drag ────────────────────────────────────────────────────
     * Toggling drag off lets particles travel in pure ballistic
     * arcs — useful for visualising initial velocity directions. */
    case 'd': case 'D':
        s->drag_on = !s->drag_on;
        break;

    /* ── reset ───────────────────────────────────────────────────
     * Clears all particles and resets the emitter to preset defaults. */
    case 'r': case 'R':
        scene_reset(s, sc->cols, sc->rows);
        break;

    /* ── cycle presets ───────────────────────────────────────────
     * 'p' advances forward, 'P' goes backward through the 4 presets.
     * Resets the particle pool so the new effect starts clean.  */
    case 'p':
        pool_clear();
        memset(g_density,      0, sizeof g_density);
        memset(g_prev_density, 0, sizeof g_prev_density);
        preset_apply(s, (s->preset_id + 1) % PRESET_COUNT, sc->cols, sc->rows);
        break;
    case 'P':
        pool_clear();
        memset(g_density,      0, sizeof g_density);
        memset(g_prev_density, 0, sizeof g_prev_density);
        preset_apply(s, (s->preset_id + PRESET_COUNT - 1) % PRESET_COUNT,
                     sc->cols, sc->rows);
        break;

    /* ── cycle themes ────────────────────────────────────────────
     * No color pair reinitialisation needed — all 4×9 = 36 pairs
     * were defined at startup.  Switching is zero-cost.        */
    case 't': case 'T':
        s->theme_id = (s->theme_id + 1) % N_THEMES;
        break;

    /* ── cycle render modes ──────────────────────────────────────
     * Switching modes is instant; no state to reset except the
     * density grid which we clear to avoid a stale frame.      */
    case 'v': case 'V':
        s->render_mode = (s->render_mode + 1) % RENDER_COUNT;
        if (s->render_mode == RENDER_HEATMAP) {
            memset(g_density,      0, sizeof g_density);
            memset(g_prev_density, 0, sizeof g_prev_density);
        }
        break;

    /* ── simulation Hz ───────────────────────────────────────────
     * Higher Hz = finer physics timestep = more accurate but
     * more CPU.  Lower Hz = coarser but cheaper.               */
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
 * main() — fixed-timestep accumulator game loop
 *
 * Identical in structure to framework.c main().  Only scene_*() calls
 * change between animations.  See framework.c §8 for a detailed
 * explanation of each loop phase.
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

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* pause guard */

        /* ── sim accumulator (fixed timestep) ────────────────────── *
         * Physics always runs at exactly sim_fps Hz regardless of the
         * render frame rate.  The leftover in sim_accum carries forward
         * to the next frame — no time is ever lost or invented.      */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha — sub-tick render interpolation factor ─────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter (500 ms sliding window) ─────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep BEFORE render ─────────────────────── *
         * Sleeping before render means terminal I/O time is NOT charged
         * against the frame budget.  The next frame's dt measurement
         * starts after the sleep ends, not after doupdate() returns.  */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
