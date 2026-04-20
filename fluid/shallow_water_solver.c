/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fluid/shallow_water_solver.c — 2-D Shallow Water Height Field Simulator
 *
 * Solves the linearized 2-D shallow water equations on the terminal grid:
 *
 *   ∂u/∂t  = -g · ∂h/∂x              (x-momentum: gravity drives x-flow)
 *   ∂v/∂t  = -g · ∂h/∂y              (y-momentum: gravity drives y-flow)
 *   ∂h/∂t  = -H₀·(∂u/∂x + ∂v/∂y)    (mass conservation: divergence of
 *                                       velocity field changes water level)
 *
 * using explicit finite differences on a collocated grid.  Velocity is
 * updated from height gradients first (symplectic-like order); height is
 * then updated from the new velocity divergence.  The two loops must stay
 * separate — same reason as the membrane solver.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *  §1  config      — all tunable constants
 *  §2  clock       — monotonic nanosecond clock + sleep
 *  §3  theme       — height-field color pipeline; ASCII ramp; LUT
 *  §4  grid        — static field arrays (h, u, v, obstacles)
 *  §5  solver      — init_heightfield, update_velocity, update_height,
 *                    apply_boundary_conditions, apply_obstacles, compute_stats
 *  §6  excitation  — apply_drop, obstacle helpers, preset functions
 *  §7  render      — render_heightmap (height + arrows + shoreline),
 *                    render_overlay
 *  §8  scene       — Scene struct, scene_init/tick/draw/reset
 *  §9  screen      — ncurses double-buffer display layer
 *  §10 app         — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q/ESC   quit        space  pause/resume    s    single step
 *   r       reset       d      drop at center  b    trigger dam break
 *   g/G     gravity±    a      flow arrows     l    shoreline toggle
 *   o       obstacles   n      cycle BC        p/P  cycle preset
 *   t       theme       [/]    sim Hz±
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/shallow_water_solver.c \
 *       -o shallow_water -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Shallow Water Equations (§5):
 *   The SWE describe depth-averaged flow in a thin layer of fluid.
 *   'h' here is the HEIGHT PERTURBATION: h = H_total - H₀, where H₀ is the
 *   undisturbed rest depth.  Positive h = water above rest (crest).
 *   Negative h = water below rest (trough / depression).
 *
 *   In linearized form (valid when |h| << H₀):
 *     ∂u/∂t = -g ∂h/∂x       pressure gradient accelerates flow toward low h
 *     ∂v/∂t = -g ∂h/∂y
 *     ∂h/∂t = -H₀(∂u/∂x + ∂v/∂y)   divergence of flow lowers water level
 *
 * Mass conservation (§5 update_height):
 *   The third equation is the continuity equation.  It says: if more water
 *   flows out of a cell than in (positive divergence), the water level drops.
 *   Over the whole domain, the total ∑h is conserved (no sources or sinks).
 *   The dam break preset violates this momentarily to set up the initial
 *   condition; after that, mass is conserved to numerical precision.
 *
 * Wave propagation speed (§5, §7):
 *   Small disturbances travel at c = √(g · H₀) in all directions.
 *   Physically: deeper water → higher wave speed (tsunamis in deep ocean
 *   travel at ~800 km/h; in shallow coastal regions they slow to ~100 km/h).
 *   Here H₀=1 (normalised), so c = √g.  Default g=400 → c=20 cells/s.
 *   Increasing gravity speeds up all waves proportionally.
 *
 * Finite difference stencil (§5):
 *   We use forward-difference for gradients and backward-difference for
 *   divergence.  This "upwind" coupling avoids the checkerboard instability
 *   that plagues centered-difference schemes on collocated grids, while
 *   keeping the code simple without a staggered (Arakawa C) grid layout:
 *
 *     ∂h/∂x at (r,c)  ≈  h[r][c+1] - h[r][c]   (forward)
 *     ∂u/∂x at (r,c)  ≈  u[r][c]   - u[r][c-1]  (backward)
 *
 *   The asymmetry between the two equations creates an implicit half-cell
 *   stagger: velocity u[r][c] conceptually lives at the right edge of cell c.
 *
 * CFL numerical stability (§5, §7):
 *   For 2-D explicit SWE: CFL = c · dt / dx ≤ 1/√2 ≈ 0.707.
 *   With dx=1 (terminal cells), dt = 1/sim_fps:
 *     CFL = c / sim_fps = √g / sim_fps.
 *   Default: √400 / 60 = 20/60 ≈ 0.333  (comfortable STABLE margin).
 *   If the user raises gravity or lowers sim_fps, CFL → instability.
 *   The overlay shows CFL and colours it green/yellow/red.
 *
 * Obstacle handling (§5 apply_obstacles):
 *   Obstacle cells are flagged in g_obs[r][c].  After every velocity and
 *   height update, all three fields are zeroed at obstacle cells — this
 *   enforces a no-flux (impermeable wall) condition.  Waves hitting an
 *   obstacle reflect off its boundary.  A thin wall one cell wide is
 *   sufficient; wider obstacles reduce numerical diffraction effects.
 *
 * Dam break (§6 preset_apply PRESET_DAM_BREAK):
 *   At t=0: left half h = +AMP, right half h = -AMP, u = v = 0.
 *   The pressure gradient ∂h/∂x is enormous at the dam face (col=cols/2),
 *   so velocity rapidly builds there.  Within a few ticks, a shock-like
 *   bore propagates rightward and a rarefaction wave runs leftward.
 *   In reality (nonlinear SWE) the bore steepens; here (linearized) it
 *   spreads out slightly due to numerical dispersion — still visually dramatic.
 *
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =   5,
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =   5,

    TARGET_FPS      =  60,
    FPS_UPDATE_MS   = 500,
    HUD_COLS        =  80,
};

/*
 * Grid — physics runs directly in terminal-cell coordinates.
 * Three float fields + one bool field:
 *   3 × 300×100 × 4 bytes = 360 KB   (BSS)
 *   1 × 300×100 × 1 byte  =  30 KB
 *   Total ≈ 390 KB — well within BSS limits.
 */
#define GRID_MAX_W   300
#define GRID_MAX_H   100

/* Rest depth H₀ (normalised to 1; changes wave speed via g only) */
#define H0           1.0f

/* Gravity — controls wave speed: c = √(g·H₀) = √g (with H₀=1). */
#define GRAVITY_DEFAULT  400.0f   /* → c = 20 cells/s, CFL≈0.33 at 60 Hz */
#define GRAVITY_MIN       25.0f   /* → c = 5  cells/s                     */
#define GRAVITY_MAX     1600.0f   /* → c = 40 cells/s (CFL→0.67 at 60 Hz) */
#define GRAVITY_STEP     100.0f

/* Velocity damping (mimics bottom friction) */
#define DAMPING_DEFAULT  0.004f
#define DAMPING_MIN      0.000f
#define DAMPING_MAX      0.060f
#define DAMPING_STEP     0.002f

/* Excitation */
#define DROP_AMP         1.4f    /* peak height of a radial drop     */
#define DROP_RADIUS      3.0f    /* Gaussian half-width in cells     */
#define DAM_AMP          1.2f    /* ±height of dam-break condition   */

/* Rendering thresholds */
#define DISPLAY_RANGE    1.8f    /* |h| outside ±RANGE clipped       */
#define ARROW_SPD_THRESH 0.05f   /* min speed to draw arrow          */
#define SHORE_THRESH     0.04f   /* |h| below this → shoreline check */

/* Boundary conditions */
enum {
    BC_WALL     = 0,   /* reflecting walls: normal velocity = 0      */
    BC_OPEN     = 1,   /* outgoing: Neumann copy — waves slide out   */
    BC_PERIODIC = 2,   /* torus: top↔bottom, left↔right              */
    BC_COUNT    = 3,
};
static const char *const k_bc_names[BC_COUNT] = {
    "wall", "open", "periodic"
};

/* Presets */
enum {
    PRESET_DAM_BREAK  = 0,
    PRESET_RADIAL_DROP = 1,
    PRESET_CHANNEL     = 2,
    PRESET_OBSTACLE    = 3,
    PRESET_COUNT       = 4,
};
static const char *const k_preset_names[PRESET_COUNT] = {
    "dam_break", "radial_drop", "channel", "obstacle"
};

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
/* §3  theme — height-field color pipeline; ASCII ramp; LUT              */
/* ===================================================================== */

/*
 * ASCII ramp — visual density ordered sparse → dense.
 * Both positive (crest) and negative (trough) amplitude use the same
 * ramp; sign is encoded by color (warm vs cool).
 *
 *   ' '  0%   at rest / undisturbed
 *   '.'  3%   tiny ripple
 *   ':'  9%   gentle swell
 *   '+'  20%  moderate wave
 *   'x'  34%  strong wave
 *   '*'  50%  intense surge
 *   'X'  66%  near-crest
 *   '#'  82%  high crest / deep trough
 *   '@'  94%  maximum (clipped)
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N  (int)(sizeof k_ramp - 1)   /* 9 levels */

static const float k_breaks[RAMP_N] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f,
    0.500f, 0.660f, 0.820f, 0.940f,
};

static int lut_index(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return RAMP_N - 1;
    float g = powf(v, 1.0f / 2.2f);   /* gamma correction */
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (g >= k_breaks[i]) return i;
    return 0;
}

/*
 * Themes — 3 palettes for signed height amplitude.
 *
 *   theme 0 "water"  — crests: cyan/white  |  troughs: deep blue/navy
 *   theme 1 "magma"  — crests: yellow/red  |  troughs: dark violet
 *   theme 2 "current"— crests: green/white |  troughs: dark teal
 *
 * Color pair layout:
 *   CP_POS(t,i) = positive (high water) at theme t, level i
 *   CP_NEG(t,i) = negative (low water)  at theme t, level i
 *   CP_SHORE    = shoreline glyph color
 *   CP_OBS      = obstacle color
 *   CP_HUD      = overlay / stats color
 */
#define N_THEMES    3
#define CP_POS(t,i) (1 + (t)*(RAMP_N*2) + (i))
#define CP_NEG(t,i) (1 + (t)*(RAMP_N*2) + RAMP_N + (i))
#define CP_SHORE    (1 + N_THEMES*(RAMP_N*2))
#define CP_OBS      (CP_SHORE + 1)
#define CP_HUD      (CP_OBS   + 1)

typedef struct {
    const char *name;
    int pos256[RAMP_N];
    int neg256[RAMP_N];
    int pos8  [RAMP_N];
    int neg8  [RAMP_N];
} SWTheme;

static const SWTheme k_themes[N_THEMES] = {
    {   /* 0  water: deep blue → cyan (crests), navy (troughs) */
        "water",
        /* pos: dark teal → cyan → white */
        {  23,  30,  37,  44,  51,  87, 123, 159, 231 },
        /* neg: navy → midnight → dark indigo */
        {  17,  18,  19,  20,  21,  22,  23,  24,  25 },
        {  COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN,
           COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
        {  COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE,
           COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN },
    },
    {   /* 1  magma: yellow/red crests, dark violet troughs */
        "magma",
        /* pos: dark orange → amber → yellow → white */
        {  52,  88, 130, 166, 202, 208, 214, 220, 231 },
        /* neg: dark violet → indigo → blue */
        {  53,  54,  55,  56,  57,  93,  99, 135, 141 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE, COLOR_CYAN },
    },
    {   /* 2  current: green/white crests, dark green troughs */
        "current",
        /* pos: dark green → lime → white */
        {  22,  28,  34,  40,  46,  82, 118, 154, 231 },
        /* neg: dark teal → dark green */
        {  17,  22,  23,  28,  29,  30,  36,  22,  28 },
        {  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
           COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,
           COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,  COLOR_CYAN },
    },
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    for (int t = 0; t < N_THEMES; t++) {
        for (int i = 0; i < RAMP_N; i++) {
            if (COLORS >= 256) {
                init_pair(CP_POS(t,i), k_themes[t].pos256[i], COLOR_BLACK);
                init_pair(CP_NEG(t,i), k_themes[t].neg256[i], COLOR_BLACK);
            } else {
                init_pair(CP_POS(t,i), k_themes[t].pos8[i], COLOR_BLACK);
                init_pair(CP_NEG(t,i), k_themes[t].neg8[i], COLOR_BLACK);
            }
        }
    }
    /* Shoreline: bright white/cyan */
    if (COLORS >= 256)
        init_pair(CP_SHORE, 51, COLOR_BLACK);
    else
        init_pair(CP_SHORE, COLOR_CYAN, COLOR_BLACK);
    /* Obstacle: dim gray */
    if (COLORS >= 256)
        init_pair(CP_OBS, 238, COLOR_BLACK);
    else
        init_pair(CP_OBS, COLOR_WHITE, COLOR_BLACK);
    /* HUD: amber */
    if (COLORS >= 256)
        init_pair(CP_HUD, 220, COLOR_BLACK);
    else
        init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
}

static attr_t height_attr(int theme, bool positive, int level)
{
    attr_t a = positive ? COLOR_PAIR(CP_POS(theme, level))
                        : COLOR_PAIR(CP_NEG(theme, level));
    if (level >= RAMP_N - 2) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  grid — static field arrays                                         */
/* ===================================================================== */

/*
 * g_h[r][c]   — height perturbation η = H_total - H₀.
 *               Positive: water above rest level (crest / dam side).
 *               Negative: water below rest level (trough / dry side).
 *
 * g_u[r][c]   — horizontal x-velocity (cells/s).
 *               Positive: flow rightward.  Negative: flow leftward.
 *               In the forward-difference stencil, g_u[r][c] conceptually
 *               sits at the right edge of cell (r,c) — between c and c+1.
 *
 * g_v[r][c]   — horizontal y-velocity (cells/s).
 *               Positive: flow downward (row increases).
 *               Negative: flow upward.
 *               Similarly, g_v[r][c] sits at the bottom edge of (r,c).
 *
 * g_obs[r][c] — obstacle mask.  true = solid cell.
 *               After each update, u=v=h=0 is enforced here.
 *               Waves reflect off obstacle boundaries.
 */
static float g_h  [GRID_MAX_H][GRID_MAX_W];
static float g_u  [GRID_MAX_H][GRID_MAX_W];
static float g_v  [GRID_MAX_H][GRID_MAX_W];
static bool  g_obs[GRID_MAX_H][GRID_MAX_W];

/* ===================================================================== */
/* §5  solver                                                             */
/* ===================================================================== */

static void field_zero(int cols, int rows)
{
    for (int r = 0; r < rows; r++) {
        memset(g_h[r], 0, (size_t)cols * sizeof(float));
        memset(g_u[r], 0, (size_t)cols * sizeof(float));
        memset(g_v[r], 0, (size_t)cols * sizeof(float));
    }
}

/*
 * apply_bc() — enforce boundary conditions on g_h, g_u, g_v.
 *
 * BC_WALL (reflecting):
 *   Normal velocity components are zeroed at each wall.  Height is copied
 *   from the nearest interior neighbor (Neumann: zero normal gradient).
 *   Incident wave energy reflects back; no energy is lost at the wall.
 *   The forward-difference stencil means u[r][0] and u[r][cols-1] are the
 *   flux-carrying terms — setting them to zero seals the wall.
 *
 * BC_OPEN (outgoing):
 *   All fields copy from the nearest interior neighbor.  This implements
 *   a simple Neumann (zero-gradient) outflow condition.  It is not perfectly
 *   non-reflecting — some energy returns — but it visually absorbs most of
 *   the wave energy at the boundary.
 *
 * BC_PERIODIC:
 *   Domain wraps around: leaving the right side re-enters on the left, etc.
 *   The total domain behaves like a torus.  No reflections occur at all.
 */
static void apply_bc(int bc, int cols, int rows)
{
    switch (bc) {

    case BC_WALL:
        for (int c = 0; c < cols; c++) {
            /* Top wall: v at top row is zero (no flow through top) */
            g_v[0][c]        = 0.0f;
            g_v[rows-1][c]   = 0.0f;
            g_u[0][c]        = g_u[1][c];
            g_u[rows-1][c]   = g_u[rows-2][c];
            g_h[0][c]        = g_h[1][c];
            g_h[rows-1][c]   = g_h[rows-2][c];
        }
        for (int r = 0; r < rows; r++) {
            /* Left/right wall: u at edges is zero (no flow through wall) */
            g_u[r][0]        = 0.0f;
            g_u[r][cols-1]   = 0.0f;
            g_v[r][0]        = g_v[r][1];
            g_v[r][cols-1]   = g_v[r][cols-2];
            g_h[r][0]        = g_h[r][1];
            g_h[r][cols-1]   = g_h[r][cols-2];
        }
        break;

    case BC_OPEN:
        /* Copy all fields from nearest interior row/col */
        for (int c = 0; c < cols; c++) {
            g_h[0][c]      = g_h[1][c];      g_u[0][c]      = g_u[1][c];
            g_v[0][c]      = g_v[1][c];
            g_h[rows-1][c] = g_h[rows-2][c]; g_u[rows-1][c] = g_u[rows-2][c];
            g_v[rows-1][c] = g_v[rows-2][c];
        }
        for (int r = 0; r < rows; r++) {
            g_h[r][0]      = g_h[r][1];      g_u[r][0]      = g_u[r][1];
            g_v[r][0]      = g_v[r][1];
            g_h[r][cols-1] = g_h[r][cols-2]; g_u[r][cols-1] = g_u[r][cols-2];
            g_v[r][cols-1] = g_v[r][cols-2];
        }
        break;

    case BC_PERIODIC:
        for (int c = 0; c < cols; c++) {
            g_h[0][c]      = g_h[rows-2][c]; g_h[rows-1][c] = g_h[1][c];
            g_u[0][c]      = g_u[rows-2][c]; g_u[rows-1][c] = g_u[1][c];
            g_v[0][c]      = g_v[rows-2][c]; g_v[rows-1][c] = g_v[1][c];
        }
        for (int r = 0; r < rows; r++) {
            g_h[r][0]      = g_h[r][cols-2]; g_h[r][cols-1] = g_h[r][1];
            g_u[r][0]      = g_u[r][cols-2]; g_u[r][cols-1] = g_u[r][1];
            g_v[r][0]      = g_v[r][cols-2]; g_v[r][cols-1] = g_v[r][1];
        }
        break;
    }
}

/*
 * apply_obstacles() — enforce solid-cell conditions after each update.
 *
 * Setting u=v=h=0 inside obstacle cells makes them act as impermeable
 * bodies.  The abrupt zeroing effectively creates a reflecting boundary
 * at every obstacle face — waves bounce off obstacle edges exactly as
 * they bounce off the domain walls under BC_WALL.
 *
 * Thicker obstacles produce crisper reflections; a 1-cell-wide wall can
 * cause mild diffraction (waves "leak" around single-cell corners).
 */
static void apply_obstacles(int cols, int rows)
{
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            if (g_obs[r][c]) {
                g_h[r][c] = 0.0f;
                g_u[r][c] = 0.0f;
                g_v[r][c] = 0.0f;
            }
}

/*
 * update_velocity() — advance u and v by one timestep dt.
 *
 * READS g_h (current height).  WRITES g_u, g_v (in-place).
 *
 * Discretisation: forward difference for the height gradient.
 *
 *   ∂h/∂x at (r,c) ≈ h[r][c+1] - h[r][c]   (no /dx since dx=1)
 *   ∂h/∂y at (r,c) ≈ h[r+1][c] - h[r][c]
 *
 * The forward-difference stencil means cell c sees the pressure from
 * its right/bottom neighbors.  Combined with the backward-difference
 * in update_height(), this forms an implicit half-cell stagger that
 * avoids the odd-even decoupling (checkerboard instability) that
 * plagues purely centered schemes on collocated grids.
 *
 * Damping multiplies velocity by (1 - γ·dt) each tick — models bottom
 * friction.  This is the exact solution of dv/dt = -γv over interval dt.
 */
static void update_velocity(float dt, float gravity, float damping,
                            int cols, int rows)
{
    float damp = 1.0f - damping * dt;
    if (damp < 0.0f) damp = 0.0f;

    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (g_obs[r][c]) continue;

            /*
             * Pressure gradient accelerates flow from high h to low h.
             * Note: g_h[r][c+1] > g_h[r][c]  →  dh/dx > 0  →  u decreases
             * (water flows from high to low, i.e., in the -x direction).
             * The minus sign in the SWE ∂u/∂t = -g ∂h/∂x captures this.
             */
            g_u[r][c] -= gravity * (g_h[r][c+1] - g_h[r][c]) * dt;
            g_v[r][c] -= gravity * (g_h[r+1][c] - g_h[r][c]) * dt;
            g_u[r][c] *= damp;
            g_v[r][c] *= damp;
        }
    }
}

/*
 * update_height() — advance h by one timestep dt.
 *
 * READS g_u, g_v (already updated by update_velocity this tick).
 * WRITES g_h (in-place).
 *
 * Discretisation: backward difference for velocity divergence.
 *
 *   ∂u/∂x at (r,c) ≈ u[r][c] - u[r][c-1]
 *   ∂v/∂y at (r,c) ≈ v[r][c] - v[r-1][c]
 *
 * Mass conservation:
 *   If u[r][c] > u[r][c-1]: more water leaving cell c rightward than
 *   entering from the left → divergence > 0 → h decreases.
 *   Conversely, convergence (more inflow than outflow) raises h.
 *   Summing over all cells: ∑h changes only at the domain boundary,
 *   and with BC_WALL the boundary flux is zero → perfect mass conservation.
 *
 * This loop MUST stay separate from update_velocity.
 * If merged, g_h[r][c+1] used in the forward-difference gradient of
 * update_velocity would already have been modified for the row above,
 * creating a data race that destroys numerical accuracy.
 */
static void update_height(float dt, int cols, int rows)
{
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (g_obs[r][c]) continue;

            /*
             * Divergence of velocity field (backward-difference):
             *   div_u = ∂u/∂x = u[r][c] - u[r][c-1]
             *   div_v = ∂v/∂y = v[r][c] - v[r-1][c]
             * Positive divergence → water flowing away → h drops.
             * Negative divergence → water converging → h rises.
             */
            float div_u = g_u[r][c] - g_u[r][c-1];
            float div_v = g_v[r][c] - g_v[r-1][c];
            g_h[r][c] -= H0 * (div_u + div_v) * dt;
        }
    }
}

/*
 * compute_stats() — derive overlay quantities from the current fields.
 *
 * max_height   — peak |h| over all non-obstacle cells; used to normalise display.
 * avg_velocity — mean |velocity| = mean of √(u²+v²); indicates flow intensity.
 * wave_speed   — c = √(g·H₀) = √gravity; propagation speed of small disturbances.
 * cfl          — c·dt; stability indicator (should be < 1/√2 ≈ 0.707).
 */
static void compute_stats(float gravity, float dt,
                          int cols, int rows,
                          float *max_height,
                          float *avg_velocity,
                          float *wave_speed,
                          float *cfl)
{
    float mx  = 0.0f;
    double sum_spd = 0.0;
    int    n_cells = 0;

    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (g_obs[r][c]) continue;
            float ah = fabsf(g_h[r][c]);
            if (ah > mx) mx = ah;
            float spd = sqrtf(g_u[r][c]*g_u[r][c] + g_v[r][c]*g_v[r][c]);
            sum_spd += (double)spd;
            n_cells++;
        }
    }

    *max_height   = mx;
    *avg_velocity = (n_cells > 0) ? (float)(sum_spd / n_cells) : 0.0f;
    *wave_speed   = sqrtf(gravity * H0);
    *cfl          = *wave_speed * dt;   /* CFL = c·dt/dx, dx=1 */
}

static void init_heightfield(int bc, int cols, int rows)
{
    field_zero(cols, rows);
    apply_bc(bc, cols, rows);
}

/* ===================================================================== */
/* §6  excitation — apply_drop, obstacle helpers, preset functions        */
/* ===================================================================== */

/*
 * apply_drop() — add a Gaussian height pulse at (cx, cy).
 *
 * ADDS to g_h (additive, so multiple drops accumulate).
 * g_u and g_v are not changed — this models an impulsive vertical
 * displacement rather than a momentum injection.
 *
 * The resulting pressure gradient then drives velocity in update_velocity
 * on the very next tick, launching a radially symmetric wave outward.
 */
static void apply_drop(float cx, float cy, float amp, float radius,
                       int cols, int rows)
{
    float inv2r2 = 1.0f / (2.0f * radius * radius);
    for (int r = 0; r < rows; r++) {
        float dy = (float)r - cy;
        float dy2 = dy * dy;
        for (int c = 0; c < cols; c++) {
            if (g_obs[r][c]) continue;
            float dx = (float)c - cx;
            g_h[r][c] += amp * expf(-(dx*dx + dy2) * inv2r2);
        }
    }
}

/* ── obstacle setup ─────────────────────────────────────────────────── */

static void obs_clear(int cols, int rows)
{
    for (int r = 0; r < rows; r++)
        memset(g_obs[r], 0, (size_t)cols * sizeof(bool));
}

/*
 * obs_channel() — two horizontal walls forming a narrow east-west channel.
 *
 * The walls occupy the upper and lower thirds of the domain, spanning the
 * middle half of the width.  This leaves a gap in the centre through which
 * waves must squeeze — demonstrating diffraction as the wavefront spreads
 * outward in a semicircle after passing through the opening.
 */
static void obs_channel(int cols, int rows)
{
    int wall_top = rows / 3;
    int wall_bot = (rows * 2) / 3;
    int left  = cols / 4;
    int right = (cols * 3) / 4;

    for (int r = 0; r < rows; r++) {
        if (r < wall_top || r > wall_bot) {
            for (int c = left; c <= right; c++)
                g_obs[r][c] = true;
        }
    }
}

/*
 * obs_circle() — solid circular obstacle centred in the domain.
 *
 * Radius is 12% of the smaller domain dimension — large enough to cast
 * a visible shadow (diffraction shadow) behind it, yet small enough to
 * leave clear paths on both sides for passing waves.
 */
static void obs_circle(int cols, int rows)
{
    float cx = (float)(cols - 1) * 0.5f;
    float cy = (float)(rows - 1) * 0.5f;
    float rad = (float)(rows < cols ? rows : cols) * 0.12f;
    float rad2 = rad * rad;

    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            float dx = (float)c - cx, dy = (float)r - cy;
            if (dx*dx + dy*dy < rad2) g_obs[r][c] = true;
        }
}

/* ── preset application ─────────────────────────────────────────────── */

typedef struct Scene Scene;   /* forward declaration */
static void preset_apply(Scene *s, int id, int cols, int rows);

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/*
 * Arrow direction LUT — 8 compass directions, chosen by velocity octant.
 *
 * The octant is computed from atan2(vy, vx): angle ∈ [0, 2π) divided into
 * 8 equal sectors of π/4 radians each.  The index order matches:
 *   0 = E (right), 1 = SE, 2 = S (down), 3 = SW,
 *   4 = W (left),  5 = NW, 6 = N (up),  7 = NE.
 * Terminal y increases downward, so 'v' (S) corresponds to positive vy.
 */
static const char k_arrows[8] = { '>', '/', 'v', '\\', '<', '/', '^', '\\' };

/*
 * render_heightmap() — draw h, u, v fields into ncurses window w.
 *
 * Rendering layers (applied in order, each may override the previous):
 *
 *   1. Height shading: |h| → character density, sign → color temperature.
 *      Crests (h>0) in warm colors; troughs (h<0) in cool colors.
 *      Matches the signed-amplitude encoding used in the membrane solver.
 *
 *   2. Shoreline detection (show_shore): cells where |h| < SHORE_THRESH
 *      AND an adjacent cell has opposite sign of h.  These zero-crossings
 *      mark the physical waterline / wavefront boundary.  Drawn as '~'
 *      in bright cyan to make the wavefront trajectory visible.
 *
 *   3. Flow arrows (show_arrows): cells where |velocity| > ARROW_SPD_THRESH
 *      have their height glyph replaced by a direction arrow.  The arrow
 *      shows the instantaneous flow direction — useful for seeing the
 *      pressure-driven flow pattern during dam break or reflection events.
 *
 *   4. Obstacles: always rendered as '░' in dim gray, regardless of h.
 *      They are solid and impermeable; no height or velocity data applies.
 */
static void render_heightmap(WINDOW *w, int cols, int rows,
                             int theme, bool show_arrows, bool show_shore,
                             float display_max)
{
    if (display_max < 0.05f) display_max = 0.05f;
    float inv_max = 1.0f / display_max;
    float shore_thresh = SHORE_THRESH;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {

            /* ── Obstacle ─────────────────────────────────────────── */
            if (g_obs[r][c]) {
                wattron(w, COLOR_PAIR(CP_OBS) | A_DIM);
                mvwaddch(w, r, c, '#');
                wattroff(w, COLOR_PAIR(CP_OBS) | A_DIM);
                continue;
            }

            float h = g_h[r][c];

            /* ── Shoreline detection ──────────────────────────────── *
             * A wavefront / shoreline exists where h crosses zero.
             * We check if this cell is near-zero while its neighbours
             * have opposite signs — the zero-crossing is between them.  */
            if (show_shore && fabsf(h) < shore_thresh) {
                bool near_pos = false, near_neg = false;
                if (r > 0 && !g_obs[r-1][c]) {
                    near_pos |= (g_h[r-1][c] > shore_thresh);
                    near_neg |= (g_h[r-1][c] < -shore_thresh);
                }
                if (r < rows-1 && !g_obs[r+1][c]) {
                    near_pos |= (g_h[r+1][c] > shore_thresh);
                    near_neg |= (g_h[r+1][c] < -shore_thresh);
                }
                if (c > 0 && !g_obs[r][c-1]) {
                    near_pos |= (g_h[r][c-1] > shore_thresh);
                    near_neg |= (g_h[r][c-1] < -shore_thresh);
                }
                if (c < cols-1 && !g_obs[r][c+1]) {
                    near_pos |= (g_h[r][c+1] > shore_thresh);
                    near_neg |= (g_h[r][c+1] < -shore_thresh);
                }
                if (near_pos && near_neg) {
                    wattron(w, COLOR_PAIR(CP_SHORE) | A_BOLD);
                    mvwaddch(w, r, c, '~');
                    wattroff(w, COLOR_PAIR(CP_SHORE) | A_BOLD);
                    continue;
                }
                continue;   /* near-zero, no crossing → skip (background) */
            }

            /* ── Height shading ───────────────────────────────────── */
            bool positive = (h >= 0.0f);
            float norm    = fabsf(h) * inv_max;
            if (norm > 1.0f) norm = 1.0f;

            int lvl = lut_index(norm);
            if (lvl == 0) continue;

            attr_t attr = height_attr(theme, positive, lvl);

            /* ── Flow arrow overlay ───────────────────────────────── *
             * Arrow replaces the height glyph at fast-moving cells.
             * atan2 returns angle in (-π, π]; we shift to [0, 2π).
             * Dividing by π/4 gives the octant index 0..7.           */
            if (show_arrows) {
                float spd = sqrtf(g_u[r][c]*g_u[r][c] + g_v[r][c]*g_v[r][c]);
                if (spd > ARROW_SPD_THRESH) {
                    float ang = atan2f(g_v[r][c], g_u[r][c]);
                    if (ang < 0.0f) ang += 2.0f * (float)M_PI;
                    int oct = (int)(ang / ((float)M_PI * 0.25f)) % 8;
                    wattron(w, attr);
                    mvwaddch(w, r, c, k_arrows[oct]);
                    wattroff(w, attr);
                    continue;
                }
            }

            wattron(w, attr);
            mvwaddch(w, r, c, k_ramp[lvl]);
            wattroff(w, attr);
        }
    }
}

/*
 * render_overlay() — stats panel, bottom-left corner.
 */
static void render_overlay(WINDOW *w, int cols, int rows,
                           float max_height, float avg_vel,
                           float wave_spd, float cfl,
                           float gravity, float damping,
                           int bc, float sim_time,
                           bool paused, bool show_arrows,
                           bool show_shore, int preset_id)
{
    int pw = 30;
    int ph = 14;
    int ox = 1;
    int oy = rows - ph - 1;
    if (oy < 0) oy = 0;
    if (ox + pw > cols) return;

    /* CFL stability: green < 0.50, yellow < 0.70, red ≥ 0.70 */
    int         cfl_color;
    const char *cfl_label;
    if (cfl < 0.50f)      { cfl_color = CP_NEG(0, 5); cfl_label = "STABLE  "; }
    else if (cfl < 0.70f) { cfl_color = CP_HUD;        cfl_label = "MARGINAL"; }
    else                  { cfl_color = CP_POS(1, 7);   cfl_label = "UNSTABLE"; }

    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, oy+ 0, ox, "+--- SHALLOW WATER --------+");
    mvwprintw(w, oy+ 1, ox, "| max_h   %8.4f           |", max_height);
    mvwprintw(w, oy+ 2, ox, "| avg_vel %8.4f           |", avg_vel);
    mvwprintw(w, oy+ 3, ox, "| wave_spd%7.2f cells/s    |", wave_spd);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

    /* CFL row: colour-coded by stability */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, oy+ 4, ox, "| CFL     ");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wattron(w, COLOR_PAIR(cfl_color) | A_BOLD);
    wprintw(w, "%5.3f %-8s", cfl, cfl_label);
    wattroff(w, COLOR_PAIR(cfl_color) | A_BOLD);
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wprintw(w, "|");

    mvwprintw(w, oy+ 5, ox, "| gravity %7.1f            |", gravity);
    mvwprintw(w, oy+ 6, ox, "| damping %8.4f           |", damping);
    mvwprintw(w, oy+ 7, ox, "| BC      %-10s         |", k_bc_names[bc]);
    mvwprintw(w, oy+ 8, ox, "| sim_t   %8.2f s          |", sim_time);
    mvwprintw(w, oy+ 9, ox, "| preset  %-10s         |", k_preset_names[preset_id]);
    mvwprintw(w, oy+10, ox, "| arrows  %-3s shore %-3s      |",
              show_arrows ? "ON " : "OFF",
              show_shore  ? "ON " : "OFF");
    mvwprintw(w, oy+11, ox, "| %s                         |",
              paused ? "PAUSED        " : "running       ");
    mvwprintw(w, oy+12, ox, "+---------------------------+");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

struct Scene {
    /* physics */
    float   gravity;
    float   damping;
    int     bc;

    /* control */
    bool    paused;
    bool    step_requested;
    int     preset_id;

    /* visual */
    int     theme;
    bool    show_arrows;
    bool    show_shore;
    bool    show_obstacles;   /* render obstacle cells; obstacles still active */

    /* stats */
    float   max_height;
    float   avg_velocity;
    float   wave_speed;
    float   cfl;
    float   simulation_time;

    /* grid dimensions */
    int     cols;
    int     rows;
};

static void scene_reset(Scene *s)
{
    field_zero(s->cols, s->rows);
    apply_bc(s->bc, s->cols, s->rows);
    s->simulation_time = 0.0f;
    s->max_height      = 0.0f;
    s->avg_velocity    = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    init_heightfield(s->bc, cols, rows);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused && !s->step_requested) return;
    s->step_requested = false;
    s->simulation_time += dt;

    update_velocity(dt, s->gravity, s->damping, s->cols, s->rows);
    update_height  (dt, s->cols, s->rows);
    apply_obstacles(s->cols, s->rows);
    apply_bc(s->bc, s->cols, s->rows);

    compute_stats(s->gravity, dt,
                  s->cols, s->rows,
                  &s->max_height, &s->avg_velocity,
                  &s->wave_speed, &s->cfl);
}

static void scene_draw(const Scene *s, WINDOW *w, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    render_heightmap(w, s->cols, s->rows,
                     s->theme, s->show_arrows, s->show_shore,
                     s->max_height);

    render_overlay(w, s->cols, s->rows,
                   s->max_height, s->avg_velocity,
                   s->wave_speed, s->cfl,
                   s->gravity, s->damping,
                   s->bc, s->simulation_time,
                   s->paused, s->show_arrows,
                   s->show_shore, s->preset_id);
}

/* ── preset implementations ─────────────────────────────────────────── */

static void preset_apply(Scene *s, int id, int cols, int rows)
{
    obs_clear(cols, rows);
    scene_reset(s);
    s->preset_id = id;

    float cx = (float)(cols - 1) * 0.5f;
    float cy = (float)(rows - 1) * 0.5f;

    switch (id) {

    case PRESET_DAM_BREAK:
        /*
         * Classic 1-D dam break in 2-D domain.
         *
         * At t=0 a vertical dam divides the domain: left half has water
         * at height H₀+DAM_AMP, right half at H₀-DAM_AMP (stored as ±DAM_AMP
         * perturbation with H₀ implicit).  When released, gravity-driven flow
         * builds at the dam face.  A bore (shock) propagates rightward into
         * the undisturbed region; a rarefaction wave travels leftward into
         * the raised region.
         *
         * In the linearised SWE the bore speed equals c = √(gH₀) exactly.
         * In the nonlinear case the bore is slightly faster; watch the CFL
         * readout — if the bore speed exceeds c, the solver may go unstable.
         */
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                g_h[r][c] = (c < cols / 2) ? DAM_AMP : -DAM_AMP;
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_RADIAL_DROP:
        /*
         * Gaussian height pulse at the domain centre.
         *
         * The symmetric initial condition launches a circular wave ring that
         * expands outward at wave speed c = √g.  With BC_WALL, the ring
         * reflects at all four walls and returns as four arcs converging on
         * the centre — the reflected pulses arrive simultaneously and produce
         * a sharp focus peak at the origin.  With BC_OPEN, energy exits and
         * the pattern is simpler.
         */
        apply_drop(cx, cy, DROP_AMP, DROP_RADIUS, cols, rows);
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_CHANNEL:
        /*
         * Two horizontal obstacle walls forming a narrow east-west channel.
         *
         * A radial drop is placed to the left of the channel entrance.
         * Waves funnel through the gap and diffract on the far side —
         * a single opening acts as a point source in Huygens' principle.
         * The semicircular diffraction pattern on the exit side is clearly
         * visible.  The gap width relative to the wavelength λ = c/f
         * controls the diffraction angle: narrow gap → wide spread.
         */
        obs_channel(cols, rows);
        apply_drop((float)(cols - 1) * 0.20f, cy,
                   DROP_AMP, DROP_RADIUS, cols, rows);
        apply_obstacles(cols, rows);
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_OBSTACLE:
        /*
         * Solid circular obstacle at the centre; source drop on the left.
         *
         * Waves strike the obstacle and scatter in all directions.  Behind
         * the obstacle a "shadow zone" appears where amplitude is reduced.
         * Two symmetric diffraction arcs wrap around the obstacle and
         * interfere with each other behind it — constructive interference
         * on the axis of symmetry (directly behind the disc) and destructive
         * at oblique angles.
         *
         * This demonstrates the same physics as sound diffraction around
         * a column or light around a wire in Babinet's principle.
         */
        obs_circle(cols, rows);
        apply_drop((float)(cols - 1) * 0.15f, cy,
                   DROP_AMP, DROP_RADIUS, cols, rows);
        apply_obstacles(cols, rows);
        apply_bc(s->bc, cols, rows);
        break;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->gravity      = GRAVITY_DEFAULT;
    s->damping      = DAMPING_DEFAULT;
    s->bc           = BC_WALL;
    s->theme        = 0;
    s->show_arrows  = true;
    s->show_shore   = true;
    s->show_obstacles = true;
    s->cols         = cols;
    s->rows         = rows;
    init_heightfield(s->bc, cols, rows);
    preset_apply(s, PRESET_DAM_BREAK, cols, rows);
}

/* ===================================================================== */
/* §9  screen — ncurses double-buffer display layer                      */
/* ===================================================================== */

typedef struct { int cols; int rows; } Screen;

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

static void screen_free(Screen *s) { (void)s; endwin(); }

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
    scene_draw(sc, stdscr, alpha, dt_sec);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  "
             "SWE  g=%.0f  c=%.1f  BC=%-8s ",
             fps, sim_fps,
             sc->gravity, sc->wave_speed,
             k_bc_names[sc->bc]);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit spc:pause s:step r:reset "
             "d:drop b:dam g/G:gravity± a:arrows l:shore "
             "n:BC o:obs p:preset t:theme [/]:Hz ");
    attroff(A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10 app — signals, resize, input, main loop                           */
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
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — dispatch all user input.
 *
 * Groups:
 *   flow control  — q, ESC, space, s
 *   excitation    — d (radial drop), b (dam break reset)
 *   physics       — g/G (gravity), n (BC cycle)
 *   simulation    — r (reset), p/P (preset cycle), o (obstacles)
 *   visual        — a (arrows), l (shoreline), t (theme), [/] (sim Hz)
 */
static bool app_handle_key(App *app, int ch)
{
    Scene  *s  = &app->scene;
    Screen *sc = &app->screen;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 's': case 'S':
        s->paused         = true;
        s->step_requested = true;
        break;

    /* ── excitation ─────────────────────────────────────────────── */
    case 'd': case 'D':
        /* Additive radial drop at centre — works even while paused */
        apply_drop((float)(sc->cols-1)*0.5f, (float)(sc->rows-1)*0.5f,
                   DROP_AMP, DROP_RADIUS, sc->cols, sc->rows);
        apply_obstacles(sc->cols, sc->rows);
        apply_bc(s->bc, sc->cols, sc->rows);
        break;

    case 'b': case 'B':
        /* Re-apply dam break on current grid (keeps obstacles) */
        field_zero(sc->cols, sc->rows);
        for (int r = 0; r < sc->rows; r++)
            for (int c = 0; c < sc->cols; c++)
                if (!g_obs[r][c])
                    g_h[r][c] = (c < sc->cols / 2) ? DAM_AMP : -DAM_AMP;
        apply_obstacles(sc->cols, sc->rows);
        apply_bc(s->bc, sc->cols, sc->rows);
        s->simulation_time = 0.0f;
        break;

    /* ── gravity ────────────────────────────────────────────────── *
     * Gravity sets the wave speed: c = √g.
     * Raising g → faster waves → shorter CFL margin.
     * Lowering g → slower, more languid waves.                     */
    case 'g':
        s->gravity += GRAVITY_STEP;
        if (s->gravity > GRAVITY_MAX) s->gravity = GRAVITY_MAX;
        break;
    case 'G':
        s->gravity -= GRAVITY_STEP;
        if (s->gravity < GRAVITY_MIN) s->gravity = GRAVITY_MIN;
        break;

    /* ── boundary conditions ────────────────────────────────────── */
    case 'n': case 'N':
        s->bc = (s->bc + 1) % BC_COUNT;
        preset_apply(s, s->preset_id, sc->cols, sc->rows);
        break;

    /* ── reset / presets ────────────────────────────────────────── */
    case 'r': case 'R':
        preset_apply(s, s->preset_id, sc->cols, sc->rows);
        break;

    case 'p':
        preset_apply(s, (s->preset_id + 1) % PRESET_COUNT,
                     sc->cols, sc->rows);
        break;
    case 'P':
        preset_apply(s, (s->preset_id + PRESET_COUNT - 1) % PRESET_COUNT,
                     sc->cols, sc->rows);
        break;

    /* ── obstacles toggle ───────────────────────────────────────── *
     * Cycles obstacle display.  The obstacles remain physically active
     * (they still block flow) even when rendered transparently, so
     * pressing 'o' is purely a visual aid, not a physics toggle.   */
    case 'o': case 'O':
        s->show_obstacles = !s->show_obstacles;
        break;

    /* ── visual ─────────────────────────────────────────────────── */
    case 'a': case 'A':
        s->show_arrows = !s->show_arrows;
        break;

    case 'l': case 'L':
        s->show_shore = !s->show_shore;
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % N_THEMES;
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
 * main() — fixed-timestep accumulator loop (same pattern as framework.c)
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

    Screen *sc = &app->screen;
    Scene  *s  = &app->scene;

    screen_init(sc);
    scene_init(s, sc->cols, sc->rows);

    /* FPS measurement */
    int64_t fps_window_start = clock_ns();
    int     fps_frame_count  = 0;
    double  fps_display      = 0.0;

    /* Fixed-timestep accumulator */
    int64_t sim_accum  = 0;
    int64_t last_frame = clock_ns();

    while (app->running) {
        if (app->need_resize) app_do_resize(app);

        /* ── Input ──────────────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }
        if (!app->running) break;

        /* ── Timing ─────────────────────────────────────────────── */
        int64_t now      = clock_ns();
        int64_t frame_dt = now - last_frame;
        last_frame = now;
        if (frame_dt > 100 * NS_PER_MS) frame_dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += frame_dt;

        /* ── Physics ────────────────────────────────────────────── */
        float dt = 1.0f / (float)app->sim_fps;
        while (sim_accum >= tick_ns) {
            scene_tick(s, dt);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS measurement ────────────────────────────────────── */
        fps_frame_count++;
        int64_t fps_elapsed = now - fps_window_start;
        if (fps_elapsed >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display      = (double)fps_frame_count /
                               ((double)fps_elapsed / (double)NS_PER_SEC);
            fps_frame_count  = 0;
            fps_window_start = now;
        }

        /* ── Render ─────────────────────────────────────────────── */
        screen_draw(sc, s, fps_display, app->sim_fps, alpha, dt);
        screen_present();

        /* ── Sleep until next frame ─────────────────────────────── */
        int64_t target_ns = NS_PER_SEC / TARGET_FPS;
        int64_t elapsed   = clock_ns() - now;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(sc);
    return 0;
}
