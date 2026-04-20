/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/membrane.c — 2-D Vibrating Membrane Wave Equation Simulator
 *
 * Solves the damped scalar wave equation on the terminal grid:
 *
 *   ∂²u/∂t² = c² ∇²u − γ ∂u/∂t
 *
 * using an explicit 5-point finite-difference Laplacian and a
 * symplectic (velocity-field) time integrator.  The terminal itself
 * IS the simulation grid — one terminal cell = one grid point.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *  §1  config      — all tunable constants
 *  §2  clock       — monotonic nanosecond clock + sleep
 *  §3  theme       — signed-amplitude color pipeline; ASCII ramp; LUT
 *  §4  grid        — static field arrays (height + velocity)
 *  §5  solver      — init_grid, update_wave, apply_bc, compute_stats
 *  §6  excitation  — apply_excitation, preset functions
 *  §7  render      — render_membrane, render_overlay
 *  §8  scene       — Scene struct, scene_init/tick/draw/reset
 *  §9  screen      — ncurses double-buffer display layer
 *  §10 app         — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q/ESC   quit          space   pause/resume     s     single step
 *   r       reset         b       center strike     e     edge strike
 *   f       double strike m       resonance mode    l     nodal lines
 *   c/C     wave speed±   d/D     damping±          n     cycle BC
 *   p/P     cycle preset  t       cycle theme       ]/[   sim Hz±
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/membrane.c \
 *       -o membrane -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Wave equation (§5 update_wave):
 *   ∂²u/∂t² = c² ∇²u − γ ∂u/∂t
 *   u(x,y,t) — vertical displacement of the membrane at point (x,y)
 *   c         — wave speed [cells / second]; higher → faster propagation
 *   ∇²u       — 2-D Laplacian; measures how curved u is at each point
 *   γ         — damping coefficient [1/s]; converts kinetic energy to heat
 *
 * Finite difference stencil (§5):
 *   The 5-point Laplacian approximates ∇²u at interior grid point (r,c):
 *
 *              u[r-1][c]
 *                  |
 *   u[r][c-1] — u[r][c] — u[r][c+1]
 *                  |
 *              u[r+1][c]
 *
 *   ∇²u[r,c] ≈ u[r-1,c] + u[r+1,c] + u[r,c-1] + u[r,c+1] − 4·u[r,c]
 *
 *   With grid spacing dx = dy = 1 cell, the /dx² factor is 1 and drops out.
 *   Truncation error is O(dx²) — second-order accurate in space.
 *   The stencil touches only 4 neighbours so boundary handling is simple.
 *
 * Velocity-field time integrator (§5):
 *   We maintain an explicit velocity field v = ∂u/∂t alongside u.
 *   At each timestep dt:
 *
 *     v[r,c] += ( c² · ∇²u[r,c]  −  γ · v[r,c] ) · dt   ← force on membrane
 *     u[r,c] += v[r,c] · dt                               ← displacement update
 *
 *   Updating v BEFORE u (symplectic Euler) gives better energy conservation
 *   than updating both simultaneously (explicit Euler).  For undamped waves,
 *   the total mechanical energy stays nearly constant over thousands of steps.
 *
 * Wave speed meaning:
 *   c is the speed at which small disturbances travel across the grid.
 *   Physically: c = √(T/ρ) where T = membrane tension, ρ = area density.
 *   A pulse initiated at the center reaches the wall in (W/2)/c seconds.
 *   Example: c=25, W=80 → first reflection arrives after 80/(2·25) = 1.6 s.
 *
 * CFL stability condition (§5, §7):
 *   The explicit scheme is conditionally stable.  For 2-D with dx=dy=1:
 *     CFL_2D = c · dt · √2  must satisfy  CFL_2D ≤ 1
 *   If CFL_2D > 1, errors grow exponentially — the simulation "blows up".
 *   The overlay displays CFL_2D and flags STABLE / MARGINAL / UNSTABLE.
 *   Default: c=25, dt=1/60 → CFL_2D = 25·(1/60)·√2 ≈ 0.589  ✓
 *
 * Boundary conditions (§5 apply_bc):
 *   DIRICHLET — u=0 at all 4 edges.  Models a clamped drumhead rim.
 *               Wave reflects with inversion (phase reversal).
 *               Supports clear standing wave modes.
 *   NEUMANN   — ∂u/∂n=0 (zero normal gradient) at edges.
 *               Models a free membrane edge or an absorbing wall.
 *               Wave reflects WITHOUT inversion (no phase reversal).
 *               Implemented by copying adjacent interior values to the border.
 *   PERIODIC  — top wraps to bottom, left wraps to right.
 *               No reflections at all — wave travels as on a torus.
 *
 * Nodal lines (§7):
 *   In a standing wave, nodal lines are the curves where u = 0 at all times.
 *   They separate regions oscillating in opposite phase (+ vs -).
 *   We detect them by checking sign changes between adjacent cells: if
 *   u[r][c] and u[r][c+1] have opposite signs, the zero crossing lies between.
 *   We mark these cells with a neutral glyph ('-' or '|') so the mode shape
 *   is visible even when positive and negative regions have similar brightness.
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

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
    SIM_FPS_MIN      =   5,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =   5,

    TARGET_FPS       =  60,
    FPS_UPDATE_MS    = 500,
    HUD_COLS         =  80,
};

/* ── grid limits ────────────────────────────────────────────────────── */
/*
 * Physics runs directly in terminal-cell coordinates.  Unlike particle
 * simulations (which use a separate pixel space), the membrane grid IS
 * the character grid: g_u[row][col] is drawn at terminal cell (col, row).
 *
 * GRID_MAX_W × GRID_MAX_H × 2 fields × 4 bytes = 300×100×8 = 240 KB (BSS).
 */
#define GRID_MAX_W   300
#define GRID_MAX_H   100

/* ── wave physics defaults ──────────────────────────────────────────── */
/*
 * WAVE_SPEED is in cells/second.  The CFL stability limit at 60 Hz is:
 *   c_max = 1 / (dt · √2) = 60 / √2 ≈ 42 cells/s.
 * Default 25 gives CFL_2D ≈ 0.59 — comfortable margin.
 */
#define WAVE_SPEED_DEFAULT  25.0f
#define WAVE_SPEED_MIN       5.0f
#define WAVE_SPEED_MAX      42.0f   /* hard limit: CFL_2D → 1 at 60 Hz */
#define WAVE_SPEED_STEP      3.0f

#define DAMPING_DEFAULT     0.003f  /* gentle decay; τ ≈ 1/γ ≈ 333 ticks */
#define DAMPING_MIN         0.000f
#define DAMPING_MAX         0.060f
#define DAMPING_STEP        0.003f

/* ── excitation ─────────────────────────────────────────────────────── */
#define EXCITE_AMP          1.2f   /* peak amplitude of a strike         */
#define EXCITE_RADIUS       3.5f   /* Gaussian half-width in cells       */
#define RESONANCE_AMP       1.0f   /* amplitude for mode-shape presets   */

/* ── rendering ──────────────────────────────────────────────────────── */
#define NODAL_SIGN_THRESH   0.015f /* |u| below this treated as near-zero */
#define DISPLAY_RANGE       1.5f   /* u values outside ±DISPLAY_RANGE clipped */

/* ── boundary conditions ────────────────────────────────────────────── */
enum {
    BC_DIRICHLET = 0,   /* clamped rim: u=0 at edges                    */
    BC_NEUMANN   = 1,   /* free edge:   du/dn=0 at edges                */
    BC_PERIODIC  = 2,   /* torus:       top↔bottom, left↔right          */
    BC_COUNT     = 3,
};
static const char *const k_bc_names[BC_COUNT] = {
    "dirichlet", "neumann", "periodic"
};

/* ── presets ────────────────────────────────────────────────────────── */
enum {
    PRESET_CENTER    = 0,
    PRESET_EDGE      = 1,
    PRESET_DOUBLE    = 2,
    PRESET_RESONANCE = 3,
    PRESET_COUNT     = 4,
};
static const char *const k_preset_names[PRESET_COUNT] = {
    "center", "edge", "double", "resonance"
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
/* §3  theme — signed-amplitude color pipeline; ASCII ramp; LUT          */
/* ===================================================================== */

/*
 * ASCII ramp — characters ordered by visual ink density (sparse → dense).
 * Used identically for positive and negative amplitude; color encodes sign.
 *
 *   ' '   0%   at rest / nodal
 *   '.'   3%   barely displaced
 *   ':'  10%   slight ripple
 *   '+'  22%   medium wave
 *   'x'  38%   strong wave
 *   '*'  55%   intense oscillation
 *   'X'  72%   near-peak
 *   '#'  87%   peak displacement
 *   '@'  96%   maximum (clipped)
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N  (int)(sizeof k_ramp - 1)   /* 9 levels */

/* LUT breakpoints on normalised |amplitude| ∈ [0, 1] after gamma correction */
static const float k_breaks[RAMP_N] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f,
    0.500f, 0.660f, 0.820f, 0.940f,
};

static int lut_index(float v)
{
    /* v is already in [0,1]; gamma correct for perceptual uniformity */
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return RAMP_N - 1;
    float g = powf(v, 1.0f / 2.2f);
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (g >= k_breaks[i]) return i;
    return 0;
}

/*
 * Themes — 3 palettes for signed amplitude.
 *
 * Each palette has RAMP_N fg colors for POSITIVE amplitude (crest)
 * and RAMP_N fg colors for NEGATIVE amplitude (trough).
 *
 *   theme 0  "wave"    — blue troughs  ↔  red/orange crests (classical)
 *   theme 1  "thermal" — violet troughs ↔ yellow/white crests
 *   theme 2  "ocean"   — deep-blue troughs ↔ cyan/white crests
 *
 * Color pair layout (all themes defined at startup, switched by index):
 *   theme t, positive level i : CP_POS(t,i) = 1 + t*(RAMP_N*2) + i
 *   theme t, negative level i : CP_NEG(t,i) = 1 + t*(RAMP_N*2) + RAMP_N + i
 *   nodal line marker         : CP_NODAL = 1 + N_THEMES*(RAMP_N*2)
 *   HUD / overlay             : CP_HUD   = CP_NODAL + 1
 *
 * With 3 themes × 9 levels × 2 signs = 54 pairs + 2 = 56 pairs total.
 */
#define N_THEMES    3
#define CP_POS(t,i) (1 + (t)*(RAMP_N*2) + (i))
#define CP_NEG(t,i) (1 + (t)*(RAMP_N*2) + RAMP_N + (i))
#define CP_NODAL    (1 + N_THEMES*(RAMP_N*2))
#define CP_HUD      (CP_NODAL + 1)

typedef struct {
    const char *name;
    int pos256[RAMP_N];   /* 256-colour foreground for each crest level     */
    int neg256[RAMP_N];   /* 256-colour foreground for each trough level    */
    int pos8  [RAMP_N];   /* 8-colour fallback, crests                      */
    int neg8  [RAMP_N];   /* 8-colour fallback, troughs                     */
} WaveTheme;

static const WaveTheme k_themes[N_THEMES] = {
    {   /* 0  wave — blue troughs, red/yellow crests */
        "wave",
        /* pos: dark red → orange → yellow → white */
        {  52,  88, 124, 160, 196, 202, 208, 220, 231 },
        /* neg: dark blue → blue → cyan → white     */
        {  17,  19,  21,  27,  33,  39,  45,  51, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
        {  COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,   COLOR_BLUE,
           COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,  COLOR_WHITE },
    },
    {   /* 1  thermal — violet troughs, yellow crests */
        "thermal",
        /* pos: dark orange → amber → yellow → white */
        {  52,  94, 130, 166, 202, 208, 214, 220, 231 },
        /* neg: dark violet → magenta → pink         */
        {  53,  54,  91,  92, 129, 165, 201, 207, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE, COLOR_WHITE },
    },
    {   /* 2  ocean — deep-blue troughs, cyan/white crests */
        "ocean",
        /* pos: teal → cyan → white */
        {  23,  29,  36,  43, 51, 87, 123, 159, 231 },
        /* neg: navy → midnight blue → indigo        */
        {  17,  18,  19,  20, 21, 27,  33,  39,  45 },
        {  COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN,   COLOR_CYAN,
           COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE },
        {  COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
           COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN },
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
    /* Nodal line: dim gray (or dim yellow fallback) */
    if (COLORS >= 256)
        init_pair(CP_NODAL, 240, COLOR_BLACK);   /* dark gray */
    else
        init_pair(CP_NODAL, COLOR_WHITE, COLOR_BLACK);
    /* HUD */
    if (COLORS >= 256)
        init_pair(CP_HUD, 220, COLOR_BLACK);   /* amber */
    else
        init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
}

/* Return ncurses attribute for (theme, sign, ramp_level). */
static attr_t wave_attr(int theme, bool positive, int level)
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
 * Two fields, both in BSS (zero-initialised by the OS at startup).
 * No heap allocation anywhere in the simulation update path.
 *
 * g_u[row][col] — displacement field u(x,y,t).
 *                 Positive = membrane is above rest plane.
 *                 Negative = below rest plane.
 *
 * g_v[row][col] — velocity field ∂u/∂t.
 *                 Updated first each tick (symplectic Euler).
 *                 Positive = moving upward; negative = moving downward.
 *
 * Interior points: rows 1..rows-2, cols 1..cols-2.
 * Boundary points: row 0, row rows-1, col 0, col cols-1.
 * The solver only writes interior; apply_bc() handles boundary rows/cols.
 */
static float g_u[GRID_MAX_H][GRID_MAX_W];   /* displacement field */
static float g_v[GRID_MAX_H][GRID_MAX_W];   /* velocity field     */

/* ===================================================================== */
/* §5  solver                                                             */
/* ===================================================================== */

/*
 * init_grid() — zero both fields and apply initial boundary conditions.
 *
 * Called at startup and on reset.  No heap allocation.
 */
static void init_grid(int bc, int cols, int rows);   /* forward decl */

static void grid_zero(int cols, int rows)
{
    for (int r = 0; r < rows; r++) {
        memset(g_u[r], 0, (size_t)cols * sizeof(float));
        memset(g_v[r], 0, (size_t)cols * sizeof(float));
    }
}

/*
 * apply_bc() — enforce boundary conditions on g_u and g_v.
 *
 * Must be called AFTER every update_wave() call so the boundary cells
 * are always consistent with the chosen policy.
 *
 * DIRICHLET:
 *   u = v = 0 at all 4 borders.  This models a rigid rim that does not
 *   move.  Incident waves reflect with a 180° phase flip (inversion).
 *   Physical analogy: drumhead clamped at the rim.
 *
 * NEUMANN (∂u/∂n = 0):
 *   Each border cell is set equal to its nearest interior neighbour.
 *   This gives zero normal gradient — the wave "slides" along the wall.
 *   Incident waves reflect WITHOUT phase flip.
 *   Physical analogy: free edge (e.g. a plate, not clamped).
 *
 * PERIODIC:
 *   Top and bottom rows, left and right columns are identified.
 *   There are no reflections at all — wave exits one side and
 *   re-enters from the opposite side, as on a torus.
 */
static void apply_bc(int bc, int cols, int rows)
{
    switch (bc) {

    case BC_DIRICHLET:
        /* Top and bottom rows: zero displacement and velocity */
        for (int c = 0; c < cols; c++) {
            g_u[0][c] = 0.0f;       g_v[0][c] = 0.0f;
            g_u[rows-1][c] = 0.0f;  g_v[rows-1][c] = 0.0f;
        }
        /* Left and right columns */
        for (int r = 0; r < rows; r++) {
            g_u[r][0] = 0.0f;       g_v[r][0] = 0.0f;
            g_u[r][cols-1] = 0.0f;  g_v[r][cols-1] = 0.0f;
        }
        break;

    case BC_NEUMANN:
        /* Top row copies row 1 (its only interior neighbour) */
        for (int c = 0; c < cols; c++) {
            g_u[0][c]      = g_u[1][c];      g_v[0][c]      = g_v[1][c];
            g_u[rows-1][c] = g_u[rows-2][c]; g_v[rows-1][c] = g_v[rows-2][c];
        }
        /* Left and right columns copy their interior neighbour */
        for (int r = 0; r < rows; r++) {
            g_u[r][0]      = g_u[r][1];      g_v[r][0]      = g_v[r][1];
            g_u[r][cols-1] = g_u[r][cols-2]; g_v[r][cols-1] = g_v[r][cols-2];
        }
        break;

    case BC_PERIODIC:
        /* Top ↔ second-from-bottom; bottom ↔ second-from-top */
        for (int c = 0; c < cols; c++) {
            g_u[0][c]      = g_u[rows-2][c]; g_v[0][c]      = g_v[rows-2][c];
            g_u[rows-1][c] = g_u[1][c];      g_v[rows-1][c] = g_v[1][c];
        }
        /* Left ↔ second-from-right; right ↔ second-from-left */
        for (int r = 0; r < rows; r++) {
            g_u[r][0]      = g_u[r][cols-2]; g_v[r][0]      = g_v[r][cols-2];
            g_u[r][cols-1] = g_u[r][1];      g_v[r][cols-1] = g_v[r][1];
        }
        break;
    }
}

/*
 * update_wave() — advance the wave equation by one fixed timestep dt.
 *
 * STEP 1 — velocity update (reads g_u, writes g_v):
 *   For every interior cell, compute the 5-point discrete Laplacian:
 *     L = u[r-1,c] + u[r+1,c] + u[r,c-1] + u[r,c+1] − 4·u[r,c]
 *   Note: dx=1, so L already approximates ∇²u without dividing by dx².
 *   The restoring force is c²·L; the damping force is −γ·v.
 *   Velocity is updated with their sum:
 *     v += (c²·L − γ·v) · dt
 *   This is safe to do in-place because STEP 2 has not touched g_u yet.
 *
 * STEP 2 — displacement update (reads g_v, writes g_u):
 *   u += v · dt   (use the newly updated v — symplectic Euler).
 *   These two loops must remain SEPARATE.  If they were merged into one
 *   loop, updating u[r,c] in step 2 would corrupt the Laplacian
 *   computation for u[r+1,c] in step 1 — a subtle race condition.
 *
 * STEP 3 — boundary conditions:
 *   Reset boundary rows/columns according to the chosen BC policy.
 *   Must run after both updates so the border is always consistent.
 *
 * Stability note:
 *   CFL_2D = c · dt · √2 must be < 1 for the scheme to be stable.
 *   If the user increases c or sim_fps decreases, the CFL indicator
 *   in the overlay turns red to warn of impending instability.
 *   In the truly unstable regime (CFL ≥ 1), amplitudes grow rapidly —
 *   the simulation will visually "explode" with huge values.
 */
static void update_wave(float dt, float wave_speed, float damping,
                        int bc, int cols, int rows)
{
    float c2 = wave_speed * wave_speed;   /* c² for the restoring term */

    /* STEP 1: velocity update — iterate only interior points */
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            /*
             * 5-point Laplacian stencil:
             *   Each neighbour contributes its displacement;
             *   the 4·u[r,c] term represents the "rest" influence.
             *   When all four neighbours have the same value as u[r,c],
             *   L=0 and no restoring force acts — the cell is at an extremum.
             *   When neighbours are higher, L>0 → upward push.
             *   When neighbours are lower, L<0 → downward pull.
             */
            float lap = g_u[r-1][c] + g_u[r+1][c]
                      + g_u[r][c-1] + g_u[r][c+1]
                      - 4.0f * g_u[r][c];

            /*
             * Force accumulation:
             *   restoring = c² · lap   (wave equation term; always present)
             *   drag      = −γ · v     (damping; removes energy gradually)
             * Total acceleration = restoring + drag.
             * Velocity is updated BEFORE position (symplectic Euler).
             */
            g_v[r][c] += (c2 * lap - damping * g_v[r][c]) * dt;
        }
    }

    /* STEP 2: displacement update — use the freshly computed v */
    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            g_u[r][c] += g_v[r][c] * dt;

    /* STEP 3: boundary conditions */
    apply_bc(bc, cols, rows);
}

/*
 * compute_stats() — derive overlay quantities from the current fields.
 *
 * max_amplitude — peak |u| across all cells; used to normalise the display.
 * energy_est    — total mechanical energy (kinetic + potential):
 *                   E = Σ [ ½v² + ½c²(∇u)² ]   (unnormalised, per-cell)
 *                 Computed via centred finite differences for the gradient.
 * mode_nx, ny   — estimated mode numbers for Dirichlet BC.
 *                 Count sign changes along the centre row and column.
 *                 For the (n,m) mode: crossing_count = n (or m) half-waves.
 * cfl_2d        — c · dt · √2; stability indicator.
 */
static void compute_stats(float wave_speed, float dt_sec,
                          int cols, int rows,
                          float *max_amplitude,
                          float *energy_est,
                          int *mode_nx, int *mode_ny,
                          float *cfl_2d)
{
    float mx  = 0.0f;
    double ke = 0.0, pe = 0.0;
    float c2  = wave_speed * wave_speed;

    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            float u = g_u[r][c];
            float v = g_v[r][c];
            float a = fabsf(u);
            if (a > mx) mx = a;

            /* Kinetic energy: ½ v² (mass = 1 per cell) */
            ke += 0.5 * (double)(v * v);

            /* Potential energy: ½ c² |∇u|²
             * Centred-difference gradient components */
            float gx = (g_u[r][c+1] - g_u[r][c-1]) * 0.5f;
            float gy = (g_u[r+1][c] - g_u[r-1][c]) * 0.5f;
            pe += 0.5 * c2 * (double)(gx*gx + gy*gy);
        }
    }

    *max_amplitude = mx;
    *energy_est    = (float)((ke + pe) / ((rows-2) * (cols-2)));
    *cfl_2d        = wave_speed * dt_sec * 1.41421356f;  /* c·dt·√2 */

    /* Mode estimation: zero-crossing count along centre row and column.
     * A zero crossing occurs when consecutive samples have opposite sign.
     * For the (nx, ny) mode: crossing count ≈ nx (or ny). */
    int cr = rows / 2, cc = cols / 2;
    int cx = 0, cy = 0;
    for (int c = 1; c < cols - 1; c++)
        if (g_u[cr][c-1] * g_u[cr][c] < 0.0f) cx++;
    for (int r = 1; r < rows - 1; r++)
        if (g_u[r-1][cc] * g_u[r][cc] < 0.0f) cy++;

    *mode_nx = (cx < 1) ? 1 : cx;
    *mode_ny = (cy < 1) ? 1 : cy;
}

static void init_grid(int bc, int cols, int rows)
{
    grid_zero(cols, rows);
    apply_bc(bc, cols, rows);
}

/* ===================================================================== */
/* §6  excitation                                                          */
/* ===================================================================== */

/*
 * apply_excitation() — add a Gaussian displacement pulse to g_u.
 *
 * The pulse shape is:
 *   Δu(x,y) = amp · exp( −[(x−cx)² + (y−cy)²] / (2·r²) )
 *
 * We ADD to the current field rather than replacing it so that multiple
 * strikes accumulate (useful for the double-strike preset).
 *
 * Only the displacement field g_u is perturbed; g_v is left unchanged.
 * This models an impulsive displacement (a drumstick hit) rather than
 * an impulse of momentum.  To model a momentum impulse, add to g_v instead.
 */
static void apply_excitation(float cx, float cy, float amp, float radius,
                             int cols, int rows)
{
    float inv_2r2 = 1.0f / (2.0f * radius * radius);
    for (int r = 0; r < rows; r++) {
        float dy = (float)r - cy;
        float dy2 = dy * dy;
        for (int c = 0; c < cols; c++) {
            float dx = (float)c - cx;
            float d2 = dx*dx + dy2;
            g_u[r][c] += amp * expf(-d2 * inv_2r2);
        }
    }
}

/*
 * preset_resonance() — initialise g_u to the (nx, ny) standing wave mode.
 *
 * For Dirichlet boundary conditions on a W×H domain, the eigenfunctions are:
 *   u(x,y) = A · sin(nx·π·x/W) · sin(ny·π·y/H)
 *
 * where nx, ny = 1, 2, 3, … are the mode numbers.
 * This directly excites a single standing wave mode instead of a
 * superposition of many (as a strike would produce).
 *
 * Setting g_v = 0 means the membrane starts at maximum displacement with
 * zero velocity — it will oscillate with period T = 2π / ω_mn where
 * ω_mn = c·π·√(nx²/W² + ny²/H²).
 */
static void preset_resonance(int nx, int ny, float amp, int cols, int rows)
{
    float kx = (float)nx * (float)M_PI / (float)(cols - 1);
    float ky = (float)ny * (float)M_PI / (float)(rows - 1);
    for (int r = 0; r < rows; r++) {
        float sy = sinf(ky * (float)r);
        for (int c = 0; c < cols; c++) {
            float sx = sinf(kx * (float)c);
            g_u[r][c] = amp * sx * sy;
            g_v[r][c] = 0.0f;
        }
    }
}

/* ── preset application helpers ─────────────────────────────────────── */

typedef struct Scene Scene;   /* forward declaration for preset callbacks */

static void preset_apply(Scene *s, int preset_id, int cols, int rows);

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/*
 * render_membrane() — draw g_u into the ncurses window w.
 *
 * For each cell (r, c):
 *   1. Read u = g_u[r][c].
 *   2. Normalise: n = u / display_max  ∈ [-1, 1] (clamped).
 *   3. Map |n| → ramp level via LUT (gamma-corrected).
 *   4. Sign of u → choose positive (warm) or negative (cool) color pair.
 *   5. If show_nodal: detect sign changes between adjacent cells and mark
 *      them with a neutral glyph to highlight nodal lines.
 *
 * Visual encoding choices:
 *   • Signed amplitude → colour: crests (u>0) in warm colours, troughs
 *     (u<0) in cool colours.  This makes the wave phase immediately visible
 *     without needing a signed character set.
 *   • |Amplitude| → character density: dense glyphs ('@','#') represent
 *     large displacement; sparse glyphs ('.',':') represent small.
 *     This reinforces the colour signal with a redundant luminance cue —
 *     readable even on monochrome terminals.
 *   • Nodal line: neutral dim glyph where u ≈ 0 between regions of
 *     opposite sign.  Makes mode structure visible between oscillations.
 *   • display_max normalisation: always use the current maximum amplitude
 *     as the full-scale reference so the display is meaningful at any
 *     overall amplitude level (from initial strike to near-silence).
 */
static void render_membrane(WINDOW *w, int cols, int rows,
                            int theme, bool show_nodal,
                            float display_max)
{
    /* Ensure display_max has a reasonable floor to avoid divide-by-zero */
    if (display_max < 0.05f) display_max = 0.05f;
    float inv_max = 1.0f / display_max;

    /* Nodal line threshold: cells below this fraction of max are "near zero" */
    float nodal_thresh = NODAL_SIGN_THRESH * display_max;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float u = g_u[r][c];

            /* ── Nodal line detection ────────────────────────────────── *
             * A cell is on a nodal line if its displacement is near zero
             * AND adjacent cells have opposite signs (zero crossing nearby).
             * We mark these with a dim '·' in a neutral color.           */
            if (show_nodal && fabsf(u) < nodal_thresh) {
                bool near_pos = false, near_neg = false;
                if (r > 0) {
                    near_pos |= (g_u[r-1][c] > nodal_thresh);
                    near_neg |= (g_u[r-1][c] < -nodal_thresh);
                }
                if (r < rows-1) {
                    near_pos |= (g_u[r+1][c] > nodal_thresh);
                    near_neg |= (g_u[r+1][c] < -nodal_thresh);
                }
                if (c > 0) {
                    near_pos |= (g_u[r][c-1] > nodal_thresh);
                    near_neg |= (g_u[r][c-1] < -nodal_thresh);
                }
                if (c < cols-1) {
                    near_pos |= (g_u[r][c+1] > nodal_thresh);
                    near_neg |= (g_u[r][c+1] < -nodal_thresh);
                }
                if (near_pos && near_neg) {
                    /* This cell sits between opposite-phase regions → nodal */
                    wattron(w, COLOR_PAIR(CP_NODAL) | A_DIM);
                    mvwaddch(w, r, c, '.');
                    wattroff(w, COLOR_PAIR(CP_NODAL) | A_DIM);
                    continue;
                }
                /* Near-zero with no opposite-sign neighbour: just background */
                continue;
            }

            /* ── Amplitude → character + color ──────────────────────── */
            bool positive = (u >= 0.0f);
            float norm    = fabsf(u) * inv_max;
            if (norm > 1.0f) norm = 1.0f;

            int   lvl  = lut_index(norm);
            if (lvl == 0) continue;   /* below display threshold — skip */

            attr_t attr = wave_attr(theme, positive, lvl);
            wattron(w, attr);
            mvwaddch(w, r, c, k_ramp[lvl]);
            wattroff(w, attr);
        }
    }
}

/*
 * render_overlay() — stats panel in the bottom-left corner.
 *
 * Displays:
 *   max_amplitude  — peak |u|; indicates energy still in the system
 *   energy_est     — mean (KE + PE) per cell; decays with damping
 *   mode (nx, ny)  — estimated standing wave mode from zero-crossing count
 *   CFL_2D         — c·dt·√2; stability indicator with colour coding
 *   BC             — active boundary condition name
 *   wave_speed     — current c in cells/s
 *   damping        — current γ
 *   sim_time       — total elapsed simulation time
 */
static void render_overlay(WINDOW *w, int cols, int rows,
                           float max_amp, float energy,
                           int mode_nx, int mode_ny,
                           float cfl, float wave_speed,
                           float damping, int bc,
                           float sim_time, bool paused,
                           bool show_nodal, int preset_id)
{
    int pw = 30;   /* panel width in characters */
    int ph = 13;   /* panel height (rows) */
    int ox = 1;
    int oy = rows - ph - 1;
    if (oy < 0) oy = 0;
    if (ox + pw > cols) return;

    /* CFL stability colour: green < 0.7, yellow < 0.9, red ≥ 0.9 */
    int cfl_color;
    const char *cfl_label;
    if (cfl < 0.70f)      { cfl_color = CP_NEG(0, 5); cfl_label = "STABLE  "; }
    else if (cfl < 0.90f) { cfl_color = CP_HUD;        cfl_label = "MARGINAL"; }
    else                  { cfl_color = CP_POS(0, 7);   cfl_label = "UNSTABLE"; }

    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, oy+ 0, ox, "+--- MEMBRANE WAVE --------+");
    mvwprintw(w, oy+ 1, ox, "| max_amp  %8.4f          |", max_amp);
    mvwprintw(w, oy+ 2, ox, "| energy   %8.4f          |", energy);
    mvwprintw(w, oy+ 3, ox, "| mode    (%3d, %3d)         |", mode_nx, mode_ny);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

    /* CFL row: coloured by stability */
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(w, oy+4, ox, "| CFL_2D  ");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wattron(w, COLOR_PAIR(cfl_color) | A_BOLD);
    wprintw(w, "%5.3f %-8s", cfl, cfl_label);
    wattroff(w, COLOR_PAIR(cfl_color) | A_BOLD);
    wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
    wprintw(w, "|");

    mvwprintw(w, oy+ 5, ox, "| BC       %-10s        |", k_bc_names[bc]);
    mvwprintw(w, oy+ 6, ox, "| speed   %6.1f cells/s    |", wave_speed);
    mvwprintw(w, oy+ 7, ox, "| damping %8.4f           |", damping);
    mvwprintw(w, oy+ 8, ox, "| sim_t   %8.2f s          |", sim_time);
    mvwprintw(w, oy+ 9, ox, "| preset  %-10s       |", k_preset_names[preset_id]);
    mvwprintw(w, oy+10, ox, "| nodal   %-3s  %s            |",
              show_nodal ? "ON " : "OFF",
              paused ? "PAUSED " : "running");
    mvwprintw(w, oy+11, ox, "+---------------------------+");
    wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — owns all mutable simulation state.
 *
 * The scene knows nothing about ncurses setup; it only performs physics
 * updates and draws into a passed WINDOW*.  This separation lets the solver
 * be tested without a terminal, and makes resize handling simple (just
 * call scene_resize which re-initialises the grid to the new dimensions).
 */
struct Scene {
    /* physics parameters */
    float   wave_speed;   /* c in cells/s                          */
    float   damping;      /* γ in 1/s                              */
    int     bc;           /* boundary condition type               */

    /* simulation control */
    bool    paused;
    bool    step_requested;   /* advance one tick then re-pause    */
    int     preset_id;

    /* visual */
    int     theme;
    bool    show_nodal;

    /* stats (recomputed each tick; read by render_overlay) */
    float   max_amplitude;
    float   energy_est;
    int     mode_nx, mode_ny;
    float   cfl_2d;
    float   simulation_time;
    float   dt_sec;

    /* grid dimensions (equal to terminal dimensions) */
    int     cols;
    int     rows;
};

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->wave_speed  = WAVE_SPEED_DEFAULT;
    s->damping     = DAMPING_DEFAULT;
    s->bc          = BC_DIRICHLET;
    s->theme       = 0;
    s->show_nodal  = true;
    s->cols        = cols;
    s->rows        = rows;
    init_grid(s->bc, cols, rows);
    preset_apply(s, PRESET_CENTER, cols, rows);
}

static void scene_reset(Scene *s)
{
    init_grid(s->bc, s->cols, s->rows);
    s->simulation_time = 0.0f;
    s->max_amplitude   = 0.0f;
    s->energy_est      = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    init_grid(s->bc, cols, rows);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep.
 *
 * Called from the accumulator loop in §10.  dt is always exactly
 * 1/sim_fps seconds.  The order of operations is:
 *   1. Solve wave PDE for one step (update_wave)
 *   2. Compute stats for the overlay (compute_stats)
 *
 * If paused and no step is requested, return immediately — the
 * physics state is frozen but the display continues to render.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused && !s->step_requested) return;
    s->step_requested = false;

    s->dt_sec           = dt;
    s->simulation_time += dt;

    update_wave(dt, s->wave_speed, s->damping, s->bc, s->cols, s->rows);

    compute_stats(s->wave_speed, dt,
                  s->cols, s->rows,
                  &s->max_amplitude, &s->energy_est,
                  &s->mode_nx, &s->mode_ny,
                  &s->cfl_2d);
}

static void scene_draw(const Scene *s, WINDOW *w, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;  /* membrane has no continuous-motion interpolation */

    render_membrane(w, s->cols, s->rows,
                    s->theme, s->show_nodal,
                    s->max_amplitude);

    render_overlay(w, s->cols, s->rows,
                   s->max_amplitude, s->energy_est,
                   s->mode_nx, s->mode_ny,
                   s->cfl_2d, s->wave_speed,
                   s->damping, s->bc,
                   s->simulation_time, s->paused,
                   s->show_nodal, s->preset_id);
}

/* ── preset implementations ─────────────────────────────────────────── */

static void preset_apply(Scene *s, int id, int cols, int rows)
{
    scene_reset(s);
    s->preset_id = id;

    float cx = (float)(cols - 1) * 0.5f;
    float cy = (float)(rows - 1) * 0.5f;

    switch (id) {
    case PRESET_CENTER:
        /* Single Gaussian strike at the centre.
         * Produces a symmetric expanding ring that reflects repeatedly
         * from all four walls.  The Dirichlet BC causes phase inversion
         * on each reflection — you can count reflections by watching
         * crests turn to troughs at the boundary.                       */
        apply_excitation(cx, cy, EXCITE_AMP, EXCITE_RADIUS, cols, rows);
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_EDGE:
        /* Strike near the left edge.
         * The asymmetric position drives a rich superposition of modes —
         * many harmonics are excited simultaneously.  On a Dirichlet
         * boundary, the eventual steady-state (if no damping) is
         * quasi-periodic Chladni-like patterns.                         */
        apply_excitation((float)(cols - 1) * 0.15f, cy,
                         EXCITE_AMP, EXCITE_RADIUS, cols, rows);
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_DOUBLE:
        /* Two simultaneous strikes at symmetric off-centre positions.
         * The two expanding rings interfere — where crests meet,
         * the amplitude doubles; where crest meets trough, they cancel.
         * Constructive/destructive interference is clearly visible in the
         * first few seconds before the first wall reflections mix them.  */
        apply_excitation((float)(cols - 1) * 0.30f,
                         (float)(rows - 1) * 0.35f,
                         EXCITE_AMP, EXCITE_RADIUS, cols, rows);
        apply_excitation((float)(cols - 1) * 0.70f,
                         (float)(rows - 1) * 0.65f,
                         EXCITE_AMP, EXCITE_RADIUS, cols, rows);
        apply_bc(s->bc, cols, rows);
        break;

    case PRESET_RESONANCE:
        /* Fundamental (1,1) standing wave mode.
         * Direct mode-shape initialisation (no strike needed).
         * u = sin(π·x/W) · sin(π·y/H)  — one half-wave in each direction.
         * With Dirichlet BC, this mode oscillates perfectly and nodal
         * lines are the four boundary edges only.
         * Try switching to BC_NEUMANN to see a different mode shape emerge. */
        preset_resonance(1, 1, RESONANCE_AMP, cols, rows);
        apply_bc(s->bc, cols, rows);
        break;
    }
}

/* ===================================================================== */
/* §9  screen — ncurses double-buffer display layer                      */
/* ===================================================================== */

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

    /* HUD — top-right corner */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  "
             "wave  c=%.0f  γ=%.4f  BC=%-10s ",
             fps, sim_fps,
             sc->wave_speed, sc->damping,
             k_bc_names[sc->bc]);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Key hint — bottom row, right-aligned to avoid overlay clash */
    attron(A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit spc:pause s:step r:reset b:center e:edge "
             "f:double m:mode c/C:speed d/D:damp n:BC l:nodal t:theme [/]:Hz ");
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
 * app_handle_key() — dispatch all user input in one place.
 *
 * Groups:
 *   flow control  — q, ESC, space, s (single step)
 *   excitation    — b (center), e (edge), f (double), m (resonance)
 *   physics       — c/C (wave speed), d/D (damping), n (BC)
 *   simulation    — r (reset), p/P (cycle preset)
 *   visual        — l (nodal), t (theme), ]/[ (sim Hz)
 */
static bool app_handle_key(App *app, int ch)
{
    Scene  *s  = &app->scene;
    Screen *sc = &app->screen;

    switch (ch) {
    /* ── flow control ───────────────────────────────────────────── */
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 's': case 'S':
        /* Single-step: pause if running, then request one tick.
         * Useful for studying wave propagation frame by frame.     */
        s->paused         = true;
        s->step_requested = true;
        break;

    /* ── manual excitation ──────────────────────────────────────── */
    case 'b': case 'B':
        /* Center strike — works regardless of paused state */
        apply_excitation((float)(sc->cols-1)*0.5f, (float)(sc->rows-1)*0.5f,
                         EXCITE_AMP, EXCITE_RADIUS, sc->cols, sc->rows);
        apply_bc(s->bc, sc->cols, sc->rows);
        break;

    case 'e': case 'E':
        apply_excitation((float)(sc->cols-1)*0.15f, (float)(sc->rows-1)*0.5f,
                         EXCITE_AMP, EXCITE_RADIUS, sc->cols, sc->rows);
        apply_bc(s->bc, sc->cols, sc->rows);
        break;

    case 'f': case 'F':
        apply_excitation((float)(sc->cols-1)*0.30f, (float)(sc->rows-1)*0.35f,
                         EXCITE_AMP, EXCITE_RADIUS, sc->cols, sc->rows);
        apply_excitation((float)(sc->cols-1)*0.70f, (float)(sc->rows-1)*0.65f,
                         EXCITE_AMP, EXCITE_RADIUS, sc->cols, sc->rows);
        apply_bc(s->bc, sc->cols, sc->rows);
        break;

    case 'm': case 'M':
        preset_apply(s, PRESET_RESONANCE, sc->cols, sc->rows);
        break;

    /* ── wave speed (c) ─────────────────────────────────────────── *
     * Increasing c raises all eigenfrequencies proportionally,
     * shortening the oscillation period of every mode.
     * The CFL_2D indicator shows how close to instability we are.  */
    case 'c':
        s->wave_speed += WAVE_SPEED_STEP;
        if (s->wave_speed > WAVE_SPEED_MAX) s->wave_speed = WAVE_SPEED_MAX;
        break;
    case 'C':
        s->wave_speed -= WAVE_SPEED_STEP;
        if (s->wave_speed < WAVE_SPEED_MIN) s->wave_speed = WAVE_SPEED_MIN;
        break;

    /* ── damping (γ) ────────────────────────────────────────────── *
     * Higher damping drains energy faster; at max γ the wave
     * decays in a handful of oscillation periods.
     * At γ=0 the wave bounces indefinitely (ideal membrane).       */
    case 'd':
        s->damping += DAMPING_STEP;
        if (s->damping > DAMPING_MAX) s->damping = DAMPING_MAX;
        break;
    case 'D':
        s->damping -= DAMPING_STEP;
        if (s->damping < DAMPING_MIN) s->damping = DAMPING_MIN;
        break;

    /* ── boundary conditions ────────────────────────────────────── *
     * Switching BC changes reflection behaviour immediately.
     * After switching, trigger a fresh reset so the new BC applies
     * to a clean grid (the old field may have incompatible values). */
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

    /* ── visual ─────────────────────────────────────────────────── */
    case 'l': case 'L':
        s->show_nodal = !s->show_nodal;
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % N_THEMES;
        break;

    /* ── simulation Hz ──────────────────────────────────────────── *
     * Changing sim_fps changes dt = 1/sim_fps, which changes CFL.
     * Raising Hz shrinks dt → smaller CFL → safer but more CPU.
     * Lowering Hz increases dt → larger CFL → may go unstable.    */
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
 * Same structure as framework.c §8 main().  See that file for the
 * detailed walk-through of each loop phase.
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

        /* ── dt measurement ──────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* pause guard */

        /* ── fixed-timestep accumulator ──────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ── render interpolation factor ─────────────────────────── */
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

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
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
