/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_boltzman_fluid_simulator.c — D2Q9 Lattice Boltzmann Fluid Simulator
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Flow past a circular cylinder (Kármán vortex street).
 *  Each terminal cell = one LBM lattice node.
 *  Fluid enters from the left, cylinder generates alternating vortices.
 *
 *  Visualization modes (press v/o/d to cycle):
 *    VELOCITY  — speed magnitude shaded dark-blue → red heat-map
 *    VORTICITY — curl field, blue=clockwise, red=counter-clockwise
 *    DENSITY   — pressure field (rho deviation from 1.0)
 *
 *  Streamline overlay (press s): arrow chars show local flow direction.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — DISTRIBUTION FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  In the LBM, instead of tracking individual molecules, we track the
 *  probability f_i(x,t) that a "particle" at lattice node x moves in
 *  direction i at time t.  The D2Q9 model uses 9 directions:
 *
 *    6  2  5          Direction  (ex,ey)   Weight w
 *    3  0  1          ─────────────────────────────
 *    7  4  8          0  rest   (0, 0)    4/9
 *                     1  E      (1, 0)    1/9
 *  The 9 distribution functions {f_0 … f_8} per cell fully describe     2  S      (0, 1)    1/9
 *  the local fluid state.  Macroscopic density and velocity are their    3  W      (-1,0)    1/9
 *  zeroth and first moments:                                             4  N      (0,-1)    1/9
 *                                                                        5  SE     (1, 1)    1/36
 *    ρ = Σ f_i                                                           6  SW     (-1,1)    1/36
 *    ρu = Σ f_i · e_i                                                    7  NW     (-1,-1)   1/36
 *                                                                        8  NE     (1,-1)    1/36
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — EQUILIBRIUM MODEL
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The Maxwell-Boltzmann equilibrium distribution truncated to 2nd order:
 *
 *    f^eq_i = w_i · ρ · [1 + (e_i·u)/cs² + (e_i·u)²/(2cs⁴) - u²/(2cs²)]
 *
 *  where cs² = 1/3 is the lattice speed of sound squared.
 *  Substituting cs²=1/3:  coefficients become 3, 4.5, 1.5 (used in code).
 *
 *  Physical meaning: f^eq is what f would be if the fluid were in local
 *  thermodynamic equilibrium at density ρ, velocity u.  Real fluids
 *  are always relaxing toward this state — that relaxation IS viscosity.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — COLLISION RELAXATION (BGK)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Bhatnagar-Gross-Krook (BGK) collision: relax f toward f^eq at rate ω:
 *
 *    f_i* = f_i − ω·(f_i − f^eq_i)      ω = 1/τ
 *
 *  τ is the relaxation time.  After collision, f* is streamed (propagated).
 *
 *  Physical link to viscosity:
 *    kinematic viscosity  ν = cs²·(τ − 0.5) = (τ − 0.5) / 3
 *
 *  τ→0.5:  ν→0, inviscid flow, numerically unstable (singular).
 *  τ=1.0:  ν=1/6 ≈ 0.167, very viscous (low Reynolds number).
 *  τ=0.6:  ν=0.033, moderate viscosity — good for most demos.
 *
 *  Reynolds number estimate:  Re = U·D / ν   (U=inlet speed, D=diameter)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  BOUNDARY CONDITIONS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  BOUNCE-BACK (no-slip walls + cylinder):
 *    When a particle streams into a solid node, it reflects back with
 *    reversed direction: f_OPP(i) at source ← f_i streamed value.
 *    This enforces zero velocity at the wall (no-slip condition).
 *
 *  INLET (x=0): f set to equilibrium at (ρ=1, u=U_IN, v=0).
 *  OUTLET (x=nx-1): zero-gradient — copy f from penultimate column.
 *  TOP/BOTTOM: bounce-back walls.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Controls:
 *    q/ESC   quit          space   pause/resume
 *    v       velocity mode  o       vorticity mode   d  density mode
 *    s       streamline overlay toggle
 *    +/-     increase/decrease tau (viscosity)
 *    r       reset simulation    n   new obstacle position
 *    w/W     speed up/slow inlet velocity
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/lattice_boltzman_fluid_simulator.c \
 *        -o lbm_fluid -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config    §2 clock    §3 color    §4 D2Q9 model
 *  §5 grid      §6 collide  §7 stream   §8 macroscopic
 *  §9 vorticity §10 render  §11 overlay §12 scene
 *  §13 screen   §14 app
 * ─────────────────────────────────────────────────────────────────────
 */

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
    SIM_FPS_DEFAULT  = 60,
    TARGET_FPS       = 30,    /* render rate — LBM is expensive per cell   */
    FPS_UPDATE_MS    = 500,

    /*
     * STEPS_PER_FRAME = 4
     *
     * LBM is expensive: every step visits every cell (nx×ny nodes).
     * Running multiple physics steps per render frame decouples the
     * physical evolution rate from the display refresh rate.  With 4
     * steps per frame at 30 fps we effectively advance the simulation
     * at 120 physics steps/second while only painting 30 frames/second.
     * Fewer steps → simulation evolves too slowly (boring).
     * More steps  → each frame takes too long, fps drops.
     */
    STEPS_PER_FRAME  = 4,

    /* Visualization modes */
    VIS_VELOCITY  = 0,
    VIS_VORTICITY = 1,
    VIS_DENSITY   = 2,
    VIS_COUNT     = 3,

    /* Color pair IDs */
    CP_VEL0 =  1,   /* slowest — dark blue  */
    CP_VEL1 =  2,
    CP_VEL2 =  3,
    CP_VEL3 =  4,
    CP_VEL4 =  5,
    CP_VEL5 =  6,
    CP_VEL6 =  7,
    CP_VEL7 =  8,   /* fastest — bright red */
    CP_VNEG0=  9,   /* vorticity negative — light blue  */
    CP_VNEG1= 10,
    CP_VNEG2= 11,   /* strong negative — deep blue      */
    CP_VPOS0= 12,   /* vorticity positive — light red   */
    CP_VPOS1= 13,
    CP_VPOS2= 14,   /* strong positive — deep red       */
    CP_SOLID= 15,   /* obstacle / wall                  */
    CP_HUD  = 16,   /* status bar                       */
    CP_ARROW= 17,   /* streamline arrows                */
};

/*
 * U_IN_DEFAULT = 0.10  (lattice units)
 *
 * The inlet velocity expressed in lattice units (lu/step).  The LBM
 * low-Mach assumption requires u ≪ cs = 1/√3 ≈ 0.577.  Above u ≈ 0.3
 * compressibility errors in the weakly-compressible LBM grow rapidly.
 * At the default value the lattice Mach number is:
 *   Ma = u / cs = 0.10 / (1/√3) ≈ 0.17  (safely subsonic)
 * Interactive key 'w' multiplies by 1.2 (capped at 0.30), 'W' divides.
 */
#define U_IN_DEFAULT  0.10f   /* inlet velocity (lattice units, must be ≪1) */

/*
 * TAU_DEFAULT = 0.60
 *
 * The BGK relaxation time τ controls how quickly distributions relax
 * toward equilibrium.  It maps directly to kinematic viscosity:
 *   ν = (τ − 0.5) / 3  →  ν = (0.6 − 0.5) / 3 ≈ 0.0333
 * At this viscosity and the default cylinder size (Re ≈ 50) the flow
 * sits just above the vortex-shedding onset (Re ≈ 47).  The expected
 * Strouhal number St ≈ 0.2, so shedding frequency f ≈ 0.2·U/D ≈ 0.25
 * steps⁻¹ — visible as a rhythmic alternating vortex street.
 */
#define TAU_DEFAULT   0.60f   /* relaxation time: ν = (τ−0.5)/3             */

/*
 * TAU_MIN = 0.505
 *
 * The hard lower bound on τ.  When τ < 0.5, the formula ν = (τ−0.5)/3
 * yields a negative kinematic viscosity — physically meaningless and
 * numerically unconditionally unstable (distributions diverge in one
 * step).  The margin of 0.005 above 0.5 keeps ν small but positive,
 * providing a thin cushion against rounding pushing it below zero.
 */
#define TAU_MIN       0.505f  /* below 0.5 → unstable                        */

/*
 * TAU_MAX = 2.0
 *
 * The upper bound on τ.  At τ = 2.0:
 *   ν = (2.0 − 0.5) / 3 = 0.5  — very high viscosity.
 * Flow becomes extremely laminar; Re drops so low that no vortex
 * shedding occurs and the simulation looks like thick honey flowing
 * around the cylinder.  Useful as an upper pedagogical reference.
 */
#define TAU_MAX       2.0f

/*
 * TAU_STEP = 0.025
 *
 * Interactive increment/decrement step for τ (keys +/-).
 * Small enough that each keypress changes ν by only Δν = 0.025/3 ≈ 0.008,
 * avoiding large sudden jumps in Reynolds number that could destabilize
 * an already-running simulation.
 */
#define TAU_STEP      0.025f

/*
 * CYL_X_FRAC = 0.25
 *
 * Cylinder center placed at 25% of the grid width from the left.
 * This gives enough upstream distance for the inlet equilibrium
 * profile to establish itself before hitting the obstacle, and leaves
 * ~75% of the domain downstream for the vortex street to develop and
 * convect away without reflecting off the outlet boundary.
 */
#define CYL_X_FRAC   0.25f   /* x position as fraction of grid width        */

/*
 * CYL_Y_FRAC = 0.50
 *
 * Cylinder centered vertically.  A perfectly centered obstacle in a
 * symmetric channel will NOT shed vortices spontaneously — symmetry
 * would be preserved forever.  The tiny random perturbation added in
 * lbm_init() breaks this symmetry so shedding eventually develops.
 */
#define CYL_Y_FRAC   0.50f   /* y position as fraction of grid height        */

/*
 * CYL_R_FRAC = 0.08
 *
 * Cylinder radius = 8% of grid height, so diameter D = 2×0.08×ny = 0.16·ny.
 * Reynolds number estimate with default parameters:
 *   Re = U·D / ν = 0.10 × (0.16·ny) / 0.0333 ≈ 0.48·ny
 * On a typical 50-row terminal that gives Re ≈ 24; on an 80-row terminal
 * Re ≈ 38; on a 120-row terminal Re ≈ 58 — straddling the shedding
 * threshold Re ≈ 47 nicely for medium-to-large terminals.
 * (Smaller terminals will show steady attached flow; larger ones will
 *  show clear vortex shedding.)
 */
#define CYL_R_FRAC   0.08f   /* radius as fraction of grid height            */

/* Velocity display chars: 8 ASCII levels (space = no flow, @ = max)        */
static const char VEL_CHARS[8] = {' ', '.', ':', '+', 'x', 'X', '#', '@'};

/* Arrow glyphs for streamline overlay (8 directions, 0=E going CCW)        */
static const char ARROW8[8]    = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    struct timespec req = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        /* Velocity: dark-blue → cyan → green → yellow → orange → red */
        init_pair(CP_VEL0, 17,  COLOR_BLACK);
        init_pair(CP_VEL1, 19,  COLOR_BLACK);
        init_pair(CP_VEL2, 27,  COLOR_BLACK);
        init_pair(CP_VEL3, 45,  COLOR_BLACK);
        init_pair(CP_VEL4, 46,  COLOR_BLACK);
        init_pair(CP_VEL5, 226, COLOR_BLACK);
        init_pair(CP_VEL6, 208, COLOR_BLACK);
        init_pair(CP_VEL7, 196, COLOR_BLACK);
        /* Vorticity negative (clockwise) — blue shades */
        init_pair(CP_VNEG0, 153, COLOR_BLACK);
        init_pair(CP_VNEG1,  33, COLOR_BLACK);
        init_pair(CP_VNEG2,  17, COLOR_BLACK);
        /* Vorticity positive (CCW) — red shades */
        init_pair(CP_VPOS0, 217, COLOR_BLACK);
        init_pair(CP_VPOS1, 160, COLOR_BLACK);
        init_pair(CP_VPOS2,  52, COLOR_BLACK);
        /* Misc */
        init_pair(CP_SOLID, 240, COLOR_BLACK);
        init_pair(CP_HUD,    51, COLOR_BLACK);
        init_pair(CP_ARROW, 255, COLOR_BLACK);
    } else {
        init_pair(CP_VEL0, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_VEL1, COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_VEL2, COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_VEL3, COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_VEL4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_VEL5, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_VEL6, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_VEL7, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_VNEG0, COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_VNEG1, COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_VNEG2, COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_VPOS0, COLOR_RED,    COLOR_BLACK);
        init_pair(CP_VPOS1, COLOR_RED,    COLOR_BLACK);
        init_pair(CP_VPOS2, COLOR_RED,    COLOR_BLACK);
        init_pair(CP_SOLID, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_HUD,   COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_ARROW, COLOR_WHITE,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  D2Q9 model constants                                               */
/* ===================================================================== */

/*
 * D2Q9 lattice velocities and weights.
 *
 *   Dir   (ex, ey)   weight      Opposite
 *    0    ( 0,  0)   4/9         0
 *    1    ( 1,  0)   1/9         3
 *    2    ( 0,  1)   1/9         4   (+y = screen-down)
 *    3    (-1,  0)   1/9         1
 *    4    ( 0, -1)   1/9         2
 *    5    ( 1,  1)   1/36        7
 *    6    (-1,  1)   1/36        8
 *    7    (-1, -1)   1/36        5
 *    8    ( 1, -1)   1/36        6
 */
static const int   EX[9]  = { 0,  1,  0, -1,  0,  1, -1, -1,  1 };
static const int   EY[9]  = { 0,  0,  1,  0, -1,  1,  1, -1, -1 };
static const float W[9]   = {
    4.0f/9.0f,
    1.0f/9.0f,  1.0f/9.0f,  1.0f/9.0f,  1.0f/9.0f,
    1.0f/36.0f, 1.0f/36.0f, 1.0f/36.0f, 1.0f/36.0f
};
static const int   OPP[9] = { 0, 3, 4, 1, 2, 7, 8, 5, 6 };

/*
 * feq() — Maxwell-Boltzmann equilibrium at density rho, velocity (ux,uy).
 *
 *   f^eq_i = w_i · ρ · [1 + 3(e·u) + 4.5(e·u)² − 1.5|u|²]
 *
 * --- Derivation of cs² = 1/3 ---
 * In the D2Q9 quadrature the lattice vectors have components ±1 or 0.
 * The thermal speed is fixed so that Σ_i W[i] * EX[i]² = 1/3 (the
 * x-variance of the weight distribution), giving cs² = 1/3.
 *
 * --- Coefficient derivation from cs² = 1/3 ---
 * The full equilibrium is:
 *   f^eq_i = w·ρ·[ 1 + (e·u)/cs² + (e·u)²/(2cs⁴) − u²/(2cs²) ]
 * Substituting cs² = 1/3:
 *   1/cs²  = 3          → coefficient of the linear momentum term (e·u)
 *   1/(2cs⁴) = 4.5      → coefficient of the quadratic term (e·u)²
 *   1/(2cs²) = 1.5      → coefficient of the speed-magnitude term u²
 *
 * --- Physical meaning of each term ---
 *   1            : rest contribution — even at zero flow a node has mass
 *   3(e·u)       : momentum bias — more particles move in the flow direction
 *   4.5(e·u)²    : momentum correction — accounts for the kinetic energy
 *                  carried by the directed motion (makes f^eq non-negative)
 *   −1.5u²       : velocity-magnitude correction — subtracts the isotropic
 *                  part of kinetic energy so total energy is conserved
 *
 * Together these four terms are the 2nd-order Taylor expansion of the
 * Maxwell-Boltzmann distribution in powers of u/cs.
 */
static inline float feq(int i, float rho, float ux, float uy)
{
    float eu = (float)EX[i] * ux + (float)EY[i] * uy;
    float u2 = ux * ux + uy * uy;
    return W[i] * rho * (1.0f + 3.0f * eu + 4.5f * eu * eu - 1.5f * u2);
}

/* ===================================================================== */
/* §5  grid — allocation, initialization, obstacle setup                  */
/* ===================================================================== */

/*
 * LBM struct — all simulation state in one place.
 *
 * Memory layout notes:
 *
 *   f[ny*nx*9]
 *     The 9 distribution functions for every lattice node, laid out as
 *     f[(y*nx + x)*9 + i].  Total memory ≈ 9 × 4 × nx × ny bytes
 *     (floats).  For a 200×50 grid that is ~360 KB — fits in L2 cache
 *     on most CPUs, which is why LBM is cache-friendly.
 *
 *   ftmp[ny*nx*9]
 *     Streaming scratch buffer (double-buffering).  During the stream
 *     step we write into ftmp while reading from f.  Without this
 *     second buffer, a streamed value could be read again in the same
 *     pass, causing race-condition-like errors even in single-threaded
 *     code.  After streaming, f and ftmp are swapped via pointer swap
 *     (O(1), no copy).
 *
 *   rho[ny*nx], ux[ny*nx], uy[ny*nx]
 *     Macroscopic fields: density and velocity components.  These are
 *     the zeroth and first moments of f (computed in
 *     compute_macroscopic).  Used by collision (feq) and rendering.
 *
 *   vort[ny*nx]
 *     Vorticity (curl of u), computed on demand only when vis_mode ==
 *     VIS_VORTICITY.  Skipping this computation in other modes saves
 *     a full O(nx×ny) pass every step.
 *
 *   solid[ny*nx]
 *     Obstacle/wall mask: 1 = solid node, 0 = fluid node.  Checked
 *     before every collision and stream step.  Solid nodes never have
 *     meaningful distribution functions — bounce-back handles them at
 *     the boundary.
 *
 *   tau / omega
 *     tau  = relaxation time (user-controllable via +/- keys).
 *     omega = 1/tau, cached so the inner collision loop does a multiply
 *     instead of a divide (division is ~5× slower on most FPUs).
 *     Updated together whenever tau changes.
 *
 *   cyl_x, cyl_y, cyl_r
 *     Cylinder obstacle geometry in grid-cell units.  Derived from the
 *     CYL_*_FRAC constants scaled to the current terminal size.
 *
 *   vel_max / rho_min / rho_max
 *     Per-frame statistics collected inside compute_macroscopic.  Used
 *     by the render functions to normalize color maps and by the HUD
 *     overlay to display physical quantities.
 */
typedef struct {
    int    nx, ny;        /* grid dimensions (= terminal cols, rows-2)     */
    float *f;             /* distribution functions [ny*nx*9]               */
    float *ftmp;          /* streaming scratch buffer                        */
    float *rho;           /* macroscopic density       [ny*nx]              */
    float *ux, *uy;       /* macroscopic velocity      [ny*nx]              */
    float *vort;          /* vorticity field (curl uy/dx - curl ux/dy)      */
    uint8_t *solid;       /* 1 = solid (wall/obstacle), 0 = fluid           */

    float  tau;           /* relaxation time                                 */
    float  omega;         /* 1/tau (cached)                                  */
    float  u_in;          /* inlet velocity magnitude                        */

    /* cylinder obstacle */
    int    cyl_x, cyl_y, cyl_r;

    /* runtime visualization state */
    int    vis_mode;
    bool   show_stream;
    bool   paused;

    /* per-frame stats (computed by compute_macroscopic) */
    float  vel_max;
    float  rho_min, rho_max;
    float  vort_max;
    int    step_count;
} LBM;

static int lbm_alloc(LBM *g, int nx, int ny)
{
    g->nx    = nx;
    g->ny    = ny;
    int n    = nx * ny;
    g->f     = calloc(n * 9, sizeof(float));
    g->ftmp  = calloc(n * 9, sizeof(float));
    g->rho   = calloc(n,     sizeof(float));
    g->ux    = calloc(n,     sizeof(float));
    g->uy    = calloc(n,     sizeof(float));
    g->vort  = calloc(n,     sizeof(float));
    g->solid = calloc(n,     sizeof(uint8_t));
    return (g->f && g->ftmp && g->rho && g->ux && g->uy && g->vort && g->solid)
           ? 0 : -1;
}

static void lbm_free(LBM *g)
{
    free(g->f);  free(g->ftmp); free(g->rho);
    free(g->ux); free(g->uy);   free(g->vort); free(g->solid);
}

static void lbm_build_solid(LBM *g)
{
    int nx = g->nx, ny = g->ny;
    memset(g->solid, 0, nx * ny);

    /* Top and bottom walls */
    for (int x = 0; x < nx; x++) {
        g->solid[0 * nx + x]        = 1;
        g->solid[(ny-1) * nx + x]   = 1;
    }

    /* Circular cylinder */
    for (int y = 0; y < ny; y++)
        for (int x = 0; x < nx; x++) {
            int dx = x - g->cyl_x, dy = y - g->cyl_y;
            if (dx*dx + dy*dy <= g->cyl_r * g->cyl_r)
                g->solid[y * nx + x] = 1;
        }
}

static void lbm_init(LBM *g, int nx, int ny, float tau, float u_in)
{
    g->tau      = tau;
    g->omega    = 1.0f / tau;
    g->u_in     = u_in;
    g->vis_mode = VIS_VELOCITY;
    g->show_stream = false;
    g->paused   = false;
    g->step_count = 0;

    g->cyl_x = (int)(nx * CYL_X_FRAC);
    g->cyl_y = (int)(ny * CYL_Y_FRAC);
    g->cyl_r = (int)(ny * CYL_R_FRAC);
    if (g->cyl_r < 2) g->cyl_r = 2;

    lbm_build_solid(g);

    /* Initialize all fluid cells to equilibrium at (rho=1, ux=u_in, uy=0)
     * Add tiny random uy perturbation to break symmetry and seed shedding. */
    srand(12345);
    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            int idx = y * nx + x;
            float perturb = g->solid[idx] ? 0.0f
                          : (float)(rand() % 200 - 100) * 0.00005f;
            for (int i = 0; i < 9; i++)
                g->f[idx * 9 + i] = feq(i, 1.0f, u_in, perturb);
            g->rho[idx] = 1.0f;
            g->ux[idx]  = g->solid[idx] ? 0.0f : u_in;
            g->uy[idx]  = g->solid[idx] ? 0.0f : perturb;
        }
    }
}

/* ===================================================================== */
/* §6  collide() — BGK collision step                                     */
/* ===================================================================== */

/*
 * BGK collision: for each fluid cell, relax each distribution f_i
 * toward its local equilibrium value f^eq_i at rate omega = 1/tau.
 *
 * The update rule written two equivalent ways:
 *
 *   f_i* = f_i − ω·(f_i − f^eq_i)        [subtract deviation]
 *         = (1 − ω)·f_i + ω·f^eq_i        [weighted average form]
 *
 * The second form is easier to interpret: the new distribution is a
 * blend of the old value (weight 1−ω) and equilibrium (weight ω).
 * When ω=1 (τ=1) the distribution jumps all the way to equilibrium
 * in one step — very aggressive mixing.  When ω→0 (τ→∞) the
 * distribution barely moves — extremely slow relaxation (high viscosity).
 *
 * Physical picture: "collision" represents molecules at a lattice node
 * exchanging momentum with each other.  After many collisions the local
 * velocity distribution approaches Maxwell-Boltzmann equilibrium.  The
 * rate at which this happens is ω — it IS the viscosity mechanism.
 *
 * Solid cells are skipped: they contain no fluid, so their f values
 * are never meaningful.  The boundary condition (bounce-back) is handled
 * entirely in the stream step.
 */
static void collide(LBM *g)
{
    int nx = g->nx, ny = g->ny;
    float omega = g->omega;

    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            int idx = y * nx + x;
            if (g->solid[idx]) continue;

            float r = g->rho[idx];
            float u = g->ux[idx];
            float v = g->uy[idx];

            float *fi = g->f + idx * 9;
            for (int i = 0; i < 9; i++)
                fi[i] += omega * (feq(i, r, u, v) - fi[i]);
        }
    }
}

/* ===================================================================== */
/* §7  stream() — propagation + bounce-back boundaries                    */
/* ===================================================================== */

/*
 * Streaming: propagate post-collision distributions to neighbouring nodes.
 * Each f_i(x) moves to the node at (x + e_i) — think of it as each
 * "particle packet" sliding one lattice step in its direction of travel.
 *
 * --- Bounce-back and the no-slip condition ---
 * When the destination node (x + e_i) is a solid cell (wall or cylinder),
 * the particle cannot enter.  Instead it is reflected back: it arrives
 * at the SAME source node x but in the OPPOSITE direction OPP[i].
 *   fnew[x, OPP[i]] += f[x, i]
 * This means the net momentum contribution of that distribution is
 * reversed — the wall exerts an equal and opposite force.  Summing over
 * all directions at the wall node gives net velocity = 0 (no-slip).
 *
 * --- Inlet boundary condition (x = 0) ---
 * After streaming, the inlet column is OVERWRITTEN with f^eq at
 * (rho=1, u=u_in, uy=0).  This models an infinite reservoir upstream
 * that continuously supplies fluid at the prescribed velocity,
 * regardless of what the interior flow delivered to x=0.
 *
 * --- Outlet boundary condition (x = nx-1) ---
 * The outlet uses a zero-gradient (Neumann) condition: f at the last
 * column is copied from the penultimate column (nx-2).  This is the
 * simplest open boundary: it prevents artificial pressure waves from
 * reflecting back into the domain — the same idea as the "absorbing
 * outlet" used in the acoustic wave solver.
 *
 * --- Buffer swap ---
 * We write the streamed result into ftmp while reading from f, then
 * swap the pointers in O(1) — the same double-buffer trick used by the
 * acoustic solver's p/p_old swap.  No data is copied; only two pointers
 * are exchanged.
 */
static void stream(LBM *g)
{
    int nx = g->nx, ny = g->ny;
    float *f    = g->f;
    float *fnew = g->ftmp;

    memset(fnew, 0, nx * ny * 9 * sizeof(float));

    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            if (g->solid[y * nx + x]) continue;

            float *src = f + (y * nx + x) * 9;

            for (int i = 0; i < 9; i++) {
                int nx2 = x + EX[i];
                int ny2 = y + EY[i];

                /* Vertical out-of-bounds or solid → bounce back */
                bool bounce = (ny2 < 0 || ny2 >= ny);
                if (!bounce && nx2 >= 0 && nx2 < nx)
                    bounce = (g->solid[ny2 * nx + nx2] != 0);

                if (bounce) {
                    /* Reflect back to source cell in opposite direction */
                    fnew[(y * nx + x) * 9 + OPP[i]] += src[i];
                } else {
                    /* Clamp x for inlet/outlet (handled below) */
                    int tx = nx2 < 0 ? 0 : (nx2 >= nx ? nx - 1 : nx2);
                    fnew[(ny2 * nx + tx) * 9 + i] += src[i];
                }
            }
        }
    }

    /* Swap buffers — g->f now holds the streamed result */
    float *tmp = g->f;
    g->f = g->ftmp;
    g->ftmp = tmp;

    /* ── Inlet BC (x=0): force equilibrium at u_in ── */
    for (int y = 1; y < ny - 1; y++) {
        if (g->solid[y * nx + 0]) continue;
        float *fi = g->f + (y * nx + 0) * 9;
        for (int i = 0; i < 9; i++)
            fi[i] = feq(i, 1.0f, g->u_in, 0.0f);
    }

    /* ── Outlet BC (x=nx-1): zero-gradient extrapolation ── */
    for (int y = 0; y < ny; y++) {
        float *dst = g->f + (y * nx + (nx-1)) * 9;
        float *src2 = g->f + (y * nx + (nx-2)) * 9;
        memcpy(dst, src2, 9 * sizeof(float));
    }
}

/* ===================================================================== */
/* §8  compute_macroscopic() — ρ, u from distribution moments             */
/* ===================================================================== */

/*
 * Recover the macroscopic fields (density and velocity) from the 9
 * distribution functions via their statistical moments.
 *
 * --- Zeroth moment: density ---
 *   ρ = Σ_i f_i
 * Sum all 9 distributions at a node.  This is mass conservation: the
 * total "probability mass" equals the fluid density at that node.
 *
 * --- First moment: momentum ---
 *   ρ·u = Σ_i f_i · e_i
 * Weighted sum of distributions with their lattice velocities e_i.
 * This is momentum conservation.  Dividing by ρ gives velocity u = ρu/ρ.
 * The e_i vectors are the D2Q9 unit directions defined in EX[] / EY[].
 *
 * Solid cells are set to rho=1, u=0 (no fluid, no flow).
 *
 * Stats (vel_max, rho_min, rho_max) are collected here in the same
 * O(nx×ny) pass, so the render functions and HUD can use them without
 * an extra traversal.
 */
static void compute_macroscopic(LBM *g)
{
    int nx = g->nx, ny = g->ny;
    float vmax = 0.0f, rmin = 1e9f, rmax = -1e9f;

    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            int idx = y * nx + x;
            if (g->solid[idx]) {
                g->rho[idx] = 1.0f;
                g->ux[idx]  = 0.0f;
                g->uy[idx]  = 0.0f;
                continue;
            }
            float *fi = g->f + idx * 9;
            float r = 0.0f, ru = 0.0f, rv = 0.0f;
            for (int i = 0; i < 9; i++) {
                r  += fi[i];
                ru += fi[i] * (float)EX[i];
                rv += fi[i] * (float)EY[i];
            }
            if (r < 1e-6f) r = 1e-6f;
            g->rho[idx] = r;
            g->ux[idx]  = ru / r;
            g->uy[idx]  = rv / r;

            float vm = sqrtf(g->ux[idx]*g->ux[idx] + g->uy[idx]*g->uy[idx]);
            if (vm > vmax)  vmax = vm;
            if (r  < rmin)  rmin = r;
            if (r  > rmax)  rmax = r;
        }
    }
    g->vel_max  = vmax > 1e-6f ? vmax : 1e-6f;
    g->rho_min  = rmin;
    g->rho_max  = rmax;
}

/* ===================================================================== */
/* §9  vorticity — discrete curl of the velocity field                    */
/* ===================================================================== */

/*
 * Vorticity is the rotation rate of a fluid element.  In 3D, vorticity
 * is a vector ω = ∇×u.  In 2D flow (our simulation lives entirely in
 * the x-y plane), only the z-component is non-zero:
 *
 *   ω_z = ∂uy/∂x  −  ∂ux/∂y
 *
 * Why only z?  Because u has no z-component, ∂/∂z = 0 everywhere, so:
 *   ωx = ∂uz/∂y − ∂uy/∂z = 0
 *   ωy = ∂ux/∂z − ∂uz/∂x = 0
 *   ωz = ∂uy/∂x − ∂ux/∂y  ← this is all that survives in 2D
 *
 * Positive ωz means counter-clockwise rotation (right-hand rule about z).
 * Negative ωz means clockwise rotation.
 * The Kármán vortex street shows alternating +/− blobs downstream.
 *
 * Approximated with central finite differences on the lattice:
 *   ∂uy/∂x ≈ [uy(x+1,y) − uy(x−1,y)] / 2     (Δx = 1 lattice unit)
 *   ∂ux/∂y ≈ [ux(x,y+1) − ux(x,y−1)] / 2     (Δy = 1 lattice unit)
 *
 * Border cells (y=0, y=ny-1, x=0, x=nx-1) are skipped because central
 * differences need one neighbour on each side.
 */
static void compute_vorticity(LBM *g)
{
    int nx = g->nx, ny = g->ny;
    float wmax = 0.0f;

    for (int y = 1; y < ny - 1; y++) {
        for (int x = 1; x < nx - 1; x++) {
            int idx = y * nx + x;
            if (g->solid[idx]) { g->vort[idx] = 0.0f; continue; }
            float duy_dx = (g->uy[y*nx+(x+1)] - g->uy[y*nx+(x-1)]) * 0.5f;
            float dux_dy = (g->ux[(y+1)*nx+x] - g->ux[(y-1)*nx+x]) * 0.5f;
            float w = duy_dx - dux_dy;
            g->vort[idx] = w;
            float aw = fabsf(w);
            if (aw > wmax) wmax = aw;
        }
    }
    g->vort_max = wmax > 1e-8f ? wmax : 1e-8f;
}

/* ===================================================================== */
/* §10  render — velocity, vorticity, density, streamlines                */
/* ===================================================================== */

/*
 * render_velocity() — heat-map of speed |u|.
 *
 * Reveals: where the flow is fast (acceleration around cylinder flanks,
 * jet in the wake) and slow (stagnation point upstream of cylinder,
 * recirculation zone immediately downstream).
 *
 * The speed |u| is normalized to [0, vel_max] and mapped to 8 levels.
 * Color pairs CP_VEL0..CP_VEL7 progress dark-blue → red (cold → hot).
 * attron/attroff calls are kept to the minimum needed per cell — calling
 * them once per cell rather than per-character is a key performance
 * optimization: each ncurses attr change flushes internal state.
 */
static void render_velocity(const LBM *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;
    float inv = 7.0f / g->vel_max;

    for (int y = 0; y < ny && y < rows; y++) {
        for (int x = 0; x < nx && x < cols; x++) {
            int idx = y * nx + x;

            if (g->solid[idx]) {
                attron(COLOR_PAIR(CP_SOLID) | A_DIM);
                mvaddch(y + 1, x, '#');
                attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
                continue;
            }

            float vm = sqrtf(g->ux[idx]*g->ux[idx] + g->uy[idx]*g->uy[idx]);
            int   lv = (int)(vm * inv);
            if (lv < 0) lv = 0;
            if (lv > 7) lv = 7;

            int   cp = CP_VEL0 + lv;
            attr_t at = (lv >= 5) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(cp) | at);
            mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
            attroff(COLOR_PAIR(cp) | at);
        }
    }
}

/*
 * render_vorticity() — diverging heatmap of ω_z = curl(u).
 *
 * Reveals: the vortex street — alternating clockwise (blue) and
 * counter-clockwise (red) vortices shed from the cylinder.  This mode
 * makes the Kármán vortex street most visually dramatic.
 *
 * Negative vorticity (clockwise) → blue color pairs CP_VNEG0..2.
 * Positive vorticity (CCW)       → red  color pairs CP_VPOS0..2.
 * 3 intensity levels per sign → 6 color pairs total.
 *
 * attron/attroff: same optimization as render_velocity — one pair of
 * calls per cell, not per character.  A_BOLD is used only for the
 * strongest vortex cores (lv 0 or 5) to make them pop visually.
 */
static void render_vorticity(const LBM *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;
    float inv = 2.9f / g->vort_max;   /* scale to [−3, +3] */

    static const char VOR_CHARS[6] = { '#', 'x', '.', '.', 'x', '#' };
    /* index: 0-2 = negative (blue), 3-5 = positive (red) */
    static const int  VOR_CP[6] = {
        CP_VNEG2, CP_VNEG1, CP_VNEG0, CP_VPOS0, CP_VPOS1, CP_VPOS2
    };

    for (int y = 0; y < ny && y < rows; y++) {
        for (int x = 0; x < nx && x < cols; x++) {
            int idx = y * nx + x;

            if (g->solid[idx]) {
                attron(COLOR_PAIR(CP_SOLID) | A_DIM);
                mvaddch(y + 1, x, '#');
                attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
                continue;
            }

            float w = g->vort[idx] * inv;   /* in [−3, +3] roughly */
            int   lv;
            if (w < 0.0f) {
                lv = (int)(-w);
                if (lv > 2) lv = 2;
                lv = 2 - lv;               /* 0=strong, 2=weak */
            } else {
                lv = (int)(w);
                if (lv > 2) lv = 2;
                lv = 3 + lv;               /* 3=weak, 5=strong */
            }

            attr_t at = (lv == 0 || lv == 5) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(VOR_CP[lv]) | at);
            mvaddch(y + 1, x, (chtype)VOR_CHARS[lv]);
            attroff(COLOR_PAIR(VOR_CP[lv]) | at);
        }
    }
}

/*
 * render_density() — pressure / density field.
 *
 * Reveals: pressure distribution.  In the weakly-compressible LBM,
 * pressure p = cs²·ρ = ρ/3, so density deviations from 1.0 ARE pressure
 * variations.  High-density (rho > 1) regions appear in warm colors —
 * these are the high-pressure zones upstream of the cylinder and on
 * stagnation points.  Low-density (rho < 1) regions appear in cool
 * colors — these are the low-pressure cores of shed vortices.
 *
 * The color scale is stretched to span [rho_min, rho_max] each frame,
 * so even tiny pressure differences (typically Δρ ~ 0.001) are visible.
 * Reuses the CP_VEL0..7 color ramp (dark = low pressure, bright = high).
 */
static void render_density(const LBM *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;
    float rng = (g->rho_max - g->rho_min) > 1e-6f
              ? (g->rho_max - g->rho_min) : 1e-6f;

    for (int y = 0; y < ny && y < rows; y++) {
        for (int x = 0; x < nx && x < cols; x++) {
            int idx = y * nx + x;

            if (g->solid[idx]) {
                attron(COLOR_PAIR(CP_SOLID) | A_DIM);
                mvaddch(y + 1, x, '#');
                attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
                continue;
            }

            float norm = (g->rho[idx] - g->rho_min) / rng;
            int   lv   = (int)(norm * 7.0f);
            if (lv < 0) lv = 0;
            if (lv > 7) lv = 7;

            attron(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
            mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
            attroff(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
        }
    }
}

/*
 * render_streamlines() — sparse arrow overlay on top of any base render.
 * Sample every STREAM_DX cols / STREAM_DY rows; draw directional arrow
 * where speed > threshold.  Arrow is one of 8 ASCII direction chars.
 */
#define STREAM_DX  4
#define STREAM_DY  2

/*
 * STREAM_THR = 0.005
 *
 * Minimum speed required to draw a streamline arrow.  Set to 5% of
 * U_IN_DEFAULT.  In near-stagnation regions (directly upstream of the
 * cylinder nose, or deep in the recirculation zone) the velocity is
 * nearly zero and its direction is dominated by numerical noise.
 * Drawing arrows there produces a confusing spray of random directions.
 * The threshold suppresses these "noise arrows" while still showing
 * flow direction everywhere the fluid is actually moving.
 */
#define STREAM_THR 0.005f   /* only draw arrow if speed above this threshold */

static void render_streamlines(const LBM *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;

    for (int y = STREAM_DY; y < ny - STREAM_DY && y < rows - 1; y += STREAM_DY) {
        for (int x = STREAM_DX; x < nx - STREAM_DX && x < cols; x += STREAM_DX) {
            int idx = y * nx + x;
            if (g->solid[idx]) continue;

            float u = g->ux[idx];
            float v = g->uy[idx];
            float spd = sqrtf(u*u + v*v);
            if (spd < STREAM_THR) continue;

            /* 8-direction angle: atan2 gives [-π, π]; map to [0,7] */
            float ang = atan2f(v, u);                /* screen: +y = down  */
            int   dir = (int)((ang + (float)M_PI) / (2.0f*(float)M_PI) * 8.0f + 0.5f) & 7;

            attron(COLOR_PAIR(CP_ARROW) | A_BOLD);
            mvaddch(y + 1, x, (chtype)ARROW8[dir]);
            attroff(COLOR_PAIR(CP_ARROW) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §11  render_overlay() — HUD status bar                                 */
/* ===================================================================== */

static const char *VIS_NAMES[VIS_COUNT] = { "VELOCITY ", "VORTICITY", "DENSITY  " };

static void render_overlay(const LBM *g, int cols, int rows, double fps)
{
    /* Reynolds estimate: Re = U·D / ν,  ν = (τ−0.5)/3 */
    float nu = (g->tau - 0.5f) / 3.0f;
    float re = (nu > 1e-6f)
             ? (g->u_in * (float)(2 * g->cyl_r) / nu) : 0.0f;

    char top[256];
    snprintf(top, sizeof top,
        " LBM D2Q9 | [%s]%s | τ=%.3f ν=%.4f Re≈%.0f"
        " | ρ[%.3f,%.3f] |u|_max=%.4f | step:%-6d | %.0ffps ",
        VIS_NAMES[g->vis_mode],
        g->show_stream ? "+stream" : "       ",
        g->tau, nu, re,
        g->rho_min, g->rho_max, g->vel_max,
        g->step_count, fps);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, top, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom key help */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  v:velocity  o:vorticity  d:density"
        "  s:streamlines  +/-:tau  r:reset  w/W:speed ");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (g->paused) {
        attron(COLOR_PAIR(CP_VEL6) | A_BOLD);
        mvprintw(rows / 2, cols / 2 - 4, " PAUSED ");
        attroff(COLOR_PAIR(CP_VEL6) | A_BOLD);
    }
}

/* ===================================================================== */
/* §12  scene                                                              */
/* ===================================================================== */

typedef struct { LBM lbm; } Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    LBM *g = &sc->lbm;
    int nx = cols;
    int ny = rows - 2;   /* minus top HUD row and bottom key row */
    if (ny < 4) ny = 4;

    if (lbm_alloc(g, nx, ny) != 0) return;
    lbm_init(g, nx, ny, TAU_DEFAULT, U_IN_DEFAULT);
}

static void scene_free(Scene *sc) { lbm_free(&sc->lbm); }

static void scene_resize(Scene *sc, int cols, int rows)
{
    LBM *g = &sc->lbm;
    int  vm = g->vis_mode;
    bool ss = g->show_stream;
    float tau = g->tau, u_in = g->u_in;

    lbm_free(g);
    memset(g, 0, sizeof *g);
    g->vis_mode   = vm;
    g->show_stream = ss;

    int nx = cols, ny = rows - 2;
    if (ny < 4) ny = 4;
    if (lbm_alloc(g, nx, ny) != 0) return;
    lbm_init(g, nx, ny, tau, u_in);
}

/*
 * scene_tick() — one LBM time step.
 * Order: macroscopic → collide → stream → macroscopic (for render).
 * Vorticity computed once per step for vorticity mode.
 */
static void scene_tick(Scene *sc)
{
    LBM *g = &sc->lbm;
    if (g->paused) return;

    compute_macroscopic(g);
    collide(g);
    stream(g);
    compute_macroscopic(g);
    if (g->vis_mode == VIS_VORTICITY) compute_vorticity(g);
    g->step_count++;
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps)
{
    erase();
    const LBM *g = &sc->lbm;

    switch (g->vis_mode) {
    case VIS_VELOCITY:  render_velocity(g, cols, rows - 2);  break;
    case VIS_VORTICITY: render_vorticity(g, cols, rows - 2); break;
    case VIS_DENSITY:   render_density(g, cols, rows - 2);   break;
    }

    if (g->show_stream) render_streamlines(g, cols, rows - 2);
    render_overlay(g, cols, rows, fps);
}

/* ===================================================================== */
/* §13  screen                                                             */
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
static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §14  app                                                                */
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

static bool handle_key(App *app, int ch)
{
    LBM *g = &app->scene.lbm;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  g->paused = !g->paused; break;
    case 'v': case 'V': g->vis_mode = VIS_VELOCITY;  break;
    case 'o': case 'O': g->vis_mode = VIS_VORTICITY; break;
    case 'd': case 'D': g->vis_mode = VIS_DENSITY;   break;
    case 's': case 'S': g->show_stream = !g->show_stream; break;
    case '+': case '=':
        g->tau = g->tau + TAU_STEP;
        if (g->tau > TAU_MAX) g->tau = TAU_MAX;
        g->omega = 1.0f / g->tau;
        break;
    case '-':
        g->tau = g->tau - TAU_STEP;
        if (g->tau < TAU_MIN) g->tau = TAU_MIN;
        g->omega = 1.0f / g->tau;
        break;
    case 'w':
        g->u_in *= 1.2f;
        if (g->u_in > 0.30f) g->u_in = 0.30f;
        break;
    case 'W':
        g->u_in /= 1.2f;
        if (g->u_in < 0.01f) g->u_in = 0.01f;
        break;
    case 'r': case 'R':
        scene_resize(&app->scene, app->screen.cols, app->screen.rows);
        break;
    default: break;
    }
    return true;
}

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
    int64_t fps_accum   = 0;
    int     fps_count   = 0;
    double  fps_disp    = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
        }

        /* ── dt ──────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── physics: multiple LBM steps per frame ───── */
        for (int s = 0; s < STEPS_PER_FRAME; s++)
            scene_tick(&app->scene);

        /* ── fps counter ─────────────────────────────── */
        fps_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_accum / (double)NS_PER_SEC);
            fps_count = 0;
            fps_accum = 0;
        }

        /* ── sleep to cap render at TARGET_FPS ───────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ── render ──────────────────────────────────── */
        scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR)
            if (!handle_key(app, ch)) { app->running = 0; break; }
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
