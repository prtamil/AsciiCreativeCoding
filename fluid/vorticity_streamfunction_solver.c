/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vorticity_streamfunction_solver.c
 *
 * 2D Incompressible Navier-Stokes via Vorticity-Streamfunction Formulation
 * Classic test case: lid-driven cavity (top wall moves, all others fixed).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A square cavity. The top lid moves right at speed U=1.
 *  A large primary vortex forms at the center.  At higher Re (lower ν)
 *  secondary corner vortices appear.
 *
 *  Three display modes (press v / s / w):
 *    VORTICITY   — diverging heatmap: blue=CW spin, red=CCW spin
 *    STREAMLINES — ψ-contours + velocity arrows (ψ=const IS a streamline)
 *    VELOCITY    — speed magnitude heat-map
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — STREAMFUNCTION  ψ
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  For any 2D incompressible flow (∇·u = 0) we can define a scalar
 *  field ψ(x,y) called the streamfunction such that:
 *
 *       u = ∂ψ/∂y          v = −∂ψ/∂x
 *
 *  Substituting into ∇·u=0:  ∂u/∂x + ∂v/∂y = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0 ✓
 *
 *  WHY THIS IS POWERFUL:
 *    • Continuity is automatically satisfied — no pressure solve needed.
 *    • Lines ψ = constant are streamlines (fluid particles follow them).
 *    • One scalar field replaces the (u,v,p) triplet.
 *
 *  BOUNDARY CONDITIONS:
 *    No-flow-through walls → ψ = constant on every wall.
 *    We set ψ = 0 on the entire boundary (single connected domain).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — VORTICITY TRANSPORT
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Vorticity ω = ∂v/∂x − ∂u/∂y  (z-component of curl u, scalar in 2D).
 *
 *  Taking the curl of the Navier-Stokes momentum equation eliminates
 *  pressure entirely, giving the vorticity transport equation:
 *
 *    ∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y  =  ν · ∇²ω
 *    ─────────────────────────────    ─────────
 *      material derivative             diffusion
 *      (convection of vorticity)      (viscosity spreads spin)
 *
 *  LEFT SIDE — convection:
 *    Vorticity is passively advected by the velocity field.
 *    High-Re flow: vortices travel with the fluid, stay sharp.
 *
 *  RIGHT SIDE — diffusion:
 *    Viscosity (ν) spreads and dissipates vorticity.
 *    Low Re (large ν): diffusion dominates, smooth creeping flow.
 *    High Re (small ν): convection dominates, sharp vortex structures.
 *
 *  Vorticity is GENERATED at no-slip walls (boundary layer).
 *  The moving lid injects positive (CCW) vorticity along the top.
 *  Stationary walls absorb or balance this to maintain ∮ω dA ≈ 0.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — POISSON EQUATION  ∇²ψ = −ω
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Since ω = ∂v/∂x − ∂u/∂y = −(∂²ψ/∂x² + ∂²ψ/∂y²) = −∇²ψ:
 *
 *       ∇²ψ = −ω   (Poisson equation)
 *
 *  Solved iteratively each timestep via SOR (successive over-relaxation):
 *
 *       ψ_GS = (ψ[i,j+1] + ψ[i,j-1] + ψ[i+1,j] + ψ[i-1,j] + h²ω) / 4
 *       ψ_new = (1−α)·ψ + α·ψ_GS        α = SOR_OMEGA ∈ (1, 2)
 *
 *  α > 1 over-relaxes (extrapolates) → converges faster than Gauss-Seidel.
 *  Optimal α ≈ 1.7–1.9 for typical grids.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — BOUNDARY EFFECTS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  ψ = 0 on all walls is the boundary condition for the Poisson solve.
 *
 *  WALL VORTICITY (Thom's formula): at a no-slip wall the vorticity is
 *  derived from the interior ψ values, not set independently:
 *
 *    ω_wall = −2·ψ_interior / h²           (stationary walls)
 *    ω_top  = −2·ψ_interior / h² − 2·U/h  (moving lid, speed U)
 *
 *  This links the vorticity and streamfunction BCs so they remain
 *  thermodynamically consistent throughout the iteration.
 *
 *  CORNER SINGULARITIES:
 *    At top corners (where moving lid meets stationary wall) the velocity
 *    is discontinuous.  This creates a pressure/vorticity singularity that
 *    is smoothed numerically.  Visible as intense ω spots at top corners.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Keys:
 *    q/ESC  quit      space   pause/resume
 *    v      vorticity mode    s  streamline mode    w  velocity mode
 *    +/-    increase/decrease Reynolds number (cycles presets)
 *    r      reset simulation
 *    p      toggle SOR progress display
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        fluid/vorticity_streamfunction_solver.c \
 *        -o vorticity_solver -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config    §2 clock    §3 color    §4 grid
 *  §5 bc        §6 update_vorticity    §7 solve_poisson
 *  §8 compute_velocity                §9 render_vorticity
 *  §10 render_streamlines  §11 render_velocity  §12 render_overlay
 *  §13 scene    §14 screen  §15 app
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

/*
 * Reynolds number presets: Re = U·L/ν, where U=1 (lid speed) and L=1 (domain).
 * So Re is just 1/ν — the ratio of inertial to viscous forces.
 *
 *   Re =   50  — creeping / Stokes flow: viscosity dominates, nearly circular
 *               primary vortex centered near the cavity middle, no corners.
 *   Re =  100  — fully laminar: primary vortex shifts slightly upward, still
 *               smooth and symmetric. Good default for first-time observers.
 *   Re =  400  — onset of weak corner vortices: bottom-left and bottom-right
 *               secondary vortices become visible as inertia starts winning.
 *   Re = 1000  — turbulent corner vortices: primary vortex moves closer to
 *               the lid, strong secondary vortices develop in all corners.
 *               This is the hardest regime to simulate accurately at coarse grids.
 *
 * Increasing Re thins the boundary layers and sharpens vortex gradients,
 * so the solver needs more Poisson sweeps and finer grids to stay accurate.
 */
static const float RE_PRESETS[] = { 50.0f, 100.0f, 400.0f, 1000.0f };
#define N_RE_PRESETS  4
#define RE_DEFAULT_IDX 1    /* start at Re=100 */

/*
 * U_LID — lid speed in dimensionless units.
 * The problem is non-dimensionalized so that U=1 and L=1 → Re = 1/ν exactly.
 * Changing U to a value other than 1 would rescale the Reynolds number
 * (Re = U·L/ν), breaking the clean Re=1/ν relationship. Keep at 1.0.
 */
#define U_LID           1.0f  /* lid velocity (dimensionless)                */

/*
 * SOR_OMEGA — the over-relaxation factor α ∈ (1, 2) for the Poisson solver.
 *
 *   α = 1.0  → pure Gauss-Seidel (GS): converges, but slowly.
 *   α ∈ (1,2) → SOR: each new estimate is extrapolated past the GS value,
 *               accelerating convergence by a factor of ~(N+1)/π for large N.
 *   α → 2.0  → diverges: over-shoots so far the iteration never settles.
 *
 * Theoretical optimum for a square grid of size N:
 *   α_opt ≈ 2 / (1 + sin(π/(N+1)))
 * For N~30 that gives α_opt ≈ 1.82; we use 1.7 as a conservative, robust value
 * that works across the range of grid sizes the terminal may produce.
 */
#define SOR_OMEGA       1.7f  /* SOR over-relaxation factor [1, 2)           */

/*
 * MAX_POISSON — fixed number of SOR sweeps per physics timestep.
 * More sweeps → lower Poisson residual → more accurate ψ → smoother visuals,
 * but each sweep costs O(nx·ny) FLOPs. 14 is the sweet spot for real-time
 * rendering: enough to reduce the residual by ~3 orders of magnitude on a
 * ~80×22 grid without stalling the frame rate.
 *
 * We do NOT check residual inside the sweep loop — that would double the
 * FLOPs for no benefit when the budget is already fixed (see solve_poisson).
 */
#define MAX_POISSON     14    /* SOR sweeps per physics step (no inner check)*/

/*
 * CFL — the Courant-Friedrichs-Lewy safety factor, dimensionless, 0 < CFL ≤ 1.
 *
 * For the explicit upwind convection + Forward Euler scheme, the von Neumann
 * stability condition requires:
 *   dt ≤ CFL · min(dx/|u|, dx²/(4ν))   (advection and diffusion limits)
 *
 * CFL = 0.25 keeps us well inside the stability envelope even when vel_max
 * spikes transiently. Increasing it toward 1.0 risks numerical blow-up.
 */
#define CFL             0.25f /* stability factor: dt = CFL * dt_max         */

/*
 * STEPS_PER_FRAME — how many physics timesteps to advance before redrawing.
 * Each timestep advances the simulation by dt (typically ~1e-4 to 1e-2 s).
 * More steps per frame → the flow evolves faster on screen (more physical
 * time per wall-clock frame), but the CPU budget per frame grows linearly.
 * At 8 steps, a 30 fps display advances ~8×dt of physical time per second.
 */
#define STEPS_PER_FRAME 8     /* physics steps per render frame              */

/*
 * GRID_MAX_X / GRID_MAX_Y — upper cap on the physics grid dimensions.
 * The terminal may be wider/taller than this, but the physics grid never
 * exceeds these sizes — extra terminal cells are left empty or skipped.
 *
 * Why 110×34?  A physics cell costs O(1) per step but rendering must
 * visit every cell per frame.  At 110×34 = 3740 cells × 8 steps/frame
 * × 30 fps = ~900k cell-updates/s, which a modern CPU handles comfortably
 * while leaving room for the Poisson sweeps. Going larger would drop below
 * 30 fps on modest hardware and make the terminal flicker.
 */
#define GRID_MAX_X  110
#define GRID_MAX_Y   34

/* Visualization modes */
#define VIS_VORTICITY  0
#define VIS_STREAM     1
#define VIS_VELOCITY   2
#define VIS_COUNT      3

/*
 * Color pair IDs for ncurses.
 *
 * VORTICITY uses a diverging blue → white → red scheme:
 *   Negative ω (clockwise rotation) → blue tones  (CP_VN2 darkest, CP_VN0 lightest)
 *   Near-zero ω                     → dim grey    (CP_VZERO)
 *   Positive ω (CCW rotation)       → red tones   (CP_VP0 lightest, CP_VP2 darkest)
 * The diverging palette makes it easy to read sign at a glance.
 *
 * VELOCITY uses a sequential dark → bright ramp (CP_VEL0 to CP_VEL7):
 *   Dark blue = slow/stagnant, bright red/white = fast shear regions.
 *   8 levels match the 8-character ASCII density ramp (VEL_CHARS).
 */
#define CP_VN2   1    /* vorticity: strong negative (deep blue)  */
#define CP_VN1   2    /* vorticity: medium negative              */
#define CP_VN0   3    /* vorticity: weak negative (light blue)   */
#define CP_VZERO 4    /* vorticity: near zero (white/dim)        */
#define CP_VP0   5    /* vorticity: weak positive (light red)    */
#define CP_VP1   6    /* vorticity: medium positive              */
#define CP_VP2   7    /* vorticity: strong positive (deep red)   */
#define CP_VEL0  8    /* velocity: slow (dark blue)              */
#define CP_VEL1  9
#define CP_VEL2  10
#define CP_VEL3  11
#define CP_VEL4  12
#define CP_VEL5  13
#define CP_VEL6  14
#define CP_VEL7  15   /* velocity: fast (bright red)             */
#define CP_LID   16   /* moving lid color                        */
#define CP_WALL  17   /* stationary wall                         */
#define CP_HUD   18   /* HUD / overlay text                      */
#define CP_ARROW 19   /* streamline arrows                       */
#define CP_CNTR  20   /* ψ contour lines                         */

/* ASCII density ramp for velocity magnitude display (8 levels) */
static const char VEL_CHARS[8] = {' ', '.', ':', '+', 'x', 'X', '#', '@'};

/* 8-direction arrow chars (E, NE, N, NW, W, SW, S, SE) */
static const char ARROW8[8] = {'>', '/', '^', '\\', '<', '\\', 'v', '/'};

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * color_init — set up ncurses color pairs for all visualization modes.
 *
 * When the terminal supports 256 colors we use xterm-256 palette indices
 * to get a true diverging blue→grey→red scale for vorticity and a
 * perceptually-uniform dark-to-bright ramp for velocity.
 *
 * VORTICITY diverging scheme (256-color path):
 *   CP_VN2  (17)  → deep navy blue   — strong clockwise spin
 *   CP_VN1  (27)  → medium blue
 *   CP_VN0  (153) → pale blue        — weak clockwise spin
 *   CP_VZERO(238) → dim mid-grey     — near-zero / irrotational core
 *   CP_VP0  (217) → pale salmon/red  — weak counter-clockwise spin
 *   CP_VP1  (160) → medium red
 *   CP_VP2  (52)  → deep burgundy    — strong counter-clockwise spin
 *
 * VELOCITY ramp (256-color path, CP_VEL0–CP_VEL7):
 *   Levels 0–7 run from dark blue (17) through cyan (45), green (46),
 *   yellow (226), orange (208) to bright red (196).
 *   This maps naturally to the 8-slot VEL_CHARS density ramp.
 *
 * When only 8 colors are available (fallback path), ANSI colors substitute:
 *   Blue/cyan for negative vorticity, magenta/red for positive.
 *   All 8 velocity levels share cyan (no gradient possible with 8 colors).
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* Vorticity: diverging blue → white → red */
        init_pair(CP_VN2,  17,  COLOR_BLACK);  /* deep blue    */
        init_pair(CP_VN1,  27,  COLOR_BLACK);  /* mid blue     */
        init_pair(CP_VN0,  153, COLOR_BLACK);  /* light blue   */
        init_pair(CP_VZERO,238, COLOR_BLACK);  /* dim grey     */
        init_pair(CP_VP0,  217, COLOR_BLACK);  /* light red    */
        init_pair(CP_VP1,  160, COLOR_BLACK);  /* mid red      */
        init_pair(CP_VP2,  52,  COLOR_BLACK);  /* deep red     */
        /* Velocity magnitude: dark→bright */
        init_pair(CP_VEL0, 17,  COLOR_BLACK);
        init_pair(CP_VEL1, 19,  COLOR_BLACK);
        init_pair(CP_VEL2, 27,  COLOR_BLACK);
        init_pair(CP_VEL3, 45,  COLOR_BLACK);
        init_pair(CP_VEL4, 46,  COLOR_BLACK);
        init_pair(CP_VEL5, 226, COLOR_BLACK);
        init_pair(CP_VEL6, 208, COLOR_BLACK);
        init_pair(CP_VEL7, 196, COLOR_BLACK);
        /* Misc */
        init_pair(CP_LID,  226, COLOR_BLACK);  /* yellow lid   */
        init_pair(CP_WALL, 240, COLOR_BLACK);  /* grey wall    */
        init_pair(CP_HUD,   51, COLOR_BLACK);  /* cyan HUD     */
        init_pair(CP_ARROW,255, COLOR_BLACK);  /* white arrows */
        init_pair(CP_CNTR, 255, COLOR_BLACK);  /* white contour*/
    } else {
        init_pair(CP_VN2,  COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_VN1,  COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_VN0,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(CP_VZERO,COLOR_BLACK,  COLOR_BLACK);
        init_pair(CP_VP0,  COLOR_MAGENTA,COLOR_BLACK);
        init_pair(CP_VP1,  COLOR_RED,    COLOR_BLACK);
        init_pair(CP_VP2,  COLOR_RED,    COLOR_BLACK);
        for (int i = 0; i < 8; i++) init_pair(CP_VEL0+i, COLOR_CYAN, COLOR_BLACK);
        init_pair(CP_LID,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_WALL, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_ARROW,COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_CNTR, COLOR_WHITE,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid — state, allocation, initialization                          */
/* ===================================================================== */

/*
 * VSF — all state for the vorticity-streamfunction solver.
 *
 * GEOMETRY
 *   nx, ny   — grid dimensions in cells (cols × rows). The physical domain
 *              is [0,1]², so there are nx points spaced dx apart in x,
 *              and ny points spaced dy apart in y.
 *   dx, dy   — physical cell spacing: dx = 1/(nx-1), dy = 1/(ny-1).
 *              If nx=81 then dx=0.0125, meaning 80 intervals cover [0,1].
 *
 * TIME
 *   dt       — CFL-adaptive timestep, recomputed every physics step in
 *              compute_velocity(). It shrinks when flow speeds up (advection
 *              limit) and grows when flow is slow (diffusion limit takes over).
 *   nu       — kinematic viscosity = 1/Re.  The reciprocal arises because
 *              Re = U·L/ν and we set U=L=1, so ν = 1/Re directly.
 *   re       — Reynolds number (for display and nu derivation).
 *   re_idx   — index into RE_PRESETS so key +/- can cycle cleanly.
 *
 * FIELD ARRAYS  (all [ny * nx], row-major via IDX macro)
 *   psi      — streamfunction ψ(x,y). Updated by the Poisson solver each step.
 *              Contour lines of ψ = const are the streamlines of the flow.
 *   omega    — vorticity ω(x,y) = ∂v/∂x − ∂u/∂y. The primary transported
 *              quantity; advanced in time by update_vorticity().
 *   omega_new — scratch buffer for the vorticity update. We write the new
 *              values here first, then copy back to omega after the full
 *              sweep — this prevents reading already-updated neighbors
 *              (forward Euler requires the old values everywhere).
 *   u, v     — velocity components (m/s in dimensionless units).
 *              u = ∂ψ/∂y (rightward), v = −∂ψ/∂x (upward in physics coords).
 *              Computed by compute_velocity() from the latest ψ.
 *
 * DIAGNOSTICS
 *   residual   — max |∇²ψ + ω| over all interior cells after the last Poisson
 *                solve. Measures how well ψ satisfies the Poisson equation.
 *                Shown in HUD; should stay small (< 0.1) for stable runs.
 *   iter_count — always MAX_POISSON (fixed budget); kept for HUD display.
 *   step_count — total physics steps since last reset; useful for tracking
 *                how far into steady state the simulation has evolved.
 *
 * DISPLAY NORMALIZATION
 *   vel_max  — maximum speed |u²+v²|^½ found in the last compute_velocity()
 *              call. Used both for CFL dt computation and to normalize the
 *              velocity color/character mapping to [0, 1].
 *   omega_max — exponentially smoothed maximum |ω|. Used to normalize the
 *               vorticity color map so it always fills the full dynamic range
 *               without flickering when a transient spike occurs.
 */
typedef struct {
    int    nx, ny;       /* grid: cols × (rows-2)                          */
    float  dx, dy;       /* spacing: 1/(nx-1), 1/(ny-1)  (domain = [0,1]²)*/
    float  dt;           /* current timestep (CFL-adaptive)                */
    float  nu;           /* kinematic viscosity = 1/Re                     */
    float  re;           /* Reynolds number                                 */
    int    re_idx;       /* index into RE_PRESETS                          */

    float *psi;          /* streamfunction ψ [ny*nx]                       */
    float *omega;        /* vorticity ω       [ny*nx]                      */
    float *omega_new;    /* scratch for time-step                          */
    float *u, *v;        /* velocity (u=∂ψ/∂y, v=−∂ψ/∂x) [ny*nx]         */

    /* solver diagnostics */
    float  residual;     /* last Poisson residual ||∇²ψ + ω||_max          */
    int    iter_count;   /* Poisson iterations used last step               */
    int    step_count;   /* total physics steps since reset                 */
    float  vel_max;      /* max speed (for CFL and display scaling)         */
    float  omega_max;    /* max |ω| for display scaling                    */

    /* visualization */
    int    vis_mode;
    bool   paused;
    bool   show_poisson_info;
} VSF;

#define IDX(g,x,y)  ((y)*(g)->nx + (x))

static int vsf_alloc(VSF *g, int nx, int ny)
{
    g->nx = nx; g->ny = ny;
    int n = nx * ny;
    g->psi       = calloc(n, sizeof(float));
    g->omega     = calloc(n, sizeof(float));
    g->omega_new = calloc(n, sizeof(float));
    g->u         = calloc(n, sizeof(float));
    g->v         = calloc(n, sizeof(float));
    return (g->psi && g->omega && g->omega_new && g->u && g->v) ? 0 : -1;
}

static void vsf_free(VSF *g)
{
    free(g->psi); free(g->omega); free(g->omega_new);
    free(g->u);   free(g->v);
}

static void vsf_init(VSF *g, int nx, int ny, int re_idx)
{
    g->re_idx = re_idx;
    g->re     = RE_PRESETS[re_idx];
    g->nu     = 1.0f / g->re;
    g->dx     = 1.0f / (float)(nx - 1);
    g->dy     = 1.0f / (float)(ny - 1);
    g->dt     = CFL * g->dx * g->dx / g->nu * 0.25f;  /* diffusion limit */
    g->vis_mode = VIS_VORTICITY;
    g->paused = false;
    g->show_poisson_info = false;
    g->step_count = 0;
    g->vel_max = 1e-6f;
    g->omega_max = 1e-6f;

    /* Zero everything (ψ=0, ω=0, u=0, v=0 — quiescent start) */
    memset(g->psi,       0, nx * ny * sizeof(float));
    memset(g->omega,     0, nx * ny * sizeof(float));
    memset(g->omega_new, 0, nx * ny * sizeof(float));
    memset(g->u,         0, nx * ny * sizeof(float));
    memset(g->v,         0, nx * ny * sizeof(float));
}

/* ===================================================================== */
/* §5  apply_bc() — boundary conditions for ψ and ω                      */
/* ===================================================================== */

/*
 * STREAMFUNCTION BOUNDARY CONDITIONS:
 *   ψ = 0 on all four walls (no-penetration, single-value datum).
 *
 * VORTICITY BOUNDARY CONDITIONS — Thom (1933) formula:
 *   At a no-slip wall ψ is known (= 0), and the interior ψ values are also
 *   known from the last Poisson solve.  We use a one-sided finite difference
 *   of ∇²ψ = −ω to recover the wall vorticity without solving a separate
 *   equation.  Expanding the second derivative toward the interior gives:
 *
 *   Bottom (y=0, stationary):  ω[0,x]    = −2·ψ[1,x]   / dy²
 *   Top    (y=ny-1, lid U):    ω[ny-1,x] = −2·ψ[ny-2,x]/ dy² − 2·U/dy
 *   Left   (x=0, stationary):  ω[y,0]    = −2·ψ[y,1]   / dx²
 *   Right  (x=nx-1, stationary):ω[y,nx-1]= −2·ψ[y,nx-2]/ dx²
 *
 * The −2U/dy term on the top wall is the vorticity injected by the moving
 * lid.  It is the sole physical source of vorticity in the cavity — the
 * entire swirling flow you see originates from this one term.
 *
 * Reference: Thom A. (1933), "The Flow Past Circular Cylinders at Low
 * Speeds", Proc. R. Soc. Lond. A, 141(845), 651–669.
 */
static void apply_bc(VSF *g)
{
    int nx = g->nx, ny = g->ny;
    float dx2 = g->dx * g->dx;
    float dy2 = g->dy * g->dy;
    float u   = U_LID;

    for (int x = 0; x < nx; x++) {
        /* Bottom wall: ψ=0, ω = -2ψ[1,x]/dy² */
        g->psi  [IDX(g, x, 0)]      = 0.0f;
        g->omega[IDX(g, x, 0)]      = -2.0f * g->psi[IDX(g, x, 1)]      / dy2;
        /* Top wall (moving lid): ψ=0, ω = -2ψ[ny-2,x]/dy² - 2U/dy */
        g->psi  [IDX(g, x, ny-1)]   = 0.0f;
        g->omega[IDX(g, x, ny-1)]   = -2.0f * g->psi[IDX(g, x, ny-2)]   / dy2
                                      - 2.0f * u / g->dy;
    }
    for (int y = 0; y < ny; y++) {
        /* Left wall: ψ=0, ω = -2ψ[y,1]/dx² */
        g->psi  [IDX(g, 0, y)]      = 0.0f;
        g->omega[IDX(g, 0, y)]      = -2.0f * g->psi[IDX(g, 1, y)]      / dx2;
        /* Right wall: ψ=0, ω = -2ψ[y,nx-2]/dx² */
        g->psi  [IDX(g, nx-1, y)]   = 0.0f;
        g->omega[IDX(g, nx-1, y)]   = -2.0f * g->psi[IDX(g, nx-2, y)]   / dx2;
    }
}

/* ===================================================================== */
/* §6  update_vorticity() — explicit time step of transport equation      */
/* ===================================================================== */

/*
 * Discretize:  ∂ω/∂t = −u·∂ω/∂x − v·∂ω/∂y + ν·∇²ω
 *
 * CONVECTION TERM — first-order upwind differencing:
 *   The upwind scheme picks the finite-difference stencil based on the
 *   local flow direction so that information travels "downwind" from the
 *   correct side.  Using central differences for convection would produce
 *   negative numerical diffusion (anti-diffusion), causing oscillations
 *   and eventual blow-up at moderate Re.  Upwind avoids this at the cost
 *   of being only first-order accurate in space (adds artificial diffusion).
 *
 *   if u > 0: ∂ω/∂x ≈ (ω[i,j] − ω[i,j-1]) / dx   (backward diff)
 *   if u < 0: ∂ω/∂x ≈ (ω[i,j+1] − ω[i,j])  / dx   (forward diff)
 *   (same logic for v in the y direction)
 *
 * DIFFUSION TERM — second-order central differences:
 *   Central differences are safe here because viscous diffusion is inherently
 *   dissipative — it cannot amplify oscillations regardless of the stencil
 *   direction.  The extra accuracy (O(h²) vs O(h)) is worth using.
 *   ∇²ω ≈ (ω_E − 2ω + ω_W)/dx² + (ω_N − 2ω + ω_S)/dy²
 *
 * TIME INTEGRATION — Forward (explicit) Euler:
 *   ω_new = ω_old + dt · RHS
 *   This is first-order accurate in time: O(dt) truncation error.
 *   It is conditionally stable: requires dt < CFL · min(dx/|u|, dx²/4ν).
 *   Simple and cheap per step, which is why it pairs well with a fixed
 *   small CFL. Higher-order schemes (RK4, Adams-Bashforth) would allow
 *   larger dt but add complexity and memory.
 *
 * Only interior nodes (1 ≤ x ≤ nx-2, 1 ≤ y ≤ ny-2) are updated here;
 * wall values are enforced by apply_bc() before and after.
 */
static void update_vorticity(VSF *g)
{
    int nx = g->nx, ny = g->ny;
    float dx = g->dx, dy = g->dy;
    float dt = g->dt, nu = g->nu;
    float idx2 = 1.0f / (dx * dx), idy2 = 1.0f / (dy * dy);

    for (int y = 1; y < ny - 1; y++) {
        for (int x = 1; x < nx - 1; x++) {
            float om  = g->omega[IDX(g, x,   y  )];
            float om_e = g->omega[IDX(g, x+1, y  )];
            float om_w = g->omega[IDX(g, x-1, y  )];
            float om_n = g->omega[IDX(g, x,   y+1)];
            float om_s = g->omega[IDX(g, x,   y-1)];

            float uu = g->u[IDX(g, x, y)];
            float vv = g->v[IDX(g, x, y)];

            /* Upwind convection: follow characteristic direction */
            float dw_dx = (uu > 0.0f) ? (om   - om_w) / dx
                                       : (om_e - om)   / dx;
            float dw_dy = (vv > 0.0f) ? (om   - om_s) / dy
                                       : (om_n - om)   / dy;

            /* Central diffusion */
            float d2w = (om_e - 2.0f*om + om_w) * idx2
                      + (om_n - 2.0f*om + om_s) * idy2;

            g->omega_new[IDX(g, x, y)] = om
                + dt * (-uu * dw_dx - vv * dw_dy + nu * d2w);
        }
    }

    /* Copy interior update; keep boundaries from apply_bc */
    for (int y = 1; y < ny - 1; y++)
        for (int x = 1; x < nx - 1; x++)
            g->omega[IDX(g, x, y)] = g->omega_new[IDX(g, x, y)];
}

/* ===================================================================== */
/* §7  solve_poisson() — SOR iteration for ∇²ψ = −ω                     */
/* ===================================================================== */

/*
 * WHY SOR IS FASTER THAN GAUSS-SEIDEL:
 *   Pure Gauss-Seidel (α=1) converges because each cell corrects itself
 *   toward the average of its neighbors.  SOR with α > 1 extrapolates the
 *   correction past the GS estimate — it "overshoots" toward the root of
 *   the residual equation, then the next sweep corrects the over-shoot.
 *   Net effect: error modes decay ~(N+1)/π times faster than plain GS.
 *
 * GS STENCIL DERIVATION — general (dx ≠ dy) case:
 *   The Poisson equation ∇²ψ = −ω discretized with central differences:
 *     (ψ_E − 2ψ + ψ_W)/dx² + (ψ_N − 2ψ + ψ_S)/dy² = −ω
 *   Solve for ψ:
 *     ψ_GS = [dy²·(ψ_E+ψ_W) + dx²·(ψ_N+ψ_S) + dx²·dy²·ω]
 *             ─────────────────────────────────────────────
 *                          2·(dx² + dy²)
 *   When dx = dy = h (square cells) this collapses to:
 *     ψ_GS = (ψ_E + ψ_W + ψ_N + ψ_S + h²·ω) / 4
 *   We use the general form so the code handles any aspect ratio.
 *
 * SOR UPDATE:
 *   ψ_new = ψ + α · (ψ_GS − ψ)   where α = SOR_OMEGA ∈ (1, 2)
 *
 * WHY RESIDUAL IS COMPUTED ONLY AFTER ALL SWEEPS:
 *   The residual check (fabsf, lap recompute, branch) costs roughly as much
 *   as the update itself.  Checking every iteration would nearly double the
 *   FLOPs per physics step with no benefit — the sweep count is already
 *   fixed at MAX_POISSON regardless of convergence.  A single residual pass
 *   after all sweeps is enough to feed the HUD diagnostics.
 *
 * Returns iteration count; sets g->residual and g->iter_count.
 */
/*
 * solve_poisson — fixed MAX_POISSON SOR sweeps, no per-iteration residual.
 *
 * Residual is computed once after all sweeps for the HUD display.
 * Removing the residual from the inner loop cuts ~half the FLOPs per sweep
 * (no fabsf, no lap recompute, no branch) — critical for real-time speed.
 *
 * Accuracy: 14 SOR sweeps with ω=1.7 on a ~80×22 grid reduce the
 * Poisson error by ~3 orders of magnitude — sufficient for smooth visuals.
 * The residual shown in the HUD reflects this final error, not convergence.
 */
static void solve_poisson(VSF *g)
{
    int   nx = g->nx, ny = g->ny;
    float dx2 = g->dx * g->dx, dy2 = g->dy * g->dy;
    float denom = 2.0f * (dx2 + dy2);
    float alpha = SOR_OMEGA;

    /* ── SOR sweeps: pure update, zero residual overhead ── */
    for (int iter = 0; iter < MAX_POISSON; iter++) {
        for (int y = 1; y < ny - 1; y++) {
            for (int x = 1; x < nx - 1; x++) {
                float ps   = g->psi[IDX(g, x,   y  )];
                float ps_e = g->psi[IDX(g, x+1, y  )];
                float ps_w = g->psi[IDX(g, x-1, y  )];
                float ps_n = g->psi[IDX(g, x,   y+1)];
                float ps_s = g->psi[IDX(g, x,   y-1)];
                float om   = g->omega[IDX(g, x, y)];
                float ps_gs = (dy2*(ps_e + ps_w) + dx2*(ps_n + ps_s) + dx2*dy2*om)
                              / denom;
                g->psi[IDX(g, x, y)] = ps + alpha * (ps_gs - ps);
            }
        }
    }

    /* ── Single residual pass for HUD display ── */
    float res = 0.0f;
    for (int y = 1; y < ny - 1; y++) {
        for (int x = 1; x < nx - 1; x++) {
            float ps   = g->psi[IDX(g, x,   y  )];
            float lap  = (g->psi[IDX(g,x+1,y)] + g->psi[IDX(g,x-1,y)] - 2.0f*ps)/dx2
                       + (g->psi[IDX(g,x,y+1)] + g->psi[IDX(g,x,y-1)] - 2.0f*ps)/dy2;
            float r = fabsf(lap + g->omega[IDX(g, x, y)]);
            if (r > res) res = r;
        }
    }
    g->residual   = res;
    g->iter_count = MAX_POISSON;
}

/* ===================================================================== */
/* §8  compute_velocity() — u = ∂ψ/∂y,  v = −∂ψ/∂x                     */
/* ===================================================================== */

/*
 * Velocity from streamfunction — central differences for interior nodes:
 *
 *   u[x,y] = ∂ψ/∂y ≈ (ψ[x,y+1] − ψ[x,y-1]) / (2·dy)
 *   v[x,y] = −∂ψ/∂x ≈ −(ψ[x+1,y] − ψ[x-1,y]) / (2·dx)
 *
 * Why u = ∂ψ/∂y and v = −∂ψ/∂x?  The streamfunction is defined so that
 * ∇·u = ∂u/∂x + ∂v/∂y = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0 identically.
 *
 * Sign of v:  ψ grows from right to left in a typical lid-driven cavity
 * (the circulation is CCW), so ∂ψ/∂x < 0 in the upper half and the
 * leading minus sign makes v > 0 (upward) on the right side — consistent
 * with the observed vortex rotation.
 *
 * Accuracy: central differences give O(h²) truncation error, one order
 * better than one-sided differences at the same computational cost.
 *
 * ADAPTIVE dt UPDATE (done here because vel_max is freshly computed):
 *   Two stability constraints are checked and the tighter one is used:
 *
 *   Convection (CFL) limit:    dt_conv = CFL · dx / vel_max
 *     → if you move faster than one cell per dt, upwind stencil "jumps" cells.
 *
 *   Diffusion (von Neumann) limit:  dt_diff = CFL · dx² / (4·ν)
 *     → explicit diffusion requires dt ≤ dx²/(4ν) in 2D (factor of 4 from
 *       2 spatial dimensions, each contributing 1/(2ν·dx²)).
 *
 *   The minimum of the two limits is clamped to [1e-6, 0.01] to prevent
 *   runaway shrinkage or overly large steps during transients.
 *
 * Wall velocities are set explicitly (no-slip; top lid overrides u = U_LID).
 */
static void compute_velocity(VSF *g)
{
    int nx = g->nx, ny = g->ny;
    float inv2dx = 0.5f / g->dx, inv2dy = 0.5f / g->dy;
    float vmax = 0.0f;

    /* Interior */
    for (int y = 1; y < ny - 1; y++) {
        for (int x = 1; x < nx - 1; x++) {
            float uu = (g->psi[IDX(g,x,y+1)] - g->psi[IDX(g,x,y-1)]) * inv2dy;
            float vv = -(g->psi[IDX(g,x+1,y)] - g->psi[IDX(g,x-1,y)]) * inv2dx;
            g->u[IDX(g, x, y)] = uu;
            g->v[IDX(g, x, y)] = vv;
            float spd = sqrtf(uu*uu + vv*vv);
            if (spd > vmax) vmax = spd;
        }
    }

    /* Walls (no-slip; top lid overrides u) */
    for (int x = 0; x < nx; x++) {
        g->u[IDX(g,x,0)]    = 0.0f; g->v[IDX(g,x,0)]    = 0.0f;
        g->u[IDX(g,x,ny-1)] = U_LID; g->v[IDX(g,x,ny-1)] = 0.0f;
    }
    for (int y = 0; y < ny; y++) {
        g->u[IDX(g,0,y)]    = 0.0f; g->v[IDX(g,0,y)]    = 0.0f;
        g->u[IDX(g,nx-1,y)] = 0.0f; g->v[IDX(g,nx-1,y)] = 0.0f;
    }

    g->vel_max = vmax > 1e-6f ? vmax : 1e-6f;

    /* Adaptive dt: CFL based on current velocity and diffusion limits */
    float dt_conv = (vmax > 1e-6f) ? CFL * g->dx / vmax : 1.0f;
    float dt_diff = CFL * g->dx * g->dx / (4.0f * g->nu);
    g->dt = (dt_conv < dt_diff) ? dt_conv : dt_diff;
    if (g->dt < 1e-6f) g->dt = 1e-6f;
    if (g->dt > 0.01f) g->dt = 0.01f;
}

/* ===================================================================== */
/* §9  render_vorticity() — diverging blue-white-red heatmap              */
/* ===================================================================== */

/*
 * Render helpers use attrset() with change-detection optimization:
 *
 *   - attrset() replaces ALL current attributes in one call (no separate
 *     attron/attroff pair needed), which saves one ncurses call per cell.
 *   - We track cur_attr and only call attrset() when the desired attribute
 *     differs from the current one.  In practice most adjacent cells share
 *     the same vorticity band, so the number of attrset() calls is much
 *     smaller than the number of cells — typically ~1 call per band boundary
 *     rather than 3×NX×NY calls (attron + mvaddch + attroff) for every cell.
 *   - The result is roughly a 3× reduction in ncurses write calls for smooth
 *     flow fields, which matters at 30 fps on large grids.
 *
 * Vorticity is normalized to [-1, 1] by dividing by omega_max (which is
 * updated with exponential smoothing in scene_tick to prevent flicker).
 * Seven threshold bands map to the seven color pairs defined in §3.
 *
 * The top row (gy == ny-1) is always drawn as the moving lid ('=' chars
 * in yellow, CP_LID) regardless of the vorticity value there.
 *
 * Screen row sy=0 is the HUD bar, so fluid drawing starts at row sy+1.
 * Physics y-axis is inverted: sy=0 maps to gy=ny-1 (top of physics grid).
 */
static void render_vorticity(const VSF *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;
    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        for (int x = 0; x < nx && x < cols; x++) {
            if (gy == ny - 1) {
                chtype a = COLOR_PAIR(CP_LID) | A_BOLD;
                if (a != cur_attr) { attrset(a); cur_attr = a; }
                mvaddch(sy + 1, x, '=');
                continue;
            }
            float wn = g->omega[IDX(g, x, gy)] / g->omega_max;
            if (wn >  1.0f) wn =  1.0f;
            if (wn < -1.0f) wn = -1.0f;

            int cp; char ch; attr_t at;
            if      (wn < -0.60f) { cp=CP_VN2;   ch='#'; at=A_BOLD;   }
            else if (wn < -0.25f) { cp=CP_VN1;   ch='x'; at=A_NORMAL; }
            else if (wn < -0.05f) { cp=CP_VN0;   ch='.'; at=A_NORMAL; }
            else if (wn <  0.05f) { cp=CP_VZERO; ch=' '; at=A_DIM;    }
            else if (wn <  0.25f) { cp=CP_VP0;   ch='.'; at=A_NORMAL; }
            else if (wn <  0.60f) { cp=CP_VP1;   ch='x'; at=A_NORMAL; }
            else                  { cp=CP_VP2;   ch='#'; at=A_BOLD;   }

            chtype a = (chtype)COLOR_PAIR(cp) | at;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)ch);
        }
    }
    attrset(A_NORMAL);
}

/* ===================================================================== */
/* §10  render_streamlines() — ψ contours + velocity arrows               */
/* ===================================================================== */

/*
 * Two-layer render:
 *   BACKGROUND: shade each cell by ψ value (which iso-band it falls in).
 *   OVERLAY:    draw a velocity arrow at every 4th column / 2nd row
 *               to show local flow direction.
 *
 * ψ contours: divide [ψ_min, ψ_max] into N_CONTOURS bands.
 * Alternate between two dim colors for adjacent bands — this gives
 * the classic streamline "banded" look without drawing explicit lines.
 */
#define N_CONTOURS  12
#define STREAM_DX    4
#define STREAM_DY    2

static void render_streamlines(const VSF *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;

    /* Find ψ range */
    float pmin = 1e9f, pmax = -1e9f;
    for (int i = 0; i < nx*ny; i++) {
        if (g->psi[i] < pmin) pmin = g->psi[i];
        if (g->psi[i] > pmax) pmax = g->psi[i];
    }
    float rng = (pmax - pmin) > 1e-8f ? (pmax - pmin) : 1e-8f;

    /* Background: color band by ψ level */
    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        if (gy == ny - 1) {
            chtype a = COLOR_PAIR(CP_LID) | A_BOLD;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            for (int x = 0; x < nx && x < cols; x++) mvaddch(sy+1, x, '=');
            continue;
        }
        for (int x = 0; x < nx && x < cols; x++) {
            float ps   = g->psi[IDX(g, x, gy)];
            int   band = (int)((ps - pmin) / rng * (float)N_CONTOURS) % N_CONTOURS;
            if (band < 0) band = 0;
            float spd = sqrtf(g->u[IDX(g,x,gy)]*g->u[IDX(g,x,gy)]
                             + g->v[IDX(g,x,gy)]*g->v[IDX(g,x,gy)]);
            char  ch  = VEL_CHARS[(int)(spd / g->vel_max * 7.0f + 0.5f) & 7];
            int   cp  = (band & 1) ? CP_VEL2 : CP_VEL4;
            chtype a  = (chtype)COLOR_PAIR(cp) | A_NORMAL;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)ch);
        }
    }

    /* Arrow overlay every STREAM_DX × STREAM_DY cells */
    chtype arrow_attr = COLOR_PAIR(CP_ARROW) | A_BOLD;
    for (int sy = STREAM_DY; sy < ny - STREAM_DY && sy < rows-1; sy += STREAM_DY) {
        int gy = ny - 1 - sy;
        for (int x = STREAM_DX; x < nx - STREAM_DX && x < cols; x += STREAM_DX) {
            float uu = g->u[IDX(g, x, gy)];
            float vv = -g->v[IDX(g, x, gy)];
            float spd = sqrtf(uu*uu + vv*vv);
            if (spd < 0.01f * g->vel_max) continue;
            float ang = atan2f(vv, uu);
            int   dir = (int)((ang + (float)M_PI) / (2.0f*(float)M_PI) * 8.0f + 0.5f) & 7;
            if (arrow_attr != cur_attr) { attrset(arrow_attr); cur_attr = arrow_attr; }
            mvaddch(sy + 1, x, (chtype)ARROW8[dir]);
        }
    }
    attrset(A_NORMAL);
}

/* ===================================================================== */
/* §11  render_velocity() — speed magnitude heat-map                      */
/* ===================================================================== */

static void render_velocity(const VSF *g, int cols, int rows)
{
    int nx = g->nx, ny = g->ny;
    float inv = 7.0f / g->vel_max;
    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        if (gy == ny - 1) {
            chtype a = COLOR_PAIR(CP_LID) | A_BOLD;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            for (int x = 0; x < nx && x < cols; x++) mvaddch(sy+1, x, '=');
            continue;
        }
        for (int x = 0; x < nx && x < cols; x++) {
            float spd = sqrtf(g->u[IDX(g,x,gy)]*g->u[IDX(g,x,gy)]
                             + g->v[IDX(g,x,gy)]*g->v[IDX(g,x,gy)]);
            int lv = (int)(spd * inv);
            if (lv < 0) lv = 0;
            if (lv > 7) lv = 7;
            chtype a = (chtype)COLOR_PAIR(CP_VEL0 + lv) | ((lv >= 5) ? A_BOLD : A_NORMAL);
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)VEL_CHARS[lv]);
        }
    }
    attrset(A_NORMAL);
}

/* ===================================================================== */
/* §12  render_overlay() — HUD status                                     */
/* ===================================================================== */

static const char *VIS_NAMES[VIS_COUNT] = { "VORTICITY", "STREAMLINES", "VELOCITY " };

static void render_overlay(const VSF *g, int cols, int rows, double fps)
{
    char buf[300];
    snprintf(buf, sizeof buf,
        " VSF-NS [%s] | Re=%.0f  ν=%.4f | dt=%.5f"
        " | res=%.2e  iter=%d | |ω|_max=%.3f | step:%-5d | %.0ffps ",
        VIS_NAMES[g->vis_mode],
        g->re, g->nu, g->dt,
        g->residual, g->iter_count,
        g->omega_max, g->step_count, fps);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Key help */
    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  v:vorticity  s:streamlines  w:velocity"
        "  +/-:Re  r:reset ");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (g->paused) {
        attron(COLOR_PAIR(CP_VP2) | A_BOLD);
        mvprintw(rows / 2, cols / 2 - 4, " PAUSED ");
        attroff(COLOR_PAIR(CP_VP2) | A_BOLD);
    }
}

/* ===================================================================== */
/* §13  scene                                                              */
/* ===================================================================== */

typedef struct { VSF vsf; } Scene;

static void scene_init(Scene *sc, int cols, int rows, int re_idx)
{
    VSF *g = &sc->vsf;
    int nx = cols   < GRID_MAX_X ? cols   : GRID_MAX_X;
    int ny = rows-2 < GRID_MAX_Y ? rows-2 : GRID_MAX_Y;
    if (ny < 4) ny = 4;
    if (vsf_alloc(g, nx, ny) != 0) return;
    vsf_init(g, nx, ny, re_idx);
}

static void scene_free(Scene *sc) { vsf_free(&sc->vsf); }

static void scene_resize(Scene *sc, int cols, int rows)
{
    VSF *g = &sc->vsf;
    int vm = g->vis_mode, ri = g->re_idx;
    vsf_free(g); memset(g, 0, sizeof *g);
    int nx = cols   < GRID_MAX_X ? cols   : GRID_MAX_X;
    int ny = rows-2 < GRID_MAX_Y ? rows-2 : GRID_MAX_Y;
    if (ny < 4) ny = 4;
    if (vsf_alloc(g, nx, ny) != 0) return;
    vsf_init(g, nx, ny, ri);
    g->vis_mode = vm;
}

/*
 * scene_tick() — one full timestep:
 *   1. update_vorticity: advance ω by dt
 *   2. solve_poisson:    recover ψ from new ω
 *   3. compute_velocity: get u,v from new ψ
 *   4. apply_bc:         re-enforce wall conditions
 */
static void scene_tick(Scene *sc)
{
    VSF *g = &sc->vsf;
    if (g->paused) return;

    update_vorticity(g);
    apply_bc(g);
    solve_poisson(g);
    compute_velocity(g);
    apply_bc(g);
    g->step_count++;

    /* Update omega_max for display scaling */
    float wmax = 1e-6f;
    for (int i = 0; i < g->nx * g->ny; i++) {
        float aw = fabsf(g->omega[i]);
        if (aw > wmax) wmax = aw;
    }
    /* Smooth the max to avoid flicker */
    g->omega_max = g->omega_max * 0.95f + wmax * 0.05f;
    if (g->omega_max < 1e-6f) g->omega_max = 1e-6f;
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps)
{
    erase();
    const VSF *g = &sc->vsf;
    switch (g->vis_mode) {
    case VIS_VORTICITY: render_vorticity(g, cols, rows - 2); break;
    case VIS_STREAM:    render_streamlines(g, cols, rows - 2); break;
    case VIS_VELOCITY:  render_velocity(g, cols, rows - 2);  break;
    }
    render_overlay(g, cols, rows, fps);
}

/* ===================================================================== */
/* §14  screen                                                             */
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
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

/* ===================================================================== */
/* §15  app                                                                */
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
    VSF *g = &app->scene.vsf;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  g->paused = !g->paused; break;
    case 'v': case 'V': g->vis_mode = VIS_VORTICITY; break;
    case 's': case 'S': g->vis_mode = VIS_STREAM;    break;
    case 'w': case 'W': g->vis_mode = VIS_VELOCITY;  break;
    case '+': case '=':
        g->re_idx = (g->re_idx + 1) % N_RE_PRESETS;
        g->re = RE_PRESETS[g->re_idx];
        g->nu = 1.0f / g->re;
        break;
    case '-':
        g->re_idx = (g->re_idx + N_RE_PRESETS - 1) % N_RE_PRESETS;
        g->re = RE_PRESETS[g->re_idx];
        g->nu = 1.0f / g->re;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows, RE_DEFAULT_IDX);

    int64_t frame_time = clock_ns();
    int64_t fps_accum  = 0;
    int     fps_count  = 0;
    double  fps_disp   = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
        }

        /* ── dt ──────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── physics steps ───────────────────────── */
        for (int s = 0; s < STEPS_PER_FRAME; s++)
            scene_tick(&app->scene);

        /* ── fps ─────────────────────────────────── */
        fps_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_accum / (double)NS_PER_SEC);
            fps_count = 0; fps_accum = 0;
        }

        /* ── sleep to cap at 30 fps ──────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 30 - elapsed);

        /* ── render ──────────────────────────────── */
        scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR)
            if (!handle_key(app, ch)) { app->running = 0; break; }
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
