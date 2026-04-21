/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/multigrid_solver_visualizer.c -- Multigrid V-cycle Poisson Solver
 *
 * Solves  -Del^2u = f  on a grid with Dirichlet u=0 boundary conditions.
 * Animated step-by-step: one Gauss-Seidel sweep per tick, with
 * restrict/prolong as instant transitions between levels.
 *
 * -------------------------------------------------------------------------
 *  Grid hierarchy (4 levels, halved each time):
 *    L0: 64x32  (fine)    L1: 32x16    L2: 16x8    L3: 8x4  (coarse)
 *
 *  Panel layout -- each level gets terminal columns ~ its logical width:
 *    L0(53%) | L1(27%) | L2(13%) | L3(7%)   (for any terminal width)
 *  Each panel renders the residual as a heat map.
 *  Active level is highlighted; V-cycle position shown in HUD.
 *
 * -------------------------------------------------------------------------
 *  Section map
 * -------------------------------------------------------------------------
 *  S1  config      -- grid sizes, V-cycle params, timing
 *  S2  clock       -- nanosecond monotonic clock + sleep
 *  S3  color       -- heat-map palette + UI color pairs
 *  S4  grid        -- global field arrays, source init, norms
 *  S5  solver      -- relax(), compute_residual(), restrict_op(), prolong_op()
 *  S6  vcycle      -- V-cycle state machine, convergence tracking
 *  S7  scene       -- Scene, scene_init/tick/reset
 *  S8  render      -- render_grid(), render_vcycle_hud(), render_overlay()
 *  S9  screen      -- ncurses layer
 *  S10 app         -- signals, resize, input, main loop
 * -------------------------------------------------------------------------
 *
 * Keys:  SPACE pause/resume   s single-step   r reset (new source)
 *        +/-   speed          q quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       physics/multigrid_solver_visualizer.c \
 *       -o mg -lncurses -lm
 */

/* -- CONCEPTS ------------------------------------------------------------ *
 *
 * Multigrid key insight (S5, S6):
 *   Gauss-Seidel quickly damps HIGH-frequency error (fine-scale wiggles)
 *   but converges very slowly on LOW-frequency error (smooth, broad humps).
 *   On a coarser grid, those same smooth errors look HIGH-frequency relative
 *   to the coarse spacing, so a few GS sweeps there kill them cheaply.
 *   The V-cycle exploits this: smooth on fine -> restrict -> smooth on coarse
 *   -> ... -> coarse solve -> prolong correction back -> smooth on fine.
 *
 * Error smoothing (S5 relax):
 *   Each red-black Gauss-Seidel sweep replaces u[i,j] with the average of
 *   its four neighbours plus the source:
 *     u[i,j] = ( u[i-1,j]+u[i+1,j]+u[i,j-1]+u[i,j+1] + f[i,j] ) / 4
 *   This is a local averaging operation -- it erases high-frequency modes
 *   in O(1) sweeps but cannot reach long-range equilibrium.
 *
 * Grid hierarchy benefit (S6 restrict_op, prolong_op):
 *   After pre-smoothing, the remaining error is smooth.  Restricting
 *   the residual to a 2x coarser grid halves the wavelength-to-grid-spacing
 *   ratio, making the smooth error look rough again.  A few sweeps there
 *   cost 1/4 the work (quarter of the grid points) and eliminate it.
 *   The correction is interpolated back (prolonged) to the fine grid.
 *   Net effect: O(N) work per V-cycle vs O(N^2) for plain Gauss-Seidel.
 *
 * Convergence rate (S6):
 *   Ideal multigrid converges with a geometry-independent rate rho < 0.2,
 *   meaning each V-cycle reduces ||r|| by at least 80%.  The overlay
 *   shows rho = ||r_k|| / ||r_{k-1}|| and a bar chart of ||r|| per cycle.
 *
 * ----------------------------------------------------------------------- */

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
/* S1  config                                                             */
/* ===================================================================== */

#define SIM_FPS_DEFAULT   6
#define SIM_FPS_MIN       1
#define SIM_FPS_MAX      30
#define TARGET_FPS       60
#define FPS_UPDATE_MS    500

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Grid hierarchy */
#define LEVELS     4
#define L0_NX     64     /* fine grid width  (boundary-inclusive) */
#define L0_NY     32     /* fine grid height (boundary-inclusive) */
#define MAX_PTS   (L0_NX * L0_NY)   /* 2048 -- largest level */

/* Grid dimensions at level l */
#define LNX(l)        (L0_NX >> (l))
#define LNY(l)        (L0_NY >> (l))
#define LIDX(l, x, y) ((y) * LNX(l) + (x))

/* Total logical columns across all levels (for proportional panel widths) */
#define TOTAL_LNX  (L0_NX + (L0_NX>>1) + (L0_NX>>2) + (L0_NX>>3))  /* 120 */

/* V-cycle sweep counts */
#define PRE_SMOOTH    2
#define POST_SMOOTH   2
#define COARSE_ITERS 20   /* extra sweeps at coarsest level */

/* Source term: Gaussian bump at fine-grid centre */
#define SRC_AMP    8.0f
#define SRC_SIGMA  4.0f

/* Convergence history ring */
#define CONV_HIST     32
#define CONV_HIST_LEN 32

/* HUD rows at bottom */
#define HUD_ROWS    9

/* ===================================================================== */
/* S2  clock                                                              */
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
/* S3  color                                                              */
/* ===================================================================== */

enum {
    CP_NONE = 0,
    /* Heat-map ramp: 9 levels, low -> high residual */
    CP_H0, CP_H1, CP_H2, CP_H3, CP_H4, CP_H5, CP_H6, CP_H7, CP_H8,
    /* UI */
    CP_BORDER_ACT,   /* active level border   */
    CP_BORDER_OFF,   /* inactive level border */
    CP_HUD,          /* HUD text              */
    CP_HEADER,       /* header row            */
    CP_LABEL,        /* axis/divider labels   */
    CP_EXPLAIN,      /* footer explanation    */
    CP_PHASE_DOWN,   /* pre-smooth going down */
    CP_PHASE_UP,     /* post-smooth going up  */
    CP_PHASE_COARSE, /* coarse solve          */
    CP_CONV_BAR,     /* convergence bar       */
    CP_CONV_CUR,     /* current cycle bar     */
    CP_CONV_GOOD,    /* rate < 0.2            */
    CP_CONV_MID,     /* rate 0.2-0.5          */
    CP_CONV_BAD,     /* rate > 0.5            */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* Heat: dark navy -> blue -> cyan -> green -> yellow -> orange -> red */
        init_pair(CP_H0,  17,  -1);
        init_pair(CP_H1,  19,  -1);
        init_pair(CP_H2,  27,  -1);
        init_pair(CP_H3,  39,  -1);
        init_pair(CP_H4,  46,  -1);
        init_pair(CP_H5, 226,  -1);
        init_pair(CP_H6, 208,  -1);
        init_pair(CP_H7, 196,  -1);
        init_pair(CP_H8, 201,  -1);
        init_pair(CP_BORDER_ACT,  51,  -1);
        init_pair(CP_BORDER_OFF, 238,  -1);
        init_pair(CP_HUD,        252,  -1);
        init_pair(CP_HEADER,      51,  -1);
        init_pair(CP_LABEL,      244,  -1);
        init_pair(CP_EXPLAIN,    227,  -1);
        init_pair(CP_PHASE_DOWN, 201,  -1);
        init_pair(CP_PHASE_UP,    46,  -1);
        init_pair(CP_PHASE_COARSE,226, -1);
        init_pair(CP_CONV_BAR,    51,  -1);
        init_pair(CP_CONV_CUR,   226,  -1);
        init_pair(CP_CONV_GOOD,   46,  -1);
        init_pair(CP_CONV_MID,   226,  -1);
        init_pair(CP_CONV_BAD,   196,  -1);
    } else {
        init_pair(CP_H0, COLOR_BLUE,    -1);
        init_pair(CP_H1, COLOR_BLUE,    -1);
        init_pair(CP_H2, COLOR_BLUE,    -1);
        init_pair(CP_H3, COLOR_CYAN,    -1);
        init_pair(CP_H4, COLOR_GREEN,   -1);
        init_pair(CP_H5, COLOR_GREEN,   -1);
        init_pair(CP_H6, COLOR_YELLOW,  -1);
        init_pair(CP_H7, COLOR_RED,     -1);
        init_pair(CP_H8, COLOR_RED,     -1);
        init_pair(CP_BORDER_ACT,  COLOR_CYAN,    -1);
        init_pair(CP_BORDER_OFF,  COLOR_WHITE,   -1);
        init_pair(CP_HUD,         COLOR_WHITE,   -1);
        init_pair(CP_HEADER,      COLOR_CYAN,    -1);
        init_pair(CP_LABEL,       COLOR_WHITE,   -1);
        init_pair(CP_EXPLAIN,     COLOR_YELLOW,  -1);
        init_pair(CP_PHASE_DOWN,  COLOR_MAGENTA, -1);
        init_pair(CP_PHASE_UP,    COLOR_GREEN,   -1);
        init_pair(CP_PHASE_COARSE,COLOR_YELLOW,  -1);
        init_pair(CP_CONV_BAR,    COLOR_CYAN,    -1);
        init_pair(CP_CONV_CUR,    COLOR_YELLOW,  -1);
        init_pair(CP_CONV_GOOD,   COLOR_GREEN,   -1);
        init_pair(CP_CONV_MID,    COLOR_YELLOW,  -1);
        init_pair(CP_CONV_BAD,    COLOR_RED,     -1);
    }
}

/* 9 characters ordered by ink density */
static const char k_heat[] = " .:+x*X#@";

static void heat_cell(float val, float vmax, int *cp_out, char *ch_out)
{
    if (vmax < 1e-12f) { *cp_out = CP_H0; *ch_out = ' '; return; }
    float t = val / vmax;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    /* sqrt for perceptual uniformity */
    t = sqrtf(t);
    int idx = (int)(t * 8.99f);
    if (idx > 8) idx = 8;
    *cp_out = CP_H0 + idx;
    *ch_out = k_heat[idx];
}

/* ===================================================================== */
/* S4  grid state                                                         */
/* ===================================================================== */

/*
 * g_u[l] -- current solution approximation at level l.
 *          Zero on boundary (Dirichlet u=0).
 * g_f[l] -- RHS: at l=0 this is the original source; at l>0 it holds the
 *          restricted fine-level residual (the coarse error equation RHS).
 * g_r[l] -- residual: r = f + Del^2u (we solve -Del^2u = f, so r = f - (-Del^2u)).
 *
 * All in BSS -- no heap allocation.
 */
static float g_u[LEVELS][MAX_PTS];
static float g_f[LEVELS][MAX_PTS];
static float g_r[LEVELS][MAX_PTS];

static void grid_set_source(unsigned seed)
{
    srand(seed);
    int nx = LNX(0), ny = LNY(0);
    float cx = nx * 0.5f, cy = ny * 0.5f;
    float inv2s2 = 1.0f / (2.0f * SRC_SIGMA * SRC_SIGMA);
    /* Two Gaussian bumps of opposite sign for a richer test problem */
    float ox = nx * 0.2f, oy = ny * 0.2f;
    for (int j = 1; j < ny-1; j++) {
        float dy0 = (float)j - cy, dy1 = (float)j - (cy - oy);
        for (int i = 1; i < nx-1; i++) {
            float dx0 = (float)i - cx, dx1 = (float)i - (cx + ox);
            float v0  =  SRC_AMP * expf(-(dx0*dx0 + dy0*dy0) * inv2s2);
            float v1  = -SRC_AMP * 0.6f * expf(-(dx1*dx1 + dy1*dy1) * inv2s2);
            g_f[0][LIDX(0, i, j)] = v0 + v1;
        }
    }
}

static void grid_reset(unsigned seed)
{
    memset(g_u, 0, sizeof g_u);
    memset(g_f, 0, sizeof g_f);
    memset(g_r, 0, sizeof g_r);
    grid_set_source(seed);
}

static float grid_l2norm(int l)
{
    int n = LNX(l) * LNY(l);
    float s = 0.0f;
    for (int k = 0; k < n; k++) s += g_r[l][k] * g_r[l][k];
    return sqrtf(s / (float)n);   /* normalised L2 */
}

static float grid_maxabs(const float *arr, int l)
{
    int n = LNX(l) * LNY(l);
    float mx = 0.0f;
    for (int k = 0; k < n; k++) {
        float a = fabsf(arr[k]);
        if (a > mx) mx = a;
    }
    return mx;
}

/* ===================================================================== */
/* S5  solver (multigrid operators)                                       */
/* ===================================================================== */

/*
 * relax() -- one red-black Gauss-Seidel sweep on level l.
 *
 * Standard 5-point Poisson stencil (h=1):
 *   -Del^2u[i,j] = f[i,j]
 *   => u[i,j] = (nbrs + f[i,j]) / 4
 *
 * Red-black ordering: in pass 0 update cells where (i+j) is even;
 * in pass 1 update odd cells.  Each pass only reads cells updated in
 * the OTHER pass, so all updates within a pass are independent --
 * this is cache-friendly and gives better convergence than pure GS.
 *
 * Error smoothing: each sweep damps high-frequency error components
 * (those with wavelength <= 2h) by roughly 2x.  Low-frequency modes
 * (wavelength >> h) are barely touched -- hence the need for coarser grids.
 */
static void relax(int l)
{
    int nx = LNX(l), ny = LNY(l);
    for (int pass = 0; pass < 2; pass++) {
        for (int j = 1; j < ny-1; j++) {
            for (int i = 1; i < nx-1; i++) {
                if (((i + j) & 1) != pass) continue;
                float nb = g_u[l][LIDX(l, i-1, j)]
                         + g_u[l][LIDX(l, i+1, j)]
                         + g_u[l][LIDX(l, i, j-1)]
                         + g_u[l][LIDX(l, i, j+1)];
                g_u[l][LIDX(l, i, j)] = (nb + g_f[l][LIDX(l, i, j)]) * 0.25f;
            }
        }
    }
}

/*
 * compute_residual() -- compute r[l] = f[l] + Del^2u[l] at every interior cell.
 *
 * Since we solve -Del^2u = f, the residual is r = f - (-Del^2u) = f + Del^2u.
 * With the 5-point stencil: Del^2u[i,j] = nbrs - 4.u[i,j].
 * A large |r| indicates the current u poorly satisfies the equation there.
 */
static void compute_residual(int l)
{
    int nx = LNX(l), ny = LNY(l);
    for (int j = 1; j < ny-1; j++) {
        for (int i = 1; i < nx-1; i++) {
            float lap = g_u[l][LIDX(l, i-1, j)]
                      + g_u[l][LIDX(l, i+1, j)]
                      + g_u[l][LIDX(l, i, j-1)]
                      + g_u[l][LIDX(l, i, j+1)]
                      - 4.0f * g_u[l][LIDX(l, i, j)];
            g_r[l][LIDX(l, i, j)] = g_f[l][LIDX(l, i, j)] + lap;
        }
    }
    /* Zero boundary residual -- boundary conditions are already satisfied */
    for (int i = 0; i < nx; i++) {
        g_r[l][LIDX(l, i,    0)] = 0.0f;
        g_r[l][LIDX(l, i, ny-1)] = 0.0f;
    }
    for (int j = 0; j < ny; j++) {
        g_r[l][LIDX(l,    0, j)] = 0.0f;
        g_r[l][LIDX(l, nx-1, j)] = 0.0f;
    }
}

/*
 * restrict_op() -- full-weighting restriction: residual r[l] -> RHS f[l+1].
 *
 * The coarse-level correction equation is  A_c e_c = r_c, where r_c is
 * the restricted fine residual.  The full-weighting stencil (9-point, 1/16)
 * gives a second-order accurate transfer operator:
 *
 *   f[l+1][I,J] = (1/16) * (  4.r[l][2I,  2J  ]
 *                             +2.r[l][2I+/-1,2J  ]
 *                             +2.r[l][2I,  2J+/-1]
 *                             +1.r[l][2I+/-1,2J+/-1] )
 *
 * We also zero g_u[l+1] so the coarse solve starts from scratch.
 */
static void restrict_op(int l)
{
    int cnx = LNX(l+1), cny = LNY(l+1);
    memset(g_u[l+1], 0, sizeof(float) * (size_t)(cnx * cny));
    memset(g_f[l+1], 0, sizeof(float) * (size_t)(cnx * cny));

    for (int cj = 1; cj < cny-1; cj++) {
        for (int ci = 1; ci < cnx-1; ci++) {
            int fi = 2*ci, fj = 2*cj;
            float v = 4.0f * g_r[l][LIDX(l, fi,   fj  )]
                    + 2.0f * g_r[l][LIDX(l, fi-1, fj  )]
                    + 2.0f * g_r[l][LIDX(l, fi+1, fj  )]
                    + 2.0f * g_r[l][LIDX(l, fi,   fj-1)]
                    + 2.0f * g_r[l][LIDX(l, fi,   fj+1)]
                    + 1.0f * g_r[l][LIDX(l, fi-1, fj-1)]
                    + 1.0f * g_r[l][LIDX(l, fi+1, fj-1)]
                    + 1.0f * g_r[l][LIDX(l, fi-1, fj+1)]
                    + 1.0f * g_r[l][LIDX(l, fi+1, fj+1)];
            g_f[l+1][LIDX(l+1, ci, cj)] = v * (1.0f / 16.0f);
        }
    }
}

/*
 * prolong_op() -- bilinear prolongation: correction g_u[l+1] -> g_u[l].
 *
 * After solving the coarse error equation, the coarse correction e_c
 * is interpolated back to the fine grid and added to u[l]:
 *
 *   At coincident points   (2I,   2J  ) : += e[I,J]
 *   At x-midpoints         (2I+1, 2J  ) : += 0.5*(e[I,J]+e[I+1,J])
 *   At y-midpoints         (2I,   2J+1) : += 0.5*(e[I,J]+e[I,J+1])
 *   At xy-midpoints        (2I+1, 2J+1) : += 0.25*(e[I,J]+e[I+1,J]+e[I,J+1]+e[I+1,J+1])
 *
 * This is the standard piecewise-bilinear interpolation operator P.
 * Its transpose is the restriction operator R = (1/4).P^T.
 */
static void prolong_op(int l)
{
    /* Coarse level: l+1.  Fine level: l. */
    int cnx = LNX(l+1), cny = LNY(l+1);
    int fnx = LNX(l),   fny = LNY(l);

    for (int cj = 0; cj < cny-1; cj++) {
        for (int ci = 0; ci < cnx-1; ci++) {
            float c00 = g_u[l+1][LIDX(l+1, ci,   cj  )];
            float c10 = g_u[l+1][LIDX(l+1, ci+1, cj  )];
            float c01 = g_u[l+1][LIDX(l+1, ci,   cj+1)];
            float c11 = g_u[l+1][LIDX(l+1, ci+1, cj+1)];
            int fi = 2*ci, fj = 2*cj;

#define PADD(ix, iy, val) \
    if ((ix) > 0 && (ix) < fnx-1 && (iy) > 0 && (iy) < fny-1) \
        g_u[l][LIDX(l, (ix), (iy))] += (val)

            PADD(fi,   fj,   c00);
            PADD(fi+1, fj,   0.5f*(c00+c10));
            PADD(fi,   fj+1, 0.5f*(c00+c01));
            PADD(fi+1, fj+1, 0.25f*(c00+c10+c01+c11));
#undef PADD
        }
    }
}

/* ===================================================================== */
/* S6  V-cycle state machine                                              */
/* ===================================================================== */

typedef enum {
    VCPHASE_PRE,     /* Gauss-Seidel pre-smoothing, going down  */
    VCPHASE_COARSE,  /* Extra sweeps at coarsest level           */
    VCPHASE_POST,    /* Gauss-Seidel post-smoothing, going up    */
} VCPhase;

typedef struct {
    VCPhase phase;
    int     level;          /* 0 = fine, LEVELS-1 = coarsest    */
    int     smooth_count;   /* sweeps done at current level/phase */
    bool    going_down;

    /* Diagnostics */
    int     cycle_count;
    float   r0_norm;                  /* ||r|| at start of solver     */
    float   rnorm[LEVELS];            /* current ||r|| per level      */
    float   conv_rate;                /* ||r_k|| / ||r_{k-1}||        */
    float   conv_hist[CONV_HIST_LEN]; /* ||r|| after each V-cycle     */
    int     conv_n;

    bool    paused;
    bool    step_req;
} VCycle;

static int sweep_limit(const VCycle *vc)
{
    if (vc->level == LEVELS-1) return COARSE_ITERS;
    return vc->going_down ? PRE_SMOOTH : POST_SMOOTH;
}

static void vcycle_init(VCycle *vc)
{
    memset(vc, 0, sizeof *vc);
    vc->going_down = true;
    compute_residual(0);
    vc->rnorm[0] = grid_l2norm(0);
    vc->r0_norm  = vc->rnorm[0];
    if (vc->conv_n < CONV_HIST_LEN)
        vc->conv_hist[vc->conv_n++] = vc->rnorm[0];
}

/*
 * vcycle_step() -- advance the V-cycle by exactly one action:
 *   either one GS sweep, one restrict, or one prolong.
 *
 * Returns true when a complete V-cycle finishes (useful for resetting
 * the source or logging).
 *
 * State machine diagram:
 *
 *   L0:PRE  --restrict--> L1:PRE  --restrict--> L2:PRE  --restrict--> L3:COARSE
 *                                                                           |
 *   L0:POST <--prolong-- L1:POST <--prolong-- L2:POST <--prolong----------+
 *      |
 *    cycle done, restart
 */
static bool vcycle_step(VCycle *vc)
{
    if (vc->paused && !vc->step_req) return false;
    vc->step_req = false;

    /* One relaxation sweep at current level */
    relax(vc->level);
    compute_residual(vc->level);
    vc->rnorm[vc->level] = grid_l2norm(vc->level);
    vc->smooth_count++;

    if (vc->smooth_count < sweep_limit(vc))
        return false;

    vc->smooth_count = 0;

    if (vc->going_down) {
        if (vc->level < LEVELS-1) {
            /* Restrict fine residual to coarse RHS, descend */
            restrict_op(vc->level);
            vc->level++;
            vc->rnorm[vc->level] = grid_l2norm(vc->level);
        } else {
            /* Reached coarsest: switch to going up */
            vc->going_down = false;
        }
    } else {
        /* Going up: prolong coarse correction into finer level */
        if (vc->level > 0) {
            prolong_op(vc->level - 1);
            vc->level--;
            compute_residual(vc->level);
            vc->rnorm[vc->level] = grid_l2norm(vc->level);
        } else {
            /* Completed a V-cycle */
            vc->cycle_count++;
            float rn = vc->rnorm[0];
            if (vc->conv_n >= 2) {
                float prev = vc->conv_hist[vc->conv_n - 1];
                vc->conv_rate = (prev > 1e-15f) ? (rn / prev) : 0.0f;
            }
            if (vc->conv_n < CONV_HIST_LEN)
                vc->conv_hist[vc->conv_n++] = rn;
            /* Restart */
            vc->going_down = true;
            vc->level      = 0;
            return true;
        }
    }
    return false;
}

/* ===================================================================== */
/* S7  scene                                                              */
/* ===================================================================== */

typedef struct {
    VCycle   vc;
    unsigned seed;
} Scene;

static void scene_init(Scene *sc)
{
    sc->seed = (unsigned)time(NULL);
    grid_reset(sc->seed);
    vcycle_init(&sc->vc);
}

static void scene_reset(Scene *sc)
{
    sc->seed += 9973;
    grid_reset(sc->seed);
    vcycle_init(&sc->vc);
}

static void scene_tick(Scene *sc)
{
    vcycle_step(&sc->vc);
}

/* ===================================================================== */
/* S8  render                                                             */
/* ===================================================================== */

/*
 * Panel x-offset and width for level l, given terminal column count.
 * Each level gets a fraction of columns proportional to its grid width.
 */
static int panel_col(int l, int cols)
{
    int acc = 0;
    for (int k = 0; k < l; k++) acc += LNX(k);
    return (int)((float)acc / (float)TOTAL_LNX * (float)cols);
}
static int panel_width(int l, int cols)
{
    return panel_col(l + 1, cols) - panel_col(l, cols);
}

/*
 * render_grid() -- draw residual heatmap for level l in its terminal panel.
 *
 * Maps logical grid cells (i,j) to terminal cells (px+col, py_start+row)
 * via nearest-neighbour sampling.  Active level gets bright border and
 * shows the current sweep count; inactive levels show a dim border.
 *
 * vmax: maximum |r| across all levels (for consistent color scale).
 */
static void render_grid(const VCycle *vc, int l,
                        int px, int pw, int py, int ph,
                        float vmax)
{
    if (pw < 3 || ph < 3) return;
    int nx = LNX(l), ny = LNY(l);
    bool is_active = (l == vc->level);

    /* -- border --------------------------------------------------- */
    int cp_b = is_active ? CP_BORDER_ACT : CP_BORDER_OFF;
    attr_t ba = COLOR_PAIR(cp_b) | (is_active ? A_BOLD : A_DIM);

    attron(ba);
    /* Top border */
    for (int c = px; c < px + pw && c < COLS; c++)
        mvaddch(py, c, '-');
    /* Label: level index, grid size, current |r| */
    char lbl[32];
    snprintf(lbl, sizeof lbl, "L%d %dx%d |r|=%.1e",
             l, nx-2, ny-2, (double)vc->rnorm[l]);
    mvprintw(py, px + 1, "%.*s", pw - 2, lbl);

    /* Left and right edge ticks */
    for (int r = 1; r < ph; r++) {
        mvaddch(py + r, px,        '|');
        if (px + pw - 1 < COLS)
            mvaddch(py + r, px + pw - 1, '|');
    }
    attroff(ba);

    /* -- heatmap content ------------------------------------------- */
    int content_w = pw - 2;    /* inside the | borders */
    int content_h = ph - 1;    /* below the top border */
    if (content_w < 1 || content_h < 1) return;

    for (int row = 0; row < content_h; row++) {
        /* Map terminal row -> logical j (interior only: 1..ny-2) */
        int j = 1 + (int)((float)row / (float)content_h * (float)(ny - 2));
        if (j < 1)    j = 1;
        if (j > ny-2) j = ny-2;

        for (int col = 0; col < content_w; col++) {
            int i = 1 + (int)((float)col / (float)content_w * (float)(nx - 2));
            if (i < 1)    i = 1;
            if (i > nx-2) i = nx-2;

            float val = fabsf(g_r[l][LIDX(l, i, j)]);
            int cp; char ch;
            heat_cell(val, vmax, &cp, &ch);
            attron(COLOR_PAIR(cp));
            mvaddch(py + 1 + row, px + 1 + col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    /* Phase label in top-right of active panel */
    if (is_active) {
        const char *phase_str;
        int cp_ph;
        if (vc->level == LEVELS-1) {
            phase_str = "COARSE";  cp_ph = CP_PHASE_COARSE;
        } else if (vc->going_down) {
            phase_str = "PRE-SM";  cp_ph = CP_PHASE_DOWN;
        } else {
            phase_str = "POST-S";  cp_ph = CP_PHASE_UP;
        }
        int label_col = px + pw - 1 - (int)strlen(phase_str) - 1;
        if (label_col > px + 1) {
            attron(COLOR_PAIR(cp_ph) | A_BOLD);
            mvprintw(py + 1, label_col, "%s", phase_str);
            attroff(COLOR_PAIR(cp_ph) | A_BOLD);
        }
        /* Sweep progress bar */
        int lim = sweep_limit(vc);
        if (lim > 0 && content_w > 4) {
            int filled = (int)((float)vc->smooth_count / (float)lim * (float)(content_w));
            attron(COLOR_PAIR(cp_ph));
            for (int c = 0; c < filled && c < content_w; c++)
                mvaddch(py + 2, px + 1 + c, '=');
            attroff(COLOR_PAIR(cp_ph));
        }
    }
}

/*
 * render_vcycle_hud() -- ASCII V-shape diagram showing cycle position.
 *
 * Renders something like:
 *   L0 >-> L1 >-> L2 >-> L3
 *               ^              (current position marked)
 *   L0 <-< L1 <-< L2 <-< L3
 */
static void render_vcycle_hud(const VCycle *vc, int row, int col, int max_w)
{
    char buf[256];
    int  pos = 0;

    /* Build top row: L0 > L1 > L2 > L3 */
    for (int l = 0; l < LEVELS; l++) {
        bool act = (vc->going_down && l == vc->level);
        if (act) {
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, "[L%d]", l);
        } else {
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, " L%d ", l);
        }
        if (l < LEVELS-1)
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, "-->");
    }
    attron(COLOR_PAIR(CP_PHASE_DOWN) | A_BOLD);
    mvprintw(row, col, "%.*s", max_w, buf);
    attroff(COLOR_PAIR(CP_PHASE_DOWN) | A_BOLD);

    /* Build bottom row: L0 < L1 < L2 < L3 */
    pos = 0;
    for (int l = 0; l < LEVELS; l++) {
        bool act = (!vc->going_down && l == vc->level);
        if (act) {
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, "[L%d]", l);
        } else {
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, " L%d ", l);
        }
        if (l < LEVELS-1)
            pos += snprintf(buf + pos, sizeof buf - (size_t)pos, "<--");
    }
    attron(COLOR_PAIR(CP_PHASE_UP) | A_BOLD);
    mvprintw(row + 1, col, "%.*s", max_w, buf);
    attroff(COLOR_PAIR(CP_PHASE_UP) | A_BOLD);
}

/*
 * render_conv_plot() -- bar chart of log||r|| after each V-cycle.
 * Tallest bar = lowest residual = most converged.
 */
static void render_conv_plot(const VCycle *vc, int row, int col, int pw, int ph)
{
    if (vc->conv_n < 1 || pw < 4 || ph < 3) return;

    /* Log residual range */
    float lmax = -1e30f, lmin = 1e30f;
    for (int k = 0; k < vc->conv_n; k++) {
        float rn = vc->conv_hist[k];
        if (rn > 1e-15f) {
            float lr = log10f(rn);
            if (lr > lmax) lmax = lr;
            if (lr < lmin) lmin = lr;
        }
    }
    if (lmax - lmin < 0.5f) lmin = lmax - 1.5f;

    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(row, col, "log||r||");
    attroff(COLOR_PAIR(CP_LABEL));

    int bar_ph = ph - 2;

    for (int k = 0; k < vc->conv_n && k < CONV_HIST_LEN; k++) {
        float rn = (vc->conv_hist[k] > 1e-15f) ? vc->conv_hist[k] : 1e-15f;
        float lr = log10f(rn);
        float t  = (lmax > lmin) ? (lmax - lr) / (lmax - lmin) : 1.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        int h = 1 + (int)(t * (float)(bar_ph - 1));
        if (h > bar_ph) h = bar_ph;

        int bc = col + 1 + (int)((float)k / (float)(CONV_HIST_LEN - 1) * (float)(pw - 3));
        if (bc >= col + pw) continue;

        bool is_cur = (k == vc->conv_n - 1);
        int  cp     = is_cur ? CP_CONV_CUR : CP_CONV_BAR;
        attron(COLOR_PAIR(cp) | A_BOLD);
        for (int r = 0; r < h; r++)
            mvaddch(row + 1 + bar_ph - r, bc, '|');
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(row + 1,           col, "%.0f", (double)lmax);
    mvprintw(row + 1 + bar_ph,  col, "%.0f", (double)lmin);
    attroff(COLOR_PAIR(CP_LABEL));
}

static void render_overlay(const Scene *sc, int rows, int cols)
{
    const VCycle *vc = &sc->vc;
    int hud_row = rows - HUD_ROWS;

    /* Divider */
    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(hud_row, c, '=');
    attroff(COLOR_PAIR(CP_LABEL));

    /* Line 1: cycle/phase/sweep */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hud_row + 1, 1,
        " V-cycle: %3d   level: L%d   sweep: %d/%d   ||r0||: %.3e   ||r_cur||: %.3e",
        vc->cycle_count, vc->level,
        vc->smooth_count, sweep_limit(vc),
        (double)vc->r0_norm,
        (double)vc->rnorm[0]);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Line 2: convergence rate */
    int cp_rate = CP_CONV_MID;
    if (vc->conv_rate > 0.0f && vc->conv_rate < 0.2f) cp_rate = CP_CONV_GOOD;
    else if (vc->conv_rate >= 0.5f) cp_rate = CP_CONV_BAD;

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud_row + 2, 1, " conv rate rho = ");
    attroff(COLOR_PAIR(CP_HUD));
    if (vc->conv_n >= 2) {
        attron(COLOR_PAIR(cp_rate) | A_BOLD);
        printw("%.4f", (double)vc->conv_rate);
        attroff(COLOR_PAIR(cp_rate) | A_BOLD);
        attron(COLOR_PAIR(CP_HUD));
        printw(" (ideal < 0.2; each cycle reduces ||r|| by %.0f%%)",
               (double)((1.0f - vc->conv_rate) * 100.0f));
        attroff(COLOR_PAIR(CP_HUD));
    } else {
        attron(COLOR_PAIR(CP_HUD)); printw("---"); attroff(COLOR_PAIR(CP_HUD));
    }

    /* V-cycle diagram + convergence plot side by side */
    render_vcycle_hud(vc, hud_row + 3, 1, cols / 2 - 2);
    render_conv_plot(vc, hud_row + 3, cols / 2, cols / 2 - 1, 3);

    /* Explanation lines */
    attron(COLOR_PAIR(CP_EXPLAIN));
    mvprintw(hud_row + 5, 1,
        " Error smoothing: GS damps high-freq error fast (2x per sweep) but is"
        " slow on smooth/broad error modes.");
    mvprintw(hud_row + 6, 1,
        " Grid hierarchy: coarse grid sees smooth fine error as high-freq ->"
        " kills it cheaply (1/4 the work).");
    mvprintw(hud_row + 7, 1,
        " Heat map: |residual| per cell -- dark blue = near solved, red = large"
        " error. V-cycle collapses all scales.");
    attroff(COLOR_PAIR(CP_EXPLAIN));

    /* Controls */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(hud_row + 8, 1,
        " SPACE pause   s step   r reset   +/- speed   q quit");
    attroff(COLOR_PAIR(CP_LABEL));
}

static void render_header(const Scene *sc, int cols)
{
    (void)sc;
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(0, 0,
        " Multigrid V-cycle Poisson Solver  -Del^2u = f"
        "   %d levels: %dx%d -> %dx%d -> %dx%d -> %dx%d",
        LEVELS,
        LNX(0)-2, LNY(0)-2, LNX(1)-2, LNY(1)-2,
        LNX(2)-2, LNY(2)-2, LNX(3)-2, LNY(3)-2);
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(1, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));
}

/* ===================================================================== */
/* S9  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(const Screen *s, const Scene *sc)
{
    int rows = s->rows, cols = s->cols;
    erase();

    render_header(sc, cols);

    /* Grid panels fill rows 2 .. rows-HUD_ROWS-1 */
    int py = 2;
    int ph = rows - HUD_ROWS - py;
    if (ph < 4) ph = 4;

    /* Global max |r| for consistent heatmap scale */
    float vmax = 0.0f;
    for (int l = 0; l < LEVELS; l++) {
        float mx = grid_maxabs(g_r[l], l);
        if (mx > vmax) vmax = mx;
    }
    if (vmax < 1e-12f) vmax = 1e-12f;

    for (int l = 0; l < LEVELS; l++) {
        int px = panel_col(l, cols);
        int pw = panel_width(l, cols);
        render_grid(&sc->vc, l, px, pw, py, ph, vmax);
    }

    render_overlay(sc, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* S10 app -- signals, resize, input, main loop                           */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void cleanup(void)              { endwin(); }
static void on_exit(int sig)           { (void)sig; g_app.running    = 0; }
static void on_resize(int sig)         { (void)sig; g_app.need_resize = 1; }

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':
        app->scene.vc.paused = !app->scene.vc.paused;
        break;
    case 's': case 'S':
        app->scene.vc.paused   = true;
        app->scene.vc.step_req = true;
        break;
    case 'r': case 'R':
        scene_reset(&app->scene);
        break;
    case '+': case '=':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps++;
        break;
    case '-': case '_':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps--;
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
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            frame_time       = clock_ns();
            sim_accum        = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count /
                          ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }
        (void)fps_display;

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        screen_draw(&app->screen, &app->scene);

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    return 0;
}
