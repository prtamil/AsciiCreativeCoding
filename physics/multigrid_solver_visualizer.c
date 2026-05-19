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
 *  Screen layout:
 *    +--------------------------------------------------------------------+
 *    | TOP HUD   row 0: title + V-cycle counter                           |
 *    |           row 1: L | sweep | ||r0||->||r_cur|| | rho               |
 *    |           row 2: L0 >>> *L1* >>> L2 >>> L3   (position bar)        |
 *    +--------------------------------------------------------------------+
 *    | PANELS:  active level gets WIDE_FRAC (~62%) of cols, the other     |
 *    | three share the rest equally as thumbnails.  Each panel renders    |
 *    | |residual| as a heat map.  Panels morph live as the V-cycle moves. |
 *    +--------------------------------------------------------------------+
 *    | PHASE BANNER  full-width directional: >>> PRE-SMOOTH, === COARSE,  |
 *    |                                       <<< POST-SMOOTH              |
 *    | BOTTOM HUD    single line of key bindings                          |
 *    +--------------------------------------------------------------------+
 *
 * -------------------------------------------------------------------------
 *  What you are looking at on screen
 * -------------------------------------------------------------------------
 *
 *  Active-panel spotlight (big block).
 *    The wide panel is whichever level the solver is currently working on.
 *    It morphs live: as the V-cycle descends L0 -> L1 -> L2 -> L3, the
 *    spotlight marches right; on the way back up it marches left.  When L0
 *    is active you see fine 64x32 detail; when L3 is active each cell is
 *    a chunky 8x4 block -- the same residual array, just zoomed by 8x.
 *
 *  Thumbnails (three skinny panels).
 *    The non-active levels, kept on-screen for context.  Even though we
 *    only relax on the active level, you can watch the others get
 *    populated (during restrict) and consumed (during prolong).
 *
 *  Heat map (every panel).
 *    |residual| per cell.  Dark navy = near solved; bright red/magenta =
 *    large local error.  At t=0 the map glows where the Gaussian source
 *    sits.  Convergence is literally the screen going dark.
 *
 *  Phase banner (bright bar just above the key-binding row).
 *    Full-width arrows that announce both phase and direction:
 *       >>>>  PRE-SMOOTH    descending; smoothing on a finer level
 *       ====  COARSE SOLVE  parked at the coarsest level for extra sweeps
 *       <<<<  POST-SMOOTH   ascending; smoothing after the prolong step
 *
 *  V-cycle position bar (top HUD, row 2).
 *    Marks the active level among all four:  L0 >>> *L1* >>> L2 >>> L3.
 *    The star moves left-to-right and back once per complete V-cycle.
 *
 *  Numeric status (top HUD, row 1).
 *    rho       convergence rate.  Healthy multigrid stays GREEN at
 *              0.05-0.15 (each cycle kills 85-95% of the residual).
 *              YELLOW = sluggish; RED (>= 0.5) = something is wrong with
 *              the operator or the boundary conditions.
 *    ||r0||    initial residual norm at solver start; baseline for "how
 *              much error did we start with."
 *    ||r_cur|| current residual norm.  Should drop ~10x every 1-2 cycles
 *              and reach float32 precision (~1e-7) in roughly 10 cycles.
 *
 *  V-cycle counter (top HUD, row 0 right).
 *    Number of complete V-cycles since the last reset.
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
 *  S8  render      -- compute_panels(), render_grid(), render_phase_banner(),
 *                     render_top_hud() (status), render_bottom_hud() (actions)
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
 *   meaning each V-cycle reduces ||r|| by at least 80%.  The top HUD shows
 *   the live rho = ||r_k|| / ||r_{k-1}||, colored green/yellow/red.
 *
 * -------------------------------------------------------------------------
 *  References
 * -------------------------------------------------------------------------
 *  Multigrid algorithm:
 *
 *  [1] Briggs, Henson, McCormick.  "A Multigrid Tutorial," 2nd ed.
 *      SIAM, 2000.  -- The canonical introduction.  Chapters 2-4 cover every
 *      operator in S5 (Gauss-Seidel smoothing, full-weighting restriction,
 *      bilinear prolongation, V-cycle schedule).  Start here.
 *
 *  [2] Trottenberg, Oosterlee, Schueller.  "Multigrid."
 *      Academic Press, 2001.  -- Comprehensive reference.  Section 2.3 derives
 *      restriction/prolongation as adjoint operators (R = (1/4) P^T).  Section
 *      3 covers smoothing-factor analysis -- the theory behind rho < 0.2.
 *
 *  [3] Brandt, A.  "Multi-Level Adaptive Solutions to Boundary-Value Problems."
 *      Math. Comp. 31(138), 333-390, 1977.  -- The original paper.  Worth
 *      reading for the historical framing: how the O(N log N) -> O(N) jump
 *      came about and why "smooth error looks oscillatory on coarse grids".
 *
 *  [4] Hackbusch, W.  "Multi-Grid Methods and Applications."
 *      Springer, 1985.  -- Theoretical foundations.  Convergence proofs for the
 *      V-cycle on elliptic PDEs (the regime our Poisson problem sits in).
 *
 *  [5] Press, Teukolsky, Vetterling, Flannery.  "Numerical Recipes," 3rd ed.,
 *      Section 20.6 (Multigrid Methods for Boundary Value Problems).
 *      -- Practical C implementation; closest in spirit to this file's code.
 *
 *  Underlying PDE / discretization:
 *
 *  [6] LeVeque, R. J.  "Finite Difference Methods for Ordinary and Partial
 *      Differential Equations."  SIAM, 2007.  Ch. 3.  -- 5-point Poisson
 *      stencil, Dirichlet boundary conditions, second-order accuracy.
 *
 *  Rendering:
 *
 *  [7] Kovesi, P.  "Good Colour Maps: How to Design Them."
 *      arXiv:1509.03700, 2015.  -- Perceptual uniformity in heat maps.  The
 *      sqrt(t) warp in S3 heat_cell() approximates perceptual lightness, so
 *      equal residual differences appear as equal visual differences.
 *
 *  [8] Bourke, P.  "Character representation of grey scale images."
 *      paulbourke.net/dataformats/asciiart/  -- The tradition of ordering
 *      ASCII glyphs by ink density.  Our 9-char ramp " .:+x*X#@" is in this
 *      family.
 *
 * ----------------------------------------------------------------------- */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* S1  config                                                             */
/* ===================================================================== */

#define SIM_FPS_DEFAULT 6
#define SIM_FPS_MIN 1
#define SIM_FPS_MAX 30
#define TARGET_FPS 60
#define FPS_UPDATE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Grid hierarchy */
#define LEVELS 4
#define L0_NX 64                /* fine grid width  (boundary-inclusive) */
#define L0_NY 32                /* fine grid height (boundary-inclusive) */
#define MAX_PTS (L0_NX * L0_NY) /* 2048 -- largest level */

/* Grid dimensions at level l */
#define LNX(l) (L0_NX >> (l))
#define LNY(l) (L0_NY >> (l))
#define LIDX(l, x, y) ((y) * LNX(l) + (x))

/* V-cycle sweep counts */
#define PRE_SMOOTH 2
#define POST_SMOOTH 2
#define COARSE_ITERS 20 /* extra sweeps at coarsest level */

/* Source term: Gaussian bump at fine-grid centre */
#define SRC_AMP 8.0f
#define SRC_SIGMA 4.0f

/* Convergence history ring length (entries) */
#define CONV_HIST_LEN 32

/* HUD layout: status zone on top, action zone on bottom.
 *   TOP    = title row, status numbers, V-cycle position bar
 *   BOTTOM = phase banner + single action-key row
 */
#define TOP_HUD_ROWS 3
#define BOTTOM_HUD_ROWS 2

/* Spotlight layout: fraction of cols allocated to the active level.
 * Remaining (1-WIDE_FRAC) is split equally among the LEVELS-1 thumbnails.
 * 0.62 ~ golden ratio: active panel reads as the focal point. */
#define WIDE_FRAC 0.62f

/* ===================================================================== */
/* S2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* S3  color                                                              */
/* ===================================================================== */

enum {
  CP_NONE = 0,
  /* Heat-map ramp: 9 levels, low -> high residual */
  CP_H0,
  CP_H1,
  CP_H2,
  CP_H3,
  CP_H4,
  CP_H5,
  CP_H6,
  CP_H7,
  CP_H8,
  /* UI */
  CP_BORDER_ACT,   /* active level border   */
  CP_BORDER_OFF,   /* inactive level border */
  CP_HUD,          /* HUD text              */
  CP_HEADER,       /* header row            */
  CP_LABEL,        /* action key bindings   */
  CP_PHASE_DOWN,   /* pre-smooth going down */
  CP_PHASE_UP,     /* post-smooth going up  */
  CP_PHASE_COARSE, /* coarse solve          */
  CP_CONV_GOOD,    /* rate < 0.2            */
  CP_CONV_MID,     /* rate 0.2-0.5          */
  CP_CONV_BAD,     /* rate > 0.5            */
};

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    /* Heat: dark navy -> blue -> cyan -> green -> yellow -> orange -> red */
    init_pair(CP_H0, 25, -1);
    init_pair(CP_H1, 27, -1);
    init_pair(CP_H2, 27, -1);
    init_pair(CP_H3, 39, -1);
    init_pair(CP_H4, 46, -1);
    init_pair(CP_H5, 226, -1);
    init_pair(CP_H6, 208, -1);
    init_pair(CP_H7, 196, -1);
    init_pair(CP_H8, 201, -1);
    init_pair(CP_BORDER_ACT, 51, -1);
    init_pair(CP_BORDER_OFF, 246, -1);
    init_pair(CP_HUD, 252, -1);
    init_pair(CP_HEADER, 51, -1);
    init_pair(CP_LABEL, 244, -1);
    init_pair(CP_PHASE_DOWN, 201, -1);
    init_pair(CP_PHASE_UP, 46, -1);
    init_pair(CP_PHASE_COARSE, 226, -1);
    init_pair(CP_CONV_GOOD, 46, -1);
    init_pair(CP_CONV_MID, 226, -1);
    init_pair(CP_CONV_BAD, 196, -1);
  } else {
    init_pair(CP_H0, COLOR_BLUE, -1);
    init_pair(CP_H1, COLOR_BLUE, -1);
    init_pair(CP_H2, COLOR_BLUE, -1);
    init_pair(CP_H3, COLOR_CYAN, -1);
    init_pair(CP_H4, COLOR_GREEN, -1);
    init_pair(CP_H5, COLOR_GREEN, -1);
    init_pair(CP_H6, COLOR_YELLOW, -1);
    init_pair(CP_H7, COLOR_RED, -1);
    init_pair(CP_H8, COLOR_RED, -1);
    init_pair(CP_BORDER_ACT, COLOR_CYAN, -1);
    init_pair(CP_BORDER_OFF, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_WHITE, -1);
    init_pair(CP_HEADER, COLOR_CYAN, -1);
    init_pair(CP_LABEL, COLOR_WHITE, -1);
    init_pair(CP_PHASE_DOWN, COLOR_MAGENTA, -1);
    init_pair(CP_PHASE_UP, COLOR_GREEN, -1);
    init_pair(CP_PHASE_COARSE, COLOR_YELLOW, -1);
    init_pair(CP_CONV_GOOD, COLOR_GREEN, -1);
    init_pair(CP_CONV_MID, COLOR_YELLOW, -1);
    init_pair(CP_CONV_BAD, COLOR_RED, -1);
  }
}

/* 9 characters ordered by ink density */
static const char k_heat[] = " .:+x*X#@";

static void heat_cell(float val, float vmax, int *cp_out, char *ch_out) {
  if (vmax < 1e-12f) {
    *cp_out = CP_H0;
    *ch_out = ' ';
    return;
  }
  float t = val / vmax;
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  /* sqrt for perceptual uniformity */
  t = sqrtf(t);
  int idx = (int)(t * 8.99f);
  if (idx > 8)
    idx = 8;
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
 * g_r[l] -- residual: r = f + Del^2u (we solve -Del^2u = f, so r = f -
 * (-Del^2u)).
 *
 * All in BSS -- no heap allocation.
 */
static float g_u[LEVELS][MAX_PTS];
static float g_f[LEVELS][MAX_PTS];
static float g_r[LEVELS][MAX_PTS];

static void grid_set_source(unsigned seed) {
  srand(seed);
  int nx = LNX(0), ny = LNY(0);
  float cx = nx * 0.5f, cy = ny * 0.5f;
  float inv2s2 = 1.0f / (2.0f * SRC_SIGMA * SRC_SIGMA);
  /* Two Gaussian bumps of opposite sign for a richer test problem */
  float ox = nx * 0.2f, oy = ny * 0.2f;
  for (int j = 1; j < ny - 1; j++) {
    float dy0 = (float)j - cy, dy1 = (float)j - (cy - oy);
    for (int i = 1; i < nx - 1; i++) {
      float dx0 = (float)i - cx, dx1 = (float)i - (cx + ox);
      float v0 = SRC_AMP * expf(-(dx0 * dx0 + dy0 * dy0) * inv2s2);
      float v1 = -SRC_AMP * 0.6f * expf(-(dx1 * dx1 + dy1 * dy1) * inv2s2);
      g_f[0][LIDX(0, i, j)] = v0 + v1;
    }
  }
}

static void grid_reset(unsigned seed) {
  memset(g_u, 0, sizeof g_u);
  memset(g_f, 0, sizeof g_f);
  memset(g_r, 0, sizeof g_r);
  grid_set_source(seed);
}

static float grid_l2norm(int l) {
  int n = LNX(l) * LNY(l);
  float s = 0.0f;
  for (int k = 0; k < n; k++)
    s += g_r[l][k] * g_r[l][k];
  return sqrtf(s / (float)n); /* normalised L2 */
}

static float grid_maxabs(const float *arr, int l) {
  int n = LNX(l) * LNY(l);
  float mx = 0.0f;
  for (int k = 0; k < n; k++) {
    float a = fabsf(arr[k]);
    if (a > mx)
      mx = a;
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
static void relax(int l) {
  int nx = LNX(l), ny = LNY(l);
  for (int pass = 0; pass < 2; pass++) {
    for (int j = 1; j < ny - 1; j++) {
      for (int i = 1; i < nx - 1; i++) {
        if (((i + j) & 1) != pass)
          continue;
        float nb = g_u[l][LIDX(l, i - 1, j)] + g_u[l][LIDX(l, i + 1, j)] +
                   g_u[l][LIDX(l, i, j - 1)] + g_u[l][LIDX(l, i, j + 1)];
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
static void compute_residual(int l) {
  int nx = LNX(l), ny = LNY(l);
  for (int j = 1; j < ny - 1; j++) {
    for (int i = 1; i < nx - 1; i++) {
      float lap = g_u[l][LIDX(l, i - 1, j)] + g_u[l][LIDX(l, i + 1, j)] +
                  g_u[l][LIDX(l, i, j - 1)] + g_u[l][LIDX(l, i, j + 1)] -
                  4.0f * g_u[l][LIDX(l, i, j)];
      g_r[l][LIDX(l, i, j)] = g_f[l][LIDX(l, i, j)] + lap;
    }
  }
  /* Zero boundary residual -- boundary conditions are already satisfied */
  for (int i = 0; i < nx; i++) {
    g_r[l][LIDX(l, i, 0)] = 0.0f;
    g_r[l][LIDX(l, i, ny - 1)] = 0.0f;
  }
  for (int j = 0; j < ny; j++) {
    g_r[l][LIDX(l, 0, j)] = 0.0f;
    g_r[l][LIDX(l, nx - 1, j)] = 0.0f;
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
static void restrict_op(int l) {
  int cnx = LNX(l + 1), cny = LNY(l + 1);
  memset(g_u[l + 1], 0, sizeof(float) * (size_t)(cnx * cny));
  memset(g_f[l + 1], 0, sizeof(float) * (size_t)(cnx * cny));

  for (int cj = 1; cj < cny - 1; cj++) {
    for (int ci = 1; ci < cnx - 1; ci++) {
      int fi = 2 * ci, fj = 2 * cj;
      float v = 4.0f * g_r[l][LIDX(l, fi, fj)] +
                2.0f * g_r[l][LIDX(l, fi - 1, fj)] +
                2.0f * g_r[l][LIDX(l, fi + 1, fj)] +
                2.0f * g_r[l][LIDX(l, fi, fj - 1)] +
                2.0f * g_r[l][LIDX(l, fi, fj + 1)] +
                1.0f * g_r[l][LIDX(l, fi - 1, fj - 1)] +
                1.0f * g_r[l][LIDX(l, fi + 1, fj - 1)] +
                1.0f * g_r[l][LIDX(l, fi - 1, fj + 1)] +
                1.0f * g_r[l][LIDX(l, fi + 1, fj + 1)];
      g_f[l + 1][LIDX(l + 1, ci, cj)] = v * (1.0f / 16.0f);
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
 *   At xy-midpoints        (2I+1, 2J+1) : +=
 * 0.25*(e[I,J]+e[I+1,J]+e[I,J+1]+e[I+1,J+1])
 *
 * This is the standard piecewise-bilinear interpolation operator P.
 * Its transpose is the restriction operator R = (1/4).P^T.
 */
static void prolong_op(int l) {
  /* Coarse level: l+1.  Fine level: l. */
  int cnx = LNX(l + 1), cny = LNY(l + 1);
  int fnx = LNX(l), fny = LNY(l);

  for (int cj = 0; cj < cny - 1; cj++) {
    for (int ci = 0; ci < cnx - 1; ci++) {
      float c00 = g_u[l + 1][LIDX(l + 1, ci, cj)];
      float c10 = g_u[l + 1][LIDX(l + 1, ci + 1, cj)];
      float c01 = g_u[l + 1][LIDX(l + 1, ci, cj + 1)];
      float c11 = g_u[l + 1][LIDX(l + 1, ci + 1, cj + 1)];
      int fi = 2 * ci, fj = 2 * cj;

#define PADD(ix, iy, val)                                                      \
  if ((ix) > 0 && (ix) < fnx - 1 && (iy) > 0 && (iy) < fny - 1)                \
  g_u[l][LIDX(l, (ix), (iy))] += (val)

      PADD(fi, fj, c00);
      PADD(fi + 1, fj, 0.5f * (c00 + c10));
      PADD(fi, fj + 1, 0.5f * (c00 + c01));
      PADD(fi + 1, fj + 1, 0.25f * (c00 + c10 + c01 + c11));
#undef PADD
    }
  }
}

/* ===================================================================== */
/* S6  V-cycle state machine                                              */
/* ===================================================================== */

/*
 * VCPhase -- legacy phase tags for the V-cycle state machine.
 *
 * VESTIGIAL: declared for documentation value but never set or read on
 * the live code path.  The current phase is computed on the fly from
 *   (going_down, level == LEVELS-1)
 * in sweep_limit() and in phase_glyphs() (S8) -- the latter is the
 * shared classifier read by the banner, the in-panel sweep bar, and
 * the top-HUD position row.  The enum names below are kept because
 * they spell out the three phases the algorithm passes through:
 *
 *   PRE     pre-smoothing on a level on the way DOWN
 *   COARSE  bottom-of-V; extra sweeps at the coarsest level
 *   POST    post-smoothing on a level on the way UP
 *
 * Reference:
 *   Briggs, Henson, McCormick.  A Multigrid Tutorial, 2nd ed., SIAM 2000.
 *   Algorithm 3.1 (V-cycle pseudocode), Ch. 3.
 */
typedef enum {
  VCPHASE_PRE,    /* Gauss-Seidel pre-smoothing, going down   */
  VCPHASE_COARSE, /* Extra sweeps at coarsest level           */
  VCPHASE_POST,   /* Gauss-Seidel post-smoothing, going up    */
} VCPhase;

/*
 * VCycle -- the V-cycle state machine.
 *
 * INTENT: hold everything needed to advance the multigrid solver by
 * exactly ONE "action" per simulation tick, where an action is one of
 *   - one Gauss-Seidel sweep at the current level
 *   - one restrict (descend a level)
 *   - one prolong  (ascend a level)
 *
 * The actual numerical state (u, f, r grids) lives in S4 BSS globals.
 * VCycle is purely the SCHEDULER + the DIAGNOSTIC readout shown in the
 * top HUD.  Keeping the grid arrays out of this struct is deliberate:
 * they are ~8 KB each at L0 and only S5 touches them directly, so we do
 * not want every render function carrying that bulk by reference.
 *
 * Members grouped by purpose:
 *
 *   CONTROL STATE -- what the scheduler is currently doing.
 *     level         current grid index; 0 = fine (L0_NX x L0_NY),
 *                   LEVELS-1 = coarsest.  Mutated by restrict (++)
 *                   and prolong (--) transitions.
 *     going_down    direction of V-cycle traversal.
 *                     true  -> pre-smoothing; restrict next on completion
 *                     false -> post-smoothing; prolong next on completion
 *     smooth_count  Gauss-Seidel sweeps done at the current
 *                   (level, going_down) tuple.  Compared against
 *                   sweep_limit(vc) to decide when to transition.
 *
 *   DIAGNOSTICS -- numbers the top HUD displays.
 *     cycle_count   complete V-cycles since last reset.  Incremented in
 *                   record_cycle_convergence() when post-smoothing lands
 *                   back at L0.
 *     r0_norm       ||r||_2 at solver start; baseline for "how much
 *                   error did we start with."  Computed once in
 *                   vcycle_init().
 *     rnorm[]       per-level current ||r||_2.  rnorm[0] is what the
 *                   top HUD shows; rnorm[l>0] is kept so future debug
 *                   panels can surface mid-cycle residual at each
 *                   level without recomputing.
 *     conv_rate     rho = ||r_k||_2 / ||r_{k-1}||_2, the geometric
 *                   reduction per completed V-cycle.  Healthy multigrid
 *                   stays at rho ~ 0.05..0.15 (Briggs Ch. 5, Tab. 5.1).
 *                   Color-coded in render_top_hud: green/yellow/red.
 *     conv_hist[]   ring of historical ||r||_2 after each completed
 *                   V-cycle.  Only conv_hist[conv_n - 1] is read (for
 *                   the rate computation against the new ||r||); rest
 *                   are dead-write since the convergence bar chart was
 *                   removed.  Could collapse to a single float prev_rn.
 *     conv_n        number of valid entries in conv_hist.  Gates the
 *                   rate computation -- need >= 2 cycles to ratio.
 *
 *   UI FLAGS -- read by solver_should_advance(), written by
 *               app_handle_key().
 *     paused        true while SPACE-paused; vcycle_step() returns early
 *                   without advancing.
 *     step_req      one-shot: 's' sets it, solver_should_advance() drains
 *                   it to let exactly one action through while paused.
 *
 *   LEGACY:
 *     phase         see VCPhase comment above; never set or read.
 *
 * References:
 *   [1] Briggs et al., A Multigrid Tutorial, 2nd ed., Ch. 3 -- V-cycle
 *       algorithm.  This struct is the data carrier for the loop
 *       variables of Algorithm 3.1.
 *   [2] Trottenberg et al., Multigrid, Ch. 2.4 -- formal statement of
 *       the V-cycle scheme; matches the (level, going_down) transitions
 *       handled by vcycle_step().
 */
typedef struct {
  VCPhase phase;
  int level;        /* 0 = fine, LEVELS-1 = coarsest      */
  int smooth_count; /* sweeps done at current level/phase */
  bool going_down;  /* V-cycle traversal direction        */

  /* Diagnostics */
  int cycle_count;                /* complete V-cycles since reset */
  float r0_norm;                  /* ||r||_2 at solver start       */
  float rnorm[LEVELS];            /* current ||r||_2 per level     */
  float conv_rate;                /* rho = ||r_k|| / ||r_{k-1}||   */
  float conv_hist[CONV_HIST_LEN]; /* ||r||_2 after each V-cycle    */
  int conv_n;                     /* valid entries in conv_hist    */

  bool paused;   /* SPACE pause flag             */
  bool step_req; /* one-shot single-step flag    */
} VCycle;

static int sweep_limit(const VCycle *vc) {
  if (vc->level == LEVELS - 1)
    return COARSE_ITERS;
  return vc->going_down ? PRE_SMOOTH : POST_SMOOTH;
}

static void vcycle_init(VCycle *vc) {
  memset(vc, 0, sizeof *vc);
  vc->going_down = true;
  compute_residual(0);
  vc->rnorm[0] = grid_l2norm(0);
  vc->r0_norm = vc->rnorm[0];
  if (vc->conv_n < CONV_HIST_LEN)
    vc->conv_hist[vc->conv_n++] = vc->rnorm[0];
}

/* ---- vcycle_step() helpers: one named operation per pseudocode line --- *
 *
 * vcycle_step() reads top-down as Briggs Algorithm 3.1.  Each helper
 * below is one statement of that pseudocode, surfaced with a name that
 * says what it does to the solver state.
 */

/*
 * solver_should_advance() -- consume the pause / single-step gating.
 *   returns true  -> solver may do work this tick
 *   returns false -> solver stays put (SPACE-paused with no 's' pending)
 *
 * The step_req latch is one-shot: an 's' press triggers exactly one
 * advance even while paused, then we fall back to "stays put".
 */
static bool solver_should_advance(VCycle *vc) {
  if (!vc->paused)
    return true;
  if (!vc->step_req)
    return false;
  vc->step_req = false; /* consume the latch */
  return true;
}

/*
 * gauss_seidel_relax_and_measure() -- one red-black GS sweep at the
 * current level, followed by a residual recompute and an L2 norm read.
 *
 * Math:
 *   u <- (I - omega D^{-1} (A u - f))     with omega = 1 (standard GS)
 *   r <- f - (-Del^2 u)                    (5-point stencil)
 *   ||r||_2 stored in rnorm[level]         (for HUD readout)
 *
 * Advances smooth_count -- the scheduler compares against sweep_limit()
 * to decide when to descend / ascend.
 */
static void gauss_seidel_relax_and_measure(VCycle *vc) {
  relax(vc->level);
  compute_residual(vc->level);
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
  vc->smooth_count++;
}

/*
 * sweep_budget_reached() -- have we done enough sweeps at the current
 * (level, going_down) pair to transition?
 *
 * Budget per phase (S1 #defines):
 *   PRE_SMOOTH    sweeps on each level going down (default 2)
 *   POST_SMOOTH   sweeps on each level going up   (default 2)
 *   COARSE_ITERS  extra sweeps at the coarsest level (default 20)
 */
static bool sweep_budget_reached(const VCycle *vc) {
  return vc->smooth_count >= sweep_limit(vc);
}

/*
 * restrict_residual_and_descend() -- transfer the fine-level residual
 * to a coarser RHS, then move scheduler down one level.
 *
 * Physical intuition: after pre-smoothing, the remaining error on the
 * fine grid is SMOOTH.  Restricting its residual to a 2x coarser grid
 * halves the wavelength-to-spacing ratio -- the same smooth error now
 * LOOKS oscillatory on the coarse grid, so a few GS sweeps there will
 * kill it cheaply (1/4 the cells, 1/4 the work).
 *
 * Operator: full-weighting (9-point, 1/16 stencil), see S5 restrict_op.
 */
static void restrict_residual_and_descend(VCycle *vc) {
  restrict_op(vc->level);
  vc->level++;
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
}

/*
 * reverse_at_coarsest_grid() -- bottom of the V.
 *
 * At the coarsest level we've consumed our COARSE_ITERS budget; flip
 * direction so the scheduler begins the post-smoothing ascent.  The
 * coarse-grid correction has driven the local error to nearly zero;
 * the prolong steps will lift that correction back up.
 */
static void reverse_at_coarsest_grid(VCycle *vc) { vc->going_down = false; }

/*
 * prolong_correction_and_ascend() -- interpolate the coarse correction
 * up to the next finer level (added INTO u_fine), then step up one
 * level and refresh the fine residual.
 *
 * Operator: bilinear interpolation P, see S5 prolong_op.  The transpose
 * relation R = (1/4) P^T is the symmetry between this and the matching
 * restriction (Trottenberg, Section 2.3).
 *
 * Residual is re-measured AFTER prolonging because u_fine has just been
 * mutated; any high-frequency error introduced by the (piecewise-linear)
 * interpolation will be damped by the post-smoothing sweeps that follow.
 */
static void prolong_correction_and_ascend(VCycle *vc) {
  prolong_op(vc->level - 1);
  vc->level--;
  compute_residual(vc->level);
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
}

/*
 * record_cycle_convergence() -- a complete V-cycle has just landed
 * back at L0.  Increment the cycle counter and update the convergence
 * rate diagnostic:
 *
 *   rho_k = ||r_k||_2 / ||r_{k-1}||_2
 *
 * Healthy multigrid stays at rho ~ 0.05..0.15 -- each cycle kills
 * 85-95% of the residual independent of grid size (Briggs Ch. 5,
 * Table 5.1).  The new norm is also appended to conv_hist[] (legacy
 * ring; only the latest entry is read by the next call).
 */
static void record_cycle_convergence(VCycle *vc) {
  vc->cycle_count++;
  float rn = vc->rnorm[0];
  if (vc->conv_n >= 2) {
    float prev = vc->conv_hist[vc->conv_n - 1];
    vc->conv_rate = (prev > 1e-15f) ? (rn / prev) : 0.0f;
  }
  if (vc->conv_n < CONV_HIST_LEN)
    vc->conv_hist[vc->conv_n++] = rn;
}

/*
 * restart_at_finest_grid() -- reset scheduler state so the next tick
 * begins a fresh V-cycle from L0 going down.
 */
static void restart_at_finest_grid(VCycle *vc) {
  vc->going_down = true;
  vc->level = 0;
}

/*
 * vcycle_step() -- advance the V-cycle by exactly one action: one GS
 * sweep, one restrict, or one prolong.  Returns true when a complete
 * V-cycle finishes (useful for resetting source / logging upstream).
 *
 * Reads top-down as Algorithm 3.1 of Briggs et al., A Multigrid
 * Tutorial, 2nd ed., Ch. 3.  Each line is one named helper.
 *
 * State machine:
 *   L0:PRE  --restrict--> L1:PRE  --restrict--> L2:PRE  --restrict--> L3:COARSE
 *                                                                          |
 *   L0:POST <--prolong-- L1:POST <--prolong-- L2:POST <--prolong-----------+
 *      |
 *    cycle done, restart
 */
static bool vcycle_step(VCycle *vc) {
  if (!solver_should_advance(vc))
    return false;

  gauss_seidel_relax_and_measure(vc);

  if (!sweep_budget_reached(vc))
    return false;
  vc->smooth_count = 0; /* fresh budget for next (level, dir) */

  if (vc->going_down) {
    if (vc->level < LEVELS - 1)
      restrict_residual_and_descend(vc);
    else
      reverse_at_coarsest_grid(vc);
  } else {
    if (vc->level > 0) {
      prolong_correction_and_ascend(vc);
    } else {
      record_cycle_convergence(vc);
      restart_at_finest_grid(vc);
      return true; /* one full V-cycle landed */
    }
  }
  return false;
}

/* ===================================================================== */
/* S7  scene                                                              */
/* ===================================================================== */

/*
 * Scene -- the simulation's persistent state, lifespan = whole program.
 *
 * INTENT: single source of truth for "what is the solver doing right
 * now."  Anything stored here survives across frames and is what
 * scene_reset() snapshots / re-initialises.
 *
 * LOCALITY PRINCIPLE -- where simulation vs rendering state lives
 * --------------------------------------------------------------
 * Multigrid has two clocks (sim and render at different rates), so
 * stateful separation matters: nothing in the render path should be
 * able to drift the solver, and nothing in the solver should care about
 * pixels.  This codebase puts them in different places by design.
 *
 *   SIMULATION state (persistent, lives HERE or in S4 BSS):
 *     vc             V-cycle scheduler + diagnostics (see VCycle above).
 *     seed           RNG seed for the source term f.  Bumped by
 *                    scene_reset() so each 'r' press lays down a
 *                    different Gaussian-pair source pattern.
 *                    Reproducibility: same seed -> identical source.
 *     g_u, g_f, g_r  the numerical grids (S4 globals).  Kept in BSS
 *                    rather than inside Scene because they are ~32 KB
 *                    total and only S5 touches them.  Embedding them
 *                    here would force every render function to drag
 *                    that bulk through its parameter list.
 *
 *   RENDER state (ephemeral, lives ELSEWHERE):
 *     Screen.cols/rows         terminal size, in S9.  Cached from
 *                              ncurses; refreshed on SIGWINCH only.
 *     panel positions/widths   computed per-frame by compute_panels()
 *                              in S8.  Not cached because they depend
 *                              on vc->level which changes mid-V-cycle
 *                              (spotlight migrates with the active
 *                              level).
 *     vmax (heat scale)        computed per-frame in screen_draw()
 *                              from the current residual magnitudes.
 *                              Recomputed because the residual decays
 *                              and we want the heatmap to keep
 *                              resolving fine detail as bright reds
 *                              fade.
 *     color pairs              static after color_init() in S3 --
 *                              configuration, not state.
 *     WIDE_FRAC, TOP_HUD_ROWS  constants in S1; not state.
 *
 *   APP-CONTROL state (lives in App, S10):
 *     sim_fps, running, need_resize -- loop control, not sim or render.
 *
 * Rule of thumb: if a value is READ by S5 (solver) it goes here or in
 * the S4 globals.  If only S8/S9 (render) read it, it stays local to
 * those functions or sits in Screen.  This is why Scene is tiny: most
 * of what a frame needs to draw is either looked up from globals or
 * recomputed cheaply.
 */
typedef struct {
  VCycle vc;     /* V-cycle scheduler + diagnostics */
  unsigned seed; /* source-term RNG seed, bumped on reset */
} Scene;

static void scene_init(Scene *sc) {
  sc->seed = (unsigned)time(NULL);
  grid_reset(sc->seed);
  vcycle_init(&sc->vc);
}

static void scene_reset(Scene *sc) {
  sc->seed += 9973;
  grid_reset(sc->seed);
  vcycle_init(&sc->vc);
}

static void scene_tick(Scene *sc) { vcycle_step(&sc->vc); }

/* ===================================================================== */
/* S8  render                                                             */
/* ===================================================================== */

/*
 * Spotlight panel layout: the active level gets WIDE_FRAC of cols; the
 * remaining three levels share the rest equally as thumbnails.  This makes
 * L3 (8x4) actually readable when it's the focus, while keeping context
 * panels visible.  Positions shift when the active level changes -- the
 * panels visibly "morph" in sync with the V-cycle phase.
 */
static void compute_panels(int active, int cols, int px[LEVELS],
                           int pw[LEVELS]) {
  int wide_w = (int)(WIDE_FRAC * (float)cols);
  int n_thumb = LEVELS - 1;
  int thumb_w = (cols - wide_w) / n_thumb;
  if (thumb_w < 6)
    thumb_w = 6;                     /* readability floor */
  wide_w = cols - thumb_w * n_thumb; /* rebalance to fill row */

  int x = 0;
  for (int l = 0; l < LEVELS; l++) {
    px[l] = x;
    pw[l] = (l == active) ? wide_w : thumb_w;
    x += pw[l];
  }
}

/* ---- shared phase classification (used by panel + banner) ------------ */

/*
 * PhaseGlyphs -- visual signature for the V-cycle's current phase.
 *   label    human-readable phase name
 *   fill_ch  one-character direction glyph: '>' descending,
 *            '=' parked at coarsest, '<' ascending
 *   cp       color pair tying label and fill together
 *
 * Used by render_phase_banner() (banner) and draw_sweep_progress_bar()
 * (in-panel bar) so they always agree on color.  render_top_hud()
 * derives its own arrow strings inline, but reads from the same enum
 * (CP_PHASE_*) values so the position row stays in sync visually.
 */
typedef struct {
  const char *label;
  char fill_ch;
  int cp;
} PhaseGlyphs;

/*
 * phase_glyphs() -- classify (going_down, level == LEVELS-1) into the
 * (label, fill_ch, color) triple.  Encodes V-cycle direction visually
 * through the fill glyph.
 */
static PhaseGlyphs phase_glyphs(const VCycle *vc) {
  PhaseGlyphs g;
  if (vc->level == LEVELS - 1) {
    g.label = "COARSE SOLVE";
    g.fill_ch = '=';
    g.cp = CP_PHASE_COARSE;
  } else if (vc->going_down) {
    g.label = "PRE-SMOOTH";
    g.fill_ch = '>';
    g.cp = CP_PHASE_DOWN;
  } else {
    g.label = "POST-SMOOTH";
    g.fill_ch = '<';
    g.cp = CP_PHASE_UP;
  }
  return g;
}

/* ---- render_grid() helpers ------------------------------------------- */

/*
 * panel_too_small() -- need at least 1-cell border each side + 1 cell
 * of content, so 3x3 minimum.  Below that we draw nothing.
 */
static bool panel_too_small(int pw, int ph) { return (pw < 3 || ph < 3); }

/*
 * map_term_to_grid_index() -- nearest-neighbour downsample from a
 * terminal-cell coordinate to a grid INTERIOR index.
 *
 *   term_idx       in [0, term_size)
 *   returns        in [1, grid_interior]   (Dirichlet boundary excluded)
 *
 * This is the ONLY place the "screen pixels per grid cell" conversion
 * lives.  Identity at 1:1 resolution; otherwise a uniform sample of
 * the grid stretched across the panel.  L3 active (8 interior pts
 * across 75+ cols) gives big chunky 10-cell blocks -- correct
 * nearest-neighbour behaviour, no smoothing.
 */
static int map_term_to_grid_index(int term_idx, int term_size,
                                  int grid_interior) {
  int g = 1 + (int)((float)term_idx / (float)term_size * (float)grid_interior);
  if (g < 1)
    g = 1;
  if (g > grid_interior)
    g = grid_interior;
  return g;
}

/*
 * draw_panel_frame() -- top border (with embedded label), left and
 * right edges.  Active panels get BOLD bright color; inactive get DIM.
 *
 * Label format: "L0 62x30 |r|=1.4e+00"  (level, interior dims, current
 * L2 residual at that level).
 */
static void draw_panel_frame(int l, int px, int pw, int py, int ph,
                             bool is_active, float rnorm_l) {
  int nx = LNX(l), ny = LNY(l);
  int cp_b = is_active ? CP_BORDER_ACT : CP_BORDER_OFF;
  attr_t ba = COLOR_PAIR(cp_b) | (is_active ? A_BOLD : A_DIM);

  attron(ba);
  /* Top border */
  for (int c = px; c < px + pw && c < COLS; c++)
    mvaddch(py, c, '-');
  /* Label overlay on the top border */
  char lbl[32];
  snprintf(lbl, sizeof lbl, "L%d %dx%d |r|=%.1e", l, nx - 2, ny - 2,
           (double)rnorm_l);
  mvprintw(py, px + 1, "%.*s", pw - 2, lbl);
  /* Left / right edges */
  for (int r = 1; r < ph; r++) {
    mvaddch(py + r, px, '|');
    if (px + pw - 1 < COLS)
      mvaddch(py + r, px + pw - 1, '|');
  }
  attroff(ba);
}

/*
 * paint_residual_heatmap() -- color each terminal cell inside the
 * panel border according to |r| at the nearest grid interior point.
 *
 *   colour scale: 9-step ramp (CP_H0..CP_H8) shared by all levels
 *   character:    " .:+x*X#@" ordered by ink density (Bourke ramp)
 *   normalisation: vmax (max |r| across ALL levels this frame), so
 *                  panels are visually comparable
 *
 * The sqrt(t) warp inside heat_cell() approximates perceptual
 * lightness so that equal differences in residual look like equal
 * differences in brightness (Kovesi 2015).
 */
static void paint_residual_heatmap(int l, int px, int pw, int py, int ph,
                                   float vmax) {
  int nx = LNX(l), ny = LNY(l);
  int content_w = pw - 2; /* inside the | borders */
  int content_h = ph - 1; /* below the top border */
  if (content_w < 1 || content_h < 1)
    return;

  for (int row = 0; row < content_h; row++) {
    int j = map_term_to_grid_index(row, content_h, ny - 2);
    for (int col = 0; col < content_w; col++) {
      int i = map_term_to_grid_index(col, content_w, nx - 2);
      float val = fabsf(g_r[l][LIDX(l, i, j)]);
      int cp;
      char ch;
      heat_cell(val, vmax, &cp, &ch);
      attron(COLOR_PAIR(cp));
      mvaddch(py + 1 + row, px + 1 + col, (chtype)(unsigned char)ch);
      attroff(COLOR_PAIR(cp));
    }
  }
}

/*
 * draw_sweep_progress_bar() -- single '=' row at the top of the active
 * panel's content area, length proportional to smooth_count / budget.
 *
 * Mirrors the sweep counter in the phase banner; the in-panel
 * placement keeps the user's eyes anchored on the level being worked
 * on without darting down to the banner.
 */
static void draw_sweep_progress_bar(const VCycle *vc, int px, int pw, int py) {
  int content_w = pw - 2;
  int lim = sweep_limit(vc);
  if (lim <= 0 || content_w <= 4)
    return;

  int filled = (int)((float)vc->smooth_count / (float)lim * (float)content_w);
  int cp = phase_glyphs(vc).cp;
  attron(COLOR_PAIR(cp));
  for (int c = 0; c < filled && c < content_w; c++)
    mvaddch(py + 2, px + 1 + c, '=');
  attroff(COLOR_PAIR(cp));
}

/*
 * render_grid() -- draw one level's panel.  Reads as pseudocode: frame,
 * heatmap, and (if active) the in-panel sweep progress bar.
 *
 *   px, pw       horizontal slot from compute_panels()
 *   py, ph       vertical slot (set by screen_draw)
 *   vmax         shared heat-map normalisation across all levels
 */
static void render_grid(const VCycle *vc, int l, int px, int pw, int py, int ph,
                        float vmax) {
  if (panel_too_small(pw, ph))
    return;

  bool is_active = (l == vc->level);

  draw_panel_frame(l, px, pw, py, ph, is_active, vc->rnorm[l]);
  paint_residual_heatmap(l, px, pw, py, ph, vmax);

  if (is_active)
    draw_sweep_progress_bar(vc, px, pw, py);
}

/* ---- render_phase_banner() helpers ----------------------------------- */

/*
 * format_phase_caption() -- the centered text rendered over the banner
 * fill, e.g. "  PRE-SMOOTH  L1  sweep 1/2  ".  Returns string length so
 * the painter can centre it.
 */
static int format_phase_caption(const VCycle *vc, const PhaseGlyphs *g,
                                char *out, size_t cap) {
  return snprintf(out, cap, "  %s  L%d  sweep %d/%d  ", g->label, vc->level,
                  vc->smooth_count, sweep_limit(vc));
}

/*
 * paint_banner_row() -- fill an entire screen row with fill_ch then
 * stamp the centered caption over it, all in one color pair / bold attr.
 */
static void paint_banner_row(int row, int cols, const char *caption,
                             int caption_len, char fill_ch, int cp) {
  int cap_start = (cols - caption_len) / 2;
  if (cap_start < 0)
    cap_start = 0;

  attron(COLOR_PAIR(cp) | A_BOLD);
  for (int c = 0; c < cols; c++) {
    char ch = fill_ch;
    if (c >= cap_start && c < cap_start + caption_len)
      ch = caption[c - cap_start];
    mvaddch(row, c, (chtype)(unsigned char)ch);
  }
  attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * render_phase_banner() -- full-width directional banner above bottom HUD.
 *
 *   PRE-SMOOTH  (descending)  '>>>>>>>>  PRE-SMOOTH  L1  sweep 1/2  >>>>>>>>'
 *   COARSE SOLVE              '========  COARSE SOLVE  L3  sweep 7/20  ===='
 *   POST-SMOOTH (ascending)   '<<<<<<<<  POST-SMOOTH  L1  sweep 2/2  <<<<<<<<'
 *
 * Three pseudocode steps: classify phase, format caption, paint row.
 */
static void render_phase_banner(const VCycle *vc, int row, int cols) {
  PhaseGlyphs g = phase_glyphs(vc);

  char caption[64];
  int caption_len = format_phase_caption(vc, &g, caption, sizeof caption);

  paint_banner_row(row, cols, caption, caption_len, g.fill_ch, g.cp);
}

/*
 * render_bottom_hud() -- 2-row action zone at the bottom of the screen.
 *   Row N-2: full-width phase banner (PRE / COARSE / POST + sweep)
 *   Row N-1: single line of key bindings
 */
static void render_bottom_hud(const Scene *sc, int rows, int cols) {
  const VCycle *vc = &sc->vc;

  render_phase_banner(vc, rows - 2, cols);

  attron(COLOR_PAIR(CP_LABEL) | A_BOLD);
  mvprintw(rows - 1, 1,
           " SPACE pause   s step   r reset   +/- speed   q quit ");
  attroff(COLOR_PAIR(CP_LABEL) | A_BOLD);
}

/*
 * render_top_hud() -- 3-row status zone at the top of the screen.
 *   Row 0: title (left) + V-cycle count (right)
 *   Row 1: numeric status -- level, sweep progress, residual norms, rho
 *   Row 2: V-cycle position bar across all 4 levels, color-coded by direction
 */
static void render_top_hud(const Scene *sc, int cols) {
  const VCycle *vc = &sc->vc;

  /* -- Row 0: title left, V-cycle counter right ---------------------- */
  attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
  mvprintw(0, 0,
           " Multigrid V-cycle Poisson Solver   -Del^2u = f   "
           "%d levels  %dx%d -> %dx%d -> %dx%d -> %dx%d",
           LEVELS, LNX(0) - 2, LNY(0) - 2, LNX(1) - 2, LNY(1) - 2, LNX(2) - 2,
           LNY(2) - 2, LNX(3) - 2, LNY(3) - 2);

  char cy_buf[40];
  int cy_len = snprintf(cy_buf, sizeof cy_buf, " V-cycle %d ", vc->cycle_count);
  if (cy_len < cols)
    mvprintw(0, cols - cy_len, "%s", cy_buf);
  attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

  /* -- Row 1: numeric status ---------------------------------------- */
  int cp_rate = CP_CONV_MID;
  if (vc->conv_rate > 0.0f && vc->conv_rate < 0.2f)
    cp_rate = CP_CONV_GOOD;
  else if (vc->conv_rate >= 0.5f)
    cp_rate = CP_CONV_BAD;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(1, 1, " L%d   sweep %d/%d   ||r0||=%.2e -> ||r_cur||=%.2e   rho=",
           vc->level, vc->smooth_count, sweep_limit(vc), (double)vc->r0_norm,
           (double)vc->rnorm[0]);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  if (vc->conv_n >= 2) {
    attron(COLOR_PAIR(cp_rate) | A_BOLD);
    printw("%.3f", (double)vc->conv_rate);
    attroff(COLOR_PAIR(cp_rate) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD));
    printw("  (%.0f%% reduction per cycle, ideal < 0.2)",
           (double)((1.0f - vc->conv_rate) * 100.0f));
    attroff(COLOR_PAIR(CP_HUD));
  } else {
    attron(COLOR_PAIR(CP_HUD));
    printw("---");
    attroff(COLOR_PAIR(CP_HUD));
  }

  /* -- Row 2: V-cycle position bar ---------------------------------- *
   *   PRE-SMOOTH  (down) :  L0  >>>  *L1*  >>>  L2  >>>  L3
   *   COARSE             :  L0  ===  L1  ===  L2  === *L3*
   *   POST-SMOOTH (up)   :  L0  <<<  L1  <<< *L2*  <<<  L3
   */
  const char *arr;
  int cp_dir;
  if (vc->level == LEVELS - 1) {
    arr = "===";
    cp_dir = CP_PHASE_COARSE;
  } else if (vc->going_down) {
    arr = ">>>";
    cp_dir = CP_PHASE_DOWN;
  } else {
    arr = "<<<";
    cp_dir = CP_PHASE_UP;
  }

  char vbuf[256];
  int vpos = 0;
  for (int l = 0; l < LEVELS; l++) {
    const char *fmt = (l == vc->level) ? "*L%d*" : " L%d ";
    vpos += snprintf(vbuf + vpos, sizeof vbuf - (size_t)vpos, fmt, l);
    if (l < LEVELS - 1)
      vpos += snprintf(vbuf + vpos, sizeof vbuf - (size_t)vpos, "  %s  ", arr);
  }
  int vstart = (cols - vpos) / 2;
  if (vstart < 1)
    vstart = 1;
  attron(COLOR_PAIR(cp_dir) | A_BOLD);
  mvprintw(2, vstart, "%s", vbuf);
  attroff(COLOR_PAIR(cp_dir) | A_BOLD);
}

/* ===================================================================== */
/* S9  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal dimensions.
 *
 * INTENT: hold the ncurses-derived width/height so render functions
 * don't repeatedly call getmaxyx().  Read by every render function in
 * S8; refreshed on boot (screen_init) and on SIGWINCH (screen_resize).
 *
 * Pure render-side state -- the simulation never reads it.  When the
 * terminal is resized the solver keeps running with the same level and
 * sweep_count; only the panel widths and HUD layout reflow to fit the
 * new dims.  This is the locality principle from Scene's comment in
 * practice: sim state untouched, render state refreshed.
 *
 *   cols    terminal column count  (ncurses COLS at last query)
 *   rows    terminal row count     (ncurses LINES at last query)
 */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
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

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(const Screen *s, const Scene *sc) {
  int rows = s->rows, cols = s->cols;
  erase();

  render_top_hud(sc, cols);

  /* Panels fill between top status HUD and bottom action HUD */
  int py = TOP_HUD_ROWS;
  int ph = rows - BOTTOM_HUD_ROWS - py;
  if (ph < 4)
    ph = 4;

  /* Global max |r| for consistent heatmap scale */
  float vmax = 0.0f;
  for (int l = 0; l < LEVELS; l++) {
    float mx = grid_maxabs(g_r[l], l);
    if (mx > vmax)
      vmax = mx;
  }
  if (vmax < 1e-12f)
    vmax = 1e-12f;

  int px_arr[LEVELS], pw_arr[LEVELS];
  compute_panels(sc->vc.level, cols, px_arr, pw_arr);
  for (int l = 0; l < LEVELS; l++) {
    render_grid(&sc->vc, l, px_arr[l], pw_arr[l], py, ph, vmax);
  }

  render_bottom_hud(sc, rows, cols);

  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* S10 app -- signals, resize, input, main loop                           */
/* ===================================================================== */

/*
 * App -- top-level lifecycle bundle, lives in g_app for the whole program.
 *
 * INTENT: composition root.  App OWNS the Scene (sim) and the Screen
 * (cached render dims), and adds the loop-driver fields (running,
 * need_resize, sim_fps) that don't belong to either sub-component.
 *
 * Why a single g_app global instead of pass-by-pointer everywhere:
 * signal handlers (SIGINT/SIGTERM/SIGWINCH) cannot take user pointers,
 * so they have to mutate via a known location.  Centralising that
 * location in g_app keeps the signal contract narrow and explicit.
 *
 *   scene         simulation state -- see Scene above.
 *   screen        cached terminal dims -- see Screen above.
 *   sim_fps       solver ticks per second (1..30, default 6).  Adjusted
 *                 live by '+' / '-' in app_handle_key().  Decoupled from
 *                 the ~60 Hz render rate so you can slow the V-cycle
 *                 down to watch each sweep land.
 *   running       main-loop sentinel.  Cleared by SIGINT/SIGTERM
 *                 handlers and by the 'q' key.  volatile sig_atomic_t
 *                 because a signal handler writes it asynchronously and
 *                 the main loop reads it -- this is the only portable
 *                 way to share a flag across that boundary.
 *   need_resize   SIGWINCH flag.  Set by on_resize(), drained by the
 *                 main loop on the next iteration, which calls
 *                 screen_resize() to refresh Screen.cols/rows.  Same
 *                 volatile sig_atomic_t rationale.
 *
 * Reference: POSIX.1-2017, Section 2.4 (Signal Concepts).  sig_atomic_t
 * is the standard idiom for cross-handler flags; volatile prevents the
 * compiler from caching the value across the loop body.
 */
typedef struct {
  Scene scene;                       /* simulation state */
  Screen screen;                     /* cached render dims */
  int sim_fps;                       /* solver ticks per second */
  volatile sig_atomic_t running;     /* 0 = exit main loop */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending */
} App;

static App g_app;

static void cleanup(void) { endwin(); }
static void on_exit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}

static bool app_handle_key(App *app, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
    return false;
  case ' ':
    app->scene.vc.paused = !app->scene.vc.paused;
    break;
  case 's':
  case 'S':
    app->scene.vc.paused = true;
    app->scene.vc.step_req = true;
    break;
  case 'r':
  case 'R':
    scene_reset(&app->scene);
    break;
  case '+':
  case '=':
    if (app->sim_fps < SIM_FPS_MAX)
      app->sim_fps++;
    break;
  case '-':
  case '_':
    if (app->sim_fps > SIM_FPS_MIN)
      app->sim_fps--;
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;
  app->need_resize = 0;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene);
      sim_accum -= tick_ns;
    }

    fps_accum += dt;
    frame_count++;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
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
