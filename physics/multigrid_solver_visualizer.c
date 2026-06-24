/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/multigrid_solver_visualizer.c -- watch a multigrid solver work.
 *
 * Solves for a field that smooths out a source bump (pinned to zero at the
 * edges) and animates how it gets there. The neat idea: a simple averaging
 * step erases sharp, jittery error fast but barely touches broad, gentle
 * error -- so we shrink the grid (where gentle error looks jittery again),
 * fix it cheaply, and carry the fix back up. The screen going dark means
 * the answer is settling in.
 *
 * References (the background the code can't give you):
 *   [1] Briggs, Henson, McCormick, "A Multigrid Tutorial," 2nd ed., SIAM 2000.
 *       The friendly intro; vcycle_step() follows its Algorithm 3.1.
 *   [2] Trottenberg et al., "Multigrid," Academic Press 2001. Why shrinking
 *       and growing the grid are mirror images of each other (Sec. 2.3).
 *   [3] Brandt, Math. Comp. 31(138), 1977 -- the original multigrid paper.
 *   [4] Hackbusch, "Multi-Grid Methods and Applications," Springer 1985.
 *   [5] Press et al., "Numerical Recipes," 3rd ed., Sec. 20.6.
 *   [6] LeVeque, "Finite Difference Methods," SIAM 2007, Ch. 3 -- where the
 *       neighbour-averaging stencil and zero-edge rule come from.
 *   [7] Kovesi, "Good Colour Maps," arXiv:1509.03700, 2015 -- why heat_cell()
 *       bends brightness so equal error steps look equally bright.
 *   [8] Bourke, paulbourke.net/dataformats/asciiart/ -- the " .:+x*X#@" ramp.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define SIM_FPS_DEFAULT 6
#define SIM_FPS_MIN 1
#define SIM_FPS_MAX 30
#define TARGET_FPS 60
#define FPS_UPDATE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* The grid stack: four grids, each half the width and height of the one above.
 * Sizes count the edge cells too (those stay pinned at zero). */
#define LEVELS 4
#define L0_NX 64                /* finest grid: 64 wide */
#define L0_NY 32                /* finest grid: 32 tall */
#define MAX_PTS (L0_NX * L0_NY) /* biggest grid is the finest one */

/* Size and flat-array index of grid level l (each level halves the one above). */
#define LNX(l) (L0_NX >> (l))
#define LNY(l) (L0_NY >> (l))
#define LIDX(l, x, y) ((y) * LNX(l) + (x))

/* How many smoothing passes to spend at each stop. The coarsest grid is tiny,
 * so we can afford to nearly solve it outright. */
#define PRE_SMOOTH 2
#define POST_SMOOTH 2
#define COARSE_ITERS 20

/* The source f: a bump in the field, how tall and how wide. */
#define SRC_AMP 8.0f
#define SRC_SIGMA 4.0f

#define CONV_HIST_LEN 32

/* The HUD splits in two: status numbers up top, controls down bottom. */
#define TOP_HUD_ROWS 3
#define BOTTOM_HUD_ROWS 2

/* How wide the spotlighted (active) level is, as a slice of the screen. The
 * other three levels split what's left. ~0.62 makes the active one clearly
 * the star of the show. */
#define WIDE_FRAC 0.62f

/* ── §2 clock ── */

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

/* ── §3 color ── */

/* Names for the color pairs. CP_H0..CP_H8 are the heat ramp, from "solved"
 * (cool, dark) to "lots of error here" (hot, bright); the rest tint the UI. */
enum {
  CP_NONE = 0,
  CP_H0,
  CP_H1,
  CP_H2,
  CP_H3,
  CP_H4,
  CP_H5,
  CP_H6,
  CP_H7,
  CP_H8,
  CP_BORDER_ACT,   /* border of the level being worked on  */
  CP_BORDER_OFF,   /* border of the other levels           */
  CP_HUD,          /* status text                          */
  CP_HEADER,       /* title row                            */
  CP_LABEL,        /* key-binding row                      */
  CP_PHASE_DOWN,   /* heading down the grid stack          */
  CP_PHASE_UP,     /* heading back up                      */
  CP_PHASE_COARSE, /* parked at the bottom                 */
  CP_CONV_GOOD,    /* converging fast                      */
  CP_CONV_MID,     /* converging, but slowly               */
  CP_CONV_BAD,     /* barely converging -- something's off */
};

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    /* Cool-to-hot ramp: navy, blue, cyan, green, yellow, orange, red. */
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

/* Characters from "barely any ink" to "solid", so denser glyphs read as more
 * error (Bourke ramp). */
static const char k_heat[] = " .:+x*X#@";

/* Pick a color and character for one error value, sized against the biggest
 * error on screen. The sqrt bend stretches the low end so small errors still
 * show up instead of all fading into the same dark cell. */
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
  t = sqrtf(t);
  int idx = (int)(t * 8.99f);
  if (idx > 8)
    idx = 8;
  *cp_out = CP_H0 + idx;
  *ch_out = k_heat[idx];
}

/* ── §4 grid state ── */

/*
 * The three grids the math lives in, one copy per level. Static storage, so no
 * malloc and they start zeroed.
 *
 *   g_u[l]  the answer-so-far at level l -- our best guess at the field.
 *           Always zero around the edges (that's the boundary condition).
 *   g_f[l]  the "right-hand side" we're solving against. On the finest grid
 *           it's the source bump f; on coarser grids it's the leftover error
 *           handed down from the level above.
 *   g_r[l]  the residual: how badly the current guess misses the equation at
 *           each cell. Big residual = lots of error still here. The screen
 *           going dark = residual shrinking to nothing.
 */
static float g_u[LEVELS][MAX_PTS];
static float g_f[LEVELS][MAX_PTS];
static float g_r[LEVELS][MAX_PTS];

/* Lays down the source f on the finest grid: two bumps, one positive, one
 * negative, so the field has some real shape to solve for instead of a single
 * blob. */
static void grid_set_source(unsigned seed) {
  srand(seed);
  int nx = LNX(0), ny = LNY(0);
  float cx = nx * 0.5f, cy = ny * 0.5f;
  float inv2s2 = 1.0f / (2.0f * SRC_SIGMA * SRC_SIGMA);
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

/* One number for "how much error is left" at level l -- the typical residual
 * size, averaged so grids of different sizes are comparable. */
static float grid_l2norm(int l) {
  int n = LNX(l) * LNY(l);
  float s = 0.0f;
  for (int k = 0; k < n; k++)
    s += g_r[l][k] * g_r[l][k];
  return sqrtf(s / (float)n);
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

/* ── §5 solver (multigrid operators) ── */

/*
 * One smoothing pass over level l: nudge every interior cell toward the
 * average of its four neighbours plus the local source. Repeated, this settles
 * the field -- but it only fixes fine, jittery error quickly; broad humps barely
 * budge, which is the whole reason coarser grids exist.
 *
 * Done in two waves ("red-black"): first the cells on the even squares of the
 * checkerboard, then the odd ones. Each wave only reads cells the other wave
 * owns, so they don't step on each other and the field settles faster.
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
 * Measures how wrong the current guess is at each interior cell and stores it
 * in g_r. A big value means the equation is badly violated right there; zero
 * means that cell is solved. This is what the heat map draws, and what the
 * coarser grids get handed to work on.
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
 * Hands the leftover error down to the next-coarser grid. Each coarse cell
 * takes a weighted blend of the nine fine cells around its matching spot --
 * the center counts most, edges less, corners least -- so nothing is lost in
 * the shrink. We also wipe the coarse guess so that grid starts from scratch.
 * (This blend is "full-weighting restriction"; see ref [1].)
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
 * Carries the coarse-grid fix back up to the finer grid and adds it in.
 * Fine cells that line up with a coarse cell copy its value directly; cells
 * that fall between coarse cells take an average of the neighbours they sit
 * between, the way you'd smoothly stretch a small picture over a bigger one.
 * (This is "bilinear prolongation" -- the exact mirror of restrict_op; [2].)
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

/* ── §6 V-cycle state machine ── */

/*
 * VCPhase -- names for the three stages a V-cycle passes through.
 *
 * Heads up: nothing actually reads this enum anymore. The live code figures
 * out the current stage on the fly from the direction and level (see
 * phase_glyphs() in §8). It's kept only because the three names spell out the
 * shape of the algorithm at a glance:
 *
 *   PRE     smoothing a level on the way DOWN the grid stack
 *   COARSE  parked at the bottom, doing extra cleanup
 *   POST    smoothing a level on the way back UP
 *
 * The names match the three stages of Algorithm 3.1 in ref [1].
 */
typedef enum {
  VCPHASE_PRE,    /* smoothing on the way down  */
  VCPHASE_COARSE, /* extra cleanup at the bottom */
  VCPHASE_POST,   /* smoothing on the way up    */
} VCPhase;

/*
 * VCycle -- the bookkeeping that drives the solver one step at a time.
 *
 * Each tick the solver does exactly one small thing: one smoothing sweep, or
 * one step down to a coarser grid, or one step back up. This struct remembers
 * where we are in that walk and tracks the numbers shown in the top HUD. The
 * actual grids it works on live in the §4 globals, not here -- they are bulky
 * and only the solver touches them, so we keep them out of every render call.
 *
 * Members, grouped:
 *
 *   WHERE WE ARE -- what the solver is doing right now.
 *     level         which grid; 0 is the finest, LEVELS-1 the coarsest.
 *                   Goes up when we step down, down when we step back up.
 *     going_down    true while walking down the stack, false on the way up.
 *     smooth_count  sweeps done at this level so far; once it hits the
 *                   per-level budget (sweep_limit) we move on.
 *
 *   READOUTS -- numbers for the top HUD.
 *     cycle_count   how many full V-cycles since the last reset.
 *     r0_norm       how much error we started with, measured once at the
 *                   beginning -- the baseline everything is judged against.
 *     rnorm[]       current error size at each level; rnorm[0] (the finest)
 *                   is the one the HUD shows.
 *     conv_rate     how much error one full cycle erases, as a fraction of
 *                   what was there before. Smaller is better; a healthy
 *                   solver sits around 0.05..0.15. The HUD colors it
 *                   green/yellow/red.
 *     conv_hist[]   the last few error sizes, one per finished cycle. Only
 *                   the most recent entry is actually read (to compare
 *                   against the new one for conv_rate).
 *     conv_n        how many entries conv_hist holds; we need at least two
 *                   cycles before a rate makes sense.
 *
 *   PAUSE CONTROLS -- set by the keyboard, read by the solver.
 *     paused        true while SPACE-paused; the solver sits still.
 *     step_req      's' sets this to let exactly one step through while
 *                   paused, then it clears itself.
 *
 *   UNUSED:
 *     phase         see the VCPhase note above -- never set or read.
 *
 * This is the data behind Algorithm 3.1 in refs [1] and [2].
 */
typedef struct {
  VCPhase phase;
  int level;        /* 0 = finest, LEVELS-1 = coarsest */
  int smooth_count; /* sweeps done at this level so far */
  bool going_down;  /* true = walking down the stack    */

  /* Readouts for the HUD */
  int cycle_count;                /* full V-cycles since reset       */
  float r0_norm;                  /* error size we started with      */
  float rnorm[LEVELS];            /* current error size per level    */
  float conv_rate;                /* fraction of error left per cycle */
  float conv_hist[CONV_HIST_LEN]; /* recent error sizes, one per cycle */
  int conv_n;                     /* how many of those are filled in  */

  bool paused;   /* true while SPACE-paused          */
  bool step_req; /* one-shot single-step request    */
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

/* vcycle_step() helpers. Each one is a single line of the algorithm, named
 * for what it does, so the step function below reads like its pseudocode. */

/*
 * Decides whether the solver gets to do anything this tick. Normally yes;
 * while paused, no -- unless 's' was just pressed, which lets a single step
 * through and then resets itself.
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
 * Does one full unit of work at the current level: smooth once, then measure
 * how much error is left and remember its size for the HUD. Bumps the sweep
 * counter, which is how the solver knows when this level is done.
 */
static void gauss_seidel_relax_and_measure(VCycle *vc) {
  relax(vc->level);
  compute_residual(vc->level);
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
  vc->smooth_count++;
}

/*
 * Have we done enough sweeps at this level to move on? Each stage has its own
 * budget (the PRE_SMOOTH / POST_SMOOTH / COARSE_ITERS constants in §1).
 */
static bool sweep_budget_reached(const VCycle *vc) {
  return vc->smooth_count >= sweep_limit(vc);
}

/*
 * Step down a level: hand this grid's leftover error to the next-coarser one
 * and switch focus to it. The point of going coarser: the gentle error left
 * after smoothing looks sharp and jittery on a grid half the size, so a few
 * cheap sweeps there (a quarter of the cells) wipe out what was stubborn here.
 */
static void restrict_residual_and_descend(VCycle *vc) {
  restrict_op(vc->level);
  vc->level++;
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
}

/*
 * Bottom of the V. We've done our cleanup on the smallest grid, so turn
 * around and start climbing back up, carrying the fix with us.
 */
static void reverse_at_coarsest_grid(VCycle *vc) { vc->going_down = false; }

/*
 * Step up a level: take the fix worked out on the coarser grid, stretch it
 * back onto the finer one and add it in, then switch focus up. We re-measure
 * the error right after, since the finer grid just changed; any roughness the
 * stretch introduced gets cleaned up by the smoothing sweeps that follow.
 */
static void prolong_correction_and_ascend(VCycle *vc) {
  prolong_op(vc->level - 1);
  vc->level--;
  compute_residual(vc->level);
  vc->rnorm[vc->level] = grid_l2norm(vc->level);
}

/*
 * A full cycle just finished back at the finest grid. Count it and work out
 * the convergence rate: how much error is left compared to before this cycle.
 * A healthy solver leaves only 5-15% each time, no matter the grid size.
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

/* Point everything back at the finest grid so the next tick starts a fresh
 * cycle heading down again. */
static void restart_at_finest_grid(VCycle *vc) {
  vc->going_down = true;
  vc->level = 0;
}

/*
 * The one step the solver takes each tick: smooth once, and if this level is
 * done, either move down, turn around at the bottom, or move back up. Returns
 * true the moment a whole cycle finishes. Reads top to bottom like the
 * algorithm in ref [1]. The path it walks, level 0 fine to level 3 coarse:
 *
 *   down: L0 -> L1 -> L2 -> L3 (the bottom)
 *   up:   L3 -> L2 -> L1 -> L0, then start over
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

/* ── §7 scene ── */

/*
 * Scene -- everything about the simulation that has to stick around between
 * frames. Reset wipes and re-seeds exactly this.
 *
 *   vc      the solver's bookkeeping -- where it is, how it's doing.
 *   seed    the random seed that decides the shape of the source bump.
 *           Each 'r' press nudges it so you get a fresh pattern; the same
 *           seed always gives the same one.
 *
 * Notice it's tiny. The big grids the solver crunches live in the §4 globals,
 * not in here, on purpose: they're bulky and only the solver reads them, so
 * there's no reason to drag them through every drawing call. The drawing side
 * keeps its own scratch (screen size, panel layout) elsewhere, so the solver
 * can't accidentally be thrown off by anything to do with pixels.
 */
typedef struct {
  VCycle vc;     /* the solver's bookkeeping              */
  unsigned seed; /* shape of the source bump; bumped on reset */
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

/* ── §8 render ── */

/*
 * Works out where each level's panel goes. The level being worked on gets the
 * lion's share of the width so even the tiny coarse grids are readable when
 * it's their turn; the other three share what's left as thumbnails. The layout
 * shifts as focus moves, so the panels seem to slide in step with the solver.
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

/*
 * PhaseGlyphs -- the look that goes with whatever stage the solver is in.
 *   label    the stage's name to print
 *   fill_ch  a direction arrow: '>' heading down, '=' parked at the
 *            bottom, '<' heading back up
 *   cp       the color, so the name and the arrow always match
 *
 * The banner and the in-panel progress bar both pull from here so they never
 * disagree on color. (The top HUD picks its own arrows but uses the same
 * colors.)
 */
typedef struct {
  const char *label;
  char fill_ch;
  int cp;
} PhaseGlyphs;

/* Figures out the current stage from the direction and level, and hands back
 * its name, arrow, and color. */
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

/* A panel needs room for a border on each side plus at least one cell of
 * content -- 3x3 minimum. Smaller than that, draw nothing. */
static bool panel_too_small(int pw, int ph) { return (pw < 3 || ph < 3); }

/*
 * Maps a spot on the screen to the grid cell it should show. This is the one
 * place screen-cells and grid-cells get matched up. When the grid is smaller
 * than the panel it just gets stretched -- a tiny coarse grid shows up as big
 * chunky blocks, which is exactly what we want. Edges are skipped since they
 * are pinned to zero and never carry any error.
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

/* Draws a panel's box -- top edge with a label baked in, plus the two sides.
 * The active panel is bright and bold, the rest dim. The label reads like
 * "L0 62x30 |r|=1.4e+00": the level, its size, and how much error is left. */
static void draw_panel_frame(int l, int px, int pw, int py, int ph,
                             bool is_active, float rnorm_l) {
  int nx = LNX(l), ny = LNY(l);
  int cp_b = is_active ? CP_BORDER_ACT : CP_BORDER_OFF;
  attr_t ba = COLOR_PAIR(cp_b) | (is_active ? A_BOLD : A_DIM);

  attron(ba);
  for (int c = px; c < px + pw && c < COLS; c++)
    mvaddch(py, c, '-');
  char lbl[32];
  snprintf(lbl, sizeof lbl, "L%d %dx%d |r|=%.1e", l, nx - 2, ny - 2,
           (double)rnorm_l);
  mvprintw(py, px + 1, "%.*s", pw - 2, lbl);
  for (int r = 1; r < ph; r++) {
    mvaddch(py + r, px, '|');
    if (px + pw - 1 < COLS)
      mvaddch(py + r, px + pw - 1, '|');
  }
  attroff(ba);
}

/*
 * Fills a panel with the heat map: each inside cell gets a color and a glyph
 * for how much error sits at the nearest grid point. All panels share one
 * brightness scale (vmax, the biggest error anywhere this frame) so you can
 * compare them by eye -- a bright cell means the same thing in every panel.
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

/* A little progress bar across the top of the active panel showing how far
 * through this level's sweeps we are. It's right on the panel so your eyes
 * can stay on the action instead of darting to the banner below. */
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

/* Draws one level's panel: the box, the heat map inside it, and -- if this is
 * the level being worked on -- the sweep progress bar. */
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

/* Builds the centered caption for the banner, like "  PRE-SMOOTH  L1  sweep
 * 1/2  ", and returns its length so the painter can center it. */
static int format_phase_caption(const VCycle *vc, const PhaseGlyphs *g,
                                char *out, size_t cap) {
  return snprintf(out, cap, "  %s  L%d  sweep %d/%d  ", g->label, vc->level,
                  vc->smooth_count, sweep_limit(vc));
}

/* Paints a whole screen row with the fill arrow, then stamps the caption
 * over the middle of it. */
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
 * The wide banner above the bottom row, with arrows pointing the way the
 * solver is currently moving:
 *
 *   going down  '>>>>>>>>  PRE-SMOOTH  L1  sweep 1/2  >>>>>>>>'
 *   at bottom   '========  COARSE SOLVE  L3  sweep 7/20  ===='
 *   going up    '<<<<<<<<  POST-SMOOTH  L1  sweep 2/2  <<<<<<<<'
 */
static void render_phase_banner(const VCycle *vc, int row, int cols) {
  PhaseGlyphs g = phase_glyphs(vc);

  char caption[64];
  int caption_len = format_phase_caption(vc, &g, caption, sizeof caption);

  paint_banner_row(row, cols, caption, caption_len, g.fill_ch, g.cp);
}

/* The bottom two rows: the phase banner, then a line listing the keys. */
static void render_bottom_hud(const Scene *sc, int rows, int cols) {
  const VCycle *vc = &sc->vc;

  render_phase_banner(vc, rows - 2, cols);

  attron(COLOR_PAIR(CP_LABEL) | A_BOLD);
  mvprintw(rows - 1, 1,
           " SPACE pause   s step   r reset   +/- speed   q quit ");
  attroff(COLOR_PAIR(CP_LABEL) | A_BOLD);
}

/*
 * The top three status rows:
 *   Row 0: the title, with the V-cycle count off on the right
 *   Row 1: the live numbers -- level, sweeps, error sizes, convergence rate
 *   Row 2: a little map of all four levels showing where the solver is now
 */
static void render_top_hud(const Scene *sc, int cols) {
  const VCycle *vc = &sc->vc;

  /* Row 0: title on the left, V-cycle count on the right */
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

  /* Row 1: the live numbers, with the rate colored by how it's doing */
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

  /* Row 2: where the solver is, with the current level starred:
   *   going down :  L0  >>>  *L1*  >>>  L2  >>>  L3
   *   at bottom  :  L0  ===  L1  ===  L2  === *L3*
   *   going up   :  L0  <<<  L1  <<< *L2*  <<<  L3
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

/* ── §9 screen ── */

/*
 * Screen -- the terminal's current size, remembered so the drawing code
 * doesn't have to ask ncurses on every single call. We look it up once at
 * startup and again whenever the window is resized. The solver never reads
 * this -- a resize just reflows the panels, the math keeps right on going.
 *
 *   cols    how many columns wide the terminal is
 *   rows    how many rows tall
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

  /* Panels fill the space between the top and bottom HUD strips */
  int py = TOP_HUD_ROWS;
  int ph = rows - BOTTOM_HUD_ROWS - py;
  if (ph < 4)
    ph = 4;

  /* Biggest error anywhere, so every panel shares one brightness scale */
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

/* ── §10 app -- signals, resize, input, main loop ── */

/*
 * App -- the whole program in one box: the simulation, the screen, and the
 * few knobs that run the main loop. There's a single global copy because the
 * signal handlers (Ctrl-C, terminate, window resize) can't be handed a
 * pointer -- they need a fixed spot to poke, and this is it.
 *
 *   scene         the simulation -- see Scene above.
 *   screen        the terminal size -- see Screen above.
 *   sim_fps       how many solver steps run per second (1..30, default 6).
 *                 The +/- keys change it live. It's kept separate from the
 *                 ~60-fps redraw so you can slow the solver right down and
 *                 watch each sweep land.
 *   running       the main loop keeps going while this is true; the 'q' key
 *                 and the Ctrl-C / terminate handlers clear it.
 *   need_resize   set when the window is resized; the loop notices it next
 *                 time around and re-measures the screen.
 *
 * running and need_resize are volatile sig_atomic_t because a signal handler
 * writes them out of the blue while the main loop reads them -- that's the
 * one portable way to safely share a flag across that boundary.
 */
typedef struct {
  Scene scene;                       /* the simulation        */
  Screen screen;                     /* the terminal size     */
  int sim_fps;                       /* solver steps per second */
  volatile sig_atomic_t running;     /* false = quit the loop */
  volatile sig_atomic_t need_resize; /* true = window resized */
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
