/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vorticity_streamfunction_solver.c
 *   2-D water flow you can watch.  Instead of tracking speed and
 *   pressure directly, we track two simpler things: how hard the
 *   fluid is spinning at each point (ω), and a field whose contour
 *   lines happen to be the paths the fluid follows (ψ).  Each tick
 *   spins ω forward, rebuilds ψ from it, reads the velocity back out,
 *   and floats a few hundred dots along for the ride so the motion
 *   is actually visible.  Four classic scenarios cycle with p/P:
 *   the Karman vortex street behind a cylinder, a lid-driven box, a
 *   free jet, and flow over a backward step.
 *
 * Sister demos in this folder, same physics, different approach:
 *   fluid/navier_stokes.c        — Stam's stable fluids (speed+pressure)
 *   fluid/lattice_gas.c          — flow from colliding particles (FHP)
 *   fluid/shallow_water_solver.c — depth-averaged 2-D flow
 *
 * The wall-spin trick is Thom 1933; the lid-cavity test case is the
 * standard Ghia-Ghia-Shin 1982 benchmark; the cylinder wake is von
 * Karman 1911.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* §1  config — every constant + enums */

/* §1.1 — frame timing. */

#define RENDER_FPS_CAP 30
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.2 — grid limits. */

#define GRID_COLS_MAX 200
#define GRID_ROWS_MAX 60

/* §1.3 — physics. */

/* Lid / inflow speed (dimensionless; problem non-dimensionalised so
 * that U = L = 1, hence Re = 1/ν). */
#define INFLOW_VELOCITY 1.0f

/* Reynolds number presets — cycled with '+' / '-' or set per scenario. */
static const float reynolds_preset_table[] = {50.0f, 100.0f, 200.0f, 400.0f,
                                              1000.0f};
#define REYNOLDS_PRESET_COUNT 5

/* SOR over-relaxation factor.  α ∈ (1, 2); ~1.7 robust for our size. */
#define SOR_OMEGA 1.7f

/* SOR sweeps per Poisson solve. */
#define POISSON_SOR_SWEEPS 14

/* CFL safety factor for adaptive dt. */
#define CFL_SAFETY_FACTOR 0.25f

/* Sub-steps per render frame (more = faster physics evolution). */
#define SUBSTEPS_PER_FRAME 8

/* §1.4 — view modes. */

enum {
  VIEW_VORTICITY = 0,
  VIEW_STREAMLINES,
  VIEW_VELOCITY,
  VIEW_COUNT,
};

static const char *view_name_table[VIEW_COUNT] = {"vorticity ", "streamlines",
                                                  "velocity  "};

/* §1.5 — boundary side kinds. */

enum {
  BC_SIDE_WALL_STATIONARY = 0,
  BC_SIDE_WALL_MOVING,        /* lid moving at INFLOW_VELOCITY */
  BC_SIDE_INFLOW_UNIFORM,     /* uniform u = INFLOW_VELOCITY  */
  BC_SIDE_INFLOW_TOP_HALF,    /* uniform u for top half       */
  BC_SIDE_INFLOW_CENTER_BAND, /* uniform u for centre 20%     */
  BC_SIDE_OUTFLOW             /* zero-gradient                */
};

/* §1.6 — scenario IDs. */

enum {
  SCENARIO_KARMAN = 0,
  SCENARIO_LID_CAVITY = 1,
  SCENARIO_FREE_JET = 2,
  SCENARIO_BACKWARD_STEP = 3,
  SCENARIO_COUNT,
};

/* §1.7 — colour pair IDs. */

enum {
  PAIR_VORT_NEG_STRONG = 1,
  PAIR_VORT_NEG_MID,
  PAIR_VORT_NEG_WEAK,
  PAIR_VORT_ZERO,
  PAIR_VORT_POS_WEAK,
  PAIR_VORT_POS_MID,
  PAIR_VORT_POS_STRONG,

  PAIR_VEL_FIRST, /* +0..+7  velocity ramp */
  PAIR_VEL_LAST = PAIR_VEL_FIRST + 7,

  PAIR_LID,         /* yellow lid marker     */
  PAIR_OBSTACLE,    /* solid wall / cylinder */
  PAIR_TRACER,      /* tracer particles      */
  PAIR_TRACER_FAST, /* fast tracer particles */
  PAIR_HUD,         /* yellow + bold         */
  PAIR_HINT,        /* cyan + bold           */
};

/* §1.8 — tracer particles (T9). */

#define TRACER_COUNT_MAX 500
#define TRACER_COUNT_DEFAULT 400
#define TRACER_LIFETIME_MIN 200 /* ticks before respawn */
#define TRACER_LIFETIME_MAX 400

/* §1.9 — vorticity diverging-band thresholds. */

#define VORT_BAND_STRONG_FRAC 0.60f /* |ω/ω_max| > 0.60 → strong */
#define VORT_BAND_MID_FRAC 0.25f
#define VORT_BAND_WEAK_FRAC 0.05f

/* §1.10 — velocity ramp glyph count. */

#define VEL_RAMP_SIZE 8

/* §1.11 — streamline contour count. */

#define STREAMLINE_BAND_COUNT 12

/* §2  clock — monotonic ns timer + sleep */

static int64_t clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* §3  rng — small wrapper for tracer respawn */

static float rand_uniform_unit(void) { return (float)rand() / (float)RAND_MAX; }

static int rand_in_range(int lo, int hi_exclusive) {
  if (hi_exclusive <= lo)
    return lo;
  return lo + rand() % (hi_exclusive - lo);
}

/* §4  themes — diverging vorticity + sequential velocity */
/* Colours all sit in the bright half of the 256-colour space so even
 * the faintest bands stay readable on a black terminal.  Velocity
 * runs dark-blue to cyan to yellow to red; vorticity is the usual
 * blue / grey / red split for clockwise / still / counter-clockwise. */

/* Velocity glyph ramp — sparse to dense. */
static const char velocity_glyph_table[VEL_RAMP_SIZE] = {' ', '.', ':', '+',
                                                         'x', 'X', '#', '@'};

/* Tracer particle glyph ramp by speed band. */
static const char tracer_glyph_table[4] = {'.', 'o', 'O', '@'};

/* §5  colors — pair init + diverging-band picker */

static bool terminal_has_256_colours = false;

static void colors_init(void) {
  start_color();
  use_default_colors();
  terminal_has_256_colours = (COLORS >= 256);

  if (terminal_has_256_colours) {
    /* Vorticity diverging — blue (CW spin) → grey → red (CCW). */
    init_pair(PAIR_VORT_NEG_STRONG, 19, -1);  /* deep navy */
    init_pair(PAIR_VORT_NEG_MID, 27, -1);     /* mid blue  */
    init_pair(PAIR_VORT_NEG_WEAK, 117, -1);   /* light blue*/
    init_pair(PAIR_VORT_ZERO, 244, -1);       /* mid grey  */
    init_pair(PAIR_VORT_POS_WEAK, 217, -1);   /* salmon    */
    init_pair(PAIR_VORT_POS_MID, 202, -1);    /* orange    */
    init_pair(PAIR_VORT_POS_STRONG, 196, -1); /* bright red*/

    /* Velocity ramp — dark blue → cyan → yellow → red. */
    init_pair(PAIR_VEL_FIRST + 0, 19, -1);
    init_pair(PAIR_VEL_FIRST + 1, 27, -1);
    init_pair(PAIR_VEL_FIRST + 2, 39, -1);
    init_pair(PAIR_VEL_FIRST + 3, 51, -1);
    init_pair(PAIR_VEL_FIRST + 4, 118, -1);
    init_pair(PAIR_VEL_FIRST + 5, 226, -1);
    init_pair(PAIR_VEL_FIRST + 6, 208, -1);
    init_pair(PAIR_VEL_FIRST + 7, 196, -1);

    init_pair(PAIR_LID, 226, -1);         /* yellow         */
    init_pair(PAIR_OBSTACLE, 245, -1);    /* mid grey       */
    init_pair(PAIR_TRACER, 255, -1);      /* white          */
    init_pair(PAIR_TRACER_FAST, 226, -1); /* yellow         */
    init_pair(PAIR_HUD, 226, -1);         /* bright yellow  */
    init_pair(PAIR_HINT, 51, -1);         /* bright cyan    */
  } else {
    /* 8-colour fallback. */
    init_pair(PAIR_VORT_NEG_STRONG, COLOR_BLUE, -1);
    init_pair(PAIR_VORT_NEG_MID, COLOR_BLUE, -1);
    init_pair(PAIR_VORT_NEG_WEAK, COLOR_CYAN, -1);
    init_pair(PAIR_VORT_ZERO, COLOR_WHITE, -1);
    init_pair(PAIR_VORT_POS_WEAK, COLOR_MAGENTA, -1);
    init_pair(PAIR_VORT_POS_MID, COLOR_RED, -1);
    init_pair(PAIR_VORT_POS_STRONG, COLOR_RED, -1);
    for (int i = 0; i < VEL_RAMP_SIZE; i++)
      init_pair(PAIR_VEL_FIRST + i, (i < 4) ? COLOR_CYAN : COLOR_YELLOW, -1);
    init_pair(PAIR_LID, COLOR_YELLOW, -1);
    init_pair(PAIR_OBSTACLE, COLOR_WHITE, -1);
    init_pair(PAIR_TRACER, COLOR_WHITE, -1);
    init_pair(PAIR_TRACER_FAST, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/*
 * VortBand — how to draw one cell, decided from how hard it's spinning.
 *
 * Spin is signed: counter-clockwise is positive, clockwise negative,
 * and the two need opposite colours.  vorticity_band_for() takes the
 * spin (scaled to roughly -1..1) and sorts it into one of seven tiers
 * from strong-clockwise through still to strong-counter-clockwise,
 * handing back the colour, character, and brightness all at once.
 *
 * It's a struct only so the per-cell draw loop reads as one line —
 * a C function can return just one value.  Glyph ramp follows Bourke.
 */
typedef struct {
    int    pair;      /* which colour to use (spin tier)           */
    char   glyph;     /* the character to draw                     */
    attr_t attr;      /* bold for the strongest tiers             */
} VortBand;

static VortBand vorticity_band_for(float omega_normalised) {
  if (omega_normalised < -VORT_BAND_STRONG_FRAC)
    return (VortBand){PAIR_VORT_NEG_STRONG, '#', A_BOLD};
  if (omega_normalised < -VORT_BAND_MID_FRAC)
    return (VortBand){PAIR_VORT_NEG_MID, 'x', A_NORMAL};
  if (omega_normalised < -VORT_BAND_WEAK_FRAC)
    return (VortBand){PAIR_VORT_NEG_WEAK, '.', A_NORMAL};
  if (omega_normalised < VORT_BAND_WEAK_FRAC)
    return (VortBand){PAIR_VORT_ZERO, ' ', A_NORMAL};
  if (omega_normalised < VORT_BAND_MID_FRAC)
    return (VortBand){PAIR_VORT_POS_WEAK, '.', A_NORMAL};
  if (omega_normalised < VORT_BAND_STRONG_FRAC)
    return (VortBand){PAIR_VORT_POS_MID, 'x', A_NORMAL};
  return (VortBand){PAIR_VORT_POS_STRONG, '#', A_BOLD};
}

/* §6  ramp — value to glyph-slot helpers */

static int speed_to_velocity_slot(float speed, float speed_max) {
  if (speed_max < 1e-6f)
    return 0;
  int slot = (int)(speed / speed_max * (float)(VEL_RAMP_SIZE - 1) + 0.5f);
  if (slot < 0)
    slot = 0;
  if (slot >= VEL_RAMP_SIZE)
    slot = VEL_RAMP_SIZE - 1;
  return slot;
}

/* §7  grid_state — psi, omega, omega_next, u, v, walls */
/*
 * Every field is one flat array, one float per cell, laid out row by
 * row; cell_index(x, y) finds a cell.  y = 0 is the BOTTOM of the
 * fluid, but the screen draws top-down, so the render code flips y —
 * that way the top of the window is the top of the tank.
 */

#define GRID_TOTAL_CELLS (GRID_COLS_MAX * GRID_ROWS_MAX)

/*
 * Scene — all of the demo's live state in one place.
 *
 * It's one big struct (reached through the g_scene global) rather than
 * a pile of loose globals so the field arrays sit together and nobody
 * has to thread pointers through the tight inner loops.  Everything is
 * allocated once here, in BSS — no malloc ever runs while the sim is
 * going.  The fields below are grouped by job: the simulation arrays
 * the solver reads and writes, the grid geometry both the solver and
 * the drawing share, the Reynolds-number knobs, and two render-only
 * smoothing values that must never feed back into the physics (if a
 * view changed the viscosity, the same scenario would evolve
 * differently depending on what you were looking at).
 *
 * The tracer dots, current view, scenario choice, and pause flag live
 * with their own code in §14/§15/§22, not here.
 */
typedef struct {
    /* ── The simulation itself ─────────────────────────────────── */
    float streamfunction_psi  [GRID_TOTAL_CELLS];  /* ψ — the flow field */
    float vorticity_omega     [GRID_TOTAL_CELLS];  /* ω — the spin field */
    float vorticity_omega_next[GRID_TOTAL_CELLS];  /* scratch for the spin update */
    float velocity_x          [GRID_TOTAL_CELLS];  /* speed across, read out of ψ */
    float velocity_y          [GRID_TOTAL_CELLS];  /* speed up/down, read out of ψ */
    bool  wall_mask           [GRID_TOTAL_CELLS];  /* true where a wall/obstacle is */

    /* ── Grid size + cell spacing (used by sim and drawing) ────── */
    int   grid_cols;
    int   grid_rows;
    float cell_size_x;          /* width of one cell                 */
    float cell_size_y;          /* height of one cell                */

    /* ── How thick/sticky the fluid is ─────────────────────────── */
    float kinematic_viscosity;  /* stickiness; = 1 / Reynolds        */
    float reynolds_number;      /* fast-and-swirly vs slow-and-smooth */
    int   reynolds_preset_index;/* which +/- preset we're on         */
    float current_dt;           /* time step, re-picked each tick     */

    /* ── Render-only smoothing (never touch the sim) ───────────── */
    float velocity_max_smoothed;  /* running peak speed, for colour scale */
    float vorticity_max_smoothed; /* running peak spin, for colour scale  */
} Scene;

static Scene g_scene = {
    .cell_size_x            = 1.0f,
    .cell_size_y            = 1.0f,
    .kinematic_viscosity    = 1.0f / 100.0f,
    .reynolds_number        = 100.0f,
    .reynolds_preset_index  = 1,
    .current_dt             = 0.001f,
    .velocity_max_smoothed  = 1.0f,
    .vorticity_max_smoothed = 1.0f,
};

static inline int cell_index(int x, int y) { return y * g_scene.grid_cols + x; }

static void grid_zero_all_fields(void) {
  int n = g_scene.grid_rows * g_scene.grid_cols;
  memset(g_scene.streamfunction_psi, 0, sizeof(float) * n);
  memset(g_scene.vorticity_omega, 0, sizeof(float) * n);
  memset(g_scene.vorticity_omega_next, 0, sizeof(float) * n);
  memset(g_scene.velocity_x, 0, sizeof(float) * n);
  memset(g_scene.velocity_y, 0, sizeof(float) * n);
}

static void grid_clear_walls(void) {
  int n = g_scene.grid_rows * g_scene.grid_cols;
  memset(g_scene.wall_mask, 0, sizeof(bool) * n);
}

/* §8  obstacle_layouts — cylinder, step builders */

static void obstacle_build_cylinder(float cx_frac, float cy_frac,
                                    float radius_frac) {
  float cx = cx_frac * (float)(g_scene.grid_cols - 1);
  float cy = cy_frac * (float)(g_scene.grid_rows - 1);
  float radius =
      radius_frac * (float)((g_scene.grid_cols < g_scene.grid_rows) ? g_scene.grid_cols : g_scene.grid_rows);
  float r_squared = radius * radius;

  for (int y = 0; y < g_scene.grid_rows; y++) {
    for (int x = 0; x < g_scene.grid_cols; x++) {
      float dx = (float)x - cx;
      float dy = (float)y - cy;
      if (dx * dx + dy * dy < r_squared)
        g_scene.wall_mask[cell_index(x, y)] = true;
    }
  }
}

static void obstacle_build_step(float step_height_frac) {
  int step_height = (int)(step_height_frac * (float)g_scene.grid_rows);
  int step_length = g_scene.grid_cols / 8;
  if (step_length < 1)
    step_length = 1;

  for (int y = 0; y < step_height; y++) {
    for (int x = 0; x < step_length; x++) {
      g_scene.wall_mask[cell_index(x, y)] = true;
    }
  }
}

/* §9  apply_boundary — per-side rules + Thom's wall-spin formula */

/*
 * BoundarySpec — what each of the four walls does in a scenario.
 *
 * Every wall can behave differently: in the lid-driven box the top
 * slides while the other three sit still; in pipe flow the fluid
 * comes in the left and leaves the right with solid walls top and
 * bottom.  So we keep one rule per wall (a solid wall, a moving wall,
 * one of three inflow shapes, or an open outflow) instead of a single
 * setting for the whole box.  inflow_velocity is the one speed shared
 * by whichever sides let fluid in — every scenario here uses the same
 * inflow speed, so one number covers it.
 *
 * The wall-spin formula those rules feed into is Thom (1933).
 */
typedef struct {
    int   side_top;          /* rule for the top wall               */
    int   side_right;         /* rule for the right wall            */
    int   side_bottom;        /* rule for the bottom wall           */
    int   side_left;          /* rule for the left wall             */
    float inflow_velocity;    /* speed where fluid flows in         */
} BoundarySpec;

/* Even inflow across the whole left wall: ψ climbs steadily up the
 * wall so the fluid enters at one steady speed everywhere. */
static inline float psi_uniform_inflow_profile(int y, int rows,
                                                float inflow_velocity) {
    (void)rows;
    return inflow_velocity * (float)y * g_scene.cell_size_y;
}

/* Inflow only above the step (backward-step scenario): nothing comes
 * in over the lower quarter, which acts as a solid shelf, and the
 * fluid above it pours over the edge. */
static inline float psi_top_half_inflow_profile(int y, int rows,
                                                 float inflow_velocity) {
    int step_h = rows / 4;
    if (y < step_h) return 0.0f;
    return inflow_velocity * (float)(y - step_h) * g_scene.cell_size_y;
}

/* Inflow through a narrow centre slot (free jet): fluid is driven in
 * only over the middle 40-60% band, modelling a jet squirting out of
 * a slot.  Above the band the fluid is just dragged along, below it
 * stays put. */
static inline float psi_centre_band_inflow_profile(int y, int rows,
                                                    float inflow_velocity) {
    int band_lo = rows * 4 / 10;
    int band_hi = rows * 6 / 10;
    if (y <  band_lo) return 0.0f;
    if (y >  band_hi)
        return inflow_velocity * (float)(band_hi - band_lo) * g_scene.cell_size_y;
    return inflow_velocity * (float)(y - band_lo) * g_scene.cell_size_y;
}

/* Pick the matching inflow shape for this wall's rule and read its ψ
 * at height y.  A plain wall just gets ψ = 0. */
static inline float psi_inflow_value_at(int side_kind, int y, int rows,
                                         float inflow_velocity) {
    switch (side_kind) {
        case BC_SIDE_INFLOW_UNIFORM:
            return psi_uniform_inflow_profile    (y, rows, inflow_velocity);
        case BC_SIDE_INFLOW_TOP_HALF:
            return psi_top_half_inflow_profile   (y, rows, inflow_velocity);
        case BC_SIDE_INFLOW_CENTER_BAND:
            return psi_centre_band_inflow_profile(y, rows, inflow_velocity);
        default:
            return 0.0f;
    }
}

/* Left wall: write the chosen inflow shape (or zero) down column 0. */
static inline void apply_psi_on_left_side(int side_kind, float inflow_velocity,
                                           int rows) {
    for (int y = 0; y < rows; y++)
        g_scene.streamfunction_psi[cell_index(0, y)] =
            psi_inflow_value_at(side_kind, y, rows, inflow_velocity);
}

/* Right wall: if fluid flows out here, just copy the column next to
 * the edge so the flow leaves smoothly.  Solid walls are left alone. */
static inline void apply_psi_on_right_side(int side_kind, int rows, int cols) {
    if (side_kind != BC_SIDE_OUTFLOW) return;
    for (int y = 0; y < rows; y++)
        g_scene.streamfunction_psi[cell_index(cols - 1, y)] =
            g_scene.streamfunction_psi[cell_index(cols - 2, y)];
}

/* Bottom wall: pin ψ = 0 along the floor.  This is our reference
 * line; every other ψ value is measured up from here. */
static inline void apply_psi_on_bottom_side(int cols) {
    for (int x = 0; x < cols; x++)
        g_scene.streamfunction_psi[cell_index(x, 0)] = 0.0f;
}

/* Top wall: a still wall above a uniform inflow has to carry the ψ
 * value the inflow reaches at the top, so the top streamline lines up
 * with the incoming flow instead of kinking at the corner. */
static inline void apply_psi_on_top_side(int side_kind, float inflow_velocity,
                                          int rows, int cols) {
    float psi_top = 0.0f;
    if (side_kind == BC_SIDE_WALL_STATIONARY)
        psi_top = inflow_velocity * (float)(rows - 1) * g_scene.cell_size_y;
    for (int x = 0; x < cols; x++)
        g_scene.streamfunction_psi[cell_index(x, rows - 1)] = psi_top;
}

/* Set the ψ values along whichever wall you name (L/R/B/T). */
static void boundary_set_psi_side(int side_kind, char which_side,
                                   float inflow_velocity) {
    int rows = g_scene.grid_rows;
    int cols = g_scene.grid_cols;
    switch (which_side) {
        case 'L': apply_psi_on_left_side  (side_kind, inflow_velocity, rows);       break;
        case 'R': apply_psi_on_right_side (side_kind, rows, cols);                  break;
        case 'B': apply_psi_on_bottom_side(cols);                                   break;
        case 'T': apply_psi_on_top_side   (side_kind, inflow_velocity, rows, cols); break;
        default:                                                                    break;
    }
}

/* Thom's trick: a wall where the fluid can't slip generates spin, and
 * the amount is fixed by the ψ value one cell in from the wall.  This
 * is how we work that spin out (Thom 1933). */
static inline float thom_omega_from_inner_psi(float psi_inner, float h_squared) {
    return -2.0f * psi_inner / h_squared;
}

/* Set the spin along the bottom wall (only when it's a still wall). */
static inline void apply_thom_on_bottom_wall(const BoundarySpec *spec,
                                              int cols, float dy_squared) {
    if (spec->side_bottom != BC_SIDE_WALL_STATIONARY) return;
    for (int x = 0; x < cols; x++) {
        float psi_inner = g_scene.streamfunction_psi[cell_index(x, 1)];
        g_scene.vorticity_omega[cell_index(x, 0)] =
            thom_omega_from_inner_psi(psi_inner, dy_squared);
    }
}

/* Set the spin along the top wall.  If the top is a sliding lid, add
 * an extra term for the spin its own movement drags into the fluid. */
static inline void apply_thom_on_top_wall(const BoundarySpec *spec,
                                           int rows, int cols, float dy_squared) {
    for (int x = 0; x < cols; x++) {
        float psi_inner = g_scene.streamfunction_psi[cell_index(x, rows - 2)];
        float psi_wall  = g_scene.streamfunction_psi[cell_index(x, rows - 1)];
        float w_thom    = -2.0f * (psi_inner - psi_wall) / dy_squared;
        if (spec->side_top == BC_SIDE_WALL_MOVING)
            w_thom -= 2.0f * spec->inflow_velocity / g_scene.cell_size_y;
        g_scene.vorticity_omega[cell_index(x, rows - 1)] = w_thom;
    }
}

/* Set the spin along the left side: the wall formula if it's a wall,
 * or zero where fluid flows in (incoming flow isn't spinning yet). */
static inline void apply_thom_on_left_side(const BoundarySpec *spec,
                                            int rows, float dx_squared) {
    if (spec->side_left == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            float psi_inner = g_scene.streamfunction_psi[cell_index(1, y)];
            g_scene.vorticity_omega[cell_index(0, y)] =
                thom_omega_from_inner_psi(psi_inner, dx_squared);
        }
    } else {
        for (int y = 0; y < rows; y++)
            g_scene.vorticity_omega[cell_index(0, y)] = 0.0f;
    }
}

/* Set the spin along the right side: the wall formula if it's a wall,
 * or just copy the neighbour's spin where fluid flows out. */
static inline void apply_thom_on_right_side(const BoundarySpec *spec,
                                             int rows, int cols, float dx_squared) {
    if (spec->side_right == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            float psi_inner = g_scene.streamfunction_psi[cell_index(cols - 2, y)];
            g_scene.vorticity_omega[cell_index(cols - 1, y)] =
                thom_omega_from_inner_psi(psi_inner, dx_squared);
        }
    } else {
        for (int y = 0; y < rows; y++)
            g_scene.vorticity_omega[cell_index(cols - 1, y)] =
                g_scene.vorticity_omega[cell_index(cols - 2, y)];
    }
}

/* Average ψ of the (up to four) fluid cells touching cell (x, y).
 * Used at obstacle cells to stand in for the wall formula: cells on
 * the obstacle's edge see fluid and give a sensible spin; cells buried
 * deep inside see none and report 0, which the caller treats as no
 * spin. */
static inline float average_psi_of_fluid_neighbours(int x, int y,
                                                     int *out_neighbour_count) {
    int  count    = 0;
    float psi_sum = 0.0f;
    int   neighbours[4][2] = { {x - 1, y}, {x + 1, y},
                               {x, y - 1}, {x, y + 1} };
    for (int n = 0; n < 4; n++) {
        int nx = neighbours[n][0];
        int ny = neighbours[n][1];
        if (!g_scene.wall_mask[cell_index(nx, ny)]) {
            psi_sum += g_scene.streamfunction_psi[cell_index(nx, ny)];
            count++;
        }
    }
    *out_neighbour_count = count;
    return (count > 0) ? psi_sum / (float)count : 0.0f;
}

/* Walk every obstacle cell: give edge cells their wall spin from the
 * surrounding fluid, give buried cells no spin, and zero ψ inside the
 * obstacle so no flow leaks through it. */
static inline void apply_thom_inside_obstacle_cells(int rows, int cols,
                                                     float dy_squared) {
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (!g_scene.wall_mask[cell_index(x, y)]) continue;
            int   count;
            float psi_avg = average_psi_of_fluid_neighbours(x, y, &count);
            g_scene.vorticity_omega   [cell_index(x, y)] =
                (count > 0) ? thom_omega_from_inner_psi(psi_avg, dy_squared) : 0.0f;
            g_scene.streamfunction_psi[cell_index(x, y)] = 0.0f;
        }
    }
}

/* Fill in the spin at every wall and obstacle for this tick. */
static void boundary_set_thom_wall_vorticity(const BoundarySpec *spec) {
    int   rows = g_scene.grid_rows;
    int   cols = g_scene.grid_cols;
    float dx2  = g_scene.cell_size_x * g_scene.cell_size_x;
    float dy2  = g_scene.cell_size_y * g_scene.cell_size_y;

    apply_thom_on_bottom_wall      (spec, cols, dy2);
    apply_thom_on_top_wall         (spec, rows, cols, dy2);
    apply_thom_on_left_side        (spec, rows, dx2);
    apply_thom_on_right_side       (spec, rows, cols, dx2);
    apply_thom_inside_obstacle_cells(rows, cols, dy2);
}

static void apply_boundary(const BoundarySpec *spec) {
  boundary_set_psi_side(spec->side_left, 'L', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_right, 'R', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_bottom, 'B', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_top, 'T', spec->inflow_velocity);
  boundary_set_thom_wall_vorticity(spec);
}

/* §10  vorticity_step — step the spin field forward in time */

/* A cell's own spin plus its four neighbours' spin and its own
 * velocity, grabbed together so the update below reads like the
 * physics instead of a wall of array lookups. */
typedef struct {
    float omega_centre, omega_east, omega_west, omega_north, omega_south;
    float u_local, v_local;
} OmegaStencil;

static inline OmegaStencil sample_omega_stencil_at(int x, int y) {
    OmegaStencil s;
    s.omega_centre = g_scene.vorticity_omega[cell_index(x,     y    )];
    s.omega_east   = g_scene.vorticity_omega[cell_index(x + 1, y    )];
    s.omega_west   = g_scene.vorticity_omega[cell_index(x - 1, y    )];
    s.omega_north  = g_scene.vorticity_omega[cell_index(x,     y + 1)];
    s.omega_south  = g_scene.vorticity_omega[cell_index(x,     y - 1)];
    s.u_local      = g_scene.velocity_x     [cell_index(x,     y    )];
    s.v_local      = g_scene.velocity_y     [cell_index(x,     y    )];
    return s;
}

/* How fast spin changes as the flow carries it sideways.  We always
 * look UPSTREAM (toward where the fluid is coming from) rather than
 * downstream; using the downstream side here makes the sim wobble and
 * blow up.  (LeVeque, upwind scheme.) */
static inline float upwind_omega_gradient_x(const OmegaStencil *s, float idx) {
    return (s->u_local > 0.0f)
         ? (s->omega_centre - s->omega_west)  * idx
         : (s->omega_east   - s->omega_centre) * idx;
}

static inline float upwind_omega_gradient_y(const OmegaStencil *s, float idy) {
    return (s->v_local > 0.0f)
         ? (s->omega_centre - s->omega_south) * idy
         : (s->omega_north  - s->omega_centre) * idy;
}

/* How much a cell's spin differs from its neighbours' average — this
 * is what stickiness smooths out.  Here we can look both ways evenly,
 * since smoothing has no preferred direction. */
static inline float laplacian_omega_at(const OmegaStencil *s, float idx2, float idy2) {
    return (s->omega_east  - 2.0f * s->omega_centre + s->omega_west ) * idx2
         + (s->omega_north - 2.0f * s->omega_centre + s->omega_south) * idy2;
}

/* How fast a cell's spin is changing right now: the flow drags it
 * along (the two minus terms) while stickiness smooths it out (the
 * last term).  Spin is only made at walls, never here in the open. */
static inline float vorticity_time_derivative(const OmegaStencil *s,
                                               float idx, float idy,
                                               float idx2, float idy2, float nu) {
    float dw_dx     = upwind_omega_gradient_x(s, idx);
    float dw_dy     = upwind_omega_gradient_y(s, idy);
    float lap_omega = laplacian_omega_at      (s, idx2, idy2);
    return - s->u_local * dw_dx
           - s->v_local * dw_dy
           + nu        * lap_omega;
}

/* Take one time step of spin for every open cell, writing the result
 * into the scratch array.  Wall cells are left out — their spin comes
 * from apply_boundary, not from this update. */
static inline void compute_vorticity_next_field(float dt_seconds) {
    int   cols = g_scene.grid_cols;
    int   rows = g_scene.grid_rows;
    float idx  = 1.0f / g_scene.cell_size_x;
    float idy  = 1.0f / g_scene.cell_size_y;
    float idx2 = idx * idx;
    float idy2 = idy * idy;
    float nu   = g_scene.kinematic_viscosity;

    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            OmegaStencil s   = sample_omega_stencil_at(x, y);
            float        rhs = vorticity_time_derivative(&s, idx, idy,
                                                          idx2, idy2, nu);
            g_scene.vorticity_omega_next[cell_index(x, y)] =
                s.omega_centre + dt_seconds * rhs;
        }
    }
}

/* Copy the freshly-stepped spin back into the live array.  We copy
 * only the open cells we touched rather than swap whole arrays. */
static inline void copy_omega_next_to_omega_interior(void) {
    int cols = g_scene.grid_cols;
    int rows = g_scene.grid_rows;
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            g_scene.vorticity_omega[cell_index(x, y)] =
                g_scene.vorticity_omega_next[cell_index(x, y)];
        }
    }
}

/* Advance the whole spin field by one time step. */
static void vorticity_step(float dt_seconds) {
    compute_vorticity_next_field(dt_seconds);
    copy_omega_next_to_omega_interior();
}

/* §11  poisson_solve — rebuild the flow field ψ from the spin field */

/* The best guess for one cell's ψ given its four neighbours and the
 * spin sitting there.  We pretend the neighbours are already right;
 * they aren't yet, but repeating this over the grid converges to the
 * answer.  (Gauss-Seidel; Saad.) */
static inline float gauss_seidel_psi_update_at(int x, int y,
                                                float dx2, float dy2,
                                                float denominator) {
    float psi_east  = g_scene.streamfunction_psi[cell_index(x + 1, y    )];
    float psi_west  = g_scene.streamfunction_psi[cell_index(x - 1, y    )];
    float psi_north = g_scene.streamfunction_psi[cell_index(x,     y + 1)];
    float psi_south = g_scene.streamfunction_psi[cell_index(x,     y - 1)];
    float omega     = g_scene.vorticity_omega   [cell_index(x,     y    )];
    return (dy2 * (psi_east + psi_west) + dx2 * (psi_north + psi_south)
            + dx2 * dy2 * omega) / denominator;
}

/* Don't just move ψ to the new guess — overshoot a bit past it.  That
 * deliberate overshoot (alpha around 1.5-1.8) reaches the answer in
 * far fewer passes.  alpha = 1 means no overshoot; past 2 it blows
 * up.  (Successive over-relaxation; Saad.) */
static inline float sor_relax(float psi_centre, float psi_gauss_seidel,
                               float alpha) {
    return psi_centre + alpha * (psi_gauss_seidel - psi_centre);
}

/* One pass over every open cell, nudging each ψ toward its new guess.
 * This solve is the slowest part of the program, so the overshoot
 * speedup matters. */
static inline void sor_sweep_once(float dx2, float dy2,
                                   float denominator, float alpha) {
    int cols = g_scene.grid_cols;
    int rows = g_scene.grid_rows;
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            float psi_centre = g_scene.streamfunction_psi[cell_index(x, y)];
            float psi_gs     = gauss_seidel_psi_update_at(x, y, dx2, dy2,
                                                          denominator);
            g_scene.streamfunction_psi[cell_index(x, y)] =
                sor_relax(psi_centre, psi_gs, alpha);
        }
    }
}

/* Rebuild ψ from the current spin by sweeping the grid a fixed number
 * of times — enough passes to get visually exact. */
static void poisson_solve_sor(void) {
    float dx2          = g_scene.cell_size_x * g_scene.cell_size_x;
    float dy2          = g_scene.cell_size_y * g_scene.cell_size_y;
    float denominator  = 2.0f * (dx2 + dy2);
    float alpha        = SOR_OMEGA;

    for (int sweep = 0; sweep < POISSON_SOR_SWEEPS; sweep++)
        sor_sweep_once(dx2, dy2, denominator, alpha);
}

/* §12  velocity_recompute — read speeds out of ψ, pick the next dt */

/* Read one cell's velocity out of ψ.  ψ changing across the grid IS
 * the flow: how fast it changes up/down gives the sideways speed, and
 * how fast it changes left/right gives the up/down speed (with a sign
 * flip).  We measure each slope from the two neighbouring cells. */
static inline void velocity_from_psi_at_cell(int x, int y,
                                              float inv_2dx, float inv_2dy,
                                              float *out_u, float *out_v) {
    *out_u =  (g_scene.streamfunction_psi[cell_index(x,     y + 1)]
             - g_scene.streamfunction_psi[cell_index(x,     y - 1)]) * inv_2dy;
    *out_v = -(g_scene.streamfunction_psi[cell_index(x + 1, y    )]
             - g_scene.streamfunction_psi[cell_index(x - 1, y    )]) * inv_2dx;
}

/* Read velocities out of ψ for every open cell; pin wall cells to
 * zero so nothing flows through them.  Hands back the top speed seen,
 * which the caller uses to size the time step and the colours. */
static inline float recompute_interior_velocity_from_psi(void) {
    int   cols    = g_scene.grid_cols;
    int   rows    = g_scene.grid_rows;
    float inv_2dx = 0.5f / g_scene.cell_size_x;
    float inv_2dy = 0.5f / g_scene.cell_size_y;
    float speed_max = 0.0f;

    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) {
                g_scene.velocity_x[cell_index(x, y)] = 0.0f;
                g_scene.velocity_y[cell_index(x, y)] = 0.0f;
                continue;
            }
            float u_here, v_here;
            velocity_from_psi_at_cell(x, y, inv_2dx, inv_2dy, &u_here, &v_here);
            g_scene.velocity_x[cell_index(x, y)] = u_here;
            g_scene.velocity_y[cell_index(x, y)] = v_here;
            float speed = sqrtf(u_here * u_here + v_here * v_here);
            if (speed > speed_max) speed_max = speed;
        }
    }
    return speed_max;
}

/* The inflow speed at height y on the left wall.  We set the velocity
 * directly here so the next step has correct speeds to work with
 * before ψ and the spin are refreshed. */
static inline float left_inflow_u_at(int y, int rows,
                                      const BoundarySpec *spec) {
    switch (spec->side_left) {
        case BC_SIDE_INFLOW_UNIFORM:
            return spec->inflow_velocity;
        case BC_SIDE_INFLOW_TOP_HALF: {
            int step_h = rows / 4;
            return (y >= step_h) ? spec->inflow_velocity : 0.0f;
        }
        case BC_SIDE_INFLOW_CENTER_BAND: {
            int band_lo = rows * 4 / 10;
            int band_hi = rows * 6 / 10;
            return (y >= band_lo && y <= band_hi) ? spec->inflow_velocity : 0.0f;
        }
        default:
            return 0.0f;
    }
}

/* Force the velocities along the top and bottom walls.  The bottom is
 * always still; the top moves at the lid speed in the lid-cavity
 * scenario and is still otherwise. */
static inline void apply_wall_velocity_top_and_bottom(const BoundarySpec *spec,
                                                       int rows, int cols) {
    for (int x = 0; x < cols; x++) {
        g_scene.velocity_x[cell_index(x, 0       )] = 0.0f;
        g_scene.velocity_y[cell_index(x, 0       )] = 0.0f;
        g_scene.velocity_x[cell_index(x, rows - 1)] =
            (spec->side_top == BC_SIDE_WALL_MOVING) ? spec->inflow_velocity : 0.0f;
        g_scene.velocity_y[cell_index(x, rows - 1)] = 0.0f;
    }
}

/* Force the velocities along the left and right walls.  Left takes an
 * inflow speed (or zero for a wall); right is either still or, for an
 * outflow, just copies the column beside it so the flow slides out. */
static inline void apply_wall_velocity_left_and_right(const BoundarySpec *spec,
                                                       int rows, int cols) {
    for (int y = 0; y < rows; y++) {
        g_scene.velocity_x[cell_index(0, y)] = left_inflow_u_at(y, rows, spec);
        g_scene.velocity_y[cell_index(0, y)] = 0.0f;
        if (spec->side_right == BC_SIDE_OUTFLOW) {
            g_scene.velocity_x[cell_index(cols - 1, y)] =
                g_scene.velocity_x[cell_index(cols - 2, y)];
            g_scene.velocity_y[cell_index(cols - 1, y)] =
                g_scene.velocity_y[cell_index(cols - 2, y)];
        } else {
            g_scene.velocity_x[cell_index(cols - 1, y)] = 0.0f;
            g_scene.velocity_y[cell_index(cols - 1, y)] = 0.0f;
        }
    }
}

/* Ease the "fastest speed" value used to scale the velocity colours
 * toward the latest peak.  Blending slowly (about a 12-frame lag)
 * keeps a one-frame spike from making the whole picture flash, while
 * still following a flow that's genuinely speeding up.  Kept above a
 * tiny floor so later divisions never hit zero. */
static inline void update_smoothed_velocity_max(float speed_max) {
    if (speed_max < 1e-6f) speed_max = 1e-6f;
    g_scene.velocity_max_smoothed = 0.92f * g_scene.velocity_max_smoothed
                                  + 0.08f * speed_max;
    if (g_scene.velocity_max_smoothed < 1e-6f)
        g_scene.velocity_max_smoothed = 1e-6f;
}

/* Pick the next time step.  Two things can break the sim if a step is
 * too big: fast flow jumping more than a cell, and stickiness
 * smoothing too aggressively.  We work out the safe step for each and
 * take the smaller, with a safety margin under 1. */
static inline void update_adaptive_dt(float speed_max) {
    float h          = g_scene.cell_size_x;
    float dt_adv     = CFL_SAFETY_FACTOR * h / speed_max;
    float dt_diff    = CFL_SAFETY_FACTOR * h * h
                     / (4.0f * g_scene.kinematic_viscosity);
    float dt         = (dt_adv < dt_diff) ? dt_adv : dt_diff;
    if (dt < 1e-6f)  dt = 1e-6f;
    if (dt > 0.01f)  dt = 0.01f;
    g_scene.current_dt = dt;
}

/* Read all the velocities out of ψ, force the wall values, then size
 * the next time step from the fastest flow seen. */
static void velocity_recompute_and_adapt_dt(const BoundarySpec *spec) {
    int rows = g_scene.grid_rows;
    int cols = g_scene.grid_cols;

    float speed_max = recompute_interior_velocity_from_psi();
    apply_wall_velocity_top_and_bottom (spec, rows, cols);
    apply_wall_velocity_left_and_right (spec, rows, cols);

    update_smoothed_velocity_max(speed_max);
    update_adaptive_dt          (speed_max);
}

/* §13  ns_step — one full physics tick */

/* Find the strongest spin anywhere and ease the colour-scaling value
 * toward it.  Even slower blending than for speed, because spin spikes
 * are sharper and we'd rather the colours ignore them. */
static inline void update_smoothed_vorticity_max(void) {
    int   n    = g_scene.grid_cols * g_scene.grid_rows;
    float wmax = 1e-6f;
    for (int i = 0; i < n; i++) {
        float aw = fabsf(g_scene.vorticity_omega[i]);
        if (aw > wmax) wmax = aw;
    }
    g_scene.vorticity_max_smoothed = 0.95f * g_scene.vorticity_max_smoothed
                                   + 0.05f * wmax;
    if (g_scene.vorticity_max_smoothed < 1e-6f)
        g_scene.vorticity_max_smoothed = 1e-6f;
}

/* One full step: spin the fluid forward, fix the walls, rebuild the
 * flow field, read the new speeds, fix the walls again so the next
 * step starts clean, and refresh the colour scale.  No pressure
 * anywhere — that's the whole point of working in spin + flow. */
static void ns_step(const BoundarySpec *spec) {
    vorticity_step                 (g_scene.current_dt);
    apply_boundary                 (spec);
    poisson_solve_sor              ();
    velocity_recompute_and_adapt_dt(spec);
    apply_boundary                 (spec);
    update_smoothed_vorticity_max  ();
}

/* §14  tracers — floating dots that ride the flow */

/*
 * Tracer — one dot that drifts along with the fluid.
 *
 * The spin and flow fields are just colours; they don't show which
 * way the water is actually going.  These dots do: each step we look
 * up the flow speed under a dot and nudge it that way, so over time
 * the dots trace out the streams and swirls.  Toggle the whole layer
 * off if you'd rather see the field underneath uncluttered.
 *
 * They live in one fixed-size array reused forever (no allocation
 * mid-run; the +/- keys just change how many are active).  Positions
 * are real numbers, not whole cells, so a dot can sit between cells
 * and move smoothly instead of jumping cell to cell.  Each dot also
 * carries a countdown: when it runs out the dot respawns somewhere
 * fresh, which keeps them from all piling up in the slow corners and
 * leaving the fast flow bare.
 */
typedef struct {
    float pos_x;            /* position across, can be between cells */
    float pos_y;            /* position up/down, can be between cells */
    int   ticks_remaining;  /* respawn when this reaches 0          */
} Tracer;

static Tracer tracer_pool[TRACER_COUNT_MAX];
static int active_tracer_count = TRACER_COUNT_DEFAULT;
static bool tracer_overlay_enabled = true;

/* Pull a sample point back inside the grid so the look-up below never
 * reaches off the edge. */
static inline void clamp_tracer_sample_in_grid(float *px, float *py) {
    if (*px < 0.0f)                            *px = 0.0f;
    if (*py < 0.0f)                            *py = 0.0f;
    if (*px > (float)(g_scene.grid_cols - 1))  *px = (float)(g_scene.grid_cols - 1);
    if (*py > (float)(g_scene.grid_rows - 1))  *py = (float)(g_scene.grid_rows - 1);
}

/* Break a between-cells point into the cell it sits in plus how far
 * into that cell it is.  The right/top neighbours are clamped to the
 * edge so a point right on the boundary still gives valid cells. */
static inline void integer_cell_and_subcell_2d(float px, float py,
                                                int *out_x0, int *out_y0,
                                                int *out_x1, int *out_y1,
                                                float *out_fx, float *out_fy) {
    int x0 = (int)px;
    int y0 = (int)py;
    *out_x0 = x0;
    *out_y0 = y0;
    *out_x1 = (x0 + 1 < g_scene.grid_cols) ? x0 + 1 : x0;
    *out_y1 = (y0 + 1 < g_scene.grid_rows) ? y0 + 1 : y0;
    *out_fx = px - (float)x0;
    *out_fy = py - (float)y0;
}

/* Smoothly blend four corner values by how close the point is to
 * each.  Copied into this file on purpose, not shared, so the file
 * stands alone (CLAUDE.md). */
static inline float bilinear_blend(float c00, float c10, float c01, float c11,
                                    float fx, float fy) {
    return (1.0f - fx) * (1.0f - fy) * c00
         +         fx  * (1.0f - fy) * c10
         + (1.0f - fx) *         fy  * c01
         +         fx  *         fy  * c11;
}

/* The flow speed at a between-cells point, blended from the four cells
 * around it. */
static void velocity_at_fractional(float px, float py, float *out_u,
                                    float *out_v) {
    clamp_tracer_sample_in_grid(&px, &py);

    int   x0, y0, x1, y1;
    float fx, fy;
    integer_cell_and_subcell_2d(px, py, &x0, &y0, &x1, &y1, &fx, &fy);

    *out_u = bilinear_blend(
        g_scene.velocity_x[cell_index(x0, y0)],
        g_scene.velocity_x[cell_index(x1, y0)],
        g_scene.velocity_x[cell_index(x0, y1)],
        g_scene.velocity_x[cell_index(x1, y1)],
        fx, fy);
    *out_v = bilinear_blend(
        g_scene.velocity_y[cell_index(x0, y0)],
        g_scene.velocity_y[cell_index(x1, y0)],
        g_scene.velocity_y[cell_index(x0, y1)],
        g_scene.velocity_y[cell_index(x1, y1)],
        fx, fy);
}

/* Lid cavity has no inflow, so respawn dots anywhere inside the box
 * to keep them spread through the swirl. */
static inline void place_tracer_lid_cavity_interior(Tracer *t) {
    t->pos_x = (float)rand_in_range(1, g_scene.grid_cols - 1);
    t->pos_y = (float)rand_in_range(1, g_scene.grid_rows - 1);
}

/* Backward step: drop dots into the inflow above the step so they
 * ride over the edge and reveal the swirl behind it. */
static inline void place_tracer_in_step_inflow(Tracer *t) {
    int step_h = g_scene.grid_rows / 4;
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = (float)step_h
             + rand_uniform_unit() * (float)(g_scene.grid_rows - 1 - step_h);
}

/* Free jet: drop dots into the narrow inflow slot so they get swept
 * into the jet and show how it fans out. */
static inline void place_tracer_in_jet_inflow(Tracer *t) {
    int band_lo = g_scene.grid_rows * 4 / 10;
    int band_hi = g_scene.grid_rows * 6 / 10;
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = (float)band_lo
             + rand_uniform_unit() * (float)(band_hi - band_lo);
}

/* Karman street: drop dots into the inflow across the whole left wall
 * so they stream past the cylinder and trace the shed vortices. */
static inline void place_tracer_in_karman_inflow(Tracer *t) {
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = 1.0f + rand_uniform_unit() * (float)(g_scene.grid_rows - 3);
}

/* Give the dot a random countdown so the dots don't all expire on the
 * same frame and clump up. */
static inline void assign_random_tracer_lifetime(Tracer *t) {
    t->ticks_remaining = TRACER_LIFETIME_MIN
                       + rand_in_range(0, TRACER_LIFETIME_MAX - TRACER_LIFETIME_MIN);
}

/* Place one dot using whatever spot suits the current scenario, then
 * give it a fresh countdown. */
static void tracer_spawn(Tracer *t, int active_scenario_index) {
    switch (active_scenario_index) {
        case SCENARIO_LID_CAVITY:    place_tracer_lid_cavity_interior(t); break;
        case SCENARIO_BACKWARD_STEP: place_tracer_in_step_inflow     (t); break;
        case SCENARIO_FREE_JET:      place_tracer_in_jet_inflow      (t); break;
        default:                     place_tracer_in_karman_inflow   (t); break;
    }
    assign_random_tracer_lifetime(t);
}

static void tracers_init_all(int active_scenario_index) {
  for (int i = 0; i < TRACER_COUNT_MAX; i++)
    tracer_spawn(&tracer_pool[i], active_scenario_index);
}

static bool tracer_position_in_wall(float px, float py) {
  int x = (int)(px + 0.5f);
  int y = (int)(py + 0.5f);
  if (x < 0 || x >= g_scene.grid_cols)
    return false;
  if (y < 0 || y >= g_scene.grid_rows)
    return false;
  return g_scene.wall_mask[cell_index(x, y)];
}

/* Move one dot: look up the flow under it and nudge it that way.  The
 * extra multiplier exists because sim-time runs much slower than the
 * clock, so without it the dots would barely creep even in fast flow. */
static inline void advect_one_tracer(Tracer *t, float dt_seconds) {
    const float visible_motion_gain = 6.0f;
    float u_here, v_here;
    velocity_at_fractional(t->pos_x, t->pos_y, &u_here, &v_here);
    t->pos_x += u_here * dt_seconds * visible_motion_gain;
    t->pos_y += v_here * dt_seconds * visible_motion_gain;
}

/* Is this dot done?  Yes if it drifted off the edge, wandered into a
 * wall, or its countdown ran out — any one means respawn it. */
static inline bool tracer_should_respawn(const Tracer *t) {
    bool out_of_bounds =
        t->pos_x < 0.5f
     || t->pos_x > (float)(g_scene.grid_cols - 2)
     || t->pos_y < 0.5f
     || t->pos_y > (float)(g_scene.grid_rows - 2);
    bool inside_wall = tracer_position_in_wall(t->pos_x, t->pos_y);
    bool expired     = (t->ticks_remaining <= 0);
    return out_of_bounds || inside_wall || expired;
}

/* One dot's full update: move it, tick its countdown down, respawn if
 * it's done. */
static inline void update_one_tracer(Tracer *t, float dt_seconds,
                                      int active_scenario_index) {
    advect_one_tracer(t, dt_seconds);
    t->ticks_remaining--;
    if (tracer_should_respawn(t))
        tracer_spawn(t, active_scenario_index);
}

/* Move every active dot one step (nothing to do if the overlay is
 * switched off). */
static void tracers_advance(int active_scenario_index, float dt_seconds) {
    if (!tracer_overlay_enabled) return;
    for (int i = 0; i < active_tracer_count; i++)
        update_one_tracer(&tracer_pool[i], dt_seconds, active_scenario_index);
}

/* §15  presets — the four scenarios + their loaders */

/*
 * ScenarioPreset — one ready-made flow to watch.
 *
 * Each preset is a complete recipe for one classic flow: what each
 * wall does, a starting Reynolds number (how fast and swirly it runs),
 * and any obstacle in the way — a cylinder for the Karman street, a
 * step for the separation scenario, nothing for the others.  Loading a
 * preset copies its wall rules into the live setup and stamps its
 * obstacle into the wall map.
 *
 * Obstacle positions are fractions ("0.25" = a quarter of the way
 * across) rather than fixed cells, so a scenario looks the same shape
 * on any terminal size; the loader turns them into real cells.
 *
 * The lid-cavity case is the standard Ghia 1982 test; the cylinder
 * wake is von Karman 1911.
 */
typedef struct {
    const char  *display_name;            /* short label for the HUD    */
    BoundarySpec boundary_spec;           /* what each wall does         */
    int          default_reynolds_index;  /* starting Reynolds preset    */

    /* ── Optional cylinder (the Karman obstacle) ─────────────── */
    bool         has_obstacle_cylinder;
    float        cylinder_x_frac;         /* centre, fraction across     */
    float        cylinder_y_frac;         /* centre, fraction up         */
    float        cylinder_radius_frac;    /* radius, fraction of width   */

    /* ── Optional step (the backward-step obstacle) ──────────── */
    bool         has_obstacle_step;
    float        step_height_frac;        /* step height, fraction       */
} ScenarioPreset;

static const ScenarioPreset scenario_table[SCENARIO_COUNT] = {
    /* 0 — KARMAN STREET */
    {"KARMAN STREET",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_UNIFORM,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200 default            */
     true,
     0.25f,
     0.50f,
     0.06f,
     false,
     0.0f},
    /* 1 — LID-DRIVEN CAVITY */
    {"LID CAVITY   ",
     {.side_top = BC_SIDE_WALL_MOVING,
      .side_right = BC_SIDE_WALL_STATIONARY,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_WALL_STATIONARY,
      .inflow_velocity = INFLOW_VELOCITY},
     1, /* Re = 100                    */
     false,
     0,
     0,
     0,
     false,
     0},
    /* 2 — FREE JET */
    {"FREE JET     ",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_CENTER_BAND,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200                    */
     false,
     0,
     0,
     0,
     false,
     0},
    /* 3 — BACKWARD STEP */
    {"BACKWARD STEP",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_TOP_HALF,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200                    */
     false,
     0,
     0,
     0,
     true,
     0.25f},
};

static int active_scenario_index = SCENARIO_KARMAN;

/* Pick the preset for a scenario number (clamped to a real one) and
 * remember the choice so drawing and the HUD use the same one. */
static inline const ScenarioPreset *select_scenario_preset(int scenario_index) {
    if (scenario_index < 0 || scenario_index >= SCENARIO_COUNT)
        scenario_index = 0;
    active_scenario_index = scenario_index;
    return &scenario_table[scenario_index];
}

/* Take the preset's Reynolds number and turn it into the fluid's
 * stickiness (stickiness = 1 / Reynolds) — that's what controls how
 * fast spin gets smoothed away. */
static inline void apply_scenario_reynolds(const ScenarioPreset *preset) {
    g_scene.reynolds_preset_index = preset->default_reynolds_index;
    g_scene.reynolds_number       =
        reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity   = 1.0f / g_scene.reynolds_number;
}

/* Stamp the scenario's obstacle into the wall map: a cylinder for the
 * Karman street, a step for the separation flow, nothing for the rest.
 * Every later solver pass skips these cells. */
static inline void apply_scenario_obstacles(const ScenarioPreset *preset) {
    if (preset->has_obstacle_cylinder)
        obstacle_build_cylinder(preset->cylinder_x_frac,
                                preset->cylinder_y_frac,
                                preset->cylinder_radius_frac);
    if (preset->has_obstacle_step)
        obstacle_build_step    (preset->step_height_frac);
}

/* Lay down the starting state: set the walls, then read the matching
 * speeds and pick the first time step.  After this the sim is ready
 * to run. */
static inline void apply_scenario_initial_state(const ScenarioPreset *preset) {
    apply_boundary                 (&preset->boundary_spec);
    velocity_recompute_and_adapt_dt(&preset->boundary_spec);
}

/* Reset the colour-scaling values.  Start at 1, not 0, so the first
 * few frames don't divide by zero before real peaks appear. */
static inline void reset_smoothed_maxes(void) {
    g_scene.velocity_max_smoothed  = 1.0f;
    g_scene.vorticity_max_smoothed = 1.0f;
}

/* Switch to a scenario from scratch: wipe the grid, set its
 * stickiness and obstacle, lay down the starting flow, and scatter
 * fresh dots. */
static void scenario_load(int scenario_index) {
    const ScenarioPreset *preset = select_scenario_preset(scenario_index);

    grid_clear_walls();
    grid_zero_all_fields();

    apply_scenario_reynolds     (preset);
    apply_scenario_obstacles    (preset);
    apply_scenario_initial_state(preset);

    tracers_init_all(active_scenario_index);
    reset_smoothed_maxes();
}

/* §16  render_vorticity — draw the spin field */

/* How many grid rows actually fit on screen, leaving the top row for
 * the status line and the bottom row for the key hints. */
static inline int compute_render_max_y(int term_rows) {
    int draw_rows = term_rows - 2;
    if (draw_rows < 1) draw_rows = 1;
    int rows = g_scene.grid_rows;
    return (rows < draw_rows) ? rows : draw_rows;
}

/* Turn a screen row into a grid row, flipped top-to-bottom so the top
 * of the window shows the top of the tank instead of upside-down. */
static inline int screen_row_to_grid_y(int sy) {
    return g_scene.grid_rows - 1 - sy;
}

/* A cell's spin scaled to roughly -1..1 (against the running peak) so
 * the colour picker can sort it into a tier. */
static inline float normalised_omega_at(int x, int y) {
    float wn = g_scene.vorticity_omega[cell_index(x, y)]
             / g_scene.vorticity_max_smoothed;
    if (wn >  1.0f) wn =  1.0f;
    if (wn < -1.0f) wn = -1.0f;
    return wn;
}

static inline void paint_vorticity_cell(int screen_row, int screen_col,
                                         float omega_normalised) {
    VortBand band = vorticity_band_for(omega_normalised);
    attron(COLOR_PAIR(band.pair) | band.attr);
    mvaddch(screen_row, screen_col, (chtype)(unsigned char)band.glyph);
    attroff(COLOR_PAIR(band.pair) | band.attr);
}

/* Draw the spin field: red where it spins counter-clockwise, blue
 * clockwise, grey where it's barely turning. */
static void render_vorticity_view(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);
    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++)
            paint_vorticity_cell(sy + 1, x, normalised_omega_at(x, y));
    }
}

/* §17  render_streamlines — draw the flow as contour bands */

/* Find the lowest and highest ψ on the grid in one pass.  We band
 * against this range rather than fixed values, so the contours stay
 * visible no matter how fast the flow gets. */
static inline void scan_streamfunction_range(float *out_min, float *out_max) {
    int   n   = g_scene.grid_cols * g_scene.grid_rows;
    float lo  =  1e9f;
    float hi  = -1e9f;
    for (int i = 0; i < n; i++) {
        float p = g_scene.streamfunction_psi[i];
        if (p < lo) lo = p;
        if (p > hi) hi = p;
    }
    *out_min = lo;
    *out_max = hi;
}

/* Sort a ψ value into one of a fixed set of bands.  Nearby ψ values
 * land in the same band, so the screen shows stripes — the contour
 * lines you'd draw on a flow map. */
static inline int psi_to_streamline_band(float psi, float psi_min,
                                          float psi_range) {
    float frac = (psi - psi_min) / psi_range;
    int   band = (int)(frac * (float)STREAMLINE_BAND_COUNT);
    if (band < 0)                         band = 0;
    if (band >= STREAMLINE_BAND_COUNT)    band = STREAMLINE_BAND_COUNT - 1;
    return band;
}

/* Flip between two colours on odd vs even bands.  The alternating
 * shading keeps the contour lines readable even when they're crowded. */
static inline int streamline_pair_for_band(int band) {
    return (band & 1) ? PAIR_VEL_FIRST + 2 : PAIR_VEL_FIRST + 4;
}

/* How fast the flow is moving at one cell. */
static inline float speed_magnitude_at(int x, int y) {
    float u = g_scene.velocity_x[cell_index(x, y)];
    float v = g_scene.velocity_y[cell_index(x, y)];
    return sqrtf(u * u + v * v);
}

static inline void paint_streamline_cell(int screen_row, int screen_col,
                                          int band, int speed_slot) {
    int pair = streamline_pair_for_band(band);
    attron(COLOR_PAIR(pair));
    mvaddch(screen_row, screen_col,
            (chtype)(unsigned char)velocity_glyph_table[speed_slot]);
    attroff(COLOR_PAIR(pair));
}

/* Draw the flow as contour stripes.  Each cell shows two things at
 * once: the colour says which contour band it's in, and the character
 * (faint dots to solid blocks) says how fast the flow is there. */
static void render_streamlines_view(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);

    float psi_min, psi_max;
    scan_streamfunction_range(&psi_min, &psi_max);
    float psi_range = (psi_max - psi_min) > 1e-8f
                    ?  psi_max - psi_min
                    :  1e-8f;

    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            float psi  = g_scene.streamfunction_psi[cell_index(x, y)];
            int   band = psi_to_streamline_band(psi, psi_min, psi_range);
            int   slot = speed_to_velocity_slot(speed_magnitude_at(x, y),
                                                g_scene.velocity_max_smoothed);
            paint_streamline_cell(sy + 1, x, band, slot);
        }
    }
}

/* §18  render_velocity — draw flow speed, dark = slow, bright = fast */

static void render_velocity_view(int term_rows, int term_cols) {
  int cols = g_scene.grid_cols;
  int rows = g_scene.grid_rows;
  int draw_rows = term_rows - 2;
  if (draw_rows < 1)
    draw_rows = 1;
  int max_y = (rows < draw_rows) ? rows : draw_rows;

  for (int sy = 0; sy < max_y; sy++) {
    int y = rows - 1 - sy;
    if (y < 0)
      continue;
    for (int x = 0; x < cols && x < term_cols; x++) {
      if (g_scene.wall_mask[cell_index(x, y)])
        continue;
      float u = g_scene.velocity_x[cell_index(x, y)];
      float v = g_scene.velocity_y[cell_index(x, y)];
      float speed = sqrtf(u * u + v * v);
      int slot = speed_to_velocity_slot(speed, g_scene.velocity_max_smoothed);
      attr_t attr = COLOR_PAIR(PAIR_VEL_FIRST + slot);
      if (slot >= VEL_RAMP_SIZE - 2)
        attr |= A_BOLD;
      attron(attr);
      mvaddch(sy + 1, x, (chtype)(unsigned char)velocity_glyph_table[slot]);
      attroff(attr);
    }
  }
}

/* §19  render_tracers — draw the floating dots on top */

/* Work out which screen cell a dot lands on (rounded to a cell, with
 * the top-to-bottom flip).  Returns false if it's off the grid, off
 * the visible area, or inside a wall — those dots aren't drawn. */
static inline bool tracer_to_visible_cell(const Tracer *t,
                                           int term_rows, int term_cols,
                                           int *out_gx, int *out_gy,
                                           int *out_screen_row) {
    int gx = (int)(t->pos_x + 0.5f);
    int gy = (int)(t->pos_y + 0.5f);
    if (gx < 0 || gx >= g_scene.grid_cols) return false;
    if (gy < 0 || gy >= g_scene.grid_rows) return false;
    if (g_scene.wall_mask[cell_index(gx, gy)]) return false;

    int sy        = g_scene.grid_rows - 1 - gy;
    int draw_rows = term_rows - 2;
    if (sy < 0 || sy >= draw_rows) return false;
    if (gx >= term_cols)           return false;

    *out_gx         = gx;
    *out_gy         = gy;
    *out_screen_row = sy + 1;  /* +1 because row 0 is the status row */
    return true;
}

/* Pick a dot's character from how fast it's moving, in four steps, so
 * faster dots look chunkier. */
static inline int tracer_glyph_slot_for_speed(float speed_normalised) {
    int slot = (int)(speed_normalised * 4.0f);
    if (slot < 0) slot = 0;
    if (slot >= 4) slot = 3;
    return slot;
}

/* Colour fast dots hot (red), slow ones plain (white), so the busy
 * parts of the flow stand out. */
static inline int tracer_pair_for_speed(float speed_normalised) {
    return (speed_normalised > 0.6f) ? PAIR_TRACER_FAST : PAIR_TRACER;
}

/* Draw one dot, its character and colour both set by the local speed,
 * bold so it stands out over whatever field is underneath. */
static inline void paint_one_tracer_at(int screen_row, int gx, int gy) {
    float speed_norm = speed_magnitude_at(gx, gy) / g_scene.velocity_max_smoothed;
    int   slot       = tracer_glyph_slot_for_speed(speed_norm);
    int   pair       = tracer_pair_for_speed     (speed_norm);
    attr_t attr      = COLOR_PAIR(pair) | A_BOLD;
    attron(attr);
    mvaddch(screen_row, gx, (chtype)(unsigned char)tracer_glyph_table[slot]);
    attroff(attr);
}

/* Draw all the dots on top of the current field view (nothing if the
 * overlay is off). */
static void render_tracer_overlay(int term_rows, int term_cols) {
    if (!tracer_overlay_enabled) return;
    for (int i = 0; i < active_tracer_count; i++) {
        const Tracer *t = &tracer_pool[i];
        int gx, gy, screen_row;
        if (!tracer_to_visible_cell(t, term_rows, term_cols,
                                     &gx, &gy, &screen_row))
            continue;
        paint_one_tracer_at(screen_row, gx, gy);
    }
}

/* §20  render_obstacles — draw the solid walls on top */

/* Draw one wall cell as a bold '#'.  Walls are the one thing always
 * drawn at full brightness, so the shape of the scene is clear no
 * matter which field is showing underneath. */
static inline void paint_obstacle_cell(int screen_row, int screen_col) {
    attron(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
    mvaddch(screen_row, screen_col, '#');
    attroff(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
}

static inline void paint_all_obstacle_cells(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);
    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (!g_scene.wall_mask[cell_index(x, y)]) continue;
            paint_obstacle_cell(sy + 1, x);
        }
    }
}

/* If the top wall is a sliding lid, draw a '=' bar across the top to
 * mark it.  Otherwise the lid is invisible — you'd only see the swirl
 * it stirs up, not the lid itself. */
static inline void paint_moving_lid_indicator_if_active(int term_cols) {
    const ScenarioPreset *preset = &scenario_table[active_scenario_index];
    if (preset->boundary_spec.side_top != BC_SIDE_WALL_MOVING) return;
    int cols = g_scene.grid_cols;
    attron(COLOR_PAIR(PAIR_LID) | A_BOLD);
    for (int x = 0; x < cols && x < term_cols; x++)
        mvaddch(1, x, '=');
    attroff(COLOR_PAIR(PAIR_LID) | A_BOLD);
}

/* Draw the walls and, if there is one, the sliding-lid bar. */
static void render_obstacle_overlay(int term_rows, int term_cols) {
    paint_all_obstacle_cells              (term_rows, term_cols);
    paint_moving_lid_indicator_if_active  (term_cols);
}

/* §21  hud — status line up top, key hints along the bottom */

static void hud_paint_status(int term_cols, double measured_fps,
                             int active_view_mode, bool simulation_paused) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " VSF-NS  %s  Re=%6.0f  view:%s  tracers:%s  "
           "%4.0f fps  %s ",
           scenario_table[active_scenario_index].display_name,
           (double)g_scene.reynolds_number, view_name_table[active_view_mode],
           tracer_overlay_enabled ? "ON " : "off", measured_fps,
           simulation_paused ? "PAUSED " : "running");
  int slen = (int)strlen(buf);
  int sx = term_cols - slen;
  if (sx < 0)
    sx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0,
           " q:quit  spc:pause  v:vorticity  s:streamlines  w:velocity  "
           "x:tracers  p/P:scene  +/-:Re  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §22  scene — per-frame state + tick wrapper */

static int active_view_mode = VIEW_VORTICITY;
static bool simulation_paused = false;

static void scene_init(int term_rows, int term_cols) {
  g_scene.grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  g_scene.grid_rows = (term_rows - 2 < GRID_ROWS_MAX) ? term_rows - 2 : GRID_ROWS_MAX;
  if (g_scene.grid_cols < 8)
    g_scene.grid_cols = 8;
  if (g_scene.grid_rows < 8)
    g_scene.grid_rows = 8;
  g_scene.cell_size_x = 1.0f / (float)(g_scene.grid_cols - 1);
  g_scene.cell_size_y = 1.0f / (float)(g_scene.grid_rows - 1);
  scenario_load(SCENARIO_KARMAN);
}

static void scene_resize(int term_rows, int term_cols) {
  int saved = active_scenario_index;
  g_scene.grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  g_scene.grid_rows = (term_rows - 2 < GRID_ROWS_MAX) ? term_rows - 2 : GRID_ROWS_MAX;
  if (g_scene.grid_cols < 8)
    g_scene.grid_cols = 8;
  if (g_scene.grid_rows < 8)
    g_scene.grid_rows = 8;
  g_scene.cell_size_x = 1.0f / (float)(g_scene.grid_cols - 1);
  g_scene.cell_size_y = 1.0f / (float)(g_scene.grid_rows - 1);
  scenario_load(saved);
}

static void scene_tick(void) {
  if (simulation_paused)
    return;
  const BoundarySpec *spec =
      &scenario_table[active_scenario_index].boundary_spec;
  for (int s = 0; s < SUBSTEPS_PER_FRAME; s++)
    ns_step(spec);
  tracers_advance(active_scenario_index, g_scene.current_dt * SUBSTEPS_PER_FRAME);
}

/* §23  screen — ncurses setup, teardown, and frame output */

/*
 * Screen — just the terminal's size.  ncurses holds the actual screen
 * buffers; all we need to remember is how many rows and columns we
 * have, for placing the HUD and clipping the field.  Each frame we
 * wipe, draw the field, drop the dots on top, add the HUD, and flush
 * once — ncurses writes only what changed, so there's no flicker.
 */
typedef struct {
    int rows;   /* terminal height in cells                        */
    int cols;   /* terminal width  in cells                        */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, double measured_fps) {
  erase();

  switch (active_view_mode) {
  case VIEW_VORTICITY:
    render_vorticity_view(screen->rows, screen->cols);
    break;
  case VIEW_STREAMLINES:
    render_streamlines_view(screen->rows, screen->cols);
    break;
  case VIEW_VELOCITY:
    render_velocity_view(screen->rows, screen->cols);
    break;
  }

  render_obstacle_overlay(screen->rows, screen->cols);
  render_tracer_overlay(screen->rows, screen->cols);
  hud_paint_status(screen->cols, measured_fps, active_view_mode,
                   simulation_paused);
  hud_paint_hint(screen->rows);

  wnoutrefresh(stdscr);
  doupdate();
}

/* §24  app — main loop, signals, keyboard */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    simulation_paused = !simulation_paused;
    break;

  case 'v':
  case 'V':
    active_view_mode = VIEW_VORTICITY;
    break;
  case 's':
  case 'S':
    active_view_mode = VIEW_STREAMLINES;
    break;
  case 'w':
  case 'W':
    active_view_mode = VIEW_VELOCITY;
    break;

  case 'x':
  case 'X':
    tracer_overlay_enabled = !tracer_overlay_enabled;
    break;

  case 'p':
    scenario_load((active_scenario_index + 1) % SCENARIO_COUNT);
    break;
  case 'P':
    scenario_load((active_scenario_index + SCENARIO_COUNT - 1) %
                  SCENARIO_COUNT);
    break;

  case '+':
  case '=':
    g_scene.reynolds_preset_index = (g_scene.reynolds_preset_index + 1) % REYNOLDS_PRESET_COUNT;
    g_scene.reynolds_number = reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity = 1.0f / g_scene.reynolds_number;
    break;
  case '-':
    g_scene.reynolds_preset_index =
        (g_scene.reynolds_preset_index + REYNOLDS_PRESET_COUNT - 1) %
        REYNOLDS_PRESET_COUNT;
    g_scene.reynolds_number = reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity = 1.0f / g_scene.reynolds_number;
    break;

  case 'r':
  case 'R':
    scenario_load(active_scenario_index);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned)time(NULL));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  Screen screen;
  screen_init(&screen);
  scene_init(screen.rows, screen.cols);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

  while (!g_should_quit) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch)) {
        g_should_quit = 1;
        break;
      }
    }

    /* ── resize ── */
    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_resize(screen.rows, screen.cols);
      prev_frame_ns = clock_now_ns();
    }

    /* ── dt + fps window ── */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── physics + render ── */
    scene_tick();
    screen_present_frame(&screen, measured_fps);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
