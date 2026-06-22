/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * navier_stokes.c — Jos Stam's "Stable Fluids": a grid-based fluid
 * sim you watch by injecting coloured dye.  Two spinning emitters
 * throw dye into the flow; arrow keys, dye drops, and a viscosity
 * knob let you poke at it.  The trick of the method is that it never
 * blows up no matter how big a time step you give it.
 *
 * Reference: Stam, "Stable Fluids", SIGGRAPH 1999 (the paper this
 * whole file implements) and his 2003 "Real-Time Fluid Dynamics for
 * Games" notes.  Sister files: fluid/fluid_sph.c and
 * fluid/lattice_gas.c take the opposite, particle-based approach.
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

/* ── §1  config — every tunable in one place ── */

/* Grid is N×N "real" cells plus a 1-cell border of ghost cells all
 * the way around (so neighbour math at the edge never reads off the
 * array). */
#define GRID_SIDE_INNER 80 /* "N" — the real cells */
#define GRID_SIDE_TOTAL (GRID_SIDE_INNER + 2)
#define GRID_TOTAL_CELLS (GRID_SIDE_TOTAL * GRID_SIDE_TOTAL)

#define DT_DEFAULT 0.05f           /* time step per tick */
#define DIFFUSION_DYE 0.0001f      /* how fast dye spreads on its own */
#define VISCOSITY_INITIAL 0.00001f /* how "thick" the fluid is (ν) */
#define VISCOSITY_MIN 1e-7f
#define VISCOSITY_MAX 0.1f
#define VISCOSITY_FACTOR 2.0f /* how much '+' / '-' change viscosity */

/* How many smoothing passes the linear solver runs.  16 is enough to
 * look right at this grid size; bigger grids need more. */
#define GAUSS_SEIDEL_ITERATIONS 16

/* Overall strength of injected pushes and dye, set in add_source_at(). */
#define INJECT_FORCE_SCALE 50.0f
#define INJECT_DYE_SCALE 50.0f

/* The two auto-emitters: how fast they spin and how hard they push. */
#define EMITTER_SWIRL_INCREMENT 0.04f /* radians per tick */
#define EMITTER_FORCE_AMPLITUDE 1.5f
#define EMITTER_DYE_AMPLITUDE 3.0f

/* Run this many ticks before the first frame so it isn't blank. */
#define PREWARM_TICK_COUNT 80

/* Which wall rule to use when filling the ghost border (see §7). */
enum {
  BOUNDARY_SCALAR = 0,     /* dye/pressure: copy the edge value outward */
  BOUNDARY_VELOCITY_X = 1, /* horizontal speed: flip sign at L/R walls */
  BOUNDARY_VELOCITY_Y = 2, /* vertical speed: flip sign at top/bottom */
};

/* Which colour the dye is painted in (1/2/3 keys cycle these). */
enum {
  DYE_CHANNEL_BLUE = 0,
  DYE_CHANNEL_GREEN = 1,
  DYE_CHANNEL_RED = 2,
  DYE_CHANNEL_COUNT,
};

#define DYE_SHADE_COUNT 4 /* brightness steps per colour */

/* ncurses colour-pair slots: the dye pairs come first, then the HUD. */
enum {
  PAIR_DYE_FIRST = 1,
  PAIR_HUD = PAIR_DYE_FIRST + DYE_CHANNEL_COUNT * DYE_SHADE_COUNT,
  PAIR_HINT,
};

/* Rows kept clear for the HUD: one at the top, two at the bottom. */
#define HUD_RESERVED_ROWS_TOP 1
#define HUD_RESERVED_ROWS_BOTTOM 2

#define RENDER_FPS 30
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS)

/* Density thresholds that pick which glyph a cell gets (after the
 * density has been scaled to roughly 0..1). */
#define DENSITY_GLYPH_BLANK 0.02f
#define DENSITY_GLYPH_LOW 0.20f
#define DENSITY_GLYPH_MID 0.50f
#define DENSITY_GLYPH_HIGH 0.80f

/* Keep back-traced sample points this far inside the grid edge. */
#define INTERP_CLAMP_MARGIN 0.5f

/* Smoothing weights for the dye brightness scale (see §16): mostly
 * keep the old value, nudge it toward the latest frame's peak. */
#define DYE_MAX_EMA_OLD 0.95f
#define DYE_MAX_EMA_NEW 0.05f
#define DYE_MAX_FLOOR 0.001f
#define DYE_MAX_INITIAL 1.0f

/* Turn a 2-D cell (i, j) into its slot in the flat field arrays. */
#define cell_index(i, j) ((i) + GRID_SIDE_TOTAL * (j))

/* ── §2  clock — wall-clock timer + sleep ── */

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

/* ── §3  rng — pick a random interior cell ── */
/* Only the dye-drop key uses randomness; the physics is deterministic. */
static int rand_inner_cell(void) { return 1 + rand() % GRID_SIDE_INNER; }

/* ── §4  themes — the three dye colour ramps ── */

/*
 * DyePalette — one dye colour (blue, green, or red) as a four-step
 * brightness ramp.  Denser dye in a cell picks a later, brighter step.
 *
 * We keep one of these per colour and let the 1/2/3 keys switch which
 * one paints the field.  A cell shows a single colour at a time, so
 * three colours times four steps is just twelve ncurses colour pairs.
 *
 * Two parallel ramps so the look survives a weak terminal: colour_256
 * is used on terminals that have the full 256-colour set; colour_8 is
 * the plain-8-colour fallback (it loses the fine gradient but still
 * gets brighter step by step).  We choose between them once at startup.
 *
 * Even the dimmest step is kept reasonably bright (xterm index >= 30)
 * so faint dye doesn't vanish on a black background — see the project's
 * "Theme Palette Brightness" rule.
 *
 * Members
 *   colour_256[]   four xterm-256 colour indices, dim -> bright.
 *   colour_8[]     same four steps for plain-8-colour terminals.
 *   name           label shown in the HUD ("blue" / "green" / "red").
 *
 * Invariants
 *   The four steps go from dimmest to brightest, in order.
 *   name is never NULL.
 */
typedef struct {
    short       colour_256[DYE_SHADE_COUNT];
    short       colour_8  [DYE_SHADE_COUNT];
    const char *name;
} DyePalette;

static const DyePalette dye_palette_table[DYE_CHANNEL_COUNT] = {
    /* blue: mid blue rising to bright cyan */
    {{33, 39, 51, 87},
     {COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
     "blue"},

    /* green: green rising to bright lime */
    {{34, 40, 82, 118},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE},
     "green"},

    /* red: deep red rising to orange */
    {{124, 160, 196, 208},
     {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW},
     "red"},
};

/* ── §5  colors — register the ncurses colour pairs ── */

static bool terminal_has_256_colours = false;

/* Which colour-pair slot holds a given colour + brightness step. */
static int dye_pair_id(int channel, int shade) {
  return PAIR_DYE_FIRST + channel * DYE_SHADE_COUNT + shade;
}

/* Bright yellow + cyan for the HUD (the project's standard HUD colours). */
enum {
    HUD_YELLOW_256 = 226,
    HUD_CYAN_256   = 51,
};

static inline void register_one_dye_channel(int channel,
                                             const DyePalette *pal,
                                             bool have_256_colors) {
    for (int sh = 0; sh < DYE_SHADE_COUNT; sh++) {
        short fg = have_256_colors ? pal->colour_256[sh] : pal->colour_8[sh];
        init_pair((short)dye_pair_id(channel, sh), fg, -1);
    }
}

static inline void apply_dye_palettes(bool have_256_colors) {
    for (int ch = 0; ch < DYE_CHANNEL_COUNT; ch++)
        register_one_dye_channel(ch, &dye_palette_table[ch], have_256_colors);
}

static inline void apply_chrome_palette(bool have_256_colors) {
    if (have_256_colors) {
        init_pair(PAIR_HUD,  HUD_YELLOW_256, -1);
        init_pair(PAIR_HINT, HUD_CYAN_256,   -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void colors_init(void) {
    start_color();
    use_default_colors();
    terminal_has_256_colours = (COLORS >= 256);

    apply_dye_palettes(terminal_has_256_colours);
    apply_chrome_palette(terminal_has_256_colours);
}

/* ── §6  scene — all the live state in one place ── */

/*
 * Fluid — the grid arrays the solver actually works on.  This is the
 * real state of the simulation; everything else just looks at it.
 *
 * Velocity is two arrays (sideways and up/down speed at each cell);
 * dye is one array (how much colour is in each cell).  Each of those
 * gets a matching "_prev" scratch array, because every solver pass
 * reads one buffer while filling the other, then they swap roles.
 *
 * Heads-up on the name: "_prev" does NOT mean "last tick".  It just
 * means "the other buffer in the pair the solver is bouncing between".
 * During the project step the two velocity scratch buffers get reused
 * to hold pressure and divergence instead — see §11.
 *
 * Members
 *   velocity_x          sideways speed at each cell (in cells per step).
 *                       Access with cell_index(i, j); 1..N is real, 0
 *                       and N+1 are the ghost border.
 *   velocity_y          up/down speed.  Same layout.
 *   velocity_x_prev     scratch for velocity_x; doubles as the pressure
 *                       field during project.
 *   velocity_y_prev     scratch for velocity_y; doubles as the
 *                       divergence field during project.
 *   dye_density         the colour the renderer paints.
 *   dye_density_prev    scratch for the dye passes.
 *   viscosity_kinematic how thick the fluid is.  The +/- keys change it;
 *                       it stays inside [VISCOSITY_MIN, VISCOSITY_MAX].
 *
 * Invariants
 *   Every array carries its 1-cell ghost border; boundary_apply refills
 *   that border after any pass that changes the field.
 *   viscosity_kinematic stays positive.
 *   dye_density never goes negative.
 *
 * Reference: Stam 1999/2003 — this is exactly his buffer layout.
 */
typedef struct {
    float velocity_x       [GRID_TOTAL_CELLS];
    float velocity_y       [GRID_TOTAL_CELLS];
    float velocity_x_prev  [GRID_TOTAL_CELLS];
    float velocity_y_prev  [GRID_TOTAL_CELLS];
    float dye_density      [GRID_TOTAL_CELLS];
    float dye_density_prev [GRID_TOTAL_CELLS];
    float viscosity_kinematic;
} Fluid;

/*
 * Emitter — state for the two auto-emitters that keep the flow alive.
 *
 * They sit at the one-third and two-third marks and inject dye plus a
 * push that slowly rotates.  One angle drives both so they stay in
 * step (and spin opposite ways).  It's just one number today, but the
 * named struct leaves room to add other emitter modes later.
 *
 * Members
 *   swirl_phase   the current spin angle, in radians, bumped a little
 *                 each tick.  Never wrapped — sin/cos don't care how
 *                 big it grows.
 */
typedef struct {
    float swirl_phase;
} Emitter;

/*
 * SimControls — the run/pause switch.
 *
 * The key handler flips this; the main loop checks it and skips the
 * physics (but keeps drawing) when paused, so you can study a frozen
 * frame.
 *
 * Members
 *   paused   true while paused; the HUD says "PAUSED".
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * DyeRenderer — look-only state.  The physics never touches this; it's
 * here so changing how the dye looks can't disturb the simulation.
 *
 * It holds which colour is showing and a slowly-changing "brightest
 * dye right now" value used to scale the picture.  We smooth that
 * value instead of using each frame's raw peak: the peak jumps around
 * when an emitter pulses or a blob lands, and scaling by it would make
 * the whole screen flash.  Easing it over ~10 frames keeps things calm
 * (see §16).
 *
 * Members
 *   active_channel     which colour is painted (blue / green / red).
 *   dye_max_smoothed   the eased "brightest dye" value the picture is
 *                      scaled against; kept at or above DYE_MAX_FLOOR
 *                      so we never divide by something tiny.
 *
 * Invariants
 *   active_channel is one of the three DYE_CHANNEL_* values.
 *   dye_max_smoothed >= DYE_MAX_FLOOR.
 */
typedef struct {
    int   active_channel;
    float dye_max_smoothed;
} DyeRenderer;

/*
 * Scene — the whole live state of one run, grouped by job:
 *
 *     Scene
 *       ├── fluid    the grid arrays + viscosity (the physics)
 *       ├── emitter  the auto-emitter spin angle
 *       ├── sim      the pause switch
 *       └── render   colour + brightness scale (look only)
 *
 * Grouping it this way makes it obvious from the field path whether a
 * line is touching the physics or just the picture.  There's one
 * file-scope g_scene rather than a pointer passed everywhere — the
 * inner loops touch all the arrays per cell, so threading a pointer
 * through would add noise and cost for no gain, and it keeps us to the
 * project rule of no allocation after startup.
 *
 * The one rule to keep: the physics passes may only touch fluid and
 * emitter; the drawing code may read fluid but only write render.
 * That separation is what lets you pause or recolour without
 * disturbing the flow.
 *
 * Members
 *   fluid     the grid arrays + viscosity (see Fluid above).
 *   emitter   the auto-emitter spin angle (see Emitter above).
 *   sim       the pause switch.
 *   render    colour + brightness scale; look only.
 */
typedef struct {
    Fluid       fluid;
    Emitter     emitter;
    SimControls sim;
    DyeRenderer render;
} Scene;

static Scene g_scene = {
    .fluid   = { .viscosity_kinematic = VISCOSITY_INITIAL },
    .emitter = { .swirl_phase         = 0.0f },
    .sim     = { .paused              = false },
    .render  = { .active_channel      = DYE_CHANNEL_BLUE,
                 .dye_max_smoothed    = DYE_MAX_INITIAL },
};

/* ── §7  boundary_apply — fill the ghost border ── */
/*
 * Fill the one-cell border around the grid so the math at the edges
 * has something sensible to read.  Call this after any step that
 * changes a field.  The border is filled to fake the wall behaviour we
 * want: dye and pressure just mirror the edge (nothing leaks out);
 * velocity flips sign at the wall it runs into so the fluid sticks to
 * the wall instead of flowing through it.  Corners average their two
 * neighbours.
 */
/* For one wall: copy the edge value out (mirror) for things that
 * shouldn't leak, or flip its sign for the velocity that runs straight
 * into this wall so the speed at the wall works out to zero. */
static inline float ghost_value_for_wall(float inner, bool is_normal_velocity) {
    return is_normal_velocity ? -inner : inner;
}

/* Fill the left and right border columns from the cells just inside. */
static inline void fill_left_right_wall_ghosts(int boundary_kind,
                                                float *field, int N) {
    bool is_normal = (boundary_kind == BOUNDARY_VELOCITY_X);
    for (int j = 1; j <= N; j++) {
        float left_inner  = field[cell_index(1,     j)];
        float right_inner = field[cell_index(N,     j)];
        field[cell_index(0,     j)] = ghost_value_for_wall(left_inner,  is_normal);
        field[cell_index(N + 1, j)] = ghost_value_for_wall(right_inner, is_normal);
    }
}

/* Fill the top and bottom border rows from the cells just inside. */
static inline void fill_top_bottom_wall_ghosts(int boundary_kind,
                                                float *field, int N) {
    bool is_normal = (boundary_kind == BOUNDARY_VELOCITY_Y);
    for (int i = 1; i <= N; i++) {
        float top_inner    = field[cell_index(i, 1    )];
        float bottom_inner = field[cell_index(i, N    )];
        field[cell_index(i, 0    )] = ghost_value_for_wall(top_inner,    is_normal);
        field[cell_index(i, N + 1)] = ghost_value_for_wall(bottom_inner, is_normal);
    }
}

/* Each corner sits between two filled border cells; give it their
 * average so it doesn't jump. */
static inline void fill_corner_ghosts_by_averaging(float *field, int N) {
    field[cell_index(0,     0    )] = 0.5f * (field[cell_index(1,     0    )]
                                           +  field[cell_index(0,     1    )]);
    field[cell_index(0,     N + 1)] = 0.5f * (field[cell_index(1,     N + 1)]
                                           +  field[cell_index(0,     N    )]);
    field[cell_index(N + 1, 0    )] = 0.5f * (field[cell_index(N,     0    )]
                                           +  field[cell_index(N + 1, 1    )]);
    field[cell_index(N + 1, N + 1)] = 0.5f * (field[cell_index(N,     N + 1)]
                                           +  field[cell_index(N + 1, N    )]);
}

static void boundary_apply(int boundary_kind, float *field) {
    int N = GRID_SIDE_INNER;
    fill_left_right_wall_ghosts(boundary_kind, field, N);
    fill_top_bottom_wall_ghosts(boundary_kind, field, N);
    fill_corner_ghosts_by_averaging(field, N);
}

/* ── §8  gauss_seidel — the iterative solver everything else leans on ── */
/*
 * Both diffusion and the pressure step boil down to the same job: each
 * cell's answer should equal some target plus a share of its four
 * neighbours' answers.  But the neighbours are unknowns too, so we
 * can't solve it in one shot.  Instead we sweep the grid over and over,
 * each pass using the latest neighbour values, and after enough passes
 * it settles on the answer.  The two callers just pass different a and
 * c constants.  We refill the border after each sweep so edge cells
 * keep reading valid neighbours.
 */
static void gauss_seidel_solve(int boundary_kind, float *x, const float *b,
                               float a, float c) {
  float inv_c = 1.0f / c;
  int N = GRID_SIDE_INNER;
  for (int sweep = 0; sweep < GAUSS_SEIDEL_ITERATIONS; sweep++) {
    for (int j = 1; j <= N; j++) {
      for (int i = 1; i <= N; i++) {
        float neighbour_sum = x[cell_index(i - 1, j)] +
                              x[cell_index(i + 1, j)] +
                              x[cell_index(i, j - 1)] + x[cell_index(i, j + 1)];
        x[cell_index(i, j)] = (b[cell_index(i, j)] + a * neighbour_sum) * inv_c;
      }
    }
    boundary_apply(boundary_kind, x);
  }
}

/* ── §9  diffuse — let a field smear into its neighbours ── */
/*
 * Spreading: each cell pulls a bit toward the average of its four
 * neighbours, by an amount set by the diffusion coefficient and the
 * time step.  We solve it with the §8 solver.
 *
 * The memcpy first matters: the solver improves whatever's already in
 * the output buffer, and that buffer may hold leftover junk from the
 * project step, so we seed it with the old field to start from a sane
 * guess.
 */
static void diffuse(int boundary_kind, float *field_new, const float *field_old,
                    float diffusion_coefficient, float dt) {
  float a =
      dt * diffusion_coefficient * (float)(GRID_SIDE_INNER * GRID_SIDE_INNER);

  memcpy(field_new, field_old, sizeof(float) * GRID_TOTAL_CELLS);
  gauss_seidel_solve(boundary_kind, field_new, field_old, a, 1.0f + 4.0f * a);
}

/* ── §10  advect — move the fluid along the flow ── */

/*
 * DeparturePoint — a spot the fluid came from, in grid coordinates.
 *
 * To move a field along the flow, we ask each cell "where was this
 * fluid one step ago?" and copy whatever was there.  We find that spot
 * by stepping backward along the cell's own velocity.  Looking
 * backward and copying (never inventing) values is exactly why this
 * method can't blow up.
 *
 * The spot usually lands between cells, so x and y are floats and the
 * caller blends the four surrounding cells.  It's a two-value answer,
 * so a tiny struct keeps the call site readable.
 *
 * Members
 *   x, y   the lookup spot in grid coordinates (same units as the
 *          cell indices).
 *
 * Invariants
 *   x, y are kept just inside the grid so the four-cell blend never
 *   reads past the edge.
 *
 * Reference: Stam 1999 §3 (the backward-trace step).
 */
typedef struct { float x; float y; } DeparturePoint;

/* Step backward from cell (i, j) along its own velocity to find where
 * its fluid came from (dt_cells already folds in the grid scaling). */
static inline DeparturePoint trace_velocity_backward(int i, int j,
                                                    const float *vx,
                                                    const float *vy,
                                                    float dt_cells) {
    DeparturePoint dp;
    dp.x = (float)i - dt_cells * vx[cell_index(i, j)];
    dp.y = (float)j - dt_cells * vy[cell_index(i, j)];
    return dp;
}

/* Keep the lookup spot just inside the grid.  Fast-moving fluid can
 * trace back past the edge, and without this the blend below would read
 * off the end of the array and crash. */
static inline void clamp_departure_to_interp_bounds(DeparturePoint *dp, int N) {
    float lo = INTERP_CLAMP_MARGIN;
    float hi = (float)N + INTERP_CLAMP_MARGIN;
    if (dp->x < lo) dp->x = lo;
    if (dp->x > hi) dp->x = hi;
    if (dp->y < lo) dp->y = lo;
    if (dp->y > hi) dp->y = hi;
}

/* Read the field at a between-cells spot by blending the four cells
 * around it, weighted by how close (x, y) is to each.  Because the
 * result is a weighted average, it always sits between those four
 * values — it can shrink but never grow, which is what keeps the whole
 * method stable.  [3] Bridson §3. */
static inline float bilinear_sample(const float *field, float x, float y) {
    int   i0 = (int)x;
    int   j0 = (int)y;
    int   i1 = i0 + 1;
    int   j1 = j0 + 1;
    float s1 = x - (float)i0,  s0 = 1.0f - s1;
    float t1 = y - (float)j0,  t0 = 1.0f - t1;
    return s0 * (t0 * field[cell_index(i0, j0)]
              +  t1 * field[cell_index(i0, j1)])
         + s1 * (t0 * field[cell_index(i1, j0)]
              +  t1 * field[cell_index(i1, j1)]);
}

/* Move a field along the flow: for each cell, trace back to where its
 * fluid came from and copy the old value there.  [1] Stam 1999 §3. */
static void advect(int boundary_kind, float *new_field, const float *old_field,
                   const float *vx, const float *vy, float dt) {
    int   N        = GRID_SIDE_INNER;
    float dt_cells = dt * (float)N;

    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            DeparturePoint dp = trace_velocity_backward(i, j, vx, vy, dt_cells);
            clamp_departure_to_interp_bounds(&dp, N);
            new_field[cell_index(i, j)] = bilinear_sample(old_field, dp.x, dp.y);
        }
    }

    boundary_apply(boundary_kind, new_field);
}

/* ── §11  project — keep the fluid from squashing or stretching ── */
/*
 * Real fluid doesn't pile up or leave gaps: as much flows into each
 * cell as flows out.  Diffusion and advection break that, so this step
 * fixes it in three moves.  First it measures how much each cell is a
 * source or a sink (the "divergence").  Then it solves for a pressure
 * field whose push exactly cancels that.  Then it subtracts that
 * pressure's push from the velocity.  After it, every cell balances.
 * This is the step that makes the whole thing look like fluid.
 *
 * The two scratch arrays passed in are renamed pressure_correction and
 * divergence_field here just so the code reads clearly.
 */
/* Measure how much each cell is a source or a sink, by comparing the
 * inflow and outflow across it.  The negative scale folds in a sign so
 * the solver that uses this result doesn't need a separate flip.
 * [1] Stam 1999 §3.5, [3] Bridson §4.4. */
static inline void compute_divergence_field(const float *vx, const float *vy,
                                            float *divergence_field) {
    int   N = GRID_SIDE_INNER;
    float h = 1.0f / (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            divergence_field[cell_index(i, j)] = -0.5f * h *
                (vx[cell_index(i + 1, j)] - vx[cell_index(i - 1, j)] +
                 vy[cell_index(i, j + 1)] - vy[cell_index(i, j - 1)]);
        }
    }
}

/* Clear the pressure field so the solver below starts from a clean
 * zero guess. */
static inline void zero_pressure_field(float *pressure_correction) {
    int N = GRID_SIDE_INNER;
    for (int j = 1; j <= N; j++)
        for (int i = 1; i <= N; i++)
            pressure_correction[cell_index(i, j)] = 0.0f;
}

/* Find the pressure field whose push will balance the measured
 * sources and sinks.  It's the same §8 solver with a=1, c=4: each
 * cell's pressure settles to its target plus the average of its
 * neighbours' pressures.  [5] Saad §4.1. */
static inline void solve_pressure_poisson(float *pressure_correction,
                                          const float *divergence_field) {
    gauss_seidel_solve(BOUNDARY_SCALAR, pressure_correction, divergence_field,
                       1.0f, 4.0f);
}

/* Push the velocity downhill along the pressure: subtract how steeply
 * pressure changes from one side of each cell to the other.  This is
 * the move that actually cancels the sources and sinks.  [6] Batchelor
 * §2.7. */
static inline void subtract_pressure_gradient(float *vx, float *vy,
                                              const float *pressure_correction) {
    int   N        = GRID_SIDE_INNER;
    float n_factor = 0.5f * (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            vx[cell_index(i, j)] -= n_factor *
                (pressure_correction[cell_index(i + 1, j)] -
                 pressure_correction[cell_index(i - 1, j)]);
            vy[cell_index(i, j)] -= n_factor *
                (pressure_correction[cell_index(i, j + 1)] -
                 pressure_correction[cell_index(i, j - 1)]);
        }
    }
}

static void project(float *vx, float *vy, float *pressure_correction,
                    float *divergence_field) {
    /* measure the sources and sinks, clear the pressure */
    compute_divergence_field(vx, vy, divergence_field);
    zero_pressure_field    (pressure_correction);
    boundary_apply(BOUNDARY_SCALAR, divergence_field);
    boundary_apply(BOUNDARY_SCALAR, pressure_correction);

    /* solve for the balancing pressure */
    solve_pressure_poisson(pressure_correction, divergence_field);

    /* push the velocity to cancel them, then refill the velocity border */
    subtract_pressure_gradient(vx, vy, pressure_correction);
    boundary_apply(BOUNDARY_VELOCITY_X, vx);
    boundary_apply(BOUNDARY_VELOCITY_Y, vy);
}

/* ── §12  fluid_step — one full tick of physics ── */
/*
 * One tick is: smear the velocity (diffuse), rebalance it (project),
 * carry it along the flow (advect), rebalance again, then do the dye.
 * The dye only needs smear + carry — it rides the flow but doesn't
 * push back, so it skips the rebalancing.  The buffer hand-offs look
 * fiddly but they're straight out of Stam's reference code.
 */
/* Velocity, first half: smear it, then rebalance it.  Ends with the
 * cleaned-up field back in velocity_x / velocity_y. */
static inline void step_velocity_diffuse_and_project(Fluid *f, float dt) {
  diffuse(BOUNDARY_VELOCITY_X, f->velocity_x_prev, f->velocity_x,
          f->viscosity_kinematic, dt);
  diffuse(BOUNDARY_VELOCITY_Y, f->velocity_y_prev, f->velocity_y,
          f->viscosity_kinematic, dt);
  project(f->velocity_x_prev, f->velocity_y_prev,
          f->velocity_x,      f->velocity_y);
}

/* Velocity, second half: carry it along itself, then rebalance again
 * because that carrying step throws the balance off a little. */
static inline void step_velocity_advect_and_project(Fluid *f, float dt) {
  advect(BOUNDARY_VELOCITY_X, f->velocity_x, f->velocity_x_prev,
         f->velocity_x_prev, f->velocity_y_prev, dt);
  advect(BOUNDARY_VELOCITY_Y, f->velocity_y, f->velocity_y_prev,
         f->velocity_x_prev, f->velocity_y_prev, dt);
  project(f->velocity_x,      f->velocity_y,
          f->velocity_x_prev, f->velocity_y_prev);
}

/* Dye: smear it a touch, then carry it along the finished velocity.
 * No rebalancing — dye just rides the flow. */
static inline void step_dye_diffuse_and_advect(Fluid *f, float dt) {
  diffuse(BOUNDARY_SCALAR, f->dye_density_prev, f->dye_density,
          DIFFUSION_DYE, dt);
  advect(BOUNDARY_SCALAR, f->dye_density, f->dye_density_prev,
         f->velocity_x, f->velocity_y, dt);
}

static void fluid_step(float dt) {
  Fluid *f = &g_scene.fluid;
  step_velocity_diffuse_and_project(f, dt);
  step_velocity_advect_and_project(f, dt);
  step_dye_diffuse_and_advect(f, dt);
}

/* ── §13  sources — inject a push and some dye at one cell ── */
/*
 * Used by the arrow keys, the dye-drop key, and the auto-emitters.
 * The time-step factor is baked in here so callers can think in plain
 * "amount of push" and "amount of dye" rather than per-tick rates.
 */
static void add_source_at(int i, int j, float force_x, float force_y,
                          float dye_value) {
  if (i < 1 || i > GRID_SIDE_INNER)
    return;
  if (j < 1 || j > GRID_SIDE_INNER)
    return;
  g_scene.fluid.velocity_x[cell_index(i, j)] += DT_DEFAULT * force_x * INJECT_FORCE_SCALE;
  g_scene.fluid.velocity_y[cell_index(i, j)] += DT_DEFAULT * force_y * INJECT_FORCE_SCALE;
  g_scene.fluid.dye_density[cell_index(i, j)] += DT_DEFAULT * dye_value * INJECT_DYE_SCALE;
}

/* ── §14  emitters — the two spinning dye sources ── */
/*
 * Two sources sit a third and two-thirds across, at mid-height.  Each
 * tick they nudge the spin angle and inject dye plus a push that
 * points along it.  They're set half a turn apart so they spin
 * opposite ways, which looks lively and shows off the rebalancing step.
 */
static void emitters_inject(void) {
  g_scene.emitter.swirl_phase += EMITTER_SWIRL_INCREMENT;

  int N = GRID_SIDE_INNER;

  /* left source */
  int i_left = N / 3;
  int j_left = N / 2;
  float fx_left = cosf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  float fy_left = sinf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  add_source_at(i_left, j_left, fx_left, fy_left, EMITTER_DYE_AMPLITUDE);

  /* right source, pushing the opposite way */
  int i_right = 2 * N / 3;
  int j_right = N / 2;
  float fx_right = -cosf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  float fy_right = -sinf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  add_source_at(i_right, j_right, fx_right, fy_right, EMITTER_DYE_AMPLITUDE);
}

/* ── §15  fluid_reset — clear everything back to empty ── */
/*
 * Used at startup and when 'r' is pressed.  Blank every array; the
 * emitters refill the screen over the next handful of ticks.  Also
 * reset the brightness scale so it re-tunes from scratch.
 */
static void fluid_reset(void) {
  memset(g_scene.fluid.velocity_x, 0, sizeof g_scene.fluid.velocity_x);
  memset(g_scene.fluid.velocity_y, 0, sizeof g_scene.fluid.velocity_y);
  memset(g_scene.fluid.velocity_x_prev, 0, sizeof g_scene.fluid.velocity_x_prev);
  memset(g_scene.fluid.velocity_y_prev, 0, sizeof g_scene.fluid.velocity_y_prev);
  memset(g_scene.fluid.dye_density, 0, sizeof g_scene.fluid.dye_density);
  memset(g_scene.fluid.dye_density_prev, 0, sizeof g_scene.fluid.dye_density_prev);
  g_scene.emitter.swirl_phase = 0.0f;
  g_scene.render.dye_max_smoothed = DYE_MAX_INITIAL;
}

/* ── §16  dye_normaliser — a steady brightness scale ── */
/*
 * The brightest dye on screen jumps around as the emitters pulse.  If
 * we scaled the picture by that raw peak, the whole screen would
 * flicker.  So we ease the scale toward the current peak instead of
 * snapping to it: dye_max_per_frame finds this frame's peak, and
 * dye_normaliser_advance nudges the kept value a little toward it.  The
 * eased value lives in g_scene.render.dye_max_smoothed (§6).
 */

static float dye_max_per_frame(void) {
  float frame_max = 0.0f;
  for (int j = 1; j <= GRID_SIDE_INNER; j++) {
    for (int i = 1; i <= GRID_SIDE_INNER; i++) {
      float v = g_scene.fluid.dye_density[cell_index(i, j)];
      if (v > frame_max)
        frame_max = v;
    }
  }
  return frame_max;
}

static void dye_normaliser_advance(void) {
  float frame_max = dye_max_per_frame();
  g_scene.render.dye_max_smoothed =
      DYE_MAX_EMA_OLD * g_scene.render.dye_max_smoothed + DYE_MAX_EMA_NEW * frame_max;
  if (g_scene.render.dye_max_smoothed < DYE_MAX_FLOOR)
    g_scene.render.dye_max_smoothed = DYE_MAX_FLOOR;
}

/* ── §17  glyph_picker — turn a density into a character + brightness ── */
/*
 * Denser dye gets a heavier glyph and a brighter step:
 *   faint  '.'  ->  ':'  ->  '+'  ->  '#'  dense.
 * Cells fainter than the blank threshold are skipped by the renderer.
 */

/*
 * GlyphChoice — what to draw for one cell.
 *
 * Two things go together: which character, and which brightness step
 * of the current colour.  Returning them as one little struct keeps
 * the call site tidy.  The step is kept separate from a full colour
 * pair so the caller can combine it with whichever colour is active.
 *
 * Members
 *   glyph         the character to draw, or 0 meaning "too faint, skip".
 *   shade_index   which brightness step (0..DYE_SHADE_COUNT-1).
 *
 * Invariants
 *   glyph is one of '.', ':', '+', '#', or 0.
 *   shade_index is in range.
 */
typedef struct {
    char glyph;
    int  shade_index;
} GlyphChoice;

static GlyphChoice glyph_for_density(float density_normalised) {
  GlyphChoice out = {'#', 3};
  if (density_normalised < DENSITY_GLYPH_LOW) {
    out.glyph = '.';
    out.shade_index = 0;
  } else if (density_normalised < DENSITY_GLYPH_MID) {
    out.glyph = ':';
    out.shade_index = 1;
  } else if (density_normalised < DENSITY_GLYPH_HIGH) {
    out.glyph = '+';
    out.shade_index = 2;
  } else {
    out.glyph = '#';
    out.shade_index = 3;
  }
  return out;
}

/* ── §18  grid_to_terminal — place a grid cell on screen ── */
/*
 * Spread the grid across the terminal, leaving the HUD rows clear at
 * top and bottom.  We flip j on the way down because the grid counts
 * upward but the terminal counts downward, so up stays up.
 */

static int grid_i_to_term_col(int i, int term_cols) {
  return (i - 1) * term_cols / GRID_SIDE_INNER;
}

static int grid_j_to_term_row(int j, int term_rows) {
  int draw_rows = term_rows - HUD_RESERVED_ROWS_TOP - HUD_RESERVED_ROWS_BOTTOM;
  if (draw_rows < 1)
    draw_rows = 1;
  return (GRID_SIDE_INNER - j) * draw_rows / GRID_SIDE_INNER +
         HUD_RESERVED_ROWS_TOP;
}

/* ── §19  render — draw the dye to the screen ── */
/*
 * For each cell: scale its dye against the steady brightness value,
 * skip it if it's too faint, otherwise pick a glyph + colour and draw
 * it.  This only reads the simulation and writes to the screen — it
 * never changes any sim state.
 */
/* Scale a cell's dye against the steady brightness value so the colours
 * track whatever range the field is in right now (otherwise one early
 * bright burst would leave everything afterward looking dim). */
static inline float normalise_dye_at(int i, int j) {
    return g_scene.fluid.dye_density[cell_index(i, j)] / g_scene.render.dye_max_smoothed;
}

/* Work out the screen spot for a cell; return false if it lands in the
 * HUD rows so the caller can skip it.  Doing the check here keeps the
 * draw helper clean. */
static inline bool grid_cell_to_screen(int i, int j,
                                       int term_rows, int term_cols,
                                       int *out_col, int *out_row) {
    int col = grid_i_to_term_col(i, term_cols);
    int row = grid_j_to_term_row(j, term_rows);
    if (col < 0 || col >= term_cols)                          return false;
    if (row < HUD_RESERVED_ROWS_TOP)                          return false;
    if (row >= term_rows - HUD_RESERVED_ROWS_BOTTOM)          return false;
    *out_col = col;
    *out_row = row;
    return true;
}

/* Draw one cell: density picks the glyph and brightness step (§17),
 * the active colour picks the pair (§5), one character goes out.
 * [8] Bourke for the density-to-character idea. */
static inline void paint_dye_cell(int screen_row, int screen_col,
                                  float density_normalised) {
    GlyphChoice gc      = glyph_for_density(density_normalised);
    int         pair_id = dye_pair_id(g_scene.render.active_channel,
                                       gc.shade_index);
    attron(COLOR_PAIR(pair_id));
    mvaddch(screen_row, screen_col, (chtype)(unsigned char)gc.glyph);
    attroff(COLOR_PAIR(pair_id));
}

static void render_dye_field(int term_rows, int term_cols) {
    for (int j = 1; j <= GRID_SIDE_INNER; j++) {
        for (int i = 1; i <= GRID_SIDE_INNER; i++) {
            float rho_norm = normalise_dye_at(i, j);
            if (rho_norm < DENSITY_GLYPH_BLANK)
                continue;

            int col, row;
            if (!grid_cell_to_screen(i, j, term_rows, term_cols, &col, &row))
                continue;

            paint_dye_cell(row, col, rho_norm);
        }
    }
}

/* ── §20  hud — status line up top, key hints along the bottom ── */

static void hud_paint_status(int term_cols) {
  char buf[160];
  snprintf(buf, sizeof buf,
           " StableFluids  grid:%dx%d  visc:%.2e  dye:%-5s  %s ",
           GRID_SIDE_INNER, GRID_SIDE_INNER, (double)g_scene.fluid.viscosity_kinematic,
           dye_palette_table[g_scene.render.active_channel].name,
           g_scene.sim.paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = term_cols - len;
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  const char *hint = " q:quit  p:pause  r:reset  arrows:wind  d/spc:dye  "
                     "1/2/3:colour  +/-:visc ";
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0, "%s", hint);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §21  screen — set up, tear down, and present the terminal ── */

/*
 * Screen — how big the terminal is, plus the ncurses setup/teardown.
 *
 * Holds the current width and height in characters, used to place the
 * HUD and clip the field.  We re-read them at startup and on every
 * resize.  Keeping them in their own little struct means only the
 * handful of screen functions deal with ncurses directly.
 *
 * One frame: erase, draw the dye, draw the HUD, then flush in a single
 * diff so the terminal doesn't flicker.
 *
 * Members
 *   rows   terminal height in characters.
 *   cols   terminal width in characters.
 *
 * Invariants
 *   Both are positive after setup and stay current across resizes.
 */
typedef struct {
    int rows;
    int cols;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present_frame(Screen *s) {
  erase();
  render_dye_field(s->rows, s->cols);
  hud_paint_status(s->cols);
  hud_paint_hint(s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §22  app — startup, the main loop, signals, and input ── */

/*
 * FpsCounter — a smoothed frames-per-second reading.
 *
 * A per-frame fps reading jitters with every scheduler hiccup, so this
 * adds up frames and elapsed time over a half-second window and only
 * then works out the rate.  It isn't shown in this demo's HUD; it's
 * kept for consistency with the other demos and in case a later HUD
 * tweak wants it.
 *
 * Members
 *   frame_count   frames counted so far in the current window.
 *   window_ns     time counted so far in the current window.
 *   display       the latest smoothed rate.
 */
typedef struct {
  int     frame_count;
  int64_t window_ns;
  double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt) {
  const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;
  f->frame_count++;
  f->window_ns += dt;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — the OS-side bits: the terminal, the fps reading, and the two
 * flags the signal handler sets and the main loop checks.  (The
 * simulation state lives separately in g_scene.)
 *
 * Members
 *   screen        terminal size + ncurses setup.
 *   fps           the smoothed fps reading.
 *   quit          set when asked to stop (Ctrl-C / kill).
 *   need_resize   set on a terminal resize; handled next loop.
 */
typedef struct {
    Screen     screen;
    FpsCounter fps;
    volatile sig_atomic_t quit;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig) {
  if (sig == SIGWINCH) g_app.need_resize = 1;
  else                 g_app.quit        = 1;
}

/* One small helper per key, so app_handle_key reads as a plain list. */

/* Drop a dab of dye at a random cell with no push, so you can watch it
 * spread and drift on its own. */
static void drop_random_dye_blob(void) {
  enum { DYE_BLOB_AMOUNT = 5 };
  int i = rand_inner_cell();
  int j = rand_inner_cell();
  add_source_at(i, j, 0.0f, 0.0f, (float)DYE_BLOB_AMOUNT);
}

/* Give the fluid a shove at the centre; hold an arrow to keep dragging
 * it that way. */
static void push_velocity_at_centre(float force_x, float force_y) {
  enum { ARROW_DYE_PULSE = 1 };
  int centre = GRID_SIDE_INNER / 2;
  add_source_at(centre, centre, force_x, force_y, (float)ARROW_DYE_PULSE);
}

static inline void toggle_paused(void) {
  g_scene.sim.paused = !g_scene.sim.paused;
}

/* Step viscosity up or down.  We multiply rather than add because
 * viscosity ranges over many orders of magnitude, so a fixed multiplier
 * feels like the same change everywhere on the scale. */
static void adjust_viscosity(int dir) {
  if (dir > 0) {
    g_scene.fluid.viscosity_kinematic *= VISCOSITY_FACTOR;
    if (g_scene.fluid.viscosity_kinematic > VISCOSITY_MAX)
      g_scene.fluid.viscosity_kinematic = VISCOSITY_MAX;
  } else {
    g_scene.fluid.viscosity_kinematic /= VISCOSITY_FACTOR;
    if (g_scene.fluid.viscosity_kinematic < VISCOSITY_MIN)
      g_scene.fluid.viscosity_kinematic = VISCOSITY_MIN;
  }
}

/* Pick the dye colour; this is look-only, it doesn't touch the physics. */
static inline void set_dye_channel(int channel) {
  g_scene.render.active_channel = channel;
}

/* Handle one keypress; returns false to quit. */
static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q': case 'Q': case 27 /* ESC */: return false;

  case 'p': case 'P':           toggle_paused();                  break;
  case 'r': case 'R':           fluid_reset();                    break;

  case ' ': case 'd': case 'D': drop_random_dye_blob();           break;

  case KEY_LEFT:                push_velocity_at_centre(-1, 0);   break;
  case KEY_RIGHT:               push_velocity_at_centre(+1, 0);   break;
  case KEY_UP:                  push_velocity_at_centre(0, -1);   break;
  case KEY_DOWN:                push_velocity_at_centre(0, +1);   break;

  case '+': case '=':           adjust_viscosity(+1);             break;
  case '-':                     adjust_viscosity(-1);             break;

  case '1':                     set_dye_channel(DYE_CHANNEL_BLUE);  break;
  case '2':                     set_dye_channel(DYE_CHANNEL_GREEN); break;
  case '3':                     set_dye_channel(DYE_CHANNEL_RED);   break;

  default: break;
  }
  return true;
}

int main(void) {
  srand((unsigned)time(NULL));
  atexit(screen_cleanup);
  signal(SIGINT,   on_signal);
  signal(SIGTERM,  on_signal);
  signal(SIGWINCH, on_signal);

  App *app = &g_app;
  fps_counter_init(&app->fps);
  screen_init(&app->screen);
  fluid_reset();

  /* Run a few ticks before showing anything so the first frame already
   * has dye in it instead of a blank screen. */
  for (int i = 0; i < PREWARM_TICK_COUNT; i++) {
    emitters_inject();
    fluid_step(DT_DEFAULT);
  }

  /* Start the brightness scale from what's already on screen. */
  g_scene.render.dye_max_smoothed = dye_max_per_frame();
  if (g_scene.render.dye_max_smoothed < DYE_MAX_FLOOR)
    g_scene.render.dye_max_smoothed = DYE_MAX_INITIAL;

  int64_t prev_ns = clock_now_ns();
  while (!app->quit) {
    int64_t frame_start_ns = clock_now_ns();

    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch)) {
        app->quit = 1;
        break;
      }
    }

    if (app->need_resize) {
      app->need_resize = 0;
      screen_resize(&app->screen);
    }

    if (!g_scene.sim.paused) {
      emitters_inject();
      fluid_step(DT_DEFAULT);
    }

    dye_normaliser_advance();
    screen_present_frame(&app->screen);

    int64_t now_ns = clock_now_ns();
    fps_counter_tick(&app->fps, now_ns - prev_ns);
    prev_ns = now_ns;

    /* hold the frame rate steady: sleep off whatever time is left */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
