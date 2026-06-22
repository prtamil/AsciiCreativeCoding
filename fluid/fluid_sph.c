/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fluid_sph.c — a pool of particles that behaves like water.
 *
 * Each particle is one drop.  Every tick we measure how crowded each
 * drop is, push crowded drops apart and pull lonely ones together, let
 * neighbours match speeds, add gravity, and bounce off the walls.  Out
 * of those few local rules you get splashing, sloshing, and a visible
 * surface.  No fluid equations on a grid — just particles + a smoothing
 * kernel + a spatial hash for speed.  This is the particle (Lagrangian)
 * approach; fluid/navier_stokes.c is the grid (Eulerian) sibling.
 *
 * Method: Smoothed Particle Hydrodynamics.  Originals: Lucy 1977;
 *   Gingold & Monaghan 1977.  Real-time form used here: Müller,
 *   Charypar & Gross 2003.  Spatial-hash neighbour search: Teschner
 *   et al. 2003.  Density-to-character ramp idea: Bourke 1997.
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

/* ── §1  config — every tunable in one place ── */
/*
 * Everything the code can vary lives here as a named constant, so there
 * are no bare numbers buried in the logic below.  Grouped by topic.
 */

/* particle pool */
#define PARTICLE_POOL_CAPACITY 5000

/* SPH physics */

/* How far a particle's influence reaches, in cells.  Two particles
 * affect each other only when closer than this. */
#define SMOOTH_RADIUS_CELLS 2.2

/* How stiff the fluid is.  Bigger = resists being squashed harder, but
 * the simulation blows up if you push it too far for the chosen step. */
#define PRESSURE_K 0.04

/* How strongly neighbours drag each other's speed into agreement. */
#define VISCOSITY_K 0.03

/* Downward pull per step.  Kept small so a fast drop can't jump clean
 * through the floor in one step. */
#define GRAVITY_G 0.08

/* How bouncy the walls are.  0.6 = keep 60% of the speed on a bounce,
 * lose the rest as "heat" — that loss is what stops the pool slowly
 * pumping itself up to an explosion from rounding errors. */
#define WALL_DAMPING 0.6

/* Fixed amount of simulated time per physics step.  Small enough that a
 * particle never moves more than about one influence-radius per step. */
#define SPH_DT 0.12

/* The crowd level a settled fluid wants.  When two neighbours' crowding
 * adds up to this, they neither push nor pull.  Tuned to the radius. */
#define REST_SUM 6.0

/* Tiny number added to a divisor so a lonely particle never divides by
 * zero crowding. */
#define DENSITY_DIVIDE_GUARD 0.001

/* spatial-hash grid */

/* Side of one grid cell, in cells.  Must be at least the influence
 * radius so a particle's neighbours are always in the 3x3 block of
 * cells around it. */
#define GRID_CELL_SIZE 3

/* Largest grid we'll ever allocate (caps the fixed arrays). */
#define GRID_COLS_MAX 90
#define GRID_ROWS_MAX 22

/* density -> character */

/* Crowding cutoffs that pick the glyph + colour in §15. */
#define DENSITY_THRESHOLD_CORE 3.5 /* dense interior  '#' bold */
#define DENSITY_THRESHOLD_BODY 1.2 /* fluid body       'o'      */
#define DENSITY_THRESHOLD_EDGE 0.1 /* sparse edge      '.'      */

/* HUD: bottom rows reserved for the status + key strip */
#define HUD_RESERVED_ROWS 2

/* colour pairs */
enum {
  PAIR_DENSITY_CORE = 1,
  PAIR_DENSITY_BODY,
  PAIR_DENSITY_EDGE,
  PAIR_BORDER,
  PAIR_HUD,
  PAIR_HINT,
};

#define SCENE_COUNT 5
#define THEME_COUNT 10

/* frame + sim timing */
#define SIM_HZ_MIN 10
#define SIM_HZ_DEFAULT 60
#define SIM_HZ_MAX 120
#define SIM_HZ_STEP 10
#define RENDER_FPS_CAP 60
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* ── §2  clock — a steady timer + sleep ── */

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

/* ── §3  rng — small random helper ── */

static int rand_in_range(int lo, int hi_exclusive) {
  if (hi_exclusive <= lo)
    return lo;
  return lo + rand() % (hi_exclusive - lo);
}

/* ── §4  themes — 10 colour sets + names ── */

/*
 * ColorTheme — three colours for one look of the fluid.
 *
 * The fluid is drawn in three crowding levels — dense core, body, and
 * thin edge/spray — so a theme just needs one colour for each, plus a
 * name to show in the HUD.  Three levels (not a fine gradient) is
 * plenty here: it's the SHAPE of the fluid that carries the picture,
 * not subtle shading within it.
 *
 * All three colours are kept reasonably bright (index >= 24) so even
 * the dim edge colour shows up against a black terminal; the very dark
 * end of the palette would just vanish.  init_pair turns these into
 * live colour pairs (Raymond's NCURSES HOWTO).
 *
 *   core   colour of the densest cells (drawn as '#')
 *   body   colour of the mid cells     (drawn as 'o')
 *   edge   colour of the thin surface / droplets (drawn as '.')
 *   name   short label shown in the HUD; the 't' key cycles themes
 */
typedef struct {
    short       core;     /* densest cells (liquid core)              */
    short       body;     /* medium-density cells (fluid body)         */
    short       edge;     /* low-density cells (droplets / spray)      */
    const char *name;     /* short ASCII label shown in HUD            */
} ColorTheme;

static const ColorTheme color_theme_table[THEME_COUNT] = {
    {51, 39, 27, "ocean"},     /* cyan to blue                     */
    {196, 208, 220, "lava"},   /* red to orange to yellow          */
    {226, 214, 196, "fire"},   /* white-yellow to orange to red    */
    {46, 34, 28, "matrix"},    /* bright green to mid to dark      */
    {231, 141, 93, "nova"},    /* white to violet to purple        */
    {231, 159, 117, "ice"},    /* white to sky to steel            */
    {220, 208, 197, "sunset"}, /* yellow to orange to rose         */
    {196, 160, 124, "blood"},  /* bright red to crimson to dark    */
    {201, 198, 165, "neon"},   /* magenta to pink to soft purple   */
    {154, 118, 46, "acid"},    /* yellow-green to green            */
};

/* ── §5  colours — set up the colour pairs ── */

/* Fixed bright colours for the HUD and border (the project standard). */
enum {
    HUD_YELLOW_256 = 226,
    HUD_CYAN_256   = 51,
    BORDER_GRAY_256 = 244,
};

static inline void apply_density_palette_xterm256(const ColorTheme *theme) {
    init_pair(PAIR_DENSITY_CORE, theme->core, -1);
    init_pair(PAIR_DENSITY_BODY, theme->body, -1);
    init_pair(PAIR_DENSITY_EDGE, theme->edge, -1);
}

/* Fallback for old terminals with only 8 colours. */
static inline void apply_density_palette_ansi8(void) {
    init_pair(PAIR_DENSITY_CORE, COLOR_CYAN, -1);
    init_pair(PAIR_DENSITY_BODY, COLOR_CYAN, -1);
    init_pair(PAIR_DENSITY_EDGE, COLOR_BLUE, -1);
}

/* Border + HUD colours, which don't change with the theme. */
static inline void apply_chrome_palette(bool have_256_colors) {
    if (have_256_colors) {
        init_pair(PAIR_BORDER, BORDER_GRAY_256, -1);
        init_pair(PAIR_HUD,    HUD_YELLOW_256,  -1);
        init_pair(PAIR_HINT,   HUD_CYAN_256,    -1);
    } else {
        init_pair(PAIR_BORDER, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,    COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,   -1);
    }
}

static void colors_apply_theme(int theme_index) {
    if (theme_index < 0 || theme_index >= THEME_COUNT)
        theme_index = 0;
    const ColorTheme *theme = &color_theme_table[theme_index];

    bool have_256_colors = (COLORS >= 256);
    if (have_256_colors) apply_density_palette_xterm256(theme);
    else                 apply_density_palette_ansi8();
    apply_chrome_palette(have_256_colors);
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  colors_apply_theme(theme_index);
}

/* ── §6  particle — the drop, the pool, and how to spawn them ── */

/*
 * Particle — one drop of fluid.
 *
 * The whole fluid is just a big bag of these.  Each drop carries where
 * it is, how fast it's going, a running total of the forces pushed onto
 * it this step, and a number for how crowded its neighbourhood is.
 *
 * The crowding number is worked out once per step and then read several
 * times (the force step needs it for this drop AND its neighbours, and
 * the renderer uses it to pick a character).  Caching it on the drop
 * means we don't re-measure the neighbourhood every time we need it.
 *
 * Positions and speeds are in CELL units — the terminal grid IS the
 * playing field, there's no separate pixel space.  Stored as double
 * because the force math subtracts small nearly-equal numbers and
 * floats would lose them in noise.
 *
 *   pos_col, pos_row    where the drop is
 *   vel_col, vel_row    how fast it's moving (cells per step)
 *   accel_col, accel_row this step's forces, added up; cleared and
 *                        refilled every force step
 *   density_estimate    how crowded it is; set by the density step,
 *                        read by the force step and the renderer.
 *                        Always >= 0 (it counts itself, so >= 1).
 */
typedef struct {
    double pos_col;          /* position (cells)                      */
    double pos_row;
    double vel_col;          /* velocity (cells / step)               */
    double vel_row;
    double accel_col;        /* this step's forces, added up          */
    double accel_row;
    double density_estimate; /* how crowded (set by the density step) */
} Particle;

/*
 * ParticlePool — all the drops, just storage.
 *
 * A fixed array plus a count of how many are actually in use; the rest
 * of the array is junk and must not be read.  It's one global because
 * every step and every painter touches it, so passing a pointer around
 * everywhere would just be noise.
 *
 *   pool[]   the drops; only the first `count` are real
 *   count    how many are live (0..capacity); back to 0 on reset or a
 *            scene change
 */
typedef struct {
    Particle pool [PARTICLE_POOL_CAPACITY];
    int      count;
} ParticlePool;

static ParticlePool g_particle_pool;

/* Add one drop, if there's room. */
static void particle_spawn_at(double pos_col, double pos_row) {
  if (g_particle_pool.count >= PARTICLE_POOL_CAPACITY)
    return;
  Particle *p = &g_particle_pool.pool[g_particle_pool.count];
  p->pos_col = pos_col;
  p->pos_row = pos_row;
  p->vel_col = 0.0;
  p->vel_row = 0.0;
  p->accel_col = 0.0;
  p->accel_row = 0.0;
  p->density_estimate = 0.0;
  g_particle_pool.count++;
}

/* Fill a circle of drops. */
static void particle_spawn_blob(int centre_col, int centre_row,
                                int radius_cells) {
  for (int dy = -radius_cells; dy <= radius_cells; dy++) {
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      if (dx * dx + dy * dy <= radius_cells * radius_cells)
        particle_spawn_at((double)(centre_col + dx), (double)(centre_row + dy));
    }
  }
}

/* Fill a solid rectangle of drops. */
static void particle_spawn_rectangle(int corner_col, int corner_row,
                                     int width_cells, int height_cells) {
  for (int dy = 0; dy < height_cells; dy++)
    for (int dx = 0; dx < width_cells; dx++)
      particle_spawn_at((double)(corner_col + dx), (double)(corner_row + dy));
}

/* ── §7  kernel — how much a nearby drop counts ── */
/*
 * How much one drop influences another at a given distance.  Returns 0
 * once they're out of range (farther than the influence radius), and a
 * NEGATIVE number when in range, growing toward 0 at the edge.  The
 * density step squares it (so it counts as a positive amount of
 * crowding); the force step uses the negative sign to decide which way
 * to push.  Negative-means-in-range is also the cheap range test.
 */
static double sph_kernel_signed(double distance_cells) {
  double w = distance_cells / SMOOTH_RADIUS_CELLS - 1.0;
  return (w < 0.0) ? w : 0.0;
}

/* Distance between two drops.  Also hands back the gap as separate
 * column/row pieces (this drop minus the other) — the force step needs
 * the direction, not just the distance. */
static double sph_pair_distance(const Particle *pi, const Particle *pj,
                                double *delta_col, double *delta_row) {
  double dc = pi->pos_col - pj->pos_col;
  double dr = pi->pos_row - pj->pos_row;
  *delta_col = dc;
  *delta_row = dr;
  return sqrt(dc * dc + dr * dr);
}

/* ── §8  grid — find neighbours fast ── */

/*
 * Grid — sorts drops into cells so each drop only looks at nearby drops.
 *
 * Without it, finding a drop's neighbours means checking every other
 * drop — far too slow with thousands of them.  Instead we chop the area
 * into cells (each at least one influence-radius wide) and drop each
 * particle into its cell.  Then a drop only needs to look in its own
 * cell and the eight around it; anything farther is out of range
 * anyway.  (Teschner et al. 2003.)
 *
 * The cells are stored as one linked list per cell, using only plain
 * integers (no per-cell allocation, which we avoid after startup):
 *   - head[row][col] is the first drop's index in that cell, or -1.
 *   - next[i] is the next drop sharing drop i's cell, or -1 at the end.
 * So you walk a cell with:
 *   for (j = head[row][col]; j != -1; j = next[j]) ...
 *
 *   head[][]     first drop in each cell, or -1; wiped to -1 each step
 *   next[]       next drop in the same cell, or -1; rebuilt each step
 *   active_cols  how many cells wide the grid currently is. It's the
 *   active_rows  area divided by cell size, plus a 2-cell border so the
 *                3x3 look-around never falls off the edge; capped at the
 *                array size.
 */
typedef struct {
    int head[GRID_ROWS_MAX][GRID_COLS_MAX];
    int next[PARTICLE_POOL_CAPACITY];
    int active_cols;
    int active_rows;
} Grid;

static Grid g_spatial_grid;

/* ── §8.5  scene state — World, SimControls, Scene ── */
/* (Defined up here so the physics steps below can take a Scene *.)      */

/*
 * World — the size of the play area, in cells.
 *
 * Just the width and height the fluid lives in.  Re-read every tick from
 * the terminal size (minus the rows reserved for the HUD), so resizing
 * the window resizes the pool.  The fluid never paints into the HUD
 * rows.
 */
typedef struct {
    int width;
    int height;
} World;

/*
 * SimControls — the three on/off switches the user flips.
 *
 *   paused             space bar: freeze the simulation; HUD shows PAUSED
 *   gravity_enabled    'g': add the downward pull
 *   viscosity_enabled  'v': let neighbours match speeds.  Turn it off and
 *                      the fluid gets twitchier and splashier.
 */
typedef struct {
    bool paused;
    bool gravity_enabled;
    bool viscosity_enabled;
} SimControls;

/*
 * Scene — everything about one running simulation, in one place.
 *
 * Bundling it all behind a single Scene * means helpers take that one
 * pointer and there's one spot to look for "what state exists."  The
 * pool and grid are huge (a few hundred KB), so Scene holds POINTERS to
 * the two big globals rather than copies of them — set once at startup.
 *
 *   active_id     which preset is loaded (1..5); also the HUD label
 *   theme_index   which colour set is active
 *   sim           the user switches (paused / gravity / viscosity)
 *   world         the play-area size
 *   particles     -> the one global drop pool
 *   grid          -> the one global neighbour grid
 */
typedef struct {
    int           active_id;
    int           theme_index;
    SimControls   sim;
    World         world;
    ParticlePool *particles;
    Grid         *grid;
} Scene;

static inline int grid_cell_col_of(double pos_col) {
  int gx = (int)(pos_col / GRID_CELL_SIZE);
  if (gx < 0)
    gx = 0;
  if (gx >= g_spatial_grid.active_cols)
    gx = g_spatial_grid.active_cols - 1;
  return gx;
}

static inline int grid_cell_row_of(double pos_row) {
  int gy = (int)(pos_row / GRID_CELL_SIZE);
  if (gy < 0)
    gy = 0;
  if (gy >= g_spatial_grid.active_rows)
    gy = g_spatial_grid.active_rows - 1;
  return gy;
}

/* Size the grid to the current play area, plus a 2-cell border so the
 * 3x3 look-around never falls off the edge, capped at the array size. */
static inline void compute_active_grid_bounds(const Scene *s) {
    g_spatial_grid.active_cols = s->world.width  / GRID_CELL_SIZE + 2;
    g_spatial_grid.active_rows = s->world.height / GRID_CELL_SIZE + 2;
    if (g_spatial_grid.active_cols > GRID_COLS_MAX) g_spatial_grid.active_cols = GRID_COLS_MAX;
    if (g_spatial_grid.active_rows > GRID_ROWS_MAX) g_spatial_grid.active_rows = GRID_ROWS_MAX;
}

/* Empty every cell before refilling.  (The next[] links get overwritten
 * as drops are re-added, so only the heads need clearing.) */
static inline void clear_all_cell_heads(void) {
    for (int gy = 0; gy < g_spatial_grid.active_rows; gy++)
        for (int gx = 0; gx < g_spatial_grid.active_cols; gx++)
            g_spatial_grid.head[gy][gx] = -1;
}

/* Drop particle i into its cell (push onto the front of that cell's
 * list). */
static inline void insert_particle_into_cell(int i) {
    int gx = grid_cell_col_of(g_particle_pool.pool[i].pos_col);
    int gy = grid_cell_row_of(g_particle_pool.pool[i].pos_row);
    g_spatial_grid.next[i]      = g_spatial_grid.head[gy][gx];
    g_spatial_grid.head[gy][gx] = i;
}

/* Re-sort every drop into its cell.  Done first thing each step, since
 * the drops moved last step. */
static void grid_rebuild(const Scene *s) {
    compute_active_grid_bounds(s);
    clear_all_cell_heads();
    for (int i = 0; i < g_particle_pool.count; i++)
        insert_particle_into_cell(i);
}

/* ── §9  density pass — how crowded is each drop? ── */
/*
 * For each drop, look at the 3x3 block of cells around it and add up how
 * much each nearby drop counts.  A drop counts itself too, so the answer
 * is always at least 1 — handy, it means we never divide by zero later.
 */

/* The 3x3 block of cells around a drop, trimmed to stay on the grid.
 * Hands back the column/row ranges to loop over.  Used by both the
 * density and force steps. */
static inline void cell_block_around(const Particle *pi,
                                     int *out_gx_lo, int *out_gx_hi,
                                     int *out_gy_lo, int *out_gy_hi) {
    int cx = grid_cell_col_of(pi->pos_col);
    int cy = grid_cell_row_of(pi->pos_row);
    *out_gx_lo = (cx - 1 < 0) ? 0 : cx - 1;
    *out_gx_hi = (cx + 1 >= g_spatial_grid.active_cols) ? g_spatial_grid.active_cols - 1 : cx + 1;
    *out_gy_lo = (cy - 1 < 0) ? 0 : cy - 1;
    *out_gy_hi = (cy + 1 >= g_spatial_grid.active_rows) ? g_spatial_grid.active_rows - 1 : cy + 1;
}

/* How much one neighbour adds to a drop's crowding.  Squared so it's
 * always a positive amount (Müller 2003). */
static inline double squared_kernel_contribution(const Particle *pi,
                                                  const Particle *pj) {
    double dc, dr;
    double distance = sph_pair_distance(pi, pj, &dc, &dr);
    double w        = sph_kernel_signed(distance);
    return w * w;
}

/* Give every drop its crowding number for this step. */
static void density_pass(void) {
    for (int i = 0; i < g_particle_pool.count; i++) {
        Particle *pi = &g_particle_pool.pool[i];
        pi->density_estimate = 0.0;

        int gx_lo, gx_hi, gy_lo, gy_hi;
        cell_block_around(pi, &gx_lo, &gx_hi, &gy_lo, &gy_hi);

        for (int gy = gy_lo; gy <= gy_hi; gy++) {
            for (int gx = gx_lo; gx <= gx_hi; gx++) {
                for (int j = g_spatial_grid.head[gy][gx]; j != -1; j = g_spatial_grid.next[j]) {
                    pi->density_estimate +=
                        squared_kernel_contribution(pi, &g_particle_pool.pool[j]);
                }
            }
        }
    }
}

/* ── §10  force pass — push the drops around ── */
/*
 * Two pushes between every close pair of drops:
 *   - PRESSURE keeps spacing: if the pair is more crowded than the fluid
 *     wants, shove them apart; if less, pull them together (which gives
 *     the surface its skin-like tension).
 *   - VISCOSITY (optional) nudges each drop toward its neighbours' speed,
 *     so they flow together instead of cutting through each other.
 * Gravity is added once up front as the starting push, so it isn't
 * multiplied by the neighbour count inside the pair loop.
 */

/* Reset a drop's forces to just gravity (the starting point each step). */
static inline void apply_gravity_baseline(Particle *pi, const Scene *s) {
    pi->accel_col = 0.0;
    pi->accel_row = s->sim.gravity_enabled ? GRAVITY_G : 0.0;
}

/* The spacing push between two drops.  When the pair is over-crowded the
 * push is outward (apart); when under-crowded it's inward (together);
 * when just right it's zero.  Müller 2003, simplified.  We divide by
 * this drop's crowding (plus a tiny guard so a lonely drop can't divide
 * by zero). */
static inline void apply_pressure_pair_force(Particle *pi,
                                              const Particle *pj,
                                              double dc, double dr,
                                              double w_signed) {
    double pressure_term =
        (REST_SUM - pi->density_estimate - pj->density_estimate) * PRESSURE_K;
    double force_per_dx =
        w_signed * pressure_term /
        (pi->density_estimate + DENSITY_DIVIDE_GUARD);
    pi->accel_col += dc * force_per_dx;
    pi->accel_row += dr * force_per_dx;
}

/* The speed-matching push: nudge this drop's velocity toward the
 * neighbour's, more strongly for closer neighbours. */
static inline void apply_viscosity_pair_force(Particle *pi,
                                               const Particle *pj,
                                               double w_signed) {
    double visc_weight = -w_signed;  /* flip to a positive weight */
    pi->accel_col += (pj->vel_col - pi->vel_col) * VISCOSITY_K * visc_weight;
    pi->accel_row += (pj->vel_row - pi->vel_row) * VISCOSITY_K * visc_weight;
}

/* Apply both pushes from one neighbour.  Skip the drop itself and any
 * neighbour too far away to matter. */
static inline void apply_sph_pair_forces(Particle *pi, int i,
                                          const Particle *pj, int j,
                                          const Scene *s) {
    if (j == i)
        return;
    double dc, dr;
    double distance = sph_pair_distance(pi, pj, &dc, &dr);
    double w_signed = sph_kernel_signed(distance);
    if (w_signed == 0.0)
        return;  /* out of range */

    apply_pressure_pair_force(pi, pj, dc, dr, w_signed);
    if (s->sim.viscosity_enabled)
        apply_viscosity_pair_force(pi, pj, w_signed);
}

/* Work out the total push on every drop this step. */
static void forces_pass(const Scene *s) {
    for (int i = 0; i < g_particle_pool.count; i++) {
        Particle *pi = &g_particle_pool.pool[i];

        apply_gravity_baseline(pi, s);

        int gx_lo, gx_hi, gy_lo, gy_hi;
        cell_block_around(pi, &gx_lo, &gx_hi, &gy_lo, &gy_hi);

        for (int gy = gy_lo; gy <= gy_hi; gy++) {
            for (int gx = gx_lo; gx <= gx_hi; gx++) {
                for (int j = g_spatial_grid.head[gy][gx]; j != -1; j = g_spatial_grid.next[j]) {
                    apply_sph_pair_forces(pi, i, &g_particle_pool.pool[j], j, s);
                }
            }
        }
    }
}

/* ── §11  move pass — turn forces into motion, then bounce ── */
/*
 * Update each drop's speed from its forces, then move it at the new
 * speed.  Doing speed FIRST and position SECOND (with the just-updated
 * speed) is the small trick that keeps the simulation from slowly
 * gaining energy and blowing up.  Then keep drops inside the walls.
 */

/* Speed first: add this step's forces to the drop's velocity. */
static inline void symplectic_euler_velocity_step(Particle *p, double dt) {
    p->vel_col += p->accel_col * dt;
    p->vel_row += p->accel_row * dt;
}

/* Position next, using that just-updated velocity. */
static inline void symplectic_euler_position_step(Particle *p, double dt) {
    p->pos_col += p->vel_col * dt;
    p->pos_row += p->vel_row * dt;
}

/* Bounce off one wall: if the drop went past the edge, put it back at
 * the edge and reverse that direction, losing a bit of speed.  The speed
 * loss matters — a perfect bounce would slowly heat the pool up until it
 * explodes. */
static inline void enforce_wall_bounce_axis(double *pos, double *vel,
                                             double lo, double hi) {
    if (*pos < lo) { *pos = lo; *vel = -*vel * WALL_DAMPING; }
    if (*pos > hi) { *pos = hi; *vel = -*vel * WALL_DAMPING; }
}

/* Move every drop and bounce it off the walls. */
static void integrate_pass(const Scene *s) {
    /* Walls one cell inside the area, so drops sit just inside the
     * drawn border instead of on top of it. */
    const double col_lo = 1.0, col_hi = (double)(s->world.width  - 2);
    const double row_lo = 1.0, row_hi = (double)(s->world.height - 2);

    for (int i = 0; i < g_particle_pool.count; i++) {
        Particle *p = &g_particle_pool.pool[i];
        symplectic_euler_velocity_step(p, SPH_DT);
        symplectic_euler_position_step(p, SPH_DT);
        enforce_wall_bounce_axis(&p->pos_col, &p->vel_col, col_lo, col_hi);
        enforce_wall_bounce_axis(&p->pos_row, &p->vel_row, row_lo, row_hi);
    }
}

/* ── §12  one physics tick — the whole simulation in four steps ── */
/*
 * The order matters: each step feeds the next.  Re-sort drops into
 * cells, measure crowding, turn crowding into pushes, then move.
 */
static void sph_step(const Scene *s) {
  grid_rebuild  (s);
  density_pass  ();
  forces_pass   (s);
  integrate_pass(s);
}

/* ── §13  scenes — five starting layouts ── */
/*
 * Each one wipes the pool and drops in a fresh arrangement.  Drops start
 * still, except the collision scene which throws two blobs at each other.
 *
 *   1 blob      a big ball falls and splats
 *   2 column    a tall block collapses sideways
 *   3 fountain  a pile at the floor erupts upward
 *   4 collision two blobs slam together
 *   5 rain      a curtain of drops falls and piles up
 */

static void scene_load_blob_drop(const Scene *s) {
  int cx = s->world.width / 2;
  particle_spawn_blob(cx, 6, 12);
}

static void scene_load_column_collapse(const Scene *s) {
  int cx = s->world.width / 2;
  particle_spawn_rectangle(cx - 18, 2, 36, 16);
}

static void scene_load_fountain(const Scene *s) {
  int cx        = s->world.width  / 2;
  int floor_row = s->world.height - 4;
  for (int i = 0; i < 700; i++) {
    int dx = rand_in_range(-4, 5);
    particle_spawn_at((double)(cx + dx), (double)floor_row);
  }
}

static void scene_load_collision(const Scene *s) {
  int cx = s->world.width  / 2;
  int cy = s->world.height / 2;
  particle_spawn_blob(cx - 20, cy, 10);
  particle_spawn_blob(cx + 20, cy, 10);
  /* First half of the spawn list is the LEFT blob, second half the
   * RIGHT.  Slam them together. */
  int half = g_particle_pool.count / 2;
  for (int i = 0; i < g_particle_pool.count; i++)
    g_particle_pool.pool[i].vel_col = (i < half) ? 2.5 : -2.5;
}

static void scene_load_rain(const Scene *s) {
  for (int i = 0; i < 800; i++) {
    int col = rand_in_range(2, s->world.width - 2);
    int row = rand_in_range(1, 7);
    particle_spawn_at((double)col, (double)row);
  }
}

/* Wipe the pool and load the chosen scene. */
static void scene_load_by_id(Scene *s, int id) {
  g_particle_pool.count = 0;
  switch (id) {
  case 1: scene_load_blob_drop      (s); break;
  case 2: scene_load_column_collapse(s); break;
  case 3: scene_load_fountain       (s); break;
  case 4: scene_load_collision      (s); break;
  case 5: scene_load_rain           (s); break;
  default:
    break;
  }
}

static const char *scene_name_of(int id) {
  switch (id) {
  case 1:
    return "blob-drop";
  case 2:
    return "column";
  case 3:
    return "fountain";
  case 4:
    return "collision";
  case 5:
    return "rain";
  default:
    return "?";
  }
}

/* ── §14  scene setup + per-tick update ── */
/* (The Scene/World/SimControls types are up in §8.5.)                   */

static void scene_init(Scene *s, int cols, int rows) {
  s->active_id              = 1;
  s->theme_index            = 0;
  s->sim.paused             = false;
  s->sim.gravity_enabled    = true;
  s->sim.viscosity_enabled  = true;

  /* play area = terminal, minus the HUD rows at the bottom */
  s->world.width  = cols;
  s->world.height = rows - HUD_RESERVED_ROWS;

  /* point at the two big globals */
  s->particles    = &g_particle_pool;
  s->grid         = &g_spatial_grid;

  colors_apply_theme(s->theme_index);
  scene_load_by_id(s, s->active_id);
}

static void scene_tick(Scene *s, int cols, int rows) {
  if (s->sim.paused) return;

  /* re-read the size each tick so a window resize takes effect */
  s->world.width  = cols;
  s->world.height = rows - HUD_RESERVED_ROWS;

  sph_step(s);
}

/* ── §15  draw the drops — crowding picks the character ── */
/*
 * Each drop becomes one character, chosen by how crowded it is: dense
 * interior drops are '#' (bold), mid drops 'o', thin surface drops '.',
 * and very lonely ones aren't drawn at all.  Because surface drops have
 * fewer neighbours, this makes the water's surface stand out for free.
 */

/*
 * DensityRender — how to draw one cell: which character, colour, and
 * whether to make it bold.  All three follow from the same crowding
 * number, so we bundle them and return all three at once.  Three levels
 * (not a fine gradient) is what shows the surface clearly.
 *
 *   glyph        the character: '#', 'o', or '.'
 *   pair_id      which colour pair (core / body / edge) — its colour
 *                comes from the current theme
 *   extra_attr   bold for the dense core, normal otherwise
 */
typedef struct {
    char   glyph;
    int    pair_id;
    attr_t extra_attr;
} DensityRender;

static inline DensityRender make_core_tier(void) {
    return (DensityRender){ '#', PAIR_DENSITY_CORE, A_BOLD };
}

static inline DensityRender make_body_tier(void) {
    return (DensityRender){ 'o', PAIR_DENSITY_BODY, A_NORMAL };
}

static inline DensityRender make_edge_tier(void) {
    return (DensityRender){ '.', PAIR_DENSITY_EDGE, A_NORMAL };
}

/* Pick the character + colour for a drop from its crowding. */
static DensityRender density_render_for(double density_estimate) {
    if (density_estimate >= DENSITY_THRESHOLD_CORE) return make_core_tier();
    if (density_estimate >= DENSITY_THRESHOLD_BODY) return make_body_tier();
    return make_edge_tier();
}

/* Round a drop's real position to the nearest screen cell. */
static inline void particle_to_screen_cell(const Particle *p,
                                            int *out_col, int *out_row) {
    *out_col = (int)(p->pos_col + 0.5);
    *out_row = (int)(p->pos_row + 0.5);
}

static inline bool screen_cell_in_field(int col, int row,
                                         int cols, int phys_area_rows) {
    return col >= 0 && col < cols
        && row >= 0 && row < phys_area_rows;
}

/* Skip nearly-alone drops — they'd just be specks of noise. */
static inline bool particle_should_render(const Particle *p) {
    return p->density_estimate >= DENSITY_THRESHOLD_EDGE;
}

static inline void paint_particle_cell(WINDOW *w, int row, int col,
                                        DensityRender dr) {
    attr_t attrs = COLOR_PAIR(dr.pair_id) | dr.extra_attr;
    wattron(w, attrs);
    mvwaddch(w, row, col, (chtype)(unsigned char)dr.glyph);
    wattroff(w, attrs);
}

/* Draw every drop that's on-screen and worth showing. */
static void render_particles(WINDOW *w, int cols, int phys_area_rows) {
    for (int i = 0; i < g_particle_pool.count; i++) {
        const Particle *p = &g_particle_pool.pool[i];

        int col, row;
        particle_to_screen_cell(p, &col, &row);

        if (!screen_cell_in_field(col, row, cols, phys_area_rows))
            continue;
        if (!particle_should_render(p))
            continue;

        DensityRender dr = density_render_for(p->density_estimate);
        paint_particle_cell(w, row, col, dr);
    }
}

/* ── §16  border — frame around the play area ── */

/* Plain ASCII so it looks the same on every terminal. */
enum {
    BORDER_GLYPH_HORIZONTAL = '-',
    BORDER_GLYPH_VERTICAL   = '|',
    BORDER_GLYPH_CORNER     = '+',
};

static inline void draw_horizontal_edges(WINDOW *w, int cols,
                                          int top_row, int bottom_row) {
    for (int c = 0; c < cols; c++) {
        mvwaddch(w, top_row,    c, BORDER_GLYPH_HORIZONTAL);
        mvwaddch(w, bottom_row, c, BORDER_GLYPH_HORIZONTAL);
    }
}

/* Sides only — leaves the corner rows for draw_corners. */
static inline void draw_vertical_edges(WINDOW *w, int cols,
                                        int top_row, int bottom_row) {
    int right_col = cols - 1;
    for (int r = top_row + 1; r < bottom_row; r++) {
        mvwaddch(w, r, 0,         BORDER_GLYPH_VERTICAL);
        mvwaddch(w, r, right_col, BORDER_GLYPH_VERTICAL);
    }
}

/* The four '+' corners, drawn last so they sit over the edges. */
static inline void draw_corners(WINDOW *w, int cols,
                                 int top_row, int bottom_row) {
    int right_col = cols - 1;
    mvwaddch(w, top_row,    0,         BORDER_GLYPH_CORNER);
    mvwaddch(w, top_row,    right_col, BORDER_GLYPH_CORNER);
    mvwaddch(w, bottom_row, 0,         BORDER_GLYPH_CORNER);
    mvwaddch(w, bottom_row, right_col, BORDER_GLYPH_CORNER);
}

/* Draw the frame, after the drops, so it always shows on top. */
static void render_border(WINDOW *w, int cols, int phys_area_rows) {
    enum { TOP_ROW = 0 };
    int bottom_row = phys_area_rows - 1;

    wattron(w, COLOR_PAIR(PAIR_BORDER));
    draw_horizontal_edges(w, cols, TOP_ROW, bottom_row);
    draw_vertical_edges  (w, cols, TOP_ROW, bottom_row);
    draw_corners         (w, cols, TOP_ROW, bottom_row);
    wattroff(w, COLOR_PAIR(PAIR_BORDER));
}

/* ── §17  HUD — status line on top, key hints on the bottom ── */

static void hud_paint_status(WINDOW *w, int cols, double fps_display,
                             int sim_hz, const Scene *s) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%3dHz  scene:%-9s  n:%4d  "
           "g:%s  v:%s  theme:%-7s  %s ",
           fps_display, sim_hz, scene_name_of(s->active_id), g_particle_pool.count,
           s->sim.gravity_enabled ? "ON" : "off", s->sim.viscosity_enabled ? "ON" : "off",
           color_theme_table[s->theme_index].name,
           s->sim.paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = cols - len;
  if (x < 0)
    x = 0;
  wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwprintw(w, 0, x, "%s", buf);
  wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(WINDOW *w, int rows) {
  const char *hint = " q:quit  spc:pause  1-5:scene  g:grav  v:visc  r:reset  "
                     "b:blob  t:theme  ]/[:simHz ";
  wattron(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(w, rows - 1, 0, "%s", hint);
  wattroff(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §18  screen — start, stop, and draw the terminal ── */

/*
 * Screen — just the terminal's current width and height.
 *
 * World is the fluid's play area; Screen is the whole terminal it's
 * drawn on.  The play area is the screen minus the HUD rows at the
 * bottom, so the HUD never gets painted over.  Kept apart from World so
 * a resize updates this first and the play area follows.
 *
 *   cols, rows   terminal size in cells (from getmaxyx)
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s, int theme_index) {
  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init(theme_index);
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_present_frame(Screen *s, const Scene *sc, double fps_display,
                                 int sim_hz) {
  erase();
  int phys_area_rows = s->rows - HUD_RESERVED_ROWS;
  render_particles(stdscr, s->cols, phys_area_rows);
  render_border(stdscr, s->cols, phys_area_rows);
  hud_paint_status(stdscr, s->cols, fps_display, sim_hz, sc);
  hud_paint_hint(stdscr, s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §19  app — the main loop, signals, and keys ── */

/*
 * FpsCounter — a steady frame-rate reading for the HUD.
 *
 * A raw per-frame fps jumps around too much to read, so we count frames
 * over a half-second window and show the average, refreshing it each
 * time the window fills.
 *
 *   frame_count   frames seen so far this window
 *   window_ns     time piled up this window, in nanoseconds
 *   display       the smoothed fps the HUD shows
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
  const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;     /* 500 ms */
  f->frame_count++;
  f->window_ns += dt;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — everything the program holds onto, as one global.
 *
 * It's a global because the signal handlers below need to reach it, and
 * a signal handler can only safely poke the two sig_atomic_t flags (the
 * volatile keeps the main loop re-reading them after a signal lands).
 * sim_hz lives here, not in Scene, because it's about loop timing, which
 * the simulation itself doesn't care about.
 *
 *   scene        the simulation
 *   screen       the terminal size
 *   fps          the frame-rate reading
 *   sim_hz       physics steps per second ([ and ] adjust it)
 *   running      cleared on Ctrl-C / kill to stop the loop
 *   need_resize  set on a window resize so the loop re-reads the size
 */
typedef struct {
    Scene      scene;
    Screen     screen;
    FpsCounter fps;
    int        sim_hz;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal_quit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_signal_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}

/* One small function per key, so the key switch below reads as a plain
 * list of "key -> action". */

/* Drop a little extra blob near the top, kept off the side walls. */
static void spawn_random_top_blob(const Scene *s) {
  enum { TOP_BLOB_RADIUS = 3, EDGE_INSET = 3, TOP_ROW = 3 };
  particle_spawn_blob(rand_in_range(EDGE_INSET, s->world.width - EDGE_INSET),
                      TOP_ROW, TOP_BLOB_RADIUS);
}

/* Switch to the next colour theme. */
static void cycle_theme(Scene *s) {
  s->theme_index = (s->theme_index + 1) % THEME_COUNT;
  colors_apply_theme(s->theme_index);
}

/* Speed up / slow down the physics, kept within sane bounds.  The main
 * loop notices the new rate on its next pass. */
static void adjust_sim_hz(App *app, int delta) {
  int next = app->sim_hz + delta;
  if (next < SIM_HZ_MIN) next = SIM_HZ_MIN;
  if (next > SIM_HZ_MAX) next = SIM_HZ_MAX;
  app->sim_hz = next;
}

/* Load one of the five scenes. */
static void select_scene_preset(Scene *s, int new_id) {
  s->active_id = new_id;
  scene_load_by_id(s, new_id);
}

/* Handle one keypress; return false only when it's time to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q': case 'Q': case 27 /* ESC */: return false;
  case ' ':           s->sim.paused = !s->sim.paused;                break;

  case '1': case '2': case '3': case '4': case '5':
                      select_scene_preset(s, ch - '0');              break;

  case 'g': case 'G': s->sim.gravity_enabled   = !s->sim.gravity_enabled;   break;
  case 'v': case 'V': s->sim.viscosity_enabled = !s->sim.viscosity_enabled; break;

  case 'r': case 'R': scene_load_by_id(s, s->active_id);             break;
  case 'b': case 'B': spawn_random_top_blob(s);                      break;
  case 't': case 'T': cycle_theme(s);                                break;

  case ']':           adjust_sim_hz(app, +SIM_HZ_STEP);              break;
  case '[':           adjust_sim_hz(app, -SIM_HZ_STEP);              break;

  default: break;
  }
  return true;
}

/* ── §20  main — set up, then run the loop ── */

/*
 * Each pass of the loop: handle a resize, measure how long the last
 * frame took, run as many fixed-size physics steps as that time bought,
 * draw, update the fps reading, read keys, then sleep to hold the frame
 * rate steady.  Stepping the physics at a fixed size (instead of by the
 * real frame time) keeps the simulation behaving the same on any
 * machine.
 */
int main(void) {
  srand((unsigned int)(clock_now_ns() & 0xFFFFFFFF));
  atexit(screen_cleanup);
  signal(SIGINT,   on_signal_quit);
  signal(SIGTERM,  on_signal_quit);
  signal(SIGWINCH, on_signal_resize);

  App *app     = &g_app;
  app->running = 1;
  app->sim_hz  = SIM_HZ_DEFAULT;
  fps_counter_init(&app->fps);

  screen_init(&app->screen, 0);
  scene_init (&app->scene, app->screen.cols, app->screen.rows);

  const int64_t DT_CAP_NS       = 100 * NS_PER_MS;          /* cap a long stall */
  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / RENDER_FPS_CAP;

  int64_t prev_ns      = clock_now_ns();
  int64_t sim_accum_ns = 0;

  while (app->running) {
    int64_t frame_start = clock_now_ns();

    /* (1) window resized? re-read the size */
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->scene.world.width  = app->screen.cols;
      app->scene.world.height = app->screen.rows - HUD_RESERVED_ROWS;
      sim_accum_ns     = 0;
      app->need_resize = 0;
    }

    /* (2) how long since last frame? (capped, so one stall can't snowball) */
    int64_t dt_ns = frame_start - prev_ns;
    prev_ns       = frame_start;
    if (dt_ns > DT_CAP_NS) dt_ns = DT_CAP_NS;

    /* (3) run fixed-size physics steps until we've used up that time */
    const int64_t TICK_LEN_NS = TICK_NS(app->sim_hz);
    sim_accum_ns += dt_ns;
    while (sim_accum_ns >= TICK_LEN_NS) {
      scene_tick(&app->scene, app->screen.cols, app->screen.rows);
      sim_accum_ns -= TICK_LEN_NS;
    }

    /* (4) draw the frame */
    screen_present_frame(&app->screen, &app->scene,
                         app->fps.display, app->sim_hz);

    /* (5) update the fps reading */
    fps_counter_tick(&app->fps, dt_ns);

    /* (6) handle any keys waiting */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    /* (7) sleep out the rest of the frame to hold a steady rate */
    int64_t spent = clock_now_ns() - frame_start;
    if (spent < FRAME_BUDGET_NS)
      clock_sleep_ns(FRAME_BUDGET_NS - spent);
  }

  return 0;
}
