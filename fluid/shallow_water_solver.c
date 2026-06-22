/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shallow_water_solver.c — ripples on a thin sheet of water, in the
 * terminal.  Each tick we nudge the flow toward lower water, then let
 * that flow raise or lower the surface; together that makes waves that
 * spread, reflect, and bend around obstacles.  Four scenes (dam break,
 * a drop, a channel, an obstacle), three colour looks, three edge styles.
 *
 * The model is the linearised 2-D shallow-water equations; the original
 * 1-D form is Saint-Venant (1871).  Sister demos: fluid/navier_stokes.c
 * (full incompressible flow) and fluid/cfl_stability_explorer.c (the
 * timestep-stability limit this solver lives inside).
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

/* §1  config — every tunable constant + enums */

/* frame timing */

#define SIM_HZ_DEFAULT 60
#define SIM_HZ_MIN 5
#define SIM_HZ_MAX 120
#define SIM_HZ_STEP 5

#define RENDER_FPS_CAP 60
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* grid size limits */

#define GRID_COLS_MAX 300
#define GRID_ROWS_MAX 100

/* physics */

/* Resting water depth.  Kept at 1 so wave speed depends on gravity alone. */
#define REST_DEPTH 1.0f

/* Gravity sets how fast waves travel (speed = sqrt(gravity), since depth
 * is 1).  Bigger gravity = faster waves; push it too high and the sim
 * goes unstable (see the CFL readout). */
#define GRAVITY_DEFAULT 400.0f
#define GRAVITY_MIN 25.0f
#define GRAVITY_MAX 1600.0f
#define GRAVITY_STEP 100.0f

/* Drag that slowly bleeds the motion away, like friction on the bottom. */
#define BOTTOM_FRICTION_DEFAULT 0.004f
#define BOTTOM_FRICTION_MIN 0.000f
#define BOTTOM_FRICTION_MAX 0.060f

/* how big the disturbances we inject are */

#define DROP_HEIGHT_AMPLITUDE 1.4f
#define DROP_GAUSSIAN_RADIUS 3.0f
#define DAM_BREAK_AMPLITUDE 1.2f

/* drawing thresholds */

/* Heights past this are drawn at full brightness (everything above is clipped). */
#define DISPLAY_HEIGHT_RANGE 1.8f

/* Cells slower than this don't get a flow arrow. */
#define ARROW_SPEED_THRESHOLD 0.05f

/* Heights this close to zero count as "at the waterline" for the shoreline overlay. */
#define SHORELINE_HEIGHT_THRESHOLD 0.04f

/* Stability readout colour bands: below STABLE = green, below MARGINAL =
 * yellow, above = red (the sim is near or past blowing up). */
#define CFL_STABLE_LIMIT 0.50f
#define CFL_MARGINAL_LIMIT 0.70f

/* what happens to waves at the edge of the grid */

enum {
  BC_WALL = 0,     /* solid wall: waves bounce back            */
  BC_OPEN = 1,     /* waves slide out and fade                 */
  BC_PERIODIC = 2, /* edges wrap around to the opposite side   */
  BC_COUNT,
};

static const char *bc_name_table[BC_COUNT] = {"wall    ", "open    ",
                                              "periodic"};

/* the four scenes */

enum {
  PRESET_DAM_BREAK = 0,
  PRESET_RADIAL_DROP = 1,
  PRESET_CHANNEL = 2,
  PRESET_OBSTACLE = 3,
  PRESET_COUNT,
};

static const char *preset_name_table[PRESET_COUNT] = {
    "dam_break  ", "radial_drop", "channel    ", "obstacle   "};

/* how many brightness tiers in the glyph ramp, and how many colour looks */

#define RAMP_SLOT_COUNT 9 /* must match the glyph + colour tables below */
#define THEME_COUNT 3

/* colour-pair numbers.  Each theme gets a full crest ramp (POS) and a
 * trough ramp (NEG); the shoreline / obstacle / HUD / hint pairs follow. */

#define PAIR_POS(theme, slot) (1 + (theme) * (RAMP_SLOT_COUNT * 2) + (slot))
#define PAIR_NEG(theme, slot)                                                  \
  (1 + (theme) * (RAMP_SLOT_COUNT * 2) + RAMP_SLOT_COUNT + (slot))
#define PAIR_SHORELINE (1 + THEME_COUNT * (RAMP_SLOT_COUNT * 2))
#define PAIR_OBSTACLE (PAIR_SHORELINE + 1)
#define PAIR_HUD (PAIR_OBSTACLE + 1)
#define PAIR_HINT (PAIR_HUD + 1)

/* §2  clock — steady nanosecond timer + sleep */

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

/* §3  themes — the colour looks */

/*
 * One colour look for the water.  Height is signed — crests rise above
 * the resting level, troughs sink below — so each theme carries TWO
 * colour ramps: one for crests, one for troughs.  Reading them as
 * opposing hues lets the eye tell crests from troughs at a glance.
 *
 * Each ramp goes from faint (slot 0, barely-there ripple) to vivid
 * (last slot, biggest crest or trough).
 *
 * There are two copies of each ramp: a rich 256-colour version and a
 * plain 8-colour fallback.  We pick one at startup based on what the
 * terminal supports, so the drawing code never has to branch later.
 */
typedef struct {
    const char *display_name;               /* short label in the HUD       */
    short       pos_256[RAMP_SLOT_COUNT];   /* crest colours, 256-colour    */
    short       neg_256[RAMP_SLOT_COUNT];   /* trough colours, 256-colour   */
    short       pos_8  [RAMP_SLOT_COUNT];   /* crest colours, 8-colour      */
    short       neg_8  [RAMP_SLOT_COUNT];   /* trough colours, 8-colour     */
} Theme;

static const Theme theme_table[THEME_COUNT] = {
    /* 0  WATER — cyan / white crests, navy troughs. */
    {
        "water  ",
        /* pos: dark teal → cyan → white */
        {31, 37, 44, 51, 87, 123, 159, 195, 231},
        /* neg: navy → indigo */
        {18, 19, 20, 21, 25, 26, 27, 31, 39},
        {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
         COLOR_BLUE, COLOR_CYAN, COLOR_CYAN},
    },
    /* 1  MAGMA — red / yellow crests, dark violet troughs. */
    {
        "magma  ",
        /* pos: dark orange → amber → yellow → white */
        {88, 130, 166, 202, 208, 214, 220, 226, 231},
        /* neg: dark violet → indigo → blue */
        {53, 54, 55, 56, 57, 93, 99, 135, 141},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE,
         COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN},
    },
    /* 2  CURRENT — green / white crests, dark teal troughs. */
    {
        "current",
        /* pos: dark green → lime → white */
        {28, 34, 40, 46, 82, 118, 154, 191, 231},
        /* neg: dark teal */
        {22, 23, 29, 30, 36, 37, 44, 45, 51},
        {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN,
         COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN},
    },
};

/* §4  colors — register colour pairs, pick a pair for a height */

static bool terminal_has_256_colours = false;

static void colors_init(void) {
  start_color();
  use_default_colors();
  terminal_has_256_colours = (COLORS >= 256);

  for (int t = 0; t < THEME_COUNT; t++) {
    for (int s = 0; s < RAMP_SLOT_COUNT; s++) {
      short pos = terminal_has_256_colours ? theme_table[t].pos_256[s]
                                           : theme_table[t].pos_8[s];
      short neg = terminal_has_256_colours ? theme_table[t].neg_256[s]
                                           : theme_table[t].neg_8[s];
      init_pair((short)PAIR_POS(t, s), pos, -1);
      init_pair((short)PAIR_NEG(t, s), neg, -1);
    }
  }

  /* waterline marker + obstacle fill */
  if (terminal_has_256_colours) {
    init_pair(PAIR_SHORELINE, 51, -1); /* bright cyan  */
    init_pair(PAIR_OBSTACLE, 244, -1); /* mid grey     */
  } else {
    init_pair(PAIR_SHORELINE, COLOR_CYAN, -1);
    init_pair(PAIR_OBSTACLE, COLOR_WHITE, -1);
  }

  /* HUD text — bright so it stays readable over any animation */
  if (terminal_has_256_colours) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static attr_t height_pair_attr(int theme_index, bool positive, int slot) {
  int pair =
      positive ? PAIR_POS(theme_index, slot) : PAIR_NEG(theme_index, slot);
  attr_t a = COLOR_PAIR(pair);
  if (slot >= RAMP_SLOT_COUNT - 2)
    a |= A_BOLD; /* make the tallest crests/deepest troughs pop */
  return a;
}

/* §5  ramp — pick a character for a wave height */
/*
 * Characters from faint to dense.  Crests and troughs use the same
 * shapes; colour (warm vs cool) is what tells them apart.  Slot 0 is
 * blank (flat water); slots 1-8 actually draw.
 */

static const char ramp_glyph_table[RAMP_SLOT_COUNT] = {
    ' ', /* 0 — at rest                        */
    '.', /* 1 — tiny ripple                    */
    ':', /* 2 — gentle swell                   */
    '+', /* 3 — moderate wave                  */
    'x', /* 4 — strong wave                    */
    '*', /* 5 — intense surge                  */
    'X', /* 6 — near-crest                     */
    '#', /* 7 — high crest / deep trough       */
    '@', /* 8 — peak (clipped at DISPLAY_RANGE) */
};

static const float ramp_threshold_table[RAMP_SLOT_COUNT] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f, 0.500f, 0.660f, 0.820f, 0.940f,
};

/* Turn a 0..1 brightness into a ramp slot.  The gamma step spreads out
 * the faint end so small ripples are still visible. */
static int normalised_height_to_slot(float normalised) {
  if (normalised <= 0.0f)
    return 0;
  if (normalised >= 1.0f)
    return RAMP_SLOT_COUNT - 1;
  float gamma_corrected = powf(normalised, 1.0f / 2.2f);
  for (int i = RAMP_SLOT_COUNT - 1; i >= 0; i--)
    if (gamma_corrected >= ramp_threshold_table[i])
      return i;
  return 0;
}

/* §6  grid_state — the water fields */
/*
 * The whole simulation is four grids, indexed [row][col] with row 0 at
 * the top of the screen:
 *   height_perturbation — how far the surface is above (or below) rest
 *   velocity_x          — how fast water is moving sideways
 *   velocity_y          — how fast water is moving up/down the screen
 *   wall_mask           — true where a solid obstacle sits
 * They're fixed-size and statically allocated, so there's no malloc.
 */

static float height_perturbation[GRID_ROWS_MAX][GRID_COLS_MAX];
static float velocity_x[GRID_ROWS_MAX][GRID_COLS_MAX];
static float velocity_y[GRID_ROWS_MAX][GRID_COLS_MAX];
static bool wall_mask[GRID_ROWS_MAX][GRID_COLS_MAX];

static int grid_active_rows = 0;
static int grid_active_cols = 0;

/* Reset the water to flat and still (leaves the obstacles in place). */
static void grid_zero_fluid_fields(void) {
  for (int r = 0; r < grid_active_rows; r++) {
    memset(height_perturbation[r], 0, (size_t)grid_active_cols * sizeof(float));
    memset(velocity_x[r], 0, (size_t)grid_active_cols * sizeof(float));
    memset(velocity_y[r], 0, (size_t)grid_active_cols * sizeof(float));
  }
}

static void grid_clear_walls(void) {
  for (int r = 0; r < grid_active_rows; r++)
    memset(wall_mask[r], 0, (size_t)grid_active_cols * sizeof(bool));
}

/* §7  boundary — what waves do at the grid edges */
/*
 * Run after every update.  Three styles:
 *   BC_WALL      waves bounce off a solid wall (no flow through it)
 *   BC_OPEN      waves slide out and fade
 *   BC_PERIODIC  waves wrap around to the opposite edge
 */

/* Wall on the top and bottom rows: stop water flowing through (set the
 * up/down velocity to zero), and copy the sideways velocity and the
 * height inward so the wave reflects cleanly.  [1] Stoker. */
static inline void bc_wall_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        velocity_y         [0       ][c] = 0.0f;
        velocity_y         [rows - 1][c] = 0.0f;
        velocity_x         [0       ][c] = velocity_x         [1       ][c];
        velocity_x         [rows - 1][c] = velocity_x         [rows - 2][c];
        height_perturbation[0       ][c] = height_perturbation[1       ][c];
        height_perturbation[rows - 1][c] = height_perturbation[rows - 2][c];
    }
}

/* Wall on the left and right columns: same idea, but now the sideways
 * velocity is the one that can't pass through. */
static inline void bc_wall_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        velocity_x         [r][0       ] = 0.0f;
        velocity_x         [r][cols - 1] = 0.0f;
        velocity_y         [r][0       ] = velocity_y         [r][1       ];
        velocity_y         [r][cols - 1] = velocity_y         [r][cols - 2];
        height_perturbation[r][0       ] = height_perturbation[r][1       ];
        height_perturbation[r][cols - 1] = height_perturbation[r][cols - 2];
    }
}

/* Open top/bottom: each edge row just copies its inside neighbour, so
 * waves mostly slide out.  It's a cheap trick, not perfect — a little
 * still bounces back — but it looks right. */
static inline void bc_open_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        height_perturbation[0       ][c] = height_perturbation[1       ][c];
        velocity_x         [0       ][c] = velocity_x         [1       ][c];
        velocity_y         [0       ][c] = velocity_y         [1       ][c];
        height_perturbation[rows - 1][c] = height_perturbation[rows - 2][c];
        velocity_x         [rows - 1][c] = velocity_x         [rows - 2][c];
        velocity_y         [rows - 1][c] = velocity_y         [rows - 2][c];
    }
}

/* Open left/right — same as the top/bottom version, other axis. */
static inline void bc_open_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        height_perturbation[r][0       ] = height_perturbation[r][1       ];
        velocity_x         [r][0       ] = velocity_x         [r][1       ];
        velocity_y         [r][0       ] = velocity_y         [r][1       ];
        height_perturbation[r][cols - 1] = height_perturbation[r][cols - 2];
        velocity_x         [r][cols - 1] = velocity_x         [r][cols - 2];
        velocity_y         [r][cols - 1] = velocity_y         [r][cols - 2];
    }
}

/* Wrap-around top/bottom: the top edge copies from the bottom and vice
 * versa, so a wave leaving one side re-enters the other.  Nothing is lost. */
static inline void bc_periodic_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        height_perturbation[0       ][c] = height_perturbation[rows - 2][c];
        velocity_x         [0       ][c] = velocity_x         [rows - 2][c];
        velocity_y         [0       ][c] = velocity_y         [rows - 2][c];
        height_perturbation[rows - 1][c] = height_perturbation[1       ][c];
        velocity_x         [rows - 1][c] = velocity_x         [1       ][c];
        velocity_y         [rows - 1][c] = velocity_y         [1       ][c];
    }
}

/* Wrap-around left/right — same as top/bottom, other axis. */
static inline void bc_periodic_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        height_perturbation[r][0       ] = height_perturbation[r][cols - 2];
        velocity_x         [r][0       ] = velocity_x         [r][cols - 2];
        velocity_y         [r][0       ] = velocity_y         [r][cols - 2];
        height_perturbation[r][cols - 1] = height_perturbation[r][1       ];
        velocity_x         [r][cols - 1] = velocity_x         [r][1       ];
        velocity_y         [r][cols - 1] = velocity_y         [r][1       ];
    }
}

/* Run the chosen edge style on all four sides. */
static void apply_boundary(int boundary_kind) {
    int rows = grid_active_rows;
    int cols = grid_active_cols;

    switch (boundary_kind) {
        case BC_WALL:     bc_wall_top_bottom    (rows, cols);
                          bc_wall_left_right    (rows, cols); break;
        case BC_OPEN:     bc_open_top_bottom    (rows, cols);
                          bc_open_left_right    (rows, cols); break;
        case BC_PERIODIC: bc_periodic_top_bottom(rows, cols);
                          bc_periodic_left_right(rows, cols); break;
        default:          break;
    }
}

/* §8  obstacles — solid shapes the water flows around */

/* Force the water to rest inside every obstacle cell.  Doing this each
 * tick makes obstacles behave like little walls that waves bounce off. */
static void zero_fields_in_walls(void) {
  for (int r = 0; r < grid_active_rows; r++)
    for (int c = 0; c < grid_active_cols; c++)
      if (wall_mask[r][c]) {
        height_perturbation[r][c] = 0.0f;
        velocity_x[r][c] = 0.0f;
        velocity_y[r][c] = 0.0f;
      }
}

/* Build two horizontal walls with a gap in the middle, so waves are
 * funnelled through a narrow slot. */
static void obstacle_build_channel(void) {
  int wall_top = grid_active_rows / 3;
  int wall_bottom = (grid_active_rows * 2) / 3;
  int left_col = grid_active_cols / 4;
  int right_col = (grid_active_cols * 3) / 4;

  for (int r = 0; r < grid_active_rows; r++) {
    if (r < wall_top || r > wall_bottom) {
      for (int c = left_col; c <= right_col; c++)
        wall_mask[r][c] = true;
    }
  }
}

/* Build a solid round disc in the middle for waves to scatter off. */
static void obstacle_build_circle(void) {
  float cx = (float)(grid_active_cols - 1) * 0.5f;
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float radius = (grid_active_rows < grid_active_cols ? grid_active_rows
                                                      : grid_active_cols) *
                 0.12f;
  float radius_squared = radius * radius;

  for (int r = 0; r < grid_active_rows; r++) {
    for (int c = 0; c < grid_active_cols; c++) {
      float dx = (float)c - cx;
      float dy = (float)r - cy;
      if (dx * dx + dy * dy < radius_squared)
        wall_mask[r][c] = true;
    }
  }
}

/* §9  update_velocity — let the surface slope push the water */
/*
 * Phase one of a tick.  Water gets pushed toward wherever the surface
 * is lower, the way a ball rolls downhill.  We measure how much the
 * height rises to the right and below each cell, and shove the flow the
 * opposite way.  Reads the height (left untouched here) and writes the
 * two velocity fields.
 */
/* How fast the surface rises as you step right and as you step down,
 * measured at cell (r, c).  We compare this cell with its right and
 * lower neighbour. */
static inline void forward_height_gradient_at(int r, int c,
                                               float *out_dh_dx,
                                               float *out_dh_dy) {
    *out_dh_dx = height_perturbation[r    ][c + 1] - height_perturbation[r][c];
    *out_dh_dy = height_perturbation[r + 1][c    ] - height_perturbation[r][c];
}

/* The number we multiply velocity by each tick to bleed off a little
 * motion (friction).  Just under 1.0; clamped at 0 so a big friction
 * value can't flip the flow backwards. */
static inline float bottom_friction_damping_factor(float bottom_friction_coeff,
                                                    float dt_seconds) {
    float damping = 1.0f - bottom_friction_coeff * dt_seconds;
    return damping < 0.0f ? 0.0f : damping;
}

/* Update one cell's flow: push it downhill (scaled by gravity and the
 * timestep), then apply friction.  Obstacle cells are skipped. */
static inline void update_velocity_at_cell(int r, int c,
                                            float gravity_acceleration,
                                            float dt_seconds,
                                            float damping_factor) {
    if (wall_mask[r][c]) return;
    float dh_dx, dh_dy;
    forward_height_gradient_at(r, c, &dh_dx, &dh_dy);
    velocity_x[r][c] -= gravity_acceleration * dt_seconds * dh_dx;
    velocity_y[r][c] -= gravity_acceleration * dt_seconds * dh_dy;
    velocity_x[r][c] *= damping_factor;
    velocity_y[r][c] *= damping_factor;
}

/* Push every interior cell's flow downhill, then damp it. */
static void update_velocity(float gravity_acceleration,
                             float bottom_friction_coeff, float dt_seconds) {
    float damping_factor = bottom_friction_damping_factor(bottom_friction_coeff,
                                                          dt_seconds);
    int   rows = grid_active_rows;
    int   cols = grid_active_cols;

    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            update_velocity_at_cell(r, c, gravity_acceleration, dt_seconds,
                                     damping_factor);
}

/* §10  update_height — raise or lower the surface as water moves */
/*
 * Phase two of a tick.  Wherever more water flows out of a cell than
 * into it, the surface there drops; where more flows in, it rises.
 * Reads the velocities (just set in phase one) and writes the height.
 *
 * This has to be a SEPARATE pass from update_velocity — if you merged
 * them, a cell would read its neighbour's brand-new velocity instead of
 * the old one and the numbers would fall apart.
 */
/* Net outflow at cell (r, c): how much faster water is leaving to the
 * right and below than arriving from the left and above.  The
 * "previous cell" subtraction here pairs with the "next cell"
 * subtraction in update_velocity — together they keep the scheme
 * stable.  [5] Arakawa & Lamb 1977. */
static inline float backward_velocity_divergence_at(int r, int c) {
    float divergence_x = velocity_x[r    ][c] - velocity_x[r    ][c - 1];
    float divergence_y = velocity_y[r    ][c] - velocity_y[r - 1][c    ];
    return divergence_x + divergence_y;
}

/* Drop one cell's surface in proportion to its net outflow (or raise it
 * if water is piling in).  Obstacle cells are skipped. */
static inline void update_height_at_cell(int r, int c, float dt_seconds) {
    if (wall_mask[r][c]) return;
    float divergence = backward_velocity_divergence_at(r, c);
    height_perturbation[r][c] -= REST_DEPTH * divergence * dt_seconds;
}

/* Raise or lower every interior cell's surface from how the water moved.
 * Must run as its own pass, after update_velocity has finished. */
static void update_height(float dt_seconds) {
    int rows = grid_active_rows;
    int cols = grid_active_cols;

    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            update_height_at_cell(r, c, dt_seconds);
}

/* §11  stats — live numbers for the HUD */

/*
 * A snapshot of "how's the simulation doing right now," rebuilt once per
 * frame and handed to the HUD so it doesn't re-scan the grid itself.
 *
 *   max_abs_height — the tallest crest/deepest trough; "how big are the waves"
 *   average_speed  — how much water is moving overall
 *   wave_speed     — how fast waves travel, set by gravity
 *   cfl_number     — a stability gauge.  It's the wave speed times the
 *                    timestep; as it nears 1 the explicit scheme is about
 *                    to blow up, so the HUD turns it red.  This is the
 *                    classic Courant limit, [4] LeVeque.
 */
typedef struct {
    float max_abs_height;   /* biggest crest or trough this frame    */
    float average_speed;    /* average flow speed over the grid      */
    float wave_speed;       /* how fast waves travel                 */
    float cfl_number;       /* stability gauge (near 1 = trouble)    */
} SimStats;

/* Flow speed at one cell (combining the sideways and up/down parts). */
static inline float velocity_magnitude_at(int r, int c) {
    return sqrtf(velocity_x[r][c] * velocity_x[r][c] +
                 velocity_y[r][c] * velocity_y[r][c]);
}

/* One sweep over the water cells to find the biggest wave and the
 * average flow speed.  Obstacle cells are skipped (they're held at zero
 * and would drag the average down). */
static inline void scan_max_height_and_average_speed(float *out_max_h,
                                                      float *out_avg_speed) {
    float  max_h     = 0.0f;
    double speed_sum = 0.0;
    int    counted   = 0;

    int rows = grid_active_rows;
    int cols = grid_active_cols;
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (wall_mask[r][c]) continue;
            float ah = fabsf(height_perturbation[r][c]);
            if (ah > max_h) max_h = ah;
            speed_sum += (double)velocity_magnitude_at(r, c);
            counted++;
        }
    }

    *out_max_h     = max_h;
    *out_avg_speed = (counted > 0) ? (float)(speed_sum / counted) : 0.0f;
}

/* How fast waves travel, which depends only on gravity (depth is 1). */
static inline float swe_wave_speed(float gravity_acceleration) {
    return sqrtf(gravity_acceleration * REST_DEPTH);
}

/* The stability gauge: how far a wave moves in one timestep.  Once it
 * gets near 1 the simulation is on the edge of blowing up. */
static inline float swe_cfl_number(float wave_speed, float dt_seconds) {
    return wave_speed * dt_seconds;
}

/* Fill in all four HUD numbers. */
static void compute_sim_stats(float gravity_acceleration, float dt_seconds,
                              SimStats *out_stats) {
    scan_max_height_and_average_speed(&out_stats->max_abs_height,
                                       &out_stats->average_speed);
    out_stats->wave_speed = swe_wave_speed(gravity_acceleration);
    out_stats->cfl_number = swe_cfl_number(out_stats->wave_speed, dt_seconds);
}

/* §12  swe_step — one full tick of the simulation */
/*
 * The order matters: push the water (§9), let that reshape the surface
 * (§10), hold obstacles still (§8), fix up the edges (§7), then read off
 * the HUD numbers (§11).
 */

static void swe_step(int boundary_kind, float gravity_acceleration,
                     float bottom_friction_coeff, float dt_seconds,
                     SimStats *out_stats) {
  update_velocity(gravity_acceleration, bottom_friction_coeff, dt_seconds);
  update_height(dt_seconds);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
  compute_sim_stats(gravity_acceleration, dt_seconds, out_stats);
}

/* §13  excitation — ways to poke the water */

/* A smooth bump that's tallest at the centre and fades with distance —
 * the shape of a single drop landing.  The caller hands in 1/(2σ²)
 * already worked out so we don't redo it for every cell. */
static inline float gaussian_value(float dx_sq, float dy_sq, float amplitude,
                                    float inv_two_sigma_sq) {
    return amplitude * expf(-(dx_sq + dy_sq) * inv_two_sigma_sq);
}

/* Bump one cell's surface up by the drop shape.  Obstacle cells are
 * skipped, since they're held flat and bumping them would spit out a
 * fake wave next tick. */
static inline void add_gaussian_pulse_to_cell(int r, int c,
                                               float cx, float cy,
                                               float amplitude,
                                               float inv_two_sigma_sq) {
    if (wall_mask[r][c]) return;
    float dx    = (float)c - cx;
    float dy    = (float)r - cy;
    height_perturbation[r][c] +=
        gaussian_value(dx * dx, dy * dy, amplitude, inv_two_sigma_sq);
}

/* Drop a smooth bump onto the surface at (cx, cy).  We only lift the
 * water — the flow stays still — so next tick it collapses and sends a
 * ring spreading outward, just like a real drop. */
static void apply_gaussian_drop(float cx, float cy, float amplitude,
                                 float sigma) {
    float inv_two_sigma_sq = 1.0f / (2.0f * sigma * sigma);
    for (int r = 0; r < grid_active_rows; r++)
        for (int c = 0; c < grid_active_cols; c++)
            add_gaussian_pulse_to_cell(r, c, cx, cy, amplitude,
                                        inv_two_sigma_sq);
}

/* Set the left half high and the right half low, like a dam wall about
 * to burst.  The moment it runs, water rushes from the high side. */
static void apply_dam_break_initial_state(void) {
  int half_col = grid_active_cols / 2;
  for (int r = 0; r < grid_active_rows; r++) {
    for (int c = 0; c < grid_active_cols; c++) {
      if (wall_mask[r][c])
        continue;
      height_perturbation[r][c] =
          (c < half_col) ? DAM_BREAK_AMPLITUDE : -DAM_BREAK_AMPLITUDE;
    }
  }
}

/* §14  presets — the four scenes */

/*
 * One ready-made scene.  Each shows off a different bit of wave physics:
 *   dam_break   — a high/low step bursts and a front races across
 *   radial_drop — one drop spreads as a clean ring
 *   channel     — a wave squeezes through a gap and fans out (diffraction)
 *   obstacle    — a wave scatters off a disc, leaving a shadow behind it
 *
 * Each scene is a little function (the loader) that paints its own
 * starting water and walls.  We use a function pointer rather than a
 * table of numbers because the scenes differ in shape, not in a few
 * settings.  The loader is handed the current edge style so it can fix
 * up the borders right after laying down its initial water.
 * [1] Stoker, dam-break problem.
 */
typedef struct {
    const char *display_name;                  /* short label in the HUD */
    void      (*loader)(int boundary_kind);    /* paints the starting scene */
} Preset;

static void preset_dam_break(int boundary_kind);
static void preset_radial_drop(int boundary_kind);
static void preset_channel(int boundary_kind);
static void preset_obstacle(int boundary_kind);

static const Preset preset_table[PRESET_COUNT] = {
    {"dam_break  ", preset_dam_break},
    {"radial_drop", preset_radial_drop},
    {"channel    ", preset_channel},
    {"obstacle   ", preset_obstacle},
};

static void preset_dam_break(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  apply_dam_break_initial_state();
  apply_boundary(boundary_kind);
}

static void preset_radial_drop(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  float cx = (float)(grid_active_cols - 1) * 0.5f;
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  apply_gaussian_drop(cx, cy, DROP_HEIGHT_AMPLITUDE, DROP_GAUSSIAN_RADIUS);
  apply_boundary(boundary_kind);
}

static void preset_channel(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  obstacle_build_channel();
  /* Start the wave on the left so it heads into the gap. */
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float cx_source = (float)(grid_active_cols - 1) * 0.20f;
  apply_gaussian_drop(cx_source, cy, DROP_HEIGHT_AMPLITUDE,
                      DROP_GAUSSIAN_RADIUS);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
}

static void preset_obstacle(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  obstacle_build_circle();
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float cx_source = (float)(grid_active_cols - 1) * 0.15f;
  apply_gaussian_drop(cx_source, cy, DROP_HEIGHT_AMPLITUDE,
                      DROP_GAUSSIAN_RADIUS);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
}

/* §15  shoreline — finding the waterline */
/*
 * The "shoreline" is the line where the surface crosses from a crest to
 * a trough — the calm spot right between an up-wave and a down-wave.  We
 * mark a cell as shoreline when its own height is near zero AND it has a
 * crest on one neighbouring side and a trough on another.  Demanding
 * both sides weeds out plain flat water (also near zero) and the calm
 * edges of obstacles.
 */

/* Is (r, c) on the grid and not an obstacle?  Obstacle cells are held
 * at zero height, so without excluding them every cell next to a wall
 * would look like a waterline. */
static inline bool neighbour_is_water_cell(int r, int c) {
    return r >= 0 && r < grid_active_rows
        && c >= 0 && c < grid_active_cols
        && !wall_mask[r][c];
}

/* Look at one neighbour: if it's clearly a crest, note "saw a crest";
 * if clearly a trough, note "saw a trough".  The caller runs this on all
 * four neighbours through the same pair of flags. */
static inline void inspect_neighbour_for_polarities(int r, int c,
                                                     bool *seen_positive,
                                                     bool *seen_negative) {
    if (!neighbour_is_water_cell(r, c)) return;
    float h = height_perturbation[r][c];
    if (h >  SHORELINE_HEIGHT_THRESHOLD) *seen_positive = true;
    if (h < -SHORELINE_HEIGHT_THRESHOLD) *seen_negative = true;
}

/* True if (r, c) sits on the waterline: a crest on one side, a trough on
 * another.  Checks the four neighbours up/down/left/right. */
static bool is_shoreline_cell(int r, int c) {
    bool seen_positive = false;
    bool seen_negative = false;

    inspect_neighbour_for_polarities(r - 1, c    , &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r + 1, c    , &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r    , c - 1, &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r    , c + 1, &seen_positive, &seen_negative);

    return seen_positive && seen_negative;
}

/* §16  render_field — draw the water as characters */

static const char arrow_glyph_table[8] = {'>', '/', 'v', '\\',
                                          '<', '/', '^', '\\'};

static int velocity_to_arrow_octant(float vx, float vy) {
  float angle = atan2f(vy, vx);
  if (angle < 0.0f)
    angle += 2.0f * (float)M_PI;
  return (int)(angle / ((float)M_PI * 0.25f)) % 8;
}

/* Draw an obstacle cell as a grey '#'. */
static inline void paint_obstacle_layer(WINDOW *w, int r, int c) {
    attr_t attr = COLOR_PAIR(PAIR_OBSTACLE);
    wattron(w, attr);
    mvwaddch(w, r, c, '#');
    wattroff(w, attr);
}

/* Draw a waterline cell as a bright '~'. */
static inline void paint_shoreline_layer(WINDOW *w, int r, int c) {
    attr_t attr = COLOR_PAIR(PAIR_SHORELINE) | A_BOLD;
    wattron(w, attr);
    mvwaddch(w, r, c, '~');
    wattroff(w, attr);
}

/* From a height, work out whether it's a crest (positive) and which
 * brightness tier to draw.  Tier 0 means too faint to bother — the
 * caller skips that cell. */
static inline int height_to_polarity_and_slot(float h, float inv_display_max,
                                                bool *out_positive) {
    *out_positive = (h >= 0.0f);
    float normalised = fabsf(h) * inv_display_max;
    if (normalised > 1.0f) normalised = 1.0f;
    return normalised_height_to_slot(normalised);
}

/* If this cell's water is moving fast enough, draw a little arrow
 * pointing which way it flows and return true (so the caller doesn't
 * also draw the height character).  The arrow keeps the height colour. */
static inline bool paint_arrow_layer_if_fast(WINDOW *w, int r, int c,
                                              attr_t height_attr) {
    float speed = velocity_magnitude_at(r, c);
    if (speed <= ARROW_SPEED_THRESHOLD) return false;
    int octant = velocity_to_arrow_octant(velocity_x[r][c], velocity_y[r][c]);
    wattron(w, height_attr);
    mvwaddch(w, r, c, (chtype)(unsigned char)arrow_glyph_table[octant]);
    wattroff(w, height_attr);
    return true;
}

/* Draw the height character for this cell at the given brightness tier. */
static inline void paint_height_layer(WINDOW *w, int r, int c, int slot,
                                       attr_t height_attr) {
    wattron(w, height_attr);
    mvwaddch(w, r, c, (chtype)(unsigned char)ramp_glyph_table[slot]);
    wattroff(w, height_attr);
}

/* Draw one water cell.  Each cell gets a single character, chosen by
 * priority: a waterline mark if it's right at the zero line, else a flow
 * arrow if it's moving fast, else the plain height shading. */
static inline void paint_one_cell_layered(WINDOW *w, int r, int c,
                                           int active_theme_index,
                                           bool show_arrows,
                                           bool show_shoreline,
                                           float inv_display_max) {
    float h = height_perturbation[r][c];

    /* Waterline mark — only when the surface is basically flat here. */
    if (show_shoreline && fabsf(h) < SHORELINE_HEIGHT_THRESHOLD) {
        if (is_shoreline_cell(r, c)) paint_shoreline_layer(w, r, c);
        return;
    }

    /* Otherwise: height shading, with a flow arrow on top if fast. */
    bool positive;
    int  slot = height_to_polarity_and_slot(h, inv_display_max, &positive);
    if (slot == 0) return;
    attr_t height_attr = height_pair_attr(active_theme_index, positive, slot);

    if (show_arrows && paint_arrow_layer_if_fast(w, r, c, height_attr)) return;
    paint_height_layer(w, r, c, slot, height_attr);
}

/* Draw the whole grid: obstacles first (if shown), then each water cell. */
static void render_field(WINDOW *w, int active_theme_index, bool show_arrows,
                          bool show_shoreline, bool show_obstacles) {
    float display_max     = DISPLAY_HEIGHT_RANGE;
    if (display_max < 0.05f) display_max = 0.05f;
    float inv_display_max = 1.0f / display_max;

    int rows = grid_active_rows;
    int cols = grid_active_cols;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (wall_mask[r][c]) {
                if (show_obstacles) paint_obstacle_layer(w, r, c);
                continue;
            }
            paint_one_cell_layered(w, r, c, active_theme_index,
                                    show_arrows, show_shoreline,
                                    inv_display_max);
        }
    }
}

/* §17  stats_panel — the box of live numbers in the corner */
/*
 * A small box drawn over the water showing the live readouts (biggest
 * wave, average speed, wave speed, stability, gravity, friction, edge
 * style, elapsed time, scene).  The stability line is coloured
 * green / yellow / red.
 */

/* Where the stats box sits on screen: top-left corner plus width and
 * height, all in character cells.  Worked out once per frame and passed
 * to the drawing code so it knows where each row goes. */
typedef struct {
    int ox;   /* left edge, in cells   */
    int oy;   /* top edge, in cells    */
    int pw;   /* width, in cells       */
    int ph;   /* height, in cells      */
} OverlayBox;

static OverlayBox stats_panel_layout(int term_rows, int term_cols) {
  OverlayBox box = {1, 0, 30, 13};
  box.oy = term_rows - box.ph - 1;
  if (box.oy < 0)
    box.oy = 0;
  if (box.ox + box.pw > term_cols)
    box.pw = term_cols - box.ox;
  return box;
}

/* Colour for the stability line: green / yellow / red by how close to
 * blowing up.  Pulls colours from theme 0 so they exist at startup. */
static int stats_cfl_pair(float cfl) {
  if (cfl < CFL_STABLE_LIMIT)
    return PAIR_NEG(0, 5); /* dim blue (green-ish) */
  if (cfl < CFL_MARGINAL_LIMIT)
    return PAIR_HUD;     /* yellow */
  return PAIR_POS(1, 7); /* magma red */
}

static const char *stats_cfl_label(float cfl) {
  if (cfl < CFL_STABLE_LIMIT)
    return "STABLE  ";
  if (cfl < CFL_MARGINAL_LIMIT)
    return "MARGINAL";
  return "UNSTABLE";
}

static void render_stats_panel(WINDOW *w, int term_rows, int term_cols,
                               const SimStats *stats,
                               float gravity_acceleration,
                               float bottom_friction_coeff,
                               int active_boundary_kind,
                               float simulation_time_seconds,
                               bool simulation_paused, bool show_arrows,
                               bool show_shoreline, int active_preset_index) {
  OverlayBox box = stats_panel_layout(term_rows, term_cols);
  if (box.pw < 20 || box.ph < 6)
    return; /* no room — skip the panel */

  int ox = box.ox;
  int oy = box.oy;

  /* Yellow + bold so the panel clearly reads as an overlay. */
  attr_t border_attr = COLOR_PAIR(PAIR_HUD) | A_BOLD;
  wattron(w, border_attr);
  mvwprintw(w, oy + 0, ox, "+--- SHALLOW WATER --------+");
  mvwprintw(w, oy + 1, ox, "| max_h   %8.4f           |",
            (double)stats->max_abs_height);
  mvwprintw(w, oy + 2, ox, "| avg_vel %8.4f           |",
            (double)stats->average_speed);
  mvwprintw(w, oy + 3, ox, "| wavespd %7.2f cells/s    |",
            (double)stats->wave_speed);

  mvwprintw(w, oy + 4, ox, "| CFL     ");
  wattroff(w, border_attr);
  {
    attr_t cfl_attr = COLOR_PAIR(stats_cfl_pair(stats->cfl_number)) | A_BOLD;
    wattron(w, cfl_attr);
    wprintw(w, "%5.3f %-8s", (double)stats->cfl_number,
            stats_cfl_label(stats->cfl_number));
    wattroff(w, cfl_attr);
  }
  wattron(w, border_attr);
  wprintw(w, "|");

  mvwprintw(w, oy + 5, ox, "| gravity %7.1f            |",
            (double)gravity_acceleration);
  mvwprintw(w, oy + 6, ox, "| damping %8.4f           |",
            (double)bottom_friction_coeff);
  mvwprintw(w, oy + 7, ox, "| BC      %s         |",
            bc_name_table[active_boundary_kind]);
  mvwprintw(w, oy + 8, ox, "| sim_t   %8.2f s          |",
            (double)simulation_time_seconds);
  mvwprintw(w, oy + 9, ox, "| preset  %s         |",
            preset_name_table[active_preset_index]);
  mvwprintw(w, oy + 10, ox, "| arrows  %-3s shore %-3s      |",
            show_arrows ? "ON " : "OFF", show_shoreline ? "ON " : "OFF");
  mvwprintw(w, oy + 11, ox, "| %s                       |",
            simulation_paused ? "PAUSED        " : "running       ");
  mvwprintw(w, oy + 12, ox, "+---------------------------+");
  wattroff(w, border_attr);
}

/* §18  hud — status line on top, key hints on the bottom */

static void hud_paint_status(int term_cols, double measured_fps,
                             int sim_steps_per_second,
                             float gravity_acceleration,
                             float wave_speed_cells_per_sec,
                             int active_boundary_kind, int active_theme_index) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " ShallowWater  %5.1f fps  sim:%3dHz  g=%.0f  c=%.1f  "
           "BC=%s  theme:%s ",
           measured_fps, sim_steps_per_second, (double)gravity_acceleration,
           (double)wave_speed_cells_per_sec,
           bc_name_table[active_boundary_kind],
           theme_table[active_theme_index].display_name);
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
           " q:quit  spc:pause  s:step  r:reset  d:drop  b:dam  "
           "g/G:gravity  a:arrows  l:shore  n:BC  o:obs  "
           "p/P:preset  t:theme  ]/[:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §19  scene — all the live state, in one place */

/*
 * Everything the demo is currently doing, grouped by what it's for.  The
 * h / u / v grids themselves live as file-scope arrays, not here, because
 * every solver loop touches them and passing a struct around would just
 * clutter the math.  Keeping the render toggles separate from the physics
 * settings is deliberate: flipping "show arrows" must never change the
 * water.
 */
typedef struct {
    /* ── physics settings (the keys tweak these) ── */
    float gravity_acceleration;     /* sets wave speed               */
    float bottom_friction_coeff;    /* drag that calms the water     */
    int   active_boundary_kind;     /* wall / open / periodic        */

    /* ── run control ── */
    bool  simulation_paused;        /* frozen if true                */
    bool  step_request_pending;     /* advance one frame, then freeze */
    int   active_preset_index;      /* which scene is loaded         */

    /* ── look only (never touches the water) ── */
    int   active_theme_index;       /* which colour look             */
    bool  show_arrows;              /* draw flow arrows              */
    bool  show_shoreline;           /* mark the waterline            */
    bool  show_obstacles;           /* draw the solid shapes         */

    /* ── HUD readouts ── */
    SimStats stats;                   /* refreshed each tick           */
    float    simulation_time_seconds; /* time since the scene loaded   */
} Scene;

static void scene_init(Scene *scene, int cols, int rows) {
  memset(scene, 0, sizeof *scene);
  scene->gravity_acceleration = GRAVITY_DEFAULT;
  scene->bottom_friction_coeff = BOTTOM_FRICTION_DEFAULT;
  scene->active_boundary_kind = BC_WALL;
  scene->active_theme_index = 0;
  scene->show_arrows = true;
  scene->show_shoreline = true;
  scene->show_obstacles = true;

  grid_active_cols = (cols < GRID_COLS_MAX) ? cols : GRID_COLS_MAX;
  grid_active_rows = (rows < GRID_ROWS_MAX) ? rows : GRID_ROWS_MAX;
  if (grid_active_rows < 4)
    grid_active_rows = 4;
  if (grid_active_cols < 4)
    grid_active_cols = 4;

  grid_clear_walls();
  grid_zero_fluid_fields();
  preset_table[PRESET_DAM_BREAK].loader(scene->active_boundary_kind);
  scene->active_preset_index = PRESET_DAM_BREAK;
}

static void scene_resize(Scene *scene, int cols, int rows) {
  grid_active_cols = (cols < GRID_COLS_MAX) ? cols : GRID_COLS_MAX;
  grid_active_rows = (rows < GRID_ROWS_MAX) ? rows : GRID_ROWS_MAX;
  if (grid_active_rows < 4)
    grid_active_rows = 4;
  if (grid_active_cols < 4)
    grid_active_cols = 4;
  /* Reload the scene so its walls and starting water fit the new size. */
  preset_table[scene->active_preset_index].loader(scene->active_boundary_kind);
  scene->simulation_time_seconds = 0.0f;
}

static void scene_load_preset(Scene *scene, int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  scene->active_preset_index = preset_index;
  preset_table[preset_index].loader(scene->active_boundary_kind);
  scene->simulation_time_seconds = 0.0f;
}

static void scene_tick(Scene *scene, float dt_seconds) {
  if (scene->simulation_paused && !scene->step_request_pending)
    return;
  scene->step_request_pending = false;
  scene->simulation_time_seconds += dt_seconds;
  swe_step(scene->active_boundary_kind, scene->gravity_acceleration,
           scene->bottom_friction_coeff, dt_seconds, &scene->stats);
}

/* §20  screen — ncurses setup, teardown, and drawing a frame */

/*
 * Just the terminal's size in characters — ncurses keeps the actual
 * screen buffers; we only need width and height to place the HUD and
 * clip the grid.
 */
typedef struct {
    int cols;   /* terminal width,  in cells */
    int rows;   /* terminal height, in cells */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
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

static void screen_present_frame(Screen *screen, const Scene *scene,
                                 double measured_fps,
                                 int sim_steps_per_second) {
  erase();

  render_field(stdscr, scene->active_theme_index, scene->show_arrows,
               scene->show_shoreline, scene->show_obstacles);

  render_stats_panel(stdscr, screen->rows, screen->cols, &scene->stats,
                     scene->gravity_acceleration, scene->bottom_friction_coeff,
                     scene->active_boundary_kind,
                     scene->simulation_time_seconds, scene->simulation_paused,
                     scene->show_arrows, scene->show_shoreline,
                     scene->active_preset_index);

  hud_paint_status(screen->cols, measured_fps, sim_steps_per_second,
                   scene->gravity_acceleration, scene->stats.wave_speed,
                   scene->active_boundary_kind, scene->active_theme_index);
  hud_paint_hint(screen->rows);

  wnoutrefresh(stdscr);
  doupdate();
}

/* §21  app — the main loop, signals, and key input */

/*
 * The whole program's state in one global, so the signal handlers can
 * reach it.  A handler can only safely poke a sig_atomic_t, and
 * "volatile" makes sure the main loop actually re-reads these flags
 * instead of caching them — that's how a Ctrl-C or a window resize
 * reaches the loop.
 *
 * sim_steps_per_second lives here, not in Scene, because it's a
 * timing-loop knob (it sets how often we tick); scene_tick just gets
 * the resulting dt handed to it.
 */
typedef struct {
    Scene  scene;                          /* the world + its controls  */
    Screen screen;                         /* terminal size             */
    int    sim_steps_per_second;           /* physics ticks per second  */
    volatile sig_atomic_t running;         /* Ctrl-C / kill clears this  */
    volatile sig_atomic_t need_resize;     /* window resize sets this    */
} App;

static App app_state;

static void on_signal_quit(int sig) {
  (void)sig;
  app_state.running = 0;
}
static void on_signal_resize(int sig) {
  (void)sig;
  app_state.need_resize = 1;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scene = &app->scene;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    scene->simulation_paused = !scene->simulation_paused;
    break;

  case 's':
  case 'S':
    scene->simulation_paused = true;
    scene->step_request_pending = true;
    break;

  case 'r':
  case 'R':
    scene_load_preset(scene, scene->active_preset_index);
    break;

  case 'd':
  case 'D': {
    float cx = (float)(grid_active_cols - 1) * 0.5f;
    float cy = (float)(grid_active_rows - 1) * 0.5f;
    apply_gaussian_drop(cx, cy, DROP_HEIGHT_AMPLITUDE, DROP_GAUSSIAN_RADIUS);
    zero_fields_in_walls();
    apply_boundary(scene->active_boundary_kind);
    break;
  }

  case 'b':
  case 'B':
    grid_zero_fluid_fields();
    apply_dam_break_initial_state();
    zero_fields_in_walls();
    apply_boundary(scene->active_boundary_kind);
    scene->simulation_time_seconds = 0.0f;
    break;

  case 'g':
    scene->gravity_acceleration += GRAVITY_STEP;
    if (scene->gravity_acceleration > GRAVITY_MAX)
      scene->gravity_acceleration = GRAVITY_MAX;
    break;
  case 'G':
    scene->gravity_acceleration -= GRAVITY_STEP;
    if (scene->gravity_acceleration < GRAVITY_MIN)
      scene->gravity_acceleration = GRAVITY_MIN;
    break;

  case 'n':
  case 'N':
    scene->active_boundary_kind = (scene->active_boundary_kind + 1) % BC_COUNT;
    scene_load_preset(scene, scene->active_preset_index);
    break;

  case 'p':
    scene_load_preset(scene, (scene->active_preset_index + 1) % PRESET_COUNT);
    break;
  case 'P':
    scene_load_preset(scene, (scene->active_preset_index + PRESET_COUNT - 1) %
                                 PRESET_COUNT);
    break;

  case 'o':
  case 'O':
    scene->show_obstacles = !scene->show_obstacles;
    break;

  case 'a':
  case 'A':
    scene->show_arrows = !scene->show_arrows;
    break;

  case 'l':
  case 'L':
    scene->show_shoreline = !scene->show_shoreline;
    break;

  case 't':
  case 'T':
    scene->active_theme_index = (scene->active_theme_index + 1) % THEME_COUNT;
    break;

  case ']':
    app->sim_steps_per_second += SIM_HZ_STEP;
    if (app->sim_steps_per_second > SIM_HZ_MAX)
      app->sim_steps_per_second = SIM_HZ_MAX;
    break;
  case '[':
    app->sim_steps_per_second -= SIM_HZ_STEP;
    if (app->sim_steps_per_second < SIM_HZ_MIN)
      app->sim_steps_per_second = SIM_HZ_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned)(clock_now_ns() & 0xFFFFFFFF));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal_quit);
  signal(SIGTERM, on_signal_quit);
  signal(SIGWINCH, on_signal_resize);

  App *app = &app_state;
  app->running = 1;
  app->sim_steps_per_second = SIM_HZ_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;
  int64_t sim_accumulator_ns = 0;

  const int64_t render_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (app->running) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }
    if (!app->running)
      break;

    /* ── resize ── */
    if (app->need_resize) {
      app->need_resize = 0;
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      prev_frame_ns = clock_now_ns();
      sim_accumulator_ns = 0;
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

    /* ── physics — fixed-dt accumulator ── */
    int64_t tick_ns = TICK_NS(app->sim_steps_per_second);
    float dt_sec = 1.0f / (float)app->sim_steps_per_second;
    sim_accumulator_ns += dt_ns;
    while (sim_accumulator_ns >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accumulator_ns -= tick_ns;
    }

    /* ── render ── */
    screen_present_frame(&app->screen, &app->scene, measured_fps,
                         app->sim_steps_per_second);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < render_cap_ns)
      clock_sleep_ns(render_cap_ns - spent);
  }

  return 0;
}
