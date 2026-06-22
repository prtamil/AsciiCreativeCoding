/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_gas.c — a tiny "particles on a honeycomb" model that turns into
 * real fluid flow. Each cell holds six yes/no bits (one per hex direction);
 * every tick, particles collide by a lookup table then hop one cell over.
 * Average over a little patch and you see wakes, jets, and channel flow —
 * with no fluid equation anywhere in the code. This is the FHP-I model.
 *
 * Original idea: Frisch, Hasslacher & Pomeau, Phys. Rev. Lett. 56 (1986).
 * Sister files: fluid/navier_stokes.c (solves the equation directly instead),
 *               fluid/fluid_sph.c (particles with no grid at all).
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config — all the tunable numbers in one place ── */

/* Biggest grid we'll ever allocate (held in BSS, not malloc'd). */
#define GRID_ROWS_MAX 128
#define GRID_COLS_MAX 512

/* How many physics ticks we run per drawn frame — turn it up for time-lapse. */
#define STEPS_PER_FRAME_DEFAULT 8
#define STEPS_PER_FRAME_MIN 1
#define STEPS_PER_FRAME_MAX 32

/* Chance each left-edge cell spits out a new eastward particle (the "wind"). */
#define INLET_PROB_DEFAULT 0.55f
#define INLET_PROB_MIN 0.10f
#define INLET_PROB_MAX 0.95f
#define INLET_PROB_STEP 0.05f

/* How often the physics advances, in ticks per second. */
#define SIM_HZ_DEFAULT 20
#define SIM_HZ_MIN 5
#define SIM_HZ_MAX 60
#define SIM_HZ_STEP 5

/* Don't draw faster than this many frames per second. */
#define RENDER_FPS_CAP 60

/* Terminal cells are about twice as tall as they are wide, so a "circle"
 * measured in cells comes out as an oval. Stretch vertical distances by
 * this factor when carving round obstacles so they actually look round. */
#define ASPECT_FACTOR_CELL_TO_VISUAL 2.0f

/* The only five cell patterns that actually scatter; everything else passes
 * through unchanged. Each name says which directions have a particle. */
#define COLLISION_HEADON_EW 0x09    /* E + W       (head-on pair)   */
#define COLLISION_HEADON_NE_SW 0x12 /* NE + SW     (head-on pair)   */
#define COLLISION_HEADON_NW_SE 0x24 /* NW + SE     (head-on pair)   */
#define COLLISION_THREEY_EVEN 0x15  /* E + NW + SW (three-way "Y")  */
#define COLLISION_THREEY_ODD 0x2A   /* NE + W + SE (the flipped Y)  */

#define COLLISION_TABLE_SIZE 64       /* every possible 6-bit cell state */
#define HEX_DIRECTIONS 6
#define HEX_DIRECTION_BITS_MASK 0x3F  /* keep only the 6 used bits */

/* Which bit means which direction. The collision constants above bake in
 * this exact ordering, so don't reshuffle it. */
enum {
  DIR_E = 0,  /* east       */
  DIR_NE = 1, /* north-east */
  DIR_NW = 2, /* north-west */
  DIR_W = 3,  /* west       */
  DIR_SW = 4, /* south-west */
  DIR_SE = 5, /* south-east */
};

/* Flow direction maps to one of 9 colours: strong-west … still … strong-east. */
#define MOMENTUM_BUCKET_COUNT 9

/* Glyphs from empty to packed; [0] is blank, [1..6] go sparse to dense. */
#define DENSITY_GLYPH_COUNT 7

/* We smooth the noisy grid by averaging a 3x3 block; this is the "1" in 3x3. */
#define AVG_WINDOW_HALF 1

/* Below this much fluid, draw nothing — the cell reads as empty. */
#define DENSITY_BLANK_THRESHOLD 0.25f

/* Above this much fluid, draw the glyph bold so dense flow pops out. */
#define DENSITY_BOLD_THRESHOLD 3.5f

/* Colour-pair slots. The 9 momentum colours come first, then wall + HUD. */
enum {
  PAIR_MOMENTUM_FIRST = 1, /* +0..+8 */
  PAIR_WALL = 1 + MOMENTUM_BUCKET_COUNT,
  PAIR_HUD,
  PAIR_HINT,
};

#define PRESET_COUNT 6
#define THEME_COUNT 5

/* Two rows at top/bottom are saved for the HUD, not the fluid. */
#define HUD_RESERVED_ROWS 2

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* ── §2  clock — wall time and sleeping ── */

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

/* ── §3  rng — a fast, cheap random number source ── *
 * xorshift32: small and quick, plenty random for our needs. Only the inlet
 * and the "Free" preset's starting gas use it; everything else is exact. */

static uint32_t xorshift_state = 12345u;

static inline uint32_t xorshift_next(void) {
  uint32_t s = xorshift_state;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  xorshift_state = s;
  return s;
}

/* A random number from 0 up to (but not including) 1. */
static inline float xorshift_unit_float(void) {
  return (float)(xorshift_next() >> 8) / (float)(1u << 24);
}

/* ── §4  hex_directions — where each of the 6 neighbours sits ── *
 *
 * We store a honeycomb in a plain rectangular array. The catch: on a
 * honeycomb, odd rows are shifted half a cell sideways from even rows, so
 * "the cell to my north-east" lands at a different (row, col) offset
 * depending on whether you're in an even or odd row. That's why each table
 * has two rows — picked by the row's parity (even = 0, odd = 1).
 *
 * The step a particle takes when it moves: hex_direction_drow gives the row
 * change, hex_direction_dcol the column change, for each of the 6 directions
 * (E, NE, NW, W, SW, SE in that order). */

static const int hex_direction_drow[2][HEX_DIRECTIONS] = {
    {0, -1, -1, 0, +1, +1}, /* even row */
    {0, -1, -1, 0, +1, +1}, /* odd  row */
};

static const int hex_direction_dcol[2][HEX_DIRECTIONS] = {
    {+1, 0, -1, -1, -1, 0}, /* even row */
    {+1, +1, 0, -1, 0, +1}, /* odd  row */
};

/* The exact opposite of each direction (used to reflect off walls). */
static const int direction_bounce_back[HEX_DIRECTIONS] = {
    DIR_W,  /* opposite of E  */
    DIR_SW, /* opposite of NE */
    DIR_SE, /* opposite of NW */
    DIR_E,  /* opposite of W  */
    DIR_NE, /* opposite of SW */
    DIR_NW, /* opposite of SE */
};

/* ── §5  collision rules — the lookup table that scatters particles ── *
 *
 * CollisionRules — the answer to "given what's in a cell, what comes out
 * after the particles bounce off each other?" — worked out once at startup.
 *
 * What it is
 *   A collision is purely local: take a cell's 6 bits (which directions hold
 *   a particle), look up the post-bounce 6 bits. There are only 64 possible
 *   inputs, so we just precompute all 64 answers and read them as an array in
 *   the hot loop. The bounce always keeps the same number of particles and
 *   the same total momentum — that conservation is the whole reason the
 *   averaged result behaves like a fluid. (Frisch-Hasslacher-Pomeau 1986.)
 *
 * Why two tables, one per "parity"
 *   When two particles meet head-on they can scatter either left or right —
 *   both are equally valid. If we always picked the same way, the whole fluid
 *   would slowly spin in one direction (a fake bias). The fix: split cells
 *   into a checkerboard by (row + col) being even or odd, and let the two
 *   colours scatter opposite ways. The biases cancel out.
 *
 * Layout
 *   lookup[parity][input bits] = output bits. Almost every entry just copies
 *   the input straight through (no collision). Only 5 entries per parity get
 *   overwritten: 3 head-on rotations and 2 three-way "Y" swaps.
 */
typedef struct {
    /* lookup[parity][input 6 bits] -> output 6 bits. parity is (row+col)&1;
     * the bits follow the E, NE, NW, W, SW, SE order from §4. */
    uint8_t lookup[2][COLLISION_TABLE_SIZE];
} CollisionRules;

static CollisionRules g_collisions;

static void collision_table_build(void) {
    /* Start with "nothing happens" everywhere: output = input. */
    for (int s = 0; s < COLLISION_TABLE_SIZE; s++) {
        g_collisions.lookup[0][s] = (uint8_t)s;
        g_collisions.lookup[1][s] = (uint8_t)s;
    }

    /* Even cells: head-on pairs rotate one way around the cycle. */
    g_collisions.lookup[0][COLLISION_HEADON_EW]    = COLLISION_HEADON_NE_SW;
    g_collisions.lookup[0][COLLISION_HEADON_NE_SW] = COLLISION_HEADON_NW_SE;
    g_collisions.lookup[0][COLLISION_HEADON_NW_SE] = COLLISION_HEADON_EW;

    /* Odd cells: the same pairs rotate the other way (cancels the bias). */
    g_collisions.lookup[1][COLLISION_HEADON_EW]    = COLLISION_HEADON_NW_SE;
    g_collisions.lookup[1][COLLISION_HEADON_NE_SW] = COLLISION_HEADON_EW;
    g_collisions.lookup[1][COLLISION_HEADON_NW_SE] = COLLISION_HEADON_NE_SW;

    /* The three-way "Y" just swaps with its mirror — same for both colours. */
    g_collisions.lookup[0][COLLISION_THREEY_EVEN] = COLLISION_THREEY_ODD;
    g_collisions.lookup[1][COLLISION_THREEY_EVEN] = COLLISION_THREEY_ODD;
    g_collisions.lookup[0][COLLISION_THREEY_ODD]  = COLLISION_THREEY_EVEN;
    g_collisions.lookup[1][COLLISION_THREEY_ODD]  = COLLISION_THREEY_EVEN;
}

/* ── §6  momentum_extract — read "how much" and "which way" from a cell ── *
 *
 * cell_momentum_x_doubled measures net rightward push. Each direction
 * contributes its horizontal share: E pushes fully right (+1), W fully left
 * (-1), the four diagonals half as much (±1/2). To avoid fractions we work
 * in doubled units (E = +2, diagonals = ±1), and the colour code divides it
 * back out later. */

static inline int cell_particle_count(uint8_t state) {
  return __builtin_popcount((unsigned)state);
}

static inline int cell_momentum_x_doubled(uint8_t state) {
  int e = (state >> DIR_E) & 1;
  int ne = (state >> DIR_NE) & 1;
  int nw = (state >> DIR_NW) & 1;
  int w = (state >> DIR_W) & 1;
  int sw = (state >> DIR_SW) & 1;
  int se = (state >> DIR_SE) & 1;
  return 2 * e + ne - nw - 2 * w - sw + se;
}

/* ── §7  themes — colour palettes that show flow direction ── *
 *
 * ColorTheme — one named look (Classic, Ocean, ...) for painting the flow.
 *
 * We colour each cell by which way its fluid is moving, not by how much fluid
 * is there. Momentum is the telling signal: a few collisions later the
 * density is nearly flat everywhere, so colouring by density would give a
 * dull, uniform picture, while momentum lights up flow fronts and the swirl
 * behind obstacles.
 *
 * Each theme holds two parallel colour lists of 9 entries each, plus a name
 * for the HUD. We pick which list to use once, at startup, based on the
 * terminal: the rich 256-colour list if it supports it, otherwise the basic
 * 8-colour fallback (fewer shades, but the left/still/right meaning survives).
 *
 * The 9 entries always run the same way: entry 0 = strongest leftward (west)
 * flow, entry 4 = barely moving, entry 8 = strongest rightward (east) flow.
 *
 * Brightness note: the 256-colour values are kept out of the darkest part of
 * the palette so even the dimmest tier stays visible on a black background. */
typedef struct {
    short       palette_256[MOMENTUM_BUCKET_COUNT];  /* colours for a 256-colour terminal */
    short       palette_8  [MOMENTUM_BUCKET_COUNT];  /* fallback for basic terminals */
    const char *name;                                /* shown in the HUD */
} ColorTheme;

static const ColorTheme color_theme_table[THEME_COUNT] = {
    /* 0 Classic — deep blue → cyan → grey → orange → red */
    {{21, 33, 51, 87, 244, 208, 202, 196, 160},
     {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_RED, COLOR_RED},
     "Classic"},
    /* 1 Ocean — midnight blue → teal → cyan → white */
    {{19, 27, 39, 51, 87, 123, 159, 195, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     "Ocean"},
    /* 2 Plasma — deep purple → magenta → gold */
    {{91, 129, 165, 201, 207, 213, 214, 220, 226},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED, COLOR_RED,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     "Plasma"},
    /* 3 Matrix — dark green → bright lime */
    {{28, 34, 40, 46, 82, 118, 154, 190, 226},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE},
     "Matrix"},
    /* 4 Heat — dark red → orange → bright yellow */
    {{52, 88, 124, 166, 172, 178, 214, 220, 226},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     "Heat"},
};

/* ── §8  colors — turn a theme into live ncurses colour pairs ── */

static bool terminal_has_256_colors = false;

static void colors_apply_theme(int theme_index) {
  const ColorTheme *theme = &color_theme_table[theme_index];
  for (int i = 0; i < MOMENTUM_BUCKET_COUNT; i++) {
    short fg =
        terminal_has_256_colors ? theme->palette_256[i] : theme->palette_8[i];
    init_pair((short)(PAIR_MOMENTUM_FIRST + i), fg, -1);
  }

  if (terminal_has_256_colors) {
    init_pair(PAIR_WALL, 244, -1);
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_WALL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  terminal_has_256_colors = (COLORS >= 256);
  colors_apply_theme(theme_index);
}

/* Turn a flow value (roughly -2 = full west .. +2 = full east) into one of
 * the 9 colour slots, clamped so out-of-range values just hit the ends. */
static inline int momentum_to_color_bucket(float m_avg_doubled) {
  float t = (m_avg_doubled + 2.0f) / 4.0f; /* rescale to 0..1 */
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  int idx = (int)(t * 8.49f); /* spread across the 9 slots */
  if (idx < 0)
    idx = 0;
  if (idx >= MOMENTUM_BUCKET_COUNT)
    idx = MOMENTUM_BUCKET_COUNT - 1;
  return idx;
}

/* Empty to packed: more fluid in a cell picks a glyph further along. */
static const char density_glyph_ramp[DENSITY_GLYPH_COUNT] = {' ', '.', ',', 'o',
                                                             'O', '0', '@'};

/* ── §9  scene — one struct holding everything the running demo needs ── *
 *
 * Scene — all the live state in one place: the grid itself, the obstacles,
 * and the knobs the user can turn. It's a single fixed-size global (no
 * malloc), so every part of the code reaches it as g_scene.<field>.
 *
 * The fields are grouped by who touches them so it's obvious at a glance
 * which subsystem owns each one. One grouping matters for correctness:
 * active_theme_index is a pure display choice and the physics must never
 * read it — otherwise picking a different colour theme could change how the
 * fluid evolves, which would be a bug. */
typedef struct {
    /* ── The simulation grid (touched every physics tick) ── */
    uint8_t particle_grid[GRID_ROWS_MAX][GRID_COLS_MAX]; /* 6 direction bits per cell */
    uint8_t stream_buffer[GRID_ROWS_MAX][GRID_COLS_MAX]; /* scratch copy for the hop step */
    bool    wall_mask    [GRID_ROWS_MAX][GRID_COLS_MAX]; /* true = solid obstacle */

    /* ── How much of the grid is actually in use (sim and drawing both read these) ── */
    int grid_active_rows;
    int grid_active_cols;

    /* ── What the user picked, and whether we're paused ── */
    int  active_preset_index;          /* which scene (Cylinder, Channel, ...) */
    bool simulation_paused;            /* gates the tick and shows "PAUSED" */

    /* ── Speed and inlet knobs ── */
    int   physics_steps_per_frame;     /* ticks run per drawn frame (time-lapse) */
    int   sim_steps_per_second;        /* how often the physics advances */
    bool  inlet_enabled;               /* is the left-edge wind on? */
    float inlet_probability_per_cell;  /* per-cell chance of a new particle */

    /* ── Display only — must not affect the physics ── */
    int active_theme_index;            /* which colour palette is showing */

    /* ── Bookkeeping ── */
    long physics_tick_count;           /* total ticks since reset (for the HUD) */
} Scene;

static Scene g_scene = {
    .physics_steps_per_frame    = STEPS_PER_FRAME_DEFAULT,
    .sim_steps_per_second       = SIM_HZ_DEFAULT,
    .inlet_enabled              = true,
    .inlet_probability_per_cell = INLET_PROB_DEFAULT,
};

/* ── §10  step_inject_inlet — the wind that keeps the flow going ── *
 * Along the left edge, randomly add eastward particles. Without this, the
 * flow would just settle and stop. */

static void step_inject_inlet(void) {
  if (!g_scene.inlet_enabled)
    return;
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    if (g_scene.wall_mask[r][0])
      continue;
    if (xorshift_unit_float() < g_scene.inlet_probability_per_cell)
      g_scene.particle_grid[r][0] |= (uint8_t)(1u << DIR_E);
  }
}

/* ── §11  step_collide — let particles bounce off each other ── *
 * One table lookup per cell swaps its bits for the post-collision bits.
 * The checkerboard parity decides which scatter direction this cell uses. */
static void step_collide(void) {
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      if (g_scene.wall_mask[r][c])
        continue;
      int parity = (r + c) & 1;
      uint8_t s = g_scene.particle_grid[r][c] & HEX_DIRECTION_BITS_MASK;
      g_scene.particle_grid[r][c] = g_collisions.lookup[parity][s];
    }
  }
}

/* ── §12  step_stream — every particle hops one cell forward ── *
 * We write into a separate buffer and copy it back, so particles move
 * exactly once (writing in place would double-move some of them). If a
 * particle would land in a wall it instead stays put with its direction
 * flipped — a simple bounce that makes the wall act like a solid surface.
 * Particles that run off an edge wrap around to the other side. */

static void step_stream(void) {
  memset(g_scene.stream_buffer, 0, sizeof g_scene.stream_buffer);

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    int parity = r & 1;
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      if (g_scene.wall_mask[r][c])
        continue;
      uint8_t state = g_scene.particle_grid[r][c];
      if (state == 0)
        continue;

      for (int d = 0; d < HEX_DIRECTIONS; d++) {
        if (!((state >> d) & 1))
          continue;

        int nr = (r + hex_direction_drow[parity][d] + g_scene.grid_active_rows) %
                 g_scene.grid_active_rows;
        int nc = (c + hex_direction_dcol[parity][d] + g_scene.grid_active_cols) %
                 g_scene.grid_active_cols;

        if (g_scene.wall_mask[nr][nc]) {
          /* Bounce-back: stay at source, flip direction. */
          g_scene.stream_buffer[r][c] |= (uint8_t)(1u << direction_bounce_back[d]);
        } else {
          /* Normal hop: deposit bit d at neighbour cell. */
          g_scene.stream_buffer[nr][nc] |= (uint8_t)(1u << d);
        }
      }
    }
  }

  memcpy(g_scene.particle_grid, g_scene.stream_buffer, sizeof g_scene.particle_grid);
}

/* ── §13  step — one full tick: inject, collide, stream ── *
 * That's the entire simulation, three lines. */
static void physics_step(void) {
  step_inject_inlet();
  step_collide();
  step_stream();
  g_scene.physics_tick_count++;
}

/* ── §14  presets — the scenes the user can flip between ── *
 *
 * Preset — one ready-made scene that shows off a different fluid effect:
 * the wake behind a cylinder, jets through slits in a wall, fast-in-the-
 * middle channel flow, or a random gas left to settle on its own.
 *
 * A preset just says how the scene STARTS — whether the left-edge wind is
 * on, and whether to seed random particles up front (only "Free" does, since
 * the rest rely on the wind to fill the grid). The actual obstacle shapes
 * are carved by the build_* helpers below, picked by index in scene_load. */
typedef struct {
    bool        inlet;            /* turn on the left-edge wind? */
    float       initial_density;  /* fraction of bits to seed at start (0 unless "Free") */
    const char *name;             /* short label for the HUD */
    const char *desc;             /* one line telling the user what to watch for */
} Preset;

static const Preset preset_table[PRESET_COUNT] = {
    {true, 0.0f, "Cylinder",
     "Particles hit cylinder -- BLUE wake forms and grows behind"},
    {true, 0.0f, "2-Slit",
     "Two gaps in a wall -- jets spread out and meet in the middle"},
    {true, 0.0f, "3-Slit",
     "Three gaps -- three jets spread and interfere downstream"},
    {true, 0.0f, "4-Slit",
     "Four gaps -- tight jets, rich interference pattern"},
    {true, 0.0f, "Channel",
     "Parallel walls top & bottom -- flow fastest at centre (Poiseuille)"},
    {false, 0.40f, "Free",
     "No inlet, no walls -- random gas relaxes toward equilibrium"},
};

/* ── §15  scene_load — set up the grid for a chosen preset ── *
 * Clear everything, optionally sprinkle in starting particles, then carve
 * the obstacles for that scene. */

static void scene_seed_random_particles(float density_per_bit) {
  if (density_per_bit <= 0.0f)
    return;
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      uint8_t bits = 0;
      for (int b = 0; b < HEX_DIRECTIONS; b++)
        if (xorshift_unit_float() < density_per_bit)
          bits |= (uint8_t)(1u << b);
      g_scene.particle_grid[r][c] = bits;
    }
  }
}

static void scene_build_cylinder(void) {
  int centre_col = g_scene.grid_active_cols * 2 / 5;
  int centre_row = g_scene.grid_active_rows / 2;
  int radius = g_scene.grid_active_cols / 12;
  if (radius < 3)
    radius = 3;

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      float dx = (float)(c - centre_col);
      float dy = (float)(r - centre_row) * ASPECT_FACTOR_CELL_TO_VISUAL;
      if (dx * dx + dy * dy < (float)(radius * radius)) {
        g_scene.wall_mask[r][c] = true;
        g_scene.particle_grid[r][c] = 0;
      }
    }
  }
}

static void scene_build_slit_wall(int slit_count) {
  int wall_col = g_scene.grid_active_cols / 3;
  int gap_height = g_scene.grid_active_rows / (slit_count * 3);
  if (gap_height < 2)
    gap_height = 2;

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    bool inside_slit = false;
    for (int s = 0; s < slit_count; s++) {
      int slit_centre_row = g_scene.grid_active_rows * (s + 1) / (slit_count + 1);
      if (r >= slit_centre_row - gap_height / 2 &&
          r < slit_centre_row + gap_height / 2) {
        inside_slit = true;
        break;
      }
    }
    if (inside_slit)
      continue;

    /* Make the wall two cells thick so fast particles can't slip through. */
    for (int dc = 0; dc < 2; dc++) {
      int wc = wall_col + dc;
      if (wc >= g_scene.grid_active_cols)
        break;
      g_scene.wall_mask[r][wc] = true;
      g_scene.particle_grid[r][wc] = 0;
    }
  }
}

static void scene_build_channel(void) {
  int wall_thickness = (g_scene.grid_active_rows > 8) ? 2 : 1;
  for (int c = 0; c < g_scene.grid_active_cols; c++) {
    for (int i = 0; i < wall_thickness; i++) {
      g_scene.wall_mask[i][c] = true;
      g_scene.wall_mask[g_scene.grid_active_rows - 1 - i][c] = true;
      g_scene.particle_grid[i][c] = 0;
      g_scene.particle_grid[g_scene.grid_active_rows - 1 - i][c] = 0;
    }
  }
}

static void scene_load(int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  g_scene.active_preset_index = preset_index;
  g_scene.inlet_enabled = preset_table[preset_index].inlet;
  g_scene.physics_tick_count = 0;

  memset(g_scene.wall_mask, 0, sizeof g_scene.wall_mask);
  memset(g_scene.particle_grid, 0, sizeof g_scene.particle_grid);
  memset(g_scene.stream_buffer, 0, sizeof g_scene.stream_buffer);

  scene_seed_random_particles(preset_table[preset_index].initial_density);

  switch (preset_index) {
  case 0:
    scene_build_cylinder();
    break;
  case 1:
    scene_build_slit_wall(2);
    break;
  case 2:
    scene_build_slit_wall(3);
    break;
  case 3:
    scene_build_slit_wall(4);
    break;
  case 4:
    scene_build_channel();
    break;
  case 5: /* Free — no obstacles */
    break;
  default:
    break;
  }
}

/* ── §16  cell_neighbourhood — smooth out the noise before drawing ── *
 * A single cell is too jumpy to look at, so we average the fluid amount and
 * the flow direction over the 3x3 block around it (skipping walls). What's
 * left is the smooth, large-scale flow we actually want to see. */

static void cell_neighbourhood_average(int r, int c, float *density_out,
                                       float *momentum_x_doubled_out) {
  float density_sum = 0.0f;
  float momentum_sum = 0.0f;
  int counted = 0;

  for (int dr = -AVG_WINDOW_HALF; dr <= AVG_WINDOW_HALF; dr++) {
    for (int dc = -AVG_WINDOW_HALF; dc <= AVG_WINDOW_HALF; dc++) {
      int nr = r + dr;
      int nc = c + dc;
      if (nr < 0 || nr >= g_scene.grid_active_rows)
        continue;
      if (nc < 0 || nc >= g_scene.grid_active_cols)
        continue;
      if (g_scene.wall_mask[nr][nc])
        continue;

      uint8_t s = g_scene.particle_grid[nr][nc];
      density_sum += (float)cell_particle_count(s);
      momentum_sum += (float)cell_momentum_x_doubled(s);
      counted++;
    }
  }

  if (counted > 0) {
    density_sum /= (float)counted;
    momentum_sum /= (float)counted;
  }
  *density_out = density_sum;
  *momentum_x_doubled_out = momentum_sum;
}

/* ── §17  render — paint the fluid as glyphs and colours ── *
 * Walls are '#'. Empty cells stay blank. For everything else, the amount of
 * fluid picks the glyph (sparse to dense) and the flow direction picks the
 * colour (west to east). So you read "how much" from the character and
 * "which way" from the colour, independently. */

static void render_fluid_field(int draw_rows, int draw_cols) {
  for (int r = 0; r < draw_rows; r++) {
    for (int c = 0; c < draw_cols; c++) {

      if (g_scene.wall_mask[r][c]) {
        attron(COLOR_PAIR(PAIR_WALL) | A_BOLD);
        mvaddch(r, c, '#');
        attroff(COLOR_PAIR(PAIR_WALL) | A_BOLD);
        continue;
      }

      float density, momentum_doubled;
      cell_neighbourhood_average(r, c, &density, &momentum_doubled);

      if (density < DENSITY_BLANK_THRESHOLD) {
        mvaddch(r, c, ' ');
        continue;
      }

      int glyph_index = (int)(density + 0.5f);
      if (glyph_index < 1)
        glyph_index = 1;
      if (glyph_index >= DENSITY_GLYPH_COUNT)
        glyph_index = DENSITY_GLYPH_COUNT - 1;

      int bucket = momentum_to_color_bucket(momentum_doubled);
      int pair = PAIR_MOMENTUM_FIRST + bucket;
      attr_t bold = (density >= DENSITY_BOLD_THRESHOLD) ? A_BOLD : 0;

      attron(COLOR_PAIR(pair) | bold);
      mvaddch(r, c, (chtype)(unsigned char)density_glyph_ramp[glyph_index]);
      attroff(COLOR_PAIR(pair) | bold);
    }
  }
}

/* ── §18  hud — status line, scene blurb, and key hints ── */

static void hud_paint_status(int term_cols) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " [%d] %-9s tick:%-6ld inlet:%3.0f%% steps:%2d sim:%2dHz "
           "theme:%-7s %s ",
           g_scene.active_preset_index + 1, preset_table[g_scene.active_preset_index].name,
           g_scene.physics_tick_count, g_scene.inlet_probability_per_cell * 100.0f,
           g_scene.physics_steps_per_frame, g_scene.sim_steps_per_second,
           color_theme_table[g_scene.active_theme_index].name,
           g_scene.simulation_paused ? "PAUSED" : "running");
  int len = (int)strlen(buf);
  int x = term_cols - len;
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_description(int term_rows, int term_cols) {
  /* Preset description on the second row from bottom (above hint). */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(term_rows - 2, 0, "  %-*s", term_cols - 2,
           preset_table[g_scene.active_preset_index].desc);
  attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_paint_hint(int term_rows) {
  const char *hint = " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme  "
                     "i/I:inlet  +/-:steps  ]/[:simHz ";
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0, "%s", hint);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §19  screen — ncurses setup, teardown, and the per-frame draw ── *
 *
 * Screen — just the terminal's current size in character cells. ncurses
 * holds the real screen buffer; we only need the width and height to place
 * the HUD and to clip the fluid to whatever fits. */
typedef struct {
    int rows;   /* terminal height in cells */
    int cols;   /* terminal width  in cells */
} Screen;

static void screen_init(Screen *s, int theme_index) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init(theme_index);
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);

  g_scene.grid_active_rows = (s->rows - HUD_RESERVED_ROWS < GRID_ROWS_MAX)
                         ? (s->rows - HUD_RESERVED_ROWS)
                         : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (s->cols < GRID_COLS_MAX) ? s->cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
}

static void screen_present_frame(Screen *s) {
  erase();
  int draw_rows = (g_scene.grid_active_rows < s->rows - HUD_RESERVED_ROWS)
                      ? g_scene.grid_active_rows
                      : s->rows - HUD_RESERVED_ROWS;
  int draw_cols = (g_scene.grid_active_cols < s->cols) ? g_scene.grid_active_cols : s->cols;
  render_fluid_field(draw_rows, draw_cols);
  hud_paint_status(s->cols);
  hud_paint_description(s->rows, s->cols);
  hud_paint_hint(s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §20  app — main loop, signal handling, keyboard input ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch, Screen *s) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case 'p':
  case ' ':
    g_scene.simulation_paused = !g_scene.simulation_paused;
    break;

  case 'r':
  case 'R':
    scene_load(g_scene.active_preset_index);
    break;

  case 'n':
    scene_load((g_scene.active_preset_index + 1) % PRESET_COUNT);
    break;
  case 'N':
    scene_load((g_scene.active_preset_index + PRESET_COUNT - 1) % PRESET_COUNT);
    break;

  case 't':
    g_scene.active_theme_index = (g_scene.active_theme_index + 1) % THEME_COUNT;
    colors_apply_theme(g_scene.active_theme_index);
    break;
  case 'T':
    g_scene.active_theme_index = (g_scene.active_theme_index + THEME_COUNT - 1) % THEME_COUNT;
    colors_apply_theme(g_scene.active_theme_index);
    break;

  case 'i':
    if (g_scene.inlet_probability_per_cell + INLET_PROB_STEP <= INLET_PROB_MAX)
      g_scene.inlet_probability_per_cell += INLET_PROB_STEP;
    break;
  case 'I':
    if (g_scene.inlet_probability_per_cell - INLET_PROB_STEP >= INLET_PROB_MIN)
      g_scene.inlet_probability_per_cell -= INLET_PROB_STEP;
    break;

  case '+':
  case '=':
    if (g_scene.physics_steps_per_frame < STEPS_PER_FRAME_MAX)
      g_scene.physics_steps_per_frame++;
    break;
  case '-':
    if (g_scene.physics_steps_per_frame > STEPS_PER_FRAME_MIN)
      g_scene.physics_steps_per_frame--;
    break;

  case ']':
    if (g_scene.sim_steps_per_second + SIM_HZ_STEP <= SIM_HZ_MAX)
      g_scene.sim_steps_per_second += SIM_HZ_STEP;
    break;
  case '[':
    if (g_scene.sim_steps_per_second - SIM_HZ_STEP >= SIM_HZ_MIN)
      g_scene.sim_steps_per_second -= SIM_HZ_STEP;
    break;

  default:
    break;
  }
  (void)s;
  return true;
}

int main(void) {
  xorshift_state = (uint32_t)time(NULL) ^ 0xFACEB00Cu;

  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  collision_table_build();

  Screen screen;
  screen_init(&screen, g_scene.active_theme_index);

  g_scene.grid_active_rows = (screen.rows - HUD_RESERVED_ROWS < GRID_ROWS_MAX)
                         ? (screen.rows - HUD_RESERVED_ROWS)
                         : GRID_ROWS_MAX;
  g_scene.grid_active_cols =
      (screen.cols < GRID_COLS_MAX) ? screen.cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;

  scene_load(g_scene.active_preset_index);

  int64_t next_physics_at_ns = clock_now_ns();
  const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (!g_should_quit) {
    int64_t frame_start = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch, &screen)) {
        g_should_quit = 1;
        break;
      }
    }

    /* ── resize ── */
    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_load(g_scene.active_preset_index);
      next_physics_at_ns = clock_now_ns();
    }

    /* ── physics ── */
    int64_t now_ns = clock_now_ns();
    if (!g_scene.simulation_paused && now_ns >= next_physics_at_ns) {
      for (int s = 0; s < g_scene.physics_steps_per_frame; s++)
        physics_step();
      next_physics_at_ns = now_ns + TICK_NS(g_scene.sim_steps_per_second);
    }

    /* ── render ── */
    screen_present_frame(&screen);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
