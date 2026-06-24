/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire.c — a flame that fills the terminal, painted in ASCII characters.
 *
 * Three completely different ways of making the fire take turns: a
 * cellular automaton (the old Doom fire trick), a swarm of rising
 * particles, and a procedural plasma built from sine waves.  All three
 * just fill a grid of heat values; one shared renderer turns that heat
 * into glyphs.  Press a key to swap engines, themes, wind, or fuel.
 *
 * Doom fire trick: Sanglard, "How DOOM Fire Was Done" (fabiensanglard.net).
 * Dithering: Floyd & Steinberg, "An adaptive algorithm for spatial gray
 * scale" (Proc. SID 17/2, 1976).
 *
 *   gcc -std=c11 -O2 -Wall -Wextra fire.c -o fire -lncurses -lm
 */

/* ── §1  includes ── */
/* The feature-test macro turns on clock_gettime() and nanosleep(). */

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

/* ── §2  loop presets ── */
/* Knobs for the outer loop. The picture redraws at a fixed 60 fps; the
 * simulation runs at its own rate, which [ and ] adjust between the min
 * and max below. */

enum {
  SIM_FPS_MIN = 5,      /* lower bound for [/] cycling                */
  SIM_FPS_DEFAULT = 30, /* initial simulation rate                    */
  SIM_FPS_MAX = 60,     /* upper bound for [/] cycling                */
  SIM_FPS_STEP = 5,     /* one press of [/] adjusts by this many Hz   */

  HUD_COLS = 64,       /* max width of any HUD status string         */
  FPS_UPDATE_MS = 500, /* render-fps averaging window                */

  N_ALGOS = 3,          /* CA, Particle, Plasma                       */
  N_DEBUG_MODES = 4,    /* off, raw heat, gamma-only, arch envelope   */
  MAX_FIRE_PARTS = 800, /* particle pool size                         */
};

#define MAX_HEAT 1.0f /* heat-grid ceiling; cells live in [0, MAX_HEAT] */
#define WIND_MAX 3    /* maximum |wind| in cells / tick                 */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* ── §3  source presets ── */
/* These shape the fuel along the bottom row — the arch every engine
 * draws heat from. The margins stay cold, a little randomness makes the
 * base flicker, and the warmup count lets the flame build up instead of
 * popping into existence at full size. */

#define ARCH_MARGIN_FRAC 0.04f  /* fraction of width kept cold at each side  */
#define FUEL_JITTER_BASE 0.82f  /* smallest random fuel multiplier           */
#define FUEL_JITTER_RANGE 0.18f /* random amount added on top, 0 to 0.18     */
#define WARMUP_TICKS 80 /* ticks to fade fuel from 0 up to full      */

/* ── §4  ca presets ── */
/* Tuning for the Doom-style cellular automaton (engine 0). We aim the
 * flame top at a chosen fraction of the screen height and work out how
 * fast heat must fade to land there, so it looks the same on any size
 * terminal. The floors stop a tiny window from fading to nothing. */

#define CA_REACH_FRAC 0.75f /* aim the flame top at 75% of the height    */
#define CA_DECAY_BASE_FRAC                                                     \
  0.55f /* steady part of the fade, as a share of avg  */
#define CA_DECAY_RAND_FRAC                                                     \
  0.90f /* random part of the fade, as a share of avg  */
#define CA_DECAY_BASE_MIN                                                      \
  0.010f /* smallest steady fade, for tiny terminals   */
#define CA_DECAY_RAND_MIN                                                      \
  0.015f /* smallest random fade, for tiny terminals   */

/* ── §5  particle presets ── */
/* Tuning for the rising-particle engine (engine 1). These set how long
 * each ember lives, how fast it shoots up, a small sideways kick at
 * birth, a random nudge each tick that makes it wander, how quickly
 * sideways drift settles, and how many new embers appear per tick. */

#define PART_LIFE_MIN 15.f   /* shortest ember lifetime, in ticks          */
#define PART_LIFE_RANGE 20.f /* extra random lifetime added on top         */
#define PART_VY_BASE 0.5f    /* slowest upward speed, cells per tick        */
#define PART_VY_RANGE 0.8f   /* extra random upward speed                   */
#define PART_VX_SPREAD 0.5f  /* size of the sideways kick at birth          */
#define PART_TURB_STEP 0.15f /* random sideways nudge each tick             */
#define PART_VX_DAMP 0.96f   /* how fast sideways drift settles each tick   */
#define SPAWN_PER_TICK 20    /* new embers per tick once fully warmed up    */

/* ── §6  plasma presets ── */
/* Tuning for the sine-wave plasma engine (engine 2). TIME_STEP is how
 * far the animation clock advances each tick. BASE keeps the flame
 * height positive. The three H1/H2/H3 groups are three sine waves added
 * together: AMP is the wave's strength, XFREQ how many ripples fit
 * across the screen, TSPD how fast it drifts over time. */

#define PLASMA_TIME_STEP 0.07f
#define PLASMA_BASE 0.50f
#define PLASMA_H1_AMP 0.28f
#define PLASMA_H1_XFREQ 5.0f
#define PLASMA_H1_TSPD 2.2f
#define PLASMA_H2_AMP 0.18f
#define PLASMA_H2_XFREQ 11.0f
#define PLASMA_H2_TSPD 1.6f
#define PLASMA_H3_AMP 0.10f
#define PLASMA_H3_XFREQ 3.0f
#define PLASMA_H3_TSPD 0.7f

/* ── §7  monotonic clock ── */
/* The loop's clock. We use the monotonic clock — a stopwatch that only
 * counts forward and never jumps, even if someone resets the system
 * time — so the animation never stutters. */

static int64_t clock_ns(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (int64_t)now.tv_sec * NS_PER_SEC + now.tv_nsec;
}

/* Pause for the given time so the loop holds 60 fps. A zero or negative
 * request just returns. */
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* ── §8  ramp + lut ── */
/* The nine characters we paint with, coldest to hottest, plus the
 * brightness cutoff at which each one kicks in. A space is cold/empty,
 * '@' is the white-hot core. The cutoffs bunch up in the middle because
 * that is where the eye notices flame detail most. */

static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 9 visible glyphs */

static const float k_lut_breaks[RAMP_N] = {
    0.000f, /* ' '  cold      */
    0.080f, /* '.'  ember     */
    0.180f, /* ':'  low       */
    0.290f, /* '+'  mid-low   */
    0.390f, /* 'x'  mid       */
    0.500f, /* '*'  mid-high  */
    0.620f, /* 'X'  hot       */
    0.750f, /* '#'  very hot  */
    0.900f, /* '@'  core      */
};

/* Pick which of the nine characters fits a brightness in [0,1]: return
 * the highest one whose cutoff it clears. */
static int lut_index(float v) {
  int bucket = 0;
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (v >= k_lut_breaks[i]) {
      bucket = i;
      break;
    }
  return bucket;
}

/* The "typical" brightness a character stands for — the middle of its
 * range. Dithering compares the real value against this to find how much
 * rounding error to spread around. */
static float lut_midpoint(int idx) {
  if (idx <= 0)
    return 0.f;
  if (idx >= RAMP_N - 1)
    return 1.f;
  return (k_lut_breaks[idx] + k_lut_breaks[idx + 1]) * 0.5f;
}

/* ── §9  theme palettes ── */
/* Six colour schemes for the flame. Each one gives a colour to every
 * character in the ramp. The faint end stays bright enough to read (no
 * near-black, which vanishes on dark terminals) and the hot end is
 * near-white so the core always stands out. Old terminals that only have
 * eight colours fall back to a coarser version. */

/* One colour scheme. For each of the nine ramp characters it stores the
 * colour to use, both as a full 256-colour value and as a basic
 * eight-colour fallback with a dim/normal/bold tweak to fake the
 * in-between shades. */
typedef struct {
  const char *name;     /* scheme name shown in the HUD             */
  int fg256[RAMP_N];    /* colour per character, 256-colour palette */
  int fg8[RAMP_N];      /* colour per character, basic 8-colour set */
  attr_t attr8[RAMP_N]; /* dim/normal/bold tweak for the 8-colour set */
} FireTheme;

#define CP_BASE 1 /* ramp pairs: CP_BASE .. CP_BASE+RAMP_N-1 */
#define PAIR_HUD                                                               \
  (CP_BASE + RAMP_N) /* bright yellow status (row 0 / row 1)    */
#define PAIR_HINT                                                              \
  (CP_BASE + RAMP_N + 1) /* bright cyan key hint (row rows-1)       */

static const FireTheme k_themes[] = {
    {/* 0  fire — classic red / orange / yellow */
     "fire",
     {88, 124, 160, 196, 202, 208, 214, 220, 231},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 1  ice — sky blue / cyan / white */
     "ice",
     {25, 27, 33, 39, 45, 51, 87, 159, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_BOLD, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 2  plasma — violet / magenta / white */
     "plasma",
     {55, 91, 93, 129, 165, 201, 207, 213, 231},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_DIM, A_NORMAL,
      A_BOLD}},
    {/* 3  nova — green / lime / white */
     "nova",
     {28, 34, 40, 46, 82, 118, 154, 190, 231},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 4  poison — olive / yellow-green / white */
     "poison",
     {58, 64, 70, 76, 118, 154, 184, 220, 231},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 5  gold — amber / orange / yellow */
     "gold",
     {130, 136, 172, 178, 208, 214, 220, 226, 231},
     {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_BOLD, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* ── §10 colour setup ── */
/* Tells ncurses which colours to use. The flame colours are re-pointed
 * whenever the user cycles themes, but the HUD colours are set once and
 * left alone, so the readout stays legible no matter the theme. */

/* Re-point the flame's colours to the chosen theme. Leaves the HUD
 * colours untouched. */
static void theme_apply(int theme_index) {
  const FireTheme *theme = &k_themes[theme_index];
  for (int i = 0; i < RAMP_N; i++) {
    if (COLORS >= 256)
      init_pair(CP_BASE + i, theme->fg256[i], COLOR_BLACK);
    else
      init_pair(CP_BASE + i, theme->fg8[i], COLOR_BLACK);
  }
}

/* One-time colour setup at startup: load the first theme's flame colours
 * and the two HUD colours, which then stay fixed for the whole run. */
static void color_init(int initial_theme) {
  start_color();
  theme_apply(initial_theme);

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(PAIR_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

/* The full drawing style for one ramp character: its colour plus a bold
 * touch. With 256 colours the colour is vivid enough on its own, so only
 * the two hottest characters get bolded; with eight colours the theme's
 * dim/normal/bold table fills in the missing shades. */
static attr_t ramp_attr(int idx, int theme_index) {
  attr_t attr = COLOR_PAIR(CP_BASE + idx);
  if (COLORS >= 256) {
    if (idx >= RAMP_N - 2)
      attr |= A_BOLD;
  } else {
    attr |= k_themes[theme_index].attr8[idx];
  }
  return attr;
}

/* ── §11 grid storage ── */
/* The simulation's lasting state. The Grid holds the heat values and the
 * user's wind/fuel/theme choices; FirePart is one ember used by the
 * particle engine. The whole ember pool lives inside the Grid so the hot
 * loop never has to chase a pointer or allocate memory. */

/* One short-lived ember in the particle engine. It is born at the base,
 * floats up, and its heat fades a little every tick until it burns out.
 *   x, y    where the ember is, in grid cells
 *   vx, vy  how fast it is moving, in cells per tick; negative vy is up
 *   heat    how hot it glows right now, from 1 down toward 0
 *   decay   how much heat it loses each tick (1 / lifetime)
 *   active  false means this slot is free for a new ember
 */
typedef struct {
  float x, y;
  float vx, vy;
  float heat;
  float decay;
  bool active;
} FirePart;

/* Everything the simulation needs in one place. The three buffers are
 * flat arrays the size of the screen, read as [y * cols + x]; row 0 is
 * the top of the terminal and the last row is the fuel line. The ember
 * pool sits inline so a resize never has to move it.
 *   heat       this frame's temperature for every cell, 0 to MAX_HEAT
 *   prev_heat  last frame's temperature, used to tell what changed
 *   dither     scratch space for the dithering pass
 *   cols, rows grid size in cells
 *   fuel       how strong the fire is, user-set, 0.1 to 1.0
 *   wind       sideways push the user sets, -WIND_MAX to WIND_MAX
 *   wind_acc   running sideways offset the arch has drifted so far
 *   theme      which colour scheme is active
 *   warmup     counts up to WARMUP_TICKS then stops; fades the fire in
 *   algo       which engine is running (0 CA, 1 particle, 2 plasma)
 *   plasma_t   the plasma engine's animation clock
 *   parts      the fixed pool of embers
 *   part_idx   where the spawner last looked for a free ember slot
 */
typedef struct {
  float *heat;
  float *prev_heat;
  float *dither;
  int cols, rows;

  float fuel;
  int wind;
  int wind_acc;
  int theme;
  int warmup;
  int algo;
  float plasma_t;

  FirePart parts[MAX_FIRE_PARTS];
  int part_idx;
} Grid;

/* Reserve the three screen-sized buffers, zeroed. Called once at startup
 * and again on resize, so the running loop never allocates. The caller
 * owns this memory and must free it with grid_free(). */
static void grid_alloc(Grid *grid, int cols, int rows) {
  grid->cols = cols;
  grid->rows = rows;
  grid->heat = calloc((size_t)(cols * rows), sizeof(float));
  grid->prev_heat = calloc((size_t)(cols * rows), sizeof(float));
  grid->dither = calloc((size_t)(cols * rows), sizeof(float));
}

/* Give back the three buffers and blank the struct. */
static void grid_free(Grid *grid) {
  free(grid->heat);
  free(grid->prev_heat);
  free(grid->dither);
  memset(grid, 0, sizeof *grid);
}

/* Throw away the old buffers and make new ones at the new size. The fire
 * starts over from cold; the next ticks rebuild it. */
static void grid_resize(Grid *grid, int cols, int rows) {
  grid_free(grid);
  grid_alloc(grid, cols, rows);
}

/* First-time setup: allocate, then set the starting defaults — classic
 * Doom fire, full fuel, no wind, starting from cold. */
static void grid_init(Grid *grid, int cols, int rows, int theme) {
  grid_alloc(grid, cols, rows);
  grid->fuel = 1.0f;
  grid->wind = 0;
  grid->wind_acc = 0;
  grid->theme = theme;
  grid->warmup = 0;
  grid->algo = 0;
  grid->plasma_t = 0.f;
  grid->part_idx = 0;
}

/* ── §12 shared helpers ── */
/* Little tools that more than one engine reaches for: clamping numbers
 * to a range, the fade-in counter, the sideways drift of the fuel arch,
 * the bell shape of the fuel along the bottom, and the routines that
 * seed fuel and stamp a soft blob of heat. */

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Named random helpers, so the engines can say what a random value means
 * instead of spelling out rand() arithmetic:
 *   rand_unit()           — a random number from 0 up to 1
 *   rand_signed_unit()    — a random number from -0.5 up to 0.5
 *   rand_lateral_jitter() — a random one-cell nudge: -1, 0, or +1
 */
static inline float rand_unit(void) { return (float)rand() / RAND_MAX; }
static inline float rand_signed_unit(void) { return rand_unit() - 0.5f; }
static inline int rand_lateral_jitter(void) { return (rand() % 3) - 1; }

/* A fade-in dial that climbs from 0 to 1 over the first WARMUP_TICKS
 * ticks, so the fire builds up instead of popping in. Also ticks the
 * counter forward, so call it once per frame. */
static float warmup_scale_factor(Grid *grid) {
  float scale = (grid->warmup < WARMUP_TICKS)
                    ? (float)grid->warmup / (float)WARMUP_TICKS
                    : 1.f;
  if (grid->warmup < WARMUP_TICKS)
    grid->warmup++;
  return scale;
}

/* Slide the fuel arch sideways by one tick of wind. It snaps back to
 * centre once it has drifted a full screen width, so the fire never
 * wanders off and stays gone. */
static void advance_wind(Grid *grid) {
  grid->wind_acc += grid->wind;
  if (grid->wind_acc >= grid->cols || grid->wind_acc <= -grid->cols)
    grid->wind_acc = 0;
}

/* How much fuel a given column gets: a hump that is zero at the cold
 * margins, full in the middle, and falls off smoothly toward the edges.
 * The hump slides with the wind. Returns a weight from 0 to 1. */
static float arch_envelope(int x, int cols, int wind_acc) {
  float margin = (float)cols * ARCH_MARGIN_FRAC;
  float span = (float)cols - 2.f * margin;
  float shifted_x = (float)x - margin - (float)wind_acc;
  float t = (span > 0.f) ? shifted_x / span : 0.f;
  if (t < 0.f || t > 1.f)
    return 0.f;
  float edge = (t < 0.5f) ? t : 1.f - t;
  float weight = edge * 2.f;
  return weight * weight;
}

/* Lay down the hump of fuel along the bottom row — think a row of gas
 * burners, each turned up by the arch shape, the fuel knob, the warmup
 * fade, and a little random flicker. This is the heat the CA engine
 * draws upward from. */
static void seed_fuel_row(Grid *grid, float warmup_scale) {
  int cols = grid->cols;
  int fuel_y = grid->rows - 1;
  float *heat_grid = grid->heat;

  for (int x = 0; x < cols; x++) {
    float arch_weight = arch_envelope(x, cols, grid->wind_acc);
    if (arch_weight <= 0.f) {
      heat_grid[fuel_y * cols + x] = 0.f;
      continue;
    }

    float random_jitter = FUEL_JITTER_BASE + FUEL_JITTER_RANGE * rand_unit();
    float fuel_at_column =
        MAX_HEAT * grid->fuel * arch_weight * random_jitter * warmup_scale;
    heat_grid[fuel_y * cols + x] = fuel_at_column;
  }
}

/* Stamp a soft 3x3 blob of heat centred on a cell: the middle gets the
 * most, the corners the least, and the nine weights add up to 1 so the
 * blob spreads the heat without inventing any extra. Adds to whatever is
 * already there. Used to paint each rising ember. */
static void splat3x3(float *heat_grid, int cols, int rows, int cx, int cy,
                     float v) {
  static const float kernel[3][3] = {
      {0.0625f, 0.125f, 0.0625f},
      {0.125f, 0.25f, 0.125f},
      {0.0625f, 0.125f, 0.0625f},
  };
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int nx = cx + dx, ny = cy + dy;
      if (nx >= 0 && nx < cols && ny >= 0 && ny < rows)
        heat_grid[ny * cols + nx] += v * kernel[dy + 1][dx + 1];
    }
  }
}

/* ── §13 algo 0 — Doom CA fire ── */
/* Engine 0: the cellular automaton from Doom (1993). Each tick refreshes
 * the fuel along the bottom, then every cell above copies heat from the
 * cell below it (shifted a random step left or right) minus a little
 * fade. Heat drifts upward and dies out near the top. The fade is sized
 * to the screen so the flame reaches the same relative height on any
 * terminal. */

/* Work out how fast heat must fade per step so the flame top lands near
 * the target height. Roughly: fade-per-step times steps-to-the-top
 * should use up all the heat, so fade ~= full heat / number of rows it
 * should climb. Hands back a steady part and a random part via the two
 * out-pointers; the propagation loop adds them as
 * fade = base + random * range. Floored so tiny terminals still work. */
static void ca_compute_adaptive_decay(int rows, float *decay_base_out,
                                      float *decay_range_out) {
  float reach_height_in_rows = (float)rows * CA_REACH_FRAC;
  float average_decay_per_step = (reach_height_in_rows > 1.f)
                                     ? (MAX_HEAT / reach_height_in_rows)
                                     : MAX_HEAT;

  float decay_base = average_decay_per_step * CA_DECAY_BASE_FRAC;
  float decay_range = average_decay_per_step * CA_DECAY_RAND_FRAC;
  if (decay_base < CA_DECAY_BASE_MIN)
    decay_base = CA_DECAY_BASE_MIN;
  if (decay_range < CA_DECAY_RAND_MIN)
    decay_range = CA_DECAY_RAND_MIN;

  *decay_base_out = decay_base;
  *decay_range_out = decay_range;
}

/* The heart of the Doom trick: a cell's new heat is the heat from the
 * cell just below — nudged one step left, right, or not at all — minus a
 * little random fade, never going below zero. The sideways nudge is what
 * makes the flames lick and twist instead of rising in straight stripes. */
static float ca_propagate_one_cell(const float *heat_grid, int cols, int x,
                                   int y, float decay_base, float decay_range) {
  int lateral_offset = rand_lateral_jitter();
  int source_column = clamp_int(x + lateral_offset, 0, cols - 1);
  float source_heat = heat_grid[(y + 1) * cols + source_column];
  float energy_lost = decay_base + rand_unit() * decay_range;
  float propagated_heat = source_heat - energy_lost;
  return (propagated_heat < 0.f) ? 0.f : propagated_heat;
}

/* One frame of the Doom CA fire, in four plain steps. */
static void ca_fire_tick(Grid *grid) {
  int cols = grid->cols;
  int rows = grid->rows;
  float *heat_grid = grid->heat;

  /* fade the fire in and drift it with the wind */
  float warmup_scale = warmup_scale_factor(grid);
  advance_wind(grid);

  /* lay down fresh fuel along the bottom */
  seed_fuel_row(grid, warmup_scale);

  /* size the fade so the flame reaches the target height */
  float decay_base, decay_range;
  ca_compute_adaptive_decay(rows, &decay_base, &decay_range);

  /* pull heat upward, one cell at a time, from each row below */
  for (int y = 0; y < rows - 1; y++) {
    for (int x = 0; x < cols; x++) {
      heat_grid[y * cols + x] =
          ca_propagate_one_cell(heat_grid, cols, x, y, decay_base, decay_range);
    }
  }
}

/* ── §14 algo 1 — particle fire ── */
/* Engine 1: a pool of rising embers. Each tick every live ember moves
 * and cools, new embers are born along the fuel arch, and then each
 * ember stamps a soft blob of heat onto the grid for the shared renderer
 * to draw. */

/* Pick the column a new ember is born in, favouring the middle of the
 * arch. We pick a random spot and keep it with a chance equal to how
 * much fuel is there — so the busy centre gets picked more often. Try a
 * few times; if nothing sticks, just use the centre. */
static float rejection_sample_birth_column(Grid *grid, float warmup_scale) {
  int cols = grid->cols;
  float source_margin = (float)cols * ARCH_MARGIN_FRAC;
  float source_span = (float)cols - 2.f * source_margin;
  float fallback_centre = (float)cols * 0.5f;

  for (int dart = 0; dart < 8; dart++) {
    float t = rand_unit();
    float candidate_column =
        source_margin + t * source_span + (float)grid->wind_acc;
    float edge_distance = (t < 0.5f) ? t : 1.f - t;
    float arch_weight = (edge_distance * 2.f) * (edge_distance * 2.f);
    float acceptance_prob = arch_weight * grid->fuel * warmup_scale;
    if (rand_unit() < acceptance_prob)
      return candidate_column;
  }
  return fallback_centre;
}

/* Bring one new ember to life at the bottom of the arch: choose its
 * column, place it, give it a small sideways kick and a strong upward
 * push, and set how long it will live. */
static void fire_part_spawn(FirePart *particle, Grid *grid,
                            float warmup_scale) {
  /* choose a column, leaning toward the centre */
  float birth_column = rejection_sample_birth_column(grid, warmup_scale);

  /* start it on the bottom row */
  particle->x = birth_column;
  particle->y = (float)(grid->rows - 1);

  /* small sideways kick, strong push upward */
  float initial_vx = rand_signed_unit() * PART_VX_SPREAD;
  float initial_vy = -(PART_VY_BASE + rand_unit() * PART_VY_RANGE);
  particle->vx = initial_vx;
  particle->vy = initial_vy;

  /* turn a lifetime into a per-tick cooling rate */
  float lifetime_ticks = PART_LIFE_MIN + rand_unit() * PART_LIFE_RANGE;
  particle->heat = 1.0f;
  particle->decay = 1.0f / lifetime_ticks;
  particle->active = true;
}

/* Move one ember for a tick: give it a random sideways wobble, let that
 * drift settle a little, slide it by its speed, and cool it. If it has
 * burned out or wandered off the grid, mark its slot free. */
static void particle_advance_one(FirePart *ember, int cols) {
  float turbulence_kick = rand_signed_unit() * PART_TURB_STEP;
  ember->vx = (ember->vx + turbulence_kick) * PART_VX_DAMP;
  ember->x += ember->vx;
  ember->y += ember->vy;
  ember->heat -= ember->decay;

  bool burned_out = (ember->heat <= 0.f);
  bool left_top_edge = (ember->y < 0.f);
  bool left_side_edge = (ember->x < 0.f) || (ember->x >= (float)cols);
  if (burned_out || left_top_edge || left_side_edge)
    ember->active = false;
}

/* Move every live ember forward one tick. */
static void phase_a_advance_active_particles(Grid *grid) {
  int cols = grid->cols;
  for (int i = 0; i < MAX_FIRE_PARTS; i++) {
    FirePart *ember = &grid->parts[i];
    if (!ember->active)
      continue;
    particle_advance_one(ember, cols);
  }
}

/* Light up a batch of new embers this tick (fewer while still warming
 * up). For each one, scan the pool for a free slot and fill it. */
static void phase_b_spawn_new_particles(Grid *grid, float warmup_scale) {
  int spawn_target = (int)((float)SPAWN_PER_TICK * warmup_scale) + 1;

  for (int s = 0; s < spawn_target; s++) {
    for (int tries = 0; tries < MAX_FIRE_PARTS; tries++) {
      grid->part_idx = (grid->part_idx + 1) % MAX_FIRE_PARTS;
      FirePart *slot = &grid->parts[grid->part_idx];
      if (!slot->active) {
        fire_part_spawn(slot, grid, warmup_scale);
        break;
      }
    }
  }
}

/* Wipe the heat grid, then stamp each live ember's soft blob onto it.
 * This turns the moving embers back into a plain heat grid that the
 * shared renderer knows how to draw. */
static void phase_c_splat_all_to_grid(Grid *grid) {
  int cols = grid->cols, rows = grid->rows;

  memset(grid->heat, 0, (size_t)(cols * rows) * sizeof(float));

  for (int i = 0; i < MAX_FIRE_PARTS; i++) {
    const FirePart *ember = &grid->parts[i];
    if (!ember->active)
      continue;
    int splat_centre_x = (int)(ember->x + 0.5f);
    int splat_centre_y = (int)(ember->y + 0.5f);
    splat3x3(grid->heat, cols, rows, splat_centre_x, splat_centre_y,
             ember->heat);
  }
}

/* Where several embers pile up, their blobs add together and can push a
 * cell past full heat. Cap every cell at MAX_HEAT so the renderer always
 * sees a value in range. */
static void phase_d_clamp_oversaturation(Grid *grid) {
  int total_cells = grid->cols * grid->rows;
  for (int i = 0; i < total_cells; i++)
    if (grid->heat[i] > MAX_HEAT)
      grid->heat[i] = MAX_HEAT;
}

/* One frame of the particle fire: warm up and drift, move the embers,
 * spawn new ones, paint them onto the grid, then cap the heat. */
static void particle_fire_tick(Grid *grid) {
  float warmup_scale = warmup_scale_factor(grid);
  advance_wind(grid);

  phase_a_advance_active_particles(grid);
  phase_b_spawn_new_particles(grid, warmup_scale);
  phase_c_splat_all_to_grid(grid);
  phase_d_clamp_oversaturation(grid);
}

/* ── §15 algo 2 — plasma fire ── */
/* Engine 2: a fire drawn straight from math, with no memory between
 * frames except an animation clock. For each column we work out how tall
 * the flame reaches, then shade the cells below that height from hot at
 * the bottom to cold at the tip. */

/* How tall the flame reaches in one column. We add three sine waves of
 * different sizes and speeds — chosen so they never line up the same way
 * twice, which makes the flicker look organic — then scale the result by
 * the fuel, the warmup fade, and the arch shape. The arch is softened
 * with a square root so the flame tapers off gently at the edges. */
static float plasma_tongue_height(float wind_shifted_x, float phase_t,
                                  float fuel, float warmup_scale,
                                  float arch_weight) {
  float harmonic_1 = PLASMA_H1_AMP * sinf(wind_shifted_x * PLASMA_H1_XFREQ +
                                          phase_t * PLASMA_H1_TSPD);
  float harmonic_2 = PLASMA_H2_AMP * sinf(wind_shifted_x * PLASMA_H2_XFREQ -
                                          phase_t * PLASMA_H2_TSPD);
  float harmonic_3 = PLASMA_H3_AMP * sinf(wind_shifted_x * PLASMA_H3_XFREQ +
                                          phase_t * PLASMA_H3_TSPD);

  float raw_tongue = PLASMA_BASE + harmonic_1 + harmonic_2 + harmonic_3;
  float clamped_tongue = clampf(raw_tongue, 0.f, 1.f);
  float final_tongue =
      clamped_tongue * fuel * warmup_scale * sqrtf(arch_weight);
  return final_tongue;
}

/* Paint one column of the plasma flame: full heat at the bottom fading
 * smoothly to zero at the flame's tip, and nothing above the tip. */
static void plasma_shade_column(Grid *grid, int x, float tongue_height) {
  int cols = grid->cols;
  int rows = grid->rows;
  float inverse_tongue =
      (tongue_height > 0.01f) ? (1.f / tongue_height) : 100.f;
  float tongue_tip_ny = 1.f - tongue_height;

  for (int y = 0; y < rows; y++) {
    float normalised_y = (float)y / (float)rows;
    float above_tongue_tip = normalised_y - tongue_tip_ny;
    float heat_value = clampf(above_tongue_tip * inverse_tongue, 0.f, 1.f);
    grid->heat[y * cols + x] = heat_value;
  }
}

/* Blank a whole column — used for the cold margins outside the arch. */
static void plasma_clear_column(Grid *grid, int x) {
  int cols = grid->cols;
  int rows = grid->rows;
  for (int y = 0; y < rows; y++)
    grid->heat[y * cols + x] = 0.f;
}

/* One frame of the plasma fire: for every column, either blank it (cold
 * margin) or work out its flame height and shade it from hot to cold. */
static void plasma_fire_tick(Grid *grid) {
  int cols = grid->cols;

  /* fade in and drift with the wind */
  float warmup_scale = warmup_scale_factor(grid);
  advance_wind(grid);

  /* tick the animation clock forward */
  float phase_t = grid->plasma_t;
  grid->plasma_t += PLASMA_TIME_STEP;

  /* for each column: find the flame height, then shade it */
  for (int x = 0; x < cols; x++) {
    float normalised_x = (float)x / (float)cols;
    float wind_shifted_x = normalised_x + (float)grid->wind_acc / (float)cols;
    float arch_weight = arch_envelope(x, cols, grid->wind_acc);

    if (arch_weight <= 0.f) {
      plasma_clear_column(grid, x);
      continue;
    }

    float tongue_height = plasma_tongue_height(
        wind_shifted_x, phase_t, grid->fuel, warmup_scale, arch_weight);
    plasma_shade_column(grid, x, tongue_height);
  }
}

/* ── §16 dispatch ── */
/* The one spot that checks which engine is selected, so the rest of the
 * code never has to. Pressing 'a' changes the choice. */

/* Run one tick of whichever engine is currently selected. */
static void grid_tick(Grid *grid) {
  switch (grid->algo) {
  case 0:
    ca_fire_tick(grid);
    break;
  case 1:
    particle_fire_tick(grid);
    break;
  case 2:
    plasma_fire_tick(grid);
    break;
  default:
    break;
  }
}

/* ── §17 render pipeline ── */
/* The shared renderer that turns the heat grid into characters,
 * whichever engine filled it. It adjusts each cell's brightness so the
 * shading looks even to the eye, spreads the rounding error to
 * neighbours so the gradient stays smooth, draws the matching character,
 * and erases cells that just went cold. Cells that were already cold are
 * skipped so the screen update stays tiny. */

/* Which diagnostic overlay is showing, if any. The debug helper that
 * uses these modes lives down in §18; this declares it early so the
 * renderer can call it. */
typedef enum {
  DEBUG_OFF = 0,             /* normal fire                            */
  DEBUG_RAW_HEAT = 1,        /* heat with no brightness adjustment     */
  DEBUG_GAMMA_NO_DITHER = 2, /* adjusted brightness but no smoothing   */
  DEBUG_ARCH_ENVELOPE = 3,   /* just the fuel-arch shape               */
} DebugMode;

static DebugMode g_debug_mode = DEBUG_OFF;

static void debug_fill_dither_buffer(Grid *grid);

/* Pass 1: copy each cell's heat into the scratch buffer, adjusted so
 * equal steps look equally bright to the eye (our eyes don't see
 * brightness in a straight line). Cold cells get -1 as a flag, which
 * tells the next pass to erase or skip them instead of drawing. */
static void pipeline_pass1_gamma_correct(Grid *grid) {
  int total_cells = grid->cols * grid->rows;
  float *heat_grid = grid->heat;
  float *dither_buffer = grid->dither;

  for (int i = 0; i < total_cells; i++) {
    float linear_heat = heat_grid[i];
    if (linear_heat <= 0.f) {
      dither_buffer[i] = -1.f;
      continue;
    }
    float saturated_heat = fminf(1.f, linear_heat / MAX_HEAT);
    float perceptual = powf(saturated_heat, 1.f / 2.2f);
    dither_buffer[i] = perceptual;
  }
}

/* Spread one cell's rounding error onto its not-yet-drawn neighbours so
 * the gradient stays smooth instead of banding (the classic
 * Floyd-Steinberg pattern: most goes right, the rest to the three cells
 * below). Cells already flagged cold are skipped so they don't flicker. */
static void pipeline_diffuse_quant_error(float *dither_buffer, int cols,
                                         int rows, int x, int y,
                                         float quant_error) {
  int i = y * cols + x;

  if (x + 1 < cols && dither_buffer[i + 1] >= 0.f)
    dither_buffer[i + 1] += quant_error * (7.f / 16.f);

  if (y + 1 < rows) {
    if (x - 1 >= 0 && dither_buffer[i + cols - 1] >= 0.f)
      dither_buffer[i + cols - 1] += quant_error * (3.f / 16.f);
    if (dither_buffer[i + cols] >= 0.f)
      dither_buffer[i + cols] += quant_error * (5.f / 16.f);
    if (x + 1 < cols && dither_buffer[i + cols + 1] >= 0.f)
      dither_buffer[i + cols + 1] += quant_error * (1.f / 16.f);
  }
}

/* Draw one lit cell: pick the character for its brightness, optionally
 * pass the leftover error to its neighbours, and paint it in the theme's
 * colour. */
static void pipeline_draw_lit_cell(Grid *grid, int x, int y, float perceptual,
                                   bool apply_dither) {
  int bucket = lut_index(perceptual);

  if (apply_dither) {
    float bucket_midpoint = lut_midpoint(bucket);
    float quant_error = perceptual - bucket_midpoint;
    pipeline_diffuse_quant_error(grid->dither, grid->cols, grid->rows, x, y,
                                 quant_error);
  }

  attr_t glyph_attr = ramp_attr(bucket, grid->theme);
  attron(glyph_attr);
  mvaddch(y, x, (chtype)(unsigned char)k_ramp[bucket]);
  attroff(glyph_attr);
}

/* Pass 2: walk every on-screen cell. Draw the lit ones; for a cell now
 * cold but lit last frame, write a single space to wipe the old
 * character; leave cells that were already cold alone. */
static void pipeline_pass2_quantise_and_draw(Grid *grid, int tcols, int trows,
                                             bool apply_dither) {
  int cols = grid->cols;
  int rows = grid->rows;
  float *dither_buffer = grid->dither;
  float *previous_heat = grid->prev_heat;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;

      int i = y * cols + x;
      float perceptual = dither_buffer[i];

      if (perceptual < 0.f) {
        bool was_lit_last_frame = (previous_heat[i] > 0.f);
        if (was_lit_last_frame)
          mvaddch(y, x, ' ');
        continue;
      }

      pipeline_draw_lit_cell(grid, x, y, perceptual, apply_dither);
    }
  }
}

/* Pass 3: remember this frame's heat as "last frame" so the next frame
 * can tell which cells just went cold. We swap the two buffers (cheap)
 * and copy back, leaving the working buffer holding the same values for
 * the next round of engine writes. */
static void pipeline_pass3_archive_current_frame(Grid *grid) {
  int total_cells = grid->cols * grid->rows;
  float *tmp = grid->prev_heat;
  grid->prev_heat = grid->heat;
  grid->heat = tmp;
  memcpy(grid->heat, grid->prev_heat, (size_t)total_cells * sizeof(float));
}

/* Draw one frame by running the three passes in order. In a debug mode
 * the first pass is swapped for a diagnostic fill and the smoothing is
 * turned off. */
static void grid_draw(Grid *grid, int tcols, int trows) {
  bool is_normal_render = (g_debug_mode == DEBUG_OFF);

  /* fill the scratch buffer: normal brightness, or a debug overlay */
  if (is_normal_render)
    pipeline_pass1_gamma_correct(grid);
  else
    debug_fill_dither_buffer(grid);

  /* draw the cells; only smooth in normal mode */
  pipeline_pass2_quantise_and_draw(grid, tcols, trows, is_normal_render);

  /* save this frame so the next one can spot the changes */
  pipeline_pass3_archive_current_frame(grid);
}

/* ── §18 debug overlays ── */
/* Three teaching views the user cycles with 'd' and 'D'. Each one only
 * changes what the renderer feeds in, so the drawing stays the same:
 *   raw-heat   — skip the brightness fix; the coarse banding shows why
 *                the fix is needed.
 *   gamma-only — keep the fix but drop the smoothing; the leftover
 *                banding shows why the smoothing is needed.
 *   arch       — show just the fuel-arch shape every engine sits on. */

/* Fill the scratch buffer for whichever debug view is active, instead of
 * the normal brightness fill. Each view is a different rule for "what
 * brightness goes here"; the rest of the renderer is reused as-is. */
static void debug_fill_dither_buffer(Grid *grid) {
  int cols = grid->cols;
  int rows = grid->rows;
  float *heat_grid = grid->heat;
  float *dither_buffer = grid->dither;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      int i = y * cols + x;
      float v;

      switch (g_debug_mode) {
      case DEBUG_RAW_HEAT:
        v = heat_grid[i];
        if (v <= 0.f) {
          dither_buffer[i] = -1.f;
          continue;
        }
        v = fminf(1.f, v / MAX_HEAT);
        break;

      case DEBUG_GAMMA_NO_DITHER:
        v = heat_grid[i];
        if (v <= 0.f) {
          dither_buffer[i] = -1.f;
          continue;
        }
        v = powf(fminf(1.f, v / MAX_HEAT), 1.f / 2.2f);
        break;

      case DEBUG_ARCH_ENVELOPE:
        v = arch_envelope(x, cols, grid->wind_acc);
        if (v <= 0.f) {
          dither_buffer[i] = -1.f;
          continue;
        }
        break;

      default:
        dither_buffer[i] = -1.f;
        continue;
      }

      dither_buffer[i] = v;
    }
  }
}

/* Short label for the active overlay, shown in the HUD. */
static const char *debug_mode_name(DebugMode mode) {
  switch (mode) {
  case DEBUG_OFF:
    return "off";
  case DEBUG_RAW_HEAT:
    return "raw-heat";
  case DEBUG_GAMMA_NO_DITHER:
    return "gamma-only";
  case DEBUG_ARCH_ENVELOPE:
    return "arch";
  default:
    return "?";
  }
}

/* ── §19 scene ── */
/* Wraps the grid with two bits of UI state: whether we're paused, and a
 * one-shot "clear the screen next frame" flag. The clear flag is raised
 * after switching engine, theme, or size so leftover characters from the
 * old look don't linger. */

typedef struct {
  Grid grid;
  bool paused;      /* SPACE toggles this; pauses the simulation */
  bool needs_clear; /* wipe the screen once on the next frame    */
} Scene;

static void scene_init(Scene *scene, int cols, int rows, int theme) {
  memset(scene, 0, sizeof *scene);
  grid_init(&scene->grid, cols, rows, theme);
}

static void scene_free(Scene *scene) { grid_free(&scene->grid); }

/* Handle a terminal resize: rebuild the grid at the new size, but keep
 * the user's theme, fuel, wind, and engine choices. The fire fades back
 * in from cold. */
static void scene_resize(Scene *scene, int cols, int rows) {
  int saved_theme = scene->grid.theme;
  float saved_fuel = scene->grid.fuel;
  int saved_wind = scene->grid.wind;
  int saved_algo = scene->grid.algo;

  grid_resize(&scene->grid, cols, rows);

  scene->grid.fuel = saved_fuel;
  scene->grid.wind = saved_wind;
  scene->grid.theme = saved_theme;
  scene->grid.algo = saved_algo;
  scene->grid.warmup = 0;
  scene->needs_clear = true;
}

static void scene_tick(Scene *scene) {
  if (scene->paused)
    return;
  grid_tick(&scene->grid);
}

static void scene_draw(Scene *scene, int cols, int rows) {
  grid_draw(&scene->grid, cols, rows);
}

/* ── §20 screen + hud ── */
/* The only place that talks to ncurses for setup and the status display.
 * The HUD is three lines: frame rate and pause state up top right, the
 * current settings just below, and the list of keys along the bottom. */

typedef struct {
  int cols, rows; /* current terminal size in cells */
} Screen;

static void screen_init(Screen *screen, int initial_theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let input interrupt diff write */
  color_init(initial_theme);
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_free(Screen *screen) {
  (void)screen;
  endwin();
}

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static const char *algo_name(int algo_index) {
  switch (algo_index) {
  case 0:
    return "CA";
  case 1:
    return "particles";
  case 2:
    return "plasma";
  default:
    return "?";
  }
}

/* Draw one whole frame: the fire, then the HUD on top of it. */
static void screen_draw(Screen *screen, Scene *scene, double fps, int sim_fps) {
  if (scene->needs_clear) {
    erase();
    scene->needs_clear = false;
  }
  scene_draw(scene, screen->cols, screen->rows);

  const Grid *grid = &scene->grid;
  char buf[HUD_COLS + 1];

  /* top line: frame rate, sim speed, paused state */
  snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ", fps, sim_fps,
           scene->paused ? "PAUSED " : "running");
  int row0_x = screen->cols - (int)strlen(buf);
  if (row0_x < 0)
    row0_x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, row0_x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* second line: current settings */
  const char *wind_arrow = grid->wind > 0   ? ">>>"
                           : grid->wind < 0 ? "<<<"
                                            : "---";
  snprintf(buf, sizeof buf, " theme:%s  algo:%s  fuel:%.2f  wind:%s  dbg:%s ",
           k_themes[grid->theme].name, algo_name(grid->algo), grid->fuel,
           wind_arrow, debug_mode_name(g_debug_mode));
  int row1_x = screen->cols - (int)strlen(buf);
  if (row1_x < 0)
    row1_x = 0;
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, row1_x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* bottom line: the keys you can press */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(screen->rows - 1, 0,
           " q:quit  spc:pause  a:algo  t:theme  g/G:fuel  w/W:wind  0:calm  "
           "[/]:Hz  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §21 application ── */
/* The outermost layer: signal handlers, key handling, and the main loop.
 * The picture redraws at a steady 60 fps while the simulation runs at its
 * own rate. Ctrl-C asks the loop to stop; a resize asks it to rebuild the
 * grid on the next pass. */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                          /* simulation ticks per second  */
  volatile sig_atomic_t running;        /* cleared by a quit signal     */
  volatile sig_atomic_t need_resize;    /* raised by a resize signal    */
} App;

static App g_app;
static void on_exit_signal(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize_signal(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup_on_exit(void) { endwin(); }

/* Act on one key press. Returns false only when the user asks to quit
 * (q, Q, or ESC); every other key just changes a setting. One case per
 * key, so adding a control means adding a case. */
static bool app_handle_key(App *app, int ch) {
  Grid *grid = &app->scene.grid;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    app->scene.paused = !app->scene.paused;
    break;

  case 'a':
  case 'A':
    grid->algo = (grid->algo + 1) % N_ALGOS;
    grid->warmup = 0;
    grid->wind_acc = 0;
    memset(grid->parts, 0, sizeof grid->parts);
    grid->part_idx = 0;
    app->scene.needs_clear = true;
    break;

  case 't':
  case 'T':
    grid->theme = (grid->theme + 1) % THEME_COUNT;
    theme_apply(grid->theme);
    app->scene.needs_clear = true;
    break;

  case 'g':
    grid->fuel += 0.05f;
    if (grid->fuel > 1.0f)
      grid->fuel = 1.0f;
    break;
  case 'G':
    grid->fuel -= 0.05f;
    if (grid->fuel < 0.1f)
      grid->fuel = 0.1f;
    break;

  case 'w':
    grid->wind++;
    if (grid->wind > WIND_MAX)
      grid->wind = WIND_MAX;
    break;
  case 'W':
    grid->wind--;
    if (grid->wind < -WIND_MAX)
      grid->wind = -WIND_MAX;
    break;
  case '0':
    grid->wind = 0;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  case 'd':
    g_debug_mode = (DebugMode)(((int)g_debug_mode + 1) % N_DEBUG_MODES);
    app->scene.needs_clear = true;
    break;
  case 'D':
    g_debug_mode =
        (DebugMode)(((int)g_debug_mode + N_DEBUG_MODES - 1) % N_DEBUG_MODES);
    app->scene.needs_clear = true;
    break;

  default:
    break;
  }
  return true;
}

/* The program's heartbeat. Each pass: handle a resize if one happened,
 * measure how much time passed, run as many simulation ticks as that
 * time allows, update the frame-rate readout, sleep to hold 60 fps, draw
 * the frame, then read one key. The fixed-time-per-tick trick keeps the
 * fire moving at the same speed no matter how fast the terminal draws. */
int main(void) {
  srand((unsigned int)clock_ns());
  atexit(cleanup_on_exit);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen, 0);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

  int64_t frame_time_ns = clock_ns();
  int64_t sim_accum_ns = 0;
  int64_t fps_window_ns = 0;
  int fps_frame_count = 0;
  double fps_smoothed = 0.0;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      frame_time_ns = clock_ns();
      sim_accum_ns = 0;
    }

    int64_t now = clock_ns();
    int64_t delta_ns = now - frame_time_ns;
    frame_time_ns = now;
    if (delta_ns > 100 * NS_PER_MS)
      delta_ns = 100 * NS_PER_MS;

    int64_t tick_period_ns = TICK_NS(app->sim_fps);
    sim_accum_ns += delta_ns;
    while (sim_accum_ns >= tick_period_ns) {
      scene_tick(&app->scene);
      sim_accum_ns -= tick_period_ns;
    }

    fps_frame_count++;
    fps_window_ns += delta_ns;
    if (fps_window_ns >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_smoothed = (double)fps_frame_count /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      fps_frame_count = 0;
      fps_window_ns = 0;
    }

    int64_t elapsed_this_iteration = clock_ns() - frame_time_ns + delta_ns;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed_this_iteration);

    screen_draw(&app->screen, &app->scene, fps_smoothed, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
