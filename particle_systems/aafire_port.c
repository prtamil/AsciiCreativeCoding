/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aafire_port.c — the classic aalib "aafire" terminal flame, rebuilt here.
 *
 * Heat sits in a grid of bytes (0 = cold, 255 = white-hot). Each frame the
 * bottom two rows are reseeded with random "fuel", then every cell cools to
 * a blend of the few cells below it — so heat drifts upward and the flame
 * climbs. We then turn the heat numbers into the glyphs " .:+x*X#@" and
 * paint them in one of 10 colour themes.
 *
 * Ports the fire automaton from aalib 1.4 (Jan Hubička, aafire.c).
 * Floyd-Steinberg dithering: Floyd & Steinberg (1976), Proc. SID 17(2): 75.
 *
 * Keys: q/ESC quit · space pause · t/T theme · d/D debug view · g/G fuel · ]/[ speed
 * Build: gcc -std=c11 -O2 -Wall -Wextra particle_systems/aafire_port.c -o aafire_port -lncurses -lm
 *
 * ── §1  config            tunable constants — speed, fuel, timing, palette
 * ── §2  clock             a steady nanosecond timer and a sleep helper
 * ── §3  LUT               turn a brightness 0..1 into one of the glyphs
 * ── §4  themes            10 colour palettes + the code that applies one
 * ── §5  bitmap state      the Bitmap struct that holds all the fire state
 * ── §6  decay table       precompute how much each cell cools per frame
 * ── §7  propagation       the core step: cool every cell from those below it
 * ── §8  fuel seeding      reseed the bottom rows so the flame keeps burning
 * ── §9  bitmap lifecycle  allocate / free / init / one-step driver
 * ── §10 render pipeline   heat → glyph on screen (with dithering)
 * ── §11 scene             ties it together: pause, debug view, tick, draw
 * ── §12 debug overlay     extra views for poking at the internals
 * ── §13 screen + HUD      ncurses setup + the on-screen status text
 * ── §14 app               signal handlers + keypress handling
 * ── §15 main              the main loop with a steady timestep
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

/* ── §1  config ── */

enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  HUD_COLS = 52,
  FPS_UPDATE_MS = 500,
  CYCLE_TICKS = 300, /* frames between automatic theme switches */
};

/*
 * Size of the cooling lookup table (see §6). The biggest number we ever
 * look up is the sum of 5 cells, each at most 255, so 5*255 = 1275. We
 * round up to 1280 — the same size the original aalib used.
 */
#define MAXTABLE 1280

/* How strong the fuel burns, 0.1 (faint) to 1.0 (full). Adjustable with g/G. */
#define FUEL_DEFAULT 1.0f
#define FUEL_STEP 0.05f
#define FUEL_MIN 0.1f
#define FUEL_MAX 1.0f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2  clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3  LUT ── */

/* The 9 glyphs we draw, from coldest (space) to hottest ('@'). */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

/* Colour slots for the status text. We park them just past the theme's
 * 9 colour slots so switching themes never recolours the status line. */
#define PAIR_HUD (CP_BASE + RAMP_N)
#define PAIR_HINT (CP_BASE + RAMP_N + 1)

/* The brightness cut-offs that pick a glyph. They're not evenly spaced:
 * the eye notices changes in dim values more than bright ones, so the
 * dim end gets finer steps. A value at or above a cut-off uses that glyph. */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f, 0.500f, 0.620f, 0.750f, 0.900f,
};

/* Pick which of the 9 glyphs a brightness 0..1 lands on. */
static int lut_index(float v) {
  int idx = 0;
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (v >= k_lut_breaks[i]) {
      idx = i;
      break;
    }
  return idx;
}
static float lut_midpoint(int idx) {
  if (idx <= 0)
    return 0.f;
  if (idx >= RAMP_N - 1)
    return 1.f;
  return (k_lut_breaks[idx] + k_lut_breaks[idx + 1]) * 0.5f;
}

/* ── §4  themes ── */

/*
 * FireTheme — one colour scheme for the flame: a 9-step colour ramp that
 * runs from the coldest glyph to the hottest, matching k_ramp slot for slot.
 *
 *   name   the label shown in the status line (e.g. "fire", "ocean")
 *   fg256  colour for each of the 9 steps on a 256-colour terminal
 *   fg8    fallback colour for each step on a plain 8-colour terminal
 *   attr8  bold/dim/normal per step on 8-colour terminals — with only 8
 *          colours to work with, dim vs bold is how we fake more shades
 *
 * The list of themes (k_themes below) is fixed; Bitmap.theme just picks one
 * by index. theme_apply() loads the chosen scheme into the colour slots the
 * flame draws with. The status-line colours live outside those slots, so
 * the status text keeps its colour no matter which theme is active.
 */
typedef struct {
  const char *name;
  int fg256[RAMP_N];
  int fg8[RAMP_N];
  attr_t attr8[RAMP_N];
} FireTheme;

#define CP_BASE 1

/* The 10 themes you cycle through with t/T. Slot 0 is the empty/cold glyph
 * (a space), so its colour never actually shows. The other slots use bright
 * colours on purpose — dark ones would be invisible on a black terminal. */
static const FireTheme k_themes[] = {
    {"matrix",
     {232, 22, 28, 34, 40, 46, 82, 118, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"neon",
     {232, 53, 89, 125, 161, 197, 213, 219, 231},
     {COLOR_BLACK, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD,
      A_BOLD}},
    {"nova",
     {232, 52, 88, 124, 160, 196, 208, 220, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"ocean",
     {232, 17, 19, 21, 27, 33, 39, 123, 231},
     {COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
      COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_BOLD, A_BOLD,
      A_BOLD}},
    {"fire",
     {232, 52, 88, 124, 160, 196, 202, 214, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD,
      A_BOLD}},
    {"toxic",
     {232, 22, 28, 34, 106, 154, 190, 226, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {"gold",
     {232, 94, 130, 136, 172, 178, 214, 220, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"ice",
     {232, 17, 19, 21, 27, 33, 51, 123, 231},
     {COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
      COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {"aurora",
     {232, 22, 28, 34, 121, 159, 207, 219, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN,
      COLOR_CYAN, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"plasma",
     {232, 53, 91, 129, 165, 207, 213, 219, 231},
     {COLOR_BLACK, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_BOLD,
      A_BOLD}},
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* Load theme t's colours into the slots the flame draws with. */
static void theme_apply(int t) {
  const FireTheme *th = &k_themes[t];
  for (int i = 0; i < RAMP_N; i++) {
    if (COLORS >= 256)
      init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
    else
      init_pair(CP_BASE + i, th->fg8[i], COLOR_BLACK);
  }
}

/* Turn colour on, load the starting theme, and set the status-line colours.
 * The status-line colours are set once here; theme switches never touch them. */
static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);

  init_pair(PAIR_HUD, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, COLORS >= 256 ? 51 : COLOR_CYAN, -1);
}

/* The colour + style to draw glyph step i with, under the current theme. */
static attr_t ramp_attr(int i, int theme) {
  attr_t a = COLOR_PAIR(CP_BASE + i);
  if (COLORS >= 256) {
    if (i >= RAMP_N - 2)
      a |= A_BOLD;
  } else {
    a |= k_themes[theme].attr8[i];
  }
  return a;
}

/* ── §5  bitmap state ── */

/* Keep v inside [lo, hi]. */
static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
/* Keep v inside a single byte's range, 0..255. */
static inline int clamp_uchar_int(int v) { return clamp_int(v, 0, 255); }

/*
 * Bitmap — everything the fire needs lives here. There's exactly one of
 * these and every part of the simulation reads and writes it directly.
 *
 *   bmap    the heat grid, one byte per cell (0 cold .. 255 white-hot).
 *           It's two rows TALLER than the screen: the extra bottom two
 *           rows are the hidden "fuel" rows that feed the flame.
 *   prev    the heat grid as it was last frame. We compare against it to
 *           know which cells went cold and need erasing.
 *   dither  scratch space (one float per cell) used while smoothing the
 *           picture before drawing — see the render pipeline in §10.
 *   table   the cooling lookup table (§6): given the summed heat of a
 *           cell's lower neighbours, it gives back the cooled-down value.
 *
 *   cols, rows   screen size in cells (rows counts visible rows only,
 *                not the two fuel rows underneath).
 *   height       a warm-up counter — the flame grows taller over the
 *                first few frames instead of popping up at full size.
 *   loop         countdown that gates how often new fuel bursts appear.
 *   sloop        running tally of fuel sweeps (carried over from aalib).
 *   fuel         burn strength 0.1..1.0, set by the g/G keys.
 *   theme        which colour scheme is active.
 *   cycle_tick   frames since the last auto theme switch.
 *
 * Allocated and set up by bitmap_init at startup and again on resize;
 * freed by bitmap_free at exit. Sections §6–§10 all work on it in place.
 */
typedef struct {
  unsigned char *bmap;
  unsigned char *prev;
  float *dither;
  unsigned int table[MAXTABLE];
  int cols;
  int rows;
  int height;
  int loop;
  int sloop;
  float fuel;
  int theme;
  int cycle_tick;
} Bitmap;

/* ── §6  decay table ── */

/*
 * Work out one cell's new heat from the summed heat of its lower neighbours.
 * We take a fixed bite out first (so weak cells die off completely), then
 * average back down. Taking the bite before dividing means dim cells lose a
 * bigger share, which makes the flame fade out cleanly at the edges.
 */
static unsigned int decay_table_entry(int neighbour_sum, int cooling_per_row) {
  if (neighbour_sum <= cooling_per_row)
    return 0;
  return (unsigned int)(neighbour_sum - cooling_per_row) / 5;
}

/* Fill in the whole cooling table once so the per-frame loop is just a
 * lookup. How fast the flame cools depends on screen height, so we rebuild
 * this on startup and again whenever the window resizes. */
static void gentable(Bitmap *b) {
  int cooling_per_row = 800 / b->rows;
  if (cooling_per_row == 0)
    cooling_per_row = 1;

  for (int neighbour_sum = 0; neighbour_sum < MAXTABLE; neighbour_sum++)
    b->table[neighbour_sum] = decay_table_entry(neighbour_sum, cooling_per_row);
}

/* ── §7  propagation ── */

/*
 * Add up the heat of the five cells just below cell (x, y). This is the
 * heart of the flame: a cell becomes a blend of what's underneath it, so
 * heat spreads upward. The five cells form this shape:
 *
 *      .   .   .          row y      (the cell we're filling in)
 *      a   b   c          row y+1    (the three directly below)
 *      d       e          row y+2    (two more, skipping the middle)
 *
 * Skipping that middle cell on the lower row pulls heat in from the sides,
 * which is what gives aafire its rounded blobs instead of straight streaks.
 * On the left/right edges we reuse the edge column so we never read off-grid,
 * and the two extra fuel rows keep row y+2 in bounds for the bottom cells.
 */
static unsigned int sample_five_neighbours_below(const unsigned char *bmap,
                                                 int cols, int x, int y) {
  int left_column = clamp_int(x - 1, 0, cols - 1);
  int right_column = clamp_int(x + 1, 0, cols - 1);
  int row_below = y + 1;
  int row_deeper = y + 2;

  unsigned int below_left = bmap[row_below * cols + left_column];
  unsigned int below_centre = bmap[row_below * cols + x];
  unsigned int below_right = bmap[row_below * cols + right_column];
  unsigned int deeper_left = bmap[row_deeper * cols + left_column];
  unsigned int deeper_right = bmap[row_deeper * cols + right_column];

  return below_left + below_centre + below_right + deeper_left + deeper_right;
}

/*
 * The main step: cool every visible cell to a blend of the cells below it,
 * and the whole flame rises by one frame's worth.
 *
 * We sweep top to bottom on purpose. If we went bottom to top, a cell would
 * sometimes read a neighbour we'd already updated this same frame, and the
 * flame would shoot upward too fast. Going top-down means each cell still
 * sees last frame's heat below it, which keeps the motion steady.
 *
 * Cheap by design: one pass over the grid, no memory allocation, and no
 * division in the loop — the cooling table in §6 did all the dividing up front.
 */
static void firemain(Bitmap *b) {
  int cols = b->cols;
  int rows = b->rows;
  unsigned char *bmap = b->bmap;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      unsigned int neighbour_sum =
          sample_five_neighbours_below(bmap, cols, x, y);
      unsigned int table_index =
          (neighbour_sum < MAXTABLE) ? neighbour_sum : MAXTABLE - 1;
      bmap[y * cols + x] = (unsigned char)b->table[table_index];
    }
  }
}

/* ── §8  fuel seeding ── */

/*
 * ArchSweep — bookkeeping for walking across the fuel row while shaping the
 * flame into an arch: tall in the middle, low at the two edges.
 *
 *   column   which column we're seeding right now (0 up to cols)
 *   i1       a left-edge limit: starts small and climbs as we move right
 *   i2       a right-edge limit: starts large and shrinks as we move right
 *
 * For each column the most heat we allow is the smaller of i1 and i2 (plus
 * the warm-up height). Near the left edge i1 is the small one; near the right
 * edge i2 is; in the middle both are large — that's what makes the arch. Both
 * limits move in steps of 4, an aalib tweak that makes the arch's sides curve
 * a bit more sharply than a straight ramp would.
 */
typedef struct {
  int column;
  int i1;
  int i2;
} ArchSweep;

/* The most heat this column may get: whichever limit is smallest, but never
 * below 1 so there's always at least a flicker of fuel. */
static int arch_sweep_cap(const ArchSweep *sweep, int height) {
  int cap = (sweep->i1 < sweep->i2) ? sweep->i1 : sweep->i2;
  if (height < cap)
    cap = height;
  if (cap < 1)
    cap = 1;
  return cap;
}

/* Step one column to the right and nudge both edge limits along with it. */
static void arch_sweep_advance_one_cell(ArchSweep *sweep) {
  sweep->column += 1;
  sweep->i1 += 4;
  sweep->i2 -= 4;
}

/*
 * Lay down one short "burst" of fuel — up to six cells in a row. As we go we
 * jitter the heat up or down a little each step, which makes the base of the
 * flame shimmer instead of sitting flat. Each burst starts from its own random
 * value, so one patch of flame can be much brighter than the patch next to it.
 */
static void seed_one_burst(Bitmap *b, ArchSweep *sweep, unsigned char *fuel_row,
                           unsigned char *fuel_row2) {
  int cap = arch_sweep_cap(sweep, b->height);
  int seed_value = (int)((float)(rand() % cap) * b->fuel);
  int burst_length = rand() % 6;

  for (int j = 0; j <= burst_length && sweep->column < b->cols; j++) {
    seed_value = clamp_uchar_int(seed_value);
    fuel_row[sweep->column] = (unsigned char)seed_value;
    seed_value = clamp_uchar_int(seed_value + rand() % 6 - 2);
    fuel_row2[sweep->column] = (unsigned char)seed_value;
    seed_value += rand() % 6 - 2;
    arch_sweep_advance_one_cell(sweep);
  }
}

/* Refill the two hidden fuel rows at the bottom of the grid. We walk across
 * them dropping a burst, leaving a one-cell gap, dropping another, and so on. */
static void drawfire(Bitmap *b) {
  int cols = b->cols;
  int rows = b->rows;
  unsigned char *fuel_row = b->bmap + rows * cols;         /* first hidden row */
  unsigned char *fuel_row_2 = b->bmap + (rows + 1) * cols; /* second hidden row */

  /* Age the warm-up counter, and tick down the burst timer; when it runs out,
   * reset it to a small random delay so bursts don't appear on a fixed beat. */
  b->height++;
  b->loop--;
  if (b->loop < 0) {
    b->loop = rand() % 3;
    b->sloop++;
  }

  ArchSweep sweep = {.column = 0, .i1 = 1, .i2 = 4 * cols + 1};
  while (sweep.column < cols) {
    seed_one_burst(b, &sweep, fuel_row, fuel_row_2);
    arch_sweep_advance_one_cell(&sweep); /* skip one cell so bursts don't merge */
  }
}

/* ── §9  bitmap lifecycle ── */

/* Allocate the three grids. The heat grid gets two extra rows on the bottom
 * for the hidden fuel that the flame step reads from. calloc zeroes them, so
 * the fire starts out cold. */
static void bitmap_alloc(Bitmap *b, int cols, int rows) {
  b->cols = cols;
  b->rows = rows;
  b->bmap = calloc((size_t)(cols * (rows + 2)), sizeof(unsigned char));
  b->prev = calloc((size_t)(cols * rows), sizeof(unsigned char));
  b->dither = calloc((size_t)(cols * rows), sizeof(float));
}

/* Free the grids and wipe the struct so nothing dangles. */
static void bitmap_free(Bitmap *b) {
  free(b->bmap);
  free(b->prev);
  free(b->dither);
  memset(b, 0, sizeof *b);
}

/* Allocate the grids and set every value to its starting state. Run once at
 * startup and again each time the window is resized. */
static void bitmap_init(Bitmap *b, int cols, int rows, int theme) {
  bitmap_alloc(b, cols, rows);
  b->height = 0;
  b->loop = 0;
  b->sloop = 0;
  b->fuel = FUEL_DEFAULT;
  b->theme = theme;
  b->cycle_tick = 0;
  gentable(b);
}

/*
 * Advance the fire by one frame: drop fresh fuel, then let it rise.
 *
 * Fuel goes down first on purpose. The rising step reads the two bottom rows,
 * so if we seeded fuel afterward the new flames wouldn't show up until the
 * next frame — a visible one-frame lag.
 *
 * Returns true only on the frames where the theme just auto-switched. The
 * caller relies on that: it must wipe the screen before drawing, or leftover
 * glyphs in the old theme's colours would linger.
 */
static bool bitmap_tick(Bitmap *b) {
  drawfire(b);
  firemain(b);

  b->cycle_tick++;
  if (b->cycle_tick >= CYCLE_TICKS) {
    b->cycle_tick = 0;
    b->theme = (b->theme + 1) % THEME_COUNT;
    theme_apply(b->theme);
    return true;
  }
  return false;
}

/* ── §10  render pipeline ── */

/* Convert a raw heat byte to a brightness 0..1 that matches how the eye sees
 * it. Equal jumps in the raw number don't look equally bright, so we bend the
 * scale (the standard "gamma" curve) to even that out. */
static inline float gamma_correct_byte(unsigned char heat) {
  float linear = (float)heat / 255.f;
  return powf(linear, 1.f / 2.2f);
}

/* A marker we drop into the scratch buffer for cells that are stone cold.
 * It's negative so one "is this below zero?" check tells cold cells apart
 * from real brightness values, which are always 0 or above. */
#define COLD_SENTINEL (-1.0f)

/* Pass 1: fill the scratch buffer with each cell's eye-matched brightness,
 * marking the cold cells so later passes can skip them. */
static void pipeline_pass1_gamma_correct(Bitmap *b) {
  int total_cells = b->cols * b->rows;
  const unsigned char *heat_bitmap = b->bmap;
  float *dither_buffer = b->dither;

  for (int i = 0; i < total_cells; i++) {
    unsigned char linear_heat = heat_bitmap[i];
    if (linear_heat == 0) {
      dither_buffer[i] = COLD_SENTINEL;
      continue;
    }
    dither_buffer[i] = gamma_correct_byte(linear_heat);
  }
}

/* When we snap a cell to one of the 9 glyphs we lose a little brightness in
 * the rounding. Dithering hides that: we hand the leftover error to nearby
 * cells we haven't drawn yet so the whole picture averages out right. This is
 * the classic Floyd-Steinberg recipe — most of the error goes right, the rest
 * spreads to the three cells below. Cold cells don't take any. */
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

/* Draw one lit cell: pick its glyph, pass the rounding leftover to its
 * neighbours, and paint it in the theme colour. */
static void pipeline_draw_lit_cell(Bitmap *b, int x, int y, float perceptual) {
  int bucket = lut_index(perceptual);
  float bucket_midpoint = lut_midpoint(bucket);
  float quant_error = perceptual - bucket_midpoint;

  pipeline_diffuse_quant_error(b->dither, b->cols, b->rows, x, y, quant_error);

  attr_t glyph_attr = ramp_attr(bucket, b->theme);
  attron(glyph_attr);
  mvaddch(y, x, (chtype)(unsigned char)k_ramp[bucket]);
  attroff(glyph_attr);
}

/* Pass 2: walk every cell. If it's cold now but was lit last frame, blank it
 * out; otherwise draw it. We only touch cells that changed, which keeps the
 * terminal from flickering. */
static void pipeline_pass2_quantise_and_draw(Bitmap *b, int tcols, int trows) {
  int cols = b->cols;
  int rows = b->rows;
  const float *dither_buffer = b->dither;
  const unsigned char *previous_heat = b->prev;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;

      int i = y * cols + x;
      float perceptual = dither_buffer[i];

      if (perceptual < 0.f) {
        bool was_lit_last_frame = (previous_heat[i] > 0);
        if (was_lit_last_frame)
          mvaddch(y, x, ' ');
        continue;
      }

      pipeline_draw_lit_cell(b, x, y, perceptual);
    }
  }
}

/* Pass 3: remember this frame's heat. Next frame's pass 2 checks it to tell
 * which cells just went cold and need erasing. */
static void pipeline_pass3_archive_current_frame(Bitmap *b) {
  int total_cells = b->cols * b->rows;
  memcpy(b->prev, b->bmap, (size_t)total_cells * sizeof(unsigned char));
}

/* Draw the whole fire in three passes: brightness, then glyphs, then save the
 * frame for next time's comparison. */
static void bitmap_draw(Bitmap *b, int tcols, int trows) {
  pipeline_pass1_gamma_correct(b);
  pipeline_pass2_quantise_and_draw(b, tcols, trows);
  pipeline_pass3_archive_current_frame(b);
}

/* ── §11  scene ── */

/* The four views you cycle with d/D. The first is the real flame; the rest
 * are inspection views that show the raw data or skip a render step. */
typedef enum {
  DEBUG_OFF = 0,      /* the normal flame (full §10 pipeline)               */
  DEBUG_RAW = 1,      /* show each cell's heat byte as a hex digit          */
  DEBUG_FUEL = 2,     /* show only the two fuel rows                        */
  DEBUG_NODITHER = 3, /* the flame, but with the dithering turned off       */
  DEBUG_COUNT = 4,
} DebugMode;

/* The label shown for the current view in the status line. */
static const char *debug_mode_name(DebugMode m) {
  switch (m) {
  case DEBUG_OFF:
    return "off     ";
  case DEBUG_RAW:
    return "raw heat";
  case DEBUG_FUEL:
    return "fuel    ";
  case DEBUG_NODITHER:
    return "nodither";
  default:
    return "?       ";
  }
}

/*
 * Scene — the fire (bmap) plus a few flags that control how it's run and drawn.
 *
 *   bmap        the actual fire state from §5
 *   paused      space toggles this; when set, the fire stops advancing
 *   needs_clear when true, wipe the whole screen before the next draw. We set
 *               it after a theme switch, a resize, or a view change, because
 *               the normal draw only repaints cells that changed and can't
 *               clean up glyphs the new view won't overwrite.
 *   debug_mode  which of the four views (above) to draw
 *
 * Set up by scene_init, ticked and drawn each frame, freed by scene_free.
 * A resize rebuilds the fire but keeps your theme, fuel, and view choice.
 */
typedef struct {
  Bitmap bmap;
  bool paused;
  bool needs_clear;
  DebugMode debug_mode;
} Scene;

/* Defined down in §12, used here. */
static void scene_draw_debug(Scene *s, int cols, int rows);

/* Set up a fresh scene, starting on the normal (non-debug) view. */
static void scene_init(Scene *s, int cols, int rows, int theme) {
  memset(s, 0, sizeof *s);
  bitmap_init(&s->bmap, cols, rows, theme);
  s->debug_mode = DEBUG_OFF;
}

/* Release the fire's memory. */
static void scene_free(Scene *s) { bitmap_free(&s->bmap); }

/* Rebuild the fire at the new window size, but carry over the theme, fuel
 * level, and current view so a resize doesn't reset your settings. */
static void scene_resize(Scene *s, int cols, int rows) {
  int t = s->bmap.theme;
  float fuel = s->bmap.fuel;
  DebugMode debug_mode = s->debug_mode;

  bitmap_free(&s->bmap);
  bitmap_init(&s->bmap, cols, rows, t);

  s->bmap.fuel = fuel;
  s->debug_mode = debug_mode;
  s->needs_clear = true;
  gentable(&s->bmap); /* the cooling table depends on row count, so redo it */
}

/* Switch to the next/previous view. We force a full screen wipe because the
 * old view may have left glyphs the new one won't paint over. */
static void scene_cycle_debug_mode(Scene *s, int step) {
  int next = ((int)s->debug_mode + step + DEBUG_COUNT) % DEBUG_COUNT;
  s->debug_mode = (DebugMode)next;
  s->needs_clear = true;
}

/* Advance the fire one step, unless paused. If the theme just auto-switched,
 * remember to wipe the screen next draw. */
static void scene_tick(Scene *s) {
  if (!s->paused) {
    if (bitmap_tick(&s->bmap))
      s->needs_clear = true;
  }
}

/* Draw the scene: the normal flame, or one of the debug views. Keeping this a
 * simple either/or means the debug views never disturb the real render code. */
static void scene_draw(Scene *s, int cols, int rows) {
  if (s->debug_mode == DEBUG_OFF)
    bitmap_draw(&s->bmap, cols, rows);
  else
    scene_draw_debug(s, cols, rows);
}

/* ── §12  debug overlay ── */

/* Show a heat byte as a single hex digit 0..F (using its top half, so the
 * digit tracks roughly how hot the cell is). */
static inline char hex_digit_for_heat(unsigned char heat) {
  static const char k_hex[] = "0123456789ABCDEF";
  return k_hex[heat >> 4];
}

/* Raw-heat view: print each cell's heat as a hex digit, coloured by how hot
 * it is. Lets you read the actual numbers behind the flame. */
static void debug_render_raw_heat(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;
      unsigned char heat = b->bmap[y * cols + x];

      int bucket = heat >> 5; /* pick a colour from how hot the cell is */
      if (bucket >= RAMP_N)
        bucket = RAMP_N - 1;
      char glyph = hex_digit_for_heat(heat);
      attr_t a = ramp_attr(bucket, b->theme);

      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
}

/* Fuel-only view: clear the screen and draw just the two hidden fuel rows, so
 * you can watch the seeding that feeds the flame. */
static void debug_render_fuel_only(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  erase();

  for (int row_offset = 0; row_offset < 2; row_offset++) {
    int fuel_row = rows + row_offset;       /* which hidden row in the grid */
    int screen_row = rows - 2 + row_offset; /* where to show it on screen */
    if (screen_row < 0 || screen_row >= trows)
      continue;

    for (int x = 0; x < cols && x < tcols; x++) {
      unsigned char heat = b->bmap[fuel_row * cols + x];
      float perceptual = (heat == 0) ? 0.f : gamma_correct_byte(heat);
      int bucket = lut_index(perceptual);
      char glyph = k_ramp[bucket];
      attr_t a = ramp_attr(bucket, b->theme);
      attron(a);
      mvaddch(screen_row, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
}

/* No-dither view: draw the flame normally but skip the dithering, so you can
 * see how much smoother the dithered version looks by comparison. */
static void debug_render_no_dither(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;
      int i = y * cols + x;

      unsigned char heat = b->bmap[i];
      if (heat == 0) {
        if (b->prev[i] > 0)
          mvaddch(y, x, ' ');
        continue;
      }
      float perceptual = gamma_correct_byte(heat);
      int bucket = lut_index(perceptual);
      char glyph = k_ramp[bucket];
      attr_t a = ramp_attr(bucket, b->theme);
      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
  memcpy(b->prev, b->bmap, (size_t)(cols * rows) * sizeof(unsigned char));
}

/* Pick the right debug view to draw; anything unexpected falls back to the
 * normal flame. */
static void scene_draw_debug(Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_RAW:
    debug_render_raw_heat(s, cols, rows);
    break;
  case DEBUG_FUEL:
    debug_render_fuel_only(s, cols, rows);
    break;
  case DEBUG_NODITHER:
    debug_render_no_dither(s, cols, rows);
    break;
  default:
    bitmap_draw(&s->bmap, cols, rows);
    break;
  }
}

/* ── §13  screen + HUD ── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
  getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) {
  (void)s;
  endwin();
}
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_hud_top_right(const Screen *s, const Scene *sc,
                                      double fps, int sim_fps) {
  char buf[HUD_COLS + 1];
  snprintf(buf, sizeof buf, " %4.1f fps  [%s]  fuel:%.2f  dbg:%s  sim:%d ", fps,
           k_themes[sc->bmap.theme].name, sc->bmap.fuel,
           debug_mode_name(sc->debug_mode), sim_fps);

  int hud_x = s->cols - (int)strlen(buf);
  if (hud_x < 0)
    hud_x = 0;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, hud_x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_draw_hint_bottom(const Screen *s) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q/ESC:quit  space:pause  t/T:theme  d/D:debug  g/G:fuel  ]/[:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, Scene *sc, double fps, int sim_fps) {
  if (sc->needs_clear) {
    erase();
    sc->needs_clear = false;
  }
  scene_draw(sc, s->cols, s->rows);
  screen_draw_hud_top_right(s, sc, fps, sim_fps);
  screen_draw_hint_bottom(s);
}
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §14  app ── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_cycle_theme(App *a, int step) {
  Bitmap *b = &a->scene.bmap;
  int next = (b->theme + step + THEME_COUNT) % THEME_COUNT;
  b->theme = next;
  b->cycle_tick = 0;
  theme_apply(next);
  a->scene.needs_clear = true;
}

static void app_nudge_fuel(App *a, float step) {
  Bitmap *b = &a->scene.bmap;
  float v = b->fuel + step;
  if (v > FUEL_MAX)
    v = FUEL_MAX;
  if (v < FUEL_MIN)
    v = FUEL_MIN;
  b->fuel = v;
}

static void app_nudge_sim_fps(App *a, int step) {
  int v = a->sim_fps + step;
  if (v > SIM_FPS_MAX)
    v = SIM_FPS_MAX;
  if (v < SIM_FPS_MIN)
    v = SIM_FPS_MIN;
  a->sim_fps = v;
}

static bool app_handle_key(App *a, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    a->scene.paused = !a->scene.paused;
    break;

  case 't':
    app_cycle_theme(a, +1);
    break;
  case 'T':
    app_cycle_theme(a, -1);
    break;

  case 'd':
    scene_cycle_debug_mode(&a->scene, +1);
    break;
  case 'D':
    scene_cycle_debug_mode(&a->scene, -1);
    break;

  case 'g':
    app_nudge_fuel(a, +FUEL_STEP);
    break;
  case 'G':
    app_nudge_fuel(a, -FUEL_STEP);
    break;

  case ']':
    app_nudge_sim_fps(a, +SIM_FPS_STEP);
    break;
  case '[':
    app_nudge_sim_fps(a, -SIM_FPS_STEP);
    break;

  default:
    break;
  }
  return true;
}

/* ── §15  main ── */

int main(void) {
  srand((unsigned int)clock_ns());
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen, 0);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

  int64_t ft = clock_ns(), sa = 0, fa = 0;
  int fc = 0;
  double fpsd = 0.;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      ft = clock_ns();
      sa = 0;
    }

    int64_t now = clock_ns(), dt = now - ft;
    ft = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick = TICK_NS(app->sim_fps);
    sa += dt;
    while (sa >= tick) {
      scene_tick(&app->scene);
      sa -= tick;
    }
    float alpha = (float)sa / (float)tick;
    (void)alpha;

    fc++;
    fa += dt;
    if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
      fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    /* Hold the frame to ~60 fps. We sleep before drawing so the time spent
     * writing to the terminal doesn't make the frame rate wobble. */
    int64_t el = clock_ns() - ft + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - el);

    screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
