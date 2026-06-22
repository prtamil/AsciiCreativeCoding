/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * complex_flowfield.c — particles riding an invisible "wind", where the
 * wind can be generated four different ways (press 'a' to switch): swirly
 * fluid-like noise, spinning vortices, crossed waves, or a galaxy spiral.
 * All four plug into the same particle engine. Colours come from Inigo
 * Quilez's cosine palette (iquilezles.org/articles/palettes).
 *
 * Sister files: fluid/flowfield.c is the simpler single-mode version this
 * one builds on; fluid/navier_stokes.c is a real fluid solver (this only
 * fakes the look); physics/magnetic_field.c uses the same Biot-Savart math
 * in an electromagnetism setting.
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

/* §1  config — all the tunable knobs, grouped by what they affect */

enum {
  /* Render frame cap. */
  RENDER_FPS_CAP = 60,

  /* Simulation step rate. */
  SIM_HZ_MIN = 5,
  SIM_HZ_DEFAULT = 30,
  SIM_HZ_MAX = 60,
  SIM_HZ_STEP = 5,

  /* HUD recompute cadence. */
  FPS_RECOMPUTE_MS = 500,

  /* Tracer pool. */
  TRACERS_MIN = 50,
  TRACERS_DEFAULT = 400,
  TRACERS_MAX = 800,
  TRACERS_STEP = 50,

  /* Trail ring buffer (per tracer). */
  TRAIL_LEN_MIN = 3,
  TRAIL_LEN_DEFAULT = 18,
  TRAIL_LEN_MAX = 24,
  TRAIL_LEN_HARD_MAX = 24, /* compile-time array size */

  /* FBM octaves used by the curl-noise generator. */
  CURL_FBM_OCTAVES = 3,

  /* Cosine palette: 16 evenly-spaced samples become ncurses pairs. */
  PALETTE_PAIR_COUNT = 16,

  /* Number of themes. */
  THEME_COUNT = 6,

  /* Number of background modes (blank / arrows / colormap). */
  BG_MODE_COUNT = 3,

  /* Number of field generator kinds (curl / vortex / sine / spiral). */
  FIELD_KIND_COUNT = 4,

  /* Vortex-lattice config. */
  VORTEX_COUNT = 6,
};

/* Physical / visual constants — units noted on each. */

/* How far a particle moves each tick (cells), plus a per-particle wobble. */
#define TRACER_STEP_BASE_CPT 0.85f
#define TRACER_STEP_JITTER_CPT 0.40f

/* How many ticks a particle lives before it dies and respawns, plus wobble. */
#define TRACER_LIFE_BASE_TICKS 120
#define TRACER_LIFE_JITTER_TICKS 80

/* How fast the wind pattern morphs over time (advance per tick). */
#define FIELD_EVOLUTION_DEFAULT 0.006f
#define FIELD_EVOLUTION_MIN 0.001f
#define FIELD_EVOLUTION_MAX 0.080f
#define FIELD_EVOLUTION_FACTOR 1.5f /* how much f/F speeds up / slows down */

/* Curl-noise tuning: how zoomed-in the noise is, and how far apart the
 * four probe samples sit when we measure the slope. */
#define CURL_NOISE_SCALE_X 0.030f
#define CURL_NOISE_SCALE_Y 0.055f
#define CURL_DIFFERENCE_EPSILON 1.20f

/* Where the ring of vortices sits and how it spins. */
#define VORTEX_RING_RADIUS_FRAC 0.28f  /* ring radius, as a fraction of min(W,H)/2 */
#define VORTEX_RING_ORBIT_SPEED 0.014f /* ring spin, radians per tick */
#define VORTEX_STRENGTH_GAMMA 3.0f     /* how hard each vortex swirls */
#define VORTEX_SOFTEN_PIXELS 5.0f      /* fudge that keeps speed finite at a vortex centre */

/* How tight the crossed-wave pattern is. */
#define SINE_FREQ_X 0.055f
#define SINE_FREQ_Y 0.095f

/* How strongly the galaxy spiral breathes in and out. */
#define SPIRAL_RADIAL_WEIGHT 0.65f

/* Terminal cells are about twice as tall as wide; squash y by this so
 * circles look round instead of like tall eggs. */
#define ASPECT_FACTOR 0.5f

/* Time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* Colour pair IDs.  The 16 palette pairs occupy 1..16; HUD pairs follow. */
enum {
  PAIR_PALETTE_BASE = 1,
  PAIR_HUD = PAIR_PALETTE_BASE + PALETTE_PAIR_COUNT,
  PAIR_HINT,
};

/* Which wind generator is live; names below show up in the HUD. */
enum field_kind {
  FIELD_KIND_CURL = 0,
  FIELD_KIND_VORTEX,
  FIELD_KIND_SINE,
  FIELD_KIND_SPIRAL,
};

static const char *field_kind_name_table[FIELD_KIND_COUNT] = {
    "curl-noise", "vortex-lattice", "sine-lattice", "radial-spiral"};

/* Background mode names. */
static const char *bg_mode_name_table[BG_MODE_COUNT] = {"blank", "arrows",
                                                        "colormap"};

/* §2  clock — a steady nanosecond timer plus a sleep */

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

/* §3  rng — build Perlin noise's one shuffled lookup table
 *
 * All the randomness in Perlin noise comes from this shuffled 256-byte
 * table.  Built once at startup, then read-only forever.
 */

static uint8_t perlin_perm_table[512];

static void perlin_perm_init(void) {
  uint8_t identity[256];
  for (int i = 0; i < 256; i++)
    identity[i] = (uint8_t)i;
  for (int i = 255; i > 0; i--) {
    int j = rand() % (i + 1);
    uint8_t tmp = identity[i];
    identity[i] = identity[j];
    identity[j] = tmp;
  }
  /* Store it twice back-to-back so lookups can skip the wrap-around math. */
  for (int i = 0; i < 512; i++)
    perlin_perm_table[i] = identity[i & 255];
}

/* §4  perlin — smooth random noise: one number in [-1, +1] at any (x, y)
 *
 * Same noise routine as fluid/flowfield.c §4; see that file if it's new
 * to you.
 */

static inline float smoothstep_cubic(float t) {
  return t * t * (3.0f - 2.0f * t);
}

static inline float lerp_scalar(float a, float b, float t) {
  return a + t * (b - a);
}

static inline float perlin_gradient_dot(int hash, float x, float y) {
  int bits = hash & 3;
  float term1 = (bits < 2) ? x : y;
  float term2 = (bits < 2) ? y : x;
  return ((hash & 1) ? -term1 : term1) + ((hash & 2) ? -term2 : term2);
}

static float perlin_value(float x, float y) {
  int xi = (int)floorf(x) & 255;
  int yi = (int)floorf(y) & 255;
  float fx = x - floorf(x);
  float fy = y - floorf(y);
  float ux = smoothstep_cubic(fx);
  float uy = smoothstep_cubic(fy);

  int h00 = perlin_perm_table[perlin_perm_table[xi] + yi];
  int h10 = perlin_perm_table[perlin_perm_table[xi + 1] + yi];
  int h01 = perlin_perm_table[perlin_perm_table[xi] + yi + 1];
  int h11 = perlin_perm_table[perlin_perm_table[xi + 1] + yi + 1];

  float d00 = perlin_gradient_dot(h00, fx, fy);
  float d10 = perlin_gradient_dot(h10, fx - 1.0f, fy);
  float d01 = perlin_gradient_dot(h01, fx, fy - 1.0f);
  float d11 = perlin_gradient_dot(h11, fx - 1.0f, fy - 1.0f);

  float top = lerp_scalar(d00, d10, ux);
  float bottom = lerp_scalar(d01, d11, ux);
  return lerp_scalar(top, bottom, uy);
}

/* §5  fbm — layered noise: stack several noise copies for natural detail
 *
 * Add a few noise layers, each finer and fainter than the last, so the
 * result has both broad shapes and small wrinkles.  See fluid/flowfield.c
 * for why.
 */

static float fbm_value(float x, float y, float t, int octaves) {
  float result = 0.0f;
  float amplitude = 1.0f;
  float frequency = 1.0f;
  for (int oct = 0; oct < octaves; oct++) {
    result +=
        perlin_value(x * frequency + t, y * frequency + t * 0.7f) * amplitude;
    amplitude *= 0.5f;
    frequency *= 2.0f;
  }
  return result;
}

/* §6  cosine_palette — Inigo Quilez's colour-gradient trick, six themes */

/*
 * CosinePaletteTheme — one colour theme.
 *
 * Inigo Quilez's palette trick (iquilezles.org/articles/palettes) builds a
 * smooth colour gradient from just four numbers per channel.  Feed it one
 * value t and it hands back an (R, G, B), with each channel computed as:
 *
 *       channel = bias + amplitude * cos( 2*pi * (frequency*t + phase) )
 *
 * So one theme is four little three-number vectors (one number per R, G, B)
 * plus a name.  Swapping in different vectors gives wildly different colour
 * journeys (cosmic, ember, ocean, ...) from the exact same formula.  The
 * four arrays are kept side by side so the table reads just like the
 * formula above.
 *
 *   bias       baseline brightness of each channel
 *   amplitude  how far the channel swings up and down
 *   frequency  how many colour cycles you pass through as t goes 0..1
 *   phase      a per-channel head-start that sets the hue
 */
typedef struct {
    float       bias_rgb[3];
    float       amplitude_rgb[3];
    float       frequency_rgb[3];
    float       phase_rgb[3];
    const char *name;             /* short label shown in the HUD */
} CosinePaletteTheme;

/* Six themes; the comment on each is the look it aims for. */
static const CosinePaletteTheme palette_theme_table[THEME_COUNT] = {
    /* 0 cosmic — electric violet → cyan → hot magenta */
    {{0.50f, 0.50f, 0.50f},
     {0.50f, 0.50f, 0.50f},
     {1.00f, 1.00f, 0.50f},
     {0.80f, 0.90f, 0.30f},
     "cosmic"},

    /* 1 ember — deep red → orange → pale yellow */
    {{0.55f, 0.30f, 0.05f},
     {0.45f, 0.30f, 0.05f},
     {1.00f, 0.80f, 0.30f},
     {0.00f, 0.10f, 0.25f},
     "ember"},

    /* 2 ocean — navy → teal → ice */
    {{0.15f, 0.40f, 0.60f},
     {0.20f, 0.35f, 0.40f},
     {0.50f, 0.70f, 1.00f},
     {0.00f, 0.10f, 0.30f},
     "ocean"},

    /* 3 neon — electric green → hot pink → violet */
    {{0.50f, 0.50f, 0.50f},
     {0.50f, 0.50f, 0.50f},
     {1.00f, 0.50f, 1.00f},
     {0.00f, 0.50f, 0.33f},
     "neon"},

    /* 4 sunset — purple → crimson → amber */
    {{0.50f, 0.38f, 0.30f},
     {0.50f, 0.38f, 0.30f},
     {1.00f, 0.85f, 0.60f},
     {0.00f, 0.18f, 0.40f},
     "sunset"},

    /* 5 mono — silver-blue clean grayscale */
    {{0.45f, 0.48f, 0.55f},
     {0.40f, 0.42f, 0.45f},
     {0.50f, 0.50f, 0.50f},
     {0.00f, 0.02f, 0.05f},
     "mono"},
};

/* Run the palette formula for one theme at position t (0..1), returning an
 * (R, G, B) with each channel pinned to 0..1. */
static void cosine_palette_eval(const CosinePaletteTheme *theme,
                                float palette_param, float *out_r, float *out_g,
                                float *out_b) {
  float r =
      theme->bias_rgb[0] +
      theme->amplitude_rgb[0] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[0] * palette_param + theme->phase_rgb[0]));
  float g =
      theme->bias_rgb[1] +
      theme->amplitude_rgb[1] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[1] * palette_param + theme->phase_rgb[1]));
  float b =
      theme->bias_rgb[2] +
      theme->amplitude_rgb[2] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[2] * palette_param + theme->phase_rgb[2]));
  if (r < 0.0f)
    r = 0.0f;
  else if (r > 1.0f)
    r = 1.0f;
  if (g < 0.0f)
    g = 0.0f;
  else if (g > 1.0f)
    g = 1.0f;
  if (b < 0.0f)
    b = 0.0f;
  else if (b > 1.0f)
    b = 1.0f;
  *out_r = r;
  *out_g = g;
  *out_b = b;
}

/* Snap a 0..1 (R, G, B) to the nearest of the 216 colours in the terminal's
 * 6x6x6 colour cube (palette indices 16..231). */
static int rgb_to_xterm256_cube(float r, float g, float b) {
  int r5 = (int)(r * 5.0f + 0.5f);
  int g5 = (int)(g * 5.0f + 0.5f);
  int b5 = (int)(b * 5.0f + 0.5f);
  if (r5 > 5)
    r5 = 5;
  if (r5 < 0)
    r5 = 0;
  if (g5 > 5)
    g5 = 5;
  if (g5 < 0)
    g5 = 0;
  if (b5 > 5)
    b5 = 5;
  if (b5 < 0)
    b5 = 0;
  return 16 + 36 * r5 + 6 * g5 + b5;
}

/* Plan B for ancient 8-colour terminals: one fixed colour list per theme. */
static const int palette_fallback_8[THEME_COUNT][PALETTE_PAIR_COUNT] = {
    {COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_CYAN, COLOR_BLUE, COLOR_WHITE, COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE,
     COLOR_WHITE, COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE},
    {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_RED, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
     COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_BLUE, COLOR_CYAN,
     COLOR_CYAN, COLOR_WHITE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
     COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
    {COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE, COLOR_GREEN,
     COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA,
     COLOR_BLUE, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE,
     COLOR_WHITE},
    {COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW,
     COLOR_WHITE},
    {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE},
};

/* §7  colors — load a theme into ncurses, and turn an angle into a colour */

static void colors_apply_theme(int theme_index) {
  const CosinePaletteTheme *theme = &palette_theme_table[theme_index];
  bool has_256 = (COLORS >= 256);

  for (int i = 0; i < PALETTE_PAIR_COUNT; i++) {
    float palette_param = (float)i / (float)(PALETTE_PAIR_COUNT - 1);
    if (has_256) {
      float r, g, b;
      cosine_palette_eval(theme, palette_param, &r, &g, &b);
      int fg = rgb_to_xterm256_cube(r, g, b);
      init_pair(PAIR_PALETTE_BASE + i, fg, -1);
    } else {
      init_pair(PAIR_PALETTE_BASE + i, palette_fallback_8[theme_index][i], -1);
    }
  }

  /* HUD colours: bright yellow + bright cyan so they stay readable. */
  if (has_256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  colors_apply_theme(theme_index);
}

/* Pick a colour for a flow direction: wrap the angle onto the palette so
 * nearby directions get nearby colours and opposite directions get
 * opposite colours. */
static int angle_to_palette_pair(float angle_radians) {
  float a = angle_radians;
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  int idx =
      (int)(a / (2.0f * (float)M_PI) * PALETTE_PAIR_COUNT) % PALETTE_PAIR_COUNT;
  return PAIR_PALETTE_BASE + idx;
}

/* §8  field — the angle grid, smooth lookups, and the generator switch */

#define FIELD_COLS_MAX 256
#define FIELD_ROWS_MAX 80

/*
 * flow_field — the "wind map" every particle rides.
 *
 * Whichever generator is live (curl / vortex / sine / spiral), it produces
 * the same thing: a flow direction (an angle) at every grid cell, stored in
 * angle[][].  Particles never call a generator directly — they just look up
 * the angle near them and step that way.  That's the whole point of the
 * file: the wind is a swappable black box, and the particle code never
 * changes when you swap it.
 *
 * We fill in the whole grid once per tick rather than asking the generator
 * fresh for every particle.  With far more particles than cells that's much
 * cheaper, and reading between grid cells (bilinear lookup) gives smooth
 * motion for free.
 *
 * The vortex bookkeeping lives here too (not at file scope) so the vortices'
 * positions and spin survive resizes and theme switches.  Only the vortex
 * generator looks at it.
 */
typedef struct {
    /* Size of the part of the grid we're actually using (clamped on resize). */
    int active_cols;
    int active_rows;

    /* Which generator is live (a FIELD_KIND_* value); the 'a' key cycles it. */
    int active_kind;

    /* The wind's own clock.  time_axis creeps forward by evolution_speed each
     * tick and is the "t" the generators morph with.  Kept separate from
     * real frame time so f/F can speed up the morphing without changing how
     * particles move. */
    float time_axis;
    float evolution_speed;

    /* The flow direction (radians) at every cell, refilled each tick. */
    float angle[FIELD_ROWS_MAX][FIELD_COLS_MAX];

    /* Vortex bookkeeping (only the vortex generator uses this): where each
     * vortex sits, how hard and which way it spins, and the ring's rotation. */
    float vortex_pos_col      [VORTEX_COUNT];
    float vortex_pos_row      [VORTEX_COUNT];
    float vortex_strength_table[VORTEX_COUNT];
    float vortex_ring_phase;
} flow_field;

/* The four generators, defined in §9-§12. */
static float field_curl_angle(const flow_field *f, float x, float y, float t);
static float field_vortex_angle(const flow_field *f, float x, float y);
static float field_sine_angle(const flow_field *f, float x, float y, float t);
static float field_spiral_angle(const flow_field *f, float x, float y, float t);

/* Lay the vortices out evenly on a ring around screen centre, advanced by
 * the ring's current rotation. */
static void field_update_vortex_positions(flow_field *f) {
  float centre_col = (float)f->active_cols * 0.5f;
  float centre_row = (float)f->active_rows * 0.5f;
  int smaller =
      (f->active_cols < f->active_rows) ? f->active_cols : f->active_rows;
  float radius = VORTEX_RING_RADIUS_FRAC * (float)smaller * 0.5f;

  for (int i = 0; i < VORTEX_COUNT; i++) {
    float angle = f->vortex_ring_phase +
                  (float)i * (2.0f * (float)M_PI / (float)VORTEX_COUNT);
    f->vortex_pos_col[i] = centre_col + radius * cosf(angle);
    /* squash y so the ring looks round, not stretched tall */
    f->vortex_pos_row[i] = centre_row + radius * sinf(angle) * ASPECT_FACTOR;
  }
}

static void field_init(flow_field *f, int cols, int rows, int kind) {
  if (cols > FIELD_COLS_MAX)
    cols = FIELD_COLS_MAX;
  if (rows > FIELD_ROWS_MAX)
    rows = FIELD_ROWS_MAX;
  f->active_cols = cols;
  f->active_rows = rows;
  f->active_kind = kind;
  f->time_axis = 0.0f;
  f->evolution_speed = FIELD_EVOLUTION_DEFAULT;
  f->vortex_ring_phase = 0.0f;
  memset(f->angle, 0, sizeof f->angle);

  /* Make neighbouring vortices spin opposite ways (one clockwise, the next
   * counter-clockwise) so flow threads neatly between them. */
  for (int i = 0; i < VORTEX_COUNT; i++)
    f->vortex_strength_table[i] =
        (i & 1) ? -VORTEX_STRENGTH_GAMMA : VORTEX_STRENGTH_GAMMA;

  field_update_vortex_positions(f);
}

/* Tick the wind forward: nudge its clock and the vortex ring, then refill
 * every cell's flow angle using whichever generator is live. */
static void field_evolve_and_rebuild(flow_field *f) {
  f->time_axis += f->evolution_speed;
  f->vortex_ring_phase += VORTEX_RING_ORBIT_SPEED;
  field_update_vortex_positions(f);

  for (int r = 0; r < f->active_rows; r++) {
    for (int c = 0; c < f->active_cols; c++) {
      float angle;
      switch (f->active_kind) {
      case FIELD_KIND_CURL:
        angle = field_curl_angle(f, (float)c, (float)r, f->time_axis);
        break;
      case FIELD_KIND_VORTEX:
        angle = field_vortex_angle(f, (float)c, (float)r);
        break;
      case FIELD_KIND_SINE:
        angle = field_sine_angle(f, (float)c, (float)r, f->time_axis);
        break;
      case FIELD_KIND_SPIRAL:
      default:
        angle = field_spiral_angle(f, (float)c, (float)r, f->time_axis);
        break;
      }
      f->angle[r][c] = angle;
    }
  }
}

/* The flow angle stored at a whole-number cell, with out-of-range
 * coordinates pulled back to the edge. */
static inline float flow_angle_at_cell(const flow_field *f, int c, int r) {
  if (c < 0)
    c = 0;
  if (c >= f->active_cols)
    c = f->active_cols - 1;
  if (r < 0)
    r = 0;
  if (r >= f->active_rows)
    r = f->active_rows - 1;
  return f->angle[r][c];
}

/* Keep a cell index in range; the smooth lookup peeks one cell past the
 * edge, so it needs this. */
static inline int clamp_to_active(int idx, int max_count) {
    if (idx < 0)              return 0;
    if (idx >= max_count)     return max_count - 1;
    return idx;
}

/* How far into its cell a fractional position sits (0..1 on each axis);
 * these become the blend weights for the smooth lookup. */
static inline void compute_subcell_offsets(float col, float row,
                                           float *out_fx, float *out_fy) {
    *out_fx = col - floorf(col);
    *out_fy = row - floorf(row);
}

/* Blend the four corner values of a cell to get the value at a point inside
 * it: mix left-to-right on each row, then mix those two top-to-bottom. */
static inline float bilinear_lerp_4corners(float tl, float tr,
                                            float bl, float br,
                                            float fx, float fy) {
    float top    = lerp_scalar(tl, tr, fx);
    float bottom = lerp_scalar(bl, br, fx);
    return lerp_scalar(top, bottom, fy);
}

/* The flow angle at a fractional position, blended smoothly from the four
 * surrounding cells so motion doesn't jump from cell to cell. */
static float flow_angle_bilinear(const flow_field *f, float col, float row) {
    int   c0 = clamp_to_active((int)floorf(col),     f->active_cols);
    int   c1 = clamp_to_active((int)floorf(col) + 1, f->active_cols);
    int   r0 = clamp_to_active((int)floorf(row),     f->active_rows);
    int   r1 = clamp_to_active((int)floorf(row) + 1, f->active_rows);

    float fx, fy;
    compute_subcell_offsets(col, row, &fx, &fy);

    return bilinear_lerp_4corners(f->angle[r0][c0], f->angle[r0][c1],
                                   f->angle[r1][c0], f->angle[r1][c1],
                                   fx, fy);
}

/* §9  field_curl — swirly, fluid-like wind that never lets particles pile up
 *
 * Trick (Bridson et al., "Curl-Noise for Procedural Fluid Flow", 2007):
 * take a smooth noise hill, then make the wind flow *along* its contour
 * lines instead of up or down them.  Probe the noise just above, below,
 * left, and right of the point, and the wind is the sideways slope.  Flow
 * that follows contours can't drain into a point or gush out of one, so
 * particles spread out like real fluid instead of clumping.
 */

static float field_curl_angle(const flow_field *f, float x, float y, float t) {
  (void)f; /* curl reads only the noise, not the field's own state */
  float eps = CURL_DIFFERENCE_EPSILON;

  float psi_north =
      fbm_value(x * CURL_NOISE_SCALE_X, y * CURL_NOISE_SCALE_Y + eps, t,
                CURL_FBM_OCTAVES);
  float psi_south =
      fbm_value(x * CURL_NOISE_SCALE_X, y * CURL_NOISE_SCALE_Y - eps, t,
                CURL_FBM_OCTAVES);
  float psi_east = fbm_value(x * CURL_NOISE_SCALE_X + eps,
                             y * CURL_NOISE_SCALE_Y, t, CURL_FBM_OCTAVES);
  float psi_west = fbm_value(x * CURL_NOISE_SCALE_X - eps,
                             y * CURL_NOISE_SCALE_Y, t, CURL_FBM_OCTAVES);

  /* wind = sideways slope of the noise (the bit that follows contours) */
  float vx = (psi_north - psi_south);
  float vy = -(psi_east - psi_west);
  return atan2f(vy, vx);
}

/* §10  field_vortex — wind that spins around a ring of little whirlpools
 *
 * Each vortex makes the air circle around it, harder up close and gently
 * far away (the same 1/distance falloff as a magnet's field, the
 * Biot-Savart law).  Add up every vortex's pull and you get the wind here.
 * The little softening fudge in the denominator stops the speed from
 * blowing up to infinity right at a vortex centre.
 */

static float field_vortex_angle(const flow_field *f, float x, float y) {
  float vx_total = 0.0f;
  float vy_total = 0.0f;

  for (int i = 0; i < VORTEX_COUNT; i++) {
    float dx = x - f->vortex_pos_col[i];
    float dy = (y - f->vortex_pos_row[i]) / ASPECT_FACTOR; /* un-squash y */
    float r2 = dx * dx + dy * dy + VORTEX_SOFTEN_PIXELS;
    float strength = f->vortex_strength_table[i];
    vx_total += strength * (-dy) / r2;
    vy_total += strength * (dx) / r2;
  }
  return atan2f(vy_total, vx_total);
}

/* §11  field_sine — wind built from crossed waves, like ripples overlapping
 *
 * No physics here, just sine waves added together.  Where the waves line up
 * or cancel you get a regular plaid / moire crosshatch.  The slightly
 * different time speeds keep the pattern drifting instead of standing still.
 */

static float field_sine_angle(const flow_field *f, float x, float y, float t) {
  (void)f;
  float vx = sinf(x * SINE_FREQ_X + t) + sinf(y * SINE_FREQ_Y - t * 0.7f);
  float vy =
      cosf(x * SINE_FREQ_X - t * 0.5f) + cosf(y * SINE_FREQ_Y + t * 0.3f);
  return atan2f(vy, vx);
}

/* §12  field_spiral — a galaxy: wind that circles the centre and breathes
 *
 * Base wind circles the screen centre counter-clockwise.  On top of that a
 * slow in-and-out pulse pushes particles outward, then back inward, over
 * and over, so the whole thing looks like a breathing galaxy.
 */

static float field_spiral_angle(const flow_field *f, float x, float y,
                                float t) {
  float centre_col = (float)f->active_cols * 0.5f;
  float centre_row = (float)f->active_rows * 0.5f;
  float dx = x - centre_col;
  float dy = (y - centre_row) / ASPECT_FACTOR; /* un-squash y so it stays round */
  float radius = sqrtf(dx * dx + dy * dy);
  if (radius < 1e-4f)
    return 0.0f; /* dead centre has no direction; skip it */

  float theta = atan2f(dy, dx);
  float pulse = SPIRAL_RADIAL_WEIGHT * sinf(t * 0.8f); /* the breathing */

  float vx = -sinf(theta) + pulse * cosf(theta);
  float vy = cosf(theta) + pulse * sinf(theta);
  return atan2f(vy, vx);
}

/* §13  arrows — turn a direction into one of 8 little arrow characters */

#define ARROW_OCTANT_COUNT 8

static const char arrow_glyph_table[ARROW_OCTANT_COUNT] = {
    '>',  /* 0  E   */
    '/',  /* 1  NE  */
    '^',  /* 2  N   */
    '\\', /* 3  NW  */
    '<',  /* 4  W   */
    '/',  /* 5  SW  */
    'v',  /* 6  S   */
    '\\', /* 7  SE  */
};

static int angle_to_octant(float angle_radians) {
  float a = angle_radians;
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  return (int)(a / (2.0f * (float)M_PI) * ARROW_OCTANT_COUNT + 0.5f) %
         ARROW_OCTANT_COUNT;
}

static inline char arrow_glyph_for_angle(float angle_radians) {
  return arrow_glyph_table[angle_to_octant(angle_radians)];
}

/* §14  tracer — one particle, with the fading trail it leaves behind */

/*
 * tracer — one particle drifting in the wind, plus its trail.
 *
 * Each tick a particle reads the wind under it, takes a step that way, and
 * drops its old spot into a short trail.  The paint pass draws that trail as
 * a fading streak (newest spot brightest, oldest faintest).
 *
 * The trail is a fixed-size ring buffer: a small array we keep overwriting
 * in a loop, so a trail never needs new memory while the program runs.
 * trail_write_index is where the next spot goes; trail_filled_count counts
 * up until the buffer is full and then stops.
 *
 * When a particle's time runs out it dies and respawns somewhere new.  Each
 * one gets a slightly different lifetime so they don't all blink out on the
 * same frame.  Its colour is locked in at birth (not re-picked every tick),
 * which gives the nice "ribbons of one colour" look instead of flickering.
 */
typedef struct {
    /* Life cycle. */
    bool  tracer_alive;
    int   ticks_until_respawn;     /* counts down; at 0 the particle dies */

    /* Where it is and how it's moving. */
    float pos_col;
    float pos_row;
    float step_cells;              /* how far it moves per tick */
    float last_angle;              /* the direction it just went (sets the head glyph) */

    /* The trail ring buffer. */
    int   trail_pair_id;           /* colour, locked in at birth */
    int   trail_col[TRAIL_LEN_HARD_MAX];
    int   trail_row[TRAIL_LEN_HARD_MAX];
    int   trail_write_index;       /* next slot to overwrite */
    int   trail_filled_count;      /* how many slots hold real data so far */
    int   trail_active_length;     /* how long the trail currently is */
} tracer;

/* A random spot somewhere on the grid for a new particle. */
static inline void pick_random_spawn_position(int active_cols, int active_rows,
                                              float *out_col, float *out_row) {
    *out_col = (float)(rand() % active_cols);
    *out_row = (float)(rand() % active_rows);
}

/* A slightly random step speed, so neighbouring particles don't all move in
 * lockstep and the flow looks textured. */
static inline float pick_jittered_step_speed_cpt(void) {
    return TRACER_STEP_BASE_CPT
         - TRACER_STEP_JITTER_CPT * 0.5f
         + TRACER_STEP_JITTER_CPT * ((float)rand() / (float)RAND_MAX);
}

/* A slightly random lifetime, so particles don't all die on the same frame. */
static inline int pick_random_lifetime_ticks(void) {
    return TRACER_LIFE_BASE_TICKS + rand() % TRACER_LIFE_JITTER_TICKS;
}

/* A random colour the particle keeps for its whole life (see tracer doc). */
static inline int pick_random_palette_pair(void) {
    return PAIR_PALETTE_BASE + rand() % PALETTE_PAIR_COUNT;
}

/* Clear the trail and seed every slot with the spawn position, so no leftover
 * dots from a past life flash on screen. */
static inline void reset_trail_ring_at(tracer *t,
                                       int initial_col, int initial_row,
                                       int trail_active_length) {
    t->trail_write_index   = 0;
    t->trail_filled_count  = 0;
    t->trail_active_length = trail_active_length;
    for (int i = 0; i < TRAIL_LEN_HARD_MAX; i++) {
        t->trail_col[i] = initial_col;
        t->trail_row[i] = initial_row;
    }
}

/* Give a dead particle a fresh life: new spot, speed, lifetime, and colour. */
static void tracer_respawn(tracer *t, int active_cols, int active_rows,
                           int trail_active_length) {
    t->tracer_alive        = true;
    pick_random_spawn_position(active_cols, active_rows,
                               &t->pos_col, &t->pos_row);
    t->step_cells          = pick_jittered_step_speed_cpt();
    t->last_angle          = 0.0f;
    t->ticks_until_respawn = pick_random_lifetime_ticks();
    t->trail_pair_id       = pick_random_palette_pair();
    reset_trail_ring_at(t, (int)t->pos_col, (int)t->pos_row,
                        trail_active_length);
}

/* §15  tracer_step — move one particle one step with the wind */

/* Drop the particle's current spot into its trail. */
static inline void push_position_into_trail(tracer *t) {
    t->trail_col[t->trail_write_index] = (int)t->pos_col;
    t->trail_row[t->trail_write_index] = (int)t->pos_row;
    t->trail_write_index = (t->trail_write_index + 1) % t->trail_active_length;
    if (t->trail_filled_count < t->trail_active_length)
        t->trail_filled_count++;
}

/* Read the wind direction under the particle and remember it (the paint pass
 * reuses it for the head arrow). */
static inline float sample_field_at_tracer(tracer *t, const flow_field *f) {
    float angle = flow_angle_bilinear(f, t->pos_col, t->pos_row);
    t->last_angle = angle;
    return angle;
}

/* Nudge the particle one step in the wind's direction. */
static inline void step_tracer_along_angle(tracer *t, float angle) {
    t->pos_col += cosf(angle) * t->step_cells;
    t->pos_row += sinf(angle) * t->step_cells;
}

/* Wrap a particle around the edges: walk off the right and you reappear on
 * the left, etc., so nobody just drifts off and vanishes. */
static inline void wrap_position_toroidally(tracer *t,
                                            int active_cols, int active_rows) {
    if (t->pos_col <  0.0f)              t->pos_col += (float)active_cols;
    if (t->pos_col >= (float)active_cols) t->pos_col -= (float)active_cols;
    if (t->pos_row <  0.0f)              t->pos_row += (float)active_rows;
    if (t->pos_row >= (float)active_rows) t->pos_row -= (float)active_rows;
}

/* Re-tint the trail from the direction it's heading, so particles flowing
 * opposite ways get opposite colours. */
static inline void recolour_tracer_from_angle(tracer *t, float angle) {
    t->trail_pair_id = angle_to_palette_pair(angle);
}

/* Count down the particle's life and kill it when it hits zero. */
static inline void age_tracer_and_check_death(tracer *t) {
    t->ticks_until_respawn--;
    if (t->ticks_until_respawn <= 0)
        t->tracer_alive = false;
}

/* One full tick for one particle: record where it is, read the wind, step,
 * wrap, recolour, and age it. */
static void tracer_advance_one_tick(tracer *t, const flow_field *f,
                                    int active_cols, int active_rows) {
    if (!t->tracer_alive)
        return;

    push_position_into_trail(t);
    float angle = sample_field_at_tracer(t, f);
    step_tracer_along_angle(t, angle);
    wrap_position_toroidally(t, active_cols, active_rows);
    recolour_tracer_from_angle(t, angle);
    age_tracer_and_check_death(t);
}

/* §16  tracer_paint — draw a particle's trail as a fading streak
 *
 * Walk the trail oldest-to-newest: the head gets a bold direction mark, the
 * body uses a dot-to-hash ramp, and the oldest bit dims out.  (A_DIM is
 * banned for the HUD but fine here — it's the fade effect we want.)
 */

#define TRAIL_RAMP_STR ".,;+~*#"
#define TRAIL_RAMP_LEN 7

static char trail_head_glyph_for_angle(float angle_radians) {
  static const char head_glyph_table[ARROW_OCTANT_COUNT] = {
      '-', '/', '|', '\\', '-', '/', '|', '\\'};
  return head_glyph_table[angle_to_octant(angle_radians)];
}

/* Turn "the i-th oldest dot" into the real array slot it lives in.  i=0 is
 * the oldest, i=filled-1 the newest.  The +4*len just keeps the wrap-around
 * arithmetic from going negative. */
static inline int trail_slot_index(const tracer *t, int i, int filled, int len) {
    return (t->trail_write_index - filled + i + 4 * len) % len;
}

/* Is this spot actually on screen? */
static inline bool position_in_field(int col, int row,
                                     int active_cols, int active_rows) {
    return col >= 0 && col < active_cols && row >= 0 && row < active_rows;
}

/* Which character to draw for trail dot i: an arrow at the head, otherwise a
 * dot-to-hash character chosen by how old the dot is (oldest = '.'). */
static inline char pick_trail_glyph(const tracer *t, int i, int filled,
                                    bool is_head) {
    if (is_head)
        return trail_head_glyph_for_angle(t->last_angle);
    int ramp_index = (i * TRAIL_RAMP_LEN) / (filled > 1 ? filled : 1);
    if (ramp_index >= TRAIL_RAMP_LEN)
        ramp_index = TRAIL_RAMP_LEN - 1;
    return TRAIL_RAMP_STR[ramp_index];
}

/* How bright to draw trail dot i: bold head, normal middle, dim tail. */
static inline attr_t pick_trail_brightness(int i, int filled, bool is_head) {
    if (is_head)                  return A_BOLD;
    if (i >= filled / 4)          return A_NORMAL;
    return A_DIM;
}

/* Draw one trail character on screen. */
static inline void paint_trail_cell(WINDOW *win, int row, int col,
                                    char glyph, int pair_id, attr_t bright) {
    attr_t a = COLOR_PAIR(pair_id) | bright;
    wattron(win, a);
    mvwaddch(win, row, col, (chtype)(unsigned char)glyph);
    wattroff(win, a);
}

/* Draw one particle's whole fading trail. */
static void tracer_paint(const tracer *t, WINDOW *win, int active_cols,
                         int active_rows) {
    if (!t->tracer_alive)               return;
    if (t->trail_filled_count == 0)     return;

    int filled = t->trail_filled_count;
    int len    = t->trail_active_length;

    for (int i = 0; i < filled; i++) {
        int slot = trail_slot_index(t, i, filled, len);
        int col  = t->trail_col[slot];
        int row  = t->trail_row[slot];

        if (!position_in_field(col, row, active_cols, active_rows))
            continue;

        bool   is_head    = (i == filled - 1);
        char   glyph      = pick_trail_glyph(t, i, filled, is_head);
        attr_t brightness = pick_trail_brightness(i, filled, is_head);

        paint_trail_cell(win, row, col, glyph, t->trail_pair_id, brightness);
    }
}

/* §17  scene — holds everything live and runs one tick of the whole world */

enum bg_mode {
  BG_BLANK = 0,
  BG_ARROWS,
  BG_COLORMAP,
};

/*
 * scene_state — the one struct that holds all of the demo's live state.
 *
 * It bundles the wind, the particles riding it, and the viewer's choices
 * (which colour theme, which background).  Everything that never changes
 * (the palette tables, arrow glyphs, key bindings) lives elsewhere as
 * constants; only the stuff that moves or that the user can poke lives here.
 *
 * Fields are grouped by who touches them so it's clear at a glance whether
 * something is simulation or just looks.  Keeping the two apart matters: if
 * a purely-visual choice like the theme leaked into the simulation, swapping
 * colours would change how particles move, which would defeat the whole
 * "wind and looks are independent" idea.
 */
typedef struct {
    /* Simulation: the wind, and the particles moving through it. */
    flow_field flow;
    tracer     pool[TRACERS_MAX];

    /* Used by both sim and drawing: how many particles are live, and how
     * long their trails are.  Tuned with +/- and s/S. */
    int active_tracer_count;
    int trail_active_length;

    /* Looks only — changing these must not touch the simulation. */
    int theme_index;        /* which colour theme */
    int bg_mode;            /* blank / arrows / colormap background */

    /* Control: pauses the sim and shows "PAUSED" in the HUD. */
    bool paused;
} scene_state;

static void scene_init(scene_state *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->active_tracer_count = TRACERS_DEFAULT;
  s->trail_active_length = TRAIL_LEN_DEFAULT;
  s->theme_index = 0;
  s->bg_mode = BG_BLANK;
  s->paused = false;

  field_init(&s->flow, cols, rows, FIELD_KIND_CURL);
  field_evolve_and_rebuild(&s->flow);

  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_resize(scene_state *s, int cols, int rows) {
  field_init(&s->flow, cols, rows, s->flow.active_kind);
  field_evolve_and_rebuild(&s->flow);
  for (int i = 0; i < TRACERS_MAX; i++)
    if (s->pool[i].tracer_alive)
      tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_reset(scene_state *s, int cols, int rows) {
  s->flow.time_axis = 0.0f;
  s->flow.vortex_ring_phase = 0.0f;
  field_update_vortex_positions(&s->flow);
  field_evolve_and_rebuild(&s->flow);
  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
  for (int i = s->active_tracer_count; i < TRACERS_MAX; i++)
    s->pool[i].tracer_alive = false;
}

static void scene_set_trail_length(scene_state *s, int new_length) {
  if (new_length < TRAIL_LEN_MIN)
    new_length = TRAIL_LEN_MIN;
  if (new_length > TRAIL_LEN_MAX)
    new_length = TRAIL_LEN_MAX;
  s->trail_active_length = new_length;
  for (int i = 0; i < TRACERS_MAX; i++)
    s->pool[i].trail_active_length = new_length;
}

static void scene_tick(scene_state *s, int cols, int rows) {
  if (s->paused)
    return;

  field_evolve_and_rebuild(&s->flow);

  for (int i = 0; i < s->active_tracer_count; i++) {
    if (!s->pool[i].tracer_alive)
      tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
    tracer_advance_one_tick(&s->pool[i], &s->flow, cols, rows);
  }
}

/* Optionally show the wind itself behind the particles: nothing, a field of
 * little arrows, or a denser coloured version of the same. */
static void scene_paint_background(const scene_state *s, WINDOW *win, int cols,
                                   int rows) {
  if (s->bg_mode == BG_BLANK)
    return;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float angle = flow_angle_at_cell(&s->flow, c, r);
      int pair = angle_to_palette_pair(angle);
      char glyph = arrow_glyph_for_angle(angle);
      wattron(win, COLOR_PAIR(pair));
      mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
      wattroff(win, COLOR_PAIR(pair));
    }
  }
}

static void scene_paint(const scene_state *s, WINDOW *win, int cols, int rows) {
  scene_paint_background(s, win, cols, rows);
  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_paint(&s->pool[i], win, cols, rows);
}

/* §18  hud — status line up top, key hints along the bottom */

static void hud_paint_status(WINDOW *win, int cols, double fps_display,
                             int sim_hz, const scene_state *s) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%2dHz  tracers:%3d  trail:%2d  "
           "field:%-15s  bg:%-9s  theme:%-7s  evol:%.4f  %s ",
           fps_display, sim_hz, s->active_tracer_count, s->trail_active_length,
           field_kind_name_table[s->flow.active_kind],
           bg_mode_name_table[s->bg_mode],
           palette_theme_table[s->theme_index].name,
           (double)s->flow.evolution_speed, s->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = cols - len;
  if (x < 0)
    x = 0;
  wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwprintw(win, 0, x, "%s", buf);
  wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hints(WINDOW *win, int rows) {
  const char *hint = " q:quit  spc:pause  r:reset  a:field  t:theme  v:bg  "
                     "+/-:tracers  s/S:trail  ]/[:simHz  f/F:evol ";
  wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(win, rows - 1, 0, "%s", hint);
  wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §19  screen — start up and shut down ncurses */

static void screen_init(int theme_index) {
  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init(theme_index);
}

static void screen_cleanup(void) { endwin(); }

/* §20  app — the main loop, signal handling, and keyboard input */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int key, scene_state *s, int *sim_hz, int cols,
                           int rows) {
  switch (key) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 'r':
  case 'R':
    scene_reset(s, cols, rows);
    break;

  case 'a':
  case 'A':
    s->flow.active_kind = (s->flow.active_kind + 1) % FIELD_KIND_COUNT;
    scene_reset(s, cols, rows);
    break;

  case 't':
  case 'T':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    colors_apply_theme(s->theme_index);
    break;

  case 'v':
  case 'V':
    s->bg_mode = (s->bg_mode + 1) % BG_MODE_COUNT;
    break;

  case '+':
  case '=':
    if (s->active_tracer_count + TRACERS_STEP <= TRACERS_MAX) {
      int old_count = s->active_tracer_count;
      s->active_tracer_count += TRACERS_STEP;
      for (int i = old_count; i < s->active_tracer_count; i++)
        tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
    }
    break;
  case '-':
  case '_':
    if (s->active_tracer_count - TRACERS_STEP >= TRACERS_MIN) {
      s->active_tracer_count -= TRACERS_STEP;
      for (int i = s->active_tracer_count;
           i < s->active_tracer_count + TRACERS_STEP; i++)
        s->pool[i].tracer_alive = false;
    }
    break;

  case ']':
    if (*sim_hz + SIM_HZ_STEP <= SIM_HZ_MAX)
      *sim_hz += SIM_HZ_STEP;
    break;
  case '[':
    if (*sim_hz - SIM_HZ_STEP >= SIM_HZ_MIN)
      *sim_hz -= SIM_HZ_STEP;
    break;

  case 'f':
    s->flow.evolution_speed *= FIELD_EVOLUTION_FACTOR;
    if (s->flow.evolution_speed > FIELD_EVOLUTION_MAX)
      s->flow.evolution_speed = FIELD_EVOLUTION_MAX;
    break;
  case 'F':
    s->flow.evolution_speed /= FIELD_EVOLUTION_FACTOR;
    if (s->flow.evolution_speed < FIELD_EVOLUTION_MIN)
      s->flow.evolution_speed = FIELD_EVOLUTION_MIN;
    break;

  case 's':
    scene_set_trail_length(s, s->trail_active_length + 1);
    break;
  case 'S':
    scene_set_trail_length(s, s->trail_active_length - 1);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_now_ns());
  perlin_perm_init();

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  static scene_state scene;
  int sim_hz = SIM_HZ_DEFAULT;

  screen_init(0);
  atexit(screen_cleanup);

  int cols, rows;
  getmaxyx(stdscr, rows, cols);
  scene_init(&scene, cols, rows);

  /* Run the simulation at a steady rate no matter the frame rate, by saving
   * up leftover time and spending it in fixed-size steps. */
  int64_t prev_ns = clock_now_ns();
  int64_t sim_accum_ns = 0;

  /* Rolling FPS measurement. */
  int frames_in_window = 0;
  int64_t window_accum_ns = 0;
  double fps_display = 0.0;

  const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (!g_should_quit) {
    int64_t frame_start = clock_now_ns();

    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_resize(&scene, cols, rows);
      sim_accum_ns = 0;
    }

    int64_t dt_ns = frame_start - prev_ns;
    prev_ns = frame_start;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    int64_t tick_ns = NS_PER_SEC / sim_hz;
    sim_accum_ns += dt_ns;
    while (sim_accum_ns >= tick_ns) {
      scene_tick(&scene, cols, rows);
      sim_accum_ns -= tick_ns;
    }

    erase();
    scene_paint(&scene, stdscr, cols, rows);
    hud_paint_status(stdscr, cols, fps_display, sim_hz, &scene);
    hud_paint_hints(stdscr, rows);
    wnoutrefresh(stdscr);
    doupdate();

    frames_in_window++;
    window_accum_ns += dt_ns;
    if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      fps_display = (double)frames_in_window /
                    ((double)window_accum_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      window_accum_ns = 0;
    }

    int key;
    while ((key = getch()) != ERR) {
      if (!app_handle_key(key, &scene, &sim_hz, cols, rows)) {
        g_should_quit = 1;
        break;
      }
    }

    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
