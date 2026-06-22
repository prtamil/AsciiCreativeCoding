/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flowfield.c — animated 2-D flow in ASCII.
 *
 * Hundreds of tiny tracer particles drift across the screen as if
 * riding an invisible wind.  The "wind" is made-up smooth random
 * noise; each tracer reads the local direction and walks one step
 * that way every tick, leaving a fading trail.  Press 'a' to see the
 * wind arrows underneath.  This is NOT a real fluid solver — the wind
 * is a noise lookup, not physics.  For the real thing see
 * fluid/navier_stokes.c; for a more fluid-like noise variant see
 * procedural/fields/curl_noise_vector_field.c.
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

/* ── §1  config — every tunable knob in one place ── */

enum {
  /* Render frame cap. */
  TARGET_FPS_HZ = 60,

  /* Simulation step rate (independent of render rate). */
  SIM_HZ_MIN = 5,
  SIM_HZ_DEFAULT = 30,
  SIM_HZ_MAX = 60,
  SIM_HZ_STEP = 5,

  /* HUD recompute cadence. */
  FPS_RECOMPUTE_MS = 500,

  /* Tracer pool — fixed-size pre-allocated array. */
  TRACERS_MIN = 50,
  TRACERS_DEFAULT = 300,
  TRACERS_MAX = 800,
  TRACERS_STEP = 50,

  /* Trail buffer per tracer. */
  TRAIL_LEN_MIN = 3,
  TRAIL_LEN_DEFAULT = 14,
  TRAIL_LEN_MAX = 20,

  /* How many noise layers to stack — 3 looks organic without costing much. */
  FBM_OCTAVES = 3,

  /* How many distinct colours we cycle around the direction wheel. */
  HUE_WHEEL_PAIRS = 8,

  /* Number of themes (rainbow + 3 monos). */
  THEME_COUNT = 4,

  /* Maximum trail length compiled in (sized once for static arrays). */
  TRAIL_LEN_HARD_MAX = 20,
};

/* How far a tracer moves each tick, in cells.  A little random jitter
 * is added per tracer so they don't all march in lockstep. */
#define TRACER_STEP_BASE_CPT 0.9f
#define TRACER_STEP_JITTER_CPT 0.4f /* spread, centred on the base */

/* How fast the wind changes shape over time.  Smaller = slower drift,
 * so tracers form longer, more stable streaks. */
#define FIELD_EVOLUTION_DEFAULT 0.008f
#define FIELD_EVOLUTION_MIN 0.001f
#define FIELD_EVOLUTION_MAX 0.080f
#define FIELD_EVOLUTION_FACTOR 1.4f /* how much one F/f press multiplies it */

/* How "zoomed in" the noise is — smaller numbers make bigger swirls.
 * Using different x/y values makes the wind lean horizontal, like a
 * breeze that prefers to blow sideways. */
#define NOISE_SCALE_X 0.04f
#define NOISE_SCALE_Y 0.07f

/* How many ticks a tracer lives before it respawns somewhere fresh
 * (plus a random amount so they don't all die at once). */
#define TRACER_LIFE_BASE_TICKS 100
#define TRACER_LIFE_JITTER_TICKS 60

/* Time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ncurses colour-pair IDs.  Trail colours take the first block; the
 * HUD and key-hint colours sit just past them. */
enum {
  PAIR_TRAIL_BASE = 1, /* trail colours occupy 1..HUE_WHEEL_PAIRS */
  PAIR_HUD = PAIR_TRAIL_BASE + HUE_WHEEL_PAIRS,
  PAIR_HINT,
};

/* ── §2  clock — a steady timer for animation ──
 * We use the monotonic clock (one that only ever counts up) so the
 * animation isn't disturbed if the system clock jumps for NTP or a
 * manual time change. */

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

/* ── §3  rng — shuffled lookup table the noise draws from ──
 * The noise function gets all its randomness from one shuffled list of
 * 256 bytes.  We shuffle it once at startup, then mirror it into a
 * 512-byte copy so the noise code can add two indices together and
 * still land in range without a wraparound check. */

static uint8_t perlin_perm_table[512];

/* Start with [0, 1, ..., n-1] so the shuffle below has something to
 * scramble into a random order. */
static inline void fill_identity_permutation_u8(uint8_t *p, int n) {
    for (int i = 0; i < n; i++)
        p[i] = (uint8_t)i;
}

/* Shuffle the list into a random order, every ordering equally likely.
 * Walk from the end, swapping each slot with a random earlier one.
 * (Fisher-Yates; Knuth Vol. 2 §3.4.2.) */
static inline void fisher_yates_shuffle_u8(uint8_t *p, int n) {
    for (int i = n - 1; i > 0; i--) {
        int     j   = rand() % (i + 1);
        uint8_t tmp = p[i];
        p[i] = p[j];
        p[j] = tmp;
    }
}

/* Copy the shuffled 256 bytes twice into a 512-byte array, so adding
 * any two values in [0, 255] still lands inside it — no wraparound
 * check needed in the noise inner loop (Perlin 2002 §3). */
static inline void mirror_perm_table_to_512(const uint8_t *src256,
                                            uint8_t *dst512) {
    for (int i = 0; i < 512; i++)
        dst512[i] = src256[i & 255];
}

static void perlin_perm_init(void) {
    uint8_t identity[256];
    fill_identity_permutation_u8(identity, 256);
    fisher_yates_shuffle_u8     (identity, 256);
    mirror_perm_table_to_512    (identity, perlin_perm_table);
}

/* ── §4  perlin — smooth random noise ──
 * Perlin noise (Perlin 1985, improved 2002) gives a smooth random
 * value in [-1, +1] at any point (x, y).  Unlike rand(), nearby points
 * give nearby values, so the result looks like rolling hills, not TV
 * static — exactly what we want for a believable wind.  The grid is
 * divided into unit cells; each corner has a fixed random direction,
 * and we blend the four corners smoothly to get the value at a point. */

/* Smoothstep curve: eases in and out instead of going straight.  Using
 * it to blend the corners hides the grid — straight blending would
 * leave visible creases at the cell edges. */
static inline float smoothstep_cubic(float t) {
  return t * t * (3.0f - 2.0f * t);
}

static inline float lerp_scalar(float a, float b, float t) {
  return a + t * (b - a);
}

/* Each corner has a random direction; this measures how far in that
 * direction the query point lies (the dot product of the corner's
 * direction with the offset to the point).  The low bits of the hash
 * pick one of four directions. */
static inline float perlin_gradient_dot(int hash, float x, float y) {
  int bits = hash & 3;
  float term1 = (bits < 2) ? x : y;
  float term2 = (bits < 2) ? y : x;
  return ((hash & 1) ? -term1 : term1) + ((hash & 2) ? -term2 : term2);
}

/* Turn a grid corner (xi, yi) into a stable random number by looking it
 * up in the shuffled table twice.  Doing two lookups means the same
 * corner always gets the same direction, no matter which cell asks for
 * it — so neighbouring cells agree on the corner they share. */
static inline int hash_lattice_corner(int xi, int yi) {
    return perlin_perm_table[perlin_perm_table[xi] + yi];
}

/* Compute the four corner values for the cell the point falls in.
 * d00/d10/d01/d11 are the top-left, top-right, bottom-left and
 * bottom-right corners (y grows downward, screen-style). */
static inline void lattice_corner_dot_products(int xi, int yi,
                                               float fx, float fy,
                                               float *out_d00, float *out_d10,
                                               float *out_d01, float *out_d11) {
    *out_d00 = perlin_gradient_dot(hash_lattice_corner(xi,     yi    ),
                                    fx,        fy       );
    *out_d10 = perlin_gradient_dot(hash_lattice_corner(xi + 1, yi    ),
                                    fx - 1.f,  fy       );
    *out_d01 = perlin_gradient_dot(hash_lattice_corner(xi,     yi + 1),
                                    fx,        fy - 1.f );
    *out_d11 = perlin_gradient_dot(hash_lattice_corner(xi + 1, yi + 1),
                                    fx - 1.f,  fy - 1.f );
}

/* Blend the four corner values into one, mixing left-to-right then
 * top-to-bottom.  Caller passes blend weights already eased through
 * smoothstep so cell edges don't show. */
static inline float bilinear_smooth_interp(float d00, float d10,
                                            float d01, float d11,
                                            float ux, float uy) {
    float top    = lerp_scalar(d00, d10, ux);
    float bottom = lerp_scalar(d01, d11, ux);
    return lerp_scalar(top, bottom, uy);
}

static float perlin_value(float x, float y) {
    int xi = (int)floorf(x) & 255;
    int yi = (int)floorf(y) & 255;

    float fx = x - floorf(x);
    float fy = y - floorf(y);

    float ux = smoothstep_cubic(fx);
    float uy = smoothstep_cubic(fy);

    float d00, d10, d01, d11;
    lattice_corner_dot_products(xi, yi, fx, fy, &d00, &d10, &d01, &d11);

    return bilinear_smooth_interp(d00, d10, d01, d11, ux, uy);
}

/* ── §5  fbm — stacking noise layers for richer detail ──
 * One layer of Perlin noise has detail at a single size.  Real flow has
 * detail at every size: huge swirls, medium eddies, tiny ripples.  So
 * we add several layers, each one twice as fine and half as strong as
 * the last.  The sum looks far more natural.  (Mandelbrot; Voss,
 * "Random Fractal Forgeries".) */

/* Add one more, finer, fainter layer of noise to the running total. */
static inline void accumulate_perlin_octave(float *running_sum,
                                            float x, float y,
                                            float frequency, float amplitude) {
    *running_sum += perlin_value(x * frequency, y * frequency) * amplitude;
}

static float fbm_value(float x, float y, int octaves) {
    float result    = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    for (int oct = 0; oct < octaves; oct++) {
        accumulate_perlin_octave(&result, x, y, frequency, amplitude);
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }
    return result;
}

/* ── §6  field — the invisible wind: build, sample, evolve ──
 * The wind is just a grid with one angle stored per cell — the
 * direction the wind blows there.  We get the angle by sampling the
 * noise twice (for an x-part and a y-part) and asking atan2 which way
 * that pair points.  A slowly advancing "time" value keeps the whole
 * pattern drifting so it never sits still. */

#define FIELD_COLS_MAX 256
#define FIELD_ROWS_MAX 80

/*
 * flow_field — the invisible wind that every tracer rides.
 *
 * We work out a direction for every cell up front and store it, rather
 * than recomputing noise for each tracer.  Computing it once per cell
 * is cheaper than once per tracer (there are far more tracers), and it
 * lets a tracer sitting between cells blend its four neighbours for a
 * smooth direction.
 *
 * The grid is a plain fixed array (no malloc) so the running animation
 * never allocates memory — 256x80 floats is only ~80 KB.  On a window
 * resize we just shrink the active part; the array stays put.
 */
typedef struct {
    /* The part of the grid actually in use.  Capped at the *_MAX sizes
     * and re-clamped whenever the terminal is resized. */
    int   active_cols;
    int   active_rows;

    /* The wind's clock.  time_axis creeps forward by evolution_speed
     * every tick, which slides the noise pattern along and makes the
     * wind morph.  They're kept separate so the user can speed up or
     * slow down the morphing (f/F keys change evolution_speed) without
     * jolting the pattern. */
    float time_axis;
    float evolution_speed;

    /* The wind direction at each cell, in radians, refreshed every
     * tick.  Tracers read this (blending neighbours) and never touch
     * the noise generators themselves. */
    float angle[FIELD_ROWS_MAX][FIELD_COLS_MAX];
} flow_field;

static void field_init(flow_field *f, int cols, int rows) {
  if (cols > FIELD_COLS_MAX)
    cols = FIELD_COLS_MAX;
  if (rows > FIELD_ROWS_MAX)
    rows = FIELD_ROWS_MAX;
  f->active_cols = cols;
  f->active_rows = rows;
  f->time_axis = 0.0f;
  f->evolution_speed = FIELD_EVOLUTION_DEFAULT;
  memset(f->angle, 0, sizeof f->angle);
}

/* Nudge the wind's clock forward one step.  Kept next to the field it
 * belongs to so we can't accidentally advance it twice or forget. */
static inline void advance_noise_clock(flow_field *f) {
    f->time_axis += f->evolution_speed;
}

/* Get the wind vector (vx, vy) at a cell by sampling the noise twice.
 * The second sample is shifted far away in noise-space (the +100.3 /
 * +200.7 offsets) so the two readings are unrelated — otherwise vx and
 * vy would move together and the wind would only ever point along one
 * diagonal. */
static inline void sample_velocity_components_at(float c, float r, float t,
                                                  float *out_vx,
                                                  float *out_vy) {
    *out_vx = fbm_value(c * NOISE_SCALE_X         + t,
                        r * NOISE_SCALE_Y         + t * 0.7f,
                        FBM_OCTAVES);
    *out_vy = fbm_value(c * NOISE_SCALE_X + 100.3f + t * 1.1f,
                        r * NOISE_SCALE_Y + 200.7f + t * 0.5f,
                        FBM_OCTAVES);
}

/* Turn a wind vector into the single angle we store.  atan2 handles
 * all directions (plain atan can't tell left from right). */
static inline float velocity_to_angle(float vx, float vy) {
    return atan2f(vy, vx);
}

/* Move the wind forward one step and recompute every cell's direction.
 * We rebuild the whole grid each tick for smooth motion — it's only a
 * fraction of a millisecond. */
static void field_evolve_and_rebuild(flow_field *f) {
    advance_noise_clock(f);
    float t = f->time_axis;
    for (int r = 0; r < f->active_rows; r++) {
        for (int c = 0; c < f->active_cols; c++) {
            float vx, vy;
            sample_velocity_components_at((float)c, (float)r, t, &vx, &vy);
            f->angle[r][c] = velocity_to_angle(vx, vy);
        }
    }
}

/* The exact wind direction at one whole-number cell.  Used to draw the
 * arrow overlay; out-of-range cells clamp to the edge. */
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

/* Keep a cell index inside the grid so the +1 neighbour below never
 * reads past the edge. */
static inline int clamp_cell_to_active(int idx, int max_count) {
    if (idx < 0)              return 0;
    if (idx >= max_count)     return max_count - 1;
    return idx;
}

/* How far the point sits into its cell, 0 at the left/top edge up to
 * just under 1 at the right/bottom — the weights for blending. */
static inline void compute_subcell_offsets_xy(float col, float row,
                                              float *out_fx, float *out_fy) {
    *out_fx = col - floorf(col);
    *out_fy = row - floorf(row);
}

/*
 * Wind direction at a fractional position, blended from the four
 * surrounding cells so a tracer between cells gets a smooth answer.
 *
 * Strictly speaking, blending angles is wrong where they wrap from +π
 * to -π — but the wind turns so gently between neighbours that a tracer
 * almost never straddles that seam, so it's invisible here.
 */
static float flow_angle_bilinear(const flow_field *f, float col, float row) {
    int c0 = clamp_cell_to_active((int)floorf(col),     f->active_cols);
    int c1 = clamp_cell_to_active((int)floorf(col) + 1, f->active_cols);
    int r0 = clamp_cell_to_active((int)floorf(row),     f->active_rows);
    int r1 = clamp_cell_to_active((int)floorf(row) + 1, f->active_rows);

    float fx, fy;
    compute_subcell_offsets_xy(col, row, &fx, &fy);

    /* Reuse the blend helper from §4, but with plain weights — the
     * wind is already smooth, so no easing is needed here. */
    return bilinear_smooth_interp(f->angle[r0][c0], f->angle[r0][c1],
                                   f->angle[r1][c0], f->angle[r1][c1],
                                   fx, fy);
}

/* ── §7  arrows — turn a direction into an arrow glyph and a colour ──
 * We round each angle to one of 8 compass directions, then look up the
 * matching arrow character and the matching rainbow colour. */

#define ARROW_TABLE_LEN 8
static const char arrow_glyph_table[ARROW_TABLE_LEN] = {
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
  int octant = (int)(a / (2.0f * (float)M_PI) * ARROW_TABLE_LEN + 0.5f) %
               ARROW_TABLE_LEN;
  return octant;
}

static inline char arrow_glyph_for_angle(float angle_radians) {
  return arrow_glyph_table[angle_to_octant(angle_radians)];
}

static inline int hue_pair_for_angle(float angle_radians) {
  return PAIR_TRAIL_BASE + angle_to_octant(angle_radians);
}

/* ── §8  themes — colour palettes (just data) ──
 * Each theme is a row of 8 colours.  The rainbow theme spreads its 8
 * across the colour wheel (so direction picks the colour); the mono
 * themes are one hue from bright to dim (so newer trail = brighter).
 * One table for full-colour terminals, a fallback table for 8-colour
 * ones.  Slot 1 is brightest, slot 8 dimmest. */

static const char *theme_name_table[THEME_COUNT] = {"rainbow", "cyan", "green",
                                                    "white"};

static const int theme_palette_256[THEME_COUNT][HUE_WHEEL_PAIRS] = {
    /* RAINBOW — 8 hues spread around the wheel. */
    {196, 208, 226, 46, 51, 33, 129, 201},
    /* CYAN — bright to dim. */
    {51, 45, 39, 33, 27, 26, 25, 25},
    /* GREEN — bright to dim. */
    {82, 46, 40, 34, 28, 28, 28, 28},
    /* WHITE / GREY — bright to dim. */
    {255, 250, 247, 245, 243, 241, 240, 240},
};

static const int theme_palette_8[THEME_COUNT][HUE_WHEEL_PAIRS] = {
    {COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN, COLOR_BLUE,
     COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE},
    {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
     COLOR_BLUE, COLOR_BLUE},
    {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
     COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
    {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
};

/* ── §9  colors — hand the palette to ncurses ──
 * Build the trail colour pairs from the chosen theme, plus the fixed
 * HUD (yellow) and hint (cyan) colours.  Everything uses -1 for the
 * background so the terminal's own background shows through. */

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();

  bool has_256 = (COLORS >= 256);
  const int *palette =
      has_256 ? theme_palette_256[theme_index] : theme_palette_8[theme_index];

  for (int i = 0; i < HUE_WHEEL_PAIRS; i++)
    init_pair(PAIR_TRAIL_BASE + i, palette[i], -1);

  if (has_256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* ── §10  tracer — one particle and its trail ──
 *
 * A tracer is one drifting particle.  Each tick it reads the wind where
 * it stands, steps that way, and remembers where it just was so we can
 * draw a fading trail behind it.
 *
 * The trail is a ring buffer: a fixed-size array used in a loop.  We
 * keep the last few positions, overwriting the oldest as we go — so the
 * trail stays a fixed length and we never allocate memory while running.
 *
 * Tracers don't bounce off walls; they live for a random number of
 * ticks and then respawn at a fresh random spot.  The randomness keeps
 * them from all dying on the same frame, which would look like popping.
 */
typedef struct {
    /* Is this slot in use, and how many ticks until it respawns. */
    bool  tracer_alive;
    int   ticks_until_respawn;

    /* Where it is (fractional cell position), how far it moves per tick,
     * and the wind direction it last followed (used to draw its head). */
    float pos_col;
    float pos_row;
    float step_cells;
    float last_angle;

    /* The trail ring buffer: recent positions plus the bookkeeping for
     * the loop.  trail_pair_base is this tracer's colour, fixed for its
     * whole life so it doesn't flicker.  trail_write_index is where the
     * next position goes; trail_filled_count is how many slots are real
     * yet; trail_active_length is the current trail length (s/S keys). */
    int   trail_pair_base;
    int   trail_col[TRAIL_LEN_HARD_MAX];
    int   trail_row[TRAIL_LEN_HARD_MAX];
    int   trail_write_index;
    int   trail_filled_count;
    int   trail_active_length;
} tracer;

/* Uniform random spawn point inside the active field rect. */
static inline void pick_random_spawn_position(int active_cols, int active_rows,
                                              float *out_col, float *out_row) {
    *out_col = (float)(rand() % active_cols);
    *out_row = (float)(rand() % active_rows);
}

/* Pick a slightly random speed for this tracer so neighbours don't all
 * move in step, which would look like a marching grid. */
static inline float pick_jittered_step_speed_cpt(void) {
    return TRACER_STEP_BASE_CPT
         - TRACER_STEP_JITTER_CPT * 0.5f
         + TRACER_STEP_JITTER_CPT * ((float)rand() / (float)RAND_MAX);
}

/* Pick a random lifetime so the tracers don't all respawn together. */
static inline int pick_random_lifetime_ticks(void) {
    return TRACER_LIFE_BASE_TICKS + rand() % TRACER_LIFE_JITTER_TICKS;
}

/* Pick this tracer's colour, fixed for its whole life.  (For the
 * rainbow theme the per-tick step overrides it with the direction's
 * colour; mono themes keep this one.) */
static inline int pick_random_palette_pair_base(void) {
    return 1 + rand() % HUE_WHEEL_PAIRS;
}

/* Seed the whole trail with the spawn spot so the very first frame
 * doesn't show a gap between the tracer and its empty trail. */
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

/* Give a tracer a fresh start: new spot, speed, colour and lifetime.
 * Runs at startup, when a tracer's time runs out, and on the 'r' key. */
static void tracer_respawn(tracer *t, int active_cols, int active_rows,
                           int trail_active_length) {
    t->tracer_alive        = true;
    pick_random_spawn_position(active_cols, active_rows,
                               &t->pos_col, &t->pos_row);
    t->step_cells          = pick_jittered_step_speed_cpt();
    t->last_angle          = 0.0f;
    t->ticks_until_respawn = pick_random_lifetime_ticks();
    t->trail_pair_base     = pick_random_palette_pair_base();
    reset_trail_ring_at(t, (int)t->pos_col, (int)t->pos_row,
                        trail_active_length);
}

/* ── §11  tracer_step — move one tracer one tick ──
 * The heart of the demo: remember where we are, read the wind, step
 * that way, wrap around the edges, recolour, and age by one tick. */

/* Drop the tracer's current spot into the trail, advancing the write
 * position around the ring. */
static inline void push_position_into_trail(tracer *t) {
    t->trail_col[t->trail_write_index] = (int)t->pos_col;
    t->trail_row[t->trail_write_index] = (int)t->pos_row;
    t->trail_write_index = (t->trail_write_index + 1) % t->trail_active_length;
    if (t->trail_filled_count < t->trail_active_length)
        t->trail_filled_count++;
}

/* Read the wind where the tracer stands and remember it, so the drawing
 * code later shows the same direction the tracer actually moved. */
static inline float sample_field_at_tracer(tracer *t, const flow_field *f) {
    float angle = flow_angle_bilinear(f, t->pos_col, t->pos_row);
    t->last_angle = angle;
    return angle;
}

/* Take one step in the wind's direction, scaled by the tracer's speed.
 * Steps are small, so the slight curve we miss between steps doesn't
 * show. */
static inline void step_tracer_along_angle(tracer *t, float angle) {
    t->pos_col += cosf(angle) * t->step_cells;
    t->pos_row += sinf(angle) * t->step_cells;
}

/* Wrap a tracer that leaves an edge back in on the opposite side, so
 * nothing ever disappears off-screen. */
static inline void wrap_position_toroidally(tracer *t,
                                            int active_cols, int active_rows) {
    if (t->pos_col <  0.0f)              t->pos_col += (float)active_cols;
    if (t->pos_col >= (float)active_cols) t->pos_col -= (float)active_cols;
    if (t->pos_row <  0.0f)              t->pos_row += (float)active_rows;
    if (t->pos_row >= (float)active_rows) t->pos_row -= (float)active_rows;
}

/* In the rainbow theme, colour the tracer by which way it's heading, so
 * different directions read as different colours.  Other themes leave
 * the tracer's fixed colour alone. */
static inline void maybe_recolour_for_rainbow_theme(tracer *t, float angle,
                                                    int theme_index) {
    if (theme_index == 0)
        t->trail_pair_base = hue_pair_for_angle(angle);
}

/* Count down the tracer's life; mark it dead when it runs out so the
 * next tick respawns it. */
static inline void age_tracer_and_check_death(tracer *t) {
    t->ticks_until_respawn--;
    if (t->ticks_until_respawn <= 0)
        t->tracer_alive = false;
}

static void tracer_advance_one_tick(tracer *t, const flow_field *f,
                                    int active_cols, int active_rows,
                                    int theme_index) {
    if (!t->tracer_alive)
        return;

    push_position_into_trail(t);
    float angle = sample_field_at_tracer(t, f);
    step_tracer_along_angle(t, angle);
    wrap_position_toroidally(t, active_cols, active_rows);
    maybe_recolour_for_rainbow_theme(t, angle, theme_index);
    age_tracer_and_check_death(t);
}

/* ── §12  tracer_paint — draw one tracer's fading trail ──
 * Walk the trail from oldest to newest so the head draws last and sits
 * on top.  Older cells get fainter glyphs and dimmer colours; the head
 * gets an arrow showing its direction. */

#define TRAIL_RAMP_LEN 5
static const char trail_ramp_glyph[TRAIL_RAMP_LEN] = {'.', ',', '+', '~', '*'};

/* The arrow character drawn at the head, picked from the same 8
 * directions as §7. */
static char trail_head_glyph_for_angle(float angle_radians) {
  static const char head_dir_glyph[ARROW_TABLE_LEN] = {'-', '/', '|', '\\',
                                                       '-', '/', '|', '\\'};
  return head_dir_glyph[angle_to_octant(angle_radians)];
}

/* Turn walk-step i (0 = oldest, filled-1 = newest) into the real array
 * slot.  The `+ 2*len` keeps the index positive before the wrap, since
 * C's % can return a negative result. */
static inline int trail_slot_index(const tracer *t, int i,
                                   int filled, int len) {
    return (t->trail_write_index - filled + i + 2 * len) % len;
}

/* Is this trail position still on the visible screen? */
static inline bool position_in_field(int col, int row,
                                     int active_cols, int active_rows) {
    return col >= 0 && col < active_cols && row >= 0 && row < active_rows;
}

/* Glyph for one trail cell: the head gets its direction arrow, the rest
 * fade through '.' ',' '+' '~' as they get newer (the brightest '*' is
 * saved for the head). */
static inline char pick_trail_glyph(const tracer *t, int i, int filled,
                                    bool is_head) {
    if (is_head)
        return trail_head_glyph_for_angle(t->last_angle);
    int ramp_index = (i * (TRAIL_RAMP_LEN - 1)) /
                     (filled > 1 ? filled - 1 : 1);
    if (ramp_index >= TRAIL_RAMP_LEN - 1)
        ramp_index = TRAIL_RAMP_LEN - 2;   /* head's '*' reserved */
    return trail_ramp_glyph[ramp_index];
}

/* Colour for one trail cell: bright at the head, fading toward the tail.
 * Mainly for the mono themes — the rainbow theme already recolours the
 * whole tracer each frame. */
static inline int pick_trail_pair_fading(const tracer *t, int i, int filled,
                                          bool is_head) {
    int pair;
    if (is_head) {
        pair = t->trail_pair_base;
    } else {
        int age_from_head = filled - 1 - i;
        int divisor       = (filled > 1 ? filled - 1 : 1);
        pair = t->trail_pair_base -
               (age_from_head * (t->trail_pair_base - 1)) / divisor;
    }
    if (pair < 1)                 pair = 1;
    if (pair > HUE_WHEEL_PAIRS)   pair = HUE_WHEEL_PAIRS;
    return pair;
}

/* Draw one trail cell, bolding it if it's the head. */
static inline void paint_trail_cell(WINDOW *win, int row, int col,
                                    char glyph, int pair_id, bool is_head) {
    attr_t a = COLOR_PAIR(pair_id);
    if (is_head)
        a |= A_BOLD;
    wattron(win, a);
    mvwaddch(win, row, col, (chtype)(unsigned char)glyph);
    wattroff(win, a);
}

static void tracer_paint(const tracer *t, WINDOW *win, int active_cols,
                         int active_rows) {
    if (!t->tracer_alive)             return;
    if (t->trail_filled_count == 0)   return;

    int filled = t->trail_filled_count;
    int len    = t->trail_active_length;

    for (int i = 0; i < filled; i++) {
        int slot = trail_slot_index(t, i, filled, len);
        int col  = t->trail_col[slot];
        int row  = t->trail_row[slot];

        if (!position_in_field(col, row, active_cols, active_rows))
            continue;

        bool is_head = (i == filled - 1);
        char glyph   = pick_trail_glyph     (t, i, filled, is_head);
        int  pair    = pick_trail_pair_fading(t, i, filled, is_head);

        paint_trail_cell(win, row, col, glyph, pair, is_head);
    }
}

/* ── §13  scene — holds everything and runs each tick ── */

/*
 * scene_state — all of the demo's live state in one place.
 *
 * It holds the wind, the pool of tracers riding it, and the user's
 * display choices (theme, whether arrows show, paused).  The fields are
 * grouped by who uses them so it's clear at a glance whether changing
 * one affects the simulation or only the picture.  Keeping the display
 * choices out of the simulation matters: if the tick ever read the
 * theme, the same noise seed would stop producing the same wind.
 *
 * The tracer pool is sized for the maximum up front and partly used, so
 * a window resize never has to allocate.
 */
typedef struct {
    /* The simulation itself: the wind, and the tracers moving in it. */
    flow_field flow;
    tracer     pool[TRACERS_MAX];

    /* How many tracers are live, and how long their trails are.  Both
     * the tick and the drawing loop over these, and the user changes
     * them with +/- (count) and s/S (trail length). */
    int active_tracer_count;
    int trail_active_length;

    /* Display-only choices — changing these must not touch the
     * simulation, only what gets drawn. */
    int  theme_index;            /* which palette */
    bool show_field_arrows;      /* show the wind arrows underneath? */

    /* Paused freezes the tick and shows "PAUSED" in the HUD. */
    bool paused;
} scene_state;

static void scene_init(scene_state *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->active_tracer_count = TRACERS_DEFAULT;
  s->trail_active_length = TRAIL_LEN_DEFAULT;
  s->theme_index = 0;
  s->show_field_arrows = false;
  s->paused = false;

  field_init(&s->flow, cols, rows);
  field_evolve_and_rebuild(&s->flow);

  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_resize(scene_state *s, int cols, int rows) {
  field_init(&s->flow, cols, rows);
  field_evolve_and_rebuild(&s->flow);
  for (int i = 0; i < TRACERS_MAX; i++)
    if (s->pool[i].tracer_alive)
      tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_respawn_all(scene_state *s, int cols, int rows) {
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
    tracer_advance_one_tick(&s->pool[i], &s->flow, cols, rows, s->theme_index);
  }
}

/* Draw an arrow in every cell showing the wind there (only when the 'a'
 * overlay is on).  Arrows are normal weight so they stay visible but
 * don't fight with the bold tracer heads on top. */
static void scene_paint_field_arrows(const scene_state *s, WINDOW *win,
                                     int cols, int rows) {
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float angle = flow_angle_at_cell(&s->flow, c, r);
      char glyph = arrow_glyph_for_angle(angle);
      int pair = (s->theme_index == 0) ? hue_pair_for_angle(angle) : 1;
      wattron(win, COLOR_PAIR(pair));
      mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
      wattroff(win, COLOR_PAIR(pair));
    }
  }
}

static void scene_paint(const scene_state *s, WINDOW *win, int cols, int rows) {
  if (s->show_field_arrows)
    scene_paint_field_arrows(s, win, cols, rows);

  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_paint(&s->pool[i], win, cols, rows);
}

/* ── §14  hud — status line on top, key hints on the bottom ── */

static void hud_paint_status(WINDOW *win, int cols, double fps, int sim_hz,
                             const scene_state *s) {
  char buf[160];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%2dHz  tracers:%3d  trail:%2d  "
           "theme:%-7s  field-spd:%.4f  %s ",
           fps, sim_hz, s->active_tracer_count, s->trail_active_length,
           theme_name_table[s->theme_index], (double)s->flow.evolution_speed,
           s->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = cols - len;
  if (x < 0)
    x = 0;
  wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwprintw(win, 0, x, "%s", buf);
  wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hints(WINDOW *win, int rows) {
  const char *hint =
      " q:quit  spc:pause  r:respawn  a:arrows  t:theme  +/-:tracers  "
      "s/S:trail  ]/[:simHz  f/F:field-spd ";
  wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(win, rows - 1, 0, "%s", hint);
  wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §15  screen — ncurses setup and teardown ── */

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

/* ── §16  app — main loop, signals, keyboard ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

/* Returns false if the user wants to quit. */
static bool app_handle_key(int key, scene_state *s, int *sim_hz, int cols,
                           int rows) {
  switch (key) {
  case 'q':
  case 'Q':
  case 27: /* 27 = ESC */
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 'r':
  case 'R':
    scene_respawn_all(s, cols, rows);
    break;

  case 'a':
  case 'A':
    s->show_field_arrows = !s->show_field_arrows;
    break;

  case 't':
  case 'T':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    colors_init(s->theme_index);
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
  /* Seed randomness from the clock, then build the noise lookup table. */
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

  /* Keeps the simulation running at a steady rate no matter how fast we
   * draw (Glenn Fiedler, "Fix Your Timestep!"). */
  int64_t prev_ns = clock_now_ns();
  int64_t sim_accum_ns = 0;

  /* Counts frames over a short window to show a smooth fps number. */
  int frames_in_window = 0;
  int64_t window_accum_ns = 0;
  double fps_display = 0.0;

  const int64_t frame_cap_ns = NS_PER_SEC / TARGET_FPS_HZ;

  while (!g_should_quit) {
    int64_t frame_start = clock_now_ns();

    /* Handle a window resize, flagged by a signal, here at the top. */
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_resize(&scene, cols, rows);
      sim_accum_ns = 0;
    }

    /* Time since last frame, capped so a long stall (e.g. laptop sleep)
     * can't make the sim try to catch up with a huge burst of ticks. */
    int64_t dt_ns = frame_start - prev_ns;
    prev_ns = frame_start;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    /* Run as many fixed-rate sim steps as the elapsed time has earned. */
    int64_t tick_ns = NS_PER_SEC / sim_hz;
    sim_accum_ns += dt_ns;
    while (sim_accum_ns >= tick_ns) {
      scene_tick(&scene, cols, rows);
      sim_accum_ns -= tick_ns;
    }

    /* Draw frame. */
    erase();
    scene_paint(&scene, stdscr, cols, rows);
    hud_paint_status(stdscr, cols, fps_display, sim_hz, &scene);
    hud_paint_hints(stdscr, rows);
    wnoutrefresh(stdscr);
    doupdate();

    /* FPS readout — refresh every ~500 ms. */
    frames_in_window++;
    window_accum_ns += dt_ns;
    if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      fps_display = (double)frames_in_window /
                    ((double)window_accum_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      window_accum_ns = 0;
    }

    /* Drain input queue. */
    int key;
    while ((key = getch()) != ERR) {
      if (!app_handle_key(key, &scene, &sim_hz, cols, rows)) {
        g_should_quit = 1;
        break;
      }
    }

    /* Sleep to cap render rate. */
    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
