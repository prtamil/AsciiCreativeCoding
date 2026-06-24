/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snow.c — falling snowflakes that drift, sway, and pile up at the bottom.
 *
 * Each flake falls on its own, wobbling side to side as it goes, while a
 * steady wind nudges the whole field along. When a flake reaches the snow
 * pile at its column it lands and adds a cell of snow — and if a neighbour
 * column is shorter, it rolls there instead, so dips fill in before peaks
 * and the pile settles into natural-looking drifts. Three presets (FLURRY,
 * SNOWFALL, BLIZZARD) run on one engine; only their numbers differ.
 *
 * Sister file: particle_systems/rain.c shares the pool + pattern setup;
 *   grids/cell_grids/sandpile.c is where the roll-into-lower-neighbour idea
 *   comes from.
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

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_FLAKES = 900,
  MAX_COLS = 800, /* how many columns of snow pile we can ever track */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour slot numbers. The first two are reserved for the HUD project-wide. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_FLAKE_BASE = 3, /* +0..+7: eight flake colours, faint small to bright big */
  PAIR_PILE_BASE = 11, /* +0..+7: eight pile colours, dark deep to bright surface */
  PAIR_SKY = 19,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* A little randomness per flake so they don't all move in lockstep. */
#define FLAKE_SPEED_VARIANCE 0.50f /* fall speed varies by ±25% per flake */
#define FLAKE_WIND_JITTER 1.0f     /* wind varies by ±0.5 cells/sec per flake */

/* How tall the snow pile is allowed to get: 15% of the screen, then it stops. */
#define PILE_MAX_FRAC 0.15f

/* How much one press of w / W shifts the wind. */
#define WIND_STEP 3.0f

/* The three snowfall presets. */
typedef enum {
  PATTERN_FLURRY = 0,
  PATTERN_SNOWFALL = 1,
  PATTERN_BLIZZARD = 2,
  N_PATTERNS = 3,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_FLURRY:
    return "FLURRY  ";
  case PATTERN_SNOWFALL:
    return "SNOWFALL";
  case PATTERN_BLIZZARD:
    return "BLIZZARD";
  default:
    return "?       ";
  }
}

/*
 * PatternParams — the numbers that make one snowfall preset look different
 * from another. The engine never checks "which preset is this?"; it just
 * reads these fields and runs. Same code, three very different snowfalls.
 *
 *   target_flakes  : how many flakes to keep in the air. The spawn loop
 *                    tops up toward this each tick (gently, so resuming
 *                    after a pause doesn't dump a flood). More = denser sky.
 *                    FLURRY 120 (sparse), BLIZZARD 700 (whiteout).
 *
 *   fall_speed     : how fast a flake falls, in cells/sec. Real snow drops
 *                    at a steady speed almost at once, so we just use a
 *                    constant and add a little per-flake variation.
 *                    Flurries crawl at 10, blizzards rush at 45.
 *
 *   wind_x         : steady sideways push, in cells/sec, given to each flake
 *                    when it spawns. Positive blows right, negative left.
 *                    The w / W keys add an extra push on top of this.
 *
 *   sway_amp_min,  : how wide a flake wobbles, in cells. Each flake picks a
 *   sway_amp_max     value somewhere in this range when it spawns.
 *                    FLURRY 1.5..4.0 (big lazy swings); BLIZZARD 0.3..1.0
 *                    (almost straight, because the wind takes over).
 *
 *   sway_freq_min, : how fast a flake wobbles back and forth. Each flake
 *   sway_freq_max    picks a value in this range at spawn. FLURRY 0.4..1.3
 *                    (slow gentle swing); BLIZZARD 0.2..0.8.
 *
 *   pile_growth_mul: chance (0..1) that a landing flake actually adds to the
 *                    pile. BLIZZARD 1.0 (every hit sticks); FLURRY 0.60 (most
 *                    melt away) so a light flurry builds up slower than a
 *                    blizzard even with the same flakes. Purely a look knob,
 *                    not real physics.
 *
 * The idea of one engine driven by a small table of constants per kind of
 * snowfall comes from Reeves' particle-systems paper (1983, §4).
 */
typedef struct {
  int target_flakes;
  float fall_speed;
  float wind_x;
  float sway_amp_min, sway_amp_max;
  float sway_freq_min, sway_freq_max;
  float pile_growth_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* FLURRY    */ {120, 10.0f, 3.0f, 1.5f, 4.0f, 0.40f, 1.30f, 0.60f},
    /* SNOWFALL  */ {320, 18.0f, 6.0f, 0.8f, 2.5f, 0.30f, 1.10f, 0.85f},
    /* BLIZZARD  */ {700, 45.0f, 22.0f, 0.3f, 1.0f, 0.20f, 0.80f, 1.00f},
};

/*
 * Theme — one colour scheme. flake[8] runs faint (small flakes) to bright
 * (big flakes); pile[8] runs dark (deep, old snow) to bright (fresh surface).
 * All colours stay in the bright half of the palette so even the darkest tier
 * is still visible against a black terminal.
 */
typedef struct {
  const char *name;
  short flake[8]; /* small flakes to big */
  short pile[8];  /* deep snow to surface */
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* each row: name, flake colours (small to big), pile colours (deep to top), sky */

    {"MATRIX",
     {28, 34, 40, 46, 82, 118, 154, 190},
     {28, 34, 40, 46, 82, 118, 154, 190},
     234},
    {"FIRE",
     {88, 124, 130, 166, 202, 208, 214, 226},
     {52, 88, 124, 160, 196, 202, 208, 220},
     233},
    {"OCEANIC",
     {24, 31, 38, 44, 51, 87, 159, 195},
     {24, 25, 31, 38, 44, 51, 87, 159},
     234},
    {"NEON",
     {53, 91, 134, 165, 201, 207, 213, 219},
     {53, 91, 134, 165, 201, 207, 213, 225},
     234},
    {"MONO",
     {244, 246, 248, 250, 252, 253, 254, 255},
     {240, 244, 247, 249, 251, 253, 254, 255},
     232},
    {"ICE",
     {117, 153, 159, 195, 225, 231, 254, 255},
     {110, 117, 153, 159, 195, 231, 254, 255},
     235},
    {"NOVA",
     {24, 75, 117, 159, 195, 219, 226, 231},
     {60, 75, 117, 159, 195, 219, 226, 231},
     234},
    {"FOREST",
     {28, 64, 70, 76, 112, 148, 184, 220},
     {28, 64, 70, 76, 112, 148, 184, 220},
     234},
    {"DESERT",
     {94, 130, 137, 173, 179, 215, 222, 229},
     {94, 130, 137, 143, 179, 215, 222, 229},
     234},
    {"ECLIPSE",
     {52, 88, 95, 131, 167, 173, 209, 215},
     {52, 88, 95, 131, 167, 173, 209, 215},
     232},
};

/* The character a flake is drawn with, picked by its size (small to big). */
static const char FLAKE_GLYPHS[8] = {'`', '.', '\'', ',', ':', '+', '*', '*'};

/* ── §2 clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3 color ── */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++) {
      init_pair((short)(PAIR_FLAKE_BASE + i), t->flake[i], -1);
      init_pair((short)(PAIR_PILE_BASE + i), t->pile[i], -1);
    }
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++) {
      init_pair((short)(PAIR_FLAKE_BASE + i), COLOR_WHITE, -1);
      init_pair((short)(PAIR_PILE_BASE + i), COLOR_WHITE, -1);
    }
    init_pair(PAIR_SKY, COLOR_BLACK, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── §4 flake ── */

/*
 * Flake — one falling snowflake, holding everything we need to move and draw
 * it. They all live in one fixed array (no allocation while running); the
 * `active` flag says whether a slot is in use, and reused slots host new
 * flakes as old ones land.
 *
 * The wobble is the one subtle bit: the spot we actually draw isn't center_x,
 * it's center_x plus a side-to-side sine wiggle. Keeping the drifting centre
 * separate from the wiggle lets the wind push the average path while the flake
 * still sways around it. Each flake gets a random starting phase so they don't
 * all swing together (which would look like a marching band).
 *
 *   center_x   : the flake's average horizontal spot, in cells. The wind
 *                slides this along; the drawn position adds the wiggle on top.
 *
 *   y          : vertical position in cells. Grows downward (ncurses counts
 *                rows top-down) at speed vy until it lands or drifts off-screen.
 *
 *   vy         : fall speed in cells/sec, fixed for this flake's life with a
 *                small random offset so the field doesn't fall in lockstep.
 *
 *   drift_vx   : steady sideways speed in cells/sec (preset wind + your w/W
 *                override + a touch of randomness), fixed once it spawns.
 *
 *   sway_amp   : how far the wiggle reaches, in cells. Bigger = wider swing.
 *
 *   sway_freq  : how fast the wiggle cycles. One full back-and-forth takes
 *                2*pi / sway_freq seconds.
 *
 *   sway_phase : a random starting point for the wiggle (0..2*pi) so flakes
 *                don't all line up and swing in unison.
 *
 *   age        : seconds since this flake spawned. Only used as the clock that
 *                drives the wiggle; it never kills the flake (landing or
 *                leaving the screen does that).
 *
 *   size_idx   : size bucket 0..7, picked at spawn. Chooses both the character
 *                (FLAKE_GLYPHS) and the colour. Bigger looks denser and brighter.
 *
 *   active     : is this slot in use? Inactive slots are skipped everywhere;
 *                spawning grabs the first inactive one it finds.
 *
 * The fixed-pool design and the side-to-side sine wobble follow Reeves'
 * particle-systems paper (1983) and Bourg's "Physics for Game Developers".
 */
typedef struct {
  float center_x;
  float y;
  float vy;
  float drift_vx;
  float sway_amp;
  float sway_freq;
  float sway_phase;
  float age;
  int size_idx;
  bool active;
} Flake;

/* Cheap random-number generator. State lives in the Scene, not a global, so
 * each scene's randomness is its own. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24); /* a number in [0, 1) */
}

/* Where the flake is actually drawn right now: its centre plus the wiggle. */
static inline float flake_x(const Flake *f) {
  return f->center_x +
         f->sway_amp * sinf(f->sway_freq * f->age + f->sway_phase);
}

/* Pick a random number between lo and hi. The other samplers build on this. */
static inline float sample_uniform_in_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}

/* Fall speed for one flake: the preset speed nudged up or down a bit so the
 * field doesn't drop in perfect unison. */
static inline float sample_terminal_velocity_jittered(uint32_t *rng,
                                                      float base_vy) {
  float scale = (1.0f - FLAKE_SPEED_VARIANCE * 0.5f) +
                lcg_unit(rng) * FLAKE_SPEED_VARIANCE;
  return base_vy * scale;
}

/* Sideways speed for one flake: the wind plus a small random nudge. */
static inline float sample_drift_velocity_jittered(uint32_t *rng,
                                                   float wind_base) {
  float jitter = (lcg_unit(rng) - 0.5f) * 2.0f * FLAKE_WIND_JITTER;
  return wind_base + jitter;
}

/* A random starting point for the wiggle so flakes don't all swing together. */
static inline float sample_random_phase_2pi(uint32_t *rng) {
  return lcg_unit(rng) * 2.0f * (float)M_PI;
}

/* Pick a size bucket, weighted so most flakes are small and big ones are rare
 * (squaring a 0..1 number bunches the result toward the low end). */
static inline int sample_size_class_small_biased(uint32_t *rng) {
  float r = lcg_unit(rng);
  int idx = (int)(r * r * 7.999f);
  if (idx < 0)
    idx = 0;
  if (idx > 7)
    idx = 7;
  return idx;
}

/* Pick where a new flake appears across the top. When there's wind we let it
 * start a bit past the upwind edge, so flakes blowing across keep the whole
 * screen evenly filled instead of thinning out on the side they blow from. */
static inline float sample_spawn_x_wind_extended(uint32_t *rng, int cols,
                                                 float wind) {
  float over = fabsf(wind) * 0.5f;
  float r = lcg_unit(rng);
  if (wind > 0.5f)
    return r * ((float)cols + over) - over;
  if (wind < -0.5f)
    return r * ((float)cols + over);
  return r * (float)cols;
}

/* Move a flake one step: slide it sideways, drop it down, advance its clock. */
static inline void integrate_flake_euler(Flake *f, float dt) {
  f->center_x += f->drift_vx * dt;
  f->y += f->vy * dt;
  f->age += dt;
}

/* Has the flake drifted well off either side of the screen? */
static inline bool flake_drifted_offscreen_x(const Flake *f, int cols) {
  return f->center_x < -8.0f || f->center_x > (float)(cols + 8);
}

/* ── §5 pile — snow build-up, with snow rolling into lower neighbours ── */

/*
 * pile_deposit() — drop one cell of snow at column `col`, but first check its
 * neighbours: if the column to the left or right is shorter, the snow rolls
 * there instead.
 *
 * That little roll is what makes the pile look real — dips fill in before
 * peaks grow. Without it, every flake would just stack straight up where it
 * landed, giving an unnatural picket-fence of columns.
 *
 * Does nothing if the chosen column has already hit its height cap.
 */
static void pile_deposit(int *pile, int cols, int col, int max_h) {
  if (col < 0 || col >= cols)
    return;

  int target = col;
  if (col > 0 && pile[col - 1] < pile[target])
    target = col - 1;
  if (col < cols - 1 && pile[col + 1] < pile[target])
    target = col + 1;

  if (pile[target] < max_h)
    pile[target] += 1;
}

/* Tallest any one column is allowed to get (a fraction of the screen height). */
static inline int compute_pile_height_cap(int rows) {
  int max_h = (int)((float)(rows - 2) * PILE_MAX_FRAC);
  return max_h < 1 ? 1 : max_h;
}

/* The screen row of the pile's top at a column — a flake lands when it reaches
 * this row or below. */
static inline float pile_floor_y(int pile_h, int rows) {
  return (float)(rows - 2 - pile_h);
}

/* A coin flip weighted by p: true p of the time, false the rest. */
static inline bool bernoulli_trial(uint32_t *rng, float p) {
  return lcg_unit(rng) < p;
}

/* ── §6 scene — the flake pool, the per-tick update, and drawing ── */

/*
 * Scene — all the state that changes while snow falls. It splits cleanly into
 * two parts: the simulation part (what the update step reads and writes) and a
 * single render-only field (which colour theme to draw with, never touched by
 * the physics). The Scene knows nothing about the terminal — the update writes
 * the flakes and pile, the drawing code reads them.
 */
typedef struct {
  /* ── the simulation part: the update step reads and writes these ── */

  /* When true, the update step does nothing, so snow freezes in place.
   * Drawing keeps going, so you see a still frame. Toggled by space. */
  bool paused;

  /* A whole-number speed multiplier on time. Default runs at real speed;
   * +/= doubles it, - halves it. It only stretches or squeezes how fast
   * the sim clock moves, not any of the physics numbers. */
  int speed;

  /* Which of the three presets (FLURRY / SNOWFALL / BLIZZARD) is active.
   * n / N cycle it. Switching doesn't rebuild anything — the flake count
   * drifts toward the new target over a few seconds, which looks natural. */
  Pattern current_pattern;

  /* Extra wind from the w / W keys, added on top of the preset's wind.
   * It sticks around when you switch presets, and resets on 'r'. */
  float wind_override;

  /* The random-number state for this scene. Seeded at startup and re-seeded
   * on 'r'. Every bit of randomness (spawn positions, speeds, wobble, sizes,
   * the does-it-stick coin flip) draws from here. */
  uint32_t rng;

  /* The terminal's current size, remembered here so the busy per-frame code
   * never has to ask ncurses. Refreshed at startup and whenever you resize. */
  int rows, cols;

  /* Seconds since the scene started. Currently unused by the motion (each
   * flake has its own age clock) but kept around for future effects. */
  float time_accum;

  /* The flakes. One fixed array, never resized while running. See Flake for
   * what each slot holds; spawning grabs the first inactive slot. */
  Flake flakes[MAX_FLAKES];

  /* The snow pile: pile[c] is how many cells of snow are stacked at column c,
   * counted up from just above the bottom HUD row. We only ever use the first
   * scene.cols entries. pile_deposit fills it (with the roll-into-shorter-
   * neighbour rule); each column tops out at the height cap and never melts,
   * so it only grows until you press 'c' to wipe it. */
  int pile[MAX_COLS];

  /* ── the render-only part: drawing reads this, the physics never does ── */

  /* Which colour theme to draw with. t / T cycle it. Purely cosmetic — the
   * snow behaves exactly the same whatever theme is picked. */
  int current_theme;
} Scene;

static int flake_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_FLAKES; i++)
    if (!s->flakes[i].active)
      return i;
  return -1;
}

/*
 * scene_spawn_flake — bring one new flake to life.
 *   y_min, y_max : the vertical band a flake may start in. Normal spawning
 *                  uses (-6, -1) so flakes enter just above the screen; the
 *                  startup fill spreads them across the whole height instead.
 *
 * Each value is picked by its own helper, so reading the body tells you the
 * flake's starting state: where it is, how it moves, how it wobbles, how big.
 */
static void scene_spawn_flake(Scene *s, float y_min, float y_max) {
  int idx = flake_pool_find_inactive(s);
  if (idx < 0)
    return;
  Flake *f = &s->flakes[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind_total = pp->wind_x + s->wind_override;

  f->center_x = sample_spawn_x_wind_extended(&s->rng, s->cols, wind_total);
  f->y = sample_uniform_in_range(&s->rng, y_min, y_max);
  f->vy = sample_terminal_velocity_jittered(&s->rng, pp->fall_speed);
  f->drift_vx = sample_drift_velocity_jittered(&s->rng, wind_total);
  f->sway_amp =
      sample_uniform_in_range(&s->rng, pp->sway_amp_min, pp->sway_amp_max);
  f->sway_freq =
      sample_uniform_in_range(&s->rng, pp->sway_freq_min, pp->sway_freq_max);
  f->sway_phase = sample_random_phase_2pi(&s->rng);
  f->age = 0.0f;
  f->size_idx = sample_size_class_small_biased(&s->rng);
  f->active = true;
}

static void scene_clear_flakes(Scene *s) {
  for (int i = 0; i < MAX_FLAKES; i++)
    s->flakes[i].active = false;
}

static void scene_clear_pile(Scene *s) { memset(s->pile, 0, sizeof s->pile); }

/*
 * scene_prewarm — fill the screen with flakes spread over the whole height, so
 * the very first frame already looks like snow that's been falling a while,
 * instead of one neat wave starting at the top.
 */
static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_flakes;
  if (target > MAX_FLAKES)
    target = MAX_FLAKES;

  int active = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      active++;

  float y_max = (float)(s->rows - 2);
  for (int k = active; k < target; k++)
    scene_spawn_flake(s, -6.0f, y_max);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_SNOWFALL;
  s->wind_override = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  scene_clear_flakes(s);
  scene_clear_pile(s);
  scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  /* Keep the pile as-is; if the window got narrower we just draw fewer columns. */
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->wind_override = 0.0f;
  scene_clear_flakes(s);
  scene_clear_pile(s);
  scene_prewarm(s);
}

/* Count how many flakes are currently in the air. */
static int count_active_flakes(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      n++;
  return n;
}

/* How many flakes to spawn this tick: enough to head toward the preset's
 * target, but limited per tick so resuming after a long pause doesn't dump a
 * sudden flood all at once. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt) {
  int target = pp->target_flakes;
  if (target > MAX_FLAKES)
    target = MAX_FLAKES;
  int spawn_cap = (int)((float)pp->target_flakes * dt * 4.0f) + 4;
  int n = target - active;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

/* Add n new flakes just above the top of the screen. */
static void flake_pool_topup_from_sky(Scene *s, int n) {
  for (int k = 0; k < n; k++)
    scene_spawn_flake(s, -6.0f, -1.0f);
}

/* Check whether a flake has reached the pile below it. If so, maybe add to the
 * pile (the preset's stick-chance decides, so flurries build slower) and retire
 * the flake. If its wobble briefly carried it off the side, we skip the check
 * and let it swing back next tick. */
static void try_pile_contact_and_deposit(Scene *s, Flake *f,
                                         const PatternParams *pp,
                                         int max_pile_h) {
  float fx = flake_x(f);
  int col = (int)(fx + 0.5f);
  if (col < 0 || col >= s->cols)
    return;

  float floor_y = pile_floor_y(s->pile[col], s->rows);
  if (f->y < floor_y)
    return;

  if (bernoulli_trial(&s->rng, pp->pile_growth_mul))
    pile_deposit(s->pile, s->cols, col, max_pile_h);
  f->active = false;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;
  s->time_accum += dt;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* 1. Add flakes until we're near the preset's target count. */
  int to_spawn = compute_spawn_count_for_tick(count_active_flakes(s), pp, dt);
  flake_pool_topup_from_sky(s, to_spawn);

  /* 2. Move every flake, then drop ones that left the side or hit the pile. */
  int max_pile_h = compute_pile_height_cap(s->rows);
  for (int i = 0; i < MAX_FLAKES; i++) {
    Flake *f = &s->flakes[i];
    if (!f->active)
      continue;

    integrate_flake_euler(f, dt);

    if (flake_drifted_offscreen_x(f, s->cols)) {
      f->active = false;
      continue;
    }

    try_pile_contact_and_deposit(s, f, pp, max_pile_h);
  }
}

/* Brightness for a colour slot: bold near the top, dim near the bottom. Both
 * the flakes and the pile use this so their gradients look consistent. */
static inline int ramp_attr_by_brightness_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* The character for a pile cell, by how deep it is: `*` on top, `#` just below,
 * `+` deeper down — gives a sense of depth using plain ASCII. */
static inline char pile_cell_glyph_by_depth(int depth_from_top) {
  if (depth_from_top == 0)
    return '*';
  if (depth_from_top < 3)
    return '#';
  return '+';
}

/* Draw one column of the pile, from the surface downward; the top cell gets the
 * brightest colour. */
static void pile_column_draw(int col, int height, int rows_eff) {
  for (int k = 0; k < height; k++) {
    int y = (rows_eff - 1) - k;
    if (y < 0)
      break;

    int ramp_slot = 7 - k;
    if (ramp_slot < 0)
      ramp_slot = 0;

    char glyph = pile_cell_glyph_by_depth(k);
    int attr = ramp_attr_by_brightness_slot(ramp_slot);
    int pair = PAIR_PILE_BASE + ramp_slot;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(y, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Draw the whole pile, column by column. Done before the flakes so falling
 * flakes sit cleanly on top. */
static void pile_draw_all_columns(const Scene *s, int rows_eff) {
  int cap = s->cols < MAX_COLS ? s->cols : MAX_COLS;
  for (int c = 0; c < cap; c++) {
    int h = s->pile[c];
    if (h > 0)
      pile_column_draw(c, h, rows_eff);
  }
}

/* Draw one flake at its current spot, choosing its character and colour from
 * its size. */
static void flake_draw(const Flake *f, int cols, int rows_eff) {
  int ix = (int)(flake_x(f) + 0.5f);
  int iy = (int)(f->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  char glyph = FLAKE_GLYPHS[f->size_idx];
  int pair = PAIR_FLAKE_BASE + f->size_idx;
  int attr = ramp_attr_by_brightness_slot(f->size_idx);

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Draw every flake that's in the air, on top of the pile. */
static void flakes_draw_all_active(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_FLAKES; i++) {
    const Flake *f = &s->flakes[i];
    if (f->active)
      flake_draw(f, s->cols, rows_eff);
  }
}

static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1;         /* keep the bottom row free for the HUD */
  pile_draw_all_columns(s, rows_eff); /* pile first, flakes drawn over it */
  flakes_draw_all_active(s, rows_eff);
}

/* ── §7 screen ── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Tally the numbers the HUD shows: flakes in the air and the tallest pile. */
static void scene_counts(const Scene *s, int *out_flakes, int *out_max_pile) {
  int n = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      n++;
  *out_flakes = n;

  int mp = 0;
  int cap = s->cols < MAX_COLS ? s->cols : MAX_COLS;
  for (int c = 0; c < cap; c++)
    if (s->pile[c] > mp)
      mp = s->pile[c];
  *out_max_pile = mp;
}

/*
 * screen_draw — draw the snow, then lay two info bars over it: a status line
 * across the top (preset or PAUSED, theme, counts, wind, fps, speed) and a key
 * list across the bottom. Both bars fill their whole row with colour and are
 * drawn last, so no flake shows through them.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int flakes, max_pile;
  scene_counts(s, &flakes, &max_pile);
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;

  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  /* ── top row: the live status line ── */
  char status[220];
  snprintf(status, sizeof status,
           " SNOW   %s   theme:%-8s   flakes:%4d  pile_h:%2d   "
           "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, flakes, max_pile,
           (double)wind, fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── bottom row: the list of keys ── */
  const char *hints =
      " q:quit  spc:pause  r:reseed  c:clear  n/p:pattern  t/T:theme  "
      "w/W:wind  +/-:speed  ]/[:Hz ";

  int hint_row = sc->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reseed(s);
    break;
  case 'c':
    scene_clear_pile(s);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
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

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    scene_prewarm(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_prewarm(s);
    break;

  case 'w':
    s->wind_override += WIND_STEP;
    break;
  case 'W':
    s->wind_override -= WIND_STEP;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
