/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * leaves.c — autumn leaves that tumble and flutter down the terminal.
 *
 * The whole trick: every leaf spins, and the way it leans is what steers it —
 * lean into the air and you glide that way, so leaves swoop in lazy S-curves
 * instead of dropping straight. Three "weather" presets (DRIFT/BREEZE/GUST)
 * run the same code with different numbers.
 *
 * Sister file: particle_systems/snow.c (same pool + preset + theme setup).
 * The physics ideas come from Tanabe & Kaneko 1994 (why falling paper flutters
 * and tumbles) and Reeves 1983 (particle systems driven by a small preset table).
 *
 * Keys:  q quit  space pause  r reseed  n/p preset  t/T theme  w/W wind
 *        +/- speed  ]/[ sim-Hz
 * Build: gcc -std=c11 -O2 -Wall -Wextra particle_systems/leaves.c -o leaves -lncurses -lm
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

/* ── §1 CONFIG — the tweakable numbers, the weather presets, the colour themes ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_LEAVES = 700,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* ncurses numbers each colour pair. 1 and 2 are the HUD's — same in every
   * demo in this project, so the HUD always looks alike. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_LEAF_BASE = 3, /* leaf colours 0..7: faint (small) up to vivid (big) */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* A little per-leaf randomness so they don't all move in lockstep. */
#define LEAF_SPEED_VARIANCE 0.50f /* fall speed varies by ±25% per leaf */
#define LEAF_WIND_JITTER 1.0f     /* wind varies by ±0.5 cells/sec per leaf */

/* How much one press of w / W shifts the wind. */
#define WIND_STEP 3.0f

/* Leaves in the bottom few rows are drawn dim, so they fade into shadow near
 * the ground instead of blinking out when they get recycled. */
#define FADE_ROWS 3

/* How far past an edge a leaf may wander before we recycle it (cells). Stops a
 * leaf that briefly swings offscreen from flickering at the border. */
#define OFFSCREEN_MARGIN 8.0f

/* New leaves are born this many rows above the top edge; it's also the extra
 * distance a leaf falls before it drops off the bottom. */
#define SPAWN_ABOVE_ROWS 4.0f

/* How new leaves get added (see leaf_pool_spawn_count):
 *   ACCUM_CAP    — biggest "leftover" spawn we'll save up, so a quiet spell
 *                  can't suddenly dump a pile of leaves at once.
 *   BURST_FACTOR — most we'll add in one step, as a multiple of the normal
 *                  top-up rate.
 *   BURST_FLOOR  — but always allow at least this many per step. */
#define SPAWN_ACCUM_CAP 2.0f
#define SPAWN_BURST_FACTOR 4.0f
#define SPAWN_BURST_FLOOR 4

/* Frame timing (main loop):
 *   MAX_CATCHUP_MS — if a frame took longer than this (say the program sat in
 *                    a debugger), pretend only this much time passed, so the
 *                    sim doesn't try to replay a huge jump all at once.
 *   FRAME_CAP_FPS  — how many frames a second we draw, separate from the sim's
 *                    own Hz setting. */
#define MAX_CATCHUP_MS 100
#define FRAME_CAP_FPS 60

/* Which of the three presets the player has picked. It's just an index into
 * the weather table below (see pattern_to_weather). */
typedef enum {
  PATTERN_DRIFT = 0,
  PATTERN_BREEZE = 1,
  PATTERN_GUST = 2,
  N_PATTERNS = 3,
} Pattern;

/*
 * Weather — the numbers describing one wind condition: how leaves fall in it.
 * The engine never asks "which preset is this?"; it just reads a Weather and
 * runs. Same code, three very different falls (the small-table idea, Reeves 1983).
 *
 *   target_leaves  : how many leaves to keep in the air — more means a denser sky.
 *   fall_speed     : how fast a leaf drops, in cells/sec (roughly constant, like
 *                    a real leaf that reaches a steady speed almost at once).
 *   wind_x         : steady sideways push in cells/sec (+ blows right); w/W add more.
 *   tumble_frac    : share of leaves (0..1) that TUMBLE — spin all the way over —
 *                    rather than FLUTTER (just rock side to side). Gusty = more tumble.
 *   spin_min,      : how fast a leaf turns (radians/sec). For a tumbler this is its
 *   spin_max         steady spin; for a flutterer it's how far it rocks. Picked per leaf.
 *   flutter_freq_min, : how many times a second a flutterer rocks back and forth.
 *   flutter_freq_max    Only flutterers use it.
 *   lift_gain      : how strongly a leaf's lean steers it sideways (cells/sec). This
 *                    is what turns spin into swooping. Set it to 0 and leaves fall straight.
 */
typedef struct {
  int target_leaves;
  float fall_speed;
  float wind_x;
  float tumble_frac;
  float spin_min, spin_max;
  float flutter_freq_min, flutter_freq_max;
  float lift_gain;
} Weather;

static const Weather weathers[N_PATTERNS] = {
    /* DRIFT  */ {120, 6.0f, 1.0f, 0.25f, 0.8f, 2.0f, 0.5f, 1.2f, 3.0f},
    /* BREEZE */ {300, 10.0f, 4.0f, 0.45f, 1.5f, 3.5f, 0.7f, 1.6f, 5.0f},
    /* GUST   */ {550, 16.0f, 10.0f, 0.70f, 3.0f, 6.0f, 1.0f, 2.0f, 7.0f},
};

/*
 * Theme — one autumn colour scheme. leaf[8] runs from faint (small, dry leaves)
 * to vivid (big, fresh-fallen ones). Every colour sits in the bright half of the
 * palette, so even the darkest one stays visible against a black terminal.
 */
typedef struct {
  const char *name;
  short leaf[8]; /* faded to vivid */
  short sky;
} Theme;

#define N_THEMES 8

static const Theme themes[N_THEMES] = {
    /* name, leaf colours (faded to vivid), sky */
    {"AUTUMN", {130, 166, 172, 178, 208, 214, 220, 226}, 234},
    {"MAPLE", {88, 124, 130, 166, 196, 202, 208, 214}, 233},
    {"GOLDEN", {136, 142, 172, 178, 214, 220, 226, 229}, 234},
    {"EMBER", {88, 124, 130, 166, 202, 208, 214, 220}, 233},
    {"COPPER", {94, 130, 136, 172, 173, 179, 215, 222}, 234},
    {"RUST", {88, 124, 130, 166, 172, 208, 214, 220}, 233},
    {"FOREST", {64, 100, 106, 142, 148, 184, 214, 220}, 234},
    {"PARCHMENT", {94, 101, 137, 143, 179, 180, 222, 223}, 233},
};

/* The four characters a leaf is drawn as, one per 45° of turn. As it spins it
 * steps `-` `/` `|` `\` and back — and that's what makes the spin visible. */
static const char LEAF_GLYPHS[4] = {'-', '/', '|', '\\'};

/* ── §2 STATE — the data types everything else works on ── */

/*
 * Leaf — one falling leaf. On top of the usual position and speed, it carries a
 * tilt angle and the recipe for how that angle changes. The angle is the star:
 * it both picks the character we draw and, through lift, steers the leaf.
 *
 *   x, y       : where it is, in cells (float). y grows downward. x is the real
 *                drawn column — the sideways glide is folded straight into it.
 *   vy         : fall speed in cells/sec, fixed for this leaf's life (with a small
 *                random offset so the field doesn't drop in perfect unison).
 *   angle      : which way it's tilted, in radians. Each step we nudge it by the
 *                spin. This one number picks the glyph AND steers the leaf sideways.
 *   tumble     : true  → spins steadily all the way over (uses spin_rate).
 *                false → flutters: rocks back and forth (uses the amp/freq/phase).
 *   spin_rate  : tumblers only — steady turn speed (radians/sec; sign = which way).
 *   flutter_amp,  : flutterers only — the rocking turn speed is
 *   flutter_freq,   amp·cos(freq·age + phase): amp = how far it rocks, freq = how
 *   flutter_phase   fast, phase = a random offset so leaves don't rock in sync.
 *   wind_jitter: a small fixed sideways nudge (cells/sec) so leaves in the same
 *                wind don't drift as one flat sheet.
 *   age        : seconds alive — the clock that drives a flutterer's rocking.
 *   size_idx   : size bucket 0..7 (bigger = brighter). Just for looks.
 *   active     : is this slot in use? Empty slots are skipped everywhere.
 *
 * The tumble/flutter split is the real behaviour of falling flat objects
 * (Tanabe & Kaneko 1994): which one you get depends on how fast it's turning.
 */
typedef struct {
  float x, y;
  float vy;
  float angle;
  bool tumble;
  float spin_rate;
  float flutter_amp;
  float flutter_freq;
  float flutter_phase;
  float wind_jitter;
  float age;
  int size_idx;
  bool active;
} Leaf;

/*
 * LeafPool — one big fixed array holding every leaf, plus the bits used to keep
 * spawning new ones. We never allocate memory while running: when a leaf falls
 * off, its slot just goes free (active = false) and the next new leaf reuses it.
 * Reusing a fixed pool like this is the standard particle-systems trick.
 *
 *   leaves      : the slots. Spawning grabs the first free one.
 *   rng         : this pool's own random state — every new leaf's position,
 *                 speed, spin, and size is drawn from it.
 *   spawn_accum : saves up the fractional part of the spawn rate, so a calm
 *                 preset (well under one new leaf per step) still adds leaves.
 */
typedef struct {
  Leaf leaves[MAX_LEAVES];
  uint32_t rng;
  float spawn_accum;
} LeafPool;

/*
 * Scene — the whole little world, laid out like a table of contents:
 *   WHAT  is falling  — the pool of leaves.
 *   HOW   you drive it — the knobs the keyboard changes, plus run/pause.
 *   WHERE we are      — the play-area size and a running clock.
 *   plus one look-only knob (the theme) the physics never reads.
 * Only the big coordinating functions (init, reset, prewarm, tick) get the whole
 * Scene; everything else is handed just the piece it needs.
 */
typedef struct {
  /* WHAT — the leaves */
  LeafPool pool;

  /* HOW — the knobs the keyboard drives */
  Pattern current_pattern; /* which weather preset is active  (n/p)   */
  int speed;               /* whole-number time multiplier    (+/-)   */
  float wind_override;     /* extra wind on top of the preset (w/W)   */
  bool paused;             /* freeze the update               (space) */

  /* WHERE — the play area and a running clock */
  int rows, cols;   /* terminal size the sim reads for its edges */
  float time_accum; /* seconds elapsed */

  /* look-only — the physics never reads this */
  int current_theme; /* colour scheme (t/T) */
} Scene;

/* The terminal's current width and height, refreshed on every resize. */
typedef struct {
  int cols, rows;
} Screen;

/*
 * App — everything the program owns: the world, the terminal size, and the
 * flags the signal handlers flip to ask the main loop to quit or resize.
 */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                       /* simulation steps per second (]/[ ) */
  volatile sig_atomic_t running;     /* a signal handler sets this to 0 to quit */
  volatile sig_atomic_t need_resize; /* SIGWINCH sets this; handled next frame */
} App;

/* ── §3 PERFORMANCE — a steady stopwatch (the frame loop that uses it is in §7) ── */

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

/* ── §4 LOGIC — little helpers that just work things out; they change nothing ── */

/* The one spot that reads the preset table — look up a preset's numbers. */
static const Weather *pattern_to_weather(Pattern p) { return &weathers[p]; }

static const char *pattern_to_name(Pattern p) {
  switch (p) {
  case PATTERN_DRIFT:
    return "DRIFT ";
  case PATTERN_BREEZE:
    return "BREEZE";
  case PATTERN_GUST:
    return "GUST  ";
  default:
    return "?     ";
  }
}

/*
 * How fast this leaf is turning right now (radians/sec). A tumbler spins at a
 * fixed rate. A flutterer's turn speed swings back and forth (a cosine), which
 * makes it rock around upright instead of flipping over — that's the whole
 * tumble-vs-flutter difference.
 */
static inline float leaf_angular_velocity(const Leaf *f) {
  if (f->tumble)
    return f->spin_rate;
  return f->flutter_amp * cosf(f->flutter_freq * f->age + f->flutter_phase);
}

/* Has the leaf left the play area — off the bottom, or well past either side? */
static inline bool leaf_offscreen(const Leaf *f, int cols, int rows) {
  return f->y > (float)rows || f->x < -OFFSCREEN_MARGIN ||
         f->x > (float)cols + OFFSCREEN_MARGIN;
}

/*
 * Pick the character for how the leaf is tilted right now. A line looks the same
 * upside-down, so we only care about half a turn; we split that half-turn into
 * four wedges and pick `-` `/` `|` `\`. (The + M_PI fixes fmodf occasionally
 * handing back a tiny negative, which would land outside the array.)
 */
static inline char angle_to_glyph(float angle) {
  float a = fmodf(angle, (float)M_PI);
  if (a < 0.0f)
    a += (float)M_PI;
  int bucket = (int)(a / ((float)M_PI / 4.0f));
  if (bucket > 3)
    bucket = 3;
  return LEAF_GLYPHS[bucket];
}

/* Bigger leaves are drawn bold and bright, tiny ones dim, the rest normal. */
static inline int size_to_attr(int size_idx) {
  if (size_idx >= 6)
    return A_BOLD;
  if (size_idx <= 1)
    return A_DIM;
  return A_NORMAL;
}

static int leaf_pool_count_active(const LeafPool *pool) {
  int n = 0;
  for (int i = 0; i < MAX_LEAVES; i++)
    if (pool->leaves[i].active)
      n++;
  return n;
}

/* ── §5 SIMULATION — the actual falling: this is what moves the leaves ── */

/* A cheap random-number generator. Its state lives in the LeafPool rather than
 * a global, so each pool has its own independent randomness. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24); /* a number in [0, 1) */
}

static inline float sample_uniform_in_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}

/* One leaf's fall speed: the preset's speed nudged a bit, so they don't all
 * fall at exactly the same rate. */
static inline float sample_terminal_velocity_jittered(uint32_t *rng,
                                                       float base_vy) {
  float scale = (1.0f - LEAF_SPEED_VARIANCE * 0.5f) +
                lcg_unit(rng) * LEAF_SPEED_VARIANCE;
  return base_vy * scale;
}

/* A random angle (0 up to a full turn), for a leaf's starting tilt and its
 * flutter timing, so no two leaves are in sync. */
static inline float sample_random_phase_2pi(uint32_t *rng) {
  return lcg_unit(rng) * 2.0f * (float)M_PI;
}

/* A random spin speed between lo and hi, then a coin-flip for direction — so a
 * tumbler is just as likely to spin one way as the other. */
static inline float sample_spin_signed(uint32_t *rng, float lo, float hi) {
  float mag = sample_uniform_in_range(rng, lo, hi);
  return (lcg_unit(rng) < 0.5f) ? -mag : mag;
}

/* Pick a size bucket, tilted so most leaves are small and big ones are rare
 * (squaring a 0..1 number bunches it toward the low end). */
static inline int sample_size_class_small_biased(uint32_t *rng) {
  float r = lcg_unit(rng);
  int idx = (int)(r * r * 7.999f);
  if (idx < 0)
    idx = 0;
  if (idx > 7)
    idx = 7;
  return idx;
}

/* Pick where a new leaf comes in across the top. When there's wind we let it
 * start a bit past the upwind edge, so leaves blowing across keep the whole
 * width filled instead of thinning out on the side they blow in from. */
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

static int leaf_pool_find_inactive(LeafPool *pool) {
  for (int i = 0; i < MAX_LEAVES; i++)
    if (!pool->leaves[i].active)
      return i;
  return -1;
}

/*
 * Bring one new leaf to life somewhere in the vertical band y_min..y_max. Normal
 * top-ups drop it just above the screen; the startup fill spreads leaves down the
 * whole height, so it doesn't open with one tidy wave. `wind` is the total wind
 * (preset + the player's extra) already worked out by the caller.
 */
static void leaf_pool_spawn(LeafPool *pool, const Weather *w, int cols,
                            float wind, float y_min, float y_max) {
  int idx = leaf_pool_find_inactive(pool);
  if (idx < 0)
    return;
  Leaf *f = &pool->leaves[idx];

  /* where it enters, how fast it drops, which way it starts facing */
  f->x = sample_spawn_x_wind_extended(&pool->rng, cols, wind);
  f->y = sample_uniform_in_range(&pool->rng, y_min, y_max);
  f->vy = sample_terminal_velocity_jittered(&pool->rng, w->fall_speed);
  f->angle = sample_random_phase_2pi(&pool->rng);

  /* its spin style — a steady tumble, or a rocking flutter */
  f->tumble = lcg_unit(&pool->rng) < w->tumble_frac;
  f->spin_rate = sample_spin_signed(&pool->rng, w->spin_min, w->spin_max);
  f->flutter_amp = sample_uniform_in_range(&pool->rng, w->spin_min, w->spin_max);
  f->flutter_freq =
      sample_uniform_in_range(&pool->rng, w->flutter_freq_min, w->flutter_freq_max);
  f->flutter_phase = sample_random_phase_2pi(&pool->rng);

  /* its little sideways nudge, and looks */
  f->wind_jitter = (lcg_unit(&pool->rng) - 0.5f) * 2.0f * LEAF_WIND_JITTER;
  f->age = 0.0f;
  f->size_idx = sample_size_class_small_biased(&pool->rng);
  f->active = true;
}

static void leaf_pool_clear(LeafPool *pool) {
  for (int i = 0; i < MAX_LEAVES; i++)
    pool->leaves[i].active = false;
}

/*
 * Fill the pool up to the preset's target, spreading the new leaves down the
 * whole height — so the very first frame already looks like leaves that have
 * been falling a while, not one neat wave starting at the top.
 */
static void leaf_pool_prewarm(LeafPool *pool, const Weather *w, int cols,
                              int rows, float wind) {
  int target = w->target_leaves;
  if (target > MAX_LEAVES)
    target = MAX_LEAVES;

  int active = leaf_pool_count_active(pool);
  float y_max = (float)(rows - 2);
  for (int k = active; k < target; k++)
    leaf_pool_spawn(pool, w, cols, wind, -SPAWN_ABOVE_ROWS, y_max);
}

/*
 * How many new leaves to add this step.
 *
 * At or below the target, just add back however many we're short, so the count
 * settles right at the target.
 *
 * ABOVE the target (you just switched from a busy preset to a calm one) the
 * naive "add the shortfall" gives zero — so nothing new appears at the top while
 * the extra leaves rain out the bottom, leaving an ugly empty gap that creeps
 * down the screen. Instead we keep trickling leaves in at the top at the rate
 * they're leaving. The crowd still thins to the target (more leave than arrive),
 * but the top never goes bare and no leaf is ever yanked out of mid-air.
 *
 * That trickle is usually less than one leaf per step, so we save up the
 * fraction in spawn_accum and spend it once it adds up to a whole leaf.
 */
static int leaf_pool_spawn_count(LeafPool *pool, const Weather *w, int rows,
                                 int active, float dt) {
  int target = w->target_leaves;
  if (target > MAX_LEAVES)
    target = MAX_LEAVES;

  /* save up the trickle that replaces the leaves falling off the bottom */
  float travel = (float)rows + SPAWN_ABOVE_ROWS; /* full fall distance */
  float steady_inflow = (float)target * w->fall_speed / travel * dt;
  pool->spawn_accum += steady_inflow;
  if (pool->spawn_accum > SPAWN_ACCUM_CAP)
    pool->spawn_accum = SPAWN_ACCUM_CAP;
  int steady = (int)pool->spawn_accum;
  pool->spawn_accum -= (float)steady;

  /* below target, fill the gap fast; above it, feed only the trickle */
  int n = (active > target) ? steady : (target - active);

  int spawn_cap =
      (int)((float)target * dt * SPAWN_BURST_FACTOR) + SPAWN_BURST_FLOOR;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

static void leaf_pool_topup(LeafPool *pool, const Weather *w, int cols,
                            float wind, int n) {
  for (int k = 0; k < n; k++)
    leaf_pool_spawn(pool, w, cols, wind, -SPAWN_ABOVE_ROWS, -1.0f);
}

/* The spawn half of one step: work out how many new leaves to add, and add them
 * at the top. (The how-many decision lives in leaf_pool_spawn_count.) */
static void leaf_pool_replenish(LeafPool *pool, const Weather *w, int cols,
                                int rows, float wind, float dt) {
  int active = leaf_pool_count_active(pool);
  int to_spawn = leaf_pool_spawn_count(pool, w, rows, active, dt);
  leaf_pool_topup(pool, w, cols, wind, to_spawn);
}

/*
 * Move one leaf forward by dt. The interesting line is the sideways glide,
 * lift_gain · sin(angle): a leaf edge-on to its fall (tilt near flat or straight
 * up) catches no air and drifts nowhere, while one turned side-on catches the
 * most. As it spins, that push flips direction — and that carves the S-curves.
 */
static inline void integrate_leaf(Leaf *f, float wind, float lift_gain,
                                  float dt) {
  float omega = leaf_angular_velocity(f);
  f->angle += omega * dt;

  float glide = lift_gain * sinf(f->angle);
  f->x += (wind + f->wind_jitter + glide) * dt;
  f->y += f->vy * dt;
  f->age += dt;
}

/* Refill the pool for whichever preset is active right now — used at startup and
 * each time the player switches presets. Just looks up the current weather and
 * wind, then hands off to leaf_pool_prewarm. */
static void scene_prewarm(Scene *s) {
  const Weather *w = pattern_to_weather(s->current_pattern);
  leaf_pool_prewarm(&s->pool, w, s->cols, s->rows, w->wind_x + s->wind_override);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_BREEZE;
  s->wind_override = 0.0f;
  s->pool.rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  leaf_pool_clear(&s->pool);
  scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

/* Fresh start ('r' key): new random seed, wind reset, pool refilled. */
static void scene_reseed(Scene *s) {
  s->pool.rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->wind_override = 0.0f;
  leaf_pool_clear(&s->pool);
  scene_prewarm(s);
}

/* The moving half of one step: nudge every live leaf forward, and free the slot
 * of any that has drifted off-screen so a new leaf can reuse it. */
static void leaf_pool_advance(LeafPool *pool, float wind, float lift, int cols,
                              int rows, float dt) {
  for (int i = 0; i < MAX_LEAVES; i++) {
    Leaf *f = &pool->leaves[i];
    if (!f->active)
      continue;
    integrate_leaf(f, wind, lift, dt);
    if (leaf_offscreen(f, cols, rows))
      f->active = false;
  }
}

/* One step of the whole simulation — the only place the leaves actually change.
 * Skip if paused, apply the speed setting, grab the current weather, then add
 * new leaves and move the ones we have. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF; /* apply the speed knob */
  s->time_accum += dt;

  const Weather *w = pattern_to_weather(s->current_pattern);
  float wind = w->wind_x + s->wind_override;

  leaf_pool_replenish(&s->pool, w, s->cols, s->rows, wind, dt);
  leaf_pool_advance(&s->pool, wind, w->lift_gain, s->cols, s->rows, dt);
}

/* ── §6 RENDER — turn the leaves into characters; only reads, never changes them ── */

/* Load one theme's colours into ncurses. Falls back to plain yellow if the
 * terminal only has the basic 8 colours. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_LEAF_BASE + i), t->leaf[i], -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_LEAF_BASE + i), COLOR_YELLOW, -1);
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

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* stop ncurses peeking at input mid-draw — that causes tearing */
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
/* The endwin()/refresh() two-step is how you make ncurses notice a new size. */
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Draw one leaf — character from its tilt, colour from its size, and dimmed in
 * the bottom few rows so it fades into shadow near the ground. */
static void leaf_draw(const Leaf *f, int cols, int rows_eff) {
  int ix = (int)(f->x + 0.5f);
  int iy = (int)(f->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  char glyph = angle_to_glyph(f->angle);
  int pair = PAIR_LEAF_BASE + f->size_idx;
  int attr = size_to_attr(f->size_idx);
  if (iy >= rows_eff - FADE_ROWS)
    attr = A_DIM;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Draw every leaf that's in the air. rows_eff is the height minus the HUD row. */
static void leaf_pool_draw(const LeafPool *pool, int cols, int rows_eff) {
  for (int i = 0; i < MAX_LEAVES; i++) {
    const Leaf *f = &pool->leaves[i];
    if (f->active)
      leaf_draw(f, cols, rows_eff);
  }
}

/* Paint one full-width HUD bar: flood the whole row with the pair's colour, then
 * lay the text over it, so no leaf shows through the bar. */
static void draw_status_bar(int row, int cols, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  for (int x = 0; x < cols; x++)
    mvaddch(row, x, ' ');
  mvprintw(row, 0, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the leaves, then two info bars on top: a status line along the top and
 * the key list along the bottom. The bars go last so no leaf pokes through. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  leaf_pool_draw(&s->pool, s->cols, s->rows - 1); /* bottom row kept for HUD */

  int leaves = leaf_pool_count_active(&s->pool);
  const Weather *w = pattern_to_weather(s->current_pattern);
  float wind = w->wind_x + s->wind_override;
  const char *state_str =
      s->paused ? "PAUSED" : pattern_to_name(s->current_pattern);

  char status[220];
  snprintf(status, sizeof status,
           " LEAVES  %s   theme:%-9s   leaves:%4d   wind:%+5.1f c/s   "
           "%5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, leaves, (double)wind, fps,
           sim_fps, s->speed);
  const char *hints =
      " q:quit  spc:pause  r:reseed  n/p:preset  t/T:theme  "
      "w/W:wind  +/-:speed  ]/[:Hz ";

  draw_status_bar(0, sc->cols, PAIR_HUD, status);            /* top: live status */
  draw_status_bar(sc->rows - 1, sc->cols, PAIR_HINT, hints); /* bottom: keys */
}

/* Push our drawing to the terminal in one write, which avoids flicker. */
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 APP — keypresses, resizing, and main(): where it all comes together ── */

static App g_app;

/* A signal handler can safely do almost nothing, so these just flip a flag and
 * let the main loop act on it next time around. */
static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
/* Runs at exit (via atexit) so the terminal is restored even if we crash out. */
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* Handle one keypress. Returns false only when the user asked to quit. */
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

/* How long since the last frame, in nanoseconds, and set frame_time to now. If
 * it was a really long gap (paused in a debugger, laptop asleep), we cap it —
 * otherwise the sim would try to catch that whole gap up at once and lock up. */
static int64_t frame_elapsed_ns(int64_t *frame_time) {
  int64_t now = clock_ns();
  int64_t dt = now - *frame_time;
  *frame_time = now;
  if (dt > MAX_CATCHUP_MS * NS_PER_MS)
    dt = MAX_CATCHUP_MS * NS_PER_MS;
  return dt;
}

/* Run one fixed-size sim step for each whole step's worth of time that's built
 * up. A fixed step size means the physics behaves the same at any frame rate. */
static void run_due_ticks(Scene *s, int64_t *sim_accum, int64_t tick_ns,
                          float dt_sec) {
  while (*sim_accum >= tick_ns) {
    scene_tick(s, dt_sec);
    *sim_accum -= tick_ns;
  }
}

/* Sleep out the rest of the frame so every frame takes about the same time —
 * a steady frame rate no matter how little work this one needed. */
static void cap_frame_rate(int64_t frame_start, int64_t dt) {
  int64_t elapsed = clock_ns() - frame_start + dt;
  clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);
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

    /* measure elapsed time (clamped), then run every sim step it buys */
    int64_t dt = frame_elapsed_ns(&frame_time);
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;
    sim_accum += dt;
    run_due_ticks(&app->scene, &sim_accum, tick_ns, dt_sec);

    /* refresh the on-screen fps reading about twice a second */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    cap_frame_rate(frame_time, dt);

    /* draw the frame, then handle one keypress */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
