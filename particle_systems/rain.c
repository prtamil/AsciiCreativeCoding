/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rain.c — falling rain with streaky trails and ground splashes.
 *
 * Drops fall across the screen (you can blow them sideways with wind),
 * each drawn as a short streak that fades from a bright head to a faint
 * tail. When a drop hits the bottom it vanishes and tosses up a few
 * little splash bits that arc up and fall back. Four presets go from a
 * light DRIZZLE up to a full MONSOON.
 *
 * Sister files: brust.c reuses the same particle-pool idea; the splash
 * arc borrows the gravity step from physics/bounce_ball.c.
 * Rain optics (why streaks look the way they do): Garg & Nayar, "Vision
 * and Rain", IJCV 75(1), 2007.
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

  MAX_DROPS = 1024,
  MAX_SPLASHES = 800,
  SPLASH_BASE_PER_DROP = 4,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slots. The HUD bars always use the first two. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAIN_BASE = 3, /* eight pairs in a row: faint tail up to bright head */
  PAIR_SPLASH = 11,
  PAIR_SKY = 12,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Splash physics. Speeds are in cells/sec, gravity in cells/sec/sec. */
#define SPLASH_GRAVITY 200.0f
#define SPLASH_KICK_UP 35.0f /* how hard a splash bit jumps upward at birth */
#define SPLASH_KICK_X 25.0f  /* how far sideways it can fan, either direction */
#define SPLASH_DRAG 2.0f     /* how fast sideways motion bleeds off per second */
#define SPLASH_LIFE_MIN 0.30f
#define SPLASH_LIFE_MAX 0.65f

/* Gap between glyphs along a drop's trail, in cells. */
#define TRAIL_SPACING_MIN 0.55f
#define TRAIL_SPACING_MAX 0.85f

/* Each drop gets slightly randomized speed and wind so they don't all
 * fall in perfect lockstep (which looks like a sheet sliding down rather
 * than separate drops). SPEED_VARIANCE is the fraction of wiggle on fall
 * speed; WIND_JITTER is the extra cells/sec of sideways nudge. */
#define DROP_SPEED_VARIANCE 0.40f
#define DROP_WIND_JITTER 2.0f

/* How much one w/W keypress nudges the wind, in cells/sec. */
#define WIND_STEP 5.0f

/* The four rain presets, light to heavy. */
typedef enum {
  PATTERN_DRIZZLE = 0,
  PATTERN_SHOWER = 1,
  PATTERN_STORM = 2,
  PATTERN_MONSOON = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_DRIZZLE:
    return "DRIZZLE";
  case PATTERN_SHOWER:
    return "SHOWER ";
  case PATTERN_STORM:
    return "STORM  ";
  case PATTERN_MONSOON:
    return "MONSOON";
  default:
    return "?      ";
  }
}

/*
 * PatternParams — the dials that make one rain preset feel different
 * from another. There is only one simulation; it just reads these
 * numbers and behaves accordingly, so the same code gives you a gentle
 * drizzle or a wall of monsoon. The n/N keys pick which row to read.
 *
 *   target_drops  : how many drops to keep on screen at once. The spawn
 *                   loop tops up toward this number every tick. Bigger =
 *                   thicker rain. DRIZZLE 150, MONSOON 800.
 *
 *   drop_speed    : how fast drops fall, in cells/sec. They fall at a
 *                   steady speed rather than speeding up, because real
 *                   raindrops hit their top speed almost instantly
 *                   (Garg & Nayar). 35 for drizzle up to 190 for monsoon.
 *
 *   wind_x        : sideways speed at birth, in cells/sec. Negative blows
 *                   left, positive blows right. Together with drop_speed
 *                   it sets how slanted each streak is, which decides
 *                   whether a drop draws as '|', '/', '\', or '~'. The
 *                   player's w/W keys add more wind on top of this.
 *
 *   length_min,   : how long a drop's streak is, in cells. Each drop
 *   length_max      picks a random length between these two at birth.
 *                   Longer = more stretched-out streaks. DRIZZLE is
 *                   short (1.0-2.0), MONSOON long (4.0-6.5).
 *
 *   splash_mul    : how many splash bits each ground hit throws up,
 *                   as a multiple of SPLASH_BASE_PER_DROP. DRIZZLE 0.4
 *                   (a few scattered specks), MONSOON 1.3 (a busy line
 *                   of splashes along the floor).
 */
typedef struct {
  int target_drops;
  float drop_speed;
  float wind_x;
  float length_min;
  float length_max;
  float splash_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* DRIZZLE  */ {150, 35.0f, 3.0f, 1.0f, 2.0f, 0.40f},
    /* SHOWER   */ {240, 75.0f, 12.0f, 2.0f, 3.5f, 0.70f},
    /* STORM    */ {480, 130.0f, 35.0f, 3.0f, 5.0f, 1.00f},
    /* MONSOON  */ {800, 190.0f, 60.0f, 4.0f, 6.5f, 1.30f},
};

/*
 * Theme — one colour scheme for the rain. The ramp is eight colours
 * running from the faint tail of a streak (ramp[0]) up to its bright
 * head (ramp[7]); splash bits and the sky get their own colour. All
 * colours are kept in the brighter half of the palette so even the
 * dimmest stays visible on a dark terminal.
 */
typedef struct {
  const char *name;
  short ramp[8]; /* faint tail to bright head */
  short splash;
  short sky;
} Theme;

#define N_THEMES 11

static const Theme themes[N_THEMES] = {
    /* name         ramp colours (faint tail to bright head)       splash sky */
    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 190}, 154, 234},
    {"FIRE", {88, 124, 130, 166, 196, 208, 214, 226}, 226, 233},
    {"OCEANIC", {24, 25, 31, 38, 44, 51, 87, 159}, 159, 234},
    {"NEON", {53, 91, 134, 165, 201, 207, 213, 219}, 219, 234},
    {"MONO", {240, 243, 245, 247, 249, 251, 253, 255}, 255, 232},
    {"ICE", {24, 31, 67, 110, 117, 153, 195, 231}, 231, 235},
    {"NOVA", {24, 75, 117, 159, 195, 219, 226, 231}, 231, 234},
    {"FOREST", {28, 64, 70, 76, 112, 148, 184, 220}, 184, 234},
    {"DESERT", {94, 130, 137, 143, 179, 215, 222, 229}, 222, 234},
    {"ECLIPSE", {52, 88, 95, 131, 167, 173, 209, 215}, 209, 232},
    {"TROPICAL", {29, 35, 37, 44, 50, 86, 122, 159}, 158, 234},
};

/* The trail characters, going from faintest tail to densest near-head. */
static const char RAMP_GLYPHS[8] = {'`', '.', ',', ':', ';', '-', '+', '*'};

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
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAIN_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_SPLASH, t->splash, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    static const short fb[8] = {
        COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,
        COLOR_CYAN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAIN_BASE + i), fb[i], -1);
    init_pair(PAIR_SPLASH, COLOR_CYAN, -1);
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

/* ── §4 drop — one falling rain particle ── */

/*
 * Drop — one falling raindrop, with everything the simulation and the
 * drawing code need packed into one flat record. Drops live in a fixed
 * array (no memory is allocated while running), so a "dead" drop is just
 * a free slot waiting to be reused.
 *
 * A drop is born just above the top of the screen at a random column,
 * falls at a steady slanted speed each tick, and when it reaches the
 * bottom it dies and throws up a few splash bits (scene_emit_splashes).
 * The next spawn grabs the first free slot it finds.
 *
 *   x, y    : where the drop is, in cells, kept as floats. Keeping a
 *             fraction of a cell is what makes the streak glide smoothly
 *             instead of jumping a whole cell at a time; we round to a
 *             whole cell only when actually drawing.
 *
 *   vx, vy  : how fast the drop moves, in cells/sec. vx is sideways
 *             (negative = left, positive = right); vy is downward and
 *             always positive (no gravity — it falls at a steady speed).
 *             This pair also decides how slanted the streak looks, and
 *             so which character draws the head: '|', '/', '\', or '~'.
 *
 *   length  : how many cells long the streak is, picked at birth from
 *             the preset's range. Longer means a more stretched streak.
 *
 *   spacing : the gap between trail characters, in cells. Picked per
 *             drop so even a slow drizzle drop still shows a visible
 *             trail instead of all its trail piling onto the head.
 *
 *   active  : true if this slot holds a live drop. Dead slots are
 *             skipped, and spawning just scans for the first false one.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float length;
  float spacing;
  bool active;
} Drop;

/* A tiny, fast random-number generator. Each scene keeps its own state
 * so nothing shares a global, which keeps the randomness predictable. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24); /* a value in [0, 1) */
}

/*
 * Pick the character for the head of a streak based on how slanted the
 * drop is moving: nearly straight down draws '|', a gentle diagonal
 * draws '/' or '\' depending on the wind direction, and a very shallow,
 * almost-sideways drop draws '~'.
 */
static char drop_glyph_for_slope(float vx, float vy) {
  float ax = fabsf(vx);
  float ay = fabsf(vy);
  if (ax < 1e-3f)
    return '|';
  float slope = ay / ax;
  if (slope > 4.0f)
    return '|';
  if (slope < 0.5f)
    return '~';
  return (vx > 0.0f) ? '\\' : '/';
}

/* ── §5 splash — small particles emitted on drop impact ── */

/*
 * Splash — a tiny short-lived bit thrown up when a drop hits the floor.
 * It moves differently from a drop: a drop falls at a steady speed, but
 * a splash bit gets flung upward, then gravity pulls it back down while
 * drag slows its sideways drift — the little up-and-over arc you see
 * when rain hits a puddle. Splashes live in their own free-slot array
 * exactly like drops do.
 *
 *   x, y    : where the bit is, in cells. Starts at the drop's impact
 *             point (set by scene_emit_splashes).
 *
 *   vx, vy  : how fast it moves, in cells/sec. At birth vy points up so
 *             it jumps, and vx is random left-or-right so the bits fan
 *             out from the hit. After that gravity drags vy back down
 *             and drag eats away at vx, giving the arc.
 *
 *   age     : how many seconds this bit has lived. Used both to know
 *             when it dies (age reaches life) and which character to
 *             draw as it fades: '*' fresh, '+' middle-aged, '.' dying.
 *
 *   life    : how long this bit will last, picked at random at birth.
 *             Giving each bit its own lifespan means they fade out at
 *             different moments instead of all blinking off together.
 *
 *   active  : true if this slot holds a live bit (same idea as a drop).
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age;
  float life;
  bool active;
} Splash;

/* ── §6 scene — pools, tick, draw ── */

/*
 * Scene — holds everything about the rain that can change while it runs.
 * It splits into two groups: the simulation fields that the physics step
 * reads and updates, and the render field (just the theme) that only the
 * drawing code looks at. The Scene itself knows nothing about ncurses, so
 * the physics could in principle run with no terminal at all.
 */
typedef struct {
  /* ── simulation: read and written by the physics step ── */

  /* When true, the physics step does nothing, so the rain freezes in
   * place. Drawing still runs, so you see a still frame. Space toggles it. */
  bool paused;

  /* Time-speed multiplier. The default means real time; +/= doubles it,
   * - halves it. It only fast-forwards or slows the clock, not the
   * physics numbers themselves. */
  int speed;

  /* Which of the four presets (DRIZZLE / SHOWER / STORM / MONSOON) is
   * active; n/N cycle it. Switching doesn't wipe the screen — the new
   * target drop count fills in or drains away over a few seconds, so the
   * weather seems to shift naturally. */
  Pattern current_pattern;

  /* Extra wind the player dialed in with w/W, added on top of the
   * preset's wind. It survives preset changes and resets on 'r'. */
  float wind_override;

  /* The scene's own random-number state, seeded from the clock at start
   * and re-seeded on 'r'. Everything random (spawn columns, streak length,
   * splash directions and lifespans) draws from this. */
  uint32_t rng;

  /* The terminal's current size, remembered here so the busy per-frame
   * code doesn't have to ask ncurses every time. Refreshed at start and
   * whenever the window is resized. */
  int rows, cols;

  /* The two fixed arrays of particles. drops[] is the falling rain;
   * splashes[] is the bits flung up where drops land. The `active` flag
   * on each entry says whether the slot is in use. */
  Drop drops[MAX_DROPS];
  Splash splashes[MAX_SPLASHES];

  /* ── render: only the drawing code reads this ── */

  /* Which colour scheme is showing; t/T cycle it. Purely cosmetic — the
   * rain falls exactly the same whatever the theme. */
  int current_theme;
} Scene;

static void scene_clear_pools(Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++)
    s->drops[i].active = false;
  for (int i = 0; i < MAX_SPLASHES; i++)
    s->splashes[i].active = false;
}

static int drop_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++)
    if (!s->drops[i].active)
      return i;
  return -1;
}

static int splash_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_SPLASHES; i++)
    if (!s->splashes[i].active)
      return i;
  return -1;
}

/* ── spawn helpers: pick the starting values for one new particle ── */

/* Choose where along the top a new drop appears. When there's wind, we
 * also let drops start a little off the upwind edge, so slanted rain
 * seems to blow in from the side instead of only falling straight down
 * over each column. The stronger the wind, the further off-edge they
 * can start. */
static inline float drop_spawn_x_wind_aware(uint32_t *rng, float wind,
                                            int cols) {
  float rngx = lcg_unit(rng);
  float over = fabsf(wind) * 0.5f;
  if (wind > 0.5f)
    return rngx * ((float)cols + over) - over;
  if (wind < -0.5f)
    return rngx * ((float)cols + over);
  return rngx * (float)cols;
}

/* Give a new drop its speed. The preset sets the average fall speed and
 * wind, but we nudge each drop's speed and slant a little at random so
 * they don't all march down in a perfect grid. */
static inline void drop_apply_birth_velocity(Drop *d, uint32_t *rng,
                                             float drop_speed, float wind) {
  float speed_jitter =
      (1.0f - DROP_SPEED_VARIANCE * 0.5f) + lcg_unit(rng) * DROP_SPEED_VARIANCE;
  float wind_jitter = (lcg_unit(rng) - 0.5f) * 2.0f * DROP_WIND_JITTER;
  d->vx = wind + wind_jitter;
  d->vy = drop_speed * speed_jitter;
}

/* Give a new drop its trail look: a random streak length from the
 * preset's range and a random gap between trail characters. The variety
 * keeps the rain from looking too uniform. */
static inline void drop_apply_birth_trail(Drop *d, uint32_t *rng,
                                          const PatternParams *pp) {
  d->length =
      pp->length_min + lcg_unit(rng) * (pp->length_max - pp->length_min);
  d->spacing = TRAIL_SPACING_MIN +
               lcg_unit(rng) * (TRAIL_SPACING_MAX - TRAIL_SPACING_MIN);
}

/* ── spawn one new drop ── */

/* The y range is a parameter so one function covers two needs: normal
 * spawns pass a band just above the screen so drops fall in from the top,
 * while the initial fill passes the whole height so the screen looks full
 * of rain from the very first frame. */
static void scene_spawn_drop(Scene *s, float y_min, float y_max) {
  int idx = drop_pool_find_inactive(s);
  if (idx < 0)
    return;
  Drop *d = &s->drops[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;

  d->x = drop_spawn_x_wind_aware(&s->rng, wind, s->cols);
  d->y = y_min + lcg_unit(&s->rng) * (y_max - y_min);
  drop_apply_birth_velocity(d, &s->rng, pp->drop_speed, wind);
  drop_apply_birth_trail(d, &s->rng, pp);
  d->active = true;
}

/*
 * Fill the screen with rain right away by scattering drops over the whole
 * height instead of only at the top. Run at start, on reseed, and on a
 * preset change, so you never see an empty screen slowly filling in.
 */
static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_drops;
  if (target > MAX_DROPS)
    target = MAX_DROPS;

  /* Only add what's missing, so switching to a heavier preset keeps the
   * drops already on screen and just tops up. */
  int active = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      active++;

  float y_max = (float)(s->rows - 2);
  for (int k = active; k < target; k++)
    scene_spawn_drop(s, -6.0f, y_max);
}

/*
 * Throw up a little burst of splash bits where a drop just landed. How
 * many depends on the preset; each gets a small upward jump and a random
 * sideways kick so they fan out.
 */
static void scene_emit_splashes(Scene *s, float impact_x, float impact_y) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int n = (int)((float)SPLASH_BASE_PER_DROP * pp->splash_mul + 0.5f);
  for (int k = 0; k < n; k++) {
    int idx = splash_pool_find_inactive(s);
    if (idx < 0)
      return;
    Splash *sp = &s->splashes[idx];

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);

    sp->x = impact_x;
    sp->y = impact_y;
    sp->vx = SPLASH_KICK_X * (r1 * 2.0f - 1.0f);
    sp->vy = -SPLASH_KICK_UP * (0.6f + r2 * 0.4f);
    sp->age = 0.0f;
    sp->life = SPLASH_LIFE_MIN + r3 * (SPLASH_LIFE_MAX - SPLASH_LIFE_MIN);
    sp->active = true;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_SHOWER;
  s->wind_override = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_pools(s);
  scene_prewarm(s); /* start with the screen already full of rain */
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  /* No need to rebuild the rain — it refills itself to the new size. */
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->wind_override = 0.0f;
  scene_clear_pools(s);
  scene_prewarm(s);
}

/* ── physics-step helpers ── */

static int drops_count_active(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      n++;
  return n;
}

/* Top the rain back up toward the preset's target count, but only add so
 * many per step. The cap scales with the time elapsed, so resuming after
 * a long pause doesn't dump a whole flood at once. */
static void drops_emit_to_target(Scene *s, float dt) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int active = drops_count_active(s);
  int target = pp->target_drops;
  if (target > MAX_DROPS)
    target = MAX_DROPS;
  int spawn_cap = (int)((float)pp->target_drops * dt * 4.0f) + 4;
  int to_spawn = target - active;
  if (to_spawn > spawn_cap)
    to_spawn = spawn_cap;
  for (int k = 0; k < to_spawn; k++)
    scene_spawn_drop(s, -6.0f, -2.0f);
}

/* Move one drop for this step. It just slides at its current speed —
 * raindrops fall at a steady rate, so there's no speeding up. */
static inline void drop_step_constant_velocity(Drop *d, float dt) {
  d->x += d->vx * dt;
  d->y += d->vy * dt;
}

/* Move every drop one step and clear out the ones that died this frame:
 * those blown off the sides, and those that reached the floor (which also
 * throw up their splash bits). */
static void drops_integrate_and_cull(Scene *s, float dt) {
  float kill_y = (float)(s->rows - 2);
  for (int i = 0; i < MAX_DROPS; i++) {
    Drop *d = &s->drops[i];
    if (!d->active)
      continue;

    drop_step_constant_velocity(d, dt);

    if (d->x < -8.0f || d->x > (float)(s->cols + 8)) {
      d->active = false;
      continue;
    }
    if (d->y >= kill_y) {
      scene_emit_splashes(s, d->x, kill_y);
      d->active = false;
    }
  }
}

/* Move one splash bit for this step: gravity pulls it down, drag shaves a
 * bit off its sideways speed, then it slides and ages. The caller works
 * out the drag amount once per step and passes it in. */
static inline void splash_step_kinematic(Splash *sp, float drag_factor,
                                         float dt) {
  sp->vy += SPLASH_GRAVITY * dt;
  sp->vx *= drag_factor;
  sp->x += sp->vx * dt;
  sp->y += sp->vy * dt;
  sp->age += dt;
}

/* Move every splash bit one step and retire the ones that ran out of
 * life or fell below the floor. */
static void splashes_integrate_and_cull(Scene *s, float dt) {
  float kill_y = (float)(s->rows - 2);
  float drag_factor = expf(-SPLASH_DRAG * dt);

  for (int i = 0; i < MAX_SPLASHES; i++) {
    Splash *sp = &s->splashes[i];
    if (!sp->active)
      continue;
    splash_step_kinematic(sp, drag_factor, dt);
    if (sp->age >= sp->life || sp->y > kill_y + 1.0f)
      sp->active = false;
  }
}

/* ── one simulation step ── */

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  drops_emit_to_target(s, dt);
  drops_integrate_and_cull(s, dt);
  splashes_integrate_and_cull(s, dt);
}

/* ── drawing one particle at a time ── */

/* Push the brightness a little further than the eight colours alone can:
 * make the brightest steps bold and the dimmest steps dim. */
static inline int ramp_slot_attr(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Work out which way the trail should stretch: straight back along the
 * drop's path, the opposite of where it's heading. Returns false for a
 * drop that isn't moving, since then there's no direction to trail. */
static inline bool drop_trail_unit_back(const Drop *d, float *out_ux,
                                        float *out_uy) {
  float vlen = sqrtf(d->vx * d->vx + d->vy * d->vy);
  if (vlen < 1e-3f)
    return false;
  *out_ux = -d->vx / vlen;
  *out_uy = -d->vy / vlen;
  return true;
}

/* Draw one drop as a bright head plus a fading streak behind it. The head
 * is the slant character; each step further back uses a fainter character
 * and colour, walking backwards along the drop's path. */
static void drop_render_with_trail(const Drop *d, int cols, int rows) {
  float ux, uy;
  if (!drop_trail_unit_back(d, &ux, &uy))
    return;

  char head_glyph = drop_glyph_for_slope(d->vx, d->vy);
  int trail_n = (int)d->length;
  if (trail_n < 1)
    trail_n = 1;
  if (trail_n > 7)
    trail_n = 7; /* can't fade past the dimmest trail character */

  for (int t = 0; t <= trail_n; t++) {
    int ix = (int)(d->x + ux * d->spacing * (float)t + 0.5f);
    int iy = (int)(d->y + uy * d->spacing * (float)t + 0.5f);
    if (ix < 0 || ix >= cols)
      continue;
    if (iy < 0 || iy >= rows - 1)
      continue;

    int ramp_slot = 7 - t;
    if (ramp_slot < 0)
      ramp_slot = 0;

    char glyph = (t == 0) ? head_glyph : RAMP_GLYPHS[ramp_slot];
    int attr = ramp_slot_attr(ramp_slot);
    int pair = PAIR_RAIN_BASE + ramp_slot;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Pick how a splash bit looks based on how far through its life it is:
 * fresh ones show a bold '*', middle-aged ones a plain '+', and dying
 * ones a dim '.', so each bit visibly fades as it ages. */
static inline void splash_life_phase(float life_ratio, char *out_g,
                                     int *out_attr) {
  if (life_ratio < 0.30f) {
    *out_g = '*';
    *out_attr = A_BOLD;
  } else if (life_ratio < 0.65f) {
    *out_g = '+';
    *out_attr = A_NORMAL;
  } else {
    *out_g = '.';
    *out_attr = A_DIM;
  }
}

static void splash_render(const Splash *sp, int cols, int rows) {
  int ix = (int)(sp->x + 0.5f);
  int iy = (int)(sp->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows - 1)
    return;

  char glyph;
  int attr;
  splash_life_phase(sp->age / sp->life, &glyph, &attr);

  attron(COLOR_PAIR(PAIR_SPLASH) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(PAIR_SPLASH) | attr);
}

static void drops_render(const Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++) {
    const Drop *d = &s->drops[i];
    if (!d->active)
      continue;
    drop_render_with_trail(d, s->cols, s->rows);
  }
}

static void splashes_render(const Scene *s) {
  for (int i = 0; i < MAX_SPLASHES; i++) {
    const Splash *sp = &s->splashes[i];
    if (!sp->active)
      continue;
    splash_render(sp, s->cols, s->rows);
  }
}

/* Draw the whole scene: the falling drops, then the splash bits on top. */
static void scene_draw(const Scene *s) {
  drops_render(s);
  splashes_render(s);
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

/* Tally how many drops and splash bits are live, for the status bar. */
static void scene_counts(const Scene *s, int *out_drops, int *out_spl) {
  int d = 0, p = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      d++;
  for (int i = 0; i < MAX_SPLASHES; i++)
    if (s->splashes[i].active)
      p++;
  *out_drops = d;
  *out_spl = p;
}

/* ── the two HUD bars ── */

/* Paint the top status line. We first lay a coloured background across
 * the whole row, then print the live readout over it (preset, theme,
 * counts, wind, speed, and the measured frame rate). */
static void screen_paint_status_bar(Screen *sc, const Scene *s, double fps,
                                    int sim_fps) {
  int drops, spls;
  scene_counts(s, &drops, &spls);
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;
  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  char status[200];
  snprintf(status, sizeof status,
           " RAIN   %s   theme:%-8s   drops:%4d  splashes:%3d   "
           "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, drops, spls, (double)wind,
           fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Paint the bottom line listing the keys, with its own coloured
 * background filled in the same way as the status bar. */
static void screen_paint_hint_bar(Screen *sc) {
  const char *hints = " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
                      "w/W:wind  +/-:speed  ]/[:Hz ";

  int hint_row = sc->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Draw a frame: clear, draw the rain, then lay the status bar (top) and
 * key hints (bottom) over it. The bars go last so no drop shows through. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);
  screen_paint_status_bar(sc, s, fps, sim_fps);
  screen_paint_hint_bar(sc);
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
    scene_prewarm(s); /* fill in for the new preset's heavier rain */
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
