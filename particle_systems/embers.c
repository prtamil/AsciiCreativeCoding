/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * embers.c — glowing embers that rise from a fire and cool as they go.
 *
 * Each ember is its own little particle: it pops up at a heat source near
 * the bottom of the screen, floats upward, and fades from white-hot through
 * orange and red to dark as it ages out. A bit of random side-to-side wobble
 * makes the column flicker like a real flame. Four presets (bonfire, forge,
 * dragon, hearth) just feed different numbers into the same engine.
 *
 * Sister file: fire.c does the same look with a grid of cells instead of free
 * particles — worth comparing the two.
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

  MAX_EMBERS = 1000,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Color-pair slot numbers. HUD text and the hint line get fixed slots
   * (project convention); the 8 heat colours start at PAIR_HEAT_BASE. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_HEAT_BASE = 3, /* heat ramp uses this slot plus the next 7 */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How far the w/W keys nudge the fire left or right, in cells. */
#define SOURCE_SHIFT_STEP 8.0f

/* The four built-in fire presets. */
typedef enum {
  PATTERN_BONFIRE = 0,
  PATTERN_FORGE = 1,
  PATTERN_DRAGON = 2,
  PATTERN_HEARTH = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_BONFIRE:
    return "BONFIRE";
  case PATTERN_FORGE:
    return "FORGE  ";
  case PATTERN_DRAGON:
    return "DRAGON ";
  case PATTERN_HEARTH:
    return "HEARTH ";
  default:
    return "?      ";
  }
}

/*
 * PatternParams — all the knobs that make one fire preset look the way it
 * does. The simulation never checks "which preset am I?"; it just reads these
 * numbers, so a new fire is one more row in the table below — no new code.
 *
 *   target_embers       : how many embers to keep alive at once (density)
 *   source_x_spread     : how wide the fire is — embers spawn within this
 *                         many cells either side of the source centre
 *   source_y_offset     : how many rows above the bottom HUD line the fire sits
 *   vy_init_mag         : how fast embers shoot up at birth (cells/sec)
 *   vx_init_spread      : random sideways speed at birth, ± cells/sec
 *   buoyancy_accel      : steady upward pull each tick. Negative because
 *                         screen y grows downward, so "up" is a negative
 *                         number (cells/sec²)
 *   turbulence          : strength of the random side-to-side wobble that
 *                         makes the flame flicker (cells/sec²)
 *   life_min, life_max  : an ember lives a random time in this range (seconds)
 *   oscillation_amp_frac: how far the whole fire sways left/right, as a
 *                         fraction of screen width. 0 means it stays put;
 *                         the dragon preset uses this to "breathe"
 *   oscillation_period  : seconds for one full left-right sway
 */
typedef struct {
  int target_embers;
  float source_x_spread;
  int source_y_offset;
  float vy_init_mag;
  float vx_init_spread;
  float buoyancy_accel;
  float turbulence;
  float life_min, life_max;
  float oscillation_amp_frac;
  float oscillation_period;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* Columns follow the struct above:
     *  target xspr yoff vy_i vx_i buoy turb lmin lmax oscamp oscper */
    /* BONFIRE */ {380, 18.0f, 1, 32.0f, 12.0f, -22.0f, 60.0f, 2.5f, 4.2f,
                   0.00f, 0.0f},
    /* FORGE   */
    {220, 3.5f, 1, 60.0f, 4.0f, -38.0f, 90.0f, 1.6f, 2.8f, 0.00f, 0.0f},
    /* DRAGON  */
    {480, 10.0f, 1, 48.0f, 22.0f, -28.0f, 85.0f, 2.6f, 4.0f, 0.22f, 3.5f},
    /* HEARTH  */
    {160, 6.0f, 1, 22.0f, 5.0f, -16.0f, 40.0f, 3.0f, 5.0f, 0.00f, 0.0f},
};

/*
 * Theme — one colour scheme for the fire. The heat[] array is the gradient an
 * ember walks as it cools: slot 0 is the coolest, dimmest colour (a dying
 * ember) and slot 7 is the hottest, brightest (a fresh one). DEFAULT is a
 * normal red-to-yellow fire; the others recolour the same gradient (blue gas
 * flame, green dragon breath, and so on). sky is the faint background tint.
 *
 *   name    : label shown in the HUD
 *   heat[8] : the eight colours, ordered cool -> hot
 *   sky     : background colour for this theme
 *
 * Every colour is picked from the brighter half of the 256-colour palette so
 * even the coolest ember stays visible against a black terminal.
 */
typedef struct {
  const char *name;
  short heat[8]; /* cool -> hot */
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    {"DEFAULT", {88, 124, 160, 166, 202, 208, 214, 226}, 234},
    {"COAL", {52, 88, 124, 160, 166, 202, 208, 220}, 233},
    {"FORGE", {124, 160, 196, 202, 208, 214, 220, 226}, 234},
    {"BLUE", {17, 19, 27, 39, 45, 87, 153, 195}, 233},
    {"GREEN", {22, 28, 34, 64, 70, 112, 156, 192}, 234},
    {"VIOLET", {53, 91, 134, 165, 207, 213, 219, 225}, 234},
    {"COPPER", {130, 137, 173, 179, 215, 222, 229, 230}, 234},
    {"AURORA", {43, 79, 115, 121, 157, 195, 230, 231}, 234},
    {"WHITE_HOT", {240, 243, 245, 247, 249, 251, 253, 255}, 232},
    {"MONO", {244, 246, 248, 250, 252, 253, 254, 255}, 232},
};

/* The characters an ember shows as it heats up: faint marks for cool embers,
 * heavier ones for hot embers. Same idea as the colour ramp, but for shape. */
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

/* Loads one theme's eight heat colours into the terminal's colour slots.
 * Falls back to a coarse red/yellow/white ramp on terminals that only
 * have 8 colours, so the demo still reads as fire there. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), t->heat[i], -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    static const short fb[8] = {
        COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), fb[i], -1);
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

/* ── §3b debug overlays ── */

/*
 * DebugMode — the d/D keys cycle through these views. Each swaps the glyphs
 * for something that reveals one part of how the fire works; the colours stay
 * the same so the picture is still recognisable.
 *
 *   DBG_NORMAL       the real fire — heat colour and glyph per ember.
 *   DBG_VELOCITY     each ember becomes an arrow pointing where it's heading,
 *                    so you can watch the wobble push embers sideways.
 *   DBG_TEMPERATURE  each ember becomes a digit 0-9 for how hot it is now,
 *                    9 fresh and 0 about to die — you see them count down.
 *   DBG_SOURCE       lights up the spawn strip so you can see exactly where
 *                    embers come from (handy for the swaying dragon).
 */
typedef enum {
  DBG_NORMAL = 0,
  DBG_VELOCITY = 1,
  DBG_TEMPERATURE = 2,
  DBG_SOURCE = 3,
  DBG_COUNT = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal",
    "velocity",
    "temperature",
    "source",
};

static DebugMode g_debug = DBG_NORMAL;

/* Turns a velocity into an arrow character ('^', '>', '/', etc.) pointing the
 * way the ember is moving. Used only by the velocity debug view. */
static char dir_char(float vx, float vy) {
  static const char k_dirs[8] = {'>', '\\', 'v', '/', '<', '\\', '^', '/'};
  if (vx == 0.0f && vy == 0.0f)
    return '.';
  float a = atan2f(vy, vx);
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  int idx = (int)((a + (float)M_PI / 8.0f) / ((float)M_PI / 4.0f)) & 7;
  return k_dirs[idx];
}

/* ── §4 ember ── */

/*
 * Ember — one glowing particle. It only stores physics: where it is, how fast
 * it's moving, how old it is, and how long it will live. It does NOT store a
 * colour or a character — those are worked out fresh each frame from its age,
 * which is what lets you switch themes instantly without touching every ember.
 *
 *   x, y     : position on screen, in cells. Kept as floats so motion is
 *              smooth between whole cells.
 *   vx, vy   : speed in cells per second. vy is negative when rising, because
 *              screen y counts downward.
 *   age      : seconds since this ember was born.
 *   life     : seconds this ember will live before it dies. How hot it looks
 *              is just how much life it has left: full life = white-hot,
 *              no life left = cold and gone.
 *   active   : is this pool slot currently a live ember?
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age;
  float life;
  bool active;
} Ember;

/* A tiny, fast random-number generator. Spawning and the per-tick wobble call
 * this thousands of times a frame, so it's kept cheap; plain rand() is only
 * used once at startup to seed it. lcg_unit returns a value in [0, 1). */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── §5 scene ── */

/*
 * Scene — the whole running fire. It holds the pool of embers and all the
 * current settings (paused, which preset and theme, speed, how far the user
 * has nudged the fire, the random seed, and a running clock). The two big jobs
 * are scene_tick (spawn new embers, move them, remove dead ones) and
 * scene_draw (paint the source glow and every ember).
 *
 *   paused          : is the simulation frozen?
 *   speed           : sim-speed multiplier set by +/- keys
 *   current_theme   : index into themes[]
 *   current_pattern : which fire preset is active
 *   source_offset_x : how far the user has shifted the fire sideways (w/W)
 *   rng             : seed for the fast random generator
 *   rows, cols      : current screen size in cells
 *   time_accum      : seconds elapsed, used to sway the dragon's source.
 *                     Only counts up while running, so pausing freezes the
 *                     sway exactly where it was.
 *   embers          : the fixed pool of particles (most are inactive)
 */
typedef struct {
  bool paused;
  int speed;
  int current_theme;
  Pattern current_pattern;
  float source_offset_x;
  uint32_t rng;
  int rows, cols;

  float time_accum;

  Ember embers[MAX_EMBERS];
} Scene;

static int ember_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_EMBERS; i++)
    if (!s->embers[i].active)
      return i;
  return -1;
}

static void scene_clear_embers(Scene *s) {
  for (int i = 0; i < MAX_EMBERS; i++)
    s->embers[i].active = false;
}

/* Where the centre of the fire is right now, left to right. Starts at screen
 * middle, plus the user's nudge, plus the dragon's gentle sway if this preset
 * has one. */
static float scene_source_cx(const Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float cx = (float)s->cols * 0.5f + s->source_offset_x;
  if (pp->oscillation_amp_frac > 0.0f && pp->oscillation_period > 0.0f) {
    float amp = pp->oscillation_amp_frac * (float)s->cols;
    float w = 2.0f * (float)M_PI / pp->oscillation_period;
    cx += amp * sinf(w * s->time_accum);
  }
  return cx;
}

/*
 * The five spawn_random_* helpers below each pick one starting value for a new
 * ember, with a little randomness mixed in. The randomness matters: if every
 * ember started identically they'd rise in lockstep and the flame would look
 * like a marching grid instead of a living fire. The ranges are small, so the
 * embers still clearly belong to the same source.
 */
/* A spawn x somewhere across the width of the source. */
static inline float spawn_random_x(uint32_t *rng, float source_cx,
                                   const PatternParams *pp) {
  float r = lcg_unit(rng);
  return source_cx + (r - 0.5f) * 2.0f * pp->source_x_spread;
}

/* A spawn y at the source row, nudged down a touch so the base has some depth. */
static inline float spawn_random_y(uint32_t *rng, const PatternParams *pp,
                                   int rows) {
  float r = lcg_unit(rng);
  return (float)(rows - 2 - pp->source_y_offset) - r * 1.5f;
}

/* A small random sideways speed at birth. */
static inline float spawn_random_vx(uint32_t *rng, const PatternParams *pp) {
  float r = lcg_unit(rng);
  return (r - 0.5f) * 2.0f * pp->vx_init_spread;
}

/* An upward launch speed, give or take 30% so embers don't all rise together.
 * Negative because "up" is a negative y on screen. */
static inline float spawn_random_vy(uint32_t *rng, const PatternParams *pp) {
  float r = lcg_unit(rng);
  return -pp->vy_init_mag * (0.7f + r * 0.6f);
}

/* A random lifetime within this preset's range. */
static inline float spawn_random_life(uint32_t *rng, const PatternParams *pp) {
  float r = lcg_unit(rng);
  return pp->life_min + r * (pp->life_max - pp->life_min);
}

static void scene_spawn_ember(Scene *s) {
  int idx = ember_pool_find_inactive(s);
  if (idx < 0)
    return;
  Ember *e = &s->embers[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float source_cx = scene_source_cx(s);

  e->x = spawn_random_x(&s->rng, source_cx, pp);
  e->y = spawn_random_y(&s->rng, pp, s->rows);
  e->vx = spawn_random_vx(&s->rng, pp);
  e->vy = spawn_random_vy(&s->rng, pp);
  e->age = 0.0f;
  e->life = spawn_random_life(&s->rng, pp);
  e->active = true;
}

/*
 * Fills the screen with a fully-burning fire right away, so the first frame
 * already looks alive instead of starting from a single spark at the bottom.
 * It spawns a full batch of embers but pretends each one has already been
 * burning a random amount of time, then jumps it up to roughly where it would
 * be by now. It's an estimate, not an exact replay, but it looks right.
 */
static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_embers;
  if (target > MAX_EMBERS)
    target = MAX_EMBERS;

  int active = 0;
  for (int i = 0; i < MAX_EMBERS; i++)
    if (s->embers[i].active)
      active++;

  for (int k = active; k < target; k++) {
    scene_spawn_ember(s);
    /* Find the ember we just made and pretend it's already been burning. */
    for (int i = 0; i < MAX_EMBERS; i++) {
      Ember *e = &s->embers[i];
      if (e->active && e->age == 0.0f) {
        float age_frac = lcg_unit(&s->rng);
        e->age = age_frac * e->life;
        /* Jump it up to roughly where it would have risen by now. */
        e->y += e->vy * e->age + 0.5f * pp->buoyancy_accel * e->age * e->age;
        e->x += e->vx * e->age;
        break;
      }
    }
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_BONFIRE;
  s->source_offset_x = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  scene_clear_embers(s);
  scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->source_offset_x = 0.0f;
  scene_clear_embers(s);
  scene_prewarm(s);
}

/*
 * Below are the per-ember physics steps and the tick that ties them together.
 * A tick has two phases: top up the pool with new embers, then move every
 * live one and drop the ones that have died.
 *
 * The spawn rate is capped per frame so the fire grows in smoothly rather than
 * popping into existence all at once (which would look jarring right after you
 * switch to a denser preset).
 */
/* How many embers are alive right now. */
static int count_active_embers(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_EMBERS; i++)
    if (s->embers[i].active)
      n++;
  return n;
}

/* The steady upward pull on the ember this tick. */
static inline void ember_apply_buoyancy(Ember *e, const PatternParams *pp,
                                        float dt) {
  e->vy += pp->buoyancy_accel * dt;
}

/* A small random sideways shove this tick — this is what makes a flame flicker
 * instead of rising in a dead-straight line. */
static inline void ember_apply_turbulence_kick(Ember *e,
                                               const PatternParams *pp,
                                               float dt, uint32_t *rng) {
  float turb = (lcg_unit(rng) - 0.5f) * pp->turbulence;
  e->vx += turb * dt;
}

/* Move the ember by its speed and age it by one tick. */
static inline void ember_integrate_position(Ember *e, float dt) {
  e->x += e->vx * dt;
  e->y += e->vy * dt;
  e->age += dt;
}

/* True once an ember has either burned out or drifted off the screen. */
static inline bool ember_is_dead(const Ember *e, int cols) {
  if (e->age >= e->life)
    return true;
  if (e->x < -2.0f || e->x > (float)(cols + 2))
    return true;
  if (e->y < -2.0f)
    return true;
  return false;
}

/* One full step for a single ember: wobble, rise, move, then check if it died. */
static inline void ember_advance_one(Ember *e, const PatternParams *pp,
                                     float dt, uint32_t *rng, int cols) {
  ember_apply_turbulence_kick(e, pp, dt, rng);
  ember_apply_buoyancy(e, pp, dt);
  ember_integrate_position(e, dt);
  if (ember_is_dead(e, cols))
    e->active = false;
}

/* Phase 1 of a tick: add new embers until the pool is back up to this preset's
 * target, but only a few per frame so the fire fills in gradually. */
static void topup_ember_pool(Scene *s, const PatternParams *pp, float dt) {
  int active = count_active_embers(s);
  int target = pp->target_embers;
  if (target > MAX_EMBERS)
    target = MAX_EMBERS;
  int spawn_cap = (int)((float)pp->target_embers * dt * 4.0f) + 4;
  int to_spawn = target - active;
  if (to_spawn > spawn_cap)
    to_spawn = spawn_cap;
  for (int k = 0; k < to_spawn; k++)
    scene_spawn_ember(s);
}

/* Phase 2 of a tick: advance every live ember by one step. */
static void integrate_all_embers(Scene *s, const PatternParams *pp, float dt) {
  for (int i = 0; i < MAX_EMBERS; i++) {
    Ember *e = &s->embers[i];
    if (!e->active)
      continue;
    ember_advance_one(e, pp, dt, &s->rng, s->cols);
  }
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  dt *= speed_mul;
  s->time_accum += dt;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  topup_ember_pool(s, pp, dt);     /* add new embers */
  integrate_all_embers(s, pp, dt); /* move them, remove dead ones */
}

/*
 * Drawing happens in two passes, in this order on purpose: first a bright
 * glow strip at the base of the fire, then the embers on top of it. Painting
 * the glow first means embers always sit over it, never the other way round.
 *
 * The glow strip is a small cheat — just some hot cells where embers are born
 * — but without it the embers look like sparks floating in mid-air with no
 * fire underneath them.
 */
/* How hot an ember looks: 1.0 when freshly born, fading to 0.0 as it dies.
 * It's just how much of its life is left. */
static inline float ember_temperature(const Ember *e) {
  if (e->life <= 0.0f)
    return 0.0f;
  float T = 1.0f - e->age / e->life;
  if (T < 0.0f)
    T = 0.0f;
  if (T > 1.0f)
    T = 1.0f;
  return T;
}

/* Turns a heat value (0 to 1) into one of the 8 colour/glyph steps. */
static inline int temperature_to_ramp_slot(float T) {
  int slot = (int)(T * 7.999f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* Makes the hottest embers bold and the dying ones faint, for extra contrast. */
static inline int attr_for_ramp_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Draws one character with its colour and style, so callers don't repeat the
 * attron/attroff dance. */
static inline void paint_glyph(int y, int x, char glyph, int pair, int attr) {
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(y, x, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Picks the glow brightness across the base: hottest in the middle, cooler
 * toward the edges, but never below a fairly hot floor. */
static inline int source_band_slot_for_dx(int dx, int half) {
  float r = (float)abs(dx) / (float)(half + 1);
  int slot = (int)((1.0f - r) * 7.0f + 0.5f);
  if (slot < 4)
    slot = 4; /* keep the whole base looking hot */
  if (slot > 7)
    slot = 7;
  return slot;
}

/* Pass 1: paints a two-row glowing strip at the base so the fire looks like
 * it's coming from somewhere. */
static void draw_source_band(const Scene *s, const PatternParams *pp,
                             int rows_eff) {
  float source_cx = scene_source_cx(s);
  int src_y0 = s->rows - 2 - pp->source_y_offset;
  int src_y1 = src_y0 + 1;
  int half = (int)(pp->source_x_spread + 0.5f);

  for (int dy = 0; dy <= 1; dy++) {
    int y = (dy == 0) ? src_y0 : src_y1;
    if (y < 0 || y >= rows_eff)
      continue;
    for (int dx = -half; dx <= half; dx++) {
      int x = (int)(source_cx + 0.5f) + dx;
      if (x < 0 || x >= s->cols)
        continue;
      int slot = source_band_slot_for_dx(dx, half);
      char glyph = (g_debug == DBG_SOURCE) ? '#' : (slot >= 6) ? '*' : '+';
      int pair =
          (g_debug == DBG_SOURCE) ? PAIR_HEAT_BASE + 7 : PAIR_HEAT_BASE + slot;
      paint_glyph(y, x, glyph, pair, A_BOLD);
    }
  }
}

/* Chooses the character to draw for an ember. Normally it's the heat glyph,
 * but the debug views swap in an arrow or a digit instead. The colour is
 * decided elsewhere, so debug views still look like fire. */
static char pick_ember_glyph(int slot, float T, const Ember *e) {
  switch (g_debug) {
  case DBG_VELOCITY:
    return dir_char(e->vx, e->vy);
  case DBG_TEMPERATURE: {
    int digit = (int)(T * 9.0f + 0.5f);
    if (digit < 0)
      digit = 0;
    if (digit > 9)
      digit = 9;
    return (char)('0' + digit);
  }
  case DBG_NORMAL:
  case DBG_SOURCE:
  default:
    return RAMP_GLYPHS[slot];
  }
}

/* Draws a single ember at its rounded cell position, skipping it if off-screen. */
static void draw_one_ember(const Ember *e, int cols, int rows_eff) {
  int ix = (int)(e->x + 0.5f);
  int iy = (int)(e->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  float T = ember_temperature(e);
  int slot = temperature_to_ramp_slot(T);
  char glyph = pick_ember_glyph(slot, T, e);
  int attr = attr_for_ramp_slot(slot);
  paint_glyph(iy, ix, glyph, PAIR_HEAT_BASE + slot, attr);
}

/* Pass 2: paints every live ember on top of the glow strip. */
static void draw_all_embers(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_EMBERS; i++) {
    const Ember *e = &s->embers[i];
    if (!e->active)
      continue;
    draw_one_ember(e, s->cols, rows_eff);
  }
}

static void scene_draw(const Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int rows_eff = s->rows - 1; /* keep the bottom row free for the HUD */

  draw_source_band(s, pp, rows_eff); /* glow strip first */
  draw_all_embers(s, rows_eff);      /* embers on top */
}

/* ── §6 screen ── */

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

/* Counts live embers for the HUD readout. */
static int scene_active_count(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_EMBERS; i++)
    if (s->embers[i].active)
      n++;
  return n;
}

/*
 * Draws the whole frame: the fire, the status line along the top (fps, preset,
 * theme, ember count, and so on), and the key reminder along the bottom. The
 * debug label is only added to the top line when a debug view is on, to keep
 * it uncluttered the rest of the time.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int active = scene_active_count(s);
  const char *state_str =
      s->paused ? "PAUSED" : pattern_name(s->current_pattern);

  char top[160];
  if (g_debug == DBG_NORMAL) {
    snprintf(top, sizeof top,
             " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
             "embers:%d  src_x:%+.1f ",
             fps, sim_fps, s->speed, state_str, themes[s->current_theme].name,
             active, (double)s->source_offset_x);
  } else {
    snprintf(top, sizeof top,
             " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
             "embers:%d  src_x:%+.1f  [dbg:%s] ",
             fps, sim_fps, s->speed, state_str, themes[s->current_theme].name,
             active, (double)s->source_offset_x, k_debug_names[g_debug]);
  }
  int top_x = sc->cols - (int)strlen(top);
  if (top_x < 0)
    top_x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, top_x, "%s", top);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q/ESC:quit  spc:pause  r:reseed  n/p:pattern"
           "  t/T:theme  w/W:source  +/-:speed  ]/[:Hz  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 app ── */

/*
 * App — top-level state for the running program: the fire, the terminal, the
 * tick rate, and two flags that the OS signal handlers flip. running goes to 0
 * to quit cleanly; need_resize is set when the terminal window changes size.
 * Both are sig_atomic_t because a signal can touch them at any moment.
 */
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
    scene_prewarm(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_prewarm(s);
    break;

  case 'w':
    s->source_offset_x += SOURCE_SHIFT_STEP;
    break;
  case 'W':
    s->source_offset_x -= SOURCE_SHIFT_STEP;
    break;

  case 'd':
    g_debug = (DebugMode)((g_debug + 1) % DBG_COUNT);
    break;
  case 'D':
    g_debug = (DebugMode)((g_debug + DBG_COUNT - 1) % DBG_COUNT);
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
