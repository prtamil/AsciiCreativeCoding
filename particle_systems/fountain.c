/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fountain.c — water shoots up from a source and arcs back down under gravity.
 *
 * One little engine drives four looks: a thin GEYSER jet, a wide
 * FOUNTAIN cone, a falling WATERFALL sheet, and a hot VOLCANIC spray.
 * Each drop is launched at a random angle inside a cone, gravity bends
 * its path into an arc, and when it hits the floor it bursts into a few
 * short-lived splash specks. The four looks differ only in the numbers
 * fed to the engine (see pattern_params below), never in the code path.
 *
 * Sister files: rain.c (same pool + splash trick),
 * physics/bounce_ball.c (same gravity-arc motion).
 * Idea credit: Reeves, "Particle Systems", ACM TOG 2(2), 1983.
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

  MAX_DROPS = 900,
  MAX_SPLASHES = 600,
  SPLASH_BASE_PER_DROP = 4,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Slot numbers for our color pairs. HUD and HINT slots are fixed
     project-wide so every demo's status bars match. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_WATER_BASE = 3, /* this slot plus the next 7 = water ramp, dim to bright */
  PAIR_LAVA_BASE = 11, /* this slot plus the next 7 = lava ramp,  dim to bright */
  PAIR_SPLASH = 19,
  PAIR_SKY = 20,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How the little splash specks behave when a drop lands. */
#define SPLASH_GRAVITY 220.0f
#define SPLASH_KICK_UP 25.0f
#define SPLASH_KICK_X 20.0f
#define SPLASH_DRAG 2.5f
#define SPLASH_LIFE_MIN 0.25f
#define SPLASH_LIFE_MAX 0.55f

/* Each drop's launch speed is wiggled by this much so the stream isn't
   robotically uniform — here, give or take 15%. */
#define DROP_SPEED_VARIANCE 0.30f

/* The four fountain looks. */
typedef enum {
  PATTERN_GEYSER = 0,
  PATTERN_FOUNTAIN = 1,
  PATTERN_WATERFALL = 2,
  PATTERN_VOLCANIC = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_GEYSER:
    return "GEYSER   ";
  case PATTERN_FOUNTAIN:
    return "FOUNTAIN ";
  case PATTERN_WATERFALL:
    return "WATERFALL";
  case PATTERN_VOLCANIC:
    return "VOLCANIC ";
  default:
    return "?        ";
  }
}

/*
 * PatternParams — the recipe for one fountain look.
 *
 * This is the heart of the "one engine, four looks" idea: the engine
 * code never asks "which pattern is this?". It just reads the numbers
 * below and does the same thing every time. Swap the numbers and you
 * get a geyser instead of a waterfall, no new code.
 *
 *   target_drops    : how many drops to keep in the air at once. The
 *                     spawner tops the stream up toward this number
 *                     each tick. Bigger = thicker stream.
 *   source_top      : where the drops are born. true = up at the top
 *                     edge, so they rain down into view (the waterfall).
 *                     false = just above the bottom status bar, so they
 *                     shoot up. Also decides which end of the colour
 *                     ramp is the bright end (see drop_height_fraction).
 *   source_x_spread : how far left/right of centre, in cells, drops can
 *                     appear. Small (about 2) makes a narrow column;
 *                     large (28) makes a wide curtain.
 *   speed_init      : how fast a drop is launched, in cells per second.
 *                     The real speed is nudged a little each time so the
 *                     stream looks alive (see DROP_SPEED_VARIANCE).
 *   cone_half_angle : how wide the spray fans out, in radians (half the
 *                     full cone). Small (0.10, about 6 degrees) is a
 *                     tight jet; large (0.75, about 43 degrees) is a
 *                     broad spray.
 *   upward          : true = drops are flung up and gravity arcs them
 *                     back; false = drops are aimed down (the waterfall).
 *   gravity         : how hard down-pull tugs each tick, in cells per
 *                     second squared. More gravity = lower, snappier arcs.
 *   life_max        : oldest a drop may get, in seconds. A safety net so
 *                     a drop drifting sideways forever eventually dies
 *                     instead of haunting the screen.
 *   hot_palette     : true = colour drops with the hot lava ramp (the
 *                     volcanic look); false = the cool water ramp.
 *   splash_mul      : how many splash specks each landing throws, as a
 *                     multiple of the base count. 1.40 = chunky lava
 *                     splatter, 0.50 = light water mist.
 */
typedef struct {
  int target_drops;
  bool source_top;
  float source_x_spread;
  float speed_init;
  float cone_half_angle;
  bool upward;
  float gravity;
  float life_max;
  bool hot_palette;
  float splash_mul;
} PatternParams;

/* The four recipes. Fields in order: target_drops, source_top,
   source_x_spread, speed_init, cone_half_angle, upward, gravity,
   life_max, hot_palette, splash_mul. */
static const PatternParams pattern_params[N_PATTERNS] = {
    /* GEYSER     */ {240, false, 2.0f, 85.0f, 0.10f, true, 120.0f, 5.0f, false,
                      0.50f},
    /* FOUNTAIN   */
    {380, false, 3.0f, 72.0f, 0.55f, true, 110.0f, 4.5f, false, 0.85f},
    /* WATERFALL  */
    {450, true, 28.0f, 20.0f, 0.10f, false, 130.0f, 3.5f, false, 1.10f},
    /* VOLCANIC   */
    {340, false, 5.0f, 115.0f, 0.75f, true, 130.0f, 5.0f, true, 1.40f},
};

/*
 * Theme — one colour scheme for the whole fountain.
 *
 * Each theme carries two 8-colour gradients ("ramps"): a cool water ramp
 * for the normal fountains and a hot lava ramp for the volcanic one.
 * A drop picks a colour from the ramp by how high it is — low drops use
 * the dim end (slot 0), drops near the peak use the bright end (slot 7).
 *
 * The colour numbers are deliberately all kept in the brighter half of
 * the terminal's 256-colour set so nothing turns invisible on a black
 * background (project palette rule).
 *
 *   name   : label shown in the status bar.
 *   water  : 8 colours, dim (slot 0) to bright (slot 7), for water drops.
 *   lava   : 8 colours, dim to bright, for volcanic drops.
 *   splash : single colour for the splash specks.
 *   sky    : background tint (currently unused for fill, kept per theme).
 */
typedef struct {
  const char *name;
  short water[8];
  short lava[8];
  short splash;
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] =
    {
        {"OCEANIC",
         {24, 30, 31, 38, 44, 51, 87, 159},
         {30, 36, 73, 79, 122, 159, 195, 231},
         159,
         234},
        {"MATRIX",
         {28, 34, 40, 46, 82, 118, 154, 190},
         {58, 64, 100, 106, 142, 184, 190, 226},
         154,
         234},
        {"NEON",
         {53, 91, 134, 165, 201, 207, 213, 219},
         {198, 199, 200, 201, 207, 213, 219, 225},
         219,
         234},
        {"FIRE",
         {52, 88, 124, 160, 196, 202, 208, 214},
         {88, 124, 160, 196, 202, 208, 214, 226},
         214,
         234},
        {"ICE",
         {24, 31, 67, 110, 117, 153, 195, 231},
         {117, 153, 159, 195, 230, 231, 254, 255},
         231,
         235},
        {"NOVA",
         {24, 75, 117, 159, 195, 219, 226, 231},
         {130, 166, 202, 208, 214, 220, 226, 231},
         231,
         234},
        {"SUNSET",
         {95, 131, 167, 174, 210, 217, 224, 230},
         {88, 124, 160, 166, 202, 208, 214, 220},
         217,
         234},
        {"FOREST",
         {28, 64, 70, 76, 112, 148, 184, 220},
         {94, 130, 136, 172, 208, 214, 220, 226},
         184,
         234},
        {"AMETHYST",
         {54, 91, 92, 98, 134, 141, 177, 219},
         {88, 125, 162, 199, 200, 207, 213, 219},
         213,
         234},
        {"ECLIPSE",
         {52, 88, 95, 131, 167, 173, 209, 215},
         {52, 88, 124, 160, 196, 202, 208, 214},
         209,
         232},
};

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
      init_pair((short)(PAIR_WATER_BASE + i), t->water[i], -1);
      init_pair((short)(PAIR_LAVA_BASE + i), t->lava[i], -1);
    }
    init_pair(PAIR_SPLASH, t->splash, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++) {
      init_pair((short)(PAIR_WATER_BASE + i), COLOR_CYAN, -1);
      init_pair((short)(PAIR_LAVA_BASE + i), COLOR_YELLOW, -1);
    }
    init_pair(PAIR_SPLASH, COLOR_WHITE, -1);
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

/* ── §4 drop ── */

/*
 * Drop — one flying water particle, mid-arc.
 *
 *   x, y   : where it is right now, measured in cells. Kept as floats so
 *            motion looks smooth; we only round to a whole cell at the
 *            last moment, when drawing.
 *   vx, vy : how fast it's moving, in cells per second. Remember the
 *            screen's y grows downward, so a NEGATIVE vy means going UP
 *            and a positive vy means falling. When vy is near zero the
 *            drop is at the very top of its arc.
 *   age    : seconds since it was born. Used to retire a drop that has
 *            been around too long (a drifting one that never lands).
 *   active : is this pool slot in use? Dead slots are skipped, and the
 *            spawner reuses the first free one it finds.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age;
  bool active;
} Drop;

/* A tiny, fast random-number generator (the classic "linear congruential"
   trick) so we don't lean on rand() in the hot loop. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/*
 * Pick the character to draw for a drop based on how it's moving, so the
 * stream visibly rises, hangs at the top, and falls. Barely moving up or
 * down looks like the peak ('*'); rising shows '^' or '\''; falling
 * shows ',' or '.', with the faster one in each pair being the punchier
 * glyph.
 */
static char drop_glyph_for_velocity(float vy) {
  float a = fabsf(vy);
  if (a < 6.0f)
    return '*'; /* hanging at the top of the arc */
  if (vy < 0.0f)
    return (a > 40.0f ? '^' : '\'');
  return (a > 40.0f) ? ',' : '.';
}

/* ── §5 splash ── */

/*
 * Splash — one tiny speck thrown off when a drop hits the floor. It
 * pops up, fans out, falls back, and quickly fades. Like a Drop, but it
 * carries its own lifespan so each speck dies at a slightly different
 * moment, which reads as a natural fizzle rather than all at once.
 *
 *   x, y   : where the speck is, in cells. Starts at the landing spot.
 *   vx, vy : its speed, in cells per second. It launches upward (vy
 *            starts negative) and a random amount left or right, so the
 *            burst fans out. As it flies the sideways speed fades away
 *            (drag) while gravity pulls it back down.
 *   age    : seconds since this speck appeared.
 *   life   : how long this speck is allowed to live, picked at random in
 *            the SPLASH_LIFE_MIN..MAX range. Once age reaches life it
 *            disappears. The age/life ratio also drives the fade from
 *            '*' (fresh) to '+' to '.' (about to vanish).
 *   active : is this pool slot in use? (same idea as Drop.active.)
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age, life;
  bool active;
} Splash;

/* ── §6 scene — pools, tick, draw ── */

typedef struct {
  bool paused;
  int speed;
  int current_theme;
  Pattern current_pattern;
  uint32_t rng;
  int rows, cols;

  Drop drops[MAX_DROPS];
  Splash splashes[MAX_SPLASHES];
} Scene;

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

static void scene_clear_pools(Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++)
    s->drops[i].active = false;
  for (int i = 0; i < MAX_SPLASHES; i++)
    s->splashes[i].active = false;
}

/*
 * Launch one new drop. It picks a random angle inside the pattern's
 * spray cone and a slightly random speed, then turns that angle+speed
 * into sideways and up/down velocity. Upward patterns aim up; the
 * waterfall aims down.
 */
static void scene_spawn_drop(Scene *s) {
  int idx = drop_pool_find_inactive(s);
  if (idx < 0)
    return;
  Drop *d = &s->drops[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];

  float r1 = lcg_unit(&s->rng);
  float r2 = lcg_unit(&s->rng);
  float r3 = lcg_unit(&s->rng);

  /* Where the drop is born. */
  float src_cx = (float)s->cols * 0.5f;
  float src_x = src_cx + (r1 - 0.5f) * 2.0f * pp->source_x_spread;
  float src_y = pp->source_top ? (-1.0f - r2 * 1.5f) : (float)(s->rows - 3);

  /* Random aim within the cone, and a slightly random launch speed. */
  float angle = (r2 - 0.5f) * 2.0f * pp->cone_half_angle;
  float speed = pp->speed_init * ((1.0f - DROP_SPEED_VARIANCE * 0.5f) +
                                  r3 * DROP_SPEED_VARIANCE);
  float vx = speed * sinf(angle);
  float vy = pp->upward ? -speed * cosf(angle) : speed * cosf(angle);

  d->x = src_x;
  d->y = src_y;
  d->vx = vx;
  d->vy = vy;
  d->age = 0.0f;
  d->active = true;
}

/* Throw a little burst of splash specks at the spot where a drop landed. */
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
  s->current_pattern = PATTERN_FOUNTAIN;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_pools(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  scene_clear_pools(s);
}

/* How many drops are alive right now — the spawner uses this to decide
   how many more to add. */
static int drops_count_active(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      n++;
  return n;
}

/*
 * Top the stream up toward its target drop count. We add only a few per
 * tick (a cap that scales with the time step) so that after a pause or a
 * hiccup the fountain refills smoothly instead of belching out a huge
 * clump all at once.
 */
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
    scene_spawn_drop(s);
}

/*
 * Move one drop forward by one small time step: gravity speeds it
 * downward, then it slides over by its current speed. Doing this every
 * tick is what bends the straight launch into a falling arc.
 */
static void drop_step_ballistic(Drop *d, float gravity, float dt) {
  d->vy += gravity * dt;
  d->x += d->vx * dt;
  d->y += d->vy * dt;
  d->age += dt;
}

/*
 * Move every live drop one step, then retire the ones that are done.
 * A drop dies if it's too old, if it has wandered off the sides, or if
 * it reached the floor — and a floor landing is the fun part, since it
 * kicks off a splash.
 */
static void drops_integrate_and_cull(Scene *s, float dt) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float kill_y = (float)(s->rows - 2);

  for (int i = 0; i < MAX_DROPS; i++) {
    Drop *d = &s->drops[i];
    if (!d->active)
      continue;

    drop_step_ballistic(d, pp->gravity, dt);

    if (d->age > pp->life_max) {
      d->active = false;
      continue;
    }
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

/*
 * Move one splash speck forward one step: gravity pulls it down and its
 * sideways speed bleeds off a bit (drag), then it slides to its new
 * spot. The caller works out the drag amount once and shares it across
 * all specks this frame.
 */
static void splash_step_kinematic(Splash *sp, float drag_factor, float dt) {
  sp->vy += SPLASH_GRAVITY * dt;
  sp->vx *= drag_factor;
  sp->x += sp->vx * dt;
  sp->y += sp->vy * dt;
  sp->age += dt;
}

/* Move every splash speck one step and retire any that have used up
   their lifespan or dropped below the floor. */
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

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  drops_emit_to_target(s, dt);
  drops_integrate_and_cull(s, dt);
  splashes_integrate_and_cull(s, dt);
}

/*
 * Turn a drop's height into a 0-to-1 number that picks its colour, where
 * 1 is the bright end of the ramp. For upward fountains, higher = brighter
 * (the peak glows). For the waterfall, the top of the screen is brightest
 * and it dims on the way down. The little "+1" just guards against
 * dividing by zero on a very short screen.
 */
static float drop_height_fraction(const Drop *d, const PatternParams *pp,
                                  int rows, float kill_y) {
  float h;
  if (pp->source_top) {
    h = 1.0f - d->y / kill_y;
  } else {
    float source_y = (float)(rows - 3);
    h = (source_y - d->y) / (source_y + 1.0f);
  }
  if (h < 0.0f)
    h = 0.0f;
  if (h > 1.0f)
    h = 1.0f;
  return h;
}

/* Squeeze a bit more contrast out of the 8-colour ramp: make the top
   colours bold and the bottom ones dim, leaving the middle plain. */
static int ramp_slot_attr(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Draw one drop, skipping it if it rounds to a spot off the screen. */
static void drop_render(const Drop *d, const PatternParams *pp, int pair_base,
                        int rows, int cols, float kill_y) {
  int ix = (int)(d->x + 0.5f);
  int iy = (int)(d->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows - 1)
    return;

  float h_frac = drop_height_fraction(d, pp, rows, kill_y);
  int ramp_slot = (int)(h_frac * 7.0f + 0.5f);
  if (ramp_slot < 0)
    ramp_slot = 0;
  if (ramp_slot > 7)
    ramp_slot = 7;

  char glyph = drop_glyph_for_velocity(d->vy);
  int attr = ramp_slot_attr(ramp_slot);
  int pair = pair_base + ramp_slot;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

static void drops_render(const Scene *s, const PatternParams *pp, int pair_base,
                         float kill_y) {
  for (int i = 0; i < MAX_DROPS; i++) {
    const Drop *d = &s->drops[i];
    if (!d->active)
      continue;
    drop_render(d, pp, pair_base, s->rows, s->cols, kill_y);
  }
}

/*
 * Choose how a splash speck looks based on how far through its life it
 * is, so it visibly fades out: a fresh, bright '*', then a plainer '+',
 * then a dim '.' just before it vanishes.
 */
static void splash_life_phase(float life_ratio, char *out_glyph,
                              int *out_attr) {
  if (life_ratio < 0.30f) {
    *out_glyph = '*';
    *out_attr = A_BOLD;
  } else if (life_ratio < 0.65f) {
    *out_glyph = '+';
    *out_attr = A_NORMAL;
  } else {
    *out_glyph = '.';
    *out_attr = A_DIM;
  }
}

static void splash_render(const Splash *sp, const PatternParams *pp, int rows,
                          int cols) {
  int ix = (int)(sp->x + 0.5f);
  int iy = (int)(sp->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows - 1)
    return;

  char glyph;
  int attr;
  splash_life_phase(sp->age / sp->life, &glyph, &attr);

  /* Volcanic specks glow with the hottest lava colour; everything else
     uses the plain splash colour. */
  int splash_pair = pp->hot_palette ? (PAIR_LAVA_BASE + 6) : PAIR_SPLASH;
  attron(COLOR_PAIR(splash_pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(splash_pair) | attr);
}

static void splashes_render(const Scene *s, const PatternParams *pp) {
  for (int i = 0; i < MAX_SPLASHES; i++) {
    const Splash *sp = &s->splashes[i];
    if (!sp->active)
      continue;
    splash_render(sp, pp, s->rows, s->cols);
  }
}

static void scene_draw(const Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int pair_base = pp->hot_palette ? PAIR_LAVA_BASE : PAIR_WATER_BASE;
  float kill_y = (float)(s->rows - 2);

  drops_render(s, pp, pair_base, kill_y);
  splashes_render(s, pp);
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

/*
 * Draw the fountain, then lay the two status bars on top: a bright info
 * line across the top (pattern, theme, counts, speed) and a key-list line
 * across the bottom. We paint the bars last so no drop ever shows through
 * them, and we fill each bar's whole row with colour even where the text
 * runs short.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int drops, spls;
  scene_counts(s, &drops, &spls);

  const char *state_str =
      s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

  /* Top bar: what's running. */
  char status[200];
  snprintf(status, sizeof status,
           " FOUNTAIN   %s   theme:%-9s   drops:%4d  splashes:%3d   "
           "%5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, drops, spls, fps, sim_fps,
           s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom bar: every key you can press. */
  const char *hints = " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
                      "+/-:speed  ]/[:Hz ";

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
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
