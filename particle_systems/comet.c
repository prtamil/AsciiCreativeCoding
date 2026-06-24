/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * comet.c — comets that fly in from a screen edge, leave a fading trail,
 * and burst into sparks when they hit the floor.
 *
 * There are three looks (SHOOTING_STAR, FIREBALL, PLASMA_BOLT) and 10
 * colour themes you can flip between live. The trick that makes the trail
 * lag behind the comet (drop dots where the comet is, but don't give them
 * the comet's speed) comes from Reeves, "Particle Systems", ACM TOG 2(2),
 * 1983.
 *
 * Keys
 *   q | Q | ESC   quit                spc   pause / resume
 *   r             clear and respawn one fresh comet
 *   n / N         next look           p / P  previous look
 *   t / T         next / prev theme
 *   + / =         go faster           -      go slower
 *   ] / [         raise / lower sim rate
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/comet.c \
 *       -o comet -lncurses -lm
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

  MAX_COMETS = 8,
  MAX_TRAIL = 1000,
  MAX_BLASTS = 8, /* every comet can be mid-explosion at once */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slot numbers. The HUD/HINT pairs are reserved project-wide. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* +0..+7: the 8 trail shades, dim/cool up to bright/hot */
  PAIR_HEAD = 11,     /* the bright dot at the comet's head */
  PAIR_HALO = 12,     /* the faint glow around the head */
  PAIR_SKY = 13,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How far a comet may drift past the edge before we let it off the screen. */
#define EDGE_MARGIN 3.0f

/* Knobs for the explosion when a comet hits the floor. */
#define BLAST_PARTICLES 32        /* sparks thrown per explosion */
#define BLAST_FLASH_SEC 0.06f     /* how long the bright '*+' cross shows */
#define BLAST_LIFE_BASE 0.50f     /* how long a spark lives, before jitter */
#define BLAST_LIFE_JITTER 0.25f   /* random extra life added on top, for variety */
#define BLAST_SPEED_MIN 18.0f     /* slowest a fresh spark can fly */
#define BLAST_SPEED_MAX 42.0f     /* fastest a fresh spark can fly */
#define BLAST_DRAG_PER_SEC 3.0f   /* how fast sparks slow down (about 95%/sec) */
#define BLAST_WAVE_COUNT 4        /* sparks go out in this many ripples */
#define BLAST_MAX_DELAY_SEC 0.10f /* head start of the first ripple over the last */

typedef enum {
  PATTERN_SHOOTING_STAR = 0,
  PATTERN_FIREBALL = 1,
  PATTERN_PLASMA_BOLT = 2,
  N_PATTERNS = 3,
} Pattern;

/* The labels are padded to the same width so the HUD doesn't jump around. */
static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_SHOOTING_STAR:
    return "SHOOTING_STAR";
  case PATTERN_FIREBALL:
    return "FIREBALL     ";
  case PATTERN_PLASMA_BOLT:
    return "PLASMA_BOLT  ";
  default:
    return "?            ";
  }
}

/*
 * PatternParams — the recipe for one comet "look". One generic engine reads
 * the row for the active look and behaves accordingly, so adding a new comet
 * type means adding a row here, not new code. The three rows below are
 * SHOOTING_STAR (fast, tight), FIREBALL (slow, puffy), PLASMA_BOLT (fast,
 * wobbling).
 *
 *   max_comets           how many of this comet may fly at once
 *   speed                how fast it travels, in cells per second
 *   speed_jitter         random wiggle on the speed, as a fraction (0.2 = ±10%)
 *   angular_kick         tiny random steering each tick, in radians (0 = straight)
 *   emit_rate            trail dots dropped per second
 *   particle_life        how long a trail dot lives, in seconds
 *   particle_spread      how far sideways a dot can land from the comet, in cells
 *   spread_drift_factor  how much a dot drifts outward as it fades (0 = sits still)
 *   trail_drag           how fast trail dots slow their drift, per second
 *   head_glyph           the character drawn at the comet's head
 */
typedef struct {
  int max_comets;
  float speed;
  float speed_jitter;
  float angular_kick;
  float emit_rate;
  float particle_life;
  float particle_spread;
  float spread_drift_factor;
  float trail_drag;
  char head_glyph;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                       cnt  spd  jit  ang   emit  life  spread sdrift drag
       head */
    /* SHOOTING_STAR    */ {1, 150.0f, 0.20f, 0.00f, 90.0f, 0.40f, 0.4f, 0.0f,
                            0.5f, '*'},
    /* FIREBALL         */
    {1, 55.0f, 0.20f, 0.00f, 180.0f, 1.20f, 1.5f, 6.0f, 1.0f, 'O'},
    /* PLASMA_BOLT      */
    {2, 130.0f, 0.30f, 0.10f, 150.0f, 0.80f, 1.0f, 3.0f, 1.5f, '*'},
};

/*
 * Theme — one named colour scheme. Themes only change which 256-colour codes
 * the pairs point at, so flipping a theme recolours everything instantly with
 * no redraw.
 *
 *   name   shown in the HUD
 *   ramp   the 8 trail shades, from a faded dying dot (slot 0) to a fresh
 *          bright one (slot 7)
 *   head   the bright colour of the comet's head
 *   halo   the softer colour of the glow around the head
 *   sky    a background tint; kept for completeness but not drawn right now
 */
typedef struct {
  const char *name;
  short ramp[8];
  short head;
  short halo;
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]                                       head halo
       sky */

    {"DEFAULT", {110, 117, 153, 159, 195, 195, 231, 255}, 231, 195, 234},
    {"ICE", {24, 31, 67, 110, 117, 153, 195, 231}, 231, 195, 235},
    {"FIRE", {88, 124, 130, 166, 196, 208, 214, 226}, 226, 220, 234},
    {"PLASMA", {53, 91, 134, 165, 207, 213, 219, 225}, 225, 219, 234},
    {"GOLD", {130, 137, 173, 179, 215, 222, 229, 230}, 230, 222, 234},
    {"GREEN", {28, 34, 40, 64, 70, 112, 156, 192}, 192, 156, 234},
    {"AURORA", {43, 79, 115, 121, 157, 195, 230, 231}, 231, 195, 234},
    {"ROSE", {88, 131, 167, 174, 211, 217, 218, 231}, 231, 218, 234},
    {"MONO", {244, 246, 248, 250, 252, 253, 254, 255}, 255, 252, 232},
    {"VIOLET", {53, 54, 91, 134, 135, 176, 213, 219}, 225, 219, 233},
};

/* The character used for a trail dot, from faintest (dying) to boldest (fresh). */
static const char RAMP_GLYPHS[8] = {'`', '.', ',', ':', ';', '-', '+', '*'};

/* ── §2 clock ── */

/* A steady clock (never jumps backward), counted in nanoseconds. */
static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep for this many nanoseconds. Asking for zero or less just returns. */
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

/* Point all the colour pairs at the chosen theme's colours. Falls back to
 * plain white when the terminal can't do 256 colours. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_HEAD, t->head, -1);
    init_pair(PAIR_HALO, t->halo, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), COLOR_WHITE, -1);
    init_pair(PAIR_HEAD, COLOR_WHITE, -1);
    init_pair(PAIR_HALO, COLOR_WHITE, -1);
    init_pair(PAIR_SKY, COLOR_BLACK, -1);
  }
}

/* Turn colour on, set up the fixed HUD/HINT pairs, and load the first theme. */
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

/* ── §4 comet ── */

/*
 * Comet — the flying head that leaves a trail. It stores no glyph or colour;
 * those come from the current look and theme at draw time, so changing either
 * recolours a comet already in flight, no respawn needed.
 *
 *   x, y         position in cells; kept as floats so it can move smoothly
 *   vx, vy       velocity in cells per second
 *   emit_carry   leftover fraction of a trail dot owed from last tick (so a
 *                rate like 90.5 dots/sec doesn't lose the half each frame)
 *   age          seconds alive; only used by the HUD, not for dying
 *   active       is this pool slot in use?
 *
 * Life story: born in scene_spawn_comet (startup, 'r', or topping up the pool)
 * → moved and aged each tick → marked inactive when it hits the floor or
 * drifts off the screen.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float emit_carry;
  float age;
  bool active;
} Comet;

/* A tiny, fast random-number generator (the classic one from Numerical
 * Recipes). Each call scrambles the state and hands back the new value. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}

/* A random float from 0 up to (but not including) 1. */
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* A random float from -0.5 up to 0.5 — handy for "wiggle either way". */
static inline float lcg_signed(uint32_t *rng) { return lcg_unit(rng) - 0.5f; }
/* A random float somewhere between lo and hi. */
static inline float lcg_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}
/* Pin a value so it can't go below lo or above hi. */
static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Turn a freshness of 0..1 into one of the 8 shade slots. We multiply by
 * 7.999 rather than 8 so a perfectly fresh dot lands in slot 7, not slot 8. */
static inline int ramp_slot_from_freshness(float freshness_0_to_1) {
  float clamped = clampf(freshness_0_to_1, 0.0f, 1.0f);
  int bucket = (int)(clamped * 7.999f);
  return clamp_int(bucket, 0, 7);
}

/* Snap a smooth float position to the nearest whole character cell. */
static inline int round_to_cell(float v) { return (int)(v + 0.5f); }

/* Is this cell actually on screen (and above the HUD row)? */
static inline bool cell_visible(int cell_x, int cell_y, int cols,
                                int rows_playable) {
  return cell_x >= 0 && cell_x < cols && cell_y >= 0 && cell_y < rows_playable;
}

/* How fresh a trail dot is: 1 when newborn, fading evenly to 0 at death. */
static inline float trail_freshness(float age, float life) {
  if (life <= 0.0f)
    return 0.0f;
  return 1.0f - age / life;
}

/* Same idea for a spark, whose life counts down: 1 when newborn, 0 at death. */
static inline float blast_freshness(float remaining_life, float max_life) {
  if (max_life <= 0.0f)
    return 0.0f;
  return remaining_life / max_life;
}

/* Brightest trail shades draw bold, faintest draw dim, the rest plain. */
static inline int trail_attr_for_slot(int ramp_slot) {
  if (ramp_slot >= 6)
    return A_BOLD;
  if (ramp_slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Sparks get bold or plain, never dim: a dim spark just looks like a smudge,
 * so faint ones simply disappear instead of fading out. */
static inline int blast_attr_for_slot(int ramp_slot) {
  return (ramp_slot >= 6) ? A_BOLD : A_NORMAL;
}

/* Draw one character with a colour and style, turning the style on and off
 * around it so every caller stays a single line. */
static inline void paint_cell(int cell_y, int cell_x, char glyph,
                              int color_pair, int attr) {
  attron(COLOR_PAIR(color_pair) | attr);
  mvaddch(cell_y, cell_x, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(color_pair) | attr);
}

/* ── §5 trail + blast particles ── */

/*
 * TrailParticle — one fading dot the comet left behind.
 *
 *   x, y      where it sits on screen, in cells (its own spot, not the comet's)
 *   vx, vy    its own slow drift, in cells per second — deliberately NOT the
 *             comet's velocity, which is exactly why the trail lags behind
 *   age, life age counts up; the dot dies once its age reaches its life
 *   active    is this pool slot in use?
 *
 * Life story: dropped by scene_emit_trail as the comet flies → drifts and
 * ages each tick → marked inactive when its age reaches its life.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age, life;
  bool active;
} TrailParticle;

/*
 * BlastParticle — one spark flung out by an explosion.
 *
 *   rx, ry    position measured from the blast's centre, not the screen origin
 *   vx, vy    velocity in cells per second
 *   life      seconds of life remaining; counts down to zero
 *   max_life  the life it started with, so we can tell how fresh it still is
 *   delay     a short hold before this spark starts moving, so the sparks go
 *             out in ripples instead of all at once; counts down to zero
 *   sym       its character, picked once at birth from "*+.,oO!#"
 *   alive     is this pool slot in use?
 *
 * Positions are stored relative to the blast's centre so all 32 sparks share
 * one centre, which keeps the on-screen clipping in one place (the draw step).
 */
typedef struct {
  float rx, ry;
  float vx, vy;
  float life;
  float max_life;
  float delay;
  char sym;
  bool alive;
} BlastParticle;

/*
 * Blast — one explosion: a brief bright cross plus a fan of 32 sparks.
 *
 *   cx, cy                where the comet hit, in cells
 *   flash_ttl             seconds left to show the bright '*+' cross; 0 = gone
 *   active                is this pool slot in use?
 *   parts[BLAST_PARTICLES] this blast's own 32 sparks, stored inline
 *
 * Life story: created by blast_ignite the moment a comet reaches the floor →
 * advanced each tick → marked inactive once the cross is gone AND every spark
 * has died.
 */
typedef struct {
  float cx, cy;
  float flash_ttl;
  bool active;
  BlastParticle parts[BLAST_PARTICLES];
} Blast;

/* ── §6 scene — pools, tick, draw ── */

/*
 * Scene — the whole running simulation in one struct: three fixed-size pools
 * of objects plus the live settings. Everything is allocated up front; nothing
 * is allocated while it runs.
 *
 *   paused           when true, time stops (nothing moves)
 *   speed            time dial set by +/-; SPEED_DEF means normal speed
 *   current_theme    which colour theme is showing (flipped by t/T)
 *   current_pattern  which comet look is active (flipped by n/p)
 *   rng              the random-number state, the one source of all randomness
 *   rows, cols       terminal size in cells
 *   comets           the comet pool (up to MAX_COMETS in flight)
 *   trail            the trail-dot pool (up to MAX_TRAIL)
 *   blasts           the explosion pool (one slot per possible comet)
 *
 * Life story: set up once by scene_init → advanced and drawn every frame →
 * told the new size on a terminal resize → cleared and reseeded on 'r'. To
 * find a free slot we just scan the pool; at these small sizes that's faster
 * than keeping a list of free slots.
 */
typedef struct {
  bool paused;
  int speed;
  int current_theme;
  Pattern current_pattern;
  uint32_t rng;
  int rows, cols;

  Comet comets[MAX_COMETS];
  TrailParticle trail[MAX_TRAIL];
  Blast blasts[MAX_BLASTS];
} Scene;

/* Find a free slot in each pool, or return -1 when the pool is full. */
static int comet_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_COMETS; i++)
    if (!s->comets[i].active)
      return i;
  return -1;
}

static int trail_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_TRAIL; i++)
    if (!s->trail[i].active)
      return i;
  return -1;
}

static int blast_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_BLASTS; i++)
    if (!s->blasts[i].active)
      return i;
  return -1;
}

/* Empty all three pools by marking every slot free. */
static void scene_clear_pools(Scene *s) {
  for (int i = 0; i < MAX_COMETS; i++)
    s->comets[i].active = false;
  for (int i = 0; i < MAX_TRAIL; i++)
    s->trail[i].active = false;
  for (int i = 0; i < MAX_BLASTS; i++)
    s->blasts[i].active = false;
}

/* Pick the direction for spark number i. The angle is spread across the upper
 * half-circle so sparks fly up and sideways, never down through the floor. */
static float blast_emission_angle(int i, int total, uint32_t *rng) {
  float evenly_spaced = (float)M_PI + ((float)i / (float)total) * (float)M_PI;
  float jitter = lcg_signed(rng) * 0.2f;
  return evenly_spaced + jitter;
}

/* Pick a random launch speed for a spark. */
static float blast_emission_speed(uint32_t *rng) {
  return lcg_range(rng, BLAST_SPEED_MIN, BLAST_SPEED_MAX);
}

/* How long this ripple waits before its sparks start moving, so the explosion
 * reads as expanding rings rather than one instant flash. */
static float blast_wave_delay(int wave, int wave_count, float max_delay_sec) {
  if (wave_count <= 1)
    return 0.0f;
  return (float)wave * (max_delay_sec / (float)(wave_count - 1));
}

/* Set up one spark: it starts at the blast centre, flies off in the given
 * direction at the given speed, and gets a random life and character. */
static void blast_spawn_one_spark(BlastParticle *p, float angle, float speed,
                                  float delay_sec, uint32_t *rng) {
  static const char k_blast_syms[] = "*+.,oO!#";
  static const int n_syms = (int)sizeof k_blast_syms - 1;

  p->rx = 0.0f;
  p->ry = 0.0f;
  p->vx = cosf(angle) * speed;
  p->vy = sinf(angle) * speed;
  p->max_life =
      lcg_range(rng, BLAST_LIFE_BASE, BLAST_LIFE_BASE + BLAST_LIFE_JITTER);
  p->life = p->max_life;
  p->delay = delay_sec;
  p->sym = k_blast_syms[lcg_next(rng) % (unsigned)n_syms];
  p->alive = true;
}

/*
 * Start one explosion at the spot a comet hit. Claims a free blast slot, sets
 * its centre and bright cross, then fills it with 32 sparks fanned across the
 * upper half-circle and split into a few timed ripples.
 *
 * Sparks only aim upward and sideways: on screen, down is the +y direction, so
 * a downward spark would instantly fall off the bottom — pointless. The ripple
 * timing turns one flat ring into a wave of expanding rings, which looks more
 * like a real shockwave.
 *
 * Takes the impact centre (cx, cy) in cells; cy is the floor row.
 */
static void blast_ignite(Scene *s, float cx, float cy) {
  /* Grab a free explosion slot; if every one is busy, skip this blast. */
  int slot_index = blast_pool_find_inactive(s);
  if (slot_index < 0)
    return;
  Blast *b = &s->blasts[slot_index];

  /* Place the centre and start the bright cross. */
  b->cx = cx;
  b->cy = cy;
  b->flash_ttl = BLAST_FLASH_SEC;
  b->active = true;

  /* Throw the spark fan, each spark assigned to one of the timed ripples. */
  for (int i = 0; i < BLAST_PARTICLES; i++) {
    float angle = blast_emission_angle(i, BLAST_PARTICLES, &s->rng);
    float speed = blast_emission_speed(&s->rng);
    int wave_index = i % BLAST_WAVE_COUNT;
    float delay_sec =
        blast_wave_delay(wave_index, BLAST_WAVE_COUNT, BLAST_MAX_DELAY_SEC);
    blast_spawn_one_spark(&b->parts[i], angle, speed, delay_sec, &s->rng);
  }
}

/* Which side of the screen a new comet flies in from. */
typedef enum {
  EDGE_TOP = 0,
  EDGE_LEFT = 1,
  EDGE_RIGHT = 2,
  EDGE_BOTTOM = 3,
} SpawnEdge;

/* Pick an entry edge, weighted so most comets fall from the top: top 45%,
 * left 25%, right 25%, bottom only 5%. */
static SpawnEdge pick_spawn_edge(uint32_t *rng) {
  float u = lcg_unit(rng);
  if (u < 0.45f)
    return EDGE_TOP;
  if (u < 0.70f)
    return EDGE_LEFT;
  if (u < 0.95f)
    return EDGE_RIGHT;
  return EDGE_BOTTOM;
}

/* Pick the comet's start point, placed just past the chosen edge so it looks
 * like it's flying in from off screen. */
static void compute_spawn_point(SpawnEdge edge, int cols, int rows,
                                uint32_t *rng, float *spawn_x, float *spawn_y) {
  float along_edge = lcg_unit(rng);
  switch (edge) {
  case EDGE_TOP:
    *spawn_x = along_edge * (float)cols;
    *spawn_y = -EDGE_MARGIN;
    break;
  case EDGE_LEFT:
    *spawn_x = -EDGE_MARGIN;
    *spawn_y = along_edge * (float)rows;
    break;
  case EDGE_RIGHT:
    *spawn_x = (float)cols + EDGE_MARGIN;
    *spawn_y = along_edge * (float)rows;
    break;
  case EDGE_BOTTOM:
    *spawn_x = along_edge * (float)cols;
    *spawn_y = (float)rows + EDGE_MARGIN;
    break;
  }
}

/* Pick a point to aim at, over on the far side of the screen, so the comet
 * cuts a clear diagonal across instead of nicking the corner it entered from. */
static void compute_target_point(SpawnEdge edge, int cols, int rows,
                                 uint32_t *rng, float *target_x,
                                 float *target_y) {
  float r_a = lcg_unit(rng);
  float r_b = lcg_unit(rng);
  switch (edge) {
  case EDGE_TOP:
    *target_x = r_a * (float)cols;
    *target_y = (float)rows * (0.55f + r_b * 0.45f);
    break;
  case EDGE_LEFT:
    *target_x = (float)cols * (0.55f + r_a * 0.45f);
    *target_y = r_b * (float)rows;
    break;
  case EDGE_RIGHT:
    *target_x = (float)cols * (0.00f + r_a * 0.45f);
    *target_y = r_b * (float)rows;
    break;
  case EDGE_BOTTOM:
    *target_x = r_a * (float)cols;
    *target_y = (float)rows * (0.00f + r_b * 0.45f);
    break;
  }
}

/* Take the look's base speed and nudge it up or down a little at random, so
 * the comets don't all move in lockstep. */
static float compute_speed_with_jitter(const PatternParams *pp, uint32_t *rng) {
  float jitter_lo = 1.0f - pp->speed_jitter * 0.5f;
  float jitter_hi = jitter_lo + pp->speed_jitter;
  return pp->speed * lcg_range(rng, jitter_lo, jitter_hi);
}

/*
 * Add one new comet to the pool: pick an entry edge, a start point just past
 * it, and a target on the far side, then aim the comet from start to target at
 * a slightly randomised speed.
 *
 * Aiming across the screen is the point — a purely random direction would,
 * half the time, send the comet straight back out the edge it came in on, so
 * you'd never see it. Does nothing if every comet slot is already in use.
 */
static void scene_spawn_comet(Scene *s) {
  /* Grab a free comet slot, or give up if the pool is full. */
  int slot_index = comet_pool_find_inactive(s);
  if (slot_index < 0)
    return;
  Comet *c = &s->comets[slot_index];
  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* Pick which edge it enters from. */
  SpawnEdge edge = pick_spawn_edge(&s->rng);

  /* Its start point, just outside that edge. */
  float spawn_x, spawn_y;
  compute_spawn_point(edge, s->cols, s->rows, &s->rng, &spawn_x, &spawn_y);

  /* A point on the far side to head toward. */
  float target_x, target_y;
  compute_target_point(edge, s->cols, s->rows, &s->rng, &target_x, &target_y);

  /* Turn "start to target" into a direction, then scale it to the speed.
   * The tiny-distance guard avoids dividing by zero if the two points land
   * on top of each other. */
  float delta_x = target_x - spawn_x;
  float delta_y = target_y - spawn_y;
  float distance = sqrtf(delta_x * delta_x + delta_y * delta_y);
  if (distance < 1e-3f)
    distance = 1.0f;
  float speed = compute_speed_with_jitter(pp, &s->rng);

  /* Fill in the comet. */
  c->x = spawn_x;
  c->y = spawn_y;
  c->vx = speed * delta_x / distance;
  c->vy = speed * delta_y / distance;
  c->emit_carry = 0.0f;
  c->age = 0.0f;
  c->active = true;
}

/*
 * Drop one trail dot at the comet's current spot. This is the heart of the
 * whole effect.
 *
 * The dot lands a little to the side of the comet (a random nudge across the
 * comet's flight direction), and it's given only a slow drift of its own — NOT
 * the comet's speed. That's the key: the comet races on while the dot barely
 * moves, so the dot is left behind and the line of dots reads as a streak. If
 * you instead handed the dot the comet's velocity, it would ride along and
 * there'd be no streak at all.
 *
 * Reads the emitting comet, writes one trail dot into the pool. Does nothing
 * if the trail pool is full.
 */
static void scene_emit_trail(Scene *s, const Comet *c) {
  /* Grab a free trail slot, or give up if the pool is full. */
  int slot_index = trail_pool_find_inactive(s);
  if (slot_index < 0)
    return;
  TrailParticle *p = &s->trail[slot_index];
  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* A unit-length direction at right angles to the comet's flight, so we can
   * nudge the dot sideways. The tiny-speed guard keeps the division safe if
   * the comet is momentarily near-still. */
  float comet_speed = sqrtf(c->vx * c->vx + c->vy * c->vy);
  if (comet_speed < 1e-3f)
    comet_speed = 1.0f;
  float perp_x = -c->vy / comet_speed;
  float perp_y = c->vx / comet_speed;

  /* How far sideways this dot lands, random within the look's spread. */
  float kick_magnitude = lcg_signed(&s->rng) * 2.0f * pp->particle_spread;

  /* Turn that into an actual sideways offset. */
  float offset_x = perp_x * kick_magnitude;
  float offset_y = perp_y * kick_magnitude;

  /* The dot's own slow drift. Zero for SHOOTING_STAR (a tight line); larger
   * for FIREBALL/PLASMA_BOLT, so their dots puff outward as they fade. */
  float drift_vx = offset_x * pp->spread_drift_factor;
  float drift_vy = offset_y * pp->spread_drift_factor;

  /* Give each dot a slightly different lifespan (about 0.7x to 1.3x the
   * look's base) so the trail doesn't vanish all at once. */
  float life_factor = lcg_range(&s->rng, 0.7f, 1.3f);
  float life_sec = pp->particle_life * life_factor;

  /* Fill in the dot. */
  p->x = c->x + offset_x;
  p->y = c->y + offset_y;
  p->vx = drift_vx;
  p->vy = drift_vy;
  p->age = 0.0f;
  p->life = life_sec;
  p->active = true;
}

/* Wipe the scene, set the starting options, and launch one comet so the
 * screen isn't blank at startup. */
static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_SHOOTING_STAR;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_pools(s);
  scene_spawn_comet(s);
}

/* Remember the new terminal size. The pools are fixed-size, so there's
 * nothing else to do on a resize. */
static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

/* The 'r' key: clear everything, freshen the random seed, launch one comet. */
static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  scene_clear_pools(s);
  scene_spawn_comet(s);
}

/* How many comets are in flight right now. */
static int count_active_comets(const Scene *s) {
  int active = 0;
  for (int i = 0; i < MAX_COMETS; i++)
    if (s->comets[i].active)
      active++;
  return active;
}

/* Phase 1: spawn comets until we're back up to the look's allowed count. */
static void phase1_topup_comet_pool(Scene *s, const PatternParams *pp) {
  int active_now = count_active_comets(s);
  int target_count =
      (pp->max_comets > MAX_COMETS) ? MAX_COMETS : pp->max_comets;
  for (int k = active_now; k < target_count; k++)
    scene_spawn_comet(s);
}

/* Steer the comet a random tiny amount left or right. We rotate the velocity
 * rather than just adding to it, so the comet keeps the same speed and only
 * changes direction. Only PLASMA_BOLT uses this; the others pass 0 and skip. */
static void comet_apply_plasma_kick(Comet *c, uint32_t *rng,
                                    float kick_strength) {
  if (kick_strength <= 0.0f)
    return;

  float angle = lcg_signed(rng) * 2.0f * kick_strength;
  float cos_a = cosf(angle);
  float sin_a = sinf(angle);
  float new_vx = cos_a * c->vx - sin_a * c->vy;
  float new_vy = sin_a * c->vx + cos_a * c->vy;
  c->vx = new_vx;
  c->vy = new_vy;
}

/* Did this comet just reach the floor heading down? If so, set off an
 * explosion there, retire the comet, and report true. */
static bool comet_hit_ground(Scene *s, Comet *c) {
  float ground_y = (float)s->rows - 2.0f;
  if (!(c->vy > 0.0f && c->y >= ground_y))
    return false;

  float impact_x = clampf(c->x, 0.0f, (float)s->cols - 1.0f);
  blast_ignite(s, impact_x, ground_y);
  c->active = false;
  return true;
}

/* Has this comet drifted past any edge (with a little slack)? If so, retire it
 * and report true. */
static bool comet_off_screen(Scene *s, Comet *c) {
  bool off_left = (c->x < -EDGE_MARGIN);
  bool off_right = (c->x > (float)s->cols + EDGE_MARGIN);
  bool off_top = (c->y < -EDGE_MARGIN);
  bool off_below = (c->y > (float)s->rows + EDGE_MARGIN);
  if (off_left || off_right || off_top || off_below) {
    c->active = false;
    return true;
  }
  return false;
}

/* Decide how many trail dots to drop this frame. We add up the dots owed
 * (rate times elapsed time), drop the whole ones now, and carry the leftover
 * fraction to next frame so nothing is lost to rounding. */
static void comet_emit_trail_particles(Scene *s, Comet *c, float dt,
                                       const PatternParams *pp) {
  c->emit_carry += pp->emit_rate * dt;
  int emit_count = (int)c->emit_carry;
  c->emit_carry -= (float)emit_count;
  for (int k = 0; k < emit_count; k++)
    scene_emit_trail(s, c);
}

/* Move one comet through a single frame: maybe steer it, slide it forward,
 * check whether it died, and if it's still alive, drop its trail dots. We
 * check for a floor hit before going off-screen so an explosion still fires. */
static void phase2_advance_one_comet(Scene *s, Comet *c, float dt,
                                     const PatternParams *pp) {
  comet_apply_plasma_kick(c, &s->rng, pp->angular_kick);

  c->x += c->vx * dt;
  c->y += c->vy * dt;
  c->age += dt;

  if (comet_hit_ground(s, c))
    return;
  if (comet_off_screen(s, c))
    return;

  comet_emit_trail_particles(s, c, dt, pp);
}

/* Phase 2: advance every comet that's currently in flight. */
static void phase2_advance_all_comets(Scene *s, float dt,
                                      const PatternParams *pp) {
  for (int i = 0; i < MAX_COMETS; i++) {
    Comet *c = &s->comets[i];
    if (!c->active)
      continue;
    phase2_advance_one_comet(s, c, dt, pp);
  }
}

/* Phase 3: move and age every trail dot. Each dot drifts a bit, slows down,
 * gets older, and is retired once it's lived out its lifespan. */
static void phase3_advance_trail(Scene *s, float dt, const PatternParams *pp) {
  float drag_factor = expf(-pp->trail_drag * dt);

  for (int i = 0; i < MAX_TRAIL; i++) {
    TrailParticle *p = &s->trail[i];
    if (!p->active)
      continue;

    p->x += p->vx * dt;
    p->y += p->vy * dt;
    p->vx *= drag_factor;
    p->vy *= drag_factor;
    p->age += dt;
    if (p->age >= p->life)
      p->active = false;
  }
}

/* Move one spark through a frame. Returns true if it's still alive afterward.
 * A spark that's still waiting for its ripple just holds at the centre; once
 * released it slows down, drifts, ages, and dies when its life runs out or it
 * leaves the screen. */
static bool blast_particle_tick_one(BlastParticle *p, const Blast *b,
                                    float drag_factor, float dt, int cols,
                                    int rows) {
  if (!p->alive)
    return false;

  /* Still waiting for its ripple's turn — sit tight at the centre. */
  if (p->delay > 0.0f) {
    p->delay -= dt;
    return true;
  }

  /* Slow it down a touch. */
  p->vx *= drag_factor;
  p->vy *= drag_factor;

  /* Slide it along by its velocity. */
  p->rx += p->vx * dt;
  p->ry += p->vy * dt;

  /* Count down its remaining life. */
  p->life -= dt;

  /* Die if life ran out or it left the screen. */
  float screen_x = b->cx + p->rx;
  float screen_y = b->cy + p->ry;
  bool burned_out = (p->life <= 0.0f);
  bool off_screen = (screen_x < 0.0f || screen_x >= (float)cols ||
                     screen_y < 0.0f || screen_y >= (float)(rows - 1));
  if (burned_out || off_screen) {
    p->alive = false;
    return false;
  }
  return true;
}

/* Advance one explosion: count down its bright cross and tick all its sparks.
 * Returns true while the blast still has something to show (the cross is up or
 * at least one spark is alive). */
static bool phase4_advance_one_blast(Blast *b, float drag_factor, float dt,
                                     int cols, int rows) {
  if (b->flash_ttl > 0.0f)
    b->flash_ttl -= dt;

  bool any_alive = false;
  for (int j = 0; j < BLAST_PARTICLES; j++) {
    if (blast_particle_tick_one(&b->parts[j], b, drag_factor, dt, cols, rows))
      any_alive = true;
  }

  bool flash_done = (b->flash_ttl <= 0.0f);
  return any_alive || !flash_done;
}

/* Phase 4: advance every active explosion. We work out the slow-down factor
 * once and reuse it for all of them. */
static void phase4_advance_all_blasts(Scene *s, float dt) {
  float drag_factor = expf(-BLAST_DRAG_PER_SEC * dt);

  for (int i = 0; i < MAX_BLASTS; i++) {
    Blast *b = &s->blasts[i];
    if (!b->active)
      continue;
    if (!phase4_advance_one_blast(b, drag_factor, dt, s->cols, s->rows))
      b->active = false;
  }
}

/*
 * Advance the whole simulation by one step, in four phases: spawn comets, move
 * comets (which drop trail dots), age the trail, then update explosions. The
 * speed dial scales the time step first, so one knob speeds up or slows down
 * everything at once. Does nothing while paused.
 *
 * The order matters: comets must exist before we move them, and they drop
 * their trail dots as they move, so the trail must be aged after that. The
 * explosions don't depend on any of it, so they go last.
 */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float global_speed_mul = (float)s->speed / (float)SPEED_DEF;
  dt *= global_speed_mul;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  phase1_topup_comet_pool(s, pp);
  phase2_advance_all_comets(s, dt, pp);
  phase3_advance_trail(s, dt, pp);
  phase4_advance_all_blasts(s, dt);
}

/* Draw one trail dot: find its cell, pick the character and shade from how
 * fresh it still is, and paint it. */
static void draw_one_trail_particle(const TrailParticle *p, int cols,
                                    int rows_playable) {
  int cell_x = round_to_cell(p->x);
  int cell_y = round_to_cell(p->y);
  if (!cell_visible(cell_x, cell_y, cols, rows_playable))
    return;

  float freshness = trail_freshness(p->age, p->life);
  int ramp_slot = ramp_slot_from_freshness(freshness);
  char glyph = RAMP_GLYPHS[ramp_slot];
  int color_pair = PAIR_RAMP_BASE + ramp_slot;
  int attr = trail_attr_for_slot(ramp_slot);

  paint_cell(cell_y, cell_x, glyph, color_pair, attr);
}

/* Bottom layer: paint all the trail dots. */
static void scene_draw_trail_layer(const Scene *s, int rows_playable) {
  for (int i = 0; i < MAX_TRAIL; i++) {
    const TrailParticle *p = &s->trail[i];
    if (!p->active)
      continue;
    draw_one_trail_particle(p, s->cols, rows_playable);
  }
}

/* Draw the explosion's bright cross: a '*' at the centre with '+' to the left,
 * right, and above. Nothing below, since that's into the floor. */
static void draw_blast_flash(const Blast *b, int cols, int rows_playable) {
  if (b->flash_ttl <= 0.0f)
    return;

  int cx = round_to_cell(b->cx);
  int cy = round_to_cell(b->cy);
  if (!cell_visible(cx, cy, cols, rows_playable))
    return;

  paint_cell(cy, cx, '*', PAIR_HEAD, A_BOLD);
  if (cx > 0)
    paint_cell(cy, cx - 1, '+', PAIR_HEAD, A_BOLD);
  if (cx < cols - 1)
    paint_cell(cy, cx + 1, '+', PAIR_HEAD, A_BOLD);
  if (cy > 0)
    paint_cell(cy - 1, cx, '+', PAIR_HEAD, A_BOLD);
}

/* Draw one spark, if it's alive and past its ripple delay. Its offset is added
 * to the blast centre to find its cell, and its shade comes from how fresh it
 * still is. */
static void draw_one_blast_spark(const Blast *b, const BlastParticle *p,
                                 int cols, int rows_playable) {
  if (!p->alive || p->delay > 0.0f)
    return;

  int cell_x = round_to_cell(b->cx + p->rx);
  int cell_y = round_to_cell(b->cy + p->ry);
  if (!cell_visible(cell_x, cell_y, cols, rows_playable))
    return;

  float freshness = blast_freshness(p->life, p->max_life);
  int ramp_slot = ramp_slot_from_freshness(freshness);
  int color_pair = PAIR_RAMP_BASE + ramp_slot;
  int attr = blast_attr_for_slot(ramp_slot);

  paint_cell(cell_y, cell_x, p->sym, color_pair, attr);
}

/* Draw one whole explosion: its bright cross, then all of its sparks. */
static void draw_one_blast(const Blast *b, int cols, int rows_playable) {
  draw_blast_flash(b, cols, rows_playable);
  for (int j = 0; j < BLAST_PARTICLES; j++)
    draw_one_blast_spark(b, &b->parts[j], cols, rows_playable);
}

/* Middle layer: paint all the explosions. */
static void scene_draw_blast_layer(const Scene *s, int rows_playable) {
  for (int i = 0; i < MAX_BLASTS; i++) {
    const Blast *b = &s->blasts[i];
    if (!b->active)
      continue;
    draw_one_blast(b, s->cols, rows_playable);
  }
}

/* Draw the glow around the comet head: the 8 surrounding cells, with a bold
 * '+' straight up/down/left/right and a dim ':' on the diagonals. */
static void draw_comet_halo(int head_x, int head_y, int cols,
                            int rows_playable) {
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0)
        continue;

      int cell_x = head_x + dx;
      int cell_y = head_y + dy;
      if (!cell_visible(cell_x, cell_y, cols, rows_playable))
        continue;

      bool is_cardinal = (dx == 0 || dy == 0);
      char glyph = is_cardinal ? '+' : ':';
      int attr = is_cardinal ? A_BOLD : A_DIM;

      paint_cell(cell_y, cell_x, glyph, PAIR_HALO, attr);
    }
  }
}

/* Draw one comet: the glow first, then its head on top. */
static void draw_one_comet(const Comet *c, char head_glyph, int cols,
                           int rows_playable) {
  int head_x = round_to_cell(c->x);
  int head_y = round_to_cell(c->y);

  draw_comet_halo(head_x, head_y, cols, rows_playable);

  if (!cell_visible(head_x, head_y, cols, rows_playable))
    return;
  paint_cell(head_y, head_x, head_glyph, PAIR_HEAD, A_BOLD);
}

/* Top layer: paint all the comets. */
static void scene_draw_comet_layer(const Scene *s, int rows_playable) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  for (int i = 0; i < MAX_COMETS; i++) {
    const Comet *c = &s->comets[i];
    if (!c->active)
      continue;
    draw_one_comet(c, pp->head_glyph, s->cols, rows_playable);
  }
}

/*
 * Draw the whole scene back to front: trails first, then explosions, then
 * comets on top. We paint in that order so that wherever things land on the
 * same cell, the comet (the "now") wins over the explosion, which wins over
 * the old trail. The bottom screen row is left for the HUD.
 */
static void scene_draw(const Scene *s) {
  int rows_playable = s->rows - 1;
  scene_draw_trail_layer(s, rows_playable);
  scene_draw_blast_layer(s, rows_playable);
  scene_draw_comet_layer(s, rows_playable);
}

/* ── §7 screen ── */

/* The terminal's current size in characters. */
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

static void scene_counts(const Scene *s, int *out_comets, int *out_trail) {
  int c = 0, t = 0;
  for (int i = 0; i < MAX_COMETS; i++)
    if (s->comets[i].active)
      c++;
  for (int i = 0; i < MAX_TRAIL; i++)
    if (s->trail[i].active)
      t++;
  *out_comets = c;
  *out_trail = t;
}

static inline const char *hud_status_label(const Scene *s) {
  return s->paused ? "PAUSED       " : pattern_name(s->current_pattern);
}

static void format_hud_text(char *buf, size_t n, const Scene *s, double fps,
                            int sim_fps) {
  int comets, trails;
  scene_counts(s, &comets, &trails);

  snprintf(buf, n,
           " COMET   %s   theme:%-8s   comets:%d  trail:%4d   "
           "%5.1f fps  %3d Hz  speed:%-3d   "
           "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
           hud_status_label(s), themes[s->current_theme].name, comets, trails,
           fps, sim_fps, s->speed);
}

static void paint_hud_bar(int row, int cols, const char *text) {
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < cols; x++)
    mvaddch(row, x, ' ');
  mvprintw(row, 0, "%s", text);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  char hud_text[200];
  format_hud_text(hud_text, sizeof hud_text, s, fps, sim_fps);
  paint_hud_bar(sc->rows - 1, sc->cols, hud_text);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

/*
 * App — ties everything together: the simulation, the terminal size, the
 * chosen sim rate, and two flags the signal handlers flip.
 *
 *   sim_fps      how many simulation steps per second we aim for ([ and ])
 *   running      cleared to stop the main loop (also set by Ctrl-C / kill)
 *   need_resize  set when the terminal was resized, handled next loop pass
 *
 * The two flags are sig_atomic_t because they're written from signal handlers,
 * which can fire between any two instructions.
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

static void app_install_signals(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  app_install_signals();

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
