/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vortex.c — specks spiral inward toward a drain in the middle of the
 * screen, speeding up as they fall.
 *
 * Each speck is tracked by how far it is from the centre and what angle
 * it sits at (not by an x/y position). Every step we pull it a little
 * closer to the centre and turn it a little around — and a spiral falls
 * out of that on its own. One engine drives ten looks (whirlpool, tornado,
 * black hole, ...); they differ only in the numbers in pattern_params[].
 *
 * Sister files: physics/blackhole.c (the real-gravity raymarched version),
 * procedural/worldgen/procedural_galaxy.c (spiral arms drawn as a formula
 * instead of one speck at a time), and rain.c / snow.c / fountain.c /
 * embers.c (same pool/spawn/tick/draw shape, but pushed by flow instead of
 * pulled by a force).
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

  MAX_PARTICLES = 1000,

  TRAIL_LEN = 4, /* how many cells of trailing streak each speck draws */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slots. HUD/HINT are reserved project-wide (see CLAUDE.md). */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* +0..+7: the 8 colours from outer-dim to inner-bright */
  PAIR_CENTER = 11,   /* the glyph drawn at the drain */
  PAIR_SKY = 12,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define ASPECT_Y 2.0f /* a terminal cell is about twice as tall as wide;
                       * we squash vertically by this so a circle looks
                       * round instead of stretched */

/* The spin rate has a 1/r term, so a speck right at the centre would spin
 * infinitely fast. These two numbers keep that from happening: never divide
 * by a radius smaller than R_MIN_DENOM, and kill a speck once it's inside
 * R_DRAIN_CELLS of the centre so it never gets that close anyway. */
#define R_MIN_DENOM 1.0f
#define R_DRAIN_CELLS 2.5f

/* How far back in time each trail step looks when drawing the streak (sec). */
#define TRAIL_STEP_DT 0.04f

/* The ten looks. */
typedef enum {
  PATTERN_WHIRLPOOL = 0,
  PATTERN_TORNADO = 1,
  PATTERN_BLACK_HOLE = 2,
  PATTERN_SINK = 3,
  PATTERN_GALAXY = 4,
  PATTERN_HURRICANE = 5,
  PATTERN_NEBULA = 6,
  PATTERN_CYCLONE = 7,
  PATTERN_MAELSTROM = 8,
  PATTERN_PULSAR = 9,
  N_PATTERNS = 10,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_WHIRLPOOL:
    return "WHIRLPOOL ";
  case PATTERN_TORNADO:
    return "TORNADO   ";
  case PATTERN_BLACK_HOLE:
    return "BLACK_HOLE";
  case PATTERN_SINK:
    return "SINK      ";
  case PATTERN_GALAXY:
    return "GALAXY    ";
  case PATTERN_HURRICANE:
    return "HURRICANE ";
  case PATTERN_NEBULA:
    return "NEBULA    ";
  case PATTERN_CYCLONE:
    return "CYCLONE   ";
  case PATTERN_MAELSTROM:
    return "MAELSTROM ";
  case PATTERN_PULSAR:
    return "PULSAR    ";
  default:
    return "?         ";
  }
}

/*
 * PatternParams — the dial settings that make one of the ten looks.
 *
 * The whole point of the file: there is exactly ONE engine, and these
 * eight numbers are everything that makes a tornado different from a
 * galaxy. To add a new look you append one row to pattern_params[] —
 * no new code. (This data-table-of-behaviours idea is straight from
 * Reeves' 1983 particle-systems paper, ACM TOG 2(2): 91-108.)
 *
 * Two of the dials decide how fast a speck falls inward (its inflow),
 * two decide how fast it spins, and the rest are looks. The spin pair
 * in particular picks between the classic vortex shapes physicists know
 * (Acheson, "Elementary Fluid Dynamics", Ch. 5): all-spin-the-same is a
 * solid spinning disc; spin that jumps near the centre is a tornado-like
 * "potential vortex"; a mix of both is the in-between Rankine vortex.
 *
 * FIELDS:
 *   target_count    How many specks this look tries to keep alive. We
 *                   top up toward this number every step (a little at a
 *                   time, so unpausing doesn't dump them all at once).
 *                   Bigger = denser spiral. Ranges 300 (NEBULA, a thin
 *                   dust drift) to 800 (GALAXY, packed arms).
 *
 *   r_outer_frac    How big the vortex is: the spawn ring's radius as a
 *                   fraction of half the screen. Smaller = tighter.
 *                   Ranges 0.65 (PULSAR, a small tight core) to 0.98
 *                   (HURRICANE, fills the screen).
 *
 *   inflow_log      Pull-inward strength that scales with distance: a
 *                   far speck falls fast, a near one drifts. On its own
 *                   this is what makes the smooth even-spaced arms of a
 *                   galaxy. Ranges 0.04 (slow drift) to 0.55 (BLACK_HOLE,
 *                   yanked in hard). Units 1/s; adds −inflow_log·r to dr/dt.
 *
 *   inflow_const    Pull-inward strength that's the same everywhere, near
 *                   or far — the steady suck of a bathtub drain (SINK).
 *                   Ranges 0 (GALAXY, no constant pull) to 4.0 (SINK).
 *                   Units cells/s; adds −inflow_const to dr/dt.
 *
 *   angular_const   Spin rate that's the same at every distance, so the
 *                   whole thing turns like a solid disc. Ranges 0.20
 *                   (NEBULA, barely turning) to 2.20 (CYCLONE, whipping
 *                   around). Units rad/s; adds +angular_const to dθ/dt.
 *
 *   angular_kepler  Extra spin that kicks in near the centre — the closer
 *                   a speck gets, the faster it whirls. This is the
 *                   skater-pulling-their-arms-in effect (Kepler's law of
 *                   equal areas, "Astronomia Nova" 1609). Adds
 *                   +angular_kepler / distance to dθ/dt. Three regimes:
 *                     0       far and near spin alike — plain circles
 *                             (CYCLONE, GALAXY, WHIRLPOOL)
 *                     4 – 5.5 a tight fast whip at the core
 *                             (TORNADO, MAELSTROM, PULSAR)
 *                     8       runaway spin-up, a black-hole accretion disc
 *                             (BLACK_HOLE)
 *
 *   center_glyph    The single character drawn on the drain, so each look
 *                   has its own centre mark: '.' WHIRLPOOL/NEBULA,
 *                   '*' TORNADO/GALAXY, '@' BLACK_HOLE/HURRICANE,
 *                   'O' SINK, 'o' CYCLONE, '+' PULSAR, '#' MAELSTROM.
 *
 *   center_pulse    Nonzero makes the centre mark throb bright/dim like a
 *                   heartbeat. On for BLACK_HOLE, MAELSTROM, PULSAR.
 */
typedef struct {
  int target_count;
  float r_outer_frac;
  float inflow_log;
  float inflow_const;
  float angular_const;
  float angular_kepler;
  char center_glyph;
  int center_pulse;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  target  rfrac  inLog  inConst  angC    angKep  ctr pulse
     */
    /* WHIRLPOOL  */ {500, 0.85f, 0.40f, 0.30f, 1.40f, 0.0f, '.', 0},
    /* TORNADO    */ {600, 0.85f, 0.20f, 1.20f, 0.80f, 4.0f, '*', 0},
    /* BLACK_HOLE */ {700, 0.95f, 0.55f, 2.00f, 0.50f, 8.0f, '@', 1},
    /* SINK       */ {450, 0.85f, 0.00f, 4.00f, 2.00f, 1.0f, 'O', 0},
    /* GALAXY     */ {800, 0.95f, 0.06f, 0.00f, 0.50f, 0.0f, '*', 0},
    /* HURRICANE  */ {600, 0.98f, 0.15f, 0.10f, 1.00f, 2.5f, '@', 0},
    /* NEBULA     */ {300, 0.85f, 0.04f, 0.05f, 0.20f, 0.5f, '.', 0},
    /* CYCLONE    */ {550, 0.90f, 0.20f, 0.50f, 2.20f, 0.0f, 'o', 0},
    /* MAELSTROM  */ {700, 0.85f, 0.45f, 1.80f, 1.60f, 4.0f, '#', 1},
    /* PULSAR     */ {400, 0.65f, 0.30f, 0.60f, 1.40f, 5.5f, '+', 1},
};

/*
 * Theme — one colour scheme: an 8-colour fade for the specks plus a
 * couple of accent colours.
 *
 * Each theme sticks to one colour family so flipping through them with
 * t/T is obvious at a glance — except NOVA, which deliberately sweeps
 * several hues (blue → magenta → orange → yellow → white) to mimic a
 * supernova's spectrum across the arms. All colours are picked from the
 * bright half of the palette so even the dimmest one stays visible on a
 * black terminal (see the "Theme Palette Brightness" note in CLAUDE.md).
 *
 * FIELDS:
 *   name     The label shown in the HUD.
 *   ramp     The 8 speck colours, dim outer (slot 0) to bright inner
 *            (slot 7); a speck picks its colour by how close it is to
 *            the drain, so the spiral glows hotter toward the middle.
 *   center   The colour of the drain glyph — the brightest, poppiest hue
 *            that still goes with the ramp.
 *   sky      The background tint for this theme.
 */
typedef struct {
  const char *name;
  short ramp[8];
  short center;
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] =
    {
        /* name        ramp[0..7]  (outer dim → inner bright) center  sky */
        {"MATRIX",
         {28, 34, 40, 46, 82, 118, 154, 190},
         226,
         234}, /* phosphor green       */
        {"FIRE",
         {52, 88, 124, 160, 202, 208, 220, 231},
         231,
         233}, /* coal→flame→white-hot */
        {"OCEANIC",
         {24, 25, 32, 38, 44, 80, 116, 152},
         195,
         234}, /* deep blue → teal     */
        {"NEON",
         {54, 92, 128, 165, 201, 207, 213, 219},
         225,
         234}, /* purple → hot pink    */
        {"MONO",
         {240, 244, 247, 250, 252, 253, 254, 255},
         255,
         232}, /* grayscale            */
        {"ICE",
         {117, 153, 159, 195, 231, 251, 253, 255},
         255,
         235}, /* pale frost → white   */
        {"NOVA",
         {27, 99, 165, 201, 208, 220, 226, 231},
         231,
         234}, /* SPECTRUM (multi-hue) */
        {"FOREST",
         {58, 64, 70, 106, 142, 148, 184, 220},
         226,
         234}, /* olive → leaf → gold  */
        {"DESERT",
         {94, 130, 137, 173, 215, 222, 228, 230},
         230,
         234}, /* dune → sand → cream  */
        {"ECLIPSE",
         {53, 54, 89, 125, 161, 197, 204, 209},
         219,
         232}, /* void purple → crimson*/
};

/* The 8 speck characters, faint outer to solid inner — matched up with
 * the 8 ramp colours so glyph and colour both get bolder toward the drain. */
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
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_CENTER, t->center, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), COLOR_CYAN, -1);
    init_pair(PAIR_CENTER, COLOR_WHITE, -1);
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

/* ── §4 particle ── */

/*
 * Particle — one swirling speck.
 *
 * The key trick: a speck isn't stored as an x/y point. It's stored as a
 * distance from the centre and an angle around it (polar coordinates).
 * That's only two numbers, and it means the physics just nudges those two
 * each step — shrink the distance, grow the angle — and a spiral comes out
 * for free. We don't draw a spiral curve anywhere; it emerges. We only turn
 * distance+angle back into an x/y cell at the very end, when drawing.
 *
 * All the specks live in one fixed array filled once at startup, never with
 * malloc. The `active` flag says whether a slot is in use; to spawn a new
 * speck we just scan for the first slot that's free.
 *
 * Life of a speck: it's born out on the rim, falls inward and spins up each
 * step, and dies the moment it's close enough to the drain — every speck
 * ends at the centre, none escape outward. (The pool-and-cull design is
 * Reeves 1983, ACM TOG 2(2): 91-108; the spin-up-as-it-falls is Kepler's
 * equal-areas law.)
 *
 * FIELDS:
 *   r       Distance from the centre, in cells. Always positive. We never
 *           let the spin math divide by anything smaller than R_MIN_DENOM,
 *           so even a speck at the very middle can't spin infinitely fast.
 *
 *   theta   Angle around the centre, in radians. It just keeps growing —
 *           we never wrap it back into 0..2π. sin/cos handle big numbers
 *           fine, and a speck dies near the drain after only a few turns
 *           anyway, so precision is never an issue.
 *
 *   age     Seconds since this speck was born. Nothing uses it right now
 *           (specks die by distance, and colour comes from distance too) —
 *           it's here in case a future tweak wants age-based effects like
 *           fading in at birth.
 *
 *   active  Is this slot in use? Loops skip the inactive ones, and spawning
 *           grabs the first inactive slot it finds.
 */
typedef struct {
  float r;
  float theta;
  float age;
  bool active;
} Particle;

/* A cheap built-in random-number generator (a linear congruential generator)
 * — fast, no library calls, plenty random enough for scattering specks. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── §5 scene — pool, polar tick, draw ── */

/*
 * Scene — all the changing state of the demo in one box.
 *
 * It's split into two groups on purpose. The first group is everything the
 * physics step touches each frame; the second is the one thing only drawing
 * cares about (the colour theme). Keeping them apart makes each part of the
 * code easy to read, and keeps the hot physics fields packed together in
 * memory so the per-frame loop stays fast.
 *
 * The Scene knows nothing about the terminal — physics writes here, drawing
 * reads here, and the two never tangle. That split means you could run the
 * simulation with no screen at all (say, to profile it).
 */
typedef struct {
  /* ── simulation: the physics step reads and writes these ── */

  /* When true the physics freezes but drawing keeps going, so you see a
   * still frame with every speck stopped mid-spiral. Toggled by SPACE. */
  bool paused;

  /* A time multiplier for slow-mo / fast-forward. It only stretches or
   * compresses time — it doesn't change the look's character, just how
   * fast you watch it. +/= speeds up, − slows down. */
  int speed;

  /* Which of the ten looks is active (an index into pattern_params[]).
   * n/N cycle it; switching refills the screen so the new look is full
   * from the first frame. This is the one field both physics and drawing
   * read — physics for the motion, drawing for the centre glyph. */
  Pattern current_pattern;

  /* The state of this scene's random-number generator. Used for picking
   * each new speck's angle and starting distance. r reseeds it. */
  uint32_t rng;

  /* The current terminal size, remembered here so the per-frame code never
   * has to ask ncurses for it. Refreshed at startup and on a resize. */
  int rows, cols;

  /* Seconds since the demo started, ticking up by dt each step. Drives the
   * heartbeat throb on the BLACK_HOLE / MAELSTROM / PULSAR centre glyphs. */
  float time_accum;

  /* Every speck. One fixed array, filled at startup, never resized. */
  Particle particles[MAX_PARTICLES];

  /* ── render: only drawing reads this; physics ignores it ── */

  /* Which colour theme is active (an index into themes[]). t/T cycle it.
   * Purely a look — the physics runs identically whatever the theme. */
  int current_theme;
} Scene;

static int particle_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (!s->particles[i].active)
      return i;
  return -1;
}

static void scene_clear_particles(Scene *s) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    s->particles[i].active = false;
}

/* How far out the rim of the vortex sits, in cells, for the screen we have. */
static float scene_r_outer(const Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  /* Fit to whichever screen side is smaller so the vortex always fits. */
  float half_w = (float)s->cols * 0.5f;
  float half_h = (float)s->rows * 0.5f * ASPECT_Y;
  float min_h = half_w < half_h ? half_w : half_h;
  return min_h * pp->r_outer_frac;
}

/*
 * Wake up one speck at a random spot in the distance band r_min..r_max.
 * Normally that band is a thin ring out at the rim; at startup it's the
 * whole range, so the spiral is already full on the first frame.
 */
static void scene_spawn_particle(Scene *s, float r_min, float r_max) {
  int idx = particle_pool_find_inactive(s);
  if (idx < 0)
    return;
  Particle *p = &s->particles[idx];

  float r1 = lcg_unit(&s->rng);
  float r2 = lcg_unit(&s->rng);

  p->r = r_min + r1 * (r_max - r_min);
  p->theta = r2 * 2.0f * (float)M_PI;
  p->age = 0.0f;
  p->active = true;
}

static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_count;
  if (target > MAX_PARTICLES)
    target = MAX_PARTICLES;

  int active = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (s->particles[i].active)
      active++;

  float r_outer = scene_r_outer(s);
  /* Scatter specks across the whole radius so the spiral starts full. */
  for (int k = active; k < target; k++)
    scene_spawn_particle(s, R_DRAIN_CELLS + 1.0f, r_outer);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_WHIRLPOOL;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  scene_clear_particles(s);
  scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  scene_clear_particles(s);
  scene_prewarm(s);
}

/* For a speck at distance r, how fast it's falling inward and spinning
 * around, under the current look's dials. The one spot the spin's
 * speed-up-near-the-centre actually gets computed. */
static inline void scene_polar_rates(const PatternParams *pp, float r,
                                     float *out_dr, float *out_dtheta) {
  float r_safe = r > R_MIN_DENOM ? r : R_MIN_DENOM;
  *out_dr = -(pp->inflow_log * r + pp->inflow_const);
  *out_dtheta = pp->angular_const + pp->angular_kepler / r_safe;
}

/* ── helpers the tick step leans on ── */

/* Count how many specks are currently alive. */
static int count_active_particles(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (s->particles[i].active)
      n++;
  return n;
}

/* How many specks to add this step. We aim for the look's target count, but
 * only add a few per step (scaled by dt) so a long pause doesn't dump the
 * whole population back in a single frame when you unpause. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt) {
  int target = pp->target_count;
  if (target > MAX_PARTICLES)
    target = MAX_PARTICLES;
  int spawn_cap = (int)((float)pp->target_count * dt * 4.0f) + 4;
  int n = target - active;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

/* Add n fresh specks out near the rim, to replace the ones that just fell
 * into the drain. New ones always start at the edge so they get the full
 * ride inward. */
static void particle_pool_topup_at_outer_band(Scene *s, float r_outer, int n) {
  for (int k = 0; k < n; k++)
    scene_spawn_particle(s, r_outer * 0.85f, r_outer);
}

/* Move one speck forward by a time step dt: nudge it inward and around by
 * its current rates, and age it a touch. (Simplest possible integration —
 * just current rate times the time step.) */
static inline void
integrate_particle_polar_euler(Particle *p, const PatternParams *pp, float dt) {
  float dr, dtheta;
  scene_polar_rates(pp, p->r, &dr, &dtheta);
  p->r += dr * dt;
  p->theta += dtheta * dt;
  p->age += dt;
}

/* Has this speck reached the drain and finished its life? The cutoff sits
 * a little out from dead-centre, so specks vanish before they ever get close
 * enough for the 1/r spin term to blow up. */
static inline bool particle_drained(const Particle *p) {
  return p->r < R_DRAIN_CELLS;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;
  s->time_accum += dt;

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float r_outer = scene_r_outer(s);

  /* 1. Add fresh specks at the rim to keep the population up. */
  int active = count_active_particles(s);
  int to_spawn = compute_spawn_count_for_tick(active, pp, dt);
  particle_pool_topup_at_outer_band(s, r_outer, to_spawn);

  /* 2. Move every speck inward and around; retire the ones that reached
   *    the drain. */
  for (int i = 0; i < MAX_PARTICLES; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;

    integrate_particle_polar_euler(p, pp, dt);
    if (particle_drained(p))
      p->active = false;
  }
}

/*
 * Turn a speck's distance + angle into an actual screen cell, measured out
 * from the centre (cx, cy). The vertical squash makes a circle look round
 * instead of stretched, since terminal cells are taller than they are wide.
 */
static inline void polar_to_cell(float r, float theta, float cx, float cy,
                                 float *out_x, float *out_y) {
  *out_x = cx + r * cosf(theta);
  *out_y = cy + r * sinf(theta) / ASPECT_Y;
}

/* ── drawing pieces ── */

/* Pick which of the 8 brightness slots (0..7) a speck gets from how close
 * it is to the drain: out at the rim it's slot 0 (dim), at the centre it's
 * slot 7 (bright). That's why the spiral looks like it's heating up inward. */
static inline int inner_fraction_to_ramp_slot(float r, float r_outer) {
  float f = 1.0f - r / (r_outer + 1.0f);
  if (f < 0.0f)
    f = 0.0f;
  if (f > 1.0f)
    f = 1.0f;
  int slot = (int)(f * 7.0f + 0.5f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* Bold for the brightest slots, dim for the darkest, normal in between —
 * an extra nudge to the colour ramp so the spiral reads hotter inward. */
static inline int head_attr_by_brightness_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* The throbbing centre mark on the pulsing looks: a steady sine over time
 * flips the glyph between bold and normal, like a heartbeat. It never goes
 * fully dark, so the centre stays visible the whole time. */
static inline int center_pulse_attr(float time_accum) {
  float pulse = 0.5f + 0.5f * sinf(time_accum * 3.0f);
  return (pulse < 0.4f) ? A_NORMAL : A_BOLD;
}

/* Work out one "step into the past" for the trail. We take the speck's
 * current motion and run it backwards a tiny bit: it was a little farther
 * out and a little less far around. Stepping back this way makes the streak
 * curve along the spiral instead of cutting a straight line across it. */
static inline void compute_trail_back_step(const PatternParams *pp,
                                           const Particle *p, float *dr_back,
                                           float *dtheta_back) {
  float dr, dtheta;
  scene_polar_rates(pp, p->r, &dr, &dtheta);
  *dr_back = -dr * TRAIL_STEP_DT;
  *dtheta_back = dtheta * TRAIL_STEP_DT;
}

/* Draw one speck as a short fading streak plus a bright head. We step back
 * along the spiral a few times, dimming as we go, then draw the head last —
 * drawing tail-first means the bright head always sits on top where streaks
 * cross, so the dot your eye follows never gets buried. */
static void trail_draw_one_particle(const Particle *p, const PatternParams *pp,
                                    int head_slot, float cx, float cy, int cols,
                                    int rows_eff) {
  float dr_back, dtheta_back;
  compute_trail_back_step(pp, p, &dr_back, &dtheta_back);

  for (int t = TRAIL_LEN; t >= 0; t--) {
    float r_at = p->r + dr_back * (float)t;
    float theta_at = p->theta - dtheta_back * (float)t;
    float fx, fy;
    polar_to_cell(r_at, theta_at, cx, cy, &fx, &fy);
    int ix = (int)(fx + 0.5f);
    int iy = (int)(fy + 0.5f);
    if (ix < 0 || ix >= cols)
      continue;
    if (iy < 0 || iy >= rows_eff)
      continue;

    int slot = head_slot - t;
    if (slot < 0)
      slot = 0;
    char glyph = RAMP_GLYPHS[slot];
    int attr = head_attr_by_brightness_slot(slot);
    int pair = PAIR_RAMP_BASE + slot;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Draw every live speck as a streak-plus-head — together they are the
 * spiral arms. */
static void particle_pool_draw(const Scene *s, float cx, float cy,
                               int rows_eff) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float r_outer = scene_r_outer(s);
  for (int i = 0; i < MAX_PARTICLES; i++) {
    const Particle *p = &s->particles[i];
    if (!p->active)
      continue;
    int head_slot = inner_fraction_to_ramp_slot(p->r, r_outer);
    trail_draw_one_particle(p, pp, head_slot, cx, cy, s->cols, rows_eff);
  }
}

/* Draw the drain mark in the middle. On the pulsing looks it throbs;
 * on the rest it just stays bold. */
static void center_drain_draw(const Scene *s, float cx, float cy,
                              int rows_eff) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int icx = (int)(cx + 0.5f);
  int icy = (int)(cy + 0.5f);
  if (icx < 0 || icx >= s->cols)
    return;
  if (icy < 0 || icy >= rows_eff)
    return;

  int attr = pp->center_pulse ? center_pulse_attr(s->time_accum) : (int)A_BOLD;
  attron(COLOR_PAIR(PAIR_CENTER) | attr);
  mvaddch(icy, icx, (chtype)(unsigned char)pp->center_glyph);
  attroff(COLOR_PAIR(PAIR_CENTER) | attr);
}

static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1; /* keep the bottom row free for the HUD */
  float cx = (float)s->cols * 0.5f;
  float cy = (float)rows_eff * 0.5f;

  particle_pool_draw(s, cx, cy, rows_eff); /* the spiral arms */
  center_drain_draw(s, cx, cy, rows_eff);  /* the drain in the middle */
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

static int scene_active_count(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (s->particles[i].active)
      n++;
  return n;
}

/*
 * Draw the spiral, then lay two info bars on top: a status line along the
 * top (which look, which theme, speck count, frame rate, speed) and a key
 * hint line along the bottom. We paint the bars after the spiral and fill
 * their whole width with colour first, so specks never show through them.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int active = scene_active_count(s);
  const char *state_str =
      s->paused ? "PAUSED    " : pattern_name(s->current_pattern);

  /* ── top row: live status ── */
  char status[220];
  snprintf(status, sizeof status,
           " VORTEX   %s   theme:%-8s   particles:%4d   "
           "%5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, active, fps, sim_fps,
           s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── bottom row: every key you can press ── */
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

/* ── §7 app ── */

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
