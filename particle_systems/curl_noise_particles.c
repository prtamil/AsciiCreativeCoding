/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_particles.c — a few hundred dots that swirl around the
 * screen like smoke in a draft.
 *
 * Instead of running a real fluid simulation, we fake the look. We take
 * a smooth blob of noise and let particles flow along its contour lines.
 * That trick keeps the swarm evenly spread forever — it never clumps up
 * or tears holes. Four presets (calm, turbulent, hurricane, wind tunnel)
 * just change a handful of numbers; the flow code stays the same.
 *
 * Sister files: fluid/navier_stokes.c is the real fluid solver (slower,
 * far more code); particle_systems/vortex.c does a single central drain.
 *
 * The flow trick comes from Bridson, Houriham & Nordenstam, "Curl-Noise
 * for Procedural Fluid Flow", ACM TOG 26(3), 2007 (SIGGRAPH).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/curl_noise_particles.c \
 *       -o curl_noise_particles -lncurses -lm
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

  MAX_PARTICLES = 600,
  TRAIL_LEN = 4,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* ncurses colour-pair slots. HUD and hint are reserved project-wide. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* slots +0..+7: the 8 speed colours, slow to fast */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Terminal cells are about twice as tall as they are wide, so we stretch
 * the y axis by this much to make the noise see square ground. */
#define ASPECT_Y 2.0f

/* How far apart to sample the noise when measuring its slope. */
#define CURL_H 0.5f

/* The speed (cells/sec) that counts as "full brightness". A particle this
 * fast gets the brightest colour; slower ones get dimmer colours. Drop
 * this number to make slow particles look brighter. */
#define SPEED_REF 25.0f

/* The four flow presets. */
typedef enum {
  PATTERN_CALM = 0,
  PATTERN_TURBULENT = 1,
  PATTERN_HURRICANE = 2,
  PATTERN_WIND_TUNNEL = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_CALM:
    return "CALM       ";
  case PATTERN_TURBULENT:
    return "TURBULENT  ";
  case PATTERN_HURRICANE:
    return "HURRICANE  ";
  case PATTERN_WIND_TUNNEL:
    return "WIND_TUNNEL";
  default:
    return "?          ";
  }
}

/*
 * PatternParams — the dials that define one flow preset. Each preset is
 * just one row of these numbers; the flow code never branches on which
 * preset is active, it only reads these values. Adding a new look means
 * adding a row, not new code.
 *
 *   target_count : how many particles to keep alive (visual density)
 *   fbm_octaves  : how many layers of noise to stack. 1 is a single
 *                  smooth wave, 4 gives rich detail at many scales.
 *   fbm_freq     : how zoomed-in the noise is (in 1/cells). Small means
 *                  big lazy swirls, large means lots of tiny ones.
 *   field_mag    : how hard the flow pushes particles (see note below).
 *   time_drift_x : how fast the noise slides sideways over time, so the
 *   time_drift_y   flow keeps changing instead of standing still.
 *   global_rot   : extra spin around the screen centre, in rad/sec.
 *                  Zero except for the hurricane preset.
 *   wind_bias    : extra steady push to the right, in cells/sec.
 *                  Zero except for the wind-tunnel preset.
 */
typedef struct {
  int target_count;
  int fbm_octaves;
  float fbm_freq;
  float field_mag;
  float time_drift_x;
  float time_drift_y;
  float global_rot;
  float wind_bias;
} PatternParams;

/*
 * field_mag has to be big. The noise's slope is naturally tiny at low
 * zoom, so we multiply it up to get particles moving at a few cells per
 * second. The hurricane and wind-tunnel rows can afford a smaller value
 * because their extra spin or push already keeps things moving.
 */
static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  cnt  oct   freq    mag     tdx    tdy    rot    wind */
    /* CALM         */ {280, 2, 0.025f, 800.0f, 0.30f, 0.20f, 0.00f, 0.0f},
    /* TURBULENT    */ {450, 4, 0.060f, 1100.0f, 0.80f, 0.50f, 0.00f, 0.0f},
    /* HURRICANE    */ {380, 2, 0.020f, 500.0f, 0.30f, 0.20f, 0.40f, 0.0f},
    /* WIND_TUNNEL  */ {380, 3, 0.045f, 600.0f, 0.50f, 0.30f, 0.00f, 18.0f},
};

/*
 * Theme — one colour scheme. Each is an 8-step colour gradient that runs
 * from slow particles to fast ones, so a particle's speed shows up as its
 * colour and you can read the shape of the flow at a glance.
 *
 *   name : label shown in the heads-up display
 *   ramp : the 8 colours, slowest particle first, fastest last
 *   sky  : the background colour
 *
 * Every colour stays in the bright half of the 256-colour range so even
 * the slowest particles remain visible (project palette-brightness rule).
 */
typedef struct {
  const char *name;
  short ramp[8]; /* slow particle first, fast particle last */
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name         ramp[0..7]  (slow → fast)                          sky */

    {"DEFAULT", {24, 31, 38, 45, 87, 117, 153, 195}, 234},
    {"TROPICAL", {29, 35, 37, 44, 50, 86, 122, 159}, 234},
    {"FIRE", {88, 124, 130, 166, 196, 208, 214, 226}, 233},
    {"AURORA", {43, 79, 115, 121, 157, 195, 230, 231}, 234},
    {"VIOLET", {53, 91, 134, 165, 207, 213, 219, 225}, 234},
    {"ICE", {24, 31, 67, 110, 117, 153, 195, 231}, 235},
    {"NEON", {199, 213, 207, 219, 225, 231, 195, 153}, 234},
    {"COPPER", {130, 137, 173, 179, 215, 222, 229, 230}, 234},
    {"MONO", {240, 243, 245, 247, 249, 251, 253, 255}, 232},
    {"INFRARED", {17, 21, 39, 46, 154, 226, 208, 196}, 233},
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
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), COLOR_CYAN, -1);
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
 * DebugMode — the d/D keys cycle through four ways of drawing the scene,
 * each one showing a different part of what's going on under the hood:
 *
 *   DBG_NORMAL     the usual look: particles with fading trails, coloured
 *                  by speed.
 *   DBG_VELOCITY   replaces each particle with a little arrow showing
 *                  which way it's heading, so you can see the flow lines.
 *   DBG_HEATMAP    shades the whole screen by the underlying noise blob —
 *                  the hidden landscape the particles flow across.
 *                  Particles travel along its contour lines.
 *   DBG_DIVERGENCE shades the screen by how much the flow is "piling up"
 *                  anywhere. In theory that's exactly zero everywhere
 *                  (that's the whole point of the trick), so the screen
 *                  should look almost empty. Any bright specks are just
 *                  tiny rounding errors from measuring the slopes.
 *
 * Kept as a file-wide variable so the draw code can read it without
 * passing it through every function. It's display-only — it never affects
 * the physics and resets each run.
 */
typedef enum {
  DBG_NORMAL = 0,
  DBG_VELOCITY = 1,
  DBG_HEATMAP = 2,
  DBG_DIVERGENCE = 3,
  DBG_COUNT = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal",
    "velocity",
    "heatmap",
    "divergence",
};

static DebugMode g_debug = DBG_NORMAL;

/* Picks an ASCII arrow pointing roughly the way (vx, vy) goes. Note y
 * grows downward on screen, so the diagonals look flipped from a math
 * diagram. Returns '.' when there's no motion. */
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

/* ── §4 noise ── */

/*
 * This section builds the smooth random "landscape" the particles flow
 * across, in two layers:
 *
 *   perlin2d(x, y)      one layer of smooth random noise, valued ~[-1, 1]
 *   fbm2(x, y, octaves) several Perlin layers stacked: each one twice as
 *                       fine and half as strong as the last, which adds
 *                       detail at many sizes at once
 *
 * It's pasted in here rather than shared from a header because every file
 * in this project is meant to stand alone.
 *
 * The smoothing curve below (the "quintic fade") is extra smooth on
 * purpose. We later measure the slope of this landscape twice over, and a
 * rougher curve would leave kinks at the grid lines that show up as fake
 * swirls right where we don't want them.
 */

static uint8_t perm[512];

static void perm_shuffle(int seed) {
  uint8_t base[256];
  for (int i = 0; i < 256; i++)
    base[i] = (uint8_t)i;
  uint32_t st = (uint32_t)seed * 2654435761u;
  for (int i = 255; i > 0; i--) {
    st = st * 1664525u + 1013904223u;
    int j = (int)(st >> 16) % (i + 1);
    uint8_t t = base[i];
    base[i] = base[j];
    base[j] = t;
  }
  for (int i = 0; i < 256; i++) {
    perm[i] = base[i];
    perm[i + 256] = base[i];
  }
}

static inline float fade_q(float t) {
  return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}
static inline float lerp_f(float a, float b, float t) {
  return a + t * (b - a);
}
static inline float grad2(int hash, float x, float y) {
  int h = hash & 7;
  float u = (h < 4) ? x : y;
  float v = (h < 4) ? y : x;
  return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/* One sample of smooth random noise at (x, y), valued roughly [-1, 1].
 * Picture a hidden arrow planted at every whole-number grid point; the
 * value here is a smooth blend of how those nearby arrows line up with the
 * direction to (x, y). One unit of x or y is one grid step. */
static float perlin2d(float x, float y) {
  int X = (int)floorf(x) & 255;
  int Y = (int)floorf(y) & 255;
  x -= floorf(x);
  y -= floorf(y);
  float u = fade_q(x), v = fade_q(y);
  int A = perm[X] + Y;
  int B = perm[X + 1] + Y;
  float n00 = grad2(perm[A], x, y);
  float n10 = grad2(perm[B], x - 1.0f, y);
  float n01 = grad2(perm[A + 1], x, y - 1.0f);
  float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
  return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* Stacks several layers of Perlin noise, each one twice as fine and half
 * as strong as the last, to get detail at many sizes at once: one big
 * swirl with smaller swirls inside it. More layers (octaves) means more
 * fine chop. Returns roughly [-0.5, 0.5]. */
static float fbm2(float x, float y, int octaves) {
  float total = 0, amp = 1, freq = 1, max_amp = 0;
  for (int o = 0; o < octaves; o++) {
    total += amp * perlin2d(x * freq, y * freq);
    max_amp += amp;
    amp *= 0.5f;
    freq *= 2.0f;
  }
  return (total / max_amp) * 0.5f + 0.0f;
}

/* ── §5 curl ── */

/*
 * This is the heart of the demo. Section 4 gave us a smooth random
 * landscape; here we turn it into a flow direction at any point. The trick:
 * instead of rolling downhill, particles travel *along* the contour lines,
 * keeping the high ground on one side. That sideways-to-the-slope rule is
 * what keeps the swarm from ever piling up or thinning out.
 *
 * We work in stretched ("physical") coordinates where the cells are square,
 * so the landscape isn't lopsided. The caller stretches the y axis on the
 * way in and un-stretches it on the way out — not this function.
 */

/*
 * Returns the flow velocity at one point, for an active particle to follow.
 * Hurricane spin and wind-tunnel push are folded in here too since they're
 * just extra velocity added on top.
 *
 * It works by sampling the landscape just east, west, north and south of the
 * point, seeing which way it tilts, and turning sideways to that tilt.
 *
 * The time t slides the landscape along over the seconds, so the flow keeps
 * shifting instead of standing still — without it the motion looks dead
 * after a moment.
 *
 *   px_phys, py_phys   the point, in stretched (square) coordinates
 *   t                  seconds elapsed, used to drift the landscape
 *   pp                 which preset's dials to read
 *   screen_cx/cy_phys  screen centre, only used by the hurricane spin
 *   out_vx/out_vy      where the resulting velocity is written
 *
 * Called once per particle to move it, and again at draw time to colour it
 * by speed; kept as one function so that math lives in a single place.
 */
static void curl_velocity_at(float px_phys, float py_phys, float t,
                             const PatternParams *pp, float screen_cx_phys,
                             float screen_cy_phys, float *out_vx_phys,
                             float *out_vy_phys) {
  float k = pp->fbm_freq;
  float tx = t * pp->time_drift_x;
  float ty = t * pp->time_drift_y;

  float n_xp =
      fbm2((px_phys + CURL_H) * k + tx, py_phys * k + ty, pp->fbm_octaves);
  float n_xm =
      fbm2((px_phys - CURL_H) * k + tx, py_phys * k + ty, pp->fbm_octaves);
  float n_yp =
      fbm2(px_phys * k + tx, (py_phys + CURL_H) * k + ty, pp->fbm_octaves);
  float n_ym =
      fbm2(px_phys * k + tx, (py_phys - CURL_H) * k + ty, pp->fbm_octaves);

  float dN_dx = (n_xp - n_xm) / (2.0f * CURL_H);
  float dN_dy = (n_yp - n_ym) / (2.0f * CURL_H);

  float vx = dN_dy * pp->field_mag;
  float vy = -dN_dx * pp->field_mag;

  /* Hurricane: add a steady spin around the screen centre. */
  if (pp->global_rot > 0.0f) {
    float dx = px_phys - screen_cx_phys;
    float dy = py_phys - screen_cy_phys;
    vx += -dy * pp->global_rot;
    vy += dx * pp->global_rot;
  }
  /* Wind tunnel: add a steady push to the right. */
  if (pp->wind_bias != 0.0f) {
    vx += pp->wind_bias;
  }

  *out_vx_phys = vx;
  *out_vy_phys = vy;
}

/* ── §6 particle ── */

/*
 * Particle — one drifting dot, plus a short memory of where it just was so
 * we can draw a fading tail behind it.
 *
 *   px, py      where it is now, in screen cells
 *   trail_x/y   its last few positions, kept in a small ring buffer
 *   trail_head  index of the newest entry in that ring buffer
 *   trail_count how many of the trail slots are filled yet (it fills up
 *               over the first few frames after a spawn)
 *   active      true if this slot holds a live particle
 *
 * Particles never die here — they wrap around the screen edges and live
 * forever. The pool is a fixed array, so a slot is reused rather than freed.
 */
typedef struct {
  float px, py; /* where it is now, in screen cells */
  float trail_x[TRAIL_LEN];
  float trail_y[TRAIL_LEN];
  int trail_head;
  int trail_count;
  bool active;
} Particle;

/* A tiny, fast random-number generator (good enough for scattering dots). */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── §7 scene ── */

/*
 * This section ties it all together. The Scene holds the whole running
 * state — the particles, the clock, the chosen preset and theme — and
 * offers two jobs per frame: scene_tick moves everything forward, and
 * scene_draw paints it. The smaller helpers spawn particles and resize
 * or reset the pool when the preset changes.
 */

/*
 * Scene — everything the running demo needs to remember.
 *
 *   paused          true while the user has frozen the simulation
 *   speed            user speed dial (a multiplier on time)
 *   current_theme    index into the colour-theme table
 *   current_pattern  which flow preset is active
 *   rng              seed/state for the random scatter of particles
 *   rows, cols       current screen size in cells
 *   time_accum       seconds of simulated time, used to drift the noise
 *   particles        the fixed pool of dots
 */
typedef struct {
  bool paused;
  int speed;
  int current_theme;
  Pattern current_pattern;
  uint32_t rng;
  int rows, cols;

  float time_accum; /* seconds of sim time, drifts the noise over time */

  Particle particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    s->particles[i].active = false;
}

static void scene_spawn_particle(Scene *s, int idx) {
  Particle *p = &s->particles[idx];
  p->px = lcg_unit(&s->rng) * (float)s->cols;
  p->py = lcg_unit(&s->rng) * (float)(s->rows - 1);
  p->trail_head = 0;
  p->trail_count = 0;
  /* Start the trail sitting on the spawn point so it doesn't streak in. */
  for (int k = 0; k < TRAIL_LEN; k++) {
    p->trail_x[k] = p->px;
    p->trail_y[k] = p->py;
  }
  p->active = true;
}

static void scene_populate_to_target(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_count;
  if (target > MAX_PARTICLES)
    target = MAX_PARTICLES;
  int active = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (s->particles[i].active)
      active++;
  /* If we're short, fill empty slots up to the target count. */
  int spawned = 0;
  for (int i = 0; i < MAX_PARTICLES && spawned < target - active; i++) {
    if (!s->particles[i].active) {
      scene_spawn_particle(s, i);
      spawned++;
    }
  }
  /* If we have too many, switch off the extras from the end of the pool. */
  if (active > target) {
    int to_remove = active - target;
    for (int i = MAX_PARTICLES - 1; i >= 0 && to_remove > 0; i--) {
      if (s->particles[i].active) {
        s->particles[i].active = false;
        to_remove--;
      }
    }
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_CALM;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  perm_shuffle((int)(s->rng & 0xFFFF));
  scene_clear_particles(s);
  scene_populate_to_target(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  perm_shuffle((int)(s->rng & 0xFFFF));
  scene_clear_particles(s);
  scene_populate_to_target(s);
}

/*
 * Moves every live particle forward by one step: look up the flow where it
 * sits, nudge it along, wrap it if it ran off an edge, and record its new
 * spot in the trail. All the real math is in curl_velocity_at; this just
 * drives it. No drawing happens here — keeping move and draw apart means a
 * test or batch tool could run the simulation without any screen.
 */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  dt *= speed_mul;

  s->time_accum += dt;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  int rows_eff = s->rows - 1;
  float screen_cx_phys = (float)s->cols * 0.5f;
  float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

  for (int i = 0; i < MAX_PARTICLES; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;

    /* Stretch the y axis so the flow lookup sees square ground. */
    float px_phys = p->px;
    float py_phys = p->py * ASPECT_Y;

    float vx_phys, vy_phys;
    curl_velocity_at(px_phys, py_phys, s->time_accum, pp, screen_cx_phys,
                     screen_cy_phys, &vx_phys, &vy_phys);

    /* Un-stretch the y axis to get back to screen cells. */
    float vx_cell = vx_phys;
    float vy_cell = vy_phys / ASPECT_Y;

    p->px += vx_cell * dt;
    p->py += vy_cell * dt;

    /* Off one edge, back on the opposite one. The loop usually runs at
     * most once or twice, which is cheaper than a modulo here. */
    while (p->px < 0.0f)
      p->px += (float)s->cols;
    while (p->px >= (float)s->cols)
      p->px -= (float)s->cols;
    while (p->py < 0.0f)
      p->py += (float)rows_eff;
    while (p->py >= (float)rows_eff)
      p->py -= (float)rows_eff;

    /* Record the new position in the trail. */
    p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
    p->trail_x[p->trail_head] = p->px;
    p->trail_y[p->trail_head] = p->py;
    if (p->trail_count < TRAIL_LEN)
      p->trail_count++;
  }
}

/*
 * Heatmap overlay: shades the whole screen by the underlying noise
 * landscape, so you can see the hidden terrain the particles flow across.
 * Bright cells are high ground, dark cells low; particles travel along the
 * contour lines between them.
 */
static void bg_heatmap_draw(const Scene *s) {
  int rows_eff = s->rows - 1;
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float k = pp->fbm_freq;
  float tx = s->time_accum * pp->time_drift_x;
  float ty = s->time_accum * pp->time_drift_y;

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      float px_phys = (float)col;
      float py_phys = (float)row * ASPECT_Y;
      float v = fbm2(px_phys * k + tx, py_phys * k + ty, pp->fbm_octaves);
      float f = v + 0.5f; /* shift the noise range up into 0..1 */
      if (f < 0.0f)
        f = 0.0f;
      if (f > 1.0f)
        f = 1.0f;
      int slot = (int)(f * 7.999f);

      char glyph = (slot >= 6)   ? '#'
                   : (slot >= 4) ? '+'
                   : (slot >= 2) ? '.'
                                 : ' ';
      if (glyph == ' ')
        continue;
      int pair = PAIR_RAMP_BASE + slot;
      attron(COLOR_PAIR(pair));
      mvaddch(row, col, (chtype)(unsigned char)glyph);
      attroff(COLOR_PAIR(pair));
    }
  }
}

/*
 * Divergence overlay: shades the screen by how much the flow seems to be
 * piling up or draining at each spot. The flow trick guarantees this is
 * truly zero everywhere, so the screen should look nearly empty. Any faint
 * specks are just rounding errors from measuring the slopes — a nice way to
 * see for yourself that the trick really does keep particles evenly spread.
 *
 * Only checks every other cell to keep the cost down, since each cell here
 * needs four flow lookups.
 */
static void bg_divergence_draw(const Scene *s) {
  int rows_eff = s->rows - 1;
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float screen_cx_phys = (float)s->cols * 0.5f;
  float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

  for (int row = 0; row < rows_eff; row += 2) {
    for (int col = 0; col < s->cols; col += 2) {
      float px_phys = (float)col;
      float py_phys = (float)row * ASPECT_Y;

      float vx_xp, vy_xp, vx_xm, vy_xm, vx_yp, vy_yp, vx_ym, vy_ym;
      curl_velocity_at(px_phys + CURL_H, py_phys, s->time_accum, pp,
                       screen_cx_phys, screen_cy_phys, &vx_xp, &vy_xp);
      curl_velocity_at(px_phys - CURL_H, py_phys, s->time_accum, pp,
                       screen_cx_phys, screen_cy_phys, &vx_xm, &vy_xm);
      curl_velocity_at(px_phys, py_phys + CURL_H, s->time_accum, pp,
                       screen_cx_phys, screen_cy_phys, &vx_yp, &vy_yp);
      curl_velocity_at(px_phys, py_phys - CURL_H, s->time_accum, pp,
                       screen_cx_phys, screen_cy_phys, &vx_ym, &vy_ym);

      float div =
          (vx_xp - vx_xm) / (2.0f * CURL_H) + (vy_yp - vy_ym) / (2.0f * CURL_H);
      float adiv = fabsf(div);

      /* Scaled so a perfect flow looks empty; whatever shows is just
       * rounding error. */
      float f = adiv / 0.02f;
      if (f < 0.0f)
        f = 0.0f;
      if (f > 1.0f)
        f = 1.0f;
      int slot = (int)(f * 7.999f);
      if (slot == 0)
        continue;

      char glyph = (slot >= 6) ? '#' : (slot >= 3) ? '+' : '.';
      int pair = PAIR_RAMP_BASE + slot;
      attron(COLOR_PAIR(pair) | A_BOLD);
      mvaddch(row, col, (chtype)(unsigned char)glyph);
      attroff(COLOR_PAIR(pair) | A_BOLD);
    }
  }
}

/* Paints every live particle: a fading tail behind it and a brighter head
 * on top, coloured by how fast it's moving. Touches the screen only, never
 * the simulation state. */
static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1;
  const PatternParams *pp = &pattern_params[s->current_pattern];

  float screen_cx_phys = (float)s->cols * 0.5f;
  float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

  /* Draw the debug background first, under the particles, so you can watch
   * the dots flow over the field they're following. */
  if (g_debug == DBG_HEATMAP)
    bg_heatmap_draw(s);
  if (g_debug == DBG_DIVERGENCE)
    bg_divergence_draw(s);

  for (int i = 0; i < MAX_PARTICLES; i++) {
    const Particle *p = &s->particles[i];
    if (!p->active)
      continue;

    /* Look up how fast this particle is moving, to pick its colour. */
    float vx_phys, vy_phys;
    curl_velocity_at(p->px, p->py * ASPECT_Y, s->time_accum, pp, screen_cx_phys,
                     screen_cy_phys, &vx_phys, &vy_phys);
    float speed = sqrtf(vx_phys * vx_phys + vy_phys * vy_phys);
    /* The square root brightens the slow particles. Most of the swarm is
     * slow, and without this nudge they'd all sit in the darkest colour and
     * vanish into the background. */
    float f = speed / SPEED_REF;
    if (f < 0.0f)
      f = 0.0f;
    if (f > 1.0f)
      f = 1.0f;
    f = sqrtf(f);
    int head_slot = (int)(f * 7.999f);
    if (head_slot < 1)
      head_slot = 1; /* keep the head from going invisible */
    if (head_slot > 7)
      head_slot = 7;

    /* Draw the tail oldest-first so the newest point lands on top as the
     * bright head. */
    int n = p->trail_count;
    for (int k = 0; k < n; k++) {
      int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
      float tx = p->trail_x[idx];
      float ty = p->trail_y[idx];
      int ix = (int)(tx + 0.5f);
      int iy = (int)(ty + 0.5f);
      if (ix < 0 || ix >= s->cols)
        continue;
      if (iy < 0 || iy >= rows_eff)
        continue;

      int slot = head_slot - (n - 1 - k);
      if (slot < 0)
        slot = 0;
      char glyph;
      int attr;
      if (k == n - 1) {
        /* The head; in velocity-debug mode it becomes a direction arrow. */
        if (g_debug == DBG_VELOCITY)
          glyph = dir_char(vx_phys, vy_phys);
        else
          glyph = (head_slot >= 5) ? '*' : (head_slot >= 2) ? '+' : '.';
        attr = (head_slot >= 6) ? A_BOLD : (head_slot <= 1) ? A_DIM : A_NORMAL;
      } else {
        glyph = (slot >= 4) ? '+' : (slot >= 1) ? '.' : '`';
        attr = (slot <= 1) ? A_DIM : A_NORMAL;
      }
      int pair = PAIR_RAMP_BASE + slot;
      attron(COLOR_PAIR(pair) | attr);
      mvaddch(iy, ix, (chtype)(unsigned char)glyph);
      attroff(COLOR_PAIR(pair) | attr);
    }
  }
}

/* ── §8 screen ── */

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

/* Draws the frame plus the two status lines: a stats readout along the top
 * and the key reminders along the bottom. The debug label is tacked on only
 * when a debug mode is active, to keep the top line tidy. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int active = scene_active_count(s);
  const PatternParams *pp = &pattern_params[s->current_pattern];

  const char *state_str =
      s->paused ? "PAUSED" : pattern_name(s->current_pattern);

  char top[160];
  if (g_debug == DBG_NORMAL) {
    snprintf(top, sizeof top,
             " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
             "N:%d oct:%d freq:%.3f ",
             fps, sim_fps, s->speed, state_str, themes[s->current_theme].name,
             active, pp->fbm_octaves, (double)pp->fbm_freq);
  } else {
    snprintf(top, sizeof top,
             " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
             "N:%d oct:%d freq:%.3f  [dbg:%s] ",
             fps, sim_fps, s->speed, state_str, themes[s->current_theme].name,
             active, pp->fbm_octaves, (double)pp->fbm_freq,
             k_debug_names[g_debug]);
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
           "  t/T:theme  +/-:speed  ]/[:Hz  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app ── */

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
    scene_populate_to_target(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_populate_to_target(s);
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
