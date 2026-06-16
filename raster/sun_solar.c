/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_solar.c — a flat, screen-space sun in the terminal: a glowing disc with a
 * darker rim, drifting surface texture, sunspots, a soft halo, and flares that
 * arc out and fade. No 3-D and no ray tracing — for each character cell we just
 * ask how far it sits from the sun's centre and colour it from that.
 *
 * Sister files: raster/donut.c (also draws straight to the screen, no rays),
 * raymarcher/raymarcher.c (a real 3-D surface). The rim-darkening idea is
 * Eddington's (1926); the cloudy surface texture is value-noise fBm (Perlin
 * 1985; Quílez, iquilezles.org/articles/morenoise).
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

/* ── §1 settings — every number you can tweak, in one place ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  FPS_UPDATE_MS = 500,

  /* Rows reserved for the HUD; the sun renders in the rows between them. */
  HUD_TOP_ROWS = 1,    /* yellow status row at the top   */
  HUD_BOTTOM_ROWS = 1, /* cyan hint row at the bottom    */

  PAIR_HUD = 1,       /* colour slot for the top status row    */
  PAIR_HINT = 2,      /* colour slot for the bottom hint row   */
  PAIR_RAMP_BASE = 3, /* first of the 8 brightness colours     */

  N_FLARES_MAX = 12, /* most flares alive at once (a fixed pool) */

  ARC_SAMPLES = 36, /* dots drawn along each flare's arc        */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define CELL_ASPECT 2.0f /* a character cell is ~twice as tall as it is wide */

/* How big the sun is, as a fraction of the smaller screen side. */
#define DISC_FRAC 0.30f      /* disc radius vs. the smaller screen side  */
#define CORONA_MULT 1.7f     /* the halo reaches this many disc-radii out */
#define CORONA_FALLOFF 0.35f /* how fast the halo fades (smaller=tighter) */
#define CORONA_GAIN 0.45f    /* halo brightness right at the disc edge    */

/* Zoom: z grows the disc, Z shrinks it. */
#define ZOOM_DEFAULT 1.0f
#define ZOOM_MIN 0.40f
#define ZOOM_MAX 2.50f
#define ZOOM_STEP 0.15f

/* The disc is brightest in the middle and dimmer at the rim, like a real sun
 * (Eddington 1926).  The contrast is cranked up on purpose: with only 8
 * brightness steps a gentle fade leaves the disc looking flat, so we let the rim
 * go fairly dark while the centre stays full-bright. */
#define LIMB_BASE 0.45f /* brightness at the very rim          */
#define LIMB_BIAS 0.55f /* extra brightness added at the centre */

/* Granulation — the mottled, simmering texture of the sun's surface. */
#define GRAN_SCALE 0.18f /* texture zoom (smaller = bigger blobs)   */
#define GRAN_DRIFT 1.7f  /* how fast it drifts sideways, cells/sec  */
#define GRAN_AMP 0.35f   /* how strongly it lightens/darkens cells  */

/* Sunspots — the dark patches, where the surface texture dips darkest.  We take
 * how far below the cutoff a dip is (as a fraction of the cutoff) and square it,
 * so only the deepest dips go truly dark; an earlier version barely darkened at
 * all and the spots were invisible. */
#define SPOT_THRESH 0.40f /* texture below this starts a spot    */
#define SPOT_AMP 1.60f    /* how dark the spot centres get       */

/* Flares — the bright loops that erupt, arc out, and fade. */
#define FLARE_LIFETIME_MIN 4.0f   /* shortest flare life, seconds       */
#define FLARE_LIFETIME_MAX 9.0f   /* longest flare life, seconds        */
#define FLARE_LIFE_EXP 1.4f       /* fade shape: rises, holds, fades    */
#define FLARE_INTENSITY 0.55f     /* overall flare brightness           */
#define FLARE_INTENSITY_MAX 1.10f /* per-flare brightness, brightest    */
#define FLARE_INTENSITY_MIN 0.20f /* per-flare brightness, faintest     */
#define FLARE_VISIBLE_MIN 0.001f  /* don't bother drawing fainter ones  */
#define ARC_APEX_MIN 0.30f        /* how high the loop arcs, as a       */
#define ARC_APEX_MAX 0.85f        /*   fraction of the disc radius      */
#define ARC_FOOT_SPAN_MIN 0.40f /* how far apart the loop's two feet sit, */
#define ARC_FOOT_MAX_SPAN 1.60f /*   measured as an angle in radians      */

/* How often new flares appear (the +/- keys change it). */
#define SPAWN_RATE_DEFAULT 1.5f /* flares per second                  */
#define SPAWN_RATE_MIN 0.0f
#define SPAWN_RATE_MAX 12.0f
#define SPAWN_RATE_STEP 1.20f /* each key press multiplies by this    */

/* Turning a brightness number into a character + colour. */
#define LUM_CLAMP 1.05f     /* brightness above this counts as the max     */
#define LUMA_TIERS 8        /* how many brightness steps we draw with      */
#define LUMA_DRAW_MIN 0.02f /* dimmer than this → leave the cell blank     */
#define SLOT_BOLD_MIN 6     /* the top steps are drawn bold                */
#define SLOT_DIM_MAX 1      /* the bottom steps are drawn dim              */

/* One colour scheme for the star, switched with the t/T keys.  `ramp` lists a
 * colour for each of the 8 brightness steps, dark→bright, so the same brightness
 * paints deep red at the rim and near-white at the core under SOLAR, or
 * blue-white all over under BLUE_GIANT.  `name` is what the HUD shows. */
typedef struct {
  const char *name;
  short ramp[LUMA_TIERS];
} Theme;

#define N_THEMES 4

static const Theme themes[N_THEMES] = {
    /* SOLAR: dim red → orange → yellow → bone-white (real-sun colours) */
    {"SOLAR     ", {124, 160, 196, 202, 208, 214, 220, 229}},

    /* BLUE_GIANT: hot blue-white throughout (Rigel-class)              */
    {"BLUE_GIANT", {24, 31, 38, 45, 87, 123, 159, 195}},

    /* RED_DWARF: cool deep red glowing into amber                      */
    {"RED_DWARF ", {52, 88, 124, 160, 166, 202, 208, 214}},

    /* ALIEN: violet body climbing into electric cyan flare-ish core    */
    {"ALIEN     ", {53, 91, 134, 165, 207, 159, 123, 51}},
};

/* The characters we draw with, faint to blazing.  The first is '.', not a space,
 * so even the dimmest visible cell still shows a dot. */
static const char LUMA_GLYPHS[LUMA_TIERS] = {'.', ',', ':', ';',
                                             '+', '*', '#', '@'};

/* ── §2 clock — a steady timer and a sleep, to pace the frames ── */

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

/* ── §3 colour — load the terminal's colour slots ── */

/* Load the current theme's 8 colours into the ramp slots; called again whenever
 * you switch theme. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  const Theme *t = &themes[idx];

  if (COLORS >= 256) {
    for (int i = 0; i < LUMA_TIERS; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
  } else {
    static const short fb[LUMA_TIERS] = {
        COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
    };
    for (int i = 0; i < LUMA_TIERS; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── §4 building blocks for the surface texture ── */

/* Turn a pair of whole-number grid coordinates into a repeatable random-looking
 * number — same input always gives the same output. */
static inline uint32_t hash2d(int x, int z, uint32_t seed) {
  uint32_t h =
      (uint32_t)x * 374761393u + (uint32_t)z * 668265263u + seed * 2147483647u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}

/* Same idea, scaled to a 0..1 fraction. */
static inline float hash_unit(int x, int z, uint32_t seed) {
  return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

/* An ease curve: 0→0, 1→1, but flat at both ends, so blended noise has no hard
 * creases at the grid lines. */
static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

/* ── §5 value noise — smooth random bumps ── */

/* A smooth random value at any point: take the four nearest grid values and
 * blend between them (eased, so the result is gently rolling, not blocky). */
static float vnoise2d(float x, float z, uint32_t seed) {
  int xi = (int)floorf(x), zi = (int)floorf(z);
  float fx = x - (float)xi, fz = z - (float)zi;
  float v00 = hash_unit(xi, zi, seed);
  float v10 = hash_unit(xi + 1, zi, seed);
  float v01 = hash_unit(xi, zi + 1, seed);
  float v11 = hash_unit(xi + 1, zi + 1, seed);
  float sx = smoothstep01(fx);
  float sz = smoothstep01(fz);
  float a = v00 * (1.0f - sx) + v10 * sx;
  float b = v01 * (1.0f - sx) + v11 * sx;
  return a * (1.0f - sz) + b * sz;
}

/* ── §6 fBm — stack a few noise layers for detail at several sizes ── */

/* Add three layers of noise, each half as strong and twice as fine as the last,
 * so you get big soft blobs plus finer speckle on top — like real cloud texture. */
static float fbm2d(float x, float z, uint32_t seed) {
  float h = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
  for (int i = 0; i < 3; i++) {
    h += amp * vnoise2d(x * freq, z * freq, seed + (uint32_t)i * 17u);
    norm += amp;
    amp *= 0.5f;
    freq *= 2.0f;
  }
  return h / norm;
}

/* ── §7 surface brightness — rim darkening, texture, and spots combined ── */

/* The brightness of one cell inside the disc.  Three things combine: the disc is
 * dimmer toward the rim, the simmering texture lightens and darkens it, and the
 * deepest texture dips become dark sunspots. */
static float surface_lum(float dx, float dy, float r, float r_disc, float t,
                         uint32_t seed) {
  /* Pretend the disc is a real ball: how head-on are we looking here? 1 dead
   * centre, 0 at the rim — this is what makes the rim darker. */
  float r_norm = r / r_disc;
  if (r_norm > 1.0f)
    r_norm = 1.0f;
  float mu = sqrtf(1.0f - r_norm * r_norm);

  /* The surface texture at this point, sliding sideways as time passes. */
  float tex = fbm2d((dx - GRAN_DRIFT * t) * GRAN_SCALE, dy * GRAN_SCALE, seed);
  float tex_c = tex - 0.5f; /* shift so it can lighten or darken */

  /* Sunspots: where the texture dips well below the cutoff, darken hard.  We
   * measure the dip as a fraction of the cutoff and square it, so only deep dips
   * go truly dark while shallow ones fade out. */
  float spot = (SPOT_THRESH - tex) / SPOT_THRESH;
  if (spot < 0.0f)
    spot = 0.0f;
  spot *= spot; /* square it: dark cores, soft edges */

  /* Put it together: rim-darkened base brightness, nudged by texture and spots. */
  float base = LIMB_BASE + LIMB_BIAS * mu;
  float modu = 1.0f - GRAN_AMP * tex_c * 2.0f - SPOT_AMP * spot;
  if (modu < 0.0f)
    modu = 0.0f;

  return base * modu;
}

/* ── §8 corona — the halo's brightness, fading outward ── */

/* Brightness in the halo around the disc: full at the disc edge, fading fast the
 * farther out you go. */
static inline float corona_lum(float r, float r_disc) {
  float d = r - r_disc;
  return CORONA_GAIN * expf(-d / (r_disc * CORONA_FALLOFF));
}

/* ── §9 flares — one flare, the pool that holds them, and a random generator ── */

/* One flare: a bright loop that rises off the disc and fades.  It's described by
 * where it is in its life, the two points on the disc rim its loop springs from
 * (given as angles), how high the loop arcs, and how bright this one is. */
typedef struct {
  bool active;   /* is this slot in use?            */
  float age;     /* seconds since it appeared       */
  float lifetime; /* seconds it lives in total      */

  float theta_a, theta_b; /* the two rim points the loop springs from, as angles */
  float apex_height;      /* loop height, as a fraction of the disc radius */
  float intensity;        /* this flare's own brightness */
} Flare;

/* The fixed set of flares alive on the sun, plus the random generator that
 * spawns them.  The fixed size matters: when every slot is busy, spawning just
 * does nothing and the flare count stops climbing.  The generator sits here
 * because its only job is seeding new flares — keeping it next to them shows
 * that. */
typedef struct {
  Flare flares[N_FLARES_MAX]; /* the slots; .active marks the live ones */
  uint32_t rng;               /* random generator state                 */
} FlarePool;

/* A tiny random-number generator.  The exact constants don't matter — we only
 * want some variety, not real randomness. */
static inline uint32_t lcg_step(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}

static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_step(st) >> 8) / (float)(1u << 24);
}

/* A uniform random float in [lo, hi). */
static inline float rand_range(uint32_t *st, float lo, float hi) {
  return lo + lcg_unit(st) * (hi - lo);
}

/* ── §10 flare pool — make, age, and count flares ──
 *
 * How OFTEN flares appear is decided elsewhere (scene_tick); these just do the
 * work of clearing the pool, filling one free slot, ageing everyone, and
 * counting who's alive. */

static void flare_pool_clear(FlarePool *p) {
  memset(p->flares, 0, sizeof p->flares); /* wipe the slots; keep the rng */
}

/* Start a new flare in the first free slot (does nothing if all are busy).  The
 * two feet are set a random angle apart — too close and the loop is a thin
 * spike, too far and it flattens into a straight line across the disc. */
static void flare_pool_spawn(FlarePool *p) {
  int idx = -1;
  for (int i = 0; i < N_FLARES_MAX; i++) {
    if (!p->flares[i].active) {
      idx = i;
      break;
    }
  }
  if (idx < 0)
    return; /* pool full */

  Flare *f = &p->flares[idx];
  f->active = true;
  f->age = 0.0f;
  f->lifetime = rand_range(&p->rng, FLARE_LIFETIME_MIN, FLARE_LIFETIME_MAX);

  /* Two footpoints: a random angle, and a second one a random span away on
   * either side. */
  float theta_a = rand_range(&p->rng, 0.0f, 2.0f * (float)M_PI);
  float span = rand_range(&p->rng, ARC_FOOT_SPAN_MIN, ARC_FOOT_MAX_SPAN);
  float dir = (lcg_step(&p->rng) & 1) ? +1.0f : -1.0f; /* which side of A */
  f->theta_a = theta_a;
  f->theta_b = theta_a + dir * span;

  f->apex_height = rand_range(&p->rng, ARC_APEX_MIN, ARC_APEX_MAX);
  f->intensity = rand_range(&p->rng, FLARE_INTENSITY_MIN, FLARE_INTENSITY_MAX);
}

static void flare_pool_tick(FlarePool *p, float dt) {
  for (int i = 0; i < N_FLARES_MAX; i++) {
    if (!p->flares[i].active)
      continue;
    p->flares[i].age += dt;
    if (p->flares[i].age >= p->flares[i].lifetime)
      p->flares[i].active = false;
  }
}

static int flare_pool_active_count(const FlarePool *p) {
  int n = 0;
  for (int i = 0; i < N_FLARES_MAX; i++)
    if (p->flares[i].active)
      n++;
  return n;
}

/* ── §11 flare brightness over its life — fade in, hold, fade out ── */

/* How bright a flare is right now: 0 when it's born, up to 1 at the middle of
 * its life, back to 0 as it dies.  The exponent shapes that arc into a quick
 * rise, a held peak, and a quick fall rather than a plain triangle. */
static inline float flare_envelope(const Flare *f) {
  float tau = f->age / f->lifetime;
  float u = fabsf(2.0f * tau - 1.0f);
  return 1.0f - powf(u, FLARE_LIFE_EXP);
}

/* ── §12 flare arc — the curved path of one loop ── */

/* Given how far along the loop we are (s, from 0 to 1), hand back the screen spot
 * to light and how bright it should be there.  The loop rises from one foot,
 * bulges out to a high point, and comes down at the other foot — faintest at the
 * feet, brightest at the top. */
static inline void flare_arc_point(const Flare *f, float cx, float cy,
                                   float r_disc, float s, float *out_px,
                                   float *out_py, float *out_amp) {
  /* The two feet, on the disc rim.  Dividing the up/down part by CELL_ASPECT
   * keeps the rim a circle instead of a tall oval (cells are tall). */
  float ax = cx + cosf(f->theta_a) * r_disc;
  float ay = cy + sinf(f->theta_a) * r_disc / CELL_ASPECT;
  float bx = cx + cosf(f->theta_b) * r_disc;
  float by = cy + sinf(f->theta_b) * r_disc / CELL_ASPECT;

  /* Midpoint between the feet, and which way is "outward" from the sun's centre. */
  float mx = 0.5f * (ax + bx);
  float my = 0.5f * (ay + by);
  float ox = mx - cx;
  float oy = my - cy;
  float olen = sqrtf(ox * ox + oy * oy);
  if (olen < 1e-3f) { /* feet sit on the centre: just point up */
    ox = 0.0f;
    oy = -1.0f;
    olen = 1.0f;
  }
  float onx = ox / olen, ony = oy / olen;

  /* The top of the loop: out from the midpoint by the loop's height. */
  float ah = f->apex_height * r_disc;
  float apx = mx + onx * ah;
  float apy = my + ony * ah / CELL_ASPECT;

  /* Trace the curve: slide from foot to foot, pulled toward the top in the
   * middle (the pull is strongest at the halfway point, zero at the feet). */
  float bz = 4.0f * s * (1.0f - s);
  float px = (1.0f - s) * ax + s * bx + bz * (apx - mx);
  float py = (1.0f - s) * ay + s * by + bz * (apy - my);

  *out_px = px;
  *out_py = py;
  *out_amp = sinf((float)M_PI * s); /* brightest at the top, fading to the feet */
}

/* ── §13 scene — the whole world, and the one step that advances it ── */

/* Everything the program keeps track of, in one place:
 *   - the flares alive on the sun (the disc itself is recomputed every frame, so
 *     it stores nothing);
 *   - the knobs you can turn: how often flares appear, zoom, and pause;
 *   - where we are in time: the clock, the spawn counter, the texture seed;
 *   - things that are on screen but aren't part of the physics: the chosen
 *     colour theme and the canvas size.
 * Only scene_tick advances this over time; the init/reset/resize helpers and the
 * key handler change it too, but between frames, not as part of a tick. */
typedef struct {
  /* the flares on the sun */
  FlarePool flare_pool;

  /* the knobs you can turn */
  float spawn_rate; /* new flares per second                 */
  float zoom;       /* disc size, z/Z keys                   */
  bool paused;      /* space key freezes everything          */

  /* where we are in time */
  float time;        /* seconds elapsed                       */
  float spawn_accum; /* leftover credit toward the next flare */
  uint32_t seed;     /* fixes the surface texture pattern     */

  /* on screen but not part of the physics */
  int current_theme; /* which colour scheme                   */
  int cols, rows;    /* canvas size, copied from Screen on resize */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->current_theme = 0;
  s->cols = cols;
  s->rows = rows;
  s->seed = (uint32_t)clock_ns() ^ 0x1FACADEu;
  s->time = 0.0f;
  s->spawn_rate = SPAWN_RATE_DEFAULT;
  s->spawn_accum = 0.0f;
  s->zoom = ZOOM_DEFAULT;

  flare_pool_clear(&s->flare_pool);
  s->flare_pool.rng = (uint32_t)clock_ns() ^ 0xCAFEBABEu;
  /* Pre-spawn a couple so the screen isn't empty at t=0. */
  flare_pool_spawn(&s->flare_pool);
  flare_pool_spawn(&s->flare_pool);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset(Scene *s) {
  s->time = 0.0f;
  s->spawn_accum = 0.0f;
  s->zoom = ZOOM_DEFAULT;
  flare_pool_clear(&s->flare_pool);
  s->flare_pool.rng = (uint32_t)clock_ns() ^ 0xCAFEBABEu;
}

/* Move everything forward by one frame: advance the clock, age the flares, and
 * spawn new ones.  Spawning saves up "credit" at the chosen rate and starts a
 * flare each time the credit passes one whole flare — that way a fractional rate
 * like 1.5/sec works out smoothly over time. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
  flare_pool_tick(&s->flare_pool, dt);

  if (s->spawn_rate > 0.0f) {
    s->spawn_accum += dt * s->spawn_rate;
    while (s->spawn_accum >= 1.0f) {
      flare_pool_spawn(&s->flare_pool);
      s->spawn_accum -= 1.0f;
    }
  }
}

/* ── §14 drawing the sun — build a brightness map, then paint it ──
 *
 * We work in three passes over an off-screen brightness map: first the disc and
 * halo, then the flares added on top, then turn each cell's brightness into a
 * character and colour.  The map exists because flares ADD light onto whatever's
 * already there, which is awkward to do while painting directly. */

/* Biggest map we keep (sized for a large terminal); bigger terminals just clip. */
#define MAX_BUF_W 280
#define MAX_BUF_H 90

/* --- turning a brightness number into a character + colour --- */

/* Pin a raw brightness into range and rescale it to 0..1. */
static inline float normalize_luma(float L) {
  if (L < 0.0f)
    L = 0.0f;
  if (L > LUM_CLAMP)
    L = LUM_CLAMP;
  return L / LUM_CLAMP;
}

/* Pick which of the brightness steps a 0..1 value falls into. */
static inline int luma_to_slot(float Ln) {
  int slot = (int)(Ln * (LUMA_TIERS - 0.001f));
  if (slot < 0)
    slot = 0;
  if (slot > LUMA_TIERS - 1)
    slot = LUMA_TIERS - 1;
  return slot;
}

/* The brightest slots glow bold, the dimmest fade; the middle stays normal. */
static inline attr_t slot_attr(int slot) {
  return (slot >= SLOT_BOLD_MIN)  ? A_BOLD
         : (slot <= SLOT_DIM_MAX) ? A_DIM
                                  : A_NORMAL;
}

/* --- the three passes over the brightness map --- */

/* PASS 1 — fill every cell from its distance to the centre: textured sun inside
 * the disc, fading halo in the ring around it, dark space beyond. */
static void paint_disc_and_corona(float lum[MAX_BUF_H][MAX_BUF_W], int cols,
                                  int rows, float cx, float cy, float r_disc,
                                  float r_corona, float time, uint32_t seed) {
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      float dx = (float)col - cx;
      float dy = ((float)row - cy) * CELL_ASPECT; /* round disc on tall cells */
      float r = sqrtf(dx * dx + dy * dy);

      float L;
      if (r < r_disc)
        L = surface_lum(dx, dy, r, r_disc, time, seed);
      else if (r < r_corona)
        L = corona_lum(r, r_disc);
      else
        L = 0.0f;
      lum[row][col] = L;
    }
  }
}

/* PASS 2 — add each live flare's arc on top, sampled point by point and
 * brightest at the apex (the envelope fades it in over its lifetime). */
static void overlay_flares(float lum[MAX_BUF_H][MAX_BUF_W], int cols, int rows,
                           float cx, float cy, float r_disc,
                           const FlarePool *pool) {
  for (int i = 0; i < N_FLARES_MAX; i++) {
    const Flare *f = &pool->flares[i];
    if (!f->active)
      continue;
    float life_amp = flare_envelope(f);
    if (life_amp < FLARE_VISIBLE_MIN)
      continue;

    for (int k = 0; k <= ARC_SAMPLES; k++) {
      float s = (float)k / (float)ARC_SAMPLES; /* 0..1 along the arc */
      float px, py, arc_amp;
      flare_arc_point(f, cx, cy, r_disc, s, &px, &py, &arc_amp);

      /* Round to nearest cell.  Dense samples near the apex
       * pile into the same cell — the additive blend saturates
       * there, which reads as "arc is brightest at apex". */
      int xi = (int)(px + 0.5f);
      int yi = (int)(py + 0.5f);
      if (xi < 0 || xi >= cols)
        continue;
      if (yi < 0 || yi >= rows)
        continue;

      float boost = arc_amp * life_amp * f->intensity * FLARE_INTENSITY;
      lum[yi][xi] += boost;
    }
  }
}

/* PASS 3 — turn each cell's brightness into a character + colour and draw it.
 * We only tell the terminal to switch colour when it actually changes, so a run
 * of same-brightness cells doesn't reset the colour over and over. */
static void emit_cells(const float lum[MAX_BUF_H][MAX_BUF_W], int cols, int rows,
                       int y_offset) {
  int last_pair = -1;
  attr_t last_attr = 0;

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      float Ln = normalize_luma(lum[row][col]);

      if (Ln < LUMA_DRAW_MIN) { /* too dark — leave the cell blank */
        if (last_pair >= 0) {
          attroff(COLOR_PAIR(last_pair) | last_attr);
          last_pair = -1;
        }
        continue;
      }

      int slot = luma_to_slot(Ln);
      char glyph = LUMA_GLYPHS[slot];
      int pair = PAIR_RAMP_BASE + slot;
      attr_t attr = slot_attr(slot);

      if (pair != last_pair || attr != last_attr) {
        if (last_pair >= 0)
          attroff(COLOR_PAIR(last_pair) | last_attr);
        attron(COLOR_PAIR(pair) | attr);
        last_pair = pair;
        last_attr = attr;
      }
      mvaddch(row + y_offset, col, (chtype)(unsigned char)glyph);
    }
  }
  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void scene_render(const Scene *s) {
  /* Drawing area: leave the HUD rows alone, and don't exceed the map size. */
  int rows_eff = s->rows - (HUD_TOP_ROWS + HUD_BOTTOM_ROWS);
  int y_offset = HUD_TOP_ROWS;
  if (rows_eff < 1)
    return;
  int cols = s->cols;
  if (cols > MAX_BUF_W)
    cols = MAX_BUF_W;
  if (rows_eff > MAX_BUF_H)
    rows_eff = MAX_BUF_H;

  static float lum_buf[MAX_BUF_H][MAX_BUF_W];

  /* Where the sun sits and how big it is: centre of the screen, then the disc
   * and halo sizes (scaled by zoom). */
  float cx = (float)cols * 0.5f;
  float cy = (float)rows_eff * 0.5f;
  float min_dim = (float)cols < (float)rows_eff * CELL_ASPECT
                      ? (float)cols
                      : (float)rows_eff * CELL_ASPECT;
  float r_disc = min_dim * DISC_FRAC * s->zoom;
  float r_corona = r_disc * CORONA_MULT;

  paint_disc_and_corona(lum_buf, cols, rows_eff, cx, cy, r_disc, r_corona,
                        s->time, s->seed);
  overlay_flares(lum_buf, cols, rows_eff, cx, cy, r_disc, &s->flare_pool);
  emit_cells(lum_buf, cols, rows_eff, y_offset);
}

/* ── §15 screen — set up the terminal, draw the HUD, show the frame ── */

/* The terminal we draw into — just its size in character cells, re-checked
 * whenever the window resizes.  Kept separate from the Scene so the drawing code
 * can read the live size without touching (or accidentally changing) the
 * simulation; the Scene keeps its own copy for centring the sun. */
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
  typeahead(-1); /* don't let waiting keypresses interrupt drawing (avoids tearing) */
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
/* The endwin()+refresh() pair is what makes ncurses notice the new window size. */
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Draw the sun, then the two HUD rows on top: a yellow status line and a cyan
 * key hint.  The fps gets its own short label on the left so it stays readable
 * even when the longer status text is cut off on a narrow terminal. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_render(s);

  /* Top row — title + fps left, settings status right (truncated). */
  char left[48];
  snprintf(left, sizeof left, " SUN  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  char status[200];
  snprintf(status, sizeof status,
           " %s  theme:%s  zoom:%.2f  flares:%2d  spawn:%4.2f/s  "
           "t:%6.1fs  sim:%3dHz ",
           s->paused ? "PAUSED" : "BURNING", themes[s->current_theme].name,
           (double)s->zoom, flare_pool_active_count(&s->flare_pool),
           (double)s->spawn_rate,
           (double)s->time, sim_fps);
  int slen = (int)strlen(status);
  int max_slen = sc->cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, sc->cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reset  t/T:theme  +/-:spawn  "
           "z/Z:zoom  ]/[:fps ");
  clrtoeol();
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §16 app — wire it together: set up, run the loop, handle keys ── */

/* Everything the running program holds onto: the world (scene), the terminal it's
 * drawn on (screen), and the loop's own bookkeeping — the target frame rate and
 * two flags the signal handlers flip (time to quit, window was resized).  Just
 * the glue that the main loop drives; not part of the sun itself. */
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

/* Handle a window resize — happens between frames, not part of the animation. */
static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* Handle one key press — each is a one-off change, separate from the animation. */
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
    scene_reset(s);
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case '=':
  case '+':
    s->spawn_rate *= SPAWN_RATE_STEP;
    if (s->spawn_rate > SPAWN_RATE_MAX)
      s->spawn_rate = SPAWN_RATE_MAX;
    break;
  case '-':
    s->spawn_rate /= SPAWN_RATE_STEP;
    if (s->spawn_rate < SPAWN_RATE_MIN)
      s->spawn_rate = SPAWN_RATE_MIN;
    break;

  case 'z':
    /* zoom IN — bigger sun disc */
    s->zoom += ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    /* zoom OUT — smaller sun disc */
    s->zoom -= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
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
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* Window resized? rebuild for the new size before anything else. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* One frame, in order: measure time, advance, pace, draw, read input. */

    /* How long since the last frame? Cap it so a stall can't make the sun jump. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* Advance the sun by that much time. */
    float dt_sec = (float)dt / (float)NS_PER_SEC;
    scene_tick(&app->scene, dt_sec);

    /* Update the fps reading, then sleep off the rest of the frame to hold the rate. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t target_ns = TICK_NS(app->sim_fps);
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(target_ns - elapsed);

    /* Paint the frame and show it. */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    /* Read one keypress, if any, and act on it. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
