/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* A misty forest at dawn with shafts of sunlight slanting between the trees.
 * For every character we shoot a ray and step it through the fog: where the fog
 * can see the sun through a gap in the trunks it lights up; where a trunk blocks
 * the sun it stays dark — and that contrast is what draws the shafts. Keys drift
 * the sun, reseed the forest, change the light's warmth, and zoom.
 * Sister files (same engine): god_rays_silhouette.c, solar_eclipse.c. */

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

/* ── §1  settings you can tweak ── */

/* §1.1 frame pacing and how fast the sun moves */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_REF = 8,   /* the speed value that means "normal" 1× sun motion */
  SPEED_DEF = 64,  /* startup speed = 8× normal, so the sun visibly drifts */
  SPEED_MAX = 128, /* fastest = 16× normal */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 the camera. It sits at eye height and looks dead ahead (no tilt), so the
 * slowly drifting sun naturally sweeps the top of the frame. ASPECT_Y squashes
 * the picture vertically so the tall terminal cells don't stretch it. */
#define ASPECT_Y 2.0f
#define FOV_H_BASE 0.55f /* base horizontal field of view (radians) */
#define ZOOM_MIN 0.25f
#define ZOOM_MAX 4.0f
#define ZOOM_STEP 1.25f

#define CAMERA_HEIGHT 1.6f /* eye height above the ground (metres) */

/* §1.3 the sun. Treated as infinitely far away, so it's just one direction.
 * It sits low and drifts slowly up/down and left/right on two different timers,
 * so its path doesn't obviously repeat. The disc is drawn bigger than the real
 * sun (which is tiny) so it actually shows up on a character grid. */
#define SUN_DIST 1000.0f     /* only used as a far cap for the shadow test */
#define SUN_ANG_RADIUS 0.07f /* apparent disc size (radians, ~4°) */
#define SUN_EMIT_HDR 14.0f   /* how bright the sun is (well past white) */

#define SUN_EL_BASE 0.22f      /* height above the horizon (radians, ~12.6°) */
#define SUN_EL_AMP 0.044f      /* how far it drifts up/down (±2.5°) */
#define SUN_AZ_BASE 0.0f       /* left-right centre */
#define SUN_AZ_AMP 0.087f      /* how far it drifts left/right (±5°) */
#define SUN_EL_PERIOD_S 200.0f /* seconds for one up/down drift */
#define SUN_AZ_PERIOD_S 280.0f /* seconds for one left/right drift */

/* §1.4 the forest. We split the view into a 5-wide × 4-deep grid of cells; each
 * cell independently gets a tree about 70% of the time, and the empty ~30% are
 * the gaps the shafts shine through. Far cells are spaced wider than near ones
 * so distant trees don't look unnaturally sparse. */
#define N_AZIM_BINS 5
#define N_DEPTH_BINS 4
#define N_TREES (N_AZIM_BINS * N_DEPTH_BINS)
#define TREE_FILL_PROB 0.70f /* chance each cell gets a tree */
#define TREE_Z_MIN 3.5f      /* nearest / farthest tree distance (m) */
#define TREE_Z_MAX 28.0f
#define AZIMUTH_HALF 0.55f /* half the left-right spread of trees (rad, ~31°) */
#define TREE_R_MIN 0.08f   /* trunk radius range (m) */
#define TREE_R_MAX 0.22f
#define TREE_H_MIN 4.5f    /* trunk height range (m) */
#define TREE_H_MAX 9.0f

/* §1.5 the flat ground (at height 0) and the near-black trunk colour, so the
 * trees stand out as dark silhouettes against the bright shafts. */
#define GROUND_Y 0.0f
#define GROUND_ALBEDO_R 0.18f /* ground colour, a dim brown */
#define GROUND_ALBEDO_G 0.13f
#define GROUND_ALBEDO_B 0.08f
#define GROUND_AMBIENT_FAC 0.25f /* how much sky-glow lights the shadowed ground */
#define TRUNK_COLOR_R 0.025f /* trunk colour, almost black */
#define TRUNK_COLOR_G 0.020f
#define TRUNK_COLOR_B 0.015f

/* §1.6 the fog. Thickest at the ground, thinning with height. That exact
 * thinning shape is the trick that lets us find how much light survives the path
 * to the sun with a single formula instead of a slow second march (§7). */
#define MIST_SIGMA 0.25f     /* fog thickness at ground level */
#define MIST_SCALE_H 1.0f    /* height over which it thins out (m) */
#define HG_G 0.55f           /* how forward-pointed the fog's glow is (0..1) */
#define MARCH_STEPS 32       /* samples taken along each eye ray */
#define FAR_CLIP 40.0f       /* how far to march a ray that hits nothing (m) */
#define SUN_NEE_DIST 50.0f   /* how far the shadow-to-sun test looks (m) */
#define INSCATTER_GAIN 2.25f /* overall shaft brightness */
#define EXTINCTION_EPS 1e-6f /* thinner fog than this → skip the step */
#define TRANSMITTANCE_CUTOFF 1e-3f /* stop once the ray is basically blocked */

/* §1.7 sky colours — warm near the horizon, deep blue overhead. */
#define SKY_ZENITH_R 0.08f /* the overhead colour */
#define SKY_ZENITH_G 0.12f
#define SKY_ZENITH_B 0.22f
#define SKY_HORIZON_FAC 0.40f /* horizon tint = sun colour × this */

/* §1.8 biggest frame we'll render, and the terminal colour-slot numbers we use. */
#define BUF_MAX_W 400
#define BUF_MAX_H 200

enum {
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_SUN_FALLBACK = 3, /* used when the terminal lacks 256 colours */
  PAIR_CUBE_BASE = 16,   /* slots 16..231 = the 6×6×6 colour grid */
};
#define CUBE_SIDE 6 /* 256-colour terminals give a 6×6×6 grid of colours */

/* §1.9 the dark-to-light characters we draw with; pick one by brightness. */
static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.9b a very bright cell is drawn bold and a very dark one dim, to squeeze a
 * little extra brightness range out of the terminal. */
#define CELL_BOLD_LUMA 0.85f
#define CELL_DIM_LUMA 0.15f

/* A named warmth setting for the sunlight. Lower numbers look redder (dawn,
 * dusk), higher numbers whiter (noon); blackbody_rgb (§5) turns the number into
 * an actual colour. `t`/`T` cycle through the presets.
 *   name    label shown in the HUD (fixed width)
 *   kelvin  colour temperature, in kelvin
 * (Uses the Tanner-Helland fit of Planck's blackbody law.) */
typedef struct {
  const char *name;
  float kelvin;
} KelvinPreset;

static const KelvinPreset KELVINS[] = {
    {"DAWN  ", 2000.0f},
    {"GOLDEN", 3500.0f},
    {"NOON  ", 5500.0f},
    {"DUSK  ", 2500.0f},
};
#define N_KELVINS ((int)(sizeof KELVINS / sizeof KELVINS[0]))

/* Which view the `d' key shows:
 *   MODE_NORMAL   the finished picture
 *   MODE_SCATTER  only the light shafts, everything else black
 *   MODE_SURFACE  only the trees/ground/sky, with no fog glow
 *   MODE_TR       how much the fog dims each ray, shown as grey
 *   MODE_N        how many modes there are (for wrap-around) */
typedef enum {
  MODE_NORMAL = 0,
  MODE_SCATTER = 1,
  MODE_SURFACE = 2,
  MODE_TR = 3,
  MODE_N = 4,
} DebugMode;

static const char *debug_mode_name(DebugMode m) {
  switch (m) {
  case MODE_NORMAL:
    return "NORMAL ";
  case MODE_SCATTER:
    return "SCATTER";
  case MODE_SURFACE:
    return "SURFACE";
  case MODE_TR:
    return "TR     ";
  default:
    return "?      ";
  }
}

/* §1.12 how we squeeze bright (HDR) light down to what the terminal can show.
 * TONE_WHITE is the brightness that comes out pure white; we set it to the sun's
 * brightness so the sun and the brightest shafts reach the top characters
 * distinctly instead of all blurring together near the top. TONE_EXPOSURE
 * brightens everything a touch first (low-res reads better that way). */
#define TONE_WHITE SUN_EMIT_HDR /* brightness that maps to pure white */
#define TONE_EXPOSURE 1.4f      /* overall brightness lift before the squeeze */

/* ── §2  timing helpers ── */

/* A clock that only counts forward, so animation timing isn't thrown off if the
 * system clock jumps (NTP, daylight saving, sleep/resume). */
static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep the leftover part of a frame so we don't peg a CPU core at 100%. */
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3  3-D vectors ── */

/* A point or direction in 3-D — camera, rays, sun, and tree positions all use
 * it. Passed by value (it's tiny) to keep the per-character loop fast. */
typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3_add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3_sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3_scl(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float v3_dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(V3 a) { return sqrtf(v3_dot(a, a)); }

/* Scale a vector to length 1 (same direction). Returns (0,0,0) for a zero-length
 * vector so we never divide by zero. */
static inline V3 v3_norm(V3 a) {
  float length = v3_len(a);
  if (length < 1e-12f)
    return v3(0, 0, 0);
  return v3_scl(a, 1.0f / length);
}

/* A ray: a start point and a direction — the straight line one character (or a
 * shadow check) looks along. origin and dir travel together, so they share one
 * struct. dir is kept length 1 so a hit distance comes out as a real distance. */
typedef struct {
  V3 origin;
  V3 dir;
} Ray;

/* ── §4  colours (as amounts of light) ── */

/* An amount of coloured light. Kept separate from V3 on purpose: these are light
 * values we add up and multiply along a ray, and they can climb well above 1.0
 * (HDR) until we squeeze them back to a displayable range at paint time. */
typedef struct {
  float r, g, b;
} RGB;

static inline RGB rgb_make(float r, float g, float b) { return (RGB){r, g, b}; }
static inline RGB rgb_add(RGB a, RGB b) {
  return rgb_make(a.r + b.r, a.g + b.g, a.b + b.b);
}
static inline RGB rgb_mul(RGB a, RGB b) {
  return rgb_make(a.r * b.r, a.g * b.g, a.b * b.b);
}
static inline RGB rgb_scl(RGB a, float s) {
  return rgb_make(a.r * s, a.g * s, a.b * s);
}
static inline RGB rgb_lerp(RGB a, RGB b, float t) {
  return rgb_make(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
                  a.b + (b.b - a.b) * t);
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float luma_of(RGB c) {
  return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

/* ── §5  warmth → colour ── */

/* Turn a colour temperature (kelvin) into an RGB tint: low = warm red, high =
 * cool white — the way a glowing-hot object actually looks. This is a cheap
 * published fit (Tanner-Helland); the magic numbers below are its coefficients. */
static RGB blackbody_rgb(float kelvin) {
  float K = kelvin / 100.0f;
  float r, g, b;

  if (K <= 66.0f)
    r = 1.0f;
  else
    r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

  if (K <= 66.0f)
    g = (99.47f * logf(K) - 161.12f) / 255.0f;
  else
    g = 288.12f * powf(K - 60.0f, -0.0755f) / 255.0f;

  if (K >= 66.0f)
    b = 1.0f;
  else if (K <= 19.0f)
    b = 0.0f;
  else
    b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

  return rgb_make(clamp01(r), clamp01(g), clamp01(b));
}

/* ── §6  squeezing bright light into a displayable range ── *
 *
 * Do this ONCE per cell, right at the end — never mid-sum. The squeeze is
 * non-linear, so squeezing a total is NOT the same as summing squeezed pieces. */

/* The brightness squeeze (see §1.12): leaves dark/mid tones almost untouched,
 * but bends the bright end so TONE_WHITE comes out exactly white and the sun and
 * shaft cores land on distinct top characters. TONE_EXPOSURE brightens first. */
static inline float reinhard(float x) {
  x *= TONE_EXPOSURE;
  return x * (1.f + x / (TONE_WHITE * TONE_WHITE)) / (1.f + x);
}
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* ── §7  the fog: how thick, and how much light gets through ── */

/* How thick the fog is at a point — thickest at the ground, thinning with
 * height. (Below-ground heights are clamped to 0 so the formula stays sane.) */
static inline float sigma_e_at(V3 scatter_point) {
  float y = (scatter_point.y < 0.f) ? 0.f : scatter_point.y;
  return MIST_SIGMA * expf(-y / MIST_SCALE_H);
}

/* How much sunlight survives the trip from a point up to the sun through the
 * fog. Because the fog thins exponentially with height, this is one quick
 * formula instead of a second slow march. Returns 0 when the sun is at or below
 * the horizon (the path would skim endless ground-level fog). */
static inline float tr_to_sun(V3 scatter_point, V3 sun_dir) {
  if (sun_dir.y < 1e-3f)
    return 0.0f;
  float y = (scatter_point.y < 0.f) ? 0.f : scatter_point.y;
  float optical_depth =
      MIST_SIGMA * expf(-y / MIST_SCALE_H) * MIST_SCALE_H / sun_dir.y;
  return expf(-optical_depth);
}

/* ── §8  which way fog scatters light ── */

/* Given the angle between where you're looking and the sun, how strongly the fog
 * scatters sunlight toward you. Tuned to favour the forward direction, so the
 * shafts show up mostly when you look roughly toward the sun. (The standard
 * Henyey-Greenstein function, 1941.) */
static inline float hg_phase(float cos_angle_eye_to_sun) {
  float g = HG_G;
  float g2 = g * g;
  float denom = 1.0f + g2 - 2.0f * g * cos_angle_eye_to_sun;
  if (denom < 1e-9f)
    denom = 1e-9f;
  return (1.0f - g2) / (4.0f * (float)M_PI * powf(denom, 1.5f));
}

/* ── §9  cheap randomness for the forest layout ── *
 *
 * The render itself is fully repeatable; the only randomness is where the trees
 * go, decided by a seed. These turn integers into repeatable pseudo-random
 * numbers. */

/* Scramble three integers into one repeatable pseudo-random number. */
static inline uint32_t hash3(int kx, int ky, int kz) {
  uint32_t h = (uint32_t)kx * 73856093u ^ (uint32_t)ky * 19349663u ^
               (uint32_t)kz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

/* Turn a hash into a float in [0, 1). */
static inline float hash01(uint32_t h) {
  return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* ── §10  drawing one character to the screen ── */

static int g_have_256; /* does the terminal have 256 colours? */

/* Set up the colour slots: the 6×6×6 colour grid on a 256-colour terminal, or a
 * couple of basic colours as a fallback. */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);
  if (g_have_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* Tone-map + gamma one HDR channel down to a displayable 0..1 value. */
static inline float display_channel(float linear) {
  return gamma_enc(reinhard(linear));
}

/* Pick a glyph from the density ramp for a 0..1 brightness. */
static inline char ramp_char(float luma) {
  int ri = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
  if (ri < 0)
    ri = 0;
  if (ri >= RAMP_LEN)
    ri = RAMP_LEN - 1;
  return k_ramp[ri];
}

/* Snap one 0..1 colour channel to one of the cube's 6 brightness levels. */
static inline int cube_level(float channel) {
  int q = (int)(channel * (float)(CUBE_SIDE - 1) + 0.5f);
  if (q > CUBE_SIDE - 1)
    q = CUBE_SIDE - 1;
  if (q < 0)
    q = 0;
  return q;
}

/* Displayable r,g,b → ncurses pair index into the 6×6×6 colour cube. */
static inline int cube_pair(float r, float g, float b) {
  return PAIR_CUBE_BASE + cube_level(r) * CUBE_SIDE * CUBE_SIDE +
         cube_level(g) * CUBE_SIDE + cube_level(b);
}

/* Brightness → emphasis attribute: bold the brightest cells, dim the darkest. */
static inline int cell_attr(float luma) {
  return (luma > CELL_BOLD_LUMA)   ? A_BOLD
         : (luma < CELL_DIM_LUMA) ? A_DIM
                                  : A_NORMAL;
}

static void paint_cell(int sx, int sy, RGB col) {
  float r = display_channel(col.r);
  float g = display_channel(col.g);
  float b = display_channel(col.b);
  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  char glyph = ramp_char(luma);

  int pair = PAIR_SUN_FALLBACK;
  int attr = A_NORMAL;
  if (g_have_256) {
    pair = cube_pair(r, g, b);
    attr = cell_attr(luma);
  }
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §11  ray vs the ground ── */

/* Where does a ray cross the flat ground? Only counts rays heading DOWN into it
 * (so looking up at the sky doesn't "hit" the floor), and only in front of us. */
static bool ray_plane_y(Ray ray, float plane_y, float *out_t) {
  if (ray.dir.y > -1e-6f)
    return false;
  float t = (plane_y - ray.origin.y) / ray.dir.y;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* ── §12  ray vs a tree trunk ── */

/* Where does a ray hit a trunk (a vertical cylinder, radius r, height h)? We
 * solve where it crosses the cylinder's side, then check that crossing is
 * actually on the trunk — a trunk is finite, so a ray can pass through its
 * column above the top or below the base and still miss. If the nearer crossing
 * is off the trunk we try the farther one. */
static bool ray_cylinder(Ray ray, V3 base, float r, float h, float *out_t) {
  float dx = ray.origin.x - base.x;
  float dz = ray.origin.z - base.z;
  float a = ray.dir.x * ray.dir.x + ray.dir.z * ray.dir.z;
  if (a < 1e-9f)
    return false;
  float b = 2.0f * (dx * ray.dir.x + dz * ray.dir.z);
  float c = dx * dx + dz * dz - r * r;
  float disc = b * b - 4.0f * a * c;
  if (disc < 0)
    return false;
  float sq = sqrtf(disc);
  float t_near = (-b - sq) / (2.0f * a);
  float t_far = (-b + sq) / (2.0f * a);
  if (t_far < 1e-3f)
    return false;

  /* the nearer crossing if it lands on the trunk, else the farther one */
  float t = (t_near >= 1e-3f) ? t_near : t_far;
  float y_at = ray.origin.y + t * ray.dir.y;
  if (y_at < base.y || y_at > base.y + h) {
    if (t == t_near) {
      t = t_far;
      y_at = ray.origin.y + t * ray.dir.y;
      if (y_at < base.y || y_at > base.y + h)
        return false;
    } else {
      return false;
    }
  }
  *out_t = t;
  return true;
}

/* ── §13  the forest: trees, the hit record, and the shadow test ── */

/* One tree trunk — just a vertical cylinder standing on the ground. That's the
 * whole forest: there are no leaves, because the shafts come from the GAPS
 * between trunks, so the trunks are all that matter to the light.
 *   base    where the trunk stands (x, 0, z)
 *   radius  trunk thickness (world units)
 *   height  trunk height; the trunk runs from y=0 up to y=height */
typedef struct {
  V3 base;
  float radius;
  float height;
} Tree;

/* What an eye ray ran into first, and where. SKY means it hit nothing and
 * escaped to the sky. trace_ray reads this to decide how to colour the far end.
 *   type      what was hit: sky (a miss), a tree, or the ground
 *   t         how far along the ray the hit is
 *   tree_idx  which tree was hit — only meaningful when type is a tree */
typedef enum { HIT_SKY = 0, HIT_TREE = 1, HIT_GROUND = 2 } HitType;

typedef struct {
  HitType type;
  float t;
  int tree_idx;
} Hit;

/* All the trees the rays travel through. Empty grid cells are skipped, so the
 * placed trees are packed into the front of the array.
 *   trees  the placed trees, valid for indices 0..count-1
 *   count  how many were actually placed (≤ N_TREES)
 * Rebuilt at startup and on `r` (reseed), then read-only while drawing. */
typedef struct {
  Tree trees[N_TREES];
  int count;
} Forest;

static Forest g_forest;

/* Sample a value in [lo, hi] at u ∈ [0,1] but spaced by equal RATIOS, so far
 * depth cells come out wider than near ones (matches how a forest looks). */
static inline float log_uniform(float lo, float hi, float u) {
  return lo * powf(hi / lo, u);
}

/* Scatter the trees: walk the grid of cells, and in each one roll the dice to
 * decide whether a tree appears and, if so, jitter its distance, angle, radius,
 * and height. Each random draw uses its own hash so the choices don't fall into
 * visible patterns. Same seed → same forest. */
static void place_trees(int seed) {
  int idx = 0;
  for (int dz = 0; dz < N_DEPTH_BINS; dz++) {
    for (int ax = 0; ax < N_AZIM_BINS; ax++) {
      uint32_t fill_hash = hash3(ax, dz, seed ^ 0x05000);
      float fill = hash01(fill_hash);
      if (fill > TREE_FILL_PROB)
        continue;

      uint32_t z_hash = hash3(ax, dz, seed ^ 0x10000);
      uint32_t a_hash = hash3(ax, dz, seed ^ 0x20000);
      uint32_t r_hash = hash3(ax, dz, seed ^ 0x30000);
      uint32_t h_hash = hash3(ax, dz, seed ^ 0x40000);

      /* this cell's distance range (far cells are wider) */
      float u_z0 = (float)dz / (float)N_DEPTH_BINS;
      float u_z1 = (float)(dz + 1) / (float)N_DEPTH_BINS;
      float z0 = log_uniform(TREE_Z_MIN, TREE_Z_MAX, u_z0);
      float z1 = log_uniform(TREE_Z_MIN, TREE_Z_MAX, u_z1);
      float jz = hash01(z_hash);
      float z = z0 + jz * (z1 - z0);

      /* this cell's left-right angle range */
      float u_a0 = ((float)(ax) / (float)N_AZIM_BINS - 0.5f) * 2.0f;
      float u_a1 = ((float)(ax + 1) / (float)N_AZIM_BINS - 0.5f) * 2.0f;
      float ja = hash01(a_hash);
      float u_a = u_a0 + ja * (u_a1 - u_a0);
      float angle = u_a * AZIMUTH_HALF;
      float x = z * tanf(angle);

      float u_r = hash01(r_hash);
      float u_h = hash01(h_hash);

      g_forest.trees[idx].base = v3(x, 0.0f, z);
      g_forest.trees[idx].radius = TREE_R_MIN + u_r * (TREE_R_MAX - TREE_R_MIN);
      g_forest.trees[idx].height = TREE_H_MIN + u_h * (TREE_H_MAX - TREE_H_MIN);
      idx++;
    }
  }
  g_forest.count = idx;
}

/* Is the straight path from a point toward the sun blocked by any trunk (within
 * max_distance)? This is what carves the shafts: fog with a clear path to the
 * sun glows, fog in a trunk's shadow stays dark. Checks every tree — fine for
 * the ~14 we have. */
static bool scene_blocked_to_sun(V3 scatter_point, V3 sun_dir,
                                 float max_distance) {
  for (int i = 0; i < g_forest.count; i++) {
    float t_tree;
    if (ray_cylinder((Ray){scatter_point, sun_dir}, g_forest.trees[i].base,
                     g_forest.trees[i].radius, g_forest.trees[i].height,
                     &t_tree)) {
      if (t_tree > 1e-3f && t_tree < max_distance)
        return true;
    }
  }
  return false;
}

/* ── §14  what does a ray hit first? ── */

/* Find the closest thing a ray meets — a tree, the ground, or (if neither) the
 * sky. Sky is the fallback for a ray that escapes the scene entirely. */
static Hit scene_hit(Ray ray) {
  Hit hit = {HIT_SKY, 1e30f, -1};

  /* Ground. */
  float t_ground;
  if (ray_plane_y(ray, GROUND_Y, &t_ground)) {
    if (t_ground < hit.t) {
      hit.type = HIT_GROUND;
      hit.t = t_ground;
      hit.tree_idx = -1;
    }
  }

  /* Trees. */
  for (int i = 0; i < g_forest.count; i++) {
    float t_tree;
    if (ray_cylinder(ray, g_forest.trees[i].base, g_forest.trees[i].radius,
                     g_forest.trees[i].height, &t_tree)) {
      if (t_tree < hit.t) {
        hit.type = HIT_TREE;
        hit.t = t_tree;
        hit.tree_idx = i;
      }
    }
  }

  return hit;
}

/*
 * SunSky — the frame's outdoor lighting environment, computed once per frame and
 * read by every shader. The sun is treated as directional (one direction at
 * infinity) plus an HDR emission colour; the sky's horizon tint completes both
 * the sky gradient and the ground's ambient term.
 *   sun_dir      unit direction toward the sun (the same from every point)
 *   sun_em       sun's emitted radiance (chroma × HDR) — the light source
 *   horizon_col  warm sky tint at the horizon (chroma × factor)
 */
typedef struct {
  V3 sun_dir;
  RGB sun_em;
  RGB horizon_col;
} SunSky;

/* ── §15  colour of the sky (and the sun disc) ── */

/* Colour for a ray looking at open sky: blend from the warm horizon up to deep
 * blue overhead, then add a bright spot if the ray points near the sun. */
static RGB sky_radiance(V3 ray_dir, SunSky sky) {
  float h = ray_dir.y;
  if (h < 0.f)
    h = 0.f;
  if (h > 1.f)
    h = 1.f;

  RGB zenith = rgb_make(SKY_ZENITH_R, SKY_ZENITH_G, SKY_ZENITH_B);
  RGB col = rgb_lerp(sky.horizon_col, zenith, h);

  float cos_ray_to_sun = v3_dot(ray_dir, sky.sun_dir);
  float cos_disc_edge = cosf(SUN_ANG_RADIUS);
  if (cos_ray_to_sun > cos_disc_edge) {
    float t_edge = (cos_ray_to_sun - cos_disc_edge) / (1.0f - cos_disc_edge);
    if (t_edge > 1.f)
      t_edge = 1.f;
    col = rgb_add(col, rgb_scl(sky.sun_em, t_edge));
  }
  return col;
}

/* ── §16  colouring the ground and the trees ── */

/* Colour of the ground at a point: some direct sunlight (only when the sun is up
 * AND no trunk blocks the path to it — that's the ground's own shadow), plus a
 * faint fill from the sky so shadowed ground isn't pure black. */
static RGB shade_ground(V3 ground_point, SunSky sky) {
  RGB albedo = rgb_make(GROUND_ALBEDO_R, GROUND_ALBEDO_G, GROUND_ALBEDO_B);
  float cos_normal_to_sun = sky.sun_dir.y;
  if (cos_normal_to_sun < 0.f)
    cos_normal_to_sun = 0.f;

  RGB direct = rgb_make(0.f, 0.f, 0.f);
  if (cos_normal_to_sun > 0.f) {
    bool blocked = scene_blocked_to_sun(ground_point, sky.sun_dir, SUN_DIST);
    if (!blocked) {
      float lambert = cos_normal_to_sun / (float)M_PI;
      direct = rgb_scl(rgb_mul(sky.sun_em, albedo), lambert);
    }
  }
  RGB ambient = rgb_scl(rgb_mul(sky.horizon_col, albedo), GROUND_AMBIENT_FAC);
  return rgb_add(direct, ambient);
}

/* Trunk colour: a fixed near-black, so trees read as silhouettes against the
 * bright shafts. (The tree index isn't used — every trunk is the same colour.) */
static RGB shade_tree(int tree_idx) {
  (void)tree_idx;
  return rgb_make(TRUNK_COLOR_R, TRUNK_COLOR_G, TRUNK_COLOR_B);
}

/* ── §17  the core: trace one ray through the fog ── *
 *
 * For each character: find what the ray hits, walk through the fog adding up the
 * sunlight scattered toward us (the shafts), then add whatever is behind the
 * fog, dimmed by however much fog it had to shine through. */

/* The light coming back along one ray, with the pieces kept separate so the
 * debug views can show each on its own. The finished pixel is `total`.
 *   total                the finished colour: shafts + (background, dimmed)
 *   in_scatter           just the light the fog scattered toward us (the shafts)
 *   surface              the tree/ground/sky behind the fog, at full strength
 *   t_far_transmittance  how much of that background survived the fog (0..1) */
typedef struct {
  RGB total;
  RGB in_scatter;
  RGB surface;
  float t_far_transmittance;
} Radiance;

/* Walk the ray through the fog in MARCH_STEPS small steps, adding up the sunlight
 * the fog scatters toward us — but only from steps that can actually see the sun
 * (no trunk in the way). Also tracks how much of the background still shows
 * through the fog and hands that back. */
static RGB march_in_scatter(Ray ray, SunSky sky, float march_distance,
                            float phase_value, float *out_transmittance) {
  float step_length = march_distance / (float)MARCH_STEPS;
  RGB in_scatter_radiance = rgb_make(0.f, 0.f, 0.f);
  float transmittance_along_eye = 1.0f;

  for (int i = 0; i < MARCH_STEPS; i++) {
    if (transmittance_along_eye <= TRANSMITTANCE_CUTOFF)
      break; /* eye ray is essentially opaque — nothing more gets through */

    float step_t = ((float)i + 0.5f) * step_length;
    V3 scatter_point = v3_add(ray.origin, v3_scl(ray.dir, step_t));
    float extinction = sigma_e_at(scatter_point);

    if (extinction > EXTINCTION_EPS) {
      /* can this spot see the sun, or is a trunk in the way? */
      bool blocked =
          scene_blocked_to_sun(scatter_point, sky.sun_dir, SUN_NEE_DIST);

      if (!blocked) {
        float transmittance_to_sun = tr_to_sun(scatter_point, sky.sun_dir);
        float step_contribution = extinction * phase_value *
                                  transmittance_to_sun * INSCATTER_GAIN *
                                  step_length;
        RGB add = rgb_scl(rgb_scl(sky.sun_em, step_contribution),
                          transmittance_along_eye);
        in_scatter_radiance = rgb_add(in_scatter_radiance, add);
      }

      /* the fog over this step also dims everything behind it */
      transmittance_along_eye *= expf(-extinction * step_length);
    }
  }
  *out_transmittance = transmittance_along_eye;
  return in_scatter_radiance;
}

/* Colour of whatever the ray finally hit — a dark trunk, the lit ground, or the
 * sky. This is the background the fog's glow sits in front of. */
static RGB shade_hit(const Hit *hit, Ray ray, SunSky sky) {
  switch (hit->type) {
  case HIT_TREE:
    return shade_tree(hit->tree_idx);
  case HIT_GROUND: {
    V3 ground_point = v3_add(ray.origin, v3_scl(ray.dir, hit->t));
    return shade_ground(ground_point, sky);
  }
  case HIT_SKY:
  default:
    return sky_radiance(ray.dir, sky);
  }
}

static Radiance trace_ray(Ray ray, SunSky sky) {
  /* what the eye ray hits, and how far to march: a surface hit's distance, or
   * FAR_CLIP for a sky-bound ray (sky rays still march the near-camera mist).
   */
  Hit hit = scene_hit(ray);
  float march_distance = (hit.type == HIT_SKY) ? FAR_CLIP : hit.t;
  if (march_distance > FAR_CLIP)
    march_distance = FAR_CLIP;

  /* integrate the in-scatter shafts; the march also reports how much of the far
   * background still reaches the eye. */
  float cos_eye_to_sun = v3_dot(ray.dir, sky.sun_dir);
  float phase_value = hg_phase(cos_eye_to_sun);
  float transmittance_along_eye;
  RGB in_scatter_radiance = march_in_scatter(
      ray, sky, march_distance, phase_value, &transmittance_along_eye);

  /* the background behind the mist, dimmed by the surviving transmittance. */
  RGB surface_radiance = shade_hit(&hit, ray, sky);
  RGB total = rgb_add(in_scatter_radiance,
                      rgb_scl(surface_radiance, transmittance_along_eye));

  Radiance result = {
      .total = total,
      .in_scatter = in_scatter_radiance,
      .surface = surface_radiance,
      .t_far_transmittance = transmittance_along_eye,
  };
  return result;
}

/* ── §18  the changing state: the sun clock and the forest ── */

/* Everything that changes while the program runs. scene_init/reseed/tick take
 * the whole Scene; everything else takes just the field it needs.
 *   time_secs   the sun's clock; scene_tick moves it forward
 *   seed        which forest layout we got
 *   paused      freeze the sun clock
 *   speed       how fast the sun moves (relative to SPEED_REF)
 *   kelvin_idx  which warmth preset is selected
 *   zoom        how zoomed-in the view is
 *   debug_mode  which view the `d' key shows */
typedef struct {
  float time_secs;
  int seed;
  bool paused;
  int speed;
  int kelvin_idx;
  float zoom;
  DebugMode debug_mode;
} Scene;

/* Which way the sun is right now: a unit direction pointing toward it. It drifts
 * slowly up/down and left/right on two timers, and always stays above the
 * horizon in this scene. */
static V3 scene_sun_dir(const Scene *s) {
  float omega_el = 2.0f * (float)M_PI / SUN_EL_PERIOD_S;
  float omega_az = 2.0f * (float)M_PI / SUN_AZ_PERIOD_S;
  float el = SUN_EL_BASE + SUN_EL_AMP * sinf(omega_el * s->time_secs);
  float az = SUN_AZ_BASE + SUN_AZ_AMP * sinf(omega_az * s->time_secs);
  float ce = cosf(el);
  return v3_norm(v3(sinf(az) * ce, sinf(el), cosf(az) * ce));
}

/* The `r' key: grow a fresh forest. Mixes in the current time so repeated
 * presses don't land on the same layout. */
static void scene_reseed(Scene *s) {
  uint32_t h = hash3((int)(s->time_secs * 1000.0f), s->seed, 0xC0FFEE);
  s->seed = (int)(h ^ 0x5A5A5A5Au);
  place_trees(s->seed);
}

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->kelvin_idx = 0; /* DAWN by default */
  s->zoom = 1.0f;
  s->seed = 0xF0E57u;
  s->debug_mode = MODE_NORMAL;
  place_trees(s->seed);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_REF;
  s->time_secs += dt * speed_mul;
}

/* ── §19  the camera: turning a cell into a ray ── */

/* A simple camera at eye height looking straight ahead. camera_ray turns a
 * screen cell into a world ray; the up/down field of view is worked out from the
 * across one and the cell shape so nothing looks stretched.
 *   pos          eye position, (0, CAMERA_HEIGHT, 0)
 *   fov_h, fov_v how wide the view is, across and up/down (radians)
 *   cols, rows   the image size this camera renders to */
typedef struct {
  V3 pos;
  float fov_h, fov_v;
  int cols, rows;
} Camera;

static void camera_make(Camera *c, int cols, int rows, float fov_h) {
  c->cols = cols;
  c->rows = rows;
  c->pos = v3(0.0f, CAMERA_HEIGHT, 0.0f);
  c->fov_h = fov_h;
  c->fov_v = fov_h * (float)rows * ASPECT_Y / (float)cols;
}

/* Turn a screen cell into a ray out from the eye. The minus sign on the vertical
 * term flips "row 0 is the top of the screen" into "up is +y in the world". */
static Ray camera_ray(const Camera *c, int sx, int sy) {
  float u =
      ((2.0f * (float)sx + 1.0f) - (float)c->cols) / (float)c->cols * c->fov_h;
  float v =
      -((2.0f * (float)sy + 1.0f) - (float)c->rows) / (float)c->rows * c->fov_v;
  return (Ray){c->pos, v3_norm(v3(u, v, 1.0f))};
}

/* ── §20  draw the whole frame ── */

/* The terminal we draw to — just its current size, re-read on resize so the
 * render and HUD always match the real window.
 *   cols, rows  terminal size, in characters */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static RGB g_buf[BUF_MAX_H][BUF_MAX_W];

/* Pick which piece of the traced light the current debug view wants to show. */
static RGB radiance_component(const Radiance *rad, DebugMode mode) {
  switch (mode) {
  default:
  case MODE_NORMAL:
    return rad->total;
  case MODE_SCATTER:
    return rad->in_scatter;
  case MODE_SURFACE:
    return rad->surface;
  case MODE_TR: {
    float t = rad->t_far_transmittance;
    return rgb_make(t, t, t);
  }
  }
}

/* Draw one frame: trace a ray for every cell into an off-screen buffer, then
 * paint the buffer to the terminal. */
static void scene_draw(const Screen *sc, const Scene *s) {
  /* leave the top and bottom rows for the HUD (unless the window is tiny), and
   * never exceed the fixed buffer size */
  int rows_eff = sc->rows - 2;
  int row_off = 1;
  if (rows_eff < 4) {
    rows_eff = sc->rows;
    row_off = 0;
  }
  if (rows_eff > BUF_MAX_H)
    rows_eff = BUF_MAX_H;
  int cols_eff = sc->cols;
  if (cols_eff > BUF_MAX_W)
    cols_eff = BUF_MAX_W;

  Camera cam;
  camera_make(&cam, cols_eff, rows_eff, FOV_H_BASE / s->zoom);

  float kelvin = KELVINS[s->kelvin_idx].kelvin;
  RGB sun_chrom = blackbody_rgb(kelvin);
  SunSky sky = {scene_sun_dir(s), rgb_scl(sun_chrom, SUN_EMIT_HDR),
                rgb_scl(sun_chrom, SKY_HORIZON_FAC)};

  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      Ray ray = camera_ray(&cam, c, r);
      Radiance rad = trace_ray(ray, sky);
      g_buf[r][c] = radiance_component(&rad, s->debug_mode);
    }
  }

  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      paint_cell(c, r + row_off, g_buf[r][c]);
    }
  }
}

/* ── §21  the status line + key hints ── */

/* Draw the on-screen text: a status line top-right (fps, sun angle, settings),
 * the title top-left, and the key hints along the bottom. */
static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  const KelvinPreset *k = &KELVINS[s->kelvin_idx];
  V3 sd = scene_sun_dir(s);
  float el_deg = asinf(sd.y) * 180.0f / (float)M_PI;
  float az_deg = atan2f(sd.x, sd.z) * 180.0f / (float)M_PI;

  char buf[256];
  snprintf(buf, sizeof buf,
           " %5.1f fps %3d Hz  %s  %s %5.0fK  sun:el%4.1f deg az%+5.1f deg  "
           "z:%3.1fx  spd:%d  %s ",
           fps, sim_fps, s->paused ? "PAUSED" : "      ", k->name,
           (double)k->kelvin, (double)el_deg, (double)az_deg, (double)s->zoom,
           s->speed, debug_mode_name(s->debug_mode));
  int len = (int)strlen(buf);
  if (len > sc->cols)
    len = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " FOREST-GOD-RAYS · VOLUMETRIC PT ");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reseed  d:debug  t/T:kelvin  z/Z:zoom  "
           "+/-:spd  []:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(sc, s);
  hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §22  startup, input, and the main loop ── */

/* The whole running program in one place: the scene, the terminal, the sim-rate
 * knob, and two flags the signal handlers flip.
 *   scene        the simulation and view settings
 *   screen       the terminal we draw to
 *   sim_fps      how many simulation steps per second ([ and ] change it)
 *   running      cleared to quit (by `q' or a signal)
 *   need_resize  set when the window changed; handled on the next loop */
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

  case 'd':
  case 'D':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % MODE_N);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    break;
  case '-':
  case '_':
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
    s->kelvin_idx = (s->kelvin_idx + 1) % N_KELVINS;
    break;
  case 'T':
    s->kelvin_idx = (s->kelvin_idx + N_KELVINS - 1) % N_KELVINS;
    break;

  case 'z':
    s->zoom *= ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    s->zoom /= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
    break;
  }
  return true;
}

static void app_init(App *app) {
  memset(app, 0, sizeof *app);
  scene_init(&app->scene);
  screen_init(&app->screen);
  app->sim_fps = SIM_FPS_DEFAULT;
  app->running = 1;
}

static void app_run(App *app) {
  int64_t prev = clock_ns();
  int64_t sim_accum = 0;
  int64_t frame_count = 0;
  int64_t fps_window_start = prev;
  double fps_meas = 0.0;

  struct sigaction sa = {0};
  sa.sa_handler = on_exit_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sa.sa_handler = on_resize_signal;
  sigaction(SIGWINCH, &sa, NULL);
  atexit(cleanup);

  while (app->running) {
    /* handle any keys the user pressed (these don't advance the sun) */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }
    if (!app->running)
      break;
    /* window was resized: re-read its size */
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
    }

    /* real time since the last loop, capped so one stall can't make the sun jump */
    int64_t now = clock_ns();
    int64_t dt = now - prev;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    prev = now;

    /* advance the sun in fixed steps — however many fit in the elapsed time.
     * scene_tick is the only thing that moves the sun (and it does nothing while
     * paused). */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, (float)tick_ns / (float)NS_PER_SEC);
      sim_accum -= tick_ns;
    }

    /* draw the frame and flip it to the screen */
    screen_draw(&app->screen, &app->scene, fps_meas, app->sim_fps);
    screen_present();

    /* update the fps number shown in the HUD */
    frame_count++;
    if (now - fps_window_start >= NS_PER_SEC) {
      fps_meas = (double)frame_count * (double)NS_PER_SEC /
                 (double)(now - fps_window_start);
      frame_count = 0;
      fps_window_start = now;
    }

    /* sleep any leftover time so we don't redraw faster than ~60 fps */
    int64_t target = clock_ns();
    int64_t left = TICK_NS(SIM_FPS_DEFAULT * 2) - (target - now);
    if (left > 0)
      clock_sleep_ns(left);
  }
}

int main(void) {
  app_init(&g_app);
  app_run(&g_app);
  cleanup();
  return 0;
}
