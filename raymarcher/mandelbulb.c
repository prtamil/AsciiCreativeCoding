/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb.c — a spiky 3-D fractal you can orbit around in the terminal.
 * It's the 2-D Mandelbrot idea ("does z = z² + c blow up?") lifted to 3-D,
 * with the squaring done in spherical coordinates and the power cranked to
 * 8.  For each character cell we shoot a ray and creep along it until it
 * touches the surface, then shade what we hit (soft shadows, crevice
 * darkening, depth-based colour).  Keys are listed on the bottom HUD line.
 *
 * Sister files raymarcher/raymarcher.c (sphere) and raymarcher_cube.c (box)
 * use the same ray march with a simple, exact distance; here the distance
 * is only a safe under-estimate, which needs the extra marching care in §6.
 *
 * Where the ideas come from:
 *   - the 3-D fractal: White & Nylander, "Mandelbulb" (2009),
 *     https://www.skytopia.com/project/fractal/mandelbulb.html
 *   - distance estimate for escape-time fractals: Hubbard & Douady; see
 *     Iñigo Quílez, https://iquilezles.org/articles/distancefractals/
 *   - creep-along-the-ray rendering: Hart, "Sphere Tracing" (1996)
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

/*
 * How the file is laid out — each job in its own area so it's easy to
 * follow and hard to break:
 *   §4–§6   the math: pure functions that take inputs and return answers —
 *           the Mandelbulb distance estimate + surface normal, and the ray
 *           march + soft shadow + shading.  (§7 adds the camera and the
 *           hit→character helpers.)  None of it touches shared state or the
 *           screen.
 *   §7      the state, plus the one thing that moves on its own: scene_tick
 *           nudges the camera's orbit angle each frame.  §7 also holds the
 *           per-cell render loop.
 *   §3,§8   drawing to the terminal: colour setup and ncurses I/O.
 *   §9      input (keys, resize, quit) — changes state, but between frames.
 *   §2,§9   timekeeping and pacing.
 * (No stored glows/trails and no scripted pauses, so two of the usual
 *  layers simply don't exist here.)
 *
 * One frame, in order: apply a pending resize, measure elapsed time, run
 * the sim forward, refresh fps + sleep to pace, draw, read a key.  Scene
 * (§7) is the single bundle of state; only scene_tick and the key/resize
 * handlers ever change it.
 */

/* §1  config — all the tunable numbers live here */

/* §1.1 frame rate + screen layout. */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500, /* how often to refresh the fps readout (ms) */
  HUD_ROWS = 2,        /* top + bottom rows are reserved for the HUD */
  ITERS_MIN = 3,
  ITERS_MAX = 14,
  ITERS_DEFAULT = 8, /* the classic Mandelbulb look */
};

/* §1.2 ncurses colour-pair slots. */
enum {
  PAIR_HUD = 1,       /* yellow status line (top)        */
  PAIR_HINT = 2,      /* cyan key reminders (bottom)     */
  PAIR_RAMP_BASE = 3, /* +0..+7 — the 8 depth colours    */
};

/* §1.3 time helpers + cell aspect. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define RENDER_FPS_CAP 60 /* frames/sec the draw loop is paced to */
/* If a frame takes longer than this (a resize, a stall), pretend it was
 * only this long, so the simulation can't lurch trying to catch up. */
#define MAX_FRAME_NS (100 * NS_PER_MS)

#define CELL_ASPECT 2.0f /* terminal cell h / w */

/* §1.4 the fractal shape. */
#define MANDELBULB_POWER 8.0f /* the exponent; 8 gives the classic spiky bulb */
#define BAILOUT 4.0f          /* once |z| passes this, the point has escaped  */

/* §1.5 ray-march tuning.  The distance we get is only a safe under-estimate,
 * so two knobs keep the march honest (see §6):
 *   MAX_STEPS        give up after this many steps along one ray
 *   HIT_EPS          base "close enough to count as a hit" distance
 *   ADAPTIVE_FACTOR  how fast that threshold loosens with distance
 *   MAX_T            if a ray gets this far out, it hit nothing
 *   STEP_RELAX       take steps a bit short (×0.85) so we never overshoot */
#define MAX_STEPS 90
#define HIT_EPS 1.0e-3f
#define ADAPTIVE_FACTOR 0.012f
#define MAX_T 6.0f
#define STEP_RELAX 0.85f

/* §1.6 how far to nudge when feeling which way the surface faces. */
#define NORMAL_EPS 3.5e-3f

/* §1.7 soft shadows.
 *   SHADOW_K      edge hardness — bigger = sharper-edged shadow
 *   SHADOW_FLOOR  how lit a fully-shadowed spot still stays (never black) */
#define SHADOW_STEPS 16
#define SHADOW_NEAR 0.012f
#define SHADOW_FAR 2.5f
#define SHADOW_K 32.0f
#define SHADOW_FLOOR 0.30f

/* §1.8 lighting + crevice darkening.
 *   AMBIENT      base brightness everywhere (nothing is pure black)
 *   AO_FLOOR     darkest a crevice can get
 *   AO_STRENGTH  how strongly deep crevices are darkened */
#define AMBIENT 0.18f
#define AO_FLOOR 0.35f
#define AO_STRENGTH 0.70f

/* Light direction in world space (up and to one side; normalised at use). */
#define LIGHT_X 0.55f
#define LIGHT_Y 0.75f
#define LIGHT_Z -0.25f

/* §1.9 camera. */
#define CAM_DIST_DEFAULT 3.2f
#define CAM_DIST_MIN 1.5f /* outside the bulb's bounding sphere */
#define CAM_DIST_MAX 8.0f
#define CAM_DIST_STEP 0.20f
#define FOV_DEG 45.0f
#define ORBIT_YAW_RATE 0.30f  /* rad / sec auto-orbit               */
#define ORBIT_PITCH_DEF 0.25f /* default static tilt above equator  */
#define MANUAL_YAW_STEP 0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX 1.30f /* clamp short of the poles           */

/* §1.10 quantisation — number of glyph / colour slots. */
#define LUMA_SLOTS 8         /* 8 brightness / colour steps */
#define LUMA_SLOT_FLT 7.999f /* scale a 0..1 value to a 0..7 slot, no overflow */

/* §1.11 colour themes.  Each is a name plus 8 colour codes running from the
 * outer shell (slot 0) to the deep interior (slot 7), cycled with t/T.
 *
 * One theme, NEGATIVE, is a photographic negative — dark fractal on a white
 * background.  Its `inverted` flag switches two things on: the background is
 * painted white (theme_apply / prefill_canvas), and the bold/dim emphasis is
 * dropped (luma_attr), since "bold = lighter" would fight a white page. */
typedef struct {
  const char *name;
  short ramp[LUMA_SLOTS]; /* 8 xterm-256 colour codes, outer shell → deep core */
  bool inverted;          /* true = dark-on-white photographic negative */
} Theme;

#define N_THEMES 5

static const Theme THEMES[N_THEMES] = {
    /* CLASSIC: warm crimson → red → orange → amber → bone — Daniel
     * White's iconic "alien fruit lit by sunset" palette. */
    {"CLASSIC ", {124, 160, 196, 202, 208, 214, 220, 229}, false},

    /* ICE: deep teal → bright cyan → ice → near-white. */
    {"ICE     ", {30, 37, 44, 51, 87, 123, 159, 195}, false},

    /* PLASMA: high-saturation neon arc — magenta → cyan → yellow. */
    {"PLASMA  ", {125, 165, 207, 213, 87, 123, 220, 229}, false},

    /* MONO: clean grayscale — best for studying fractal shape. */
    {"MONO    ", {240, 244, 247, 250, 252, 253, 254, 231}, false},

    /* NEGATIVE: photographic-negative inversion (see comment above). */
    {"NEGATIVE", {253, 250, 245, 240, 237, 234, 232, 16}, true},
};

/* §1.12 the shading characters, faint (`.`) to solid (`@`).  No blank at the
 * start, so even the dimmest hit still draws something visible. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {'.', ',', ':', ';',
                                             '+', '*', '#', '@'};

/* §2  clock — read the time, and sleep.  Pure timekeeping; the frame
 * pacing that uses it lives in §9. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                         .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* §3  colour setup (part of the RENDER layer; the rest is §7/§8).  Loads the
 * active theme's 8 depth colours plus the fixed HUD colours into ncurses.
 * theme_apply is split out so t/T can swap palettes without re-initialising
 * everything.  An inverted theme also sets a white background, which is what
 * lets prefill_canvas paint the page white before the fractal draws on it. */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  const Theme *t = &THEMES[idx];
  short bg256 = t->inverted ? 231 : -1;
  short bg8 = t->inverted ? COLOR_WHITE : -1;

  if (COLORS >= 256) {
    for (int i = 0; i < LUMA_SLOTS; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg256);
  } else {
    static const short FB[LUMA_SLOTS] = {
        COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_RED,  COLOR_RED,  COLOR_YELLOW,  COLOR_WHITE,
    };
    for (int i = 0; i < LUMA_SLOTS; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), t->inverted ? COLOR_BLACK : FB[i],
                bg8);
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

/* The math layer (§4-§6, plus the camera/ray and hit→character helpers in
 * §7): every function here just takes its inputs and returns an answer — no
 * shared state, no screen — so the drawing code can't change its results. */

/* §4  vec3 — a 3-D point or direction (x, y, z) and the usual math on it. */

typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3scale(float s, V3 a) {
  return v3(s * a.x, s * a.y, s * a.z);
}
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline V3 v3cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float L = v3len(a);
  return (L > 1e-12f) ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/* §5  the fractal itself.  Repeatedly fold a point: convert to spherical
 * coordinates, raise it to the 8th power (which spins its angles and stretches
 * its radius), and add back the starting point.  Points that stay put are
 * inside the bulb; points that fly off are outside.  How fast they fly off,
 * tracked alongside a running "spread" factor, tells us roughly how far the
 * point is from the surface — which is what the ray march needs. */

/* A point in spherical coordinates — like latitude/longitude plus a radius:
 * r = distance from centre, theta = angle from the +y pole, phi = angle
 * around the y axis. */
typedef struct {
  float r, theta, phi;
} Spherical;

static inline Spherical to_spherical(V3 z) {
  float r = sqrtf(z.x * z.x + z.y * z.y + z.z * z.z);
  Spherical s = {r, 0.0f, 0.0f};
  if (r > 1e-20f) { /* skip the angles at the origin (avoid 0/0) */
    s.theta = acosf(z.y / r);
    s.phi = atan2f(z.z, z.x);
  }
  return s;
}

/* update the running "how fast does z spread out" factor used by the
 * distance estimate, one step at a time */
static inline float update_dr(float dr, float r) {
  return powf(r, MANDELBULB_POWER - 1.0f) * MANDELBULB_POWER * dr + 1.0f;
}

/* one fold: raise z to the 8th power in spherical form, then add c */
static inline V3 apply_power_and_add(Spherical s, V3 c) {
  float zr = powf(s.r, MANDELBULB_POWER);
  float p_th = MANDELBULB_POWER * s.theta;
  float p_ph = MANDELBULB_POWER * s.phi;
  float sin_th = sinf(p_th);
  return v3(zr * sin_th * cosf(p_ph) + c.x, zr * cosf(p_th) + c.y,
            zr * sin_th * sinf(p_ph) + c.z);
}

/* smooth_escape_count — a fractional version of "how many folds until it flew
 * off", so the colours blend smoothly instead of banding at whole-number
 * boundaries.  i = the fold it escaped on, escape_r = how far out it was. */
static float smooth_escape_count(int i, int max_iter, float escape_r) {
  if (i >= max_iter)
    return (float)max_iter; /* never escaped */
  float ln_r = logf(escape_r);
  float log_bail = logf(BAILOUT);
  if (ln_r > 0.0f && log_bail > 0.0f)
    return (float)i + 1.0f - log2f(ln_r / log_bail) / log2f(MANDELBULB_POWER);
  return (float)i;
}

/* How far point p is from the fractal surface — a safe under-estimate, which
 * is all the ray march needs.  Optionally also reports the escape count for
 * colouring (pass NULL to skip it, e.g. when computing normals). */
static float mandelbulb_de(V3 p, int max_iter, float *smooth_out) {
  V3 z = p;
  float dr = 1.0f;
  Spherical s = {0.0f, 0.0f, 0.0f};
  int i;

  for (i = 0; i < max_iter; i++) {
    s = to_spherical(z);
    if (s.r > BAILOUT)
      break;
    dr = update_dr(dr, s.r);
    z = apply_power_and_add(s, p);
  }

  if (smooth_out)
    *smooth_out = smooth_escape_count(i, max_iter, s.r);
  return 0.5f * logf(s.r) * s.r / dr;
}

/* Which way the surface faces at p (its "normal"), needed for lighting.  We
 * can't read it directly, so we check how the distance changes a tiny step
 * each way along x, y, z — that points away from the surface.  Sampling both
 * sides of each axis keeps the result even (no lean toward one corner). */
static V3 mandelbulb_normal(V3 p, int max_iter) {
  float e = NORMAL_EPS;
  float dx = mandelbulb_de(v3(p.x + e, p.y, p.z), max_iter, NULL) -
             mandelbulb_de(v3(p.x - e, p.y, p.z), max_iter, NULL);
  float dy = mandelbulb_de(v3(p.x, p.y + e, p.z), max_iter, NULL) -
             mandelbulb_de(v3(p.x, p.y - e, p.z), max_iter, NULL);
  float dz = mandelbulb_de(v3(p.x, p.y, p.z + e), max_iter, NULL) -
             mandelbulb_de(v3(p.x, p.y, p.z - e), max_iter, NULL);
  return v3norm(v3(dx, dy, dz));
}

/* §6  follow a ray to the surface, then light the spot it hits.  sphere_trace
 * creeps along the ray until it touches something; soft_shadow checks whether
 * the light can reach that spot; shade turns it all into one brightness. */

/* What a ray found: did it hit, where, and how many steps it took (the step
 * count doubles as a cheap "how tucked-away is this" measure). */
typedef struct {
  bool hit;
  V3 p;
  int march_steps;
} TraceResult;

static TraceResult sphere_trace(V3 origin, V3 dir, int max_iter) {
  TraceResult tr = {false, {0, 0, 0}, 0};
  float t = 0.0f;
  int step;

  for (step = 0; step < MAX_STEPS; step++) {
    V3 p = v3add(origin, v3scale(t, dir));
    float d = mandelbulb_de(p, max_iter, NULL);
    /* "close enough" threshold widens with distance — far cells cover more
     * world space, so they need less precision (saves march steps). */
    float eps = HIT_EPS * (1.0f + t * ADAPTIVE_FACTOR);

    if (d < eps) {
      tr.hit = true;
      tr.p = p;
      tr.march_steps = step;
      return tr;
    }
    if (t > MAX_T)
      break;
    t += d * STEP_RELAX;
  }
  tr.march_steps = step;
  return tr;
}

/* How much light reaches this spot, 0 (blocked) to 1 (clear).  March a short
 * way toward the light; if something almost grazes the path the spot is
 * partly shadowed, which gives soft edges for free (Christensen's trick). */
static float soft_shadow(V3 origin, V3 light_dir, int max_iter) {
  float result = 1.0f;
  float t = SHADOW_NEAR; /* start a little off the surface so it doesn't shadow itself */

  for (int i = 0; i < SHADOW_STEPS; i++) {
    V3 p = v3add(origin, v3scale(t, light_dir));
    float d = mandelbulb_de(p, max_iter, NULL);

    if (d < HIT_EPS)
      return SHADOW_FLOOR; /* fully blocked */

    float k = SHADOW_K * d / t;
    if (k < result)
      result = k;

    t += d;
    if (t > SHADOW_FAR)
      break;
  }
  return clampf(result, SHADOW_FLOOR, 1.0f);
}

/* Final brightness of a hit spot (0..1): a base glow, scaled by how squarely
 * it faces the light, then dimmed by shadow and by how tucked-away it is.
 *
 * The light is "wrapped" around the surface (half-Lambert, Valve/Mitchell
 * 2006) rather than cut off at the terminator: normally the side facing away
 * goes flat black and loses its shape — fatal here, since the character shows
 * the brightness — so even the far side keeps a gentle gradient.  The
 * tucked-away darkening reuses the trace's step count (deep folds take more
 * steps), so it's essentially free. */
static float shade(V3 hit_p, V3 normal, V3 light_dir, int max_iter,
                   int march_steps) {
  /* how much the surface faces the light, wrapped from [-1,1] to [0,1] */
  float ndl = v3dot(normal, light_dir);
  float diffuse = ndl * 0.5f + 0.5f;
  diffuse *= diffuse;

  float soft = soft_shadow(hit_p, light_dir, max_iter);

  float ao = 1.0f - ((float)march_steps / (float)MAX_STEPS) * AO_STRENGTH;
  if (ao < AO_FLOOR)
    ao = AO_FLOOR;

  return clampf(AMBIENT + (1.0f - AMBIENT) * diffuse * soft * ao, 0.0f, 1.0f);
}

/* §7  the state and the things around it: the Scene plus scene_tick (which
 * moves the orbit), the camera, the per-cell decorators, and the render
 * loop.  Only scene_tick and the key/resize handlers ever change a Scene. */

/* Orbit — where the camera sits, always looking at the bulb's centre.
 * Kept as the auto-orbit angles (advanced by scene_tick) plus the manual
 * offsets the arrow keys nudge; orbit_to_camera simply sums the two. */
typedef struct {
  float dist;                 /* distance from the origin (zoom)       */
  float yaw, pitch;           /* auto-orbit angles (yaw auto-advances) */
  float user_yaw, user_pitch; /* manual offsets from the arrow keys    */
} Orbit;

/* Scene — all simulation state, as a table of contents:
 *   WHAT is shown    iters          Mandelbulb iteration depth (3..14)
 *   HOW it's viewed  orbit          camera viewpoint (see §7.1)
 *                    current_theme  colour palette
 *                    paused         freezes the auto-orbit
 *   WHERE            cols, rows     terminal size in cells */
typedef struct {
  int iters;

  Orbit orbit;
  int current_theme;
  bool paused;

  int cols, rows;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->current_theme = 0;
  s->iters = ITERS_DEFAULT;
  s->cols = cols;
  s->rows = rows;
  s->orbit.dist = CAM_DIST_DEFAULT;
  s->orbit.yaw = 0.5f;
  s->orbit.pitch = ORBIT_PITCH_DEF;
  s->orbit.user_yaw = 0.0f;
  s->orbit.user_pitch = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset_cam(Scene *s) {
  s->orbit.dist = CAM_DIST_DEFAULT;
  s->orbit.yaw = 0.5f;
  s->orbit.pitch = ORBIT_PITCH_DEF;
  s->orbit.user_yaw = 0.0f;
  s->orbit.user_pitch = 0.0f;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->orbit.yaw += ORBIT_YAW_RATE * dt;
  if (s->orbit.yaw > (float)(2.0 * M_PI))
    s->orbit.yaw -= (float)(2.0 * M_PI);
}

/* §7.1 the camera.  orbit_to_camera turns the Orbit viewpoint into the eye
 * position and the three "which way is forward / right / up" directions;
 * pixel_ray then fires one ray per cell from those. */

/* Camera — the eye point plus its three view directions, and the lens
 * settings (field of view, and the squash that keeps tall cells from
 * stretching the picture). */
typedef struct {
  V3 origin;
  V3 fwd, right, up;
  float fov_t;
  float phys_aspect;
} Camera;

static Camera orbit_to_camera(const Orbit *o, int cols, int rows_eff) {
  float yaw = o->yaw + o->user_yaw;
  float pitch = clampf(o->pitch + o->user_pitch, -MANUAL_PITCH_MAX,
                       MANUAL_PITCH_MAX);

  Camera c;
  c.origin = v3(o->dist * cosf(pitch) * cosf(yaw), o->dist * sinf(pitch),
                o->dist * cosf(pitch) * sinf(yaw));
  c.fwd = v3norm(v3sub(v3(0, 0, 0), c.origin));
  V3 wup = v3(0, 1, 0);
  c.right = v3norm(v3cross(c.fwd, wup));
  c.up = v3cross(c.right, c.fwd);
  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)cols;
  return c;
}

/* Direction of the ray through one cell.  Row 0 is the top, and the tall-cell
 * stretch is undone so the bulb comes out round, not squashed. */
static V3 pixel_ray(int col, int row, int cols, int rows_eff, const Camera *c) {
  float u = ((float)col + 0.5f) / (float)cols * 2.0f - 1.0f;
  float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
  V3 sx = v3scale(u * c->fov_t, c->right);
  V3 sy = v3scale(v * c->fov_t * c->phys_aspect, c->up);
  return v3norm(v3add(c->fwd, v3add(sx, sy)));
}

/* §7.2 work out what to draw for a hit, then draw it.  assemble_hit /
 * shade_hit / to_slot just compute a glyph + colour; emit_cell and
 * prefill_canvas do the actual writing to the terminal. */

/* Everything about one cell's hit that the colouring needs. */
typedef struct {
  bool hit;
  V3 p;
  V3 normal;       /* which way the surface faces */
  float smooth;    /* escape count → colour */
  float luminance; /* final brightness, 0..1 */
  int march_steps; /* how tucked-away the spot is (crevice darkening) */
} Hit;

/* Fill in the normal, colour value, and brightness for a hit.  All the
 * costly per-hit distance-function work happens here, in one place. */
static Hit assemble_hit(TraceResult tr, int max_iter, V3 light) {
  Hit h = {tr.hit, tr.p, v3(0, 1, 0), 0.0f, 0.0f, tr.march_steps};
  if (!tr.hit)
    return h;

  h.normal = mandelbulb_normal(tr.p, max_iter);
  (void)mandelbulb_de(tr.p, max_iter, &h.smooth);
  h.luminance = shade(tr.p, h.normal, light, max_iter, tr.march_steps);
  return h;
}

/* One terminal cell, ready to print: the character, its colour, and a
 * bold/dim/normal attribute.  pair < 0 means "the ray missed — leave this
 * cell alone" (so the background shows through). */
typedef struct {
  char glyph;
  int pair;
  attr_t attr;
} Cell;

/* map a 0..1 value to one of the 8 brightness/colour slots */
static int to_slot(float x_01) {
  int s = (int)(x_01 * LUMA_SLOT_FLT);
  if (s < 0)
    s = 0;
  if (s >= LUMA_SLOTS)
    s = LUMA_SLOTS - 1;
  return s;
}

/* Bold for the brightest slots, dim for the darkest, to stretch the contrast
 * of a short character ramp.  Turned off on the white-background theme, where
 * "bold = lighter" would work against us. */
static attr_t luma_attr(int slot, bool inverted) {
  if (inverted)
    return A_NORMAL;
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Pick the character and colour for one hit: the character shows brightness,
 * the colour shows how deep into the fractal the point is. */
static Cell shade_hit(const Hit *h, int max_iter, bool inverted) {
  if (!h->hit) {
    return (Cell){' ', -1, 0}; /* miss → don't paint */
  }
  int s_lum = to_slot(h->luminance);
  int s_clr = to_slot(h->smooth / (float)max_iter);
  return (Cell){
      .glyph = LUMA_GLYPHS[s_lum],
      .pair = PAIR_RAMP_BASE + s_clr,
      .attr = luma_attr(s_lum, inverted),
  };
}

/* Print one cell.  Changing colour in ncurses isn't free, so we only switch
 * when this cell differs from the last — a big saving across flat regions. */
static void emit_cell(int row, int col, Cell c, int *last_pair,
                      attr_t *last_attr) {
  if (c.pair < 0)
    return; /* the ray missed — leave the cell as-is */

  if (c.pair != *last_pair || c.attr != *last_attr) {
    if (*last_pair >= 0)
      attroff(COLOR_PAIR(*last_pair) | *last_attr);
    attron(COLOR_PAIR(c.pair) | c.attr);
    *last_pair = c.pair;
    *last_attr = c.attr;
  }
  mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/* For the white-background theme only: paint the whole canvas white first, so
 * the cells the fractal misses are left showing white. */
static void prefill_canvas(int y0, int rows_eff, int cols, bool inverted) {
  if (!inverted)
    return;
  attron(COLOR_PAIR(PAIR_RAMP_BASE));
  for (int row = 0; row < rows_eff; row++)
    for (int col = 0; col < cols; col++)
      mvaddch(y0 + row, col, ' ');
  attroff(COLOR_PAIR(PAIR_RAMP_BASE));
}

/* §7.3 draw the whole frame: for every cell, shoot a ray, see what it hits,
 * and paint it.  Reads the scene, only writes the screen. */
static void scene_render(const Scene *s) {
  int rows_eff = s->rows - HUD_ROWS;
  if (rows_eff < 1)
    return;

  bool inverted = THEMES[s->current_theme].inverted;
  int y0 = 1; /* shift down 1 for the top HUD row */

  prefill_canvas(y0, rows_eff, s->cols, inverted);

  Camera cam = orbit_to_camera(&s->orbit, s->cols, rows_eff);
  V3 light = v3norm(v3(LIGHT_X, LIGHT_Y, LIGHT_Z));

  int last_pair = inverted ? PAIR_RAMP_BASE : -1;
  attr_t last_attr = 0;

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      V3 ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
      TraceResult tr = sphere_trace(cam.origin, ray, s->iters);
      Hit h = assemble_hit(tr, s->iters, light);
      Cell c = shade_hit(&h, s->iters, inverted);
      emit_cell(y0 + row, col, c, &last_pair, &last_attr);
    }
  }

  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* §8  screen — start/stop ncurses, draw the HUD, push the frame out. */

/* The terminal's current size in character cells. */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE); /* getch() returns at once if no key is waiting */
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt our drawing */
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

/* The endwin + refresh dance makes ncurses notice the new terminal size. */
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Draw the two HUD rows: title + status along the top, key reminders along
 * the bottom.  Both stay bold so they're legible over any fractal colour,
 * including the white-background theme. */
static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  char status[140];
  snprintf(status, sizeof status,
           " %5.1f fps  %3d Hz  theme:%s  iters:%2d  dist:%4.2f  %s ", fps,
           sim_fps, THEMES[s->current_theme].name, s->iters,
           (double)s->orbit.dist, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > sc->cols)
    slen = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - slen, "%s", status);
  mvprintw(0, 0, " MANDELBULB ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reset  t/T:theme  i/I:iters  "
           "z/Z:zoom  arrows:orbit ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_render(s);
  hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §9  input + the main loop.  Keys and resize change the state between
 * frames; main() is the one place that runs everything each frame, in order
 * (apply input → step the sim → draw → pace), and holds the frame rate. */

/* Everything the running program owns: the scene, the terminal size, the
 * target sim rate, and two flags the signal handlers flip. */
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
    scene_reset_cam(s);
    s->iters = ITERS_DEFAULT;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'i':
    if (s->iters > ITERS_MIN)
      s->iters--;
    break;
  case 'I':
    if (s->iters < ITERS_MAX)
      s->iters++;
    break;

  case 'z':
    s->orbit.dist -= CAM_DIST_STEP;
    if (s->orbit.dist < CAM_DIST_MIN)
      s->orbit.dist = CAM_DIST_MIN;
    break;
  case 'Z':
    s->orbit.dist += CAM_DIST_STEP;
    if (s->orbit.dist > CAM_DIST_MAX)
      s->orbit.dist = CAM_DIST_MAX;
    break;

  case KEY_LEFT:
    s->orbit.user_yaw -= MANUAL_YAW_STEP;
    break;
  case KEY_RIGHT:
    s->orbit.user_yaw += MANUAL_YAW_STEP;
    break;
  case KEY_UP:
    s->orbit.user_pitch += MANUAL_PITCH_STEP;
    break;
  case KEY_DOWN:
    s->orbit.user_pitch -= MANUAL_PITCH_STEP;
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

/* update_fps — count this frame; every FPS_UPDATE_MS recompute the rate to
 * show and start a fresh window.  Returns the value to display (unchanged
 * between refreshes); updates the two accumulators it's handed. */
static double update_fps(int64_t *fps_accum, int *frame_count, int64_t dt,
                         double current) {
  (*frame_count)++;
  *fps_accum += dt;
  if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS)
    return current;
  double fps = (double)*frame_count / ((double)*fps_accum / (double)NS_PER_SEC);
  *frame_count = 0;
  *fps_accum = 0;
  return fps;
}

/* pace_frame — sleep off the rest of this frame's time budget so the draw
 * loop holds RENDER_FPS_CAP no matter how quick the work was. */
static void pace_frame(int64_t frame_time, int64_t dt) {
  int64_t target_ns = NS_PER_SEC / RENDER_FPS_CAP;
  int64_t elapsed = clock_ns() - frame_time + dt;
  clock_sleep_ns(target_ns - elapsed);
}

int main(void) {
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

    if (app->need_resize) { /* 1. EVENTS — apply a pending resize */
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* 2. PERFORMANCE — measure frame dt, clamp against a stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > MAX_FRAME_NS)
      dt = MAX_FRAME_NS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    /* 3. SIMULATION — advance whole fixed-timestep ticks */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* 4. PERFORMANCE — refresh the fps readout, then hold the frame cap */
    fps_display = update_fps(&fps_accum, &frame_count, dt, fps_display);
    pace_frame(frame_time, dt);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps); /* 5. RENDER */
    screen_present();

    int ch = getch(); /* 6. EVENTS — drain one key */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
