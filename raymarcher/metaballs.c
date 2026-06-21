/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * metaballs.c — six spheres that melt into one blob, drawn in the terminal.
 * Each sphere knows how far any point is from its surface; we blend those
 * distances with a "soft minimum" so the spheres merge with smooth necks
 * instead of meeting at a hard edge — the classic metaball look.  The blob
 * is drawn by ray marching: for each cell we shoot a ray and creep along it
 * until it touches the surface, then shade it (soft shadows, curvature
 * colour, optional anti-aliasing).  Keys are on the bottom HUD line.
 *
 * Sister file raymarcher/raymarcher.c renders a single sphere with no
 * blending — read it first for the bare ray-march loop.
 *
 * Where the ideas come from:
 *   - blobby surfaces: Blinn (1982)
 *   - the soft-minimum blend + soft shadow: Iñigo Quílez,
 *     https://iquilezles.org/articles/distfunctions/ and .../smin/
 *   - creep-along-the-ray rendering: Hart, "Sphere Tracing" (1996)
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * How the file is laid out — each job in its own area so it's easy to
 * follow and hard to break:
 *   §4–§14  the math: pure functions that take inputs and return answers —
 *           the sphere distance + soft-min blend + whole-blob distance, the
 *           surface normal, curvature, the ray march, soft shadow, and
 *           shading.  None of it touches shared state or the screen.
 *   §15     the state, plus the one thing that moves on its own: scene_tick
 *           advances the clock and re-places the balls each frame.
 *   §16-§20 drawing: fill the Canvas (a per-frame scratch buffer) by ray
 *           marching, then paint it as terminal cells.  Reads state only.
 *   §3,§20  colour setup and the rest of the ncurses I/O.
 *   §21     input (keys, resize, quit) — changes state, but between frames.
 *   §2,§21  timekeeping and pacing.
 * (Nothing stores glows/trails and there are no scripted pauses, so two of
 *  the usual layers simply don't exist here.)
 *
 * One frame, in order: apply a pending resize, measure elapsed time, read
 * keys, advance the sim, refresh fps, draw, sleep to pace.  Scene (§15) is
 * the single bundle of state; only scene_tick and the key/resize handlers
 * change it.
 */

/* §1  config — all the tunable numbers live here */

/* §1.1 frame rate, screen layout, scene size. */
enum {
  TARGET_FPS = 24,
  FPS_UPDATE_MS = 500, /* how often to refresh the fps readout (ms) */
  HUD_ROWS = 2,        /* top + bottom rows are reserved for the HUD */
  DT_CAP_MS = 200,     /* clamp a slow frame so the balls don't teleport */

  N_BALLS = 6,
  N_THEMES = 4,
  N_CURV_BANDS = 8, /* curvature is bucketed into this many colours */
};

/* §1.2 ncurses colour-pair slots.  Each (theme, curvature-band) pair gets
 * its own slot; PAIR_FOR maps the two indices to it. */
#define PAIR_HUD 1
#define PAIR_HINT 2
#define PAIR_THEME_BASE 3
#define PAIR_FOR(theme, band)                                                  \
  (PAIR_THEME_BASE + (theme) * N_CURV_BANDS + (band))

/* §1.3 time. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* §1.4 half-resolution rendering: each canvas pixel is drawn as a
 * CELL_W × CELL_H block of terminal cells, which quarters the rays per frame. */
#define CELL_W 2
#define CELL_H 2
#define CELL_ASPECT 2.0f /* terminal cells are ~2× taller than wide */

/* §1.5 ray-march tuning.
 *   RM_MAX_STEPS  give up after this many steps along one ray
 *   RM_HIT_EPS    how close counts as "touched the surface"
 *   RM_MAX_DIST   if a ray gets this far out, it hit nothing */
#define RM_MAX_STEPS 64
#define RM_HIT_EPS 0.005f
#define RM_MAX_DIST 12.0f

/* §1.6 camera (zoom). */
#define CAM_Z_DEFAULT 5.0f
#define CAM_Z_MIN 2.5f /* nearest zoom — keeps the eye outside the ball orbits */
#define CAM_Z_MAX 14.0f
#define CAM_ZOOM_STEP 0.40f
#define FOV_HALF_TAN 0.55f /* tan(half the field of view) */

/* §1.6.1 anti-aliasing: shoot 4 rays per pixel on a 2×2 sub-grid and average,
 * for smoother edges.  Costs 4× the rays, so it's toggleable (a / A). */
#define AA_SAMPLES 4

static const float AA_OFFSETS[AA_SAMPLES][2] = {
    {0.25f, 0.25f},
    {0.75f, 0.25f},
    {0.25f, 0.75f},
    {0.75f, 0.75f},
};

/* §1.7 how strongly the balls melt together (the j/k knob).
 *   K_MIN  → barely melt (near-sharp joins); must stay > 0 (the blend
 *            formula divides by it)
 *   K_MAX  → balls bulge toward each other even when apart */
#define K_DEFAULT 0.8f
#define K_MIN 0.05f
#define K_MAX 4.0f
#define K_STEP 1.35f

/* §1.8 animation speed (scales how fast the orbits advance). */
#define SPD_DEFAULT 0.35f
#define SPD_MIN 0.02f
#define SPD_MAX 3.0f
#define SPD_STEP 1.35f

/* §1.9 shading + bold threshold.  Tuned for a low-res terminal: a tight
 * specular sparkles across the coarse half-res canvas, so SHIN/KS keep the
 * highlight broad and gentle.  Diffuse is half-Lambert (wrapped) — see §12. */
#define KA 0.08f
#define KD 0.75f
#define KS 0.35f
#define SHIN 16.0f
#define BOLD_SHADE_THRESHOLD 0.72f

/* §1.10 soft shadows.
 *   SHADOW_K     edge hardness — bigger = sharper-edged shadow
 *   SHADOW_BIAS  start the shadow ray a touch off the surface so a spot
 *                doesn't shadow itself
 *   SHADOW_NEAR  how far along to start the shadow march */
#define SHADOW_STEPS 16
#define SHADOW_K 8.0f
#define SHADOW_BIAS 0.01f
#define SHADOW_NEAR 0.02f

/* §1.11 how far to nudge when measuring surface facing / bendiness. */
#define NORMAL_EPS 0.004f
#define CURV_EPS 0.06f
#define CURV_SCALE 0.25f /* rescales raw curvature into roughly [0, 1] */

/* §1.12 light orbit. */
#define LIGHT_RATE 0.6f
#define LIGHT_RATE_Y (0.6f * 0.45f)
#define LIGHT_RADIUS_X 4.0f
#define LIGHT_RADIUS_Y 2.0f
#define LIGHT_BIAS_Y 2.5f
#define LIGHT_HEIGHT_Z 3.5f

/* §1.13 each ball drifts along its own looping path — independent sine waves
 * on x, y, and z, so the balls weave past each other without ever repeating
 * in lockstep.  BallOrbit is one ball's wave settings; ORBITS is the table
 * of six.  (x = ORBIT_RX·sin(freq_x·t + phase_x), and likewise for y, z.) */
typedef struct {
  float freq_x, freq_y, freq_z; /* wave speed on each axis     */
  float phase_x, phase_y;       /* starting offset (so balls differ) */
  float radius;                 /* this ball's size            */
} BallOrbit;

#define ORBIT_RX 1.35f
#define ORBIT_RY 0.75f
#define ORBIT_RZ 0.40f

static const BallOrbit ORBITS[N_BALLS] = {
    /*  fx     fy     fz     phx     phy     r     */
    {1.0f, 2.0f, 1.5f, 0.000f, 0.785f, 0.60f},
    {2.0f, 1.0f, 3.0f, 1.047f, 0.000f, 0.55f},
    {1.5f, 3.0f, 1.0f, 2.094f, 1.571f, 0.50f},
    {3.0f, 1.0f, 2.0f, 3.141f, 0.524f, 0.58f},
    {2.5f, 1.5f, 1.0f, 4.189f, 3.927f, 0.45f},
    {1.0f, 2.5f, 2.0f, 5.236f, 0.262f, 0.52f},
};

/* §1.14 colour themes: a name plus one colour per curvature band (flat necks
 * → bulgy caps).  Cycled with c. */
typedef struct {
  const char *name;
  short bands[N_CURV_BANDS]; /* xterm-256 codes, low curvature → high */
} Theme;

static const Theme THEMES[N_THEMES] = {
    {"CLASSIC ", {27, 33, 38, 44, 130, 166, 202, 214}},
    {"OCEAN   ", {25, 26, 27, 31, 38, 45, 51, 159}},
    {"EMBER   ", {88, 124, 160, 196, 202, 208, 214, 228}},
    {"NEON    ", {201, 165, 129, 93, 57, 82, 118, 155}},
};

/* §1.15 the shading characters, dark (space) to bright (@). */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define LUMA_RAMP_LEN ((int)(sizeof LUMA_RAMP - 1))

/* §1.16 alternate views, cycled with d / D — handy for seeing what the
 * renderer is actually computing. */
typedef enum {
  DEBUG_NORMAL = 0,    /* the normal, fully-lit blob               */
  DEBUG_NORMALS = 1,   /* colour by which way each surface faces    */
  DEBUG_CURVATURE = 2, /* colour by surface bendiness, no lighting  */
  DEBUG_SHADOW = 3,    /* brightness = how lit each spot is         */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "CURV   ",
    "SHADOW ",
};

/* §2  clock — read the time, and sleep.  Pure timekeeping; the frame
 * pacing that uses it lives in §21. */

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

/* §3  colour setup (part of the RENDER layer; the rest is §17–§20).  Loads
 * every (theme, curvature-band) colour plus the HUD colours into ncurses,
 * with a coarse fallback when the terminal has fewer than 256 colours. */

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
    for (int t = 0; t < N_THEMES; t++)
      for (int b = 0; b < N_CURV_BANDS; b++)
        init_pair((short)PAIR_FOR(t, b), THEMES[t].bands[b], -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    for (int t = 0; t < N_THEMES; t++)
      for (int b = 0; b < N_CURV_BANDS; b++)
        init_pair((short)PAIR_FOR(t, b),
                  (b < 3)   ? COLOR_BLUE
                  : (b < 5) ? COLOR_CYAN
                  : (b < 7) ? COLOR_YELLOW
                            : COLOR_WHITE,
                  -1);
  }
}

/* The math layer (§4-§14, plus a few pure helpers parked in §15/§16/§18):
 * every function here just takes its inputs and returns an answer — no
 * shared state, no screen — so the drawing code can't change its results. */

/* §4  vec3 — a 3-D point or direction (x, y, z) and the usual math on it. */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3norm(Vec3 a) {
  float L = v3len(a);
  return (L > 1e-12f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/* §5  distance from point p to a sphere's surface (negative inside).  Every
 * shape in the file is built by blending these with smin (§6). */
static inline float sphere_sdf(Vec3 p, Vec3 c, float r) {
  return v3len(v3sub(p, c)) - r;
}

/* §6  smooth minimum: like min(a, b), but when the two distances are within
 * k of each other it eases between them and bulges outward — that bulge is
 * what fuses two balls into a smooth neck.  Bigger k = more melting.
 * (k must stay > 0; we clamp it at K_MIN.  Quílez's polynomial smin.) */
static float smin(float a, float b, float k) {
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

/* §7  the whole blob's distance function.
 *
 * Ball is one metaball: a sphere (centre + radius).  SceneSDF is a read-only
 * view of the blob — the array of balls plus the melt strength — handed to
 * every SDF helper by const-pointer so §8..§13 take one pointer, not several.
 */
typedef struct {
  Vec3 centre;
  float radius;
} Ball;

typedef struct {
  const Ball *balls;
  int n;
  float k_blend; /* melt strength (see smin, §6) */
} SceneSDF;

/* distance to the blob = the balls' distances melted together with smin.
 * (Fold order barely matters for our blend values and spacing.) */
static float scene_sdf(Vec3 p, const SceneSDF *s) {
  float d = sphere_sdf(p, s->balls[0].centre, s->balls[0].radius);
  for (int i = 1; i < s->n; i++)
    d = smin(d, sphere_sdf(p, s->balls[i].centre, s->balls[i].radius),
             s->k_blend);
  return d;
}

/* §8  which way the surface faces at p (its "normal"), for lighting.  We
 * can't read it directly, so we sample the distance at four points around p
 * (the corners of a tetrahedron) and combine them — four taps placed off-axis
 * give a cleaner result at sharp spots than the usual six. */
static Vec3 estimate_normal(Vec3 p, const SceneSDF *s) {
  float e = NORMAL_EPS;
  float f0 = scene_sdf(v3(p.x + e, p.y - e, p.z - e), s);
  float f1 = scene_sdf(v3(p.x - e, p.y + e, p.z - e), s);
  float f2 = scene_sdf(v3(p.x - e, p.y - e, p.z + e), s);
  float f3 = scene_sdf(v3(p.x + e, p.y + e, p.z + e), s);
  return v3norm(v3(f0 - f1 - f2 + f3, -f0 + f1 - f2 + f3, -f0 - f1 + f2 + f3));
}

/* §9  how bendy the surface is at p — convex caps read one way, the dimpled
 * necks between balls another.  We get it by comparing the distance at p to
 * the average of its six neighbours; the caller rescales it into [0,1] to
 * choose a colour band.  (Discrete Laplacian of the distance field.) */
static float estimate_curvature(Vec3 p, const SceneSDF *s) {
  float e = CURV_EPS;
  float c0 = scene_sdf(p, s);
  float lap = scene_sdf(v3(p.x + e, p.y, p.z), s) +
              scene_sdf(v3(p.x - e, p.y, p.z), s) +
              scene_sdf(v3(p.x, p.y + e, p.z), s) +
              scene_sdf(v3(p.x, p.y - e, p.z), s) +
              scene_sdf(v3(p.x, p.y, p.z + e), s) +
              scene_sdf(v3(p.x, p.y, p.z - e), s) - 6.0f * c0;
  return lap / (e * e);
}

/* §10  ray marching: step along the ray, each time jumping forward by the
 * distance to the nearest surface (safe — nothing is closer).  When that
 * shrinks to ~zero we've hit the blob; if the ray runs too far, it missed.
 * Returns how far along the ray the hit is, or -1 for a miss. */
static float sphere_trace(Vec3 origin, Vec3 dir, const SceneSDF *s) {
  float t = 0.0f;
  for (int i = 0; i < RM_MAX_STEPS; i++) {
    Vec3 p = v3add(origin, v3mul(dir, t));
    float d = scene_sdf(p, s);
    if (d < RM_HIT_EPS)
      return t;
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  return -1.0f;
}

/* §11  how much light reaches this spot: 1 if the path to the light is clear,
 * 0 if blocked, and in between if the ray skims close to the blob on its way
 * — that near-miss is what gives soft shadow edges for free (Quílez). */
static float soft_shadow(Vec3 origin, Vec3 to_light_dir, float t_min,
                         float t_max, const SceneSDF *s) {
  float res = 1.0f;
  float t = t_min;
  for (int i = 0; i < SHADOW_STEPS && t < t_max; i++) {
    float h = scene_sdf(v3add(origin, v3mul(to_light_dir, t)), s);
    if (h < RM_HIT_EPS * 0.5f)
      return 0.0f;
    res = fminf(res, SHADOW_K * h / t);
    t += fmaxf(h, RM_HIT_EPS);
  }
  return clampf(res, 0.0f, 1.0f);
}

/* §12  final brightness of a hit spot (0..1): a base glow, plus how squarely
 * it faces the light and a shiny highlight, all dimmed by shadow (the base
 * glow never goes fully black, so shape survives in shadow).
 *
 * The light is "wrapped" around the surface (half-Lambert, Valve/Mitchell
 * 2006) rather than cut off where it grazes: normally the side facing away
 * goes flat and loses its shape — bad here, since the character carries the
 * shape — so even the far side keeps a gradient.  The highlight only appears
 * on the genuinely lit side. */
static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light, float shadow) {
  Vec3 L_dir = v3norm(v3sub(light, hit));
  Vec3 V_dir = v3norm(v3sub(cam, hit));

  /* how much the surface faces the light, wrapped from [-1,1] to [0,1] */
  float ndl = v3dot(N, L_dir);
  float wrap = ndl * 0.5f + 0.5f;
  float diffuse = wrap * wrap;

  float spec = 0.0f;
  if (ndl > 0.0f) {
    Vec3 R_dir = v3sub(v3mul(N, 2.0f * ndl), L_dir);
    spec = powf(fmaxf(0.0f, v3dot(R_dir, V_dir)), SHIN);
  }
  return clampf(KA + shadow * (KD * diffuse + KS * spec), 0.0f, 1.0f);
}

/* §13  trace one ray and report everything about what it found.  Each AA
 * sample produces one Hit; supersample_pixel (§17) averages them. */

/* What one ray found.  The separate fields let each debug view read just the
 * one it needs. */
typedef struct {
  bool hit;
  Vec3 p;
  Vec3 normal;     /* which way the surface faces */
  float shade;     /* final brightness, 0..1 */
  float shadow;    /* how lit the spot is, 0..1 */
  float curvature; /* surface bendiness, 0..1 */
} Hit;

static Hit cast_ray(Vec3 origin, Vec3 dir, const SceneSDF *s, Vec3 light,
                    bool soft_shadows) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0.0f};

  float t = sphere_trace(origin, dir, s);
  if (t < 0.0f)
    return h;

  h.hit = true;
  h.p = v3add(origin, v3mul(dir, t));
  h.normal = estimate_normal(h.p, s);

  /* how lit this spot is — trace a soft shadow ray, or 1.0 if shadows are off */
  h.shadow = 1.0f;
  if (soft_shadows) {
    Vec3 L_dir = v3norm(v3sub(light, h.p));
    Vec3 shad_o = v3add(h.p, v3mul(h.normal, SHADOW_BIAS));
    float to_lite = v3len(v3sub(light, h.p));
    h.shadow = soft_shadow(shad_o, L_dir, SHADOW_NEAR, to_lite, s);
  }

  h.shade = phong(h.p, h.normal, origin, light, h.shadow);
  float raw_c = estimate_curvature(h.p, s);
  h.curvature = clampf(raw_c * CURV_SCALE, 0.0f, 1.0f);
  return h;
}

/* §14  where each ball and the light sit at time t (their orbit paths). */

static Vec3 ball_position_at(int i, float t) {
  const BallOrbit *o = &ORBITS[i];
  return v3(ORBIT_RX * sinf(o->freq_x * t + o->phase_x),
            ORBIT_RY * sinf(o->freq_y * t + o->phase_y),
            ORBIT_RZ * cosf(o->freq_z * t));
}

static Vec3 light_position_at(float t) {
  return v3(cosf(t * LIGHT_RATE) * LIGHT_RADIUS_X,
            sinf(t * LIGHT_RATE_Y) * LIGHT_RADIUS_Y + LIGHT_BIAS_Y,
            LIGHT_HEIGHT_Z);
}

/* §15  the state, and the one thing that moves on its own.  Scene holds
 * everything; only scene_tick (and the key/resize handlers) change it.
 * scene_tick advances the clock and re-places the balls; scene_sdf_view is a
 * pure helper that hands the SDF code a read-only view of the balls. */

/* Scene — all simulation state, as a table of contents:
 *   WHAT is simulated  balls         the metaballs, recomputed each tick
 *   HOW it's driven     k_blend       smooth-min strength (j/k)
 *                       speed         animation rate (+/-)
 *                       cam_z         zoom (z/Z)
 *                       theme         colour palette (c)
 *                       debug_mode    which view (d/D)
 *                       soft_shadows  shadows on/off (s)
 *                       aa_enabled    anti-aliasing on/off (a)
 *                       paused        freezes the orbits (space)
 *   WHERE/when          time          scene clock (seconds × speed) */
typedef struct {
  Ball balls[N_BALLS];

  float k_blend;
  float speed;
  float cam_z;
  int theme;
  DebugMode debug_mode;
  bool soft_shadows;
  bool aa_enabled;
  bool paused;

  float time;
} Scene;

static void scene_update_balls(Scene *s) {
  for (int i = 0; i < N_BALLS; i++) {
    s->balls[i].centre = ball_position_at(i, s->time);
    s->balls[i].radius = ORBITS[i].radius;
  }
}

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->k_blend = K_DEFAULT;
  s->speed = SPD_DEFAULT;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
  s->soft_shadows = true;
  s->aa_enabled = true;
  scene_update_balls(s);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt * s->speed;
  scene_update_balls(s);
}

static SceneSDF scene_sdf_view(const Scene *s) {
  return (SceneSDF){
      .balls = s->balls,
      .n = N_BALLS,
      .k_blend = s->k_blend,
  };
}

/* No EFFECTS layer: the Canvas (§17) is a render scratch buffer, refilled
 * from scratch every frame — not stored cosmetic state.  Curvature, shadow,
 * and shade are recomputed each frame, never carried between frames. */

/* No DELAYS layer: the only pause is the user `paused` flag, handled inside
 * §15 scene_tick.  There are no scripted holds or timers. */

/* §16  the camera (fixed, looking down the −z axis at the blob) and the ray
 * through each pixel.  Both are pure — they read the canvas size + zoom and
 * return a value. */

/* Camera — the eye point, the lens width, and the squash that keeps tall
 * cells from stretching the picture. */
typedef struct {
  Vec3 origin;
  float fov_t;       /* tan(half the field of view) */
  float phys_aspect; /* corrects for cell + block shape so the blob is round */
} Camera;

static Camera camera_for_canvas(int cw, int ch, float cam_z) {
  return (Camera){
      .origin = v3(0.0f, 0.0f, cam_z),
      .fov_t = FOV_HALF_TAN,
      .phys_aspect = ((float)ch * (float)CELL_H * CELL_ASPECT) /
                     ((float)cw * (float)CELL_W),
  };
}

/* Direction of the ray through canvas pixel (px, py) at sub-pixel offset
 * (ox, oy).  AA shoots the 2×2 grid; the plain pass uses the centre (0.5,
 * 0.5).  py = 0 is the top of the screen. */
static Vec3 pixel_ray(int px, int py, float ox, float oy, int cw, int ch,
                      const Camera *c) {
  float u = ((float)px + ox) / (float)cw * 2.0f - 1.0f;
  float v = -((float)py + oy) / (float)ch * 2.0f + 1.0f;
  return v3norm(v3(u * c->fov_t, v * c->fov_t * c->phys_aspect, -1.0f));
}

/* §17–§20 are RENDER: read the Scene, build the picture.  canvas_render fills
 * the Canvas (a scratch buffer); §18–§20 turn it into terminal cells.  These
 * write only the Canvas, the ncurses colour table, and the screen. */

/* §17  the render buffer.  Canvas is one frame's results, kept as four
 * parallel arrays (w×h) — one value per pixel — so each debug view can read
 * just the field it needs.  shades < 0 marks a pixel the rays missed.
 * canvas_render writes it each frame; canvas_alloc/free own the memory
 * (allocated at startup, freed + re-made on resize). */
typedef struct {
  int w, h;
  float *shades;
  float *curvs;
  float *shadows;
  Vec3 *normals;
} Canvas;

static void canvas_alloc(Canvas *c, int term_cols, int term_rows) {
  int draw_rows = term_rows - HUD_ROWS;
  if (draw_rows < 1)
    draw_rows = 1;

  c->w = term_cols / CELL_W;
  c->h = draw_rows / CELL_H;
  if (c->w < 1)
    c->w = 1;
  if (c->h < 1)
    c->h = 1;

  size_t n = (size_t)c->w * (size_t)c->h;
  c->shades = malloc(n * sizeof(float));
  c->curvs = malloc(n * sizeof(float));
  c->shadows = malloc(n * sizeof(float));
  c->normals = malloc(n * sizeof(Vec3));
}

static void canvas_free(Canvas *c) {
  free(c->shades);
  c->shades = NULL;
  free(c->curvs);
  c->curvs = NULL;
  free(c->shadows);
  c->shadows = NULL;
  free(c->normals);
  c->normals = NULL;
  c->w = c->h = 0;
}

/* Shoot the anti-aliasing samples for one pixel, average the hits, and store
 * the result.  Coverage (hits / samples) dims edge pixels where only some
 * samples landed; a pixel with no hits gets the miss marker (shade < 0). */
static void supersample_pixel(Canvas *c, int px, int py, const Camera *cam,
                              const SceneSDF *view, Vec3 light,
                              bool soft_shadows, int n_samples) {
  float sum_shade = 0.0f;
  float sum_curv = 0.0f;
  float sum_shadow = 0.0f;
  Vec3 sum_normal = v3(0, 0, 0);
  int hit_count = 0;

  for (int sample = 0; sample < n_samples; sample++) {
    float ox, oy; /* sub-pixel offset: centre for 1 sample, else the 2×2 grid */
    if (n_samples == 1) {
      ox = 0.5f;
      oy = 0.5f;
    } else {
      ox = AA_OFFSETS[sample][0];
      oy = AA_OFFSETS[sample][1];
    }

    Vec3 ray = pixel_ray(px, py, ox, oy, c->w, c->h, cam);
    Hit h = cast_ray(cam->origin, ray, view, light, soft_shadows);
    if (h.hit) {
      sum_shade += h.shade;
      sum_curv += h.curvature;
      sum_shadow += h.shadow;
      sum_normal = v3add(sum_normal, h.normal);
      hit_count++;
    }
  }

  int idx = py * c->w + px;
  if (hit_count == 0) {
    c->shades[idx] = -1.0f;
    c->curvs[idx] = 0.0f;
    c->shadows[idx] = 0.0f;
    c->normals[idx] = v3(0, 0, 0);
  } else {
    float coverage = (float)hit_count / (float)n_samples;
    c->shades[idx] = (sum_shade / (float)hit_count) * coverage;
    c->curvs[idx] = sum_curv / (float)hit_count;
    c->shadows[idx] = sum_shadow / (float)hit_count;
    c->normals[idx] = v3norm(sum_normal);
  }
}

/* Build this frame's camera, light, and scene-view, then render every pixel. */
static void canvas_render(Canvas *c, const Scene *s) {
  Camera cam = camera_for_canvas(c->w, c->h, s->cam_z);
  Vec3 light = light_position_at(s->time);
  SceneSDF view = scene_sdf_view(s);
  int n_samples = s->aa_enabled ? AA_SAMPLES : 1;

  for (int py = 0; py < c->h; py++)
    for (int px = 0; px < c->w; px++)
      supersample_pixel(c, px, py, &cam, &view, light, s->soft_shadows,
                        n_samples);
}

/* §18  turn the Canvas buffer into terminal cells.  decorate / shade_to_glyph
 * / curvature_to_band just pick a character + colour (pure); emit_block,
 * canvas_offsets, and canvas_draw do the actual drawing. */

/* One terminal cell, ready to print: a character, its colour, and a
 * bold/normal attribute.  pair < 0 means "miss — don't paint". */
typedef struct {
  char glyph;
  int pair;
  attr_t attr;
} Cell;

static char shade_to_glyph(float shade) {
  int idx = (int)(shade * (float)(LUMA_RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= LUMA_RAMP_LEN)
    idx = LUMA_RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static int curvature_to_band(float curv) {
  int b = (int)(curv * (float)(N_CURV_BANDS - 1) + 0.5f);
  if (b < 0)
    b = 0;
  if (b >= N_CURV_BANDS)
    b = N_CURV_BANDS - 1;
  return b;
}

static Cell decorate(float shade, float curv, int theme) {
  if (shade < 0.0f)
    return (Cell){' ', -1, 0};
  int band = curvature_to_band(curv);
  return (Cell){
      .glyph = shade_to_glyph(shade),
      .pair = PAIR_FOR(theme, band),
      .attr = (shade > BOLD_SHADE_THRESHOLD) ? A_BOLD : 0,
  };
}

/* Paint one canvas pixel as its CELL_W × CELL_H block of terminal cells
 * (clipped to the screen).  A miss cell paints nothing. */
static void emit_block(int tx0, int ty0, Cell c, int term_cols, int term_rows) {
  if (c.pair < 0)
    return;

  attr_t a = COLOR_PAIR(c.pair) | c.attr;
  attron(a);
  for (int by = 0; by < CELL_H; by++) {
    int ty = ty0 + by;
    if (ty < 0 || ty >= term_rows)
      continue;
    for (int bx = 0; bx < CELL_W; bx++) {
      int tx = tx0 + bx;
      if (tx < 0 || tx >= term_cols)
        continue;
      mvaddch(ty, tx, (chtype)(unsigned char)c.glyph);
    }
  }
  attroff(a);
}

/* Where to put the canvas's top-left so it sits centred below the HUD. */
static void canvas_offsets(const Canvas *c, int term_cols, int term_rows,
                           int *out_off_x, int *out_off_y) {
  int off_x = (term_cols - c->w * CELL_W) / 2;
  int off_y = 1 + (term_rows - HUD_ROWS - c->h * CELL_H) / 2;
  if (off_x < 0)
    off_x = 0;
  if (off_y < 1)
    off_y = 1;
  *out_off_x = off_x;
  *out_off_y = off_y;
}

/* The normal view: brightness picks the character, curvature picks the colour. */
static void canvas_draw(const Canvas *c, int term_cols, int term_rows,
                        int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      Cell cell = decorate(c->shades[idx], c->curvs[idx], theme);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, cell, term_cols, term_rows);
    }
  }
}

/* §19  the alternate views (d/D).  Each paints one of the Canvas arrays
 * directly so you can see what the renderer computed. */

/* colour by which compass direction the surface faces; character by how
 * much it tilts up */
static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows,
                                int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      Vec3 N = c->normals[idx];
      float azimuth = atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
      float y_lit = clampf(N.y * 0.5f + 0.5f, 0.0f, 1.0f);

      int band = curvature_to_band(azimuth);
      char glyph = shade_to_glyph(y_lit);
      Cell cell = (Cell){glyph, PAIR_FOR(theme, band), 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

/* character + colour both from surface bendiness */
static void canvas_draw_curvature(const Canvas *c, int term_cols, int term_rows,
                                  int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      float curv = c->curvs[idx];
      int band = curvature_to_band(curv);
      char glyph = shade_to_glyph(curv);
      Cell cell =
          (Cell){glyph, PAIR_FOR(theme, band), (curv > 0.7f) ? A_BOLD : 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

/* character + colour both from how lit each spot is (1 = lit, 0 = shadowed) */
static void canvas_draw_shadow(const Canvas *c, int term_cols, int term_rows,
                               int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      float shadow = c->shadows[idx];
      int band = curvature_to_band(shadow);
      char glyph = shade_to_glyph(shadow);
      Cell cell =
          (Cell){glyph, PAIR_FOR(theme, band), (shadow > 0.7f) ? A_BOLD : 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

static void canvas_draw_active(const Canvas *c, int term_cols, int term_rows,
                               const Scene *s) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_CURVATURE:
    canvas_draw_curvature(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_SHADOW:
    canvas_draw_shadow(c, term_cols, term_rows, s->theme);
    break;
  default:
    canvas_draw(c, term_cols, term_rows, s->theme);
    break;
  }
}

/* §20  screen — start/stop ncurses, draw the HUD, and own the Canvas's size
 * (it's reallocated to match the terminal on resize). */

/* The terminal size plus the render buffer sized to match it. */
typedef struct {
  int cols, rows;
  Canvas canvas;
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
  canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

static void screen_free(Screen *sc) {
  canvas_free(&sc->canvas);
  endwin();
}

/* On resize, endwin + refresh makes ncurses pick up the new size; then we
 * rebuild the Canvas to match. */
static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
  canvas_free(&sc->canvas);
  canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

/* Draw the two HUD rows: title + status on top, key reminders on the bottom. */
static void hud_draw(const Screen *sc, const Scene *s, double fps) {
  char status[200];
  snprintf(status, sizeof status,
           " %5.1f fps  k=%4.2f  spd=%4.2f  zoom=%4.2f  aa=%-3s  "
           "shadow=%-3s  theme=%s  debug=%s  %s ",
           fps, (double)s->k_blend, (double)s->speed, (double)s->cam_z,
           s->aa_enabled ? "on" : "off", s->soft_shadows ? "on" : "off",
           THEMES[s->theme].name, DEBUG_MODE_NAMES[s->debug_mode],
           s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > sc->cols)
    slen = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - slen, "%s", status);
  mvprintw(0, 0, " METABALLS ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  j/k:blend  z/Z:zoom  a:aa  "
           "s:shadow  c:theme  d/D:debug  +/-:speed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps) {
  erase();
  canvas_render(&sc->canvas, s);
  canvas_draw_active(&sc->canvas, sc->cols, sc->rows, s);
  hud_draw(sc, s, fps);
  wnoutrefresh(stdscr);
  doupdate();
}

/* §21  input + the main loop.  Keys and resize change the state between
 * frames; main() runs everything each frame, in order (read input → step the
 * sim → draw → pace), and holds the frame rate. */

/* Everything the running program owns: the scene, the screen + its buffer,
 * and two flags the signal handlers flip. */
typedef struct {
  Scene scene;
  Screen screen;
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

  case 'k':
    if (s->k_blend < K_MAX)
      s->k_blend *= K_STEP;
    if (s->k_blend > K_MAX)
      s->k_blend = K_MAX;
    break;
  case 'j':
    if (s->k_blend > K_MIN)
      s->k_blend /= K_STEP;
    if (s->k_blend < K_MIN)
      s->k_blend = K_MIN;
    break;

  case 's':
  case 'S':
    s->soft_shadows = !s->soft_shadows;
    break;

  case 'a':
  case 'A':
    s->aa_enabled = !s->aa_enabled;
    break;

  case 'z':
    s->cam_z -= CAM_ZOOM_STEP;
    if (s->cam_z < CAM_Z_MIN)
      s->cam_z = CAM_Z_MIN;
    break;
  case 'Z':
    s->cam_z += CAM_ZOOM_STEP;
    if (s->cam_z > CAM_Z_MAX)
      s->cam_z = CAM_Z_MAX;
    break;

  case 'c':
  case 'C':
    s->theme = (s->theme + 1) % N_THEMES;
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  case '+':
  case '=':
    if (s->speed < SPD_MAX)
      s->speed *= SPD_STEP;
    if (s->speed > SPD_MAX)
      s->speed = SPD_MAX;
    break;
  case '-':
  case '_':
    if (s->speed > SPD_MIN)
      s->speed /= SPD_STEP;
    if (s->speed < SPD_MIN)
      s->speed = SPD_MIN;
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

/* pace_frame — sleep off the rest of this frame's time budget so the loop
 * holds TARGET_FPS no matter how quick the work was. */
static void pace_frame(int64_t frame_time, int64_t dt) {
  int64_t elapsed = clock_ns() - frame_time + dt;
  clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) { /* 1. EVENTS — apply a pending resize */
      app->need_resize = 0;
      screen_resize(&app->screen);
      frame_time = clock_ns();
    }

    /* 2. PERFORMANCE — measure frame dt, clamp against a stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_MS * NS_PER_MS)
      dt = DT_CAP_MS * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    int ch; /* 3. EVENTS — drain all pending keys */
    while ((ch = getch()) != ERR)
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }

    scene_tick(&app->scene, dt_sec); /* 4. SIMULATION — advance */

    /* 5. PERFORMANCE — refresh the fps readout */
    fps_display = update_fps(&fps_accum, &frame_count, dt, fps_display);

    screen_draw(&app->screen, &app->scene, fps_display); /* 6. RENDER */

    pace_frame(frame_time, dt); /* 7. PERFORMANCE — hold the frame cap */
  }

  screen_free(&app->screen);
  return 0;
}
