/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reflect_spheres.c — the classic ray-tracer scene: shiny spheres on a
 * checkerboard floor under a gradient sky, with real mirror reflections.
 *
 * DEMO: A chrome sphere and two coloured ones sit on an endless checkerboard;
 * the camera slowly circles them. For every character cell we shoot one ray into
 * the scene; when it hits something shiny we shoot ANOTHER ray off the surface to
 * see what's mirrored there, and so on for a few bounces. So the floor's checker
 * warps across the chrome sphere, the spheres reflect each other, and the floor
 * faintly mirrors the sky. Reflections that bounce ray-off-ray are the one thing
 * ray tracing does that ordinary 3-D drawing can't — press +/- to add or remove
 * bounces and watch the mirrored world appear and vanish.
 *
 * Study alongside: raytracing/sphere_raytrace.c (one ray per cell, but a single
 *   sphere with no reflection) and raster/cube_raster.c (the same shapes drawn the
 *   OTHER way — projecting triangles instead of shooting rays).
 *
 * Section map (concern layers — see ARCHITECTURE below):
 *   §1 CONFIG       — constants, the sphere list, palettes (mutates nothing)
 *   §2 STATE        — Vec3, Sphere, Hit + the run harness (mutates nothing)
 *   §3 PERFORMANCE  — clock + smoothed fps; the frame cap lives in §8 main
 *   §4 LOGIC        — 3-D vector maths (pure: no mutation, no I/O)
 *   §5 LOGIC        — ray tracing: intersect, shade, trace (pure)
 *   §6 SIMULATION   — orbit the camera
 *   §7 RENDER       — shoot a ray per cell → screen; reads only
 *   §8 APP          — events (keys/resize) + the main loop
 *
 * Keys:  q/ESC quit  space/p pause  r reset  +/- bounces
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/reflect_spheres.c -o reflect_spheres -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Recursive (Whitted) ray tracing. From the eye, shoot one ray
 *                  per pixel and find the nearest surface it hits. Shade that point
 *                  with direct light plus a shadow check, then — if the surface is
 *                  mirror-like — shoot a reflected ray and blend in whatever IT
 *                  sees. Recursing that reflected ray a few times gives mirrors
 *                  reflecting mirrors, the signature ray-tracing look.
 *
 * Data-structure : Just a short list of spheres and one infinite floor plane —
 *                  each ray is tested against all of them and keeps the closest.
 *                  Nothing per-pixel is stored; the image is recomputed each frame.
 *
 * Rendering      : Each ray returns an RGB colour; we map it to the nearest of the
 *                  256-colour cube and pick a glyph by brightness. Pixel aspect is
 *                  corrected (cells are ~2× tall) so spheres render round.
 *
 * Performance    : cols×rows rays, each testing a few spheres + the plane, times a
 *                  few reflection bounces and a shadow ray. A few hundred thousand
 *                  intersections a frame — fine at terminal resolution and 60 fps.
 *                  No allocation.
 *
 * References     : T. Whitted, "An Improved Illumination Model for Shaded Display",
 *                    CACM 23(6) 1980 — the original recursive ray tracer.
 *                  P. Shirley, "Ray Tracing in One Weekend" — https://raytracing.github.io
 *                  Ray–sphere intersection — https://en.wikipedia.org/wiki/Line–sphere_intersection
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Your eye doesn't send rays out, but pretend it does: from the camera, fire a
 * straight line through each pixel into the world and see what it touches first.
 * Colour that pixel by the light landing there. The magic step: if the thing you
 * hit is a mirror, the pixel's colour is really "what you'd see looking off that
 * mirror" — so fire a NEW ray in the bounced direction and ask the same question
 * again. Do that a few times and mirrors show mirrors show mirrors.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each surface point answers "how lit am I?" in three parts: a little ambient fill
 * so nothing is pure black; direct light IF the light isn't blocked by another
 * sphere (that block is the shadow); and a bright pinpoint highlight where the
 * light glints off toward the eye. A shiny surface then mixes in a fraction of the
 * reflected ray's answer. The floor is a checkerboard just because that pattern
 * makes the reflections and the perspective easy to read.
 *
 * DRAWING METHOD / ALGORITHM IN STEPS   (per pixel)
 * ─────────────────────────────────────
 *   1. Build the ray from the eye through this cell (aspect-corrected).
 *   2. cast_ray(ray, bounces):
 *        a. Find the nearest sphere/floor hit. Miss → return the sky colour.
 *        b. Shade the hit: ambient + (light, if not shadowed) + a highlight.
 *        c. If it's shiny and we have bounces left, reflect the ray and blend in
 *           cast_ray(reflected ray, bounces−1).
 *   3. Map the returned colour to a glyph + a 256-colour, stamp the cell.
 *
 * KEY FORMULAS
 * ────────────
 *   ray hits sphere : nearest t where |eye + t·dir − centre| = radius
 *   ray hits floor  : t = −eye.y / dir.y   (the height drops to 0)
 *   reflect(d, n)   : d − 2·(d·n)·n        bounce a ray off a surface facing n
 *   shading         : colour·(ambient + lit) + white·highlight
 *   final           : local·(1−shininess) + reflected·shininess
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Reflected and shadow rays start a hair OFF the surface (along the normal) or
 *    they immediately "hit" the surface they left — the classic self-shadow acne.
 *  - Reflection recursion is capped (bounces) so two facing mirrors don't loop
 *    forever; +/- changes the cap live.
 *  - Pixel aspect MUST be corrected or the spheres come out as tall eggs.
 *  - Downward rays always meet the infinite floor; only upward rays reach the sky.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Set bounces to 0 ('-' a few times): the chrome sphere turns flat/matte, the
 *    floor stops mirroring — pure direct lighting. Raise it and reflections return.
 *  - The checker squares should curve smoothly across the chrome sphere.
 *  - Each sphere casts a dark shadow blob onto the floor toward the light.
 *  - Pause (space): a still frame you can inspect; the spheres stay round.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * The concerns this program actually has, each in its own labelled section.
 *
 *   LAYER        SECTION          MUTATES
 *   ───────────  ───────────────  ───────────────────────────────────────────
 *   (config)     §1 CONFIG        nothing — constants + the sphere/palette tables
 *   (state)      §2 STATE         nothing — just the struct definitions
 *   PERFORMANCE  §3 PERFORMANCE   the FpsCounter; the frame-cap sleep + dt clamp
 *                                 live in main(). Pure clock reads otherwise.
 *   LOGIC        §4–§5 LOGIC      nothing — pure vector maths + the whole ray
 *                                 tracer. It reads the fixed scene and returns a
 *                                 colour; no mutation and no I/O, so nothing in
 *                                 RENDER can change a LOGIC result. (nearest_hit
 *                                 fills a caller's Hit — an output, not state.)
 *   SIMULATION   §6 SIMULATION    the Scene's camera — scene_tick advances the
 *                                 orbit angle; scene_init sets the start.
 *   RENDER       §7 RENDER        the terminal only (ncurses). Reads the Scene;
 *                                 never writes simulation state. (rgb_to_cube and
 *                                 luminance are pure render helpers.)
 *   EVENTS       §8 APP           Scene + App, but OUTSIDE the tick — see below.
 *
 * No EFFECTS layer: there is no stored cosmetic state — the sky, reflections, and
 *   shading are all recomputed per ray every frame, never kept in a buffer.
 * No DELAYS layer: the only pause is the `paused` gate at the top of scene_tick
 *   (space/p toggles it); there are no staged holds or timers.
 *
 * PER-FRAME COMBINE — the ONE place simulation state advances is scene_tick()
 * (§6), called once per frame from main():
 *     1. if paused, do nothing
 *     2. advance the camera around its orbit by dt         (SIMULATION)
 * RENDER runs after the tick: erase → render_scene → HUD → present.
 *
 * EVENTS ARE NOT PART OF THE TICK. Quit, pause, reset (r), bounces (+/-), and
 * resize all mutate state from app_handle_key() / app_do_resize() (§8), between
 * frames.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 CONFIG — image/camera/scene constants, the sphere list, palettes ── */

enum {
  TARGET_FPS = 60,
};

/* Camera: it circles the scene at this radius and height, always looking at a
 * point just above the floor centre. */
#define ORBIT_RADIUS 9.5f
#define CAMERA_HEIGHT 3.2f
#define LOOK_AT_HEIGHT 0.9f
#define ORBIT_SPEED 0.35f /* radians per second */
#define FOV_DEGREES 55.0f /* vertical field of view */

/* A terminal cell is ~2× taller than wide; the camera's horizontal is divided by
 * this so the spheres render round, not as tall eggs. */
#define CELL_ASPECT 2.0f
#define DEG_TO_RAD ((float)M_PI / 180.0f)

/* How many times a reflected ray may bounce before we stop (0 = no mirrors).
 * Cheap knob with a big visual payoff, so it's adjustable with +/-. */
#define BOUNCES_MIN 0
#define BOUNCES_MAX 5
#define BOUNCES_DEFAULT 3

/* Lighting. The light is directional (like the sun): one direction, no distance.
 * Ambient keeps shadows from being pure black; the highlight is the glossy glint. */
#define AMBIENT 0.15f
#define SHININESS 32.0f     /* higher = tighter, sharper highlight */
#define HIGHLIGHT 0.6f      /* strength of the glossy glint */
#define FLOOR_REFLECT 0.25f /* the floor faintly mirrors the sky + spheres */

/* Shape of the sun in the sky: a broad soft halo plus a tight bright core. */
#define SUN_GLOW_FALLOFF 4.0f   /* lower = wider halo around the sun */
#define SUN_GLOW_STRENGTH 0.55f /* brightness of the halo */
#define SUN_DISC_FALLOFF 220.0f /* higher = tighter bright sun disc */

/* Start reflected/shadow rays this far off the surface so they don't re-hit it. */
#define SURFACE_EPS 1e-3f

/* "No hit yet" starting distance — farther than anything in the scene. */
#define RAY_FAR 1e30f

/* World units per checkerboard square. */
#define CHECK_SCALE 1.0f

/* Longest frame we'll trust. A hiccup (window drag, debugger) can make one frame
 * huge; clamping it stops the camera from jumping. */
#define DT_CAP_SEC 0.10f

#define NS_PER_SEC 1000000000LL
#define HUD_BUF_LEN 80

/* Glyphs by brightness, dark → bright. A space at the bottom leaves the rare
 * pure-black cell empty; everything else is filled. */
static const char k_ramp[] = " .:-=+*#%@";
#define RAMP_LEN ((int)sizeof k_ramp - 1)

/* ncurses colour-pair slots. On a 256-colour terminal we use one pair per colour
 * of the 6×6×6 cube (216 of them); the HUD pairs sit just past those. */
enum {
  CUBE_BASE = 1, /* pairs CUBE_BASE .. CUBE_BASE + 215 map to cube colours 0..215 */
  PAIR_HUD = CUBE_BASE + 216,
  PAIR_HINT,
};

/* True when the terminal has the 256-colour palette. Set once at startup; without
 * it we fall back to a plain brightness ramp. */
static bool g_has_256 = false;

/* ── §2 STATE — the domain types (Vec3, Sphere, Hit) + the run harness (Scene, App) ── */

/*
 * Vec3 — a point or direction in 3-D (also reused as an RGB colour, where x,y,z
 * are red,green,blue in 0..1). Three floats that always travel together.
 */
typedef struct {
  float x, y, z;
} Vec3;

/*
 * Sphere — one ball in the scene.
 *
 *   center   its centre, in world units.
 *   radius   its size, in world units.
 *   color    its base colour (RGB 0..1).
 *   reflect  how mirror-like it is, 0 (matte) .. 1 (perfect mirror): the fraction
 *            of its colour that comes from the reflected ray instead of its own.
 */
typedef struct {
  Vec3 center;
  float radius;
  Vec3 color;
  float reflect;
} Sphere;

/*
 * Hit — what a ray found where it landed: the surface point, which way the
 * surface faces (the normal), its colour and shininess. Filled by nearest_hit and
 * consumed by the shader, so the shading code never re-derives the geometry.
 */
typedef struct {
  float t;      /* distance along the ray to the hit */
  Vec3 point;   /* where it hit, in world units */
  Vec3 normal;  /* unit vector: which way the surface faces there */
  Vec3 color;   /* surface colour at the hit (checker colour for the floor) */
  float reflect;/* surface shininess 0..1 */
} Hit;

/* The scene's spheres — fixed geometry, so they live in config, not in Scene. */
static const Sphere k_spheres[] = {
    {{0.0f, 1.0f, 0.0f}, 1.0f, {0.90f, 0.90f, 0.95f}, 0.80f},  /* chrome, centre */
    {{-2.8f, 0.8f, -0.5f}, 0.8f, {0.90f, 0.25f, 0.20f}, 0.30f}, /* red */
    {{2.8f, 0.8f, 0.5f}, 0.8f, {0.20f, 0.45f, 0.90f}, 0.30f},   /* blue */
};
#define N_SPHERES (int)(sizeof k_spheres / sizeof k_spheres[0])

/*
 * Scene — the whole run: where the camera is in its orbit, how many reflection
 * bounces to allow, and whether the orbit is frozen.
 *
 *   cam_angle   the camera's position around its circle, in radians (advances
 *               with time; the eye point is derived from it each frame).
 *   bounces     current reflection-depth cap, BOUNCES_MIN..BOUNCES_MAX.
 *   paused      when true the orbit stops (the screen still redraws).
 */
typedef struct {
  float cam_angle;
  int bounces;
  bool paused;
} Scene;

/* Terminal size in cells. Row 0 holds the status line; the last row the hints. */
typedef struct {
  int cols, rows;
} Screen;

/*
 * FpsCounter — a smoothed frames-per-second readout. One frame at a time jumps
 * around, so this averages over a short window.
 */
typedef struct {
  int frame_count;
  int64_t window_ns;
  double display;
} FpsCounter;

/*
 * App — everything kept alive for one run.
 *
 * g_app is the only global: the signal handlers need to reach the two flags and
 * can't be handed a pointer, so the flags are sig_atomic_t.
 */
typedef struct {
  Scene scene;
  Screen screen;
  FpsCounter fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

/* ── §3 PERFORMANCE — clock + smoothed fps (frame cap is in §8 main) ── */

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

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns = 0;
  f->display = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
  const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2; /* 500 ms */
  f->frame_count++;
  f->window_ns += dt_ns;
  if (f->window_ns < FPS_WINDOW_NS)
    return;
  f->display =
      (double)f->frame_count * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns = 0;
}

/* ── §4 LOGIC — 3-D vector maths: pure, no mutation, no I/O ── */

static Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static Vec3 vadd(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 vsub(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 vscale(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static Vec3 vcross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static Vec3 vnorm(Vec3 a) {
  float len = sqrtf(vdot(a, a));
  return len > 0.0f ? vscale(a, 1.0f / len) : a;
}

/* Bounce a ray direction off a surface that faces `n` (a mirror reflection). */
static Vec3 vreflect(Vec3 d, Vec3 n) { return vsub(d, vscale(n, 2.0f * vdot(d, n))); }

/* Blend from a to b by t in 0..1. */
static Vec3 vlerp(Vec3 a, Vec3 b, float t) { return vadd(vscale(a, 1.0f - t), vscale(b, t)); }

static Vec3 vclamp01(Vec3 a) {
  float x = a.x < 0 ? 0 : (a.x > 1 ? 1 : a.x);
  float y = a.y < 0 ? 0 : (a.y > 1 ? 1 : a.y);
  float z = a.z < 0 ? 0 : (a.z > 1 ? 1 : a.z);
  return v3(x, y, z);
}

/* ── §5 LOGIC — ray tracing: intersect, shade, recursive trace; pure ── */

/* Fixed scene lighting + colours (pre-normalized light direction toward the sun). */
static const Vec3 LIGHT_DIR = {0.487f, 0.811f, 0.324f};
static const Vec3 WORLD_UP = {0.0f, 1.0f, 0.0f};
static const Vec3 WHITE = {1.0f, 1.0f, 1.0f};
static const Vec3 SKY_HORIZON = {0.80f, 0.86f, 0.95f};
static const Vec3 SKY_ZENITH = {0.25f, 0.45f, 0.85f};
static const Vec3 SUN_COLOR = {1.00f, 0.92f, 0.75f}; /* warm glow of the sun disc */
static const Vec3 CHECK_LIGHT = {0.90f, 0.90f, 0.90f};
static const Vec3 CHECK_DARK = {0.20f, 0.20f, 0.25f};

/* The point on a ray at distance t from its origin. */
static Vec3 ray_at(Vec3 O, Vec3 D, float t) { return vadd(O, vscale(D, t)); }

/* Nearest positive distance where a ray from O in direction D meets a sphere, or
 * -1 for a miss. D is assumed unit length, so the quadratic loses its 'a' term. */
static float sphere_distance(const Sphere *s, Vec3 O, Vec3 D) {
  Vec3 to_center = vsub(O, s->center);
  float half_b = vdot(to_center, D);
  float c = vdot(to_center, to_center) - s->radius * s->radius;
  float disc = half_b * half_b - c;
  if (disc < 0.0f)
    return -1.0f;
  float root = sqrtf(disc);
  float t = -half_b - root; /* near intersection */
  if (t < SURFACE_EPS)
    t = -half_b + root; /* inside the sphere: take the far one */
  return t < SURFACE_EPS ? -1.0f : t;
}

/* Distance where a ray meets the floor plane y = 0, or -1 if it never does. */
static float floor_distance(Vec3 O, Vec3 D) {
  if (fabsf(D.y) < 1e-6f)
    return -1.0f;
  float t = -O.y / D.y;
  return t < SURFACE_EPS ? -1.0f : t;
}

/* The floor's checkerboard colour at a point: alternating squares by the parity
 * of the summed integer coordinates. */
static Vec3 checker_color(Vec3 p) {
  int cell = (int)(floorf(p.x / CHECK_SCALE) + floorf(p.z / CHECK_SCALE));
  return (cell & 1) ? CHECK_DARK : CHECK_LIGHT;
}

/* Sky colour for a ray that hit nothing: a vertical gradient (paler at the
 * horizon, deeper blue up high) plus a warm glow where the ray points toward the
 * sun. The sun sits at a fixed spot in the world, so it slides across the sky as
 * the camera orbits — which is what keeps the sky from looking frozen. */
static Vec3 sky_color(Vec3 D) {
  float up = D.y < 0 ? 0 : (D.y > 1 ? 1 : D.y);
  Vec3 grad = vlerp(SKY_HORIZON, SKY_ZENITH, sqrtf(up)); /* bias so the blue shows */

  float toward_sun = fmaxf(0.0f, vdot(D, LIGHT_DIR));
  float glow = powf(toward_sun, SUN_GLOW_FALLOFF) * SUN_GLOW_STRENGTH +
               powf(toward_sun, SUN_DISC_FALLOFF);
  return vadd(grad, vscale(SUN_COLOR, glow));
}

/* Record a sphere hit into h: where the ray landed, which way the surface faces
 * there, and the sphere's material. */
static void record_sphere_hit(Hit *h, const Sphere *s, Vec3 O, Vec3 D, float t) {
  h->t = t;
  h->point = ray_at(O, D, t);
  h->normal = vnorm(vsub(h->point, s->center));
  h->color = s->color;
  h->reflect = s->reflect;
}

/* Record a floor hit into h: the surface faces straight up and takes its colour
 * from the checkerboard. */
static void record_floor_hit(Hit *h, Vec3 O, Vec3 D, float t) {
  h->t = t;
  h->point = ray_at(O, D, t);
  h->normal = WORLD_UP;
  h->color = checker_color(h->point);
  h->reflect = FLOOR_REFLECT;
}

/* Nearest surface a ray meets; fills `h` and returns true, or false for a miss. */
static bool nearest_hit(Vec3 O, Vec3 D, Hit *h) {
  float best = RAY_FAR;
  bool found = false;

  for (int i = 0; i < N_SPHERES; i++) {
    float t = sphere_distance(&k_spheres[i], O, D);
    if (t > 0.0f && t < best) {
      best = t;
      record_sphere_hit(h, &k_spheres[i], O, D, t);
      found = true;
    }
  }

  float tf = floor_distance(O, D);
  if (tf > 0.0f && tf < best) {
    record_floor_hit(h, O, D, tf);
    found = true;
  }
  return found;
}

/* Is the light blocked by a sphere between this point and the sun? (Only spheres
 * cast shadows; the light comes from above the floor.) */
static bool in_shadow(Vec3 point) {
  Vec3 origin = vadd(point, vscale(LIGHT_DIR, SURFACE_EPS));
  for (int i = 0; i < N_SPHERES; i++)
    if (sphere_distance(&k_spheres[i], origin, LIGHT_DIR) > 0.0f)
      return true;
  return false;
}

/* Direct lighting at a hit: ambient fill, plus (if the sun isn't blocked) diffuse
 * light and a glossy highlight. This is the surface's OWN colour, before mirrors. */
static Vec3 local_shade(const Hit *h, Vec3 view_dir) {
  bool shadowed = in_shadow(h->point);
  float diffuse = shadowed ? 0.0f : fmaxf(0.0f, vdot(h->normal, LIGHT_DIR));
  Vec3 lit = vscale(h->color, AMBIENT + diffuse * (1.0f - AMBIENT));
  if (shadowed)
    return lit;

  Vec3 halfway = vnorm(vsub(LIGHT_DIR, view_dir));
  float glint = powf(fmaxf(0.0f, vdot(h->normal, halfway)), SHININESS);
  return vadd(lit, vscale(WHITE, glint * HIGHLIGHT));
}

/* Follow one ray and return the colour it sees: the nearest surface's own shading
 * blended with what its mirror ray sees, recursing until we run out of bounces. */
static Vec3 cast_ray(Vec3 O, Vec3 D, int bounces) {
  Hit h = {0}; /* fully overwritten on a hit; zeroed to satisfy -Wmaybe-uninitialized */
  if (!nearest_hit(O, D, &h))
    return sky_color(D);

  Vec3 local = local_shade(&h, D);
  if (bounces <= 0 || h.reflect <= 0.0f)
    return local;

  Vec3 mirror_origin = vadd(h.point, vscale(h.normal, SURFACE_EPS));
  Vec3 mirror = cast_ray(mirror_origin, vreflect(D, h.normal), bounces - 1);
  return vlerp(local, mirror, h.reflect);
}

/* ── §6 SIMULATION — orbit the camera ── */

static void scene_init(Scene *s) {
  s->cam_angle = 0.0f;
  s->bounces = BOUNCES_DEFAULT;
  s->paused = false;
}

/* Advance the camera around its circle. While paused it does nothing, but the
 * screen still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->cam_angle += ORBIT_SPEED * dt;
  if (s->cam_angle > 2.0f * (float)M_PI)
    s->cam_angle -= 2.0f * (float)M_PI;
}

/* ── §7 RENDER — shoot a ray per cell → screen; reads state, writes the terminal ── */

/* Set up the colour pairs: on a 256-colour terminal, one pair per colour of the
 * 6×6×6 cube; always the HUD pairs. */
static void colors_init(void) {
  if (g_has_256)
    for (int k = 0; k < 216; k++)
      init_pair(CUBE_BASE + k, 16 + k, COLOR_BLACK);
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* Snap an RGB colour to the nearest slot of the 216-colour cube (0..215). */
static int rgb_to_cube(Vec3 c) {
  int r = (int)(c.x * 5.0f + 0.5f);
  int g = (int)(c.y * 5.0f + 0.5f);
  int b = (int)(c.z * 5.0f + 0.5f);
  return 36 * r + 6 * g + b;
}

/* Perceived brightness of a colour (green counts most, blue least). */
static float luminance(Vec3 c) { return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z; }

/* Stamp one cell: a glyph chosen by brightness, in the ray's colour. */
static void draw_pixel(int row, int col, Vec3 c) {
  int gi = (int)(luminance(c) * (float)RAMP_LEN);
  if (gi < 0)
    gi = 0;
  if (gi >= RAMP_LEN)
    gi = RAMP_LEN - 1;

  attr_t attr = g_has_256 ? COLOR_PAIR(CUBE_BASE + rgb_to_cube(c))
                          : (luminance(c) > 0.6f ? A_BOLD : A_NORMAL);
  attron(attr);
  mvaddch(row, col, (chtype)(unsigned char)k_ramp[gi]);
  attroff(attr);
}

/* Where the camera sits on its circular orbit for a given angle. */
static Vec3 orbit_eye(float angle) {
  return v3(cosf(angle) * ORBIT_RADIUS, CAMERA_HEIGHT, sinf(angle) * ORBIT_RADIUS);
}

/* Horizontal position of a column on the image plane: −1..+1 across the width,
 * scaled by the pixel aspect and the field of view. */
static float image_plane_x(int col, int cols, float aspect, float half) {
  return (2.0f * ((float)col + 0.5f) / (float)cols - 1.0f) * aspect * half;
}

/* Vertical position of a row on the image plane: +1 at the top down to −1 at the
 * bottom, scaled by the field of view. */
static float image_plane_y(int row, int rows, float half) {
  return (1.0f - 2.0f * ((float)row + 0.5f) / (float)rows) * half;
}

/* Shoot a ray from the eye through every cell and paint what it sees. The camera
 * axes are built once; the horizontal is divided by the pixel aspect (cells ~2×
 * tall) so the spheres come out round. */
static void render_scene(const Scene *s, int cols, int rows) {
  Vec3 eye = orbit_eye(s->cam_angle);
  Vec3 fwd = vnorm(vsub(v3(0.0f, LOOK_AT_HEIGHT, 0.0f), eye));
  Vec3 right = vnorm(vcross(fwd, WORLD_UP));
  Vec3 up = vcross(right, fwd);

  float half = tanf(FOV_DEGREES * 0.5f * DEG_TO_RAD);
  float aspect = (float)cols / (CELL_ASPECT * (float)rows);

  for (int row = 0; row < rows; row++) {
    float py = image_plane_y(row, rows, half);
    for (int col = 0; col < cols; col++) {
      float px = image_plane_x(col, cols, aspect, half);
      Vec3 dir = vnorm(vadd(fwd, vadd(vscale(right, px), vscale(up, py))));
      draw_pixel(row, col, vclamp01(cast_ray(eye, dir, s->bounces)));
    }
  }
}

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let stdin interrupt frame writes */
  start_color();
  use_default_colors(); /* lets HUD pairs use the terminal's own background */
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Print one coloured, bold line — shared by both HUD rows. */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top-row status text: fps, reflection bounces, paused. */
static void format_hud_status(const Scene *s, double fps, char *buf, size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  bounces:%d  %s ", fps, s->bounces,
           s->paused ? "PAUSED " : "running");
}

/* Paint the status flush against the right edge of row 0. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
  char buf[HUD_BUF_LEN];
  format_hud_status(s, fps, buf, sizeof buf);
  int right_col = sc->cols - (int)strlen(buf);
  if (right_col < 0)
    right_col = 0;
  hud_paint_text(0, right_col, PAIR_HUD, buf);
}

/* Paint the key-list strip along the bottom row. */
static void draw_hud_hint(const Screen *sc) {
  static const char *KEY_HINT = " q:quit  spc:pause  r:reset  +/-:bounces ";
  hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw_hud(const Screen *sc, double fps, const Scene *s) {
  draw_hud_status(sc, s, fps);
  draw_hud_hint(sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 APP — events (mutate state OUTSIDE the tick) + main loop ── */

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

/* Nudge the reflection-depth cap, clamped to a sane range. */
static void scene_adjust_bounces(Scene *s, int delta) {
  s->bounces += delta;
  if (s->bounces < BOUNCES_MIN)
    s->bounces = BOUNCES_MIN;
  if (s->bounces > BOUNCES_MAX)
    s->bounces = BOUNCES_MAX;
}

/* Re-read the size on resize; the image is recomputed each frame, so there is
 * nothing to rebuild. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  app->need_resize = 0;
}

/* Act on one keypress. Returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;

  case ' ':
  case 'p':
  case 'P':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    s->cam_angle = 0.0f;
    break;

  case '+':
  case '=':
    scene_adjust_bounces(s, +1);
    break;
  case '-':
  case '_':
    scene_adjust_bounces(s, -1);
    break;

  default:
    break;
  }
  return true;
}

/* The main loop: each pass measures real elapsed time, reads keys, orbits the
 * camera by that much, ray-traces the frame, then sleeps to hold ~60 fps. */
int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  Scene *scene = &app->scene;
  app->running = 1;
  fps_counter_init(&app->fps);

  screen_init(&app->screen);
  g_has_256 = (COLORS >= 256);
  scene_init(scene);
  colors_init();

  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
  int64_t last_ns = clock_ns();

  while (app->running) {
    if (app->need_resize) {
      app_do_resize(app);
      last_ns = clock_ns();
    }

    int64_t frame_start_ns = clock_ns();
    int64_t dt_ns = frame_start_ns - last_ns;
    last_ns = frame_start_ns;
    float dt = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC)
      dt = DT_CAP_SEC;

    for (int ch; (ch = getch()) != ERR;) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    scene_tick(scene, dt);
    fps_counter_tick(&app->fps, dt_ns);

    erase();
    render_scene(scene, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
