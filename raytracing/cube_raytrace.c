/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* A cube spinning in space, ray-traced one terminal character at a time. Each
 * character shoots a ray; if it hits the cube we work out the colour and
 * brightness there and pick a glyph for it. Lights are plain white, so each
 * material — gold, glass, neon, … — shows its own real look. Keys spin, zoom,
 * and switch material/view (the on-screen hint strip lists them).
 * Same skeleton, other shapes: sphere_raytrace.c, capsule_raytrace.c,
 * torus_raytrace.c. */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1  settings & constants ── */

/* §1.1 frame pacing */
#define TARGET_FPS 60
#define DT_CAP_NS 100000000LL /* if a frame stalls, pretend at most 0.1s passed
                               * so the cube can't suddenly lurch forward */

/* §1.2 view. Terminal cells are taller than they are wide, so we squash the
 * picture vertically to stop the cube looking stretched. FOV = how wide a
 * view the camera takes in. */
#define ASPECT 0.47f
#define FOV_DEG 55.0f

/* §1.3 cube half-size: it runs from -CUBE_S to +CUBE_S on each axis, centred at 0 */
#define CUBE_S 0.80f

/* §1.4 wireframe view: how near an edge still counts as "on the edge", and how
 * bright those edge characters are */
#define WIRE_THRESH 0.055f  /* edge band width, as a fraction of the cube size */
#define WIRE_LUMA_BASE 0.7f /* brightness at the outer rim of the band */
#define WIRE_LUMA_GAIN 0.3f /* extra brightness right on the edge */

/* §1.5 spin speed (radians per second) */
#define ROT_Y 0.52f /* main turn, left-right */
#define ROT_X 0.35f /* gentler tilt, up-down */

/* §1.6 camera pull-back; bigger = cube looks smaller / further away */
#define CAM_DIST_DEF 3.2f
#define CAM_DIST_MIN 1.5f
#define CAM_DIST_MAX 7.0f
#define CAM_DIST_STEP 0.25f

/* §1.7 lighting knobs */
#define AMBIENT 0.20f   /* faint base glow so shadowed faces aren't pure black */
#define SHININESS 32.0f /* highlight tightness. Kept low on purpose: a cube face
                         * is flat, so a tight highlight would flip the whole
                         * face bright-or-dark at once. Low spreads it into a
                         * gradient the character grid can actually show. */
#define RIM_SHININESS 10.f  /* the back light's highlight, wider than the rest */
#define FRESNEL_POWER 2.5f  /* how fast edges brighten as they face away from us */
#define FRESNEL_GAIN 0.5f   /* how strong that edge brightening is */
#define DEPTH_FAR_SCALE 2.f /* depth view: how far away counts as fully dark */

/* §1.8 brightness-to-character ladder, dark on the left to bright on the right:
 * we pick a glyph by how bright a point is, so " " is darkest and "@" brightest.
 * (Paul Bourke's 92-character set.) */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.9 colour. Terminals give us a 6×6×6 grid of colours (xterm 16..231); we
 * claim ncurses colour-slots 1..216 for those, plus two for the HUD text. */
#define CUBE_SIDE 6
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.10 tiny tolerances for the hit math */
#define T_EPS 1e-4f        /* ignore hits this close to the ray's start (self-hits) */
#define PARALLEL_EPS 1e-9f /* a direction this small counts as "parallel to a wall" */
#define T_EXIT_MATCH_EPS                                                        \
  1e-6f /* slop for "is this the wall the ray leaves through?" */

/* §1.11 stand-in for infinity while we narrow down the hit range */
#define BIG_T 1e30f

/* ── §2  timing helpers ── */

/* A clock that only ever counts forward, so animation timing isn't thrown off
 * if the system clock jumps (NTP, daylight saving, etc.). */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* Sleep the unused part of a frame so we don't spin a CPU core at 100%. */
static void clock_sleep_ns(long long nanoseconds) {
  if (nanoseconds <= 0)
    return;
  struct timespec request = {nanoseconds / 1000000000LL,
                             nanoseconds % 1000000000LL};
  nanosleep(&request, NULL);
}

/* ── §3  3-D vectors and rotations ── */

/* A point or direction in 3-D — and we reuse it as an R,G,B colour too, since
 * both are just three numbers. Passed around by value (it's tiny) to keep the
 * per-character loop fast. */
typedef struct {
  float x, y, z;
} V3;

static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3sub(V3 a, V3 b) {
  return (V3){a.x - b.x, a.y - b.y, a.z - b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }

static inline V3 v3norm(V3 a) {
  float length = v3len(a);
  return (length > 1e-9f) ? v3scale(1.f / length, a) : (V3){0.f, 1.f, 0.f};
}

/* Bounce a direction off a surface, like a ball off a wall. Used to turn
 * "where the light comes from" into "where it reflects toward". */
static inline V3 v3reflect(V3 v, V3 n) {
  return v3sub(v, v3scale(2.f * v3dot(v, n), n));
}

static inline V3 v3clamp01(V3 v) {
  return (V3){v.x < 0.f   ? 0.f
              : v.x > 1.f ? 1.f
                          : v.x,
              v.y < 0.f   ? 0.f
              : v.y > 1.f ? 1.f
                          : v.y,
              v.z < 0.f   ? 0.f
              : v.z > 1.f ? 1.f
                          : v.z};
}

/* A 3×3 rotation, stored as three rows. Because it's a pure rotation, flipping
 * it along the diagonal (the transpose) gives its inverse for free — the trick
 * that lets us rotate the *ray* into the cube's frame instead of the cube. */
typedef struct {
  V3 row[3];
} Mat3;

/* Builds the cube's current orientation from two spin angles (radians): a turn
 * left-right plus a tilt up-down. */
static Mat3 mat3_rotation(float angle_x, float angle_y) {
  float cos_x = cosf(angle_x), sin_x = sinf(angle_x);
  float cos_y = cosf(angle_y), sin_y = sinf(angle_y);
  Mat3 m;
  m.row[0] = (V3){cos_y, 0.f, sin_y};
  m.row[1] = (V3){sin_x * sin_y, cos_x, -sin_x * cos_y};
  m.row[2] = (V3){-cos_x * sin_y, sin_x, cos_x * cos_y};
  return m;
}

/* Rotate a vector by the matrix — used to turn which-way-a-face-points from the
 * cube's own frame back into world space, so we can light it. */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.row[0], v), v3dot(m.row[1], v), v3dot(m.row[2], v)};
}

/* Rotate by the inverse (the same matrix, just flipped — see Mat3). Used to push
 * a world-space ray into the cube's own frame, where the cube sits square to the
 * axes and the hit test is simple. */
static V3 mat3_mulT(Mat3 m, V3 v) {
  return (V3){m.row[0].x * v.x + m.row[1].x * v.y + m.row[2].x * v.z,
              m.row[0].y * v.x + m.row[1].y * v.y + m.row[2].y * v.z,
              m.row[0].z * v.x + m.row[1].z * v.y + m.row[2].z * v.z};
}

/* A ray: a start point and a direction — the straight line a single character
 * "looks" along. The two always travel together, so they share one struct.
 *   origin  where the ray starts (the camera, in world space)
 *   dir     which way it points (we keep it unit length so hit distances
 *           come out as real distances) */
typedef struct {
  V3 origin;
  V3 dir;
} Ray;

/* ── §4  materials (what each surface is made of) ── */

/* What a surface is made of. The lights are plain white on purpose, so a
 * material's whole look comes from these numbers — gold reads as gold, glass as
 * glass — rather than being painted on by a coloured light.
 *   albedo          the body colour you'd see in flat, even light (0..1 RGB)
 *   specular        colour of the shiny highlight. For metals it matches the
 *                   body colour (a gold highlight is yellow); for everything
 *                   else it's near-white. (Graphics calls this "F0".)
 *   emissive        colour the surface gives off by itself, added after the
 *                   lighting so it glows even in shadow. Usually black; neon.
 *   diffuse_weight  how much plain body colour shows vs. shine, 0..1: metals
 *                   ~0.15 (almost all shine), gems ~0.70, plastic/ceramic ~0.85,
 *                   glass ~0.10 (dark body + bright shine fakes see-through),
 *                   neon ~0.20.
 *   name            shown in the status line.
 * The 20 materials below come in 4 families; t / T cycle through them. Metal
 * colours are real measured values (Naty Hoffman, "Physics and Math of
 * Shading", SIGGRAPH 2013), nudged a little for terminal contrast. */
typedef struct {
  V3 albedo;
  V3 specular;
  V3 emissive;
  float diffuse_weight;
  const char *name;
} Material;

static const Material g_materials[] = {
    /* Metals (12): almost all shine, very little body colour, and the shine is
     * tinted like the metal itself (a gold highlight is yellow). */

    /* gold     — warm yellow precious metal                            */
    {{1.00f, 0.77f, 0.34f},
     {1.00f, 0.77f, 0.34f},
     {0.f, 0.f, 0.f},
     0.15f,
     "gold"},
    /* silver   — bright cool precious metal, near-pure white           */
    {{0.97f, 0.96f, 0.92f},
     {0.97f, 0.96f, 0.92f},
     {0.f, 0.f, 0.f},
     0.15f,
     "silver"},
    /* copper   — warm orange-red metal                                 */
    {{0.96f, 0.64f, 0.54f},
     {0.96f, 0.64f, 0.54f},
     {0.f, 0.f, 0.f},
     0.15f,
     "copper"},
    /* bronze   — warm brown alloy (Cu+Sn)                              */
    {{0.78f, 0.55f, 0.30f},
     {0.78f, 0.55f, 0.30f},
     {0.f, 0.f, 0.f},
     0.15f,
     "bronze"},
    /* brass    — yellow-green alloy (Cu+Zn)                            */
    {{0.85f, 0.70f, 0.25f},
     {0.85f, 0.70f, 0.25f},
     {0.f, 0.f, 0.f},
     0.15f,
     "brass"},
    /* platinum — cool greyish-white precious metal                     */
    {{0.83f, 0.81f, 0.78f},
     {0.83f, 0.81f, 0.78f},
     {0.f, 0.f, 0.f},
     0.15f,
     "platinum"},
    /* titanium — dark silvery metal                                    */
    {{0.62f, 0.60f, 0.55f},
     {0.62f, 0.60f, 0.55f},
     {0.f, 0.f, 0.f},
     0.15f,
     "titanium"},
    /* iron     — neutral grey base metal                               */
    {{0.56f, 0.57f, 0.58f},
     {0.56f, 0.57f, 0.58f},
     {0.f, 0.f, 0.f},
     0.15f,
     "iron"},
    /* steel    — cool blue-grey alloy                                  */
    {{0.65f, 0.70f, 0.78f},
     {0.65f, 0.70f, 0.78f},
     {0.f, 0.f, 0.f},
     0.15f,
     "steel"},
    /* chrome   — mirror-bright cool metal                              */
    {{0.92f, 0.94f, 0.96f},
     {0.92f, 0.94f, 0.96f},
     {0.f, 0.f, 0.f},
     0.15f,
     "chrome"},
    /* mercury  — liquid silver                                         */
    {{0.85f, 0.85f, 0.88f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.15f,
     "mercury"},
    /* aluminum — pale neutral metal                                    */
    {{0.91f, 0.92f, 0.92f},
     {0.91f, 0.92f, 0.92f},
     {0.f, 0.f, 0.f},
     0.15f,
     "aluminum"},

    /* Gems (4): a deep, rich body colour with a white highlight. */

    /* ruby     — red corundum (Cr-doped)                               */
    {{0.85f, 0.10f, 0.18f},
     {1.00f, 0.95f, 0.95f},
     {0.f, 0.f, 0.f},
     0.70f,
     "ruby"},
    /* emerald  — green beryl (Cr-doped)                                */
    {{0.10f, 0.70f, 0.30f},
     {0.95f, 1.00f, 0.95f},
     {0.f, 0.f, 0.f},
     0.70f,
     "emerald"},
    /* sapphire — blue corundum (Fe/Ti-doped)                           */
    {{0.10f, 0.30f, 0.88f},
     {0.95f, 0.95f, 1.00f},
     {0.f, 0.f, 0.f},
     0.70f,
     "sapphire"},
    /* amethyst — purple quartz                                         */
    {{0.55f, 0.30f, 0.85f},
     {1.00f, 0.95f, 1.00f},
     {0.f, 0.f, 0.f},
     0.70f,
     "amethyst"},

    /* Plastic, glass, ceramic (3): full body colour with a white highlight. */

    /* plastic  — saturated blue plastic, full body colour              */
    {{0.20f, 0.40f, 0.92f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.85f,
     "plastic"},
    /* glass    — dark base + bright spec fakes transparency            */
    {{0.10f, 0.12f, 0.16f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.10f,
     "glass"},
    /* ceramic  — soft warm-cream porcelain                             */
    {{0.92f, 0.90f, 0.85f},
     {1.00f, 0.98f, 0.95f},
     {0.f, 0.f, 0.f},
     0.85f,
     "ceramic"},

    /* Neon (1): a dark "off" body that glows hot pink on its own, even in
     * shadow, because the emissive colour is added after lighting. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f, 0.02f, 0.10f},
     {0.80f, 0.80f, 1.00f},
     {1.00f, 0.20f, 0.85f},
     0.20f,
     "neon"},
};
#define MATERIAL_N ((int)(sizeof g_materials / sizeof g_materials[0]))

/* ── §5  does a ray hit the cube? (the core) ── *
 *
 * Think of the cube as the overlap of three "slabs": the gap between the
 * left/right walls, the bottom/top walls, and the front/back walls. A ray is
 * inside the cube only while it's inside all three at once. So for each pair of
 * walls we find the stretch of the ray that lies between them, then overlap the
 * three stretches — if anything's left (and it's in front of us), that's a hit.
 * No square roots, just a few comparisons per character.
 * (The classic "slab method": Kay & Kajiya 1986; Ericson, Real-Time Collision
 * Detection §5.3.) */

/* What we learn when a ray meets the cube — filled once, then read by the
 * shading and the debug views so nothing gets worked out twice. Computed in the
 * cube's own frame; the face direction is rotated back to world space later.
 *   hit            did it hit? Check this first — the rest is meaningless if 0.
 *   axis_at_enter  which pair of walls it came in through (0/1/2 = X/Y/Z). This
 *                  is what tells us which way the face points; the AXIS debug
 *                  view colours by it.
 *   t_hit          how far along the ray the visible surface is. The entry point
 *                  normally; the exit point if the camera is inside the cube.
 *   t_enter        when the ray enters the cube (the latest of the three slabs).
 *   t_exit         when it leaves (the earliest of the three). exit − enter is
 *                  how far it travels through the cube; the INTERVAL view uses it.
 *   normal_obj     which way the hit face points (one of ±X/±Y/±Z), in the
 *                  cube's frame — the starting point for all the shading.
 *   inside         1 if the camera started inside the cube. */
typedef struct {
  int hit;
  int axis_at_enter;
  float t_hit;
  float t_enter;
  float t_exit;
  V3 normal_obj;
  int inside;
} BoxHit;

/* The thing we're drawing: a cube at the origin, the same size on every axis,
 * plus how far it has spun. We keep the cube square to the axes and spin the
 * *ray* instead — far simpler math — so the spin only ever touches the rays.
 *   half_extent  half the cube's width; it runs -half_extent .. +half_extent
 *   spin_x       tilt so far, up-down (radians)
 *   spin_y       turn so far, left-right (radians) */
typedef struct {
  float half_extent;
  float spin_x;
  float spin_y;
} Box;

/* One axis at a time: when is the ray between this pair of walls? Hands back the
 * time it enters that gap, the time it leaves, and which way the entry wall
 * faces. Returns 0 (an instant, total miss) only in the odd case where the ray
 * runs parallel to the walls and starts outside them; if it's parallel but
 * between them, it's always between them, so we report no time limit.
 * (We use min/max instead of a sign test because a ray pointing the negative
 * way crosses the far wall first.) */
static int slab_test(float ray_origin_i, float ray_dir_i, float half_extent,
                     float *out_t_enter, float *out_t_exit,
                     float *out_enter_sign) {
  if (fabsf(ray_dir_i) < PARALLEL_EPS) {
    /* ray runs parallel to these two walls */
    if (ray_origin_i < -half_extent || ray_origin_i > half_extent)
      return 0;            /* started outside them → never between them */
    *out_t_enter = -BIG_T; /* always between them → no time limit */
    *out_t_exit = BIG_T;
    *out_enter_sign = 0.f; /* never crosses an entry wall here */
    return 1;
  }
  float inv_dir = 1.f / ray_dir_i;
  float t0 = (-half_extent - ray_origin_i) * inv_dir;
  float t1 = (half_extent - ray_origin_i) * inv_dir;
  if (t0 < t1) {
    *out_t_enter = t0;
    *out_t_exit = t1;
  } else {
    *out_t_enter = t1;
    *out_t_exit = t0;
  }
  /* the entry wall faces back against the ray */
  *out_enter_sign = (ray_dir_i > 0.f) ? -1.f : 1.f;
  return 1;
}

/* Turn "which wall, which side" into the direction that face points — one of
 * ±X / ±Y / ±Z. */
static inline V3 axis_normal(int axis, float sign) {
  V3 normal = {0.f, 0.f, 0.f};
  if (axis == 0)
    normal.x = sign;
  else if (axis == 1)
    normal.y = sign;
  else
    normal.z = sign;
  return normal;
}

/* When the camera started inside the cube, the surface we can see is the one the
 * ray leaves through. Find that wall (its exit time matches the cube's exit
 * time); its face points the way the ray is heading. Returns the axis and its
 * sign, or -1 if nothing matches. */
static int exit_face(const float ray_origin_arr[3], const float ray_dir_arr[3],
                     float half_extent, float t_exit, float *out_sign) {
  for (int axis = 0; axis < 3; axis++) {
    float t_enter_slab, t_exit_slab, slab_enter_sign;
    if (!slab_test(ray_origin_arr[axis], ray_dir_arr[axis], half_extent,
                   &t_enter_slab, &t_exit_slab, &slab_enter_sign))
      return -1;
    if (fabsf(t_exit_slab - t_exit) < T_EXIT_MATCH_EPS) {
      *out_sign = (ray_dir_arr[axis] > 0.f) ? +1.f : -1.f;
      return axis;
    }
  }
  return -1;
}

/* The hit test. Check each pair of walls (X, Y, Z) and keep the overlap: enter
 * at the latest of the three entry times, leave at the earliest of the exits. If
 * those cross, the ray misses; if the whole overlap is behind us, it misses too.
 * Normally we see the entry face; if the camera is inside the cube we hand back
 * the exit face. The face we came in through is just the axis whose entry won. */
static BoxHit ray_aabb(Ray ray, float half_extent) {
  float ray_origin_arr[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
  float ray_dir_arr[3] = {ray.dir.x, ray.dir.y, ray.dir.z};

  BoxHit out = {.hit = 0};

  float t_enter = -BIG_T, t_exit = BIG_T;
  int axis_at_enter = -1;
  float enter_sign = 0.f;

  for (int axis = 0; axis < 3; axis++) {
    float t_enter_slab, t_exit_slab, slab_enter_sign;
    if (!slab_test(ray_origin_arr[axis], ray_dir_arr[axis], half_extent,
                   &t_enter_slab, &t_exit_slab, &slab_enter_sign))
      return out; /* runs alongside these walls and outside them → miss */

    if (t_enter_slab > t_enter) { /* latest entry wins, and names the face */
      t_enter = t_enter_slab;
      axis_at_enter = axis;
      enter_sign = slab_enter_sign;
    }
    if (t_exit_slab < t_exit)
      t_exit = t_exit_slab;

    if (t_enter > t_exit) /* the ranges no longer overlap → miss */
      return out;
  }

  if (t_exit < T_EPS)
    return out; /* the whole cube is behind the camera */

  out.t_enter = t_enter;
  out.t_exit = t_exit;

  float t_hit;
  if (t_enter > T_EPS) {
    t_hit = t_enter;
    out.inside = 0;
  } else {
    t_hit = t_exit;
    out.inside = 1;
    /* Camera inside → the visible surface is the EXIT face. */
    axis_at_enter = exit_face(ray_origin_arr, ray_dir_arr, half_extent, t_exit,
                              &enter_sign);
    if (axis_at_enter < 0)
      return out;
  }

  out.normal_obj = axis_normal(axis_at_enter, enter_sign);

  out.t_hit = t_hit;
  out.axis_at_enter = axis_at_enter;
  out.hit = 1;
  return out;
}

/* How close a hit point is to the edge of its face, for the wireframe view:
 * 0 right on an edge, 1 dead centre. We look at the two coordinates that run
 * along the face and take whichever is nearest its border. */
static float face_edge_dist(V3 point_obj, V3 normal_obj, float half_extent) {
  float u, v;
  if (fabsf(normal_obj.x) > 0.5f) {
    u = point_obj.y;
    v = point_obj.z;
  } else if (fabsf(normal_obj.y) > 0.5f) {
    u = point_obj.x;
    v = point_obj.z;
  } else {
    u = point_obj.x;
    v = point_obj.y;
  }

  float du = half_extent - fabsf(u);
  float dv = half_extent - fabsf(v);
  return fminf(du, dv) / half_extent;
}

/* Where a hit sits on its face, as two numbers in -1..+1 (centre is 0,0, a
 * corner is ±1,±1). Used by the face-uv debug view. */
static void face_uv(V3 point_obj, V3 normal_obj, float half_extent,
                    float *out_u, float *out_v) {
  float u, v;
  if (fabsf(normal_obj.x) > 0.5f) {
    u = point_obj.y;
    v = point_obj.z;
  } else if (fabsf(normal_obj.y) > 0.5f) {
    u = point_obj.x;
    v = point_obj.z;
  } else {
    u = point_obj.x;
    v = point_obj.y;
  }
  *out_u = u / half_extent;
  *out_v = v / half_extent;
}

/* ── §6  turning a hit into a colour ── */

/* The four ways to colour a hit, switched with `s`: one real lit look plus
 * three views that show the math behind it. MODE_N is just the count, so we can
 * wrap around with (mode + 1) % MODE_N; the order matches k_mode_names[] and the
 * switch in shade_surface.
 *   MODE_PHONG   the proper lit look
 *   MODE_NORMAL  paint each face by which way it points (a normals check)
 *   MODE_WIRE    draw only the edges
 *   MODE_DEPTH   brighter = closer */
typedef enum {
  MODE_PHONG = 0,
  MODE_NORMAL,
  MODE_WIRE,
  MODE_DEPTH,
  MODE_N
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "wireframe",
                                           "depth"};

/* One light in the three-light setup: where it sits, plus a few dials for how it
 * adds in. Keeping each light's dials next to its position lets the shader just
 * loop over the lights instead of repeating itself three times.
 *   pos        where the light is (a spot in space, not a direction)
 *   diffuse    how much soft, all-over light it adds
 *   specular   how much shine it adds (0 = a soft-only light, like the fill)
 *   shininess  how tight that shine is (smaller = broader) */
typedef struct {
  V3 pos;
  float diffuse;
  float specular;
  float shininess;
} Light;

/* Three white lights, the classic photo setup:
 *   KEY  main light, up and to the right — bright, with a crisp shine
 *   FILL down to the left — soft only, just lifts the shadow side
 *   RIM  behind and low — a wide back-glow that picks out the edge */
static const Light g_lights[] = {
    {{3.0f, 4.0f, -2.0f}, 1.00f, 1.30f, SHININESS},     /* KEY  */
    {{-4.0f, 1.0f, -1.0f}, 0.55f, 0.00f, SHININESS},    /* FILL */
    {{0.5f, -1.0f, 5.0f}, 0.40f, 1.20f, RIM_SHININESS}, /* RIM  */
};
#define LIGHT_N ((int)(sizeof g_lights / sizeof g_lights[0]))

/* Add one white light's share to a running colour: a soft part that's strongest
 * where the surface faces the light head-on, plus a shiny part that flares where
 * the light bounces straight back toward the camera. (We hand the bounce the
 * flipped light direction so "light coming in" lines up with "bounce going out".) */
static V3 add_phong_light(V3 colour, const Light *light, V3 point_world,
                          V3 normal_world, V3 view_dir,
                          const Material *material) {
  V3 light_dir = v3norm(v3sub(light->pos, point_world));
  float diffuse = fmaxf(0.f, v3dot(normal_world, light_dir));
  colour = v3add(colour, v3scale(diffuse * material->diffuse_weight *
                                     light->diffuse,
                                 material->albedo));
  V3 reflect_dir = v3reflect(v3scale(-1.f, light_dir), normal_world);
  float specular =
      powf(fmaxf(0.f, v3dot(reflect_dir, view_dir)), light->shininess);
  colour =
      v3add(colour, v3scale(specular * light->specular, material->specular));
  return colour;
}

/* The real lit look. Faint base glow, then the three lights, then a rim that
 * brightens edges turning away from us. That rim matters because a cube face is
 * flat: the lights alone paint each face one even tone, and the rim is what
 * gives it shape and makes the edges show up on a coarse character grid. Last,
 * add any self-glow (neon). Because the lights are white, each material ends up
 * showing its own colour. */
static V3 shade_phong(V3 point_world, V3 normal_world, V3 view_dir,
                      const Material *material) {
  V3 colour = v3scale(AMBIENT, material->albedo); /* faint base glow */

  for (int i = 0; i < LIGHT_N; i++)
    colour = add_phong_light(colour, &g_lights[i], point_world, normal_world,
                             view_dir, material);

  /* Rim: brighten the surface as it turns edge-on to us. It's tinted by the
   * material's shine colour, so metals get a coloured edge and others white. */
  float facing = v3dot(normal_world, view_dir);
  /* Clamp facing to 0..1 on BOTH ends. A face seen dead-on can round just above
   * 1, making (1 - facing) negative; raising a negative to a fractional power
   * gives NaN, which then picks a garbage character and crashes. (We hit this.) */
  facing = facing < 0.f ? 0.f : (facing > 1.f ? 1.f : facing);
  float fresnel = powf(1.f - facing, FRESNEL_POWER);
  colour = v3add(colour, v3scale(fresnel * FRESNEL_GAIN, material->specular));

  colour = v3add(colour, material->emissive); /* self-glow, shows even in shadow */
  return v3clamp01(colour);
}

/* Debug view: colour a face by which way it points, so you can eyeball the face
 * directions. Each cube face points one fixed way, so it comes out one flat
 * colour. (x,y,z of the direction shifted from -1..1 into 0..1.) */
static V3 shade_normal(V3 normal_world) {
  return (V3){normal_world.x * 0.5f + 0.5f, normal_world.y * 0.5f + 0.5f,
              normal_world.z * 0.5f + 0.5f};
}

/* Wireframe tint, only used for cells right by an edge: brighter the closer to
 * the edge, coloured by the face it belongs to. */
static V3 shade_wire(V3 normal_world, float edge_dist) {
  V3 colour = shade_normal(normal_world);
  float k = 1.f - edge_dist / WIRE_THRESH; /* 1 on the edge, 0 at the band's rim */
  return v3clamp01(v3scale(0.6f + 0.4f * k, colour));
}

/* Debug view: nearer surfaces are brighter, far ones fade to black. */
static V3 shade_depth(float t_hit, float distance_max, const Material *material) {
  float depth_norm = 1.f - fminf(t_hit / distance_max, 1.f);
  depth_norm = depth_norm * depth_norm;
  return v3clamp01(v3scale(depth_norm, material->albedo));
}

/* Boil a colour down to one brightness number, weighted the way the eye sees it
 * (green counts most), so we can pick a character for it. */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* Same idea for the "which way it faces" view. Its false colours look wrong
 * under plain brightness, so we lean even harder on green to match how bright
 * they feel. */
static inline float normal_vis_luma(V3 normal_world) {
  return (normal_world.x * 0.5f + 0.5f) * 0.30f +
         (normal_world.y * 0.5f + 0.5f) * 0.60f +
         (normal_world.z * 0.5f + 0.5f) * 0.10f;
}

/* Work out the colour and brightness for one hit in the current view. Returns 0
 * to leave the cell blank — only the wireframe view does that, skipping the
 * middle of each face so just the edges show. */
static int shade_surface(ShadeMode mode, const BoxHit *hit, V3 hit_point_obj,
                         V3 hit_point_world, V3 normal_world, V3 view_dir,
                         float cam_dist, float half_extent, const Material *material,
                         V3 *out_colour, float *out_luminance) {
  switch (mode) {
  default:
  case MODE_PHONG:
    *out_colour = shade_phong(hit_point_world, normal_world, view_dir, material);
    *out_luminance = rec601_luma(*out_colour);
    break;
  case MODE_NORMAL:
    *out_colour = shade_normal(normal_world);
    *out_luminance = normal_vis_luma(normal_world);
    break;
  case MODE_WIRE: {
    float edge_dist =
        face_edge_dist(hit_point_obj, hit->normal_obj, half_extent);
    if (edge_dist > WIRE_THRESH)
      return 0; /* face interior — nothing to draw */
    *out_colour = shade_wire(normal_world, edge_dist);
    *out_luminance =
        WIRE_LUMA_BASE + WIRE_LUMA_GAIN * (1.f - edge_dist / WIRE_THRESH);
    break;
  }
  case MODE_DEPTH:
    *out_colour = shade_depth(hit->t_hit, cam_dist * DEPTH_FAR_SCALE, material);
    *out_luminance = rec601_luma(*out_colour);
    break;
  }
  return 1;
}

/* ── §7  debug views of the math ── *
 *
 * Each replaces the normal colour with a picture of what the hit test found, so
 * the math isn't a black box. Switched with `d`. */

/* A debug overlay, switched with `d`. DEBUG_N is the count, for wrap-around;
 * order matches k_debug_names[] and apply_debug.
 *   DEBUG_OFF       leave the normal colour
 *   DEBUG_AXIS      colour by which pair of walls the ray came in through
 *   DEBUG_INTERVAL  brighter where the ray passes deeper through the cube
 *   DEBUG_FACE_UV   colour by where the hit lands on its face */
typedef enum {
  DEBUG_OFF = 0,
  DEBUG_AXIS,
  DEBUG_INTERVAL,
  DEBUG_FACE_UV,
  DEBUG_N
} DebugMode;
static const char *const k_debug_names[] = {"off", "axis", "interval",
                                            "face-uv"};

/* Blend from a cool blue to a warm yellow as t goes 0 → 1. */
static V3 gradient_cold_hot(float t) {
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  V3 cold = (V3){0.10f, 0.20f, 0.95f};
  V3 hot = (V3){1.00f, 0.85f, 0.20f};
  return (V3){cold.x + (hot.x - cold.x) * t, cold.y + (hot.y - cold.y) * t,
              cold.z + (hot.z - cold.z) * t};
}

/* Pick the overlay colour + brightness for a hit:
 *   AXIS      red/green/blue = which pair of walls (X/Y/Z) the ray entered
 *   INTERVAL  brighter where the ray cuts deeper through the cube
 *   FACE-UV   colour by where the hit lands on its face */
static void apply_debug(DebugMode mode, const BoxHit *hit, V3 hit_point_obj,
                        float half_extent, V3 *out_colour,
                        float *out_luminance) {
  switch (mode) {
  case DEBUG_AXIS: {
    V3 c = (V3){0.f, 0.f, 0.f};
    switch (hit->axis_at_enter) {
    case 0:
      c = (V3){0.95f, 0.20f, 0.20f};
      break; /* red    */
    case 1:
      c = (V3){0.20f, 0.95f, 0.30f};
      break; /* green  */
    case 2:
      c = (V3){0.30f, 0.40f, 1.00f};
      break; /* blue   */
    default:
      break;
    }
    *out_colour = c;
    *out_luminance = 0.85f;
    break;
  }
  case DEBUG_INTERVAL: {
    /* how far the ray travels inside the cube, scaled to 0..1 against the
     * longest path it could take (corner to corner) */
    float thickness = hit->t_exit - hit->t_enter;
    float norm = thickness / (2.f * sqrtf(3.f) * half_extent);
    if (norm < 0.f)
      norm = 0.f;
    if (norm > 1.f)
      norm = 1.f;
    *out_colour = gradient_cold_hot(norm);
    *out_luminance = 0.3f + 0.65f * norm;
    break;
  }
  case DEBUG_FACE_UV: {
    float u, v;
    face_uv(hit_point_obj, hit->normal_obj, half_extent, &u, &v);
    /* position on the face → red/green; blue fixed so only the colour shifts */
    *out_colour = (V3){u * 0.5f + 0.5f, v * 0.5f + 0.5f, 0.5f};
    *out_luminance = 0.85f;
    break;
  }
  default:
    /* DEBUG_OFF — caller keeps its own colour. */
    break;
  }
}

/* ── §7.5  drawing a coloured character ── */

static int g_have_256; /* true if the terminal has 256 colours; false = plain */

/* Claim the colour slots once: 1..216 for the 6×6×6 colour grid, plus two for
 * the HUD text. */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);
  if (g_have_256) {
    for (int i = 0; i < 216; i++)
      init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
  }
  init_pair(PAIR_HUD, 226, -1);
  init_pair(PAIR_HINT, 51, -1);
}

/* Pick a character for a brightness 0..1: dark → " ", bright → "@". */
static inline char ramp_char(float luminance) {
  if (luminance < 0.f)
    luminance = 0.f;
  if (luminance > 1.f)
    luminance = 1.f;
  return k_ramp[(int)(luminance * (RAMP_LEN - 1))];
}

/* Snap one colour channel (0..1) onto the terminal's 6 brightness levels. */
static inline int cube_level(float channel) {
  int level = (int)(channel * (CUBE_SIDE - 1) + 0.5f);
  return level > CUBE_SIDE - 1 ? CUBE_SIDE - 1 : level;
}

/* Turn an R,G,B colour into the matching terminal colour-slot number. */
static inline int cube_pair(V3 colour) {
  return PAIR_CUBE_BASE + cube_level(colour.x) * CUBE_SIDE * CUBE_SIDE +
         cube_level(colour.y) * CUBE_SIDE + cube_level(colour.z);
}

/* Draw one cell: a character chosen by brightness, in the colour for this RGB
 * (plain terminals just get the character). */
static void draw_color(int row, int col, V3 colour, float luminance) {
  char ch = ramp_char(luminance);

  if (g_have_256) {
    int pair = cube_pair(colour);
    attron(COLOR_PAIR(pair));
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair));
  } else {
    mvaddch(row, col, (chtype)(unsigned char)ch);
  }
}

/* ── §8  shoot one ray per character ── *
 *
 * For each cell we build a ray from the camera through it, rotate it into the
 * cube's frame, test for a hit, and if it hits, shade it and draw it. The bottom
 * row is left empty so the key-hint strip has somewhere to sit. */

/* The ray from the camera out through one character cell. We turn the cell's
 * column and row into a direction, squashing the vertical a little so the tall
 * cells don't stretch the cube, then aim it out from the camera. */
static V3 primary_ray_dir(int col, int row, float screen_centre_x,
                          float screen_centre_y, float fov_half_tan,
                          V3 view_right, V3 view_up, V3 view_forward) {
  float screen_u = (col - screen_centre_x) / screen_centre_x * fov_half_tan;
  float screen_v =
      -(row - screen_centre_y) / screen_centre_x * fov_half_tan / ASPECT;
  return v3norm(v3add(view_forward, v3add(v3scale(screen_u, view_right),
                                          v3scale(screen_v, view_up))));
}

static void render(const Box *box, const Material *material, ShadeMode shade_mode,
                   DebugMode debug_mode, float cam_dist, int cols, int rows) {
  float fov_half_tan = tanf(FOV_DEG * (float)M_PI / 360.f);

  /* the cube's current spin, as a rotation we can apply to rays and faces */
  Mat3 rotation = mat3_rotation(box->spin_x, box->spin_y);

  /* the camera sits back along -Z and looks toward +Z; the cube spins, not it */
  V3 camera_origin = {0.f, 0.f, -cam_dist};
  V3 view_forward = {0.f, 0.f, 1.f};
  V3 view_right = {1.f, 0.f, 0.f};
  V3 view_up = {0.f, 1.f, 0.f};

  float screen_centre_x = cols * 0.5f;
  float screen_centre_y = rows * 0.5f;

  /* the camera in the cube's frame — same for every cell, so compute it once */
  V3 ray_origin_obj = mat3_mulT(rotation, camera_origin);

  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      V3 ray_dir_world =
          primary_ray_dir(col, row, screen_centre_x, screen_centre_y,
                          fov_half_tan, view_right, view_up, view_forward);
      /* the same ray, rotated into the cube's frame for the hit test */
      Ray ray_obj = {ray_origin_obj, mat3_mulT(rotation, ray_dir_world)};

      BoxHit hit = ray_aabb(ray_obj, box->half_extent);
      if (!hit.hit)
        continue;

      /* where the hit is (the distance is the same in either frame), plus which
       * way the face points and the direction back to the camera */
      V3 hit_point_obj =
          v3add(ray_obj.origin, v3scale(hit.t_hit, ray_obj.dir));
      V3 hit_point_world =
          v3add(camera_origin, v3scale(hit.t_hit, ray_dir_world));
      V3 normal_world = mat3_mul(rotation, hit.normal_obj);
      V3 view_dir = v3norm(v3sub(camera_origin, hit_point_world));

      /* colour it; the wireframe view returns 0 to skip a face's middle */
      V3 colour;
      float luminance;
      if (!shade_surface(shade_mode, &hit, hit_point_obj, hit_point_world,
                         normal_world, view_dir, cam_dist, box->half_extent, material,
                         &colour, &luminance))
        continue;

      if (debug_mode != DEBUG_OFF)
        apply_debug(debug_mode, &hit, hit_point_obj, box->half_extent, &colour,
                    &luminance);

      draw_color(row, col, colour, luminance);
    }
  }
}

/* ── §9  status line + key hints ── */

static void hud_draw(int cols, int rows, float fps, const Material *material,
                     ShadeMode shade_mode, DebugMode debug_mode, float cam_dist,
                     int paused) {
  /* right side: fps, distance, material, paused/running. fps goes on the right
   * so a long mode name on the left can't shove it off a narrow screen. */
  char status[96];
  snprintf(status, sizeof status, " %5.1f fps  dist:%.1f  %-9s  %s ",
           (double)fps, (double)cam_dist, material->name,
           paused ? "PAUSED " : "running");
  int status_len = (int)strlen(status);
  if (status_len > cols)
    status_len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - status_len, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* left side: which view, and which debug overlay */
  char left_label[80];
  snprintf(left_label, sizeof left_label, " mode:%-9s  debug:%-8s ",
           k_mode_names[shade_mode], k_debug_names[debug_mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", left_label);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* bottom: the key hints, bold so they stay readable over the animation */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(
      rows - 1, 0,
      " q:quit  spc/p:pause  s:mode  d:debug  t:material  r:reset  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9.5  all the live state in one place ── */

/* Everything that changes while the program runs, gathered so the main loop
 * reads like a summary. Other functions take only the piece they need (a const
 * Box*, a const Material*, a number); just init/reset/advance take the whole
 * Scene. */
typedef struct {
  Box box; /* the spinning cube */

  /* view settings — these change only the picture, never the cube; all driven
   * by keys in the main loop */
  int material_idx;     /* which material (wraps around MATERIAL_N) */
  ShadeMode shade_mode; /* which of the four views */
  DebugMode debug_mode; /* which debug overlay, if any */
  float cam_dist;       /* how far back the camera sits */
  int paused;           /* when set, the cube stops spinning */

  int cols, rows; /* terminal size, re-read when the window changes */

  /* frame-rate readout for the HUD, averaged over ~0.5s so it doesn't flicker */
  float fps;
  long long fps_accum_ns;
  int fps_frames;
} Scene;

/* Starting state: cube at zero spin, default view, current window size. */
static void scene_init(Scene *s) {
  s->box.half_extent = CUBE_S;
  s->box.spin_x = 0.f;
  s->box.spin_y = 0.f;
  s->material_idx = 0;
  s->shade_mode = MODE_PHONG;
  s->debug_mode = DEBUG_OFF;
  s->cam_dist = CAM_DIST_DEF;
  s->paused = 0;
  getmaxyx(stdscr, s->rows, s->cols);
  s->fps = 0.f;
  s->fps_accum_ns = 0;
  s->fps_frames = 0;
}

/* The `r` key: stop the spin (back to facing straight on). View settings stay. */
static void scene_reset(Scene *s) {
  s->box.spin_x = 0.f;
  s->box.spin_y = 0.f;
}

/* One step forward in time: spin the cube a little (unless paused) and update the
 * frame-rate counter. The only place the spin changes during a frame. */
static void scene_advance(Scene *s, long long dt_ns) {
  if (!s->paused) {
    float seconds = (float)dt_ns * 1e-9f;
    s->box.spin_y += ROT_Y * seconds;
    s->box.spin_x += ROT_X * seconds;
  }
  s->fps_accum_ns += dt_ns;
  s->fps_frames++;
  if (s->fps_accum_ns >= 500000000LL) {
    s->fps = (float)s->fps_frames * 1e9f / (float)s->fps_accum_ns;
    s->fps_accum_ns = 0;
    s->fps_frames = 0;
  }
}

/* ── §10  startup, input, main loop ── */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_run = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1); /* stop ncurses pausing our drawing to peek at the keyboard,
                  * which otherwise tears the picture */
  atexit(cleanup);
  color_init();

  Scene scene;
  scene_init(&scene);

  long long frame_ns = 1000000000LL / TARGET_FPS;
  long long last = clock_ns();

  while (g_run) {
    /* window was resized: reset ncurses and re-read the new size */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, scene.rows, scene.cols);
    }

    /* time since the last frame, capped so one hiccup can't make the cube jump */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* move the cube on by that much time (does nothing while paused) */
    scene_advance(&scene, dt);

    /* draw the frame: clear, cube, HUD, then flip it to the screen */
    const Material *material = &g_materials[scene.material_idx % MATERIAL_N];
    long long frame_start = clock_ns();
    erase();
    render(&scene.box, material, scene.shade_mode, scene.debug_mode, scene.cam_dist,
           scene.cols, scene.rows);
    hud_draw(scene.cols, scene.rows, scene.fps, material, scene.shade_mode,
             scene.debug_mode, scene.cam_dist, scene.paused);
    wnoutrefresh(stdscr);
    doupdate();

    /* handle one keypress — these change the view, never the cube's motion */
    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27 /* ESC */:
      g_run = 0;
      break;
    case ' ':
    case 'p':
    case 'P':
      scene.paused = !scene.paused;
      break;
    case 'r':
    case 'R':
      scene_reset(&scene);
      break;
    case 's':
    case 'S':
      scene.shade_mode = (ShadeMode)((scene.shade_mode + 1) % MODE_N);
      break;
    case 'd':
    case 'D':
      scene.debug_mode = (DebugMode)((scene.debug_mode + 1) % DEBUG_N);
      break;
    case 't':
      scene.material_idx = (scene.material_idx + 1) % MATERIAL_N;
      break;
    case 'T':
      scene.material_idx = (scene.material_idx + MATERIAL_N - 1) % MATERIAL_N;
      break;
    case '+':
    case '=':
      scene.cam_dist -= CAM_DIST_STEP;
      if (scene.cam_dist < CAM_DIST_MIN)
        scene.cam_dist = CAM_DIST_MIN;
      break;
    case '-':
    case '_':
      scene.cam_dist += CAM_DIST_STEP;
      if (scene.cam_dist > CAM_DIST_MAX)
        scene.cam_dist = CAM_DIST_MAX;
      break;
    default:
      break;
    }

    /* sleep whatever's left of the frame so we hold roughly 60 fps */
    clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
  }
  return 0;
}
