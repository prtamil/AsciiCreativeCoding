/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * csg_atlas.c — a browsable catalogue of the ways two 3-D shapes can be
 * merged.  On screen: a sphere and a box (the box drifts in and out).
 * You pick one of 13 "combine" rules and watch how the two shapes join.
 *
 * The picture is drawn by ray marching: for each character cell we shoot a
 * ray into the scene and creep along it until it touches a surface, then
 * shade whatever we hit.  The 13 rules fall into four families that differ
 * only in how they treat the SEAM where the shapes meet — a sharp crease
 * (HARD), a soft bulge (SMOOTH), a rounded fillet (ROUND), or a flat 45°
 * cut (CHAMFER).  The bottom HUD line lists the keys at runtime.
 *
 * Sister file raymarcher/sdf_gallery.c uses these same combine rules to
 * build whole scenes; this file instead holds the shapes fixed and sweeps
 * every rule.
 *
 * Where the math comes from:
 *   - creep-along-the-ray rendering: Hart, "Sphere Tracing" (1996)
 *   - the combine rules + soft-minimum: Iñigo Quílez,
 *     https://iquilezles.org/articles/distfunctions/ and .../smin/
 *   - the lighting model: Phong (1975)
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * How the file is laid out — each job kept in its own area so it's easy to
 * follow and hard to break:
 *   §3-§17  the math (and orbit_to_camera in §18): pure helpers that take
 *           inputs and return answers — the shapes, the 13 combine rules,
 *           the ray march, lighting, and the brightness-to-character maps.
 *           They never touch shared state or the screen.
 *   §18     the only thing that moves by itself: scene_tick nudges the
 *           clock and the camera's orbit angle once per frame.
 *   §19     drawing: turns the current state into characters; only reads.
 *   §20     input: keys, resize, quit — these change state, but between
 *           frames, never in the middle of a tick.
 *   §2,§21  timekeeping and the "sleep a little to hold a steady frame
 *           rate" bookkeeping.
 * (There's no glow/trail state to store and no scripted pauses, so two of
 *  the usual layers simply don't exist here.)
 *
 * One frame in order: apply a pending resize, measure how long the last
 * frame took, advance the sim, refresh fps and sleep to pace, draw, then
 * read one keypress.  Scene is the single bundle of state; only scene_tick
 * and the key/resize handlers ever change it.
 */

/* §1  config — every tunable number, named in one place */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,

  FPS_UPDATE_MS = 500, /* how often to refresh the fps readout (ms) */
  HUD_ROWS = 2,        /* top + bottom rows are reserved for the HUD */

  LUMI_N = 8,
  PAIR_HUD = LUMI_N + 1,
  PAIR_HINT = LUMI_N + 2,

  OP_COUNT = 13,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* If a frame takes longer than this (a resize, a stall), pretend it was
 * only this long — stops one slow frame from making the box lurch. */
#define MAX_FRAME_NS (100 * NS_PER_MS)

/* Terminal cells are about twice as tall as wide; squash vertically to
 * match so the sphere looks round instead of stretched. */
#define CELL_ASPECT 2.0f

/* §1.1 ray-march tuning.
 *   MARCH_MAX    give up after this many steps along one ray
 *   MARCH_EPS    how close counts as "touched the surface"
 *   MARCH_FAR    if a ray gets this far out, it hit nothing — a miss
 *   NORMAL_EPS   tiny nudge used to work out which way a surface faces
 *   MARCH_RELAX  step a little shorter than the safe distance (×0.85) so
 *                the soft/rounded seams aren't overshot and left speckly */
#define MARCH_MAX 90
#define MARCH_EPS 0.002f
#define MARCH_FAR 20.0f
#define NORMAL_EPS 0.002f
#define MARCH_RELAX 0.85f

/* §1.2 camera + zoom. */
#define CAM_DIST_DEFAULT 3.5f
#define CAM_DIST_MIN 2.0f
#define CAM_DIST_MAX 7.0f
#define CAM_ZOOM_STEP 0.30f
#define CAM_ELEVATION 0.55f  /* radians above horizon                */
#define CAM_ORBIT_RATE 0.30f /* radians / sec                        */
#define FOV_DEG 50.0f

/* §1.3 the box slides back and forth through the sphere so you can watch
 * the seam form and break. */
#define BOX_SWING_AMP 1.05f  /* how far the box slides each way          */
#define BOX_SWING_RATE 0.55f /* sliding speed (radians/sec of the cycle) */
#define SPHERE_RADIUS 1.00f
#define BOX_HALF_EXTENT 0.70f
#define BOX_CORNER_R 0.05f /* slightly rounded box corners */

/* §1.4 the single "softness" knob the user turns with +/-.  Bigger = a
 * more rounded, blended seam.  SMOOTH uses it as-is; ROUND and CHAMFER
 * halve it.  Held between PARAM_MIN and PARAM_MAX. */
#define PARAM_DEFAULT 0.30f
#define PARAM_MIN 0.05f
#define PARAM_MAX 1.00f
#define PARAM_STEP 0.05f

/* §1.5 lighting knobs, tuned for a low-res terminal (only ~8 brightness
 * steps).  A tight bright highlight would flicker across single cells, and
 * plain lighting would leave the shadowed half a flat dark blob — so the
 * highlight is broad and dim and the main shading is "wrapped" (see §15).
 *   KA    base brightness everywhere, so nothing is pure black
 *   KD    strength of the main shading
 *   KS    strength of the shiny highlight (small on purpose)
 *   SHIN  highlight tightness — low means a soft, spread-out highlight */
#define KA 0.10f
#define KD 0.80f
#define KS 0.25f
#define SHIN 14.0f
#define LIGHT_X 3.0f
#define LIGHT_Y 4.5f
#define LIGHT_Z 2.5f

/* §1.6 the characters used for shading, darkest (space) to brightest (@). */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1))

/* §1.7 colour themes.  Each is a name plus eight colour codes running
 * dark→bright; shading picks one by brightness.  t/T cycles them. */
typedef struct {
  const char *name;
  short ramp[LUMI_N]; /* 8 xterm-256 colour codes, dark to bright */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    {"CLASSIC ", {235, 238, 241, 244, 247, 250, 253, 255}},
    {"AMBER   ", {130, 136, 166, 172, 178, 208, 214, 220}},
    {"MATRIX  ", {28, 34, 40, 46, 82, 118, 154, 190}},
    {"NEON    ", {53, 91, 129, 165, 201, 207, 213, 227}},
    {"ICE     ", {25, 31, 38, 45, 51, 87, 123, 159}},
    {"COPPER  ", {94, 130, 136, 166, 172, 208, 214, 220}},
};

/* §1.8 the 13 combine rules, listed family by family.  The grouping is
 * load-bearing: §11 walks within a family by relying on its entries being
 * adjacent here.  HARD has a fourth rule (XOR); the rest have three. */
typedef enum {
  OP_HARD_UNION = 0,
  OP_HARD_INTERSECT,
  OP_HARD_SUBTRACT,
  OP_HARD_XOR,

  OP_SMOOTH_UNION,
  OP_SMOOTH_INTERSECT,
  OP_SMOOTH_SUBTRACT,

  OP_ROUND_UNION,
  OP_ROUND_INTERSECT,
  OP_ROUND_SUBTRACT,

  OP_CHAMFER_UNION,
  OP_CHAMFER_INTERSECT,
  OP_CHAMFER_SUBTRACT,
} OpIndex;

/* §1.9 alternate views, cycled with d / D — handy for seeing what the
 * renderer is actually computing. */
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit picture            */
  DEBUG_NORMALS = 1, /* colour shows which way each surface faces */
  DEBUG_DEPTH = 2,   /* brighter = closer to the camera          */
  DEBUG_STEPS = 3,   /* brighter = the ray took more steps to land */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* In the DEPTH view, surfaces within this many world-units of the camera
 * distance span the full dark→bright range. */
#define DEBUG_DEPTH_RANGE 1.5f

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
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* The math layer (§3-§17, plus orbit_to_camera in §18): every function
 * here just takes its inputs and returns an answer — nothing shared and no
 * screen is touched, so the drawing code can never corrupt these results. */

/* §3  pick the ncurses colour to print for a brightness level (0..7).
 * On 8-colour terminals there aren't enough real colours, so we fake the
 * dark and bright ends with the dim/bold attributes. */
static attr_t lumi_attr(int slot) {
  if (slot < 0)
    slot = 0;
  if (slot >= LUMI_N)
    slot = LUMI_N - 1;
  attr_t a = COLOR_PAIR(slot + 1);
  if (COLORS < 256) {
    if (slot < 3)
      a |= A_DIM;
    else if (slot >= 6)
      a |= A_BOLD;
  }
  return a;
}

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
  return (L > 1e-7f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

/* abs / "clamp negatives to zero" on each component — the box shape uses
 * both to fold its eight corners into one. */
static inline Vec3 v3abs(Vec3 a) {
  return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}
static inline Vec3 v3max0(Vec3 a) {
  return v3(fmaxf(a.x, 0.0f), fmaxf(a.y, 0.0f), fmaxf(a.z, 0.0f));
}

/* Cross product a × b — vector perpendicular to both (right-hand rule). */
static inline Vec3 v3cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}

/* Clamp a scalar to the unit interval — used wherever a raw signal is
 * mapped to the [0,1] brightness the glyph / colour ramps expect. */
static inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/* §5  how far point p is from the sphere's surface: negative inside, zero
 * on it, positive outside.  This "distance to the nearest surface" is the
 * one thing the ray march needs from a shape. */
static float sd_sphere(Vec3 p, float r) { return v3len(p) - r; }

/* §6  same idea for a box with slightly rounded corners.  Fold the point
 * into one corner with abs(), measure how far outside the box it sits,
 * then pull the surface out by r to round the edges.  (Exact box-distance
 * trick from Quílez; b is the box's half-size in each direction.) */
static float sd_round_box(Vec3 p, Vec3 b, float r) {
  Vec3 q = v3sub(v3abs(p), b);
  Vec3 q_pos = v3max0(q);
  float outside = v3len(q_pos);
  float inside = fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.0f);
  return outside + inside - r;
}

/* §7  HARD combines — the plain set operations, giving a crisp crease
 * where the shapes meet.  Each takes the two surface-distances a (sphere)
 * and b (box) and returns the distance for the combined shape. */

/* union — everything in either shape */
static inline float op_hard_union(float a, float b) { return fminf(a, b); }

/* intersection — only where the two shapes overlap */
static inline float op_hard_intersect(float a, float b) { return fmaxf(a, b); }

/* subtract — the sphere with the box carved out of it */
static inline float op_hard_subtract(float a, float b) { return fmaxf(a, -b); }

/* xor — in one shape or the other, but not in the overlap */
static inline float op_hard_xor(float a, float b) {
  return fmaxf(fminf(a, b), -fmaxf(a, b));
}

/* §8  SMOOTH combines — instead of a crease, melt the shapes together with
 * a soft bulge.  k is how wide the blend is: more than k from the seam
 * these match the HARD versions, and within k the surface eases across.
 * (Quílez's polynomial smooth-minimum.)  A near-zero k falls back to HARD
 * so we never divide by zero. */

static inline float op_smooth_union(float a, float b, float k) {
  if (k < 1e-6f)
    return fminf(a, b);
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

static inline float op_smooth_intersect(float a, float b, float k) {
  if (k < 1e-6f)
    return fmaxf(a, b);
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fmaxf(a, b) + h * h * k * 0.25f;
}

static inline float op_smooth_subtract(float a, float b, float k) {
  if (k < 1e-6f)
    return fmaxf(a, -b);
  float h = fmaxf(k - fabsf(a + b), 0.0f) / k;
  return fmaxf(a, -b) + h * h * k * 0.25f;
}

/* §9  ROUND combines — blend with a true circular fillet (a quarter-round,
 * like a rounded inside corner) of radius r, crisper than SMOOTH's bulge.
 * Beyond radius r they match the HARD versions. */

static inline float op_round_union(float a, float b, float r) {
  float ux = fmaxf(r - a, 0.0f);
  float uy = fmaxf(r - b, 0.0f);
  return fmaxf(r, fminf(a, b)) - sqrtf(ux * ux + uy * uy);
}

static inline float op_round_intersect(float a, float b, float r) {
  float ux = fmaxf(r + a, 0.0f);
  float uy = fmaxf(r + b, 0.0f);
  return fminf(-r, fmaxf(a, b)) + sqrtf(ux * ux + uy * uy);
}

static inline float op_round_subtract(float a, float b, float r) {
  float ux = fmaxf(r + a, 0.0f);
  float uy = fmaxf(r - b, 0.0f);
  return fminf(-r, fmaxf(a, -b)) + sqrtf(ux * ux + uy * uy);
}

/* §10  CHAMFER combines — replace the seam with a flat 45° cut of width r,
 * like a bevelled edge.  The extra term is the distance to that angled
 * cut; near the seam it wins, elsewhere the shapes act like HARD. */

#define INV_SQRT2 0.70710678f /* 1/sqrt(2), the slope of the 45° cut */

static inline float op_chamfer_union(float a, float b, float r) {
  return fminf(fminf(a, b), (a - r + b) * INV_SQRT2);
}

static inline float op_chamfer_intersect(float a, float b, float r) {
  return fmaxf(fmaxf(a, b), (a + r + b) * INV_SQRT2);
}

static inline float op_chamfer_subtract(float a, float b, float r) {
  return fmaxf(fmaxf(a, -b), (a + r - b) * INV_SQRT2);
}

/* §11  the catalogue as a table, plus helpers to move around it.  Picture
 * a grid: the four families down the side, the union/intersect/subtract/xor
 * kinds across.  family_of and op_in_family find a rule's row and column,
 * so the number keys (family) and n/N (kind) can navigate it predictably. */

/* One row of the catalogue: the labels shown in the HUD, and whether the
 * softness knob does anything for this rule. */
typedef struct {
  const char *op_kind; /* UNION / INTERSECT / SUBTRACT / XOR */
  const char *family;  /* HARD / SMOOTH / ROUND / CHAMFER    */
  bool uses_param;
} OpInfo;

static const OpInfo OP_TABLE[OP_COUNT] = {
    /* HARD family — 4 ops (the only one with XOR) */
    {"UNION    ", "HARD   ", false},
    {"INTERSECT", "HARD   ", false},
    {"SUBTRACT ", "HARD   ", false},
    {"XOR      ", "HARD   ", false},

    /* SMOOTH family — 3 ops */
    {"UNION    ", "SMOOTH ", true},
    {"INTERSECT", "SMOOTH ", true},
    {"SUBTRACT ", "SMOOTH ", true},

    /* ROUND family — 3 ops */
    {"UNION    ", "ROUND  ", true},
    {"INTERSECT", "ROUND  ", true},
    {"SUBTRACT ", "ROUND  ", true},

    /* CHAMFER family — 3 ops */
    {"UNION    ", "CHAMFER", true},
    {"INTERSECT", "CHAMFER", true},
    {"SUBTRACT ", "CHAMFER", true},
};

#define FAMILY_COUNT 4
static const int FAMILY_FIRST[FAMILY_COUNT] = {
    OP_HARD_UNION,    /* 1 — HARD,    4 ops (incl. XOR) */
    OP_SMOOTH_UNION,  /* 2 — SMOOTH,  3 ops */
    OP_ROUND_UNION,   /* 3 — ROUND,   3 ops */
    OP_CHAMFER_UNION, /* 4 — CHAMFER, 3 ops */
};
static const int FAMILY_SIZE[FAMILY_COUNT] = {4, 3, 3, 3};

/* which family (row) a rule belongs to, 0..3 */
static int family_of(int op_idx) {
  for (int f = FAMILY_COUNT - 1; f >= 0; f--)
    if (op_idx >= FAMILY_FIRST[f])
      return f;
  return 0;
}

/* a rule's position within its family (column) */
static int op_in_family(int op_idx) {
  return op_idx - FAMILY_FIRST[family_of(op_idx)];
}

/* Which combine rule is on screen right now, bundled with its softness
 * knob so the whole render pipeline can pass it around as one value.
 *   index   which of the 13 rules (an OpIndex)
 *   blend   the softness setting — used as-is by SMOOTH, halved for
 *           ROUND/CHAMFER, and ignored by HARD (which has no soft seam) */
typedef struct {
  int index;
  float blend;
} CsgOperator;

/* §12  the whole scene as one distance function: combine the sphere and
 * the (currently sliding) box with the chosen rule, and return how far the
 * given point is from the result.  The ray march only ever calls this, so
 * it never needs to know which rule is active. */

static float scene_sdf(Vec3 p, float time, CsgOperator op) {
  float a = sd_sphere(p, SPHERE_RADIUS); /* distance to the sphere */

  /* distance to the box at its current slid-over position */
  float swing = BOX_SWING_AMP * sinf(time * BOX_SWING_RATE);
  Vec3 box_pos = v3(swing, 0.0f, 0.0f);
  Vec3 bp = v3sub(p, box_pos);
  Vec3 b_ext = v3(BOX_HALF_EXTENT, BOX_HALF_EXTENT, BOX_HALF_EXTENT);
  float b = sd_round_box(bp, b_ext, BOX_CORNER_R);

  float r = op.blend * 0.5f; /* ROUND/CHAMFER use a halved knob */

  switch (op.index) {
  case OP_HARD_UNION:
    return op_hard_union(a, b);
  case OP_HARD_INTERSECT:
    return op_hard_intersect(a, b);
  case OP_HARD_SUBTRACT:
    return op_hard_subtract(a, b);
  case OP_HARD_XOR:
    return op_hard_xor(a, b);

  case OP_SMOOTH_UNION:
    return op_smooth_union(a, b, op.blend);
  case OP_SMOOTH_INTERSECT:
    return op_smooth_intersect(a, b, op.blend);
  case OP_SMOOTH_SUBTRACT:
    return op_smooth_subtract(a, b, op.blend);

  case OP_ROUND_UNION:
    return op_round_union(a, b, r);
  case OP_ROUND_INTERSECT:
    return op_round_intersect(a, b, r);
  case OP_ROUND_SUBTRACT:
    return op_round_subtract(a, b, r);

  case OP_CHAMFER_UNION:
    return op_chamfer_union(a, b, r);
  case OP_CHAMFER_INTERSECT:
    return op_chamfer_intersect(a, b, r);
  case OP_CHAMFER_SUBTRACT:
    return op_chamfer_subtract(a, b, r);
  }
  return op_hard_union(a, b); /* can't happen; keeps the compiler happy */
}

/* §13  ray marching: follow a ray out from ro, and at each step jump
 * forward by the distance to the nearest surface (which is safe — nothing
 * is closer).  When that distance shrinks to ~zero we've touched something;
 * if the ray wanders too far we've missed.  Returns how far along the ray
 * the hit is (or -1 for a miss), and reports the step count for the STEPS
 * overlay.  See Hart, "Sphere Tracing" (1996). */
static float sphere_trace(Vec3 ro, Vec3 rd, float time, CsgOperator op,
                          int *out_steps) {
  float t = 0.0f;
  int step;
  for (step = 0; step < MARCH_MAX; step++) {
    Vec3 p = v3add(ro, v3mul(rd, t));
    float d = scene_sdf(p, time, op);
    if (d < MARCH_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > MARCH_FAR)
      break;
    t += d * MARCH_RELAX;
  }
  if (out_steps)
    *out_steps = step;
  return -1.0f;
}

/* §14  which way the surface faces at point p (its "normal"), needed for
 * lighting.  We can't read it directly, so we sample the distance at four
 * points nudged around p and see which direction it grows fastest — that's
 * "uphill", away from the surface.  Four samples (a tetrahedron) stays
 * crisp at sharp seams; NORMAL_EPS sets how far we nudge. */
static Vec3 sdf_normal(Vec3 p, float time, CsgOperator op) {
  const float e = NORMAL_EPS;
  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = scene_sdf(v3add(p, k0), time, op);
  float d1 = scene_sdf(v3add(p, k1), time, op);
  float d2 = scene_sdf(v3add(p, k2), time, op);
  float d3 = scene_sdf(v3add(p, k3), time, op);

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/* §15  work out how bright a hit point should be (0..1): a base glow, plus
 * how squarely the surface faces the light, plus a small shiny highlight.
 *
 * The twist for a low-res terminal: normally a surface facing away from the
 * light goes flat black, losing all its shape.  Instead we "wrap" the light
 * around — even the far side keeps a gentle gradient, so its curvature
 * still reads.  (Half-Lambert, Valve/Mitchell 2006.)  The highlight only
 * appears on the lit side and is kept soft so it doesn't flicker. */
static float phong_shade(Vec3 N, Vec3 hit, Vec3 cam, Vec3 light) {
  Vec3 L_dir = v3norm(v3sub(light, hit)); /* toward the light */
  Vec3 V_dir = v3norm(v3sub(cam, hit));   /* toward the camera */

  /* how much the surface faces the light, wrapped from [-1,1] to [0,1] */
  float ndl = v3dot(N, L_dir);
  float wrap = ndl * 0.5f + 0.5f;
  float diff = wrap * wrap;

  /* shiny highlight, only where the surface actually faces the light */
  float spec = 0.0f;
  if (ndl > 0.0f) {
    Vec3 R_dir = v3sub(v3mul(N, 2.0f * ndl), L_dir);
    spec = powf(fmaxf(0.0f, v3dot(R_dir, V_dir)), SHIN);
  }

  return clamp01(KA + KD * diff + KS * spec);
}

/* The camera in two parts.  Orbit is what the user steers — where the eye
 * sits, like longitude-and-zoom on a globe (the up/down angle is fixed).
 * orbit_to_camera turns that into a Camera: the eye point plus the three
 * "which way is forward / right / up" directions the renderer fires rays
 * along. */
typedef struct {
  float distance; /* eye's distance from the centre (zoom) */
  float azimuth;  /* angle around the scene, in radians */
} Orbit;

typedef struct {
  Vec3 origin, fwd, right, up; /* eye point + the three view directions */
  float fov_t;                 /* sets how wide the view is */
} Camera;

/* §16  shoot one ray for one screen cell and report what it found.
 *
 * Hit bundles everything the four views might want, so the ray is only
 * traced once:
 *   hit        did the ray reach a surface at all?
 *   intensity  shaded brightness 0..1 (the normal picture)
 *   N          which way that surface faces (NORMALS view)
 *   hit_t      how far along the ray the surface is (DEPTH view)
 *   steps      how many march steps it took (STEPS view) */
typedef struct {
  bool hit;
  float intensity;
  Vec3 N;
  float hit_t;
  int steps;
} Hit;

/* Direction of the ray for one screen cell: turn the cell's spot into a
 * -1..+1 position on screen (row 0 is the top), undo the tall-cell stretch,
 * and aim that far off straight-ahead. */
static Vec3 primary_ray(int col, int row, int cw, int ch, Camera cam) {
  float ndc_x = ((float)col + 0.5f) / (float)cw * 2.0f - 1.0f;
  float ndc_y = -((float)row + 0.5f) / (float)ch * 2.0f + 1.0f;
  float cell_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

  Vec3 tilt = v3add(v3mul(cam.right, ndc_x * cam.fov_t),
                    v3mul(cam.up, ndc_y * cam.fov_t * cell_aspect));
  return v3norm(v3add(cam.fwd, tilt));
}

static Hit cast_ray(int col, int row, int cw, int ch, Camera cam, Vec3 light,
                    float time, CsgOperator op) {
  Hit h = {false, 0.0f, {0, 0, 1}, 0.0f, 0};

  Vec3 rd = primary_ray(col, row, cw, ch, cam);

  int steps = 0;
  float t = sphere_trace(cam.origin, rd, time, op, &steps);
  h.steps = steps;
  if (t < 0.0f)
    return h;

  h.hit = true;
  h.hit_t = t;
  Vec3 hit_p = v3add(cam.origin, v3mul(rd, t));
  h.N = sdf_normal(hit_p, time, op);
  h.intensity = phong_shade(h.N, hit_p, cam.origin, light);
  return h;
}

/* §17  turn a brightness (0..1) into a character + colour to print. */

static char intensity_to_glyph(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= RAMP_LEN)
    idx = RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static attr_t intensity_to_attr(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  int slot = (idx * LUMI_N) / RAMP_LEN;
  return lumi_attr(slot);
}

/* The alternate views: each turns one piece of a Hit into a 0..1 value,
 * fed through the same character/colour ramp as the normal picture. */

/* which compass direction the surface faces, as 0..1 → colour */
static float normal_hue(Vec3 N) {
  return atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
}

/* how much the surface faces upward, as 0..1 → brightness */
static float normal_upness(Vec3 N) { return clamp01(N.y * 0.5f + 0.5f); }

/* closer to the camera = brighter, within a fixed near/far window */
static float depth_to_brightness(float hit_t, float cam_dist) {
  float t_near = cam_dist - DEBUG_DEPTH_RANGE;
  float t_far = cam_dist + DEBUG_DEPTH_RANGE;
  if (t_near < 0.0f)
    t_near = 0.0f;
  return clamp01((t_far - hit_t) / (t_far - t_near));
}

/* more march steps = brighter; edges seen at a glancing angle take the
 * most steps, so they light up */
static float steps_to_brightness(int steps) {
  return clamp01((float)steps / (float)MARCH_MAX);
}

/* §18  the state, and the one function that moves it on its own.  Scene
 * below holds everything the program is showing; only scene_tick and the
 * key/resize handlers ever change it. */

/* Everything currently on screen, gathered in one place:
 *   op           which combine rule, plus its softness knob
 *   view         where the orbiting camera is (angle + zoom)
 *   theme_index  which colour theme  (t/T)
 *   debug_mode   which view          (d/D)
 *   paused       is the animation stopped?  (space)
 *   time         animation clock — drives the box slide and the orbit
 *   cols, rows   current terminal size */
typedef struct {
  CsgOperator op;
  Orbit view;
  int theme_index;
  DebugMode debug_mode;
  bool paused;
  float time;
  int cols, rows;
} Scene;

/* Set up, reset, and resize the state — called at startup and from the
 * key/resize handling, never from the per-frame tick. */
static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->cols = cols;
  s->rows = rows;
  s->time = 0.0f;
  s->op.index = OP_HARD_UNION;
  s->op.blend = PARAM_DEFAULT;
  s->theme_index = 0;
  s->view.distance = CAM_DIST_DEFAULT;
  s->view.azimuth = 0.5f;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset(Scene *s) {
  s->time = 0.0f;
  s->op.index = OP_HARD_UNION;
  s->op.blend = PARAM_DEFAULT;
  s->view.distance = CAM_DIST_DEFAULT;
  s->view.azimuth = 0.5f;
  s->debug_mode = DEBUG_NORMAL;
}

/* The only thing that moves on its own: nudge the clock and the camera's
 * orbit angle forward by dt.  Pausing freezes both. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
  s->view.azimuth += dt * CAM_ORBIT_RATE;
  if (s->view.azimuth > (float)(2.0 * M_PI))
    s->view.azimuth -= (float)(2.0 * M_PI);
}

/* Build the camera (eye point + view directions, looking at the centre)
 * from wherever the user has orbited to. */
static Camera orbit_to_camera(Orbit orbit) {
  float yaw = orbit.azimuth;
  float ele = CAM_ELEVATION;

  Camera c;
  /* put the eye on its orbit, then aim it back at the centre */
  c.origin = v3(orbit.distance * cosf(ele) * cosf(yaw),
                orbit.distance * sinf(ele),
                orbit.distance * cosf(ele) * sinf(yaw));
  c.fwd = v3norm(v3sub(v3(0, 0, 0), c.origin));

  /* work out right and up from the forward direction */
  Vec3 world_up = v3(0, 1, 0);
  c.right = v3norm(v3cross(c.fwd, world_up));
  c.up = v3cross(c.right, c.fwd);

  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  return c;
}

/* §19  drawing: set up colours, start and stop ncurses, paint the scene,
 * and show the two HUD rows.  Nothing here changes the state — it only
 * reads it and talks to the terminal. */

/* Load a theme's eight brightness colours (plain white as a fallback on
 * terminals with fewer than 256 colours). */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= THEME_COUNT)
    idx = 0;
  const Theme *t = &THEMES[idx];
  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), t->ramp[i], COLOR_BLACK);
  } else {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), COLOR_WHITE, COLOR_BLACK);
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

/* The endwin + refresh dance makes ncurses notice the new terminal size
 * after a resize; then we read it back. */
static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Print one character.  Changing colour in ncurses isn't free, so we only
 * switch when this cell's colour differs from the last one we printed. */
static void paint_cell(int ty, int tx, char glyph, attr_t attr,
                       int *last_pair_attr) {
  int pa = (int)attr;
  if (pa != *last_pair_attr) {
    if (*last_pair_attr != -1)
      attroff((attr_t)*last_pair_attr);
    attron(attr);
    *last_pair_attr = pa;
  }
  mvaddch(ty, tx, (chtype)(unsigned char)glyph);
}

/* Draw the scene: one ray per cell, filling every row except the two kept
 * for the HUD.  The current view (d/D) decides what each cell shows. */
static void scene_render(const Scene *s) {
  int rows_eff = s->rows - HUD_ROWS;
  int y_offset = 1;
  if (rows_eff < 1)
    return;

  Camera cam = orbit_to_camera(s->view);
  Vec3 light = v3(LIGHT_X, LIGHT_Y, LIGHT_Z);

  int last_pair_attr = -1;

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      Hit h = cast_ray(col, row, s->cols, rows_eff, cam, light, s->time, s->op);
      if (!h.hit) {
        if (last_pair_attr != -1) {
          attroff((attr_t)last_pair_attr);
          last_pair_attr = -1;
        }
        continue;
      }

      /* Each overlay maps a different Hit signal to a [0,1] value. */
      float v;
      switch (s->debug_mode) {
      case DEBUG_NORMALS:
        /* two signals: colour from surface azimuth, glyph from upness */
        paint_cell(row + y_offset, col, intensity_to_glyph(normal_upness(h.N)),
                   intensity_to_attr(normal_hue(h.N)), &last_pair_attr);
        continue;
      case DEBUG_DEPTH:
        v = depth_to_brightness(h.hit_t, s->view.distance);
        break;
      case DEBUG_STEPS:
        v = steps_to_brightness(h.steps);
        break;
      case DEBUG_NORMAL:
      default:
        v = h.intensity; /* the normal picture */
        break;
      }

      char glyph = intensity_to_glyph(v);
      attr_t attr = intensity_to_attr(v);
      paint_cell(row + y_offset, col, glyph, attr, &last_pair_attr);
    }
  }
  if (last_pair_attr != -1)
    attroff((attr_t)last_pair_attr);
}

/* Draw the two HUD rows: status across the top, key reminders across the
 * bottom.  The fps sits in the top-left label so it stays on screen even
 * when the terminal is too narrow to fit the rest of the status line. */
static void hud_draw(const Screen *sc, const Scene *s, double fps) {
  /* top row: title + fps on the left, status on the right (trimmed to fit) */
  char left[48];
  snprintf(left, sizeof left, " CSG ATLAS  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  const OpInfo *info = &OP_TABLE[s->op.index];
  char param_field[16];
  if (info->uses_param)
    snprintf(param_field, sizeof param_field, "%4.2f", (double)s->op.blend);
  else
    snprintf(param_field, sizeof param_field, "—");

  char status[200];
  snprintf(status, sizeof status,
           " family:%s  op:%s  param:%s  debug:%s  theme:%s  zoom:%4.2f  %s ",
           info->family, info->op_kind, param_field,
           DEBUG_MODE_NAMES[s->debug_mode], THEMES[s->theme_index].name,
           (double)s->view.distance, s->paused ? "PAUSED" : "running");
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

  /* bottom row: the key reminders */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  1-4:family(row)  n/N:op(col)  "
           "+/-:param  d/D:debug  t/T:theme  z/Z:zoom  r:reset ");
  clrtoeol();
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps) {
  erase();
  scene_render(s);
  hud_draw(sc, s, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §20  input: keypresses, resize, and quit.  These change the state, but
 * only between frames — never in the middle of a tick. */

/* Everything the running program owns: the scene, the terminal size, the
 * target frame rate, and two flags the signal handlers flip. */
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
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case 'n': {
    /* n / N step through the rules inside the current family; the number
     * keys are what switch families. */
    int f = family_of(s->op.index);
    int kind = (op_in_family(s->op.index) + 1) % FAMILY_SIZE[f];
    s->op.index = FAMILY_FIRST[f] + kind;
    break;
  }
  case 'N': {
    int f = family_of(s->op.index);
    int kind = (op_in_family(s->op.index) + FAMILY_SIZE[f] - 1) % FAMILY_SIZE[f];
    s->op.index = FAMILY_FIRST[f] + kind;
    break;
  }

  case '1':
  case '2':
  case '3':
  case '4': {
    /* number keys switch family but keep the same kind (union, intersect,
     * …).  Only HARD has XOR, so coming from XOR we fall back a slot. */
    int new_f = ch - '1';
    int kind = op_in_family(s->op.index);
    if (kind >= FAMILY_SIZE[new_f])
      kind = FAMILY_SIZE[new_f] - 1;
    s->op.index = FAMILY_FIRST[new_f] + kind;
    break;
  }

  case '=':
  case '+':
    s->op.blend += PARAM_STEP;
    if (s->op.blend > PARAM_MAX)
      s->op.blend = PARAM_MAX;
    break;
  case '-':
    s->op.blend -= PARAM_STEP;
    if (s->op.blend < PARAM_MIN)
      s->op.blend = PARAM_MIN;
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  case 'z':
    s->view.distance -= CAM_ZOOM_STEP;
    if (s->view.distance < CAM_DIST_MIN)
      s->view.distance = CAM_DIST_MIN;
    break;
  case 'Z':
    s->view.distance += CAM_ZOOM_STEP;
    if (s->view.distance > CAM_DIST_MAX)
      s->view.distance = CAM_DIST_MAX;
    break;

  default:
    break;
  }
  return true;
}

/* §21  the main loop — the one place the state moves forward.  Each pass
 * does the six numbered steps marked below, in order. */

/* Count this frame; a few times a second, work out the real frame rate and
 * start a fresh count.  Returns the number to display (unchanged in
 * between).  Updates the two counters it's handed. */
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

/* Sleep off whatever time is left in this frame's budget, so the loop runs
 * at a steady rate no matter how quick the drawing was. */
static void pace_frame(int sim_fps, int64_t frame_time, int64_t dt) {
  int64_t target_ns = TICK_NS(sim_fps);
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
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {
    if (app->need_resize) { /* 1. EVENTS — apply pending resize */
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* 2. PERFORMANCE — measure frame dt, clamped against a stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > MAX_FRAME_NS)
      dt = MAX_FRAME_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt_sec); /* 3. SIMULATION — advance */

    /* 4. PERFORMANCE — refresh fps readout, then hold the frame rate */
    fps_display = update_fps(&fps_accum, &frame_count, dt, fps_display);
    pace_frame(app->sim_fps, frame_time, dt);

    screen_draw(&app->screen, &app->scene, fps_display); /* 5. RENDER */
    screen_present();

    int ch = getch(); /* 6. EVENTS — drain one key */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
