/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher_primitives.c — a gallery of 17 shapes drawn with text in the
 * terminal, each by shooting a ray per character cell and shading what it hits.
 * One renderer, 17 shapes: a function-pointer table picks the distance function,
 * and Tab cycles it.  The shapes tumble so you see every side, and dithering
 * smooths the shading.  Read raymarcher.c (a single sphere) and
 * raymarcher_cube.c first.
 *
 * Ideas borrowed: Hart's "Sphere Tracing" (1996) for the marching, Iñigo
 * Quílez's distance-function catalogue for the 17 shapes
 * (https://iquilezles.org/articles/distfunctions/), Floyd & Steinberg (1976)
 * for the dithering, and Paul Bourke's 92-char ASCII ramp
 * (https://paulbourke.net/dataformats/asciiart/).
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

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
  N_PRIMS = 17,
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  HUD_COLS = 56,
  FPS_UPDATE_MS = 500,
};

/* §1.2 cell aspect (terminal cells are ~2× as tall as wide). */
#define CELL_ASPECT 2.0f

/* §1.3 ray-march tuning. */
#define RM_MAX_STEPS 100       /* give up after this many steps along one ray  */
#define RM_HIT_EPS 0.002f      /* this close counts as "touched the surface"   */
#define RM_MAX_DIST 20.0f      /* if a ray gets this far out, it hit nothing   */
#define RM_NORM_EPS 0.001f     /* how far apart the points we sample to find the facing (§11) */
#define RM_T_START 0.05f       /* start a bit down the ray so it can't "hit" at the eye */
#define RM_TETRA_SCALE 0.5773f /* ≈1/√3: scales the ±1 tetra corners (cancels in normalise) */

/* §1.4 camera (zoom). */
#define CAM_Z_DEFAULT 4.5f
#define CAM_Z_MIN 3.0f /* nearest zoom — keeps the eye outside the biggest shape */
#define CAM_Z_MAX 12.0f
#define CAM_ZOOM_STEP 0.30f
#define FOV_HALF_TAN 0.65f /* how wide the lens sees; bigger = wider */

/* §1.5 primitive size. */
#define PRIM_SIZE_DEFAULT 1.0f
#define PRIM_SIZE_STEP 1.15f
#define PRIM_SIZE_MIN 0.2f
#define PRIM_SIZE_MAX 3.0f

/* §1.6 the tumble: how fast it spins around the up axis, plus a sideways tilt
 * that follows at rx = ry · ratio.  The ratio is deliberately not a tidy
 * fraction, so the spin never loops and every side eventually faces you. */
#define ROT_Y_SPD_DEFAULT 0.60f
#define ROT_X_RATIO 0.37f
#define ROT_SPD_STEP 1.3f
#define ROT_SPD_MIN 0.0f
#define ROT_SPD_MAX 5.0f

/* §1.7 where the light sits — off to one side and above.  The shape turns;
 * the light stays put. */
#define LIGHT_X -3.0f
#define LIGHT_Y 3.5f
#define LIGHT_Z 2.5f

/* §1.8 how much each kind of light counts when shading a spot (see §11). */
#define KA 0.10f   /* a little glow everywhere, so nothing is pure black  */
#define KD 0.78f   /* brightness from facing the light                    */
#define KS 0.55f   /* shiny-highlight strength                            */
#define SHIN 40.0f /* highlight tightness — bigger = smaller, sharper spot */

/* §1.8b display gamma — terminals don't show brightness evenly, so we bend the
 * shaded values to compensate before turning them into characters (§14). */
#define DISPLAY_GAMMA 2.2f

/* §1.9 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* §1.10 brightness bands + colour-pair slots.  The 92-step character ramp is
 * grouped down to 8 colour bands; the HUD gets its own yellow + cyan. */
enum {
  LUMI_N = 8,
  PAIR_HUD = LUMI_N + 1,
  PAIR_HINT = LUMI_N + 2,
};

/* §1.11 colour themes — a name plus 8 colour codes running dark→bright.  c/C
 * picks which one is active; the shapes and shading never change.  Every colour
 * sits in the bright half of the 256-colour set so even the dimmest stays
 * visible on black. */
typedef struct {
  const char *display_name; /* shown in the HUD, padded to a fixed width */
  short ramp_256[LUMI_N];   /* 8 xterm-256 codes, dark to bright         */
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

/* §1.12 the views you flip through with d / D — the normal lit shape, plus
 * three that paint a raw fact instead of shading it. */
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit shape                */
  DEBUG_NORMALS = 1, /* colour by which way the surface faces      */
  DEBUG_DEPTH = 2,   /* brighter = closer to the camera            */
  DEBUG_STEPS = 3,   /* brighter = the ray took more steps to land */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* ── §2 clock — read the time and sleep ──────────────────────────────── *
 * We use the monotonic clock because it only ever counts forward. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 color — themes + HUD colours ─────────────────────────────────── *
 * Hands ncurses the theme's 8 shades plus the two HUD colours, at startup and
 * on theme change.  Terminals with fewer than 256 colours fall back to white,
 * faking bright/dim with bold/dim. */

static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *t = &THEMES[theme_index];
  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), t->ramp_256[i], COLOR_BLACK);
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

static attr_t lumi_attr(int l) {
  if (l < 0)
    l = 0;
  if (l > LUMI_N - 1)
    l = LUMI_N - 1;
  attr_t a = COLOR_PAIR(l + 1);
  if (COLORS < 256) {
    if (l < 3)
      a |= A_DIM;
    else if (l >= 6)
      a |= A_BOLD;
  }
  return a;
}

/* ── §4 vec2 / vec3 — 2-D and 3-D vector helpers ─────────────────────── *
 * Everything from here through §11 is pure math: it reads its inputs and
 * returns an answer, touching no shared state and no screen. */

typedef struct {
  float x, y;
} V2;
typedef struct {
  float x, y, z;
} V3;

static inline V2 v2(float x, float y) { return (V2){x, y}; }
static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }

static inline V2 v2add(V2 a, V2 b) { return v2(a.x + b.x, a.y + b.y); }
static inline V2 v2sub(V2 a, V2 b) { return v2(a.x - b.x, a.y - b.y); }
static inline V2 v2mul(V2 a, float s) { return v2(a.x * s, a.y * s); }
static inline V2 v2abs(V2 a) { return v2(fabsf(a.x), fabsf(a.y)); }
static inline V2 v2max0(V2 a) { return v2(fmaxf(a.x, 0), fmaxf(a.y, 0)); }
static inline float v2dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
static inline float v2dot2(V2 a) { return a.x * a.x + a.y * a.y; }
static inline float v2len(V2 a) { return sqrtf(v2dot2(a)); }

static inline V3 v3add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3mul(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline V3 v3abs(V3 a) { return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z)); }
static inline V3 v3max0(V3 a) {
  return v3(fmaxf(a.x, 0), fmaxf(a.y, 0), fmaxf(a.z, 0));
}
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float l = v3len(a);
  return l > 1e-7f ? v3mul(a, 1.f / l) : v3(0, 0, 1);
}

/* bounce direction v off a surface that faces way n (n must be unit length) —
 * used to find where light reflects toward the eye (§11). */
static inline V3 v3reflect(V3 v, V3 n) {
  return v3sub(v3mul(n, 2.f * v3dot(n, v)), v);
}

static inline float clmpf(float v, float lo, float hi) {
  return fmaxf(lo, fminf(hi, v));
}

/* ── §5 rotation — spin a point around the up or sideways axis ────────── *
 * tumble spins around Y (up), then X (sideways).  The order matters — doing X
 * first looks different at large angles. */

static inline V3 rot_y(V3 p, float a) {
  float c = cosf(a), s = sinf(a);
  return v3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z);
}
static inline V3 rot_x(V3 p, float a) {
  float c = cosf(a), s = sinf(a);
  return v3(p.x, c * p.y - s * p.z, s * p.y + c * p.z);
}

/* tumble — spin the SAMPLE POINT around Y then X.  Spinning the point one way
 * is the same as spinning the shape the other way: the renderer rotates the
 * question, not the shape (see raymarcher_cube.c for the trick). */
static inline V3 tumble(V3 p, float ry, float rx) {
  return rot_x(rot_y(p, ry), rx);
}

/* ── §6 shapes A — sphere + box family ───────────────────────────────── *
 * §6–§9 are the 17 shape functions.  Each takes a point and returns how far it
 * is from that shape's surface (negative inside, zero on
 * it, positive outside) — the one number the ray march needs.  All from Iñigo
 * Quílez's catalogue.
 *
 *   sphere     distance is just |point| − radius
 *   box        nearest face/edge/corner outside, nearest face inside
 *   round box  a box with its surface pushed out, so the edges round off
 *   box frame  only the twelve edges of a box (three thin slabs)
 *   hex prism  a six-sided bar */

/* 1. Sphere of radius r. */
static float sdf_sphere(V3 p, float r) { return v3len(p) - r; }

/* 2. Box reaching b on each axis. */
static float sdf_box(V3 p, V3 b) {
  V3 q = v3sub(v3abs(p), b);
  return v3len(v3max0(q)) + fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.f);
}

/* 3. Round box — a box (size b) with its surface pushed out by r, rounding the edges. */
static float sdf_round_box(V3 p, V3 b, float r) {
  V3 q = v3sub(v3abs(p), b);
  return v3len(v3max0(q)) + fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.f) - r;
}

/* 4. Box frame — just the box's edges: three thin slabs, keep whichever is nearest. */
static float sdf_box_frame(V3 p, V3 b, float e) {
  V3 pa = v3sub(v3abs(p), b);
  V3 q = v3sub(v3abs(v3add(pa, v3(e, e, e))), v3(e, e, e));
  float d0 = v3len(v3max0(v3(pa.x, q.y, q.z))) +
             fminf(fmaxf(pa.x, fmaxf(q.y, q.z)), 0.f);
  float d1 = v3len(v3max0(v3(q.x, pa.y, q.z))) +
             fminf(fmaxf(q.x, fmaxf(pa.y, q.z)), 0.f);
  float d2 = v3len(v3max0(v3(q.x, q.y, pa.z))) +
             fminf(fmaxf(q.x, fmaxf(q.y, pa.z)), 0.f);
  return fminf(fminf(d0, d1), d2);
}

/* 10. Hex prism — a six-sided bar; rx sizes the hexagon, ry its length. */
static float sdf_hex_prism(V3 p, float rx, float ry) {
  const float kx = -0.8660254f, ky = 0.5f, kz = 0.57735f;
  V3 q = v3abs(p);
  float d = kx * q.x + ky * q.y;
  if (d < 0.f) {
    q.x -= 2.f * d * kx;
    q.y -= 2.f * d * ky;
  }
  float cx = clmpf(q.x, -kz * rx, kz * rx);
  V2 dv = v2(v2len(v2sub(v2(q.x, q.y), v2(cx, rx))) * (q.y < rx ? -1.f : 1.f),
             q.z - ry);
  return fminf(fmaxf(dv.x, dv.y), 0.f) + v2len(v2max0(dv));
}

/* ── §7 shapes B — the torus family ──────────────────────────────────── *
 * Rings and loops.  The trick: collapse the 3-D distance to a 2-D one by first
 * measuring how far the point is from the ring's centre circle.
 *   torus         a full donut
 *   capped torus  a donut with a slice missing (an arc)
 *   link          a stretched donut, like one link of a chain */

/* 5. Torus (donut): ring radius R, tube radius r. */
static float sdf_torus(V3 p, float R, float r) {
  return v2len(v2(v2len(v2(p.x, p.z)) - R, p.y)) - r;
}

/* 6. Capped torus — a donut with a slice cut out.  sx, cx are the sine and
 * cosine of how wide the remaining arc is. */
static float sdf_capped_torus(V3 p, float ra, float rb, float sx, float cx) {
  p.x = fabsf(p.x);
  float k = (cx * p.x > sx * p.y) ? v2dot(v2(p.x, p.y), v2(cx, sx))
                                  : sqrtf(p.x * p.x + p.y * p.y);
  return sqrtf(v3dot(p, p) + ra * ra - 2.f * ra * k) - rb;
}

/* 7. Link — one chain link.  le = how stretched it is, r1 = loop radius,
 * r2 = tube thickness. */
static float sdf_link(V3 p, float le, float r1, float r2) {
  V3 q = v3(p.x, fmaxf(fabsf(p.y) - le, 0.f), p.z);
  return v2len(v2(v2len(v2(q.x, q.y)) - r1, q.z)) - r2;
}

/* ── §8 shapes C — cones, capsules, cylinders ────────────────────────── *
 * Shapes built around a line: find the closest point on the line, then step a
 * fixed radius out from it.
 *   cone        a tilted line whose radius shrinks to a point
 *   plane       a thin disc (so a flat plane has something to show)
 *   capsule     a pill: a sphere swept along a line
 *   cylinder    a capsule with flat ends
 *   round cone  a cone with rounded ends of different sizes (a drip) */

/* 8. Cone — point at the top, circular base h below it.  si, co are the sine
 * and cosine of how wide it flares.  (Quílez's exact formula, kept verbatim.) */
static float sdf_cone(V3 p, float si, float co, float h) {
  V2 q = v2(h * si / co, -h);
  V2 w = v2(v2len(v2(p.x, p.z)), p.y);
  V2 a = v2sub(w, v2mul(q, clmpf(v2dot(w, q) / v2dot(q, q), 0.f, 1.f)));
  V2 b = v2sub(w, v2(clmpf(w.x / q.x, 0.f, 1.f) * q.x, q.y));
  float k = (q.y < 0.f) ? -1.f : 1.f;
  float d = fminf(v2dot(a, a), v2dot(b, b));
  float s = fmaxf(k * (w.x * q.y - w.y * q.x), k * (w.y - q.y));
  return sqrtf(d) * (s >= 0.f ? 1.f : -1.f);
}

/* 9. Plane — drawn as a thin disc (radius R, thickness t) so it actually shows. */
static float sdf_plane_disc(V3 p, float R, float t) {
  float r2d = v2len(v2(p.x, p.z)) - R;
  float ry = fabsf(p.y) - t;
  return fminf(fmaxf(r2d, ry), 0.f) + v2len(v2max0(v2(r2d, ry)));
}

/* 11. Capsule (pill): a sphere of radius r swept from point a to point b. */
static float sdf_capsule(V3 p, V3 a, V3 b, float r) {
  V3 pa = v3sub(p, a), ba = v3sub(b, a);
  float h = clmpf(v3dot(pa, ba) / v3dot(ba, ba), 0.f, 1.f);
  return v3len(v3sub(pa, v3mul(ba, h))) - r;
}

/* 12. Cylinder — vertical, capped.  Half-height h, radius r. */
static float sdf_cylinder(V3 p, float h, float r) {
  V2 d = v2sub(v2abs(v2(v2len(v2(p.x, p.z)), p.y)), v2(r, h));
  return fminf(fmaxf(d.x, d.y), 0.f) + v2len(v2max0(d));
}

/* 13. Round cone — a cone with rounded ends: radius r1 at the bottom, r2 at the
 * top, height h.  Three parts: round bottom, straight sides, round top. */
static float sdf_round_cone(V3 p, float r1, float r2, float h) {
  V2 q = v2(v2len(v2(p.x, p.z)), p.y);
  float b = (r1 - r2) / h, a = sqrtf(1.f - b * b), k = v2dot(q, v2(-b, a));
  if (k < 0.f)
    return v2len(q) - r1;
  if (k > a * h)
    return v2len(v2sub(q, v2(0.f, h))) - r2;
  return v2dot(q, v2(a, b)) - r1;
}

/* ── §9 shapes D — solids with flat faces, and flat slabs ────────────── *
 * The trickier ones: solids with flat faces, and flat 2-D shapes given a little
 * thickness so they aren't invisibly thin.
 *   octahedron  two square pyramids base-to-base
 *   pyramid     square base, apex on top
 *   triangle    a flat triangle with thickness
 *   quad        a flat rectangle with thickness */

/* 14. Octahedron (two pyramids base-to-base), size s.  It checks which of the
 * eight slanted faces is nearest, with a fallback near the centre. */
static float sdf_octahedron(V3 p, float s) {
  V3 q = v3abs(p);
  float m = q.x + q.y + q.z - s;
  V3 r;
  if (3.f * q.x < m)
    r = q;
  else if (3.f * q.y < m)
    r = v3(q.y, q.z, q.x);
  else if (3.f * q.z < m)
    r = v3(q.z, q.x, q.y);
  else
    return m * 0.57735027f;
  float k = clmpf(0.5f * (r.z - r.y + s), 0.f, s);
  return v3len(v3(r.x, r.y - s + k, r.z - k));
}

/* 15. Pyramid — square base, apex on top, height h. */
static float sdf_pyramid(V3 p, float h) {
  float m2 = h * h + 0.25f;
  p.x = fabsf(p.x);
  p.z = fabsf(p.z);
  if (p.z > p.x) {
    float t = p.x;
    p.x = p.z;
    p.z = t;
  }
  p.x -= 0.5f;
  p.z -= 0.5f;
  V3 q = v3(p.z, h * p.y - 0.5f * p.x, h * p.x + 0.5f * p.y);
  float ss = fmaxf(-q.x, 0.f);
  float t = clmpf((q.y - 0.5f * p.z) / (m2 + 0.25f), 0.f, 1.f);
  float a = m2 * (q.x + ss) * (q.x + ss) + q.y * q.y;
  float b = m2 * (q.x + 0.5f * t) * (q.x + 0.5f * t) +
            (q.y - m2 * t) * (q.y - m2 * t);
  float d2 = (fminf(q.y, -q.x * m2 - q.y * 0.5f) > 0.f) ? 0.f : fminf(a, b);
  return sqrtf((d2 + q.z * q.z) / m2) * (fmaxf(q.z, -p.y) >= 0.f ? 1.f : -1.f);
}

/* 16. Triangle with thickness.  The corners are listed anticlockwise (top,
 * bottom-left, bottom-right).  The inside test below uses `> 0` to match that
 * winding — an earlier `< 0` made the triangle render hollow (only its edges
 * showed), so don't flip it. */
static float sdf_triangle(V3 p, float sz, float thick) {
  V2 a = v2(0.f, sz);
  V2 b = v2(-sz * 0.866f, -sz * 0.5f);
  V2 c = v2(sz * 0.866f, -sz * 0.5f);
  V2 q = v2(p.x, p.z);
  V2 ab = v2sub(b, a), bc = v2sub(c, b), ca = v2sub(a, c);
  V2 qa = v2sub(q, a), qb = v2sub(q, b), qc = v2sub(q, c);
  float d2 = fminf(
      fminf(v2dot2(v2sub(
                qa, v2mul(ab, clmpf(v2dot(qa, ab) / v2dot2(ab), 0.f, 1.f)))),
            v2dot2(v2sub(
                qb, v2mul(bc, clmpf(v2dot(qb, bc) / v2dot2(bc), 0.f, 1.f))))),
      v2dot2(
          v2sub(qc, v2mul(ca, clmpf(v2dot(qc, ca) / v2dot2(ca), 0.f, 1.f)))));
  int inside =
      (ab.x * qa.y - ab.y * qa.x > 0.f && bc.x * qb.y - bc.y * qb.x > 0.f &&
       ca.x * qc.y - ca.y * qc.x > 0.f);
  float d_xz = (inside ? -1.f : 1.f) * sqrtf(d2);
  float d_y = fabsf(p.y) - thick;
  return fminf(fmaxf(d_xz, d_y), 0.f) + v2len(v2max0(v2(d_xz, d_y)));
}

/* 17. Quad — a flat rectangle (wx by wz) with thickness. */
static float sdf_quad(V3 p, float wx, float wz, float thick) {
  V2 q2 = v2sub(v2abs(v2(p.x, p.z)), v2(wx, wz));
  float d_xz = v2len(v2max0(q2)) + fminf(fmaxf(q2.x, q2.y), 0.f);
  float d_y = fabsf(p.y) - thick;
  return fminf(fmaxf(d_xz, d_y), 0.f) + v2len(v2max0(v2(d_xz, d_y)));
}

/* ── §10 wrappers + the shape table ──────────────────────────────────── *
 * Each shape function wants its own parameters (radii, heights…); a wrapper
 * turns the one user-facing size knob into those.  A few also pre-tilt flat
 * shapes so they aren't edge-on (and invisible) when the spin is at zero.
 *
 * The k_prims[] table is the renderer's "switch statement" turned into data:
 * one row per shape, and Tab just moves which row is active. */

/* Prim — one entry in the shape gallery: a display name and the function that
 * says how far a point is from that shape's surface. */
typedef struct {
  const char *name;
  float (*sdf)(V3 p, float s); /* distance to this shape, sized by s */
} Prim;

static float w_sphere(V3 p, float s) { return sdf_sphere(p, s); }
static float w_box(V3 p, float s) {
  return sdf_box(p, v3(s * .70f, s * .70f, s * .70f));
}
static float w_round_box(V3 p, float s) {
  return sdf_round_box(p, v3(s * .55f, s * .55f, s * .55f), s * .15f);
}
static float w_box_frame(V3 p, float s) {
  return sdf_box_frame(p, v3(s * .65f, s * .65f, s * .65f), s * .07f);
}

/* Torus: rotate 90° around X so the ring faces the camera. */
static float w_torus(V3 p, float s) {
  return sdf_torus(rot_x(p, 1.5708f), s * .62f, s * .21f);
}

/* Capped torus: same tilt, ~270° arc. */
static float w_cap_torus(V3 p, float s) {
  return sdf_capped_torus(rot_x(p, 1.5708f), s * .62f, s * .17f, sinf(2.4f),
                          cosf(2.4f));
}

/* Link: stands vertical; two rounded ends visible from front. */
static float w_link(V3 p, float s) {
  return sdf_link(p, s * .32f, s * .26f, s * .11f);
}

/* Cone: tip up, base down.  sdf_cone puts the tip at the origin, so we nudge it
 * up to sit centred. */
static float w_cone(V3 p, float s) {
  float h = s * 0.7f;
  V3 pp = v3(p.x, p.y + h * 0.5f, p.z);
  return sdf_cone(pp, sinf(0.48f), cosf(0.48f), h);
}

/* Plane: thin horizontal disc, tilted ~30° forward to show the face. */
static float w_plane(V3 p, float s) {
  V3 q = rot_x(p, 0.52f);
  return sdf_plane_disc(q, s * .85f, s * .04f);
}

/* Hex prism: rotate so a hexagonal face points at the camera. */
static float w_hex_prism(V3 p, float s) {
  return sdf_hex_prism(rot_x(p, 1.5708f), s * .55f, s * .35f);
}

/* Capsule: vertical, endpoints above and below origin. */
static float w_capsule(V3 p, float s) {
  return sdf_capsule(p, v3(0, -s * .52f, 0), v3(0, s * .52f, 0), s * .27f);
}

static float w_cylinder(V3 p, float s) {
  return sdf_cylinder(p, s * .58f, s * .36f);
}

/* Round cone: wider base at bottom, narrower top (drop / drip shape). */
static float w_round_cone(V3 p, float s) {
  V3 pp = v3(p.x, p.y + s * .38f, p.z);
  return sdf_round_cone(pp, s * .34f, s * .11f, s * .76f);
}

static float w_octahedron(V3 p, float s) { return sdf_octahedron(p, s * .82f); }

/* Pyramid: apex up, base shifted down so centred at origin. */
static float w_pyramid(V3 p, float s) {
  float h = s * 1.0f;
  V3 pp = v3(p.x, p.y + h * .33f, p.z);
  return sdf_pyramid(pp, h);
}

/* Triangle slab: tilt ~30° so face AND edge visible during tumble. */
static float w_triangle(V3 p, float s) {
  V3 q = rot_x(p, 0.52f);
  return sdf_triangle(q, s * .80f, s * .055f);
}

/* Quad slab: same tilt as triangle. */
static float w_quad(V3 p, float s) {
  V3 q = rot_x(p, 0.52f);
  return sdf_quad(q, s * .72f, s * .52f, s * .05f);
}

static const Prim k_prims[N_PRIMS] = {
    {"Sphere", w_sphere},          /*  1 */
    {"Box", w_box},                /*  2 */
    {"Round Box", w_round_box},    /*  3 */
    {"Box Frame", w_box_frame},    /*  4 */
    {"Torus", w_torus},            /*  5 */
    {"Capped Torus", w_cap_torus}, /*  6 */
    {"Link", w_link},              /*  7 */
    {"Cone", w_cone},              /*  8 */
    {"Plane", w_plane},            /*  9 */
    {"Hex Prism", w_hex_prism},    /* 10 */
    {"Capsule", w_capsule},        /* 11 */
    {"Cylinder", w_cylinder},      /* 12 */
    {"Round Cone", w_round_cone},  /* 13 */
    {"Octahedron", w_octahedron},  /* 14 */
    {"Pyramid", w_pyramid},        /* 15 */
    {"Triangle", w_triangle},      /* 16 */
    {"Quad", w_quad},              /* 17 */
};

/* ── §11 raymarch — shoot one ray, return what it found ──────────────── *
 * Given a ray and the tumbling shape, returns a Hit and changes nothing else.
 *
 * Same creep-along-the-ray loop as raymarcher.c, except it asks the shape table
 * which shape to measure.  Each cell is traced once; the normal view and the
 * three debug views all read back from the same Hit. */

/* Tumbler — the primitive currently on the turntable: which of the 17 shapes,
 * how big, how far it has turned, and how fast it spins.  cast_ray reads it
 * (never writes); scene_tick advances `angle` by `spin` each tick.  Angle in
 * radians, spin in radians/second, size a scalar (1.0 = default). */
typedef struct {
  int prim;    /* index into k_prims[] — which shape is shown */
  float size;  /* size knob the wrapper scales by */
  float angle; /* how far it has turned so far (radians); the sideways tilt follows from it */
  float spin;  /* turn speed (radians per second) */
} Tumbler;

/* Hit — what one ray found at one cell.  Filled once and stored so every view
 * can read whatever it needs without tracing again.  When hit is false, the
 * other fields are meaningless. */
typedef struct {
  bool hit;        /* did the ray reach the shape at all? */
  V3 p;            /* where on the surface it landed */
  V3 normal;       /* which way the surface faces there */
  float intensity; /* brightness 0..1, for the normal view */
  float t;         /* how far the ray travelled to get there (DEPTH view) */
  int steps;       /* how many creep-steps it took (STEPS view) */
} Hit;

/* the point a distance t along the ray from origin ro */
static inline V3 ray_at(V3 ro, V3 rd, float t) {
  return v3add(ro, v3mul(rd, t));
}

/* Creep along the ray until it touches the shape.  Each step jumps forward by
 * the distance to the surface (always safe — nothing is closer).  Before each
 * measurement we tumble the point into the shape's own frame.  Returns the
 * distance to the hit (or -1 for a miss), and how many steps it took. */
static float rm_march(V3 ro, V3 rd, int prim, float s, float ry,
                      int *out_steps) {
  float rx = ry * ROT_X_RATIO;
  float t = RM_T_START;
  int step;
  for (step = 0; step < RM_MAX_STEPS; step++) {
    V3 p = tumble(ray_at(ro, rd, t), ry, rx);
    float d = k_prims[prim].sdf(p, s);
    if (d < RM_HIT_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  if (out_steps)
    *out_steps = step;
  return -1.f;
}

/* the i-th of four sample directions arranged as a tetrahedron's corners (no
 * two share an axis, which keeps the normal sharp at the shape's edges). */
static inline V3 tetra_offset(int i) {
  float bx = (float)(((i + 3) >> 1) & 1), by = (float)((i >> 1) & 1),
        bz = (float)(i & 1);
  return v3mul(v3(2.f * bx - 1.f, 2.f * by - 1.f, 2.f * bz - 1.f), RM_TETRA_SCALE);
}

/* Which way the surface faces at the hit.  There's no neat formula for these
 * shapes, so we feel it out: check the distance at four nearby points and see
 * which way it grows fastest — that points straight out.  pos is in world
 * space, so each sample is tumbled into the shape's frame first. */
static V3 rm_normal(V3 pos, int prim, float s, float ry) {
  float rx = ry * ROT_X_RATIO;
  V3 n = v3(0, 0, 0);
  for (int i = 0; i < 4; i++) {
    V3 ev = tetra_offset(i);
    V3 sp = tumble(v3add(pos, v3mul(ev, RM_NORM_EPS)), ry, rx);
    n = v3add(n, v3mul(ev, k_prims[prim].sdf(sp, s)));
  }
  return v3norm(n);
}

/* the specular highlight: a bright spot where the light reflects toward the eye.
 * Only where the surface actually faces the light (true N·L > 0) — clamping N·L
 * before building the reflection would let a back face catch a phantom highlight. */
static float specular_term(V3 N, V3 L, V3 V, float ndl) {
  if (ndl <= 0.f)
    return 0.f;
  V3 R = v3reflect(L, N);
  return powf(fmaxf(0.f, v3dot(R, V)), SHIN);
}

/* Turn a surface spot into a brightness 0..1: a faint everywhere-glow, plus how
 * squarely it faces the light, plus a shiny highlight (Phong, 1975). */
static float rm_shade(V3 N, V3 hit, V3 cam, V3 light) {
  V3 L = v3norm(v3sub(light, hit));
  V3 V = v3norm(v3sub(cam, hit));
  float ndl = v3dot(N, L);

  float ambient = KA;
  float diffuse = KD * fmaxf(0.f, ndl);
  float specular = KS * specular_term(N, L, V, ndl);

  float I = ambient + diffuse + specular;
  return I > 1.f ? 1.f : I;
}

/* the primary ray direction through one cell's centre: map the cell to screen
 * coords in [-1,1] (row flipped so row 0 is the top), aim through that point,
 * and let the aspect squash undo the tall-cell stretch. */
static V3 ray_through_pixel(int px, int py, int cw, int ch) {
  float u = ((float)px + 0.5f) / (float)cw * 2.f - 1.f;
  float v = -((float)py + 0.5f) / (float)ch * 2.f + 1.f;
  float pa = ((float)ch * CELL_ASPECT) / (float)cw;
  return v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * pa, -1.f));
}

static Hit cast_ray(int px, int py, int cw, int ch, const Tumbler *tum,
                    V3 light, float cam_z) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.f, 0.f, 0};

  V3 ro = v3(0.f, 0.f, cam_z);
  V3 rd = ray_through_pixel(px, py, cw, ch);

  int steps = 0;
  float t = rm_march(ro, rd, tum->prim, tum->size, tum->angle, &steps);
  h.steps = steps;
  if (t < 0.f)
    return h;

  h.hit = true;
  h.t = t;
  h.p = ray_at(ro, rd, t);
  h.normal = rm_normal(h.p, tum->prim, tum->size, tum->angle);
  h.intensity = rm_shade(h.normal, h.p, ro, light);
  return h;
}

/* ── §12 canvas — the per-frame picture, in three buffers ────────────── *
 * Owns the render buffers (made at startup, remade on resize); the render fills
 * them and the views read them. */

#define CANVAS_MISS -1
#define INTENSITY_MISS -1.0f

/* Canvas — three arrays, one entry per cell, that the render fills in stages:
 *   hits[]      what each ray found (the full Hit).  The debug views read this.
 *   intensity[] just the brightness 0..1 (or a miss marker).  Fed to the
 *               shading + dithering step.
 *   pixels[]    the final character to print (or a miss marker) from that step;
 *               the normal view prints from here.
 * Owned here — calloc'd by canvas_alloc, freed by canvas_free. */
typedef struct {
  int w, h;         /* size in cells */
  Hit *hits;        /* w*h: what each ray found */
  float *intensity; /* w*h: brightness 0..1, or INTENSITY_MISS */
  int *pixels;      /* w*h: character-ramp index, or CANVAS_MISS */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->w = cols;
  c->h = rows;
  c->hits = calloc((size_t)(cols * rows), sizeof(Hit));
  c->intensity = calloc((size_t)(cols * rows), sizeof(float));
  c->pixels = calloc((size_t)(cols * rows), sizeof(int));
}

static void canvas_free(Canvas *c) {
  free(c->hits);
  free(c->intensity);
  free(c->pixels);
  c->hits = NULL;
  c->intensity = NULL;
  c->pixels = NULL;
  c->w = c->h = 0;
}

/* ── §13 the character ramp + its lookup table ───────────────────────── *
 * Paul Bourke ranked all the printable characters from lightest (' ') to
 * darkest ('@') by how much ink they put on the page — 92 of them.  More steps
 * than the usual 8–13 means smoother shading.  The lookup table (k_lut_breaks)
 * just splits 0..1 into 92 even slices; it's a table so a fancier, perceptual
 * split could be dropped in later without touching anything else. */

static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 92 */

static float k_lut_breaks[RAMP_N]; /* filled once at startup */

static void lut_init(void) {
  for (int i = 0; i < RAMP_N; i++)
    k_lut_breaks[i] = (float)i / (float)(RAMP_N - 1);
}

/* which ramp slot (0..91) a 0..1 brightness falls into */
static inline int lut_index(float v) {
  int idx = RAMP_N - 1;
  for (int i = 0; i < RAMP_N - 1; i++) {
    if (v < k_lut_breaks[i + 1]) {
      idx = i;
      break;
    }
  }
  return idx;
}

/* the brightness a ramp slot stands for (its midpoint), for the dither's error */
static inline float lut_value(int idx) {
  if (idx <= 0)
    return 0.f;
  if (idx >= RAMP_N - 1)
    return 1.f;
  return (k_lut_breaks[idx] + k_lut_breaks[idx + 1]) * 0.5f;
}

/* ── §14 shade_to_terminal — make the shading look smooth ────────────── *
 * Three steps turn a brightness into a character:
 *   1. gamma   — terminals are non-linear, so adjust values to match (§1.8b).
 *   2. pick    — choose the closest of the 92 ramp characters.
 *   3. dither  — spread each rounding error onto neighbouring cells, so the
 *                steps between characters blur into what the eye reads as a
 *                smooth gradient (Floyd-Steinberg, 1976).
 *
 * If the scratch buffer can't be allocated it falls back to a plain mapping
 * (no gamma, no dither) so the program keeps running. */

/* true if a working-buffer value is a real shaded pixel, not the miss sentinel */
static inline bool is_lit(float v) { return v > INTENSITY_MISS + 0.5f; }

/* linear light → display brightness (terminals are non-linear, ~gamma 2.2) */
static inline float linear_to_display(float v) {
  return powf(clmpf(v, 0.f, 1.f), 1.f / DISPLAY_GAMMA);
}

/* Floyd-Steinberg: push a pixel's quantisation error onto the four not-yet-drawn
 * neighbours (right, and the three below) so the local average brightness stays
 * correct.  Skips neighbours that were ray misses. */
static void diffuse_error(float *buf, int w, int h, int x, int y, float err) {
  int i = y * w + x;
  if (x + 1 < w && is_lit(buf[i + 1]))
    buf[i + 1] += err * (7.f / 16.f);
  if (y + 1 < h) {
    if (x - 1 >= 0 && is_lit(buf[i + w - 1]))
      buf[i + w - 1] += err * (3.f / 16.f);
    if (is_lit(buf[i + w]))
      buf[i + w] += err * (5.f / 16.f);
    if (x + 1 < w && is_lit(buf[i + w + 1]))
      buf[i + w + 1] += err * (1.f / 16.f);
  }
}

static void shade_to_terminal(Canvas *c) {
  int w = c->w, h = c->h;
  int n = w * h;
  float *buf = malloc((size_t)n * sizeof(float));
  if (!buf) {
    /* fallback: direct linear mapping */
    for (int i = 0; i < n; i++) {
      float v = c->intensity[i];
      if (v < 0.f) {
        c->pixels[i] = CANVAS_MISS;
        continue;
      }
      int idx = (int)(v * (float)(RAMP_N - 1) + 0.5f);
      c->pixels[i] = idx < RAMP_N ? idx : RAMP_N - 1;
    }
    return;
  }

  /* Step 1+2: gamma correction + carry into working buffer. */
  for (int i = 0; i < n; i++) {
    float v = c->intensity[i];
    buf[i] = v < 0.f ? INTENSITY_MISS : linear_to_display(v);
  }

  /* Step 3: Floyd-Steinberg error diffusion. */
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int i = y * w + x;
      float v = buf[i];

      if (v < INTENSITY_MISS + 0.5f) {
        c->pixels[i] = CANVAS_MISS;
        continue;
      }

      int idx = lut_index(v);
      c->pixels[i] = idx;

      float err = v - lut_value(idx);
      diffuse_error(buf, w, h, x, y, err);
    }
  }

  free(buf);
}

/* ── §15 render the canvas, then draw it ─────────────────────────────── *
 * canvas_render traces a ray for every cell and fills the buffers; canvas_draw
 * prints the normal view from pixels[].  The debug views (§16) read the hits
 * directly instead. */
static void canvas_render(Canvas *c, const Tumbler *tum, V3 light, float cam_z) {
  /* Phase 1: cast each ray, store full Hit + intensity. */
  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      Hit h = cast_ray(px, py, c->w, c->h, tum, light, cam_z);
      c->hits[idx] = h;
      c->intensity[idx] = h.hit ? h.intensity : -1.f;
    }
  }
  /* Phase 2: turn those brightnesses into characters, smoothed by dithering. */
  shade_to_terminal(c);
}

/* the normal view: print each cell's chosen character in its band colour */
static void canvas_draw(const Canvas *c, int tcols, int trows) {
  int ox = (tcols - c->w) / 2, oy = (trows - c->h) / 2;
  for (int y = 0; y < c->h; y++) {
    for (int x = 0; x < c->w; x++) {
      int idx = c->pixels[y * c->w + x];
      if (idx == CANVAS_MISS)
        continue;
      int tx = ox + x, ty = oy + y;
      if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows)
        continue;
      char ch = k_ramp[idx];
      int band = (idx * LUMI_N) / RAMP_N; /* group the 92 ramp slots into 8 colour bands */
      attr_t attr = lumi_attr(band);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)ch);
      attroff(attr);
    }
  }
}

/* ── §16 debug views (d/D) — paint one raw fact instead of shading ───── *
 * Each picks one fact from each ray's Hit and paints it straight from the Hit
 * buffer (no dithering), using a simple 13-step ramp. */

static const char DEBUG_GLYPHS[] = " .,:;+*oxOX#@";
#define DEBUG_GLYPHS_N ((int)(sizeof DEBUG_GLYPHS - 1))

static char debug_glyph(float v) {
  int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= DEBUG_GLYPHS_N)
    idx = DEBUG_GLYPHS_N - 1;
  return DEBUG_GLYPHS[idx];
}

static int debug_slot(float v) {
  int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
  int slot = (idx * LUMI_N) / DEBUG_GLYPHS_N;
  return slot;
}

/* NORMALS view — colour by which compass direction the surface faces (its turn
 * around the up axis), and brighten the parts that face upward. */
static void canvas_draw_normals(const Canvas *c, int tcols, int trows) {
  int ox = (tcols - c->w) / 2, oy = (trows - c->h) / 2;
  for (int y = 0; y < c->h; y++) {
    for (int x = 0; x < c->w; x++) {
      const Hit *h = &c->hits[y * c->w + x];
      if (!h->hit)
        continue;
      int tx = ox + x, ty = oy + y;
      if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows)
        continue;

      V3 N = h->normal;
      float azimuth = atan2f(N.x, N.z) / (2.f * (float)M_PI) + 0.5f;
      float y_lit = N.y * 0.5f + 0.5f;
      if (y_lit < 0.f)
        y_lit = 0.f;
      if (y_lit > 1.f)
        y_lit = 1.f;

      attr_t attr = lumi_attr(debug_slot(azimuth));
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(y_lit));
      attroff(attr);
    }
  }
}

/* DEPTH view — nearer the camera = brighter, scaled across how near and far the
 * shape can possibly be. */
static void canvas_draw_depth(const Canvas *c, int tcols, int trows,
                              float cam_z) {
  int ox = (tcols - c->w) / 2, oy = (trows - c->h) / 2;
  float t_min = cam_z - PRIM_SIZE_MAX;
  float t_max = cam_z + PRIM_SIZE_MAX;
  if (t_min < 0.f)
    t_min = 0.f;

  for (int y = 0; y < c->h; y++) {
    for (int x = 0; x < c->w; x++) {
      const Hit *h = &c->hits[y * c->w + x];
      if (!h->hit)
        continue;
      int tx = ox + x, ty = oy + y;
      if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows)
        continue;

      float depth_n = (t_max - h->t) / (t_max - t_min);
      if (depth_n < 0.f)
        depth_n = 0.f;
      if (depth_n > 1.f)
        depth_n = 1.f;

      attr_t attr = lumi_attr(debug_slot(depth_n));
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(depth_n));
      attroff(attr);
    }
  }
}

/* STEPS view — the rim glows: rays grazing the edge take the most steps before
 * they decide whether they hit. */
static void canvas_draw_steps(const Canvas *c, int tcols, int trows) {
  int ox = (tcols - c->w) / 2, oy = (trows - c->h) / 2;
  for (int y = 0; y < c->h; y++) {
    for (int x = 0; x < c->w; x++) {
      const Hit *h = &c->hits[y * c->w + x];
      if (!h->hit)
        continue;
      int tx = ox + x, ty = oy + y;
      if (tx < 0 || tx >= tcols || ty < 0 || ty >= trows)
        continue;

      float steps_n = (float)h->steps / (float)RM_MAX_STEPS;
      if (steps_n > 1.f)
        steps_n = 1.f;

      attr_t attr = lumi_attr(debug_slot(steps_n));
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(steps_n));
      attroff(attr);
    }
  }
}

/* ── §17 scene + screen + app ────────────────────────────────────────── *
 * Scene is the one bundle of program state; scene_tick is the only thing that
 * moves it forward.  Then the screen (ncurses) and the main loop. */

/* Scene — the whole program state, as a table of contents:
 *   WHAT is shown        tumbler      the primitive on the turntable
 *   HOW the user drives   cam_z        camera distance / zoom (z/Z)
 *                         theme_index  colour palette (c/C)
 *                         debug_mode   which view (d/D)
 *   WHERE/when            time         animation clock (seconds)
 *                         paused       freezes the tumble (space)
 *   render buffers        canvas       per-cell hit/intensity/pixel (scratch, §12)
 * (camera and the view knobs are one or two loose fields each — too thin to be
 *  their own types, so they live directly on Scene.) */
typedef struct {
  Tumbler tumbler;

  float cam_z;
  int theme_index;
  DebugMode debug_mode;

  float time;
  bool paused;

  Canvas canvas;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  canvas_alloc(&s->canvas, cols, rows);
  s->tumbler.prim = 0;
  s->tumbler.size = PRIM_SIZE_DEFAULT;
  s->tumbler.angle = 0.f;
  s->tumbler.spin = ROT_Y_SPD_DEFAULT;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme_index = 0;
  s->debug_mode = DEBUG_NORMAL;
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

/* the light never moves; the shape turns under it */
static V3 scene_light(void) { return v3(LIGHT_X, LIGHT_Y, LIGHT_Z); }

/* the only thing that moves the scene forward: advance the clock and the tumble
 * angle (both frozen while paused) */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
  s->tumbler.angle += s->tumbler.spin * dt;
}

static void scene_render(Scene *s) {
  canvas_render(&s->canvas, &s->tumbler, scene_light(), s->cam_z);
}

static void scene_draw(const Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(&s->canvas, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(&s->canvas, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(&s->canvas, cols, rows, s->cam_z);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(&s->canvas, cols, rows);
    break;
  default:
    canvas_draw(&s->canvas, cols, rows);
    break;
  }
}

/* §17.1 screen — ncurses init + HUD + present. */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE); /* getch() returns right away if no key is waiting */
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt our drawing */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

/* the endwin + refresh dance makes ncurses pick up the new terminal size */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0       PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1  PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sfps) {
  erase();
  scene_draw(sc, s->cols, s->rows);

  char status[200];
  snprintf(status, sizeof status,
           " %4.1f fps  [%2d/%2d] %-14s  size:%.2f  zoom:%.2f  "
           "theme:%s  debug:%s  sim:%d  %s ",
           fps, sc->tumbler.prim + 1, N_PRIMS, k_prims[sc->tumbler.prim].name, sc->tumbler.size,
           sc->cam_z, THEMES[sc->theme_index].display_name,
           DEBUG_MODE_NAMES[sc->debug_mode], sfps,
           sc->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " PRIMITIVES ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  Tab/t/T:prim  +/-:size  r/R:spin  "
           "z/Z:zoom  c/C:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §17.2 app — the main loop and the keys.
 * The main loop ties everything together each frame.  Key presses and resizes
 * change the scene between frames, not during the simulation step. */

/* App — everything the running program owns.  running/need_resize are flipped
 * from inside signal handlers, so they're volatile and acted on between frames. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                                /* how many times a second the tumble steps */
  volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *a) {
  screen_resize(&a->screen);
  scene_resize(&a->scene, a->screen.cols, a->screen.rows);
  a->need_resize = 0;
}

static bool app_handle_key(App *a, int ch) {
  Scene *s = &a->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case '\t':
  case 't':
    s->tumbler.prim = (s->tumbler.prim + 1) % N_PRIMS;
    s->tumbler.angle = 0.f;
    break;
  case 'T':
    s->tumbler.prim = (s->tumbler.prim + N_PRIMS - 1) % N_PRIMS;
    s->tumbler.angle = 0.f;
    break;

  case ' ':
    s->paused = !s->paused;
    break;

  case '=':
  case '+':
    s->tumbler.size *= PRIM_SIZE_STEP;
    if (s->tumbler.size > PRIM_SIZE_MAX)
      s->tumbler.size = PRIM_SIZE_MAX;
    break;
  case '-':
    s->tumbler.size /= PRIM_SIZE_STEP;
    if (s->tumbler.size < PRIM_SIZE_MIN)
      s->tumbler.size = PRIM_SIZE_MIN;
    break;

  case 'r':
    s->tumbler.spin += 0.15f;
    if (s->tumbler.spin > ROT_SPD_MAX)
      s->tumbler.spin = ROT_SPD_MAX;
    break;
  case 'R':
    s->tumbler.spin -= 0.15f;
    if (s->tumbler.spin < ROT_SPD_MIN)
      s->tumbler.spin = ROT_SPD_MIN;
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
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'C':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;
  lut_init();
  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t ft = clock_ns(), sa = 0, fa = 0;
  int fc = 0;
  double fpsd = 0.;

  while (app->running) {
    if (app->need_resize) { /* 1. apply a pending resize, before timing the frame */
      app_do_resize(app);
      ft = clock_ns();
      sa = 0;
    }

    /* 2. measure how long the last frame took, capping a long stall */
    int64_t now = clock_ns(), dt = now - ft;
    ft = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick = TICK_NS(app->sim_fps);
    float dts = (float)tick / (float)NS_PER_SEC;

    /* 3. advance the simulation in fixed steps — the only place state moves */
    sa += dt;
    while (sa >= tick) {
      scene_tick(&app->scene, dts);
      sa -= tick;
    }

    scene_render(&app->scene); /* 4. trace the rays and shade into the canvas */

    /* 5. refresh the fps number, then sleep to hold a steady frame rate */
    fc++;
    fa += dt;
    if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
      fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    int64_t el = clock_ns() - ft + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - el);

    screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps); /* 6. draw it */
    screen_present();

    int ch = getch(); /* 7. read one key (changes state for the next frame) */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
