/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sdf_gallery.c — five little 3-D scenes drawn with text in the terminal, each
 * built by gluing simple shapes together a different way: a smooth blend, a
 * boolean cut, a twist, an endless repeat, and a sculpted figure.  Same
 * ray-per-cell sphere tracer as its siblings — read raymarcher.c and
 * raymarcher_primitives.c first.
 *
 * Ideas borrowed: Hart's "Sphere Tracing" (1996) for the marching, and Iñigo
 * Quílez's distance functions + smooth-minimum for the shapes
 * (https://iquilezles.org/articles/distfunctions/).
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 canvas + frame layout.  We redraw the whole picture every tick (not a
 * few rows at a time) so the spinning stays smooth — affordable because the
 * per-frame trig is cached (§7). */
#define CANVAS_MAX_W 220
#define CANVAS_MAX_H 55
#define ROWS_PER_TICK 55   /* = CANVAS_MAX_H — a full frame each tick */
#define CELL_ASPECT 2.1f   /* a terminal cell is ~2.1× taller than wide */
#define FOV_HALF_TAN 0.57f /* how wide the lens sees (about a 60° view) */

/* §1.2 ray-march tuning.  The twist scene takes smaller steps (MARCH_TW): its
 * distance guess slightly overshoots, so a full step can punch through. */
#define MARCH_MAX 120     /* give up after this many steps along one ray */
#define MARCH_EPS 0.0015f /* this close counts as "touched the surface"  */
#define MARCH_FAR 18.0f   /* if a ray gets this far out, it hit nothing   */
#define MARCH_STEP 0.85f  /* fraction of the safe distance to step each time */
#define MARCH_TW 0.60f    /* the smaller step for the twist scene */

/* §1.3 normals + shadow + AO. */
#define NORM_H 0.007f /* how far apart the points we sample to find the facing */
#define SH_STEPS 24
#define SH_K 12.0f
#define SH_FLOOR 0.12f      /* never fully-black shadow             */
#define SHADOW_BIAS 0.010f  /* lift the shadow ray off the surface  */
#define SHADOW_START 0.025f /* and start it a touch out, to avoid self-shadow */
#define AO_STEPS 5
#define AO_STEP 0.12f
#define AO_DECAY 0.72f

/* §1.4 Phong shading coefficients (used in mode 1). */
#define KA 0.10f
#define KD 0.72f
#define KS 0.26f
#define SHIN 24.0f
#define FILL_STR 0.14f      /* fill-light strength                  */
#define RIM_STR 0.09f       /* rim-light strength                   */
#define KEY_ORBIT_SPD 0.40f /* key light circles at this rate (rad/sec) */

/* §1.5 camera + zoom limits.  CAM_DIST_MIN keeps the eye outside the
 * larger primitives; closer than ~1.0 the marcher starts inside. */
#define CAM_DIST_DEF 2.8f
#define CAM_DIST_MIN 1.0f
#define CAM_DIST_MAX 10.0f
#define CAM_ZOOM_STEP 0.20f
#define CAM_THETA_DEF 0.38f
#define CAM_PHI_DEF 0.0f
#define CAM_ORBIT_DEF 0.30f

/* §1.6 colour-pair slots: yellow for the top status line, cyan for the bottom
 * hints, then a block of gradient colours (8 per theme) starting at CP_BASE. */
#define GRAD_N 8
#define N_THEMES 5
#define CP_HUD 1
#define CP_HINT 2
#define CP_BASE 20
#define PALETTE_CYCLE_SPD 0.4f /* gradient colours drift at this rate (bands/sec) */

/* §1.7 the shading characters, dark (space) to bright (@).  Space means the ray
 * hit nothing; the other 7 are the visible shades. */
#define BOURKE_LEN 8
static const char k_bourke[BOURKE_LEN + 1] = " .:=+*#@";
/* terminals don't show brightness evenly, so bend the values first (≈1/2.2) to
 * spread them across the 7 shades. */
#define DISPLAY_GAMMA_ENC 0.45f

/* §1.8 the views you flip through with d / D — the normal lit scene, plus three
 * that paint a raw fact instead of shading it. */
/* DEPTH view: how near/far a hit can be is cam_dist ± this (world units). */
#define DEPTH_HALF_RANGE 1.5f
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit scene                */
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

/* ── §2 clock — read the time ────────────────────────────────────────── *
 * The monotonic clock only ever counts forward, so it won't jump if someone
 * changes the system time. */

static double clock_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ── §3 color — themes + HUD colours ─────────────────────────────────── *
 * Five themes, 8 shades each (dark→bright).  Every shade is kept bright enough
 * to read on a black background.  pixel_to_cell turns a brightness + colour
 * signal into a character and a colour. */

static const short k_palette[N_THEMES][GRAD_N] = {
    /* 0 Studio   gold → yellow → warm white (all r=4..5)          */
    {214, 220, 221, 226, 227, 228, 230, 231},
    /* 1 Ember    red → orange → yellow fire (r=5 throughout)      */
    {196, 202, 203, 208, 209, 214, 220, 226},
    /* 2 Arctic   cyan → sky → lavender → white                    */
    {51, 87, 123, 159, 153, 189, 225, 231},
    /* 3 Toxic    bright green → lime → yellow-green               */
    {46, 82, 118, 119, 154, 155, 190, 226},
    /* 4 Neon     magenta → violet → pink → white                  */
    {201, 165, 171, 207, 177, 213, 219, 231},
};

static int g_theme = 0;
static int g_color_offset = 0; /* rotated each tick for palette anim  */

static void colors_init(void) {
  start_color();
  use_default_colors();

  /* HUD pairs — theme-independent yellow + cyan per CLAUDE.md spec. */
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }

  for (int th = 0; th < N_THEMES; th++)
    for (int i = 0; i < GRAD_N; i++)
      init_pair((short)(CP_BASE + th * GRAD_N + i), k_palette[th][i], -1);
}

/* turn a pixel's brightness + colour signal into a character and a colour.
 * Brightness picks the character (bent first, see §1.7); a lit pixel never gets
 * the blank ' ' so even the dimmest hit shows.  The colour signal picks a shade
 * of the current theme, nudged by the slowly-drifting palette offset. */
static void pixel_to_cell(float luma, float col, char *ch_out,
                          attr_t *attr_out) {
  float lg = powf(fmaxf(luma, 0.0f), DISPLAY_GAMMA_ENC);
  int ri = (int)(lg * (float)(BOURKE_LEN - 1) + 0.5f);
  if (ri < 1)
    ri = 1;
  if (ri >= BOURKE_LEN)
    ri = BOURKE_LEN - 1;
  *ch_out = k_bourke[ri];

  int gi = (int)(col * (float)(GRAD_N - 1) + 0.5f + g_color_offset) % GRAD_N;
  if (gi < 0)
    gi += GRAD_N;
  *attr_out = COLOR_PAIR(CP_BASE + g_theme * GRAD_N + gi) | A_BOLD;
}

/* ── §4 vec3 — a 3-D point or direction (x,y,z) and the math on it ────── *
 * Everything from here through §16 is pure: it reads its inputs and returns an
 * answer, touching no shared state and no screen. */

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
static inline Vec3 v3neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline Vec3 v3cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3norm(Vec3 a) {
  float l = v3len(a);
  return l > 1e-9f ? v3mul(a, 1.0f / l) : v3(0, 1, 0);
}

/* the point a distance t along the ray from origin o (also used to step a tiny
 * way along a surface normal) */
static inline Vec3 ray_at(Vec3 o, Vec3 dir, float t) {
  return v3add(o, v3mul(dir, t));
}

/* reflect v about the axis n (n unit): 2(n·v)n − v, the mirror direction the
 * specular highlight uses (§15) */
static inline Vec3 v3reflect(Vec3 v, Vec3 n) {
  return v3sub(v3mul(n, 2.0f * v3dot(n, v)), v);
}

/* squeeze a value into 0..1 — brightness and colour signals must land there */
static inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/* ── §5 shapes — the building blocks ─────────────────────────────────── *
 * Four basic shapes (sphere, capsule, torus, rounded box).  Each takes a point
 * and returns how far it is from that shape's surface — the one number the ray
 * march needs.  (From Iñigo Quílez's catalogue.) */

/* SDF2 — what every shape and scene reports at a point: how far the nearest
 * surface is (d, the distance the marcher safely steps by; negative inside),
 * plus a 0..1 signal (col) for which shade to paint it. */
typedef struct {
  float d;   /* distance to the nearest surface */
  float col; /* 0..1, picks a colour shade      */
} SDF2;

/* Sphere of radius r. */
static float sdSphere(Vec3 p, float r) { return v3len(p) - r; }

/* Capsule (pill): a tube of radius r from point a to point b. */
static float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
  Vec3 ab = v3sub(b, a);
  Vec3 ap = v3sub(p, a);
  float t = v3dot(ap, ab) / v3dot(ab, ab);
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  return v3len(v3sub(p, v3add(a, v3mul(ab, t)))) - r;
}

/* Torus (donut), lying flat: ring radius R, tube radius r. */
static float sdTorus(Vec3 p, float R, float r) {
  float qx = sqrtf(p.x * p.x + p.z * p.z) - R;
  return sqrtf(qx * qx + p.y * p.y) - r;
}

/* Box of half-size b with its corners rounded off by r. */
static float sdRoundBox(Vec3 p, Vec3 b, float r) {
  float qx = fabsf(p.x) - b.x;
  float qy = fabsf(p.y) - b.y;
  float qz = fabsf(p.z) - b.z;
  float ex = qx > 0.0f ? qx : 0.0f;
  float ey = qy > 0.0f ? qy : 0.0f;
  float ez = qz > 0.0f ? qz : 0.0f;
  float ins = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
  return sqrtf(ex * ex + ey * ey + ez * ez) + ins - r;
}

/* ── §6 operators — ways to combine and bend the shapes ──────────────── *
 * smin glues two shapes with a smooth weld instead of a hard seam.  twist and
 * domain_rep_xz don't touch the shapes — they bend the point you ask about
 * first (spinning it, or folding it into one repeating cell), which makes the
 * shape look spun or endlessly repeated.
 *
 * The plain boolean joins (overlap = min, common part = max, cut-away =
 * max(a, −b)) are written inline in scene 2 — they're one operator each. */

/* rotate_y_cs — turn p around the Y (up) axis, given precomputed cos c and
 * sin s of the angle.  The scenes hoist their trig into g_scene_cache (§7), so
 * they pass it in rather than recomputing cosf/sinf per point. */
static Vec3 rotate_y_cs(Vec3 p, float c, float s) {
  return v3(c * p.x - s * p.z, p.y, s * p.x + c * p.z);
}

/* smooth minimum — like min(a,b), but the two shapes melt together over a
 * rounded weld of width k (k = 0 is a plain sharp min).  (Quílez.) */
static float smin(float a, float b, float k) {
  if (k < 1e-6f)
    return a < b ? a : b;
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

/* twist — spin the point more the higher up it is (angle = height · k), so the
 * shape looks wrung like a towel.  Bend the point before measuring the shape. */
static Vec3 twist(Vec3 p, float k) {
  float angle = p.y * k;
  return rotate_y_cs(p, cosf(angle), sinf(angle));
}

/* fold the point into one repeating tile on the floor grid — so a single shape
 * looks like an endless grid of copies. */
static Vec3 domain_rep_xz(Vec3 p, float cell) {
  p.x -= cell * roundf(p.x / cell);
  p.z -= cell * roundf(p.z / cell);
  return p;
}

/* ── §7 scene cache — this frame's slowly-changing numbers ───────────── *
 * Each scene animates a few values off the clock (how far apart the spheres
 * sit, how hard the twist is, the spin angle…).  Computing sin/cos of the time
 * inside the shape function would repeat it tens of thousands of times a frame,
 * so we work them out ONCE per tick into this struct and the scenes just read
 * it.  scene_cache_update fills it (called from the main loop). */

/* SceneCache — the per-frame animated values, grouped by which scene uses them.
 * The cos/sin pairs are a spin angle's cosine and sine, ready to rotate points. */
typedef struct {
  float scene1_sep;                 /* blend: how far apart the two spheres sit */
  float scene1_k;                   /* blend: how soft the weld between them is  */
  float scene2_rot_c, scene2_rot_s; /* boolean: spin angle (cos, sin)           */
  float scene3_twist_k;             /* twist: how hard the shape is wrung        */
  float scene5_rot_c, scene5_rot_s; /* sculpt: spin angle (cos, sin)            */
} SceneCache;

static SceneCache g_scene_cache;

static void scene_cache_update(float t) {
  g_scene_cache.scene1_sep = 0.55f + 0.38f * cosf(t * 0.70f);
  g_scene_cache.scene1_k = 0.14f + 0.11f * sinf(t * 0.50f);

  float a2 = t * 0.28f;
  g_scene_cache.scene2_rot_c = cosf(a2);
  g_scene_cache.scene2_rot_s = sinf(a2);

  g_scene_cache.scene3_twist_k = 1.85f * sinf(t * 0.48f);

  float a5 = t * 0.38f;
  g_scene_cache.scene5_rot_c = cosf(a5);
  g_scene_cache.scene5_rot_s = sinf(a5);
}

/* ── §8 scene 1: blend — two spheres and a ring melted together ───────── *
 * The five scenes (§8–§12) are each a pure function of a point.  This one welds
 * two spheres and a torus with smooth-minimum.  The gap between the spheres and
 * the weld softness both breathe over time, so the join thickens and thins.
 * Colour goes by distance from the centre. */
static SDF2 scene1_blend(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  float sep = g_scene_cache.scene1_sep;
  float k = g_scene_cache.scene1_k;

  float s1 = sdSphere(v3sub(p, v3(-sep, 0.0f, 0.0f)), 0.48f);
  float s2 = sdSphere(v3sub(p, v3(sep, 0.0f, 0.0f)), 0.48f);
  float to = sdTorus(p, 0.62f, 0.17f);

  float d = smin(smin(s1, s2, k), to, k * 0.75f);

  float col = clamp01(v3len(p) / 1.3f);
  return (SDF2){d, col};
}

/* ── §9 scene 2: boolean — three ways to join two shapes ─────────────── *
 * Three groups side by side, each combining a sphere and a box differently:
 *   left   — both together (overlap / union)
 *   centre — only where they both are (common part / intersection)
 *   right  — sphere with a capsule punched out (cut-away / subtraction)
 * Whichever surface is nearest wins, and gets that group's colour.  The whole
 * scene turns slowly so you see every side. */
static SDF2 scene2_boolean(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  /* slow Y rotation around the whole scene */
  Vec3 pr =
      rotate_y_cs(p, g_scene_cache.scene2_rot_c, g_scene_cache.scene2_rot_s);

  /* LEFT — union of sphere and round box */
  Vec3 pl = v3sub(pr, v3(-2.0f, 0.0f, 0.0f));
  float la = sdSphere(pl, 0.60f);
  float lb = sdRoundBox(v3sub(pl, v3(0.25f, 0.25f, 0.25f)),
                        v3(0.38f, 0.38f, 0.38f), 0.05f);
  float ld = fminf(la, lb); /* min = union */

  /* CENTRE — intersection of sphere and round box */
  float ca = sdSphere(pr, 0.65f);
  float cb = sdRoundBox(pr, v3(0.46f, 0.46f, 0.46f), 0.05f);
  float cd = fmaxf(ca, cb); /* max = intersection */

  /* RIGHT — subtraction: sphere minus capsule */
  Vec3 prr = v3sub(pr, v3(2.0f, 0.0f, 0.0f));
  float ra = sdSphere(prr, 0.62f);
  float rb =
      sdCapsule(prr, v3(0.0f, -0.75f, 0.0f), v3(0.0f, 0.75f, 0.0f), 0.24f);
  float rd = fmaxf(ra, -rb); /* max(a, −b) = subtraction */

  /* Argmin over the three columns; tag with that column's colour. */
  float d, col;
  if (ld <= cd && ld <= rd) {
    d = ld;
    col = 0.15f;
  } else if (cd <= rd) {
    d = cd;
    col = 0.50f;
  } else {
    d = rd;
    col = 0.85f;
  }

  return (SDF2){d, col};
}

/* ── §10 scene 3: twist — a box wrung like a towel ───────────────────── *
 * Twist the point before measuring a tall box, so the box looks wrung; how hard
 * it's wrung swings back and forth over time.  The twist makes the distance
 * guess overshoot, so this scene marches in smaller steps (§14).  Colour goes
 * by height. */
static SDF2 scene3_twist(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  float twist_k = g_scene_cache.scene3_twist_k;
  Vec3 tp = twist(p, twist_k);
  float d = sdRoundBox(tp, v3(0.33f, 1.10f, 0.33f), 0.08f);

  float col = (p.y + 1.1f) / 2.2f;
  col = col > 1.0f ? 1.0f : (col < 0.0f ? 0.0f : col);
  return (SDF2){d, col};
}

/* ── §11 scene 4: repeat — one sphere, endless grid ──────────────────── *
 * Fold the point into one tile, then measure a single sphere — and you get an
 * endless grid of spheres for the price of one.  The colour comes from the
 * point's direction taken BEFORE folding, so the tint sweeps across the whole
 * grid instead of resetting in each tile. */
static SDF2 scene4_repeat(Vec3 p, float t) {
  (void)t;
  float cell = 2.2f;

  /* which compass direction the point lies in (0..1), from the ORIGINAL point
   * before folding — so the tint spans the whole grid */
  float ang = atan2f(p.z, p.x) / (2.0f * (float)M_PI) + 0.5f;

  /* fold p into a single cell */
  Vec3 pr = domain_rep_xz(p, cell);

  float d = sdSphere(pr, 0.72f);
  return (SDF2){d, ang};
}

/* ── §12 scene 5: sculpt — a little figure from eight welded parts ────── *
 * Weld a body, head, neck, belt and four arms together with smooth-minimum into
 * one seamless surface — showing how soft welds turn rigid shapes into
 * something organic.  It turns slowly so all four arms come into view. */
static SDF2 scene5_sculpt(Vec3 p, float t) {
  (void)t; /* trig hoisted to g_scene_cache */
  /* slow Y rotation */
  Vec3 pr =
      rotate_y_cs(p, g_scene_cache.scene5_rot_c, g_scene_cache.scene5_rot_s);

  float k = 0.10f;  /* normal blend strength               */
  float k2 = 0.06f; /* tighter blend for the belt          */

  float body = sdRoundBox(v3sub(pr, v3(0.0f, -0.35f, 0.0f)),
                          v3(0.40f, 0.44f, 0.30f), 0.14f);
  float head = sdSphere(v3sub(pr, v3(0.0f, 0.82f, 0.0f)), 0.27f);
  float neck =
      sdCapsule(pr, v3(0.0f, 0.20f, 0.0f), v3(0.0f, 0.58f, 0.0f), 0.13f);
  float belt = sdTorus(v3sub(pr, v3(0.0f, -0.62f, 0.0f)), 0.43f, 0.075f);
  float armL =
      sdCapsule(pr, v3(-0.42f, -0.10f, 0.0f), v3(-0.86f, -0.52f, 0.0f), 0.10f);
  float armR =
      sdCapsule(pr, v3(0.42f, -0.10f, 0.0f), v3(0.86f, -0.52f, 0.0f), 0.10f);
  float armF =
      sdCapsule(pr, v3(0.0f, -0.10f, 0.40f), v3(0.0f, -0.52f, 0.82f), 0.09f);
  float armB =
      sdCapsule(pr, v3(0.0f, -0.10f, -0.40f), v3(0.0f, -0.52f, -0.82f), 0.09f);

  float d = smin(body, head, k);
  d = smin(d, neck, k);
  d = smin(d, belt, k2);
  d = smin(d, armL, k);
  d = smin(d, armR, k);
  d = smin(d, armF, k);
  d = smin(d, armB, k);

  float col = clamp01((pr.y + 1.0f) / 2.2f);
  return (SDF2){d, col};
}

/* ── §13 scene_map — which scene are we drawing? ─────────────────────── *
 * The renderer only ever calls this; it picks the scene by number, so the rest
 * of the pipeline never needs to know which scene is showing. */

static SDF2 scene_map(int preset, Vec3 p, float t) {
  switch (preset) {
  case 0:
    return scene1_blend(p, t);
  case 1:
    return scene2_boolean(p, t);
  case 2:
    return scene3_twist(p, t);
  case 3:
    return scene4_repeat(p, t);
  default:
    return scene5_sculpt(p, t);
  }
}

/* ── §14 march — find the surface, its facing, its shadow and shading ── *
 * Creep a ray to the surface, work out which way it faces and how shaded it is.
 * All pure: ray + scene in, a number out. */

/* which way the surface faces at a point.  There's no neat formula for these
 * glued-together shapes, so we feel it out: check the distance at four nearby
 * points (arranged as a tetrahedron's corners, no two sharing an axis) and see
 * which way it grows fastest — that points straight out.  Keeps edges crisp. */
static Vec3 sdf_normal(int preset, Vec3 p, float t) {
  const float e = NORM_H;
  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = scene_map(preset, v3add(p, k0), t).d;
  float d1 = scene_map(preset, v3add(p, k1), t).d;
  float d2 = scene_map(preset, v3add(p, k2), t).d;
  float d3 = scene_map(preset, v3add(p, k3), t).d;

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/* how lit a point is by the key light: march a ray toward the light and watch
 * how closely it skims other shapes — a near miss casts a soft half-shadow.
 * Returns 1 (full light) down to SH_FLOOR; we never go fully black, so shadows
 * stay readable. */
static float sdf_shadow(int preset, Vec3 ro, Vec3 rd, float t) {
  float res = 1.0f;
  float tm = SHADOW_START;
  for (int i = 0; i < SH_STEPS && tm < MARCH_FAR; i++) {
    float d = scene_map(preset, ray_at(ro, rd, tm), t).d;
    if (d < MARCH_EPS)
      return SH_FLOOR;
    float r = SH_K * d / tm;
    if (r < res)
      res = r;
    tm += d;
  }
  return res < SH_FLOOR ? SH_FLOOR : res;
}

/* ambient occlusion — how boxed-in a spot is.  Step a few times straight out
 * from the surface; if the shape keeps crowding the steps (less open space than
 * expected), it's in a crevice and we darken it a little. */
static float sdf_ao(int preset, Vec3 p, Vec3 N, float t) {
  float occ = 0.0f, wt = 1.0f;
  for (int i = 1; i <= AO_STEPS; i++) {
    float dist = (float)i * AO_STEP;
    float d = scene_map(preset, ray_at(p, N, dist), t).d;
    occ += wt * (dist - d);
    wt *= AO_DECAY;
  }
  float ao = 1.0f - 2.0f * occ;
  return clamp01(ao);
}

/* creep along the ray until it touches a surface.  Each step jumps forward by
 * the distance to the nearest surface (always safe).  Returns the distance to
 * the hit (or -1 for a miss), the colour signal there, and how many steps it
 * took.  The twist scene steps smaller (its distance guess overshoots). */
static float sdf_march(int preset, Vec3 ro, Vec3 rd, float t, float *col_out,
                       int *out_steps) {
  float step_scale = (preset == 2) ? MARCH_TW : MARCH_STEP;
  float tm = 0.0f;
  int step;
  for (step = 0; step < MARCH_MAX; step++) {
    Vec3 p = ray_at(ro, rd, tm);
    SDF2 s = scene_map(preset, p, t);
    if (s.d < MARCH_EPS) {
      *col_out = s.col;
      *out_steps = step + 1;
      return tm;
    }
    if (tm > MARCH_FAR)
      break;
    tm += s.d * step_scale;
  }
  *out_steps = step;
  return -1.0f;
}

/* ── §15 shade — turn a surface spot into a brightness 0..1 ──────────── *
 * Three modes you flip with the 'l' key:
 *   N·V   — brightness from how squarely the spot faces the camera (cheap)
 *   Phong — a proper lit look: a key light (with optional shadow + highlight),
 *           plus gentle fill and rim lights
 *   Flat  — no shading at all, just the colour (good for inspecting it) */
/* Lighting — how the surface is lit.  mode picks the model (0 = N·V, the
 * camera-facing falloff; 1 = three-point Phong; 2 = flat colour).  shadows and
 * ao toggle the two extra cost terms, which only matter in Phong mode. */
typedef struct {
  int mode;     /* 0 = N·V, 1 = Phong, 2 = Flat   */
  bool shadows; /* cast soft shadows (Phong only) */
  bool ao;      /* apply ambient occlusion        */
} Lighting;

/* the specular highlight: a bright spot where the key light reflects toward the
 * eye.  Only where the surface faces the light (ndl > 0); otherwise a back face
 * would catch a phantom highlight from the reflected vector. */
static float specular_term(Vec3 N, Vec3 key_L, Vec3 V, float ndl) {
  if (ndl <= 0.0f)
    return 0.0f;
  Vec3 R = v3reflect(key_L, N);
  return powf(fmaxf(0.0f, v3dot(R, V)), SHIN);
}

static float shade_luma(int preset, Vec3 hit, Vec3 N, Vec3 V, Vec3 key_L,
                        Vec3 fill_L, Vec3 rim_L, float t,
                        const Lighting *light) {
  if (light->mode == 2)
    return 1.0f;

  float ao = 1.0f;
  if (light->ao)
    ao = sdf_ao(preset, hit, N, t);

  if (light->mode == 0) {
    /* N·V — brightness from how directly surface faces the camera */
    float ndv = fmaxf(0.0f, v3dot(N, V));
    float luma = KA + KD * ndv * ao;
    return luma > 1.0f ? 1.0f : luma;
  }

  /* Phong (light->mode == 1): key diffuse + specular, plus fill and rim. */
  float ndl_key = fmaxf(0.0f, v3dot(N, key_L));
  float ndl_fill = fmaxf(0.0f, v3dot(N, fill_L));
  float ndl_rim = fmaxf(0.0f, v3dot(N, rim_L));
  float spec = specular_term(N, key_L, V, v3dot(N, key_L));

  float sh = 1.0f;
  if (light->shadows) {
    Vec3 sro = ray_at(hit, N, SHADOW_BIAS); /* lift off the surface */
    sh = sdf_shadow(preset, sro, key_L, t);
  }

  float luma = KA + (KD * ndl_key + KS * spec) * sh * ao +
               FILL_STR * KD * ndl_fill * ao + RIM_STR * KD * ndl_rim;
  return luma > 1.0f ? 1.0f : luma;
}

/* ── §16 cast_pixel — one cell's whole trip → a Pixel ────────────────── *
 * Aim a ray for one cell, march it, and shade the hit.  Returns everything any
 * view might need, so each cell is traced just once. */

/* Pixel — what one ray found at one cell.  The normal view uses luma + col; the
 * debug views read the raw fields straight (not the shaded brightness).  When
 * hit is false, the rest is meaningless. */
typedef struct {
  float luma;  /* shaded brightness 0..1 (the normal view) */
  float col;   /* 0..1 colour signal, picks a shade        */
  Vec3 N;      /* which way the surface faces (NORMALS view) */
  float hit_t; /* how far the ray travelled to the hit (DEPTH view) */
  int steps;   /* how many march steps it took (STEPS view) */
  bool hit;    /* did the ray reach a surface at all? */
} Pixel;

/* the primary ray direction through one cell's centre: map the cell to screen
 * coords in [-1,1] (row flipped so row 0 is the top), aim it through the camera
 * basis, and apply the aspect squash for tall terminal cells. */
static Vec3 ray_through_pixel(int px, int py, int cw, int ch, Vec3 fwd,
                              Vec3 right, Vec3 up) {
  float u = ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
  float v = -(((float)py + 0.5f) / (float)ch * 2.0f - 1.0f);
  float aspect = ((float)ch * CELL_ASPECT) / (float)cw;
  return v3norm(v3add(v3add(v3mul(right, u * FOV_HALF_TAN),
                            v3mul(up, v * FOV_HALF_TAN * aspect)),
                      fwd));
}

static Pixel cast_pixel(int px, int py, int cw, int ch, Vec3 cam_pos, Vec3 fwd,
                        Vec3 right, Vec3 up, int preset, float scene_t,
                        Vec3 key_L, Vec3 fill_L, Vec3 rim_L,
                        const Lighting *light) {
  Pixel out = {0.0f, 0.0f, {0, 0, 1}, 0.0f, 0, false};

  Vec3 rd = ray_through_pixel(px, py, cw, ch, fwd, right, up);

  float col_val = 0.0f;
  int steps = 0;
  float hit_t = sdf_march(preset, cam_pos, rd, scene_t, &col_val, &steps);
  out.steps = steps;
  if (hit_t < 0.0f)
    return out;

  out.hit = true;
  out.col = col_val;
  out.hit_t = hit_t;

  Vec3 hit = ray_at(cam_pos, rd, hit_t);
  Vec3 N = sdf_normal(preset, hit, scene_t);
  Vec3 V = v3norm(v3sub(cam_pos, hit));
  out.N = N;

  out.luma = shade_luma(preset, hit, N, V, key_L, fill_L, rim_L, scene_t, light);
  return out;
}

/* ── §17 canvas — render the frame, then draw it ─────────────────────── *
 * We render into one buffer (g_fbuf) and keep the last finished frame in a
 * second (g_stable), then draw from g_stable so a frame never shows up
 * half-drawn.  The camera is snapshotted at the first row, so every row of a
 * frame uses the same camera even though it keeps orbiting. */

static Pixel g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W];
static Pixel g_stable[CANVAS_MAX_H][CANVAS_MAX_W];
static int g_render_row = 0;
static bool g_dirty = true;

static Vec3 g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up;

/* Camera — the eye circles the scene like a point on a globe.  dist is how far
 * out it sits (zoom), theta how high up (think latitude), phi how far around
 * (longitude, which keeps turning at orbit_spd).  camera_basis turns these into
 * an eye position and the directions it looks. */
typedef struct {
  float dist;      /* how far the eye sits from the centre (zoom)     */
  float theta;     /* up/down angle, radians (like latitude)          */
  float phi;       /* around angle, radians (like longitude); orbits  */
  float orbit_spd; /* how fast phi turns (radians/sec)                */
} Camera;

static void camera_basis(const Camera *cam, Vec3 *cam_pos, Vec3 *fwd,
                         Vec3 *right, Vec3 *up) {
  float ct = cosf(cam->theta), st = sinf(cam->theta);
  float cp = cosf(cam->phi), sp = sinf(cam->phi);
  *cam_pos = v3mul(v3(ct * cp, st, ct * sp), cam->dist);
  *fwd = v3norm(v3neg(*cam_pos));
  /* looking almost straight up or down, "up" is ambiguous — borrow a sideways
   * up instead so the camera doesn't flip */
  Vec3 wup =
      (fabsf(v3dot(*fwd, v3(0, 1, 0))) > 0.99f) ? v3(0, 0, 1) : v3(0, 1, 0);
  *right = v3norm(v3cross(*fwd, wup));
  *up = v3cross(*right, *fwd);
}

/* trace a ray for every cell into g_fbuf, then copy it to g_stable as the
 * finished frame.  (Set up to render the whole frame in one call.) */
static bool canvas_render_rows(int cw, int ch, int preset, float scene_t,
                               const Camera *cam, Vec3 key_L, Vec3 fill_L,
                               Vec3 rim_L, const Lighting *light) {
  if (g_render_row == 0)
    camera_basis(cam, &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);

  for (int k = 0; k < ROWS_PER_TICK; k++) {
    int py = g_render_row;
    for (int px = 0; px < cw; px++) {
      g_fbuf[py][px] = cast_pixel(
          px, py, cw, ch, g_snap_cam, g_snap_fwd, g_snap_right, g_snap_up,
          preset, scene_t, key_L, fill_L, rim_L, light);
    }
    if (++g_render_row >= ch) {
      g_render_row = 0;
      memcpy(g_stable, g_fbuf, sizeof g_stable);
      return true;
    }
  }
  return false;
}

/* §17.1 paint the frame: canvas_draw is the normal view; the three debug views
 * paint a raw fact straight, with no brightness-bending or smoothing. */

static const Pixel *canvas_row(int vy, int cw) {
  (void)cw;
  return (vy < g_render_row) ? g_fbuf[vy] : g_stable[vy];
}

/* where to put the canvas's top-left so it sits centred, leaving row 0 for the
 * status line and the last row for the key hints. */
static void canvas_offsets(int cw, int ch, int cols, int rows, int *out_off_x,
                           int *out_off_y) {
  *out_off_x = (cols - cw) / 2;
  *out_off_y = 1 + (rows - 2 - ch) / 2;
  if (*out_off_y < 1)
    *out_off_y = 1;
}

static void canvas_draw(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      char ch_c;
      attr_t attr;
      pixel_to_cell(px->luma, px->col, &ch_c, &attr);

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)ch_c);
      attroff(attr);
    }
  }
}

/* a plain character + colour for the debug views (no brightness-bending) */
static const char DEBUG_GLYPHS[] = " .:=+*#@";
#define DEBUG_GLYPHS_N ((int)(sizeof DEBUG_GLYPHS - 1))

static char debug_glyph(float v) {
  int idx = (int)(v * (float)(DEBUG_GLYPHS_N - 1) + 0.5f);
  if (idx < 1)
    idx = 1;
  if (idx >= DEBUG_GLYPHS_N)
    idx = DEBUG_GLYPHS_N - 1;
  return DEBUG_GLYPHS[idx];
}

static attr_t debug_attr(float v) {
  int gi = (int)(v * (float)(GRAD_N - 1) + 0.5f) % GRAD_N;
  if (gi < 0)
    gi += GRAD_N;
  return COLOR_PAIR(CP_BASE + g_theme * GRAD_N + gi) | A_BOLD;
}

/* NORMALS view — colour by which compass direction the surface faces, and
 * brighten the parts that face upward. */
static void canvas_draw_normals(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      Vec3 N = px->N;
      float azimuth = atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
      float y_lit = N.y * 0.5f + 0.5f;
      if (y_lit < 0.0f)
        y_lit = 0.0f;
      if (y_lit > 1.0f)
        y_lit = 1.0f;

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(azimuth);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(y_lit));
      attroff(attr);
    }
  }
}

/* DEPTH view — nearer the camera is brighter. */
static void canvas_draw_depth(int cw, int ch, int cols, int rows,
                              float cam_dist) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  /* Bracket t by camera distance so closer hits ≈ 1. */
  float t_min = cam_dist - DEPTH_HALF_RANGE;
  float t_max = cam_dist + DEPTH_HALF_RANGE;
  if (t_min < 0.0f)
    t_min = 0.0f;

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      float depth_n = clamp01((t_max - px->hit_t) / (t_max - t_min));

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(depth_n);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(depth_n));
      attroff(attr);
    }
  }
}

/* STEPS view — the rim glows: rays grazing the edge take the most steps. */
static void canvas_draw_steps(int cw, int ch, int cols, int rows) {
  int off_x, off_y;
  canvas_offsets(cw, ch, cols, rows, &off_x, &off_y);

  for (int vy = 0; vy < ch && vy < CANVAS_MAX_H; vy++) {
    const Pixel *row_src = canvas_row(vy, cw);
    for (int vx = 0; vx < cw && vx < CANVAS_MAX_W; vx++) {
      const Pixel *px = &row_src[vx];
      if (!px->hit)
        continue;

      float steps_n = (float)px->steps / (float)MARCH_MAX;
      if (steps_n > 1.0f)
        steps_n = 1.0f;

      int tx = off_x + vx;
      int ty = off_y + vy;
      if (tx < 0 || tx >= cols || ty < 0 || ty >= rows - 1)
        continue;
      attr_t attr = debug_attr(steps_n);
      attron(attr);
      mvaddch(ty, tx, (chtype)(unsigned char)debug_glyph(steps_n));
      attroff(attr);
    }
  }
}

/* ── §18 app — the program's state, the keys, and the main loop ──────── *
 * App holds everything; app_tick is the only thing that moves it forward. */

static const char *k_preset_names[5] = {"1:Blend", "2:Boolean", "3:Twist",
                                        "4:Repeat", "5:Sculpt"};
static const char *k_theme_names[N_THEMES] = {"Studio", "Ember", "Arctic",
                                              "Toxic", "Neon"};

/* App — all per-run state, as a table of contents:
 *   WHAT is shown    preset       which of the 5 scenes (keys 1..5)
 *                    scene_t      animation clock fed to the scenes (seconds)
 *   HOW the user drives  camera       the orbiting eye (zoom z/Z, orbit +/-, r)
 *                    light        lighting model + shadow/ao toggles (l/s/o)
 *                    debug_mode   which view (d/D)
 *   WHERE/when        paused       freezes the tick (space/p)
 *                    color_phase  slow palette colour-cycle accumulator
 *   canvas + run      cw, ch       canvas size in cells (set on resize)
 *                    quit         set by q/ESC to end the loop
 * (the framebuffers live as file-scope globals in §17; the active palette index
 *  g_theme / g_color_offset in §3.) */
typedef struct {
  int preset;
  float scene_t;

  Camera camera;
  Lighting light;
  DebugMode debug_mode;

  bool paused;
  float color_phase;

  int cw, ch;
  bool quit;
} App;

/* Key presses and resizes change the App between frames, not during a tick. */

static volatile sig_atomic_t g_resize = 0;
static void on_sigwinch(int sig) {
  (void)sig;
  g_resize = 1;
}

static void app_init(App *a, int cols, int rows) {
  memset(a, 0, sizeof *a);
  a->cw = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
  /* Reserve TWO rows for the HUD: row 0 (yellow status) + last row
   * (cyan hint).  Canvas occupies rows 1..rows-2. */
  a->ch = (rows - 2 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 2;
  a->preset = 0;
  a->camera.dist = CAM_DIST_DEF;
  a->camera.theta = CAM_THETA_DEF;
  a->camera.phi = CAM_PHI_DEF;
  a->camera.orbit_spd = CAM_ORBIT_DEF;
  a->light.ao = true;
  a->light.shadows = false; /* off by default for speed */
  a->light.mode = 0;    /* N·V default */
  a->debug_mode = DEBUG_NORMAL;

  memset(g_fbuf, 0, sizeof g_fbuf);
  memset(g_stable, 0, sizeof g_stable);
  g_render_row = 0;
  g_dirty = true;

  camera_basis(&a->camera, &g_snap_cam, &g_snap_fwd, &g_snap_right, &g_snap_up);
}

static void app_resize(App *a, int cols, int rows) {
  a->cw = (cols > CANVAS_MAX_W) ? CANVAS_MAX_W : cols;
  a->ch = (rows - 2 > CANVAS_MAX_H) ? CANVAS_MAX_H : rows - 2;
  g_render_row = 0;
  g_dirty = true;
}

static void app_handle_key(App *a, int ch) {
  switch (ch) {
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
    if (a->preset != ch - '1') {
      a->preset = ch - '1';
      g_dirty = true;
      a->scene_t = 0.0f;
    }
    break;
  case 't':
    g_theme = (g_theme + 1) % N_THEMES;
    break;
  case 'p':
  case ' ':
    a->paused = !a->paused;
    break;
  case 's':
    a->light.shadows = !a->light.shadows;
    g_dirty = true;
    break;
  case 'o':
    a->light.ao = !a->light.ao;
    g_dirty = true;
    break;
  case 'l':
    a->light.mode = (a->light.mode + 1) % 3;
    g_dirty = true;
    break;
  case 'd':
    a->debug_mode = (DebugMode)((a->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    a->debug_mode =
        (DebugMode)((a->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;
  case '+':
  case '=':
    a->camera.orbit_spd += 0.05f;
    break;
  case '-':
    a->camera.orbit_spd -= 0.05f;
    if (a->camera.orbit_spd < 0.0f)
      a->camera.orbit_spd = 0.0f;
    break;
  case 'z':
    /* zoom IN — closer camera, smaller cam_dist */
    a->camera.dist -= CAM_ZOOM_STEP;
    if (a->camera.dist < CAM_DIST_MIN)
      a->camera.dist = CAM_DIST_MIN;
    g_dirty = true;
    break;
  case 'Z':
    /* zoom OUT — farther camera, larger cam_dist */
    a->camera.dist += CAM_ZOOM_STEP;
    if (a->camera.dist > CAM_DIST_MAX)
      a->camera.dist = CAM_DIST_MAX;
    g_dirty = true;
    break;
  case 'r':
    a->camera.dist = CAM_DIST_DEF;
    a->camera.theta = CAM_THETA_DEF;
    a->camera.phi = CAM_PHI_DEF;
    a->camera.orbit_spd = CAM_ORBIT_DEF;
    g_dirty = true;
    break;
  case 'q':
  case 27: /* ESC */
    a->quit = true;
    break;
  default:
    break;
  }
}

/* the only thing that moves the scene forward: advance the clock, orbit the
 * camera, and drift the palette (all frozen while paused). */
static void app_tick(App *a, float dt) {
  if (a->paused)
    return;

  a->scene_t += dt;
  a->camera.phi += a->camera.orbit_spd * dt;

  /* nudge the gradient colours along for a slow palette shimmer */
  a->color_phase += PALETTE_CYCLE_SPD * dt;
  g_color_offset = (int)(a->color_phase) % GRAD_N;
}

/* draw whichever view is selected */
static void draw_active(const App *a, int cols, int rows) {
  switch (a->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(a->cw, a->ch, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(a->cw, a->ch, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(a->cw, a->ch, cols, rows, a->camera.dist);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(a->cw, a->ch, cols, rows);
    break;
  default:
    canvas_draw(a->cw, a->ch, cols, rows);
    break;
  }
}

/* the two HUD strips: title + fps + settings on the top row, key reminders on
 * the bottom.  fps sits in the left label so it stays visible even when the
 * settings string is too wide for the terminal. */
static void draw_hud(const App *a, double fps, int cols, int rows) {
  char left[48];
  snprintf(left, sizeof left, " SDF GALLERY  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  char status[220];
  snprintf(status, sizeof status,
           " scene:%s  theme:%s  light:%s  debug:%s  "
           "shadow:%s  ao:%s  zoom:%.2f  orbit:%.2f  %s ",
           k_preset_names[a->preset], k_theme_names[g_theme],
           a->light.mode == 1 ? "Phong" : (a->light.mode == 2 ? "Flat" : "N·V"),
           DEBUG_MODE_NAMES[a->debug_mode], a->light.shadows ? "on" : "off",
           a->light.ao ? "on" : "off", (double)a->camera.dist, (double)a->camera.orbit_spd,
           a->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  /* clamp so status doesn't overlap the left label */
  int max_slen = cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  1-5:scene  l:light  d/D:debug  "
           "t:theme  s:shadow  o:ao  z/Z:zoom  +/-:orbit  r:reset ");
  clrtoeol();
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* the main loop: set up, then each frame read input, advance the scene, render
 * it, draw it, and pause briefly. */
int main(void) {
  signal(SIGWINCH, on_sigwinch);

  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  colors_init();

  int cols, rows;
  getmaxyx(stdscr, rows, cols);

  App a;
  app_init(&a, cols, rows);

  double t_prev = clock_now();
  double fps_window = 0.0;  /* accumulated dt within window */
  int fps_frames = 0;       /* frames in current window     */
  double fps_display = 0.0; /* most recent measured fps     */

  while (!a.quit) {
    /* handle input first: apply a pending resize, then any queued keys */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      app_resize(&a, cols, rows);
    }

    int ch;
    while ((ch = getch()) != ERR)
      app_handle_key(&a, ch);

    /* measure how long the last frame took, capping a long stall */
    double t_now = clock_now();
    float dt = (float)(t_now - t_prev);
    if (dt > 0.1f)
      dt = 0.1f;
    t_prev = t_now;

    app_tick(&a, dt); /* advance the scene — the only place state moves */

    /* refresh the fps number about twice a second */
    fps_frames++;
    fps_window += dt;
    if (fps_window >= 0.5) {
      fps_display = (double)fps_frames / fps_window;
      fps_frames = 0;
      fps_window = 0.0;
    }

    /* work out the three light directions for this frame (the key light orbits) */
    float kang = a.scene_t * KEY_ORBIT_SPD;
    Vec3 key_L = v3norm(v3(cosf(kang) * 0.70f, 0.65f, sinf(kang) * 0.70f));
    Vec3 fill_L = v3norm(v3(-1.5f, 0.50f, -1.2f));
    Vec3 rim_L = v3norm(v3(0.0f, -0.40f, -1.0f));

    /* on a settings change, wipe the buffer once; then work out this frame's
     * animated values (§7) before rendering */
    if (g_dirty) {
      g_render_row = 0;
      memset(g_fbuf, 0, sizeof(Pixel) * (size_t)a.ch * CANVAS_MAX_W);
      g_dirty = false;
    }
    scene_cache_update(a.scene_t);

    /* render the frame, then draw the view and the HUD */
    canvas_render_rows(a.cw, a.ch, a.preset, a.scene_t, &a.camera, key_L, fill_L,
                       rim_L, &a.light);

    erase();
    draw_active(&a, cols, rows);
    draw_hud(&a, fps_display, cols, rows);
    wnoutrefresh(stdscr);
    doupdate();

    /* sleep ~16 ms so we don't spin faster than ~60 fps */
    struct timespec sl = {0, 16000000L};
    nanosleep(&sl, NULL);
  }

  endwin();
  return 0;
}
