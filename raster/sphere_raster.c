/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere_raster.c — a little planet spinning in coloured ASCII in the terminal.
 * It builds a sphere out of triangles, draws them one by one, and colours each
 * cell by brightness through a switchable palette ("theme"). Keys: s = shading
 * look, t = colour theme, c = show inside faces, +/- = zoom, space = pause.
 *
 * Sister file raster/cube_raster.c is the same renderer on a cube and the best
 * starting point; the theme-palette idea is borrowed from raymarcher/raymarcher.c.
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/sphere_raster.c -o sphere -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 settings — every number you'd tweak, named in one place ── */

enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500, /* how often the fps readout refreshes */
  HUD_COLS = 80,       /* max width of the status string      */
};

/* §1.1 the camera — how wide a view, and how far back we sit (zoom). */
#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 2.6f      /* starting distance      */
#define CAM_DIST_MIN 1.0f  /* closest zoom           */
#define CAM_DIST_MAX 8.0f  /* furthest zoom          */
#define CAM_ZOOM_STEP 0.2f /* step per +/- keypress  */

/* §1.2 the ball's size — 1.0 fills the view nicely at the default zoom. */
#define SPHERE_R 1.0f

/* §1.3 how finely the ball is chopped into triangles, like a globe's grid:
 * U = lines around (longitude), V = lines pole to pole (latitude). More =
 * rounder but slower; 36×24 looks smooth without much cost. */
#define TESS_U 36
#define TESS_V 24

/*
 * The sphere spins under a fixed sun and a fixed camera. A smooth ball looks
 * identical from every angle, so to make the spin actually visible the surface
 * carries a little procedural planet (see planet_albedo) whose continents
 * scroll past as it turns. That — not a moving light — is what reads as a
 * spinning sphere.
 */
#define ROT_Y 0.40f /* spin speed around the vertical axis, rad/s */
#define ROT_X 0.10f /* slight tilt so the poles drift into view   */

/*
 * Lighting: one warm sun plus a cool "atmosphere" glow around the rim. The sun
 * gives a broad day side fading to a dim night side — that terminator sweeping
 * over the continents is the spin cue — and the rim glow outlines the globe.
 */
#define SPEC_GAIN 0.35f /* sun glint on the surface                          */
#define RIM_POWER 2.5f  /* atmosphere thickness (higher = thinner band)      */
#define RIM_GAIN 0.45f  /* atmosphere brightness                            */
#define ATMO_R 0.35f    /* atmosphere colour — cool blue                     */
#define ATMO_G 0.60f
#define ATMO_B 1.00f

/* Glass shader look (see frag_glass): clear centre, bright rim, sharp glint. */
#define GLASS_FRESNEL_POWER 2.5f /* how tight the bright rim is (higher = thinner) */
#define GLASS_BODY 0.10f         /* faint tint of the see-through centre           */
#define GLASS_SHININESS 150.0f   /* very tight, glassy glint                       */
#define GLASS_SPEC_GAIN 0.9f     /* glint brightness                              */

/* Toon shader steps (see frag_toon). */
#define TOON_SPEC_CUT 0.94f /* N·H above this gets a hard white highlight   */
#define TOON_SPEC 0.7f      /* that highlight's brightness                  */
#define TOON_FLOOR 0.12f    /* dimmest band, so the shaded side isn't black */

/* Rasteriser guards (see §6). */
#define NEAR_CLIP_W 0.001f /* a corner this close behind the eye counts as off-screen */
#define W_DIVIDE_EPS 1e-6f /* never divide by anything smaller than this              */

/* The characters we draw with, ordered darkest → brightest by how much ink
 * each one has (Paul Bourke's ramp). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Colour themes. Rather than paint the ball's own colour, we take how bright
 * each shaded cell came out and look it up in an 8-step colour gradient — so
 * the whole ball glows in one palette, switchable with 't'. Every step is kept
 * in the bright half of the palette so even the darkest stays visible on black. */
#define LUMI_N 8 /* colour pairs 1..8 hold the active theme's gradient */
typedef struct {
  const char *name;
  short ramp[LUMI_N]; /* 256-colour ids, dark → bright */
} Theme;
static const Theme THEMES[] = {
    {"CLASSIC", {235, 238, 241, 244, 247, 250, 253, 255}},
    {"AMBER  ", {130, 136, 166, 172, 178, 208, 214, 220}},
    {"MATRIX ", {28, 34, 40, 46, 82, 118, 154, 190}},
    {"NEON   ", {53, 91, 129, 165, 201, 207, 213, 227}},
    {"ICE    ", {25, 31, 38, 45, 51, 87, 123, 159}},
    {"COPPER ", {94, 130, 136, 166, 172, 208, 214, 220}},
};
#define THEME_COUNT ((int)(sizeof THEMES / sizeof THEMES[0]))
#define PAIR_HUD (LUMI_N + 1)  /* 9 : status row — yellow + bold */
#define PAIR_HINT (LUMI_N + 2) /* 10: key-hint row — cyan + bold */

/* Dither pattern — a tiny repeating grid of brightness nudges that scatters
 * the line between two characters, so flat areas don't show hard bands. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.15f /* brightness wobble that hides ramp banding */
#define BOLD_LUMA 0.85f  /* brighter than this ⇒ draw the cell bold   */

/* A terminal character is taller than it is wide (~8×16 px). We tell the camera
 * about that so the sphere comes out round instead of squashed. */
#define CELL_W 8
#define CELL_H 16

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ── §2 math — points, vectors, and 4×4 transforms (all pure helpers) ── */

typedef struct {
  float x, y, z;
} Vec3;
typedef struct {
  float x, y, z, w;
} Vec4;
typedef struct {
  float m[4][4];
} Mat4;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3_add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3_sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3_scale(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_reflect(Vec3 d, Vec3 n) {
  return v3_sub(d, v3_scale(n, 2.f * v3_dot(d, n)));
}
/* blend three corner values by weights that add up to 1 */
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
}
/* mix from a to b — t=0 gives a, t=1 gives b */
static inline Vec3 v3_lerp(Vec3 a, Vec3 b, float t) {
  return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}
/* a soft 0→1 ramp between lo and hi — eases in and out, no hard edge */
static inline float smoothstep01(float lo, float hi, float x) {
  float t = (x - lo) / (hi - lo);
  t = t < 0.f ? 0.f : (t > 1.f ? 1.f : t);
  return t * t * (3.f - 2.f * t);
}

static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}

static inline Mat4 m4_identity(void) {
  Mat4 m = {{{0}}};
  m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
  return m;
}
static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}
static inline Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {{{0}}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}
static inline Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}
static inline Vec3 m4_dir(Mat4 m, Vec3 d) {
  Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
  return v3(r.x, r.y, r.z);
}
static Mat4 m4_rotate_y(float a) {
  Mat4 m = m4_identity();
  m.m[0][0] = cosf(a);
  m.m[0][2] = sinf(a);
  m.m[2][0] = -sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}
static Mat4 m4_rotate_x(float a) {
  Mat4 m = m4_identity();
  m.m[1][1] = cosf(a);
  m.m[1][2] = -sinf(a);
  m.m[2][1] = sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}
/* The camera lens: makes far things look smaller, like a normal 3-D view. */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * .5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}
/* Aim an eye at a target: builds the transform that puts the world in front of
 * it, looking down its line of sight. */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 r = v3_norm(v3(f.y * up.z - f.z * up.y, f.z * up.x - f.x * up.z,
                      f.x * up.y - f.y * up.x));
  Vec3 u =
      v3(r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x);
  Mat4 m = m4_identity();
  m.m[0][0] = r.x;
  m.m[0][1] = r.y;
  m.m[0][2] = r.z;
  m.m[0][3] = -v3_dot(r, eye);
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

/* The matrix that rotates "which way the surface faces" along with the model.
 * For our evenly-scaled sphere it's just the rotation, but doing it the proper
 * way means it'd still be right if we ever squashed or stretched the shape. */
static Mat4 m4_normal_mat(Mat4 m) {
  Mat4 n = m4_identity();
  n.m[0][0] = m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1];
  n.m[0][1] = m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2];
  n.m[0][2] = m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0];
  n.m[1][0] = m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2];
  n.m[1][1] = m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0];
  n.m[1][2] = m.m[0][1] * m.m[2][0] - m.m[0][0] * m.m[2][1];
  n.m[2][0] = m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1];
  n.m[2][1] = m.m[0][2] * m.m[1][0] - m.m[0][0] * m.m[1][2];
  n.m[2][2] = m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0];
  return n;
}

/* For a point on the screen, work out three "blend weights" — one per triangle
 * corner — saying how much each corner counts there. If any is negative the
 * point is outside the triangle; otherwise the weights blend the corners. */
static void barycentric(const float sx[3], const float sy[3], float px,
                        float py, float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-6f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/* True when the point sits inside the triangle (no weight went negative). */
static bool bary_inside(const float b[3]) {
  return b[0] >= 0.f && b[1] >= 0.f && b[2] >= 0.f;
}

/* True when the triangle faces away from us, so we can skip drawing its back.
 * (Its corners wind the "wrong" way on screen, which shows up as area ≤ 0.) */
static bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area <= 0.f;
}

/* The little box of cells a triangle could touch, clipped to the screen, so the
 * fill loop only visits cells near the triangle instead of the whole screen. */
static void triangle_bbox(const float sx[3], const float sy[3], int cols,
                          int rows, int *x0, int *x1, int *y0, int *y1) {
  *x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  *x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  *y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  *y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
}

/* ── §3 shaders — work out each surface point's colour ── */

/* VSIn — one mesh vertex entering the vertex stage: where it is, which way the
 * surface faces there, and its (u,v) spot on the sphere's UV grid. */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} VSIn;

/* VSOut — a vertex after the vertex stage: clip-space position for the
 * rasteriser, plus world position/normal and (u,v) for the fragment stage.
 * custom[] is spare per-vertex payload the rasteriser interpolates (the normals
 * shader stashes the world normal there). */
typedef struct {
  Vec4 clip_pos;
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
} VSOut;

/* FSIn — the inputs to one fragment (one covered cell): the VSOut fields
 * blended across the triangle to this pixel, plus its screen position. */
typedef struct {
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
  int px, py;
} FSIn;

/* FSOut — what a fragment shader hands back: a colour, or discard to skip it. */
typedef struct {
  Vec3 color;
  bool discard;
} FSOut;

/* ShaderProgram — a paired vertex + fragment shader plus the read-only data
 * each one reads. The data is void* so a shader can take whatever uniform
 * struct it wants (phong/normals/glass read Uniforms; toon reads ToonUniforms). */
typedef void (*VertShaderFn)(const VSIn *in, VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in, FSOut *out, const void *uni);
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni;
  const void *frag_uni;
} ShaderProgram;

/* ── uniforms ─────────────────────────────────────────────────────── */

/* Uniforms — everything the shaders read for one frame, in three groups: the
 * transform stack that places the sphere on screen, the single light, and the
 * surface material. The scene rebuilds these each tick; shaders only read them.
 * (No separate Light/Material types — each is one or two fields, so they sit
 * here with the rest of what a shader reads rather than as thin wrappers.) */
typedef struct {
  /* transforms + camera — rebuilt every tick by the scene */
  Mat4 model, view, proj, mvp, norm_mat;
  Vec3 cam_pos;
  /* the one light */
  Vec3 light_pos, light_col, ambient;
  /* the surface material */
  Vec3 obj_color;
  float shininess;
} Uniforms;

/* ToonUniforms — what the toon shader reads: all of Uniforms plus how many
 * flat brightness steps to break the shading into. */
typedef struct {
  Uniforms base;
  int bands;
} ToonUniforms;

/* ── vertex shaders — place each corner, hand its data downstream ── *
 *
 * Both do the same placement work (vert_base); they differ only in the extra
 * payload they stash in custom[] for the rasteriser to blend. */

/* Move one corner from the model's own space to where it lands on screen, and
 * also note its world position and which way it faces — the lighting needs those. */
static void vert_base(const VSIn *in, VSOut *out, const Uniforms *u) {
  out->clip_pos = m4_mul_v4(u->mvp, v4(in->pos.x, in->pos.y, in->pos.z, 1.f));
  out->world_pos = m4_pt(u->model, in->pos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
  out->u = in->u;
  out->v = in->v;
  out->custom[0] = out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

static void vert_default(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
}

/* Same, but also tucks the facing direction into custom[] so the normals
 * shader can read it back per pixel. */
static void vert_normals(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
  out->custom[0] = out->world_nrm.x;
  out->custom[1] = out->world_nrm.y;
  out->custom[2] = out->world_nrm.z;
}

/* ── fragment shaders ────────────────────────────────────────────── */

/* How lit a surface is, softened: a plain "facing the sun?" test makes the
 * shaded side go flat, so this keeps a gentle gradient round the back — a broad
 * day-to-night falloff that still reads on the chunky grid. (Valve's trick.) */
static float half_lambert(float ndotl) {
  float wrap = 0.5f * ndotl + 0.5f;
  return wrap * wrap;
}

/* The base colour at a point on the surface — a little procedural planet. A few
 * low-frequency waves carve "continents" (green) out of "ocean" (blue), with
 * bright ice caps near the poles. It's built from the UV coords, so it turns
 * with the sphere — that scrolling is what makes a smooth ball read as spinning. */
static Vec3 planet_albedo(float u, float v) {
  float lon = u * 6.2831853f; /* longitude, 0..2π around    */
  float lat = v * 3.1415927f; /* latitude,  0..π pole-to-pole */
  float f = sinf(lon + 1.7f) * 0.6f + sinf(lon * 2.f + lat * 2.f) * 0.5f +
            sinf(lon * 3.f - lat + 4.f) * 0.35f + cosf(lat * 3.f) * 0.3f;
  float land = smoothstep01(0.0f, 0.5f, f);
  Vec3 col = v3_lerp(v3(0.08f, 0.28f, 0.68f),  /* ocean */
                     v3(0.32f, 0.52f, 0.20f),  /* land  */
                     land);
  float ice = smoothstep01(0.80f, 0.97f, fabsf(v - 0.5f) * 2.f); /* near the poles */
  return v3_lerp(col, v3(0.92f, 0.95f, 1.0f), ice);
}

/* The headline look: a sunlit planet. Day/night from one sun (broad wrap
 * falloff), a soft sun glint, and a cool atmosphere glow at the rim. The
 * continents come from planet_albedo and scroll as the sphere spins. */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float day = half_lambert(v3_dot(N, L)); /* broad day → night falloff */
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess) * SPEC_GAIN;
  float rim = powf(1.f - fmaxf(0.f, v3_dot(N, V)), RIM_POWER) * RIM_GAIN;

  Vec3 base = planet_albedo(in->u, in->v);
  Vec3 atmo = v3(ATMO_R, ATMO_G, ATMO_B);

  /* ambient + sunlit surface + a sun glint + the atmosphere rim glow */
  float r = u->ambient.x + base.x * u->light_col.x * day + spec * u->light_col.x + atmo.x * rim;
  float g = u->ambient.y + base.y * u->light_col.y * day + spec * u->light_col.y + atmo.y * rim;
  float b = u->ambient.z + base.z * u->light_col.z * day + spec * u->light_col.z + atmo.z * rim;
  out->color.x = powf(fminf(r, 1.f), 1.f / 2.2f);
  out->color.y = powf(fminf(g, 1.f), 1.f / 2.2f);
  out->color.z = powf(fminf(b, 1.f), 1.f / 2.2f);
  out->discard = false;
}

/* Cartoon look: snap the shading into a few flat brightness steps instead of a
 * smooth fade, so the surface reads as banded rings, plus a hard white spot
 * where it catches the light. */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));
  float diff = fmaxf(0.f, v3_dot(N, L));
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > TOON_SPEC_CUT) ? TOON_SPEC : 0.f;
  Vec3 c = planet_albedo(in->u, in->v);
  out->color.x = fminf(c.x * (banded + TOON_FLOOR) + spec, 1.f);
  out->color.y = fminf(c.y * (banded + TOON_FLOOR) + spec, 1.f);
  out->color.z = fminf(c.z * (banded + TOON_FLOOR) + spec, 1.f);
  out->discard = false;
}

/* Debug view: colour each point by which way its surface faces. Good for
 * eyeballing the geometry. (The §5 theme palette then collapses it to
 * brightness, so on screen it reads as a gradient rather than full colour.) */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/*
 * frag_glass — a glassy / soap-bubble look.
 *
 * Real glass is almost see-through when you look straight into it, but lights
 * up like a mirror around the rim where your line of sight grazes the surface
 * (the Fresnel effect). We fake that: a faint tinted body, a bright coloured
 * rim that flares toward the silhouette, and a tiny sharp glint where it
 * catches the light head-on. The dark centre with a glowing edge reads as a
 * hollow glass ball, and the surface pattern drifting under the fixed
 * catch-light shows the spin.
 */
static void frag_glass(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float ndv = fmaxf(0.f, v3_dot(N, V));
  float fres = powf(1.f - ndv, GLASS_FRESNEL_POWER); /* 0 centre → 1 at the rim */
  float glint = powf(fmaxf(0.f, v3_dot(N, H)), GLASS_SHININESS) * GLASS_SPEC_GAIN;

  Vec3 c = planet_albedo(in->u, in->v);
  float edge = GLASS_BODY + fres; /* faint body that flares bright at the rim */
  float r = u->ambient.x + c.x * edge + glint;
  float g = u->ambient.y + c.y * edge + glint;
  float b = u->ambient.z + c.z * edge + glint;
  out->color.x = powf(fminf(r, 1.f), 1.f / 2.2f);
  out->color.y = powf(fminf(g, 1.f), 1.f / 2.2f);
  out->color.z = powf(fminf(b, 1.f), 1.f / 2.2f);
  out->discard = false;
}

typedef enum { SH_PHONG = 0, SH_TOON, SH_NORMALS, SH_GLASS, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong", "toon", "normals", "glass"};

/* ── §4 mesh — build the ball out of triangles (done once at startup) ── */

/* Vertex — one corner of the mesh: position, the way the surface faces there
 * (its normal), and its (u,v) coordinate on the sphere's UV grid. */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;
/* Triangle — three indices into the vertex list above. */
typedef struct {
  int v[3];
} Triangle;
/* Mesh — a whole shape: a bag of vertices and the triangles joining them. Both
 * arrays are malloc'd by tessellate_sphere; mesh_free releases them. */
typedef struct {
  Vertex *verts;
  int nvert; /* how many vertices are filled in */
  Triangle *tris;
  int ntri; /* how many triangles are filled in */
} Mesh;

/* Frees the two arrays tessellate_sphere allocated and zeroes the struct, so a
 * leftover pointer can't be reused by mistake. */
static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/* Build the ball like a globe: walk a grid of latitude rings × longitude lines,
 * drop a point at each crossing, and stitch each grid square into two triangles.
 * On a sphere, "which way the surface faces" at a point is just the direction
 * from the centre out to it — except right at the poles, where that's undefined,
 * so we set it straight up/down by hand. The corner order (winding) is what lets
 * back-face culling tell front from back. */
static Mesh tessellate_sphere(void) {
  int nu = TESS_U;
  int nv = TESS_V;
  float R = SPHERE_R;
  float PI = 3.14159265f;
  float PI2 = 2.f * PI;

  int nvert = (nu + 1) * (nv + 1);
  int ntri = nu * nv * 2;

  Mesh m;
  m.verts = malloc((size_t)nvert * sizeof(Vertex));
  m.tris = malloc((size_t)ntri * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* Step 1 — place a point at every grid crossing. */
  for (int j = 0; j <= nv; j++) {
    float v = (float)j / (float)nv;
    float phi = v * PI; /* latitude  */
    float sp = sinf(phi), cp = cosf(phi);

    for (int i = 0; i <= nu; i++) {
      float u = (float)i / (float)nu;
      float theta = u * PI2; /* longitude */
      float st = sinf(theta), ct = cosf(theta);

      Vec3 pos = v3(R * sp * ct, R * cp, R * sp * st);
      Vec3 nrm = (sp < 1e-6f) /* pole guard */
                     ? ((j == 0) ? v3(0, 1, 0) : v3(0, -1, 0))
                     : v3_norm(pos);

      m.verts[m.nvert++] = (Vertex){pos, nrm, u, v};
    }
  }

  /* Step 2 — stitch each grid square into two triangles.
   *   r0 = top-left   r1 = top-right
   *   r2 = bot-left   r3 = bot-right */
  for (int j = 0; j < nv; j++) {
    for (int i = 0; i < nu; i++) {
      int r0 = j * (nu + 1) + i, r1 = r0 + 1;
      int r2 = r0 + (nu + 1), r3 = r2 + 1;
      m.tris[m.ntri++] = (Triangle){{r0, r2, r1}};
      m.tris[m.ntri++] = (Triangle){{r1, r2, r3}};
    }
  }

  return m;
}

/* ── §5 framebuffer — the off-screen image, theme colours, and blit ── */

/* Cell — one finished character on screen: which glyph, which colour pair, and
 * whether to draw it bold. */
typedef struct {
  char ch;
  int color_pair;
  bool bold;
} Cell;
/* Framebuffer — the off-screen image we build before showing it: a depth value
 * per cell (zbuf — remembers the nearest surface so closer wins) and the
 * finished Cell per cell (cbuf). Both are cols×rows, re-allocated on resize. */
typedef struct {
  float *zbuf;
  Cell *cbuf;
  int cols, rows;
} Framebuffer;

static void fb_alloc(Framebuffer *fb, int cols, int rows) {
  fb->cols = cols;
  fb->rows = rows;
  fb->zbuf = malloc((size_t)(cols * rows) * sizeof(float));
  fb->cbuf = malloc((size_t)(cols * rows) * sizeof(Cell));
}
static void fb_free(Framebuffer *fb) {
  free(fb->zbuf);
  free(fb->cbuf);
  *fb = (Framebuffer){0};
}
static void fb_clear(Framebuffer *fb) {
  for (int i = 0; i < fb->cols * fb->rows; i++)
    fb->zbuf[i] = FLT_MAX;
  memset(fb->cbuf, 0, (size_t)(fb->cols * fb->rows) * sizeof(Cell));
}

/* Point colour pairs 1..8 at the chosen theme's gradient. Called once at start
 * and again every time 't' switches theme. */
static void theme_apply(int idx) {
  if (idx < 0)
    idx = 0;
  if (idx >= THEME_COUNT)
    idx = THEME_COUNT - 1;
  for (int i = 0; i < LUMI_N; i++) {
    if (COLORS >= 256)
      init_pair((short)(i + 1), THEMES[idx].ramp[i], COLOR_BLACK);
    else /* 8-colour terminals can't theme — fall back to white */
      init_pair((short)(i + 1), COLOR_WHITE, COLOR_BLACK);
  }
}

/* Set up colours once: the two HUD pairs, then the starting theme. On an
 * 8-colour terminal themes can't show, so theme_apply falls back to white. */
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

/* rgb_to_cell — read the shaded cell's BRIGHTNESS and turn it into a glyph
 * (denser = brighter) plus a band of the active colour theme. Hue is ignored on
 * purpose: the theme palette, not the material, decides the colour — that's what
 * gives the whole ball one glowing gradient. A little ordered dither hides the
 * banding in the smooth falloff. */
static Cell rgb_to_cell(Vec3 col, int px, int py) {
  float luma = 0.2126f * col.x + 0.7152f * col.y + 0.0722f * col.z;
  float d = luma + (k_bayer[py & 3][px & 3] - 0.5f) * DITHER_AMP;
  d = d < 0.f ? 0.f : d > 1.f ? 1.f : d;
  int idx = (int)(d * (BOURKE_LEN - 1)); /* glyph */
  int slot = (int)(d * LUMI_N);          /* theme band 0..7 */
  if (slot >= LUMI_N)
    slot = LUMI_N - 1;
  return (Cell){k_bourke[idx], slot + 1, d > BOLD_LUMA};
}

static void fb_blit(const Framebuffer *fb) {
  for (int y = 0; y < fb->rows; y++) {
    for (int x = 0; x < fb->cols; x++) {
      Cell c = fb->cbuf[y * fb->cols + x];
      if (!c.ch)
        continue;
      attr_t a = COLOR_PAIR(c.color_pair) | (c.bold ? A_BOLD : 0);
      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)c.ch);
      attroff(a);
    }
  }
}

/* ── §6 the rasteriser — turn triangles into coloured cells ── */

/* Run the vertex shader on a triangle's three corners → screen + world data. */
static void run_vertex_stage(const Mesh *mesh, const Triangle *tri,
                             const ShaderProgram *sh, VSOut vo[3]) {
  for (int vi = 0; vi < 3; vi++) {
    const Vertex *vtx = &mesh->verts[tri->v[vi]];
    VSIn in;
    in.pos = vtx->pos;
    in.normal = vtx->normal;
    in.u = vtx->u;
    in.v = vtx->v;
    memset(&vo[vi], 0, sizeof vo[vi]);
    sh->vert(&in, &vo[vi], sh->vert_uni);
  }
}

/* True if all three corners sit behind the near plane (skip the whole tri). */
static bool behind_near_plane(const VSOut vo[3]) {
  return vo[0].clip_pos.w < NEAR_CLIP_W && vo[1].clip_pos.w < NEAR_CLIP_W &&
         vo[2].clip_pos.w < NEAR_CLIP_W;
}

/* Turn each corner from camera-math coordinates into actual screen cells
 * (Y-flipped), keeping a depth value. Dividing by w is what makes far things
 * look smaller. */
static void project_to_screen(const VSOut vo[3], float sx[3], float sy[3],
                              float sz[3], int cols, int rows) {
  for (int vi = 0; vi < 3; vi++) {
    float w = vo[vi].clip_pos.w;
    if (fabsf(w) < W_DIVIDE_EPS)
      w = W_DIVIDE_EPS;
    sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
    sz[vi] = vo[vi].clip_pos.z / w;
  }
}

/* Blend the three corners' attributes by their weights to get this pixel's
 * shading inputs. */
static void fragment_from_bary(const VSOut vo[3], const float b[3], int px,
                               int py, FSIn *out) {
  out->world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos, vo[2].world_pos,
                           b[0], b[1], b[2]);
  out->world_nrm = v3_norm(v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                   vo[2].world_nrm, b[0], b[1], b[2]));
  out->u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
  out->v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
  out->px = px;
  out->py = py;
  for (int c = 0; c < 4; c++)
    out->custom[c] = b[0] * vo[0].custom[c] + b[1] * vo[1].custom[c] +
                     b[2] * vo[2].custom[c];
}

/* Draw the whole mesh: push each triangle through the pipeline — place its
 * corners, drop ones off-screen or facing away, then fill its covered cells
 * (nearest wins) and shade each one. */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               const ShaderProgram *sh, bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;

  for (int ti = 0; ti < mesh->ntri; ti++) {
    VSOut vo[3];
    run_vertex_stage(mesh, &mesh->tris[ti], sh, vo);
    if (behind_near_plane(vo))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(vo, sx, sy, sz, cols, rows);
    if (cull_backface && is_back_facing(sx, sy))
      continue;

    int x0, x1, y0, y1;
    triangle_bbox(sx, sy, cols, rows, &x0, &x1, &y0, &y1);

    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
        if (!bary_inside(b))
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        int idx = py * cols + px;
        if (z >= fb->zbuf[idx])
          continue; /* something nearer is already here */
        fb->zbuf[idx] = z;

        FSIn fsin;
        fragment_from_bary(vo, b, px, py, &fsin);

        FSOut fsout;
        fsout.discard = false;
        sh->frag(&fsin, &fsout, sh->frag_uni);
        if (fsout.discard)
          continue;

        fb->cbuf[idx] = rgb_to_cell(fsout.color, px, py);
      }
    }
  }
}

/* ── §7 scene — what's in the world and how it changes each frame ── */

/* Scene — the whole spinning-sphere world, as a table of contents:
 *   WHAT  — the sphere mesh.
 *   HOW (simulation knobs) — spin angles, zoom, pause.
 *   HOW (render choices)   — which shader, which colour theme, cull on/off.
 *                            (Kept apart from the sim knobs on purpose: a
 *                            render toggle isn't a simulation knob just because
 *                            both ride the keyboard — no catch-all "Controls".)
 *   DERIVED — the active shader and the uniforms it reads, rebuilt each tick. */
typedef struct {
  Mesh mesh; /* WHAT is shown */

  float angle_x, angle_y; /* spin angles, advanced each tick */
  float cam_dist;         /* zoom distance — +/-             */
  bool paused;            /* freezes the spin (space)        */

  ShaderIdx shade_idx;    /* which shader look ('s')         */
  int theme_idx;          /* which colour theme ('t')        */
  bool cull_backface;     /* hide inner faces? ('c')         */

  ShaderProgram shader;   /* the active shader (from shade_idx) */
  Uniforms uni;           /* what that shader reads (rebuilt each tick) */
  ToonUniforms toon_uni;  /* toon's uniforms (uni + band count) */
} Scene;

/* Rebuild the combined "model → screen" transform whenever the spin, zoom, or
 * window size changed. Takes just the uniforms it edits. */
static void uniforms_update_mvp(Uniforms *u) {
  u->mvp = m4_mul(u->proj, m4_mul(u->view, u->model));
}

/* Place the (fixed) camera straight back along +Z at the current zoom. */
static void scene_set_view(Scene *s) {
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  uniforms_update_mvp(&s->uni);
}

static void scene_build_shader(Scene *s) {
  switch (s->shade_idx) {
  case SH_PHONG:
    s->shader = (ShaderProgram){vert_default, frag_phong, &s->uni, &s->uni};
    break;
  case SH_TOON:
    s->toon_uni.base = s->uni;
    s->toon_uni.bands = 4;
    s->shader = (ShaderProgram){vert_default, frag_toon, &s->uni, &s->toon_uni};
    break;
  case SH_NORMALS:
    s->shader = (ShaderProgram){vert_normals, frag_normals, &s->uni, &s->uni};
    break;
  case SH_GLASS:
    s->shader = (ShaderProgram){vert_default, frag_glass, &s->uni, &s->uni};
    break;
  default:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_sphere();
  s->shade_idx = SH_PHONG;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;

  s->uni.light_pos = v3(4.f, 3.f, 3.f);       /* the sun — fixed up-and-right */
  s->uni.light_col = v3(1.0f, 0.96f, 0.86f);  /* warm sunlight */
  s->uni.ambient = v3(0.05f, 0.06f, 0.10f);   /* faint skylight so the night side isn't pure black */
  s->uni.shininess = 30.f;                    /* broad, soft sun sheen */
  s->uni.obj_color = v3(0.25f, 0.55f, 0.95f); /* (lit shaders use planet_albedo instead) */

  s->uni.model = m4_identity(); /* a valid pose before the first tick */
  s->uni.norm_mat = m4_normal_mat(s->uni.model);

  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
  scene_set_view(s);

  scene_build_shader(s);
}

static void scene_set_zoom(Scene *s) { scene_set_view(s); }

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
  uniforms_update_mvp(&s->uni); /* proj changed → refresh mvp (works paused too) */
}

/* The one place the world moves forward each frame: nudge the spin and rebuild
 * the transforms. `paused` freezes it; key presses change other fields, but
 * only this advances things over time. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  /* spin the sphere; the camera and light stay put */
  s->angle_y += ROT_Y * dt;
  s->angle_x += ROT_X * dt;
  s->uni.model = m4_mul(m4_rotate_y(s->angle_y), m4_rotate_x(s->angle_x));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
  uniforms_update_mvp(&s->uni);
  s->toon_uni.base = s->uni;
}

/* Paint one frame from the current scene into the framebuffer. Takes the scene
 * read-only (const) — it draws from the scene but never changes it. */
static void scene_draw(const Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, s->cull_backface);
  fb_blit(fb);
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

/* ── §8 screen — set up the terminal, draw the HUD, push the frame ── */

/* Screen — the terminal we draw into: its size in character cells, refreshed
 * on startup and on resize. */
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
  typeahead(-1); /* don't let pending keypresses interrupt our drawing */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) {
  (void)s;
  endwin();
}
/* ncurses only picks up the terminal's new size after an endwin()+refresh(). */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* Draw the two overlay strips: status across the top, key hints along the
 * bottom. Colour pairs come from §1. */
static void screen_draw_hud(const Screen *s, const Scene *sc, double fps) {
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " %5.1f fps  shader:%s  theme:%s  zoom:%.1f  cull:%s%s ", fps,
           k_shader_names[sc->shade_idx], THEMES[sc->theme_idx].name,
           sc->cam_dist, sc->cull_backface ? "on " : "off",
           sc->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " SPHERE · RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  s:shader  t:theme  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app — wire it all together and run the loop ── */

/* App — the top-level holder the program runs on: the simulated Scene, the
 * terminal it's shown in, the off-screen Framebuffer, and two flags the signal
 * handlers poke (quit / window-resized). */
typedef struct {
  Scene scene;
  Screen screen;
  Framebuffer fb;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

/* Global so the signal handlers (which take no arguments) can reach the flags. */
static App g_app;
static void on_exit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_free(&app->fb);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
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
  case 's':
  case 'S':
    scene_next_shader(s);
    break;
  case 't':
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
    break;
  case 'T':
    s->theme_idx = (s->theme_idx + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
    break;
  case 'c':
  case 'C':
    s->cull_backface = !s->cull_backface;
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    scene_set_zoom(s);
    break;
  case '-':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    scene_set_zoom(s);
    break;
  default:
    break;
  }
  return true;
}

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                       .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_disp = 0.0;

  while (app->running) {

    /* window resized? rebuild the buffers at the new size */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* time since last frame; capped so a long stall can't make the spin jump */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* one frame: move the world forward, then draw it */
    scene_tick(&app->scene, dt_sec);

    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* draw the sphere, then the HUD on top */
    erase();
    scene_draw(&app->scene, &app->fb);
    screen_draw_hud(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* handle one key press — these just flip settings, they don't run a frame */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* nap for the rest of this frame's time budget to hold a steady rate; don't
     * re-add dt (doing so once ran the loop at about twice the target speed). */
    int64_t elapsed = clock_ns() - frame_time;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  screen_free(&app->screen);
  return 0;
}
