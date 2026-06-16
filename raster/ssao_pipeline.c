/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ssao_pipeline.c — a stepped pyramid + a sphere on a floor, in coloured ASCII,
 * showing ambient occlusion: the soft darkening that gathers where surfaces
 * meet (under the sphere, in the step corners, around each base). It's worked
 * out from the screen alone — hence "screen-space" AO (Crytek, 2007).
 *
 * Press 'a' to compare three views — the grayscale "where is it dark" map, the
 * lighting without AO, and the lighting with AO. Pause (space) and toggle to
 * see exactly what AO adds; '['/']' grow/shrink it.
 *
 * Sister files: raster/deferred_rendering_pipeline.c (same G-buffer, no AO) and
 * raster/cube_raster.c (the plain triangle renderer this builds on). Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/ssao_pipeline.c -o ssao -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

/* §1.1 frame rate + readout. */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,            /* how often the fps number refreshes */
  HUD_ROWS = 5, /* rows reserved at the bottom: title + 3 info rows + key hint */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 biggest screen we handle — the off-screen buffers are sized once to this. */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 the camera — it orbits the scene at a fixed height, looking at the middle. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f

#define CAM_DIST 6.0f
#define CAM_DIST_MIN 3.0f
#define CAM_DIST_MAX 12.0f
#define CAM_ZOOM_STEP 0.4f
#define CAM_EYE_Y 1.9f  /* slight elevation, sees floor + top */
#define CAM_LOOK_Y 1.3f /* aim at structure mid-height        */
#define CAM_ORBIT_RAD_PER_SEC 0.20f

#define CELL_W 8
#define CELL_H 16

/* §1.4 the SSAO knobs.
 *
 * RADIUS — how far out (in world units) we look for nearby surfaces. Small =
 *   only tight corners darken; large = a soft overall shade. Sized ~one step.
 * BIAS — a tiny fudge so a flat surface doesn't darken itself. Measured in
 *   real (linear) distance, which behaves the same near and far; an earlier
 *   version measured it in screen depth and left random blotches crawling on
 *   the far floor.
 * SAMPLES × VARIANTS — 12 probe directions per cell, in 4 different sets spread
 *   over a 2×2 tile (48 directions across the tile); the later 3×3 blur then
 *   averages neighbours, so each result is smoothed by 100+ probes. */
#define SSAO_SAMPLES 12
#define SSAO_KERNEL_VARIANTS 4
#define SSAO_RADIUS_DEF 0.60f /* a touch wide so the dark bands read at terminal res */
#define SSAO_RADIUS_MIN 0.10f
#define SSAO_RADIUS_MAX 1.40f
#define SSAO_RADIUS_STEP 0.05f
#define SSAO_BIAS 0.02f /* in real distance, not screen depth */

/* §1.4b rasteriser guards (see §3/§6). */
#define CLIP_W_MIN 0.001f  /* a vertex/sample with w below this is behind the eye */
#define W_DIVIDE_EPS 1e-6f /* never divide by a w smaller than this */

/* §1.5 lighting — one warm sun plus a soft fill light from all around.
 *
 * AO scales the AMBIENT term only — bright enough that the darkening shows, not
 * so bright it flattens the picture. The fill isn't one flat colour: it fades
 * from a cool "sky" tint on upward faces to a warm "bounce" tint on downward
 * ones, so the flat faces look lit by an environment instead of one dead grey.
 * (That's the AO-safe bit of nicer lighting — a rim glow or a fill light would
 * brighten exactly the crevices SSAO is trying to darken.) The sun is direct
 * and unaffected by AO. */
/* The sun's direction: up high, a little to one side, and toward the camera so
 * the faces we actually see are the lit ones. Aim it the other way and only the
 * unseen back of the structure gets lit. */
static const float SUN_DIR[3] = {-0.55f, -0.85f, -0.30f};
static const float SUN_COL[3] = {0.95f, 0.85f, 0.65f};
static const float AMBIENT_SKY[3] = {0.34f, 0.40f, 0.50f};    /* fill from above — cool   */
static const float AMBIENT_GROUND[3] = {0.30f, 0.27f, 0.23f}; /* fill from below — warm   */
#define SHININESS 24.0f
#define SPEC_GAIN 0.30f

/* §1.6 where things sit: a floor at height 0, three square steps stacked into a
 * little pyramid (each resting exactly on the one below), and a sphere sitting
 * on the top step. The four spots SSAO darkens are the floor around the base
 * step, each step-top around the next step's edge, and a ring under the sphere. */
#define FLOOR_HALF_X 4.0f
#define FLOOR_HALF_Z 4.0f

#define STEP0_HX 1.50f
#define STEP1_HX 1.00f
#define STEP2_HX 0.55f
#define STEP_HY 0.40f            /* every step is 0.8 tall                 */
#define STEP0_CY (STEP_HY)       /* 0.4  → y 0..0.8 */
#define STEP1_CY (STEP_HY * 3.f) /* 1.2  → y 0.8..1.6 */
#define STEP2_CY (STEP_HY * 5.f) /* 2.0  → y 1.6..2.4 */

#define SPHERE_RADIUS 0.35f
#define SPHERE_CY (STEP_HY * 6.f + SPHERE_RADIUS) /* 2.4 + 0.35 */
#define SPHERE_RINGS 12
#define SPHERE_SEGS 18

/* Object indices — used to assign meshes/colours in scene_init. */
enum {
  OBJ_FLOOR = 0,
  OBJ_STEP0,
  OBJ_STEP1,
  OBJ_STEP2,
  OBJ_SPHERE,
  N_OBJECTS,
};

/* §1.7 the characters we draw with, darkest → brightest by how much ink each
 * one has (Paul Bourke's ramp). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.8 dither — a tiny repeating grid of brightness nudges so big flat areas
 * don't all snap to one character and show hard bands. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.10f
#define BOLD_LUMA 0.85f /* brighter than this ⇒ draw the cell bold */
#define DIM_LUMA 0.15f  /* darker than this ⇒ draw it dim          */

/* §1.9 colour-slot numbers (ncurses refers to each colour by a number). */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218
#define CUBE_SIZE 6 /* 6 steps per channel → 6×6×6 = 216 colours we can draw */

/* ── §2 clock — a steady timer and a sleep, to pace the frames ── */

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

/* ── §3 math — vectors, 4×4 matrices, and the camera transforms ── */

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
static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}

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
static inline Vec3 v3_bary(Vec3 p0, Vec3 p1, Vec3 p2, float b0, float b1,
                           float b2) {
  return v3(b0 * p0.x + b1 * p1.x + b2 * p2.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y,
            b0 * p0.z + b1 * p1.z + b2 * p2.z);
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

/* The perspective lens. It sets things up so that dividing by w (done later)
 * shrinks far-away things on screen, the way real distance does. */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * 0.5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  /* "right" is forward crossed with up, in this order. Flip the order and you
   * also flip up, which rolls the whole view 180° — the scene comes out
   * upside-down and mirrored. */
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

/* The matrix that transforms a surface's normal (its "which way am I facing"
 * arrow). You can't just reuse the model matrix: stretching an object more in
 * one direction would tilt the arrows the wrong way. This fixes that. */
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

static Mat4 m4_translate(float x, float y, float z) {
  Mat4 m = m4_identity();
  m.m[0][3] = x;
  m.m[1][3] = y;
  m.m[2][3] = z;
  return m;
}

/* For one point and one triangle, how much each of the three corners "owns"
 * that point (three weights that add up to 1). These weights both tell us if
 * the point is inside and let us blend the corners' colours/depths. Returns
 * -1s for a zero-area triangle so the caller skips it. */
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

/* All three weights non-negative means the point is inside the triangle. */
static bool bary_inside(const float b[3]) {
  return b[0] >= 0.f && b[1] >= 0.f && b[2] >= 0.f;
}

/* Turn a triangle's three transformed corners into actual screen positions:
 * divide by w (this is what makes distant things smaller), flip Y so up is up,
 * and keep a depth value per corner for sorting near vs far later. */
static void clip_to_screen(const Vec4 clip[3], float sx[3], float sy[3],
                           float sz[3], int cols, int rows) {
  for (int vi = 0; vi < 3; vi++) {
    float w = clip[vi].w;
    if (fabsf(w) < W_DIVIDE_EPS)
      w = W_DIVIDE_EPS;
    sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
    sz[vi] = clip[vi].z / w;
  }
}

/* Is this triangle facing away from us? We never see the back of a solid
 * object, so we skip those to save work. The trick: with our corner ordering
 * and Y-flip, a triangle facing us comes out with negative area; zero or
 * positive means it's turned away. */
static bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area >= 0.f;
}

/* The smallest screen rectangle that contains the triangle (clamped to the
 * screen edges), so the fill loop only checks cells the triangle could cover
 * instead of the whole screen. */
static void triangle_bbox(const float sx[3], const float sy[3], int cols,
                          int rows, int *x0, int *x1, int *y0, int *y1) {
  *x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  *x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  *y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  *y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
}

/* When SSAO probes the area around a point, it should only look outward from the
 * surface, not down into it. If a probe direction points into the surface, flip
 * it to point out (N is the surface's outward direction). */
static Vec3 hemisphere_flip(Vec3 dir, Vec3 N) {
  return (v3_dot(dir, N) < 0.f) ? v3_neg(dir) : dir;
}

/* ── §4 paint — turn a colour + brightness into a terminal colour + glyph ── */

static int g_256;

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);

  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline int clampi(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* rec709_luma — perceived brightness (green counts most, blue least). */
static float rec709_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* rgb_to_cube6 — snap display RGB to the nearest of the 216 terminal colours
 * (6 shades each of R/G/B) and return its slot, 0..215. */
static int rgb_to_cube6(float r, float g, float b) {
  int r5 = clampi((int)(r * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  int g5 = clampi((int)(g * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  int b5 = clampi((int)(b * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  return r5 * (CUBE_SIZE * CUBE_SIZE) + g5 * CUBE_SIZE + b5;
}

/* ramp_index — map a luma in [0,1] to a glyph index in the Bourke ramp. */
static int ramp_index(float luma) {
  return clampi((int)(luma * (BOURKE_LEN - 1) + 0.5f), 0, BOURKE_LEN - 1);
}

/*
 * Draw one screen cell given its final colour. The steps: adjust the colour so
 * it looks right to the eye, nudge it with the dither grid, pick the closest
 * terminal colour, choose a character by how bright it is, and make very bright
 * cells bold / very dark ones dim.
 *
 * We only do the gamma adjustment, not a fancy tone-map: the lighting here never
 * gets very bright or very dark, so a tone-map would just squash everything into
 * the dark characters. Plain gamma keeps the full range of characters in use,
 * which matters on a terminal with only a handful of brightness steps.
 */
static void paint_cell(int sx, int sy, Vec3 col) {
  Vec3 disp = v3(gamma_enc(col.x), gamma_enc(col.y), gamma_enc(col.z));

  float luma = rec709_luma(disp);
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith); /* dither breaks up flat banding */

  int pair = g_256 ? PAIR_CUBE_BASE + rgb_to_cube6(disp.x, disp.y, disp.z)
                   : PAIR_CUBE_BASE;
  int glyph = ramp_index(lum_d);

  int attr = (luma > BOLD_LUMA) ? A_BOLD : (luma < DIM_LUMA) ? A_DIM : A_NORMAL;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[glyph]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh — build the shapes out of triangles, once at startup ── */

typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;
typedef struct {
  int v[3];
} Triangle;
typedef struct {
  Vertex *verts;
  Triangle *tris;
  int nvert, ntri;
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/*
 * Add one flat rectangle (as two triangles) to a mesh. You give a starting
 * corner and two edge directions; it walks the four corners around the rectangle
 * and records which way the surface faces. The caller has to pick the two edge
 * directions in the right order, or the rectangle ends up facing inward and gets
 * skipped as a back face (tessellate_box notes the correct order per face).
 */
static void mesh_add_quad(Mesh *m, Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  int v0 = m->nvert;
  Vec3 p0 = origin;
  Vec3 p1 = v3_add(origin, e1);
  Vec3 p2 = v3_add(origin, v3_add(e1, e2));
  Vec3 p3 = v3_add(origin, e2);
  m->verts[m->nvert++] = (Vertex){p0, nrm, 0.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p1, nrm, 1.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p2, nrm, 1.f, 1.f};
  m->verts[m->nvert++] = (Vertex){p3, nrm, 0.f, 1.f};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

/*
 * Build a box centred at the origin. Each of the 6 faces gets its own 4 corners
 * (24 in total) instead of sharing the 8 box corners. Sharing would blend the
 * facing directions at the corners and make the box look rounded — but this demo
 * is all about flat faces and sharp edges, so we keep them crisp.
 */
static Mesh tessellate_box(float hx, float hy, float hz) {
  Mesh m;
  m.verts = malloc(24 * sizeof(Vertex));
  m.tris = malloc(12 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* +X (right):   e1×e2 = (+, 0, 0) */
  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(0, 2 * hy, 0), v3(0, 0, 2 * hz),
                v3(1, 0, 0));
  /* -X (left):    e1×e2 = (-, 0, 0) */
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(0, 2 * hy, 0), v3(0, 0, -2 * hz),
                v3(-1, 0, 0));
  /* +Y (top):     e1×e2 = (0, +, 0) */
  mesh_add_quad(&m, v3(-hx, hy, hz), v3(2 * hx, 0, 0), v3(0, 0, -2 * hz),
                v3(0, 1, 0));
  /* -Y (bottom):  e1×e2 = (0, -, 0) */
  mesh_add_quad(&m, v3(-hx, -hy, -hz), v3(2 * hx, 0, 0), v3(0, 0, 2 * hz),
                v3(0, -1, 0));
  /* +Z (front):   e1×e2 = (0, 0, +) */
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, 1));
  /* -Z (back):    e1×e2 = (0, 0, -) */
  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(-2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, -1));
  return m;
}

/* A single flat rectangle on its own — used for the floor. */
static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  Mesh m;
  m.verts = malloc(4 * sizeof(Vertex));
  m.tris = malloc(2 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;
  mesh_add_quad(&m, origin, e1, e2, nrm);
  return m;
}

/*
 * Build a sphere, like a globe made of rows (rings) and columns (segments). We
 * march down from the north pole to the south pole and around each ring, placing
 * a grid of points on the surface and joining them into triangles.
 *
 * Each corner's facing direction points straight out from the centre. Because
 * the renderer blends these directions across each triangle, the sphere shades
 * as a smooth curve rather than showing flat facets — so its contact shadow
 * curves smoothly around it too.
 */
static Mesh tessellate_sphere(float radius, int rings, int segs) {
  int n_verts = (rings + 1) * (segs + 1);
  int n_tris = rings * segs * 2;
  Mesh m;
  m.verts = malloc((size_t)n_verts * sizeof(Vertex));
  m.tris = malloc((size_t)n_tris * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int i = 0; i <= rings; i++) {
    float theta = (float)M_PI * (float)i / (float)rings;
    float st = sinf(theta), ct = cosf(theta);
    for (int j = 0; j <= segs; j++) {
      float phi = 2.f * (float)M_PI * (float)j / (float)segs;
      float x = radius * st * cosf(phi);
      float y = radius * ct;
      float z = radius * st * sinf(phi);
      Vec3 pos = v3(x, y, z);
      Vec3 nrm = v3(x / radius, y / radius, z / radius);
      float u = (float)j / (float)segs;
      float vv = (float)i / (float)rings;
      m.verts[m.nvert++] = (Vertex){pos, nrm, u, vv};
    }
  }

  for (int i = 0; i < rings; i++) {
    for (int j = 0; j < segs; j++) {
      int v00 = i * (segs + 1) + j;
      int v10 = (i + 1) * (segs + 1) + j;
      int v11 = (i + 1) * (segs + 1) + (j + 1);
      int v01 = i * (segs + 1) + (j + 1);
      m.tris[m.ntri++] = (Triangle){{v00, v10, v01}};
      m.tris[m.ntri++] = (Triangle){{v10, v11, v01}};
    }
  }
  return m;
}

/* ── §5b scene pieces — the small types the render passes below take ── */

/* One thing in the scene: its shape (mesh), its plain colour (albedo), and where
 * it sits in the world (model). The floor, the three steps, and the sphere are
 * each one of these. */
typedef struct {
  Mesh mesh;
  Vec3 albedo;
  Mat4 model;
} SceneObject;

/* The eye we view from. It holds where the eye is in the world plus the
 * ready-made transforms (built from the orbit angle and zoom) that move a point
 * from the world onto the screen. */
typedef struct {
  Mat4 view, proj, vp;
  Vec3 pos;
  float dist; /* how far back the eye sits — zoom */
  float yaw;  /* how far around the scene it has turned */
} Camera;

/* ── §6 first pass — draw the scene into per-cell buffers (no colour yet) ── */

/* Instead of lighting each triangle as we draw it, we first record, for every
 * screen cell, the facts a later step will need: the surface's world position,
 * which way it faces, its plain colour, and how far away it is. SSAO and the
 * lighting step then work from these buffers. We keep two distances: a quick one
 * for deciding what's in front, and a true (evenly-spaced) one that SSAO needs
 * to judge nearby surfaces correctly. (This split-into-passes idea is called
 * deferred shading.) */
typedef struct {
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];      /* where the surface is, in the world */
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];   /* which way it faces */
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];   /* its plain colour, before lighting */
  float zbuf[GBUF_MAX_H][GBUF_MAX_W];    /* quick depth, for deciding what's in front */
  float z_view[GBUF_MAX_H][GBUF_MAX_W];  /* true distance from the eye, used by SSAO */
  uint8_t valid[GBUF_MAX_H][GBUF_MAX_W]; /* did anything get drawn at this cell? */
} GBuffer;

static GBuffer g_gbuf;

static void gbuffer_clear(GBuffer *g, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g->zbuf[r][c] = 1.0f;
      g->z_view[r][c] = -CAM_FAR;
      g->valid[r][c] = 0;
    }
  }
}

/*
 * Draw one object into the per-cell buffers. For each of its triangles: move the
 * corners into place and onto the screen, drop triangles facing away, then for
 * every cell the triangle covers, work out the surface details there and store
 * them — but only if this triangle is closer than whatever was stored before.
 */
static void rasterize_object(const Mesh *mesh, Vec3 albedo, Mat4 mvp,
                             Mat4 model, Mat4 modelview, Mat4 norm_mat,
                             GBuffer *g, int cols, int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    /* Step 1: move each corner into world space and toward the screen. */
    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    float vz[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
      wpos[vi] = m4_pt(model, v->pos);
      wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
      vz[vi] = m4_pt(modelview, v->pos).z;
    }

    if (clip[0].w < CLIP_W_MIN && clip[1].w < CLIP_W_MIN &&
        clip[2].w < CLIP_W_MIN)
      continue; /* whole triangle behind the eye */

    /* Step 2: finish the screen positions, then skip it if it faces away. */
    float sx[3], sy[3], sz[3];
    clip_to_screen(clip, sx, sy, sz, cols, rows);
    if (is_back_facing(sx, sy))
      continue;

    int x0, x1, y0, y1;
    triangle_bbox(sx, sy, cols, rows, &x0, &x1, &y0, &y1);

    /* Step 3: for each covered cell, record its surface details. */
    for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
      for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (!bary_inside(b))
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z >= g->zbuf[py][px])
          continue; /* something nearer already here */

        g->zbuf[py][px] = z;
        g->z_view[py][px] = b[0] * vz[0] + b[1] * vz[1] + b[2] * vz[2];
        g->pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
        g->normal[py][px] =
            v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
        g->albedo[py][px] = albedo;
        g->valid[py][px] = 1;
      }
    }
  }
}

static void render_gbuffer(const SceneObject *objs, int n_objects,
                           const Camera *cam, GBuffer *g, int cols, int rows) {
  gbuffer_clear(g, cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mv = m4_mul(cam->view, objs[oi].model);
    Mat4 mvp = m4_mul(cam->proj, mv);
    Mat4 nmat = m4_normal_mat(objs[oi].model);
    rasterize_object(&objs[oi].mesh, objs[oi].albedo, mvp, objs[oi].model, mv,
                     nmat, g, cols, rows);
  }
}

/* ── §7 ssao — measure how shut-in each cell is, and smooth the result ── */

/*
 * §7.1 the probe directions.
 *
 * To measure how enclosed a point is, we shoot a handful of short feeler
 * directions out around it and see how many bump into nearby surfaces. We
 * pre-make several sets of these directions; neighbouring cells use different
 * sets, and the later blur averages neighbours together, so the result looks
 * smooth instead of speckled.
 *
 * The feelers are mostly short, with a few longer ones: most stay very close to
 * catch tight crevices, a few reach out toward the full radius. That's why snug
 * corners darken more than open flat areas.
 *
 * We build them with a simple repeatable random generator so the directions come
 * out the same on every run.
 */
static Vec3 k_ssao[SSAO_KERNEL_VARIANTS][SSAO_SAMPLES];

static unsigned lcg_step(unsigned *s) {
  *s = *s * 1664525u + 1013904223u;
  return *s;
}
static float lcg_unit(unsigned *s) {
  return (lcg_step(s) >> 8) / (float)0x01000000;
}

static void ssao_init_kernel(void) {
  unsigned seed = 0xC0FFEE5Au;

  for (int v = 0; v < SSAO_KERNEL_VARIANTS; v++) {
    for (int i = 0; i < SSAO_SAMPLES; i++) {

      /* (1) pick a random direction: keep guessing random points in a cube
       *     until one lands inside the unit ball, then point at it. */
      float dx, dy, dz, len2;
      do {
        dx = 2.f * lcg_unit(&seed) - 1.f;
        dy = 2.f * lcg_unit(&seed) - 1.f;
        dz = 2.f * lcg_unit(&seed) - 1.f;
        len2 = dx * dx + dy * dy + dz * dz;
      } while (len2 > 1.f || len2 < 1e-6f);

      float inv = 1.f / sqrtf(len2);
      Vec3 dir = v3(dx * inv, dy * inv, dz * inv);

      /* (2) make the early feelers short and later ones longer. */
      float t = (float)i / (float)SSAO_SAMPLES;
      float scale = 0.1f + 0.9f * t * t;

      k_ssao[v][i] = v3_scale(dir, scale);
    }
  }
}

/* §7.2 the darkening amount per cell — before and after smoothing. */

static float g_ao[GBUF_MAX_H][GBUF_MAX_W];      /* straight from the probes, speckled */
static float g_ao_blur[GBUF_MAX_H][GBUF_MAX_W]; /* after averaging neighbours, smooth */

/* Follow one feeler out from point P and ask: is there a surface in the way? We
 * step to the feeler's tip, find which screen cell it lands on, and compare its
 * distance with what's drawn there. Reports whether that cell blocks the feeler,
 * and how much this answer should count (a far-away surface counts for less, and
 * something off-screen counts for nothing). center_view_z is P's own distance,
 * used to fade out far blockers. */
static float ssao_sample(const GBuffer *g, Vec3 P, Vec3 dir, float radius,
                         float center_view_z, Mat4 vp, Mat4 view, int cols,
                         int rows, bool *occluded) {
  *occluded = false;

  Vec3 S = v3_add(P, v3_scale(dir, radius)); /* the feeler's tip, out in the world */

  Vec4 clip = m4_mul_v4(vp, v4(S.x, S.y, S.z, 1.f));
  if (clip.w < CLIP_W_MIN)
    return 0.f;
  int ix = (int)((clip.x / clip.w + 1.f) * 0.5f * (float)cols);
  int iy = (int)((-clip.y / clip.w + 1.f) * 0.5f * (float)rows);
  if (ix < 0 || ix >= cols || ix >= GBUF_MAX_W || iy < 0 || iy >= rows ||
      iy >= GBUF_MAX_H)
    return 0.f;
  if (!g->valid[iy][ix])
    return 0.f;

  /* a blocker much nearer or farther than P shouldn't count — fade it by how
   * far off in distance it is, and ignore it entirely past the radius */
  float dz = fabsf(center_view_z - g->z_view[iy][ix]);
  float attn = 1.f - dz / radius;
  if (attn <= 0.f)
    return 0.f;

  /* blocked if the surface drawn here sits in front of the feeler's tip */
  float s_view_z = m4_pt(view, S).z;
  if (g->z_view[iy][ix] > s_view_z + SSAO_BIAS)
    *occluded = true;
  return attn;
}

/*
 * §7.3 the main loop: for each visible cell, fan its feelers out around the way
 * the surface faces, ask each whether something blocks it, and combine the
 * answers into one "how shut-in is this" number (1 = open, 0 = fully boxed in).
 */
static void ssao_pass(const GBuffer *g, Mat4 vp, Mat4 view, float radius,
                      int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g->valid[r][c]) {
        g_ao[r][c] = 1.f;
        continue;
      }

      /* (a) where this cell's surface is, and which way it faces */
      Vec3 P = g->pos[r][c];
      Vec3 N = g->normal[r][c];

      /* (b) pick which set of feelers to use, alternating across a 2×2 tile */
      int variant = (c & 1) | ((r & 1) << 1);

      /* (c) tally up the blocked feelers against the total that counted */
      float occlude_w = 0.f;
      float total_w = 0.f;

      for (int i = 0; i < SSAO_SAMPLES; i++) {
        Vec3 dir = hemisphere_flip(k_ssao[variant][i], N);
        bool occluded;
        float w = ssao_sample(g, P, dir, radius, g->z_view[r][c], vp, view, cols,
                              rows, &occluded);
        total_w += w;
        if (occluded)
          occlude_w += w;
      }

      /* (d) the darkening amount; treat "no feeler counted" as fully open */
      float ao = (total_w > 1e-6f) ? (1.f - occlude_w / total_w) : 1.f;
      g_ao[r][c] = clamp01(ao);
    }
  }
}

/*
 * §7.4 smooth the result by replacing each cell with the average of itself and
 * its 8 neighbours. This evens out the speckle left by neighbouring cells using
 * different feeler sets. Empty cells are left out of the average so the
 * darkening doesn't smear past an object's outline onto the background.
 */
static void ssao_blur(const GBuffer *g, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g->valid[r][c]) {
        g_ao_blur[r][c] = 1.f;
        continue;
      }

      float sum = 0.f;
      int count = 0;
      for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
          int rr = r + dr, cc = c + dc;
          if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
            continue;
          if (!g->valid[rr][cc])
            continue;
          sum += g_ao[rr][cc];
          count++;
        }
      }
      g_ao_blur[r][c] = (count > 0) ? (sum / (float)count) : 1.f;
    }
  }
}

/* ── §8 lighting — work out the final colour of every cell ── *
 *
 * Each cell's colour adds up three things: a soft fill light that reaches
 * everywhere (ambient), the direct sun where it actually shines on the surface
 * (diffuse), and a bright highlight where the sun glints toward the eye
 * (specular).
 *
 * The AO darkening is applied ONLY to the soft fill light, not to the direct
 * sun. That's the whole point: a crevice still catches direct sunlight, but
 * gets less of the bounced-around fill light — so it reads as gently shaded,
 * not blacked out. Dimming the sun too would also flatten the with-AO /
 * without-AO comparison the demo is built around. */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W]; /* the lit colour per cell */

static void render_lightpass(const GBuffer *g, Vec3 cam_pos, bool use_ao,
                             int cols, int rows) {
  Vec3 sun_dir = v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]);
  Vec3 sun_col = v3(SUN_COL[0], SUN_COL[1], SUN_COL[2]);
  Vec3 sky = v3(AMBIENT_SKY[0], AMBIENT_SKY[1], AMBIENT_SKY[2]);
  Vec3 ground = v3(AMBIENT_GROUND[0], AMBIENT_GROUND[1], AMBIENT_GROUND[2]);
  Vec3 L = v3_norm(v3_neg(sun_dir));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g->valid[r][c]) {
        g_light[r][c] = v3(0, 0, 0);
        continue;
      }

      Vec3 P = g->pos[r][c];
      Vec3 N = g->normal[r][c];
      Vec3 albedo = g->albedo[r][c];
      float ao = use_ao ? g_ao_blur[r][c] : 1.f;

      /* the soft fill: a cool sky tint on surfaces facing up, a warm bounced
       * tint on those facing down — gentler than one flat fill colour */
      float up = clamp01(N.y * 0.5f + 0.5f);
      Vec3 acol = v3(ground.x + (sky.x - ground.x) * up,
                     ground.y + (sky.y - ground.y) * up,
                     ground.z + (sky.z - ground.z) * up);
      Vec3 amb = v3(acol.x * albedo.x * ao, acol.y * albedo.y * ao,
                    acol.z * albedo.z * ao);

      float diff = fmaxf(0.f, v3_dot(N, L));
      Vec3 dif = v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
                    albedo.z * sun_col.z * diff);

      /* Specular only where the sun actually reaches the surface (N·L > 0);
       * otherwise a face turned away from the light could still flash a stray
       * highlight when its half-vector lined up by chance. */
      Vec3 V = v3_norm(v3_sub(cam_pos, P));
      Vec3 H = v3_norm(v3_add(L, V));
      float spec = (diff > 0.f) ? powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN
                                : 0.f;
      Vec3 sp = v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);

      Vec3 sum = v3_add(v3_add(amb, dif), sp);
      g_light[r][c] =
          v3(fminf(1.f, sum.x), fminf(1.f, sum.y), fminf(1.f, sum.z));
    }
  }
}

/* ── §9 scene — everything the world is made of, and how it changes ── */

/* The three things the 'a' key cycles between: just the darkening map on its own,
 * the full lighting without it, and the full lighting with it. */
typedef enum {
  MODE_AO_ONLY = 0,
  MODE_LIT_NO_AO,
  MODE_LIT_WITH_AO,
  MODE_COUNT,
} Mode;

static const char *k_mode_names[MODE_COUNT] = {
    "AO_ONLY",
    "LIT_NO_AO",
    "LIT_WITH_AO",
};

/* The whole world in one place: the things in it, the eye looking at them, and
 * the few settings the keys change. The big working buffers (the per-cell
 * buffers, the darkening maps, the lit colours) are kept separately, not here,
 * because they're scratch space the drawing code owns. */
typedef struct {
  SceneObject objects[N_OBJECTS]; /* floor, three steps, sphere        */
  Camera cam;                     /* the eye looking at them           */

  float ssao_radius; /* how far the darkening reaches — the [ and ] keys */
  Mode mode;         /* which of the three views to show — the 'a' key   */
  bool paused;       /* is the orbit frozen? — the space key             */

  int scene_cols, scene_rows; /* size of the drawing area, in cells    */
} Scene;

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  /* Work out the width-to-height ratio from pixels, not cells: a terminal cell
   * is taller than it is wide (8×16), so counting cells would stretch the view. */
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->cam.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/*
 * Place the eye for the current orbit angle and zoom, then rebuild the combined
 * transform from it. Run every frame as the view turns and whenever you zoom, so
 * the drawing, darkening, and lighting all agree on where the eye is.
 */
static void scene_rebuild_view(Scene *s) {
  float r = s->cam.dist;
  s->cam.pos = v3(sinf(s->cam.yaw) * r, CAM_EYE_Y, cosf(s->cam.yaw) * r);
  s->cam.view = m4_lookat(s->cam.pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
  s->cam.vp = m4_mul(s->cam.proj, s->cam.view);
}

/*
 * Build the scene from scratch: the floor, three steps stacked into a pyramid,
 * and the sphere on top. The heights are picked so each step rests exactly on
 * the one below with no gap or overlap, and the sphere sits right on the top
 * step.
 */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->objects[i].mesh);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->mode = MODE_LIT_WITH_AO;
  s->paused = true;       /* open paused so the 'a' AO compare is easy first thing */
  s->ssao_radius = SSAO_RADIUS_DEF;
  s->cam.dist = CAM_DIST;
  s->cam.yaw = 0.55f;     /* 3/4 view — shows the front, a side, and the step tops */

  /* OBJ_FLOOR — large quad at y = 0, normal up. */
  s->objects[OBJ_FLOOR].mesh = tessellate_quad(
      v3(-FLOOR_HALF_X, 0.f, FLOOR_HALF_Z), v3(2 * FLOOR_HALF_X, 0.f, 0.f),
      v3(0.f, 0.f, -2 * FLOOR_HALF_Z), v3(0.f, 1.f, 0.f));
  s->objects[OBJ_FLOOR].albedo = v3(0.42f, 0.46f, 0.50f); /* slate */
  s->objects[OBJ_FLOOR].model = m4_identity();

  /* OBJ_STEP0..2 — same warm sandstone, decreasing footprints.
   * Deeper than a literal sandstone so the lit faces don't blow out to near-
   * white on the terminal's bright end. */
  Vec3 stone = v3(0.60f, 0.46f, 0.30f);

  s->objects[OBJ_STEP0].mesh = tessellate_box(STEP0_HX, STEP_HY, STEP0_HX);
  s->objects[OBJ_STEP0].albedo = stone;
  s->objects[OBJ_STEP0].model = m4_translate(0.f, STEP0_CY, 0.f);

  s->objects[OBJ_STEP1].mesh = tessellate_box(STEP1_HX, STEP_HY, STEP1_HX);
  s->objects[OBJ_STEP1].albedo = stone;
  s->objects[OBJ_STEP1].model = m4_translate(0.f, STEP1_CY, 0.f);

  s->objects[OBJ_STEP2].mesh = tessellate_box(STEP2_HX, STEP_HY, STEP2_HX);
  s->objects[OBJ_STEP2].albedo = stone;
  s->objects[OBJ_STEP2].model = m4_translate(0.f, STEP2_CY, 0.f);

  /* OBJ_SPHERE — cream, sits on step 2's top. */
  s->objects[OBJ_SPHERE].mesh =
      tessellate_sphere(SPHERE_RADIUS, SPHERE_RINGS, SPHERE_SEGS);
  s->objects[OBJ_SPHERE].albedo = v3(0.68f, 0.61f, 0.50f); /* deeper cream */
  s->objects[OBJ_SPHERE].model = m4_translate(0.f, SPHERE_CY, 0.f);

  scene_rebuild_proj(s, total_cols, s->scene_rows);
  scene_rebuild_view(s);
}

/* The one thing that changes on its own over time: the eye creeps a little
 * further around its orbit each frame. Nothing else moves, and pausing freezes
 * it. The darkening is recomputed every frame for the current view, so when
 * paused the picture holds perfectly still. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->cam.yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  scene_rebuild_view(s);
}

/* The colour to draw at one cell for the current view. The main loop has already
 * run whichever steps this view needs, so the right values are waiting here. */
static Vec3 mode_to_rgb(const GBuffer *g, Mode mode, int r, int c) {
  if (!g->valid[r][c])
    return v3(0, 0, 0);

  switch (mode) {
  case MODE_AO_ONLY: {
    float a = g_ao_blur[r][c];
    return v3(a, a, a);
  }
  case MODE_LIT_NO_AO:
  case MODE_LIT_WITH_AO:
    return g_light[r][c];
  default:
    return v3(0, 0, 0);
  }
}

/* ── §10 screen — draw every cell, then the readout on top ── */

static void render_scene(const Scene *s, const GBuffer *g) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g->valid[r][c])
        continue;
      Vec3 col = mode_to_rgb(g, s->mode, r, c);
      paint_cell(c, r, col);
    }
  }
}

/*
 * Draw the text overlay: a yellow status line along the top, a cyan key-hint
 * along the bottom, and three dimmer rows that explain the current view and
 * point out what to look for.
 */
static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int total_tris = 0;
  for (int i = 0; i < N_OBJECTS; i++)
    total_tris += s->objects[i].mesh.ntri;

  /* Row 0: title + status. */
  char status[140];
  snprintf(status, sizeof status,
           " %5.1f fps  mode:%s  K=%d  R=%.2f  zoom:%.1f  tris:%d  %s ", fps,
           k_mode_names[s->mode], SSAO_SAMPLES, (double)s->ssao_radius,
           (double)s->cam.dist, total_tris, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " SSAO · STEPPED PYRAMID ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Educational rows. */
  attron(COLOR_PAIR(PAIR_HUD));

  int total_dirs = SSAO_SAMPLES * SSAO_KERNEL_VARIANTS;
  mvprintw(hr + 0, 1,
           "samples=%d (across %d variants = %d distinct dirs)   blur=3x3   "
           "bias=%.4f",
           SSAO_SAMPLES, SSAO_KERNEL_VARIANTS, total_dirs, (double)SSAO_BIAS);

  const char *explain = "";
  switch (s->mode) {
  case MODE_AO_ONLY:
    explain = "AO_ONLY: grayscale crevice map. White=open. Dark=corners (4 "
              "features).";
    break;
  case MODE_LIT_NO_AO:
    explain =
        "LIT_NO_AO: full Blinn-Phong, no AO. Sphere looks pasted on the step.";
    break;
  case MODE_LIT_WITH_AO:
    explain = "LIT_WITH_AO: ambient * AO + direct. Pyramid + sphere settle "
              "into floor.";
    break;
  default:
    break;
  }
  mvprintw(hr + 1, 1, "%s", explain);

  mvprintw(hr + 2, 1,
           "Look for: floor halo, two horizontal step bands, "
           "circular halo under the sphere");

  attroff(COLOR_PAIR(PAIR_HUD));

  /* Cyan hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  a:mode  [/]:radius  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §11 app — set up the terminal, run the loop, handle keys ── */

/* Everything the running program needs to keep around: the scene, the terminal
 * size, and two flags the OS sets for us — one to quit, one to notice the
 * window was resized. */
typedef struct {
  Scene scene;
  int total_cols;
  int total_rows;
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

static void screen_init(void) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt drawing (avoids tearing) */
  color_init();
}

static void app_do_resize(App *app) {
  endwin();
  refresh();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);
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
    scene_init(s, app->total_cols, app->total_rows);
    break;
  case 'a':
  case 'A':
    s->mode = (Mode)((s->mode + 1) % MODE_COUNT);
    break;
  case '[':
    s->ssao_radius -= SSAO_RADIUS_STEP;
    if (s->ssao_radius < SSAO_RADIUS_MIN)
      s->ssao_radius = SSAO_RADIUS_MIN;
    break;
  case ']':
    s->ssao_radius += SSAO_RADIUS_STEP;
    if (s->ssao_radius > SSAO_RADIUS_MAX)
      s->ssao_radius = SSAO_RADIUS_MAX;
    break;
  case '=':
  case '+':
    s->cam.dist -= CAM_ZOOM_STEP;
    if (s->cam.dist < CAM_DIST_MIN)
      s->cam.dist = CAM_DIST_MIN;
    scene_rebuild_view(s);
    break;
  case '-':
  case '_':
    s->cam.dist += CAM_ZOOM_STEP;
    if (s->cam.dist > CAM_DIST_MAX)
      s->cam.dist = CAM_DIST_MAX;
    scene_rebuild_view(s);
    break;
  default:
    break;
  }
  return true;
}

/*
 * Run only the steps the current view needs. The first step (drawing the scene
 * into the per-cell buffers) always runs; the darkening and the lighting are
 * each skipped when the view doesn't show them. Skipping keeps the toggle keys
 * responsive, and shows that the darkening and the lighting are independent
 * steps you can turn on or off separately.
 */
static void dispatch_passes(Scene *s) {
  render_gbuffer(s->objects, N_OBJECTS, &s->cam, &g_gbuf, s->scene_cols,
                 s->scene_rows);

  if (s->mode != MODE_LIT_NO_AO) {
    ssao_pass(&g_gbuf, s->cam.vp, s->cam.view, s->ssao_radius, s->scene_cols,
              s->scene_rows);
    ssao_blur(&g_gbuf, s->scene_cols, s->scene_rows);
  }

  if (s->mode != MODE_AO_ONLY) {
    bool use_ao = (s->mode == MODE_LIT_WITH_AO);
    render_lightpass(&g_gbuf, s->cam.pos, use_ao, s->scene_cols, s->scene_rows);
  }
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  ssao_init_kernel();

  App *app = &g_app;
  app->running = 1;

  screen_init();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* If the window was resized, rebuild the scene for the new size. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* How long since the last frame? Cap it so a hiccup can't jump the orbit. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* One frame, in order: move, time it, draw, then read input. */
    scene_tick(&app->scene, dt_sec); /* nudge the eye along its orbit */

    /* Update the fps number, then sleep off any spare time to hold the rate. */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    /* Draw: run the steps this view needs, paint the cells, add the overlay. */
    Scene *s = &app->scene;
    erase();
    dispatch_passes(s);
    render_scene(s, &g_gbuf);
    hud_draw(s, fps_display);
    screen_present();

    /* Read one keypress, if any, and act on it. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&app->scene.objects[i].mesh);

  endwin();
  return 0;
}
