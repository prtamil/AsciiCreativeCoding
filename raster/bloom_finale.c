/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bloom_finale.c — the folder's capstone. One frame runs three image tricks
 * back to back: deferred shading, SSAO contact shadows, and bloom. The scene is
 * a dark floor, two sandstone cubes, and a glowing orb; the orb is brighter than
 * the screen can show, and that extra brightness leaks out around it as a soft
 * halo (the bloom). SSAO darkens the creases where the cubes meet the floor.
 *
 * Keys: b bloom · a SSAO · +/- zoom · space pause · r reset · q/ESC quit
 * Builds on: deferred_rendering_pipeline.c (the G-buffer), ssao_pipeline.c (AO).
 * Ideas from: Reinhard tone-map (SIGGRAPH '02); separable Gaussian bloom
 *   (LearnOpenGL, "Bloom"); Floyd–Steinberg dithering; Blinn-Phong (1977).
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/bloom_finale.c -o bloom -lncurses -lm
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

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_ROWS = 5,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 G-buffer dimensions (static; sized for a large terminal). */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 view geometry — eye orbits the scene at fixed elevation. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f
/* Anything closer to the eye than this (or behind it) is thrown away. The step
 * that makes far things smaller divides by distance, and that blows up for a
 * point sitting right on top of the camera. */
#define CLIP_W_MIN 0.001f

#define CAM_DIST 5.5f
#define CAM_DIST_MIN 3.0f
#define CAM_DIST_MAX 10.0f
#define CAM_ZOOM_STEP 0.4f
#define CAM_EYE_Y 2.4f
#define CAM_LOOK_Y 0.5f
#define CAM_ORBIT_RAD_PER_SEC 0.18f

#define CELL_W 8
#define CELL_H 16

/* §1.4 lighting — the glowing orb is the main light: it sits at one spot and
 * throws light outward (a "point" light). A dim, far-away "sun" pointing one
 * fixed direction, plus a little all-over ambient, keep the sides facing away
 * from the orb from going pure black. */
static const float SUN_DIR[3] = {-0.55f, -0.65f, 0.30f}; /* fill direction   */
static const float SUN_COL[3] = {0.95f, 0.85f, 0.65f};   /* fill base colour */
static const float AMBIENT_COL[3] = {0.18f, 0.20f, 0.25f};
#define FILL_GAIN 0.30f /* scales the directional sun down to a dim fill */
#define SHININESS 24.0f
#define SPEC_GAIN 0.30f

/* The orb doubles as a real light that lights the scene. Its colour is scaled
 * up by intensity when used, and it gets dimmer the farther a surface is from
 * it — so it lays down a bright pool that fades across the floor. That fade is a
 * strong depth cue a terminal shows off far better than flat, even lighting. */
static const float POINT_LIGHT_COL[3] = {1.00f, 0.62f, 0.30f}; /* warm, orb-like */
#define POINT_LIGHT_INTENSITY 6.0f
#define POINT_ATTEN_LINEAR 0.10f
#define POINT_ATTEN_QUAD 0.25f
/* Just a visual aid: the sun has a direction but no position, so we drop a glyph
 * this far from the scene centre in the direction the sunlight comes from, to
 * eyeball that the shading lines up. */
#define SUN_MARKER_DIST 4.5f

/* §1.5 SSAO — fake soft shadows in nooks and crannies (corners, contact lines).
 *
 * Same recipe as ssao_pipeline.c: around each surface point, poke 12 little test
 * points into the air just above it and count how many are blocked by nearby
 * geometry — more blocked means a tighter nook, so darken it. We keep 4 slightly
 * different sets of test points and pick one per cell so the pattern doesn't
 * line up into stripes, then a 3×3 average smooths the leftover speckle. Only
 * the ambient (fill-everywhere) light gets darkened. */
#define SSAO_SAMPLES 12
#define SSAO_KERNEL_VARIANTS 4
#define SSAO_RADIUS 0.45f
#define SSAO_BIAS 0.0008f
/* The closest test point sits this fraction of the way out; the rest fan out
 * faster (as t²) so most of them cluster near the surface, where blocking
 * actually matters. */
#define SSAO_SAMPLE_MIN 0.1f

/* §1.6 bloom — the soft glow around very bright things.
 *
 * THRESHOLD — how bright a pixel must be to glow. Set just above full white so
 *   only the orb (which is brighter than white) blooms; normal lit surfaces
 *   stay calm.
 * INTENSITY — how much of the blurred glow we add back on top. 1.0 is strong.
 * KERNEL    — the blur shape: a bell curve (Gaussian) of weights, run once
 *   sideways and once up/down (cheaper than a full 2-D blur, same result).
 *   13 weights spread the glow ~6 cells past the orb. An earlier 7-weight
 *   version was too tight to read as a glow on the coarse terminal grid — the
 *   halo barely cleared the orb's edge. Wider + more weights gives a real,
 *   visibly-fading halo. */
#define BLOOM_THRESHOLD 1.00f
#define BLOOM_INTENSITY 1.00f
#define BLOOM_RADIUS 6                    /* weights each side */
#define BLOOM_TAPS (2 * BLOOM_RADIUS + 1) /* = 13             */
static const float BLOOM_KERNEL[BLOOM_TAPS] = {
    /* bell-curve weights (width 3.0), scaled so they add up to 1.0 — blurring
     * moves brightness around without adding or losing any. */
    0.0185f, 0.0342f, 0.0563f, 0.0831f, 0.1097f, 0.1296f, 0.1370f,
    0.1296f, 0.1097f, 0.0831f, 0.0563f, 0.0342f, 0.0185f};

/* §1.7 scene geometry — floor + two cubes + one glowing orb.
 *
 *                     ●  ← orb (emissive, the bloom showcase)
 *                  ┌──┐         ┌──┐
 *                  │AA│         │BB│   cubes (sandstone)
 *                  └──┘         └──┘
 *               ───────────────────  floor (slate)                 */
#define FLOOR_HALF_X 3.0f
#define FLOOR_HALF_Z 3.0f

#define CUBE_HALF 0.45f
#define CUBE_A_CX -1.10f
#define CUBE_A_CY (CUBE_HALF)
#define CUBE_A_CZ 0.40f
#define CUBE_B_CX 1.10f
#define CUBE_B_CY (CUBE_HALF)
#define CUBE_B_CZ -0.30f

#define ORB_RADIUS 0.40f
#define ORB_RINGS 12
#define ORB_SEGS 18
#define ORB_CX 0.0f
#define ORB_CY 0.95f
#define ORB_CZ 0.05f
/* The orb's own glow. These numbers go above 1.0 (brighter than the screen can
 * show) on purpose — that's the "extra" brightness bloom looks for. A value of
 * 1.0 would just be plain white and wouldn't bloom. */
static const float ORB_EMISSIVE[3] = {3.20f, 1.80f, 0.80f};

enum {
  OBJ_FLOOR = 0,
  OBJ_CUBE_A,
  OBJ_CUBE_B,
  OBJ_ORB,
  N_OBJECTS,
};

/* §1.8 character ramp — darkest to brightest, 18 characters, no faint ones.
 *
 * Brightness is drawn by character: a space for black, up through busier
 * characters to '#'/'$' for the brightest. The usual long 92-character ramp has
 * a bunch of nearly-blank characters at the dim end ('.', '`', ',', etc.). At
 * the faint brightness a bloom halo actually has (~0.05-0.15) those look like
 * scattered dots, not a smooth glow.
 *
 * So this short ramp drops the blank-looking characters entirely. Its dimmest
 * non-space is ':' — two clear dots — so even a faint halo cell shows a visible
 * mark instead of a near-invisible speck. The bright end keeps the dense
 * '%&@#$'.
 *
 * Trade-off: fewer brightness steps (18 vs 92) — but the 216-colour cube
 * already limits us to about 6 steps per colour anyway, so 18 is plenty. */
static const char k_bourke[] = " .:;~-+*coxOQ0%&@#$";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 dithering — done in render_scene (§11). We only have a few colours and
 * characters to work with, so a smooth gradient would otherwise show ugly steps.
 * The fix (Floyd–Steinberg): when we round a cell to the nearest colour we keep
 * track of how far off we were, and pass that leftover on to the next-door cells
 * so the error averages out. No knob to tune — it just smooths the banding. */

/* §1.10 ncurses pair IDs — 216 cube + yellow HUD + cyan hint. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.11 turning a colour into a terminal cell — see paint_cell (§4). */
#define CUBE_LEVELS 6             /* 6 steps per channel → 6×6×6 = 216 colours   */
#define DISPLAY_GAMMA 2.2f        /* screen-brightness correction, applied last   */
#define BOLD_LUMA_THRESHOLD 0.85f /* brighter than this → draw the cell in bold   */

/* ── §2 clock — reading the time and sleeping ──────────────────────────── */

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

/* ── §3 math — vectors and 4×4 matrices ────────────────────────────────── */

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
static inline float v3_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
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

/* Builds the "stand at eye, look toward at, with up roughly up" camera
 * transform — the usual way to point a camera at something. */
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

/* ── §4 paint — turning a colour into one terminal cell ────────────────── */

static int g_256;

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);

  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) {
  return powf(clamp01(x), 1.f / DISPLAY_GAMMA);
}

/* Squashes a too-bright colour down into what the screen can show: first the
 * x/(1+x) curve (gently rolls big values toward white instead of clipping), then
 * a brightness correction for the display. This is the one and only place the
 * over-bright values get brought back into range. */
static inline Vec3 tonemap_encode(Vec3 hdr) {
  return v3(gamma_enc(reinhard(hdr.x)), gamma_enc(reinhard(hdr.y)),
            gamma_enc(reinhard(hdr.z)));
}

/* Snaps a 0..1 value to the nearest of `levels` evenly-spaced steps, and reports
 * back through *err how far the snap missed by — that leftover is what dithering
 * hands to the neighbouring cells. */
static inline int quantize_level(float v, int levels, float *err) {
  int lvl = (int)(v * (float)(levels - 1) + 0.5f);
  if (lvl < 0)
    lvl = 0;
  if (lvl > levels - 1)
    lvl = levels - 1;
  *err = v - (float)lvl / (float)(levels - 1);
  return lvl;
}

/* Scratch for the dithering: a grid of "leftover error so far" per cell, one
 * grid for each thing we round — red, green, blue, and overall brightness (which
 * picks the character). render_scene wipes them each frame, then every cell adds
 * its rounding miss into the cells it hasn't drawn yet, so gradients dither
 * smoothly instead of stepping. */
static float g_err_r[GBUF_MAX_H][GBUF_MAX_W];
static float g_err_g[GBUF_MAX_H][GBUF_MAX_W];
static float g_err_b[GBUF_MAX_H][GBUF_MAX_W];
static float g_err_y[GBUF_MAX_H][GBUF_MAX_W];

/* Hands this cell's rounding leftover to the cells we'll draw next, split with
 * the classic Floyd–Steinberg weights (7,3,5,1 out of 16):
 *        .   X   7        (X = this cell, just drawn)
 *        3   5   1        (the row below)
 * Only cells ahead of us get a share, so drawing left→right, top→bottom means
 * each cell has collected all its handed-down error before we reach it. */
static inline void diffuse_error(float buf[GBUF_MAX_H][GBUF_MAX_W], int sx,
                                 int sy, int cols, int rows, float err) {
  if (sx + 1 < cols)
    buf[sy][sx + 1] += err * (7.f / 16.f);
  if (sy + 1 < rows) {
    if (sx - 1 >= 0)
      buf[sy + 1][sx - 1] += err * (3.f / 16.f);
    buf[sy + 1][sx] += err * (5.f / 16.f);
    if (sx + 1 < cols)
      buf[sy + 1][sx + 1] += err * (1.f / 16.f);
  }
}

/* Turns a red/green/blue step (each 0..5) into the ncurses colour-pair number
 * for that colour in the 216-colour set. */
static inline int cube_pair(int r5, int g5, int b5) {
  return PAIR_CUBE_BASE + r5 * (CUBE_LEVELS * CUBE_LEVELS) + g5 * CUBE_LEVELS +
         b5;
}

/* Draws one terminal cell from a colour — the last stop in the whole pipeline.
 * Reads top to bottom: squash the colour into screen range → measure its
 * brightness → snap red/green/blue to the colour set and brightness to a
 * character, mixing in the dither leftovers and passing the new leftovers on →
 * draw it. cols/rows just bound where the leftover error may be pushed. */
static void paint_cell(int sx, int sy, int cols, int rows, Vec3 col) {
  Vec3 rgb = tonemap_encode(col);
  float luma = v3_luma(rgb); /* un-dithered, used for the bold decision below */

  /* snap each value, folding in the error handed down by earlier cells */
  float er, eg, eb, ey;
  int r5 = quantize_level(clamp01(rgb.x + g_err_r[sy][sx]), CUBE_LEVELS, &er);
  int g5 = quantize_level(clamp01(rgb.y + g_err_g[sy][sx]), CUBE_LEVELS, &eg);
  int b5 = quantize_level(clamp01(rgb.z + g_err_b[sy][sx]), CUBE_LEVELS, &eb);
  int idx = quantize_level(clamp01(luma + g_err_y[sy][sx]), BOURKE_LEN, &ey);

  /* pass each snap's leftover on to the cells we'll draw next */
  diffuse_error(g_err_r, sx, sy, cols, rows, er);
  diffuse_error(g_err_g, sx, sy, cols, rows, eg);
  diffuse_error(g_err_b, sx, sy, cols, rows, eb);
  diffuse_error(g_err_y, sx, sy, cols, rows, ey);

  int pair = g_256 ? cube_pair(r5, g5, b5) : PAIR_CUBE_BASE;

  /* Deliberately no "dim" attribute on dark cells: most terminals halve its
   * brightness, which made the faint bloom halo (brightness ~0.05-0.15) nearly
   * vanish. Only the bright orb goes bold, so it still pops over its halo. */
  int attr = (luma > BOLD_LUMA_THRESHOLD) ? A_BOLD : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh — building the shapes at startup ──────────────────────────── */

/* Vertex — one corner of a triangle, given in the object's own coordinates (the
 * object's model matrix places it into the world later). Fields:
 *   pos    — where the corner is (object's own units; meshes are built ~1 wide)
 *   normal — which way the surface faces here. Points straight out from the
 *            centre for the sphere (so it looks smooth) and straight out from
 *            each flat face for the box (so it looks faceted); the lighting
 *            blends it across the triangle, corner to corner.
 * No texture coordinates here — this demo doesn't use any images. */
typedef struct {
  Vec3 pos;
  Vec3 normal;
} Vertex;

/* Triangle — one face, stored as three slot-numbers into its Mesh's vertex list
 * (so shared corners live once and faces just point at them). Its corners are
 * listed counter-clockwise as seen from the front; the "don't draw faces turned
 * away from us" check in rasterize_object relies on that ordering. */
typedef struct {
  int v[3];
} Triangle;

/* Mesh — a whole shape: a list of corners plus the triangles that connect them.
 * Allocated on the heap (one of the allowed mallocs — built once in scene_init,
 * freed by mesh_free) because each shape needs a different size: a flat quad is
 * 4 corners / 2 triangles, the sphere far more.
 *   verts / tris — the owned lists (freed by mesh_free)
 *   nvert / ntri — how many are filled in; also the write position while building */
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

static void mesh_add_quad(Mesh *m, Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  int v0 = m->nvert;
  Vec3 p0 = origin;
  Vec3 p1 = v3_add(origin, e1);
  Vec3 p2 = v3_add(origin, v3_add(e1, e2));
  Vec3 p3 = v3_add(origin, e2);
  m->verts[m->nvert++] = (Vertex){p0, nrm};
  m->verts[m->nvert++] = (Vertex){p1, nrm};
  m->verts[m->nvert++] = (Vertex){p2, nrm};
  m->verts[m->nvert++] = (Vertex){p3, nrm};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

/* Builds a box centred on the origin. 24 corners (4 per face × 6 faces) rather
 * than 8 shared ones, so each face can face its own flat direction and look
 * crisp instead of rounded. */
static Mesh tessellate_box(float hx, float hy, float hz) {
  Mesh m;
  m.verts = malloc(24 * sizeof(Vertex));
  m.tris = malloc(12 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(0, 2 * hy, 0), v3(0, 0, 2 * hz),
                v3(1, 0, 0));
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(0, 2 * hy, 0), v3(0, 0, -2 * hz),
                v3(-1, 0, 0));
  mesh_add_quad(&m, v3(-hx, hy, hz), v3(2 * hx, 0, 0), v3(0, 0, -2 * hz),
                v3(0, 1, 0));
  mesh_add_quad(&m, v3(-hx, -hy, -hz), v3(2 * hx, 0, 0), v3(0, 0, 2 * hz),
                v3(0, -1, 0));
  mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, 1));
  mesh_add_quad(&m, v3(hx, -hy, -hz), v3(-2 * hx, 0, 0), v3(0, 2 * hy, 0),
                v3(0, 0, -1));
  return m;
}

static Mesh tessellate_quad(Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  Mesh m;
  m.verts = malloc(4 * sizeof(Vertex));
  m.tris = malloc(2 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;
  mesh_add_quad(&m, origin, e1, e2, nrm);
  return m;
}

/* The sphere's corners sit on a grid of rings (top to bottom) and segments
 * (around). This finds the slot for one (ring, segment). Each ring keeps one
 * extra column so the seam where it wraps around lines up cleanly. */
static inline int sphere_vertex_index(int ring, int seg, int segs) {
  return ring * (segs + 1) + seg;
}

/* Builds a ball out of rings and segments, like a globe's lat/long lines, with
 * each corner facing straight out from the centre so it shades smoothly. */
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
      m.verts[m.nvert++] = (Vertex){pos, nrm};
    }
  }

  for (int i = 0; i < rings; i++) {
    for (int j = 0; j < segs; j++) {
      int v00 = sphere_vertex_index(i, j, segs);
      int v10 = sphere_vertex_index(i + 1, j, segs);
      int v11 = sphere_vertex_index(i + 1, j + 1, segs);
      int v01 = sphere_vertex_index(i, j + 1, segs);
      m.tris[m.ntri++] = (Triangle){{v00, v10, v01}};
      m.tris[m.ntri++] = (Triangle){{v10, v11, v01}};
    }
  }
  return m;
}

/* ── named value types · (Material / Camera / SceneObject / PointLight) ── *
 * A few small types the drawing passes (§6-§9) read. They're defined up here
 * just because C needs a type written down before it's used. Material, Camera,
 * and SceneObject describe the scene (the Scene owns them, §10); PointLight is
 * built fresh each frame inside the lighting pass (§8) and parked here so the
 * small types sit together. (The big GBuffer type lives next to its use, §6.) */

/* Material — what a surface is made of, light-wise: how it bounces light and
 * whether it gives off any of its own. One per object, fed to the lighting (§8):
 *   albedo   — its base colour, i.e. how much red/green/blue it bounces back,
 *              each 0..1 (0 = black, 1 = bounces all of it). Floor ≈ slate
 *              (0.32,0.36,0.42), cubes ≈ sandstone (0.78,0.62,0.42), orb ≈ dark
 *              (0.20) since it mostly glows rather than reflects.
 *   emissive — light the surface gives off by itself, and it's allowed to go
 *              above 1.0 (brighter than the screen). Added straight in, so it
 *              shines even with no light on it or in a shadow, and SSAO never
 *              dims it. Zero for everything but the orb (≈3.2,1.8,0.8); those
 *              over-1.0 numbers are exactly what bloom grabs (§9). Ref: emissive
 *              + bloom, LearnOpenGL "Bloom".
 *   spec     — how shiny it is, 0..1: 1 = glossy, catches a bright hotspot (the
 *              cubes); 0 = matte (the floor, kept flat so it doesn't look like a
 *              wet metal plate and steal attention from the cubes and orb). */
typedef struct {
  Vec3 albedo;
  Vec3 emissive;
  float spec;
} Material;

/* Camera — the eye circling the scene, plus the transforms worked out from it.
 * Only two things are really "set" — the distance and the angle; everything else
 * is recomputed from them (by camera_rebuild_view / _proj) and is just a cache:
 *   dist  — how far the eye is from the centre; the +/- keys change it, kept
 *           within [CAM_DIST_MIN, CAM_DIST_MAX]. Farther back = scene looks
 *           smaller.
 *   yaw   — how far around the scene the eye has swung; scene_tick keeps nudging
 *           it so the view slowly orbits. dist + yaw together place the eye.
 *   pos   — the eye's actual spot, worked out from dist + yaw; also where the
 *           shininess highlight is measured from (§8).
 *   view  — the "look from the eye toward the centre" transform (from m4_lookat).
 *   proj  — the perspective transform that makes far things smaller (from
 *           m4_perspective), matched to the screen's shape.
 *   vp    — view and proj rolled into one, for dropping any world point onto the
 *           screen (the SSAO test points, the sun marker). */
typedef struct {
  float dist;
  float yaw;
  Vec3 pos;
  Mat4 view, proj, vp;
} Camera;

/* SceneObject — one thing in the scene, with everything needed to draw it kept
 * together (instead of scattered across separate arrays):
 *   mesh     — its shape, in its own coordinates (owned; see Mesh)
 *   material — what it's made of (see Material)
 *   model    — where to put it in the world. Each frame render_gbuffer combines
 *              this with the camera to know where it lands on screen. */
typedef struct {
  Mesh mesh;
  Material material;
  Mat4 model;
} SceneObject;

/* PointLight — a light that sits at one spot and shines in every direction, here
 * the glowing orb. Unlike the far-off sun, it gets dimmer with distance
 * (point_attenuation, §8), so it pools bright nearby and fades out — strong
 * contrast a coarse terminal shows off well.
 *   pos        — where it is (the orb's centre)
 *   color      — its colour, already turned up by its brightness (can exceed 1.0)
 *   lin, quad  — the two dials that set how fast it fades with distance */
typedef struct {
  Vec3 pos;
  Vec3 color;
  float lin, quad;
} PointLight;

/* ── §6 G-buffer — drawing each surface's facts into the tables ────────── */

/* GBuffer — a set of full-screen tables describing the surface seen at each
 * cell, the heart of "deferred" shading (Saito & Takahashi, SIGGRAPH '90). The
 * idea: split "WHICH surface shows up at this cell" from "HOW is it lit". We
 * draw every object ONCE, recording its facts here per cell; then the lighting
 * pass (§8) goes cell by cell and lights each one from these facts. So lighting
 * runs once per cell instead of once per (cell × triangle) — and, the reason
 * this file needs it, SSAO and bloom can read the surface straight off the
 * screen without touching the 3-D shapes again.
 *
 * One table per fact (rather than one fat record per cell) so each pass can run
 * straight down a single table. All sized to the biggest terminal we support and
 * read as [row][col]:
 *   pos      — where the surface point is in the world. Used by SSAO and for
 *              the shininess highlight.
 *   normal   — which way the surface faces here (blended across the triangle,
 *              then re-straightened to unit length).
 *   albedo   — the surface's base colour here (from its Material).
 *   emissive — the surface's own glow here (can be >1.0; the orb's is bloom's
 *              only source, §9).
 *   spec     — how shiny it is here, 0 matte … 1 glossy.
 *   zbuf     — how near this cell's surface is, used to keep only the closest:
 *              a new surface is written only if it's nearer than what's there
 *              (starts at 1.0 = farthest).
 *   z_view   — the surface's true distance from the eye. SSAO compares these
 *              because they grow evenly with distance; the zbuf number doesn't,
 *              and would skew SSAO's range check.
 *   valid    — 1 where some object was drawn, 0 for empty background. Every
 *              later pass skips the empty cells. */
typedef struct {
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 emissive[GBUF_MAX_H][GBUF_MAX_W]; /* the surface's own glow      */
  float spec[GBUF_MAX_H][GBUF_MAX_W];    /* shininess 0..1              */
  float zbuf[GBUF_MAX_H][GBUF_MAX_W];    /* nearness, for keeping closest */
  float z_view[GBUF_MAX_H][GBUF_MAX_W];  /* true distance from the eye   */
  uint8_t valid[GBUF_MAX_H][GBUF_MAX_W];
} GBuffer;

/* The one G-buffer. It's a drawing scratchpad, kept here rather than inside the
 * Scene because it's wiped and refilled every frame and the scene never reads
 * it — so the scene and the pixels stay cleanly separate. */
static GBuffer g_gbuf;

static void gbuffer_clear(GBuffer *gb, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      gb->zbuf[r][c] = 1.0f;
      gb->z_view[r][c] = -CAM_FAR;
      gb->valid[r][c] = 0;
    }
  }
}

/* For a point (px,py) and a triangle, returns three weights saying how much each
 * corner pulls on that point — they tell us if the point is inside (all three
 * positive) and how to blend the corners' values there. */
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

/* Takes a corner the camera has already transformed and finishes the job: divide
 * by distance (so farther is smaller), then scale into screen cells. Returns
 * (screen x, screen y, depth); y is flipped because screen rows count downward
 * but up should be up. */
static inline Vec3 project_to_screen(Vec4 clip, int cols, int rows) {
  float w = clip.w;
  if (fabsf(w) < 1e-6f)
    w = 1e-6f;
  return v3((clip.x / w + 1.f) * 0.5f * (float)cols,
            (-clip.y / w + 1.f) * 0.5f * (float)rows, clip.z / w);
}

/* True if this triangle is turned away from us (the back of the shape), so we
 * can skip drawing it. The trick: measure the triangle's signed area on screen;
 * with our corner ordering and the y-flip, faces toward us come out negative, so
 * anything zero-or-positive is facing away. */
static inline bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area >= 0.f;
}

/* Draws one object's triangles into the G-buffer tables. Per triangle: move its
 * corners onto the screen, drop it if it's behind us or facing away, then for
 * every cell it covers and is the nearest thing seen so far, record the surface
 * facts there (position, facing, colour, glow, shininess). The orb's glow is the
 * one that's above white and that bloom later picks up. */
static void rasterize_object(GBuffer *gb, const Mesh *mesh, Material mat,
                             Mat4 mvp, Mat4 model, Mat4 modelview,
                             Mat4 norm_mat, int cols, int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    /* work out each corner's screen spot, plus its world position, its facing,
     * and its distance from the eye — the per-cell fill below blends these */
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

    /* skip a triangle that's entirely behind the camera */
    if (clip[0].w < CLIP_W_MIN && clip[1].w < CLIP_W_MIN &&
        clip[2].w < CLIP_W_MIN)
      continue;

    /* finish each corner onto the screen */
    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      Vec3 s = project_to_screen(clip[vi], cols, rows);
      sx[vi] = s.x;
      sy[vi] = s.y;
      sz[vi] = s.z;
    }

    if (is_back_facing(sx, sy))
      continue;

    /* the small box of cells the triangle could touch, clipped to the screen */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    /* walk those cells: is the cell inside the triangle, and is it nearer than
     * whatever's already there? if so, record this surface's facts */
    for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
      for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue; /* outside the triangle */

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z >= gb->zbuf[py][px])
          continue; /* something nearer is already here */

        gb->zbuf[py][px] = z;
        gb->z_view[py][px] = b[0] * vz[0] + b[1] * vz[1] + b[2] * vz[2];
        gb->pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
        gb->normal[py][px] =
            v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
        gb->albedo[py][px] = mat.albedo;
        gb->emissive[py][px] = mat.emissive;
        gb->spec[py][px] = mat.spec;
        gb->valid[py][px] = 1;
      }
    }
  }
}

static void render_gbuffer(GBuffer *gb, const SceneObject *objects,
                           int n_objects, const Camera *cam, int cols,
                           int rows) {
  gbuffer_clear(gb, cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mv = m4_mul(cam->view, objects[oi].model);
    Mat4 mvp = m4_mul(cam->proj, mv);
    Mat4 nmat = m4_normal_mat(objects[oi].model);
    rasterize_object(gb, &objects[oi].mesh, objects[oi].material, mvp,
                     objects[oi].model, mv, nmat, cols, rows);
  }
}

/* ── §7 ssao · RENDER ─────────────────────────────────────────────────────────── *
 *
 * Same idea as ssao_pipeline.c. For each visible cell, scatter a handful of test
 * points into the air just above the surface, and for each one ask the screen
 * "is there already something drawn in front of where this floated to?" The more
 * yes-answers, the more tucked-away the spot is, so the darker we shade it. A
 * 3×3 average at the end smooths out the speckle from using only a few points. */

static Vec3 k_ssao[SSAO_KERNEL_VARIANTS][SSAO_SAMPLES];

/* A tiny do-it-yourself random number generator (so the test points are the same
 * every run). lcg_step gives the next number; lcg_unit maps it to 0..1. */
static unsigned lcg_step(unsigned *s) {
  *s = *s * 1664525u + 1013904223u;
  return *s;
}
static float lcg_unit(unsigned *s) {
  return (lcg_step(s) >> 8) / (float)0x01000000;
}

/* Picks the SSAO test points once at startup: random directions, then pushed out
 * to various distances, with most kept close to the surface. Four separate sets
 * so neighbouring cells can use different ones (see kernel_variant). */
static void ssao_init_kernel(void) {
  unsigned seed = 0xC0FFEE5Au;
  for (int v = 0; v < SSAO_KERNEL_VARIANTS; v++) {
    for (int i = 0; i < SSAO_SAMPLES; i++) {
      float dx, dy, dz, len2;
      do {
        dx = 2.f * lcg_unit(&seed) - 1.f;
        dy = 2.f * lcg_unit(&seed) - 1.f;
        dz = 2.f * lcg_unit(&seed) - 1.f;
        len2 = dx * dx + dy * dy + dz * dz;
      } while (len2 > 1.f || len2 < 1e-6f);
      float inv = 1.f / sqrtf(len2);
      float t = (float)i / (float)SSAO_SAMPLES;
      float scale =
          SSAO_SAMPLE_MIN + (1.f - SSAO_SAMPLE_MIN) * t * t; /* keep most near */
      k_ssao[v][i] = v3_scale(v3(dx * inv, dy * inv, dz * inv), scale);
    }
  }
}

static float g_ao[GBUF_MAX_H][GBUF_MAX_W];
static float g_ao_blur[GBUF_MAX_H][GBUF_MAX_W];

/* Which of the 4 test-point sets this cell uses, in a 2×2 checker by row/column.
 * Giving neighbours different sets turns what would be visible stripes into fine
 * speckle, which the 3×3 average then cleans up. */
static inline int kernel_variant(int r, int c) {
  return (c & 1) | ((r & 1) << 1);
}

/* Flips a test point to the side the surface faces, so it always pokes out into
 * the open air rather than down into the surface. */
static inline Vec3 hemisphere_dir(Vec3 dir, Vec3 N) {
  return v3_dot(dir, N) < 0.f ? v3_neg(dir) : dir;
}

static void ssao_pass(const GBuffer *gb, const Camera *cam, int cols,
                      int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        g_ao[r][c] = 1.f;
        continue;
      }

      Vec3 P = gb->pos[r][c];
      Vec3 N = gb->normal[r][c];
      int variant = kernel_variant(r, c);

      float occlude_w = 0.f, total_w = 0.f;
      for (int i = 0; i < SSAO_SAMPLES; i++) {
        /* float a test point just off the surface */
        Vec3 dir = hemisphere_dir(k_ssao[variant][i], N);
        Vec3 S = v3_add(P, v3_scale(dir, SSAO_RADIUS));

        /* find where that point lands on screen, to see what's drawn there */
        Vec4 clip = m4_mul_v4(cam->vp, v4(S.x, S.y, S.z, 1.f));
        if (clip.w < CLIP_W_MIN)
          continue;
        int ix = (int)((clip.x / clip.w + 1.f) * 0.5f * (float)cols);
        int iy = (int)((-clip.y / clip.w + 1.f) * 0.5f * (float)rows);
        if (ix < 0 || ix >= cols || iy < 0 || iy >= rows)
          continue;
        if (!gb->valid[iy][ix])
          continue;

        /* ignore blockers that are way deeper — they're a different surface, not
         * a nearby nook, so they shouldn't cast a dark halo */
        float dz = fabsf(gb->z_view[r][c] - gb->z_view[iy][ix]);
        float attn = 1.f - dz / SSAO_RADIUS;
        if (attn <= 0.f)
          continue;
        total_w += attn;

        /* if what's drawn there is nearer than our floated point, the point is
         * blocked — count it toward the darkening */
        float ndc_z_S = clip.z / clip.w;
        if (gb->zbuf[iy][ix] < ndc_z_S - SSAO_BIAS)
          occlude_w += attn;
      }
      float ao = (total_w > 1e-6f) ? (1.f - occlude_w / total_w) : 1.f;
      g_ao[r][c] = clamp01(ao);
    }
  }
}

static void ssao_blur(const GBuffer *gb, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
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
          if (!gb->valid[rr][cc])
            continue;
          sum += g_ao[rr][cc];
          count++;
        }
      }
      g_ao_blur[r][c] = (count > 0) ? (sum / (float)count) : 1.f;
    }
  }
}

/* ── §8 lightpass · RENDER — orb light + sun fill + glow ────────────────────── *
 *
 * Adds up the light at each cell:
 *   ambient — a little base light everywhere, the only part SSAO darkens
 *   orb     — the warm orb as a real light, fading with distance
 *   sun     — a dim far-off fill so the orb's shadow side isn't pitch black
 *   glow    — the surface's own light (the orb), added straight in
 *
 * The total is deliberately NOT capped at white — the orb's glow and bright pool
 * are meant to go over, because that's what bloom feeds on. The squashing-back-
 * into-range happens later, at paint time. */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

/* How much a point light dims at distance d — strong up close, weaker far away.
 * The leading 1 keeps it sane when something is right on top of the light. */
static inline float point_attenuation(float d, float lin, float quad) {
  return 1.f / (1.f + lin * d + quad * d * d);
}

/* Works out the colour of one surface cell: base light + the orb's light + the
 * dim sun + the surface's own glow. Left un-capped so the glow and the bright
 * pool can go over white for bloom to catch. The orb's light fades with distance
 * (a pool that falls off); the sun is a steady dim fill so faces turned away from
 * the orb still show their shape. SSAO only dims the base light; the glow is
 * added raw, so the orb stays bright even though it faces away from its own
 * light. Ref: Blinn-Phong highlight (Blinn, '77). */
static Vec3 shade_surface(Vec3 P, Vec3 N, Material mat, float ao, Vec3 cam_pos,
                          const PointLight *key, Vec3 fill_L, Vec3 fill_col,
                          Vec3 ambient) {
  /* base light — present everywhere; the only part SSAO darkens */
  Vec3 amb = v3(ambient.x * mat.albedo.x * ao, ambient.y * mat.albedo.y * ao,
                ambient.z * mat.albedo.z * ao);

  /* the orb's light — brighter the more the surface faces it, fading with how
   * far away the orb is */
  Vec3 to_light = v3_sub(key->pos, P);
  float dist = v3_len(to_light);
  Vec3 L = v3_scale(to_light, 1.f / fmaxf(dist, 1e-4f));
  float atten = point_attenuation(dist, key->lin, key->quad);
  float kdiff = fmaxf(0.f, v3_dot(N, L)) * atten;
  Vec3 dif = v3(mat.albedo.x * key->color.x * kdiff,
                mat.albedo.y * key->color.y * kdiff,
                mat.albedo.z * key->color.z * kdiff);

  Vec3 V = v3_norm(v3_sub(cam_pos, P));
  Vec3 H = v3_norm(v3_add(L, V));
  /* the shiny hotspot, only on glossy surfaces: 0 → matte floor, 1 → glossy cube */
  float kspec =
      powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN * atten * mat.spec;
  Vec3 sp =
      v3(key->color.x * kspec, key->color.y * kspec, key->color.z * kspec);

  /* the dim sun, so the side away from the orb isn't solid black */
  float fdiff = fmaxf(0.f, v3_dot(N, fill_L));
  Vec3 fil = v3(mat.albedo.x * fill_col.x * fdiff,
                mat.albedo.y * fill_col.y * fdiff,
                mat.albedo.z * fill_col.z * fdiff);

  return v3(amb.x + dif.x + sp.x + fil.x + mat.emissive.x,
            amb.y + dif.y + sp.y + fil.y + mat.emissive.y,
            amb.z + dif.z + sp.z + fil.z + mat.emissive.z);
}

static void render_lightpass(const GBuffer *gb, const Camera *cam,
                             bool ssao_enabled, int cols, int rows) {
  /* the orb as a warm light that fades with distance */
  PointLight key = {.pos = v3(ORB_CX, ORB_CY, ORB_CZ),
                    .color = v3(POINT_LIGHT_COL[0] * POINT_LIGHT_INTENSITY,
                                POINT_LIGHT_COL[1] * POINT_LIGHT_INTENSITY,
                                POINT_LIGHT_COL[2] * POINT_LIGHT_INTENSITY),
                    .lin = POINT_ATTEN_LINEAR,
                    .quad = POINT_ATTEN_QUAD};

  /* the dim sun: one direction and colour shared by the whole frame */
  Vec3 fill_L = v3_norm(v3_neg(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2])));
  Vec3 fill_col = v3(SUN_COL[0] * FILL_GAIN, SUN_COL[1] * FILL_GAIN,
                     SUN_COL[2] * FILL_GAIN);
  Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        g_light[r][c] = v3(0, 0, 0); /* background stays black */
        continue;
      }

      Material mat = {.albedo = gb->albedo[r][c],
                      .emissive = gb->emissive[r][c],
                      .spec = gb->spec[r][c]};
      float ao = ssao_enabled ? g_ao_blur[r][c] : 1.f;

      g_light[r][c] = shade_surface(gb->pos[r][c], gb->normal[r][c], mat, ao,
                                    cam->pos, &key, fill_L, fill_col, ambient);
    }
  }
}

/* ── §9 bloom · RENDER — pull out the bright bits, blur, add back ─────────────── *
 *
 * The glow, in four little passes:
 *
 *   bloom_extract — copy only the over-bright pixels into a side buffer;
 *     everything dimmer becomes black.
 *
 *   bloom_blur_h / bloom_blur_v — smear that side buffer sideways, then up/down.
 *     Doing it in two one-direction passes gives the same result as one big
 *     2-D blur but is much cheaper.
 *
 *   bloom_composite — add the blurred glow back on top of the lit image. Now
 *     each bright source wears a soft halo. */

static Vec3 g_bloom[GBUF_MAX_H][GBUF_MAX_W];     /* the glow buffer */
static Vec3 g_bloom_tmp[GBUF_MAX_H][GBUF_MAX_W]; /* scratch between the two blurs */

static void bloom_extract(float threshold, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 lit = g_light[r][c];
      g_bloom[r][c] = (v3_luma(lit) > threshold) ? lit : v3(0, 0, 0);
    }
  }
}

/* Keeps a coordinate inside the screen, so a blur reaching off the edge just
 * reuses the edge pixel instead of running off and clipping the halo. */
static inline int clamp_ix(int v, int max) {
  return v < 0 ? 0 : (v >= max ? max - 1 : v);
}

static void bloom_blur_h(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 sum = v3(0, 0, 0);
      for (int k = 0; k < BLOOM_TAPS; k++) {
        int sc = clamp_ix(c + k - BLOOM_RADIUS, cols);
        float w = BLOOM_KERNEL[k];
        Vec3 s = g_bloom[r][sc];
        sum = v3_add(sum, v3_scale(s, w));
      }
      g_bloom_tmp[r][c] = sum;
    }
  }
}

static void bloom_blur_v(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 sum = v3(0, 0, 0);
      for (int k = 0; k < BLOOM_TAPS; k++) {
        int sr = clamp_ix(r + k - BLOOM_RADIUS, rows);
        float w = BLOOM_KERNEL[k];
        Vec3 s = g_bloom_tmp[sr][c];
        sum = v3_add(sum, v3_scale(s, w));
      }
      g_bloom[r][c] = sum;
    }
  }
}

static void bloom_composite(float intensity, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g_light[r][c] = v3_add(g_light[r][c], v3_scale(g_bloom[r][c], intensity));
    }
  }
}

/* ── §10 scene — the world and the camera ──────────────────────────────── */

/* Scene — everything about WHAT we're showing and HOW the viewer is steering it,
 * in one place. It holds only the scene itself — the big drawing buffers live
 * elsewhere as their own globals — so reading Scene tells you about the world,
 * not the pixels. Only scene_init, scene_tick, and the keypress handler write it;
 * the drawing passes never do.
 *   objects          — the things in the scene, in fixed order: floor, cube A,
 *                      cube B, orb (the N_OBJECTS list, §1.7)
 *   camera           — the orbiting eye
 *   bloom_on/ssao_on — the b / a on-off switches; render_frame checks them to
 *                      skip those effects when off (which is how you compare)
 *   paused           — when true, the orbit freezes (space toggles it)
 *   scene_cols/_rows — how many cells the scene gets: the full width, and the
 *                      height minus the few rows saved at the bottom for the HUD */
typedef struct {
  SceneObject objects[N_OBJECTS];
  Camera camera;

  bool bloom_on;
  bool ssao_on;
  bool paused;
  int scene_cols;
  int scene_rows;
} Scene;

/* Rebuilds the perspective so the scene isn't stretched — it accounts for the
 * screen's shape and that terminal cells are taller than they are wide. */
static void camera_rebuild_proj(Camera *cam, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  cam->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/* Moves the eye to its spot on the orbit (from distance + angle) and refreshes
 * the look-from-here transforms. Call after the projection is set, so the
 * combined one picks it up. */
static void camera_rebuild_view(Camera *cam) {
  float r = cam->dist;
  cam->pos = v3(sinf(cam->yaw) * r, CAM_EYE_Y, cosf(cam->yaw) * r);
  cam->view = m4_lookat(cam->pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
  cam->vp = m4_mul(cam->proj, cam->view);
}

/* Builds the scene: a floor, two cubes, and the glowing orb. The orb is the only
 * thing that glows on its own, and that glow is what bloom feeds on. */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->objects[i].mesh);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->bloom_on = true;
  s->ssao_on = true;
  s->camera.dist = CAM_DIST;
  s->camera.yaw = 0.f;

  s->objects[OBJ_FLOOR].mesh = tessellate_quad(
      v3(-FLOOR_HALF_X, 0.f, FLOOR_HALF_Z), v3(2 * FLOOR_HALF_X, 0.f, 0.f),
      v3(0.f, 0.f, -2 * FLOOR_HALF_Z), v3(0.f, 1.f, 0.f));
  s->objects[OBJ_FLOOR].material =
      (Material){.albedo = v3(0.32f, 0.36f, 0.42f), /* dark slate */
                 .emissive = v3(0, 0, 0),
                 .spec = 0.f}; /* matte: no metallic highlight on the floor */
  s->objects[OBJ_FLOOR].model = m4_identity();

  s->objects[OBJ_CUBE_A].mesh = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->objects[OBJ_CUBE_A].material =
      (Material){.albedo = v3(0.78f, 0.62f, 0.42f), /* sandstone */
                 .emissive = v3(0, 0, 0),
                 .spec = 1.f}; /* glossy: cubes catch the orb's highlight */
  s->objects[OBJ_CUBE_A].model = m4_translate(CUBE_A_CX, CUBE_A_CY, CUBE_A_CZ);

  s->objects[OBJ_CUBE_B].mesh = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->objects[OBJ_CUBE_B].material =
      (Material){.albedo = v3(0.78f, 0.62f, 0.42f),
                 .emissive = v3(0, 0, 0),
                 .spec = 1.f};
  s->objects[OBJ_CUBE_B].model = m4_translate(CUBE_B_CX, CUBE_B_CY, CUBE_B_CZ);

  s->objects[OBJ_ORB].mesh = tessellate_sphere(ORB_RADIUS, ORB_RINGS, ORB_SEGS);
  s->objects[OBJ_ORB].material =
      (Material){.albedo = v3(0.20f, 0.20f, 0.20f), /* mostly emit */
                 .emissive = v3(ORB_EMISSIVE[0], ORB_EMISSIVE[1], ORB_EMISSIVE[2]),
                 .spec = 0.f}; /* emissive source; its surface stays matte */
  s->objects[OBJ_ORB].model = m4_translate(ORB_CX, ORB_CY, ORB_CZ);

  camera_rebuild_proj(&s->camera, total_cols, s->scene_rows);
  camera_rebuild_view(&s->camera);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->camera.yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  camera_rebuild_view(&s->camera);
}

/* ── §11 screen — painting the frame, the marker, and the HUD ──────────── */

static void render_scene(const GBuffer *gb, int cols, int rows) {
  int C = cols < GBUF_MAX_W ? cols : GBUF_MAX_W;
  int R = rows < GBUF_MAX_H ? rows : GBUF_MAX_H;

  /* wipe the dither leftovers so each frame starts fresh — otherwise last
   * frame's error would faintly ghost into this one */
  memset(g_err_r, 0, sizeof g_err_r);
  memset(g_err_g, 0, sizeof g_err_g);
  memset(g_err_b, 0, sizeof g_err_b);
  memset(g_err_y, 0, sizeof g_err_y);

  /* go left→right, top→bottom — the order the dither hands its error forward */
  for (int r = 0; r < R; r++) {
    for (int c = 0; c < C; c++) {
      if (!gb->valid[r][c])
        continue;
      paint_cell(c, r, C, R, g_light[r][c]);
    }
  }
}

/* Drops a little '*' in the sky to show where the dim sun is shining from — just
 * a visual aid, since the sun has a direction but nowhere you can point at. (The
 * orb light you can already see; this is only for the fill.) Drawn on top of the
 * scene and not part of the lighting or bloom. */
static void sun_marker_draw(const Camera *cam, int cols, int rows) {
  Vec3 to_sun = v3_norm(v3_neg(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2])));
  Vec3 world =
      v3_add(v3(0.f, CAM_LOOK_Y, 0.f), v3_scale(to_sun, SUN_MARKER_DIST));

  Vec4 clip = m4_mul_v4(cam->vp, v4(world.x, world.y, world.z, 1.f));
  if (clip.w < 0.001f)
    return;
  int sx = (int)((clip.x / clip.w + 1.f) * 0.5f * (float)cols);
  int sy = (int)((-clip.y / clip.w + 1.f) * 0.5f * (float)rows);
  if (sx < 0 || sx >= cols || sy < 0 || sy >= rows)
    return;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvaddch(sy, sx, (chtype)(unsigned char)'*');
  if (sx + 5 < cols)
    mvprintw(sy, sx + 1, "sun");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int total_tris = 0;
  for (int i = 0; i < N_OBJECTS; i++)
    total_tris += s->objects[i].mesh.ntri;

  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  bloom:%s  ssao:%s  zoom:%.1f  tris:%d  %s ", fps,
           s->bloom_on ? "ON " : "OFF", s->ssao_on ? "ON " : "OFF",
           (double)s->camera.dist, total_tris, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " BLOOM · DEFERRED · SSAO ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "passes: gbuffer -> ssao+blur -> lightpass(HDR) -> "
           "bloom(extract+blurH+blurV+composite) -> paint");
  mvprintw(hr + 1, 1,
           "bloom: threshold=%.2f  intensity=%.2f  kernel=%d-tap Gaussian "
           "(separable)",
           (double)BLOOM_THRESHOLD, (double)BLOOM_INTENSITY, BLOOM_TAPS);
  mvprintw(hr + 2, 1,
           "Toggle 'b' off: orb is a hard-edged disc.   "
           "Toggle 'a' off: cube corners stop darkening.");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  b:bloom  a:ssao  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* Runs the whole drawing pipeline for one frame, in order, each step feeding the
 * next: record the surfaces, (optionally) work out the SSAO shadows, light
 * everything, then (optionally) add the bloom glow. Reads the scene but changes
 * only the drawing buffers, never the scene itself. */
static void render_frame(const Scene *s) {
  GBuffer *gb = &g_gbuf;
  render_gbuffer(gb, s->objects, N_OBJECTS, &s->camera, s->scene_cols,
                 s->scene_rows);

  if (s->ssao_on) {
    ssao_pass(gb, &s->camera, s->scene_cols, s->scene_rows);
    ssao_blur(gb, s->scene_cols, s->scene_rows);
  }

  render_lightpass(gb, &s->camera, s->ssao_on, s->scene_cols, s->scene_rows);

  if (s->bloom_on) {
    bloom_extract(BLOOM_THRESHOLD, s->scene_cols, s->scene_rows);
    bloom_blur_h(s->scene_cols, s->scene_rows);
    bloom_blur_v(s->scene_cols, s->scene_rows);
    bloom_composite(BLOOM_INTENSITY, s->scene_cols, s->scene_rows);
  }
}

/* ── §12 app — setup, the main loop, and keypresses ────────────────────── */

/* App — the whole running program: the scene plus the bits the main loop and the
 * signal handlers need. It's not part of the 3-D world — it's the process — and
 * it exists as one shared object (g_app) so the signal handlers can reach it
 * without being handed a pointer.
 *   scene                 — the scene we're drawing (see Scene)
 *   total_cols/total_rows — the full terminal size; the HUD takes a few rows at
 *                           the bottom and the scene gets the rest.
 *   running               — the main loop keeps going while this is set; cleared
 *                           by Ctrl-C / kill or the q/ESC key.
 *   need_resize           — set when the terminal is resized, handled at the top
 *                           of the next frame (re-measure and rebuild the scene).
 * running/need_resize are marked volatile sig_atomic_t because a signal can set
 * them at any moment, and that tells the compiler to always read/write them for
 * real rather than optimise the access away. */
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
  typeahead(-1);
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
  case 'b':
  case 'B':
    s->bloom_on = !s->bloom_on;
    break;
  case 'a':
  case 'A':
    s->ssao_on = !s->ssao_on;
    break;
  case '=':
  case '+':
    s->camera.dist -= CAM_ZOOM_STEP;
    if (s->camera.dist < CAM_DIST_MIN)
      s->camera.dist = CAM_DIST_MIN;
    camera_rebuild_view(&s->camera);
    break;
  case '-':
  case '_':
    s->camera.dist += CAM_ZOOM_STEP;
    if (s->camera.dist > CAM_DIST_MAX)
      s->camera.dist = CAM_DIST_MAX;
    camera_rebuild_view(&s->camera);
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

    /* handle a terminal resize if one happened */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* how long since last frame (capped, so a hiccup doesn't lurch the orbit) */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* advance the scene (just nudges the camera's orbit) */
    scene_tick(&app->scene, dt_sec);

    /* update the fps readout and wait to hold a steady frame rate */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    /* draw the frame: run the pipeline, paint it, add the marker and HUD */
    Scene *s = &app->scene;
    erase();
    render_frame(s);
    render_scene(&g_gbuf, s->scene_cols, s->scene_rows);
    sun_marker_draw(&s->camera, s->scene_cols, s->scene_rows);
    hud_draw(s, fps_display);
    screen_present();

    /* handle a keypress (pause / reset / toggles / zoom / quit) */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&app->scene.objects[i].mesh);

  endwin();
  return 0;
}
