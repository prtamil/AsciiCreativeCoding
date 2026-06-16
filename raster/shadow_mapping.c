/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shadow_mapping.c — draws a floor, a cube, and a sphere lit by a sun, and
 * paints the shadows they cast, all in coloured ASCII in the terminal.
 * Press 's' to drop the shadows and watch every face get full sun even
 * where something is in the way.
 *
 * The shadow trick (Williams 1978): first look at the scene from the sun's
 * side and note how far away the nearest thing is in each direction — that
 * record is the "shadow map". Then, for each spot the camera sees, if
 * something was nearer to the sun, that spot must be sitting in shadow.
 *
 * Builds on the plain triangle renderer in raster/cube_raster.c. Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/shadow_mapping.c -o shadow -lncurses -lm
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

enum {
  FPS_TARGET = 60,
  HUD_ROWS = 2, /* rows reserved at the bottom for the readout */
};

/* §1.1 the camera — where we watch from and how wide a view. */
#define CAM_FOV (50.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST_DEF 5.0f
#define CAM_DIST_MIN 2.0f
#define CAM_DIST_MAX 9.0f
#define CAM_ZOOM_STEP 0.25f
#define CAM_HEIGHT 1.6f
#define CAM_LOOK_Y 0.5f
#define CELL_W 8  /* a terminal cell is roughly 8 wide × 16 tall in pixels; */
#define CELL_H 16 /* we account for that so the picture isn't squashed      */

/* §1.2 the sun. It sits up and to one side and slowly rocks left-to-right
 * (like a lazy sundial), so the shadows sweep across the floor on their own
 * while the camera and floor stay put. It's kept on the camera's side so we
 * see lit faces, and off to one side so each shadow falls clear of its
 * object instead of hiding behind it. */
static const float SUN_DIR[3] = {0.40f, -0.54f, -0.74f}; /* which way its light travels */
#define LIGHT_ARC_HALF 0.40f          /* how far it rocks each way (radians, ~23°) */
#define LIGHT_ORBIT_RAD_PER_SEC 0.40f /* how fast it rocks */
static const float SUN_COL[3] = {0.98f, 0.88f, 0.68f};     /* warm sunlight colour */
static const float AMBIENT_COL[3] = {0.18f, 0.20f, 0.26f}; /* cool fill so shadows aren't pure black */
#define SHININESS 24.0f /* bigger = a tighter, smaller shiny highlight */
#define SPEC_GAIN 0.30f /* how bright that highlight is */

/* §1.3 the shadow map — the snapshot of distances taken from the sun's side.
 *
 * Since the sun's rays are treated as parallel, we look at the scene through a
 * straight-sided box (not a cone). LIGHT_ORTHO_HALF is half that box's width;
 * it has to cover the whole floor, or shadows landing past the edge get cut
 * off in a hard straight line. DISTANCE/NEAR/FAR just place the box.
 *
 * The bias numbers are a small fudge so a flat lit surface doesn't shadow
 * itself (which shows up as ugly stripes). We nudge the compared distance by a
 * hair — a bit more when the sun grazes at a shallow angle — but cap it, or
 * shadows start drifting away from the objects that cast them. */
#define LIGHT_DISTANCE 6.0f
#define LIGHT_ORTHO_HALF 4.0f
#define LIGHT_NEAR 0.5f
#define LIGHT_FAR 11.0f
#define SHADOW_W 384 /* shadow snapshot size; bigger = crisper shadow edges */
#define SHADOW_H 384
#define SHADOW_BIAS 0.0015f      /* base nudge for surfaces facing the sun head-on */
#define SHADOW_SLOPE_BIAS 0.007f /* extra nudge as the sun grazes at a shallow angle */
#define SHADOW_BIAS_MAX 0.020f   /* cap, so shadows don't drift off their objects */

/* §1.4 where things sit. The floor is a flat square at height 0; the cube
 * and sphere rest on top of it, one to each side. Everything is in world
 * units (the floor reaches 2.5 out from the centre in each direction). */
#define FLOOR_HALF_X 2.5f
#define FLOOR_HALF_Z 2.5f

#define CUBE_HALF 0.55f
#define CUBE_CX -0.75f
#define CUBE_CY (CUBE_HALF) /* lifted half its height so its base meets the floor */
#define CUBE_CZ -0.20f

#define SPHERE_R 0.55f
#define SPHERE_RINGS 12 /* how finely the ball is tessellated (more = rounder) */
#define SPHERE_SEGS 18
#define SPHERE_CX 0.85f
#define SPHERE_CY (SPHERE_R) /* lifted by its radius so it rests on the floor */
#define SPHERE_CZ 0.30f

enum { OBJ_FLOOR = 0, OBJ_CUBE, OBJ_SPHERE, N_OBJECTS };

/* §1.5 the biggest screen we'll handle — the off-screen buffers below are
 * sized once to this, so we never reallocate as the window changes. */
#define GBUF_MAX_W 400
#define GBUF_MAX_H 200

/* §1.6 the characters we draw with, darkest to brightest — ordered by how
 * much ink each one has (Paul Bourke's 92-character ramp). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 dither — a tiny repeating checkerboard of brightness nudges. Without
 * it, a big flat area snaps to one character and shows ugly bands; the nudges
 * scatter the boundary so it reads as a smooth gradient. DITHER_AMP is how
 * strong the nudge is — kept small so the floor doesn't look like wallpaper. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.04f

/* §1.8 colour slots — ncurses refers to each foreground/background pairing
 * by a number; these reserve the ranges we use. */
#define PAIR_CUBE_BASE 1 /* slots 1..216 hold the 6×6×6 colour cube */
#define PAIR_HUD 217
#define PAIR_HINT 218
#define CUBE_SIZE 6 /* 6 shades per channel → 6×6×6 = 216 colours */

/* §1.9 small safety numbers and thresholds, named so the code below reads in
 * words instead of bare digits. */
#define CLIP_W_MIN 0.001f   /* a vertex this far behind the eye counts as off-screen */
#define W_DIVIDE_EPS 1e-6f  /* never divide by anything smaller than this */
#define NDL_BIAS_FLOOR 0.1f /* floor on the sun-angle term so the bias math can't blow up */
#define PCF_RADIUS 1        /* shadow softening reaches 1 cell out → a 3×3 block */
#define BOLD_LUMA 0.85f     /* brighter than this and we draw the cell bold */

/* ── §2 timing — read a steady clock, and sleep for a while ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {.tv_sec = (time_t)(ns / 1000000000LL),
                         .tv_nsec = (long)(ns % 1000000000LL)};
  nanosleep(&req, NULL);
}

/* ── §3 math — 3D points, 4×4 transforms, and small geometry helpers ── */

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
static inline Vec3 v3_mul(Vec3 a, Vec3 b) { /* multiply part-by-part — tint one colour by another */
  return v3(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3_norm(Vec3 a) {
  float l = sqrtf(v3_dot(a, a));
  return (l > 1e-9f) ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(a.x * u + b.x * v + c.x * w, a.y * u + b.y * v + c.y * w,
            a.z * u + b.z * v + c.z * w);
}

static Mat4 m4_identity(void) {
  Mat4 m = {0};
  for (int i = 0; i < 4; i++)
    m.m[i][i] = 1.f;
  return m;
}

static Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {0};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

static Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}

static Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

/* Rotate a direction — like "which way a surface faces" — without shifting it.
 * Safe here because we only ever rotate and scale evenly, never squash. */
static Vec3 m4_dir(Mat4 m, Vec3 d) {
  return v3(m.m[0][0] * d.x + m.m[0][1] * d.y + m.m[0][2] * d.z,
            m.m[1][0] * d.x + m.m[1][1] * d.y + m.m[1][2] * d.z,
            m.m[2][0] * d.x + m.m[2][1] * d.y + m.m[2][2] * d.z);
}

static Mat4 m4_translate(float x, float y, float z) {
  Mat4 m = m4_identity();
  m.m[0][3] = x;
  m.m[1][3] = y;
  m.m[2][3] = z;
  return m;
}

/* The camera's lens: makes far things look smaller, like a normal 3D view. */
static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  float f = 1.f / tanf(fovy * 0.5f);
  Mat4 m = {0};
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

/* The sun's "lens": no shrinking with distance, since its rays run parallel. */
static Mat4 m4_orthographic(float l, float r, float b, float t, float n,
                            float f) {
  Mat4 m = m4_identity();
  m.m[0][0] = 2.f / (r - l);
  m.m[1][1] = 2.f / (t - b);
  m.m[2][2] = -2.f / (f - n);
  m.m[0][3] = -(r + l) / (r - l);
  m.m[1][3] = -(t + b) / (t - b);
  m.m[2][3] = -(f + n) / (f - n);
  return m;
}

/* Aim an eye at a target: builds the transform that puts the world in front
 * of it, looking straight down its gaze. */
static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 s = v3_norm(v3_cross(f, up));
  Vec3 u = v3_cross(s, f);
  Mat4 m = m4_identity();
  m.m[0][0] = s.x;
  m.m[0][1] = s.y;
  m.m[0][2] = s.z;
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[0][3] = -v3_dot(s, eye);
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline int clampi(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* For a point on the screen, work out three "blend weights" — one per triangle
 * corner — saying how much each corner counts at that point. If any comes out
 * negative the point is outside the triangle; otherwise the weights let us
 * blend the corners' depth and colour. Used by both passes (§6, §7). */
static inline void barycentric(float sx[3], float sy[3], float px, float py,
                               float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-9f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/* Turn a triangle's three corners from camera-math coordinates into actual
 * screen cells, plus a depth value per corner. Dividing by w is what makes
 * farther points crowd together — i.e. look smaller. w/h is the target size. */
static void clip_to_screen(const Vec4 clip[3], float sx[3], float sy[3],
                           float sz[3], int w, int h) {
  for (int vi = 0; vi < 3; vi++) {
    float cw = clip[vi].w;
    if (fabsf(cw) < W_DIVIDE_EPS)
      cw = W_DIVIDE_EPS;
    sx[vi] = (clip[vi].x / cw + 1.f) * 0.5f * (float)w;
    sy[vi] = (-clip[vi].y / cw + 1.f) * 0.5f * (float)h;
    sz[vi] = clip[vi].z / cw;
  }
}

/* The small rectangle of cells a triangle could touch, clipped to the screen,
 * so we only test cells near the triangle instead of the whole screen. */
static void triangle_bbox(const float sx[3], const float sy[3], int w, int h,
                          int *x0, int *x1, int *y0, int *y1) {
  *x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  *x1 = (int)fminf(w - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  *y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  *y1 = (int)fminf(h - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
}

/* True when the point sits inside the triangle (no weight went negative). */
static inline bool bary_inside(const float b[3]) {
  return b[0] >= 0.f && b[1] >= 0.f && b[2] >= 0.f;
}

/* ── §4 paint — turn a colour into a character + colour on the screen ── */

static int g_256;

/* Set up the colour palette once. Modern terminals give 256 colours, so we map
 * the 216-colour cube straight in; on an 8-colour terminal we fall back to
 * plain white text so the demo still runs. */
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

/* Squeeze a bright, open-ended colour down into the 0..1 range a screen can
 * show, then gamma-correct it so the mid-tones look right to the eye. */
static Vec3 tonemap(Vec3 c) {
  return v3(gamma_enc(reinhard(c.x)), gamma_enc(reinhard(c.y)),
            gamma_enc(reinhard(c.z)));
}

/* How bright a colour looks to us — green counts most, blue least. */
static float rec709_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* Snap a colour to the nearest of the 216 terminal colours (6 shades each of
 * red, green, blue) and return its slot number, 0..215. */
static int rgb_to_cube6(float r, float g, float b) {
  int r5 = clampi((int)(r * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  int g5 = clampi((int)(g * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  int b5 = clampi((int)(b * (CUBE_SIZE - 1) + 0.5f), 0, CUBE_SIZE - 1);
  return r5 * (CUBE_SIZE * CUBE_SIZE) + g5 * CUBE_SIZE + b5;
}

/* Pick which character to draw: brighter spots get a denser-looking glyph. */
static int ramp_index(float luma) {
  return clampi((int)(luma * (BOURKE_LEN - 1) + 0.5f), 0, BOURKE_LEN - 1);
}

/* Draw one cell: its character comes from how bright the colour is, its colour
 * from the colour itself. */
static void paint_cell(int sx, int sy, Vec3 col) {
  Vec3 disp = tonemap(col); /* make the colour screen-ready */

  float luma = rec709_luma(disp);
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith); /* nudge to avoid flat bands */

  int pair = g_256 ? PAIR_CUBE_BASE + rgb_to_cube6(disp.x, disp.y, disp.z)
                   : PAIR_CUBE_BASE;
  int glyph = ramp_index(lum_d);

  int attr = (luma > BOLD_LUMA) ? A_BOLD : A_NORMAL;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[glyph]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 shapes — build the floor, cube, and sphere out of triangles ── */

/* One corner point of a shape: where it is, and which way the surface faces
 * there (its "normal" — we use that to work out how much light it catches). */
typedef struct {
  Vec3 pos;
  Vec3 normal;
} Vertex;

/* One triangle, given as three positions in the vertex list above. */
typedef struct {
  int v[3];
} Triangle;

/* A whole shape: a bag of corner points and the triangles that join them. The
 * two arrays are malloc'd by the mesh_* builders below; mesh_free releases them. */
typedef struct {
  Vertex *verts;
  Triangle *tris;
  int nvert, ntri; /* how many of each are actually filled in */
} Mesh;

/* Frees what a builder allocated and zeroes the struct, so a leftover pointer
 * can't be reused by mistake. */
static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/* Add one flat rectangle as two triangles, all facing the same way. */
static void mesh_add_quad(Mesh *m, Vec3 origin, Vec3 e1, Vec3 e2, Vec3 nrm) {
  int v0 = m->nvert;
  m->verts[v0 + 0] = (Vertex){origin, nrm};
  m->verts[v0 + 1] = (Vertex){v3_add(origin, e1), nrm};
  m->verts[v0 + 2] = (Vertex){v3_add(v3_add(origin, e1), e2), nrm};
  m->verts[v0 + 3] = (Vertex){v3_add(origin, e2), nrm};
  m->nvert += 4;

  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

static Mesh mesh_floor(float hx, float hz) {
  Mesh m = {0};
  m.verts = malloc(4 * sizeof *m.verts);
  m.tris = malloc(2 * sizeof *m.tris);
  mesh_add_quad(&m, v3(-hx, 0, hz), /* origin (back-left) */
                v3(2 * hx, 0, 0),   /* +x edge            */
                v3(0, 0, -2 * hz),  /* -z edge            */
                v3(0, 1, 0));
  return m;
}

static Mesh mesh_box(float hx, float hy, float hz) {
  Mesh m = {0};
  m.verts = malloc(24 * sizeof *m.verts);
  m.tris = malloc(12 * sizeof *m.tris);
  /* +X face */ mesh_add_quad(&m, v3(hx, -hy, -hz), v3(0, 0, 2 * hz),
                              v3(0, 2 * hy, 0), v3(1, 0, 0));
  /* -X face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(0, 0, -2 * hz),
                              v3(0, 2 * hy, 0), v3(-1, 0, 0));
  /* +Y face */ mesh_add_quad(&m, v3(-hx, hy, -hz), v3(2 * hx, 0, 0),
                              v3(0, 0, 2 * hz), v3(0, 1, 0));
  /* -Y face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0),
                              v3(0, 0, -2 * hz), v3(0, -1, 0));
  /* +Z face */ mesh_add_quad(&m, v3(-hx, -hy, hz), v3(2 * hx, 0, 0),
                              v3(0, 2 * hy, 0), v3(0, 0, 1));
  /* -Z face */ mesh_add_quad(&m, v3(hx, -hy, -hz), v3(-2 * hx, 0, 0),
                              v3(0, 2 * hy, 0), v3(0, 0, -1));
  return m;
}

/* Build a ball like a globe: step down in rings (latitude) and around in
 * segments (longitude), drop a point at each crossing, and stitch each little
 * grid square into two triangles. On a ball, "which way the surface faces" at
 * a point is just the direction from the centre out to it — except right at
 * the poles, where that's undefined, so we set it straight up/down by hand. */
static Mesh mesh_sphere(float radius, int rings, int segs) {
  int nv = (rings + 1) * (segs + 1);
  int nt = rings * segs * 2;
  Mesh m = {0};
  m.verts = malloc(nv * sizeof *m.verts);
  m.tris = malloc(nt * sizeof *m.tris);

  for (int r = 0; r <= rings; r++) {
    float phi = (float)M_PI * (float)r / (float)rings; /* 0..π */
    float sp = sinf(phi), cp = cosf(phi);
    for (int s = 0; s <= segs; s++) {
      float th = 2.f * (float)M_PI * (float)s / (float)segs;
      float st = sinf(th), ct = cosf(th);
      Vec3 p = v3(radius * sp * ct, radius * cp, radius * sp * st);
      Vec3 n = (sp < 1e-6f) ? v3(0, (cp > 0 ? 1.f : -1.f), 0) : v3_norm(p);
      m.verts[m.nvert++] = (Vertex){p, n};
    }
  }
  for (int r = 0; r < rings; r++) {
    for (int s = 0; s < segs; s++) {
      int a = r * (segs + 1) + s;
      int b = r * (segs + 1) + (s + 1);
      int c = (r + 1) * (segs + 1) + s;
      int d = (r + 1) * (segs + 1) + (s + 1);
      m.tris[m.ntri++] = (Triangle){{a, b, d}};
      m.tris[m.ntri++] = (Triangle){{a, d, c}};
    }
  }
  return m;
}

/* ── §5b the things in the scene — gathered into Scene (§9) ── */

/* One thing in the scene: its shape, its plain colour, and where it sits. The
 * floor, cube, and sphere are three of these. */
typedef struct {
  Mesh mesh;   /* the shape, as triangles                       */
  Vec3 albedo; /* its colour before any light hits it           */
  Mat4 model;  /* where it sits / how it's turned, in the world */
} SceneObject;

/* Where we watch from. You steer it with two knobs — how far back (zoom) and
 * the angle — and from those we work out the eye position and the transforms
 * used to draw. */
typedef struct {
  Mat4 view, proj; /* the draw transforms, rebuilt when a knob changes */
  Vec3 pos;        /* where the eye ends up in the world               */
  float dist;      /* how far back we sit — the zoom knob              */
  float yaw;       /* the angle we sit at                              */
} Camera;

/* The sun, treated as a far-off light whose rays all run parallel. It has a
 * direction and a colour, plus an angle that slowly rocks the direction so the
 * shadows drift across the floor. */
typedef struct {
  Vec3 dir;   /* which way the light travels (set from yaw) */
  Vec3 color; /* the light's colour                         */
  float yaw;  /* the rocking angle, nudged each frame       */
} Sun;

/* ── §6 the camera pass — find what's visible at each screen cell ── */

/* For every screen cell, what the camera sees there — filled by this pass and
 * used by the lighting pass next. We record raw facts now and work out colours
 * in a second step, instead of all at once while drawing triangles. The depth
 * field doubles as the z-buffer: it remembers the nearest thing at each cell so
 * closer surfaces hide farther ones. (This bundle is known as a "G-buffer".) */
typedef struct {
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];     /* the 3D spot shown at this cell       */
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];  /* which way the surface faces there    */
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];  /* its plain colour                     */
  float depth[GBUF_MAX_H][GBUF_MAX_W];  /* nearest distance seen here (z-buffer) */
  int valid[GBUF_MAX_H][GBUF_MAX_W];    /* did anything get drawn at this cell?  */
} GBuffer;

static GBuffer g_gbuf;

static void gbuffer_clear(GBuffer *g, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      g->depth[r][c] = 1.f;
      g->valid[r][c] = 0;
    }
  }
}

/* Draw one object's triangles into the buffer, keeping whichever surface is
 * nearest at each cell. */
static void rasterize_object(const SceneObject *obj, Mat4 mvp, GBuffer *g,
                             int cols, int rows) {
  const Mesh *mesh = &obj->mesh;
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
      wpos[vi] = m4_pt(obj->model, v->pos);
      wnrm[vi] = v3_norm(m4_dir(obj->model, v->normal));
    }
    if (clip[0].w < CLIP_W_MIN && clip[1].w < CLIP_W_MIN &&
        clip[2].w < CLIP_W_MIN)
      continue; /* whole triangle behind the near plane */

    float sx[3], sy[3], sz[3];
    clip_to_screen(clip, sx, sy, sz, cols, rows);

    /* We don't bother skipping triangles that face away from us. For a closed
     * shape it wouldn't change the picture (the nearest-surface test hides
     * them anyway), and leaving them in means a shape whose triangles were
     * built facing the wrong way still shows up instead of vanishing. */
    int x0, x1, y0, y1;
    triangle_bbox(sx, sy, cols, rows, &x0, &x1, &y0, &y1);

    for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
      for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (!bary_inside(b))
          continue;

        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z >= g->depth[py][px])
          continue; /* something nearer is already here — skip */

        /* record this surface at this cell */
        g->depth[py][px] = z;
        g->pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
        g->normal[py][px] =
            v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
        g->albedo[py][px] = obj->albedo;
        g->valid[py][px] = 1;
      }
    }
  }
}

/* Clear the buffer, then draw every object from the camera's viewpoint. */
static void render_gbuffer(const SceneObject *objs, int n_objects,
                           const Camera *cam, GBuffer *g, int cols, int rows) {
  gbuffer_clear(g, cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mvp = m4_mul(m4_mul(cam->proj, cam->view), objs[oi].model);
    rasterize_object(&objs[oi], mvp, g, cols, rows);
  }
}

/* ── §7 the sun's view — build the shadow map, and read it back ── */

/* The scene as the sun sees it: for each cell of the sun's view, how far the
 * nearest thing is. We stash the sun's own draw transforms right next to that
 * picture, so checking whether a spot is shadowed needs nothing but this. This
 * is the classic shadow map (Williams 1978). */
typedef struct {
  Mat4 view, proj;                 /* the sun's draw transforms               */
  float depth[SHADOW_H][SHADOW_W]; /* nearest distance the sun sees, per cell  */
} ShadowMap;

static ShadowMap g_shadow;

static void shadowmap_clear(ShadowMap *sm) {
  for (int r = 0; r < SHADOW_H; r++)
    for (int c = 0; c < SHADOW_W; c++)
      sm->depth[r][c] = 1.f;
}

/* Like the camera pass, but from the sun and recording only distances, no
 * colour — that's all the shadow map needs. */
static void rasterize_object_shadow(const Mesh *mesh, Mat4 light_mvp,
                                    ShadowMap *sm) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    Vec4 clip[3];
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *v = &mesh->verts[tri->v[vi]];
      clip[vi] = m4_mul_v4(light_mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
    }
    float sx[3], sy[3], sz[3];
    clip_to_screen(clip, sx, sy, sz, SHADOW_W, SHADOW_H);

    /* Same as the camera pass: we don't skip away-facing triangles. The
     * nearest-distance test sorts it out, and it keeps oddly-built shapes
     * casting shadows instead of quietly dropping them. */
    int x0, x1, y0, y1;
    triangle_bbox(sx, sy, SHADOW_W, SHADOW_H, &x0, &x1, &y0, &y1);

    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
        if (!bary_inside(b))
          continue;
        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        if (z < sm->depth[py][px])
          sm->depth[py][px] = z; /* remember the nearest thing the sun sees here */
      }
    }
  }
}

/* Make this frame's shadow map: aim a straight-sided "camera" from the sun
 * back at the scene, keep its transforms in the map, then draw every object
 * into it recording only distances. */
static void shadow_pass(const SceneObject *objs, int n_objects, Vec3 sun_dir,
                        ShadowMap *sm) {
  Vec3 light_eye = v3_scale(sun_dir, -LIGHT_DISTANCE);
  sm->view = m4_lookat(light_eye, v3(0, 0, 0), v3(0, 1, 0));
  sm->proj = m4_orthographic(-LIGHT_ORTHO_HALF, LIGHT_ORTHO_HALF,
                             -LIGHT_ORTHO_HALF, LIGHT_ORTHO_HALF, LIGHT_NEAR,
                             LIGHT_FAR);
  shadowmap_clear(sm);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mvp = m4_mul(m4_mul(sm->proj, sm->view), objs[oi].model);
    rasterize_object_shadow(&objs[oi].mesh, mvp, sm);
  }
}

/* Is this point inside the slab of space the sun can actually see? */
static inline bool inside_ndc(Vec3 ndc) {
  return ndc.x >= -1.f && ndc.x <= 1.f && ndc.y >= -1.f && ndc.y <= 1.f &&
         ndc.z >= -1.f && ndc.z <= 1.f;
}

/* How big a fudge to allow when comparing distances. A surface the sun barely
 * grazes needs a bigger nudge to stop it speckling itself with fake shadow; one
 * facing the sun head-on needs almost none. Capped so shadows don't float off. */
static float shadow_slope_bias(float ndl) {
  float c = fmaxf(ndl, NDL_BIAS_FLOOR);     /* keep it above zero       */
  float slope_tan = sqrtf(1.f - c * c) / c; /* steeper grazing → bigger */
  float bias = SHADOW_BIAS + SHADOW_SLOPE_BIAS * slope_tan;
  return (bias > SHADOW_BIAS_MAX) ? SHADOW_BIAS_MAX : bias;
}

/* Soften the shadow edge: instead of a flat yes/no, test a little block of
 * neighbouring cells and return the fraction that are blocked. That gives a
 * smooth fade at the rim instead of a hard, jagged step. */
static float pcf_occlusion(const ShadowMap *sm, int ix, int iy, float z_frag,
                           float bias) {
  int hits = 0, count = 0;
  for (int dy = -PCF_RADIUS; dy <= PCF_RADIUS; dy++) {
    for (int dx = -PCF_RADIUS; dx <= PCF_RADIUS; dx++) {
      int rr = iy + dy, cc = ix + dx;
      if (rr < 0 || rr >= SHADOW_H || cc < 0 || cc >= SHADOW_W)
        continue;
      if (sm->depth[rr][cc] + bias < z_frag)
        hits++;
      count++;
    }
  }
  return (count > 0) ? ((float)hits / (float)count) : 0.f;
}

/* Is this spot in shadow? Find where it lands in the sun's view and compare its
 * distance against what the sun recorded there. Anything outside the sun's view
 * counts as lit. Returns 0 (full sun) up to 1 (full shadow). */
static float shadow_sample(const ShadowMap *sm, Vec3 world_pos, float ndl) {
  Vec4 clip = m4_mul_v4(m4_mul(sm->proj, sm->view),
                        v4(world_pos.x, world_pos.y, world_pos.z, 1.f));
  if (clip.w < W_DIVIDE_EPS)
    return 0.f;
  Vec3 ndc = v3(clip.x / clip.w, clip.y / clip.w, clip.z / clip.w);
  if (!inside_ndc(ndc))
    return 0.f; /* not in the light's view → assume lit */

  int ix = (int)((ndc.x + 1.f) * 0.5f * (float)SHADOW_W);
  int iy = (int)((-ndc.y + 1.f) * 0.5f * (float)SHADOW_H);
  if (ix < 0 || ix >= SHADOW_W || iy < 0 || iy >= SHADOW_H)
    return 0.f;

  return pcf_occlusion(sm, ix, iy, ndc.z, shadow_slope_bias(ndl));
}

/* ── §8 the lighting pass — colour each cell from light and shadow ── */

/* How lit a surface is, softened. A plain "is it facing the sun?" check makes
 * the shaded side go flat and muddy; this keeps a gentle gradient round the
 * back so curved things still read as curved. Faces aimed at the sun are
 * unchanged. (Valve's "half-Lambert" trick.) */
static float half_lambert(float ndl) {
  float wrap = 0.5f * ndl + 0.5f;
  return wrap * wrap;
}

/* The bright pinpoint highlight — strongest where the surface is angled to
 * bounce the sun straight back at the camera. */
static float blinn_spec(Vec3 N, Vec3 V, Vec3 L) {
  Vec3 H = v3_norm(v3_add(L, V));
  return powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
}

/* The finished colour for every cell — filled here, drawn to screen by §10. */
static Vec3 g_light[GBUF_MAX_H][GBUF_MAX_W];

/* Walk every cell the camera filled in and work out its final colour:
 * background fill light + the sun's light, blocked where it's in shadow. */
static void render_lightpass(const GBuffer *g, const ShadowMap *sm,
                             const Sun *sun, const Camera *cam, bool shadows_on,
                             int cols, int rows) {
  Vec3 sun_col = sun->color;
  Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
  Vec3 L = v3_norm(v3_neg(sun->dir));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!g->valid[r][c]) {
        g_light[r][c] = v3(0, 0, 0);
        continue;
      }

      Vec3 P = g->pos[r][c];
      Vec3 N = g->normal[r][c];
      Vec3 albedo = g->albedo[r][c];

      /* final = fill light + (sun's direct light, switched off in shadow). The
       * fill is never shadowed, so dark spots aren't pitch black. */
      float ndl = v3_dot(N, L);
      Vec3 amb = v3_mul(ambient, albedo);
      Vec3 dif = v3_scale(v3_mul(albedo, sun_col), half_lambert(ndl));
      Vec3 V = v3_norm(v3_sub(cam->pos, P));
      Vec3 sp = v3_scale(sun_col, blinn_spec(N, V, L));

      float shadow = shadows_on ? shadow_sample(sm, P, ndl) : 0.f;
      float lit = 1.f - shadow;

      g_light[r][c] = v3_add(amb, v3_scale(v3_add(dif, sp), lit));
    }
  }
}

/* ── §9 the scene — what exists, and how it changes over time ── */

/* Everything that makes up the world, in one place: the things in it, the
 * camera and sun that show it, and a couple of flags. The big drawing buffers
 * (g_gbuf, g_shadow, g_light) live on their own as scratch space, not here. */
typedef struct {
  SceneObject objects[N_OBJECTS]; /* the floor, cube, and sphere      */
  Camera cam;                     /* where we watch from              */
  Sun sun;                        /* the light that casts the shadows */

  bool shadows_on; /* 's' turns shadows on and off         */
  bool paused;     /* space freezes the sun's slow rocking  */
  int scene_cols;  /* drawing area size, in cells           */
  int scene_rows;
} Scene;

static void camera_rebuild_proj(Camera *cam, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  cam->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void camera_rebuild_view(Camera *cam) {
  float r = cam->dist;
  cam->pos = v3(sinf(cam->yaw) * r, CAM_HEIGHT, cosf(cam->yaw) * r);
  cam->view = m4_lookat(cam->pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* Point the sun for this frame: take its resting direction and swing it a
 * little to the side based on the rocking angle, so the shadows slide across
 * the floor without the light ever ending up behind everything. */
static void sun_aim(Sun *sun) {
  Vec3 d = v3_norm(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]));
  float az = LIGHT_ARC_HALF * sinf(sun->yaw);
  float ca = cosf(az), sa = sinf(az);
  sun->dir = v3(d.x * ca + d.z * sa, d.y, -d.x * sa + d.z * ca);
}

/* Build the whole world from scratch. Also the resize path, so it first frees
 * any shapes left from a previous build. */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->objects[i].mesh);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->shadows_on = true;

  s->cam.dist = CAM_DIST_DEF;
  s->cam.yaw = 0.f;

  s->sun.color = v3(SUN_COL[0], SUN_COL[1], SUN_COL[2]);
  s->sun.yaw = 0.f;

  s->objects[OBJ_FLOOR].mesh = mesh_floor(FLOOR_HALF_X, FLOOR_HALF_Z);
  s->objects[OBJ_FLOOR].albedo = v3(0.32f, 0.36f, 0.42f); /* dark slate */
  s->objects[OBJ_FLOOR].model = m4_identity();

  s->objects[OBJ_CUBE].mesh = mesh_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->objects[OBJ_CUBE].albedo = v3(0.78f, 0.62f, 0.42f); /* sandstone */
  s->objects[OBJ_CUBE].model = m4_translate(CUBE_CX, CUBE_CY, CUBE_CZ);

  s->objects[OBJ_SPHERE].mesh = mesh_sphere(SPHERE_R, SPHERE_RINGS, SPHERE_SEGS);
  s->objects[OBJ_SPHERE].albedo = v3(0.78f, 0.78f, 0.82f); /* cool grey */
  s->objects[OBJ_SPHERE].model = m4_translate(SPHERE_CX, SPHERE_CY, SPHERE_CZ);

  camera_rebuild_proj(&s->cam, total_cols, s->scene_rows);
  camera_rebuild_view(&s->cam);
  sun_aim(&s->sun);
}

/* The one spot where the world moves forward each frame: nudge the sun's
 * rocking angle (unless paused) and re-point it. Everything else is drawing. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->sun.yaw += LIGHT_ORBIT_RAD_PER_SEC * dt; /* orbit the sun, not the camera */
  sun_aim(&s->sun);
}

/* ── §10 to the screen — draw the finished colours and the readout ── */

/* Draw every filled-in cell's colour to the terminal. */
static void render_scene(const GBuffer *g, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++)
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++)
      if (g->valid[r][c])
        paint_cell(c, r, g_light[r][c]);
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  char status[120];
  snprintf(status, sizeof status, " %5.1f fps  shadow=%s  zoom=%.1f ", fps,
           s->shadows_on ? "on " : "off", (double)s->cam.dist);
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " SHADOW MAPPING ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "shadow map: %dx%d float NDC z   bias=%.4f   "
           "ortho frustum: %.1f x %.1f x %.1f",
           SHADOW_W, SHADOW_H, (double)SHADOW_BIAS,
           2.0 * (double)LIGHT_ORTHO_HALF, 2.0 * (double)LIGHT_ORTHO_HALF,
           (double)(LIGHT_FAR - LIGHT_NEAR));
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + 1, 0, " q:quit  s:shadow  +/-:zoom  spc:pause  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §11 the program — set up the terminal, then loop ── */

/* Poked by the signal handlers below; this odd type is the only thing a
 * handler is allowed to touch safely. */
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
  atexit(cleanup);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt our drawing */
  color_init();

  int cols, rows;
  getmaxyx(stdscr, rows, cols);

  Scene s;
  memset(&s, 0, sizeof s);
  scene_init(&s, cols, rows);

  int64_t prev = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps = 0.0;
  int64_t frame_ns = 1000000000LL / FPS_TARGET;

  while (g_run) {
    /* window resized? rebuild everything at the new size */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_init(&s, cols, rows);
    }

    /* time since the last frame; capped so a long pause can't make us lurch */
    int64_t now = clock_ns();
    int64_t dt = now - prev;
    if (dt > 100000000LL)
      dt = 100000000LL;
    prev = now;

    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= 500000000LL) {
      fps = (double)fps_cnt * 1e9 / (double)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    /* one frame, in order: move the world, then draw it */
    scene_tick(&s, (float)dt * 1e-9f); /* move the sun a little */

    /* draw it: shadow map from the sun, then the camera's view, then colours */
    if (s.shadows_on)
      shadow_pass(s.objects, N_OBJECTS, s.sun.dir, &g_shadow);
    render_gbuffer(s.objects, N_OBJECTS, &s.cam, &g_gbuf, s.scene_cols,
                   s.scene_rows);
    render_lightpass(&g_gbuf, &g_shadow, &s.sun, &s.cam, s.shadows_on,
                     s.scene_cols, s.scene_rows);

    /* push it all to the screen */
    erase();
    render_scene(&g_gbuf, s.scene_cols, s.scene_rows);
    hud_draw(&s, fps);
    wnoutrefresh(stdscr);
    doupdate();

    /* handle key presses — these just flip settings, they don't run a frame */
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_run = 0;
        break;
      case 's':
      case 'S':
        s.shadows_on = !s.shadows_on;
        break;
      case ' ':
        s.paused = !s.paused;
        break;
      case 'r':
      case 'R':
        s.cam.dist = CAM_DIST_DEF;
        s.cam.yaw = 0.f;
        s.sun.yaw = 0.f;
        camera_rebuild_view(&s.cam);
        sun_aim(&s.sun); /* reset the sun's orbit even while paused */
        break;
      case '+':
      case '=':
        s.cam.dist -= CAM_ZOOM_STEP;
        if (s.cam.dist < CAM_DIST_MIN)
          s.cam.dist = CAM_DIST_MIN;
        camera_rebuild_view(&s.cam);
        break;
      case '-':
      case '_':
        s.cam.dist += CAM_ZOOM_STEP;
        if (s.cam.dist > CAM_DIST_MAX)
          s.cam.dist = CAM_DIST_MAX;
        camera_rebuild_view(&s.cam);
        break;
      }
    }

    /* nap for whatever time is left, to hold a steady frame rate */
    int64_t target = clock_ns();
    int64_t left = frame_ns - (target - now);
    if (left > 0)
      clock_sleep_ns(left);
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s.objects[i].mesh);
  return 0;
}
