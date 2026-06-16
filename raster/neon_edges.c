/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * neon_edges.c — Tron-style glowing outlines on dark shapes
 *
 * A cube, a tetrahedron, and an octahedron drift on a near-black floor, barely
 * lit — but their outlines and the creases between their faces glow bright cyan
 * with a soft halo, while the camera orbits and each shape slowly spins. The
 * trick: draw the scene normally into per-pixel tables, then hunt for cells
 * where depth jumps suddenly (an outline against empty space) or the surface
 * suddenly faces a new direction (a crease) — paint those with a bright colour
 * and let the bloom blur spread the glow.
 *
 * Keys: e edges on/off · b bloom on/off · +/- zoom · space pause · r reset · q quit
 * Read raster/bloom_finale.c first — same lighting + bloom; this adds the edge
 * step in front. Sister files: ssao_pipeline.c (same depth tables),
 * deferred_rendering_pipeline.c (same camera setup).
 * Ideas from: the Sobel edge filter (Sobel & Feldman, 1968); screen-space
 *   outlines (Mitchell et al., GDC 2007); Reinhard tone-map (SIGGRAPH '02).
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/neon_edges.c -o neon -lncurses -lm
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

/* §1.2 size of the per-pixel tables — fixed (no malloc), big enough for a large
 * terminal; anything past the edge is skipped. */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 the orbiting camera. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f
#define NEAR_W_EPS 0.001f /* a corner closer to the eye than this is behind it */

#define CAM_DIST 5.5f
#define CAM_DIST_MIN 3.0f
#define CAM_DIST_MAX 10.0f
#define CAM_ZOOM_STEP 0.4f
#define CAM_EYE_Y 2.4f
#define CAM_LOOK_Y 0.5f
#define CAM_ORBIT_RAD_PER_SEC 0.18f

#define CELL_W 8
#define CELL_H 16

/* §1.4 lighting — kept deliberately dim. The whole look is a near-black scene
 * with glowing outlines; if the sun or the fill light get too bright the lit
 * faces start competing with the glow. Keep every channel well under 0.2. */
static const float SUN_DIR[3] = {-0.55f, -0.65f, 0.30f};
static const float SUN_COL[3] = {0.10f, 0.12f, 0.14f};
static const float AMBIENT_COL[3] = {0.03f, 0.04f, 0.06f};
#define SHININESS 24.0f
#define SPEC_GAIN 0.20f

/* §1.5 edge glow. NEON_COLOR is deliberately over-bright (channels above 1.0)
 * so a finished edge sails past the bloom threshold and gets a halo.
 *   THRESHOLD / KNEE — how strong a depth-or-crease jump has to be before it
 *     counts as an edge, and how gently the glow fades in around that cutoff
 *     (a soft fade hides single-pixel stair-stepping).
 *   DEPTH_SCALE / NORMAL_SCALE — the two edge signals come in different units,
 *     so each is scaled until a real outline and a real crease end up about
 *     equally bright and one threshold works for both. */
static const float NEON_COLOR[3] = {0.50f, 2.00f, 2.50f};
#define EDGE_INTENSITY 1.50f
#define EDGE_THRESHOLD 0.75f
#define EDGE_KNEE 0.45f
#define DEPTH_SCALE 0.05f
#define NORMAL_SCALE 0.40f

/* §1.6 bloom — the soft halo. Threshold low enough that even a faint edge
 * bleeds; intensity high enough that the glow reaches a few cells out. */
#define BLOOM_THRESHOLD 0.90f
#define BLOOM_INTENSITY 1.50f
#define BLOOM_RADIUS 3
#define BLOOM_TAPS (2 * BLOOM_RADIUS + 1)
static const float BLOOM_KERNEL[BLOOM_TAPS] = {
    0.0702f, 0.1311f, 0.1907f, 0.2161f, 0.1907f, 0.1311f, 0.0702f};

/* §1.7 where each shape sits and how big it is — a cube, a tetrahedron, and an
 * octahedron spaced left to right on the floor. */
#define FLOOR_HALF_X 3.0f
#define FLOOR_HALF_Z 3.0f

#define CUBE_HALF 0.55f
#define CUBE_CX -1.40f
#define CUBE_CY 0.65f
#define CUBE_CZ 0.20f

#define TETRA_R 0.85f
#define TETRA_CX 0.00f
#define TETRA_CY 0.70f
#define TETRA_CZ -0.20f

#define OCTA_R 0.70f
#define OCTA_CX 1.40f
#define OCTA_CY 0.75f
#define OCTA_CZ 0.30f

enum {
  OBJ_FLOOR = 0,
  OBJ_CUBE,
  OBJ_TETRA,
  OBJ_OCTA,
  N_OBJECTS,
};

/* §1.8 characters ordered faint-to-dense, so brightness picks a glyph: a space
 * for near-black up to '@' for the brightest cells (Paul Bourke's ramp). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 a 4×4 grid of dither thresholds. Nudging each cell's brightness by its
 * grid value before picking a glyph stops smooth gradients from banding. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.10f
#define LUMA_BOLD_ABOVE 0.85f /* glyph drawn A_BOLD above this brightness */
#define LUMA_DIM_BELOW 0.15f  /* glyph drawn A_DIM below this brightness  */

/* §1.10 colour-pair slots: the 216 RGB-cube colours, plus the HUD's yellow and
 * the hint line's cyan. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* ── §2 clock — a steady timer and a sleep, for pacing frames ────────── */

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

/* ── §3 math — 3-D vectors and 4×4 matrices, the usual graphics toolkit ─ */

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

/* Builds the camera's view from where it sits (eye), what it looks at, and which
 * way is up — the standard glm recipe. */
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

/* When a shape is rotated or scaled, the little "which way the surface faces"
 * arrows can't ride the same matrix or they'd stop being square to the surface.
 * This builds the corrected matrix for them (the inverse-transpose). */
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

/* Rotations around X / Y axes — used for per-object spin. */
static Mat4 m4_rotate_x(float a) {
  Mat4 m = m4_identity();
  float c = cosf(a), s = sinf(a);
  m.m[1][1] = c;
  m.m[1][2] = -s;
  m.m[2][1] = s;
  m.m[2][2] = c;
  return m;
}
static Mat4 m4_rotate_y(float a) {
  Mat4 m = m4_identity();
  float c = cosf(a), s = sinf(a);
  m.m[0][0] = c;
  m.m[0][2] = s;
  m.m[2][0] = -s;
  m.m[2][2] = c;
  return m;
}

/* ── §4 paint — turn an HDR colour into one coloured terminal character ─ */

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
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* How bright a colour looks to the eye (green counts most, blue least). */
static inline float rec709_luma(float r, float g, float b) {
  return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

/* Snap one colour channel (0..1) to one of the 216-cube's 6 steps (0..5). */
static inline int quantize_unit_to_5(float x) {
  int q = (int)(x * 5.f + 0.5f);
  return q < 0 ? 0 : (q > 5 ? 5 : q);
}

/* ncurses pair for the nearest 216-cube colour to `col` (gamma-encoded
 * channels), or the single white fallback pair on an 8-colour terminal. */
static int cube_pair(float r, float g, float b) {
  if (!g_256)
    return PAIR_CUBE_BASE;
  return PAIR_CUBE_BASE + quantize_unit_to_5(r) * 36 + quantize_unit_to_5(g) * 6 +
         quantize_unit_to_5(b);
}

/* Pick the Bourke ramp character for a brightness (0..1): faint chars for dim
 * cells, dense chars for bright ones. */
static inline int ramp_index(float luma) {
  int idx = (int)(luma * (BOURKE_LEN - 1) + 0.5f);
  return idx < 0 ? 0 : (idx >= BOURKE_LEN ? BOURKE_LEN - 1 : idx);
}

static void paint_cell(int sx, int sy, Vec3 col) {
  /* bring each HDR channel into displayable 0..1 range (roll off, then gamma) */
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  /* nudge brightness with an ordered dither so neighbours don't all snap to the
   * same glyph */
  float luma = rec709_luma(r, g, b);
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float luma_dithered = clamp01(luma + dith);

  int pair = cube_pair(r, g, b);
  int glyph = ramp_index(luma_dithered); /* character from the dithered value */
  int attr = (luma > LUMA_BOLD_ABOVE)  ? A_BOLD /* bold/dim from the true value */
             : (luma < LUMA_DIM_BELOW) ? A_DIM
                                       : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[glyph]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh — build each shape's triangles once, at startup ──────────── */

/* One corner of a triangle: where it is, which way the surface faces there, and
 * a texture coordinate we carry along but don't really use. */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;

/* One triangle, as three positions into the mesh's vertex array. */
typedef struct {
  int v[3];
} Triangle;

/* A whole shape: its corners and the triangles joining them. Both arrays are
 * malloc'd once when the shape is built and freed by mesh_free; nvert / ntri are
 * how many of each are actually filled in. */
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
  m->verts[m->nvert++] = (Vertex){p0, nrm, 0.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p1, nrm, 1.f, 0.f};
  m->verts[m->nvert++] = (Vertex){p2, nrm, 1.f, 1.f};
  m->verts[m->nvert++] = (Vertex){p3, nrm, 0.f, 1.f};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  m->tris[m->ntri++] = (Triangle){{v0, v0 + 2, v0 + 3}};
}

/* Builds a faceted shape from its corner points and a list of triangles. Each
 * triangle gets its own three fresh corners so neighbouring faces never share a
 * facing direction — that hard break is exactly what makes the creases between
 * faces light up. Caller frees the returned mesh with mesh_free. */
static Mesh tessellate_polyhedron(const Vec3 *verts, int n_verts,
                                  const int (*faces)[3], int n_faces) {
  (void)n_verts;
  Mesh m;
  m.verts = malloc((size_t)n_faces * 3 * sizeof(Vertex));
  m.tris = malloc((size_t)n_faces * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int f = 0; f < n_faces; f++) {
    Vec3 a = verts[faces[f][0]];
    Vec3 b = verts[faces[f][1]];
    Vec3 c = verts[faces[f][2]];
    Vec3 e1 = v3_sub(b, a);
    Vec3 e2 = v3_sub(c, a);
    Vec3 nrm = v3_norm(v3(e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z,
                          e1.x * e2.y - e1.y * e2.x));
    int v0 = m.nvert;
    m.verts[m.nvert++] = (Vertex){a, nrm, 0.f, 0.f};
    m.verts[m.nvert++] = (Vertex){b, nrm, 1.f, 0.f};
    m.verts[m.nvert++] = (Vertex){c, nrm, 0.f, 1.f};
    m.tris[m.ntri++] = (Triangle){{v0, v0 + 1, v0 + 2}};
  }
  return m;
}

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

/* A 4-sided pyramid (tetrahedron) centred on the origin. Its corners are four
 * alternating corners of a cube, which happen to be evenly spaced. */
static Mesh tessellate_tetrahedron(float r) {
  float s = r / sqrtf(3.f);
  Vec3 verts[4] = {
      v3(s, s, s),
      v3(s, -s, -s),
      v3(-s, s, -s),
      v3(-s, -s, s),
  };
  static const int faces[4][3] = {
      {0, 1, 2},
      {0, 2, 3},
      {0, 3, 1},
      {1, 3, 2},
  };
  return tessellate_polyhedron(verts, 4, faces, 4);
}

/* An 8-faced diamond (octahedron): one point sticking out along each direction
 * of each axis, with a triangle filling in each of the eight corners between. */
static Mesh tessellate_octahedron(float r) {
  Vec3 verts[6] = {
      v3(r, 0, 0), v3(-r, 0, 0), /* 0=+X, 1=-X */
      v3(0, r, 0), v3(0, -r, 0), /* 2=+Y, 3=-Y */
      v3(0, 0, r), v3(0, 0, -r), /* 4=+Z, 5=-Z */
  };
  static const int faces[8][3] = {
      {0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, /* +Z half */
      {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}, /* -Z half */
  };
  return tessellate_polyhedron(verts, 6, faces, 8);
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

/* SceneObject — one thing in the scene: its shape, its colour, where it sits +
 * how it is turned, and how it spins. model is rebuilt from spin_angle each tick
 * (the floor has spin_speed 0, so it never turns). The Scene owns an array of
 * these (§10); defined here because it bundles a Mesh and the geometry pass
 * needs the type. */
typedef struct {
  Mesh mesh;        /* geometry, built once at init (freed by scene_init) */
  Vec3 albedo;      /* flat surface colour                                */
  Mat4 model;       /* where it sits in the world (rebuilt each tick)     */
  float spin_angle; /* current spin angle (radians)                       */
  float spin_speed; /* spin rate (rad/s; 0 = static, e.g. the floor)      */
} SceneObject;

/* ── §6 G-buffer — draw the shapes once into per-cell tables ──────────── */

/* One big set of tables, one entry per screen cell, remembering everything we
 * learned about the surface drawn there — its place, which way it faces, its
 * colour, how near it is — so the later passes can light it and find its edges
 * without touching the triangles again. (The classic "G-buffer" of Saito &
 * Takahashi, 1987.) The drawing pass fills the surface tables; the later passes
 * fill the two colour-output tables. Every table is [row][col]:
 *   what's drawn here (the drawing pass)
 *     pos / normal / albedo — where it is in the world, which way it faces, its
 *                             plain colour
 *     zbuf   — how near it is, used to keep only the closest surface (smaller =
 *              nearer; starts at 1.0 meaning "nothing/far")
 *     z_view — a second, evenly-spaced distance. The edge finder uses THIS one
 *              because equal steps of real distance give equal steps here, while
 *              zbuf is all bunched up near the camera and would mislead it
 *     valid  — did anything get drawn in this cell this frame? (0 = background)
 *   the finished colours (the later passes)
 *     edge   — the bright glow colour for this cell (filled in §7)
 *     light  — the finished lit colour, with the halo mixed in (§8) */
typedef struct {
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];
  float zbuf[GBUF_MAX_H][GBUF_MAX_W];
  float z_view[GBUF_MAX_H][GBUF_MAX_W];
  uint8_t valid[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 edge[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 light[GBUF_MAX_H][GBUF_MAX_W];
} GBuffer;

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

/* For a point inside a screen triangle, work out its three "blend weights" — how
 * much each corner counts — so we can mix the corners' depth, colour, and facing
 * at that exact spot. Returns negative weights when the point is outside. */
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

/* For each of the triangle's three corners, work out the forms the later steps
 * need: a projected position (for placing it on screen and checking it faces
 * us), its spot and facing out in the world (for lighting), and how far back it
 * sits (for the edge finder). */
static void transform_vertices(const Mesh *mesh, const Triangle *tri, Mat4 mvp,
                               Mat4 model, Mat4 modelview, Mat4 norm_mat,
                               Vec4 clip[3], Vec3 wpos[3], Vec3 wnrm[3],
                               float vz[3]) {
  for (int vi = 0; vi < 3; vi++) {
    const Vertex *v = &mesh->verts[tri->v[vi]];
    clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
    wpos[vi] = m4_pt(model, v->pos);
    wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
    vz[vi] = m4_pt(modelview, v->pos).z;
  }
}

/* All three corners are behind the camera (or too close in front of it) — none
 * of this triangle can be seen, so the caller skips it. */
static bool all_behind_near_plane(const Vec4 clip[3]) {
  return clip[0].w < NEAR_W_EPS && clip[1].w < NEAR_W_EPS &&
         clip[2].w < NEAR_W_EPS;
}

/* Turn each corner into a spot on the cell grid, making farther things land
 * closer together so the scene looks 3-D. The up/down value is flipped because
 * screen rows count downward while the math counts up. */
static void project_to_screen(const Vec4 clip[3], int cols, int rows,
                              float sx[3], float sy[3], float sz[3]) {
  for (int vi = 0; vi < 3; vi++) {
    float w = clip[vi].w;
    if (fabsf(w) < 1e-6f)
      w = 1e-6f;
    sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows;
    sz[vi] = clip[vi].z / w;
  }
}

/* Is this triangle the back side of a shape, turned away from us? We check which
 * way its corners wind on screen; with our setup the away-facing ones wind the
 * "wrong" way, and we drop them so we don't draw hidden back faces. */
static bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area >= 0.f;
}

/* Walk the cells the screen triangle covers; for each one inside it and nearer
 * than what's already there, record the surface's facts into the G-buffer. */
static void rasterize_fragments(GBuffer *gb, const float sx[3], const float sy[3],
                                const float sz[3], const float vz[3],
                                const Vec3 wpos[3], const Vec3 wnrm[3],
                                Vec3 albedo, int cols, int rows) {
  int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

  for (int py = y0; py <= y1 && py < GBUF_MAX_H; py++) {
    for (int px = x0; px <= x1 && px < GBUF_MAX_W; px++) {
      float b[3];
      barycentric(sx, sy, (float)px + 0.5f, (float)py + 0.5f, b);
      if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
        continue;

      float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
      if (z >= gb->zbuf[py][px])
        continue;

      gb->zbuf[py][px] = z;
      gb->z_view[py][px] = b[0] * vz[0] + b[1] * vz[1] + b[2] * vz[2];
      gb->pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
      gb->normal[py][px] =
          v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
      gb->albedo[py][px] = albedo;
      gb->valid[py][px] = 1;
    }
  }
}

static void rasterize_object(GBuffer *gb, const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 modelview, Mat4 norm_mat,
                             int cols, int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    float vz[3];
    transform_vertices(mesh, tri, mvp, model, modelview, norm_mat, clip, wpos,
                       wnrm, vz);
    if (all_behind_near_plane(clip))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(clip, cols, rows, sx, sy, sz);
    if (is_back_facing(sx, sy))
      continue;

    rasterize_fragments(gb, sx, sy, sz, vz, wpos, wnrm, albedo, cols, rows);
  }
}

static void render_gbuffer(GBuffer *gb, const SceneObject *objects,
                           int n_objects, Mat4 view, Mat4 proj, int cols,
                           int rows) {
  gbuffer_clear(gb, cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 model = objects[oi].model;
    Mat4 mv = m4_mul(view, model);
    Mat4 mvp = m4_mul(proj, mv);
    Mat4 nmat = m4_normal_mat(model);
    rasterize_object(gb, &objects[oi].mesh, objects[oi].albedo, mvp, model, mv,
                     nmat, cols, rows);
  }
}

/* ── §7 edge — find the cells where depth or facing changes sharply ───── */

/* Measures how fast the nine values in a little 3×3 patch are changing — a big
 * number means a sharp jump, which is what an edge looks like. (The Sobel
 * filter.) The patch is laid out row by row: 0-2 top, 3-5 middle, 6-8 bottom. */
static float sobel_magnitude(const float field[9]) {
  float gx = (field[2] + 2.f * field[5] + field[8]) -
             (field[0] + 2.f * field[3] + field[6]);
  float gy = (field[6] + 2.f * field[7] + field[8]) -
             (field[0] + 2.f * field[1] + field[2]);
  return sqrtf(gx * gx + gy * gy);
}

static inline int clamp_ix(int v, int max) {
  return v < 0 ? 0 : (v >= max ? max - 1 : v);
}

/* Grab the depths of a cell and its 8 neighbours. Any neighbour that's empty
 * background counts as "very far away" — so at the outline of a shape, where
 * background meets surface, the depths jump hugely and the edge stands out. */
static void sample_zview_3x3(const GBuffer *gb, int r, int c, int cols, int rows,
                             float out[9]) {
  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      int rr = clamp_ix(r + dr, rows);
      int cc = clamp_ix(c + dc, cols);
      out[(dr + 1) * 3 + (dc + 1)] =
          gb->valid[rr][cc] ? gb->z_view[rr][cc] : -CAM_FAR;
    }
  }
}

/* Grab one piece of the facing direction (x, y, or z) for a cell and its 8
 * neighbours. Empty background counts as 0; creases show up as a sharp change in
 * facing between two faces, and outlines are already handled by depth. */
static void sample_normal_3x3(const GBuffer *gb, int r, int c, int axis,
                              int cols, int rows, float out[9]) {
  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      int rr = clamp_ix(r + dr, rows);
      int cc = clamp_ix(c + dc, cols);
      float v = 0.f;
      if (gb->valid[rr][cc]) {
        Vec3 n = gb->normal[rr][cc];
        v = (axis == 0) ? n.x : (axis == 1) ? n.y : n.z;
      }
      out[(dr + 1) * 3 + (dc + 1)] = v;
    }
  }
}

/* A soft on-switch: returns 0 below edge0, 1 above edge1, and eases smoothly
 * between — used so edges fade in instead of popping on with jagged stair-steps. */
static float smoothstep(float edge0, float edge1, float x) {
  float t = (x - edge0) / (edge1 - edge0);
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  return t * t * (3.f - 2.f * t);
}

/* How strongly this cell sits on an outline: how much the depth jumps around it. */
static float depth_gradient(const GBuffer *gb, int r, int c, int cols,
                            int rows) {
  float field[9];
  sample_zview_3x3(gb, r, c, cols, rows, field);
  return sobel_magnitude(field);
}

/* How strongly this cell sits on a crease: how much the surface's facing
 * direction changes around it, totalled over all three direction components. */
static float normal_gradient(const GBuffer *gb, int r, int c, int cols,
                             int rows) {
  float field[9];
  float total = 0.f;
  for (int axis = 0; axis < 3; axis++) {
    sample_normal_3x3(gb, r, c, axis, cols, rows, field);
    total += sobel_magnitude(field);
  }
  return total;
}

/* For every drawn cell, score how outline-like and how crease-like it is, take
 * whichever is stronger, fade it softly above the cutoff, and store that much
 * over-bright cyan in the edge table. Empty cells get no glow. The lightpass
 * adds this straight into the lit colour, and bloom turns it into a halo. */
static void edge_pass(GBuffer *gb, int cols, int rows) {
  Vec3 neon = v3(NEON_COLOR[0], NEON_COLOR[1], NEON_COLOR[2]);

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        gb->edge[r][c] = v3(0, 0, 0);
        continue;
      }

      float depth_grad = depth_gradient(gb, r, c, cols, rows);
      float normal_grad = normal_gradient(gb, r, c, cols, rows);

      float edge = fmaxf(depth_grad * DEPTH_SCALE, normal_grad * NORMAL_SCALE);
      float t = smoothstep(EDGE_THRESHOLD, EDGE_THRESHOLD + EDGE_KNEE, edge);

      gb->edge[r][c] = v3_scale(neon, t * EDGE_INTENSITY);
    }
  }
}

/* ── §8 lightpass — softly light each surface cell, then add its glow ──── */

/* The soft surface shading, but with a twist (Valve's "half-Lambert"): even the
 * side turned away from the sun keeps a faint gradient instead of going dead
 * flat. The brightest point doesn't change and the sun is very dim anyway, so
 * this only brings back a little shape on the dark side at chunky terminal
 * resolution — the near-black look and the glowing edges stay intact. */
static Vec3 half_lambert_diffuse(Vec3 N, Vec3 L, Vec3 albedo, Vec3 sun_col) {
  float wrap = 0.5f * v3_dot(N, L) + 0.5f;
  float diff = wrap * wrap;
  return v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
            albedo.z * sun_col.z * diff);
}

/* The shiny highlight — bright where the surface is angled just right to bounce
 * the sun toward the eye. It's the sun's own colour, not the surface's. */
static Vec3 blinn_phong_specular(Vec3 N, Vec3 L, Vec3 V, Vec3 sun_col) {
  Vec3 H = v3_norm(v3_add(L, V));
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
  return v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);
}

/* The colour of one surface cell: a little fill light, plus the soft sun
 * shading, plus the shiny highlight, plus the edge glow when edges are on. The
 * glow can push it well past full brightness — bloom and the final squash deal
 * with that. */
static Vec3 shade_pixel(const GBuffer *gb, int r, int c, Vec3 L, Vec3 sun_col,
                        Vec3 ambient, Vec3 cam_pos, bool edges_on) {
  Vec3 P = gb->pos[r][c];
  Vec3 N = gb->normal[r][c];
  Vec3 albedo = gb->albedo[r][c];

  Vec3 amb =
      v3(ambient.x * albedo.x, ambient.y * albedo.y, ambient.z * albedo.z);
  Vec3 dif = half_lambert_diffuse(N, L, albedo, sun_col);
  Vec3 V = v3_norm(v3_sub(cam_pos, P));
  Vec3 sp = blinn_phong_specular(N, L, V, sun_col);
  Vec3 edge = edges_on ? gb->edge[r][c] : v3(0, 0, 0);

  return v3(amb.x + dif.x + sp.x + edge.x, amb.y + dif.y + sp.y + edge.y,
            amb.z + dif.z + sp.z + edge.z);
}

static void render_lightpass(GBuffer *gb, Vec3 cam_pos, bool edges_on, int cols,
                             int rows) {
  Vec3 sun_dir = v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2]);
  Vec3 sun_col = v3(SUN_COL[0], SUN_COL[1], SUN_COL[2]);
  Vec3 ambient = v3(AMBIENT_COL[0], AMBIENT_COL[1], AMBIENT_COL[2]);
  Vec3 L = v3_norm(v3_neg(sun_dir));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        gb->light[r][c] = v3(0, 0, 0);
        continue;
      }
      gb->light[r][c] =
          shade_pixel(gb, r, c, L, sun_col, ambient, cam_pos, edges_on);
    }
  }
}

/* ── §9 bloom — spread the bright edges into a soft halo ──────────────── */

/* Two scratch tables: the bright pixels we pulled out, and a half-blurred copy.
 * Reused every frame, so they live here as plain globals. */
static Vec3 g_bloom[GBUF_MAX_H][GBUF_MAX_W];
static Vec3 g_bloom_tmp[GBUF_MAX_H][GBUF_MAX_W];

static void bloom_extract(const GBuffer *gb, float threshold, int cols,
                          int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 lit = gb->light[r][c];
      g_bloom[r][c] = (v3_luma(lit) > threshold) ? lit : v3(0, 0, 0);
    }
  }
}

static void bloom_blur_h(int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      Vec3 sum = v3(0, 0, 0);
      for (int k = 0; k < BLOOM_TAPS; k++) {
        int sc = clamp_ix(c + k - BLOOM_RADIUS, cols);
        float w = BLOOM_KERNEL[k];
        sum = v3_add(sum, v3_scale(g_bloom[r][sc], w));
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
        sum = v3_add(sum, v3_scale(g_bloom_tmp[sr][c], w));
      }
      g_bloom[r][c] = sum;
    }
  }
}

static void bloom_composite(GBuffer *gb, float intensity, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      gb->light[r][c] =
          v3_add(gb->light[r][c], v3_scale(g_bloom[r][c], intensity));
    }
  }
}

/* The whole halo effect: keep only the bright pixels, blur them sideways then
 * up-and-down, and add the soft result back onto the lit image. */
static void bloom_pass(GBuffer *gb, int cols, int rows) {
  bloom_extract(gb, BLOOM_THRESHOLD, cols, rows);
  bloom_blur_h(cols, rows);
  bloom_blur_v(cols, rows);
  bloom_composite(gb, BLOOM_INTENSITY, cols, rows);
}

/* ── §10 scene — all the moving state, advanced once per frame ────────── */

/* The orbiting camera. Two knobs you actually set — how far around it has swung
 * and how far out it sits — and three things worked out from them each time
 * those change (where the eye ends up, and the two matrices the renderer needs).
 *   yaw  — how far around the scene it has swung (radians)
 *   dist — how far out the eye sits (the zoom)
 *   pos  — where the eye actually is, worked out from yaw + dist
 *   view — looks from the eye toward the scene centre
 *   proj — makes far things smaller, matched to the window shape */
typedef struct {
  float yaw;
  float dist;
  Vec3 pos;
  Mat4 view;
  Mat4 proj;
} Camera;

/* Scene — everything the demo is about, in one place:
 *   WHAT  — the renderable objects (the floor + the three spinning shapes)
 *   HOW   — the knobs the keys change (edge glow, bloom, pause)
 *   WHERE — the camera
 * plus how big the drawing area is. Written only by scene_init / scene_tick and
 * the key handler; the render passes read it but never change it. */
typedef struct {
  /* what's drawn */
  SceneObject objects[N_OBJECTS];

  /* what the keys change */
  bool edges_on; /* 'e' — Sobel edge glow on/off       */
  bool bloom_on; /* 'b' — soft bloom halo on/off       */
  bool paused;   /* space — freeze spin + camera orbit */

  /* where we view from */
  Camera cam;

  /* drawing area in cells (full width; height minus the HUD band) */
  int scene_cols, scene_rows;
} Scene;

static void camera_rebuild_proj(Camera *cam, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  cam->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void camera_rebuild_view(Camera *cam) {
  float r = cam->dist;
  cam->pos = v3(sinf(cam->yaw) * r, CAM_EYE_Y, cosf(cam->yaw) * r);
  cam->view = m4_lookat(cam->pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* Places a shape at (cx, cy, cz) and spins it by `angle` (a bit more around the
 * up-axis than the side-axis, so it tumbles). The turning is what keeps swinging
 * fresh outlines and creases into view. */
static Mat4 spin_model(float cx, float cy, float cz, float angle) {
  Mat4 ry = m4_rotate_y(angle);
  Mat4 rx = m4_rotate_x(angle * 0.6f);
  Mat4 t = m4_translate(cx, cy, cz);
  return m4_mul(t, m4_mul(ry, rx));
}

static void scene_init(Scene *s, int total_cols, int total_rows) {
  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&s->objects[i].mesh);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->edges_on = true;
  s->bloom_on = true;
  s->cam.dist = CAM_DIST;
  s->cam.yaw = 0.f;

  /* OBJ_FLOOR — near-black slate; doesn't spin. */
  s->objects[OBJ_FLOOR].mesh = tessellate_quad(
      v3(-FLOOR_HALF_X, 0.f, FLOOR_HALF_Z), v3(2 * FLOOR_HALF_X, 0.f, 0.f),
      v3(0.f, 0.f, -2 * FLOOR_HALF_Z), v3(0.f, 1.f, 0.f));
  s->objects[OBJ_FLOOR].albedo = v3(0.06f, 0.07f, 0.09f);
  s->objects[OBJ_FLOOR].model = m4_identity();
  s->objects[OBJ_FLOOR].spin_speed = 0.f;

  /* OBJ_CUBE — 6 faces, 12 creases. Slow spin. */
  s->objects[OBJ_CUBE].mesh = tessellate_box(CUBE_HALF, CUBE_HALF, CUBE_HALF);
  s->objects[OBJ_CUBE].albedo = v3(0.10f, 0.10f, 0.13f);
  s->objects[OBJ_CUBE].spin_speed = 0.45f;
  s->objects[OBJ_CUBE].model = spin_model(CUBE_CX, CUBE_CY, CUBE_CZ, 0.f);

  /* OBJ_TETRA — 4 faces, 6 creases. Faster spin (smaller poly count). */
  s->objects[OBJ_TETRA].mesh = tessellate_tetrahedron(TETRA_R);
  s->objects[OBJ_TETRA].albedo = v3(0.10f, 0.10f, 0.13f);
  s->objects[OBJ_TETRA].spin_speed = 0.65f;
  s->objects[OBJ_TETRA].model = spin_model(TETRA_CX, TETRA_CY, TETRA_CZ, 0.f);

  /* OBJ_OCTA — 8 faces, 12 creases, sharp star silhouette. */
  s->objects[OBJ_OCTA].mesh = tessellate_octahedron(OCTA_R);
  s->objects[OBJ_OCTA].albedo = v3(0.10f, 0.10f, 0.13f);
  s->objects[OBJ_OCTA].spin_speed = -0.55f; /* opposite direction */
  s->objects[OBJ_OCTA].model = spin_model(OCTA_CX, OCTA_CY, OCTA_CZ, 0.f);

  camera_rebuild_proj(&s->cam, total_cols, s->scene_rows);
  camera_rebuild_view(&s->cam);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  s->cam.yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  camera_rebuild_view(&s->cam);

  s->objects[OBJ_CUBE].spin_angle += s->objects[OBJ_CUBE].spin_speed * dt;
  s->objects[OBJ_TETRA].spin_angle += s->objects[OBJ_TETRA].spin_speed * dt;
  s->objects[OBJ_OCTA].spin_angle += s->objects[OBJ_OCTA].spin_speed * dt;

  s->objects[OBJ_CUBE].model =
      spin_model(CUBE_CX, CUBE_CY, CUBE_CZ, s->objects[OBJ_CUBE].spin_angle);
  s->objects[OBJ_TETRA].model =
      spin_model(TETRA_CX, TETRA_CY, TETRA_CZ, s->objects[OBJ_TETRA].spin_angle);
  s->objects[OBJ_OCTA].model =
      spin_model(OCTA_CX, OCTA_CY, OCTA_CZ, s->objects[OBJ_OCTA].spin_angle);
}

/* ── §11 screen — build the frame in memory, then paint it ────────────── */

/* Builds one finished frame in the per-cell tables: draw the shapes, find their
 * edges, light everything, and spread the halo. Edges and halo are skippable
 * (the 'e' and 'b' toggles). */
static void render_frame(const Scene *s, GBuffer *gb) {
  render_gbuffer(gb, s->objects, N_OBJECTS, s->cam.view, s->cam.proj,
                 s->scene_cols, s->scene_rows);

  if (s->edges_on)
    edge_pass(gb, s->scene_cols, s->scene_rows);

  render_lightpass(gb, s->cam.pos, s->edges_on, s->scene_cols, s->scene_rows);

  if (s->bloom_on)
    bloom_pass(gb, s->scene_cols, s->scene_rows);
}

static void render_scene(const Scene *s, const GBuffer *gb) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c])
        continue;
      paint_cell(c, r, gb->light[r][c]);
    }
  }
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int total_tris = 0;
  for (int i = 0; i < N_OBJECTS; i++)
    total_tris += s->objects[i].mesh.ntri;

  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  edges:%s  bloom:%s  zoom:%.1f  tris:%d  %s ", fps,
           s->edges_on ? "ON " : "OFF", s->bloom_on ? "ON " : "OFF",
           (double)s->cam.dist, total_tris, s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " NEON EDGES · SOBEL + BLOOM ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "passes: gbuffer -> edge(Sobel on z+normal) -> "
           "lightpass(HDR) -> bloom -> paint");
  mvprintw(hr + 1, 1,
           "edge: threshold=%.2f knee=%.2f  depth_scale=%.2f normal_scale=%.2f",
           (double)EDGE_THRESHOLD, (double)EDGE_KNEE, (double)DEPTH_SCALE,
           (double)NORMAL_SCALE);
  mvprintw(hr + 2, 1,
           "Toggle 'e' off: shapes go nearly invisible.   "
           "'b' off: edges are hard pixel lines, no glow.");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  e:edges  b:bloom  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §12 app — startup, the main loop, signals, and keys ──────────────── */

/* Everything the running program needs to hold onto: the scene, the current
 * terminal size, and two flags the signal handlers flip — one to quit, one to
 * note the window changed size. The flags are sig_atomic_t because a signal can
 * write them at any moment. */
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
  typeahead(-1); /* stop ncurses peeking at input mid-draw, which causes tearing */
  color_init();
}

/* Rebuild for the new terminal size after a resize. The endwin + refresh dance
 * is how ncurses is told to pick up the new dimensions. */
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
  case 'e':
  case 'E':
    s->edges_on = !s->edges_on;
    break;
  case 'b':
  case 'B':
    s->bloom_on = !s->bloom_on;
    break;
  case '=':
  case '+':
    s->cam.dist -= CAM_ZOOM_STEP;
    if (s->cam.dist < CAM_DIST_MIN)
      s->cam.dist = CAM_DIST_MIN;
    camera_rebuild_view(&s->cam);
    break;
  case '-':
  case '_':
    s->cam.dist += CAM_ZOOM_STEP;
    if (s->cam.dist > CAM_DIST_MAX)
      s->cam.dist = CAM_DIST_MAX;
    camera_rebuild_view(&s->cam);
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

  App *app = &g_app;
  app->running = 1;

  screen_init();
  getmaxyx(stdscr, app->total_rows, app->total_cols);
  scene_init(&app->scene, app->total_cols, app->total_rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_display = 0.0;

  /* The heartbeat: move the world a little, draw it, read a key. Repeat. */
  while (app->running) {

    /* did the window change size? rebuild for it before anything else */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* how long since the last frame? (capped, so one hiccup can't make the
     * shapes leap across the screen) */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* move everything forward by that much — the one place the world changes */
    scene_tick(&app->scene, dt_sec);

    /* refresh the fps readout every so often */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* nap until it's time for the next frame, before we touch the screen */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    /* draw: build the frame in memory, paint it, lay the HUD on top, show it */
    Scene *s = &app->scene;
    erase();
    render_frame(s, &g_gbuf);
    render_scene(s, &g_gbuf);
    hud_draw(s, fps_display);
    screen_present();

    /* read one key — it may pause, reset, toggle a stage, zoom, or quit */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  for (int i = 0; i < N_OBJECTS; i++)
    mesh_free(&app->scene.objects[i].mesh);

  endwin();
  return 0;
}
