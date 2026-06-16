/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * torus_raster.c — a spinning, smooth-shaded donut drawn into the terminal with
 * a tiny software 3-D renderer. Press 's' to cycle four looks: lit (phong),
 * cartoon bands (toon), a colour-by-facing debug view (normals), and wireframe.
 *
 * Sister files (same renderer, different shape): raster/sphere_raster.c,
 * raster/cube_raster.c, raster/displace_raster.c. The lit shader's look is
 * borrowed from sphere_raster.c. Inspired by Andy Sloane's donut.c (1986).
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

/* ── §1 config ───────────────────────────────────────────────────────── */

enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_COLS = 38,

  /* How finely we chop the donut into triangles: more = smoother but slower.
   * TESS_U goes around the big ring, TESS_V around the tube. 32×24 looks round
   * enough at terminal sizes. */
  TESS_U = 32,
  TESS_V = 24,
};

/* Camera */
#define CAM_FOV (60.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 3.2f

/* Donut size */
#define TORUS_R 0.65f /* ring radius — donut centre to the middle of the tube */
#define TORUS_r 0.28f /* tube radius — how fat the tube is                    */

/* How fast it spins (radians per second) */
#define ROT_Y 0.70f
#define ROT_X 0.28f

/*
 * Lighting (frag_phong) — ported from sphere_raster.c and tuned for the chunky
 * terminal grid: a soft "wrap" so the shadow side stays readable rather than
 * flat black, a gentle sun glint, and a cool rim glow that outlines the donut
 * (and the hole) against the dark.
 */
#define SPEC_GAIN 0.35f /* brightness of the sun glint                 */
#define RIM_POWER 2.5f  /* rim-glow thickness (higher = thinner band)  */
#define RIM_GAIN 0.40f  /* rim-glow brightness                         */
#define ATMO_R 0.35f    /* rim-glow colour — cool blue                 */
#define ATMO_G 0.60f
#define ATMO_B 1.00f

/* How thick the wireframe lines are: a pixel this close to a triangle edge gets
 * drawn (bigger = thicker lines). */
#define WIRE_THRESH 0.08f

/* Cartoon-shader steps (see frag_toon). */
#define TOON_SPEC_CUT 0.94f /* a glint this sharp gets a hard white dot     */
#define TOON_SPEC 0.7f      /* that dot's brightness                        */
#define TOON_FLOOR 0.12f    /* dimmest band, so the shaded side isn't black */

/* Rasteriser guards (see §6). */
#define NEAR_CLIP_W 0.001f /* a corner this close behind the eye counts as off-screen */
#define W_DIVIDE_EPS 1e-6f /* never divide by a w smaller than this              */

/* The characters we draw with, faintest to densest (Paul Bourke's ramp). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* Dither: a small repeating grid of brightness nudges that breaks up banding. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};

/* Luma → terminal cell (see rgb_to_cell). */
#define DITHER_AMP 0.15f   /* dither strength — hides brightness banding */
#define BOLD_LUMA 0.6f     /* cells brighter than this are drawn bold     */
#define HUE_MIN_CHROMA 0.08f /* below this the colour has no usable hue   */

/* A terminal cell is about twice as tall as it is wide. We feed that shape into
 * the camera so the donut comes out round instead of squashed into an oval. */
#define CELL_W 8
#define CELL_H 16

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ── §2 math (V3, V4, Mat4) ──────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} Vec3;
typedef struct {
  float x, y, z, w;
} Vec4;
typedef struct {
  float m[4][4];
} Mat4;

/* ── Vec3 ────────────────────────────────────────────────────────── */
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

/* Blend three corner values by weights that add up to 1 — how we smooth a value
 * across a triangle. */
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
}

/* ── Vec4 ────────────────────────────────────────────────────────── */
static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}

/* ── Mat4 ────────────────────────────────────────────────────────── */
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

/* Move a point through the matrix (a position — it shifts with translation). */
static inline Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

/* Move a direction through the matrix (an arrow — translation is ignored). */
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

/* The camera lens: makes far-away things smaller on screen. `aspect` is the
 * terminal's true width-to-height in pixels (not cells), so the donut stays
 * round rather than stretching into a tall oval. */
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

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 r = v3_norm((Vec3){f.y * up.z - f.z * up.y, f.z * up.x - f.x * up.z,
                          f.x * up.y - f.y * up.x});
  Vec3 u = (Vec3){r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z,
                  r.x * f.y - r.y * f.x};
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

/* The matrix for turning a surface's facing direction along with the model. You
 * can't just reuse the model matrix — if the model stretches unevenly, that
 * would tilt the directions wrong. This builds the corrected one. (We normalise
 * the result later, so its overall scale doesn't matter.) */
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

/* ── §3 shaders — how each look is computed ──
 *
 * A "shader" is two small functions: one runs per triangle corner (vertex
 * shader), one runs per pixel (fragment shader). The renderer in §6 calls the
 * corner one first, smoothly blends its outputs across the triangle, then calls
 * the pixel one to pick a colour.
 *
 * The `custom[4]` slots in VSOut/FSIn are a spare channel each shader can use
 * however it likes — the renderer blends them across the triangle just like the
 * position and normal, so a shader can stash extra per-corner data (the normals
 * view stores the facing direction; wireframe stores edge distances) and read it
 * back per pixel for free. */

/* What the vertex shader reads: one mesh vertex in model (object) space. */
typedef struct {
  Vec3 pos;    /* model space */
  Vec3 normal; /* model space */
  float u, v;
} VSIn;

/* What the vertex shader writes per vertex. The pipeline blends these three
 * (one per triangle corner) across the triangle to produce each FSIn. */
typedef struct {
  Vec4 clip_pos;  /* where the corner lands on screen (the renderer needs this) */
  Vec3 world_pos; /* where it is in the world */
  Vec3 world_nrm; /* which way it faces       */
  float u, v;
  float custom[4]; /* spare per-corner data (see §3) */
} VSOut;

/* What the fragment shader reads at one pixel: the surface values blended to
 * that exact spot, plus the cell coordinates (used for the dither pattern). */
typedef struct {
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
  int px, py; /* screen cell coordinates — for dither pattern */
} FSIn;

/* What the fragment shader returns for one pixel: its colour, or a request to
 * skip the pixel entirely (used by the wireframe shader). */
typedef struct {
  Vec3 color;
  bool discard; /* true = pipeline skips this cell entirely */
} FSOut;

typedef void (*VertShaderFn)(const VSIn *in, VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in, FSOut *out, const void *uni);

/* One complete shader: its vertex + fragment functions and the uniform blocks
 * each reads. Switching the look is just rebuilding this (see scene_build_shader). */
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni; /* passed to vert() */
  const void *frag_uni; /* passed to frag() */
} ShaderProgram;

/* ── Uniforms ────────────────────────────────────────────────────── */

/* The constants every shader reads for one frame's draw — "uniforms" is the
 * graphics term: the same values for every vertex and pixel in the draw. Three
 * groups: the transforms that place the torus on screen, the light + camera, and
 * the material the torus is made of. */
typedef struct {
  /* transforms */
  Mat4 model;
  Mat4 view;
  Mat4 proj;
  Mat4 mvp;      /* proj * view * model — precomputed each frame */
  Mat4 norm_mat; /* transforms normals correctly under the model matrix */

  /* light + camera */
  Vec3 light_pos;
  Vec3 light_col;
  Vec3 ambient;
  Vec3 cam_pos;

  /* material */
  Vec3 obj_color;
  float shininess;
} Uniforms;

/* The toon shader's extra setting (how many shade bands), on top of the normal
 * uniforms. Uniforms sits first on purpose, so a pointer to this also works
 * anywhere a plain Uniforms* is expected. */
typedef struct {
  Uniforms base;
  int bands;
} ToonUniforms;

/* ── §3a vertex shaders — place each corner; stash any extra per look ── */

/* The shared work every vertex shader does: figure out where the corner lands on
 * screen, and where it is and which way it faces in the world (for lighting). */
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

/* Like the default, but also tucks the facing direction into the spare channel
 * so the normals view can read it back per pixel (it could read the normal
 * directly — this just shows the spare channel in use). */
static void vert_normals(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
  out->custom[0] = out->world_nrm.x;
  out->custom[1] = out->world_nrm.y;
  out->custom[2] = out->world_nrm.z;
}

/* Just the default work; the renderer fills in the wireframe's edge-distance tags
 * after this runs (see §6). */
static void vert_wire(const VSIn *in, VSOut *out, const void *u_) {
  vert_base(in, out, (const Uniforms *)u_);
}

/* ── §3b fragment shaders — pick each pixel's colour ── */

/* How lit a point is, softened: a plain "is it facing the sun?" test makes the
 * shadow side go flat and dark, which reads badly on the coarse grid. This wraps
 * the light around so it fades gently from full day to night (Valve's trick). */
static inline float half_lambert(float ndotl) {
  float wrap = 0.5f * ndotl + 0.5f;
  return wrap * wrap;
}

/*
 * frag_phong — the lit donut.  Same idea as sphere_raster.c: a warm sun with a
 * broad day→night falloff, a soft glint where the sun reflects toward the eye,
 * and a cool rim glow that lights up the silhouette (and the hole's edge).  The
 * wrap + rim are what keep the shape readable on a terminal's handful of shades.
 */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;

  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V)); /* direction halfway between light and eye */

  float day = half_lambert(v3_dot(N, L)); /* broad lit → shadow falloff */
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess) * SPEC_GAIN;
  float rim = powf(1.f - fmaxf(0.f, v3_dot(N, V)), RIM_POWER) * RIM_GAIN;

  Vec3 c = u->obj_color;
  Vec3 atmo = v3(ATMO_R, ATMO_G, ATMO_B);

  /* ambient + sunlit surface + sun glint + cool rim glow */
  float r = u->ambient.x + c.x * u->light_col.x * day + spec * u->light_col.x + atmo.x * rim;
  float g = u->ambient.y + c.y * u->light_col.y * day + spec * u->light_col.y + atmo.y * rim;
  float b = u->ambient.z + c.z * u->light_col.z * day + spec * u->light_col.z + atmo.z * rim;

  /* gamma correction */
  out->color.x = powf(fminf(r, 1.f), 1.f / 2.2f);
  out->color.y = powf(fminf(g, 1.f), 1.f / 2.2f);
  out->color.z = powf(fminf(b, 1.f), 1.f / 2.2f);
  out->discard = false;
}

/* Cartoon look: snap the smooth lighting into a few flat brightness steps, so
 * the surface reads as bold bands instead of a gradient, with one hard white
 * dot where the glint is strongest. */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;

  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float diff = fmaxf(0.f, v3_dot(N, L));
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > TOON_SPEC_CUT) ? TOON_SPEC : 0.0f;

  Vec3 c = u->obj_color;
  out->color.x = fminf(c.x * (banded + TOON_FLOOR) + spec, 1.f);
  out->color.y = fminf(c.y * (banded + TOON_FLOOR) + spec, 1.f);
  out->color.z = fminf(c.z * (banded + TOON_FLOOR) + spec, 1.f);
  out->discard = false;
}

/* A debug view (and a pretty one): colour each pixel by which way the surface
 * faces — right is red, up is green, toward the camera is blue. No lighting at
 * all. On a donut the facing direction turns smoothly both ways around, so you
 * get a rainbow that wraps around the ring and around the tube. */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/* Draw only the triangle edges. The spare channel carries each pixel's distance
 * to the three edges; a pixel near any edge is painted, the rest are skipped —
 * leaving just the mesh lines. */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  float b0 = in->custom[0];
  float b1 = in->custom[1];
  float b2 = in->custom[2];
  float edge = fminf(b0, fminf(b1, b2)); /* distance to the nearest edge */

  if (edge > WIRE_THRESH) {
    out->discard = true;
    return;
  }
  /* fade slightly toward the edge for a smoother line */
  float t = edge / WIRE_THRESH;
  out->color = v3(0.9f - t * 0.3f, 0.9f - t * 0.3f, 0.9f - t * 0.3f);
  out->discard = false;
}

/* ── shader names for HUD ─────────────────────────────────────────── */
typedef enum { SH_PHONG = 0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong", "toon", "normals", "wire"};

/* ── §4 mesh — build the donut out of triangles ── */

/* One corner of the surface: where it is, which way it faces, and its position
 * on the texture grid (u around the ring, v around the tube). */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;

/* A triangle as three indices into the vertex array. */
typedef struct {
  int v[3];
} Triangle;

/* The torus as triangles: a flat array of corners and a flat array of triangles
 * indexing into it. Built once at startup and never changes (only the camera
 * and the model matrix move it). */
typedef struct {
  Vertex *verts;
  int nvert;
  Triangle *tris;
  int ntri;
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/*
 * torus_vertex — one point on the torus surface for grid coordinates
 * (u, v) ∈ [0,1]: u sweeps around the ring, v around the tube. The outward
 * normal is just the direction from the ring's centreline out to the point —
 * the torus's closed-form-normal trick, no derivatives needed.
 */
static Vertex torus_vertex(float u, float v) {
  float theta = u * 2.f * 3.14159265f; /* angle around the ring */
  float phi = v * 2.f * 3.14159265f;   /* angle around the tube */
  float ct = cosf(theta), st = sinf(theta);
  float cp = cosf(phi), sp = sinf(phi);

  Vec3 pos = v3((TORUS_R + TORUS_r * cp) * ct, TORUS_r * sp,
                (TORUS_R + TORUS_r * cp) * st);
  Vec3 ring_centre = v3(TORUS_R * ct, 0.f, TORUS_R * st);
  Vec3 nrm = v3_norm(v3_sub(pos, ring_centre)); /* outward */

  return (Vertex){pos, nrm, u, v};
}

/*
 * tessellate_torus — build the torus mesh once: place a vertex at every point of
 * a (TESS_U+1) × (TESS_V+1) grid, then stitch each grid square into two
 * triangles. The triangle winding makes "outward = front" for the back-face cull.
 */
static Mesh tessellate_torus(void) {
  int nu = TESS_U, nv = TESS_V;
  int nvert = (nu + 1) * (nv + 1);
  int ntri = nu * nv * 2;

  Mesh m;
  m.verts = malloc((size_t)nvert * sizeof(Vertex));
  m.tris = malloc((size_t)ntri * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* Step 1 — a vertex at every (u, v) grid point. */
  for (int i = 0; i <= nu; i++)
    for (int j = 0; j <= nv; j++)
      m.verts[m.nvert++] = torus_vertex((float)i / (float)nu, (float)j / (float)nv);

  /* Step 2 — stitch each grid square into two triangles.
   *   r0 = (i,   j)      r1 = (i,   j+1)
   *   r2 = (i+1, j)      r3 = (i+1, j+1)                       */
  for (int i = 0; i < nu; i++) {
    for (int j = 0; j < nv; j++) {
      int r0 = i * (nv + 1) + j, r1 = r0 + 1;
      int r2 = r0 + (nv + 1), r3 = r2 + 1;
      m.tris[m.ntri++] = (Triangle){{r0, r2, r1}};
      m.tris[m.ntri++] = (Triangle){{r1, r2, r3}};
    }
  }

  return m;
}

/* ── §5 the off-screen canvas — depth + colour buffers, then paint ── */

/* One drawn terminal cell: its character, colour pair, and whether it's bold. */
typedef struct {
  char ch;
  int color_pair;
  bool bold;
} Cell;

/* Where we render before touching the screen: two screen-sized grids — a depth
 * buffer that remembers how near the closest surface drawn at each cell is (so
 * closer surfaces win), and a colour buffer of the Cells to paint. Reallocated
 * on resize. */
typedef struct {
  float *zbuf; /* [cols*rows]  FLT_MAX = empty */
  Cell *cbuf;  /* [cols*rows]  ch==0   = empty */
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

/* Set up 7 colour slots (red→magenta). Falls back to the basic 8 colours when
 * the terminal can't do 256. */
static void color_init(void) {
  start_color();
  if (COLORS >= 256) {
    init_pair(1, 196, COLOR_BLACK);
    init_pair(2, 208, COLOR_BLACK);
    init_pair(3, 226, COLOR_BLACK);
    init_pair(4, 46, COLOR_BLACK);
    init_pair(5, 51, COLOR_BLACK);
    init_pair(6, 33, COLOR_BLACK);
    init_pair(7, 201, COLOR_BLACK);
  } else {
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_GREEN, COLOR_BLACK);
    init_pair(5, COLOR_CYAN, COLOR_BLACK);
    init_pair(6, COLOR_BLUE, COLOR_BLACK);
    init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
  }
}

/* Pick the closest of the 7 colour slots (red, orange, yellow, green, cyan,
 * blue, magenta) to a colour's hue. Returns -1 for near-greys that have no real
 * hue (the caller then falls back to a plain brightness ramp). */
static int hue_to_pair(Vec3 c) {
  float mx = fmaxf(c.x, fmaxf(c.y, c.z));
  float mn = fminf(c.x, fminf(c.y, c.z));
  float chroma = mx - mn;
  if (chroma < HUE_MIN_CHROMA)
    return -1;
  float h;
  if (mx == c.x)
    h = 60.f * fmodf((c.y - c.z) / chroma, 6.f);
  else if (mx == c.y)
    h = 60.f * ((c.z - c.x) / chroma + 2.f);
  else
    h = 60.f * ((c.x - c.y) / chroma + 4.f);
  if (h < 0.f)
    h += 360.f;
  static const float pal[7] = {0.f, 30.f, 60.f, 120.f, 180.f, 240.f, 300.f};
  int best = 0;
  float bd = 1e9f;
  for (int i = 0; i < 7; i++) {
    float d = fabsf(h - pal[i]);
    if (d > 180.f)
      d = 360.f - d;
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best + 1;
}

/* Perceived brightness of a colour (green counts most, blue least). */
static inline float luma_rec709(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* Turn a fragment's colour into a drawn cell: the character comes from how bright
 * it is, the colour from its hue. Splitting it this way is what makes the shaders
 * look different — the normals view is a rainbow while phong is one steady colour;
 * picking the character by brightness alone would make them all look the same.
 * Near-greys with no real hue fall back to a brightness ramp. */
static Cell rgb_to_cell(Vec3 col, int px, int py) {
  /* brightness, nudged by the dither pattern so flat areas don't band */
  float thr = k_bayer[py & 3][px & 3];
  float d = luma_rec709(col) + (thr - 0.5f) * DITHER_AMP;
  d = d < 0.f ? 0.f : d > 1.f ? 1.f : d;

  /* glyph from the density ramp; colour from the fragment's hue */
  char ch = k_bourke[(int)(d * (BOURKE_LEN - 1))];
  int cp = hue_to_pair(col);
  if (cp < 0) {
    cp = 1 + (int)(d * 6.f); /* desaturated → step through the luma ramp */
    if (cp > 7)
      cp = 7;
  }
  bool bold = d > BOLD_LUMA;
  return (Cell){ch, cp, bold};
}

/* Copy the finished colour buffer onto the screen. Empty cells are skipped, so
 * the background stays black. */
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

/* ── §6 pipeline — turn the triangles into shaded cells ── */

/* For a pixel and a triangle, work out the three weights (one per corner) that
 * say how much each corner influences this pixel. They add up to 1 inside the
 * triangle; if any comes out negative, the pixel is outside it. */
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

/* Run the vertex shader on the triangle's 3 corners. For wireframe we also tag
 * each corner with a barycentric unit vector ((1,0,0), (0,1,0), (0,0,1)) so the
 * fragment shader can tell how close each pixel is to an edge afterwards. */
static void shade_triangle_vertices(const Mesh *mesh, const Triangle *tri,
                                    ShaderProgram *sh, bool is_wire,
                                    VSOut vo[3]) {
  static const float wire_u[3] = {1.f, 0.f, 0.f};
  static const float wire_v[3] = {0.f, 1.f, 0.f};
  for (int vi = 0; vi < 3; vi++) {
    const Vertex *vtx = &mesh->verts[tri->v[vi]];
    VSIn in;
    in.pos = vtx->pos;
    in.normal = vtx->normal;
    in.u = is_wire ? wire_u[vi] : vtx->u;
    in.v = is_wire ? wire_v[vi] : vtx->v;

    memset(&vo[vi], 0, sizeof vo[vi]);
    sh->vert(&in, &vo[vi], sh->vert_uni);

    if (is_wire) {
      vo[vi].custom[0] = wire_u[vi];
      vo[vi].custom[1] = wire_v[vi];
      vo[vi].custom[2] = 1.f - wire_u[vi] - wire_v[vi];
    }
  }
}

/* Perspective divide + screen mapping for the 3 corners: divide by w (which
 * makes far corners smaller), flip Y (clip-space up → screen-space down), and
 * scale into cell coordinates. sz carries each corner's depth for the z-test. */
static void project_triangle(const VSOut vo[3], int cols, int rows, float sx[3],
                             float sy[3], float sz[3]) {
  for (int vi = 0; vi < 3; vi++) {
    float w = vo[vi].clip_pos.w;
    if (fabsf(w) < W_DIVIDE_EPS)
      w = W_DIVIDE_EPS;
    sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
    sz[vi] = vo[vi].clip_pos.z / w;
  }
}

/* Twice the signed area of the screen triangle. With our winding, > 0 means it
 * faces the camera; ≤ 0 means it faces away and gets culled. */
static inline float signed_area(const float sx[3], const float sy[3]) {
  return (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
}

/* The screen rectangle the triangle covers, clamped to the framebuffer, so the
 * fill loop only visits cells that could be inside it. */
static void triangle_bbox(const float sx[3], const float sy[3], int cols,
                          int rows, int *x0, int *x1, int *y0, int *y1) {
  *x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  *x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  *y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  *y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
}

/* Blend the 3 corners' shaded values to one pixel using its barycentric weights
 * — this is what makes the surface vary smoothly across each triangle. */
static FSIn interpolate_fragment(const VSOut vo[3], const float b[3], int px,
                                 int py) {
  FSIn f;
  f.world_pos =
      v3_bary(vo[0].world_pos, vo[1].world_pos, vo[2].world_pos, b[0], b[1], b[2]);
  f.world_nrm = v3_norm(
      v3_bary(vo[0].world_nrm, vo[1].world_nrm, vo[2].world_nrm, b[0], b[1], b[2]));
  f.u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
  f.v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
  f.px = px;
  f.py = py;
  for (int c = 0; c < 4; c++)
    f.custom[c] =
        b[0] * vo[0].custom[c] + b[1] * vo[1].custom[c] + b[2] * vo[2].custom[c];
  return f;
}

/*
 * pipeline_draw_mesh — the forward rasteriser: turn the triangle mesh into
 * shaded terminal cells. Each triangle goes through the classic stages — shade
 * its corners, project to the screen, cull if back-facing, then fill the pixels
 * it covers (coverage test, z-test, interpolate, fragment shader, paint).
 */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               ShaderProgram *sh, bool is_wire) {
  int cols = fb->cols, rows = fb->rows;

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    /* vertex stage: shade the 3 corners (model → clip space) */
    VSOut vo[3];
    shade_triangle_vertices(mesh, tri, sh, is_wire, vo);

    /* drop the whole triangle if all 3 corners are behind the near plane */
    if (vo[0].clip_pos.w < NEAR_CLIP_W && vo[1].clip_pos.w < NEAR_CLIP_W &&
        vo[2].clip_pos.w < NEAR_CLIP_W)
      continue;

    /* project to the screen, then drop it if it faces away from the camera */
    float sx[3], sy[3], sz[3];
    project_triangle(vo, cols, rows, sx, sy, sz);
    if (signed_area(sx, sy) <= 0.f)
      continue;

    /* fill the cells inside the triangle's bounding box */
    int x0, x1, y0, y1;
    triangle_bbox(sx, sy, cols, rows, &x0, &x1, &y0, &y1);
    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        /* inside the triangle? the barycentric weights say so (all ≥ 0) */
        float b[3];
        barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        /* z-test: keep this pixel only if it's nearer than what's there */
        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        int idx = py * cols + px;
        if (z >= fb->zbuf[idx])
          continue;
        fb->zbuf[idx] = z;

        /* shade the pixel and paint it */
        FSIn fsin = interpolate_fragment(vo, b, px, py);
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

/* ── §7 scene — the whole world, and how it changes each frame ── */

/* Everything the program tracks, in one place: the donut and how far it has spun,
 * the two things the user toggles (pause, which shader), and the render wiring —
 * the active shader and the values it reads, which are rebuilt every frame from
 * the spin angles and the chosen shader (so they're derived, not user state).
 * Only the scene_* functions take a whole Scene*; the shaders and renderer take
 * just the piece they need (Uniforms, Mesh, ShaderProgram). */
typedef struct {
  /* the spinning donut */
  Mesh mesh;
  float angle_x, angle_y; /* how far it has spun, radians */

  /* what the user toggles */
  bool paused;         /* space — freeze the spin        */
  ShaderIdx shade_idx; /* s — which of the 4 looks       */

  /* render wiring, rebuilt each frame */
  ShaderProgram shader;  /* the active shader's functions + value pointers */
  Uniforms uni;          /* this frame's transforms, light, material       */
  ToonUniforms toon_uni; /* uni plus the toon band count                   */
} Scene;

/* Wire up the functions + values for the current look. Called whenever the
 * shader changes, so the rest of the code never branches on which one is active. */
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
  case SH_WIRE:
    s->shader = (ShaderProgram){vert_wire, frag_wire, &s->uni, &s->uni};
    break;
  default:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_torus();
  s->shade_idx = SH_PHONG;
  s->paused = false;

  s->uni.light_pos = v3(4.f, 3.f, 3.f);       /* sun — up, right, and in front */
  s->uni.light_col = v3(1.0f, 0.96f, 0.86f);  /* warm sunlight */
  s->uni.ambient = v3(0.05f, 0.06f, 0.10f);   /* faint cool fill on the shadow side */
  s->uni.shininess = 30.f;                    /* broad, soft glint */
  s->uni.cam_pos = v3(0.f, 0.f, CAM_DIST);
  s->uni.obj_color = v3(0.3f, 0.7f, 0.9f); /* torus — cyan-blue */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));

  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

  scene_build_shader(s);
}

/* Rebuild the camera lens after a resize (the aspect ratio changed). */
static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/* Advance one frame: nudge the spin, then rebuild the transforms that place and
 * orient the donut for the renderer. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->angle_y += ROT_Y * dt;
  s->angle_x += ROT_X * dt;

  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  s->uni.mvp = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);

  s->toon_uni.base = s->uni; /* the toon shader reads its own copy */
}

static void scene_draw(Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  bool is_wire = (s->shade_idx == SH_WIRE);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, is_wire);
  fb_blit(fb);
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

/* ── §8 screen — set up the terminal, draw the HUD, show the frame ── */

/* The terminal we draw on — just its size in cells, re-checked on resize. Kept
 * apart from Scene so the framebuffer can be sized from it without the render
 * code reaching into scene state. */
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
  typeahead(-1); /* don't let waiting keypresses interrupt drawing (avoids tearing) */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) {
  (void)s;
  endwin();
}
/* The endwin()+refresh() pair is what makes ncurses notice the new window size. */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* The HUD reuses two of the colour slots — yellow for the status line, cyan for
 * the key hint — named here so the HUD code doesn't say "slot 3 / slot 5". */
#define PAIR_HUD 3  /* yellow */
#define PAIR_HINT 5 /* cyan   */

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps) {
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status, " %5.1f fps  shader:%s%s ", fps,
           k_shader_names[sc->shade_idx], sc->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " TORUS · RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0, " q:quit  spc:pause  s:shader ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app — wire it together: set up, run the loop, handle keys ── */

/* The top-level program: the scene, the terminal, the framebuffer they share,
 * and two flags the signal handlers flip (time to quit, window was resized).
 * Harness glue that the main loop drives — not part of the torus itself. */
typedef struct {
  Scene scene;
  Screen screen;
  Framebuffer fb;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

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

/* On a resize: re-make the buffers at the new size and rebuild the camera shape. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_free(&app->fb);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    app->scene.paused = !app->scene.paused;
    break;
  case 's':
  case 'S':
    scene_next_shader(&app->scene);
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

    /* deal with a pending resize before anything else */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* time since last frame, capped so a stall can't make the spin jump */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt_sec);

    /* refresh the fps reading a couple of times a second */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* draw the frame: the donut, then the HUD on top */
    erase();
    scene_draw(&app->scene, &app->fb);
    screen_draw_hud(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* handle one keypress, if any */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* sleep off the rest of the frame to hold the target rate */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  screen_free(&app->screen);
  return 0;
}
