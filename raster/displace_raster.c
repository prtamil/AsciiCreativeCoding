/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * displace_raster.c — a sphere whose surface really moves: every frame each
 * point is pushed in or out by a rippling/waving/pulsing/spiky pattern, and the
 * surface's facing is recomputed so the lighting follows the new shape (not a
 * fake bump-map — the silhouette genuinely deforms). Press 'd' for the four
 * patterns, 's' for the four looks (phong, toon, normals, glass), c/+/-/space/q.
 *
 * Read sphere_raster.c first — this is that same renderer with one extra step in
 * the per-corner stage. The only new idea here is "move the point, then re-figure
 * which way it faces."
 * Ideas from: displacement shaders (Cook, SIGGRAPH '84); the central-difference
 *   trick for recomputing a surface's facing; z-buffer (Catmull 1974).
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/displace_raster.c -o displace -lncurses -lm
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
  HUD_COLS = 80,

  /* How finely the sphere is chopped into triangles: more = smoother and shows
   * finer ripples, but slower. 48×32 ≈ 3000 triangles, smooth at 60fps in a
   * terminal. Drop to 36×24 if it's sluggish on yours. */
  TESS_U = 48,
  TESS_V = 32,
};

#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 3.2f
/* Closest the camera may get. Kept far enough back that even a fully-grown spike
 * never reaches the camera — a point that crosses behind the lens can't be drawn
 * and would smear across the screen. */
#define CAM_DIST_MIN 1.8f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f
#define NEAR_W_EPS 0.001f /* a corner closer to the eye than this is behind it */

#define SPHERE_R 1.0f

/*
 * Lighting, tuned for the chunky terminal grid where one cell is one brightness.
 * Three tweaks from textbook lighting, all so the shape (and its rippling) stays
 * readable when everything is so coarse:
 *   • a wide highlight, spread over several cells instead of one tiny dot;
 *   • light that wraps a bit past the edge of shadow, so the dark side keeps
 *     some shading instead of going flat black;
 *   • an edge glow that brightens the rim, popping the outline and the bumps
 *     you see along it.
 */
#define LIGHT_SHININESS 24.0f /* lower = a wider highlight                   */
#define SPEC_STRENGTH 0.6f    /* how strong the highlight is                */
#define RIM_STRENGTH 0.5f     /* how bright the edge glow is                */
#define RIM_POWER 2.5f        /* lower = a wider edge glow                  */

/* The toon (flat cartoon) look, also tuned for the terminal. */
#define TOON_SPEC_THRESH 0.94f  /* angle this sharp gets a hard bright dot */
#define TOON_SPEC 0.7f          /* how bright that dot is                  */
#define TOON_AMBIENT_LIFT 0.12f /* a floor on every band, so none is black */

/* How fast it tumbles — slow, so you can watch the surface ripple. */
#define ROT_Y 0.30f
#define ROT_X 0.12f

/*
 * The glass look — fake "see-through" glass. There's nothing actually behind the
 * sphere, so it bends your line of sight through the surface and reads a made-up
 * stripe pattern with it; because the bent surface steers that, the stripes
 * swirl over every ripple and spike. The edges turn mirror-like and the middle
 * stays clearer, the way real glass looks.
 */
#define GLASS_ETA 0.6667f     /* how much glass bends light (air→glass)   */
#define GLASS_F0 0.04f        /* how reflective it is head-on             */
#define GLASS_ENV_FREQ 6.0f   /* how dense the made-up stripes are        */
#define GLASS_GLINT_MULT 2.0f /* makes the glossy glint tighter           */

/*
 * The little step used to re-figure which way the bent surface faces (we sample
 * the pattern a hair to each side and see how it changed). Too small and float
 * rounding swamps it (the lighting flickers); too big and it lags the real
 * curvature (sharp spikes get rounded off). 3% of the radius works for all four
 * patterns.
 */
#define CD_EPS (0.03f * SPHERE_R)

/* The brightness-to-character ladder, sparse to dense (Paul Bourke's). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* A fixed 4×4 nudge pattern so neighbours of similar brightness pick different
 * characters, hiding the steps in a smooth gradient. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.15f      /* how strong that nudge is                  */
#define LUMA_BOLD_ABOVE 0.6f  /* brighter than this → draw the cell bold    */
#define CHROMA_MIN 0.08f      /* greyer than this → treat it as plain grey  */

#define CELL_W 8
#define CELL_H 16

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define PI 3.14159265f

/* ── §2 math — vectors and 4×4 matrices ────────────────────────────────── */

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
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
}

/* Bends a ray as it passes through a surface, the way light bends entering
 * water or glass (how much is set by eta). Used by the glass look to bend your
 * line of sight through the bumpy surface. Returns zero in the rare case the ray
 * can't get through (it would bounce back instead). */
static inline Vec3 v3_refract(Vec3 I, Vec3 N, float eta) {
  float ndi = v3_dot(N, I);
  float k = 1.f - eta * eta * (1.f - ndi * ndi);
  if (k < 0.f)
    return v3(0, 0, 0);
  return v3_sub(v3_scale(I, eta), v3_scale(N, eta * ndi + sqrtf(k)));
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

/* Clamps a number to 0..1. */
static inline float clamp01(float x) { return x < 0.f ? 0.f : x > 1.f ? 1.f : x; }

/* Brightness correction for the screen, with an upper clamp so a too-bright sum
 * doesn't wrap past white. */
static inline float gamma_encode(float x) {
  return powf(fminf(x, 1.f), 1.f / 2.2f);
}

static inline Vec3 v3_gamma(Vec3 c) {
  return v3(gamma_encode(c.x), gamma_encode(c.y), gamma_encode(c.z));
}

/* ── §3 displacement — how the surface moves, and which way it then faces ── */

/*
 * The four patterns. Each one takes a point on the sphere and the time, and
 * returns how far to push that point — positive pushes it out, negative pushes
 * it in. (amp and freq scale how big and how dense the pattern is.)
 *
 * They must be pure — the same inputs always give the same answer — because to
 * find which way the bent surface faces, we sample each one a hair to either
 * side of a point and compare. That only works if the answer depends on the
 * point alone, with no hidden state.
 */

/*
 * DispMode — which pattern is active; the 'd' key cycles through them. The idea
 * of "swap the push pattern, keep the renderer" is Cook's displacement shader
 * (SIGGRAPH '84). They're numbered from 0 in a row so 'd' can step through them
 * and so they double as indices into the tables below.
 */
typedef enum {
  DM_RIPPLE = 0, /* rings sweeping out from the equator           */
  DM_WAVE,       /* one diagonal wave rolling across              */
  DM_PULSE,      /* the whole ball breathes, biggest at the middle */
  DM_SPIKY,      /* a porcupine of moving spikes                  */
  DM_COUNT,      /* how many patterns there are (for cycling)     */
} DispMode;

/* Display label per mode, indexed by DispMode; shown in the HUD. */
static const char *k_disp_names[] = {"ripple", "wave", "pulse", "spiky"};

/* Rings that ride outward from the equator. The push rises and falls with
 * distance from the axis (that's the rings) and creeps over time (they travel);
 * it's eased off near the poles so they don't look broken. */
static float displace_ripple(Vec3 pos, float time, float amp, float freq) {
  float r = sqrtf(pos.x * pos.x + pos.z * pos.z);
  float taper = 1.f - fabsf(pos.y) * 0.6f;
  return sinf(time * 2.5f + r * freq) * amp * taper;
}

/* One wave rolling across the whole ball on a slant — the push depends on a
 * slanted mix of the point's x/y/z, so the whole surface undulates as one. */
static float displace_wave(Vec3 pos, float time, float amp, float freq) {
  float phase = pos.x * freq + pos.y * freq * 0.8f + pos.z * freq * 0.5f;
  return sinf(time * 2.0f + phase) * amp;
}

/* The whole ball breathing in and out. Two sine waves of time (a slow one plus a
 * faster wobble so it isn't too mechanical), eased down toward the poles so the
 * middle heaves the most and the poles stay put. */
static float displace_pulse(Vec3 pos, float time, float amp, float freq) {
  float r = sqrtf(pos.x * pos.x + pos.z * pos.z);
  float breathe = sinf(time * 1.5f) * 0.85f + sinf(time * 4.5f) * 0.15f;
  float falloff = expf(-r * freq * 0.4f);
  return breathe * amp * falloff;
}

/* A spiky ball. Three sine waves (one per axis) multiplied together spike up
 * only where all three happen to peak at once; the time terms drift the spikes
 * around, and the 0.6 power gives each spike a rounded base instead of a needle. */
static float displace_spiky(Vec3 pos, float time, float amp, float freq) {
  float f = freq * 1.4f;
  float t = time * 0.8f;
  float val = fabsf(sinf(pos.x * f + t) * sinf(pos.y * f + t * 0.7f) *
                    sinf(pos.z * f + t * 1.3f));
  return powf(val, 0.6f) * amp;
}

/* A pointer to whichever pattern is active. Swapping it is the whole trick — the
 * renderer never changes, you just point at a different push function. Must be
 * pure (see above), since the surface-facing recompute samples it either side of
 * a point. */
typedef float (*DispFn)(Vec3, float, float, float);

/* Dispatch table — k_disp_fn[mode] is the active field. Indexed by DispMode. */
static const DispFn k_disp_fn[DM_COUNT] = {
    displace_ripple,
    displace_wave,
    displace_pulse,
    displace_spiky,
};

/* Finds two directions that lie flat along the surface at a point (both at right
 * angles to the way it faces, and to each other) — the two directions we'll step
 * in to feel out how the surface tilts. Built from the facing with cross
 * products; near the poles it picks a different reference so the math doesn't
 * collapse. */
static void make_tangent_basis(Vec3 N, Vec3 *T, Vec3 *B) {
  Vec3 up = (fabsf(N.y) < 0.9f) ? v3(0, 1, 0) : v3(1, 0, 0);
  *T = v3_norm(v3_cross(up, N));
  *B = v3_cross(N, *T); /* already unit length since N and T are */
}

/* How much the push changes as you step a little to each side of a point: sample
 * the pattern just ahead and just behind and subtract. (Sampling both sides,
 * rather than just one, cancels most of the error.) */
static float central_diff(DispFn fn, Vec3 pos, Vec3 dir, float eps, float time,
                          float amp, float freq) {
  float fp = fn(v3_add(pos, v3_scale(dir, eps)), time, amp, freq);
  float fm = fn(v3_add(pos, v3_scale(dir, -eps)), time, amp, freq);
  return fp - fm;
}

/*
 * Re-figures which way the surface faces after the push has bent it — the trick
 * that makes the lighting follow the new shape. Moving a point changes how the
 * surface tilts around it, so the old "straight out from the centre" facing is
 * wrong. We feel out the new tilt: step a little along the two flat directions,
 * see how much the push rose or fell each way, rebuild the two now-tilted
 * directions, and the way that faces is the new facing. Works for any pattern,
 * no calculus needed.
 */
static Vec3 displaced_normal(Vec3 pos, Vec3 N, DispFn fn, float time, float amp,
                             float freq) {
  Vec3 T, B;
  make_tangent_basis(N, &T, &B);

  float eps = CD_EPS;

  /* how much the surface rises/falls as we step each way */
  float df_T = central_diff(fn, pos, T, eps, time, amp, freq);
  float df_B = central_diff(fn, pos, B, eps, time, amp, freq);

  /* tilt each flat direction by that rise/fall, then the direction at right
   * angles to both is the new facing */
  Vec3 T_disp = v3_add(v3_scale(T, 2.f * eps), v3_scale(N, df_T));
  Vec3 B_disp = v3_add(v3_scale(B, 2.f * eps), v3_scale(N, df_B));

  return v3_norm(v3_cross(T_disp, B_disp));
}

/* ── §4 shaders — the per-corner and per-pixel steps ───────────────────── */

/*
 * The four little records that flow between the drawing steps, copying how a GPU
 * works: the per-corner step turns a VSIn into a VSOut; the rasteriser blends
 * the VSOuts across a triangle into one FSIn per pixel; the per-pixel step turns
 * that into an FSOut (a colour).
 */

/* VSIn — one corner going into the per-corner step, in the sphere's own space. */
typedef struct {
  Vec3 pos;    /* the corner's position                 */
  Vec3 normal; /* which way it faces                    */
  float u, v;  /* texture coords (0..1)                 */
} VSIn;

/* VSOut — what the per-corner step produces: where the corner lands on screen,
 * plus the info that gets blended across the triangle (world spot, facing, uv). */
typedef struct {
  Vec4 clip_pos;  /* where it lands, before the divide-by-distance */
  Vec3 world_pos; /* its spot in the world, for the lighting       */
  Vec3 world_nrm; /* which way it faces in the world               */
  float u, v;     /* texture coords                                */
} VSOut;

/* FSIn — one pixel going into the per-pixel step: the VSOut info blended to this
 * pixel, plus which cell it is (px,py), used by the dither. */
typedef struct {
  Vec3 world_pos; /* this pixel's spot in the world */
  Vec3 world_nrm; /* which way the surface faces here */
  float u, v;     /* blended texture coords */
  int px, py;     /* which screen cell this is */
} FSIn;

/* FSOut — what the per-pixel step produces: a colour, or "skip me". No current
 * look sets discard, but it stays as an escape hatch for cutout effects. */
typedef struct {
  Vec3 color;   /* the colour for this pixel               */
  bool discard; /* true = leave the cell as-is             */
} FSOut;

/* The two steps as function pointers; the trailing pointer is the step's shared
 * data (cast to the right type inside). */
typedef void (*VertShaderFn)(const VSIn *, VSOut *, const void *);
typedef void (*FragShaderFn)(const FSIn *, FSOut *, const void *);

/*
 * ShaderProgram — one look: its per-corner step, its per-pixel step, and a
 * separate data pointer for each. They're separate because the per-corner step
 * always needs the displacement settings while the per-pixel step (toon) may
 * need its own different settings — one shared pointer couldn't be both types.
 */
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni; /* handed to the per-corner step — always the disp data */
  const void *frag_uni; /* handed to the per-pixel step — disp or toon data     */
} ShaderProgram;

/* ── the shared data the steps read ───────────────────────────────────── */

/*
 * Uniforms — the values that stay the same for a whole frame and that every step
 * reads. Two groups: the transforms (rebuilt each frame by scene_tick) and the
 * lighting (set once at startup).
 *   model/view/proj/mvp — the transforms: place the sphere, look from the camera,
 *                         apply perspective, all chained into mvp
 *   norm_mat            — the matrix for rotating surface-facing directions
 *   light_pos/_col      — where the light is and its colour
 *   ambient             — a little light everywhere
 *   cam_pos             — where the eye is (for highlights)
 *   obj_color/shininess — the surface's colour and how glossy it is
 */
typedef struct {
  /* the transforms (rebuilt each frame) */
  Mat4 model;
  Mat4 view;
  Mat4 proj;
  Mat4 mvp;
  Mat4 norm_mat;
  /* the lighting (set once at startup) */
  Vec3 light_pos;
  Vec3 light_col;
  Vec3 ambient;
  Vec3 cam_pos;
  Vec3 obj_color;
  float shininess;
} Uniforms;

/* ToonUniforms — the usual Uniforms plus how many flat steps the cartoon look
 * snaps to. Starts with `base` so a pointer to it can also be read as a plain
 * Uniforms. */
typedef struct {
  Uniforms base; /* the usual shared data            */
  int bands;     /* how many flat shading steps      */
} ToonUniforms;

/* DisplaceUniforms — the usual Uniforms plus the push settings the per-corner
 * step needs. Starts with `base` so a pointer to it can also be read as a plain
 * Uniforms (which the lighting steps want).
 *   disp_fn   — the active push pattern
 *   time      — seconds since start (the animation clock)
 *   amplitude — how big the push is
 *   frequency — how dense the pattern is
 *   mode      — the active pattern, for the HUD label only
 */
typedef struct {
  Uniforms base;
  DispFn disp_fn;
  float time;
  float amplitude;
  float frequency;
  DispMode mode;
} DisplaceUniforms;

/* ── the per-corner step ───────────────────────────────────────────── */

/*
 * The heart of the demo. For each corner: figure how far the pattern pushes it,
 * move it that far along its facing, re-figure which way the moved surface faces,
 * then hand the moved spot and new facing down to the per-pixel step. The
 * per-pixel looks never learn the surface moved — they just shade whatever spot
 * and facing they're handed, which is why all four work on it unchanged.
 */
static void vert_displace(const VSIn *in, VSOut *out, const void *u_) {
  const DisplaceUniforms *du = (const DisplaceUniforms *)u_;
  const Uniforms *u = &du->base;

  Vec3 N = v3_norm(in->pos); /* on a sphere, the facing is just the direction out */
  float d = du->disp_fn(in->pos, du->time, du->amplitude, du->frequency);
  Vec3 dpos = v3_add(in->pos, v3_scale(N, d)); /* moved spot */
  Vec3 dnrm = displaced_normal(in->pos, N, du->disp_fn, du->time, du->amplitude,
                               du->frequency); /* new facing */

  out->clip_pos = m4_mul_v4(u->mvp, v4(dpos.x, dpos.y, dpos.z, 1.f));
  out->world_pos = m4_pt(u->model, dpos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, dnrm));

  out->u = in->u;
  out->v = in->v;
}

/* ── the per-pixel steps (the four looks) ──────────────────────────── */

/* The normal lit look, tuned for the coarse grid (see §1 lighting): light that
 * wraps past the shadow edge, a wide highlight, and an edge glow. Together they
 * let the highlight glide visibly across the ripples and spikes instead of
 * shrinking to one invisible dot. */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  /* wrap the light past the shadow edge so the dark side keeps some shading */
  float wrap = 0.5f * v3_dot(N, L) + 0.5f;
  float diff = wrap * wrap;

  /* the highlight and the edge glow, both kept wide so they show on the grid */
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess) * SPEC_STRENGTH;
  float rim = powf(1.f - fmaxf(0.f, v3_dot(N, V)), RIM_POWER) * RIM_STRENGTH;

  Vec3 c = u->obj_color;
  float r = u->ambient.x + c.x * u->light_col.x * diff + spec + rim;
  float g = u->ambient.y + c.y * u->light_col.y * diff + spec + rim;
  float b = u->ambient.z + c.z * u->light_col.z * diff + spec + rim;
  out->color = v3_gamma(v3(r, g, b));
  out->discard = false;
}

/* The flat cartoon look: snap the lighting to a few flat steps, so you see hard
 * bands. On the bumpy sphere the band edges hug the wave crests, so the cartoon
 * shading reacts to the moving surface, not just the overall ball. */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));
  /* wrap the light around so the bands span the whole ball, not just the lit half */
  float diff = 0.5f * v3_dot(N, L) + 0.5f;
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > TOON_SPEC_THRESH) ? TOON_SPEC : 0.f;
  Vec3 c = u->obj_color;
  out->color.x = fminf(c.x * (banded + TOON_AMBIENT_LIFT) + spec, 1.f);
  out->color.y = fminf(c.y * (banded + TOON_AMBIENT_LIFT) + spec, 1.f);
  out->color.z = fminf(c.z * (banded + TOON_AMBIENT_LIFT) + spec, 1.f);
  out->discard = false;
}

/* A debugging view: paint each pixel by which way the surface faces, turned into
 * a colour. On the bumpy sphere this shows the recomputed facings directly —
 * the wave crests show up as swirling colour bands, so you can see the
 * facing-recompute is working. */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/* The fake-glass look. With nothing actually behind the sphere, we bend the line
 * of sight through the surface and read a made-up stripe pattern with it; since
 * the bumpy surface steers that, the stripes swirl over every wave and spike —
 * that swirling IS the "see-through" effect. The edges turn mirror-bright and
 * the centre stays clearer (like real glass), with a tight glint on top. */
static void frag_glass(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos)); /* toward the eye */
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  /* how mirror-like this spot is: nearly clear head-on, full mirror at the edge */
  float ndv = fmaxf(0.f, v3_dot(N, V));
  float fres = GLASS_F0 + (1.f - GLASS_F0) * powf(1.f - ndv, 5.f);

  /* bend the line of sight through the surface and read the stripes with it */
  Vec3 R = v3_refract(v3_scale(V, -1.f), N, GLASS_ETA);
  float stripes = 0.5f + 0.5f * sinf((R.x + R.y) * GLASS_ENV_FREQ);
  Vec3 tint = v3(0.45f, 0.80f, 0.95f); /* cool cyan glass */
  Vec3 refr = v3_scale(tint, 0.25f + 0.75f * stripes);

  /* the mirror part: a bright near-white edge plus a tight glint */
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess * GLASS_GLINT_MULT);
  Vec3 refl = v3(0.90f, 0.95f, 1.0f);

  /* mix: see-through in the centre, mirror at the rim, glint on top */
  Vec3 c = v3_add(v3_scale(refr, 1.f - fres), v3_scale(refl, fres));
  c = v3_add(c, v3(spec, spec, spec));

  out->color = v3_gamma(c);
  out->discard = false;
}

/*
 * ShaderIdx — which look is showing; the 's' key cycles through them. Numbered
 * from 0 in a row so 's' can step through and so they double as labels.
 */
typedef enum {
  SH_PHONG = 0, /* normal realistic lighting          */
  SH_TOON,      /* flat cartoon shading               */
  SH_NORMALS,   /* facing-as-colour debug view         */
  SH_GLASS,     /* fake see-through glass              */
  SH_COUNT,     /* how many looks there are (for cycling) */
} ShaderIdx;

/* HUD label per shader, indexed by ShaderIdx. */
static const char *k_shader_names[] = {"phong", "toon", "normals", "glass"};

/* ── §5 mesh — building the sphere once at startup ─────────────────────── */

/*
 * Vertex / Triangle / Mesh — a shape made of triangles. To avoid storing the
 * same corner over and over, the corners live once in one list and each triangle
 * just points at three of them by their slot number.
 *
 * Each corner carries which way it faces so the surface shades smoothly; on the
 * plain sphere that's the direction straight out, but the per-corner step
 * overwrites it each frame with the bent-surface facing. Mesh owns its two lists
 * on the heap — the only memory the program allocates, built once.
 */
typedef struct {
  Vec3 pos;    /* the corner's position (on the plain, un-pushed sphere) */
  Vec3 normal; /* which way it faces                                     */
  float u, v;  /* its place around/along the sphere (0..1)               */
} Vertex;
typedef struct {
  int v[3]; /* three corner-slots, counter-clockwise from outside */
} Triangle;
typedef struct {
  Vertex *verts;  /* the corner list (heap, built once)   */
  int nvert;      /* how many corners are filled in        */
  Triangle *tris; /* the triangle list (heap, built once) */
  int ntri;       /* how many triangles are filled in      */
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/* The corners sit on a grid of rows (top to bottom) and columns (around). This
 * finds the slot for one (row, col). Each row keeps one extra column so the seam
 * where it wraps lines up cleanly. */
static inline int grid_index(int row, int col, int nu) {
  return row * (nu + 1) + col;
}

/* Places one corner on the sphere, like a point at a given latitude (row) and
 * longitude (column) on a globe. It faces straight out — except right at the two
 * poles, where "straight out" is undefined, so we pin it to up / down. */
static Vertex sphere_vertex(int j, int i, int nu, int nv) {
  float v = (float)j / nv;
  float u = (float)i / nu;
  float phi = v * PI;
  float theta = u * 2.f * PI;
  float sp = sinf(phi), cp = cosf(phi);
  Vec3 pos = v3(SPHERE_R * sp * cosf(theta), SPHERE_R * cp,
                SPHERE_R * sp * sinf(theta));
  Vec3 nrm =
      (sp < 1e-6f) ? ((j == 0) ? v3(0, 1, 0) : v3(0, -1, 0)) : v3_norm(pos);
  return (Vertex){pos, nrm, u, v};
}

/* Builds the ball: place every corner on the grid, then stitch each little grid
 * square into two triangles (wound counter-clockwise so the renderer can tell
 * front from back). Built once at startup. */
static Mesh tessellate_sphere(void) {
  int nu = TESS_U, nv = TESS_V;
  Mesh m;
  m.verts = malloc((size_t)(nu + 1) * (nv + 1) * sizeof(Vertex));
  m.tris = malloc((size_t)nu * nv * 2 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* place every corner on the grid */
  for (int j = 0; j <= nv; j++)
    for (int i = 0; i <= nu; i++)
      m.verts[m.nvert++] = sphere_vertex(j, i, nu, nv);

  /* stitch each grid square into two triangles */
  for (int j = 0; j < nv; j++) {
    for (int i = 0; i < nu; i++) {
      int r0 = grid_index(j, i, nu);
      int r1 = grid_index(j, i + 1, nu);
      int r2 = grid_index(j + 1, i, nu);
      int r3 = grid_index(j + 1, i + 1, nu);
      m.tris[m.ntri++] = (Triangle){{r0, r2, r1}};
      m.tris[m.ntri++] = (Triangle){{r1, r2, r3}};
    }
  }
  return m;
}

/* ── §6 framebuffer — where we draw before showing it ──────────────────── */

/*
 * Cell — one finished character on screen: which character, what colour, and
 * whether it's bold. The colour has already been boiled down to these, so the
 * copy-to-screen step just emits them. ch=0 means the cell was never touched.
 */
typedef struct {
  char ch;        /* the character (0 = empty, skipped when shown) */
  int color_pair; /* which colour                                  */
  bool bold;      /* bold for bright cells                         */
} Cell;

/*
 * Framebuffer — our own off-screen page, the size of the terminal, that we draw
 * into and then copy out all at once. Two grids of cols×rows:
 *   cbuf — the finished character + colour for each cell
 *   zbuf — how near the closest thing drawn at each cell is, so a farther
 *          surface can't paint over a nearer one (the classic z-buffer, Catmull
 *          1974; empty cells start "infinitely far")
 * Keeping cbuf as plain data means the drawing math never touches ncurses —
 * fb_blit is the one place that does.
 */
typedef struct {
  float *zbuf;    /* nearness per cell (FLT_MAX = empty); nearest wins */
  Cell *cbuf;     /* finished character/colour per cell                */
  int cols, rows; /* size, = the terminal size                        */
} Framebuffer;

static void fb_alloc(Framebuffer *fb, int c, int r) {
  fb->cols = c;
  fb->rows = r;
  fb->zbuf = malloc((size_t)(c * r) * sizeof(float));
  fb->cbuf = malloc((size_t)(c * r) * sizeof(Cell));
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

/* How bright a colour looks to the eye (green counts most, blue least). */
static inline float rec709_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* Picks the character for a given brightness (0..1): faint for dim, dense for bright. */
static inline int ramp_index(float v) { return (int)(v * (BOURKE_LEN - 1)); }

/* Which colour a colour is — its spot on the colour wheel, in degrees. Returns
 * -1 if it's too greyish to have a real colour. */
static float rgb_hue(Vec3 c) {
  float mx = fmaxf(c.x, fmaxf(c.y, c.z));
  float mn = fminf(c.x, fminf(c.y, c.z));
  float chroma = mx - mn;
  if (chroma < CHROMA_MIN)
    return -1.f;
  float h;
  if (mx == c.x)
    h = 60.f * fmodf((c.y - c.z) / chroma, 6.f);
  else if (mx == c.y)
    h = 60.f * ((c.z - c.x) / chroma + 2.f);
  else
    h = 60.f * ((c.x - c.y) / chroma + 4.f);
  if (h < 0.f)
    h += 360.f;
  return h;
}

/* Snaps a colour-wheel position to the nearest of our 7 colours (going the short
 * way around the wheel) and returns its pair number. */
static int nearest_palette_pair(float hue) {
  static const float pal[7] = {0.f, 30.f, 60.f, 120.f, 180.f, 240.f, 300.f};
  int best = 0;
  float bd = 1e9f;
  for (int i = 0; i < 7; i++) {
    float d = fabsf(hue - pal[i]);
    if (d > 180.f)
      d = 360.f - d;
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best + 1;
}

/* Picks a colour for a pixel by its colour-wheel position, or -1 if it's grey. */
static int hue_to_pair(Vec3 c) {
  float hue = rgb_hue(c);
  if (hue < 0.f)
    return -1;
  return nearest_palette_pair(hue);
}

/* Turns a pixel's colour into a finished cell: the character comes from its
 * brightness (with a dither nudge so gradients stay smooth), and the colour from
 * its hue — so the facing-view reads as a rainbow, not just shades of one colour.
 * Greyish pixels fall back to brightness-based shades. */
static Cell rgb_to_cell(Vec3 col, int px, int py) {
  float dithered =
      clamp01(rec709_luma(col) + (k_bayer[py & 3][px & 3] - 0.5f) * DITHER_AMP);
  int glyph = ramp_index(dithered);

  int pair = hue_to_pair(col);
  if (pair < 0) { /* greyish → shade it by brightness instead */
    pair = 1 + (int)(dithered * 6.f);
    if (pair > 7)
      pair = 7;
  }
  return (Cell){k_bourke[glyph], pair, dithered > LUMA_BOLD_ABOVE};
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

/* ── §7 pipeline — drawing the triangles ───────────────────────────────── */

/* For a pixel and a triangle, returns three weights (one per corner) saying how
 * much each corner pulls on that pixel. They tell us if the pixel is inside (all
 * three positive) and how to blend the corners' values there. A degenerate
 * (zero-area) triangle returns negatives so the caller skips it. */
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

/* Runs the per-corner step on a triangle's three corners. */
static void run_vertex_shader(const Mesh *mesh, const Triangle *tri,
                              ShaderProgram *sh, VSOut vo[3]) {
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

/* True when all three corners are behind the camera — the whole triangle is
 * off-screen, so skip it. */
static bool all_behind_near_plane(const VSOut vo[3]) {
  return vo[0].clip_pos.w < NEAR_W_EPS && vo[1].clip_pos.w < NEAR_W_EPS &&
         vo[2].clip_pos.w < NEAR_W_EPS;
}

/* Finishes the corners onto the screen: divide by distance (far = smaller),
 * scale into cells, and flip y because screen rows count downward but up should
 * be up. */
static void project_to_screen(const VSOut vo[3], int cols, int rows,
                              float sx[3], float sy[3], float sz[3]) {
  for (int vi = 0; vi < 3; vi++) {
    float w = vo[vi].clip_pos.w;
    if (fabsf(w) < 1e-6f)
      w = 1e-6f;
    sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
    sz[vi] = vo[vi].clip_pos.z / w;
  }
}

/* Measures the triangle's signed area on screen; its sign tells which way the
 * triangle faces. After the y-flip, front faces come out positive, so culling
 * keeps the positive ones. */
static float signed_area(const float sx[3], const float sy[3]) {
  return (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
}

/* Walks the cells the triangle covers; for each one inside it and nearer than
 * whatever's there, blends the corners to that pixel, runs the per-pixel step,
 * and stores the cell. */
static void rasterize_fragments(Framebuffer *fb, const VSOut vo[3],
                                const float sx[3], const float sy[3],
                                const float sz[3], ShaderProgram *sh) {
  int cols = fb->cols, rows = fb->rows;
  int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

  for (int py = y0; py <= y1; py++) {
    for (int px = x0; px <= x1; px++) {
      float b[3];
      barycentric(sx, sy, px + .5f, py + .5f, b);
      if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
        continue;

      float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
      int idx = py * cols + px;
      if (z >= fb->zbuf[idx])
        continue;
      fb->zbuf[idx] = z;

      FSIn fsin;
      fsin.world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos,
                               vo[2].world_pos, b[0], b[1], b[2]);
      fsin.world_nrm = v3_norm(v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                       vo[2].world_nrm, b[0], b[1], b[2]));
      fsin.u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
      fsin.v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
      fsin.px = px;
      fsin.py = py;

      FSOut fsout;
      fsout.discard = false;
      sh->frag(&fsin, &fsout, sh->frag_uni);
      if (fsout.discard)
        continue;

      fb->cbuf[idx] = rgb_to_cell(fsout.color, px, py);
    }
  }
}

/* Draws the whole sphere, one triangle at a time: run the per-corner step, drop
 * it if it's behind the camera or facing away, then fill the cells it covers. */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               ShaderProgram *sh, bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    VSOut vo[3];
    run_vertex_shader(mesh, tri, sh, vo);
    if (all_behind_near_plane(vo))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(vo, cols, rows, sx, sy, sz);
    if (cull_backface && signed_area(sx, sy) <= 0.f)
      continue;

    rasterize_fragments(fb, vo, sx, sy, sz, sh);
  }
}

/* ── §8 scene — the sphere, the clock, and the chosen settings ─────────── */

/* How big and how dense each pattern is — hand-tuned so each one looks its best.
 * amp = how far to push (as a fraction of the radius); freq = how dense. */
static const float k_amp[DM_COUNT] = {0.22f, 0.18f, 0.30f, 0.35f};
static const float k_freq[DM_COUNT] = {8.0f, 5.0f, 4.0f, 4.5f};

/*
 * Scene — everything about what's on screen and how the viewer is steering it.
 * Only three things actually change on their own each frame — the clock and the
 * two tumble angles, all advanced by scene_tick; from those it rebuilds the
 * transforms and the shared-data blocks. The rest is what the keys set (pause,
 * cull, which look, which pattern, zoom) or the drawing wiring.
 *
 * There are three shared-data blocks because different steps want different
 * types: `uni` is the master copy scene_tick fills, and disp_uni / toon_uni are
 * copied from it so the steps that need extra fields have them.
 */
typedef struct {
  /* the shape */
  Mesh mesh; /* the sphere, built once at startup */
  /* the only things that change on their own each frame */
  float angle_x, angle_y; /* tumble angles                    */
  float time;             /* the animation clock (seconds)    */
  /* what the keys set */
  float cam_dist;      /* zoom distance (+/-)                  */
  bool paused;         /* space freezes everything             */
  bool cull_backface;  /* 'c' shows/hides the inside faces     */
  ShaderIdx shade_idx; /* 's' which look                       */
  DispMode disp_idx;   /* 'd' which pattern                    */
  /* the drawing wiring (rebuilt from the above) */
  ShaderProgram shader;      /* the active look + its data pointers */
  Uniforms uni;              /* the master shared data              */
  ToonUniforms toon_uni;     /* uni + band count, for the toon look */
  DisplaceUniforms disp_uni; /* uni + push settings, for every look */
} Scene;

/* §8.1 ── rebuild the drawing wiring from the current settings ────────── */

/* Points the active look's slots at the right pair of steps and data for the
 * shader the user picked. Every look uses the same per-corner step (it needs the
 * push settings); the per-pixel step and its data are what differ. The toon look
 * needs its own data with the band count; the others just reuse the push data,
 * which the lighting steps can read as plain shared data because it starts with
 * that. */
static void scene_build_shader(Scene *s) {
  switch (s->shade_idx) {
  case SH_PHONG:
    s->shader.vert = vert_displace;
    s->shader.frag = frag_phong;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni;
    break;
  case SH_TOON:
    s->toon_uni.base = s->disp_uni.base;
    s->toon_uni.bands = 4;
    s->shader.vert = vert_displace;
    s->shader.frag = frag_toon;
    s->shader.vert_uni = &s->disp_uni; /* the per-corner step needs the push   */
    s->shader.frag_uni = &s->toon_uni; /* the toon step needs the band count   */
    break;
  case SH_NORMALS:
    s->shader.vert = vert_displace;
    s->shader.frag = frag_normals;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni;
    break;
  case SH_GLASS:
    s->shader.vert = vert_displace;
    s->shader.frag = frag_glass;
    s->shader.vert_uni = &s->disp_uni;
    s->shader.frag_uni = &s->disp_uni;
    break;
  default:
    break;
  }
}

/* Copies the master shared data into the push block and refreshes the push
 * settings — called every frame so the transforms and the clock stay current. */
static void scene_sync_disp(Scene *s) {
  s->disp_uni.base = s->uni;
  s->disp_uni.disp_fn = k_disp_fn[s->disp_idx];
  s->disp_uni.time = s->time;
  s->disp_uni.amplitude = k_amp[s->disp_idx];
  s->disp_uni.frequency = k_freq[s->disp_idx];
  s->disp_uni.mode = s->disp_idx;

  /* keep the toon block's transforms current too, when that look is active */
  if (s->shade_idx == SH_TOON)
    s->toon_uni.base = s->uni;
}

/* §8.2 ── build the sphere and set the starting state ─────────────────── */
static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_sphere();
  s->shade_idx = SH_PHONG;
  s->disp_idx = DM_RIPPLE;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;
  s->time = 0.f;

  s->uni.light_pos = v3(4.f, 5.f, 3.f);
  s->uni.light_col = v3(1.f, 1.f, 1.f);
  s->uni.ambient = v3(0.06f, 0.06f, 0.06f);
  s->uni.shininess = LIGHT_SHININESS;
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.obj_color = v3(0.2f, 0.7f, 0.95f); /* ocean blue */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

  scene_sync_disp(s);
  scene_build_shader(s);
}

/* §8.3 ── the one place the scene moves forward each frame ─────────────── */
/* Advances the clock and the tumble (unless paused), then rebuilds the
 * transforms and refreshes the shared data from them. */
static void scene_tick(Scene *s, float dt) {
  if (!s->paused) {
    s->time += dt;
    s->angle_y += ROT_Y * dt;
    s->angle_x += ROT_X * dt;
  }
  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  s->uni.mvp = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
  scene_sync_disp(s);
}

/* §8.4 ── responding to keys (these don't advance the animation) ──────── */

/* Moves the camera in/out for the new zoom and refreshes its look-from transform. */
static void scene_set_zoom(Scene *s) {
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
}

/* Rebuilds the perspective for a new window size so the sphere isn't stretched. */
static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

static void scene_next_disp(Scene *s) {
  s->disp_idx = (DispMode)((s->disp_idx + 1) % DM_COUNT);
  scene_sync_disp(s);
  scene_build_shader(s);
}

/* ── §9 screen · RENDER — scene_draw + ncurses init / resize / HUD ────── */

/* Draws one frame: wipe the page, draw the sphere through the current look, copy
 * it to the screen. Reads the scene, writes only the page — never the scene. */
static void scene_draw(Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, s->cull_backface);
  fb_blit(fb);
}

/* Screen — the terminal window's current size in cells, re-read on every resize.
 * The page and the perspective are rebuilt from it whenever it changes. */
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
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) {
  (void)s;
  endwin();
}
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* The two overlay lines: a yellow status across the top, a cyan key reminder
 * along the bottom. */
#define PAIR_HUD 3  /* yellow */
#define PAIR_HINT 5 /* cyan   */

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps) {
  /* the status, pinned to the right */
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " %5.1f fps  disp:%s  shader:%s  zoom:%.1f  cull:%s%s ", fps,
           k_disp_names[sc->disp_idx], k_shader_names[sc->shade_idx],
           sc->cam_dist, sc->cull_backface ? "on " : "off",
           sc->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " DISPLACE · UV SPHERE ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  d:disp  s:shader  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10 app — setup, the main loop, and keypresses ────────────────────── */

/*
 * App — the whole running program: the scene, the terminal size, the drawing
 * page, and two flags the signal handlers set. One shared instance (g_app) so
 * the handlers, which take no arguments, can reach it. The two flags are marked
 * volatile sig_atomic_t because a signal can set them at any moment, so the
 * compiler must always read/write them for real.
 */
typedef struct {
  Scene scene;       /* everything drawn                  */
  Screen screen;     /* terminal size                     */
  Framebuffer fb;    /* the off-screen page               */
  volatile sig_atomic_t running;     /* cleared to stop the loop          */
  volatile sig_atomic_t need_resize; /* set when the window was resized   */
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

/* Re-reads the new window size and rebuilds the page and perspective to match. */
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
  case 'd':
  case 'D':
    scene_next_disp(s);
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

/* Reads the clock and sleeps, for timing and the frame-rate cap. */
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

    /* handle a window resize if one happened */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* how long since the last frame (capped, so a hiccup doesn't lurch things) */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* move the animation forward */
    scene_tick(&app->scene, dt_sec);

    /* update the fps readout */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* draw the frame and show it */
    erase();
    scene_draw(&app->scene, &app->fb);
    screen_draw_hud(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* handle a keypress (pause / look / pattern / cull / zoom / quit) */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* wait out the rest of the frame to hold a steady rate */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  screen_free(&app->screen);
  return 0;
}
