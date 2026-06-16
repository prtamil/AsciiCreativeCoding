/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb_raster.c — software rasteriser of the Mandelbulb: raymarch the
 * fractal into a triangle mesh ONCE at init, then render that mesh with a
 * forward rasteriser every frame.  Captures only the outermost skin (a radial
 * ray misses inner pods/cavities) — the raster-vs-raymarch tradeoff.
 *
 * Sister files: raytracing/mandelbulb_raymarcher.c (same fractal + colour map,
 *   per-pixel raymarched); raster/cube_raster.c (same pipeline scaffolding).
 * References: White & Nylander, "Hypercomplex Fractals" (2009) — the spherical
 *   Mandelbulb; Hubbard & Douady (1985) — the 0.5·log(r)·r/dr distance bound;
 *   Inigo Quilez, https://iquilezles.org/articles/mandelbulb/ — the DE.
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

enum {
  FPS_TARGET = 30,
  FPS_UPDATE_MS = 500,
};

/* Camera */
#define CAM_FOV (50.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 3.2f
#define CAM_DIST_MIN 1.4f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f

/* Rotation speed */
#define ROT_Y 0.45f /* rad/s */
#define ROT_X 0.28f

/* Lighting (phong_hue shader) — tuned for legibility on a COARSE terminal
 * grid.  These shape the HSV *value* (brightness); the hue still comes from
 * the smooth-iteration depth, so shape and depth stay orthogonal.  Three
 * low-res adaptations vs textbook Phong:
 *   • broad highlight  — low specular exponent spreads it over several cells
 *   • half-Lambert     — wrap the diffuse so the far side keeps a gradient
 *                        instead of going flat at the ambient floor
 *   • rim light        — brighten grazing angles to pop the fractal's pods
 *                        at the silhouette */
#define LIGHT_SHININESS 24.0f  /* specular exponent — low = broad highlight */
#define LIGHT_AMBIENT 0.08f    /* floor brightness on the darkest face      */
#define DIFFUSE_STRENGTH 0.70f /* half-Lambert diffuse weight               */
#define SPEC_STRENGTH 0.30f    /* specular weight in the brightness sum     */
#define RIM_STRENGTH 0.35f     /* rim (edge) brightness                     */
#define RIM_POWER 2.5f         /* rim falloff — lower = wider rim band      */

/* Terminal cell aspect — same ratio as cube_raster.c */
#define CELL_W 8
#define CELL_H 16

/* How finely we sample the bulb: one ray per cell of an NLAT × NLON grid
 * wrapped around it (like a globe's latitude/longitude lines). */
#define NLAT 28 /* rows, pole to pole   */
#define NLON 56 /* columns, around      */

/* Fractal-shape knobs used while building the mesh. */
#define MB_BAIL 8.0f       /* once a point's distance grows past this, it's "escaped" (outside) */
#define MB_ITERS 48        /* how many times to run the fractal formula        */
#define MB_AUX_ITERS 24    /* fewer iterations are fine just to find a facing direction */
#define MB_HIT_EPS 0.003f  /* a ray has "touched" the surface once it's this close */
#define MB_MARCH_STEPS 220 /* give up after this many steps along one ray      */

/* How a ray creeps toward the surface (cast_ray_to_surface). */
#define MARCH_START_R 1.5f    /* start here, just outside the bulb            */
#define MARCH_SAFETY 0.85f    /* take 85% of the safe step so we don't skip past */
#define MARCH_MIN_STEP 0.004f /* but always move at least this much           */
#define MARCH_MISS_R 0.01f    /* if we reach the centre, the ray hit nothing  */
#define MB_NORMAL_EPS 0.012f  /* how far to peek either side to find the facing */

/* Default fractal power */
#define MB_POWER_DEFAULT 8
#define MB_POWER_MIN 2
#define MB_POWER_MAX 16

/* Hue color pairs — 12-step rainbow wheel (30° per pair) for the
 * fragment shader output, plus two named HUD pairs (yellow status
 * and cyan key hint per the CLAUDE.md HUD spec). */
#define HUE_N 12              /* 12-step rainbow */
#define PAIR_HUD (HUE_N + 1)  /* bright yellow status */
#define PAIR_HINT (HUE_N + 2) /* bright cyan hint     */

/* Default hue bands: how many full rainbow cycles span smooth [0,1] */
#define HUE_BANDS_DEFAULT 3
#define HUE_BANDS_MAX 5

/* characters ordered dark→bright; brightness picks one (the classic Bourke ramp) */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN (int)(sizeof k_bourke - 1)

/* a 4×4 nudge pattern that scatters neighbouring cells across the ramp so
 * smooth gradients don't show hard stripes (ordered "Bayer" dithering) */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.15f     /* how strong that nudge is                     */
#define BOLD_LUMA_ABOVE 0.5f /* brighter than this → draw the cell bold      */
#define CHROMA_MIN 0.04f     /* too grey to have a real colour → use grey    */
#define NEAR_W_EPS 0.001f    /* a triangle corner this close to/behind the camera is skipped */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ── §2 math · LOGIC (pure) — V3, V4, Mat4 + hsv_to_rgb ─────────────────────────────── */

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
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
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

/* v3_gamma — gamma-encode a linear RGB colour for display (sRGB-ish 1/2.2). */
static inline Vec3 v3_gamma(Vec3 c) {
  return v3(powf(c.x, 1.f / 2.2f), powf(c.y, 1.f / 2.2f), powf(c.z, 1.f / 2.2f));
}

/* HSV→RGB, all in [0,1]; h=0 red, ⅓ green, ⅔ blue. */
static Vec3 hsv_to_rgb(float h, float s, float v) {
  h *= 6.0f;
  int i = (int)h;
  float f = h - (float)i;
  float p = v * (1.0f - s);
  float q = v * (1.0f - s * f);
  float t = v * (1.0f - s * (1.0f - f));
  switch (i % 6) {
  case 0:
    return v3(v, t, p);
  case 1:
    return v3(q, v, p);
  case 2:
    return v3(p, v, t);
  case 3:
    return v3(p, q, v);
  case 4:
    return v3(t, p, v);
  default:
    return v3(v, p, q);
  }
}

/* ── §3 shaders · RENDER — VS + 3 FS variants + uniforms ──────────────────────── */

/*
 * The four little "what flows between the drawing stages" records, copying how
 * a GPU works: the vertex stage turns each mesh corner (VSIn) into a screen
 * position + some extra values (VSOut); the rasteriser fills in each covered
 * pixel by blending the triangle's three corners (FSIn); the fragment stage
 * turns that pixel into a colour (FSOut).
 * Heads-up: `u` carries the fractal "depth" value (used to pick a colour), not
 * a texture coordinate — the mesh tucks it into Vertex.u.
 */
typedef struct {
  Vec3 pos;    /* corner position, before any camera transform */
  Vec3 normal; /* which way the surface faces here             */
  float u, v;  /* u = fractal depth (→ hue), v = longitude     */
} VSIn;
typedef struct {
  Vec4 clip_pos;             /* the corner's on-screen position           */
  Vec3 world_pos, world_nrm; /* position + facing, kept for lighting       */
  float u, v;                /* the same extras, blended across the face   */
  float custom[4];           /* spare blended values (custom[0] = depth)   */
} VSOut;
typedef struct {
  Vec3 world_pos, world_nrm; /* this pixel's position + facing */
  float u, v;
  float custom[4];
  int px, py; /* which terminal cell this is (the dither needs it) */
} FSIn;
typedef struct {
  Vec3 color;   /* the pixel's colour                            */
  bool discard; /* true = leave the pixel alone (never set here) */
} FSOut;

/* A drawing stage is just a function pointer; the trailing pointer is its bag
 * of constants (the Uniforms below). */
typedef void (*VertShaderFn)(const VSIn *, VSOut *, const void *);
typedef void (*FragShaderFn)(const FSIn *, FSOut *, const void *);

/*
 * ShaderProgram — one chosen vertex+fragment pair plus the constants they
 * read.  The two constant pointers are kept separate so a future stage could
 * read a different set; here both just point at the scene's one Uniforms.
 */
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni; /* constants for the vertex stage   */
  const void *frag_uni; /* constants for the fragment stage */
} ShaderProgram;

/*
 * Uniforms — the values that stay the same for the whole frame and every
 * shader reads.  The camera/transform matrices are rebuilt each frame (and
 * whenever you zoom or resize); the lighting + colour settings are set once.
 */
typedef struct {
  /* The chain of transforms that places a vertex on screen. */
  Mat4 model, view, proj; /* spin the object / look from the camera / project */
  Mat4 mvp;               /* the three above pre-multiplied (saves work)      */
  Mat4 norm_mat;          /* keeps normals pointing right under that transform */
  /* Lighting + colour. */
  Vec3 light_pos, cam_pos; /* where the light and the eye are            */
  float shininess;         /* how tight the specular highlight is        */
  float hue_bands;         /* how many rainbow cycles span the depth range */
} Uniforms;

/* All 3 shaders share this vertex stage; it carries each corner's fractal-depth
 * value (in custom[0]) down to the pixel stage so it can be coloured. */
static void vert_mb(const VSIn *in, VSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  out->clip_pos = m4_mul_v4(u->mvp, v4(in->pos.x, in->pos.y, in->pos.z, 1.f));
  out->world_pos = m4_pt(u->model, in->pos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
  out->u = in->u;
  out->v = in->v;
  out->custom[0] = in->u; /* fractal depth → colour */
  out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

/* Lit shader: the colour comes from the fractal's depth, the brightness from
 * lighting — two separate signals (the raymarcher could only show one).  The
 * lighting is tuned to read on a coarse grid (see §1). */
static void frag_phong_hue(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));    /* toward light */
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));      /* toward eye   */
  Vec3 H = v3_norm(v3_add(L, V));

  /* Wrap the light right around the shape so even the shadow side keeps some
   * shading (so the form still reads); add a wide highlight and an edge glow,
   * all broad enough to show up on the chunky grid. */
  float wrap = 0.5f * v3_dot(N, L) + 0.5f;
  float diff = wrap * wrap;
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess) * SPEC_STRENGTH;
  float rim = powf(1.f - fmaxf(0.f, v3_dot(N, V)), RIM_POWER) * RIM_STRENGTH;
  float luma = LIGHT_AMBIENT + DIFFUSE_STRENGTH * diff + spec + rim;
  if (luma > 1.f)
    luma = 1.f;

  /* depth → colour, cycling the rainbow hue_bands times from outside in */
  float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
  Vec3 c = hsv_to_rgb(hue, 0.82f, luma);

  out->color = v3_gamma(c); /* brighten for the display */
  out->discard = false;
}

/* Orientation view: colour by which way the surface faces (its compass
 * direction), with up/down setting brightness.  Ignores fractal depth. */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  float azimuth = atan2f(N.z, N.x) / (2.f * (float)M_PI) + 0.5f;
  float value = 0.45f + 0.55f * (N.y * 0.5f + 0.5f);
  out->color = hsv_to_rgb(azimuth, 0.80f, value);
  out->discard = false;
}

/* Pure depth→hue, no lighting — the rasterised twin of
 * mandelbulb_raymarcher.c's depth mode (same colour formula; differences are
 * mesh coverage only). */
static void frag_depth_hue(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
  out->color = hsv_to_rgb(hue, 0.90f, 0.88f);
  out->discard = false;
}

/* Active fragment shader (s cycles); indexes k_shader_names. */
typedef enum { SH_PHONG_HUE = 0, SH_NORMALS, SH_DEPTH_HUE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong_hue", "normals  ", "depth_hue"};

/* ── §4 mesh · LOGIC (pure DE) + BUILD (tessellation) ─────────── */

/*
 * Vertex / Triangle / Mesh — an ordinary triangle mesh.  Corner points live in
 * one list (verts); each triangle just stores three numbers, the slots of its
 * corners (so shared corners aren't duplicated).  Built from the fractal once
 * at startup, then drawn like any normal mesh.
 * Heads-up: a vertex's `u` holds its fractal-depth value (used to colour it),
 * not a texture coordinate.
 */
typedef struct {
  Vec3 pos;    /* where on the bulb's skin this corner sits */
  Vec3 normal; /* which way the surface faces here          */
  float u, v;  /* u = fractal depth (→ colour), v = longitude */
} Vertex;
typedef struct {
  int v[3]; /* indices into Mesh.verts; CCW from outside (front-facing) */
} Triangle;
typedef struct {
  Vertex *verts;  /* heap pool — owned; freed by mesh_free   */
  int nvert;      /* live vertex count                       */
  Triangle *tris; /* heap pool — owned; freed by mesh_free   */
  int ntri;       /* live triangle count                     */
} Mesh;

/* frees both pools; safe to call on an already-freed (zeroed) Mesh */
static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/*
 * Roughly how far this point sits from the Mandelbulb's surface.  We can't
 * measure that directly, so we run the fractal's repeat-the-formula test and
 * turn the result into a SAFE under-estimate — a ray can always step this far
 * without punching through the surface.  (The Hubbard-Douady distance trick,
 * 1985, using White & Nylander's 3-D "power", 2009.)
 * If out_smooth is given it also reports a 0..1 "how deep into the fractal"
 * value used to pick a colour; pass NULL when you only need the distance.
 */
static float mb_de(Vec3 pos, float power, int max_iter, float *out_smooth) {
  Vec3 z = pos;
  float r = 0.0f;
  float dr = 1.0f;
  int esc = max_iter; /* the step where z ran off to infinity (escaped) */

  for (int i = 0; i < max_iter; i++) {
    r = v3_len(z);
    if (r > MB_BAIL) { /* escaped — this point is outside the set */
      esc = i;
      break;
    }

    /* describe z by its angles, the way latitude/longitude describe a globe */
    float theta = acosf(z.z / (r + 1e-8f));
    float phi = atan2f(z.y, z.x);

    /* keep tracking how fast nearby points spread apart (dr) — the distance
     * estimate below divides by it */
    float rp = powf(r, power);
    dr = (rp / r) * power * dr + 1.0f;

    /* the fractal step: raise z to the power (multiply angles, grow length)
     * then add the starting point back in */
    float st = sinf(power * theta), ct = cosf(power * theta);
    float sp = sinf(power * phi), cp = cosf(power * phi);
    z = v3_add(v3_scale(v3(st * cp, st * sp, ct), rp), pos);
  }

  if (out_smooth) {
    /* turn the (whole-number) escape step into a smooth 0..1 "depth" so
     * colour bands blend instead of stair-stepping */
    if (r > MB_BAIL && esc < max_iter) {
      float mu = (float)esc + 1.0f -
                 logf(logf(r) / logf(MB_BAIL)) / logf(power + 1e-6f);
      mu /= (float)max_iter;
      *out_smooth = mu < 0.f ? 0.f : (mu > 1.f ? 1.f : mu);
    } else {
      *out_smooth = 1.0f; /* never escaped — treat as fully inside */
    }
  }

  if (r < 1e-7f)
    return 0.0f;                  /* essentially at the centre — zero distance */
  return 0.5f * logf(r) * r / dr; /* the safe distance estimate */
}

/* Which way the surface faces here.  No formula for it, so we taste the
 * distance a hair either side along each axis — the surface tilts toward
 * wherever the distance grows fastest.  (Cheaper iteration: a facing direction
 * needs less precision than a distance.) */
static Vec3 mb_normal(Vec3 p, float power) {
  const float H = MB_NORMAL_EPS;
  float dx = mb_de(v3(p.x + H, p.y, p.z), power, MB_AUX_ITERS, NULL) -
             mb_de(v3(p.x - H, p.y, p.z), power, MB_AUX_ITERS, NULL);
  float dy = mb_de(v3(p.x, p.y + H, p.z), power, MB_AUX_ITERS, NULL) -
             mb_de(v3(p.x, p.y - H, p.z), power, MB_AUX_ITERS, NULL);
  float dz = mb_de(v3(p.x, p.y, p.z + H), power, MB_AUX_ITERS, NULL) -
             mb_de(v3(p.x, p.y, p.z - H), power, MB_AUX_ITERS, NULL);
  return v3_norm(v3(dx, dy, dz));
}

/* turn a (latitude i, longitude j) grid cell into a direction pointing out
 * from the centre — like aiming at one spot on a globe. */
static Vec3 uv_sphere_direction(int i, int j) {
  float theta =
      -(float)M_PI * 0.5f + ((float)i / (float)(NLAT - 1)) * (float)M_PI;
  float phi = ((float)j / (float)NLON) * 2.0f * (float)M_PI;
  float ct = cosf(theta);
  return v3(ct * cosf(phi), sinf(theta), ct * sinf(phi));
}

/* Fire a ray from outside the bulb straight toward the centre and creep inward
 * until it touches the surface (or reaches the middle having hit nothing).  On
 * a hit, fills in *out_pos + *out_smooth and returns true; on a miss leaves
 * them alone and returns false. */
static bool cast_ray_to_surface(Vec3 dir, float power, Vec3 *out_pos,
                                float *out_smooth) {
  float r = MARCH_START_R;
  for (int step = 0; step < MB_MARCH_STEPS; step++) {
    Vec3 p = v3_scale(dir, r);
    float d = mb_de(p, power, MB_ITERS, NULL);

    if (d < MB_HIT_EPS) {
      mb_de(p, power, MB_ITERS, out_smooth); /* second pass: smooth */
      *out_pos = p;
      return true;
    }
    r -= fmaxf(d * MARCH_SAFETY, MARCH_MIN_STEP); /* safe march step */
    if (r < MARCH_MISS_R)
      return false; /* reached the centre — no surface along this ray */
  }
  return false;
}

/* The three helpers above just describe the fractal; the three below actually
 * BUILD the mesh from it.  This runs once at startup and again when you change
 * the power ('p'/'P') — never during normal animation. */

/* the spot in the flat vertex-index list for grid cell (latitude i, longitude j) */
static inline int grid_index(int i, int j) { return i * NLON + j; }

/* Step 1 of building the mesh: fire one ray per grid cell; every ray that hits
 * becomes a vertex (its position, which way it faces, and its depth-for-
 * colour).  vidx remembers which cell made which vertex (−1 means the ray
 * missed).  Returns how many vertices we ended up with. */
static int collect_skin_vertices(float power, Vertex *verts, int *vidx) {
  int nvert = 0;
  for (int i = 0; i < NLAT; i++) {
    for (int j = 0; j < NLON; j++) {
      Vec3 hit_pos;
      float smooth;
      Vec3 dir = uv_sphere_direction(i, j);

      if (cast_ray_to_surface(dir, power, &hit_pos, &smooth)) {
        vidx[grid_index(i, j)] = nvert;
        verts[nvert].pos = hit_pos;
        verts[nvert].normal = mb_normal(hit_pos, power);
        verts[nvert].u = smooth; /* hue input */
        verts[nvert].v = (float)j / (float)NLON;
        nvert++;
      } else {
        vidx[grid_index(i, j)] = -1;
      }
    }
  }
  return nvert;
}

/* Step 2: join neighbouring vertices into little four-sided patches, each cut
 * into two triangles.  If any corner's ray missed, that triangle is skipped —
 * that's why very spiky areas show gaps.  Longitude wraps around so the seam
 * at the back closes up.  Returns how many triangles we made. */
static int stitch_quads(const int *vidx, Triangle *tris) {
  int ntri = 0;
  for (int i = 0; i < NLAT - 1; i++) {
    for (int j = 0; j < NLON; j++) {
      int j1 = (j + 1) % NLON;
      int v00 = vidx[grid_index(i, j)];
      int v01 = vidx[grid_index(i, j1)];
      int v10 = vidx[grid_index(i + 1, j)];
      int v11 = vidx[grid_index(i + 1, j1)];

      if (v00 >= 0 && v01 >= 0 && v11 >= 0)
        tris[ntri++] = (Triangle){{v00, v11, v01}};
      if (v00 >= 0 && v10 >= 0 && v11 >= 0)
        tris[ntri++] = (Triangle){{v00, v10, v11}};
    }
  }
  return ntri;
}

/* Build the whole mesh of the bulb's outer skin: grab memory, shoot a sphere
 * of rays to find surface points, then stitch those points into triangles. */
static Mesh tessellate_mandelbulb(float power) {
  Vertex *verts = malloc((size_t)(NLAT * NLON) * sizeof(Vertex));
  Triangle *tris = malloc((size_t)((NLAT - 1) * NLON * 2) * sizeof(Triangle));
  int *vidx = malloc((size_t)(NLAT * NLON) * sizeof(int)); /* grid → vert, -1 */

  int nvert = collect_skin_vertices(power, verts, vidx);
  int ntri = stitch_quads(vidx, tris);

  free(vidx);
  return (Mesh){verts, nvert, tris, ntri};
}

/* ── §5 framebuffer · RENDER — zbuf + cbuf + 12-pair hue palette + Bourke ──────── */

/* Cell — one finished terminal character: the letter to print, its colour, and
 * whether it's bold.  The pixel's colour has already been turned into a
 * character (by brightness) and a colour (by hue), so drawing just prints it.
 * ch=0 means nothing was drawn here (skipped). */
typedef struct {
  char ch;        /* the character to print (0 = empty) */
  int color_pair; /* its colour (1..HUE_N)              */
  bool bold;      /* bold for bright cells              */
} Cell;

/* Framebuffer — where we draw before showing it: one grid for colours (cbuf)
 * and one for depth (zbuf), both cols×rows.  The depth grid lets nearer
 * surfaces hide farther ones — each cell keeps the closest thing drawn there so
 * far (the classic z-buffer, Catmull 1974).  Sized to the terminal and
 * re-made on resize. */
typedef struct {
  float *zbuf;    /* closest depth seen per cell (huge = empty); freed by fb_free */
  Cell *cbuf;     /* the finished character per cell; freed by fb_free            */
  int cols, rows; /* grid size = terminal size                                   */
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

/* 12-step hue wheel → ncurses pairs (256-colour codes, 8-colour fallback). */
static const int k_hue256[HUE_N] = {
    196, 202, 208, 226, 154, 46, 48,
    51,  39,  21,  93,  201
    /*  red  org  amb  yel  lim  grn teal cyn sky  blu vio  mag */
};
static const int k_hue8[HUE_N] = {COLOR_RED,    COLOR_RED,     COLOR_RED,
                                  COLOR_YELLOW, COLOR_YELLOW,  COLOR_GREEN,
                                  COLOR_CYAN,   COLOR_CYAN,    COLOR_BLUE,
                                  COLOR_BLUE,   COLOR_MAGENTA, COLOR_MAGENTA};

static void color_init(void) {
  start_color();
  use_default_colors();
  for (int i = 0; i < HUE_N; i++) {
    if (COLORS >= 256)
      init_pair(i + 1, k_hue256[i], COLOR_BLACK);
    else
      init_pair(i + 1, k_hue8[i], COLOR_BLACK);
  }
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* which colour an RGB triple is — its spot on the colour wheel as 0..1 (red,
 * then yellow, green, … back to red).  Returns -1 if it's basically grey. */
static float rgb_hue(Vec3 c) {
  float r = c.x, g = c.y, b = c.z;
  float cmax = r > g ? (r > b ? r : b) : (g > b ? g : b);
  float cmin = r < g ? (r < b ? r : b) : (g < b ? g : b);
  float delta = cmax - cmin;
  if (delta < CHROMA_MIN)
    return -1.f;

  float h;
  if (cmax == r)
    h = fmodf((g - b) / delta, 6.0f);
  else if (cmax == g)
    h = (b - r) / delta + 2.0f;
  else
    h = (r - g) / delta + 4.0f;
  if (h < 0.0f)
    h += 6.0f;
  return h / 6.0f;
}

/* how bright a colour looks to the eye (green counts most; the Rec.709 mix) */
static inline float rec709_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* pick a character from the ramp by brightness (0 = darkest, 1 = brightest) */
static inline int ramp_index(float v) { return (int)(v * (BOURKE_LEN - 1)); }

/* Pick the terminal colour closest to this RGB's hue — so the bulb's colour on
 * screen reflects the surface's actual colour, not just how bright it is. */
static int rgb_to_pair(Vec3 c) {
  float h = rgb_hue(c);
  if (h < 0.f)
    return 1; /* near-grey → red pair (any pair works) */
  return (int)(h * (float)HUE_N) % HUE_N + 1;
}

/* Turn a pixel's RGB into a terminal character: brightness picks the character
 * (with a dither nudge so gradients stay smooth), hue picks the colour, and
 * bright pixels are drawn bold. */
static Cell color_to_cell(Vec3 color, int px, int py) {
  float luma = rec709_luma(color);
  float dithered = luma + (k_bayer[py & 3][px & 3] - 0.5f) * DITHER_AMP;
  dithered = dithered < 0.f ? 0.f : dithered > 1.f ? 1.f : dithered;
  char ch = k_bourke[ramp_index(dithered)];
  return (Cell){ch, rgb_to_pair(color), luma > BOLD_LUMA_ABOVE};
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

/* ── §6 pipeline · RENDER — vertex transform → cull → raster → FS ─── */

/* For pixel (px,py): how much of it belongs to each of the triangle's 3
 * corners.  Those three weights add to 1 and are used to blend the corners'
 * values; if any comes out negative the pixel is outside the triangle. */
static void barycentric(const float sx[3], const float sy[3], float px,
                        float py, float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-6f) { /* zero-area (degenerate) triangle — mark outside */
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/* Stage 1: run the vertex stage on the triangle's 3 corners, turning each into
 * a screen position plus the extras the pixel stage will need. */
static void run_vertex_shader(const Mesh *mesh, const Triangle *tri,
                              const ShaderProgram *sh, VSOut vo[3]) {
  for (int vi = 0; vi < 3; vi++) {
    const Vertex *vtx = &mesh->verts[tri->v[vi]];
    VSIn in = {vtx->pos, vtx->normal, vtx->u, vtx->v};
    memset(&vo[vi], 0, sizeof vo[vi]);
    sh->vert(&in, &vo[vi], sh->vert_uni);
  }
}

/* true if all 3 corners are behind the camera — the whole triangle is
 * off-screen, so skip it. */
static bool all_behind_near_plane(const VSOut vo[3]) {
  return vo[0].clip_pos.w < NEAR_W_EPS && vo[1].clip_pos.w < NEAR_W_EPS &&
         vo[2].clip_pos.w < NEAR_W_EPS;
}

/* Stage 2: turn each corner's 3-D position into an actual (column, row) on
 * screen — including the divide that makes far things smaller and the flip that
 * puts world-up at the top of the terminal. */
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

/* The sign tells us which way the triangle faces.  After the Y-flip, faces
 * pointing toward us come out negative — so culling drops the ones that are
 * >= 0 (facing away). */
static float signed_area(const float sx[3], const float sy[3]) {
  return (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
}

/* Stage 3: fill the triangle in.  For each cell it covers, keep the cell only
 * if it's truly inside the triangle and closer than whatever's already there,
 * then run the pixel shader and store the result. */
static void rasterize_fragments(Framebuffer *fb, const VSOut vo[3],
                                const float sx[3], const float sy[3],
                                const float sz[3], const ShaderProgram *sh) {
  int cols = fb->cols, rows = fb->rows;
  int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
  int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
  int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
  int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

  for (int py = y0; py <= y1; py++) {
    for (int px = x0; px <= x1; px++) {
      float b[3];
      barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
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
      for (int ci = 0; ci < 4; ci++)
        fsin.custom[ci] = b[0] * vo[0].custom[ci] + b[1] * vo[1].custom[ci] +
                          b[2] * vo[2].custom[ci];

      FSOut fsout;
      fsout.discard = false;
      sh->frag(&fsin, &fsout, sh->frag_uni);
      if (fsout.discard)
        continue;

      fb->cbuf[idx] = color_to_cell(fsout.color, px, py);
    }
  }
}

/*
 * Draw the whole mesh, one triangle at a time, reading like its own recipe:
 * transform the 3 corners → drop it if it's off-screen → place it on screen →
 * drop it if it faces away → fill it in.
 */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               const ShaderProgram *sh, bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    VSOut vo[3];
    run_vertex_shader(mesh, tri, sh, vo);
    if (all_behind_near_plane(vo))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(vo, cols, rows, sx, sy, sz);
    if (cull_backface && signed_area(sx, sy) >= 0.f)
      continue;

    rasterize_fragments(fb, vo, sx, sy, sz, sh);
  }
}

/* ── §7 scene · SIMULATION — state; tick (sim) + draw (render) + user events ────────── */

/*
 * Scene — the whole demo's mutable state, grouped by concept.  The fractal
 * GEOMETRY is the Mesh (built once from `power`); the only thing that advances
 * per tick is the orientation; the rest are user knobs and the derived render
 * state the shaders read.
 */
typedef struct {
  /* WHAT — the Mandelbulb being rendered. */
  Mesh mesh;              /* outermost-skin triangle mesh, built from `power` */
  float power;            /* fractal exponent — defines the shape (p/P)       */
  float angle_x, angle_y; /* tumble orientation — the ONLY per-tick sim state */

  /* HOW the user drives it — control knobs + run-state. */
  float cam_dist;      /* zoom: camera distance (+/-)            */
  int hue_bands;       /* rainbow cycles across fractal depth (b) */
  ShaderIdx shade_idx; /* active fragment shader (s)             */
  bool cull_backface;  /* back-face culling on/off (c)           */
  bool paused;         /* freeze the tick (space)                */

  /* Derived RENDER state — recomputed from the above (events / each tick). */
  ShaderProgram shader; /* the program resolved for shade_idx       */
  Uniforms uni;         /* the per-frame constants the shaders read */
} Scene;

/* §7.1 ── narrow helpers (each takes the SUB-TYPE it touches, not Scene*).
 * They recompute derived state or rebuild the mesh; called by init, the tick,
 * and the user-event responders in §9.  None advances the tick. */

/* shader_for — pure mapping: pick the (vertex, fragment) program for a shader
 * index, both reading the same Uniforms block. */
static ShaderProgram shader_for(ShaderIdx idx, const Uniforms *uni) {
  switch (idx) {
  case SH_NORMALS:
    return (ShaderProgram){vert_mb, frag_normals, uni, uni};
  case SH_DEPTH_HUE:
    return (ShaderProgram){vert_mb, frag_depth_hue, uni, uni};
  case SH_PHONG_HUE:
  default:
    return (ShaderProgram){vert_mb, frag_phong_hue, uni, uni};
  }
}

/* mesh_rebuild — replace the mesh wholesale by re-tessellating at `power`
 * (the one-shot BUILD behind the 'p'/'P' keys). */
static void mesh_rebuild(Mesh *m, float power) {
  mesh_free(m);
  *m = tessellate_mandelbulb(power);
}

/* Rebuild the combined object→screen transform from its three parts.  Called
 * whenever any of them changes, so a zoom/resize takes effect even while paused
 * (the per-frame tick skips this rebuild when paused). */
static void uniforms_update_mvp(Uniforms *u) {
  u->mvp = m4_mul(u->proj, m4_mul(u->view, u->model));
}

/* uniforms_set_view — point the camera at the origin from `cam_dist` on +Z. */
static void uniforms_set_view(Uniforms *u, float cam_dist) {
  u->cam_pos = v3(0.f, 0.f, cam_dist);
  u->view = m4_lookat(u->cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  uniforms_update_mvp(u);
}

/* uniforms_set_proj — rebuild the perspective projection for a terminal size. */
static void uniforms_set_proj(Uniforms *u, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  u->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
  uniforms_update_mvp(u);
}

/* §7.2 ── init (one-shot, not a tick): seed all state + build the first mesh. */
static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->power = (float)MB_POWER_DEFAULT;
  s->shade_idx = SH_PHONG_HUE;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;
  s->hue_bands = HUE_BANDS_DEFAULT;

  s->uni.light_pos = v3(3.f, 4.f, 3.f);
  s->uni.shininess = LIGHT_SHININESS;
  s->uni.hue_bands = (float)s->hue_bands;
  s->uni.model = m4_identity(); /* so the view's mvp is valid pre-tick */

  uniforms_set_view(&s->uni, s->cam_dist);
  uniforms_set_proj(&s->uni, cols, rows);
  s->shader = shader_for(s->shade_idx, &s->uni);
  mesh_rebuild(&s->mesh, s->power); /* tessellate — pays mesh cost once here */
}

/* §7.3 ── SIMULATION — the ONLY per-tick state advance.  Mutates angle_x/y
 * (when !paused), then derives model / mvp / norm_mat. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->angle_y += ROT_Y * dt;
  s->angle_x += ROT_X * dt;
  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  uniforms_update_mvp(&s->uni);
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
}

/* §7.4 ── RENDER — state → framebuffer → terminal.  Reads the Scene (const),
 * writes only the framebuffer (via the pipeline) and the screen. */
static void scene_draw(const Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, s->cull_backface);
  fb_blit(fb);
}

/* ── §8 screen · RENDER — ncurses init / resize / HUD / present ────────────────── */

/* Screen — the terminal size in cells, cached from getmaxyx (init + SIGWINCH);
 * the framebuffer is reallocated to match. */
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

/* yellow status row 0, cyan key-hint bottom row (CLAUDE.md HUD spec) */
static void screen_draw_hud(const Screen *sc, const Scene *s, double fps) {
  char status[140];
  snprintf(status, sizeof status,
           " %5.1f fps  tris:%d  power:%.0f  bands:%d  shader:%s  cull:%s%s ",
           fps, s->mesh.ntri, s->power, s->hue_bands,
           k_shader_names[s->shade_idx], s->cull_backface ? "on " : "off",
           s->paused ? " PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > sc->cols)
    slen = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - slen, "%s", status);
  mvprintw(0, 0, " MANDELBULB · RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(
      sc->rows - 1, 0,
      " q:quit  spc:pause  s:shader  c:cull  p/P:power  b:bands  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app · ORCHESTRATION + PERFORMANCE + USER EVENTS — main loop, signals, resize ─────────────────────── */

/* App — top-level program state.  Single static instance (g_app) so the
 * argument-less signal handlers can reach it; running / need_resize are
 * volatile sig_atomic_t (the only type safe to touch from a handler). */
typedef struct {
  Scene scene;
  Screen screen;
  Framebuffer fb;
  volatile sig_atomic_t running;     /* 0 = exit loop (SIGINT/SIGTERM/q) */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending             */
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

/* USER EVENTS (resize / keys) — mutate scene + render state but never advance
 * the tick. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_free(&app->fb);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  uniforms_set_proj(&app->scene.uni, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
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
    s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
    s->shader = shader_for(s->shade_idx, &s->uni);
    break;
  case 'c':
  case 'C':
    s->cull_backface = !s->cull_backface;
    break;
  case 'p':
    s->power += 1.0f;
    if (s->power > (float)MB_POWER_MAX)
      s->power = (float)MB_POWER_MAX;
    mesh_rebuild(&s->mesh, s->power);
    break;
  case 'P':
    s->power -= 1.0f;
    if (s->power < (float)MB_POWER_MIN)
      s->power = (float)MB_POWER_MIN;
    mesh_rebuild(&s->mesh, s->power);
    break;
  case 'b':
  case 'B':
    s->hue_bands = s->hue_bands % HUE_BANDS_MAX + 1;
    s->uni.hue_bands = (float)s->hue_bands;
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    uniforms_set_view(&s->uni, s->cam_dist);
    break;
  case '-':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    uniforms_set_view(&s->uni, s->cam_dist);
    break;
  default:
    break;
  }
  return true;
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

  /* Mesh build happens here — show a "building..." message */
  mvprintw(app->screen.rows / 2, app->screen.cols / 2 - 10,
           "building mesh (power=%d)...", MB_POWER_DEFAULT);
  refresh();

  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_disp = 0.0;
  const int64_t frame_ns = NS_PER_SEC / FPS_TARGET;

  /* the one place the layers combine; nothing outside advances sim state */
  while (app->running) {
    /* USER EVENT (not part of the tick): SIGWINCH-driven resize at loop top. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* [1] PERFORMANCE — measure dt since last frame, capped. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* USER EVENT (not part of the tick): drain input before advancing. */
    int key;
    while ((key = getch()) != ERR)
      if (!app_handle_key(app, key)) {
        app->running = 0;
        break;
      }
    if (!app->running)
      break;

    /* [2] SIMULATION — the ONLY sim-advancing call this tick. */
    scene_tick(&app->scene, dt_sec);

    /* [3] PERFORMANCE — fps rolling average. */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* [4] RENDER — state → framebuffer → terminal (reads scene, never mutates). */
    erase();
    scene_draw(&app->scene, &app->fb);
    screen_draw_hud(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* [5] PERFORMANCE — sleep to the frame cap. */
    clock_sleep_ns(frame_ns - (clock_ns() - now));
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  screen_free(&app->screen);
  return 0;
}
