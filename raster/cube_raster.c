/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube_raster.c — the smallest software 3-D renderer in this folder: a glossy
 * orange cube tumbling on black, drawn one triangle at a time on the CPU (no
 * GPU). Press 's' to flip between four looks (phong, toon, normals, fresnel),
 * 'c' to show/hide the inside faces, '+/-' to zoom, space to pause, q to quit.
 *
 * Read this one first — sphere_raster.c, torus_raster.c, and
 * displace_raster.c reuse the same skeleton with a different shape, and
 * deferred_rendering_pipeline.c adds a G-buffer on top.
 * Ideas from: Reinhard tone-map (SIGGRAPH '02); the signed-area triangle test
 *   (Möller, Game Programming Gems 2000); z-buffer (Catmull 1974).
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/cube_raster.c -o cube_rt -lncurses -lm
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

/* §1.1 frame rate */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 view geometry */
#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
/* A corner this close to the eye (or behind it) is dropped. The step that makes
 * far things smaller divides by distance, and that blows up for a point sitting
 * right on top of the camera. */
#define CLIP_W_MIN 0.001f
#define CAM_DIST 2.8f /* how far back the camera starts          */
#define CAM_DIST_MIN 1.0f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f

/* A terminal cell is taller than it is wide; these numbers let the projection
 * correct for that, otherwise the cube would look like a squashed tall blob. */
#define CELL_W 8
#define CELL_H 16

/* §1.3 cube geometry */
#define CUBE_S 0.75f /* half the cube's width: corners sit at ±this */

/* §1.3b how the light dims with distance (used by the phong shader). A nearby
 * light plus this fade gives each flat face a bright-to-dim gradient across it;
 * without it, a face all points the same way and shades to one flat tone. */
#define LIGHT_ATTEN_LIN 0.09f
#define LIGHT_ATTEN_QUAD 0.07f

/* §1.4 how fast the cube spins (radians/sec) — different on each axis so it
 * tumbles instead of just spinning flat. */
#define ROT_Y 0.55f
#define ROT_X 0.37f

/* §1.5 the fresnel look — a glowing outline. Surfaces turned edge-on to you get
 * a bright cool rim; the inside stays dark. On the black terminal it reads like
 * a glowing wireframe of the cube. */
#define FRESNEL_POWER 1.8f /* smaller = a broader, brighter rim          */
static const float FRESNEL_BASE[3] = {0.05f, 0.07f, 0.12f}; /* dark cool inside */
static const float FRESNEL_RIM[3] = {0.55f, 0.95f, 1.35f};  /* bright icy rim   */

/* §1.6 the brightness-to-character ladder: a space for near-black up to '@' for
 * the brightest. Brighter pixels get busier characters. */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 a fixed 4×4 nudge pattern (values 0..1). Added to each cell's brightness
 * so neighbouring cells of similar brightness pick different characters, which
 * hides the steps in a smooth gradient. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};

#define DITHER_AMP 0.12f

/* §1.8 ncurses colour-pair numbers. */
#define PAIR_CUBE_BASE 1 /* +0..215 = the 216 cube colours         */
#define PAIR_HUD 217     /* yellow, the top status row             */
#define PAIR_HINT 218    /* cyan, the bottom hint row              */

/* §1.9 turning a colour into a cell — see paint_compute_cell (§4). */
#define CUBE_LEVELS 6              /* 6 steps per channel → 6×6×6 = 216 colours  */
#define DISPLAY_GAMMA 2.2f         /* screen-brightness correction, applied last  */
#define BOLD_LUMA_THRESHOLD 0.85f  /* brighter than this → draw the cell bold     */
#define DIM_LUMA_THRESHOLD 0.15f   /* darker than this   → draw the cell dim      */

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

/* Makes the matrix used to rotate the surface-facing directions (normals). You
 * can't just reuse the cube's own matrix: if a shape were stretched unevenly,
 * its normals would come out skewed. For our cube (only rotated) this works out
 * the same, but doing it properly means a stretched shape would still look right. */
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

/* ── §4 paint — turning a colour into one terminal cell ────────────────── */

static int g_256;

/* Sets up our colours: 216 of them arranged as a 6×6×6 cube of reds × greens ×
 * blues, plus a yellow for the status bar and a cyan for the hint line. Falls
 * back to plain white if the terminal can't do 256 colours. */
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

/* Two small steps that bring a colour into the range the screen can show:
 *   reinhard — gently rolls over-bright values toward white instead of clipping
 *   gamma    — a brightness correction so mid-tones look right
 * Without them, every bright colour would slam into the same brightest cube
 * cell and the highlights would lose all their shape. */
static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) {
  return powf(clamp01(x), 1.f / DISPLAY_GAMMA);
}

/* Brings a too-bright colour into screen range, channel by channel (the roll-off
 * then the brightness correction). The one and only place that happens. */
static inline Vec3 tonemap_encode(Vec3 hdr) {
  return v3(gamma_enc(reinhard(hdr.x)), gamma_enc(reinhard(hdr.y)),
            gamma_enc(reinhard(hdr.z)));
}

/* How bright a colour looks to the eye (green counts most, blue least). The
 * character is chosen from this, so a brighter-looking colour gets a busier one. */
static inline float v3_luma(Vec3 c) {
  return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

/* to_cube_level — quantise one encoded channel [0,1] to a cube level
 * 0..CUBE_LEVELS-1 (round-to-nearest, clamped). */
static inline int to_cube_level(float c) {
  int lvl = (int)(c * (CUBE_LEVELS - 1) + 0.5f);
  if (lvl < 0)
    lvl = 0;
  if (lvl > CUBE_LEVELS - 1)
    lvl = CUBE_LEVELS - 1;
  return lvl;
}

/* Finds the closest of our 216 colours to this RGB and returns its pair number. */
static inline int cube_pair(Vec3 rgb) {
  return PAIR_CUBE_BASE + to_cube_level(rgb.x) * (CUBE_LEVELS * CUBE_LEVELS) +
         to_cube_level(rgb.y) * CUBE_LEVELS + to_cube_level(rgb.z);
}

/* Picks the character for a given brightness (0..1) from the ladder. */
static inline int ramp_index(float luma) {
  int idx = (int)(luma * (BOURKE_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= BOURKE_LEN)
    idx = BOURKE_LEN - 1;
  return idx;
}

/* The nudge for this cell from the 4×4 pattern, centred around zero and scaled
 * down. Added to brightness before picking a character, to smooth gradients. */
static inline float bayer_dither(int px, int py) {
  return (k_bayer[py & 3][px & 3] - 0.5f) * DITHER_AMP;
}

/* Cell — one character on screen, the thing the renderer ultimately produces.
 * Kept as plain data (not drawn straight to ncurses) so the drawing math just
 * fills cells and a single pass at the end pushes them to the terminal.
 *   ch   — the character to show; 0 means empty (left blank, so the black
 *          background shows through)
 *   pair — which colour (a number into our 216 colours, §4)
 *   attr — extra style bits: bold on bright cells, dim on dark ones */
typedef struct {
  char ch;
  int pair;
  int attr;
} Cell;

/* Turns a colour into a finished cell: bring it into screen range, measure its
 * brightness, pick the closest colour and the matching character (with a dither
 * nudge so gradients stay smooth), and choose bold/dim. The nudge only affects
 * the character, not the colour, so smooth shading doesn't band on the grid. */
static Cell paint_compute_cell(Vec3 col, int px, int py) {
  Vec3 rgb = tonemap_encode(col);
  float luma = v3_luma(rgb); /* un-dithered, used for the bold/dim choice */

  Cell c;
  c.pair = g_256 ? cube_pair(rgb) : PAIR_CUBE_BASE;
  c.ch = k_bourke[ramp_index(clamp01(luma + bayer_dither(px, py)))];
  c.attr = (luma > BOLD_LUMA_THRESHOLD)  ? A_BOLD
           : (luma < DIM_LUMA_THRESHOLD) ? A_DIM
                                         : A_NORMAL;
  return c;
}

/* ── §5 shaders — four ways to colour the cube ─────────────────────────── */

/* §5.1 ── the little records that flow through the drawing steps ───────── */

/* Just like a GPU, drawing happens in two programmable steps — one per corner,
 * then one per pixel — and these four records carry data between them:
 *   VSIn  — a cube corner going IN to the per-corner step (its raw position etc.)
 *   VSOut — what that step produces (where the corner lands + info to pass on)
 *   FSIn  — VSOut blended across the triangle to a single pixel
 *   FSOut — what the per-pixel step produces (a colour, + a "skip me" flag)
 * A matched per-corner + per-pixel pair is a ShaderProgram (below). */
typedef struct {
  Vec3 pos;    /* the corner's position in the cube's own coordinates */
  Vec3 normal; /* which way its face points (same for all 4 of a face) */
  float u, v;  /* texture coords, carried along but unused here        */
} VSIn;

typedef struct {
  Vec4 clip_pos;  /* where the corner lands, before the divide-by-distance */
  Vec3 world_pos; /* the corner's spot in the world, for the lighting      */
  Vec3 world_nrm; /* which way it faces in the world                       */
  float u, v;     /* texture coords, passed along                          */
} VSOut;

typedef struct {
  Vec3 world_pos; /* this pixel's spot in the world (blended from corners)  */
  Vec3 world_nrm; /* which way the surface faces here                       */
  float u, v;     /* blended texture coords                                 */
  int px, py;     /* which screen cell this is                              */
} FSIn;

typedef struct {
  Vec3 color;   /* the colour this pixel came out (before screen-range fix) */
  bool discard; /* true → don't draw this pixel at all                      */
} FSOut;

typedef void (*VertShaderFn)(const VSIn *in, VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in, FSOut *out, const void *uni);

/* ShaderProgram — one swappable "look": a per-corner step, a per-pixel step, and
 * a pointer to the shared values each reads. The drawing code calls the two
 * steps through these pointers without knowing what they do, so switching looks
 * is just swapping this struct (scene_build_shader). The value pointers are
 * untyped so a look can point at its own extra data (e.g. ToonUniforms). */
typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni;
  const void *frag_uni;
} ShaderProgram;

/* Uniforms — the values that stay the same for a whole frame and that every look
 * reads. Grouped:
 *   model/view/proj/mvp/norm_mat — the transforms: spin the cube, look from the
 *                                  camera, apply perspective, all chained into
 *                                  mvp, plus the one for the facing directions
 *   light_pos/light_col/ambient  — the one light, plus a dim everywhere-fill
 *   cam_pos                      — where the eye is (for highlights)
 *   obj_color/shininess          — the cube's colour and how glossy it is
 * There's only one cube and one light, so these live together here rather than
 * in their own types. Ref: Blinn-Phong shading (Blinn, SIGGRAPH '77). */
typedef struct {
  Mat4 model, view, proj, mvp, norm_mat;
  Vec3 light_pos, light_col, ambient, cam_pos, obj_color;
  float shininess;
} Uniforms;

/* ToonUniforms — the extra data the toon look needs: all the usual Uniforms plus
 * how many flat brightness steps to snap to. It wraps Uniforms because a look
 * gets handed a single pointer, and toon needs `bands` carried alongside.
 *   base  — the full Uniforms (transforms + light + material)
 *   bands — how many flat steps (4 here); more = a smoother stepped look */
typedef struct {
  Uniforms base;
  int bands;
} ToonUniforms;

/* §5.2 ── the per-corner step (shared by all four looks) ──────────────── */

/* All four looks need the same per-corner work — where the corner lands on
 * screen, where it sits in the world, and which way it faces — so one shared
 * step does it; the looks differ only in the per-pixel colouring below. */
static void vert_default(const VSIn *in, VSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  out->clip_pos = m4_mul_v4(u->mvp, v4(in->pos.x, in->pos.y, in->pos.z, 1.f));
  out->world_pos = m4_pt(u->model, in->pos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
  out->u = in->u;
  out->v = in->v;
}

/* §5.3 ── phong: ordinary realistic lighting ─────────────────────────── */

/* The standard lit look: a dim base everywhere, plus light where the surface
 * faces the lamp (brighter the more directly), plus a shiny hotspot. The lamp
 * also fades with distance.
 *
 * Each cube face points one single way, so a far-off lamp would paint a whole
 * face one flat tone. Putting the lamp CLOSE (set in scene_init) means its
 * distance and direction change noticeably across one face, so you get a real
 * gradient — bright near the lamp's corner, dim at the far one — plus a glossy
 * spot that slides around as the cube turns. */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 to_light = v3_sub(u->light_pos, in->world_pos);
  float dist = v3_len(to_light);
  Vec3 L = v3_scale(to_light, 1.f / fmaxf(dist, 1e-4f));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  /* the distance fade — this is what gives a flat face its gradient: distance
   * and light direction change across the face even though its facing doesn't */
  float atten =
      1.f / (1.f + LIGHT_ATTEN_LIN * dist + LIGHT_ATTEN_QUAD * dist * dist);
  float diff = fmaxf(0.f, v3_dot(N, L)) * atten;
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess) * atten;

  Vec3 c = u->obj_color;
  out->color = v3(u->ambient.x + c.x * u->light_col.x * diff + spec * 0.5f,
                  u->ambient.y + c.y * u->light_col.y * diff + spec * 0.5f,
                  u->ambient.z + c.z * u->light_col.z * diff + spec * 0.5f);
  out->discard = false;
}

/* §5.4 ── toon: flat cartoon shading ─────────────────────────────────── */

/* Like the comic-book / cel-shaded look: snap the lighting to a few flat steps
 * instead of a smooth gradient, so you get hard bands. Each cube face is lit
 * evenly, so a whole face falls in one band and the line between a bright and a
 * dim face is razor sharp. The highlight is all-or-nothing too — a bright spot
 * appears only where the surface points nearly straight at the lamp-and-eye. */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float diff = fmaxf(0.f, v3_dot(N, L));
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > 0.94f) ? 0.7f : 0.f;

  Vec3 c = u->obj_color;
  out->color = v3(c.x * (banded + 0.12f) + spec, c.y * (banded + 0.12f) + spec,
                  c.z * (banded + 0.12f) + spec);
  out->discard = false;
}

/* §5.5 ── normals: show which way each face points, as colour ────────── */

/* A debugging view: paint each surface by the direction it faces, turned into a
 * colour. Each cube face points one way, so each shows as one solid colour
 * (right-facing reddish, up-facing greenish, toward-you bluish, and so on). The
 * colours swing around as the cube turns. Handy to check the facings are right. */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/* §5.6 ── fresnel: a glowing outline ─────────────────────────────────── */

/* Fakes the way real surfaces light up along their edges. A surface facing you
 * straight on stays dark; one turned nearly edge-on glows. So the glow gathers
 * exactly on the outline and the faces seen at a steep angle.
 *
 * On the cube that's a bright cool rim around a dark interior — a glowing
 * outline that needs nothing but each pixel's facing and view direction. Looks
 * great on the black terminal background. */
static void frag_fresnel(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  float facing = fmaxf(0.f, v3_dot(N, V));
  float rim = powf(1.f - facing, FRESNEL_POWER);

  Vec3 base = v3(FRESNEL_BASE[0], FRESNEL_BASE[1], FRESNEL_BASE[2]);
  out->color = v3(base.x + FRESNEL_RIM[0] * rim, base.y + FRESNEL_RIM[1] * rim,
                  base.z + FRESNEL_RIM[2] * rim);
  out->discard = false;
}

typedef enum { SH_PHONG = 0, SH_TOON, SH_NORMALS, SH_FRESNEL, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong", "toon", "normals", "fresnel"};

/* ── §6 mesh — building the cube out of triangles ──────────────────────── */

/* Vertex — one corner, in the cube's own coordinates (the model matrix places it
 * into the world later). Fields:
 *   pos    — where the corner is (the cube's corners sit at ±CUBE_S)
 *   normal — which way its face points. The cube keeps a SEPARATE copy of each
 *            corner per face so every face can face its own way — that's what
 *            makes the edges crisp. Sharing corners would blend the facings and
 *            round the cube off.
 *   u, v   — texture coords, filled in but unused here (no images). */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;

/* Triangle — one face, stored as three slot-numbers into its Mesh's corner list
 * (so shared corners live once and faces just point at them). Its corners are
 * listed counter-clockwise seen from outside; the "skip faces turned away" check
 * in §8 relies on that ordering. */
typedef struct {
  int v[3];
} Triangle;

/* Mesh — a whole shape: a list of corners plus the triangles connecting them.
 * Built once at startup (tessellate_cube), freed by mesh_free. For the cube:
 * 24 corners (6 faces × 4 of their own) and 12 triangles (each square face cut
 * into two).
 *   verts / nvert — the corner list and how many are filled in
 *   tris  / ntri  — the triangle list and how many are filled in */
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

/* Builds the cube: 6 faces, each a square cut into 2 triangles. Every face gets
 * its own 4 corners all facing the same way, so the faces stay flat and the
 * edges crisp (sharing corners would round it off — right for a ball, wrong for
 * a box). Corners are listed counter-clockwise seen from outside, which is what
 * lets the renderer tell front faces from back ones. 24 corners, 12 triangles,
 * built once. */
static Mesh tessellate_cube(void) {
  float s = CUBE_S;

  static const float face_nrm[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
  };

  /* the 4 corners of each face, listed counter-clockwise from outside; each is
   * at ±1 and gets scaled to the cube's size in the loop below */
  static const float face_vtx[6][4][3] = {
      /* +X */ {{1, -1, 1}, {1, 1, 1}, {1, 1, -1}, {1, -1, -1}},
      /* -X */ {{-1, -1, -1}, {-1, 1, -1}, {-1, 1, 1}, {-1, -1, 1}},
      /* +Y */ {{-1, 1, 1}, {1, 1, 1}, {1, 1, -1}, {-1, 1, -1}},
      /* -Y */ {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}},
      /* +Z */ {{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}},
      /* -Z */ {{1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}},
  };

  static const float face_uv[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

  Mesh m;
  m.verts = malloc(24 * sizeof(Vertex));
  m.tris = malloc(12 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int f = 0; f < 6; f++) {
    Vec3 n = v3(face_nrm[f][0], face_nrm[f][1], face_nrm[f][2]);
    int base = m.nvert;

    for (int i = 0; i < 4; i++) {
      Vec3 p = v3(face_vtx[f][i][0] * s, face_vtx[f][i][1] * s,
                  face_vtx[f][i][2] * s);
      m.verts[m.nvert++] = (Vertex){p, n, face_uv[i][0], face_uv[i][1]};
    }

    /* cut the square into two triangles sharing a diagonal */
    m.tris[m.ntri++] = (Triangle){{base + 0, base + 1, base + 2}};
    m.tris[m.ntri++] = (Triangle){{base + 0, base + 2, base + 3}};
  }

  return m;
}

/* ── §7 framebuffer — where we draw before showing it ──────────────────── */

/* Framebuffer — our own off-screen page, the size of the terminal, that we draw
 * into and then copy to the screen all at once. Two grids of cols×rows:
 *   cbuf — the character + colour for each cell
 *   zbuf — how near the closest thing drawn at each cell is, so a farther
 *          surface can't paint over a nearer one (the classic z-buffer, Catmull
 *          1974; starts at "infinitely far" and keeps the nearest)
 * Wiped each frame (fb_clear), filled by the pipeline, copied out by fb_blit. */
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

/* Copies our finished page to the actual terminal, skipping empty cells. */
static void fb_blit(const Framebuffer *fb) {
  for (int y = 0; y < fb->rows; y++) {
    for (int x = 0; x < fb->cols; x++) {
      Cell c = fb->cbuf[y * fb->cols + x];
      if (!c.ch)
        continue;
      attron(COLOR_PAIR(c.pair) | c.attr);
      mvaddch(y, x, (chtype)(unsigned char)c.ch);
      attroff(COLOR_PAIR(c.pair) | c.attr);
    }
  }
}

/* ── §8 pipeline — drawing the triangles ───────────────────────────────── */

/* §8.1 ── filling a triangle ──────────────────────────────────────────── */

/* For a pixel and a triangle, returns three weights (one per corner) saying how
 * much each corner pulls on that pixel. They tell us if the pixel is inside (all
 * three come out positive) and how to blend the corners' values there. A
 * degenerate (zero-area) triangle returns negatives so the caller skips it. */
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

/* Takes a corner the camera has already transformed and finishes it onto the
 * screen: divide by distance (so farther is smaller), then scale into cells.
 * Returns (screen x, screen y, depth); y is flipped because screen rows count
 * downward but up should be up. */
static inline Vec3 project_to_screen(Vec4 clip, int cols, int rows) {
  float w = clip.w;
  if (fabsf(w) < 1e-6f)
    w = 1e-6f;
  return v3((clip.x / w + 1.f) * 0.5f * (float)cols,
            (-clip.y / w + 1.f) * 0.5f * (float)rows, clip.z / w);
}

/* True if this triangle is turned away from us (the back of the cube), so we can
 * skip it. The trick: measure the triangle's signed area on screen; with our
 * corner ordering and the y-flip, faces toward us come out negative, so anything
 * zero-or-positive is facing away. */
static inline bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area >= 0.f;
}

/* §8.2 ── draw the whole mesh, triangle by triangle ───────────────────── */

/* The main drawing loop. For each triangle: run the per-corner step on its 3
 * corners, drop it if it's behind the camera or facing away, then for every
 * pixel it covers and is the nearest thing seen so far, blend the corners to
 * that pixel, run the per-pixel step to get a colour, and store the cell. */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               const ShaderProgram *sh, bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    VSOut vo[3];

    /* run the per-corner step on the triangle's 3 corners */
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

    /* skip it if all three corners are behind the camera */
    if (vo[0].clip_pos.w < CLIP_W_MIN && vo[1].clip_pos.w < CLIP_W_MIN &&
        vo[2].clip_pos.w < CLIP_W_MIN)
      continue;

    /* finish each corner onto the screen */
    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      Vec3 s = project_to_screen(vo[vi].clip_pos, cols, rows);
      sx[vi] = s.x;
      sy[vi] = s.y;
      sz[vi] = s.z;
    }

    /* skip faces turned away from us, if culling is on */
    if (cull_backface && is_back_facing(sx, sy))
      continue;

    /* the little box of cells the triangle could touch, clipped to the screen */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    /* walk those cells and fill the ones inside the triangle */
    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        /* how near this pixel is; skip it if something nearer is already here */
        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        int idx = py * cols + px;
        if (z >= fb->zbuf[idx])
          continue;
        fb->zbuf[idx] = z;

        /* blend the three corners to this exact pixel for the per-pixel step */
        FSIn fsin;
        fsin.world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos,
                                 vo[2].world_pos, b[0], b[1], b[2]);
        fsin.world_nrm = v3_norm(v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                         vo[2].world_nrm, b[0], b[1], b[2]));
        fsin.u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
        fsin.v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
        fsin.px = px;
        fsin.py = py;

        FSOut fsout = {v3(0, 0, 0), false};
        sh->frag(&fsin, &fsout, sh->frag_uni);
        if (fsout.discard)
          continue;

        fb->cbuf[idx] = paint_compute_cell(fsout.color, px, py);
      }
    }
  }
}

/* ── §9 scene — the cube, the camera, and the chosen look ──────────────── */

/* Scene — everything about WHAT is on screen and HOW the viewer is steering it.
 * The cube and how it's turned are the only things that change on their own each
 * frame; the knobs are what the keys change; the rest is rebuilt from those
 * every frame (the active look and the values it reads), kept here so the look's
 * pointers always have a steady address to point at. */
typedef struct {
  /* the object and how it's turned right now */
  Mesh mesh;
  float angle_x, angle_y; /* tumble angles, nudged along by scene_tick      */

  /* what the keys change */
  float cam_dist;      /* zoom distance, limited by the +/- keys         */
  ShaderIdx shade_idx; /* which look is showing (cycled by 's')          */
  bool cull_backface;  /* whether to hide the inside faces ('c')         */
  bool paused;         /* when true, the tumble freezes (space)          */

  /* rebuilt from the above every frame */
  ShaderProgram shader;  /* the active look (its two steps + value pointers) */
  Uniforms uni;          /* transforms (from the pose) + light + material    */
  ToonUniforms toon_uni; /* uni + band count, for the toon look              */
} Scene;

/* Points the active look's slot at the right pair of steps + values for the
 * shader the user picked. */
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
    s->shader = (ShaderProgram){vert_default, frag_normals, &s->uni, &s->uni};
    break;
  case SH_FRESNEL:
    s->shader = (ShaderProgram){vert_default, frag_fresnel, &s->uni, &s->uni};
    break;
  default:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_cube();
  s->shade_idx = SH_PHONG;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;

  s->uni.light_pos = v3(1.4f, 1.7f, 1.9f);    /* close in, so faces get a gradient */
  s->uni.light_col = v3(1.40f, 1.34f, 1.22f); /* bright warm, lifts lit faces     */
  s->uni.ambient = v3(0.10f, 0.10f, 0.13f);   /* dim cool fill so dark faces read */
  s->uni.shininess = 64.f;
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.obj_color = v3(0.9f, 0.55f, 0.15f); /* warm orange */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

  scene_build_shader(s);
}

/* Moves the camera in/out after a +/- key and refreshes its look-from transform,
 * so the new zoom shows up on the very next frame. */
static void scene_set_zoom(Scene *s) {
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
}

/* Rebuilds the perspective for a new window size so the cube isn't stretched. */
static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

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
  s->toon_uni.base = s->uni;
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

/* ── §10 screen — drawing the frame, the HUD, and showing it ───────────── */

/* Screen — the terminal window: how many cells wide and tall, and the home of
 * the ncurses setup / resize / show-it calls. */
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

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* Draws the two overlay lines: a yellow status across the top (fps, current
 * look, zoom, cull) and a cyan key reminder along the bottom. */
static void hud_draw(const Screen *s, const Scene *sc, double fps) {
  char buf[96];
  snprintf(buf, sizeof buf, " %5.1f fps  [%s]  z:%.1f  cull:%s%s ", fps,
           k_shader_names[sc->shade_idx], (double)sc->cam_dist,
           sc->cull_backface ? "on " : "off", sc->paused ? " PAUSED" : "");
  int len = (int)strlen(buf);
  if (len > s->cols)
    len = s->cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " CUBE-RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0, " q:quit  spc:pause  s:shader  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* Draws one frame: wipe the page, draw the cube through the current look, copy
 * it to the screen. Reads the scene, writes only the page — never the scene. */
static void scene_draw(const Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, s->cull_backface);
  fb_blit(fb);
}

/* ── §11 app — setup, the main loop, and keypresses ────────────────────── */

/* App — the whole running program: the scene, the screen, the drawing page, and
 * the loop's stop/resize flags. Not part of the 3-D world — it just bundles
 * everything so the signal handlers and main loop share one object (g_app). The
 * two flags are marked volatile sig_atomic_t because a signal can set them at
 * any moment, so the compiler must always read/write them for real. */
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
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 's':
  case 'S':
    scene_next_shader(s);
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
  case '_':
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

int main(void) {
  srand((unsigned int)clock_ns());
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

    /* how long since the last frame (capped, so a hiccup doesn't lurch the cube) */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* advance the cube's tumble */
    scene_tick(&app->scene, dt_sec);

    /* update the fps readout (averaged over half-second windows) */
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
    hud_draw(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* handle a keypress (pause / shader / cull / zoom / quit) */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* wait out the rest of the frame to hold a steady rate */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  endwin();
  return 0;
}
