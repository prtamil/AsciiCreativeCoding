/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_cubes.c — four coloured blobs ("metaballs") orbit inside a box; where
 * their combined glow is strong enough a smooth surface appears, and that surface
 * is rebuilt from scratch every frame. As blobs drift apart you see separate
 * shapes; as they pass close they merge, their colours blending in the seam.
 *
 * The trick (marching cubes): chop space into a grid of little cubes and, for
 * each cube, look up from a table which triangles approximate the surface passing
 * through it. The rest is an ordinary triangle renderer — the same path as
 * ssao_pipeline.c / deferred_rendering_pipeline.c.
 *
 * Keys: space pause · n theme · +/- zoom · t/g threshold (tighter/fatter) · r reset · q quit
 * Refs: marching cubes — Lorensen & Cline, SIGGRAPH '87; case tables — Paul
 *   Bourke, https://paulbourke.net/geometry/polygonise/
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/marching_cubes.c -o mc -lncurses -lm
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

/* §1.2 size of the per-pixel screen tables — fixed, big enough for a large
 * terminal; anything past the edge is skipped. */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 the camera — it slowly circles the box of blobs. */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 50.0f
/* A corner this close to the eye (or behind it) is off-screen; a triangle with
 * all three there is dropped before the divide-by-distance step blows up. */
#define NEAR_W_EPS 0.001f

#define CAM_DIST 3.4f
#define CAM_DIST_MIN 1.6f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f
#define CAM_EYE_Y 0.6f
#define CAM_LOOK_Y 0.0f
#define CAM_ORBIT_RAD_PER_SEC 0.18f

#define CELL_W 8
#define CELL_H 16

/* §1.4 the grid of little cubes the surface is carved from.
 *
 * GRID_DIM cubes along each axis (so one more sample point in each direction).
 * More = a smoother surface but slower; 24 looks good at terminal resolution.
 * The box runs from -WORLD_HALF to +WORLD_HALF on each axis. */
#define GRID_DIM 24
#define WORLD_HALF 1.50f

/* The cutoff that decides where the surface is: the surface forms where the
 * blobs' combined glow crosses this. Higher = tighter blobs; the t/g keys move
 * it between the min and max. */
#define MC_THRESHOLD_DEF 1.00f
#define MC_THRESHOLD_MIN 0.30f
#define MC_THRESHOLD_MAX 3.00f
#define MC_THRESHOLD_STEP 0.05f

/* How many triangle corners the mesh can hold (3 per triangle). Four blobs
 * never make a surface anywhere near this big. */
#define MC_MAX_VERTS 18000

/* §1.5 the blobs themselves. */
#define N_METABALLS 4
#define BALL_STRENGTH 0.18f /* how strongly each blob glows         */
#define BALL_EPSILON 0.05f  /* keeps the glow from spiking to infinity at the centre */

/* §1.6 lighting — the sun shines from one fixed direction (shared by all
 * themes); its colour, the fill light, the rim glow, and the blob colours all
 * come from the active theme. The sun points from in front so the side facing
 * the camera is the lit one. */
static const float SUN_DIR[3] = {-0.55f, -0.85f, -0.30f};
#define SHININESS 18.0f /* how tight the glossy highlight is (bigger = tighter) */
#define SPEC_GAIN 0.28f /* how strong the highlight is                          */
#define RIM_POWER 2.5f  /* lower = a wider edge glow                            */

/* §1.7 themes — a preset look: four blob colours plus the lighting mood. Two
 * knobs set the 3-D feel:
 *   - low ambient → strong light/shadow contrast (the back goes dark, the
 *     outline pops)
 *   - high rim_strength → a bright glow along the outline where the surface
 *     curves out of view, so each blob reads as round, not a flat disc
 * 'n' cycles through them, swapping the colours and the lighting live. Every
 * colour is kept bright enough to show on a black background, even the darkest. */
typedef struct {
  const char *name;                  /* HUD label                       */
  float ball_colors[N_METABALLS][3]; /* one RGB per blob                */
  float ambient[3];                  /* the everywhere fill light       */
  float sun_col[3];                  /* the sun's colour                */
  float rim_strength;                /* how bright the outline glow is  */
} Theme;

static const Theme THEMES[] = {
    /* PRIMARY — high-contrast RGB+yellow, the classic graphics-paper look. */
    {"PRIMARY",
     {{0.95f, 0.30f, 0.30f},
      {0.30f, 0.95f, 0.40f},
      {0.40f, 0.50f, 0.95f},
      {1.00f, 0.85f, 0.25f}},
     {0.20f, 0.22f, 0.28f},
     {0.95f, 0.85f, 0.65f},
     0.45f},

    /* LAVA — molten reds and oranges in a hot, dim ambient. */
    {"LAVA",
     {{0.95f, 0.30f, 0.10f},
      {1.00f, 0.65f, 0.15f},
      {0.85f, 0.20f, 0.05f},
      {1.00f, 0.85f, 0.20f}},
     {0.18f, 0.10f, 0.06f},
     {1.00f, 0.78f, 0.45f},
     0.65f},

    /* PLASMA — electric blue / purple / cyan, cool atmosphere. */
    {"PLASMA",
     {{0.30f, 0.50f, 1.00f},
      {0.60f, 0.30f, 0.95f},
      {0.20f, 0.85f, 0.95f},
      {0.85f, 0.30f, 0.85f}},
     {0.10f, 0.10f, 0.22f},
     {0.65f, 0.80f, 1.00f},
     0.75f},

    /* MATRIX — green-on-near-black, cyber feel, strong rim. */
    {"MATRIX",
     {{0.20f, 0.95f, 0.30f},
      {0.40f, 0.90f, 0.20f},
      {0.10f, 0.75f, 0.40f},
      {0.55f, 1.00f, 0.40f}},
     {0.08f, 0.18f, 0.10f},
     {0.55f, 0.95f, 0.55f},
     0.75f},

    /* OCEAN — blues and teals with a cool sun. */
    {"OCEAN",
     {{0.20f, 0.55f, 0.95f},
      {0.30f, 0.75f, 0.95f},
      {0.40f, 0.90f, 0.80f},
      {0.20f, 0.45f, 0.95f}},
     {0.10f, 0.20f, 0.30f},
     {0.70f, 0.85f, 1.00f},
     0.55f},

    /* SUNSET — warm pinks / oranges / purples on a violet ambient. */
    {"SUNSET",
     {{0.95f, 0.50f, 0.40f},
      {1.00f, 0.70f, 0.30f},
      {0.85f, 0.30f, 0.55f},
      {0.70f, 0.30f, 0.75f}},
     {0.28f, 0.16f, 0.26f},
     {1.00f, 0.75f, 0.55f},
     0.55f},

    /* NEON — saturated magenta / cyan / yellow, near-black ambient. */
    {"NEON",
     {{1.00f, 0.20f, 0.85f},
      {0.20f, 1.00f, 0.85f},
      {0.85f, 1.00f, 0.20f},
      {0.20f, 0.50f, 1.00f}},
     {0.08f, 0.08f, 0.16f},
     {1.00f, 1.00f, 1.00f},
     0.85f},
};
#define N_THEMES ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

/* §1.8 the brightness-to-character ladder, dim → bright (Paul Bourke's). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.9 a fixed 4×4 nudge pattern added to each cell's brightness so neighbours
 * of similar brightness pick different characters, hiding the steps in a smooth
 * gradient. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.10f /* how strong that nudge is */

/* §1.10 ncurses colour-pair numbers: 216 for the RGB cube, plus a yellow for
 * the status bar and a cyan for the hint line. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

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
static inline Vec3 v3_lerp(Vec3 a, Vec3 b, float t) {
  return v3_add(a, v3_scale(v3_sub(b, a), t));
}
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

/* Builds the perspective transform — the one that makes far things smaller. */
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

/* Builds the "stand at eye, look toward at" camera transform. */
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

/* ── §4 paint — turning a colour into one terminal cell ────────────────── */

static int g_256;

/* Sets up our colours: 216 of them as a 6×6×6 cube of reds × greens × blues,
 * plus a yellow for the status bar and a cyan for the hint line. Falls back to
 * plain white if the terminal can't do 256 colours. */
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
/* reinhard — roll an over-bright value gently toward white instead of clipping. */
static inline float reinhard(float x) { return x / (1.f + x); }
/* gamma_enc — brightness correction so the colour looks right on screen. */
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* Draws one terminal cell from a colour: bring it into screen range, pick the
 * closest of our 216 colours, and pick a character for its brightness (with a
 * dither nudge so gradients stay smooth); brightest cells go bold, darkest dim. */
static void paint_cell(int sx, int sy, Vec3 col) {
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  float dith = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith);

  int pair;
  if (g_256) {
    int r5 = (int)(r * 5.f + 0.5f);
    if (r5 > 5)
      r5 = 5;
    if (r5 < 0)
      r5 = 0;
    int g5 = (int)(g * 5.f + 0.5f);
    if (g5 > 5)
      g5 = 5;
    if (g5 < 0)
      g5 = 0;
    int b5 = (int)(b * 5.f + 0.5f);
    if (b5 > 5)
      b5 = 5;
    if (b5 < 0)
      b5 = 0;
    pair = PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
  } else {
    pair = PAIR_CUBE_BASE;
  }

  int idx = (int)(lum_d * (BOURKE_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= BOURKE_LEN)
    idx = BOURKE_LEN - 1;

  int attr = (luma > 0.85f) ? A_BOLD : (luma < 0.15f) ? A_DIM : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[idx]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 metaballs — the glow, the facing, and the colour at a point ───── */

/* Metaball — one glowing blob. Each frame it slides around a horizontal circle;
 * the "field" functions below read its position to work out the glow, surface
 * facing, and colour at any point in space.
 *   pos        — where it is right now (moved by scene_tick)
 *   color      — its colour from the theme
 *   orbit_r    — how wide its circle is
 *   orbit_speed— how fast it goes around (and which way; negative = reversed)
 *   orbit_y    — how high its circle sits
 *   phase      — where it currently is on the circle */
typedef struct {
  Vec3 pos;
  Vec3 color;
  float orbit_r;
  float orbit_speed;
  float orbit_y;
  float phase;
} Metaball;

/* The total glow at a point: each blob adds glow that's strongest up close and
 * fades with distance, and they simply add up — which is what lets two blobs
 * merge into one surface where their glows overlap. */
static float metaball_field(Vec3 p, const Metaball *balls, int n) {
  float sum = 0.f;
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    sum += BALL_STRENGTH / d2;
  }
  return sum;
}

/* Which way the surface faces at a point — its outward direction. The glow
 * drops off as you move away from the blobs, so "downhill" points inward;
 * the outward facing is just the other way (away from the blobs). */
static Vec3 metaball_outward_normal(Vec3 p, const Metaball *balls, int n) {
  Vec3 g = v3(0, 0, 0);
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    float k = BALL_STRENGTH / (d2 * d2);
    g = v3_add(g, v3_scale(d, k));
  }
  return v3_norm(g);
}

/* The colour at a point: a blend of the blob colours, each weighted by how much
 * it glows here — so the closest blob dominates, and the seam where two meet
 * fades smoothly from one colour to the other. */
static Vec3 metaball_color(Vec3 p, const Metaball *balls, int n) {
  Vec3 c = v3(0, 0, 0);
  float total_w = 0.f;
  for (int i = 0; i < n; i++) {
    Vec3 d = v3_sub(p, balls[i].pos);
    float d2 = v3_dot(d, d) + BALL_EPSILON;
    float w = BALL_STRENGTH / d2;
    c = v3_add(c, v3_scale(balls[i].color, w));
    total_w += w;
  }
  return total_w > 1e-6f ? v3_scale(c, 1.f / total_w) : v3(1, 1, 1);
}

/* ── §6 marching cubes — building the surface mesh each frame ──────────── *
 * The heart of the demo: turn the blobs' glow into actual triangles. This is
 * where the surface mesh (g_mesh) gets rebuilt from scratch every frame. */

/* §6.1 ── how a cube's 8 corners and 12 edges are numbered ──────────── *
 *
 * The numbering MUST match the lookup tables below (it's Paul Bourke's standard
 * layout). Corner offsets are within a unit cube; the 12 edges connect them:
 *
 *           7────────6              corners 0..3 = bottom face (y = 0)
 *          /│       /│              corners 4..7 = top    face (y = 1)
 *         4─┼──────5 │
 *         │ 3──────┼─2              edges 0..3  = bottom  ring
 *         │/       │/               edges 4..7  = top     ring
 *         0────────1                edges 8..11 = vertical pillars
 */
static const int k_corner[8][3] = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
    {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
};
static const int k_edge[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, /* bottom ring */
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, /* top    ring */
    {0, 4}, {1, 5}, {2, 6}, {3, 7}, /* vertical pillars */
};

/* §6.2 ── which edges the surface crosses, per case ────────────────── *
 *
 * A cube has 8 corners, each either inside or outside the surface — 256
 * combinations. For each one, this is a 12-bit mask: bit e is set when edge e
 * has one end inside and one end outside, i.e. the surface crosses it and we'll
 * need a vertex there. (Paul Bourke's reference table.) */
static const unsigned short EDGE_TABLE[256] = {
    0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f,
    0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f,
    0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 0x230,
    0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936,
    0xe3a, 0xf33, 0xc39, 0xd30, 0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5,
    0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 0x460, 0x569,
    0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a,
    0x963, 0xa69, 0xb60, 0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 0x650, 0x759, 0x453,
    0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53,
    0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc,
    0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 0x8c0, 0x9c9, 0xac3, 0xbca,
    0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9,
    0x7c0, 0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055,
    0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6,
    0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f,
    0x066, 0x76a, 0x663, 0x569, 0x460, 0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af,
    0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0, 0xd30,
    0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636,
    0x13a, 0x033, 0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895,
    0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190, 0xf00, 0xe09,
    0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a,
    0x203, 0x109, 0x000,
};

/* §6.3 ── how to connect those crossings into triangles, per case ──── *
 *
 * For each of the 256 cases, the edges to join into triangles: read in groups
 * of 3 (each number is an edge from §6.1), with -1 marking the end. Up to 5
 * triangles per cube. The corner ordering makes the triangles face outward.
 * (Paul Bourke's reference table.) */
static const int TRI_TABLE[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
};

/* A startup sanity check that the two big tables above agree (the triangles a
 * case uses should reference exactly the edges it marks as crossed) — catches a
 * typo in the tables before it silently corrupts the surface. Returns -1 if all
 * 256 cases are fine, or the first bad case number. */
static int mc_verify_tables(void) {
  for (int c = 0; c < 256; c++) {
    unsigned derived = 0;
    for (int i = 0; i < 16 && TRI_TABLE[c][i] != -1; i++)
      derived |= (1u << TRI_TABLE[c][i]);
    if (derived != EDGE_TABLE[c])
      return c;
  }
  return -1;
}

/* §6.4 ── the mesh the surface gets built into ──────────────────────── */

/* MCVertex — one corner of one triangle, carrying everything the renderer needs
 * to blend across a face: its position, which way the surface faces there, and
 * its colour. */
typedef struct {
  Vec3 pos;
  Vec3 normal;
  Vec3 color;
} MCVertex;

/* Mesh — the surface, rebuilt from scratch each frame: a flat list of corners (3
 * per triangle, no shared corners) plus how many are filled in. The renderer
 * reads it as count/3 triangles. The flat list wastes a little memory but keeps
 * the format dead simple — and the count belongs with the list it counts. */
typedef struct {
  MCVertex verts[MC_MAX_VERTS]; /* the list; verts[0 .. count) are filled in */
  int count;                    /* how many corners (= 3 × triangles)        */
} Mesh;

static Mesh g_mesh;

/* §6.5 ── extracting the surface ────────────────────────────────────── */

/* g_field — scratch: the blobs' total glow sampled at every grid corner. We fill
 * it once per frame (here) and reuse it across cubes, since each corner is shared
 * by up to 8 neighbouring cubes — computing it once instead of eight times. */
static float g_field[GRID_DIM + 1][GRID_DIM + 1][GRID_DIM + 1];

/* Turns a grid corner index (0..GRID_DIM) into its world coordinate. */
static inline float grid_to_world(int i) {
  const float step = 2.f * WORLD_HALF / (float)GRID_DIM;
  return -WORLD_HALF + (float)i * step;
}

/* Fill the whole field grid by sampling the blobs' glow at every corner. */
static void mc_compute_field(const Metaball *balls, int n) {
  for (int k = 0; k <= GRID_DIM; k++) {
    float wz = grid_to_world(k);
    for (int j = 0; j <= GRID_DIM; j++) {
      float wy = grid_to_world(j);
      for (int i = 0; i <= GRID_DIM; i++) {
        float wx = grid_to_world(i);
        g_field[k][j][i] = metaball_field(v3(wx, wy, wz), balls, n);
      }
    }
  }
}

/* Add one corner to the mesh, working out its facing and colour from the blobs.
 * Returns false if the mesh list is full. */
static bool mc_push_vertex(Mesh *mesh, Vec3 pos, const Metaball *balls, int n) {
  if (mesh->count >= MC_MAX_VERTS)
    return false;
  MCVertex *v = &mesh->verts[mesh->count++];
  v->pos = pos;
  v->normal = metaball_outward_normal(pos, balls, n);
  v->color = metaball_color(pos, balls, n);
  return true;
}

/* Find the point along an edge where the glow exactly hits the threshold — the
 * surface crosses there. Slides between the two ends based on how far each end's
 * glow is from the threshold (clamped to the edge against rounding noise). */
static Vec3 mc_edge_lerp(Vec3 pa, float fa, Vec3 pb, float fb,
                         float threshold) {
  float denom = fb - fa;
  float t = (fabsf(denom) > 1e-6f) ? (threshold - fa) / denom : 0.5f;
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  return v3_lerp(pa, pb, t);
}

/* Read one cube's 8 corner field values (fv) and world positions (pv). */
static void mc_sample_cube(int i, int j, int k, float fv[8], Vec3 pv[8]) {
  for (int c = 0; c < 8; c++) {
    int ci = i + k_corner[c][0];
    int cj = j + k_corner[c][1];
    int ck = k + k_corner[c][2];
    fv[c] = g_field[ck][cj][ci];
    pv[c] = v3(grid_to_world(ci), grid_to_world(cj), grid_to_world(ck));
  }
}

/* Classify the 8 corners inside/outside into the 8-bit case index: bit c is set
 * when corner c is INSIDE the surface (its field exceeds the threshold). */
static int mc_cube_case(const float fv[8], float threshold) {
  int cube_case = 0;
  for (int c = 0; c < 8; c++)
    if (fv[c] > threshold)
      cube_case |= (1 << c);
  return cube_case;
}

/* Put a vertex on each edge the surface crosses (the ones EDGE_TABLE marks),
 * each at the exact spot along the edge where the glow hits the threshold. */
static void mc_interpolate_edges(const float fv[8], const Vec3 pv[8],
                                 unsigned emask, float threshold,
                                 Vec3 edge_pos[12]) {
  for (int e = 0; e < 12; e++) {
    if (!(emask & (1u << e)))
      continue;
    int a = k_edge[e][0];
    int b = k_edge[e][1];
    edge_pos[e] = mc_edge_lerp(pv[a], fv[a], pv[b], fv[b], threshold);
  }
}

/* Append the triangles TRI_TABLE lists for this case (groups of 3 edge
 * vertices). Returns false if the mesh pool filled up mid-cube. */
static bool mc_emit_triangles(Mesh *mesh, const int *tri,
                              const Vec3 edge_pos[12], const Metaball *balls,
                              int n) {
  for (int t = 0; tri[t] != -1; t += 3) {
    if (!mc_push_vertex(mesh, edge_pos[tri[t]], balls, n))
      return false;
    if (!mc_push_vertex(mesh, edge_pos[tri[t + 1]], balls, n))
      return false;
    if (!mc_push_vertex(mesh, edge_pos[tri[t + 2]], balls, n))
      return false;
  }
  return true;
}

/*
 * Rebuild the whole surface mesh. Sample the glow once at every grid corner,
 * then for each little cube: decide which corners are inside, look up which
 * edges the surface crosses, put a vertex on each, and add that cube's triangles.
 */
static void mc_extract(Mesh *mesh, const Metaball *balls, int n,
                       float threshold) {
  mesh->count = 0;
  mc_compute_field(balls, n);

  for (int k = 0; k < GRID_DIM; k++) {
    for (int j = 0; j < GRID_DIM; j++) {
      for (int i = 0; i < GRID_DIM; i++) {
        float fv[8];
        Vec3 pv[8];
        mc_sample_cube(i, j, k, fv, pv);

        int cube_case = mc_cube_case(fv, threshold);
        unsigned emask = EDGE_TABLE[cube_case];
        if (emask == 0)
          continue; /* cube entirely inside or outside — no surface crosses it */

        Vec3 edge_pos[12];
        mc_interpolate_edges(fv, pv, emask, threshold, edge_pos);

        if (!mc_emit_triangles(mesh, TRI_TABLE[cube_case], edge_pos, balls, n))
          return; /* mesh pool full */
      }
    }
  }
}

/* ── §7 G-buffer — drawing the surface into per-cell tables ────────────── */

/* GBuffer — a set of full-screen tables describing the surface seen at each
 * cell. We draw the triangles once, recording each cell's facts here, then light
 * each cell from those facts (the classic two-step "deferred" approach — Saito &
 * Takahashi, 1987). Six tables, all read as [row][col]:
 *   pos / normal / albedo — where the surface is, which way it faces, its colour
 *   zbuf                  — how near it is, so a farther surface can't paint over
 *                           a nearer one (starts at 1.0 = farthest, smaller wins)
 *   valid                 — 1 where a triangle was drawn, 0 for empty background
 *   light                 — the final lit colour (filled by render_lightpass) */
typedef struct {
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];
  float zbuf[GBUF_MAX_H][GBUF_MAX_W];
  uint8_t valid[GBUF_MAX_H][GBUF_MAX_W];
  Vec3 light[GBUF_MAX_H][GBUF_MAX_W];
} GBuffer;

static GBuffer g_gbuf;

/* Wipe the tables for a new frame: every cell empty and "farthest". */
static void gbuffer_clear(GBuffer *gb, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      gb->zbuf[r][c] = 1.0f;
      gb->valid[r][c] = 0;
    }
  }
}

/* For a pixel and a triangle, returns three weights (one per corner) saying how
 * much each corner pulls on that pixel — they tell us if the pixel is inside
 * (all three ≥ 0) and how to blend the corners' values there. A degenerate
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

/* Drop the 3 corners onto the cell grid: divide by distance (far things shrink),
 * scale into cells, and flip y because screen rows count downward but up should
 * be up. Returns each corner's screen x/y and a depth value. */
static void project_to_screen(const Vec4 clip[3], int cols, int rows,
                              float sx[3], float sy[3], float sz[3]) {
  for (int vi = 0; vi < 3; vi++) {
    float w = clip[vi].w;
    if (fabsf(w) < 1e-6f)
      w = 1e-6f;
    sx[vi] = (clip[vi].x / w + 1.f) * 0.5f * (float)cols;
    sy[vi] = (-clip[vi].y / w + 1.f) * 0.5f * (float)rows; /* flip y */
    sz[vi] = clip[vi].z / w;
  }
}

/* True when all three corners are behind the camera — the whole triangle is
 * off-screen, so skip it. */
static bool all_behind_near_plane(const Vec4 clip[3]) {
  return clip[0].w < NEAR_W_EPS && clip[1].w < NEAR_W_EPS &&
         clip[2].w < NEAR_W_EPS;
}

/* True if the triangle faces away from us (so we can skip it). The trick:
 * measure its signed area on screen; with our corner order and the y-flip,
 * faces toward us come out positive, so zero-or-less is facing away. */
static bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area <= 0.f;
}

/* Fill the cells one triangle covers: walk its bounding box, keep the cells
 * inside it and nearer than whatever's there, and record the blended surface
 * sample — position, facing, and the per-corner colour that smooths the seam
 * where two blobs merge. */
static void rasterize_fragments(GBuffer *gb, const MCVertex *v0,
                                const MCVertex *v1, const MCVertex *v2,
                                const float sx[3], const float sy[3],
                                const float sz[3], int cols, int rows) {
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
      gb->pos[py][px] = v3_bary(v0->pos, v1->pos, v2->pos, b[0], b[1], b[2]);
      gb->normal[py][px] = v3_norm(
          v3_bary(v0->normal, v1->normal, v2->normal, b[0], b[1], b[2]));
      gb->albedo[py][px] =
          v3_bary(v0->color, v1->color, v2->color, b[0], b[1], b[2]);
      gb->valid[py][px] = 1;
    }
  }
}

/* Draw the whole mesh into the tables, one triangle at a time: move its corners
 * onto the screen, drop it if it's off-screen or facing away, then fill the
 * cells it covers. (The mesh is already in world coordinates, so there's no
 * separate "place the object" step.) */
static void rasterize_mc_pool(GBuffer *gb, const Mesh *mesh, Mat4 mvp, int cols,
                              int rows) {
  int n_tri = mesh->count / 3;
  for (int ti = 0; ti < n_tri; ti++) {
    const MCVertex *v0 = &mesh->verts[ti * 3 + 0];
    const MCVertex *v1 = &mesh->verts[ti * 3 + 1];
    const MCVertex *v2 = &mesh->verts[ti * 3 + 2];

    Vec4 clip[3] = {
        m4_mul_v4(mvp, v4(v0->pos.x, v0->pos.y, v0->pos.z, 1.f)),
        m4_mul_v4(mvp, v4(v1->pos.x, v1->pos.y, v1->pos.z, 1.f)),
        m4_mul_v4(mvp, v4(v2->pos.x, v2->pos.y, v2->pos.z, 1.f)),
    };
    if (all_behind_near_plane(clip))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(clip, cols, rows, sx, sy, sz);
    if (is_back_facing(sx, sy))
      continue;

    rasterize_fragments(gb, v0, v1, v2, sx, sy, sz, cols, rows);
  }
}

static void render_gbuffer(GBuffer *gb, const Mesh *mesh, Mat4 view, Mat4 proj,
                           int cols, int rows) {
  gbuffer_clear(gb, cols, rows);
  Mat4 mvp = m4_mul(proj, view); /* the mesh is already placed in the world */
  rasterize_mc_pool(gb, mesh, mvp, cols, rows);
}

/* ── §8 lightpass — colour every surface cell from the light ──────────── *
 * Reads the surface tables, writes the lit colour into the light table. Two
 * choices keep the shape readable on the coarse terminal grid:
 *   - the light wraps a bit past the edge of shadow (half-Lambert), so the side
 *     facing away from the sun keeps a soft gradient instead of going flat —
 *     otherwise that patch reads as a featureless blob;
 *   - a bright glow along the outline, so each blob reads as round even where it
 *     curves out of the light. */

/* The lit colour of one surface cell: a little fill light everywhere, plus more
 * where it faces the sun (wrapped so the dark side keeps a gradient), plus a
 * glossy highlight, plus the outline glow — clamped to what the screen can show.
 * Pure: the per-frame light setup (sun direction, colour, fill, glow strength)
 * is passed in, worked out once by render_lightpass. */
static Vec3 shade_surface(Vec3 P, Vec3 N, Vec3 albedo, Vec3 L, Vec3 sun_col,
                          Vec3 ambient, float rim_str, Vec3 cam_pos) {
  Vec3 amb =
      v3(ambient.x * albedo.x, ambient.y * albedo.y, ambient.z * albedo.z);

  /* wrap the light past the edge of shadow so the dark side keeps a gradient */
  float wrap = 0.5f * v3_dot(N, L) + 0.5f;
  float diff = wrap * wrap;
  Vec3 dif = v3(albedo.x * sun_col.x * diff, albedo.y * sun_col.y * diff,
                albedo.z * sun_col.z * diff);

  Vec3 V = v3_norm(v3_sub(cam_pos, P));
  Vec3 H = v3_norm(v3_add(L, V));
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS) * SPEC_GAIN;
  Vec3 sp = v3(sun_col.x * spec, sun_col.y * spec, sun_col.z * spec);

  float facing = fmaxf(0.f, v3_dot(N, V));
  float rim = powf(1.f - facing, RIM_POWER) * rim_str;
  Vec3 rm = v3(sun_col.x * rim, sun_col.y * rim, sun_col.z * rim);

  Vec3 sum = v3_add(v3_add(v3_add(amb, dif), sp), rm);
  return v3(fminf(1.f, sum.x), fminf(1.f, sum.y), fminf(1.f, sum.z));
}

static void render_lightpass(GBuffer *gb, Vec3 cam_pos, const Theme *theme,
                             int cols, int rows) {
  /* per-frame light environment, constant across the whole pass */
  Vec3 sun_col = v3(theme->sun_col[0], theme->sun_col[1], theme->sun_col[2]);
  Vec3 ambient = v3(theme->ambient[0], theme->ambient[1], theme->ambient[2]);
  float rim_str = theme->rim_strength;
  Vec3 L = v3_norm(v3_neg(v3(SUN_DIR[0], SUN_DIR[1], SUN_DIR[2])));

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        gb->light[r][c] = v3(0, 0, 0); /* background — nothing to light */
        continue;
      }
      gb->light[r][c] =
          shade_surface(gb->pos[r][c], gb->normal[r][c], gb->albedo[r][c], L,
                        sun_col, ambient, rim_str, cam_pos);
    }
  }
}

/* ── §9 scene — the blobs, the camera, and the chosen settings ─────────── */

/* Camera — the eye circling the scene, plus the transforms worked out from it.
 * Only yaw and dist are "set" (the orbit nudges yaw each frame; +/- change dist);
 * pos, view, and proj are recomputed from those by camera_rebuild_view / _proj.
 *   yaw  — how far around the scene the eye has swung
 *   dist — how far back the eye is (the zoom)
 *   pos  — the eye's actual spot, from yaw + dist
 *   view — the "look from the eye toward the centre" transform
 *   proj — the perspective (makes far things smaller), matched to the window */
typedef struct {
  float yaw;
  float dist;
  Vec3 pos;
  Mat4 view;
  Mat4 proj;
} Camera;

/* Scene — everything the demo is about, in one place:
 *   WHAT  — the four orbiting blobs
 *   HOW   — the knobs the keys change (surface threshold, theme, pause)
 *   WHERE — the camera
 * plus how big the drawing area is. Written only by scene_init / scene_tick and
 * the key handler; the drawing passes read it but never change it. */
typedef struct {
  /* what's simulated */
  Metaball balls[N_METABALLS];

  /* what the keys change */
  float threshold; /* where the surface forms — higher = tighter blobs (t / g) */
  int theme_idx;   /* which colour theme (n)                                   */
  bool paused;     /* freeze the blobs + camera (space)                        */

  /* where we view from */
  Camera cam;

  /* drawing area in cells (full width; height minus the HUD band) */
  int scene_cols, scene_rows;
} Scene;

/* Copies the active theme's colours onto the blobs. (The lighting colours aren't
 * copied anywhere — the light pass reads the active theme directly each frame.) */
static void theme_apply(Scene *s) {
  const Theme *t = &THEMES[s->theme_idx];
  for (int i = 0; i < N_METABALLS; i++) {
    s->balls[i].color =
        v3(t->ball_colors[i][0], t->ball_colors[i][1], t->ball_colors[i][2]);
  }
}

/* Rebuilds the perspective for a new window size so the scene isn't stretched
 * (terminal cells are taller than they are wide, which it accounts for). */
static void camera_rebuild_proj(Camera *cam, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  cam->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/* Places the eye on its circle (from yaw + dist) and refreshes the look-from
 * transform. */
static void camera_rebuild_view(Camera *cam) {
  float r = cam->dist;
  cam->pos = v3(sinf(cam->yaw) * r, CAM_EYE_Y, cosf(cam->yaw) * r);
  cam->view = m4_lookat(cam->pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* Sets up the scene: four blobs on circles whose speeds don't line up, so the
 * arrangement keeps drifting and never settles into a repeating pattern. */
static void scene_init(Scene *s, int total_cols, int total_rows) {
  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->threshold = MC_THRESHOLD_DEF;
  s->cam.dist = CAM_DIST;
  s->cam.yaw = 0.f;
  s->theme_idx = 0;

  static const struct {
    float r, speed, y, phase;
  } k_orbit[N_METABALLS] = {
      {0.55f, 0.40f, 0.30f, 0.f},
      {0.65f, 0.55f, -0.25f, (float)M_PI * 0.5f},
      {0.50f, -0.65f, 0.05f, (float)M_PI}, /* reversed */
      {0.70f, 0.85f, -0.10f, (float)M_PI * 1.5f},
  };
  for (int i = 0; i < N_METABALLS; i++) {
    Metaball *b = &s->balls[i];
    b->orbit_r = k_orbit[i].r;
    b->orbit_speed = k_orbit[i].speed;
    b->orbit_y = k_orbit[i].y;
    b->phase = k_orbit[i].phase;
    b->pos = v3(b->orbit_r * cosf(b->phase), b->orbit_y,
                b->orbit_r * sinf(b->phase));
  }

  theme_apply(s);
  camera_rebuild_proj(&s->cam, total_cols, s->scene_rows);
  camera_rebuild_view(&s->cam);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  for (int i = 0; i < N_METABALLS; i++) {
    Metaball *b = &s->balls[i];
    b->phase += b->orbit_speed * dt;
    b->pos = v3(b->orbit_r * cosf(b->phase), b->orbit_y,
                b->orbit_r * sinf(b->phase));
  }

  s->cam.yaw += CAM_ORBIT_RAD_PER_SEC * dt;
  camera_rebuild_view(&s->cam);
}

/* ── §10 screen — painting the cells and the HUD ───────────────────────── */

/* Paints every surface cell to the terminal from the light table. */
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

static void hud_draw(const Scene *s, const Mesh *mesh, double fps) {
  int hr = s->scene_rows;
  int cols = s->scene_cols;

  int n_tri = mesh->count / 3;

  /* top row — title on the left, live status on the right */
  char status[160];
  snprintf(
      status, sizeof status,
      " %5.1f fps  theme:%s  grid:%dx%dx%d  T=%.2f  tris:%d  zoom:%.1f  %s ",
      fps, THEMES[s->theme_idx].name, GRID_DIM, GRID_DIM, GRID_DIM,
      (double)s->threshold, n_tri, (double)s->cam.dist,
      s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " MARCHING CUBES · METABALLS ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(
      hr + 0, 1,
      "field f(p) = %d * %.2f / (|p-c|^2 + %.2f)   surface = level set f = T",
      N_METABALLS, (double)BALL_STRENGTH, (double)BALL_EPSILON);
  mvprintw(hr + 1, 1,
           "Per cube: 8 corner reads -> 8-bit case -> EDGE_TABLE -> "
           "TRI_TABLE -> emit triangles.");
  mvprintw(hr + 2, 1,
           "Watch the seam where blobs merge: per-vertex colour blend "
           "shows the influence mix.");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  n:theme  t/g:threshold  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §11 app — setup, the main loop, and keypresses ────────────────────── */

/* App — the whole running program: the scene plus the terminal size and two
 * flags the signal handlers set. One shared instance (g_app) so the handlers,
 * which take no arguments, can reach it. The two flags are marked volatile
 * sig_atomic_t because a signal can set them at any moment, so the compiler must
 * always read/write them for real. */
typedef struct {
  Scene scene;
  int total_cols;
  int total_rows;
  volatile sig_atomic_t running;     /* cleared to stop the loop        */
  volatile sig_atomic_t need_resize; /* set when the window was resized */
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
  case 't':
  case 'T':
    s->threshold += MC_THRESHOLD_STEP;
    if (s->threshold > MC_THRESHOLD_MAX)
      s->threshold = MC_THRESHOLD_MAX;
    break;
  case 'g':
  case 'G':
    s->threshold -= MC_THRESHOLD_STEP;
    if (s->threshold < MC_THRESHOLD_MIN)
      s->threshold = MC_THRESHOLD_MIN;
    break;
  case 'n':
  case 'N':
    s->theme_idx = (s->theme_idx + 1) % N_THEMES;
    theme_apply(s);
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

/* The set of edges TRI_TABLE actually uses for a case — so the startup error
 * message can show what it found, if the tables disagree. */
static unsigned tri_table_derived_edges(int cube_case) {
  unsigned d = 0;
  for (int i = 0; i < 16 && TRI_TABLE[cube_case][i] != -1; i++)
    d |= (1u << TRI_TABLE[cube_case][i]);
  return d;
}

int main(void) {
  /* bail at startup if the two big tables disagree — a typo there would
   * silently produce a broken surface */
  int bad = mc_verify_tables();
  if (bad >= 0) {
    fprintf(stderr,
            "MC table mismatch at case %d: edge=0x%03x derived=0x%03x\n"
            "Fix TRI_TABLE[%d] or EDGE_TABLE[%d] (Bourke reference).\n",
            bad, EDGE_TABLE[bad], tri_table_derived_edges(bad), bad, bad);
    return 1;
  }

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
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* move the blobs and the camera along */
    scene_tick(&app->scene, dt_sec);

    /* update the fps readout */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* wait out the rest of the frame to hold a steady rate */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);

    /* draw the frame: rebuild the surface → fill the tables → light them →
     * paint → HUD → show it */
    Scene *s = &app->scene;
    erase();
    mc_extract(&g_mesh, s->balls, N_METABALLS, s->threshold);
    render_gbuffer(&g_gbuf, &g_mesh, s->cam.view, s->cam.proj, s->scene_cols,
                   s->scene_rows);
    render_lightpass(&g_gbuf, s->cam.pos, &THEMES[s->theme_idx], s->scene_cols,
                     s->scene_rows);
    render_scene(s, &g_gbuf);
    hud_draw(s, &g_mesh, fps_display);
    screen_present();

    /* handle a keypress (pause / reset / theme / threshold / zoom / quit) */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  endwin();
  return 0;
}
