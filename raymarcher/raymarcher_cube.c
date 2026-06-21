/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher_cube.c — a spinning, lit cube drawn with text in the terminal.
 * Every character cell shoots one ray into the scene and shades whatever it
 * touches.  Same skeleton as its sibling raymarcher.c (a sphere) — read that
 * one first; only the shape's distance function (§5) changes here.
 *
 * Ideas borrowed: Hart's "Sphere Tracing" (1996) for the marching, Phong
 * (1975) for the lighting, and Iñigo Quílez's box distance function at
 * https://iquilezles.org/articles/distfunctions/
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── *
 *
 * Every tunable lives here.  No magic numbers anywhere else in the
 * file — if a literal carries meaning, it gets a name in §1.
 */

/* §1.1 frame rate. */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  FPS_UPDATE_MS = 500,
};

/* §1.2 one canvas pixel = one terminal cell.  (The squash that keeps the cube
 * from looking stretched — since cells are about twice as tall as wide — lives
 * in the ray aim, §11.) */
#define CELL_W 1
#define CELL_H 1
#define CELL_ASPECT 2.0f /* a terminal cell is about 2× taller than it is wide */

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* §1.3 ray-march tuning. */
#define RM_MAX_STEPS 80   /* give up after this many steps along one ray  */
#define RM_HIT_EPS 0.003f /* this close counts as "touched the surface"   */
#define RM_MAX_DIST 20.0f /* if a ray gets this far out, it hit nothing   */
#define RM_T_START 0.5f   /* start a bit down the ray so it can't "hit" at the eye */

/* §1.4 how far apart the sample points sit when we work out which way a
 * surface faces (§8). */
#define RM_NORM_EPS 0.001f

/* §1.5 camera (zoom). */
#define CAM_Z_DEFAULT 4.5f
#define CAM_Z_MIN 2.6f /* nearest zoom — keeps the eye outside the biggest cube */
#define CAM_Z_MAX 12.0f
#define CAM_ZOOM_STEP 0.30f
#define FOV_HALF_TAN 0.65f /* how wide the lens sees; bigger = wider (~66°) */

/* §1.6 cube size (half-extent). */
#define CUBE_H_DEFAULT 0.9f
#define SIZE_STEP 1.15f
#define SIZE_MIN 0.15f
#define SIZE_MAXX 2.5f
/* √3: a cube's centre-to-corner distance per unit half-extent.  Tells the
 * DEPTH view how near and how far the cube can possibly be (§15). */
#define CUBE_CORNER_DIST 1.732f

/* §1.7 rotation speeds (radians / second). */
#define ROT_X_DEFAULT 0.7f
#define ROT_Y_DEFAULT 1.1f
#define ROT_STEP 1.3f
#define ROT_MIN 0.02f
#define ROT_MAX 10.0f

/* §1.8 light orbit speed (radians/sec). */
#define LIGHT_SPD_DEFAULT 0.6f
#define LIGHT_SPD_STEP 1.3f
#define LIGHT_SPD_MIN 0.02f
#define LIGHT_SPD_MAX 8.0f

/* §1.9 the light's looping path — how wide it circles, how high it bobs, and
 * how far back it sits (a figure-eight-ish loop that never quite repeats). */
#define LIGHT_RADIUS_X 3.5f
#define LIGHT_RADIUS_Z 3.5f
#define LIGHT_BIAS_Y 2.0f
#define LIGHT_AMPLITUDE_Y 1.0f
#define LIGHT_RATE_Y 0.6f
#define LIGHT_BIAS_Z 1.0f

/* §1.10 how much each kind of light counts when shading a spot (see §10). */
#define KA 0.08f   /* a little glow everywhere, so nothing is pure black    */
#define KD 0.72f   /* brightness from facing the light                      */
#define KS 0.65f   /* shiny-highlight strength (stronger than the sphere's) */
#define SHIN 50.0f /* highlight tightness — bigger = smaller, sharper spot  */

/* §1.11 the brightness ramp + colour-pair slots. */
enum {
  LUMI_N = 8,             /* 8 colour pairs hold the brightness ramp */
  PAIR_HUD = LUMI_N + 1,  /* yellow status line (top)                */
  PAIR_HINT = LUMI_N + 2, /* cyan key reminders (bottom)             */
};

/* the shading characters, dark (space) to bright (@) */
static const char LUMA_RAMP[] = " .,:;+=oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1))

/* §1.12 colour themes — a name plus 8 colour codes running dark→bright.  t/T
 * picks which one is active; the shape and shading never change.  Every colour
 * sits in the bright half of the 256-colour set so even the dimmest step stays
 * visible on a black terminal. */
typedef struct {
  const char *display_name; /* shown in the HUD, padded to a fixed width */
  short ramp_256[LUMI_N];   /* 8 xterm-256 codes, dark to bright         */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    {"CLASSIC ", {235, 238, 241, 244, 247, 250, 253, 255}},
    {"AMBER   ", {130, 136, 166, 172, 178, 208, 214, 220}},
    {"MATRIX  ", {28, 34, 40, 46, 82, 118, 154, 190}},
    {"NEON    ", {53, 91, 129, 165, 201, 207, 213, 227}},
    {"ICE     ", {25, 31, 38, 45, 51, 87, 123, 159}},
    {"COPPER  ", {94, 130, 136, 166, 172, 208, 214, 220}},
};

/* §1.13 the views you flip through with d / D — the normal lit cube, plus
 * three that paint a raw fact instead of shading it. */
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit cube                 */
  DEBUG_NORMALS = 1, /* colour by which way each face points       */
  DEBUG_DEPTH = 2,   /* brighter = closer to the camera            */
  DEBUG_STEPS = 3,   /* brighter = the ray took more steps to land */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* §1.14 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock — read the time and sleep ──────────────────────────────── *
 * We use the monotonic clock because it only ever counts forward; it won't
 * jump if someone changes the system time. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3 color — themes + HUD colours ─────────────────────────────────── *
 * Hands ncurses the current theme's 8 shades plus the two HUD colours, at
 * startup and on theme change.  Terminals with fewer than 256 colours fall
 * back to plain white and fake brighter/dimmer with bold and dim instead. */

/* point the 8 brightness slots at the chosen theme's colours */
static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), theme->ramp_256[i], COLOR_BLACK);
  } else {
    /* not enough colours for themes — just use white; lumi_attr fakes
     * brighter/dimmer with bold and dim instead. */
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), COLOR_WHITE, COLOR_BLACK);
  }
}

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

/* the ncurses colour/attribute for one of the 8 brightness slots */
static attr_t lumi_attr(int l) {
  if (l < 0)
    l = 0;
  if (l > LUMI_N - 1)
    l = LUMI_N - 1;
  attr_t a = COLOR_PAIR(l + 1);
  if (COLORS < 256) {
    if (l < 3)
      a |= A_DIM;
    else if (l >= 6)
      a |= A_BOLD;
  }
  return a;
}

/* ── §4 vec3 — a 3-D point or direction (x,y,z) and the math on it ────── *
 * Everything from here through §11 is pure: it reads its inputs and returns an
 * answer — no shared state, no screen — so the drawing code can never change
 * what it computes. */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3norm(Vec3 a) {
  float L = v3len(a);
  return (L > 1e-7f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

static inline Vec3 v3abs(Vec3 a) {
  return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

static inline Vec3 v3max0(Vec3 a) {
  return v3(fmaxf(a.x, 0.0f), fmaxf(a.y, 0.0f), fmaxf(a.z, 0.0f));
}

/* the biggest of the three components (used by the box distance, §5) */
static inline float v3maxcomp(Vec3 a) {
  return fmaxf(a.x, fmaxf(a.y, a.z));
}

/* bounce direction v off a surface that faces way n (n must be unit length) —
 * used to find where light reflects toward the eye (§10). */
static inline Vec3 v3reflect(Vec3 v, Vec3 n) {
  return v3sub(v3mul(n, 2.0f * v3dot(n, v)), v);
}

/* squeeze a value into 0..1 — brightness has to land there before it picks a
 * character or colour. */
static inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/* ── §5 box SDF — how far a point is from the cube's surface ──────────── *
 * Negative inside, zero on the surface, positive outside.  It's the exact
 * distance, never an over-estimate, which is what lets the ray march step
 * safely (§9).  Two parts add up: how far OUTSIDE the box you are (distance to
 * the nearest face, edge, or corner) and, if you're inside, how far in.
 * (Iñigo Quílez's box function.) */
static float sdf_box(Vec3 p, float cube_half) {
  Vec3 q = v3sub(v3abs(p), v3(cube_half, cube_half, cube_half));
  Vec3 q_pos = v3max0(q);
  float outside = v3len(q_pos);
  float inside = fminf(v3maxcomp(q), 0.0f);
  return outside + inside;
}

/* ── §6 rotate query point — make the cube appear to spin ────────────── *
 * The cube never actually moves.  To make it LOOK like it's spinning, we spin
 * the question instead: before asking "how far is this point from the cube?",
 * we rotate the point the other way — first around the up axis, then around
 * the sideways one, which gives the tumbling look.
 *
 * The matrices, for reference:
 *      around Y            around X
 *      [ c  0  s]          [1  0  0]
 *      [ 0  1  0]          [0  c -s]
 *      [-s  0  c]          [0  s  c]
 * (Because we turn the point rather than the cube, the cube appears to spin
 *  the opposite way from the angle's sign — invisible, since the speeds are
 *  just positive numbers.) */
/* spin p around the Y (up) axis by ry — see the Y matrix above */
static inline Vec3 rotate_y(Vec3 p, float ry) {
  float c = cosf(ry), s = sinf(ry);
  return v3(p.x * c + p.z * s, p.y, -p.x * s + p.z * c);
}

/* spin p around the X (sideways) axis by rx — see the X matrix above */
static inline Vec3 rotate_x(Vec3 p, float rx) {
  float c = cosf(rx), s = sinf(rx);
  return v3(p.x, p.y * c - p.z * s, p.y * s + p.z * c);
}

static Vec3 rotate_query_point(Vec3 p, float rx, float ry) {
  p = rotate_y(p, ry);
  p = rotate_x(p, rx);
  return p;
}

/* ── §7 scene SDF — the distance to the whole scene ──────────────────── *
 * Rotate the point, then ask the box.  The ray marcher (§9) calls this at
 * every step but knows nothing about cubes — only this one function does, so
 * swapping in a different shape would change just these two lines. */
static float sdf_scene(Vec3 p, float rx, float ry, float cube_half) {
  Vec3 p_local = rotate_query_point(p, rx, ry);
  return sdf_box(p_local, cube_half);
}

/* ── §8 surface normal — which way the surface faces at the hit ──────── *
 * There's no neat formula for a rotated box, so we feel it out: check the
 * distance at four points nudged around the hit and see which way it grows
 * fastest — that direction points straight out of the surface.  The four
 * nudges are the corners of a tetrahedron (no two share an axis), which keeps
 * the cube's edges crisp instead of rounded off. */
static Vec3 surface_normal(Vec3 p, float rx, float ry, float cube_half) {
  const float e = RM_NORM_EPS;

  Vec3 k0 = v3(e, -e, -e);
  Vec3 k1 = v3(-e, -e, e);
  Vec3 k2 = v3(-e, e, -e);
  Vec3 k3 = v3(e, e, e);

  float d0 = sdf_scene(v3add(p, k0), rx, ry, cube_half);
  float d1 = sdf_scene(v3add(p, k1), rx, ry, cube_half);
  float d2 = sdf_scene(v3add(p, k2), rx, ry, cube_half);
  float d3 = sdf_scene(v3add(p, k3), rx, ry, cube_half);

  Vec3 n = v3add(v3add(v3mul(k0, d0), v3mul(k1, d1)),
                 v3add(v3mul(k2, d2), v3mul(k3, d3)));
  return v3norm(n);
}

/* ── §9 sphere trace — creep along the ray until it hits ─────────────── *
 * Each step jumps forward by the distance to the surface — always safe, since
 * nothing is closer than that.  Stop once we're basically touching it; give up
 * if the ray flies off into nothing.  Returns the distance to the hit (or -1
 * for a miss) and, for the STEPS view, how many steps it took — pass NULL if
 * you don't need that.  We start a little way down the ray so it can't "hit"
 * right at the eye.  (Hart, "Sphere Tracing", 1996.) */

/* the point a distance t along the ray from origin ro */
static inline Vec3 ray_at(Vec3 ro, Vec3 rd, float t) {
  return v3add(ro, v3mul(rd, t));
}

static float sphere_trace(Vec3 ro, Vec3 rd, float rx, float ry, float cube_half,
                          int *out_steps) {
  float t = RM_T_START;
  int step;
  for (step = 0; step < RM_MAX_STEPS; step++) {
    Vec3 p = ray_at(ro, rd, t);
    float d = sdf_scene(p, rx, ry, cube_half);
    if (d < RM_HIT_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  if (out_steps)
    *out_steps = step;
  return -1.0f;
}

/* ── §10 shade — turn a surface spot into a brightness 0..1 ──────────── *
 * Add up three classic pieces: a faint everywhere-glow (ambient), how squarely
 * the spot faces the light (diffuse), and a shiny highlight (specular).
 * (Phong, 1975.) */

/* the shiny highlight — a bright dot where the light bounces straight back at
 * the eye.  Only where the spot actually faces the light; otherwise a back face
 * would catch a phantom highlight from the reflected direction. */
static float specular_term(Vec3 N, Vec3 L, Vec3 V, float ndl) {
  if (ndl <= 0.0f)
    return 0.0f;
  Vec3 R = v3reflect(L, N);
  return powf(fmaxf(0.0f, v3dot(R, V)), SHIN);
}

static float phong_shade(Vec3 N, Vec3 hit, Vec3 cam, Vec3 light) {
  Vec3 L = v3norm(v3sub(light, hit)); /* toward the light */
  Vec3 V = v3norm(v3sub(cam, hit));   /* toward the eye   */
  float ndl = v3dot(N, L);

  float ambient = KA;
  float diffuse = KD * fmaxf(0.0f, ndl);
  float specular = KS * specular_term(N, L, V, ndl);

  return clamp01(ambient + diffuse + specular);
}

/* ── §11 cast_ray — one cell's whole trip → a Hit ────────────────────── *
 * Aim a ray for one cell, march it, and on a hit record where it landed,
 * which way the surface faces, and how bright it is.  Inputs: the cube
 * (read-only), the light, and the camera distance.  Each cell is traced just
 * once; the normal view and all three debug views read back from the same
 * Hit, so the ray is never traced twice. */

/* Cube — the spinning cube the whole renderer is about: its current
 * orientation (angle_x, angle_y, advanced each tick at spin_x / spin_y) and
 * its size (half_extent, the half-width the +/- keys change).  Angles in
 * radians, spin in radians/second, half_extent in world units. */
typedef struct {
  float half_extent;      /* cube spans -half_extent .. +half_extent per axis */
  float angle_x, angle_y; /* current orientation (rad) */
  float spin_x, spin_y;   /* spin rate (rad/sec)       */
} Cube;

/* Hit — what one ray found at one cell.  Filled once and stored so every view
 * can read whatever it needs without tracing again.  When hit is false, the
 * other fields are meaningless. */
typedef struct {
  bool hit;             /* did the ray reach the cube at all? */
  Vec3 hit_point;       /* where on the surface it landed */
  Vec3 normal;          /* which way the surface faces there */
  float intensity;      /* brightness 0..1, for the normal view */
  float trace_distance; /* how far the ray travelled to get there (DEPTH view) */
  int step_count;       /* how many creep-steps it took (STEPS view) */
} Hit;

/* which way to fire the ray for one cell: turn the cell's spot on screen into
 * an aim direction.  Top row is row 0, and the squash keeps tall text cells
 * from stretching the cube into a brick. */
static Vec3 ray_through_pixel(int col, int row, int cw, int ch) {
  float u = ((float)col + 0.5f) / (float)cw * 2.0f - 1.0f;
  float v = -((float)row + 0.5f) / (float)ch * 2.0f + 1.0f;
  float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;
  return v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * phys_aspect, -1.0f));
}

static Hit cast_ray(int col, int row, int cw, int ch, const Cube *cube,
                    Vec3 light, float cam_z) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0};

  Vec3 ro = v3(0.0f, 0.0f, cam_z);
  Vec3 rd = ray_through_pixel(col, row, cw, ch);

  int steps = 0;
  float t = sphere_trace(ro, rd, cube->angle_x, cube->angle_y,
                         cube->half_extent, &steps);
  h.step_count = steps;

  if (t < 0.0f)
    return h; /* ray missed the cube */

  h.hit = true;
  h.trace_distance = t;
  h.hit_point = ray_at(ro, rd, t);
  h.normal = surface_normal(h.hit_point, cube->angle_x, cube->angle_y,
                            cube->half_extent);
  h.intensity = phong_shade(h.normal, h.hit_point, ro, light);
  return h;
}

/* ── §12 canvas — one Hit per cell, the whole picture for a frame ─────── *
 * This buffer is scratch for drawing, not part of the simulation: §13 fills it
 * from the scene each frame, §14/§15 read it to paint.  The memory is owned
 * here — made at startup, remade on resize. */

/* Canvas — the grid of ray hits for one frame, plus its size. */
typedef struct {
  int w, h;  /* size in cells */
  Hit *hits; /* w*h Hits, stored row by row */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->w = canvas_w_from_cols(cols);
  c->h = canvas_h_from_rows(rows);
  c->hits = calloc((size_t)(c->w * c->h), sizeof(Hit));
}

static void canvas_free(Canvas *c) {
  free(c->hits);
  c->hits = NULL;
  c->w = c->h = 0;
}

/* ── §13 render — trace a ray for every cell, fill the Hit buffer ─────── *
 * Doesn't know about characters or colour yet — that's the next section.
 * cam_z is passed in (not fixed) so the zoom keys can change it. */
static void canvas_render(Canvas *c, const Cube *cube, Vec3 light, float cam_z) {
  for (int row = 0; row < c->h; row++) {
    for (int col = 0; col < c->w; col++) {
      c->hits[row * c->w + col] =
          cast_ray(col, row, c->w, c->h, cube, light, cam_z);
    }
  }
}

/* ── §14 draw — the normal view: brightness → character + colour ─────── *
 * A spot's brightness picks both the character and the colour it's drawn in. */

static char intensity_to_glyph(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= RAMP_LEN)
    idx = RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static attr_t intensity_to_attr(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  int slot = (idx * LUMI_N) / RAMP_LEN;
  return lumi_attr(slot);
}

/* where to put the canvas's top-left so it sits centred in the terminal */
static void canvas_offsets(const Canvas *c, int term_cols, int term_rows,
                           int *out_off_x, int *out_off_y) {
  int total_w = c->w * CELL_W;
  int total_h = c->h * CELL_H;
  *out_off_x = (term_cols - total_w) / 2;
  *out_off_y = (term_rows - total_h) / 2;
}

/* draw one canvas pixel.  It's a single cell here, but we loop over a
 * CELL_W×CELL_H block (clipped to the screen) so the sibling demos can use
 * chunkier pixels. */
static void emit_block(int tx0, int ty0, char glyph, attr_t attr, int term_cols,
                       int term_rows) {
  attron(attr);
  for (int by = 0; by < CELL_H; by++) {
    for (int bx = 0; bx < CELL_W; bx++) {
      int tx = tx0 + bx;
      int ty = ty0 + by;
      if (tx < 0 || tx >= term_cols)
        continue;
      if (ty < 0 || ty >= term_rows)
        continue;
      mvaddch(ty, tx, (chtype)(unsigned char)glyph);
    }
  }
  attroff(attr);
}

static void canvas_draw(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      char glyph = intensity_to_glyph(h->intensity);
      attr_t attr = intensity_to_attr(h->intensity);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §15 debug views (d/D) — paint one raw fact instead of shading ───── *
 * Each is worked out here from the Hit buffer at draw time, nothing stored.
 *   NORMALS — colour each cell by which way its face points.  A cube has six
 *             flat faces, so you get six solid patches of colour.
 *   DEPTH   — nearer the camera = brighter.
 *   STEPS   — the rim glows: rays grazing the edge work hardest before they
 *             decide they missed. */

/* pick a colour slot from which way a face points.  Whichever of x/y/z the
 * normal leans toward most names the face (left/right, up/down, front/back),
 * and its sign picks between the two — so each of the six faces gets its own. */
static int face_slot_for_normal(Vec3 N) {
  float ax = fabsf(N.x), ay = fabsf(N.y), az = fabsf(N.z);
  if (ax >= ay && ax >= az)
    return (N.x > 0) ? 7 : 6;
  if (ay >= az)
    return (N.y > 0) ? 5 : 4;
  return (N.z > 0) ? 2 : 1;
}

static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      int slot = face_slot_for_normal(h->normal);
      char glyph = LUMA_RAMP[(slot * RAMP_LEN) / LUMI_N];
      attr_t attr = lumi_attr(slot);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_depth(const Canvas *c, int term_cols, int term_rows,
                              float cam_z) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  /* The cube reaches at most ±SIZE_MAXX·√3 from the origin (corner distance),
   * so the nearest/farthest a hit can be is cam_z ∓ that.  Closer = brighter. */
  float t_min = cam_z - SIZE_MAXX * CUBE_CORNER_DIST;
  float t_max = cam_z + SIZE_MAXX * CUBE_CORNER_DIST;
  if (t_min < 0.0f)
    t_min = 0.0f;

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float depth_n = clamp01((t_max - h->trace_distance) / (t_max - t_min));

      char glyph = intensity_to_glyph(depth_n);
      attr_t attr = intensity_to_attr(depth_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_steps(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float steps_n = clamp01((float)h->step_count / (float)RM_MAX_STEPS);

      char glyph = intensity_to_glyph(steps_n);
      attr_t attr = intensity_to_attr(steps_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §16 scene — the program's state bundle ──────────────────────────── *
 * Scene holds everything; scene_tick is the only thing that moves it forward. */

/* Scene — the whole program state, as a table of contents:
 *   WHAT is simulated   cube         the spinning cube (orientation+size+spin)
 *   HOW the user drives  light_spd    light-orbit speed (l/L)
 *                        cam_z        camera distance / zoom (z/Z)
 *                        theme_index  colour palette (t/T)
 *                        debug_mode   which view (d/D)
 *   WHERE/when           time         animation clock (seconds)
 *                        paused       freezes the tick (space)
 *   render buffer        canvas       one Hit per cell (render scratch, §12)
 * (light, camera, and the view knobs are one or two loose fields each — too
 *  thin to be their own types, so they live directly on Scene.) */
typedef struct {
  Cube cube;

  float light_spd;     /* light orbit speed (rad/sec)   */
  float cam_z;         /* camera z; smaller = zoomed-in */
  int theme_index;     /* index into THEMES[]           */
  DebugMode debug_mode;

  float time;          /* seconds since start */
  bool paused;

  Canvas canvas;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  canvas_alloc(&s->canvas, cols, rows);
  s->cube.angle_x = 0.3f;
  s->cube.angle_y = 0.5f;
  s->cube.spin_x = ROT_X_DEFAULT;
  s->cube.spin_y = ROT_Y_DEFAULT;
  s->cube.half_extent = CUBE_H_DEFAULT;
  s->light_spd = LIGHT_SPD_DEFAULT;
  s->time = 0.0f;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme_index = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

/* where the light is right now: it loops around the cube on a path that never
 * quite repeats, and is shaped so it never crosses into the cube. */
static Vec3 scene_light(const Scene *s) {
  float a = s->time * s->light_spd;
  return v3(LIGHT_RADIUS_X * cosf(a),
            LIGHT_BIAS_Y + LIGHT_AMPLITUDE_Y * sinf(LIGHT_RATE_Y * a),
            LIGHT_RADIUS_Z * sinf(a) + LIGHT_BIAS_Z);
}

/* the only thing that moves the scene forward: turn the cube and advance the
 * clock (both frozen while paused) */
static void scene_tick(Scene *s, float dt_sec) {
  if (s->paused)
    return;
  s->cube.angle_x += s->cube.spin_x * dt_sec;
  s->cube.angle_y += s->cube.spin_y * dt_sec;
  s->time += dt_sec;
}

static void scene_render(Scene *s) {
  canvas_render(&s->canvas, &s->cube, scene_light(s), s->cam_z);
}

/* draw whichever view is currently picked */
static void scene_draw_active(const Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(&s->canvas, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(&s->canvas, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(&s->canvas, cols, rows, s->cam_z);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(&s->canvas, cols, rows);
    break;
  default:
    canvas_draw(&s->canvas, cols, rows);
    break;
  }
}

/* ── §17 screen — ncurses init / HUD / present ───────────────────────── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE); /* getch() returns right away if no key is waiting */
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt our drawing */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

/* the endwin + refresh dance makes ncurses pick up the new terminal size */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* fill buf with the top status line: fps + every live parameter. */
static void hud_format_status(char *buf, size_t n, const Scene *sc, double fps) {
  snprintf(buf, n,
           " %5.1f fps  spd:%.2f  h:%.2f  zoom:%.2f  theme:%s  "
           "debug:%s  [%dx%d]  %s ",
           fps, sc->cube.spin_y, sc->cube.half_extent, sc->cam_z,
           THEMES[sc->theme_index].display_name,
           DEBUG_MODE_NAMES[sc->debug_mode], sc->canvas.w, sc->canvas.h,
           sc->paused ? "PAUSED" : "running");
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Scene *sc, double fps) {
  erase();
  scene_draw_active(sc, s->cols, s->rows);

  char status[200];
  hud_format_status(status, sizeof status, sc, fps);
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " CUBE ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  ]/[:spin  +/-:size  l/L:light  "
           "z/Z:zoom  t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §18 app — the main loop and the keys ────────────────────────────── *
 * The main loop ties everything together each frame.  Key presses and resizes
 * change the scene between frames, not during the simulation step. */

/* App — everything the running program owns.  The last two are flipped from
 * inside signal handlers, so they're volatile and acted on between frames. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                       /* how many times a second the cube steps */
  volatile sig_atomic_t running;     /* a quit signal sets this to 0 */
  volatile sig_atomic_t need_resize; /* SIGWINCH sets this; handled next frame */
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

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
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

  case ']':
    s->cube.spin_x *= ROT_STEP;
    s->cube.spin_y *= ROT_STEP;
    if (s->cube.spin_x > ROT_MAX)
      s->cube.spin_x = ROT_MAX;
    if (s->cube.spin_y > ROT_MAX)
      s->cube.spin_y = ROT_MAX;
    break;
  case '[':
    s->cube.spin_x /= ROT_STEP;
    s->cube.spin_y /= ROT_STEP;
    if (s->cube.spin_x < ROT_MIN)
      s->cube.spin_x = ROT_MIN;
    if (s->cube.spin_y < ROT_MIN)
      s->cube.spin_y = ROT_MIN;
    break;

  case '=':
  case '+':
    s->cube.half_extent *= SIZE_STEP;
    if (s->cube.half_extent > SIZE_MAXX)
      s->cube.half_extent = SIZE_MAXX;
    break;
  case '-':
    s->cube.half_extent /= SIZE_STEP;
    if (s->cube.half_extent < SIZE_MIN)
      s->cube.half_extent = SIZE_MIN;
    break;

  case 'l':
    s->light_spd *= LIGHT_SPD_STEP;
    if (s->light_spd > LIGHT_SPD_MAX)
      s->light_spd = LIGHT_SPD_MAX;
    break;
  case 'L':
    s->light_spd /= LIGHT_SPD_STEP;
    if (s->light_spd < LIGHT_SPD_MIN)
      s->light_spd = LIGHT_SPD_MIN;
    break;

  case 'z':
    s->cam_z -= CAM_ZOOM_STEP;
    if (s->cam_z < CAM_Z_MIN)
      s->cam_z = CAM_Z_MIN;
    break;
  case 'Z':
    s->cam_z += CAM_ZOOM_STEP;
    if (s->cam_z > CAM_Z_MAX)
      s->cam_z = CAM_Z_MAX;
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
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
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) { /* 1. apply a pending resize, before timing the frame */
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* 2. measure how long the last frame took, capping a long stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    /* 3. advance the simulation in fixed steps — the only place state moves */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    scene_render(&app->scene); /* 4. fill the Hit buffer by tracing the rays */

    /* 5. refresh the fps number, then sleep to hold a steady frame rate */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display); /* 6. draw it */
    screen_present();

    int ch = getch(); /* 7. read one key (changes state for the next frame) */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
