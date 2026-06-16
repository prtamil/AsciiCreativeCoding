/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * deferred_rendering_pipeline.c — a white sphere lit by red, green, and blue
 * lights orbiting around it; where two pools overlap you get yellow/cyan/magenta,
 * where all three meet, white. It shows off "deferred" rendering: draw the
 * surface ONCE into a few per-pixel tables, then add up the lights in a separate
 * pass — so an extra light (press 'l', up to 8) costs almost nothing.
 *
 * Keys: g layer view · l add light · +/- zoom · space pause · r reset · q quit
 * Read raster/cube_raster.c first — this is the same renderer split into two
 * passes. The same trick powers Unity HDRP and Unreal's default path.
 * Ideas from: the G-buffer (Saito & Takahashi, SIGGRAPH '90); Blinn-Phong
 *   lighting (Blinn 1977); Reinhard tone-map (SIGGRAPH '02).
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/deferred_rendering_pipeline.c -o deferred -lncurses -lm
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

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + scene capacity */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
  HUD_ROWS = 5,    /* yellow row 0 + 3 educational rows + cyan hint */
  MAX_OBJECTS = 4, /* one sphere — extras reserved for later        */
  MAX_LIGHTS = 8,  /* 3 RGB primaries + 5 'l'-key extras            */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 size of the per-pixel tables. Fixed-size (no malloc), big enough for a
 * large terminal; anything past the edge is skipped. */
#define GBUF_MAX_W 300
#define GBUF_MAX_H 80

/* §1.3 view geometry */
#define CAM_FOV (55.0f * (float)M_PI / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 20.0f
#define NEAR_W_EPS 0.001f /* a corner closer to the eye than this is behind it */

/* CAM_DIST is the DEFAULT eye Z distance; +/- keys slide the eye between
 * CAM_DIST_MIN (close-up — sphere fills the screen) and CAM_DIST_MAX
 * (wide shot — sphere is small, all three coloured pools easy to track
 * at once). MIN is kept above BALL_RADIUS (0.95) so the camera never
 * crosses inside the sphere; MAX is well within CAM_FAR (20.0) so the
 * sphere never falls past the far plane. */
#define CAM_DIST 3.8f      /* default eye Z distance (world units) */
#define CAM_DIST_MIN 1.8f  /* closest zoom — just outside sphere   */
#define CAM_DIST_MAX 8.0f  /* farthest zoom                        */
#define CAM_ZOOM_STEP 0.2f /* world units moved per +/- press      */

#define CELL_W 8  /* terminal cell width  (pixels)              */
#define CELL_H 16 /* terminal cell height (pixels)              */

/* §1.4 lighting */
#define SHININESS 32.0f        /* how tight the shiny highlight is (bigger = tighter) */
#define AMBIENT_STR 0.06f      /* a little light everywhere, so nothing is pure black */
#define AMBIENT_BLUE_BIAS 1.1f /* tint that base light slightly cool                  */
#define SPEC_GAIN 0.35f        /* how strong the shiny highlight is                   */

/* §1.5 the scene's sizes (world units) */

/* The one and only object: a white sphere. Big enough to fill most of the screen
 * so the coloured pools have room to show. White means the coloured lights show
 * up as their true colour — red light on white = pure red, not a pinkish mix. */
#define BALL_RADIUS 0.95f
#define BALL_RINGS 16
#define BALL_SEGS 24

/* The camera sits still, looking at the sphere from the front with a slight tilt.
 * Since only the lights move, anything that changes on screen is the lighting,
 * not the viewpoint — which is the whole point of the demo. */
#define CAM_EYE_Y 0.30f /* slight tilt up, for a sense of depth */
#define CAM_LOOK_Y 0.0f /* aimed at the sphere's centre        */

/* §1.6 the brightness-to-character ladder, sparse to dense (Paul Bourke's). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 a fixed 4×4 nudge pattern (values 0..1). Added to each cell's brightness
 * so neighbours of similar brightness pick different characters, hiding the
 * steps in a smooth gradient. */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_AMP 0.12f

/* §1.7b a cell brighter than this is drawn bold, darker than the other is drawn
 * dim, in between is normal — so only highlights pop and only near-black recedes. */
#define LUMA_BOLD_ABOVE 0.85f
#define LUMA_DIM_BELOW 0.15f

/* §1.8 ncurses colour-pair numbers: 216 for the RGB cube, plus a yellow for the
 * status bar and a cyan for the hint line. */
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

/* Transforms a point (translation counts). */
static inline Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

/* Transforms a direction (translation doesn't count — directions don't have a
 * location). */
static inline Vec3 m4_dir(Mat4 m, Vec3 d) {
  Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
  return v3(r.x, r.y, r.z);
}

/* Builds the perspective transform — the one that makes far things smaller, the
 * usual way 3-D looks on a screen. */
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

/* Makes the matrix for rotating surface-facing directions (normals). You can't
 * just reuse the object's own matrix: if a shape were stretched unevenly its
 * normals would come out skewed; this keeps them pointing straight out. */
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
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* Snaps one colour channel (0..1) to one of the cube's 6 steps (0..5). */
static inline int quantize_unit_to_5(float x) {
  int q = (int)(x * 5.f + 0.5f);
  return q < 0 ? 0 : (q > 5 ? 5 : q);
}

/* How bright a colour looks to the eye (green counts most, blue least). */
static inline float rec709_luma(float r, float g, float b) {
  return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

/* Picks the character for a given brightness (0..1): faint chars for dim,
 * dense chars for bright. */
static inline int ramp_index(float luma) {
  int idx = (int)(luma * (BOURKE_LEN - 1) + 0.5f);
  return idx < 0 ? 0 : (idx >= BOURKE_LEN ? BOURKE_LEN - 1 : idx);
}

/* Draws one terminal cell from a colour — the last stop in the whole pipeline.
 * Brings the colour into screen range, then picks the closest cube colour and a
 * character for the brightness (with a dither nudge so gradients stay smooth),
 * and bolds the bright cells / dims the dark ones. The range-fix must come first:
 * skip it and every bright colour would slam into the same brightest cube cell. */
static void paint_cell(int sx, int sy, Vec3 col) {
  /* bring each channel into screen range (roll off the too-bright, then correct) */
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  /* nudge the brightness so neighbours don't all pick the same character */
  float luma = rec709_luma(r, g, b);
  float dither = (k_bayer[sy & 3][sx & 3] - 0.5f) * DITHER_AMP;
  float luma_dithered = clamp01(luma + dither);

  /* the closest cube colour */
  int pair = PAIR_CUBE_BASE;
  if (g_256)
    pair += quantize_unit_to_5(r) * 36 + quantize_unit_to_5(g) * 6 +
            quantize_unit_to_5(b);

  /* the character (from the nudged brightness) and bold/dim (from the real one) */
  int glyph = ramp_index(luma_dithered);
  int attr = (luma > LUMA_BOLD_ABOVE)  ? A_BOLD
             : (luma < LUMA_DIM_BELOW) ? A_DIM
                                       : A_NORMAL;

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)k_bourke[glyph]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* ── §5 mesh — building the sphere at startup ──────────────────────────── */

/*
 * Vertex / Triangle / Mesh — a shape made of triangles. To avoid storing the
 * same corner over and over, the corners live once in one list and each triangle
 * just points at three of them by their slot number.
 *
 * Vertex is one corner. It carries which way the surface faces there; on the
 * sphere that's just the direction from the centre out to the corner, and since
 * neighbouring corners face slightly different ways, the in-between pixels blend
 * smoothly and the ball looks round (not faceted). The texture coords are filled
 * in but unused — this demo has no images.
 *
 * Triangle is three corner-slots, listed counter-clockwise seen from outside;
 * the "skip faces turned away" check relies on that ordering.
 *
 * Mesh owns its two lists on the heap — the only memory the program allocates,
 * done once when the sphere is built; the per-frame drawing never allocates.
 */
typedef struct {
  Vec3 pos;    /* the corner's position, in the shape's own coordinates */
  Vec3 normal; /* which way the surface faces here                      */
  float u, v;  /* texture coords, unused here                           */
} Vertex;
typedef struct {
  int v[3]; /* three corner-slots, counter-clockwise from outside */
} Triangle;
typedef struct {
  Vertex *verts;  /* the corner list (heap, built once)   */
  Triangle *tris; /* the triangle list (heap, built once) */
  int nvert;      /* how many corners are filled in        */
  int ntri;       /* how many triangles are filled in      */
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/* The sphere's corners sit on a grid of rings (top to bottom) and segments
 * (around). This finds the slot for one (ring, segment). Each ring keeps one
 * extra column so the seam where it wraps lines up cleanly. */
static inline int grid_index(int i, int j, int segs) {
  return i * (segs + 1) + j;
}

/* Places one corner on the sphere, like a point at a given latitude (ring) and
 * longitude (segment) on a globe. It faces straight out from the centre. */
static Vertex sphere_vertex(float radius, int i, int j, int rings, int segs) {
  float theta = (float)M_PI * (float)i / (float)rings;
  float phi = 2.f * (float)M_PI * (float)j / (float)segs;
  float st = sinf(theta), ct = cosf(theta);
  Vec3 pos = v3(radius * st * cosf(phi), radius * ct, radius * st * sinf(phi));
  Vec3 nrm = v3(pos.x / radius, pos.y / radius, pos.z / radius);
  return (Vertex){pos, nrm, (float)j / (float)segs, (float)i / (float)rings};
}

/* Builds the ball out of rings and segments, like a globe's latitude and
 * longitude lines, then stitches each little grid square into two triangles.
 * Every corner faces straight out from the centre, so the ball shades smoothly. */
static Mesh tessellate_sphere(float radius, int rings, int segs) {
  int n_verts = (rings + 1) * (segs + 1);
  int n_tris = rings * segs * 2;
  Mesh m;
  m.verts = malloc((size_t)n_verts * sizeof(Vertex));
  m.tris = malloc((size_t)n_tris * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  /* place every corner on the grid */
  for (int i = 0; i <= rings; i++)
    for (int j = 0; j <= segs; j++)
      m.verts[m.nvert++] = sphere_vertex(radius, i, j, rings, segs);

  /* stitch each grid square into two triangles */
  for (int i = 0; i < rings; i++) {
    for (int j = 0; j < segs; j++) {
      int v00 = grid_index(i, j, segs);
      int v10 = grid_index(i + 1, j, segs);
      int v11 = grid_index(i + 1, j + 1, segs);
      int v01 = grid_index(i, j + 1, segs);
      m.tris[m.ntri++] = (Triangle){{v00, v10, v01}};
      m.tris[m.ntri++] = (Triangle){{v10, v11, v01}};
    }
  }
  return m;
}

/* ── §6 G-buffer — drawing the surface into per-pixel tables ───────────── */

/* §6.1 ── the tables ──────────────────────────────────────────────────── */

/*
 * GBuffer — a set of full-screen tables describing the surface seen at each
 * cell. This is the heart of "deferred" rendering (Saito & Takahashi 1990): pass
 * 1 (render_gbuffer) fills the surface tables, pass 2 (render_lightpass) reads
 * them and fills `light`, pass 3 (render_scene) shows whichever table the 'g'
 * key has selected. One global instance (g_gbuf) holds it all.
 *
 *   the surface facts  (filled by pass 1, for the nearest surface at each cell)
 *     pos     — where the surface is in the world
 *     normal  — which way it faces
 *     albedo  — its plain colour, before any light
 *     zbuf    — how near it is, so a farther surface can't paint over a nearer
 *               one (starts each frame at "infinitely far")
 *     valid   — 1 if a surface was drawn here, 0 for empty background
 *   the lit result     (filled by pass 2)
 *     light   — the colour after adding up all the lights
 *
 * Kept as its own global, not part of the Scene: it's wiped and refilled every
 * frame and the scene never reads it, so the scene and the pixels stay separate.
 */
typedef struct {
  /* the surface facts — filled by pass 1, nearest surface at each cell */
  Vec3 pos[GBUF_MAX_H][GBUF_MAX_W];      /* where it is in the world      */
  Vec3 normal[GBUF_MAX_H][GBUF_MAX_W];   /* which way it faces            */
  Vec3 albedo[GBUF_MAX_H][GBUF_MAX_W];   /* its plain colour, no light    */
  float zbuf[GBUF_MAX_H][GBUF_MAX_W];    /* how near (for keeping closest) */
  uint8_t valid[GBUF_MAX_H][GBUF_MAX_W]; /* 1 = surface here, 0 = empty    */
  /* the lit result — filled by pass 2 */
  Vec3 light[GBUF_MAX_H][GBUF_MAX_W]; /* colour after adding up the lights */
} GBuffer;

static GBuffer g_gbuf;

/* Wipes the depth and the "is there a surface" flag before each frame. The
 * pos/normal/albedo tables don't need wiping — nothing reads a cell unless its
 * valid flag says a surface was drawn there. */
static void gbuffer_clear(GBuffer *gb, int cols, int rows) {
  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      gb->zbuf[r][c] = 1.0f; /* farthest */
      gb->valid[r][c] = 0;
    }
  }
}

/* §6.2 ── filling a triangle ──────────────────────────────────────────── */

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

/* §6.3 ── drawing one object's triangles into the tables ──────────────── */

/* This is the whole first pass for one object, split into small named steps so
 * rasterize_object below reads like a checklist: place the corners, drop the
 * triangle if it's behind us or facing away, then fill the cells it covers. No
 * lighting happens here at all — that's what makes it "deferred". */

/* Works out, for a triangle's 3 corners, where each lands on screen plus where
 * it sits in the world and which way it faces (the fill step blends these). */
static void transform_vertices(const Mesh *mesh, const Triangle *tri, Mat4 mvp,
                               Mat4 model, Mat4 norm_mat, Vec4 clip[3],
                               Vec3 wpos[3], Vec3 wnrm[3]) {
  for (int vi = 0; vi < 3; vi++) {
    const Vertex *v = &mesh->verts[tri->v[vi]];
    clip[vi] = m4_mul_v4(mvp, v4(v->pos.x, v->pos.y, v->pos.z, 1.f));
    wpos[vi] = m4_pt(model, v->pos);
    wnrm[vi] = v3_norm(m4_dir(norm_mat, v->normal));
  }
}

/* True when all three corners are behind the camera — the whole triangle is
 * off-screen, so skip it. */
static bool all_behind_near_plane(const Vec4 clip[3]) {
  return clip[0].w < NEAR_W_EPS && clip[1].w < NEAR_W_EPS &&
         clip[2].w < NEAR_W_EPS;
}

/* Finishes the corners onto the screen: divide by distance (far = smaller),
 * scale into cells, and flip y because screen rows count downward but up should
 * be up. */
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

/* True if this triangle is turned away from us (the back of the sphere), so we
 * can skip it. The trick: measure its signed area on screen; with our corner
 * ordering and the y-flip, faces toward us come out negative, so zero-or-positive
 * is facing away. */
static bool is_back_facing(const float sx[3], const float sy[3]) {
  float area =
      (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
  return area >= 0.f;
}

/* Walks the cells the triangle covers; for each one inside it and nearer than
 * whatever's there, records the surface's facts into the tables. */
static void rasterize_fragments(GBuffer *gb, const float sx[3],
                                const float sy[3], const float sz[3],
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
      gb->pos[py][px] = v3_bary(wpos[0], wpos[1], wpos[2], b[0], b[1], b[2]);
      gb->normal[py][px] =
          v3_norm(v3_bary(wnrm[0], wnrm[1], wnrm[2], b[0], b[1], b[2]));
      gb->albedo[py][px] = albedo;
      gb->valid[py][px] = 1;
    }
  }
}

static void rasterize_object(GBuffer *gb, const Mesh *mesh, Vec3 albedo,
                             Mat4 mvp, Mat4 model, Mat4 norm_mat, int cols,
                             int rows) {
  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];

    Vec4 clip[3];
    Vec3 wpos[3], wnrm[3];
    transform_vertices(mesh, tri, mvp, model, norm_mat, clip, wpos, wnrm);
    if (all_behind_near_plane(clip))
      continue;

    float sx[3], sy[3], sz[3];
    project_to_screen(clip, cols, rows, sx, sy, sz);
    if (is_back_facing(sx, sy))
      continue;

    rasterize_fragments(gb, sx, sy, sz, wpos, wnrm, albedo, cols, rows);
  }
}

/* §6.4 ── pass 1: draw every object into the tables ───────────────────── */

/* Fills the tables with the nearest surface at every cell. This is the same no
 * matter how many lights there are — adding a light never re-runs it, which is
 * the whole win of deferred rendering. */
static void render_gbuffer(GBuffer *gb, const Mesh *meshes, const Vec3 *albedos,
                           const Mat4 *models, int n_objects, const Mat4 *view,
                           const Mat4 *proj, int cols, int rows) {
  gbuffer_clear(gb, cols, rows);
  for (int oi = 0; oi < n_objects; oi++) {
    Mat4 mv = m4_mul(*view, models[oi]);
    Mat4 mvp = m4_mul(*proj, mv);
    Mat4 nmat = m4_normal_mat(models[oi]);
    rasterize_object(gb, &meshes[oi], albedos[oi], mvp, models[oi], nmat, cols,
                     rows);
  }
}

/* ── §7 lightpass — pass 2: add up the lights ──────────────────────────── */

/* §7.1 ── one light ───────────────────────────────────────────────────── */

/*
 * PointLight — one coloured light circling the sphere. It rides a horizontal
 * ring; each tick scene_tick nudges its angle around the ring and recomputes
 * where it is. The first five fields are set once and don't change; only pos is
 * recomputed every frame. Its colour tints both the soft lighting and the shiny
 * highlight. Ref: Blinn-Phong lighting (Blinn, SIGGRAPH '77).
 *   color        — its colour (pure red/green/blue for the main three)
 *   orbit_radius — how wide its ring is
 *   orbit_speed  — how fast it goes around
 *   height       — how high its ring sits
 *   orbit_angle  — where it currently is on the ring
 *   pos          — its actual spot, worked out from the four above
 */
typedef struct {
  Vec3 color;         /* its colour                            */
  float orbit_radius; /* how wide its ring is (world units)    */
  float orbit_speed;  /* how fast it circles (radians/second)  */
  float height;       /* how high its ring sits (world units)  */
  float orbit_angle;  /* where it is on the ring right now      */
  Vec3 pos;           /* its actual spot, from the four above   */
} PointLight;

/* §7.2 ── one light's contribution at one pixel ───────────────────────── */

/* Works out how much one light brightens one surface point: more where the
 * surface faces the light (soft light), plus a shiny highlight where the angle
 * lines up with the eye. The surface colour tints the soft light but not the
 * highlight — a highlight reflects the light's own colour. The classic
 * Blinn-Phong model. */
static Vec3 blinn_phong(Vec3 P, Vec3 N, Vec3 albedo, Vec3 light_pos,
                        Vec3 light_col, Vec3 cam_pos) {
  Vec3 L = v3_norm(v3_sub(light_pos, P));
  Vec3 V = v3_norm(v3_sub(cam_pos, P));
  Vec3 H = v3_norm(v3_add(L, V));

  float diff = fmaxf(0.f, v3_dot(N, L));
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), SHININESS);

  return v3(albedo.x * light_col.x * diff + light_col.x * spec * SPEC_GAIN,
            albedo.y * light_col.y * diff + light_col.y * spec * SPEC_GAIN,
            albedo.z * light_col.z * diff + light_col.z * spec * SPEC_GAIN);
}

/* §7.3 ── add up every light across the screen ────────────────────────── */

/* The colour of one surface cell: the little base light, plus every active
 * light's contribution added on top. Notice this never touches the triangles —
 * it only reads the per-pixel tables, so an extra light is just one more trip
 * around the inner loop, not another whole redraw. */
static Vec3 shade_pixel(const GBuffer *gb, int r, int c,
                        const PointLight *lights, int n_lights, Vec3 cam_pos,
                        Vec3 ambient) {
  Vec3 P = gb->pos[r][c];
  Vec3 N = gb->normal[r][c];
  Vec3 albedo = gb->albedo[r][c];

  Vec3 lit =
      v3(ambient.x * albedo.x, ambient.y * albedo.y, ambient.z * albedo.z);

  for (int li = 0; li < n_lights; li++) {
    Vec3 contrib =
        blinn_phong(P, N, albedo, lights[li].pos, lights[li].color, cam_pos);
    lit.x += contrib.x;
    lit.y += contrib.y;
    lit.z += contrib.z;
  }

  return v3(fminf(1.f, lit.x), fminf(1.f, lit.y), fminf(1.f, lit.z));
}

static void render_lightpass(GBuffer *gb, const PointLight *lights,
                             int n_lights, Vec3 cam_pos, int cols, int rows) {
  Vec3 ambient = v3(AMBIENT_STR, AMBIENT_STR, AMBIENT_STR * AMBIENT_BLUE_BIAS);

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c]) {
        gb->light[r][c] = v3(0, 0, 0); /* empty background — nothing to light */
        continue;
      }
      gb->light[r][c] =
          shade_pixel(gb, r, c, lights, n_lights, cam_pos, ambient);
    }
  }
}

/* ── §8 scene — the sphere, the orbiting lights, the camera ────────────── */

/* §8.1 ── the view mode, the camera, the scene ────────────────────────── */

/*
 * GBufMode — which table the 'g' key is currently showing. The first three are
 * debugging views (and they prove the point: adding lights never changes them);
 * the fourth is the real lit picture. They're numbered from 0 in a row so the
 * 'g' key can step through them and so they double as labels (k_mode_names).
 */
typedef enum {
  MODE_POSITION = 0, /* the depth table as a near→far colour gradient   */
  MODE_NORMAL,       /* the facing table shown as colour                */
  MODE_ALBEDO,       /* the plain surface colour, no lighting           */
  MODE_LIGHTING,     /* the real lit result (pass 2's output)           */
  MODE_COUNT,        /* how many modes there are (for cycling)          */
} GBufMode;

/* HUD label per mode, indexed by GBufMode (excludes the MODE_COUNT sentinel). */
static const char *k_mode_names[MODE_COUNT] = {
    "POSITION",
    "NORMAL",
    "ALBEDO",
    "LIGHTING",
};

/* §8.2 ── the 8 light presets: 3 main + 5 you add with 'l'.
 *
 * The first three are pure red, green, and blue — the three primary colours of
 * light, the classic three-flashlights demo. They share one ring but start a
 * third of the way apart, so they keep an even triangle as they circle. On the
 * white sphere you see a red pool, a green pool, and a blue pool slide around;
 * where two overlap you get yellow / cyan / magenta, and where all three meet,
 * white. That's the lighting pass made visible: each pixel = the sum of the
 * lights hitting it.
 *
 * The other five (yellow, cyan, magenta, white, orange) are there for the cost
 * story — press 'l' to switch them on one at a time. The surface tables never
 * get redrawn; only the adding-up grows.
 */
static const struct {
  Vec3 color;                              /* this light's colour            */
  float orbit_radius, orbit_speed, height; /* its ring: width, speed, height */
  float angle_start;                       /* where it starts on the ring    */
} LIGHT_PRESETS[MAX_LIGHTS] = {
    /* the three primaries — same ring, evenly spaced, so the pools always sit
     * in a clear triangle as they sweep */
    {{1.00f, 0.00f, 0.00f}, 1.55f, 0.45f, 0.40f, 0.0000f}, /* RED   @ 0°  */
    {{0.00f, 1.00f, 0.00f},
     1.55f,
     0.45f,
     0.40f,
     2.0944f}, /* GREEN @ 120° (2π/3) */
    {{0.00f, 0.00f, 1.00f},
     1.55f,
     0.45f,
     0.40f,
     4.1888f}, /* BLUE  @ 240° (4π/3) */

    /* the extras — 'l' switches them on one at a time, each on a slightly
     * different ring so they don't overlap; drawing cost stays fixed */
    {{1.00f, 1.00f, 0.00f}, 1.30f, 0.65f, -0.30f, 1.000f}, /* YELLOW   */
    {{0.00f, 1.00f, 1.00f}, 1.30f, 0.65f, -0.30f, 3.000f}, /* CYAN     */
    {{1.00f, 0.00f, 1.00f}, 1.30f, 0.65f, -0.30f, 5.000f}, /* MAGENTA  */
    {{1.00f, 1.00f, 1.00f}, 1.80f, 0.30f, 1.20f, 0.500f},  /* WHITE    */
    {{1.00f, 0.55f, 0.00f}, 1.20f, 0.80f, 0.00f, 2.500f},  /* ORANGE   */
};

/*
 * Camera — the eye, plus the transforms worked out from it. The eye sits still
 * looking at the sphere; only `dist` (the zoom) changes, with +/-. `pos` and
 * `view` are recomputed from `dist`, and `proj` from the window shape, so they
 * just need rebuilding once on a zoom rather than every frame.
 *   dist — how far back the eye is; the +/- keys change it
 *   pos  — the eye's actual spot, from dist
 *   view — the "look from the eye toward the sphere" transform
 *   proj — the perspective (makes far things smaller), matched to the window
 */
typedef struct {
  Mat4 view;
  Mat4 proj;
  Vec3 pos;
  float dist;
} Camera;

/*
 * Scene — everything the demo is about, in one place. The drawing passes read it
 * but never change it. It answers: what's drawn, how it's lit, from where, plus
 * a little display state.
 *
 * The objects are held in three side-by-side arrays read together: object i is
 * mesh meshes[i], colour albedos[i], placed by models[i]. (There's only one
 * object — the sphere — but the arrays make adding more painless.)
 */
typedef struct {
  /* what's drawn — three side-by-side arrays, one entry per object */
  Mesh meshes[MAX_OBJECTS];  /* each object's shape   */
  Vec3 albedos[MAX_OBJECTS]; /* each object's colour  */
  Mat4 models[MAX_OBJECTS];  /* where each one sits   */
  int n_objects;             /* how many (just 1 here) */

  /* how it's lit — the lights added up in pass 2 */
  PointLight lights[MAX_LIGHTS]; /* all 8 ready; only the first n_lights are on */
  int n_lights;                  /* how many are on; 'l' steps it 3 → 8         */

  /* from where */
  Camera cam;

  /* display state */
  GBufMode mode;  /* which table is shown ('g' cycles)                 */
  bool paused;    /* space freezes the light orbits                    */
  int scene_cols; /* how many cells wide the scene gets                */
  int scene_rows; /* how many tall (the rest is the HUD at the bottom) */
} Scene;

/* §8.3 ── setting up and advancing the scene ──────────────────────────── */

/* Rebuilds the perspective for a new window size so the sphere isn't stretched
 * (terminal cells are taller than they are wide, which it accounts for). */
static void camera_rebuild_proj(Camera *cam, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  cam->proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

/* Moves the eye in/out for the new zoom and refreshes its look-from transform,
 * so the next frame (both the drawing and the highlights) uses the new spot. */
static void camera_set_zoom(Camera *cam) {
  cam->pos = v3(0.f, CAM_EYE_Y, cam->dist);
  cam->view = m4_lookat(cam->pos, v3(0, CAM_LOOK_Y, 0), v3(0, 1, 0));
}

/* Changes the zoom by delta, kept within range. delta < 0 moves closer. */
static void camera_zoom(Camera *cam, float delta) {
  cam->dist += delta;
  if (cam->dist < CAM_DIST_MIN)
    cam->dist = CAM_DIST_MIN;
  if (cam->dist > CAM_DIST_MAX)
    cam->dist = CAM_DIST_MAX;
  camera_set_zoom(cam);
}

/* Copies every preset into a live light — including the ones that are off — so
 * pressing 'l' later just turns the next one on without disturbing the others
 * mid-animation. */
static void seed_lights(Scene *s) {
  for (int li = 0; li < MAX_LIGHTS; li++) {
    PointLight *l = &s->lights[li];
    l->color = LIGHT_PRESETS[li].color;
    l->orbit_radius = LIGHT_PRESETS[li].orbit_radius;
    l->orbit_speed = LIGHT_PRESETS[li].orbit_speed;
    l->height = LIGHT_PRESETS[li].height;
    l->orbit_angle = LIGHT_PRESETS[li].angle_start;
  }
}

static void scene_init(Scene *s, int total_cols, int total_rows) {
  /* Free meshes from previous scene (reset / resize). */
  for (int i = 0; i < s->n_objects; i++)
    mesh_free(&s->meshes[i]);

  memset(s, 0, sizeof *s);
  s->scene_cols = total_cols;
  s->scene_rows = total_rows - HUD_ROWS;
  s->mode = MODE_LIGHTING;

  /* Camera — pose is fixed except for +/- zoom. cam.dist lives in
   * Scene so the user can move the eye at runtime; camera_set_zoom
   * derives cam.pos and the view matrix from it. */
  s->cam.dist = CAM_DIST;
  camera_set_zoom(&s->cam);
  camera_rebuild_proj(&s->cam, total_cols, s->scene_rows);

  /* the one object: a white sphere at the centre */
  s->meshes[0] = tessellate_sphere(BALL_RADIUS, BALL_RINGS, BALL_SEGS);
  s->albedos[0] = v3(1.0f, 1.0f, 1.0f); /* pure white */
  s->models[0] = m4_identity();         /* at the centre, no rotation */
  s->n_objects = 1;

  /* the three primaries on by default; 'l' turns on more */
  s->n_lights = 3;
  seed_lights(s);
}

/* Works out where a light is on its ring, given how far around it's gone. */
static inline Vec3 orbit_position(float radius, float angle, float height) {
  return v3(radius * cosf(angle), height, radius * sinf(angle));
}

/* The only thing that moves: nudge each light a bit further around its ring. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  for (int li = 0; li < MAX_LIGHTS; li++) {
    PointLight *l = &s->lights[li];
    l->orbit_angle += l->orbit_speed * dt;
    l->pos = orbit_position(l->orbit_radius, l->orbit_angle, l->height);
  }
}

/* ── §9 screen — pick a colour per cell, paint it, draw the HUD ────────── */

/* §9.1 ── which colour to show for the active mode ────────────────────── */

/* Turns the chosen table into a colour at one cell; paint_cell (§4) then turns
 * that colour into an actual character. Position shows a near→far gradient,
 * normal shows the facing as colour, albedo is the plain colour, lighting is the
 * real lit result. */

/* Remaps the depth (near…far) to a plain 0..1 number. */
static float ndc_depth_to_unit(float z) {
  float t = (z + 1.f) * 0.5f;
  return t < 0.f ? 0.f : (t > 1.f ? 1.f : t);
}

/* A hand-tuned warm-near → cool-far colour ramp over 0..1: reds fade out, blue
 * rises, green peaks in the middle. */
static Vec3 depth_gradient_rgb(float t) {
  return v3(0.95f - t * 0.7f, 0.55f - t * 0.3f + (1.f - t) * 0.2f,
            0.30f + t * 0.65f);
}

static Vec3 mode_to_rgb(const GBuffer *gb, GBufMode mode, int r, int c) {
  if (!gb->valid[r][c])
    return v3(0, 0, 0);

  switch (mode) {
  case MODE_POSITION:
    return depth_gradient_rgb(ndc_depth_to_unit(gb->zbuf[r][c]));
  case MODE_NORMAL: {
    /* show the facing direction as a colour */
    Vec3 N = gb->normal[r][c];
    return v3(N.x * 0.5f + 0.5f, N.y * 0.5f + 0.5f, N.z * 0.5f + 0.5f);
  }
  case MODE_ALBEDO:
    return gb->albedo[r][c];
  case MODE_LIGHTING:
    return gb->light[r][c];
  default:
    return v3(0, 0, 0);
  }
}

/* §9.2 ── painting the scene and the HUD ──────────────────────────────── */

/* Paints every cell: ask for its colour in the current mode, hand it to
 * paint_cell. Adding lights changes only the lighting table — the position,
 * normal, and albedo views look exactly the same, which is the demo's whole
 * point, plainly visible. */
static void render_scene(const Scene *s, const GBuffer *gb) {
  int cols = s->scene_cols;
  int rows = s->scene_rows;

  for (int r = 0; r < rows && r < GBUF_MAX_H; r++) {
    for (int c = 0; c < cols && c < GBUF_MAX_W; c++) {
      if (!gb->valid[r][c])
        continue;
      Vec3 col = mode_to_rgb(gb, s->mode, r, c);
      paint_cell(c, r, col);
    }
  }
}

/* Finds the closest of our 216 colours to this RGB — used by the HUD to draw
 * each light's swatch in its own colour. */
static int cube_pair(Vec3 col) {
  return PAIR_CUBE_BASE + quantize_unit_to_5(col.x) * 36 +
         quantize_unit_to_5(col.y) * 6 + quantize_unit_to_5(col.z);
}

/* Draws the overlay: a yellow status line on top, three teaching lines plus the
 * coloured light dots near the bottom, and a cyan key reminder on the last row. */

/* count_total_triangles — sum of triangle counts across all live objects. */
static int count_total_triangles(const Scene *s) {
  int total = 0;
  for (int i = 0; i < s->n_objects; i++)
    total += s->meshes[i].ntri;
  return total;
}

/* mode_explanation — one-line teaching caption for the active G-buffer view. */
static const char *mode_explanation(GBufMode mode) {
  switch (mode) {
  case MODE_POSITION:
    return "POSITION: warm-near → cool-far depth. 3-D layout BEFORE lighting.";
  case MODE_NORMAL:
    return "NORMAL: (N+1)/2 RGB. Smooth sphere → smooth gradient.";
  case MODE_ALBEDO:
    return "ALBEDO: flat surface colour, NO lighting. 'l' never changes this.";
  case MODE_LIGHTING:
    return "LIGHTING: Blinn-Phong over G-buffer. 'l' adds a light, geometry "
           "stays.";
  default:
    return "";
  }
}

/* hud_draw_swatches — one '@' per active light at `row`, each painted in
 * that light's own cube colour (red light → red glyph, etc.). */
static void hud_draw_swatches(const Scene *s, int row) {
  int cols = s->scene_cols;
  int lx = 9;
  for (int li = 0; li < s->n_lights && lx < cols - 2; li++) {
    int pair = cube_pair(s->lights[li].color);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(row, lx, (chtype)(unsigned char)'@');
    attroff(COLOR_PAIR(pair) | A_BOLD);
    lx += 2;
  }
}

static void hud_draw(const Scene *s, double fps) {
  int hr = s->scene_rows; /* first HUD row = first row past scene */
  int cols = s->scene_cols;

  int total_tris = count_total_triangles(s);
  int fwd_calls = s->n_objects * s->n_lights;   /* forward:  obj × lights */
  int defer_calls = s->n_objects + s->n_lights; /* deferred: obj + lights */
  int saved_pct =
      fwd_calls > 0 ? 100 * (fwd_calls - defer_calls) / fwd_calls : 0;

  /* ── Row 0: title left, status right (yellow + bold per spec). ── */
  char status[120];
  snprintf(status, sizeof status,
           " %5.1f fps  mode:%s  lights:%d  zoom:%.1f  tris:%d  %s ", fps,
           k_mode_names[s->mode], s->n_lights, (double)s->cam.dist, total_tris,
           s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > cols)
    slen = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - slen, "%s", status);
  mvprintw(0, 0, " DEFERRED · RGB ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── Educational rows (yellow, no bold so they sit under the
   *    primary status row visually). ── */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(hr + 0, 1,
           "fwd = %d (obj × lights)   def = %d (obj + lights)   saved = %d%%",
           fwd_calls, defer_calls, saved_pct);
  mvprintw(hr + 1, 1, "%s", mode_explanation(s->mode));
  mvprintw(hr + 2, 1, "Lights:");
  attroff(COLOR_PAIR(PAIR_HUD));

  hud_draw_swatches(s, hr + 2);

  /* ── Cyan hint bottom row (per spec: A_BOLD, never A_DIM). ── */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hr + HUD_ROWS - 1, 0,
           " q:quit  spc:pause  g:layer  l:add-light  +/-:zoom  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10 app — setup, the main loop, and keypresses ────────────────────── */

/*
 * App — the whole running program: the scene plus the terminal size and two
 * flags the signal handlers set. One shared instance (g_app) so the handlers,
 * which take no arguments, can still reach it.
 *
 * running / need_resize are marked volatile sig_atomic_t because a signal can
 * set them at any moment, so the compiler must always read/write them for real.
 * total_cols/rows are the full terminal; the scene gets all but the few HUD rows
 * at the bottom, recomputed on every resize.
 */
typedef struct {
  Scene scene;                       /* everything drawn                   */
  int total_cols;                    /* full terminal width  (cells)      */
  int total_rows;                    /* full terminal height (cells)      */
  volatile sig_atomic_t running;     /* cleared to stop the loop          */
  volatile sig_atomic_t need_resize; /* set when the window was resized   */
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
  case 'g':
  case 'G':
    s->mode = (GBufMode)((s->mode + 1) % MODE_COUNT);
    break;
  case 'l':
  case 'L':
    s->n_lights = (s->n_lights >= MAX_LIGHTS) ? 1 : s->n_lights + 1;
    break;
  case '=':
  case '+':
    camera_zoom(&s->cam, -CAM_ZOOM_STEP); /* closer */
    break;
  case '-':
  case '_':
    camera_zoom(&s->cam, +CAM_ZOOM_STEP); /* farther */
    break;
  default:
    break;
  }
  return true;
}

/* Draws one whole frame: pass 1 fills the surface tables, pass 2 adds up the
 * lights, pass 3 paints the chosen table, then the HUD goes on top. Reads the
 * scene, writes only the tables and the terminal. */
static void render_frame(const Scene *s, double fps) {
  erase();
  render_gbuffer(&g_gbuf, s->meshes, s->albedos, s->models, s->n_objects,
                 &s->cam.view, &s->cam.proj, s->scene_cols, s->scene_rows);
  render_lightpass(&g_gbuf, s->lights, s->n_lights, s->cam.pos, s->scene_cols,
                   s->scene_rows);
  render_scene(s, &g_gbuf);
  hud_draw(s, fps);
  screen_present();
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

    /* move the lights along */
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

    /* draw everything */
    render_frame(&app->scene, fps_display);

    /* handle a keypress (pause / reset / mode / lights / zoom / quit) */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  for (int i = 0; i < app->scene.n_objects; i++)
    mesh_free(&app->scene.meshes[i]);

  endwin();
  return 0;
}
