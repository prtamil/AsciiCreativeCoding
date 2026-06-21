/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher.c — one lit sphere drawn with text in the terminal. For each
 * character on screen we fire a ray from the eye and creep it forward until it
 * touches the sphere, then shade that spot from dark to bright.
 *
 * Simplest demo in this folder; the siblings (raymarcher_cube, kifs_fractal,
 * mandelbulb, metaballs) reuse this skeleton with a fancier shape, so read it
 * first. Ideas borrowed: Hart's "Sphere Tracing" (1996) for the creeping ray,
 * Phong (1975) for the lighting, and Iñigo Quílez's shape cookbook at
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

/* §1  config — all the tunable numbers live here */

/* §1.1 frame rate. */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  FPS_UPDATE_MS = 500, /* how often to refresh the fps readout (ms) */
};

/* §1.2 one canvas pixel = one terminal cell.  (The aspect fix that keeps the
 * sphere round lives in the camera, §9.) */
#define CELL_W 1
#define CELL_H 1
#define CELL_ASPECT 2.0f /* terminal cells are ~2× taller than wide */

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* §1.3 ray-march tuning.
 *   RM_MAX_STEPS  give up after this many steps along one ray
 *   RM_HIT_EPS    how close counts as "touched the surface"
 *   RM_MAX_DIST   if a ray gets this far out, it hit nothing */
#define RM_MAX_STEPS 80
#define RM_HIT_EPS 0.002f
#define RM_MAX_DIST 20.0f

/* §1.4 camera (zoom). */
#define CAM_Z_DEFAULT 4.0f
#define CAM_Z_MIN 2.0f /* nearest zoom — keeps the eye outside the sphere */
#define CAM_Z_MAX 12.0f
#define CAM_ZOOM_STEP 0.30f
#define FOV_HALF_TAN 0.7f /* how wide the lens sees; bigger = wider view (~70°) */

/* §1.5 sphere radius. */
#define SPHERE_R_DEFAULT 1.1f
#define SPHERE_R_STEP 1.15f
#define SPHERE_R_MIN 0.2f
#define SPHERE_R_MAX 3.0f

/* §1.6 light orbit speed (radians/sec). */
#define LIGHT_SPD_DEFAULT 0.8f
#define LIGHT_SPD_STEP 1.35f
#define LIGHT_SPD_MIN 0.02f
#define LIGHT_SPD_MAX 8.0f

/* §1.7 the light's looping path: it circles in x, bobs up and down in y, and
 * sits at a fixed depth (z). */
#define LIGHT_RADIUS_X 3.0f
#define LIGHT_BIAS_Y 1.5f
#define LIGHT_AMPLITUDE_Y 1.0f
#define LIGHT_RATE_Y 0.7f
#define LIGHT_HEIGHT_Z 2.5f

/* §1.8 how much each kind of light counts when shading a spot. Tuned for chunky
 * text cells: the dim side still fades smoothly instead of going flat-black, and
 * the shiny highlight is soft so it doesn't flicker cell-to-cell. See §8. */
#define KA 0.10f   /* a little glow everywhere, so nothing is pure black */
#define KD 0.80f   /* brightness from facing the light                   */
#define KS 0.25f   /* shiny-highlight strength (low so it never blows out white) */
#define SHIN 14.0f /* highlight tightness — smaller spreads it into a soft spot  */

/* §1.9 ncurses colour-pair slots. */
enum {
  LUMI_N = 8,             /* 8 pairs hold the brightness ramp    */
  PAIR_HUD = LUMI_N + 1,  /* yellow status line (top)            */
  PAIR_HINT = LUMI_N + 2, /* cyan key reminders (bottom)         */
};

/* the shading characters, dark (space) to bright (@). */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1))

/* §1.10 colour themes: a name plus 8 colour codes running dark→bright.  t/T
 * swaps which one is active; the geometry never changes.  (Kept in the bright
 * half of the 256-colour set so even the dimmest slot stays visible.) */
typedef struct {
  const char *display_name;
  short ramp_256[LUMI_N]; /* 8 xterm-256 codes, dark to bright */
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

/* §1.11 alternate views, cycled with d / D — handy for seeing what the
 * renderer is actually computing. */
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit sphere               */
  DEBUG_NORMALS = 1, /* colour by which way each surface faces      */
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

/* §1.12 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define RENDER_FPS_CAP 60 /* frames/sec the draw loop is paced to */
/* Clamp a slow frame (resize, stall) to this so the fixed-timestep
 * accumulator can't spiral trying to catch up. */
#define MAX_FRAME_NS (100 * NS_PER_MS)

/* §2  reading the clock and sleeping. We use the monotonic clock because it only
 * ever counts forward — it won't jump if someone changes the system time. */

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

/* §3  colour setup: hand ncurses the current theme's 8 shades plus the two HUD
 * colours. Terminals with fewer than 256 colours fall back to plain white and
 * fake brighter/dimmer with bold and dim instead. */

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

/* §4  vec3 — a 3-D point or direction (x, y, z) and the usual math on it. */

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

/* bounce direction v off a surface that faces way n (n must be unit length) —
 * used to find where light reflects toward the eye. */
static inline Vec3 v3reflect(Vec3 v, Vec3 n) {
  return v3sub(v3mul(n, 2.0f * v3dot(n, v)), v);
}

/* squeeze a value into the 0..1 range — brightness has to land there before we
 * can pick a character or colour for it. */
static inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/* §5  how far point p is from the sphere's surface: negative inside, zero on
 * it, positive outside.  This "distance to the nearest surface" is the one
 * thing the ray march needs from a shape. */
static float sdf_sphere(Vec3 p, float radius) { return v3len(p) - radius; }

/* §6  ray marching — the heart of the demo. */

/* the spot you reach by walking distance t along the ray from origin */
static inline Vec3 ray_at(Vec3 origin, Vec3 dir, float t) {
  return v3add(origin, v3mul(dir, t));
}

/* Creep along the ray toward the sphere. Each step jumps forward by the distance
 * to the surface — always safe, since nothing is closer than that. Stop once
 * we're basically touching it; give up if the ray flies off into nothing.
 * Returns the distance to the hit (or -1 for a miss) and, for the STEPS view,
 * how many steps it took — pass NULL if you don't need that. (Hart, 1996.) */
static float sphere_trace(Vec3 origin, Vec3 dir, float radius, int *out_steps) {
  float t = 0.0f;
  int step;
  for (step = 0; step < RM_MAX_STEPS; step++) {
    Vec3 p = ray_at(origin, dir, t);
    float d = sdf_sphere(p, radius);
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

/* §7  which way the surface faces at the hit — the "normal", which the lighting
 * needs. On a sphere that's just the direction from the centre out to the point,
 * so there's nothing to compute. (Other shapes have to estimate it.) */
static Vec3 sphere_normal(Vec3 p) { return v3norm(p); }

/* §8  shading — turn one surface spot into a brightness from 0 (black) to 1
 * (full). It's the sum of three classic pieces: a faint everywhere-glow, how
 * squarely the spot faces the light, and a shiny highlight. (Phong, 1975.) */

/* How squarely the spot faces the light (ndl), but "wrapped" so the far side
 * fades out gently instead of snapping to black — otherwise the whole back of
 * the sphere would collapse to one flat dark character, and here the character
 * IS the shape. (Half-Lambert, Valve/Mitchell 2006.) */
static float half_lambert(float ndl) {
  float wrap = ndl * 0.5f + 0.5f;
  return wrap * wrap;
}

/* The shiny highlight — a bright dot where the light bounces straight back at the
 * eye. Only on the lit side (ndl > 0), so it can't show up where light can't reach. */
static float specular_term(Vec3 N, Vec3 to_light, Vec3 to_eye, float ndl) {
  if (ndl <= 0.0f)
    return 0.0f;
  Vec3 reflected = v3reflect(to_light, N);
  return powf(fmaxf(0.0f, v3dot(reflected, to_eye)), SHIN);
}

static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light) {
  Vec3 to_light = v3norm(v3sub(light, hit));
  Vec3 to_eye = v3norm(v3sub(cam, hit));
  float ndl = v3dot(N, to_light);

  float ambient = KA;
  float diffuse = KD * half_lambert(ndl);
  float specular = KS * specular_term(N, to_light, to_eye, ndl);

  return clamp01(ambient + diffuse + specular);
}

/* §9  the camera and one cell's whole trip: aim a ray, march it, and bottle up
 * what it found in a Hit. Each cell is traced just once — the normal view and
 * all the debug views read back from the same Hit. */

/* Camera — a fixed eye that sits back on the z axis looking at the middle. Built
 * once per frame so the lens and the squash aren't redone for every cell. */
typedef struct {
  Vec3 origin;       /* where the eye sits */
  float fov_t;       /* how wide the lens sees */
  float phys_aspect; /* squash that cancels out tall, narrow text cells */
} Camera;

static Camera camera_for_canvas(int canvas_w, int canvas_h, float cam_z) {
  return (Camera){
      .origin = v3(0.0f, 0.0f, cam_z),
      .fov_t = FOV_HALF_TAN,
      .phys_aspect = ((float)canvas_h * CELL_ASPECT) / (float)canvas_w,
  };
}

/* Hit — what one ray found at one cell. Filled once and stored so every view can
 * read whatever it needs without tracing the ray again. When hit is false, the
 * rest of the fields are meaningless. */
typedef struct {
  bool hit;             /* did the ray reach the sphere at all? */
  Vec3 hit_point;       /* where on the surface it landed */
  Vec3 normal;          /* which way the surface faces there */
  float intensity;      /* brightness 0..1, for the normal view */
  float trace_distance; /* how far the ray travelled to get there (DEPTH view) */
  int step_count;       /* how many creep-steps it took (STEPS view) */
} Hit;

/* Which way to fire the ray for one cell: turn the cell's spot on screen into an
 * aim direction. Top row is row 0, and the squash keeps tall text cells from
 * stretching the sphere into an egg. */
static Vec3 ray_through_pixel(int col, int row, int canvas_w, int canvas_h,
                              const Camera *cam) {
  float u = ((float)col + 0.5f) / (float)canvas_w * 2.0f - 1.0f;
  float v = -((float)row + 0.5f) / (float)canvas_h * 2.0f + 1.0f;
  return v3norm(v3(u * cam->fov_t, v * cam->fov_t * cam->phys_aspect, -1.0f));
}

static Hit cast_ray(int col, int row, int canvas_w, int canvas_h,
                    float sphere_radius, Vec3 light, const Camera *cam) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0};

  Vec3 rd = ray_through_pixel(col, row, canvas_w, canvas_h, cam);

  int steps = 0;
  float t = sphere_trace(cam->origin, rd, sphere_radius, &steps);
  h.step_count = steps;

  if (t < 0.0f)
    return h; /* ray missed the sphere */

  h.hit = true;
  h.trace_distance = t;
  h.hit_point = ray_at(cam->origin, rd, t);
  h.normal = sphere_normal(h.hit_point);
  h.intensity = phong(h.hit_point, h.normal, cam->origin, light);
  return h;
}

/* §10  Canvas — one Hit per cell, the whole picture for this frame. Refilled
 * every frame before it's drawn. The memory is owned here: allocated at startup
 * and remade on resize (canvas_alloc / canvas_free). */
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

/* §11  fill the buffer: set up the camera once, then trace a ray for every cell.
 * Doesn't know about characters or colour yet — that's the next section. */
static void canvas_render(Canvas *c, float sphere_radius, Vec3 light,
                          float cam_z) {
  Camera cam = camera_for_canvas(c->w, c->h, cam_z);
  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      c->hits[py * c->w + px] =
          cast_ray(px, py, c->w, c->h, sphere_radius, light, &cam);
    }
  }
}

/* §12  the normal view: a spot's brightness picks both its character and its
 * colour. */

/* pick the character for a brightness, from the dark-to-bright ramp */
static char intensity_to_glyph(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= RAMP_LEN)
    idx = RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

/* pick the colour for a brightness, from the 8-shade ramp */
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

/* draw one canvas pixel. It's a single cell here, but we loop over a CELL_W×CELL_H
 * block (clipped to the screen) so the sibling demos can use chunkier pixels. */
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

/* §13  the debug views (d/D): instead of shading, paint one raw fact from each
 * Hit so you can see what the tracer actually found — which way the surface
 * faces, how near it is, or how hard the ray had to work. */

/* which way around (like a compass heading) the surface faces, as 0..1 for colour */
static float normal_hue(Vec3 N) {
  return atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
}

/* how much the surface faces up, as 0..1 for brightness */
static float normal_upness(Vec3 N) { return clamp01(N.y * 0.5f + 0.5f); }

/* nearer the camera = brighter, scaled across the sphere's near-to-far span */
static float depth_to_brightness(float trace_distance, float cam_z) {
  float t_min = cam_z - SPHERE_R_MAX;
  float t_max = cam_z + SPHERE_R_MAX;
  if (t_min < 0.0f)
    t_min = 0.0f;
  return clamp01((t_max - trace_distance) / (t_max - t_min));
}

/* more steps = brighter; rays grazing the edge work hardest, so the rim glows */
static float steps_to_brightness(int step_count) {
  return clamp01((float)step_count / (float)RM_MAX_STEPS);
}

static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      char glyph = intensity_to_glyph(normal_upness(h->normal));
      attr_t attr = intensity_to_attr(normal_hue(h->normal));
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

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float depth_n = depth_to_brightness(h->trace_distance, cam_z);
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

      float steps_n = steps_to_brightness(h->step_count);
      char glyph = intensity_to_glyph(steps_n);
      attr_t attr = intensity_to_attr(steps_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* §14  all the program's state lives in Scene. Only scene_tick and the
 * key/resize handlers ever change it; everything else just reads it. */

/* Scene — all program state, as a table of contents:
 *   WHAT is shown    sphere_r      the sphere's radius (+/-)
 *   HOW it's driven   light_spd     light-orbit speed (]/[)
 *                     cam_z         camera distance / zoom (z/Z)
 *                     theme_index   colour palette (t/T)
 *                     debug_mode    which view (d/D)
 *                     paused        freezes the light orbit (space)
 *   WHERE/when        time          animation clock
 *   render buffer     canvas        one Hit per cell, refilled each frame
 *                                   (owned by §11 render, not sim state) */
typedef struct {
  float sphere_r;

  float light_spd;
  float cam_z;
  int theme_index;
  DebugMode debug_mode;
  bool paused;

  float time;

  Canvas canvas;
} Scene;

/* the light's position right now: it circles in x, bobs up and down in y,
 * and stays at a fixed depth — a looping path that never quite repeats. */
static Vec3 scene_light(const Scene *s) {
  float t = s->time * s->light_spd;
  return v3(LIGHT_RADIUS_X * cosf(t),
            LIGHT_BIAS_Y + LIGHT_AMPLITUDE_Y * sinf(LIGHT_RATE_Y * t),
            LIGHT_HEIGHT_Z);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  canvas_alloc(&s->canvas, cols, rows);
  s->time = 0.0f;
  s->light_spd = LIGHT_SPD_DEFAULT;
  s->sphere_r = SPHERE_R_DEFAULT;
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

static void scene_tick(Scene *s, float dt_sec) {
  if (!s->paused)
    s->time += dt_sec;
}

static void scene_render(Scene *s) {
  canvas_render(&s->canvas, s->sphere_r, scene_light(s), s->cam_z);
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

/* §15  screen — start/stop ncurses, draw the HUD, and track the terminal size. */

/* Screen — the terminal's current size in characters, refreshed on every resize. */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE); /* getch() returns at once if no key is waiting */
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
           " %5.1f fps  spd:%.2f  r:%.2f  zoom:%.2f  theme:%s  "
           "debug:%s  [%dx%d]  %s ",
           fps, sc->light_spd, sc->sphere_r, sc->cam_z,
           THEMES[sc->theme_index].display_name,
           DEBUG_MODE_NAMES[sc->debug_mode], sc->canvas.w, sc->canvas.h,
           sc->paused ? "PAUSED" : "running");
}

/* Draw the frame: the sphere (or a debug view), then the two HUD rows —
 * title + status on top, key reminders on the bottom. */
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
  mvprintw(0, 0, " RAYMARCHER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  ]/[:light-speed  +/-:size  z/Z:zoom  "
           "t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §16  input + the main loop.  Keys and resize change the state between
 * frames; main() runs everything each frame, in order (apply input → step the
 * sim → draw → pace), and holds the frame rate. */

/* Everything the running program owns. The last two are flipped from inside
 * signal handlers, so they're volatile and acted on between frames, not mid-draw. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                       /* how many times a second the light steps */
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
    s->light_spd *= LIGHT_SPD_STEP;
    if (s->light_spd > LIGHT_SPD_MAX)
      s->light_spd = LIGHT_SPD_MAX;
    break;
  case '[':
    s->light_spd /= LIGHT_SPD_STEP;
    if (s->light_spd < LIGHT_SPD_MIN)
      s->light_spd = LIGHT_SPD_MIN;
    break;

  case '=':
  case '+':
    s->sphere_r *= SPHERE_R_STEP;
    if (s->sphere_r > SPHERE_R_MAX)
      s->sphere_r = SPHERE_R_MAX;
    break;
  case '-':
    s->sphere_r /= SPHERE_R_STEP;
    if (s->sphere_r < SPHERE_R_MIN)
      s->sphere_r = SPHERE_R_MIN;
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

/* Work out the fps number to show. Only recomputes every half-second or so, so
 * the readout holds still long enough to read. Returns the same value between
 * updates and bumps the two running totals it's handed. */
static double update_fps(int64_t *fps_accum, int *frame_count, int64_t dt,
                         double current) {
  (*frame_count)++;
  *fps_accum += dt;
  if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS)
    return current;
  double fps = (double)*frame_count / ((double)*fps_accum / (double)NS_PER_SEC);
  *frame_count = 0;
  *fps_accum = 0;
  return fps;
}

/* Sleep away whatever time is left in this frame's budget, so the loop runs at a
 * steady speed no matter how fast the drawing was. */
static void pace_frame(int64_t frame_time, int64_t dt) {
  int64_t target_ns = NS_PER_SEC / RENDER_FPS_CAP;
  int64_t elapsed = clock_ns() - frame_time + dt;
  clock_sleep_ns(target_ns - elapsed);
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

    if (app->need_resize) { /* 1. EVENTS — apply a pending resize */
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* 2. PERFORMANCE — measure frame dt, clamp against a stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > MAX_FRAME_NS)
      dt = MAX_FRAME_NS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    /* 3. SIMULATION — advance whole fixed-timestep ticks */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    scene_render(&app->scene); /* 4. RENDER — fill the Hit buffer */

    /* 5. PERFORMANCE — refresh the fps readout, then hold the frame cap */
    fps_display = update_fps(&fps_accum, &frame_count, dt, fps_display);
    pace_frame(frame_time, dt);

    screen_draw(&app->screen, &app->scene, fps_display); /* 6. RENDER — present */
    screen_present();

    int ch = getch(); /* 7. EVENTS — drain one key */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
