/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kifs_fractal.c — three 3-D fractals you can fly around in the terminal.
 * Each is built by taking a point in space, folding it across a few mirror
 * planes, then shrinking it toward a fixed point — over and over.  Doing
 * that lets us measure roughly how far any point is from the fractal, which
 * is exactly what a ray-marching renderer needs to draw it.
 *
 * Three presets share one renderer; only the fold differs: TETRA (a
 * Sierpinski tetrahedron), MENGER (a Menger sponge), and KIFS_ROT (an
 * animated rotating crystal).  Keys are listed on the bottom HUD line.
 *
 * Sister file raymarcher/mandelbulb.c is another fractal drawn the same
 * way, but from one non-linear formula instead of folds.
 *
 * Where the ideas come from:
 *   - the fold-and-shrink fractal trick: Knighty, "Kaleidoscopic IFS" (2010)
 *   - creep-along-the-ray rendering: Hart, "Sphere Tracing" (1996)
 *   - the sphere/box distance helpers: Iñigo Quílez,
 *     https://iquilezles.org/articles/distfunctions/
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * How the file is laid out — each job in its own area so it's easy to
 * follow and hard to break:
 *   §4-§17  the math (plus orbit_to_camera in §15, build helpers in §18):
 *           pure functions that take inputs and return answers — the
 *           fold-and-shrink distance estimator, the surface normal, the ray
 *           march, the camera, lighting, and turning a hit into a glyph.
 *           They never touch shared state or the screen.
 *   §18     the only thing that moves on its own: scene_tick nudges the
 *           camera's orbit angle and the KIFS_ROT animation each frame.
 *   §19-§21 drawing: walk the cells, trace a ray each, paint the result.
 *           Reads the state, never changes it.
 *   §22     input (keys, resize, quit) — changes state, but between frames.
 *   §2,§22  timekeeping and pacing.
 * (No stored glows/trails and no scripted pauses, so two of the usual
 *  layers simply don't exist here.)
 *
 * One frame, in order: apply a pending resize, measure elapsed time, run
 * the simulation forward, refresh fps + sleep to pace, draw, read a key.
 * Scene (§18) is the single bundle of state; only scene_tick and the
 * key/resize handlers ever change it.
 */

/* §1  config — all the tunable numbers live here */

/* §1.1 frame rate + screen layout. */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500, /* how often to refresh the fps readout (ms) */
  HUD_ROWS = 2,        /* top + bottom rows are reserved for the HUD */
  ITERS_MIN = 3,
  ITERS_MAX = 18,
};

/* §1.2 ncurses colour-pair slots. */
enum {
  PAIR_HUD = 1,       /* yellow status line (top)          */
  PAIR_HINT = 2,      /* cyan key reminders (bottom)       */
  PAIR_TRAP_BASE = 3, /* +0..+7 — the 8 fractal colours    */
  PAIR_BG = 11,       /* the empty background              */
};

/* §1.3 time. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define RENDER_FPS_CAP 60 /* frames/sec the draw loop is paced to */
/* If a frame takes longer than this (a resize, a stall), pretend it was
 * only this long, so the simulation can't lurch trying to catch up. */
#define MAX_FRAME_NS (100 * NS_PER_MS)

/* §1.4 camera. */
#define CAM_DIST_DEFAULT 3.6f
#define CAM_DIST_MIN 1.6f /* nearest zoom — stops the eye going inside the shape */
#define CAM_DIST_MAX 12.0f
#define CAM_DIST_STEP 0.20f
#define FOV_DEG 55.0f
#define CELL_ASPECT 2.0f     /* terminal cells are ~2× taller than wide */
#define ORBIT_YAW_RATE 0.35f /* auto-spin speed (radians/sec)           */
#define FOLD_ROT_RATE 0.18f  /* KIFS_ROT morph speed (radians/sec)      */
#define MANUAL_YAW_STEP 0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX 1.20f /* keep the up/down tilt short of straight up */

/* §1.5 ray-march tuning.
 *   MAX_STEPS   give up after this many steps along one ray
 *   HIT_EPS     how close counts as "touched the surface"
 *   MAX_T       if a ray gets this far out, it hit nothing
 *   NORMAL_EPS  how far to nudge when feeling which way the surface faces */
#define MAX_STEPS 70
#define HIT_EPS 1.5e-3f
#define MAX_T 14.0f
#define NORMAL_EPS 6.0e-3f

/* §1.6 lighting.  The shading is "wrapped" so the side facing away from the
 * light keeps a gradient instead of going flat black — see §16.
 *   AMBIENT_LUM  base brightness everywhere (nothing is pure black)
 *   DIFFUSE_LUM  strength of the directional shading
 *   AO_FLOOR     darkest a crevice can get (0 = black, 1 = no darkening) */
#define AMBIENT_LUM 0.18f
#define DIFFUSE_LUM 0.82f
#define AO_FLOOR 0.60f
/* Light direction in world space (up and to one side; normalised at use). */
#define LIGHT_DIR_X 0.55f
#define LIGHT_DIR_Y 0.75f
#define LIGHT_DIR_Z 0.35f

/* §1.7 turning numbers into characters + colours. */
#define LUMA_SLOTS 8         /* 8 brightness / colour steps */
#define LUMA_SLOT_FLT 7.999f /* scale a 0..1 value to a 0..7 slot, no overflow */
#define TRAP_NORM_RANGE 1.4f /* largest closest-approach value we expect (see §10) */
#define TRAP_NORM_INV (1.0f / TRAP_NORM_RANGE)
/* The brightest slots are drawn bold and the darkest dim, squeezing extra
 * contrast out of a short character ramp. */
#define SLOT_BOLD_MIN 6
#define SLOT_DIM_MAX 1

/* §1.8 the three fractals.  Preset names the choice; PresetParams is the
 * recipe for one — how its fold-and-shrink is set up. */
typedef enum {
  PRESET_TETRA = 0,
  PRESET_MENGER = 1,
  PRESET_KIFS_ROT = 2,
  N_PRESETS = 3,
} Preset;

typedef struct {
  const char *name;
  int default_iters;              /* how many fold+shrink rounds by default */
  float scale;                    /* how much each round shrinks toward... */
  float offset_x, offset_y, offset_z; /* ...this fixed point */
  int primitive;                  /* shape measured at the end: 0 sphere, 1 box */
  float bound_radius;             /* rough size, to place the camera */
} PresetParams;

static const PresetParams PRESETS[N_PRESETS] = {
    /*   name        iters scale  offx offy offz  prim  bound */
    /* TETRA   */ {"TETRA   ", 12, 2.00f, 1.00f, 1.00f, 1.00f, 0, 1.8f},
    /* MENGER  */ {"MENGER  ", 7, 3.00f, 1.00f, 1.00f, 1.00f, 1, 1.8f},
    /* KIFS_ROT*/ {"KIFS_ROT", 10, 2.05f, 0.85f, 1.10f, 0.85f, 0, 1.8f},
};

/* §1.9 colour themes: each is a name plus 8 colour codes, dark→bright.
 * (Kept in the bright half of the 256-colour set so they stay legible.) */
typedef struct {
  const char *name;
  short trap[LUMA_SLOTS]; /* 8 xterm-256 colour codes, dark to bright */
} Theme;

#define N_THEMES 6

static const Theme THEMES[N_THEMES] = {
    {"GOLD   ", {130, 137, 173, 215, 222, 229, 230, 231}},
    {"ICE    ", {24, 31, 38, 45, 87, 153, 195, 231}},
    {"COBALT ", {25, 26, 27, 33, 39, 45, 51, 159}},
    {"COPPER ", {130, 166, 173, 209, 215, 222, 229, 230}},
    {"ALIEN  ", {53, 91, 134, 165, 207, 213, 219, 159}},
    {"MONO   ", {244, 246, 248, 250, 252, 253, 254, 255}},
};

/* §1.10 the shading characters, darkest to brightest. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {'`', '.', ',', ':',
                                             '-', '+', '*', '#'};

/* §1.11 alternate views, cycled with d / D — handy for seeing what the
 * renderer is actually computing. */
typedef enum {
  DEBUG_NORMAL = 0,  /* the normal, fully-lit fractal             */
  DEBUG_TRAP = 1,    /* the value that drives the colours          */
  DEBUG_STEPS = 2,   /* how many steps each ray took (the AO cue)  */
  DEBUG_NORMALS = 3, /* colour by which way each surface faces     */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "TRAP   ",
    "STEPS  ",
    "NORMALS",
};

/* §2  clock — read the time, and sleep.  Pure timekeeping; the frame
 * pacing that uses it lives in §22. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                         .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* §3  colour setup (part of the RENDER layer; the rest is §17/§19-§21).
 * Loads the 8 fractal colours plus the HUD colours into ncurses.  theme_apply
 * is split out so t/T can swap palettes without re-initialising everything. */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &THEMES[idx];
    for (int i = 0; i < LUMA_SLOTS; i++)
      init_pair((short)(PAIR_TRAP_BASE + i), t->trap[i], -1);
  } else {
    static const short FB[LUMA_SLOTS] = {
        COLOR_BLUE,   COLOR_MAGENTA, COLOR_CYAN,  COLOR_GREEN,
        COLOR_YELLOW, COLOR_YELLOW,  COLOR_WHITE, COLOR_WHITE,
    };
    for (int i = 0; i < LUMA_SLOTS; i++)
      init_pair((short)(PAIR_TRAP_BASE + i), FB[i], -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
    init_pair(PAIR_BG, 242, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_BG, COLOR_BLACK, -1);
  }
  theme_apply(0);
}

/* The math layer (§4-§17, plus a couple of pure helpers parked in §15/§18):
 * every function here just takes its inputs and returns an answer — no
 * shared state, no screen — so the drawing code can't change its results. */

/* §4  vec3 — a 3-D point or direction (x, y, z) and the usual math on it. */

typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3scale(float s, V3 a) {
  return v3(s * a.x, s * a.y, s * a.z);
}
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline V3 v3cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float L = v3len(a);
  return (L > 1e-12f) ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/* §5  KifsParams — everything one frame's distance function needs, packed
 * once so the distance helpers (called millions of times per frame) don't
 * each take eight arguments.  scene_build_kifs (§18) fills it from Scene;
 * the helpers only ever read it.  A few fields are pre-computed to keep the
 * inner loop cheap. */
typedef struct {
  int preset;             /* which fractal (TETRA / MENGER / KIFS_ROT) */
  int iters;              /* fold+shrink rounds (3..18)                */
  float scale;            /* how much each round shrinks               */
  float sm1;              /* scale − 1, used by the shrink (pre-computed) */
  float offx, offy, offz; /* the point everything shrinks toward       */
  float fold_rot_c;       /* cos/sin of the KIFS_ROT angle, so the     */
  float fold_rot_s;       /*   fold doesn't recompute them per point   */
  float inv_scale_pow;    /* 1 / scale^iters — un-shrinks the final distance */
} KifsParams;

/* §6  the fold step — reflect a point across this preset's mirror planes.
 * Each helper rewrites p in place; one of these runs per round.  (TETRA,
 * MENGER, KIFS_ROT folds, in turn.) */

static inline void fold_iter_tetra(V3 *p) {
  if (p->x + p->y < 0) {
    float t = -p->y;
    p->y = -p->x;
    p->x = t;
  }
  if (p->x + p->z < 0) {
    float t = -p->z;
    p->z = -p->x;
    p->x = t;
  }
  if (p->y + p->z < 0) {
    float t = -p->z;
    p->z = -p->y;
    p->y = t;
  }
}

/* MENGER: mirror into the positive corner, then sort biggest-first. */
static inline void fold_iter_menger(V3 *p) {
  p->x = fabsf(p->x);
  p->y = fabsf(p->y);
  p->z = fabsf(p->z);
  if (p->x < p->y) {
    float t = p->x;
    p->x = p->y;
    p->y = t;
  }
  if (p->x < p->z) {
    float t = p->x;
    p->x = p->z;
    p->z = t;
  }
  if (p->y < p->z) {
    float t = p->y;
    p->y = p->z;
    p->z = t;
  }
}

/* KIFS_ROT: spin around the vertical axis, mirror into the corner, one swap.
 * The spin is what makes this preset slowly morph. */
static inline void fold_iter_rot(V3 *p, float c, float s) {
  float xr = p->x * c - p->z * s;
  float zr = p->x * s + p->z * c;
  p->x = xr;
  p->z = zr;
  p->x = fabsf(p->x);
  p->y = fabsf(p->y);
  p->z = fabsf(p->z);
  if (p->x < p->y) {
    float t = p->x;
    p->x = p->y;
    p->y = t;
  }
}

/* §7  run whichever fold the current preset uses. */
static inline void fold_iter(V3 *p, const KifsParams *kp) {
  switch (kp->preset) {
  case PRESET_TETRA:
    fold_iter_tetra(p);
    break;
  case PRESET_MENGER:
    fold_iter_menger(p);
    break;
  case PRESET_KIFS_ROT:
    fold_iter_rot(p, kp->fold_rot_c, kp->fold_rot_s);
    break;
  }
}

/* §8  the shrink step — pull the point toward the fixed `offset` by `scale`.
 * (Scale is > 1, so this actually pushes outward; the fold beforehand is
 * what turns the pair into convergence onto the fractal.) */
static inline void contract_toward_offset(V3 *p, const KifsParams *kp) {
  p->x = p->x * kp->scale - kp->offx * kp->sm1;
  p->y = p->y * kp->scale - kp->offy * kp->sm1;
  p->z = p->z * kp->scale - kp->offz * kp->sm1;
}

/* §9  Menger-only fix-up: nudge z back into range before the box is
 * measured, otherwise the central column reads the wrong distance and the
 * sponge's recursion visibly breaks.  Every Menger KIFS includes this. */
static inline void menger_z_foldback(V3 *p, const KifsParams *kp) {
  if (p->z < -0.5f * kp->offz * kp->sm1)
    p->z += kp->offz * kp->sm1;
}

/* §10  remember the closest the point ever passed to the origin during the
 * rounds — that "closest approach" is what we colour the surface by, and it
 * varies smoothly across the fractal.  (Squared distance here; the square
 * root is taken once at the end.) */
static inline void track_orbit_trap(V3 p, float *trap_sq) {
  float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
  if (r2 < *trap_sq)
    *trap_sq = r2;
}

/* §11  after all the folding, measure how far the point is from a simple
 * unit shape.  These are exact distances (unlike the folded result, which
 * is only an estimate). */

/* distance to a unit sphere at the origin */
static inline float sphere_de(V3 p) { return v3len(p) - 1.0f; }

/* distance to a unit cube at the origin (Quílez's exact box formula) */
static inline float box_de(V3 p) {
  float qx = fabsf(p.x) - 1.0f;
  float qy = fabsf(p.y) - 1.0f;
  float qz = fabsf(p.z) - 1.0f;
  float dx = fmaxf(qx, 0), dy = fmaxf(qy, 0), dz = fmaxf(qz, 0);
  float outside = sqrtf(dx * dx + dy * dy + dz * dz);
  float inside = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
  return outside + inside;
}

/* MENGER is measured against the box; the others against the sphere. */
static inline float primitive_de(int preset, V3 p) {
  return (preset == PRESET_MENGER) ? box_de(p) : sphere_de(p);
}

/* §12  the distance estimate itself: fold-and-shrink the point a fixed
 * number of rounds, measure it against the simple shape, then undo the
 * shrinking so the answer is back in world units.  It's an estimate (a safe
 * under-guess), which is all ray marching needs.  trap_out is optional —
 * pass NULL while marching; we only want the colour value on a hit. */
static float kifs_de_with_trap(V3 p, const KifsParams *kp, float *trap_out) {
  float trap_sq = 1e10f; /* start huge so the first round wins the "closest" */

  for (int i = 0; i < kp->iters; i++) {
    fold_iter(&p, kp);
    contract_toward_offset(&p, kp);
    if (kp->preset == PRESET_MENGER)
      menger_z_foldback(&p, kp);
    track_orbit_trap(p, &trap_sq);
  }

  if (trap_out)
    *trap_out = sqrtf(trap_sq);
  return primitive_de(kp->preset, p) * kp->inv_scale_pow;
}

/* the common case: just the distance, no colour value */
static inline float kifs_de(V3 p, const KifsParams *kp) {
  return kifs_de_with_trap(p, kp, NULL);
}

/* §13  which way the surface faces at p (its "normal"), needed for lighting.
 * We can't read it directly, so we check how the distance changes a tiny
 * step each way along x, y, z — that points "downhill" toward the surface.
 * Six samples (two per axis) instead of three keeps the shading even. */
static V3 kifs_normal(V3 p, const KifsParams *kp) {
  float e = NORMAL_EPS;
  float dx =
      kifs_de(v3(p.x + e, p.y, p.z), kp) - kifs_de(v3(p.x - e, p.y, p.z), kp);
  float dy =
      kifs_de(v3(p.x, p.y + e, p.z), kp) - kifs_de(v3(p.x, p.y - e, p.z), kp);
  float dz =
      kifs_de(v3(p.x, p.y, p.z + e), kp) - kifs_de(v3(p.x, p.y, p.z - e), kp);
  return v3norm(v3(dx, dy, dz));
}

/* §14  ray marching: step along the ray, each time jumping forward by the
 * distance to the nearest surface (safe — nothing is closer).  When that
 * shrinks to ~zero we've hit something; if the ray runs too far, it missed.
 * On a hit we evaluate the distance once more to grab the colour value (we
 * skip it during the march, where only the final hit's colour matters). */

/* What a ray found.  All the views read from this, so we trace only once. */
typedef struct {
  bool hit;     /* did the ray reach a surface? */
  V3 p;         /* where it landed */
  V3 normal;    /* which way that surface faces */
  float trap;   /* the colour value, 0..1 (closest approach during folding) */
  int steps;    /* how many steps it took (more = deeper crevice, the AO cue) */
} Hit;

static Hit sphere_trace(V3 origin, V3 dir, const KifsParams *kp) {
  Hit out = {false, {0, 0, 0}, {0, 1, 0}, 0.0f, 0};
  float t = 0.0f;

  for (int i = 0; i < MAX_STEPS; i++) {
    V3 p = v3add(origin, v3scale(t, dir));
    float d = kifs_de(p, kp);

    if (d < HIT_EPS) {
      float trap = 0.0f;
      (void)kifs_de_with_trap(p, kp, &trap);

      out.hit = true;
      out.p = p;
      out.steps = i;
      out.trap = clampf(trap * TRAP_NORM_INV, 0.0f, 1.0f);
      out.normal = kifs_normal(p, kp);
      return out;
    }

    if (t > MAX_T)
      break;
    t += d;
  }
  return out;
}

/* §15  the camera.  Orbit is the viewpoint the user steers; orbit_to_camera
 * turns it into the eye position and the three "which way is forward / right
 * / up" directions, and pixel_ray fires one ray per cell from those. */

/* Orbit — where the camera sits, always looking at the fractal centre.
 * Kept as the auto-orbit angles (advanced by scene_tick) plus the manual
 * offsets the arrow keys nudge; orbit_to_camera simply sums the two. */
typedef struct {
  float dist;                 /* distance from the origin (zoom)       */
  float yaw, pitch;           /* auto-orbit angles (yaw auto-advances) */
  float user_yaw, user_pitch; /* manual offsets from the arrow keys    */
} Orbit;

/* Camera — the orthonormal view frame orbit_to_camera derives from an Orbit
 * each frame; pixel_ray reads it to build each cell's ray. */
typedef struct {
  V3 origin;
  V3 fwd, right, up;
  float fov_t;
  float phys_aspect;
} Camera;

/* orbit_to_camera — pure map Orbit → Camera (reads the viewpoint + viewport,
 * mutates nothing).  cols/rows_eff drive the cell-aspect correction. */
static Camera orbit_to_camera(const Orbit *o, int cols, int rows_eff) {
  float yaw = o->yaw + o->user_yaw;
  float pitch = clampf(o->pitch + o->user_pitch, -MANUAL_PITCH_MAX,
                       MANUAL_PITCH_MAX);

  Camera c;
  c.origin = v3(o->dist * cosf(pitch) * cosf(yaw), o->dist * sinf(pitch),
                o->dist * cosf(pitch) * sinf(yaw));
  c.fwd = v3norm(v3sub(v3(0, 0, 0), c.origin));
  V3 wup = v3(0, 1, 0);
  c.right = v3norm(v3cross(c.fwd, wup));
  c.up = v3cross(c.right, c.fwd);
  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)cols;
  return c;
}

/* Direction of the ray through one cell.  Row 0 is the top, and the tall-cell
 * stretch is undone so the fractal comes out round, not squashed. */
static V3 pixel_ray(int col, int row, int cols, int rows_eff, const Camera *c) {
  float u = ((float)col + 0.5f) / (float)cols * 2.0f - 1.0f;
  float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
  V3 sx = v3scale(u * c->fov_t, c->right);
  V3 sy = v3scale(v * c->fov_t * c->phys_aspect, c->up);
  return v3norm(v3add(c->fwd, v3add(sx, sy)));
}

/* §16  how bright a hit point is (0..1): a base glow plus how squarely the
 * surface faces the light, darkened a bit inside crevices.
 *
 * The light is "wrapped" around the surface (half-Lambert, Valve/Mitchell
 * 2006): normally the side facing away from the light goes flat black and
 * loses all its shape — fatal here, since the character itself shows the
 * lighting — so we let even the far side keep a gentle gradient.
 *
 * The crevice darkening (AO) is a cheap trick: rays into deep folds take
 * more steps to land, so a high step count stands in for "tucked away in a
 * concavity".  Not physically real, but it reads well and costs nothing. */
static float lambert_with_ao(V3 normal, int steps, V3 light) {
  float ndl = v3dot(normal, light);
  float wrap = ndl * 0.5f + 0.5f;
  float diffuse = wrap * wrap;
  float lum = AMBIENT_LUM + DIFFUSE_LUM * diffuse;

  float ao = 1.0f - (float)steps / (float)MAX_STEPS;
  if (ao < AO_FLOOR)
    ao = AO_FLOOR;
  return lum * ao;
}

/* §17  turn a hit into something printable, then print it.  shade_hit /
 * to_slot just compute (a glyph + colour); emit_cell does the actual
 * writing to the terminal. */

/* map a 0..1 value to one of the 8 brightness/colour slots */
static int to_slot(float x_01) {
  int s = (int)(x_01 * LUMA_SLOT_FLT);
  if (s < 0)
    s = 0;
  if (s >= LUMA_SLOTS)
    s = LUMA_SLOTS - 1;
  return s;
}

/* One terminal cell, ready to print: the character, its colour, and a
 * bold/dim/normal attribute. */
typedef struct {
  char glyph;
  int pair;
  attr_t attr;
} Cell;

/* The normal view: the character shows brightness, the colour shows the
 * fractal's "closest approach" value. */
static Cell shade_hit(const Hit *h, V3 light) {
  if (!h->hit) {
    return (Cell){' ', PAIR_BG, A_NORMAL};
  }

  float lum = lambert_with_ao(h->normal, h->steps, light);
  int s_lum = to_slot(lum);
  int s_clr = to_slot(h->trap);

  return (Cell){
      .glyph = LUMA_GLYPHS[s_lum],
      .pair = PAIR_TRAP_BASE + s_clr,
      .attr = (s_lum >= SLOT_BOLD_MIN)  ? A_BOLD
              : (s_lum <= SLOT_DIM_MAX) ? A_DIM
                                        : A_NORMAL,
  };
}

/* Print one cell.  Changing colour in ncurses isn't free, so we only switch
 * when this cell differs from the last — big saving across flat regions. */
static void emit_cell(int row, int col, Cell c, int *last_pair,
                      attr_t *last_attr) {
  if (c.pair != *last_pair || c.attr != *last_attr) {
    if (*last_pair >= 0)
      attroff(COLOR_PAIR(*last_pair) | *last_attr);
    attron(COLOR_PAIR(c.pair) | c.attr);
    *last_pair = c.pair;
    *last_attr = c.attr;
  }
  mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/* §18  the state, and the one function that moves it on its own.  Scene
 * holds everything; only scene_tick (and the key/resize handlers) change it.
 * scene_iters and scene_build_kifs are pure helpers that just read Scene. */

/* Scene — all simulation state, as a table of contents:
 *   WHAT is shown    current_preset  which fractal (indexes PRESETS)
 *                    iters_override  fold depth; 0 = use the preset default
 *                    fold_rot        KIFS_ROT animation angle (auto-advances)
 *   HOW it's viewed  orbit           camera viewpoint (see §15)
 *                    current_theme   colour palette
 *                    debug_mode      which view (production / TRAP / …)
 *                    paused          freezes the orbit + animation
 *   WHERE            cols, rows      terminal size in cells */
typedef struct {
  int current_preset;
  int iters_override;
  float fold_rot;

  Orbit orbit;
  int current_theme;
  DebugMode debug_mode;
  bool paused;

  int cols, rows;
} Scene;

static int scene_iters(const Scene *s) {
  int it = (s->iters_override > 0) ? s->iters_override
                                   : PRESETS[s->current_preset].default_iters;
  if (it < ITERS_MIN)
    it = ITERS_MIN;
  if (it > ITERS_MAX)
    it = ITERS_MAX;
  return it;
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->current_preset = PRESET_TETRA;
  s->current_theme = 0;
  s->iters_override = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->cols = cols;
  s->rows = rows;
  s->orbit.dist = CAM_DIST_DEFAULT;
  s->orbit.yaw = 0.5f;
  s->orbit.pitch = 0.25f;
  s->orbit.user_yaw = 0.0f;
  s->orbit.user_pitch = 0.0f;
  s->fold_rot = 0.4f;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset_cam(Scene *s) {
  s->orbit.dist = CAM_DIST_DEFAULT;
  s->orbit.yaw = 0.5f;
  s->orbit.pitch = 0.25f;
  s->orbit.user_yaw = 0.0f;
  s->orbit.user_pitch = 0.0f;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->orbit.yaw += ORBIT_YAW_RATE * dt;
  if (s->orbit.yaw > (float)(2.0 * M_PI))
    s->orbit.yaw -= (float)(2.0 * M_PI);
  if (s->orbit.yaw < -(float)(2.0 * M_PI))
    s->orbit.yaw += (float)(2.0 * M_PI);

  s->fold_rot += FOLD_ROT_RATE * dt;
  if (s->fold_rot > (float)(2.0 * M_PI))
    s->fold_rot -= (float)(2.0 * M_PI);
}

/* Gather this frame's fractal settings into one flat KifsParams so the
 * distance function (called once per cell, every cell) reads them cheaply.
 * inv_scale_pow is the un-shrink factor, worked out once here. */
static void scene_build_kifs(const Scene *s, KifsParams *kp) {
  const PresetParams *pp = &PRESETS[s->current_preset];
  int iters = scene_iters(s);

  kp->preset = s->current_preset;
  kp->iters = iters;
  kp->scale = pp->scale;
  kp->sm1 = pp->scale - 1.0f;
  kp->offx = pp->offset_x;
  kp->offy = pp->offset_y;
  kp->offz = pp->offset_z;
  kp->fold_rot_c = cosf(s->fold_rot);
  kp->fold_rot_s = sinf(s->fold_rot);
  kp->inv_scale_pow = expf(-(float)iters * logf(pp->scale));
}

/* §19  draw the normal view: set up the camera, light, and fractal for this
 * frame, then for every cell shoot a ray, see what it hits, and paint it. */
static void render_normal(const Scene *s) {
  int rows_eff = s->rows - HUD_ROWS;
  if (rows_eff < 1)
    return;

  Camera cam = orbit_to_camera(&s->orbit, s->cols, rows_eff);
  V3 light = v3norm(v3(LIGHT_DIR_X, LIGHT_DIR_Y, LIGHT_DIR_Z));
  KifsParams kp;
  scene_build_kifs(s, &kp);

  int last_pair = -1;
  attr_t last_attr = 0;
  int y0 = 1; /* shift down 1 for top HUD row */

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      V3 ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
      Hit h = sphere_trace(cam.origin, ray, &kp);
      Cell c = shade_hit(&h, light);
      emit_cell(y0 + row, col, c, &last_pair, &last_attr);
    }
  }

  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* §20  the alternate views (d/D).  Each paints one piece of what the
 * renderer computed, with no lighting, so you can see it directly:
 *   TRAP    — the value that drives the colours
 *   STEPS   — how hard each ray had to work (the crevice-darkening cue)
 *   NORMALS — which way each surface faces
 * The debug_cell_* helpers just pick a glyph+colour; render_debug draws. */

static Cell debug_cell_for_trap(const Hit *h) {
  if (!h->hit)
    return (Cell){' ', PAIR_BG, A_NORMAL};
  int s_clr = to_slot(h->trap);
  return (Cell){
      .glyph = LUMA_GLYPHS[s_clr],
      .pair = PAIR_TRAP_BASE + s_clr,
      .attr = A_NORMAL,
  };
}

static Cell debug_cell_for_steps(const Hit *h) {
  if (!h->hit)
    return (Cell){' ', PAIR_BG, A_NORMAL};
  float t = (float)h->steps / (float)(MAX_STEPS - 1);
  int slot = to_slot(t);
  return (Cell){
      .glyph = LUMA_GLYPHS[slot],
      .pair = PAIR_TRAP_BASE + slot,
      .attr = (slot >= SLOT_BOLD_MIN) ? A_BOLD : A_NORMAL,
  };
}

static Cell debug_cell_for_normals(const Hit *h) {
  if (!h->hit)
    return (Cell){' ', PAIR_BG, A_NORMAL};
  float azimuth = atan2f(h->normal.z, h->normal.x); /* −π..+π */
  float t = (azimuth + (float)M_PI) / (2.0f * (float)M_PI);
  int slot = to_slot(t);
  float y_lit = (h->normal.y * 0.5f + 0.5f); /* 0..1 */
  int g_slot = to_slot(y_lit);
  return (Cell){
      .glyph = LUMA_GLYPHS[g_slot],
      .pair = PAIR_TRAP_BASE + slot,
      .attr = A_NORMAL,
  };
}

/* Same loop as the normal view; only the per-cell choice differs. */
static void render_debug(const Scene *s, DebugMode mode) {
  int rows_eff = s->rows - HUD_ROWS;
  if (rows_eff < 1)
    return;

  Camera cam = orbit_to_camera(&s->orbit, s->cols, rows_eff);
  KifsParams kp;
  scene_build_kifs(s, &kp);

  int last_pair = -1;
  attr_t last_attr = 0;
  int y0 = 1;

  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      V3 ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
      Hit h = sphere_trace(cam.origin, ray, &kp);

      Cell c;
      switch (mode) {
      case DEBUG_TRAP:
        c = debug_cell_for_trap(&h);
        break;
      case DEBUG_STEPS:
        c = debug_cell_for_steps(&h);
        break;
      case DEBUG_NORMALS:
        c = debug_cell_for_normals(&h);
        break;
      default:
        c = (Cell){' ', PAIR_BG, A_NORMAL};
        break;
      }
      emit_cell(y0 + row, col, c, &last_pair, &last_attr);
    }
  }

  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void render_active_view(const Scene *s) {
  if (s->debug_mode == DEBUG_NORMAL)
    render_normal(s);
  else
    render_debug(s, s->debug_mode);
}

/* §21  screen — start/stop ncurses, draw the HUD, push the frame out. */

/* The terminal's current size in character cells. */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE); /* getch() returns at once if no key is waiting */
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt our drawing */
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

/* The endwin + refresh dance makes ncurses notice the new terminal size. */
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Draw the two HUD rows: title + status along the top, key reminders along
 * the bottom. */
static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  %3d Hz  preset:%s  theme:%s  iters:%2d  "
           "debug:%s  dist:%4.2f  %s ",
           fps, sim_fps, PRESETS[s->current_preset].name,
           THEMES[s->current_theme].name, scene_iters(s),
           DEBUG_MODE_NAMES[s->debug_mode], (double)s->orbit.dist,
           s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > sc->cols)
    slen = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - slen, "%s", status);
  mvprintw(0, 0, " KIFS · FRACTAL ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme  "
           "d/D:debug  i/I:iters  z/Z:zoom  arrows:orbit ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  render_active_view(s);
  hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §22  input + the main loop.  Keys and resize change the state between
 * frames; main() is the one place that runs everything each frame, in order
 * (apply input → step the sim → draw → pace), and holds the frame rate. */

/* Everything the running program owns: the scene, the terminal size, the
 * target sim rate, and two flags the signal handlers flip. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
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

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
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
    scene_reset_cam(s);
    s->iters_override = 0;
    break;

  case 'n':
    s->current_preset = (s->current_preset + 1) % N_PRESETS;
    s->iters_override = 0;
    break;
  case 'N':
    s->current_preset = (s->current_preset + N_PRESETS - 1) % N_PRESETS;
    s->iters_override = 0;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  case 'i': {
    int it = scene_iters(s);
    if (it > ITERS_MIN)
      s->iters_override = it - 1;
    break;
  }
  case 'I': {
    int it = scene_iters(s);
    if (it < ITERS_MAX)
      s->iters_override = it + 1;
    break;
  }

  case 'z':
    s->orbit.dist += CAM_DIST_STEP;
    if (s->orbit.dist > CAM_DIST_MAX)
      s->orbit.dist = CAM_DIST_MAX;
    break;
  case 'Z':
    s->orbit.dist -= CAM_DIST_STEP;
    if (s->orbit.dist < CAM_DIST_MIN)
      s->orbit.dist = CAM_DIST_MIN;
    break;

  case KEY_LEFT:
    s->orbit.user_yaw -= MANUAL_YAW_STEP;
    break;
  case KEY_RIGHT:
    s->orbit.user_yaw += MANUAL_YAW_STEP;
    break;
  case KEY_UP:
    s->orbit.user_pitch += MANUAL_PITCH_STEP;
    break;
  case KEY_DOWN:
    s->orbit.user_pitch -= MANUAL_PITCH_STEP;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

/* update_fps — count this frame; every FPS_UPDATE_MS recompute the rate to
 * show and start a fresh window.  Returns the value to display (unchanged
 * between refreshes); updates the two accumulators it's handed. */
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

/* pace_frame — sleep off the rest of this frame's time budget so the draw
 * loop holds RENDER_FPS_CAP no matter how quick the work was. */
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

    /* 4. PERFORMANCE — refresh the fps readout, then hold the frame cap */
    fps_display = update_fps(&fps_accum, &frame_count, dt, fps_display);
    pace_frame(frame_time, dt);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps); /* 5. RENDER */
    screen_present();

    int ch = getch(); /* 6. EVENTS — drain one key */
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
