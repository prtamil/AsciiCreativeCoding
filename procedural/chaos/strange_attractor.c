/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * strange_attractor.c — ten chaotic attractors, each drawn as a glowing
 * trail of recent points spun around in 3-D.  Nine are flat 2-D maps
 * lifted onto a sheet; the tenth is the Lorenz butterfly.  Same look as
 * physics/lorenz.c (sister file -- shares the colour and camera ideas).
 *
 * Attractor formulas and parameter sets: Sprott, *Strange Attractors*
 * (1993) and Paul Bourke's catalog at paulbourke.net/fractals.
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ── §1  config ── */

enum {
  TRAIL_LEN          = 2500, /* how many recent points each trail remembers */
  SUB_STEPS_ODE      = 8,    /* Lorenz integration steps per tick           */
  ITERS_PER_TICK_MAP = 10,   /* map iterations per tick                     */
  N_GHOSTS           = 5,    /* faint shadow orbits started a hair apart    */
  WARMUP_ITERS       = 5000, /* throwaway steps to settle onto the shape    */
  BBOX_SAMPLES       = 50000,/* steps sampled to measure the shape's extent */
  N_STARS            = 60,   /* backdrop dots                               */
  SPEED_MIN          = 1,
  SPEED_MAX          = 8,
  FPS_UPDATE_MS      = 500,
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS      (NS_PER_SEC / 60)        /* aim for 60 frames a second */

#define MAX_FRAME_DT_NS      (100 * NS_PER_MS) /* cap a frame's elapsed time so a long stall can't trigger a catch-up avalanche */
#define FIXED_TICK_DT_SEC    (1.0f / 60.0f)    /* wall seconds one sim tick stands for */

#define BBOX_MARGIN_FRAC      0.05f       /* breathing room around the measured shape */
#define BBOX_RADIUS_EPS       1e-6f       /* below this the shape is basically a point */
#define BBOX_RADIUS_FALLBACK  1.0f        /* stand-in size when that happens, avoids divide-by-zero */

#define ORBIT_SEED            0.1f        /* starting x=y=z for every orbit */

/* Marks "nothing drawn here yet" for the duplicate-cell check.  A real
 * screen cell can never be at (-999, -999). */
#define CELL_NONE            (-999)

/* The classic chaotic Lorenz settings.  They ride in an attractor row as
 * a, b, c (sigma, rho, beta); d carries the integration step size. */
#define LORENZ_SIGMA  10.0f
#define LORENZ_RHO    28.0f
#define LORENZ_BETA   (8.0f / 3.0f)
#define LORENZ_H      0.005f

#define HUD_TOP_ROWS     2
#define HUD_BOTTOM_ROWS  1

/* Terminal cells are taller than they are wide; this keeps circles round. */
#define CELL_W 8
#define CELL_H 16
#define ASPECT ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

#define VIEW_PHI_DEFAULT    0.5f      /* starting spin angle (rad)        */
#define VIEW_THETA_DEFAULT  0.55f     /* starting tilt angle (rad)        */
#define VIEW_PHI_SPEED      0.08f     /* hands-free spin rate (rad/s)     */
#define VIEW_PHI_STEP       0.10f     /* spin nudge per key press         */
#define VIEW_THETA_STEP     0.05f     /* tilt nudge per key press         */
#define VIEW_THETA_MIN      0.10f     /* don't tilt fully flat (it vanishes) */
#define VIEW_THETA_MAX      1.40f     /* don't tilt fully edge-on            */

#define VIEW_FILL_FRAC      0.80f     /* leave a small margin around the shape */

/* When does a point count as "near the camera" vs "far behind"?  Given as
 * a fraction of the shape's size so it works for tiny and huge attractors
 * alike (Lorenz is ~25x bigger than Henon). */
#define DEPTH_CLOSE_FRAC   -0.25f
#define DEPTH_FAR_FRAC      0.25f

/* Where the trail switches brightness, by age (0 = newest, 1 = oldest). */
#define AGE_HEAD_LIMIT      0.25f
#define AGE_MID_LIMIT       0.60f
#define AGE_TAIL_LIMIT      0.80f

/* How many points at the very front get the glowing '+' halo. */
#define BLOOM_HEAD_COUNT    2

#define KEY_ESC  27

/*
 * The colour slots we paint with.  The first four change with the theme;
 * the last two are the fixed HUD colours.
 *   head/mid/tail  the three brightness tiers of the trail, newest to oldest
 *   ghost          the faint shadow orbits
 *   hud / hint     status bar (yellow) and key list (cyan)
 */
enum {
  CP_TRAIL_HEAD = 1,
  CP_TRAIL_MID  = 2,
  CP_TRAIL_TAIL = 3,
  CP_GHOST      = 4,
  CP_HUD        = 5,
  CP_HINT       = 6,
};

#define HUD_DATA_YELLOW_256   226
#define HUD_TITLE_CYAN_256     51

/* ── §2  clock ── */

static int64_t clock_ns(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
  if (ns <= 0) return;
  struct timespec req = {
    .tv_sec  = (time_t)(ns / NS_PER_SEC),
    .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3  color / theme ── */

/*
 * Theme -- one colour scheme.  Four colour codes, one for each thing the
 * trail draws: the newest, middle, and oldest parts of the trail, plus
 * the faint shadow orbits.  theme_apply() loads these into the live
 * colour slots when you cycle themes.
 *
 * We pick four fixed colours rather than a smooth gradient because each
 * character on screen can only hold one colour -- we fake the in-between
 * shades by dimming and brightening these four (done in draw_trail).
 * The HUD colours are separate and never change with the theme.
 *
 * The codes are `short` to match what ncurses' init_pair() wants.
 */
typedef struct {
  const char *name;     /* shown in the HUD, e.g. "Inferno"              */
  short       head;     /* newest part of the trail -- the bright tip    */
  short       mid;      /* middle of the trail                           */
  short       tail;     /* oldest part -- the fading end                 */
  short       ghost;    /* the faint shadow orbits                       */
} Theme;

/*
 * Ten colour schemes.  Each picks colours from the bright half of the
 * 256-colour set so even the faded tail and dim shadows stay readable --
 * picking dull mid-cube colours would wash out once we dim them.
 */
static const Theme THEMES[] = {
  /*  name           head  mid  tail  ghost   palette character                  */
  { "Neon",          213,  207, 201,   51 }, /* hot pink ramp + cyan accent     */
  { "Matrix",         82,   46,  41,   51 }, /* lime → pure green + cyan accent */
  { "Sunset",        220,  214, 209,  213 }, /* gold → orange → salmon + pink   */
  { "Ocean",          51,   45,  39,  213 }, /* cyan → cyan-blue → blue + pink  */
  { "Plasma",        201,  165, 129,   51 }, /* magenta → purple → indigo + cyan*/
  { "Inferno",       226,  220, 208,  196 }, /* yellow → amber → orange → red   */
  { "Mint",          122,   86,  50,  213 }, /* mint → bright mint → cyan-mint  */
  { "Aurora",        122,   87,  51,  207 }, /* mint → light cyan → cyan + pink */
  { "Synthwave",     207,  165,  51,  213 }, /* hot pink → magenta → cyan + pink*/
  { "Rainbow",       196,  226,  46,   51 }, /* red → yellow → green + cyan     */
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/* Load one theme's four colours into the live trail slots.  The HUD
 * colours are left alone -- they never change with the theme. */
static void theme_apply(int idx)
{
  if (idx < 0 || idx >= N_THEMES) return;
  const Theme *th = &THEMES[idx];

  if (COLORS >= 256) {
    init_pair(CP_TRAIL_HEAD, th->head,  -1);
    init_pair(CP_TRAIL_MID,  th->mid,   -1);
    init_pair(CP_TRAIL_TAIL, th->tail,  -1);
    init_pair(CP_GHOST,      th->ghost, -1);
  } else {
    /* On an 8-colour terminal all themes look the same. */
    init_pair(CP_TRAIL_HEAD, COLOR_RED,     -1);
    init_pair(CP_TRAIL_MID,  COLOR_YELLOW,  -1);
    init_pair(CP_TRAIL_TAIL, COLOR_GREEN,   -1);
    init_pair(CP_GHOST,      COLOR_MAGENTA, -1);
  }
}

/* Set up the fixed HUD colours once and load the first theme. */
static void color_init(void)
{
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD,  HUD_DATA_YELLOW_256, -1);
    init_pair(CP_HINT, HUD_TITLE_CYAN_256,  -1);
  } else {
    init_pair(CP_HUD,  COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN,   -1);
  }
  theme_apply(0);
}

/* ── §4  coords: turning a 3-D point into a screen cell ── */

/* Three small steps that project() chains together: shift so the shape
 * sits at the origin, spin it, then tilt it. */

/* Shift the point so the shape's centre lands at the origin.  The centre
 * comes from the size measurement in §5. */
static inline void world_center_to_origin(float lx, float ly, float lz,
                                          float cx, float cy, float cz,
                                          float *px, float *py, float *pz)
{
  *px = lx - cx;
  *py = ly - cy;
  *pz = lz - cz;
}

/* Spin the point around the vertical axis by angle phi -- this is what
 * turns the shape left/right as you watch. */
static inline void rotate_around_z_axis(float px, float py, float phi,
                                        float *rx, float *ry)
{
  float c = cosf(phi), s = sinf(phi);
  *rx =  px * c + py * s;
  *ry = -px * s + py * c;
}

/* Tilt the point up/down by angle theta.  sy ends up as the on-screen
 * height; sz is how far the point sits from the camera (negative = in
 * front / closer, positive = behind / farther) -- used for shading. */
static inline void tilt_around_x_axis(float ry, float pz, float theta,
                                      float *sy, float *sz)
{
  float c = cosf(theta), s = sinf(theta);
  *sy =  ry * c + pz * s;
  *sz = -ry * s + pz * c;
}

/*
 * Turn a point in attractor space into a screen cell (column, row) plus
 * its distance from the camera.  Shift to centre, spin, tilt, then scale
 * into cell coordinates.  Pass NULL for out_depth if you don't need the
 * distance.  Returns false when the point falls off the visible area.
 */
static bool project(float lx, float ly, float lz,
                    float cx, float cy, float cz,
                    float phi, float theta, float scale,
                    int screen_cx, int screen_cy, int cols, int rows,
                    int *out_col, int *out_row, float *out_depth)
{
  float px, py, pz;
  world_center_to_origin(lx, ly, lz, cx, cy, cz,
                         &px, &py, &pz);

  float rx, ry;
  rotate_around_z_axis(px, py, phi, &rx, &ry);

  float sx = rx;
  float sy, sz;
  tilt_around_x_axis(ry, pz, theta, &sy, &sz);

  /* place it on the grid (ASPECT keeps it from looking squashed) */
  int col = screen_cx + (int)(sx * scale);
  int row = screen_cy - (int)(sy * scale * ASPECT);

  if (out_col)   *out_col   = col;
  if (out_row)   *out_row   = row;
  if (out_depth) *out_depth = sz;

  /* off-screen, or in the rows reserved for the HUD? caller skips it */
  return (col >= 0 && col < cols
       && row >= HUD_TOP_ROWS && row < rows - HUD_BOTTOM_ROWS);
}

/* ── §5  attractor definitions ── */

/*
 * AttrDef -- one of the ten presets.
 *
 * `step` is a function pointer: it picks which formula advances the orbit
 * one step, so adding a new attractor is just adding a row plus a step
 * function -- no big switch to edit.  a, b, c, d are that formula's knobs.
 *
 * Every step takes x, y, z because Lorenz needs all three; the flat 2-D
 * maps just ignore z.  is_continuous flags Lorenz, the one preset that's
 * a smooth flowing curve rather than a sequence of jumps -- scene_tick
 * uses it to choose how to advance the orbit.
 */
struct AttrDef;
typedef void (*attr_step_fn)(const struct AttrDef *at,
                             float *x, float *y, float *z);

typedef struct AttrDef {
  const char  *name;
  float        a, b, c, d;     /* the formula's tuning knobs            */
  attr_step_fn step;           /* the formula that moves the orbit one step */
  bool         is_continuous;  /* true only for Lorenz (a smooth curve) */
} AttrDef;

/* Each function below is one attractor's formula -- given the current
 * point, it works out the next one.  The math is the attractor; the
 * comment names it and cites where the formula comes from. */

/* Henon (1976): x' = 1 - a*x^2 + y,   y' = b*x */
static void henon_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = 1.0f - at->a * (*x) * (*x) + (*y);
  float ny = at->b * (*x);
  *x = nx; *y = ny;
}

/* Hopalong (Barry Martin):
 *   x' = y - sign(x) * sqrt|b*x - c|,   y' = a - x */
static void hopalong_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float sign_x = (*x > 0.0f) ? 1.0f : ((*x < 0.0f) ? -1.0f : 0.0f);
  float nx = *y - sign_x * sqrtf(fabsf(at->b * (*x) - at->c));
  float ny = at->a - *x;
  *x = nx; *y = ny;
}

/* Clifford / de Jong / Marek / Rampe shared form:
 *   x' = sin(a·y) + c·cos(a·x),   y' = sin(b·x) + d·cos(b·y). */
static void clifford_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = sinf(at->a * (*y)) + at->c * cosf(at->a * (*x));
  float ny = sinf(at->b * (*x)) + at->d * cosf(at->b * (*y));
  *x = nx; *y = ny;
}

/* Tinkerbell:
 *   x' = x² − y² + a·x + b·y,   y' = 2·x·y + c·x + d·y. */
static void tinkerbell_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float xx = (*x) * (*x);
  float yy = (*y) * (*y);
  float nx = xx - yy + at->a * (*x) + at->b * (*y);
  float ny = 2.0f * (*x) * (*y) + at->c * (*x) + at->d * (*y);
  *x = nx; *y = ny;
}

/* Svensson:
 *   x' = d·sin(a·x) − sin(b·y),   y' = c·cos(a·x) + cos(b·y). */
static void svensson_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = at->d * sinf(at->a * (*x)) - sinf(at->b * (*y));
  float ny = at->c * cosf(at->a * (*x)) + cosf(at->b * (*y));
  *x = nx; *y = ny;
}

/* Bedhead:
 *   x' = sin(x·y/b)·y + cos(a·x − y),   y' = x + sin(y)/b. */
static void bedhead_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float b_safe = (fabsf(at->b) < 1e-4f) ? 1e-4f : at->b;
  float nx = sinf((*x) * (*y) / b_safe) * (*y) + cosf(at->a * (*x) - (*y));
  float ny = (*x) + sinf(*y) / b_safe;
  *x = nx; *y = ny;
}

/* ---- Lorenz: the one continuous-flow preset ---- */

/*
 * Which way the Lorenz orbit is heading right now -- its velocity at the
 * current point (Lorenz 1963).
 *   dx/dt = sigma*(y - x)
 *   dy/dt = x*(rho - z) - y
 *   dz/dt = x*y - beta*z
 */
static inline void lorenz_deriv(float x, float y, float z,
                                float *dx, float *dy, float *dz)
{
  *dx = LORENZ_SIGMA * (y - x);
  *dy = x * (LORENZ_RHO - z) - y;
  *dz = x * y - LORENZ_BETA * z;
}

/*
 * Move the Lorenz orbit forward one small step using RK4 -- a recipe that
 * samples the velocity four times across the step and blends them, far
 * more accurate than a single straight-line guess.  The step h = 0.005 is
 * small enough that the orbit looks dead-on for as long as you'll watch.
 * (Press et al., *Numerical Recipes*, Ch.17.)
 */
static void lorenz_step(const AttrDef *at, float *x, float *y, float *z)
{
  float h = at->d;
  float k1x, k1y, k1z, k2x, k2y, k2z, k3x, k3y, k3z, k4x, k4y, k4z;

  lorenz_deriv(*x, *y, *z, &k1x, &k1y, &k1z);
  lorenz_deriv(*x + 0.5f * h * k1x,
               *y + 0.5f * h * k1y,
               *z + 0.5f * h * k1z, &k2x, &k2y, &k2z);
  lorenz_deriv(*x + 0.5f * h * k2x,
               *y + 0.5f * h * k2y,
               *z + 0.5f * h * k2z, &k3x, &k3y, &k3z);
  lorenz_deriv(*x + h * k3x,
               *y + h * k3y,
               *z + h * k3z, &k4x, &k4y, &k4z);

  *x += (h / 6.0f) * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
  *y += (h / 6.0f) * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
  *z += (h / 6.0f) * (k1z + 2.0f * k2z + 2.0f * k3z + k4z);
}

/* The ten presets, in the order the n/p and number keys step through.
 * The a,b,c,d values are the well-known tunings from Sprott and Bourke. */
static const AttrDef ATTRS[] = {
  /*  name           a              b              c             d           step             continuous */
  { "Henon",       1.40f,         0.30f,         0.00f,         0.00f,    henon_step,      false },
  { "Hopalong",    7.70f,         0.13f,         8.15f,         0.00f,    hopalong_step,   false },
  { "Clifford",   -1.40f,         1.60f,         1.00f,         0.70f,    clifford_step,   false },
  { "de Jong",    -1.70f,         1.30f,        -0.10f,        -1.20f,    clifford_step,   false },
  { "Rampe",       1.00f,        -1.20f,        -0.50f,         0.50f,    clifford_step,   false },
  { "Tinkerbell",  0.90f,        -0.6013f,       2.00f,         0.50f,    tinkerbell_step, false },
  { "Svensson",    1.50f,        -1.80f,         1.60f,         0.90f,    svensson_step,   false },
  { "Bedhead",    -0.81f,        -0.92f,         0.00f,         0.00f,    bedhead_step,    false },
  { "Marek",      -2.00f,        -2.00f,        -1.20f,         2.00f,    clifford_step,   false },
  { "Lorenz",      LORENZ_SIGMA,  LORENZ_RHO,    LORENZ_BETA,   LORENZ_H, lorenz_step,     true  },
};
#define N_ATTRS  ((int)(sizeof ATTRS / sizeof ATTRS[0]))

/* Move the orbit one step using whichever formula this preset carries. */
static inline void attractor_step(const AttrDef *at,
                                  float *x, float *y, float *z)
{
  at->step(at, x, y, z);
}

/* ── §6  orbits: one moving point and the trail it leaves ── */

/*
 * Three nested pieces for tracking one moving point:
 *   Point3  one spot in space
 *   Trail   the last few thousand spots it visited
 *   Orbit   where it is now, plus its Trail
 * The Scene (§7) holds one main Orbit plus a handful of shadow Orbits.
 */

/* One point in the attractor's 3-D space.  A struct (not three loose
 * floats) so a point can be passed and returned as one value. */
typedef struct {
  float x, y, z;
} Point3;

/*
 * Trail -- the recent history of one orbit, drawn as the glowing tail.
 *
 * It's a ring buffer: a fixed array that wraps around, so once it's full
 * each new point overwrites the oldest one.  The orbit runs forever, so a
 * plain growing array would either leak memory or fill up -- the ring
 * keeps the last TRAIL_LEN points and nothing more.
 *
 * `head` is where the newest point sits; `count` is how many points are
 * real so far (it climbs to TRAIL_LEN as the ring fills, then stays
 * there).  We track count rather than a second index because the ring
 * only ever fills, never drains -- a tail index would be redundant, and
 * count also tells the renderer not to draw empty slots.
 */
typedef struct {
  Point3 data[TRAIL_LEN]; /* the ring storage                          */
  int    head;            /* slot holding the newest point             */
  int    count;           /* how many real points so far (<= TRAIL_LEN)*/
} Trail;

/* Empty the trail. */
static void trail_clear(Trail *t)
{
  t->head  = 0;
  t->count = 0;
}

/* Add one point, overwriting the oldest once the ring is full. */
static void trail_push(Trail *t, Point3 p)
{
  t->head = (t->head + 1) % TRAIL_LEN;
  t->data[t->head] = p;
  if (t->count < TRAIL_LEN) t->count++;
}

/* Fetch the k-th newest point (k=0 is newest).  Hides the wrap-around
 * arithmetic so loops can just walk newest-to-oldest.  Caller must keep
 * k within count. */
static inline Point3 trail_at(const Trail *t, int k)
{
  int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
  return t->data[idx];
}

/*
 * Orbit -- one moving point: where it is now, plus its Trail.  The two
 * always travel together (every step moves the point and records it), so
 * they live in one struct.  We keep the live position separately rather
 * than re-reading the trail's head each step, since it's the input the
 * next step works from.
 */
typedef struct {
  Point3 pos;          /* where the point is right now                 */
  Trail  trail;        /* the points it has visited recently           */
} Orbit;

/* Drop the orbit at a starting point and wipe its trail. */
static void orbit_seed(Orbit *o, Point3 p)
{
  o->pos = p;
  trail_clear(&o->trail);
}

/* Advance one step and record the new spot.  Every orbit (main and
 * shadows) goes through here so they all take the same number of steps --
 * that's what makes the shadows drifting apart actually mean something. */
static void orbit_step(Orbit *o, const AttrDef *at)
{
  attractor_step(at, &o->pos.x, &o->pos.y, &o->pos.z);
  trail_push(&o->trail, o->pos);
}

/* ── §7  scene ── */

/*
 * The scene is everything that lives in attractor space and gets painted
 * each frame.  It splits into a few small pieces:
 *   Camera     where you're looking from (two angles)
 *   Bounds     how big the current attractor is, so it fits the screen
 *   Starfield  the backdrop dots
 *   RenderCtx  a few numbers worked out fresh each frame for drawing
 *   Scene      the whole thing, wrapping the orbits plus all of the above
 */

/*
 * Camera -- where we're viewing from, as two angles.  No eye position is
 * needed because the projection has no perspective; only the direction we
 * look from matters.
 *
 *   phi    spin angle -- turns the shape left and right, like spinning a
 *          globe.  Any value; it just wraps around.
 *   theta  tilt angle -- leans the view up or down.  Kept between
 *          VIEW_THETA_MIN and MAX so it never goes fully flat (the shape
 *          would vanish to a line) or straight down.
 *   auto_rotate  when on, the view drifts by itself; the arrow keys turn
 *          it off and let you aim by hand.
 */
typedef struct {
  float phi;           /* spin angle (rad)                             */
  float theta;         /* tilt angle (rad)                             */
  bool  auto_rotate;   /* drift the view on its own when true          */
} Camera;

/*
 * Bounds -- how big the current attractor is and where its middle sits.
 * Measured once per preset (scene_calibrate_bounds) and cached here so the
 * renderer isn't recomputing it for every point.
 *
 *   xmin..zmax  the box that just contains the shape, with a little
 *               padding so its edges don't get clipped.
 *   cx, cy, cz  the box's centre.  project() subtracts this so the shape
 *               sits in the middle of the screen, not off in a corner.
 *   radius      roughly the shape's size (half its diagonal).  Used both
 *               to scale it to fit the screen and as the yardstick for the
 *               near/far shading -- so it all works whether the attractor
 *               is tiny (Henon) or large (Lorenz).
 *
 * For the flat 2-D presets the z range is zero -- still a valid box, just
 * a flat one.
 */
typedef struct {
  float xmin, xmax;    /* the bounding box, per axis                   */
  float ymin, ymax;
  float zmin, zmax;
  float cx, cy, cz;    /* box centre -- the point we centre on screen  */
  float radius;        /* rough size of the shape (half the diagonal)  */
} Bounds;

/*
 * Starfield -- the backdrop dots.  N_STARS fixed points scattered in a
 * box around the attractor.  They spin with the same camera as the orbit,
 * so as the view turns, far stars slide past near ones -- that motion is
 * what makes the scene read as 3-D, no real depth buffer needed.
 *
 * The box (±40 wide) is sized to comfortably wrap even the biggest preset
 * (Lorenz, ~±25); for the tiny ones it just looks like a wide starry sky.
 *
 * `initialised` lets us seed the stars exactly once -- the backdrop is the
 * same for every attractor, so switching presets leaves it untouched.
 */
#define STAR_BOX_HALF_EXTENT  40.0f   /* half-width of the x,y box        */
#define STAR_BOX_Z_MIN       -20.0f   /* near edge of the depth box       */
#define STAR_BOX_Z_RANGE      60.0f   /* depth of the box (far edge = +40)*/

typedef struct {
  float x[N_STARS];    /* where each star sits                          */
  float y[N_STARS];
  float z[N_STARS];
  bool  initialised;   /* seed them only the first time                 */
} Starfield;

/*
 * RenderCtx -- a few view numbers worked out once at the start of each
 * frame and handed to every draw helper, so they take (scene, ctx)
 * instead of a long list of loose values.  Rebuilt every frame because
 * the terminal can be resized or the preset switched at any time;
 * recomputing five numbers per frame costs nothing.
 *   cols, rows           the terminal size
 *   screen_cx/cy         where on screen the shape's centre goes
 *   scale                attractor units to screen cells
 *   depth_unit           the size yardstick the near/far shading uses
 */
typedef struct {
  int   cols, rows;       /* terminal size                            */
  int   screen_cx;        /* where the shape's centre lands, column   */
  int   screen_cy;        /* where the shape's centre lands, row      */
  float scale;            /* attractor units -> screen cells          */
  float depth_unit;       /* size yardstick for near/far shading      */
} RenderCtx;

/*
 * How far each shadow orbit starts from the main one.  The tiny gaps span
 * a wide range (0.001 up to 0.1) so you can watch them peel away at
 * different rates within one sitting.  On Lorenz the smallest gap grows to
 * fill the whole shape in about ten seconds -- chaos made visible; on the
 * flat maps the shadows just make extra pretty dots.
 */
static const float GHOST_EPS_TABLE[N_GHOSTS] = { 0.001f, 0.005f, 0.01f,
                                                 0.05f,  0.1f };

/*
 * Scene -- the whole visible world in one bundle, so one `Scene *` can be
 * passed around instead of a dozen separate things.  Fields are grouped by
 * what they're for: the moving orbits, which attractor and how big it is,
 * the camera and backdrop, the on/off toggles the keys flip, and the
 * numbers shown in the HUD.
 *
 * The trails make this large (~180 KB); it lives inside g_app (§10), never
 * on the stack.
 */
typedef struct {
  /* the moving orbits: one main, plus the shadows that drift away from it */
  Orbit main;
  Orbit ghosts[N_GHOSTS];

  /* which preset is showing, and how big it is */
  int    attr_idx;
  Bounds bounds;

  /* the view and the backdrop */
  Camera    camera;
  Starfield stars;

  /* toggles flipped by the keys */
  bool show_ghost;       /* g -- show the shadow orbits               */
  bool paused;           /* space -- freeze the motion                */

  /* numbers shown in the HUD */
  int       speed;       /* how fast the orbit runs (+/- keys)        */
  int       theme_idx;   /* which colour scheme is active             */
  long long total_pts;   /* points computed this run                  */
} Scene;

static inline const AttrDef *scene_current_attr(const Scene *s)
{
  return &ATTRS[s->attr_idx];
}

/* A tiny self-contained random generator, used only to scatter the stars.
 * Keeping our own avoids disturbing the shared rand() and gives the same
 * star layout every run. */
static unsigned int stars_lcg_next(unsigned int *s)
{
  *s = (*s) * 1103515245u + 12345u;
  return (*s) >> 16;
}

static inline float stars_lcg_unit(unsigned int *s)
{
  return (float)(stars_lcg_next(s) & 0xFFFF) / 65535.0f;
}

static void starfield_init(Starfield *sf)
{
  if (sf->initialised) return;
  unsigned int s = 987654321u;
  for (int i = 0; i < N_STARS; i++) {
    sf->x[i] = (stars_lcg_unit(&s) - 0.5f) * 2.0f * STAR_BOX_HALF_EXTENT;
    sf->y[i] = (stars_lcg_unit(&s) - 0.5f) * 2.0f * STAR_BOX_HALF_EXTENT;
    sf->z[i] = STAR_BOX_Z_MIN + stars_lcg_unit(&s) * STAR_BOX_Z_RANGE;
  }
  sf->initialised = true;
}

/* Near stars are drawn normally, far ones dimmed -- same near/far cutoff
 * the trail uses, so both layers fade together. */
static inline attr_t star_attr_by_depth(float depth, float depth_unit)
{
  return (depth < depth_unit * DEPTH_CLOSE_FRAC) ? A_NORMAL : A_DIM;
}

/* Draw the backdrop: each star, spun by the same camera as the orbit so
 * they all move together, dimmed by distance, painted as a faint dot. */
static void starfield_draw(const Starfield *sf, const Camera *cam,
                           const RenderCtx *ctx)
{
  for (int i = 0; i < N_STARS; i++) {
    int   col, row;
    float depth;
    if (!project(sf->x[i], sf->y[i], sf->z[i],
                 0.0f, 0.0f, 0.0f,    /* stars use absolute coords, not the shape's centre */
                 cam->phi, cam->theta, ctx->scale,
                 ctx->screen_cx, ctx->screen_cy, ctx->cols, ctx->rows,
                 &col, &row, &depth))
      continue;

    attr_t at = star_attr_by_depth(depth, ctx->depth_unit);

    attron(COLOR_PAIR(CP_TRAIL_TAIL) | at);
    mvaddch(row, col, '.');
    attroff(COLOR_PAIR(CP_TRAIL_TAIL) | at);
  }
}

/* Put the main orbit and every shadow at their starting points and clear
 * their trails.  The shadows start a hair off the main orbit in x and
 * nowhere else, so anything that happens afterwards is pure chaos pulling
 * them apart, not different starting setups. */
static void scene_seed_trajectories(Scene *s)
{
  const Point3 main_seed = { ORBIT_SEED, ORBIT_SEED, ORBIT_SEED };
  orbit_seed(&s->main, main_seed);

  for (int g = 0; g < N_GHOSTS; g++) {
    Point3 ghost_seed = { ORBIT_SEED + GHOST_EPS_TABLE[g],
                          ORBIT_SEED,
                          ORBIT_SEED };
    orbit_seed(&s->ghosts[g], ghost_seed);
  }
}

/* Measuring an attractor's size is four steps: settle onto it, sweep it
 * and track the extremes, pad the result, then work out its centre and
 * size.  One helper each so the driver reads as that recipe. */

/* Run a bunch of steps and throw them away -- the orbit starts off the
 * attractor and needs a moment to settle onto it before we measure. */
static void bbox_burn_in_transient(const AttrDef *at,
                                    float *px, float *py, float *pz)
{
  for (int i = 0; i < WARMUP_ITERS; i++)
    attractor_step(at, px, py, pz);
}

/* Sweep many more points, remembering the smallest and largest on each
 * axis.  Seeded from the first point so it works at any scale. */
static void bbox_probe_axis_extremes(const AttrDef *at,
                                      float *px, float *py, float *pz,
                                      float *xmin, float *xmax,
                                      float *ymin, float *ymax,
                                      float *zmin, float *zmax)
{
  *xmin = *xmax = *px;
  *ymin = *ymax = *py;
  *zmin = *zmax = *pz;
  for (int i = 0; i < BBOX_SAMPLES; i++) {
    attractor_step(at, px, py, pz);
    if (*px < *xmin) *xmin = *px;
    if (*px > *xmax) *xmax = *px;
    if (*py < *ymin) *ymin = *py;
    if (*py > *ymax) *ymax = *py;
    if (*pz < *zmin) *zmin = *pz;
    if (*pz > *zmax) *zmax = *pz;
  }
}

/* Pad the box a little on every side so the outermost points don't sit
 * right on the screen edge and get clipped. */
static void bbox_inflate_by_margin(Bounds *b,
                                    float xmin, float xmax,
                                    float ymin, float ymax,
                                    float zmin, float zmax)
{
  float x_pad = (xmax - xmin) * BBOX_MARGIN_FRAC;
  float y_pad = (ymax - ymin) * BBOX_MARGIN_FRAC;
  float z_pad = (zmax - zmin) * BBOX_MARGIN_FRAC;
  b->xmin = xmin - x_pad;  b->xmax = xmax + x_pad;
  b->ymin = ymin - y_pad;  b->ymax = ymax + y_pad;
  b->zmin = zmin - z_pad;  b->zmax = zmax + z_pad;
}

/* Work out the box's centre and rough size from its corners.  If the
 * shape collapsed to a point, fall back to a unit size so nothing later
 * divides by zero. */
static void bbox_derive_centre_and_radius(Bounds *b)
{
  b->cx = 0.5f * (b->xmin + b->xmax);
  b->cy = 0.5f * (b->ymin + b->ymax);
  b->cz = 0.5f * (b->zmin + b->zmax);

  float bw = b->xmax - b->xmin;
  float bh = b->ymax - b->ymin;
  float bd = b->zmax - b->zmin;
  b->radius = 0.5f * sqrtf(bw * bw + bh * bh + bd * bd);
  if (b->radius < BBOX_RADIUS_EPS) b->radius = BBOX_RADIUS_FALLBACK;
}

/* Measure the current attractor's size and centre into s->bounds.  Uses
 * a throwaway orbit, so the live orbits aren't disturbed. */
static void scene_calibrate_bounds(Scene *s)
{
  const AttrDef *at = scene_current_attr(s);
  float px = ORBIT_SEED, py = ORBIT_SEED, pz = ORBIT_SEED;

  bbox_burn_in_transient(at, &px, &py, &pz);

  float xmin, xmax, ymin, ymax, zmin, zmax;
  bbox_probe_axis_extremes(at, &px, &py, &pz,
                           &xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  bbox_inflate_by_margin(&s->bounds, xmin, xmax, ymin, ymax, zmin, zmax);
  bbox_derive_centre_and_radius(&s->bounds);
}

/* Start the current preset fresh: re-seed the orbits and re-measure its
 * size.  Run on preset change, the 'r' key, and resize. */
static void scene_reset(Scene *s)
{
  scene_seed_trajectories(s);
  scene_calibrate_bounds(s);
  s->total_pts = 0;
}

static void scene_init(Scene *s)
{
  memset(s, 0, sizeof *s);
  s->attr_idx           = 0;
  s->theme_idx          = 0;
  s->camera.phi         = VIEW_PHI_DEFAULT;
  s->camera.theta       = VIEW_THETA_DEFAULT;
  s->camera.auto_rotate = true;
  s->show_ghost         = true;
  s->paused             = false;
  s->speed              = 1;
  starfield_init(&s->stars);
  scene_reset(s);
}

/* If auto-rotate is on, nudge the spin angle a touch; otherwise leave it
 * alone (the user is steering with the arrow keys). */
static inline void camera_drift_auto_rotate(Camera *cam, float dt)
{
  if (cam->auto_rotate) cam->phi += VIEW_PHI_SPEED * dt;
}

/* How many orbit steps to run this tick.  Lorenz needs many tiny ones to
 * stay smooth; the maps take fewer bigger jumps.  Times the speed knob. */
static inline int iters_per_tick_for_family(const AttrDef *at, int speed)
{
  return at->is_continuous
       ? SUB_STEPS_ODE       * speed
       : ITERS_PER_TICK_MAP  * speed;
}

/* Step every orbit once.  Keeping them in lockstep is what makes the
 * shadows drifting apart meaningful -- they've all taken the same number
 * of steps, so any gap is the attractor pulling them apart, nothing else. */
static inline void step_all_trajectories_once(Scene *s, const AttrDef *at)
{
  orbit_step(&s->main, at);
  for (int g = 0; g < N_GHOSTS; g++)
    orbit_step(&s->ghosts[g], at);
}

/* Advance the world one tick: drift the view, then step every orbit the
 * right number of times.  dt (wall seconds) is only used for the drift. */
static void scene_tick(Scene *s, float dt)
{
  if (s->paused) return;

  camera_drift_auto_rotate(&s->camera, dt);

  const AttrDef *at = scene_current_attr(s);
  int iters = iters_per_tick_for_family(at, s->speed);

  for (int i = 0; i < iters; i++) {
    step_all_trajectories_once(s, at);
    s->total_pts++;
  }
}

/* Pick the zoom so the shape fills most of the screen but never spills
 * off as it spins.  Tries both width and height and takes the tighter
 * fit, allowing for the tall-cell aspect. */
static inline float compute_view_scale(const Bounds *b, int cols, int rows)
{
  int usable_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
  if (usable_rows < 1) usable_rows = 1;

  float diameter = 2.0f * b->radius;
  float scale_x = (float)cols        * VIEW_FILL_FRAC / diameter;
  float scale_y = (float)usable_rows * VIEW_FILL_FRAC / (diameter * ASPECT);
  return fminf(scale_x, scale_y);
}

static RenderCtx render_ctx_make(const Scene *s, int cols, int rows)
{
  RenderCtx ctx;
  ctx.cols       = cols;
  ctx.rows       = rows;
  ctx.screen_cx  = cols / 2;
  ctx.screen_cy  = HUD_TOP_ROWS
                 + (rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS) / 2;
  ctx.scale      = compute_view_scale(&s->bounds, cols, rows);
  ctx.depth_unit = s->bounds.radius;
  return ctx;
}

/* These small helpers each turn one fact about a trail point -- its age,
 * how close it is, whether it's a head or a shadow -- into one drawing
 * choice (which colour, how bright, which character).  draw_trail combines
 * them. */

/* Newer points get the brighter colour tier. */
static inline short trail_sample_color_pair(float age)
{
  if (age < AGE_HEAD_LIMIT) return CP_TRAIL_HEAD;
  if (age < AGE_MID_LIMIT)  return CP_TRAIL_MID;
  return CP_TRAIL_TAIL;
}

/* Brightness from how close the point is: near = bold, far = dim.  The
 * very front always glows bold, the oldest tail always fades. */
static inline attr_t trail_sample_depth_attr(float depth_norm, int k,
                                             float age)
{
  attr_t at;
  if      (depth_norm < DEPTH_CLOSE_FRAC) at = A_BOLD;
  else if (depth_norm > DEPTH_FAR_FRAC)   at = A_DIM;
  else                                    at = A_NORMAL;

  if      (k < BLOOM_HEAD_COUNT)          at = A_BOLD;  /* the glowing front */
  else if (age > AGE_TAIL_LIMIT)          at = A_DIM;   /* the fading end    */
  return at;
}

/* The newest point gets a marker; older ones are dots.  Shadows get their
 * own characters so they read as separate. */
static inline char trail_sample_glyph(bool is_ghost, int k)
{
  if (k == 0) return is_ghost ? 'x' : 'O';
  return is_ghost ? ',' : '.';
}

/* Stamp a little plus-shaped glow around the front of the trail. */
static inline void paint_bloom_halo(int row, int col, const RenderCtx *ctx)
{
  static const int hdr[4] = { -1,  1,  0,  0 };
  static const int hdc[4] = {  0,  0, -1,  1 };
  attron(COLOR_PAIR(CP_TRAIL_HEAD));
  for (int hi = 0; hi < 4; hi++) {
    int br = row + hdr[hi];
    int bc = col + hdc[hi];
    if (br >= HUD_TOP_ROWS && br < ctx->rows - HUD_BOTTOM_ROWS
        && bc >= 0 && bc < ctx->cols)
      mvaddch(br, bc, '+');
  }
  attroff(COLOR_PAIR(CP_TRAIL_HEAD));
}

/* A few more one-fact helpers used by draw_trail below. */

/* How old this point is, 0.0 newest to 1.0 oldest.  Guards a count of 0
 * or 1 so we never divide by zero. */
static inline float trail_sample_age_fraction(int k, int count)
{
  return (float)k / (float)(count > 1 ? count - 1 : 1);
}

/* Did this point land on the same cell as the last one we drew?  Dense
 * curves hit the same cell repeatedly; skipping the repeats saves work. */
static inline bool sample_overlaps_previous_cell(int col, int row,
                                                  int last_col, int last_row)
{
  return col == last_col && row == last_row;
}

/* Is this one of the front points that gets the glow halo? */
static inline bool is_comet_head_sample(int k)
{
  return k < BLOOM_HEAD_COUNT;
}

/* Pick colour + brightness for a main-trail point.  Distance is measured
 * against the shape's size so the near/far cutoffs work at any scale. */
static inline chtype compose_main_trail_attr(float depth, float depth_unit,
                                              int k, float age)
{
  float  depth_norm = depth / depth_unit;
  short  cp = trail_sample_color_pair(age);
  attr_t at = trail_sample_depth_attr(depth_norm, k, age);
  return COLOR_PAIR(cp) | at;
}

/* Shadow points are always the same: dim, in the ghost colour, so they
 * sit visually behind the main trail. */
static inline chtype compose_ghost_trail_attr(void)
{
  return COLOR_PAIR(CP_GHOST) | A_DIM;
}

/* Paint one cell, turning the colour on and back off so it can't leak.
 * The cast stops characters above 127 from being mistaken for negatives
 * (a classic ncurses gotcha). */
static inline void paint_trail_cell(int row, int col, chtype attr, char ch)
{
  attron(attr);
  mvaddch(row, col, (chtype)(unsigned char)ch);
  attroff(attr);
}

/*
 * Draw one trail, newest point to oldest.  is_ghost picks the faint
 * shadow look (one dim colour, no glow); the main trail gets the full
 * colour/brightness fade plus the glow around its front.
 */
static void draw_trail(const Scene *s, const Trail *t, bool is_ghost,
                       const RenderCtx *ctx)
{
  const Camera *cam = &s->camera;
  const Bounds *b   = &s->bounds;
  int last_col = CELL_NONE, last_row = CELL_NONE;

  for (int k = 0; k < t->count; k++) {
    Point3 p   = trail_at(t, k);
    float  age = trail_sample_age_fraction(k, t->count);

    int   col, row;
    float depth;
    if (!project(p.x, p.y, p.z,
                 b->cx, b->cy, b->cz,
                 cam->phi, cam->theta, ctx->scale,
                 ctx->screen_cx, ctx->screen_cy, ctx->cols, ctx->rows,
                 &col, &row, &depth))
      continue;                       /* off-screen */

    if (sample_overlaps_previous_cell(col, row, last_col, last_row))
      continue;                       /* same cell as last point */
    last_col = col;
    last_row = row;

    char ch = trail_sample_glyph(is_ghost, k);

    chtype attr = is_ghost
                ? compose_ghost_trail_attr()
                : compose_main_trail_attr(depth, ctx->depth_unit, k, age);

    paint_trail_cell(row, col, attr, ch);
    if (!is_ghost && is_comet_head_sample(k))
      paint_bloom_halo(row, col, ctx);
  }
}

/* Paint the whole frame back to front: stars, then shadows, then the main
 * trail on top.  The widest shadow is drawn first so closer ones land over
 * it.  The view numbers are worked out once here and shared. */
static void scene_draw(const Scene *s, int cols, int rows)
{
  RenderCtx ctx = render_ctx_make(s, cols, rows);

  starfield_draw(&s->stars, &s->camera, &ctx);

  if (s->show_ghost) {
    for (int g = N_GHOSTS - 1; g >= 0; g--)
      draw_trail(s, &s->ghosts[g].trail, true, &ctx);
  }

  draw_trail(s, &s->main.trail, false, &ctx);
}

/* ── §8  hud ── */

#define HUD_DATA_COL  24

static void hud_draw_title(void)
{
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(0, 0, " [STRANGE ATTRACTOR] ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw_state(const Scene *s)
{
  const AttrDef *at = scene_current_attr(s);
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, HUD_DATA_COL,
           " [%2d] %-11s  theme:%-7s  ghost:%-3s  rot:%-6s  %-7s ",
           s->attr_idx + 1, at->name,
           THEMES[s->theme_idx].name,
           s->show_ghost          ? "on"   : "off",
           s->camera.auto_rotate  ? "auto" : "manual",
           s->paused              ? "PAUSED " : "running");
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void hud_draw_params(const Scene *s, double fps)
{
  const AttrDef *at = scene_current_attr(s);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0,
           " params: a=%6.2f  b=%6.2f  c=%6.2f  d=%6.2f   speed:%dx   fps:%5.1f   pts:%lld ",
           (double)at->a, (double)at->b, (double)at->c, (double)at->d,
           s->speed, fps, s->total_pts);
  attroff(COLOR_PAIR(CP_HUD));
}

static void hud_draw_action_bar(int rows)
{
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/p:preset  1-9/0:jump  "
           "t/T:theme  g:ghost  a:rot  arrows:view  +/-:speed ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw(const Scene *s, int rows, double fps)
{
  hud_draw_title();
  hud_draw_state(s);
  hud_draw_params(s, fps);
  hud_draw_action_bar(rows);
}

/* ── §9  screen ── */

/*
 * Screen -- just the terminal's size in characters.  Kept apart from the
 * Scene on purpose: the simulation works in the attractor's own units and
 * the screen works in cells, and they only meet when we draw.  So resizing
 * the window can't disturb the running orbit.
 */
typedef struct {
  int cols;    /* terminal width in characters                          */
  int rows;    /* terminal height in characters                         */
} Screen;

static void screen_init(Screen *sc)
{
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s, double fps)
{
  erase();
  scene_draw(s, sc->cols, sc->rows);
  hud_draw(s, sc->rows, fps);
}

static void screen_present(void)
{
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10  app ── */

/*
 * App -- everything the program owns, in one bundle so a single pointer
 * can carry it through the loop.
 *
 * The two flags are set by signal handlers (Ctrl-C / window resize).  A
 * handler can't be passed anything, so there's one file-scope g_app it
 * writes to.  The flags are volatile sig_atomic_t so the handler can set
 * them safely and the loop is guaranteed to see the change.  Handlers
 * only flip a flag; the real work happens back in the loop, because
 * things like ncurses cleanup aren't safe to do inside a handler.
 *   running       cleared to make the loop exit cleanly
 *   need_resize   set on a window resize; the loop re-fits next frame
 */
typedef struct {
  Scene                 scene;
  Screen                screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
  screen_resize(&app->screen);
  app->need_resize = 0;
}

/* One small function per key, so app_handle_key reads like the key list. */

static void action_pause            (Scene *s) { s->paused = !s->paused; }
static void action_reset            (Scene *s) { scene_reset(s); }
static void action_toggle_ghost     (Scene *s) { s->show_ghost         = !s->show_ghost;         }
static void action_toggle_auto_rot  (Scene *s) { s->camera.auto_rotate = !s->camera.auto_rotate; }

static void action_preset_next(Scene *s)
{
  s->attr_idx = (s->attr_idx + 1) % N_ATTRS;
  scene_reset(s);
}

static void action_preset_prev(Scene *s)
{
  s->attr_idx = (s->attr_idx + N_ATTRS - 1) % N_ATTRS;
  scene_reset(s);
}

static void action_preset_jump(Scene *s, int idx)
{
  if (idx < 0)        idx = 0;
  if (idx >= N_ATTRS) idx = N_ATTRS - 1;
  if (idx == s->attr_idx) return;
  s->attr_idx = idx;
  scene_reset(s);
}

static void action_theme_next(Scene *s)
{
  s->theme_idx = (s->theme_idx + 1) % N_THEMES;
  theme_apply(s->theme_idx);
}

static void action_theme_prev(Scene *s)
{
  s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
  theme_apply(s->theme_idx);
}

static void action_phi_left(Scene *s)
{
  s->camera.auto_rotate = false;
  s->camera.phi -= VIEW_PHI_STEP;
}

static void action_phi_right(Scene *s)
{
  s->camera.auto_rotate = false;
  s->camera.phi += VIEW_PHI_STEP;
}

static void action_theta_up(Scene *s)
{
  s->camera.theta += VIEW_THETA_STEP;
  if (s->camera.theta > VIEW_THETA_MAX) s->camera.theta = VIEW_THETA_MAX;
}

static void action_theta_down(Scene *s)
{
  s->camera.theta -= VIEW_THETA_STEP;
  if (s->camera.theta < VIEW_THETA_MIN) s->camera.theta = VIEW_THETA_MIN;
}

static void action_speed_faster(Scene *s)
{
  s->speed *= 2;
  if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
}

static void action_speed_slower(Scene *s)
{
  s->speed /= 2;
  if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
}

/* Returns false if the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
  Scene *s = &app->scene;
  switch (ch) {
  case 'q': case 'Q': case KEY_ESC:  return false;

  case ' ':           action_pause(s);           break;
  case 'r': case 'R': action_reset(s);           break;
  case 'n': case 'N': action_preset_next(s);     break;
  case 'p': case 'P': action_preset_prev(s);     break;
  case 't':           action_theme_next(s);      break;
  case 'T':           action_theme_prev(s);      break;
  case 'g': case 'G': action_toggle_ghost(s);    break;
  case 'a': case 'A': action_toggle_auto_rot(s); break;

  case KEY_LEFT:      action_phi_left(s);        break;
  case KEY_RIGHT:     action_phi_right(s);       break;
  case KEY_UP:        action_theta_up(s);        break;
  case KEY_DOWN:      action_theta_down(s);      break;

  case '+': case '=': action_speed_faster(s);    break;
  case '-': case '_': action_speed_slower(s);    break;

  default:
    /* number keys jump straight to a preset; '0' is the tenth */
    if (ch >= '1' && ch <= '9' && (ch - '1') < N_ATTRS)
      action_preset_jump(s, ch - '1');
    else if (ch == '0' && N_ATTRS >= 10)
      action_preset_jump(s, 9);
    break;
  }
  return true;
}

/* The frame loop is a handful of steps; each is its own little function
 * below so main() reads as the sequence and the timing math stays out of
 * the way. */

/* Restore the terminal on exit, and catch Ctrl-C / kill / resize. */
static void install_signal_handlers(void)
{
  atexit(cleanup);
  signal(SIGINT,   on_exit_signal);
  signal(SIGTERM,  on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

static void app_init(App *app)
{
  app->running     = 1;
  app->need_resize = 0;
  screen_init(&app->screen);
  scene_init(&app->scene);
}

/* If the window was resized, re-read its size.  The clock is reset too so
 * the time spent resizing doesn't get counted as simulation owed. */
static inline void app_service_pending_resize(App *app, int64_t *frame_time,
                                               int64_t *sim_accum)
{
  if (!app->need_resize) return;
  app_do_resize(app);
  *frame_time = clock_ns();
  *sim_accum  = 0;
}

/* Time since the last frame, capped.  The cap matters: if the program was
 * paused for a while (debugger, suspended terminal), without it the next
 * frame would try to catch up thousands of steps at once and lock up. */
static inline int64_t frame_dt_capped(int64_t now, int64_t prev)
{
  int64_t dt = now - prev;
  return (dt > MAX_FRAME_DT_NS) ? MAX_FRAME_DT_NS : dt;
}

/* Run the simulation in fixed-size ticks: bank the elapsed time, then
 * spend it one tick at a time.  This keeps the motion running at the same
 * pace no matter how fast or jerky the drawing is. */
static inline void pump_fixed_simulation(Scene *scene, int64_t *sim_accum,
                                          int64_t dt)
{
  *sim_accum += dt;
  while (*sim_accum >= TICK_NS) {
    scene_tick(scene, FIXED_TICK_DT_SEC);
    *sim_accum -= TICK_NS;
  }
}

/* Update the fps number shown in the HUD, but only twice a second, so it
 * holds still long enough to read instead of flickering every frame. */
static inline void fps_counter_update(int64_t dt, int64_t *fps_accum,
                                       int *frame_count, double *fps_display)
{
  (*frame_count)++;
  *fps_accum += dt;
  if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
    *fps_display = (double)(*frame_count) /
                   ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
  }
}

/* Wait out the rest of this frame's time budget so we hold ~60 fps.  If
 * the frame already ran long, the sleep amount goes negative and the sleep
 * just returns at once. */
static inline void throttle_to_target_fps(int64_t frame_start, int64_t dt)
{
  int64_t elapsed = clock_ns() - frame_start + dt;
  clock_sleep_ns(TICK_NS - elapsed);
}

/* Draw the frame and push it to the screen.  The only place we write to
 * the terminal. */
static inline void present_frame(App *app, double fps_display)
{
  screen_draw(&app->screen, &app->scene, fps_display);
  screen_present();
}

/* Check for a keypress without blocking and act on it; one key per frame
 * is plenty at 60 fps.  Quits the loop if the user asked to. */
static inline void pump_one_keystroke(App *app)
{
  int ch = getch();
  if (ch != ERR && !app_handle_key(app, ch))
    app->running = 0;
}

int main(void)
{
  install_signal_handlers();

  App *app = &g_app;
  app_init(app);

  int64_t frame_time  = clock_ns();
  int64_t sim_accum   = 0;
  int64_t fps_accum   = 0;
  int     frame_count = 0;
  double  fps_display = 0.0;

  while (app->running) {
    app_service_pending_resize(app, &frame_time, &sim_accum);

    int64_t now = clock_ns();
    int64_t dt  = frame_dt_capped(now, frame_time);
    frame_time  = now;

    pump_fixed_simulation(&app->scene, &sim_accum, dt);
    fps_counter_update(dt, &fps_accum, &frame_count, &fps_display);
    throttle_to_target_fps(frame_time, dt);
    present_frame(app, fps_display);
    pump_one_keystroke(app);
  }

  screen_free(&app->screen);
  return 0;
}
