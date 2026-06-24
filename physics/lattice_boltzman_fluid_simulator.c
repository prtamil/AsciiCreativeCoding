/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_boltzman_fluid_simulator.c
 *
 * A 2-D fluid flowing left-to-right past an obstacle, drawn one terminal
 * cell per grid point.  Give it ~30 seconds and you see the classic
 * vortex street: swirls peeling off the obstacle in an alternating
 * left/right pattern as they drift downstream.  Three views (speed,
 * spin, pressure), ten obstacle shapes, ten colour themes.  The method
 * is Lattice Boltzmann (D2Q9, BGK collision) — instead of tracking
 * molecules, each grid point tracks how much "stuff" is moving in nine
 * directions, and the real fluid behaviour falls out of that.
 *
 * Good references if you want the theory the code can't give you:
 *   Krüger et al. (2017), The Lattice Boltzmann Method: Principles and
 *     Practice (Springer) — the code mirrors its Ch. 3–4.
 *   Succi (2001), The Lattice Boltzmann Equation for Fluid Dynamics.
 *   Williamson (1996), Vortex Dynamics in the Cylinder Wake — the
 *     vortex-street physics this demo reproduces.
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

/* ── §1 config ── */

enum {
  SIM_FPS_DEFAULT = 60,
  TARGET_FPS = 30, /* painting more often than this isn't worth it — the
                    * fluid is the slow part */
  FPS_UPDATE_MS = 500,

  /* How many fluid steps to run between screen redraws.  The fluid is
   * slow (every step touches every cell), so we run a few steps per
   * frame to make the flow evolve at a watchable pace without redrawing
   * faster than the eye needs.  Fewer = too sluggish, more = fps drops. */
  STEPS_PER_FRAME = 4,

  /* The three views (speed / spin / pressure), cycled by v/o/d. */
  VIS_VELOCITY = 0,
  VIS_VORTICITY = 1,
  VIS_DENSITY = 2,
  VIS_COUNT = 3,

  /* Colour-pair slots.  CP_VEL0..7 and the two vorticity pairs change
   * with the chosen theme; the last four (solid, HUD, hint, arrow) stay
   * fixed.  Each vorticity sign uses ONE pair — brightness, not a third
   * pair, carries the strength. */
  CP_VEL0 = 1, /* coldest end of velocity ramp                    */
  CP_VEL1 = 2,
  CP_VEL2 = 3,
  CP_VEL3 = 4,
  CP_VEL4 = 5,
  CP_VEL5 = 6,
  CP_VEL6 = 7,
  CP_VEL7 = 8,   /* hottest end of velocity ramp                    */
  CP_VNEG = 9,   /* vorticity clockwise — 3 levels via attr         */
  CP_VPOS = 10,  /* vorticity CCW       — 3 levels via attr         */
  CP_SOLID = 11, /* obstacle / wall (theme-independent gray)        */
  CP_HUD = 12,   /* status bar — canonical bright yellow (226)      */
  CP_HINT = 13,  /* action keys — canonical bright cyan  (51)       */
  CP_ARROW = 14, /* streamline arrows                                */
};

/* How fast the fluid enters from the left.  The method only stays honest
 * when the flow is slow compared to its internal "speed of sound", so
 * this has to stay small; push it too high and the numbers go bad.  The
 * w/W keys nudge it (capped at 0.30). */
#define U_IN_DEFAULT 0.10f /* inlet speed, in the sim's own units (keep small) */

/* The single thickness/viscosity knob (τ).  Lower = thinner, livelier
 * fluid; higher = thicker, more sluggish.  The default sits right at the
 * sweet spot where the obstacle starts shedding its rhythmic vortices. */
#define TAU_DEFAULT 0.60f

/* Don't let τ drop to 0.5 or below: the viscosity it implies goes to
 * zero or negative there and the simulation blows up in a single step.
 * The tiny margin keeps a cushion against rounding nudging it under. */
#define TAU_MIN 0.505f

/* Upper limit on τ — at this thickness the fluid oozes like honey, far
 * too sluggish to shed any vortices.  Handy as a "see what very thick
 * looks like" extreme. */
#define TAU_MAX 2.0f

/* How much one +/- keypress changes τ.  Kept small so a tap nudges the
 * flow rather than jolting a running simulation into instability. */
#define TAU_STEP 0.025f

/* Obstacle's left-right placement: a quarter of the way across.  Leaves
 * room for the incoming flow to settle before it hits the obstacle, and
 * three-quarters of the screen downstream for the wake to play out. */
#define CYL_X_FRAC 0.25f

/* Obstacle's vertical placement: dead centre.  A perfectly centred
 * obstacle in a symmetric channel would never shed vortices on its own;
 * a tiny random nudge added at startup breaks that symmetry. */
#define CYL_Y_FRAC 0.50f

/* Obstacle size as a fraction of screen height.  Bigger terminals make
 * a bigger obstacle, which pushes the flow past the threshold where
 * vortex shedding kicks in — so the bigger the window, the livelier. */
#define CYL_R_FRAC 0.08f

/* The 8 characters used to draw speed/pressure, faint (space) to bold (@). */
static const char VEL_CHARS[8] = {' ', '.', ':', '+', 'x', 'X', '#', '@'};

/* Arrow glyphs for the direction overlay, one per compass heading. */
static const char ARROW8[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ── §2 clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* ── §3 color / theme ── */

/*
 * Theme — a colour scheme, which here is just 8 colours that run cold to
 * hot.  That single ramp does all the colouring.
 *
 *   name  — what the HUD shows for it.
 *   ramp  — 8 palette numbers, ramp[0] the coolest, ramp[7] the
 *           hottest.  Speed and pressure map straight onto these 8.
 *           Spin reuses the two ends — cool colour for one rotation
 *           direction, hot for the other — so the whole picture shares
 *           one colour story.  Brightness (dim/normal/bold), not extra
 *           colours, shows how strong the spin is, so each rotation
 *           direction needs only one colour.
 *
 * Four other colours stay the same no matter the theme: yellow for the
 * status line, cyan for the key hints, gray for the obstacle, white for
 * the arrows (set up in color_init).
 */
typedef struct {
  const char *name;
  short ramp[8]; /* 8 colours, coolest to hottest                   */
} Theme;

/*
 * The 10 themes.  Each is 8 numbers picked by eye along a smooth
 * cold-to-hot path through the terminal's 256-colour palette.
 *
 *   Ocean    — deep blue   to bright cyan
 *   Sunset   — indigo      to gold
 *   Forest   — dark green  to yellow-green
 *   Fire     — dark red    to bright yellow
 *   Lava     — navy        to bright red
 *   Aurora   — purple      to green (through blue / cyan)
 *   Plasma   — magenta     to yellow (electric)
 *   Arctic   — dusky gray  to ice cyan
 *   Mono     — pure greyscale
 *   Toxic    — dark green  to lime
 */
static const Theme THEMES[] = {
    /* name       0    1    2    3    4    5    6    7      (cold → hot) */
    {"Ocean", {17, 18, 24, 25, 31, 38, 45, 51}},
    {"Sunset", {17, 53, 91, 161, 167, 173, 215, 220}},
    {"Forest", {22, 28, 34, 64, 100, 142, 178, 226}},
    {"Fire", {52, 88, 124, 160, 196, 202, 208, 220}},
    {"Lava", {18, 19, 91, 125, 161, 196, 202, 226}},
    {"Aurora", {54, 91, 99, 105, 81, 49, 47, 82}},
    {"Plasma", {53, 91, 161, 199, 207, 213, 220, 226}},
    {"Arctic", {59, 60, 67, 74, 81, 123, 159, 195}},
    {"Mono", {232, 235, 238, 242, 246, 250, 253, 255}},
    {"Toxic", {22, 28, 34, 40, 76, 112, 118, 154}},
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/* Which theme is active, cycled by t/T.  A global because the theme is
 * purely a look choice — it has nothing to do with the fluid — and one
 * global lets both startup and the keypress handler call theme_apply(). */
static int g_theme_idx = 0;

/* Loads one theme's colours into the colour slots.  The fixed colours
 * (status, hint, obstacle, arrow) are left alone — those live in
 * color_init.  On old 8-colour terminals there's no 256-colour palette,
 * so we fall back to a plain blue-to-red ramp and the demo still runs. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    return;
  const Theme *t = &THEMES[idx];

  if (COLORS >= 256) {
    for (int i = 0; i < 8; i++)
      init_pair((short)(CP_VEL0 + i), t->ramp[i], -1);
    init_pair(CP_VNEG, t->ramp[0], -1); /* one spin direction: cool end  */
    init_pair(CP_VPOS, t->ramp[7], -1); /* the other:          hot  end  */
  } else {
    /* old 8-colour terminal: every theme falls back to this one ramp */
    init_pair(CP_VEL0, COLOR_BLUE, -1);
    init_pair(CP_VEL1, COLOR_BLUE, -1);
    init_pair(CP_VEL2, COLOR_CYAN, -1);
    init_pair(CP_VEL3, COLOR_CYAN, -1);
    init_pair(CP_VEL4, COLOR_GREEN, -1);
    init_pair(CP_VEL5, COLOR_YELLOW, -1);
    init_pair(CP_VEL6, COLOR_RED, -1);
    init_pair(CP_VEL7, COLOR_RED, -1);
    init_pair(CP_VNEG, COLOR_BLUE, -1);
    init_pair(CP_VPOS, COLOR_RED, -1);
  }
}

/* Sets up colour once at startup: the four fixed colours here, then the
 * theme-dependent ones via theme_apply. */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1);   /* canonical bright yellow */
    init_pair(CP_HINT, 51, -1);   /* canonical bright cyan   */
    init_pair(CP_SOLID, 240, -1); /* mid gray (theme-neutral)*/
    init_pair(CP_ARROW, 255, -1); /* near-white               */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_SOLID, COLOR_WHITE, -1);
    init_pair(CP_ARROW, COLOR_WHITE, -1);
  }

  theme_apply(g_theme_idx);
}

/* ── §4 D2Q9 model constants ── */

/*
 * The nine directions stuff can move at each grid point: stay put, the
 * four straight neighbours, and the four diagonals.  EX/EY are the step
 * for each one (+y is screen-down).  W is how much weight each direction
 * carries (the straight ones count more than the diagonals, the rest
 * point counts most).  OPP[i] is the exact reverse of direction i — used
 * when something bounces off a wall and has to head back the way it came.
 *
 *   Dir   (ex, ey)   weight      Opposite
 *    0    ( 0,  0)   4/9         0
 *    1    ( 1,  0)   1/9         3
 *    2    ( 0,  1)   1/9         4   (+y = screen-down)
 *    3    (-1,  0)   1/9         1
 *    4    ( 0, -1)   1/9         2
 *    5    ( 1,  1)   1/36        7
 *    6    (-1,  1)   1/36        8
 *    7    (-1, -1)   1/36        5
 *    8    ( 1, -1)   1/36        6
 */
static const int EX[9] = {0, 1, 0, -1, 0, 1, -1, -1, 1};
static const int EY[9] = {0, 0, 1, 0, -1, 1, 1, -1, -1};
static const float W[9] = {4.0f / 9.0f,  1.0f / 9.0f,  1.0f / 9.0f,
                           1.0f / 9.0f,  1.0f / 9.0f,  1.0f / 36.0f,
                           1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f};
static const int OPP[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

/*
 * The "settled" amount of stuff that should be moving in direction i if
 * the fluid at this point — with density rho and velocity (ux,uy) — were
 * perfectly relaxed.  This is the target the real values keep drifting
 * toward; that drift is what makes the fluid behave like a fluid.
 *
 *   f^eq_i = w_i · ρ · [1 + 3(e·u) + 4.5(e·u)² − 1.5|u|²]
 *
 * In words: start from the rest amount (w_i·ρ), then tilt it toward the
 * flow direction (the e·u terms push more stuff the way the fluid is
 * already heading) and trim back for overall speed so nothing is created
 * or lost.  The magic numbers 3, 4.5, 1.5 come from this lattice's fixed
 * internal speed of sound (cs² = 1/3); see Krüger et al. Ch. 3.
 */
static inline float feq(int i, float rho, float ux, float uy) {
  float eu = (float)EX[i] * ux + (float)EY[i] * uy;
  float u2 = ux * ux + uy * uy;
  return W[i] * rho * (1.0f + 3.0f * eu + 4.5f * eu * eu - 1.5f * u2);
}

/* ── §5 grid ── */

/*
 * LBM — the whole fluid world in one struct: the lattice buffers, the
 * three physics knobs, the obstacle shape, the per-frame stats, and the
 * view toggles.  Everything has the same lifetime (born at lbm_alloc,
 * dies at lbm_free, refit on resize), so one `LBM *g` argument can carry
 * the entire world through collide / stream / render without a dozen
 * loose parameters.
 *
 * It's deliberately one struct, not a separate sim half and render half:
 * the stats (peak speed, density range, …) are produced by the simulation
 * and read by the renderer, so splitting them would just mean shuttling a
 * shared bundle back and forth.  The fields are grouped below — geometry,
 * buffers, physics, obstacle, stats, view toggles — so reading top to
 * bottom tells you everything one frame needs.
 *
 * References for the ideas behind the fields: Krüger and Succi [1, 2] for
 * the overall method, [3] for the viscosity knob, [7] for the obstacle's
 * Reynolds number.
 */
typedef struct {

  /* ── grid size ── */

  int nx, ny;
  /* How many grid points wide and tall.  nx = terminal columns,
   * ny = terminal rows minus 2 (the top status row and bottom key
   * row).  scene_resize recomputes these and re-allocates every
   * buffer below when the window changes size; the loops everywhere
   * count up to these, so resizing just works.                  */

  /* ── lattice buffers (all on the heap, one entry per grid point) ── */

  float *f;
  /* The heart of the method: for every grid point, how much "stuff"
   * is currently moving in each of the 9 directions.  Stored as
   * f[(y·nx + x)·9 + i] — all 9 directions of one point sit next to
   * each other in memory, which is what collide() wants since it
   * touches all 9 at a point before moving on.  About 430 KB for a
   * 200×60 grid. [1, 2]                                          */

  float *ftmp;
  /* Scratch copy used while moving stuff to neighbours.  The move
   * step reads from f and writes here, then we just swap the two
   * pointers — no copying.  The separate buffer is what stops a
   * value from being moved twice in one pass, which would quietly
   * corrupt the flow. [2]                                        */

  float *rho;
  /* Density at each point — how much fluid is there.  Found by adding
   * up all 9 directions.  Fed back into the physics, and used as the
   * pressure picture in density view (pressure is just density/3 in
   * these units).                                               */

  float *ux, *uy;
  /* Which way and how fast the fluid is moving at each point (x and y
   * parts).  Found from the 9 direction amounts.  Used by the physics,
   * the spin calculation, the speed picture, and the flow arrows.  */

  float *vort;
  /* Spin (vorticity) at each point — how fast a little fluid parcel is
   * rotating, and which way.  Only filled in when the spin view is on,
   * since it costs an extra pass over the whole grid.            */

  uint8_t *solid;
  /* The obstacle-and-walls map: 1 = solid, 0 = fluid.  Painted by the
   * current shape's pat_* function (cylinder, square, airfoil, …).
   * The move step bounces fluid off solid cells, collide skips them,
   * and every view draws them as '#'. [2]                        */

  /* ── physics knobs ── */

  float tau;
  /* The one thickness knob.  Lower means thinner, livelier fluid;
   * higher means thicker and more sluggish.  The +/- keys nudge it,
   * kept inside [TAU_MIN, TAU_MAX].  Too close to 0.5 and the fluid
   * has effectively zero thickness and the numbers blow up. [3]  */

  float omega;
  /* Just 1/tau, kept around because the inner collision loop multiplies
   * by it for every point every step, and a multiply is much cheaper
   * than redoing the division each time.  Updated whenever tau changes. */

  float u_in;
  /* How fast fluid enters from the left.  Has to stay small — the
   * method only behaves while the flow is slow compared to its own
   * internal speed of sound, and past ~0.3 the errors grow fast.  The
   * w/W keys change it, capped at 0.30.                          */

  /* ── obstacle placement ── */

  int cyl_x, cyl_y, cyl_r;
  /* Where the obstacle sits and how big it is, worked out once from the
   * CYL_*_FRAC fractions times the grid size, so the obstacle scales
   * with the terminal.  Every shape uses these as its size reference (a
   * square's half-width is cyl_r, a diamond's reach is cyl_r, …), so
   * the 10 shapes all take up about the same room.  The HUD's Reynolds
   * number uses the obstacle's width, 2·cyl_r. [7]               */

  /* ── per-frame stats (the simulation fills these; the views read them) ── */

  float vel_max;
  /* Fastest speed anywhere this frame.  The speed view divides by it so
   * the colours span the full range from slow to fast.           */

  float rho_min, rho_max;
  /* Lowest and highest density this frame.  The density view stretches
   * its colours across this range so the tiny pressure differences
   * (often around 0.001) still show up.                          */

  float vort_max;
  /* Strongest spin anywhere this frame (only filled in spin view).
   * The spin view divides by it to pick colour strength.         */

  int step_count;
  /* How many physics steps since the last fresh start.  Just a readout
   * in the HUD so you can tell whether the flow has had enough time to
   * settle after a change.                                       */

  /* ── view toggles (the keys set these; the renderer reads them) ── */
  /* Changing a view never touches the physics — it only changes what */
  /* gets drawn.                                                      */

  int vis_mode;
  /* Which picture to show: speed, spin, or density.  Set by the v/o/d
   * keys.  scene_tick checks it to decide whether to also compute spin. */

  bool show_stream;
  /* Whether the flow-arrow overlay is on (s key).               */

  bool paused;
  /* When true the physics is frozen but drawing continues, so you can
   * study a moment of the flow while it holds still.             */
} LBM;

static int lbm_alloc(LBM *g, int nx, int ny) {
  g->nx = nx;
  g->ny = ny;
  int n = nx * ny;
  g->f = calloc(n * 9, sizeof(float));
  g->ftmp = calloc(n * 9, sizeof(float));
  g->rho = calloc(n, sizeof(float));
  g->ux = calloc(n, sizeof(float));
  g->uy = calloc(n, sizeof(float));
  g->vort = calloc(n, sizeof(float));
  g->solid = calloc(n, sizeof(uint8_t));
  return (g->f && g->ftmp && g->rho && g->ux && g->uy && g->vort && g->solid)
             ? 0
             : -1;
}

static void lbm_free(LBM *g) {
  free(g->f);
  free(g->ftmp);
  free(g->rho);
  free(g->ux);
  free(g->uy);
  free(g->vort);
  free(g->solid);
}

/* ── the obstacle shapes ──
 *
 * Each pat_* function paints a different obstacle into the solid map;
 * nothing else about the fluid changes.  That's the whole point of
 * having shapes — you watch the same physics produce different wakes
 * behind different shapes [2, 7].
 *
 * They all build on three little helpers: pat_walls clears the map and
 * lays the top/bottom walls, stamp_disc fills a circle, stamp_rect fills
 * a rectangle.  Most shapes size themselves off the cyl_x/y/r reference
 * point so they all take up about the same room.                        */

static void pat_walls(LBM *g) {
  int nx = g->nx, ny = g->ny;
  memset(g->solid, 0, nx * ny);
  for (int x = 0; x < nx; x++) {
    g->solid[0 * nx + x] = 1;
    g->solid[(ny - 1) * nx + x] = 1;
  }
}

static void stamp_disc(LBM *g, int cx, int cy, int r) {
  int nx = g->nx, ny = g->ny;
  int r2 = r * r;
  for (int y = 0; y < ny; y++)
    for (int x = 0; x < nx; x++) {
      int dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy <= r2)
        g->solid[y * nx + x] = 1;
    }
}

static void stamp_rect(LBM *g, int x0, int y0, int x1, int y1) {
  int nx = g->nx, ny = g->ny;
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 >= nx)
    x1 = nx - 1;
  if (y1 >= ny)
    y1 = ny - 1;
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++)
      g->solid[y * nx + x] = 1;
}

/* ── the shapes ── */

/* Cylinder — the classic.  Past a certain flow speed it sheds the
 * famous alternating left/right swirls (the Kármán vortex street). [7] */
static void pat_cylinder(LBM *g) {
  pat_walls(g);
  stamp_disc(g, g->cyl_x, g->cyl_y, g->cyl_r);
}

/* Square — its sharp front corners make the flow peel off right away,
 * giving a wider wake than a cylinder of the same width. */
static void pat_square(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_rect(g, cx - r, cy - r, cx + r, cy + r);
}

/* Diamond — a square turned 45° so a point faces upstream.  The flow
 * splits cleanly at the nose and the wake is narrower than the square's. */
static void pat_diamond(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  for (int y = 0; y < g->ny; y++)
    for (int x = 0; x < g->nx; x++) {
      int dx = x - cx, dy = y - cy;
      int adx = dx < 0 ? -dx : dx;
      int ady = dy < 0 ? -dy : dy;
      if (adx + ady <= r)
        g->solid[y * g->nx + x] = 1;
    }
}

/* Airfoil — a long thin ellipse tilted about 12°, like a wing at a
 * slight angle.  The flow bends downward behind it — that downward kick
 * is the same thing that gives a wing its lift. */
static void pat_airfoil(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y;
  float r = (float)g->cyl_r;
  float c = cosf(0.2094f), s = sinf(0.2094f); /* 12° in radians */
  float a = 3.0f * r, b = 0.5f * r;
  float ia2 = 1.0f / (a * a), ib2 = 1.0f / (b * b);
  for (int y = 0; y < g->ny; y++)
    for (int x = 0; x < g->nx; x++) {
      float dx = (float)(x - cx), dy = (float)(y - cy);
      float u = c * dx + s * dy; /* turn the point into the wing's own tilted frame */
      float v = -s * dx + c * dy;
      if (u * u * ia2 + v * v * ib2 <= 1.0f)
        g->solid[y * g->nx + x] = 1;
    }
}

/* Twin — two equal cylinders one behind the other.  The back one sits in
 * the front one's wake, which changes how both shed swirls. */
static void pat_twin(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_disc(g, cx, cy, r);
  stamp_disc(g, cx + 4 * r, cy, r);
}

/* Array — a 3×3 grid of small cylinders, like flow through a bundle of
 * pipes.  The fluid speeds up squeezing through the gaps and swirls
 * behind each one. */
static void pat_array(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  int rs = r / 2;
  if (rs < 1)
    rs = 1;
  for (int ry = -1; ry <= 1; ry++)
    for (int rx = -1; rx <= 1; rx++)
      stamp_disc(g, cx + 2 * r * rx, cy + 2 * r * ry, rs);
}

/* Step — a block fills the top part of the channel, then suddenly ends.
 * Right past the drop-off the flow can't follow the corner, so it curls
 * into a steady trapped swirl — a classic textbook case. */
static void pat_step(LBM *g) {
  pat_walls(g);
  int step_h = (int)(g->ny * 0.4f);
  stamp_rect(g, 0, 0, 2 * g->cyl_x, step_h);
}

/* Venturi — wedges from top and bottom pinch the channel to a narrow
 * throat in the middle.  The fluid has to speed up to get through, and
 * its pressure drops where it's fastest — easiest to see in density view. */
static void pat_venturi(LBM *g) {
  pat_walls(g);
  int nx = g->nx, ny = g->ny;
  int cx = g->cyl_x;
  int half_gap = g->cyl_r;
  int wedge_len = 3 * g->cyl_r;
  int wall_max = ny / 2 - half_gap;
  if (wall_max < 1)
    wall_max = 1;
  for (int x = 0; x < nx; x++) {
    int d = x - cx;
    if (d < 0)
      d = -d;
    if (d > wedge_len)
      continue;
    int taper = wedge_len - d; /* peaks at center */
    int wall = taper * wall_max / wedge_len;
    for (int y = 0; y <= wall; y++)
      g->solid[y * nx + x] = 1;
    for (int y = ny - 1 - wall; y < ny; y++)
      g->solid[y * nx + x] = 1;
  }
}

/* Splitter — a cylinder with a thin plate trailing straight out behind
 * it.  The plate keeps the upper and lower halves of the wake from
 * talking to each other, so the swirling stops and the wake goes calm. */
static void pat_splitter(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_disc(g, cx, cy, r);
  stamp_rect(g, cx + r, cy, cx + r + 4 * r, cy);
}

/* Wedge — a triangle with its point facing upstream and its flat back
 * facing downstream.  Both back corners shed swirls evenly, giving a
 * symmetric wake. */
static void pat_wedge(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  int x0 = cx - r, x1 = cx + r;
  for (int x = x0; x <= x1; x++) {
    if (x < 0 || x >= g->nx)
      continue;
    int progress = x - x0; /* 0 at vertex, 2r at base */
    int half_h = progress / 2;
    for (int y = cy - half_h; y <= cy + half_h; y++)
      if (y >= 0 && y < g->ny)
        g->solid[y * g->nx + x] = 1;
  }
}

/* ── shape table + picker ── */

/*
 * Pattern — one obstacle shape you can cycle to with n/p: a display name
 * plus the function that paints it.  Keeping them in a table means adding
 * a new shape is just one more row here — no switch statements to update.
 *
 * The build function takes the whole LBM because it needs the grid size
 * and the obstacle reference point to place and size itself.  Every build
 * function starts by calling pat_walls so the caller never has to
 * remember the channel walls.  Williamson [7] catalogues how these
 * different shapes behave in real wind-tunnel tests.
 */
typedef struct {
  const char *name; /* what the HUD shows: "Cylinder", "Airfoil", … */
  void (*build)(LBM *g);
  /* paints this shape into g->solid[]; starts
   * by calling pat_walls for the channel walls. */
} Pattern;

static const Pattern PATTERNS[] = {
    {"Cylinder", pat_cylinder}, /* 0 — the classic, sheds swirls    */
    {"Square", pat_square},     /* 1 — sharp front corners          */
    {"Diamond", pat_diamond},   /* 2 — square turned 45°            */
    {"Airfoil", pat_airfoil},   /* 3 — tilted wing, shows lift      */
    {"Twin", pat_twin},         /* 4 — two cylinders in a row       */
    {"Array", pat_array},       /* 5 — 3x3 grid of small cylinders  */
    {"Step", pat_step},         /* 6 — sudden drop-off in the wall  */
    {"Venturi", pat_venturi},   /* 7 — pinched-in throat            */
    {"Splitter", pat_splitter}, /* 8 — cylinder with a trailing fin */
    {"Wedge", pat_wedge},       /* 9 — triangle, point upstream     */
};
#define N_PATTERNS ((int)(sizeof PATTERNS / sizeof PATTERNS[0]))

/* Which shape is active right now — the n/p keys move through them.  Kept
 * at file scope so lbm_init can read it without an extra parameter. */
static int g_pattern_idx = 0;

/* Paints whichever shape is currently selected. */
static void lbm_build_solid(LBM *g) { PATTERNS[g_pattern_idx].build(g); }

/* ── starting the fluid up ──
 *
 * lbm_init is a short recipe — set the physics knobs, reset the view
 * toggles, work out where the obstacle goes, paint it, then fill the
 * fluid with a calm, even starting state.  Each step is its own little
 * helper so lbm_init reads like the recipe.                             */

/* Set the thickness knob, its cached 1/tau, and the inlet speed. */
static void lbm_set_physics_knobs(LBM *g, float tau, float u_in) {
  g->tau = tau;
  g->omega = 1.0f / tau;
  g->u_in = u_in;
}

/* Reset the view toggles to defaults.  pattern_apply puts the user's
 * chosen view back afterward, so switching shapes with n/p doesn't yank
 * you out of, say, spin view. */
static void lbm_reset_ui_state(LBM *g) {
  g->vis_mode = VIS_VELOCITY;
  g->show_stream = false;
  g->paused = false;
  g->step_count = 0;
}

/* Work out where the obstacle sits and how big it is, from the CYL_*_FRAC
 * fractions times the current grid size, so it scales with the window. */
static void lbm_compute_obstacle_geometry(LBM *g) {
  g->cyl_x = (int)(g->nx * CYL_X_FRAC);
  g->cyl_y = (int)(g->ny * CYL_Y_FRAC);
  g->cyl_r = (int)(g->ny * CYL_R_FRAC);
  if (g->cyl_r < 2)
    g->cyl_r = 2;
}

/* Fill every fluid cell with a calm, even starting state moving at the
 * inlet speed.  We add a tiny random up/down wobble: a perfectly
 * symmetric channel would stay perfectly symmetric forever and never
 * start shedding swirls, so this little nudge breaks the tie. [7]
 *
 * The random seed is fixed, so the same shape and settings always replay
 * the exact same flow. */
static void lbm_seed_fluid_to_equilibrium(LBM *g) {
  int nx = g->nx, ny = g->ny;
  srand(12345);
  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      int idx = y * nx + x;
      float perturb =
          g->solid[idx] ? 0.0f : (float)(rand() % 200 - 100) * 0.00005f;
      for (int i = 0; i < 9; i++)
        g->f[idx * 9 + i] = feq(i, 1.0f, g->u_in, perturb);
      g->rho[idx] = 1.0f;
      g->ux[idx] = g->solid[idx] ? 0.0f : g->u_in;
      g->uy[idx] = g->solid[idx] ? 0.0f : perturb;
    }
  }
}

static void lbm_init(LBM *g, int nx, int ny, float tau, float u_in) {
  g->nx = nx;
  g->ny = ny;
  lbm_set_physics_knobs(g, tau, u_in); /* (1) physics knobs   */
  lbm_reset_ui_state(g);               /* (2) view defaults   */
  lbm_compute_obstacle_geometry(g);    /* (3) place obstacle  */
  lbm_build_solid(g);                  /* (4) paint the shape */
  lbm_seed_fluid_to_equilibrium(g);    /* (5) start the fluid */
}

/*
 * Switch to shape `idx` and restart the fluid around it.  We keep your
 * view choice and your physics tuning across the switch, so n/p doesn't
 * reset your spin view or your thickness setting.
 *
 * A full restart is the simplest honest thing to do: the new shape turns
 * some old fluid cells into solid and vice versa, so the old flow state
 * no longer makes sense and starting fresh is cleanest.
 */
static void pattern_apply(LBM *g, int idx) {
  if (idx < 0 || idx >= N_PATTERNS)
    return;
  g_pattern_idx = idx;

  int vm = g->vis_mode;
  bool ss = g->show_stream;
  bool pp = g->paused;
  float tau = g->tau;
  float u_in = g->u_in;

  lbm_init(g, g->nx, g->ny, tau, u_in);

  g->vis_mode = vm;
  g->show_stream = ss;
  g->paused = pp;
}

/* ── §6 collide ── */

/*
 * The "collide" step.  At every fluid cell, nudge each of the 9 direction
 * amounts a little toward its settled value (the feq target computed in
 * §4).  How big that nudge is is set by omega = 1/tau: near 1 the cell
 * snaps almost all the way to settled each step (thin, lively fluid);
 * near 0 it barely moves (thick, sluggish fluid).  That nudge rate IS the
 * fluid's thickness — it's the whole reason the fluid behaves like a
 * fluid.
 *
 * Solid cells are skipped — they hold no fluid.  Bouncing off walls is
 * handled later, in the move step.
 */
static void collide(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float omega = g->omega;

  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      int idx = y * nx + x;
      if (g->solid[idx])
        continue;

      float r = g->rho[idx];
      float u = g->ux[idx];
      float v = g->uy[idx];

      float *fi = g->f + idx * 9;
      for (int i = 0; i < 9; i++)
        fi[i] += omega * (feq(i, r, u, v) - fi[i]);
    }
  }
}

/* ── §7 stream ── */

/* ── moving the fluid along ──
 *
 * The "stream" step hands each direction amount off to the neighbour it
 * points at.  It's five small parts, each its own helper so stream()
 * below reads like a list:
 *
 *   (1) clear the scratch buffer
 *   (2) for each cell and direction, either hand the amount to the
 *       neighbour, or — if a wall is there — bounce it straight back
 *   (3) swap the buffers (no copying)
 *   (4) force the left edge to keep feeding fluid in at the inlet speed
 *   (5) let the right edge flow out freely
 *                                                                       */

/* Clear the scratch buffer first, because the move step adds into it and
 * one wall cell can collect bounced-back amounts from several directions. */
static void stream_clear_scratch(LBM *g) {
  memset(g->ftmp, 0, g->nx * g->ny * 9 * sizeof(float));
}

/* The actual move.  For each fluid cell and each direction, look at the
 * neighbour that direction points to:
 *   neighbour is open  → hand the amount over to it
 *   neighbour is solid → bounce it straight back the way it came
 * (the bounce is what makes fluid stick to walls instead of sliding past
 * — the incoming and reflected amounts cancel to zero flow at the wall [2]).
 *
 * Top and bottom edges count as walls.  Left and right edges get clamped
 * here but it doesn't matter — steps (4) and (5) overwrite those columns
 * outright anyway. */
static void stream_propagate_or_bounce(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float *f = g->f;
  float *fnew = g->ftmp;

  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      if (g->solid[y * nx + x])
        continue;

      float *src = f + (y * nx + x) * 9;

      for (int i = 0; i < 9; i++) {
        int nx2 = x + EX[i];
        int ny2 = y + EY[i];

        bool bounce = (ny2 < 0 || ny2 >= ny);
        if (!bounce && nx2 >= 0 && nx2 < nx)
          bounce = (g->solid[ny2 * nx + nx2] != 0);

        if (bounce) {
          fnew[(y * nx + x) * 9 + OPP[i]] += src[i];
        } else {
          int tx = nx2 < 0 ? 0 : (nx2 >= nx ? nx - 1 : nx2);
          fnew[(ny2 * nx + tx) * 9 + i] += src[i];
        }
      }
    }
  }
}

/* Swap the buffers: the scratch now holds the moved-along state, so make
 * it the live one.  Just a pointer swap, no copying. */
static void stream_swap_buffers(LBM *g) {
  float *tmp = g->f;
  g->f = g->ftmp;
  g->ftmp = tmp;
}

/* Left edge: keep pumping fluid in.  Overwrite the leftmost column with a
 * steady flow at the inlet speed, whatever the interior did to it — as if
 * an endless supply of moving fluid sits just off-screen to the left. */
static void stream_apply_inlet_bc(LBM *g) {
  int nx = g->nx, ny = g->ny;
  for (int y = 1; y < ny - 1; y++) {
    if (g->solid[y * nx + 0])
      continue;
    float *fi = g->f + (y * nx + 0) * 9;
    for (int i = 0; i < 9; i++)
      fi[i] = feq(i, 1.0f, g->u_in, 0.0f);
  }
}

/* Right edge: let fluid leave freely.  Just copy the second-to-last
 * column into the last one, so the flow drifts out without anything
 * special happening — the simplest open exit, and it keeps pressure
 * waves from bouncing back inward. */
static void stream_apply_outlet_bc(LBM *g) {
  int nx = g->nx, ny = g->ny;
  for (int y = 0; y < ny; y++) {
    float *dst = g->f + (y * nx + (nx - 1)) * 9;
    float *src2 = g->f + (y * nx + (nx - 2)) * 9;
    memcpy(dst, src2, 9 * sizeof(float));
  }
}

static void stream(LBM *g) {
  stream_clear_scratch(g);       /* (1) clear the scratch buffer  */
  stream_propagate_or_bounce(g); /* (2) move along + bounce walls */
  stream_swap_buffers(g);        /* (3) swap buffers              */
  stream_apply_inlet_bc(g);      /* (4) feed fluid in at the left */
  stream_apply_outlet_bc(g);     /* (5) let it out at the right   */
}

/* ── §8 compute_macroscopic ── */

/* ── reading speed and density back out ──
 *
 * The 9 direction amounts are the real state; the speed and density we
 * draw are just summaries of them.  Two little helpers pull those
 * summaries out, and a third fills in placeholder values for solid cells.
 * Nothing is created or lost in the process. [1, 2]
 *                                                                       */

/* Density = add up all 9 direction amounts at a cell — how much fluid is
 * there. */
static inline float density_zeroth_moment(const float *fi) {
  float r = 0.0f;
  for (int i = 0; i < 9; i++)
    r += fi[i];
  return r;
}

/* Velocity = the net flow from the 9 directions, divided by the density.
 * Each direction pushes its amount its own way; add those up and you get
 * which way and how fast the fluid is moving. */
static inline void velocity_first_moment(const float *fi, float rho, float *ux,
                                         float *uy) {
  float ru = 0.0f, rv = 0.0f;
  for (int i = 0; i < 9; i++) {
    ru += fi[i] * (float)EX[i];
    rv += fi[i] * (float)EY[i];
  }
  *ux = ru / rho;
  *uy = rv / rho;
}

/* Solid cells hold no fluid, so give them tidy stand-in values (density 1,
 * no motion) so the views and the inlet don't read leftover garbage. */
static inline void set_node_solid_defaults(LBM *g, int idx) {
  g->rho[idx] = 1.0f;
  g->ux[idx] = 0.0f;
  g->uy[idx] = 0.0f;
}

/*
 * Read density and velocity back out at every cell, and while we're
 * sweeping the whole grid anyway, note the fastest speed and the density
 * range — the views need those to scale their colours.
 */
static void compute_macroscopic(LBM *g) {
  float vmax = 0.0f, rmin = 1e9f, rmax = -1e9f;

  for (int y = 0; y < g->ny; y++) {
    for (int x = 0; x < g->nx; x++) {
      int idx = y * g->nx + x;

      if (g->solid[idx]) {
        set_node_solid_defaults(g, idx);
        continue;
      }

      const float *fi = g->f + idx * 9;
      float r = density_zeroth_moment(fi);
      if (r < 1e-6f)
        r = 1e-6f; /* ÷0 guard */
      g->rho[idx] = r;
      velocity_first_moment(fi, r, &g->ux[idx], &g->uy[idx]);

      float vm = sqrtf(g->ux[idx] * g->ux[idx] + g->uy[idx] * g->uy[idx]);
      if (vm > vmax)
        vmax = vm;
      if (r < rmin)
        rmin = r;
      if (r > rmax)
        rmax = r;
    }
  }

  g->vel_max = vmax > 1e-6f ? vmax : 1e-6f;
  g->rho_min = rmin;
  g->rho_max = rmax;
}

/* ── §9 vorticity ── */

/*
 * Spin (vorticity): how fast the fluid is rotating at each point, and
 * which way.  We measure it by comparing a cell's neighbours — if the
 * up/down flow changes as you go left-to-right, and the left/right flow
 * changes as you go top-to-bottom, the difference between those two is
 * the local spin.  Positive means turning one way, negative the other;
 * the alternating swirls in the wake show up as alternating + and − blobs.
 *
 * Edge cells are skipped, since the measurement needs a neighbour on
 * both sides.
 */
static void compute_vorticity(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float wmax = 0.0f;

  for (int y = 1; y < ny - 1; y++) {
    for (int x = 1; x < nx - 1; x++) {
      int idx = y * nx + x;
      if (g->solid[idx]) {
        g->vort[idx] = 0.0f;
        continue;
      }
      float duy_dx = (g->uy[y * nx + (x + 1)] - g->uy[y * nx + (x - 1)]) * 0.5f;
      float dux_dy = (g->ux[(y + 1) * nx + x] - g->ux[(y - 1) * nx + x]) * 0.5f;
      float w = duy_dx - dux_dy;
      g->vort[idx] = w;
      float aw = fabsf(w);
      if (aw > wmax)
        wmax = aw;
    }
  }
  g->vort_max = wmax > 1e-8f ? wmax : 1e-8f;
}

/* ── §10 render ── */

/*
 * Speed view — colours each cell by how fast the fluid is moving, cool
 * for slow, hot for fast.  Shows where the flow races (around the sides
 * of the obstacle) and where it nearly stops (right in front of it).
 * Speed is scaled against the frame's fastest and sorted into 8 colour
 * steps.  We turn the colour on once per cell, not per character — every
 * colour change is a little extra work for the terminal.
 */
static void render_velocity(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float inv = 7.0f / g->vel_max;

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float vm = sqrtf(g->ux[idx] * g->ux[idx] + g->uy[idx] * g->uy[idx]);
      int lv = (int)(vm * inv);
      if (lv < 0)
        lv = 0;
      if (lv > 7)
        lv = 7;

      int cp = CP_VEL0 + lv;
      attr_t at = (lv >= 5) ? A_BOLD : A_NORMAL;
      attron(COLOR_PAIR(cp) | at);
      mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/*
 * Spin view — the best look at the swirls.  Fluid turning one way gets
 * one colour, fluid turning the other way gets another, and you can watch
 * them peel off the obstacle in an alternating row downstream.  Strength
 * of the spin is shown by brightness (dim to bold), so the two directions
 * only need two colours between them.  Colour is set once per cell, same
 * as the speed view.
 */
static void render_vorticity(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float inv = 2.9f / g->vort_max; /* spread the strength over the levels */

  static const char VOR_CHARS[6] = {'#', 'x', '.', '.', 'x', '#'};
  /* levels 0..2 turn one way, 3..5 turn the other; brightness (below)
   * carries the strength so all 6 share just two colours. */
  static const attr_t VOR_ATTR[6] = {
      A_BOLD, A_NORMAL, A_DIM, /* one way: strong → weak */
      A_DIM,  A_NORMAL, A_BOLD /* other way: weak → strong */
  };

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float w = g->vort[idx] * inv; /* roughly -3..+3 */
      int lv;
      if (w < 0.0f) {
        lv = (int)(-w);
        if (lv > 2)
          lv = 2;
        lv = 2 - lv; /* one direction, strongest first */
      } else {
        lv = (int)(w);
        if (lv > 2)
          lv = 2;
        lv = 3 + lv; /* other direction, weakest first */
      }

      short cp = (lv < 3) ? CP_VNEG : CP_VPOS;
      attr_t at = VOR_ATTR[lv];
      attron(COLOR_PAIR(cp) | at);
      mvaddch(y + 1, x, (chtype)VOR_CHARS[lv]);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/*
 * Density view — basically a pressure map (here pressure is just density
 * divided by 3).  Crowded, high-pressure spots glow warm — like the
 * region right in front of the obstacle — while the low-pressure cores of
 * the swirls show up cool.  The differences are tiny (around 0.001), so
 * each frame stretches the colours across whatever range it actually has.
 */
static void render_density(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float rng =
      (g->rho_max - g->rho_min) > 1e-6f ? (g->rho_max - g->rho_min) : 1e-6f;

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float norm = (g->rho[idx] - g->rho_min) / rng;
      int lv = (int)(norm * 7.0f);
      if (lv < 0)
        lv = 0;
      if (lv > 7)
        lv = 7;

      attron(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
      mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
      attroff(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
    }
  }
}

/*
 * Flow arrows — a sparse layer of little direction arrows drawn over
 * whichever view is active, one every few cells, showing which way the
 * fluid is heading.
 */
#define STREAM_DX 4
#define STREAM_DY 2

/* Don't draw an arrow where the fluid is barely moving.  In the dead-calm
 * spots (right in front of the obstacle, or deep in a trapped swirl) the
 * flow is near zero and its "direction" is just rounding noise, which
 * would scatter random arrows everywhere.  This cutoff hides those. */
#define STREAM_THR 0.005f /* skip arrows below this speed */

static void render_streamlines(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;

  for (int y = STREAM_DY; y < ny - STREAM_DY && y < rows - 1; y += STREAM_DY) {
    for (int x = STREAM_DX; x < nx - STREAM_DX && x < cols; x += STREAM_DX) {
      int idx = y * nx + x;
      if (g->solid[idx])
        continue;

      float u = g->ux[idx];
      float v = g->uy[idx];
      float spd = sqrtf(u * u + v * v);
      if (spd < STREAM_THR)
        continue;

      /* turn the flow direction into one of 8 compass headings */
      float ang = atan2f(v, u); /* remember +y points down on screen */
      int dir =
          (int)((ang + (float)M_PI) / (2.0f * (float)M_PI) * 8.0f + 0.5f) & 7;

      attron(COLOR_PAIR(CP_ARROW) | A_BOLD);
      mvaddch(y + 1, x, (chtype)ARROW8[dir]);
      attroff(COLOR_PAIR(CP_ARROW) | A_BOLD);
    }
  }
}

/* ── §11 render_overlay ── */

static const char *VIS_NAMES[VIS_COUNT] = {"VELOCITY ", "VORTICITY",
                                           "DENSITY  "};

/*
 * The two-line HUD: a status line across the top (current view, shape,
 * theme, the physics numbers, fps, paused or not) and the key legend
 * along the bottom.  Both are drawn bright and bold so they stay readable
 * over the busy animation.  The paused note sits inline in the status
 * rather than as a big centred banner, so it never covers the flow.
 */
static void render_overlay(const LBM *g, int cols, int rows, double fps) {
  /* Reynolds number — a single figure for how lively the flow is, from
   * the inlet speed, obstacle width, and thickness. */
  float nu = (g->tau - 0.5f) / 3.0f;
  float re = (nu > 1e-6f) ? (g->u_in * (float)(2 * g->cyl_r) / nu) : 0.0f;

  /* ── top: status line ── */
  char top[256];
  snprintf(top, sizeof top,
           " LBM D2Q9  [%s]%s  pat:%s  theme:%s  tau=%.3f nu=%.4f Re~%.0f"
           "  rho[%.3f,%.3f] |u|max=%.4f  step:%-6d  %.0ffps  %s",
           VIS_NAMES[g->vis_mode], g->show_stream ? "+stream" : "       ",
           PATTERNS[g_pattern_idx].name, THEMES[g_theme_idx].name, g->tau, nu,
           re, g->rho_min, g->rho_max, g->vel_max, g->step_count, fps,
           g->paused ? "PAUSED " : "running");

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, 0, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── bottom: key legend ── */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  v:vel  o:vort  d:density  s:stream"
           "  n/p:pattern  +/-:tau  w/W:speed  t/T:theme  r:reset ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §12 scene ── */

/*
 * Scene — a thin wrapper that just holds the LBM for now.  The LBM
 * already carries the whole world, so this looks redundant, and it kind
 * of is.  It's here as room to grow: if the demo ever sprouts something
 * that isn't part of the fluid itself — a tracer overlay, a recording
 * buffer, a second fluid to compare side by side — it can be added here
 * without touching the scene_init / tick / draw / resize functions every
 * caller already uses.
 */
typedef struct {
  LBM lbm; /* the whole fluid world — see §5 for the field breakdown */
} Scene;

static void scene_init(Scene *sc, int cols, int rows) {
  LBM *g = &sc->lbm;
  int nx = cols;
  int ny = rows - 2; /* minus top HUD row and bottom key row */
  if (ny < 4)
    ny = 4;

  if (lbm_alloc(g, nx, ny) != 0)
    return;
  lbm_init(g, nx, ny, TAU_DEFAULT, U_IN_DEFAULT);
}

static void scene_free(Scene *sc) { lbm_free(&sc->lbm); }

static void scene_resize(Scene *sc, int cols, int rows) {
  LBM *g = &sc->lbm;
  int vm = g->vis_mode;
  bool ss = g->show_stream;
  float tau = g->tau, u_in = g->u_in;

  lbm_free(g);
  memset(g, 0, sizeof *g);
  g->vis_mode = vm;
  g->show_stream = ss;

  int nx = cols, ny = rows - 2;
  if (ny < 4)
    ny = 4;
  if (lbm_alloc(g, nx, ny) != 0)
    return;
  lbm_init(g, nx, ny, tau, u_in);
}

/*
 * One step of the simulation: read the current speed/density, collide,
 * move the fluid along, then read speed/density again so the renderer has
 * fresh numbers.  Spin is computed too, but only when the spin view is on.
 */
static void scene_tick(Scene *sc) {
  LBM *g = &sc->lbm;
  if (g->paused)
    return;

  compute_macroscopic(g);
  collide(g);
  stream(g);
  compute_macroscopic(g);
  if (g->vis_mode == VIS_VORTICITY)
    compute_vorticity(g);
  g->step_count++;
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps) {
  erase();
  const LBM *g = &sc->lbm;

  switch (g->vis_mode) {
  case VIS_VELOCITY:
    render_velocity(g, cols, rows - 2);
    break;
  case VIS_VORTICITY:
    render_vorticity(g, cols, rows - 2);
    break;
  case VIS_DENSITY:
    render_density(g, cols, rows - 2);
    break;
  }

  if (g->show_stream)
    render_streamlines(g, cols, rows - 2);
  render_overlay(g, cols, rows, fps);
}

/* ── §13 screen ── */

/*
 * Screen — just the current terminal width and height in cells.  Kept
 * separate from the grid size (LBM.nx/ny) on purpose: when the window
 * resizes, this updates straight from ncurses, and then the grid tries to
 * re-fit to match — but if the window got too small the grid keeps its
 * old size and the renderer simply draws as much as fits.  The top and
 * bottom rows are the HUD, so the fluid is drawn in the rows between.
 */
typedef struct {
  int cols, rows; /* terminal width and height in cells */
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

/* ── §14 app ── */

/*
 * App — everything the program needs in one box: the fluid world, the
 * terminal size, and two flags that say "time to quit" and "the window
 * resized".  It's born when main starts and dies when main returns, so
 * one App pointer can carry the whole program through the keypress and
 * resize helpers without scattering loose globals around.
 *
 * The two flags need a quick word, because they're touched from two
 * places at once.  When you press Ctrl-C or resize the window, the
 * operating system interrupts the program and runs a tiny handler.  That
 * handler can't be handed our App, so we keep one App at file scope and
 * let the handler flip a flag on it.  The main loop checks the flags and
 * does the real work — we don't do it inside the handler itself, because
 * things like ncurses redraws and malloc aren't safe to call mid-
 * interrupt.
 *
 *   running     — set to 0 to break out of the main loop and quit
 *                 cleanly.
 *   need_resize — set when the window resizes; the main loop notices and
 *                 re-fits everything to the new size.
 *
 * Both are marked `volatile sig_atomic_t`: volatile so the compiler
 * never assumes the value can't change behind its back (it can — a
 * handler may change it any moment), and sig_atomic_t because that's the
 * one integer type C promises can be read and written in one piece even
 * if an interrupt lands halfway.
 */
typedef struct {
  Scene scene;                       /* the whole fluid world (see §12)  */
  Screen screen;                     /* current terminal size (see §13)  */
  volatile sig_atomic_t running;     /* set to 0 to quit                 */
  volatile sig_atomic_t need_resize; /* set when the window resized      */
} App;

static App g_app;
static void on_exit(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static bool handle_key(App *app, int ch) {
  LBM *g = &app->scene.lbm;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    g->paused = !g->paused;
    break;
  case 'v':
  case 'V':
    g->vis_mode = VIS_VELOCITY;
    break;
  case 'o':
  case 'O':
    g->vis_mode = VIS_VORTICITY;
    break;
  case 'd':
  case 'D':
    g->vis_mode = VIS_DENSITY;
    break;
  case 's':
  case 'S':
    g->show_stream = !g->show_stream;
    break;
  case '+':
  case '=':
    g->tau = g->tau + TAU_STEP;
    if (g->tau > TAU_MAX)
      g->tau = TAU_MAX;
    g->omega = 1.0f / g->tau;
    break;
  case '-':
    g->tau = g->tau - TAU_STEP;
    if (g->tau < TAU_MIN)
      g->tau = TAU_MIN;
    g->omega = 1.0f / g->tau;
    break;
  case 'w':
    g->u_in *= 1.2f;
    if (g->u_in > 0.30f)
      g->u_in = 0.30f;
    break;
  case 'W':
    g->u_in /= 1.2f;
    if (g->u_in < 0.01f)
      g->u_in = 0.01f;
    break;
  case 'r':
  case 'R':
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    break;
  case 'n':
    pattern_apply(g, (g_pattern_idx + 1) % N_PATTERNS);
    break;
  case 'p':
    pattern_apply(g, (g_pattern_idx + N_PATTERNS - 1) % N_PATTERNS);
    break;
  case 't':
    g_theme_idx = (g_theme_idx + 1) % N_THEMES;
    theme_apply(g_theme_idx);
    break;
  case 'T':
    g_theme_idx = (g_theme_idx + N_THEMES - 1) % N_THEMES;
    theme_apply(g_theme_idx);
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
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int fps_count = 0;
  double fps_disp = 0.0;

  while (app->running) {

    /* ── resize ──────────────────────────────────── */
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      frame_time = clock_ns();
    }

    /* ── dt ──────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 200 * NS_PER_MS)
      dt = 200 * NS_PER_MS;

    /* ── physics: multiple LBM steps per frame ───── */
    for (int s = 0; s < STEPS_PER_FRAME; s++)
      scene_tick(&app->scene);

    /* ── fps counter ─────────────────────────────── */
    fps_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_count / ((double)fps_accum / (double)NS_PER_SEC);
      fps_count = 0;
      fps_accum = 0;
    }

    /* ── sleep to cap render at TARGET_FPS ───────── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* ── render ──────────────────────────────────── */
    scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
    wnoutrefresh(stdscr);
    doupdate();

    /* ── input ───────────────────────────────────── */
    int ch;
    while ((ch = getch()) != ERR)
      if (!handle_key(app, ch)) {
        app->running = 0;
        break;
      }
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
