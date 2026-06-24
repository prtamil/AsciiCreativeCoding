/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lorenz.c — the Lorenz "butterfly" attractor, drawn live in the terminal.
 *
 * We follow a point as it loops forever through the three Lorenz equations,
 * tracing the famous butterfly shape.  Alongside it we run a few near-identical
 * "ghost" points started a hair apart; watching them fan out is chaos made
 * visible — tiny differences blow up fast.
 *
 * References the code can't tell you on its own:
 *   Lorenz, E. N. (1963), "Deterministic Nonperiodic Flow", J. Atmos. Sci.
 *     20(2), 130-141 — the original equations.
 *   Strogatz, Nonlinear Dynamics and Chaos (2014), Ch.9 — best gentle intro.
 *   Press et al., Numerical Recipes (2007), Ch.17 — the RK4 step we use.
 *   Wolf et al. (1985), Physica D 16(3), 285-317 — where the chaos rate
 *     λ ≈ 0.9 comes from (how fast the ghosts pull apart).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lorenz.c -o lorenz -lncurses -lm
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

/* ── §1  config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500,
  N_COLORS = 7,
  TRAIL_LEN = 2500, /* how many past points each trail remembers   */
  SUB_STEPS = 8,    /* physics steps taken per on-screen tick       */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* The three knobs in the Lorenz equations.  These exact values are the
 * famous chaotic ones that give the butterfly; nudge them and the shape
 * changes (too small and the point just spirals to a stop instead). */
#define L_SIGMA 10.0f
#define L_RHO 28.0f
#define L_BETA (8.0f / 3.0f)

/* How big a step the physics takes each time.  Small means accurate but
 * slow; this value is small enough that the trail stays glued to the real
 * butterfly.  Make it much bigger and the curve visibly drifts off. */
#define L_H 0.005f

/* How many "ghost" points we run beside the main one.  Each starts a tiny
 * bit apart; watching them spread out is the whole point of the demo. */
#define N_GHOSTS 5

/* How far each ghost starts from the main point (a nudge in x).  We pick a
 * wide spread, from a whisker (0.001) to a noticeable gap (0.1), so within
 * one sitting you can see the close one slowly pull away while the far one
 * scatters almost at once. */
static const float GHOST_EPS_TABLE[N_GHOSTS] = {0.001f, 0.005f, 0.01f, 0.05f,
                                                0.1f};

/* View */
#define CELL_W 8
#define CELL_H 16
/* Terminal cells are taller than they are wide, so we squash the vertical
 * axis by this ratio to keep the butterfly from looking stretched. */
#define ASPECT ((float)CELL_W / (float)CELL_H)

#define VIEW_PHI_DEFAULT 0.5f    /* starting left/right spin angle      */
#define VIEW_THETA_DEFAULT 0.55f /* starting up/down tilt angle         */
#define VIEW_PHI_SPEED 0.08f     /* how fast the hands-free spin turns  */

/* ── §2  clock ── */

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

/* ── §3  color / theme ── */

enum {
  CP_TRAIL_HEAD = 1, /* newest part of the trail — brightest colour  */
  CP_TRAIL_MID = 2,  /* middle-aged part of the trail                */
  CP_TRAIL_TAIL = 3, /* oldest, fading part of the trail             */
  CP_GHOST = 4,      /* the ghost trails and the gap-size readout    */
  CP_HUD = 5,        /* top status bar — bright yellow               */
  CP_HINT = 6,       /* bottom key hints — bright cyan               */
};

/*
 * Theme — the four colours one look uses for the trail.
 *
 * The trail fades with age: its newest stretch is painted in `head`
 * (and bold), the middle in `mid`, the old end in `tail`.  `ghost`
 * colours the faint shadow trails and the gap-size readout text.
 *
 * Switching themes is just swapping these four colours — no blending or
 * math.  Every theme keeps the same rule (newest = brightest) so the
 * trail still reads the same way, just in different hues.
 *
 * Two more colours stay fixed no matter the theme: yellow for the status
 * bar and cyan for the key hints (set up once in color_init).
 */
typedef struct {
  const char *name;
  short head;  /* newest trail colour (drawn bold)                  */
  short mid;   /* middle-aged trail colour                          */
  short tail;  /* oldest, fading trail colour                       */
  short ghost; /* ghost trails + gap-size readout                   */
} Theme;

/*
 * Ten ready-made looks, each just four colours picked to go together.
 * Names hint at the mood: MATRIX (green CRT), FIRE (flame), OCEANIC,
 * NEON, MONO (greyscale), ICE, NOVA, FOREST, DESERT, ECLIPSE.
 */
static const Theme THEMES[] = {
    /*  name         head  mid  tail  ghost */
    {"MATRIX", 46, 40, 22, 246},    {"FIRE", 196, 208, 226, 201},
    {"OCEANIC", 51, 45, 24, 153},   {"NEON", 199, 213, 57, 226},
    {"MONO", 255, 250, 244, 238},   {"ICE", 195, 159, 75, 153},
    {"NOVA", 231, 213, 57, 117},    {"FOREST", 154, 142, 22, 130},
    {"DESERT", 220, 215, 130, 137}, {"ECLIPSE", 196, 88, 53, 67},
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/* Which theme is showing right now; the t/T keys step through them.
 * Kept here at file scope so both startup and the key handler can read it. */
static int g_theme_idx = 0;

/*
 * Switches the trail colours over to theme `idx`.  Leaves the fixed
 * yellow/cyan UI colours alone — only the four trail/ghost colours change.
 */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    return;
  const Theme *t = &THEMES[idx];

  if (COLORS >= 256) {
    init_pair(CP_TRAIL_HEAD, t->head, -1);
    init_pair(CP_TRAIL_MID, t->mid, -1);
    init_pair(CP_TRAIL_TAIL, t->tail, -1);
    init_pair(CP_GHOST, t->ghost, -1);
  } else {
    /* Old 8-colour terminal: no room for themes, so fall back to one
     * fixed set of basic colours. */
    init_pair(CP_TRAIL_HEAD, COLOR_RED, -1);
    init_pair(CP_TRAIL_MID, COLOR_YELLOW, -1);
    init_pair(CP_TRAIL_TAIL, COLOR_GREEN, -1);
    init_pair(CP_GHOST, COLOR_MAGENTA, -1);
  }
}

/*
 * Sets up colours once at startup: the fixed yellow status bar and cyan
 * hints, then the starting theme.
 */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }

  theme_apply(g_theme_idx);
}

/* ── §4  coords — turning a 3-D point into a screen cell ── */

/* Turning a point in 3-D space into a spot on the flat screen takes four
 * little steps, done in order: slide the whole shape so it's centred,
 * spin it left/right, tilt it up/down, then place it on a terminal cell.
 * Each step is its own tiny helper so project() below reads like a recipe. */

/* Step 1: slide the shape so it sits centred on screen.  The butterfly
 * naturally floats above the origin, so we just pull it down; left/right
 * and depth stay put. */
static inline void world_center_at_attractor(float lx, float ly, float lz,
                                             float *px, float *py, float *pz) {
  *px = lx;
  *py = ly;
  *pz = lz - 25.0f;
}

/* Step 2: spin the shape left/right by angle phi, like turning a model on
 * a lazy Susan.  After the spin, ry measures how far "into the screen" a
 * point sits. */
static inline void rotate_around_z_axis(float px, float py, float phi,
                                        float *rx, float *ry) {
  *rx = px * cosf(phi) + py * sinf(phi);
  *ry = -px * sinf(phi) + py * cosf(phi);
}

/* Step 3: tilt the shape up/down by angle theta, like nodding it toward
 * or away from you.  sy comes out as the up/down screen position; sz is
 * how near or far the point ended up (negative = toward you / closer,
 * positive = away / farther).  We use sz later to brighten near points. */
static inline void tilt_around_x_axis(float ry, float pz, float theta,
                                      float *sy, float *sz) {
  *sy = ry * cosf(theta) + pz * sinf(theta);
  *sz = -ry * sinf(theta) + pz * cosf(theta);
}

/*
 * Takes one 3-D point and works out which terminal cell it lands on, plus
 * how near/far it is.  Runs the four steps above (centre, spin, tilt,
 * place), then checks the result is actually on screen.
 *
 * Pass NULL for out_depth if you don't care about near/far (the fixed-point
 * markers and the heatmap don't).  The trail and stars pass a real pointer
 * so closer points can be drawn brighter.
 *
 * Returns false when the point would land off-screen.
 */
static bool project(float lx, float ly, float lz, float phi, float theta,
                    float scale, int cx, int cy, int cols, int rows,
                    int *out_col, int *out_row, float *out_depth) {
  float px, py, pz;
  world_center_at_attractor(lx, ly, lz, &px, &py, &pz); /* centre */

  float rx, ry;
  rotate_around_z_axis(px, py, phi, &rx, &ry); /* spin */

  float sx = rx;
  float sy, sz;
  tilt_around_x_axis(ry, pz, theta, &sy, &sz); /* tilt */

  /* scale up and shift to the middle of the screen */
  int col = cx + (int)(sx * scale);
  int row = cy - (int)(sy * scale * ASPECT);

  *out_col = col;
  *out_row = row;
  if (out_depth)
    *out_depth = sz;
  return (col >= 0 && col < cols && row >= 1 && row < rows - 1); /* on screen? */
}

/* ── §5  entity — Lorenz ── */

/*
 * Trail — the recent path of one moving point, so we can draw it as a
 * fading streak instead of a lone dot.
 *
 * It's a ring buffer: a fixed block of slots that we write round and
 * round, overwriting the oldest point each time.  The simulation runs
 * forever, so we can't just keep adding to a list — this keeps memory
 * flat while always holding the most recent TRAIL_LEN points.  That's
 * about five seconds of motion, enough to see the whole butterfly at once.
 *
 * We track where the newest point is (head) and how many points we have
 * so far (count) — not a separate "oldest" marker, since that's just
 * implied by those two.  count also tells the renderer whether the trail
 * has filled up yet, so it never draws leftover garbage slots.
 *
 * The three coordinates live in separate arrays purely because it matches
 * how points go in and out (three plain numbers at a time); it makes no
 * speed difference here.
 */
typedef struct {
  float x[TRAIL_LEN]; /* the last TRAIL_LEN points, split by coordinate */
  float y[TRAIL_LEN];
  float z[TRAIL_LEN];
  int head;  /* slot holding the newest point                          */
  int count; /* how many slots are filled so far (caps at TRAIL_LEN)    */
} Trail;

static void trail_push(Trail *t, float x, float y, float z) {
  t->head = (t->head + 1) % TRAIL_LEN;
  t->x[t->head] = x;
  t->y[t->head] = y;
  t->z[t->head] = z;
  if (t->count < TRAIL_LEN)
    t->count++;
}

static void trail_clear(Trail *t) {
  t->head = 0;
  t->count = 0;
}

/* ── Lorenz ODE ────────────────────────────────────────────────────── */

static void lorenz_deriv(float x, float y, float z, float *dx, float *dy,
                         float *dz) {
  *dx = L_SIGMA * (y - x);
  *dy = x * (L_RHO - z) - y;
  *dz = x * y - L_BETA * z;
}

static void lorenz_rk4(float *x, float *y, float *z, float h) {
  float k1x, k1y, k1z;
  float k2x, k2y, k2z;
  float k3x, k3y, k3z;
  float k4x, k4y, k4z;

  lorenz_deriv(*x, *y, *z, &k1x, &k1y, &k1z);
  lorenz_deriv(*x + h * 0.5f * k1x, *y + h * 0.5f * k1y, *z + h * 0.5f * k1z,
               &k2x, &k2y, &k2z);
  lorenz_deriv(*x + h * 0.5f * k2x, *y + h * 0.5f * k2y, *z + h * 0.5f * k2z,
               &k3x, &k3y, &k3z);
  lorenz_deriv(*x + h * k3x, *y + h * k3y, *z + h * k3z, &k4x, &k4y, &k4z);

  *x += (h / 6.0f) * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
  *y += (h / 6.0f) * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
  *z += (h / 6.0f) * (k1z + 2.0f * k2z + 2.0f * k3z + k4z);
}

/*
 * Lorenz — the whole visible world in one box.
 *
 * It holds six moving points (one main point plus five ghosts), the
 * trail behind each, where the camera is looking, and a handful of
 * on/off switches for the different views.  We keep it all in one
 * struct so a single `Lorenz *l` can be handed to the update, draw,
 * and key-handling code without passing fifteen separate arguments.
 *
 * It's one struct, not two (a "simulation" half and a "drawing" half),
 * because the drawing code reads the point positions every frame while
 * the simulation never looks at the view switches — splitting them
 * would just mean passing two things everywhere for no gain.  The
 * fields below are still grouped by job: the moving points first, then
 * the camera, then the view switches.
 */
typedef struct {

  /* ── the main point ── */
  float mx, my, mz;
  /* Where the main point is right now in 3-D space.  Nudged forward
   * a little each step.  It starts at (1, 1, 1), which sits right on
   * the butterfly, so the trail joins the shape almost at once. */

  Trail mt;
  /* The path the main point has traced — its fading streak.  A new
   * spot is added every step; see the Trail doc above. */

  /* ── the ghost points ── */
  float gx[N_GHOSTS], gy[N_GHOSTS], gz[N_GHOSTS];
  /* Where each ghost point is right now.  Every ghost starts a tiny
   * nudge away from the main point (the nudges come from
   * GHOST_EPS_TABLE) and then obeys the exact same rules.  Because
   * the only difference is that first tiny nudge, watching them drift
   * apart is chaos itself — small starts, wildly different endings. */

  Trail gt[N_GHOSTS];
  /* The fading streak behind each ghost.  Drawn faint, with a comma,
   * and the widest-nudge ghost is painted first so the closer ones
   * land on top. */

  /* ── where the camera looks ── */
  float phi;
  /* Left/right spin angle of the view.  Creeps along on its own when
   * auto-spin is on, and the left/right arrow keys nudge it by hand
   * (which also turns auto-spin off). */

  float theta;
  /* Up/down tilt angle of the view.  The up/down arrows change it,
   * kept inside a sensible range so the view never flips over or
   * goes flat. */

  bool auto_rotate;
  /* When on, the view slowly spins by itself for a hands-free show.
   * Toggled with 'a', and also switched off the moment you steer with
   * the arrow keys, so you're never fighting the drift. */

  /* ── view switches (these only change what you see, never the math) ── */
  bool show_ghost;
  /* g — show the five ghost trails under the main one.  Off, and you
   * just see the main trail and the gap-size readout. */

  bool show_lobe;
  /* l — colour the main trail by which wing of the butterfly it's on:
   * one colour for the right wing, another for the left.  The colour
   * flips every time the point jumps wings, which is the chaos you
   * can't predict, shown as a sudden change of hue. */

  bool show_density;
  /* h — swap the trail for a heatmap: instead of the path, show where
   * the point spends most of its time, brightest where it lingers.
   * It's tied to one camera angle, so it clears itself whenever you
   * turn the view. */

  bool paused;
  /* space — freeze the motion.  The math stops, but drawing keeps
   * going so you can study the frozen shape.  The auto-spin pauses
   * too, since the spin happens inside the same update step. */
} Lorenz;

/* Setup is four small steps: place the main point, place the ghosts,
 * reset the camera, reset the view switches. */

/* Drop the main point at (1,1,1), which sits right on the butterfly, so
 * the trail joins the shape straight away instead of flying in from far
 * off. */
static void seed_main_trajectory(Lorenz *l) {
  l->mx = 1.0f;
  l->my = 1.0f;
  l->mz = 1.0f;
  trail_clear(&l->mt);
  trail_push(&l->mt, l->mx, l->my, l->mz);
}

/* Place the ghosts, each one a tiny nudge in x away from the main point
 * (the nudges come from GHOST_EPS_TABLE).  Same start in y and z; only
 * x differs.  From there they pull apart fast — that's the whole show. */
static void seed_ghost_shower(Lorenz *l) {
  for (int g = 0; g < N_GHOSTS; g++) {
    l->gx[g] = 1.0f + GHOST_EPS_TABLE[g];
    l->gy[g] = 1.0f;
    l->gz[g] = 1.0f;
    trail_clear(&l->gt[g]);
    trail_push(&l->gt[g], l->gx[g], l->gy[g], l->gz[g]);
  }
}

/* Put the camera back to its starting angles, with auto-spin on. */
static void reset_view_defaults(Lorenz *l) {
  l->phi = VIEW_PHI_DEFAULT;
  l->theta = VIEW_THETA_DEFAULT;
  l->auto_rotate = true;
}

/* Reset the view switches: ghosts on (that's the demo's signature
 * look), the wing-colour and heatmap views off (you opt into those),
 * and not paused. */
static void reset_render_toggles(Lorenz *l) {
  l->show_ghost = true;
  l->show_lobe = false;
  l->show_density = false;
  l->paused = false;
}

static void lorenz_init(Lorenz *l) {
  seed_main_trajectory(l); /* place the main point */
  seed_ghost_shower(l);    /* place the ghosts     */
  reset_view_defaults(l);  /* reset the camera     */
  reset_render_toggles(l); /* reset view switches  */
}

static void lorenz_tick(Lorenz *l, float dt) {
  if (l->paused)
    return;

  if (l->auto_rotate)
    l->phi += VIEW_PHI_SPEED * dt;

  for (int s = 0; s < SUB_STEPS; s++) {
    lorenz_rk4(&l->mx, &l->my, &l->mz, L_H);
    trail_push(&l->mt, l->mx, l->my, l->mz);
    for (int g = 0; g < N_GHOSTS; g++) {
      lorenz_rk4(&l->gx[g], &l->gy[g], &l->gz[g], L_H);
      trail_push(&l->gt[g], l->gx[g], l->gy[g], l->gz[g]);
    }
  }
}

/* ── starfield backdrop ── */

/* About 60 fixed stars scattered in a box around the butterfly.  They're
 * drawn first, so the butterfly sits in front of them.  They spin with
 * the same view, so as you turn, the stars slide past behind the shape —
 * a cheap but convincing sense of depth. */

#define N_STARS 60

static float g_star_x[N_STARS];
static float g_star_y[N_STARS];
static float g_star_z[N_STARS];
static bool g_stars_initialised = false;

/* Its own little random-number generator, kept separate from the program's
 * main one so the stars land in the same spots every run. */
static unsigned int stars_lcg_next(unsigned int *s) {
  *s = (*s) * 1103515245u + 12345u;
  return (*s) >> 16;
}

static void stars_init(void) {
  if (g_stars_initialised)
    return;
  unsigned int s = 987654321u;
  for (int i = 0; i < N_STARS; i++) {
    g_star_x[i] =
        ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f - 0.5f) * 80.0f;
    g_star_y[i] =
        ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f - 0.5f) * 80.0f;
    g_star_z[i] =
        ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f) * 70.0f - 10.0f;
  }
  g_stars_initialised = true;
}

static void stars_draw(float phi, float theta, float scale, int cx, int cy,
                       int cols, int rows, WINDOW *w) {
  if (!g_stars_initialised)
    stars_init();
  for (int i = 0; i < N_STARS; i++) {
    int col, row;
    float depth;
    if (!project(g_star_x[i], g_star_y[i], g_star_z[i], phi, theta, scale, cx,
                 cy, cols, rows, &col, &row, &depth))
      continue;
    /* Near stars a touch brighter than far ones, so even the backdrop
     * hints at depth. */
    attr_t at = (depth < -10.0f) ? A_NORMAL : A_DIM;
    wattron(w, COLOR_PAIR(CP_TRAIL_TAIL) | at);
    mvwaddch(w, row, col, '.');
    wattroff(w, COLOR_PAIR(CP_TRAIL_TAIL) | at);
  }
}

/* ── the three still points ── */

/* The butterfly has three special spots where a point placed exactly
 * there would never move: the centre (0,0,0) and one in the heart of
 * each wing.  Nothing actually rests there — the two wings circle around
 * those spots — so marking them shows the frame the whole shape is built
 * on.  We draw a '+' at each. */

#define FP_C_XY 8.485281f /* the wing-centre offset (square root of 72) */
#define FP_C_Z 27.0f

static const float FIXED_POINTS[3][3] = {
    {0.0f, 0.0f, 0.0f},           /* centre     */
    {FP_C_XY, FP_C_XY, FP_C_Z},   /* right wing */
    {-FP_C_XY, -FP_C_XY, FP_C_Z}, /* left wing  */
};

static void equilibria_draw(float phi, float theta, float scale, int cx, int cy,
                            int cols, int rows, WINDOW *w) {
  for (int i = 0; i < 3; i++) {
    int col, row;
    if (!project(FIXED_POINTS[i][0], FIXED_POINTS[i][1], FIXED_POINTS[i][2],
                 phi, theta, scale, cx, cy, cols, rows, &col, &row, NULL))
      continue;
    wattron(w, COLOR_PAIR(CP_TRAIL_MID) | A_BOLD);
    mvwaddch(w, row, col, '+');
    wattroff(w, COLOR_PAIR(CP_TRAIL_MID) | A_BOLD);
  }
}

/* ── heatmap of where the point lingers ── */

/* A tally per screen cell: each frame we mark every cell the trail
 * passes over and bump its count.  Cells that get visited often are
 * drawn brighter ('.' for a few visits, '*' for more, '#' for a lot) —
 * so you can see where the point really spends its time.  The tally is
 * tied to one camera angle, so we wipe it whenever the view turns. */

#define DENSITY_MAX_COLS 600
#define DENSITY_MAX_ROWS 200

static uint8_t g_density[DENSITY_MAX_ROWS][DENSITY_MAX_COLS];
static float g_density_view_phi = -999.0f;
static float g_density_view_theta = -999.0f;

static void density_reset(void) {
  memset(g_density, 0, sizeof g_density);
  g_density_view_phi = -999.0f;
  g_density_view_theta = -999.0f;
}

/* The tally only makes sense for the angle it was built at.  If the
 * camera has turned more than a hair since last time, wipe it and start
 * fresh at the new angle. */
static void density_invalidate_on_view_change(float phi, float theta) {
  if (fabsf(phi - g_density_view_phi) > 0.05f ||
      fabsf(theta - g_density_view_theta) > 0.05f) {
    density_reset();
    g_density_view_phi = phi;
    g_density_view_theta = theta;
  }
}

/* Bump the tally for one cell.  Ignores cells off the edge, and stops at
 * 255 so a long session never rolls the count back around to zero. */
static void density_bin_sample(int col, int row) {
  if (col < 0 || col >= DENSITY_MAX_COLS)
    return;
  if (row < 0 || row >= DENSITY_MAX_ROWS)
    return;
  if (g_density[row][col] < 255)
    g_density[row][col]++;
}

/* Wipe the tally if the view turned, then run down the trail marking
 * every cell it crosses. */
static void density_accumulate(const Trail *t, float phi, float theta,
                               float scale, int cx, int cy, int cols,
                               int rows) {
  density_invalidate_on_view_change(phi, theta);

  for (int k = 0; k < t->count; k++) {
    int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
    int col, row;
    if (!project(t->x[idx], t->y[idx], t->z[idx], phi, theta, scale, cx, cy,
                 cols, rows, &col, &row, NULL))
      continue;
    density_bin_sample(col, row);
  }
}

static void density_draw(WINDOW *w, int cols, int rows) {
  int rmax = rows < DENSITY_MAX_ROWS ? rows - 1 : DENSITY_MAX_ROWS;
  int cmax = cols < DENSITY_MAX_COLS ? cols : DENSITY_MAX_COLS;
  for (int r = 1; r < rmax; r++) {
    for (int c = 0; c < cmax; c++) {
      uint8_t n = g_density[r][c];
      if (n == 0)
        continue;
      short cp;
      attr_t at;
      char ch;
      if (n >= 16) {
        cp = CP_TRAIL_HEAD;
        at = A_BOLD;
        ch = '#';
      } else if (n >= 4) {
        cp = CP_TRAIL_MID;
        at = A_NORMAL;
        ch = '*';
      } else {
        cp = CP_TRAIL_TAIL;
        at = A_DIM;
        ch = '.';
      }
      wattron(w, COLOR_PAIR(cp) | at);
      mvwaddch(w, r, c, ch);
      wattroff(w, COLOR_PAIR(cp) | at);
    }
  }
}

/* Drawing one dot of a trail comes down to four separate choices: which
 * colour, how bright, which character, and whether to add a little glow.
 * Each gets its own small helper so the loop below reads as those four
 * steps. */

/* Which colour.  Normally it's by age — newest, middling, oldest.  In
 * wing-colour mode it's by which wing the point is on instead. */
static inline short trail_sample_color_pair(bool show_lobe, float lx,
                                            float age) {
  if (show_lobe)
    return (lx > 0.0f) ? CP_TRAIL_HEAD : CP_GHOST;
  if (age < 0.25f)
    return CP_TRAIL_HEAD;
  if (age < 0.60f)
    return CP_TRAIL_MID;
  return CP_TRAIL_TAIL;
}

/* How bright.  Mostly by depth — nearer is bolder — but the very newest
 * dots are always bold (the bright comet head) and the very oldest are
 * always faint (the fading tail). */
static inline attr_t trail_sample_depth_attr(float depth, int k, float age) {
  attr_t at;
  if (depth < -10.0f)
    at = A_BOLD; /* close to camera */
  else if (depth > 10.0f)
    at = A_DIM; /* far behind      */
  else
    at = A_NORMAL;
  if (k < 2)
    at = A_BOLD; /* comet head      */
  else if (age > 0.80f)
    at = A_DIM; /* fading tail     */
  return at;
}

/* Which character: the leading dot gets a marker, the rest a plain dot.
 * Ghosts use their own quieter pair. */
static inline char trail_sample_glyph(bool is_ghost, int k) {
  if (k == 0)
    return is_ghost ? 'x' : 'O';
  return is_ghost ? ',' : '.';
}

/* A little glow around the leading dot: '+' marks just above, below, and
 * to the sides, so the head looks bright and fat without smearing over
 * the trail behind it. */
static inline void paint_bloom_halo(int row, int col, int cols, int rows,
                                    WINDOW *w) {
  static const int hdr[4] = {-1, 1, 0, 0};
  static const int hdc[4] = {0, 0, -1, 1};
  wattron(w, COLOR_PAIR(CP_TRAIL_HEAD));
  for (int hi = 0; hi < 4; hi++) {
    int br = row + hdr[hi], bc = col + hdc[hi];
    if (br >= 1 && br < rows - 1 && bc >= 0 && bc < cols)
      mvwaddch(w, br, bc, '+');
  }
  wattroff(w, COLOR_PAIR(CP_TRAIL_HEAD));
}

/* Draw one trail, newest dot to oldest, using the four choices above,
 * plus a glow on the newest couple of dots.  Ghost trails skip the
 * wing-colour and the glow — they're always the same quiet grey comma. */
static void lorenz_draw_trail(const Trail *t, bool is_ghost, bool show_lobe,
                              float phi, float theta, float scale, int cx,
                              int cy, int cols, int rows, WINDOW *w) {
  int last_col = -999, last_row = -999;

  for (int k = 0; k < t->count; k++) {
    int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
    float age = (float)k / (float)(t->count > 1 ? t->count - 1 : 1);

    int col, row;
    float depth;
    if (!project(t->x[idx], t->y[idx], t->z[idx], phi, theta, scale, cx, cy,
                 cols, rows, &col, &row, &depth))
      continue;

    /* The curve is dense, so many points land on the same cell — skip
     * the repeat to save needless drawing. */
    if (col == last_col && row == last_row)
      continue;
    last_col = col;
    last_row = row;

    chtype attr;
    if (is_ghost) {
      attr = COLOR_PAIR(CP_GHOST) | A_DIM;
    } else {
      short cp = trail_sample_color_pair(show_lobe, t->x[idx], age);
      attr_t at = trail_sample_depth_attr(depth, k, age);
      attr = COLOR_PAIR(cp) | at;
    }

    char ch = trail_sample_glyph(is_ghost, k);

    wattron(w, attr);
    mvwaddch(w, row, col, ch);
    wattroff(w, attr);

    if (!is_ghost && k < 2)
      paint_bloom_halo(row, col, cols, rows, w);
  }
}

/* The full picture is painted back to front: stars, then the still-point
 * markers, then the trails (or the heatmap), then the gap-size readout.
 * Each layer gets its own little helper. */

/* Pick how big to draw the butterfly so it fills about 80% of the screen
 * height, allowing for the fact that terminal cells aren't square. */
static inline float compute_view_scale(int rows) {
  float usable = (float)(rows - 4);
  return usable * 0.80f / (39.0f * ASPECT);
}

/* The normal view: draw the ghost trails first (widest-nudge one first
 * so the closer ones sit on top), then the main trail over them. */
static void draw_trajectory_layer(const Lorenz *l, float scale, int cx, int cy,
                                  int cols, int rows, WINDOW *w) {
  if (l->show_ghost) {
    for (int g = N_GHOSTS - 1; g >= 0; g--)
      lorenz_draw_trail(&l->gt[g], true, false, l->phi, l->theta, scale, cx, cy,
                        cols, rows, w);
  }
  lorenz_draw_trail(&l->mt, false, l->show_lobe, l->phi, l->theta, scale, cx,
                    cy, cols, rows, w);
}

/* The heatmap view ('h').  Add this frame's trail into the tally, then
 * paint it.  No trails are drawn in this mode. */
static void draw_density_layer(const Lorenz *l, float scale, int cx, int cy,
                               int cols, int rows, WINDOW *w) {
  density_accumulate(&l->mt, l->phi, l->theta, scale, cx, cy, cols, rows);
  density_draw(w, cols, rows);
}

/* A line of text showing the live gap between the main point and the
 * closest ghost.  We track that one because the others quickly fly so
 * far apart their gap stops changing — the closest is where you watch
 * the slow pull-apart happen. */
static void draw_sep_indicator(const Lorenz *l, WINDOW *w, int rows) {
  float dx = l->gx[0] - l->mx;
  float dy = l->gy[0] - l->my;
  float dz = l->gz[0] - l->mz;
  float sep = sqrtf(dx * dx + dy * dy + dz * dz);

  wattron(w, COLOR_PAIR(CP_GHOST) | A_DIM);
  mvwprintw(w, rows - 3, 1, " ε-sep[g0=%.3f]: %.4f ", GHOST_EPS_TABLE[0], sep);
  wattroff(w, COLOR_PAIR(CP_GHOST) | A_DIM);
}

/* Paint the whole frame back to front: stars, the still-point markers,
 * then either the trails or the heatmap, and finally the gap readout. */
static void lorenz_draw(const Lorenz *l, WINDOW *w, int cols, int rows) {
  int cx = cols / 2;
  int cy = rows / 2;
  float scale = compute_view_scale(rows);

  stars_draw(l->phi, l->theta, scale, cx, cy, cols, rows, w);      /* 1 */
  equilibria_draw(l->phi, l->theta, scale, cx, cy, cols, rows, w); /* 2 */

  if (l->show_density)
    draw_density_layer(l, scale, cx, cy, cols, rows, w); /* 3b */
  else
    draw_trajectory_layer(l, scale, cx, cy, cols, rows, w); /* 3a */

  draw_sep_indicator(l, w, rows); /* 4 */
}

/* ── §6  scene ── */

/*
 * Scene — a thin box around the Lorenz world.
 *
 * Right now it just holds the one Lorenz, which already is the whole
 * visible world.  The wrapper exists as room to grow: if we ever want
 * something that doesn't belong inside Lorenz — a second butterfly to
 * compare, an overlay, a saved state for replay — it can be added here
 * without touching every function that already takes a Scene.  The
 * three Scene calls (set up, advance one tick, draw) stay the same.
 */
typedef struct {
  Lorenz lorenz; /* the entire visible world — see the Lorenz struct in §5 */
} Scene;

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  lorenz_init(&s->lorenz);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  (void)cols;
  (void)rows;
  lorenz_tick(&s->lorenz, dt);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  (void)alpha;
  (void)dt_sec;
  lorenz_draw(&s->lorenz, w, cols, rows);
}

/* ── §7  screen ── */

/*
 * Screen — just the current terminal size, kept apart from the
 * simulation so resizing the window can't disturb the math.  It's set
 * once at startup and again whenever the window changes size, and
 * nothing else writes it.  Everything that needs to know where the
 * middle of the screen is, or how big to draw, reads it from here.
 */
typedef struct {
  int cols, rows; /* current terminal width and height, in cells */
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

/* Draws the two status bars: the top row carries the title and a live
 * readout (frame rate, speed, which views are on, theme), and the bottom
 * row lists the keys.  Both are drawn bright and bold so they stay
 * readable over the moving trail. */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  const Lorenz *l = &sc->lorenz;

  /* top-left: the title, in the current theme's brightest colour */
  attron(COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);
  mvprintw(0, 1, " LORENZ ATTRACTOR ");
  attroff(COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);

  /* top-right: the live readout */
  char buf[200];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%3d Hz  ghost:%s  lobe:%s  density:%s  rot:%s"
           "  theme:%s  %s ",
           fps, sim_fps, l->show_ghost ? "on " : "off",
           l->show_lobe ? "on " : "off", l->show_density ? "on " : "off",
           l->auto_rotate ? "auto" : "manual", THEMES[g_theme_idx].name,
           l->paused ? "PAUSED " : "running");
  int hx = s->cols - (int)strlen(buf);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", buf);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* bottom row: the key list */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  r:reset  g:ghost  l:lobe  h:density"
           "  a:rot  arrows:view  [/]:Hz  t/T:theme ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8  app ── */

/*
 * App — the program as a whole, gathered in one place: the world, the
 * screen size, the speed setting, and a couple of flags the rest of the
 * loop watches.
 *
 * sim_fps (the speed) lives here, not with the math, because it's about
 * how fast you watch — how many simulation steps happen each real second
 * — not about the math itself.  Turning it up or down with [ and ] just
 * fast-forwards or slows the very same butterfly.
 *
 * The last two fields are flags the operating system can flip from
 * outside the loop: one to ask the program to quit (Ctrl-C), one to note
 * the window was resized.  Those handlers can't take any arguments, so
 * we keep one App at file scope (g_app) for them to poke.  The flags are
 * marked `volatile sig_atomic_t` for two reasons: `volatile` stops the
 * compiler from assuming they never change behind its back, and
 * `sig_atomic_t` is the one integer type the language promises can be
 * read and written safely even mid-interruption.  The handlers only flip
 * a flag and return — the loop does the real work next time around,
 * because the cleanup and screen calls aren't safe to run inside a
 * handler.
 */
typedef struct {
  Scene scene;                       /* the world (§6)                       */
  Screen screen;                     /* terminal size (§7)                   */
  int sim_fps;                       /* how fast we watch (10..120, [/] keys)*/
  volatile sig_atomic_t running;     /* set to 0 to quit cleanly             */
  volatile sig_atomic_t need_resize; /* set when the window was resized      */
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
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Lorenz *l = &app->scene.lorenz;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    l->paused = !l->paused;
    break;

  case 'r':
  case 'R':
    lorenz_init(l);
    density_reset(); /* start the heatmap tally over after a restart */
    break;

  case 'g':
  case 'G':
    l->show_ghost = !l->show_ghost;
    break;

  case 'a':
  case 'A':
    l->auto_rotate = !l->auto_rotate;
    break;

  case 'l':
  case 'L':
    l->show_lobe = !l->show_lobe;
    break;

  case 'h':
  case 'H':
    l->show_density = !l->show_density;
    if (l->show_density)
      density_reset();
    break;

  case KEY_LEFT:
    l->auto_rotate = false;
    l->phi -= 0.1f;
    break;
  case KEY_RIGHT:
    l->auto_rotate = false;
    l->phi += 0.1f;
    break;

  case KEY_UP:
    l->theta += 0.05f;
    if (l->theta > 1.4f)
      l->theta = 1.4f;
    break;
  case KEY_DOWN:
    l->theta -= 0.05f;
    if (l->theta < 0.1f)
      l->theta = 0.1f;
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
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    float alpha = (float)sim_accum / (float)tick_ns;

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

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
