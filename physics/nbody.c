/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nbody.c — gravity sandbox in the terminal
 *
 * A handful of point masses pull on each other through gravity, and we draw
 * where they go.  Ten ready-made setups (presets) range from one clean orbit
 * to whole galaxies colliding; trails show the paths so you can watch orbits,
 * slingshots, ejections, and the famous figure-8 dance.
 *
 * Worth knowing that the code can't tell you:
 *   - The figure-8 setup uses published exact starting numbers
 *     (Chenciner & Montgomery, Annals of Mathematics 152(3), 2000).
 *   - The "Pythagorean" chaotic setup follows Burrau (1913); its classic
 *     numerical solution is Szebehely & Peters, Astronomical J. 72, 1967.
 *   - The galaxy-merger tidal tails are inspired by Toomre & Toomre,
 *     Astrophysical J. 178, 1972.
 *   - For the why-orbits-don't-spiral theory behind the integrator, see
 *     Hairer, Lubich & Wanner, "Geometric Numerical Integration," 2nd ed.,
 *     Springer 2006, Ch. VI.
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

  MAX_BODIES = 32,
  TRAIL_LEN = 150, /* how many past positions each body remembers for its trail */
  SUB_STEPS = 4,   /* physics steps per tick; more = steadier orbits */
  N_PRESETS = 10,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How strong gravity is here.  Not the real-world number; it's tuned by hand
 * so a typical orbit takes a few pleasant seconds to go around on screen. */
#define G_CONST 500000.0f

/* A little fudge that keeps two bodies from feeling infinite force when they
 * pass nearly on top of each other (which would fling them off-screen).
 * We store the value squared.  It works out to a softening of 40 px. */
#define SOFT2 (40.0f * 40.0f)

/* Size, in pixels, that the figure-8 setup is drawn at.  Its published
 * starting numbers assume an abstract unit grid; this scales them to fit. */
#define F8_SCALE 150.0f

/* How far a body may wander before we give up on it and let it vanish:
 * this many times the screen size, leaving a margin past the visible edge. */
#define EJECT_FACTOR 2.5f

/* How many physics pixels make up one terminal character, across and down. */
#define CELL_W 8
#define CELL_H 16

/* The two text bars take up these rows (top and bottom).  Bodies and trails
 * skip them so flying particles never scribble over the readouts. */
#define TOP_HUD_ROWS 2
#define BOTTOM_HUD_ROWS 1

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

/* ── §3  color ── */

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(1, 196, COLOR_BLACK);
    init_pair(2, 208, COLOR_BLACK);
    init_pair(3, 226, COLOR_BLACK);
    init_pair(4, 46, COLOR_BLACK);
    init_pair(5, 51, COLOR_BLACK);
    init_pair(6, 75, COLOR_BLACK);
    init_pair(7, 201, COLOR_BLACK);
  } else {
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_GREEN, COLOR_BLACK);
    init_pair(5, COLOR_CYAN, COLOR_BLACK);
    init_pair(6, COLOR_BLUE, COLOR_BLACK);
    init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
  }
}

/* ── §4  coords — pixel ↔ cell ── */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5  entity — Body, NBody ── */

/* ── Body ── */

/*
 * One body: a single dot of mass moving under gravity.
 *
 * Everything the physics needs to push this body forward one step lives here,
 * plus the little memory of where it has been so we can draw its trail.  The
 * fields fall into four groups:
 *
 *   WHERE IT IS AND HOW IT'S MOVING (all in pixels, the physics works in pixels):
 *     x, y      its position on screen.
 *     vx, vy    how fast it's moving, in pixels per second.
 *     ax, ay    how hard gravity is currently pushing it (its acceleration).
 *               We hang onto this between steps because the stepping method
 *               needs both the old push and the freshly-computed new push to
 *               update the velocity correctly.
 *     mass      how heavy it is.  Heavier bodies pull harder and are nudged
 *               less by others' pull.
 *
 *   HOW IT LOOKS:
 *     color     which color this body (and its trail) is drawn in; also acts
 *               as its identity tag.  The actual character is chosen from the
 *               mass at draw time, not stored here.
 *
 *   ITS TRAIL (a fixed-size loop of recent positions, oldest overwritten):
 *     tx[], ty[]  the last TRAIL_LEN positions it visited, in pixels.
 *     thead       slot holding the most recent position.
 *     tcount      how many slots actually hold real positions yet — kept
 *                 separate from thead so a brand-new body doesn't draw a
 *                 trail through empty (zeroed) slots it never visited.
 *
 *   WHETHER IT STILL COUNTS:
 *     active    becomes false once the body drifts too far off-screen; after
 *               that the physics and the drawing both ignore it and it quietly
 *               disappears.  Happens a lot in the chaotic setups (5, 8, 9).
 */
typedef struct {
  /* where it is and how it's moving (pixels) */
  float x, y;   /* position */
  float vx, vy; /* velocity (px/s) */
  float ax, ay; /* current gravity push, kept for the next step */
  float mass;   /* heavier = pulls harder, nudged less */

  /* how it looks */
  int color; /* color-pair index and identity tag */

  /* its trail: a loop of recent positions, oldest gets overwritten */
  float tx[TRAIL_LEN]; /* past x positions */
  float ty[TRAIL_LEN]; /* past y positions */
  int thead;           /* slot with the newest position */
  int tcount;          /* how many slots hold real positions yet */

  /* whether it still counts */
  bool active; /* false once it wanders too far off-screen */
} Body;

static void body_trail_push(Body *b, float x, float y) {
  b->thead = (b->thead + 1) % TRAIL_LEN;
  b->tx[b->thead] = x;
  b->ty[b->thead] = y;
  if (b->tcount < TRAIL_LEN)
    b->tcount++;
}

/* ── NBody ── */

/*
 * The whole bunch of bodies, plus a few switches that belong to the group
 * rather than to any one body.
 *
 * The bodies live in a fixed-size array rather than a growable list — we just
 * reserve room for the most we'll ever need (MAX_BODIES) and use however many
 * the current setup wants.  Simple, fast, and the memory cost is tiny.
 *
 *   bodies[]      the array; only the first n entries are in use.
 *   n             how many bodies the current setup has.
 *   show_trails   whether trails are drawn ('t' toggles it).  Even when off,
 *                 we keep recording positions, so turning it back on picks up
 *                 mid-flight instead of starting from a blank trail.
 *   paused        whether the simulation is frozen (space toggles it).  When
 *                 paused, nothing moves and no new trail points are recorded.
 *   preset        which of the ten setups is loaded; shown in the top bar and
 *                 used when 'r' restarts the same one.
 */
typedef struct {
  Body bodies[MAX_BODIES]; /* only the first n are in use */
  int n;                   /* how many bodies in the current setup */
  bool show_trails;        /* 't' toggle */
  bool paused;             /* space toggle */
  int preset;              /* which setup is loaded */
} NBody;

/* ── Force computation: every body pulls on every other ── */

/* We add up each body's pull one pair at a time, so start the running totals
 * back at zero. */
static void zero_accelerations(Body *bodies, int n) {
  for (int i = 0; i < n; i++) {
    bodies[i].ax = 0.0f;
    bodies[i].ay = 0.0f;
  }
}

/* Work out the gravity between two bodies and add it to both of them at once.
 * The pull is equal and opposite (one tugs the other just as hard the other
 * way), so handling the pair together does the work once instead of twice.
 * The SOFT2 fudge in the distance keeps the force finite when they're nearly
 * touching, so close passes don't blow up. */
static void accumulate_pair_gravity(Body *bi, Body *bj) {
  float dx = bj->x - bi->x;
  float dy = bj->y - bi->y;
  float r2 = dx * dx + dy * dy + SOFT2; /* squared distance, softened */
  float r = sqrtf(r2);
  float inv = G_CONST / (r2 * r);

  /* same pull, opposite directions, scaled by the other body's mass */
  bi->ax += inv * bj->mass * dx;
  bi->ay += inv * bj->mass * dy;
  bj->ax -= inv * bi->mass * dx;
  bj->ay -= inv * bi->mass * dy;
}

/* Recompute the gravity push on every body: clear the totals, then add up the
 * pull from each pair of bodies still in play. */
static void nbody_forces(Body *bodies, int n) {
  zero_accelerations(bodies, n);
  for (int i = 0; i < n; i++) {
    if (!bodies[i].active)
      continue;
    for (int j = i + 1; j < n; j++) {
      if (!bodies[j].active)
        continue;
      accumulate_pair_gravity(&bodies[i], &bodies[j]);
    }
  }
}

/* ── Moving the bodies forward one step ──
 *
 * We move everything ahead a tiny slice of time using a method called
 * velocity Verlet.  The trick that makes it good: it moves each body using
 * the gravity push from the START of the step, then changes its speed using
 * the AVERAGE of the start push and the end push.  That little averaging is
 * why orbits keep their shape over time instead of slowly winding inward or
 * drifting outward.  One step goes: remember the old push, move the bodies,
 * drop any that flew off, work out the new push, update the speeds.
 */

/* Keep a copy of each body's current gravity push.  We need it again at the
 * end of the step, but working out the new push overwrites it, so copy it
 * aside first. */
static void verlet_stash_old_accelerations(const NBody *nb, float *ax_old,
                                           float *ay_old) {
  for (int i = 0; i < nb->n; i++) {
    ax_old[i] = nb->bodies[i].ax;
    ay_old[i] = nb->bodies[i].ay;
  }
}

/* Move each body to its new spot, using its current speed and the gravity
 * push from the start of the step (the old push we saved). */
static void verlet_drift_positions(NBody *nb, const float *ax_old,
                                   const float *ay_old, float dt) {
  for (int i = 0; i < nb->n; i++) {
    Body *b = &nb->bodies[i];
    if (!b->active)
      continue;
    b->x += b->vx * dt + 0.5f * ax_old[i] * dt * dt;
    b->y += b->vy * dt + 0.5f * ay_old[i] * dt * dt;
  }
}

/* Any body that has drifted far past the screen edge is switched off and
 * quietly forgotten, so the physics and drawing stop bothering with it. */
static void mark_ejected_bodies(NBody *nb, int cols, int rows) {
  float limit_x = (float)pw(cols) * EJECT_FACTOR;
  float limit_y = (float)ph(rows) * EJECT_FACTOR;
  for (int i = 0; i < nb->n; i++) {
    Body *b = &nb->bodies[i];
    if (!b->active)
      continue;
    if (fabsf(b->x) > limit_x || fabsf(b->y) > limit_y)
      b->active = false;
  }
}

/* Update each body's speed using the average of the gravity push at the
 * start and end of the step.  That averaging is the bit that keeps orbits
 * from slowly winding in or out. */
static void verlet_kick_velocities(NBody *nb, const float *ax_old,
                                   const float *ay_old, float dt) {
  for (int i = 0; i < nb->n; i++) {
    Body *b = &nb->bodies[i];
    if (!b->active)
      continue;
    b->vx += 0.5f * (ax_old[i] + b->ax) * dt;
    b->vy += 0.5f * (ay_old[i] + b->ay) * dt;
  }
}

/* One small step forward in time, reading as five plain moves: save the old
 * push, move the bodies, drop the ones that flew off, work out the new push,
 * update the speeds. */
static void nbody_step(NBody *nb, float dt, int cols, int rows) {
  float ax_old[MAX_BODIES], ay_old[MAX_BODIES];
  verlet_stash_old_accelerations(nb, ax_old, ay_old);
  verlet_drift_positions(nb, ax_old, ay_old, dt);
  mark_ejected_bodies(nb, cols, rows);
  nbody_forces(nb->bodies, nb->n);
  verlet_kick_velocities(nb, ax_old, ay_old, dt);
}

/* ── Presets ──
 *
 * Ten ready-made setups, ordered simplest first:
 *   0  Kepler          one clean orbit
 *   1  Binary Star     two equal stars circling each other
 *   2  Solar System    a sun and four planets
 *   3  Lagrange Tri    three stars in a rigidly spinning triangle
 *   4  Figure-8        three bodies chasing each other on a figure-8
 *   5  Pythagorean     three at rest that fall into chaos
 *   6  Black Hole      one heavy centre with 20 orbiters
 *   7  Galaxy          20 random bodies in a disc
 *   8  Binary BH       two heavy centres plus 20 light particles
 *   9  Galaxy Merger   two clusters on a collision course
 *
 * Each setup ends by calling nbody_forces() so every body has its
 * starting gravity push ready before the first move.
 */

/* ── Preset building blocks ──
 *
 * A few small helpers each setup is assembled from, so the setups read
 * as a plain list ("put a body here, this heavy, moving this fast")
 * instead of a wall of field assignments.
 */

/* A random number from 0 up to (but not including) 1, for picking random
 * distances, angles, and masses in the setups. */
static inline float random01(void) { return (float)rand() / (float)RAND_MAX; }

/* The shorter side of the screen, in pixels.  We size layouts against this so
 * they stay nicely framed even on a very wide or very tall terminal. */
static inline float screen_min_dim(int cols, int rows) {
  return (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows));
}

/* The sideways speed that gives a perfect circle around a single heavy body
 * at distance r.  Go slower and the orbit becomes an oval; go fast enough and
 * the body escapes entirely.  (v = sqrt(G*M / r)) */
static inline float kepler_circular_speed(float central_mass, float radius) {
  return sqrtf(G_CONST * central_mass / radius);
}

/* Set up one body in a single call: its place, speed, weight, and colour,
 * with a clean (empty) trail and switched on.  Every setup builds its bodies
 * with this. */
static void body_set(Body *b, float x, float y, float vx, float vy, float mass,
                     int color) {
  b->x = x;
  b->y = y;
  b->vx = vx;
  b->vy = vy;
  b->ax = 0.0f;
  b->ay = 0.0f;
  b->mass = mass;
  b->color = color;
  b->active = true;
  b->thead = 0;
  b->tcount = 0;
}

/* Cancel out the group's overall drift so the whole cluster stays centred on
 * screen instead of sliding off.  We work out how fast the group as a whole
 * is moving and subtract that from every body. */
static void subtract_com_velocity(NBody *nb) {
  float pcx = 0, pcy = 0, total = 0;
  for (int i = 0; i < nb->n; i++) {
    pcx += nb->bodies[i].vx * nb->bodies[i].mass;
    pcy += nb->bodies[i].vy * nb->bodies[i].mass;
    total += nb->bodies[i].mass;
  }
  if (total < 1e-9f)
    return;
  for (int i = 0; i < nb->n; i++) {
    nb->bodies[i].vx -= pcx / total;
    nb->bodies[i].vy -= pcy / total;
  }
}

/* Build one little galaxy: a heavy body in the middle with a handful of light
 * ones circling it, and the whole bunch gliding along at (vx, vy).  The merger
 * setup uses this twice to make two galaxies that then crash together. */
static void build_cluster(NBody *nb, int start, int n_orbiters, float cx,
                          float cy, float vx, float vy, float M_central,
                          float R, int color_central, int color_orbiters) {
  body_set(&nb->bodies[start], cx, cy, vx, vy, M_central, color_central);
  for (int i = 1; i <= n_orbiters; i++) {
    float r = R * (0.3f + 0.7f * random01());
    float ang = 2.0f * (float)M_PI * random01();
    float v = kepler_circular_speed(M_central, r);
    body_set(&nb->bodies[start + i], cx + r * cosf(ang), cy + r * sinf(ang),
             vx - sinf(ang) * v, vy + cosf(ang) * v, /* CCW tangent */
             1.0f, color_orbiters);
  }
}

/* Preset 0 — Kepler: the simplest case.  One heavy body sits still while a
 * light one circles it in a clean, almost-perfect ring. */
static void preset_kepler(NBody *nb, int cols, int rows) {
  nb->n = 2;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float R = screen_min_dim(cols, rows) * 0.25f;
  float M = 50.0f;

  body_set(&nb->bodies[0], cx, cy, 0, 0, M, 3);
  body_set(&nb->bodies[1], cx + R, cy, 0, kepler_circular_speed(M, R), 1.0f, 5);

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 1 — Binary Star: two equal stars waltzing around the point exactly
 * between them. */
static void preset_binary(NBody *nb, int cols, int rows) {
  nb->n = 2;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float R = screen_min_dim(cols, rows) * 0.18f;
  float m = 5.0f;
  float v = 0.5f * kepler_circular_speed(m, R); /* binary-star formula */

  body_set(&nb->bodies[0], cx - R, cy, 0, -v, m, 1);
  body_set(&nb->bodies[1], cx + R, cy, 0, v, m, 5);

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 2 — Solar System: a sun with four planets at growing distances.  You
 * can see the rule that the farther-out planets crawl around noticeably slower
 * than the inner ones (Kepler's third law). */
static void preset_solar(NBody *nb, int cols, int rows) {
  nb->n = 5;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float Rmax = screen_min_dim(cols, rows) * 0.32f;
  float M_sun = 200.0f;

  body_set(&nb->bodies[0], cx, cy, 0, 0, M_sun, 3);

  const float radii[4] = {0.30f, 0.50f, 0.72f, 1.00f};
  const int colors[4] = {7, 4, 5, 1}; /* mag, grn, cyn, red */
  for (int i = 0; i < 4; i++) {
    float r = Rmax * radii[i];
    body_set(&nb->bodies[i + 1], cx + r, cy, 0, kepler_circular_speed(M_sun, r),
             1.0f, colors[i]);
  }

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 3 — Lagrange Triangle: three equal stars at the corners of a triangle
 * that spins as one rigid shape, the corners forever holding their distances.
 * One of Lagrange's exact three-body solutions (1772).  It only just barely
 * holds together, so small rounding errors eventually pull it apart. */
static void preset_lagrange(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float L = screen_min_dim(cols, rows) * 0.28f; /* length of one side    */
  float r = L / sqrtf(3.0f);                    /* corner to the middle  */
  float m = 3.0f;
  float v = sqrtf(G_CONST * m / L); /* speed that keeps the triangle rigid */

  const int colors[3] = {1, 4, 5};
  for (int i = 0; i < 3; i++) {
    float ang = (float)i * (2.0f * (float)M_PI / 3.0f) -
                (float)M_PI * 0.5f; /* top vertex first */
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang),
             -v * sinf(ang), v * cosf(ang), /* CCW tangent */
             m, colors[i]);
  }

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 4 — Figure-8: three equal stars chasing each other forever around a
 * single figure-8 loop.  The exact starting numbers below are the published
 * ones (Chenciner & Montgomery, 2000); they're written for an abstract grid,
 * so we scale them up to fit the screen. */
static void preset_figure8(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;

  /* scale the abstract published numbers up to screen pixels */
  float L = F8_SCALE;
  float ts = sqrtf(L * L * L / G_CONST); /* scales the time numbers  */
  float vs = L / ts;                     /* scales the speed numbers */

  /* the exact published starting positions and speeds */
  const float q1x = -0.97000436f, q1y = 0.24308753f;
  const float q3x = 0.97000436f, q3y = -0.24308753f;
  const float v1x = 0.46620369f, v1y = 0.43236573f;
  const float v2x = -0.93240737f, v2y = -0.86473146f;

  body_set(&nb->bodies[0], cx + q1x * L, cy + q1y * L, v1x * vs, v1y * vs, 1.0f,
           1);
  body_set(&nb->bodies[1], cx, cy, v2x * vs, v2y * vs, 1.0f, 4);
  body_set(&nb->bodies[2], cx + q3x * L, cy + q3y * L, v1x * vs, v1y * vs, 1.0f,
           5);

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 5 — Pythagorean: three bodies of weight 3, 4, and 5 dropped from rest
 * at the corners of a 3-4-5 triangle (Burrau, 1913).  Famously chaotic — they
 * swing past each other, sling each other around, and one eventually gets
 * flung away for good. */
static void preset_pythagorean(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  /* how big to draw the triangle; smaller makes the action faster */
  float S = screen_min_dim(cols, rows) * 0.10f;

  const float xs[3] = {1.0f, -2.0f, 1.0f};
  const float ys[3] = {3.0f, -1.0f, -1.0f};
  const float ms[3] = {3.0f, 4.0f, 5.0f};
  const int cs[3] = {1, 4, 7};
  for (int i = 0; i < 3; i++) {
    body_set(&nb->bodies[i], cx + S * xs[i], cy + S * ys[i], 0, 0, /* at rest */
             ms[i], cs[i]);
  }

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 6 — Black Hole: one very heavy body in the middle with 20 light ones
 * orbiting.  We start them a touch slower than a perfect circle (the 0.98) so
 * the orbits slowly turn over time and you can watch them drift. */
static void preset_blackhole(NBody *nb, int cols, int rows) {
  nb->n = 21;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float M_bh = 100.0f;
  float min_r = 100.0f;
  float max_r = screen_min_dim(cols, rows) * 0.28f;

  body_set(&nb->bodies[0], cx, cy, 0, 0, M_bh, 3);

  for (int i = 1; i < nb->n; i++) {
    float r = min_r + (max_r - min_r) * random01();
    float ang = 2.0f * (float)M_PI * random01();
    float v = kepler_circular_speed(M_bh, r) * 0.98f; /* slightly slow */
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang),
             -v * sinf(ang), v * cosf(ang), /* CCW tangent */
             1.0f, ((i - 1) % (N_COLORS - 1)) + 1);
  }

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 7 — Galaxy: 20 random bodies scattered across a disc, each set
 * circling the middle.  We start them a fair bit slower than a perfect circle
 * (the 0.75) so the orbits are ovals and the whole thing looks livelier. */
static void preset_galaxy(NBody *nb, int cols, int rows) {
  nb->n = 20;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float R = screen_min_dim(cols, rows) * 0.30f;

  /* scatter the bodies evenly across the disc, not bunched at the centre */
  for (int i = 0; i < nb->n; i++) {
    float r = R * sqrtf(random01());
    float ang = 2.0f * (float)M_PI * random01();
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang), 0,
             0, /* v set below */
             1.0f + 3.0f * random01(), (i % N_COLORS) + 1);
  }

  /* add up all the weight to estimate how fast each body should circle */
  float M_total = 0;
  for (int i = 0; i < nb->n; i++)
    M_total += nb->bodies[i].mass;

  /* aim each body sideways, a bit slow, so its orbit is a gentle oval */
  for (int i = 0; i < nb->n; i++) {
    Body *b = &nb->bodies[i];
    float dx = b->x - cx, dy = b->y - cy;
    float r = sqrtf(dx * dx + dy * dy);
    if (r < 1.0f) {
      b->vx = 0;
      b->vy = 0;
      continue;
    }
    float v = kepler_circular_speed(M_total, r) * 0.75f;
    b->vx = -dy / r * v;
    b->vy = dx / r * v;
  }

  subtract_com_velocity(nb); /* keep the whole cluster centred on screen */
  nbody_forces(nb->bodies, nb->n);
}

/* Preset 8 — Binary Black Holes: two heavy bodies circling each other, ringed
 * by 20 light test particles.  Because the two heavies keep shifting around,
 * the light ones get jostled, swung about, and often flung clear off. */
static void preset_binary_bh(NBody *nb, int cols, int rows) {
  nb->n = 22;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float Rbh = screen_min_dim(cols, rows) * 0.08f;
  float Rmax = screen_min_dim(cols, rows) * 0.34f;
  float M_bh = 50.0f;
  float v_bh = 0.5f * kepler_circular_speed(M_bh, Rbh); /* binary-star speed */

  /* the two black holes circle each other */
  body_set(&nb->bodies[0], cx - Rbh, cy, 0, -v_bh, M_bh, 3);
  body_set(&nb->bodies[1], cx + Rbh, cy, 0, v_bh, M_bh, 3);

  /* light test particles in a wide ring, circling the pair's combined weight */
  for (int i = 2; i < nb->n; i++) {
    float r = Rbh * 3.0f + (Rmax - Rbh * 3.0f) * random01();
    float ang = 2.0f * (float)M_PI * random01();
    float v = kepler_circular_speed(2.0f * M_bh, r) * 0.95f;
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang),
             -v * sinf(ang), v * cosf(ang), /* CCW tangent */
             1.0f, ((i - 2) % (N_COLORS - 1)) + 1);
  }

  nbody_forces(nb->bodies, nb->n);
}

/* Preset 9 — Galaxy Merger: two small galaxies (a heavy centre plus 12
 * orbiters each) aimed at each other.  As they pass, each one's gravity tears
 * stars off the other, drawing the long curling tails you see in photos of
 * real galaxy collisions (like the Antennae galaxies). */
static void preset_merger(NBody *nb, int cols, int rows) {
  nb->n = 26;
  float W = (float)pw(cols);
  float H = (float)ph(rows);
  float D = screen_min_dim(cols, rows) * 0.30f; /* how far apart they start */
  float R = screen_min_dim(cols, rows) * 0.10f; /* size of each galaxy      */
  float Mc = 50.0f;
  float v_app = 0.4f * kepler_circular_speed(Mc, D); /* closing speed */

  /* two matching galaxies heading toward each other across the screen */
  build_cluster(nb, 0, 12, W * 0.5f - D, H * 0.5f, v_app, 0, Mc, R, 3, 1);
  build_cluster(nb, 13, 12, W * 0.5f + D, H * 0.5f, -v_app, 0, Mc, R, 3, 5);

  nbody_forces(nb->bodies, nb->n);
}

/* ── Preset lookup tables ──
 * The on-screen name and the build function for each setup, both looked up by
 * the preset number. */

static const char *preset_names[N_PRESETS] = {
    "Kepler",        /* 0 */
    "Binary Star",   /* 1 */
    "Solar System",  /* 2 */
    "Lagrange Tri",  /* 3 */
    "Figure-8",      /* 4 */
    "Pythagorean",   /* 5 */
    "Black Hole",    /* 6 */
    "Galaxy",        /* 7 */
    "Binary BH",     /* 8 */
    "Galaxy Merger", /* 9 */
};

typedef void (*PresetFn)(NBody *, int, int);

static const PresetFn preset_fns[N_PRESETS] = {
    preset_kepler,    preset_binary,      preset_solar,     preset_lagrange,
    preset_figure8,   preset_pythagorean, preset_blackhole, preset_galaxy,
    preset_binary_bh, preset_merger,
};

static void nbody_init(NBody *nb, int preset, int cols, int rows) {
  memset(nb, 0, sizeof *nb);
  nb->show_trails = true;
  nb->paused = false;
  if (preset < 0 || preset >= N_PRESETS)
    preset = 0;
  nb->preset = preset;
  preset_fns[preset](nb, cols, rows);
}

static void nbody_tick(NBody *nb, float dt, int cols, int rows) {
  if (nb->paused)
    return;

  float sub_dt = dt / (float)SUB_STEPS;
  for (int s = 0; s < SUB_STEPS; s++) {
    nbody_step(nb, sub_dt, cols, rows);
    /* save each body's spot so the trail stays smooth, not jumpy */
    for (int i = 0; i < nb->n; i++) {
      if (nb->bodies[i].active)
        body_trail_push(&nb->bodies[i], nb->bodies[i].x, nb->bodies[i].y);
    }
  }
}

/* How many bodies are still in play (haven't been flung off-screen); shown in
 * the top status bar. */
static int nbody_count_active(const NBody *nb) {
  int n = 0;
  for (int i = 0; i < nb->n; i++)
    if (nb->bodies[i].active)
      n++;
  return n;
}

/* ── Drawing the bodies and their trails ── */

/* Pick the character to draw a body with, so heavier bodies look denser:
 *   '@'  the heaviest — black holes, giant stars
 *   'O'  medium stars
 *   'o'  light stars
 *   '*'  dust and test particles */
static char glyph_for_mass(float mass) {
  if (mass > 50.0f)
    return '@';
  if (mass > 3.0f)
    return 'O';
  if (mass > 1.5f)
    return 'o';
  return '*';
}

/* True if a cell is in the drawable area — on screen and clear of the top and
 * bottom status bars, so flying bodies never scribble over the text. */
static bool cell_in_scene_band(int cx, int cy, int cols, int y_min, int y_max) {
  return cx >= 0 && cx < cols && cy >= y_min && cy < y_max;
}

/* Draw one body's recent path as a line of dots, walking from newest to
 * oldest.  The freshest part of the trail is bright and the older part is
 * faded, so the line glows just behind the body and dims away behind that. */
static void paint_body_trail(WINDOW *w, const Body *b, int cols, int y_min,
                             int y_max) {
  for (int k = 1; k < b->tcount; k++) {
    int idx = (b->thead - k + TRAIL_LEN) % TRAIL_LEN;
    int tx = px_to_cell_x(b->tx[idx]);
    int ty = px_to_cell_y(b->ty[idx]);
    if (!cell_in_scene_band(tx, ty, cols, y_min, y_max))
      continue;
    float age = (float)k / (float)TRAIL_LEN;
    chtype atr = (age < 0.4f) ? (chtype)COLOR_PAIR(b->color)
                              : (chtype)(COLOR_PAIR(b->color) | A_DIM);
    wattron(w, atr);
    mvwaddch(w, ty, tx, '.');
    wattroff(w, atr);
  }
}

/* Draw the body itself where it is right now.  Its character comes from its
 * weight, and we draw it bright so it stands out against its own faded trail. */
static void paint_body_glyph(WINDOW *w, const Body *b, int cols, int y_min,
                             int y_max) {
  int cx = px_to_cell_x(b->x);
  int cy = px_to_cell_y(b->y);
  if (!cell_in_scene_band(cx, cy, cols, y_min, y_max))
    return;
  char ch = glyph_for_mass(b->mass);
  wattron(w, COLOR_PAIR(b->color) | A_BOLD);
  mvwaddch(w, cy, cx, ch);
  wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
}

/* Draw all the live bodies — each one's trail (if trails are on) and then the
 * body on top — into the area between the two status bars. */
static void nbody_draw(const NBody *nb, WINDOW *w, int cols, int rows) {
  const int y_min = TOP_HUD_ROWS;
  const int y_max = rows - BOTTOM_HUD_ROWS; /* exclusive */

  for (int i = 0; i < nb->n; i++) {
    const Body *b = &nb->bodies[i];
    if (!b->active)
      continue;
    if (nb->show_trails)
      paint_body_trail(w, b, cols, y_min, y_max);
    paint_body_glyph(w, b, cols, y_min, y_max);
  }
}

/* ── §6  scene ── */

/*
 * Scene -- everything the simulation is doing right now.  Lives for the whole
 * run, owned by App, and rebuilt from scratch whenever you restart ('r') or
 * switch setups ('n' / 'p').
 *
 * Why it's so small: we keep the *simulation* facts here and let the drawing
 * side work out everything it needs on the fly.  The two run on separate
 * clocks (physics ticks at its own rate, drawing at about 60 a second), and
 * keeping them apart is what lets the window resize without ever disturbing
 * the physics.  The rough rule: if building a setup or stepping the physics
 * needs to read a value, it lives here; if only the drawing code needs it,
 * it's worked out per frame and not stored.
 *
 *   nbody       all the bodies plus the group switches (paused, trails on/off,
 *               which setup, how many bodies).
 *   cols, rows  the terminal size as of the last physics tick.  Kept here so
 *               the setup builders and the ejection check can place bodies and
 *               measure against the real screen without reaching into the
 *               ncurses layer.  Refreshed each tick -- the one spot where the
 *               physics and the screen size meet.
 */
typedef struct {
  NBody nbody;    /* the bodies plus the group switches */
  int cols, rows; /* terminal size as of the last tick */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  memset(&s->nbody, 0, sizeof s->nbody);
  nbody_init(&s->nbody, 0, cols, rows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  nbody_tick(&s->nbody, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  (void)alpha;
  (void)dt_sec;
  nbody_draw(&s->nbody, w, cols, rows);
}

/* ── §7  screen ── */

/*
 * Screen -- the current terminal size, remembered so the drawing code doesn't
 * have to ask ncurses for it every frame.  Set once at startup and updated
 * whenever the window is resized.
 *
 * Only the drawing side looks at this; the physics keeps its own copy.  So
 * when you resize the window, the bodies carry on exactly where they were --
 * only the status bars and the edge clipping rearrange to fit the new size.
 *
 *   cols    how many columns wide the terminal is
 *   rows    how many rows tall the terminal is
 */
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

/* The two-row status bar at the top.
 *   Top row:    which setup you're watching (left) and the speed readouts (right)
 *   Second row: how many bodies are left, whether trails are on, and
 *               paused-or-running. */
static void render_top_hud(const NBody *nb, int cols, double fps, int sim_fps) {
  /* top row: setup name on the left, speed readouts on the right */
  attron(COLOR_PAIR(2) | A_BOLD);
  mvprintw(0, 1, " N-BODY [%d/%d]: %s ", nb->preset + 1, N_PRESETS,
           preset_names[nb->preset]);
  attroff(COLOR_PAIR(2) | A_BOLD);

  char fps_buf[64];
  int fps_len = snprintf(fps_buf, sizeof fps_buf, " %5.1f fps   sim:%3d Hz ",
                         fps, sim_fps);
  int fps_col = cols - fps_len;
  if (fps_col < 0)
    fps_col = 0;
  attron(COLOR_PAIR(3) | A_BOLD);
  mvprintw(0, fps_col, "%s", fps_buf);
  attroff(COLOR_PAIR(3) | A_BOLD);

  /* second row: body count, trails on/off, and the paused/running word */
  int active = nbody_count_active(nb);
  char status[64];
  int status_len = snprintf(status, sizeof status, " bodies: %2d   trails: %s ",
                            active, nb->show_trails ? "on " : "off");
  attron(COLOR_PAIR(3) | A_BOLD);
  mvprintw(1, 1, "%s", status);
  attroff(COLOR_PAIR(3) | A_BOLD);

  /* red when paused, green when running */
  int cp_state = nb->paused ? 1 : 4;
  const char *state = nb->paused ? "PAUSED " : "running";
  attron(COLOR_PAIR(cp_state) | A_BOLD);
  mvprintw(1, 1 + status_len + 1, " %s ", state);
  attroff(COLOR_PAIR(cp_state) | A_BOLD);
}

/* The one-row key hint along the bottom, listing every key you can press.
 * Drawn bright so it stays readable over whatever's moving behind it. */
static void render_bottom_hud(int rows) {
  attron(COLOR_PAIR(5) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  r:restart  n/p:preset  t:trails  [/]:Hz ");
  attroff(COLOR_PAIR(5) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);
  render_top_hud(&sc->nbody, s->cols, fps, sim_fps);
  render_bottom_hud(s->rows);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8  app ── */

/*
 * App -- the whole program in one bundle, living for the entire run.  It owns
 * the simulation (scene) and the screen size (screen), and holds the few
 * loop-control values that belong to neither.
 *
 * There's a single global copy (g_app) for one reason: the signal handlers,
 * which fire when you press Ctrl-C or resize the window, can't be handed a
 * pointer to anything.  Putting the handoff in one named global keeps that
 * narrow -- the handlers touch only the two flags below and nothing else.
 *
 *   scene         the simulation -- see Scene.
 *   screen        the terminal size -- see Screen.
 *   sim_fps       how many physics ticks per second, adjustable with '[' and
 *                 ']'.  Separate from the drawing rate, so you can speed the
 *                 physics up or slow it down without touching the frame rate.
 *   running       set to 0 to quit.  Pressing 'q'/ESC or hitting Ctrl-C clears
 *                 it.  Marked `volatile sig_atomic_t` because a signal handler
 *                 may flip it at any instant while the main loop is reading it,
 *                 which is the only safe way to share a flag across that line.
 *   need_resize   raised by the resize signal, handled on the next loop pass
 *                 (which re-reads the new terminal size).  Same reason for the
 *                 volatile sig_atomic_t.
 *
 * Reference: POSIX.1-2017, §2.4 Signal Concepts -- sig_atomic_t is the
 * standard type for a flag shared with a signal handler, and volatile stops
 * the compiler from caching it across the loop.
 */
typedef struct {
  Scene scene;                       /* the simulation */
  Screen screen;                     /* the terminal size */
  int sim_fps;                       /* physics ticks per second */
  volatile sig_atomic_t running;     /* 0 = quit */
  volatile sig_atomic_t need_resize; /* 1 = window was resized */
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
  NBody *nb = &app->scene.nbody;
  Screen *sc = &app->screen;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    nb->paused = !nb->paused;
    break;

  case 'r':
  case 'R':
    nbody_init(nb, nb->preset, sc->cols, sc->rows);
    break;

  case 'n':
  case 'N':
    nbody_init(nb, (nb->preset + 1) % N_PRESETS, sc->cols, sc->rows);
    break;

  case 'p':
  case 'P': {
    int p = nb->preset - 1;
    if (p < 0)
      p = N_PRESETS - 1;
    nbody_init(nb, p, sc->cols, sc->rows);
    break;
  }

  case 't':
  case 'T':
    nb->show_trails = !nb->show_trails;
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
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

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
