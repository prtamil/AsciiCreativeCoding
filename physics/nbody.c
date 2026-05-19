/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nbody.c — N-Body Gravity Simulation
 *
 * Point masses with softened 1/r² gravity integrated with velocity Verlet.
 * Trails reveal orbital paths, slingshots, ejections, and figure-8
 * choreographies.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Force on body i from body j:
 *   F_ij = G · m_i · m_j · (r_j − r_i) / (|r_j − r_i|² + ε²)^(3/2)
 *
 * ε (softening) prevents 1/r² singularity when bodies pass close.
 *
 * Velocity Verlet integration (2nd-order symplectic):
 *   x_new = x + v·dt + ½·a·dt²
 *   compute a_new from x_new
 *   v_new = v + ½·(a + a_new)·dt
 *
 * Physics lives in pixel space (px, py).
 *   pw = cols × CELL_W,  ph = rows × CELL_H
 *   Body drawn at: px_to_cell_x(px), px_to_cell_y(py)
 *
 * Ten presets, simple → complex (cycle with n / p).
 * Each row: setup + what you should see on screen.
 *   0  Kepler          1 heavy + 1 light; closed ellipse, textbook orbit
 *   1  Binary Star     2 equal masses; mirror-image dance around the midpoint
 *   2  Solar System    sun + 4 planets; outer planets visibly slower (T ∝
 * r^1.5) 3  Lagrange Tri    3 equal masses at corners; rigid equilateral
 * rotation 4  Figure-8        3 bodies chasing each other along an ∞ curve 5
 * Pythagorean     m=3,4,5 at rest; chaotic close encounters, ejection 6  Black
 * Hole      heavy yellow giant + 20 orbiters; precessing rosettes 7  Galaxy 20
 * random bodies in a disc; mildly elliptical swirl 8  Binary BH       2 heavies
 * + 20 testers; resonances, slingshot ejects 9  Galaxy Merger   2 clusters
 * approach; tidal tails strip stars on flyby
 *
 * WHAT YOU SEE ON SCREEN
 * ─────────────────────────────────────────────────────────────────────
 *
 * Top HUD (2 rows):
 *   row 0   " N-BODY [i/10]: <preset> "   (orange, left)
 *           " <fps> fps   sim:<hz> Hz "   (yellow, right)
 *   row 1   " bodies: N   trails: on "    (yellow)  +
 *           " PAUSED " / " running "      (red / green)
 *
 * Scene (middle band, between the HUDs):
 *   Bodies   one glyph at the current pixel position, color = identity.
 *            Glyph is picked by mass:
 *              '@'  m > 50      black-hole-class (heaviest)
 *              'O'  m >  3      medium star
 *              'o'  m >  1.5    light star
 *              '*'  otherwise   test particle / dust
 *   Trails   '.' marks of the last TRAIL_LEN positions per body.
 *            Recent trail (age < 40%) drawn bold; older fades to dim --
 *            so the curve in front of the body is bright and the
 *            history dims behind.  Toggle with 't'.
 *   Eject    Bodies wandering past EJECT_FACTOR × screen vanish silently
 *            (common in Pythagorean / Binary BH / Galaxy Merger).
 *
 * Bottom HUD (1 row): action keys, bright cyan + bold.
 *
 * Visual cues to recognise each preset family:
 *   Closed loops    perfectly periodic orbits     (Kepler, Binary, Solar,
 *                                                  Lagrange, Figure-8)
 *   Rosettes        nearly-closed orbits that
 *                   slowly rotate their pericentre (Black Hole)
 *   Chaos           irregular swings, near-misses,
 *                   sudden ejections             (Pythagorean, Binary BH)
 *   Tidal tails     long streams of particles
 *                   pulled between heavy bodies   (Galaxy Merger)
 *
 * ─────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart current preset
 *   n / p     next / previous preset
 *   t         toggle trails
 *   ] / [     sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra nbody.c -o nbody -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Velocity Verlet (Störmer-Verlet) integration.
 *                  2nd-order symplectic integrator — conserves a modified
 *                  Hamiltonian (energy drifts only by round-off, not by
 *                  accumulating phase error like explicit Euler).
 *                  x_new = x + v·dt + ½·a·dt²
 *                  a_new = F(x_new)/m
 *                  v_new = v + ½·(a + a_new)·dt
 *
 * Physics        : Softened Newtonian gravity.
 *                  Real 1/r² force → singularity when r→0.  Softening ε
 *                  (SOFT2 = ε²) limits force magnitude at close approach,
 *                  modelling extended mass distributions or simply
 *                  preventing numerical blow-up.  The force becomes:
 *                  F = G·m₁·m₂·r / (r² + ε²)^(3/2)
 *
 * Performance    : O(N²) force computation each sub-step (brute force).
 *                  For N≤32 this is fast; N-body above ~1000 would need
 *                  Barnes-Hut (O(N log N)) or FMM (O(N)).
 *                  SUB_STEPS=4 improves accuracy without changing display fps.
 *
 * Math           : Figure-8 choreography (Chenciner & Montgomery, 2000).
 *                  An exact periodic solution where 3 equal masses chase
 *                  each other around a figure-8 curve.  Initial conditions
 *                  in natural units (G=1, m=1) are rescaled to pixel space
 *                  using dimensional analysis.
 *
 * Data-structure : Per-body ring-buffer trails; each body stores its own
 *                  TRAIL_LEN positions.  Memory: 32 bodies × 150 × 8 B
 *                  = 38 KB — fits easily in L1 cache for small N.
 *
 * References
 * ─────────────────────────────────────────────────────────────────────
 *
 *   Integration & N-body methods:
 *
 *   [1] Hairer, Lubich, Wanner.  "Geometric Numerical Integration,"
 *       2nd ed.  Springer, 2006.  Ch. VI is the canonical theory of
 *       symplectic integrators; velocity Verlet is the lowest-order
 *       example.  Read first if you want to know WHY orbits don't
 *       spiral despite the round-off.
 *
 *   [2] Press, Teukolsky, Vetterling, Flannery.  "Numerical Recipes,"
 *       3rd ed.  §17.4 (Verlet) and §20.5 (softening) are the practical
 *       C-language references closest in spirit to this file.
 *
 *   [3] Aarseth, S. J.  "Gravitational N-Body Simulations: Tools and
 *       Algorithms."  Cambridge University Press, 2003.  Standard
 *       reference for production N-body codes — softening regimes,
 *       neighbour schemes, regularisation of close encounters.
 *
 *   Classical 3-body solutions (presets 3, 4, 5):
 *
 *   [4] Lagrange, J.-L.  "Essai sur le problème des trois corps."
 *       Œuvres complètes, vol. 6, 1772.  Original derivation of the
 *       five Lagrange points; the equilateral L4/L5 family is preset 3.
 *
 *   [5] Chenciner, A. & Montgomery, R.  "A remarkable periodic solution
 *       of the three-body problem in the case of equal masses."
 *       Annals of Mathematics 152(3), 881-901, 2000.  Figure-8
 *       existence proof; preset 4 uses these published ICs verbatim.
 *
 *   [6] Szebehely, V. & Peters, C. F.  "Complete solution of a general
 *       problem of three bodies."  Astronomical Journal 72, 876-883,
 *       1967.  Canonical numerical integration of Burrau's Pythagorean
 *       problem (preset 5) -- where the now-classic chaotic behaviour
 *       and ejection event were first traced.
 *
 *   Astrophysics (preset 9 inspiration):
 *
 *   [7] Toomre, A. & Toomre, J.  "Galactic bridges and tails."
 *       Astrophysical Journal 178, 623-666, 1972.  The pioneering
 *       restricted-N-body simulation of galaxy mergers; the long
 *       spiral tails in preset 9 come straight out of this paper.
 *
 *   Rendering:
 *
 *   [8] Bourke, P.  "Character representation of grey scale images."
 *       paulbourke.net/dataformats/asciiart/  Tradition of ordering
 *       ASCII glyphs by density; informs the '@' / 'O' / 'o' / '*'
 *       mass-glyph scale in glyph_for_mass().
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500,
  N_COLORS = 7,

  MAX_BODIES = 32,
  TRAIL_LEN = 150, /* trail ring-buffer length per body     */
  SUB_STEPS = 4,   /* velocity-Verlet sub-steps per tick    */
  N_PRESETS = 10,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* G_CONST: gravitational constant in pixel-space units (px³ / (mass · s²)).
 * In SI: G = 6.67×10⁻¹¹ N·m²/kg².  Here we define G empirically so that
 * two unit-mass bodies 200 px apart produce a visually interesting orbital
 * period of a few seconds.  Circular orbit period: T = 2π√(r³/GM).
 * At r=200 px, T = 2π√(8e6/5e5) ≈ 2π·4 ≈ 25 s — gentle galactic timescale. */
#define G_CONST 500000.0f

/* SOFT2: softening length squared (ε² in pixels²).  ε = 40 px.
 * Without softening, two bodies passing within 1 px would produce
 * force ∝ 1/r² → ∞, flinging them off-screen.
 * ε = 40 px is ~5% of a typical 800 px wide screen — suppresses
 * singularity at close approach while leaving distant gravity accurate.  */
#define SOFT2 (40.0f * 40.0f)

/* F8_SCALE: natural length unit for the figure-8 choreography (pixels).
 * The published ICs use G=1, m=1.  F8_SCALE converts natural length → pixels.
 * Time and velocity scales are derived from F8_SCALE and G_CONST via
 * dimensional analysis: t_scale = sqrt(L³/G), v_scale = L/t_scale.       */
#define F8_SCALE 150.0f

/* EJECT_FACTOR: body deactivated when |pos| > EJECT_FACTOR × screen half-size.
 * 2.5 gives a buffer zone outside the visible area before discarding.     */
#define EJECT_FACTOR 2.5f

/* Cell dimensions for pixel↔cell conversion */
#define CELL_W 8
#define CELL_H 16

/* HUD layout: status zone on top, action zone on bottom.
 *   TOP    = title + fps/sim Hz on row 0; bodies/trails/paused on row 1
 *   BOTTOM = single action-key row
 * Body and trail rendering clip out these rows so the HUD is never
 * overwritten by orbital motion. */
#define TOP_HUD_ROWS 2
#define BOTTOM_HUD_ROWS 1

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Body, NBody                                               */
/* ===================================================================== */

/* ── Body ──────────────────────────────────────────────────────────── */

/*
 * Body -- one point-mass particle.
 *
 * INTENT: hold every per-particle quantity the integrator needs to
 * advance a body by one velocity-Verlet half-step, plus the trail
 * ring buffer that paints the orbit on screen.  Sized so MAX_BODIES
 * copies fit comfortably in L1 cache (~38 KB total at MAX_BODIES=32).
 *
 * Members are grouped by purpose:
 *
 *   STATE VECTOR -- the 6-tuple (x,y, vx,vy, ax,ay) that Verlet steps.
 *     x, y         position in PIXEL units (px).  All physics lives in
 *                  pixel space; conversion to terminal cells happens
 *                  only at draw time via px_to_cell_x/y().
 *     vx, vy       velocity in px/s.
 *     ax, ay       acceleration from the most recent force evaluation.
 *                  Velocity Verlet needs the OLD a to step position
 *                  and the NEW a to step velocity, so the current a
 *                  is always cached here between sub-steps:
 *                      x_new = x + v dt + ½ a_old dt²
 *                      a_new = F(x_new) / m
 *                      v_new = v + ½ (a_old + a_new) dt
 *     mass         scalar; enters in two places:
 *                    F_ij = G m_i m_j r_ij / (|r_ij|² + ε²)^(3/2)
 *                    a_i  = F_i / m_i
 *                  Used by accumulate_pair_gravity() to apply equal-
 *                  and-opposite a's (Newton's 3rd law).
 *
 *   APPEARANCE -- read by paint_body_trail() / paint_body_glyph().
 *     color        1-based ncurses color-pair index; per-body identity
 *                  tag, also used to tint the trail.  Glyph itself is
 *                  picked from mass by glyph_for_mass() at draw time
 *                  (@ / O / o / *).
 *
 *   TRAIL RING BUFFER -- visualises orbital history.
 *     tx[], ty[]   last TRAIL_LEN positions in pixel units; ring
 *                  buffer with newest at thead.
 *     thead        index of the newest sample (0..TRAIL_LEN-1).
 *     tcount       number of VALID samples, clamped at TRAIL_LEN.
 *                  Distinct from thead so early frames don't render
 *                  zeroed slots as fake trail points.
 *
 *   LIFECYCLE:
 *     active       false once the body has been ejected past
 *                  EJECT_FACTOR × screen.  Inactive bodies are skipped
 *                  by force, integrator, and renderer alike -- they
 *                  vanish silently.  Common in chaotic presets (5, 8, 9).
 *
 * References:
 *   [1] Hairer, Lubich, Wanner.  Geometric Numerical Integration,
 *       2nd ed., Springer 2006.  Ch. VI; velocity Verlet is the
 *       canonical low-order symplectic scheme that integrates the
 *       state vector below.
 *   [2] Aarseth.  Gravitational N-Body Simulations, CUP 2003, §1.2 --
 *       discusses softening parameter ε and its role in the
 *       gravitational force computation that mutates (ax, ay).
 */
typedef struct {
  /* state vector (pixel space) */
  float x, y;   /* position (px)              */
  float vx, vy; /* velocity (px/s)            */
  float ax, ay; /* cached accel from last F   */
  float mass;   /* scalar mass                */

  /* appearance */
  int color; /* 1-based ncurses pair index */

  /* trail ring-buffer */
  float tx[TRAIL_LEN]; /* historical x positions     */
  float ty[TRAIL_LEN]; /* historical y positions     */
  int thead;           /* newest sample index (mod)  */
  int tcount;          /* valid samples, <= TRAIL_LEN */

  /* lifecycle */
  bool active; /* false once ejected off-screen */
} Body;

static void body_trail_push(Body *b, float x, float y) {
  b->thead = (b->thead + 1) % TRAIL_LEN;
  b->tx[b->thead] = x;
  b->ty[b->thead] = y;
  if (b->tcount < TRAIL_LEN)
    b->tcount++;
}

/* ── NBody ─────────────────────────────────────────────────────────── */

/*
 * NBody -- the full N-particle system + sim-level scheduler flags.
 *
 * INTENT: bundle the array of Body's with state that is NOT a
 * per-particle property: how many bodies are populated, whether the
 * integrator is paused, whether trails are drawn, and which preset
 * is loaded.
 *
 * The Body array is a fixed-size MAX_BODIES slab embedded in BSS,
 * not a dynamic vector.  Rationale: the O(N²) force kernel benefits
 * from contiguous memory, and the inner loop produces tighter code
 * when the array bound is a compile-time constant.  Memory cost is
 * predictable (MAX_BODIES × sizeof(Body) ≈ 38 KB), entirely L1-cache
 * resident for the typical N ≤ 26.
 *
 * Members:
 *
 *   bodies[]      Backing slab.  Only indices [0, n) are populated;
 *                 unused slots are memset to zero by nbody_init().
 *   n             Number of bodies in the current preset
 *                 (2..MAX_BODIES).  Inner loops cap at n, not
 *                 MAX_BODIES, so unused slots cost nothing.
 *   show_trails   bool, toggled by 't'.  When false, nbody_draw()
 *                 skips the trail render, but body_trail_push()
 *                 keeps recording -- so toggling back resumes the
 *                 trail mid-flight rather than restarting blank.
 *   paused        bool, toggled by SPACE.  Short-circuits
 *                 nbody_tick(); positions, velocities, accelerations,
 *                 AND trails freeze.  Force values stay valid so
 *                 resume is seamless.
 *   preset        Current preset index in [0, N_PRESETS).  Used by
 *                 'r' (restart same preset) and shown in the top HUD
 *                 as "[i/N]".
 *
 * Reference:
 *   Aarseth, S. J.  Gravitational N-Body Simulations, CUP 2003.
 *   Ch. 1-3 lay out the standard structure for direct-summation
 *   N-body codes; the (bodies[], n) pair is the textbook layout.
 */
typedef struct {
  Body bodies[MAX_BODIES]; /* backing slab; only [0,n) live */
  int n;                   /* active count for current preset */
  bool show_trails;        /* 't' toggle */
  bool paused;             /* SPACE toggle */
  int preset;              /* index in preset_fns[] */
} NBody;

/* ── Force computation: O(N²) softened Newtonian gravity ─────────────── */

/*
 * zero_accelerations() -- reset a's to zero before the pair-sum.
 * Force computation accumulates; we start each call from a clean slate.
 */
static void zero_accelerations(Body *bodies, int n) {
  for (int i = 0; i < n; i++) {
    bodies[i].ax = 0.0f;
    bodies[i].ay = 0.0f;
  }
}

/*
 * accumulate_pair_gravity() -- add the softened gravitational pull
 * between two bodies into BOTH of their accelerations.
 *
 *   F_ij = G m_i m_j (r_j - r_i) / (|r|² + ε²)^(3/2)
 *   a_i  = F_i / m_i = sum_j G m_j (r_j - r_i) / (|r|² + ε²)^(3/2)
 *
 * Implementation detail: the per-mass factor G / (r² + ε²)^(3/2) is
 * computed once as `inv` and multiplied by the OTHER body's mass.
 * Newton's third law is exploited by applying the same `inv * (dx,dy)`
 * with opposite sign and the other body's mass -- so each unordered
 * pair costs one inv-sqrt, half the work of a naive double loop.
 *
 * Softening ε (SOFT2 = ε²) prevents 1/r² blow-up at close approach
 * (Aarseth, §1.2).
 */
static void accumulate_pair_gravity(Body *bi, Body *bj) {
  float dx = bj->x - bi->x;
  float dy = bj->y - bi->y;
  float r2 = dx * dx + dy * dy + SOFT2; /* softened |r|² */
  float r = sqrtf(r2);
  float inv = G_CONST / (r2 * r); /* G / (|r|² + ε²)^(3/2) */

  /* Newton's 3rd law: equal-magnitude, opposite-direction a's */
  bi->ax += inv * bj->mass * dx;
  bi->ay += inv * bj->mass * dy;
  bj->ax -= inv * bi->mass * dx;
  bj->ay -= inv * bi->mass * dy;
}

/*
 * nbody_forces() -- recompute a's for every body from current positions.
 * Pseudocode: clear all a's, then sum over each unordered active pair.
 */
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

/* ── Velocity Verlet integrator (drift → forces → kick) ──────────────── *
 *
 * The four helpers below implement the canonical velocity-Verlet step:
 *
 *   x_new = x + v dt + ½ a_old dt²        (DRIFT, with a_old)
 *   a_new = F(x_new) / m                  (FORCE evaluation)
 *   v_new = v + ½ (a_old + a_new) dt      (KICK, averaging a_old and a_new)
 *
 * Splitting it this way preserves the integrator's 2nd-order symplectic
 * property (Hairer-Lubich-Wanner Ch. VI): the modified Hamiltonian is
 * conserved up to round-off, so energy doesn't drift secularly the way
 * it does with explicit Euler.
 *
 * "Drift" and "kick" are standard symplectic-integrator names:
 *   drift    advance position holding momentum fixed
 *   kick     advance momentum (velocity) holding position fixed
 */

/*
 * verlet_stash_old_accelerations() -- copy current (ax, ay) into
 * the caller's temp arrays.  We need a_old AFTER the position update
 * (which overwrites a via nbody_forces), so we cache them up front.
 */
static void verlet_stash_old_accelerations(const NBody *nb, float *ax_old,
                                           float *ay_old) {
  for (int i = 0; i < nb->n; i++) {
    ax_old[i] = nb->bodies[i].ax;
    ay_old[i] = nb->bodies[i].ay;
  }
}

/*
 * verlet_drift_positions() -- the position half of Verlet:
 *   x += v dt + ½ a_old dt²
 * Uses a_old (passed in) because by the time the matching kick runs,
 * a will have been replaced with a_new from the force evaluation.
 */
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

/*
 * mark_ejected_bodies() -- post-drift cull.  Bodies that have wandered
 * past EJECT_FACTOR × screen are marked inactive (vanish silently);
 * subsequent force/integrator/render passes skip them.
 */
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

/*
 * verlet_kick_velocities() -- the velocity half of Verlet:
 *   v += ½ (a_old + a_new) dt
 * Averaging a_old and a_new is what makes the scheme 2nd-order and
 * symplectic (a simple v += a_new dt would be 1st-order semi-implicit).
 */
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

/*
 * nbody_step() -- one velocity-Verlet substep.  Reads as the 5-line
 * algorithm: stash, drift, cull, recompute forces, kick.
 */
static void nbody_step(NBody *nb, float dt, int cols, int rows) {
  float ax_old[MAX_BODIES], ay_old[MAX_BODIES];
  verlet_stash_old_accelerations(nb, ax_old, ay_old);
  verlet_drift_positions(nb, ax_old, ay_old, dt);
  mark_ejected_bodies(nb, cols, rows);
  nbody_forces(nb->bodies, nb->n);
  verlet_kick_velocities(nb, ax_old, ay_old, dt);
}

/* ── Preset initialisation ──────────────────────────────────────────────
 *
 * Ten presets in simple → complex order.  Index = complexity rank:
 *   0  Kepler          2 bodies, clean circular orbit
 *   1  Binary Star     2 equal masses, common barycenter
 *   2  Solar System    1 sun + 4 planets (Kepler's 3rd law)
 *   3  Lagrange Tri    3 equal masses, equilateral rigid rotation
 *   4  Figure-8        Chenciner-Montgomery 3-body choreography
 *   5  Pythagorean     Burrau 1913: m=3,4,5 at rest, chaotic
 *   6  Black Hole      central mass + 20 orbiters
 *   7  Galaxy          20 random bodies in a disc
 *   8  Binary BH       2 heavy + 20 test particles
 *   9  Galaxy Merger   2 clusters on collision course
 *
 * Every preset finishes with nbody_forces() so a-values are seeded
 * for the first velocity-Verlet half-step.
 * ───────────────────────────────────────────────────────────────────── */

/* ── Preset construction helpers ─────────────────────────────────────── *
 *
 * Six small named operations every preset is built from.  They turn
 * each factory from a wall of struct-field assignments into a flat
 * pseudocode list of physics statements: "place body here at this
 * mass with this circular-orbit speed".
 */

/*
 * random01() -- uniform sample in [0, 1) from libc rand().
 * Used wherever a preset needs randomness (radii, angles, masses).
 */
static inline float random01(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * screen_min_dim() -- shorter of the two screen pixel dimensions.
 * Used as the natural length scale for placing bodies so layouts
 * stay framed when the terminal is much wider than tall (or v.v.).
 */
static inline float screen_min_dim(int cols, int rows) {
  return (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows));
}

/*
 * kepler_circular_speed() -- tangential speed for a closed CIRCULAR
 * orbit around a single point mass at the origin:
 *
 *   v = sqrt(G·M / r)
 *
 * Smaller v gives an ellipse with the start point at apoapsis; larger
 * v puts it at periapsis; v² > 2·G·M/r is unbound (parabolic/hyperbolic).
 */
static inline float kepler_circular_speed(float central_mass, float radius) {
  return sqrtf(G_CONST * central_mass / radius);
}

/*
 * body_set() -- one-line body initialisation used by every preset.
 * Fills the state vector (x, y, vx, vy), zeroes a, sets mass/color,
 * clears the trail ring buffer, and marks the body active.
 */
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

/*
 * subtract_com_velocity() -- shift into the centre-of-mass (barycentric)
 * frame so the cluster as a whole stays put on screen.
 *
 *   p_total = Σ m_i v_i        v_CoM = p_total / Σ m_i
 *   v_i' = v_i - v_CoM    →    Σ m_i v_i' = 0
 */
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

/*
 * build_cluster() -- populate bodies[start..start+n_orbiters] with a
 * heavy central body + n_orbiters in random circular orbits around it.
 * The cluster as a whole drifts at (vx, vy).  Used by preset_merger
 * to assemble the two galaxies symmetrically.
 *
 *   r ∈ [0.3R, R]            uniform in radius (not in area)
 *   ang ∈ [0, 2π)            uniform tangent direction
 *   v_orbit = sqrt(G·M_central / r)   circular around M_central
 *   v_body  = v_drift + v_orbit_tangent
 */
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

/*
 * Preset 0 — Kepler (Two-Body Orbit)
 * 1 heavy + 1 light body in a near-circular Kepler orbit.  The cleanest
 * gravitational system; an exact closed-form elliptical solution.
 *
 *   v = sqrt(G·M / r)        (circular orbit speed)
 */
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

/*
 * Preset 1 — Binary Star
 * Two equal masses orbiting their common barycenter.
 *
 *   For equal mass m at half-separation R from CoM (full separation 2R):
 *     gravity   F = G·m² / (2R)²
 *     centripetal F = m·v² / R
 *     => v = (1/2) · sqrt(G·m / R)
 */
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

/*
 * Preset 2 — Solar System
 * Central "sun" + 4 "planets" at increasing radii.  Demonstrates
 * Kepler's 3rd law visually: T² ∝ r³, so outer planets sweep
 * noticeably slower than inner ones.
 *
 *   v(r) = sqrt(G·M_sun / r)
 */
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

/*
 * Preset 3 — Lagrange Equilateral Triangle (Euler-Lagrange, 1772)
 * 3 equal masses at the vertices of an equilateral triangle, rotating
 * rigidly as a single body.  One of Lagrange's 5 explicit central-
 * configuration solutions of the 3-body problem.
 *
 *   side length L, vertex-to-centroid distance r = L/√3
 *   Two gravity forces project onto the centroid direction at 30°:
 *     F_centripetal = 2 · G·m²/L² · cos(30°) = G·m²·√3 / L²
 *   Equating to m·ω²·r and solving:
 *     ω = sqrt(3 G m / L³),  v = ω·r = sqrt(G·m / L)
 *
 * For equal masses the triangular L4/L5 equilibrium is marginally
 * stable (Gascheau 1843): perfect ICs hold for many revolutions before
 * numerical drift compounds.
 */
static void preset_lagrange(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float L = screen_min_dim(cols, rows) * 0.28f; /* triangle side  */
  float r = L / sqrtf(3.0f);                    /* vertex→centroid */
  float m = 3.0f;
  float v = sqrtf(G_CONST * m / L); /* Lagrange tangent */

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

/*
 * Preset 4 — Figure-8 (Chenciner & Montgomery, 2000)
 * Three equal masses chasing each other along a single figure-8 curve.
 * An exact periodic solution of the 3-body problem, discovered
 * numerically by Moore (1993) and proven to exist by Chenciner and
 * Montgomery (Annals of Mathematics, 2000).
 *
 * Natural units: G=1, m=1.  Rescaled to pixel space:
 *   pos_scale  = F8_SCALE
 *   time_scale = sqrt(F8_SCALE³ / G_CONST)
 *   vel_scale  = F8_SCALE / time_scale
 *
 * Published ICs:
 *   q1 = (-0.97000436,  0.24308753),  q3 = ( 0.97000436, -0.24308753)
 *   v1 = v3 = ( 0.46620369,  0.43236573)
 *   v2 = (-0.93240737, -0.86473146)
 */
static void preset_figure8(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;

  /* natural-unit -> pixel-unit conversion factors */
  float L = F8_SCALE;
  float ts = sqrtf(L * L * L / G_CONST); /* sec per natural-time unit */
  float vs = L / ts;                     /* px/s per natural-velocity */

  /* published ICs (Chenciner-Montgomery 2000) */
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

/*
 * Preset 5 — Pythagorean Three-Body (Burrau, 1913)
 * Masses 3, 4, 5 placed at rest at the corners of a 3-4-5 right
 * triangle.  Famously chaotic: the trio undergoes close encounters,
 * slingshots, and (in the canonical numerical integration by Szebehely
 * & Peters, 1967) one body is eventually ejected to infinity.
 *
 * Burrau ICs (vertex opposite m_k has length k):
 *   m=3 at ( 1,  3),  m=4 at (-2, -1),  m=5 at ( 1, -1)
 */
static void preset_pythagorean(NBody *nb, int cols, int rows) {
  nb->n = 3;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  /* S = pixels per natural length unit; smaller -> faster dynamics */
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

/*
 * Preset 6 — Black Hole
 * Central mass=100 at screen centre + 20 orbiting light bodies.
 *
 *   v = sqrt(G·M_central / r) * 0.98
 *   0.98 ≈ near-circular: a slight under-circular factor keeps orbits
 *   from being perfectly periodic so precession is visible.
 */
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
    float v = kepler_circular_speed(M_bh, r) * 0.98f; /* see comment */
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang),
             -v * sinf(ang), v * cosf(ang), /* CCW tangent */
             1.0f, ((i - 1) % (N_COLORS - 1)) + 1);
  }

  nbody_forces(nb->bodies, nb->n);
}

/*
 * Preset 7 — Galaxy
 * 20 random bodies (mass 1–4) placed in a disc, each given near-
 * circular velocity around the system's centre of mass.  CoM
 * velocity is then subtracted so the cluster as a whole stays put.
 *
 *   v = sqrt(G·M_total / r) * 0.75   (under-circularised for
 *                                     mildly elliptical, livelier orbits)
 */
static void preset_galaxy(NBody *nb, int cols, int rows) {
  nb->n = 20;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float R = screen_min_dim(cols, rows) * 0.30f;

  /* place bodies in a sqrt-warped disc (uniform AREA density) */
  for (int i = 0; i < nb->n; i++) {
    float r = R * sqrtf(random01());
    float ang = 2.0f * (float)M_PI * random01();
    body_set(&nb->bodies[i], cx + r * cosf(ang), cy + r * sinf(ang), 0,
             0, /* v set below */
             1.0f + 3.0f * random01(), (i % N_COLORS) + 1);
  }

  /* total mass used as the "central" mass for orbital speed estimate */
  float M_total = 0;
  for (int i = 0; i < nb->n; i++)
    M_total += nb->bodies[i].mass;

  /* under-circular tangential velocity → mildly elliptical orbits */
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

  subtract_com_velocity(nb); /* transform to CoM frame */
  nbody_forces(nb->bodies, nb->n);
}

/*
 * Preset 8 — Binary Black Holes
 * Two heavy masses orbiting each other + 20 light test particles in a
 * surrounding disc.  The time-varying binary potential drives the test
 * particles through complex resonances and frequent slingshot ejections.
 *
 *   binary velocity from preset 1 formula
 *   test-particle orbits assume combined point mass at centre
 */
static void preset_binary_bh(NBody *nb, int cols, int rows) {
  nb->n = 22;
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float Rbh = screen_min_dim(cols, rows) * 0.08f;
  float Rmax = screen_min_dim(cols, rows) * 0.34f;
  float M_bh = 50.0f;
  float v_bh = 0.5f * kepler_circular_speed(M_bh, Rbh); /* binary speed */

  /* the two black holes orbit each other */
  body_set(&nb->bodies[0], cx - Rbh, cy, 0, -v_bh, M_bh, 3);
  body_set(&nb->bodies[1], cx + Rbh, cy, 0, v_bh, M_bh, 3);

  /* test particles in a distant disc around the combined point mass */
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

/*
 * Preset 9 — Galaxy Merger
 * Two compact clusters on collision course.  Each cluster = a heavy
 * central body + 12 orbiters.  Tidal forces strip stars from both as
 * they approach, leaving the long spiral tails seen in real merger
 * observations (e.g. the Antennae galaxies NGC 4038/4039).
 */
static void preset_merger(NBody *nb, int cols, int rows) {
  nb->n = 26;
  float W = (float)pw(cols);
  float H = (float)ph(rows);
  float D = screen_min_dim(cols, rows) * 0.30f; /* half-separation */
  float R = screen_min_dim(cols, rows) * 0.10f; /* cluster radius  */
  float Mc = 50.0f;
  float v_app = 0.4f * kepler_circular_speed(Mc, D); /* approach speed  */

  /* Two symmetric clusters approaching each other along the x-axis */
  build_cluster(nb, 0, 12, W * 0.5f - D, H * 0.5f, v_app, 0, Mc, R, 3, 1);
  build_cluster(nb, 13, 12, W * 0.5f + D, H * 0.5f, -v_app, 0, Mc, R, 3, 5);

  nbody_forces(nb->bodies, nb->n);
}

/* ── Dispatch table ─────────────────────────────────────────────────── *
 * Display name + factory pointer, both indexed by the preset number.  */

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
    /* record trail every sub-step */
    for (int i = 0; i < nb->n; i++) {
      if (nb->bodies[i].active)
        body_trail_push(&nb->bodies[i], nb->bodies[i].x, nb->bodies[i].y);
    }
  }
}

/*
 * nbody_count_active() -- number of bodies that have not been ejected.
 * Used by the top-HUD status row.
 */
static int nbody_count_active(const NBody *nb) {
  int n = 0;
  for (int i = 0; i < nb->n; i++)
    if (nb->bodies[i].active)
      n++;
  return n;
}

/* ── nbody_draw decomposition: trail and glyph rendering ─────────────── */

/*
 * glyph_for_mass() -- pick an ASCII glyph whose ink density tracks
 * the body's mass.  Order follows the Bourke density ramp tradition:
 *
 *   '@'  m > 50    black-hole / supergiant (densest)
 *   'O'  m >  3    medium star
 *   'o'  m >  1.5  light star
 *   '*'  otherwise dust / test particle
 */
static char glyph_for_mass(float mass) {
  if (mass > 50.0f)
    return '@';
  if (mass > 3.0f)
    return 'O';
  if (mass > 1.5f)
    return 'o';
  return '*';
}

/*
 * cell_in_scene_band() -- viewport clip predicate.  Excludes the top
 * and bottom HUD strips so motion never overwrites the UI.
 */
static bool cell_in_scene_band(int cx, int cy, int cols, int y_min, int y_max) {
  return cx >= 0 && cx < cols && cy >= y_min && cy < y_max;
}

/*
 * paint_body_trail() -- draw the orbital history of one body as '.'
 * marks.  Walks the ring buffer newest -> oldest.  Age fade: the
 * front 40% of the trail is bold; the back 60% is dim, so the curve
 * dims behind the body and brightens in front of it.
 */
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

/*
 * paint_body_glyph() -- draw the body itself at its current cell
 * position.  Glyph picked by mass; identity color + A_BOLD makes the
 * body pop against its own dim trail.
 */
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

/*
 * nbody_draw() -- render every active body's trail (optional) and
 * glyph into the scene band between the top and bottom HUDs.
 */
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

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene -- the simulation's persistent state, lifespan = whole program.
 *
 * INTENT: single source of truth for "what the simulation is doing
 * right now."  Owned by App; snapshotted whenever 'r' restarts a
 * preset or 'n'/'p' switches to a new one.
 *
 * LOCALITY PRINCIPLE -- where simulation vs. rendering state lives
 * --------------------------------------------------------------
 * The codebase runs two clocks at different rates (sim at sim_fps,
 * render at ~60 Hz).  Cleanly separating sim from render state is
 * what lets the render side reflow on SIGWINCH without disturbing
 * the integrator, and lets the integrator step at any rate without
 * needing to know what the screen looks like.
 *
 *   SIMULATION state (persistent, lives HERE):
 *     nbody         all bodies + sim-level flags
 *                   (paused, show_trails, preset, n).
 *     cols, rows    LAST KNOWN terminal size at the most recent tick.
 *                   Cached here so preset_*() factories and
 *                   nbody_step() can place bodies / check ejection
 *                   against the actual screen without taking a
 *                   Screen* parameter.  Keeps the preset functions
 *                   decoupled from the §7 ncurses layer.  Refreshed
 *                   each scene_tick() from App-supplied dims --
 *                   that's the only sync point with the render side.
 *
 *   RENDER state (ephemeral, lives ELSEWHERE):
 *     Screen.cols / Screen.rows
 *                   live ncurses size in §7.  Refreshed only on
 *                   SIGWINCH; Scene's mirror lags by at most one tick.
 *     glyph / colour picking
 *                   pure functions of mass and color, recomputed per
 *                   frame by glyph_for_mass() / paint_body_glyph().
 *                   Nothing to cache.
 *     trail-age dim cutoff
 *                   computed per frame inside paint_body_trail() from
 *                   the ring-buffer offset.  Stateless.
 *     CELL_W, CELL_H, EJECT_FACTOR, TRAIL_LEN, TOP_HUD_ROWS, ...
 *                   constants in §1 -- configuration, not state.
 *     HUD text strings (fps, sim hz, body count, paused indicator)
 *                   formatted per frame by render_top_hud().
 *                   Stateless.
 *
 *   APP-CONTROL state (lives in App, §8):
 *     sim_fps, running, need_resize -- loop driver, neither sim nor
 *     render.  Mutated by the keyboard handler and by signals.
 *
 * Rule of thumb: if a preset_*() factory or nbody_tick() READS a
 * value, it goes here.  If only nbody_draw() / render_top_hud() /
 * render_bottom_hud() read it, it lives in Screen or is recomputed.
 * That is why Scene is tiny: almost everything the renderer needs is
 * a pure function of body state.
 */
typedef struct {
  NBody nbody;    /* particle system + sim flags */
  int cols, rows; /* mirror of last-known terminal size */
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

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal dimensions.
 *
 * INTENT: hold the live ncurses dims so render functions don't have
 * to call getmaxyx() per frame.  Refreshed on boot in screen_init()
 * and on every SIGWINCH in screen_resize().
 *
 * Pure render-side state -- the simulation never reads it directly
 * (Scene keeps its own mirror, updated each scene_tick()).  When the
 * terminal is resized the bodies keep their pixel positions and
 * velocities unchanged; only HUD layout and the top/bottom-strip
 * clipping in nbody_draw() reflow against the new dims.  This is
 * the locality principle from Scene's comment in practice: sim
 * untouched, render refreshed.
 *
 *   cols    last-known terminal column count  (ncurses COLS)
 *   rows    last-known terminal row count     (ncurses LINES)
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

/*
 * render_top_hud() -- 2-row status zone at the top of the screen.
 *   Row 0: preset title (left, orange) + fps / sim Hz (right, yellow)
 *   Row 1: bodies count, trails state, paused/running indicator
 */
static void render_top_hud(const NBody *nb, int cols, double fps, int sim_fps) {
  /* -- Row 0: title left, fps/sim right ----------------------------- */
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

  /* -- Row 1: bodies / trails / paused-state ------------------------ */
  int active = nbody_count_active(nb);
  char status[64];
  int status_len = snprintf(status, sizeof status, " bodies: %2d   trails: %s ",
                            active, nb->show_trails ? "on " : "off");
  attron(COLOR_PAIR(3) | A_BOLD);
  mvprintw(1, 1, "%s", status);
  attroff(COLOR_PAIR(3) | A_BOLD);

  /* paused indicator: bright red when paused, green when running */
  int cp_state = nb->paused ? 1 : 4;
  const char *state = nb->paused ? "PAUSED " : "running";
  attron(COLOR_PAIR(cp_state) | A_BOLD);
  mvprintw(1, 1 + status_len + 1, " %s ", state);
  attroff(COLOR_PAIR(cp_state) | A_BOLD);
}

/*
 * render_bottom_hud() -- 1-row action zone at the bottom of the screen.
 * Lists every interactive key.  Bright cyan + A_BOLD per project HUD
 * standard (never A_DIM -- the row must stay legible over any animation).
 */
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

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App -- top-level lifecycle bundle, lives in g_app for the program's
 * entire lifetime.
 *
 * INTENT: composition root.  App OWNS the Scene (sim) and the Screen
 * (cached render dims), and carries the loop-driver fields
 * (sim_fps, running, need_resize) that don't belong to either
 * sub-component.
 *
 * Why a single g_app global and not pass-by-pointer everywhere:
 * signal handlers (SIGINT, SIGTERM, SIGWINCH) cannot accept user
 * pointers.  Centralising the mutable handoff in one named global
 * keeps the signal contract narrow and explicit -- handlers touch
 * exactly two flags (running, need_resize), nothing more.
 *
 *   scene         simulation state -- see Scene.
 *   screen        cached render dims -- see Screen.
 *   sim_fps       solver ticks per second; SIM_FPS_MIN..SIM_FPS_MAX,
 *                 stepped by SIM_FPS_STEP via '[' / ']'.  Independent
 *                 of the ~60 Hz render rate so you can crank physics
 *                 up or slow it down without disturbing the HUD fps
 *                 readout.
 *   running       Main-loop sentinel.  Cleared by SIGINT / SIGTERM
 *                 handlers and by 'q' / ESC.  Declared
 *                 `volatile sig_atomic_t` because a signal handler
 *                 writes it asynchronously while the main loop reads
 *                 it -- the only portable way to share a flag across
 *                 that boundary.
 *   need_resize   SIGWINCH flag.  Set by on_resize_signal(); drained
 *                 next iteration by app_do_resize(), which refreshes
 *                 Screen.cols/rows from ncurses.  Same volatile
 *                 sig_atomic_t rationale.
 *
 * Reference:
 *   POSIX.1-2017, §2.4 Signal Concepts.  sig_atomic_t is the standard
 *   idiom for cross-handler flags; volatile prevents the compiler
 *   from caching the value across the main-loop body.
 */
typedef struct {
  Scene scene;                       /* simulation state */
  Screen screen;                     /* cached render dims */
  int sim_fps;                       /* solver ticks per second */
  volatile sig_atomic_t running;     /* 0 = exit main loop */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending */
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
