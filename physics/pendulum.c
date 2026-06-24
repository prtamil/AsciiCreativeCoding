/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pendulum.c — a chain of 1 to 5 swinging rods that hangs from a fixed
 * point and swings under gravity. One rod just rocks back and forth;
 * two or more rods go chaotic — tiny differences in the start blow up
 * fast. A dim "ghost" chain starts a hair's-breadth off the main one to
 * show that blow-up happen live.
 *
 * The motion comes from the standard physics of a hanging chain of rods
 * (Lagrangian mechanics). For the double pendulum see Shinbrot et al.,
 * "Chaos in a double pendulum", Am. J. Phys. 60(6) (1992); for the
 * chaos background, Strogatz, "Nonlinear Dynamics and Chaos" (2014).
 * Sister file: double_pendulum.c (the fixed N = 2 case).
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
  N_MAX = 5,     /* most rods we allow in the chain          */
  N_DEFAULT = 1, /* start with a single rod                  */
  N_THEMES = 10, /* number of colour palettes (see §3)       */

  /* How many physics steps per second. Higher is more accurate; we run
   * fast (300) because chaos punishes a sloppy integrator — drop too low
   * and the simulated chain drifts off the true path within seconds. */
  SIM_FPS_DEFAULT = 300,
  SIM_FPS_MIN = 60,
  SIM_FPS_MAX = 600,
  SIM_FPS_STEP = 60,

  TRAIL_LEN = 500, /* how many past bob positions we keep      */
  TRAIL_DEF = 360, /* how many of them we draw by default      */
  TRAIL_MIN = 20,
  TRAIL_STEP = 20,

  HUD_COLS = 64,
  FPS_UPDATE_MS = 500, /* how often the fps number refreshes (ms)  */
};

/* How many fine "pixels" fit in one terminal cell. The physics works in
 * these pixels; we only round to whole cells at the moment we draw. */
#define CELL_W 8
#define CELL_H 16

/* How tall the fully-stretched chain is, as a fraction of the screen
 * height, so it never hangs off the bottom. Each rod gets 1/N of this. */
#define MAX_REACH_FRAC 0.44f

/* Strength of gravity, in pixels per second squared. Tuned by eye for a
 * lively swing (same value as double_pendulum.c). */
#define GRAVITY_PX 2000.0f

/* The angle every rod starts at, measured from straight-down. 120° is
 * well past horizontal — lots of energy, so chaos kicks in within about
 * a second once there are two or more rods. */
#define INIT_THETA_DEG 120.0f

/* How far the ghost's top angle is nudged from the real chain, in
 * radians. 0.001 rad is about 0.057° — a difference far too small to see
 * — yet two or more rods will blow it up into total divergence in a few
 * seconds. That's the whole point of the ghost. */
#define GHOST_EPSILON 0.001f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock ── */

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

/* ── §3 color — themes + fixed HUD colours ── */

/*
 * Colour slots. Most follow the active theme; the last three are fixed
 * so the structure and on-screen text stay readable in every palette.
 *   CP_ARM1..CP_ARM5  the rods (each rod a different shade)
 *   CP_JOINT          the joints between rods
 *   CP_BOB            the swinging weight on the end
 *   CP_TR1..CP_TR3    the trail, brightest (newest) to dimmest (oldest)
 *   CP_GHOST          the faint ghost chain
 *   CP_BAR            the [+] mark at the fixed pivot point
 *   CP_HUD            top status line
 *   CP_HINT           bottom key-list line
 */
enum {
  CP_BAR = 1,
  CP_ARM1,
  CP_ARM2,
  CP_ARM3,
  CP_ARM4,
  CP_ARM5,
  CP_JOINT,
  CP_BOB,
  CP_TR1,
  CP_TR2,
  CP_TR3,
  CP_GHOST,
  CP_HUD,
  CP_HINT,
};

/*
 * Theme — one named colour palette for the whole demo.
 *   name    what to show in the HUD ("Matrix", "Fire", ...).
 *   ramp    four shades from dim to bright; the rods, joints, and trail
 *           tiers all pick from here.
 *   accent  the brightest, most eye-catching colour — used for the
 *           end weight and the last rod's cap.
 *   ghost   a faint shade for the barely-there ghost chain.
 *
 * Every colour is picked from the brighter half of the palette so it
 * stays visible against a black background (see CLAUDE.md).
 */
typedef struct {
  const char *name;
  short ramp[4];
  short accent;
  short ghost;
} Theme;

static const Theme k_themes[N_THEMES] = {
    {"Matrix", {28, 34, 40, 46}, 82, 28},        /* cyber green  */
    {"Fire", {130, 208, 202, 196}, 226, 88},     /* warm → red   */
    {"Oceanic", {24, 31, 39, 51}, 195, 24},      /* teal → cyan  */
    {"Neon", {129, 165, 201, 213}, 219, 53},     /* purple → pink*/
    {"Mono", {240, 247, 250, 255}, 255, 240},    /* grayscale    */
    {"Ice", {153, 117, 159, 195}, 231, 24},      /* light blues  */
    {"Nova", {129, 141, 177, 213}, 219, 53},     /* stellar      */
    {"Forest", {58, 100, 142, 190}, 226, 28},    /* leaves/bark  */
    {"Desert", {130, 178, 214, 220}, 226, 94},   /* sand/gold    */
    {"Eclipse", {240, 244, 124, 196}, 226, 240}, /* gray + red   */
};

/* Switch the live colours to theme t. The last three slots (pivot mark
 * and the two HUD lines) are set the same way every time so the text
 * never gets lost in a dark palette. */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_ARM1, th->ramp[0], COLOR_BLACK);
    init_pair(CP_ARM2, th->ramp[1], COLOR_BLACK);
    init_pair(CP_ARM3, th->ramp[2], COLOR_BLACK);
    init_pair(CP_ARM4, th->ramp[3], COLOR_BLACK);
    init_pair(CP_ARM5, th->accent, COLOR_BLACK);
    init_pair(CP_JOINT, th->ramp[2], COLOR_BLACK);
    init_pair(CP_BOB, th->accent, COLOR_BLACK);
    init_pair(CP_TR1, th->ramp[3], COLOR_BLACK);
    init_pair(CP_TR2, th->ramp[2], COLOR_BLACK);
    init_pair(CP_TR3, th->ramp[1], COLOR_BLACK);
    init_pair(CP_GHOST, th->ghost, COLOR_BLACK);
    /* fixed chrome */
    init_pair(CP_BAR, 231, COLOR_BLACK); /* bright white  */
    init_pair(CP_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(CP_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    /* 8-colour fallback — theme-independent. */
    init_pair(CP_ARM1, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_ARM2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_ARM3, COLOR_GREEN, COLOR_BLACK);
    init_pair(CP_ARM4, COLOR_RED, COLOR_BLACK);
    init_pair(CP_ARM5, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(CP_JOINT, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_BOB, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_TR1, COLOR_RED, COLOR_BLACK);
    init_pair(CP_TR2, COLOR_RED, COLOR_BLACK);
    init_pair(CP_TR3, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_GHOST, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(CP_BAR, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

static void color_init(void) {
  start_color();
  theme_apply(0);
}

/* Which colour slot the n-th rod uses (counting from 0). */
static int arm_pair(int link) {
  static const int arms[N_MAX] = {CP_ARM1, CP_ARM2, CP_ARM3, CP_ARM4, CP_ARM5};
  if (link < 0)
    return CP_ARM1;
  if (link >= N_MAX)
    return CP_ARM5;
  return arms[link];
}

/* ── §4 coords ── */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 physics ── */

/*
 * StateN — a complete snapshot of where the chain is and how fast it's
 * moving, at one instant. That's all the physics needs to step forward.
 *   th[i]   the angle of rod i, in radians, measured from straight-down.
 *   om[i]   how fast that angle is changing, in radians per second.
 * Two flat arrays (rather than an array of pairs) so the integrator can
 * add and scale them rod-by-rod with simple loops. Sized for the most
 * rods we ever allow, so no chain ever needs to allocate memory; only
 * the first n entries are live, where n is the chain's rod count.
 * Entry 0 is the top rod (at the pivot); entry n-1 is the bottom rod
 * carrying the end weight.
 */
typedef struct {
  float th[N_MAX]; /* angle of each rod, radians, 0 = straight down  */
  float om[N_MAX]; /* how fast each angle changes, radians/second    */
} StateN;

/* Take a trial step forward from where the chain is, following the slope
 * k. The stepper below uses this to build its in-between probes. */
static void state_step_n(int n, const StateN *s, float dt, const StateN *k,
                         StateN *out) {
  for (int i = 0; i < n; i++) {
    out->th[i] = s->th[i] + dt * k->th[i];
    out->om[i] = s->om[i] + dt * k->om[i];
  }
}

/* Pick the best row to work with next: the one with the biggest number
 * in column i. Using the biggest keeps the arithmetic from blowing up. */
static int find_pivot_row(int n, float M[N_MAX][N_MAX], int i) {
  int piv = i;
  float maxv = fabsf(M[i][i]);
  for (int r = i + 1; r < n; r++) {
    if (fabsf(M[r][i]) > maxv) {
      maxv = fabsf(M[r][i]);
      piv = r;
    }
  }
  return piv;
}

/* Swap two equations. The right-hand side b must swap along with M, or
 * we'd be solving a different problem. */
static void swap_rows_augmented(int n, float M[N_MAX][N_MAX], float b[N_MAX],
                                int i, int j) {
  for (int c = 0; c < n; c++) {
    float t = M[i][c];
    M[i][c] = M[j][c];
    M[j][c] = t;
  }
  float t = b[i];
  b[i] = b[j];
  b[j] = t;
}

/* Subtract the right multiple of equation i from each equation below it
 * so that column i becomes zero there — one step of working the system
 * down to a staircase shape we can read off bottom-up. */
static void eliminate_below_pivot(int n, float M[N_MAX][N_MAX], float b[N_MAX],
                                  int i) {
  for (int r = i + 1; r < n; r++) {
    float f = M[r][i] / M[i][i];
    for (int c = i; c < n; c++)
      M[r][c] -= f * M[i][c];
    b[r] -= f * b[i];
  }
}

/*
 * Second half of the solver. By now the equations form a staircase: the
 * last one has a single unknown, so solve it, then plug that answer into
 * the one above, and walk up to the top.
 */
static void back_substitute(int n, float M[N_MAX][N_MAX], float b[N_MAX],
                            float x[N_MAX]) {
  for (int i = n - 1; i >= 0; i--) {
    float s = b[i];
    for (int j = i + 1; j < n; j++)
      s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
}

/*
 * Solves a small set of linear equations M·x = b for the unknowns x —
 * here, the rods' accelerations. The plan is the one from school: clear
 * out variables one column at a time until the equations form a
 * staircase, then read the answers back off bottom-up. Picking the
 * biggest row to work with each step (see find_pivot_row) keeps the
 * arithmetic well-behaved. Reference: Golub & Van Loan §3.4; Press et
 * al. "Numerical Recipes" §2.2.
 */
static void solve_linear(int n, float M[N_MAX][N_MAX], float b[N_MAX],
                         float x[N_MAX]) {
  /* work the equations down into staircase shape */
  for (int i = 0; i < n; i++) {
    int piv = find_pivot_row(n, M, i);
    if (piv != i)
      swap_rows_augmented(n, M, b, i, piv);
    eliminate_below_pivot(n, M, b, i);
  }
  /* read the answers back off, bottom-up */
  back_substitute(n, M, b, x);
}

/*
 * The easy half of the motion. We track each rod's angle and its turning
 * speed; how fast an angle is changing is just its turning speed, so this
 * is a plain copy. The hard half (how the speeds change) is solved below.
 */
static void copy_velocities_to_thetadot(int n, const StateN *s, StateN *out) {
  for (int i = 0; i < n; i++)
    out->th[i] = s->om[i];
}

/*
 * Builds the grid of numbers that says how much each rod's motion drags
 * on every other rod's — heavy rods near the top pull on more of the
 * chain, and rods pointing the same way push and pull together more. The
 * rest of the physics is solving against this grid. Reference: Goldstein
 * 2001 §1.6 (the same "mass matrix" idea used for any chain of rods).
 */
static void assemble_mass_matrix(int n, const StateN *s, float L,
                                 float M[N_MAX][N_MAX]) {
  for (int j = 0; j < n; j++) {
    for (int k = 0; k < n; k++) {
      int mx = (j > k) ? j : k;
      float coef = (float)(n - mx) * L * L;
      float th_jk = s->th[j] - s->th[k];
      M[j][k] = coef * cosf(th_jk);
    }
  }
}

/*
 * Builds the list of pushes acting on each rod right now. Two things push:
 * gravity pulling each rod back toward straight-down, and the fling from
 * the other rods already whipping around (the force you feel on a fast
 * merry-go-round). Together with the drag grid above, these decide how the
 * turning speeds change this instant.
 */
static void assemble_force_vector(int n, const StateN *s, float L, float g,
                                  float b[N_MAX]) {
  for (int j = 0; j < n; j++) {
    b[j] = 0.0f;
    for (int k = 0; k < n; k++) {
      int mx = (j > k) ? j : k;
      float coef = (float)(n - mx) * L * L;
      float th_jk = s->th[j] - s->th[k];
      b[j] -= coef * sinf(th_jk) * s->om[k] * s->om[k];
    }
    b[j] -= g * L * (float)(n - j) * sinf(s->th[j]);
  }
}

/*
 * The heart of the physics: given where the chain is and how fast it's
 * turning, work out how all of that is changing right now. The angles
 * change at the turning speeds (easy). The turning speeds change by an
 * amount we have to solve for — build the drag grid and the push list,
 * then solve one set of equations to get every rod's acceleration at once.
 * Reference: Goldstein 2001 §1.6, Shinbrot et al. 1992.
 */
static void deriv_n(int n, const StateN *s, float L, float g, StateN *out) {
  copy_velocities_to_thetadot(n, s, out);

  float M[N_MAX][N_MAX];
  float b[N_MAX];
  assemble_mass_matrix(n, s, L, M);
  assemble_force_vector(n, s, L, g, b);

  /* solve drag · acceleration = push; the answers fill out->om */
  solve_linear(n, M, b, out->om);
}

/*
 * Take a trial hop forward using the last slope we measured, then measure
 * the slope again there. The stepper below uses this to peek ahead a
 * little before committing to the real step.
 */
static void rk4_slope_at(int n, const StateN *s, float dt_frac,
                         const StateN *prev_slope, float L, float g,
                         StateN *out_slope, StateN *scratch) {
  state_step_n(n, s, dt_frac, prev_slope, scratch);
  deriv_n(n, scratch, L, g, out_slope);
}

/*
 * Take the four slopes we measured across the step and blend them into one
 * best-guess slope, trusting the two middle measurements twice as much,
 * then actually move the chain forward by it. Averaging four probes
 * instead of trusting the first is what makes the step accurate.
 */
static void rk4_combine_simpson(int n, StateN *s, float dt, const StateN *k1,
                                const StateN *k2, const StateN *k3,
                                const StateN *k4) {
  float w = dt / 6.0f;
  for (int i = 0; i < n; i++) {
    s->th[i] +=
        w * (k1->th[i] + 2.0f * k2->th[i] + 2.0f * k3->th[i] + k4->th[i]);
    s->om[i] +=
        w * (k1->om[i] + 2.0f * k2->om[i] + 2.0f * k3->om[i] + k4->om[i]);
  }
}

/*
 * Move the chain forward by one small time step, accurately. Instead of
 * trusting a single slope, it probes four: at the start, twice in the
 * middle, and at the end, each guided by the one before, then blends them.
 * The accuracy matters here — a chaotic chain drifts off the true path
 * fast if each step is sloppy. This is the classic Runge-Kutta method;
 * see Hairer-Nørsett-Wanner 1993.
 */
static void rk4_step_n(int n, StateN *s, float L, float g, float dt) {
  /* Zero them out: we only fill the first n slots, but gcc can't tell and
   * would warn about the rest looking uninitialised.                     */
  StateN k1, k2, k3, k4, tmp;
  memset(&k1, 0, sizeof k1);
  memset(&k2, 0, sizeof k2);
  memset(&k3, 0, sizeof k3);
  memset(&k4, 0, sizeof k4);
  memset(&tmp, 0, sizeof tmp);

  deriv_n(n, s, L, g, &k1);                            /* slope at start    */
  rk4_slope_at(n, s, 0.5f * dt, &k1, L, g, &k2, &tmp); /* slope at midpoint */
  rk4_slope_at(n, s, 0.5f * dt, &k2, L, g, &k3, &tmp); /* midpoint, refined */
  rk4_slope_at(n, s, dt, &k3, L, g, &k4, &tmp);        /* slope at the end  */

  rk4_combine_simpson(n, s, dt, &k1, &k2, &k3, &k4);
}

/*
 * NPend — one whole pendulum chain: where it is, how it's shaped, and
 * how many rods it has. Two of these live in the Scene — the bright one
 * you watch, and a faint "ghost" started a tiny bit off it so you can
 * see chaos pull them apart.
 *
 * The fields fall into three groups:
 *
 *   What's moving:
 *     s      where the chain is right now (every rod's angle and how
 *            fast it's turning).
 *     prev   where it was at the start of this physics step. We keep
 *            both so the drawing can blend between them — that lets the
 *            motion look smooth even when we draw far less often than we
 *            do the physics (the "fixed timestep + interpolation" trick,
 *            Fiedler 2004).
 *
 *   Fixed shape (set once when the chain is built, never changes after):
 *     pivot_px, pivot_py   where the top of the chain is pinned.
 *     arm_len              how long each rod is. Every rod is the same
 *                          length, sized so the whole chain reaches the
 *                          same fraction of the screen no matter how
 *                          many rods there are.
 *
 *   How many rods:
 *     n      the rod count, 1..N_MAX. Says how many slots of s and prev
 *            are actually in use. Changing it rebuilds the chain.
 */
typedef struct {
  /* ── what's moving (changes every tick) ── */
  StateN s;    /* where the chain is right now              */
  StateN prev; /* where it was at the start of this step;
                  the drawing blends prev→s for smoothness  */

  /* ── fixed shape (set once when built) ── */
  float pivot_px; /* x of the pinned top, in pixel units       */
  float pivot_py; /* y of the pinned top, in pixel units       */
  float arm_len;  /* length of each rod; all equal so the
                     chain's total reach is screen-relative    */

  /* ── how many rods ── */
  int n; /* rod count in use, 1..N_MAX                */
} NPend;

/* Pin the top of the chain at the middle of the screen, so it has room to
 * swing both above and below. */
static void place_pivot_at_screen_center(NPend *p, int cols, int rows) {
  p->pivot_px = (float)pw(cols) * 0.5f;
  p->pivot_py = (float)ph(rows) * 0.5f;
}

/*
 * Pick the rod length so the whole chain reaches the same way down the
 * screen no matter how many rods it has — split the fixed total reach
 * evenly. One rod is long; five rods are short but reach just as far.
 */
static void size_arms_for_n_links(NPend *p, int n_links, int rows) {
  p->arm_len = (float)ph(rows) * MAX_REACH_FRAC / (float)n_links;
}

/*
 * Set every rod to the same starting angle and at rest. The chain begins
 * stretched out and lifted well past horizontal — lots of stored energy,
 * so with two or more rods it goes chaotic within about a second.
 */
static void set_uniform_initial_angles(NPend *p, int n_links, float theta_deg) {
  float theta_rad = (float)(theta_deg * M_PI / 180.0);
  for (int i = 0; i < n_links; i++) {
    p->s.th[i] = theta_rad;
    p->s.om[i] = 0.0f;
  }
}

/*
 * Nudge just the top rod's angle by a tiny amount. This is how we make the
 * ghost: a difference far too small to see (~0.057°) that, with two or
 * more rods, grows into total divergence in a few seconds — chaos made
 * visible. The real chain passes 0 here, so it's left untouched.
 */
static void perturb_first_angle(NPend *p, float extra_deg) {
  p->s.th[0] += (float)(extra_deg * M_PI / 180.0);
}

/*
 * Build a fresh chain in its starting pose. th_extra_deg is the little
 * nudge for the ghost (the real chain passes 0). The last step copies the
 * current pose into prev so the very first frame has a "before" to blend
 * from.
 */
static void npend_init(NPend *p, int n_links, int cols, int rows,
                       float th_extra_deg) {
  p->n = n_links;
  place_pivot_at_screen_center(p, cols, rows);
  size_arms_for_n_links(p, n_links, rows);
  set_uniform_initial_angles(p, n_links, INIT_THETA_DEG);
  perturb_first_angle(p, th_extra_deg);
  p->prev = p->s;
}

static void npend_tick(NPend *p, float dt) {
  p->prev = p->s;
  rk4_step_n(p->n, &p->s, p->arm_len, GRAVITY_PX, dt);
}

/*
 * Turn the chain's angles into actual on-screen points. Start at the
 * pinned top and follow each rod in turn — its angle says which way it
 * points, so step that rod's length in that direction to reach the next
 * joint. The very last point is the swinging weight.
 */
static void npend_positions(const NPend *p, const float *th, float *xs,
                            float *ys) {
  xs[0] = p->pivot_px;
  ys[0] = p->pivot_py;
  for (int i = 0; i < p->n; i++) {
    xs[i + 1] = xs[i] + p->arm_len * sinf(th[i]);
    ys[i + 1] = ys[i] + p->arm_len * cosf(th[i]);
  }
}

/* ── §6 scene ── */

/*
 * Trail — the breadcrumb trail the swinging weight leaves behind. We
 * remember its last few hundred positions and draw them as a fading
 * tail, which is what makes the looping, never-repeating path visible.
 *
 * It's a ring buffer: a fixed-size circular list. Every tick we drop one
 * new position in and the oldest one quietly falls off the back, so we
 * never shift anything or allocate memory — just overwrite in a loop.
 *
 *   px, py   the remembered positions, in fine "pixel" units
 *   head     where the next position will be written
 *   count    how many positions we've stored so far; stops growing once
 *            the buffer is full
 *
 * To walk the trail oldest-first, start at (head − count) wrapped around
 * and step forward, wrapping at the end. How many of these we actually
 * paint each frame is a separate user-adjustable number (see trail_draw
 * in Scene), so we can keep a long memory but show a short tail.
 */
typedef struct {
  float px[TRAIL_LEN]; /* remembered x position (pixel units)       */
  float py[TRAIL_LEN]; /* remembered y position (pixel units)       */
  int head;            /* where the next position goes (wraps round) */
  int count;           /* how many we've stored, up to TRAIL_LEN     */
} Trail;

static void trail_push(Trail *t, float px, float py) {
  t->px[t->head] = px;
  t->py[t->head] = py;
  t->head = (t->head + 1) % TRAIL_LEN;
  if (t->count < TRAIL_LEN)
    t->count++;
}

static void trail_clear(Trail *t) {
  t->head = 0;
  t->count = 0;
}

/*
 * Scene — everything the demo needs to run, in one bundle. Pretty much
 * every function takes a Scene* and works on it. The only state living
 * outside is the two signal flags in §8, which have to be globals
 * because C's signal handlers can't be handed a pointer.
 *
 * The fields split into two camps: the ones the physics touches, and the
 * ones only the drawing touches.
 *
 *   The physics side:
 *     primary, ghost  Two identical chains. The ghost starts a hair off
 *                     the real one (one top angle nudged by ~0.057°).
 *                     Both step forward every tick. With two or more
 *                     rods the gap between them explodes — that's the
 *                     live proof of chaos, shown in the HUD as "div:".
 *                     With one rod they stay locked together; a single
 *                     pendulum isn't chaotic.
 *     n_links         How many rods, shared by both chains. Changing it
 *                     rebuilds the scene from scratch (the n/p keys).
 *     paused          When true the physics simply doesn't step. Drawing
 *                     still happens, so the chain freezes on screen.
 *
 *   The drawing side (none of this feeds back into the physics):
 *     trail           The weight's breadcrumb trail. Filled in once per
 *                     tick, read every frame when we paint the tail.
 *     cols, rows      Terminal size in cells, refreshed after a resize.
 *     theme           Which colour palette is active (t/T to cycle).
 *     show_ghost      Whether to draw the ghost chain at all (g).
 *     show_trail      Whether to draw the trail at all (l).
 *     trail_draw      How much of the stored trail to actually paint
 *                     (+/- to lengthen or shorten the visible tail).
 */
typedef struct {
  /* ── physics side ── */
  NPend primary; /* the chain shown in full colour            */
  NPend ghost;   /* twin chain, top angle nudged a hair; the
                    faint one that proves chaos               */
  int n_links;   /* rod count, shared by both, 1..N_MAX       */
  bool paused;   /* spc: when true the physics stops stepping */

  /* ── drawing side (never affects the physics) ── */
  Trail trail;     /* the weight's breadcrumb trail             */
  int cols, rows;  /* terminal size in cells (after a resize)   */
  int theme;       /* t/T: which colour palette is active       */
  bool show_ghost; /* g  : draw the faint ghost chain?          */
  bool show_trail; /* l  : draw the breadcrumb trail?           */
  int trail_draw;  /* +/-: how much of the trail to paint       */
} Scene;

static void scene_init(Scene *s, int n_links, int cols, int rows) {
  if (n_links < 1)
    n_links = 1;
  if (n_links > N_MAX)
    n_links = N_MAX;

  s->cols = cols;
  s->rows = rows;
  s->n_links = n_links;
  s->theme = 0;
  s->paused = false;
  s->show_ghost = true;
  s->show_trail = true;
  s->trail_draw = TRAIL_DEF;

  npend_init(&s->primary, n_links, cols, rows, 0.0f);
  /* Ghost: same setup but θ₁ shifted by GHOST_EPSILON rad. */
  npend_init(&s->ghost, n_links, cols, rows,
             (float)(GHOST_EPSILON * 180.0 / M_PI));

  trail_clear(&s->trail);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  npend_tick(&s->primary, dt);
  npend_tick(&s->ghost, dt);

  /* Record the end-bob position into the trail. */
  float xs[N_MAX + 1], ys[N_MAX + 1];
  npend_positions(&s->primary, s->primary.s.th, xs, ys);
  trail_push(&s->trail, xs[s->n_links], ys[s->n_links]);
}

/* ── draw helpers ── */

/*
 * Draw a straight line between two cells, picking the character that best
 * matches the line's slope (-, |, / or \). Same routine as in
 * double_pendulum.c.
 */
static void draw_line(int x0, int y0, int x1, int y1, int cols, int rows,
                      attr_t attr) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;) {
    if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows) {
      int e2 = 2 * err;
      bool step_x = (e2 > -dy);
      bool step_y = (e2 < dx);
      chtype ch;
      if (step_x && step_y)
        ch = (sx == sy) ? '\\' : '/';
      else if (step_x)
        ch = '-';
      else
        ch = '|';
      attron(attr);
      mvaddch(y0, x0, ch);
      attroff(attr);
    }
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

/* Blend last tick's angles toward this tick's by alpha, so the on-screen
 * motion stays smooth even when we draw less often than we step. */
static void interp_angles(const NPend *p, float alpha, float *out) {
  for (int i = 0; i < p->n; i++)
    out[i] = p->prev.th[i] + (p->s.th[i] - p->prev.th[i]) * alpha;
}

/* Mark the pinned top of the chain with a little [+]. */
static void draw_pivot_marker(const NPend *p, int cols, int rows) {
  int cx = px_to_cell_x(p->pivot_px);
  int cy = px_to_cell_y(p->pivot_py);
  if (cx >= 1 && cx < cols - 1 && cy >= 0 && cy < rows) {
    attron(COLOR_PAIR(CP_BAR) | A_BOLD);
    mvaddch(cy, cx - 1, '[');
    mvaddch(cy, cx, '+');
    mvaddch(cy, cx + 1, ']');
    attroff(COLOR_PAIR(CP_BAR) | A_BOLD);
  }
}

/*
 * Paint the weight's trail as a fading tail. Rather than fade each dot
 * individually, we split the tail into three bands — newest is brightest,
 * oldest is dimmest — which looks like a smooth fade for far less work.
 */
static void draw_trail(const Scene *s) {
  if (!s->show_trail || s->trail.count == 0)
    return;

  int draw = s->trail_draw;
  if (draw > s->trail.count)
    draw = s->trail.count;
  int start = (s->trail.head - draw + TRAIL_LEN) % TRAIL_LEN;

  for (int i = 0; i < draw; i++) {
    int idx = (start + i) % TRAIL_LEN;
    int cx = px_to_cell_x(s->trail.px[idx]);
    int cy = px_to_cell_y(s->trail.py[idx]);
    if (cx < 0 || cx >= s->cols || cy < 0 || cy >= s->rows)
      continue;

    float age = (float)i / (float)draw; /* 0=oldest, 1=newest */
    int cp;
    chtype ch;
    if (age > 0.70f) {
      cp = CP_TR1;
      ch = 'o';
    } else if (age > 0.35f) {
      cp = CP_TR2;
      ch = '.';
    } else {
      cp = CP_TR3;
      ch = ',';
    }

    attron(COLOR_PAIR(cp));
    mvaddch(cy, cx, ch);
    attroff(COLOR_PAIR(cp));
  }
}

/*
 * Work out where to draw the chain this frame: blend last tick's angles
 * toward this tick's for smoothness, then turn those angles into points.
 * Fills the pivot, every joint, and the end weight (the last point).
 */
static void interpolated_joint_positions(const NPend *p, float alpha, float *xs,
                                         float *ys) {
  float th[N_MAX] = {0};
  interp_angles(p, alpha, th);
  npend_positions(p, th, xs, ys);
}

/*
 * Draw the rods as lines between the joints. On the real chain each rod
 * gets its own colour so you can tell them apart; the ghost draws all its
 * rods in one faint colour so it stays in the background.
 */
static void draw_arm_segments(const NPend *p, const float *xs, const float *ys,
                              int cols, int rows, bool is_ghost) {
  for (int i = 0; i < p->n; i++) {
    int cx0 = px_to_cell_x(xs[i]), cy0 = px_to_cell_y(ys[i]);
    int cx1 = px_to_cell_x(xs[i + 1]), cy1 = px_to_cell_y(ys[i + 1]);
    attr_t a = is_ghost ? (attr_t)COLOR_PAIR(CP_GHOST)
                        : (attr_t)(COLOR_PAIR(arm_pair(i)) | A_BOLD);
    draw_line(cx0, cy0, cx1, cy1, cols, rows, a);
  }
}

/*
 * Mark the joints in the middle of the chain with 'O'. The top (a [+])
 * and the end weight (a (@)) are drawn elsewhere, so skip those. The ghost
 * skips these entirely to stay faint.
 */
static void draw_intermediate_joints(const NPend *p, const float *xs,
                                     const float *ys, int cols, int rows) {
  for (int i = 1; i < p->n; i++) {
    int cx = px_to_cell_x(xs[i]);
    int cy = px_to_cell_y(ys[i]);
    if (cx >= 0 && cx < cols && cy > 0 && cy < rows) {
      attron(COLOR_PAIR(CP_JOINT) | A_BOLD);
      mvaddch(cy, cx, 'O');
      attroff(COLOR_PAIR(CP_JOINT) | A_BOLD);
    }
  }
}

/*
 * Draw the swinging weight at the chain's tip. The real one is a bright
 * (@) so your eye follows it; the ghost is just a faint 'x'.
 */
static void draw_end_bob(const NPend *p, const float *xs, const float *ys,
                         int cols, int rows, bool is_ghost) {
  int b_cx = px_to_cell_x(xs[p->n]);
  int b_cy = px_to_cell_y(ys[p->n]);
  if (b_cy <= 0 || b_cy >= rows)
    return;

  if (is_ghost) {
    if (b_cx >= 1 && b_cx < cols - 1) {
      attron(COLOR_PAIR(CP_GHOST));
      mvaddch(b_cy, b_cx, 'x');
      attroff(COLOR_PAIR(CP_GHOST));
    }
    return;
  }
  attron(COLOR_PAIR(CP_BOB) | A_BOLD);
  if (b_cx > 0 && b_cx < cols - 1) {
    mvaddch(b_cy, b_cx - 1, '(');
    mvaddch(b_cy, b_cx, '@');
    mvaddch(b_cy, b_cx + 1, ')');
  } else if (b_cx >= 0 && b_cx < cols) {
    mvaddch(b_cy, b_cx, '@');
  }
  attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
}

/*
 * Draw one whole chain: figure out the points once, then paint the rods,
 * the middle joints, and the end weight from that one snapshot so nothing
 * shifts partway through the frame.
 */
static void draw_chain(const NPend *p, float alpha, int cols, int rows,
                       bool is_ghost) {
  float xs[N_MAX + 1], ys[N_MAX + 1];
  interpolated_joint_positions(p, alpha, xs, ys);

  draw_arm_segments(p, xs, ys, cols, rows, is_ghost);
  if (!is_ghost)
    draw_intermediate_joints(p, xs, ys, cols, rows);
  draw_end_bob(p, xs, ys, cols, rows, is_ghost);
}

/*
 * Draw one full frame, back to front so the important stuff lands on top:
 * pivot mark, then the trail, then the faint ghost, then the real chain
 * over everything.
 */
static void scene_draw(const Scene *s, float alpha) {
  draw_pivot_marker(&s->primary, s->cols, s->rows);
  draw_trail(s);
  if (s->show_ghost)
    draw_chain(&s->ghost, alpha, s->cols, s->rows, /*is_ghost=*/true);
  draw_chain(&s->primary, alpha, s->cols, s->rows, /*is_ghost=*/false);
}

/* ── §7 screen ── */

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

/* Spell out the chain length for the HUD: "single", "double", and so on. */
static const char *pendulum_count_name(int n) {
  static const char *names[N_MAX] = {
      "single", "double", "triple", "quadruple", "quintuple",
  };
  if (n < 1)
    return names[0];
  if (n > N_MAX)
    return names[N_MAX - 1];
  return names[n - 1];
}

/*
 * Top status line: frame rate, chain length, the end weight's angle, how
 * far the ghost has drifted from the real chain, trail length, theme, and
 * whether we're paused.
 */
static void draw_hud_top(Screen *s, const Scene *sc, double fps) {
  const NPend *p = &sc->primary;
  const NPend *g = &sc->ghost;

  /* How far the ghost has drifted: add up the angle gap across all rods. */
  float div_deg = 0.0f;
  for (int i = 0; i < p->n; i++)
    div_deg += fabsf(p->s.th[i] - g->s.th[i]);
  div_deg *= (float)(180.0 / M_PI);
  if (div_deg > 9999.0f)
    div_deg = 9999.0f;

  /* The end weight's angle — the single most telling number to show. */
  float th_end_deg = p->s.th[p->n - 1] * (float)(180.0 / M_PI);

  char buf[220];
  snprintf(buf, sizeof buf,
           " %5.1f fps  %s pendulum (N=%d)  t_end:%+7.1f  div:%6.1f  "
           "tr:%d  theme:%s  %s ",
           fps, pendulum_count_name(p->n), p->n, th_end_deg, div_deg,
           sc->trail_draw, k_themes[sc->theme].name,
           sc->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int col = s->cols - len;
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, s->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * Bottom line: the list of keys. Falls back to a shorter list when the
 * terminal is too narrow to fit them all.
 */
static void draw_hud_bottom(Screen *s) {
  const char *hint_full = " q:quit  spc:pause  r:reset  n/p:N  t/T:theme  "
                          "g:ghost  l:trail  +/-:tail  ]/[:fps ";
  const char *hint_short = " q:quit  spc:pause  n/p:N  t:theme  l:trail ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= s->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(s->rows - 1, 0, hint, s->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, float alpha) {
  erase();
  scene_draw(sc, alpha);
  draw_hud_top(s, sc, fps);
  draw_hud_bottom(s);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

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
  screen_resize(&app->screen);
  scene_init(&app->scene, app->scene.n_links, app->screen.cols,
             app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    sc->paused = !sc->paused;
    break;

  case 'r':
  case 'R':
    scene_init(sc, sc->n_links, app->screen.cols, app->screen.rows);
    break;

  case 'n':
  case 'N': {
    int nn = sc->n_links + 1;
    if (nn > N_MAX)
      nn = N_MAX;
    scene_init(sc, nn, app->screen.cols, app->screen.rows);
    break;
  }
  case 'p':
  case 'P': {
    int nn = sc->n_links - 1;
    if (nn < 1)
      nn = 1;
    scene_init(sc, nn, app->screen.cols, app->screen.rows);
    break;
  }

  case 'g':
  case 'G':
    sc->show_ghost = !sc->show_ghost;
    break;

  case 'l':
  case 'L':
    sc->show_trail = !sc->show_trail;
    break;

  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    theme_apply(sc->theme);
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme);
    break;

  case '+':
  case '=':
    sc->trail_draw += TRAIL_STEP;
    if (sc->trail_draw > TRAIL_LEN)
      sc->trail_draw = TRAIL_LEN;
    break;

  case '-':
    sc->trail_draw -= TRAIL_STEP;
    if (sc->trail_draw < TRAIL_MIN)
      sc->trail_draw = TRAIL_MIN;
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
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, N_DEFAULT, app->screen.cols, app->screen.rows);

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
      scene_tick(&app->scene, dt_sec);
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

    screen_draw(&app->screen, &app->scene, fps_display, alpha);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
