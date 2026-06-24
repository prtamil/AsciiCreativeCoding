/*  conjugate_gradient_linear_solver.c
 *
 *  Two ways to solve the same linear system Ax = b, racing side by side.
 *  Both walk downhill to the bottom of a bowl-shaped surface; the bottom
 *  is the answer. Conjugate Gradient (green) gets there in just two steps.
 *  Steepest Descent (red) follows the slope and zig-zags forever. Drawing
 *  both paths on one bowl makes the difference obvious at a glance.
 *
 *  The system is 2x2, so the bowl you see on screen IS the real surface
 *  the math walks on — nothing is faked or flattened.
 *
 *  Best companion read: Shewchuk's "Conjugate Gradient Without the
 *  Agonizing Pain" (CMU CS-94-125, 1994) — start there if CG is new.
 *  The CONCEPTS block below explains the idea; the references at its end
 *  cite the original papers.
 */

/* ── CONCEPTS ──
 *
 * The problem    : Solve Ax = b. Here A is "symmetric positive definite"
 *                  — a matrix whose energy surface is a clean bowl with a
 *                  single bottom, no saddles or ridges. Solving the system
 *                  is the same as finding the lowest point of that bowl,
 *                  because the slope is zero exactly where Ax = b. So
 *                  "solve the equations" turns into "walk to the bottom".
 *
 * Conjugate      : Hestenes & Stiefel, 1952. Keeps a current guess x, the
 * Gradient (CG)    leftover error r = b - Ax (how wrong we still are), and
 *                  a chosen direction p to step along. Each step takes the
 *                  exact best stride down p, then picks the next direction
 *                  cleverly so it never undoes earlier progress. On a 2x2
 *                  bowl that means done in 2 steps. One matrix-times-vector
 *                  multiply per step, plus a couple of dot products.
 *
 * Steepest       : Cauchy, 1847. The naive method: always step straight
 * Descent (SD)     downhill (along r), as far as helps. The catch — each
 *                  new downhill direction comes out at a right angle to the
 *                  last one, so on a long stretched bowl the path bounces
 *                  side to side and crawls toward the bottom. It works, but
 *                  slowly.
 *
 * Why CG wins    : CG's directions are picked so that, in the bowl's own
 *                  stretched geometry, each is "independent" of the others.
 *                  Every step cleans up one direction once and for all and
 *                  never revisits it — so no zig-zag.
 *
 * The eigen-panel: A bowl can be stretched more along one axis than
 *                  another. Those two natural axes (the eigenvectors) and
 *                  how stretched each is (the eigenvalues) decide how hard
 *                  the problem is — the more lopsided the stretch, the more
 *                  SD struggles. The right-hand panel splits each method's
 *                  leftover error along those two axes so you can watch
 *                  which one dies first: CG kills both fast, SD lets the
 *                  hard one linger.
 *
 * Why 2x2        : With two unknowns the bowl on screen is the actual
 *                  surface — the path you watch matches the math exactly.
 *                  Bigger systems would need a flattened slice that lies
 *                  about the motion.
 *
 * Drawing        : The bowl is shown as contour rings (like a topographic
 *                  map): chop the height into bands and mark a cell only
 *                  where it sits on the edge between two bands. The two
 *                  paths are drawn as dotted lines with a dot at each stop,
 *                  so each reads as a trail you can follow with your eye.
 *                  CG green / SD red / target gold never change with the
 *                  theme, so the story stays readable.
 *
 * References     :
 *   • Hestenes, M. R. & Stiefel, E. (1952) "Methods of Conjugate
 *     Gradients for Solving Linear Systems", J. Res. NBS 49(6), 409-436.
 *     — The original paper where CG was invented. Direct, terse.
 *
 *   • Shewchuk, J. R. (1994) "An Introduction to the Conjugate Gradient
 *     Method Without the Agonizing Pain", CMU CS-94-125.
 *     — The best place to learn CG: pictures, intuition, every formula
 *       built up from scratch. Read this first if CG is new.
 *
 *   • Saad, Y. (2003) "Iterative Methods for Sparse Linear Systems"
 *     (2nd ed.), SIAM. — The reference text; ch. 6 covers CG in depth.
 *
 *   • Trefethen, L. N. & Bau, D. (1997) "Numerical Linear Algebra", SIAM.
 *     — Lecture 38 is CG; lectures 24-28 cover the eigenvalue picture the
 *       eigen-panel draws.
 *
 *   • Bresenham, J. E. (1965) "Algorithm for computer control of a
 *     digital plotter", IBM Systems Journal 4(1), 25-30.
 *     — The line-drawing trick the dotted trails descend from.
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

#define N 2                 /* two unknowns — the bowl on screen is the real one */
#define MAX_ITER_CG (N + 2) /* CG finishes in at most N steps */
#define MAX_ITER_SD 80      /* SD cap (high enough to feel endless) */
#define HISTORY_LEN_CG (MAX_ITER_CG + 2)
#define HISTORY_LEN_SD (MAX_ITER_SD + 2)

#define SIM_FPS_DEFAULT 30 /* how many times a second we tick + redraw */
#define SIM_FPS_MIN 6
#define SIM_FPS_MAX 120
#define CG_STEP_TICKS 60 /* CG takes a step every 2 seconds */
#define SD_STEP_TICKS 20 /* SD steps 3x as often, so you watch it limp */
#define HOLD_TICKS 90    /* freeze 3s after both finish, then restart */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define RES_NORM_TOL 1e-6f /* "close enough to the answer" cutoff */
#define EIGEN_PANEL_W 24   /* columns set aside for the right-hand panel */
#define N_CONTOURS 9       /* how many contour rings to draw */
#define N_THEMES 10

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

/* ── §3 color — themes + fixed marker colours ── */

/*
 * Colour slots. The four contour-ring colours follow the active theme;
 * everything else is fixed so the story reads the same in every theme.
 *
 *   CP_E_LOW..CP_E_PEAK   the four contour-ring shades, dark to bright
 *   CP_CG                 green  — CG's trail and (X) marker
 *   CP_SD                 red    — SD's trail and (Y) marker
 *   CP_TARGET             gold   — the true answer, marked [*]
 *   CP_EIGEN              cyan   — labels in the right-hand panel
 *   CP_HUD / CP_HEADER / CP_LABEL / CP_EXPLAIN / CP_TOP / CP_HINT
 *                         the standard overlay/status colours (CLAUDE.md)
 */
enum {
  CP_DEFAULT = 0,
  CP_E_LOW,
  CP_E_MID,
  CP_E_HIGH,
  CP_E_PEAK,
  CP_CG,
  CP_SD,
  CP_TARGET,
  CP_EIGEN,
  CP_HUD,
  CP_HEADER,
  CP_LABEL,
  CP_EXPLAIN,
  CP_TOP,
  CP_HINT,
  CP_COUNT
};

/*
 * Theme — a named four-colour set for the contour rings.
 *   name   shown in the status bar; cycled with t/T.
 *   ramp   four 256-colour codes, darkest to brightest, used for the
 *          contour bands from the bowl's floor up to its rim.
 */
typedef struct {
  const char *name;
  short ramp[4];
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp: darkest .. brightest                              */
    {"Matrix", {28, 34, 40, 46}},      /* cyber green       */
    {"Fire", {130, 208, 202, 196}},    /* warm → red        */
    {"Oceanic", {24, 31, 39, 51}},     /* teal → cyan       */
    {"Neon", {129, 165, 201, 213}},    /* purple → pink     */
    {"Mono", {240, 247, 250, 255}},    /* grayscale         */
    {"Ice", {153, 117, 159, 195}},     /* light blues       */
    {"Nova", {129, 141, 177, 213}},    /* stellar           */
    {"Forest", {58, 100, 142, 190}},   /* leaves / bark     */
    {"Desert", {130, 178, 214, 220}},  /* sand / gold       */
    {"Eclipse", {240, 244, 124, 196}}, /* gray + red flare  */
};

static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_E_LOW, th->ramp[0], -1);
    init_pair(CP_E_MID, th->ramp[1], -1);
    init_pair(CP_E_HIGH, th->ramp[2], -1);
    init_pair(CP_E_PEAK, th->ramp[3], -1);
    init_pair(CP_CG, 46, -1);
    init_pair(CP_SD, 196, -1);
    init_pair(CP_TARGET, 226, -1);
    init_pair(CP_EIGEN, 51, -1);
    init_pair(CP_HUD, 252, -1);
    init_pair(CP_HEADER, 51, -1);
    init_pair(CP_LABEL, 244, -1);
    init_pair(CP_EXPLAIN, 227, -1);
    init_pair(CP_TOP, 226, -1);
    init_pair(CP_HINT, 51, -1);
  } else {
    init_pair(CP_E_LOW, COLOR_BLUE, -1);
    init_pair(CP_E_MID, COLOR_CYAN, -1);
    init_pair(CP_E_HIGH, COLOR_YELLOW, -1);
    init_pair(CP_E_PEAK, COLOR_RED, -1);
    init_pair(CP_CG, COLOR_GREEN, -1);
    init_pair(CP_SD, COLOR_RED, -1);
    init_pair(CP_TARGET, COLOR_YELLOW, -1);
    init_pair(CP_EIGEN, COLOR_CYAN, -1);
    init_pair(CP_HUD, COLOR_WHITE, -1);
    init_pair(CP_HEADER, COLOR_CYAN, -1);
    init_pair(CP_LABEL, COLOR_WHITE, -1);
    init_pair(CP_EXPLAIN, COLOR_YELLOW, -1);
    init_pair(CP_TOP, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  theme_apply(0);
}

/* ── §4 linear algebra (small 2x2 / 2-vector helpers) ── */

typedef float Mat[N * N]; /* a 2x2 matrix, stored row by row */
typedef float Vec[N];     /* a 2-element vector */

static void mat_vec_mul(const float *A, const float *x, float *out) {
  for (int i = 0; i < N; i++) {
    float s = 0.0f;
    for (int j = 0; j < N; j++)
      s += A[i * N + j] * x[j];
    out[i] = s;
  }
}

static float vdot(const float *a, const float *b) {
  float s = 0.0f;
  for (int i = 0; i < N; i++)
    s += a[i] * b[i];
  return s;
}

static float vnorm(const float *v) { return sqrtf(vdot(v, v)); }

/* out = a + alpha*b. Safe to pass the same array as both out and a. */
static void vaxpy(float *out, const float *a, float alpha, const float *b) {
  for (int i = 0; i < N; i++)
    out[i] = a[i] + alpha * b[i];
}

/*
 * The two eigenvalues of a symmetric 2x2 matrix — how much it stretches
 * along each of its two natural axes. Closed-form (just a quadratic), so
 * no iteration. Returned largest first (*out_l1 >= *out_l2).
 */
static void solve_secular_eq_2x2(float a, float b, float c, float *out_l1,
                                 float *out_l2) {
  float tr = a + c;
  float det = a * c - b * b;
  float disc = sqrtf(fmaxf(tr * tr * 0.25f - det, 0.0f));
  *out_l1 = tr * 0.5f + disc;
  *out_l2 = tr * 0.5f - disc;
}

/*
 * The natural axis (unit length) that goes with one eigenvalue — i.e. a
 * direction the matrix only stretches, never turns. If the matrix is
 * already aligned to the x/y axes (b ~ 0), just hand back x or y.
 */
static void eigenvector_for_value_2x2(float a, float b, float lambda, Vec out) {
  if (fabsf(b) > 1e-10f) {
    out[0] = b;
    out[1] = lambda - a;
  } else {
    /* Already axis-aligned: pick whichever of x or y this eigenvalue
     * belongs to. */
    if (fabsf(lambda - a) < 1e-10f) {
      out[0] = 1.0f;
      out[1] = 0.0f;
    } else {
      out[0] = 0.0f;
      out[1] = 1.0f;
    }
  }
  float n = sqrtf(out[0] * out[0] + out[1] * out[1]);
  if (n > 1e-10f) {
    out[0] /= n;
    out[1] /= n;
  }
}

/* The direction at a right angle to `in` (turned 90 degrees). */
static void perpendicular_2d(const Vec in, Vec out) {
  out[0] = -in[1];
  out[1] = in[0];
}

/*
 * Finds the bowl's two natural axes and how stretched it is along each:
 * the two eigenvalues (l1 >= l2) and the matching directions (v1, v2,
 * which sit at right angles). The eigen-panel uses these to split each
 * method's leftover error per axis.
 */
static void eigen2(const Mat A, float *out_l1, float *out_l2, Vec out_v1,
                   Vec out_v2) {
  float a = A[0], b = A[1], c = A[3];
  solve_secular_eq_2x2(a, b, c, out_l1, out_l2);
  eigenvector_for_value_2x2(a, b, *out_l1, out_v1);
  perpendicular_2d(out_v1, out_v2);
}

/* ── §5 build a random problem + solve it exactly ── */

/*
 * Make a random bowl that's reliably stretched, so the two methods look
 * different (on a round bowl they'd behave the same and the demo would be
 * dull). We pick how much it stretches along each axis, pick a random
 * tilt, and build the matrix from that.
 */
static void build_spd(Mat A, unsigned seed) {
  srand(seed);
  float l1 = 0.6f + (float)(rand() % 100) / 200.0f; /* 0.6 .. 1.1 */
  float l2 = 2.5f + (float)(rand() % 100) / 50.0f;  /* 2.5 .. 4.5 */
  float theta = (float)(rand() % 360) * (float)M_PI / 180.0f;
  float cs = cosf(theta), sn = sinf(theta);

  float a = l1 * cs * cs + l2 * sn * sn;
  float b = (l1 - l2) * cs * sn;
  float c = l1 * sn * sn + l2 * cs * cs;

  A[0] = a;
  A[1] = b;
  A[2] = b;
  A[3] = c;
}

/*
 * Cholesky: split A into L times its own mirror image (a triangular L,
 * the matrix "square root"). This is the prep step that makes solving
 * Ax = b a quick two-pass sweep. Reference: Golub & Van Loan §4.2.
 */
static void cholesky_decompose(const float *A, float *L) {
  memset(L, 0, sizeof(float) * N * N);
  for (int i = 0; i < N; i++) {
    for (int j = 0; j <= i; j++) {
      float s = A[i * N + j];
      for (int k = 0; k < j; k++)
        s -= L[i * N + k] * L[j * N + k];
      if (i == j) {
        L[i * N + j] = (s > 0.0f) ? sqrtf(s) : 1e-10f;
      } else {
        L[i * N + j] = s / L[j * N + j];
      }
    }
  }
}

/*
 * Solve L*y = b by sweeping top to bottom — each row has only one new
 * unknown, so you just read them off in order.
 */
static void forward_substitute(const float *L, const float *b, float *y) {
  for (int i = 0; i < N; i++) {
    float s = b[i];
    for (int k = 0; k < i; k++)
      s -= L[i * N + k] * y[k];
    y[i] = s / L[i * N + i];
  }
}

/*
 * The mirror of forward_substitute: solve Lᵀ*x = y by sweeping bottom to
 * top instead. Together the two passes finish solving the system.
 */
static void back_substitute_transpose(const float *L, const float *y,
                                      float *x) {
  for (int i = N - 1; i >= 0; i--) {
    float s = y[i];
    for (int k = i + 1; k < N; k++)
      s -= L[k * N + i] * x[k];
    x[i] = s / L[i * N + i];
  }
}

/*
 * The exact answer to Ax = b, computed in one shot (split A, then sweep
 * down and back up). This is the gold target drawn as [*] that the two
 * iterative methods race toward. Called once per new problem, never in
 * the animation loop.
 */
static void cholesky_solve(const float *A, const float *b, float *x) {
  float L[N * N];
  Vec y;
  cholesky_decompose(A, L);
  forward_substitute(L, b, y);
  back_substitute_transpose(L, y, x);
}

/* ── §6 solvers — CG and SD ── */

/*
 * CGSolver — everything one run of Conjugate Gradient needs to remember.
 * The first group is the live working state, kept exactly as the 1952
 * Hestenes-Stiefel recipe describes it; the rest is bookkeeping for
 * stopping and for drawing the trail. Fields are grouped by who touches
 * them, not by type.
 *
 *   working state — read and rewritten on every step:
 *     x      the current guess (the answer so far)
 *     r      how wrong we still are: r = b - A*x (also points downhill)
 *     p      the direction we'll step along next — chosen so it doesn't
 *            undo any earlier step
 *     Ap     A applied to p, saved so we don't recompute it
 *     rr     r dotted with itself, saved to avoid redoing that dot product
 *     alpha  how far we stepped on the last step
 *     beta   how much of the old direction we mixed into the new one
 *
 *   stopping — set by cg_step, read by everyone:
 *     iter       how many steps are done (0..MAX_ITER_CG-1)
 *     converged  once true, cg_step does nothing more
 *
 *   trail snapshot — written once per step, read every frame to draw:
 *     hist[k]  where the guess was at step k (its x,y)
 *     res[k]   how wrong it was at step k. Room for MAX_ITER_CG+2 stops,
 *              plenty for a 2x2 problem.
 *
 * References: Hestenes & Stiefel 1952 §3 names each quantity above;
 *             Shewchuk 1994 §B7 shows the same steps with pictures.
 */
typedef struct {
  /* working state — read+written every cg_step */
  Vec x;       /* current guess */
  Vec r;       /* how wrong we still are: b - A*x */
  Vec p;       /* next direction to step along */
  Vec Ap;      /* A applied to p, cached */
  float rr;    /* r dotted with itself, cached */
  float alpha; /* how far the last step went */
  float beta;  /* how much of the old direction the new one kept */

  /* stopping — set by cg_step */
  int iter;       /* steps completed, 0..MAX_ITER_CG-1 */
  bool converged; /* once true, cg_step is a no-op */

  /* trail snapshot — written once per step, read every frame */
  float hist[HISTORY_LEN_CG][2]; /* guess position at each step */
  float res[HISTORY_LEN_CG];     /* how wrong it was at each step */
} CGSolver;

/*
 * SDSolver — one run of Steepest Descent (Cauchy 1847), the foil to CG.
 * Simpler than CG: there's no chosen direction to remember, because SD
 * always just steps straight downhill (along r). That's also its flaw —
 * each new downhill comes out at a right angle to the last, so on a
 * stretched bowl the path zig-zags and crawls. Same field layout as
 * CGSolver so the two read side by side.
 *
 *   working state — read and rewritten each step:
 *     x      the current guess
 *     r      how wrong we still are / the downhill direction
 *     alpha  how far the last step went
 *
 *   stopping:
 *     iter       steps completed (0..MAX_ITER_SD-1)
 *     converged  reached tolerance, or hit the step cap
 *
 *   trail snapshot — written once per step, read every frame:
 *     hist[k]  the zig-zag path, position at each step
 *     res[k]   how wrong it was at each step. Sized MAX_ITER_SD+2 — SD
 *              takes dozens of steps where CG takes two.
 *
 * References: Cauchy 1847 (the original descent method); Trefethen & Bau
 *             1997 §38.3 explains the zig-zag; Shewchuk 1994 §6 compares
 *             SD and CG with pictures.
 */
typedef struct {
  /* working state — read+written every sd_step */
  Vec x;       /* current guess */
  Vec r;       /* how wrong we still are / the downhill direction */
  float alpha; /* how far the last step went */

  /* stopping */
  int iter;       /* steps completed, 0..MAX_ITER_SD-1 */
  bool converged; /* reached tolerance or hit the step cap */

  /* trail snapshot — written once per step, read every frame */
  float hist[HISTORY_LEN_SD][2]; /* the zig-zag path */
  float res[HISTORY_LEN_SD];     /* how wrong it was at each step */
} SDSolver;

/* Start CG fresh from the origin, ready for a new problem. */
static void cg_reset(CGSolver *s, const Mat A, const Vec b) {
  memset(s->x, 0, sizeof s->x);
  memcpy(s->r, b, sizeof s->r);
  memcpy(s->p, b, sizeof s->p);
  mat_vec_mul(A, s->p, s->Ap);
  s->rr = vdot(s->r, s->r);
  s->alpha = 0.0f;
  s->beta = 0.0f;
  s->iter = 0;
  s->converged = false;
  s->hist[0][0] = s->x[0];
  s->hist[0][1] = s->x[1];
  s->res[0] = vnorm(s->r);
}

/*
 * When CG has nothing left to do. Stops if it already finished, if it
 * hit the safety cap on steps, or if it's now close enough to the
 * answer. The last two also latch the "finished" flag so later calls
 * bail straight away.
 */
static bool cg_should_stop(CGSolver *s) {
  if (s->converged)
    return true;
  if (s->iter >= MAX_ITER_CG - 1) {
    s->converged = true;
    return true;
  }
  if (vnorm(s->r) < RES_NORM_TOL) {
    s->converged = true;
    return true;
  }
  return false;
}

/*
 * How far to step along the current direction — the exact stride that
 * reaches the lowest point on that line, no guessing. If the direction
 * somehow has no slope to descend (shouldn't happen on a real bowl, but
 * cheap to guard), it marks itself finished and steps nowhere.
 */
static float cg_optimal_step_size(CGSolver *s) {
  float pAp = vdot(s->p, s->Ap);
  if (fabsf(pAp) < 1e-14f) {
    s->converged = true;
    return 0.0f;
  }
  return s->rr / pAp;
}

/*
 * Take the step: slide the guess along the chosen direction, and update
 * how-wrong-we-are the cheap way (a running formula, no fresh matrix
 * multiply). Refreshes the cached r·r that the next-direction step needs.
 */
static void cg_walk_and_update_residual(CGSolver *s, float alpha) {
  vaxpy(s->x, s->x, alpha, s->p);
  vaxpy(s->r, s->r, -alpha, s->Ap);
  s->rr = vdot(s->r, s->r);
}

/*
 * Choose the next direction to step in. It's a blend of straight downhill
 * plus a little of the old direction — mixed just so that this step won't
 * spoil any earlier one (that's the trick that kills the zig-zag). Also
 * caches the matrix-times-direction the next step will reuse.
 */
static void cg_new_direction_fletcher_reeves(CGSolver *s, float rr_old,
                                             const Mat A) {
  s->beta = s->rr / rr_old;
  vaxpy(s->p, s->r, s->beta, s->p);
  mat_vec_mul(A, s->p, s->Ap);
}

/* Save where this step landed so the trail can be drawn, and note if
 * we've arrived. */
static void cg_record_step(CGSolver *s) {
  s->iter++;
  if (s->iter < HISTORY_LEN_CG) {
    s->hist[s->iter][0] = s->x[0];
    s->hist[s->iter][1] = s->x[1];
    s->res[s->iter] = vnorm(s->r);
  }
  if (vnorm(s->r) < RES_NORM_TOL)
    s->converged = true;
}

/*
 * One CG step, in the four moves of the recipe: figure out how far to go,
 * take the step (and update how wrong we are), pick the next direction,
 * then record the result.
 */
static void cg_step(CGSolver *s, const Mat A) {
  if (cg_should_stop(s))
    return;

  float rr_old = s->rr;
  float alpha = cg_optimal_step_size(s);
  if (s->converged)
    return;
  s->alpha = alpha;

  cg_walk_and_update_residual(s, alpha);
  cg_new_direction_fletcher_reeves(s, rr_old, A);
  cg_record_step(s);
}

/* sd_reset — same start state as CG so the comparison is fair. */
static void sd_reset(SDSolver *s, const Vec b) {
  memset(s->x, 0, sizeof s->x);
  memcpy(s->r, b, sizeof s->r);
  s->alpha = 0.0f;
  s->iter = 0;
  s->converged = false;
  s->hist[0][0] = s->x[0];
  s->hist[0][1] = s->x[1];
  s->res[0] = vnorm(s->r);
}

/* sd_should_stop — early-exit predicates, parallel to cg_should_stop. */
static bool sd_should_stop(SDSolver *s) {
  if (s->converged)
    return true;
  if (s->iter >= MAX_ITER_SD - 1) {
    s->converged = true;
    return true;
  }
  if (vnorm(s->r) < RES_NORM_TOL) {
    s->converged = true;
    return true;
  }
  return false;
}

/*
 * How far to step straight downhill — the exact stride that bottoms out
 * along the downhill line. Same idea as CG's stride, but SD always heads
 * straight downhill and never adjusts for past steps, which is exactly
 * why it zig-zags. Marks itself finished if there's no slope to descend.
 */
static float sd_optimal_step_size(SDSolver *s, const Vec Ar) {
  float rr = vdot(s->r, s->r);
  float rAr = vdot(s->r, Ar);
  if (rAr < 1e-14f) {
    s->converged = true;
    return 0.0f;
  }
  return rr / rAr;
}

/*
 * Take the step downhill and update how-wrong-we-are the cheap way. A
 * quirk of the math: each new downhill direction comes out at a right
 * angle to the last one — which is what makes the path bounce side to
 * side forever on a stretched bowl.
 */
static void sd_walk_and_update_residual(SDSolver *s, float alpha,
                                        const Vec Ar) {
  vaxpy(s->x, s->x, alpha, s->r);
  vaxpy(s->r, s->r, -alpha, Ar);
}

/* Save where this step landed for the trail, and note if we've arrived. */
static void sd_record_step(SDSolver *s) {
  s->iter++;
  if (s->iter < HISTORY_LEN_SD) {
    s->hist[s->iter][0] = s->x[0];
    s->hist[s->iter][1] = s->x[1];
    s->res[s->iter] = vnorm(s->r);
  }
  if (vnorm(s->r) < RES_NORM_TOL)
    s->converged = true;
}

/*
 * One SD step (Cauchy 1847): work out how far to step straight downhill,
 * take it, and record the result.
 */
static void sd_step(SDSolver *s, const Mat A) {
  if (sd_should_stop(s))
    return;

  Vec Ar;
  mat_vec_mul(A, s->r, Ar);

  float alpha = sd_optimal_step_size(s, Ar);
  if (s->converged)
    return;
  s->alpha = alpha;

  sd_walk_and_update_residual(s, alpha, Ar);
  sd_record_step(s);
}

/* ── §7 scene ── */

/*
 * Scene — everything the demo needs to remember, in one place.
 *
 * One Scene IS the whole demo: both racers, the problem they solve (A
 * and b), the answer, the bowl's natural axes, the timing counters, and
 * the on-screen settings. Pass &scene around; nothing important lives
 * outside it. Fields are grouped by who uses them, and the drawing code
 * only ever reads them, never writes.
 *
 *   the problem — A, b, x_true
 *       A and b are the system Ax = b; x_true is the exact answer, found
 *       once up front and drawn as the gold [*] goal. Set when a new
 *       problem starts; read by both racers every step and by the bowl
 *       drawing every frame.
 *
 *   the bowl's natural axes — l1, l2, v1, v2
 *       v1 and v2 are the two directions the bowl is stretched along;
 *       l1 and l2 say how stretched (l1 the bigger). Neither racer needs
 *       these — only the right-hand panel does, to split the leftover
 *       error per axis.
 *
 *   the two racers — cg, sd
 *       Conjugate Gradient and Steepest Descent, both working the same
 *       bowl. Stepped at their own rhythms; drawing reads their state.
 *
 *   timing — cg_acc, sd_acc, hold
 *       cg_acc / sd_acc count ticks so each racer steps on its own beat
 *       (CG slow, SD fast — how you see CG sprint and SD limp). Once
 *       both finish, hold counts up to HOLD_TICKS, then a fresh random
 *       problem starts.
 *
 *   on-screen settings — paused, theme
 *       Changed only by key presses. paused freezes the sim; theme picks
 *       the colour set.
 *
 *   restart — seed
 *       The random seed for the NEXT problem. Bumped on the `r` key and
 *       on auto-restart.
 *
 * Reference: Shewchuk 1994 §B tells the whole CG-vs-SD story as one
 *            picture on one bowl — what this scene puts on screen.
 */
typedef struct {
  /* the problem — set once per new problem, read everywhere */
  Mat A;      /* the matrix both racers work on */
  Vec b;      /* right-hand side (non-zero, so the answer isn't the origin) */
  Vec x_true; /* the exact answer, drawn as the gold [*] */

  /* the bowl's natural axes — only the right-hand panel reads these */
  float l1, l2; /* how stretched along each axis, bigger first */
  Vec v1, v2;   /* the two axes themselves (at right angles, unit length) */

  /* the two racers */
  CGSolver cg; /* Conjugate Gradient */
  SDSolver sd; /* Steepest Descent */

  /* timing — each tick bumps a counter; a racer steps when its counter
   * crosses its threshold. Different thresholds give CG one slow step
   * for every few quick SD steps. */
  int cg_acc;
  int sd_acc;
  int hold; /* after both finish, counts up to HOLD_TICKS, then restart */

  /* on-screen settings — changed only by key presses */
  bool paused; /* space freezes the sim */
  int theme;   /* t/T picks the colour set */

  /* restart */
  unsigned seed; /* random seed for the next problem */
} Scene;

static void scene_reset(Scene *sc) {
  build_spd(sc->A, sc->seed);
  /* Pick a non-zero right-hand side so the answer isn't at the origin. */
  sc->b[0] = 1.4f;
  sc->b[1] = 0.7f;
  cholesky_solve(sc->A, sc->b, sc->x_true);
  eigen2(sc->A, &sc->l1, &sc->l2, sc->v1, sc->v2);

  cg_reset(&sc->cg, sc->A, sc->b);
  sd_reset(&sc->sd, sc->b);
  sc->cg_acc = 0;
  sc->sd_acc = 0;
  sc->hold = 0;
}

static void scene_init(Scene *sc) {
  memset(sc, 0, sizeof *sc);
  sc->seed = (unsigned)time(NULL);
  sc->theme = 0;
  sc->paused = false;
  scene_reset(sc);
  theme_apply(sc->theme);
}

/*
 * tick_post_convergence — what happens when both algorithms are done.
 * Hold the final tableau for HOLD_TICKS render frames so the user can
 * actually look at it, then bump the seed and reset for a new race.
 */
static void tick_post_convergence(Scene *sc) {
  if (sc->hold < HOLD_TICKS) {
    sc->hold++;
  } else {
    sc->seed += 99991;
    scene_reset(sc);
  }
}

/*
 * pace_cg_step / pace_sd_step — tick-rate throttles.
 * Each tick increments the corresponding accumulator; the solver fires
 * a single step once the accumulator crosses its STEP_TICKS threshold.
 * Different thresholds = different per-real-time step rates, which is
 * how SD ends up taking 3 steps for every 1 of CG.
 */
static void pace_cg_step(Scene *sc) {
  if (sc->cg.converged)
    return;
  if (++sc->cg_acc >= CG_STEP_TICKS) {
    cg_step(&sc->cg, sc->A);
    sc->cg_acc = 0;
  }
}

static void pace_sd_step(Scene *sc) {
  if (sc->sd.converged)
    return;
  if (++sc->sd_acc >= SD_STEP_TICKS) {
    sd_step(&sc->sd, sc->A);
    sc->sd_acc = 0;
  }
}

/*
 * scene_tick — one render-rate tick.  Three things may happen:
 *   1. paused → no-op
 *   2. both converged → hold + maybe auto-restart
 *   3. otherwise → throttled CG step + throttled SD step
 *
 * Asymmetric pacing is the whole point: CG gets long beats between steps
 * (you see it sprint to *), SD gets quick beats (you see it zig-zag).
 */
static void scene_tick(Scene *sc) {
  if (sc->paused)
    return;

  if (sc->cg.converged && sc->sd.converged) {
    tick_post_convergence(sc);
    return;
  }
  pace_cg_step(sc);
  pace_sd_step(sc);
}

/* ── §8 rendering ── */

#define HDR_ROWS 2 /* row 0 status, row 1 divider */
#define FTR_ROWS 8 /* 7 rows narrator + 1 row action HUD */

/*
 * View — where the camera is pointed this frame.
 *
 * Pure drawing-side info: no sim code touches it, and it lasts only one
 * frame. compute_view builds it from the current Scene, then hands it
 * (read-only) to every drawing helper so they all agree on where things
 * land on screen. Bundling it in one struct beats passing the same six
 * numbers to every helper.
 *
 * Two coordinate worlds meet here:
 *
 *   world coordinates — the real (x1, x2) space the algorithms move in.
 *                       A guess sits at something like (0.42, -0.18). The
 *                       visible window is xlo..xhi across and ylo..yhi up.
 *
 *   screen cells      — terminal character cells, (0, 0) at top-left.
 *                       The bowl fills columns 1..plot_w-2 and rows
 *                       HDR_ROWS..HDR_ROWS+ph-1.
 *
 * to_col / to_row convert a world point to a cell. The row direction is
 * flipped (high y at the top row) to match how screens count rows.
 *
 * The window re-fits itself every frame: it starts small around the
 * answer, then grows to take in every step of both racers. That keeps
 * SD's wandering trail on screen without losing the bowl while the early
 * steps are still huddled near the origin.
 */
typedef struct {
  /* the bowl's size on screen — set once per frame */
  int plot_w; /* how many columns the bowl gets */
  int ph;     /* how many rows the bowl gets */

  /* the visible window in world coordinates — high y maps to the top
   * row, matching the screen's top-down row order */
  float xlo, xhi; /* visible left-right range */
  float ylo, yhi; /* visible bottom-top range */
} View;

static int to_col(float v, float lo, float hi, int c0, int c1) {
  if (hi <= lo)
    return c0;
  float t = (v - lo) / (hi - lo);
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  return c0 + (int)(t * (float)(c1 - c0));
}
static int to_row(float v, float lo, float hi, int r0, int r1) {
  if (hi <= lo)
    return r0;
  float t = (v - lo) / (hi - lo);
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  return r1 - (int)(t * (float)(r1 - r0));
}

static float energy2d(const Scene *sc, float x0, float x1) {
  float a00 = sc->A[0], a01 = sc->A[1], a11 = sc->A[3];
  float q = 0.5f * (a00 * x0 * x0 + 2.0f * a01 * x0 * x1 + a11 * x1 * x1);
  return q - sc->b[0] * x0 - sc->b[1] * x1;
}

/*
 * view_layout — assign cell-space dimensions to the View.
 * Bowl gets cols minus the eigen panel; height is rows minus HUD chrome.
 */
static void view_layout(int rows, int cols, int *out_plot_w, int *out_ph) {
  int plot_w = cols - EIGEN_PANEL_W;
  if (plot_w < 30)
    plot_w = 30;
  int ph = rows - HDR_ROWS - FTR_ROWS;
  if (ph < 4)
    ph = 4;
  *out_plot_w = plot_w;
  *out_ph = ph;
}

/*
 * Widen the view's reach so every step of a racer's path stays on
 * screen, with a little padding so the trail never touches the edge.
 */
static void expand_range_to_fit_trail(const float (*hist)[2], int n_iter,
                                      float cx, float cy, float *range) {
  for (int k = 0; k <= n_iter; k++) {
    float dx = fabsf(hist[k][0] - cx);
    float dy = fabsf(hist[k][1] - cy);
    if (dx + 0.20f > *range)
      *range = dx + 0.20f;
    if (dy + 0.20f > *range)
      *range = dy + 0.20f;
  }
}

/*
 * compute_view — build the per-frame camera.
 *   1. cell-space dimensions (plot_w, ph)
 *   2. centre on x*
 *   3. auto-zoom outward to contain BOTH algorithms' trails
 *
 * Auto-zoom is what keeps SD's wandering path in frame without losing
 * the bowl when iterates are still clustered near the origin.
 */
static View compute_view(const Scene *sc, int rows, int cols) {
  View v;
  view_layout(rows, cols, &v.plot_w, &v.ph);

  float cx = sc->x_true[0];
  float cy = sc->x_true[1];
  float range = 0.5f;
  expand_range_to_fit_trail(sc->cg.hist, sc->cg.iter, cx, cy, &range);
  expand_range_to_fit_trail(sc->sd.hist, sc->sd.iter, cx, cy, &range);

  v.xlo = cx - range;
  v.xhi = cx + range;
  v.ylo = cy - range;
  v.yhi = cy + range;
  return v;
}

/*
 * Turn a screen cell back into the world point it stands for — the
 * reverse of placing a world point on screen.
 */
static void cell_to_world(const View *v, int c, int r, float *out_xv,
                          float *out_yv) {
  *out_xv =
      v->xlo + (v->xhi - v->xlo) * (float)(c - 1) / (float)(v->plot_w - 3);
  *out_yv = v->yhi - (v->yhi - v->ylo) * (float)r / (float)(v->ph - 1);
}

/*
 * First pass of the topo-map drawing: walk every cell to find the lowest
 * and highest height on screen, so the next pass can slice that range
 * into even bands.
 */
static void find_energy_window(const Scene *sc, const View *v, float *out_emin,
                               float *out_emax) {
  float emin = 1e30f, emax = -1e30f;
  for (int r = 0; r < v->ph; r++) {
    for (int c = 1; c < v->plot_w - 1; c++) {
      float xv, yv;
      cell_to_world(v, c, r, &xv, &yv);
      float e = energy2d(sc, xv, yv);
      if (e < emin)
        emin = e;
      if (e > emax)
        emax = e;
    }
  }
  *out_emin = emin;
  *out_emax = emax;
}

/*
 * Decide whether a cell sits on the edge between two height bands — i.e.
 * on a contour ring of the topo map. Looks at the cell's own band and
 * its neighbours to the right and below; if a neighbour is in a
 * different band, a ring passes through here, and the change direction
 * picks the glyph ('|' for a vertical edge, '-' for horizontal, '.' for
 * a corner). Cells deep inside one band aren't on a ring, so they stay
 * blank.
 */
static bool contour_classify_cell(const Scene *sc, const View *v, int c, int r,
                                  float emin, float step, int *out_level,
                                  char *out_ch) {
  float xv, yv, xv_r, yv_d;
  cell_to_world(v, c, r, &xv, &yv);
  cell_to_world(v, c + 1, r, &xv_r, &yv);
  cell_to_world(v, c, r + 1, &xv, &yv_d);
  int lvl = (int)((energy2d(sc, xv, yv) - emin) / step);
  int lvl_r = (int)((energy2d(sc, xv_r, yv) - emin) / step);
  int lvl_d = (int)((energy2d(sc, xv, yv_d) - emin) / step);
  if (lvl == lvl_r && lvl == lvl_d)
    return false;

  char ch = '.';
  if (lvl != lvl_r && lvl == lvl_d)
    ch = '|';
  else if (lvl == lvl_r && lvl != lvl_d)
    ch = '-';
  *out_level = lvl;
  *out_ch = ch;
  return true;
}

/* Pick which of the four ring shades (floor up to rim) a given height
 * band gets. */
static int level_to_tier_pair(int level) {
  int tier = (level * 4) / N_CONTOURS;
  if (tier < 0)
    tier = 0;
  if (tier > 3)
    tier = 3;
  return CP_E_LOW + tier;
}

/*
 * Draw the bowl as a topo map — nested rings, not a solid fill, so it
 * reads at a glance. First find the height range, then mark only the
 * cells that sit on a ring.
 */
static void render_bowl_contours(const Scene *sc, const View *v) {
  float emin, emax;
  find_energy_window(sc, v, &emin, &emax);
  if (emax <= emin)
    return;
  float step = (emax - emin) / (float)N_CONTOURS;

  for (int r = 0; r < v->ph - 1; r++) {
    for (int c = 1; c < v->plot_w - 2; c++) {
      int level;
      char ch;
      if (!contour_classify_cell(sc, v, c, r, emin, step, &level, &ch))
        continue;
      int cp = level_to_tier_pair(level);
      attron(COLOR_PAIR(cp));
      mvaddch(HDR_ROWS + r, c, (chtype)(unsigned char)ch);
      attroff(COLOR_PAIR(cp));
    }
  }
}

static void render_axis_labels(const View *v) {
  attron(COLOR_PAIR(CP_LABEL));
  mvprintw(HDR_ROWS + v->ph - 1, 1, "%.2f", (double)v->xlo);
  if (v->plot_w > 14)
    mvprintw(HDR_ROWS + v->ph - 1, v->plot_w - 10, "x1 %.2f", (double)v->xhi);
  mvprintw(HDR_ROWS, 0, "%.2f", (double)v->yhi);
  mvprintw(HDR_ROWS + v->ph / 2, 0, "x2");
  attroff(COLOR_PAIR(CP_LABEL));
}

/* Place one step of a racer's path onto its screen cell. */
static void iterate_to_cell(const float (*hist)[2], int k, const View *v,
                            int *out_c, int *out_r) {
  *out_c = to_col(hist[k][0], v->xlo, v->xhi, 1, v->plot_w - 2);
  *out_r = to_row(hist[k][1], v->ylo, v->yhi, HDR_ROWS, HDR_ROWS + v->ph - 1);
}

/* cell_in_bowl — bounds check inside the plot's drawable region. */
static bool cell_in_bowl(int c, int r, const View *v) {
  return r >= HDR_ROWS && r < HDR_ROWS + v->ph && c >= 1 && c < v->plot_w - 1;
}

/*
 * Draw a dotted straight line of '.' between two cells. Leaves the two
 * endpoints alone — the caller stamps a dot there instead.
 */
static void draw_dotted_segment(int c0, int r0, int c1, int r1, const View *v) {
  int dx = c1 - c0, dy = r1 - r0;
  int steps = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);
  for (int t = 1; t < steps; t++) {
    int cx = c0 + t * dx / steps;
    int cy = r0 + t * dy / steps;
    if (cell_in_bowl(cx, cy, v))
      mvaddch(cy, cx, '.');
  }
}

/*
 * render_trail — dotted line between consecutive iterates, `o` at each.
 * The trail reads as "a path you can trace with your eye", not as
 * scattered dots.
 */
static void render_trail(const float (*hist)[2], int n_iter, int color_pair,
                         const View *v) {
  attron(COLOR_PAIR(color_pair) | A_BOLD);
  for (int k = 0; k <= n_iter; k++) {
    int c0, r0;
    iterate_to_cell(hist, k, v, &c0, &r0);

    if (k < n_iter) {
      int c1, r1;
      iterate_to_cell(hist, k + 1, v, &c1, &r1);
      draw_dotted_segment(c0, r0, c1, r1, v);
    }
    if (cell_in_bowl(c0, r0, v))
      mvaddch(r0, c0, 'o');
  }
  attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * render_marker — bracketed glyph at (x,y) with an optional text label.
 * Label tries the upper-right first; flips to the left or below if there
 * isn't room.  Used for (X)=CG, (Y)=SD, [*]=GOAL.
 */
static void render_marker(float x, float y, char glyph, char lb, char rb,
                          const char *label, int color_pair, const View *v) {
  int c = to_col(x, v->xlo, v->xhi, 1, v->plot_w - 2);
  int r = to_row(y, v->ylo, v->yhi, HDR_ROWS, HDR_ROWS + v->ph - 1);
  if (r < HDR_ROWS || r >= HDR_ROWS + v->ph)
    return;
  if (c < 1 || c >= v->plot_w - 1)
    return;

  attron(COLOR_PAIR(color_pair) | A_BOLD);
  if (c - 1 >= 1)
    mvaddch(r, c - 1, lb);
  mvaddch(r, c, glyph);
  if (c + 1 < v->plot_w - 1)
    mvaddch(r, c + 1, rb);

  if (label && *label) {
    int lbl_len = (int)strlen(label);
    int lc = c + 3;
    if (lc + lbl_len >= v->plot_w - 1)
      lc = c - 2 - lbl_len;
    int lr = r - 1;
    if (lr < HDR_ROWS)
      lr = r + 1;
    if (lr >= HDR_ROWS && lr < HDR_ROWS + v->ph && lc >= 1 &&
        lc + lbl_len < v->plot_w - 1)
      mvprintw(lr, lc, "%s", label);
  }
  attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * render_bar — small inline bar at (y, x).  Filled cells `#` in fill_cp;
 * empty cells `.` in CP_LABEL (dim).  Used by the eigen panel and the
 * narrator's convergence bars.
 */
static void render_bar(int y, int x, int width, float frac, int fill_cp) {
  if (frac < 0.0f)
    frac = 0.0f;
  if (frac > 1.0f)
    frac = 1.0f;
  int filled = (int)(frac * (float)width + 0.5f);
  attron(COLOR_PAIR(fill_cp) | A_BOLD);
  for (int i = 0; i < filled; i++)
    mvaddch(y, x + i, '#');
  attroff(COLOR_PAIR(fill_cp) | A_BOLD);
  attron(COLOR_PAIR(CP_LABEL));
  for (int i = filled; i < width; i++)
    mvaddch(y, x + i, '.');
  attroff(COLOR_PAIR(CP_LABEL));
}

/* eigen_panel_header — "EIGENMODES" title at the top of the panel. */
static void eigen_panel_header(int x0, int top) {
  attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
  mvprintw(top, x0, "EIGENMODES");
  attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
}

/*
 * How much of each racer's leftover error points along one of the bowl's
 * natural axes. Returns that amount for CG and SD, plus the amount they
 * both started with, so the caller can show it as a "fraction still
 * left" bar. Reference: Shewchuk 1994 §B7 splits the error this way.
 */
static void project_error_onto_eigvec(const Scene *sc, const Vec v,
                                      float *out_cg_err, float *out_sd_err,
                                      float *out_start) {
  Vec cg_err, sd_err;
  for (int i = 0; i < N; i++) {
    cg_err[i] = sc->cg.x[i] - sc->x_true[i];
    sd_err[i] = sc->sd.x[i] - sc->x_true[i];
  }
  *out_cg_err = fabsf(vdot(cg_err, v));
  *out_sd_err = fabsf(vdot(sd_err, v));
  float start = fabsf(vdot(sc->x_true, v));
  if (start < 1e-6f)
    start = 1.0f;
  *out_start = start;
}

/*
 * draw_eigenmode_section — one eigenmode block: header + CG bar + SD bar.
 * Advances *y by 3 rows.
 */
static void draw_eigenmode_section(int *y, int x0, int bar_w, const char *label,
                                   float lambda, float cg_frac, float sd_frac) {
  attron(COLOR_PAIR(CP_EIGEN) | A_BOLD);
  mvprintw((*y)++, x0, label, (double)lambda);
  attroff(COLOR_PAIR(CP_EIGEN) | A_BOLD);

  attron(COLOR_PAIR(CP_HUD));
  mvprintw(*y, x0, " CG ");
  attroff(COLOR_PAIR(CP_HUD));
  render_bar(*y, x0 + 4, bar_w, cg_frac, CP_CG);
  (*y)++;

  attron(COLOR_PAIR(CP_HUD));
  mvprintw(*y, x0, " SD ");
  attroff(COLOR_PAIR(CP_HUD));
  render_bar(*y, x0 + 4, bar_w, sd_frac, CP_SD);
  (*y)++;
}

/*
 * Show how lopsided the bowl is — the ratio of its biggest stretch to
 * its smallest. The more lopsided, the harder SD struggles. A short
 * legend tells the viewer what a shorter bar means.
 */
static void draw_kappa_footer(int *y, int x0, float l1, float l2) {
  float kappa = (l2 > 1e-10f) ? (l1 / l2) : 0.0f;
  attron(COLOR_PAIR(CP_HUD));
  mvprintw((*y)++, x0, "κ(A) = %.2f", (double)kappa);
  mvprintw((*y)++, x0, "(eigenvalue ratio)");
  (*y)++;
  mvprintw((*y)++, x0, "Shorter bar =");
  mvprintw((*y)++, x0, "that mode is");
  mvprintw((*y)++, x0, "already killed.");
  attroff(COLOR_PAIR(CP_HUD));
}

/*
 * render_eigen_panel — error-in-eigenmodes view on the right.
 *
 *   1. title bar
 *   2. λ1 block (CG bar + SD bar)        — the "easy" mode
 *   3. λ2 block (CG bar + SD bar)        — the "hard" mode (slow)
 *   4. the lopsidedness footer
 *
 * Watching CG kill both modes in two steps while SD's λ2 bar barely
 * shrinks is the educational payoff of this panel.
 */
static void render_eigen_panel(const Scene *sc, int rows, int cols) {
  int x0 = cols - EIGEN_PANEL_W + 1;
  int top = HDR_ROWS;
  int ph = rows - HDR_ROWS - FTR_ROWS;
  if (x0 < 0 || ph < 12)
    return;

  int bar_w = EIGEN_PANEL_W - 8;
  if (bar_w < 5)
    bar_w = 5;

  eigen_panel_header(x0, top);

  int y = top + 2;

  float cg1, sd1, start1;
  project_error_onto_eigvec(sc, sc->v1, &cg1, &sd1, &start1);
  draw_eigenmode_section(&y, x0, bar_w, "λ1 = %.2f", sc->l1, cg1 / start1,
                         sd1 / start1);
  y++;

  float cg2, sd2, start2;
  project_error_onto_eigvec(sc, sc->v2, &cg2, &sd2, &start2);
  draw_eigenmode_section(&y, x0, bar_w, "λ2 = %.2f  (slow)", sc->l2,
                         cg2 / start2, sd2 / start2);
  y++;

  draw_kappa_footer(&y, x0, sc->l1, sc->l2);
}

/*
 * render_plot — orchestrator for the bowl region.  Order matters:
 *   1) contour rings (background)
 *   2) trails (CG green, SD red) — drawn on top of contours
 *   3) markers (X, Y, *) — drawn last so labels never get clipped
 *   4) axis labels in the corners
 */
static void render_plot(const Scene *sc, const View *v) {
  render_bowl_contours(sc, v);
  render_trail(sc->cg.hist, sc->cg.iter, CP_CG, v);
  render_trail(sc->sd.hist, sc->sd.iter, CP_SD, v);
  render_marker(sc->cg.x[0], sc->cg.x[1], 'X', '(', ')', " CG", CP_CG, v);
  render_marker(sc->sd.x[0], sc->sd.x[1], 'Y', '(', ')', " SD", CP_SD, v);
  render_marker(sc->x_true[0], sc->x_true[1], '*', '[', ']', " GOAL", CP_TARGET,
                v);
  render_axis_labels(v);
}

/* narrator_divider — '-' line separating the plot from the narrator. */
static void narrator_divider(int hud, int cols) {
  attron(COLOR_PAIR(CP_LABEL));
  for (int c = 0; c < cols; c++)
    mvaddch(hud, c, '-');
  attroff(COLOR_PAIR(CP_LABEL));
}

/*
 * narrator_step_bars — row 1: live progress for both solvers.
 * Each racer gets a label, a how-far-along bar (based on how wrong it
 * still is), and a "DONE" / "..." status.
 */
static void narrator_step_bars(int row, int cols, const Scene *sc) {
  float cg_frac = 1.0f - vnorm(sc->cg.r) / (sc->cg.res[0] + 1e-12f);
  float sd_frac = 1.0f - vnorm(sc->sd.r) / (sc->sd.res[0] + 1e-12f);

  int bar_w = (cols - 30) / 2 - 12;
  if (bar_w < 8)
    bar_w = 8;

  attron(COLOR_PAIR(CP_CG) | A_BOLD);
  mvprintw(row, 1, " CG %d/%d ", sc->cg.iter, MAX_ITER_CG - 1);
  attroff(COLOR_PAIR(CP_CG) | A_BOLD);
  render_bar(row, 12, bar_w, cg_frac, CP_CG);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(row, 12 + bar_w + 1, "%s", sc->cg.converged ? "DONE  " : "      ");
  attroff(COLOR_PAIR(CP_HUD));

  int sd_x = 12 + bar_w + 9;
  attron(COLOR_PAIR(CP_SD) | A_BOLD);
  mvprintw(row, sd_x, " SD %d ", sc->sd.iter);
  attroff(COLOR_PAIR(CP_SD) | A_BOLD);
  render_bar(row, sd_x + 9, bar_w, sd_frac, CP_SD);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(row, sd_x + 9 + bar_w + 1, "%s", sc->sd.converged ? "DONE" : "...");
  attroff(COLOR_PAIR(CP_HUD));
}

/* narrator_legend_glyph — one coloured glyph + description pair. */
static void narrator_legend_glyph(int row, int col, const char *glyph,
                                  int glyph_pair, const char *desc) {
  attron(COLOR_PAIR(glyph_pair) | A_BOLD);
  mvprintw(row, col, "%s", glyph);
  attroff(COLOR_PAIR(glyph_pair) | A_BOLD);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(row, col + (int)strlen(glyph), "%s", desc);
  attroff(COLOR_PAIR(CP_HUD));
}

/* narrator_legend — row 2: maps coloured glyphs on the plot to words. */
static void narrator_legend(int row) {
  narrator_legend_glyph(row, 1, "(X)", CP_CG, " CG path        ");
  narrator_legend_glyph(row, 21, "(Y)", CP_SD, " SD path        ");
  narrator_legend_glyph(row, 41, "[*]", CP_TARGET, " target x*      ");
  narrator_legend_glyph(row, 61, "...", CP_LABEL, " contour ring");
}

/*
 * narrator_explanations — rows 4-6: plain-English colour-coded story.
 * GREEN (CG) gets two lines for its A-conjugacy story; RED (SD) gets
 * one line on the right-angle zig-zag.
 */
static void narrator_explanations(int hud) {
  attron(COLOR_PAIR(CP_CG) | A_BOLD);
  mvprintw(hud + 4, 1, " GREEN ");
  attroff(COLOR_PAIR(CP_CG) | A_BOLD);
  attron(COLOR_PAIR(CP_EXPLAIN));
  mvprintw(hud + 4, 9,
           "CG picks each direction A-conjugate to all previous ones — never");
  mvprintw(
      hud + 5, 9,
      "re-walks a dimension.  Done in exactly N = 2 steps.  Watch it cut.");
  attroff(COLOR_PAIR(CP_EXPLAIN));

  attron(COLOR_PAIR(CP_SD) | A_BOLD);
  mvprintw(hud + 6, 1, " RED   ");
  attroff(COLOR_PAIR(CP_SD) | A_BOLD);
  attron(COLOR_PAIR(CP_EXPLAIN));
  mvprintw(hud + 6, 9,
           "SD follows the gradient.  Each step lands perpendicular to the");
  attroff(COLOR_PAIR(CP_EXPLAIN));
}

/*
 * render_narrator — six rows of plain-English explanation under the plot.
 *
 *   row 0 : divider
 *   row 1 : convergence bars + step counts        (narrator_step_bars)
 *   row 2 : coloured legend strip                 (narrator_legend)
 *   row 4 : CG green — "what makes CG fast"       (narrator_explanations)
 *   row 5 : (continuation)
 *   row 6 : SD red — "why SD struggles"
 *
 * Row 7 (rows-1) is the canonical bottom HUD, drawn elsewhere.
 */
static void render_narrator(const Scene *sc, int rows, int cols) {
  int hud = rows - 8;
  narrator_divider(hud, cols);
  narrator_step_bars(hud + 1, cols, sc);
  narrator_legend(hud + 2);
  narrator_explanations(hud);
}

/*
 * draw_hud_top — canonical top bar.  Title (left) + live status (right).
 *   row 0 left  CP_HEADER + bold  static title
 *   row 0 right CP_TOP    + bold  CG step / SD step / theme / state
 *   row 1       CP_LABEL          '-' divider
 */
static void draw_hud_top(const Scene *sc, int cols, int sim_fps) {
  attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
  mvprintw(0, 0, " CG vs Steepest Descent  [N=%d] ", N);
  attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

  char buf[200];
  const char *state = (sc->cg.converged && sc->sd.converged) ? "BOTH DONE"
                      : sc->cg.converged ? "CG DONE — SD GOING"
                      : sc->paused       ? "PAUSED "
                                         : "running";
  snprintf(buf, sizeof buf, " CG:%d/%d  SD:%d  theme:%s  speed:%d/s  %s ",
           sc->cg.iter, MAX_ITER_CG - 1, sc->sd.iter, k_themes[sc->theme].name,
           sim_fps, state);
  int len = (int)strlen(buf);
  int col = cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_TOP) | A_BOLD);
  mvaddnstr(0, col, buf, cols);
  attroff(COLOR_PAIR(CP_TOP) | A_BOLD);

  attron(COLOR_PAIR(CP_LABEL));
  for (int c = 0; c < cols; c++)
    mvaddch(1, c, '-');
  attroff(COLOR_PAIR(CP_LABEL));
}

static void draw_hud_bottom(int cols, int rows) {
  const char *hint_full =
      " q:quit  spc:pause  r:new system  t/T:theme  +/-:speed ";
  const char *hint_short = " q:quit  spc:pause  r:new  t:theme  +/-:speed ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §9 screen + app ── */

typedef struct {
  int cols, rows;
} Screen;

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void cleanup(void) { endwin(); }
static void on_exit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}

static void screen_init(Screen *sc) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(const App *app) {
  int rows = app->screen.rows;
  int cols = app->screen.cols;
  erase();
  draw_hud_top(&app->scene, cols, app->sim_fps);
  View v = compute_view(&app->scene, rows, cols);
  render_plot(&app->scene, &v);
  render_eigen_panel(&app->scene, rows, cols);
  render_narrator(&app->scene, rows, cols);
  draw_hud_bottom(cols, rows);
  wnoutrefresh(stdscr);
  doupdate();
}

static bool app_handle_key(App *app, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
    return false;
  case ' ':
    app->scene.paused = !app->scene.paused;
    break;
  case 'r':
  case 'R':
    app->scene.seed += 99991;
    scene_reset(&app->scene);
    break;
  case 't':
    app->scene.theme = (app->scene.theme + 1) % N_THEMES;
    theme_apply(app->scene.theme);
    break;
  case 'T':
    app->scene.theme = (app->scene.theme + N_THEMES - 1) % N_THEMES;
    theme_apply(app->scene.theme);
    break;
  case '+':
  case '=':
    if (app->sim_fps < SIM_FPS_MAX)
      app->sim_fps += 6;
    break;
  case '-':
  case '_':
    if (app->sim_fps > SIM_FPS_MIN)
      app->sim_fps -= 6;
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
  app->need_resize = 0;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene);
      sim_accum -= tick_ns;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(app);

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  return 0;
}
