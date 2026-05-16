/*  conjugate_gradient_linear_solver.c
 *
 *  Conjugate Gradient (CG) vs Steepest Descent (SD) — side by side.
 *
 *  Both algorithms solve the same SPD system Ax = b by walking to the
 *  bottom of the energy bowl f(x) = (1/2) x^T A x - b^T x.  CG provably
 *  converges in exactly N steps.  SD chases the gradient and zig-zags
 *  forever.  Drawing both paths on one bowl makes the difference
 *  immediately legible.
 *
 *  N = 2 — the bowl on screen IS the actual energy surface.  No
 *  projection trickery; what you see is what the algorithms see.
 *
 *  Layout:
 *    row 0           canonical top HUD (title + status)
 *    row 1           divider
 *    rows 2..N-9     iso-contour bowl (left) + eigenmode panel (right)
 *    rows N-8..N-2   narrator (7 rows)
 *    row N-1         canonical bottom HUD (actions)
 *
 *  Themes (10, cycle with t/T): drive the iso-contour ramp only.
 *  CG green / SD red / target gold / eigen-bar cyan stay fixed across
 *  every theme so the story stays readable.
 *
 *    Matrix, Fire, Oceanic, Neon, Mono, Ice, Nova, Forest, Desert, Eclipse
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/conjugate_gradient_linear_solver.c \
 *        -o cg_solver -lncurses -lm
 *
 *  Keys:  SPACE  pause / resume
 *         r      new random SPD system
 *         t / T  next / previous theme
 *         + / -  speed up / slow down
 *         q      quit
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * The problem    : Solve Ax = b where A is symmetric positive definite.
 *                  Equivalent to minimising the quadratic energy
 *                      f(x) = (1/2) x'Ax  -  b'x
 *                  whose gradient grad f(x) = Ax - b vanishes exactly
 *                  when Ax = b.  So "solve a linear system" =
 *                  "walk to the bottom of a paraboloid bowl".
 *
 * Algorithm (CG) : Hestenes & Stiefel 1952.
 *                    r0  = b - A x0     (initial residual)
 *                    p0  = r0           (first search direction)
 *                    for k = 0, 1, ... :
 *                        alpha = (r·r) / (p·A·p)         ← optimal step
 *                        x_new = x + alpha · p
 *                        r_new = r - alpha · A·p          (recurrence)
 *                        beta  = (r_new · r_new)/(r · r)
 *                        p_new = r_new + beta · p
 *                  Cost: one mat-vec (A·p) per step + a few dot products.
 *                  Stops in <= N iterations (exact arithmetic).
 *
 * Algorithm (SD) : Cauchy 1847 steepest descent.
 *                    alpha = (r·r)/(r·A·r)              ← optimal step
 *                    x_new = x + alpha · r
 *                    r_new = r - alpha · A·r
 *                  Each new gradient r_new is orthogonal to the OLD r
 *                  (r_new · r = 0 by construction), so on an elongated
 *                  bowl the iterates bounce between the long and short
 *                  axes — zig-zag forever.
 *
 * Why CG wins    : A-conjugacy.  CG's directions satisfy
 *                      p_i · A · p_j = 0   for i != j.
 *                  Each step lands on a "ridge" never used before, so
 *                  all N dimensions get cleaned up exactly once.  No
 *                  re-walked direction → no zig-zag.
 *
 * Spectral view  : Write the error e_k = x_k - x* in A's eigenbasis.
 *                  In that basis the problem decouples — each eigenmode
 *                  decays independently at a rate set by the eigenvalue
 *                  ratio.  Convergence speed depends on the condition
 *                  number  kappa(A) = lambda_max / lambda_min :
 *                    SD: error shrinks by factor (kappa - 1)/(kappa + 1)
 *                        per step — terrible when kappa is large.
 *                    CG: bound is sqrt(kappa) instead of kappa — much
 *                        better, and N-step exact termination.
 *                  The right-hand "EIGENMODES" panel projects the error
 *                  onto v1, v2 so the viewer SEES which mode dies first.
 *
 * Why N = 2      : With two coordinates the bowl on screen IS the actual
 *                  energy surface — no projection lie.  CG provably
 *                  terminates in 2 steps, which is the dramatic punch.
 *                  Higher N would force a misleading 2D slice through
 *                  N-D motion (the prior version of this demo did that,
 *                  and the walk on screen never matched the math).
 *
 * Rendering      : Iso-energy contour rings — quantise f into N levels,
 *                  mark a cell only if a 4-neighbour has a different
 *                  level (cheap "marching squares" cousin).  The result
 *                  reads as a topographic map of the bowl instead of a
 *                  density character fill.
 *                  Trails are DDA-style: dotted line between consecutive
 *                  iterates + an `o` bullet at each one, so the walk
 *                  reads as a path, not scattered marks.
 *                  CG green / SD red / target gold stay fixed across
 *                  every theme so the story stays legible.
 *
 * References     :
 *   • Hestenes, M. R. & Stiefel, E. (1952) "Methods of Conjugate
 *     Gradients for Solving Linear Systems", J. Res. NBS 49(6), 409-436.
 *     — The original paper.  CG was invented here.  Direct, terse.
 *
 *   • Shewchuk, J. R. (1994) "An Introduction to the Conjugate Gradient
 *     Method Without the Agonizing Pain", CMU CS-94-125 (tech report).
 *     — By far the best pedagogical introduction.  Geometric intuition
 *       (ellipses, A-conjugacy explained pictorially), steepest descent
 *       vs CG comparison, every formula derived from first principles.
 *       Read this BEFORE anything else if CG is new.
 *
 *   • Saad, Y. (2003) "Iterative Methods for Sparse Linear Systems"
 *     (2nd ed.), SIAM.
 *     — The reference text for Krylov subspace methods.  Chapter 6
 *       covers CG in depth; chapter 9 covers preconditioning, which is
 *       what makes CG work on real-world ill-conditioned systems.
 *
 *   • Trefethen, L. N. & Bau, D. (1997) "Numerical Linear Algebra", SIAM.
 *     — Clean modern textbook.  Lecture 38 = CG with the spectral /
 *       polynomial-residual perspective.  Lectures 24-28 cover the
 *       eigenvalue analysis this demo's eigen-panel depends on.
 *
 *   • Bresenham, J. E. (1965) "Algorithm for computer control of a
 *     digital plotter", IBM Systems Journal 4(1), 25-30.
 *     — The DDA segment rasteriser our trail rendering descends from.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── §1 config ──────────────────────────────────────────────────── */

#define N                 2          /* honest 2D — no projection lie  */
#define MAX_ITER_CG       (N + 2)    /* CG converges in <= N steps     */
#define MAX_ITER_SD       80         /* SD bound (effectively forever) */
#define HISTORY_LEN_CG    (MAX_ITER_CG + 2)
#define HISTORY_LEN_SD    (MAX_ITER_SD + 2)

#define SIM_FPS_DEFAULT   30         /* render+tick rate (Hz)          */
#define SIM_FPS_MIN       6
#define SIM_FPS_MAX       120
#define CG_STEP_TICKS     60         /* one CG step every 2.0s @ 30 Hz */
#define SD_STEP_TICKS     20         /* one SD step every 0.67s (3x faster) */
#define HOLD_TICKS        90         /* 3s pause after both converge   */
#define NS_PER_SEC        1000000000LL
#define NS_PER_MS         1000000LL
#define TICK_NS(f)        (NS_PER_SEC / (f))

#define RES_NORM_TOL      1e-6f
#define EIGEN_PANEL_W     24         /* right panel reservation (cols) */
#define N_CONTOURS        9          /* iso-energy ring count          */
#define N_THEMES          10

/* ── §2 clock ───────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — themes + fixed marker chrome ────────────────────── */

/*
 * Theme-driven roles (change with the active palette):
 *   CP_E_LOW..CP_E_PEAK   four-tier iso-energy contour ramp
 *
 * Fixed roles (theme-independent — green/red/gold stay readable):
 *   CP_CG       bright green — Conjugate Gradient trail and (X) marker
 *   CP_SD       bright red   — Steepest Descent  trail and (Y) marker
 *   CP_TARGET   bright gold  — true solution [*] marker
 *   CP_EIGEN    bright cyan  — eigen-panel labels
 *
 * Canonical chrome (CLAUDE.md):
 *   CP_HUD      light grey         — overlay body text
 *   CP_HEADER   cyan + bold        — section titles
 *   CP_LABEL    medium grey        — axis labels / dividers
 *   CP_EXPLAIN  yellow             — narrator body text
 *   CP_TOP      bright yellow+bold — top status bar
 *   CP_HINT     bright cyan+bold   — bottom action bar
 */
enum {
    CP_DEFAULT = 0,
    CP_E_LOW, CP_E_MID, CP_E_HIGH, CP_E_PEAK,
    CP_CG, CP_SD, CP_TARGET, CP_EIGEN,
    CP_HUD, CP_HEADER, CP_LABEL, CP_EXPLAIN,
    CP_TOP, CP_HINT,
    CP_COUNT
};

/* Theme — one palette tuned for iso-contour ring rendering. */
typedef struct {
    const char *name;
    short ramp[4];
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0]  ramp[1]  ramp[2]  ramp[3]                       */
    { "Matrix",   {  28,     34,      40,      46  } },  /* cyber green       */
    { "Fire",     { 130,    208,     202,     196  } },  /* warm → red        */
    { "Oceanic",  {  24,     31,      39,      51  } },  /* teal → cyan       */
    { "Neon",     { 129,    165,     201,     213  } },  /* purple → pink     */
    { "Mono",     { 240,    247,     250,     255  } },  /* grayscale         */
    { "Ice",      { 153,    117,     159,     195  } },  /* light blues       */
    { "Nova",     { 129,    141,     177,     213  } },  /* stellar           */
    { "Forest",   {  58,    100,     142,     190  } },  /* leaves / bark     */
    { "Desert",   { 130,    178,     214,     220  } },  /* sand / gold       */
    { "Eclipse",  { 240,    244,     124,     196  } },  /* gray + red flare  */
};

static void theme_apply(int t) {
    const Theme *th = &k_themes[t % N_THEMES];
    if (COLORS >= 256) {
        init_pair(CP_E_LOW,    th->ramp[0],   -1);
        init_pair(CP_E_MID,    th->ramp[1],   -1);
        init_pair(CP_E_HIGH,   th->ramp[2],   -1);
        init_pair(CP_E_PEAK,   th->ramp[3],   -1);
        init_pair(CP_CG,        46,  -1);
        init_pair(CP_SD,       196,  -1);
        init_pair(CP_TARGET,   226,  -1);
        init_pair(CP_EIGEN,     51,  -1);
        init_pair(CP_HUD,      252,  -1);
        init_pair(CP_HEADER,    51,  -1);
        init_pair(CP_LABEL,    244,  -1);
        init_pair(CP_EXPLAIN,  227,  -1);
        init_pair(CP_TOP,      226,  -1);
        init_pair(CP_HINT,      51,  -1);
    } else {
        init_pair(CP_E_LOW,    COLOR_BLUE,    -1);
        init_pair(CP_E_MID,    COLOR_CYAN,    -1);
        init_pair(CP_E_HIGH,   COLOR_YELLOW,  -1);
        init_pair(CP_E_PEAK,   COLOR_RED,     -1);
        init_pair(CP_CG,       COLOR_GREEN,   -1);
        init_pair(CP_SD,       COLOR_RED,     -1);
        init_pair(CP_TARGET,   COLOR_YELLOW,  -1);
        init_pair(CP_EIGEN,    COLOR_CYAN,    -1);
        init_pair(CP_HUD,      COLOR_WHITE,   -1);
        init_pair(CP_HEADER,   COLOR_CYAN,    -1);
        init_pair(CP_LABEL,    COLOR_WHITE,   -1);
        init_pair(CP_EXPLAIN,  COLOR_YELLOW,  -1);
        init_pair(CP_TOP,      COLOR_YELLOW,  -1);
        init_pair(CP_HINT,     COLOR_CYAN,    -1);
    }
}

static void color_init(void) {
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ── §4 linear algebra (N=2 dense) ──────────────────────────────── */

typedef float Mat[N * N];
typedef float Vec[N];

static void mat_vec_mul(const float *A, const float *x, float *out) {
    for (int i = 0; i < N; i++) {
        float s = 0.0f;
        for (int j = 0; j < N; j++) s += A[i * N + j] * x[j];
        out[i] = s;
    }
}

static float vdot(const float *a, const float *b) {
    float s = 0.0f;
    for (int i = 0; i < N; i++) s += a[i] * b[i];
    return s;
}

static float vnorm(const float *v) { return sqrtf(vdot(v, v)); }

/* out = a + alpha * b  (BLAS daxpy).  Aliasing out==a is safe. */
static void vaxpy(float *out, const float *a, float alpha, const float *b) {
    for (int i = 0; i < N; i++) out[i] = a[i] + alpha * b[i];
}

/*
 * solve_secular_eq_2x2 — characteristic polynomial of symmetric 2x2.
 *
 * Roots of  det(A - λI) = λ² - tr·λ + det = 0   with
 *   tr  = a + c
 *   det = ac - b²
 *   disc = sqrt((tr/2)² - det)        (real, since A is symmetric)
 *
 * Returns the two eigenvalues sorted so *out_l1 >= *out_l2.
 */
static void solve_secular_eq_2x2(float a, float b, float c,
                                 float *out_l1, float *out_l2)
{
    float tr   = a + c;
    float det  = a * c - b * b;
    float disc = sqrtf(fmaxf(tr * tr * 0.25f - det, 0.0f));
    *out_l1 = tr * 0.5f + disc;
    *out_l2 = tr * 0.5f - disc;
}

/*
 * eigenvector_for_value_2x2 — unit eigenvector for a known eigenvalue.
 *
 * From  A v = λ v   we get   (a - λ) v[0] + b v[1] = 0
 * → v ∝ (b, λ - a).  Falls back to an axis-aligned unit vector when
 * b ≈ 0 (A already diagonal — eigenvectors are just the standard basis).
 */
static void eigenvector_for_value_2x2(float a, float b, float lambda,
                                      Vec out)
{
    if (fabsf(b) > 1e-10f) {
        out[0] = b;
        out[1] = lambda - a;
    } else {
        /* A diagonal: λ matches either A[0,0]=a or A[1,1]=c.  Pick the
         * matching standard-basis vector.                                */
        if (fabsf(lambda - a) < 1e-10f) {
            out[0] = 1.0f; out[1] = 0.0f;
        } else {
            out[0] = 0.0f; out[1] = 1.0f;
        }
    }
    float n = sqrtf(out[0] * out[0] + out[1] * out[1]);
    if (n > 1e-10f) { out[0] /= n; out[1] /= n; }
}

/*
 * perpendicular_2d — 90° rotation in the plane.
 * For an orthonormal basis in 2D, v2 ⊥ v1 = (-v1.y, v1.x).
 */
static void perpendicular_2d(const Vec in, Vec out)
{
    out[0] = -in[1];
    out[1] =  in[0];
}

/*
 * eigen2 — closed-form eigendecomposition for symmetric 2x2.
 *
 * Reads like the math:
 *   1) solve the characteristic polynomial          → eigenvalues
 *   2) lift one eigenvalue to its eigenvector       → v1
 *   3) take the perpendicular                       → v2
 *
 * The eigen-panel uses (l1, l2, v1, v2) to project the error onto each
 * axis so the viewer SEES which mode each algorithm kills first.
 */
static void eigen2(const Mat A, float *out_l1, float *out_l2,
                   Vec out_v1, Vec out_v2)
{
    float a = A[0], b = A[1], c = A[3];
    solve_secular_eq_2x2(a, b, c, out_l1, out_l2);
    eigenvector_for_value_2x2(a, b, *out_l1, out_v1);
    perpendicular_2d(out_v1, out_v2);
}

/* ── §5 SPD generation + reference solution ─────────────────────── */

/*
 * Build a 2x2 SPD matrix with controlled eigenvalue separation so the
 * bowl is reliably elongated (otherwise CG vs SD look identical).
 *
 * Procedure: pick two random eigenvalues with a 3-6× ratio, pick a
 * random rotation θ, set A = R diag(λ1, λ2) Rᵀ.
 */
static void build_spd(Mat A, unsigned seed)
{
    srand(seed);
    float l1 = 0.6f + (float)(rand() % 100) / 200.0f;   /* 0.6 .. 1.1 */
    float l2 = 2.5f + (float)(rand() % 100) /  50.0f;   /* 2.5 .. 4.5 */
    float theta = (float)(rand() % 360) * (float)M_PI / 180.0f;
    float cs = cosf(theta), sn = sinf(theta);

    float a = l1 * cs * cs + l2 * sn * sn;
    float b = (l1 - l2) * cs * sn;
    float c = l1 * sn * sn + l2 * cs * cs;

    A[0] = a;  A[1] = b;
    A[2] = b;  A[3] = c;
}

/*
 * cholesky_decompose — A = L · Lᵀ for SPD A (lower-triangular L).
 *
 * Standard "outer product" form: for each row i, fill L[i][0..i].
 *   diagonal:     L[i][i] = sqrt(A[i][i] - Σ_{k<i} L[i][k]²)
 *   below-diag:   L[i][j] = (A[i][j] - Σ_{k<j} L[i][k]·L[j][k]) / L[j][j]
 *
 * Reference: Golub & Van Loan, Matrix Computations, §4.2.
 */
static void cholesky_decompose(const float *A, float *L)
{
    memset(L, 0, sizeof(float) * N * N);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= i; j++) {
            float s = A[i * N + j];
            for (int k = 0; k < j; k++) s -= L[i * N + k] * L[j * N + k];
            if (i == j) {
                L[i * N + j] = (s > 0.0f) ? sqrtf(s) : 1e-10f;
            } else {
                L[i * N + j] = s / L[j * N + j];
            }
        }
    }
}

/*
 * forward_substitute — solve L · y = b for y (L is lower-triangular).
 * Sweep from row 0 down:  y[i] = (b[i] − Σ_{k<i} L[i][k]·y[k]) / L[i][i]
 */
static void forward_substitute(const float *L, const float *b, float *y)
{
    for (int i = 0; i < N; i++) {
        float s = b[i];
        for (int k = 0; k < i; k++) s -= L[i * N + k] * y[k];
        y[i] = s / L[i * N + i];
    }
}

/*
 * back_substitute_transpose — solve Lᵀ · x = y for x.
 * Sweep from the bottom up:  x[i] = (y[i] − Σ_{k>i} L[k][i]·x[k]) / L[i][i]
 * (the entries above the diagonal of Lᵀ come from below the diagonal of L)
 */
static void back_substitute_transpose(const float *L, const float *y, float *x)
{
    for (int i = N - 1; i >= 0; i--) {
        float s = y[i];
        for (int k = i + 1; k < N; k++) s -= L[k * N + i] * x[k];
        x[i] = s / L[i * N + i];
    }
}

/*
 * cholesky_solve — direct solver for x* = A⁻¹ b, A symmetric positive definite.
 *
 *   1) decompose A = L Lᵀ        (Cholesky)
 *   2) forward-solve L · y = b
 *   3) back-solve   Lᵀ · x = y
 *
 * Drawn on screen as [*] — the "ground truth" the iterative solvers
 * race toward.  Used only at reset; the hot loop never calls this.
 */
static void cholesky_solve(const float *A, const float *b, float *x)
{
    float L[N * N];
    Vec   y;
    cholesky_decompose(A, L);
    forward_substitute(L, b, y);
    back_substitute_transpose(L, y, x);
}

/* ── §6 solvers — CG and SD ─────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────
 * CGSolver — one running instance of Conjugate Gradient.
 *
 * Mirrors the Hestenes-Stiefel 1952 iteration field-for-field — every
 * member here corresponds to a named quantity in the textbook write-up,
 * so the struct doubles as a glossary of the algorithm:
 *
 *     x_k       current iterate (the answer so far)
 *     r_k       residual b − A x_k = − grad f(x_k)
 *     p_k       search direction (A-conjugate to all previous)
 *     A p_k     cached mat-vec product (avoid redundant work)
 *
 *     alpha_k   step size,   = (r·r) / (p·A·p)         optimal along p
 *     beta_k    blend coeff, = ‖r_new‖² / ‖r_old‖²     Fletcher-Reeves
 *     rr        cached r·r   (saves one dot product per step)
 *
 * Members are grouped by access pattern, NOT by type:
 *
 *   iteration state  : x, r, p, Ap, rr, alpha, beta
 *       Hot path.  Read+written every cg_step.  Pixel-space N-vectors.
 *
 *   termination      : iter, converged
 *       Loop control.  cg_step writes both; everything else only reads.
 *
 *   render snapshot  : hist[], res[]
 *       Write-once per CG step (latest position + residual norm).  Read
 *       every render frame by the trail / narrator code.  hist[k][0..1]
 *       is the 2D position of iterate k; res[k] is ‖r_k‖.  Sized to
 *       hold MAX_ITER_CG+2 entries — more than enough for N=2.
 *
 * References: Hestenes & Stiefel 1952 (original) — section 3 defines
 *             every variable above.
 *             Shewchuk 1994 — §B7 has the same recurrence with picture
 *             intuition for what each variable does geometrically.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ iteration state — read+write every cg_step ─────────────────── */
    Vec   x;        /* current iterate x_k, the answer-in-progress     */
    Vec   r;        /* residual r_k = b − A x_k                        */
    Vec   p;        /* search direction p_k (A-conjugate to past p_j)  */
    Vec   Ap;       /* cached A · p_k (saves a mat-vec in α denom)     */
    float rr;       /* cached r·r (saves a dot product in β numerator) */
    float alpha;    /* step size for the LAST step taken               */
    float beta;     /* direction blend coeff for the LAST step taken   */

    /* ─ termination — set by cg_step ───────────────────────────────── */
    int   iter;     /* count of completed steps, 0..MAX_ITER_CG-1      */
    bool  converged;/* true → cg_step is a no-op afterwards            */

    /* ─ render snapshot — written 1× per step, read 30× / sec ──────── */
    float hist[HISTORY_LEN_CG][2];  /* (x[0], x[1]) at each iterate     */
    float res [HISTORY_LEN_CG];     /* ‖r_k‖ for the narrator's "was X"*/
} CGSolver;

/* ─────────────────────────────────────────────────────────────────────
 * SDSolver — one running instance of Steepest Descent (Cauchy 1847).
 *
 * The comparison foil to CG.  SD's update is much simpler than CG's:
 *
 *     alpha = (r · r) / (r · A · r)        optimal step along r
 *     x_new = x + alpha · r
 *     r_new = r - alpha · A · r
 *
 * No search direction state and no β: the search direction IS just r
 * each iteration.  That simplicity is also SD's curse — consecutive
 * gradients are exactly orthogonal (r_new · r = 0), so on an elongated
 * elliptic bowl the iterates zig-zag between the long and short axes
 * and inch forward by tiny amounts.  Convergence rate is governed by
 * ((κ − 1)/(κ + 1))^k where κ = λ_max / λ_min.
 *
 * Same field grouping as CGSolver so the two are 1-to-1 readable:
 *
 *   iteration state  : x, r, alpha
 *   termination      : iter, converged
 *   render snapshot  : hist[], res[]   (sized MAX_ITER_SD+2 — SD takes
 *                                       dozens of steps where CG takes 2)
 *
 * References: Cauchy 1847 (the very first descent method paper).
 *             Trefethen & Bau 1997 — Lecture 38 §38.3 shows the
 *             zig-zag pattern and derives the (κ−1)/(κ+1) bound.
 *             Shewchuk 1994 — §6 contrasts SD vs CG geometrically.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ iteration state — read+write every sd_step ─────────────────── */
    Vec   x;        /* current iterate                                 */
    Vec   r;        /* residual (also the negative gradient)           */
    float alpha;    /* step size for the LAST step taken               */

    /* ─ termination ───────────────────────────────────────────────── */
    int   iter;     /* count of completed steps, 0..MAX_ITER_SD-1      */
    bool  converged;/* true → SD's tolerance reached or step bound hit */

    /* ─ render snapshot — written 1× per step ──────────────────────── */
    float hist[HISTORY_LEN_SD][2];  /* zig-zag trail in 2D              */
    float res [HISTORY_LEN_SD];     /* per-step residual norm           */
} SDSolver;

/* cg_reset — x0 = 0, r0 = b - Ax0 = b, p0 = r0 */
static void cg_reset(CGSolver *s, const Mat A, const Vec b)
{
    memset(s->x, 0, sizeof s->x);
    memcpy(s->r, b, sizeof s->r);
    memcpy(s->p, b, sizeof s->p);
    mat_vec_mul(A, s->p, s->Ap);
    s->rr        = vdot(s->r, s->r);
    s->alpha     = 0.0f;
    s->beta      = 0.0f;
    s->iter      = 0;
    s->converged = false;
    s->hist[0][0] = s->x[0];
    s->hist[0][1] = s->x[1];
    s->res [0]    = vnorm(s->r);
}

/*
 * cg_should_stop — early-exit predicates.
 *
 * Three reasons to skip a CG step:
 *   1. already converged
 *   2. step count hit MAX_ITER_CG (safety bound)
 *   3. residual already within tolerance
 *
 * Mutates s->converged when (2) or (3) trip so the caller's next visit
 * also short-circuits.
 */
static bool cg_should_stop(CGSolver *s)
{
    if (s->converged) return true;
    if (s->iter >= MAX_ITER_CG - 1) { s->converged = true; return true; }
    if (vnorm(s->r) < RES_NORM_TOL) { s->converged = true; return true; }
    return false;
}

/*
 * cg_optimal_step_size — α_k = (r·r) / (p·A·p).
 *
 * The α that exactly minimises f(x + α·p) along the search direction p.
 * Derivation: ∂f/∂α = pᵀ(A x − b) + α pᵀA p = -pᵀr + α pᵀA p,
 * setting to zero gives α = pᵀr / pᵀA p = rᵀr / pᵀA p (using pᵀr = rᵀr
 * which holds when p is generated by the CG recurrence).
 *
 * Returns the step size.  Sets s->converged if pᵀA p is degenerate
 * (numerically zero — the search direction is in A's null space, which
 * shouldn't happen for SPD A but the guard is cheap insurance).
 */
static float cg_optimal_step_size(CGSolver *s)
{
    float pAp = vdot(s->p, s->Ap);
    if (fabsf(pAp) < 1e-14f) { s->converged = true; return 0.0f; }
    return s->rr / pAp;
}

/*
 * cg_walk_and_update_residual — apply the position and residual updates.
 *
 *   x_{k+1} = x_k + α p_k          (walk along search direction)
 *   r_{k+1} = r_k − α A p_k        (residual recurrence — no extra mat-vec)
 *
 * Refreshes the cached rr = r·r so the Fletcher-Reeves β can read it.
 */
static void cg_walk_and_update_residual(CGSolver *s, float alpha)
{
    vaxpy(s->x, s->x,  alpha, s->p);
    vaxpy(s->r, s->r, -alpha, s->Ap);
    s->rr = vdot(s->r, s->r);
}

/*
 * cg_new_direction_fletcher_reeves — pick the next A-conjugate p.
 *
 *   β_k    = ‖r_{k+1}‖² / ‖r_k‖²          (Fletcher-Reeves form)
 *   p_{k+1} = r_{k+1} + β_k · p_k          (A-conjugate to all past p_j)
 *
 * Also refreshes the cached A·p for the NEXT step's α denominator.
 */
static void cg_new_direction_fletcher_reeves(CGSolver *s, float rr_old,
                                             const Mat A)
{
    s->beta = s->rr / rr_old;
    vaxpy(s->p, s->r, s->beta, s->p);
    mat_vec_mul(A, s->p, s->Ap);
}

/* cg_record_step — bump iter counter and snapshot history for render. */
static void cg_record_step(CGSolver *s)
{
    s->iter++;
    if (s->iter < HISTORY_LEN_CG) {
        s->hist[s->iter][0] = s->x[0];
        s->hist[s->iter][1] = s->x[1];
        s->res [s->iter]    = vnorm(s->r);
    }
    if (vnorm(s->r) < RES_NORM_TOL) s->converged = true;
}

/*
 * cg_step — one CG iteration.  Reads like the textbook recipe:
 *
 *   1. step size:        α = rᵀr / pᵀA p
 *   2. position+residual: x ← x + α p,   r ← r − α A p
 *   3. new direction:     β = ‖r_new‖²/‖r_old‖²,  p ← r + β p,  cache A p
 *   4. record:            iter++, snapshot history, maybe converged
 */
static void cg_step(CGSolver *s, const Mat A)
{
    if (cg_should_stop(s)) return;

    float rr_old = s->rr;
    float alpha  = cg_optimal_step_size(s);
    if (s->converged) return;
    s->alpha = alpha;

    cg_walk_and_update_residual(s, alpha);
    cg_new_direction_fletcher_reeves(s, rr_old, A);
    cg_record_step(s);
}

/* sd_reset — same start state as CG so the comparison is fair. */
static void sd_reset(SDSolver *s, const Vec b)
{
    memset(s->x, 0, sizeof s->x);
    memcpy(s->r, b, sizeof s->r);
    s->alpha     = 0.0f;
    s->iter      = 0;
    s->converged = false;
    s->hist[0][0] = s->x[0];
    s->hist[0][1] = s->x[1];
    s->res [0]    = vnorm(s->r);
}

/* sd_should_stop — early-exit predicates, parallel to cg_should_stop. */
static bool sd_should_stop(SDSolver *s)
{
    if (s->converged) return true;
    if (s->iter >= MAX_ITER_SD - 1) { s->converged = true; return true; }
    if (vnorm(s->r) < RES_NORM_TOL) { s->converged = true; return true; }
    return false;
}

/*
 * sd_optimal_step_size — α = (r·r) / (r·A·r).
 *
 * The α that exactly minimises f(x + α·r) along the gradient r.  Same
 * line-search principle as CG, but the search direction IS r each step
 * (no orthogonalisation against previous directions — which is the
 * whole reason SD zig-zags).
 *
 * Sets s->converged if rᵀA r is degenerate.
 */
static float sd_optimal_step_size(SDSolver *s, const Vec Ar)
{
    float rr  = vdot(s->r, s->r);
    float rAr = vdot(s->r, Ar);
    if (rAr < 1e-14f) { s->converged = true; return 0.0f; }
    return rr / rAr;
}

/*
 * sd_walk_and_update_residual — apply the position and residual updates.
 *
 *   x_{k+1} = x_k + α r_k         (walk along the gradient)
 *   r_{k+1} = r_k − α A r_k        (recurrence — saves a mat-vec)
 *
 * Math fact: by construction r_{k+1} · r_k = 0.  That's why SD's
 * trajectory turns exactly 90° every step — and on an elongated bowl
 * that means perpetual zig-zag.
 */
static void sd_walk_and_update_residual(SDSolver *s, float alpha, const Vec Ar)
{
    vaxpy(s->x, s->x,  alpha, s->r);
    vaxpy(s->r, s->r, -alpha, Ar);
}

/* sd_record_step — bump iter counter and snapshot history for render. */
static void sd_record_step(SDSolver *s)
{
    s->iter++;
    if (s->iter < HISTORY_LEN_SD) {
        s->hist[s->iter][0] = s->x[0];
        s->hist[s->iter][1] = s->x[1];
        s->res [s->iter]    = vnorm(s->r);
    }
    if (vnorm(s->r) < RES_NORM_TOL) s->converged = true;
}

/*
 * sd_step — one Steepest Descent iteration.  Cauchy 1847 recipe:
 *
 *   1. cache A·r            (reused by both α denominator and recurrence)
 *   2. step size:           α = rᵀr / rᵀA r
 *   3. position+residual:   x ← x + α r,   r ← r − α A r
 *   4. record:              iter++, snapshot history, maybe converged
 */
static void sd_step(SDSolver *s, const Mat A)
{
    if (sd_should_stop(s)) return;

    Vec Ar;
    mat_vec_mul(A, s->r, Ar);

    float alpha = sd_optimal_step_size(s, Ar);
    if (s->converged) return;
    s->alpha = alpha;

    sd_walk_and_update_residual(s, alpha, Ar);
    sd_record_step(s);
}

/* ── §7 scene ───────────────────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────
 * Scene — the entire demo's state.
 *
 * One Scene IS the demo.  Both solvers, the problem definition (A, b),
 * the spectral analysis (eigenvalues + eigenvectors), the pacing clocks,
 * and the UI state all live here.  Pass &scene to anything that needs
 * anything; there's no shared mutable state outside.
 *
 * Members grouped by what TOUCHES them, per CLAUDE.md "locality" rule.
 * Simulation-side groups (problem / spectral / solvers / pacing) live
 * separately from UI state, and the render side reads but never writes.
 *
 *   problem definition  : A, b, x_true
 *       The linear system Ax = b and the reference answer.  Set once
 *       per scene_reset; read by both solvers every step and by the
 *       bowl renderer every frame.
 *
 *   spectral analysis   : l1, l2, v1, v2
 *       Eigendecomposition of A (computed once per reset by eigen2).
 *       Pure metadata — neither solver uses it; only the eigen-panel
 *       renderer reads it to project the error onto v1/v2.
 *
 *   solvers (sim)       : cg, sd
 *       The two algorithms racing on the same bowl.  Stepped by
 *       scene_tick at independent rates; render code reads their state.
 *
 *   tick pacing (sim)   : cg_acc, sd_acc, hold
 *       Per-tick counters that throttle solver stepping.  cg_step fires
 *       every CG_STEP_TICKS ticks (≈ 2 s); sd_step every SD_STEP_TICKS
 *       (≈ 0.67 s).  After both converge, hold counts up to HOLD_TICKS
 *       then triggers auto-restart with a new random A.
 *
 *   UI state            : paused, theme
 *       Written only by app_handle_key; read by scene_tick (paused) and
 *       rendering (theme).  No render-only fields live elsewhere on the
 *       Scene — the render pipeline takes &scene and never mutates.
 *
 *   restart driver      : seed
 *       The RNG seed for the NEXT call to scene_reset.  Bumped on `r`
 *       key and on auto-restart.
 *
 * Reference: Shewchuk 1994 §B treats the entire CG-vs-SD comparison
 *            as one geometric story on one bowl — that's the framing
 *            this scene exists to put on screen.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ problem definition — set once per reset, read everywhere ───── */
    Mat       A;       /* SPD matrix the solvers are working on        */
    Vec       b;       /* right-hand side (non-zero so x* != origin)   */
    Vec       x_true;  /* reference solution via Cholesky — drawn as [*]*/

    /* ─ spectral analysis — pure metadata, read only by eigen panel ── */
    float     l1, l2;  /* eigenvalues of A, l1 >= l2 (so kappa=l1/l2)  */
    Vec       v1, v2;  /* corresponding orthonormal eigenvectors        */

    /* ─ solvers (simulation, hot path) ─────────────────────────────── */
    CGSolver  cg;      /* Conjugate Gradient state                      */
    SDSolver  sd;      /* Steepest Descent  state                       */

    /* ─ tick pacing (simulation driver) ────────────────────────────── *
     * scene_tick increments cg_acc / sd_acc each call, fires the
     * corresponding solver step when the counter crosses its threshold.
     * Asymmetric thresholds give CG 1 step / 2 s and SD ~3 steps / 2 s
     * — exactly the rhythm where you watch CG sprint and SD limp.      */
    int       cg_acc;
    int       sd_acc;
    int       hold;    /* counts up to HOLD_TICKS post-converge, then  */
                       /* triggers auto-restart                         */

    /* ─ UI state — written only by app_handle_key ──────────────────── */
    bool      paused;  /* space: freezes scene_tick                     */
    int       theme;   /* t/T: index into k_themes[]                    */

    /* ─ restart driver ────────────────────────────────────────────── */
    unsigned  seed;    /* RNG seed for the NEXT scene_reset             */
} Scene;

static void scene_reset(Scene *sc)
{
    build_spd(sc->A, sc->seed);
    /* Non-zero RHS so x* isn't at the origin. */
    sc->b[0] = 1.4f;
    sc->b[1] = 0.7f;
    cholesky_solve(sc->A, sc->b, sc->x_true);
    eigen2(sc->A, &sc->l1, &sc->l2, sc->v1, sc->v2);

    cg_reset(&sc->cg, sc->A, sc->b);
    sd_reset(&sc->sd, sc->b);
    sc->cg_acc = 0;
    sc->sd_acc = 0;
    sc->hold   = 0;
}

static void scene_init(Scene *sc)
{
    memset(sc, 0, sizeof *sc);
    sc->seed   = (unsigned)time(NULL);
    sc->theme  = 0;
    sc->paused = false;
    scene_reset(sc);
    theme_apply(sc->theme);
}

/*
 * tick_post_convergence — what happens when both algorithms are done.
 * Hold the final tableau for HOLD_TICKS render frames so the user can
 * actually look at it, then bump the seed and reset for a new race.
 */
static void tick_post_convergence(Scene *sc)
{
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
static void pace_cg_step(Scene *sc)
{
    if (sc->cg.converged) return;
    if (++sc->cg_acc >= CG_STEP_TICKS) {
        cg_step(&sc->cg, sc->A);
        sc->cg_acc = 0;
    }
}

static void pace_sd_step(Scene *sc)
{
    if (sc->sd.converged) return;
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
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;

    if (sc->cg.converged && sc->sd.converged) {
        tick_post_convergence(sc);
        return;
    }
    pace_cg_step(sc);
    pace_sd_step(sc);
}

/* ── §8 rendering ───────────────────────────────────────────────── */

#define HDR_ROWS 2     /* row 0 status, row 1 divider */
#define FTR_ROWS 8     /* 7 rows narrator + 1 row action HUD */

/* ─────────────────────────────────────────────────────────────────────
 * View — the per-frame camera onto the (x1, x2) world.
 *
 * Pure RENDER-side metadata.  No simulation function touches it; no
 * Scene member depends on it.  Lifetime: one frame.  Created by
 * compute_view from the current Scene, then passed by const-ref to
 * every sub-renderer so they share one consistent projection.
 *
 * Why a struct: the bowl, the trails, the markers, and the axis labels
 * each independently need (plot_w, ph, xlo..yhi).  Computing the window
 * in each renderer would be duplicate work; passing six floats per call
 * is noisy.  One View bundles them.
 *
 * Two coordinate systems involved:
 *
 *   world space    — the actual (x1, x2) Lagrangian space where the
 *                    algorithms operate.  Iterates have positions like
 *                    (0.42, -0.18).  Axis range = [xlo, xhi] × [ylo, yhi].
 *
 *   cell space     — terminal character cells.  Top-left = (0, 0).
 *                    The bowl occupies columns 1..plot_w-2 and rows
 *                    HDR_ROWS..HDR_ROWS+ph-1.
 *
 * to_col / to_row do the affine map world → cell.  Y is inverted
 * (yhi maps to top row HDR_ROWS) to match the screen convention.
 *
 * The window auto-zooms each frame: compute_view starts at a small
 * range centred on x*, then expands to include every iterate of both
 * algorithms.  Lets SD's wandering trail stay in frame without losing
 * the bowl's structure when iterates are still near the origin.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ layout (cell space) — set once by compute_view ─────────────── */
    int   plot_w;        /* columns reserved for the bowl region        */
    int   ph;            /* bowl height in rows                         */

    /* ─ camera window (world space) ─────────────────────────────────── *
     * Affine map: cell c → world x = xlo + (xhi-xlo) · (c-1)/(plot_w-3)
     *             cell r → world y = yhi - (yhi-ylo) · r/(ph-1)
     * (yhi at the top row matches screen convention: low row = high y.) */
    float xlo, xhi;      /* visible x1 range                            */
    float ylo, yhi;      /* visible x2 range                            */
} View;

static int to_col(float v, float lo, float hi, int c0, int c1) {
    if (hi <= lo) return c0;
    float t = (v - lo) / (hi - lo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return c0 + (int)(t * (float)(c1 - c0));
}
static int to_row(float v, float lo, float hi, int r0, int r1) {
    if (hi <= lo) return r0;
    float t = (v - lo) / (hi - lo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return r1 - (int)(t * (float)(r1 - r0));
}

static float energy2d(const Scene *sc, float x0, float x1) {
    float a00 = sc->A[0], a01 = sc->A[1], a11 = sc->A[3];
    float q   = 0.5f * (a00 * x0 * x0
                       + 2.0f * a01 * x0 * x1
                       + a11 * x1 * x1);
    return q - sc->b[0] * x0 - sc->b[1] * x1;
}

/*
 * view_layout — assign cell-space dimensions to the View.
 * Bowl gets cols minus the eigen panel; height is rows minus HUD chrome.
 */
static void view_layout(int rows, int cols, int *out_plot_w, int *out_ph)
{
    int plot_w = cols - EIGEN_PANEL_W;
    if (plot_w < 30) plot_w = 30;
    int ph = rows - HDR_ROWS - FTR_ROWS;
    if (ph < 4) ph = 4;
    *out_plot_w = plot_w;
    *out_ph     = ph;
}

/*
 * expand_range_to_fit_trail — grow the half-width `range` so every
 * iterate in `hist[0..n_iter]` falls inside the [cx ± range] × [cy ± range]
 * square (with a 0.2 padding margin so trails don't kiss the edge).
 */
static void expand_range_to_fit_trail(const float (*hist)[2], int n_iter,
                                      float cx, float cy, float *range)
{
    for (int k = 0; k <= n_iter; k++) {
        float dx = fabsf(hist[k][0] - cx);
        float dy = fabsf(hist[k][1] - cy);
        if (dx + 0.20f > *range) *range = dx + 0.20f;
        if (dy + 0.20f > *range) *range = dy + 0.20f;
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
static View compute_view(const Scene *sc, int rows, int cols)
{
    View v;
    view_layout(rows, cols, &v.plot_w, &v.ph);

    float cx = sc->x_true[0];
    float cy = sc->x_true[1];
    float range = 0.5f;
    expand_range_to_fit_trail(sc->cg.hist, sc->cg.iter, cx, cy, &range);
    expand_range_to_fit_trail(sc->sd.hist, sc->sd.iter, cx, cy, &range);

    v.xlo = cx - range;  v.xhi = cx + range;
    v.ylo = cy - range;  v.yhi = cy + range;
    return v;
}

/*
 * cell_to_world — inverse of the View's affine map.
 * Maps a (column, row) inside the bowl region back to world coordinates.
 */
static void cell_to_world(const View *v, int c, int r,
                          float *out_xv, float *out_yv)
{
    *out_xv = v->xlo + (v->xhi - v->xlo) * (float)(c - 1)
                                          / (float)(v->plot_w - 3);
    *out_yv = v->yhi - (v->yhi - v->ylo) * (float)r
                                          / (float)(v->ph - 1);
}

/*
 * find_energy_window — pass 1 of the contour renderer.
 * Scan every cell to find the [emin, emax] energy range we'll quantise
 * into N_CONTOURS buckets.
 */
static void find_energy_window(const Scene *sc, const View *v,
                               float *out_emin, float *out_emax)
{
    float emin = 1e30f, emax = -1e30f;
    for (int r = 0; r < v->ph; r++) {
        for (int c = 1; c < v->plot_w - 1; c++) {
            float xv, yv;
            cell_to_world(v, c, r, &xv, &yv);
            float e = energy2d(sc, xv, yv);
            if (e < emin) emin = e;
            if (e > emax) emax = e;
        }
    }
    *out_emin = emin;
    *out_emax = emax;
}

/*
 * contour_classify_cell — does this cell sit on a contour line?
 *
 * Compare the quantised energy level of (c, r) against its RIGHT and
 * DOWN 4-neighbours.  If the level differs from either neighbour, the
 * iso-line passes through this cell:
 *
 *   horizontal change only → contour runs vertically here  → '|'
 *   vertical change only   → contour runs horizontally here → '-'
 *   both changed           → corner / curved bit            → '.'
 *
 * Returns true if the cell is on a contour; out_level and out_ch are
 * set accordingly.  Cells inside a uniform level return false (blank).
 */
static bool contour_classify_cell(const Scene *sc, const View *v,
                                  int c, int r,
                                  float emin, float step,
                                  int *out_level, char *out_ch)
{
    float xv, yv, xv_r, yv_d;
    cell_to_world(v, c,     r,     &xv,   &yv  );
    cell_to_world(v, c + 1, r,     &xv_r, &yv  );
    cell_to_world(v, c,     r + 1, &xv,   &yv_d);
    int lvl   = (int)((energy2d(sc, xv,   yv  ) - emin) / step);
    int lvl_r = (int)((energy2d(sc, xv_r, yv  ) - emin) / step);
    int lvl_d = (int)((energy2d(sc, xv,   yv_d) - emin) / step);
    if (lvl == lvl_r && lvl == lvl_d) return false;

    char ch = '.';
    if (lvl != lvl_r && lvl == lvl_d)       ch = '|';
    else if (lvl == lvl_r && lvl != lvl_d)  ch = '-';
    *out_level = lvl;
    *out_ch    = ch;
    return true;
}

/* level_to_tier_pair — map quantised energy level (0..N_CONTOURS-1) to
 * one of the four CP_E_LOW..CP_E_PEAK colour-pair indices. */
static int level_to_tier_pair(int level)
{
    int tier = (level * 4) / N_CONTOURS;
    if (tier < 0) tier = 0;
    if (tier > 3) tier = 3;
    return CP_E_LOW + tier;
}

/*
 * render_bowl_contours — discrete iso-energy rings, not a density fill.
 *
 * Two passes over the bowl region:
 *   1) find_energy_window   → [emin, emax]
 *   2) for each cell: contour_classify_cell → maybe draw
 *
 * Result: nested elliptic rings that read instantly as "topo map of a
 * bowl" instead of a confusing wall-to-wall character grid.
 */
static void render_bowl_contours(const Scene *sc, const View *v)
{
    float emin, emax;
    find_energy_window(sc, v, &emin, &emax);
    if (emax <= emin) return;
    float step = (emax - emin) / (float)N_CONTOURS;

    for (int r = 0; r < v->ph - 1; r++) {
        for (int c = 1; c < v->plot_w - 2; c++) {
            int  level; char ch;
            if (!contour_classify_cell(sc, v, c, r, emin, step, &level, &ch))
                continue;
            int cp = level_to_tier_pair(level);
            attron(COLOR_PAIR(cp));
            mvaddch(HDR_ROWS + r, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

static void render_axis_labels(const View *v)
{
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(HDR_ROWS + v->ph - 1, 1, "%.2f", (double)v->xlo);
    if (v->plot_w > 14)
        mvprintw(HDR_ROWS + v->ph - 1, v->plot_w - 10,
                 "x1 %.2f", (double)v->xhi);
    mvprintw(HDR_ROWS,            0, "%.2f", (double)v->yhi);
    mvprintw(HDR_ROWS + v->ph / 2, 0, "x2");
    attroff(COLOR_PAIR(CP_LABEL));
}

/* iterate_to_cell — affine map from world iterate to bowl cell. */
static void iterate_to_cell(const float (*hist)[2], int k, const View *v,
                            int *out_c, int *out_r)
{
    *out_c = to_col(hist[k][0], v->xlo, v->xhi, 1, v->plot_w - 2);
    *out_r = to_row(hist[k][1], v->ylo, v->yhi,
                    HDR_ROWS, HDR_ROWS + v->ph - 1);
}

/* cell_in_bowl — bounds check inside the plot's drawable region. */
static bool cell_in_bowl(int c, int r, const View *v)
{
    return r >= HDR_ROWS && r < HDR_ROWS + v->ph
        && c >= 1        && c < v->plot_w - 1;
}

/*
 * draw_dotted_segment — DDA line of '.' between two cells.
 * Skips the endpoints (they get the bullet glyph from the caller).
 */
static void draw_dotted_segment(int c0, int r0, int c1, int r1, const View *v)
{
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
static void render_trail(const float (*hist)[2], int n_iter,
                         int color_pair, const View *v)
{
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
static void render_marker(float x, float y, char glyph,
                          char lb, char rb, const char *label,
                          int color_pair, const View *v)
{
    int c = to_col(x, v->xlo, v->xhi, 1, v->plot_w - 2);
    int r = to_row(y, v->ylo, v->yhi, HDR_ROWS, HDR_ROWS + v->ph - 1);
    if (r < HDR_ROWS || r >= HDR_ROWS + v->ph) return;
    if (c < 1 || c >= v->plot_w - 1) return;

    attron(COLOR_PAIR(color_pair) | A_BOLD);
    if (c - 1 >= 1)              mvaddch(r, c - 1, lb);
    mvaddch(r, c, glyph);
    if (c + 1 < v->plot_w - 1)   mvaddch(r, c + 1, rb);

    if (label && *label) {
        int lbl_len = (int)strlen(label);
        int lc = c + 3;
        if (lc + lbl_len >= v->plot_w - 1) lc = c - 2 - lbl_len;
        int lr = r - 1;
        if (lr < HDR_ROWS) lr = r + 1;
        if (lr >= HDR_ROWS && lr < HDR_ROWS + v->ph
            && lc >= 1 && lc + lbl_len < v->plot_w - 1)
            mvprintw(lr, lc, "%s", label);
    }
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * render_bar — small inline bar at (y, x).  Filled cells `#` in fill_cp;
 * empty cells `.` in CP_LABEL (dim).  Used by the eigen panel and the
 * narrator's convergence bars.
 */
static void render_bar(int y, int x, int width, float frac, int fill_cp)
{
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    int filled = (int)(frac * (float)width + 0.5f);
    attron(COLOR_PAIR(fill_cp) | A_BOLD);
    for (int i = 0; i < filled; i++) mvaddch(y, x + i, '#');
    attroff(COLOR_PAIR(fill_cp) | A_BOLD);
    attron(COLOR_PAIR(CP_LABEL));
    for (int i = filled; i < width; i++) mvaddch(y, x + i, '.');
    attroff(COLOR_PAIR(CP_LABEL));
}

/* eigen_panel_header — "EIGENMODES" title at the top of the panel. */
static void eigen_panel_header(int x0, int top)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(top, x0, "EIGENMODES");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
}

/*
 * project_error_onto_eigvec — error magnitude in one eigendirection.
 *
 * For each algorithm, computes |⟨x_alg − x*, v⟩| — the absolute value
 * of the error projected onto eigenvector v.  Also returns the starting
 * projected error (|⟨x_true, v⟩|, since x_0 = 0) so the caller can
 * normalise: bar fraction = current / start.
 *
 * Reference: Shewchuk 1994 §B7 — the eigenvalue analysis of CG/SD
 * decomposes the error in this basis.
 */
static void project_error_onto_eigvec(const Scene *sc, const Vec v,
                                      float *out_cg_err,
                                      float *out_sd_err,
                                      float *out_start)
{
    Vec cg_err, sd_err;
    for (int i = 0; i < N; i++) {
        cg_err[i] = sc->cg.x[i] - sc->x_true[i];
        sd_err[i] = sc->sd.x[i] - sc->x_true[i];
    }
    *out_cg_err = fabsf(vdot(cg_err,    v));
    *out_sd_err = fabsf(vdot(sd_err,    v));
    float start = fabsf(vdot(sc->x_true, v));
    if (start < 1e-6f) start = 1.0f;
    *out_start  = start;
}

/*
 * draw_eigenmode_section — one eigenmode block: header + CG bar + SD bar.
 * Advances *y by 3 rows.
 */
static void draw_eigenmode_section(int *y, int x0, int bar_w,
                                   const char *label, float lambda,
                                   float cg_frac, float sd_frac)
{
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
 * draw_kappa_footer — condition number + helper text.
 * κ(A) = λ_max / λ_min governs how hard the problem is.  Display it
 * with a short legend so the viewer knows what "shorter bar" means.
 */
static void draw_kappa_footer(int *y, int x0, float l1, float l2)
{
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
 *   4. κ(A) footer
 *
 * Watching CG kill both modes in two steps while SD's λ2 bar barely
 * shrinks is the educational payoff of this panel.
 */
static void render_eigen_panel(const Scene *sc, int rows, int cols)
{
    int x0  = cols - EIGEN_PANEL_W + 1;
    int top = HDR_ROWS;
    int ph  = rows - HDR_ROWS - FTR_ROWS;
    if (x0 < 0 || ph < 12) return;

    int bar_w = EIGEN_PANEL_W - 8;
    if (bar_w < 5) bar_w = 5;

    eigen_panel_header(x0, top);

    int y = top + 2;

    float cg1, sd1, start1;
    project_error_onto_eigvec(sc, sc->v1, &cg1, &sd1, &start1);
    draw_eigenmode_section(&y, x0, bar_w,
                           "λ1 = %.2f", sc->l1,
                           cg1 / start1, sd1 / start1);
    y++;

    float cg2, sd2, start2;
    project_error_onto_eigvec(sc, sc->v2, &cg2, &sd2, &start2);
    draw_eigenmode_section(&y, x0, bar_w,
                           "λ2 = %.2f  (slow)", sc->l2,
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
static void render_plot(const Scene *sc, const View *v)
{
    render_bowl_contours(sc, v);
    render_trail(sc->cg.hist, sc->cg.iter, CP_CG, v);
    render_trail(sc->sd.hist, sc->sd.iter, CP_SD, v);
    render_marker(sc->cg.x[0], sc->cg.x[1], 'X', '(', ')',
                  " CG", CP_CG, v);
    render_marker(sc->sd.x[0], sc->sd.x[1], 'Y', '(', ')',
                  " SD", CP_SD, v);
    render_marker(sc->x_true[0], sc->x_true[1], '*', '[', ']',
                  " GOAL", CP_TARGET, v);
    render_axis_labels(v);
}

/* narrator_divider — '-' line separating the plot from the narrator. */
static void narrator_divider(int hud, int cols)
{
    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(hud, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));
}

/*
 * narrator_step_bars — row 1: live progress for both solvers.
 * Each algorithm gets a label, a fraction-complete bar (driven by
 * residual norm), and a "DONE" / "..." status.
 */
static void narrator_step_bars(int row, int cols, const Scene *sc)
{
    float cg_frac = 1.0f - vnorm(sc->cg.r) / (sc->cg.res[0] + 1e-12f);
    float sd_frac = 1.0f - vnorm(sc->sd.r) / (sc->sd.res[0] + 1e-12f);

    int bar_w = (cols - 30) / 2 - 12;
    if (bar_w < 8) bar_w = 8;

    attron(COLOR_PAIR(CP_CG) | A_BOLD);
    mvprintw(row, 1, " CG %d/%d ", sc->cg.iter, MAX_ITER_CG - 1);
    attroff(COLOR_PAIR(CP_CG) | A_BOLD);
    render_bar(row, 12, bar_w, cg_frac, CP_CG);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(row, 12 + bar_w + 1,
             "%s", sc->cg.converged ? "DONE  " : "      ");
    attroff(COLOR_PAIR(CP_HUD));

    int sd_x = 12 + bar_w + 9;
    attron(COLOR_PAIR(CP_SD) | A_BOLD);
    mvprintw(row, sd_x, " SD %d ", sc->sd.iter);
    attroff(COLOR_PAIR(CP_SD) | A_BOLD);
    render_bar(row, sd_x + 9, bar_w, sd_frac, CP_SD);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(row, sd_x + 9 + bar_w + 1,
             "%s", sc->sd.converged ? "DONE" : "...");
    attroff(COLOR_PAIR(CP_HUD));
}

/* narrator_legend_glyph — one coloured glyph + description pair. */
static void narrator_legend_glyph(int row, int col, const char *glyph,
                                  int glyph_pair, const char *desc)
{
    attron(COLOR_PAIR(glyph_pair) | A_BOLD);
    mvprintw(row, col, "%s", glyph);
    attroff(COLOR_PAIR(glyph_pair) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(row, col + (int)strlen(glyph), "%s", desc);
    attroff(COLOR_PAIR(CP_HUD));
}

/* narrator_legend — row 2: maps coloured glyphs on the plot to words. */
static void narrator_legend(int row)
{
    narrator_legend_glyph(row,  1, "(X)", CP_CG,     " CG path        ");
    narrator_legend_glyph(row, 21, "(Y)", CP_SD,     " SD path        ");
    narrator_legend_glyph(row, 41, "[*]", CP_TARGET, " target x*      ");
    narrator_legend_glyph(row, 61, "...", CP_LABEL,  " contour ring");
}

/*
 * narrator_explanations — rows 4-6: plain-English colour-coded story.
 * GREEN (CG) gets two lines for its A-conjugacy story; RED (SD) gets
 * one line on the right-angle zig-zag.
 */
static void narrator_explanations(int hud)
{
    attron(COLOR_PAIR(CP_CG) | A_BOLD);
    mvprintw(hud + 4, 1, " GREEN ");
    attroff(COLOR_PAIR(CP_CG) | A_BOLD);
    attron(COLOR_PAIR(CP_EXPLAIN));
    mvprintw(hud + 4, 9,
        "CG picks each direction A-conjugate to all previous ones — never");
    mvprintw(hud + 5, 9,
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
static void render_narrator(const Scene *sc, int rows, int cols)
{
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
static void draw_hud_top(const Scene *sc, int cols, int sim_fps)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(0, 0, " CG vs Steepest Descent  [N=%d] ", N);
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

    char buf[200];
    const char *state =
        (sc->cg.converged && sc->sd.converged) ? "BOTH DONE" :
         sc->cg.converged                      ? "CG DONE — SD GOING" :
         sc->paused                            ? "PAUSED " :
                                                 "running";
    snprintf(buf, sizeof buf,
             " CG:%d/%d  SD:%d  theme:%s  speed:%d/s  %s ",
             sc->cg.iter, MAX_ITER_CG - 1,
             sc->sd.iter,
             k_themes[sc->theme].name,
             sim_fps, state);
    int len = (int)strlen(buf);
    int col = cols - len;
    if (col < 0) col = 0;
    attron(COLOR_PAIR(CP_TOP) | A_BOLD);
    mvaddnstr(0, col, buf, cols);
    attroff(COLOR_PAIR(CP_TOP) | A_BOLD);

    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(1, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));
}

static void draw_hud_bottom(int cols, int rows)
{
    const char *hint_full =
        " q:quit  spc:pause  r:new system  t/T:theme  +/-:speed ";
    const char *hint_short =
        " q:quit  spc:pause  r:new  t:theme  +/-:speed ";
    const char *hint = hint_full;
    if ((int)strlen(hint_full) >= cols - 1) hint = hint_short;

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(rows - 1, 0, hint, cols);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §9 screen + app ────────────────────────────────────────────── */

typedef struct { int cols, rows; } Screen;

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void cleanup(void)      { endwin(); }
static void on_exit(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize(int sig) { (void)sig; g_app.need_resize = 1; }

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
    case 'q': case 'Q': return false;
    case ' ':
        app->scene.paused = !app->scene.paused;
        break;
    case 'r': case 'R':
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
    case '+': case '=':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps += 6;
        break;
    case '-': case '_':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps -= 6;
        break;
    default: break;
    }
    return true;
}

int main(void) {
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app = &g_app;
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
            frame_time       = clock_ns();
            sim_accum        = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

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
