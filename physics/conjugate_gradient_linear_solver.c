/*  conjugate_gradient_linear_solver.c
 *
 *  Conjugate Gradient (CG) linear solver visualizer.
 *  Solves Ax = b (A symmetric positive definite) step by step.
 *
 *  Left panel  : 2D energy landscape (first two components of x) +
 *                CG iterate trail + current search direction arrow.
 *  Right panel : log‖r‖ convergence bar chart per iteration.
 *  Bottom HUD  : iteration k, ‖r‖, step sizes α/β, x distance to x*, legend.
 *  Footer      : three lines explaining A-conjugacy and energy minimization.
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/conjugate_gradient_linear_solver.c \
 *        -o cg_solver -lncurses -lm
 *
 *  Keys:  SPACE  pause / resume
 *         r      new random SPD system
 *         + / -  speed up / slow down
 *         q      quit
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── §1 config ──────────────────────────────────────────────────── */

#define SIM_FPS_DEFAULT   2           /* CG steps per second          */
#define NS_PER_SEC        1000000000LL
#define NS_PER_MS         1000000LL
#define TICK_NS(f)        (NS_PER_SEC / (f))

#define N                 10          /* linear system dimension       */
#define MAX_ITER          (N + 2)     /* CG converges in ≤ N steps     */
#define HISTORY_LEN       (N + 4)     /* residual/trail history slots  */

#define PLOT_W_FRAC       0.60f       /* left panel fraction of cols   */
#define RES_NORM_TOL      1e-9f       /* convergence threshold         */
#define AUTO_RESTART_TICKS 60        /* ticks after convergence before restart */

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

/* ── §3 color ───────────────────────────────────────────────────── */

enum {
    CP_DEFAULT = 0,
    CP_E_LOW,           /* deep blue  — low energy bowl               */
    CP_E_MID,           /* cyan       — mid energy                    */
    CP_E_HIGH,          /* yellow     — high energy                   */
    CP_E_PEAK,          /* red        — energy peak                   */
    CP_SOLUTION,        /* bright green  — true solution x*           */
    CP_ITERATE,         /* bright white  — current iterate xₖ        */
    CP_SEARCH,          /* magenta    — search direction pₖ arrow     */
    CP_TRAIL,           /* grey       — past iterates                 */
    CP_BAR,             /* cyan       — residual plot bars            */
    CP_BAR_CUR,         /* bright yellow — current bar               */
    CP_HUD,             /* light grey — HUD text                      */
    CP_HEADER,          /* cyan       — header                        */
    CP_LABEL,           /* medium grey — axis labels / dividers       */
    CP_EXPLAIN,         /* yellow     — explanation footer            */
    CP_COUNT
};

static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_E_LOW,    17,  -1);
        init_pair(CP_E_MID,    33,  -1);
        init_pair(CP_E_HIGH,   214, -1);
        init_pair(CP_E_PEAK,   196, -1);
        init_pair(CP_SOLUTION, 46,  -1);
        init_pair(CP_ITERATE,  255, -1);
        init_pair(CP_SEARCH,   201, -1);
        init_pair(CP_TRAIL,    240, -1);
        init_pair(CP_BAR,      51,  -1);
        init_pair(CP_BAR_CUR,  226, -1);
        init_pair(CP_HUD,      252, -1);
        init_pair(CP_HEADER,   51,  -1);
        init_pair(CP_LABEL,    244, -1);
        init_pair(CP_EXPLAIN,  227, -1);
    } else {
        init_pair(CP_E_LOW,    COLOR_BLUE,    -1);
        init_pair(CP_E_MID,    COLOR_CYAN,    -1);
        init_pair(CP_E_HIGH,   COLOR_YELLOW,  -1);
        init_pair(CP_E_PEAK,   COLOR_RED,     -1);
        init_pair(CP_SOLUTION, COLOR_GREEN,   -1);
        init_pair(CP_ITERATE,  COLOR_WHITE,   -1);
        init_pair(CP_SEARCH,   COLOR_MAGENTA, -1);
        init_pair(CP_TRAIL,    COLOR_WHITE,   -1);
        init_pair(CP_BAR,      COLOR_CYAN,    -1);
        init_pair(CP_BAR_CUR,  COLOR_YELLOW,  -1);
        init_pair(CP_HUD,      COLOR_WHITE,   -1);
        init_pair(CP_HEADER,   COLOR_CYAN,    -1);
        init_pair(CP_LABEL,    COLOR_WHITE,   -1);
        init_pair(CP_EXPLAIN,  COLOR_YELLOW,  -1);
    }
}

/* ── §4 linear algebra (N-vectors, dense N×N matrices) ─────────── */

/* Row-major: A[i*N + j] */
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

/* out = a + alpha * b  (BLAS daxpy) */
static void vaxpy(float *out, const float *a, float alpha, const float *b) {
    for (int i = 0; i < N; i++) out[i] = a[i] + alpha * b[i];
}

/* ── §5 solver ──────────────────────────────────────────────────── */

typedef struct {
    Mat   A;                        /* N×N SPD matrix                     */
    Vec   b;                        /* right-hand side                    */
    Vec   x_true;                   /* reference solution (Cholesky)      */

    /* CG running state */
    Vec   x;                        /* current iterate xₖ                 */
    Vec   r;                        /* residual  rₖ = b − Axₖ            */
    Vec   p;                        /* search direction pₖ                */
    Vec   Ap;                       /* A·pₖ (cached to avoid redo)        */
    float rr;                       /* rₖᵀrₖ  (cached)                   */
    float alpha;                    /* step size αₖ = rᵀr / (pᵀAp)       */
    float beta;                     /* direction update βₖ = ‖r_new‖²/‖r‖²*/

    /* History */
    float res_hist[HISTORY_LEN];    /* ‖rₖ‖ per iteration                 */
    float x2d[HISTORY_LEN][2];     /* 2D projection of trail             */
    int   iter;                     /* current k                          */

    bool  converged;
    bool  paused;
} Solver;

/* A = BᵀB + N·I ensures positive definiteness for any B */
static void build_spd(Mat A, unsigned seed) {
    srand(seed);
    float B[N * N];
    for (int i = 0; i < N * N; i++)
        B[i] = (float)(rand() % 21 - 10) * 0.15f;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) s += B[k * N + i] * B[k * N + j];
            A[i * N + j] = s;
        }
    }
    /* Diagonal shift guarantees eigenvalues ≥ N */
    for (int i = 0; i < N; i++) A[i * N + i] += (float)N;
}

/* Cholesky factorization to find x_true = A⁻¹b (reference only) */
static void cholesky_solve(const float *A, const float *b, float *x) {
    float L[N * N];
    memset(L, 0, sizeof(L));

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

    float y[N];
    for (int i = 0; i < N; i++) {
        float s = b[i];
        for (int k = 0; k < i; k++) s -= L[i * N + k] * y[k];
        y[i] = s / L[i * N + i];
    }
    for (int i = N - 1; i >= 0; i--) {
        float s = y[i];
        for (int k = i + 1; k < N; k++) s -= L[k * N + i] * x[k];
        x[i] = s / L[i * N + i];
    }
}

static void solver_reset(Solver *s, unsigned seed) {
    build_spd(s->A, seed);

    /* b alternates [1, 2, 1, 2, ...] for a non-trivial rhs */
    for (int i = 0; i < N; i++) s->b[i] = (i % 2 == 0) ? 1.0f : 2.0f;

    cholesky_solve(s->A, s->b, s->x_true);

    /* CG init: x₀ = 0, r₀ = b − Ax₀ = b, p₀ = r₀ */
    memset(s->x, 0, sizeof(s->x));
    memcpy(s->r, s->b, sizeof(s->r));
    memcpy(s->p, s->b, sizeof(s->p));
    mat_vec_mul(s->A, s->p, s->Ap);
    s->rr        = vdot(s->r, s->r);
    s->alpha     = 0.0f;
    s->beta      = 0.0f;
    s->iter      = 0;
    s->converged = false;

    s->res_hist[0]  = vnorm(s->r);
    s->x2d[0][0]    = s->x[0];
    s->x2d[0][1]    = s->x[1];
}

/* Re-compute residual from scratch (used as a diagnostic utility) */
static void compute_residual(Solver *s) {
    Vec Ax;
    mat_vec_mul(s->A, s->x, Ax);
    for (int i = 0; i < N; i++) s->r[i] = s->b[i] - Ax[i];
    s->rr = vdot(s->r, s->r);
}

static void update_iteration(Solver *s) {
    if (s->converged || s->paused) return;
    if (s->iter >= MAX_ITER - 1) { s->converged = true; return; }
    if (vnorm(s->r) < RES_NORM_TOL) { s->converged = true; return; }

    float rr_old = s->rr;
    float pAp    = vdot(s->p, s->Ap);
    if (fabsf(pAp) < 1e-14f) { s->converged = true; return; }

    /*  αₖ = rᵀr / (pᵀAp)         optimal step along pₖ                */
    s->alpha = rr_old / pAp;

    /*  xₖ₊₁ = xₖ + αₖ pₖ                                               */
    vaxpy(s->x, s->x,  s->alpha, s->p);

    /*  rₖ₊₁ = rₖ − αₖ A pₖ       recurrence avoids a full mat-vec      */
    vaxpy(s->r, s->r, -s->alpha, s->Ap);

    /*  βₖ = ‖rₖ₊₁‖² / ‖rₖ‖²     Gram–Schmidt coefficient              */
    s->rr   = vdot(s->r, s->r);
    s->beta = s->rr / rr_old;

    /*  pₖ₊₁ = rₖ₊₁ + βₖ pₖ       A-conjugate to all previous pⱼ       */
    vaxpy(s->p, s->r, s->beta, s->p);

    mat_vec_mul(s->A, s->p, s->Ap);   /* cache for next α denominator   */

    s->iter++;
    if (s->iter < HISTORY_LEN) {
        s->res_hist[s->iter] = vnorm(s->r);
        s->x2d[s->iter][0]   = s->x[0];
        s->x2d[s->iter][1]   = s->x[1];
    }

    if (vnorm(s->r) < RES_NORM_TOL) s->converged = true;
}

/* ── §6 scene ───────────────────────────────────────────────────── */

typedef struct {
    Solver   solver;
    unsigned seed;
    int      restart_ticks;   /* countdown after convergence */
} Scene;

static void scene_init(Scene *sc) {
    sc->seed          = (unsigned)time(NULL);
    sc->restart_ticks = 0;
    sc->solver.paused = false;
    solver_reset(&sc->solver, sc->seed);
}

static void scene_tick(Scene *sc) {
    Solver *s = &sc->solver;
    if (s->converged) {
        sc->restart_ticks++;
        if (sc->restart_ticks > AUTO_RESTART_TICKS) {
            sc->seed += 12347;
            solver_reset(s, sc->seed);
            sc->restart_ticks = 0;
        }
        return;
    }
    update_iteration(s);
}

/* ── §7 rendering ───────────────────────────────────────────────── */

static int to_col(float v, float lo, float hi, int c0, int c1) {
    if (hi <= lo) return c0;
    float t = (v - lo) / (hi - lo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return c0 + (int)(t * (float)(c1 - c0));
}

/* Invert y: high value maps to low row index (top of screen) */
static int to_row(float v, float lo, float hi, int r0, int r1) {
    if (hi <= lo) return r0;
    float t = (v - lo) / (hi - lo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return r1 - (int)(t * (float)(r1 - r0));
}

/*  f(x₁,x₂) = ½ xᵀ A x − bᵀx  projected onto first two dimensions.
 *  This gives the 2D cross-section of the full N-D energy landscape,
 *  which is an upward-opening elliptic paraboloid (bowl) in 2D.        */
static float energy2d(const Solver *s, float x0, float x1) {
    float a00 = s->A[0 * N + 0];
    float a01 = s->A[0 * N + 1];
    float a11 = s->A[1 * N + 1];
    float q   = 0.5f * (a00 * x0 * x0 + 2.0f * a01 * x0 * x1 + a11 * x1 * x1);
    return q - s->b[0] * x0 - s->b[1] * x1;
}

static const char e_chars[] = " .:!|=*#@";

static char emap_char(float e, float emin, float emax) {
    if (emax <= emin) return ' ';
    float t = (e - emin) / (emax - emin);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    int idx = (int)(t * 8.0f);
    if (idx > 8) idx = 8;
    return e_chars[idx];
}

static int emap_pair(float t) {
    if (t < 0.25f) return CP_E_LOW;
    if (t < 0.50f) return CP_E_MID;
    if (t < 0.75f) return CP_E_HIGH;
    return CP_E_PEAK;
}

static void render_plot(const Scene *sc, int rows, int cols) {
    const Solver *s   = &sc->solver;
    int plot_w        = (int)((float)cols * PLOT_W_FRAC);
    if (plot_w < 20) plot_w = 20;

    static const int HDR = 2;
    static const int FTR = 8;
    int ph = rows - HDR - FTR;
    if (ph < 4) return;

    /* View range centred on true solution, expanded to fit all iterates */
    float cx    = s->x_true[0];
    float cy    = s->x_true[1];
    float range = 0.8f;
    for (int k = 0; k <= s->iter && k < HISTORY_LEN; k++) {
        float dx = fabsf(s->x2d[k][0] - cx);
        float dy = fabsf(s->x2d[k][1] - cy);
        if (dx + 0.15f > range) range = dx + 0.15f;
        if (dy + 0.15f > range) range = dy + 0.15f;
    }
    float xlo = cx - range, xhi = cx + range;
    float ylo = cy - range, yhi = cy + range;

    /* Pass 1: energy min/max for normalisation */
    float emin = 1e30f, emax = -1e30f;
    for (int row = 0; row < ph; row++) {
        for (int ci = 1; ci < plot_w - 1; ci++) {
            float xv = xlo + (xhi - xlo) * (float)(ci - 1) / (float)(plot_w - 3);
            float yv = yhi - (yhi - ylo) * (float)row       / (float)(ph - 1);
            float e  = energy2d(s, xv, yv);
            if (e < emin) emin = e;
            if (e > emax) emax = e;
        }
    }

    /* Pass 2: draw energy landscape */
    for (int row = 0; row < ph; row++) {
        for (int ci = 1; ci < plot_w - 1; ci++) {
            float xv = xlo + (xhi - xlo) * (float)(ci - 1) / (float)(plot_w - 3);
            float yv = yhi - (yhi - ylo) * (float)row       / (float)(ph - 1);
            float e  = energy2d(s, xv, yv);
            float t  = (emax > emin) ? (e - emin) / (emax - emin) : 0.0f;
            char  ch = emap_char(e, emin, emax);
            attron(COLOR_PAIR(emap_pair(t)));
            mvaddch(HDR + row, ci, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(emap_pair(t)));
        }
    }

    /* Trail of past iterates */
    for (int k = 0; k < s->iter && k < HISTORY_LEN - 1; k++) {
        int c = to_col(s->x2d[k][0], xlo, xhi, 1, plot_w - 2);
        int r = to_row(s->x2d[k][1], ylo, yhi, HDR, HDR + ph - 1);
        if (r >= HDR && r < HDR + ph && c >= 1 && c < plot_w - 1) {
            attron(COLOR_PAIR(CP_TRAIL) | A_BOLD);
            mvaddch(r, c, '+');
            attroff(COLOR_PAIR(CP_TRAIL) | A_BOLD);
        }
    }

    /* Current search direction arrow  x → x + (p/‖p‖)·range·0.35 */
    if (!s->converged) {
        float pn = sqrtf(s->p[0] * s->p[0] + s->p[1] * s->p[1]);
        if (pn > 1e-10f) {
            float scale = range * 0.35f;
            float dpx   = s->p[0] / pn * scale;
            float dpy   = s->p[1] / pn * scale;
            for (int pt = 0; pt <= 6; pt++) {
                float t  = (float)pt / 6.0f;
                float xv = s->x[0] + dpx * t;
                float yv = s->x[1] + dpy * t;
                int c = to_col(xv, xlo, xhi, 1, plot_w - 2);
                int r = to_row(yv, ylo, yhi, HDR, HDR + ph - 1);
                if (r >= HDR && r < HDR + ph && c >= 1 && c < plot_w - 1) {
                    attron(COLOR_PAIR(CP_SEARCH) | A_BOLD);
                    mvaddch(r, c, (chtype)(unsigned char)((pt == 6) ? '>' : '-'));
                    attroff(COLOR_PAIR(CP_SEARCH) | A_BOLD);
                }
            }
        }
    }

    /* Current iterate xₖ */
    {
        int c = to_col(s->x[0], xlo, xhi, 1, plot_w - 2);
        int r = to_row(s->x[1], ylo, yhi, HDR, HDR + ph - 1);
        if (r >= HDR && r < HDR + ph && c >= 1 && c < plot_w - 1) {
            attron(COLOR_PAIR(CP_ITERATE) | A_BOLD);
            mvaddch(r, c, 'X');
            attroff(COLOR_PAIR(CP_ITERATE) | A_BOLD);
        }
    }

    /* True solution x* */
    {
        int c = to_col(s->x_true[0], xlo, xhi, 1, plot_w - 2);
        int r = to_row(s->x_true[1], ylo, yhi, HDR, HDR + ph - 1);
        if (r >= HDR && r < HDR + ph && c >= 1 && c < plot_w - 1) {
            attron(COLOR_PAIR(CP_SOLUTION) | A_BOLD);
            mvaddch(r, c, '*');
            attroff(COLOR_PAIR(CP_SOLUTION) | A_BOLD);
        }
    }

    /* Axis corner labels */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(HDR + ph - 1, 1, "%.2f", (double)xlo);
    if (plot_w > 12)
        mvprintw(HDR + ph - 1, plot_w - 8, "x1 %.2f", (double)xhi);
    mvprintw(HDR,          0, "%.2f", (double)yhi);
    mvprintw(HDR + ph / 2, 0, "x2");
    attroff(COLOR_PAIR(CP_LABEL));

    /* Vertical divider between panels */
    for (int r = HDR; r < rows - FTR; r++) {
        attron(COLOR_PAIR(CP_LABEL));
        mvaddch(r, plot_w, '|');
        attroff(COLOR_PAIR(CP_LABEL));
    }
}

static void render_residual_panel(const Scene *sc, int rows, int cols) {
    const Solver *s  = &sc->solver;
    int plot_w       = (int)((float)cols * PLOT_W_FRAC);
    if (plot_w < 20) plot_w = 20;
    int px  = plot_w + 1;
    int pw  = cols - px - 1;
    if (pw < 5) return;

    static const int HDR = 2;
    static const int FTR = 8;
    int ph = rows - HDR - FTR - 2;   /* -2: sub-header + bottom label  */
    if (ph < 3) return;

    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(HDR, px, " log||r|| vs iteration k");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

    /* Log-residual range across history */
    float lmax = -1e30f, lmin = 1e30f;
    for (int k = 0; k <= s->iter && k < HISTORY_LEN; k++) {
        float rn = s->res_hist[k];
        if (rn > 1e-15f) {
            float lr = log10f(rn);
            if (lr > lmax) lmax = lr;
            if (lr < lmin) lmin = lr;
        }
    }
    if (lmax - lmin < 0.5f) lmin = lmax - 1.5f;

    int sub_hdr = HDR + 1;

    /* One vertical bar per iteration */
    for (int k = 0; k <= s->iter && k < HISTORY_LEN; k++) {
        float rn = (s->res_hist[k] > 1e-15f) ? s->res_hist[k] : 1e-15f;
        float lr = log10f(rn);
        /* Taller bar = better convergence (lower residual) */
        float t     = (lmax > lmin) ? (lmax - lr) / (lmax - lmin) : 1.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        int bar_h   = 1 + (int)(t * (float)(ph - 1));
        if (bar_h > ph) bar_h = ph;

        int bc = px + 1 + (int)((float)k / (float)(HISTORY_LEN - 1) * (float)(pw - 3));
        if (bc >= cols - 1) continue;

        int cp = (k == s->iter) ? CP_BAR_CUR : CP_BAR;
        attron(COLOR_PAIR(cp) | A_BOLD);
        for (int row = 0; row < bar_h; row++) {
            int y = sub_hdr + ph - row;
            if (y >= sub_hdr && y < rows - FTR)
                mvaddch(y, bc, '|');
        }
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    /* Y-axis labels */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(sub_hdr,      px, "10^%.0f", (double)lmax);
    mvprintw(sub_hdr + ph, px, "10^%.0f", (double)lmin);
    mvprintw(sub_hdr + ph + 1, px, "k=0");
    if (px + pw - 6 < cols)
        mvprintw(sub_hdr + ph + 1, px + pw - 6, "k=%d", MAX_ITER - 1);
    attroff(COLOR_PAIR(CP_LABEL));

    /* Converged label */
    if (s->converged) {
        attron(COLOR_PAIR(CP_SOLUTION) | A_BOLD);
        mvprintw(sub_hdr + 1, px + 1, "CONVERGED");
        mvprintw(sub_hdr + 2, px + 1, "in %d steps", s->iter);
        attroff(COLOR_PAIR(CP_SOLUTION) | A_BOLD);
    }
}

static void render_overlay(const Scene *sc, int rows, int cols) {
    const Solver *s  = &sc->solver;
    int hud          = rows - 8;

    /* Divider */
    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(hud, c, '-');
    attroff(COLOR_PAIR(CP_LABEL));

    /* Line 1: core numerics */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hud + 1, 1,
        " iter: %2d/%2d    |r|: %.3e    alpha: %+.4f    beta: %.4f",
        s->iter, MAX_ITER - 1,
        (double)vnorm(s->r),
        (double)s->alpha,
        (double)s->beta);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Line 2: x distance to true solution */
    Vec dx;
    for (int i = 0; i < N; i++) dx[i] = s->x[i] - s->x_true[i];
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(hud + 2, 1,
        " ||x - x*||: %.3e    x[0]=% .4f  x[1]=% .4f    x*[0]=% .4f  x*[1]=% .4f",
        (double)vnorm(dx),
        (double)s->x[0], (double)s->x[1],
        (double)s->x_true[0], (double)s->x_true[1]);
    attroff(COLOR_PAIR(CP_HUD));

    /* Line 3: legend */
    int lr = hud + 3;
    attron(COLOR_PAIR(CP_SOLUTION) | A_BOLD); mvaddch(lr, 2, '*'); attroff(COLOR_PAIR(CP_SOLUTION) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD)); mvprintw(lr, 3, " x*");  attroff(COLOR_PAIR(CP_HUD));

    attron(COLOR_PAIR(CP_ITERATE) | A_BOLD);  mvaddch(lr, 9,  'X'); attroff(COLOR_PAIR(CP_ITERATE) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD)); mvprintw(lr, 10, " xk"); attroff(COLOR_PAIR(CP_HUD));

    attron(COLOR_PAIR(CP_SEARCH) | A_BOLD);   mvprintw(lr, 15, "-->"); attroff(COLOR_PAIR(CP_SEARCH) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD)); mvprintw(lr, 18, " search dir pk"); attroff(COLOR_PAIR(CP_HUD));

    attron(COLOR_PAIR(CP_TRAIL) | A_BOLD);    mvaddch(lr, 34, '+'); attroff(COLOR_PAIR(CP_TRAIL) | A_BOLD);
    attron(COLOR_PAIR(CP_HUD)); mvprintw(lr, 35, " trail"); attroff(COLOR_PAIR(CP_HUD));

    attron(COLOR_PAIR(CP_E_LOW));  mvprintw(lr, 44, ".:"); attroff(COLOR_PAIR(CP_E_LOW));
    attron(COLOR_PAIR(CP_E_MID));  mvprintw(lr, 46, "!|"); attroff(COLOR_PAIR(CP_E_MID));
    attron(COLOR_PAIR(CP_E_HIGH)); mvprintw(lr, 48, "=*"); attroff(COLOR_PAIR(CP_E_HIGH));
    attron(COLOR_PAIR(CP_E_PEAK)); mvprintw(lr, 50, "#@"); attroff(COLOR_PAIR(CP_E_PEAK));
    attron(COLOR_PAIR(CP_HUD)); mvprintw(lr, 53, " energy (low→high)"); attroff(COLOR_PAIR(CP_HUD));

    /* Lines 4–6: education */
    attron(COLOR_PAIR(CP_EXPLAIN));
    mvprintw(hud + 4, 1,
        " Orthogonal dirs: p_i' A p_j = 0 for i!=j  (A-conjugate, not Euclidean-orthogonal)");
    mvprintw(hud + 5, 1,
        " Energy view: f(x)=0.5*x'Ax-b'x is a bowl; CG steps to the minimum in <=N strides");
    mvprintw(hud + 6, 1,
        " Each alpha_k minimises f along p_k exactly; beta_k keeps next dir A-conjugate to all prev");
    attroff(COLOR_PAIR(CP_EXPLAIN));

    /* Line 7: controls */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(hud + 7, 1,
        " SPACE pause/resume   r new system   +/- speed   q quit");
    attroff(COLOR_PAIR(CP_LABEL));
}

static void render_header(const Scene *sc, int cols) {
    const Solver *s = &sc->solver;
    (void)s;
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(0, 0,
        " Conjugate Gradient Solver  [%dx%d SPD]"
        "   2D projection of x_k on first two components",
        N, N);
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);

    attron(COLOR_PAIR(CP_LABEL));
    for (int c = 0; c < cols; c++) mvaddch(1, c, '=');
    attroff(COLOR_PAIR(CP_LABEL));
}

/* ── §8 screen + app ────────────────────────────────────────────── */

typedef struct { int cols, rows; } Screen;

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void cleanup(void)              { endwin(); }
static void on_exit(int sig)           { (void)sig; g_app.running    = 0; }
static void on_resize(int sig)         { (void)sig; g_app.need_resize = 1; }

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
    render_header(&app->scene, cols);
    render_plot(&app->scene, rows, cols);
    render_residual_panel(&app->scene, rows, cols);
    render_overlay(&app->scene, rows, cols);
    wnoutrefresh(stdscr);
    doupdate();
}

static bool app_handle_key(App *app, int ch) {
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':
        app->scene.solver.paused = !app->scene.solver.paused;
        break;
    case 'r': case 'R':
        app->scene.seed += 99991;
        solver_reset(&app->scene.solver, app->scene.seed);
        app->scene.restart_ticks = 0;
        break;
    case '+': case '=':
        if (app->sim_fps < 20) app->sim_fps++;
        break;
    case '-': case '_':
        if (app->sim_fps > 1) app->sim_fps--;
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

    App *app    = &g_app;
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    /* Suppress unused-function warning for compute_residual (available
     * as a diagnostic; called here once to verify initial residual).    */
    compute_residual(&app->scene.solver);

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
