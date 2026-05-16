/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pendulum.c  —  N-link chaotic pendulum (N = 1..5) terminal demo
 *
 * A chain of N rigid rods with unit-mass bobs at each joint.  All arm
 * lengths equal L, no damping, gravity straight down.  Boots as the
 * single pendulum (N = 1, the textbook periodic setup); n adds links
 * up to N = 5, p removes them.  Each N gives qualitatively different
 * motion:
 *
 *     N = 1   simple pendulum — periodic (the textbook setup, no chaos)
 *     N = 2   double — chaotic but tame; the classic Lyapunov demo
 *     N = 3   triple — wilder, the end bob whips
 *     N = 4   quadruple — increasingly violent chaos
 *     N = 5   quintuple — fully unhinged, dramatic
 *
 *   Equations of motion (Lagrangian, equal masses m_i = 1, equal arms L):
 *
 *     M(θ) · α  =  b(θ, θ̇)
 *
 *     M[j][k]   =  L² · (N − max(j,k) + 1) · cos(θⱼ − θₖ)
 *     b[j]      = −L² · Σₖ (N − max(j,k) + 1) · sin(θⱼ − θₖ) · ωₖ²
 *                 − g · L · (N − j + 1) · sin(θⱼ)
 *
 *   Each tick: build M and b, solve M·α = b for the angular accels
 *   by Gaussian elimination with partial pivoting (≤ 5×5 system),
 *   integrate the state (θ, ω) by 4th-order Runge-Kutta.  For N=2 this
 *   reduces to the closed-form double-pendulum equations.  For N=1 it
 *   collapses to θ̈ = −(g/L)·sin θ.  The general matrix form keeps the
 *   code uniform across all N.
 *
 * Integration: 4th-order Runge-Kutta.
 *   RK4 is essential for chaotic systems — lower-order integrators
 *   (e.g. Euler) accumulate phase errors on the Lyapunov time-scale
 *   (~3–5 s) that are visually indistinguishable from real chaos.
 *
 * Chaos demo:
 *   A dim "ghost" pendulum starts with θ₁ + GHOST_EPSILON.  Both
 *   trajectories are identical at first; after ~3–5 s (N ≥ 2) they
 *   diverge completely, demonstrating sensitive dependence on initial
 *   conditions.  The HUD shows the total angular separation growing
 *   exponentially.  N = 1 is non-chaotic, so the ghost stays glued.
 *
 * Trail:
 *   The end-bob (last joint) traces a colour-faded ring-buffer trail.
 *   Recent positions are bright red/orange; older ones fade to dim grey.
 *   Reveals the complex attractor geometry as the system evolves.
 *
 * Themes (t / T cycle, 10 total): Matrix, Fire, Oceanic, Neon, Mono,
 *   Ice, Nova, Forest, Desert, Eclipse.  Drive arm / joint / bob / trail
 *   / ghost colours.  Pivot bracket and the HUD chrome (top yellow,
 *   bottom cyan) stay fixed across every theme so the structure and
 *   key list remain legible whichever palette is active.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset (same initial conditions; ghost re-syncs)
 *   n / p     more / fewer links  (1..5)
 *   t / T     next / previous theme
 *   g         toggle ghost pendulum
 *   l         toggle trail (was 't' — moved so 't' can cycle theme)
 *   + =       longer trail
 *   -         shorter trail
 *   ] [       faster / slower simulation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/pendulum.c \
 *       -o pendulum -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords
 *   §5  physics — StateN, deriv, RK4, NPend, npend_positions
 *   §6  scene   — Scene, trail ring-buffer, draw
 *   §7  screen
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : 4th-order Runge-Kutta (RK4) integration of an N-link
 *                  chain whose accelerations come from solving a small
 *                  linear system each derivative evaluation.  RK4 makes
 *                  four derivative calls per step (k1..k4), so per RK4
 *                  step we solve four ≤5×5 linear systems — still cheap.
 *
 * Physics        : Lagrangian mechanics, classical chaos.
 *                  For N ≥ 2 the system is chaotic: a tiny perturbation
 *                  in θ₁ (the ghost's GHOST_EPSILON ≈ 0.057°) grows
 *                  exponentially at rate e^(λt) where λ is the Lyapunov
 *                  exponent.  Visible divergence in ~3–5 s.  N = 1 is
 *                  integrable — periodic motion, ghost never separates.
 *
 * Math           : Mass matrix M and force vector b derived from the
 *                  Lagrangian T − V for equal m_i = 1 and L_i = L.
 *                  M is symmetric and (for our initial conditions and
 *                  N ≤ 5) well-conditioned, so Gaussian elimination
 *                  with partial pivoting solves M·α = b stably.
 *
 *                    M[j][k] = (N − max(j,k) + 1) · L² · cos(θⱼ − θₖ)
 *                    b[j]    = −Σₖ (N − max(j,k) + 1) · L² · sin(θⱼ − θₖ) · ωₖ²
 *                              − g · L · (N − j + 1) · sin(θⱼ)
 *
 *                  (Indices above are 1-based to match textbook
 *                  conventions; the code uses 0-based indices, with the
 *                  factor (N − max + 1) replaced by (N − max_0idx).)
 *
 * Performance    : Render-interpolation (alpha).  Sim runs at 300 Hz,
 *                  display at ~60 Hz.  Between sim ticks the angles are
 *                  linearly interpolated by alpha ∈ [0,1] for smooth
 *                  motion without needing 300-Hz rendering.
 *
 * Data-structure : Ring-buffer trail (TRAIL_LEN positions) iterated
 *                  oldest→newest by offset arithmetic — no shifting.
 *
 * Rendering      : Arm segments rasterised by a Bresenham line walk,
 *                  glyph chosen by step direction (`-` | `|` / `\`).
 *                  Trail drawn from a ring buffer in three brightness
 *                  tiers (newest → oldest).  Render uses linear angle
 *                  interpolation between consecutive sim ticks so the
 *                  display can refresh at any rate without juddering.
 *                  Themes drive arm / joint / bob / trail / ghost
 *                  colours; pivot bracket and HUD chrome stay fixed.
 *
 * References     :
 *   • Lagrange, J. L. (1788) "Mécanique analytique", Paris.
 *     — The foundation of Lagrangian mechanics.  The M·α = b formulation
 *       we build each step descends directly from the Euler-Lagrange
 *       equations introduced here.
 *
 *   • Goldstein, H., Poole, C. P., Safko, J. L. (2001) "Classical
 *     Mechanics" (3rd ed.), Addison-Wesley.  Chapters 1-2 and 8.
 *     — Standard graduate textbook.  Cleanest modern derivation of the
 *       Lagrangian equations of motion for a chain of rigid links.
 *
 *   • Shinbrot, T., Grebogi, C., Wisdom, J., Yorke, J. A. (1992)
 *     "Chaos in a double pendulum", Am. J. Phys. 60(6), 491-499.
 *     — The canonical paper on double-pendulum chaos.  Derives the
 *       Lyapunov exponent and shows experimentally that the ghost test
 *       (two near-identical pendulums diverging exponentially) is
 *       definitive evidence of chaos.
 *
 *   • Strogatz, S. H. (2014) "Nonlinear Dynamics and Chaos: With
 *     Applications to Physics, Biology, Chemistry, and Engineering"
 *     (2nd ed.), Westview Press.
 *     — The best pedagogical introduction to chaos theory: Lyapunov
 *       exponents, sensitive dependence on initial conditions, the
 *       butterfly effect.  Ch. 9-12 specifically cover the chaotic
 *       behaviour we visualise with the ghost pendulum.
 *
 *   • Lorenz, E. N. (1963) "Deterministic Nonperiodic Flow", J.
 *     Atmospheric Sciences 20(2), 130-141.
 *     — Founded modern chaos theory.  Same sensitive-dependence result
 *       we demonstrate here, in a 3-equation atmospheric model.
 *
 *   • Press, W. H., Teukolsky, S. A., Vetterling, W. T., Flannery,
 *     B. P. (2007) "Numerical Recipes" (3rd ed.), Cambridge.  Ch. 17.
 *     — The pragmatic reference for RK4 and adaptive stepping with
 *       working code.  Start here if you want to upgrade rk4_step_n to
 *       RK45 (Dormand-Prince) or a symplectic integrator.
 *
 *   • Hairer, E., Nørsett, S. P., Wanner, G. (1993) "Solving Ordinary
 *     Differential Equations I: Nonstiff Problems" (2nd ed.), Springer.
 *     — Rigorous reference for explicit Runge-Kutta methods; error
 *       analysis and order conditions that justify why RK4's O(dt⁴)
 *       global error is enough for chaotic systems on our time-scales.
 *
 *   • Bresenham, J. E. (1965) "Algorithm for computer control of a
 *     digital plotter", IBM Systems Journal 4(1), 25-30.
 *     — Original digital line drawing.  The arm rendering in
 *       draw_line() is a direct descendant.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    N_MAX           =    5,   /* maximum number of pendulum links        */
    N_DEFAULT       =    1,   /* boot configuration — single pendulum     */
    N_THEMES        =   10,   /* see k_themes[] in §3                     */

    /* 300 Hz sim gives dt ≈ 3.3 ms per RK4 step.  RK4 global error
     * O(dt⁴) ≈ 1e-10 per step — negligible on the few-second time
     * scales we care about.  Lower fps → visible divergence from the
     * true trajectory in just 2–3 seconds.                                */
    SIM_FPS_DEFAULT =  300,
    SIM_FPS_MIN     =   60,
    SIM_FPS_MAX     =  600,
    SIM_FPS_STEP    =   60,

    TRAIL_LEN       =  500,   /* ring-buffer capacity (max trail positions) */
    TRAIL_DEF       =  360,   /* positions drawn by default (newest 360)    */
    TRAIL_MIN       =   20,
    TRAIL_STEP      =   20,

    HUD_COLS        =   64,   /* fixed-width HUD string length            */
    FPS_UPDATE_MS   =  500,   /* FPS display refresh interval (ms)        */
};

/* CELL_W / CELL_H — sub-pixel resolution.  Physics in pixel space;
 * terminal cells only appear in the draw step. */
#define CELL_W   8
#define CELL_H  16

/*
 * MAX_REACH_FRAC — total chain length as a fraction of pixel-space
 * height (so the whole pendulum can hang fully extended without going
 * off-screen).  Each arm = MAX_REACH_FRAC / N.  At 0.44 a fully-extended
 * chain covers 44 % of screen height, matching the old double-pendulum
 * visual when N = 2.
 */
#define MAX_REACH_FRAC  0.44f

/* GRAVITY_PX: acceleration in terminal pixel-space (px/s²).
 * Empirical 2000 gives a dramatic swing — the same as double_pendulum.c. */
#define GRAVITY_PX      2000.0f

/*
 * INIT_THETA_DEG — starting angle for every link, from straight-down.
 * 120° puts each arm 30° past horizontal — high energy, chaotic onset
 * within ~1 s for N ≥ 2.  All links start at the same angle so the
 * chain is initially fully extended at 120°.
 */
#define INIT_THETA_DEG  120.0f

/* GHOST_EPSILON: initial θ₁ perturbation for the ghost (radians).
 * 0.001 rad ≈ 0.057° — within typical angle-measurement precision.
 * Demonstrates that chaos is not an artifact of large errors: even
 * this nanoscopic difference grows exponentially in ~3–5 s.            */
#define GHOST_EPSILON   0.001f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color — themes + canonical HUD chrome                              */
/* ===================================================================== */

/*
 * Theme-driven roles (change with the active palette):
 *   CP_ARM1..CP_ARM5  arm segments (5-colour cycle drawn from theme)
 *   CP_JOINT          intermediate joint markers (theme mid tier)
 *   CP_BOB            end-bob accent (brightest theme highlight)
 *   CP_TR1, CP_TR2, CP_TR3  trail tiers newest → oldest
 *   CP_GHOST          ghost pendulum (dim, theme-tinted)
 *
 * Fixed chrome (theme-independent):
 *   CP_BAR    pivot bracket [+] (bright white — structural anchor)
 *   CP_HUD    canonical top status (bright yellow + bold)
 *   CP_HINT   canonical bottom action bar (bright cyan + bold)
 */
enum {
    CP_BAR   = 1,
    CP_ARM1, CP_ARM2, CP_ARM3, CP_ARM4, CP_ARM5,
    CP_JOINT,
    CP_BOB,
    CP_TR1, CP_TR2, CP_TR3,
    CP_GHOST,
    CP_HUD,
    CP_HINT,
};

/*
 * Theme — one palette covering every theme-driven role.
 *   ramp[4]   four colour tiers, dim → bright.  Used for arms 0..3
 *             (arm 4 = accent), trail tiers, and joint colour.
 *   accent    brightest highlight — end bob + arm-5 cap colour.
 *   ghost     dim theme-tinted shade for the perturbed ghost pendulum.
 *
 * Brightness safety (CLAUDE.md): every entry sits at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier.  The forbidden zones
 * (16-23 cube, 232-239 gray) would vanish against default-black.
 */
typedef struct {
    const char *name;
    short ramp[4];
    short accent;
    short ghost;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0..3]                  accent  ghost              */
    { "Matrix",    {  28,  34,  40,  46 },        82,    28 }, /* cyber green  */
    { "Fire",      { 130, 208, 202, 196 },       226,    88 }, /* warm → red   */
    { "Oceanic",   {  24,  31,  39,  51 },       195,    24 }, /* teal → cyan  */
    { "Neon",      { 129, 165, 201, 213 },       219,    53 }, /* purple → pink*/
    { "Mono",      { 240, 247, 250, 255 },       255,   240 }, /* grayscale    */
    { "Ice",       { 153, 117, 159, 195 },       231,    24 }, /* light blues  */
    { "Nova",      { 129, 141, 177, 213 },       219,    53 }, /* stellar      */
    { "Forest",    {  58, 100, 142, 190 },       226,    28 }, /* leaves/bark  */
    { "Desert",    { 130, 178, 214, 220 },       226,    94 }, /* sand/gold    */
    { "Eclipse",   { 240, 244, 124, 196 },       226,   240 }, /* gray + red   */
};

/*
 * theme_apply — install the palette for theme index t.
 *
 * Mapping per theme:
 *   CP_ARM1..4 = ramp[0..3]              (5-arm chain cycles arm0..3
 *   CP_ARM5    = accent                   then highlight on arm 4)
 *   CP_JOINT   = ramp[2]                  (mid tier — visible joints)
 *   CP_BOB     = accent                   (brightest highlight)
 *   CP_TR1..3  = ramp[3], [2], [1]        (newest→oldest trail tiers)
 *   CP_GHOST   = ghost
 *
 * Fixed chrome (CP_BAR / CP_HUD / CP_HINT) never changes — stays
 * readable across every theme.
 */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t % N_THEMES];
    if (COLORS >= 256) {
        init_pair(CP_ARM1,  th->ramp[0],  COLOR_BLACK);
        init_pair(CP_ARM2,  th->ramp[1],  COLOR_BLACK);
        init_pair(CP_ARM3,  th->ramp[2],  COLOR_BLACK);
        init_pair(CP_ARM4,  th->ramp[3],  COLOR_BLACK);
        init_pair(CP_ARM5,  th->accent,   COLOR_BLACK);
        init_pair(CP_JOINT, th->ramp[2],  COLOR_BLACK);
        init_pair(CP_BOB,   th->accent,   COLOR_BLACK);
        init_pair(CP_TR1,   th->ramp[3],  COLOR_BLACK);
        init_pair(CP_TR2,   th->ramp[2],  COLOR_BLACK);
        init_pair(CP_TR3,   th->ramp[1],  COLOR_BLACK);
        init_pair(CP_GHOST, th->ghost,    COLOR_BLACK);
        /* fixed chrome */
        init_pair(CP_BAR,   231,          COLOR_BLACK);  /* bright white  */
        init_pair(CP_HUD,   226,          COLOR_BLACK);  /* bright yellow */
        init_pair(CP_HINT,   51,          COLOR_BLACK);  /* bright cyan   */
    } else {
        /* 8-colour fallback — theme-independent. */
        init_pair(CP_ARM1,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_ARM2,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_ARM3,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_ARM4,  COLOR_RED,     COLOR_BLACK);
        init_pair(CP_ARM5,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_JOINT, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_BOB,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_TR1,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TR2,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TR3,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_GHOST, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_BAR,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HUD,   COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_HINT,  COLOR_CYAN,    COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    theme_apply(0);
}

/* arm_pair — colour cycle for the n-th arm segment (0-indexed). */
static int arm_pair(int link)
{
    static const int arms[N_MAX] = {
        CP_ARM1, CP_ARM2, CP_ARM3, CP_ARM4, CP_ARM5
    };
    if (link < 0)          return CP_ARM1;
    if (link >= N_MAX)     return CP_ARM5;
    return arms[link];
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  physics                                                            */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────
 * StateN — phase-space state vector for an N-link chain.
 *
 * The state of a Lagrangian system with n generalised coordinates lives
 * in 2n-dimensional phase space.  Here:
 *
 *     th[i]   generalised coordinate θᵢ — angle of link i from vertical
 *     om[i]   generalised velocity   ωᵢ = θ̇ᵢ
 *
 * Stored as parallel arrays so the RK4 update can do component-wise
 * arithmetic without struct gymnastics.  Sized N_MAX (5) at compile
 * time so an NPend doesn't need a malloc; only the first n slots are
 * touched per call (n comes from the enclosing NPend).
 *
 * Slot ordering:
 *     index 0   = topmost link, attached to the pivot
 *     index n-1 = bottommost link, carrying the end bob
 *
 * Reference:
 *   • Goldstein 2001 §1.4 — generalised coordinates and the (q, q̇)
 *     state vector this struct represents.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    float th[N_MAX];   /* θᵢ (radians, 0 = straight down) for i=0..n-1   */
    float om[N_MAX];   /* ωᵢ = θ̇ᵢ (radians/second)         for i=0..n-1   */
} StateN;

/* state_step_n — out = s + dt * k  (used by RK4 intermediate stages) */
static void state_step_n(int n, const StateN *s, float dt,
                         const StateN *k, StateN *out)
{
    for (int i = 0; i < n; i++) {
        out->th[i] = s->th[i] + dt * k->th[i];
        out->om[i] = s->om[i] + dt * k->om[i];
    }
}

/*
 * find_pivot_row — partial-pivot row index for column i.
 * Returns the row r ∈ [i, n) whose |M[r][i]| is largest — the most
 * numerically stable choice for the next elimination step.
 */
static int find_pivot_row(int n, float M[N_MAX][N_MAX], int i)
{
    int   piv  = i;
    float maxv = fabsf(M[i][i]);
    for (int r = i + 1; r < n; r++) {
        if (fabsf(M[r][i]) > maxv) {
            maxv = fabsf(M[r][i]);
            piv  = r;
        }
    }
    return piv;
}

/*
 * swap_rows_augmented — exchange two rows of the augmented system
 * [M | b].  Row swap on M alone changes the solution; swapping b in
 * lockstep keeps the system equivalent.
 */
static void swap_rows_augmented(int n, float M[N_MAX][N_MAX],
                                float b[N_MAX], int i, int j)
{
    for (int c = 0; c < n; c++) {
        float t = M[i][c]; M[i][c] = M[j][c]; M[j][c] = t;
    }
    float t = b[i]; b[i] = b[j]; b[j] = t;
}

/*
 * eliminate_below_pivot — zero column i in every row r > i by the
 * elementary row operation  rowₚ ← rowₚ − f · rowᵢ  where
 * f = M[r][i] / M[i][i].  The pivot row M[i][·] is left intact.
 */
static void eliminate_below_pivot(int n, float M[N_MAX][N_MAX],
                                  float b[N_MAX], int i)
{
    for (int r = i + 1; r < n; r++) {
        float f = M[r][i] / M[i][i];
        for (int c = i; c < n; c++) M[r][c] -= f * M[i][c];
        b[r] -= f * b[i];
    }
}

/*
 * back_substitute — solve the upper-triangular system left by the
 * forward elimination.  Walks bottom row up; at row i:
 *   x[i] = (b[i] − Σ_{j>i} M[i][j] · x[j]) / M[i][i]
 */
static void back_substitute(int n, float M[N_MAX][N_MAX],
                            float b[N_MAX], float x[N_MAX])
{
    for (int i = n - 1; i >= 0; i--) {
        float s = b[i];
        for (int j = i + 1; j < n; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
}

/*
 * solve_linear — in-place M·x = b by Gaussian elimination with partial
 * pivoting.  n ≤ N_MAX.  Stable for the well-conditioned mass matrices
 * deriv_n produces (M is symmetric positive-definite when the chain
 * is well-posed — i.e. no two consecutive links exactly aligned).
 *
 * Reads like the textbook recipe:
 *   1. forward elimination → upper-triangular
 *   2. back-substitution    → x
 *
 * Reference: Golub & Van Loan §3.4 (Gaussian elimination with partial
 * pivoting); Press et al. "Numerical Recipes" §2.2.
 */
static void solve_linear(int n, float M[N_MAX][N_MAX],
                         float b[N_MAX], float x[N_MAX])
{
    /* forward elimination: turn M into an upper-triangular U */
    for (int i = 0; i < n; i++) {
        int piv = find_pivot_row(n, M, i);
        if (piv != i) swap_rows_augmented(n, M, b, i, piv);
        eliminate_below_pivot(n, M, b, i);
    }
    /* back-substitute: solve U·x = b */
    back_substitute(n, M, b, x);
}

/*
 * copy_velocities_to_thetadot — trivial half of the EOM:  θ̇ = ω.
 * The state vector for an N-link chain in phase space is (θ, ω); the
 * derivative is (ω, α).  The "ω" component is just a copy.
 */
static void copy_velocities_to_thetadot(int n, const StateN *s, StateN *out)
{
    for (int i = 0; i < n; i++) out->th[i] = s->om[i];
}

/*
 * assemble_mass_matrix — fill the symmetric N×N matrix M(θ).
 *
 *   M[j][k] = (N − max(j, k)) · L² · cos(θⱼ − θₖ)
 *
 * The factor (N − max(j,k)) counts how many links of mass contribute
 * to the kinetic-energy cross-term between coordinates j and k; the
 * cosine couples their angular velocities via the geometric overlap.
 *
 * Reference: Goldstein 2001 §1.6 — Lagrangian mass matrix for chains
 * of rigid bodies; same construction as the finite-element "consistent
 * mass matrix".
 */
static void assemble_mass_matrix(int n, const StateN *s, float L,
                                 float M[N_MAX][N_MAX])
{
    for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            int   mx     = (j > k) ? j : k;
            float coef   = (float)(n - mx) * L * L;
            float th_jk  = s->th[j] - s->th[k];
            M[j][k] = coef * cosf(th_jk);
        }
    }
}

/*
 * assemble_force_vector — fill the generalised-force vector b(θ, ω).
 *
 *   b[j] = −Σₖ (N − max(j,k)) · L² · sin(θⱼ − θₖ) · ωₖ²    ← centrifugal
 *          − g · L · (N − j) · sin(θⱼ)                     ← gravity
 *
 * The first sum is the Coriolis / centrifugal coupling between links;
 * the second is the gravity moment on link j (which the (N−j) masses
 * below it all contribute to).
 */
static void assemble_force_vector(int n, const StateN *s, float L, float g,
                                  float b[N_MAX])
{
    for (int j = 0; j < n; j++) {
        b[j] = 0.0f;
        for (int k = 0; k < n; k++) {
            int   mx    = (j > k) ? j : k;
            float coef  = (float)(n - mx) * L * L;
            float th_jk = s->th[j] - s->th[k];
            b[j] -= coef * sinf(th_jk) * s->om[k] * s->om[k];
        }
        b[j] -= g * L * (float)(n - j) * sinf(s->th[j]);
    }
}

/*
 * deriv_n — equations of motion for the N-link pendulum.
 *
 * Returns ds/dt = (ω_i, α_i) where α = θ̈ are the angular accelerations
 * obtained by solving the Lagrangian linear system  M(θ) · α = b(θ, ω):
 *
 *     1.  θ̇ = ω                          (trivial half of phase space)
 *     2.  assemble M(θ)                   (mass matrix from Lagrangian T)
 *     3.  assemble b(θ, ω)                (Coriolis + gravity terms)
 *     4.  solve M · α = b                 (Gaussian elim + back-sub)
 *
 * Sanity checks (matches the textbook double-pendulum derivation):
 *   N=1:  M = [L²], b = [−gL sin θ₁]  →  α = −(g/L)·sin θ₁  ✓
 *   N=2:  same equations as double_pendulum.c's closed form  ✓
 *
 * Reference: Goldstein 2001 §1.6, Shinbrot et al. 1992.
 */
static void deriv_n(int n, const StateN *s, float L, float g, StateN *out)
{
    copy_velocities_to_thetadot(n, s, out);

    float M[N_MAX][N_MAX];
    float b[N_MAX];
    assemble_mass_matrix (n, s, L,    M);
    assemble_force_vector(n, s, L, g, b);

    /* α = M⁻¹ · b — written into out->om to complete the (ω, α) pair */
    solve_linear(n, M, b, out->om);
}

/*
 * rk4_slope_at — evaluate the derivative at s + dt_frac · prev_slope.
 *
 * The middle two RK4 stages probe the midpoint (dt_frac = dt/2); the
 * fourth probes the endpoint (dt_frac = dt).  Each uses the previous
 * stage's slope as the predictor.
 */
static void rk4_slope_at(int n, const StateN *s, float dt_frac,
                         const StateN *prev_slope,
                         float L, float g,
                         StateN *out_slope, StateN *scratch)
{
    state_step_n(n, s, dt_frac, prev_slope, scratch);
    deriv_n(n, scratch, L, g, out_slope);
}

/*
 * rk4_combine_simpson — Simpson's-rule weighted average of the four
 * slopes, applied to advance the state by one full dt:
 *
 *     s ← s + (dt / 6) · (k1 + 2·k2 + 2·k3 + k4)
 *
 * The (1, 2, 2, 1) weights come from approximating ∫₀^dt f(s(t)) dt
 * by Simpson's rule using the four sampled slopes.  This is what
 * gives classical RK4 its O(dt⁴) global accuracy.
 */
static void rk4_combine_simpson(int n, StateN *s, float dt,
                                const StateN *k1, const StateN *k2,
                                const StateN *k3, const StateN *k4)
{
    float w = dt / 6.0f;
    for (int i = 0; i < n; i++) {
        s->th[i] += w * (k1->th[i] + 2.0f*k2->th[i]
                                   + 2.0f*k3->th[i] + k4->th[i]);
        s->om[i] += w * (k1->om[i] + 2.0f*k2->om[i]
                                   + 2.0f*k3->om[i] + k4->om[i]);
    }
}

/*
 * rk4_step_n — one classical 4th-order Runge-Kutta step.
 *
 * Reads like the textbook recipe:
 *   k1 = slope at start
 *   k2 = slope at midpoint, using k1's prediction
 *   k3 = slope at midpoint, using k2's correction
 *   k4 = slope at endpoint, using k3's prediction
 *   s  = s + (dt/6) · (k1 + 2k2 + 2k3 + k4)            (Simpson weights)
 *
 * 4th-order globally accurate; for chaotic systems the lower orders
 * lose phase tracking within seconds.  See Hairer-Nørsett-Wanner 1993
 * for the order analysis.
 */
static void rk4_step_n(int n, StateN *s, float L, float g, float dt)
{
    /* Zero-initialise — only entries [0..n) are touched, but gcc can't
     * see that and emits maybe-uninitialised warnings otherwise.        */
    StateN k1, k2, k3, k4, tmp;
    memset(&k1,  0, sizeof k1);
    memset(&k2,  0, sizeof k2);
    memset(&k3,  0, sizeof k3);
    memset(&k4,  0, sizeof k4);
    memset(&tmp, 0, sizeof tmp);

    deriv_n(n, s, L, g, &k1);                                  /* k1: start    */
    rk4_slope_at(n, s, 0.5f * dt, &k1, L, g, &k2, &tmp);       /* k2: mid (k1) */
    rk4_slope_at(n, s, 0.5f * dt, &k2, L, g, &k3, &tmp);       /* k3: mid (k2) */
    rk4_slope_at(n, s, dt,        &k3, L, g, &k4, &tmp);       /* k4: endpoint */

    rk4_combine_simpson(n, s, dt, &k1, &k2, &k3, &k4);
}

/* ─────────────────────────────────────────────────────────────────────
 * NPend — one complete N-link pendulum chain.
 *
 * Bundles the dynamical state (s, prev) with the fixed geometry
 * (pivot, arm_len) and the topology field (n).  Two NPends live in
 * Scene: the primary chain the user sees, and a "ghost" started with
 * θ₁ + GHOST_EPSILON to visualise chaotic divergence.
 *
 * Members are grouped by access pattern:
 *
 *   simulation state  : s, prev
 *       Hot path.  Read+written every RK4 step via rk4_step_n.
 *       npend_tick snapshots s into prev BEFORE the integration so the
 *       renderer can lerp between (prev, s) by alpha for smooth motion
 *       at any frame rate.  This is the standard "fixed-timestep +
 *       render interpolation" trick (Fiedler 2004).
 *
 *   geometry          : pivot_px, pivot_py, arm_len
 *       Set once by npend_init from the screen size and the chosen N,
 *       constant for the chain's lifetime.  arm_len = MAX_REACH_FRAC ·
 *       screen_height / N — every link the same length, total chain
 *       reach the same fraction of the screen regardless of N.
 *
 *   topology          : n
 *       Active link count.  Set at init; controls how many slots of
 *       s.th[]/s.om[] are alive.  Changing n requires a fresh
 *       npend_init (which scene_init handles when the user hits n/p).
 *
 * References:
 *   • Shinbrot, Grebogi, Wisdom & Yorke 1992 — defines the canonical
 *     double-pendulum experimental setup; this struct is the N-link
 *     generalisation.
 *   • Goldstein 2001 §1.4-1.6 — Lagrangian state representation that
 *     the (s, prev) pair encodes.
 *   • Fiedler, G. (2004) "Fix Your Timestep" — the lerp between prev
 *     and current state is the technique we use for `alpha`.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ simulation state — hot path, read+write every tick ─ */
    StateN  s;             /* current (θ, ω) for all n links            */
    StateN  prev;          /* snapshot taken at the start of the current
                              physics tick; renderer lerps (prev, s) by
                              alpha for smooth sub-tick motion          */

    /* ─ geometry — set once at npend_init, constant after ─ */
    float   pivot_px;      /* anchor x in pixel space (chain top)       */
    float   pivot_py;      /* anchor y in pixel space                   */
    float   arm_len;       /* link length L (px); all arms equal so
                              chain total = MAX_REACH_FRAC × screen_h   */

    /* ─ topology — set at init, read every tick ─ */
    int     n;             /* active link count, 1..N_MAX               */
} NPend;

/*
 * place_pivot_at_screen_center — anchor the chain at the centre of the
 * pixel-space terminal so the swing has room above and below.
 */
static void place_pivot_at_screen_center(NPend *p, int cols, int rows)
{
    p->pivot_px = (float)pw(cols) * 0.5f;
    p->pivot_py = (float)ph(rows) * 0.5f;
}

/*
 * size_arms_for_n_links — each link L = MAX_REACH_FRAC · screen_h / N,
 * so the fully-extended chain occupies the SAME fraction of the screen
 * regardless of N.  N=1 → one big arm; N=5 → five short arms with the
 * same overall reach.
 */
static void size_arms_for_n_links(NPend *p, int n_links, int rows)
{
    p->arm_len = (float)ph(rows) * MAX_REACH_FRAC / (float)n_links;
}

/*
 * set_uniform_initial_angles — seed every link with the same angle
 * (INIT_THETA_DEG = 120°) and zero angular velocity.  The chain
 * starts fully extended at that angle — a high-energy configuration
 * that hits chaotic onset within ~1 s for N ≥ 2.
 */
static void set_uniform_initial_angles(NPend *p, int n_links, float theta_deg)
{
    float theta_rad = (float)(theta_deg * M_PI / 180.0);
    for (int i = 0; i < n_links; i++) {
        p->s.th[i] = theta_rad;
        p->s.om[i] = 0.0f;
    }
}

/*
 * perturb_first_angle — add `extra_deg` to θ₀ only.  Used to seed the
 * "ghost" pendulum with GHOST_EPSILON ≈ 0.057°.  After ~3-5 s (N ≥ 2)
 * this nanoscopic difference grows exponentially and the two chains
 * diverge completely — the visual signature of chaos.
 *
 * For the primary pendulum the caller passes 0.0f, leaving angles
 * untouched.
 */
static void perturb_first_angle(NPend *p, float extra_deg)
{
    p->s.th[0] += (float)(extra_deg * M_PI / 180.0);
}

/*
 * npend_init — fresh pendulum at the boot configuration.
 *
 *   1. record N
 *   2. anchor pivot at screen centre
 *   3. size arms so total reach fits
 *   4. seed every angle to INIT_THETA_DEG, zero velocities
 *   5. (optional) perturb θ₀ — used to spawn the ghost
 *   6. snapshot prev = s so the first render lerp has both ends
 */
static void npend_init(NPend *p, int n_links, int cols, int rows,
                       float th_extra_deg)
{
    p->n = n_links;
    place_pivot_at_screen_center(p, cols, rows);
    size_arms_for_n_links       (p, n_links, rows);
    set_uniform_initial_angles  (p, n_links, INIT_THETA_DEG);
    perturb_first_angle         (p, th_extra_deg);
    p->prev = p->s;
}

static void npend_tick(NPend *p, float dt)
{
    p->prev = p->s;
    rk4_step_n(p->n, &p->s, p->arm_len, GRAVITY_PX, dt);
}

/*
 * npend_positions — joint pixel coordinates for given (interpolated)
 * angles th[0..n-1].
 *
 *   xs[0], ys[0]  = pivot
 *   xs[i+1] = xs[i] + L · sin(θᵢ)
 *   ys[i+1] = ys[i] + L · cos(θᵢ)      (y axis points DOWN)
 *
 * The end bob sits at xs[n], ys[n].
 */
static void npend_positions(const NPend *p, const float *th,
                            float *xs, float *ys)
{
    xs[0] = p->pivot_px;
    ys[0] = p->pivot_py;
    for (int i = 0; i < p->n; i++) {
        xs[i + 1] = xs[i] + p->arm_len * sinf(th[i]);
        ys[i + 1] = ys[i] + p->arm_len * cosf(th[i]);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────
 * Trail — ring buffer of past end-bob pixel positions.
 *
 * The end bob traces a chaotic curve through space; the trail is what
 * makes the attractor geometry visible.  A ring buffer is the right
 * data structure because every tick pushes ONE new sample and the
 * oldest expires automatically — O(1) per push, no shifting, no
 * allocation, no enumeration of "live" entries.
 *
 * Semantics:
 *   head     index of the NEXT slot to write   (0..TRAIL_LEN-1)
 *   count    number of valid entries already written, saturates at
 *            TRAIL_LEN once the buffer fills
 *
 * Iterate oldest → newest in the live window:
 *     start = (head − count + TRAIL_LEN) % TRAIL_LEN
 *     for k = 0..count-1:  idx = (start + k) % TRAIL_LEN
 *
 * Scene::trail_draw further limits how many of those `count` entries
 * paint each frame (user-adjustable via +/-), so a long buffer is
 * always available but the user controls the visual tail length
 * independently.
 *
 * Reference:
 *   • Knuth, TAOCP Vol. 1 §2.2.2 — sequential allocation with circular
 *     queues.  Standard ring-buffer construction.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    float px[TRAIL_LEN];  /* recorded x position (pixel space)         */
    float py[TRAIL_LEN];  /* recorded y position (pixel space)         */
    int   head;           /* next-write index (advances mod TRAIL_LEN) */
    int   count;          /* live entries, 0..TRAIL_LEN (saturates)    */
} Trail;

static void trail_push(Trail *t, float px, float py)
{
    t->px[t->head] = px;
    t->py[t->head] = py;
    t->head = (t->head + 1) % TRAIL_LEN;
    if (t->count < TRAIL_LEN) t->count++;
}

static void trail_clear(Trail *t) { t->head = 0; t->count = 0; }

/* ─────────────────────────────────────────────────────────────────────
 * Scene — the entire demo state.
 *
 * One Scene IS the demo.  Every non-trivial function takes a Scene*
 * (or const Scene* for read-only) and does its work on it; the only
 * mutable state outside Scene is the two signal-handler flags in §8
 * (which must be globals because the C signal API has no context
 * pointer).
 *
 * Members are grouped by what TOUCHES them, separating SIMULATION-side
 * concerns from RENDER-only concerns per the locality principle:
 *
 *   ── SIMULATION ───────────────────────────────────────────────────
 *
 *   simulation pendulums   : primary, ghost
 *       Two NPends running the same equations of motion at the same N
 *       and arm_len, but the ghost starts θ₁ + GHOST_EPSILON (≈ 0.057°)
 *       ahead of the primary.  scene_tick steps BOTH each tick.  Their
 *       angular divergence — shown in the HUD as `div:` — is the visual
 *       evidence of sensitive dependence on initial conditions for
 *       N ≥ 2 (chaos).  For N = 1 the two stay glued; pendulum 1 is
 *       integrable, not chaotic.
 *
 *   simulation parameters  : n_links
 *       The N for both pendulums (always kept in sync).  Set only at
 *       scene_init; changing N requires a fresh scene_init (which the
 *       n/p key handler does).  Read every tick by deriv_n.
 *
 *   simulation switch      : paused
 *       Treated as simulation state because it gates the integrator
 *       (scene_tick early-returns when true), not the renderer.  Set
 *       by app_handle_key, read by scene_tick.
 *
 *   ── RENDERING ────────────────────────────────────────────────────
 *
 *   render-side history    : trail
 *       Ring buffer of the primary end-bob's past pixel positions.
 *       Written once per tick by scene_tick (right after the integrator
 *       update), read every frame by draw_trail.  Purely a render
 *       artefact — the simulation never reads it.
 *
 *   screen dimensions      : cols, rows
 *       Cell-space terminal size, refreshed by app_do_resize after
 *       SIGWINCH.  Read everywhere in the render code, never written
 *       outside the resize path.
 *
 *   render-only parameters : theme, show_ghost, show_trail, trail_draw
 *       Pure presentation knobs — none feed back into the integrator.
 *       Set by app_handle_key, read by the renderers.  Cycling theme
 *       calls theme_apply but does not perturb any physical state.
 *
 * References:
 *   • Shinbrot, Grebogi, Wisdom & Yorke 1992 — the primary/ghost
 *     comparison-pendulum setup this Scene implements.  See the
 *     CONCEPTS header for the full reference list.
 *   • Goldstein 2001 — Lagrangian state representation used in
 *     primary/ghost (each NPend holds one (θ, ω) pair).
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ─ SIMULATION ──────────────────────────────────────────────────
     * Pendulum chains and the integrator gate.                       */
    NPend   primary;       /* the chain rendered in full theme colour  */
    NPend   ghost;         /* identical NPend, θ₁ + GHOST_EPSILON;
                              dim render — the chaos divergence demo   */
    int     n_links;       /* shared N for both pendulums, 1..N_MAX     */
    bool    paused;        /* spc: scene_tick is a no-op when true      */

    /* ─ RENDERING ───────────────────────────────────────────────────
     * Pure presentation state — never feeds back into the integrator.*/
    Trail   trail;         /* ring buffer of primary end-bob positions  */
    int     cols, rows;    /* cell-space terminal size (post-SIGWINCH)  */
    int     theme;         /* t/T: index into k_themes[]                */
    bool    show_ghost;    /* g  : toggle ghost-pendulum rendering      */
    bool    show_trail;    /* l  : toggle end-bob trail rendering       */
    int     trail_draw;    /* +/-: how many recorded samples to paint   */
} Scene;

static void scene_init(Scene *s, int n_links, int cols, int rows)
{
    if (n_links < 1)     n_links = 1;
    if (n_links > N_MAX) n_links = N_MAX;

    s->cols       = cols;
    s->rows       = rows;
    s->n_links    = n_links;
    s->theme      = 0;
    s->paused     = false;
    s->show_ghost = true;
    s->show_trail = true;
    s->trail_draw = TRAIL_DEF;

    npend_init(&s->primary, n_links, cols, rows, 0.0f);
    /* Ghost: same setup but θ₁ shifted by GHOST_EPSILON rad. */
    npend_init(&s->ghost,   n_links, cols, rows,
               (float)(GHOST_EPSILON * 180.0 / M_PI));

    trail_clear(&s->trail);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    npend_tick(&s->primary, dt);
    npend_tick(&s->ghost,   dt);

    /* Record the end-bob position into the trail. */
    float xs[N_MAX + 1], ys[N_MAX + 1];
    npend_positions(&s->primary, s->primary.s.th, xs, ys);
    trail_push(&s->trail, xs[s->n_links], ys[s->n_links]);
}

/* ── draw helpers ──────────────────────────────────────────────────── */

/*
 * draw_line — Bresenham line; character chosen by step direction.
 * Identical to double_pendulum.c.
 */
static void draw_line(int x0, int y0, int x1, int y1,
                      int cols, int rows, attr_t attr)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows) {
            int  e2     = 2 * err;
            bool step_x = (e2 > -dy);
            bool step_y = (e2 <  dx);
            chtype ch;
            if      (step_x && step_y) ch = (sx == sy) ? '\\' : '/';
            else if (step_x)           ch = '-';
            else                       ch = '|';
            attron(attr);
            mvaddch(y0, x0, ch);
            attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* interp_angles — lerp prev→current angles by alpha for smooth render. */
static void interp_angles(const NPend *p, float alpha, float *out)
{
    for (int i = 0; i < p->n; i++)
        out[i] = p->prev.th[i] + (p->s.th[i] - p->prev.th[i]) * alpha;
}

/*
 * draw_pivot_marker — '[+]' bracket at the chain's anchor.
 */
static void draw_pivot_marker(const NPend *p, int cols, int rows)
{
    int cx = px_to_cell_x(p->pivot_px);
    int cy = px_to_cell_y(p->pivot_py);
    if (cx >= 1 && cx < cols - 1 && cy >= 0 && cy < rows) {
        attron(COLOR_PAIR(CP_BAR) | A_BOLD);
        mvaddch(cy, cx - 1, '[');
        mvaddch(cy, cx,     '+');
        mvaddch(cy, cx + 1, ']');
        attroff(COLOR_PAIR(CP_BAR) | A_BOLD);
    }
}

/*
 * draw_trail — three-tier coloured tail: newest 30% bright red, middle
 * 35% orange, oldest 35% dim grey.  Gives a visually fading tail
 * without per-entry alpha.
 */
static void draw_trail(const Scene *s)
{
    if (!s->show_trail || s->trail.count == 0) return;

    int draw = s->trail_draw;
    if (draw > s->trail.count) draw = s->trail.count;
    int start = (s->trail.head - draw + TRAIL_LEN) % TRAIL_LEN;

    for (int i = 0; i < draw; i++) {
        int idx = (start + i) % TRAIL_LEN;
        int cx  = px_to_cell_x(s->trail.px[idx]);
        int cy  = px_to_cell_y(s->trail.py[idx]);
        if (cx < 0 || cx >= s->cols || cy < 0 || cy >= s->rows) continue;

        float age = (float)i / (float)draw;   /* 0=oldest, 1=newest */
        int    cp;  chtype ch;
        if      (age > 0.70f) { cp = CP_TR1; ch = 'o'; }
        else if (age > 0.35f) { cp = CP_TR2; ch = '.'; }
        else                  { cp = CP_TR3; ch = ','; }

        attron(COLOR_PAIR(cp));
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp));
    }
}

/*
 * interpolated_joint_positions — render-time forward kinematics.
 *
 * Lerps the chain's previous and current angles by alpha (so the
 * display reads smoothly even at frame rates much lower than the
 * physics rate), then walks the chain to fill xs[]/ys[] with the
 * pixel-space positions of the pivot (index 0) and every joint
 * (indices 1..n).  xs[n], ys[n] is the end bob.
 */
static void interpolated_joint_positions(const NPend *p, float alpha,
                                         float *xs, float *ys)
{
    float th[N_MAX] = {0};
    interp_angles(p, alpha, th);
    npend_positions(p, th, xs, ys);
}

/*
 * draw_arm_segments — paint the n line segments between consecutive
 * joints.  Each arm uses its own colour (theme-driven cycle via
 * arm_pair) for the primary chain; the ghost chain paints all arms
 * in a single dim theme-tinted colour.
 */
static void draw_arm_segments(const NPend *p, const float *xs, const float *ys,
                              int cols, int rows, bool is_ghost)
{
    for (int i = 0; i < p->n; i++) {
        int cx0 = px_to_cell_x(xs[i]),     cy0 = px_to_cell_y(ys[i]);
        int cx1 = px_to_cell_x(xs[i + 1]), cy1 = px_to_cell_y(ys[i + 1]);
        attr_t a = is_ghost ? (attr_t)COLOR_PAIR(CP_GHOST)
                            : (attr_t)(COLOR_PAIR(arm_pair(i)) | A_BOLD);
        draw_line(cx0, cy0, cx1, cy1, cols, rows, a);
    }
}

/*
 * draw_intermediate_joints — 'O' markers at every joint EXCEPT the
 * pivot (drawn elsewhere as a [+] bracket) and the end bob (drawn as
 * (@) below).  Skipped entirely for the ghost chain to keep it dim.
 */
static void draw_intermediate_joints(const NPend *p,
                                     const float *xs, const float *ys,
                                     int cols, int rows)
{
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
 * draw_end_bob — final joint marker.
 *   primary: bright bracketed (@) — the eye should track this.
 *   ghost  : single dim 'x'      — readable but subordinate.
 */
static void draw_end_bob(const NPend *p, const float *xs, const float *ys,
                         int cols, int rows, bool is_ghost)
{
    int b_cx = px_to_cell_x(xs[p->n]);
    int b_cy = px_to_cell_y(ys[p->n]);
    if (b_cy <= 0 || b_cy >= rows) return;

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
        mvaddch(b_cy, b_cx,     '@');
        mvaddch(b_cy, b_cx + 1, ')');
    } else if (b_cx >= 0 && b_cx < cols) {
        mvaddch(b_cy, b_cx, '@');
    }
    attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
}

/*
 * draw_chain — orchestrator: kinematics → arms → joints → end bob.
 * Three passes so each draw uses the same xs/ys snapshot; nothing
 * shifts mid-frame.
 */
static void draw_chain(const NPend *p, float alpha,
                       int cols, int rows, bool is_ghost)
{
    float xs[N_MAX + 1], ys[N_MAX + 1];
    interpolated_joint_positions(p, alpha, xs, ys);

    draw_arm_segments(p, xs, ys, cols, rows, is_ghost);
    if (!is_ghost)
        draw_intermediate_joints(p, xs, ys, cols, rows);
    draw_end_bob(p, xs, ys, cols, rows, is_ghost);
}

/*
 * scene_draw — render one frame with render-interpolation alpha.
 *
 * Draw order (back → front so key elements overwrite background detail):
 *   1. Pivot marker
 *   2. Trail (oldest → newest, three brightness tiers)
 *   3. Ghost chain (if enabled) — drawn dim under the primary
 *   4. Primary chain — arms, intermediate joints, end bob
 */
static void scene_draw(const Scene *s, float alpha)
{
    draw_pivot_marker(&s->primary, s->cols, s->rows);
    draw_trail(s);
    if (s->show_ghost)
        draw_chain(&s->ghost, alpha, s->cols, s->rows, /*is_ghost=*/true);
    draw_chain(&s->primary,   alpha, s->cols, s->rows, /*is_ghost=*/false);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * pendulum_count_name — friendly name for the chain length used in the
 * HUD: "single", "double", "triple", "quadruple", "quintuple".
 */
static const char *pendulum_count_name(int n)
{
    static const char *names[N_MAX] = {
        "single", "double", "triple", "quadruple", "quintuple",
    };
    if (n < 1)         return names[0];
    if (n > N_MAX)     return names[N_MAX - 1];
    return names[n - 1];
}

/*
 * draw_hud_top — canonical top status bar (row 0).
 * Right-aligned: fps + chain name + end-bob angle + ghost divergence +
 * trail length + theme + paused.  CP_HUD (bright yellow + bold).
 */
static void draw_hud_top(Screen *s, const Scene *sc, double fps)
{
    const NPend *p = &sc->primary;
    const NPend *g = &sc->ghost;

    /* Total angular divergence — sum of |Δθᵢ| in degrees. */
    float div_deg = 0.0f;
    for (int i = 0; i < p->n; i++)
        div_deg += fabsf(p->s.th[i] - g->s.th[i]);
    div_deg *= (float)(180.0 / M_PI);
    if (div_deg > 9999.0f) div_deg = 9999.0f;

    /* End-bob angle for the HUD (most informative single number). */
    float th_end_deg = p->s.th[p->n - 1] * (float)(180.0 / M_PI);

    char buf[220];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %s pendulum (N=%d)  t_end:%+7.1f  div:%6.1f  "
             "tr:%d  theme:%s  %s ",
             fps, pendulum_count_name(p->n), p->n,
             th_end_deg, div_deg, sc->trail_draw,
             k_themes[sc->theme].name,
             sc->paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    int col = s->cols - len;
    if (col < 0) col = 0;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, col, buf, s->cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_bottom — canonical bottom action bar (row -1).
 * Left-aligned key list; short fallback if the terminal is narrow.
 * CP_HINT (bright cyan + bold).
 */
static void draw_hud_bottom(Screen *s)
{
    const char *hint_full =
        " q:quit  spc:pause  r:reset  n/p:N  t/T:theme  "
        "g:ghost  l:trail  +/-:tail  ]/[:fps ";
    const char *hint_short =
        " q:quit  spc:pause  n/p:N  t:theme  l:trail ";
    const char *hint = hint_full;
    if ((int)strlen(hint_full) >= s->cols - 1) hint = hint_short;

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvaddnstr(s->rows - 1, 0, hint, s->cols);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, float alpha)
{
    erase();
    scene_draw(sc, alpha);
    draw_hud_top(s, sc, fps);
    draw_hud_bottom(s);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->scene.n_links,
               app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, sc->n_links, app->screen.cols, app->screen.rows);
        break;

    case 'n': case 'N': {
        int nn = sc->n_links + 1;
        if (nn > N_MAX) nn = N_MAX;
        scene_init(sc, nn, app->screen.cols, app->screen.rows);
        break;
    }
    case 'p': case 'P': {
        int nn = sc->n_links - 1;
        if (nn < 1) nn = 1;
        scene_init(sc, nn, app->screen.cols, app->screen.rows);
        break;
    }

    case 'g': case 'G':
        sc->show_ghost = !sc->show_ghost;
        break;

    case 'l': case 'L':
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

    case '+': case '=':
        sc->trail_draw += TRAIL_STEP;
        if (sc->trail_draw > TRAIL_LEN) sc->trail_draw = TRAIL_LEN;
        break;

    case '-':
        sc->trail_draw -= TRAIL_STEP;
        if (sc->trail_draw < TRAIL_MIN) sc->trail_draw = TRAIL_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app      = &g_app;
    app->running  = 1;
    app->sim_fps  = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, N_DEFAULT,
               app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
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
