/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rk_method_comparision.c
 *
 * ODE Integrator Comparison — Euler / RK2 / RK4 / Verlet
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Two panels run simultaneously from the same initial condition:
 *
 *    LEFT  — Phase portrait  (position q  vs  velocity p)
 *    RIGHT — Time series     (q(t), scrolling left)
 *
 *  Four integrators advance in parallel.  A high-accuracy reference
 *  solution (RK4, 32 micro-substeps, shown in white) is the ground truth.
 *
 *    Euler  (red)    — orbit spirals outward  : energy grows without bound
 *    RK2    (yellow) — slight inward drift    : amplitude slowly decays
 *    RK4    (green)  — indistinguishable from reference at moderate dt
 *    Verlet (cyan)   — orbit stays closed     : no long-term energy drift
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — INTEGRATION ORDER
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Each step of an integrator introduces a local truncation error (LTE).
 *  After N steps covering total time T = N·h the global error scales as:
 *
 *    Euler:   LTE = O(h²)  →  global error = O(h)
 *             Keeps one term of the Taylor series.
 *
 *    RK2:     LTE = O(h³)  →  global error = O(h²)
 *             Midpoint correction cancels the O(h) term.
 *
 *    RK4:     LTE = O(h⁵)  →  global error = O(h⁴)
 *             Four stages cancel through the h⁴ term.
 *             Halving h shrinks error by 16× — very fast convergence.
 *
 *    Verlet:  LTE = O(h³)  →  global error = O(h²)
 *             Same asymptotic order as RK2 but symplectic (see below).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — STABILITY LIMITS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  For the test ODE dy/dt = λy, stability requires |R(λh)| ≤ 1 where
 *  R is the amplification factor.  For a harmonic oscillator λ = ±iω,
 *  so we need |R(iωh)| ≤ 1.
 *
 *    Euler:   |R(iωh)| = √(1 + ω²h²) > 1  always.
 *             Euler is unconditionally UNSTABLE for oscillators.
 *             Any h > 0 causes the orbit to spiral out.
 *
 *    RK2:     stable when ωh ≤ √2  ≈ 1.41
 *    RK4:     stable when ωh ≤ 2√2 ≈ 2.83
 *    Verlet:  stable when ωh ≤ 2   (same as leapfrog)
 *
 *  Press + to raise dt and watch Euler blow up first, then RK2.
 *  At ωh = 0.3 (default): Euler drifts slowly, others look fine.
 *  At ωh = 1.0: Euler spirals rapidly off screen within seconds.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — SYMPLECTIC INTEGRATORS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A symplectic integrator preserves the phase-space volume element
 *  dq ∧ dp  (Liouville's theorem).  For Hamiltonian systems this means:
 *
 *    — No secular energy drift:  E(t) oscillates near E(0) forever.
 *      Euler gains energy; RK2/RK4 lose it slowly.
 *    — The computed orbit lies on a slightly perturbed Hamiltonian's
 *      exact trajectory (shadow orbit property).
 *    — Critical for long-time simulation: solar system, molecular
 *      dynamics, particle accelerators.
 *
 *  Velocity Verlet IS symplectic.  RK4 is NOT.  At moderate dt for
 *  short runs they look identical — but run for 10⁴+ periods and only
 *  Verlet's orbit remains closed.
 *
 *  Velocity Verlet algorithm for H = ½p² + V(q):
 *    q_{n+1} = q_n + p_n·h + ½·a_n·h²
 *    a_{n+1} = -∂V/∂q at q_{n+1}
 *    p_{n+1} = p_n + ½·(a_n + a_{n+1})·h
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — ACCURACY DIFFERENCE DEMONSTRATED
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Phase portrait reveals the character of the error:
 *
 *    Euler:   orbit drifts outward — total mechanical energy grows.
 *             E(t) = E₀·(1 + ω²h²)^n  →  ∞ as n → ∞.
 *             Position error grows unboundedly.
 *
 *    RK2:     orbit drifts inward — amplitude slowly decays.
 *             |R(iωh)| = √(1 - ω⁴h⁴/4) < 1 for ωh < 2^½.
 *             Not symplectic — energy leaks to numerical dissipation.
 *
 *    RK4:     orbit practically closed — error only visible at large dt.
 *             Still not symplectic: very slow energy drift over long runs.
 *
 *    Verlet:  orbit perfectly closed — phase portrait is a fixed ellipse.
 *             Energy oscillates O(h²) around E₀, no secular trend.
 *             Same positional accuracy as RK2 but structurally superior.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Keys:
 *    q/ESC    quit               space    pause/resume
 *    s        switch system      1-4      toggle Euler/RK2/RK4/Verlet
 *    +/-      increase/decrease timestep dt
 *    r        reset simulation   t/T      cycle theme (10 palettes)
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/rk_method_comparision.c \
 *        -o rk_compare -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1  config           §2  clock            §3  color / themes
 *  §4  ode              — State, Method, ODE structs + Hamiltonian primitives
 *  §5  integrators      — forward_euler, midpoint, classical_rk4,
 *                          velocity_verlet, reference, dispatch +
 *                          ode_advance_one_tick pipeline
 *  §6  error analysis   — compute_method_errors, energy_drift_pct
 *  §7  plot panels      — render_phase_portrait, render_time_series
 *  §8  HUD overlay      — top status bar + per-method lines + bottom hints
 *  §9  scene            — Scene struct, scene_init / _tick / _render / _key
 *  §10 screen           §11 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* DT_DEFAULT = 0.05: ωh = 0.05 rad (with ω=1 rad/s default).
 * Well within the stability limit for all four methods.
 * Small enough that drift is slow and clearly visible over many seconds. */
#define DT_DEFAULT 0.05f

/* DT_MIN = 0.005: smallest timestep the user can dial down to.
 * Prevents animation from becoming uselessly slow (too many tiny steps);
 * differences between methods are still visible even at this fine scale. */
#define DT_MIN 0.005f

/* DT_MAX = 0.35: ωh = 0.35 rad at the ceiling.
 * For Euler: |R(iωh)| = √(1 + 0.35²) ≈ √1.1225 ≈ 1.060 per step
 * → orbit amplitude grows by 6% every step — exponential blow-up is
 * dramatically visible.  RK2 remains stable up to ωh ≈ 1.41, so it
 * still tracks the reference while Euler flies off screen. */
#define DT_MAX 0.35f

/* DT_STEP = 0.005: increment/decrement per + or - keypress.
 * Fine enough for interactive control; each press changes ωh by 0.005,
 * letting the user nudge into or out of each method's stability region. */
#define DT_STEP 0.005f

/* OMEGA = 1.0 rad/s: angular frequency of the harmonic oscillator.
 * Period T = 2π/ω = 2π ≈ 6.28 s.  Setting ω=1 makes "ωh" numerically
 * equal to dt, simplifying the stability analysis in the header. */
#define OMEGA 1.0f /* oscillator angular frequency (rad/s) */

/* G_OVER_L = 1.0: effective gravity / pendulum length = 1 (unit pendulum).
 * At small angles the pendulum period = 2π√(L/g) = 2π√(1/1) = 2π s,
 * matching the harmonic oscillator period.  At Q0_PEND = 1.40 rad the
 * nonlinear term makes the actual period noticeably longer than 2π. */
#define G_OVER_L 1.0f /* pendulum g/L (unit pendulum) */

/* Q0_OSC = 1.0: initial displacement of the harmonic oscillator.
 * With p0=0 this sets the orbit radius in phase space: the exact
 * solution traces a circle of radius 1 in the (q, p/ω) plane. */
#define Q0_OSC 1.0f /* oscillator initial displacement */

/* Q0_PEND = 1.40 rad ≈ 80°: initial angle for the pendulum.
 * This is deliberately a LARGE angle so nonlinear effects dominate.
 * The small-angle approximation (period ≈ 2π) is noticeably wrong here:
 * the exact period is longer, visible as a phase lag vs the oscillator.
 * The pendulum is NOT a simple sine wave at this amplitude. */
#define Q0_PEND 1.40f /* pendulum initial angle (rad), ~80 deg */

/* REF_SUBSTEPS = 32: the reference integrator runs RK4 with 32 sub-steps
 * of size h/32 for each display step h.  Global error ≈ O((h/32)⁴)
 * ≈ O(h⁴ / 10⁶) — effectively machine-precision for these parameters.
 * All four methods are compared against this "exact" solution. */
#define REF_SUBSTEPS 32

/* STEPS_PER_FRAME = 3: the simulation advances 3 × dt per rendered frame.
 * At 30 fps this gives 3 × 0.05 = 0.15 sim-seconds per wall-second,
 * so one full oscillator period (T ≈ 6.28 s) takes ~42 wall seconds —
 * slow enough to watch drift accumulate in real time. */
#define STEPS_PER_FRAME 3
#define TARGET_FPS 30
#define FRAME_US (1000000 / TARGET_FPS)

/* TRAJ_LEN = 1024: ring buffer capacity for phase-portrait trail dots.
 * 1024 past states are kept per method.  Too short → orbit looks partial
 * (less than one full loop visible).  Too long → drawing loop slows down.
 * Ring buffer means the oldest entry is silently overwritten; no malloc. */
#define TRAJ_LEN 1024 /* phase portrait history per method */

/* TIME_LEN = 512: ring buffer capacity for the time-series panel.
 * Each stored sample maps to one terminal column, so 512 should match
 * or exceed the width of a typical wide terminal (e.g. 220 columns).
 * Same ring-buffer overwrite strategy as TRAJ_LEN. */
#define TIME_LEN 512 /* time-series samples per method */

/* HUD_TOP_ROWS = 5 — terminal rows consumed by the top overlay:
 *   row 0       — top status bar (CP_HUD, bright yellow + bold,
 *                  right-aligned).  Carries system / dt / t / theme /
 *                  fps / paused.
 *   rows 1..4   — one per integrator (M_EULER..M_VERLET).  Each row is
 *                  drawn in that method's color and shows err_cur,
 *                  err_max, E_drift, and the method's accuracy order.
 *
 * HUD_BOT_ROWS = 1 — terminal rows consumed by the bottom overlay:
 *   row rows-1  — action key bar (CP_HINT, bright cyan + bold).
 *
 * Plot area occupies rows HUD_TOP_ROWS .. rows-HUD_BOT_ROWS-1. */
#define HUD_TOP_ROWS 5
#define HUD_BOT_ROWS 1

/* N_THEMES = 10 — palettes in k_themes[] (§3).  t / T keys wrap modulo
 * this; must match the literal array length. */
#define N_THEMES 10

/* N_METHODS = 4 — number of integrators compared side-by-side.  Defined
 * up here (rather than down in §4 with M_EULER..M_VERLET) because §3
 * theme_apply needs to iterate methods[] when installing a palette. */
#define N_METHODS 4

/* PHASE_SCALE_OSC = 3.0: the phase-portrait viewport spans
 * ±(Q0_OSC × 3.0) = ±3.0 in both q and p (scaled by ω).
 * The extra headroom (3× the initial amplitude) is needed so Euler's
 * exponentially growing orbit stays on screen for a useful demo time. */
#define PHASE_SCALE_OSC 3.0f

/* PHASE_SCALE_PEND = 1.6: pendulum viewport spans ±(Q0_PEND × 1.6).
 * The pendulum's position is bounded by ±Q0_PEND (it can't swing past
 * its release angle), so less headroom is needed than for the oscillator.
 * Velocity range is also bounded (energy conservation), so 1.6× suffices. */
#define PHASE_SCALE_PEND 1.6f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long now_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000L + ts.tv_nsec / 1000L;
}

/* ===================================================================== */
/* §3  color / theme — 10 brightness-safe palettes + fixed HUD chrome    */
/* ===================================================================== */

/*
 * Color-pair layout (§3):
 *   CP_M0..CP_M3   per-integrator colors — THEME-DRIVEN, dim → bright
 *                  ramp matched to physical "worst→best" (Euler dimmest,
 *                  Verlet brightest).  Cycling themes only repaints
 *                  these pairs — the State of every integrator stays
 *                  byte-identical, so theme is a render param (§9 Scene).
 *   CP_REF         the reference solution ('*' dots) — theme-driven,
 *                  always picked as the theme's brightest accent so it
 *                  reads on top of every method ramp.
 *   CP_GRID        axes / panel separators / inactive labels — dim grey
 *                  that doesn't compete with the integrator data.
 *   CP_TITLE       panel titles ("Phase Portrait", "Time Series") —
 *                  theme-driven, bright accent.
 *   CP_HUD         top status bar — FIXED bright yellow + bold,
 *                  theme-INDEPENDENT (CLAUDE.md HUD standard).
 *   CP_HINT        bottom action bar — FIXED bright cyan + bold,
 *                  theme-INDEPENDENT.
 */
enum {
  CP_M0 = 1,
  CP_M1,
  CP_M2,
  CP_M3, /* method colors, dim → bright */
  CP_REF,
  CP_GRID,
  CP_TITLE,
  CP_HUD,
  CP_HINT,
};

/*
 * Theme — perceptual palette for one rendering of the demo.
 *
 * The methods[4] ramp is monotonic DIM → BRIGHT and indexed by method
 * id (M_EULER=0 .. M_VERLET=3).  Mapping the "worst" integrator to the
 * dimmest stop is deliberate: Euler is the loud cautionary tale and
 * benefits visually from being subdued, while Verlet (best) reads
 * crisply.  This matches the original red→cyan semantic — red is the
 * dimmest of the 8-color palette by perceptual luminance.
 *
 * Brightness safety (CLAUDE.md):
 *   - methods[0]: lowest tier; allowed in 24-29 / 240-243 only
 *   - methods[1..3], ref, title: must be ≥ 30 in the 256-cube
 *   - grid:      may use 240+ (dim grey is fine for axes)
 *   - 16-23 / 232-239 forbidden (vanish on default-bg terminals)
 */
typedef struct {
  const char *name;
  short methods[4]; /* M_EULER, M_RK2, M_RK4, M_VERLET — dim → bright */
  short ref;        /* reference solution ('*') */
  short grid;       /* axes, panel separators, inactive method label */
  short title;      /* panel titles */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        methods[0..3]          ref   grid  title                  */
    {"Matrix", {28, 40, 82, 154}, 231, 240, 46},      /* greens       */
    {"Fire", {130, 166, 208, 226}, 231, 240, 202},    /* embers→bright */
    {"Oceanic", {24, 31, 45, 51}, 195, 240, 87},      /* deep teal→cyan */
    {"Neon", {93, 129, 165, 213}, 231, 240, 201},     /* purple → pink */
    {"Mono", {244, 248, 252, 255}, 255, 240, 252},    /* grayscale    */
    {"Ice", {75, 111, 117, 159}, 231, 240, 195},      /* polar blues  */
    {"Nova", {54, 92, 141, 213}, 231, 240, 177},      /* violet→pink  */
    {"Forest", {58, 100, 142, 184}, 231, 240, 190},   /* leaves       */
    {"Desert", {130, 172, 214, 230}, 231, 240, 220},  /* sand → gold  */
    {"Eclipse", {240, 124, 196, 220}, 231, 240, 208}, /* dark → corona */
};

/* Fixed HUD chrome — same on every theme so status / hint stay legible. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

static bool g_256;

/*
 * theme_apply — push the chosen palette into ncurses' colour-pair table.
 * Idempotent; safe to call from t / T handlers and at startup.  `idx`
 * is wrapped negative-safe so callers don't need to pre-modulo.  CP_HUD
 * and CP_HINT are reinstated every call so a future bug that overwrites
 * them can't outlive one keypress.
 */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[((idx % N_THEMES) + N_THEMES) % N_THEMES];
  if (g_256) {
    for (int i = 0; i < N_METHODS; i++)
      init_pair(CP_M0 + i, t->methods[i], -1);
    init_pair(CP_REF, t->ref, -1);
    init_pair(CP_GRID, t->grid, -1);
    init_pair(CP_TITLE, t->title, -1);
    init_pair(CP_HUD, CHROME_HUD_256, -1);
    init_pair(CP_HINT, CHROME_HINT_256, -1);
  } else {
    /* 8-colour fallback: theme-independent semantic palette. */
    init_pair(CP_M0, COLOR_RED, -1);
    init_pair(CP_M1, COLOR_YELLOW, -1);
    init_pair(CP_M2, COLOR_GREEN, -1);
    init_pair(CP_M3, COLOR_CYAN, -1);
    init_pair(CP_REF, COLOR_WHITE, -1);
    init_pair(CP_GRID, COLOR_BLUE, -1);
    init_pair(CP_TITLE, COLOR_MAGENTA, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void init_colors(int boot_theme) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  theme_apply(boot_theme);
}

/* ===================================================================== */
/* §4  ode  —  Hamiltonian system data model + the five integrator instances */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * State — one point in the phase space of a 1-DOF Hamiltonian system.
 *
 * Hamiltonian mechanics (Goldstein, *Classical Mechanics*, Ch. 8):
 *   The generalized coordinate q and conjugate momentum p span the
 *   phase plane.  Time evolution is governed by Hamilton's equations
 *
 *       dq/dt =  ∂H/∂p
 *       dp/dt = -∂H/∂q                                  (Eq. 8.18)
 *
 *   For our two test systems with mass m = 1 absorbed into the time
 *   scale:
 *       H_osc(q,p)  = ½ p² + ½ ω² q²                    quadratic
 *       H_pend(q,p) = ½ p² + (g/L)(1 - cos q)           nonlinear
 *
 *   In both cases dq/dt = p, so p numerically equals dq/dt — that's why
 *   the type can describe both "(displacement, velocity)" for the
 *   oscillator and "(angle, angular velocity)" for the pendulum.
 *
 * Why this matters for the demo:
 *   - Phase trajectories of a conservative Hamiltonian are LEVEL SETS
 *     of H — closed curves for bound motion (the oscillator is exact
 *     ellipses; the pendulum is libration-ellipses that distort with
 *     amplitude).  A SYMPLECTIC integrator's numerical orbit lies on
 *     a shadow Hamiltonian's exact level set — that's why Verlet's
 *     phase portrait is a closed loop after 10⁴ steps while RK4's
 *     slowly spirals.
 *   - Liouville's theorem: the flow preserves the phase-space area
 *     element dq ∧ dp.  Symplectic integrators preserve this exactly;
 *     non-symplectic ones don't, and the area drift IS the energy drift.
 *
 * Refs: Goldstein §8.1–8.4 (Hamilton's equations, phase space);
 *       Arnold, *Mathematical Methods of Classical Mechanics*, Ch. 8
 *       (symplectic structure, Liouville's theorem); Hairer, Lubich,
 *       Wanner, *Geometric Numerical Integration*, Ch. VI (the modern
 *       textbook on symplectic integrators).
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* q — generalized position in the system's natural coordinates:
   *
   *   SYS_OSCILLATOR  displacement from equilibrium (dimensionless,
   *                   normalised so the unit oscillator has q ∈ [−1, 1]
   *                   at the initial amplitude Q0_OSC = 1.0).
   *   SYS_PENDULUM    angle from the downward vertical (radians);
   *                   ode_init sets q₀ = Q0_PEND = 1.40 rad ≈ 80°
   *                   so the cos-series nonlinearity dominates and
   *                   the period stretches ~15 % over small-angle
   *                   prediction (Hairer-Wanner §I.14 worked example).
   *
   * Because mass m = 1 in our normalisation, q's time derivative is
   * recoverable directly from p (see below) — no mass division needed
   * in any integrator. */
  float q;

  /* p — generalized momentum conjugate to q.  Hamilton's equation
   *   dq/dt = ∂H/∂p
   * collapses to dq/dt = p for both our test systems since
   *   H = ½p² + V(q)
   * has p² as the entire kinetic part.  Concrete interpretation:
   *
   *   SYS_OSCILLATOR  p = dq/dt  (linear velocity)
   *   SYS_PENDULUM    p = dθ/dt  (angular velocity, rad/s)
   *
   * The integrator primitives in §5 use p both as "the momentum to
   * update next tick" AND as "the current slope of q" — that dual
   * role is what makes the leapfrog form of velocity Verlet work
   * (the OLD p drives the position update; the new p is averaged
   * across the two endpoints). */
  float p;
} State;

/* ── System catalogue ─────────────────────────────────────────────────── *
 *
 * Two test problems sit on opposite ends of the linear/nonlinear divide:
 *
 *   SYS_OSCILLATOR  Linear: H(q,p) = ½p² + ½ω²q².  Closed-form orbit is
 *                   a perfect ellipse; period T = 2π/ω is amplitude-
 *                   independent.  Hardest test case for Euler (orbit
 *                   spirals outward immediately — any |R(iωh)| > 1).
 *
 *   SYS_PENDULUM    Nonlinear: H(q,p) = ½p² + (g/L)(1 − cos q).
 *                   At small q the cos series cos q ≈ 1 − q²/2 reduces
 *                   it to the oscillator; at large q (e.g. Q0_PEND = 1.4
 *                   rad ≈ 80°) the period stretches by ~15 % over the
 *                   small-angle prediction.  RK4 tracks the true period
 *                   easily; Verlet is symplectic so it tracks the
 *                   long-term energy band even if its phase drifts.
 * ─────────────────────────────────────────────────────────────────────── */
#define SYS_OSCILLATOR 0
#define SYS_PENDULUM 1

/* SYS_NAME — full equation form, kept in source comments for reference
 * (and so future code can surface it if a "show equation" key is added).
 * The HUD uses SYS_SHORT_NAME below to keep the top bar width-bounded.
 *
 *     Harmonic Oscillator   x'' = -w^2 x
 *     Nonlinear Pendulum    t'' = -(g/L) sin t                          */

/* ── Integrator catalogue ────────────────────────────────────────────── *
 *
 * Four candidate integrators run against a high-accuracy reference.
 * Indices match the 1..4 toggle keys.  ORDER lists the GLOBAL error
 * order (LTE − 1) — see file-header THEORY for the derivation:
 *
 *   M_EULER   Forward Euler          — O(h)    non-symplectic, unstable
 *   M_RK2     Explicit Midpoint      — O(h²)   non-symplectic
 *   M_RK4     Classical Runge-Kutta  — O(h⁴)   non-symplectic
 *   M_VERLET  Velocity Verlet        — O(h²)   SYMPLECTIC
 *
 * Verlet has the same global order as RK2 yet behaves qualitatively
 * better — the entire point of the demo is to show that ORDER does not
 * equal STRUCTURE-PRESERVATION.
 * ─────────────────────────────────────────────────────────────────────── */
#define M_EULER 0
#define M_RK2 1
#define M_RK4 2
#define M_VERLET 3

static const char *METHOD_NAME[N_METHODS] = {"Euler", "RK2", "RK4", "Verlet"};
static const int METHOD_CP[N_METHODS] = {CP_M0, CP_M1, CP_M2, CP_M3};
static const char *METHOD_ORDER[N_METHODS] = {"O(h)", "O(h^2)", "O(h^4)",
                                              "O(h^2) symplectic"};

/* Short system names used in the top HUD bar — full SYS_NAME entries
 * have the equation appended (~40 chars) which is too wide. */
static const char *SYS_SHORT_NAME[2] = {"Oscillator", "Pendulum"};

/* ── Initial condition + force law + Hamiltonian energy ───────────────── *
 *
 *   initial_condition(sys)      → State at t = 0
 *   acceleration_at(q, sys)     → d²q/dt² as a function of q
 *   state_derivative(s, sys)    → (dq/dt, dp/dt) = (p, accel)
 *   hamiltonian_energy(s, sys)  → H(q, p)
 *
 * These four are the ONLY system-dependent primitives.  Every
 * integrator in §5 calls state_derivative or acceleration_at and
 * acceleration_at; ode_init seeds the simulation with initial_condition
 * and stores hamiltonian_energy(s, sys) as e0 for drift analysis (§6).
 * Adding a new system means writing a new case in each of these four —
 * the integrators stay untouched.
 * ─────────────────────────────────────────────────────────────────────── */

/* initial_condition — release the system from rest at its starting q. */
static State initial_condition(int sys) {
  if (sys == SYS_OSCILLATOR)
    return (State){Q0_OSC, 0.0f};
  /* SYS_PENDULUM */ return (State){Q0_PEND, 0.0f};
}

/* acceleration_at — d²q/dt² for the given system.  Used by Verlet
 * (which only needs the force law, not the full state derivative). */
static float acceleration_at(float q, int sys) {
  if (sys == SYS_OSCILLATOR)
    return -(OMEGA * OMEGA * q);                   /* SHM    */
  /* SYS_PENDULUM */ return -(G_OVER_L * sinf(q)); /* nonlin */
}

/* state_derivative — the right-hand side of Hamilton's equations:
 *   d(q,p)/dt = (p, acceleration_at(q))
 * Used by Euler / RK2 / RK4 which need the full vector field. */
static State state_derivative(State s, int sys) {
  return (State){s.p, acceleration_at(s.q, sys)};
}

/*
 * hamiltonian_energy — H(q, p) for the active system.
 *
 *   Oscillator: H = ½p² + ½ω²q²                  (quadratic potential)
 *   Pendulum  : H = ½p² + (g/L)(1 − cos q)        (exact nonlinear)
 *
 * Used to track energy drift per method (§6).  Expected signs of the
 * fractional drift (E(t) − E₀)/E₀ at moderate dt:
 *   Euler  → positive drift (gains energy)
 *   RK2    → negative drift (numerical dissipation)
 *   RK4    → tiny drift, sign depends on dt
 *   Verlet → oscillates near 0 % — no secular trend (symplectic)
 */
static float hamiltonian_energy(State s, int sys) {
  float kinetic = 0.5f * s.p * s.p;
  if (sys == SYS_OSCILLATOR)
    return kinetic + 0.5f * OMEGA * OMEGA * s.q * s.q;
  /* SYS_PENDULUM */
  return kinetic + G_OVER_L * (1.0f - cosf(s.q));
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Method — per-integrator workspace.
 *
 * One instance per candidate integrator (Euler, RK2, RK4, Verlet).  All
 * four share the same initial condition (set by ode_init) but evolve
 * independently from there.  At any time t the four Methods' states
 * diverge in characteristic ways the demo is built to show:
 *
 *   M_EULER   spirals OUTWARD          non-symplectic, energy injection
 *   M_RK2     spirals INWARD           non-symplectic, dissipation
 *   M_RK4     nearly closed orbit      high accuracy, NOT symplectic
 *   M_VERLET  exactly closed orbit     SYMPLECTIC; E oscillates near E₀
 *
 * Storage: ~12 KB per Method (the two ring buffers dominate).  All BSS,
 * no allocation.  Per-field documentation lives in the typedef body
 * below; this preamble covers only the high-level intent.
 *
 * Ring buffer contract (used by phase_history_push / time_history_push
 * in §5):
 *   - head always points at the NEXT write slot
 *   - count saturates at capacity (stops growing when full)
 *   - readers walk forward from (head − count + cap) % cap for count
 *     entries to get chronological order
 *   - O(1) push, no malloc, oldest entries are silently overwritten
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── State + history ──────────────────────────────────────────────
   * The integrator-output state plus two parallel ring buffers for
   * the two visualisation panels.  Both rings follow the SAME write-
   * head + saturating-count contract described above; only the
   * element type differs (State for phase, float for time-series).
   */

  /* s — current (q, p) of THIS integrator.  Written every tick by
   * dispatch_integrator (§5).  Read by:
   *   §6 compute_method_errors    (err_cur = |s.q − ref.q|)
   *   §6 energy_drift_pct         (H(s, system) vs e0 baseline)
   *   §7 draw_method_phase_head   (bold method-id glyph in panel)
   *   §8 render_overlay           (the per-method status line)
   * Each tick the four Method.s values diverge from each other and
   * from o->ref — that divergence is the entire demo. */
  State s;

  /* traj[] — TRAJ_LEN-entry ring of phase-portrait history (q, p).
   * Element traj[i] is NOT the i-th oldest: walk the ring in
   * chronological order via
   *   start = (traj_head − traj_count + TRAJ_LEN) % TRAJ_LEN
   *   for k in 0..traj_count-1 : traj[(start + k) % TRAJ_LEN]
   * TRAJ_LEN = 1024 sized to comfortably hold one full oscillator
   * period at STEPS_PER_FRAME × dt × TARGET_FPS — so the closed
   * loop of Verlet stays visible without truncation. */
  State traj[TRAJ_LEN];
  int traj_head;  /* next-write slot, ≡ traj_count + start mod TRAJ_LEN */
  int traj_count; /* valid entries, saturates at TRAJ_LEN               */

  /* tser[] — TIME_LEN-entry ring of q-only samples for the scrolling
   * time-series panel.  Same ring discipline.  Capacity (=512) sized
   * to match or exceed the terminal width of typical wide displays;
   * narrower terminals just render the last `cols` samples. */
  float tser[TIME_LEN];
  int tser_head;
  int tser_count;

  /* ── Error accumulators  (refreshed every tick by §6) ─────────────
   * Position-only error tracking.  We deliberately do NOT track
   * |p − p_ref| separately: for bound motion |p| ≤ √(2H) is
   * automatically bounded by the energy_drift readout below, and
   * adding a second error number would be visual noise.
   */

  /* err_cur — |q − q_ref| at the current tick.  Instantaneous
   * absolute error in position.  Updated every tick by
   * compute_method_errors.  Used both for err_max accumulation and
   * for the per-method HUD line in §8. */
  float err_cur;

  /* err_max — monotonic running max of err_cur since the last reset.
   * NEVER decreases except via r/R reset (which calls ode_init,
   * which zeroes the whole struct).  Useful for "what was the worst
   * divergence so far?" rather than instantaneous noise. */
  float err_max;

  /* e0 — Hamiltonian energy H(s, system) captured at t = 0 and
   * stored once by ode_init.  Baseline for the relative drift
   * readout in §6:
   *   energy_drift_pct = (H(s_now) − e0) / e0 × 100%
   * Storing the baseline (rather than recomputing from initial
   * conditions every frame) lets the drift readout survive even if
   * IC formulas change at runtime in a future feature.  Near-zero
   * e0 (degenerate q₀ = p₀ = 0 case) is guarded by an explicit zero
   * return in energy_drift_pct. */
  float e0;

  /* ── Render gate ──────────────────────────────────────────────────
   * Per-method visibility flag, NOT a pause.  Important to keep
   * physics evolution running while hidden: when the user re-shows
   * a method with the same numbered key, its state must be the same
   * one it would have had had it stayed visible — otherwise toggling
   * would distort the comparison.
   */

  /* active — toggled by keys 1..4 (§9 scene_key).  Affects RENDER
   * ONLY: §7 phase / time draw helpers `return` immediately when
   * !active, and the §8 status line shows a "(hidden)" placeholder.
   * The integrator itself still advances every tick.  scene_key
   * also clears the ring buffers on toggle so the dots vanish
   * immediately instead of leaving a stale tail. */
  bool active;
} Method;

/* ─────────────────────────────────────────────────────────────────────── *
 * ODE — top-level simulation state shared across all integrators.
 *
 * Five integrators run in parallel from the same initial condition:
 *   m[0..3]   the four candidate methods being compared
 *   ref       the reference solution (RK4 with REF_SUBSTEPS=32 micro-
 *             steps per display tick → global error O((h/32)⁴) ≈ machine
 *             precision; the ground truth all four candidates are
 *             measured against)
 *
 * Why ref is a State on ODE rather than a fifth Method:
 *   The reference is conceptually different — a benchmark, not a
 *   candidate.  Keeping it off the Method[] array means iteration over
 *   "the candidates being compared" never accidentally includes the
 *   reference (which would defeat the comparison).  Its ring buffers
 *   live on ODE alongside its State for the same locality reason.
 *
 * Per-field details (shared simulation knobs, pre-computed display
 * ranges, locality contract for q_range / p_range) are documented in
 * the typedef body below.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Candidate integrators  (the four being compared) ───────────── */

  /* m[] — one Method per candidate integrator, indexed by M_EULER..
   * M_VERLET so dispatch_integrator (§5) can switch on the loop index
   * directly.  All four start from identical initial conditions in
   * ode_init and evolve INDEPENDENTLY thereafter — that divergence
   * is the data the demo visualises.  The array is fixed-size so
   * the entire ODE struct lives in BSS / stack with no allocation. */
  Method m[N_METHODS];

  /* ── Reference solution  (the ground truth they're judged against) */

  /* ref — current (q, p) of the high-accuracy reference integrator,
   * advanced each tick by reference_solution_step (= classical_rk4
   * with REF_SUBSTEPS=32 micro-steps).  Effective global error
   * ≈ O((h/32)⁴), well below single-precision noise for our (h, T)
   * regime — close enough to call "exact".
   *
   * Why ref is a State on ODE rather than a fifth Method:
   *   1. It's a BENCHMARK, not a candidate.  Loops over m[] never
   *      include it — that would defeat the comparison.
   *   2. It has different rendering (white '*' / 'R' glyphs vs the
   *      coloured '.' / digit glyphs the methods use).
   *   3. It has no err / e0 / active fields — those concepts don't
   *      apply to the ground truth.
   */
  State ref;

  /* ref_traj[] / ref_tser[] — reference history rings, mirroring
   * the ones on Method.  Live on ODE rather than a fifth Method for
   * the locality reason above.  Both follow the same ring contract
   * (write head + saturating count) as the per-method buffers. */
  State ref_traj[TRAJ_LEN];
  int ref_traj_head;
  int ref_traj_count;
  float ref_tser[TIME_LEN];
  int ref_tser_head;
  int ref_tser_count;

  /* ── Shared simulation knobs  (apply uniformly to ref + every method) */

  /* dt — current step size h (s).  Mutable: + / − keys nudge by
   * DT_STEP within [DT_MIN, DT_MAX].  Applies UNIFORMLY to ref and
   * every candidate so the comparison stays apples-to-apples even
   * as the user dials dt up to provoke Euler-blow-up.  At ωh = 0.35
   * Euler grows by 6 %/step — the entire demo's "danger zone" is
   * just this number being tuned past method stability limits. */
  float dt;

  /* t — accumulated simulation time (s) since last reset.  Sum of
   * dt over all ticks.  Surface in the HUD; not read by any
   * integrator (each integrator only needs dt and current state). */
  float t;

  /* system — SYS_OSCILLATOR (=0) or SYS_PENDULUM (=1).  Toggled by
   * the 's' key, which triggers a full ode_init() so initial
   * conditions, e0 baselines, and the display ranges below all get
   * re-derived for the new system.  All §5 primitives branch on
   * this through state_derivative / acceleration_at. */
  int system;

  /* paused — when true, scene_tick (§9) skips the entire
   * ode_advance_one_tick pipeline.  Every Method.s and ref stay
   * frozen — unpause continues from the EXACT same state, no
   * discontinuity, no "catch-up" jump.  Mirrored to the HUD via
   * the "PAUSED " badge in §8. */
  bool paused;

  /* step_count — monotonically increasing tick counter.  Reset by
   * ode_init.  Useful for the user to gauge "how many time-steps
   * have accumulated when comparing divergence rates" — visible in
   * the HUD via §8. */
  int step_count;

  /* ── Display window  (rendering only, derived from physics) ────── *
   *
   * q_range / p_range are PHYSICS-INDEPENDENT rendering parameters
   * cached on ODE for performance.  They live here (not on Scene)
   * because they depend on the SYSTEM choice:
   *
   *   - oscillator needs ~3× the initial amplitude as range (so
   *     Euler's growing spiral has room to be visible)
   *   - pendulum is bounded by ±Q0_PEND in q (energy conservation),
   *     so 1.6× suffices and gives a tighter view
   *
   * Computed once per system in ode_init (sqrt + cos for pendulum
   * p_range) and then read every frame by §7 — caching avoids
   * recomputing the transcendentals in the render hot loop.
   *
   * Locality contract:  mutating q_range / p_range does NOT change
   * what any integrator produces.  They satisfy the "rendering
   * parameter" test from Scene's locality contract — they just
   * happen to live on ODE rather than Scene for the dependency-on-
   * system reason above.
   */

  /* q_range — half-range of the q axis used by BOTH the phase panel
   * (horizontal axis) AND the time panel (vertical axis).  The
   * viewport spans [−q_range, +q_range].  Values outside that range
   * (e.g. Euler's spiraling orbit past the initial amplitude)
   * clamp to the panel edge via physics_value_to_screen_*. */
  float q_range;

  /* p_range — half-range of the p axis used ONLY by the phase panel.
   * Computed from system-specific max-velocity estimate:
   *   oscillator:  Q0_OSC · ω · PHASE_SCALE_OSC          (linear)
   *   pendulum :   √(2·g/L·(1−cos Q0_PEND)) · scale      (energy cons.)
   * The pendulum form is the maximum speed at the swing's bottom,
   * derived from H = E₀ ⇒ ½p_max² = (g/L)(1 − cos Q0). */
  float p_range;
} ODE;

/* ===================================================================== */
/* §5  integrators + one-tick evolution pipeline                          */
/*                                                                       */
/* ode_advance_one_tick() is the per-frame heartbeat:                     */
/*                                                                       */
/*     advance_reference_solution(o)   // RK4 × REF_SUBSTEPS micro-steps  */
/*     advance_candidate_methods(o)    // 4 dispatched integrator steps   */
/*     accumulate_simulation_clock(o)  // t += dt ; step_count++          */
/*                                                                       */
/* Each candidate is implemented by a single integrator primitive named  */
/* after its standard mathematical identity:                              */
/*                                                                       */
/*     forward_euler_step       O(h)      explicit Euler                 */
/*     explicit_midpoint_step   O(h²)     RK2 midpoint rule              */
/*     classical_rk4_step       O(h⁴)     Kutta 1901 four-stage RK       */
/*     velocity_verlet_step     O(h²)     SYMPLECTIC leapfrog            */
/*                                                                       */
/* The reference solution is just classical_rk4_step run REF_SUBSTEPS×    */
/* (=32) micro-steps per display tick, giving effective machine-precision */
/* truth.  See the file-header THEORY section for the derivation of each */
/* method's stability bound and accuracy order.                           */
/* ===================================================================== */

/*
 * forward_euler_step — explicit (forward) Euler.  One state_derivative
 * evaluation, advance by h:
 *
 *     y_{n+1} = y_n + h · f(y_n)
 *
 * First-order accurate: the Taylor expansion keeps only the O(h) term,
 * so local truncation error is O(h²) and global error is O(h).  For
 * any oscillator the amplification factor satisfies
 *     |R(iωh)| = √(1 + ω²h²) > 1   for all h > 0
 * → the orbit ALWAYS spirals outward.  Euler is therefore UNCONDITIONALLY
 * UNSTABLE on harmonic oscillators (Hairer-Wanner I §I.7).
 */
static State forward_euler_step(State s, float dt, int sys) {
  State d = state_derivative(s, sys);
  return (State){s.q + dt * d.q, s.p + dt * d.p};
}

/*
 * explicit_midpoint_step — RK2 midpoint rule.  Two state_derivative
 * evaluations:
 *
 *     k₁ = f(y_n)                    — slope at the interval start
 *     k₂ = f(y_n + ½h · k₁)          — slope at the estimated midpoint
 *     y_{n+1} = y_n + h · k₂
 *
 * Using the midpoint slope k₂ instead of the endpoint slope k₁ cancels
 * the leading error term, lifting global error to O(h²).  The orbit
 * drifts INWARD because |R(iωh)| = √(1 − ω⁴h⁴/4) < 1 for ωh < √2 —
 * symptom of non-symplectic energy DISSIPATION.
 */
static State explicit_midpoint_step(State s, float dt, int sys) {
  State k1 = state_derivative(s, sys);
  State mid = {s.q + 0.5f * dt * k1.q, s.p + 0.5f * dt * k1.p};
  State k2 = state_derivative(mid, sys);
  return (State){s.q + dt * k2.q, s.p + dt * k2.p};
}

/*
 * classical_rk4_step — Kutta's 1901 four-stage Runge-Kutta.  Four
 * state_derivative evaluations, weighted Simpson-style:
 *
 *     k₁ = f(y_n)
 *     k₂ = f(y_n + ½h · k₁)
 *     k₃ = f(y_n + ½h · k₂)
 *     k₄ = f(y_n +  h · k₃)
 *     y_{n+1} = y_n + h · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * The weighted average (with midpoint stages double-counted) cancels
 * error terms through O(h⁴), giving global error O(h⁴) — halving h
 * shrinks the error by 16× (Hairer-Norsett-Wanner I §II.1).  Still NOT
 * symplectic; energy drift is tiny but secular over long integrations.
 */
static State classical_rk4_step(State s, float dt, int sys) {
  State k1 = state_derivative(s, sys);
  State s2 = {s.q + 0.5f * dt * k1.q, s.p + 0.5f * dt * k1.p};
  State k2 = state_derivative(s2, sys);
  State s3 = {s.q + 0.5f * dt * k2.q, s.p + 0.5f * dt * k2.p};
  State k3 = state_derivative(s3, sys);
  State s4 = {s.q + dt * k3.q, s.p + dt * k3.p};
  State k4 = state_derivative(s4, sys);
  return (State){s.q + dt * (k1.q + 2.0f * k2.q + 2.0f * k3.q + k4.q) / 6.0f,
                 s.p + dt * (k1.p + 2.0f * k2.p + 2.0f * k3.p + k4.p) / 6.0f};
}

/*
 * velocity_verlet_step — symplectic leapfrog.  Requires only the force
 * law (acceleration_at), not the full state_derivative.
 *
 *     a_n     = acceleration_at(q_n)                  — old force
 *     q_{n+1} = q_n + p_n·h + ½ a_n h²                — drift  (uses OLD p)
 *     a_{n+1} = acceleration_at(q_{n+1})              — new force
 *     p_{n+1} = p_n + ½ (a_n + a_{n+1}) · h           — kick   (trapezoid)
 *
 * Symplectic because the position update uses p_n (the OLD momentum):
 * the resulting Jacobian has unit determinant, so dq ∧ dp is preserved
 * exactly each step.  Consequence: energy E(t) oscillates O(h²) around
 * E₀ FOREVER — no secular drift even over millions of steps.  Global
 * position error is the same O(h²) as RK2 but the orbit STRUCTURE is
 * exact-shadow-Hamiltonian, which is what makes Verlet the right choice
 * for solar-system / molecular-dynamics work.
 *
 * Refs: Hairer / Lubich / Wanner [HLW], *Geometric Numerical Integration*,
 *       Ch. VI §VI.3 (the velocity Verlet form); Verlet (1967),
 *       "Computer Experiments on Classical Fluids".
 */
static State velocity_verlet_step(State s, float dt, int sys) {
  float a_old = acceleration_at(s.q, sys);
  float q_new = s.q + s.p * dt + 0.5f * a_old * dt * dt;
  float a_new = acceleration_at(q_new, sys);
  float p_new = s.p + 0.5f * (a_old + a_new) * dt;
  return (State){q_new, p_new};
}

/*
 * reference_solution_step — the "ground truth" integrator.  Just
 * classical_rk4_step run REF_SUBSTEPS=32 micro-steps of size h/32 per
 * display tick.  Total global error ≈ O((h/32)⁴) ≈ machine precision
 * for our (h, T) regime — close enough to call it exact.  Every
 * candidate method is compared against this reference in §6.
 */
static State reference_solution_step(State s, float dt, int sys) {
  float micro_h = dt / (float)REF_SUBSTEPS;
  for (int i = 0; i < REF_SUBSTEPS; i++)
    s = classical_rk4_step(s, micro_h, sys);
  return s;
}

/*
 * dispatch_integrator — single point that maps a method index to its
 * implementing primitive.  Keeps the per-method ID → function mapping
 * in ONE table-driven place so adding a new integrator means writing
 * the primitive + adding one case here (plus an enum/name entry).
 */
static State dispatch_integrator(int method, State s, float dt, int sys) {
  switch (method) {
  case M_EULER:
    return forward_euler_step(s, dt, sys);
  case M_RK2:
    return explicit_midpoint_step(s, dt, sys);
  case M_RK4:
    return classical_rk4_step(s, dt, sys);
  case M_VERLET:
    return velocity_verlet_step(s, dt, sys);
  default:
    return s;
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Ring-buffer history primitives  (used by both candidate methods and
 * the reference).  Both follow the "write head + saturating count"
 * pattern documented on the Method struct above.
 * ─────────────────────────────────────────────────────────────────────── */

/* phase_history_push — append one (q, p) sample to a TRAJ_LEN ring.
 * Used for the phase-portrait panel.  O(1) — one write, one mod-incr. */
static void phase_history_push(State *ring, int *head, int *count, State s) {
  ring[*head] = s;
  *head = (*head + 1) % TRAJ_LEN;
  if (*count < TRAJ_LEN)
    (*count)++;
}

/* time_history_push — same ring pattern, but for a scalar (q only).
 * Used for the scrolling time-series panel. */
static void time_history_push(float *ring, int *head, int *count, float v) {
  ring[*head] = v;
  *head = (*head + 1) % TIME_LEN;
  if (*count < TIME_LEN)
    (*count)++;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * One-tick evolution pipeline.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * advance_reference_solution — step the ground-truth integrator by dt
 * and push its new (q, p) and q onto the reference's ring buffers.
 * Runs FIRST in the tick so the candidate methods can be compared
 * against the freshest reference value (§6).
 */
static void advance_reference_solution(ODE *o) {
  o->ref = reference_solution_step(o->ref, o->dt, o->system);
  phase_history_push(o->ref_traj, &o->ref_traj_head, &o->ref_traj_count,
                     o->ref);
  time_history_push(o->ref_tser, &o->ref_tser_head, &o->ref_tser_count,
                    o->ref.q);
}

/*
 * advance_candidate_methods — step every Method by dt and push its
 * new state onto its phase / time ring buffers.  Order: M_EULER..
 * M_VERLET, deterministic so reproducible debug runs are possible.
 */
static void advance_candidate_methods(ODE *o) {
  for (int mi = 0; mi < N_METHODS; mi++) {
    Method *m = &o->m[mi];
    m->s = dispatch_integrator(mi, m->s, o->dt, o->system);
    phase_history_push(m->traj, &m->traj_head, &m->traj_count, m->s);
    time_history_push(m->tser, &m->tser_head, &m->tser_count, m->s.q);
  }
}

/* accumulate_simulation_clock — bump the simulation time and tick count.
 * Kept as its own helper so the pipeline reads as three named stages. */
static void accumulate_simulation_clock(ODE *o) {
  o->t += o->dt;
  o->step_count += 1;
}

/*
 * ode_advance_one_tick — full per-tick pipeline.  Reads as the
 * pseudocode pinned to the top of §5:
 *
 *     advance_reference_solution(o);
 *     advance_candidate_methods(o);
 *     accumulate_simulation_clock(o);
 *
 * Called STEPS_PER_FRAME times per rendered frame from scene_tick (§9).
 */
static void ode_advance_one_tick(ODE *o) {
  advance_reference_solution(o);
  advance_candidate_methods(o);
  accumulate_simulation_clock(o);
}

/* ===================================================================== */
/* §6  error analysis  (position error vs reference + energy drift)       */
/* ===================================================================== */

/*
 * compute_method_errors — refresh err_cur / err_max for every method
 * using the reference's CURRENT q as ground truth.
 *
 *     err_cur = |q_method − q_ref|       (this tick only)
 *     err_max = max(err_max, err_cur)    (monotone, reset by ode_init)
 *
 * Only the q error is tracked — momentum errors are dominated by the q
 * errors here (both systems are bound, so |p| ≤ √(2H) is small) and
 * adding a p-error would just be visual noise.
 */
static void compute_method_errors(ODE *o) {
  for (int mi = 0; mi < N_METHODS; mi++) {
    Method *m = &o->m[mi];
    m->err_cur = fabsf(m->s.q - o->ref.q);
    if (m->err_cur > m->err_max)
      m->err_max = m->err_cur;
  }
}

/*
 * energy_drift_pct — fractional change of the Hamiltonian since t = 0,
 * expressed as a percentage:
 *
 *     drift(t) = (H(s_method(t), sys) − H₀) / H₀ × 100%
 *
 * where H₀ = m->e0 was stored once at reset (so the percentage is
 * always relative to the initial energy, not the previous tick).  The
 * sign carries information: Euler always >0 (gains energy), RK2 always
 * <0 (dissipates), Verlet oscillates near 0 forever (symplectic).
 *
 * Guard against division by near-zero H₀ (would happen at q₀ = p₀ = 0,
 * a degenerate equilibrium).
 */
static float energy_drift_pct(const Method *m, int sys) {
  float e_cur = hamiltonian_energy(m->s, sys);
  if (fabsf(m->e0) < 1e-9f)
    return 0.0f;
  return (e_cur - m->e0) / m->e0 * 100.0f;
}

/* ===================================================================== */
/* §7  plot panels — phase portrait + scrolling time series              */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Physics-to-screen coordinate mappings.
 *
 * Both panels need to map a physical scalar (q ∈ [−qr, +qr] or p ∈
 * [−pr, +pr]) onto a 1-D screen interval.  We split this into two
 * helpers so the intent of each callsite is clear:
 *
 *   physics_value_to_screen_row(v, range, base, span)
 *     y-style mapping where +range → TOP of panel, −range → BOTTOM
 *     (standard math convention; screen-rows happen to grow downward
 *     so the formula flips the sign internally).
 *
 *   physics_q_to_screen_col(q, qr, base, span)
 *     x-style mapping where −qr → LEFT of panel, +qr → RIGHT.
 *     No sign flip — screen columns and the q-axis both grow rightward.
 *
 * Both clamp the output to [base, base+span−1] so out-of-range values
 * (e.g. Euler's spiraling orbit beyond ±qr) get pinned to the panel
 * edge instead of corrupting neighbouring panels.
 * ─────────────────────────────────────────────────────────────────────── */

static int physics_value_to_screen_row(float v, float range, int base,
                                       int span) {
  float t = (range - v) / (2.0f * range); /* 0=top/+range, 1=bot/-range */
  int r = base + (int)(t * (float)(span - 1) + 0.5f);
  if (r < base)
    r = base;
  if (r >= base + span)
    r = base + span - 1;
  return r;
}

static int physics_q_to_screen_col(float q, float qr, int base, int span) {
  float t = (q + qr) / (2.0f * qr); /* 0=left/−qr, 1=right/+qr */
  int c = base + (int)(t * (float)(span - 1) + 0.5f);
  if (c < base)
    c = base;
  if (c >= base + span)
    c = base + span - 1;
  return c;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Phase-portrait renderer  (q vs p).
 *
 * render_phase_portrait() is the orchestrator.  It dispatches:
 *
 *     draw_phase_grid              q=0 / p=0 axes
 *     for each active method:
 *         draw_method_phase_trail  history dots
 *         draw_method_phase_head   current-position glyph (bold)
 *     draw_reference_phase_trail   ground truth on top
 *     draw_reference_phase_head    bold 'R' for ref's current point
 *     draw_panel_title             "Phase Portrait q vs p"
 *
 * Z-order rationale (ncurses writes overwrite):
 *   - Methods drawn in index order so later methods cover earlier ones
 *     where they overlap.
 *   - Reference drawn LAST so its '*' dots always win — the ground
 *     truth must be readable even when a method's trail crosses it.
 *   - Title is painted last so it survives every preceding draw call.
 * ─────────────────────────────────────────────────────────────────────── */

/* Plot a single dot at the screen position corresponding to (q, p) if
 * the cell lies inside the panel.  Shared by every trail/head helper. */
static void plot_phase_dot(float q, float p, float qr, float pr, int px, int py,
                           int pw, int ph, char glyph) {
  int sx = physics_q_to_screen_col(q, qr, px, pw);
  int sy = physics_value_to_screen_row(p, pr, py, ph);
  if (sx >= px && sx < px + pw && sy >= py && sy < py + ph)
    mvaddch(sy, sx, (chtype)glyph);
}

/* Draw the q=0 / p=0 axes for the phase portrait. */
static void draw_phase_grid(float qr, float pr, int px, int py, int pw,
                            int ph) {
  int ax = physics_q_to_screen_col(0.0f, qr, px, pw);     /* col q=0 */
  int ay = physics_value_to_screen_row(0.0f, pr, py, ph); /* row p=0 */
  attrset((chtype)COLOR_PAIR(CP_GRID) | A_BOLD);
  for (int y = py; y < py + ph; y++)
    mvaddch(y, ax, '|');
  for (int x = px; x < px + pw; x++)
    mvaddch(ay, x, '-');
  mvaddch(ay, ax, '+');
}

/* Walk one method's phase-history ring (oldest → newest) and plot
 * a '.' at each (q, p).  No-op when the method is hidden. */
static void draw_method_phase_trail(const Method *m, int mi, float qr, float pr,
                                    int px, int py, int pw, int ph) {
  if (!m->active)
    return;
  attrset((chtype)COLOR_PAIR(METHOD_CP[mi]));
  int cnt = m->traj_count;
  int start = (m->traj_head - cnt + TRAJ_LEN) % TRAJ_LEN;
  for (int i = 0; i < cnt; i++) {
    const State *s = &m->traj[(start + i) % TRAJ_LEN];
    plot_phase_dot(s->q, s->p, qr, pr, px, py, pw, ph, '.');
  }
}

/* Bold glyph at the method's CURRENT (q, p) — '1' for Euler, '2' for RK2,
 * etc.  Sits on top of its own trail so the reader can see where each
 * integrator is right now without scanning the latest trail dot. */
static void draw_method_phase_head(const Method *m, int mi, float qr, float pr,
                                   int px, int py, int pw, int ph) {
  if (!m->active)
    return;
  attrset((chtype)COLOR_PAIR(METHOD_CP[mi]) | A_BOLD);
  plot_phase_dot(m->s.q, m->s.p, qr, pr, px, py, pw, ph, '1' + mi);
}

/* Reference solution trail — '*' dots; drawn after every method so it
 * always reads as the topmost layer. */
static void draw_reference_phase_trail(const ODE *o, int px, int py, int pw,
                                       int ph) {
  attrset((chtype)COLOR_PAIR(CP_REF));
  int cnt = o->ref_traj_count;
  int start = (o->ref_traj_head - cnt + TRAJ_LEN) % TRAJ_LEN;
  for (int i = 0; i < cnt; i++) {
    const State *s = &o->ref_traj[(start + i) % TRAJ_LEN];
    plot_phase_dot(s->q, s->p, o->q_range, o->p_range, px, py, pw, ph, '*');
  }
}

/* Bold 'R' at the reference's current (q, p). */
static void draw_reference_phase_head(const ODE *o, int px, int py, int pw,
                                      int ph) {
  attrset((chtype)COLOR_PAIR(CP_REF) | A_BOLD);
  plot_phase_dot(o->ref.q, o->ref.p, o->q_range, o->p_range, px, py, pw, ph,
                 'R');
}

/* Panel title; one helper shared by both phase + time panels.  Always
 * painted last so it survives all earlier draws at the same cells. */
static void draw_panel_title(int py, int px, const char *title) {
  attrset((chtype)COLOR_PAIR(CP_TITLE) | A_BOLD);
  mvaddstr(py, px + 2, title);
}

/* render_phase_portrait — top-level orchestrator (reads as pseudocode). */
static void render_phase_portrait(const ODE *o, int px, int py, int pw,
                                  int ph) {
  float qr = o->q_range;
  float pr = o->p_range;
  attrset(A_NORMAL);

  draw_phase_grid(qr, pr, px, py, pw, ph);

  for (int mi = 0; mi < N_METHODS; mi++) {
    draw_method_phase_trail(&o->m[mi], mi, qr, pr, px, py, pw, ph);
    draw_method_phase_head(&o->m[mi], mi, qr, pr, px, py, pw, ph);
  }

  draw_reference_phase_trail(o, px, py, pw, ph);
  draw_reference_phase_head(o, px, py, pw, ph);

  draw_panel_title(py, px, "Phase Portrait  q vs p");
  attrset(A_NORMAL);
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Time-series renderer  (q vs t, scrolling left).
 *
 * Same orchestration pattern as the phase portrait:
 *
 *     draw_time_zero_line          horizontal q=0 reference line
 *     for each active method:
 *         draw_method_time_trace   each tick = one column
 *     draw_reference_time_trace    ground truth '*' on top
 *     draw_panel_title             "Time Series q(t)"
 *
 * Scrolling: the rightmost column shows the newest sample (head-1),
 * each leftward column is one tick older.  show = min(count, pw) so
 * partially-filled rings still align to the right edge.
 * ─────────────────────────────────────────────────────────────────────── */

/* Horizontal q = 0 line across the time panel. */
static void draw_time_zero_line(float qr, int px, int py, int pw, int ph) {
  int zy = physics_value_to_screen_row(0.0f, qr, py, ph);
  attrset((chtype)COLOR_PAIR(CP_GRID) | A_BOLD);
  for (int x = px; x < px + pw; x++)
    mvaddch(zy, x, '-');
}

/* Walk a scalar ring (m->tser or o->ref_tser) newest-LAST so the right
 * edge of the panel always shows the most recent sample.  Shared by
 * method + reference traces (different glyph + color). */
static void plot_time_ring(const float *ring, int head, int count, float qr,
                           int px, int py, int pw, int ph, char glyph) {
  int show = (count < pw) ? count : pw;
  for (int sx = 0; sx < show; sx++) {
    int back = show - 1 - sx;
    int idx = ((head - 1 - back) % TIME_LEN + TIME_LEN) % TIME_LEN;
    float v = ring[idx];
    int sy = physics_value_to_screen_row(v, qr, py, ph);
    mvaddch(sy, px + (pw - show) + sx, (chtype)glyph);
  }
}

/* One method's time trace ('.' in its theme color). */
static void draw_method_time_trace(const Method *m, int mi, float qr, int px,
                                   int py, int pw, int ph) {
  if (!m->active)
    return;
  attrset((chtype)COLOR_PAIR(METHOD_CP[mi]));
  plot_time_ring(m->tser, m->tser_head, m->tser_count, qr, px, py, pw, ph, '.');
}

/* Reference time trace ('*'); drawn last so it always wins overlaps. */
static void draw_reference_time_trace(const ODE *o, int px, int py, int pw,
                                      int ph) {
  attrset((chtype)COLOR_PAIR(CP_REF));
  plot_time_ring(o->ref_tser, o->ref_tser_head, o->ref_tser_count, o->q_range,
                 px, py, pw, ph, '*');
}

/* render_time_series — top-level orchestrator. */
static void render_time_series(const ODE *o, int px, int py, int pw, int ph) {
  float qr = o->q_range;
  attrset(A_NORMAL);

  draw_time_zero_line(qr, px, py, pw, ph);

  for (int mi = 0; mi < N_METHODS; mi++)
    draw_method_time_trace(&o->m[mi], mi, qr, px, py, pw, ph);

  draw_reference_time_trace(o, px, py, pw, ph);

  draw_panel_title(py, px, "Time Series  q(t)");
  attrset(A_NORMAL);
}

/* ===================================================================== */
/* §8  HUD overlay — top status bar + per-method lines + bottom hints    */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * HUD overlay  (top status bar + per-method lines + bottom hint bar)
 *
 * render_overlay() is the orchestrator.  Reads as pseudocode:
 *
 *     draw_top_status_bar(o, cols, theme, fps)        // row 0
 *     for mi in 0..N_METHODS:
 *         draw_method_status_line(o.m[mi], mi, system, row=1+mi)
 *     draw_bottom_action_bar(rows, cols)              // row rows-1
 *
 * The two chrome bars (top status, bottom hint) use FIXED bright
 * yellow / cyan that stay legible on every theme palette per the
 * CLAUDE.md HUD convention.  Per-method lines use the method's own
 * theme color so colour-to-trail association is unambiguous when the
 * reader scans between the panels and the status block.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * draw_top_status_bar — paint the live "what is the demo doing" line
 * at row 0, RIGHT-aligned in bright yellow + bold (CP_HUD).
 *
 * Fields: active system name, current step size dt, accumulated
 * simulation time t, current theme name, paused/running badge, and
 * the most recent rolling-window fps reading.  Right alignment keeps
 * the chrome out of the way when the terminal is wider than the
 * status string.
 */
static void draw_top_status_bar(const ODE *o, int cols, int theme_idx,
                                int fps) {
  char buf[200];
  snprintf(buf, sizeof buf, " %s  dt:%.3f  t:%6.2f  theme:%s  %s  %d fps ",
           SYS_SHORT_NAME[o->system], o->dt, o->t,
           k_themes[((theme_idx % N_THEMES) + N_THEMES) % N_THEMES].name,
           o->paused ? "PAUSED " : "running", fps);

  int col = cols - (int)strlen(buf);
  if (col < 0)
    col = 0;

  attrset((chtype)COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, cols);
}

/*
 * draw_method_status_line — paint one row of per-integrator diagnostic
 * data in the method's own theme color.
 *
 *   Active  : " [N] Name  err=… max=… E_drift=…%  O(h^k) [symplectic] "
 *   Hidden  : " [N] Name  (hidden — press N to show) "        (CP_GRID)
 *
 * `err_cur` and `err_max` are absolute position errors against the
 * reference (§6); the energy_drift_pct comes from H(s_now)/H₀ − 1
 * (§6).  The accuracy-order tag at the right is the most important
 * pedagogical cue: pairs of methods with the same order (RK2 and
 * Verlet, both O(h²)) line up here, so the reader can SEE that order
 * alone doesn't predict long-term behaviour — Verlet's symplecticity
 * still wins.
 */
static void draw_method_status_line(const Method *m, int mi, int system,
                                    int row) {
  if (m->active) {
    float drift = energy_drift_pct(m, system);
    attrset((chtype)COLOR_PAIR(METHOD_CP[mi]) | A_BOLD);
    mvprintw(row, 0, " [%d] %-6s  err=%7.4f  max=%7.4f  E_drift=%+7.2f%%  %s",
             mi + 1, METHOD_NAME[mi], m->err_cur, m->err_max, drift,
             METHOD_ORDER[mi]);
  } else {
    attrset((chtype)COLOR_PAIR(CP_GRID));
    mvprintw(row, 0, " [%d] %-6s  (hidden — press %d to show)", mi + 1,
             METHOD_NAME[mi], mi + 1);
  }
  clrtoeol();
}

/*
 * draw_bottom_action_bar — paint the action-key reference at row
 * rows-1, LEFT-aligned in bright cyan + bold (CP_HINT).
 *
 * Two variants:
 *   `full`  — every key mapped, fits ~80+ cols
 *   `shrt`  — minimal essentials, fits even on narrow terminals
 * The narrower string is chosen when the full one would overflow.
 */
static void draw_bottom_action_bar(int rows, int cols) {
  const char *full = " q:quit  spc:pause  s:system  r:reset  1-4:toggle  "
                     "+/-:dt  t/T:theme ";
  const char *shrt = " q:quit  spc:pause  1-4:toggle  t:theme ";
  const char *line = (int)strlen(full) >= cols - 1 ? shrt : full;

  attrset((chtype)COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, line, cols);
}

/*
 * render_overlay — top-level HUD orchestrator.  Per the pseudocode at
 * the top of §8 — three named stages with no inline content of its own.
 */
static void render_overlay(const ODE *o, int rows, int cols, int theme_idx,
                           int fps) {
  attrset(A_NORMAL);

  draw_top_status_bar(o, cols, theme_idx, fps);

  for (int mi = 0; mi < N_METHODS; mi++)
    draw_method_status_line(&o->m[mi], mi, o->system, 1 + mi);

  draw_bottom_action_bar(rows, cols);

  attrset(A_NORMAL);
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level UI state bundling the ODE simulation with the
 * window-side bookkeeping the main loop needs.  Single instance `sc`
 * lives on main()'s stack.
 *
 * Field locality (the same simulation-vs-rendering contract as the
 * other physics demos):
 *
 *   ode        ALL simulation state — every integrator, every history
 *              ring, every Hamiltonian/error accumulator.  Mutated by
 *              ode_advance_one_tick (§5) and ode_init (§9).
 *
 *   rows, cols cached terminal geometry; refreshed on SIGWINCH.  Lives
 *              here (not in g_*) so the renderer doesn't need globals.
 *
 *   theme      index into k_themes[] (§3).  Cycling themes is a PURE
 *              VISUAL change: ode state is byte-identical before and
 *              after t / T, only colour-pair RGB is repainted.  This
 *              is what justifies theme being on Scene rather than on
 *              ODE — it's a render param.
 *
 *   fps_disp   rolling 1-second frame-rate readout for the top HUD.
 *              Updated by main()'s frame-counter once per second; the
 *              renderer just consumes it.
 *
 * Why ode is embedded (not a pointer):
 *   ODE is ~12 KB × 4 methods + 12 KB ref + scalars ≈ 60 KB.  Embedding
 *   it lets the whole simulation live on the stack with one Scene
 *   instance — zero malloc, byte-copy snapshotting is trivial if a
 *   future feature wants undo/replay.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  ODE ode;
  int rows, cols;
  int theme;    /* index into k_themes[]; cycled by t / T   */
  int fps_disp; /* most recent rolling-window fps reading   */
} Scene;

/* ─────────────────────────────────────────────────────────────────────── *
 * ode_init — full simulation reset.  Reads as pseudocode:
 *
 *     saved_dt = preserve_user_timestep(o)
 *     clear_simulation_buffers(o)
 *     o.system = sys ; o.dt = saved_dt
 *     apply_initial_condition_to_all(o, sys)
 *     derive_phase_space_window(o, sys)
 *
 * Called at boot, on r/R reset, and whenever the user switches system
 * with 's'.  The only piece of state that SURVIVES a reset is the user-
 * tuned step size dt — every other field is zeroed and re-seeded.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * preserve_user_timestep — extract the dt the user has interactively
 * tuned (with +/-) so it survives the memset that follows.  Returns
 * DT_DEFAULT when no valid dt is found, i.e. on the first call when
 * o is freshly zeroed BSS.
 *
 * Why this matters: pressing r should keep the user's last dt choice
 * — otherwise every reset would snap dt back to DT_DEFAULT and the
 * "+/- to provoke Euler blow-up" workflow becomes infuriating.
 */
static float preserve_user_timestep(const ODE *o) {
  return (o->dt >= DT_MIN) ? o->dt : DT_DEFAULT;
}

/*
 * clear_simulation_buffers — wipe every per-integrator state, every
 * history ring, every error accumulator, every clock.  Implemented as
 * a memset because Method / ODE are POD with no embedded pointers, so
 * "all zero" is a valid initial value for every field.
 */
static void clear_simulation_buffers(ODE *o) { memset(o, 0, sizeof(*o)); }

/*
 * apply_initial_condition_to_all — broadcast the system's t=0 state to
 * the reference solver and every candidate Method, and capture the
 * baseline Hamiltonian energy on each Method for later drift analysis.
 *
 * After this call every integrator starts from the same (q₀, p₀), so
 * any subsequent divergence in §7's panels is purely a property of the
 * INTEGRATOR, not the initial condition — exactly the experimental
 * isolation the demo needs.
 */
static void apply_initial_condition_to_all(ODE *o, int sys) {
  State s0 = initial_condition(sys);
  float e0 = hamiltonian_energy(s0, sys);

  o->ref = s0;
  for (int mi = 0; mi < N_METHODS; mi++) {
    o->m[mi].s = s0;
    o->m[mi].e0 = e0;
    o->m[mi].active = true;
  }
}

/*
 * derive_phase_space_window — compute the (q_range, p_range) half-
 * extents of the phase-portrait viewport for the given system.  Cached
 * on ODE so the render hot loop (§7) doesn't redo the transcendentals.
 *
 *   SYS_OSCILLATOR:
 *       q_range = Q0_OSC · PHASE_SCALE_OSC          (linear)
 *       p_range = Q0_OSC · ω · PHASE_SCALE_OSC      (linear, scaled by ω)
 *       Extra headroom (PHASE_SCALE_OSC=3) so Euler's exponentially
 *       growing orbit stays on screen long enough to be educational.
 *
 *   SYS_PENDULUM:
 *       q_range = Q0_PEND · PHASE_SCALE_PEND        (bounded by ±Q0)
 *       p_range = √(2·g/L·(1−cos Q0_PEND)) · scale
 *
 *       The p_range formula is the max angular speed at the swing's
 *       bottom, from energy conservation:
 *           H(Q0, 0) = H(0, p_max) = ½ p_max²
 *           p_max    = √(2 · (g/L) · (1 − cos Q0_PEND))
 *       PHASE_SCALE_PEND=1.6 is just enough headroom since the orbit
 *       is BOUNDED by energy (closed curve) — no Euler-style growth
 *       to worry about for the pendulum.
 */
static void derive_phase_space_window(ODE *o, int sys) {
  if (sys == SYS_OSCILLATOR) {
    o->q_range = Q0_OSC * PHASE_SCALE_OSC;
    o->p_range = Q0_OSC * OMEGA * PHASE_SCALE_OSC;
  } else {
    float p_max = sqrtf(2.0f * G_OVER_L * (1.0f - cosf(Q0_PEND)));
    o->q_range = Q0_PEND * PHASE_SCALE_PEND;
    o->p_range = p_max * PHASE_SCALE_PEND;
  }
}

/*
 * ode_init — orchestrator (5-line pseudocode of the pipeline above).
 */
static void ode_init(ODE *o, int sys) {
  float saved_dt = preserve_user_timestep(o);

  clear_simulation_buffers(o);
  o->system = sys;
  o->dt = saved_dt;

  apply_initial_condition_to_all(o, sys);
  derive_phase_space_window(o, sys);
}

static void scene_init(Scene *sc, int rows, int cols) {
  sc->rows = rows;
  sc->cols = cols;
  memset(&sc->ode, 0, sizeof(sc->ode));
  ode_init(&sc->ode, SYS_OSCILLATOR);
}

static void scene_resize(Scene *sc, int rows, int cols) {
  sc->rows = rows;
  sc->cols = cols;
}

static void scene_tick(Scene *sc) {
  if (sc->ode.paused)
    return;
  for (int i = 0; i < STEPS_PER_FRAME; i++) {
    ode_advance_one_tick(&sc->ode);
    compute_method_errors(&sc->ode);
  }
}

static void scene_render(const Scene *sc) {
  erase();

  int rows = sc->rows, cols = sc->cols;
  int plot_top = HUD_TOP_ROWS; /* skip status + 4 method lines */
  int plot_rows = rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
  if (plot_rows < 4)
    plot_rows = 4;

  int pw_left = cols / 2;
  int sep_col = pw_left;
  int pw_right = cols - pw_left - 1; /* -1 for separator column */

  /* panel separator */
  {
    attrset((chtype)COLOR_PAIR(CP_GRID) | A_BOLD);
    for (int y = plot_top; y < plot_top + plot_rows; y++)
      mvaddch(y, sep_col, '|');
    attrset(A_NORMAL);
  }

  render_phase_portrait(&sc->ode, 0, plot_top, pw_left, plot_rows);
  if (pw_right > 4)
    render_time_series(&sc->ode, sep_col + 1, plot_top, pw_right, plot_rows);
  render_overlay(&sc->ode, rows, cols, sc->theme, sc->fps_disp);

  refresh();
}

static void scene_key(Scene *sc, int ch) {
  ODE *o = &sc->ode;
  switch (ch) {
  case ' ':
    o->paused = !o->paused;
    break;
  case 's':
    ode_init(o, (o->system + 1) % 2);
    break;
  case '1':
  case '2':
  case '3':
  case '4': {
    int mi = ch - '1';
    o->m[mi].active = !o->m[mi].active;
    /* clear ring buffers so dots vanish immediately on hide */
    o->m[mi].traj_head = 0;
    o->m[mi].traj_count = 0;
    o->m[mi].tser_head = 0;
    o->m[mi].tser_count = 0;
    break;
  }
  case '+':
  case '=':
    o->dt += DT_STEP;
    if (o->dt > DT_MAX)
      o->dt = DT_MAX;
    break;
  case '-':
    o->dt -= DT_STEP;
    if (o->dt < DT_MIN)
      o->dt = DT_MIN;
    break;
  case 'r':
    ode_init(o, o->system);
    break;
  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    theme_apply(sc->theme);
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme);
    break;
  default:
    break;
  }
}

/* ===================================================================== */
/* §10  screen                                                            */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit = 0;

static void handle_sigwinch(int sig) {
  (void)sig;
  g_resize = 1;
}
static void handle_sigint(int sig) {
  (void)sig;
  g_quit = 1;
}

static void screen_init(int boot_theme) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  init_colors(boot_theme);
}

static void screen_fini(void) { endwin(); }

/* ===================================================================== */
/* §11  app                                                               */
/* ===================================================================== */

int main(void) {
  signal(SIGWINCH, handle_sigwinch);
  signal(SIGINT, handle_sigint);

  screen_init(0);

  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  Scene sc;
  scene_init(&sc, rows, cols);

  long next_frame = now_us();
  long fps_window_start = next_frame;
  int fps_frames = 0;

  while (!g_quit) {
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_resize(&sc, rows, cols);
    }

    int ch = getch();
    if (ch == 'q' || ch == 27)
      break;
    scene_key(&sc, ch);

    long t = now_us();
    if (t >= next_frame) {
      scene_tick(&sc);
      scene_render(&sc);
      next_frame = t + FRAME_US;

      /* rolling 1-second fps window */
      fps_frames++;
      if (t - fps_window_start >= 1000000L) {
        sc.fps_disp = fps_frames;
        fps_frames = 0;
        fps_window_start = t;
      }
    }
  }

  screen_fini();
  return 0;
}
