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
 *    r        reset simulation
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/rk_method_comparision.c \
 *        -o rk_compare -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config      §2 clock       §3 color      §4 ode
 *  §5 integrate_system           §6 compute_error
 *  §7 render_plot                §8 render_overlay
 *  §9 scene       §10 screen     §11 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
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
#define DT_DEFAULT      0.05f

/* DT_MIN = 0.005: smallest timestep the user can dial down to.
 * Prevents animation from becoming uselessly slow (too many tiny steps);
 * differences between methods are still visible even at this fine scale. */
#define DT_MIN          0.005f

/* DT_MAX = 0.35: ωh = 0.35 rad at the ceiling.
 * For Euler: |R(iωh)| = √(1 + 0.35²) ≈ √1.1225 ≈ 1.060 per step
 * → orbit amplitude grows by 6% every step — exponential blow-up is
 * dramatically visible.  RK2 remains stable up to ωh ≈ 1.41, so it
 * still tracks the reference while Euler flies off screen. */
#define DT_MAX          0.35f

/* DT_STEP = 0.005: increment/decrement per + or - keypress.
 * Fine enough for interactive control; each press changes ωh by 0.005,
 * letting the user nudge into or out of each method's stability region. */
#define DT_STEP         0.005f

/* OMEGA = 1.0 rad/s: angular frequency of the harmonic oscillator.
 * Period T = 2π/ω = 2π ≈ 6.28 s.  Setting ω=1 makes "ωh" numerically
 * equal to dt, simplifying the stability analysis in the header. */
#define OMEGA           1.0f   /* oscillator angular frequency (rad/s) */

/* G_OVER_L = 1.0: effective gravity / pendulum length = 1 (unit pendulum).
 * At small angles the pendulum period = 2π√(L/g) = 2π√(1/1) = 2π s,
 * matching the harmonic oscillator period.  At Q0_PEND = 1.40 rad the
 * nonlinear term makes the actual period noticeably longer than 2π. */
#define G_OVER_L        1.0f   /* pendulum g/L (unit pendulum) */

/* Q0_OSC = 1.0: initial displacement of the harmonic oscillator.
 * With p0=0 this sets the orbit radius in phase space: the exact
 * solution traces a circle of radius 1 in the (q, p/ω) plane. */
#define Q0_OSC          1.0f   /* oscillator initial displacement */

/* Q0_PEND = 1.40 rad ≈ 80°: initial angle for the pendulum.
 * This is deliberately a LARGE angle so nonlinear effects dominate.
 * The small-angle approximation (period ≈ 2π) is noticeably wrong here:
 * the exact period is longer, visible as a phase lag vs the oscillator.
 * The pendulum is NOT a simple sine wave at this amplitude. */
#define Q0_PEND         1.40f  /* pendulum initial angle (rad), ~80 deg */

/* REF_SUBSTEPS = 32: the reference integrator runs RK4 with 32 sub-steps
 * of size h/32 for each display step h.  Global error ≈ O((h/32)⁴)
 * ≈ O(h⁴ / 10⁶) — effectively machine-precision for these parameters.
 * All four methods are compared against this "exact" solution. */
#define REF_SUBSTEPS    32

/* STEPS_PER_FRAME = 3: the simulation advances 3 × dt per rendered frame.
 * At 30 fps this gives 3 × 0.05 = 0.15 sim-seconds per wall-second,
 * so one full oscillator period (T ≈ 6.28 s) takes ~42 wall seconds —
 * slow enough to watch drift accumulate in real time. */
#define STEPS_PER_FRAME  3
#define TARGET_FPS       30
#define FRAME_US         (1000000 / TARGET_FPS)

/* TRAJ_LEN = 1024: ring buffer capacity for phase-portrait trail dots.
 * 1024 past states are kept per method.  Too short → orbit looks partial
 * (less than one full loop visible).  Too long → drawing loop slows down.
 * Ring buffer means the oldest entry is silently overwritten; no malloc. */
#define TRAJ_LEN        1024   /* phase portrait history per method */

/* TIME_LEN = 512: ring buffer capacity for the time-series panel.
 * Each stored sample maps to one terminal column, so 512 should match
 * or exceed the width of a typical wide terminal (e.g. 220 columns).
 * Same ring-buffer overwrite strategy as TRAJ_LEN. */
#define TIME_LEN        512    /* time-series samples per method */

/* HUD_ROWS = 7: number of terminal rows consumed by the status overlay.
 * Breakdown: 1 separator line + 1 system/dt info line + 4 method lines
 * (one per integrator) + 1 key-hint line = 7 rows total. */
#define HUD_ROWS         7

/* PHASE_SCALE_OSC = 3.0: the phase-portrait viewport spans
 * ±(Q0_OSC × 3.0) = ±3.0 in both q and p (scaled by ω).
 * The extra headroom (3× the initial amplitude) is needed so Euler's
 * exponentially growing orbit stays on screen for a useful demo time. */
#define PHASE_SCALE_OSC  3.0f

/* PHASE_SCALE_PEND = 1.6: pendulum viewport spans ±(Q0_PEND × 1.6).
 * The pendulum's position is bounded by ±Q0_PEND (it can't swing past
 * its release angle), so less headroom is needed than for the oscillator.
 * Velocity range is also bounded (energy conservation), so 1.6× suffices. */
#define PHASE_SCALE_PEND 1.6f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000L + ts.tv_nsec / 1000L;
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

#define CP_EULER    1
#define CP_RK2      2
#define CP_RK4      3
#define CP_VERLET   4
#define CP_REF      5
#define CP_GRID     6
#define CP_HUD      7
#define CP_TITLE    8
#define CP_LABEL    9

static void init_colors(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_EULER,   COLOR_RED,     -1);
    init_pair(CP_RK2,     COLOR_YELLOW,  -1);
    init_pair(CP_RK4,     COLOR_GREEN,   -1);
    init_pair(CP_VERLET,  COLOR_CYAN,    -1);
    init_pair(CP_REF,     COLOR_WHITE,   -1);
    init_pair(CP_GRID,    COLOR_BLACK,   -1);
    init_pair(CP_HUD,     COLOR_WHITE,   -1);
    init_pair(CP_TITLE,   COLOR_BLUE,    -1);
    init_pair(CP_LABEL,   COLOR_MAGENTA, -1);
}

/* ===================================================================== */
/* §4  ode                                                                */
/* ===================================================================== */

/* State: the two degrees of freedom of the Hamiltonian system.
 *   q = generalized position — displacement (oscillator) or angle (pendulum).
 *   p = generalized momentum — velocity here because mass = 1.
 * Hamiltonian mechanics notation: the phase space is the (q, p) plane,
 * and trajectories are level sets of H(q, p) = constant. */
typedef struct { float q, p; } State; /* position, momentum/velocity */

#define SYS_OSCILLATOR  0
#define SYS_PENDULUM    1

static const char *SYS_NAME[2] = {
    "Harmonic Oscillator  (x'' = -w^2 x)",
    "Nonlinear Pendulum   (t'' = -(g/L) sin t)"
};

#define N_METHODS  4
#define M_EULER    0
#define M_RK2      1
#define M_RK4      2
#define M_VERLET   3

static const char *METHOD_NAME[N_METHODS]  = {"Euler",  "RK2",    "RK4",    "Verlet"};
static const int   METHOD_CP[N_METHODS]    = {CP_EULER, CP_RK2,   CP_RK4,   CP_VERLET};
static const char *METHOD_ORDER[N_METHODS] = {"O(h)",   "O(h^2)", "O(h^4)", "O(h^2) symplectic"};

static State ic(int sys)
{
    if (sys == SYS_OSCILLATOR) return (State){ Q0_OSC, 0.0f };
    return (State){ Q0_PEND, 0.0f };
}

static float accel(float q, int sys)
{
    if (sys == SYS_OSCILLATOR) return -(OMEGA * OMEGA * q);
    return -(G_OVER_L * sinf(q));
}

static State deriv(State s, int sys)
{
    return (State){ s.p, accel(s.q, sys) };
}

/* energy: Hamiltonian H(q, p) = kinetic + potential energy.
 *
 *   Oscillator: H = ½p² + ½ω²q²
 *     KE = ½p²  (kinetic, quadratic in momentum)
 *     PE = ½ω²q²  (quadratic potential → simple harmonic motion)
 *
 *   Pendulum: H = ½p² + (g/L)(1 − cos q)
 *     KE = ½p²  (same kinetic term)
 *     PE = (g/L)(1 − cos q)  (exact nonlinear potential, zero at q=0)
 *     At small q: cos q ≈ 1 − q²/2, so PE ≈ ½(g/L)q² → reduces to oscillator.
 *
 * energy_drift_pct = (E(t) − E₀) / E₀ × 100%
 *   Euler:  positive drift (gains energy)
 *   RK2:    negative drift (loses energy — numerical dissipation)
 *   Verlet: oscillates near 0% — no secular trend (symplectic) */
static float energy(State s, int sys)
{
    float ke = 0.5f * s.p * s.p;
    if (sys == SYS_OSCILLATOR)
        return ke + 0.5f * OMEGA * OMEGA * s.q * s.q;
    return ke + G_OVER_L * (1.0f - cosf(s.q));
}

/* Method: per-integrator state bundled together.
 *
 *   s:           current (q, p) state advanced by this integrator.
 *   traj[]:      ring buffer of TRAJ_LEN past (q,p) points for the phase portrait.
 *                Ring design: traj_head always points at the NEXT write slot;
 *                wraps modulo TRAJ_LEN so the oldest entry is silently overwritten.
 *                No malloc, no O(N) shift — just one write and a modulo increment.
 *   tser[]:      ring buffer of TIME_LEN past q values for the time-series panel.
 *   err_cur:     |q_method − q_ref| at the current timestep (absolute position error).
 *   err_max:     running maximum of err_cur since last reset; shows worst divergence.
 *   e0:          energy at t=0 (stored at reset); required for energy_drift_pct.
 *   active:      when false the method is hidden from both the renderer and updater
 *                (toggled with keys 1–4). */
typedef struct {
    State  s;
    State  traj[TRAJ_LEN];
    int    traj_head;
    int    traj_count;
    float  tser[TIME_LEN];
    int    tser_head;
    int    tser_count;
    float  err_cur;
    float  err_max;
    float  e0;
    bool   active;
} Method;

/* ODE: top-level simulation state shared across all methods.
 *
 *   m[]:          one Method struct per integrator (Euler, RK2, RK4, Verlet).
 *   ref / ref_*:  the high-accuracy reference solution and its ring buffers.
 *   dt:           current timestep h (interactive; changed with +/-).
 *   system:       SYS_OSCILLATOR or SYS_PENDULUM (toggled with 's').
 *   t:            accumulated simulation time in seconds.
 *   q_range:      phase-portrait and time-series half-range for position q.
 *                 The viewport spans [−q_range, +q_range] horizontally.
 *   p_range:      phase-portrait half-range for momentum p (vertical axis). */
typedef struct {
    Method  m[N_METHODS];
    State   ref;
    State   ref_traj[TRAJ_LEN];
    int     ref_traj_head;
    int     ref_traj_count;
    float   ref_tser[TIME_LEN];
    int     ref_tser_head;
    int     ref_tser_count;
    float   dt;
    float   t;
    int     system;
    bool    paused;
    int     step_count;
    float   q_range;   /* phase/time display half-range for q */
    float   p_range;   /* phase display half-range for p */
} ODE;

/* ===================================================================== */
/* §5  integrate_system                                                   */
/* ===================================================================== */

/* euler_step: forward Euler — one derivative evaluation per step.
 * y_{n+1} = y_n + h·f(y_n)
 * "First order" means the Taylor expansion keeps only the O(h) term;
 * the O(h²) and higher terms are truncated, giving LTE = O(h²).
 * For oscillators the amplification factor |R(iωh)| = √(1+ω²h²) > 1
 * for ANY h > 0, so the orbit always spirals outward — unavoidable. */
static State euler_step(State s, float dt, int sys)
{
    State d = deriv(s, sys);
    return (State){ s.q + dt * d.q, s.p + dt * d.p };
}

/* rk2_step: explicit midpoint (Runge-Kutta order 2).
 * k1 = f(y_n)                   — derivative at the start of the interval
 * k2 = f(y_n + ½h·k1)           — derivative at the estimated midpoint
 * y_{n+1} = y_n + h·k2          — advance using the midpoint estimate
 * Using k2 (the midpoint) instead of k1 (the endpoint like Euler) cancels
 * the leading error term → LTE = O(h³), global error = O(h²).
 * The orbit drifts inward because |R(iωh)| < 1 for this method. */
static State rk2_step(State s, float dt, int sys)
{
    State k1 = deriv(s, sys);
    State sm = { s.q + 0.5f * dt * k1.q, s.p + 0.5f * dt * k1.p };
    State k2 = deriv(sm, sys);
    return (State){ s.q + dt * k2.q, s.p + dt * k2.p };
}

/* rk4_step: classical Runge-Kutta order 4 — four derivative evaluations.
 * k1 = f(y_n)                      — slope at step start
 * k2 = f(y_n + ½h·k1)              — slope at midpoint, estimated via k1
 * k3 = f(y_n + ½h·k2)              — slope at midpoint, estimated via k2
 * k4 = f(y_n + h·k3)               — slope at step end, estimated via k3
 * y_{n+1} = y_n + h·(k1 + 2k2 + 2k3 + k4)/6
 * The weighted average (midpoint stages count double) cancels error terms
 * through O(h⁴) → LTE = O(h⁵), global error = O(h⁴).
 * Halving h reduces global error by 16×. Still not symplectic — RK4
 * slowly leaks energy over very long runs (barely visible here). */
static State rk4_step(State s, float dt, int sys)
{
    State k1 = deriv(s, sys);
    State s2 = { s.q + 0.5f * dt * k1.q, s.p + 0.5f * dt * k1.p };
    State k2 = deriv(s2, sys);
    State s3 = { s.q + 0.5f * dt * k2.q, s.p + 0.5f * dt * k2.p };
    State k3 = deriv(s3, sys);
    State s4 = { s.q + dt * k3.q, s.p + dt * k3.p };
    State k4 = deriv(s4, sys);
    return (State){
        s.q + dt * (k1.q + 2.0f * k2.q + 2.0f * k3.q + k4.q) / 6.0f,
        s.p + dt * (k1.p + 2.0f * k2.p + 2.0f * k3.p + k4.p) / 6.0f
    };
}

/* Velocity Verlet — symplectic, requires accel(q) independent of p
 *
 * Algorithm (two acceleration evaluations):
 *   a_n   = accel(q_n)                  — acceleration at current position
 *   q_{n+1} = q_n + p_n·h + ½·a_n·h²   — position update (uses p_n, NOT p_{n+1})
 *   a_{n+1} = accel(q_{n+1})            — acceleration at NEW position
 *   p_{n+1} = p_n + ½·(a_n + a_{n+1})·h — velocity update = trapezoidal average
 *
 * Symplectic because the position update uses p_n (the OLD momentum):
 * this preserves the phase-space area element dq ∧ dp (Liouville's theorem).
 * Consequence: energy E(t) oscillates O(h²) around E₀ forever with no
 * secular drift, even over millions of steps — unlike Euler or RK4.
 * Global positional error is O(h²), same as RK2, but structurally superior
 * for Hamiltonian systems. */
static State verlet_step(State s, float dt, int sys)
{
    float a0 = accel(s.q, sys);
    float q1 = s.q + s.p * dt + 0.5f * a0 * dt * dt;
    float a1 = accel(q1, sys);
    float p1 = s.p + 0.5f * (a0 + a1) * dt;
    return (State){ q1, p1 };
}

/* ref_step: reference solution — RK4 with REF_SUBSTEPS=32 micro-steps.
 * Each display step h is split into 32 sub-steps of size h/32.
 * Global error ≈ O((h/32)⁴) = O(h⁴/1048576) — effectively exact.
 * This is shown in white ('*') and treated as the ground truth against
 * which all four methods are measured. */
static State ref_step(State s, float dt, int sys)
{
    float h = dt / (float)REF_SUBSTEPS;
    for (int i = 0; i < REF_SUBSTEPS; i++)
        s = rk4_step(s, h, sys);
    return s;
}

static State step_method(int method, State s, float dt, int sys)
{
    switch (method) {
        case M_EULER:   return euler_step(s, dt, sys);
        case M_RK2:     return rk2_step(s, dt, sys);
        case M_RK4:     return rk4_step(s, dt, sys);
        case M_VERLET:  return verlet_step(s, dt, sys);
        default:        return s;
    }
}

/* traj_push: append one (q,p) state to a ring buffer for the phase portrait.
 * Ring buffer logic:
 *   1. Write the new state at traj[*head].
 *   2. Advance head: *head = (*head + 1) % TRAJ_LEN
 *      → head wraps back to 0 after reaching TRAJ_LEN-1.
 *      → oldest entry is silently overwritten; no malloc, no O(N) shift.
 *   3. Clamp count at TRAJ_LEN (stops incrementing once buffer is full).
 * To iterate all stored states in chronological order, start at index
 * (head - count + TRAJ_LEN) % TRAJ_LEN and walk forward count steps. */
static void traj_push(State *traj, int *head, int *count, State s)
{
    traj[*head] = s;
    *head = (*head + 1) % TRAJ_LEN;
    if (*count < TRAJ_LEN) (*count)++;
}

/* tser_push: same ring-buffer logic as traj_push but stores a single
 * float (the position q) for the scrolling time-series panel.
 * buf[*head] is the NEXT write slot; (*head + 1) % TIME_LEN wraps it. */
static void tser_push(float *buf, int *head, int *count, float v)
{
    buf[*head] = v;
    *head = (*head + 1) % TIME_LEN;
    if (*count < TIME_LEN) (*count)++;
}

static void ode_step(ODE *o)
{
    o->ref = ref_step(o->ref, o->dt, o->system);
    traj_push(o->ref_traj, &o->ref_traj_head, &o->ref_traj_count, o->ref);
    tser_push(o->ref_tser, &o->ref_tser_head, &o->ref_tser_count, o->ref.q);

    for (int mi = 0; mi < N_METHODS; mi++) {
        Method *m = &o->m[mi];
        m->s = step_method(mi, m->s, o->dt, o->system);
        traj_push(m->traj, &m->traj_head, &m->traj_count, m->s);
        tser_push(m->tser, &m->tser_head, &m->tser_count, m->s.q);
    }

    o->t += o->dt;
    o->step_count++;
}

/* ===================================================================== */
/* §6  compute_error                                                      */
/* ===================================================================== */

static void compute_error(ODE *o)
{
    for (int mi = 0; mi < N_METHODS; mi++) {
        Method *m = &o->m[mi];
        m->err_cur = fabsf(m->s.q - o->ref.q);
        if (m->err_cur > m->err_max)
            m->err_max = m->err_cur;
    }
}

/* energy_drift_pct: fractional energy error as a percentage.
 *   drift = (E(t) − E₀) / E₀ × 100%
 * E₀ = m->e0, stored once at reset (so relative drift is always w.r.t. t=0).
 * Guard against division by near-zero initial energy (e.g. q0=p0=0).
 * Expected signs:
 *   Euler  → positive drift (energy injected each step)
 *   RK2    → negative drift (numerical dissipation)
 *   Verlet → oscillates near 0% (symplectic, no secular trend) */
static float energy_drift_pct(const Method *m, int sys)
{
    float e_cur = energy(m->s, sys);
    if (fabsf(m->e0) < 1e-9f) return 0.0f;
    return (e_cur - m->e0) / m->e0 * 100.0f;
}

/* ===================================================================== */
/* §7  render_plot                                                        */
/* ===================================================================== */

/* map_val: converts a physical value v ∈ [−range, +range] to a screen row
 * (or column) within the interval [base, base+span).
 *
 * Formula:  t = (range − v) / (2·range)
 *   When v = +range → t = 0   → row = base        (top of panel)
 *   When v = 0      → t = 0.5 → row = base + span/2 (center)
 *   When v = −range → t = 1   → row = base + span−1 (bottom)
 * So +range maps to the TOP of the screen and −range to the BOTTOM,
 * matching the conventional y-axis orientation.
 * Clamps to [base, base+span−1] so out-of-range values don't corrupt
 * adjacent panels (e.g. Euler's spiraling orbit clipped at the edge). */
static int map_val(float v, float range, int base, int span)
{
    float t = (range - v) / (2.0f * range);           /* 0=top/+range, 1=bot/-range */
    int   r = base + (int)(t * (float)(span - 1) + 0.5f);
    if (r < base) r = base;
    if (r >= base + span) r = base + span - 1;
    return r;
}

/* map_q: converts a position value q ∈ [−qr, +qr] to a screen column
 * within [px, px+pw).  Left edge = −qr, right edge = +qr.
 * Formula: t = (q + qr) / (2·qr)  maps [−qr,+qr] → [0,1] linearly. */
static int map_q(float q, float qr, int px, int pw)   /* q → column */
{
    float t = (q + qr) / (2.0f * qr);
    int   c = px + (int)(t * (float)(pw - 1) + 0.5f);
    if (c < px) c = px;
    if (c >= px + pw) c = px + pw - 1;
    return c;
}

/* draw_phase_panel: renders the q-vs-p phase portrait.
 *
 * Drawing order matters because ncurses mvaddch overwrites whatever was
 * already at that cell.  We draw in this order so that more important
 * elements appear ON TOP:
 *   1. Grid axes (dimmest — background)
 *   2. Method trajectories Euler→RK2→RK4→Verlet (Euler first so later
 *      methods overwrite it where they overlap; reference ends up on top)
 *   3. Reference trajectory last ('*' dots) — always visible on top of all
 *      method dots; a method dot obscuring the reference would be misleading
 *   4. Panel title (top-left corner, drawn last to survive overwriting) */
static void draw_phase_panel(const ODE *o, int px, int py, int pw, int ph)
{
    float qr = o->q_range;
    float pr = o->p_range;

    /* grid axes */
    chtype cur_attr = A_NORMAL;
    attrset(A_NORMAL);

    int ax = map_q(0.0f, qr, px, pw);         /* column for q=0 */
    int ay = map_val(0.0f, pr, py, ph);        /* row for p=0 */

    {
        chtype a = (chtype)COLOR_PAIR(CP_GRID) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        for (int y = py; y < py + ph; y++) mvaddch(y, ax, '|');
        for (int x = px; x < px + pw; x++) mvaddch(ay, x, '-');
        mvaddch(ay, ax, '+');
    }

    /* method trajectories — draw Euler first so reference is on top.
     * Rationale: later draw calls overwrite earlier ones at the same cell.
     * By drawing Euler (worst method) first and the reference ('*') last,
     * the reference dots are always visible even when they coincide with
     * a method's trail — making divergence immediately apparent. */
    for (int mi = 0; mi < N_METHODS; mi++) {
        const Method *m = &o->m[mi];
        if (!m->active) continue;

        chtype a = (chtype)COLOR_PAIR(METHOD_CP[mi]);
        if (a != cur_attr) { attrset(a); cur_attr = a; }

        int cnt   = m->traj_count;
        int start = (m->traj_head - cnt + TRAJ_LEN) % TRAJ_LEN;
        for (int i = 0; i < cnt; i++) {
            const State *s = &m->traj[(start + i) % TRAJ_LEN];
            int sx = map_q(s->q, qr, px, pw);
            int sy = map_val(s->p, pr, py, ph);
            if (sx >= px && sx < px + pw && sy >= py && sy < py + ph)
                mvaddch(sy, sx, '.');
        }

        /* current position: bold method character */
        {
            int sx = map_q(m->s.q, qr, px, pw);
            int sy = map_val(m->s.p, pr, py, ph);
            if (sx >= px && sx < px + pw && sy >= py && sy < py + ph) {
                chtype ab = (chtype)COLOR_PAIR(METHOD_CP[mi]) | A_BOLD;
                if (ab != cur_attr) { attrset(ab); cur_attr = ab; }
                mvaddch(sy, sx, (chtype)('1' + mi));
            }
        }
    }

    /* reference trajectory on top.
     * Drawn LAST so '*' always overwrites any method dot at the same cell.
     * The reference is the ground truth — it must always be readable. */
    {
        chtype a = (chtype)COLOR_PAIR(CP_REF);
        if (a != cur_attr) { attrset(a); cur_attr = a; }

        int cnt   = o->ref_traj_count;
        int start = (o->ref_traj_head - cnt + TRAJ_LEN) % TRAJ_LEN;
        for (int i = 0; i < cnt; i++) {
            const State *s = &o->ref_traj[(start + i) % TRAJ_LEN];
            int sx = map_q(s->q, qr, px, pw);
            int sy = map_val(s->p, pr, py, ph);
            if (sx >= px && sx < px + pw && sy >= py && sy < py + ph)
                mvaddch(sy, sx, '*');
        }

        {
            int sx = map_q(o->ref.q, qr, px, pw);
            int sy = map_val(o->ref.p, pr, py, ph);
            if (sx >= px && sx < px + pw && sy >= py && sy < py + ph) {
                chtype ab = (chtype)COLOR_PAIR(CP_REF) | A_BOLD;
                if (ab != cur_attr) { attrset(ab); cur_attr = ab; }
                mvaddch(sy, sx, 'R');
            }
        }
    }

    /* panel title */
    {
        chtype a = (chtype)COLOR_PAIR(CP_TITLE) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        mvaddstr(py, px + 2, "Phase Portrait  q vs p");
    }

    attrset(A_NORMAL);
}

static void draw_time_panel(const ODE *o, int px, int py, int pw, int ph)
{
    float qr = o->q_range;
    chtype cur_attr = A_NORMAL;
    attrset(A_NORMAL);

    /* zero line */
    int zy = map_val(0.0f, qr, py, ph);
    {
        chtype a = (chtype)COLOR_PAIR(CP_GRID) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        for (int x = px; x < px + pw; x++) mvaddch(zy, x, '-');
    }

    /* method time series */
    for (int mi = 0; mi < N_METHODS; mi++) {
        const Method *m = &o->m[mi];
        if (!m->active) continue;

        chtype a = (chtype)COLOR_PAIR(METHOD_CP[mi]);
        if (a != cur_attr) { attrset(a); cur_attr = a; }

        int show = (m->tser_count < pw) ? m->tser_count : pw;
        for (int sx = 0; sx < show; sx++) {
            /* sx=0 oldest visible, sx=show-1 newest */
            int back = show - 1 - sx;
            int idx  = ((m->tser_head - 1 - back) % TIME_LEN + TIME_LEN) % TIME_LEN;
            float v  = m->tser[idx];
            int   sy = map_val(v, qr, py, ph);
            mvaddch(sy, px + (pw - show) + sx, '.');
        }
    }

    /* reference time series on top */
    {
        chtype a = (chtype)COLOR_PAIR(CP_REF);
        if (a != cur_attr) { attrset(a); cur_attr = a; }

        int show = (o->ref_tser_count < pw) ? o->ref_tser_count : pw;
        for (int sx = 0; sx < show; sx++) {
            int back = show - 1 - sx;
            int idx  = ((o->ref_tser_head - 1 - back) % TIME_LEN + TIME_LEN) % TIME_LEN;
            float v  = o->ref_tser[idx];
            int   sy = map_val(v, qr, py, ph);
            mvaddch(sy, px + (pw - show) + sx, '*');
        }
    }

    /* panel title */
    {
        chtype a = (chtype)COLOR_PAIR(CP_TITLE) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        mvaddstr(py, px + 2, "Time Series  q(t)");
    }

    attrset(A_NORMAL);
}

/* ===================================================================== */
/* §8  render_overlay                                                     */
/* ===================================================================== */

static void render_overlay(const ODE *o, int rows, int cols)
{
    int y = rows - HUD_ROWS;
    chtype cur_attr = A_NORMAL;
    attrset(A_NORMAL);

    /* separator */
    {
        chtype a = (chtype)COLOR_PAIR(CP_GRID) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        for (int x = 0; x < cols; x++) mvaddch(y, x, '=');
    }
    y++;

    /* system / timestep info */
    {
        chtype a = (chtype)COLOR_PAIR(CP_HUD) | A_BOLD;
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        mvprintw(y, 0, "%-42s  dt=%.3f  t=%7.2f  steps=%-5d%s",
            SYS_NAME[o->system], o->dt, o->t, o->step_count,
            o->paused ? "  [PAUSED]" : "");
    }
    y++;

    /* method lines */
    for (int mi = 0; mi < N_METHODS; mi++) {
        const Method *m = &o->m[mi];

        if (m->active) {
            float drift = energy_drift_pct(m, o->system);
            chtype a = (chtype)COLOR_PAIR(METHOD_CP[mi]) | A_BOLD;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvprintw(y + mi, 0,
                "[ON] [%d] %-6s  err=%8.4f  max=%8.4f  E_drift=%+7.2f%%  %s",
                mi + 1, METHOD_NAME[mi],
                m->err_cur, m->err_max, drift, METHOD_ORDER[mi]);
        } else {
            chtype a = (chtype)COLOR_PAIR(CP_GRID) | A_BOLD;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvprintw(y + mi, 0,
                "[--] [%d] %-6s  (hidden — press %d to show)",
                mi + 1, METHOD_NAME[mi], mi + 1);
            /* erase rest of line */
            clrtoeol();
        }
    }
    y += N_METHODS;

    /* key hints */
    {
        chtype a = (chtype)COLOR_PAIR(CP_LABEL);
        if (a != cur_attr) { attrset(a); cur_attr = a; }
        mvprintw(y, 0,
            "s=system  1-4=toggle  +/-=dt  r=reset  space=pause  q=quit"
            "   * = reference (RK4 x32 substeps)");
    }

    attrset(A_NORMAL);
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

typedef struct {
    ODE  ode;
    int  rows, cols;
} Scene;

static void ode_init(ODE *o, int sys)
{
    float saved_dt = o->dt;
    bool  had_dt   = (saved_dt >= DT_MIN);

    memset(o, 0, sizeof(*o));
    o->system = sys;
    o->dt     = had_dt ? saved_dt : DT_DEFAULT;

    State s0 = ic(sys);
    o->ref = s0;

    for (int mi = 0; mi < N_METHODS; mi++) {
        o->m[mi].s      = s0;
        o->m[mi].e0     = energy(s0, sys);
        o->m[mi].active = true;
    }

    /* display ranges: large enough to show Euler spiraling ~3x amplitude */
    if (sys == SYS_OSCILLATOR) {
        o->q_range = Q0_OSC  * PHASE_SCALE_OSC;
        o->p_range = Q0_OSC  * OMEGA * PHASE_SCALE_OSC;
    } else {
        float vmax  = sqrtf(2.0f * G_OVER_L * (1.0f - cosf(Q0_PEND)));
        o->q_range  = Q0_PEND * PHASE_SCALE_PEND;
        o->p_range  = vmax    * PHASE_SCALE_PEND;
    }
}

static void scene_init(Scene *sc, int rows, int cols)
{
    sc->rows = rows;
    sc->cols = cols;
    memset(&sc->ode, 0, sizeof(sc->ode));
    ode_init(&sc->ode, SYS_OSCILLATOR);
}

static void scene_resize(Scene *sc, int rows, int cols)
{
    sc->rows = rows;
    sc->cols = cols;
}

static void scene_tick(Scene *sc)
{
    if (sc->ode.paused) return;
    for (int i = 0; i < STEPS_PER_FRAME; i++) {
        ode_step(&sc->ode);
        compute_error(&sc->ode);
    }
}

static void scene_render(const Scene *sc)
{
    erase();

    int rows = sc->rows, cols = sc->cols;
    int plot_rows = rows - HUD_ROWS;
    if (plot_rows < 4) plot_rows = 4;

    int pw_left  = cols / 2;
    int sep_col  = pw_left;
    int pw_right = cols - pw_left - 1;   /* -1 for separator column */

    /* panel separator */
    {
        attrset((chtype)COLOR_PAIR(CP_GRID) | A_BOLD);
        for (int y = 0; y < plot_rows; y++) mvaddch(y, sep_col, '|');
        attrset(A_NORMAL);
    }

    draw_phase_panel(&sc->ode, 0, 0, pw_left, plot_rows);
    if (pw_right > 4)
        draw_time_panel(&sc->ode, sep_col + 1, 0, pw_right, plot_rows);
    render_overlay(&sc->ode, rows, cols);

    refresh();
}

static void scene_key(Scene *sc, int ch)
{
    ODE *o = &sc->ode;
    switch (ch) {
        case ' ':   o->paused = !o->paused;              break;
        case 's':   ode_init(o, (o->system + 1) % 2);   break;
        case '1': case '2': case '3': case '4': {
            int mi = ch - '1';
            o->m[mi].active = !o->m[mi].active;
            /* clear ring buffers so dots vanish immediately on hide */
            o->m[mi].traj_head  = 0;
            o->m[mi].traj_count = 0;
            o->m[mi].tser_head  = 0;
            o->m[mi].tser_count = 0;
            break;
        }
        case '+': case '=':
            o->dt += DT_STEP;
            if (o->dt > DT_MAX) o->dt = DT_MAX;
            break;
        case '-':
            o->dt -= DT_STEP;
            if (o->dt < DT_MIN) o->dt = DT_MIN;
            break;
        case 'r':   ode_init(o, o->system);              break;
        default:    break;
    }
}

/* ===================================================================== */
/* §10  screen                                                            */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int sig) { (void)sig; g_resize = 1; }
static void handle_sigint(int sig)   { (void)sig; g_quit   = 1; }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    init_colors();
}

static void screen_fini(void)
{
    endwin();
}

/* ===================================================================== */
/* §11  app                                                               */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGINT,   handle_sigint);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene sc;
    scene_init(&sc, rows, cols);

    long next_frame = now_us();

    while (!g_quit) {
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&sc, rows, cols);
        }

        int ch = getch();
        if (ch == 'q' || ch == 27) break;
        scene_key(&sc, ch);

        long t = now_us();
        if (t >= next_frame) {
            scene_tick(&sc);
            scene_render(&sc);
            next_frame = t + FRAME_US;
        }
    }

    screen_fini();
    return 0;
}
