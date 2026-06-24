/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rk_method_comparision.c
 *
 * Race four ways of stepping an ODE forward in time — Euler, RK2, RK4, and
 * Verlet — against a near-exact reference, all from the same starting point.
 * The left panel plots position vs velocity (you can literally watch Euler
 * spiral out of control and Verlet stay on a clean closed loop); the right
 * panel scrolls position over time.  The point: a method's accuracy "order"
 * isn't the whole story — Verlet and RK2 share the same order, yet only
 * Verlet keeps energy honest over the long haul.
 *
 * Background reading (the math behind why each method behaves as it does):
 *   Hairer, Lubich, Wanner, "Geometric Numerical Integration" — the modern
 *   reference on symplectic integrators (energy-preserving steppers).
 *   Goldstein, "Classical Mechanics" Ch. 8 — Hamiltonian phase space.
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

/* ── §1  config ── */

/* The time step. Small steps are accurate but slow to watch; big steps
 * make the inaccurate methods misbehave faster. The user nudges it with
 * +/- between MIN and MAX; STEP is how much each press moves it. */
#define DT_DEFAULT 0.05f
#define DT_MIN 0.005f
/* At the ceiling Euler grows about 6% per step — its blow-up becomes
 * dramatic and obvious, while the better methods still hold steady. */
#define DT_MAX 0.35f
#define DT_STEP 0.005f

/* Spring stiffness, set to 1 so the oscillator's period is a clean 2*PI. */
#define OMEGA 1.0f /* oscillator angular frequency (rad/s) */

/* Pendulum gravity-over-length, also 1 (a "unit" pendulum). */
#define G_OVER_L 1.0f /* pendulum g/L (unit pendulum) */

/* Where each system starts, released from rest (velocity 0). */
#define Q0_OSC 1.0f /* oscillator initial displacement */

/* A deliberately big swing (~80 degrees) so the pendulum's nonlinearity
 * shows: at this angle it's clearly not a plain sine wave, and its period
 * runs noticeably longer than the small-angle guess. */
#define Q0_PEND 1.40f /* pendulum initial angle (rad), ~80 deg */

/* The reference "truth" solver does 32 tiny sub-steps per display step,
 * which makes its error far smaller than the methods we're judging. */
#define REF_SUBSTEPS 32

/* Run a few physics steps per drawn frame so motion is smooth but slow
 * enough that you can actually see the drift pile up second by second. */
#define STEPS_PER_FRAME 3
#define TARGET_FPS 30
#define FRAME_US (1000000 / TARGET_FPS)

/* How much past history each method keeps. The phase trail wants enough
 * points to show a full loop; the time series wants roughly one sample
 * per terminal column. Both are fixed-size rings (oldest point is
 * overwritten), so there's no allocation at runtime. */
#define TRAJ_LEN 1024 /* phase portrait history per method */
#define TIME_LEN 512  /* time-series samples per method */

/* Rows the HUD eats: 5 at the top (one status line + one per method),
 * 1 at the bottom for the key hints. The plots fill what's left. */
#define HUD_TOP_ROWS 5
#define HUD_BOT_ROWS 1

/* Must match the number of palettes in k_themes[] (§3). */
#define N_THEMES 10

/* The four methods being compared. Lives up here because §3's theme code
 * needs the count before §4 defines the method ids. */
#define N_METHODS 4

/* How far the phase-portrait view zooms out, as a multiple of the start
 * amplitude. The oscillator needs extra room so Euler's growing spiral
 * stays on screen; the pendulum can't swing past its release angle, so a
 * tighter view is fine. */
#define PHASE_SCALE_OSC 3.0f
#define PHASE_SCALE_PEND 1.6f

/* ── §2  clock ── */

static long now_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000L + ts.tv_nsec / 1000L;
}

/* ── §3  color / theme ── */

/* Color slots ncurses uses to paint each part of the screen. The four
 * method colors run dim to bright on purpose: the worst method (Euler)
 * is the dimmest so it doesn't shout, the best (Verlet) reads crisp. The
 * HUD and hint bars get fixed bright yellow/cyan so they stay readable on
 * every theme. */
enum {
  CP_M0 = 1,
  CP_M1,
  CP_M2,
  CP_M3, /* method colors, dim -> bright */
  CP_REF,
  CP_GRID,
  CP_TITLE,
  CP_HUD,
  CP_HINT,
};

/*
 * One color palette for the whole demo. Switching themes only repaints
 * these colors — it never touches the simulation, so it's purely a look
 * change. The methods[] ramp runs dimmest (Euler) to brightest (Verlet)
 * to match worst-to-best.
 *
 *   name      what the t/T key shows for this palette
 *   methods   one color per method, dim -> bright (Euler..Verlet)
 *   ref       the reference "truth" dots; the theme's brightest accent
 *             so they read on top of everything
 *   grid      axes and separators; a quiet grey that stays out of the way
 *   title     panel headings
 *
 * Every color sits in the bright half of the 256-color space so nothing
 * vanishes against a black terminal (project palette rule in CLAUDE.md).
 */
typedef struct {
  const char *name;
  short methods[4];
  short ref;
  short grid;
  short title;
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

/* HUD colors, same on every theme so the status/hint bars stay legible. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

static bool g_256;

/* Repaints the color slots for the chosen palette. Safe to call any time
 * (startup, theme key); idx is wrapped so negative values are fine. */
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
    /* Old 8-color terminals: ignore themes, use fixed sensible colors. */
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

/* ── §4  ode ── */

/*
 * One snapshot of a moving system: where it is and how fast it's going.
 * Both test systems are set up so velocity is just p directly (no mass to
 * divide out), which is why the same two numbers describe a sliding spring
 * and a swinging pendulum.
 *
 *   q  position — distance from rest for the spring, or angle from
 *      straight-down for the pendulum (radians)
 *   p  velocity — how fast q is changing (linear speed, or angular speed)
 *
 * In physics terms this is a point in "phase space," and the path it
 * traces is what the left panel draws. For a perfectly conserved system
 * that path is a closed loop; whether a method keeps it closed is exactly
 * the thing the demo is testing. (Goldstein, Classical Mechanics, Ch. 8.)
 */
typedef struct {
  float q; /* position: spring displacement, or pendulum angle (rad) */
  float p; /* velocity: how fast q changes (also doubles as q's slope) */
} State;

/* The two systems we can switch between with the 's' key.  They sit on
 * opposite sides of the simple/hard line:
 *
 *   SYS_OSCILLATOR  A perfect spring.  Its true motion is a clean ellipse
 *                   in the phase view, and one full swing always takes the
 *                   same time no matter how big the swing.  This is the
 *                   meanest test for Euler — its orbit spirals outward from
 *                   the very first step.
 *
 *   SYS_PENDULUM    A real swinging pendulum.  For tiny swings it acts just
 *                   like the spring, but our starting angle (~80 degrees) is
 *                   big, so its motion isn't a plain sine wave and one swing
 *                   takes noticeably longer.  A good test of whether a method
 *                   keeps the right shape, not just the right speed. */
#define SYS_OSCILLATOR 0
#define SYS_PENDULUM 1

/* The actual equations of motion, kept here for the curious (the HUD shows
 * just the short names to keep the top bar narrow):
 *     Harmonic Oscillator   x'' = -w^2 x
 *     Nonlinear Pendulum    t'' = -(g/L) sin t                          */

/* The four contestants, in the order the 1..4 keys toggle them. They run
 * from worst to best: Euler is crude and unstable, RK2 and RK4 are
 * progressively more careful, and Verlet is special — it's no more
 * "accurate" than RK2 on paper, but it keeps energy honest over the long
 * haul, which is the whole point the demo is built to show. */
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

/* These four little functions are the only place the chosen system
 * (spring vs pendulum) actually matters: where it starts, the force on it,
 * the full rate-of-change it feels, and its energy. Every integrator in §5
 * goes through them, so teaching the demo a new system means just adding a
 * case to each of the four — the integrators never change. */

/* initial_condition — release the system from rest at its starting q. */
static State initial_condition(int sys) {
  if (sys == SYS_OSCILLATOR)
    return (State){Q0_OSC, 0.0f};
  /* SYS_PENDULUM */ return (State){Q0_PEND, 0.0f};
}

/* acceleration_at — the push the system feels right now, given where it is.
 * Verlet only needs this force, not the fuller derivative below. */
static float acceleration_at(float q, int sys) {
  if (sys == SYS_OSCILLATOR)
    return -(OMEGA * OMEGA * q);                   /* SHM    */
  /* SYS_PENDULUM */ return -(G_OVER_L * sinf(q)); /* nonlin */
}

/* state_derivative — how fast position and velocity are each changing right
 * now: position changes at the current velocity, velocity changes at the
 * current acceleration. Euler/RK2/RK4 all need this full picture. */
static State state_derivative(State s, int sys) {
  return (State){s.p, acceleration_at(s.q, sys)};
}

/* hamiltonian_energy — the system's total energy: motion energy plus stored
 * "stretch" energy. A perfect system keeps this fixed forever, so §6 watches
 * how much each method lets it wander. The telltale pattern: Euler steadily
 * gains energy, RK2 steadily bleeds it away, RK4 barely moves, and Verlet
 * wobbles around the start without ever drifting off. */
static float hamiltonian_energy(State s, int sys) {
  float kinetic = 0.5f * s.p * s.p;
  if (sys == SYS_OSCILLATOR)
    return kinetic + 0.5f * OMEGA * OMEGA * s.q * s.q;
  /* SYS_PENDULUM */
  return kinetic + G_OVER_L * (1.0f - cosf(s.q));
}

/*
 * Method — everything one contestant needs to run and be drawn.
 *
 * There's one of these per integrator (Euler, RK2, RK4, Verlet). All four
 * start from the exact same point, then go their separate ways — and that
 * parting of ways is the whole show: Euler spirals outward, RK2 spirals
 * inward, RK4 nearly closes its loop, and Verlet closes it cleanly. Each
 * keeps its own little recorder of past positions so the panels can draw
 * the trail behind it. About 12 KB each, all in fixed storage, no malloc.
 *
 * The trails are "ring buffers" — fixed-size lists that wrap around, so the
 * newest point quietly overwrites the oldest. For each ring, `head` marks
 * where the next point goes and `count` stops growing once it's full. To
 * replay a ring oldest-to-newest, start at (head − count) and walk forward,
 * wrapping at the end.
 */
typedef struct {
  /* s — where this contestant is right now (its current position+velocity).
   * Updated every tick. The four methods' s values drifting apart from each
   * other and from the reference is the entire demo. */
  State s;

  /* traj[] — recent (position, velocity) points, for the phase panel's
   * trail. Sized (1024) to comfortably hold one full loop, so Verlet's
   * closed orbit shows without the tail getting cut off. */
  State traj[TRAJ_LEN];
  int traj_head;  /* where the next point goes */
  int traj_count; /* how many points are stored, capped at the size */

  /* tser[] — recent positions only, for the scrolling time panel. Same ring
   * idea. Sized (512) to cover wide terminals; narrow ones just show the
   * last few columns' worth. */
  float tser[TIME_LEN];
  int tser_head;
  int tser_count;

  /* How far this method has strayed from the reference "truth". We track
   * position only — velocity error stays small for this kind of motion, and
   * a second number would just clutter the readout. */

  /* err_cur — how far off position is this very tick. */
  float err_cur;

  /* err_max — the worst it's ever been since the last reset. Only ever goes
   * up (until a reset wipes it), so it answers "what was the worst gap so
   * far?" rather than the noisy moment-to-moment value. */
  float err_max;

  /* e0 — the energy this method had at the very start, recorded once. §6
   * compares the current energy against this baseline to show drift. We save
   * the starting value rather than recompute it so the readout still works
   * if the starting conditions ever change. */
  float e0;

  /* active — whether this method is shown, toggled by keys 1..4. This hides
   * it from the panels only; the physics keeps running underneath. That
   * matters: re-showing a method must reveal exactly where it would have
   * been all along, otherwise toggling would skew the comparison. */
  bool active;
} Method;

/*
 * ODE — the whole simulation in one place.
 *
 * Five things move forward together from the same start: the four
 * contestants in m[], plus ref, the reference "truth". The reference is
 * just RK4 run with 32 tiny sub-steps per display tick, which makes its
 * error far too small to see — close enough to call exact. We keep it as
 * its own field rather than a fifth method on purpose: it's the yardstick,
 * not a contestant, so loops over m[] never accidentally sweep it in (which
 * would defeat the comparison), and it's drawn differently anyway.
 */
typedef struct {
  /* m[] — the four contestants, indexed by M_EULER..M_VERLET. They start
   * identical and then drift apart; that drift is the data we draw. */
  Method m[N_METHODS];

  /* ref — where the reference "truth" is right now. Its own trail rings sit
   * right beside it for the same reason the methods carry theirs. */
  State ref;
  State ref_traj[TRAJ_LEN];
  int ref_traj_head;
  int ref_traj_count;
  float ref_tser[TIME_LEN];
  int ref_tser_head;
  int ref_tser_count;

  /* dt — the time step, the same for everyone so the race stays fair. The
   * +/- keys nudge it; turning it up is how you provoke Euler into blowing
   * up while the better methods hold. */
  float dt;

  /* t — how much simulated time has passed since the last reset (just the
   * sum of all the steps). Shown in the HUD; the integrators don't use it. */
  float t;

  /* system — which setup is running, spring or pendulum. The 's' key flips
   * it and triggers a full reset so everything is re-seeded for the new one. */
  int system;

  /* paused — when set, the simulation freezes in place. Unpausing picks up
   * from the exact same spot, with no jump. */
  bool paused;

  /* step_count — how many steps have run since reset; handy for gauging how
   * fast the methods are pulling apart. Shown in the HUD. */
  int step_count;

  /* q_range / p_range — how far the phase view zooms out, half the width and
   * half the height of the window it shows. They depend on which system is
   * running (the oscillator needs extra room for Euler's growing spiral; the
   * pendulum can't swing past its start, so a tighter view fits). Worked out
   * once per system at reset and just read while drawing — they don't change
   * the physics, only the view. Anything outside the window gets pinned to
   * the panel edge instead of spilling over. */
  float q_range;
  float p_range;
} ODE;

/* ── §5  integrators + one-tick evolution pipeline ── */

/* Every frame does the same three things, in this order:
 *     advance_reference_solution(o)   // step the near-perfect "truth"
 *     advance_candidate_methods(o)    // step the four contestants
 *     accumulate_simulation_clock(o)  // bump the clock and the tick count
 *
 * Each contestant gets its own little stepper, named after the real
 * method it implements (Euler, RK2 midpoint, classic RK4, velocity
 * Verlet).  The "truth" is just RK4 run with 32 tiny sub-steps per
 * frame, which makes its error too small to see. */

/* forward_euler_step — the simplest possible stepper: look at how things are
 * changing right now, and assume they keep changing that way for the whole
 * step. Quick and crude. On a spring it always overshoots a little every
 * step, so its orbit spirals outward no matter how small the step — which is
 * exactly the cautionary tale the demo wants to show. */
static State forward_euler_step(State s, float dt, int sys) {
  State d = state_derivative(s, sys);
  return (State){s.q + dt * d.q, s.p + dt * d.p};
}

/* explicit_midpoint_step — RK2: peek ahead to the middle of the step to get a
 * better sense of how things are changing, then take the whole step using
 * that mid-step rate. Much more accurate than Euler for the same step size.
 * It errs the other way, though — its orbit slowly drifts inward, quietly
 * losing energy. */
static State explicit_midpoint_step(State s, float dt, int sys) {
  State k1 = state_derivative(s, sys);
  State mid = {s.q + 0.5f * dt * k1.q, s.p + 0.5f * dt * k1.p};
  State k2 = state_derivative(mid, sys);
  return (State){s.q + dt * k2.q, s.p + dt * k2.p};
}

/* classical_rk4_step — the workhorse RK4 (Kutta, 1901). It samples the rate
 * of change four times across the step — start, two guesses at the middle,
 * and the end — then blends them, leaning hardest on the middle ones. Very
 * accurate: halve the step and the error shrinks about sixteenfold. But it
 * still isn't energy-honest, so over very long runs its energy creeps. */
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

/* velocity_verlet_step — the star of the demo. It moves the position using
 * the OLD velocity, then averages the force before and after to update the
 * velocity. That little ordering trick is what keeps it energy-honest: its
 * energy wobbles around the starting value forever but never drifts off,
 * even over millions of steps. On paper it's no more accurate than RK2, yet
 * its orbit stays a clean closed loop — which is why it's the method of
 * choice for planetary orbits and molecular dynamics.
 *
 * Refs: Hairer, Lubich, Wanner, "Geometric Numerical Integration" Ch. VI;
 *       Verlet (1967), "Computer Experiments on Classical Fluids". */
static State velocity_verlet_step(State s, float dt, int sys) {
  float a_old = acceleration_at(s.q, sys);
  float q_new = s.q + s.p * dt + 0.5f * a_old * dt * dt;
  float a_new = acceleration_at(q_new, sys);
  float p_new = s.p + 0.5f * (a_old + a_new) * dt;
  return (State){q_new, p_new};
}

/* reference_solution_step — our stand-in for the exact answer. It's just RK4
 * again, but chopped into 32 tiny sub-steps per display step, which shrinks
 * its error below what we could ever notice. Everything else is measured
 * against this in §6. */
static State reference_solution_step(State s, float dt, int sys) {
  float micro_h = dt / (float)REF_SUBSTEPS;
  for (int i = 0; i < REF_SUBSTEPS; i++)
    s = classical_rk4_step(s, micro_h, sys);
  return s;
}

/* dispatch_integrator — the one spot that picks which stepper a given method
 * uses. Keeping the choice here means adding a new method is just writing its
 * stepper and adding one case. */
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

/* Two tiny helpers for adding a point to a trail. Both work the same way:
 * drop the new point in, advance the write spot, and stop the count growing
 * once the trail is full (so the oldest point gets overwritten). */

/* phase_history_push — record one position+velocity point for the phase
 * panel's trail. */
static void phase_history_push(State *ring, int *head, int *count, State s) {
  ring[*head] = s;
  *head = (*head + 1) % TRAJ_LEN;
  if (*count < TRAJ_LEN)
    (*count)++;
}

/* time_history_push — record one position value for the scrolling time
 * panel's trail. */
static void time_history_push(float *ring, int *head, int *count, float v) {
  ring[*head] = v;
  *head = (*head + 1) % TIME_LEN;
  if (*count < TIME_LEN)
    (*count)++;
}

/* advance_reference_solution — move the "truth" forward one step and record
 * it. Goes first each tick so the contestants are judged against the
 * freshest reference value. */
static void advance_reference_solution(ODE *o) {
  o->ref = reference_solution_step(o->ref, o->dt, o->system);
  phase_history_push(o->ref_traj, &o->ref_traj_head, &o->ref_traj_count,
                     o->ref);
  time_history_push(o->ref_tser, &o->ref_tser_head, &o->ref_tser_count,
                    o->ref.q);
}

/* advance_candidate_methods — move all four contestants forward one step and
 * record each one's new spot on its trails. */
static void advance_candidate_methods(ODE *o) {
  for (int mi = 0; mi < N_METHODS; mi++) {
    Method *m = &o->m[mi];
    m->s = dispatch_integrator(mi, m->s, o->dt, o->system);
    phase_history_push(m->traj, &m->traj_head, &m->traj_count, m->s);
    time_history_push(m->tser, &m->tser_head, &m->tser_count, m->s.q);
  }
}

/* accumulate_simulation_clock — bump the running clock and step counter. Its
 * own helper so the tick below reads as three clean named stages. */
static void accumulate_simulation_clock(ODE *o) {
  o->t += o->dt;
  o->step_count += 1;
}

/* ode_advance_one_tick — one full step of the simulation: move the truth,
 * move the contestants, tick the clock. Run a few times per drawn frame. */
static void ode_advance_one_tick(ODE *o) {
  advance_reference_solution(o);
  advance_candidate_methods(o);
  accumulate_simulation_clock(o);
}

/* ── §6  error analysis  (position error vs truth + energy drift) ── */

/* compute_method_errors — for each contestant, measure how far its position
 * has strayed from the truth this tick, and remember the worst gap so far.
 * Position only — see the err fields on Method for why velocity is skipped. */
static void compute_method_errors(ODE *o) {
  for (int mi = 0; mi < N_METHODS; mi++) {
    Method *m = &o->m[mi];
    m->err_cur = fabsf(m->s.q - o->ref.q);
    if (m->err_cur > m->err_max)
      m->err_max = m->err_cur;
  }
}

/* energy_drift_pct — how far a method's energy has wandered from its starting
 * value, as a percent. The sign tells the story: Euler reads positive (gains
 * energy), RK2 negative (loses it), Verlet hovers near zero forever. The
 * guard avoids dividing by zero if the system starts perfectly at rest. */
static float energy_drift_pct(const Method *m, int sys) {
  float e_cur = hamiltonian_energy(m->s, sys);
  if (fabsf(m->e0) < 1e-9f)
    return 0.0f;
  return (e_cur - m->e0) / m->e0 * 100.0f;
}

/* ── §7  plot panels — phase portrait + scrolling time series ── */

/* Two helpers that turn a physical number into a screen cell. One maps a
 * value to a row (bigger is higher up, since math goes up but screen rows go
 * down, the helper flips it); the other maps position to a column (left is
 * negative, right is positive). Both pin anything off the edge to the edge,
 * so a runaway orbit can't scribble into the neighbouring panel. */

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

/* The left panel: position across, velocity up the side. The drawing order
 * matters because later dots paint over earlier ones — the reference "truth"
 * is drawn last so it always stays readable on top, and the title is painted
 * last of all so nothing covers it. */

/* plot_phase_dot — drop one dot where a position+velocity pair lands, but
 * only if it's actually inside the panel. */
static void plot_phase_dot(float q, float p, float qr, float pr, int px, int py,
                           int pw, int ph, char glyph) {
  int sx = physics_q_to_screen_col(q, qr, px, pw);
  int sy = physics_value_to_screen_row(p, pr, py, ph);
  if (sx >= px && sx < px + pw && sy >= py && sy < py + ph)
    mvaddch(sy, sx, (chtype)glyph);
}

/* draw_phase_grid — the crosshair axes through the center of the phase
 * panel, marking zero position and zero velocity. */
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

/* The right panel: position over time, scrolling left. The rightmost column
 * is the newest moment; each column to its left is one step older. Same idea
 * as the phase panel — reference last so it stays on top, title last of all. */

/* draw_time_zero_line — the horizontal baseline marking zero position. */
static void draw_time_zero_line(float qr, int px, int py, int pw, int ph) {
  int zy = physics_value_to_screen_row(0.0f, qr, py, ph);
  attrset((chtype)COLOR_PAIR(CP_GRID) | A_BOLD);
  for (int x = px; x < px + pw; x++)
    mvaddch(zy, x, '-');
}

/* plot_time_ring — draw one trail of positions across the panel, oldest on
 * the left and newest on the right edge. Shared by the methods and the
 * reference (they just pass a different glyph and color). */
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

/* ── §8  HUD overlay — top status bar + per-method lines + bottom hints ── */

/* The on-screen readouts: a status line up top, one line per method, and a
 * key-hint bar at the bottom. The top and bottom bars stay a fixed bright
 * yellow/cyan so they're readable on every theme; each method line uses that
 * method's own color so you can match a line to its dots at a glance. */

/* draw_top_status_bar — the live "what's it doing right now" line at the top
 * right: system, step size, elapsed time, theme, paused/running, and fps. */
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

/* draw_method_status_line — one method's readout: its current and worst-ever
 * position error, its energy drift, and its accuracy tag. The tag is the key
 * takeaway: RK2 and Verlet share the same tag, yet only Verlet keeps energy
 * steady — proof that the accuracy label alone doesn't tell the whole story. */
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

/* draw_bottom_action_bar — the key-hint bar along the bottom. Falls back to a
 * shorter version when the terminal is too narrow to fit the full list. */
static void draw_bottom_action_bar(int rows, int cols) {
  const char *full = " q:quit  spc:pause  s:system  r:reset  1-4:toggle  "
                     "+/-:dt  t/T:theme ";
  const char *shrt = " q:quit  spc:pause  1-4:toggle  t:theme ";
  const char *line = (int)strlen(full) >= cols - 1 ? shrt : full;

  attrset((chtype)COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, line, cols);
}

/* render_overlay — paint all the readouts: top bar, the per-method lines,
 * and the bottom hint bar. */
static void render_overlay(const ODE *o, int rows, int cols, int theme_idx,
                           int fps) {
  attrset(A_NORMAL);

  draw_top_status_bar(o, cols, theme_idx, fps);

  for (int mi = 0; mi < N_METHODS; mi++)
    draw_method_status_line(&o->m[mi], mi, o->system, 1 + mi);

  draw_bottom_action_bar(rows, cols);

  attrset(A_NORMAL);
}

/* ── §9  scene ── */

/*
 * Scene — the simulation plus the bits the window loop needs around it. One
 * of these lives on main's stack.
 *
 *   ode       the entire simulation (every method, every trail, every
 *             reading). The whole thing is embedded, not pointed to, so it
 *             all sits on the stack with no malloc anywhere.
 *   rows,cols the terminal size, refreshed when the window is resized.
 *   theme     which color palette is active. Changing it is purely cosmetic
 *             — the simulation is untouched — which is why it lives here
 *             rather than alongside the physics.
 *   fps_disp  the once-a-second frame-rate reading shown in the HUD.
 */
typedef struct {
  ODE ode;
  int rows, cols;
  int theme;    /* which palette, cycled by t / T */
  int fps_disp; /* most recent frame-rate reading */
} Scene;

/* Resetting the whole simulation, used at startup, on the reset key, and when
 * switching systems. Everything gets wiped and re-seeded except one thing:
 * the step size the user has dialed in stays put across a reset. */

/* preserve_user_timestep — grab the user's chosen step size before the reset
 * clears everything, so pressing reset doesn't snap it back to the default
 * and undo their "+/- to provoke a blow-up" tinkering. */
static float preserve_user_timestep(const ODE *o) {
  return (o->dt >= DT_MIN) ? o->dt : DT_DEFAULT;
}

/* clear_simulation_buffers — zero out everything: states, trails, readings,
 * clock. Safe as a plain wipe since none of these fields hold pointers. */
static void clear_simulation_buffers(ODE *o) { memset(o, 0, sizeof(*o)); }

/* apply_initial_condition_to_all — put the reference and all four methods at
 * the exact same starting point, and record each one's starting energy.
 * Identical starts are the whole point: any difference you see later is the
 * method's doing, not a head start. */
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

/* derive_phase_space_window — work out how far the phase view should zoom out
 * for the chosen system. The spring gets generous headroom (about 3x) so
 * Euler's growing spiral stays on screen; the pendulum can't swing past its
 * release angle, so its fastest point (at the bottom of the swing) sets a
 * tighter view. Done once per system so drawing doesn't redo the math. */
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

/* ode_init — the full reset: keep the user's step size, wipe everything,
 * put all integrators at the start, and set up the view. */
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

/* ── §10  screen ── */

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

/* ── §11  app ── */

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
