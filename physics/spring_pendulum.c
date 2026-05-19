/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spring_pendulum.c  —  ncurses ASCII spring pendulum
 *
 * A mass hangs from a fixed pivot at the top centre via a spring that
 * can both rotate (pendulum) and stretch/compress (spring motion).
 * When ω_spring ≈ 2 × ω_pendulum the two modes exchange energy and
 * the bob traces a complex rosette path before slowly settling.
 *
 * Physics: polar-coordinate spring pendulum (Lagrangian mechanics).
 *
 *   r   — current spring length  (pixels)
 *   θ   — angle from downward vertical  (rad, + = right)
 *
 *   Equations of motion (y-axis positive downward):
 *     r̈  =  r·θ̇²  +  g·cos θ  −  (k/m)·(r − r₀)  −  d·ṙ
 *     θ̈  =  −[g·sin θ + 2·ṙ·θ̇] / r  −  d·θ̇
 *
 *   Integration: semi-implicit (symplectic) Euler.
 *     Velocities are updated first, then positions.
 *     Conserves the symplectic structure — no long-term energy drift.
 *
 * The Preset table drives the initial conditions AND the constants
 * (k, g, damping, rest length) — same code, ten very different motions
 * from a simple straight pendulum to chaotic large-angle coupling.
 *
 * Coordinate spaces (same convention as bounce_ball.c):
 *   Pixel space  — square, CELL_W × CELL_H sub-pixels per cell.
 *                  All physics lives here.
 *   Cell space   — terminal columns and rows.
 *                  Only drawing converts to cell coords.
 *
 * Rendering (three independent tricks layered):
 *   1. Fixed-step sim + render interpolation: sim ticks at 120 Hz,
 *      frames render at ~60 Hz.  (prev_r, prev_θ) is the state at the
 *      end of the previous tick; the draw state lerps between prev_*
 *      and current with alpha = sim_accum / tick_ns.  See [6].
 *   2. Bresenham line rasterisation between spring-coil nodes — the
 *      glyph per step ('\' '/' '-' '|') is chosen from the local step
 *      direction so the spring stays visually connected at any angle.
 *   3. ncurses internal double buffer (newscr / curscr) — erase()
 *      clears the back buffer in memory only; wnoutrefresh()+doupdate()
 *      writes ONE diff to the terminal per frame, eliminating flicker.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset (re-apply current preset's initial conditions)
 *   n / p     next / previous preset (10 presets, simple → very complex)
 *   t / T     next / previous theme   (10 themes)
 *   ]  [      decrease / increase damping
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra spring_pendulum.c -o spring_pendulum
 * -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — every tunable constant
 *   §2  clock     — monotonic ns clock + sleep
 *   §3  color     — 10 themes; HUD pairs fixed across themes
 *   §4  coords    — pixel↔cell conversion; aspect-ratio fix
 *   §5  pendulum  — state struct + EOM phase helpers + tick orchestrator
 *   §6  presets   — 10 named regimes (ICs + constants + pedagogical context)
 *   §7  scene     — composition + Bresenham helpers + scene_draw layers
 *                   (bar → wire → coils → nodes → bob)
 *   §8  screen    — single stdscr; CLAUDE.md HUD layout
 *   §9  app       — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Symplectic (semi-implicit) Euler integration.
 *                  Update velocities ṙ, θ̇ from accelerations r̈, θ̈ first,
 *                  then update positions r, θ using the new velocities.
 *                  This "leapfrog" ordering conserves a modified Hamiltonian
 *                  — bounded energy error, no secular drift.  See [4].
 *
 * Physics        : Spring pendulum (elastic pendulum / "swinging spring").
 *                  Two coupled oscillators in polar coordinates:
 *                    Pendulum mode ω_pend   = √(g/r₀)
 *                    Spring   mode ω_spring = √(k/m)
 *                  At ω_spring ≈ 2·ω_pend (parametric 2:1 resonance):
 *                  energy flows periodically between the two modes — the bob
 *                  traces a spirograph-like rosette before slowly reversing.
 *                  Full beat / precession analysis in Lynch 2002 [3].
 *
 * Math           : Polar-coordinate Lagrangian equations of motion.
 *                  Coriolis-like term 2·ṙ·θ̇/r in θ̈ comes from the
 *                  non-inertial polar frame.  Singular at r → 0 — hence the
 *                  radial clamp at 0.2·r₀ with velocity-zero stop, otherwise
 *                  high-energy presets (SLINGSHOT) pump unbounded angular
 *                  velocity through the 1/r term over a few oscillations.
 *
 * Rendering      : Three independent tricks layered together:
 *                    (1) Fixed-step sim + render interpolation: sim at
 *                        120 Hz, frames at ~60 Hz, draw state lerps from
 *                        (prev_r, prev_θ) to (r, θ) by alpha = sim_accum
 *                        / tick_ns.  Smooth motion regardless of frame
 *                        jitter.  Method from Fiedler [6].
 *                    (2) Bresenham line rasterisation [7] between spring
 *                        coil nodes — glyph per step ('\' '/' '-' '|')
 *                        picked from local step direction so the line
 *                        stays visually connected at any angle.
 *                    (3) ncurses internal double-buffer [8] — erase()
 *                        clears the back buffer in memory only;
 *                        wnoutrefresh()+doupdate() writes one diff to the
 *                        terminal per frame, eliminating flicker.
 *
 * References     :  [1] Goldstein, Poole & Safko, "Classical Mechanics",
 *                       3rd ed. (Addison-Wesley 2002) — Lagrangian
 *                       polar-coordinate EOM derivation, chap. 1-2.
 *                   [2] Hand & Finch, "Analytical Mechanics" (CUP 1998) §6
 *                       — coupled oscillators and parametric resonance.
 *                   [3] Lynch, P. (2002), "The Swinging Spring: A Simple
 *                       Model of Atmospheric Balance", Large-Scale
 *                       Atmosphere-Ocean Dynamics II, CUP — canonical
 *                       2:1-resonance analysis for the elastic pendulum.
 *                   [4] Hairer, Lubich, Wanner, "Geometric Numerical
 *                       Integration", 2nd ed. (Springer 2006) §VI —
 *                       symplectic-Euler bounded-energy theorem.
 *                   [5] Bender & Orszag, "Advanced Mathematical Methods
 *                       for Scientists and Engineers" (McGraw-Hill 1978)
 *                       ch. 11 — parametric resonance asymptotics.
 *                   [6] Fiedler, G., "Fix Your Timestep!" (2004) —
 *                       gafferongames.com/post/fix_your_timestep, the
 *                       definitive fixed-step + interpolation writeup.
 *                   [7] Bresenham, J. E. (1965), "Algorithm for computer
 *                       control of a digital plotter", IBM Syst. J. 4(1)
 *                       pp. 25-30 — the integer line rasteriser used for
 *                       the spring coil between nodes.
 *                   [8] Raymond, E. S., "NCURSES Programming HOWTO" —
 *                       tldp.org/HOWTO/NCURSES-Programming-HOWTO, covers
 *                       the newscr/curscr diff buffer used in scene_draw.
 *                   [9] https://en.wikipedia.org/wiki/Elastic_pendulum
 *                  [10] https://en.wikipedia.org/wiki/Symplectic_integrator
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
  SIM_FPS_DEFAULT = 120, /* high rate — spring ODEs need small steps  */
  SIM_FPS_MIN = 30,
  SIM_FPS_MAX = 120,

  FPS_UPDATE_MS = 500,

  N_COILS = 8, /* spring coils drawn between pivot and bob  */

  N_THEMES = 10,
  N_PRESETS = 10,

  THEME_DEFAULT = 2,  /* OCEANIC                                   */
  PRESET_DEFAULT = 4, /* RES-2:1 (the classic spirograph orbit)    */

  /* CLAUDE.md HUD layout:
   *   row 0       — right-aligned bright-yellow + BOLD status
   *   row 1       — left-aligned  bright-yellow  no-bold  parameters
   *   row rows-1  — left-aligned  bright-cyan   + BOLD    key hint
   *   The bar lives just inside the field (row HUD_ROWS_TOP); the pivot
   *   sits one cell below the bar.                                       */
  HUD_ROWS_TOP = 2,
  HUD_ROWS_BOT = 1,
};

/*
 * CELL_W / CELL_H — sub-pixel resolution per terminal cell.
 * Must match the terminal cell aspect ratio (~2 tall : 1 wide).
 * Physics uses pixel units; cell coords only appear in the draw step.
 */
#define CELL_W 8
#define CELL_H 16

/*
 * COIL_SPREAD — perpendicular half-width of the spring zigzag, pixels.
 * Two cells wide on each side of the spring axis.
 */
#define COIL_SPREAD (CELL_W * 2)

/*
 * Damping bounds.  Default value is per-preset; the user can still nudge
 * it with [ and ] while a preset is running.  Clamped to [MIN, MAX].
 */
#define DAMPING_STEP 0.02f
#define DAMPING_MIN 0.00f
#define DAMPING_MAX 0.80f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

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

/*
 * 5 colour-pair slots.  The four scene pairs (BAR / WIRE / SPRING / BALL)
 * are repainted by theme_apply().  The two HUD pairs are fixed across
 * themes so the status text and key hint always read clearly — bright
 * yellow (226) on default bg for the top status / params, bright cyan
 * (51) for the bottom hint, both with A_BOLD.
 */
enum {
  CP_BAR = 1,
  CP_WIRE,
  CP_SPRING,
  CP_BALL,
  CP_HUD,
  CP_HINT,
};

/*
 * Theme — a palette bundle for ONE of the 10 named looks (MATRIX, FIRE …).
 *
 * Intent
 *   The four scene elements that change colour per theme are exactly the
 *   four colour pairs CP_BAR / CP_WIRE / CP_SPRING / CP_BALL.  Themes do
 *   NOT touch CP_HUD or CP_HINT — those stay bright-yellow / bright-cyan
 *   forever so the readout stays legible against any animation.  This
 *   separation is mandated by the CLAUDE.md HUD spec.
 *
 * Why two palette tracks
 *   bar256 … ball256 are 256-colour cube/gray-ramp indices used when the
 *   terminal supports it (COLORS >= 256).  bar8 … ball8 are the 8-colour
 *   fallbacks (one of COLOR_RED, COLOR_GREEN, …).  Picking by terminal
 *   capability at init time means the demo renders cleanly on PuTTY's
 *   8-colour default and on a modern xterm-256color session alike.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every 256-index in this table sits at ≥ 24 in the cube or ≥ 240 in
 *   the gray ramp.  Anything in 16–23 or 232–239 reads as black against
 *   the terminal's default background and becomes invisible.
 *
 * Reference [8] ESR's NCURSES HOWTO §6 explains init_pair() semantics.
 */
typedef struct {
  const char *name; /* short ASCII label shown in HUD            */
  /* 256-colour palette (preferred path)                                */
  short bar256;    /* CP_BAR  — pivot bar across the top        */
  short wire256;   /* CP_WIRE — straight stubs pivot↔coil↔bob   */
  short spring256; /* CP_SPRING — coil nodes + connecting lines */
  short ball256;   /* CP_BALL — the iron bob "(@)"              */
  /* 8-colour fallback (low-COLORS terminals)                           */
  short bar8;
  short wire8;
  short spring8;
  short ball8;
} Theme;

/*
 * Ten themes.  Each entry has been kept inside the safe-brightness band
 * (cube ≥ 24, gray ≥ 240) so even the "dim" wire colour stays visible
 * over a black background per CLAUDE.md theme rules.
 */
static const Theme k_themes[N_THEMES] = {
    {"MATRIX", /* 256 */ 46, 28, 82, 154,
     /* 8   */ COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
    {"FIRE", /* 256 */ 220, 124, 208, 231,
     /* 8   */ COLOR_YELLOW, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {"OCEANIC", /* 256 */ 195, 31, 51, 231,
     /* 8   */ COLOR_CYAN, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE},
    {"NEON", /* 256 */ 207, 93, 200, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
    {"MONO", /* 256 */ 231, 243, 250, 231,
     /* 8   */ COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
    {"ICE", /* 256 */ 195, 39, 51, 231,
     /* 8   */ COLOR_CYAN, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE},
    {"NOVA", /* 256 */ 207, 92, 200, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
    {"FOREST", /* 256 */ 154, 28, 100, 184,
     /* 8   */ COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW},
    {"DESERT", /* 256 */ 222, 130, 208, 231,
     /* 8   */ COLOR_YELLOW, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {"ECLIPSE", /* 256 */ 219, 89, 197, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  bool full = (COLORS >= 256);
  init_pair(CP_BAR, full ? t->bar256 : t->bar8, -1);
  init_pair(CP_WIRE, full ? t->wire256 : t->wire8, -1);
  init_pair(CP_SPRING, full ? t->spring256 : t->spring8, -1);
  init_pair(CP_BALL, full ? t->ball256 : t->ball8, -1);
  init_pair(CP_HUD, full ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, full ? 51 : COLOR_CYAN, -1);
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                     */
/* ===================================================================== */

/*
 * Same pixel↔cell convention as bounce_ball.c.
 * Physics operates entirely in pixel space.
 * px_to_cell_x/y are the only calls that cross into cell space.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* Forward declaration — preset table defined in §6, used by §5. */
typedef struct Preset Preset;

/* ===================================================================== */
/* §5  pendulum                                                           */
/* ===================================================================== */

/*
 * Pendulum — complete spring-pendulum state.
 *
 * Coordinate system: polar (r, θ) with origin AT THE PIVOT.
 *   ball_px = pivot_px + r · sin(θ)
 *   ball_py = pivot_py + r · cos(θ)        (y axis positive downward)
 *
 * Why polar instead of Cartesian
 *   The natural degrees of freedom of an elastic pendulum are exactly the
 *   spring length r and the swing angle θ.  In Cartesian (x, y) we would
 *   have to enforce the spring extension as a derived quantity each step;
 *   in polar coordinates the EOM derived from the Lagrangian decomposes
 *   cleanly into one radial and one angular equation [1] Goldstein, §1-2;
 *   [3] Lynch 2002.
 *
 * Why velocities AND positions are stored
 *   The EOM is 2nd-order — the symplectic Euler integrator [4] needs the
 *   full state (r, θ, ṙ, θ̇).  Storing ṙ / θ̇ explicitly (instead of
 *   deriving them from successive positions) is what makes the integrator
 *   *symplectic* rather than just "finite-difference Verlet".
 *
 * Why prev_r / prev_theta exist
 *   Pure render aid — they are NOT used by the physics step.  Each tick
 *   snapshots the previous (r, θ) so scene_draw can lerp between
 *   (prev_r, prev_θ) and (r, θ) by alpha = sim_accum / tick_ns.  This
 *   decouples the 120 Hz sim from the ~60 Hz render and gives sub-tick
 *   smoothness.  See [6] Fiedler, "Fix Your Timestep!".
 *
 * Why the constants live PER INSTANCE
 *   Each preset reconfigures (spring_k, gravity, damping, r₀, …) so they
 *   cannot be #define-level globals.  Keeping them inside Pendulum means
 *   pendulum_tick reads only from its argument — no global state, easy
 *   to reason about and trivially reentrant if we ever want a second bob.
 */
typedef struct {
  /* ── State (the EOM unknowns) ─────────────────────────────────── */
  float r;     /* current spring length (pixels), > 0          */
  float theta; /* angle from downward vertical (rad, + = right)*/

  /* ── Velocities (full state for 2nd-order ODE) ────────────────── */
  float r_dot;  /* radial velocity        (px/s)                */
  float th_dot; /* angular velocity       (rad/s)               */

  /* ── Render-interpolation snapshot (NOT read by physics) ──────── */
  float prev_r;     /* r at the end of the previous tick            */
  float prev_theta; /* theta at the end of the previous tick        */

  /* ── Fixed pivot in pixel space (recomputed only on SIGWINCH) ─── */
  float pivot_px; /* horizontal pivot,   pw(cols) / 2             */
  float pivot_py; /* vertical pivot, one cell below the top bar   */

  /* ── Per-preset physical constants (mass = 1 throughout) ──────── */
  float r0;       /* natural rest length of the spring (px)       */
  float spring_k; /* spring stiffness; ω_spring = √k              */
  float gravity;  /* g, in px/s²; ω_pend = √(g/r₀)                */
  float damping;  /* linear damping on both ṙ and θ̇               */
} Pendulum;

static void pendulum_init(Pendulum *p, int cols, int rows, const Preset *pr);

/* ── Phase 0: snapshot for render interpolation ─────────────────────── *
 * Save (r, θ) so scene_draw can lerp from this frame's starting state to
 * the new state after the tick — see [6] Fiedler.  Pure render aid;
 * nothing physics-side reads prev_*.                                    */
static inline void snapshot_state_for_interpolation(Pendulum *p) {
  p->prev_r = p->r;
  p->prev_theta = p->theta;
}

/* ── Phase 1a: radial Lagrangian EOM ────────────────────────────────── *
 *   r̈ = r·θ̇²  +  g·cos θ  −  k·(r − r₀)  −  d·ṙ
 *   ┊    ┊         ┊            ┊              ┊
 *   ┊    ┊         ┊            ┊              └─ linear damping
 *   ┊    ┊         ┊            └──────────────── Hooke's-law spring
 *   ┊    ┊         └──────────────────────────── gravity (radial proj)
 *   ┊    └────────────────────────────────────── centripetal (pseudo)
 *   See [1] Goldstein §1-2 for the polar Lagrangian derivation.         */
static inline float compute_radial_acceleration(const Pendulum *p) {
  return p->r * p->th_dot * p->th_dot + p->gravity * cosf(p->theta) -
         p->spring_k * (p->r - p->r0) - p->damping * p->r_dot;
}

/* ── Phase 1b: angular Lagrangian EOM ───────────────────────────────── *
 *   θ̈ = −[ g·sin θ + 2·ṙ·θ̇ ] / r  −  d·θ̇
 *           ┊             ┊       ┊        ┊
 *           ┊             ┊       ┊        └─── damping on θ̇
 *           ┊             ┊       └──────────── 1/r term — singular at
 *           ┊             ┊                     r → 0, tamed by the
 *           ┊             ┊                     radial clamp below
 *           ┊             └──────────────────── Coriolis-like coupling
 *           └────────────────────────────────── gravity (tangential)   */
static inline float compute_angular_acceleration(const Pendulum *p) {
  return -(p->gravity * sinf(p->theta) + 2.f * p->r_dot * p->th_dot) / p->r -
         p->damping * p->th_dot;
}

/* ── Phase 2: symplectic-Euler step 1 — velocities advance first ────── *
 * The "leapfrog" ordering (velocities BEFORE positions) is what makes
 * the integrator symplectic.  It conserves a modified Hamiltonian and
 * gives bounded energy error to all orders.  See [4] Hairer §VI.        */
static inline void integrate_velocities_symplectic(Pendulum *p, float r_ddot,
                                                   float th_ddot, float dt) {
  p->r_dot += r_ddot * dt;
  p->th_dot += th_ddot * dt;
}

/* ── Phase 3: symplectic-Euler step 2 — positions use NEW velocities ── *
 * The defining feature of semi-implicit Euler: positions are advanced
 * from the just-updated velocities, equivalent to leapfrog up to a
 * half-step phase shift.                                                */
static inline void integrate_positions_from_new_velocities(Pendulum *p,
                                                           float dt) {
  p->r += p->r_dot * dt;
  p->theta += p->th_dot * dt;
}

/* ── Phase 4: keep r out of the 1/r singularity ─────────────────────── *
 * r is clamped to [0.20·r₀, 3.5·r₀].  At each wall the radial velocity
 * pointing INTO the wall is zeroed (inelastic stop) so the bob does not
 * keep injecting momentum frame after frame.  Energy lost here is the
 * cost of staying numerically bounded for high-energy presets like
 * SLINGSHOT — without it the angular EOM's 1/r term pumps θ̇ to
 * arbitrarily large values within a few oscillations.                   */
static inline void enforce_radial_clamp_with_inelastic_stop(Pendulum *p) {
  const float r_min = p->r0 * 0.20f;
  const float r_max = p->r0 * 3.5f;
  if (p->r < r_min) {
    p->r = r_min;
    if (p->r_dot < 0.f)
      p->r_dot = 0.f;
  }
  if (p->r > r_max) {
    p->r = r_max;
    if (p->r_dot > 0.f)
      p->r_dot = 0.f;
  }
}

/*
 * pendulum_tick — ONE symplectic-Euler step of the elastic-pendulum EOM.
 *
 * Pseudocode of the orchestrator below:
 *   snapshot the current state for render interpolation
 *   r̈ ← radial   EOM          (centripetal + gravity + spring + damping)
 *   θ̈ ← angular  EOM          (gravity + Coriolis − damping, has 1/r)
 *   ṙ, θ̇ ← integrate velocities (symplectic order: velocity FIRST)
 *   r, θ ← integrate positions  (using the NEW velocities)
 *   clamp r away from the polar singularity, zero ṙ at the wall
 */
static void pendulum_tick(Pendulum *p, float dt) {
  snapshot_state_for_interpolation(p);

  float r_ddot = compute_radial_acceleration(p);
  float th_ddot = compute_angular_acceleration(p);

  integrate_velocities_symplectic(p, r_ddot, th_ddot, dt);
  integrate_positions_from_new_velocities(p, dt);

  enforce_radial_clamp_with_inelastic_stop(p);
}

/* ===================================================================== */
/* §6  presets                                                            */
/* ===================================================================== */

/*
 * Preset — one named "regime" of the elastic pendulum.
 *
 * Intent
 *   A preset is what the user picks with n/p.  It is NOT just an
 *   initial-conditions vector — each preset bundles the FULL setup that
 *   makes its motion look qualitatively different: the four state ICs
 *   (θ₀, r/r₀, ṙ₀, θ̇₀) AND the four physical constants (k, g, damping,
 *   rest length).  This is essential because the conceptual difference
 *   between, say, ELLIPSE and CIRCLE is not just velocity — it's also
 *   the spring stiffness that decides whether the spring "fights back"
 *   or stays nearly rigid.  Lynch [3] classifies the elastic pendulum's
 *   dynamical regimes by exactly these parameter ratios.
 *
 * Ordering
 *   The k_presets[] table is sorted simple → very complex so n cycles
 *   through a teaching progression:
 *     1 PENDULUM (1 mode) → 2 SPRING (1 mode) → 3 ELLIPSE (linear 2D) →
 *     4 CIRCLE (centripetal balance) → 5,6 RES (resonance) →
 *     7 PRECESS → 8 SLINGSHOT → 9 CHAOS → 10 DECAY.
 *
 * Frequency rule of thumb (50-row terminal, field_h ≈ 47 rows ≈ 752 px):
 *   r₀       = rest_len_frac · 752 px
 *   ω_pend   = √(g / r₀)             [pendulum mode]
 *   ω_spring = √(k)        (m = 1)   [spring mode]
 *
 * Reference
 *   [3] Lynch, P. (2002), "The Swinging Spring" — full regime taxonomy.
 */
struct Preset {
  const char *name; /* short ASCII label shown in HUD            */

  /* ── State at t = 0 (initial conditions) ──────────────────────── */
  float theta0_deg; /* initial angle in degrees from vertical    */
  float r_stretch;  /* r_init / r₀  (1.0 = exactly at rest len)  */
  float r_dot0;     /* initial radial velocity   (px/s)          */
  float th_dot0;    /* initial angular velocity  (rad/s)         */

  /* ── Physical constants (consumed by pendulum_tick) ───────────── */
  float spring_k;      /* spring stiffness; ω_spring = √k           */
  float gravity;       /* g in px/s²;       ω_pend  = √(g/r₀)       */
  float damping;       /* linear damping on ṙ and θ̇                 */
  float rest_len_frac; /* r₀ as fraction of field-pixel height      */
};

static const Preset k_presets[N_PRESETS] = {
    /* ─────────────────────────────────────────────────────────────────
     *  1 PENDULUM
     *  Rigid-spring limit.  k = 400 puts ω_spring ≈ 20 rad/s far above
     *  ω_pend = √(g/r₀) ≈ 2.5 rad/s, so the spring stays nearly at rest
     *  length and all energy lives in the swing.  Initial 25° is well
     *  inside the small-angle (sin θ ≈ θ) regime so the motion is a
     *  clean simple-harmonic swing with period 2π·√(r₀/g) ≈ 2.5 s.
     *  Pedagogy: the baseline against which every other preset compares.
     * ───────────────────────────────────────────────────────────────── */
    {"PENDULUM", 25.0f, 1.00f, 0.0f, 0.0f, 400.0f, 2000.0f, 0.05f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     *  2 SPRING
     *  Pure radial mode.  θ₀ = 0 and θ̇₀ = 0 leave nothing to break the
     *  vertical symmetry, so g·sin θ ≡ 0 and the angular EOM gives
     *  θ̈ = 0 forever.  The bob bounces straight down/up with
     *  ω_spring = √30 ≈ 5.5 rad/s.
     *  Pedagogy: no coupling means no rosette — just a hanging weight.
     * ───────────────────────────────────────────────────────────────── */
    {"SPRING", 0.0f, 1.60f, 0.0f, 0.0f, 30.0f, 2000.0f, 0.05f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     *  3 ELLIPSE
     *  Linear 2-D harmonic oscillator.  Small tangential nudge
     *  (θ̇₀ = 2 rad/s) with the spring near rest length and k = 120
     *  (stiff enough that r barely changes) produces a slowly-rotating
     *  ellipse — the textbook 2-D simple-harmonic motion.
     *  Pedagogy: in the limit k → ∞ the elastic pendulum collapses to a
     *  plain 2-D harmonic oscillator [1] Goldstein §3-3.
     * ───────────────────────────────────────────────────────────────── */
    {"ELLIPSE", 30.0f, 1.00f, 0.0f, 2.0f, 120.0f, 2000.0f, 0.00f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     *  4 CIRCLE
     *  Centripetal balance.  Spinning at θ̇₀ = 5 rad/s with r = 1.2·r₀
     *  and k = 220, the spring tension k·(r − r₀) supplies just enough
     *  inward force to nearly balance gravity at all angles.  Zero
     *  damping → the orbit is stable indefinitely.  r₀ shortened to
     *  0.30·field so the orbit fits visibly on a small terminal.
     *  Pedagogy: a Hooke's-law spring sustains a circle just as well as
     *  the inverse-square law does (Bertrand's theorem [1] §3-6).
     * ───────────────────────────────────────────────────────────────── */
    {"CIRCLE", 0.0f, 1.20f, 0.0f, 5.0f, 220.0f, 2000.0f, 0.00f, 0.30f},

    /* ─────────────────────────────────────────────────────────────────
     *  5 RES-2:1
     *  Parametric 2:1 resonance — the canonical "swinging spring" of
     *  Lynch 2002 [3].  With r₀ = 0.40·field and k = 25 on a 50-row
     *  terminal:
     *      ω_pend  = √(2000 / r₀) ≈ 2.50 rad/s
     *      ω_spring = √25         = 5.00 rad/s  →  ratio = 2.00 ✓
     *  Energy slowly transfers between the two modes; the bob traces a
     *  rosette that contracts, reverses, and re-expands every ~10 s.
     *  Pedagogy: the textbook example of a parametric resonator.
     * ───────────────────────────────────────────────────────────────── */
    {"RES-2:1", 40.0f, 1.15f, 0.0f, 0.0f, 25.0f, 2000.0f, 0.06f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     *  6 RES-3:1
     *  3:1 parametric resonance.  k bumped to 56 → ω_spring ≈ 7.5 ≈
     *  3·ω_pend.  Higher-order resonances exist but with smaller
     *  energy-transfer amplitudes per mode-coupling order, so the
     *  rosette is denser and the breathing is subtler than the 2:1 case.
     *  Pedagogy: not every integer ratio gives equally dramatic beating.
     * ───────────────────────────────────────────────────────────────── */
    {"RES-3:1", 40.0f, 1.10f, 0.0f, 0.0f, 56.0f, 2000.0f, 0.06f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     *  7 PRECESS
     *  Driven precession.  Fast spin (θ̇₀ = 3 rad/s) with a very stiff
     *  spring (k = 300) keeps r close to r₀; the small residual radial
     *  breathing drives a slow rotation of the orbital ellipse's major
     *  axis.  Damping = 0 lets the precession run indefinitely.
     *  Pedagogy: analogous to the Foucault-pendulum plane drift, but the
     *  drift is driven by internal elastic coupling, not Earth rotation.
     * ───────────────────────────────────────────────────────────────── */
    {"PRECESS", 30.0f, 1.00f, 0.0f, 3.0f, 300.0f, 2000.0f, 0.00f, 0.35f},

    /* ─────────────────────────────────────────────────────────────────
     *  8 SLINGSHOT
     *  Energetic snap-in.  Released from r = 2.5·r₀ with the spring
     *  pre-loaded.  Initial spring PE = ½·k·(1.5·r₀)² ≈ 1.45 M px²/s²
     *  for a 50-row terminal.  Bob whips inward fast, reflects off the
     *  0.20·r₀ radial clamp (see pendulum_tick → the 1/r-singularity
     *  fix), then oscillates radially with slow decay at d = 0.04.
     *  Pedagogy: energy initially stored in the spring redistributes
     *  between modes via the Coriolis-like 2ṙθ̇/r coupling.
     * ───────────────────────────────────────────────────────────────── */
    {"SLINGSHOT", 30.0f, 2.50f, 0.0f, 0.0f, 30.0f, 2000.0f, 0.04f, 0.35f},

    /* ─────────────────────────────────────────────────────────────────
     *  9 CHAOS
     *  Past the small-angle horizon.  120° puts the bob nearly inverted
     *  at t = 0 where sin θ is large and the angular EOM is firmly
     *  nonlinear.  Add 1 rad/s spin and zero damping and the trajectory
     *  is non-repeating in any practical sense — sensitive to ICs, no
     *  symmetries, no closed orbits.
     *  Pedagogy: "small-angle ≠ small motion".  Large-angle pendulums are
     *  chaotic; their phase-space trajectories never close.
     * ───────────────────────────────────────────────────────────────── */
    {"CHAOS", 120.0f, 1.40f, 0.0f, 1.0f, 25.0f, 2000.0f, 0.00f, 0.40f},

    /* ─────────────────────────────────────────────────────────────────
     * 10 DECAY
     *  Over-damped collapse.  d = 0.50 (5/8 of DAMPING_MAX) bleeds energy
     *  faster than the modes can couple — by the time 2:1 beating would
     *  start, both modes are already attenuated.  High damping
     *  linearises the response and the bob spirals to rest along its
     *  slowest exponential.
     *  Pedagogy: useful as a clean "watch it stop" reference; also
     *  illustrates that damping eventually wins over every conservative
     *  coupling, no matter how clever the ICs are.
     * ───────────────────────────────────────────────────────────────── */
    {"DECAY", 80.0f, 1.50f, 0.0f, 1.5f, 25.0f, 2000.0f, 0.50f, 0.40f},
};

static void pendulum_init(Pendulum *p, int cols, int rows, const Preset *pr) {
  int field_rows = rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (field_rows < 4)
    field_rows = 4;

  p->pivot_px = (float)pw(cols) * 0.5f;
  p->pivot_py = (float)CELL_H * (float)(HUD_ROWS_TOP + 1);

  p->r0 = (float)ph(field_rows) * pr->rest_len_frac;
  p->r = p->r0 * pr->r_stretch;
  p->theta = (float)(pr->theta0_deg * M_PI / 180.0);
  p->r_dot = pr->r_dot0;
  p->th_dot = pr->th_dot0;
  p->spring_k = pr->spring_k;
  p->gravity = pr->gravity;
  p->damping = pr->damping;

  p->prev_r = p->r;
  p->prev_theta = p->theta;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — the single owner of everything that lives on screen.
 *
 * Intent
 *   Scene is the integration point between the PHYSICS (a Pendulum that
 *   doesn't know what a terminal is) and the RENDERING (an ncurses
 *   surface that doesn't know what an EOM is).  The two layers never
 *   talk directly — they meet here.  Anything that doesn't need to live
 *   in either Pendulum or Screen lives in Scene.
 *
 * Locality (sim vs render)
 *   The fields below are explicitly grouped so a reader can tell at a
 *   glance which subsystem reads each one.  When in doubt: if scene_tick
 *   reads it → simulation; if scene_draw / hud_draw reads it → rendering;
 *   if BOTH read it (cols, rows) → geometry.  Mis-classifying a field is
 *   a source of subtle bugs — a "render" field accidentally read by the
 *   tick would couple physics to the terminal extent, defeating the
 *   sim/render separation that fixed-step + interpolation rests on [6].
 *
 *   The Pendulum itself is the LARGEST piece of simulation state; it's
 *   composed-in rather than allocated separately so a Scene is one
 *   compact struct on the stack (or in BSS as part of g_app).
 *
 * Why these specific fields and no others
 *   - pend                — the only mutable physics state we own.
 *   - preset_idx          — needed by scene_tick? no.  Needed by reset
 *                            ('r') and by HUD?  yes.  Lives here because
 *                            it is "control state" — user choice that
 *                            outlives any single frame.
 *   - paused              — control flag for scene_tick.
 *   - cols, rows          — geometry, shared by sim (pivot placement,
 *                            r₀ computation) and render (clip bounds,
 *                            HUD positioning).  Updated only on resize.
 *   - theme_idx           — pure-render user choice; outlives a resize.
 *
 * Things that DO NOT live here
 *   - Frame timing (sim_accum, frame_count)         → in main()
 *   - Signal flags (running, need_resize)           → in App
 *   - Render buffers                                → owned by ncurses
 *   - Pivot pixel coords                            → in Pendulum
 *   - Active sim FPS                                → in App
 */
typedef struct {
  /* ── Geometry  (shared by simulation AND rendering) ───────────── *
   * Terminal cell extent.  pendulum_init reads these to place the   *
   * pivot at pw(cols)/2 and to compute r₀ as a fraction of pixel-   *
   * space field height.  scene_draw and hud_draw read these as      *
   * clip bounds and HUD row positions.  Reset on SIGWINCH.          */
  int cols;
  int rows;

  /* ── Simulation  (read by scene_tick) ─────────────────────────── *
   * The actual mechanics state plus the control flags that gate it. *
   * preset_idx is technically "control" (selects which row of       *
   * k_presets[] reset reloads from), but it is grouped with sim     *
   * because it survives a theme change unchanged.                   */
  Pendulum pend;
  int preset_idx; /* 0 .. N_PRESETS-1                          */
  bool paused;    /* if true scene_tick is a no-op             */

  /* ── Rendering  (read by scene_draw / hud_draw only) ──────────── *
   * Pure-render user choice.  Changing theme_idx must NEVER touch   *
   * the simulation — it just calls theme_apply() to repaint the     *
   * colour pairs.  Survives SIGWINCH unchanged.                     */
  int theme_idx; /* 0 .. N_THEMES-1                           */
} Scene;

static void scene_init(Scene *s, int cols, int rows, int preset_idx,
                       int theme_idx) {
  s->cols = cols;
  s->rows = rows;
  s->preset_idx = preset_idx;
  s->theme_idx = theme_idx;
  s->paused = false;
  pendulum_init(&s->pend, cols, rows, &k_presets[preset_idx]);
}

static void scene_tick(Scene *s, float dt) {
  if (!s->paused)
    pendulum_tick(&s->pend, dt);
}

/* ── drawing helpers ─────────────────────────────────────────────────── */

/*
 * Bresenham integer-DDA state.  Just the bookkeeping needed by the
 * original 1965 algorithm [7]: per-axis distances, per-axis unit step
 * direction, and the residual that triggers each axis advance.
 */
typedef struct {
  int dx, dy; /* |x1-x0|, |y1-y0|             */
  int sx, sy; /* unit step ±1 in each axis    */
  int err;    /* cumulative slope residual    */
} BresenhamState;

/* Initialise the DDA state for a line from (x0,y0) to (x1,y1).
 * Reference [7] Bresenham 1965, IBM Syst. J., the 4-quadrant symmetric
 * formulation. */
static inline void bresenham_init(BresenhamState *b, int x0, int y0, int x1,
                                  int y1) {
  b->dx = abs(x1 - x0);
  b->dy = abs(y1 - y0);
  b->sx = (x0 < x1) ? 1 : -1;
  b->sy = (y0 < y1) ? 1 : -1;
  b->err = b->dx - b->dy;
}

/* Pick the glyph from the LOCAL step direction so the line looks
 * connected at any slope:
 *     both axes advance  → '/' (anti-diagonal) or '\' (diagonal)
 *     x-only step        → '-'
 *     y-only step        → '|'                                          */
static inline chtype slope_glyph(const BresenhamState *b) {
  int e2 = 2 * b->err;
  bool step_x = (e2 > -b->dy);
  bool step_y = (e2 < b->dx);
  if (step_x && step_y)
    return (b->sx == b->sy) ? '\\' : '/';
  if (step_x)
    return '-';
  (void)step_y;
  return '|';
}

/* Plot only inside the field rect [0, cols) × [HUD_TOP, rows-HUD_BOT)
 * so the spring never overdraws the HUD rows. */
static inline void plot_clipped(int x, int y, chtype glyph, attr_t attr,
                                int cols, int rows) {
  int ymin = HUD_ROWS_TOP;
  int ymax = rows - HUD_ROWS_BOT;
  if (x < 0 || x >= cols || y < ymin || y >= ymax)
    return;
  attron(attr);
  mvaddch(y, x, glyph);
  attroff(attr);
}

/* Advance the DDA one cell along the dominant axis (or both axes if the
 * residual permits a diagonal step). */
static inline void bresenham_advance(BresenhamState *b, int *x, int *y) {
  int e2 = 2 * b->err;
  if (e2 > -b->dy) {
    b->err -= b->dy;
    *x += b->sx;
  }
  if (e2 < b->dx) {
    b->err += b->dx;
    *y += b->sy;
  }
}

/*
 * draw_line — slope-glyph Bresenham line, clipped to the field rect.
 *
 * Pseudocode:
 *   bresenham_init
 *   loop:
 *       plot current cell with slope_glyph, clipped to the field
 *       if at endpoint, break
 *       bresenham_advance
 */
static void draw_line(int x0, int y0, int x1, int y1, int cols, int rows,
                      attr_t attr) {
  BresenhamState b;
  bresenham_init(&b, x0, y0, x1, y1);

  for (;;) {
    plot_clipped(x0, y0, slope_glyph(&b), attr, cols, rows);
    if (x0 == x1 && y0 == y1)
      break;
    bresenham_advance(&b, &x0, &y0);
  }
}

/* A polar sample (r, θ).  Used as the render-interpolated draw state. */
typedef struct {
  float r, theta;
} PolarSample;

/* The spring's local 2-D basis at angle θ.  axis points pivot→bob,
 * perp is 90° CCW from axis.  Both unit vectors. */
typedef struct {
  float ax, ay;
  float perp_x, perp_y;
} SpringBasis;

/* ── Geometry helpers ────────────────────────────────────────────── */

/* Sub-tick render interpolation: blend (prev_r, prev_θ) → (r, θ).
 * alpha ∈ [0,1): fraction of the next tick elapsed.  [6] Fiedler.       */
static inline PolarSample lerp_polar_state(const Pendulum *p, float alpha) {
  PolarSample s;
  s.r = p->prev_r + (p->r - p->prev_r) * alpha;
  s.theta = p->prev_theta + (p->theta - p->prev_theta) * alpha;
  return s;
}

/* Polar (r, θ) anchored at pixel pivot → cell coordinates.
 *   ball_px = pivot_px + r·sin θ
 *   ball_py = pivot_py + r·cos θ      (y positive downward)              */
static inline void polar_to_cell(float pivot_px, float pivot_py, float r,
                                 float theta, int *cx, int *cy) {
  *cx = px_to_cell_x(pivot_px + r * sinf(theta));
  *cy = px_to_cell_y(pivot_py + r * cosf(theta));
}

/* Clamp the bob cell to the visible field rect (pure display clip; the
 * physics position is not modified).                                    */
static inline void clamp_bob_to_field(int *cx, int *cy, int cols, int rows) {
  int ymin = HUD_ROWS_TOP + 1;
  int ymax = rows - HUD_ROWS_BOT - 1;
  if (*cx < 1)
    *cx = 1;
  if (*cx > cols - 2)
    *cx = cols - 2;
  if (*cy < ymin)
    *cy = ymin;
  if (*cy > ymax)
    *cy = ymax;
}

/* Build the spring's local frame: unit axis pivot→bob, unit perpendicular
 * 90° CCW.  Length normalisation is free because sin²+cos²=1.            */
static inline SpringBasis compute_spring_basis(float theta) {
  SpringBasis b;
  b.ax = sinf(theta);
  b.ay = cosf(theta);
  b.perp_x = -b.ay;
  b.perp_y = b.ax;
  return b;
}

/* Lay out 2·N_COILS spring nodes along the axis with alternating
 * ±COIL_SPREAD perpendicular offsets.  Parameter t = (i+1)/(N+1) keeps
 * the endpoints free for the wire stubs at pivot and bob.               */
static inline void lay_out_zigzag_coils(const Pendulum *p, float draw_r,
                                        SpringBasis basis, float *node_px,
                                        float *node_py, int n_nodes) {
  for (int i = 0; i < n_nodes; i++) {
    float t = (float)(i + 1) / (float)(n_nodes + 1);
    float bx = p->pivot_px + t * draw_r * basis.ax;
    float by = p->pivot_py + t * draw_r * basis.ay;
    float sign = (i % 2 == 0) ? 1.f : -1.f;
    node_px[i] = bx + sign * COIL_SPREAD * basis.perp_x;
    node_py[i] = by + sign * COIL_SPREAD * basis.perp_y;
  }
}

/* ── Layer painters (back → front so each layer can overwrite the
 *    cells of the layer beneath it) ──────────────────────────────── */

/* Layer 1: horizontal pivot bar at row HUD_ROWS_TOP with a 'v' marker
 * pointing down to the actual pivot column. */
static inline void draw_pivot_bar(int cols, int pivot_cx) {
  attron(COLOR_PAIR(CP_BAR) | A_BOLD);
  for (int x = 0; x < cols; x++)
    mvaddch(HUD_ROWS_TOP, x, '=');
  if (pivot_cx >= 0 && pivot_cx < cols)
    mvaddch(HUD_ROWS_TOP, pivot_cx, 'v');
  attroff(COLOR_PAIR(CP_BAR) | A_BOLD);
}

/* Layers 2 & 4: a straight wire stub (Bresenham line in CP_WIRE).  Same
 * routine for both stubs — only the endpoints differ. */
static inline void draw_wire_stub(int x0, int y0, int x1, int y1, int cols,
                                  int rows) {
  draw_line(x0, y0, x1, y1, cols, rows, COLOR_PAIR(CP_WIRE));
}

/* Layer 3: Bresenham segments between adjacent coil nodes — these form
 * the zigzag legs of the spring. */
static inline void draw_coil_segments(const float *node_px,
                                      const float *node_py, int n_nodes,
                                      int cols, int rows) {
  for (int i = 0; i < n_nodes - 1; i++) {
    int x0 = px_to_cell_x(node_px[i]);
    int y0 = px_to_cell_y(node_py[i]);
    int x1 = px_to_cell_x(node_px[i + 1]);
    int y1 = px_to_cell_y(node_py[i + 1]);
    draw_line(x0, y0, x1, y1, cols, rows, COLOR_PAIR(CP_SPRING));
  }
}

/* Layer 5: a '*' glyph at every coil node.  Drawn AFTER the segments so
 * the node cell is unambiguous even when several segments cross it. */
static inline void draw_coil_nodes(const float *node_px, const float *node_py,
                                   int n_nodes, int cols, int rows) {
  int ymin = HUD_ROWS_TOP + 1;
  int ymax = rows - HUD_ROWS_BOT;
  attron(COLOR_PAIR(CP_SPRING) | A_BOLD);
  for (int i = 0; i < n_nodes; i++) {
    int cx = px_to_cell_x(node_px[i]);
    int cy = px_to_cell_y(node_py[i]);
    if (cx >= 0 && cx < cols && cy >= ymin && cy < ymax)
      mvaddch(cy, cx, '*');
  }
  attroff(COLOR_PAIR(CP_SPRING) | A_BOLD);
}

/* Layer 6: the iron bob "(@)", drawn last so its three cells are
 * unambiguous over any wire/spring glyphs underneath. */
static inline void draw_iron_bob(int bob_cx, int bob_cy, int cols) {
  attron(COLOR_PAIR(CP_BALL) | A_BOLD);
  if (bob_cx > 0 && bob_cx < cols - 1) {
    mvaddch(bob_cy, bob_cx - 1, '(');
    mvaddch(bob_cy, bob_cx, '@');
    mvaddch(bob_cy, bob_cx + 1, ')');
  } else {
    mvaddch(bob_cy, bob_cx, '@');
  }
  attroff(COLOR_PAIR(CP_BALL) | A_BOLD);
}

/*
 * scene_draw — composite spring-pendulum frame.
 *
 * Pseudocode (each line is one named helper):
 *   draw_state ← lerp_polar_state(prev, curr, alpha)
 *   pivot_cell ← px_to_cell(pivot_px, pivot_py)
 *   bob_cell   ← polar_to_cell(pivot, draw_state); clamp_bob_to_field
 *   basis      ← compute_spring_basis(draw_theta)
 *   nodes      ← lay_out_zigzag_coils(pivot, draw_r, basis)
 *   ── paint back-to-front so each layer overwrites the one beneath ──
 *   draw_pivot_bar
 *   draw_wire_stub (pivot → first coil)
 *   draw_coil_segments
 *   draw_wire_stub (last coil → bob)
 *   draw_coil_nodes
 *   draw_iron_bob
 */
static void scene_draw(const Scene *s, float alpha) {
  const Pendulum *p = &s->pend;
  const int cols = s->cols, rows = s->rows;

  PolarSample draw = lerp_polar_state(p, alpha);

  int pivot_cx = px_to_cell_x(p->pivot_px);
  int pivot_cy = px_to_cell_y(p->pivot_py);
  if (pivot_cy < HUD_ROWS_TOP + 1)
    pivot_cy = HUD_ROWS_TOP + 1;

  int bob_cx, bob_cy;
  polar_to_cell(p->pivot_px, p->pivot_py, draw.r, draw.theta, &bob_cx, &bob_cy);
  clamp_bob_to_field(&bob_cx, &bob_cy, cols, rows);

  SpringBasis basis = compute_spring_basis(draw.theta);

  const int N_NODES = N_COILS * 2;
  float node_px[N_COILS * 2];
  float node_py[N_COILS * 2];
  lay_out_zigzag_coils(p, draw.r, basis, node_px, node_py, N_NODES);

  draw_pivot_bar(cols, pivot_cx);
  {
    int nx0 = px_to_cell_x(node_px[0]);
    int ny0 = px_to_cell_y(node_py[0]);
    int nxN = px_to_cell_x(node_px[N_NODES - 1]);
    int nyN = px_to_cell_y(node_py[N_NODES - 1]);
    draw_wire_stub(pivot_cx, pivot_cy, nx0, ny0, cols, rows);
    draw_coil_segments(node_px, node_py, N_NODES, cols, rows);
    draw_wire_stub(nxN, nyN, bob_cx, bob_cy, cols, rows);
  }
  draw_coil_nodes(node_px, node_py, N_NODES, cols, rows);
  draw_iron_bob(bob_cx, bob_cy, cols);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-extent record + an implicit ncurses double-buffer.
 *
 * Intent
 *   Screen is deliberately tiny.  The actual back-buffer (newscr) and
 *   front-buffer (curscr) live inside ncurses; we only carry the cell
 *   dimensions so callers can clip and place HUD rows.  Everything else
 *   ncurses owns — we never call mvprintw against a private buffer.
 *
 * Render pipeline (one frame)
 *   erase()              clears newscr in memory only, NO terminal I/O
 *   scene_draw()         writes spring + bob → newscr
 *   hud_draw()           writes status / params / hint → newscr (last,
 *                        so HUD always wins overlap with the spring)
 *   wnoutrefresh(stdscr) marks newscr ready, STILL no terminal I/O
 *   doupdate()           ONE atomic diff (newscr vs curscr) → terminal
 *
 * Why one buffer is enough
 *   A second user-managed buffer would just duplicate what ncurses
 *   already does.  The diff in doupdate() only emits the cells that
 *   changed since the last frame, which is what eliminates flicker —
 *   you cannot improve on that by writing your own back-buffer.
 *
 * Reference [8] ESR's NCURSES HOWTO §11 covers the newscr/curscr model.
 */
typedef struct {
  int cols; /* terminal width  in cells (getmaxyx)          */
  int rows; /* terminal height in cells (getmaxyx)          */
} Screen;

static void screen_init(Screen *s, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
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
 * hud_draw() — three HUD elements per CLAUDE.md spec.
 *
 *   row 0       — right-aligned bright-yellow + BOLD: fps + preset + state
 *   row 1       — left-aligned  bright-yellow no-bold: pendulum parameters
 *   row rows-1  — left-aligned  bright-cyan   + BOLD: complete key hint
 *
 * The bottom hint lists every interactive key (q, spc, r, n/p, t/T, [/]).
 */
static void hud_draw(Screen *s, const Scene *sc, double fps) {
  const Pendulum *p = &sc->pend;
  float deg = p->theta * (float)(180.0 / M_PI);
  /* normalise displayed angle to [-180, 180] so the readout stays bounded
   * even when the pendulum spins past full revolutions (CHAOS / CIRCLE). */
  while (deg > 180.f)
    deg -= 360.f;
  while (deg <= -180.f)
    deg += 360.f;
  float stretch = (p->r - p->r0) / p->r0 * 100.0f;

  /* ── row 0: right-aligned status ──────────────────────────────── */
  char status[80];
  int pn = sc->preset_idx + 1;
  snprintf(status, sizeof status, " %5.1f fps  PRESET %2d %-10s  %s ", fps, pn,
           k_presets[sc->preset_idx].name, sc->paused ? "PAUSED " : "running");
  int hx = s->cols - (int)strlen(status);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── row 1: left-aligned parameters ───────────────────────────── */
  char params[160];
  snprintf(params, sizeof params,
           " theta:%+6.1f deg  dr:%+6.1f%%  damp:%.2f  "
           "k:%5.1f  g:%6.0f  theme:%s ",
           deg, stretch, p->damping, p->spring_k, p->gravity,
           k_themes[sc->theme_idx].name);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0, "%s", params);
  attroff(COLOR_PAIR(CP_HUD));

  /* ── row rows-1: left-aligned key hint ────────────────────────── */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/p:preset  t/T:theme  [/]:damp ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, float alpha) {
  erase();
  scene_draw(sc, alpha);
  hud_draw(s, sc, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — top-level container, lives in BSS as the single g_app instance.
 *
 * Intent
 *   The signal handlers (on_exit_signal, on_resize_signal) need to
 *   reach SOMETHING that the main loop polls.  Stashing it on the
 *   stack inside main() would force handlers to use globals anyway,
 *   so we just declare g_app globally and let handlers and main()
 *   share it.
 *
 * Why the volatile sig_atomic_t flags
 *   POSIX requires signal handlers touch only sig_atomic_t variables
 *   and only with simple writes — any wider type or operation is
 *   undefined.  `volatile` forces every read in the main loop to go
 *   back to memory (the compiler must not cache the value into a
 *   register across signal arrival).  Together they form the standard
 *   signal-driven "wake up the main loop" pattern: handler flips a
 *   bit, main loop notices, main loop does the work in user context.
 *
 * Why sim_fps is here, not in Scene
 *   sim_fps is a frame-loop concern (it picks the inner-loop tick
 *   period TICK_NS(sim_fps)) — it has no meaning inside scene_tick
 *   which receives the resulting dt as a parameter.  Putting it in
 *   App keeps Scene free of timing details.  RENDER_FPS stays fixed
 *   at 60; only the sim rate is App-tunable.  See [6] Fiedler for
 *   the fixed-sim / variable-render rationale.
 *
 * What is NOT here
 *   No frame-timing accumulators (sim_accum, fps_accum, frame_time);
 *   those are pure locals inside main().  Bringing them here would
 *   leak loop bookkeeping into App's public footprint.
 */
typedef struct {
  Scene scene;                       /* world + control state         */
  Screen screen;                     /* terminal extent               */
  int sim_fps;                       /* sim tick rate, 30..120 Hz     */
  volatile sig_atomic_t running;     /* SIGINT/TERM clears this       */
  volatile sig_atomic_t need_resize; /* SIGWINCH sets this            */
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
  int preset = app->scene.preset_idx;
  int theme = app->scene.theme_idx;
  scene_init(&app->scene, app->screen.cols, app->screen.rows, preset, theme);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Pendulum *p = &app->scene.pend;
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
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;

  /* n/p debounced ~200 ms so a held key doesn't rapid-cycle and
   * blur the bob through every preset's initial conditions.  Single
   * taps always pass.                                              */
  case 'n':
  case 'N': {
    static int64_t last_ns = 0;
    int64_t now = clock_ns();
    if (now - last_ns < 200 * NS_PER_MS)
      break;
    last_ns = now;
    sc->preset_idx = (sc->preset_idx + 1) % N_PRESETS;
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;
  }
  case 'p':
  case 'P': {
    static int64_t last_ns = 0;
    int64_t now = clock_ns();
    if (now - last_ns < 200 * NS_PER_MS)
      break;
    last_ns = now;
    sc->preset_idx = (sc->preset_idx + N_PRESETS - 1) % N_PRESETS;
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;
  }

  case 't':
    sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
    theme_apply(sc->theme_idx);
    break;
  case 'T':
    sc->theme_idx = (sc->theme_idx + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme_idx);
    break;

  case ']':
    p->damping -= DAMPING_STEP;
    if (p->damping < DAMPING_MIN)
      p->damping = DAMPING_MIN;
    break;

  case '[':
    p->damping += DAMPING_STEP;
    if (p->damping > DAMPING_MAX)
      p->damping = DAMPING_MAX;
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

  screen_init(&app->screen, THEME_DEFAULT);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, PRESET_DEFAULT,
             THEME_DEFAULT);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* ── resize ──────────────────────────────────────────────── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── dt ──────────────────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* ── sim accumulator ─────────────────────────────────────── */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;

    /* ── FPS counter ─────────────────────────────────────────── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* ── draw + present ──────────────────────────────────────── */
    screen_draw(&app->screen, &app->scene, fps_display, alpha);
    screen_present();

    /* ── input ───────────────────────────────────────────────── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
