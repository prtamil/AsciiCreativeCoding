/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/membrane.c — 2-D Vibrating Membrane Wave Equation Simulator
 *
 * Solves the damped scalar wave equation on the terminal grid:
 *
 *   ∂²u/∂t² = c² ∇²u − γ ∂u/∂t
 *
 * using an explicit 5-point finite-difference Laplacian and a
 * symplectic (velocity-form) time integrator.  The terminal itself
 * IS the simulation grid — one terminal cell = one grid point.
 *
 * Strike the membrane (b / e / f / m) and watch travelling waves
 * expand, hit the edges, reflect with or without phase inversion
 * depending on the boundary condition, interfere, and slowly fade as
 * damping converts the motion to "heat" the model does not represent.
 *
 * ═════════════════════════════════════════════════════════════════════
 *  WHAT YOU ARE SEEING ON SCREEN
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ┌──────────────────────────────────────────────────────────────────┐
 *  │ membrane  60.0 fps  sim:60 Hz  c=25  g=0.003  BC=Dirichlet  …    │ ← row 0
 * STATUS HUD │   . . . . . . . . . . . . . . . . . . . . . . │ (bright yellow +
 * bold) │   . .            ████████                . . .                   │ │
 * . .       ████████████████████         . .                     │ │   . .
 * ██████████████████████████      . .                     │ ← cells colored by
 * displacement u(x,y,t): │   . .    ████████  · · · · · · ░░░░      . . │     ⊕
 * warm  (red/orange/yellow/white) = crest (u > 0) │   . .    ▓▓░░░░  · · · · ·
 * · · · ░▒▒     . .                     │     ⊖  cool  (blue/cyan/violet) =
 * trough (u < 0) │   . .    ░░░░ · · · · · · · · · · ░░     . . │ brightness ∝
 * |u| / max_amplitude │   . .       · · · · · · · · · · · ·      . . │ │   . .
 * ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒            . .                     │ ← faint '·' on dim gray
 * = NODAL LINES │   . .             ▓▓▓▓▓▓▓                . . │ (zero-crossing
 * curves where u changes sign; │   . . . . . . . . . . . . . . . . . . . . . .
 * │      the geometry of the standing-wave mode) │ +--- MEMBRANE WAVE -------+
 * │ │ | max_amp   0.94          |                                      │ ←
 * bottom-left overlay panel: │ | energy    0.012         | │     numerical
 * readouts of the wave's state │ | mode     (1, 1)         | │     and the CFL
 * stability indicator │ | CFL_2D   0.589 STABLE   | │     (green=STABLE,
 * yellow=MARGINAL, red=UNSTABLE) │ | …                       | │ │ q:quit
 * spc:pause  s:step  r:reset  b:center  e:edge  m:mode  …  │ ← row n-1
 * ACTION-KEYS HUD
 *  └──────────────────────────────────────────────────────────────────┘ (bright
 * cyan + bold)
 *
 *   READING THE PATTERNS:
 *     • A pulse travels at c cells/s, so on an 80-wide grid with c=25
 *       it reaches the wall in (W/2)/c ≈ 1.6 s.  Count the reflections
 *       per second to verify c by eye.
 *     • DIRICHLET BC → reflected wave is INVERTED (phase flips at the
 *       wall) — what was a crest comes back as a trough.
 *     • NEUMANN BC   → reflected wave keeps its sign (free edge).
 *     • PERIODIC BC  → no reflection at all; wave exits one side and
 *       re-enters the opposite — the grid acts as a torus.
 *     • CENTER strike → symmetric ring of waves; perfect example of
 *       Huygens-like radial propagation.
 *     • EDGE / DOUBLE strikes → asymmetric superposition; interference
 *       crests where two ring fronts meet.
 *     • MODE press 'm' → directly initialises a standing-wave eigenmode
 *       (nx, ny); the cycle table picks visually distinct shapes —
 *       these are the discrete cousins of the analytical modes Rayleigh
 *       derived [ref 1] and that Chladni first visualised [ref 8] with
 *       sand-on-plate nodal patterns.
 *     • Watch the side panel: `energy` falls monotonically as damping
 *       acts; `max_amp` decays too but oscillates inside its envelope.
 *
 * ═════════════════════════════════════════════════════════════════════
 *  SECTION MAP
 * ═════════════════════════════════════════════════════════════════════
 *  §1  config      — all tunable constants
 *  §2  clock       — monotonic nanosecond clock + sleep
 *  §3  theme       — signed-amplitude color pipeline; ASCII ramp; LUT
 *  §4  grid        — static field arrays (height + velocity)
 *  §5  solver      — init_grid, update_wave, apply_bc, compute_stats
 *  §6  excitation  — apply_excitation, preset functions, mode cycle
 *  §7  render      — render_membrane, render_overlay
 *  §8  scene       — Scene struct, scene_init/tick/draw/reset
 *  §9  screen      — ncurses double-buffer display layer + two-row HUD
 *  §10 app         — signals, resize, input, main loop
 *
 * Keys:
 *   q / ESC      quit                 space     pause / resume
 *   s            single step          r         reset current preset
 *   b            centre strike        e         edge strike
 *   f            double strike        m         next resonance mode
 *   c / C        wave speed ±         d / D     damping ±
 *   n            cycle boundary       l         toggle nodal lines
 *   p / P        cycle preset         t         cycle theme
 *   [ / ]        sim Hz ±
 *
 * ═════════════════════════════════════════════════════════════════════
 *  REFERENCES   (cite inline as [n])
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ── Wave physics & vibrating membranes ─────────────────────────────
 *
 *   [1] Rayleigh, J. W. S. (1877) — *The Theory of Sound*, Vol. I,
 *       Macmillan, London.  Chapters IX-X derive the eigenmodes of
 *       vibrating membranes — the analytical counterpart to the
 *       (nx, ny) standing waves that the 'm' key excites here.
 *       The classical primary reference for everything this demo
 *       computes.
 *
 *   [2] Morse, P. M.; Ingard, K. U. (1968) — *Theoretical Acoustics*,
 *       McGraw-Hill (reprinted Princeton 1986).  Modern treatment of
 *       the wave equation in finite domains with the three boundary
 *       conditions used here (Dirichlet/Neumann/periodic).  Read this
 *       AFTER Rayleigh for a 20th-century perspective with the same
 *       analytical depth.
 *
 *   [3] Strauss, W. A. (2008) — *Partial Differential Equations: An
 *       Introduction*, 2nd ed., Wiley.  Ch.2-4 cover the 1-D and 2-D
 *       wave equation accessibly, including separation of variables,
 *       reflection at boundaries, and the d'Alembert formula.  The
 *       gentle starting point if Rayleigh is too dense.
 *
 *  ── Numerical methods: finite differences & stability ──────────────
 *
 *   [4] Courant, R.; Friedrichs, K.; Lewy, H. (1928) —
 *       "Über die partiellen Differenzengleichungen der mathematischen
 *       Physik", *Mathematische Annalen* 100, 32-74.  THE original
 *       paper deriving the stability condition c·dt/dx ≤ 1 (now
 *       universally called the CFL condition) for explicit
 *       finite-difference wave solvers.  The "CFL_2D" gauge in our
 *       overlay is exactly the quantity these three authors showed
 *       must stay ≤ 1.  Historical primary.
 *
 *   [5] LeVeque, R. J. (2007) — *Finite Difference Methods for Ordinary
 *       and Partial Differential Equations: Steady-State and
 *       Time-Dependent Problems*, SIAM.  The standard modern reference
 *       for the 5-point Laplacian stencil (§5 update_wave), its O(dx²)
 *       truncation error, and the von Neumann stability analysis that
 *       gives the √2 factor in the 2-D CFL bound.
 *
 *   [6] Strikwerda, J. C. (2004) — *Finite Difference Schemes and
 *       Partial Differential Equations*, 2nd ed., SIAM.  Chapter 9
 *       specifically analyses explicit wave-equation schemes and their
 *       dispersion relations — useful for understanding why a discrete
 *       wave on a finite grid moves slightly slower than the
 *       continuous c, especially at short wavelengths.
 *
 *   [7] Hairer, E.; Lubich, C.; Wanner, G. (2006) — *Geometric
 *       Numerical Integration*, 2nd ed., Springer.  Chapters I-VI
 *       cover SYMPLECTIC INTEGRATORS — why the "velocity-first" Euler
 *       in §5 conserves a shadow Hamiltonian over thousands of steps
 *       while plain explicit Euler injects spurious energy.
 *
 *  ── Visualisation & nodal-line history ─────────────────────────────
 *
 *   [8] Chladni, E. F. F. (1787) — *Entdeckungen über die Theorie des
 *       Klanges*, Weidmanns, Leipzig.  Chladni sprinkled sand on
 *       vibrating plates and observed it migrating to the NODAL LINES
 *       — the curves where displacement stays zero.  The same lines
 *       the 'l' key toggles here in the renderer.  Historical primary
 *       for nodal-pattern visualisation.
 *
 *   [9] Ware, C. (2020) — *Information Visualization: Perception for
 *       Design*, 4th ed., Morgan Kaufmann.  Ch.4 on DIVERGING colour
 *       maps backs the warm/cool split for ±u (perceptually symmetric
 *       around zero so the eye reads sign before magnitude); Ch.5 on
 *       pre-attentive luminance contrast backs the brightness-from-|u|
 *       ramp used inside each sign.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/membrane.c \
 *       -o membrane -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Wave equation (§5 update_wave) — Rayleigh [ref 1], Morse & Ingard [ref 2]:
 *   ∂²u/∂t² = c² ∇²u − γ ∂u/∂t
 *   u(x,y,t) — vertical displacement of the membrane at point (x,y)
 *   c         — wave speed [cells / second]; higher → faster propagation
 *   ∇²u       — 2-D Laplacian; measures how curved u is at each point
 *   γ         — damping coefficient [1/s]; bleeds energy out of the system
 *   Strauss [ref 3] gives the gentle textbook derivation.
 *
 * Finite difference stencil (§5) — LeVeque [ref 5]:
 *   The 5-point Laplacian approximates ∇²u at interior grid point (r,c):
 *
 *              u[r-1][c]
 *                  |
 *   u[r][c-1] — u[r][c] — u[r][c+1]
 *                  |
 *              u[r+1][c]
 *
 *   ∇²u[r,c] ≈ u[r-1,c] + u[r+1,c] + u[r,c-1] + u[r,c+1] − 4·u[r,c]
 *
 *   With grid spacing dx = dy = 1 cell, the /dx² factor is 1 and drops out.
 *   Truncation error is O(dx²) — second-order accurate in space.
 *   Strikwerda [ref 6] analyses the resulting numerical dispersion (waves
 *   shorter than ~8 cells travel slower than the continuous c).
 *
 * Velocity-field time integrator (§5) — Hairer/Lubich/Wanner [ref 7]:
 *   We maintain an explicit velocity field v = ∂u/∂t alongside u.
 *   At each timestep dt:
 *
 *     v[r,c] += ( c² · ∇²u[r,c]  −  γ · v[r,c] ) · dt   ← force on membrane
 *     u[r,c] += v[r,c] · dt                              ← displacement update
 *
 *   Updating v BEFORE u (symplectic Euler) conserves a SHADOW HAMILTONIAN
 *   over thousands of steps; plain explicit Euler would inject phantom
 *   energy and the wave would grow without bound.
 *
 * Wave speed meaning:
 *   c is the speed at which small disturbances travel across the grid.
 *   Physically: c = √(T/ρ) where T = membrane tension, ρ = area density.
 *   A pulse initiated at the centre reaches the wall in (W/2)/c seconds.
 *   Example: c=25, W=80 → first reflection arrives after 80/(2·25) = 1.6 s.
 *
 * CFL stability condition (§5, §7) — Courant, Friedrichs, Lewy [ref 4]:
 *   The explicit scheme is conditionally stable.  For 2-D with dx=dy=1:
 *     CFL_2D = c · dt · √2  must satisfy  CFL_2D ≤ 1
 *   If CFL_2D > 1, errors grow exponentially — the simulation "blows up".
 *   The √2 factor comes from von Neumann stability analysis of the 2-D
 *   5-point stencil [ref 5].  The overlay displays CFL_2D and flags it
 *   STABLE / MARGINAL / UNSTABLE — the exact quantity the 1928 CFL paper
 *   bounded.  Default: c=25, dt=1/60 → CFL_2D = 25·(1/60)·√2 ≈ 0.589  ✓
 *
 * Boundary conditions (§5 apply_bc) — Morse & Ingard [ref 2] Ch.5:
 *   DIRICHLET — u=0 at all 4 edges.  Models a clamped drumhead rim.
 *               Wave reflects with INVERSION (phase reversal).
 *               Supports the cleanest standing-wave eigenmodes [ref 1].
 *   NEUMANN   — ∂u/∂n=0 (zero normal gradient) at edges.
 *               Models a free membrane edge.
 *               Wave reflects WITHOUT inversion (no phase reversal).
 *               Implemented by copying adjacent interior values to the border.
 *   PERIODIC  — top wraps to bottom, left wraps to right.
 *               No reflections at all — the grid acts as a torus.
 *
 * Nodal lines (§7) — Chladni [ref 8], Rayleigh [ref 1]:
 *   In a standing wave, NODAL LINES are the curves where u = 0 at all
 *   times.  They separate regions oscillating in opposite phase (+ vs −)
 *   and form the geometric "fingerprint" of each eigenmode.  Historically,
 *   Chladni revealed them experimentally by sprinkling sand on vibrating
 *   plates — the sand migrated to the still nodes.  We detect them by
 *   checking sign changes between adjacent cells and mark them with a
 *   dim glyph so the mode shape is visible even when positive and
 *   negative regions have similar brightness.
 *
 * Signed colour pipeline (§3, §7) — Ware [ref 9]:
 *   The renderer uses a DIVERGING colour map: warm hues for u > 0, cool
 *   hues for u < 0, perceptually symmetric around zero so the eye reads
 *   sign before magnitude.  Within each sign, a 9-tier brightness ramp
 *   encodes |u|/max_amplitude (pre-attentive luminance contrast).
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

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 5,

  TARGET_FPS = 60,
  FPS_UPDATE_MS = 500,
  HUD_COLS = 80,
};

/* ── grid limits ────────────────────────────────────────────────────── */
/*
 * Physics runs directly in terminal-cell coordinates.  Unlike particle
 * simulations (which use a separate pixel space), the membrane grid IS
 * the character grid: g_grid.u[row][col] is drawn at terminal cell (col, row).
 *
 * GRID_MAX_W × GRID_MAX_H × 2 fields × 4 bytes = 300×100×8 = 240 KB (BSS).
 */
#define GRID_MAX_W 300
#define GRID_MAX_H 100

/* ── wave physics defaults ──────────────────────────────────────────── */
/*
 * WAVE_SPEED is in cells/second.  The CFL stability limit at 60 Hz is:
 *   c_max = 1 / (dt · √2) = 60 / √2 ≈ 42 cells/s.
 * Default 25 gives CFL_2D ≈ 0.59 — comfortable margin.
 */
#define WAVE_SPEED_DEFAULT 25.0f
#define WAVE_SPEED_MIN 5.0f
#define WAVE_SPEED_MAX 42.0f /* hard limit: CFL_2D → 1 at 60 Hz */
#define WAVE_SPEED_STEP 3.0f

#define DAMPING_DEFAULT 0.003f /* gentle decay; τ ≈ 1/γ ≈ 333 ticks */
#define DAMPING_MIN 0.000f
#define DAMPING_MAX 0.060f
#define DAMPING_STEP 0.003f

/* ── excitation ─────────────────────────────────────────────────────── */
#define EXCITE_AMP 1.2f    /* peak amplitude of a strike         */
#define EXCITE_RADIUS 3.5f /* Gaussian half-width in cells       */
#define RESONANCE_AMP 1.0f /* amplitude for mode-shape presets   */

/* ── rendering ──────────────────────────────────────────────────────── */
#define NODAL_SIGN_THRESH 0.015f /* |u| below this treated as near-zero */
#define DISPLAY_RANGE 1.5f       /* u values outside ±DISPLAY_RANGE clipped */
#define DISPLAY_MAX_FLOOR                                                      \
  0.05f /* min normaliser — keeps inv_max finite                             \
         * when the field is nearly silent      */

/* ── math constants used by the solver / stats ──────────────────────── *
 * Named so the formulas read like the textbook expressions they
 * implement, not like "what does this 0.7071 mean again".                */
#define HALF 0.5f /* ½ in KE = ½·m·v², PE = ½·c²|∇u|²    */
#define CDIFF_FACTOR                                                           \
  0.5f /* centred-difference: (f[i+1] - f[i-1])                                \
        * / (2·dx)  — with dx=1, divide by 2,                               \
        * i.e. multiply by 0.5 [LeVeque ref 5] */
#define SQRT2 1.41421356f
/* √2 — appears in CFL_2D = c·dt·√2
 * for the 2-D 5-point stencil [ref 4] */
#define INV_SQRT2 0.70710678f
/* 1/√2 — splits a unit amplitude into
 * (cos·A, sin·A) at the 45° phase
 * point of a harmonic oscillator      */

/* ── strike-position constants ──────────────────────────────────────── *
 * Each preset places its Gaussian pulse at a fraction of the grid
 * spanning (cols-1) × (rows-1).  Names describe WHERE on the membrane.  */
#define STRIKE_X_CENTRE 0.50f
#define STRIKE_Y_CENTRE 0.50f
#define STRIKE_X_LEFT_EDGE                                                     \
  0.15f                         /* 15% in from the left wall —               \
                                 * asymmetric enough to excite many            \
                                 * harmonics, not so close that the            \
                                 * Gaussian tail touches the boundary  */
#define STRIKE_X_DOUBLE_L 0.30f /* DOUBLE preset, left strike  */
#define STRIKE_Y_DOUBLE_L 0.35f
#define STRIKE_X_DOUBLE_R                                                      \
  0.70f /* DOUBLE preset, right strike —                                     \
         * 0.30 and 0.70 are symmetric about                                   \
         * the centre, but the y-offsets        */
#define STRIKE_Y_DOUBLE_R                                                      \
  0.65f /* (0.35, 0.65) make the line through                                  \
         * the two strikes NOT pass through                                    \
         * the centre — that produces a                                      \
         * richer non-symmetric interference                                   \
         * pattern.                              */

/* Canonical reference wave speed used inside preset_resonance() to set
 * an initial KINETIC amplitude for the velocity kick.  The actual
 * simulation `wave_speed` may differ — that's fine, the kick just
 * provides a tasteful starting velocity; the real c drives the
 * subsequent integration.                                                */
#define RESONANCE_KICK_REF_C WAVE_SPEED_DEFAULT

/* ── boundary conditions ────────────────────────────────────────────── */
enum {
  BC_DIRICHLET = 0, /* clamped rim: u=0 at edges                    */
  BC_NEUMANN = 1,   /* free edge:   du/dn=0 at edges                */
  BC_PERIODIC = 2,  /* torus:       top↔bottom, left↔right          */
  BC_COUNT = 3,
};
static const char *const k_bc_names[BC_COUNT] = {"dirichlet", "neumann",
                                                 "periodic"};

/* ── presets ────────────────────────────────────────────────────────── */
enum {
  PRESET_CENTER = 0,
  PRESET_EDGE = 1,
  PRESET_DOUBLE = 2,
  PRESET_RESONANCE = 3,
  PRESET_COUNT = 4,
};
static const char *const k_preset_names[PRESET_COUNT] = {"center", "edge",
                                                         "double", "resonance"};

/* ── resonance-mode cycle table ─────────────────────────────────────── *
 * Successive presses of 'm' cycle through these (nx, ny) standing-wave
 * modes.  The (1,1) fundamental is included LAST because it oscillates
 * slowest and has the least spatial structure — starting the cycle
 * with (2,2) gives a more visually obvious mode on the first press.  */
static const int k_mode_table[][2] = {
    {2, 2}, {3, 2}, {2, 3}, {3, 3}, {4, 3},
    {3, 4}, {4, 4}, {1, 2}, {2, 1}, {1, 1},
};
#define MODE_TABLE_LEN ((int)(sizeof k_mode_table / sizeof k_mode_table[0]))

/* ── timing ─────────────────────────────────────────────────────────── */
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
  struct timespec r = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme — signed-amplitude color pipeline; ASCII ramp; LUT          */
/* ===================================================================== */

/*
 * ASCII ramp — characters ordered by visual ink density (sparse → dense).
 * Used identically for positive and negative amplitude; color encodes sign.
 *
 *   ' '   0%   at rest / nodal
 *   '.'   3%   barely displaced
 *   ':'  10%   slight ripple
 *   '+'  22%   medium wave
 *   'x'  38%   strong wave
 *   '*'  55%   intense oscillation
 *   'X'  72%   near-peak
 *   '#'  87%   peak displacement
 *   '@'  96%   maximum (clipped)
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 9 levels */

/* LUT breakpoints on normalised |amplitude| ∈ [0, 1] after gamma correction */
static const float k_breaks[RAMP_N] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f, 0.500f, 0.660f, 0.820f, 0.940f,
};

static int lut_index(float v) {
  /* v is already in [0,1]; gamma correct for perceptual uniformity */
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return RAMP_N - 1;
  float g = powf(v, 1.0f / 2.2f);
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (g >= k_breaks[i])
      return i;
  return 0;
}

/*
 * Themes — 3 palettes for signed amplitude.
 *
 * Each palette has RAMP_N fg colors for POSITIVE amplitude (crest)
 * and RAMP_N fg colors for NEGATIVE amplitude (trough).
 *
 *   theme 0  "wave"    — blue troughs  ↔  red/orange crests (classical)
 *   theme 1  "thermal" — violet troughs ↔ yellow/white crests
 *   theme 2  "ocean"   — deep-blue troughs ↔ cyan/white crests
 *
 * Color pair layout (all themes defined at startup, switched by index):
 *   theme t, positive level i : CP_POS(t,i) = 1 + t*(RAMP_N*2) + i
 *   theme t, negative level i : CP_NEG(t,i) = 1 + t*(RAMP_N*2) + RAMP_N + i
 *   nodal line marker         : CP_NODAL = 1 + N_THEMES*(RAMP_N*2)
 *   HUD STATUS  (row 0)       : CP_HUD   = CP_NODAL + 1   (bright yellow)
 *   HUD HINT    (row n-1)     : CP_HINT  = CP_NODAL + 2   (bright cyan)
 *
 * CP_HUD / CP_HINT are CANONICAL per CLAUDE.md HUD Standard — fixed
 * bright yellow / bright cyan, A_BOLD at the call site, NEVER A_DIM
 * so they stay legible against any wave palette.
 *
 * With 3 themes × 9 levels × 2 signs = 54 pairs + 3 = 57 pairs total.
 */
#define N_THEMES 3
#define CP_POS(t, i) (1 + (t) * (RAMP_N * 2) + (i))
#define CP_NEG(t, i) (1 + (t) * (RAMP_N * 2) + RAMP_N + (i))
#define CP_NODAL (1 + N_THEMES * (RAMP_N * 2))
#define CP_HUD (CP_NODAL + 1)
#define CP_HINT (CP_NODAL + 2)

/*
 * WaveTheme — one named DIVERGING colour map (Ware [ref 9], Ch.4).
 *
 * The wave equation produces a signed scalar field u(x,y,t) — crests
 * (u > 0) and troughs (u < 0) are physically opposite quantities that
 * must read as visually opposite.  Diverging maps split into two
 * perceptually monotone luminance ramps that meet at a NEUTRAL midpoint
 * (background black, here): a WARM ramp for crests, a COOL ramp for
 * troughs.  The eye reads SIGN from hue and MAGNITUDE from luminance —
 * pre-attentive contrast (Ware [ref 9], Ch.5).
 *
 * WHY a struct (not loose const arrays):
 *   The crest ramp and trough ramp must be CORRELATED — same number of
 *   tiers (RAMP_N=9), same brightness progression from dark to bright,
 *   so |u|=0.3·max in red corresponds to the same eye-perceived
 *   intensity as |u|=0.3·max in blue.  Bundling the four arrays in one
 *   struct forbids editing one ramp out of sync with its partner.
 *
 * WHY dual palettes (256 + 8):
 *   Modern terminals expose 256 indexed colours so each tier can have
 *   its own luminance step; legacy / minimal TTYs ($TERM = "linux",
 *   "dumb") expose only 8, where the LO/HI tiers necessarily collapse
 *   onto fewer distinct colours.  color_init() inspects ncurses COLORS
 *   at runtime and binds the matching set.  Sign discrimination is
 *   preserved in both cases — that's the one property that MUST
 *   survive even on 8-colour terminals.
 *
 * WHY ramps go DARK → BRIGHT (not dark → bright → dark):
 *   "Sequential within each sign" is what makes |u| readable.  Using a
 *   non-monotone ramp (rainbow / jet) would scramble magnitude
 *   ordering and force a viewer to memorise the legend.  Both ramps
 *   end at COLOR 231 (near-white) so the brightest crest and brightest
 *   trough are equally salient — visual "fairness" between signs.
 *
 * WHY this struct does NOT include CP_HUD / CP_HINT / CP_NODAL:
 *   Those are CANONICAL pairs (CLAUDE.md HUD Standard for CP_HUD/HINT
 *   in bright yellow/cyan; CP_NODAL is dim grey across every theme so
 *   the geometry of standing-wave modes reads the same regardless of
 *   theme).  Keeping them outside means theme cycling doesn't disturb
 *   the HUD or the nodal-line overlay.
 */
typedef struct {
  const char *name; /* shown in HUD theme field; theme cycle label   */

  /* 256-colour palette (used when COLORS >= 256) ─────────────────── *
   * Index i runs 0..RAMP_N-1 from FAINTEST to BRIGHTEST tier.  Ramp
   * i = lut_index(|u|/max) selects which entry to use; A_BOLD is
   * added by wave_attr() for the top two tiers to amplify the
   * brightest cells against any background. */
  int pos256[RAMP_N]; /* CREST  ramp — u > 0, "warm" diverging side    */
  int neg256[RAMP_N]; /* TROUGH ramp — u < 0, "cool" diverging side    */

  /* 8-colour ANSI fallback ────────────────────────────────────── *
   * Coarser bins; typically several adjacent tiers share a colour
   * with only the highest one or two ramping toward WHITE.  Sign
   * discrimination is preserved — magnitude discrimination relies
   * more on cell density than colour at this resolution. */
  int pos8[RAMP_N];
  int neg8[RAMP_N];
} WaveTheme;

static const WaveTheme k_themes[N_THEMES] = {
    {
        /* 0  wave — blue troughs, red/yellow crests */
        "wave",
        /* pos: dark red → orange → yellow → white */
        {52, 88, 124, 160, 196, 202, 208, 220, 231},
        /* neg: dark blue → blue → cyan → white     */
        {17, 19, 21, 27, 33, 39, 45, 51, 231},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
         COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
    },
    {
        /* 1  thermal — violet troughs, yellow crests */
        "thermal",
        /* pos: dark orange → amber → yellow → white */
        {52, 94, 130, 166, 202, 208, 214, 220, 231},
        /* neg: dark violet → magenta → pink         */
        {53, 54, 91, 92, 129, 165, 201, 207, 231},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
         COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
    },
    {
        /* 2  ocean — deep-blue troughs, cyan/white crests */
        "ocean",
        /* pos: teal → cyan → white */
        {23, 29, 36, 43, 51, 87, 123, 159, 231},
        /* neg: navy → midnight blue → indigo        */
        {17, 18, 19, 20, 21, 27, 33, 39, 45},
        {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
         COLOR_BLUE, COLOR_BLUE, COLOR_CYAN},
    },
};

static void color_init(void) {
  start_color();
  use_default_colors();
  for (int t = 0; t < N_THEMES; t++) {
    for (int i = 0; i < RAMP_N; i++) {
      if (COLORS >= 256) {
        init_pair(CP_POS(t, i), k_themes[t].pos256[i], COLOR_BLACK);
        init_pair(CP_NEG(t, i), k_themes[t].neg256[i], COLOR_BLACK);
      } else {
        init_pair(CP_POS(t, i), k_themes[t].pos8[i], COLOR_BLACK);
        init_pair(CP_NEG(t, i), k_themes[t].neg8[i], COLOR_BLACK);
      }
    }
  }
  /* Nodal line: dim gray (or dim yellow fallback) */
  if (COLORS >= 256)
    init_pair(CP_NODAL, 240, COLOR_BLACK); /* dark gray */
  else
    init_pair(CP_NODAL, COLOR_WHITE, COLOR_BLACK);
  /* HUD STATUS row 0 — canonical bright yellow + A_BOLD at call site */
  if (COLORS >= 256)
    init_pair(CP_HUD, 226, -1);
  else
    init_pair(CP_HUD, COLOR_YELLOW, -1);

  /* HUD HINT row n-1 — canonical bright cyan + A_BOLD at call site */
  if (COLORS >= 256)
    init_pair(CP_HINT, 51, -1);
  else
    init_pair(CP_HINT, COLOR_CYAN, -1);
}

/* Return ncurses attribute for (theme, sign, ramp_level). */
static attr_t wave_attr(int theme, bool positive, int level) {
  attr_t a = positive ? COLOR_PAIR(CP_POS(theme, level))
                      : COLOR_PAIR(CP_NEG(theme, level));
  if (level >= RAMP_N - 2)
    a |= A_BOLD;
  return a;
}

/* ===================================================================== */
/* §4  grid — Grid struct (fields + live dimensions)                      */
/* ===================================================================== */

/*
 * Grid — the membrane's PHYSICAL STATE: two scalar fields plus the
 *        live dimensions that bound them.
 *
 * ALGORITHM context — Rayleigh [ref 1], Morse & Ingard [ref 2]:
 *   The continuous wave equation ∂²u/∂t² = c²∇²u − γ∂u/∂t is
 *   second-order in time.  We turn it into a FIRST-ORDER pair by
 *   introducing the velocity field v = ∂u/∂t:
 *
 *       ∂v/∂t = c²∇²u − γv         (force on the membrane)
 *       ∂u/∂t = v                  (kinematic definition)
 *
 *   The solver discretises BOTH equations on the same grid.  That's
 *   why TWO fields are needed — without v we'd need a "previous
 *   timestep" buffer to estimate ∂²u/∂t² by finite differences (the
 *   "leapfrog" alternative).  The velocity-field form is cleaner and
 *   plays nicely with symplectic Euler [ref 7] for energy conservation.
 *
 * WHY one struct (not two separate arrays + scattered dims):
 *   • Cohesion — u, v, cols, rows are ALWAYS used together.  Pulling
 *     them into one type makes the unit of physical state explicit.
 *   • Self-describing — `g->cols` is INSIDE the struct that owns the
 *     fields, so a function operating on a Grid can't accidentally
 *     pair u/v with stale dimensions.
 *   • Atomic resize — re-fitting the grid for a new terminal size is
 *     a single struct update; no risk of "I updated rows but forgot
 *     to invalidate the field contents".
 *   • Cleaner function signatures — every function that touches the
 *     grid now takes `Grid *g` (or `const Grid *g`) and reads dims
 *     via `g->cols / g->rows`, dropping the `(cols, rows)` param pair
 *     that previously rode along with every call.
 *
 * WHY 2-D static arrays inside the struct (not pointer-to-pointer):
 *   • BSS-resident — the OS zero-fills the pages at exec time, so the
 *     initial state is already u = v = 0 (membrane at rest) without
 *     any code running.  No init phase, no malloc on the hot path
 *     (CLAUDE.md Memory rule).
 *   • Static dimensions GRID_MAX_W × GRID_MAX_H × 4 bytes × 2 fields
 *     ≈ 240 KB per Grid — fits comfortably in BSS.
 *   • Indexing g->u[row][col] reads naturally as "row r, column c"
 *     matching the way the renderer steps over the grid.
 *
 * WHY ROW-MAJOR (u[row][col], not u[col][row]):
 *   The renderer and the BC loops iterate column-within-row (the
 *   outer loop is rows, inner is cols).  Row-major layout keeps each
 *   inner-loop scan in a single cache line — the 5-point Laplacian's
 *   u[r±1, c] reads cost one cache line each, but the u[r, c±1] reads
 *   come for free from the cache line the iterator is already
 *   streaming.
 *
 * WHY cols/rows live HERE (not in Scene):
 *   They are the dimensions OF THIS GRID — every solver loop bounds
 *   itself with them, and apply_bc indexes the last row/col via them.
 *   Keeping them next to u/v guarantees the three values can never
 *   diverge.  Scene used to carry duplicate copies; those are gone.
 *
 * Interior / boundary split:
 *   • Interior cells: rows 1..rows-2, cols 1..cols-2 — written by the
 *                     wave solver each tick.
 *   • Boundary cells: row 0, row rows-1, col 0, col cols-1 — written
 *                     by apply_bc() per the active BC policy
 *                     (Dirichlet zero / Neumann mirror / periodic wrap).
 *   The split is enforced by the loop bounds in update_wave (1..N-1).
 *
 * WHY bc is NOT in Grid:
 *   The boundary CONDITION is a user-tunable policy (toggled with 'n')
 *   and belongs in Scene's SIMULATION PARAMS region.  The boundary
 *   CELLS belong to the grid; the policy choice does not.
 */
typedef struct {
  float u[GRID_MAX_H][GRID_MAX_W]; /* u(x,y,t) — DISPLACEMENT field
                                    * [cells].  Sign convention: > 0
                                    * above the rest plane (crest),
                                    * < 0 below (trough).  Read by
                                    * every render pass; written by
                                    * update_wave + apply_bc +
                                    * apply_excitation + presets.      */
  float v[GRID_MAX_H][GRID_MAX_W]; /* v(x,y,t) — VELOCITY field ∂u/∂t.
                                    * Updated FIRST each tick (the
                                    * symplectic-Euler half-step);
                                    * sign indicates upward (>0) vs
                                    * downward (<0) motion at that
                                    * instant.                          */
  int cols;                        /* live width  (≤ GRID_MAX_W) —
                                    * matches the terminal column
                                    * count; the renderer indexes one
                                    * cell per column.                  */
  int rows;                        /* live height (≤ GRID_MAX_H) —
                                    * matches the terminal row count.   */
} Grid;

/* THE single grid instance — file-scope BSS, no malloc.  Every
 * physics/render helper below takes `Grid *g` (or `const Grid *g`)
 * and accesses fields via `g->u / g->v / g->cols / g->rows`.  Scene
 * carries a pointer to this object so higher-level code can pass
 * `&scene.grid` interchangeably.                                       */
static Grid g_grid;

/* ===================================================================== */
/* §5  solver                                                             */
/* ===================================================================== */

/*
 * init_grid() — set live dims, zero both fields, apply initial BC.
 *
 * Called at startup and on resize.  No heap allocation.
 */
static void init_grid(int bc, int cols, int rows); /* forward decl */

/* Zero u and v across the live grid (cells outside cols/rows are
 * irrelevant — the solver loops never touch them). */
static void grid_zero(void) {
  for (int r = 0; r < g_grid.rows; r++) {
    memset(g_grid.u[r], 0, (size_t)g_grid.cols * sizeof(float));
    memset(g_grid.v[r], 0, (size_t)g_grid.cols * sizeof(float));
  }
}

/* DIRICHLET BC — clamped rim.  u = v = 0 at all 4 borders.  Physical
 * analogue: a drumhead glued to a rigid frame; the rim cannot move,
 * so any wave reaching it is reflected back with a 180° phase flip
 * (crest → trough).  Morse & Ingard [ref 2] §5.2. */
static void apply_dirichlet_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                               int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = 0.0f;
    v[0][c] = 0.0f;
    u[rows - 1][c] = 0.0f;
    v[rows - 1][c] = 0.0f;
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = 0.0f;
    v[r][0] = 0.0f;
    u[r][cols - 1] = 0.0f;
    v[r][cols - 1] = 0.0f;
  }
}

/* NEUMANN BC — free edge.  ∂u/∂n = 0 at the rim, i.e. the FIRST
 * derivative normal to the boundary vanishes.  Implemented by copying
 * each border cell from its one interior neighbour so the discrete
 * gradient across the boundary is zero.  Waves reflect WITHOUT phase
 * inversion (crest reflects as crest).  Morse & Ingard [ref 2] §5.3. */
static void apply_neumann_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                             int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = u[1][c];
    v[0][c] = v[1][c];
    u[rows - 1][c] = u[rows - 2][c];
    v[rows - 1][c] = v[rows - 2][c];
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = u[r][1];
    v[r][0] = v[r][1];
    u[r][cols - 1] = u[r][cols - 2];
    v[r][cols - 1] = v[r][cols - 2];
  }
}

/* PERIODIC BC — toroidal wrap.  No reflections at all: a wave leaving
 * the right edge re-enters at the left, top↔bottom likewise.  We use
 * `rows-2` / `cols-2` (not `rows-1` / `cols-1`) so the wrap maps each
 * boundary cell to its OPPOSITE INTERIOR neighbour — that way the
 * 5-point Laplacian sees a consistent neighbourhood from both sides
 * of the seam.  Strauss [ref 3] Ch.4 covers the torus geometry. */
static void apply_periodic_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                              int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = u[rows - 2][c];
    v[0][c] = v[rows - 2][c];
    u[rows - 1][c] = u[1][c];
    v[rows - 1][c] = v[1][c];
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = u[r][cols - 2];
    v[r][0] = v[r][cols - 2];
    u[r][cols - 1] = u[r][1];
    v[r][cols - 1] = v[r][1];
  }
}

/* Dispatch to the chosen boundary policy.  Reads as pure selection;
 * each named helper holds one self-contained BC implementation. */
static void apply_bc(int bc) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  switch (bc) {
  case BC_DIRICHLET:
    apply_dirichlet_bc(u, v, cols, rows);
    break;
  case BC_NEUMANN:
    apply_neumann_bc(u, v, cols, rows);
    break;
  case BC_PERIODIC:
    apply_periodic_bc(u, v, cols, rows);
    break;
  }
}

/* 5-POINT DISCRETE LAPLACIAN [LeVeque ref 5] —
 *      ∇²u[r,c] ≈ u[r-1,c] + u[r+1,c] + u[r,c-1] + u[r,c+1] − 4·u[r,c]
 *
 * Geometric reading: L is the SUM of differences to each of the four
 * Manhattan neighbours.  If all four neighbours match u[r,c], L=0
 * and no restoring force acts (the cell is at a local extremum); if
 * neighbours pull the cell toward their average, L has the sign of
 * that average and the wave equation pushes u back toward equilibrium.
 *
 * `static inline` so the compiler can fold this into the hot velocity
 * loop without function-call overhead.                                  */
static inline float laplacian5(const float (*u)[GRID_MAX_W], int r, int c) {
  return u[r - 1][c] + u[r + 1][c] + u[r][c - 1] + u[r][c + 1] - 4.0f * u[r][c];
}

/* SYMPLECTIC HALF-STEP 1 — velocity update on every INTERIOR cell.
 * Reads u, writes v.  Force breakdown:
 *     restoring   = c² · ∇²u    (wave equation, always present)
 *     drag        = −γ · v      (damping, drains energy gradually)
 *     v_new = v + (restoring + drag) · dt
 * Doing the velocity update FIRST (using the current u) is what makes
 * this scheme SYMPLECTIC [Hairer/Lubich/Wanner ref 7] — it conserves a
 * shadow Hamiltonian instead of leaking energy like explicit Euler.    */
static void velocity_half_step(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                               int cols, int rows, float c2, float damping,
                               float dt) {
  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      float lap = laplacian5((const float(*)[GRID_MAX_W])u, r, c);
      v[r][c] += (c2 * lap - damping * v[r][c]) * dt;
    }
  }
}

/* SYMPLECTIC HALF-STEP 2 — displacement update using the NEW velocity.
 *     u_new = u + v_new · dt
 * Must be a SEPARATE loop from STEP 1 — if it were merged, updating
 * u[r,c] would corrupt the Laplacian computation for u[r+1,c] still to
 * come in step 1.  The price is one extra pass over the interior; the
 * benefit is correctness.                                              */
static void displacement_half_step(float (*u)[GRID_MAX_W],
                                   const float (*v)[GRID_MAX_W], int cols,
                                   int rows, float dt) {
  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      u[r][c] += v[r][c] * dt;
    }
  }
}

/*
 * update_wave — advance the damped wave equation by one fixed dt.
 *
 * Pseudocode (symplectic Euler on the first-order pair):
 *     step 1   v ← v + (c²∇²u − γv) · dt        (velocity half-step)
 *     step 2   u ← u + v · dt                    (displacement half-step)
 *     step 3   apply_bc                          (consistent boundary)
 */
static void update_wave(float dt, float wave_speed, float damping, int bc) {
  float c2 = wave_speed * wave_speed;
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  velocity_half_step(u, v, cols, rows, c2, damping, dt);
  displacement_half_step(u, (const float(*)[GRID_MAX_W])v, cols, rows, dt);
  apply_bc(bc);
}

/* PEAK + ENERGY SCAN — one pass over the interior collects:
 *   • mx   = peak |u|        (display normaliser)
 *   • ke   = Σ ½·v²          (kinetic energy density × area; m = 1)
 *   • pe   = Σ ½·c²·|∇u|²    (elastic potential energy density × area)
 *
 * The gradient is estimated by centred differences [LeVeque ref 5]:
 *   ∂u/∂x ≈ (u[c+1] − u[c-1]) · CDIFF_FACTOR     (factor = 1/(2·dx) = ½)
 *
 * One pass instead of three keeps the iteration friendly to the
 * prefetcher — each cache line of u/v gets read once per frame. */
static void scan_peak_and_energy(const float (*u)[GRID_MAX_W],
                                 const float (*v)[GRID_MAX_W], int cols,
                                 int rows, float c2, float *out_max_amp,
                                 double *out_ke, double *out_pe) {
  float mx = 0.0f;
  double ke = 0.0, pe = 0.0;

  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      float uu = u[r][c];
      float vv = v[r][c];
      float a = fabsf(uu);
      if (a > mx)
        mx = a;

      /* KE density:  ½·m·v²  with m = 1 per cell */
      ke += (double)HALF * (double)(vv * vv);

      /* PE density:  ½·c²·|∇u|²   (centred differences) */
      float gx = (u[r][c + 1] - u[r][c - 1]) * CDIFF_FACTOR;
      float gy = (u[r + 1][c] - u[r - 1][c]) * CDIFF_FACTOR;
      pe += (double)HALF * (double)c2 * (double)(gx * gx + gy * gy);
    }
  }
  *out_max_amp = mx;
  *out_ke = ke;
  *out_pe = pe;
}

/* MODE ESTIMATION — count sign changes (zero crossings) along the
 * centre row and centre column.  Rayleigh [ref 1]: for a pure
 * (nx, ny) standing wave, the centre row holds exactly nx half-waves,
 * each contributing one zero crossing — so crossing_count ≈ nx.
 * A pulse-superposition state has many simultaneous modes and the
 * count is "the dominant nx that fits".  Bounded below at 1 so the
 * HUD never reads (0, 0) when the field is at rest. */
static void estimate_mode_numbers(const float (*u)[GRID_MAX_W], int cols,
                                  int rows, int *out_nx, int *out_ny) {
  int cr = rows / 2;
  int cc = cols / 2;
  int crossings_x = 0;
  int crossings_y = 0;

  for (int c = 1; c < cols - 1; c++)
    if (u[cr][c - 1] * u[cr][c] < 0.0f)
      crossings_x++;
  for (int r = 1; r < rows - 1; r++)
    if (u[r - 1][cc] * u[r][cc] < 0.0f)
      crossings_y++;

  *out_nx = (crossings_x < 1) ? 1 : crossings_x;
  *out_ny = (crossings_y < 1) ? 1 : crossings_y;
}

/* CFL_2D — Courant-Friedrichs-Lewy stability gauge for the 2-D
 * explicit 5-point scheme [CFL ref 4, LeVeque ref 5].  Bound:
 *     c · dt · √2  ≤  1     (with dx = dy = 1)
 * The √2 = SQRT2 falls out of von Neumann stability analysis on the
 * 5-point Laplacian.  Render_overlay traffic-lights this value:
 *   < 0.70 green (STABLE),  < 0.90 yellow (MARGINAL),  else red.   */
static float cfl_2d_stability(float wave_speed, float dt_sec) {
  return wave_speed * dt_sec * SQRT2;
}

/*
 * compute_stats — derive every per-tick overlay quantity from u and v.
 *
 * Pseudocode:
 *     one pass over interior  → peak |u|, KE, PE
 *     centre-line zero-cross  → (mode_nx, mode_ny)
 *     wave_speed · dt · √2    → CFL_2D gauge
 *     normalise (KE+PE) by interior cell count → per-cell energy
 */
static void compute_stats(float wave_speed, float dt_sec, float *max_amplitude,
                          float *energy_est, int *mode_nx, int *mode_ny,
                          float *cfl_2d) {
  const float(*ufld)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.u;
  const float(*vfld)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;
  float c2 = wave_speed * wave_speed;

  double ke, pe;
  scan_peak_and_energy(ufld, vfld, cols, rows, c2, max_amplitude, &ke, &pe);
  estimate_mode_numbers(ufld, cols, rows, mode_nx, mode_ny);

  *energy_est = (float)((ke + pe) / ((rows - 2) * (cols - 2)));
  *cfl_2d = cfl_2d_stability(wave_speed, dt_sec);
}

/* Set live grid dims, then zero u/v and stamp the boundary policy. */
static void init_grid(int bc, int cols, int rows) {
  g_grid.cols = cols;
  g_grid.rows = rows;
  grid_zero();
  apply_bc(bc);
}

/* ===================================================================== */
/* §6  excitation                                                          */
/* ===================================================================== */

/*
 * apply_excitation() — add a Gaussian displacement pulse to g_grid.u.
 *
 * The pulse shape is:
 *   Δu(x,y) = amp · exp( −[(x−cx)² + (y−cy)²] / (2·r²) )
 *
 * We ADD to the current field rather than replacing it so that multiple
 * strikes accumulate (useful for the double-strike preset).
 *
 * Only the displacement field is perturbed; g_grid.v is left unchanged.
 * This models an impulsive displacement (a drumstick hit) rather than
 * an impulse of momentum.  To model a momentum impulse, add to g_grid.v
 * instead.
 */
static void apply_excitation(float cx, float cy, float amp, float radius) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  int cols = g_grid.cols;
  int rows = g_grid.rows;
  float inv_2r2 = 1.0f / (2.0f * radius * radius);
  for (int r = 0; r < rows; r++) {
    float dy = (float)r - cy;
    float dy2 = dy * dy;
    for (int c = 0; c < cols; c++) {
      float dx = (float)c - cx;
      float d2 = dx * dx + dy2;
      u[r][c] += amp * expf(-d2 * inv_2r2);
    }
  }
}

/* WAVENUMBERS for the (nx, ny) Dirichlet eigenmode on a (cols × rows)
 * grid.  An n-half-wave standing pattern across a span L has spatial
 * frequency k = nπ/L; we use the discrete length L = (cols-1) for the
 * x-axis (cells 0..cols-1 inclusive) and similarly for y. */
static void mode_wavenumbers(int nx, int ny, int cols, int rows, float *out_kx,
                             float *out_ky) {
  *out_kx = (float)nx * (float)M_PI / (float)(cols - 1);
  *out_ky = (float)ny * (float)M_PI / (float)(rows - 1);
}

/* EIGEN-FREQUENCY of a 2-D Dirichlet (nx, ny) mode at wave speed c
 * [Rayleigh ref 1, Crawford ref 6]:
 *     ω_mn = c · π · √(nx²/W² + ny²/H²)
 *          = c · √(kx² + ky²)         (since k_i = n_i·π/L_i)
 * Used here only to scale the velocity kick — see preset_resonance. */
static float mode_eigen_frequency(float c, float kx, float ky) {
  return c * sqrtf(kx * kx + ky * ky);
}

/* STAMP the (nx, ny) standing-wave EIGENMODE onto u and v.
 *
 * Spatial shape: ψ(x,y) = sin(kx·x) · sin(ky·y)  (Dirichlet, Rayleigh [1]).
 *
 * Temporal phase:  We don't start from rest (u = amp·ψ, v = 0); we
 * start at the 45° phase point of the harmonic oscillation:
 *     u(t=0) = (amp · cos 45°) · ψ = amp/√2 · ψ
 *     v(t=0) = (amp · ω · sin 45°) · ψ = amp·ω/√2 · ψ
 * INV_SQRT2 = cos(π/4) = sin(π/4) makes this a single multiplier per
 * field.  Result: the FIRST rendered frame already shows non-zero
 * motion — the eye sees the mode animating immediately instead of
 * staring at a frozen extremum waiting for the half-period to pass. */
static void stamp_standing_wave_state(float (*u)[GRID_MAX_W],
                                      float (*v)[GRID_MAX_W], int cols,
                                      int rows, float kx, float ky, float amp,
                                      float omega) {
  float u_scale = INV_SQRT2 * amp;
  float v_scale = INV_SQRT2 * amp * omega;

  for (int r = 0; r < rows; r++) {
    float sy = sinf(ky * (float)r);
    for (int c = 0; c < cols; c++) {
      float sx = sinf(kx * (float)c);
      float psi = sx * sy;
      u[r][c] = u_scale * psi;
      v[r][c] = v_scale * psi;
    }
  }
}

/*
 * preset_resonance — initialise the field to a single eigenmode of
 *                    the wave equation, mid-oscillation.
 *
 * Pseudocode:
 *     k_x, k_y ← wavenumbers for mode (nx, ny)
 *     ω       ← eigen-frequency at the reference wave speed
 *     stamp standing-wave state with 45°-phase scaling
 */
static void preset_resonance(int nx, int ny, float amp) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  float kx, ky;
  mode_wavenumbers(nx, ny, cols, rows, &kx, &ky);

  /* Use the CANONICAL reference wave speed (not the current
   * scene.wave_speed) — this only sets a tasteful starting kinetic
   * amplitude; the real c still drives the subsequent integration,
   * so the visible frequency follows whatever the user has set. */
  float omega = mode_eigen_frequency(RESONANCE_KICK_REF_C, kx, ky);

  stamp_standing_wave_state(u, v, cols, rows, kx, ky, amp, omega);
}

/* ── preset application helpers ─────────────────────────────────────── */

typedef struct Scene Scene; /* forward declaration for preset callbacks */

static void preset_apply(Scene *s, int preset_id);

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* NODAL-CELL TEST — is cell (r,c) ON a Chladni nodal line?
 *
 * Two conditions BOTH have to hold [Chladni ref 8, Rayleigh ref 1]:
 *   1. |u| at this cell is below `near_zero_thresh` (it's a candidate
 *      zero of the field).
 *   2. At least one Manhattan neighbour has u > +near_zero_thresh AND
 *      at least one has u < −near_zero_thresh — i.e. the cell sits
 *      BETWEEN regions of opposite phase, which is the geometric
 *      definition of a nodal curve.
 *
 * Without condition (2), every "calm" cell would be flagged — but
 * those are just background, not nodes.  The two-side test is what
 * makes the dim '·' overlay trace the actual mode-shape geometry. */
static bool cell_is_nodal_line(const float (*u)[GRID_MAX_W], int r, int c,
                               int cols, int rows, float near_zero_thresh) {
  if (fabsf(u[r][c]) >= near_zero_thresh)
    return false;

  bool near_pos = false, near_neg = false;
  if (r > 0) {
    near_pos |= (u[r - 1][c] > near_zero_thresh);
    near_neg |= (u[r - 1][c] < -near_zero_thresh);
  }
  if (r < rows - 1) {
    near_pos |= (u[r + 1][c] > near_zero_thresh);
    near_neg |= (u[r + 1][c] < -near_zero_thresh);
  }
  if (c > 0) {
    near_pos |= (u[r][c - 1] > near_zero_thresh);
    near_neg |= (u[r][c - 1] < -near_zero_thresh);
  }
  if (c < cols - 1) {
    near_pos |= (u[r][c + 1] > near_zero_thresh);
    near_neg |= (u[r][c + 1] < -near_zero_thresh);
  }
  return near_pos && near_neg;
}

/* Paint one nodal-line marker — dim '·' in the neutral CP_NODAL pair.
 * Stays the same colour across every theme so the geometric overlay
 * doesn't compete with the diverging amplitude colours. */
static void paint_nodal_cell(WINDOW *w, int r, int c) {
  wattron(w, COLOR_PAIR(CP_NODAL) | A_DIM);
  mvwaddch(w, r, c, '.');
  wattroff(w, COLOR_PAIR(CP_NODAL) | A_DIM);
}

/* Paint one AMPLITUDE cell using the diverging warm/cool ramp.
 *
 * Steps (Ware [ref 9]):
 *   1. Sign of u  → choose WARM (positive) or COOL (negative) family
 *   2. |u|/dmax   → normalised magnitude ∈ [0, 1]
 *   3. lut_index  → bucket into ramp tier (0..RAMP_N-1) via gamma LUT
 *   4. tier 0 cells are below the display threshold — skip drawing
 *      (leaves the cell black background; this is how rest-state
 *      cells disappear, making wave fronts pop)
 *   5. tier ≥ 1 → fetch glyph from k_ramp + attribute from wave_attr */
static void paint_amplitude_cell(WINDOW *w, int r, int c, float u, int theme,
                                 float inv_max) {
  bool positive = (u >= 0.0f);
  float norm = fabsf(u) * inv_max;
  if (norm > 1.0f)
    norm = 1.0f;

  int lvl = lut_index(norm);
  if (lvl == 0)
    return; /* below display threshold — skip */

  attr_t attr = wave_attr(theme, positive, lvl);
  wattron(w, attr);
  mvwaddch(w, r, c, k_ramp[lvl]);
  wattroff(w, attr);
}

/*
 * render_membrane — paint g_grid.u into the ncurses window.
 *
 * Pseudocode per cell:
 *     if show_nodal and cell sits on a sign-change boundary:
 *         paint dim '·' (nodal marker)
 *     else:
 *         paint by signed amplitude (warm/cool ramp)
 *
 * The display normaliser `display_max` is clamped to DISPLAY_MAX_FLOOR
 * so dividing by it stays finite when the field is near silence. */
static void render_membrane(WINDOW *w, int theme, bool show_nodal,
                            float display_max) {
  const float(*u)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.u;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  if (display_max < DISPLAY_MAX_FLOOR)
    display_max = DISPLAY_MAX_FLOOR;
  float inv_max = 1.0f / display_max;
  float near_zero_thresh = NODAL_SIGN_THRESH * display_max;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      if (show_nodal &&
          cell_is_nodal_line(u, r, c, cols, rows, near_zero_thresh)) {
        paint_nodal_cell(w, r, c);
        continue;
      }
      paint_amplitude_cell(w, r, c, u[r][c], theme, inv_max);
    }
  }
}

/*
 * render_overlay() — stats panel in the bottom-left corner.
 *
 * Displays:
 *   max_amplitude  — peak |u|; indicates energy still in the system
 *   energy_est     — mean (KE + PE) per cell; decays with damping
 *   mode (nx, ny)  — estimated standing wave mode from zero-crossing count
 *   CFL_2D         — c·dt·√2; stability indicator with colour coding
 *   BC             — active boundary condition name
 *   wave_speed     — current c in cells/s
 *   damping        — current γ
 *   sim_time       — total elapsed simulation time
 */
static void render_overlay(WINDOW *w, int cols, int rows, float max_amp,
                           float energy, int mode_nx, int mode_ny, float cfl,
                           float wave_speed, float damping, int bc,
                           float sim_time, bool paused, bool show_nodal,
                           int preset_id) {
  int pw = 30; /* panel width in characters */
  int ph = 13; /* panel height (rows) */
  int ox = 1;
  int oy = rows - ph - 1;
  if (oy < 0)
    oy = 0;
  if (ox + pw > cols)
    return;

  /* CFL stability colour: green < 0.7, yellow < 0.9, red ≥ 0.9 */
  int cfl_color;
  const char *cfl_label;
  if (cfl < 0.70f) {
    cfl_color = CP_NEG(0, 5);
    cfl_label = "STABLE  ";
  } else if (cfl < 0.90f) {
    cfl_color = CP_HUD;
    cfl_label = "MARGINAL";
  } else {
    cfl_color = CP_POS(0, 7);
    cfl_label = "UNSTABLE";
  }

  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  mvwprintw(w, oy + 0, ox, "+--- MEMBRANE WAVE --------+");
  mvwprintw(w, oy + 1, ox, "| max_amp  %8.4f          |", max_amp);
  mvwprintw(w, oy + 2, ox, "| energy   %8.4f          |", energy);
  mvwprintw(w, oy + 3, ox, "| mode    (%3d, %3d)         |", mode_nx, mode_ny);
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

  /* CFL row: coloured by stability */
  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  mvwprintw(w, oy + 4, ox, "| CFL_2D  ");
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
  wattron(w, COLOR_PAIR(cfl_color) | A_BOLD);
  wprintw(w, "%5.3f %-8s", cfl, cfl_label);
  wattroff(w, COLOR_PAIR(cfl_color) | A_BOLD);
  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  wprintw(w, "|");

  mvwprintw(w, oy + 5, ox, "| BC       %-10s        |", k_bc_names[bc]);
  mvwprintw(w, oy + 6, ox, "| speed   %6.1f cells/s    |", wave_speed);
  mvwprintw(w, oy + 7, ox, "| damping %8.4f           |", damping);
  mvwprintw(w, oy + 8, ox, "| sim_t   %8.2f s          |", sim_time);
  mvwprintw(w, oy + 9, ox, "| preset  %-10s       |",
            k_preset_names[preset_id]);
  mvwprintw(w, oy + 10, ox, "| nodal   %-3s  %s            |",
            show_nodal ? "ON " : "OFF", paused ? "PAUSED " : "running");
  mvwprintw(w, oy + 11, ox, "+---------------------------+");
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — every piece of mutable SCALAR state for one running session.
 *
 * Owns the SIMULATION knobs, the UI / VISUAL state, and the per-tick
 * STATS.  Does NOT own the field arrays or the grid dimensions —
 * those live in the §4 Grid struct (g_grid).  The split keeps each
 * struct readable on one screen: Grid is the PHYSICAL STATE of the
 * membrane, Scene is the RUN STATE around it.
 *
 * Knows nothing about ncurses setup — it only performs physics updates
 * and draws into a passed WINDOW*.  That separation makes the solver
 * testable headlessly and makes resize handling simple (call
 * scene_resize, which re-initialises Grid to the new dimensions).
 *
 * LAYOUT — fields are grouped into FOUR locality regions ordered the
 * way each per-frame pass touches them:
 *
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │ (A) SIMULATION PARAMS  — wave_speed, damping, bc                 │
 *   │     read by  : update_wave + apply_bc (HOT, per tick)            │
 *   │     written  : input handler (c/C, d/D, n keys)                  │
 *   │                                                                  │
 *   │ (B) UI / CONTROL       — paused, step_requested, preset_id,      │
 *   │                          mode_idx                                │
 *   │     read by  : scene_tick (paused gate), preset_apply,           │
 *   │                render_overlay (preset_id readout)                │
 *   │     written  : input handler (q/space/s/r/p/m)                   │
 *   │                                                                  │
 *   │ (C) RENDERING / VISUAL — theme, show_nodal                       │
 *   │     read by  : render_membrane, render_overlay                   │
 *   │     written  : input handler (t, l keys)                         │
 *   │                                                                  │
 *   │ (D) STATS              — max_amplitude, energy_est, mode_nx/ny,  │
 *   │                          cfl_2d, simulation_time, dt_sec         │
 *   │     read by  : render_overlay (every frame, HUD readouts)        │
 *   │     written  : compute_stats at the end of every scene_tick      │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * GEOMETRY (cols, rows) lives in §4 Grid, NOT here — see the §4 Grid
 * doc for the rationale (dims travel with the field arrays they bound).
 *
 * WHY this grouping (locality + clarity):
 *   • Each per-frame pass touches a CONTIGUOUS region, friendly to
 *     the prefetcher and to a reader scanning top-to-bottom.
 *   • Region (A) is HOT (read every substep) and tiny — fits in
 *     registers across the inner update_wave loop.
 *   • Region (B) is COLD (only changes on keypress) but its bits
 *     gate big behaviours (paused stops physics, mode_idx selects a
 *     standing-wave mode shape on 'm' press).
 *   • Region (C) is read-only in the render pass, written only on
 *     theme/nodal toggle — no physics path touches it.
 *   • Region (D) is the BRIDGE: physics writes, render reads.  Caching
 *     these once in compute_stats means the renderer never has to
 *     re-scan the field arrays to draw the HUD overlay.
 *
 * WHY a separate Grid struct (not embedded in Scene):
 *   The field arrays are ~240 KB; embedding Grid would make Scene
 *   240 KB and obscure where the SIMULATION STATE actually lives.
 *   Keeping Grid distinct lets a reader open the §4 block and see
 *   "everything that defines the membrane right now" without wading
 *   through UI flags and stats.  Scene fits in one screen this way.
 *
 * WHY one big Scene (not split SimScene + RenderScene + UIScene):
 *   The simulation rebuilds atomically on resize / reset / preset
 *   change — `memset(s, 0, sizeof *s)` + a few field writes is the
 *   cleanest possible reset.  Splitting would force coordinated
 *   resets across multiple structs and risk drift between them
 *   (e.g. resize that rebuilds physics but forgets to clear stats).
 */
struct Scene {
  /* ── (A) SIMULATION PARAMS — hot-loop physics knobs ────────────── *
   * Read every tick by update_wave.  Mutated by the user via the
   * keyboard handler (c/C, d/D, n).  Independent of g_grid: changing
   * wave_speed doesn't rewrite g_grid.u, it just changes the forces
   * update_wave generates next tick.                                  */
  float wave_speed; /* c in cells/s — propagation speed of small
                     * disturbances.  Sets ω for every mode; the
                     * CFL bound c·dt·√2 ≤ 1 caps how high it
                     * can go for a given sim_fps [refs 4, 5]. */
  float damping;    /* γ in 1/s — coefficient of the −γ·v drag
                     * term.  Time-constant for free decay is
                     * τ ≈ 1/γ; default γ=0.003 gives τ ≈ 333
                     * ticks ≈ 5.5 s at 60 Hz.                  */
  int bc;           /* boundary condition enum — DIRICHLET (0,
                     * clamped rim, phase-flipped reflection),
                     * NEUMANN (1, free edge, no flip), or
                     * PERIODIC (2, toroidal wrap).  Switched
                     * by 'n'; reset triggers a fresh preset.   */

  /* ── (B) UI / CONTROL — flow + preset state ──────────────────── *
   * Owned by the input handler.  scene_tick reads `paused` /
   * `step_requested` to decide whether to advance physics this
   * frame.  Preset bookkeeping lives here too so the renderer can
   * label the current preset in the side overlay.                   */
  bool paused;         /* freeze update_wave each tick; HUD shows
                        * "PAUSED" when set.  Toggled by space/p. */
  bool step_requested; /* one-shot: when both `paused` and this
                        * are true, scene_tick runs ONCE then
                        * clears this flag.  The 's' key sets it. */
  int preset_id;       /* 0..PRESET_COUNT-1 — last preset triggered;
                        * used as the reset target for 'r' and as
                        * a readout in render_overlay.            */
  int mode_idx;        /* index into k_mode_table[] — advances on
                        * every 'm' press so successive resonance
                        * triggers pick visibly different (nx,ny)
                        * modes [ref 1].  -1 before first 'm'.    */

  /* ── (C) RENDERING / VISUAL — palette + overlay knobs ──────── *
   * Read by render_membrane (theme picks the colour ramp) and the
   * nodal-line detector in render_membrane.  No physics function
   * touches this region.                                              */
  int theme;       /* 0..N_THEMES-1 — index into k_themes[];
                    * cycles on 't'.  Pure visual: changes
                    * colour-pair bindings, does NOT touch
                    * g_grid or any solver value.              */
  bool show_nodal; /* draw dim '·' on zero-crossing cells so
                    * the geometry of each standing-wave mode
                    * is visible — the discrete analogue of
                    * Chladni's sand patterns [ref 8].         */

  /* ── (D) STATS — physics-to-HUD bridge ─────────────────────── *
   * compute_stats() WRITES these once at the end of every tick;
   * render_overlay() READS them every frame to populate the HUD
   * panel.  Caching here avoids re-scanning g_grid.u every frame.     */
  float max_amplitude;   /* peak |u| across the grid — the display
                          * normaliser so the colour ramp uses the
                          * full dynamic range even as the wave
                          * decays.                                  */
  float energy_est;      /* mean (½v² + ½c²|∇u|²) per cell — the
                          * total mechanical energy, falls
                          * monotonically with damping [ref 1].      */
  int mode_nx, mode_ny;  /* estimated mode numbers from zero-crossing
                          * count along the centre row / column —
                          * for a pure (n,m) standing wave these
                          * match exactly.                           */
  float cfl_2d;          /* c·dt·√2 — CFL stability gauge [ref 4].
                          * Coloured green/yellow/red in the overlay
                          * by render_overlay.                       */
  float simulation_time; /* total seconds advanced since last reset;
                          * grows by dt per scene_tick.              */
  float dt_sec;          /* last tick's dt — exposed so the overlay
                          * can show "actual dt" if substepping or
                          * frame-pacing diverged.                   */

  /* GEOMETRY — owned by Grid, NOT Scene.  Read via g_grid.cols /
   * g_grid.rows everywhere else in the file.                        */
};

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->wave_speed = WAVE_SPEED_DEFAULT;
  s->damping = DAMPING_DEFAULT;
  s->bc = BC_DIRICHLET;
  s->theme = 0;
  s->show_nodal = true;
  s->mode_idx = -1; /* first 'm' press → k_mode_table[0] */
  init_grid(s->bc, cols, rows);
  preset_apply(s, PRESET_CENTER);
}

static void scene_reset(Scene *s) {
  init_grid(s->bc, g_grid.cols, g_grid.rows);
  s->simulation_time = 0.0f;
  s->max_amplitude = 0.0f;
  s->energy_est = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows) {
  init_grid(s->bc, cols, rows);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep.
 *
 * Called from the accumulator loop in §10.  dt is always exactly
 * 1/sim_fps seconds.  The order of operations is:
 *   1. Solve wave PDE for one step (update_wave)
 *   2. Compute stats for the overlay (compute_stats)
 *
 * If paused and no step is requested, return immediately — the
 * physics state is frozen but the display continues to render.
 */
static void scene_tick(Scene *s, float dt) {
  if (s->paused && !s->step_requested)
    return;
  s->step_requested = false;

  s->dt_sec = dt;
  s->simulation_time += dt;

  update_wave(dt, s->wave_speed, s->damping, s->bc);

  compute_stats(s->wave_speed, dt, &s->max_amplitude, &s->energy_est,
                &s->mode_nx, &s->mode_ny, &s->cfl_2d);
}

static void scene_draw(const Scene *s, WINDOW *w, float alpha, float dt_sec) {
  (void)alpha;
  (void)dt_sec; /* membrane has no continuous-motion interpolation */

  render_membrane(w, s->theme, s->show_nodal, s->max_amplitude);

  render_overlay(w, g_grid.cols, g_grid.rows, s->max_amplitude, s->energy_est,
                 s->mode_nx, s->mode_ny, s->cfl_2d, s->wave_speed, s->damping,
                 s->bc, s->simulation_time, s->paused, s->show_nodal,
                 s->preset_id);
}

/* ── preset implementations ─────────────────────────────────────────── */

/* Map a fractional position (fx, fy) ∈ [0,1] to actual cell coords on
 * the live grid.  Centralised so every strike helper expresses its
 * position as "30 % from the left, 35 % from the top" instead of
 * doing the (cols-1)·fx arithmetic inline. */
static void cell_at_fraction(float fx, float fy, float *out_x, float *out_y) {
  *out_x = (float)(g_grid.cols - 1) * fx;
  *out_y = (float)(g_grid.rows - 1) * fy;
}

/* STRIKE 0 — CENTRE PULSE.  One Gaussian impulse at the geometric
 * centre of the membrane.  Produces a symmetric expanding ring that
 * reflects repeatedly off all four walls.  Under Dirichlet BC each
 * reflection inverts phase — count crest↔trough flips at a corner to
 * verify the wall round-trip time. */
static void strike_centre_pulse(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_CENTRE, STRIKE_Y_CENTRE, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* STRIKE 1 — LEFT-EDGE PULSE.  Single impulse 15% in from the left wall,
 * centred vertically.  The asymmetric position drives a RICH
 * superposition of modes (many harmonics simultaneously) — under
 * Dirichlet BC and zero damping the long-term state would settle into
 * quasi-periodic Chladni-like patterns [ref 8]. */
static void strike_left_edge_pulse(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_LEFT_EDGE, STRIKE_Y_CENTRE, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* STRIKE 2 — DOUBLE OFF-CENTRE.  Two simultaneous Gaussian pulses at
 * (0.30, 0.35) and (0.70, 0.65) cell-fractions.  Symmetric about the
 * centre in x but offset in y, so the line through the two strikes
 * does NOT pass through the centre — that produces a richer
 * non-symmetric interference pattern (Crawford [ref 6] §9.3,
 * two-source addition/cancellation fringes). */
static void strike_double_offcentre_pulses(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_DOUBLE_L, STRIKE_Y_DOUBLE_L, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  cell_at_fraction(STRIKE_X_DOUBLE_R, STRIKE_Y_DOUBLE_R, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* STRIKE 3 — NEXT STANDING-WAVE MODE.  Reads (nx, ny) from the mode
 * cycle table at the scene's current mode_idx, then stamps the
 * matching eigenmode onto the field.  See preset_resonance() for the
 * 45°-phase scaling and Rayleigh-eigenfrequency derivation. */
static void strike_next_standing_wave_mode(const Scene *s, int bc) {
  int idx = s->mode_idx;
  if (idx < 0 || idx >= MODE_TABLE_LEN)
    idx = 0;
  preset_resonance(k_mode_table[idx][0], k_mode_table[idx][1], RESONANCE_AMP);
  apply_bc(bc);
}

/* Dispatch the preset id to the matching strike helper.  Body reads
 * as pure SELECTION; the resetting + preset-id bookkeeping happens
 * once at the top, all impulse placement lives in the named helpers. */
static void preset_apply(Scene *s, int id) {
  scene_reset(s);
  s->preset_id = id;

  switch (id) {
  case PRESET_CENTER:
    strike_centre_pulse(s->bc);
    break;
  case PRESET_EDGE:
    strike_left_edge_pulse(s->bc);
    break;
  case PRESET_DOUBLE:
    strike_double_offcentre_pulses(s->bc);
    break;
  case PRESET_RESONANCE:
    strike_next_standing_wave_mode(s, s->bc);
    break;
  }
}

/* ===================================================================== */
/* §9  screen — ncurses double-buffer display layer                      */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer's minimal mutable state.
 *
 * Holds ONLY the current terminal dimensions.  Everything else needed
 * for rendering (the active WINDOW, the colour pairs, attribute
 * masks) lives in ncurses-managed globals (stdscr, the colour-pair
 * table set up by color_init).
 *
 * WHY a struct around two ints (not just file-scope globals):
 *   • Group ownership — `cols` and `rows` always belong together;
 *     packaging them prevents accidental "I updated one but not the
 *     other" bugs on resize.
 *   • Pass-by-pointer semantics — screen_init / screen_resize take a
 *     Screen* and write both fields atomically, so the call site
 *     never sees a half-updated state.
 *   • Future-proof — if a future variant adds a second WINDOW (e.g.
 *     a status pad) the new field lives here without further code
 *     surgery elsewhere.
 *
 * WHY this is DIFFERENT from Grid.cols / Grid.rows:
 *   Grid's dims track the SIMULATION grid (which equals the terminal
 *   for this demo, but conceptually doesn't have to — another variant
 *   could simulate a smaller fixed grid centred in a larger terminal).
 *   Screen's dims track what ncurses thinks the terminal is.  They are
 *   kept in sync by app_do_resize(), which is the single point of
 *   truth for resize handling: it reads the new terminal dims into
 *   Screen, then calls scene_resize → init_grid to push the same dims
 *   into Grid.
 */
typedef struct {
  int cols; /* current terminal width  (from getmaxyx) */
  int rows; /* current terminal height (from getmaxyx) */
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

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, alpha, dt_sec);

  /* ── Row 0  STATUS HUD ──────────────────────────────────────────── *
   * Canonical CLAUDE.md HUD: bright yellow + A_BOLD, full row cleared
   * first so the wave never bleeds through.  Shows fps, sim Hz, wave
   * params, BC, and the paused / running state.                        */
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " membrane  %5.1f fps  sim:%3d Hz  c=%.0f  "
           "g=%.4f  BC=%-10s  %s ",
           fps, sim_fps, sc->wave_speed, sc->damping, k_bc_names[sc->bc],
           sc->paused ? "PAUSED " : "running");
  move(0, 0);
  clrtoeol();
  int hx = s->cols - (int)strlen(status);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── Row n-1  ACTION-KEYS HUD ───────────────────────────────────── *
   * Canonical CLAUDE.md HUD: bright cyan + A_BOLD (NEVER A_DIM — dim
   * text vanishes against any animated palette).  Full row cleared
   * first so the wave never bleeds through the legend.                 */
  int hint_row = s->rows - 1;
  move(hint_row, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(hint_row, 0,
           " q:quit  spc:pause  s:step  r:reset  b:center  e:edge"
           "  f:double  m:mode  c/C:speed  d/D:damp  n:BC"
           "  l:nodal  t:theme  [/]:Hz ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §10 app — signals, resize, input, main loop                           */
/* ===================================================================== */

/*
 * App — the top-level container holding everything that lives across
 *       the lifetime of the running program.
 *
 * Owns the Scene (simulation), the Screen (terminal display layer),
 * the simulation-Hz knob (separate from Scene because it's a loop
 * pacing parameter, not a physics quantity), and two volatile signal
 * flags that the SIGINT/SIGTERM/SIGWINCH handlers use to communicate
 * back to the main loop.
 *
 * WHY group Scene + Screen + signal flags in one struct:
 *   • Single point of truth for the whole running program — `g_app`
 *     is the only true file-scope global the loop touches.
 *   • Cleaner signal handling — the C standard requires signal handlers
 *     to use `volatile sig_atomic_t` for flags they set; keeping those
 *     flags inside the App struct means the rest of the codebase can
 *     pass `App*` around without worrying about which globals to
 *     access (the handlers themselves still need file scope, hence
 *     `static App g_app`).
 *   • `App *app = &g_app` at the top of main() is the only place
 *     anyone touches the global by name — the rest of main() reads
 *     `app->scene`, `app->running`, etc., which makes the code easy
 *     to refactor toward fully heap-allocated apps later.
 *
 * Signal-handler discipline:
 *   • Both `running` and `need_resize` are `volatile sig_atomic_t` —
 *     mandatory for cross-thread / signal-context flag passing (C11
 *     §7.14.1.1).  Anything else (locks, function calls, struct
 *     writes) is UNDEFINED BEHAVIOUR from a signal handler.
 *   • The handlers (`on_exit_signal`, `on_resize_signal`) do the
 *     ABSOLUTE MINIMUM: flip a flag.  All real work — calling
 *     endwin(), getmaxyx, rebuilding the scene — happens
 *     synchronously from the main loop's next iteration.
 *
 * Storage:
 *   `static App g_app` at file scope so the signal handlers (which
 *   take no user pointer) can reach the flags.  Despite the name,
 *   nothing in the per-frame critical path reads `g_app` by name —
 *   main() takes its address once and uses `app->...` from there on.
 */
typedef struct {
  Scene scene;                       /* full simulation state         */
  Screen screen;                     /* terminal display geometry     */
  int sim_fps;                       /* physics-step rate [Hz].
                                      * Independent of TARGET_FPS (the
                                      * render-frame cap) — the
                                      * accumulator loop in main()
                                      * runs `sim_fps` substeps per
                                      * wall-clock second regardless of
                                      * how fast frames render.       */
  volatile sig_atomic_t running;     /* 0 → main loop exits next iter.
                                      * Set by SIGINT/SIGTERM handler
                                      * AND by the 'q'/ESC key path.   */
  volatile sig_atomic_t need_resize; /* 1 → app_do_resize() runs next
                                      * iter to rebuild Scene + Screen
                                      * for the new terminal size.
                                      * Set by the SIGWINCH handler.   */
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
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/*
 * app_handle_key() — dispatch all user input in one place.
 *
 * Groups:
 *   flow control  — q, ESC, space, s (single step)
 *   excitation    — b (center), e (edge), f (double), m (resonance)
 *   physics       — c/C (wave speed), d/D (damping), n (BC)
 *   simulation    — r (reset), p/P (cycle preset)
 *   visual        — l (nodal), t (theme), ]/[ (sim Hz)
 */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;

  switch (ch) {
  /* ── flow control ───────────────────────────────────────────── */
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 's':
  case 'S':
    /* Single-step: pause if running, then request one tick.
     * Useful for studying wave propagation frame by frame.     */
    s->paused = true;
    s->step_requested = true;
    break;

  /* ── manual excitation ──────────────────────────────────────── */
  case 'b':
  case 'B':
    /* Center strike — works regardless of paused state */
    apply_excitation((float)(g_grid.cols - 1) * 0.5f,
                     (float)(g_grid.rows - 1) * 0.5f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'e':
  case 'E':
    apply_excitation((float)(g_grid.cols - 1) * 0.15f,
                     (float)(g_grid.rows - 1) * 0.5f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'f':
  case 'F':
    apply_excitation((float)(g_grid.cols - 1) * 0.30f,
                     (float)(g_grid.rows - 1) * 0.35f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_excitation((float)(g_grid.cols - 1) * 0.70f,
                     (float)(g_grid.rows - 1) * 0.65f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'm':
  case 'M':
    /* Advance the mode cycle BEFORE applying — first press picks
     * k_mode_table[0] (an interesting non-fundamental mode), every
     * subsequent press picks the next shape.  Clearing s->paused
     * guarantees the user sees the new mode animate immediately,
     * even if they pressed 's' (single-step) at some earlier point. */
    s->mode_idx = (s->mode_idx + 1) % MODE_TABLE_LEN;
    s->paused = false;
    preset_apply(s, PRESET_RESONANCE);
    break;

  /* ── wave speed (c) ─────────────────────────────────────────── *
   * Increasing c raises all eigenfrequencies proportionally,
   * shortening the oscillation period of every mode.
   * The CFL_2D indicator shows how close to instability we are.  */
  case 'c':
    s->wave_speed += WAVE_SPEED_STEP;
    if (s->wave_speed > WAVE_SPEED_MAX)
      s->wave_speed = WAVE_SPEED_MAX;
    break;
  case 'C':
    s->wave_speed -= WAVE_SPEED_STEP;
    if (s->wave_speed < WAVE_SPEED_MIN)
      s->wave_speed = WAVE_SPEED_MIN;
    break;

  /* ── damping (γ) ────────────────────────────────────────────── *
   * Higher damping drains energy faster; at max γ the wave
   * decays in a handful of oscillation periods.
   * At γ=0 the wave bounces indefinitely (ideal membrane).       */
  case 'd':
    s->damping += DAMPING_STEP;
    if (s->damping > DAMPING_MAX)
      s->damping = DAMPING_MAX;
    break;
  case 'D':
    s->damping -= DAMPING_STEP;
    if (s->damping < DAMPING_MIN)
      s->damping = DAMPING_MIN;
    break;

  /* ── boundary conditions ────────────────────────────────────── *
   * Switching BC changes reflection behaviour immediately.
   * After switching, trigger a fresh reset so the new BC applies
   * to a clean grid (the old field may have incompatible values). */
  case 'n':
  case 'N':
    s->bc = (s->bc + 1) % BC_COUNT;
    preset_apply(s, s->preset_id);
    break;

  /* ── reset / presets ────────────────────────────────────────── */
  case 'r':
  case 'R':
    preset_apply(s, s->preset_id);
    break;

  case 'p':
    preset_apply(s, (s->preset_id + 1) % PRESET_COUNT);
    break;
  case 'P':
    preset_apply(s, (s->preset_id + PRESET_COUNT - 1) % PRESET_COUNT);
    break;

  /* ── visual ─────────────────────────────────────────────────── */
  case 'l':
  case 'L':
    s->show_nodal = !s->show_nodal;
    break;

  case 't':
  case 'T':
    s->theme = (s->theme + 1) % N_THEMES;
    break;

  /* ── simulation Hz ──────────────────────────────────────────── *
   * Changing sim_fps changes dt = 1/sim_fps, which changes CFL.
   * Raising Hz shrinks dt → smaller CFL → safer but more CPU.
   * Lowering Hz increases dt → larger CFL → may go unstable.    */
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

/* ─────────────────────────────────────────────────────────────────────
 * main() — fixed-timestep accumulator game loop
 * Same structure as framework.c §8 main().  See that file for the
 * detailed walk-through of each loop phase.
 * ───────────────────────────────────────────────────────────────────── */
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

    /* ── resize ──────────────────────────────────────────────── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── dt measurement ──────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* pause guard */

    /* ── fixed-timestep accumulator ──────────────────────────── */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* ── render interpolation factor ─────────────────────────── */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* ── FPS counter (500 ms sliding window) ─────────────────── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── frame cap — sleep BEFORE render ─────────────────────── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* ── draw + present ──────────────────────────────────────── */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    /* ── input ───────────────────────────────────────────────── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
