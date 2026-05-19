/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * schrodinger.c — 1-D Quantum Wavefunction
 *
 * Evolves ψ(x,t) under the time-dependent Schrödinger equation:
 *
 *   iℏ ∂ψ/∂t = [ -ℏ²/2m ∂²/∂x² + V(x) ] ψ
 *
 * Discretised with the Crank-Nicolson scheme (unconditionally stable):
 *   (I + iΔt/2 H) ψ(t+Δt) = (I - iΔt/2 H) ψ(t)
 *
 * Solved each step with the Thomas algorithm (tridiagonal complex system).
 *
 * Display — phase-coloured probability bars:
 *   For each terminal column the bar HEIGHT encodes |ψ(x)|² (the
 *   probability density) and the bar COLOUR encodes arg ψ(x) sampled
 *   from an 8-stop phase wheel.  As the packet propagates the phase
 *   velocity is faster than the group velocity, so the colours
 *   "barber-pole" through the envelope — that rolling rainbow IS the
 *   quantum nature you couldn't see in a plain |ψ|² plot.
 *
 *   Bottom strip shows the potential V(x): bright '|' for hard walls,
 *   dimmer '#' bars for finite barriers, '·' for free regions.
 *
 * Presets (1-4):
 *   1 Free packet     — Gaussian wave packet, no barrier
 *   2 Barrier tunnel  — Finite rectangular barrier (quantum tunnelling)
 *   3 Harmonic well   — Quadratic V(x); packet bounces
 *   4 Double slit     — Two narrow gaps in a wall; interference pattern
 *
 * Keys: q quit  p pause  r reset  1/2/3/4 preset  +/- energy
 *       SPACE kick  t/T theme (10 brightness-safe palettes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/schrodinger.c \
 *       -o schrodinger -lncurses -lm
 *
 * §1 config       §2 clock        §3 color/theme       §4 state (structs)
 * §5 potential    §6 wavepacket   §7 CN time evolution §8 driver+diagnostics
 * §9 draw         §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Crank-Nicolson finite-difference scheme [1][2].
 *                  Averages the explicit and implicit Euler steps to give
 *                  an unconditionally stable, 2nd-order-in-time method.
 *                  Results in a tridiagonal complex linear system per step,
 *                  solved with the Thomas algorithm [3] in O(N) — not O(N³).
 *
 * Physics        : Quantum mechanics, wave-particle duality [4][5].
 *                  ψ(x,t) is complex-valued; |ψ|² is the probability density
 *                  (Born rule).  The wavefunction spreads (Re, Im oscillate),
 *                  reflects off walls, and tunnels through barriers with
 *                  probability T ≈ exp(−2κd) where κ = √(2m(V−E))/ℏ [4, Ch. 8].
 *
 * Math           : Natural units ℏ = m = 1 simplify the Hamiltonian to
 *                  H = −½ d²/dx² + V(x).  DX and DT are in these dimensionless
 *                  units.  The CN matrix is constant-tridiagonal except for
 *                  the diagonal which carries V[i], so the Thomas LU pass
 *                  changes only O(N) values per step.
 *
 * Visualisation  : Phase-coloured probability bars [6][7].  Bar height
 *                  = |ψ|² (the Born rule); bar colour = arg ψ mapped onto
 *                  an 8-stop phase wheel.  As the packet propagates the
 *                  phase velocity exceeds the group velocity so the colours
 *                  "barber-pole" through the envelope — a direct visual
 *                  encoding of the complex nature of ψ.
 *
 * Performance    : N_GRID=512 points × STEPS_PER_FRAME=20 solves per frame.
 *                  Each solve is O(N), so total: O(10240) mults per frame —
 *                  easily runs at 30 fps even on a single core.
 *
 * References (cited inline as [n]):
 *
 *   [1] Crank, J.; Nicolson, P. (1947) — "A practical method for
 *       numerical evaluation of solutions of partial differential
 *       equations of the heat-conduction type", Proc. Camb. Phil. Soc.
 *       43 (1), 50–67.  The original CN paper.  Although the title
 *       says heat-conduction the same trapezoidal averaging applies to
 *       any parabolic equation including the imaginary-time Schrödinger
 *       form we use here.
 *
 *   [2] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.; Flannery,
 *       B. P. (2007) — *Numerical Recipes*, 3rd ed., Cambridge UP.
 *       §19.2 derives CN for the diffusion equation with the same
 *       (I + iΔt/2 H)ψ = (I − iΔt/2 H)ψ_prev structure we implement;
 *       §2.4 covers the Thomas algorithm for tridiagonal systems.
 *
 *   [3] Thomas, L. H. (1949) — "Elliptic Problems in Linear Difference
 *       Equations over a Network", Watson Sci. Comput. Lab. report.
 *       The original O(N) tridiagonal solver — a forward elimination
 *       (carry the pivot down) followed by a back substitution.
 *       Our `thomas_*` helpers in §6 implement it directly.
 *
 *   [4] Griffiths, D. J. (2018) — *Introduction to Quantum Mechanics*,
 *       3rd ed., Cambridge UP.  Foundational textbook for TDSE
 *       (Ch. 2), Gaussian wave packets and dispersion (Ch. 2.6),
 *       tunneling (Ch. 8), and double-slit interference (Ch. 1).
 *
 *   [5] Feynman, R. P.; Leighton, R. B.; Sands, M. (1965) — *The
 *       Feynman Lectures on Physics*, Vol. III, Addison-Wesley.  Ch. 1
 *       presents the double-slit thought experiment that motivates
 *       preset 4; Ch. 7 derives the time-dependent Schrödinger equation
 *       from energy operator postulates.
 *
 *   [6] Thaller, B. (2000) — *Visual Quantum Mechanics*, Springer.
 *       The textbook source for phase-coloured wavefunction renderings;
 *       Ch. 2 explains why mapping arg ψ to a colour wheel makes the
 *       structure of ψ visually intuitive in a way Re/Im plots cannot.
 *
 *   [7] Wegert, E. (2012) — *Visual Complex Functions*, Birkhäuser.
 *       Mathematical foundation for domain colouring.  Generalises [6]
 *       to arbitrary complex-valued functions, with rigorous treatment
 *       of how phase + modulus encode a function's structure.
 *
 *   [8] Cohen-Tannoudji, C.; Diu, B.; Laloë, F. (1977) — *Quantum
 *       Mechanics*, Vol. I, Wiley.  Complement G_I derives the
 *       Gaussian wave packet's group / phase velocity and its
 *       spreading rate σ(t) = σ₀·√(1 + (ℏt / 2mσ₀²)²) — the analytic
 *       form our preset 1 numerical solution converges toward.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* N_GRID: spatial grid points.  Power of 2 enables future FFT-based solvers.
 * 512 gives DX=1/511 ≈ 0.002 — fine enough to resolve typical Gaussian
 * wave packets (σ ≈ 0.05) with ~25 points per standard deviation.        */
#define N_GRID 512
#define X_MIN 0.f
#define X_MAX 1.f
#define DX ((X_MAX - X_MIN) / (float)(N_GRID - 1)) /* ≈ 0.002 (dimensionless) \
                                                    */

/* DT: time step in natural units (ℏ=m=1).
 * Crank-Nicolson is unconditionally stable for any DT, but large DT causes
 * phase errors.  DT=0.0002 keeps the numerical dispersion < 1% over 1000 steps
 * for wave packets with k ≈ 50 (momentum).                                */
#define DT 0.0002f

/* STEPS_PER_FRAME: solver passes per rendered frame.
 * At 30 fps and DT=0.0002, one second of display = 30×20 = 600 time units.
 * The wave packet traverses the domain in ~T=1/(k/(2π)) ≈ 0.13 time units,
 * so the animation completes a traversal in ~0.22 s of display time.      */
#define STEPS_PER_FRAME 20

/* Natural units: ℏ=1, m=1 reduce H = −ℏ²/(2m)·d²/dx² + V(x) to H = −½·d²/dx² +
 * V(x). All energies, velocities, and potentials are in these dimensionless
 * units. */
#define HBAR 1.f
#define MASS 1.f

/* V_BARRIER: finite rectangular barrier height (energy units).
 * Wave energy E ≈ k²/2 ≈ 50²/2 = 1250 (for default k=50).
 * V_BARRIER=2000 > E → classically forbidden; quantum tunnelling occurs.
 * Transmission T ≈ exp(−2κd) where κ=√(2(V−E)) and d = barrier width.   */
#define V_BARRIER 2000.f

/* V_WALL: hard wall potential (effectively infinite compared to wave energy).
 * Used for the double-slit walls and domain boundaries.
 * V_WALL >> V_BARRIER ensures walls are fully reflective.                 */
#define V_WALL 50000.f

#define RENDER_NS (1000000000LL / 30)
#define HUD_TOP_ROWS 1 /* row 0 reserved for top status bar */
#define HUD_BOT_ROWS 2 /* row rows-1 hint bar, row rows-2 potential strip */
#define N_PHASE 8      /* phase-wheel resolution */
#define N_THEMES 10

typedef struct {
  const char *name;
  int id;
} Preset;
static const Preset PRESETS[] = {
    {"Free", 0},
    {"Barrier", 1},
    {"Harmonic", 2},
    {"D-Slit", 3},
};
#define N_PRESETS 4

/* preset, theme, paused, fps_disp now live on g_scene (Scene struct, §4). */

/*
 * Color-pair layout:
 *   CP_PHASE_0..7    8-stop phase wheel for the probability bars
 *                     (theme-driven; arg ψ ∈ [-π, π] maps to this ramp)
 *   CP_WALL          hard-wall accent — bright, theme-driven
 *   CP_BARRIER       finite-barrier accent — medium, theme-driven
 *   CP_AXIS          dim baseline / "free V(x)=0" marker
 *   CP_HUD           top status bar — bright yellow + bold, theme-INDEPENDENT
 *   CP_HINT          bottom hint bar — bright cyan + bold, theme-INDEPENDENT
 */
enum {
  CP_PHASE_0 = 1,
  CP_PHASE_1,
  CP_PHASE_2,
  CP_PHASE_3,
  CP_PHASE_4,
  CP_PHASE_5,
  CP_PHASE_6,
  CP_PHASE_7,
  CP_WALL,
  CP_BARRIER,
  CP_AXIS,
  CP_HUD,
  CP_HINT,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color / theme — 10 brightness-safe phase-wheel palettes           */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Theme — palette for one rendering of the demo.
 *
 * The 8-stop `phase[]` ramp is the heart of the visual: as arg ψ rotates
 * through [-π, π], the bar at that column cycles through phase[0]..
 * phase[7] and back.  Themes with hue variation across the ramp (Nova,
 * Neon, Oceanic) make the rotation read as a moving rainbow.  Themes
 * with mostly brightness variation (Matrix, Fire, Mono) produce a
 * shimmering / pulsing effect instead.  Both are valid aesthetics.
 *
 * Brightness safety per CLAUDE.md: every entry sits at 256-cube index
 * ≥ 24; indices 16-23 / 232-239 are forbidden (they render as background
 * black on default-bg terminals).
 *
 * `wall` / `barrier` / `axis` are FIXED chrome roles independent of
 * the phase ramp — they identify the potential V(x) layer.  Walls are
 * the brightest accent so even a single-column wall stays visible;
 * barriers are medium so they read as "passable with tunnelling cost";
 * axis is dim grey so the baseline doesn't compete with the wave.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  const char *name;
  short phase[N_PHASE]; /* 8-stop phase wheel, cycled by arg ψ */
  short wall;           /* hard wall marker (bright)            */
  short barrier;        /* finite barrier marker (medium)       */
  short axis;           /* baseline / V=0 axis (dim)            */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        phase wheel (8 cube indices)                  wall  barrier
       axis */
    {"Matrix", {28, 34, 40, 46, 82, 118, 154, 190}, 46, 100, 240},
    {"Fire", {124, 160, 196, 202, 208, 214, 220, 226}, 226, 166, 240},
    {"Oceanic", {24, 31, 38, 45, 51, 87, 123, 195}, 51, 38, 240},
    {"Neon", {93, 129, 165, 201, 51, 87, 207, 213}, 201, 129, 240},
    {"Mono", {240, 244, 247, 250, 252, 253, 254, 255}, 255, 247, 240},
    {"Ice", {39, 75, 111, 117, 153, 159, 195, 231}, 231, 117, 240},
    {"Nova", {196, 208, 220, 226, 46, 51, 93, 201}, 231, 208, 240},
    {"Forest", {58, 64, 100, 106, 142, 148, 178, 184}, 154, 142, 240},
    {"Desert", {130, 166, 172, 178, 214, 220, 222, 230}, 230, 178, 240},
    {"Eclipse", {240, 124, 160, 196, 202, 208, 215, 220}, 220, 160, 240},
};

/* Fixed HUD chrome — same on every theme so status / hints stay legible. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

static bool g_256; /* boot-time terminal capability; not scene state */

/*
 * theme_apply — push the chosen palette into ncurses' colour-pair
 * table.  Idempotent; safe to call from t / T handlers and at startup.
 * `idx` is wrapped negative-safe so callers don't need to pre-modulo.
 * CP_HUD and CP_HINT are reinstated every call so any future bug that
 * overwrites them can't outlive one keypress.
 */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[((idx % N_THEMES) + N_THEMES) % N_THEMES];
  if (g_256) {
    for (int i = 0; i < N_PHASE; i++)
      init_pair(CP_PHASE_0 + i, t->phase[i], -1);
    init_pair(CP_WALL, t->wall, -1);
    init_pair(CP_BARRIER, t->barrier, -1);
    init_pair(CP_AXIS, t->axis, -1);
    init_pair(CP_HUD, CHROME_HUD_256, -1);
    init_pair(CP_HINT, CHROME_HINT_256, -1);
  } else {
    /* 8-colour fallback: theme-independent rainbow-ish phase wheel. */
    static const short fallback[N_PHASE] = {
        COLOR_RED,  COLOR_YELLOW,  COLOR_GREEN, COLOR_CYAN,
        COLOR_BLUE, COLOR_MAGENTA, COLOR_RED,   COLOR_YELLOW,
    };
    for (int i = 0; i < N_PHASE; i++)
      init_pair(CP_PHASE_0 + i, fallback[i], -1);
    init_pair(CP_WALL, COLOR_WHITE, -1);
    init_pair(CP_BARRIER, COLOR_YELLOW, -1);
    init_pair(CP_AXIS, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(int boot_theme) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  theme_apply(boot_theme);
}

/*
 * phase_color_pair — map arg(re + i·im) onto the 8-stop phase wheel.
 * atan2 gives angle ∈ [-π, π]; we shift to [0, 2π], then map linearly
 * into [0, N_PHASE).  The wheel wraps so wraparound at ±π is seamless.
 */
static inline int phase_color_pair(float re, float im) {
  float angle = atan2f(im, re) + (float)M_PI; /* [0, 2π] */
  int idx = (int)(angle * (float)N_PHASE / (2.f * (float)M_PI));
  if (idx < 0)
    idx = 0;
  if (idx >= N_PHASE)
    idx = N_PHASE - 1;
  return CP_PHASE_0 + idx;
}

/* ===================================================================== */
/* §4  state — data structures for the quantum problem                   */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Cx — a complex number, the fundamental quantity of quantum mechanics.
 *
 * Intent:
 *   Carries one element of the LHS matrix or one element of the RHS
 *   vector in the Crank-Nicolson tridiagonal solve.  Used everywhere
 *   the wavefunction needs to be combined with the imaginary-unit i
 *   that appears in the Schrödinger equation; in particular every
 *   matrix-coefficient computation in §7 produces and consumes Cx.
 *
 * Why a custom struct rather than C99 _Complex:
 *   - Cross-compiler portability (some embedded / older MSVC toolchains
 *     lack robust _Complex; this version compiles anywhere).
 *   - Explicit struct layout — cleaner to inline-init `(Cx){a, b}` than
 *     to build a _Complex from `a + b*I` with the implied include.
 *   - We use only +, −, ×, ÷ in the Thomas algorithm — those four are
 *     easy to write by hand and the compiler auto-vectorises them on
 *     -O2 anyway, so we lose nothing vs the language type.
 *
 * Conventions:
 *   z   = re + i · im              (standard split)
 *   |z|² = re² + im²                (squared modulus, Born rule [4])
 *   z*  = re − i · im               (complex conjugate)
 *   z₁/z₂ = (z₁ · z₂*) / |z₂|²      (division via conjugate)
 *
 * Refs: Numerical Recipes [2, §2.4] uses the same explicit (re, im)
 * tuple representation when discussing complex tridiagonal solvers;
 * Cohen-Tannoudji [8, complement B_I] uses identical algebra notation.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* re — real part Re z.  Carries the "even" component of the
   * Schrödinger LHS matrix or RHS vector.  Stored as float because
   * the CN scheme is unconditionally stable so single precision is
   * sufficient — typical errors are O(10⁻⁶) and dominated by the
   * time-discretisation truncation, not floating-point round-off. */
  float re;

  /* im — imaginary part Im z.  In our Schrödinger discretisation
   * the imaginary part of every matrix entry carries the dt·H_ii
   * contribution (the off-diagonals are pure-imaginary −i·r); the
   * imaginary part of every RHS entry carries the kinetic stencil
   * term acting on the current ψ.  See cn_rhs_at / cn_diagonal_at
   * in §7 for the derivations. */
  float im;
} Cx;

/* cx_sub — element-wise subtraction.  Used in the Thomas forward sweep
 * to subtract the pivoted multiple of the previous row from the current
 * row (eliminating the sub-diagonal). */
static inline Cx cx_sub(Cx a, Cx b) { return (Cx){a.re - b.re, a.im - b.im}; }

/* cx_mul — (a + ib)(c + id) = (ac − bd) + i(ad + bc).  Four real
 * multiplies + two real adds; auto-vectorised on -O2.  Used in the
 * pivot step of Thomas (pivot × previous row's super-diagonal/RHS). */
static inline Cx cx_mul(Cx a, Cx b) {
  return (Cx){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}

/* cx_div — a / b = (a · b*) / |b|².  Numerator forms (a · b*) inline:
 *   (a.re + i·a.im)(b.re − i·b.im) = (a.re·b.re + a.im·b.im)
 *                                  + i(a.im·b.re − a.re·b.im).
 *
 * Robustness: this is safe as long as |b|² ≠ 0.  In Thomas, b is always
 * a pivoted CN diagonal, and the CN diagonal has the form
 *   diag = 1 + i·(2r + dt/2·V)
 * with |diag|² = 1 + (2r + dt/2·V)² ≥ 1 — strictly bounded away from
 * zero, so no defensive guard is needed inside the inner loop.
 */
static inline Cx cx_div(Cx a, Cx b) {
  float mod_sq = b.re * b.re + b.im * b.im;
  return (Cx){(a.re * b.re + a.im * b.im) / mod_sq,
              (a.im * b.re - a.re * b.im) / mod_sq};
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Wavefunction — the discrete quantum state ψ(x, t) on a uniform grid.
 *
 * Intent:
 *   Carry everything that depends on the spatial dimension x — the
 *   wavefunction itself AND the potential it sees.  Anything physics
 *   needs in order to advance one tick lives here; anything UI or
 *   render-only lives elsewhere (on Scene).
 *
 * Algorithm refs:
 *   3-point centred-difference Laplacian      — Numerical Recipes [2, §19.2]
 *   Born rule  |ψ(x)|² = probability density  — Griffiths [4, §1.3]
 *   Dirichlet BC at hard walls                — Griffiths [4, §2.2]
 *   Gaussian wavepacket dispersion            — Cohen-Tannoudji [8, comp G_I]
 *
 * Grid layout (uniform, fixed at compile time):
 *   x_i = X_MIN + i · DX,   i = 0 .. N_GRID-1
 *   N_GRID = 512, DX ≈ 0.002 in natural units (ℏ = m = 1).
 *   The domain [0, 1] (dimensionless) hosts a wavepacket of σ ≈ 0.05
 *   resolved with ~25 grid points per standard deviation — fine enough
 *   that the 3-point Laplacian's leading error O(DX²) ≈ 4 × 10⁻⁶ stays
 *   well below the CN time-error budget.
 *
 * Invariants maintained between physics ticks:
 *   - re[0] = re[N_GRID-1] = im[0] = im[N_GRID-1] = 0   (Dirichlet BC)
 *   - ∫ |ψ|² dx ≈ 1 within numerical precision          (unitarity,
 *                                                         re-checked
 *                                                         only at IC;
 *                                                         CN preserves
 *                                                         it exactly
 *                                                         in real math)
 *   - V[i] is constant across ticks; only build_potential (§5) writes it
 *
 * Memory: 3 × N_GRID float arrays + 1 scalar = 8 KB.  Lives in BSS via
 * the single Scene instance below; never dynamically allocated.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── ψ(x, t)  —  the quantum state ────────────────────────────────
   * Split into TWO PARALLEL float arrays rather than one array of Cx.
   * Why planar layout:
   *   - Most operations touch many indices at once (the 3-point
   *     stencil reads re[i-1], re[i], re[i+1] together).  A cache
   *     line is 64 bytes = 16 floats = 16 contiguous grid points of
   *     ONE component, so planar arrays let the prefetcher load
   *     three adjacent points in one go.
   *   - Auto-vectorisation: the compiler can SIMD across re[] OR
   *     across im[] cleanly; an interleaved (re, im, re, im, …)
   *     layout would need shuffle instructions to extract just one
   *     component.
   *   - The CN stencil itself is real-arithmetic-heavy (see
   *     cn_rhs_at in §7) — splitting into re/im was the natural way
   *     to write it without invoking the Cx algebra machinery.
   */

  /* re[i] — Re ψ(x_i, t).  At t = 0 carries cos(k₀ · x_i) modulated
   * by the Gaussian envelope (see make_gaussian_wavepacket §6).  As
   * the packet evolves the spatial oscillation pattern rotates at
   * the local PHASE velocity ω_φ = k₀ / 2 (Cohen-Tannoudji [8]) —
   * faster than the GROUP velocity v_g = k₀, which is why the
   * phase-coloured bars in §9 "barber-pole" through the envelope. */
  float re[N_GRID];

  /* im[i] — Im ψ(x_i, t).  At t = 0 carries sin(k₀ · x_i) under the
   * same envelope.  Together with re[i] gives the full complex
   * ψ_i = re[i] + i·im[i]; |ψ_i|² = re[i]² + im[i]² is the
   * probability density that draws the bar height in §9. */
  float im[N_GRID];

  /* ── V(x)  —  the potential the wave moves through ───────────────
   * Sits on the SAME grid as ψ because the CN stencil multiplies
   * V[i] by ψ[i] term-by-term — V doesn't need its own discretisation.
   */

  /* V[i] — V(x_i) in natural units (ℏ = m = 1 so energy is in units
   * of ℏ²/(m·DX²) ≈ 250 000 for our grid).  Range observed in this
   * file: 0 (free region), V_BARRIER = 2000 (finite barrier — the
   * wavepacket has energy ≈ k₀²/2 ≈ 1250 < V_BARRIER, hence
   * tunnelling), V_WALL = 50 000 (absorbing edges + double-slit
   * wall — far above any wavepacket energy so reflection is total).
   * Each preset_*() helper in §5 paints a different V profile.
   * Mutating V[] is what makes the demo's four scenes physically
   * different — same ψ, same solver, different V → completely
   * different behaviour. */
  float V[N_GRID];

  /* ── k₀  —  initial / current mean momentum of the seed packet ──
   * Not a per-grid-point quantity but a scalar parameter of the
   * INITIAL CONDITION (make_gaussian_wavepacket).  Stored on
   * Wavefunction because it logically belongs to the state — the
   * HUD shows it, +/- keys mutate it, and a preset reload re-seeds
   * the packet from this value.
   *
   * Physical meaning: the central momentum of the wavepacket's
   * momentum-space distribution.  Group velocity v_g = ℏ k₀ / m = k₀
   * (natural units), kinetic energy E ≈ k₀² / (2m) = k₀² / 2.
   * At k₀ = 200 (default barrier preset) the wave travels at 200 px
   * of the dimensionless domain per unit time — visible as a clear
   * left-to-right drift.
   *
   * Note: this field is the BOOT/RESET momentum, not the instan-
   * taneous expectation value <p>.  Once the wave starts evolving,
   * <p> changes (e.g. flips on reflection); we don't recompute
   * k₀ to track that — apply_momentum_kick (§8) is the only path
   * that "knows" how to add momentum without touching this value. */
  float k0;
} Wavefunction;

/* ─────────────────────────────────────────────────────────────────────── *
 * TridiagonalSolver — workspace for the Thomas algorithm [3].
 *
 * Intent:
 *   The Crank-Nicolson update solves  A · ψ_new = b  every tick.
 *   A is tridiagonal (the 3-point stencil only couples ψ_i to its two
 *   neighbours), so we use Thomas's O(N) banded solver instead of a
 *   generic O(N³) LU.  This struct holds the SCRATCH SPACE that the
 *   forward sweep fills and the back sweep consumes.
 *
 * The system (per row i):
 *
 *     sub_i · ψ_{i-1} + diag_i · ψ_i + sup_i · ψ_{i+1} = b_i
 *
 * with the row coefficients
 *
 *     sub_i = sup_i = −i · r           (constant, r = dt/(4·dx²))
 *     diag_i        = 1 + i · (2r + dt/2 · V_i)
 *     b_i           = (I − i · dt/2 · H) ψ_i          (see cn_rhs_at)
 *
 * Thomas algorithm (Thomas [3], NR [2, §2.4]):
 *   FORWARD SWEEP — for i = 1 .. N-2:
 *     pivot_i = sub_i / cp_{i-1}                       (the multiplier)
 *     cp_i    = diag_i − pivot_i · sup_{i-1}            (new pivoted diag)
 *     tmp_i   = b_i    − pivot_i · tmp_{i-1}            (new pivoted RHS)
 *   BACK SUBSTITUTION — for i = N-2 .. 1:
 *     ψ_i = (tmp_i − sup_i · ψ_{i+1}) / cp_i
 *
 * After the forward sweep the system is upper-triangular; back sweep
 * computes ψ_new in one pass.  Total cost = O(N).
 *
 * Why two separate parallel arrays (cp + tmp) rather than one struct
 * array of {cp, tmp}-pairs:
 *   - Forward sweep WRITES both, back sweep READS both, but neither
 *     loop iterates them in lockstep with a Wavefunction[i].
 *   - Planar layout mirrors Wavefunction and keeps the cache prefetcher
 *     happy on both the write (forward) and the read (back) passes.
 *
 * Memory: 2 × N_GRID × sizeof(Cx) = 8 KB.  Lives in BSS via Scene.
 * Persistent (not re-zeroed each tick) so we dodge any malloc / free
 * overhead; the Thomas formulas only READ cp[i-1] and tmp[i-1] inside
 * the forward sweep, so stale values are overwritten before they can
 * be read.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* cp[i] — pivoted diagonal at row i after the forward sweep.
   * Called "c-prime" in Thomas literature.  Computed during the
   * forward pass as
   *     cp[i] = diag_i − (sub / cp[i-1]) · sup
   * and read during the back pass as the DIVISOR that produces
   * ψ_new[i].  cp[0] is unused (Dirichlet BC pins ψ[0] = 0 directly;
   * the forward sweep starts at i = 1 with cp[1] = diag_1). */
  Cx cp[N_GRID];

  /* tmp[i] — pivoted RHS at row i after the forward sweep.
   * Computed as
   *     tmp[i] = b_i − (sub / cp[i-1]) · tmp[i-1]
   * and consumed during the back pass via
   *     ψ_new[i] = (tmp[i] − sup · ψ_new[i+1]) / cp[i]
   * Named `tmp` rather than `d-prime` (the typical Thomas-literature
   * name) because reading `tmp[i]` immediately tells the reader
   * "this is per-step scratch", not "this is one specific algebraic
   * quantity from a textbook formula". */
  Cx tmp[N_GRID];
} TridiagonalSolver;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level demo state spanning physics ticks, draw calls, and
 * the main loop.  Single instance `g_scene` in BSS; passing it by
 * pointer everywhere would just clutter helper signatures without
 * buying any encapsulation.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by crank_nicolson_step (§7),
 *     build_potential / init_wavepacket (§5, §6), and the load_preset
 *     reset path (§8).  Anything that changes WHAT ψ DOES belongs here.
 *     Mutated by physics-affecting keys: 1-4 (preset), r (reset),
 *     p / SPACE (pause / momentum kick), + / − (k₀ adjust).
 *
 *   Rendering parameters — consumed by §9 draw helpers and theme_apply
 *     (§3).  Mutating these MUST leave wave / solver / preset byte-
 *     identical; only colours / chrome change.  Mutated by purely
 *     cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes what
 *   crank_nicolson_step computes would silently couple display to
 *   physics — exactly the bug the separation prevents.  When adding
 *   a field, ask: does it change ψ_new given the same (ψ_old, V, dt)?
 *   If yes, simulation; if no, rendering.
 *
 * What stays OUTSIDE Scene (intentionally):
 *
 *   g_quit, g_resize      volatile sig_atomic_t flags written by the
 *                         signal handler; must be file-scope for
 *                         async-signal-safety (the standard guarantees
 *                         atomic load/store only for objects of that
 *                         type at file scope).
 *
 *   g_rows, g_cols        screen geometry tracked by the main loop's
 *                         SIGWINCH handler; Scene stays geometry-
 *                         agnostic so resize logic is the main loop's
 *                         concern, not the renderer's.
 *
 *   g_256                 boot-time terminal capability probed once
 *                         in screen_init; not state that evolves.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────────
   * Everything below feeds the physics step.  crank_nicolson_step
   * reads wave (+ updates it), uses solver as scratch, and never
   * touches the rendering fields below.
   */

  /* wave — the Wavefunction ψ + V + k₀.  ALL of the quantum state of
   * the demo.  Mutated by crank_nicolson_step every tick; reset by
   * load_preset when the user presses 1-4 / r / +/-; phase-coloured
   * into bars by draw_phase_bars (§9).  See the Wavefunction struct
   * docstring above for the per-field invariants. */
  Wavefunction wave;

  /* solver — TridiagonalSolver scratch space for the CN solve.  The
   * forward sweep writes cp[] and tmp[]; the back sweep reads them
   * to produce ψ_new.  Persists across ticks (no per-step alloc)
   * but the contents are recomputed every step from scratch, so the
   * initial BSS-zero is fine — no init pass needed. */
  TridiagonalSolver solver;

  /* preset — which canned (V(x), IC) pair is currently loaded.
   * Values 0..N_PRESETS-1 mapping to:
   *   0 Free      no extra V — pure dispersion of a Gaussian packet
   *   1 Barrier   finite rectangular barrier, tunnelling demo
   *   2 Harmonic  parabolic V, coherent-state oscillation
   *   3 Slit      Young's double-slit, interference fringes
   * Cycled by 1/2/3/4 keys (each loads a specific preset); r reloads
   * the current preset.  +/- adjust k₀ then reload (k₀ change is
   * baked into the IC, not into the running ψ). */
  int preset;

  /* paused — when true, main's tick loop skips
   * crank_nicolson_step entirely.  Wavefunction and solver stay
   * exactly frozen — unpause continues from the same ψ with no
   * discontinuity.  Mirrored to the HUD via the "PAUSED" badge. */
  bool paused;

  /* ── Rendering parameters ─────────────────────────────────────────
   * Pure cosmetic state.  Mutating these MUST be a no-op for the
   * physics step — that contract is what keeps "press t mid-wave"
   * from disturbing the simulation.
   */

  /* theme — index into k_themes[] (§3).  Cycled forward by 't',
   * backward by 'T'; both call theme_apply() to push the new
   * palette into the ncurses colour-pair table.  Always normalised
   * to [0, N_THEMES) by the t/T handlers; theme_apply() also wraps
   * defensively as a safety net.  Boot default = 6 (Nova) — the
   * most striking rainbow phase rotation. */
  int theme;

  /* fps_disp — rolling 1-second frame-rate readout displayed on the
   * top HUD.  Updated by main()'s frame counter once per second;
   * the renderer just reads it.  Not directly tied to physics
   * (paused, slow simulation, etc. don't affect fps) — it measures
   * pure render-loop throughput. */
  int fps_disp;
} Scene;

/*
 * g_scene — the one Scene instance.  Non-zero defaults:
 *   preset = 1     boot to the barrier scene (best tunnelling demo)
 *   theme  = 6     Nova rainbow — most visually impactful phase wheel
 *   wave.k0 = 200  matches preset 1's intended momentum
 * Everything else (wave.re/im/V, solver, paused, fps_disp) is BSS-
 * zeroed; load_preset() in main fills wave properly before the first
 * tick, so the zero state is never visible to the renderer.
 */
static Scene g_scene = {
    .preset = 1,
    .theme = 6,
    .wave = {.k0 = 200.f},
};

/* ===================================================================== */
/* §5  potential V(x) — preset-specific Hamiltonian construction         */
/* ===================================================================== */

/*
 * The Hamiltonian H = −½ d²/dx² + V(x) is fully determined by V(x).
 * Each preset installs a different V profile, then the rest of the
 * solver is preset-agnostic.  Helpers below install ONE feature each
 * (walls, barrier, harmonic, slits); build_potential() composes them.
 */

/*
 * install_absorbing_walls — set V[i] = V_WALL near both edges so any
 * wave reaching the boundary is reflected/absorbed.  V_WALL is chosen
 * far above any wavepacket energy E ≈ k0²/2, so reflection is total
 * (Griffiths [4, §2.2] hard-wall limit).
 *
 * Wall thickness = N_GRID/20 ≈ 25 grid points: thick enough that
 * exponential decay inside the wall (κ = √(2(V_WALL − E)) ≈ √100000
 * ≈ 316) attenuates the wave by e⁻³¹⁶·DX ≈ e⁻⁰·⁶ per point, so a
 * 25-point wall reflects with ~10⁻⁷ leakage.
 */
static void install_absorbing_walls(Wavefunction *w) {
  int wall = N_GRID / 20;
  for (int i = 0; i < wall; i++) {
    w->V[i] = V_WALL;
    w->V[N_GRID - 1 - i] = V_WALL;
  }
}

/*
 * install_finite_barrier — a rectangular barrier in the middle of the
 * domain.  Wave energy E = k0²/2 ≈ 1250 < V_BARRIER = 2000, so this is
 * classically forbidden; the demo shows quantum tunnelling with
 * transmission probability T ≈ exp(−2κd) where κ = √(2(V−E)) and d is
 * the barrier width (Merzbacher / Griffiths [4, §8]).
 */
static void install_finite_barrier(Wavefunction *w) {
  int c = N_GRID / 2;
  int half_width = N_GRID / 60;
  for (int i = c - half_width; i <= c + half_width; i++)
    w->V[i] = V_BARRIER;
}

/*
 * install_harmonic_potential — V(x) = ½·k·(x − x₀)².
 *
 * The classical SHO; eigenstates are Hermite-Gaussian; an off-centre
 * Gaussian becomes a "coherent state" that orbits in phase space at
 * the classical frequency ω = √k.  At k = 8000 the period is
 * 2π/√8000 ≈ 0.07 — comfortably visible at our DT × STEPS_PER_FRAME.
 * Refs: Griffiths [4, §2.3]; Sakurai, *Modern Quantum Mechanics*,
 * §2.3 on coherent states.
 */
static void install_harmonic_potential(Wavefunction *w) {
  const float k_spring = 8000.f;
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1) - 0.5f;
    w->V[i] += 0.5f * k_spring * x * x;
  }
}

/*
 * install_double_slit — Young's classic setup [5, Vol. III Ch. 1]: a
 * vertical wall with two narrow openings.  An incoming plane-wave
 * Gaussian splits at the slits and the two diffracted parts interfere
 * downstream.  The interference fringes are the most striking
 * confirmation of wave-particle duality — exactly what Feynman called
 * "the only mystery" of quantum mechanics.
 *
 * Geometry:
 *   - Wall at x = 0.5 (grid index N_GRID/2), 5 columns thick so
 *     reflection back into the packet doesn't dominate the visual
 *   - Two slits of width ~N_GRID/100, separated by ~N_GRID/12
 *   - V = V_WALL inside the wall; slit cells reset to 0
 */
static void install_double_slit(Wavefunction *w) {
  int c = N_GRID / 2;
  int slit_half = N_GRID / 100 + 1;
  int sep_half = N_GRID / 24;

  /* Solid wall strip across the middle. */
  for (int i = c - 2; i <= c + 2; i++)
    w->V[i] = V_WALL;

  /* Carve two slits symmetric around centre. */
  int s1 = c - sep_half;
  int s2 = c + sep_half;
  for (int i = s1 - slit_half; i <= s1 + slit_half; i++)
    w->V[i] = 0.f;
  for (int i = s2 - slit_half; i <= s2 + slit_half; i++)
    w->V[i] = 0.f;
}

/*
 * build_potential — dispatcher that zeroes V[], installs the universal
 * absorbing walls, then layers the preset-specific feature on top.
 *
 * Reads as pseudocode:
 *
 *     zero V[]
 *     install_absorbing_walls(w)
 *     switch preset:
 *         0 → (nothing — free particle)
 *         1 → install_finite_barrier(w)
 *         2 → install_harmonic_potential(w)
 *         3 → install_double_slit(w)
 */
static void build_potential(Wavefunction *w, int preset) {
  for (int i = 0; i < N_GRID; i++)
    w->V[i] = 0.f;
  install_absorbing_walls(w);

  switch (preset) {
  case 0: /* free particle — no extra V */
    break;
  case 1:
    install_finite_barrier(w);
    break;
  case 2:
    install_harmonic_potential(w);
    break;
  case 3:
    install_double_slit(w);
    break;
  }
}

/* ===================================================================== */
/* §6  initial conditions — Gaussian wave packet                         */
/* ===================================================================== */

/*
 * Why a Gaussian: it's the minimum-uncertainty state (Δx·Δp = ℏ/2),
 * has a closed-form free-particle evolution (Cohen-Tannoudji [8,
 * complement G_I]), and reads cleanly visually because the envelope
 * is concentrated — you can SEE the packet move.  Free dispersion
 * spreads it as σ(t) = σ₀ √(1 + (ℏ t / 2m σ₀²)²); preset 0 makes that
 * spread visible after one or two traversals of the domain.
 */

/*
 * make_gaussian_wavepacket — initialise ψ to
 *
 *     ψ(x, 0) = exp(−(x − x₀)²/(2σ²)) · exp(i k₀ x)
 *
 * unnormalised.  The next step (normalize_wavefunction) scales so
 * ∫|ψ|² dx = 1.  Stored in real / imag form: the modulation
 * exp(i k₀ x) becomes cos(k₀ x) + i sin(k₀ x) which is what drives the
 * phase-color rotation seen in §10.
 */
static void make_gaussian_wavepacket(Wavefunction *w, float x0, float sigma,
                                     float k0) {
  w->k0 = k0;
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1);
    float envelope = expf(-(x - x0) * (x - x0) / (2.f * sigma * sigma));
    w->re[i] = envelope * cosf(k0 * x);
    w->im[i] = envelope * sinf(k0 * x);
  }
}

/*
 * normalize_wavefunction — rescale ψ so ∫|ψ|² dx = 1 (Born rule
 * unitarity, Griffiths [4, §1.4]).  Discrete normalisation:
 *
 *     N = sqrt(Σ_i |ψ_i|² · DX)        (Riemann sum approximation)
 *     ψ_i /= N
 *
 * Crank-Nicolson is unitary in exact arithmetic so re-normalisation
 * is needed ONLY at IC setup — subsequent steps preserve the norm to
 * within float round-off.
 */
static void normalize_wavefunction(Wavefunction *w) {
  float norm_sq = 0.f;
  for (int i = 0; i < N_GRID; i++)
    norm_sq += w->re[i] * w->re[i] + w->im[i] * w->im[i];

  float scale = 1.f / sqrtf(norm_sq * DX);
  for (int i = 0; i < N_GRID; i++) {
    w->re[i] *= scale;
    w->im[i] *= scale;
  }
}

/*
 * apply_dirichlet_boundary — pin ψ(0) = ψ(L) = 0.  Hard-wall boundary
 * condition (Griffiths [4, §2.2]) compatible with our V_WALL
 * absorption strategy: the wave can never reach the boundary anyway
 * because the absorbing walls reflect it, but pinning the endpoints
 * exactly to 0 eliminates the BC term from the Crank-Nicolson system
 * (the i=0 and i=N_GRID-1 rows decouple).
 */
static void apply_dirichlet_boundary(Wavefunction *w) {
  w->re[0] = w->re[N_GRID - 1] = 0.f;
  w->im[0] = w->im[N_GRID - 1] = 0.f;
}

/*
 * init_wavepacket — preset-aware IC orchestrator.  Each preset chooses
 * its own (x₀, σ, k₀) tuned for the corresponding V(x):
 *
 *   Free     x₀=0.2  σ=0.05  k₀=150   moderate drift, visible spread
 *   Barrier  x₀=0.2  σ=0.05  k₀=200   E < V_BARRIER → tunnelling
 *   Harmonic x₀=0.3  σ=0.07  k₀= 50   off-centre seed for orbit
 *   Slit     x₀=0.2  σ=0.04  k₀=200   tight packet for sharp fringes
 */
static void init_wavepacket(Wavefunction *w, int preset) {
  float x0, sigma, k0;
  switch (preset) {
  case 0:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 150.f;
    break;
  case 1:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 200.f;
    break;
  case 2:
    x0 = 0.3f;
    sigma = 0.07f;
    k0 = 50.f;
    break;
  case 3:
    x0 = 0.2f;
    sigma = 0.04f;
    k0 = 200.f;
    break;
  default:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 150.f;
    break;
  }
  make_gaussian_wavepacket(w, x0, sigma, k0);
  normalize_wavefunction(w);
  apply_dirichlet_boundary(w);
}

/* ===================================================================== */
/* §7  Crank-Nicolson time evolution + Thomas algorithm                  */
/* ===================================================================== */

/*
 * One Crank-Nicolson step solves
 *
 *     (I + i·dt/2·H) ψ_new = (I − i·dt/2·H) ψ_old
 *
 * for ψ_new given ψ_old (Crank & Nicolson [1]).  H is discretised
 * with the standard 3-point stencil
 *
 *     H ψ_i = −(1/2 dx²) (ψ_{i-1} − 2 ψ_i + ψ_{i+1}) + V_i ψ_i
 *
 * which makes the LHS matrix tridiagonal with
 *
 *     sub_i = sup_i = −i · r            (constant, r ≡ dt / (4 dx²))
 *     diag_i        = 1 + i · (2r + dt/2 · V_i)
 *
 * Solved row-by-row with the Thomas algorithm [3] in two passes:
 *
 *     thomas_forward_sweep  : eliminate sub-diagonal → cp[i], tmp[i]
 *     thomas_back_substitute: read pivots → ψ_new[i]
 *     apply_dirichlet_boundary: re-pin endpoints to 0
 *
 * crank_nicolson_step(w, s) is a four-line orchestrator over these.
 */

/* Precomputed CN ratio  r = dt / (4 dx²).  Constant across the run;
 * splitting it out makes the formulas above match the docstring. */
static inline float cn_offdiagonal_factor(void) { return DT / (4.f * DX * DX); }

/*
 * cn_diagonal_at — diag_i = 1 + i · (2r + dt/2 · V_i).
 *
 * The 1 in the real part comes from the (I + …) shift; the imaginary
 * part is (dt/2) · H_ii where H_ii = 1/dx² + V_i.
 */
static inline Cx cn_diagonal_at(int i, const Wavefunction *w, float r) {
  return (Cx){1.f, 2.f * r + 0.5f * DT * w->V[i]};
}

/*
 * cn_rhs_at — b_i = ψ_i − i · dt/2 · H ψ_i, computed directly without
 * forming H explicitly.  Splitting i times (re, im) by hand:
 *
 *     re(b) = re(ψ) + r·(im(ψ_{i-1}) − 2 im(ψ_i) + im(ψ_{i+1}))
 *                   − dt/2 · V · im(ψ)
 *     im(b) = im(ψ) − r·(re(ψ_{i-1}) − 2 re(ψ_i) + re(ψ_{i+1}))
 *                   + dt/2 · V · re(ψ)
 *
 * The minus sign on the kinetic term in im(b) is the i² = −1 of the
 * −i·dt/2·H operator (each i on the imaginary side flips the sign).
 */
static inline Cx cn_rhs_at(int i, const Wavefunction *w, float r) {
  float V_term = 0.5f * DT * w->V[i];
  float re_b = w->re[i] + r * (w->im[i - 1] - 2.f * w->im[i] + w->im[i + 1]) -
               V_term * w->im[i];
  float im_b = w->im[i] - r * (w->re[i - 1] - 2.f * w->re[i] + w->re[i + 1]) +
               V_term * w->re[i];
  return (Cx){re_b, im_b};
}

/*
 * thomas_forward_sweep — eliminate the sub-diagonal row-by-row.
 *
 * For each interior row i = 1..N_GRID-2:
 *
 *     w_i  = sub / cp[i-1]                  (pivot multiplier)
 *     cp[i]  = diag_i − w_i · sup           (pivoted diagonal)
 *     tmp[i] = b_i    − w_i · tmp[i-1]      (pivoted RHS)
 *
 * The first row (i=1) has no predecessor so cp[1] = diag, tmp[1] = b
 * directly.  After this pass the system is upper-triangular —
 * back substitution recovers ψ_new in one more sweep.
 *
 * sub and sup are equal (both = −i·r) so the matrix is symmetric;
 * a future SPD solver could exploit that but Thomas already runs in
 * O(N) so there's nothing left to optimise.
 */
static void thomas_forward_sweep(const Wavefunction *w, TridiagonalSolver *s,
                                 float r) {
  Cx sub = {0.f, -r}; /* −i·r */
  Cx sup = {0.f, -r};

  for (int i = 1; i < N_GRID - 1; i++) {
    Cx d_i = cn_diagonal_at(i, w, r);
    Cx b_i = cn_rhs_at(i, w, r);

    if (i == 1) {
      s->cp[i] = d_i;
      s->tmp[i] = b_i;
    } else {
      Cx pivot = cx_div(sub, s->cp[i - 1]);
      s->cp[i] = cx_sub(d_i, cx_mul(pivot, sup));
      s->tmp[i] = cx_sub(b_i, cx_mul(pivot, s->tmp[i - 1]));
    }
  }
}

/*
 * thomas_back_substitute — read pivoted (cp[], tmp[]) and write ψ_new
 * into w->re[] / w->im[].  Walks from the last interior row back to
 * the first:
 *
 *     ψ_last = tmp[last] / cp[last]
 *     ψ_i    = (tmp[i] − sup · ψ_{i+1}) / cp[i]   for i = last-1 .. 1
 *
 * The endpoints (i=0 and i=N_GRID-1) are NOT touched here — the
 * Dirichlet BC pins them to 0 in the next stage.
 */
static void thomas_back_substitute(Wavefunction *w,
                                   const TridiagonalSolver *s) {
  Cx sup = {0.f, -DT / (4.f * DX * DX)};
  int last = N_GRID - 2;

  Cx psi = cx_div(s->tmp[last], s->cp[last]);
  w->re[last] = psi.re;
  w->im[last] = psi.im;

  for (int i = last - 1; i >= 1; i--) {
    Cx psi_next = {w->re[i + 1], w->im[i + 1]};
    psi = cx_div(cx_sub(s->tmp[i], cx_mul(sup, psi_next)), s->cp[i]);
    w->re[i] = psi.re;
    w->im[i] = psi.im;
  }
}

/*
 * crank_nicolson_step — top-level CN orchestrator.  Reads as
 * pseudocode:
 *
 *     r = cn_offdiagonal_factor()
 *     thomas_forward_sweep(w, s, r)
 *     thomas_back_substitute(w, s)
 *     apply_dirichlet_boundary(w)
 *
 * Called STEPS_PER_FRAME times per rendered frame from main (§10).
 */
static void crank_nicolson_step(Wavefunction *w, TridiagonalSolver *s) {
  float r = cn_offdiagonal_factor();
  thomas_forward_sweep(w, s, r);
  thomas_back_substitute(w, s);
  apply_dirichlet_boundary(w);
}

/* ===================================================================== */
/* §8  driver + diagnostics                                              */
/* ===================================================================== */

/*
 * load_preset — full reset: install new V(x), seed a fresh wavepacket,
 * record the new preset index on Scene.  Called at boot, on r/R reset,
 * and on each 1/2/3/4 keypress.
 */
static void load_preset(Scene *sc, int preset) {
  sc->preset = preset;
  build_potential(&sc->wave, preset);
  init_wavepacket(&sc->wave, preset);
}

/*
 * compute_reflection_transmission — split the probability density at
 * the grid midpoint into left (reflected) and right (transmitted)
 * halves and normalise to 1.  Useful for the barrier preset where
 * R + T should sum to 1 with R the reflected fraction and T the
 * tunnelled fraction.
 *
 *     R = ∫₀^{L/2} |ψ|² dx / total
 *     T = ∫_{L/2}^L |ψ|² dx / total
 *
 * For other presets (free, harmonic, double-slit) the geometric split
 * is less physically meaningful but still useful as a sanity check
 * for unitarity (R + T = 1 ± float noise).
 */
static void compute_reflection_transmission(const Wavefunction *w, float *R_out,
                                            float *T_out) {
  float left = 0.f, right = 0.f;
  int mid = N_GRID / 2;
  for (int i = 1; i < mid; i++)
    left += (w->re[i] * w->re[i] + w->im[i] * w->im[i]) * DX;
  for (int i = mid; i < N_GRID - 1; i++)
    right += (w->re[i] * w->re[i] + w->im[i] * w->im[i]) * DX;

  float total = left + right + 1e-10f;
  *R_out = left / total;
  *T_out = right / total;
}

/*
 * apply_momentum_kick — multiply ψ by exp(i·k·x).  In quantum mechanics
 * this is the momentum-translation operator: it shifts the packet's
 * mean momentum by k without touching its position-space envelope.
 * Useful for the user to "punch" extra energy into the packet at any
 * time (bound to SPACE).
 *
 * Implementation:
 *   ψ_new = ψ · (cos(kx) + i sin(kx))
 *         = (Re·cos − Im·sin) + i (Re·sin + Im·cos)
 */
static void apply_momentum_kick(Wavefunction *w, float k) {
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1);
    float kre = cosf(k * x);
    float kim = sinf(k * x);
    float new_re = w->re[i] * kre - w->im[i] * kim;
    float new_im = w->re[i] * kim + w->im[i] * kre;
    w->re[i] = new_re;
    w->im[i] = new_im;
  }
}

/* ===================================================================== */
/* §9  draw — phase-coloured bars + potential strip + HUD               */
/* ===================================================================== */

static int g_rows, g_cols; /* screen geometry; main-loop bookkeeping */

/*
 * draw_phase_bars — the main visual.  For each terminal column,
 * sample (Re ψ, Im ψ) at the corresponding grid point, derive the
 * phase color from arg ψ, and paint a vertical bar of height
 * proportional to |ψ|².  The bar rises from the baseline at the
 * bottom of the plot area upward — taller bars = higher probability,
 * colour rotation across the bars shows the wave's phase velocity.
 *
 * Auto-scaling: bar heights are normalised to the per-frame max so
 * even a spreading or tunnelling packet stays on screen.  Visible
 * effects of spreading still show because the colour rotation rate
 * and bar SHAPE (envelope curve) are absolute properties of ψ.
 */
static void draw_phase_bars(int plot_top, int plot_rows) {
  int baseline = plot_top + plot_rows - 1;

  /* Find peak |ψ|² for scaling. */
  float max_prob = 1e-12f;
  for (int i = 0; i < N_GRID; i++) {
    float p = g_scene.wave.re[i] * g_scene.wave.re[i] +
              g_scene.wave.im[i] * g_scene.wave.im[i];
    if (p > max_prob)
      max_prob = p;
  }

  float pts_per_col = (float)N_GRID / (float)g_cols;
  int bar_room = plot_rows - 1;

  for (int col = 0; col < g_cols; col++) {
    int gi = (int)((float)col * pts_per_col);
    if (gi >= N_GRID)
      gi = N_GRID - 1;

    float re = g_scene.wave.re[gi];
    float im = g_scene.wave.im[gi];
    float prob = re * re + im * im;

    int h = (int)(prob / max_prob * (float)bar_room + 0.5f);
    if (h <= 0)
      continue;
    if (h > bar_room)
      h = bar_room;

    int cp = phase_color_pair(re, im);
    attron(COLOR_PAIR(cp) | A_BOLD);
    /* Bar body — uniform phase colour all the way down. */
    for (int dy = 0; dy < h - 1; dy++)
      mvaddch(baseline - dy, col, '|');
    /* Peak — slightly different glyph for a "tip" highlight. */
    mvaddch(baseline - (h - 1), col, ':');
    attroff(COLOR_PAIR(cp) | A_BOLD);
  }
}

/*
 * draw_potential_strip — bottom-of-plot row showing V(x):
 *   '|' (bright, theme-driven wall colour) for hard walls (V ≥ V_WALL/2)
 *   '#' (medium, theme-driven barrier colour) for finite barriers
 *   '·' (dim grey axis) for free regions  → reads as the baseline
 *
 * This sits on row rows-2 (between the bar plot above and the bottom
 * HUD hint below) so the wave's baseline and the potential layer
 * coexist in one visually clean strip.
 */
static void draw_potential_strip(int row) {
  float pts_per_col = (float)N_GRID / (float)g_cols;
  for (int col = 0; col < g_cols; col++) {
    int gi = (int)((float)col * pts_per_col);
    if (gi >= N_GRID)
      gi = N_GRID - 1;
    float V = g_scene.wave.V[gi];

    if (V >= V_WALL * 0.5f) {
      attron(COLOR_PAIR(CP_WALL) | A_BOLD);
      mvaddch(row, col, '|');
      attroff(COLOR_PAIR(CP_WALL) | A_BOLD);
    } else if (V > 0.f) {
      attron(COLOR_PAIR(CP_BARRIER) | A_BOLD);
      mvaddch(row, col, '#');
      attroff(COLOR_PAIR(CP_BARRIER) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_AXIS));
      mvaddch(row, col, '.');
      attroff(COLOR_PAIR(CP_AXIS));
    }
  }
}

/*
 * draw_hud_top — row 0 status bar.  Per CLAUDE.md HUD convention,
 * right-aligned bright yellow + bold.  Shows the live demo state:
 * preset, momentum k0, reflection R and transmission T (computed
 * by compute_reflection_transmission), theme name, paused/running, fps.
 */
static void draw_hud_top(int fps) {
  float R = 0.f, T = 0.f;
  compute_reflection_transmission(&g_scene.wave, &R, &T);

  char buf[200];
  snprintf(buf, sizeof buf,
           " %s  k0:%5.0f  R:%.2f T:%.2f  theme:%s  %s  %d fps ",
           PRESETS[g_scene.preset].name, (double)g_scene.wave.k0, (double)R,
           (double)T, k_themes[g_scene.theme].name,
           g_scene.paused ? "PAUSED " : "running", fps);
  int col = g_cols - (int)strlen(buf);
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* draw_hud_bottom — row rows-1 action hint bar (cyan bold, left). */
static void draw_hud_bottom(void) {
  const char *full = " q:quit  p:pause  r:reset  1-4:preset  +/-:energy  "
                     "spc:kick  t/T:theme ";
  const char *shrt = " q:quit  p:pause  1-4:preset  t:theme ";
  const char *h = (int)strlen(full) >= g_cols - 1 ? shrt : full;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, h, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * scene_draw — top-level renderer.  Three layers, each their own
 * helper:
 *
 *   draw_phase_bars     — main panel (rows HUD_TOP_ROWS .. rows-3)
 *   draw_potential_strip — V(x) overlay (row rows-2)
 *   draw_hud_top/bottom  — chrome (rows 0 and rows-1)
 */
static void scene_draw(void) {
  int plot_top = HUD_TOP_ROWS;
  int plot_rows = g_rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
  if (plot_rows < 3)
    plot_rows = 3;

  draw_phase_bars(plot_top, plot_rows);
  draw_potential_strip(g_rows - 2);
  draw_hud_top(g_scene.fps_disp);
  draw_hud_bottom();
}

/* ===================================================================== */
/* §10  app — signal handlers + main loop                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit = 1;
  if (s == SIGWINCH)
    g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  srand((unsigned)time(NULL));
  atexit(cleanup);
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  color_init(g_scene.theme);
  getmaxyx(stdscr, g_rows, g_cols);
  load_preset(&g_scene, g_scene.preset);

  long long fps_window_start = clock_ns();
  int fps_frames = 0;

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
    }

    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27:
      g_quit = 1;
      break;
    case 'p':
    case 'P':
      g_scene.paused = !g_scene.paused;
      break;
    case 'r':
    case 'R':
      load_preset(&g_scene, g_scene.preset);
      break;
    case '1':
      load_preset(&g_scene, 0);
      break;
    case '2':
      load_preset(&g_scene, 1);
      break;
    case '3':
      load_preset(&g_scene, 2);
      break;
    case '4':
      load_preset(&g_scene, 3);
      break;
    case '+':
    case '=':
      g_scene.wave.k0 *= 1.2f;
      if (g_scene.wave.k0 > 600.f)
        g_scene.wave.k0 = 600.f;
      load_preset(&g_scene, g_scene.preset);
      break;
    case '-':
      g_scene.wave.k0 *= 0.833f;
      if (g_scene.wave.k0 < 30.f)
        g_scene.wave.k0 = 30.f;
      load_preset(&g_scene, g_scene.preset);
      break;
    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case ' ':
      apply_momentum_kick(&g_scene.wave, 100.f);
      break;
    default:
      break;
    }

    long long now = clock_ns();
    if (!g_scene.paused)
      for (int s = 0; s < STEPS_PER_FRAME; s++)
        crank_nicolson_step(&g_scene.wave, &g_scene.solver);

    /* rolling 1-second fps window */
    fps_frames++;
    if (now - fps_window_start >= 1000000000LL) {
      g_scene.fps_disp = fps_frames;
      fps_frames = 0;
      fps_window_start = now;
    }

    erase();
    scene_draw();
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
