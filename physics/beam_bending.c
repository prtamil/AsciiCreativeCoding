/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * beam_bending.c — Euler-Bernoulli Beam Bending + Vibration Simulator
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  PHYSICS OVERVIEW
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The Euler-Bernoulli beam model assumes:
 *    1. Cross-sections remain plane and perpendicular to the neutral axis.
 *    2. Deflections are small (linear theory — no geometric nonlinearity).
 *    3. Material is linearly elastic (stress proportional to strain).
 *
 *  Governing equation (beam axis = x, transverse deflection = w):
 *
 *      EI · d⁴w/dx⁴ = q(x)
 *
 *  where:
 *    E  = Young's modulus   [Pa]   — material stiffness
 *    I  = second moment of area [m⁴] — cross-section shape resistance
 *    EI = flexural rigidity — the single stiffness parameter that matters
 *    q  = distributed transverse load [N/m]
 *    w  = transverse deflection [m]   (positive = downward here)
 *
 *  Bending moment — what you feel as a beam bends:
 *      M(x) = EI · d²w/dx²    (curvature × stiffness)
 *
 *  Bending stress at distance y from neutral axis:
 *      σ(x,y) = M(x)·y / I
 *  → High |M| = high stress = higher risk of yielding/fracture.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  BOUNDARY CONDITIONS — what makes each beam type unique
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Simply Supported (SS):   w(0)=0, M(0)=0, w(L)=0, M(L)=0
 *    Both ends pinned. Beam can rotate at supports. Moment is zero at
 *    the pins — the support can only push up/down, not apply a couple.
 *    → Symmetric deflection under symmetric loads.
 *    → Max deflection = PL³/(48EI) at midspan for centre point load.
 *
 *  Cantilever (Fixed-Free): w(0)=0, w'(0)=0, M(L)=0, V(L)=0
 *    Left end fully clamped — cannot deflect or rotate. Right end free.
 *    The wall provides both a reaction force AND a reaction moment.
 *    → Largest tip deflection: PL³/(3EI) — 16× more than SS midspan!
 *    → Moment is maximum at the wall and zero at the free tip.
 *
 *  Fixed-Fixed (FF):        w(0)=0, w'(0)=0, w(L)=0, w'(L)=0
 *    Both ends fully clamped. Both ends provide moment reactions.
 *    → Stiffest configuration: midspan deflection PL³/(192EI).
 *    → Hogging moments (tension at top) develop at both clamped ends.
 *    → Sagging moment at midspan. Moment diagram crosses zero twice.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  DYNAMIC MODE — modal superposition
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Free vibration: each mode n has a natural frequency ωₙ and a mode
 *  shape φₙ(x). The response is a sum over all modes:
 *
 *      w(x,t) = Σ qₙ(t) · φₙ(x)
 *
 *  Each modal coordinate qₙ(t) obeys an independent damped oscillator:
 *
 *      q̈ₙ + 2ζωₙ q̇ₙ + ωₙ² qₙ = Fₙ(t)
 *
 *  where ζ = damping ratio, Fₙ = modal load = ∫ q(x)φₙ(x)dx / Mₙ.
 *
 *  Stepped with exact transition matrix (unconditionally stable).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  VISUALIZATION DESIGN
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Main area:  deflected beam shape, curvature shading, support symbols,
 *              load arrows. Character density encodes |M(x)|/M_max.
 *              Stress ramp colours are theme-driven (lo → med → hi).
 *              Dynamic mode adds motion-blur trails behind the live
 *              beam — a 12-frame ring buffer of past shapes drawn as
 *              fading dots so the oscillation envelope reads at a glance.
 *  Side panel: bending moment diagram, horizontal bars ±M(x)/M_max.
 *
 *  HUD layout (canonical CLAUDE.md two-bar + secondary line):
 *    row 0 right (bright yellow + bold)  — live status: P, exag, theme,
 *                                          paused / dynamic, fps.
 *    row 1 left  (bright yellow)         — context: BC name, load name,
 *                                          max deflection, max moment,
 *                                          [VIBRATING] badge.
 *    row rows-1  (bright cyan + bold)    — actions / key reference.
 *
 *  Themes (10 palettes; cycle with t / T):
 *    Matrix, Fire, Oceanic, Neon, Mono, Ice, Nova, Forest, Desert, Eclipse.
 *  Theme only swaps colours — physics, geometry, and HUD layout unchanged.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  REFERENCES
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Five sources, three on the physics and two on the implementation,
 *  cover everything in this file. Inline comments cite them as [n].
 *
 *  [1] Gere, J. M. & Goodno, B. J. — "Mechanics of Materials", 9th ed.,
 *      Cengage Learning (2018).
 *      Foundational Euler-Bernoulli theory: derivation of EI·d²w/dx² = M,
 *      sign conventions, support reactions, deflection of beams (Ch. 9).
 *      Read this first if you want to understand why §4-§5 look the way
 *      they do, before diving into specific formulas.
 *
 *  [2] Young, W. C.; Budynas, R. G.; Sadegh, A. M. — "Roark's Formulas
 *      for Stress and Strain", 8th ed., McGraw-Hill (2011).
 *      Tables 8.1-8.10 give the closed-form M(x) and w(x) for every
 *      BC × load combination implemented in §5. solve_beam is essentially
 *      a transcription of these tables; cross-check any formula here.
 *
 *  [3] Rao, S. S. — "Mechanical Vibrations", 6th ed., Pearson (2017).
 *      Ch. 8 (continuous systems) covers the eigenvalue problem
 *      EI·φ'''' = ρA·ω²·φ, the characteristic equations for each BC,
 *      modal superposition, modal damping, and the step response of a
 *      damped oscillator. Backs everything in §6 (dynamics).
 *
 *  [4] Blevins, R. D. — "Formulas for Natural Frequency and Mode Shape",
 *      Krieger Publishing (1979, reissued 2001).
 *      Definitive tabulated source for the βₙ·L roots and σₙ coefficients
 *      used in dyn_setup. Table 8-1: SS = nπ exactly; Cantilever = 1.87510,
 *      4.69409, 7.85476, 10.99554; FF = 4.73004, 7.85321, 10.99561,
 *      14.13717. If a mode shape ever looks wrong, verify against Blevins.
 *
 *  [5] Ware, C. — "Information Visualization: Perception for Design",
 *      4th ed., Morgan Kaufmann (2020).
 *      Perceptually-ordered colour and luminance ramps (Ch. 4) back the
 *      lo→med→hi stress ramp; quantitative-magnitude encoding by character
 *      density (Ch. 5) backs the " .-~=*#@" glyph ramp; two-colour sign
 *      encoding (Ch. 4) backs the sag/hog moment bars. The 10 themes in
 *      §3 all respect the brightness-ordering principle Ware advocates.
 *
 * Keys:
 *   q/ESC  quit             space   pause/resume    r       reset
 *   d      toggle vibration i / I   impulse kick    g / G   toggle trails
 *   n / p  next / prev BC   l       cycle load
 *   +/-    load P           e / E   deflection exag
 *   z / Z  damping ratio    s / S   time speed
 *   t / T  next / previous theme
 *
 * Boots into dynamic mode with a cantilever + tip load — beam is
 * already vibrating when the demo opens. Cycling BC or load re-excites
 * the modes in place; you don't drop back to static.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/beam_bending.c -o beam_bending
 * -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config   §2 clock    §3 color
 *  §4 beam     §5 solver   §6 dynamics   §7 render   §8 scene
 *  §9 screen   §10 app
 * ─────────────────────────────────────────────────────────────────────
 */

#define _POSIX_C_SOURCE 200809L
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

#define N_NODES 200       /* beam discretisation points                 */
#define N_TRAIL 12        /* past beam shapes kept for motion-blur trail */
#define BEAM_HEIGHT 3     /* visible cross-section height in rows (odd) */
#define PANEL_W 22        /* right-side moment diagram width (cols)     */
#define BEAM_X_MARGIN 4   /* empty cols on each side of beam span       */
#define LOAD_ARROW_H 4    /* rows of load arrow above beam top          */
#define TARGET_FPS 30     /* render frame cap                           */
#define RAMP_SPEED 0.7f   /* load ramp: full load in ~1.4 s             */
#define N_MODES 4         /* eigenmodes used in dynamic superposition   */
#define MODAL_DAMP 0.025f /* ζ — damping ratio (2.5 % of critical)      */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/*
 * Color pairs — role-named so colour choices don't spread through code.
 * Theme array (see §3) drives the data roles (LO/MED/HI/MOM_P/MOM_N/
 * LOAD/SUPP/HDR). HUD/HINT/DIM are fixed across themes — CLAUDE.md
 * HUD convention (bright yellow + bright cyan) so the chrome stays
 * legible regardless of which palette is active.
 */
#define CP_LO 1    /* low  stress zone  — per theme */
#define CP_MED 2   /* mid  stress zone  — per theme */
#define CP_HI 3    /* high stress zone  — per theme */
#define CP_MOM_P 4 /* sagging (positive) moment bar — per theme */
#define CP_MOM_N 5 /* hogging (negative) moment bar — per theme */
#define CP_LOAD 6  /* load arrow / label            — per theme */
#define CP_SUPP 7  /* support symbols               — per theme */
#define CP_HUD 8   /* HUD top status — bright yellow + bold */
#define CP_DIM 9   /* reference lines / dim text    — grey   */
#define CP_HDR 10  /* panel header                  — per theme */
#define CP_HINT 11 /* HUD bottom hint bar — bright cyan + bold */

#define N_THEMES 10

typedef enum { BC_SS = 0, BC_CANT, BC_FF, BC_COUNT } BCType;
typedef enum { LD_CENTER = 0, LD_UDL, LD_OFFSET, LD_COUNT } LDType;

static const char *bc_names[BC_COUNT] = {
    "Simply Supported",
    "Cantilever (Fixed-Free)",
    "Fixed-Fixed",
};
static const char *ld_names[LD_COUNT] = {
    "Center Point Load",
    "Uniform Distributed Load",
    "Offset Point (3/4 span)",
};

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
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color / themes                                                     */
/* ===================================================================== */

/*
 * Theme — eight theme-driven 256-colour cube indices, one per data role.
 *
 *   lo / med / hi   stress ramp; perceptually ordered low → high so a
 *                   glance reads "where is the beam working hardest?".
 *   mom_p / mom_n   sagging / hogging moment bar. Direction is already
 *                   encoded by '>' vs '<' glyphs, so these two need
 *                   only be distinguishable, not opposite-coded.
 *   load            arrow + 'P' label glyph above the load point.
 *   supp            support symbols (pin 'A' / wall '#' / "free").
 *   hdr             panel header (' MOMENT +/- ' and 'sag+ hog-').
 *
 * Brightness safety (CLAUDE.md): every entry sits at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier. 16-23 / 232-239 are
 * forbidden — they vanish on default-bg terminals.
 */
typedef struct {
  const char *name;
  int lo, med, hi;
  int mom_p, mom_n;
  int load;
  int supp;
  int hdr;
} Theme;

static const Theme themes[N_THEMES] = {
    /*  name        lo   med  hi   mp   mn   ld   sp   hd  */
    {"Matrix", 28, 40, 46, 46, 118, 226, 82, 82},        /* cyber green */
    {"Fire", 94, 208, 196, 220, 124, 226, 255, 208},     /* hot reds    */
    {"Oceanic", 24, 39, 159, 51, 129, 226, 195, 45},     /* blues/teal  */
    {"Neon", 93, 201, 51, 51, 201, 226, 255, 213},       /* magenta/cyn */
    {"Mono", 240, 247, 255, 255, 244, 250, 252, 250},    /* grayscale   */
    {"Ice", 153, 159, 195, 195, 141, 117, 255, 159},     /* light blues */
    {"Nova", 129, 213, 226, 226, 201, 220, 255, 213},    /* purple-gold */
    {"Forest", 58, 100, 190, 82, 130, 220, 144, 178},    /* greens/tan  */
    {"Desert", 180, 215, 220, 220, 124, 226, 252, 215},  /* sand/gold   */
    {"Eclipse", 240, 244, 196, 255, 196, 226, 244, 196}, /* dim + crim. */
};

static void color_init(int theme) {
  start_color();
  use_default_colors();
  const Theme *th = &themes[theme];
  if (COLORS >= 256) {
    /* Fixed HUD chrome (theme-independent for legibility). */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
    init_pair(CP_DIM, 240, -1); /* dark grey     */

    /* Theme-driven data roles. */
    init_pair(CP_LO, th->lo, -1);
    init_pair(CP_MED, th->med, -1);
    init_pair(CP_HI, th->hi, -1);
    init_pair(CP_MOM_P, th->mom_p, -1);
    init_pair(CP_MOM_N, th->mom_n, -1);
    init_pair(CP_LOAD, th->load, -1);
    init_pair(CP_SUPP, th->supp, -1);
    init_pair(CP_HDR, th->hdr, -1);
  } else {
    /* 8-colour fallback: ignore theme, keep the original mapping. */
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_DIM, COLOR_WHITE, -1);
    init_pair(CP_LO, COLOR_GREEN, -1);
    init_pair(CP_MED, COLOR_YELLOW, -1);
    init_pair(CP_HI, COLOR_RED, -1);
    init_pair(CP_MOM_P, COLOR_GREEN, -1);
    init_pair(CP_MOM_N, COLOR_MAGENTA, -1);
    init_pair(CP_LOAD, COLOR_YELLOW, -1);
    init_pair(CP_SUPP, COLOR_WHITE, -1);
    init_pair(CP_HDR, COLOR_YELLOW, -1);
  }
}

/* ===================================================================== */
/* §4  beam entity                                                        */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * DynBeam — modal state for the free-vibration solver (§6).
 *
 * Why a separate struct (vs. inlining into Beam):
 *   The modal coordinates, mode shapes, and generalised forces only
 *   exist when dynamic mode is active. Grouping them under one named
 *   type makes the lifecycle explicit — dyn_activate populates it,
 *   dyn_tick_modes advances it, dyn_impulse perturbs it, the 'd' key
 *   flips d->active. A reader scanning Beam sees `DynBeam dyn` and
 *   knows where to look for vibration state without wading through
 *   ~10 loose g_dyn_* fields.
 *
 * What it represents (the derivation in one breath):
 *   For free vibration of an Euler-Bernoulli beam we look for solutions
 *   of the form w(x,t) = Σₙ φₙ(x)·qₙ(t). Substituting into the PDE and
 *   separating variables splits the problem in two:
 *
 *       EI·φ''''  = ρA·ω²·φ          (spatial eigenvalue ODE)
 *       q̈ₙ + 2ζωₙq̇ₙ + ωₙ²qₙ = Fₙ(t)  (per-mode temporal ODE)
 *
 *   The spatial ODE has a four-parameter general solution:
 *
 *       φₙ(x) = C₁cosh(βₙx) + C₂sinh(βₙx) + C₃cos(βₙx) + C₄sin(βₙx)
 *
 *   where β⁴ = ρA·ω²/(EI). Applying the four boundary conditions of the
 *   chosen BC reduces the system to a single characteristic equation in
 *   βL whose roots tabulate the eigenvalues.
 *
 * Tabulated βₙ·L roots (ρA = EI = 1 → ωₙ = βₙ²):
 *
 *     SS:    βₙL = nπ            (exact — char. eq. sin(βL) = 0)
 *     Cant:  cos(βL)·cosh(βL) + 1 = 0
 *            βₙL = 1.87510, 4.69409, 7.85476, 10.99554
 *     FF:    cos(βL)·cosh(βL) - 1 = 0
 *            βₙL = 4.73004, 7.85321, 10.99561, 14.13717
 *
 *   dyn_setup hard-codes these. Source: Blevins Table 8-1 [4].
 *
 * Numerical-precision note (why dyn_setup uses double):
 *   High-order Cantilever / FF mode shapes contain cosh(βₙL) terms that
 *   grow as e^(βₙL)/2 — by mode 4 cosh ≈ 3 × 10⁴. The mode-shape formula
 *   subtracts nearly equal cosh and σₙ·sinh terms; in float (24-bit
 *   mantissa) this destroys 5+ significant digits and the shape collapses
 *   to noise. Double (53-bit) survives the cancellation. The final shape
 *   is stored as float — it's only read by the renderer, not differentiated.
 *
 * Algorithm refs (header REFERENCES):
 *   Eigenvalue problem + modal superposition   — Rao Ch. 8 [3]
 *   Tabulated β·L roots and σₙ coefficients    — Blevins Table 8-1 [4]
 *   Exact transition matrix (damped oscillator) — Rao Ch. 2 [3]
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Pre-computed at dyn_activate; constant for the whole run ────── */
  float phi[N_MODES][N_NODES]; /* mode shape φₙ(xᵢ) sampled once at the
                                * N_NODES stations; the tick loop just
                                * forms qₙ·φₙ — no trig per frame.     */
  float omega[N_MODES];        /* natural frequency ωₙ = βₙ²
                                * (EI = ρA = 1 in normalised units)    */
  float modal_mass[N_MODES]; /* Mₙ = ρA·∫₀ᴸφₙ²dx — trapezoidal in
                              * dyn_setup. Normalises Fₙ so each
                              * mode's equation is q̈+2ζωq̇+ω²q=F.    */
  float Fn[N_MODES];         /* pre-divided modal force =
                              * (1/Mₙ)·∫q(x)·φₙ(x)dx. Recomputed in
                              * dyn_load when load magnitude changes;
                              * otherwise constant. Storing the divide
                              * here keeps dyn_tick_modes branch-free.*/

  /* ── Time-varying state advanced every sim step ──────────────────── */
  float q[N_MODES];    /* modal coordinate qₙ(t) — the live DOF
                        * the integrator advances.            */
  float qdot[N_MODES]; /* modal velocity q̇ₙ(t) — kept so the
                        * exact-step transition matrix has
                        * both (q, q̇) state to evolve.        */

  /* ── Display / lifecycle ─────────────────────────────────────────── */
  float ref_max_w; /* visual y-scale frozen at activation
                    * (= static peak |w| from solve_beam).
                    * Only grows, never shrinks (see
                    * dyn_tick_modes) → no scale jitter
                    * as the beam crosses zero deflection.*/
  bool active;     /* true  → scene_tick runs dyn_tick_modes
                    * false → solve_beam (static) drives w[]
                    * Toggled by 'd' key.                  */

  /* ── Motion-blur trail (read by §7 render_trails) ────────────────── */
  float trail_w[N_TRAIL][N_NODES]; /* ring buffer of past w[] snapshots
                                    * pushed by dyn_tick_modes; render_
                                    * trails reads back from head-1, -2,
                                    * ... so older frames fade sparser. */
  int trail_head;                  /* next write slot ∈ [0, N_TRAIL); wraps
                                    * via (head + 1) % N_TRAIL.            */
} DynBeam;

/* ─────────────────────────────────────────────────────────────────────── *
 * Beam — complete simulator state for one Euler-Bernoulli beam.
 *
 * Why L = EI = 1 (the normalisation choice):
 *   The governing equation EI·d⁴w/dx⁴ = q(x) becomes dimensionless when
 *   we pick L = EI = 1. Closed-form formulas in §5 then read directly
 *   without unit baggage — e.g. cantilever tip deflection is just "P/3"
 *   instead of "PL³/(3EI)". The screen never shows physical units anyway
 *   (only a visual exaggeration), so the loss of dimensionality costs
 *   nothing in clarity and buys readable formulas in solve_beam.
 *
 * Why fixed-grid sampling (N_NODES = 200):
 *   The closed-form solutions are continuous functions of x. We sample
 *   them at N_NODES equally-spaced stations so the renderer can map
 *   beam-node → screen-column with one integer divide (node_col in §7).
 *   Re-evaluating the closed-form at each screen column instead would
 *   couple the renderer to the analytic formulas and break the §5 / §7
 *   boundary that lets us swap solvers (e.g. dyn_tick_modes overwrites
 *   the same w[] without touching anything else).
 *
 * Deflection convention (positive = downward):
 *   w[i] is stored in normalised L-units (same as x[i]). The renderer
 *   applies `exag` purely as a visual multiplier — without it, even
 *   PL³/(3EI) for a cantilever lives within a single character cell.
 *   The physics never sees `exag`; it is consumed only in defl_row (§7).
 *
 * Field groups (mirrored by the struct layout below):
 *   Discrete samples — w, M, x: closed-form values at each node.
 *   Beam constants   — L, EI, P, exag (P/exag interactive; L/EI fixed).
 *   Time-varying     — load_anim, damping_ratio, paused.
 *   Configuration    — bc, load: drive the §5 / §6 dispatch.
 *   Solver outputs   — max_deflection, max_moment: renderer normalises
 *                      against these for stress-band colouring + bars.
 *   Dynamic state    — nested DynBeam, only meaningful when dyn.active.
 *
 * Algorithm refs (header REFERENCES):
 *   Euler-Bernoulli foundation (EI·w'' = M, sign conventions) — Gere [1]
 *   Closed-form M(x), w(x) per BC × load type                — Roark [2]
 *   Modal superposition (dyn sub-state)                      — Rao [3]
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Discrete samples of the continuous beam (i ∈ [0, N_NODES-1]) ─ */
  float w[N_NODES]; /* deflection at xᵢ
                     * (positive = downward, L-units)    */
  float M[N_NODES]; /* bending moment at xᵢ (sagging = +)*/
  float x[N_NODES]; /* xᵢ = i/(N_NODES-1) ∈ [0,1]; cached
                     * once at init so the §5 inner loop
                     * doesn't re-divide every node.     */

  /* ── Beam constants (physics + one cosmetic) ─────────────────────── */
  float L;    /* beam length — fixed at 1.0 (norm.)*/
  float EI;   /* flexural rigidity — fixed at 1.0  */
  float P;    /* load magnitude — interactive (+/-)
               * Range 0.05 – 100; multiplied by
               * load_anim before reaching M / w.  */
  float exag; /* render-only visual multiplier
               * (e / E keys, 0.5×–20×). Strictly
               * cosmetic — never enters physics.  */

  /* ── Time-varying / interactive state ────────────────────────────── */
  float load_anim;     /* ∈ [0,1]. Static mode: ramps in over
                        * RAMP_SPEED seconds so a fresh load
                        * grows in instead of snapping.
                        * Dynamic mode: dyn_activate clamps
                        * to 1.0 — modes see a step input.  */
  float damping_ratio; /* ζ in q̈+2ζωq̇+ω²q=F (z / Z keys)
                        * Range 0.001 (almost undamped, long
                        * ringdown) to 0.5 (heavily damped,
                        * ~no oscillation). Default 0.025
                        * matches structural steel.         */
  bool paused;         /* spc key — freezes every sim tick
                        * (beam_tick is a no-op).           */

  /* ── Configuration (drives §5 solve_beam / §6 dyn_setup dispatch) ── */
  BCType bc;   /* SS / Cantilever / FF — n / p keys */
  LDType load; /* centre / UDL / offset — l key     */

  /* ── Solver outputs (refreshed each tick; consumed by renderer) ──── */
  float max_deflection; /* max |w[i]| over the beam.
                         * Renderer uses this as the y-scale
                         * in defl_row(). In dynamic mode
                         * it's frozen via DynBeam.ref_max_w
                         * to avoid scale jitter at zero
                         * crossings.                        */
  float max_moment;     /* max |M[i]| over the beam. Drives
                         * the stress-band threshold in
                         * draw_curvature_column and the bar
                         * normalisation in the moment panel.*/

  /* ── Free-vibration sub-state (only meaningful when dyn.active) ──── */
  DynBeam dyn;
} Beam;

/* apply_load — set BC, load type, and load magnitude on the beam.
 * Does NOT reset load_anim; callers (init_beam, the n/p/l key handlers)
 * touch the ramp explicitly when they want the new load to grow in
 * from zero instead of snapping to the current ramp position. */
static void apply_load(Beam *b, BCType bc, LDType ld, float P) {
  b->bc = bc;
  b->load = ld;
  b->P = P;
}

static void init_beam(Beam *b) {
  memset(b, 0, sizeof *b);
  b->L = 1.0f;
  b->EI = 1.0f;
  b->exag = 1.0f;
  b->load_anim = 0.0f;
  b->damping_ratio = MODAL_DAMP; /* 2.5%, like structural steel */
  b->paused = false;
  for (int i = 0; i < N_NODES; i++)
    b->x[i] = (float)i / (float)(N_NODES - 1);
  /* Default to the most visually dramatic configuration: cantilever +
   * tip load. Largest absolute deflection of any BC; slowest mode-1
   * period so the eye can track the oscillation; longest ringdown
   * because the mode is far from the high-frequency damped regime. */
  apply_load(b, BC_CANT, LD_CENTER, 1.0f);
}

/* ===================================================================== */
/* §5  solver — closed-form Euler-Bernoulli solutions                     */
/* ===================================================================== */

/*
 * The static solver evaluates the analytic (M, w) at every beam node by
 * dispatching to a per-(BC, load) helper.  All closed forms come from
 * Roark Tables 8.1 – 8.10 [2]; they are derived by integrating
 * EI·w'''' = q(x) four times and applying the four boundary conditions
 * of the chosen BC.  Sign convention: w positive downward, M positive
 * sagging.
 *
 * The helpers below are pure: they read no Beam state, only their
 * arguments, and write (M, w) through output pointers.  Each comment
 * gives the closed-form expression next to the code that implements it
 * — read the docstring before the body.
 *
 * solve_beam itself is the dispatch table plus the per-node bookkeeping
 * (max_w, max_M for renderer normalisation).
 */

/*
 * simply_supported_point_load — pin-pin beam, point load P at x = a.
 *
 * Roark Table 8.1 [2].  With b = L - a:
 *
 *   x ≤ a:  M(x) = P·b·x / L
 *           w(x) = P·b·x·(L² - b² - x²) / (6·EI·L)
 *   x > a:  M and w of the right segment via symmetric substitution
 *           xs = L - x, swap a ↔ b.
 *
 * Reactions:    R_left = P·b/L,  R_right = P·a/L.
 * Moment peak:  M_max = P·a·b/L at x = a.
 * For a = L/2:  δ_max = PL³/(48·EI), M_max = PL/4 — textbook midspan.
 */
static void simply_supported_point_load(float x, float L, float EI, float P,
                                        float a, float *M, float *w) {
  float b = L - a;
  if (x <= a) {
    *M = P * b * x / L;
    *w = P * b * x * (L * L - b * b - x * x) / (6.0f * EI * L);
  } else {
    float xs = L - x;
    *M = P * a * xs / L;
    *w = P * a * xs * (L * L - a * a - xs * xs) / (6.0f * EI * L);
  }
}

/*
 * simply_supported_udl — pin-pin beam, uniform load q over full span.
 *
 * Roark Table 8.1 [2].
 *
 *   M(x) = q·x·(L - x) / 2          (parabola, peak qL²/8 at midspan)
 *   w(x) = q·x·(L³ - 2L·x² + x³) / (24·EI)
 *
 * 4th-order polynomial in x.  Symmetric: w(x) = w(L - x).
 * δ_max = 5qL⁴/(384·EI) at midspan.
 */
static void simply_supported_udl(float x, float L, float EI, float q, float *M,
                                 float *w) {
  *M = q * x * (L - x) / 2.0f;
  *w = q * x * (L * L * L - 2.0f * L * x * x + x * x * x) / (24.0f * EI);
}

/*
 * cantilever_tip_point_load — fixed-free beam, point load P at the tip.
 *
 * Roark Table 8.2 [2].
 *
 *   M(x) = P·(L - x)           (linear, max at wall, zero at tip)
 *   w(x) = P·x²·(3L - x) / (6·EI)
 *
 * δ_tip = PL³/(3·EI) — 16× the SS midspan deflection for the same P.
 * The clamped wall carries BOTH the shear P and the moment M_wall = PL.
 */
static void cantilever_tip_point_load(float x, float L, float EI, float P,
                                      float *M, float *w) {
  *M = P * (L - x);
  *w = P * x * x * (3.0f * L - x) / (6.0f * EI);
}

/*
 * cantilever_intermediate_point_load — fixed-free, point load P at x = a < L.
 *
 * Roark Table 8.2 (intermediate load) [2].
 *
 *   x ≤ a:  M(x) = P·(a - x)
 *           w(x) = P·x²·(3a - x) / (6·EI)
 *   x > a:  M(x) = 0
 *           w(x) = P·a²·(3x - a) / (6·EI)   (rigid extension)
 *
 * Beyond the load, the beam carries no internal moment, so it continues
 * as a straight tangent line from the deflection at a.
 */
static void cantilever_intermediate_point_load(float x, float L, float EI,
                                               float P, float a, float *M,
                                               float *w) {
  (void)L; /* helper is L-agnostic — only a matters here */
  if (x <= a) {
    *M = P * (a - x);
    *w = P * x * x * (3.0f * a - x) / (6.0f * EI);
  } else {
    *M = 0.0f;
    *w = P * a * a * (3.0f * x - a) / (6.0f * EI);
  }
}

/*
 * cantilever_udl — fixed-free beam, uniform load q over full span.
 *
 * Roark Table 8.2 [2].
 *
 *   M(x) = q·(L - x)² / 2          (max qL²/2 at wall, zero at tip)
 *   w(x) = q·x²·(6L² - 4L·x + x²) / (24·EI)
 *
 * δ_tip = qL⁴/(8·EI).  Wall stress is the highest of any canonical
 * cantilever loading — total load qL acts with moment arm L/2.
 */
static void cantilever_udl(float x, float L, float EI, float q, float *M,
                           float *w) {
  float Lx = L - x;
  *M = q * Lx * Lx / 2.0f;
  *w = q * x * x * (6.0f * L * L - 4.0f * L * x + x * x) / (24.0f * EI);
}

/*
 * fixed_fixed_center_point_load — both ends clamped, point load at L/2.
 *
 * Roark Table 8.3 [2].  Symmetric problem.
 *
 *   Fixed-end moments: M_A = M_B = -PL/8  (hogging)
 *   Reactions:         R_A = R_B = P/2
 *
 *   x ≤ L/2:  M(x) = P·x/2 - PL/8        (linear, crosses zero at x = L/4)
 *             w(x) = P·x²·(3L - 4x) / (48·EI)
 *   x > L/2:  mirror substitution xs = L - x.
 *
 * Stiffest of the three canonical BCs: δ_max = PL³/(192·EI), only ¼ of
 * the SS midspan deflection.  Moment diagram crosses zero TWICE
 * (sagging midspan, hogging walls).
 */
static void fixed_fixed_center_point_load(float x, float L, float EI, float P,
                                          float *M, float *w) {
  if (x <= 0.5f * L) {
    *M = P * x / 2.0f - P * L / 8.0f;
    *w = P * x * x * (3.0f * L - 4.0f * x) / (48.0f * EI);
  } else {
    float xs = L - x;
    *M = P * xs / 2.0f - P * L / 8.0f;
    *w = P * xs * xs * (3.0f * L - 4.0f * xs) / (48.0f * EI);
  }
}

/*
 * fixed_fixed_udl — both ends clamped, uniform load q over full span.
 *
 * Roark Table 8.3 [2].
 *
 *   Fixed-end moments: M_A = M_B = -qL²/12
 *   M(x) = q·x·(L - x)/2 - qL²/12
 *   w(x) = q·x²·(L - x)² / (24·EI)
 *
 * Inflection points at x = L/2 ± L/(2√3).  Wall moment qL²/12 is the
 * largest stress concentration; midspan moment is qL²/24.
 * δ_max = qL⁴/(384·EI) — one-fifth of SS midspan UDL deflection.
 */
static void fixed_fixed_udl(float x, float L, float EI, float q, float *M,
                            float *w) {
  float Lx = L - x;
  *M = q * x * Lx / 2.0f - q * L * L / 12.0f;
  *w = q * x * x * Lx * Lx / (24.0f * EI);
}

/*
 * fixed_fixed_offset_point_load — both ends clamped, point load P at x = a.
 *
 * Roark Table 8.3 (intermediate point load) [2], stiffness-method form.
 * With b = L - a:
 *
 *   Fixed-end moments: M_A = -P·a·b² / L²,   M_B = -P·a²·b / L²
 *   Wall reaction:     R_A = P·b²·(3a + b) / L³
 *
 *   x ≤ a:  M(x) = -M_A + R_A·x
 *           w(x) = P·b²·x²·(3aL - (3a + b)·x) / (6·EI·L³)
 *   x > a:  M(x) = -M_A + R_A·x - P·(x - a)
 *           Using xs = L - x:
 *           w(x) = P·a²·xs²·(3bL - (3b + a)·xs) / (6·EI·L³)
 *
 * Reduces to fixed_fixed_center_point_load when a = b = L/2.
 */
static void fixed_fixed_offset_point_load(float x, float L, float EI, float P,
                                          float a, float *M, float *w) {
  float b = L - a;
  float L2 = L * L;
  float L3 = L * L2;
  float MA = P * a * b * b / L2;
  float RA = P * b * b * (3.0f * a + b) / L3;

  *M = -MA + RA * x - (x > a ? P * (x - a) : 0.0f);
  if (x <= a) {
    *w = P * b * b * x * x * (3.0f * a * L - (3.0f * a + b) * x) /
         (6.0f * EI * L3);
  } else {
    float xs = L - x;
    *w = P * a * a * xs * xs * (3.0f * b * L - (3.0f * b + a) * xs) /
         (6.0f * EI * L3);
  }
}

/*
 * solve_beam — dispatch closed-form (M(x), w(x)) per node, then record
 * the field maxima for renderer normalisation.
 *
 * Pseudocode:
 *   P_eff ← b->P · b->load_anim                 (animated load ramp)
 *   for each node i:
 *       x ← b->x[i] · L
 *       dispatch on (b->bc, b->load) → one of the 8 helpers above,
 *           writing (M[i], w[i]).
 *   max_deflection ← max |w[i]|
 *   max_moment     ← max |M[i]|
 */
static void solve_beam(Beam *b) {
  float L = b->L;
  float EI = b->EI;
  float P = b->P * b->load_anim;

  for (int i = 0; i < N_NODES; i++) {
    float x = b->x[i] * L;
    float w = 0.0f, M = 0.0f;

    switch (b->bc) {
    case BC_SS:
      switch (b->load) {
      case LD_CENTER:
        simply_supported_point_load(x, L, EI, P, 0.5f * L, &M, &w);
        break;
      case LD_UDL:
        simply_supported_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        simply_supported_point_load(x, L, EI, P, 0.75f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    case BC_CANT:
      switch (b->load) {
      case LD_CENTER:
        cantilever_tip_point_load(x, L, EI, P, &M, &w);
        break;
      case LD_UDL:
        cantilever_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        cantilever_intermediate_point_load(x, L, EI, P, 0.5f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    case BC_FF:
      switch (b->load) {
      case LD_CENTER:
        fixed_fixed_center_point_load(x, L, EI, P, &M, &w);
        break;
      case LD_UDL:
        fixed_fixed_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        fixed_fixed_offset_point_load(x, L, EI, P, 0.75f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    default:
      break;
    }

    b->w[i] = w;
    b->M[i] = M;
  }

  /* Field extrema for renderer normalisation (stress ramp + moment bars). */
  float max_w = 0.0f, max_M = 0.0f;
  for (int i = 0; i < N_NODES; i++) {
    if (fabsf(b->w[i]) > max_w)
      max_w = fabsf(b->w[i]);
    if (fabsf(b->M[i]) > max_M)
      max_M = fabsf(b->M[i]);
  }
  b->max_deflection = max_w;
  b->max_moment = max_M;
}

/* ===================================================================== */
/* §6  dynamics — modal superposition                                     */
/* ===================================================================== */

/*
 * Mode-shape helpers — one per BC.  Each precomputes ωₙ and φₙ(xᵢ) for
 * N_MODES eigenmodes.  All work in double precision (see DynBeam
 * docstring) and write the final shape as float into d->phi for the
 * renderer / synthesis loop.
 *
 * The common form for the fixed-end cases (cantilever, FF) is:
 *
 *     φₙ(x) = cosh(βₙx) - cos(βₙx) - σₙ·(sinh(βₙx) - sin(βₙx))
 *
 * where σₙ is the BC-specific coefficient that enforces the boundary
 * conditions at x = L.  The simply-supported case collapses to pure
 * sines and skips the hyperbolic terms entirely.
 *
 * Algorithm refs: Rao §8 [3], Blevins Table 8-1 [4].
 */

/*
 * setup_simply_supported_modes — pure sine eigenmodes for the pin-pin beam.
 *
 * Characteristic equation: sin(βL) = 0  ⇒  βₙL = nπ  (closed-form roots).
 * With ρA = EI = 1:
 *     ωₙ      = βₙ² = (nπ/L)²
 *     φₙ(x)   = sin(βₙ·x)
 *
 * Pure sines have no near-equal-magnitude cancellation; double promotion
 * here is purely for consistency with the other two helpers.
 */
static void setup_simply_supported_modes(DynBeam *d, const Beam *b) {
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double beta = (double)(n + 1) * 3.14159265358979323846 / L;
    d->omega[n] = (float)(beta * beta);
    for (int i = 0; i < N_NODES; i++)
      d->phi[n][i] = (float)sin(beta * b->x[i] * L);
  }
}

/*
 * setup_cantilever_modes — fixed-free eigenmodes.
 *
 * Characteristic equation: cos(βL)·cosh(βL) + 1 = 0
 * Roots (Blevins Table 8-1 [4]):  βₙL = 1.87510, 4.69409, 7.85476, 10.99554
 *
 *     σₙ = (cos βₙL + cosh βₙL) / (sin βₙL + sinh βₙL)
 *     ωₙ = βₙ²    (EI = ρA = 1)
 *
 * Computed in double — cosh(βₙL) ≈ 30 000 at n = 4 destroys 5+ digits in
 * float during the cosh - σ·sinh subtraction.
 */
static void setup_cantilever_modes(DynBeam *d, const Beam *b) {
  static const double lam[N_MODES] = {1.87510, 4.69409, 7.85476, 10.99554};
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double bn = lam[n] / L;
    double sig = (cos(lam[n]) + cosh(lam[n])) / (sin(lam[n]) + sinh(lam[n]));
    d->omega[n] = (float)(bn * bn);
    for (int i = 0; i < N_NODES; i++) {
      double x = b->x[i] * L;
      d->phi[n][i] = (float)(cosh(bn * x) - cos(bn * x) -
                             sig * (sinh(bn * x) - sin(bn * x)));
    }
  }
}

/*
 * setup_fixed_fixed_modes — clamped-clamped eigenmodes.
 *
 * Characteristic equation: cos(βL)·cosh(βL) - 1 = 0
 * Roots (Blevins Table 8-1 [4]):  βₙL = 4.73004, 7.85321, 10.99561, 14.13717
 *
 *     σₙ = (cos βₙL - cosh βₙL) / (sinh βₙL - sin βₙL)
 *     ωₙ = βₙ²    (EI = ρA = 1)
 *
 * Same cancellation concern as cantilever — double promotion required.
 * Falls back to σₙ = -1 if the denominator collapses (shouldn't occur
 * for the tabulated roots, but defensive against numerical drift).
 */
static void setup_fixed_fixed_modes(DynBeam *d, const Beam *b) {
  static const double lam[N_MODES] = {4.73004, 7.85321, 10.99561, 14.13717};
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double bn = lam[n] / L;
    double den = sin(lam[n]) - sinh(lam[n]);
    double sig =
        (fabs(den) > 1e-10) ? (cos(lam[n]) - cosh(lam[n])) / den : -1.0;
    d->omega[n] = (float)(bn * bn);
    for (int i = 0; i < N_NODES; i++) {
      double x = b->x[i] * L;
      d->phi[n][i] = (float)(cosh(bn * x) - cos(bn * x) -
                             sig * (sinh(bn * x) - sin(bn * x)));
    }
  }
}

/*
 * integrate_modal_masses_trapezoidal — Mₙ = ρA · ∫₀ᴸ φₙ²(x) dx, computed
 * by the trapezoidal rule on the uniform N_NODES grid.
 *
 *     Mₙ ≈ ρA · Σᵢ φₙ²(xᵢ) · Δx     (Δx = L / (N_NODES - 1))
 *
 * With ρA = 1, Mₙ has units of length.  Used to normalise the
 * generalised force Fₙ so each mode's equation reads
 *     q̈ + 2ζωq̇ + ω²q = Fₙ
 * with consistent units. Clamps Mₙ to 1.0 if it numerically degenerates,
 * preventing a downstream division by zero in dyn_load.
 *
 * Algorithm ref: Rao §8 (modal mass) [3].
 */
static void integrate_modal_masses_trapezoidal(DynBeam *d, const Beam *b) {
  double dx = (double)b->L / (double)(N_NODES - 1);
  for (int n = 0; n < N_MODES; n++) {
    double mm = 0.0;
    for (int i = 0; i < N_NODES; i++)
      mm += (double)d->phi[n][i] * d->phi[n][i] * dx;
    d->modal_mass[n] = (float)(mm > 1e-9 ? mm : 1.0);
  }
}

/*
 * dyn_setup — pre-compute the modal basis (ωₙ, φₙ, Mₙ) for the current BC.
 *
 * Pseudocode:
 *   dispatch on b->bc:
 *       SS    → setup_simply_supported_modes  (pure sines)
 *       Cant  → setup_cantilever_modes        (hyperbolic-trig hybrid)
 *       FF    → setup_fixed_fixed_modes       (hyperbolic-trig hybrid)
 *   integrate_modal_masses_trapezoidal        (∫ φₙ² dx for each mode)
 *
 * Called once at dyn_activate (or re-call when BC / load changes mid-
 * vibration via n / p / l).  The result is frozen for the duration of
 * the run — q(t) varies every tick, φ(x) does not.
 */
static void dyn_setup(Beam *b) {
  DynBeam *d = &b->dyn;

  switch (b->bc) {
  case BC_SS:
    setup_simply_supported_modes(d, b);
    break;
  case BC_CANT:
    setup_cantilever_modes(d, b);
    break;
  case BC_FF:
    setup_fixed_fixed_modes(d, b);
    break;
  default:
    break;
  }

  integrate_modal_masses_trapezoidal(d, b);
}

/*
 * dyn_load — project load q(x) onto each mode to get generalised forces.
 *
 *   Fₙ = (1/Mₙ) · ∫ q(x)·φₙ(x) dx
 *
 * For point loads at position xf, the Dirac-delta integral gives:
 *   Fₙ = P·φₙ(xf) / Mₙ
 *
 * For UDL, numerical integration (rectangle rule) over all nodes.
 */
static void dyn_load(Beam *b) {
  DynBeam *d = &b->dyn;
  float L = b->L;
  float P = b->P;
  float dx = L / (float)(N_NODES - 1);

  for (int n = 0; n < N_MODES; n++) {
    float Fn = 0.0f;
    if (b->load == LD_UDL) {
      for (int i = 0; i < N_NODES; i++)
        Fn += P * d->phi[n][i] * dx;
    } else {
      float xf = (b->load == LD_CENTER) ? 0.5f : 0.75f;
      int ni = (int)(xf * (float)(N_NODES - 1) + 0.5f);
      Fn = P * d->phi[n][ni];
    }
    d->Fn[n] = Fn / d->modal_mass[n];
  }
}

/*
 * dyn_activate — switch from static to dynamic oscillation mode.
 *
 * Initial conditions: q = q̇ = 0 — the beam starts from rest at zero
 * deflection and the load is then applied as a step function.
 *
 * Dynamic magnification:  under a suddenly-applied constant load the
 * response overshoots the static deflection by up to 2× on the first
 * half-cycle (DMF = 2 for the undamped case).  Damping reduces this.
 * The damping ratio lives on the Beam as b->damping_ratio and is
 * interactive (z / Z keys); init_beam seeds it from MODAL_DAMP (2.5%,
 * a typical structural-steel value), but each call to dyn_activate
 * just consumes whatever value is currently there.
 *
 * Steps performed:
 *   1. Snap load_anim to 1.0 (modes see a clean step, not a ramp).
 *   2. Run solve_beam once to get b->max_deflection — captured as the
 *      locked visual scale (ref_max_w) so the display doesn't rezoom
 *      as the beam swings through equilibrium.
 *   3. Pre-compute the modal basis (dyn_setup) and the generalised
 *      forces (dyn_load) for the current BC and load.
 *   4. Zero out the modal coordinates / velocities and the trail ring
 *      buffer; flip d->active so beam_tick routes to dyn_tick_modes.
 */
static void dyn_activate(Beam *b) {
  b->load_anim = 1.0f;
  solve_beam(b); /* compute b->max_deflection   */

  DynBeam *d = &b->dyn;
  d->ref_max_w = b->max_deflection; /* freeze the visual y-scale   */

  dyn_setup(b); /* mode shapes + modal masses  */
  dyn_load(b);  /* generalised forces Fₙ       */

  for (int n = 0; n < N_MODES; n++) {
    d->q[n] = 0.0f;
    d->qdot[n] = 0.0f;
  }
  memset(d->trail_w, 0, sizeof d->trail_w);
  d->trail_head = 0;
  d->active = true;
}

/*
 * advance_damped_oscillator_exact — single-mode step under the exact
 * transition matrix for q̈ + 2ζω·q̇ + ω²·q = F (F constant over the step).
 *
 * WHY exact, not Euler / RK:  explicit Euler maps the homogeneous
 * eigenvalues e^(±iωdt) to (1 ± iωdt), which grows in magnitude as soon
 * as ω·dt > 2.  At ω₄ ≈ 120 rad/s and dt = 1/30 s, ω·dt ≈ 4 — Euler
 * diverges within a few frames.  The closed-form transition matrix
 * below has eigenvalues whose magnitude is exactly e^(-ζωdt) < 1, so
 * it is unconditionally stable for any dt and any ω.
 *
 * Derivation (F constant on [t, t+dt]):
 *
 *   Static offset:   q_s = F / ω²
 *   Let η = q - q_s.  Then η̈ + 2ζω·η̇ + ω²·η = 0     (homogeneous)
 *
 *   η(t + dt) = e^(-ζω·dt) · [ A·cos(ω_d·dt) + B·sin(ω_d·dt) ]
 *   η̇(t + dt) = e^(-ζω·dt) · [ (-ζω·A + ω_d·B)·cos(ω_d·dt)
 *                              + (-ζω·B - ω_d·A)·sin(ω_d·dt) ]
 *
 *   ω_d = ω · √(1 - ζ²)              (damped natural frequency)
 *   A   = η(t)
 *   B   = (η̇(t) + ζω·η(t)) / ω_d
 *
 * Algorithm ref: Rao Ch. 2 (single-DOF damped vibration) [3].
 */
static void advance_damped_oscillator_exact(float omega, float zeta, float F,
                                            float dt, float *q, float *qdot) {
  float q_s = (omega > 1e-6f) ? F / (omega * omega) : 0.0f;
  float wd = omega * sqrtf(1.0f - zeta * zeta);
  if (wd < 1e-6f)
    wd = 1e-6f; /* critical-damping floor */

  float eta = *q - q_s;
  float etad = *qdot;
  float A = eta;
  float B = (etad + zeta * omega * A) / wd;
  float e = expf(-zeta * omega * dt);
  float c = cosf(wd * dt);
  float s = sinf(wd * dt);

  *q = q_s + e * (A * c + B * s);
  *qdot =
      e * ((-zeta * omega * A + wd * B) * c + (-zeta * omega * B - wd * A) * s);
}

/*
 * synthesise_deflection_from_modes — modal-superposition synthesis.
 *
 *     w(xᵢ) = Σₙ qₙ(t) · φₙ(xᵢ)
 *
 * The integrator advances qₙ(t) in modal coordinates (each mode
 * independent — that's the whole point of solving the eigenvalue
 * problem). This helper performs the inverse transform back to
 * physical space, the "synthesis" half of modal superposition.
 *
 * Returns max |w| over the beam so the caller can grow the visual
 * y-scale in a single pass.
 *
 * Algorithm ref: Rao §8 modal superposition [3].
 */
static float synthesise_deflection_from_modes(Beam *b) {
  const DynBeam *d = &b->dyn;
  float max_w = 0.0f;
  for (int i = 0; i < N_NODES; i++) {
    float w = 0.0f;
    for (int n = 0; n < N_MODES; n++)
      w += d->q[n] * d->phi[n][i];
    b->w[i] = w;
    if (fabsf(w) > max_w)
      max_w = fabsf(w);
  }
  return max_w;
}

/*
 * push_trail_snapshot — copy the current w[] into the motion-blur ring
 * buffer and advance the head.  Renderer reads back from head-1, -2, …
 * so older snapshots fade to sparser stippling (§7 render_trails).
 */
static void push_trail_snapshot(DynBeam *d, const float *w) {
  memcpy(d->trail_w[d->trail_head], w, sizeof(float) * N_NODES);
  d->trail_head = (d->trail_head + 1) % N_TRAIL;
}

/*
 * grow_visual_scale_with_hysteresis — let the display y-scale ref_max_w
 * grow but never shrink.
 *
 * Why one-way:  as the beam swings through equilibrium, the instantaneous
 * max |w| dips toward zero. A two-way scale would rapidly rezoom and the
 * eye would see the BEAM appear to shrink/grow — visually disorienting
 * and physically wrong (the oscillation ENVELOPE hasn't changed).
 *
 * 5% hysteresis prevents micro-noise (numerical roundoff) from
 * inflating the scale frame by frame.
 */
static void grow_visual_scale_with_hysteresis(DynBeam *d, float current_max) {
  if (current_max > d->ref_max_w * 1.05f)
    d->ref_max_w = current_max;
}

/*
 * dyn_tick_modes — advance every mode, synthesise w(x), record the
 * trail, then update the display scale.
 *
 * Pseudocode:
 *   for each mode n:
 *       advance_damped_oscillator_exact(ωₙ, ζ, Fₙ, dt, qₙ, q̇ₙ)
 *   max_w ← synthesise_deflection_from_modes(b)
 *   push_trail_snapshot(b->w)
 *   grow_visual_scale_with_hysteresis(max_w)
 *   b->max_deflection ← d->ref_max_w        (renderer reads this)
 *
 * One driver, four physics-named steps — each line of the body reads
 * as a step in modal-superposition theory, not as arithmetic.
 */
static void dyn_tick_modes(Beam *b, float dt) {
  DynBeam *d = &b->dyn;
  float zeta = b->damping_ratio; /* z / Z keys */

  for (int n = 0; n < N_MODES; n++)
    advance_damped_oscillator_exact(d->omega[n], zeta, d->Fn[n], dt, &d->q[n],
                                    &d->qdot[n]);

  float max_w = synthesise_deflection_from_modes(b);
  push_trail_snapshot(d, b->w);
  grow_visual_scale_with_hysteresis(d, max_w);
  b->max_deflection = d->ref_max_w;
}

/*
 * dyn_impulse — instantaneous velocity kick at the load location.
 *
 * For a Dirac impulse P·δ(t)·δ(x-xf), the integrated equation of motion
 * for each mode gives a discrete velocity jump:
 *
 *     Δq̇ₙ = P · φₙ(xf) / Mₙ
 *
 * Strikes the beam like a tuning fork — re-excites all modes coherently
 * (in phase with where the kick lands). Use to refresh a decayed
 * oscillation without changing BC or load.
 */
static void dyn_impulse(Beam *b, float impulse) {
  DynBeam *d = &b->dyn;
  float xf = 0.5f;
  if (b->bc == BC_CANT)
    xf = 1.0f; /* free tip */
  else if (b->load == LD_OFFSET)
    xf = 0.75f;
  int ni = (int)(xf * (float)(N_NODES - 1) + 0.5f);
  if (ni >= N_NODES)
    ni = N_NODES - 1;
  for (int n = 0; n < N_MODES; n++)
    d->qdot[n] += impulse * d->phi[n][ni] / d->modal_mass[n];
}

/* beam_tick — per-frame update dispatcher */
static void beam_tick(Beam *b, float dt) {
  if (b->paused)
    return;

  if (b->dyn.active) {
    dyn_tick_modes(b, dt);
  } else {
    /* Static mode: animate the load ramp, then re-solve analytically */
    if (b->load_anim < 1.0f) {
      b->load_anim += dt * RAMP_SPEED;
      if (b->load_anim > 1.0f)
        b->load_anim = 1.0f;
    }
    solve_beam(b);
  }
}

/* ===================================================================== */
/* §7  render                                                             */
/* ===================================================================== */

/* node_col — map beam node index to screen column */
static int node_col(int i, int x0, int x1) {
  return x0 + i * (x1 - x0) / (N_NODES - 1);
}

/*
 * defl_row — convert deflection w to row offset from the neutral axis.
 *
 * The base_rows range is rows/6 (a fraction of screen height), scaled
 * by the exaggeration factor so even tiny deflections become visible.
 * Clamped to 3×base_rows so the beam never exits the visible area.
 */
static int defl_row(float w, float max_w, int base_rows, float exag) {
  if (max_w < 1e-9f)
    return 0;
  int dr = (int)(w / max_w * (float)base_rows * exag + 0.5f);
  int cap = base_rows * 3;
  return (dr > cap) ? cap : (dr < -cap) ? -cap : dr;
}

/*
 * draw_curvature_column — render one column of the beam cross-section.
 *
 * Character density encodes |M|/M_max (bending stress proxy):
 *   " .-~=*#@"  — 8 levels, sparse = low stress, dense = high stress.
 *
 * Color encodes the same quantity coarsely:
 *   green (< 35 %) → yellow (35–70 %) → red (> 70 %)
 *
 * The top and bottom flanges are always '=' to hint at the cross-section
 * shape; the web is the density character for interior cells.
 *
 * WHY this mapping: the visible character density intuitively reads as
 * "how hard is the material working here?" — dense chars look stressed.
 * Color reinforces it with traffic-light semantics (green = safe).
 */
static void draw_curvature_column(int col, int mid_row, float kn, int half_h,
                                  int rows) {
  static const char pal[] = " .-~=*#@";
  int npal = (int)(sizeof pal - 1);

  int pi = (int)(kn * (float)(npal - 1) + 0.5f);
  if (pi < 0)
    pi = 0;
  if (pi >= npal)
    pi = npal - 1;
  chtype fill = (chtype)pal[pi];

  int cp = (kn < 0.35f) ? CP_LO : (kn < 0.70f) ? CP_MED : CP_HI;

  for (int rr = mid_row - half_h; rr <= mid_row + half_h; rr++) {
    if (rr < 2 || rr >= rows - 2)
      continue;
    bool is_flange = (rr == mid_row - half_h || rr == mid_row + half_h);
    chtype ch = is_flange ? '=' : fill;
    attr_t attr = (chtype)COLOR_PAIR(cp) | (rr == mid_row ? A_BOLD : 0);
    attron(attr);
    mvaddch(rr, col, ch);
    attroff(attr);
  }
}

/*
 * Load-glyph helpers — draw the load application symbols above the
 * beam.  The wrapper draw_load_arrows applies the colour / bold
 * attribute once and dispatches; helpers are attribute-agnostic.
 */

/*
 * draw_point_load_glyph — single point-load marker at fractional span
 * xfrac ∈ [0, 1] (e.g. 0.5 for centre, 0.75 for offset).
 *
 * Layout above the local beam top at column lc:
 *
 *     [top - LOAD_ARROW_H]    P        <- label
 *     [top - LOAD_ARROW_H+1]  |
 *            ...              |        <- stem
 *     [top - 1]               v        <- arrowhead just above beam
 *
 * The marker tracks the deflected position because `top` is recomputed
 * from b->w[ni] every frame — under vibration the arrow oscillates
 * with the beam.
 */
static void draw_point_load_glyph(const Beam *b, float xfrac, int x0, int x1,
                                  int neutral_row, int base_rows, int half_h,
                                  int rows) {
  int lc = x0 + (int)(xfrac * (float)(x1 - x0) + 0.5f);
  int ni = (int)(xfrac * (float)(N_NODES - 1));
  int dr = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
  int top = neutral_row + dr - half_h;

  /* Stem */
  for (int rr = top - LOAD_ARROW_H; rr < top - 1; rr++)
    if (rr >= 2 && rr < rows - 2)
      mvaddch(rr, lc, '|');
  /* Label */
  if (top - LOAD_ARROW_H >= 2)
    mvprintw(top - LOAD_ARROW_H, lc - 1, "P");
  /* Arrowhead */
  if (top - 1 >= 2)
    mvaddch(top - 1, lc, 'v');
}

/*
 * draw_distributed_load_glyphs — UDL: rake of arrows every 3rd column.
 *
 * Each arrow contours the deformed shape because its `top` is sampled
 * from the local node's deflection — so the rake of arrowheads traces
 * the beam's profile.  A centred 'q=N.NN' label sits above the rake.
 */
static void draw_distributed_load_glyphs(const Beam *b, int x0, int x1,
                                         int neutral_row, int base_rows,
                                         int half_h, int rows) {
  for (int c = x0; c <= x1; c += 3) {
    int ni = (c - x0) * (N_NODES - 1) / (x1 - x0);
    if (ni < 0)
      ni = 0;
    if (ni >= N_NODES)
      ni = N_NODES - 1;
    int dr = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
    int top = neutral_row + dr - half_h;
    if (top - 1 >= 2 && top - 1 < rows - 2)
      mvaddch(top - 1, c, 'v');
    if (top - 2 >= 2 && top - 2 < rows - 2)
      mvaddch(top - 2, c, '|');
  }
  int mc = (x0 + x1) / 2;
  if (neutral_row - half_h - 3 >= 2)
    mvprintw(neutral_row - half_h - 3, mc - 3, "q=%.2f", b->P * b->load_anim);
}

/*
 * draw_load_arrows — dispatch on load type, apply colour once.
 *
 * Pseudocode:
 *   attron(CP_LOAD | bold)
 *   if    load == CENTER:  draw_point_load_glyph   at xfrac = 0.5
 *   elif  load == OFFSET:  draw_point_load_glyph   at xfrac = 0.75
 *   elif  load == UDL:     draw_distributed_load_glyphs across the span
 *   attroff
 */
static void draw_load_arrows(const Beam *b, int x0, int x1, int neutral_row,
                             int base_rows, int half_h, int cols, int rows) {
  (void)cols;
  attron(COLOR_PAIR(CP_LOAD) | A_BOLD);

  switch (b->load) {
  case LD_CENTER:
    draw_point_load_glyph(b, 0.5f, x0, x1, neutral_row, base_rows, half_h,
                          rows);
    break;
  case LD_OFFSET:
    draw_point_load_glyph(b, 0.75f, x0, x1, neutral_row, base_rows, half_h,
                          rows);
    break;
  case LD_UDL:
    draw_distributed_load_glyphs(b, x0, x1, neutral_row, base_rows, half_h,
                                 rows);
    break;
  default:
    break;
  }

  attroff(COLOR_PAIR(CP_LOAD) | A_BOLD);
}

/*
 * draw_supports — draw boundary condition symbols.
 *
 * SS:  'A' triangles + '/_%c\' base line (standard pin-on-roller symbol).
 * Cantilever: hatch '#' on left wall; "free" label at right tip.
 * Fixed-Fixed: hatch '#' walls on both ends.
 */
static void draw_supports(BCType bc, int x0, int x1, int neutral_row,
                          int half_h, int cols, int rows) {
  attron(COLOR_PAIR(CP_SUPP) | A_BOLD);

  switch (bc) {

  case BC_SS: {
    int sr = neutral_row + half_h + 1;
    if (sr < rows - 2) {
      mvaddch(sr, x0, 'A');
      mvaddch(sr, x1, 'A');
    }
    if (sr + 1 < rows - 2) {
      mvprintw(sr + 1, x0 - 1, "/_%c", '\\');
      mvprintw(sr + 1, x1 - 1, "/_%c", '\\');
    }
    break;
  }

  case BC_CANT: {
    for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1;
         rr++) {
      if (rr < 1 || rr >= rows - 2)
        continue;
      if (x0 - 1 >= 0)
        mvaddch(rr, x0 - 1, '#');
      if (x0 - 2 >= 0)
        mvaddch(rr, x0 - 2, '#');
    }
    if (neutral_row >= 1 && neutral_row < rows - 2 && x1 + 1 < cols)
      mvprintw(neutral_row, x1 + 1, "free");
    break;
  }

  case BC_FF: {
    for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1;
         rr++) {
      if (rr < 1 || rr >= rows - 2)
        continue;
      if (x0 - 1 >= 0)
        mvaddch(rr, x0 - 1, '#');
      if (x1 + 1 < cols)
        mvaddch(rr, x1 + 1, '#');
    }
    break;
  }

  default:
    break;
  }

  attroff(COLOR_PAIR(CP_SUPP) | A_BOLD);
}

/*
 * render_trails — motion-blur echo of recent beam shapes.
 *
 * Iterates the DynBeam.trail_w ring buffer from oldest to newest, drawing
 * each past shape as faint dots behind the current beam. The character
 * glyph and sampling stride both fade with age so older trails read as
 * sparse stippling rather than a solid band.
 *
 * Trails use CP_LO (the active theme's lowest-stress accent) so they
 * blend with the palette but stay legible without A_DIM (which would
 * make them vanish on themes with lo=24/28).
 *
 * No-op when not in dynamic mode — a stationary beam has nothing to blur.
 */
static void render_trails(const Beam *b, int cols, int rows) {
  if (!b->dyn.active)
    return;

  int x0 = BEAM_X_MARGIN;
  int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
  if (x1 <= x0 + 10)
    return;

  int neutral_row = rows / 2;
  int base_rows = rows / 6;
  if (base_rows < 3)
    base_rows = 3;

  const DynBeam *d = &b->dyn;

  /* Oldest first so newer trails overpaint. */
  for (int k = N_TRAIL; k >= 1; k--) {
    int idx = (d->trail_head - k + N_TRAIL) % N_TRAIL;
    int stride = (k <= 3) ? 3 : (k <= 7) ? 5 : 7; /* sparser with age */
    char gl = (k <= 3) ? '.' : (k <= 7) ? ',' : '`';

    attron(COLOR_PAIR(CP_LO));
    for (int i = 0; i < N_NODES; i += stride) {
      int c = node_col(i, x0, x1);
      if (c < 0 || c >= cols)
        continue;
      int dr =
          defl_row(d->trail_w[idx][i], b->max_deflection, base_rows, b->exag);
      int rr = neutral_row + dr;
      if (rr < 2 || rr >= rows - 2)
        continue;
      mvaddch(rr, c, gl);
    }
    attroff(COLOR_PAIR(CP_LO));
  }
}

/*
 * render_beam — compose the deflected beam: trails, neutral axis, body,
 * loads, supports.
 *
 * Layer order (bottom to top of z-stack):
 *   render_trails          — motion-blur of past dynamic shapes
 *   neutral axis           — '-' reference line at w=0
 *   draw_curvature_column  — one column of beam cross-section
 *   draw_load_arrows       — load application markers
 *   draw_supports          — boundary condition symbols
 *
 * All layers share the same base geometry (x0/x1/neutral_row/base_rows).
 */
static void render_beam(const Beam *b, bool show_trails, int cols, int rows) {
  int x0 = BEAM_X_MARGIN;
  int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
  if (x1 <= x0 + 10)
    return;

  int neutral_row = rows / 2;
  int base_rows = rows / 6;
  if (base_rows < 3)
    base_rows = 3;
  int half_h = BEAM_HEIGHT / 2;

  /* Neutral axis reference line — where w=0 */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int c = x0; c <= x1; c++)
    mvaddch(neutral_row, c, '-');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  /* Motion-blur trails behind the live beam. */
  if (show_trails)
    render_trails(b, cols, rows);

  /* Beam body — one column per node */
  for (int i = 0; i < N_NODES; i++) {
    int c = node_col(i, x0, x1);
    if (c < 0 || c >= cols)
      continue;

    /* kn = normalised curvature proxy |M(x)|/M_max ∈ [0,1] */
    float kn = (b->max_moment > 1e-9f) ? fabsf(b->M[i]) / b->max_moment : 0.0f;

    int dr = defl_row(b->w[i], b->max_deflection, base_rows, b->exag);
    draw_curvature_column(c, neutral_row + dr, kn, half_h, rows);
  }

  draw_load_arrows(b, x0, x1, neutral_row, base_rows, half_h, cols, rows);
  draw_supports(b->bc, x0, x1, neutral_row, half_h, cols, rows);
}

/*
 * render_moment_panel — right-side bending moment diagram.
 *
 * Layout: each screen row maps to a beam station; horizontal bars show
 * M / M_max.  Positive (sagging) bars extend rightward with '>' chars,
 * negative (hogging) bars extend leftward with '<' chars.
 *
 * WHY two colors: sign of M determines whether the top or bottom fibre
 * is in tension.  Sagging = tension at bottom (green — common/OK).
 * Hogging = tension at top (magenta — less obvious, more dangerous in
 * concrete which is weak in tension at top).
 */
static void render_moment_panel(const Beam *b, int cols, int rows) {
  int px = cols - PANEL_W;
  int half = PANEL_W / 2 - 1;
  int zero = px + half; /* column of the zero-moment axis */
  if (rows - 4 < 3 || px < 1)
    return;

  /* Panel separator */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int r = 1; r < rows - 1; r++)
    mvaddch(r, px - 1, '|');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  attron(COLOR_PAIR(CP_HDR) | A_BOLD);
  mvprintw(1, px, " MOMENT  +/- ");
  attroff(COLOR_PAIR(CP_HDR) | A_BOLD);

  /* Zero-moment vertical axis */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int r = 2; r < rows - 2; r++)
    mvaddch(r, zero, '|');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  if (b->max_moment < 1e-9f)
    return;

  for (int r = 2; r < rows - 2; r++) {
    /* Map screen row → beam node (linear interpolation) */
    int ni = (r - 2) * (N_NODES - 1) / (rows - 4);
    if (ni < 0)
      ni = 0;
    if (ni >= N_NODES)
      ni = N_NODES - 1;

    float mn = b->M[ni] / b->max_moment; /* -1..1 */
    int len = (int)(fabsf(mn) * (float)(half - 1) + 0.5f);
    if (len > half - 1)
      len = half - 1;
    if (len == 0)
      continue;

    int cp = (mn >= 0.0f) ? CP_MOM_P : CP_MOM_N;
    char bch = (mn >= 0.0f) ? '>' : '<';

    attron(COLOR_PAIR(cp));
    int dir = (mn >= 0.0f) ? 1 : -1;
    for (int k = 1; k <= len; k++) {
      int bc2 = zero + dir * k;
      if (bc2 >= px && bc2 < cols)
        mvaddch(r, bc2, (chtype)bch);
    }
    attroff(COLOR_PAIR(cp));
  }

  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  mvprintw(rows - 2, px, " sag+ hog- ");
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*
 * render_overlay — three-bar HUD per CLAUDE.md convention.
 *
 *   Row 0 right (CP_HUD + bold)   live status: P, exag, theme, state, fps
 *   Row 1 left  (CP_HUD)          context: BC name, load name, max defl,
 *                                          max moment, [VIBRATING]
 *   Row rows-1  (CP_HINT + bold)  actions / key reference
 *
 * Row 1 is clipped at cols-PANEL_W-1 so it never paints over the
 * moment panel header at (1, cols-PANEL_W).
 */
static void render_overlay(const Beam *b, int theme, float time_scale,
                           double fps, int cols, int rows) {
  /* Row 0 — right-aligned live state. */
  char top[224];
  const char *state = "running";
  if (b->paused)
    state = "PAUSED ";
  else if (b->dyn.active)
    state = "dynamic";
  snprintf(
      top, sizeof top,
      " P=%.2f  Exag=%.1fx  zeta=%.1f%%  speed=%.2fx  theme:%s  %s  %.0f fps ",
      b->P * b->load_anim, b->exag, b->damping_ratio * 100.0f, time_scale,
      themes[theme].name, state, fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Row 1 — left-aligned context + computed values; clipped before panel. */
  char sub[160];
  snprintf(sub, sizeof sub, " %s  |  %s  |  Defl=%7.5f  M=%7.5f%s",
           bc_names[b->bc], ld_names[b->load], b->max_deflection, b->max_moment,
           b->dyn.active ? "  [VIBRATING]" : "");
  int row1_max = cols - PANEL_W - 1;
  if (row1_max < 1)
    row1_max = cols;
  attron(COLOR_PAIR(CP_HUD));
  mvaddnstr(1, 0, sub, row1_max);
  attroff(COLOR_PAIR(CP_HUD));

  /* Bottom hint — actions. */
  const char *hint_full =
      " q:quit  spc:pause  r:reset  n/p:BC  l:load  +/-:P  e/E:exag  "
      "d:vibrate  i:kick  z/Z:zeta  s/S:speed  g:trails  t/T:theme ";
  const char *hint_short =
      " q:quit  spc:pause  r:reset  n/p:BC  l:load  i:kick  t/T:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= cols - 1)
    hint = hint_short;
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — state that spans scene_tick() and scene_draw() invocations.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by scene_tick → beam_tick →
 *     solve_beam / dyn_tick_modes. Anything that affects the BEAM
 *     PHYSICS (positions, velocities, modal coords, moment field) lives
 *     here. Mutated by physics-affecting keys: n / p (BC), l (load),
 *     +/− (P), d / i (vibrate / impulse), z / Z (ζ), s / S (time speed),
 *     r (reset), spc (pause).
 *
 *   Rendering parameters   — consumed by scene_draw → render_*. Theme
 *     palettes and overlay toggles live here. Do NOT alter physics.
 *     Toggling any of these while paused must leave the beam shape
 *     and modal state byte-identical; only colours / overlays may differ.
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU. A new flag that
 *   accidentally lands in the rendering group would silently couple
 *   physics to display state — exactly the bug the separation prevents.
 *   When adding a field, pick the group by asking: "does this affect
 *   what the integrator does?" If yes, simulation; if no, rendering.
 *
 * Single instance (lives inside g_app, file scope):
 *   Scene sits inside `App g_app` rather than as its own static so that
 *   signal handlers (which need access to running / need_resize on the
 *   same struct) can reach the whole context through one symbol without
 *   a pointer chase. scene_init resets only the beam — theme, time_scale,
 *   and show_trails persist across 'r' reset so cosmetic choices aren't
 *   lost.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   running / need_resize    sig_atomic_t flags live on App, not Scene.
 *                            They're polled by the main loop, never read
 *                            by scene_tick or scene_draw.
 *   Screen (cols, rows)      window geometry is App-level; passed into
 *                            scene_draw as parameters rather than held
 *                            here, because resize handling is the App's
 *                            job (Scene is geometry-agnostic).
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ───────────────────────────────────────── */
  Beam beam;        /* the simulator core itself (§4) —
                     * Body + DynBeam + arrays + config  */
  float time_scale; /* dt multiplier applied in the main
                     * loop before scene_tick is called.
                     * Range 0.1× – 4.0× (s / S keys).
                     * Counted as SIMULATION because it
                     * scales the advance of q(t) and the
                     * static load_anim ramp — physics is
                     * not invariant under this knob.    */

  /* ── Rendering parameters ────────────────────────────────────────── */
  int theme;        /* index into themes[] in §3 (t / T) */
  bool show_trails; /* motion-blur trails behind the live
                     * beam (g / G). Pure overlay — the
                     * trail ring buffer is always pushed
                     * by dyn_tick_modes; this flag only
                     * gates whether render_trails reads
                     * it back.                          */
} Scene;

static void scene_init(Scene *s) {
  /* Cosmetic/scene-level fields (theme, time_scale, show_trails) are
   * preserved across reset — only the beam is rebuilt. */
  init_beam(&s->beam);
  dyn_activate(&s->beam); /* boot directly into the vibrating state */
}

static void scene_tick(Scene *s, float dt) { beam_tick(&s->beam, dt); }

static void scene_draw(const Scene *s, int cols, int rows, double fps) {
  erase();
  render_beam(&s->beam, s->show_trails, cols, rows);
  render_moment_panel(&s->beam, cols, rows);
  render_overlay(&s->beam, s->theme, s->time_scale, fps, cols, rows);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  /* color_init() is called from main() once the scene's theme exists. */
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

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

typedef struct {
  Scene scene;
  Screen screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize_signal(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static bool app_handle_key(App *app, int ch) {
  Beam *b = &app->scene.beam;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    b->paused = !b->paused;
    break;

  case 'n':
  case 'N':
    b->bc = (BCType)((b->bc + 1) % BC_COUNT);
    if (b->dyn.active) {
      dyn_activate(b); /* re-fit modes for the new BC, keep vibrating */
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'p':
  case 'P':
    b->bc = (BCType)((b->bc + BC_COUNT - 1) % BC_COUNT);
    if (b->dyn.active) {
      dyn_activate(b);
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'l':
  case 'L':
    b->load = (LDType)((b->load + 1) % LD_COUNT);
    if (b->dyn.active) {
      dyn_activate(b);
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'd':
  case 'D':
    if (b->dyn.active) {
      /* Toggle back to static */
      b->dyn.active = false;
      b->load_anim = 1.0f;
      solve_beam(b);
    } else {
      dyn_activate(b);
    }
    break;

  case '+':
  case '=':
    b->P = (b->P * 1.25f > 100.0f) ? 100.0f : b->P * 1.25f;
    if (b->dyn.active)
      dyn_load(b);
    else
      solve_beam(b);
    break;

  case '-':
    b->P = (b->P / 1.25f < 0.05f) ? 0.05f : b->P / 1.25f;
    if (b->dyn.active)
      dyn_load(b);
    else
      solve_beam(b);
    break;

  case 'e':
    b->exag = (b->exag * 1.5f > 20.0f) ? 20.0f : b->exag * 1.5f;
    break;

  case 'E':
    b->exag = (b->exag / 1.5f < 0.5f) ? 0.5f : b->exag / 1.5f;
    break;

  case 'z':
    b->damping_ratio *= 1.4f;
    if (b->damping_ratio > 0.5f)
      b->damping_ratio = 0.5f;
    break;

  case 'Z':
    b->damping_ratio /= 1.4f;
    if (b->damping_ratio < 0.001f)
      b->damping_ratio = 0.001f;
    break;

  case 's':
    app->scene.time_scale *= 1.25f;
    if (app->scene.time_scale > 4.0f)
      app->scene.time_scale = 4.0f;
    break;

  case 'S':
    app->scene.time_scale /= 1.25f;
    if (app->scene.time_scale < 0.1f)
      app->scene.time_scale = 0.1f;
    break;

  case 'i':
  case 'I':
    if (b->dyn.active)
      dyn_impulse(b, 1.0f);
    break;

  case 'g':
  case 'G':
    app->scene.show_trails = !app->scene.show_trails;
    break;

  case 'r':
  case 'R':
    scene_init(&app->scene);
    break;

  case 't':
    app->scene.theme = (app->scene.theme + 1) % N_THEMES;
    color_init(app->scene.theme);
    break;

  case 'T':
    app->scene.theme = (app->scene.theme + N_THEMES - 1) % N_THEMES;
    color_init(app->scene.theme);
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

  screen_init(&app->screen);
  app->scene.time_scale = 1.0f;
  app->scene.show_trails = true;
  scene_init(&app->scene);
  color_init(app->scene.theme); /* depends on scene.theme being set */

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t tick_ns = NS_PER_SEC / 30;
  float dt_sec = 1.0f / 30.0f;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* Fixed-timestep accumulator */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* spiral-of-death guard */

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      /* time_scale (s/S keys) stretches sim seconds per wall tick
       * — slow-mo for inspecting modes, fast-forward for ringdown. */
      scene_tick(&app->scene, dt_sec * app->scene.time_scale);
      sim_accum -= tick_ns;
    }

    /* fps rolling average (update every 0.5 s) */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= 500 * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Frame cap — sleep before rendering to bound CPU usage */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_display);

    wnoutrefresh(stdscr);
    doupdate();

    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
