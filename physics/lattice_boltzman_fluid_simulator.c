/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_boltzman_fluid_simulator.c — D2Q9 Lattice Boltzmann Fluid Simulator
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE ON SCREEN
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A 2-D fluid flowing left-to-right past a circular cylinder, painted
 *  one terminal cell per LBM lattice node.  Within ~30 wall-clock
 *  seconds the classic KÁRMÁN VORTEX STREET emerges [6, 7]: alternating
 *  clockwise and counter-clockwise vortices peel off the cylinder's
 *  flanks and drift downstream as a staggered double row of swirls.
 *  Reynolds number at startup is ≈ 50 on a typical 80×30 terminal —
 *  just above the shedding threshold Re ≈ 47, so the wake is laminar
 *  and the shedding is rhythmic (Strouhal number St ≈ 0.2, period ≈ 20
 *  simulation steps).
 *
 *  THREE VIEW MODES — cycle with v / o / d:
 *
 *    VELOCITY  (v)  speed magnitude |u|, painted as an 8-step
 *                   SEQUENTIAL heat ramp [8] from the active theme's
 *                   COLD anchor to its HOT anchor.  Bright cells =
 *                   fast flow (accelerated channels around the
 *                   cylinder, wake jet); dark cells = slow (stagnation
 *                   point upstream of the cylinder nose, recirculation
 *                   pocket immediately downstream).
 *
 *    VORTICITY (o)  ω_z = ∂uy/∂x − ∂ux/∂y — the curl of u.  Painted
 *                   as a DIVERGING two-hue scheme [9]: one theme hue
 *                   for clockwise rotation, the other for counter-
 *                   clockwise, with intensity (A_DIM → A_NORMAL →
 *                   A_BOLD) encoding magnitude.  The most dramatic
 *                   view of the vortex street.
 *
 *    DENSITY   (d)  pressure proxy (p = cs²·ρ = ρ/3 in LBM units).
 *                   Reveals the high-pressure stagnation region upstream
 *                   of the cylinder and the low-pressure cores at the
 *                   centre of each shed vortex.
 *
 *  STREAMLINE OVERLAY (s): sparse ASCII arrows (8 compass headings)
 *  drawn on top of any base mode, so you can read direction at a glance
 *  without losing the underlying scalar field.
 *
 *  PATTERNS (n / p): 10 obstacle geometries — each shows the SAME
 *  D2Q9 BGK fluid physics flowing around a DIFFERENT shape, so you
 *  can compare how the same Reynolds number, viscosity, and inlet
 *  conditions produce qualitatively different wakes:
 *    0 Cylinder    canonical Kármán vortex street
 *    1 Square      sharp-corner separation, wider wake
 *    2 Diamond     L1 ball, vertex-pointing upstream
 *    3 Airfoil     tilted ellipse — wake deflects downward (lift)
 *    4 Twin        tandem cylinders, gap-flow interaction
 *    5 Array       3×3 porous-medium proxy
 *    6 Step        backward-facing step, recirculation bubble
 *    7 Venturi     symmetric constriction, Bernoulli speed-up
 *    8 Splitter    cylinder + plate, shedding suppressed
 *    9 Wedge       triangular bluff body, symmetric wake
 *  Switching pattern resets the flow to equilibrium around the new
 *  obstacle; your view mode (v/o/d), streamline overlay, pause, τ,
 *  and inlet speed are preserved across the switch.
 *
 *  THEMES (t / T): 10 eight-colour palettes — Ocean, Sunset, Forest,
 *  Fire, Lava, Aurora, Plasma, Arctic, Mono, Toxic.  Each theme is
 *  literally 8 hand-picked xterm-256 indices forming a cold-to-hot
 *  velocity heat-ramp; vorticity reuses the ramp's endpoints (cold for
 *  clockwise, hot for counter-clockwise), so the whole figure shares
 *  one coherent palette story.  Themes are pure rendering — the
 *  physics is identical across all 10.
 *
 *  TUNING knobs: +/- changes τ (viscosity, hence Reynolds number);
 *  w/W changes the inlet speed.  Push Re much above ~200 and the
 *  weakly-compressible 2-D BGK-LBM can no longer resolve the wake
 *  cleanly — distributions ring, then diverge [2].
 *
 *  HUD: top row (bright yellow) shows live diagnostics — mode, active
 *  theme, τ, ν, Re, density range, peak |u|, step count, fps, paused
 *  state.  Bottom row (bright cyan) lists every keystroke.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — DISTRIBUTION FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  In the LBM [1, 5], instead of tracking individual molecules, we
 *  track the probability f_i(x,t) that a "particle" at lattice node x
 *  moves in direction i at time t.  The D2Q9 model [4] uses 9
 *  directions:
 *
 *    6  2  5          Direction  (ex,ey)   Weight w
 *    3  0  1          ─────────────────────────────
 *    7  4  8          0  rest   (0, 0)    4/9
 *                     1  E      (1, 0)    1/9
 *  The 9 distribution functions {f_0 … f_8} per cell fully describe     2  S
 * (0, 1)    1/9 the local fluid state.  Macroscopic density and velocity are
 * their    3  W      (-1,0)    1/9 zeroth and first moments: 4  N      (0,-1)
 * 1/9 5  SE     (1, 1)    1/36 ρ = Σ f_i 6  SW     (-1,1)    1/36 ρu = Σ f_i ·
 * e_i                                                    7  NW     (-1,-1) 1/36
 *                                                                        8  NE
 * (1,-1)    1/36
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — EQUILIBRIUM MODEL
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The Maxwell-Boltzmann equilibrium distribution truncated to 2nd
 *  order [4] (the truncation that makes BGK-LBM recover Navier-Stokes
 *  via Chapman-Enskog expansion):
 *
 *    f^eq_i = w_i · ρ · [1 + (e_i·u)/cs² + (e_i·u)²/(2cs⁴) - u²/(2cs²)]
 *
 *  where cs² = 1/3 is the lattice speed of sound squared.
 *  Substituting cs²=1/3:  coefficients become 3, 4.5, 1.5 (used in code).
 *
 *  Physical meaning: f^eq is what f would be if the fluid were in local
 *  thermodynamic equilibrium at density ρ, velocity u.  Real fluids
 *  are always relaxing toward this state — that relaxation IS viscosity.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — COLLISION RELAXATION (BGK)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Bhatnagar-Gross-Krook (BGK) collision [3]: relax f toward f^eq at
 *  rate ω:
 *
 *    f_i* = f_i − ω·(f_i − f^eq_i)      ω = 1/τ
 *
 *  τ is the relaxation time.  After collision, f* is streamed (propagated).
 *
 *  Physical link to viscosity:
 *    kinematic viscosity  ν = cs²·(τ − 0.5) = (τ − 0.5) / 3
 *
 *  τ→0.5:  ν→0, inviscid flow, numerically unstable (singular).
 *  τ=1.0:  ν=1/6 ≈ 0.167, very viscous (low Reynolds number).
 *  τ=0.6:  ν=0.033, moderate viscosity — good for most demos.
 *
 *  Reynolds number estimate:  Re = U·D / ν   (U=inlet speed, D=diameter)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  BOUNDARY CONDITIONS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  BOUNCE-BACK (no-slip walls + cylinder) [2]:
 *    When a particle streams into a solid node, it reflects back with
 *    reversed direction: f_OPP(i) at source ← f_i streamed value.
 *    This enforces zero velocity at the wall (no-slip condition).
 *
 *  INLET (x=0): f set to equilibrium at (ρ=1, u=U_IN, v=0).
 *  OUTLET (x=nx-1): zero-gradient — copy f from penultimate column.
 *  TOP/BOTTOM: bounce-back walls.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  REFERENCES  (cite inline as [n])
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  ── LBM theory & practice ──────────────────────────────────────────
 *
 *   [1] Succi, S. (2001) — *The Lattice Boltzmann Equation for Fluid
 *       Dynamics and Beyond*, Oxford University Press.  The canonical
 *       theoretical text: Chapman-Enskog derivation, lattice tensor
 *       symmetries, BGK derivation, boundary conditions.  Start here
 *       for the WHY of LBM.
 *
 *   [2] Krüger, T.; Kusumaatmaja, H.; Kuzmin, A.; Shardt, O.;
 *       Silva, G.; Viggen, E. M. (2017) — *The Lattice Boltzmann
 *       Method: Principles and Practice*, Springer.  Modern
 *       practitioner's handbook; the single clearest source for the
 *       D2Q9 implementation in this file, bounce-back recipes, the
 *       viscosity–τ map, and stability bounds.  The code most closely
 *       mirrors Ch. 3–4 of this book.
 *
 *   [3] Bhatnagar, P. L.; Gross, E. P.; Krook, M. (1954) — "A Model
 *       for Collision Processes in Gases…", *Physical Review* 94,
 *       511–525.  The original single-relaxation-time collision
 *       operator — the BGK in BGK-LBM.  Backs the collide() function
 *       in §6.
 *
 *   [4] Qian, Y. H.; d'Humières, D.; Lallemand, P. (1992) — "Lattice
 *       BGK Models for Navier-Stokes Equation", *Europhysics Letters*
 *       17 (6), 479–484.  Defines the D2Q9 model used here: the 9
 *       discrete velocities, weights w_i, and the proof that the
 *       2nd-order equilibrium f^eq recovers Navier-Stokes via the
 *       Chapman-Enskog expansion.
 *
 *   [5] Chen, S.; Doolen, G. D. (1998) — "Lattice Boltzmann Method
 *       for Fluid Flows", *Annual Review of Fluid Mechanics* 30,
 *       329–364.  Foundational LBM review.  Read this first as a
 *       gentle entry point before tackling [1] or [2].
 *
 *  ── Cylinder wake & vortex-shedding physics ────────────────────────
 *
 *   [6] von Kármán, T. (1911) — "Über den Mechanismus des
 *       Widerstandes, den ein bewegter Körper in einer Flüssigkeit
 *       erfährt", *Nachr. K. Ges. Wiss. Göttingen*, 509–517.  The
 *       paper that first described the staggered double row of
 *       vortices behind a bluff body — the eponymous "Kármán vortex
 *       street" that this demo reproduces.
 *
 *   [7] Williamson, C. H. K. (1996) — "Vortex Dynamics in the
 *       Cylinder Wake", *Annual Review of Fluid Mechanics* 28,
 *       477–539.  Authoritative modern review of cylinder-wake
 *       physics: Reynolds-number regime map, shedding onset
 *       (Re ≈ 47, a Hopf bifurcation), the Strouhal St(Re) curve,
 *       and the 3-D transition (Re > 190 — beyond what this 2-D LBM
 *       can represent).  Backs the physical-regime statements in
 *       WHAT YOU SEE.
 *
 *  ── Scientific visualisation / colour ──────────────────────────────
 *
 *   [8] Ware, C. (2020) — *Information Visualization: Perception
 *       for Design*, 4th ed., Morgan Kaufmann.  Ch. 4 on perceptual
 *       contrast and SEQUENTIAL colour mapping — backs the
 *       monotone-luminance ramp generated from each theme's
 *       (vel_cold, vel_hot) anchor pair for the velocity heatmap.
 *
 *   [9] Brewer, C. A. (1994) — "Color Use Guidelines for Mapping
 *       and Visualization", in *Visualization in Modern
 *       Cartography*, Pergamon, 123–147.  Defines the sequential /
 *       DIVERGING / qualitative colour-map taxonomy — backs the
 *       two-hue divergent encoding for signed vorticity (one hue
 *       per rotation direction, intensity = magnitude).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Controls:
 *    q/ESC   quit          space   pause/resume
 *    v       velocity mode  o       vorticity mode   d  density mode
 *    s       streamline overlay toggle
 *    +/-     increase/decrease tau (viscosity)
 *    w/W     speed up/slow inlet velocity
 *    n/p     next/previous obstacle pattern (10 shapes)
 *    t/T     next/previous theme (10 four-anchor palettes)
 *    r       reset simulation
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/lattice_boltzman_fluid_simulator.c \
 *        -o lbm_fluid -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config    §2 clock    §3 color    §4 D2Q9 model
 *  §5 grid      §6 collide  §7 stream   §8 macroscopic
 *  §9 vorticity §10 render  §11 overlay §12 scene
 *  §13 screen   §14 app
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_DEFAULT = 60,
  TARGET_FPS = 30, /* render rate — LBM is expensive per cell   */
  FPS_UPDATE_MS = 500,

  /*
   * STEPS_PER_FRAME = 4
   *
   * LBM is expensive: every step visits every cell (nx×ny nodes).
   * Running multiple physics steps per render frame decouples the
   * physical evolution rate from the display refresh rate.  With 4
   * steps per frame at 30 fps we effectively advance the simulation
   * at 120 physics steps/second while only painting 30 frames/second.
   * Fewer steps → simulation evolves too slowly (boring).
   * More steps  → each frame takes too long, fps drops.
   */
  STEPS_PER_FRAME = 4,

  /* Visualization modes */
  VIS_VELOCITY = 0,
  VIS_VORTICITY = 1,
  VIS_DENSITY = 2,
  VIS_COUNT = 3,

  /* Color pair IDs.  CP_VEL0..7 are theme-dependent (regenerated by
   * theme_apply via the 4-anchor ramp); CP_VNEG / CP_VPOS are theme-
   * dependent single pairs (intensity comes from A_DIM/NORMAL/BOLD,
   * not from extra pairs); CP_HUD / CP_HINT / CP_SOLID / CP_ARROW
   * are fixed across themes. */
  CP_VEL0 = 1, /* coldest end of velocity ramp                    */
  CP_VEL1 = 2,
  CP_VEL2 = 3,
  CP_VEL3 = 4,
  CP_VEL4 = 5,
  CP_VEL5 = 6,
  CP_VEL6 = 7,
  CP_VEL7 = 8,   /* hottest end of velocity ramp                    */
  CP_VNEG = 9,   /* vorticity clockwise — 3 levels via attr         */
  CP_VPOS = 10,  /* vorticity CCW       — 3 levels via attr         */
  CP_SOLID = 11, /* obstacle / wall (theme-independent gray)        */
  CP_HUD = 12,   /* status bar — canonical bright yellow (226)      */
  CP_HINT = 13,  /* action keys — canonical bright cyan  (51)       */
  CP_ARROW = 14, /* streamline arrows                                */
};

/*
 * U_IN_DEFAULT = 0.10  (lattice units)
 *
 * The inlet velocity expressed in lattice units (lu/step).  The LBM
 * low-Mach assumption requires u ≪ cs = 1/√3 ≈ 0.577.  Above u ≈ 0.3
 * compressibility errors in the weakly-compressible LBM grow rapidly.
 * At the default value the lattice Mach number is:
 *   Ma = u / cs = 0.10 / (1/√3) ≈ 0.17  (safely subsonic)
 * Interactive key 'w' multiplies by 1.2 (capped at 0.30), 'W' divides.
 */
#define U_IN_DEFAULT 0.10f /* inlet velocity (lattice units, must be ≪1) */

/*
 * TAU_DEFAULT = 0.60
 *
 * The BGK relaxation time τ controls how quickly distributions relax
 * toward equilibrium.  It maps directly to kinematic viscosity:
 *   ν = (τ − 0.5) / 3  →  ν = (0.6 − 0.5) / 3 ≈ 0.0333
 * At this viscosity and the default cylinder size (Re ≈ 50) the flow
 * sits just above the vortex-shedding onset (Re ≈ 47).  The expected
 * Strouhal number St ≈ 0.2, so shedding frequency f ≈ 0.2·U/D ≈ 0.25
 * steps⁻¹ — visible as a rhythmic alternating vortex street.
 */
#define TAU_DEFAULT 0.60f /* relaxation time: ν = (τ−0.5)/3             */

/*
 * TAU_MIN = 0.505
 *
 * The hard lower bound on τ.  When τ < 0.5, the formula ν = (τ−0.5)/3
 * yields a negative kinematic viscosity — physically meaningless and
 * numerically unconditionally unstable (distributions diverge in one
 * step).  The margin of 0.005 above 0.5 keeps ν small but positive,
 * providing a thin cushion against rounding pushing it below zero.
 */
#define TAU_MIN 0.505f /* below 0.5 → unstable                        */

/*
 * TAU_MAX = 2.0
 *
 * The upper bound on τ.  At τ = 2.0:
 *   ν = (2.0 − 0.5) / 3 = 0.5  — very high viscosity.
 * Flow becomes extremely laminar; Re drops so low that no vortex
 * shedding occurs and the simulation looks like thick honey flowing
 * around the cylinder.  Useful as an upper pedagogical reference.
 */
#define TAU_MAX 2.0f

/*
 * TAU_STEP = 0.025
 *
 * Interactive increment/decrement step for τ (keys +/-).
 * Small enough that each keypress changes ν by only Δν = 0.025/3 ≈ 0.008,
 * avoiding large sudden jumps in Reynolds number that could destabilize
 * an already-running simulation.
 */
#define TAU_STEP 0.025f

/*
 * CYL_X_FRAC = 0.25
 *
 * Cylinder center placed at 25% of the grid width from the left.
 * This gives enough upstream distance for the inlet equilibrium
 * profile to establish itself before hitting the obstacle, and leaves
 * ~75% of the domain downstream for the vortex street to develop and
 * convect away without reflecting off the outlet boundary.
 */
#define CYL_X_FRAC 0.25f /* x position as fraction of grid width        */

/*
 * CYL_Y_FRAC = 0.50
 *
 * Cylinder centered vertically.  A perfectly centered obstacle in a
 * symmetric channel will NOT shed vortices spontaneously — symmetry
 * would be preserved forever.  The tiny random perturbation added in
 * lbm_init() breaks this symmetry so shedding eventually develops.
 */
#define CYL_Y_FRAC 0.50f /* y position as fraction of grid height        */

/*
 * CYL_R_FRAC = 0.08
 *
 * Cylinder radius = 8% of grid height, so diameter D = 2×0.08×ny = 0.16·ny.
 * Reynolds number estimate with default parameters:
 *   Re = U·D / ν = 0.10 × (0.16·ny) / 0.0333 ≈ 0.48·ny
 * On a typical 50-row terminal that gives Re ≈ 24; on an 80-row terminal
 * Re ≈ 38; on a 120-row terminal Re ≈ 58 — straddling the shedding
 * threshold Re ≈ 47 nicely for medium-to-large terminals.
 * (Smaller terminals will show steady attached flow; larger ones will
 *  show clear vortex shedding.)
 */
#define CYL_R_FRAC 0.08f /* radius as fraction of grid height            */

/* Velocity display chars: 8 ASCII levels (space = no flow, @ = max)        */
static const char VEL_CHARS[8] = {' ', '.', ':', '+', 'x', 'X', '#', '@'};

/* Arrow glyphs for streamline overlay (8 directions, 0=E going CCW)        */
static const char ARROW8[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

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
  struct timespec req = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Theme — an 8-colour velocity ramp.  That's the whole theme.
 *
 * Each row of THEMES[] is literally 8 numbers — 256-colour palette
 * indices forming the velocity heat-ramp from "coldest" (ramp[0])
 * to "hottest" (ramp[7]).  Pick a theme = pick 8 nice colours that
 * go cold → hot.  No anchor fields, no interpolation, no generator.
 *
 * Vorticity reuses the ramp's endpoints:
 *   CW  rotation  →  ramp[0]  (coldest)
 *   CCW rotation  →  ramp[7]  (hottest)
 * So the velocity heatmap and the vorticity diverging map share the
 * same palette story — guaranteed coherent across all 10 themes.
 *
 * WHY one CP per vorticity sign (not 3 per sign):
 *   render_vorticity expresses intensity via A_DIM / A_NORMAL / A_BOLD
 *   on a single colour pair, so 3 levels need only one pair each.
 *   Intensity = brightness; sign = hue.
 *
 * Fixed pairs across all themes:
 *   CP_HUD   — status bar         bright yellow 226 (CLAUDE.md canonical)
 *   CP_HINT  — action keys        bright cyan   51  (CLAUDE.md canonical)
 *   CP_SOLID — obstacle / walls   mid gray      240 (theme-neutral)
 *   CP_ARROW — streamline arrows  near-white    255 (theme-neutral)
 */
typedef struct {
  const char *name;
  short ramp[8]; /* 8-step velocity heat ramp, cold → hot           */
} Theme;

/*
 * 10 themes — hand-picked 8-step ramps in the 256-colour palette.
 *
 *   Ocean    — deep blue   → bright cyan
 *   Sunset   — indigo      → gold
 *   Forest   — dark green  → yellow-green
 *   Fire     — dark red    → bright yellow
 *   Lava     — navy        → bright red
 *   Aurora   — purple      → green (through blue / cyan)
 *   Plasma   — magenta     → yellow (electric)
 *   Arctic   — dusky gray  → ice cyan
 *   Mono     — pure greyscale
 *   Toxic    — dark green  → lime
 *
 * Numbers are xterm-256 palette indices.  Edit by eye: pick 8 colours
 * along a smooth path in the 6×6×6 cube (16..231) or in the greyscale
 * ramp (232..255) and the gradient will read cleanly.
 */
static const Theme THEMES[] = {
    /* name       0    1    2    3    4    5    6    7      (cold → hot) */
    {"Ocean", {17, 18, 24, 25, 31, 38, 45, 51}},
    {"Sunset", {17, 53, 91, 161, 167, 173, 215, 220}},
    {"Forest", {22, 28, 34, 64, 100, 142, 178, 226}},
    {"Fire", {52, 88, 124, 160, 196, 202, 208, 220}},
    {"Lava", {18, 19, 91, 125, 161, 196, 202, 226}},
    {"Aurora", {54, 91, 99, 105, 81, 49, 47, 82}},
    {"Plasma", {53, 91, 161, 199, 207, 213, 220, 226}},
    {"Arctic", {59, 60, 67, 74, 81, 123, 159, 195}},
    {"Mono", {232, 235, 238, 242, 246, 250, 253, 255}},
    {"Toxic", {22, 28, 34, 40, 76, 112, 118, 154}},
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/* Active theme index — cycled by t/T keys.  File-scope because the
 * theme is a pure rendering choice with no place in the LBM physics
 * state, and a single global keeps theme_apply() callable from both
 * startup (color_init) and the runtime key handler. */
static int g_theme_idx = 0;

/*
 * theme_apply — rebuild the THEME-DEPENDENT colour pairs for theme
 * `idx`.  Just copies ramp[i] → CP_VEL0+i and uses the ramp endpoints
 * for the two vorticity pairs.  Fixed pairs (HUD / HINT / SOLID /
 * ARROW) are NOT touched — they live in color_init().
 *
 * On 8-colour terminals the 256-cube isn't available; we fall back to
 * a fixed blue→red ANSI ramp so the demo still runs.
 */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    return;
  const Theme *t = &THEMES[idx];

  if (COLORS >= 256) {
    for (int i = 0; i < 8; i++)
      init_pair((short)(CP_VEL0 + i), t->ramp[i], -1);
    init_pair(CP_VNEG, t->ramp[0], -1); /* CW: coldest hue  */
    init_pair(CP_VPOS, t->ramp[7], -1); /* CCW: hottest hue */
  } else {
    /* 8-colour ANSI fallback — themes collapse to one fixed ramp */
    init_pair(CP_VEL0, COLOR_BLUE, -1);
    init_pair(CP_VEL1, COLOR_BLUE, -1);
    init_pair(CP_VEL2, COLOR_CYAN, -1);
    init_pair(CP_VEL3, COLOR_CYAN, -1);
    init_pair(CP_VEL4, COLOR_GREEN, -1);
    init_pair(CP_VEL5, COLOR_YELLOW, -1);
    init_pair(CP_VEL6, COLOR_RED, -1);
    init_pair(CP_VEL7, COLOR_RED, -1);
    init_pair(CP_VNEG, COLOR_BLUE, -1);
    init_pair(CP_VPOS, COLOR_RED, -1);
  }
}

/*
 * color_init — one-shot at startup.  Initialises ncurses colour state,
 * binds the four FIXED pairs (HUD, HINT, SOLID, ARROW) once, then
 * delegates the theme-dependent pairs to theme_apply().
 */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1);   /* canonical bright yellow */
    init_pair(CP_HINT, 51, -1);   /* canonical bright cyan   */
    init_pair(CP_SOLID, 240, -1); /* mid gray (theme-neutral)*/
    init_pair(CP_ARROW, 255, -1); /* near-white               */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_SOLID, COLOR_WHITE, -1);
    init_pair(CP_ARROW, COLOR_WHITE, -1);
  }

  theme_apply(g_theme_idx);
}

/* ===================================================================== */
/* §4  D2Q9 model constants                                               */
/* ===================================================================== */

/*
 * D2Q9 lattice velocities and weights.
 *
 *   Dir   (ex, ey)   weight      Opposite
 *    0    ( 0,  0)   4/9         0
 *    1    ( 1,  0)   1/9         3
 *    2    ( 0,  1)   1/9         4   (+y = screen-down)
 *    3    (-1,  0)   1/9         1
 *    4    ( 0, -1)   1/9         2
 *    5    ( 1,  1)   1/36        7
 *    6    (-1,  1)   1/36        8
 *    7    (-1, -1)   1/36        5
 *    8    ( 1, -1)   1/36        6
 */
static const int EX[9] = {0, 1, 0, -1, 0, 1, -1, -1, 1};
static const int EY[9] = {0, 0, 1, 0, -1, 1, 1, -1, -1};
static const float W[9] = {4.0f / 9.0f,  1.0f / 9.0f,  1.0f / 9.0f,
                           1.0f / 9.0f,  1.0f / 9.0f,  1.0f / 36.0f,
                           1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f};
static const int OPP[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

/*
 * feq() — Maxwell-Boltzmann equilibrium at density rho, velocity (ux,uy).
 *
 *   f^eq_i = w_i · ρ · [1 + 3(e·u) + 4.5(e·u)² − 1.5|u|²]
 *
 * --- Derivation of cs² = 1/3 ---
 * In the D2Q9 quadrature the lattice vectors have components ±1 or 0.
 * The thermal speed is fixed so that Σ_i W[i] * EX[i]² = 1/3 (the
 * x-variance of the weight distribution), giving cs² = 1/3.
 *
 * --- Coefficient derivation from cs² = 1/3 ---
 * The full equilibrium is:
 *   f^eq_i = w·ρ·[ 1 + (e·u)/cs² + (e·u)²/(2cs⁴) − u²/(2cs²) ]
 * Substituting cs² = 1/3:
 *   1/cs²  = 3          → coefficient of the linear momentum term (e·u)
 *   1/(2cs⁴) = 4.5      → coefficient of the quadratic term (e·u)²
 *   1/(2cs²) = 1.5      → coefficient of the speed-magnitude term u²
 *
 * --- Physical meaning of each term ---
 *   1            : rest contribution — even at zero flow a node has mass
 *   3(e·u)       : momentum bias — more particles move in the flow direction
 *   4.5(e·u)²    : momentum correction — accounts for the kinetic energy
 *                  carried by the directed motion (makes f^eq non-negative)
 *   −1.5u²       : velocity-magnitude correction — subtracts the isotropic
 *                  part of kinetic energy so total energy is conserved
 *
 * Together these four terms are the 2nd-order Taylor expansion of the
 * Maxwell-Boltzmann distribution in powers of u/cs.
 */
static inline float feq(int i, float rho, float ux, float uy) {
  float eu = (float)EX[i] * ux + (float)EY[i] * uy;
  float u2 = ux * ux + uy * uy;
  return W[i] * rho * (1.0f + 3.0f * eu + 4.5f * eu * eu - 1.5f * u2);
}

/* ===================================================================== */
/* §5  grid — allocation, initialization, obstacle setup                  */
/* ===================================================================== */

/*
 * LBM — the entire D2Q9 BGK simulation world in one struct.
 *
 * WHY a struct (and not loose globals): the LBM state is naturally a
 *   container — 7 lattice buffers + 3 physics knobs + obstacle
 *   geometry + diagnostic stats + UI latches all share the same
 *   lifetime (born at lbm_alloc, die at lbm_free, refit on resize).
 *   Bundling them lets a single `LBM *g` argument carry the whole
 *   world through collide / stream / compute_macroscopic / render
 *   without 20-argument function signatures.
 *
 * WHY ONE struct (not separate SimState + RenderState):
 *   the diagnostic stats (vel_max, rho_min, …) are produced by the
 *   SIMULATION (compute_macroscopic computes them) and consumed by
 *   the RENDERER (normalises colour maps), so splitting would force
 *   either a shared "stats" struct or duplicated pointers.  Keeping
 *   one struct + an internal divider is simpler.
 *
 * LOCALITY INTENT: fields below are grouped into FIVE blocks with
 * divider comments — LATTICE GEOMETRY / LATTICE BUFFERS / PHYSICS /
 * OBSTACLE / DIAGNOSTICS / UI.  Reading the struct top-to-bottom
 * tells you exactly what makes one frame, no scrolling needed.
 *
 *   SIMULATION half (advances the lattice):
 *     nx, ny                       — grid dimensions
 *     f, ftmp                      — distribution functions (double buf)
 *     rho, ux, uy                  — macroscopic fields (moments of f)
 *     vort                         — derived field ω_z = ∇×u
 *     solid                        — obstacle / wall mask
 *     tau, omega, u_in             — physics knobs
 *     cyl_x, cyl_y, cyl_r          — obstacle reference geometry
 *
 *   SHARED (sim writes, renderer + HUD read):
 *     vel_max, rho_min, rho_max,
 *     vort_max, step_count         — per-frame diagnostic stats
 *
 *   RENDERING / UI latches (renderer reads, key handler writes):
 *     vis_mode, show_stream        — view selectors
 *     paused                       — UI freeze flag
 *
 * Field semantics cite: [1, 2] for the LBM container as a whole,
 * [4] for f / f^eq, [3] for τ / ω, [7] for cylinder Re mapping.
 */
typedef struct {

  /* ── SIMULATION: lattice geometry ──────────────────────────────── */

  int nx, ny;
  /* Live grid dimensions.  nx = terminal cols, ny = terminal
   * rows − 2 (top + bottom HUD rows).  Recomputed and the
   * buffers below re-allocated by scene_resize on SIGWINCH.
   * The hot loops in §6 / §7 / §8 / §9 iterate to these — not
   * to any compile-time max — so resize Just Works.            */

  /* ── SIMULATION: lattice buffers (all heap, sized nx×ny) ──────── */

  float *f;
  /* The 9 distribution functions f_i(x,t) for every node,
   * laid out as f[(y·nx + x)·9 + i].  Total memory:
   * 9 × 4 × nx × ny bytes — ~430 KB for 200×60, fits in L2.
   * Cache locality is the WHY behind packing the 9 directions
   * contiguously per node (AoS layout): collide() touches all
   * 9 at one node before moving on. [1, §3] [2, ch.3]         */

  float *ftmp;
  /* Double-buffer scratch for the stream step.  Streaming reads
   * from f, writes to ftmp; we then SWAP THE POINTERS so the
   * caller sees fresh data with no copy.  Without this buffer,
   * a streamed value could be re-streamed in the same pass —
   * a single-threaded race that produces wrong but plausible
   * output. [2, ch.4]                                          */

  float *rho;
  /* Macroscopic density ρ(x) — the ZEROTH moment of f at each
   * node (Σ_i f_i).  Computed by compute_macroscopic each
   * step; consumed by collide (feeds f^eq) and by
   * render_density (heatmap source: p = ρ/3 in LBM units).   */

  float *ux, *uy;
  /* Macroscopic velocity u(x).  FIRST moments of f divided by
   * ρ (Σ_i f_i·e_i / ρ).  Consumed by collide, by
   * compute_vorticity (∇×u), by render_velocity (|u| heatmap),
   * and by render_streamlines (direction arrows).             */

  float *vort;
  /* Vorticity ω_z = ∂uy/∂x − ∂ux/∂y.  ONLY computed when
   * vis_mode == VIS_VORTICITY (saves an O(nx·ny) pass + a
   * second-order finite-diff stencil in the other two modes).
   * Source for render_vorticity.                             */

  uint8_t *solid;
  /* Obstacle / wall mask.  1 = solid (never has meaningful f),
   * 0 = fluid.  Built by the active pattern's pat_* function
   * in §5 (Cylinder, Square, Diamond, Airfoil, …).  Read by
   * stream (bounce-back at solid boundaries), by collide (skip
   * solid cells — they have no fluid), and by every render
   * mode (paint solids as '#'). [2, ch.5 bounce-back]        */

  /* ── SIMULATION: physics parameters ────────────────────────────── */

  float tau;
  /* BGK relaxation time τ in lattice units.  Single-knob
   * control of viscosity via the Chapman-Enskog result
   * ν = cs²(τ − ½) = (τ − ½)/3 [3, 4].  Adjustable via +/-
   * keys, clamped to [TAU_MIN, TAU_MAX].  τ → 0.5 → ν → 0 +
   * numerically unstable; τ → ∞ → very viscous (low Re).    */

  float omega;
  /* Cached 1/τ.  collide() multiplies by omega per cell per
   * step — caching avoids the per-iteration division (~5×
   * slower than a multiply on most FPUs).  Updated together
   * with τ whenever the user changes it.                     */

  float u_in;
  /* Inlet velocity magnitude in lattice units (lu/step).  Must
   * satisfy u ≪ cs = 1/√3 ≈ 0.577 (low-Mach LBM assumption);
   * above u ≈ 0.3 compressibility errors in the weakly-
   * compressible scheme grow rapidly.  Adjustable via w/W,
   * capped at 0.30.                                          */

  /* ── SIMULATION: obstacle reference geometry ───────────────────── */

  int cyl_x, cyl_y, cyl_r;
  /* Canonical obstacle reference point.  Derived once per init
   * from CYL_*_FRAC × grid dims, so every pattern's geometry
   * scales with the terminal.  Every pat_* function in §5 uses
   * these as its size reference — a Square's half-side is
   * cyl_r, a Diamond's vertex distance is cyl_r, etc. — so all
   * 10 patterns occupy comparable terminal real-estate.  The
   * HUD's Reynolds-number estimate uses D = 2·cyl_r as the
   * obstacle diameter. [7]                                    */

  /* ── SHARED: per-frame diagnostic stats ────────────────────────── */
  /* Written by compute_macroscopic + compute_vorticity (SIM side); */
  /* read by the render functions + render_overlay (RENDER side).   */

  float vel_max;
  /* Peak |u| this frame.  Used by render_velocity to normalise
   * the heatmap into [0, vel_max] before mapping to the 8
   * colour tiers of the active theme.                         */

  float rho_min, rho_max;
  /* Density range this frame.  Used by render_density to
   * stretch the colour scale across [min, max] so tiny
   * ~0.001 pressure variations are still visible.            */

  float vort_max;
  /* Peak |ω_z| this frame (only set in vorticity mode).  Used
   * by render_vorticity to normalise the diverging map into
   * three intensity tiers per rotation sign.                 */

  int step_count;
  /* Physics steps elapsed since the last full re-init.  Pure
   * diagnostic — shown in the HUD as "step:N" so the user can
   * tell whether the flow has had enough time to equilibrate
   * after a tuning change (τ, u_in, pattern).                */

  /* ── RENDERING / UI latches ────────────────────────────────────── */
  /* Written by handle_key (RENDER side); read by scene_tick /      */
  /* scene_draw / render_overlay.  These NEVER influence the BGK   */
  /* update — flipping vis_mode does not perturb f or u.           */

  int vis_mode;
  /* Active view: VIS_VELOCITY / VIS_VORTICITY / VIS_DENSITY.
   * Cycled by v/o/d keys.  scene_tick() reads this to decide
   * whether to spend an extra pass on compute_vorticity. */

  bool show_stream;
  /* Streamline overlay toggle (s key).  When true,
   * render_streamlines draws sparse ASCII direction arrows on
   * top of whatever base mode is active.                     */

  bool paused;
  /* When true, scene_tick() returns immediately — physics
   * frozen but the renderer keeps painting (so the user can
   * inspect the lattice without it changing under them).    */
} LBM;

static int lbm_alloc(LBM *g, int nx, int ny) {
  g->nx = nx;
  g->ny = ny;
  int n = nx * ny;
  g->f = calloc(n * 9, sizeof(float));
  g->ftmp = calloc(n * 9, sizeof(float));
  g->rho = calloc(n, sizeof(float));
  g->ux = calloc(n, sizeof(float));
  g->uy = calloc(n, sizeof(float));
  g->vort = calloc(n, sizeof(float));
  g->solid = calloc(n, sizeof(uint8_t));
  return (g->f && g->ftmp && g->rho && g->ux && g->uy && g->vort && g->solid)
             ? 0
             : -1;
}

static void lbm_free(LBM *g) {
  free(g->f);
  free(g->ftmp);
  free(g->rho);
  free(g->ux);
  free(g->uy);
  free(g->vort);
  free(g->solid);
}

/* ── Pattern subsystem — 10 obstacle geometries ─────────────────────── *
 *
 * Each pat_* function fills g->solid[] with a different OBSTACLE
 * configuration; the rest of the simulation (D2Q9, BGK collision,
 * streaming, bounce-back, inlet/outlet) is unchanged.  Switching
 * patterns is a way to compare how the SAME fluid physics produces
 * different flow phenomena around different shapes [2, 7].
 *
 * All patterns share three primitives:
 *   pat_walls(g)             — clear mask + stamp top/bottom walls
 *   stamp_disc(g, cx, cy, r) — filled circular disc
 *   stamp_rect(g, x0..y1)    — filled axis-aligned rectangle
 *
 * Most patterns derive their scale from the (cyl_x, cyl_y, cyl_r)
 * reference point lbm_init() computes from CYL_*_FRAC, so every
 * pattern occupies comparable terminal real-estate regardless of the
 * specific shape — a Square's half-side is cyl_r, a Diamond's vertex
 * distance is cyl_r, etc.                                              */

static void pat_walls(LBM *g) {
  int nx = g->nx, ny = g->ny;
  memset(g->solid, 0, nx * ny);
  for (int x = 0; x < nx; x++) {
    g->solid[0 * nx + x] = 1;
    g->solid[(ny - 1) * nx + x] = 1;
  }
}

static void stamp_disc(LBM *g, int cx, int cy, int r) {
  int nx = g->nx, ny = g->ny;
  int r2 = r * r;
  for (int y = 0; y < ny; y++)
    for (int x = 0; x < nx; x++) {
      int dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy <= r2)
        g->solid[y * nx + x] = 1;
    }
}

static void stamp_rect(LBM *g, int x0, int y0, int x1, int y1) {
  int nx = g->nx, ny = g->ny;
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 >= nx)
    x1 = nx - 1;
  if (y1 >= ny)
    y1 = ny - 1;
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++)
      g->solid[y * nx + x] = 1;
}

/* ── The 10 obstacle patterns ──────────────────────────────────────── */

/* Cylinder — the canonical bluff body.  Above Re ≈ 47 sheds the
 * Kármán vortex street with Strouhal St ≈ 0.2 [7]. */
static void pat_cylinder(LBM *g) {
  pat_walls(g);
  stamp_disc(g, g->cyl_x, g->cyl_y, g->cyl_r);
}

/* Square — sharp leading-edge corners trigger separation immediately
 * (no smooth pressure-rise lead-in).  Wake is wider than a cylinder
 * of equal frontal area; St lower (~0.13). */
static void pat_square(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_rect(g, cx - r, cy - r, cx + r, cy + r);
}

/* Diamond — square rotated 45° (the L1/taxicab ball of radius r).
 * Vertex points upstream so flow splits cleanly at the nose; trailing
 * apex sheds a narrower wake than the square. */
static void pat_diamond(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  for (int y = 0; y < g->ny; y++)
    for (int x = 0; x < g->nx; x++) {
      int dx = x - cx, dy = y - cy;
      int adx = dx < 0 ? -dx : dx;
      int ady = dy < 0 ? -dy : dy;
      if (adx + ady <= r)
        g->solid[y * g->nx + x] = 1;
    }
}

/* Airfoil — tilted ellipse, major axis 3·cyl_r, minor cyl_r/2,
 * angle of attack ~12°.  Asymmetric wake; flow deflects downward
 * — the textbook signature of lift generation. */
static void pat_airfoil(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y;
  float r = (float)g->cyl_r;
  float c = cosf(0.2094f), s = sinf(0.2094f); /* 12° in radians */
  float a = 3.0f * r, b = 0.5f * r;
  float ia2 = 1.0f / (a * a), ib2 = 1.0f / (b * b);
  for (int y = 0; y < g->ny; y++)
    for (int x = 0; x < g->nx; x++) {
      float dx = (float)(x - cx), dy = (float)(y - cy);
      float u = c * dx + s * dy; /* rotate into chord-aligned frame */
      float v = -s * dx + c * dy;
      if (u * u * ia2 + v * v * ib2 <= 1.0f)
        g->solid[y * g->nx + x] = 1;
    }
}

/* Tandem cylinders — two equal cylinders inline, separated by 4·cyl_r.
 * The downstream cylinder sits inside the upstream wake; shedding from
 * the leader is suppressed at small gaps and locked-in at larger ones. */
static void pat_twin(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_disc(g, cx, cy, r);
  stamp_disc(g, cx + 4 * r, cy, r);
}

/* 3×3 cylinder array — porous-medium / tube-bundle proxy.  Small
 * cylinders (cyl_r/2) on a 2·cyl_r grid form local channels between
 * each other; flow accelerates through gaps, recirculates behind each. */
static void pat_array(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  int rs = r / 2;
  if (rs < 1)
    rs = 1;
  for (int ry = -1; ry <= 1; ry++)
    for (int rx = -1; rx <= 1; rx++)
      stamp_disc(g, cx + 2 * r * rx, cy + 2 * r * ry, rs);
}

/* Backward-facing step — solid block at the top-left fills the upper
 * 40% of the channel for the first 2·cyl_x columns, then ends.  The
 * downstream corner traps a stationary recirculation bubble — the
 * canonical separated-flow benchmark. */
static void pat_step(LBM *g) {
  pat_walls(g);
  int step_h = (int)(g->ny * 0.4f);
  stamp_rect(g, 0, 0, 2 * g->cyl_x, step_h);
}

/* Venturi constriction — symmetric wedges from top and bottom narrow
 * the channel to a 2·cyl_r-wide throat at x = cyl_x.  Speed peaks at
 * the throat (mass conservation), density dips (Bernoulli) — most
 * dramatic in DENSITY mode. */
static void pat_venturi(LBM *g) {
  pat_walls(g);
  int nx = g->nx, ny = g->ny;
  int cx = g->cyl_x;
  int half_gap = g->cyl_r;
  int wedge_len = 3 * g->cyl_r;
  int wall_max = ny / 2 - half_gap;
  if (wall_max < 1)
    wall_max = 1;
  for (int x = 0; x < nx; x++) {
    int d = x - cx;
    if (d < 0)
      d = -d;
    if (d > wedge_len)
      continue;
    int taper = wedge_len - d; /* peaks at center */
    int wall = taper * wall_max / wedge_len;
    for (int y = 0; y <= wall; y++)
      g->solid[y * nx + x] = 1;
    for (int y = ny - 1 - wall; y < ny; y++)
      g->solid[y * nx + x] = 1;
  }
}

/* Cylinder + horizontal splitter plate.  The plate (1 cell thick,
 * 4·cyl_r long) extends downstream from the cylinder's rear, preventing
 * the upper and lower shear layers from interacting.  Vortex shedding
 * suppressed — wake becomes steady. */
static void pat_splitter(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  stamp_disc(g, cx, cy, r);
  stamp_rect(g, cx + r, cy, cx + r + 4 * r, cy);
}

/* Triangular wedge pointing upstream — vertex at x = cyl_x − cyl_r,
 * base at x = cyl_x + cyl_r.  Symmetric wake; equally strong shear
 * layers shed in phase from the two trailing-edge corners. */
static void pat_wedge(LBM *g) {
  pat_walls(g);
  int cx = g->cyl_x, cy = g->cyl_y, r = g->cyl_r;
  int x0 = cx - r, x1 = cx + r;
  for (int x = x0; x <= x1; x++) {
    if (x < 0 || x >= g->nx)
      continue;
    int progress = x - x0; /* 0 at vertex, 2r at base */
    int half_h = progress / 2;
    for (int y = cy - half_h; y <= cy + half_h; y++)
      if (y >= 0 && y < g->ny)
        g->solid[y * g->nx + x] = 1;
  }
}

/* ── Pattern table + dispatcher ─────────────────────────────────────── */

/*
 * Pattern — one entry in the n/p obstacle-cycle.
 *
 * WHY a struct of {name, build-fn} (function-pointer table):
 *   the n/p key needs to walk a list of arbitrary obstacle-mask
 *   builders AND show a human label for the current one in the HUD.
 *   Function-pointer + name is the minimum machinery; PATTERNS[]
 *   becomes a single editable table you extend by appending one row
 *   to add a new geometry — no switch statements, no enum-to-string
 *   maps, no per-key branches in handle_key.
 *
 * WHY obstacle patterns matter (the pedagogical point):
 *   the entire D2Q9 BGK fluid physics [4] is UNCHANGED across
 *   patterns — only the solid mask varies.  Watching the same
 *   Reynolds number produce a Kármán street behind a Cylinder, a
 *   wider wake behind a Square, a deflected wake behind an Airfoil
 *   (lift), and a stabilised wake behind a Splitter, makes the
 *   relationship between SHAPE and WAKE PHYSICS legible without
 *   touching a single equation.  Williamson [7] §3 catalogues these
 *   regime differences in real-world wind-tunnel data.
 *
 * WHY build() takes LBM* (not just g->solid): build functions need
 *   to read g->nx, g->ny, and g->cyl_x/y/r to size and place their
 *   geometry; some (pat_array, pat_venturi) also need bounds checks
 *   into the lattice.  Easier than threading those parameters
 *   through three or four separate arguments.
 *
 * Every build function starts by calling pat_walls(g) (clear mask +
 * stamp top/bottom channel walls) so callers don't have to remember.
 */
typedef struct {
  const char *name; /* HUD label: "Cylinder", "Airfoil", …      */
  void (*build)(LBM *g);
  /* fills g->solid[] with this pattern's
   * obstacle geometry; always starts with
   * pat_walls(g) for the channel walls.    */
} Pattern;

static const Pattern PATTERNS[] = {
    {"Cylinder", pat_cylinder}, /* 0 — canonical vortex street      */
    {"Square", pat_square},     /* 1 — sharp leading-edge corners   */
    {"Diamond", pat_diamond},   /* 2 — taxicab/L1 ball, vertex up   */
    {"Airfoil", pat_airfoil},   /* 3 — tilted ellipse, lift demo    */
    {"Twin", pat_twin},         /* 4 — tandem cylinders             */
    {"Array", pat_array},       /* 5 — 3x3 porous-medium proxy      */
    {"Step", pat_step},         /* 6 — backward-facing step         */
    {"Venturi", pat_venturi},   /* 7 — symmetric constriction       */
    {"Splitter", pat_splitter}, /* 8 — cylinder + wake stabiliser   */
    {"Wedge", pat_wedge},       /* 9 — triangular bluff body        */
};
#define N_PATTERNS ((int)(sizeof PATTERNS / sizeof PATTERNS[0]))

/* Active pattern index — cycled by n/p keys.  File-scope so
 * lbm_build_solid() (called from lbm_init()) can read it without
 * threading another parameter through. */
static int g_pattern_idx = 0;

/* Dispatcher — the lbm_init call site doesn't need to know which
 * geometry is active; it just calls lbm_build_solid which forwards
 * to PATTERNS[g_pattern_idx].build. */
static void lbm_build_solid(LBM *g) { PATTERNS[g_pattern_idx].build(g); }

/* ── lbm_init step-helpers ──────────────────────────────────────────── *
 *
 * lbm_init is a five-step recipe — tune the physics, reset the UI,
 * compute obstacle geometry, build the solid mask (via the active
 * pattern), and seed the fluid to equilibrium.  Each step gets a
 * named helper so the orchestrator reads top-to-bottom as the recipe.
 *                                                                       */

/* (1) PHYSICS KNOBS — τ, ω = 1/τ (cached for the inner collision loop),
 *     and inlet speed u_in (lattice units, must be ≪ cs = 1/√3). */
static void lbm_set_physics_knobs(LBM *g, float tau, float u_in) {
  g->tau = tau;
  g->omega = 1.0f / tau;
  g->u_in = u_in;
}

/* (2) UI / VIEW LATCHES — defaults.  pattern_apply() restores any
 *     previously-active view on top of these after init, so n/p
 *     pattern switching doesn't kick the user out of vorticity mode. */
static void lbm_reset_ui_state(LBM *g) {
  g->vis_mode = VIS_VELOCITY;
  g->show_stream = false;
  g->paused = false;
  g->step_count = 0;
}

/* (3) OBSTACLE REFERENCE GEOMETRY — derive (cyl_x, cyl_y, cyl_r) from
 *     the CYL_*_FRAC constants × current grid dims.  Every pat_*
 *     function in §5 uses these as its size reference, so the 10
 *     obstacle shapes occupy comparable terminal real-estate. */
static void lbm_compute_obstacle_geometry(LBM *g) {
  g->cyl_x = (int)(g->nx * CYL_X_FRAC);
  g->cyl_y = (int)(g->ny * CYL_Y_FRAC);
  g->cyl_r = (int)(g->ny * CYL_R_FRAC);
  if (g->cyl_r < 2)
    g->cyl_r = 2;
}

/* (5) FLUID INITIALISATION — fill every fluid node with the Maxwell-
 *     Boltzmann equilibrium f^eq at (ρ=1, u=u_in, v≈0).  A tiny
 *     random uy perturbation breaks top/bottom symmetry of the
 *     channel so vortex shedding can develop — a perfectly symmetric
 *     channel preserves symmetry forever (the Hopf bifurcation [7]
 *     needs a finite-amplitude seed to break it).
 *
 *     Fixed RNG seed → reproducible runs: same pattern + same τ +
 *     same u_in always produces the same wake at the same step. */
static void lbm_seed_fluid_to_equilibrium(LBM *g) {
  int nx = g->nx, ny = g->ny;
  srand(12345);
  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      int idx = y * nx + x;
      float perturb =
          g->solid[idx] ? 0.0f : (float)(rand() % 200 - 100) * 0.00005f;
      for (int i = 0; i < 9; i++)
        g->f[idx * 9 + i] = feq(i, 1.0f, g->u_in, perturb);
      g->rho[idx] = 1.0f;
      g->ux[idx] = g->solid[idx] ? 0.0f : g->u_in;
      g->uy[idx] = g->solid[idx] ? 0.0f : perturb;
    }
  }
}

/*
 * lbm_init — five-step recipe; body reads as the recipe.
 */
static void lbm_init(LBM *g, int nx, int ny, float tau, float u_in) {
  g->nx = nx;
  g->ny = ny;
  lbm_set_physics_knobs(g, tau, u_in); /* (1) tune physics    */
  lbm_reset_ui_state(g);               /* (2) UI defaults     */
  lbm_compute_obstacle_geometry(g);    /* (3) cyl reference   */
  lbm_build_solid(g);                  /* (4) PATTERNS[].build*/
  lbm_seed_fluid_to_equilibrium(g);    /* (5) f → f^eq        */
}

/*
 * pattern_apply — switch to pattern `idx` and re-initialise the fluid
 * to equilibrium around the new obstacle.  USER VIEW STATE (vis_mode,
 * show_stream, paused) is preserved across the reset, so flipping
 * patterns with n/p doesn't kick the user out of vorticity view back
 * to the default velocity view.
 *
 * Physics state (τ, u_in) is also preserved — you keep your tuning.
 *
 * The full lbm_init re-runs because changing the obstacle geometry
 * invalidates the existing distribution functions (a cell that was
 * fluid may now be solid, and vice versa); restarting from equilibrium
 * is the cleanest way to get a physically meaningful state.
 */
static void pattern_apply(LBM *g, int idx) {
  if (idx < 0 || idx >= N_PATTERNS)
    return;
  g_pattern_idx = idx;

  int vm = g->vis_mode;
  bool ss = g->show_stream;
  bool pp = g->paused;
  float tau = g->tau;
  float u_in = g->u_in;

  lbm_init(g, g->nx, g->ny, tau, u_in);

  g->vis_mode = vm;
  g->show_stream = ss;
  g->paused = pp;
}

/* ===================================================================== */
/* §6  collide() — BGK collision step                                     */
/* ===================================================================== */

/*
 * BGK collision: for each fluid cell, relax each distribution f_i
 * toward its local equilibrium value f^eq_i at rate omega = 1/tau.
 *
 * The update rule written two equivalent ways:
 *
 *   f_i* = f_i − ω·(f_i − f^eq_i)        [subtract deviation]
 *         = (1 − ω)·f_i + ω·f^eq_i        [weighted average form]
 *
 * The second form is easier to interpret: the new distribution is a
 * blend of the old value (weight 1−ω) and equilibrium (weight ω).
 * When ω=1 (τ=1) the distribution jumps all the way to equilibrium
 * in one step — very aggressive mixing.  When ω→0 (τ→∞) the
 * distribution barely moves — extremely slow relaxation (high viscosity).
 *
 * Physical picture: "collision" represents molecules at a lattice node
 * exchanging momentum with each other.  After many collisions the local
 * velocity distribution approaches Maxwell-Boltzmann equilibrium.  The
 * rate at which this happens is ω — it IS the viscosity mechanism.
 *
 * Solid cells are skipped: they contain no fluid, so their f values
 * are never meaningful.  The boundary condition (bounce-back) is handled
 * entirely in the stream step.
 */
static void collide(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float omega = g->omega;

  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      int idx = y * nx + x;
      if (g->solid[idx])
        continue;

      float r = g->rho[idx];
      float u = g->ux[idx];
      float v = g->uy[idx];

      float *fi = g->f + idx * 9;
      for (int i = 0; i < 9; i++)
        fi[i] += omega * (feq(i, r, u, v) - fi[i]);
    }
  }
}

/* ===================================================================== */
/* §7  stream — propagation + bounce-back + inlet/outlet BCs              */
/* ===================================================================== */

/* ── stream step-helpers ────────────────────────────────────────────── *
 *
 * The LBM streaming step is five distinct sub-steps; splitting them
 * into named helpers makes the orchestrator (stream() below) read as
 * the textbook recipe:
 *
 *   (1) zero the scratch buffer ftmp
 *   (2) propagate-or-bounce: for each (cell, direction) either
 *       stream to the neighbour or reflect at a solid (bounce-back)
 *   (3) swap pointers f ↔ ftmp (O(1), no copy)
 *   (4) overwrite inlet column with Dirichlet f^eq at u_in
 *   (5) copy outlet column from its neighbour (Neumann, zero-gradient)
 *                                                                       */

/* (1) ftmp is the destination of this pass; previous contents are
 *     stale.  Zero it once so the += accumulations in step (2) start
 *     from a clean slate (a single solid cell can receive multiple
 *     bounced-back distributions from different directions). */
static void stream_clear_scratch(LBM *g) {
  memset(g->ftmp, 0, g->nx * g->ny * 9 * sizeof(float));
}

/* (2) The core of streaming: for every fluid cell and every direction
 *     i, look at the cell at (x+EX[i], y+EY[i]).
 *
 *       NEIGHBOUR is FLUID   → propagate: fnew[neighbour, i] += f[here, i]
 *       NEIGHBOUR is SOLID   → bounce-back: fnew[here, OPP[i]] += f[here, i]
 *                              (the particle reverses direction at the
 *                              wall; sum of incoming + reflected gives
 *                              zero net velocity → no-slip [2, ch.5])
 *
 *     Vertical out-of-bounds (y=0 / y=ny-1) is treated as solid wall.
 *     Horizontal out-of-bounds (x=0 / x=nx-1) is CLAMPED — the inlet
 *     and outlet BCs in steps (4) and (5) overwrite those columns
 *     unconditionally, so any propagation into x<0 or x≥nx is fine
 *     to clamp to the boundary column (the value gets clobbered). */
static void stream_propagate_or_bounce(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float *f = g->f;
  float *fnew = g->ftmp;

  for (int y = 0; y < ny; y++) {
    for (int x = 0; x < nx; x++) {
      if (g->solid[y * nx + x])
        continue;

      float *src = f + (y * nx + x) * 9;

      for (int i = 0; i < 9; i++) {
        int nx2 = x + EX[i];
        int ny2 = y + EY[i];

        bool bounce = (ny2 < 0 || ny2 >= ny);
        if (!bounce && nx2 >= 0 && nx2 < nx)
          bounce = (g->solid[ny2 * nx + nx2] != 0);

        if (bounce) {
          fnew[(y * nx + x) * 9 + OPP[i]] += src[i];
        } else {
          int tx = nx2 < 0 ? 0 : (nx2 >= nx ? nx - 1 : nx2);
          fnew[(ny2 * nx + tx) * 9 + i] += src[i];
        }
      }
    }
  }
}

/* (3) O(1) pointer swap — ftmp now holds the post-stream state, so
 *     swap it into the g->f slot.  No memcpy: the next step expects
 *     to read from g->f, and the next stream() call will overwrite
 *     g->ftmp (the old f) anyway. */
static void stream_swap_buffers(LBM *g) {
  float *tmp = g->f;
  g->f = g->ftmp;
  g->ftmp = tmp;
}

/* (4) INLET Dirichlet BC at x=0.  Overwrite the inlet column with
 *     f^eq at (ρ=1, u=u_in, v=0) — models an infinite upstream
 *     reservoir feeding fluid at the prescribed velocity, regardless
 *     of what the interior delivered to x=0 during step (2). */
static void stream_apply_inlet_bc(LBM *g) {
  int nx = g->nx, ny = g->ny;
  for (int y = 1; y < ny - 1; y++) {
    if (g->solid[y * nx + 0])
      continue;
    float *fi = g->f + (y * nx + 0) * 9;
    for (int i = 0; i < 9; i++)
      fi[i] = feq(i, 1.0f, g->u_in, 0.0f);
  }
}

/* (5) OUTLET Neumann BC at x=nx-1.  Copy f from the penultimate
 *     column (zero-gradient ∂f/∂x = 0).  Simplest open boundary —
 *     prevents pressure waves from reflecting back into the domain,
 *     same idea as the absorbing outlet in the acoustic solver. */
static void stream_apply_outlet_bc(LBM *g) {
  int nx = g->nx, ny = g->ny;
  for (int y = 0; y < ny; y++) {
    float *dst = g->f + (y * nx + (nx - 1)) * 9;
    float *src2 = g->f + (y * nx + (nx - 2)) * 9;
    memcpy(dst, src2, 9 * sizeof(float));
  }
}

/*
 * stream — five-step LBM streaming recipe.  Body reads as the recipe.
 */
static void stream(LBM *g) {
  stream_clear_scratch(g);       /* (1) zero ftmp                */
  stream_propagate_or_bounce(g); /* (2) f → ftmp + bounce-back   */
  stream_swap_buffers(g);        /* (3) f ↔ ftmp                  */
  stream_apply_inlet_bc(g);      /* (4) Dirichlet at x=0          */
  stream_apply_outlet_bc(g);     /* (5) Neumann   at x=nx-1       */
}

/* ===================================================================== */
/* §8  compute_macroscopic — ρ, u from distribution moments               */
/* ===================================================================== */

/* ── compute_macroscopic moment primitives ──────────────────────────── *
 *
 * Recovering (ρ, u) from the distribution functions is the canonical
 * "moment problem": the macroscopic fields are statistical moments of
 * the discrete distribution {f_i}.  Two named primitives express the
 * math; a third handles the solid-cell sentinel values.  Mass and
 * momentum conservation drop out of the moment definitions [1, §3]
 * [2, §3.2].
 *                                                                       */

/* ρ = Σ_i f_i — the ZEROTH moment of f at one node.  Total probability
 * mass at this cell = local fluid density.  Mass conservation. */
static inline float density_zeroth_moment(const float *fi) {
  float r = 0.0f;
  for (int i = 0; i < 9; i++)
    r += fi[i];
  return r;
}

/* u = (Σ_i f_i e_i) / ρ — the FIRST moment of f, normalised by ρ.
 * Σ f_i e_i is momentum density (mass flux); dividing by mass gives
 * velocity.  Momentum conservation, written as velocity. */
static inline void velocity_first_moment(const float *fi, float rho, float *ux,
                                         float *uy) {
  float ru = 0.0f, rv = 0.0f;
  for (int i = 0; i < 9; i++) {
    ru += fi[i] * (float)EX[i];
    rv += fi[i] * (float)EY[i];
  }
  *ux = ru / rho;
  *uy = rv / rho;
}

/* Solid nodes hold no fluid.  Set sentinel values so renderers reading
 * rho/ux/uy don't see garbage and the inlet BC (which uses feq at u_in
 * for fluid cells only) stays sensible at wall-adjacent cells. */
static inline void set_node_solid_defaults(LBM *g, int idx) {
  g->rho[idx] = 1.0f;
  g->ux[idx] = 0.0f;
  g->uy[idx] = 0.0f;
}

/*
 * compute_macroscopic — recover (ρ, u) at every node from f via the
 * zeroth and first moments, AND collect per-frame stats (vel_max,
 * rho_min, rho_max) in the same O(nx·ny) pass for the renderer's
 * colour-map normalisation.
 */
static void compute_macroscopic(LBM *g) {
  float vmax = 0.0f, rmin = 1e9f, rmax = -1e9f;

  for (int y = 0; y < g->ny; y++) {
    for (int x = 0; x < g->nx; x++) {
      int idx = y * g->nx + x;

      if (g->solid[idx]) {
        set_node_solid_defaults(g, idx);
        continue;
      }

      const float *fi = g->f + idx * 9;
      float r = density_zeroth_moment(fi);
      if (r < 1e-6f)
        r = 1e-6f; /* ÷0 guard */
      g->rho[idx] = r;
      velocity_first_moment(fi, r, &g->ux[idx], &g->uy[idx]);

      float vm = sqrtf(g->ux[idx] * g->ux[idx] + g->uy[idx] * g->uy[idx]);
      if (vm > vmax)
        vmax = vm;
      if (r < rmin)
        rmin = r;
      if (r > rmax)
        rmax = r;
    }
  }

  g->vel_max = vmax > 1e-6f ? vmax : 1e-6f;
  g->rho_min = rmin;
  g->rho_max = rmax;
}

/* ===================================================================== */
/* §9  vorticity — discrete curl of the velocity field                    */
/* ===================================================================== */

/*
 * Vorticity is the rotation rate of a fluid element.  In 3D, vorticity
 * is a vector ω = ∇×u.  In 2D flow (our simulation lives entirely in
 * the x-y plane), only the z-component is non-zero:
 *
 *   ω_z = ∂uy/∂x  −  ∂ux/∂y
 *
 * Why only z?  Because u has no z-component, ∂/∂z = 0 everywhere, so:
 *   ωx = ∂uz/∂y − ∂uy/∂z = 0
 *   ωy = ∂ux/∂z − ∂uz/∂x = 0
 *   ωz = ∂uy/∂x − ∂ux/∂y  ← this is all that survives in 2D
 *
 * Positive ωz means counter-clockwise rotation (right-hand rule about z).
 * Negative ωz means clockwise rotation.
 * The Kármán vortex street shows alternating +/− blobs downstream.
 *
 * Approximated with central finite differences on the lattice:
 *   ∂uy/∂x ≈ [uy(x+1,y) − uy(x−1,y)] / 2     (Δx = 1 lattice unit)
 *   ∂ux/∂y ≈ [ux(x,y+1) − ux(x,y−1)] / 2     (Δy = 1 lattice unit)
 *
 * Border cells (y=0, y=ny-1, x=0, x=nx-1) are skipped because central
 * differences need one neighbour on each side.
 */
static void compute_vorticity(LBM *g) {
  int nx = g->nx, ny = g->ny;
  float wmax = 0.0f;

  for (int y = 1; y < ny - 1; y++) {
    for (int x = 1; x < nx - 1; x++) {
      int idx = y * nx + x;
      if (g->solid[idx]) {
        g->vort[idx] = 0.0f;
        continue;
      }
      float duy_dx = (g->uy[y * nx + (x + 1)] - g->uy[y * nx + (x - 1)]) * 0.5f;
      float dux_dy = (g->ux[(y + 1) * nx + x] - g->ux[(y - 1) * nx + x]) * 0.5f;
      float w = duy_dx - dux_dy;
      g->vort[idx] = w;
      float aw = fabsf(w);
      if (aw > wmax)
        wmax = aw;
    }
  }
  g->vort_max = wmax > 1e-8f ? wmax : 1e-8f;
}

/* ===================================================================== */
/* §10  render — velocity, vorticity, density, streamlines                */
/* ===================================================================== */

/*
 * render_velocity() — heat-map of speed |u|.
 *
 * Reveals: where the flow is fast (acceleration around cylinder flanks,
 * jet in the wake) and slow (stagnation point upstream of cylinder,
 * recirculation zone immediately downstream).
 *
 * The speed |u| is normalized to [0, vel_max] and mapped to 8 levels.
 * Color pairs CP_VEL0..CP_VEL7 progress dark-blue → red (cold → hot).
 * attron/attroff calls are kept to the minimum needed per cell — calling
 * them once per cell rather than per-character is a key performance
 * optimization: each ncurses attr change flushes internal state.
 */
static void render_velocity(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float inv = 7.0f / g->vel_max;

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float vm = sqrtf(g->ux[idx] * g->ux[idx] + g->uy[idx] * g->uy[idx]);
      int lv = (int)(vm * inv);
      if (lv < 0)
        lv = 0;
      if (lv > 7)
        lv = 7;

      int cp = CP_VEL0 + lv;
      attr_t at = (lv >= 5) ? A_BOLD : A_NORMAL;
      attron(COLOR_PAIR(cp) | at);
      mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/*
 * render_vorticity() — diverging heatmap of ω_z = curl(u).
 *
 * Reveals: the vortex street — alternating clockwise (blue) and
 * counter-clockwise (red) vortices shed from the cylinder.  This mode
 * makes the Kármán vortex street most visually dramatic.
 *
 * Negative vorticity (clockwise) → blue color pairs CP_VNEG0..2.
 * Positive vorticity (CCW)       → red  color pairs CP_VPOS0..2.
 * 3 intensity levels per sign → 6 color pairs total.
 *
 * attron/attroff: same optimization as render_velocity — one pair of
 * calls per cell, not per character.  A_BOLD is used only for the
 * strongest vortex cores (lv 0 or 5) to make them pop visually.
 */
static void render_vorticity(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float inv = 2.9f / g->vort_max; /* scale to [−3, +3] */

  static const char VOR_CHARS[6] = {'#', 'x', '.', '.', 'x', '#'};
  /* index 0..2 = clockwise (CP_VNEG side), 3..5 = CCW (CP_VPOS side).
   * Intensity is encoded via the attribute, not via a separate colour
   * pair — so all 6 levels share just 2 colour pairs. */
  static const attr_t VOR_ATTR[6] = {
      A_BOLD, A_NORMAL, A_DIM, /* CW: strong → weak */
      A_DIM,  A_NORMAL, A_BOLD /* CCW: weak → strong */
  };

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float w = g->vort[idx] * inv; /* in [−3, +3] roughly */
      int lv;
      if (w < 0.0f) {
        lv = (int)(-w);
        if (lv > 2)
          lv = 2;
        lv = 2 - lv; /* 0=strong, 2=weak */
      } else {
        lv = (int)(w);
        if (lv > 2)
          lv = 2;
        lv = 3 + lv; /* 3=weak, 5=strong */
      }

      short cp = (lv < 3) ? CP_VNEG : CP_VPOS;
      attr_t at = VOR_ATTR[lv];
      attron(COLOR_PAIR(cp) | at);
      mvaddch(y + 1, x, (chtype)VOR_CHARS[lv]);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/*
 * render_density() — pressure / density field.
 *
 * Reveals: pressure distribution.  In the weakly-compressible LBM,
 * pressure p = cs²·ρ = ρ/3, so density deviations from 1.0 ARE pressure
 * variations.  High-density (rho > 1) regions appear in warm colors —
 * these are the high-pressure zones upstream of the cylinder and on
 * stagnation points.  Low-density (rho < 1) regions appear in cool
 * colors — these are the low-pressure cores of shed vortices.
 *
 * The color scale is stretched to span [rho_min, rho_max] each frame,
 * so even tiny pressure differences (typically Δρ ~ 0.001) are visible.
 * Reuses the CP_VEL0..7 color ramp (dark = low pressure, bright = high).
 */
static void render_density(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;
  float rng =
      (g->rho_max - g->rho_min) > 1e-6f ? (g->rho_max - g->rho_min) : 1e-6f;

  for (int y = 0; y < ny && y < rows; y++) {
    for (int x = 0; x < nx && x < cols; x++) {
      int idx = y * nx + x;

      if (g->solid[idx]) {
        attron(COLOR_PAIR(CP_SOLID) | A_DIM);
        mvaddch(y + 1, x, '#');
        attroff(COLOR_PAIR(CP_SOLID) | A_DIM);
        continue;
      }

      float norm = (g->rho[idx] - g->rho_min) / rng;
      int lv = (int)(norm * 7.0f);
      if (lv < 0)
        lv = 0;
      if (lv > 7)
        lv = 7;

      attron(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
      mvaddch(y + 1, x, (chtype)VEL_CHARS[lv]);
      attroff(COLOR_PAIR(CP_VEL0 + lv) | A_NORMAL);
    }
  }
}

/*
 * render_streamlines() — sparse arrow overlay on top of any base render.
 * Sample every STREAM_DX cols / STREAM_DY rows; draw directional arrow
 * where speed > threshold.  Arrow is one of 8 ASCII direction chars.
 */
#define STREAM_DX 4
#define STREAM_DY 2

/*
 * STREAM_THR = 0.005
 *
 * Minimum speed required to draw a streamline arrow.  Set to 5% of
 * U_IN_DEFAULT.  In near-stagnation regions (directly upstream of the
 * cylinder nose, or deep in the recirculation zone) the velocity is
 * nearly zero and its direction is dominated by numerical noise.
 * Drawing arrows there produces a confusing spray of random directions.
 * The threshold suppresses these "noise arrows" while still showing
 * flow direction everywhere the fluid is actually moving.
 */
#define STREAM_THR 0.005f /* only draw arrow if speed above this threshold */

static void render_streamlines(const LBM *g, int cols, int rows) {
  int nx = g->nx, ny = g->ny;

  for (int y = STREAM_DY; y < ny - STREAM_DY && y < rows - 1; y += STREAM_DY) {
    for (int x = STREAM_DX; x < nx - STREAM_DX && x < cols; x += STREAM_DX) {
      int idx = y * nx + x;
      if (g->solid[idx])
        continue;

      float u = g->ux[idx];
      float v = g->uy[idx];
      float spd = sqrtf(u * u + v * v);
      if (spd < STREAM_THR)
        continue;

      /* 8-direction angle: atan2 gives [-π, π]; map to [0,7] */
      float ang = atan2f(v, u); /* screen: +y = down  */
      int dir =
          (int)((ang + (float)M_PI) / (2.0f * (float)M_PI) * 8.0f + 0.5f) & 7;

      attron(COLOR_PAIR(CP_ARROW) | A_BOLD);
      mvaddch(y + 1, x, (chtype)ARROW8[dir]);
      attroff(COLOR_PAIR(CP_ARROW) | A_BOLD);
    }
  }
}

/* ===================================================================== */
/* §11  render_overlay() — HUD status bar                                 */
/* ===================================================================== */

static const char *VIS_NAMES[VIS_COUNT] = {"VELOCITY ", "VORTICITY",
                                           "DENSITY  "};

/*
 * render_overlay — canonical CLAUDE.md two-row HUD.
 *
 *   Row 0           : STATUS — mode, theme, viscosity, Reynolds, density
 *                     range, peak speed, step count, fps, paused state.
 *                     CP_HUD (bright yellow 226) + A_BOLD.
 *   Row rows-1      : ACTION key legend.  CP_HINT (bright cyan 51) +
 *                     A_BOLD — NEVER A_DIM (CLAUDE.md HUD Standard:
 *                     dim text vanishes against bright animation).
 *
 * Pause state appears inline in the status (no centred overlay) so the
 * HUD never occludes the flow.
 */
static void render_overlay(const LBM *g, int cols, int rows, double fps) {
  /* Reynolds estimate: Re = U·D / ν,  ν = (τ−0.5)/3 */
  float nu = (g->tau - 0.5f) / 3.0f;
  float re = (nu > 1e-6f) ? (g->u_in * (float)(2 * g->cyl_r) / nu) : 0.0f;

  /* ── Top row 0: STATUS ───────────────────────────────────────── */
  char top[256];
  snprintf(top, sizeof top,
           " LBM D2Q9  [%s]%s  pat:%s  theme:%s  tau=%.3f nu=%.4f Re~%.0f"
           "  rho[%.3f,%.3f] |u|max=%.4f  step:%-6d  %.0ffps  %s",
           VIS_NAMES[g->vis_mode], g->show_stream ? "+stream" : "       ",
           PATTERNS[g_pattern_idx].name, THEMES[g_theme_idx].name, g->tau, nu,
           re, g->rho_min, g->rho_max, g->vel_max, g->step_count, fps,
           g->paused ? "PAUSED " : "running");

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, 0, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── Bottom row: ACTION keys ─────────────────────────────────── */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  v:vel  o:vort  d:density  s:stream"
           "  n/p:pattern  +/-:tau  w/W:speed  t/T:theme  r:reset ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §12  scene                                                              */
/* ===================================================================== */

/*
 * Scene — thin wrapper around the LBM container.
 *
 * Right now the Scene holds a single LBM, which IS the world state
 * for this demo — the LBM struct already contains both the simulation
 * half (lattice buffers, physics knobs, obstacle geometry, diagnostic
 * stats) and the rendering/UI latches (vis_mode, show_stream, paused).
 *
 * WHY wrap it in a Scene then?  Forward cover.  If we ever need
 *   per-scene state that doesn't belong inside the LBM struct — a
 *   particle-tracer overlay, a recording buffer, a side-by-side
 *   second lattice for A/B comparison — it goes here as a new field
 *   without churning every function signature that already takes
 *   Scene*.  scene_init / scene_tick / scene_draw / scene_resize
 *   form a stable per-frame contract; adding to Scene leaves them
 *   untouched.
 *
 * LOCALITY: the simulation-vs-rendering separation lives INSIDE the
 * LBM struct (see its "SIMULATION half / SHARED / RENDERING half"
 * field groups in §5), not at the Scene level.  Scene is currently
 * just a placeholder ready for future per-scene additions.
 *
 * The full pipeline for one frame:
 *   scene_init   — alloc + first-time lbm_init with the active pattern
 *   scene_tick   — STEPS_PER_FRAME × {macro → collide → stream → macro}
 *                  (+ compute_vorticity in vorticity mode); paused short-
 *                  circuits.
 *   scene_draw   — render_* (per mode) + render_streamlines + render_overlay.
 *   scene_resize — free + alloc + lbm_init at new dims (preserves view).
 *   scene_free   — release all heap buffers.
 */
typedef struct {
  LBM lbm; /* the entire D2Q9 BGK world — see §5 for the field
            * breakdown (lattice / physics / obstacle / stats /
            * UI latches all live in here). */
} Scene;

static void scene_init(Scene *sc, int cols, int rows) {
  LBM *g = &sc->lbm;
  int nx = cols;
  int ny = rows - 2; /* minus top HUD row and bottom key row */
  if (ny < 4)
    ny = 4;

  if (lbm_alloc(g, nx, ny) != 0)
    return;
  lbm_init(g, nx, ny, TAU_DEFAULT, U_IN_DEFAULT);
}

static void scene_free(Scene *sc) { lbm_free(&sc->lbm); }

static void scene_resize(Scene *sc, int cols, int rows) {
  LBM *g = &sc->lbm;
  int vm = g->vis_mode;
  bool ss = g->show_stream;
  float tau = g->tau, u_in = g->u_in;

  lbm_free(g);
  memset(g, 0, sizeof *g);
  g->vis_mode = vm;
  g->show_stream = ss;

  int nx = cols, ny = rows - 2;
  if (ny < 4)
    ny = 4;
  if (lbm_alloc(g, nx, ny) != 0)
    return;
  lbm_init(g, nx, ny, tau, u_in);
}

/*
 * scene_tick() — one LBM time step.
 * Order: macroscopic → collide → stream → macroscopic (for render).
 * Vorticity computed once per step for vorticity mode.
 */
static void scene_tick(Scene *sc) {
  LBM *g = &sc->lbm;
  if (g->paused)
    return;

  compute_macroscopic(g);
  collide(g);
  stream(g);
  compute_macroscopic(g);
  if (g->vis_mode == VIS_VORTICITY)
    compute_vorticity(g);
  g->step_count++;
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps) {
  erase();
  const LBM *g = &sc->lbm;

  switch (g->vis_mode) {
  case VIS_VELOCITY:
    render_velocity(g, cols, rows - 2);
    break;
  case VIS_VORTICITY:
    render_vorticity(g, cols, rows - 2);
    break;
  case VIS_DENSITY:
    render_density(g, cols, rows - 2);
    break;
  }

  if (g->show_stream)
    render_streamlines(g, cols, rows - 2);
  render_overlay(g, cols, rows, fps);
}

/* ===================================================================== */
/* §13  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-geometry container.
 *
 * Distinct from the simulation lattice dims (LBM.nx, LBM.ny) so the
 * two can change INDEPENDENTLY:
 *
 *   - SIGWINCH fires → screen_resize re-reads cols/rows from ncurses
 *     into this struct.  Then scene_resize attempts a re-fit of the
 *     lattice; if that fails (e.g. terminal shrunk below the minimum
 *     ny < 4), the lattice keeps its prior dims and the renderer
 *     just clamps its loops to whatever fits.
 *
 *   - Bottom HUD / top HUD live at screen rows 0 and rows−1; lattice
 *     rendering occupies rows [1, rows−2].  Render functions take
 *     (cols, rows−2) explicitly so they paint into the right band
 *     without knowing the HUD layout.
 *
 * Recomputed by screen_init at startup and by screen_resize on
 * SIGWINCH.  Holds no allocation — it's purely a (cols, rows) pair.
 */
typedef struct {
  int cols, rows; /* current terminal width × height in cells */
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

/* ===================================================================== */
/* §14  app                                                                */
/* ===================================================================== */

/*
 * App — top-level lifecycle container; everything in one place.
 *
 * WHY a struct (and not loose globals): the App lifecycle is well-
 *   defined — born at main() init, dies at main() return.  Bundling
 *   lets a single App* propagate through the main-loop helpers
 *   (handle_key, the resize handler) without a forest of unrelated
 *   globals.
 *
 * WHY signal flags live INSIDE App (with the volatile sig_atomic_t
 * dance):
 *
 *   The C signal-handler signature is `void handler(int)` — no place
 *   to pass an App* into it.  So we keep a SINGLE static App g_app
 *   instance at file scope; on_exit / on_resize write to its flags.
 *
 *   The flags themselves MUST be `volatile sig_atomic_t` because:
 *     - volatile      — the main-loop read can't be optimised away
 *                       (the compiler doesn't know a signal will fire)
 *     - sig_atomic_t  — the C standard guarantees these reads/writes
 *                       are atomic w.r.t. signal interruption; other
 *                       integer types may not be.
 *
 *   Signal handlers only set flags; the main loop reads them and
 *   does the real work (endwin/refresh on resize, clean exit on
 *   quit).  Doing the work IN the handler is unsafe — endwin /
 *   ncurses calls / malloc / free are not signal-safe.
 *
 *   running       — clear (= 0) to break the main loop and exit
 *                   cleanly through atexit(cleanup).
 *   need_resize   — set on SIGWINCH; main loop services it by
 *                   re-fitting Screen + Scene to the new terminal.
 */
typedef struct {
  Scene scene;                       /* §12 — the world container   */
  Screen screen;                     /* §13 — terminal geometry     */
  volatile sig_atomic_t running;     /* SIGINT/SIGTERM → 0 → exit   */
  volatile sig_atomic_t need_resize; /* SIGWINCH → 1 → resize next  */
} App;

static App g_app;
static void on_exit(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static bool handle_key(App *app, int ch) {
  LBM *g = &app->scene.lbm;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    g->paused = !g->paused;
    break;
  case 'v':
  case 'V':
    g->vis_mode = VIS_VELOCITY;
    break;
  case 'o':
  case 'O':
    g->vis_mode = VIS_VORTICITY;
    break;
  case 'd':
  case 'D':
    g->vis_mode = VIS_DENSITY;
    break;
  case 's':
  case 'S':
    g->show_stream = !g->show_stream;
    break;
  case '+':
  case '=':
    g->tau = g->tau + TAU_STEP;
    if (g->tau > TAU_MAX)
      g->tau = TAU_MAX;
    g->omega = 1.0f / g->tau;
    break;
  case '-':
    g->tau = g->tau - TAU_STEP;
    if (g->tau < TAU_MIN)
      g->tau = TAU_MIN;
    g->omega = 1.0f / g->tau;
    break;
  case 'w':
    g->u_in *= 1.2f;
    if (g->u_in > 0.30f)
      g->u_in = 0.30f;
    break;
  case 'W':
    g->u_in /= 1.2f;
    if (g->u_in < 0.01f)
      g->u_in = 0.01f;
    break;
  case 'r':
  case 'R':
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    break;
  case 'n':
    pattern_apply(g, (g_pattern_idx + 1) % N_PATTERNS);
    break;
  case 'p':
    pattern_apply(g, (g_pattern_idx + N_PATTERNS - 1) % N_PATTERNS);
    break;
  case 't':
    g_theme_idx = (g_theme_idx + 1) % N_THEMES;
    theme_apply(g_theme_idx);
    break;
  case 'T':
    g_theme_idx = (g_theme_idx + N_THEMES - 1) % N_THEMES;
    theme_apply(g_theme_idx);
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int fps_count = 0;
  double fps_disp = 0.0;

  while (app->running) {

    /* ── resize ──────────────────────────────────── */
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      frame_time = clock_ns();
    }

    /* ── dt ──────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 200 * NS_PER_MS)
      dt = 200 * NS_PER_MS;

    /* ── physics: multiple LBM steps per frame ───── */
    for (int s = 0; s < STEPS_PER_FRAME; s++)
      scene_tick(&app->scene);

    /* ── fps counter ─────────────────────────────── */
    fps_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_count / ((double)fps_accum / (double)NS_PER_SEC);
      fps_count = 0;
      fps_accum = 0;
    }

    /* ── sleep to cap render at TARGET_FPS ───────── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* ── render ──────────────────────────────────── */
    scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
    wnoutrefresh(stdscr);
    doupdate();

    /* ── input ───────────────────────────────────── */
    int ch;
    while ((ch = getch()) != ERR)
      if (!handle_key(app, ch)) {
        app->running = 0;
        break;
      }
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
