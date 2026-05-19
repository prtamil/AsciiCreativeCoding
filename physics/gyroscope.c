/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * gyroscope.c — Spinning Top / Euler Equations
 *
 * Rigid-body rotation via Euler's equations integrated with RK4.
 * Orientation stored as a unit quaternion — no gimbal lock, no drift.
 * Gram-Schmidt re-orthogonalises the extracted rotation axes each frame
 * as an additional numerical safeguard.
 *
 * Eight presets (cycle with n / p):
 *   0  Euler's Top    — torque-free symmetric top; body Z traces a cone
 *                        around the fixed angular momentum vector
 *   1  Gravity Top    — gravity-driven precession + nutation; the wobble
 *                        tightens as spin rate increases
 *   2  Dzhanibekov    — asymmetric torque-free; rotation near the
 *                        intermediate I axis is unstable → 180° flips
 *                        (tennis-racket / T-handle effect)
 *   3  Oblate Top     — symmetric with I3 > I1=I2; opposite-sign Euler
 *                        precession from preset 0 (body cone outside)
 *   4  Sleeping Top   — gravity + very fast spin; the classic "barely
 *                        wobbles, stays nearly upright" regime
 *   5  Slow Heavy Top — gravity + slow spin; wide precession circles
 *                        with visible nutation cusps
 *   6  Stable Major   — asymmetric, spin on largest-I axis (stable)
 *   7  Stable Minor   — asymmetric, spin on smallest-I axis (stable);
 *                        with preset 2 demonstrates the intermediate-
 *                        axis theorem (only the middle I is unstable)
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config   — presets, constants
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  themes   — 10 palettes + color-pair init (axes/L/ground/trail/disc/HUD)
 *   §4  coords   — CELL_W/H aspect correction; 3-D→2-D projection
 *   §5  entity   — Gyro: physics blocks (gravity / Euler / kinematics)
 *                  → gyro_deriv → RK4 step → drift projection → draw layers
 *   §6  scene
 *   §7  screen
 *   §8  app
 * ─────────────────────────────────────────────────────────────────────
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * State vector (7 floats):  s = [ωx, ωy, ωz, qw, qx, qy, qz]
 *
 * Euler's equations (body frame, with optional gravity torque τ):
 *   I₁ω̇x = (I₂−I₃)ωy·ωz + τx
 *   I₂ω̇y = (I₃−I₁)ωz·ωx + τy
 *   I₃ω̇z = (I₁−I₂)ωx·ωy + τz
 *
 * Gravity torque on a top pivoting at origin, CM at l·ez (body Z):
 *   τ_body = mgl · (gz_by, −gz_bx, 0)
 *   where gz_b = R^T · Ẑworld = world-Z unit vector expressed in
 *   the body frame, read from the rotation matrix derived from q.
 *
 * Quaternion kinematics (ω in body frame):
 *   q̇ = ½ · q ⊗ (0, ωx, ωy, ωz)
 *   → dqw = ½(−qx·ωx − qy·ωy − qz·ωz)
 *      dqx = ½( qw·ωx + qy·ωz − qz·ωy)
 *      dqy = ½( qw·ωy − qx·ωz + qz·ωx)
 *      dqz = ½( qw·ωz + qx·ωy − qy·ωx)
 *
 * After each RK4 step: project_quaternion_to_unit() snaps q back
 * onto S³ (the unit sphere), so R stays in SO(3).
 *
 * SO(3) DRIFT PREVENTION
 * ─────────────────────────────────────────────────────────────────────
 * Two cleanup steps run at the tail of every gyro_step:
 *
 *   project_quaternion_to_unit(q)   — q /= |q|, keeps |q|=1 so the
 *                                     derived rotation matrix R is
 *                                     orthogonal by construction.
 *   refresh_body_axes(g)            — quat_to_axes + gram_schmidt;
 *                                     re-orthonormalises the extracted
 *                                     (ex, ey, ez) triad so float
 *                                     round-off in the quaternion →
 *                                     matrix formula can't accumulate
 *                                     into non-orthogonal frame axes.
 *
 * PROJECTION
 * ─────────────────────────────────────────────────────────────────────
 * Orthographic projection with azimuth φ and elevation θ:
 *   rx =  wx·cos φ + wy·sin φ          (horizontal after azimuth rotation)
 *   ry = −wx·sin φ + wy·cos φ          (depth    after azimuth rotation)
 *   screen_x =  rx
 *   screen_y =  ry·cos θ + wz·sin θ    (up on screen)
 *   depth    = −ry·sin θ + wz·cos θ    (depth, used for shading)
 * Terminal column = cx + screen_x · scale
 * Terminal row    = cy − screen_y · scale · ASPECT   (ASPECT≈0.5)
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume
 *   n / p        next / previous preset
 *   g            toggle gravity (affects any preset)
 *   l            toggle polhode trail   (the trace line)
 *   t / T        next / previous theme
 *   ← →          rotate view azimuth
 *   ↑ ↓          tilt view elevation
 *   r            restart preset
 *   ] / [        raise / lower sim Hz
 *
 * Themes (12 bipolar 8-step ramps shared with charged_particles.c):
 *   0 VOLT     1 COPPER   2 NEON     3 ICE_FIRE  4 AURORA   5 VIOLET
 *   6 CYBER    7 PASTEL   8 TWILIGHT 9 SODIUM   10 ECLIPSE 11 MONO
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/gyroscope.c -o gyroscope -lncurses
 * -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : RK4 [6] integration of a 7-component state vector.
 *                  [ωx, ωy, ωz] — angular velocity in body frame (rad/s)
 *                  [qw, qx, qy, qz] — orientation quaternion (unit length)
 *                  RK4 is used because Euler's equations are nonlinear:
 *                  the cross-products (I₂−I₃)ωy·ωz etc. couple the
 *                  components, and explicit Euler drifts visibly within
 *                  a few seconds of simulation.  SUB_STEPS=8 sub-steps
 *                  per render frame keep dt small relative to the fastest
 *                  oscillation period in any preset.
 *
 * Physics        : Euler's equations of rigid-body rotation [1, 2].
 *                  In the body frame (principal axes) the inertia tensor
 *                  is diagonal with eigenvalues I₁, I₂, I₃.  Three
 *                  classical regimes are realised across the 8 presets:
 *                    • Symmetric torque-free top (Euler precession) [1]
 *                          — presets 0, 3
 *                    • Heavy symmetric top with gravity (Lagrange top,
 *                          precession + nutation, sleeping stability
 *                          ω² > 4·mgl·I₁/I₃²) [1, 2]
 *                          — presets 1, 4, 5
 *                    • Intermediate-axis instability [3]
 *                          — preset 2 (unstable) versus 6, 7 (stable):
 *                          rotation about the MIDDLE principal axis is
 *                          unstable, while rotation about the smallest
 *                          or largest is stable (tennis-racket /
 *                          Dzhanibekov theorem).
 *
 * Math           : Quaternion orientation tracking [4].
 *                  A unit quaternion q = (qw, qx, qy, qz) encodes 3-D
 *                  rotation without gimbal lock or singularities.  The
 *                  rotation matrix R(q) is derived analytically in
 *                  quat_to_axes (§5).  Quaternion kinematics
 *                    q̇ = ½ · q ⊗ (0, ω)        (quaternion_kinematics)
 *                  live on the unit sphere S³ ⊂ ℝ⁴; RK4 drifts off the
 *                  sphere by O(dt⁵) per step, so project_quaternion_
 *                  to_unit snaps q back after every step — the cheapest
 *                  case of geometric-integration projection [5].
 *                  refresh_body_axes runs gram_schmidt for the same
 *                  reason: keep R numerically in SO(3), not just
 *                  analytically.
 *
 * Rendering      : Orthographic projection of a 3-D wireframe [8] in
 *                  six painter's-order layers (draw_ground_ring →
 *                  draw_world_z_reference → draw_body_disc_equator →
 *                  draw_polhode_trail → draw_body_axes_depth_sorted →
 *                  draw_angular_momentum_vector).  All projection
 *                  parameters bundled into a Viewport struct so each
 *                  layer takes the same context.  ASPECT = CELL_W /
 *                  CELL_H ≈ 0.5 compensates for non-square terminal
 *                  cells so circles appear round, not elliptical.
 *                  Edge glyphs picked by 8-sector quantisation of the
 *                  screen-space angle [7] (one of {-, /, |, \} per
 *                  45° sector, in dir_char).  Depth-cueing via A_BOLD
 *                  (near, depth < 0) / A_DIM (far) gives a z-buffer
 *                  effect at zero memory cost.
 *
 * Themes         : 12 bipolar diverging palettes [9] — each is an
 *                  8-step cool family → bright neutral → warm family
 *                  ramp.  Body X / Y get cool-mid / warm-mid hues so
 *                  they're distinguishable at a glance; body Z (spin
 *                  axis) and L (angular momentum) get the saturated
 *                  extremes so the physics protagonists read first
 *                  regardless of which theme is active.
 *
 * References (cite inline as [n]):
 *
 *   [1] Goldstein, H.; Poole, C.; Safko, J. — *Classical Mechanics*,
 *       3rd ed., Addison-Wesley (2002).  The canonical English-language
 *       reference for rigid-body rotation: Ch. 4 (kinematics, Euler
 *       angles), Ch. 5 (Euler's equations, symmetric top, heavy
 *       symmetric top, sleeping-top stability threshold, Lagrange
 *       top).  Backs presets 0, 1, 3, 4, 5.
 *
 *   [2] Landau, L. D.; Lifshitz, E. M. — *Mechanics*, Vol. 1 of the
 *       Course of Theoretical Physics, 3rd ed., Butterworth-Heinemann
 *       (1976).  §§32-37 cover the rigid body — concise, elegant
 *       derivations of Euler's equations and the symmetric top with
 *       and without gravity.  Cross-check / second source for [1].
 *
 *   [3] Ashbaugh, M. S.; Chicone, C. C.; Cushman, R. H. (1991) —
 *       "The Twisting Tennis Racket", *J. Dynamics and Differential
 *       Equations* 3 (1), 67–85.  Rigorous analysis of the
 *       Dzhanibekov / intermediate-axis instability: proves rotation
 *       about the middle principal axis is unstable and derives the
 *       flip period from the inertia eigenvalues.  Backs preset 2
 *       and the comparison-stability presets 6 / 7.
 *
 *   [4] Diebel, J. (2006) — "Representing Attitude: Euler Angles,
 *       Unit Quaternions, and Rotation Vectors", Stanford tech report.
 *       Practical compendium of every formula needed to manipulate
 *       attitude representations: q ↔ R conversion, kinematics
 *       q̇ = ½ q ⊗ (0, ω), composition rules.  gyro_deriv and
 *       quat_to_axes in §5 follow this report's conventions verbatim.
 *
 *   [5] Hairer, E.; Lubich, C.; Wanner, G. — *Geometric Numerical
 *       Integration*, 2nd ed., Springer (2006).  Structure-preserving
 *       integrators for mechanical systems on manifolds.  Explains
 *       why naive RK4 drifts off SO(3) / S³ and why a simple
 *       projection step (q /= |q| + Gram-Schmidt on the extracted
 *       axes) is sufficient for short-horizon integration — the
 *       rationale for the post-step cleanup in gyro_step.
 *
 *   [6] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.;
 *       Flannery, B. P. — *Numerical Recipes*, 3rd ed., Cambridge
 *       University Press (2007).  Ch. 17 on ODE integration — the
 *       classical RK4 coefficients (k1..k4,
 *       y_{n+1} = y_n + (k1 + 2k2 + 2k3 + k4) · dt/6) implemented
 *       verbatim in gyro_step.
 *
 *   [7] Bresenham, J. E. (1965) — "Algorithm for Computer Control of
 *       a Digital Plotter", *IBM Systems Journal* 4 (1), 25–30.
 *       8-direction line discretisation — 360° quantised into eight
 *       45° sectors, each mapped to one of {-, /, |, \}.  Used in
 *       dir_char (§4) and provides the DDA stepping inside
 *       draw_seg3d.
 *
 *   [8] Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes, J. F. —
 *       *Computer Graphics: Principles and Practice*, 3rd ed.,
 *       Addison-Wesley (2013).  Reference for 3-D viewing transforms,
 *       orthographic projection, and depth cueing.  Backs the
 *       project() function in §4 (azimuth + elevation rotation, then
 *       drop the depth coordinate for the screen, retain it for the
 *       bold/dim shading).
 *
 *   [9] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020).  Ch. 4 on colour and
 *       perception: why DIVERGING palettes (cool family ↔ warm family
 *       with a bright neutral midpoint) are the canonical choice for
 *       signed / bipolar data.  Backs the 12-theme palette in §3
 *       (shared verbatim with charged_particles.c).
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

enum {
  SIM_FPS_MIN = 20,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SUB_STEPS = 8,   /* RK4 sub-steps per sim tick (stability) */
  TRAIL_LEN = 300, /* polhode trail ring-buffer length        */
  DISC_PTS = 32,   /* points on the body-disc equator         */
  GROUND_PTS = 48, /* points on the ground reference ring     */
  FPS_UPDATE_MS = 500,
  N_COLORS = 7,
  N_PRESETS = 8,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── Preset table ────────────────────────────────────────────────────── */

/*
 * GPreset — initial-conditions record for one demonstration scenario.
 *
 * A preset is the complete set of parameters needed to reproduce a
 * named classical case (Euler's top, Dzhanibekov, sleeping top, ...).
 * PRESETS[] holds the 8 entries; gyro_set_preset() loads a chosen
 * entry into the live Gyro state at startup and on r-reset.
 *
 * Why a flat record (not a class with virtual methods):
 *   Every preset answers the same two questions — "what initial state?"
 *   and "what torque law?" — so a tagged-union or polymorphism would
 *   just add ceremony.  Add a new preset by appending to PRESETS[] and
 *   bumping N_PRESETS.
 *
 * Field semantics (Euler's equations [1, 2]):
 *
 *   Inertia tensor (diagonal in principal axes [1]):
 *     I1, I2, I3 — principal moments along body X, Y, Z (kg·m²).
 *                  Equal pair → SYMMETRIC top (Euler's-top family);
 *                  all distinct → ASYMMETRIC body (Dzhanibekov family
 *                  — preset 2, 6, 7).
 *
 *   Initial state (body frame):
 *     omega[3]   — initial angular velocity ω₀ in rad/s; the component
 *                  distribution chooses the spin regime — pure ω_z →
 *                  pure-spin top, large ω_y on asymmetric body →
 *                  intermediate-axis instability [3].
 *     tilt_deg   — initial nutation angle: rotates body-Z away from
 *                  world-Z by this angle, around tilt_axis.  Needed
 *                  for any gravity preset (gravity torque vanishes
 *                  when the top is exactly upright).
 *     tilt_axis  — world unit-vector to tilt around (auto-normalised
 *                  in gyro_set_preset if not exactly unit length).
 *
 *   External torque:
 *     gravity    — false → torque-free (Euler's equations alone);
 *                  true → add Lagrange-top gravity torque [1].
 *     mgl        — mass × g × l (N·m); strength of the gravity torque
 *                  when gravity is on.  Drives the precession rate
 *                  ω_p ≈ mgl / (I₃·ω_z) of a sleeping-stable
 *                  symmetric top.
 */
typedef struct {
  const char *name;   /* preset display name (HUD row 1)             */
  float I1, I2, I3;   /* principal moments of inertia                */
  float omega[3];     /* initial angular velocity, body frame (rad/s)*/
  float tilt_deg;     /* initial nutation: tilt body-Z from world-Z  */
  float tilt_axis[3]; /* world axis to tilt around (auto-normalised) */
  bool gravity;       /* false=Euler torque-free; true=Lagrange top  */
  float mgl;          /* mg·l (gravity torque scale, N·m)            */
} GPreset;

/*
 * Preset 0 — Euler's Top (symmetric, torque-free)
 *   I1=I2=2 (oblate "saucer"), I3=1.
 *   ωz=8 rad/s (fast spin); small ωx seeds the initial wobble.
 *   L is conserved; body-Z precesses at Euler freq ≈ (I1-I3)/I1·ωz = 4 rad/s.
 *
 * Preset 1 — Gravity Top (symmetric, with gravity)
 *   Same inertia.  ωz=12 rad/s; mgl=1.5.
 *   Precession rate ≈ mgl/(I3·ωz) ≈ 0.125 rad/s (slow circle).
 *   Nutation freq   ≈ I3·ωz/I1 = 6 rad/s (fast wobble).
 *
 * Preset 2 — Dzhanibekov / Tennis-Racket
 *   I1<I2<I3; rotation near INTERMEDIATE axis (I2) is unstable.
 *   Flip period T ≈ 2π / (ωy·√((I2-I1)(I3-I2)/(I1·I3))) ≈ 1.1 s.
 *   ~1 dramatic 180° flip per second.
 *
 * Preset 3 — Oblate Top (symmetric, torque-free, OPPOSITE precession)
 *   I1=I2=2, I3=3 (inverted ratio from preset 0).  Euler precession
 *   rate ≈ (I3-I1)/I1·ωz = 2.5 rad/s — SAME magnitude order as
 *   preset 0 but OPPOSITE sign: body cone now precesses OUTSIDE
 *   the space cone.  Direct visual comparison to preset 0.
 *
 * Preset 4 — Sleeping Top (gravity, very fast spin)
 *   Same inertia as 1; ωz=25 rad/s, tilt=3°, mgl=1.5.
 *   Precession ≈ mgl/(I3·ωz) = 0.06 rad/s — extremely slow.
 *   The classic "sleeping top": stays nearly upright, the spin
 *   axis barely traces a circle.  Demonstrates the stability
 *   threshold ω² > 4·mgl·I1/I3² for the upright position.
 *
 * Preset 5 — Slow Heavy Top (gravity, slow spin)
 *   Same inertia; ωz=4 rad/s, tilt=40°, mgl=2.0.
 *   Precession ≈ 0.5 rad/s — wide visible circles, with clearly
 *   visible nutation cusps from the body-Z tip dipping down and
 *   recovering each precession period.  Opposite limit of 4.
 *
 * Preset 6 — Stable Major (asymmetric, spin on largest-I axis)
 *   Same I as preset 2 (1, 2.5, 3.5); ω initially along Z (largest I).
 *   Intermediate-axis theorem: rotation about the largest principal
 *   axis is STABLE — tiny perturbations stay tiny.  Pair with
 *   preset 2 to see why only the MIDDLE eigenvalue is unstable.
 *
 * Preset 7 — Stable Minor (asymmetric, spin on smallest-I axis)
 *   Same I; ω initially along X (smallest I).
 *   Rotation about the smallest principal axis is ALSO stable.
 *   Both endpoints of the inertia spectrum are stable — only the
 *   middle one isn't.  Together with presets 2 and 6, this is the
 *   complete intermediate-axis-theorem demo.
 */
static const GPreset PRESETS[N_PRESETS] = {
    {/* Euler's Top */
     "Euler\\'s Top   (torque-free, symmetric)",
     2.0f,
     2.0f,
     1.0f,
     {0.8f, 0.0f, 8.0f},
     18.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Gravity Top */
     "Gravity Top    (precession + nutation)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 12.0f},
     25.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     1.5f},
    {/* Dzhanibekov effect */
     "Dzhanibekov    (intermediate-axis flip)",
     1.0f,
     2.5f,
     3.5f,
     {0.05f, 8.0f, 0.05f},
     10.0f,
     {0.0f, 0.0f, 1.0f},
     false,
     0.0f},
    {/* Oblate Top — opposite-sign precession from preset 0 */
     "Oblate Top     (opposite precession)",
     2.0f,
     2.0f,
     3.0f,
     {0.5f, 0.0f, 5.0f},
     18.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Sleeping Top — fast spin, almost upright */
     "Sleeping Top   (fast spin under gravity)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 25.0f},
     3.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     1.5f},
    {/* Slow Heavy Top — wide precession, cusped nutation */
     "Slow Heavy Top (wide precession circles)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 4.0f},
     40.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     2.0f},
    {/* Stable Major — asymmetric, spin on max-I axis */
     "Stable Major   (asym, spin on max-I axis)",
     1.0f,
     2.5f,
     3.5f,
     {0.05f, 0.05f, 6.0f},
     15.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Stable Minor — asymmetric, spin on min-I axis */
     "Stable Minor   (asym, spin on min-I axis)",
     1.0f,
     2.5f,
     3.5f,
     {6.0f, 0.05f, 0.05f},
     15.0f,
     {0.0f, 0.0f, 1.0f},
     false,
     0.0f},
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
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  themes + color                                                     */
/* ===================================================================== */

/*
 * Color-pair indices — fixed across all themes; only the underlying RGB
 * changes when theme cycles via t / T.  The drawing code references
 * these enum names; theme cycling re-runs init_pair() to remap them.
 */
enum {
  CP_AXIS_X = 1, /* body X axis                                     */
  CP_AXIS_Y = 2, /* body Y axis                                     */
  CP_AXIS_Z = 3, /* body Z axis (spin axis — most prominent)        */
  CP_MOM = 4,    /* angular momentum L                              */
  CP_GROUND = 5, /* ground ring + world-Z reference (rendered A_DIM)*/
  CP_TRAIL = 6,  /* polhode trail                                   */
  CP_DISC = 7,   /* body-disc equator                               */
  PAIR_HUD = 8,  /* HUD parameter line (yellow, canonical)          */
  PAIR_HINT = 9, /* key-legend line (cyan, canonical)               */
};

/*
 * Theme palette — MULTICOLOR, all-bright.
 *
 * Each theme is an 8-step BIPOLAR ramp (cool family → bright neutral
 * midpoint → warm family).  Structure inherited from
 * charged_particles.c where the ramp encodes signed charge, but every
 * slot has been pulled up into the VIVID half of the cube — the
 * "cool extreme" is now bright-cool (not dark-cool) so the dim slots
 * stay readable on every terminal background.  Bipolar character is
 * carried by HUE, not brightness.
 *
 * Ramp slot semantics:
 *   slot 0..3  cool half  (bright saturated → light)
 *   slot 3..4  bright neutral midpoint
 *   slot 4..7  warm half  (light → bright saturated)
 *
 * Role → ramp-slot mapping (same across every theme):
 *
 *   ax_x   ← ramp[1]   cool side, mid-saturation
 *   ax_y   ← ramp[6]   warm side, mid-saturation
 *                      → X and Y get DIFFERENT hues
 *   ax_z   ← ramp[7]   warm extreme — spin axis (the focal element)
 *   mom    ← ramp[0]   cool extreme — L vector, set OPPOSITE the
 *                                     spin axis so the conserved
 *                                     angular-momentum direction
 *                                     reads against ax_z
 *   ground ← ramp[3]   bright cool-neutral (gets A_DIM at draw time
 *                                            but still visible)
 *   disc   ← ramp[4]   bright warm-neutral (gets A_DIM at draw time)
 *   trail  ← trail     dedicated polhode-trail color
 *
 * The bipolar palette is canonical for signed/diverging data because
 * a single-hue ramp loses orientation under eye fatigue (Ware [5] in
 * charged_particles.c).  In the gyroscope it gives the body axes
 * three intuitively distinct directions: cool-mid / warm-mid / hot.
 *
 * Brightness floor: every cube index is ≥ 75 and every gray-ramp
 * index ≥ 247 — well clear of CLAUDE.md's "NEVER" zone (16-23 /
 * 232-239) AND the "lowest tier only" zone (24-29 / 240-243).
 */
typedef struct {
  const char *name;
  short ramp[8]; /* 8-step bipolar gradient (cool → bright → warm) */
  short trail;   /* dedicated polhode trail color                  */
  short sky;     /* unused in gyroscope; kept for table parity     */
} Theme;

#define N_THEMES 12

static const Theme THEMES[N_THEMES] = {
    /*  name        ramp[0..7]  (cool extreme → neutral → warm extreme) trail
       sky */
    {"VOLT",
     {81, 117, 123, 159, 230, 220, 214, 203},
     250,
     246}, /* electric blue → yellow → red (tri)     */
    {"COPPER",
     {110, 117, 153, 195, 230, 222, 215, 209},
     250,
     246}, /* steel blue → cream → copper red (di)   */
    {"NEON",
     {75, 117, 159, 195, 230, 218, 213, 205},
     250,
     246}, /* blue → cyan → pink (tri)               */
    {"ICE_FIRE",
     {81, 117, 159, 195, 230, 215, 209, 196},
     250,
     246}, /* ice blue → flame red (di, classic)     */
    {"AURORA",
     {84, 120, 156, 195, 230, 218, 213, 205},
     250,
     246}, /* green → pink (di, aurora-like)         */
    {"VIOLET",
     {99, 141, 177, 219, 230, 224, 217, 203},
     250,
     246}, /* violet → red (di)                      */
    {"CYBER",
     {82, 120, 158, 195, 230, 220, 215, 197},
     250,
     246}, /* green → cyan → red (tri)               */
    {"PASTEL",
     {117, 153, 195, 219, 230, 224, 218, 211},
     250,
     246}, /* soft blue → soft pink (di, all light)  */
    {"TWILIGHT",
     {99, 141, 177, 219, 230, 217, 215, 203},
     250,
     246}, /* indigo → coral (di)                    */
    {"SODIUM",
     {215, 222, 215, 230, 195, 159, 123, 117},
     250,
     246}, /* sodium amber → mercury cyan (di, rev)  */
    {"ECLIPSE",
     {247, 250, 252, 253, 203, 209, 215, 220},
     250,
     246}, /* light gray → corona gold (di)          */
    {"MONO",
     {247, 248, 250, 251, 252, 253, 254, 255},
     250,
     246}, /* bright grayscale reference             */
};

/*
 * color_apply_theme — re-init the colour pairs from THEMES[idx].
 *
 * Pair indices are fixed (CP_*); only the foreground RGB changes.
 * Called once at startup and again each time the user cycles theme,
 * so already-drawn cells repaint with the new palette on the very
 * next frame without touching any drawing code.
 *
 * 8-color fallback uses standard ANSI colours; cycling has no effect
 * on terminals that can't reach the 256-cube.
 */
static void color_apply_theme(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  const Theme *th = &THEMES[idx];
  if (COLORS >= 256) {
    /* Bipolar ramp → role mapping (see Theme docstring above) */
    init_pair(CP_AXIS_X, th->ramp[1], -1); /* cool side mid          */
    init_pair(CP_AXIS_Y, th->ramp[6], -1); /* warm side mid          */
    init_pair(CP_AXIS_Z, th->ramp[7], -1); /* warm extreme — spin    */
    init_pair(CP_MOM, th->ramp[0], -1);    /* cool extreme — L       */
    init_pair(CP_GROUND, th->ramp[3], -1); /* cool neutral (A_DIM)   */
    init_pair(CP_TRAIL, th->trail, -1);    /* dedicated trail colour */
    init_pair(CP_DISC, th->ramp[4], -1);   /* warm neutral (A_DIM)   */
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    /* 8-color fallback: keep axes distinguishable on legacy terminals */
    init_pair(CP_AXIS_X, COLOR_BLUE, -1);   /* cool side             */
    init_pair(CP_AXIS_Y, COLOR_YELLOW, -1); /* warm side             */
    init_pair(CP_AXIS_Z, COLOR_RED, -1);    /* warm extreme — spin   */
    init_pair(CP_MOM, COLOR_CYAN, -1);      /* cool extreme — L      */
    init_pair(CP_GROUND, COLOR_WHITE, -1);
    init_pair(CP_TRAIL, COLOR_MAGENTA, -1);
    init_pair(CP_DISC, COLOR_GREEN, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  color_apply_theme(theme);
}

/* ===================================================================== */
/* §4  coords — aspect correction + 3-D orthographic projection          */
/* ===================================================================== */

/*
 * ASPECT CORRECTION
 * Terminal cells are ~2× taller than wide (CELL_H/CELL_W ≈ 2).
 * When converting a 3-D screen-Y coordinate to a terminal row, we
 * multiply by ASPECT = CELL_W/CELL_H ≈ 0.5 so that unit-length axes
 * appear equal length in all directions.  Only applied at draw time.
 *
 * PROJECTION
 * Two sequential rotations applied to world coordinates:
 *   1. Azimuth φ: rotate around world Z (panning left/right).
 *   2. Elevation θ: tilt the scene so the viewer is above the horizon.
 * Depth (sz) is used for depth-cueing (A_BOLD for near, A_DIM for far).
 */
#define CELL_W 8
#define CELL_H 16
#define ASPECT ((float)CELL_W / (float)CELL_H) /* ≈ 0.5 */

static void project(float wx, float wy, float wz, float phi, float theta,
                    float scale, int cx, int cy, int *col, int *row,
                    float *depth) {
  float rx = wx * cosf(phi) + wy * sinf(phi);
  float ry = -wx * sinf(phi) + wy * cosf(phi);

  float sx = rx;
  float sy = ry * cosf(theta) + wz * sinf(theta);
  *depth = -ry * sinf(theta) + wz * cosf(theta);

  *col = cx + (int)roundf(sx * scale);
  *row = cy - (int)roundf(sy * scale * ASPECT);
}

/* Direction-aware character for a 2-D segment angle */
static char dir_char(float angle) {
  float a = fmodf(angle, (float)M_PI);
  if (a < 0.0f)
    a += (float)M_PI;
  if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f)
    return '-';
  if (a < 3.0f * (float)M_PI / 8.0f)
    return '/';
  if (a < 5.0f * (float)M_PI / 8.0f)
    return '|';
  return '\\';
}

/* Draw a 3-D line segment (from origin to endpoint) using DDA */
static void draw_seg3d(WINDOW *w, float ox3, float oy3, float oz3, float ex3,
                       float ey3, float ez3, float phi, float theta,
                       float scale, int cx, int cy, int cols, int rows,
                       chtype attr) {
  int c0, r0;
  float d0;
  int c1, r1;
  float d1;
  project(ox3, oy3, oz3, phi, theta, scale, cx, cy, &c0, &r0, &d0);
  project(ex3, ey3, ez3, phi, theta, scale, cx, cy, &c1, &r1, &d1);

  float ang = atan2f((float)(r1 - r0), (float)(c1 - c0));
  char ch = dir_char(ang);

  int dx = c1 - c0, dy = r1 - r0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if (steps < 1)
    steps = 1;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    int c = c0 + (int)roundf(dx * t);
    int r = r0 + (int)roundf(dy * t);
    if (c >= 0 && c < cols && r >= 1 && r < rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, (chtype)(unsigned char)ch);
      wattroff(w, attr);
    }
  }
}

/* ===================================================================== */
/* §5  entity — Gyro                                                      */
/* ===================================================================== */

/* ── RK4 state vector [ωx, ωy, ωz, qw, qx, qy, qz] ─────────────────── */

/*
 * State7 — the 7-component vector RK4 [6] integrates each tick.
 *
 * Layout (positional — order matters across gyro_deriv / gyro_step):
 *   v[0..2]  =  ωx, ωy, ωz        angular velocity (body frame, rad/s)
 *   v[3..6]  =  qw, qx, qy, qz    orientation quaternion (unit-length)
 *
 * Why a flat array (not separate ω-vector and quaternion structs):
 *   RK4 needs the SAME element-wise operations (add, scalar multiply)
 *   on every component to compute k₁..k₄ and the weighted sum.  A
 *   split layout would force gyro_deriv to return two values and the
 *   RK4 step to add them piecewise.  Flat array → s7_add / s7_scale
 *   are trivial 7-iteration loops; cost is that callers must know the
 *   positional layout, which stays contained in two functions.
 *
 * Why ω and q in ONE state (not integrated separately):
 *   They're COUPLED — Euler's equations [1] give ω̇ in terms of ω
 *   (and torque), and quaternion kinematics [4] give q̇ in terms of
 *   BOTH q and ω: q̇ = ½ · q ⊗ (0, ω).  RK4 needs a coherent
 *   intermediate state to evaluate k₂, k₃, k₄; integrating them in
 *   two separate passes would break the implicit coupling.
 */
typedef struct {
  float v[7];
} State7;

static State7 s7_add(State7 a, State7 b) {
  State7 r;
  for (int i = 0; i < 7; i++)
    r.v[i] = a.v[i] + b.v[i];
  return r;
}
static State7 s7_scale(State7 a, float s) {
  State7 r;
  for (int i = 0; i < 7; i++)
    r.v[i] = a.v[i] * s;
  return r;
}

/* ── physics building blocks (consumed by gyro_deriv below) ───────── */

/*
 * gravity_torque_body_frame — Lagrange-top gravity torque [1].
 *
 * Centre of mass sits at l·ez (body-Z) in world frame.  In the body
 * frame, world-Z is the third row of R, which from the quaternion
 * formula [4] is (gz_bx, gz_by, gz_bz) = (2(qx·qz − qw·qy),
 * 2(qy·qz + qw·qx), 1 − 2(qx² + qy²)).
 *
 * Torque = (0, 0, l) × m·g·(−Ẑ_body)  = mgl · (gz_by, −gz_bx, 0)
 * The z-component is identically zero (gravity can't twist about
 * its own axis), so only tau_x and tau_y are returned.
 */
static void gravity_torque_body_frame(float qw, float qx, float qy, float qz,
                                      float mgl, float *tau_x, float *tau_y) {
  float gz_bx = 2.0f * (qx * qz - qw * qy);
  float gz_by = 2.0f * (qy * qz + qw * qx);
  *tau_x = mgl * gz_by;
  *tau_y = -mgl * gz_bx;
}

/*
 * euler_equations_omega_dot — Euler's equations of motion [1, 2].
 *
 *   I₁·ω̇x = (I₂ − I₃)·ωy·ωz + τx
 *   I₂·ω̇y = (I₃ − I₁)·ωz·ωx + τy
 *   I₃·ω̇z = (I₁ − I₂)·ωx·ωy + τz       (τz ≡ 0 in this demo)
 *
 * The cross-product terms (I₂ − I₃)·ωy·ωz etc. are what couple the
 * three components — without them ω̇ would be a linear ODE; with
 * them you get Euler precession, nutation, AND the intermediate-axis
 * instability [3] for free.
 */
static void euler_equations_omega_dot(float I1, float I2, float I3, float wx,
                                      float wy, float wz, float tau_x,
                                      float tau_y, float *dwx, float *dwy,
                                      float *dwz) {
  *dwx = ((I2 - I3) * wy * wz + tau_x) / I1;
  *dwy = ((I3 - I1) * wz * wx + tau_y) / I2;
  *dwz = ((I1 - I2) * wx * wy) / I3;
}

/*
 * quaternion_kinematics — q̇ from quaternion and body-frame ω [4].
 *
 *   q̇ = ½ · q ⊗ (0, ω)
 *
 * Expanded component-by-component (Hamilton product):
 *   q̇w = ½ · (−qx·ωx − qy·ωy − qz·ωz)
 *   q̇x = ½ · ( qw·ωx + qy·ωz − qz·ωy)
 *   q̇y = ½ · ( qw·ωy − qx·ωz + qz·ωx)
 *   q̇z = ½ · ( qw·ωz + qx·ωy − qy·ωx)
 *
 * Lives on the tangent space of S³ ⊂ ℝ⁴; integrating it with RK4
 * drifts off S³ by O(dt⁵) per step — fixed by project_quaternion
 * after each integration step.
 */
static void quaternion_kinematics(float qw, float qx, float qy, float qz,
                                  float wx, float wy, float wz, float *dqw,
                                  float *dqx, float *dqy, float *dqz) {
  *dqw = 0.5f * (-qx * wx - qy * wy - qz * wz);
  *dqx = 0.5f * (qw * wx + qy * wz - qz * wy);
  *dqy = 0.5f * (qw * wy - qx * wz + qz * wx);
  *dqz = 0.5f * (qw * wz + qx * wy - qy * wx);
}

/*
 * gyro_deriv — time derivative of the 7-component state.
 *
 * Input:  s = [ωx, ωy, ωz, qw, qx, qy, qz]
 * Output: ṡ = [ω̇x, ω̇y, ω̇z, q̇w, q̇x, q̇y, q̇z]
 *
 * Pseudocode:
 *   1. compute external torque (gravity, or zero)
 *   2. Euler's equations → ω̇
 *   3. quaternion kinematics → q̇
 *   4. pack ω̇ and q̇ into State7
 */
static State7 gyro_deriv(State7 s, float I1, float I2, float I3, float mgl,
                         bool gravity) {
  float wx = s.v[0], wy = s.v[1], wz = s.v[2];
  float qw = s.v[3], qx = s.v[4], qy = s.v[5], qz = s.v[6];

  /* 1. external torque (Lagrange-top gravity, or zero) */
  float tau_x = 0.0f, tau_y = 0.0f;
  if (gravity)
    gravity_torque_body_frame(qw, qx, qy, qz, mgl, &tau_x, &tau_y);

  /* 2. Euler's equations → angular acceleration */
  float dwx, dwy, dwz;
  euler_equations_omega_dot(I1, I2, I3, wx, wy, wz, tau_x, tau_y, &dwx, &dwy,
                            &dwz);

  /* 3. quaternion kinematics → quaternion rate */
  float dqw, dqx, dqy, dqz;
  quaternion_kinematics(qw, qx, qy, qz, wx, wy, wz, &dqw, &dqx, &dqy, &dqz);

  /* 4. pack into State7 (same positional layout as input s) */
  State7 d;
  d.v[0] = dwx;
  d.v[1] = dwy;
  d.v[2] = dwz;
  d.v[3] = dqw;
  d.v[4] = dqx;
  d.v[5] = dqy;
  d.v[6] = dqz;
  return d;
}

/* ── quaternion → rotation axes ─────────────────────────────────────── */

/*
 * quat_to_axes() — extract body frame axes in world coordinates.
 *
 * The rotation matrix R (body→world) has these columns:
 *   ex = first  column = body X in world
 *   ey = second column = body Y in world
 *   ez = third  column = body Z in world (the spin axis)
 *
 * Standard formula from unit quaternion q = (qw, qx, qy, qz):
 */
static void quat_to_axes(const float q[4], float ex[3], float ey[3],
                         float ez[3]) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  ex[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
  ex[1] = 2.0f * (qx * qy + qw * qz);
  ex[2] = 2.0f * (qx * qz - qw * qy);

  ey[0] = 2.0f * (qx * qy - qw * qz);
  ey[1] = 1.0f - 2.0f * (qx * qx + qz * qz);
  ey[2] = 2.0f * (qy * qz + qw * qx);

  ez[0] = 2.0f * (qx * qz + qw * qy);
  ez[1] = 2.0f * (qy * qz - qw * qx);
  ez[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

/*
 * gram_schmidt — re-orthonormalise three axis vectors in place [5].
 *
 * Prevents floating-point round-off in quat_to_axes from accumulating
 * into non-orthogonal frame axes.  Always invoked via refresh_body_axes
 * (preset load + every RK4 step), never called directly elsewhere.
 *
 *   e1 ← normalize(e1)
 *   e2 ← normalize(e2 − (e2·e1)·e1)
 *   e3 ← e1 × e2                       (always exactly orthogonal)
 */
static void gram_schmidt(float e1[3], float e2[3], float e3[3]) {
  /* Normalise e1 */
  float n = sqrtf(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
  if (n > 1e-9f) {
    e1[0] /= n;
    e1[1] /= n;
    e1[2] /= n;
  }

  /* Remove e1 component from e2, normalise */
  float d = e2[0] * e1[0] + e2[1] * e1[1] + e2[2] * e1[2];
  e2[0] -= d * e1[0];
  e2[1] -= d * e1[1];
  e2[2] -= d * e1[2];
  n = sqrtf(e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2]);
  if (n > 1e-9f) {
    e2[0] /= n;
    e2[1] /= n;
    e2[2] /= n;
  }

  /* e3 = e1 × e2 (guaranteed orthonormal) */
  e3[0] = e1[1] * e2[2] - e1[2] * e2[1];
  e3[1] = e1[2] * e2[0] - e1[0] * e2[2];
  e3[2] = e1[0] * e2[1] - e1[1] * e2[0];
}

/* ── trail ring-buffer ───────────────────────────────────────────────── */

/*
 * TrailPt — one captured screen position of the body-Z (spin axis)
 * tip, stored in the ring-buffer Gyro.trail[].
 *
 * Stored in SCREEN space (cell row / col), not world space.  The
 * trail is purely visual — it's the locus the spin axis traces out
 * on the current viewport as the body precesses / wobbles.  Storing
 * world-space points would force re-projection every frame even
 * though the view rotation is slow; screen-space captures the
 * polhode on the active viewport directly and lets gyro_draw plot
 * it in one pass with no math.
 *
 * Reset when: (a) the view resizes (SIGWINCH — old screen coords
 * stale), (b) the user toggles trail off, (c) preset switch / reset
 * (the polhode belongs to the previous body state).
 */
typedef struct {
  int col, row;
} TrailPt;

/* ─────────────────────────────────────────────────────────────────────── *
 * Gyro — the complete state of one rigid-body simulation.
 *
 * Bundles the PHYSICS state (omega + quat) with DERIVED rendering
 * helpers (ex / ey / ez / L), BODY PARAMETERS (inertia + gravity), the
 * trail RING BUFFER, the VIEW camera, and the per-instance CONTROL
 * flags.  One Gyro lives inside Scene (§6); App (§8) holds the Scene.
 *
 * Why a single owner (physics + derived + UI all in one struct):
 *   The derived axes ex / ey / ez and the world-frame angular momentum
 *   L are pointer-free reads from quat / omega after each step — they
 *   live next to their source so quat_to_axes + gram_schmidt + the
 *   L-recompute can run in one tail pass at the end of gyro_step.
 *   The trail and view live here because they're meaningless without
 *   the body that produced them; r-reset memsets the whole struct in
 *   one call.
 *
 * Sleeping-top stability threshold (symmetric top under gravity [1]):
 *   ω_z² > 4·mgl·I₁ / I₃²  ⇒ upright position is stable.
 *   Preset 4 (Sleeping Top) sits well above the threshold; preset 5
 *   (Slow Heavy Top) sits below it on purpose to expose the wide
 *   precession + nutation cusps.
 *
 * Field groups (mirrored by the layout below):
 *
 *   Physics state    — omega, quat.  The 7 floats RK4 [6] integrates,
 *                       copied into State7 inside gyro_step and back
 *                       out after the step.  See Euler's equations
 *                       [1, 2] and quaternion kinematics [4].
 *
 *   Derived (cached) — ex, ey, ez, L.  Recomputed in gyro_step from
 *                       quat (axes via [4]) and omega
 *                       (L = Σ Ii·ωi·ei).  Cached because draw + HUD
 *                       read them every frame; recomputing on every
 *                       read would multiply atan / sqrt calls inside
 *                       draw_seg3d.
 *
 *   Body parameters  — I1, I2, I3, gravity, mgl.  Loaded from the
 *                       active GPreset.  Persistent across r-reset
 *                       unless the user picks a new preset (n / p)
 *                       or toggles gravity (g).
 *
 *   Polhode trail    — trail[], trail_head, trail_fill, show_trail.
 *                       Classic circular log: trail_head walks around
 *                       overwriting oldest entries once full;
 *                       trail_fill saturates at TRAIL_LEN.  No
 *                       allocation, no shifts.
 *
 *   View camera      — view_phi (azimuth), view_theta (elevation).
 *                       Auto-rotated at 0.15 rad/s in scene_tick +
 *                       nudged by arrow keys.  Owned by Gyro (not
 *                       Scene) so r-reset preserves the user's view.
 *
 *   Control flags    — preset (active index into PRESETS[]), paused
 *                       (space toggle).  Live here so the HUD and
 *                       app_handle_key reach them through one Gyro
 *                       pointer.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Physics state (integrated by gyro_step) ──────────────────── */
  float omega[3]; /* angular velocity in body frame (rad/s)   */
  float quat[4];  /* orientation quaternion [qw,qx,qy,qz];
                   * |q|=1 maintained by post-step projection */

  /* ── Derived (recomputed in gyro_step from quat / omega) ──────── */
  float ex[3]; /* body X axis in world frame               */
  float ey[3]; /* body Y axis in world frame               */
  float ez[3]; /* body Z axis (spin axis) in world frame   */
  float L[3];  /* angular momentum L = Σ Ii·ωi·ei (world)  */

  /* ── Body parameters (loaded from active GPreset) ─────────────── */
  float I1, I2, I3; /* principal moments of inertia (kg·m²)     */

  bool gravity; /* false → Euler torque-free
                 * true  → Lagrange top, gravity torque on  */
  float mgl;    /* mass·g·l, gravity-torque scale (N·m)     */

  /* ── Polhode trail (ring buffer in SCREEN space) ──────────────── */
  TrailPt trail[TRAIL_LEN]; /* body-Z tip positions, oldest first       */
  int trail_head;           /* next-write index (mod TRAIL_LEN)         */
  int trail_fill;           /* live entries; saturates at TRAIL_LEN     */
  bool show_trail;          /* l / L toggles                            */

  /* ── View camera (preserved across r-reset) ───────────────────── */
  float view_phi;   /* azimuth, radians; auto-rotates slowly    */
  float view_theta; /* elevation, radians; clamped to [0.1,1.4] */

  /* ── Control flags ────────────────────────────────────────────── */
  int preset;  /* active index into PRESETS[]; n / p cycle */
  bool paused; /* space toggles; scene_tick is a no-op
                * when true                                */
} Gyro;

/* ── initialisation building blocks ──────────────────────────────────── */

/*
 * axis_angle_to_quat — build a unit quaternion from an axis-angle
 * rotation [4]:
 *
 *   q = (cos(θ/2),  sin(θ/2) · â)        where â = axis / |axis|
 *
 * Used to construct the initial tilt of each preset (rotates the
 * body-Z away from world-Z by tilt_deg so gravity has a lever arm).
 */
static void axis_angle_to_quat(float angle_rad, const float axis[3],
                               float q[4]) {
  float half = angle_rad * 0.5f;
  float sh = sinf(half);
  float ax = axis[0], ay = axis[1], az = axis[2];
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-9f)
    n = 1.0f;
  q[0] = cosf(half);
  q[1] = sh * ax / n;
  q[2] = sh * ay / n;
  q[3] = sh * az / n;
}

/*
 * refresh_body_axes — recompute (ex, ey, ez) from the current
 * quaternion, then Gram-Schmidt re-orthonormalise [5] to drag the
 * frame back onto SO(3) after floating-point round-off.
 *
 * Called every time quat changes (preset load, RK4 step) so the
 * cached body axes that draw / HUD read are always consistent with
 * the live orientation.
 */
static void refresh_body_axes(Gyro *g) {
  quat_to_axes(g->quat, g->ex, g->ey, g->ez);
  gram_schmidt(g->ex, g->ey, g->ez);
}

/*
 * clear_polhode_trail — drop all captured trail points.  Called on
 * preset switch, r-reset, l-trail-toggle-off, and resize — the
 * stored screen-space points are stale in every one of those cases.
 */
static void clear_polhode_trail(Gyro *g) {
  g->trail_head = 0;
  g->trail_fill = 0;
}

/*
 * load_body_parameters — copy inertia tensor + initial angular
 * velocity + gravity flag + mgl from the preset record into the live
 * Gyro.  Does NOT touch quat or the trail (those are reset by their
 * own helpers in gyro_set_preset).
 */
static void load_body_parameters(Gyro *g, const GPreset *pr) {
  g->gravity = pr->gravity;
  g->mgl = pr->mgl;
  g->I1 = pr->I1;
  g->I2 = pr->I2;
  g->I3 = pr->I3;
  g->omega[0] = pr->omega[0];
  g->omega[1] = pr->omega[1];
  g->omega[2] = pr->omega[2];
}

/*
 * gyro_set_preset — load initial conditions from a preset record.
 *
 * Pseudocode:
 *   1. record active preset index
 *   2. load body parameters (I, ω, gravity, mgl) from preset
 *   3. quat ← axis_angle_to_quat(tilt_deg, tilt_axis)
 *   4. refresh body axes from the new quaternion
 *   5. clear polhode trail (belongs to the previous body state)
 */
static void gyro_set_preset(Gyro *g, int p) {
  const GPreset *pr = &PRESETS[p];
  g->preset = p;

  load_body_parameters(g, pr);

  float tilt_rad = pr->tilt_deg * (float)M_PI / 180.0f;
  axis_angle_to_quat(tilt_rad, pr->tilt_axis, g->quat);

  refresh_body_axes(g);
  clear_polhode_trail(g);
}

static void gyro_init(Gyro *g) {
  memset(g, 0, sizeof *g);
  g->view_phi = 0.4f;
  g->view_theta = 0.6f;
  g->show_trail = true;
  gyro_set_preset(g, 0);
}

/* ── RK4 building blocks (consumed by gyro_step below) ─────────────── */

/* Pack the live Gyro state into the flat RK4 vector. */
static State7 pack_gyro_into_state7(const Gyro *g) {
  State7 s;
  s.v[0] = g->omega[0];
  s.v[1] = g->omega[1];
  s.v[2] = g->omega[2];
  s.v[3] = g->quat[0];
  s.v[4] = g->quat[1];
  s.v[5] = g->quat[2];
  s.v[6] = g->quat[3];
  return s;
}

/* Inverse of pack_gyro_into_state7 — store the RK4 result back. */
static void unpack_state7_into_gyro(State7 s, Gyro *g) {
  g->omega[0] = s.v[0];
  g->omega[1] = s.v[1];
  g->omega[2] = s.v[2];
  g->quat[0] = s.v[3];
  g->quat[1] = s.v[4];
  g->quat[2] = s.v[5];
  g->quat[3] = s.v[6];
}

/*
 * rk4_classical_step — one classical 4th-order Runge-Kutta step [6].
 *
 *   k₁ = f(s)
 *   k₂ = f(s + ½·dt·k₁)
 *   k₃ = f(s + ½·dt·k₂)
 *   k₄ = f(s + dt·k₃)
 *   s_{n+1} = s + (dt/6) · (k₁ + 2·k₂ + 2·k₃ + k₄)
 *
 * Local truncation error O(dt⁵), global O(dt⁴).  With SUB_STEPS=8
 * sub-steps per render frame at 60 Hz, the effective dt ≈ 2 ms — far
 * below the fastest oscillation period in any preset (Dzhanibekov
 * flips at ~1 Hz, nutation at ~6 Hz, ω_z up to 25 rad/s).
 */
static State7 rk4_classical_step(State7 s, float dt, float I1, float I2,
                                 float I3, float mgl, bool gravity) {
  State7 k1 = gyro_deriv(s, I1, I2, I3, mgl, gravity);
  State7 k2 =
      gyro_deriv(s7_add(s, s7_scale(k1, dt * 0.5f)), I1, I2, I3, mgl, gravity);
  State7 k3 =
      gyro_deriv(s7_add(s, s7_scale(k2, dt * 0.5f)), I1, I2, I3, mgl, gravity);
  State7 k4 = gyro_deriv(s7_add(s, s7_scale(k3, dt)), I1, I2, I3, mgl, gravity);

  State7 sum =
      s7_add(s7_add(k1, s7_scale(k2, 2.0f)), s7_add(s7_scale(k3, 2.0f), k4));
  return s7_add(s, s7_scale(sum, dt / 6.0f));
}

/*
 * project_quaternion_to_unit — divide quaternion by its length,
 * snapping it back onto the unit sphere S³ ⊂ ℝ⁴.  Each RK4 step
 * drifts off S³ by O(dt⁵); this projection is the cheapest case of
 * the geometric-integration projection method [5] and is enough to
 * stay on the manifold for our integration horizons.
 */
static void project_quaternion_to_unit(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f) {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  }
}

/*
 * recompute_angular_momentum_world — L = Σ Ii·ωi·ei.
 *
 * Each principal axis ei carries inertia Ii and rotates at ωi; the
 * angular momentum is the vector sum of these contributions in world
 * coordinates.  L̇ = 0 when there's no external torque (Euler
 * presets) — a free visual sanity check: in preset 0 the L vector
 * stays put while the body precesses around it.
 */
static void recompute_angular_momentum_world(Gyro *g) {
  for (int k = 0; k < 3; k++) {
    g->L[k] = g->I1 * g->omega[0] * g->ex[k] + g->I2 * g->omega[1] * g->ey[k] +
              g->I3 * g->omega[2] * g->ez[k];
  }
}

/*
 * gyro_step — advance the rigid-body simulation by dt.
 *
 * Pseudocode:
 *   1. pack Gyro state into State7
 *   2. RK4 classical step (k1..k4 weighted sum [6])
 *   3. unpack State7 back into Gyro
 *   4. project quaternion to unit length (S³ drift removal [5])
 *   5. refresh body axes from the new quaternion (+ Gram-Schmidt)
 *   6. recompute world-frame angular momentum L
 */
static void gyro_step(Gyro *g, float dt) {
  State7 s = pack_gyro_into_state7(g);
  State7 ns =
      rk4_classical_step(s, dt, g->I1, g->I2, g->I3, g->mgl, g->gravity);
  unpack_state7_into_gyro(ns, g);

  project_quaternion_to_unit(g->quat);
  refresh_body_axes(g);
  recompute_angular_momentum_world(g);
}

/* ── drawing — viewport + per-layer helpers consumed by gyro_draw ──── */

/*
 * Viewport — every projection parameter the draw layers need in one
 * bundle.  Computed once at the top of gyro_draw and passed by const
 * pointer to each layer helper so they don't drag 7+ args around.
 */
typedef struct {
  float phi, theta; /* camera azimuth + elevation (radians)        */
  float scale;      /* pixels per world unit (see compute_*)       */
  int cx, cy;       /* terminal centre cell                        */
  int cols, rows;   /* viewport extent (for clipping)              */
} Viewport;

/*
 * Pixels-per-world-unit so that a unit-length axis fits in roughly
 * 1/7 of the smaller screen dimension.  rows is multiplied by 2
 * because terminal cells are ~2× taller than wide (ASPECT correction
 * is applied later inside project()).
 */
static float compute_render_scale(int cols, int rows) {
  float sc_h = (float)cols / 7.0f;
  float sc_v = (float)(rows - 2) * 2.0f / 7.0f;
  float s = sc_h < sc_v ? sc_h : sc_v;
  return s < 1.0f ? 1.0f : s;
}

static Viewport make_viewport(const Gyro *g, int cols, int rows) {
  Viewport v;
  v.phi = g->view_phi;
  v.theta = g->view_theta;
  v.cx = cols / 2;
  v.cy = (rows + 1) / 2;
  v.cols = cols;
  v.rows = rows;
  v.scale = compute_render_scale(cols, rows);
  return v;
}

/*
 * draw_ground_ring — dotted ellipse in the world XY plane.
 * Gives the viewer a fixed horizon to read body tilt against;
 * without it the gyroscope would float in a featureless void.
 */
static void draw_ground_ring(WINDOW *w, const Viewport *v) {
  const float gr = 1.2f; /* ring radius (world units) */
  wattron(w, COLOR_PAIR(CP_GROUND) | A_DIM);
  for (int i = 0; i < GROUND_PTS; i++) {
    float t = 2.0f * (float)M_PI * i / GROUND_PTS;
    float wx = gr * cosf(t), wy = gr * sinf(t);
    int c, r;
    float d;
    project(wx, wy, 0.0f, v->phi, v->theta, v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1)
      mvwaddch(w, r, c, '.');
  }
  wattroff(w, COLOR_PAIR(CP_GROUND) | A_DIM);
}

/*
 * draw_world_z_reference — vertical line from origin to (0,0,1.3).
 * Together with the ground ring it makes "upright" visually concrete,
 * so the tilt of body-Z away from world-Z reads as nutation angle.
 */
static void draw_world_z_reference(WINDOW *w, const Viewport *v) {
  draw_seg3d(w, 0, 0, 0, 0, 0, 1.3f, v->phi, v->theta, v->scale, v->cx, v->cy,
             v->cols, v->rows, (chtype)(COLOR_PAIR(CP_GROUND) | A_DIM));
}

/*
 * draw_body_disc_equator — ring around the body XY plane.
 *
 * DISC_PTS dots laid out on a circle of radius dr in the BODY frame,
 * each transformed to world coords via the body axes ex / ey.  Makes
 * the body's orientation visually concrete — you literally watch the
 * disc wobble — without committing to any particular rigid shape.
 */
static void draw_body_disc_equator(WINDOW *w, const Gyro *g,
                                   const Viewport *v) {
  const float dr = 0.45f;
  chtype attr = (chtype)(COLOR_PAIR(CP_DISC) | A_DIM);
  for (int i = 0; i < DISC_PTS; i++) {
    float t = 2.0f * (float)M_PI * i / DISC_PTS;
    float ct = cosf(t), st = sinf(t);
    float wx = dr * (ct * g->ex[0] + st * g->ey[0]);
    float wy = dr * (ct * g->ex[1] + st * g->ey[1]);
    float wz = dr * (ct * g->ex[2] + st * g->ey[2]);
    int c, r;
    float d;
    project(wx, wy, wz, v->phi, v->theta, v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, 'o');
      wattroff(w, attr);
    }
  }
}

/*
 * draw_polhode_trail — body-Z tip locus, fading with age.
 *
 * Newest 1/4 = A_BOLD, middle 1/4 = normal, oldest = A_DIM.  Reveals
 * the body-cone trace: clean circles for preset 1 (steady precession),
 * cusps for preset 5 (nutation), figure-eights or wild swings for the
 * intermediate-axis presets.  Trail points are stored in screen space
 * (see TrailPt docstring), so this loop is pure ncurses output.
 */
static void draw_polhode_trail(WINDOW *w, const Gyro *g, const Viewport *v) {
  if (!g->show_trail || g->trail_fill == 0)
    return;

  int n = g->trail_fill;
  for (int i = 0; i < n; i++) {
    int idx = (g->trail_head - n + i + TRAIL_LEN) % TRAIL_LEN;
    int c = g->trail[idx].col, r = g->trail[idx].row;
    if (c < 0 || c >= v->cols || r < 1 || r >= v->rows - 1)
      continue;

    chtype attr;
    if (i > n * 3 / 4)
      attr = (chtype)(COLOR_PAIR(CP_TRAIL) | A_BOLD);
    else if (i > n / 2)
      attr = (chtype)(COLOR_PAIR(CP_TRAIL));
    else
      attr = (chtype)(COLOR_PAIR(CP_TRAIL) | A_DIM);
    wattron(w, attr);
    mvwaddch(w, r, c, '.');
    wattroff(w, attr);
  }
}

/*
 * draw_body_axes_depth_sorted — body X / Y / Z axes with tip labels.
 *
 * Painter's algorithm: project each tip, sort by depth descending,
 * draw farthest first so nearer axes paint OVER farther ones.
 * Within each axis, depth < 0 (near) → A_BOLD; depth > 0 (far) →
 * normal.  Tip label sits 15 % beyond the unit-length tip so it
 * doesn't fight the line glyph for the last cell.
 */
static void draw_body_axes_depth_sorted(WINDOW *w, const Gyro *g,
                                        const Viewport *v) {
  struct {
    const float *axis;
    const char *label;
    int pair;
  } axes[3] = {
      {g->ex, "X", CP_AXIS_X},
      {g->ey, "Y", CP_AXIS_Y},
      {g->ez, "Z", CP_AXIS_Z},
  };

  /* Probe tip depth for each axis */
  float depths[3];
  for (int i = 0; i < 3; i++) {
    int c, r;
    float d;
    project(axes[i].axis[0], axes[i].axis[1], axes[i].axis[2], v->phi, v->theta,
            v->scale, v->cx, v->cy, &c, &r, &d);
    depths[i] = d;
  }

  /* Bubble-sort indices by depth descending (farthest first) */
  int order[3] = {0, 1, 2};
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (depths[order[i]] < depths[order[j]]) {
        int t = order[i];
        order[i] = order[j];
        order[j] = t;
      }

  for (int oi = 0; oi < 3; oi++) {
    int i = order[oi];
    const float *ax = axes[i].axis;
    chtype attr =
        (chtype)(COLOR_PAIR(axes[i].pair) | (depths[i] < 0.0f ? A_BOLD : 0u));

    draw_seg3d(w, 0, 0, 0, ax[0], ax[1], ax[2], v->phi, v->theta, v->scale,
               v->cx, v->cy, v->cols, v->rows, attr);

    /* Tip label, slightly beyond the unit-length tip */
    int c, r;
    float d;
    project(ax[0] * 1.15f, ax[1] * 1.15f, ax[2] * 1.15f, v->phi, v->theta,
            v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, (chtype)(unsigned char)axes[i].label[0]);
      wattroff(w, attr);
    }
  }
}

/*
 * draw_angular_momentum_vector — L vector, normalised to unit length.
 *
 * In torque-free presets L is conserved (vector fixed in space) —
 * the body axes precess AROUND it.  Under gravity, L itself traces
 * a circle around world-Z and the precession axis is world-Z.  Tip
 * label 'L' 15 % beyond the unit tip, matching the body-axis style.
 */
static void draw_angular_momentum_vector(WINDOW *w, const Gyro *g,
                                         const Viewport *v) {
  float Lmag = sqrtf(g->L[0] * g->L[0] + g->L[1] * g->L[1] + g->L[2] * g->L[2]);
  if (Lmag < 1e-9f)
    return;

  float Ldx = g->L[0] / Lmag, Ldy = g->L[1] / Lmag, Ldz = g->L[2] / Lmag;
  draw_seg3d(w, 0, 0, 0, Ldx, Ldy, Ldz, v->phi, v->theta, v->scale, v->cx,
             v->cy, v->cols, v->rows, (chtype)(COLOR_PAIR(CP_MOM) | A_BOLD));

  int c, r;
  float d;
  project(Ldx * 1.15f, Ldy * 1.15f, Ldz * 1.15f, v->phi, v->theta, v->scale,
          v->cx, v->cy, &c, &r, &d);
  if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
    wattron(w, COLOR_PAIR(CP_MOM) | A_BOLD);
    mvwaddch(w, r, c, 'L');
    wattroff(w, COLOR_PAIR(CP_MOM) | A_BOLD);
  }
}

/*
 * gyro_draw — render the complete scene, painter's order (far → near).
 *
 * Depth-cuing throughout: A_BOLD for elements closer to the viewer
 * (depth < 0), A_DIM or normal for elements receding (depth > 0).
 *
 * Pseudocode:
 *   viewport = make_viewport(g, cols, rows)
 *   draw_ground_ring                 // world floor reference
 *   draw_world_z_reference           // upright direction
 *   draw_body_disc_equator           // body XY plane (wobble visible)
 *   draw_polhode_trail               // spin-axis history
 *   draw_body_axes_depth_sorted      // X / Y / Z with tip labels
 *   draw_angular_momentum_vector     // top — L vector with tip label
 */
static void gyro_draw(const Gyro *g, WINDOW *w, int cols, int rows) {
  Viewport vp = make_viewport(g, cols, rows);

  draw_ground_ring(w, &vp);
  draw_world_z_reference(w, &vp);
  draw_body_disc_equator(w, g, &vp);
  draw_polhode_trail(w, g, &vp);
  draw_body_axes_depth_sorted(w, g, &vp);
  draw_angular_momentum_vector(w, g, &vp);
}

/* ── trail update ────────────────────────────────────────────────────── */

static void gyro_update_trail(Gyro *g, int cx, int cy, float scale, int cols,
                              int rows) {
  int c, r;
  float d;
  project(g->ez[0], g->ez[1], g->ez[2], g->view_phi, g->view_theta, scale, cx,
          cy, &c, &r, &d);
  g->trail[g->trail_head] = (TrailPt){c, r};
  g->trail_head = (g->trail_head + 1) % TRAIL_LEN;
  if (g->trail_fill < TRAIL_LEN)
    g->trail_fill++;

  (void)cols;
  (void)rows;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level state spanning physics ticks, draw calls, and the
 * main loop.  Wraps the gyro simulation together with the cosmetic
 * theme selection.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by scene_tick / gyro_step.
 *     Anything that affects the PHYSICS lives here.  Mutated by
 *     physics-affecting keys: r (reset), space (pause), n / p
 *     (preset cycle), g (gravity toggle), l / L (trail toggle),
 *     arrows (view), ] / [ (sim Hz).
 *
 *   Rendering parameters   — consumed by gyro_draw / draw_seg3d
 *     and by color_apply_theme.  Toggling these while paused must
 *     leave the gyro state byte-identical; only the palette swaps.
 *     Mutated by purely cosmetic keys: t / T (theme cycle).
 *
 * Locality rationale (the contract that matters, not the byte layout):
 *   The split is for the READER, not the CPU.  When adding a field,
 *   ask: does this change what gyro_step / scene_tick produces?  If
 *   yes → simulation; if purely visual → rendering.  A new knob
 *   landing in the rendering group when it actually steers gyro_step
 *   would silently couple display to physics — exactly the bug this
 *   separation is here to prevent.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   App.sim_fps                       sim/render tick rate; lives on
 *                                     the App because the main-loop
 *                                     accumulator owns the tick clock.
 *   App.running / App.need_resize     volatile sig_atomic_t flags
 *                                     written from signal handlers;
 *                                     must stay on file-scope App for
 *                                     async-signal-safety.
 *   App.screen (cols, rows)           terminal geometry tracked by
 *                                     the main loop; Scene stays
 *                                     window-agnostic so resize is
 *                                     the main loop's concern.
 *
 * Single instance: lives inside file-scope `g_app` (§8).  Gyro.trail[]
 * alone is ~2.4 KB (300 × 8 B), so the whole App sits in BSS rather
 * than being passed by value anywhere.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters (read & written by scene_tick) ──────── */
  Gyro gyro; /* physics state — see Gyro docstring §5         */

  /* ── Rendering parameters (no physics side-effects) ────────────── */
  int theme; /* index into THEMES[] in §3; t / T cycles.
              * Purely cosmetic — physics is identical
              * across all 12 themes                          */
} Scene;

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  /* memset → theme = 0 (VOLT); keep current theme on r-reset */
  gyro_init(&s->gyro);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep dt.
 *
 * SUB_STEPS RK4 steps are taken within each sim tick.  This keeps
 * the individual RK4 step small relative to the rotation period,
 * ensuring accurate integration even at the default 60 Hz tick rate.
 *
 * The view azimuth auto-rotates at 0.15 rad/s for a cinematic view.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  Gyro *g = &s->gyro;
  if (g->paused)
    return;

  float sub_dt = dt / (float)SUB_STEPS;
  for (int i = 0; i < SUB_STEPS; i++)
    gyro_step(g, sub_dt);

  /* Slow auto-rotation of view */
  g->view_phi += 0.15f * dt;

  /* Update polhode trail (once per tick, not sub-step) */
  if (g->show_trail) {
    float sc_h = (float)cols / 7.0f;
    float sc_v = (float)(rows - 2) * 2.0f / 7.0f;
    float scale = sc_h < sc_v ? sc_h : sc_v;
    gyro_update_trail(g, cols / 2, (rows + 1) / 2, scale, cols, rows);
  }
}

/*
 * scene_draw() — render the gyroscope into WINDOW *w.
 *
 * alpha accepted for framework signature compatibility but unused —
 * the gyroscope uses rigid-body physics where the draw position IS
 * the physics position (no separate interpolation needed at 60 Hz).
 */
static void scene_draw(Scene *s, WINDOW *w, int cols, int rows, float alpha,
                       float dt_sec) {
  (void)alpha;
  (void)dt_sec;
  gyro_draw(&s->gyro, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — current terminal geometry, refreshed on SIGWINCH.
 *
 * Kept separate from Scene so that resize handling (a main-loop
 * concern) doesn't touch the simulation state.  Every frame the HUD
 * layout reads cols / rows from here for right-alignment on row 0,
 * left-alignment on row 1, and last-row positioning of the key
 * legend.  gyro_draw also reads them to clip line segments.
 */
typedef struct {
  int cols, rows;
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
 * Canonical two-row HUD (CLAUDE.md):
 *   Row 0 right : fps / sim Hz / paused — PAIR_HUD + A_BOLD
 *   Row 1 left  : preset + inertia + |ω| + gravity + theme —
 *                 PAIR_HUD without A_BOLD so row 0 stays dominant
 *   Last row    : key legend — PAIR_HINT + A_BOLD
 */
static void screen_draw(Screen *s, Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  const Gyro *g = &sc->gyro;
  const GPreset *pr = &PRESETS[g->preset];

  float omag = sqrtf(g->omega[0] * g->omega[0] + g->omega[1] * g->omega[1] +
                     g->omega[2] * g->omega[2]);

  /* ── Row 0 right: canonical fps / sim / paused ── */
  char top[80];
  snprintf(top, sizeof top, " %5.1f fps  sim:%3d Hz  %s ", fps, sim_fps,
           g->paused ? "PAUSED " : "running");
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - (int)strlen(top), "%s", top);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── Row 1 left: parameter readouts ── */
  char params[200];
  snprintf(params, sizeof params,
           " preset:%d %s  I=(%.1f,%.1f,%.1f)  |ω|=%.1f rad/s"
           "  g:%s  theme:[%d] %s ",
           g->preset, pr->name, g->I1, g->I2, g->I3, omag,
           g->gravity ? "ON " : "OFF", sc->theme, THEMES[sc->theme].name);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, 0, "%.*s", s->cols, params);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* ── Bottom row: key legend ── */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  n/p:preset  r:restart  g:gravity"
           "  l:trail  t/T:theme  arrows:view  ]/[:simHz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-of-process container holding everything the main loop
 * touches.  Single file-scope instance `g_app` so the signal handlers
 * can write to running / need_resize without a context parameter
 * (POSIX signal handlers can't carry one).
 *
 * Why a single struct (not separate file-scope globals):
 *   Bundling scene / screen / sim_fps / signal flags under one `g_app`
 *   makes the read sites tidy (`g_app.running`, `g_app.need_resize`)
 *   and lets the main loop take ONE pointer (`App *app = &g_app`)
 *   instead of juggling four globals.  Forward declarations stay
 *   simple and the lifetime is unambiguous: born at program start,
 *   dies at exit.
 *
 * Field groups (mirrored by the layout below):
 *
 *   Owned subsystems    — scene, screen.  Heavy state; passed by
 *                         pointer to every helper.
 *   Tick rate knob      — sim_fps.  Lives on App (not Scene) because
 *                         the main-loop accumulator owns the tick
 *                         clock; Scene stays clock-agnostic so the
 *                         simulation contract is rate-independent.
 *   Signal-handler I/O  — running, need_resize.  volatile sig_atomic_t
 *                         per POSIX — only the simplest atomic loads
 *                         and stores are guaranteed safe between
 *                         signal handler and main thread.
 */
typedef struct {
  /* ── Owned subsystems ─────────────────────────────────────────── */
  Scene scene;   /* sim + render state (§6)      */
  Screen screen; /* terminal geometry (§7)       */

  /* ── Tick-rate knob ───────────────────────────────────────────── */
  int sim_fps; /* fixed-dt sim Hz; ] / [ keys
                * clamped [SIM_FPS_MIN, MAX]  */

  /* ── Signal-handler I/O (async-signal-safe access only) ───────── */
  volatile sig_atomic_t running;     /* 0 ⇒ main loop exits         */
  volatile sig_atomic_t need_resize; /* 1 ⇒ main loop calls
                                      *      app_do_resize next tick */
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
  app->scene.gyro.trail_head = 0;
  app->scene.gyro.trail_fill = 0;
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  Gyro *g = &sc->gyro;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    g->paused = !g->paused;
    break;

  case 'n':
    gyro_set_preset(g, (g->preset + 1) % N_PRESETS);
    break;

  case 'p':
    gyro_set_preset(g, (g->preset + N_PRESETS - 1) % N_PRESETS);
    break;

  case 'r':
  case 'R':
    gyro_set_preset(g, g->preset);
    break;

  case 'g':
  case 'G':
    g->gravity = !g->gravity;
    break;

  case 'l':
  case 'L':
    g->show_trail = !g->show_trail;
    if (!g->show_trail) {
      g->trail_head = 0;
      g->trail_fill = 0;
    }
    break;

  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    color_apply_theme(sc->theme);
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    color_apply_theme(sc->theme);
    break;

  case KEY_LEFT:
    g->view_phi -= 0.1f;
    break;
  case KEY_RIGHT:
    g->view_phi += 0.1f;
    break;
  case KEY_UP:
    g->view_theta += 0.05f;
    if (g->view_theta > 1.4f)
      g->view_theta = 1.4f;
    break;
  case KEY_DOWN:
    g->view_theta -= 0.05f;
    if (g->view_theta < 0.1f)
      g->view_theta = 0.1f;
    break;

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

/* ── main-loop building blocks ───────────────────────────────────────── */

/*
 * install_signal_handlers — register atexit cleanup and the three
 * POSIX signals we care about.  SIGINT / SIGTERM trigger graceful
 * shutdown; SIGWINCH triggers a viewport rebuild on the next tick.
 * Handlers must be async-signal-safe — they only flip volatile flags.
 */
static void install_signal_handlers(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

/*
 * handle_resize_pending — process the SIGWINCH flag if it's set.
 * Re-queries terminal geometry, drops the stale screen-space trail,
 * and rebaselines the timing locals (wall clock advanced while the
 * resize handler was running; pretend we're starting fresh so the
 * next dt doesn't blow up).
 */
static void handle_resize_pending(App *app, int64_t *frame_time,
                                  int64_t *sim_accum) {
  if (!app->need_resize)
    return;
  app_do_resize(app);
  *frame_time = clock_ns();
  *sim_accum = 0;
}

/*
 * step_simulation_fixed_dt — classic fixed-dt accumulator.
 *
 *   sim_accum += wall_dt
 *   while (sim_accum >= tick_ns):
 *     scene_tick(dt_sec)
 *     sim_accum -= tick_ns
 *
 * Decouples sim rate from render rate: the integration step is always
 * exactly dt_sec wide regardless of how fast the renderer is running,
 * so accuracy (per the RK4 truncation bound) is independent of frame
 * rate.  sim_accum carries any leftover < tick_ns to the next frame.
 */
static void step_simulation_fixed_dt(Scene *scene, int64_t *sim_accum,
                                     int64_t dt_ns, int64_t tick_ns,
                                     float dt_sec, int cols, int rows) {
  *sim_accum += dt_ns;
  while (*sim_accum >= tick_ns) {
    scene_tick(scene, dt_sec, cols, rows);
    *sim_accum -= tick_ns;
  }
}

/*
 * update_fps_counter — rolling average FPS, refreshed every
 * FPS_UPDATE_MS (500 ms) so the displayed number is stable enough to
 * read.  Without the averaging window the readout would jitter wildly
 * frame-to-frame even at steady throughput.
 */
static void update_fps_counter(int64_t dt_ns, int64_t *fps_accum,
                               int *frame_count, double *fps_display) {
  (*frame_count)++;
  *fps_accum += dt_ns;
  if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
    *fps_display =
        (double)(*frame_count) / ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum = 0;
  }
}

/*
 * cap_to_render_framerate — sleep before the render write so each
 * frame takes ~1/60 s regardless of how fast sim + draw completed.
 * Cap is INDEPENDENT of sim_fps — sim runs at its own rate via the
 * accumulator above; the renderer is held at a steady 60 fps for
 * consistent visual cadence on every terminal.
 */
static void cap_to_render_framerate(int64_t frame_start_ns,
                                    int64_t prev_frame_dt_ns) {
  int64_t elapsed = clock_ns() - frame_start_ns + prev_frame_dt_ns;
  clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
}

/*
 * main — the demo's top-level loop.
 *
 * Pseudocode:
 *   srand + install signal handlers
 *   initialise scene (memset → VOLT theme, gyro_set_preset(0))
 *   initialise screen (ncurses + colour pairs from current theme)
 *
 *   loop until app->running goes false:
 *     handle pending resize (if SIGWINCH fired)
 *     measure wall-clock dt since last frame
 *     step_simulation_fixed_dt  // run scene_tick N times to drain accum
 *     update_fps_counter        // every FPS_UPDATE_MS ms
 *     cap_to_render_framerate   // hold ~60 fps render
 *     screen_draw + screen_present
 *     poll input → app_handle_key (returns false ⇒ quit)
 */
int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFFu));
  install_signal_handlers();

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  scene_init(&app->scene);                     /* theme = 0 (VOLT) */
  screen_init(&app->screen, app->scene.theme); /* color_init wants theme */

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {
    handle_resize_pending(app, &frame_time, &sim_accum);

    /* wall-clock dt since previous frame (clamped to 100 ms to
     * prevent spiral-of-death after a hiccup or breakpoint pause) */
    int64_t now = clock_ns();
    int64_t dt_ns = now - frame_time;
    frame_time = now;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    step_simulation_fixed_dt(&app->scene, &sim_accum, dt_ns, tick_ns, dt_sec,
                             app->screen.cols, app->screen.rows);

    /* alpha ∈ [0,1) — render interpolation factor for sub-tick
     * smoothing.  gyroscope draws at physics position directly
     * (alpha unused), kept for framework signature parity. */
    float alpha = (float)sim_accum / (float)tick_ns;

    update_fps_counter(dt_ns, &fps_accum, &frame_count, &fps_display);
    cap_to_render_framerate(now, dt_ns);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
