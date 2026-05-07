/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb.c — a 3-D Mandelbulb fractal raymarcher
 *
 * DEMO: An auto-orbiting view of the canonical p = 8 Mandelbulb.
 *       Soft shadows from a fixed light, ambient occlusion via march
 *       step count, smooth-iteration colouring across a 5-theme
 *       palette (one of which renders the fractal as a photographic
 *       negative).  Iteration depth, colour theme, camera distance,
 *       and orbit angles are live-controllable.
 *
 * Study alongside: raymarcher/raymarcher.c (sphere — same march
 *       loop with a one-line SDF) and raymarcher/raymarcher_cube.c
 *       (box SDF + tetrahedral normal).  This file is what happens
 *       when the SDF stops being a "true" distance function and
 *       becomes a CONSERVATIVE LOWER BOUND — the marcher needs
 *       extra tricks to stay correct.
 *
 * Section map:
 *   §1  config       — every tunable named, no magic numbers later
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — themes (incl. NEGATIVE inverted) + HUD pairs
 *   §4  vec3         — 3-D math, value types
 *   §5  mandelbulb   — iteration, distance estimator, smooth iter, normal
 *   §6  march        — sphere trace + soft shadow + AO + Phong shade
 *   §7  scene        — camera basis, ray gen, decorate + emit, render
 *   §8  screen       — ncurses init / HUD / present
 *   §9  app          — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the orbit
 *   r / R      reset camera + iteration depth
 *   t / T      next / previous theme
 *                (CLASSIC / ICE / PLASMA / MONO / NEGATIVE)
 *   i / I      iteration depth − / +   (3..14, default 8)
 *   z / Z      zoom in / out (camera closer / farther)
 *   arrows     manual orbit (left/right yaw, up/down pitch)
 *   ] / [      simulation rate up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/mandelbulb.c \
 *       -o mandelbulb -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is structured as a textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of the iteration.
 *   • GUIDED TUTORIAL  eight short answers walking from the 2D
 *                      Mandelbrot up through "why a fractal SDF
 *                      needs extra marching tricks".
 *   • §1..§9           the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in the code:
 *      z          — the iteration variable (a 3-D point)
 *      c          — the parameter (the input point we're testing)
 *      r, θ, φ    — spherical coords:  r = |z|, θ = polar from +y,
 *                                       φ = azimuth in xz plane
 *      dr         — running derivative magnitude (used by the DE)
 *      p          — the Mandelbulb power exponent (8 here)
 *      DE(p)      — distance estimator at point p
 *      N          — surface normal at the hit
 *      L          — light direction (unit vector)
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, cross, length, normalise)
 *   • familiarity with the 2D Mandelbrot iteration z ← z² + c
 *     helps a lot — Tutorial 1 recaps it briefly anyway
 *   • read raymarcher.c first if sphere tracing is unfamiliar; this
 *     file extends the same march loop with new tricks for fractals.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : MANDELBULB DISTANCE ESTIMATOR (Daniel White / Paul
 *                Nylander, 2009).  The 2D Mandelbrot iteration
 *                z ← z² + c generalised to 3D by performing the
 *                squaring (or in this case, raising to power 8) in
 *                spherical coordinates.  Combined with the Hubbard-
 *                Douady distance estimator
 *                    DE(p) = ½ · log(|z|) · |z| / |z'|
 *                this yields a LOWER BOUND on distance from p to
 *                the fractal surface — close enough for sphere
 *                tracing, with two extra tuning knobs (step
 *                relaxation + adaptive epsilon) to handle the
 *                "lower bound, not exact" nature.
 *
 * Data         : Stateless math (V3 + DE + trace + shade) on the
 *                hot path.  Per-pixel result is `Hit` (hit / p /
 *                normal / smooth_iter / luminance / march_steps);
 *                rendered to one (glyph, colour pair, attr) Cell
 *                per terminal cell.
 *
 * Rendering    : One ray per terminal cell (no virtual canvas).
 *                Glyph from a faint-to-solid 8-char ramp; colour
 *                from the active theme's 8-tier 256-colour ramp
 *                indexed by smooth iteration count.  One theme
 *                (NEGATIVE) renders the fractal photographic-
 *                negative — white background, dark foreground —
 *                handled by an `inverted` flag in the Theme struct.
 *
 * Performance  : ~6 DE evaluations per hit pixel (1 trace step at
 *                hit + 1 smooth-iter eval + 6-tap normal − 1 shared
 *                = ~6).  Plus SHADOW_STEPS (16) DEs per shadow ray
 *                (only for hits).  At ITERS_DEFAULT = 8 and a
 *                modest terminal, this stays north of 60 fps on
 *                modern CPUs.
 *
 * References   :
 *   • Daniel White & Paul Nylander (2009) — "Mandelbulb"
 *     https://www.skytopia.com/project/fractal/mandelbulb.html
 *     The original derivation.
 *   • Hubbard, J. H. & Douady, A. — distance estimator for the
 *     Mandelbrot set (general technique extended here to the bulb).
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The march loop.
 *   • Quílez, I. — "Distance Estimators for Implicit Surfaces"
 *     https://iquilezles.org/articles/distancefractals/
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a point c in 3D space.  Iterate z ← z^8 + c (where the
 * "raising to the 8th power" happens in spherical coordinates
 * around c).  If after a few iterations |z| stays small, c is INSIDE
 * the Mandelbulb; if |z| explodes off to infinity, c is outside.
 * The fractal SURFACE is the boundary between the two.  We don't
 * just test inside vs outside — we use the rate at which |z|
 * escapes (and the running magnitude of its derivative) to estimate
 * HOW FAR c is from the surface.  That distance feeds straight into
 * sphere tracing.  No mesh, no triangles, no voxels — just an
 * iteration at every sample point along every ray.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The 2D Mandelbrot is a fixed-point classifier: for every complex
 * c, "does the iteration converge or diverge?"  The Mandelbulb is
 * the same question asked of every point in 3D space, with the
 * complex squaring replaced by a spherical-coordinate operation
 * that's well-defined in any number of dimensions.  At power 8 the
 * iteration's symmetry group is the rotation group acting on the
 * sphere — the resulting fractal has 8-fold rotational symmetry
 * around the polar axis and a "lumpy / spiked" appearance with
 * deep grottoes between the lobes.
 *
 * One iteration, in cross-section (along the +y polar axis):
 *
 *      z = (r, θ, φ)                           (spherical coords)
 *           │
 *           ▼  raise radius to the 8th power
 *      r' = r^8                                 (radial stretch)
 *           │
 *           ▼  multiply both angles by 8
 *      θ' = 8θ,  φ' = 8φ                       (angular spin-up)
 *           │
 *           ▼  back to Cartesian, add c
 *      z' = (r' sin θ' cos φ', r' cos θ', r' sin θ' sin φ') + c
 *
 *      ▲ DE = ½·log(|z|)·|z| / |z'|
 *      └── Hubbard-Douady estimator: closer to the surface
 *          ⇒ smaller DE.  Used as the safe step distance.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. orbit       advance auto-yaw (paused freezes it)
 *   2. camera      build orthonormal (fwd, right, up) at the orbiting
 *                  position, compute FOV tangent + aspect correction
 *
 * Once per pixel:
 *   3. ray         build a primary ray from camera through the cell
 *   4. trace       sphere-march with adaptive ε + step relaxation
 *   5. on hit:     6-tap central-difference normal
 *                  smooth iteration count for colour
 *                  Phong + soft shadow + AO → final luminance
 *
 * Once per cell:
 *   6. quantise    luminance → glyph slot (8 levels)
 *                  smooth_iter / max_iter → colour slot (8 levels)
 *   7. emit        with attron/attroff batched on (pair, attr) change
 *
 * KEY FORMULAS
 * ────────────
 * Spherical coordinates:
 *      r = |z|,  θ = acos(z.y / r),  φ = atan2(z.z, z.x)
 *
 * Mandelbulb iteration (power p):
 *      z' = r^p · (sin pθ · cos pφ,  cos pθ,  sin pθ · sin pφ) + c
 *
 * Running derivative magnitude:
 *      dr ← p · r^(p−1) · dr  +  1
 *      (this tracks how fast |z| would change with a perturbation
 *       in c — needed by the distance estimator)
 *
 * Hubbard-Douady distance estimator (lower bound, not exact):
 *      DE(p) = ½ · log(|z|) · |z| / dr
 *
 * Smooth iteration count (continuous across the boundary):
 *      smooth = i + 1 − log₂(log(|z|) / log(BAILOUT)) / log₂(p)
 *
 * Sphere trace with under-relaxation + adaptive ε:
 *      t  ← t + α · DE(ro + t·rd)        α = STEP_RELAX < 1
 *      ε  = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *      hit when DE < ε,  miss when t > MAX_T
 *
 * Soft shadow (Christensen):
 *      result = 1
 *      for each step toward the light:
 *          result = min(result, K · DE(p) / t)
 *      result = clamp(result, SHADOW_FLOOR, 1)
 *
 * Cheap AO (step count proxy):
 *      ao = 1 − (march_steps / MAX_STEPS) · AO_STRENGTH
 *      (cells deep in concavities take more steps → naturally darker)
 *
 * Phong combine:
 *      L = AMBIENT + (1 − AMBIENT) · max(0, N·L_dir) · soft · ao
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • The DE is a LOWER BOUND on true distance, not exact.  Without
 *     STEP_RELAX (α < 1) the marcher overshoots at grazing angles.
 *     Set α = 1.0 and you'll see "shadow acne" — speckle pattern
 *     across slopes that should be smooth.
 *
 *   • ITERS too low (< 5) makes the surface look bulbous and smooth;
 *     too high (> 12) makes it crumble into noise as floating-point
 *     accumulates.  ITERS_DEFAULT = 8 is the canonical sweet spot.
 *
 *   • ADAPTIVE_FACTOR widens ε linearly with t.  At t = MAX_T the
 *     effective ε is ~7× the near-camera value — this avoids wasting
 *     march steps on sub-pixel-precision when far from the surface,
 *     but if pushed too high (say 0.05) you'll see the silhouette
 *     bulge outward at large distances.
 *
 *   • The fractal lives roughly in |p| < 1.5.  CAM_DIST_MIN = 1.5
 *     keeps the camera outside the bulb's bounding sphere; pushing
 *     closer puts the eye inside lobes and rays start with negative
 *     DE, breaking the marcher.
 *
 *   • The NEGATIVE theme requires a white CANVAS background — see
 *     prefill_canvas.  A_BOLD/A_DIM are also disabled for this theme
 *     because they invert their visual meaning against a light bg.
 *
 *   • SHADOW_STEPS = 16 is enough for the bulb's typical occlusion
 *     scale; larger values cost frame rate without visible quality.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At ITERS = 8 and default zoom, the silhouette should show
 *     8-fold rotational symmetry around the vertical axis (count
 *     the lobes around the equator).  Tap left/right arrows to
 *     orbit and confirm.
 *
 *   • Press i to drop iters to 3.  Surface becomes a smooth,
 *     pumpkin-like blob — the fractal detail emerges only at higher
 *     iterations.  Press I to crank to 14 and watch detail appear.
 *
 *   • Press t through all 5 themes.  Geometry is identical; only
 *     the colour mapping changes.  NEGATIVE switches to white-bg,
 *     dark-fg — verify the HUD and key hint stay readable.
 *
 *   • Press z several times to zoom in.  At CAM_DIST_MIN the camera
 *     touches the bulb's bounding sphere; close-up shows surface
 *     detail clearly.
 *
 *   • Press space to pause; orbit freezes mid-arc.
 *
 *   • Resize the terminal: HUD reflows, fractal re-centres.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — eight short answers ───────────────────────────── *
 *
 *
 * T1: Mandelbrot in 2D — the iteration we're generalising
 * ───────────────────────────────────────────────────────
 * For each complex number c, run:
 *
 *      z₀ = 0
 *      z_{n+1} = z_n² + c
 *
 * If |z_n| stays bounded (always < 2 suffices) for N iterations,
 * c is "in" the Mandelbrot set.  Otherwise c is outside, and the
 * iteration n at which |z_n| first crosses the bailout is a measure
 * of how QUICKLY c escapes.  Plotting black-vs-coloured by escape
 * iteration gives the iconic Mandelbrot fractal.
 *
 * Key insight: the boundary of the set is INFINITELY DETAILED.
 * Every zoom level reveals new structure.  This is what makes
 * fractals geometrically interesting and notoriously hard to render
 * by triangle meshing.
 *
 *
 * T2: Why doesn't z² + c work in 3D?
 * ──────────────────────────────────
 * Complex multiplication is intrinsically 2-dimensional.  z² in 2D
 * is "rotate z's argument by 2× and square its magnitude" — a
 * geometric operation that needs both an angle and a length.  In
 * 3D there's no single "argument" angle; you'd need at least two
 * angles (azimuth + elevation), and the "obvious" generalisation
 * is ambiguous.
 *
 * Quaternions extend complex numbers to 4D and give a unique z² + c,
 * but quaternionic Julia sets are smooth blobs — visually
 * unimpressive.  Daniel White's 2009 trick: define z^p in 3D by
 * transforming the squaring operation to spherical coordinates,
 * where it has a clean generalisation.  Pick the power, watch
 * fractal lobes form.
 *
 *
 * T3: The spherical-power trick
 * ─────────────────────────────
 * For a 3-D point z = (z.x, z.y, z.z), convert to spherical:
 *
 *      r = |z|                          radial distance
 *      θ = acos(z.y / r)                polar angle (from +y axis)
 *      φ = atan2(z.z, z.x)              azimuth in the xz plane
 *
 * The Mandelbulb's z^p operation is then:
 *
 *      z^p = r^p · ( sin(pθ) · cos(pφ),
 *                     cos(pθ),
 *                     sin(pθ) · sin(pφ) )
 *
 * Then add c (still in Cartesian) and you have one iteration step.
 *
 * Worked example with p = 8 at z = (0.5, 0.5, 0):
 *      r   = √(0.25 + 0.25 + 0) ≈ 0.707
 *      θ   = acos(0.5 / 0.707) = acos(0.707) ≈ 0.785 rad (45°)
 *      φ   = atan2(0, 0.5) = 0
 *      r⁸  ≈ 0.0625
 *      z^8 ≈ 0.0625 · (sin 6.28 · cos 0, cos 6.28, sin 6.28 · sin 0)
 *          ≈ 0.0625 · (0, 1, 0)
 *          ≈ (0, 0.0625, 0)
 *
 * The point shrunk hugely (r^8 with r < 1 → r much less than 1)
 * and rotated 8× around both axes.  After many iterations these
 * folds compose into the iconic spiked-bulb shape.
 *
 *
 * T4: Counting iterations is too coarse — distance estimation
 * ───────────────────────────────────────────────────────────
 * For 2D Mandelbrot rendering, you ASSIGN a colour by iteration
 * count and you're done — there's no "where exactly is the boundary
 * pixel?", because every pixel just gets a colour.  For raymarching
 * a 3D object, we need more: how far is THIS point from the
 * surface, so the marcher can step safely?
 *
 * Hubbard-Douady distance estimator:
 *
 *      DE(c) = ½ · log(|z|) · |z| / |z'|
 *
 * where z is the iteration value at the time of escape and |z'| is
 * the magnitude of dz/dc (how fast z changes with a perturbation
 * in c).  We track |z'| with a running scalar `dr` that updates as:
 *
 *      dr_{n+1} = p · |z_n|^(p−1) · dr_n + 1
 *
 * The DE is a CONSERVATIVE LOWER BOUND on true Euclidean distance.
 * That's important — see T5.
 *
 *
 * T5: Sphere tracing a NON-EXACT distance function
 * ────────────────────────────────────────────────
 * For sphere SDF (raymarcher.c) and box SDF (raymarcher_cube.c)
 * the distance returned was EXACT — Lipschitz-1 true distance.  The
 * marcher could step the full d safely.
 *
 * The Hubbard-Douady DE is only a LOWER BOUND.  Stepping the full
 * d can OVERSHOOT at grazing angles where the true distance is
 * much smaller than the estimate.  Two tricks fix this:
 *
 *   STEP RELAXATION: scale the step by α < 1.
 *      t ← t + α · DE(p),  with α = STEP_RELAX = 0.85
 *      Sacrifice some march speed for guaranteed-safe steps.
 *
 *   ADAPTIVE EPSILON: the "we're touching" threshold ε grows with t:
 *      ε(t) = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *      A pixel at t = 5 doesn't need sub-millimetre precision; it
 *      occupies more world distance per cell.  Widening ε avoids
 *      wasting steps when the surface is far away.
 *
 * Without either trick, the silhouette breaks up into speckle
 * (overshoots that fail to converge).
 *
 *
 * T6: Shading a fractal — three contributions
 * ───────────────────────────────────────────
 * Three components combine into the per-pixel luminance:
 *
 *   LAMBERT       max(0, N·L_dir)
 *                 N estimated by 6-tap central differences:
 *                   Nₓ = DE(p + ε x̂) − DE(p − ε x̂)   etc.
 *                 Then normalise.  6 DE evals per hit pixel — by
 *                 far the dominant frame cost.
 *
 *   SOFT SHADOW   march from p toward the light, tracking
 *                   min over t of (K · DE / t)
 *                 Result is in [0, 1]: 1 means "no occluder seen",
 *                 small values mean "something almost crossed the
 *                 light path".  K (SHADOW_K) controls penumbra
 *                 hardness.
 *
 *   AMBIENT OCC   1 − (step_count / MAX_STEPS) · AO_STRENGTH
 *                 Free AO from the marcher itself: rays that travel
 *                 deep into concavities take many march steps to
 *                 converge → step count correlates with concavity →
 *                 darker crevices automatically.  Cleanest free AO
 *                 you can ask for in a sphere tracer.
 *
 * Final:
 *      L = AMBIENT + (1 − AMBIENT) · ndl · soft · ao
 *
 *
 * T7: Smooth iteration count — gradient instead of bands
 * ──────────────────────────────────────────────────────
 * Naïve colouring = "use iteration count i as the colour index".
 * Result: visible bands at iteration boundaries, especially across
 * the smooth surface where neighbouring pixels differ by exactly
 * one iteration.
 *
 * Smooth iteration count gives a CONTINUOUS function:
 *
 *      smooth = i + 1 − log₂(log(|z|) / log(B)) / log₂(p)
 *
 * where i is the integer escape iteration, |z| is the magnitude
 * at escape, B is the bailout (= 4 here), and p is the power (= 8).
 * The expression is derived so that as a sample crosses an
 * iteration boundary, smooth advances continuously.  Plot it as a
 * colour and you get smooth gradients across the surface, not
 * stripes.
 *
 *
 * T8: Camera with auto-orbit + manual user offsets
 * ────────────────────────────────────────────────
 * The camera position is decomposed into:
 *
 *      yaw   = orbit_yaw + user_yaw         (auto + arrow keys)
 *      pitch = orbit_pitch + user_pitch     (default tilt + arrow keys)
 *
 *      eye = cam_dist · (cos pitch · cos yaw,
 *                        sin pitch,
 *                        cos pitch · sin yaw)
 *
 * orbit_yaw advances at ORBIT_YAW_RATE radians per second; pause
 * stops it.  user_yaw / user_pitch accumulate from arrow keys, so
 * the user can offset the auto-orbit live.  pitch is clamped to
 * ±MANUAL_PITCH_MAX to avoid the gimbal-lock singularity at the
 * poles.
 *
 * From eye, build orthonormal (fwd, right, up):
 *
 *      fwd   = normalise(0 − eye)               (look at origin)
 *      right = normalise(fwd × world_up)
 *      up    = right × fwd
 *
 * Per pixel, the ray direction is:
 *
 *      ray = forward + u·tan(FOV/2)·right
 *                    + v·tan(FOV/2)·aspect·up
 *
 * where u, v ∈ [−1, +1] are the pixel's NDC and `aspect` corrects
 * for terminal cells being roughly twice as tall as they are wide.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,
    FPS_UPDATE_MS    = 500,
    HUD_ROWS         =   2,    /* row 0 status + last row hint */
    ITERS_MIN        =   3,
    ITERS_MAX        =  14,
    ITERS_DEFAULT    =   8,    /* the canonical Mandelbulb at p = 8 */
};

/* §1.2 colour-pair IDs. */
enum {
    PAIR_HUD         =  1,     /* yellow + bold — top status row     */
    PAIR_HINT        =  2,     /* cyan   + bold — last row key hint  */
    PAIR_RAMP_BASE   =  3,     /* +0..+7 — depth ramp                */
};

/* §1.3 time helpers + cell aspect. */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h / w */

/* §1.4 Mandelbulb iteration. */
#define MANDELBULB_POWER 8.0f      /* the canonical power; T3 derives why */
#define BAILOUT          4.0f      /* |z| > this → escaped; T1            */

/* §1.5 sphere trace (with the two fractal-specific knobs from T5). */
#define MAX_STEPS        90
#define HIT_EPS          1.0e-3f
#define ADAPTIVE_FACTOR  0.012f    /* ε grows with t                      */
#define MAX_T            6.0f
#define STEP_RELAX       0.85f     /* α < 1: under-relax for safe stepping */

/* §1.6 normal estimator epsilon. */
#define NORMAL_EPS       3.5e-3f

/* §1.7 soft shadow (Christensen).
 *      SHADOW_K       hardness — bigger = sharper shadow edge
 *      SHADOW_FLOOR   minimum factor (cells in shadow stay this lit)
 */
#define SHADOW_STEPS     16
#define SHADOW_NEAR      0.012f
#define SHADOW_FAR       2.5f
#define SHADOW_K         32.0f
#define SHADOW_FLOOR     0.30f

/* §1.8 lighting + AO + ambient. */
#define AMBIENT          0.18f
#define AO_FLOOR         0.35f
#define AO_STRENGTH      0.70f

/* Light direction (normalised at use). */
#define LIGHT_X          0.55f
#define LIGHT_Y          0.75f
#define LIGHT_Z         -0.25f

/* §1.9 camera. */
#define CAM_DIST_DEFAULT  3.2f
#define CAM_DIST_MIN      1.5f      /* outside the bulb's bounding sphere */
#define CAM_DIST_MAX      8.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          45.0f
#define ORBIT_YAW_RATE    0.30f     /* rad / sec auto-orbit               */
#define ORBIT_PITCH_DEF   0.25f     /* default static tilt above equator  */
#define MANUAL_YAW_STEP   0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.30f     /* clamp short of the poles           */

/* §1.10 quantisation — number of glyph / colour slots. */
#define LUMA_SLOTS       8
#define LUMA_SLOT_FLT    7.999f     /* (LUMA_SLOTS - 0.001) */

/* §1.11 themes — five 8-tier 256-colour ramps.
 *
 * Slot 0 = outermost shell (escape early, low smooth iter); slot 7
 * = deep interior (iteration didn't escape).  All entries except
 * the NEGATIVE theme sit in the bright half of the 256-cube per
 * the CLAUDE.md theme rule.
 *
 * NEGATIVE is photographic-negative: white background, dark
 * foreground.  The `inverted` flag triggers special handling in
 * theme_apply (white canvas bg) and luma_attr (no A_BOLD/A_DIM,
 * which would invert the brightness intent against a light bg).
 */
typedef struct {
    const char *name;
    short       ramp[LUMA_SLOTS];
    bool        inverted;
} Theme;

#define N_THEMES 5

static const Theme THEMES[N_THEMES] = {
    /* CLASSIC: warm crimson → red → orange → amber → bone — Daniel
     * White's iconic "alien fruit lit by sunset" palette. */
    { "CLASSIC ",
      { 124, 160, 196, 202, 208, 214, 220, 229 }, false },

    /* ICE: deep teal → bright cyan → ice → near-white. */
    { "ICE     ",
      {  30,  37,  44,  51,  87, 123, 159, 195 }, false },

    /* PLASMA: high-saturation neon arc — magenta → cyan → yellow. */
    { "PLASMA  ",
      { 125, 165, 207, 213,  87, 123, 220, 229 }, false },

    /* MONO: clean grayscale — best for studying fractal shape. */
    { "MONO    ",
      { 240, 244, 247, 250, 252, 253, 254, 231 }, false },

    /* NEGATIVE: photographic-negative inversion (see comment above). */
    { "NEGATIVE",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true  },
};

/* §1.12 luminance ramp — slot 0 = `.` (faint), slot 7 = `@` (solid).
 * No space at slot 0 so even the dimmest hit pixel paints something
 * visible against the background. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {
    '.', ',', ':', ';', '+', '*', '#', '@'
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)  (ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ── §3 color — depth ramp + HUD/hint pairs ──────────────────────────── *
 *
 * Eight depth-ramp pairs (PAIR_RAMP_BASE..+7) hold the active
 * theme's colour ramp.  Two more (PAIR_HUD, PAIR_HINT) are
 * theme-independent yellow + cyan for the HUD strips.
 *
 * Inverted themes use a white bg (256-colour 231) so subsequent
 * mvaddch(' ') in prefill_canvas paints the canvas white before the
 * fractal draws over it.
 */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &THEMES[idx];
    short bg256 = t->inverted ? 231        : -1;
    short bg8   = t->inverted ? COLOR_WHITE : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg256);
    } else {
        static const short FB[LUMA_SLOTS] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
            COLOR_RED,  COLOR_RED,  COLOR_YELLOW,  COLOR_WHITE,
        };
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_RAMP_BASE + i),
                      t->inverted ? COLOR_BLACK : FB[i], bg8);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3   (float x, float y, float z) { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)                { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)                { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3scale(float s, V3 a)           { return v3(s*a.x, s*a.y, s*a.z); }
static inline float v3dot(V3 a, V3 b)                { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float L = v3len(a);
    return (L > 1e-12f) ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ── §5 mandelbulb — iteration + DE + smooth iter + normal ───────────── *
 *
 * Tutorials T3, T4, T7 derived everything in this section.  The
 * iteration body is split into three helpers (Spherical conversion,
 * dr update, power-and-add) so the per-step math is line-by-line
 * verifiable against the formulas.  At -O2 they all inline back
 * into one tight loop — no performance cost for the readability.
 */

/* z in spherical coords: r = |z|, θ = polar from +y, φ = azimuth in xz */
typedef struct { float r, theta, phi; } Spherical;

static inline Spherical to_spherical(V3 z)
{
    float r = sqrtf(z.x*z.x + z.y*z.y + z.z*z.z);
    Spherical s = { r, 0.0f, 0.0f };
    if (r > 1e-20f) {
        s.theta = acosf(z.y / r);
        s.phi   = atan2f(z.z, z.x);
    }
    return s;
}

/* dr_new = p · r^(p−1) · dr + 1   (running derivative magnitude, T4) */
static inline float update_dr(float dr, float r)
{
    return powf(r, MANDELBULB_POWER - 1.0f) * MANDELBULB_POWER * dr + 1.0f;
}

/* z' = r^p · (sin pθ · cos pφ, cos pθ, sin pθ · sin pφ) + c   (T3) */
static inline V3 apply_power_and_add(Spherical s, V3 c)
{
    float zr     = powf(s.r, MANDELBULB_POWER);
    float p_th   = MANDELBULB_POWER * s.theta;
    float p_ph   = MANDELBULB_POWER * s.phi;
    float sin_th = sinf(p_th);
    return v3(zr * sin_th * cosf(p_ph) + c.x,
              zr * cosf(p_th)          + c.y,
              zr * sin_th * sinf(p_ph) + c.z);
}

/*
 * mandelbulb_de — distance estimator + optional smooth iteration count.
 * Returns ½ · log(|z|) · |z| / dr.  If smooth_out is non-NULL also
 * writes the smooth (continuous) escape iteration count.  Pass NULL
 * when computing normals (no need for the smooth count there).
 */
static float mandelbulb_de(V3 p, int max_iter, float *smooth_out)
{
    V3        z         = p;
    float     dr        = 1.0f;
    Spherical s         = { 0.0f, 0.0f, 0.0f };
    int       i;

    const float log2_power = log2f(MANDELBULB_POWER);
    const float log_bail   = logf(BAILOUT);

    for (i = 0; i < max_iter; i++) {
        s  = to_spherical(z);
        if (s.r > BAILOUT) break;
        dr = update_dr(dr, s.r);
        z  = apply_power_and_add(s, p);
    }

    if (smooth_out) {
        if (i >= max_iter) {
            *smooth_out = (float)max_iter;       /* didn't escape */
        } else {
            float ln_r = logf(s.r);
            if (ln_r > 0.0f && log_bail > 0.0f)
                *smooth_out = (float)i + 1.0f
                            - log2f(ln_r / log_bail) / log2_power;
            else
                *smooth_out = (float)i;
        }
    }
    return 0.5f * logf(s.r) * s.r / dr;
}

/*
 * mandelbulb_normal — surface normal at p via 6-tap central differences.
 * Two DE evals per axis × 3 axes = 6 DEs total (no smooth count needed).
 * Symmetric around p so the resulting N has no octant bias.
 */
static V3 mandelbulb_normal(V3 p, int max_iter)
{
    float e = NORMAL_EPS;
    float dx = mandelbulb_de(v3(p.x + e, p.y, p.z), max_iter, NULL)
             - mandelbulb_de(v3(p.x - e, p.y, p.z), max_iter, NULL);
    float dy = mandelbulb_de(v3(p.x, p.y + e, p.z), max_iter, NULL)
             - mandelbulb_de(v3(p.x, p.y - e, p.z), max_iter, NULL);
    float dz = mandelbulb_de(v3(p.x, p.y, p.z + e), max_iter, NULL)
             - mandelbulb_de(v3(p.x, p.y, p.z - e), max_iter, NULL);
    return v3norm(v3(dx, dy, dz));
}

/* ── §6 march — sphere trace + soft shadow + AO + Phong shade ────────── *
 *
 * Tutorials T5, T6 derived the contents.  Three functions:
 * sphere_trace returns where (and after how many steps) the ray hit
 * the surface; soft_shadow returns a [0,1] visibility from a hit
 * point toward the light; shade combines Lambert + soft shadow + AO
 * into the final luminance.
 */

typedef struct {
    bool  hit;
    V3    p;
    int   march_steps;     /* AO signal: more steps = more occluded */
} TraceResult;

static TraceResult sphere_trace(V3 origin, V3 dir, int max_iter)
{
    TraceResult tr = { false, {0, 0, 0}, 0 };
    float t = 0.0f;
    int   step;

    for (step = 0; step < MAX_STEPS; step++) {
        V3    p   = v3add(origin, v3scale(t, dir));
        float d   = mandelbulb_de(p, max_iter, NULL);
        float eps = HIT_EPS * (1.0f + t * ADAPTIVE_FACTOR);

        if (d < eps) {
            tr.hit         = true;
            tr.p           = p;
            tr.march_steps = step;
            return tr;
        }
        if (t > MAX_T) break;
        t += d * STEP_RELAX;
    }
    tr.march_steps = step;
    return tr;
}

/*
 * soft_shadow — Christensen-style soft shadow from origin toward light.
 * Returns a visibility factor in [SHADOW_FLOOR, 1].  Tracks the
 * minimum SHADOW_K · DE / t over the shadow march; that minimum
 * naturally produces a soft penumbra without any extra rays.
 */
static float soft_shadow(V3 origin, V3 light_dir, int max_iter)
{
    float result = 1.0f;
    float t      = SHADOW_NEAR;     /* offset off the surface to avoid self-shadow */

    for (int i = 0; i < SHADOW_STEPS; i++) {
        V3    p = v3add(origin, v3scale(t, light_dir));
        float d = mandelbulb_de(p, max_iter, NULL);

        if (d < HIT_EPS) return SHADOW_FLOOR;     /* fully blocked */

        float k = SHADOW_K * d / t;
        if (k < result) result = k;

        t += d;
        if (t > SHADOW_FAR) break;
    }
    return clampf(result, SHADOW_FLOOR, 1.0f);
}

/*
 * shade — combine Lambert + soft shadow + cheap AO into final luminance.
 *      L = AMBIENT + (1 − AMBIENT) · max(0, N·L_dir) · soft · ao
 * The cheap AO uses the trace's step count as a concavity proxy (T6).
 */
static float shade(V3 hit_p, V3 normal, V3 light_dir,
                   int max_iter, int march_steps)
{
    float ndl = v3dot(normal, light_dir);
    if (ndl < 0.0f) ndl = 0.0f;

    float soft = soft_shadow(hit_p, light_dir, max_iter);

    float ao = 1.0f - ((float)march_steps / (float)MAX_STEPS) * AO_STRENGTH;
    if (ao < AO_FLOOR) ao = AO_FLOOR;

    return clampf(AMBIENT + (1.0f - AMBIENT) * ndl * soft * ao, 0.0f, 1.0f);
}

/* ── §7 scene — camera basis, ray gen, decorate + emit, render ───────── */

typedef struct {
    bool   paused;
    int    current_theme;
    int    iters;
    int    cols, rows;

    /* Camera (T8). */
    float  cam_dist;
    float  orbit_yaw;            /* auto-advancing                    */
    float  orbit_pitch;          /* fixed default tilt                */
    float  user_yaw, user_pitch; /* manual offsets via arrow keys     */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->current_theme = 0;
    s->iters         = ITERS_DEFAULT;
    s->cols          = cols;
    s->rows          = rows;
    s->cam_dist      = CAM_DIST_DEFAULT;
    s->orbit_yaw     = 0.5f;
    s->orbit_pitch   = ORBIT_PITCH_DEF;
    s->user_yaw      = 0.0f;
    s->user_pitch    = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset_cam(Scene *s)
{
    s->cam_dist     = CAM_DIST_DEFAULT;
    s->orbit_yaw    = 0.5f;
    s->orbit_pitch  = ORBIT_PITCH_DEF;
    s->user_yaw     = 0.0f;
    s->user_pitch   = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->orbit_yaw += ORBIT_YAW_RATE * dt;
    if (s->orbit_yaw >  (float)(2.0 * M_PI)) s->orbit_yaw -= (float)(2.0 * M_PI);
}

/* §7.1 camera basis + per-pixel ray generation (T8). */

typedef struct {
    V3    origin;
    V3    fwd, right, up;
    float fov_t;
    float phys_aspect;
} Camera;

static Camera camera_basis(const Scene *s, int rows_eff)
{
    float yaw   = s->orbit_yaw   + s->user_yaw;
    float pitch = clampf(s->orbit_pitch + s->user_pitch,
                         -MANUAL_PITCH_MAX, MANUAL_PITCH_MAX);

    Camera c;
    c.origin = v3(s->cam_dist * cosf(pitch) * cosf(yaw),
                  s->cam_dist * sinf(pitch),
                  s->cam_dist * cosf(pitch) * sinf(yaw));
    c.fwd         = v3norm(v3sub(v3(0, 0, 0), c.origin));
    V3 wup        = v3(0, 1, 0);
    c.right       = v3norm(v3cross(c.fwd, wup));
    c.up          = v3cross(c.right, c.fwd);
    c.fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;
    return c;
}

static V3 pixel_ray(int col, int row, int cols, int rows_eff, const Camera *c)
{
    float u =  ((float)col + 0.5f) / (float)cols     * 2.0f - 1.0f;
    float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
    V3 sx = v3scale(u * c->fov_t,                  c->right);
    V3 sy = v3scale(v * c->fov_t * c->phys_aspect, c->up);
    return v3norm(v3add(c->fwd, v3add(sx, sy)));
}

/* §7.2 hit assembly + cell decoration. */

typedef struct {
    bool  hit;
    V3    p;
    V3    normal;
    float smooth;          /* smooth iteration count (for colour)   */
    float luminance;       /* final shaded value in [0, 1]          */
    int   march_steps;
} Hit;

/*
 * assemble_hit — given a TraceResult, fill in normal + smooth + final
 * luminance.  Concentrates every per-hit DE evaluation in one place
 * so the cost is auditable.
 */
static Hit assemble_hit(TraceResult tr, int max_iter, V3 light)
{
    Hit h = { tr.hit, tr.p, v3(0, 1, 0), 0.0f, 0.0f, tr.march_steps };
    if (!tr.hit) return h;

    h.normal = mandelbulb_normal(tr.p, max_iter);
    (void)mandelbulb_de(tr.p, max_iter, &h.smooth);
    h.luminance = shade(tr.p, h.normal, light, max_iter, tr.march_steps);
    return h;
}

/*
 * Cell — (glyph, colour pair, attribute) for one terminal cell.
 * pair < 0 is the miss sentinel (don't paint anything).
 */
typedef struct { char glyph; int pair; attr_t attr; } Cell;

static int to_slot(float x_01)
{
    int s = (int)(x_01 * LUMA_SLOT_FLT);
    if (s < 0)             s = 0;
    if (s >= LUMA_SLOTS)   s = LUMA_SLOTS - 1;
    return s;
}

/*
 * luma_attr — A_BOLD for brightest slots, A_DIM for darkest.  Disabled
 * for inverted themes because A_BOLD's "lighter" effect REDUCES
 * contrast against a white bg.
 */
static attr_t luma_attr(int slot, bool inverted)
{
    if (inverted)        return A_NORMAL;
    if (slot >= 6)       return A_BOLD;
    if (slot <= 1)       return A_DIM;
    return                      A_NORMAL;
}

/* shade_hit — per-pixel decoration (Hit → Cell).  Glyph from
 * luminance slot; colour pair from smooth-iter slot. */
static Cell shade_hit(const Hit *h, int max_iter, bool inverted)
{
    if (!h->hit) {
        return (Cell){ ' ', -1, 0 };       /* miss → don't paint */
    }
    int s_lum = to_slot(h->luminance);
    int s_clr = to_slot(h->smooth / (float)max_iter);
    return (Cell){
        .glyph = LUMA_GLYPHS[s_lum],
        .pair  = PAIR_RAMP_BASE + s_clr,
        .attr  = luma_attr(s_lum, inverted),
    };
}

/*
 * emit_cell — paint one cell with attron/attroff batched on (pair, attr)
 * change.  Halves attribute thrash on uniform regions.
 */
static void emit_cell(int row, int col, Cell c,
                      int *last_pair, attr_t *last_attr)
{
    if (c.pair < 0) return;       /* miss — leave the cell as-is */

    if (c.pair != *last_pair || c.attr != *last_attr) {
        if (*last_pair >= 0) attroff(COLOR_PAIR(*last_pair) | *last_attr);
        attron(COLOR_PAIR(c.pair) | c.attr);
        *last_pair = c.pair;
        *last_attr = c.attr;
    }
    mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/* prefill_canvas — paint the canvas region white before fractal
 * draws over it (inverted themes only).  Misses then naturally
 * show through as white. */
static void prefill_canvas(int y0, int rows_eff, int cols, bool inverted)
{
    if (!inverted) return;
    attron(COLOR_PAIR(PAIR_RAMP_BASE));
    for (int row = 0; row < rows_eff; row++)
        for (int col = 0; col < cols; col++)
            mvaddch(y0 + row, col, ' ');
    attroff(COLOR_PAIR(PAIR_RAMP_BASE));
}

/* §7.3 scene_render — full-frame raymarch, four-line body. */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 1) return;

    bool inverted = THEMES[s->current_theme].inverted;
    int  y0       = 1;            /* shift down 1 for the top HUD row */

    prefill_canvas(y0, rows_eff, s->cols, inverted);

    Camera cam   = camera_basis(s, rows_eff);
    V3     light = v3norm(v3(LIGHT_X, LIGHT_Y, LIGHT_Z));

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            V3          ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
            TraceResult tr  = sphere_trace(cam.origin, ray, s->iters);
            Hit         h   = assemble_hit(tr, s->iters, light);
            Cell        c   = shade_hit(&h, s->iters, inverted);
            emit_cell(y0 + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §8 screen — ncurses init / resize / HUD / present ───────────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free        (Screen *sc) { (void)sc; endwin(); }

static void screen_resize_curses(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 * Both rows always use A_BOLD so the HUD stays legible against any
 * fractal colour (including inverted-theme white).
 */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  %3d Hz  theme:%s  iters:%2d  dist:%4.2f  %s ",
             fps, sim_fps,
             THEMES[s->current_theme].name,
             s->iters, (double)s->cam_dist,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > sc->cols) slen = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - slen, "%s", status);
    mvprintw(0, 0, " MANDELBULB ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t/T:theme  i/I:iters  "
             "z/Z:zoom  arrows:orbit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_render(s);
    hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §9 app — main loop, signals, key handling ───────────────────────── */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup         (void)    { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset_cam(s); s->iters = ITERS_DEFAULT; break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'i': if (s->iters > ITERS_MIN) s->iters--; break;
    case 'I': if (s->iters < ITERS_MAX) s->iters++; break;

    case 'z':
        s->cam_dist -= CAM_DIST_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        break;
    case 'Z':
        s->cam_dist += CAM_DIST_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        break;

    case KEY_LEFT:  s->user_yaw   -= MANUAL_YAW_STEP;   break;
    case KEY_RIGHT: s->user_yaw   += MANUAL_YAW_STEP;   break;
    case KEY_UP:    s->user_pitch += MANUAL_PITCH_STEP; break;
    case KEY_DOWN:  s->user_pitch -= MANUAL_PITCH_STEP; break;

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

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

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

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
