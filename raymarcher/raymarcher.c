/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * raymarcher.c — a 3-D Phong-shaded ASCII sphere
 *
 * DEMO: One sphere lit by an orbiting light.  Each terminal cell
 *       fires one ray; sphere tracing finds where the ray hits the
 *       surface; Phong shading paints the resulting brightness as
 *       a glyph.  The simplest raymarcher in this folder — read
 *       this file BEFORE raymarcher_cube.c, kifs_fractal.c,
 *       mandelbulb.c, or metaballs.c, all of which extend this
 *       skeleton with a richer SDF.
 *
 * Study alongside: raymarcher/raymarcher_cube.c (next primitive,
 *       same skeleton).  The sphere is the textbook "hello world"
 *       of raymarching — the SDF is one line, the surface normal
 *       is closed-form, and you can verify trace convergence by
 *       hand with two numbers.
 *
 * Section map:
 *   §1  config      — every tunable named, no magic numbers later
 *   §2  clock       — monotonic ns timer + sleep
 *   §3  color       — themes + HUD pairs (CLAUDE.md HUD spec)
 *   §4  vec3        — 3-D math, value types
 *   §5  sphere SDF  — the canonical |p|−R distance function
 *   §6  trace       — sphere-tracing march loop (Hart 1996)
 *   §7  normal      — closed-form ∇f for a sphere (sphere-only)
 *   §8  shade       — Phong: ambient + diffuse + specular
 *   §9  cast_ray    — one pixel's full pipeline → Hit
 *   §10 canvas      — the Hit framebuffer (one Hit per pixel)
 *   §11 render      — fill the Hit array
 *   §12 draw        — production overlay (Hit → glyph + colour)
 *   §13 debug       — three educational overlays
 *   §14 scene       — Scene struct + light + tick
 *   §15 screen      — ncurses init + HUD + present
 *   §16 app         — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume the light orbit
 *   ]  [      light orbit faster / slower
 *   =  -      grow / shrink the sphere
 *   z / Z     zoom in / out (camera closer / farther)
 *   t / T     next / previous colour theme
 *               (CLASSIC / AMBER / MATRIX / NEON / ICE / COPPER)
 *   d / D     cycle debug overlay
 *               (NORMAL / NORMALS / DEPTH / STEPS)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher.c \
 *       -o sphere -lncurses -lm
 *   (The binary is named `sphere` because `raymarcher` would
 *    collide with the directory name.  Pick any name you like.)
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + ASCII diagram of the trace loop.
 *   • GUIDED TUTORIAL  eight short build-it-up answers — each one
 *                      adds the next piece of the renderer (scene
 *                      definition → trace → normal → shading →
 *                      camera → screen).
 *   • §1..§16          the actual code, each section short and focused.
 *
 * Ten-minute version: read just the GUIDED TUTORIAL.  The §-sections
 * then read like familiar territory.
 *
 * Math notation used in code:
 *      p   — a 3-D point
 *      ro  — ray origin
 *      rd  — ray direction (unit vector)
 *      t   — distance the ray has marched along rd
 *      N   — unit surface normal at the hit point
 *      L   — unit vector from hit point toward the light
 *      V   — unit vector from hit point toward the camera
 *      R   — light direction reflected about N
 *
 * Long names appear in struct fields and orchestrator functions
 * (`cast_ray`, `phong`, `canvas_render`).  Short math letters appear
 * inside formulas where the equation is one line away.
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot product, length, normalise)
 *   • for Tutorial 5 (the closed-form normal): the gradient of
 *     |p| with respect to p.  Single-variable calculus is enough.
 * No matrix theory, no triangle-mesh familiarity, no GPU experience.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : SPHERE TRACING (Hart 1996) of a SPHERE SDF.
 *                A signed distance function returns "how far am I
 *                from the surface" — positive outside, negative
 *                inside, zero on the surface.  For a sphere of
 *                radius R, the SDF is f(p) = |p| − R.  Sphere
 *                tracing turns ANY SDF into a renderer: walk along
 *                the ray by the distance the SDF returned, repeat
 *                until you're touching (|d| < ε) or have escaped
 *                (t > MAX_DIST).
 *
 *                Per pixel:
 *                  1. build a primary ray from camera through cell
 *                  2. sphere-trace until hit or miss
 *                  3. on hit:  N      = p / |p|   (closed form)
 *                              shade  = phong(p, N, ro, light)
 *                  4. (intensity, theme) → glyph + colour pair
 *
 *                Sphere is the simplest possible SDF.  Sibling files
 *                in this folder swap this SDF for a box, fractal,
 *                metaball field, etc., with the same skeleton.
 *
 * Data         : Stateless math (Vec3 + SDF + trace + shade) on the
 *                hot path.  Each pixel produces one `Hit` struct
 *                (hit / hit_point / normal / intensity / t / steps).
 *                The `Canvas` stores `Hit[w*h]` so render and draw
 *                are decoupled — production view + three debug
 *                overlays all read the SAME Hit array.
 *
 * Rendering    : One ray per terminal cell.  Glyph from the 13-char
 *                ramp " .,:;+*oxOX#@".  Colour from the active
 *                theme's 8-band luma palette.  Aspect correction in
 *                the ray direction so the sphere renders round on
 *                terminal cells that are roughly twice as tall as
 *                they are wide.
 *
 * Performance  : ~80 trace steps per ray × ~6 ops per step ≈ 500
 *                ops per hit pixel.  At 80×24 = 1920 pixels:
 *                roughly one million ops per frame.  Trivial at
 *                60 fps.  Scales linearly with terminal area.
 *
 * References   :
 *   • Hart, J. C. (1996) "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The paper that introduced this
 *     style of marching.
 *   • Phong, B. T. (1975) "Illumination for Computer Generated
 *     Pictures", *CACM* 18(6):311-317.  The lighting model in §8.
 *   • Quílez, I. — "Distance Functions" (article catalogue):
 *     https://iquilezles.org/articles/distfunctions/
 *     The whole library of SDF primitives this folder draws on.
 *     Sphere is item #1.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The whole renderer fits in one sentence: for every pixel, fire a
 * ray; ask the sphere "how far?"; walk that far; ask again; once
 * the answer is essentially zero, you're touching the surface.  The
 * remaining work — surface normal, lighting, character on screen —
 * is bookkeeping around that core loop.  Switch the sphere to a box
 * or a fractal and the loop doesn't change; only the SDF does.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Sphere tracing is geometric Newton's method.  In Newton's method
 * for f(x)=0 you step by f(x)/f'(x); each step shrinks the residual
 * roughly by half.  In sphere tracing, each step is f(p) along the
 * ray — exactly the largest safe step toward the surface.  No
 * derivative needed because the SDF IS the distance, and walking
 * forward by that much is guaranteed not to overshoot.  The whole
 * algorithm is one or two convergent iterations away from "you've
 * arrived".
 *
 * One frame, viewed in cross-section through the sphere's centre:
 *
 *      camera                                              behind
 *      ●─────────────────────────────────────────────►     sphere
 *           t₁ = 2.9    (step exactly 2.9 forward — safe)
 *           ●·····························●
 *                                          t₂ = 0.001 < ε  ⇒  HIT
 *                                          ◍───┐
 *                                          │ N │   N = p / |p|
 *                                          │ ↗ │   (closed form)
 *                                          └───┘
 *                                          on the surface
 *                                          ▲
 *                                       sphere of radius R = 1.1
 *
 * Sphere tracing converges in two steps for a head-on ray (one big
 * jump, one tiny correction); for a grazing ray it can take the
 * full RM_MAX_STEPS budget because each tangent-touching step
 * barely advances.  The DEBUG_STEPS overlay (§13) makes this
 * visible: silhouettes glow from the higher step count.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. animate    advance time (paused freezes it)
 *   2. position   light orbits in world space (Lissajous curve)
 *
 * Once per pixel:
 *   3. ray       build a primary ray from camera through the cell
 *   4. trace     sphere-march until d < ε (hit) or t > max (miss)
 *   5. normal    if hit: N = p / |p|  (no finite differences needed)
 *   6. shade     Phong: ambient + diffuse + specular → I ∈ [0,1]
 *
 * Once per cell:
 *   7. paint     intensity → glyph + theme colour, OR one of three
 *                debug overlays if d/D was pressed
 *
 * KEY FORMULAS
 * ────────────
 * Sphere SDF:
 *      f(p, R) = |p| − R       positive outside, negative inside
 *
 * Sphere normal (closed form, no finite differences):
 *      ∇f = ∇|p| − ∇R = p/|p| − 0 = p/|p|
 *      N  = ∇f / |∇f| = p/|p|  (already unit-length on the surface)
 *
 * Sphere tracing loop:
 *      t = 0
 *      repeat up to RM_MAX_STEPS:
 *          d = SDF(ro + t·rd)
 *          if d < HIT_EPS:    return t   (HIT)
 *          if t > MAX_DIST:   return -1  (MISS)
 *          t += d
 *
 * Phong intensity at hit point H, with surface normal N:
 *      L = normalise(light − H)
 *      V = normalise(camera − H)
 *      R = 2·(N·L)·N − L
 *      I = KA + KD·max(0, N·L) + KS·max(0, R·V)^SHIN     (clamp [0,1])
 *
 * Pixel → ray (cell at column col, row row, canvas w × h):
 *      u =  (col + 0.5) / w · 2 − 1
 *      v = −(row + 0.5) / h · 2 + 1
 *      rd = normalise(u·F, v·F·aspect, −1),  F = tan(FOV/2)
 *      aspect = h · CELL_ASPECT / w           (≈ 2 on a square terminal)
 *
 * Light orbit (Lissajous):
 *      α = time · light_spd
 *      L(α) = (R·cos α,  Y₀ + Y_amp·sin(rate · α),  Z₀)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Camera inside the sphere.  If sphere_radius grows past cam_z,
 *     the camera sits inside the sphere.  The first SDF returns
 *     negative; sphere tracing wants positive distances.  The
 *     result is visually broken (full-screen fill or black).
 *     SPHERE_R_MAX (3.0) and CAM_Z_MIN (2.0) make this possible
 *     only if the user actively grows AND zooms in maximally.
 *
 *   • RM_HIT_EPS = 0.002 is tighter than typical screen-space
 *     resolution.  Raising to 0.05 produces visible ringing at the
 *     silhouette where the SDF goes nearly tangent to the ray.
 *
 *   • The 1e-7 floor in v3norm prevents NaN when a degenerate ray
 *     direction occurs (e.g. (u, v) both exactly 0 — possible only
 *     at sub-pixel exactness, but the floor is cheap insurance).
 *
 *   • Pause freezes the light orbit but render still recasts every
 *     pixel.  Image is still, FPS counter is valid.
 *
 *   • Resize calls canvas_free + canvas_alloc; pixel buffer is
 *     sized to cols × rows so very large terminals incur quadratic
 *     ray cost.  At 200×60 that's 12 000 rays per frame ≈ 6 M ops,
 *     still under one frame's budget on a modern CPU.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At default settings (light_spd = 0.8) the highlight should
 *     orbit roughly once every 2π / 0.8 ≈ 7.8 seconds.  Press ]
 *     five times to multiply by 1.35⁵ ≈ 4.5× — orbit visibly speeds
 *     up by that factor.
 *
 *   • Press = until SPHERE_R_MAX clamps the radius at 3.0; sphere
 *     occupies ~75 % of the smaller dimension.  Press - until
 *     SPHERE_R_MIN (0.2): sphere shrinks to a small dot in the
 *     centre, marching converges in just 2-3 steps because the
 *     remaining distance |p|−R is huge for most rays.
 *
 *   • Press z / Z to zoom — the sphere geometry stays put, the
 *     camera distance changes.
 *
 *   • Press t to cycle themes.  Geometry identical, only the
 *     colour ramp changes.
 *
 *   • Press d to cycle debug overlays.  NORMALS shows the surface-
 *     normal azimuth as a colour band — confirms the sphere has
 *     full 3-D normals (continuous gradient).  DEPTH shows hit
 *     distance as brightness (closer = brighter).  STEPS shows the
 *     march iteration count — silhouette glows because grazing
 *     rays take more steps to converge.
 *
 *   • Toggle pause with space; confirm the highlight stops mid-orbit.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — build it up, one piece at a time ──────────────── *
 *
 * Each tutorial adds one piece to the renderer.  By the end you've
 * built the whole pipeline from scratch.
 *
 *
 * T1: How do we represent the sphere?
 * ───────────────────────────────────
 * Two choices:
 *
 *   (a) TRIANGLE MESH.  Approximate the sphere with a few hundred
 *       triangles.  Render by projecting each triangle, rasterising,
 *       depth-testing.  Standard GPU pipeline.  Many moving parts.
 *
 *   (b) SIGNED DISTANCE FUNCTION.  Represent the sphere as a function
 *       f(p) that returns "distance from point p to the sphere's
 *       surface".  ONE LINE OF MATH: f(p) = |p| − R.
 *
 * For ASCII rendering at modest resolution, (b) wins on every axis:
 *
 *      simpler                  one function vs vertex+index buffers
 *      smooth at any zoom       no triangle-edge artefacts
 *      same skeleton everywhere swap for box, torus, fractal, …
 *      collision detection      reuse the same f for physics
 *
 * The catch: rendering an SDF is different from rendering triangles.
 * That's where SPHERE TRACING comes in (T3).
 *
 *
 * T2: What does the sphere SDF actually mean?
 * ───────────────────────────────────────────
 * For a sphere centred at the origin with radius R:
 *
 *      f(p, R) = |p| − R           where |p| = √(p.x² + p.y² + p.z²)
 *
 *      f(p) > 0    p is OUTSIDE  →  distance to surface (true Euclidean)
 *      f(p) = 0    p is ON the surface
 *      f(p) < 0    p is INSIDE   →  negated distance to nearest surface
 *
 * Geometric derivation: the closest point on a sphere of radius R
 * to any point p is along the same radial line, at distance R from
 * the origin:
 *
 *      closest_on_sphere = R · (p / |p|)
 *
 *      distance_to_surface = | p − R·(p/|p|) |
 *                          = | p − R·p̂ |              (where p̂ = p/|p|)
 *                          = | (|p| − R) · p̂ |
 *                          = | |p| − R |              (since |p̂| = 1)
 *
 * The signed version drops the outer absolute value, giving the
 * negative-inside / positive-outside sign convention.
 *
 *
 * T3: How do we find where the ray hits the sphere?
 * ─────────────────────────────────────────────────
 * For a SPHERE specifically, you could solve it analytically.  The
 * ray r(t) = ro + t·rd hits the sphere where |r(t)|² = R²:
 *
 *      t² + 2(ro·rd)t + (ro·ro − R²) = 0
 *
 * Quadratic.  Take the smaller positive root.  Done — closed-form,
 * one trig-free step.
 *
 * But this only works because the sphere has a closed-form
 * intersection.  Boxes, fractals, metaballs, signed unions — none
 * of those have a clean quadratic.  We want ONE algorithm that
 * works for any SDF, including the sphere.
 *
 * SPHERE TRACING (Hart 1996) is that algorithm:
 *
 *      t = 0
 *      repeat up to MAX_STEPS:
 *          p = ro + t · rd
 *          d = SDF(p)
 *          if d < HIT_EPS:    HIT (return t)
 *          if t > MAX_DIST:   MISS (return -1)
 *          t += d                        ← step EXACTLY by SDF value
 *
 * The "step by exactly d" is the magic.  Because d is a TRUE
 * distance (the SDF is Lipschitz-1), no point closer than d can
 * exist along the ray — so stepping that far is guaranteed safe.
 *
 *
 * T4: Why does sphere tracing converge?
 * ─────────────────────────────────────
 * Worked example.  Camera at ro = (0, 0, 4), ray straight forward
 * rd = (0, 0, −1), sphere of radius R = 1.1 at the origin:
 *
 *      step 1: p = (0,0,4),    d = |p|−R = 4   − 1.1 = 2.9   → t = 2.9
 *      step 2: p = (0,0,1.1),  d = |p|−R = 1.1 − 1.1 = 0     → t = 2.9
 *                                                              HIT
 *
 * Two steps for a head-on ray.  The first step jumps to where
 * "the surface might be"; the second step lands on it.
 *
 * Now a GRAZING ray, ro = (0, 0, 4), rd ≈ (1, 0, −1) / √2 (pointing
 * past the sphere edge):
 *
 *      step 1: p = (0,0,4),         d = 4   − 1.1 ≈ 2.9   → t = 2.9
 *      step 2: p ≈ (2.05, 0, 1.95), d = 2.83 − 1.1 ≈ 1.73 → t = 4.63
 *      step 3: p ≈ (3.27, 0, 0.73), d = 3.35 − 1.1 ≈ 2.25 → t = 6.88
 *      …                            (escapes; eventually d > MAX_DIST)
 *
 * For grazing rays sphere tracing can take many steps — the SDF
 * keeps returning small distances as the ray scrapes along an
 * imaginary tangent line.  RM_MAX_STEPS (80) is the brake.  The
 * DEBUG_STEPS overlay highlights this visually: silhouettes glow.
 *
 *
 * T5: When the ray hits, which way does the surface face?
 * ───────────────────────────────────────────────────────
 * Phong shading needs the SURFACE NORMAL — the unit vector pointing
 * outward from the surface at the hit point.  For ANY SDF, the
 * gradient ∇f points OUTWARD from the surface (this is exactly
 * because f returns SIGNED distance: walking in the +∇f direction
 * is walking away from the surface).
 *
 *      N = ∇f / |∇f|
 *
 * For the SPHERE SDF f(p) = |p| − R, the gradient is just the
 * derivative of |p| with respect to p:
 *
 *      ∇f = ∇|p|        (the −R is a constant; its gradient is 0)
 *      ∇|p| = p / |p|   (one-line calculus: derivative of length)
 *
 * On the surface |p| = R, so |∇f| = |p/|p|| = 1 — already unit-
 * length.  The normal is simply:
 *
 *      N = p / |p|              (six characters of code)
 *
 * For other SDFs (cube, fractal, metaball) there's no closed-form
 * gradient, and we'd approximate it with finite differences.
 * That's a 4-tap or 6-tap function.  For the SPHERE specifically,
 * we save four SDF evaluations per pixel by using the closed form.
 * That's a real performance win.
 *
 *
 * T6: How do we turn a normal into a brightness?
 * ──────────────────────────────────────────────
 * Phong shading layers three components:
 *
 *      AMBIENT     KA          a constant base brightness — keeps
 *                              back-faces from going totally black.
 *      DIFFUSE     KD · (N·L)  Lambert's law: surfaces facing the
 *                              light get more energy per unit area.
 *                              N·L is just cos(angle to light).
 *      SPECULAR    KS · (R·V)^SHIN
 *                              The bright spot where the reflected
 *                              light points at the camera.  SHIN
 *                              tightens the spot.  Plastic ≈ 30,
 *                              polished metal ≈ 100.
 *
 * Reflection vector — geometric derivation:
 *      L decomposes into "parallel to N" (length N·L) and
 *      "perpendicular to N".  The reflected direction R keeps the
 *      perpendicular component and FLIPS the parallel one:
 *
 *          L = (N·L)·N + L_⊥
 *          R = (N·L)·N − L_⊥
 *            = (N·L)·N − (L − (N·L)·N)
 *            = 2·(N·L)·N − L
 *
 * Final intensity:
 *      I = KA + KD · max(0, N·L) + KS · max(0, R·V)^SHIN
 *
 * Both max(0, ·) clamps prevent back-facing or away-pointing
 * contributions from going negative.  This file uses KA=0.10,
 * KD=0.78, KS=0.55, SHIN=40 — values tuned so the sphere reads as
 * a "polished plastic ball" with a soft moving highlight.
 *
 * On a sphere, every direction of N exists somewhere on the surface
 * (the normal map is the unit sphere itself).  As the light orbits,
 * the highlight sweeps continuously across the surface — there are
 * no flat faces to "stick to".  That's the visual signature of a
 * sphere vs a polyhedron.
 *
 *
 * T7: How does a 3-D ray come out of a 2-D pixel?
 * ───────────────────────────────────────────────
 * Pinhole camera.  Camera at (0, 0, cam_z) looking down −Z.  For
 * each cell (col, row), the ray direction is built from normalised
 * device coordinates:
 *
 *      u =  (col + 0.5) / cw · 2 − 1     in [−1, +1], +0.5 = cell centre
 *      v = −(row + 0.5) / ch · 2 + 1     same; NEGATED so row 0 is top
 *      rd = normalise(u·F, v·F·aspect, −1)
 *      where F = tan(FOV/2)
 *
 * F is the half-tangent of the field of view — a "lens" parameter.
 * Smaller F → telephoto lens (narrow view, distant objects look
 * close).  Larger F → wide-angle lens.  We use F = 0.7 ≈ 70°.
 *
 * The aspect factor is essential.  Terminal cells are roughly
 * twice as tall as they are wide.  Without compensation, equal
 * vertical ray spacing means equal horizontal ray spacing — but
 * the cells aren't square!  The sphere would render as a vertical
 * ellipse.  Multiplying v by aspect = (rows · CELL_ASPECT / cols)
 * keeps the sphere round.
 *
 *      Without aspect fix:               With aspect fix:
 *           ┌───┐                              ┌─┐
 *           │   │   ← tall ellipse             │ │   ← round
 *           │ O │                              │O│
 *           │   │                              └─┘
 *           └───┘
 *
 *
 * T8: How does a brightness number become a character on screen?
 * ──────────────────────────────────────────────────────────────
 * cast_ray returns a `Hit` struct with everything an overlay might
 * need:
 *
 *      Hit { hit, hit_point, normal, intensity, t, steps }
 *
 * The PRODUCTION overlay reads .intensity, quantises to one of 13
 * characters in " .,:;+*oxOX#@", picks one of LUMI_N (8) theme
 * colour bands, and writes one terminal cell.  That's all
 * canvas_draw does.
 *
 * Three DEBUG overlays read DIFFERENT fields of the same Hit:
 *
 *      DEBUG_NORMALS    azimuth(N) → colour band, elevation(N) → glyph
 *                       Sphere normals span the full direction space,
 *                       so this overlay shows a smooth gradient.
 *      DEBUG_DEPTH      t normalised by cam_z range → glyph + colour
 *                       Closer hits are brighter.
 *      DEBUG_STEPS      steps / RM_MAX_STEPS → glyph + colour
 *                       Silhouette glows because grazing rays take
 *                       many steps before they decide to miss.
 *
 * That separation — Hit produced once, four overlays each reading
 * what they need — is why §11..§13 are organised the way they are.
 * Adding a fifth overlay would be ~30 lines of new code with no
 * change to the trace loop.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* End of textbook.  The rest of the file is the worked exercises. */

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

/* ── §1 config ───────────────────────────────────────────────────────── *
 *
 * Every tunable lives here.  No magic numbers anywhere else in the
 * file — if a literal carries meaning, it gets a name in §1.
 */

/* §1.1 frame rate. */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  FPS_UPDATE_MS = 500,
};

/* §1.2 canvas / cell mapping.
 *
 * The canvas is 1:1 with the terminal — each canvas pixel becomes
 * one terminal cell.  Aspect correction lives inside cast_ray so
 * the sphere renders round on cells that are about twice as tall
 * as they are wide.
 */
#define CELL_W 1
#define CELL_H 1
#define CELL_ASPECT 2.0f /* physical height ÷ width of one terminal cell */

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* §1.3 sphere tracing limits. */
#define RM_MAX_STEPS 80   /* hard cap per ray                    */
#define RM_HIT_EPS 0.002f /* "touching the surface" threshold    */
#define RM_MAX_DIST 20.0f /* ray-length budget before declaring miss */

/* §1.4 camera (zoom). */
#define CAM_Z_DEFAULT 4.0f
#define CAM_Z_MIN 2.0f /* keep camera outside SPHERE_R_MAX sphere */
#define CAM_Z_MAX 12.0f
#define CAM_ZOOM_STEP 0.30f
#define FOV_HALF_TAN 0.7f /* tan(FOV/2); 0.7 ≈ 70° wide          */

/* §1.5 sphere radius. */
#define SPHERE_R_DEFAULT 1.1f
#define SPHERE_R_STEP 1.15f
#define SPHERE_R_MIN 0.2f
#define SPHERE_R_MAX 3.0f

/* §1.6 light orbit speed (radians/sec). */
#define LIGHT_SPD_DEFAULT 0.8f
#define LIGHT_SPD_STEP 1.35f
#define LIGHT_SPD_MIN 0.02f
#define LIGHT_SPD_MAX 8.0f

/* §1.7 light orbit shape (Lissajous parameters). */
#define LIGHT_RADIUS_X 3.0f
#define LIGHT_BIAS_Y 1.5f
#define LIGHT_AMPLITUDE_Y 1.0f
#define LIGHT_RATE_Y 0.7f
#define LIGHT_HEIGHT_Z 2.5f

/* §1.8 Phong shading coefficients. */
#define KA 0.10f   /* ambient                                       */
#define KD 0.78f   /* diffuse                                       */
#define KS 0.55f   /* specular                                      */
#define SHIN 40.0f /* specular sharpness — bigger = tighter spot    */

/* §1.9 luma ramp + colour pair indices. */
enum {
  LUMI_N = 8,             /* 8 colour pairs hold the luma ramp   */
  PAIR_HUD = LUMI_N + 1,  /* yellow + bold — top status row      */
  PAIR_HINT = LUMI_N + 2, /* cyan   + bold — bottom hint row     */
};

static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define RAMP_LEN ((int)(sizeof LUMA_RAMP - 1)) /* = 13 */

/* §1.10 themes — six 8-band 256-colour ramps, one active at a time.
 *
 * theme_apply (§3) re-points the eight luma pairs to the chosen
 * theme.  Geometry, lighting, and ramp glyphs all stay identical.
 * Per the CLAUDE.md theme-brightness rule, every entry sits in the
 * bright half of the 256-cube so even the dimmest ramp slot stays
 * visible against a black terminal background.
 */
typedef struct {
  const char *display_name;
  short ramp_256[LUMI_N];
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    {"CLASSIC ", {235, 238, 241, 244, 247, 250, 253, 255}},
    {"AMBER   ", {130, 136, 166, 172, 178, 208, 214, 220}},
    {"MATRIX  ", {28, 34, 40, 46, 82, 118, 154, 190}},
    {"NEON    ", {53, 91, 129, 165, 201, 207, 213, 227}},
    {"ICE     ", {25, 31, 38, 45, 51, 87, 123, 159}},
    {"COPPER  ", {94, 130, 136, 166, 172, 208, 214, 220}},
};

/* §1.11 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,  /* full Phong + theme (production view) */
  DEBUG_NORMALS = 1, /* normal direction → colour band       */
  DEBUG_DEPTH = 2,   /* hit distance t → brightness          */
  DEBUG_STEPS = 3,   /* march iterations → brightness        */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "DEPTH  ",
    "STEPS  ",
};

/* §1.12 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── *
 *
 * CLOCK_MONOTONIC advances at one second per second with no NTP
 * jumps — exactly what a frame timer wants.
 */

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

/* ── §3 color — themes + HUD/hint pairs ──────────────────────────────── *
 *
 * Eight colour pairs (1..8) hold the active theme's luma ramp.
 * Two more pairs (PAIR_HUD, PAIR_HINT) are reserved for the HUD
 * and key-hint strips per the CLAUDE.md HUD spec — yellow + bold
 * for status, cyan + bold for hints.
 */

static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  if (COLORS >= 256) {
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), theme->ramp_256[i], COLOR_BLACK);
  } else {
    /* 8-colour fallback: themes have no effect; lumi_attr fakes
     * brightness via A_DIM / A_BOLD on COLOR_WHITE. */
    for (int i = 0; i < LUMI_N; i++)
      init_pair((short)(i + 1), COLOR_WHITE, COLOR_BLACK);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }

  theme_apply(0);
}

static attr_t lumi_attr(int l) {
  if (l < 0)
    l = 0;
  if (l > LUMI_N - 1)
    l = LUMI_N - 1;
  attr_t a = COLOR_PAIR(l + 1);
  if (COLORS < 256) {
    if (l < 3)
      a |= A_DIM;
    else if (l >= 6)
      a |= A_BOLD;
  }
  return a;
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── *
 *
 * All operations return Vec3 by value.  At -O2 every call here
 * inlines to register operations — no heap, no aliasing concerns.
 */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3norm(Vec3 a) {
  float L = v3len(a);
  return (L > 1e-7f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

/* ── §5 sphere SDF — the canonical |p|−R distance function ───────────── *
 *
 * Tutorial T2 derived this.  One line of math:
 *
 *      f(p, R) = |p| − R
 *
 * Lipschitz-1 (true Euclidean distance) so it's safe for sphere
 * tracing.  Positive outside, zero on the surface, negative
 * (negated radial distance) inside.
 */
static float sdf_sphere(Vec3 p, float radius) { return v3len(p) - radius; }

/* ── §6 sphere trace — Hart 1996 march loop ──────────────────────────── *
 *
 * Tutorial T3 explained the algorithm.  Returns the ray-parameter
 * t at the hit, or -1 on miss.  Optionally writes the step count
 * via out_steps (used by the STEPS debug overlay; pass NULL to
 * ignore).
 */
static float sphere_trace(Vec3 origin, Vec3 dir, float radius, int *out_steps) {
  float t = 0.0f;
  int step;
  for (step = 0; step < RM_MAX_STEPS; step++) {
    Vec3 p = v3add(origin, v3mul(dir, t));
    float d = sdf_sphere(p, radius);
    if (d < RM_HIT_EPS) {
      if (out_steps)
        *out_steps = step + 1;
      return t;
    }
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  if (out_steps)
    *out_steps = step;
  return -1.0f;
}

/* ── §7 sphere normal — closed form (no finite differences needed) ───── *
 *
 * Tutorial T5 derived this from calculus.  For the sphere SDF the
 * gradient ∇f = ∇(|p| − R) = p/|p|, which is already unit length on
 * the surface where |p| = R.  So:
 *
 *      N = p / |p|
 *
 * For other SDFs (cube, fractal, metaballs) we'd estimate the
 * gradient via 4-tap or 6-tap finite differences.  For the SPHERE
 * specifically, four SDF evaluations per pixel are saved by using
 * this one-line closed form.
 */
static Vec3 sphere_normal(Vec3 p) { return v3norm(p); }

/* ── §8 Phong shade — ambient + diffuse + specular ───────────────────── *
 *
 * Tutorial T6 derived the formula.  Implementation walks each
 * component in order and clamps the final intensity to [0, 1].
 */
static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light) {
  Vec3 L = v3norm(v3sub(light, hit));
  Vec3 V = v3norm(v3sub(cam, hit));
  float ndl = fmaxf(0.0f, v3dot(N, L));
  Vec3 R = v3sub(v3mul(N, 2.0f * ndl), L);
  float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

  float I = KA + KD * ndl + KS * spec;
  if (I < 0.0f)
    I = 0.0f;
  if (I > 1.0f)
    I = 1.0f;
  return I;
}

/* ── §9 cast_ray — one pixel's full pipeline → Hit ───────────────────── *
 *
 * Builds a ray from camera through cell, sphere-traces, computes
 * the normal and Phong intensity if the ray hit, and returns a
 * `Hit` carrying every field any overlay might need.
 *
 * The Hit struct is the seam between rendering math and overlay
 * code: production view + three debug overlays all read the SAME
 * Hit array and never re-trace.
 */

typedef struct {
  bool hit;
  Vec3 hit_point;
  Vec3 normal;
  float intensity;      /* Phong result in [0,1]                    */
  float trace_distance; /* ray parameter at hit (DEBUG_DEPTH input) */
  int step_count;       /* march iterations    (DEBUG_STEPS input)  */
} Hit;

static Hit cast_ray(int col, int row, int canvas_w, int canvas_h,
                    float sphere_radius, Vec3 light, float cam_z) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0};

  /* NDC for the cell centre (T7). */
  float u = ((float)col + 0.5f) / (float)canvas_w * 2.0f - 1.0f;
  float v = -((float)row + 0.5f) / (float)canvas_h * 2.0f + 1.0f;

  /* Aspect: terminal cells are ~2× taller than wide.  Scale v
   * so equal physical distance per unit in both axes. */
  float phys_aspect = ((float)canvas_h * CELL_ASPECT) / (float)canvas_w;

  Vec3 ro = v3(0.0f, 0.0f, cam_z);
  Vec3 rd = v3norm(v3(u * FOV_HALF_TAN, v * FOV_HALF_TAN * phys_aspect, -1.0f));

  int steps = 0;
  float t = sphere_trace(ro, rd, sphere_radius, &steps);
  h.step_count = steps;

  if (t < 0.0f)
    return h;

  h.hit = true;
  h.trace_distance = t;
  h.hit_point = v3add(ro, v3mul(rd, t));
  h.normal = sphere_normal(h.hit_point);
  h.intensity = phong(h.hit_point, h.normal, ro, light);
  return h;
}

/* ── §10 canvas — the Hit framebuffer ────────────────────────────────── *
 *
 * One Hit per canvas pixel.  Sized at startup and on resize; never
 * reallocated mid-frame.  Render writes; draw / debug-overlay
 * functions read.
 */

typedef struct {
  int w, h;
  Hit *hits;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->w = canvas_w_from_cols(cols);
  c->h = canvas_h_from_rows(rows);
  c->hits = calloc((size_t)(c->w * c->h), sizeof(Hit));
}

static void canvas_free(Canvas *c) {
  free(c->hits);
  c->hits = NULL;
  c->w = c->h = 0;
}

/* ── §11 render — fill the Hit array for one frame ───────────────────── *
 *
 * Pure math.  Knows nothing about glyphs, colour, or terminals.
 * cam_z is a parameter (not a constant) so the user can zoom.
 */
static void canvas_render(Canvas *c, float sphere_radius, Vec3 light,
                          float cam_z) {
  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      c->hits[py * c->w + px] =
          cast_ray(px, py, c->w, c->h, sphere_radius, light, cam_z);
    }
  }
}

/* ── §12 draw — production overlay (Hit → glyph + theme colour) ──────── *
 *
 * The default view: intensity drives glyph and colour.  Three
 * helpers handle the bookkeeping (ramp index, attribute slot,
 * terminal-centre offset) so canvas_draw itself reads as a tight
 * loop.
 */

static char intensity_to_glyph(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= RAMP_LEN)
    idx = RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static attr_t intensity_to_attr(float intensity) {
  int idx = (int)(intensity * (float)(RAMP_LEN - 1) + 0.5f);
  int slot = (idx * LUMI_N) / RAMP_LEN;
  return lumi_attr(slot);
}

static void canvas_offsets(const Canvas *c, int term_cols, int term_rows,
                           int *out_off_x, int *out_off_y) {
  int total_w = c->w * CELL_W;
  int total_h = c->h * CELL_H;
  *out_off_x = (term_cols - total_w) / 2;
  *out_off_y = (term_rows - total_h) / 2;
}

static void emit_block(int tx0, int ty0, char glyph, attr_t attr, int term_cols,
                       int term_rows) {
  attron(attr);
  for (int by = 0; by < CELL_H; by++) {
    for (int bx = 0; bx < CELL_W; bx++) {
      int tx = tx0 + bx;
      int ty = ty0 + by;
      if (tx < 0 || tx >= term_cols)
        continue;
      if (ty < 0 || ty >= term_rows)
        continue;
      mvaddch(ty, tx, (chtype)(unsigned char)glyph);
    }
  }
  attroff(attr);
}

static void canvas_draw(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      char glyph = intensity_to_glyph(h->intensity);
      attr_t attr = intensity_to_attr(h->intensity);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §13 debug overlays — see the rendering signals raw ──────────────── *
 *
 * Three educational visualisations.  Each isolates ONE piece of
 * intermediate state from the Hit struct and paints it directly.
 *
 *   NORMALS  azimuth(N) → colour band, elevation(N) → glyph.
 *            Sphere normals span the full direction sphere, so this
 *            shows a smooth gradient.  Reveals raw geometry without
 *            theme tint.
 *
 *   DEPTH    hit distance t → glyph + colour.  Closer hits render
 *            brighter.  Reveals what the trace actually measured.
 *
 *   STEPS    march step count → glyph + colour.  Grazing silhouette
 *            rays take many steps to converge — this overlay glows
 *            at the silhouette.
 */

static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      Vec3 N = h->normal;
      float azimuth = atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
      float y_lit = N.y * 0.5f + 0.5f;
      if (y_lit < 0.0f)
        y_lit = 0.0f;
      if (y_lit > 1.0f)
        y_lit = 1.0f;

      char glyph = intensity_to_glyph(y_lit);
      attr_t attr = intensity_to_attr(azimuth); /* hue from azimuth */
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_depth(const Canvas *c, int term_cols, int term_rows,
                              float cam_z) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  /* Closer hits get higher depth_n.  Sphere extends ±SPHERE_R_MAX
   * around the origin in any direction. */
  float t_min = cam_z - SPHERE_R_MAX;
  float t_max = cam_z + SPHERE_R_MAX;
  if (t_min < 0.0f)
    t_min = 0.0f;

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float depth_n = (t_max - h->trace_distance) / (t_max - t_min);
      if (depth_n < 0.0f)
        depth_n = 0.0f;
      if (depth_n > 1.0f)
        depth_n = 1.0f;

      char glyph = intensity_to_glyph(depth_n);
      attr_t attr = intensity_to_attr(depth_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

static void canvas_draw_steps(const Canvas *c, int term_cols, int term_rows) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      const Hit *h = &c->hits[py * c->w + px];
      if (!h->hit)
        continue;

      float steps_n = (float)h->step_count / (float)RM_MAX_STEPS;
      if (steps_n > 1.0f)
        steps_n = 1.0f;

      char glyph = intensity_to_glyph(steps_n);
      attr_t attr = intensity_to_attr(steps_n);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, glyph, attr, term_cols, term_rows);
    }
  }
}

/* ── §14 scene — Scene struct + light + tick ─────────────────────────── */

typedef struct {
  Canvas canvas;
  float time;
  float light_spd;
  float sphere_r;
  float cam_z;     /* camera z; smaller = zoomed-in */
  int theme_index; /* index into THEMES[] (t/T keys) */
  DebugMode debug_mode;
  bool paused;
} Scene;

/* Lissajous orbit:
 *      α = time · light_spd
 *      L(α) = (LIGHT_RADIUS_X · cos α,
 *              LIGHT_BIAS_Y + LIGHT_AMPLITUDE_Y · sin(LIGHT_RATE_Y · α),
 *              LIGHT_HEIGHT_Z)                                        */
static Vec3 scene_light(const Scene *s) {
  float t = s->time * s->light_spd;
  return v3(LIGHT_RADIUS_X * cosf(t),
            LIGHT_BIAS_Y + LIGHT_AMPLITUDE_Y * sinf(LIGHT_RATE_Y * t),
            LIGHT_HEIGHT_Z);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  canvas_alloc(&s->canvas, cols, rows);
  s->time = 0.0f;
  s->light_spd = LIGHT_SPD_DEFAULT;
  s->sphere_r = SPHERE_R_DEFAULT;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme_index = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt_sec) {
  if (!s->paused)
    s->time += dt_sec;
}

static void scene_render(Scene *s) {
  canvas_render(&s->canvas, s->sphere_r, scene_light(s), s->cam_z);
}

/* dispatch to the active overlay */
static void scene_draw_active(const Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(&s->canvas, cols, rows);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(&s->canvas, cols, rows);
    break;
  case DEBUG_DEPTH:
    canvas_draw_depth(&s->canvas, cols, rows, s->cam_z);
    break;
  case DEBUG_STEPS:
    canvas_draw_steps(&s->canvas, cols, rows);
    break;
  default:
    canvas_draw(&s->canvas, cols, rows);
    break;
  }
}

/* ── §15 screen — ncurses init / HUD / present ───────────────────────── */

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

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void screen_draw(Screen *s, const Scene *sc, double fps) {
  erase();
  scene_draw_active(sc, s->cols, s->rows);

  char status[200];
  snprintf(status, sizeof status,
           " %5.1f fps  spd:%.2f  r:%.2f  zoom:%.2f  theme:%s  "
           "debug:%s  [%dx%d]  %s ",
           fps, sc->light_spd, sc->sphere_r, sc->cam_z,
           THEMES[sc->theme_index].display_name,
           DEBUG_MODE_NAMES[sc->debug_mode], sc->canvas.w, sc->canvas.h,
           sc->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " RAYMARCHER ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  ]/[:light-speed  +/-:size  z/Z:zoom  "
           "t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §16 app — main loop, signals, key handling ──────────────────────── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
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

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case ']':
    s->light_spd *= LIGHT_SPD_STEP;
    if (s->light_spd > LIGHT_SPD_MAX)
      s->light_spd = LIGHT_SPD_MAX;
    break;
  case '[':
    s->light_spd /= LIGHT_SPD_STEP;
    if (s->light_spd < LIGHT_SPD_MIN)
      s->light_spd = LIGHT_SPD_MIN;
    break;

  case '=':
  case '+':
    s->sphere_r *= SPHERE_R_STEP;
    if (s->sphere_r > SPHERE_R_MAX)
      s->sphere_r = SPHERE_R_MAX;
    break;
  case '-':
    s->sphere_r /= SPHERE_R_STEP;
    if (s->sphere_r < SPHERE_R_MIN)
      s->sphere_r = SPHERE_R_MIN;
    break;

  case 'z':
    s->cam_z -= CAM_ZOOM_STEP;
    if (s->cam_z < CAM_Z_MIN)
      s->cam_z = CAM_Z_MIN;
    break;
  case 'Z':
    s->cam_z += CAM_ZOOM_STEP;
    if (s->cam_z > CAM_Z_MAX)
      s->cam_z = CAM_Z_MAX;
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
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

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    scene_render(&app->scene);

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
