/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * metaballs.c — six smoothly-blending spheres traced as one blob
 *
 * DEMO: Six spheres orbit on Lissajous paths.  A polynomial smooth-min
 *       blends their distance fields so the spheres appear to MELT
 *       into each other when close — the classic metaball look.  Soft
 *       shadows from an orbiting light, curvature-based colour bands,
 *       optional 2×2 super-sample anti-aliasing, four colour themes,
 *       four debug overlays.
 *
 * Study alongside: raymarcher/raymarcher.c (single sphere SDF, no
 *       blending — read it first).  This file is what happens when
 *       you put many sphere SDFs together with a smooth blend
 *       operator instead of a hard min.
 *
 * Section map:
 *   §1   config         — every tunable named, no magic numbers later
 *   §2   clock          — monotonic timer + sleep
 *   §3   color          — theme palette + HUD/hint pairs
 *   §4   vec3           — 3-D math, value types, clampf
 *   §5   sphere SDF     — the |p − c| − r primitive
 *   §6   smooth-min     — polynomial smin that makes the blob
 *   §7   scene SDF      — fold smin over all balls
 *   §8   normal         — tetrahedron 4-tap gradient
 *   §9   curvature      — Laplacian of SDF (mean curvature signal)
 *   §10  sphere trace   — Hart 1996 march by DE
 *   §11  soft shadow    — Quílez running-min penumbra
 *   §12  Phong shade    — ambient + diffuse + specular + shadow
 *   §13  cast_ray       — one-pixel orchestrator → Hit
 *   §14  ball orbits + light position
 *   §15  scene state    — Scene struct + tick + SDF view
 *   §16  camera         — camera_for_canvas + pixel_ray (with AA offsets)
 *   §17  canvas         — alloc + render orchestrator (with AA loop)
 *   §18  cell decoration + emit + canvas_draw
 *   §19  debug overlays — see the rendering signals raw
 *   §20  screen         — ncurses init + HUD + present
 *   §21  app            — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the orbits
 *   j / k      decrease / increase smoothing (smaller k = sharper joins)
 *   z / Z      zoom in / out (camera closer / farther)
 *   a / A      toggle 2×2 anti-aliasing
 *   s / S      toggle soft shadows
 *   c / C      next colour theme
 *   d / D      cycle debug overlay (NORMAL / NORMALS / CURV / SHADOW)
 *   + / -      animation faster / slower
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/metaballs.c \
 *       -o metaballs -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is structured as a textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of the blob shape.
 *   • GUIDED TUTORIAL  ten short answers walking from "how do you
 *                      blend two SDFs?" through curvature, soft
 *                      shadows, anti-aliasing, and the Hit-driven
 *                      multi-overlay architecture.
 *   • §1..§21          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      p              — a 3-D point (the SDF input)
 *      cᵢ, rᵢ          — centre and radius of ball i
 *      d              — distance estimate
 *      k (k_blend)    — smooth-min blend strength (small = sharp join)
 *      DE(p)          — distance estimator at p
 *      N              — surface normal
 *      H              — mean curvature
 *      L              — light direction (unit vector)
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, length, normalise)
 *   • read raymarcher.c first if sphere tracing is unfamiliar
 *   • the Laplacian operator ∇²f (used in the curvature tutorial T5)
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : METABALLS rendered as a SMOOTH-MIN of N SPHERE SDFs.
 *                Each ball is f_i(p) = |p − c_i| − r_i (a sphere SDF
 *                centred at c_i with radius r_i).  Combine N balls
 *                with the polynomial smooth-min:
 *                    d = f_0
 *                    for i in 1..N−1: d = smin(d, f_i, k)
 *                where smin (Quílez) is a continuous-derivative
 *                replacement for min that softens the join between
 *                shapes.  The result is one connected SDF whose
 *                surface is the metaball blob.
 *
 *                Render this SDF with sphere tracing.  Add Phong
 *                lighting + soft shadows + curvature-based colouring
 *                + 2×2 SSAA → the demo.
 *
 * Data         : Stateless math (Vec3 + sphere_sdf + smin + chain) on
 *                the hot path.  Per-pixel result is `Hit`
 *                (hit / p / normal / shade / shadow / curvature).
 *                The `Canvas` stores 4 separate float arrays —
 *                shades / curvs / shadows / normals — so each debug
 *                overlay can read just the field it needs.
 *
 * Rendering    : 2×2 block rendering: each canvas pixel maps to a
 *                CELL_W × CELL_H block of terminal cells.  Quarters
 *                the per-frame ray count vs full-resolution.  2×2
 *                SSAA optionally casts 4 rays per canvas pixel and
 *                averages — coverage attenuation softens silhouettes.
 *
 * Performance  : Per AA sample: 1 trace (~10-30 SDF calls) + 4-tap
 *                normal + 6-tap curvature + 16-step soft shadow.
 *                ~50-100 SDF calls per hit pixel × AA_SAMPLES.  At
 *                40×12 canvas (= 80×24 terminal) and AA on, ~100K
 *                SDF calls per frame.  Each SDF is N_BALLS sphere
 *                evals = 6 vec3 ops.  Holds 24 fps comfortably.
 *
 * References   :
 *   • Blinn, J. F. (1982) — "A Generalization of Algebraic Surface
 *     Drawing", *ACM TOG* 1(3):235-256.  The original "blobby
 *     surfaces" paper that introduced the metaball idea.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The march loop.
 *   • Quílez, I. — "Distance Functions" / "smooth minimum"
 *     https://iquilezles.org/articles/distfunctions/
 *     https://iquilezles.org/articles/smin/
 *     The polynomial smin (§6) and the running-min soft shadow (§11).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Six independent spheres exist in 3-D space.  At any point p, ask
 * each one "how far am I from your surface?" and you get six
 * distances.  Naïvely combining with min() gives the union of the
 * six spheres — but the join between two spheres is a SHARP CIRCLE
 * (where the two distance fields are equal).  Replace min() with
 * SMOOTH-MIN, a polynomial that's identical to min when the two
 * distances differ but smoothly interpolates when they're close.
 * The visual result: spheres now MELT into each other near the
 * join, producing necks, droplets, drips — the classic blobby
 * metaball look.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of mercury droplets on a flat surface.  When two droplets
 * are far apart, each is a perfect sphere.  Push them close
 * together: surface tension pulls them into contact and forms a
 * smooth NECK between the two — they don't intersect with a sharp
 * circle, they merge.  smin is the math version of surface tension:
 * it bulges the iso-surface OUTWARD wherever two distance fields
 * are within a "blend distance" k of each other.
 *
 * The blob's silhouette in cross-section, two balls with smin:
 *
 *      with naïve min:                 with smin:
 *           ╭───╮  ╭───╮                  ╭───────╮
 *          (     ╳     )                 (         )
 *           ╰───╯  ╰───╯                  ╰───────╯
 *           ▲   ▲ ▲    ▲                    ▲   ▲
 *         ball  intersection             ball  smooth
 *               circle                          neck
 *               (sharp)                         (continuous derivative)
 *
 * As k → 0, smin → min (back to sharp joins).  As k grows, the
 * neck thickens until eventually two distant balls visibly bulge
 * toward each other even before they touch.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per frame:
 *   1. animate     advance time × speed (paused freezes it)
 *   2. orbit       update each ball's position via Lissajous formula
 *                  update light position via its own slower orbit
 *
 * Once per canvas pixel × per AA sample:
 *   3. ray         build a primary ray from camera through sub-pixel
 *   4. trace       sphere-march; hit when scene_sdf < HIT_EPS
 *   5. on hit:     4-tap tetrahedral normal
 *                  6-tap Laplacian → curvature
 *                  soft shadow ray to the light (running-min penumbra)
 *                  Phong shading combining all three
 *   6. accumulate  sum into per-pixel (shade, normal, curv, shadow)
 *
 * Once per canvas pixel:
 *   7. average     divide sums by hit_count
 *                  shade *= coverage = hit_count / n_samples (AA edge)
 *
 * Once per cell:
 *   8. paint       shade → glyph; curvature → colour band; emit
 *                  CELL_W × CELL_H block
 *
 * KEY FORMULAS
 * ────────────
 * Sphere SDF:
 *      f(p, c, r) = |p − c| − r
 *
 * Polynomial smooth-min (Quílez):
 *      h    = max(k − |a − b|, 0) / k
 *      smin = min(a, b) − h² · k / 4
 *
 * Scene SDF (fold smin over balls):
 *      d = f₀(p)
 *      for i in 1..N−1: d = smin(d, fᵢ(p), k)
 *
 * Tetrahedron normal (Quílez 4-tap, no shared-axis bias):
 *      f0 = SDF(p + ε·( 1, −1, −1))
 *      f1 = SDF(p + ε·(−1,  1, −1))
 *      f2 = SDF(p + ε·(−1, −1,  1))
 *      f3 = SDF(p + ε·( 1,  1,  1))
 *      N  = normalise( f0−f1−f2+f3, −f0+f1−f2+f3, −f0−f1+f2+f3 )
 *
 * Mean curvature from Laplacian of SDF:
 *      ∇²f(p) ≈ ( Σ axis-aligned neighbours of f − 6·f(p) ) / ε²
 *      H ≈ ½ ∇²f      (true at the surface for unit-gradient SDFs)
 *
 * Soft shadow (running-min penumbra):
 *      result = 1
 *      for each step toward the light:
 *          h = SDF(p)
 *          if h < ε·½: return 0
 *          result = min(result, K · h / t)
 *      return clamp(result, 0, 1)
 *
 * Phong with shadow:
 *      I = KA + shadow · (KD · max(0, N·L) + KS · max(0, R·V)^SHIN)
 *      (shadow multiplies only the direct terms — ambient never goes black)
 *
 * AA coverage softening:
 *      coverage = hit_count / n_samples
 *      pixel_shade = mean(hit_shades) · coverage
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • k → 0: smin formula DIVIDES BY k.  K_MIN (0.05) clamps.  At
 *     k = 0 you'd get a divide-by-zero NaN; below K_MIN the joins
 *     are visibly sharp anyway.
 *
 *   • Very large k: balls visibly bulge toward each other even when
 *     spatially well-separated.  The blob loses recognisable
 *     individual-ball structure.  K_MAX (4.0) is the practical
 *     upper bound where the demo still reads as "blobby spheres".
 *
 *   • Camera inside the blob: at CAM_Z_MIN (2.5) the camera is
 *     close enough that maximally-extended ball orbits can pass IN
 *     FRONT of the camera plane.  Visible as flickers when a ball
 *     swings past.  Acceptable trade-off for the dramatic close-up.
 *
 *   • SSAA off (`a` toggle): edges become blocky 2×2 stair-stepping.
 *     Useful on slow terminals.  AA on quadruples per-pixel cost
 *     but cleans up the silhouette.
 *
 *   • Soft shadow ray for highly-overlapping balls: the running-min
 *     can saturate to 0 inside the blob.  AMBIENT (KA = 0.08)
 *     ensures pixels deep in shadow still read.
 *
 *   • Curvature estimate is noisy where the SDF gradient is small
 *     (smin transition zones).  CURV_SCALE (0.25) tames the worst
 *     of it; visually it just shifts which colour band a transition
 *     pixel falls into.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press k repeatedly: balls progressively merge into one
 *     amoeba.  Watch the necks form and grow.  Press j to reverse:
 *     balls separate back toward distinct spheres.
 *
 *   • At very small k (after pressing j many times) the blob's
 *     surface develops sharp ridges where balls meet — that's the
 *     min() limit case.
 *
 *   • Press d to cycle debug overlays.  CURV shows where the colour-
 *     band signal comes from (necks have one curvature, caps another).
 *     SHADOW shows where the soft-shadow ray contributes.  NORMALS
 *     shows raw geometry without lighting.
 *
 *   • Toggle aa with `a`.  Off: blocky edges.  On: soft edges where
 *     coverage attenuates partial-hit pixels.
 *
 *   • Toggle shadow with `s`.  Off: lighting becomes uniform Phong
 *     (no contact shadows).  On: blob-on-blob darkening visible.
 *
 *   • Press c to cycle themes.  Geometry stays put; only the
 *     curvature → colour ramp changes.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: How do you combine multiple SDFs into one scene?
 * ────────────────────────────────────────────────────
 * The simplest combination operator: take the MINIMUM of each shape's
 * SDF.  At any point p:
 *
 *      union_sdf(p) = min( sphere1_sdf(p), sphere2_sdf(p), … )
 *
 * The min works because for any p outside ALL shapes, the closest
 * shape determines the distance to the union.  Inside any shape,
 * the negative SDF of THAT shape is the most-negative value (the
 * point is "deepest inside" that one), so min picks it correctly.
 *
 * Verify with two unit spheres at (−0.5, 0, 0) and (+0.5, 0, 0):
 *      p = (0, 0, 0):  s1 = 0.5 − 1 = −0.5,  s2 = 0.5 − 1 = −0.5
 *                      union_sdf = min(−0.5, −0.5) = −0.5
 *                      → point is 0.5 inside the union ✓
 *      p = (1.5, 0, 0): s1 = 2.0 − 1 = 1.0,  s2 = 1.0 − 1 = 0.0
 *                      union_sdf = min(1.0, 0.0) = 0.0
 *                      → point is on the surface of sphere 2 ✓
 *
 * The PROBLEM: min() has a discontinuous derivative at the join
 * (where the two SDFs are equal).  Visually this is a SHARP CIRCLE
 * where the spheres meet — not the smooth metaball look we want.
 *
 *
 * T2: Smooth-min — the polynomial blend
 * ─────────────────────────────────────
 * Replace min with a function that:
 *   - equals min when |a − b| > k        (no smoothing far from join)
 *   - smoothly interpolates when |a − b| < k
 *
 * Quílez's polynomial smin:
 *
 *      h    = max(k − |a − b|, 0) / k          ∈ [0, 1]
 *      smin = min(a, b) − h² · k / 4
 *
 * Verify the limiting cases:
 *      |a − b| = 0       (the join):  h = 1 → bump = k/4 → max smoothing
 *      |a − b| ≥ k       (far apart): h = 0 → bump = 0   → smin = min
 *
 * The bump h²·k/4 is what bulges the iso-surface OUTWARD — it makes
 * the SDF report a SMALLER distance than min would, in the gap
 * between the two shapes.  Pixels in that gap become "inside the
 * blob" even though they're outside both individual shapes.  That's
 * the metaball neck.
 *
 *
 * T3: Building the scene SDF — fold smin over N balls
 * ───────────────────────────────────────────────────
 * For N metaballs we apply smin as a left-fold (reduction):
 *
 *      d = sphere_sdf(p, ball_0)
 *      for i in 1..N−1:
 *          d = smin(d, sphere_sdf(p, ball_i), k)
 *      return d
 *
 * The order of the fold matters in principle (smin is associative
 * only in the limit k → 0), but for our k values and well-separated
 * balls the visual difference is invisible.
 *
 * The resulting `d` is the metaball blob's distance estimate.  Feed
 * it to sphere tracing (T4) and you have a renderer.
 *
 *
 * T4: Sphere tracing recap (see raymarcher.c for full derivation)
 * ───────────────────────────────────────────────────────────────
 *      t = 0
 *      repeat MAX_STEPS:
 *          d = scene_sdf(ro + t·rd)
 *          if d < HIT_EPS:   return t   (HIT)
 *          if t > MAX_DIST:  return -1  (MISS)
 *          t += d
 *
 * The smooth-min SDF stays Lipschitz (the derivative magnitude is
 * bounded), so sphere tracing converges.  No tricks needed beyond
 * what raymarcher.c already does.
 *
 *
 * T5: Curvature from the Laplacian of the SDF
 * ───────────────────────────────────────────
 * For an SDF with unit gradient (true distance functions are
 * Lipschitz-1), the LAPLACIAN ∇²f at a surface point equals
 * approximately TWICE the mean curvature H:
 *
 *      ∇²f ≈ 2H   at points where |∇f| = 1
 *
 * Mean curvature measures how "bendy" a surface is at p:
 *      H > 0:  convex (sphere caps, ball tops)
 *      H = 0:  flat or saddle
 *      H < 0:  concave (necks between merged balls)
 *
 * Approximate the Laplacian with a 6-tap stencil:
 *
 *      ∇²f ≈ ( f(p+εx) + f(p−εx) +
 *              f(p+εy) + f(p−εy) +
 *              f(p+εz) + f(p−εz) − 6·f(p) ) / ε²
 *
 * In the metaball demo we use this to COLOUR the surface.  Map
 * curvature to a colour band: necks (high curvature) get one band,
 * sphere caps (lower curvature) get another.  The result: the join
 * regions visually pop out from the rest of the blob.
 *
 *
 * T6: Soft shadow — Quílez running-min penumbra
 * ─────────────────────────────────────────────
 * From a hit point, march a ray TOWARD the light source.  At each
 * step, evaluate the SDF.  If we ever hit something, the light is
 * blocked.  But we want SOFT shadows — penumbra, not pure on/off.
 *
 *      result = 1
 *      for each step at distance t from origin:
 *          h = SDF(p_at_t)
 *          if h < ε·½:   return 0   (definitely blocked)
 *          result = min(result, K · h / t)
 *          t += max(h, ε)
 *
 * The KEY trick: track the RUNNING MINIMUM of K·h/t.  If the ray
 * comes CLOSE to a surface (small h at distance t), K·h/t drops
 * below 1, dimming the light proportionally.  If the ray actually
 * hits, return 0 (full shadow).  If the ray clears the surface but
 * passed close, the running min preserves that "near miss" as
 * partial shadow.
 *
 * The result is a smooth penumbra without any extra rays.  K
 * controls how SHARP the shadow edge is — higher K = harder shadow.
 *
 *
 * T7: Phong with shadow — three components stacked
 * ────────────────────────────────────────────────
 *      I = KA + shadow · (KD · max(0, N·L) + KS · max(0, R·V)^SHIN)
 *
 * The `shadow` factor multiplies only the direct components
 * (diffuse + specular), NOT the ambient.  Why: pure black pixels
 * are visually unreadable; a faint ambient floor preserves the
 * shape outline even in deep shadow.
 *
 * For metaballs, the orbiting light traces specular highlights
 * across the blob's surface as it moves — and the shadows that one
 * lobe casts on another visibly shift each frame.
 *
 *
 * T8: Anti-aliasing — coverage-based edge softening
 * ─────────────────────────────────────────────────
 * Without AA, each canvas pixel = one ray.  At silhouette pixels,
 * the single ray either hits (full shade) or misses (background) —
 * no in-between.  Result: blocky edges.
 *
 * Super-sample anti-aliasing (SSAA) casts MULTIPLE rays per pixel
 * at sub-pixel offsets, then averages:
 *
 *      for each pixel:
 *          for sample in 0..N_SAMPLES−1:
 *              cast ray at (px + AA_OFFSETS[sample])
 *              if hit: accumulate
 *          coverage = hits / N_SAMPLES
 *          pixel_shade = (sum_shade / hits) · coverage
 *
 * The COVERAGE multiplication is the key.  Where the ray-set
 * straddles a silhouette, only some samples hit (e.g. 2/4).  The
 * average of those 2 hit shades is full-bright, but multiplying
 * by 0.5 coverage produces a half-bright pixel — partial coverage
 * → edge softening.
 *
 * 4 samples (2×2 grid) is the standard quality/cost trade-off.
 * AA quadruples per-pixel ray cost; toggleable for slow terminals.
 *
 *
 * T9: Half-resolution rendering — quartering ray cost
 * ───────────────────────────────────────────────────
 * Each canvas pixel is painted as a CELL_W × CELL_H = 2×2 block
 * of terminal cells.  At an 80×24 terminal:
 *      canvas = (80/2) × (24/2) = 40 × 12 = 480 pixels
 *      vs full-res 80×24 = 1920 pixels
 *
 * That's 1/4 the rays per frame.  Combined with AA at 4 samples
 * per canvas pixel:
 *      total = 480 × 4 = 1920 rays
 *
 * Same total ray count as full-res no-AA, but spread 4-per-pixel
 * across the smaller canvas.  AA's coverage softening + half-res's
 * lower silhouette resolution combine to give edges that look
 * smoother than full-res no-AA at the same total cost.
 *
 *
 * T10: The Hit struct — one ray's complete result
 * ───────────────────────────────────────────────
 *      Hit { hit, p, normal, shade, shadow, curvature }
 *
 * Each AA sample produces one Hit.  The renderer accumulates 1-4
 * Hits per canvas pixel into per-pixel sums (then averages).  The
 * Canvas stores 4 separate float arrays for SHADE, NORMAL,
 * CURVATURE, SHADOW — that lets each debug overlay read just the
 * field it needs:
 *
 *      DEBUG_NORMAL      shade + curvature → glyph + colour
 *      DEBUG_NORMALS     normal direction → colour band, N.y → glyph
 *      DEBUG_CURVATURE   curvature → glyph + colour
 *      DEBUG_SHADOW      shadow → glyph + colour
 *
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
  TARGET_FPS = 24,
  FPS_UPDATE_MS = 500,
  HUD_ROWS = 2,    /* row 0 status + last row hint */
  DT_CAP_MS = 200, /* cap dt to avoid orbit teleport */

  N_BALLS = 6,
  N_THEMES = 4,
  N_CURV_BANDS = 8,
};

/* §1.2 colour-pair IDs. */
#define PAIR_HUD 1
#define PAIR_HINT 2
#define PAIR_THEME_BASE 3 /* +0..+(N_THEMES * N_CURV_BANDS − 1) */
#define PAIR_FOR(theme, band)                                                  \
  (PAIR_THEME_BASE + (theme) * N_CURV_BANDS + (band))

/* §1.3 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* §1.4 canvas — half-resolution block rendering (T9).
 *
 * Each canvas pixel becomes a CELL_W × CELL_H block of terminal cells.
 * 2×2 quarters the per-frame ray count.
 */
#define CELL_W 2
#define CELL_H 2
#define CELL_ASPECT 2.0f /* terminal cell h / w (physical) */

/* §1.5 sphere-trace tunables. */
#define RM_MAX_STEPS 64
#define RM_HIT_EPS 0.005f
#define RM_MAX_DIST 12.0f

/* §1.6 camera (zoom). */
#define CAM_Z_DEFAULT 5.0f
#define CAM_Z_MIN 2.5f /* keep camera outside ball orbit */
#define CAM_Z_MAX 14.0f
#define CAM_ZOOM_STEP 0.40f
#define FOV_HALF_TAN 0.55f /* tan(FOV/2) */

/* §1.6.1 anti-aliasing (T8).
 *
 * 4 samples (2×2 grid) at sub-pixel offsets.  Cost: 4× per-pixel
 * rays.  Toggle via `a` / `A` for slow terminals.
 */
#define AA_SAMPLES 4

static const float AA_OFFSETS[AA_SAMPLES][2] = {
    {0.25f, 0.25f},
    {0.75f, 0.25f},
    {0.25f, 0.75f},
    {0.75f, 0.75f},
};

/* §1.7 smooth-min blend k.
 *      K_MIN  → balls stay separate (hard min)
 *      K_MAX  → balls visibly bulge toward each other
 *      K_MIN must be > 0 to avoid division by zero (T2).
 */
#define K_DEFAULT 0.8f
#define K_MIN 0.05f
#define K_MAX 4.0f
#define K_STEP 1.35f

/* §1.8 animation speed (multiplier on dt that drives the orbit). */
#define SPD_DEFAULT 0.35f
#define SPD_MIN 0.02f
#define SPD_MAX 3.0f
#define SPD_STEP 1.35f

/* §1.9 Phong shading + bold threshold. */
#define KA 0.08f
#define KD 0.75f
#define KS 0.45f
#define SHIN 32.0f
#define BOLD_SHADE_THRESHOLD 0.72f

/* §1.10 soft shadow (T6).
 *      SHADOW_K       hardness (bigger = sharper shadow edge)
 *      SHADOW_BIAS    offset along normal at march start (anti-self-shadow)
 *      SHADOW_NEAR    t-min for the shadow march
 */
#define SHADOW_STEPS 16
#define SHADOW_K 8.0f
#define SHADOW_BIAS 0.01f
#define SHADOW_NEAR 0.02f

/* §1.11 normal + curvature estimation. */
#define NORMAL_EPS 0.004f
#define CURV_EPS 0.06f
#define CURV_SCALE 0.25f /* maps raw Laplacian into [0, 1] */

/* §1.12 light orbit. */
#define LIGHT_RATE 0.6f
#define LIGHT_RATE_Y (0.6f * 0.45f)
#define LIGHT_RADIUS_X 4.0f
#define LIGHT_RADIUS_Y 2.0f
#define LIGHT_BIAS_Y 2.5f
#define LIGHT_HEIGHT_Z 3.5f

/* §1.13 ball orbits — Lissajous frequencies + phase offsets + radii.
 *      x = ORBIT_RX · sin(freq_x · t + phase_x)
 *      y = ORBIT_RY · sin(freq_y · t + phase_y)
 *      z = ORBIT_RZ · cos(freq_z · t)
 */
typedef struct {
  float freq_x, freq_y, freq_z;
  float phase_x, phase_y;
  float radius;
} BallOrbit;

#define ORBIT_RX 1.35f
#define ORBIT_RY 0.75f
#define ORBIT_RZ 0.40f

static const BallOrbit ORBITS[N_BALLS] = {
    /*  fx     fy     fz     phx     phy     r     */
    {1.0f, 2.0f, 1.5f, 0.000f, 0.785f, 0.60f},
    {2.0f, 1.0f, 3.0f, 1.047f, 0.000f, 0.55f},
    {1.5f, 3.0f, 1.0f, 2.094f, 1.571f, 0.50f},
    {3.0f, 1.0f, 2.0f, 3.141f, 0.524f, 0.58f},
    {2.5f, 1.5f, 1.0f, 4.189f, 3.927f, 0.45f},
    {1.0f, 2.5f, 2.0f, 5.236f, 0.262f, 0.52f},
};

/* §1.14 themes — N_THEMES × N_CURV_BANDS.  Per CLAUDE.md every entry
 * sits in the bright half of the 256-cube. */
typedef struct {
  const char *name;
  short bands[N_CURV_BANDS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    {"CLASSIC ", {27, 33, 38, 44, 130, 166, 202, 214}},
    {"OCEAN   ", {25, 26, 27, 31, 38, 45, 51, 159}},
    {"EMBER   ", {88, 124, 160, 196, 202, 208, 214, 228}},
    {"NEON    ", {201, 165, 129, 93, 57, 82, 118, 155}},
};

/* §1.15 luminance glyph ramp — dark → bright. */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define LUMA_RAMP_LEN ((int)(sizeof LUMA_RAMP - 1))

/* §1.16 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,    /* full lit blob (production view)       */
  DEBUG_NORMALS = 1,   /* surface normal direction → colour     */
  DEBUG_CURVATURE = 2, /* curvature → colour, no lighting       */
  DEBUG_SHADOW = 3,    /* shadow factor → glyph density         */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "NORMALS",
    "CURV   ",
    "SHADOW ",
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                         .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&req, NULL);
}

/* ── §3 color — theme palette + HUD/hint pairs ───────────────────────── */

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
    for (int t = 0; t < N_THEMES; t++)
      for (int b = 0; b < N_CURV_BANDS; b++)
        init_pair((short)PAIR_FOR(t, b), THEMES[t].bands[b], -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    for (int t = 0; t < N_THEMES; t++)
      for (int b = 0; b < N_CURV_BANDS; b++)
        init_pair((short)PAIR_FOR(t, b),
                  (b < 3)   ? COLOR_BLUE
                  : (b < 5) ? COLOR_CYAN
                  : (b < 7) ? COLOR_YELLOW
                            : COLOR_WHITE,
                  -1);
  }
}

/* ── §4 vec3 — value-type 3-D math + clampf ──────────────────────────── */

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
  return (L > 1e-12f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/* ── §5 sphere SDF — the |p − c| − r primitive ───────────────────────── *
 *
 * The simplest possible SDF.  Tutorial T2 in raymarcher.c derives
 * this from "distance to nearest point on a sphere".  Every other
 * SDF in this file is built by combining sphere SDFs with smin (§6).
 */
static inline float sphere_sdf(Vec3 p, Vec3 c, float r) {
  return v3len(v3sub(p, c)) - r;
}

/* ── §6 smooth-min — the polynomial smin ─────────────────────────────── *
 *
 * Tutorial T2 derived this:
 *      h    = max(k − |a − b|, 0) / k
 *      smin = min(a, b) − h² · k / 4
 *
 * NOTE: k must be > 0 (we clamp at K_MIN = 0.05 in §1).  At k = 0
 * the formula divides by zero.
 */
static float smin(float a, float b, float k) {
  float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
  return fminf(a, b) - h * h * k * 0.25f;
}

/* ── §7 scene SDF — fold smin over balls + SceneSDF view ─────────────── *
 *
 * SceneSDF is a read-only "view" of the scene's metaballs, passed to
 * every SDF helper by const-pointer.  Lets §8..§13 take ONE pointer
 * instead of four arguments.
 */
typedef struct {
  const Vec3 *centres;
  const float *radii;
  int n;
  float k_blend;
} SceneSDF;

/* T3: fold smin over all balls.  Order doesn't materially affect the
 * result for our k values and well-separated balls. */
static float scene_sdf(Vec3 p, const SceneSDF *s) {
  float d = sphere_sdf(p, s->centres[0], s->radii[0]);
  for (int i = 1; i < s->n; i++)
    d = smin(d, sphere_sdf(p, s->centres[i], s->radii[i]), s->k_blend);
  return d;
}

/* ── §8 normal — tetrahedron 4-tap gradient ──────────────────────────── *
 *
 * Quílez tetrahedron formula: 4 SDF samples at corners of a regular
 * tetrahedron in {±ε}³, combined with sign vectors.  4 evals, no
 * shared-axis pair → cleaner normal at sharp features than 6-tap
 * central differences.
 */
static Vec3 estimate_normal(Vec3 p, const SceneSDF *s) {
  float e = NORMAL_EPS;
  float f0 = scene_sdf(v3(p.x + e, p.y - e, p.z - e), s);
  float f1 = scene_sdf(v3(p.x - e, p.y + e, p.z - e), s);
  float f2 = scene_sdf(v3(p.x - e, p.y - e, p.z + e), s);
  float f3 = scene_sdf(v3(p.x + e, p.y + e, p.z + e), s);
  return v3norm(v3(f0 - f1 - f2 + f3, -f0 + f1 - f2 + f3, -f0 - f1 + f2 + f3));
}

/* ── §9 curvature — Laplacian-of-SDF → mean curvature ────────────────── *
 *
 * Tutorial T5 derived ∇²f ≈ 2H at the surface.  6-sample stencil:
 *      ∇²f ≈ ( Σ axis-aligned neighbours of f − 6·f(p) ) / ε²
 *
 * Returns the raw Laplacian; the caller scales by CURV_SCALE and
 * clamps to [0, 1] for colour-band quantisation.
 */
static float estimate_curvature(Vec3 p, const SceneSDF *s) {
  float e = CURV_EPS;
  float c0 = scene_sdf(p, s);
  float lap = scene_sdf(v3(p.x + e, p.y, p.z), s) +
              scene_sdf(v3(p.x - e, p.y, p.z), s) +
              scene_sdf(v3(p.x, p.y + e, p.z), s) +
              scene_sdf(v3(p.x, p.y - e, p.z), s) +
              scene_sdf(v3(p.x, p.y, p.z + e), s) +
              scene_sdf(v3(p.x, p.y, p.z - e), s) - 6.0f * c0;
  return lap / (e * e);
}

/* ── §10 sphere trace — Hart 1996 march by DE ────────────────────────── *
 *
 * Tutorial T4 explained the algorithm.  Returns t at hit, or -1 on
 * miss.  Same loop as raymarcher.c, just with scene_sdf instead of
 * sphere_sdf.
 */
static float sphere_trace(Vec3 origin, Vec3 dir, const SceneSDF *s) {
  float t = 0.0f;
  for (int i = 0; i < RM_MAX_STEPS; i++) {
    Vec3 p = v3add(origin, v3mul(dir, t));
    float d = scene_sdf(p, s);
    if (d < RM_HIT_EPS)
      return t;
    if (t > RM_MAX_DIST)
      break;
    t += d;
  }
  return -1.0f;
}

/* ── §11 soft shadow — Quílez running-min penumbra ───────────────────── *
 *
 * Tutorial T6 derived this.  Returns 1 if the path is clear, 0 if
 * fully blocked, soft factor in (0, 1) for partial occlusion.
 */
static float soft_shadow(Vec3 origin, Vec3 to_light_dir, float t_min,
                         float t_max, const SceneSDF *s) {
  float res = 1.0f;
  float t = t_min;
  for (int i = 0; i < SHADOW_STEPS && t < t_max; i++) {
    float h = scene_sdf(v3add(origin, v3mul(to_light_dir, t)), s);
    if (h < RM_HIT_EPS * 0.5f)
      return 0.0f;
    res = fminf(res, SHADOW_K * h / t);
    t += fmaxf(h, RM_HIT_EPS);
  }
  return clampf(res, 0.0f, 1.0f);
}

/* ── §12 Phong shade — ambient + diffuse + specular + shadow ─────────── *
 *
 * Tutorial T7 wrote the formula.  shadow multiplies only direct
 * components; ambient never goes black.
 */
static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light, float shadow) {
  Vec3 L_dir = v3norm(v3sub(light, hit));
  Vec3 V_dir = v3norm(v3sub(cam, hit));
  float ndl = fmaxf(0.0f, v3dot(N, L_dir));
  Vec3 R_dir = v3sub(v3mul(N, 2.0f * ndl), L_dir);
  float spec = powf(fmaxf(0.0f, v3dot(R_dir, V_dir)), SHIN);
  return clampf(KA + shadow * (KD * ndl + KS * spec), 0.0f, 1.0f);
}

/* ── §13 cast_ray — one-pixel orchestrator → Hit ─────────────────────── *
 *
 * Tutorial T10 listed Hit's fields.  Each AA sample produces one Hit;
 * canvas_render accumulates and averages.
 */

typedef struct {
  bool hit;
  Vec3 p;
  Vec3 normal;
  float shade;     /* Phong intensity in [0, 1]            */
  float shadow;    /* soft shadow factor (DEBUG_SHADOW)    */
  float curvature; /* normalised curvature in [0, 1]       */
} Hit;

static Hit cast_ray(Vec3 origin, Vec3 dir, const SceneSDF *s, Vec3 light,
                    bool soft_shadows) {
  Hit h = {false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f, 0.0f};

  float t = sphere_trace(origin, dir, s);
  if (t < 0.0f)
    return h;

  h.hit = true;
  h.p = v3add(origin, v3mul(dir, t));
  h.normal = estimate_normal(h.p, s);

  /* Shadow factor — soft penumbra ray, or 1.0 if shadows disabled. */
  h.shadow = 1.0f;
  if (soft_shadows) {
    Vec3 L_dir = v3norm(v3sub(light, h.p));
    Vec3 shad_o = v3add(h.p, v3mul(h.normal, SHADOW_BIAS));
    float to_lite = v3len(v3sub(light, h.p));
    h.shadow = soft_shadow(shad_o, L_dir, SHADOW_NEAR, to_lite, s);
  }

  h.shade = phong(h.p, h.normal, origin, light, h.shadow);
  float raw_c = estimate_curvature(h.p, s);
  h.curvature = clampf(raw_c * CURV_SCALE, 0.0f, 1.0f);
  return h;
}

/* ── §14 ball orbits + light position ────────────────────────────────── */

static Vec3 ball_position_at(int i, float t) {
  const BallOrbit *o = &ORBITS[i];
  return v3(ORBIT_RX * sinf(o->freq_x * t + o->phase_x),
            ORBIT_RY * sinf(o->freq_y * t + o->phase_y),
            ORBIT_RZ * cosf(o->freq_z * t));
}

static Vec3 light_position_at(float t) {
  return v3(cosf(t * LIGHT_RATE) * LIGHT_RADIUS_X,
            sinf(t * LIGHT_RATE_Y) * LIGHT_RADIUS_Y + LIGHT_BIAS_Y,
            LIGHT_HEIGHT_Z);
}

/* ── §15 scene state — Scene struct + tick + SDF view ────────────────── */

typedef struct {
  Vec3 centres[N_BALLS];
  float radii[N_BALLS];
  float time; /* scene time (seconds × speed) */
  float k_blend;
  float speed;
  float cam_z; /* camera z; smaller = zoomed-in */
  int theme;
  DebugMode debug_mode;
  bool paused;
  bool soft_shadows;
  bool aa_enabled;
} Scene;

static void scene_update_balls(Scene *s) {
  for (int i = 0; i < N_BALLS; i++) {
    s->centres[i] = ball_position_at(i, s->time);
    s->radii[i] = ORBITS[i].radius;
  }
}

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->k_blend = K_DEFAULT;
  s->speed = SPD_DEFAULT;
  s->cam_z = CAM_Z_DEFAULT;
  s->theme = 0;
  s->debug_mode = DEBUG_NORMAL;
  s->paused = false;
  s->soft_shadows = true;
  s->aa_enabled = true;
  scene_update_balls(s);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt * s->speed;
  scene_update_balls(s);
}

static SceneSDF scene_sdf_view(const Scene *s) {
  return (SceneSDF){
      .centres = s->centres,
      .radii = s->radii,
      .n = N_BALLS,
      .k_blend = s->k_blend,
  };
}

/* ── §16 camera — camera_for_canvas + pixel_ray (with AA offsets) ────── */

typedef struct {
  Vec3 origin;
  float fov_t;
  float phys_aspect;
} Camera;

static Camera camera_for_canvas(int cw, int ch, float cam_z) {
  return (Camera){
      .origin = v3(0.0f, 0.0f, cam_z),
      .fov_t = FOV_HALF_TAN,
      .phys_aspect = ((float)ch * (float)CELL_H * CELL_ASPECT) /
                     ((float)cw * (float)CELL_W),
  };
}

/*
 * pixel_ray — direction from camera through canvas pixel (px, py)
 * at sub-pixel offset (ox, oy) ∈ [0, 1].  AA passes call with the
 * 2×2 grid; non-AA passes call with (0.5, 0.5).
 */
static Vec3 pixel_ray(int px, int py, float ox, float oy, int cw, int ch,
                      const Camera *c) {
  float u = ((float)px + ox) / (float)cw * 2.0f - 1.0f;
  float v = -((float)py + oy) / (float)ch * 2.0f + 1.0f;
  return v3norm(v3(u * c->fov_t, v * c->fov_t * c->phys_aspect, -1.0f));
}

/* ── §17 canvas — alloc + render orchestrator (with AA loop) ─────────── *
 *
 * Four parallel float arrays, one per Hit field.  Render writes;
 * canvas_draw + debug overlays read.
 */

typedef struct {
  int w, h;
  float *shades;  /* w*h; < 0 = miss */
  float *curvs;   /* w*h */
  float *shadows; /* w*h */
  Vec3 *normals;  /* w*h */
} Canvas;

static void canvas_alloc(Canvas *c, int term_cols, int term_rows) {
  int draw_rows = term_rows - HUD_ROWS;
  if (draw_rows < 1)
    draw_rows = 1;

  c->w = term_cols / CELL_W;
  c->h = draw_rows / CELL_H;
  if (c->w < 1)
    c->w = 1;
  if (c->h < 1)
    c->h = 1;

  size_t n = (size_t)c->w * (size_t)c->h;
  c->shades = malloc(n * sizeof(float));
  c->curvs = malloc(n * sizeof(float));
  c->shadows = malloc(n * sizeof(float));
  c->normals = malloc(n * sizeof(Vec3));
}

static void canvas_free(Canvas *c) {
  free(c->shades);
  c->shades = NULL;
  free(c->curvs);
  c->curvs = NULL;
  free(c->shadows);
  c->shadows = NULL;
  free(c->normals);
  c->normals = NULL;
  c->w = c->h = 0;
}

/*
 * canvas_render — fill all four arrays.  Tutorial T8 walked through
 * the AA accumulation; T9 explained the half-resolution gain.
 */
static void canvas_render(Canvas *c, const Scene *s) {
  Camera cam = camera_for_canvas(c->w, c->h, s->cam_z);
  Vec3 light = light_position_at(s->time);
  SceneSDF view = scene_sdf_view(s);

  int n_samples = s->aa_enabled ? AA_SAMPLES : 1;

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {

      float sum_shade = 0.0f;
      float sum_curv = 0.0f;
      float sum_shadow = 0.0f;
      Vec3 sum_normal = v3(0, 0, 0);
      int hit_count = 0;

      for (int sample = 0; sample < n_samples; sample++) {
        float ox, oy;
        if (n_samples == 1) {
          ox = 0.5f;
          oy = 0.5f;
        } else {
          ox = AA_OFFSETS[sample][0];
          oy = AA_OFFSETS[sample][1];
        }

        Vec3 ray = pixel_ray(px, py, ox, oy, c->w, c->h, &cam);
        Hit h = cast_ray(cam.origin, ray, &view, light, s->soft_shadows);
        if (h.hit) {
          sum_shade += h.shade;
          sum_curv += h.curvature;
          sum_shadow += h.shadow;
          sum_normal = v3add(sum_normal, h.normal);
          hit_count++;
        }
      }

      int idx = py * c->w + px;
      if (hit_count == 0) {
        c->shades[idx] = -1.0f;
        c->curvs[idx] = 0.0f;
        c->shadows[idx] = 0.0f;
        c->normals[idx] = v3(0, 0, 0);
      } else {
        float coverage = (float)hit_count / (float)n_samples;
        c->shades[idx] = (sum_shade / (float)hit_count) * coverage;
        c->curvs[idx] = sum_curv / (float)hit_count;
        c->shadows[idx] = sum_shadow / (float)hit_count;
        c->normals[idx] = v3norm(sum_normal);
      }
    }
  }
}

/* ── §18 cell decoration + emit + canvas_draw ────────────────────────── */

typedef struct {
  char glyph;
  int pair;
  attr_t attr;
} Cell;

static char shade_to_glyph(float shade) {
  int idx = (int)(shade * (float)(LUMA_RAMP_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= LUMA_RAMP_LEN)
    idx = LUMA_RAMP_LEN - 1;
  return LUMA_RAMP[idx];
}

static int curvature_to_band(float curv) {
  int b = (int)(curv * (float)(N_CURV_BANDS - 1) + 0.5f);
  if (b < 0)
    b = 0;
  if (b >= N_CURV_BANDS)
    b = N_CURV_BANDS - 1;
  return b;
}

static Cell decorate(float shade, float curv, int theme) {
  if (shade < 0.0f)
    return (Cell){' ', -1, 0};
  int band = curvature_to_band(curv);
  return (Cell){
      .glyph = shade_to_glyph(shade),
      .pair = PAIR_FOR(theme, band),
      .attr = (shade > BOLD_SHADE_THRESHOLD) ? A_BOLD : 0,
  };
}

/* emit_block — paint a CELL_W × CELL_H block.  Skips silently for
 * miss cells (pair < 0). */
static void emit_block(int tx0, int ty0, Cell c, int term_cols, int term_rows) {
  if (c.pair < 0)
    return;

  attr_t a = COLOR_PAIR(c.pair) | c.attr;
  attron(a);
  for (int by = 0; by < CELL_H; by++) {
    int ty = ty0 + by;
    if (ty < 0 || ty >= term_rows)
      continue;
    for (int bx = 0; bx < CELL_W; bx++) {
      int tx = tx0 + bx;
      if (tx < 0 || tx >= term_cols)
        continue;
      mvaddch(ty, tx, (chtype)(unsigned char)c.glyph);
    }
  }
  attroff(a);
}

/* canvas_offsets — top-left screen position centring the canvas. */
static void canvas_offsets(const Canvas *c, int term_cols, int term_rows,
                           int *out_off_x, int *out_off_y) {
  int off_x = (term_cols - c->w * CELL_W) / 2;
  int off_y = 1 + (term_rows - HUD_ROWS - c->h * CELL_H) / 2;
  if (off_x < 0)
    off_x = 0;
  if (off_y < 1)
    off_y = 1;
  *out_off_x = off_x;
  *out_off_y = off_y;
}

/* canvas_draw — production view (shade + curvature → glyph + band). */
static void canvas_draw(const Canvas *c, int term_cols, int term_rows,
                        int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      Cell cell = decorate(c->shades[idx], c->curvs[idx], theme);
      int tx0 = off_x + px * CELL_W;
      int ty0 = off_y + py * CELL_H;
      emit_block(tx0, ty0, cell, term_cols, term_rows);
    }
  }
}

/* ── §19 debug overlays — see the rendering signals raw ──────────────── *
 *
 * Three educational visualisations.  Each isolates ONE Canvas array.
 */

/* NORMALS: hue from azimuth atan2(N.x, N.z); glyph from N.y. */
static void canvas_draw_normals(const Canvas *c, int term_cols, int term_rows,
                                int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      Vec3 N = c->normals[idx];
      float azimuth = atan2f(N.x, N.z) / (2.0f * (float)M_PI) + 0.5f;
      float y_lit = clampf(N.y * 0.5f + 0.5f, 0.0f, 1.0f);

      int band = curvature_to_band(azimuth);
      char glyph = shade_to_glyph(y_lit);
      Cell cell = (Cell){glyph, PAIR_FOR(theme, band), 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

/* CURVATURE: glyph and colour both from curvature value. */
static void canvas_draw_curvature(const Canvas *c, int term_cols, int term_rows,
                                  int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      float curv = c->curvs[idx];
      int band = curvature_to_band(curv);
      char glyph = shade_to_glyph(curv);
      Cell cell =
          (Cell){glyph, PAIR_FOR(theme, band), (curv > 0.7f) ? A_BOLD : 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

/* SHADOW: glyph + colour both from shadow factor (1 = lit, 0 = dark). */
static void canvas_draw_shadow(const Canvas *c, int term_cols, int term_rows,
                               int theme) {
  int off_x, off_y;
  canvas_offsets(c, term_cols, term_rows, &off_x, &off_y);

  for (int py = 0; py < c->h; py++) {
    for (int px = 0; px < c->w; px++) {
      int idx = py * c->w + px;
      if (c->shades[idx] < 0.0f)
        continue;

      float shadow = c->shadows[idx];
      int band = curvature_to_band(shadow);
      char glyph = shade_to_glyph(shadow);
      Cell cell =
          (Cell){glyph, PAIR_FOR(theme, band), (shadow > 0.7f) ? A_BOLD : 0};

      emit_block(off_x + px * CELL_W, off_y + py * CELL_H, cell, term_cols,
                 term_rows);
    }
  }
}

static void canvas_draw_active(const Canvas *c, int term_cols, int term_rows,
                               const Scene *s) {
  switch (s->debug_mode) {
  case DEBUG_NORMAL:
    canvas_draw(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_NORMALS:
    canvas_draw_normals(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_CURVATURE:
    canvas_draw_curvature(c, term_cols, term_rows, s->theme);
    break;
  case DEBUG_SHADOW:
    canvas_draw_shadow(c, term_cols, term_rows, s->theme);
    break;
  default:
    canvas_draw(c, term_cols, term_rows, s->theme);
    break;
  }
}

/* ── §20 screen — ncurses init / HUD / present ───────────────────────── */

typedef struct {
  int cols, rows;
  Canvas canvas;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
  canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

static void screen_free(Screen *sc) {
  canvas_free(&sc->canvas);
  endwin();
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
  canvas_free(&sc->canvas);
  canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint */
static void hud_draw(const Screen *sc, const Scene *s, double fps) {
  char status[200];
  snprintf(status, sizeof status,
           " %5.1f fps  k=%4.2f  spd=%4.2f  zoom=%4.2f  aa=%-3s  "
           "shadow=%-3s  theme=%s  debug=%s  %s ",
           fps, (double)s->k_blend, (double)s->speed, (double)s->cam_z,
           s->aa_enabled ? "on" : "off", s->soft_shadows ? "on" : "off",
           THEMES[s->theme].name, DEBUG_MODE_NAMES[s->debug_mode],
           s->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > sc->cols)
    slen = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - slen, "%s", status);
  mvprintw(0, 0, " METABALLS ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  j/k:blend  z/Z:zoom  a:aa  "
           "s:shadow  c:theme  d/D:debug  +/-:speed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps) {
  erase();
  canvas_render(&sc->canvas, s);
  canvas_draw_active(&sc->canvas, sc->cols, sc->rows, s);
  hud_draw(sc, s, fps);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §21 app — main loop, signals, key handling ──────────────────────── */

typedef struct {
  Scene scene;
  Screen screen;
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

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;

  case 'k':
    if (s->k_blend < K_MAX)
      s->k_blend *= K_STEP;
    if (s->k_blend > K_MAX)
      s->k_blend = K_MAX;
    break;
  case 'j':
    if (s->k_blend > K_MIN)
      s->k_blend /= K_STEP;
    if (s->k_blend < K_MIN)
      s->k_blend = K_MIN;
    break;

  case 's':
  case 'S':
    s->soft_shadows = !s->soft_shadows;
    break;

  case 'a':
  case 'A':
    s->aa_enabled = !s->aa_enabled;
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

  case 'c':
  case 'C':
    s->theme = (s->theme + 1) % N_THEMES;
    break;

  case 'd':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    s->debug_mode =
        (DebugMode)((s->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
    break;

  case '+':
  case '=':
    if (s->speed < SPD_MAX)
      s->speed *= SPD_STEP;
    if (s->speed > SPD_MAX)
      s->speed = SPD_MAX;
    break;
  case '-':
  case '_':
    if (s->speed > SPD_MIN)
      s->speed /= SPD_STEP;
    if (s->speed < SPD_MIN)
      s->speed = SPD_MIN;
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
  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app->need_resize = 0;
      screen_resize(&app->screen);
      frame_time = clock_ns();
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_MS * NS_PER_MS)
      dt = DT_CAP_MS * NS_PER_MS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    int ch;
    while ((ch = getch()) != ERR)
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }

    scene_tick(&app->scene, dt_sec);

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    screen_draw(&app->screen, &app->scene, fps_display);

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
