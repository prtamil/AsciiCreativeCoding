/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * forest_god_rays.c — volumetric path tracer · bare forest with low sun
 *
 * DEMO: A horizontal field of view at eye level into a stand of bare
 *       trunks. A finite bright sun-disc sits low in the sky, drifting
 *       slowly in azimuth and elevation on different sinusoidal periods
 *       (full sweep ~3-4 minutes). Mist hugs the ground; wherever the
 *       eye ray points roughly toward the sun AND the chord-to-sun is
 *       not blocked by a trunk, volumetric scattering produces a
 *       bright SHAFT. Wherever a trunk blocks the chord, the same
 *       volume of mist reads as plain fog without the shaft — which
 *       is what makes the shafts visibly RADIATE from the gaps in the
 *       canopy.
 *
 *       Cycle four debug overlays with `d' (NORMAL → SCATTER → SURFACE
 *       → TR) to see each component of the rendering equation in
 *       isolation.
 *
 * CORE TECHNIQUE: per pixel, ray-march from the camera through height-
 * decaying mist; at each sample point, NEE (next event estimation)
 * asks "can I see the sun from here, tree-occluded test?" and
 * accumulates Henyey-Greenstein-weighted in-scattered radiance from
 * any unblocked sample. The shafts emerge because only points with
 * line-of-sight to the sun (between trunks) contribute — points
 * inside a trunk's shadow get nothing. No special "cone" geometry
 * exists; the cones ARE the unobstructed regions of the mist.
 *
 * Section map (the spine):
 *   §1  config             camera, sun, atmosphere, trees, presets
 *   §2  clock              monotonic timer + sleep
 *   §3  vec3 math          V3 + ops
 *   §4  rgb math           RGB + ops
 *   §5  blackbody          kelvin → RGB (Tanner-Helland)
 *   §6  tone map           Reinhard + 1/2.2 gamma + luma
 *   §7  atmospheric medium height-decaying σ_e + analytic Tr_to_sun
 *   §8  phase function     Henyey-Greenstein
 *   §9  RNG                hash3 + hash01 (forest layout)
 *   §10 ncurses paint      6×6×6 cube quantise + glyph ramp
 *   §11 ray-plane          ray vs horizontal plane (ground)
 *   §12 ray-cylinder       ray vs vertical finite-height cylinder (trunks)
 *   §13 scene layout       Tree struct + place_trees + sun-block test
 *   §14 scene_hit          nearest of {tree, ground, sky}
 *   §15 sky / sun          gradient + sun-disc radiance
 *   §16 surface shading    shade_ground (Lambertian + NEE), shade_tree
 *   §17 trace_ray (CORE)   volumetric path tracer, all components
 *   §18 scene state        Scene + sun motion + reseed
 *   §19 camera             pinhole (no pitch — looks straight ahead)
 *   §20 screen + scene_draw includes debug-overlay selection
 *   §21 HUD                status row + key-hint strip
 *   §22 app                signals, main loop, input
 *
 * Companion files using related path-tracer kernels:
 *   raytracing/god_rays_silhouette.c  cathedral interior, slit-aware
 *                                     wall, UNIFORM mist
 *   raytracing/solar_eclipse.c        eclipse with corona / chromosphere
 *
 * KELVIN PRESETS (`t' / `T' cycles):
 *   DAWN     2000K   deep red (pre-sunrise)
 *   GOLDEN   3500K   warm orange (golden hour)
 *   NOON     5500K   white (full day)
 *   DUSK     2500K   orange (early evening)
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause sun + scene
 *   r            reseed forest layout
 *   t / T        next / previous kelvin preset
 *   d            cycle debug overlay (NORMAL / SCATTER / SURFACE / TR)
 *   z / Z        zoom in / out (0.25× .. 4×)
 *   + / =        faster sun motion (-: slower)
 *   ] / [        raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/forest_god_rays.c \
 *       -o forest_god_rays -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Volumetric path tracing is a SHORT idea hiding
 *      behind dense math; the prose teaches the idea, the code
 *      implements it. Skip only if you already know what NEE,
 *      Beer-Lambert, and Henyey-Greenstein are.
 *   2. §1 config — every constant has a unit. The fastest tour of
 *      what is tunable.
 *   3. §17 trace_ray — the volumetric path tracer. ~50 lines of body
 *      that fuses passes 1-7 of T3 into one loop. THE core.
 *   4. The supporting sections in any order. §7 (atmospheric
 *      medium), §8 (phase function), §12 (cylinder), §13 (sun-block
 *      test) are the pieces trace_ray uses; read whichever you don't
 *      yet trust.
 *   5. §10/§19-§22 are infrastructure (ncurses + main loop). Skip
 *      on a first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere. The file uses these
 *   consistently:
 *     ray_origin / ray_dir            primary ray as (point, unit dir)
 *     scatter_point                   midpoint of a march step
 *     extinction_coefficient (σ)      density of the medium at a point
 *     transmittance_along_eye  (T)    fraction of light surviving the
 *                                     eye-ray walk so far
 *     transmittance_to_sun     (Tr)   medium transmittance from a
 *                                     scatter point toward the sun
 *     in_scatter_radiance             accumulator: light scattered into
 *                                     the eye ray at every march step
 *     surface_radiance                light leaving the far surface
 *     phase_value                     Henyey-Greenstein at this angle
 *     step_length              (dt)   march_distance / MARCH_STEPS
 *
 *   Single-letter names appear only inside tight loops (`r`, `c`, `i`)
 *   or as ad-hoc symbols whose meaning is on the line above.
 *
 * Background you need
 * ───────────────────
 *   - Vector dot product, normalisation.
 *   - The integral ∫f dx as "average of f over the domain × domain
 *     size", and a Riemann sum as "approximate the integral by adding
 *     up samples".
 *   - Exponential decay (radioactive decay, drug half-life) and
 *     therefore the form T(t) = exp(-σt). For NON-uniform σ:
 *     T(t) = exp(-∫σ ds) — an integral inside the exponent.
 *   - Quadratic formula (used by ray-cylinder).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Surface BRDFs beyond Lambertian.
 *   - Mie / Rayleigh scattering theory; we use Henyey-Greenstein
 *     as a phenomenological phase function and don't derive it.
 *   - Spatial-acceleration structures (BVH, KD-tree); the forest
 *     has ~14 trees and we test all of them brute-force.
 *
 * This file is the OUTDOOR sibling of god_rays_silhouette.c. The
 * core algorithm is identical (volumetric path tracing with NEE).
 * The differences are scene-specific:
 *
 *   god_rays_silhouette.c          forest_god_rays.c (this file)
 *   ──────────────────────         ─────────────────────────────
 *   indoors, dim                   outdoors, daylight
 *   wall-with-windows occluder     cylinder-trunk occluders
 *   UNIFORM mist                   HEIGHT-DECAYING mist
 *   slit-aware single test         iterate every trunk
 *   sun on horizon (el = 0)        sun above horizon (el ≈ 0.22 rad)
 *   pitched camera                 horizontal camera
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Single-bounce volumetric path tracing with
 *                  Next Event Estimation (NEE). For each pixel we
 *                  RAY-MARCH from the camera through height-decaying
 *                  mist. At each march step we ask "is the sun
 *                  visible from here?" — i.e., does any tree block
 *                  the chord from this scatter point toward the sun?
 *                  If no, we accumulate Henyey-Greenstein-weighted
 *                  in-scattered radiance from the sun. The visible
 *                  shafts are the regions of mist that have line-of-
 *                  sight to the sun through gaps in the trunks.
 *
 *                  At the far end of the march (a tree, the ground,
 *                  or open sky) we add the surface or sky radiance
 *                  dimmed by the remaining transmittance along the
 *                  eye ray.
 *
 *                  See §17 trace_ray for the integrator. The whole
 *                  algorithm fits in ~50 lines of body.
 *
 * Data-structure : Static tree array (max 20 cells × 70% fill ≈ 14
 *                  active). Per-frame per-pixel HDR float RGB
 *                  accumulator (g_buf) so tone-mapping happens once
 *                  per frame at the ncurses-paint stage. No malloc,
 *                  no per-pixel malloc, no dynamic state.
 *
 * Rendering      : Reinhard tone-map L/(1+L) → 1/2.2 gamma → 6×6×6
 *                  xterm cube colour pair + 16-glyph density ramp.
 *                  Bright cells get A_BOLD, dark cells A_DIM, so the
 *                  ASCII output retains visible dynamic range even
 *                  on terminals that quantise the cube aggressively.
 *
 * Performance    : ~32 march steps × cell-count × (1 + ~14 trees per
 *                  NEE test). At 200×60 cells, MARCH_STEPS=32 and
 *                  N_trees~14 ≈ 5.4 M operations/frame. Modern CPU:
 *                  ~10 ms/frame; comfortable at 30 Hz with budget
 *                  left for the HUD and tone-map. No spatial
 *                  acceleration is needed at this scene size.
 *
 * References     : Pharr, Jakob & Humphreys, "Physically Based
 *                    Rendering" 4e, https://pbr-book.org. Chapter 14
 *                    (light transport in participating media) is
 *                    the definitive treatment.
 *                  Cerezo, E. et al., "A Survey on Participating
 *                    Media Rendering Techniques", The Visual
 *                    Computer, 2005. Maps the technique landscape.
 *                  Henyey, L. G. & Greenstein, J. L., "Diffuse
 *                    radiation in the galaxy", Astrophysical
 *                    Journal, 1941. The original phase function.
 *                  Veach, E., "Robust Monte Carlo Methods for Light
 *                    Transport Simulation", PhD thesis, Stanford
 *                    1997. Chapter on direct-lighting NEE.
 *                  Helland, T., "How to Convert Temperature (K) to
 *                    RGB", https://tannerhelland.com/2012/09/18/
 *                    convert-temperature-rgb-algorithm-code.html.
 *                    Source of the blackbody approximation in §5.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Imagine a SUNNY MISTY MORNING. Three truths combine to produce the
 * visible shafts of light:
 *
 *   1. The air contains MIST (water droplets) that scatters some
 *      light from any photon passing through it — INCLUDING photons
 *      travelling sideways relative to the camera. So you can SEE the
 *      sunlight from the side, not just when looking straight at it.
 *   2. The mist is DENSER NEAR THE GROUND (wakes-of-condensation
 *      phenomenon). Density decays exponentially with height.
 *   3. TREES BLOCK SUNLIGHT. A photon starting at a scatter point
 *      and aimed at the sun either passes through a clear path between
 *      trunks (and contributes scattered light to the camera) or hits
 *      a trunk (and contributes nothing).
 *
 * The visible shafts are NOT "rays of light" in the literal physics
 * sense. They are ZONES OF MIST that have line-of-sight to the sun
 * through gaps in the trunks. Every cell of mist scatters; only the
 * cells with clear sun-paths emit perceptible light into the camera.
 *
 * The integral the path tracer evaluates is the volumetric rendering
 * equation:
 *
 *   L_eye = ∫₀^t_max  T(0, t) · σ_s(p_t) · L_in_scatter(p_t)  dt
 *         + T(0, t_max) · L_surface
 *
 *   where  L_in_scatter(p)  ≈  phase(angle) · Tr(p → sun) · vis(p, sun) · L_sun
 *
 * — using NEE to reduce the inner spherical integral to one sample
 * (the sun direction) instead of a Monte Carlo sweep.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *   Side view of the scene (camera at left, sun upper-right):
 *
 *                                                         ☀ sun
 *                                                       /     (low,
 *                                                      /        el ≈ 0.22)
 *      +─────────────────────────────────────────────/───+
 *      |                          ┃   ┃           //     |  sky
 *      |              tree        ┃   ┃        //        |  (grad)
 *      |              ┃   ┃       ┃   ┃     //           |
 *      |    eye       ┃   ┃     ↓ ┃ ↓ ┃  //              |
 *      |    C ────────┃───┃───↓───┃─↓─┃//                |
 *      |              ┃   ┃   ↓   ┃ ↓ //                 |
 *      |    ───── march steps   ↓   ┃//                   |
 *      |  ───────────────────────────/────────────────────| ground
 *      ←  T(0,t) shrinks via dense ground mist            →
 *      camera         trees                            sun
 *
 *   Each `↓' along the eye ray is one march step. At each one, we ask:
 *   "if a photon were here and tried to fly toward the sun, would
 *   any tree intercept it?" — the NEE test. If no, we add a sliver
 *   of in-scattered radiance to this pixel, weighted by:
 *
 *     · the phase function (HG): how much sun light scatters into
 *       the EYE-RAY direction at this point (forward-biased blob)
 *     · the medium transmittance Tr from this point along the chord
 *       to the sun (closed form for height-decaying σ — T2)
 *     · the eye-ray transmittance T(0, t) so far (light scattered
 *       at deeper points contributes less because more of it gets
 *       extincted by mist closer to the camera)
 *     · σ at the scatter point (high near ground, low up high)
 *     · the step length dt
 *
 *   Cells where any tree blocks the chord to the sun get NOTHING
 *   from this term. That's the negative space that paints the
 *   shadow gaps between shafts.
 *
 * ALGORITHM IN STEPS  (per pixel)
 * ───────────────────────────────
 *   1. Generate primary ray from the camera (no pitch, straight forward).
 *   2. Find what the scene hits — tree, ground, or sky.
 *      Set march_distance = min(surface t, FAR_CLIP).
 *   3. step_length = march_distance / MARCH_STEPS.
 *   4. Compute phase_value once — depends on (eye, sun) angle, which
 *      is constant for this pixel.
 *   5. transmittance_along_eye = 1.0   (camera sees full radiance)
 *      in_scatter_radiance     = 0
 *   6. For i = 0 .. MARCH_STEPS-1, while transmittance_along_eye > ε:
 *        a. scatter_point = ray_origin + ray_dir · ((i + 0.5) · step_length)
 *        b. extinction = σ(scatter_point) = MIST_SIGMA · exp(-y / H)
 *        c. if extinction > 0:
 *             d. blocked = ray_cylinder hits any tree from p toward sun
 *             e. if NOT blocked:
 *                  Tr_to_sun = closed-form column transmittance
 *                  step_contribution = extinction · phase_value
 *                                    · Tr_to_sun · GAIN · step_length
 *                  in_scatter_radiance += sun_emit · step_contribution
 *                                       · transmittance_along_eye
 *             f. transmittance_along_eye *= exp(-extinction · step_length)
 *   7. surface_radiance = sky/ground/tree at the far hit.
 *   8. final_radiance = in_scatter_radiance
 *                     + surface_radiance · transmittance_along_eye
 *   9. tone-map → cube colour + glyph → ncurses paint.
 *
 * KEY FORMULAS
 * ────────────
 *   Volumetric rendering equation (1-bounce, NEE-simplified):
 *     L_eye = ∫₀^t_max T(t) σ(p_t) phase(θ) Tr(p_t→sun) vis(p_t, sun) L_sun dt
 *           + T(t_max) L_surface
 *
 *   Beer-Lambert (extinction):
 *     T(0, t) = exp(-∫₀^t σ_e(s) ds)
 *
 *   Height-decaying mist density (this scene):
 *     σ_e(p) = MIST_SIGMA · exp(-p.y / H)
 *
 *   Closed-form transmittance to sun for height-decaying mist:
 *     For sun direction with sun_dir.y > 0,
 *     Tr(p → sun) = exp(-∫₀^∞ MIST_SIGMA · exp(-(p.y + s·sun_dir.y) / H) ds)
 *                 = exp(- MIST_SIGMA · exp(-p.y/H) · H / sun_dir.y)
 *     (See §7.3 for derivation.)
 *
 *   Henyey-Greenstein phase function:
 *     P(cosθ; g) = (1 − g²) / [4π · (1 + g² − 2g·cosθ)^1.5]
 *
 *   NEE for a directional light (the sun):
 *     L_in(p, ω_o) ≈ phase(ω_o · sun_dir) · Tr(p→sun) · vis(p, sun) · L_sun
 *
 *   Reinhard tone map + gamma:
 *     L' = L / (1 + L)
 *     out = L'^(1/2.2)
 *
 * WORKED EXAMPLE  (one pixel, four march steps, by hand)
 * ──────────────────────────────────────────────────────
 *   Pixel at frame centre. Eye ray points roughly toward the sun
 *   (cos_eye_to_sun ≈ 0.95). March from the camera (z=0) into the
 *   forest. Camera at y = 1.6, so steps near y < 2 are dense mist:
 *
 *     i  step_t    p.y    σ(p)              visible?  Tr_to_sun
 *     0   1.25     1.6    0.25·e^-1.6 ≈ 0.05  YES      ≈ 0.71
 *     1   3.75     1.6    0.05               YES       ≈ 0.71
 *     2   6.25     1.6    0.05               NO* (tree #3 blocks)
 *     3   8.75     1.6    0.05               YES      ≈ 0.71
 *
 *   Even at MIST_SIGMA = 0.25 the sigma in our scene is fairly low
 *   at eye level (y = 1.6, exp(-1.6) ≈ 0.20). Most extinction
 *   happens in the bottom metre.
 *
 *   The in-scatter at step 2 is ZERO because tree #3 blocks the
 *   sun chord — that's exactly where you see a darker stripe in the
 *   image, between the shafts of step 1 and step 3.
 *
 *   The transmittance along eye T(0, t_max) for this pixel after 32
 *   steps is approximately exp(-σ̄ · t_max) ≈ exp(-0.04 · 40) ≈ 0.20.
 *   So the eventual sky radiance at the pixel reaches the camera at
 *   ~20% strength — good visible signal.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CYLINDER MATH. ray_cylinder is a 2D quadratic in xz (T6); a
 *    common bug is forgetting to test the height range of the hit
 *    point (the cylinder is finite). Without that check, distant
 *    sky cells beyond a cylinder column would still register as hits.
 *  • SUN BELOW HORIZON. tr_to_sun returns 0 if sun_dir.y < 1e-3
 *    (chord parallel to or below ground → infinite optical depth
 *    through the densest part of the mist). With SUN_EL_BASE ≈ 0.22
 *    rad and SUN_EL_AMP ≈ 0.044 we always have sun_dir.y > 0.17, so
 *    this guard never fires in practice.
 *  • UNIFORMLY-FAR FAR_CLIP. The march goes to 40 m even when the
 *    ray hits sky early. That's because past trees we're still IN
 *    the mist and want to accumulate scatter. Forest pixels often
 *    march all 32 steps; only tree-occluded pixels stop early.
 *  • NEE BLOCK DISTANCE (SUN_NEE_DIST = 50). The block test caps at
 *    50 m — beyond that the trees in our scene are out of range
 *    anyway. With TREE_Z_MAX = 28 m the test is effectively
 *    unlimited; the cap is a safety against future wider scenes.
 *  • TONE-MAP AFTER ACCUMULATE. Same warning as for any HDR
 *    renderer: tone-map ONCE at paint time, not during the volumetric
 *    sum. Reinhard is non-linear; tonemap(sum) ≠ sum(tonemap).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `r' a few times — the forest layout should change. Some
 *    seeds have crowded trees, some have wide gaps. Empty cells
 *    (TREE_FILL_PROB = 0.7 → ~30% empty) produce wider sky gaps.
 *  • Press `t' — sun chromaticity should warm/cool. The shafts tint
 *    with it (because the in-scatter contribution is multiplied by
 *    sun_em which is sun_chrom × HDR factor).
 *  • Press `d' to cycle:
 *      NORMAL    full render
 *      SCATTER   in-scatter only — ONLY the shafts of light, no
 *                trees or sky visible. The shadow gaps where trees
 *                block the sun stand out as dark stripes.
 *      SURFACE   surface only (no in-scatter) — bare scene with
 *                trees and ground but no atmospheric glow.
 *      TR        gray-scale eye-ray transmittance at the surface.
 *                Bright = sky / clear medium; dark = thick mist or
 *                ray hit something close.
 *  • Wait ~30 s — sun drifts (azimuth + elevation); shafts shift.
 *  • SHAFT-TREE ALIGNMENT: in NORMAL mode the shafts should cluster
 *    in the BACKGROUND between visible foreground tree silhouettes.
 *    A shaft right behind a foreground tree is impossible — that
 *    tree blocks the chord-to-sun for every cell behind it.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eleven short tutorials that build volumetric path tracing for an
 * outdoor forest scene from first principles. Read in order; each
 * builds on the previous.
 *
 *   T1   The volumetric rendering equation — what we are computing
 *   T2   Beer-Lambert for HEIGHT-DECAYING mist (the closed form)
 *   T3   The 7 stages of this file's path tracer
 *   T4   Henyey-Greenstein: the BRDF for fog
 *   T5   NEE: shadow rays as cheap variance reduction
 *   T6   Cylinder intersection — finite-height vertical trunk
 *   T7   Jittered grid + probabilistic gaps — natural-looking forests
 *   T8   Sky gradient + sun disc — outdoor sky model
 *   T9   Sun motion + blackbody — the warmth of the light
 *   T10  Tone mapping HDR → ASCII — Reinhard, gamma, cube, ramp
 *   T11  Four debug overlays — see each component on its own
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE VOLUMETRIC RENDERING EQUATION — WHAT WE ARE COMPUTING
 * ──────────────────────────────────────────────────────────────
 * Surface rendering: "the colour of a pixel is the light leaving the
 * FIRST surface the eye ray hits."
 *
 * Volumetric rendering: "the colour of a pixel is the light arriving
 * at the camera from EVERY POINT along the eye ray, plus the surface
 * at the end (dimmed by what the medium absorbs along the way)."
 *
 * Mathematically, with σ_s = scattering coefficient and σ_e = total
 * extinction (here σ_s = σ_e — pure scattering):
 *
 *   L_eye = ∫₀^t_max  T(0, t) · σ_s(p_t) · L_in_scatter(p_t)  dt
 *         + T(0, t_max) · L_surface
 *
 *   T(a, b) = exp(-∫_a^b σ_e(s) ds)         (Beer-Lambert, T2)
 *
 *   L_in_scatter(p) = ∫_Ω P(ω, ω_eye) · L_i(p, ω) dω      (inner integral)
 *
 * The OUTER integral marches along the eye ray. The INNER integral
 * sums all incoming directions at each scatter point. Both are
 * recursive in L (light from a direction is itself the answer to a
 * rendering equation at the next surface), and we simplify both:
 *
 *  · OUTER — Riemann sum: split [0, t_max] into MARCH_STEPS pieces
 *    and approximate the integral as a sum of step contributions.
 *  · INNER — NEE (T5): replace the sphere of directions with ONE
 *    sample toward the sun. Unbiased for a sun-only-light scene.
 *
 * After both simplifications:
 *
 *   L_eye ≈ Σ_i  T(0, t_i) · σ_s(p_i) · phase(θ_i) · Tr(p_i → sun)
 *                          · vis(p_i, sun) · L_sun · step_length
 *         +  T(0, t_max) · L_surface
 *
 * where vis() iterates every tree (T6+T7), Tr is the height-decaying
 * mist transmittance (T2), and θ_i is the angle between the eye ray
 * and the sun direction.
 *
 * Read §17 trace_ray after T2 and T3 — that one summation IS the body
 * of the loop.
 *
 * T2  BEER-LAMBERT FOR HEIGHT-DECAYING MIST
 * ──────────────────────────────────────────
 * For uniform σ_e, T(0, t) = exp(-σ_e · t) — closed form, trivial.
 * Real-world mist is RARELY uniform; it's denser near the ground
 * where temperature/condensation favour droplet formation. We model
 * this with EXPONENTIAL HEIGHT FALLOFF:
 *
 *     σ_e(p) = MIST_SIGMA · exp(-p.y / H)         H = MIST_SCALE_H = 1.0 m
 *
 * Now T along a chord is a more involved integral. For the EYE-RAY
 * march we just evaluate σ at each scatter point and update T step-
 * wise (see §17 line `transmittance_along_eye *= ...`). The
 * straightforward Riemann sum.
 *
 * For the SUN-DIRECTION chord (Tr_to_sun), we want a CLOSED form
 * because the alternative is a second nested march per scatter point
 * — 24× more cost. The closed form depends on this trick: for sun
 * direction with a vertical component sun_dir.y > 0, the chord from
 * p in direction sun_dir at distance s has y = p.y + s·sun_dir.y. So
 *
 *   Tr_to_sun(p) = exp(- ∫₀^∞ MIST_SIGMA · exp(-(p.y + s·sun_dir.y)/H) ds)
 *
 *   Substitute u = (p.y + s·sun_dir.y) / H, du = sun_dir.y/H · ds:
 *
 *                = exp(- MIST_SIGMA · ∫_(p.y/H)^∞ exp(-u) · H/sun_dir.y · du)
 *                = exp(- MIST_SIGMA · H/sun_dir.y · exp(-p.y/H))
 *                = exp(- σ_e(p) · H / sun_dir.y)
 *
 * One closed-form transcendental call per scatter point. See §7.3
 * for the implementation; this derivation IS the comment block above
 * the function.
 *
 *   Sanity check: when sun_dir.y → 1 (sun straight overhead), the
 *   formula gives Tr = exp(-σ(p) · H) — the column over the scatter
 *   point. When sun_dir.y → 0 (sun on horizon), the chord traverses
 *   infinite mist along the dense bottom and Tr → 0.
 *
 * T3  THE 7 STAGES OF THIS FILE'S PATH TRACER
 * ────────────────────────────────────────────
 * Every volumetric path tracer has these seven conceptual stages.
 * Naming them is the most useful single thing you can do for your
 * understanding.
 *
 *   Stage 1  CAMERA RAY GENERATION         (§19)
 *            pixel → world-space ray (pinhole, no pitch — looks
 *            straight ahead).
 *
 *   Stage 2  SCENE INTERSECTION            (§14)
 *            ray → nearest hit ∈ {tree, ground, sky}. Sets
 *            march_distance.
 *
 *   Stage 3  EYE-RAY MARCH                 (§17 main loop)
 *            split [0, march_distance] into MARCH_STEPS pieces.
 *            Visit the midpoint of each.
 *
 *   Stage 4  SUN VISIBILITY (NEE)          (§13.4)
 *            For each scatter point, ask: do any trees block the
 *            chord toward the sun? This IS the trick that produces
 *            the visible shafts.
 *
 *   Stage 5  IN-SCATTER CONTRIBUTION       (§17 inner block)
 *            σ · phase · Tr_to_sun · L_sun · dt · T(0, t).
 *            Henyey-Greenstein (§8) supplies phase; the closed-form
 *            height-decaying transmittance (§7.3) supplies Tr_to_sun.
 *
 *   Stage 6  EYE-RAY TRANSMITTANCE UPDATE  (§17 main loop)
 *            T(0, t) *= exp(-σ · step_length). The contribution at
 *            step i used the T BEFORE this step's attenuation;
 *            we update it AFTER, so step i+1 sees the correct
 *            cumulative.
 *
 *   Stage 7  SURFACE CONTRIBUTION + TONE MAP   (§17 tail + §10)
 *            L_surface (tree/ground/sky) × T(0, march_distance).
 *            Sum with in-scatter; Reinhard + gamma + cube + glyph.
 *
 * If you can recite these seven by name you can READ §17 trace_ray.
 *
 * T4  HENYEY-GREENSTEIN: THE BRDF FOR FOG
 * ────────────────────────────────────────
 * A surface BRDF says "for an incoming direction ω_i, what fraction
 * of light scatters toward outgoing ω_o?". A medium PHASE FUNCTION
 * is the same idea for a scatter event INSIDE a participating medium.
 *
 * The simplest physical phase functions (Mie scattering for water
 * droplets, Rayleigh for air molecules) are messy. Computer graphics
 * uses Henyey-Greenstein (1941, originally for galactic dust):
 *
 *   P(cosθ; g) = (1 − g²) / [4π · (1 + g² − 2g·cosθ)^1.5]
 *
 *   g ∈ (-1, 1)   "anisotropy parameter"
 *     g = 0       isotropic (same in all directions)
 *     g > 0       forward-biased (most scatter goes the same way the
 *                 photon was already going)
 *     g < 0       backward-biased (rare, exotic media)
 *
 * For mist / fog / dust, g ≈ 0.4 .. 0.85 produces the look people
 * expect — strong forward lobe, with enough spread that you can SEE
 * the shafts from the side. We use HG_G = 0.55: shafts have visible
 * core brightness with a soft halo, and the look stays calm as the
 * sun moves (a higher g value would make the highlight twitchy).
 *
 * In trace_ray, phase_value is computed ONCE per pixel — it depends
 * only on the angle between the eye ray and the sun direction, not
 * on the scatter point. (This is true because the sun is at infinity:
 * sun_dir is the same from every scatter point.)
 *
 * T5  NEE: SHADOW RAYS AS CHEAP VARIANCE REDUCTION
 * ─────────────────────────────────────────────────
 * Naïve volumetric Monte Carlo at each scatter point would do this:
 *
 *   pick random ω_in
 *   trace recursive ray in direction ω_in
 *   weight by phase(ω_in, ω_eye) / pdf(ω_in)
 *
 * That works (unbiased) but converges slowly because most random
 * directions don't hit the sun, contributing zero. With a small bright
 * light (the sun), this is a HUGE waste.
 *
 * Next Event Estimation: instead of picking a random direction, pick
 * the SUN'S DIRECTION DIRECTLY. Cast a shadow ray. If unblocked, add
 * the sun's contribution weighted by the phase function and medium
 * transmittance. If blocked, contribute zero.
 *
 * Mathematically equivalent to importance sampling against the sun's
 * position. For a single point/directional light it's UNBIASED and
 * VARIANCE-FREE — every sample finds the sun (or knows it's blocked),
 * no wasted random misses.
 *
 * In this file:
 *   · Sun is treated as DIRECTIONAL (a single direction, not a point):
 *     sun_dir is computed from azimuth/elevation in §18.
 *   · The shadow ray is virtual — we don't trace anything against the
 *     full scene. We just iterate every tree and ask `ray_cylinder'
 *     whether it intersects within SUN_NEE_DIST. See §13.4.
 *   · The sun has a finite angular radius (SUN_ANG_RADIUS) used by
 *     the sky-radiance disc test in §15. The NEE code uses sun_dir
 *     as a single direction (not the disc) — slightly biased when
 *     part of the disc is unblocked but most isn't, but visually
 *     indistinguishable at terminal resolution.
 *
 * T6  CYLINDER INTERSECTION — FINITE-HEIGHT VERTICAL TRUNK
 * ─────────────────────────────────────────────────────────
 * A trunk is a vertical cylinder: axis = +Y, base at (bx, 0, bz),
 * radius r, height h. We need ray-cylinder intersection.
 *
 * Substitute the ray P(t) = ray_origin + t · ray_dir into the
 * cylinder side equation (P.x - bx)² + (P.z - bz)² = r²:
 *
 *   Let δx = ray_origin.x - bx,  δz = ray_origin.z - bz.
 *
 *   (δx + t · ray_dir.x)² + (δz + t · ray_dir.z)² = r²
 *
 * Expand and group by powers of t:
 *
 *   t² (ray_dir.x² + ray_dir.z²)
 *     + 2t (δx · ray_dir.x + δz · ray_dir.z)
 *     + (δx² + δz² − r²) = 0
 *
 * Standard quadratic at² + bt + c = 0 with
 *
 *   a = ray_dir.x² + ray_dir.z²       (zero if ray is purely vertical)
 *   b = 2 (δx · ray_dir.x + δz · ray_dir.z)
 *   c = δx² + δz² − r²
 *
 * Discriminant disc = b² − 4ac. Roots t = (-b ± √disc) / 2a. The
 * NEAR root (t = (-b − √disc) / 2a) is the front face; the FAR root
 * is the back. Pick the near root if it's in front of the camera
 * (t > eps); else fall back to the far root (camera might be inside
 * the cylinder, which doesn't happen in our scene but the code
 * handles it).
 *
 * Then check the HEIGHT RANGE. The cylinder is FINITE-HEIGHT — we
 * only count side hits where ray_origin.y + t · ray_dir.y is between
 * the base and the top (base.y + h). If the y-coordinate at the
 * intersection is outside that range, the ray missed the cylinder
 * even though it crossed the infinite-cylinder side.
 *
 *   ASCII picture of finite-height cylinder hit-test:
 *
 *               ┃   ┃           ← top (base.y + h)
 *               ┃   ┃
 *      ─→   ────●───┃─────      ← side hit, y in range → HIT
 *               ┃   ┃
 *      ─→ ──●───┃───┃─────      ← side hit, y < base.y → MISS
 *               ┃   ┃
 *               ┻───┻           ← base (y = 0)
 *
 * See §12 for the implementation. The function returns the nearest
 * VALID hit (in front of camera + within height range).
 *
 * T7  JITTERED GRID + PROBABILISTIC GAPS — NATURAL-LOOKING FORESTS
 * ─────────────────────────────────────────────────────────────────
 * A forest is "trees somewhere in 2D ground space, a few metres
 * apart". The cheapest natural-looking layout uses a JITTERED GRID:
 *
 *   1. Divide the visible region into a coarse grid (5 azimuth ×
 *      4 depth = 20 cells in this scene).
 *   2. Each cell INDEPENDENTLY decides: "place a tree?" with
 *      probability TREE_FILL_PROB = 0.70. Empty cells leave gaps.
 *   3. For cells with trees, JITTER the tree's position (and size,
 *      and trunk diameter) within the cell's range. Each draw uses
 *      a deterministic per-cell hash so seeds are reproducible.
 *
 * Variations:
 *
 *   · DEPTH BINS are LOG-SPACED. Closer cells span small z-ranges,
 *     far cells span large z-ranges. This matches the visual
 *     density of a real forest — apparent tree density per unit of
 *     screen space stays roughly constant across the depth range.
 *
 *   · AZIMUTH BINS are linear within ±AZIMUTH_HALF (slightly wider
 *     than the camera's FOV). Cells near the edges sometimes place
 *     trees outside the visible frame — adds to the natural-sparse
 *     feel.
 *
 *   · INDEPENDENT HASHES per draw (fill / depth / azim / radius /
 *     height) so all five draws are statistically uncorrelated. A
 *     single hash for "this cell" would produce noticeable patterns
 *     (taller trees correlated with darker trees etc).
 *
 * Active trees pack into the front of g_trees[] so ray_scene_hit
 * iterates only the populated entries. See §13 for the implementation.
 *
 * T8  SKY GRADIENT + SUN DISC — OUTDOOR SKY MODEL
 * ────────────────────────────────────────────────
 * The "background" radiance for any eye ray that ends up looking at
 * SKY (didn't hit tree or ground). Two components:
 *
 *   1. Vertical gradient   horizon-tinted (warm) at h ≈ 0,
 *                          zenith-tinted (cool blue) at h ≈ 1.
 *   2. Sun disc            additive bright spot when the ray points
 *                          inside SUN_ANG_RADIUS of sun_dir.
 *
 * Pseudocode:
 *   h     = clamp01(ray_dir.y)              0 = horizon, 1 = zenith
 *   sky   = lerp(horizon_col, ZENITH, h)
 *   if cos(ray_dir, sun_dir) > cos(SUN_ANG_RADIUS):
 *     edge = ramp from 0 at edge to 1 at centre
 *     sky += sun_em · edge
 *   return sky
 *
 * The horizon colour is `sun_chrom · SKY_HORIZON_FAC' — the kelvin
 * preset directly tints the horizon. So at DAWN (2000K) the horizon
 * is deep red; at NOON (5500K) it's near-white; at GOLDEN (3500K)
 * it's the warm orange you see at golden hour.
 *
 * The zenith stays a constant cool blue (SKY_ZENITH_*). That makes
 * the gradient look "right" regardless of kelvin: warm at the horizon,
 * cool overhead.
 *
 * T9  SUN MOTION + BLACKBODY — THE WARMTH OF THE LIGHT
 * ─────────────────────────────────────────────────────
 * The sun is at infinity (a direction, not a point). Its direction
 * unit vector at simulation time t:
 *
 *   az = SUN_AZ_BASE + SUN_AZ_AMP · sin(2π·t / SUN_AZ_PERIOD_S)
 *   el = SUN_EL_BASE + SUN_EL_AMP · sin(2π·t / SUN_EL_PERIOD_S)
 *
 *   sun_dir = norm(sin(az)·cos(el), sin(el), cos(az)·cos(el))
 *
 * For this scene:
 *   az ∈ ±0.087 rad ≈ ±5°,   period 280 s
 *   el ∈ 0.176 .. 0.264 rad ≈ 10° .. 15°,   period 200 s
 *
 * Different sin periods so the trajectory doesn't visibly repeat for
 * minutes. Together with HG_G = 0.55 the shafts shift smoothly.
 *
 * The sun's COLOUR is a Planckian blackbody at the chosen kelvin
 * temperature. blackbody_rgb (§5) is the Tanner-Helland approximation
 * — three piecewise polynomials per channel, fitted to the CIE
 * standard.
 *
 * The chromaticity is then scaled to HDR brightness:
 *
 *   sun_emit = sun_chrom · SUN_EMIT_HDR    (HDR factor, 14)
 *   horizon  = sun_chrom · SKY_HORIZON_FAC (display, 0.4)
 *
 * Same chromaticity feeds the sky-horizon gradient — so the horizon
 * naturally tints with the sun's temperature.
 *
 * T10 TONE MAPPING HDR → ASCII
 * ─────────────────────────────
 * The accumulator stores LINEAR HDR — the sun's contribution can
 * easily exceed 1.0 (SUN_EMIT_HDR = 14 to begin with). To display we
 * compress to [0, 1] and then map to a discrete glyph + cube colour.
 *
 * Reinhard tone map:    L' = L / (1 + L)
 *   Smooth, monotonic, never clips. Maps L=0 → 0, L=1 → 0.5,
 *   L → ∞ → 1.
 *
 * Gamma encode:         out = L'^(1/2.2)
 *   sRGB displays apply ~2.2 gamma. Pre-compensate so what we draw
 *   is linearly proportional to scene radiance.
 *
 * Cube colour:          (r5, g5, b5) = round(out · 5)
 *   216 colour pairs from the xterm 6×6×6 cube.
 *
 * Glyph density:        luma = Rec.601 luma of (r, g, b)
 *                       glyph = k_ramp[round(luma · (RAMP_LEN-1))]
 *   16 glyphs from " " (dim) to "0" (densest).
 *
 * Bright cells get A_BOLD; dark cells A_DIM — keeps the dynamic
 * range visible on terminals that quantise the cube aggressively.
 *
 * T11 FOUR DEBUG OVERLAYS — SEE EACH COMPONENT ON ITS OWN
 * ────────────────────────────────────────────────────────
 * Cycling `d' shows one of four lenses on the same path-traced
 * scene:
 *
 *   NORMAL   The full render. In-scatter + (surface · T_far). What
 *            the user actually sees by default.
 *
 *   SCATTER  In-scatter ONLY. The shafts float in pure black; trees
 *            and sky vanish. Pedagogical: this shows where the path
 *            tracer's STAGE 5 (T3) accumulator is depositing
 *            radiance — the line-of-sight shafts of unblocked mist.
 *
 *   SURFACE  Surface-only. No mist, no scatter — bare scene with
 *            trees, ground, and sky. Pedagogical: this shows what
 *            the renderer would produce if MARCH_STEPS were 0 (or
 *            if MIST_SIGMA were 0).
 *
 *   TR       Gray-scale transmittance at the surface. White =
 *            T(0, t_max) ≈ 1 (clear medium); black = T ≈ 0 (thick
 *            mist or close hit). Pedagogical: this is the eye-ray
 *            transmittance after the march. Trees nearby read as
 *            black (the medium between camera and trunk dimmed
 *            visibility); distant sky reads as a bright fade
 *            (transmittance reduced by the integrated mist over 40m).
 *
 * Adding NORMAL = SCATTER + SURFACE · TR cell-by-cell would
 * exactly reproduce NORMAL. That equality is the rendering
 * equation reduced to two summands — a useful sanity check.
 *
 * ─────────────────────────────────────────────────────────────────── */

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
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ──────────────────────────────────────────────────────── */

/* §1.1 frame-rate and motion knobs. */
enum {
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,

    SPEED_MIN       =   1,
    SPEED_DEF       =   8,
    SPEED_MAX       =  64,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define DT_CAP_NS       (100 * NS_PER_MS)

/* §1.2 view geometry — pinhole camera, no pitch.
 *
 * Camera at world origin, eye-height above ground. Looking straight
 * down +Z. Unlike god_rays_silhouette.c (which pitches up to centre
 * the windows), this scene has no fixed feature to centre on — the
 * sun moves, the trees vary per seed — so we keep the camera straight
 * and let the sun drift through the upper part of the frame.
 */
#define ASPECT_Y        2.0f
#define FOV_H_BASE      0.55f
#define ZOOM_MIN        0.25f
#define ZOOM_MAX        4.0f
#define ZOOM_STEP       1.25f

#define CAMERA_HEIGHT   1.6f          /* eye level above ground (m)       */

/* §1.3 sun (treated as DIRECTIONAL — at infinity, T9).
 *
 * Sun above horizon (el ≈ 0.22 rad ≈ 12.6°). Drifts in azimuth and
 * elevation on different sin periods so the trajectory doesn't
 * visibly repeat for minutes. Each oscillation amplitude is small
 * (±2.5° elevation, ±5° azimuth) for a calm sunrise feel.
 *
 * SUN_DIST is unused for direction calculations (the sun is treated
 * as directional) but kept as a hint for ray_cylinder block tests:
 * SUN_NEE_DIST = 50 m caps the block test, which is well within sun
 * distance and well beyond TREE_Z_MAX.
 *
 * SUN_ANG_RADIUS is exaggerated from real (0.5°/0.0087 rad) so the
 * disc registers at terminal resolution.
 */
#define SUN_DIST       1000.0f
#define SUN_ANG_RADIUS    0.07f       /* rad — ~4° apparent disc          */
#define SUN_EMIT_HDR     14.0f        /* HDR brightness multiplier        */

#define SUN_EL_BASE       0.22f       /* rad — ~12.6° median elevation   */
#define SUN_EL_AMP        0.044f      /* rad — ±2.5°                     */
#define SUN_AZ_BASE       0.0f
#define SUN_AZ_AMP        0.087f      /* rad — ±5° azimuth               */
#define SUN_EL_PERIOD_S 200.0f
#define SUN_AZ_PERIOD_S 280.0f

/* §1.4 trees — bare cylinder trunks, jittered grid + probabilistic
 * gaps (T7).
 *
 * 5 azimuth bins × 4 depth bins = 20 cells total. Each cell
 * INDEPENDENTLY decides: place a tree with TREE_FILL_PROB = 0.70?
 * On average ~14 of 20 cells get trees; the 30% empty cells leave
 * gaps where the sun-shafts can be seen.
 *
 * Depth bins are LOG-spaced (close cells short, far cells long) so
 * apparent screen-density of trees stays roughly constant across the
 * depth range.
 *
 * TREE_Z_MIN = 3.5 m — closer than that, a single trunk dominates
 * the frame. TREE_Z_MAX = 28 m sets the visible far range.
 */
#define N_AZIM_BINS      5
#define N_DEPTH_BINS     4
#define N_TREES          (N_AZIM_BINS * N_DEPTH_BINS)
#define TREE_FILL_PROB   0.70f       /* per-cell probability of tree     */
#define TREE_Z_MIN       3.5f
#define TREE_Z_MAX      28.0f
#define AZIMUTH_HALF     0.55f       /* rad (~31°), wider than camera FOV */
#define TREE_R_MIN       0.08f
#define TREE_R_MAX       0.22f
#define TREE_H_MIN       4.5f
#define TREE_H_MAX       9.0f

/* §1.5 ground plane — flat horizontal plane at y = 0. */
#define GROUND_Y         0.0f
#define GROUND_ALBEDO_R  0.18f
#define GROUND_ALBEDO_G  0.13f
#define GROUND_ALBEDO_B  0.08f

/* §1.6 atmospheric medium — height-decaying mist (T2).
 *
 *   σ_e(p) = MIST_SIGMA · exp(-p.y / MIST_SCALE_H)
 *
 * Densest at the ground, falls off exponentially with height. The
 * closed-form Tr_to_sun (§7.3) requires this exponential shape —
 * if σ varied differently with height (linear, gaussian, etc.) we'd
 * need a numerical integral inside Tr_to_sun, costing 24× more.
 *
 * MARCH_STEPS = 32 across at most FAR_CLIP world units (~40 m) gives
 * step_length ≈ 1.25 m — coarse but the mist profile is smooth
 * enough that finer steps don't add much. HG_G = 0.55 — moderate
 * forward bias; cones get geometric definition from tree gaps so
 * the phase function doesn't have to do all the work.
 */
#define MIST_SIGMA       0.25f      /* peak σ at y = 0                   */
#define MIST_SCALE_H     1.0f       /* e-folding height (m)              */
#define HG_G             0.55f      /* HG anisotropy in (-1, 1)          */
#define MARCH_STEPS      32
#define FAR_CLIP        40.0f       /* march t_max for sky-bound rays    */
#define SUN_NEE_DIST    50.0f       /* tree-block test distance to sun   */
#define INSCATTER_GAIN   2.25f      /* shaft visibility multiplier        */

/* §1.7 sky gradient — orange near horizon, deep blue overhead (T8). */
#define SKY_ZENITH_R     0.08f
#define SKY_ZENITH_G     0.12f
#define SKY_ZENITH_B     0.22f
#define SKY_HORIZON_FAC  0.40f       /* horizon = sun_chrom · this        */

/* §1.8 buffer and ncurses pair-id reservations. */
#define BUF_MAX_W        400
#define BUF_MAX_H        200

enum {
    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_SUN_FALLBACK =  3,
    PAIR_CUBE_BASE    = 16,           /* + 0..215 = 6×6×6 cube           */
};

/* §1.9 ASCII glyph ramp (16 levels). */
static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.10 kelvin presets (T9). */
typedef struct {
    const char *name;
    float       kelvin;
} KelvinPreset;

static const KelvinPreset KELVINS[] = {
    { "DAWN  ", 2000.0f },
    { "GOLDEN", 3500.0f },
    { "NOON  ", 5500.0f },
    { "DUSK  ", 2500.0f },
};
#define N_KELVINS ((int)(sizeof KELVINS / sizeof KELVINS[0]))

/* §1.11 debug-overlay enum (T11). Cycled with `d'.
 *
 *   MODE_NORMAL     full render (in-scatter + surface · T_far)
 *   MODE_SCATTER    in-scatter only — shafts float in pure black
 *   MODE_SURFACE    surface only (no in-scatter, no extinction)
 *   MODE_TR         gray-scale eye-ray transmittance at far hit
 */
typedef enum {
    MODE_NORMAL  = 0,
    MODE_SCATTER = 1,
    MODE_SURFACE = 2,
    MODE_TR      = 3,
    MODE_N       = 4,
} DebugMode;

static const char *debug_mode_name(DebugMode m)
{
    switch (m) {
    case MODE_NORMAL:  return "NORMAL ";
    case MODE_SCATTER: return "SCATTER";
    case MODE_SURFACE: return "SURFACE";
    case MODE_TR:      return "TR     ";
    default:           return "?      ";
    }
}

/* ── §2 clock ───────────────────────────────────────────────────────── */

/*
 * clock_ns — monotonic wall time in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backward across NTP / DST / suspend;
 * the right choice for animation dt and fps measurement.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns — best-effort sleep. Used to cap render rate
 * without burning CPU at 100%; subtract elapsed work and sleep
 * the remainder.
 */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 vec3 math ────────────────────────────────────────────────────── *
 *
 * V3 — three floats by value. All helpers inlined to avoid call
 * overhead in the per-pixel inner loop.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3 (float x, float y, float z) { return (V3){x,y,z}; }
static inline V3    v3_add(V3 a, V3 b)   { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3_sub(V3 a, V3 b)   { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3_scl(V3 a, float s){ return v3(a.x*s, a.y*s, a.z*s);     }
static inline float v3_dot(V3 a, V3 b)   { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3_len(V3 a)         { return sqrtf(v3_dot(a, a));         }

/*
 * v3_norm — return a unit vector in the same direction as `a',
 * or (0,0,0) if `a' is zero-length (avoids NaN propagation).
 */
static inline V3 v3_norm(V3 a)
{
    float length = v3_len(a);
    if (length < 1e-12f) return v3(0, 0, 0);
    return v3_scl(a, 1.0f / length);
}

/* ── §4 rgb math ─────────────────────────────────────────────────────── *
 *
 * RGB — three linear-space floats. NOT clamped to [0,1]; HDR until
 * the tone-map stage in §10.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct { float r, g, b; } RGB;

static inline RGB rgb_make(float r, float g, float b) { return (RGB){r,g,b}; }
static inline RGB rgb_add (RGB a, RGB b)   { return rgb_make(a.r+b.r, a.g+b.g, a.b+b.b); }
static inline RGB rgb_mul (RGB a, RGB b)   { return rgb_make(a.r*b.r, a.g*b.g, a.b*b.b); }
static inline RGB rgb_scl (RGB a, float s) { return rgb_make(a.r*s, a.g*s, a.b*s);       }
static inline RGB rgb_lerp(RGB a, RGB b, float t)
{ return rgb_make(a.r+(b.r-a.r)*t, a.g+(b.g-a.g)*t, a.b+(b.b-a.b)*t); }

static inline float clamp01 (float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float luma_of (RGB c)
{ return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

/* ── §5 blackbody (Tanner-Helland) ──────────────────────────────────── *
 *
 * Map a colour temperature in kelvin to an approximate sRGB
 * chromaticity. The Tanner-Helland fit (2012) uses three piecewise
 * polynomials per channel — accurate to within a few percent over
 * 1000K-12000K, and CHEAP (a few transcendentals).
 *
 * Used by §15 sky_radiance and the HUD; the chosen kelvin preset
 * also drives sun_em (sun emission HDR) and the sky horizon tint.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * blackbody_rgb — Planckian-locus chromaticity at `kelvin'.
 *
 * Pseudocode:
 *   K = kelvin / 100
 *   r = 1.0                                     if K <= 66
 *       329.7  · pow(K-60, -0.1332) / 255       otherwise
 *   g = 99.47 · log(K)         - 161.12 / 255   if K <= 66
 *       288.12 · pow(K-60, -0.0755) / 255       otherwise
 *   b = 1.0                                     if K >= 66
 *       0                                       if K <= 19
 *       138.52 · log(K-10)     - 305.04 / 255   otherwise
 *   return clamp01((r, g, b))
 */
static RGB blackbody_rgb(float kelvin)
{
    float K = kelvin / 100.0f;
    float r, g, b;

    if (K <= 66.0f) r = 1.0f;
    else            r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

    if (K <= 66.0f) g = (99.47f  * logf(K)            - 161.12f) / 255.0f;
    else            g =  288.12f * powf(K - 60.0f, -0.0755f)      / 255.0f;

    if (K >= 66.0f)      b = 1.0f;
    else if (K <= 19.0f) b = 0.0f;
    else                 b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

    return rgb_make(clamp01(r), clamp01(g), clamp01(b));
}

/* ── §6 tone map ─────────────────────────────────────────────────────── *
 *
 * Reinhard L/(1+L) compresses HDR → [0, 1); 1/2.2 gamma converts
 * linear → sRGB. Applied per channel inside paint_cell (§10).
 *
 * Tone-mapping happens ONCE per cell at paint time, after all
 * accumulation. NEVER tone-map during the volumetric sum: Reinhard
 * is non-linear, tonemap(sum) ≠ sum(tonemap).
 *
 * ─────────────────────────────────────────────────────────────────── */

static inline float reinhard (float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* ── §7 atmospheric medium ───────────────────────────────────────────── *
 *
 * Two helpers:
 *
 *   §7.1  sigma_e_at  — height-decaying extinction coefficient
 *   §7.2  Tr_to_sun   — analytic transmittance from a point along
 *                       the sun-direction chord through the medium
 *                       (closed form because σ_e is exponential in y)
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §7.1 ── sigma_e_at: extinction coefficient at a point ────────────── */

/*
 * sigma_e_at — extinction at world point p.
 *
 *   σ_e(p) = MIST_SIGMA · exp(-y / MIST_SCALE_H)        y >= 0
 *
 * Below ground (y < 0) we clamp y to 0 to avoid the formula blowing
 * up. In practice scatter points always have y >= 0 in our scene
 * (camera at y = 1.6, ground at y = 0).
 */
static inline float sigma_e_at(V3 scatter_point)
{
    float y = (scatter_point.y < 0.f) ? 0.f : scatter_point.y;
    return MIST_SIGMA * expf(-y / MIST_SCALE_H);
}

/* §7.2 ── Tr_to_sun: medium transmittance from p toward sun ────────── */

/*
 * tr_to_sun — analytic transmittance from p along sun_dir, integrated
 *             through the height-decaying mist column (T2).
 *
 * Pseudocode:
 *   if sun_dir.y < ε:           sun on/below horizon → 0 (chord
 *                               traverses infinite low-altitude mist)
 *   y = max(p.y, 0)
 *   τ = MIST_SIGMA · exp(-y/H) · H / sun_dir.y
 *   return exp(-τ)
 *
 * Derivation (T2):
 *   Tr = exp(-∫₀^∞ MIST_SIGMA · exp(-(p.y + s·sun_dir.y)/H) ds)
 *      = exp(-MIST_SIGMA · exp(-p.y/H) · H/sun_dir.y)
 *      = exp(-σ_e(p) · H / sun_dir.y)
 *
 * One closed-form transcendental call. NO second march required.
 * The cheap closed form is what makes this scene affordable; if σ
 * varied differently (linearly, gaussian) we'd need a numerical
 * integral here.
 */
static inline float tr_to_sun(V3 scatter_point, V3 sun_dir)
{
    if (sun_dir.y < 1e-3f) return 0.0f;
    float y   = (scatter_point.y < 0.f) ? 0.f : scatter_point.y;
    float optical_depth = MIST_SIGMA * expf(-y / MIST_SCALE_H)
                        * MIST_SCALE_H / sun_dir.y;
    return expf(-optical_depth);
}

/* ── §8 phase function ───────────────────────────────────────────────── *
 *
 * Henyey-Greenstein (T4):
 *
 *   P(cosθ; g) = (1 − g²) / [4π · (1 + g² − 2g·cosθ)^1.5]
 *
 * g > 0 forward-biased; tuned to HG_G = 0.55. In trace_ray the phase
 * value is computed ONCE per pixel — it depends only on the (eye, sun)
 * angle.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * hg_phase — HG phase value for cos(angle between two directions).
 *
 * Pseudocode:
 *   g²    = HG_G · HG_G
 *   denom = 1 + g² − 2g·cosθ
 *   return (1 − g²) / (4π · denom^1.5)
 *
 * Returns a number ≥ 0; INTEGRATES to 1 over the full sphere — so
 * the value at any one angle is a DENSITY, not a probability.
 */
static inline float hg_phase(float cos_angle_eye_to_sun)
{
    float g  = HG_G;
    float g2 = g * g;
    float denom = 1.0f + g2 - 2.0f * g * cos_angle_eye_to_sun;
    if (denom < 1e-9f) denom = 1e-9f;
    return (1.0f - g2)
         / (4.0f * (float)M_PI * powf(denom, 1.5f));
}

/* ── §9 RNG (hash3 / hash01 — forest layout only) ───────────────────── *
 *
 * The path tracer is DETERMINISTIC (no Monte Carlo at runtime). The
 * only randomness is the per-seed forest layout (place_trees in §13).
 * hash3 is a small integer hash; hash01 maps its high bits to [0, 1).
 * Pure functions — no global state.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * hash3 — mix three integer keys into a 32-bit hash. Three large
 * odd primes followed by xorshift-mul rounds. Standard recipe.
 */
static inline uint32_t hash3(int kx, int ky, int kz)
{
    uint32_t h = (uint32_t)kx * 73856093u
               ^ (uint32_t)ky * 19349663u
               ^ (uint32_t)kz * 83492791u;
    h ^= h >> 16; h *= 0x85ebca6bu;
    h ^= h >> 13; h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/*
 * hash01 — uniform float in [0, 1). Drops the top byte of a 32-bit
 * hash to avoid the rounding-to-1 issue on extreme values.
 */
static inline float hash01(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* ── §10 ncurses paint ───────────────────────────────────────────────── *
 *
 * One cell = (cube colour pair, glyph from ramp, A_BOLD / A_DIM /
 * A_NORMAL). Tone-map happens HERE — single application per cell.
 *
 * ─────────────────────────────────────────────────────────────────── */

static int g_have_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_have_256 = (COLORS >= 256);
    if (g_have_256) {
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
        init_pair(PAIR_HUD,          COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,         COLOR_CYAN,   -1);
    }
}

/*
 * paint_cell — tone-map and paint one cell.
 *
 * Pseudocode:
 *   r' = gamma(reinhard(c.r))                   per channel
 *   g' = gamma(reinhard(c.g))
 *   b' = gamma(reinhard(c.b))
 *   luma  = 0.2126 r' + 0.7152 g' + 0.0722 b'   Rec.601
 *   glyph = k_ramp[ round(luma · (RAMP_LEN-1)) ]
 *   if 256 colour:
 *     (r5, g5, b5) = round((r', g', b') · 5)
 *     pair = PAIR_CUBE_BASE + r5·36 + g5·6 + b5
 *     attr = A_BOLD if luma > 0.85 else A_DIM if luma < 0.15 else NORMAL
 *   else:
 *     pair = PAIR_SUN_FALLBACK
 *   draw glyph at (sx, sy) with pair + attr
 */
static void paint_cell(int sx, int sy, RGB col)
{
    float r = gamma_enc(reinhard(col.r));
    float g = gamma_enc(reinhard(col.g));
    float b = gamma_enc(reinhard(col.b));
    float luma = 0.2126f*r + 0.7152f*g + 0.0722f*b;
    int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
    if (ri < 0)         ri = 0;
    if (ri >= RAMP_LEN) ri = RAMP_LEN - 1;

    if (g_have_256) {
        int r5 = (int)(r * 5.f + 0.5f); if (r5 > 5) r5 = 5; if (r5 < 0) r5 = 0;
        int g5 = (int)(g * 5.f + 0.5f); if (g5 > 5) g5 = 5; if (g5 < 0) g5 = 0;
        int b5 = (int)(b * 5.f + 0.5f); if (b5 > 5) b5 = 5; if (b5 < 0) b5 = 0;
        int pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;
        int attr = (luma > 0.85f) ? A_BOLD
                 : (luma < 0.15f) ? A_DIM
                 :                  A_NORMAL;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(pair) | attr);
    } else {
        attron(COLOR_PAIR(PAIR_SUN_FALLBACK));
        mvaddch(sy, sx, (chtype)(unsigned char)k_ramp[ri]);
        attroff(COLOR_PAIR(PAIR_SUN_FALLBACK));
    }
}

/* ── §11 ray-plane intersection ─────────────────────────────────────── *
 *
 * Ray vs horizontal plane y = const. Used by the ground (GROUND_Y = 0).
 * The plane is solid from above only.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ray_plane_y — intersect ray with horizontal plane at y = plane_y.
 *
 * Pseudocode:
 *   if ray_dir.y > -ε:               ray going up or parallel → MISS
 *   t = (plane_y − ray_origin.y) / ray_dir.y
 *   if t < ε:                        behind us → MISS
 *   *out_t = t; HIT
 *
 * The `ray_dir.y > -ε' guard means we register hits ONLY where the
 * ray is heading DOWN through the plane. If the camera is above the
 * plane and the ray goes up, no hit.
 */
static bool ray_plane_y(V3 ray_origin, V3 ray_dir,
                        float plane_y, float *out_t)
{
    if (ray_dir.y > -1e-6f) return false;
    float t = (plane_y - ray_origin.y) / ray_dir.y;
    if (t < 1e-3f) return false;
    *out_t = t;
    return true;
}

/* ── §12 ray-cylinder intersection (T6) ─────────────────────────────── *
 *
 * Ray vs vertical finite-height cylinder (a tree trunk).
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ray_cylinder — ray vs vertical cylinder of radius r, base at
 *                `base', finite height h.
 *
 * Pseudocode (T6 derivation):
 *   δx = ray_origin.x − base.x        offset on x
 *   δz = ray_origin.z − base.z        offset on z
 *   a  = ray_dir.x² + ray_dir.z²       quadratic coefficients...
 *   if a < ε:  ray purely vertical → MISS
 *   b  = 2·(δx·ray_dir.x + δz·ray_dir.z)
 *   c  = δx² + δz² − r²
 *   disc = b² − 4ac
 *   if disc < 0:  no real roots → MISS
 *   √disc; t0 = (-b - √)/2a;   t1 = (-b + √)/2a
 *   if t1 < ε:  cylinder behind us → MISS
 *
 *   try near root first; if its y is outside [base.y, base.y + h]
 *   try far root; if that's also outside, MISS.
 *
 *   *out_t = the valid hit
 *
 * The "try far root if near misses height range" handles the case
 * where the ray enters the side ABOVE the cylinder's top and exits
 * below — physically inside the cylinder column but missing the
 * trunk because the trunk is shorter than the column.
 */
static bool ray_cylinder(V3 ray_origin, V3 ray_dir,
                         V3 base, float r, float h,
                         float *out_t)
{
    float dx = ray_origin.x - base.x;
    float dz = ray_origin.z - base.z;
    float a  = ray_dir.x * ray_dir.x + ray_dir.z * ray_dir.z;
    if (a < 1e-9f) return false;
    float b  = 2.0f * (dx * ray_dir.x + dz * ray_dir.z);
    float c  = dx * dx + dz * dz - r * r;
    float disc = b * b - 4.0f * a * c;
    if (disc < 0) return false;
    float sq = sqrtf(disc);
    float t_near = (-b - sq) / (2.0f * a);
    float t_far  = (-b + sq) / (2.0f * a);
    if (t_far < 1e-3f) return false;

    /* Try near intersection first; fall back to far if near is
     * outside the height range (e.g. ray crosses the cylinder
     * column above or below the trunk's top/base). */
    float t = (t_near >= 1e-3f) ? t_near : t_far;
    float y_at = ray_origin.y + t * ray_dir.y;
    if (y_at < base.y || y_at > base.y + h) {
        if (t == t_near) {
            t = t_far;
            y_at = ray_origin.y + t * ray_dir.y;
            if (y_at < base.y || y_at > base.y + h) return false;
        } else {
            return false;
        }
    }
    *out_t = t;
    return true;
}

/* ── §13 scene layout — trees + sun-block test ──────────────────────── *
 *
 *   §13.1  Tree struct + globals
 *   §13.2  place_trees       — jittered grid + probabilistic gaps
 *   §13.3  HitType / Hit
 *   §13.4  scene_blocked_to_sun — NEE shadow-ray block test
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §13.1 ── Tree struct + globals ───────────────────────────────────── */

typedef struct {
    V3    base;          /* (x, 0, z) — rooted on ground plane          */
    float radius;
    float height;
} Tree;

typedef enum { HIT_SKY = 0, HIT_TREE = 1, HIT_GROUND = 2 } HitType;

typedef struct {
    HitType type;
    float   t;
    int     tree_idx;        /* valid iff type == HIT_TREE              */
} Hit;

static Tree g_trees[N_TREES];
static int  g_n_trees_placed;

/* §13.2 ── place_trees: jittered grid layout ───────────────────────── */

/*
 * place_trees — populate g_trees[] using a jittered grid with
 *               probabilistic per-cell empties (T7).
 *
 * Pseudocode:
 *   idx = 0
 *   for each (azim_bin, depth_bin) cell:
 *     fill = hash01(hash3(ax, dz, seed^FILL))
 *     if fill > TREE_FILL_PROB: skip (empty cell)
 *     z   = log-uniform within cell's depth range
 *     az  = uniform within cell's azimuth range
 *     x   = z · tan(az)
 *     r   = uniform [TREE_R_MIN, TREE_R_MAX]
 *     h   = uniform [TREE_H_MIN, TREE_H_MAX]
 *     g_trees[idx++] = Tree{base=(x, 0, z), r, h}
 *   g_n_trees_placed = idx
 *
 * Five separate hash calls per cell so all draws are statistically
 * independent (a single hash for "this cell" would correlate the
 * draws and produce visible patterns).
 */
static void place_trees(int seed)
{
    int idx = 0;
    for (int dz = 0; dz < N_DEPTH_BINS; dz++) {
        for (int ax = 0; ax < N_AZIM_BINS; ax++) {
            uint32_t fill_hash = hash3(ax, dz, seed ^ 0x05000);
            float    fill      = hash01(fill_hash);
            if (fill > TREE_FILL_PROB) continue;

            uint32_t z_hash = hash3(ax, dz, seed ^ 0x10000);
            uint32_t a_hash = hash3(ax, dz, seed ^ 0x20000);
            uint32_t r_hash = hash3(ax, dz, seed ^ 0x30000);
            uint32_t h_hash = hash3(ax, dz, seed ^ 0x40000);

            /* Log-spaced depth bin: closer cells span small ranges,
             * far cells span large ranges. */
            float u_z0 = (float) dz      / (float)N_DEPTH_BINS;
            float u_z1 = (float)(dz + 1) / (float)N_DEPTH_BINS;
            float z0   = TREE_Z_MIN * powf(TREE_Z_MAX / TREE_Z_MIN, u_z0);
            float z1   = TREE_Z_MIN * powf(TREE_Z_MAX / TREE_Z_MIN, u_z1);
            float jz   = hash01(z_hash);
            float z    = z0 + jz * (z1 - z0);

            /* Linear azimuth bin within ±AZIMUTH_HALF. */
            float u_a0 = ((float)(ax)     / (float)N_AZIM_BINS - 0.5f) * 2.0f;
            float u_a1 = ((float)(ax + 1) / (float)N_AZIM_BINS - 0.5f) * 2.0f;
            float ja   = hash01(a_hash);
            float u_a  = u_a0 + ja * (u_a1 - u_a0);
            float angle = u_a * AZIMUTH_HALF;
            float x     = z * tanf(angle);

            float u_r = hash01(r_hash);
            float u_h = hash01(h_hash);

            g_trees[idx].base   = v3(x, 0.0f, z);
            g_trees[idx].radius = TREE_R_MIN + u_r * (TREE_R_MAX - TREE_R_MIN);
            g_trees[idx].height = TREE_H_MIN + u_h * (TREE_H_MAX - TREE_H_MIN);
            idx++;
        }
    }
    g_n_trees_placed = idx;
}

/* §13.4 ── scene_blocked_to_sun: NEE shadow-ray block test ─────────── */

/*
 * scene_blocked_to_sun — does any tree intersect the chord from
 *                        scatter_point in direction sun_dir, within
 *                        max_distance? (T5)
 *
 * Pseudocode:
 *   for each tree:
 *     if ray_cylinder(p, sun_dir, tree.base, tree.r, tree.h)
 *        gives 1e-3 < t < max_distance:
 *           return true  (blocked)
 *   return false  (clear path to sun)
 *
 * Iterates EVERY tree — brute-force O(N_trees) per call. With ~14
 * trees this is fine; a real renderer would put a 2D BVH on the
 * trees but the speedup at this scale isn't worth the complexity.
 *
 * Used by:
 *   §16.1  shade_ground   for direct-sun shadow on the ground
 *   §17    trace_ray      at every march step (the in-scatter NEE)
 */
static bool scene_blocked_to_sun(V3 scatter_point, V3 sun_dir,
                                 float max_distance)
{
    for (int i = 0; i < g_n_trees_placed; i++) {
        float t_tree;
        if (ray_cylinder(scatter_point, sun_dir,
                         g_trees[i].base, g_trees[i].radius,
                         g_trees[i].height, &t_tree)) {
            if (t_tree > 1e-3f && t_tree < max_distance) return true;
        }
    }
    return false;
}

/* ── §14 scene_hit (find nearest of {tree, ground, sky}) ────────────── */

/*
 * scene_hit — return the nearest scene hit.
 *
 * Pseudocode:
 *   start with HIT_SKY, t = INF
 *   try ray_plane_y vs ground   → if closer, set HIT_GROUND
 *   for each tree:
 *     try ray_cylinder          → if closer, set HIT_TREE
 *   return best
 *
 * Sky is the FALLBACK — represents "the ray exits the scene without
 * hitting tree or ground". Always reaches t = INF in that case.
 */
static Hit scene_hit(V3 ray_origin, V3 ray_dir)
{
    Hit hit = { HIT_SKY, 1e30f, -1 };

    /* Ground. */
    float t_ground;
    if (ray_plane_y(ray_origin, ray_dir, GROUND_Y, &t_ground)) {
        if (t_ground < hit.t) {
            hit.type     = HIT_GROUND;
            hit.t        = t_ground;
            hit.tree_idx = -1;
        }
    }

    /* Trees. */
    for (int i = 0; i < g_n_trees_placed; i++) {
        float t_tree;
        if (ray_cylinder(ray_origin, ray_dir, g_trees[i].base,
                         g_trees[i].radius, g_trees[i].height, &t_tree)) {
            if (t_tree < hit.t) {
                hit.type     = HIT_TREE;
                hit.t        = t_tree;
                hit.tree_idx = i;
            }
        }
    }

    return hit;
}

/* ── §15 sky / sun radiance (T8) ────────────────────────────────────── *
 *
 * Two components:
 *   1. Vertical gradient from horizon (warm, kelvin-tinted) to
 *      zenith (cool blue).
 *   2. Sun disc — additive bright spot when the ray points within
 *      SUN_ANG_RADIUS of sun_dir.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * sky_radiance — radiance from a ray pointed at the sky.
 *
 * Pseudocode:
 *   h     = clamp01(ray_dir.y)                  0 = horizon, 1 = zenith
 *   sky   = lerp(horizon_col, ZENITH, h)
 *   if cos(ray_dir, sun_dir) > cos(SUN_ANG_RADIUS):
 *     edge = ramp from 0 at edge to 1 at centre
 *     sky += sun_em · edge
 *   return sky
 */
static RGB sky_radiance(V3 ray_dir, V3 sun_dir, RGB sun_em, RGB horizon_col)
{
    float h = ray_dir.y;
    if (h < 0.f) h = 0.f;
    if (h > 1.f) h = 1.f;

    RGB zenith = rgb_make(SKY_ZENITH_R, SKY_ZENITH_G, SKY_ZENITH_B);
    RGB sky    = rgb_lerp(horizon_col, zenith, h);

    float cos_ray_to_sun = v3_dot(ray_dir, sun_dir);
    float cos_disc_edge  = cosf(SUN_ANG_RADIUS);
    if (cos_ray_to_sun > cos_disc_edge) {
        float t_edge = (cos_ray_to_sun - cos_disc_edge)
                     / (1.0f - cos_disc_edge);
        if (t_edge > 1.f) t_edge = 1.f;
        sky = rgb_add(sky, rgb_scl(sun_em, t_edge));
    }
    return sky;
}

/* ── §16 surface shading ─────────────────────────────────────────────── *
 *
 * Two BRDFs:
 *   §16.1  shade_ground   Lambertian + NEE direct sun + sky ambient
 *   §16.2  shade_tree     constant near-black wood
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §16.1 ── shade_ground ────────────────────────────────────────────── */

/*
 * shade_ground — Lambertian floor with NEE direct sun + sky ambient.
 *
 * Pseudocode:
 *   albedo  = (GROUND_R, GROUND_G, GROUND_B)
 *   cos_NL  = max(0, sun_dir.y)                  ground normal = +Y
 *   direct  = (cos_NL > 0 and not blocked-to-sun)
 *               ? sun_em · albedo · cos_NL / π
 *               : 0
 *   ambient = horizon_col · albedo · 0.25         faint sky term
 *   return direct + ambient
 *
 * The block-to-sun test uses scene_blocked_to_sun — the ground stays
 * in its own shadow zone where any tree blocks the chord-to-sun.
 */
static RGB shade_ground(V3 ground_point, V3 sun_dir,
                        RGB sun_em, RGB horizon_col)
{
    RGB albedo = rgb_make(GROUND_ALBEDO_R, GROUND_ALBEDO_G, GROUND_ALBEDO_B);
    float cos_normal_to_sun = sun_dir.y;
    if (cos_normal_to_sun < 0.f) cos_normal_to_sun = 0.f;

    RGB direct = rgb_make(0.f, 0.f, 0.f);
    if (cos_normal_to_sun > 0.f) {
        bool blocked = scene_blocked_to_sun(ground_point, sun_dir, SUN_DIST);
        if (!blocked) {
            float lambert = cos_normal_to_sun / (float)M_PI;
            direct = rgb_scl(rgb_mul(sun_em, albedo), lambert);
        }
    }
    RGB ambient = rgb_scl(rgb_mul(horizon_col, albedo), 0.25f);
    return rgb_add(direct, ambient);
}

/* §16.2 ── shade_tree ───────────────────────────────────────────────── */

/*
 * shade_tree — constant near-black wood colour.
 *
 * Trunks are deliberately very dark so they read as silhouettes
 * against the bright shafts behind them. A more sophisticated
 * version could add subtle rim lighting from earthshine on the lit
 * side, but the silhouette is what matters for the demo.
 */
static RGB shade_tree(int tree_idx)
{
    (void)tree_idx;
    return rgb_make(0.025f, 0.020f, 0.015f);
}

/* ── §17 trace_ray (THE CORE — volumetric path tracer) ──────────────── *
 *
 * One pixel's worth of light, computed by:
 *
 *   1. Find what the eye ray hits  → march_distance (Stage 2 of T3)
 *   2. Loop MARCH_STEPS times, accumulating in-scatter (Stages 3-6)
 *   3. Add surface contribution dimmed by remaining transmittance
 *      (Stage 7)
 *
 * Returns a Radiance struct with all components separated, so the
 * debug overlays in §20 can paint each on its own. The default
 * MODE_NORMAL display sums them as in T1's equation.
 *
 * Read the inner loop body carefully — every line is one of the
 * seven stages of T3. ────────────────────────────────────────────── */

typedef struct {
    RGB   total;                /* in_scatter + surface · T_far     */
    RGB   in_scatter;           /* in-scatter accumulator only      */
    RGB   surface;              /* surface radiance (un-dimmed)     */
    float t_far_transmittance;  /* T(0, march_distance)             */
} Radiance;

/*
 * trace_ray — volumetric path tracer for one eye ray.
 *
 * Pseudocode (mirrors the inner loop body 1:1):
 *   hit = scene_hit(ray)                         Stage 2
 *   march_distance = (hit type)
 *                    ? hit.t
 *                    : FAR_CLIP   (sky)
 *   step_length = march_distance / MARCH_STEPS
 *   phase_value = hg_phase(cos(ray_dir, sun_dir))  Stage 4 (constant)
 *   transmittance_along_eye = 1
 *   in_scatter = (0, 0, 0)
 *   for i = 0 .. MARCH_STEPS-1:
 *     if transmittance_along_eye < 1e-3: break   early-out
 *     scatter_point = ray + ((i + 0.5) · step_length)
 *     extinction    = sigma_e_at(scatter_point)
 *     if extinction > 0:
 *       if NOT scene_blocked_to_sun(scatter_point, ...):     Stage 4
 *         tr_to_sun = closed-form transmittance (§7.2)        Stage 5
 *         step_factor = extinction · phase_value
 *                     · tr_to_sun · INSCATTER_GAIN · step_length
 *         in_scatter += sun_em · step_factor
 *                     · transmittance_along_eye
 *       transmittance_along_eye *= exp(-extinction · step_length) Stage 6
 *   surface_radiance = shade_(tree|ground|sky)(...)            Stage 7
 *   total = in_scatter + surface_radiance · transmittance_along_eye
 *   return Radiance{total, in_scatter, surface_radiance,
 *                   transmittance_along_eye}
 *
 * Iterative, not recursive — the for-loop is the integrator. The
 * NEE check inside the inner block is the entire shafts trick.
 */
static Radiance trace_ray(V3 ray_origin, V3 ray_dir,
                          V3 sun_dir, RGB sun_em, RGB horizon_col)
{
    /* Stage 2 — scene intersection. */
    Hit scene_hit_record = scene_hit(ray_origin, ray_dir);

    /* march_distance: surface hit, or FAR_CLIP for a sky-bound ray.
     * (Sky-bound rays still march so we accumulate scatter through
     * the whole near-camera mist column.) */
    float march_distance = (scene_hit_record.type == HIT_SKY)
                         ? FAR_CLIP
                         : scene_hit_record.t;
    if (march_distance > FAR_CLIP) march_distance = FAR_CLIP;

    /* Stage 3 — march setup. */
    float step_length    = march_distance / (float)MARCH_STEPS;
    float cos_eye_to_sun = v3_dot(ray_dir, sun_dir);
    float phase_value    = hg_phase(cos_eye_to_sun);

    RGB   in_scatter_radiance      = rgb_make(0.f, 0.f, 0.f);
    float transmittance_along_eye  = 1.0f;

    /* Stages 3-6 — march loop. */
    for (int i = 0; i < MARCH_STEPS; i++) {
        if (transmittance_along_eye <= 1e-3f) break;       /* early out */

        float step_t        = ((float)i + 0.5f) * step_length;
        V3    scatter_point = v3_add(ray_origin, v3_scl(ray_dir, step_t));
        float extinction    = sigma_e_at(scatter_point);

        if (extinction > 1e-6f) {
            /* Stage 4 — NEE: any tree blocks the chord to the sun? */
            bool blocked = scene_blocked_to_sun(scatter_point, sun_dir,
                                                SUN_NEE_DIST);

            if (!blocked) {
                /* Stage 5 — in-scatter contribution. */
                float transmittance_to_sun = tr_to_sun(scatter_point, sun_dir);
                float step_contribution    = extinction * phase_value
                                            * transmittance_to_sun
                                            * INSCATTER_GAIN
                                            * step_length;
                RGB add = rgb_scl(rgb_scl(sun_em, step_contribution),
                                  transmittance_along_eye);
                in_scatter_radiance = rgb_add(in_scatter_radiance, add);
            }

            /* Stage 6 — eye-ray transmittance update. */
            transmittance_along_eye *= expf(-extinction * step_length);
        }
    }

    /* Stage 7 — surface contribution at the far end. */
    RGB surface_radiance;
    switch (scene_hit_record.type) {
    case HIT_TREE:
        surface_radiance = shade_tree(scene_hit_record.tree_idx);
        break;
    case HIT_GROUND: {
        V3 ground_point = v3_add(ray_origin,
                                 v3_scl(ray_dir, scene_hit_record.t));
        surface_radiance = shade_ground(ground_point, sun_dir,
                                        sun_em, horizon_col);
        break;
    }
    case HIT_SKY:
    default:
        surface_radiance = sky_radiance(ray_dir, sun_dir,
                                        sun_em, horizon_col);
        break;
    }

    /* Final radiance = in-scatter + surface · remaining T. */
    RGB total = rgb_add(in_scatter_radiance,
                        rgb_scl(surface_radiance, transmittance_along_eye));

    Radiance result = {
        .total                 = total,
        .in_scatter            = in_scatter_radiance,
        .surface               = surface_radiance,
        .t_far_transmittance   = transmittance_along_eye,
    };
    return result;
}

/* ── §18 scene state + sun motion ───────────────────────────────────── */

typedef struct {
    bool      paused;
    int       speed;
    int       kelvin_idx;
    float     zoom;
    float     time_secs;
    int       seed;
    DebugMode debug_mode;
} Scene;

/*
 * scene_sun_dir — current sun direction (unit vector, points FROM
 *                 any scene point TOWARD the sun) for time t.
 *
 * Pseudocode:
 *   ω_el = 2π / SUN_EL_PERIOD_S
 *   ω_az = 2π / SUN_AZ_PERIOD_S
 *   el   = SUN_EL_BASE + SUN_EL_AMP · sin(ω_el · t)
 *   az   = SUN_AZ_BASE + SUN_AZ_AMP · sin(ω_az · t)
 *   return norm(sin(az)·cos(el), sin(el), cos(az)·cos(el))
 *
 * For this scene the sun is always above the horizon
 * (el_min = SUN_EL_BASE − SUN_EL_AMP > 0).
 */
static V3 scene_sun_dir(const Scene *s)
{
    float omega_el = 2.0f * (float)M_PI / SUN_EL_PERIOD_S;
    float omega_az = 2.0f * (float)M_PI / SUN_AZ_PERIOD_S;
    float el = SUN_EL_BASE + SUN_EL_AMP * sinf(omega_el * s->time_secs);
    float az = SUN_AZ_BASE + SUN_AZ_AMP * sinf(omega_az * s->time_secs);
    float ce = cosf(el);
    return v3_norm(v3(sinf(az) * ce, sinf(el), cosf(az) * ce));
}

/*
 * scene_reseed — pick a new seed and re-place trees.
 *
 * Mixes the current time into the new seed so successive presses
 * of `r' don't repeat layouts.
 */
static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f), s->seed, 0xC0FFEE);
    s->seed = (int)(h ^ 0x5A5A5A5Au);
    place_trees(s->seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused     = false;
    s->speed      = SPEED_DEF;
    s->kelvin_idx = 1;     /* GOLDEN by default */
    s->zoom       = 1.0f;
    s->seed       = 0xF0E57u;
    s->debug_mode = MODE_NORMAL;
    place_trees(s->seed);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* ── §19 camera (pinhole, no pitch) ─────────────────────────────────── */

typedef struct {
    V3    pos;
    float fov_h, fov_v;
    int   cols, rows;
} Camera;

/*
 * camera_make — set up a horizontally-looking pinhole camera.
 *   pos       = (0, CAMERA_HEIGHT, 0)
 *   fov_v     = fov_h · rows · ASPECT_Y / cols
 *
 * Looking straight down +Z with no pitch — the sun drifts naturally
 * through the upper part of the frame.
 */
static void camera_make(Camera *c, int cols, int rows, float fov_h)
{
    c->cols  = cols;
    c->rows  = rows;
    c->pos   = v3(0.0f, CAMERA_HEIGHT, 0.0f);
    c->fov_h = fov_h;
    c->fov_v = fov_h * (float)rows * ASPECT_Y / (float)cols;
}

/*
 * camera_ray — pixel (sx, sy) → world-space ray direction.
 *
 * Pseudocode:
 *   u =  ((2·sx + 1) − cols) / cols  ·  fov_h         normalised x
 *   v = -((2·sy + 1) − rows) / rows  ·  fov_v         normalised y (flipped)
 *   return normalize(u, v, 1)
 *
 * The flip on v converts from "screen y down" to "world y up".
 */
static V3 camera_ray(const Camera *c, int sx, int sy)
{
    float u = ( (2.0f * (float)sx + 1.0f) - (float)c->cols)
            / (float)c->cols * c->fov_h;
    float v = -((2.0f * (float)sy + 1.0f) - (float)c->rows)
            / (float)c->rows * c->fov_v;
    return v3_norm(v3(u, v, 1.0f));
}

/* ── §20 screen + scene_draw (with debug overlays) ──────────────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
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

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static RGB g_buf[BUF_MAX_H][BUF_MAX_W];

/*
 * scene_draw — render one frame.
 *
 * Pseudocode:
 *   for each pixel (r, c):
 *     ray   = camera_ray(c, r)
 *     rad   = trace_ray(...)
 *     g_buf[r][c] = SELECT(scene.debug_mode, rad)
 *   for each pixel: paint_cell(c, r + offset, g_buf[r][c])
 *
 * The SELECT step is what makes the debug overlays cheap — trace_ray
 * has already produced all four components, we just decide which
 * one(s) to display.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int rows_eff = sc->rows - 2;
    int row_off  = 1;
    if (rows_eff < 4) { rows_eff = sc->rows; row_off = 0; }
    if (rows_eff > BUF_MAX_H) rows_eff = BUF_MAX_H;
    int cols_eff = sc->cols;
    if (cols_eff > BUF_MAX_W) cols_eff = BUF_MAX_W;

    Camera cam;
    camera_make(&cam, cols_eff, rows_eff, FOV_H_BASE / s->zoom);

    V3   sun_dir   = scene_sun_dir(s);
    float kelvin   = KELVINS[s->kelvin_idx].kelvin;
    RGB sun_chrom  = blackbody_rgb(kelvin);
    RGB sun_em     = rgb_scl(sun_chrom, SUN_EMIT_HDR);
    RGB horizon    = rgb_scl(sun_chrom, SKY_HORIZON_FAC);

    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            V3       ray_dir = camera_ray(&cam, c, r);
            Radiance rad     = trace_ray(cam.pos, ray_dir, sun_dir,
                                         sun_em, horizon);

            /* Debug-overlay selection (T11). */
            switch (s->debug_mode) {
            default:
            case MODE_NORMAL:
                g_buf[r][c] = rad.total;
                break;
            case MODE_SCATTER:
                g_buf[r][c] = rad.in_scatter;
                break;
            case MODE_SURFACE:
                g_buf[r][c] = rad.surface;
                break;
            case MODE_TR: {
                float t = rad.t_far_transmittance;
                g_buf[r][c] = rgb_make(t, t, t);
                break;
            }
            }
        }
    }

    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols_eff; c++) {
            paint_cell(c, r + row_off, g_buf[r][c]);
        }
    }
}

/* ── §21 HUD ────────────────────────────────────────────────────────── */

/*
 * hud_draw — three HUD elements:
 *   row 0 right    yellow status (fps, Hz, paused, kelvin, sun
 *                  el/az deg, zoom, speed, debug mode)
 *   row 0 left     yellow title (subtitle banner)
 *   bottom row     cyan key-hint strip (BOLD, ASCII only)
 */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    const KelvinPreset *k = &KELVINS[s->kelvin_idx];
    V3    sd     = scene_sun_dir(s);
    float el_deg = asinf(sd.y) * 180.0f / (float)M_PI;
    float az_deg = atan2f(sd.x, sd.z) * 180.0f / (float)M_PI;

    char buf[256];
    snprintf(buf, sizeof buf,
             " %5.1f fps %3d Hz  %s  %s %5.0fK  sun:el%4.1f deg az%+5.1f deg  z:%3.1fx  spd:%d  %s ",
             fps, sim_fps,
             s->paused ? "PAUSED" : "      ",
             k->name, (double)k->kelvin,
             (double)el_deg, (double)az_deg,
             (double)s->zoom, s->speed,
             debug_mode_name(s->debug_mode));
    int len = (int)strlen(buf);
    if (len > sc->cols) len = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, " FOREST-GOD-RAYS · VOLUMETRIC PT ");
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reseed  d:debug  t/T:kelvin  z/Z:zoom  +/-:spd  []:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §22 app (signals, main loop) ───────────────────────────────────── */

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
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused; break;
    case 'r': case 'R': scene_reseed(s); break;

    case 'd': case 'D':
        s->debug_mode = (DebugMode)((s->debug_mode + 1) % MODE_N);
        break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        break;
    case '-': case '_':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->kelvin_idx = (s->kelvin_idx + 1) % N_KELVINS;
        break;
    case 'T':
        s->kelvin_idx = (s->kelvin_idx + N_KELVINS - 1) % N_KELVINS;
        break;

    case 'z':
        s->zoom *= ZOOM_STEP;
        if (s->zoom > ZOOM_MAX) s->zoom = ZOOM_MAX;
        break;
    case 'Z':
        s->zoom /= ZOOM_STEP;
        if (s->zoom < ZOOM_MIN) s->zoom = ZOOM_MIN;
        break;
    }
    return true;
}

static void app_init(App *app)
{
    memset(app, 0, sizeof *app);
    scene_init(&app->scene);
    screen_init(&app->screen);
    app->sim_fps = SIM_FPS_DEFAULT;
    app->running = 1;
}

static void app_run(App *app)
{
    int64_t prev = clock_ns();
    int64_t sim_accum = 0;
    int64_t frame_count = 0;
    int64_t fps_window_start = prev;
    double  fps_meas = 0.0;

    struct sigaction sa = {0};
    sa.sa_handler = on_exit_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sa.sa_handler = on_resize_signal;
    sigaction(SIGWINCH, &sa, NULL);
    atexit(cleanup);

    while (app->running) {
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) { app->running = 0; break; }
        }
        if (!app->running) break;
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->need_resize = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - prev;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        prev = now;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, (float)tick_ns / (float)NS_PER_SEC);
            sim_accum -= tick_ns;
        }

        screen_draw(&app->screen, &app->scene, fps_meas, app->sim_fps);
        screen_present();

        frame_count++;
        if (now - fps_window_start >= NS_PER_SEC) {
            fps_meas = (double)frame_count
                     * (double)NS_PER_SEC
                     / (double)(now - fps_window_start);
            frame_count = 0;
            fps_window_start = now;
        }

        int64_t target = clock_ns();
        int64_t left   = TICK_NS(SIM_FPS_DEFAULT * 2) - (target - now);
        if (left > 0) clock_sleep_ns(left);
    }
}

int main(void)
{
    app_init(&g_app);
    app_run(&g_app);
    cleanup();
    return 0;
}
