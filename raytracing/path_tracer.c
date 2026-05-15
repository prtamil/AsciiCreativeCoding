/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * path_tracer.c — progressive Monte Carlo path tracer · Cornell Box
 *
 * DEMO: A Cornell Box (red left wall, green right wall, white floor/
 *       ceiling/back, warm light overhead, gold + indigo spheres)
 *       slowly resolves from random noise into a clean image as
 *       samples accumulate. Color bleeding makes the gold sphere
 *       blush red on its left flank and the indigo sphere green on
 *       its right — global illumination you can see materialise in
 *       real time. Cycle four debug overlays
 *       (PT → NORMAL → ALBEDO → DEPTH) to see what each early stage
 *       of the renderer contributes on its own.
 *
 *       This file is structured as the SEVEN CONCEPTUAL PASSES of
 *       path tracing (camera ray → intersection → shading decision →
 *       bounce sampling → throughput chain → termination → average +
 *       tone-map). Each pass lives in its own §-section and its own
 *       named function. Reading top-to-bottom you build the algorithm
 *       one pass at a time.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same scene primitives, NO bounces
 *                                     (direct phong only). Read this
 *                                     first to see what a single ray
 *                                     hit looks like before adding
 *                                     stochastic sampling.
 *   raytracing/cube_raytrace.c     — same skeleton, slab method.
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic.
 *
 * Section map (the spine — 7 numbered passes):
 *   §1  config         — frame rate, depth/RR/SPP caps, shade-mode enum
 *   §2  clock          — monotonic timer + sleep
 *   §3  vec3           — V3 math
 *   §4  rng            — xorshift32 + per-pixel-per-frame seed
 *   §5  scene          — Cornell-box materials, quads, spheres
 *
 *   §6  PASS 1 — CAMERA RAY GENERATION
 *                       pixel (col, row) → world-space Ray
 *   §7  PASS 2 — SCENE INTERSECTION
 *                       Ray → nearest Hit (ray vs quad, ray vs sphere)
 *   §8  PASS 3 — SHADING DECISION
 *                       Hit → Material (emission + albedo lookup)
 *   §9  PASS 4 — BOUNCE DIRECTION SAMPLING
 *                       Hit normal → cosine-weighted random direction
 *   §10 PASS 5 — THROUGHPUT CHAIN
 *                       running product: throughput *= albedo
 *   §11 PASS 6 — TERMINATION
 *                       Russian roulette + depth cap
 *   §12 path_trace     — orchestrator: combines passes 2-6 in a loop
 *   §13 PASS 7 — ACCUMULATION + TONE MAPPING
 *                       N samples → average → ASCII display
 *   §14 debug overlays — first-hit visualisations (NORMAL/ALBEDO/DEPTH)
 *
 *   §15 screen         — color init, rgb→pair, HUD
 *   §16 app            — signals, resize, main loop
 *
 * Keys:
 *   r          reset accumulator (restart convergence from sample 0)
 *   p / SPC    pause / resume sampling
 *   d / D      cycle shade mode  (PT → NORMAL → ALBEDO → DEPTH → PT)
 *   + / =      more samples per frame  (faster convergence, lower fps)
 *   -          fewer samples per frame (higher fps, slower convergence)
 *   q / ESC    quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/path_tracer.c \
 *       -o path_tracer -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order as
 *      prose before touching code. Path tracing is a SHORT idea hiding
 *      behind dense math; the prose teaches the idea, the code
 *      implements it.
 *   2. §1 config — every constant has a unit-bearing comment, so a
 *      glance is the fastest tour of what's tunable.
 *   3. The 7 passes (§6 through §13). Each pass is small (~30-100
 *      lines). Read in order; each builds on the previous.
 *   4. §12 path_trace — the orchestrator that fuses passes 2-6 into
 *      one loop. Read AFTER you've seen each pass on its own.
 *   5. §14 debug overlays — read if you want to understand what each
 *      early pass produces in isolation.
 *   6. §2/§3/§4 + §15/§16 are infrastructure; skip on a first read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere. The file uses these
 *   consistently:
 *     ray.origin / ray.dir   the ray as (point, unit direction)
 *     hit.P / hit.N          hit point and surface normal (toward ray)
 *     throughput             surviving fraction of the photon's energy
 *     albedo / emission      from a Lambertian Material
 *     accum / samples        per-pixel running sum + total sample count
 *     rng_state              xorshift32 state, mutated by rng_f()
 *
 *   When a one-letter name appears it's a tight-loop index (`i`, `s`)
 *   or a math symbol whose meaning is on the line above.
 *
 * Background you need
 * ───────────────────
 *   - Mean-of-N-samples ≈ true mean (law of large numbers).
 *   - The integral ∫f dx as "average of f over the domain × domain
 *     size", and Monte Carlo as "sample a few points and average".
 *   - Vector dot product, normalisation, Cartesian basis.
 *   - The quadratic formula (for ray-sphere).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - The full BRDF zoo (we use Lambertian only).
 *   - Probability density measure theory — pretend densities are just
 *     "how often this direction gets picked" and the math works out.
 *   - Importance sampling beyond cosine-weighted hemispheres.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Unidirectional Monte Carlo path tracing — the
 *                  unbiased numerical solver for Kajiya's rendering
 *                  equation. Per pixel, cast a ray. At each surface
 *                  hit:
 *                    (1) if the surface is emissive, accumulate its
 *                        radiance times the path's surviving
 *                        throughput and terminate;
 *                    (2) otherwise, multiply throughput by surface
 *                        albedo, sample a random new direction from
 *                        the cosine-weighted hemisphere around the
 *                        surface normal, and recurse with the new ray.
 *                  Russian-roulette termination at depth ≥ RR_DEPTH:
 *                  survive with probability p = max(throughput); on
 *                  survival multiply throughput by 1/p (preserves
 *                  unbiasedness) — finite expected depth, no truncation.
 *
 *                  The seven passes named in the section map ARE the
 *                  algorithm. Memorise them and you can read any path
 *                  tracer (smallpt, pbrt, mitsuba, cycles).
 *
 * Data-structure : Static per-pixel accumulator
 *                    g_accum[MAX_H][MAX_W][3]   — sum of radiance
 *                                                  over ALL samples
 *                                                  ever cast at this
 *                                                  pixel.
 *                    g_samples                  — total samples count.
 *                    Display = g_accum / g_samples, tone-mapped.
 *                  Resetting accum_reset() zeroes both. The buffer IS
 *                  the convergence record — each frame just adds more
 *                  samples to the running sum.
 *
 * Rendering      : Reinhard tone-map L' = L/(1+L) compresses the
 *                  open-ended HDR radiance into [0, 1), then a 1/2.2
 *                  gamma encode produces sRGB-perceptual values.
 *                  Final RGB is quantised to xterm's 6×6×6 colour
 *                  cube (216 shades) and Bourke's 92-character
 *                  density ramp picks the glyph from luminance.
 *                  Four debug modes (cycled with `d`) short-circuit
 *                  the path tracer at the first hit and visualise the
 *                  output of an early pass on its own — pedagogically
 *                  this lets you SEE what each pass contributes.
 *
 * Performance    : Per pixel per frame: SPP paths × MAX_DEPTH bounces
 *                  × scene_hit(O(quads) + O(spheres)) intersections.
 *                  At 8 quads, 2 spheres, MAX_DEPTH=7, SPP=2 that's
 *                  ~140 intersections/pixel/frame on average (RR
 *                  ends most paths early). Modern CPU: ~5 ns each →
 *                  ~700 ns/pixel/frame. A 200×60 terminal is 12 000
 *                  cells, so ~8 ms shading per frame at SPP=2 —
 *                  comfortable at 30 Hz. SPP=8 stretches that to
 *                  ~32 ms — still ok.
 *
 * References     : Kajiya, "The Rendering Equation," SIGGRAPH '86.
 *                    The original paper that defines path tracing.
 *                  Beason, "smallpt: Global Illumination in 99 lines
 *                    of C++," http://www.kevinbeason.com/smallpt/.
 *                    The canonical compressed hello world. Reading
 *                    smallpt alongside this file is the fastest way
 *                    to see how dense vs explicit code can teach the
 *                    same algorithm.
 *                  Pharr, Jakob & Humphreys, "Physically Based
 *                    Rendering: From Theory to Implementation" 4e
 *                    (free online: pbr-book.org). Definitive
 *                    reference; chapters 13-14 cover Monte Carlo and
 *                    path tracing.
 *                  Malley, "A Shading Method for Computer Generated
 *                    Images," MS thesis, U. Utah, 1988. (Cosine-
 *                    weighted hemisphere sampling.)
 *                  Veach, "Robust Monte Carlo Methods for Light
 *                    Transport Simulation," PhD thesis, Stanford
 *                    1997. (Russian roulette, bidirectional methods.)
 *                  Goral et al., "Modeling the Interaction of Light
 *                    Between Diffuse Surfaces," SIGGRAPH '84. (The
 *                    original Cornell Box.)
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA — three concepts stacked in this order
 * ────────────────────────────────────────────────
 * Path tracing is THREE concepts on top of each other. If you
 * understand each in turn, the algorithm is obvious.
 *
 *   1. LIGHT TRANSPORT IS AN INTEGRAL.
 *      The brightness of a surface point is the SUM (integral) over
 *      every direction of the light arriving from that direction,
 *      weighted by how much of it the surface scatters toward the
 *      camera. Kajiya 1986 wrote this down as the rendering equation.
 *
 *   2. ANY INTEGRAL CAN BE ESTIMATED BY AVERAGING RANDOM SAMPLES.
 *      Pick N random directions, evaluate the integrand at each,
 *      average. The average converges to the true integral as N grows.
 *      That's Monte Carlo. The estimate's noise shrinks as 1/√N.
 *
 *   3. THE INTEGRAND IS RECURSIVE — SO AVERAGE RANDOM PATHS, NOT
 *      RANDOM DIRECTIONS.
 *      The light arriving from a direction is itself the answer to
 *      another rendering-equation problem at the surface that ray
 *      hits. So instead of sampling at one point, sample WHOLE
 *      RANDOM PATHS from camera to light. Each path = one Monte
 *      Carlo sample. Many random paths averaged = the converged
 *      image.
 *
 * That's the entire idea. The seven passes (§6–§13) are the
 * implementation; everything else (importance sampling, Russian
 * roulette, tone mapping) is an optimisation on top.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Pretend you're a photon flying BACKWARDS in time. Start at the
 * camera, fly out, hit a wall, bounce in a random direction, hit
 * another wall, bounce again, … until you eventually land on a light
 * (or give up). Multiply the colours of every wall you bounced off,
 * times the light's emission. That's ONE estimate of what colour
 * this pixel should be. Do thousands of these random walks per pixel
 * and average. The average converges to the true image.
 *
 * Or, in soap-opera terms — imagine a RUMOUR network. Each character
 * (surface) has a "story-attenuation" factor — the fraction of any
 * rumour they pass along. The light is the only character who STARTS
 * rumours. To find out what rumour reaches your eye, you walk
 * randomly back through the gossip chain until you reach the light,
 * multiplying the attenuation factor of every character you visit:
 *
 *      eye  ←──  red wall (×0.65 in red)  ←──  white floor (×0.73)
 *           ←──  ceiling (×0.73)   ←──  LIGHT (radiance 15)
 *
 * Final colour ≈ 15 · 0.73 · 0.73 · 0.65  in red (lots gets through),
 *                15 · 0.73 · 0.73 · 0.05  in green (most absorbed).
 *
 * That's COLOUR BLEEDING — the red of the wall stains the rumour as
 * it travels back to your eye. Run a few thousand random walks per
 * pixel and average; the noise smooths into a clean image.
 *
 *      ┌──────────── ceiling (white) ──────────┐
 *      │                  light                │
 *      │                  ▼ ▼ ▼                │
 *      │               ╲╲╲│╱╱╱                 │
 *      │ red          rays scatter        green│
 *      │ wall         ──────────          wall │
 *      │                                       │
 *      │             ●           ●             │
 *      │          gold        indigo           │
 *      └──────────── floor (white) ────────────┘
 *
 *               ↑     ↑
 *         camera at (0, 0.05, -1.5), looking +Z
 *
 * The camera sits at the open front face. Each terminal cell fires
 * SPP rays per frame. Those rays bounce around inside the box,
 * eventually reaching the light at the top — and the colours of
 * everything they touched on the way mix into the pixel.
 *
 * ALGORITHM IN STEPS  (per pixel per sample)
 * ──────────────────────────────────────────
 *  1. PASS 1: Build a primary ray from the camera through a JITTERED
 *     sub-pixel offset. Jitter gives free anti-aliasing.
 *  2. throughput = (1, 1, 1)        ← what's left of the photon
 *     col        = (0, 0, 0)        ← accumulated radiance
 *  3. Loop up to MAX_DEPTH bounces:
 *      a. PASS 2: find nearest surface hit. If miss → break (black).
 *      b. PASS 3: look up the material at the hit.
 *      c. If surface is EMISSIVE:
 *            col += throughput · emission ;  break.
 *      d. PASS 6: RUSSIAN ROULETTE (depth ≥ RR_DEPTH):
 *            p = max(throughput.r, .g, .b)
 *            if rng > p → break (path killed)
 *            else throughput /= p   (compensate killed paths)
 *      e. PASS 5: throughput = throughput · albedo
 *      f. PASS 4: pick a new direction from the cosine-weighted
 *         hemisphere around the surface normal.
 *      g. Push the ray origin by ε·N to avoid self-intersection.
 *  4. Return col.
 *  5. PASS 7: accum[pixel] += col;  samples += 1.
 *  6. Display = tone_map(accum / samples).
 *
 * KEY FORMULAS
 * ────────────
 *   Rendering equation (Kajiya 1986):
 *     L_o(p, ω_o) = L_e(p, ω_o) + ∫_Ω f_r(p, ω_i, ω_o) · L_i(p, ω_i)
 *                                          · (ω_i · n) dω_i
 *
 *   Lambertian BRDF:
 *     f_r = ρ / π       where ρ = albedo (in [0, 1]).
 *
 *   Cosine-weighted hemisphere PDF:
 *     p(ω) = (n · ω) / π
 *
 *   Monte Carlo estimate per bounce (one sample):
 *     L̂ = f_r · L_i · cosθ / p(ω)
 *        = (ρ/π) · L_i · cosθ / (cosθ/π)
 *        = ρ · L_i           ← cosθ and π cancel: weight is just ρ.
 *
 *   That cancellation is THE WHOLE REASON we use cosine-weighted
 *   hemisphere sampling. Naïve uniform sampling would leave a `cosθ`
 *   factor in the estimator that makes near-grazing samples noisy.
 *
 *   Malley's method for cosine-weighted samples (no rejection):
 *     r1, r2 ~ U[0, 1)
 *     φ = 2π · r1
 *     local ω = (cosφ · √r2,  sinφ · √r2,  √(1 − r2))
 *     world ω = onb(n) · local ω
 *
 *   Russian roulette (Veach):
 *     p = max(throughput.r, .g, .b)
 *     kill with prob (1 − p); on survival throughput *= 1/p
 *     ⇒ E[contribution] is unchanged → unbiased.
 *
 *   Reinhard tone map + gamma:
 *     L' = L / (1 + L)
 *     out = L'^(1/2.2)
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Path:  eye → floor → red wall → light
 *   Materials:
 *     floor   albedo (0.73, 0.73, 0.73)
 *     red     albedo (0.65, 0.05, 0.05)
 *     light   emission (15, 14, 11)
 *
 *   throughput evolution:
 *     start:        (1.00, 1.00, 1.00)
 *     after floor:  (0.73, 0.73, 0.73)
 *     after wall:   (0.73·0.65, 0.73·0.05, 0.73·0.05)
 *                 = (0.4745, 0.0365, 0.0365)
 *     light hit, contribution = throughput · emission:
 *                 = (0.4745·15, 0.0365·14, 0.0365·11)
 *                 = (7.12, 0.51, 0.40)
 *
 *   That is BRIGHT RED — exactly the colour bleeding visible on the
 *   white floor near the red wall. Other paths to the same pixel may
 *   bounce different ways and contribute different colours; the
 *   AVERAGE over thousands of samples is the converged image.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SELF-INTERSECTION. After a bounce we push the ray origin by
 *    1e-4 · N. Without the push, the next intersection finds the
 *    surface we just left at t ≈ 0 and the path stops one bounce
 *    short. The push must be along N (not into the surface).
 *  • DOUBLE-SIDED NORMAL. For a quad we pick the normal that faces
 *    the incoming ray. For a sphere we flip the outward normal if it
 *    points away from the ray. Forgetting either gives black sides
 *    or NaN bounces.
 *  • RR PROBABILITY. p = max channel, NOT mean — we want to keep
 *    paths that still carry SIGNIFICANT energy in any single channel.
 *    Mean would over-kill paths whose energy is concentrated in one
 *    band (deep red, etc).
 *  • FIRST RR DEPTH. Killing too early (depth = 0, 1) raises variance
 *    sharply because every path has roughly equal expected
 *    contribution. RR_DEPTH = 3 is the standard "hold off until
 *    bounces are getting dim" choice.
 *  • CONVERGENCE = √-LAW. Variance of an N-sample average scales as
 *    1/N, so noise ∝ 1/√N. To halve the visible noise you must
 *    QUADRUPLE the samples. ACCUM_CAP = 8192 is plenty for visual
 *    cleanliness; rendering past that just burns CPU.
 *  • DOMINANT LIGHT VIA BOUNCES ONLY. This implementation has NO
 *    direct-light sampling (no NEE). Light is found purely by random
 *    bounces lucking into the light's footprint. That's slow but
 *    pedagogically clean — every line of code is path tracing,
 *    nothing is "the next-event sampler". Adding NEE would cut
 *    convergence time ~5× but add another 80 lines.
 *  • DEBUG-MODE RESET. When you cycle the shade mode (`d`) we reset
 *    the accumulator. NORMAL/ALBEDO/DEPTH produce different output
 *    ranges than PT, so mixing them under the same average would
 *    give nonsense.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' then watch:
 *      sample 1     → almost solid noise, hint of bright spot at light
 *      sample 32    → silhouettes visible, lots of grain
 *      sample 256   → clean walls, slight grain on the spheres
 *      sample 2048  → near-converged, only subtle noise remains
 *  • Press 'd' once → NORMAL: the scene appears INSTANTLY in solid
 *    flat colours, each face a different RGB-encoded normal. (Pass 1
 *    + Pass 2 only.)
 *  • Press 'd' again → ALBEDO: the scene appears as flat material
 *    colours (red wall, green wall, white floor, gold sphere, etc).
 *    No shadows, no bounces — just the body colour of each surface.
 *    (Pass 1 + 2 + 3.)
 *  • Press 'd' again → DEPTH: brightness ∝ 1/(1+t). Closer surfaces
 *    brighter, distant ones darker. Gray-scale only. Sanity check
 *    that the camera + intersection passes produce sane distances.
 *    (Pass 1 + Pass 2.)
 *  • Press 'd' once more → PT: the noisy progressive path-traced
 *    image returns. Compare ALBEDO vs PT to see how much detail
 *    comes from indirect light.
 *  • COLOUR BLEED in PT: the gold sphere has a slightly REDDISH
 *    tint on its LEFT flank (bouncing off red wall) and slightly
 *    GREENISH tint on its RIGHT (off green wall). Same for indigo.
 *  • SOFT SHADOWS in PT: the floor under each sphere is slightly
 *    darker than surrounding floor (less direct illumination from
 *    the area light reaches there).
 *  • CEILING NEAR LIGHT in PT: the parts of the ceiling adjacent to
 *    the light quad are slightly brighter than the corners (bounce
 *    light from the floor warming the ceiling indirectly).
 *  • Increasing SPP with '+' should make the image converge visibly
 *    faster but the per-frame fps drops proportionally.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Twelve short tutorials that build path tracing from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  The rendering equation — what we are solving
 *   T2  Monte Carlo: averaging random samples to estimate an integral
 *   T3  THE 7 PASSES — the universal vocabulary of path tracing
 *   T4  The backward photon — pretending we're a photon in reverse
 *   T5  Lambertian BRDF and the cosθ weight
 *   T6  Cosine-weighted hemisphere sampling — the magical cancellation
 *   T7  Malley's method — uniform-disk → hemisphere with no rejection
 *   T8  The throughput chain — how colour bleeding emerges
 *   T9  Russian roulette — terminating without bias
 *   T10 Progressive accumulation and the √N convergence law
 *   T11 The Cornell Box — what this scene specifically tests
 *   T12 Four debug overlays — PT vs NORMAL vs ALBEDO vs DEPTH
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE RENDERING EQUATION — WHAT WE ARE SOLVING
 * ─────────────────────────────────────────────────
 * Kajiya's 1986 rendering equation describes how light leaves a
 * surface point p in a given outgoing direction ω_o:
 *
 *   L_o(p, ω_o) = L_e(p, ω_o)                                emitted
 *               + ∫_Ω f_r(p, ω_i, ω_o) · L_i(p, ω_i)         scattered
 *                       · (ω_i · n) dω_i
 *
 *   L_o   outgoing radiance       (what the camera sees)
 *   L_e   emitted radiance        (zero except for lights)
 *   f_r   bidirectional reflectance distribution function (BRDF)
 *   L_i   incoming radiance       (what's coming in along ω_i)
 *   ω_i·n cosine of incidence     (Lambert's law geometry term)
 *   Ω     hemisphere above p
 *
 * Read the integral aloud: "the outgoing radiance at this point in
 * this direction equals what the surface emits, PLUS the sum over
 * every incoming direction of (BRDF × incoming radiance × cosine)".
 *
 * The catch: L_i is the SAME quantity recursively. To know what's
 * coming IN at p, you have to know what's going OUT at every point
 * around p. The equation is recursive in L. You can't solve it
 * directly — there's no closed form.
 *
 * What CAN you do? Estimate the integral numerically. T2.
 *
 * T2  MONTE CARLO: AVERAGING RANDOM SAMPLES TO ESTIMATE AN INTEGRAL
 * ──────────────────────────────────────────────────────────────────
 * Suppose you want to estimate ∫_Ω g(ω) dω.
 *
 * Pick a probability density p(ω) on Ω (any density, as long as it's
 * non-zero where g is non-zero). Draw N samples ω_1, …, ω_N from
 * p. Then:
 *
 *           ∫g(ω) dω ≈ (1/N) Σ g(ω_i) / p(ω_i)
 *
 * That ESTIMATOR is unbiased — its expected value equals the true
 * integral. The law of large numbers says the average converges to
 * the truth as N → ∞.
 *
 * For our integrand g(ω) = f_r · L_i · cosθ:
 *
 *           L̂ = (1/N) Σ  f_r(ω_i) · L_i(ω_i) · cosθ_i
 *                          / p(ω_i)
 *
 * Now the only question is: HOW DO YOU GET L_i(ω_i)?
 *
 * Recursively: by tracing a ray from p in direction ω_i and asking
 * what it sees. THAT IS PATH TRACING. Each "path" is one Monte
 * Carlo sample of the rendering-equation integral.
 *
 * T3  THE 7 PASSES — THE UNIVERSAL VOCABULARY
 * ────────────────────────────────────────────
 * Every path tracer ever written organises around the same seven
 * conceptual stages. Naming them is the single most useful thing you
 * can do for your understanding. After this tutorial you can READ
 * any path tracer (smallpt, pbrt, mitsuba, cycles, blender) and find
 * your way around — the names line up.
 *
 *   Pass 1  CAMERA RAY GENERATION         (§6)
 *           pixel → world-space Ray.
 *           Add sub-pixel jitter for free anti-aliasing.
 *
 *   Pass 2  SCENE INTERSECTION            (§7)
 *           Ray → nearest Hit (P, N, mat).
 *           Miss → background (black, here).
 *
 *   Pass 3  SHADING DECISION              (§8)
 *           Hit → Material lookup (emission + albedo).
 *           In a real renderer this dispatches on material type
 *           (diffuse vs specular vs glass); here we have one type
 *           so it's a one-liner — but the conceptual stage is real.
 *
 *   Pass 4  BOUNCE DIRECTION SAMPLING     (§9)
 *           Hit normal → random new outgoing direction.
 *           For Lambertian we use cosine-weighted hemisphere
 *           sampling via Malley's method (T6, T7).
 *
 *   Pass 5  THROUGHPUT CHAIN              (§10)
 *           throughput *= albedo. After N bounces:
 *             throughput = albedo_1 · albedo_2 · … · albedo_N.
 *           That product is "what fraction of any light the path
 *           eventually hits will reach the camera through this
 *           specific path".
 *
 *   Pass 6  TERMINATION                   (§11)
 *           Hard depth cap + Russian roulette. Stop bouncing
 *           when paths run out of interesting energy (T9).
 *
 *   Pass 7  ACCUMULATION + TONE MAP       (§13)
 *           Average many samples per pixel; tone-map HDR result;
 *           map to display colour and ASCII glyph (T10, T12).
 *
 * Mental drill: pick any 30 lines of any path tracer and ask "which
 * pass is this?" If you can't answer in 5 seconds, you don't yet
 * know that pass well enough — re-read its tutorial below.
 *
 * The orchestrator function (§12 path_trace) labels each block with
 * a "PASS N — ..." comment — six labels in twenty-five lines. That
 * IS the algorithm.
 *
 * T4  THE BACKWARD PHOTON
 * ────────────────────────
 * The naïve interpretation of "trace a ray to estimate L_i" is to
 * fire MANY rays from each point. That balloons exponentially with
 * depth (one ray hits a wall → fire 100 from there → each hits
 * another wall → fire 100 from each → 10⁴ rays for two bounces).
 *
 * The trick: don't branch. Pick ONE random direction at each bounce.
 * The single-sample estimator is still unbiased — the average of
 * many such single-sample paths converges to the truth.
 *
 * Pseudocode of one path:
 *
 *   trace(p, ω_o):
 *     hit ← scene_intersect(p, ω_o)
 *     if hit is light:
 *       return L_e(hit)
 *     else:
 *       ω_i ← random direction in hemisphere around N
 *       L_i ← trace(hit, -ω_i)             ← recurse
 *       return f_r · L_i · cosθ / p(ω_i)
 *
 * For a Lambertian surface this becomes (T5-T6):
 *
 *   trace(p, ω_o):
 *     hit ← intersect
 *     if light: return emission
 *     ω_i ← cosine_weighted_sample(N)
 *     return albedo · trace(hit, -ω_i)
 *
 * Iteratively (no recursion, no stack overflow on deep paths):
 *
 *   throughput ← (1, 1, 1)
 *   colour     ← (0, 0, 0)
 *   loop until break:
 *     hit ← intersect                          PASS 2
 *     if miss:        break
 *     mat ← shade_at_hit(hit)                  PASS 3
 *     if light:       colour += throughput · emission;  break
 *     RR check                                  PASS 6
 *     throughput     *= albedo                  PASS 5
 *     ω             ← cosine_weighted_sample(N) PASS 4
 *   return colour
 *
 * The throughput multiplies each surface's albedo as the path walks
 * backwards. Hit a light at the end → that throughput product times
 * the light's emission is your colour estimate.
 *
 * T5  LAMBERTIAN BRDF AND THE cosθ WEIGHT
 * ────────────────────────────────────────
 * A LAMBERTIAN surface scatters incoming light EQUALLY in every
 * outgoing direction in the hemisphere. Its BRDF is constant:
 *
 *   f_r(ω_i, ω_o) = ρ / π
 *
 *   ρ = albedo (∈ [0, 1] per channel)
 *   π = normalisation so the surface conserves energy
 *
 * The cosθ = (ω_i · n) factor in the rendering equation isn't part
 * of the BRDF — it's a GEOMETRIC factor expressing Lambert's law:
 * a flux d-Φ landing on a tilted surface spreads over an area
 * 1/cosθ larger than the same flux landing perpendicular, so the
 * irradiance contribution scales by cosθ.
 *
 * Substituting into the rendering equation:
 *
 *   L_o = L_e + (ρ/π) · ∫_Ω L_i(ω_i) · cosθ_i dω_i
 *
 * The (ρ/π) factor pulls outside; we're left with a cosine-weighted
 * integral of L_i. That cosθ_i is what motivates T6.
 *
 * T6  COSINE-WEIGHTED HEMISPHERE SAMPLING — THE MAGICAL CANCELLATION
 * ───────────────────────────────────────────────────────────────────
 * The Monte Carlo estimator weights each sample by f_r·L_i·cosθ/p(ω).
 *
 * If we pick samples UNIFORMLY on the hemisphere (p = 1/(2π)):
 *
 *   weight = (ρ/π) · L_i · cosθ / (1/2π)
 *          = 2ρ · L_i · cosθ
 *
 * The cosθ STAYS in the weight. Near-grazing samples (cosθ ≈ 0)
 * contribute ~0 even though they're equally likely — wasted samples.
 *
 * Better: pick samples PROPORTIONAL TO cosθ. Set p(ω) = cosθ/π.
 * Then:
 *
 *   weight = (ρ/π) · L_i · cosθ / (cosθ/π)
 *          =  ρ · L_i
 *
 * THE cosθ AND THE π CANCEL.
 *
 * Two things become true at once:
 *   - The estimator weight is just `albedo · L_i` (no cosθ, no π).
 *   - Near-grazing samples are rare (low p), so we don't waste many
 *     samples on directions that contribute little anyway.
 *
 * This is IMPORTANCE SAMPLING — match the sample density to the
 * thing you're integrating. For Lambertian + cosθ the optimal match
 * is cosine-weighted, and the algebra simplifies to "throughput ×=
 * albedo per bounce".
 *
 * Read §10 throughput_chain — the inner update is literally
 * `throughput = throughput * albedo`. That's T6 in code.
 *
 * T7  MALLEY'S METHOD — UNIFORM DISK → HEMISPHERE WITHOUT REJECTION
 * ──────────────────────────────────────────────────────────────────
 * How do you draw samples from p(ω) = cosθ/π?
 *
 * One option: rejection sampling. Sample uniformly on the hemisphere,
 * accept with probability cosθ. Slow — average rejection rate ~50%
 * with worst case unbounded.
 *
 * Better: Malley's method (1988). It exploits a beautiful identity:
 *
 *   "Uniform samples on the unit disk, projected up to the
 *    hemisphere via z = √(1 − r²), give a cosine-weighted
 *    hemisphere distribution."
 *
 * Geometric picture (cross-section through the hemisphere):
 *
 *      hemisphere      ⌒
 *                    ╱   ╲
 *                  ╱       ╲          ← uniform points on the
 *                ╱  • • •    ╲          DISK below project up
 *              ╱                ╲       onto the hemisphere via
 *            ╱   • • • • • •      ╲      z = √(1 − x² − y²)
 *      _____╱_____________________╲_____   = √(1 − r²)
 *           ↑                     ↑
 *           uniform disk          z gives cosθ
 *
 * So sample uniformly on a 2-D disk; lift to 3-D via z = √(1−r²);
 * the resulting hemisphere samples are cosine-weighted. Two RNG
 * calls, no loops, numerically stable at all angles.
 *
 * Pseudocode (matches §9.2 sample_bounce):
 *
 *   r1, r2 ~ U[0, 1)
 *   φ      = 2π · r1                      angle around the axis
 *   in-plane radius = √r2                 (uniform on disk)
 *   z              = √(1 − r2)            (lift onto hemisphere)
 *   local_dir = (cosφ · √r2, sinφ · √r2, z)
 *   world_dir = onb(N) · local_dir        (rotate so N is +Z)
 *
 * The `onb(N)` builds an orthonormal basis around the surface
 * normal so "local +Z" maps to "world N". See §9.1.
 *
 * T8  THE THROUGHPUT CHAIN — HOW COLOUR BLEEDING EMERGES
 * ───────────────────────────────────────────────────────
 * In §12, between bounces:
 *
 *     throughput = throughput · albedo
 *
 * That's a CHANNEL-WISE multiply (red × red, green × green, blue ×
 * blue). After N bounces the throughput is:
 *
 *     final_throughput = albedo_1 · albedo_2 · … · albedo_N
 *
 * If a path goes  EYE → red wall (0.65, 0.05, 0.05)
 *                     → white floor (0.73, 0.73, 0.73)
 *                     → light (15, 14, 11),
 *
 * the final contribution is:
 *
 *     (0.65, 0.05, 0.05) · (0.73, 0.73, 0.73) · (15, 14, 11)
 *     = (7.12, 0.51, 0.40)
 *
 * BRIGHT RED. That's COLOUR BLEEDING — the red wall stained the
 * floor's contribution as the photon walked back to the eye. The
 * green channel got crushed (0.05) because the red wall absorbs
 * green; barely any green light makes it through that path.
 *
 * Try it: in PT mode, watch the gold sphere's left flank as samples
 * accumulate. It picks up a red blush from paths that bounced off
 * the red wall before reaching it. The right flank picks up green
 * from the green wall. That's two-bounce color bleeding rendered
 * automatically — no special code, just throughput × albedo per
 * step.
 *
 * T9  RUSSIAN ROULETTE — TERMINATING WITHOUT BIAS
 * ────────────────────────────────────────────────
 * We can't trace paths to infinite depth. Some termination rule is
 * needed. Three options:
 *
 *   HARD CAP        stop at depth = 7. Simple but BIASED — we ignore
 *                   light that would arrive via deeper paths.
 *   FADE-OUT        scale down contributions at deep bounces. Smooth
 *                   but still biased.
 *   RUSSIAN ROULETTE  unbiased termination — the crown jewel.
 *
 * Russian roulette: at each bounce past some depth, KILL the path
 * with probability (1 − p), where p is some "survival probability".
 * On survival, multiply throughput by 1/p:
 *
 *     if rng() > p:                kill the path (return 0)
 *     else:                        throughput *= 1/p
 *
 * Why is this unbiased? Compute the expected contribution:
 *
 *     E[contribution] = p · (contribution × 1/p)
 *                     + (1 − p) · 0
 *                     = contribution
 *
 * The 1/p compensation EXACTLY counterbalances the (1 − p) kills
 * in expectation. So the answer is right ON AVERAGE, even though
 * any single path is either zero or amplified.
 *
 * Choosing p: we want to keep paths that CAN STILL contribute
 * meaningfully. The throughput's MAX channel is a good proxy for
 * remaining energy. If max(throughput) = 0.001, contributions
 * through this path are dwarfed by paths with throughput 0.5 — kill
 * with high probability. If max = 0.9, keep almost certainly.
 *
 *     p = max(throughput.r, .g, .b)
 *
 * MAX (not MEAN) so we keep paths whose energy is concentrated in
 * one band (deep red after a red-wall hit).
 *
 * RR_DEPTH = 3: don't enable RR before this depth. Early bounces
 * carry most of the energy regardless of throughput, so killing
 * them just raises variance. After ~3 bounces the throughputs have
 * dimmed enough that RR's pruning saves more than it costs.
 *
 * T10 PROGRESSIVE ACCUMULATION AND THE √N CONVERGENCE LAW
 * ────────────────────────────────────────────────────────
 * One Monte Carlo sample is NOISY. The estimator is correct on
 * AVERAGE, but any single sample is a long way from the true value.
 *
 * Variance of an N-sample average:
 *
 *     Var(L̄_N) = σ² / N
 *
 * Standard deviation (visible noise) ∝ 1/√N.
 *
 * To halve visible noise, you must QUADRUPLE the sample count.
 * This is the central pain of path tracing — fast convergence
 * doesn't exist; only fewer samples per noticeable improvement.
 *
 * Implementation: keep a per-pixel SUM of all samples ever cast,
 * and a global COUNT. Display = sum / count. Adding more samples
 * just refines that average:
 *
 *     g_accum[r][c] += new_sample;
 *     g_samples     += 1;
 *     display       = g_accum[r][c] / g_samples;
 *
 * That's PROGRESSIVE refinement. The user sees:
 *
 *     1 sample        almost solid noise, hint of bright spot
 *     32 samples      silhouettes, lots of grain
 *     256 samples     clean walls, slight grain on spheres
 *     2048 samples    near-converged, only subtle noise remains
 *     8192 samples    converged for visual purposes
 *
 * The SPP slider (`+`/`-`) controls samples-per-FRAME, not
 * samples-per-final-image. SPP=8 gives you 8× the convergence per
 * frame at 8× the per-frame cost — same asymptotic, just different
 * pacing.
 *
 * KEY: keep the accumulator in LINEAR HDR space. Tone-map only at
 * draw time, AFTER dividing by sample count. Tone-mapping the sum
 * is mathematically wrong: tonemap(a+b) ≠ tonemap(a) + tonemap(b).
 *
 * T11 THE CORNELL BOX — WHAT THIS SCENE SPECIFICALLY TESTS
 * ─────────────────────────────────────────────────────────
 * The Cornell Box (Goral et al. 1984) is the canonical test scene
 * for global-illumination renderers. It was originally a PHYSICAL
 * BOX in a Cornell University lab — built from coloured paper and
 * photographed under measured light, then matched against
 * radiosity-renderer output to validate the algorithm.
 *
 * Key features:
 *
 *   OPEN FRONT   The camera sits at z = -1.5, looking +Z into the
 *                box's open face. Light only escapes through this
 *                opening (and the camera's pixels).
 *   COLOURED     Left wall RED, right wall GREEN. White floor,
 *   WALLS        ceiling, back. The colour asymmetry produces
 *                obvious COLOUR BLEEDING that's easy to verify.
 *   AREA LIGHT   The light is a small flat quad on the ceiling, not
 *                a point. Real lights have area; point lights are a
 *                fiction. Area lights produce SOFT shadows whose
 *                penumbra width depends on light size.
 *   2 SPHERES    Diffuse spheres on the floor. They're the focus —
 *                their colour shifts subtly toward the nearer wall.
 *
 * What this scene tests, that simpler scenes don't:
 *
 *   COLOUR BLEEDING        red wall stains nearby surfaces red,
 *                          green wall stains nearby surfaces green.
 *                          Whole-scene effect, not just the wall.
 *   AREA-LIGHT SOFT        floor under each sphere is dimmer than
 *   SHADOWS                surrounding floor; shadow edges are
 *                          gradient, not crisp.
 *   INDIRECT BOUNCE        ceiling NEAR the light is brighter than
 *                          ceiling at the corners — bounce light
 *                          from the floor warms the ceiling.
 *   ENERGY BALANCE         radiosity in = radiosity out. Get this
 *                          wrong (energy leak or gain) and the
 *                          colours drift over thousands of samples.
 *
 * If a renderer produces all four of those features visibly and
 * stably, it's a CORRECT global-illumination renderer. That's why
 * the Cornell Box is the sanity-check benchmark.
 *
 * Side note: the ORIGINAL smallpt represents the Cornell box with
 * NINE huge spheres (giant spheres acting as walls — clever but
 * pedagogically confusing). We use real axis-aligned quads instead
 * because the ray-quad math is simpler and the scene reads as it's
 * drawn.
 *
 * T12 FOUR DEBUG OVERLAYS — PT VS NORMAL VS ALBEDO VS DEPTH
 * ──────────────────────────────────────────────────────────
 * Cycling `d` switches between four rendering modes. Each one
 * short-circuits the path tracer at the FIRST hit and visualises
 * the output of one or two early passes — pedagogically this lets
 * you SEE what each pass produces in isolation.
 *
 *   PT       Pass 1+2+3+4+5+6+7. Full progressive Monte Carlo path
 *            tracer (default). Slowly converges from noise to clean.
 *   NORMAL   Pass 1+2 only. Per pixel, find the FIRST hit and
 *            encode its surface normal as RGB ((N+1)/2). Each face
 *            of the Cornell Box becomes a flat colour:
 *              floor   normal +Y → (0.5, 1.0, 0.5)
 *              ceiling normal -Y → (0.5, 0.0, 0.5)
 *              left    normal +X → (1.0, 0.5, 0.5)
 *              right   normal -X → (0.0, 0.5, 0.5)
 *              back    normal -Z → (0.5, 0.5, 0.0)
 *            Diagnostic for "did intersection produce the right
 *            normal?" If a face shows the wrong colour, the
 *            geometry is broken.
 *   ALBEDO   Pass 1+2+3. Per pixel, find the FIRST hit and return
 *            the material's body colour. No bounces, no lighting.
 *            Compare ALBEDO vs PT: how much detail comes from
 *            indirect light? In a Cornell Box: a LOT.
 *   DEPTH    Pass 1+2 only. Brightness ∝ 1 / (1 + t). Closer
 *            surfaces brighter, distant darker. Gray-scale only.
 *            Sanity check that intersection produces sane distances.
 *
 * The three debug modes converge in ONE SAMPLE because no
 * randomness is involved. Switching INTO them is instant; switching
 * OUT to PT resets the accumulator (the modes have different output
 * ranges and mixing them under one average would be incoherent).
 *
 * Pedagogical contrast:
 *   NORMAL  → "this is what the geometry pipeline gives us"
 *   ALBEDO  → "this is what the materials assert their colour is"
 *   DEPTH   → "this is what the camera-ray distances look like"
 *   PT      → "this is what they look like with light bouncing
 *              everywhere — and that's what makes it photo-real"
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ──────────────────────────────────────────────────────── */

/* §1.1 frame rate. PT is compute-heavy so we target 30 Hz, not 60. */
#define TARGET_FPS 30
#define DT_CAP_NS 200000000LL /* 0.2 s — spiral-of-death cap */

/* §1.2 view geometry. */
#define ASPECT 0.47f  /* terminal cell W/H ratio    */
#define FOV_DEG 66.0f /* horizontal FOV             */

/* §1.3 path tracing. */
#define MAX_DEPTH 7   /* hard depth cap per path     */
#define RR_DEPTH 3    /* RR starts at this depth     */
#define SPP_DEFAULT 2 /* samples per pixel per frame */
#define SPP_MIN 1
#define SPP_MAX 8
#define ACCUM_CAP 8192 /* auto-pause once converged   */
#define RAY_EPS 1e-4f  /* origin offset along N       */

#define MAX_W 320 /* static accumulator width    */
#define MAX_H 100 /* static accumulator height   */

/*
 * §1.4 Cornell box coordinate system:
 *   x ∈ [-1, 1]   left (red) → right (green)
 *   y ∈ [-1, 1]   floor → ceiling
 *   z ∈ [ 0, 2]   open front → back wall
 *   Camera at (0, 0.05, -1.5) looking toward +Z. The front face
 *   (z = 0) is open so the camera sees in.
 */
#define CAM_X 0.00f
#define CAM_Y 0.05f
#define CAM_Z -1.50f

/* §1.5 character ramp — Paul Bourke 92-char density ladder. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.6 ncurses pair IDs. */
#define PAIR_CUBE_BASE 1   /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD 217       /* yellow row 0 status         */
#define PAIR_HINT 218      /* cyan bottom hint strip      */
#define PAIR_BAR_FILL 219  /* progress-bar filled cells   */
#define PAIR_BAR_EMPTY 220 /* progress-bar empty cells    */

/* §1.7 shade-mode enum (cycled with 'd'). See T12 + §14.
 *
 *   MODE_PT      full progressive Monte Carlo path tracer (default).
 *                Uses passes 1-7.
 *   MODE_NORMAL  first-hit surface normal as RGB. Passes 1+2 only.
 *   MODE_ALBEDO  first-hit material colour. Passes 1+2+3 only.
 *   MODE_DEPTH   first-hit 1/(1+t) gray. Passes 1+2 only.
 *
 * NORMAL/ALBEDO/DEPTH are deterministic — they converge in 1 sample
 * because no randomness is involved. Switching modes resets the
 * accumulator (their output ranges differ; mixing them under one
 * average would produce incoherent results).
 */
typedef enum {
  MODE_PT = 0,
  MODE_NORMAL = 1,
  MODE_ALBEDO = 2,
  MODE_DEPTH = 3,
  MODE_N = 4,
} ShadeMode;

static const char *shade_mode_name(ShadeMode m) {
  switch (m) {
  case MODE_PT:
    return "PT    ";
  case MODE_NORMAL:
    return "NORMAL";
  case MODE_ALBEDO:
    return "ALBEDO";
  case MODE_DEPTH:
    return "DEPTH ";
  default:
    return "?     ";
  }
}

/* ── §2 clock ───────────────────────────────────────────────────────── */

/*
 * clock_ns — wall-clock time in nanoseconds, monotonic.
 *
 * Why CLOCK_MONOTONIC: we care about ELAPSED real time (for animation
 * dt), not wall date. CLOCK_MONOTONIC never goes backward across NTP
 * adjustments, DST shifts, or system clock changes.
 */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/*
 * clock_sleep_ns — best-effort sleep for the requested nanoseconds.
 *
 * The main loop uses this to cap render rate without burning the CPU
 * at 100%; we subtract elapsed work time from the target frame and
 * sleep the remainder.
 */
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3 vec3 ────────────────────────────────────────────────────────── *
 *
 * V3 — three floats by value. All vector helpers are inline to avoid
 * call overhead in the per-bounce loop.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3mul(V3 a, V3 b) {
  return v3(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline V3 v3s(float s, V3 a) { return v3(s * a.x, s * a.y, s * a.z); }
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float length = v3len(a);
  return length > 1e-9f ? v3s(1.f / length, a) : v3(0, 1, 0);
}
static inline V3 v3cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}

/* v3maxc — maximum channel of a V3. Used by Russian roulette to pick
 * the survival probability (T9: max, not mean, to keep paths whose
 * energy is concentrated in a single colour band). */
static inline float v3maxc(V3 a) {
  return a.x > a.y ? (a.x > a.z ? a.x : a.z) : (a.y > a.z ? a.y : a.z);
}

/* ── §4 RNG (xorshift32, decorrelated per pixel per frame) ──────────── *
 *
 * Path tracing needs LOTS of random numbers (sub-pixel jitter + 2 per
 * cosine-hemisphere sample × MAX_DEPTH bounces × SPP × cols × rows).
 * Quality matters less than throughput: we use xorshift32, a single-
 * register PRNG with a period of 2³² − 1 — plenty for a single frame.
 *
 * Decorrelation: every (pixel, frame) gets an independent seed via a
 * small hash. Without that, neighbouring pixels would receive
 * correlated sequences and the noise would form visible streaks.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef uint32_t Rng;

/*
 * rng_f — uniform float in [0, 1) via xorshift32.
 *
 * The shift triple (13, 17, 5) is Marsaglia's, with the upper bit
 * masked off (>>1) so the result is non-negative when reinterpreted
 * via the integer-to-float divide.
 */
static float rng_f(Rng *rng_state) {
  *rng_state ^= *rng_state << 13;
  *rng_state ^= *rng_state >> 17;
  *rng_state ^= *rng_state << 5;
  return (float)(*rng_state >> 1) * (1.f / (float)0x7FFFFFFF);
}

/*
 * rng_seed — produce an independent seed for (px, py, frame).
 *
 * The three large primes + xorshift warm-up scramble the components
 * so adjacent pixels and adjacent frames don't share correlated
 * sequences. Without decorrelation, neighbouring pixels would receive
 * the same sample sequence and the noise would form visible streaks.
 */
static Rng rng_seed(int pixel_x, int pixel_y, int frame) {
  uint32_t s = (uint32_t)(pixel_x * 1973 + pixel_y * 9277 + frame * 26699 + 1);
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s ? s : 1u;
}

/* ── §5 scene (Cornell box) ─────────────────────────────────────────── *
 *
 * The Cornell Box has just three primitive types:
 *   §5.1 Material (Lambertian diffuse only)
 *   §5.2 Quad (axis-aligned rectangle)
 *   §5.3 Sphere
 *
 * Six quads form the walls (floor, ceiling, back, left, right, plus
 * the small ceiling light). Two spheres sit on the floor. That's the
 * entire geometry — six rectangles and two balls.
 *
 * See tutorial T11 for the scene's history and what it specifically
 * tests.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §5.1 ── Material ──────────────────────────────────────────────────── */

/*
 * Lambertian diffuse only.
 *   albedo ∈ [0, 1]³ — fraction of light reflected per channel
 *   emit             — radiance emitted (zero for non-emissive)
 *
 * With Lambertian BRDF f_r = albedo/π and cosine-weighted hemisphere
 * sampling p(ω) = cosθ/π (T6), the Monte Carlo estimator simplifies
 * to weight = albedo. This is why "throughput *= albedo" is the
 * entire bounce update (§10 + §12).
 */
typedef struct {
  V3 albedo;
  V3 emit;
} Mat;

static const Mat k_mats[] = {
    /* 0  white  */ {{0.73f, 0.73f, 0.73f}, {0, 0, 0}},
    /* 1  red    */ {{0.65f, 0.05f, 0.05f}, {0, 0, 0}},
    /* 2  green  */ {{0.12f, 0.45f, 0.15f}, {0, 0, 0}},
    /* 3  light  */ {{0, 0, 0}, {15.f, 14.f, 11.f}}, /* warm */
    /* 4  gold   */ {{0.80f, 0.58f, 0.18f}, {0, 0, 0}},
    /* 5  indigo */ {{0.22f, 0.28f, 0.82f}, {0, 0, 0}},
};

static inline bool mat_is_light(const Mat *m) {
  return m->emit.x > 0.f || m->emit.y > 0.f || m->emit.z > 0.f;
}

/* §5.2 ── Quad (axis-aligned rectangular plane) ───────────────────── */

/*
 * A Cornell-box wall is an axis-aligned rectangle.
 *   axis = 0 → X-plane: lo/hi are bounds in (Y, Z)
 *   axis = 1 → Y-plane: lo/hi are bounds in (X, Z)
 *   axis = 2 → Z-plane: lo/hi are bounds in (X, Y)
 *
 * Encoding the wall as "fixed coordinate + 2-axis bounds" lets the
 * intersection test be a single divide + two range checks (§7.1) —
 * far cheaper than a triangle.
 */
typedef struct {
  int axis;           /* 0=X plane, 1=Y plane, 2=Z plane              */
  float pos;          /* coordinate of the plane on its fixed axis    */
  float lo[2], hi[2]; /* bounds in the two free axes                  */
  int mat;            /* material index into k_mats                   */
} Quad;

/*
 * Cornell-box quads. The light at y = 0.98 sits just below the ceiling
 * (y = 1.0); rays going UP through the light's XZ footprint hit it
 * before the ceiling (smaller t) and pick up the emission.
 */
static const Quad k_quads[] = {
    /* floor    y=-1   x∈[-1,1] z∈[0,2] */ {
        1, -1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0},
    /* ceiling  y=+1   x∈[-1,1] z∈[0,2] */
    {1, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0},
    /* back     z=+2   x∈[-1,1] y∈[-1,1]*/
    {2, 2.0f, {-1.f, -1.f}, {1.f, 1.f}, 0},
    /* left     x=-1   y∈[-1,1] z∈[0,2] */
    {0, -1.0f, {-1.f, 0.f}, {1.f, 2.f}, 1},
    /* right    x=+1   y∈[-1,1] z∈[0,2] */
    {0, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 2},
    /* light    y=0.98 centred overhead */
    {1, 0.98f, {-0.36f, 0.62f}, {0.36f, 1.38f}, 3},
};
#define N_QUADS ((int)(sizeof k_quads / sizeof k_quads[0]))

/* §5.3 ── Sphere ───────────────────────────────────────────────────── */

/*
 * Two diffuse spheres just above the floor. Bottom at y ≈ -0.98 with
 * floor at y = -1.0, leaving a small gap to avoid numerical
 * self-intersection between sphere and floor.
 */
typedef struct {
  V3 c;
  float r;
  int mat;
} Sphere;

static const Sphere k_spheres[] = {
    {{-0.46f, -0.60f, 0.82f}, 0.38f, 4}, /* gold  , left  */
    {{0.44f, -0.60f, 1.16f}, 0.38f, 5},  /* indigo, right */
};
#define N_SPHERES ((int)(sizeof k_spheres / sizeof k_spheres[0]))

/* ── §6 PASS 1 — CAMERA RAY GENERATION ──────────────────────────────── *
 *
 * Pass 1 turns a pixel index (col, row) into a world-space Ray.
 *
 * The camera is a pinhole at (CAM_X, CAM_Y, CAM_Z) looking down +Z.
 * Each pixel maps to a screen-space (u, v) which we offset with sub-
 * pixel jitter for free anti-aliasing.
 *
 *      pixel (col, row)              world-space ray
 *      ┌─────┐                       ┌─────────────────────────┐
 *      │     │                       │ origin: camera position │
 *      │  +  │   ──── camera_ray ──▶ │ dir   : norm(forward    │
 *      │     │                       │              + u·right  │
 *      └─────┘                       │              + v·up   ) │
 *                                    └─────────────────────────┘
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  V3 origin;
  V3 dir;
} Ray;

/*
 * camera_ray — produce a primary ray for pixel (col, row).
 *
 * Pseudocode:
 *   centre_x = cols / 2          centre_y = rows / 2
 *   tan_half = tan(FOV / 2)
 *   u =  ((col + jitter_x) - centre_x) / centre_x  *  tan_half
 *   v = -((row + jitter_y) - centre_y) / centre_x  *  tan_half / ASPECT
 *   ray.origin = camera position
 *   ray.dir    = norm(forward + u·right + v·up)
 *
 * Why divide v by centre_x (NOT centre_y): we want square pixels in
 * world space — equal angular extent per cell on both axes. Then we
 * correct for the terminal cell's aspect ratio (cells are taller
 * than wide) by a final /ASPECT.
 *
 * Why the negative sign on v: row indices increase downward (y-down)
 * but world-space "up" is +Y (y-up). Negate so the world's +Y maps
 * to the top of the screen (lower row numbers).
 *
 * jitter_x, jitter_y ∈ [-0.5, +0.5] gives free anti-aliasing — each
 * sample's primary ray pierces a slightly different sub-pixel point;
 * averaging filters out staircase aliasing along edges.
 */
static Ray camera_ray(int col, int row, int cols, int rows, float jitter_x,
                      float jitter_y) {
  (void)rows; /* v normalised by cols */
  float centre_x = cols * 0.5f;
  float centre_y = rows * 0.5f;
  float tan_half = tanf(FOV_DEG * (float)M_PI / 360.f);

  float u = ((col + jitter_x) - centre_x) / centre_x * tan_half;
  float v = -((row + jitter_y) - centre_y) / centre_x * tan_half / ASPECT;

  V3 forward = v3(0, 0, 1);
  V3 right = v3(1, 0, 0);
  V3 up = v3(0, 1, 0);

  Ray ray;
  ray.origin = v3(CAM_X, CAM_Y, CAM_Z);
  ray.dir = v3norm(v3add(forward, v3add(v3s(u, right), v3s(v, up))));
  return ray;
}

/* ── §7 PASS 2 — SCENE INTERSECTION ─────────────────────────────────── *
 *
 * Pass 2 finds the closest surface a Ray hits.
 *
 * Three short tests:
 *   §7.1 ray_quad    — ray vs axis-aligned rectangle
 *   §7.2 ray_sphere  — ray vs sphere (textbook quadratic)
 *   §7.3 scene_hit   — try every primitive, keep the nearest
 *
 * No spatial structures — N is small (8 primitives). A real renderer
 * would put a BVH here; we keep it brute-force for clarity.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * Hit record: position, surface normal (oriented toward the ray), and
 * the material index. The normal-toward-ray convention means the
 * hemisphere sampling in §9 always produces directions on the
 * outgoing side without needing a flip later.
 */
typedef struct {
  float t;
  V3 P, N;
  int mat;
} Hit;

/* §7.1 ── ray vs axis-aligned quad ─────────────────────────────────── */

/*
 * ray_quad — analytic intersection of a ray with one axis-aligned
 *            rectangle.
 *
 * Pseudocode:
 *   On the FIXED axis, find t at which the ray crosses the plane:
 *     dir_a, origin_a = ray.dir[axis], ray.origin[axis]
 *     if |dir_a| < ε:   ray is parallel → MISS
 *     t = (q->pos - origin_a) / dir_a
 *     if t < t_min:     behind us → MISS
 *
 *   Compute the hit point's two FREE-axis coordinates and check
 *   against the rectangle's bounds:
 *     hit_point = ray.origin + t · ray.dir
 *     u, v      = hit_point's two free-axis components
 *     if u or v outside [lo, hi]:   MISS
 *
 *   The outward normal is the basis vector of the FIXED axis with
 *   sign chosen to FACE the incoming ray (so subsequent hemisphere
 *   sampling puts directions on the outgoing side automatically).
 */
static int ray_quad(Ray ray, const Quad *quad, float t_min, float *out_t,
                    V3 *out_normal) {
  float dir_a, origin_a;
  switch (quad->axis) {
  case 0:
    dir_a = ray.dir.x;
    origin_a = ray.origin.x;
    break;
  case 1:
    dir_a = ray.dir.y;
    origin_a = ray.origin.y;
    break;
  default:
    dir_a = ray.dir.z;
    origin_a = ray.origin.z;
    break;
  }
  if (fabsf(dir_a) < 1e-9f)
    return 0; /* parallel to plane */

  float t = (quad->pos - origin_a) / dir_a;
  if (t < t_min)
    return 0;

  float hit_x = ray.origin.x + t * ray.dir.x;
  float hit_y = ray.origin.y + t * ray.dir.y;
  float hit_z = ray.origin.z + t * ray.dir.z;

  float free_axis_u, free_axis_v;
  switch (quad->axis) {
  case 0:
    free_axis_u = hit_y;
    free_axis_v = hit_z;
    break;
  case 1:
    free_axis_u = hit_x;
    free_axis_v = hit_z;
    break;
  default:
    free_axis_u = hit_x;
    free_axis_v = hit_y;
    break;
  }
  if (free_axis_u < quad->lo[0] || free_axis_u > quad->hi[0] ||
      free_axis_v < quad->lo[1] || free_axis_v > quad->hi[1])
    return 0;

  /* Outward normal: basis vector of fixed axis, sign opposing ray. */
  V3 normal = {0, 0, 0};
  switch (quad->axis) {
  case 0:
    normal.x = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  case 1:
    normal.y = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  default:
    normal.z = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  }
  *out_t = t;
  *out_normal = normal;
  return 1;
}

/* §7.2 ── ray vs sphere ─────────────────────────────────────────────── */

/*
 * ray_sphere — analytic intersection via the textbook quadratic.
 *
 * Setup: |ray.origin + t·ray.dir − centre|² = radius². With unit-
 * length ray.dir, this expands to
 *
 *     t² + 2·b·t + (|origin_to_centre|² − radius²) = 0
 *
 * where b = ray.dir · origin_to_centre.
 *
 * Discriminant disc = b² − (|origin_to_centre|² − radius²).
 * Roots t = -b ± √disc. We pick the nearest root that's beyond
 * t_min (front face for an outside ray; far root only if the ray
 * starts INSIDE the sphere, which can't happen here — diffuse
 * surfaces don't let rays in).
 */
static int ray_sphere(Ray ray, const Sphere *sphere, float t_min,
                      float *out_t) {
  V3 origin_to_centre = v3sub(ray.origin, sphere->c);
  float b = v3dot(ray.dir, origin_to_centre);
  float disc =
      b * b - v3dot(origin_to_centre, origin_to_centre) + sphere->r * sphere->r;
  if (disc < 0.f)
    return 0;

  float sq = sqrtf(disc);
  float t = -b - sq;
  if (t < t_min)
    t = -b + sq;
  if (t < t_min)
    return 0;
  *out_t = t;
  return 1;
}

/* §7.3 ── scene_hit (find nearest surface) ─────────────────────────── */

/*
 * scene_hit — loop over all primitives, keep the smallest valid t.
 *
 * For 6 quads + 2 spheres this brute-force test is fine; a real
 * renderer would put a BVH here, but BVH belongs in a separate
 * teaching file (raymarcher.c uses spatial structures).
 *
 * The sphere normal is computed AFTER selection: an outward normal
 * `(P − centre)/r`, then flipped if it points away from the ray
 * (ensuring the hemisphere sampler gets a normal on the outgoing
 * side, regardless of front/back hit).
 */
static int scene_hit(Ray ray, float t_min, Hit *out_hit) {
  float t_best = 1e30f;
  int any = 0;

  for (int i = 0; i < N_QUADS; i++) {
    float t;
    V3 normal;
    if (ray_quad(ray, &k_quads[i], t_min, &t, &normal) && t < t_best) {
      t_best = t;
      out_hit->t = t;
      out_hit->P = v3add(ray.origin, v3s(t, ray.dir));
      out_hit->N = normal;
      out_hit->mat = k_quads[i].mat;
      any = 1;
    }
  }
  for (int i = 0; i < N_SPHERES; i++) {
    float t;
    if (ray_sphere(ray, &k_spheres[i], t_min, &t) && t < t_best) {
      t_best = t;
      out_hit->t = t;
      out_hit->P = v3add(ray.origin, v3s(t, ray.dir));
      V3 outward_n = v3norm(v3sub(out_hit->P, k_spheres[i].c));
      /* Flip outward normal to face the incoming ray. */
      out_hit->N =
          (v3dot(outward_n, ray.dir) < 0.f) ? outward_n : v3s(-1.f, outward_n);
      out_hit->mat = k_spheres[i].mat;
      any = 1;
    }
  }
  return any;
}

/* ── §8 PASS 3 — SHADING DECISION ───────────────────────────────────── *
 *
 * Pass 3 looks up the Material at a hit point. Trivial here (it's
 * just an array index) but factored into a named function so the
 * spine of the algorithm reads as "every pass = one operation".
 *
 * In a real renderer this is where you'd dispatch on material type:
 * diffuse vs specular vs glass vs subsurface. We have one type
 * (Lambertian) so it's a one-liner — but the conceptual stage is
 * real and the orchestrator (§12) calls it explicitly.
 *
 * The actual shading DECISION ("emissive? then accumulate and stop;
 * otherwise prepare to bounce") happens in path_trace using the
 * Material this function returns. That decision belongs with the
 * orchestrator, not with the lookup.
 *
 * ─────────────────────────────────────────────────────────────────── */

static const Mat *shade_at_hit(const Hit *hit) { return &k_mats[hit->mat]; }

/* ── §9 PASS 4 — BOUNCE DIRECTION SAMPLING ──────────────────────────── *
 *
 * Pass 4 picks a random new direction from the BRDF's importance-
 * sampled distribution. For Lambertian we use cosine-weighted
 * hemisphere sampling via Malley's method (T6, T7).
 *
 * Two helper functions:
 *   §9.1 onb              — orthonormal basis around a normal
 *   §9.2 sample_bounce    — Malley's method (was cos_sample_hemi)
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §9.1 ── onb: orthonormal basis around a normal ────────────────────── */

/*
 * onb — construct two perpendicular unit vectors so {u, v, n} is a
 *       right-handed orthonormal basis.
 *
 * Used by the cosine-hemisphere sampler to translate "local +Z =
 * normal" samples into world coordinates.
 *
 * The "pick non-parallel up" trick: if n is mostly along X (|n.x| ≥
 * 0.9) use Y as the seed; otherwise use X. Either way `up × n` is
 * non-zero, so v3cross produces a valid perpendicular.
 */
static void onb(V3 normal, V3 *out_u, V3 *out_v) {
  V3 up = (fabsf(normal.x) < 0.9f) ? v3(1, 0, 0) : v3(0, 1, 0);
  *out_u = v3norm(v3cross(up, normal));
  *out_v = v3cross(normal, *out_u);
}

/* §9.2 ── sample_bounce: cosine-weighted hemisphere sample ──────────── */

/*
 * sample_bounce — Malley's method (1988): the 2-D uniform-disk
 *                 sampling distribution, lifted to a hemisphere via
 *                 z = √(1 − r²), produces a 3-D distribution whose
 *                 density is exactly cosθ/π — exactly what we need.
 *
 *   r1 ∈ [0, 1)  →  φ = 2π · r1                  azimuth
 *   r2 ∈ [0, 1)  →  in-plane radius = √r2        (uniform on disc)
 *                   z              = √(1 − r2)   (lift onto hemisphere)
 *
 * Local sample is (cosφ·√r2, sinφ·√r2, √(1−r2)). Transform to world
 * using the (u, v, n) basis from §9.1.
 *
 * Why this beats rejection sampling:
 *   • Always two RNG calls — no loop with worst-case unbounded retries.
 *   • Numerically stable at grazing angles where rejection thrashes.
 *   • PDF analytically known and matches the Lambertian BRDF — the
 *     algebraic cancellation in T6 hinges on this.
 *
 * See tutorial T7 for the diagram.
 */
static V3 sample_bounce(V3 normal, Rng *rng_state) {
  float r1 = rng_f(rng_state);
  float r2 = rng_f(rng_state);
  float phi = 2.f * (float)M_PI * r1;
  float sr2 = sqrtf(r2); /* sinθ */
  float local_x = cosf(phi) * sr2;
  float local_y = sinf(phi) * sr2;
  float local_z = sqrtf(1.f - r2); /* cosθ */
  V3 basis_u, basis_v;
  onb(normal, &basis_u, &basis_v);
  return v3norm(v3add(v3s(local_x, basis_u),
                      v3add(v3s(local_y, basis_v), v3s(local_z, normal))));
}

/* ── §10 PASS 5 — THROUGHPUT CHAIN ──────────────────────────────────── *
 *
 * Pass 5 is one line of arithmetic: throughput *= albedo.
 *
 * Why it deserves its own §-section: this single multiplication is
 * the entire BRDF evaluation in our path tracer. Cosine-weighted
 * hemisphere sampling cancelled the cosθ and the π (T6), leaving
 * just "multiply by albedo" as the bounce update. Every other path
 * tracer has a more elaborate update; ours is one of the most
 * stripped-down possible.
 *
 * After N bounces:
 *   throughput = albedo_1 · albedo_2 · … · albedo_N
 *
 * That product encodes "what fraction of any light we eventually
 * find will reach the camera through this exact path". When the
 * path finally hits an emissive surface, multiplying its emission
 * by this throughput gives the contribution of this random walk —
 * see T8's worked example.
 *
 * The function below is an inline one-liner; in a textbook this
 * might be a paragraph of comment with no code at all. Naming it
 * makes Pass 5 greppable.
 *
 * ─────────────────────────────────────────────────────────────────── */

static inline V3 throughput_chain(V3 throughput, V3 albedo) {
  return v3mul(throughput, albedo);
}

/* ── §11 PASS 6 — TERMINATION ───────────────────────────────────────── *
 *
 * Pass 6 decides when to stop bouncing. Two mechanisms:
 *
 *   (a) HARD DEPTH CAP — an outer for-loop bound (MAX_DEPTH). After
 *       MAX_DEPTH bounces, force-stop even if RR didn't kill us.
 *       Belt-and-braces: keeps bad RR luck from running forever.
 *
 *   (b) RUSSIAN ROULETTE — probabilistic kill, unbiased. Used after
 *       depth ≥ RR_DEPTH so early high-energy bounces aren't killed.
 *       See T9.
 *
 * russian_roulette() implements (b). The depth cap (a) is a loop
 * bound in path_trace, not a separate function.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * russian_roulette — unbiased path termination.
 *
 * Pseudocode:
 *   p = max(throughput.r, throughput.g, throughput.b)
 *   if p < ε:                       throughput is dead anyway → kill
 *   if random() > p:                kill with probability (1 − p)
 *   throughput *= 1 / p             (compensate survivors → unbiased)
 *   return survived?
 *
 * Returning false means the caller should break out of the bounce
 * loop. Returning true means the caller continues — and the
 * throughput has already been scaled in-place to compensate.
 *
 * Why MAX channel and not MEAN: a path with throughput (0.5, 0, 0)
 * is meaningful in the red channel; killing it because the average
 * is 0.17 would erase real red contributions.
 */
static bool russian_roulette(V3 *throughput, Rng *rng_state) {
  float p = v3maxc(*throughput);
  if (p < 1e-4f)
    return false; /* throughput too low */
  if (rng_f(rng_state) > p)
    return false;                          /* rolled the kill   */
  *throughput = v3s(1.f / p, *throughput); /* compensate       */
  return true;                             /* survived         */
}

/* ── §12 path_trace (orchestrator: combines passes 2-6) ─────────────── *
 *
 * THE CORE. This function is small — one for-loop, ~25 lines of body —
 * but every line is one of the seven passes. Read it slowly.
 *
 * INPUT  : a primary Ray (produced by Pass 1 in the caller)
 *          plus a per-pixel-per-frame RNG state.
 * OUTPUT : V3 radiance contribution from this single random walk.
 *          Many such walks averaged form the converged image (Pass 7).
 *
 * Algorithm in steps (matches the 7-pass spine):
 *   throughput = (1, 1, 1)            what's left of the photon's energy
 *   color      = (0, 0, 0)            accumulated radiance
 *   for depth = 0 .. MAX_DEPTH:
 *     PASS 2: hit ← scene_hit(ray)
 *             if MISS: break (background = black)
 *     PASS 3: mat ← shade_at_hit(hit)
 *             if mat is emissive:
 *               color += throughput · emission       contribute
 *               break                                 (T4 step 11)
 *     PASS 6: if depth ≥ RR_DEPTH:
 *               if !russian_roulette(throughput): break
 *     PASS 5: throughput = throughput_chain(throughput, mat.albedo)
 *     PASS 4: ray.dir   ← sample_bounce(hit.N)
 *             ray.origin ← hit.P + ε · hit.N         self-int. push
 *   return color
 *
 * Iterative, not recursive. Iterative is clearer pedagogically because
 * the throughput chain (Pass 5) is a single line you can point at —
 * `throughput = throughput_chain(throughput, mat.albedo)`. With
 * recursion the same chain is hidden in the call/return structure.
 *
 * ─────────────────────────────────────────────────────────────────── */

static V3 path_trace(Ray ray, Rng *rng_state) {
  V3 color = v3(0, 0, 0);
  V3 throughput = v3(1, 1, 1);

  for (int depth = 0; depth < MAX_DEPTH; depth++) {

    /* PASS 2 — SCENE INTERSECTION. */
    Hit hit;
    if (!scene_hit(ray, RAY_EPS, &hit))
      break;

    /* PASS 3 — SHADING DECISION. */
    const Mat *mat = shade_at_hit(&hit);
    if (mat_is_light(mat)) {
      /* Emissive: contribute throughput · emission and stop. */
      color = v3add(color, v3mul(throughput, mat->emit));
      break;
    }

    /* PASS 6 — TERMINATION (Russian roulette after depth warm-up). */
    if (depth >= RR_DEPTH && !russian_roulette(&throughput, rng_state))
      break;

    /* PASS 5 — THROUGHPUT CHAIN. */
    throughput = throughput_chain(throughput, mat->albedo);

    /* PASS 4 — BOUNCE DIRECTION SAMPLING.  Push origin off surface. */
    ray.dir = sample_bounce(hit.N, rng_state);
    ray.origin = v3add(hit.P, v3s(RAY_EPS, hit.N));
  }
  return color;
}

/* ── §13 PASS 7 — ACCUMULATION + TONE MAPPING ───────────────────────── *
 *
 * Pass 7 averages many samples per pixel into a clean image and
 * presents the result to the screen.
 *
 * Three sub-stages:
 *   §13.1 accumulator buffer + reset
 *   §13.2 accum_add_frame   — cast `spp` fresh samples per cell
 *   §13.3 tone-map + draw   — divide, tone-map, ASCII-quantise, paint
 *
 * The accumulator stays in LINEAR HDR space at all times. Tone
 * mapping happens only at draw time, AFTER dividing by sample count
 * (T10). Static arrays (no malloc) sized to the largest terminal
 * we'll encounter; on resize we reset rather than reallocate — the
 * previous accumulator is meaningless at the new resolution anyway.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §13.1 ── accumulator + reset ──────────────────────────────────────── */

static float g_accum[MAX_H][MAX_W][3];
static int g_samples = 0;

static void accum_reset(void) {
  memset(g_accum, 0, sizeof g_accum);
  g_samples = 0;
}

/* forward declaration — debug overlays defined in §14. */
static V3 first_hit_viz(Ray ray, ShadeMode mode);

/* §13.2 ── accum_add_frame: cast `spp` samples per pixel ────────────── */

/*
 * accum_add_frame — one frame of additional samples.
 *
 * Pseudocode:
 *   for each row:
 *     for each col:
 *       sample_sum = (0, 0, 0)
 *       for s = 0 .. spp − 1:
 *         seed     = rng_seed(col, row, frame_idx * spp + s)
 *         jitter_x = uniform(-0.5, +0.5)         sub-pixel anti-aliasing
 *         jitter_y = uniform(-0.5, +0.5)
 *         primary  = camera_ray(col, row, ..., jx, jy)   PASS 1
 *         radiance = (mode == PT)
 *                    ? path_trace(primary, &seed)        PASSES 2-6
 *                    : first_hit_viz(primary, mode)      debug
 *         sample_sum += radiance
 *       g_accum[row][col] += sample_sum                  PASS 7 (sum)
 *   g_samples += spp                                     PASS 7 (count)
 *
 * No tone-mapping here — that happens in accum_draw so we keep the
 * accumulator in linear HDR space. Tone-mapping the running sum each
 * frame would compound: the right place is "after divide".
 *
 * Frame-decorrelated seed (frame * spp + s) ensures back-to-back
 * frames don't redraw the same noise pattern.
 */
static void accum_add_frame(int cols, int rows, int spp, int frame_idx,
                            ShadeMode mode) {
  for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
    for (int col = 0; col < cols && col < MAX_W; col++) {
      float sum_r = 0.f, sum_g = 0.f, sum_b = 0.f;

      for (int s = 0; s < spp; s++) {
        Rng rng_state = rng_seed(col, row, frame_idx * spp + s);

        /* Sub-pixel jitter for free anti-aliasing. */
        float jitter_x = rng_f(&rng_state) - 0.5f;
        float jitter_y = rng_f(&rng_state) - 0.5f;

        /* PASS 1 — primary ray. */
        Ray primary = camera_ray(col, row, cols, rows, jitter_x, jitter_y);

        /* PASSES 2-6 (or debug short-circuit). */
        V3 radiance = (mode == MODE_PT) ? path_trace(primary, &rng_state)
                                        : first_hit_viz(primary, mode);
        sum_r += radiance.x;
        sum_g += radiance.y;
        sum_b += radiance.z;
      }

      g_accum[row][col][0] += sum_r;
      g_accum[row][col][1] += sum_g;
      g_accum[row][col][2] += sum_b;
    }
  }
  g_samples += spp;
}

/* §13.3 ── tone-mapping + draw ──────────────────────────────────────── */

static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) {
  return powf(x < 0.f ? 0.f : (x > 1.f ? 1.f : x), 1.f / 2.2f);
}
static inline float rec601_luma(float r, float g, float b) {
  return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

/*
 * accum_draw — paint the running average to screen.
 *
 * Per cell:
 *   1. linear average:    L = accum / samples            (T10)
 *   2. Reinhard tone-map: L' = L / (1 + L)               (HDR → [0, 1))
 *   3. gamma encode:      out = L'^(1/2.2)               (linear → sRGB)
 *   4. luminance → ASCII density char
 *   5. RGB → nearest 6×6×6 xterm cube colour pair
 *
 * Step 1 must happen BEFORE tone-mapping — tone-mapping a sum of N
 * samples is not the same as N times the tone-map of a single sample.
 * (Tone-map is non-linear.)
 */
static void accum_draw(int cols, int rows) {
  if (g_samples == 0)
    return;
  float inv_samples = 1.f / (float)g_samples;

  for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
    for (int col = 0; col < cols && col < MAX_W; col++) {
      float r = g_accum[row][col][0] * inv_samples;
      float g = g_accum[row][col][1] * inv_samples;
      float b = g_accum[row][col][2] * inv_samples;

      r = gamma_enc(reinhard(r));
      g = gamma_enc(reinhard(g));
      b = gamma_enc(reinhard(b));

      float luma = rec601_luma(r, g, b);
      int ramp_index = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
      if (ramp_index < 0)
        ramp_index = 0;
      if (ramp_index >= RAMP_LEN)
        ramp_index = RAMP_LEN - 1;
      char ch = k_ramp[ramp_index];

      int red_5bit = (int)(r * 5.f + 0.5f);
      int green_5bit = (int)(g * 5.f + 0.5f);
      int blue_5bit = (int)(b * 5.f + 0.5f);
      if (red_5bit > 5)
        red_5bit = 5;
      if (red_5bit < 0)
        red_5bit = 0;
      if (green_5bit > 5)
        green_5bit = 5;
      if (green_5bit < 0)
        green_5bit = 0;
      if (blue_5bit > 5)
        blue_5bit = 5;
      if (blue_5bit < 0)
        blue_5bit = 0;
      int pair_index =
          PAIR_CUBE_BASE + red_5bit * 36 + green_5bit * 6 + blue_5bit;
      attron(COLOR_PAIR(pair_index) | A_BOLD);
      mvaddch(row, col, (chtype)(unsigned char)ch);
      attroff(COLOR_PAIR(pair_index) | A_BOLD);
    }
  }
}

/* ── §14 debug overlays (visualise individual passes) ───────────────── *
 *
 * The debug modes short-circuit the path tracer at the FIRST hit and
 * paint an intermediate quantity. Each mode lets you SEE the output
 * of one or two passes in isolation:
 *
 *   MODE_NORMAL  Pass 1 + Pass 2:   (N + 1) / 2 as RGB
 *   MODE_ALBEDO  Pass 1 + 2 + 3:    material albedo (no lighting)
 *   MODE_DEPTH   Pass 1 + Pass 2:   1 / (1 + t) as gray
 *
 * No bounces, no randomness — these modes converge in a single
 * sample. Cycle them with `d` to compare against the full PT result.
 *
 * ─────────────────────────────────────────────────────────────────── */

static V3 first_hit_viz(Ray ray, ShadeMode mode) {
  Hit hit;
  if (!scene_hit(ray, RAY_EPS, &hit))
    return v3(0, 0, 0); /* miss */

  const Mat *mat = shade_at_hit(&hit);

  switch (mode) {
  case MODE_NORMAL:
    /* Encode normal as RGB ∈ [0, 1]³ — each component −1..1 → 0..1. */
    return v3(hit.N.x * 0.5f + 0.5f, hit.N.y * 0.5f + 0.5f,
              hit.N.z * 0.5f + 0.5f);

  case MODE_ALBEDO:
    /* Material body colour. Lights show as a clamped slice of
     * emission so they read bright but don't blow out the cube. */
    if (mat_is_light(mat)) {
      return v3(fminf(mat->emit.x * 0.06f, 1.f),
                fminf(mat->emit.y * 0.06f, 1.f),
                fminf(mat->emit.z * 0.06f, 1.f));
    }
    return mat->albedo;

  case MODE_DEPTH: {
    /* 1 / (1 + t): closer surfaces brighter, distant darker.
     * Output as gray (R=G=B). */
    float v = 1.f / (1.f + hit.t);
    return v3(v, v, v);
  }

  default:
    return v3(0, 0, 0);
  }
}

/* ── §15 screen ─────────────────────────────────────────────────────── */

static int g_have_256 = 0;

/* §15.1 ── color_init: 6×6×6 cube + reserved HUD/HINT/bar pairs ────── */

static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);

  if (g_have_256) {
    /* 216-colour xterm cube: pairs PAIR_CUBE_BASE..+215. */
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow         */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan           */
    init_pair(PAIR_BAR_FILL, 46, -1);   /* bright green (filled) */
    init_pair(PAIR_BAR_EMPTY, 240, -1); /* dim grey (empty)      */
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_BAR_FILL, COLOR_GREEN, -1);
    init_pair(PAIR_BAR_EMPTY, COLOR_WHITE, -1);
  }
}

/* §15.2 ── progress bar (samples / ACCUM_CAP) ───────────────────────── */

/*
 * draw_progress_bar — visualise convergence as samples / ACCUM_CAP.
 *
 * The bar fills as samples accumulate; once full, the HUD shows
 * "CONVERGED" and the main loop stops adding new samples (so the CPU
 * doesn't burn refining a converged image past visual gain).
 *
 * Width scales to ≈ cols/3 with bounds [8, 60] so it stays useful on
 * both narrow and wide terminals.
 */
static void draw_progress_bar(int row, int cols, int samples) {
  int bar_width = cols / 3;
  if (bar_width < 8)
    bar_width = 8;
  if (bar_width > 60)
    bar_width = 60;
  int filled = (int)((float)samples / (float)ACCUM_CAP * bar_width);
  if (filled > bar_width)
    filled = bar_width;

  int x = 1;
  attron(COLOR_PAIR(PAIR_HUD));
  mvaddch(row, x++, '[');
  for (int i = 0; i < bar_width; i++) {
    bool on = (i < filled);
    int pair = on ? PAIR_BAR_FILL : PAIR_BAR_EMPTY;
    int attr = on ? A_BOLD : A_DIM;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(row, x++, (chtype)(unsigned char)(on ? '=' : '-'));
    attroff(COLOR_PAIR(pair) | attr);
  }
  attron(COLOR_PAIR(PAIR_HUD));
  mvaddch(row, x++, ']');
  attroff(COLOR_PAIR(PAIR_HUD));
}

/* §15.3 ── hud_draw (HUD spec compliant) ────────────────────────────── */

/*
 * hud_draw — three HUD elements:
 *
 *   row 0 (top, yellow + BOLD)         status: fps / spp / samples /
 *                                      mode / state
 *   row 1 (top, yellow + progress bar) convergence visualisation
 *   row rows-1 (bottom, cyan + BOLD)   key hint strip
 */
static void hud_draw(int cols, int rows, float fps, int spp, int samples,
                     ShadeMode mode, bool paused) {
  /* §15.3.1 status — top-right. */
  char buf[140];
  snprintf(buf, sizeof buf, " %5.1f fps  spp:%d  samples:%-5d  mode:%s  %s ",
           (double)fps, spp, samples, shade_mode_name(mode),
           paused                 ? "PAUSED   "
           : (mode != MODE_PT)    ? "instant  "
           : samples >= ACCUM_CAP ? "CONVERGED"
                                  : "tracing  ");
  int len = (int)strlen(buf);
  if (len > cols)
    len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* §15.3.2 title — top-left, same row. */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " PATH TRACER · CORNELL BOX ");
  attroff(COLOR_PAIR(PAIR_HUD));

  /* §15.3.3 progress bar — row 1. Only meaningful in PT mode (debug
   * modes converge in 1 sample so the bar would just show "tiny
   * progress, full bar"). Hide it in debug modes. */
  if (rows > 4 && mode == MODE_PT)
    draw_progress_bar(1, cols, samples);

  /* §15.3.4 hint — bottom row, cyan + BOLD. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0, " q:quit  spc/p:pause  r:reset  d:mode  +/-:spp ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §16 app ────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_run = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);
  atexit(cleanup);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1); /* prevent input tearing */

  int cols, rows;
  getmaxyx(stdscr, rows, cols);
  color_init();
  accum_reset();

  int spp = SPP_DEFAULT;
  bool paused = false;
  ShadeMode mode = MODE_PT;
  float fps = 0.f;
  long long fps_acc = 0;
  int fps_cnt = 0;
  long long frame_ns = 1000000000LL / TARGET_FPS;
  long long last = clock_ns();
  int frame_idx = 0;

  while (g_run) {

    /* §16.1 resize — invalidate accumulator, restart frame indexing. */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      accum_reset();
      frame_idx = 0;
    }

    /* §16.2 timing — wall clock dt with cap. */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* §16.3 fps rolling average over half-second windows. */
    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= 500000000LL) {
      fps = (float)fps_cnt * 1e9f / (float)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    /* §16.4 add samples — auto-pause once converged in PT mode.
     *
     * Debug modes (NORMAL/ALBEDO/DEPTH) are deterministic so we
     * add one frame's worth and stop (no point re-rendering the
     * same image every frame). */
    bool can_sample = !paused && (mode != MODE_PT ? (g_samples == 0)
                                                  : (g_samples < ACCUM_CAP));
    if (can_sample)
      accum_add_frame(cols, rows, spp, frame_idx++, mode);

    /* §16.5 paint frame. */
    long long frame_start = clock_ns();
    erase();
    accum_draw(cols, rows);
    hud_draw(cols, rows, fps, spp, g_samples, mode, paused);
    wnoutrefresh(stdscr);
    doupdate();

    /* §16.6 input. Resetting the accumulator on +/- (SPP changes)
     * and on 'd' (mode changes) because the average mixes paths
     * with different sampling rates / output ranges otherwise. */
    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27 /* ESC */:
      g_run = 0;
      break;
    case ' ':
    case 'p':
    case 'P':
      paused = !paused;
      break;
    case 'r':
    case 'R':
      accum_reset();
      frame_idx = 0;
      break;
    case '+':
    case '=':
      if (spp < SPP_MAX)
        spp++;
      accum_reset();
      frame_idx = 0;
      break;
    case '-':
    case '_':
      if (spp > SPP_MIN)
        spp--;
      accum_reset();
      frame_idx = 0;
      break;
    case 'd':
    case 'D':
      mode = (ShadeMode)((mode + 1) % MODE_N);
      accum_reset();
      frame_idx = 0;
      break;
    default:
      break;
    }

    /* §16.7 frame cap. */
    clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
  }
  return 0;
}
