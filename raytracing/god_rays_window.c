/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * god_rays_window.c — volumetric path tracer · Islamic-arch interior
 *
 * DEMO: A dim mosque-style interior. Two rows of pointed (lancet)
 *       arches in the wall ahead — 5 columns each row, 10 windows
 *       total. Sky and a small white sun show through the openings.
 *       Uniform mist fills the room; wherever the eye ray points
 *       roughly toward the sun and the chord-to-sun is unblocked by
 *       a wall section, volumetric scattering produces a bright
 *       shaft. The sun traces a CIRCULAR orbit in (azimuth,
 *       elevation): low-elevation passes always sweep right→left
 *       (lower row lit), high-elevation passes always sweep
 *       left→right (upper row lit). Two crossing planes of shafts
 *       produce the iconic "mosque interior at golden hour" feel.
 *
 *       Cycle four debug overlays with `d' (NORMAL → SCATTER →
 *       SURFACE → TR) to see each component of the rendering
 *       equation in isolation.
 *
 * CORE TECHNIQUE: per pixel, ray-march from the camera through
 * uniform interior mist; at each sample point, NEE (next event
 * estimation) asks "can I see the sun from here, slit-aware?" and
 * accumulates Henyey-Greenstein-weighted in-scattered radiance from
 * any unblocked sample. The "slit-aware" wall test is the single
 * line of code that produces god rays — it asks whether the chord
 * from the scatter point toward the sun passes through one of the
 * 10 pointed-arch openings or hits solid wall.
 *
 * Section map (the spine):
 *   §1  config             dimensions, sun orbit, mist, presets
 *   §2  clock              monotonic timer + sleep
 *   §3  vec3 math          V3 + ops
 *   §4  rgb math           RGB + ops
 *   §5  blackbody          kelvin → RGB (Tanner-Helland)
 *   §6  tone map           Reinhard + 1/2.2 gamma
 *   §7  atmospheric        uniform σ_e + analytic Tr_to_sun
 *   §8  phase function     Henyey-Greenstein
 *   §9  RNG                hash3 + hash01 (window x-jitter)
 *   §10 ncurses paint      6×6×6 cube quantise + glyph ramp
 *   §11 ray-plane          ray vs horizontal plane (floor)
 *   §12 arch geometry      Window struct + place_windows + lancet test
 *   §13 ray-wall           slit-aware wall intersection
 *   §14 scene_hit          nearest of {wall, floor, sky}
 *   §15 sun visibility     scene_blocked_to_sun (slit-aware NEE)
 *   §16 sky / sun          gradient + sun-disc radiance
 *   §17 surface shading    shade_wall + shade_floor (Lambertian + NEE)
 *   §18 trace_ray (CORE)   volumetric path tracer, all components
 *   §19 scene state        Scene + sun motion + reseed
 *   §20 camera             pinhole + pitch (frame both rows)
 *   §21 screen + scene_draw includes debug-overlay selection
 *   §22 HUD                status row + key-hint strip
 *   §23 app                signals, main loop, input
 *
 * Companion files using related path-tracer kernels:
 *   raytracing/forest_god_rays.c   bare-forest exterior, height-decay mist
 *   raytracing/solar_eclipse.c     eclipse with corona / chromosphere
 *
 * KELVIN PRESETS (`t' / `T' cycles):
 *   DAWN     2000K   deep red       GOLDEN   3500K   warm orange
 *   NOON     5500K   white          DUSK     2500K   orange
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause sun + scene
 *   r            reseed window x-jitter (count stays 2×5)
 *   t / T        next / previous kelvin preset
 *   d            cycle debug overlay (NORMAL / SCATTER / SURFACE / TR)
 *   z / Z        zoom in / out (0.25× .. 4×)
 *   + / =        faster sun motion (-: slower)
 *   ] / [        raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/god_rays_window.c \
 *       -o god_rays_window -lncurses -lm
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
 *   3. §12 arch geometry + §18 trace_ray — the two heart sections.
 *      §12 contains the pointed-arch point-in-shape test (T6 has the
 *      derivation); §18 is the volumetric integrator (~50 lines of
 *      body fusing passes 2-7 of T3 into one loop).
 *   4. The supporting sections in any order. §7 (medium), §8 (phase),
 *      §15 (NEE block test) are the pieces trace_ray uses.
 *   5. §10/§20-§23 are infrastructure (ncurses + main loop). Skip
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
 *   - Exponential decay; the form T(t) = exp(-σt) for a uniform medium.
 *   - Quadratic formula and the equation of a circle in 2-D.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Surface BRDFs beyond Lambertian.
 *   - Mie / Rayleigh scattering theory; we use Henyey-Greenstein
 *     as a phenomenological phase function and don't derive it.
 *   - Spatial-acceleration structures (BVH, KD-tree).
 *
 * This file is the INDOOR sibling of forest_god_rays.c. The core
 * algorithm is identical (volumetric PT with NEE). Differences are
 * scene-specific:
 *
 *   forest_god_rays.c              god_rays_window.c (this file)
 *   ─────────────────              ────────────────────────────
 *   outdoors, daylight             indoor, dim
 *   cylinder-trunk occluders       wall-with-arched-windows occluder
 *   HEIGHT-DECAYING mist           UNIFORM mist
 *   iterate every trunk            iterate every window (10)
 *   sun above horizon (~12°)       sun (low elev band 4°-13°)
 *   horizontal camera              pitched camera (frames both rows)
 *   independent sin periods        SINGLE-period CIRCULAR orbit
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Single-bounce volumetric path tracing with
 *                  Next Event Estimation (NEE). For each pixel we
 *                  RAY-MARCH from the camera through uniform mist.
 *                  At each march step we ask "is the sun visible
 *                  from here?" — i.e., does the chord from this
 *                  scatter point toward the sun pass cleanly through
 *                  one of the 10 pointed arches? If yes, we
 *                  accumulate Henyey-Greenstein-weighted in-scattered
 *                  radiance. The visible shafts are NEGATIVE space —
 *                  the regions where the wall does not block the
 *                  chord. Their cross-sections inside the medium read
 *                  as pointed-arch silhouettes too.
 *
 *                  At the far end of the march (a wall section, the
 *                  floor, or sky through a window) we add the surface
 *                  or sky radiance dimmed by the remaining
 *                  transmittance along the eye ray.
 *
 *                  See §18 trace_ray for the integrator. The whole
 *                  algorithm fits in ~50 lines of body.
 *
 * Data-structure : Static window array (10 entries — 2 rows × 5
 *                  cols). Per-frame per-pixel HDR float RGB
 *                  accumulator (g_buf) so tone-mapping happens once
 *                  per frame at the ncurses-paint stage. No malloc.
 *
 * Rendering      : Reinhard tone-map L/(1+L) → 1/2.2 gamma → 6×6×6
 *                  xterm cube colour pair + 16-glyph density ramp.
 *                  Bright cells get A_BOLD, dark cells A_DIM, so the
 *                  ASCII output retains visible dynamic range even
 *                  on terminals that quantise the cube aggressively.
 *
 * Performance    : ~32 march steps × cell-count + 1 NEE test/step.
 *                  At 200×60 cells, MARCH_STEPS=32 ≈ 380k tests/frame.
 *                  Each test does a single ray-plane crossing then
 *                  iterates 10 windows with a constant-time arch
 *                  test each. Modern CPU: ~8 ms/frame, comfortable
 *                  at 30 Hz with budget left for the HUD and tone-map.
 *
 * References     : Pharr, Jakob & Humphreys, "Physically Based
 *                    Rendering" 4e, https://pbr-book.org. Chapter 14
 *                    (light transport in participating media) is
 *                    the definitive treatment.
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
 *                  Tabbaa, Y., "An Introduction to Islamic
 *                    Architectural Geometry", Cambridge 2018.
 *                    Pointed-arch profile derivations (§9, T6).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three concepts stacked:
 *
 *   1. LIGHT IN FOG. Mist scatters some light from any photon
 *      passing through it — including photons travelling sideways
 *      relative to the camera. So you can SEE the sun's beams
 *      from the side.
 *
 *   2. THE BEAM IS WHERE THE SUN HAS LINE OF SIGHT. Anywhere the
 *      wall blocks the sun, the beam stops — but the mist remains,
 *      lit only by indirect (very dim) terms. The beam shape inside
 *      the mist is the projection of the wall's openings onto the
 *      volume of the room.
 *
 *   3. POINTED-ARCH HOLES. The 10 windows are pointed (lancet)
 *      arches: rectangle below, two circular arcs meeting at a peak
 *      above. The beam shape inside the mist is therefore a row of
 *      pointed-arch silhouettes in light — the iconic mosque-interior
 *      look.
 *
 * The integrator evaluates the volumetric rendering equation with
 * NEE (one shadow ray per scatter point):
 *
 *   L_eye = Σ_i  T_eye(t_i) · σ · phase(θ) · Tr(p_i→sun) · vis(p_i, sun)
 *                          · L_sun · dt
 *         + T_eye(t_max) · L_surface
 *
 * where vis() is the slit-aware wall test in §15 and Tr is the
 * closed-form interior-mist transmittance in §7.2.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *   Side view (camera at left, two-row window grid in wall ahead):
 *
 *      +─────────────────────────────────+
 *      |    UNIFORM MIST                 |    sky
 *      |                                 |
 *      |       ╲       /\                |       ☀ sun (drifts in
 *      |        ╲     /  \  ←── upper    |          el and az on a
 *      |         ╲   / row arches        |          CIRCULAR orbit)
 *      |     ----╲--/----\---            |
 *      |        eye      /\              |
 *      |         │      /  \  ←── lower  |
 *      |     C ──┼──────────────         |
 *      |         │     /    \  row       |
 *      |     ───┴────/--------           |
 *      |          ↑   march steps        |    sky
 *      +─────────────────────────────────+
 *      ←     T_eye(t) decays with σ      →
 *      camera         pointed-arch wall                sun
 *
 *   Each `↑' along the eye ray is one march step. At each one, we
 *   ask: "if a photon were here and tried to fly toward the sun,
 *   would it pass through one of the arches?" — that is the NEE
 *   test. If yes, we add a sliver of in-scattered radiance to this
 *   pixel.
 *
 * POINTED-ARCH GEOMETRY (the distinctive bit)
 * ───────────────────────────────────────────
 * A pointed (lancet) arch is the union of a rectangle and the
 * INTERSECTION of two circles. Each circle has its centre on the
 * springing point OPPOSITE the arc it draws — that is, the bottom
 * corner of the arch on the side OPPOSITE the curve.
 *
 *                          peak
 *                           ↑
 *                          /|\
 *                         / | \
 *                        /  |  \
 *                       /   |   \
 *                  R=2w/    .    \R=2w   ← each arc radius = 2·half_w
 *                     /     |     \
 *                    /      |      \
 *                  •────────────────•    ← top of rect body (cy_top)
 *                  |                |
 *                  |   rect body    |
 *                  |                |
 *                  +────────────────+    ← sill (cy_base)
 *                  ↑       ↑        ↑
 *             cx-half_w   cx    cx+half_w
 *             (left            (right
 *             springing)        springing)
 *
 *   Left arc centre  : (cx + half_w, cy_top)   [right springing]
 *   Right arc centre : (cx − half_w, cy_top)   [left  springing]
 *   Peak             : (cx, cy_top + √3 · half_w)
 *
 *   For (x, y) to be INSIDE the arch:
 *     |x − cx| ≤ half_w                                       (x bounds)
 *     y ≥ cy_base                                              (above sill)
 *     y ≤ cy_top + √3·half_w                                   (below peak)
 *     IF y ≤ cy_top:  inside rect → INSIDE
 *     ELSE:           must be inside BOTH arcs:
 *                       (x − (cx+half_w))² + (y − cy_top)² ≤ (2·half_w)²
 *                       (x − (cx−half_w))² + (y − cy_top)² ≤ (2·half_w)²
 *
 *   See T6 for the derivation; §12 xy_in_any_window for the code.
 *
 * ALGORITHM IN STEPS  (per pixel)
 * ───────────────────────────────
 *   1. Generate primary ray from the camera (with PITCH so both
 *      rows fit in the frame).
 *   2. Find what the scene hits — wall, floor, or sky-through-window.
 *      Set march_distance = min(surface t, ray's wall-plane crossing).
 *   3. step_length = march_distance / MARCH_STEPS.
 *   4. Compute phase_value once — depends on (eye, sun) angle.
 *   5. transmittance_along_eye = 1.0
 *      in_scatter_radiance     = 0
 *   6. For i = 0 .. MARCH_STEPS-1, while transmittance_along_eye > ε:
 *        a. scatter_point = ray_origin + ray_dir · ((i + 0.5) · step_length)
 *        b. extinction = σ_e(scatter_point)
 *        c. if extinction > 0:
 *             d. blocked = scene_blocked_to_sun(scatter_point, sun_dir)
 *             e. if NOT blocked:
 *                  Tr_to_sun = exp(-σ · d_to_wall)        closed form
 *                  step_contribution = extinction · phase_value
 *                                    · Tr_to_sun · GAIN · step_length
 *                  in_scatter_radiance += sun_emit · step_contribution
 *                                       · transmittance_along_eye
 *             f. transmittance_along_eye *= exp(-extinction · step_length)
 *   7. surface_radiance = sky/wall/floor at far hit.
 *   8. final_radiance = in_scatter_radiance
 *                     + surface_radiance · transmittance_along_eye
 *   9. tone-map → cube colour + glyph → ncurses paint.
 *
 * KEY FORMULAS
 * ────────────
 *   Volumetric rendering equation (1-bounce, NEE):
 *     L_eye = ∫ T(t) σ phase(θ) Tr(p→sun) vis(p, sun) L_sun dt
 *           + T(t_max) L_surface
 *
 *   Beer-Lambert (uniform σ inside the room):
 *     T(0, t)        = exp(-σ · t)
 *     Tr_to_sun(p)   = exp(-σ · d_to_wall),
 *                       d_to_wall = (WALL_Z − p.z) / sun_dir.z
 *
 *   Pointed-arch peak height:
 *     h_arch = √3 · half_w  ≈  1.732 · half_w
 *
 *   Henyey-Greenstein:
 *     P(cosθ; g) = (1 − g²) / [4π · (1 + g² − 2g·cosθ)^1.5]
 *
 *   Single-period circular sun orbit:
 *     el = SUN_EL_BASE + SUN_EL_AMP · cos(ωt)
 *     az = SUN_AZ_BASE + SUN_AZ_AMP · sin(ωt)
 *     ⇒ d(az)/dt and (el-base) have the same sign — high el always
 *       coincides with az going L→R, low el with R→L.
 *
 *   Reinhard tone map + gamma:
 *     L' = L / (1 + L);   out = L'^(1/2.2)
 *
 * WORKED EXAMPLE  (one pixel, four march steps, by hand)
 * ──────────────────────────────────────────────────────
 *   Pixel near the centre of the frame, looking through one of the
 *   upper-row arches. March from camera (z=0) toward the wall (z=4):
 *
 *     i  step_t  scatter_p z  σ    visible?  Tr_to_sun
 *     0   0.083    0.50   0.20  YES        exp(-0.20·3.5) ≈ 0.50
 *     1   0.250    1.50   0.20  YES        exp(-0.20·2.5) ≈ 0.61
 *     2   0.417    2.50   0.20  YES        exp(-0.20·1.5) ≈ 0.74
 *     3   0.583    3.50   0.20  YES        exp(-0.20·0.5) ≈ 0.90
 *
 *   Each step's contribution = σ · phase · Tr_to_sun · GAIN · dt · T(0, step).
 *   The T(0, t) factor decays as exp(-0.20·step_t); over 4m total we
 *   accumulate plenty of in-scatter then dim the sky beyond by the
 *   final T_far (about exp(-0.8) ≈ 0.45 for the un-shaft-ed sky).
 *
 *   For a pixel where the eye ray ends up looking at SOLID WALL (not
 *   a window), every step still gets contribution from in-scatter
 *   (the mist scatters whether or not the camera sees the sun
 *   directly). But the wall blocks the surface contribution → that
 *   pixel sees ONLY the cone glow, no sun disc.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The pointed arch's two arcs meet TANGENTIALLY at the springing
 *    line (y = cy_top), so there is no gap or overlap with the
 *    rectangle below — clean profile.
 *  • Sun elevation needs to span both rows' apparent y-positions on
 *    the wall, otherwise only one row's shafts visibly switch
 *    on/off as the sun drifts. The current el ∈ [0.07, 0.23] rad
 *    spans apparent y ∈ [1.88, 2.55] m, covering both rows.
 *  • Camera pitch is computed from ROW-CENTRE midpoint, not row 0
 *    centre — so both rows fit in the frame.
 *  • CIRCULAR sun orbit (T8) — el uses cos, az uses sin, same period.
 *    This deliberately couples them (low el ↔ az going R→L).
 *  • Tone-map AFTER accumulate. Reinhard is non-linear.
 *  • UNIFORM σ. The closed-form Tr_to_sun assumes uniform mist
 *    inside the room. Don't add height variation without also
 *    promoting Tr_to_sun to a numerical integral.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `t' to cycle DAWN → GOLDEN → NOON → DUSK; the shafts and
 *    horizon tint should warm/cool. Even at NOON the shafts have a
 *    slight cool tilt because the sky-zenith is constant cool blue.
 *  • Wait one full sun period (200 s). The sun should make exactly
 *    one circular orbit. Quarter-period checkpoints:
 *      t=0     az centred,   el high  → upper row lit, az starting L→R
 *      t=50s   az right max, el centre → crossing
 *      t=100s  az centred,   el low   → lower row lit, az starting R→L
 *      t=150s  az left  max, el centre → crossing
 *  • Press `r' — windows wiggle horizontally (jitter on cx); grid
 *    count stays 2×5.
 *  • Press `d' to cycle:
 *      NORMAL    full render
 *      SCATTER   in-scatter only — only the shafts of light, no
 *                walls or sky visible. Pointed-arch silhouettes in
 *                light, drifting with the sun.
 *      SURFACE   surface only (no in-scatter) — bare scene with
 *                wall + floor + sky-through-windows.
 *      TR        gray-scale eye-ray transmittance at far hit.
 *  • In any mode, cross-section a shaft near the wall — should read
 *    as a POINTED-ARCH silhouette (not a round-arch cathedral).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eleven short tutorials that build volumetric path tracing for an
 * Islamic-arch interior from first principles. Read in order; each
 * builds on the previous.
 *
 *   T1   The volumetric rendering equation — what we are computing
 *   T2   Beer-Lambert: how light dies in UNIFORM mist
 *   T3   The 7 stages of this file's path tracer
 *   T4   Henyey-Greenstein: the BRDF for fog
 *   T5   NEE: shadow rays as cheap variance reduction
 *   T6   Pointed-arch geometry — the union-of-rect-and-two-circles
 *   T7   Two-row layout + camera pitch — framing the grid
 *   T8   Sun motion: Lissajous → circle (the synced choreography)
 *   T9   Blackbody — the warmth of the light
 *   T10  Tone mapping HDR → ASCII
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
 * where vis() is the slit-aware wall test (T6), Tr is the medium
 * transmittance from p_i to where the chord exits the room (closed
 * form because σ is uniform), and θ_i is the angle between the eye
 * ray and the sun direction.
 *
 * Read §18 trace_ray after T2 and T3 — that one summation IS the body
 * of the loop.
 *
 * T2  BEER-LAMBERT: HOW LIGHT DIES IN UNIFORM MIST
 * ─────────────────────────────────────────────────
 * A photon flying through mist has, per unit length travelled, a
 * constant probability σ_e of being absorbed or scattered out of its
 * path. The fraction of photons surviving a length t obeys
 *
 *     dT/dt = -σ_e · T(t)
 *     T(t)  = T(0) · exp(-σ_e · t)              UNIFORM medium
 *
 * For non-uniform σ_e, integrate (use a Riemann sum during the
 * march). For UNIFORM σ — our case — the integral is closed-form:
 *
 *     T(0, t) = exp(-σ_e · t)
 *
 * The integral inside the exponent is called OPTICAL DEPTH (τ).
 * Optical depth 1 = 1/e ≈ 37% transmittance left.
 *
 * In this file:
 *   · σ_e = MIST_SIGMA = 0.20 per metre (constant inside the room).
 *   · A 4-m chord has τ = 0.80 → T = exp(-0.80) ≈ 0.45.
 *   · Outside the room (z > WALL_Z, y < 0) σ_e = 0 — vacuum.
 *
 * The single biggest performance win in the file: Tr_to_sun for a
 * uniform-σ medium, bounded by the wall plane, has the closed form
 * exp(-σ · d_to_wall) — no second march required (cf. forest_god_rays.c
 * which uses the analogous closed-form for height-decaying mist).
 *
 * T3  THE 7 STAGES OF THIS FILE'S PATH TRACER
 * ────────────────────────────────────────────
 * Every volumetric path tracer has these seven conceptual stages:
 *
 *   Stage 1  CAMERA RAY GENERATION         (§20)
 *            pixel → world-space ray, with PITCH so both rows of
 *            arches are centred in the frame.
 *
 *   Stage 2  SCENE INTERSECTION            (§14)
 *            ray → nearest hit ∈ {wall, floor, sky}.
 *
 *   Stage 3  EYE-RAY MARCH                 (§18 main loop)
 *            split [0, march_distance] into MARCH_STEPS pieces.
 *            Visit the midpoint of each.
 *
 *   Stage 4  SUN VISIBILITY (NEE)          (§15)
 *            For each scatter point, ask: does the wall block the
 *            chord toward the sun? Slit-aware via xy_in_any_window
 *            (§12). This IS the trick that produces god rays.
 *
 *   Stage 5  IN-SCATTER CONTRIBUTION       (§18 inner block)
 *            σ · phase · Tr_to_sun · L_sun · dt · T(0, t).
 *            Henyey-Greenstein (§8) supplies phase; Beer-Lambert
 *            in closed form (§7.2) supplies Tr_to_sun.
 *
 *   Stage 6  EYE-RAY TRANSMITTANCE UPDATE  (§18 main loop)
 *            T(0, t) *= exp(-σ · step_length).
 *
 *   Stage 7  SURFACE CONTRIBUTION + TONE MAP   (§18 tail + §10)
 *            L_surface (wall/floor/sky) × T(0, march_distance).
 *            Sum with in-scatter; Reinhard + gamma + cube + glyph.
 *
 * If you can recite these seven by name you can READ §18 trace_ray.
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
 * core brightness with a soft halo.
 *
 * In trace_ray, phase_value is computed ONCE per pixel — it depends
 * only on the angle between the eye ray and the sun direction.
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
 * directions don't hit the sun, contributing zero. Wasteful with
 * a small bright light.
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
 *   · Sun is treated as DIRECTIONAL. sun_dir is computed in §19.
 *   · The shadow ray is virtual — we don't trace anything. We just
 *     ask scene_blocked_to_sun (§15) "does the wall block the chord
 *     from p in direction sun_dir?"
 *   · The sun has a finite angular radius (SUN_ANG_RADIUS) used by
 *     the sky-radiance disc test in §16. The NEE code uses sun_dir
 *     as a single direction — slightly biased when part of the disc
 *     is unblocked but most isn't, but visually indistinguishable
 *     at terminal resolution.
 *
 * T6  POINTED-ARCH GEOMETRY — UNION OF RECT AND TWO CIRCLES
 * ──────────────────────────────────────────────────────────
 * Every visual feature of this scene depends on the pointed-arch
 * test in §12. Get it wrong and the demo collapses; get it right
 * and "god rays through arches" emerges automatically. Let's derive
 * the geometry from first principles.
 *
 * THE SHAPE
 * ─────────
 * A pointed (lancet) arch has two parts:
 *   1. A RECTANGULAR BODY of width 2·half_w and height rect_h,
 *      sitting on a horizontal sill at y = cy_base.
 *   2. An ARCH ZONE above the rectangle, formed by two circular
 *      arcs that converge at a peak.
 *
 * THE TWO ARCS
 * ────────────
 * Each arc is a portion of a circle. The radius is 2·half_w (the
 * width of the springing line). Each arc's centre sits on the
 * springing point OPPOSITE the arc:
 *
 *   The LEFT-SIDE arc (visible curve on the left of the arch
 *   shape) is drawn from a circle whose centre is at the RIGHT
 *   springing point: (cx + half_w, cy_top).
 *
 *   Why? The arc must (a) start at the LEFT springing
 *   (cx - half_w, cy_top) — distance from the right springing centre
 *   is sqrt((2·half_w)² + 0) = 2·half_w ✓, and (b) reach the peak
 *   at (cx, cy_peak) — symmetric.
 *
 *   Symmetric: RIGHT-SIDE arc has centre at left springing
 *   (cx - half_w, cy_top), same radius 2·half_w.
 *
 * THE PEAK
 * ────────
 * The two arcs meet at a peak directly above cx. We find its y by
 * computing the intersection. At x = cx:
 *
 *   (cx − (cx + half_w))² + (y − cy_top)² = (2·half_w)²    (left arc)
 *      half_w² + (y − cy_top)² = 4·half_w²
 *      (y − cy_top)²        = 3·half_w²
 *      y − cy_top           = √3 · half_w
 *      y_peak               = cy_top + √3 · half_w
 *
 *   (Right arc gives the same y by symmetry.)
 *
 * So the arch's peak height above the rectangle's top is
 *
 *     h_arch = √3 · half_w  ≈  1.732 · half_w
 *
 * For half_w = 0.12 m:  h_arch ≈ 0.208 m. Total window height =
 * rect_h + h_arch = 0.32 + 0.208 = 0.528 m.
 *
 * THE POINT-IN-SHAPE TEST
 * ───────────────────────
 * Given a point (x, y), is it inside the pointed arch? The shape is
 * the UNION of:
 *
 *   A. Rectangle:  |x − cx| ≤ half_w  AND  cy_base ≤ y ≤ cy_top
 *   B. Arch zone:  cy_top < y ≤ cy_peak  AND
 *                    inside left arc circle  AND  inside right arc circle
 *
 * Pseudocode:
 *
 *   inside(x, y, w):
 *     dx = x - w.cx
 *     if |dx| > w.half_w:        outside in x      → false
 *     dy = y - w.cy_base
 *     if dy < 0:                 below sill        → false
 *     if dy <= w.rect_h:         inside rectangle  → true
 *     ay = dy - w.rect_h
 *     if ay > √3 · w.half_w:     above peak        → false
 *     --- in the arch zone — must be inside BOTH arcs ---
 *     left_arc:  (dx - w.half_w)² + ay² ≤ (2·w.half_w)²
 *     right_arc: (dx + w.half_w)² + ay² ≤ (2·w.half_w)²
 *     return  left_arc AND right_arc
 *
 * Why "AND BOTH arcs" rather than "OR EITHER arc"? Because we want
 * the INTERSECTION of the two circles — only the part where both
 * arcs overlap. Each arc alone covers more space than the lancet
 * shape; their intersection is the lancet. Verify:
 *
 *   At (cx − half_w, cy_top) (left springing, on rect-arch boundary):
 *     left arc : (-2·half_w)² + 0 = 4·half_w² ≤ 4·half_w² ✓ boundary
 *     right arc: (0)² + 0 = 0 ≤ 4·half_w² ✓ inside
 *     → both pass → inside the arch shape ✓ (matches the picture)
 *
 *   At (cx, cy_top + √3·half_w) (peak):
 *     left arc : (−half_w)² + (√3·half_w)² = 4·half_w² ≤ 4·half_w² ✓
 *     right arc: (+half_w)² + (√3·half_w)² = 4·half_w² ≤ 4·half_w² ✓
 *     → both pass at boundary → inside ✓
 *
 *   At (cx, cy_top + 0.5·half_w) (well inside arch zone):
 *     left arc : (−half_w)² + (0.5·half_w)² = 1.25·half_w² < 4·half_w² ✓
 *     right arc: (+half_w)² + (0.5·half_w)² = 1.25·half_w² < 4·half_w² ✓
 *     → both pass → inside ✓
 *
 *   At (cx + 0.6·half_w, cy_top + 1.5·half_w) (toward right arc edge):
 *     left arc : (0.6·half_w − half_w)² + (1.5·half_w)² = 0.16·half_w²
 *                 + 2.25·half_w² = 2.41·half_w² < 4·half_w² ✓
 *     right arc: (0.6·half_w + half_w)² + (1.5·half_w)² = 2.56·half_w²
 *                 + 2.25·half_w² = 4.81·half_w² > 4·half_w² ✗
 *     → fails right arc → OUTSIDE ✓ (this point is outside the lancet)
 *
 * The two-circle intersection gives the canonical lancet profile —
 * tangentially smooth where the arcs meet the rectangle, converging
 * to a sharp peak. See §12 for the implementation.
 *
 * T7  TWO-ROW LAYOUT + CAMERA PITCH — FRAMING THE GRID
 * ─────────────────────────────────────────────────────
 * A 2 rows × 5 cols = 10 windows. Both rows above eye level:
 *
 *   row0_sill  = CAMERA_HEIGHT + WIN_SILL0_OFFSET_Y
 *   row1_sill  = row0_sill + total_h · (1 + WIN_ROW_GAP_FRAC)
 *
 *   total_h    = WIN_RECT_H + √3 · WIN_HALF_W ≈ 0.528 m
 *   row0_sill  ≈ 1.65 m,  row1_sill ≈ 2.34 m
 *
 *   row0 centre y ≈ 1.91 m    row1 centre y ≈ 2.60 m
 *
 * To frame BOTH rows in the centre of the image, the camera is
 * pitched UP by an angle that aims its optical axis at the MIDPOINT
 * y of the two row centres (relative to the camera's height):
 *
 *   row0_centre_above = WIN_SILL0_OFFSET_Y + total_h/2
 *   row1_centre_above = row0_centre_above + row_period
 *   mid_y_above       = (row0_centre_above + row1_centre_above) / 2
 *   pitch             = atan(mid_y_above / WALL_Z)  ≈ atan(0.66/4) ≈ 9.4°
 *
 * In camera_ray (§20):
 *
 *   ray_local = (u, v, 1)                 (unrotated pinhole)
 *   ry =  v · cos(p)  +  1 · sin(p)        (rotate around X by p)
 *   rz = -v · sin(p)  +  1 · cos(p)
 *   ray_dir   = norm(u, ry, rz)
 *
 * So a pixel at the image centre (u=0, v=0) produces a ray of
 * direction (0, sin(p), cos(p)) — pointed slightly up.
 *
 * Why pitch UP rather than position the camera HIGHER: pitching
 * keeps the room geometry symmetric. If we raised the camera, the
 * floor would dominate the bottom of the frame and the windows
 * would compete with ceiling. Pitching gives clean horizontal
 * symmetry across both rows — the cones cant down toward the camera
 * from each window because of the pitch geometry.
 *
 * T8  SUN MOTION: LISSAJOUS → CIRCLE
 * ───────────────────────────────────
 * The sun's apparent y at the wall plane (z = WALL_Z) from the
 * camera (y = CAMERA_HEIGHT, z = 0) is
 *
 *     y_at_wall = CAMERA_HEIGHT + WALL_Z · tan(el)
 *
 * For our geometry this gives:
 *   el ≈ 0.077 rad (4.4°)  →  y ≈ 1.91 m  (lower-row centre)
 *   el ≈ 0.231 rad (13.2°) →  y ≈ 2.55 m  (upper-row centre)
 *
 * So an el sweep over [0.07, 0.23] rad alternates which row the sun
 * disc passes through, producing the visible alternation between
 * "lower row lit" and "upper row lit".
 *
 * NAIVE TWO-PERIOD MOTION (drifting Lissajous):
 *   el = SUN_EL_BASE + SUN_EL_AMP · sin(2π · t / 200)
 *   az = SUN_AZ_BASE + SUN_AZ_AMP · sin(2π · t / 280)
 *
 *   Different periods → the (az, el) phase relationship drifts; the
 *   "ideal" alignment of low-el ↔ R→L sun motion happens
 *   periodically (about every couple of minutes) but isn't locked.
 *
 * SINGLE-PERIOD CIRCULAR ORBIT (this file's choice):
 *   el = SUN_EL_BASE + SUN_EL_AMP · cos(ω · t)     (NB: cos)
 *   az = SUN_AZ_BASE + SUN_AZ_AMP · sin(ω · t)     (sin)
 *   ω  = 2π / SUN_EL_PERIOD_S
 *
 * Trajectory is a circle in (az, el) space:
 *   t=0       el max,     az = 0          → upper row, az starts L→R
 *   t=π/(2ω)  el = base,  az = max (right) → crossing
 *   t=π/ω     el min,     az = 0           → lower row, az starts R→L
 *   t=3π/(2ω) el = base,  az = min (left)  → crossing
 *
 * Why this works: d(az)/dt = SUN_AZ_AMP · ω · cos(ωt). At t = 0
 * (high el), this is +SUN_AZ_AMP·ω > 0 — az is INCREASING (L→R). At
 * t = π/ω (low el), it's −SUN_AZ_AMP·ω < 0 — az is DECREASING (R→L).
 * So d(az)/dt and (el − base) always share a sign — the desired
 * choreography.
 *
 * Trade-off: locked motion is more predictable (and looks great),
 * less "natural-random". The forest_god_rays.c file keeps the
 * drifting Lissajous because in an outdoor scene the random feel
 * matches the experience of a real shifting sky.
 *
 * T9  BLACKBODY — THE WARMTH OF THE LIGHT
 * ────────────────────────────────────────
 * The sun's COLOUR is a Planckian blackbody at the chosen kelvin
 * temperature (DAWN 2000K → DUSK 2500K → GOLDEN 3500K → NOON 5500K).
 * blackbody_rgb (§5) is the Tanner-Helland approximation — three
 * piecewise polynomials per channel, fitted to the CIE standard.
 *
 * Accurate to within a few percent over 1000K-12000K, and CHEAP
 * (a few transcendentals per call).
 *
 * The chromaticity is then scaled to HDR brightness:
 *
 *   sun_emit = sun_chrom · SUN_EMIT_HDR    (HDR factor, 14)
 *   horizon  = sun_chrom · SKY_HORIZON_FAC (display, 0.5)
 *
 * Same chromaticity feeds the sky-horizon gradient — so the horizon
 * naturally tints with the sun's temperature. The zenith stays a
 * constant cool blue (SKY_ZENITH_*) regardless of kelvin.
 *
 * T10 TONE MAPPING HDR → ASCII
 * ─────────────────────────────
 * The accumulator stores LINEAR HDR — the sun's contribution can
 * easily exceed 1.0 (SUN_EMIT_HDR = 14 to begin with). To display we
 * compress to [0, 1] and map to a discrete glyph + cube colour.
 *
 * Reinhard tone map:    L' = L / (1 + L)
 *   Smooth, monotonic, never clips. L=0 → 0, L=1 → 0.5, L→∞ → 1.
 *
 * Gamma encode:         out = L'^(1/2.2)
 *   sRGB displays apply ~2.2 gamma. Pre-compensate.
 *
 * Cube colour:          (r5, g5, b5) = round(out · 5)
 *   216 colour pairs from the xterm 6×6×6 cube.
 *
 * Glyph density:        luma = Rec.601 luma of (r, g, b)
 *                       glyph = k_ramp[round(luma · (RAMP_LEN-1))]
 *   16 glyphs from " " (dim) to "0" (densest).
 *
 * Bright cells get A_BOLD; dark cells A_DIM — keeps the dynamic
 * range visible.
 *
 * T11 FOUR DEBUG OVERLAYS — SEE EACH COMPONENT ON ITS OWN
 * ────────────────────────────────────────────────────────
 * Cycling `d' shows one of four lenses on the same path-traced
 * scene:
 *
 *   NORMAL   The full render. In-scatter + (surface · T_far). What
 *            the user actually sees by default.
 *
 *   SCATTER  In-scatter ONLY. The shafts float in pure black; walls
 *            and sky vanish. The pointed-arch silhouettes in light
 *            stand out cleanly — pedagogically this shows where the
 *            path tracer's STAGE 5 (T3) accumulator is depositing
 *            radiance.
 *
 *   SURFACE  Surface-only. No mist, no scatter — bare scene with
 *            wall + floor + sky-through-windows. Pedagogical: shows
 *            what the renderer would produce if MARCH_STEPS were 0
 *            (or if MIST_SIGMA were 0).
 *
 *   TR       Gray-scale transmittance at the surface. White =
 *            T(0, t_max) ≈ 1 (clear medium); black = T ≈ 0 (thick
 *            mist). Pedagogical: this is the eye-ray transmittance
 *            after the march. Sky pixels (4m chord through 0.20 σ)
 *            read as a moderate gray (~0.45); wall pixels read
 *            depending on their depth.
 *
 * Adding NORMAL = SCATTER + SURFACE · TR cell-by-cell would
 * exactly reproduce NORMAL. That equality is the rendering equation
 * reduced to two summands — a useful sanity check.
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
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ──────────────────────────────────────────────────────── */

/* §1.1 frame-rate / motion knobs. */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 view geometry — pinhole camera with pitch (T7). */
#define ASPECT_Y 2.0f
#define FOV_H_BASE 0.55f
#define ZOOM_MIN 0.25f
#define ZOOM_MAX 4.0f
#define ZOOM_STEP 1.25f

#define CAMERA_HEIGHT 1.6f /* eye level above floor (m)        */

/* §1.3 wall + room geometry. */
#define WALL_Z 4.0f
#define FLOOR_Y 0.0f

/* §1.4 windows — 2 rows × 5 cols of POINTED ARCHES (T6).
 *
 *   half_w   = 0.12 m  (also each arc's radius / 2 = 0.24 m)
 *   rect_h   = 0.32 m
 *   arch_h   = √3 · half_w ≈ 0.208 m  (peak above rect top)
 *   total_h  ≈ 0.528 m
 */
#define WIN_ROWS 2
#define WIN_COLS 5
#define WIN_COUNT (WIN_ROWS * WIN_COLS)

#define WIN_HALF_W 0.12f
#define WIN_RECT_H 0.32f
#define WIN_PERIOD_X 0.30f     /* col-to-col spacing            */
#define WIN_ROW_GAP_FRAC 0.30f /* row gap as fraction of total_h */

#define WIN_SILL0_OFFSET_Y 0.05f /* lower-row sill above eye      */
#define WIN_JITTER_POS 0.08f     /* ±fraction of WIN_PERIOD_X     */

/* sqrt(3) for the arch peak height. */
#define WIN_SQRT3 1.7320508f

/* §1.5 atmospheric medium — uniform mist (T2).
 *
 *   σ_e = MIST_SIGMA inside (z < WALL_Z, y > 0); 0 elsewhere.
 *   Tr_to_sun is closed-form (§7.2): exp(-σ · d_to_wall).
 *   MARCH_STEPS = 32 across at most ~5m gives step_length ≈ 0.16 m.
 *   HG_G = 0.55 — moderate forward bias.
 */
#define MIST_SIGMA 0.20f /* per metre — extinction coeff. */
#define HG_G 0.55f       /* HG anisotropy in (-1, 1)      */
#define MARCH_STEPS 32
#define FAR_CLIP (WALL_Z + 1.0f)
#define INSCATTER_GAIN 1.6f /* arbitrary brightness scale     */

/* §1.6 sun (DIRECTIONAL — circular orbit, T8).
 *
 * Single-period circular orbit so low elevation always coincides with
 * az going right→left, high elevation with az going left→right.
 *
 *   el = SUN_EL_BASE + SUN_EL_AMP · cos(ωt)
 *   az = SUN_AZ_BASE + SUN_AZ_AMP · sin(ωt)
 *   ω  = 2π / SUN_EL_PERIOD_S
 *
 * el range [0.07, 0.23] rad spans both row's apparent y-positions on
 * the wall (1.88 m ↔ 2.55 m), so the sun disc passes through both
 * rows during one orbit.
 */
#define SUN_ANG_RADIUS 0.10f /* rad — apparent disc          */
#define SUN_EMIT_HDR 14.0f   /* HDR brightness multiplier    */

#define SUN_EL_BASE 0.15f /* rad — ~8.6° median           */
#define SUN_EL_AMP 0.08f  /* rad — ±4.6°                  */
#define SUN_AZ_BASE 0.0f
#define SUN_AZ_AMP 0.20f       /* rad — ±11.5° azimuth         */
#define SUN_EL_PERIOD_S 200.0f /* full orbit period (seconds)  */

/* §1.7 surfaces. */
#define WALL_R 0.10f
#define WALL_G 0.09f
#define WALL_B 0.07f

#define FLOOR_R 0.20f
#define FLOOR_G 0.15f
#define FLOOR_B 0.09f

#define SKY_ZENITH_R 0.10f
#define SKY_ZENITH_G 0.13f
#define SKY_ZENITH_B 0.20f
#define SKY_HORIZON_FAC 0.50f

/* §1.8 buffer + ncurses pair-id reservations. */
#define BUF_MAX_W 400
#define BUF_MAX_H 200

enum {
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_SUN_FALLBACK = 3,
  PAIR_CUBE_BASE = 16, /* + 0..215 = 6×6×6 cube         */
};

/* §1.9 ASCII glyph ramp (16 levels). */
static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.10 kelvin presets (T9). */
typedef struct {
  const char *name;
  float kelvin;
} KelvinPreset;

static const KelvinPreset KELVINS[] = {
    {"DAWN  ", 2000.0f},
    {"GOLDEN", 3500.0f},
    {"NOON  ", 5500.0f},
    {"DUSK  ", 2500.0f},
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
  MODE_NORMAL = 0,
  MODE_SCATTER = 1,
  MODE_SURFACE = 2,
  MODE_TR = 3,
  MODE_N = 4,
} DebugMode;

static const char *debug_mode_name(DebugMode m) {
  switch (m) {
  case MODE_NORMAL:
    return "NORMAL ";
  case MODE_SCATTER:
    return "SCATTER";
  case MODE_SURFACE:
    return "SURFACE";
  case MODE_TR:
    return "TR     ";
  default:
    return "?      ";
  }
}

/* ── §2 clock ───────────────────────────────────────────────────────── */

/*
 * clock_ns — monotonic wall time in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backward across NTP / DST / suspend.
 */
static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns — best-effort sleep. Used to cap render rate
 * without burning CPU at 100%.
 */
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ── §3 vec3 math ────────────────────────────────────────────────────── *
 *
 * V3 — three floats by value. Inlined to avoid call overhead in
 * the per-pixel inner loop.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3_add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3_sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3_scl(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float v3_dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(V3 a) { return sqrtf(v3_dot(a, a)); }

/*
 * v3_norm — return a unit vector in the same direction as `a',
 * or (0,0,0) if `a' is zero-length (avoids NaN propagation).
 */
static inline V3 v3_norm(V3 a) {
  float length = v3_len(a);
  if (length < 1e-12f)
    return v3(0, 0, 0);
  return v3_scl(a, 1.0f / length);
}

/* ── §4 rgb math ─────────────────────────────────────────────────────── *
 *
 * RGB — three linear-space floats. NOT clamped to [0,1]; HDR until
 * the tone-map stage in §10.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  float r, g, b;
} RGB;

static inline RGB rgb_make(float r, float g, float b) { return (RGB){r, g, b}; }
static inline RGB rgb_add(RGB a, RGB b) {
  return rgb_make(a.r + b.r, a.g + b.g, a.b + b.b);
}
static inline RGB rgb_mul(RGB a, RGB b) {
  return rgb_make(a.r * b.r, a.g * b.g, a.b * b.b);
}
static inline RGB rgb_scl(RGB a, float s) {
  return rgb_make(a.r * s, a.g * s, a.b * s);
}
static inline RGB rgb_lerp(RGB a, RGB b, float t) {
  return rgb_make(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
                  a.b + (b.b - a.b) * t);
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float luma_of(RGB c) {
  return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

/* ── §5 blackbody (Tanner-Helland) ──────────────────────────────────── *
 *
 * Map a colour temperature in kelvin to an approximate sRGB
 * chromaticity. Three piecewise polynomials per channel, accurate
 * to within a few percent over 1000K-12000K.
 *
 * Used by §16 sky_radiance and the HUD; the chosen kelvin preset
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
static RGB blackbody_rgb(float kelvin) {
  float K = kelvin / 100.0f;
  float r, g, b;

  if (K <= 66.0f)
    r = 1.0f;
  else
    r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

  if (K <= 66.0f)
    g = (99.47f * logf(K) - 161.12f) / 255.0f;
  else
    g = 288.12f * powf(K - 60.0f, -0.0755f) / 255.0f;

  if (K >= 66.0f)
    b = 1.0f;
  else if (K <= 19.0f)
    b = 0.0f;
  else
    b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

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

static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* ── §7 atmospheric medium ───────────────────────────────────────────── *
 *
 * Uniform mist inside the room; vacuum elsewhere. Two helpers:
 *
 *   §7.1  sigma_e_at  — extinction at a world point
 *   §7.2  Tr_to_sun   — analytic transmittance from a point to the
 *                       wall plane along the sun-direction chord
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §7.1 ── sigma_e_at: extinction coefficient at a point ────────────── */

/*
 * sigma_e_at — extinction at world point p.
 *   Inside the room (z < WALL_Z, y > FLOOR_Y): MIST_SIGMA.
 *   Outside: 0 (vacuum).
 */
static inline float sigma_e_at(V3 scatter_point) {
  if (scatter_point.z >= WALL_Z)
    return 0.0f;
  if (scatter_point.y <= FLOOR_Y)
    return 0.0f;
  return MIST_SIGMA;
}

/* §7.2 ── Tr_to_sun: medium transmittance from p toward sun ────────── */

/*
 * tr_to_sun — analytic transmittance from p along sun_dir to where
 *             the chord exits the room (the wall plane).
 *
 * Pseudocode:
 *   if sun_dir.z < ε:                  chord parallel to wall → 0
 *   if p.z >= WALL_Z:                  already past mist     → 1
 *   d_to_wall = (WALL_Z - p.z) / sun_dir.z      distance to wall
 *   τ         = MIST_SIGMA · d_to_wall          uniform-σ optical depth
 *   return exp(-τ)                              Beer-Lambert
 *
 * Beyond the wall there's no medium, so the integral terminates
 * there. Inside the medium, σ is constant — closed form. NO second
 * march required (cf. §18 trace_ray's eye-ray march).
 */
static inline float tr_to_sun(V3 scatter_point, V3 sun_dir) {
  if (sun_dir.z < 1e-3f)
    return 0.0f;
  if (scatter_point.z >= WALL_Z)
    return 1.0f;
  float distance_to_wall = (WALL_Z - scatter_point.z) / sun_dir.z;
  if (distance_to_wall < 0.0f)
    return 1.0f;
  float optical_depth = MIST_SIGMA * distance_to_wall;
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
static inline float hg_phase(float cos_angle_eye_to_sun) {
  float g = HG_G;
  float g2 = g * g;
  float denom = 1.0f + g2 - 2.0f * g * cos_angle_eye_to_sun;
  if (denom < 1e-9f)
    denom = 1e-9f;
  return (1.0f - g2) / (4.0f * (float)M_PI * powf(denom, 1.5f));
}

/* ── §9 RNG (hash3 / hash01 — window x-jitter only) ─────────────────── *
 *
 * The path tracer is DETERMINISTIC (no Monte Carlo at runtime). The
 * only randomness is the per-seed window x-jitter (place_windows in
 * §12). Pure functions — no global state.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * hash3 — mix three integer keys into a 32-bit hash. Three large
 * odd primes followed by xorshift-mul rounds. Standard recipe.
 */
static inline uint32_t hash3(int kx, int ky, int kz) {
  uint32_t h = (uint32_t)kx * 73856093u ^ (uint32_t)ky * 19349663u ^
               (uint32_t)kz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

/*
 * hash01 — uniform float in [0, 1). Drops the top byte of a 32-bit
 * hash to avoid the rounding-to-1 issue on extreme values.
 */
static inline float hash01(uint32_t h) {
  return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* ── §10 ncurses paint ───────────────────────────────────────────────── *
 *
 * One cell = (cube colour pair, glyph from ramp, A_BOLD / A_DIM /
 * A_NORMAL). Tone-map happens HERE — single application per cell.
 *
 * ─────────────────────────────────────────────────────────────────── */

static int g_have_256;

static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);
  if (g_have_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
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
static void paint_cell(int sx, int sy, RGB col) {
  float r = gamma_enc(reinhard(col.r));
  float g = gamma_enc(reinhard(col.g));
  float b = gamma_enc(reinhard(col.b));
  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;
  int ri = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
  if (ri < 0)
    ri = 0;
  if (ri >= RAMP_LEN)
    ri = RAMP_LEN - 1;

  if (g_have_256) {
    int r5 = (int)(r * 5.f + 0.5f);
    if (r5 > 5)
      r5 = 5;
    if (r5 < 0)
      r5 = 0;
    int g5 = (int)(g * 5.f + 0.5f);
    if (g5 > 5)
      g5 = 5;
    if (g5 < 0)
      g5 = 0;
    int b5 = (int)(b * 5.f + 0.5f);
    if (b5 > 5)
      b5 = 5;
    if (b5 < 0)
      b5 = 0;
    int pair = PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
    int attr = (luma > 0.85f) ? A_BOLD : (luma < 0.15f) ? A_DIM : A_NORMAL;
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
 * Single primitive: ray vs horizontal plane y = const. Used by the
 * floor (FLOOR_Y = 0). The wall is a vertical plane handled
 * separately (§13) because of the slit-aware test.
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
 * ray is heading DOWN through the plane.
 */
static bool ray_plane_y(V3 ray_origin, V3 ray_dir, float plane_y,
                        float *out_t) {
  if (ray_dir.y > -1e-6f)
    return false;
  float t = (plane_y - ray_origin.y) / ray_dir.y;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* ── §12 arch geometry — pointed (lancet) arches (T6) ───────────────── *
 *
 * Three helpers:
 *
 *   §12.1  Window struct + globals
 *   §12.2  place_windows  — populate the 2×5 grid + jitter cx
 *   §12.3  xy_in_any_window — point-in-pointed-arch test (T6)
 *
 * ─────────────────────────────────────────────────────────────────── */

/* §12.1 ── Window struct + globals ─────────────────────────────────── */

/*
 * Window — one pointed arch on the wall plane.
 *   cx       centre x on the wall plane (m)
 *   cy_base  y of the sill (rect bottom) (m)
 *   half_w   half-width — also the arc radius / 2
 *   rect_h   rectangular body height
 *
 * Arch peak height above sill = rect_h + sqrt(3)·half_w  (see T6).
 */
typedef struct {
  float cx, cy_base, half_w, rect_h;
} Window;

static Window g_windows[WIN_COUNT];

/* §12.2 ── place_windows: lay out the 2×5 grid ─────────────────────── */

/*
 * place_windows — populate g_windows for the given seed.
 *
 * Pseudocode:
 *   total_h    = WIN_RECT_H + √3 · WIN_HALF_W
 *   row_period = total_h · (1 + WIN_ROW_GAP_FRAC)
 *   row0_sill  = CAMERA_HEIGHT + WIN_SILL0_OFFSET_Y
 *   x0         = -(WIN_COLS - 1) · WIN_PERIOD_X / 2     centre on x = 0
 *   for row in 0..1:
 *     for col in 0..4:
 *       jitter = (hash01(...) - 0.5) · 2 · WIN_JITTER_POS · WIN_PERIOD_X
 *       g_windows[row*5+col] = {
 *         cx      = x0 + col · WIN_PERIOD_X + jitter,
 *         cy_base = row0_sill + row · row_period,
 *         half_w  = WIN_HALF_W,
 *         rect_h  = WIN_RECT_H
 *       }
 *
 * Only x-position is jittered; the count is fixed at 2×5 = 10.
 */
static void place_windows(int seed) {
  float total_h = WIN_RECT_H + WIN_SQRT3 * WIN_HALF_W;
  float row_period = total_h * (1.0f + WIN_ROW_GAP_FRAC);
  float row0_sill = CAMERA_HEIGHT + WIN_SILL0_OFFSET_Y;
  float x0 = -(float)(WIN_COLS - 1) * WIN_PERIOD_X * 0.5f;

  for (int row = 0; row < WIN_ROWS; row++) {
    for (int col = 0; col < WIN_COLS; col++) {
      int idx = row * WIN_COLS + col;
      uint32_t pos_hash = hash3(idx, seed, 0xA12CE001);
      float jitter =
          (hash01(pos_hash) - 0.5f) * 2.0f * WIN_JITTER_POS * WIN_PERIOD_X;

      g_windows[idx].cx = x0 + (float)col * WIN_PERIOD_X + jitter;
      g_windows[idx].cy_base = row0_sill + (float)row * row_period;
      g_windows[idx].half_w = WIN_HALF_W;
      g_windows[idx].rect_h = WIN_RECT_H;
    }
  }
}

/* §12.3 ── xy_in_any_window: point-in-pointed-arch test (T6) ───────── */

/*
 * xy_in_any_window — true if (x, y) lies inside ANY pointed-arch
 *                    window on the wall.
 *
 * Per window:
 *   |dx| <= half_w               (x bounds)
 *   dy = y - cy_base
 *   if dy < 0:                   below sill → no
 *   if dy <= rect_h:             inside rectangle → YES
 *   ay = dy - rect_h             (height above rect top)
 *   if ay > √3 · half_w:         above peak → no
 *   inside left arc:  (dx - half_w)² + ay² ≤ (2·half_w)²
 *   inside right arc: (dx + half_w)² + ay² ≤ (2·half_w)²
 *   if BOTH arcs pass → YES (inside arch zone)
 *   else → no
 *
 * The two-circle intersection IS the lancet shape (T6). Used by
 * BOTH:
 *   §13 ray_wall (eye-ray hits wall)
 *   §15 scene_blocked_to_sun (NEE shadow-ray test)
 */
static bool xy_in_any_window(float x, float y) {
  float arc_radius_sq = 4.0f * WIN_HALF_W * WIN_HALF_W;
  float arch_max = WIN_SQRT3 * WIN_HALF_W;

  for (int i = 0; i < WIN_COUNT; i++) {
    const Window *w = &g_windows[i];
    float dx = x - w->cx;
    if (fabsf(dx) > w->half_w)
      continue;

    float dy = y - w->cy_base;
    if (dy < 0.0f)
      continue;
    if (dy <= w->rect_h)
      return true; /* rectangle body */

    float ay = dy - w->rect_h;
    if (ay > arch_max)
      continue; /* above peak     */

    /* Left arc: centre at (cx + half_w, cy_top); offset = (dx − half_w, ay). */
    float left_dx = dx - w->half_w;
    if (left_dx * left_dx + ay * ay > arc_radius_sq)
      continue;

    /* Right arc: centre at (cx − half_w, cy_top); offset = (dx + half_w, ay).
     */
    float right_dx = dx + w->half_w;
    if (right_dx * right_dx + ay * ay > arc_radius_sq)
      continue;

    return true; /* inside arch    */
  }
  return false;
}

/* ── §13 ray-wall intersection (slit-aware) ─────────────────────────── *
 *
 * The wall is a vertical plane at z = WALL_Z, opaque except where
 * the 10 pointed-arch windows are cut. Hits inside any window are
 * misses — the ray passes through to the sky beyond.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ray_wall — intersect ray with the slit-aware wall plane.
 *
 * Pseudocode:
 *   if ray_dir.z < ε:           ray going backward → MISS
 *   t = (WALL_Z − ray_origin.z) / ray_dir.z
 *   if t < ε:                   behind us → MISS
 *   x = ray_origin.x + t·ray_dir.x
 *   y = ray_origin.y + t·ray_dir.y
 *   if y < 0:                   below floor → MISS
 *   if xy_in_any_window(x, y):  passes through opening → MISS
 *   *out_t = t; HIT
 */
static bool ray_wall(V3 ray_origin, V3 ray_dir, float *out_t) {
  if (ray_dir.z < 1e-6f)
    return false;
  float t = (WALL_Z - ray_origin.z) / ray_dir.z;
  if (t < 1e-3f)
    return false;
  float hit_x = ray_origin.x + t * ray_dir.x;
  float hit_y = ray_origin.y + t * ray_dir.y;
  if (hit_y < 0.0f)
    return false;
  if (xy_in_any_window(hit_x, hit_y))
    return false;
  *out_t = t;
  return true;
}

/* ── §14 scene_hit (find nearest of {wall, floor, sky}) ─────────────── */

typedef enum { HIT_SKY = 0, HIT_WALL = 1, HIT_FLOOR = 2 } HitType;

typedef struct {
  HitType type;
  float t;
} Hit;

/*
 * scene_hit — return the nearest scene hit.
 *
 * Pseudocode:
 *   start with HIT_SKY, t = INF
 *   try ray_plane_y vs floor    → if closer, set HIT_FLOOR
 *   try ray_wall (slit-aware)   → if closer, set HIT_WALL
 *   return best
 */
static Hit scene_hit(V3 ray_origin, V3 ray_dir) {
  Hit hit = {HIT_SKY, 1e30f};

  float t_floor;
  if (ray_plane_y(ray_origin, ray_dir, FLOOR_Y, &t_floor)) {
    if (t_floor < hit.t) {
      hit.type = HIT_FLOOR;
      hit.t = t_floor;
    }
  }

  float t_wall;
  if (ray_wall(ray_origin, ray_dir, &t_wall)) {
    if (t_wall < hit.t) {
      hit.type = HIT_WALL;
      hit.t = t_wall;
    }
  }

  return hit;
}

/* ── §15 sun visibility (slit-aware NEE test, T5) ───────────────────── *
 *
 * Does the wall block the chord from a scatter point in the sun's
 * direction? Same slit-aware geometry as ray_wall (§13) — the wall
 * has cutouts in both directions.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * scene_blocked_to_sun — true if the wall blocks the chord from
 *                        scatter_point in direction sun_dir.
 *
 * Pseudocode:
 *   if sun_dir.z < ε:               sun parallel/backward → not blocked
 *   if scatter_point.z >= WALL_Z:   already past wall    → not blocked
 *   t = (WALL_Z − scatter_point.z) / sun_dir.z
 *   if t < ε:                       wall behind us       → not blocked
 *   x = scatter_point.x + t·sun_dir.x
 *   y = scatter_point.y + t·sun_dir.y
 *   if y < 0:                       below floor → BLOCKED
 *   if xy_in_any_window(x, y):      through window → not blocked
 *   else:                           hits solid wall → BLOCKED
 */
static bool scene_blocked_to_sun(V3 scatter_point, V3 sun_dir) {
  if (sun_dir.z < 1e-6f)
    return false;
  if (scatter_point.z >= WALL_Z)
    return false;
  float t = (WALL_Z - scatter_point.z) / sun_dir.z;
  if (t < 1e-3f)
    return false;
  float x = scatter_point.x + t * sun_dir.x;
  float y = scatter_point.y + t * sun_dir.y;
  if (y < 0.0f)
    return true;
  if (xy_in_any_window(x, y))
    return false;
  return true;
}

/* ── §16 sky / sun radiance ──────────────────────────────────────────── *
 *
 * The "background" radiance for any eye ray that ends up looking at
 * SKY (didn't hit wall or floor). Two components:
 *
 *   1. Vertical gradient   horizon-tinted (warm) at h ≈ 0,
 *                          zenith-tinted (cool blue) at h ≈ 1.
 *   2. Sun disc            additive bright spot when the ray points
 *                          inside SUN_ANG_RADIUS of sun_dir.
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
static RGB sky_radiance(V3 ray_dir, V3 sun_dir, RGB sun_em, RGB horizon_col) {
  float h = ray_dir.y;
  if (h < 0.f)
    h = 0.f;
  if (h > 1.f)
    h = 1.f;

  RGB zenith = rgb_make(SKY_ZENITH_R, SKY_ZENITH_G, SKY_ZENITH_B);
  RGB sky = rgb_lerp(horizon_col, zenith, h);

  float cos_ray_to_sun = v3_dot(ray_dir, sun_dir);
  float cos_disc_edge = cosf(SUN_ANG_RADIUS);
  if (cos_ray_to_sun > cos_disc_edge) {
    float t_edge = (cos_ray_to_sun - cos_disc_edge) / (1.0f - cos_disc_edge);
    if (t_edge > 1.f)
      t_edge = 1.f;
    sky = rgb_add(sky, rgb_scl(sun_em, t_edge));
  }
  return sky;
}

/* ── §17 surface shading ─────────────────────────────────────────────── *
 *
 * Two BRDFs:
 *   §17.1  shade_wall   constant dim wall colour
 *   §17.2  shade_floor  Lambertian, NEE-illuminated
 *
 * ─────────────────────────────────────────────────────────────────── */

static RGB shade_wall(void) { return rgb_make(WALL_R, WALL_G, WALL_B); }

/*
 * shade_floor — Lambertian floor with NEE direct sun + sky ambient.
 *
 * Pseudocode:
 *   albedo  = (FLOOR_R, FLOOR_G, FLOOR_B)
 *   cos_NL  = max(0, sun_dir.y)                   floor normal = +Y
 *   direct  = (cos_NL > 0 and not blocked-to-sun)
 *               ? sun_em · albedo · cos_NL / π
 *               : 0
 *   ambient = horizon_col · albedo · 0.20         faint sky term
 *   return direct + ambient
 */
static RGB shade_floor(V3 floor_point, V3 sun_dir, RGB sun_em,
                       RGB horizon_col) {
  RGB albedo = rgb_make(FLOOR_R, FLOOR_G, FLOOR_B);
  float cos_normal_to_sun = sun_dir.y;
  if (cos_normal_to_sun < 0.f)
    cos_normal_to_sun = 0.f;

  RGB direct = rgb_make(0.f, 0.f, 0.f);
  if (cos_normal_to_sun > 0.f) {
    bool blocked = scene_blocked_to_sun(floor_point, sun_dir);
    if (!blocked) {
      float lambert = cos_normal_to_sun / (float)M_PI;
      direct = rgb_scl(rgb_mul(sun_em, albedo), lambert);
    }
  }
  RGB ambient = rgb_scl(rgb_mul(horizon_col, albedo), 0.20f);
  return rgb_add(direct, ambient);
}

/* ── §18 trace_ray (THE CORE — volumetric path tracer) ──────────────── *
 *
 * One pixel's worth of light, computed by:
 *
 *   1. Find what the eye ray hits  → march_distance (Stage 2 of T3)
 *   2. Loop MARCH_STEPS times, accumulating in-scatter (Stages 3-6)
 *   3. Add surface contribution dimmed by remaining transmittance
 *      (Stage 7)
 *
 * Returns a Radiance struct with all components separated, so the
 * debug overlays in §21 can paint each on its own.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  RGB total;                 /* in_scatter + surface · T_far     */
  RGB in_scatter;            /* in-scatter accumulator only      */
  RGB surface;               /* surface radiance (un-dimmed)     */
  float t_far_transmittance; /* T(0, march_distance)             */
} Radiance;

/*
 * trace_ray — volumetric path tracer for one eye ray.
 *
 * Pseudocode (mirrors the inner loop body 1:1):
 *   hit = scene_hit(ray)                         Stage 2
 *   march_distance = (hit type)
 *                    ? hit.t
 *                    : ray-to-wall-plane t (if SKY)
 *   step_length = march_distance / MARCH_STEPS
 *   phase_value = hg_phase(cos(ray_dir, sun_dir)) Stage 4 (constant)
 *   transmittance_along_eye = 1
 *   in_scatter = (0, 0, 0)
 *   for i = 0 .. MARCH_STEPS-1:
 *     if transmittance_along_eye < 1e-3: break   early-out
 *     scatter_point = ray + ((i + 0.5) · step_length)
 *     extinction    = sigma_e_at(scatter_point)
 *     if extinction > 0:
 *       if NOT scene_blocked_to_sun(scatter_point, sun_dir): Stage 4
 *         tr_to_sun = closed-form transmittance (§7.2)        Stage 5
 *         step_factor = extinction · phase_value
 *                     · tr_to_sun · INSCATTER_GAIN · step_length
 *         in_scatter += sun_em · step_factor
 *                     · transmittance_along_eye
 *       transmittance_along_eye *= exp(-extinction · step_length) Stage 6
 *   surface_radiance = shade_(wall|floor|sky)(...)            Stage 7
 *   total = in_scatter + surface_radiance · transmittance_along_eye
 *   return Radiance{total, in_scatter, surface_radiance,
 *                   transmittance_along_eye}
 *
 * Iterative, not recursive — the for-loop is the integrator. The
 * NEE check inside the inner block is the entire god-ray trick.
 */
static Radiance trace_ray(V3 ray_origin, V3 ray_dir, V3 sun_dir, RGB sun_em,
                          RGB horizon_col) {
  /* Stage 2 — scene intersection. */
  Hit scene_hit_record = scene_hit(ray_origin, ray_dir);

  /* march_distance: surface hit or wall-plane crossing for SKY. */
  float march_distance;
  if (scene_hit_record.type == HIT_SKY) {
    if (ray_dir.z > 1e-6f) {
      march_distance = (WALL_Z - ray_origin.z) / ray_dir.z;
      if (march_distance > FAR_CLIP)
        march_distance = FAR_CLIP;
    } else {
      march_distance = FAR_CLIP;
    }
  } else {
    march_distance = scene_hit_record.t;
  }
  if (march_distance < 1e-3f)
    march_distance = 1e-3f;

  /* Stage 3 — march setup. */
  float step_length = march_distance / (float)MARCH_STEPS;
  float cos_eye_to_sun = v3_dot(ray_dir, sun_dir);
  float phase_value = hg_phase(cos_eye_to_sun);

  RGB in_scatter_radiance = rgb_make(0.f, 0.f, 0.f);
  float transmittance_along_eye = 1.0f;

  /* Stages 3-6 — march loop. */
  for (int i = 0; i < MARCH_STEPS; i++) {
    if (transmittance_along_eye <= 1e-3f)
      break; /* early out */

    float step_t = ((float)i + 0.5f) * step_length;
    V3 scatter_point = v3_add(ray_origin, v3_scl(ray_dir, step_t));
    float extinction = sigma_e_at(scatter_point);

    if (extinction > 1e-6f) {
      /* Stage 4 — NEE: is the sun visible from here? */
      bool blocked = scene_blocked_to_sun(scatter_point, sun_dir);

      if (!blocked) {
        /* Stage 5 — in-scatter contribution at this step. */
        float transmittance_to_sun = tr_to_sun(scatter_point, sun_dir);
        float step_contribution = extinction * phase_value *
                                  transmittance_to_sun * INSCATTER_GAIN *
                                  step_length;
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
  case HIT_WALL:
    surface_radiance = shade_wall();
    break;
  case HIT_FLOOR: {
    V3 floor_point = v3_add(ray_origin, v3_scl(ray_dir, scene_hit_record.t));
    surface_radiance = shade_floor(floor_point, sun_dir, sun_em, horizon_col);
    break;
  }
  case HIT_SKY:
  default:
    surface_radiance = sky_radiance(ray_dir, sun_dir, sun_em, horizon_col);
    break;
  }

  /* Final radiance = in-scatter + surface · remaining T. */
  RGB total = rgb_add(in_scatter_radiance,
                      rgb_scl(surface_radiance, transmittance_along_eye));

  Radiance result = {
      .total = total,
      .in_scatter = in_scatter_radiance,
      .surface = surface_radiance,
      .t_far_transmittance = transmittance_along_eye,
  };
  return result;
}

/* ── §19 scene state + sun motion (T8) ──────────────────────────────── */

typedef struct {
  bool paused;
  int speed;
  int kelvin_idx;
  float zoom;
  float time_secs;
  int seed;
  DebugMode debug_mode;
} Scene;

/*
 * scene_sun_dir — current sun direction for time t.
 *
 * Single-period CIRCULAR orbit in (az, el) — see T8. Pseudocode:
 *
 *   ω    = 2π / SUN_EL_PERIOD_S      shared period
 *   el   = SUN_EL_BASE + SUN_EL_AMP · cos(ω · t)
 *   az   = SUN_AZ_BASE + SUN_AZ_AMP · sin(ω · t)
 *   return norm(sin(az)·cos(el), sin(el), cos(az)·cos(el))
 *
 * The cos/sin pairing locks the choreography: high el coincides with
 * az going L→R, low el with R→L (T8).
 */
static V3 scene_sun_dir(const Scene *s) {
  float omega = 2.0f * (float)M_PI / SUN_EL_PERIOD_S;
  float el = SUN_EL_BASE + SUN_EL_AMP * cosf(omega * s->time_secs);
  float az = SUN_AZ_BASE + SUN_AZ_AMP * sinf(omega * s->time_secs);
  float ce = cosf(el);
  return v3_norm(v3(sinf(az) * ce, sinf(el), cosf(az) * ce));
}

/*
 * scene_reseed — pick a new seed and re-place windows.
 * Mixes the current time into the new seed so successive presses
 * of `r' don't repeat layouts.
 */
static void scene_reseed(Scene *s) {
  uint32_t h = hash3((int)(s->time_secs * 1000.0f), s->seed, 0xC0FFEE);
  s->seed = (int)(h ^ 0x5A5A5A5Au);
  place_windows(s->seed);
}

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->kelvin_idx = 1; /* GOLDEN by default */
  s->zoom = 1.0f;
  s->seed = 0xA12CE001;
  s->debug_mode = MODE_NORMAL;
  place_windows(s->seed);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  s->time_secs += dt * speed_mul;
}

/* ── §20 camera (pinhole + pitch — T7) ──────────────────────────────── */

typedef struct {
  V3 pos;
  float fov_h, fov_v;
  float cos_pitch, sin_pitch;
  int cols, rows;
} Camera;

/*
 * camera_make — build a camera that frames BOTH rows of arches
 *               centred in the image.
 *
 * Pseudocode:
 *   pos        = (0, CAMERA_HEIGHT, 0)
 *   fov_v      = fov_h · rows · ASPECT_Y / cols
 *   total_h    = WIN_RECT_H + √3 · WIN_HALF_W
 *   row_period = total_h · (1 + WIN_ROW_GAP_FRAC)
 *   row0_centre_above = WIN_SILL0_OFFSET_Y + total_h/2
 *   row1_centre_above = row0_centre_above + row_period
 *   mid_y_above       = (row0_centre_above + row1_centre_above)/2
 *   pitch      = atan(mid_y_above / WALL_Z)
 *   cos_pitch  = cos(pitch); sin_pitch = sin(pitch)
 */
static void camera_make(Camera *c, int cols, int rows, float fov_h) {
  c->cols = cols;
  c->rows = rows;
  c->pos = v3(0.0f, CAMERA_HEIGHT, 0.0f);
  c->fov_h = fov_h;
  c->fov_v = fov_h * (float)rows * ASPECT_Y / (float)cols;

  float total_h = WIN_RECT_H + WIN_SQRT3 * WIN_HALF_W;
  float row_period = total_h * (1.0f + WIN_ROW_GAP_FRAC);
  float row0_centre_above = WIN_SILL0_OFFSET_Y + 0.5f * total_h;
  float row1_centre_above = row0_centre_above + row_period;
  float mid_y_above = 0.5f * (row0_centre_above + row1_centre_above);
  float pitch = atanf(mid_y_above / WALL_Z);
  c->cos_pitch = cosf(pitch);
  c->sin_pitch = sinf(pitch);
}

/*
 * camera_ray — pixel (sx, sy) → world-space ray direction.
 *
 * Pseudocode:
 *   u =  ((2·sx + 1) − cols) / cols  ·  fov_h         normalised x
 *   v = -((2·sy + 1) − rows) / rows  ·  fov_v         normalised y (flipped)
 *   --- unrotated direction is (u, v, 1); pitch around X axis: ---
 *   ry =  v · cos_pitch + 1 · sin_pitch
 *   rz = -v · sin_pitch + 1 · cos_pitch
 *   return normalize(u, ry, rz)
 *
 * The flip on v converts from "screen y down" to "world y up".
 */
static V3 camera_ray(const Camera *c, int sx, int sy) {
  float u =
      ((2.0f * (float)sx + 1.0f) - (float)c->cols) / (float)c->cols * c->fov_h;
  float v =
      -((2.0f * (float)sy + 1.0f) - (float)c->rows) / (float)c->rows * c->fov_v;

  float ry = v * c->cos_pitch + c->sin_pitch;
  float rz = -v * c->sin_pitch + c->cos_pitch;
  return v3_norm(v3(u, ry, rz));
}

/* ── §21 screen + scene_draw (with debug overlays) ──────────────────── */

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

static void screen_resize(Screen *s) {
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
 */
static void scene_draw(const Screen *sc, const Scene *s) {
  int rows_eff = sc->rows - 2;
  int row_off = 1;
  if (rows_eff < 4) {
    rows_eff = sc->rows;
    row_off = 0;
  }
  if (rows_eff > BUF_MAX_H)
    rows_eff = BUF_MAX_H;
  int cols_eff = sc->cols;
  if (cols_eff > BUF_MAX_W)
    cols_eff = BUF_MAX_W;

  Camera cam;
  camera_make(&cam, cols_eff, rows_eff, FOV_H_BASE / s->zoom);

  V3 sun_dir = scene_sun_dir(s);
  float kelvin = KELVINS[s->kelvin_idx].kelvin;
  RGB sun_chrom = blackbody_rgb(kelvin);
  RGB sun_em = rgb_scl(sun_chrom, SUN_EMIT_HDR);
  RGB horizon = rgb_scl(sun_chrom, SKY_HORIZON_FAC);

  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      V3 ray_dir = camera_ray(&cam, c, r);
      Radiance rad = trace_ray(cam.pos, ray_dir, sun_dir, sun_em, horizon);

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

/* ── §22 HUD ────────────────────────────────────────────────────────── */

/*
 * hud_draw — three HUD elements:
 *   row 0 right    yellow status (fps, Hz, paused, kelvin, sun
 *                  el/az deg, zoom, speed, grid, debug mode)
 *   row 0 left     yellow title (subtitle banner)
 *   bottom row     cyan key-hint strip (BOLD, ASCII only)
 */
static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  const KelvinPreset *k = &KELVINS[s->kelvin_idx];
  V3 sd = scene_sun_dir(s);
  float el_deg = asinf(sd.y) * 180.0f / (float)M_PI;
  float az_deg = atan2f(sd.x, sd.z) * 180.0f / (float)M_PI;

  char buf[256];
  snprintf(buf, sizeof buf,
           " %5.1f fps %3d Hz  %s  %s %5.0fK  sun:el%4.1f deg az%+5.1f deg  "
           "z:%3.1fx  spd:%d  %dx%d  %s ",
           fps, sim_fps, s->paused ? "PAUSED" : "      ", k->name,
           (double)k->kelvin, (double)el_deg, (double)az_deg, (double)s->zoom,
           s->speed, WIN_COLS, WIN_ROWS, debug_mode_name(s->debug_mode));
  int len = (int)strlen(buf);
  if (len > sc->cols)
    len = sc->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " GOD-RAYS-WINDOW · ISLAMIC ARCHES ");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reseed  d:debug  t/T:kelvin  z/Z:zoom  "
           "+/-:spd  []:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(sc, s);
  hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §23 app (signals, main loop) ───────────────────────────────────── */

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
  case 'r':
  case 'R':
    scene_reseed(s);
    break;

  case 'd':
  case 'D':
    s->debug_mode = (DebugMode)((s->debug_mode + 1) % MODE_N);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    break;
  case '-':
  case '_':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
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

  case 't':
    s->kelvin_idx = (s->kelvin_idx + 1) % N_KELVINS;
    break;
  case 'T':
    s->kelvin_idx = (s->kelvin_idx + N_KELVINS - 1) % N_KELVINS;
    break;

  case 'z':
    s->zoom *= ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    s->zoom /= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
    break;
  }
  return true;
}

static void app_init(App *app) {
  memset(app, 0, sizeof *app);
  scene_init(&app->scene);
  screen_init(&app->screen);
  app->sim_fps = SIM_FPS_DEFAULT;
  app->running = 1;
}

static void app_run(App *app) {
  int64_t prev = clock_ns();
  int64_t sim_accum = 0;
  int64_t frame_count = 0;
  int64_t fps_window_start = prev;
  double fps_meas = 0.0;

  struct sigaction sa = {0};
  sa.sa_handler = on_exit_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sa.sa_handler = on_resize_signal;
  sigaction(SIGWINCH, &sa, NULL);
  atexit(cleanup);

  while (app->running) {
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }
    if (!app->running)
      break;
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - prev;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
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
      fps_meas = (double)frame_count * (double)NS_PER_SEC /
                 (double)(now - fps_window_start);
      frame_count = 0;
      fps_window_start = now;
    }

    int64_t target = clock_ns();
    int64_t left = TICK_NS(SIM_FPS_DEFAULT * 2) - (target - now);
    if (left > 0)
      clock_sleep_ns(left);
  }
}

int main(void) {
  app_init(&g_app);
  app_run(&g_app);
  cleanup();
  return 0;
}
