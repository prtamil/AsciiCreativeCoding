/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * solar_eclipse.c — a path-traced solar eclipse, rendered into ASCII at
 * a low resolution but with real volumetric physics. The corona, the
 * chromosphere's red Hα ring, the prominences arcing out from the
 * lunar limb, Bailey's beads, the diamond ring at first/last contact,
 * the soft penumbra at the moon's edge — every one of those features
 * is the natural output of a single per-pixel volume-rendering kernel,
 * not a screen-space effect painted on top.
 *
 * What you'll see when you run it
 * ───────────────────────────────
 *   A small bright sun sits at the centre of a 2-D pinhole-camera view.
 *   The moon orbits horizontally across it on a sin curve, completing
 *   one full pass every ~30 seconds at default speed. As the moon
 *   crosses, the corona — invisible while the photosphere is bright —
 *   gradually fades up, peaks dramatically at totality, then fades
 *   back. At the totality boundary, photospheric light leaks through
 *   valleys in the moon's silhouette as Bailey's beads, with one
 *   dominant bead briefly forming the diamond ring. During totality
 *   itself, a thin red chromospheric ring hugs the moon, and prominence
 *   loops rise from it in fBm-shaped arcs. The moon's silhouette is
 *   filled with a dim cool-blue glow — earthshine.
 *
 * What's controlling each visual
 * ──────────────────────────────
 *   The corona's structure: σ(r) = σ₀(R/r)^3 for the spherical part,
 *     plus an additive prominence field anisotropic in latitude.
 *   The corona's COLOUR: scattered photospheric blackbody light, so
 *     hotter stars (B-class) yield bluer coronas — that's real Thomson
 *     scattering physics, not artistic licence.
 *   Bailey's beads: the moon is a sphere with a per-direction radius
 *     perturbation (44 procedural valleys around its limb). When a
 *     valley angularly aligns with the photospheric edge during
 *     first/last contact, light sneaks through.
 *   The diamond ring: one of the 14 macro valleys per random seed is
 *     2.5× deeper than the rest. When it aligns at contact, it produces
 *     a single bright flash above the smaller surrounding beads.
 *   The penumbra (soft moon-shadow edge): each scatter point in the
 *     corona uses a closed-form circle-circle disc-overlap formula to
 *     get a continuous [0..1] visibility-of-the-sun, not a binary block.
 *   Eye adaptation: the corona is multiplied by a per-frame gate that
 *     drops to zero when the photosphere is unblocked (matching what a
 *     human eye actually perceives during a real eclipse).
 *
 * Stellar classes (cycle with t / T):
 *     M-DWARF   3500 K   cool red star
 *     K-STAR    4500 K   orange
 *     G-STAR    5778 K   our Sun (default)
 *     A-STAR    9500 K   white-blue (Sirius-like)
 *     B-STAR   18000 K   hot blue
 *
 * Patterns (cycle with n / p):
 *     TOTAL     moon angular radius > sun's     → totality possible
 *     PARTIAL   moon offset above midline       → never centres
 *     ANNULAR   moon angular radius < sun's     → ring of fire
 *     TRANSIT   moon ≪ sun                     → pinpoint silhouette
 *
 * Other keys:
 *   q / ESC      quit
 *   space        pause / resume orbit
 *   r            reseed (orbit phase + lunar terrain + spicule fBm)
 *   z / Z        zoom in / out                  (0.25× .. 8×)
 *   m / M        shrink / grow moon             (0.5× .. 2×)
 *   + / =        speed up the orbit             ('-': slow down)
 *   ] / [        raise / lower simulation tick rate (Hz)
 *
 * Section map (top-level):
 *   §0  forward types    — V3 and RGB declared early so §1 can use them
 *   §1  configuration    — every tunable named & documented in one place
 *   §2  monotonic clock  — frame-pacing primitives
 *   §3  math + colour    — V3 / RGB ops, blackbody, Reinhard, gamma
 *   §4  rng + noise      — hash3, Perlin / fBm
 *   §5  terminal paint   — 6×6×6 cube + density-glyph ramp + paint_cell
 *   §6  ray-sphere       — analytic primary intersection
 *   §7  lunar terrain    — Bailey's-bead silhouette via per-seed valleys
 *   §8  penumbra         — analytic visible-fraction-of-sun-disc
 *   §9  coronal medium   — σ(r), prominences, emission, density_total
 *   §10 transmittance    — 1-D radial LUT τ(r) → exp(−τ)
 *   §11 path tracer      — march + NEE + surface BRDFs (the kernel)
 *   §12 scene state      — Scene struct, orbit, occlusion, reseed
 *   §13 camera           — ray generation from screen cell coordinates
 *   §14 screen + draw    — scene_draw orchestration, framebuffer
 *   §15 hud              — top-row status, event flash, hint strip
 *   §16 app              — signals, fixed-step main loop
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/solar_eclipse.c \
 *       -o solar_eclipse -lncurses -lm
 *
 * Study alongside:
 *   raytracing/path_tracer.c        — same Reinhard/cube pipeline
 *   raytracing/saturn_with_rings.c  — same RGB tone-mapping
 *   grids/rect_grids/01_uniform_rect.c
 *                                   — canonical MENTAL MODEL block
 */

/* ── HOW TO READ THIS FILE ─────────────────────────────────────────────── *
 *
 * Read top-to-bottom on a first pass. The four prose blocks at the top
 * (this one, plus CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL) teach the
 * algorithm without code. Once you reach §0 you should already know
 * what each function will do before you read it.
 *
 * If you only have a few minutes:
 *   - skim CONCEPTS for the algorithm name + references
 *   - read MENTAL MODEL → CORE IDEA (one paragraph)
 *   - jump to §11 trace_ray to see the volume-rendering kernel
 *
 * If you want to TUNE the visuals, the GUIDED TUTORIAL is mapped to
 * sections of §1 — each tunable paragraph in §1 says which knob inside
 * it controls which visible feature.
 *
 * Background you'll need: basic C (pointers, structs, arithmetic on
 * arrays); a feel for ray-sphere intersection (the quadratic formula);
 * a vague familiarity with what an integral is. No prior path-tracing
 * knowledge is assumed; the volume rendering equation is derived in
 * Tutorial 4 from a glitter-dust analogy.
 *
 * Variable-naming conventions used in §11 and other dense functions:
 *   `transmittance`     attenuation factor along the eye ray, [0..1]
 *   `radiance_accum`    accumulated HDR colour for the pixel
 *   `density`           local σ at the current sample point
 *   `tr_to_sun`         transmittance from a scatter point to photosphere
 *   `t_along_ray`       parametric distance from camera along the ray
 *   `dt`                length of the current march step (world units)
 * Short names like `i`, `t`, `dphi` appear only inside very short loops.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Single-scattering volumetric path tracing with
 *                  next-event estimation (NEE). For each pixel we
 *                  march along a primary eye ray; at each step we
 *                  evaluate the local extinction coefficient σ(p) of
 *                  a coronal-plasma medium and ask "if a photon
 *                  scattered here, what fraction of the photosphere is
 *                  visible from this point and what radiance does it
 *                  contribute back along the eye ray?". We sum those
 *                  contributions, attenuating each by the running
 *                  transmittance. When the eye ray finally hits sun /
 *                  moon / sky we add a surface BRDF term. The output
 *                  is HDR; we tone-map and quantise it to 6×6×6 cube
 *                  cells with an ASCII density-glyph for brightness.
 *
 * Data-structure : Coronal medium is spherically symmetric: a 1-D
 *                  function σ(r) gives the scattering coefficient at
 *                  distance r from the sun, with three regions —
 *                  power-law corona, thin chromosphere shell, then
 *                  zero outside CORONA_REACH·R. Prominences are an
 *                  additive anisotropic field (per-seed angular
 *                  positions). Two precomputed lookup tables (LUT)
 *                  carry the realtime budget:
 *                    • trans_lut[256]   τ(r) integrated radially
 *                    • lunar_lut[512]   moon's perturbed silhouette
 *                                       depth as a function of phi
 *                  Per-frame state: 80×40-ish RGB framebuffer.
 *
 * Rendering      : HDR float RGB per cell → Reinhard tone-map per
 *                  channel → gamma 1/2.2 → quantise to xterm 6×6×6 cube
 *                  → density glyph from the 16-char open-form ramp
 *                  ` .'`,-_:;~=+*oO0`. A_BOLD on bright cells, A_DIM
 *                  on dark; A_NORMAL otherwise.
 *
 * Performance    : ~3000 cells × ~30 active march steps × O(1) per step
 *                  (one transmittance LUT fetch, one lunar LUT fetch,
 *                  one closed-form penumbra formula, ~10 FLOPs of
 *                  arithmetic). At 30 fps that's roughly 3 M FLOPs of
 *                  hot-path work per second — comfortably realtime on
 *                  a single CPU core. Three optimisations earn this
 *                  budget: closed-form transmittance via 1-D radial
 *                  LUT, cheap base-sphere reject for moon-block tests,
 *                  and adaptive march (4× coarse stride outside the
 *                  corona's effective extent).
 *
 * References     : Pharr / Jakob / Humphreys, "Physically Based
 *                    Rendering" 3rd ed §15 — volumetric path tracing,
 *                    next-event estimation, transmittance integrals.
 *                  Eddington, "The Internal Constitution of the Stars"
 *                    (1926), §8 — Eddington-approximation limb
 *                    darkening.
 *                  Wikipedia: Solar corona, K-corona, Thomson
 *                    scattering, Solar prominence, Hα line, Bailey's
 *                    beads, Eclipse penumbra.
 *                  Tanner Helland, "How to convert temperature to RGB":
 *                    https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Picture the corona as glitter suspended in space around the sun. A
 * photon leaves the photosphere, hits a glitter speck, scatters, and
 * some of those scattered photons reach your eye. The brightness you
 * see at any point along your line of sight is the sum of "how much
 * glitter is here" × "how much sun-light reaches it" × "how much of
 * that scatters in your direction". Compute exactly that integral per
 * pixel and you get a real coronal image — no fake halos required.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Walk a stick along a line through the glitter cloud. Stop every
 * inch. At each stop, ask the question: "if a scatter event happened
 * here, how bright would it be?" Three sub-questions:
 *   1. Is there glitter at this point?  (Density σ at p.)
 *   2. Can sunlight reach this point?   (Transmittance scatter→sun,
 *                                        and is the moon in the way?)
 *   3. Will it scatter toward my eye?   (Phase function of the
 *                                        scatter angle.)
 * Sum these answers, weighted by how transparent the cloud is between
 * the camera and the current stop. That sum, plus whatever you hit at
 * the end of the stick, is the pixel.
 *
 *      camera ────►  ●─●─●─●─●─●─●─●─●─●─●─●─●  ←  surface or sky
 *                    │ │ │ │ │ │ │ │ │ │ │ │ │
 *                    │ │ │ each stop:
 *                    │ │ │   density σ(p)
 *                    │ │ │   NEE ↑ to photosphere (ask sun)
 *                    │ │ │   accumulate scattered light
 *                    │ │ │   attenuate transmittance for next step
 *                    ▼ ▼ ▼
 *                    photosphere (NEE target)
 *
 * DRAWING METHOD / ALGORITHM IN STEPS  (per pixel, per frame)
 * ──────────────────────────────────────────────────────────
 *  1. Build the camera ray for the cell. Find the nearest hit
 *     among {sun-sphere, moon-sphere}; if neither, t_max = "sky".
 *  2. Initialise transmittance = 1, radiance_accum = (0,0,0).
 *  3. March a parametric t from 0 toward t_max. Step size depends
 *     on whether we're inside the corona's radial extent (fine step)
 *     or outside it (4× coarse step — σ is zero out there).
 *  4. At each step's midpoint p:
 *        density = σ_corona(|p−sun|) + σ_chromos(|p−sun|)
 *                                    + σ_prom(p, medium)
 *        if density is significant:
 *           visible = analytic_circle_circle(sun-disc, moon-disc, p)
 *           if visible > 0:
 *              tr_to_sun = trans_lut(|p−sun|)
 *              phase     = thomson_phase(angle between eye-ray, p→sun)
 *              omega_sun = solid_angle(sun-sphere from p)
 *              radiance_accum += transmittance · density · phase
 *                                · tr_to_sun · omega_sun · visible
 *                                · sun_emission · dt · gain · gate
 *           radiance_accum += transmittance · emission_at(p) · dt · gate
 *           transmittance *= exp(−density · dt)
 *  5. Add the surface contribution at t_max:
 *        sun-hit:  emission · limb_darkening(N·V) · transmittance
 *        moon-hit: (sun_NEE_cos(N,sun) + earthshine) · transmittance
 *        sky:      0
 *  6. Reinhard: c' = c / (1+c). Gamma 1/2.2. Cube quantise. Glyph.
 *
 * KEY FORMULAS
 * ────────────
 *  Volume rendering equation (single-scatter):
 *      L(eye) = ∫₀^t_max T(t) σ(t) [J_emit(t) + J_scatter(t)] dt
 *             + T(t_max) · L_surface
 *  Single-scatter NEE term:
 *      J_scatter(p) = phase(θ) · Tr(p→sun) · L_sun · Ω(sun seen from p)
 *                                          · vis(p, sun, moon)
 *  Beer-Lambert transmittance:
 *      T(t) = exp(−∫₀^t σ(s) ds)
 *  Thomson phase function (electron scatter, achromatic):
 *      P_T(θ) = (3/16π) · (1 + cos²θ)
 *  Solid angle of a sphere at distance d, radius R (R ≤ d):
 *      Ω = 2π · (1 − √(1 − R²/d²))
 *  Eddington-style limb darkening at view-cosine μ:
 *      I(μ) = AMBIENT + GAIN · μ        (with our defaults 0.4, 0.6)
 *  Analytic circle-circle disc overlap (small-angle approximation):
 *      see visible_fraction_sun in §8 — closed form, no sampling.
 *  Planck blackbody → sRGB chromaticity:
 *      Tanner Helland piecewise approximation; see §3.4.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The transmittance LUT is 1-D radial. The chord from a scatter
 *    point P toward the sun centre passes through the photosphere at
 *    exactly distance R_sun, ending there. Older 2-D versions of this
 *    LUT (parameterised by impact parameter b) have a trap: at b=0
 *    the chord seems to cross the sun's interior, so they store τ=∞.
 *    But our chord ends at the surface — so the right answer is the
 *    radial integral from R_sun outward to r. The 1-D LUT bypasses
 *    the b=0 confusion entirely.
 *  • Moon-block of NEE is computed analytically as visible-fraction,
 *    not as a binary intersection test. Without this you get a hard
 *    cliff at the moon's edge instead of a soft penumbra.
 *  • The eye-adaptation gate must use the camera's view of the sun,
 *    not the per-scatter-point view. Otherwise you'd get inconsistent
 *    fade-up across pixels. One number per frame.
 *  • Adaptive marching only safe because σ ≡ 0 outside CORONA_REACH·R.
 *    Coarse strides through truly empty space contribute nothing,
 *    regardless of step size. Inside the medium we keep fine steps.
 *  • Earthshine is geometrically simplified: real Earth is BEHIND the
 *    camera, and the moon's near face is lit by Earth-reflected
 *    sunlight. We approximate this as a constant ambient with a blue
 *    Earth-albedo tint on the moon's surface. cos(N, sun) ≤ 0 in this
 *    geometry so direct sun-lit moon would be black anyway.
 *  • B-star coronas come out blue. That's correct: Thomson scattering
 *    is wavelength-flat, so the corona's chromaticity follows the
 *    photosphere's spectrum. A 18000 K star has cool-blue blackbody
 *    light, and its corona scatters that same cool-blue.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space at totality. The corona should have visible streaks
 *    (denser at the equator if you happen to land there), a thin red
 *    Hα ring just outside the moon, and a few prominence loops at
 *    random angular positions.
 *  • Cycle stellar class with t/T. The same scene geometry repeats,
 *    but M-DWARF gives a deep red sun + reddish corona, G-STAR a
 *    warm-white sun, B-STAR a blue-white sun + bluer corona. If those
 *    chromaticities don't shift together, the colour pipeline is wrong.
 *  • Outside the eclipse window: the corona should be invisible (or
 *    very faint at FLOOR=0.15). If you can see corona around an
 *    uneclipsed sun, the eye-adaptation gate is mis-wired.
 *  • Press r repeatedly during totality. Bailey's-bead positions and
 *    chromosphere spicule pattern should change visibly each time.
 *  • Press n / p to cycle pattern. ANNULAR should show a bright ring
 *    of sun around the moon at maximum (no totality). TRANSIT should
 *    show a tiny moon dot crossing the disc.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ───────────────────────────────────────────────────── *
 *
 * Ten short lessons that build the algorithm from first principles.
 * Read them in order on the first pass — each lesson assumes the
 * previous one. Then jump to the §-section listed at the end of each.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 1 — What is a solar eclipse, mechanically?
 * ────────────────────────────────────────────────────────────────────────
 *   Q: What's actually happening when an eclipse occurs?
 *
 *   The sun, moon, and an observer line up. The moon (~1740 km radius)
 *   is much smaller than the sun (~700,000 km radius), but it's also
 *   much closer to the observer. The two factors balance so that the
 *   moon's APPARENT angular size — atan(R/distance) — is comparable to
 *   the sun's. When they happen to coincide angularly, the moon
 *   blocks the photosphere from the observer's view. That brief
 *   blockage reveals a normally-invisible structure: the corona, a
 *   tenuous plasma extending a few solar radii outward.
 *
 *   We're modelling this from the observer's perspective. The camera
 *   is at the origin. Sun lives at +Z (far). Moon lives between camera
 *   and sun (closer in +Z). World units are arbitrary; we picked
 *   sun_R = 3.5, sun_Z = 50, moon_R = 0.62, moon_Z = 5 to give a
 *   pleasing screen presence with the moon's apparent radius about
 *   1.76× the sun's at default zoom.
 *
 *   ASCII diagram of the world layout:
 *
 *         camera                moon                       sun
 *           ●  ─────────────────  ●  ─────────────────────  ●
 *           │                    R=0.62                   R=3.5
 *           │                    Z=5                      Z=50
 *           ↑
 *         origin
 *           +Z increases this way →
 *
 *   Pseudocode of the per-frame setup:
 *       sun_pos       = (0, 0, 50)
 *       moon_pos      = (orbit_x * sin(omega·time), 0, 5)
 *       moon_R_world  = 0.62 (× user moon-scale)
 *
 *   See: §1.3, §12.3 (orbit), §12.4 (scene_moon_pos).
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 2 — A pinhole camera in three lines
 * ────────────────────────────────────────────────────────────────────────
 *   Q: How do we turn a pixel coordinate into a 3-D ray?
 *
 *   Pinhole model. The camera sits at the origin and looks down +Z.
 *   The image plane sits at +1 unit, and a pixel at column c, row r on
 *   a cols × rows screen maps to a point in the image plane. The ray
 *   from origin through that point, normalised, is the eye ray for
 *   the pixel.
 *
 *       u =  ((2c + 1) − cols) / cols · fov_h
 *       v = -((2r + 1) − rows) / rows · fov_v
 *       ray_dir = normalise( (u, v, 1) )
 *
 *   `fov_h` is the tan of the half-horizontal field of view. We
 *   multiply by `fov_v = fov_h · rows·ASPECT_Y / cols` so circles look
 *   round on a terminal where each cell is ~2× taller than wide. The
 *   −sign on v puts row 0 at the top of the screen.
 *
 *   Pseudocode:
 *       for each row r:
 *           for each col c:
 *               ray = camera_ray(c, r, cols, rows, fov_h)
 *               pixel = trace(ray)
 *
 *   See: §13.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 3 — Hitting a sphere (the quadratic in disguise)
 * ────────────────────────────────────────────────────────────────────────
 *   Q: How do we tell whether a ray hits a sphere, and where?
 *
 *   A point on the ray at parameter t is ray_origin + t·ray_dir.
 *   "Hits the sphere of radius R centred at C" means |that point − C|
 *   = R. Square that and you get a quadratic in t:
 *
 *       (oc + t·rd) · (oc + t·rd) = R²        with oc = ray_origin − C
 *       (rd·rd) t² + 2(oc·rd) t + (oc·oc − R²) = 0
 *       │ assuming |rd| = 1, the t² coefficient is 1 │
 *       t² + 2 b t + c = 0,  b = oc·rd,  c = oc·oc − R²
 *       t = −b ± √(b² − c)
 *
 *   Three cases:
 *       discriminant b² − c < 0  → ray misses the sphere entirely
 *       both t < epsilon         → sphere is behind the camera
 *       else                     → nearest valid t is the hit
 *
 *   The `epsilon = 1e-3` keeps us off self-intersection at the surface.
 *
 *   See: §6.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 4 — Why surfaces aren't enough: volume rendering
 * ────────────────────────────────────────────────────────────────────────
 *   Q: If we hit the sun, why don't we just draw the sun and stop?
 *
 *   Because the corona is air. There's no surface to hit. It's a thin
 *   plasma occupying a 3-D volume around the sun. Every cubic metre
 *   of corona absorbs and scatters a tiny amount of light. To render
 *   it we have to integrate along the ray.
 *
 *   The volume rendering equation says: the radiance reaching the eye
 *   is the sum of radiance contributed at every point along the ray,
 *   weighted by the transmittance from that point back to the eye.
 *   In integral form:
 *
 *       L_eye = ∫₀^t_max T(t) [σ(t) · J(t)] dt + T(t_max) · L_surface
 *
 *   where σ(t) is the local scattering coefficient at parameter t,
 *   J(t) is the local "source" — light entering the ray at that point —
 *   and T(t) = exp(−∫₀^t σ(s) ds) is the cumulative transmittance from
 *   eye to that point (Beer-Lambert law).
 *
 *   We approximate this integral with a Riemann sum: divide [0, t_max]
 *   into ~30 active steps, evaluate σ and J at the midpoint of each,
 *   accumulate. ASCII picture of one step:
 *
 *       eye ──── t_now ──╳────── t_next ──── ... ──── t_max
 *                        │
 *                  midpoint p:
 *                  σ = σ(p),  J = J(p)
 *                  L_pixel += T_now · σ · J · dt
 *                  T_next = T_now · exp(−σ · dt)
 *
 *   The "source" J has two parts: locally emitted light (chromosphere
 *   Hα emission) and in-scattered light (corona scattering photospheric
 *   light into our direction). Tutorial 5 covers the in-scatter.
 *
 *   See: §11.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 5 — Next-event estimation: sampling the sun
 * ────────────────────────────────────────────────────────────────────────
 *   Q: At each scatter point, how do we compute the in-scattered light?
 *
 *   Naively: shoot many rays from the scatter point in random
 *   directions, see which ones hit a light, average. Slow, noisy.
 *
 *   Smart: the sun is by far the brightest thing. Always sample
 *   exactly toward the sun and weight by the sun's solid angle. This
 *   is "next-event estimation" (NEE). For a sphere of radius R at
 *   distance d, the solid angle from a point outside it is:
 *
 *       Ω = 2π · (1 − cos α),   with sin α = R / d
 *
 *   The NEE estimate of in-scattered light at scatter point p:
 *
 *       J_scatter(p) = phase(θ) · transmittance(p→sun) · L_sun
 *                                                     · Ω(sun seen from p)
 *
 *   The transmittance(p→sun) accounts for absorption along the chord
 *   from p to the photosphere — coronal scattering between the scatter
 *   point and the sun dims the contribution. We compute this via a
 *   1-D LUT (Tutorial 6).
 *
 *   The phase(θ) is the Thomson phase function: probability that a
 *   photon scatters by angle θ. For free electrons (Thomson scattering)
 *   it's:
 *
 *       phase(θ) = (3 / 16π) · (1 + cos²θ)
 *
 *   This is wavelength-flat — Thomson cross-section doesn't depend on
 *   colour — which is why the corona's hue follows the photosphere's
 *   exactly.
 *
 *   See: §11 (`thomson_phase`, NEE inside `trace_ray`).
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 6 — The transmittance LUT (the realtime trick)
 * ────────────────────────────────────────────────────────────────────────
 *   Q: How do we compute transmittance(p → sun) without ray-marching
 *      a SECOND time per scatter point?
 *
 *   The chord from a scatter point P at distance r from the sun
 *   centre, going inward toward the photosphere, ends at radius R_sun.
 *   Because the medium is spherically symmetric (σ depends only on
 *   distance r), the optical depth along that chord depends only on
 *   r — not on direction. Precompute it once at startup.
 *
 *       τ(r) = ∫_{R_sun}^{r} σ(s) ds        (radial integral)
 *       Tr(r) = exp(−τ(r))
 *
 *   Store τ at 256 sample radii from R_sun to LUT_R_MAX. Lookup is
 *   one bilinear interpolation. ASCII picture of the chord:
 *
 *                     ┌──────────────── r = LUT_R_MAX
 *                     │ outer corona, σ ≈ 0
 *                     │
 *                     │  P (scatter point)
 *                     ├──── │ ←── chord direction (toward sun centre)
 *                     │     │
 *                     │     │  thin chromosphere shell, σ large
 *                     │   R_sun ←── chord ENDS here, at the photosphere
 *                     │
 *                     │  (chord does NOT continue into the sun's interior)
 *
 *   Because the chord ends at R_sun, we never need to know the impact
 *   parameter b. A single 1-D table τ(r) is enough.
 *
 *   See: §10.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 7 — Penumbra: continuous moon-shadow at the edge
 * ────────────────────────────────────────────────────────────────────────
 *   Q: How do we model the soft brightness gradient at the moon's edge?
 *
 *   From any scatter point in the corona, the sun and moon both look
 *   like circular discs on the sky. The fraction of the SUN's disc
 *   that is NOT covered by the MOON's disc is the visible fraction:
 *
 *       sun-disc:    centred at angle (sun − P), radius α_sun
 *       moon-disc:   centred at angle (moon − P), radius α_moon
 *       γ = angle between the two centres (on the unit sphere)
 *
 *   Three cases:
 *       γ ≥ α_sun + α_moon              → no overlap: visible = 1
 *       γ ≤ |α_sun − α_moon|            → full nesting:
 *           if α_moon ≥ α_sun           →   visible = 0  (total)
 *           if α_moon < α_sun           →   visible = 1 − (α_moon/α_sun)²
 *                                                            (annular)
 *       else                            → partial: closed-form area
 *                                          of two-circle intersection.
 *
 *   ASCII picture of the partial-overlap case:
 *
 *           ╭────────╮
 *          ╱  sun     ╲          ╲
 *         │           ╳overlap   │
 *          ╲          ╱           ╱
 *           ╰─────────╳───────────
 *                     │  moon  │
 *                      ╲──────╱
 *
 *           visible = (sun_area − overlap_area) / sun_area
 *
 *   No multi-sampling, no Monte Carlo, no extra rays — just one
 *   formula per scatter point. The shadow gradient at the moon's edge
 *   becomes a smooth function of position, giving the classic soft
 *   penumbra.
 *
 *   Bailey's beads are NOT done here — they come from a separate
 *   geometric perturbation of the moon's silhouette (Tutorial 8).
 *
 *   See: §8.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 8 — Bailey's beads + diamond ring via lunar terrain
 * ────────────────────────────────────────────────────────────────────────
 *   Q: What about the bright bead-like flashes at the totality boundary?
 *
 *   Real beads come from light leaking through valleys in the moon's
 *   mountain range. We model that with a per-direction perturbation of
 *   the moon's effective radius:
 *
 *       moon_R(phi) = moon_R_base · (1 − valley_depth(phi))
 *
 *   `phi` is the angular position around the moon's silhouette. We
 *   place 14 wide "macro" Gaussian valleys around the limb, each up to
 *   2% deep, with a per-seed random asymmetry (one side sharper than
 *   the other). On top of those we add 30 narrow "micro" valleys, up
 *   to 1% deep, to sharpen individual beads. One macro valley per
 *   seed is given a 2.5× depth boost — the diamond ring.
 *
 *   ASCII picture of moon's silhouette with valleys (exaggerated):
 *
 *                ╭───────╮
 *               ╱  moon   ╲
 *              │         ▼ │  ← valley (deeper inward)
 *              │           │
 *              │ ▼         │  ← another valley
 *               ╲   ◀────  ╱  ← diamond ring valley (deepest)
 *                ╰───────╯
 *
 *   When the sun's photospheric edge during first/last contact
 *   angularly aligns with a valley, photospheric light leaks through
 *   that valley directly to the camera — bead. The diamond-ring
 *   valley produces a clearly brighter flash than the others.
 *
 *   We bake the 14+30 valleys into a 512-entry LUT once per seed.
 *   At runtime, `lunar_R_at(direction)` is one bilinear lookup —
 *   no exp() in the hot path, no per-pixel terrain math.
 *
 *   IMPORTANT: this perturbation lives in the EYE-RAY hit test, not
 *   in the NEE penumbra formula. Beads are a primary-ray phenomenon
 *   (they show up in the eye-ray hit), penumbra is a NEE-secondary-
 *   ray phenomenon. Different physics, different code paths.
 *
 *   See: §7, §11 (`ray_moon` is called in eye-ray hit).
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 9 — Spicules + Hα chromosphere texture
 * ────────────────────────────────────────────────────────────────────────
 *   Q: Why is the red ring at the moon's edge textured, not smooth?
 *
 *   The chromosphere is a thin plasma layer just above the
 *   photosphere. It emits strongly in the Hα line (656.3 nm, hydrogen
 *   alpha) — a deep red. But it isn't a smooth shell: it's a forest of
 *   "spicules", jet-like structures rising radially. We model that
 *   radial-streak texture with 2-D fractal Brownian motion sampled in
 *   spherical surface coordinates (theta, phi):
 *
 *       chromos_emission(p) = base_emission(r)
 *                           · spicule_factor(theta, phi)
 *
 *       spicule_factor = SPICULE_BASE + SPICULE_AMP · fbm(phi · 12,
 *                                                          theta · 6)
 *
 *   where (theta, phi) are the polar and azimuthal angles of p in the
 *   sun's frame. The fBm "patches" extend radially through the thin
 *   shell, producing visible streaks. Bright peaks shift the Hα colour
 *   slightly toward yellow for thermal variation:
 *
 *       Hα_cool = (1.00, 0.18, 0.12)   — deep crimson
 *       Hα_hot  = (1.00, 0.45, 0.18)   — yellow-shifted at fBm peaks
 *
 *   Outer edge of shell falls off as (1−h)⁴ for sharpness, where
 *   h = (r − R_sun) / shell_thickness ∈ [0, 1].
 *
 *   See: §9.4.
 *
 * ────────────────────────────────────────────────────────────────────────
 * TUTORIAL 10 — Eye adaptation: why pre/post eclipses are dark
 * ────────────────────────────────────────────────────────────────────────
 *   Q: If the corona is "always there" physically, why is it invisible
 *      except during totality?
 *
 *   Real-life corona radiance is about 10⁶× dimmer than the
 *   photosphere. Your eye's pupil + retina adapt to the brightest
 *   thing in view; with the photosphere visible, that's "the photo-
 *   sphere", and everything 10⁶ dimmer is below your perceptual
 *   threshold. During totality the photosphere is hidden; your eye
 *   dilates and corona radiance is now well within your dynamic range.
 *
 *   We model this with a single per-frame multiplier:
 *
 *       cam_vis_sun = visible_fraction(camera, sun, moon)  ∈ [0, 1]
 *       gate        = FLOOR + RANGE · (1 − cam_vis_sun)
 *
 *   With FLOOR=0 (this file): gate = 0 when sun fully visible (corona
 *   completely invisible), gate = 1 at totality (corona at full
 *   brightness). Linear ramp through partial phases. Bump FLOOR to
 *   ~0.15 if you prefer a faint always-on baseline halo.
 *
 *   The gate multiplies corona NEE in-scatter and chromosphere/
 *   prominence emission. It does NOT multiply photosphere or moon
 *   surface contributions — those are direct light sources, the eye
 *   sees them at any time.
 *
 *   See: §1.9 (FLOOR / RANGE constants), §11 (gate parameter passed
 *   into `trace_ray`), §14.2 (gate computed once per frame).
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ── §0 forward types ────────────────────────────────────────────────── *
 *
 * V3 (a 3-D float vector) and RGB (three colour channels) are declared
 * here — at the very top — because §1.16 (the stellar-classes table)
 * embeds RGB literals when set up with explicit colour fields, and
 * because keeping these definitions early lets the rest of the file
 * use them anywhere. The math operators that consume V3 / RGB live in
 * §3; only the type tags appear here.
 *
 * No methods, no constructors — these are POD bags of three floats.
 * Throughout the file V3 represents a position OR a direction (the
 * difference is which one the variable name describes), and RGB
 * represents an HDR colour (channels can exceed 1.0 — tone-mapping
 * happens in §5).
 */

typedef struct {
  float x, y, z;
} V3;
typedef struct {
  float r, g, b;
} RGB;

/* ── §1 configuration ────────────────────────────────────────────────── *
 *
 * Every tunable in the simulation is a named #define here, with a
 * comment explaining what it controls and what changes when you nudge
 * it. The §1 sub-sections below mirror the GUIDED TUTORIAL topics —
 * if you've read Tutorial 6 (transmittance LUT), §1.8's LUT_SIZE is
 * the parameter you'd touch.
 *
 * No magic numbers anywhere else in this file. If you find one in §11
 * or further down, it's a bug — please rename and move it here.
 */

/* §1.1 frame-rate and orbit-speed knobs.
 *
 * The main loop runs a fixed-timestep simulation tick at SIM_FPS_DEFAULT
 * Hz, decoupled from the render rate. Pressing ] / [ adjusts the tick
 * rate at runtime; +/- multiplies / divides the orbital speed of the
 * moon (so you can dial in slow motion at a totality boundary).
 *
 * SPEED_DEF is the baseline — 1× orbit; SPEED_MAX is 8× faster than
 * that, SPEED_MIN is 1/8×. The scale is multiplicative.
 */
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
#define DT_CAP_NS (100 * NS_PER_MS) /* spiral-of-death guard */

/* §1.2 view geometry and zoom.
 *
 * FOV_H_BASE is tan(half horizontal field of view). The effective FOV
 * during rendering is `FOV_H_BASE / zoom` — larger zoom → smaller FOV
 * → bigger objects on screen. ASPECT_Y compensates for terminal cells
 * being roughly twice as tall as wide, so spheres render round.
 *
 * At zoom = 1, with the geometry below (sun_R = 3.5, sun_Z = 50), the
 * sun's apparent angular radius is ~7% of the screen half-width — a
 * compact, vivid disc. Zooming to 4× or 8× brings totality details
 * (chromosphere ring, prominences, beads) into focus.
 */
#define ASPECT_Y 2.0f
#define FOV_H_BASE 0.40f
#define ZOOM_MIN 0.25f
#define ZOOM_MAX 8.0f
#define ZOOM_STEP 1.25f

/* §1.3 scene geometry — sun far, moon close.
 *
 * Apparent angular radii at default geometry:
 *   sun_α   = atan(SUN_R  / SUN_Z ) = atan(3.5 / 50.0) ≈ 4.0°
 *   moon_α  = atan(0.62  / 5.0)    ≈ 7.1°
 *   ratio   = moon_α / sun_α       ≈ 1.76
 *
 * Setting moon clearly larger than sun (1.76×) makes totality read
 * unambiguously: the moon's silhouette dominates, with corona visible
 * in a halo around it. Were the ratio close to 1 the eye would read
 * the bright sun + corona as one disc and the moon as a small dark
 * intrusion — visually wrong.
 *
 * Three pattern-specific moon sizes are predefined; pattern_set
 * (§12.2) picks one when n / p is pressed. The user-controlled
 * `moon_scale` (in Scene, §12.1) multiplies on top of these.
 *
 * MOON_ORBIT_X_FRAC controls how far the moon swings horizontally —
 * the full orbit reaches MOON_ORBIT_X_FRAC × (sun_α + moon_α) on
 * either side of centre. Setting it < 1 gives a near-miss; 1.5 gives
 * a clean enter/exit cycle.
 */
#define SUN_Z 50.0f
#define SUN_R 3.5f
#define MOON_Z 5.0f
#define MOON_BASE_R_TOTAL 0.62f
#define MOON_BASE_R_ANNULAR 0.32f
#define MOON_BASE_R_TRANSIT 0.04f

#define MOON_SCALE_MIN 0.5f
#define MOON_SCALE_MAX 2.0f
#define MOON_SCALE_STEP 1.10f

#define ECLIPSE_PERIOD_S 30.0f
#define MOON_ORBIT_X_FRAC 1.5f
#define PARTIAL_Y_OFFSET 0.025f

/* §1.4 corona scattering coefficient.
 *
 * The corona's electron density falls roughly as r^(-2.5) (the K-corona
 * power law) — we use 3.0 here for a slightly steeper falloff that
 * keeps the visible glow tight to the limb. CORONA_SIGMA0 is σ at
 * exactly r = R_sun (the photospheric boundary). CORONA_REACH bounds
 * the medium: outside r > REACH·R_sun, σ ≡ 0 and the path tracer can
 * coarse-stride harmlessly.
 *
 * Real-world coronal optical depth across one solar radius is ~10^-9.
 * For an LDR ASCII display we scale all σ values up by a uniform
 * representational factor so τ along visible chords lands in the
 * [0.05, 0.6] range — this is a display scaling, not a physics hack;
 * the radial decay shape stays correct.
 */
#define CORONA_REACH 8.0f
#define CORONA_SIGMA0 0.18f
#define CORONA_DECAY 3.0f

/* §1.5 chromosphere shell.
 *
 * The chromosphere is a thin layer above the photosphere — real-world
 * thickness ~0.3% of R_sun, we use 2.5% so it's visible at low
 * resolution. CHROMOS_SIGMA0 is its peak σ (much higher than corona's
 * — chromosphere is optically thick at Hα core).
 *
 * Outer edge falls off as (1 − h)⁴ where h = (r − R_sun) / thickness ∈
 * [0,1] — sharper than a quadratic, reads as a thin band rather than
 * a soft glow.
 */
#define CHROMOS_THICK 0.025f
#define CHROMOS_SIGMA0 6.0f

/* §1.6 chromosphere spicule fBm.
 *
 * Spicules are radial jet-like structures in the chromosphere (~tens
 * of thousands of them on the real sun). We approximate the texture
 * by sampling 2-D fBm in spherical surface coordinates (theta, phi).
 * The patches extend radially through the thin shell, producing
 * visible streak-like texture rather than a smooth ring.
 *
 *   spicule_factor = SPICULE_BASE + SPICULE_AMP · fBm(phi · 12,
 *                                                     theta · 6)
 *
 * The factor multiplies chromos_emission. Hot peaks (high fBm) shift
 * the Hα colour toward yellow for thermal variation.
 */
#define SPICULE_FREQ_PHI 12.0f
#define SPICULE_FREQ_TH 6.0f
#define SPICULE_BASE 0.5f
#define SPICULE_AMP 1.2f

/* §1.7 prominences — dense Hα loops.
 *
 * Prominences are arcs of cool dense plasma rooted in the chromosphere.
 * We place PROM_COUNT of them at random angular positions per seed,
 * each with its own height (up to PROM_HEIGHT × R_sun above the
 * photosphere) and lateral angular width. Their density adds onto the
 * chromosphere shell — they are NOT spherically symmetric, so they
 * don't contribute to the radial transmittance LUT (only to the
 * additive emission term).
 */
#define PROM_COUNT 5
#define PROM_HEIGHT 0.18f
#define PROM_LATERAL 0.18f
#define PROM_SIGMA0 8.0f

/* §1.8 path tracer parameters.
 *
 * MARCH_STEPS is the budget of fine-step samples between t=0 and
 * t_max (per pixel). Adaptive marching uses these inside the corona's
 * radial extent and 4× coarser strides outside; the 96 fine-step
 * count gives Δt ≈ 0.83 in world units when t_max ≈ 80, which is
 * fine enough to step into the chromosphere shell (thickness 0.125)
 * with at least one sample inside.
 *
 * IN_SCATTER_GAIN scales corona radiance to the LDR display range
 * (real corona is ~10⁶× dimmer than photosphere; we compress that to
 * a factor of ~14 so Reinhard maps it to visibly distinct cells).
 *
 * SUN_EMIT_HDR is the photosphere radiance multiplier: high enough
 * that even after eye-ray extinction (T ≈ 0.6) the photosphere
 * saturates the tone-map at the disc centre.
 *
 * HA_EMIT_GAIN does the same job for chromosphere and prominence Hα
 * line emission — relative to corona scatter.
 *
 * LUT_SIZE / LUT_R_MAX define the radial transmittance table. 256
 * samples from R_sun to (REACH+2)·R_sun is plenty given the smooth
 * power-law σ — bilinear interpolation has sub-cell accuracy.
 */
#define MARCH_STEPS 96
#define MARCH_T_MAX (SUN_Z + SUN_R * 6.0f)
#define LUT_SIZE 256
#define LUT_R_MAX (SUN_R * (CORONA_REACH + 2.0f))
#define IN_SCATTER_GAIN 14.0f
#define SUN_EMIT_HDR 8.0f
#define HA_EMIT_GAIN 3.0f

/* §1.9 eye-adaptation gate.
 *
 * Multiplier applied to corona and chromosphere emission. Computed
 * once per frame from the camera's view of the sun:
 *
 *   gate = CORONA_GATE_FLOOR + CORONA_GATE_RANGE · (1 − vis_cam_sun)
 *
 * FLOOR=0 means no halo on an unblocked sun (this file). FLOOR=0.15
 * preserves a faint baseline glow always — change one constant to
 * switch behaviour. See Tutorial 10.
 */
#define CORONA_GATE_FLOOR 0.00f
#define CORONA_GATE_RANGE 1.00f

/* §1.10 surface BRDF parameters.
 *
 * LIMB_AMBIENT and LIMB_GAIN parameterise the photosphere's Eddington-
 * style limb darkening: I(μ) = AMBIENT + GAIN · μ where μ = N · V is
 * the cosine of the view angle. The disc edge (μ ≈ 0) reads at 40% of
 * the central brightness — close to real solar physics.
 *
 * Moon is Lambertian with a low albedo (lunar Bond albedo ≈ 0.12).
 * EARTH_ALBEDO_BLUE is the blue channel of the Earth's averaged
 * Rayleigh-tinted reflectance — the cool-blue tint the moon picks up
 * via earthshine. EARTHSHINE_GAIN scales the constant ambient term we
 * apply.
 */
#define LIMB_AMBIENT 0.40f
#define LIMB_GAIN 0.60f
#define MOON_ALBEDO 0.12f
#define EARTH_ALBEDO_BLUE 0.45f
#define EARTHSHINE_GAIN 0.10f

/* §1.11 lunar terrain — Bailey's-bead silhouette perturbation.
 *
 * Two scales of valleys:
 *   • 14 macro valleys, ~2% deep, 0.06 rad wide, ASYMMETRIC.
 *     One per seed gets a 2.5× depth boost (the diamond ring).
 *   • 30 micro valleys, ~1% deep, 0.025 rad wide, SYMMETRIC.
 *
 * Both bake into a single 512-entry angular LUT once per seed. Bigger
 * LUT size resolves narrow micro valleys without bilinear blur.
 *
 * The asymmetry: each macro valley has a per-seed coin-flip choosing
 * which side is sharp (narrower Gaussian) and which is gradual. Beads
 * "pop in" sharply on the sharp side and fade out on the gradual side,
 * mimicking real mountain shadows.
 */
#define LUNAR_VALLEY_COUNT 14
#define LUNAR_VALLEY_DEPTH 0.020f
#define LUNAR_VALLEY_WIDTH 0.06f
#define LUNAR_VALLEY_ASYM 0.45f
#define LUNAR_DIAMOND_BOOST 2.5f

#define LUNAR_MICRO_COUNT 30
#define LUNAR_MICRO_DEPTH 0.010f
#define LUNAR_MICRO_WIDTH 0.025f

#define LUNAR_LUT_SIZE 512

/* §1.12 background stars.
 *
 * Sparse pinpoint stars visible only during totality (drowned by
 * photosphere otherwise). Each star samples a blackbody at random
 * Kelvin in [STAR_K_MIN, STAR_K_MAX], log-uniform — so the field has
 * the warm-orange to cool-blue colour spread of real stars rather
 * than uniform white.
 *
 * STAR_DENSITY = 250 means roughly 1 star per 250 sky cells; tune
 * down for a denser field, up for sparser.
 */
#define STAR_DENSITY 250
#define STAR_OCC_VISIBLE 0.85f
#define STAR_K_MIN 3000.0f
#define STAR_K_MAX 30000.0f

/* §1.13 framebuffer dimensions.
 *
 * Static — no malloc anywhere in the hot path. BUF_MAX_W/H bound the
 * per-frame g_buf array; if your terminal is bigger than that you'll
 * see clipping (rare; 400×200 is generous).
 */
#define BUF_MAX_W 400
#define BUF_MAX_H 200

/* §1.14 ncurses pair IDs and ASCII glyph ramp.
 *
 * PAIR_HUD / PAIR_HINT match the project HUD convention. The cube
 * pairs occupy IDs 16..231 (216 cells of the xterm 6×6×6 cube).
 *
 * The density ramp ` .'`,-_:;~=+*oO0` uses only OPEN glyphs — no
 * filled blocks. Bright cells become points of light, not solid
 * pixels. `#` and `@` are deliberately absent.
 */
enum {
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_SUN_FALLBACK = 3,
  PAIR_EVENT_HOT = 4,
  PAIR_CUBE_BASE = 16,
};

static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.15 patterns enum.
 *
 * Four moon-size / orbit-offset configurations. pattern_set (§12.2)
 * applies the corresponding moon radius and y-offset.
 */
typedef enum {
  PATTERN_TOTAL = 0,
  PATTERN_PARTIAL = 1,
  PATTERN_ANNULAR = 2,
  PATTERN_TRANSIT = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_TOTAL:
    return "TOTAL  ";
  case PATTERN_PARTIAL:
    return "PARTIAL";
  case PATTERN_ANNULAR:
    return "ANNULAR";
  case PATTERN_TRANSIT:
    return "TRANSIT";
  default:
    return "?      ";
  }
}

/* §1.16 stellar classes — main-sequence spectral types.
 *
 * Each entry is a single number (Kelvin temperature). Every visible
 * colour in the scene derives from this:
 *   photosphere = blackbody(K)
 *   corona      = scattered photosphere = same chromaticity
 *   chromos/Hα  = fixed wavelength (independent of K)
 *
 * No artistic palettes here. M-DWARF gives a deep red sun, B-STAR
 * a hot blue one — these are real spectral types.
 */
typedef struct {
  const char *name;
  float kelvin;
} Star;

static const Star STARS[] = {
    {"M-DWARF", 3500.0f}, {"K-STAR ", 4500.0f},  {"G-STAR ", 5778.0f},
    {"A-STAR ", 9500.0f}, {"B-STAR ", 18000.0f},
};
#define N_STARS ((int)(sizeof STARS / sizeof STARS[0]))

/* ── §2 monotonic clock ──────────────────────────────────────────────── *
 *
 * Two primitives: read the monotonic clock as nanoseconds, sleep for
 * a duration in nanoseconds. Both use POSIX functions; nanosleep
 * blocks the calling thread until the time elapses. We use the
 * monotonic clock (not wall clock) so suspend/resume of the laptop
 * doesn't produce frame jumps.
 */

/*
 * clock_ns — nanoseconds since an arbitrary monotonic epoch.
 *
 * Purpose:    return a steadily-increasing time value that doesn't
 *             jump when wall-clock changes happen (NTP, DST).
 * Returns:    int64_t, ns since epoch (the value's absolute meaning
 *             doesn't matter; only differences are meaningful).
 * Why:        the simulation's accumulator-based loop in §16 needs
 *             a robust dt source. CLOCK_MONOTONIC is that.
 */
static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns — sleep for `ns` nanoseconds, no-op for non-positive.
 *
 * Purpose:    yield CPU until ns have elapsed. Returns early on signal
 *             interruption (we don't care; the main loop re-paces).
 * Inputs:     ns  duration to sleep (nanoseconds). Negative or zero
 *                 returns immediately.
 * Why:        used by the main loop to cap the maximum FPS — without
 *             it we'd burn 100% of a CPU core.
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

/* ── §3 math + colour ────────────────────────────────────────────────── *
 *
 * Vector arithmetic on V3, channel-wise arithmetic on RGB, plus tone-
 * mapping helpers and a blackbody-temperature → RGB approximation.
 * Everything is `static inline` because all of these are called in
 * tight loops — the path tracer dispatches thousands of v3_dot, v3_sub
 * calls per frame.
 *
 * We use sqrtf (single-precision) throughout. The path tracer doesn't
 * benefit from double-precision accuracy in this resolution regime.
 */

/* §3.1 V3 ops — geometric primitives.
 *
 * Constructor, addition, subtraction, scalar multiplication, dot
 * product, length, normalisation. v3_norm clamps zero-length vectors
 * to (0,0,0) instead of dividing by zero — used in a few places where
 * a chord might collapse during the orbit.
 */

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
static inline V3 v3_norm(V3 a) {
  float l = v3_len(a);
  if (l < 1e-12f)
    return v3(0, 0, 0);
  return v3_scl(a, 1.0f / l);
}

/* §3.2 RGB ops — channel-wise arithmetic.
 *
 * RGB is HDR (channels can exceed 1.0). rgb_mul is component-wise
 * multiplication, used for things like "sun colour × Earth albedo →
 * earthshine tint".
 */

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

/* §3.3 tone-map + gamma + luma.
 *
 * Reinhard tone-map: c' = c / (1+c). Maps [0, ∞) → [0, 1) — saturates
 * toward 1 as input grows. Gamma 1/2.2 is the standard sRGB encoding
 * approximation. luma_of uses Rec.601 weights to convert RGB to a
 * single brightness.
 *
 * These are the FINAL pipeline stage between HDR-radiance and the
 * 6×6×6 cube + glyph. paint_cell (§5.2) is the only call site.
 */

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }
static inline float luma_of(RGB c) {
  return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

/* §3.4 blackbody → RGB (Tanner Helland approximation).
 *
 * Purpose:     convert a temperature in Kelvin to an sRGB chromaticity.
 *              Used for the photosphere (sun_em), background star
 *              colours, and indirectly the corona (it scatters
 *              photospheric light unchanged in chromaticity).
 * Inputs:      kelvin  temperature, K. Useful range 1000–40000.
 * Returns:     RGB ∈ [0, 1] — chromaticity, NOT photometric brightness.
 *
 * Why:         every colour in the scene comes from this function. The
 *              piecewise approximation is from Tanner Helland (2012),
 *              accurate enough for visual chromaticity given the 6×6×6
 *              cube's quantisation budget. Higher precision (Planck
 *              integrated against CIE matching functions) wouldn't
 *              survive cube quantisation.
 *
 * Pseudocode:
 *      K = kelvin / 100   (scaled, for the piecewise)
 *      r = 1                                    if K ≤ 66
 *          329.7 · (K-60)^(-0.1332)             otherwise   (cap at 1)
 *      g = piecewise log/power formulae
 *      b = 0   if K ≤ 19
 *          1   if K ≥ 66
 *          piecewise log formula otherwise
 *      return clamp01(r), clamp01(g), clamp01(b)
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

/* ── §4 rng + noise ──────────────────────────────────────────────────── *
 *
 * Two RNG primitives: a 3-D integer hash for deterministic per-cell /
 * per-seed values (used for star positions, lunar valley parameters,
 * prominence layouts), and Perlin noise / fBm for the chromosphere
 * spicule texture (§9.4).
 *
 * The Perlin permutation table `perm[]` is shuffled per seed so the
 * spicule pattern changes when the user presses 'r'.
 */

/* §4.1 integer hash → uint32 / float.
 *
 * hash3 mixes three 32-bit integers into one. Used pervasively for
 * "give me a deterministic pseudo-random number for this combination
 * of indices and seed". Same inputs always produce the same output;
 * different inputs produce well-distributed outputs.
 */

static inline uint32_t hash3(int wx, int wy, int wz) {
  uint32_t h = (uint32_t)wx * 73856093u ^ (uint32_t)wy * 19349663u ^
               (uint32_t)wz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

static inline float hash01(uint32_t h) {
  return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* §4.2 Perlin noise.
 *
 * Standard 2-D Perlin noise. The permutation table `perm[]` is filled
 * by perm_shuffle from a seed; perlin2d does gradient-based smooth
 * interpolation via fade_q (the quintic 6t⁵-15t⁴+10t³ smoothstep).
 *
 * Output range: roughly (-1, 1); we re-scale to (0, 1) inside fbm2.
 */

static uint8_t perm[512];

static void perm_shuffle(int seed) {
  uint8_t base[256];
  for (int i = 0; i < 256; i++)
    base[i] = (uint8_t)i;
  uint32_t st = (uint32_t)seed * 2654435761u;
  for (int i = 255; i > 0; i--) {
    st = st * 1664525u + 1013904223u;
    int j = (int)(st >> 16) % (i + 1);
    uint8_t t = base[i];
    base[i] = base[j];
    base[j] = t;
  }
  for (int i = 0; i < 256; i++) {
    perm[i] = base[i];
    perm[i + 256] = base[i];
  }
}

static inline float fade_q(float t) {
  return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}
static inline float lerp_f(float a, float b, float t) {
  return a + t * (b - a);
}
static inline float grad2(int hash, float x, float y) {
  int h = hash & 7;
  float u = (h < 4) ? x : y;
  float v = (h < 4) ? y : x;
  return ((h & 1) ? -u : u) + ((h & 2) ? -2.f * v : 2.f * v);
}

static float perlin2d(float x, float y) {
  int X = (int)floorf(x) & 255;
  int Y = (int)floorf(y) & 255;
  x -= floorf(x);
  y -= floorf(y);
  float u = fade_q(x), v = fade_q(y);
  int A = perm[X] + Y;
  int B = perm[X + 1] + Y;
  float n00 = grad2(perm[A], x, y);
  float n10 = grad2(perm[B], x - 1.f, y);
  float n01 = grad2(perm[A + 1], x, y - 1.f);
  float n11 = grad2(perm[B + 1], x - 1.f, y - 1.f);
  return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* §4.3 fractal Brownian motion (fBm).
 *
 * Three octaves of Perlin summed with halving amplitude and doubling
 * frequency — the standard cloud-noise recipe. Returns a value in
 * [0, 1]. Used by emission_at (§9.4) for chromosphere spicule
 * texture.
 */

static float fbm2(float x, float y) {
  float total = 0.f, amp = 1.f, freq = 1.f, max_amp = 0.f;
  for (int o = 0; o < 3; o++) {
    total += amp * perlin2d(x * freq, y * freq);
    max_amp += amp;
    amp *= 0.5f;
    freq *= 2.0f;
  }
  return (total / max_amp) * 0.5f + 0.5f;
}

/* ── §5 terminal painting ────────────────────────────────────────────── *
 *
 * Final stage of the pipeline: take an HDR RGB radiance value and
 * write a glyph-with-colour to the terminal cell. Two functions:
 * color_init sets up the ncurses pair table at startup; paint_cell
 * does the per-cell tone-map → cube-quantise → glyph-pick at the end
 * of every frame.
 *
 * On 256-colour terminals we use a 6×6×6 RGB cube (216 pairs) plus
 * four named pairs for the HUD. On 8-colour terminals we fall back to
 * a single yellow-on-default pair for the whole scene — the eclipse
 * still works visually, just mono-yellow.
 */

static int g_256; /* 1 if terminal supports 256 colours */

/*
 * color_init — set up ncurses pair table.
 *
 * Purpose:     define COLOR_PAIR ids 1..231 with sensible foreground
 *              colours. Called once at startup from screen_init.
 * Inputs:      none (uses the global ncurses state set by start_color).
 * Why:         paint_cell needs the cube pairs pre-defined; the HUD
 *              uses PAIR_HUD / PAIR_HINT / PAIR_EVENT_HOT. Doing this
 *              once keeps the per-cell paint cheap.
 */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
    init_pair(PAIR_EVENT_HOT, 196, -1);
  } else {
    init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_EVENT_HOT, COLOR_RED, -1);
  }
}

/*
 * paint_cell — write one terminal cell from an HDR RGB value.
 *
 * Purpose:     end-of-pipeline conversion of HDR float radiance to a
 *              coloured glyph in the terminal at (sx, sy).
 * Inputs:      sx, sy   screen position (cell coordinates)
 *              col      HDR RGB radiance (channels can exceed 1.0)
 *
 * Pseudocode:
 *      r,g,b = gamma_enc(reinhard(col.{r,g,b}))
 *      luma  = 0.2126·r + 0.7152·g + 0.0722·b
 *      glyph = k_ramp[ round(luma · (RAMP_LEN − 1)) ]
 *      r5,g5,b5 = round(r/g/b · 5)        ∈ [0,5]
 *      pair  = PAIR_CUBE_BASE + r5·36 + g5·6 + b5
 *      attr  = A_BOLD if luma > 0.85 else A_DIM if luma < 0.15 else 0
 *      mvaddch(sy, sx, glyph) with COLOR_PAIR(pair) | attr
 *
 * Mental model: we're projecting an HDR point in colour space onto
 * one of 216 cube cells, then choosing a brightness glyph and a font
 * weight based on luma. The cube pair governs hue; the glyph governs
 * brightness; A_BOLD/A_DIM bumps brightness slightly at the extremes.
 *
 * Why: the only place HDR meets the terminal. If you want to add a
 * bloom pass or post-process, this is where it would land — but
 * currently we go straight from g_buf to mvaddch.
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

  if (g_256) {
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

/* ── §6 ray-sphere intersection ──────────────────────────────────────── *
 *
 * Analytic intersection of a unit-length ray with a sphere. The math
 * is the quadratic in disguise (Tutorial 3); this is the workhorse of
 * the path tracer's primary-ray hit test against the photosphere
 * sphere. The moon uses a perturbed variant in §7 that calls into the
 * lunar terrain LUT.
 *
 * If the ray misses or both hit-times are behind the camera, returns
 * false and *out_t is unchanged. Otherwise *out_t is the nearest hit
 * t-value (t > 1e-3 to keep us off self-intersection at the surface).
 */

/*
 * ray_sphere — analytic ray-vs-sphere with unit direction.
 *
 * Purpose:     test whether a ray from `ro` in direction `rd` (|rd|=1)
 *              hits the sphere of radius `r` centred at `c`, and if
 *              so, return the nearest valid hit-time.
 *
 * Pseudocode:
 *      oc = ro − c
 *      b  = oc · rd
 *      cc = oc·oc − r·r
 *      disc = b·b − cc
 *      if disc < 0: miss
 *      sq = √disc
 *      t  = −b − sq                ← near intersection
 *      if t < ε: t = −b + sq       ← try far intersection
 *      if t < ε: still behind     → miss
 *      *out_t = t; return true
 *
 * Inputs:      ro      ray origin (world units)
 *              rd      ray direction, must be unit length
 *              c       sphere centre (world)
 *              r       sphere radius (world units)
 *              out_t   filled with hit parameter on hit
 * Returns:     true on hit, false on miss or all-behind
 *
 * Why: the photosphere is the only purely-spherical primitive we hit
 * in the eye-ray pass. The moon needs a perturbed variant — see §7.
 */
static bool ray_sphere(V3 ro, V3 rd, V3 c, float r, float *out_t) {
  V3 oc = v3_sub(ro, c);
  float b = v3_dot(oc, rd);
  float cc = v3_dot(oc, oc) - r * r;
  float disc = b * b - cc;
  if (disc < 0)
    return false;
  float sq = sqrtf(disc);
  float t = -b - sq;
  if (t < 1e-3f)
    t = -b + sq;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* ── §7 lunar terrain — Bailey's-bead silhouette ─────────────────────── *
 *
 * The moon isn't a perfect sphere visually — it has mountains and
 * valleys around its limb. Real Bailey's beads come from light
 * leaking through valleys at the totality boundary. We model this
 * with a per-direction radius perturbation (Tutorial 8):
 *
 *      moon_R(direction) = moon_R_base · (1 − valley_depth(phi))
 *
 * `phi` is the angle from +x axis to the perpendicular projection of
 * `direction` onto the moon's silhouette plane (the xy plane in world
 * coordinates, since we look down +z). 14+30 valleys are baked into a
 * 512-entry LUT once per seed.
 *
 * §7 has three pieces:
 *   §7.1 build_lunar_lut — construct the LUT from per-seed valley params
 *   §7.2 lunar_R_at      — runtime O(1) lookup with bilinear interp
 *   §7.3 ray_moon        — perturbed ray-sphere using the LUT
 */

static float lunar_lut[LUNAR_LUT_SIZE];

/* §7.1 build_lunar_lut.
 *
 * Purpose:     bake 14 macro + 30 micro valleys into a single 512-entry
 *              angular table. After this, lunar_R_at is a constant-time
 *              lookup with no exp() calls.
 *
 * Pseudocode:
 *      pick a per-seed diamond-ring index ∈ [0, 14)
 *      for each macro k ∈ [0, 14):
 *          phi_k       = random angle around limb
 *          base_width  = LUNAR_VALLEY_WIDTH · (0.5..1.5)
 *          asym_sign   = ±1 (per seed)
 *          w_left      = base_width · (1 + asym · 0.45)
 *          w_right     = base_width · (1 − asym · 0.45)
 *          depth       = LUNAR_VALLEY_DEPTH · (0.4..1.0)
 *          if k == diamond_idx:
 *              depth *= LUNAR_DIAMOND_BOOST
 *      for each micro k ∈ [0, 30):
 *          ... narrower symmetric Gaussians ...
 *      for each LUT cell i:
 *          phi   = ((i / LUT_SIZE) · 2π) − π
 *          total = sum over macros (asym Gaussian) + sum over micros
 *          lunar_lut[i] = total
 *
 * Inputs:      seed   integer; same seed produces same valley layout.
 * Why:         baking once at reseed makes the runtime cost zero, and
 *              the bilinear interp at lookup gives sub-cell precision
 *              when the 0.06-rad valleys are ~10 LUT cells wide.
 */
static void build_lunar_lut(int seed) {
  /* Macro valleys (wide, asymmetric, the main silhouette features). */
  float phi_v[LUNAR_VALLEY_COUNT];
  float w2_left_v[LUNAR_VALLEY_COUNT];
  float w2_right_v[LUNAR_VALLEY_COUNT];
  float d_v[LUNAR_VALLEY_COUNT];

  /* Pick which macro valley is the diamond-ring (extra depth). */
  uint32_t dr_h = hash3(0, seed, 0xD1A47000u);
  int diamond_idx = (int)(hash01(dr_h) * (float)LUNAR_VALLEY_COUNT);
  if (diamond_idx >= LUNAR_VALLEY_COUNT)
    diamond_idx = LUNAR_VALLEY_COUNT - 1;

  for (int k = 0; k < LUNAR_VALLEY_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0x1A11E1);
    phi_v[k] = (hash01(h) - 0.5f) * 2.0f * (float)M_PI;
    float w_base = LUNAR_VALLEY_WIDTH * (0.5f + 1.0f * hash01(h ^ 0xE1u));

    /* Asymmetry: one side narrower (sharp), one wider (gradual).
     * `asym_sign` ∈ {-1, +1} chooses which side is sharp per seed. */
    float asym_sign = (hash01(h ^ 0xAA00u) > 0.5f) ? 1.0f : -1.0f;
    float w_left = w_base * (1.0f + asym_sign * LUNAR_VALLEY_ASYM);
    float w_right = w_base * (1.0f - asym_sign * LUNAR_VALLEY_ASYM);
    w2_left_v[k] = w_left * w_left;
    w2_right_v[k] = w_right * w_right;

    float d_k = LUNAR_VALLEY_DEPTH * (0.4f + 0.6f * hash01(h ^ 0xD0u));
    if (k == diamond_idx)
      d_k *= LUNAR_DIAMOND_BOOST;
    d_v[k] = d_k;
  }

  /* Micro valleys (narrow, symmetric — sharper beads). */
  float phi_s[LUNAR_MICRO_COUNT];
  float w2_s[LUNAR_MICRO_COUNT];
  float d_s[LUNAR_MICRO_COUNT];
  for (int k = 0; k < LUNAR_MICRO_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0xCAFEBABEu);
    phi_s[k] = (hash01(h) - 0.5f) * 2.0f * (float)M_PI;
    float w_k = LUNAR_MICRO_WIDTH * (0.6f + 0.8f * hash01(h ^ 0xE2u));
    w2_s[k] = w_k * w_k;
    d_s[k] = LUNAR_MICRO_DEPTH * (0.5f + 0.7f * hash01(h ^ 0xD1u));
  }

  for (int i = 0; i < LUNAR_LUT_SIZE; i++) {
    float phi = ((float)i / (float)LUNAR_LUT_SIZE) * 2.0f * (float)M_PI -
                (float)M_PI; /* phi ∈ [-π, π) */

    float total = 0.0f;
    /* Macro contribution — asymmetric Gaussian. */
    for (int k = 0; k < LUNAR_VALLEY_COUNT; k++) {
      float dphi = phi - phi_v[k];
      while (dphi > (float)M_PI)
        dphi -= 2.0f * (float)M_PI;
      while (dphi < -(float)M_PI)
        dphi += 2.0f * (float)M_PI;
      float w2 = (dphi < 0.0f) ? w2_left_v[k] : w2_right_v[k];
      total += d_v[k] * expf(-(dphi * dphi) / w2);
    }
    /* Micro contribution — symmetric. */
    for (int k = 0; k < LUNAR_MICRO_COUNT; k++) {
      float dphi = phi - phi_s[k];
      while (dphi > (float)M_PI)
        dphi -= 2.0f * (float)M_PI;
      while (dphi < -(float)M_PI)
        dphi += 2.0f * (float)M_PI;
      total += d_s[k] * expf(-(dphi * dphi) / w2_s[k]);
    }
    lunar_lut[i] = total;
  }
}

/* §7.2 lunar_R_at.
 *
 * Purpose:     given a unit vector pointing from moon centre toward a
 *              chord's closest-approach point, return the moon's
 *              effective radius along that direction.
 *
 * Pseudocode:
 *      phi   = atan2(from_centre.y, from_centre.x)
 *      u     = (phi + π) / (2π) · LUT_SIZE         ← LUT index, real
 *      i0    = floor(u);  i1 = (i0 + 1) mod LUT_SIZE
 *      depth = lunar_lut[i0]·(1−frac) + lunar_lut[i1]·frac
 *      return moon_R_base · (1 − depth)
 *
 * Inputs:      from_centre  unit V3; the silhouette direction
 *              moon_R_base  unperturbed radius (world units)
 *              seed         unused at runtime (baked into the LUT)
 *
 * Why: the cost-aware wrapper around the LUT. Called from ray_moon
 * (§7.3) at the closest-approach point of a chord. Bilinear lookup
 * ensures smooth transitions between LUT cells.
 */
static float lunar_R_at(V3 from_centre, float moon_R_base, int seed) {
  (void)seed; /* baked into the LUT at build time */
  float phi = atan2f(from_centre.y, from_centre.x); /* (-π, π] */
  float u = (phi + (float)M_PI) / (2.0f * (float)M_PI) * (float)LUNAR_LUT_SIZE;
  if (u < 0.0f)
    u += (float)LUNAR_LUT_SIZE;
  if (u >= LUNAR_LUT_SIZE)
    u -= (float)LUNAR_LUT_SIZE;
  int i0 = (int)u;
  int i1 = (i0 + 1) % LUNAR_LUT_SIZE;
  float fr = u - (float)i0;
  float depth = lunar_lut[i0] * (1.0f - fr) + lunar_lut[i1] * fr;
  return moon_R_base * (1.0f - depth);
}

/* §7.3 ray_moon — perturbed ray-vs-moon intersection.
 *
 * Purpose:     test a ray against the moon, using the lunar terrain LUT
 *              to vary the moon's effective radius with direction.
 *
 * The fast path:
 *      Compute b, oc² as in ray_sphere.
 *      Test against the UNPERTURBED moon_R_base sphere first.
 *      If miss → return false (cheap reject — no LUT lookup).
 *      Since lunar valleys only REDUCE radius (depth ≥ 0), missing the
 *      base sphere implies missing any perturbed variant.
 *
 * If the cheap reject doesn't fire, do the perturbed test:
 *      Compute closest-approach point along the chord to moon centre.
 *      Use it as the silhouette direction → lunar_R_at returns r_eff.
 *      Test against r_eff sphere. If hit, return nearest valid t.
 *
 * Without the cheap reject, every NEE chord would do a 14+30 = 44-
 * Gaussian evaluation on the LUT lookup path. With it, ~80% of NEE
 * chords miss the unperturbed sphere and never touch the LUT.
 *
 * Inputs:      ro            ray origin
 *              rd            ray direction (unit)
 *              c             moon centre (world)
 *              moon_R_base   unperturbed moon radius
 *              seed          unused at runtime
 *              out_t         filled with hit parameter on hit
 * Returns:     true on hit (perturbed sphere), false on miss.
 *
 * Why: the EYE-RAY hit test for the moon. The NEE penumbra calculation
 * uses §8 instead — there the moon-disc-vs-sun-disc geometry is the
 * point, and the penumbra formula handles partial coverage analytically.
 */
static bool ray_moon(V3 ro, V3 rd, V3 c, float moon_R_base, int seed,
                     float *out_t) {
  V3 oc = v3_sub(ro, c);
  float b = v3_dot(oc, rd);

  /* Cheap reject against base sphere. */
  float oc2 = v3_dot(oc, oc);
  float r2 = moon_R_base * moon_R_base;
  float disc = b * b - (oc2 - r2);
  if (disc < 0)
    return false;
  float sq = sqrtf(disc);
  float t_far = -b + sq;
  if (t_far < 1e-3f)
    return false;

  /* Now evaluate perturbation. */
  V3 nearest = v3_add(ro, v3_scl(rd, -b));
  V3 radial = v3_sub(nearest, c);
  float r_eff = lunar_R_at(v3_norm(radial), moon_R_base, seed);

  float cc_p = oc2 - r_eff * r_eff;
  float disc_p = b * b - cc_p;
  if (disc_p < 0)
    return false;
  float sq_p = sqrtf(disc_p);
  float t = -b - sq_p;
  if (t < 1e-3f)
    t = -b + sq_p;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* ── §8 penumbra — analytic visible-fraction-of-sun-disc ─────────────── *
 *
 * The path tracer's NEE step (§11) needs to know, at every scatter
 * point, how much of the sun's apparent disc is unblocked by the
 * moon. A binary "is the moon between me and the sun?" test would
 * give a hard cliff at the moon's edge — wrong; real penumbra is a
 * smooth gradient. We compute the gradient analytically as the
 * fraction of the sun's apparent disc area not covered by the moon's.
 *
 * From a scatter point p, both sun and moon look like circular discs
 * on the unit sphere around p. For small angular extents (a few
 * degrees, true here) the spherical geometry approximates well as
 * planar circles in a tangent plane — circle-circle intersection has
 * a closed form.
 */

/*
 * visible_fraction_sun — analytic [0..1] visibility of the sun.
 *
 * Purpose:     return the unobstructed fraction of the sun's apparent
 *              disc as seen from world point p, given the moon
 *              partially occludes it. Replaces a binary moon-block
 *              test in NEE — gives smooth penumbra.
 *
 * Math:
 *      α_sun  = asin(SUN_R  / |sun − p|)        sun's angular radius
 *      α_moon = asin(moon_R / |moon − p|)       moon's
 *      γ      = angle between (sun − p) and (moon − p)
 *
 *      γ ≥ α_sun + α_moon                     → visible = 1
 *      γ ≤ |α_sun − α_moon| AND α_moon ≥ α_sun → visible = 0    (total)
 *      γ ≤ |α_sun − α_moon| AND α_moon <  α_sun → visible = 1−(α_moon/α_sun)²
 *                                                               (annular)
 *      else: partial overlap, area formula:
 *          A1 = α_sun² · acos((γ²+α_sun²−α_moon²)/(2γα_sun))
 *          A2 = α_moon² · acos((γ²+α_moon²−α_sun²)/(2γα_moon))
 *          A3 = ½ · √(prod of side-length-Heron factors)
 *          overlap = A1 + A2 − A3
 *          visible = 1 − overlap / (π · α_sun²)
 *
 * Inputs:      p          world point we're sampling visibility from
 *              sun_pos    sun centre (world)
 *              moon_pos   moon centre (world)
 *              moon_R     moon radius (world; pass scene_moon_r value)
 * Returns:     visible fraction ∈ [0, 1]; clamped on output.
 *
 * Why: Bailey's beads come from EYE-RAY perturbations (§7.3); penumbra
 * comes from NEE scattering. Different physics, different code paths,
 * but they compose: the eye ray hits the perturbed moon (beads); the
 * NEE-from-corona chords use the unperturbed disc here (smooth
 * penumbra). Trying to combine both into one function gives tangled
 * formulas; separate is cleaner.
 */
static float visible_fraction_sun(V3 p, V3 sun_pos, V3 moon_pos, float moon_R) {
  V3 to_sun_v = v3_sub(sun_pos, p);
  V3 to_moon_v = v3_sub(moon_pos, p);
  float dist_sun = v3_len(to_sun_v);
  float dist_moon = v3_len(to_moon_v);
  if (dist_sun < SUN_R + 1e-3f)
    return 0.0f;
  if (dist_moon < moon_R + 1e-3f)
    return 0.0f;

  float a_sun = asinf(SUN_R / dist_sun);
  float a_moon = asinf(moon_R / dist_moon);

  V3 d_sun = v3_scl(to_sun_v, 1.0f / dist_sun);
  V3 d_moon = v3_scl(to_moon_v, 1.0f / dist_moon);
  float cos_g = v3_dot(d_sun, d_moon);
  if (cos_g > 1.0f)
    cos_g = 1.0f;
  if (cos_g < -1.0f)
    cos_g = -1.0f;
  float gamma = acosf(cos_g);

  if (gamma >= a_sun + a_moon)
    return 1.0f;
  if (gamma <= fabsf(a_sun - a_moon)) {
    if (a_moon >= a_sun)
      return 0.0f;
    float r = a_moon / a_sun;
    return 1.0f - r * r;
  }

  /* Partial overlap: circle-circle intersection area in angle space. */
  float d = gamma;
  float r1 = a_sun, r2 = a_moon;
  float r1s = r1 * r1, r2s = r2 * r2;
  float a1_arg = (d * d + r1s - r2s) / (2.0f * d * r1);
  float a2_arg = (d * d + r2s - r1s) / (2.0f * d * r2);
  if (a1_arg > 1.0f)
    a1_arg = 1.0f;
  if (a1_arg < -1.0f)
    a1_arg = -1.0f;
  if (a2_arg > 1.0f)
    a2_arg = 1.0f;
  if (a2_arg < -1.0f)
    a2_arg = -1.0f;
  float A1 = r1s * acosf(a1_arg);
  float A2 = r2s * acosf(a2_arg);

  float t1 = -d + r1 + r2;
  float t2 = d + r1 - r2;
  float t3 = d - r1 + r2;
  float t4 = d + r1 + r2;
  float prod = t1 * t2 * t3 * t4;
  if (prod < 0.0f)
    prod = 0.0f;
  float A3 = 0.5f * sqrtf(prod);

  float overlap = A1 + A2 - A3;
  float sun_a = (float)M_PI * r1s;
  if (sun_a < 1e-9f)
    return 1.0f;
  float vis = 1.0f - overlap / sun_a;
  if (vis < 0.0f)
    vis = 0.0f;
  if (vis > 1.0f)
    vis = 1.0f;
  return vis;
}

/* ── §9 coronal medium ───────────────────────────────────────────────── *
 *
 * The medium has two parts: a spherically-symmetric component (corona +
 * chromosphere shell) and an anisotropic component (prominences). The
 * separation matters because:
 *   • The 1-D radial transmittance LUT (§10) only sees the symmetric
 *     part. Prominences are too small/sparse to affect it visibly.
 *   • Per-frame medium state (Medium struct) holds prominence parameters
 *     that change with seed; the symmetric part is hard-coded and
 *     identical every frame.
 *
 * §9 has four pieces:
 *   §9.1 Medium struct
 *   §9.2 sigma_sph         spherical scattering coefficient at radius r
 *   §9.3 sigma_prom        anisotropic prominence contribution
 *   §9.4 density_total + emission_at  — combined hot-path entry points
 */

/* §9.1 Medium struct.
 *
 * Per-frame state for the coronal medium. The kelvin field is stamped
 * by scene_draw with the current stellar class. Prominence parameters
 * are filled in by medium_reseed (§12.3) when 'r' is pressed.
 */
typedef struct {
  float kelvin;                  /* photosphere temperature        */
  float prom_phi[PROM_COUNT];    /* angular position (rad)         */
  float prom_height[PROM_COUNT]; /* angular reach × R_sun          */
  float prom_amp[PROM_COUNT];    /* density multiplier             */
} Medium;

/* §9.2 sigma_sph — spherically-symmetric σ at radius r.
 *
 * Purpose:     return the local scattering coefficient at distance r
 *              from the sun centre, as the sum of corona power-law
 *              and chromosphere shell.
 *
 * Pseudocode:
 *      if r < R_sun  or  r > REACH·R_sun:  return 0
 *      corona = SIGMA0 · (R_sun / r)^DECAY
 *      h     = (r − R_sun) / (CHROMOS_THICK · R_sun)         ∈ [0, ?]
 *      if h ∈ [0, 1]:
 *          chromos = CHROMOS_SIGMA0 · (1−h)²        ← peaks at shell base
 *      else:
 *          chromos = 0
 *      return corona + chromos
 *
 * Inputs:      r   distance from sun centre (world units)
 * Returns:     σ at this radius, units 1/world-unit.
 *
 * Why: the only function the trans_lut (§10) integrates. Spherical
 * symmetry is what makes the LUT possible — a 1-D table τ(r). Adding
 * anisotropy here would force a 2-D or 3-D LUT with proportionally
 * larger memory; instead we keep prominences in §9.3, where they
 * contribute to the volume rendering integral as additive emission
 * but not to the radial transmittance.
 */
static float sigma_sph(float r) {
  if (r < SUN_R)
    return 0.0f;
  if (r > SUN_R * CORONA_REACH)
    return 0.0f;

  float u = SUN_R / r;
  float cor = CORONA_SIGMA0 * powf(u, CORONA_DECAY);

  float h = (r - SUN_R) / (SUN_R * CHROMOS_THICK);
  float chrom = 0.0f;
  if (h >= 0.0f && h <= 1.0f) {
    float fade = 1.0f - h;
    chrom = CHROMOS_SIGMA0 * fade * fade;
  }
  return cor + chrom;
}

/* §9.3 sigma_prom — additive prominence density.
 *
 * Purpose:     return extra σ at point p_local from prominence loops.
 *              Each prominence is a 2-D Gaussian in (phi, h_above)
 *              centred at a fixed angular position with random height
 *              and amplitude.
 *
 * Pseudocode:
 *      r       = |p_local|
 *      if r < R_sun or r > (1+PROM_HEIGHT)·R_sun: return 0
 *      h_above = (r − R_sun) / R_sun         ∈ [0, PROM_HEIGHT]
 *      phi     = atan2(p_local.x, p_local.z)
 *      total   = 0
 *      for k ∈ [0, PROM_COUNT):
 *          dphi    = wrap(phi − prom_phi[k]) ∈ (−π, π]
 *          lateral = exp(−dphi² / PROM_LATERAL²)
 *          radial  = (1 − h_above/prom_height[k])² if in range else 0
 *          total += prom_amp[k] · lateral · radial
 *      return PROM_SIGMA0 · total
 *
 * Inputs:      m         medium with seed-dependent prominence params
 *              p_local   point in sun-centred coords
 * Returns:     additional σ from prominences, ≥ 0.
 *
 * Why: lets us paint a few thick "loops" of cool plasma rising from
 * the chromosphere without breaking spherical symmetry of σ_sph.
 * Prominences emit Hα strongly (see emission_at) — they're the
 * brightest features at totality apart from the photosphere itself.
 */
static float sigma_prom(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return 0.0f;
  if (r > SUN_R * (1.0f + PROM_HEIGHT))
    return 0.0f;

  float phi = atan2f(p_local.x, p_local.z);
  float h_above = (r - SUN_R) / SUN_R;
  if (h_above > PROM_HEIGHT)
    return 0.0f;

  float total = 0.0f;
  for (int k = 0; k < PROM_COUNT; k++) {
    float reach_k = m->prom_height[k];
    if (h_above > reach_k)
      continue;
    float dphi = phi - m->prom_phi[k];
    while (dphi > (float)M_PI)
      dphi -= 2.0f * (float)M_PI;
    while (dphi < -(float)M_PI)
      dphi += 2.0f * (float)M_PI;
    float lat = expf(-(dphi * dphi) / (PROM_LATERAL * PROM_LATERAL));
    float t_rad = h_above / reach_k;
    float rad = (1.0f - t_rad) * (1.0f - t_rad);
    total += m->prom_amp[k] * lat * rad;
  }
  return PROM_SIGMA0 * total;
}

/* §9.4 density_total + emission_at — the path tracer's entry points.
 *
 * density_total just sums sigma_sph(r) and sigma_prom(p) — the path
 * tracer asks for "total σ at this point" once per active step.
 *
 * emission_at returns the Hα emission radiance at p — chromosphere
 * plus prominence contributions. The chromosphere part has spicule
 * texture (fBm in spherical coords) and a hot-peak hue shift; the
 * prominence part stays at the cool baseline Hα colour.
 */

static float density_total(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  return sigma_sph(r) + sigma_prom(m, p_local);
}

/*
 * emission_at — local volumetric emission per unit length at p_local.
 *
 * Purpose:     return the radiance emitted per metre by the chromos-
 *              phere + prominences at world-point p_local. The corona
 *              itself doesn't emit — it only scatters photospheric
 *              light, which is handled in the path tracer's NEE step.
 *
 * Pseudocode:
 *      r = |p_local|
 *      if outside chromos+prom range: return (0,0,0)
 *      ha_cool = (1.00, 0.18, 0.12)        cool Hα baseline
 *      ha_hot  = (1.00, 0.45, 0.18)        hot peak (yellow-shifted)
 *      h = (r − R_sun) / (CHROMOS_THICK · R_sun)
 *      chr_em = 0,  spicule_n = 0
 *      if h ∈ [0, 1]:
 *          fade   = (1 − h)^4               sharp outer edge
 *          theta  = acos(p_local.y / r)
 *          phi    = atan2(p_local.x, p_local.z)
 *          spicule_n = fbm2(phi · 12, theta · 6)        ∈ [0, 1]
 *          spicule_factor = SPICULE_BASE + SPICULE_AMP · spicule_n
 *          chr_em = CHROMOS_SIGMA0 · fade · spicule_factor
 *      pr_em = sigma_prom(m, p_local)
 *      chr_col = lerp(ha_cool, ha_hot, spicule_n)        hot-peak shift
 *      return chr_col · chr_em + ha_cool · pr_em
 *
 * Inputs:      m         medium
 *              p_local   world point in sun-centred coords
 * Returns:     RGB radiance per unit length (HDR).
 *
 * Why: chromosphere isn't smooth — it's textured by spicules. fBm in
 * (theta, phi) gives radial-streak patches that map to vertical
 * streaks on the visible chromos ring. Hot peaks lean yellow because
 * real spicule heating shifts emission lines.
 */
static RGB emission_at(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return rgb_make(0, 0, 0);
  if (r > SUN_R * (1.0f + PROM_HEIGHT))
    return rgb_make(0, 0, 0);

  RGB ha_cool = rgb_make(1.00f, 0.18f, 0.12f);
  RGB ha_hot = rgb_make(1.00f, 0.45f, 0.18f);

  float chr_em = 0.0f;
  float spicule_n = 0.0f;
  float h = (r - SUN_R) / (SUN_R * CHROMOS_THICK);
  if (h >= 0.0f && h <= 1.0f) {
    float fade = 1.0f - h;
    fade = fade * fade * fade * fade;

    float r_safe = fmaxf(r, 1e-6f);
    float yy = p_local.y / r_safe;
    if (yy > 1.0f)
      yy = 1.0f;
    if (yy < -1.0f)
      yy = -1.0f;
    float theta = acosf(yy);
    float phi = atan2f(p_local.x, p_local.z);
    spicule_n = fbm2(phi * SPICULE_FREQ_PHI, theta * SPICULE_FREQ_TH);
    float spicule_factor = SPICULE_BASE + SPICULE_AMP * spicule_n;

    chr_em = CHROMOS_SIGMA0 * fade * spicule_factor;
  }

  float pr_em = sigma_prom(m, p_local);

  RGB chr_col = rgb_make(ha_cool.r + (ha_hot.r - ha_cool.r) * spicule_n,
                         ha_cool.g + (ha_hot.g - ha_cool.g) * spicule_n,
                         ha_cool.b + (ha_hot.b - ha_cool.b) * spicule_n);

  RGB out = rgb_scl(chr_col, chr_em);
  out = rgb_add(out, rgb_scl(ha_cool, pr_em));
  return out;
}

/* ── §10 transmittance LUT ───────────────────────────────────────────── *
 *
 * The realtime trick. The path tracer needs, at every active scatter
 * point, the transmittance from that point to the photosphere along
 * the radial inward chord. Computing it by ray-marching at runtime
 * would be O(steps²) per pixel — unaffordable. Spherical symmetry of
 * σ_sph saves us: the optical depth along the radial chord from r to
 * R_sun depends only on r. Tabulate τ(r) once at startup, lookup at
 * runtime.
 */

static float trans_lut[LUT_SIZE];

/* §10.1 build_trans_lut — precompute τ(r) at LUT_SIZE radii.
 *
 * Purpose:     fill `trans_lut[]` with the cumulative radial optical
 *              depth from R_sun out to LUT_R_MAX. Called once at
 *              startup; not seed-dependent (σ_sph doesn't change with
 *              seed).
 *
 * Pseudocode:
 *      trans_lut[0] = 0                 ← τ at r = R_sun
 *      r_prev = R_sun
 *      for i ∈ [1, LUT_SIZE):
 *          r_curr = R_sun + (LUT_R_MAX − R_sun) · (i / (LUT_SIZE − 1))
 *          ds     = (r_curr − r_prev) / N_INT       fine sub-step
 *          tau    = trans_lut[i−1]
 *          for j ∈ [0, N_INT):
 *              s    = r_prev + (j + 0.5) · ds
 *              tau += sigma_sph(s) · ds
 *          trans_lut[i] = tau
 *          r_prev = r_curr
 *
 * Inputs:      none (uses globals LUT_SIZE, LUT_R_MAX, σ_sph).
 * Why: cumulative storage means lookup_transmittance can do a
 * one-bilinear-interp constant-time fetch.
 */
static void build_trans_lut(void) {
  const int N_INT = 32;
  trans_lut[0] = 0.0f;
  float r_prev = SUN_R;
  for (int i = 1; i < LUT_SIZE; i++) {
    float r_curr =
        SUN_R + (LUT_R_MAX - SUN_R) * ((float)i / (float)(LUT_SIZE - 1));
    float ds = (r_curr - r_prev) / (float)N_INT;
    float tau = trans_lut[i - 1];
    for (int j = 0; j < N_INT; j++) {
      float s = r_prev + (j + 0.5f) * ds;
      tau += sigma_sph(s) * ds;
    }
    trans_lut[i] = tau;
    r_prev = r_curr;
  }
}

/* §10.2 lookup_transmittance — fetch Tr at world point p_local.
 *
 * Purpose:     return exp(−τ(|p_local|)) by bilinear lookup. p_local
 *              is in sun-centred coordinates.
 *
 * Pseudocode:
 *      r = |p_local|
 *      if r < R_sun:                  return 0    (inside sun)
 *      if r > LUT_R_MAX:              return 1    (clear)
 *      fr = (r − R_sun)/(LUT_R_MAX − R_sun) · (LUT_SIZE − 1)
 *      ir = floor(fr); frac = fr − ir
 *      tau = trans_lut[ir] · (1−frac) + trans_lut[ir+1] · frac
 *      return exp(−tau)
 *
 * Inputs:      p_local   point in sun-centred coords.
 * Returns:     Tr ∈ [0, 1].
 *
 * Why: the inner-loop call from trace_ray's NEE. One memory access,
 * three multiplies, one expf — that's the entire NEE-transmittance
 * cost per active step.
 */
static float lookup_transmittance(V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return 0.0f;
  if (r > LUT_R_MAX)
    return 1.0f;

  float fr = (r - SUN_R) / (LUT_R_MAX - SUN_R) * (float)(LUT_SIZE - 1);
  int ir = (int)fr;
  if (ir < 0)
    ir = 0;
  if (ir > LUT_SIZE - 2)
    ir = LUT_SIZE - 2;
  float t = fr - (float)ir;
  float tau = trans_lut[ir] * (1.0f - t) + trans_lut[ir + 1] * t;
  return expf(-tau);
}

/* ── §11 path tracer ─────────────────────────────────────────────────── *
 *
 * The kernel. One function — trace_ray — handles a complete primary
 * ray: cast it from the camera, find the nearest opaque surface,
 * march through the medium accumulating in-scattered + emitted
 * radiance, then add the surface contribution.
 *
 * The volume rendering equation (Tutorial 4) becomes:
 *
 *   radiance_accum = ∫₀^t_max T(t) · [emission(t) + scatter(t)] dt
 *                  + T(t_max) · L_surface
 *
 * where T(t) = exp(−∫₀^t σ(s) ds) is the transmittance from camera to
 * march parameter t. We approximate the integral with a Riemann sum
 * (adaptive step size; fine inside the corona's reach, 4× coarse
 * outside).
 *
 * §11 has two pieces:
 *   §11.1 thomson_phase  Thomson scattering phase function
 *   §11.2 trace_ray      the kernel
 */

/* §11.1 thomson_phase.
 *
 * Purpose:     Thomson scattering phase function at angle θ between
 *              incident and outgoing directions. Wavelength-flat
 *              (achromatic) — that's why the corona's hue follows the
 *              photosphere's exactly.
 *
 *      P_T(θ) = (3/16π) · (1 + cos²θ)
 *
 * Inputs:      cos_theta   cos of scatter angle ∈ [-1, 1]
 * Returns:     phase value, 1/sr.
 *
 * Why: NEE computes scatter-toward-eye via this phase function. Other
 * media (Mie scattering for clouds, Rayleigh for blue sky) would use
 * different phases here. Thomson is correct for free electrons in
 * the K-corona.
 */
static inline float thomson_phase(float cos_theta) {
  return (3.0f / (16.0f * (float)M_PI)) * (1.0f + cos_theta * cos_theta);
}

/* §11.2 trace_ray — the path tracer kernel.
 *
 * Purpose:     return the HDR radiance for a single eye ray from the
 *              camera through the medium, ending at sun / moon / sky.
 *
 * Inputs:
 *      ro              ray origin (= camera position, the world origin)
 *      rd              ray direction, unit length
 *      m               medium with this frame's prominence parameters
 *      sun_pos         world sun centre
 *      moon_pos        world moon centre
 *      moon_R          moon radius (after user-applied moon_scale)
 *      seed            shared with lunar terrain LUT (purely for
 *                      readability — actual data is baked into
 *                      lunar_lut at reseed time)
 *      sun_em          photospheric blackbody radiance (HDR; usually
 *                      ~SUN_EMIT_HDR × blackbody chromaticity)
 *      earth_tint      earthshine colour (blue-tinted sun_em)
 *      corona_gate     eye-adaptation multiplier ∈ [0, 1] for this frame
 *
 * Returns:     RGB pixel radiance (HDR).
 *
 * Pseudocode:
 *      hit_sun, t_sun  = ray_sphere(ro, rd, sun_pos, SUN_R)
 *      hit_moon, t_moon = ray_moon (ro, rd, moon_pos, moon_R, seed)
 *      pick (t_max, surf_type) by nearest hit (or sky)
 *
 *      transmittance = 1
 *      radiance_accum = (0, 0, 0)
 *      t_along_ray = 0
 *      while t_along_ray < t_max and transmittance > 1e-3:
 *          dt = fine_step if inside corona reach else 4× coarse
 *          midpoint p = ro + (t_along_ray + dt/2)·rd
 *          density = density_total(m, p − sun_pos)
 *          if density >= 1e-6:
 *              vis  = visible_fraction_sun(p, sun, moon, moon_R)
 *              if vis > 0:
 *                  omega   = 2π·(1 − cos α);  sin α = SUN_R/dist_sun
 *                  Tr      = lookup_transmittance(p − sun_pos)
 *                  ph      = thomson_phase(rd · (sun_pos−p)/dist_sun)
 *                  in_sc   = density · ph · Tr · omega · vis · gain · dt
 *                          · corona_gate
 *                  radiance_accum += transmittance · sun_em · in_sc
 *              em = emission_at(m, p − sun_pos)
 *              radiance_accum += transmittance · em · dt · HA_GAIN
 *                                              · corona_gate
 *              transmittance *= exp(−density · dt)
 *          t_along_ray += dt
 *
 *      surface contribution at t_max (multiplied by remaining T):
 *          sun_hit:   sun_em · (LIMB_AMBIENT + LIMB_GAIN · μ),
 *                     μ = −rd · sphere_normal_at_hit
 *          moon_hit:  Lambertian (sun direct, ≈ 0 in eclipse geometry)
 *                   + Earthshine (constant ambient · earth_tint)
 *          sky:       (no extra term)
 *      return radiance_accum
 *
 * Why: this is THE function. Everything else in the file is in service
 * of feeding it the right inputs and rendering its output. Read it
 * after Tutorials 4, 5, 6, 7, and 10.
 */
static RGB trace_ray(V3 ro, V3 rd, const Medium *m, V3 sun_pos, V3 moon_pos,
                     float moon_R, int seed, RGB sun_em, RGB earth_tint,
                     float corona_gate) {
  /* Surface intersections — moon uses perturbed-radius variant for
   * Bailey's-bead-correct silhouette (see Tutorial 8). */
  float t_sun = 0.f, t_moon = 0.f;
  bool hit_sun = ray_sphere(ro, rd, sun_pos, SUN_R, &t_sun);
  bool hit_moon = ray_moon(ro, rd, moon_pos, moon_R, seed, &t_moon);

  float t_max;
  int surf_type = 0; /* 0 sky · 1 sun · 2 moon */
  if (hit_moon && (!hit_sun || t_moon < t_sun)) {
    t_max = t_moon;
    surf_type = 2;
  } else if (hit_sun) {
    t_max = t_sun;
    surf_type = 1;
  } else {
    t_max = MARCH_T_MAX;
    surf_type = 0;
  }

  /* Adaptive step size: fine inside corona reach, 4× coarse outside.
   * Outside CORONA_REACH·R_sun, σ_sph ≡ 0 (and prominences live
   * close to the photosphere too) — coarse strides through truly
   * empty space contribute nothing regardless of length. The 0.5×
   * margin in the boundary keeps the chromosphere shell on the
   * fine-step side. */
  float dt_fine = t_max / (float)MARCH_STEPS;
  float dt_coarse = dt_fine * 4.0f;
  float corona_outer = SUN_R * (CORONA_REACH + 0.5f);
  float corona_outer_R2 = corona_outer * corona_outer;

  RGB radiance_accum = rgb_make(0.f, 0.f, 0.f);
  float transmittance = 1.0f;
  float t_along_ray = 0.0f;

  while (t_along_ray < t_max && transmittance > 1e-3f) {
    /* Pick step size based on distance from sun at the start of
     * the step. */
    V3 p_now = v3_add(ro, v3_scl(rd, t_along_ray));
    V3 p_now_loc = v3_sub(p_now, sun_pos);
    float r2_now = v3_dot(p_now_loc, p_now_loc);
    float dt = (r2_now < corona_outer_R2) ? dt_fine : dt_coarse;
    if (t_along_ray + dt > t_max)
      dt = t_max - t_along_ray;

    /* Sample at midpoint of this step. */
    float t_mid = t_along_ray + 0.5f * dt;
    V3 p = v3_add(ro, v3_scl(rd, t_mid));
    V3 p_local = v3_sub(p, sun_pos);
    float density = density_total(m, p_local);

    if (density >= 1e-6f) {
      V3 to_sun_v = v3_sub(sun_pos, p);
      float dist_sun = v3_len(to_sun_v);
      V3 to_sun = v3_scl(to_sun_v, 1.0f / fmaxf(dist_sun, 1e-6f));

      /* Continuous penumbra — soft moon shadow at the edge. */
      float vis = visible_fraction_sun(p, sun_pos, moon_pos, moon_R);

      if (vis > 1e-4f) {
        float sin_a = SUN_R / fmaxf(dist_sun, SUN_R + 1e-3f);
        if (sin_a > 1.0f)
          sin_a = 1.0f;
        float cos_a = sqrtf(fmaxf(1.0f - sin_a * sin_a, 0.0f));
        float omega = 2.0f * (float)M_PI * (1.0f - cos_a);

        float tr_to_sun = lookup_transmittance(p_local);
        float cos_th = v3_dot(rd, to_sun);
        float phase = thomson_phase(cos_th);
        float in_sc = density * phase * tr_to_sun * omega * vis *
                      IN_SCATTER_GAIN * dt * corona_gate;
        radiance_accum = rgb_add(
            radiance_accum, rgb_scl(rgb_scl(sun_em, in_sc), transmittance));
      }

      RGB em = emission_at(m, p_local);
      if (em.r + em.g + em.b > 0.f) {
        radiance_accum = rgb_add(
            radiance_accum,
            rgb_scl(em, transmittance * dt * HA_EMIT_GAIN * corona_gate));
      }

      transmittance *= expf(-density * dt);
    }
    t_along_ray += dt;
  }

  /* Surface contribution. */
  if (surf_type == 1) {
    /* Photosphere — emissive with Eddington limb darkening. */
    V3 p = v3_add(ro, v3_scl(rd, t_sun));
    V3 N = v3_norm(v3_sub(p, sun_pos));
    float mu = -v3_dot(N, rd);
    if (mu < 0.f)
      mu = 0.f;
    float lim = LIMB_AMBIENT + LIMB_GAIN * mu;
    radiance_accum =
        rgb_add(radiance_accum, rgb_scl(rgb_scl(sun_em, lim), transmittance));
  } else if (surf_type == 2) {
    /* Moon — Lambertian. In eclipse geometry cos(N, sun) ≤ 0 (moon
     * faces away from sun), so direct Lambertian sun-light is
     * essentially zero. Earthshine fills the dim cool-blue glow on
     * the moon's night side. */
    V3 p = v3_add(ro, v3_scl(rd, t_moon));
    V3 N = v3_norm(v3_sub(p, moon_pos));
    V3 to_sun_v = v3_sub(sun_pos, p);
    float dist_sun = v3_len(to_sun_v);
    V3 to_sun = v3_scl(to_sun_v, 1.0f / fmaxf(dist_sun, 1e-6f));
    float cos_sn = v3_dot(N, to_sun);

    RGB direct = rgb_make(0.f, 0.f, 0.f);
    if (cos_sn > 0.0f) {
      float Lf = MOON_ALBEDO / (float)M_PI * cos_sn;
      direct = rgb_scl(sun_em, Lf);
    }
    RGB earth = rgb_scl(earth_tint, EARTHSHINE_GAIN);
    radiance_accum =
        rgb_add(radiance_accum, rgb_scl(rgb_add(direct, earth), transmittance));
  }
  /* Sky case: no extra surface term. */

  return radiance_accum;
}

/* ── §12 scene state ─────────────────────────────────────────────────── *
 *
 * Per-frame state container: which pattern is active, current
 * stellar class, zoom and moon-scale knob positions, simulation time,
 * RNG seed, and the medium struct (with prominence parameters for
 * this seed). `pattern_set` and `*_reseed` mutate the scene; `scene_*`
 * read it.
 *
 * §12 has four sub-sections:
 *   §12.1 PatternParams + Scene structs
 *   §12.2 pattern_set
 *   §12.3 medium_reseed, scene_reseed, scene_init, scene_tick
 *   §12.4 scene_moon_r, scene_moon_pos, scene_occlusion
 */

/* §12.1 PatternParams + Scene.
 *
 * PatternParams holds the geometric configuration of the current
 * pattern: moon's world radius, optional y-offset (PARTIAL only),
 * gating booleans for HUD events. Scene wraps PatternParams plus all
 * the user-controllable knobs and the Medium.
 *
 * No bitfields, no unions — flat structs, easy to read.
 */

typedef struct {
  float moon_world_r;
  float moon_y_world;
  bool totality_possible;
  bool show_corona; /* unused at runtime, kept for HUD */
} PatternParams;

typedef struct {
  bool paused;
  int speed;
  int star_idx;
  float zoom;       /* effective FOV = FOV_H_BASE / zoom */
  float moon_scale; /* live moon-radius multiplier */
  Pattern current_pattern;
  PatternParams pp;
  float time_secs;
  float seed_phase;
  int seed;
  Medium medium;
} Scene;

/* §12.2 pattern_set.
 *
 * Purpose:     apply the geometric defaults for a pattern enum. Called
 *              when n / p is pressed.
 *
 * Pseudocode:
 *      pp = (defaults: TOTAL moon radius, no offset)
 *      switch p:
 *          TOTAL    → totality_possible = true
 *          PARTIAL  → moon offset above midline by PARTIAL_Y_OFFSET·MOON_Z
 *          ANNULAR  → moon radius = MOON_BASE_R_ANNULAR (smaller)
 *          TRANSIT  → moon radius = MOON_BASE_R_TRANSIT (tiny)
 *
 * Why: rather than hard-coding pattern-specific bits in the path
 * tracer, we change the world geometry. Same kernel renders all four
 * patterns by reading current Scene state.
 */
static void pattern_set(Scene *s, Pattern p) {
  s->current_pattern = p;
  PatternParams *pp = &s->pp;
  pp->moon_world_r = MOON_BASE_R_TOTAL;
  pp->moon_y_world = 0.0f;
  pp->totality_possible = false;
  pp->show_corona = true;
  switch (p) {
  case PATTERN_TOTAL:
    pp->moon_world_r = MOON_BASE_R_TOTAL;
    pp->totality_possible = true;
    break;
  case PATTERN_PARTIAL:
    pp->moon_world_r = MOON_BASE_R_TOTAL;
    pp->moon_y_world = PARTIAL_Y_OFFSET * MOON_Z;
    break;
  case PATTERN_ANNULAR:
    pp->moon_world_r = MOON_BASE_R_ANNULAR;
    break;
  case PATTERN_TRANSIT:
    pp->moon_world_r = MOON_BASE_R_TRANSIT;
    break;
  case N_PATTERNS:
    break;
  }
}

/* §12.3 reseeding + init + tick.
 *
 * medium_reseed regenerates the prominence positions and rebuilds the
 * lunar terrain LUT and Perlin permutation. scene_reseed picks a new
 * orbit phase and seed, then delegates. scene_init zeroes the Scene
 * and calls medium_reseed once. scene_tick advances simulation time
 * proportional to user-set speed.
 */

static void medium_reseed(Medium *m, int seed) {
  /* Random prominence positions (angle, height, amplitude). */
  for (int k = 0; k < PROM_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0xF1A3E);
    m->prom_phi[k] = hash01(h) * 2.0f * (float)M_PI;
    m->prom_height[k] = PROM_HEIGHT * (0.40f + 0.60f * hash01(h ^ 0xA1u));
    m->prom_amp[k] = 0.50f + 0.50f * hash01(h ^ 0xB2u);
  }
  /* Bake the moon's silhouette perturbation. */
  build_lunar_lut(seed);

  /* Shuffle Perlin permutation table for spicule fBm. */
  perm_shuffle(seed);
}

static void scene_reseed(Scene *s) {
  uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                     (int)(s->seed_phase * 100.0f), 0xC0FFEE);
  s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
  s->seed = (int)(h ^ 0x5A5A5A5Au);
  medium_reseed(&s->medium, s->seed);
}

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->star_idx = 2; /* G-STAR (the Sun) by default */
  s->zoom = 1.0f;
  s->moon_scale = 1.0f;
  s->seed_phase = 0.0f;
  s->seed = 0xDECAF;
  medium_reseed(&s->medium, s->seed);
  pattern_set(s, PATTERN_TOTAL);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  s->time_secs += dt * speed_mul;
}

/* §12.4 derived geometry queries.
 *
 * scene_moon_r returns the EFFECTIVE moon radius (base × user scale).
 * scene_moon_pos returns the current moon world position from orbit
 * parameters (sin curve over ECLIPSE_PERIOD_S seconds). scene_occlusion
 * returns the fraction of the sun's apparent disc covered by the moon
 * AS SEEN FROM THE CAMERA — used by the HUD progress bar and by the
 * background-stars gate (§14.2).
 *
 * Note: scene_occlusion is the "binary occlusion" formula appropriate
 * for the camera's view (it includes the annular-cap case). The path
 * tracer uses visible_fraction_sun (§8) for the inverse — fraction
 * NOT covered, returned as a continuous value.
 */

static inline float scene_moon_r(const Scene *s) {
  return s->pp.moon_world_r * s->moon_scale;
}

static V3 scene_moon_pos(const Scene *s) {
  float sun_a = atanf(SUN_R / SUN_Z);
  float moon_a = atanf(scene_moon_r(s) / MOON_Z);
  float orbit_x_world = MOON_Z * (sun_a + moon_a) * MOON_ORBIT_X_FRAC;
  float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
  float phase = omega * s->time_secs + s->seed_phase;
  float mx = orbit_x_world * sinf(phase);
  return v3(mx, s->pp.moon_y_world, MOON_Z);
}

static float scene_occlusion(const Scene *s) {
  V3 sun_pos = v3(0, 0, SUN_Z);
  V3 moon_pos = scene_moon_pos(s);
  V3 sun_dir = v3_norm(sun_pos);
  V3 moon_dir = v3_norm(moon_pos);
  float cos_b = v3_dot(sun_dir, moon_dir);
  if (cos_b > 1.f)
    cos_b = 1.f;
  if (cos_b < -1.f)
    cos_b = -1.f;
  float sep = acosf(cos_b);
  float sun_a = atanf(SUN_R / SUN_Z);
  float moon_a = atanf(scene_moon_r(s) / MOON_Z);
  float sum = sun_a + moon_a;
  float dif = fabsf(sun_a - moon_a);
  if (sep >= sum)
    return 0.f;
  if (sep <= dif) {
    if (moon_a >= sun_a)
      return 1.f;
    return (moon_a / sun_a) * (moon_a / sun_a);
  }
  float t = (sum - sep) / (2.0f * fminf(sun_a, moon_a));
  if (moon_a >= sun_a)
    return t;
  return t * (moon_a / sun_a) * (moon_a / sun_a);
}

/* ── §13 camera ──────────────────────────────────────────────────────── *
 *
 * Camera is a tiny struct holding origin, fov values, and screen
 * dimensions. camera_make stamps in the per-frame fov (computed from
 * the user's zoom in scene_draw). camera_ray converts a (col, row)
 * cell coordinate to a world-space unit direction using the pinhole
 * formula in Tutorial 2.
 */

typedef struct {
  V3 pos;
  float fov_h, fov_v;
  int cols, rows;
} Camera;

static void camera_make(Camera *c, int cols, int rows, float fov_h) {
  c->cols = cols;
  c->rows = rows;
  c->pos = v3(0, 0, 0);
  c->fov_h = fov_h;
  c->fov_v = fov_h * (float)rows * ASPECT_Y / (float)cols;
}

static V3 camera_ray(const Camera *c, int sx, int sy) {
  float u =
      ((2.0f * (float)sx + 1.0f) - (float)c->cols) / (float)c->cols * c->fov_h;
  float v =
      -((2.0f * (float)sy + 1.0f) - (float)c->rows) / (float)c->rows * c->fov_v;
  return v3_norm(v3(u, v, 1.0f));
}

/* ── §14 screen + scene_draw ─────────────────────────────────────────── *
 *
 * Per-frame orchestration. screen_init / screen_resize manage the
 * ncurses window. scene_draw is the per-frame entry point: build the
 * camera, set up the medium and emission constants, then iterate
 * every cell calling trace_ray and storing the result in a
 * framebuffer; finally paint the framebuffer to the terminal.
 *
 * §14 has two pieces:
 *   §14.1 screen_init / screen_resize
 *   §14.2 scene_draw
 */

typedef struct {
  int cols, rows;
} Screen;

/* §14.1 screen lifecycle.
 *
 * screen_init configures ncurses for a non-blocking, no-echo, key-
 * captured, cursor-hidden TUI. typeahead(-1) is a small but important
 * call: it makes ncurses accept all queued input on every refresh
 * rather than blocking. screen_resize is called from the SIGWINCH
 * handler in §16.
 */

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

/* §14.2 scene_draw — per-frame orchestration.
 *
 * Purpose:     for each cell in the visible window, dispatch a
 *              path-traced ray; store the HDR result in g_buf; then
 *              paint the buffer to the terminal.
 *
 * Pseudocode:
 *      compute working rows/cols within bounds (BUF_MAX_*)
 *      build camera with fov_h = FOV_H_BASE / zoom
 *      stamp medium.kelvin from current Star
 *      sun_em      = SUN_EMIT_HDR · blackbody_rgb(kelvin)
 *      earth_alb   = (0.30, 0.45, EARTH_ALBEDO_BLUE)
 *      earth_tint  = sun_em · earth_alb               (componentwise)
 *      cam_vis_sun = visible_fraction_sun(camera, sun, moon, moon_R)
 *      corona_gate = FLOOR + RANGE · (1 − cam_vis_sun)
 *
 *      PASS 1 — for each (r, c):
 *          ray = camera_ray(c, r)
 *          col = trace_ray(camera_origin, ray, medium,
 *                          sun_pos, moon_pos, moon_R, seed,
 *                          sun_em, earth_tint, corona_gate)
 *          if col is dark and totality:
 *              maybe add a background star at (c, r) from per-cell hash
 *          g_buf[r][c] = col
 *
 *      PASS 2 — paint g_buf to terminal via paint_cell
 *
 * Why: keeps trace_ray pure (it doesn't know about the eye-adapt
 * gate, the framebuffer, or the stars). All per-frame setup happens
 * here, leaving the kernel free of bookkeeping.
 */

static RGB g_buf[BUF_MAX_H][BUF_MAX_W];

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

  V3 sun_pos = v3(0, 0, SUN_Z);
  V3 moon_pos = scene_moon_pos(s);
  float moon_R = scene_moon_r(s);

  Medium m_local = s->medium;
  m_local.kelvin = STARS[s->star_idx].kelvin;

  RGB sun_chrom = blackbody_rgb(m_local.kelvin);
  RGB sun_em = rgb_scl(sun_chrom, SUN_EMIT_HDR);

  /* Earthshine: rough Earth-as-mirror with Rayleigh-blue albedo. */
  RGB earth_alb = rgb_make(0.30f, 0.45f, EARTH_ALBEDO_BLUE);
  RGB earth_tint = rgb_mul(sun_em, earth_alb);

  /* Eye-adaptation gate — one number per frame. */
  float cam_vis_sun = visible_fraction_sun(cam.pos, sun_pos, moon_pos, moon_R);
  float corona_gate =
      CORONA_GATE_FLOOR + CORONA_GATE_RANGE * (1.0f - cam_vis_sun);

  /* PASS 1 — path tracer for each cell. */
  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      V3 ray_d = camera_ray(&cam, c, r);
      RGB col = trace_ray(cam.pos, ray_d, &m_local, sun_pos, moon_pos, moon_R,
                          s->seed, sun_em, earth_tint, corona_gate);

      /* Background stars — only at near-totality and only for
       * "sky" cells (luma below threshold from the path
       * tracer). Per-cell hash keeps positions stable across
       * frames; per-star Kelvin is log-uniform in [3000, 30000]
       * for a chromaticity spread. */
      if (luma_of(col) < 0.04f && scene_occlusion(s) > STAR_OCC_VISIBLE) {
        uint32_t h = hash3(c, r, s->seed);
        if ((h % STAR_DENSITY) == 0u) {
          float br = 0.6f + hash01(h >> 8) * 0.4f;
          float u_K = hash01(h ^ 0xC0DEC0DEu);
          float kK = STAR_K_MIN * powf(STAR_K_MAX / STAR_K_MIN, u_K);
          col = rgb_add(col, rgb_scl(blackbody_rgb(kK), br));
        }
      }

      g_buf[r][c] = col;
    }
  }

  /* PASS 2 — paint to screen. */
  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      paint_cell(c, r + row_off, g_buf[r][c]);
    }
  }
}

/* ── §15 hud ─────────────────────────────────────────────────────────── *
 *
 * One row at the top with status, plus a TOTALITY event flash, plus a
 * key-hint strip at the bottom. Per project convention everything is
 * bright yellow on default background (PAIR_HUD) for the status, with
 * the bottom strip in cyan A_BOLD (PAIR_HINT) and the event flash in
 * red A_BOLD (PAIR_EVENT_HOT).
 *
 * The status string packs in a lot:
 *   fps, simulation Hz, pattern, stellar class, sun and moon angular
 *   radii (degrees), occlusion progress bar, zoom, moon-scale, speed.
 */

/*
 * hud_draw — render the top status row + bottom hint strip.
 *
 * Purpose:     draw the per-frame status overlay. Read-only on Scene.
 *
 * Inputs:      sc        Screen with current cols, rows
 *              s         Scene to read state from
 *              fps       measured render fps
 *              sim_fps   current simulation tick rate
 *
 * Why: separated from scene_draw so the path tracer's framebuffer
 * doesn't need to know about the HUD's row reservation.
 */
static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  float occ = scene_occlusion(s);

  int bar_w = 10;
  int bar_fill = (int)(occ * (float)bar_w + 0.5f);
  if (bar_fill > bar_w)
    bar_fill = bar_w;
  if (bar_fill < 0)
    bar_fill = 0;
  char bar[16] = {0};
  for (int i = 0; i < bar_w; i++)
    bar[i] = (i < bar_fill) ? '#' : '.';

  const Star *st = &STARS[s->star_idx];
  float sun_a = atanf(SUN_R / SUN_Z) * 180.f / (float)M_PI;
  float moon_a = atanf(scene_moon_r(s) / MOON_Z) * 180.f / (float)M_PI;
  char buf[300];
  snprintf(
      buf, sizeof buf,
      " %5.1f fps %3d Hz  %s  %s %5.0fK  s:%4.2f m:%4.2f deg  [%s] %3.0f%%  "
      "z:%3.1fx mz:%3.1fx  spd:%d ",
      fps, sim_fps, s->paused ? "PAUSED " : pattern_name(s->current_pattern),
      st->name, (double)st->kelvin, (double)sun_a, (double)moon_a, bar,
      (double)(occ * 100.0f), (double)s->zoom, (double)s->moon_scale, s->speed);
  int len = (int)strlen(buf);
  if (len > sc->cols)
    len = sc->cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " SOLAR-ECLIPSE-PT · PATH-TRACED ");
  attroff(COLOR_PAIR(PAIR_HUD));

  /* Totality event flash. */
  if (s->pp.totality_possible && occ > 0.99f) {
    const char *lab = " [ TOTALITY ] ";
    int elen = (int)strlen(lab);
    attron(COLOR_PAIR(PAIR_EVENT_HOT) | A_BOLD);
    mvprintw(0, sc->cols - len - elen - 1, "%s", lab);
    attroff(COLOR_PAIR(PAIR_EVENT_HOT) | A_BOLD);
  }

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  n/p:pat  t/T:star  z/Z:zoom  m/M:moon  +/-:spd "
           " []:Hz  r:reseed ");
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

/* ── §16 app — signals + fixed-step main loop ────────────────────────── *
 *
 * Standard project pattern:
 *   • Signal handlers set volatile flags; the main loop polls them.
 *   • atexit registers cleanup (endwin) so the terminal is restored
 *     even if we crash.
 *   • Fixed-step accumulator advances simulation time at SIM_FPS;
 *     render runs at whatever rate the rest of the loop achieves.
 *   • Frame-pacing sleeps target ~60 fps render budget.
 *
 * §16 has three pieces:
 *   §16.1 signals + cleanup
 *   §16.2 app_handle_key
 *   §16.3 app_init / app_run / main
 */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* §16.1 signal handlers. Set flags, return immediately — the main loop
 * checks them after the next getch() / scene_tick. */

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/* §16.2 app_handle_key — process one key.
 *
 * Returns true to keep running, false to quit. The cases mirror the
 * "Keys" section of the file header — one switch case per key.
 */
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

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
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
    s->star_idx = (s->star_idx + 1) % N_STARS;
    break;
  case 'T':
    s->star_idx = (s->star_idx + N_STARS - 1) % N_STARS;
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

  case 'm':
    s->moon_scale /= MOON_SCALE_STEP;
    if (s->moon_scale < MOON_SCALE_MIN)
      s->moon_scale = MOON_SCALE_MIN;
    break;
  case 'M':
    s->moon_scale *= MOON_SCALE_STEP;
    if (s->moon_scale > MOON_SCALE_MAX)
      s->moon_scale = MOON_SCALE_MAX;
    break;

  case 'n':
  case 'N':
    pattern_set(s, (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
    break;
  case 'p':
  case 'P':
    pattern_set(
        s, (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
    break;
  }
  return true;
}

/* §16.3 app_init / app_run / main.
 *
 * app_init zeros the App, sets up Scene + Screen, builds the
 * transmittance LUT (only place we call it — once, at startup).
 * app_run is the main loop: poll input, advance simulation in
 * fixed-step ticks, render once per loop iteration, sleep to pace.
 * main wires it all up.
 */

static void app_init(App *app) {
  memset(app, 0, sizeof *app);
  scene_init(&app->scene);
  screen_init(&app->screen);
  app->sim_fps = SIM_FPS_DEFAULT;
  app->running = 1;
  build_trans_lut();
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
