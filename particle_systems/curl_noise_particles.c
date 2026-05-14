/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_particles.c — particles advected by 2-D curl-noise field
 *
 * DEMO: A few hundred particles drift across the screen, but they
 *       don't fall, fly, or follow gravity — they're advected by a
 *       VELOCITY FIELD computed from the CURL of an fBm scalar
 *       noise field:  v = (∂N/∂y, −∂N/∂x).  By construction this
 *       field is DIVERGENCE-FREE — no point in space gains or
 *       loses particles, so the swarm never collapses to a sink
 *       and never bursts from a source. It just SWIRLS, like
 *       smoke in a draft. A short trail behind each particle
 *       traces the local streamline so the field's structure is
 *       visible at a glance.
 *
 *       Patterns:
 *         CALM         low-frequency noise → slow lazy eddies
 *         TURBULENT    multi-octave fBm → chaotic small swirls
 *         HURRICANE    low-freq noise + global rotation around centre
 *         WIND_TUNNEL  curl noise + horizontal wind bias (rightward drift)
 *
 *       This is the visual classic for fluid-feel particles WITHOUT
 *       solving Navier-Stokes. The Reynolds-paper "Curl Noise for
 *       Procedural Fluid Flow" (Bridson 2007) showed that taking the
 *       curl of any smooth scalar field gives an incompressible flow.
 *
 * Study alongside:
 *   fluid/navier_stokes.c   — actual fluid solver (more accurate,
 *                             much more code).
 *   particle_systems/vortex.c — also force-field driven, but with
 *                             a single central drain instead of a
 *                             distributed velocity field.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair velocity-magnitude ramp
 *   §4 noise     — Perlin/fBm scaffold
 *   §5 curl      — finite-difference curl from fBm
 *   §6 particle  — Particle struct + trail buffer
 *   §7 scene     — pool, advection tick, draw
 *   §8 screen    — ncurses init / draw / resize
 *   §9 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (new noise permutation, redistribute particles)
 *   n / N      next pattern   (CALM → TURBULENT → HURRICANE → WIND_TUNNEL)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *   d / D      next / previous debug overlay
 *              (normal / velocity / heatmap / divergence)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/curl_noise_particles.c \
 *       -o curl_noise_particles -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. CONCEPTS + MENTAL MODEL (below) — algorithm in plain English.
 *   2. GUIDED TUTORIAL (below) — 9 mini-lessons that build curl noise
 *      from scratch: lattice noise → fBm → curl identity → particle
 *      advection.
 *   3. §1 config — every constant you'd tweak.  pattern_params[] is
 *      the table that defines CALM/TURBULENT/HURRICANE/WIND_TUNNEL.
 *   4. §4 noise — Perlin 2-D scaffold + fBm octave loop.  Read this
 *      first to understand what the curl is being computed FROM.
 *   5. §5 curl — finite-difference curl + pattern-specific overlays.
 *      This is the heart of the algorithm; reads as ~40 lines.
 *   6. §6 particle — the actor struct (no logic, just storage).
 *   7. §7 scene — pool + tick + draw orchestration.
 *   8. §8 screen / §9 app — ncurses + fixed-step loop boilerplate.
 *
 * NAMING
 *   perm[]                256-element Perlin permutation table
 *   perlin2d(x, y)        a single 2-D Perlin sample ∈ roughly [-1, 1]
 *   fbm2(x, y, oct)       fractional Brownian motion = sum of octaves
 *   curl_velocity_at(...) (∂fBm/∂y, −∂fBm/∂x) via central differences
 *   Particle              one tracer point + a 4-position ring buffer
 *   trail[]               ring buffer: previous N positions for rendering
 *   PatternParams         CALM/TURBULENT/HURRICANE/WIND_TUNNEL knobs
 *   field_mag             scale applied to curl-derived velocity
 *   drift_x, drift_y      noise input "scroll" per second (time evolution)
 *
 * BACKGROUND ASSUMED
 *   • Perlin noise at a "lattice + interpolated gradient" level.
 *     If new to noise, the GUIDED TUTORIAL #2 walks the whole construction.
 *   • Partial derivatives + the central-difference formula
 *     (f(x+h) − f(x−h)) / 2h.
 *   • The vector calculus identity curl(∇φ) = 0 in some form — the
 *     reason curl noise produces divergence-free flow.
 *   • Object-pool pattern (fixed array + active flag, no malloc).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pure ADVECTION of particles through a velocity
 *                  field. The velocity field is the CURL of a smooth
 *                  scalar noise field — for 2-D this collapses to:
 *
 *                    N(x, y, t)   = fBm(x · k + t · drift_x,
 *                                       y · k + t · drift_y, octaves)
 *                    v_x(x, y, t) = +∂N/∂y
 *                    v_y(x, y, t) = −∂N/∂x
 *
 *                  Computed numerically with central differences
 *                  (h = 0.5 cells in physical space).
 *
 *                  Because curl·(∇N) = 0, the divergence of v is
 *                  identically zero — so the flow is incompressible.
 *                  Particles can never accumulate at a point or
 *                  vanish from one. This is what gives the
 *                  "fluid-feel" visual: no clumps, no holes, just
 *                  smooth flowing swirls.
 *
 *                  Each tick:
 *                    1. For every active particle: sample velocity
 *                       at its current position via 4 fBm evals
 *                       (centre-difference curl); advance position
 *                       by v · dt.
 *                    2. WRAP at screen edges (toroidal); particles
 *                       never die — they cycle forever.
 *                    3. Push current position into the particle's
 *                       trail ring buffer.
 *                    4. Render trail oldest-first (dim to bright)
 *                       with colour by current velocity magnitude:
 *                       slow particles dim, fast particles bright.
 *
 *                  Pattern variations layer extra terms on top of
 *                  the base curl-noise:
 *                    HURRICANE   adds a constant angular rotation
 *                                around the screen centre (broken
 *                                divergence-free, but visually adds
 *                                a dominant cyclone)
 *                    WIND_TUNNEL adds a constant +x velocity (also
 *                                breaks divergence-free, but
 *                                imperceptibly — wrap means
 *                                particles cycle right→left)
 *
 * Data-structure : Particle[MAX_PARTICLES] object pool with a
 *                  per-particle TRAIL ring buffer of last 4
 *                  positions. No malloc at runtime.
 *
 * Rendering      : ASCII only. Particle head: ramp slot picked from
 *                  velocity magnitude. Trail: 3 previous positions
 *                  rendered with progressively dimmer ramp slots.
 *
 * Performance    : Per-tick cost: N · 4 · OCTAVES Perlin lookups.
 *                  At N = 400, OCTAVES = 4: 6400 lookups/tick →
 *                  384 k/sec at 60 fps. Each Perlin lookup is ~30 ns
 *                  (8 hash + interpolation), so ~12 ms per second of
 *                  CPU. Comfortably real-time.
 *
 * References     :
 *   • Bridson, R., Houriham, J. & Nordenstam, M. (2007) — "Curl-Noise
 *     for Procedural Fluid Flow", *ACM TOG* 26(3) (SIGGRAPH).
 *     The paper that introduced this technique. It also shows how
 *     to extend to 3-D and how to enforce no-flow boundaries.
 *   • Wikipedia — [Stream function](https://en.wikipedia.org/wiki/Stream_function).
 *     The 2-D equivalent: any scalar function ψ defines a
 *     divergence-free velocity field via (∂ψ/∂y, −∂ψ/∂x). curl-
 *     noise is exactly this with ψ = fBm.
 *   • Iñigo Quílez — "Domain warping" / curl noise tutorials,
 *     https://iquilezles.org/articles/warp/.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Four flow patterns (CALM, TURBULENT, HURRICANE, WIND_TUNNEL) are
 * produced by a SINGLE generic curl-noise advection engine driven
 * by an array of PatternParams structs.  scene_tick and scene_draw
 * never branch on the pattern enum to decide HOW the physics works —
 * they read pattern_params[s->current_pattern] and compute the
 * velocity field accordingly.  Adding a new flow effect is a matter
 * of appending one row to pattern_params[]; no new code paths.
 *
 *
 * THE GENERIC ENGINE (pseudocode)
 * ───────────────────────────────
 *
 *   loop forever (each tick of dt seconds):
 *
 *     pp = pattern_params[scene.current_pattern]   # read inputs
 *
 *     # 1. SPAWN — top up particle pool toward target density
 *     while count(active particles) < pp.target_count:
 *         p = next_inactive_slot()
 *         p.x, p.y = uniform_in_screen(scene)
 *         p.life   = uniform(LIFE_MIN, LIFE_MAX)
 *         p.active = true
 *
 *     # 2. ADVECT — compute curl-noise velocity at each particle's
 *     #    position, integrate forward.  This is the hot loop.
 *     scene.noise_t += dt
 *     for p in active particles:
 *         # Sample fBm at p's position with TIME-DRIFTING input —
 *         # makes the field non-stationary so flow EVOLVES, not
 *         # just animates particles through a fixed pattern.
 *         nx = p.x · pp.fbm_freq + scene.noise_t · pp.time_drift_x
 *         ny = p.y · pp.fbm_freq + scene.noise_t · pp.time_drift_y
 *         psi    = fbm(nx, ny, octaves=pp.fbm_octaves)
 *         # CURL of scalar field ψ → divergence-free 2-D velocity
 *         # (Bridson et al. 2007):  v = (∂ψ/∂y, −∂ψ/∂x)
 *         vx, vy = (∂fbm/∂y · pp.field_mag, −∂fbm/∂x · pp.field_mag)
 *         # Optional ROTATION around screen centre (HURRICANE)
 *         if pp.global_rot != 0:
 *             (rx, ry) = (p.x − cx, p.y − cy)
 *             vx += −ry · pp.global_rot
 *             vy +=  rx · pp.global_rot
 *         # Optional WIND bias (WIND_TUNNEL)
 *         vx += pp.wind_bias
 *         # Explicit Euler advect
 *         p.x  += vx · dt
 *         p.y  += vy · dt
 *         p.age += dt
 *         p.speed = √(vx² + vy²)        # for render-time colour ramp
 *
 *     # 3. KILL — particles that age out, leave the canvas, or
 *     #    end up in a stagnant cell are recycled.
 *     for p in active particles:
 *         if p.age >= p.life or off-screen(p):  p.active = false
 *
 *     # 4. RENDER — particles painted by SPEED → ramp slot mapping.
 *     #    Theme ramp[8] is indexed by (p.speed / SPEED_MAX) · 7,
 *     #    so the velocity field's structure (slow zones, fast
 *     #    bands) reads as colour temperature.
 *     for p in active particles:
 *         slot = clamp(p.speed / SPEED_MAX, 0, 1) · 7
 *         paint(round(p.x), round(p.y), GLYPHS[slot], ramp[slot])
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_count    →  spawn-loop refill cap        (visual density)
 *   fbm_octaves     →  fbm() octave count           (Perlin/fBm depth;
 *                                                   1 = smooth sines,
 *                                                   4 = rich turbulence)
 *   fbm_freq        →  noise input scale (1/cells)  (small = wide cells,
 *                                                   large = small cells)
 *   field_mag       →  curl velocity multiplier     (compensates for the
 *                                                   k-magnitude scaling
 *                                                   of fBm gradients)
 *   time_drift_x    →  fBm input translation rate   (field evolves over
 *   time_drift_y                                    time, not stationary)
 *   global_rot      →  added angular velocity        (Goldstein Ch.4 — solid-
 *                      around screen centre          body rotation overlay;
 *                                                   HURRICANE only)
 *   wind_bias       →  added horizontal velocity    (uniform drift overlay;
 *                                                   WIND_TUNNEL only)
 *
 * Every other engine constant — MAX_PARTICLES, SPEED_MAX, LIFE_MIN,
 * LIFE_MAX, the GLYPHS ramp, the fbm() implementation itself — is a
 * GLOBAL tuning knob shared across all patterns.  Patterns differ
 * ONLY in the eight fields above.
 *
 *
 * ARCHITECTURAL REFERENCES
 * ────────────────────────
 *
 *   Bridson, R., Houriham, J. & Nordenstam, M. (2007)
 *     "Curl-Noise for Procedural Fluid Flow", ACM TOG 26(3)
 *     (SIGGRAPH).  The paper that introduced the technique whose
 *     parameterisation is exactly what PatternParams exposes.
 *     Bridson treats noise frequency, magnitude, and time-drift as
 *     independent knobs — the table here surfaces those knobs as
 *     pattern-selectable rows.
 *
 *   Reeves, W. T. (1983)
 *     "Particle Systems — A Technique for Modeling a Class of
 *     Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     §4 makes the explicit argument that ONE engine + a struct
 *     of physical constants per phenomenon is the right
 *     architecture for natural-particle simulations.  This file
 *     applies the idea to four flow regimes; the engine is the
 *     curl-noise advector, the struct is PatternParams.
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (CALM / TURBULENT / HURRICANE /
 *     WIND_TUNNEL) interchangeable behind a single interface (the
 *     engine reading PatternParams).  In procedural C the
 *     "interface" is the struct shape; "concrete strategies" are
 *     the rows of pattern_params[]; "selecting a strategy" is
 *     updating scene.current_pattern.
 *
 *   Acton, M. (2014)
 *     "Data-Oriented Design and C++" (CppCon 2014 keynote).
 *     Argues that variation between behaviours should be
 *     represented as DATA (struct fields) rather than as control
 *     flow (if/switch on type).  pattern_params[] is a compact
 *     data table; the engine has no per-pattern code paths and
 *     the cache footprint stays predictable across pattern
 *     switches — particularly important here where the hot loop
 *     evaluates fBm at every particle every tick.
 *
 *   Nystrom, R. (2014)
 *     "Game Programming Patterns" (Genever Benning).
 *     TYPE OBJECT chapter — PatternParams is a Type Object: one
 *     shared instance per "kind" of flow.  DATA LOCALITY chapter —
 *     Particle is a flat-laid-out POD swept linearly by the
 *     advector each tick, exactly the layout Nystrom recommends
 *     for hot inner loops.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't simulate fluid. Use noise to FAKE one. Take any smooth
 * 2-D scalar field N(x, y) — fBm works perfectly — and compute the
 * vector field (∂N/∂y, −∂N/∂x). That's a divergence-free velocity
 * field by mathematical identity (curl of gradient = 0, applied to
 * the orthogonal direction). Drop particles in it, advect them
 * forward in time. They flow like smoke.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a topographic map of hills and valleys. Now imagine
 * water on top — but instead of flowing downhill (gradient flow),
 * imagine it flowing along the CONTOUR LINES, with the higher
 * land on its left. That's curl flow. Streamlines never cross,
 * particles never converge, the field never "sources" anywhere.
 * Make the topography a smoothly-evolving fBm and you have an
 * eternal swirling wind.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SEED noise permutation (Perlin 256-element scrambled table).
 *
 *  2. SPAWN N particles at random positions across the screen.
 *
 *  3. EACH TICK, per particle (px, py):
 *     a. Sample fBm noise at 4 nearby points:
 *          N+x = fbm(px+h, py, t)
 *          N-x = fbm(px-h, py, t)
 *          N+y = fbm(px, py+h, t)
 *          N-y = fbm(px, py-h, t)
 *     b. Compute partial derivatives via central diff:
 *          ∂N/∂x ≈ (N+x − N-x) / (2h)
 *          ∂N/∂y ≈ (N+y − N-y) / (2h)
 *     c. Velocity:
 *          vx = +∂N/∂y · field_mag
 *          vy = −∂N/∂x · field_mag (with aspect correction)
 *     d. Pattern-specific additions:
 *          HURRICANE: add tangential rotation around centre
 *          WIND_TUNNEL: add constant +vx
 *     e. Advance: px += vx · dt; py += vy · dt
 *     f. Wrap at edges: px = (px + cols) mod cols; same for y
 *     g. Push (px, py) into trail ring buffer
 *
 *  4. RENDER trail (oldest dim → newest bright) + head, coloured
 *     by velocity magnitude.
 *
 *  5. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Curl of scalar (2-D stream function ψ):
 *    v = (∂ψ/∂y, -∂ψ/∂x)
 *    div v = ∂vx/∂x + ∂vy/∂y
 *          = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0
 *    → flow is INCOMPRESSIBLE
 *
 *  Time-shifted fBm (drives slow field evolution):
 *    ψ(x, y, t) = fbm(x·k + t·drift_x, y·k + t·drift_y, octaves)
 *
 *  Central-difference partial derivative:
 *    ∂N/∂x ≈ (N(x+h) − N(x−h)) / (2h)
 *
 *  Aspect-correct velocity (cells are 2× taller; sample noise in
 *  physical coords, convert v back to cell coords):
 *    py_phys = cy · ASPECT_Y
 *    vx_cell = vx_phys
 *    vy_cell = vy_phys / ASPECT_Y
 *
 *  Speed → ramp slot (visualisation):
 *    speed = √(vx_cell² + (vy_cell · ASPECT_Y)²)
 *    slot  = ⌊clamp(speed / SPEED_REF, 0, 1) · 7⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DIVERGENCE-FREENESS REQUIRES SMOOTH NOISE. If noise is C¹
 *    discontinuous (cubic-ramp value noise has C¹ kinks at lattice
 *    edges), the central-difference curl will have spurious sources
 *    and sinks at lattice boundaries. Use Perlin's quintic fade
 *    function `t³(6t² − 15t + 10)` (C² continuous) for smoother
 *    derivatives. We do.
 *
 *  • H TOO SMALL → noisy curl (numerical precision lost). H too
 *    LARGE → derivative too smoothed (loses local structure).
 *    H = 0.5 cells is a reasonable balance for fBm at this freq.
 *
 *  • OCTAVE COUNT vs PERFORMANCE. Each fBm sample costs O(octaves).
 *    Particle requires 4 fBm samples per tick. 4 octaves × 4 samples
 *    = 16 noise lookups per particle. With 400 particles, that's
 *    6.4k lookups/tick — comfortable.
 *
 *  • TIME EVOLUTION. We translate the noise input by `t · drift`
 *    each tick, so the field appears to scroll over time. Particles
 *    follow the new streamlines. For more "structural" change use
 *    3-D noise (sample at (x, y, t)); we use 2-D + drift for speed.
 *
 *  • WRAP. Particles wrap at the boundary because the noise field
 *    is conceptually infinite. Without wrap, particles would
 *    eventually all leak off the screen edges and the swarm would
 *    disappear.
 *
 *  • HURRICANE BREAKS DIV-FREE. Adding a global rotation around
 *    the centre is technically incompressible too (rotation
 *    preserves area), so we're fine. WIND_TUNNEL adds a constant
 *    vx; that's also incompressible (∂vx/∂x = 0).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Particles freeze in place. Resume: motion
 *    continues from where it stopped.
 *
 *  • CALM. Lazy big swirls — particles slowly orbit around invisible
 *    eddies. The noise structure is at low frequency so the eddies
 *    are SCREEN-SIZED.
 *
 *  • TURBULENT. Many small eddies. Particles get caught and ejected
 *    rapidly, the swarm forms transient streaks.
 *
 *  • HURRICANE. The whole field rotates around the screen centre
 *    while small eddies modulate the flow locally. Particles trace
 *    spirals as they orbit + drift.
 *
 *  • WIND_TUNNEL. Particles drift right (with eddy modulation).
 *    Watch a particle exit the right edge and re-enter from the
 *    left (toroidal wrap).
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a recognisably
 *    different fluid colour: TROPICAL (green/cyan), AURORA (cycling
 *    colours by speed), INFRARED (blue at slow → red at fast), etc.
 *
 *  • Reseed (`r`). The noise permutation regenerates and particles
 *    redistribute — a new flow field appears.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Nine mini-lessons.  Read them in order; each ends with the pseudocode
 * or formula that maps onto a real symbol below.
 *
 * ─── 1.  Why fake a fluid?  ─────────────────────────────────────────── *
 *   Real fluids obey Navier-Stokes — a coupled PDE system that needs
 *   a pressure solve every tick (fluid/navier_stokes.c does this in
 *   ~1000 lines).  The visual look you actually want — smooth swirling
 *   particles, no clumps, no holes — needs just ONE property:
 *
 *       div v = 0     ("the field is incompressible")
 *
 *   No conservation of momentum, no viscosity, no pressure projection.
 *   If you can generate any v(x, y, t) with div v ≡ 0, particles
 *   advected through it look fluid-like.  Curl noise is the cheap
 *   way to manufacture exactly that v.
 *
 * ─── 2.  Perlin noise in 60 seconds  ────────────────────────────────── *
 *   Perlin noise is a smooth random-ish 2-D function with values in
 *   roughly [−1, 1].  Construction:
 *
 *       1. Pick a 256-element random permutation perm[].
 *       2. For each integer lattice point (xi, yi), hash the index
 *          into perm[] to pick one of 8 unit gradient vectors.
 *       3. For a sample at (x, y):
 *            (xi, yi) = (floor x, floor y)
 *            (fx, fy) = (x − xi, y − yi)       // local fractional pos
 *            For each of the 4 corners (xi+0/1, yi+0/1):
 *              dot the corner's gradient with (fx − dx, fy − dy)
 *          Bilinearly interpolate the 4 dots using the QUINTIC fade
 *          curve t³(6t² − 15t + 10) for C² continuity.
 *
 *   The fade curve matters: without it, derivatives have visible kinks
 *   at lattice boundaries, and the curl picks them up as bogus sources.
 *
 *       perlin2d(x, y)  ∈  ≈ [−0.7, +0.7]
 *
 * ─── 3.  fBm — adding octaves for richer noise  ─────────────────────── *
 *   A single Perlin sample gives noise at ONE spatial frequency.  Real
 *   fluid eddies span many scales.  fBm (fractional Brownian motion)
 *   stacks multiple Perlin samples at doubling frequencies:
 *
 *       fbm(x, y, octaves) = Σᵢ  perlin(x · 2ⁱ, y · 2ⁱ) · 0.5ⁱ
 *                           i=0..octaves-1
 *
 *   Each octave doubles the frequency and HALVES the amplitude.  The
 *   sum has detail at every scale up to `octaves`.  At octaves=4 you
 *   see one big swirl with small eddies inside it — exactly the fluid
 *   look.  CALM uses 2 octaves; TURBULENT uses 4.
 *
 * ─── 4.  The curl identity in 2-D  ──────────────────────────────────── *
 *   The clever step.  In 2-D, define a velocity field FROM a scalar
 *   field ψ by:
 *
 *       v_x =  ∂ψ/∂y
 *       v_y = −∂ψ/∂x
 *
 *   This is called the STREAM FUNCTION construction.  Take its
 *   divergence:
 *
 *       div v = ∂v_x/∂x + ∂v_y/∂y
 *             = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x
 *             = 0          (mixed partials are equal — Schwarz's theorem)
 *
 *   So this v is divergence-free BY CONSTRUCTION, for any sufficiently
 *   smooth ψ.  Choose ψ = fBm and you get a fluid-like velocity field
 *   without solving any PDE.
 *
 * ─── 5.  Central-difference curl (numerical part)  ──────────────────── *
 *   We can't take symbolic derivatives of fBm (it's defined by a sum
 *   of hash lookups).  But fBm is smooth, so finite differences work:
 *
 *       ∂ψ/∂x ≈ (ψ(x+h, y) − ψ(x−h, y)) / (2h)        ← central diff
 *       ∂ψ/∂y ≈ (ψ(x, y+h) − ψ(x, y−h)) / (2h)
 *
 *   Cost: 4 fBm evaluations per particle per tick (one per offset).
 *   Choice of h matters:
 *       too small → numerical precision lost; curl becomes noisy
 *       too large → over-smoothed; loses local structure
 *   h = 0.5 cells in physical space hits the sweet spot for our
 *   pattern frequencies.
 *
 * ─── 6.  Divergence-free = "particles can't pile up"  ───────────────── *
 *   Why is div v = 0 the magic property?  Imagine an infinitesimal
 *   box around a point.  In Navier-Stokes terms, mass conservation
 *   says (flux out) − (flux in) = (rate of mass accumulating inside).
 *   For an incompressible fluid that rate is 0 → flux in = flux out.
 *
 *   For our particles: if the velocity field has a SINK (div < 0),
 *   particles flow toward that point and CLUMP.  If a SOURCE (div > 0),
 *   they fly away leaving a HOLE.  Either looks wrong.  div = 0
 *   everywhere means neither happens — the swarm stays evenly
 *   distributed regardless of how long you run the simulation.
 *
 * ─── 7.  Toroidal wrap (no death, no exit)  ─────────────────────────── *
 *   Particles drift continuously — what happens at the screen edge?
 *   Two bad options:
 *       (a) DESTROY at edge → swarm thins out and the screen empties.
 *       (b) REFLECT at edge → noise field still flows OUTWARD against
 *           the reflected motion → particles pile up at the boundary.
 *   The right answer: WRAP.  px = (px + cols) mod cols.  A particle
 *   exiting the right re-enters from the left.  Treat the screen as
 *   a torus.  The noise field is conceptually infinite anyway, so
 *   nothing visual breaks at the seam.
 *
 *       px = fmodf(px + cols, cols)        // safe for negative
 *       py = fmodf(py + rows, rows)
 *
 * ─── 8.  Trail ring buffer + speed-coloured ramp  ───────────────────── *
 *   A single moving dot is barely visible at 60 fps.  Each particle
 *   keeps a fixed-size ring buffer of its last 4 positions:
 *
 *       trail[0] = pos at t-3       ← OLDEST (dimmest)
 *       trail[1] = pos at t-2
 *       trail[2] = pos at t-1
 *       trail[3] = pos at t          ← HEAD (brightest)
 *
 *   Each tick: shift down, write current pos to trail[3].  Render
 *   trail[0..3] with increasing brightness so you SEE the streamline.
 *
 *   Colour comes from current SPEED (|v|), mapped to an 8-slot ramp:
 *       slow particles dim, fast particles bright.
 *   Speed varies across the field — fast where streamlines bunch up,
 *   slow where they spread.  The colour gradient makes the field's
 *   structure visible without overlaying arrows.
 *
 * ─── 9.  Pattern overlays — adding flavour without breaking div-free  ─ *
 *   The base curl noise is one flow type.  Three pattern overlays
 *   layer extra velocity on top:
 *
 *       HURRICANE:    v += ω × (p − centre)   // rotation, still div-free
 *                     (rotation preserves area — see vorticity theorems)
 *       WIND_TUNNEL:  v += (WIND, 0)            // uniform drift, ∂vₓ/∂x = 0
 *       TURBULENT:    no overlay; just more octaves on fBm
 *       CALM:         no overlay; just FEWER octaves
 *
 *   Both HURRICANE and WIND_TUNNEL preserve incompressibility because
 *   their added fields are also div-free (a uniform field has zero
 *   divergence; a rigid rotation has zero divergence).  The particle
 *   distribution stays even.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SPEED_MIN        =   1,
    SPEED_DEF        =   8,
    SPEED_MAX        =  64,

    MAX_PARTICLES    =  600,
    TRAIL_LEN        =    4,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = 8 speed-magnitude tints    */
    PAIR_SKY         =  11,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Curl finite-difference step (physical units). */
#define CURL_H            0.5f

/* Speed → ramp slot reference (cells/sec). With sqrt mapping below,
 * a particle at exactly SPEED_REF c/s renders in slot 7 (brightest).
 * Lower → slow particles look brighter. */
#define SPEED_REF        25.0f

/* Pattern enum. */
typedef enum {
    PATTERN_CALM        = 0,
    PATTERN_TURBULENT   = 1,
    PATTERN_HURRICANE   = 2,
    PATTERN_WIND_TUNNEL = 3,
    N_PATTERNS          = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_CALM:        return "CALM       ";
    case PATTERN_TURBULENT:   return "TURBULENT  ";
    case PATTERN_HURRICANE:   return "HURRICANE  ";
    case PATTERN_WIND_TUNNEL: return "WIND_TUNNEL";
    default:                  return "?          ";
    }
}

/*
 * PatternParams — controls the curl noise field:
 *
 *   target_count   : number of particles
 *   fbm_octaves    : noise octaves (1 = single sine, 4 = rich)
 *   fbm_freq       : base spatial frequency (1/cells)
 *   field_mag      : multiplier on curl velocity
 *   time_drift_x   : per-second translation of noise input in x
 *   time_drift_y   : per-second translation of noise input in y
 *   global_rot     : added angular velocity around centre (rad/sec)
 *   wind_bias      : added horizontal velocity (cells/sec)
 */
typedef struct {
    int   target_count;
    int   fbm_octaves;
    float fbm_freq;
    float field_mag;
    float time_drift_x;
    float time_drift_y;
    float global_rot;
    float wind_bias;
} PatternParams;

/*
 * field_mag values are large because the curl of fBm at frequency k
 * has magnitude ~k (Perlin gradients are roughly unit-magnitude per
 * unit input). At k = 0.025 (CALM) you need field_mag ≈ 800 to get
 * particles moving at meaningful screen speeds (~20 cells/sec). The
 * patterns with ROT or WIND additions could get away with smaller
 * field_mag because the additive term provides baseline motion.
 */
static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  cnt  oct   freq    mag     tdx    tdy    rot    wind */
    /* CALM         */ { 280,  2,  0.025f,  800.0f, 0.30f, 0.20f, 0.00f,  0.0f },
    /* TURBULENT    */ { 450,  4,  0.060f, 1100.0f, 0.80f, 0.50f, 0.00f,  0.0f },
    /* HURRICANE    */ { 380,  2,  0.020f,  500.0f, 0.30f, 0.20f, 0.40f,  0.0f },
    /* WIND_TUNNEL  */ { 380,  3,  0.045f,  600.0f, 0.50f, 0.30f, 0.00f, 18.0f },
};

/*
 * Themes — each is an 8-step gradient indexed by particle SPEED
 * (slow → fast). The visual encodes velocity magnitude as colour
 * temperature, making the flow field's structure visible at a glance.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* slow → fast */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name         ramp[0..7]  (slow → fast)                          sky */

    { "DEFAULT",   {  24,  31,  38,  45,  87, 117, 153, 195 },          234 },
    { "TROPICAL",  {  29,  35,  37,  44,  50,  86, 122, 159 },          234 },
    { "FIRE",      {  88, 124, 130, 166, 196, 208, 214, 226 },          233 },
    { "AURORA",    {  43,  79, 115, 121, 157, 195, 230, 231 },          234 },
    { "VIOLET",    {  53,  91, 134, 165, 207, 213, 219, 225 },          234 },
    { "ICE",       {  24,  31,  67, 110, 117, 153, 195, 231 },          235 },
    { "NEON",      { 199, 213, 207, 219, 225, 231, 195, 153 },          234 },
    { "COPPER",    { 130, 137, 173, 179, 215, 222, 229, 230 },          234 },
    { "MONO",      { 240, 243, 245, 247, 249, 251, 253, 255 },          232 },
    { "INFRARED",  {  17,  21,  39,  46, 154, 226, 208, 196 },          233 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_SKY, t->sky, -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_CYAN, -1);
        init_pair(PAIR_SKY, COLOR_BLACK, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §3b debug overlays — turn the simulation into a learning instrument   */
/* ===================================================================== */

/*
 * Four rendering modes cycle with d/D.  Each makes one concept visible:
 *
 *   DBG_NORMAL      production view — trails + heads, speed-coloured.
 *
 *   DBG_VELOCITY    each particle's head replaced with an ASCII arrow
 *                   showing the local v direction.  You can SEE the
 *                   streamlines: arrows align along curves, not noise.
 *
 *   DBG_HEATMAP     full-screen sweep — sample fBm(x, y, t) at every
 *                   cell, shade by value.  Lets you see the scalar
 *                   field ψ that the curl is computed FROM.  Bright
 *                   ridges = high ψ; dark valleys = low ψ.  Particles
 *                   follow the contours (perpendicular to ∇ψ).
 *
 *   DBG_DIVERGENCE  full-screen sweep — compute div v at every cell
 *                   via 4 curl_velocity_at samples.  The mathematical
 *                   identity guarantees div v ≡ 0 ANALYTICALLY; here
 *                   we see only the NUMERICAL truncation error of the
 *                   central-difference scheme.  Most cells should be
 *                   nearly empty (div ≈ 0).  Bright spots = where the
 *                   finite-difference approximation hits a high-curvature
 *                   region of the fBm.  Pedagogical use: visually
 *                   verify the curl identity at runtime.
 *
 * The global lets us thread the mode into scene_draw without changing
 * every signature.  Same pattern as burst.c — file-scope state, no
 * physics dependency, never persists across runs.
 */
typedef enum {
    DBG_NORMAL     = 0,
    DBG_VELOCITY   = 1,
    DBG_HEATMAP    = 2,
    DBG_DIVERGENCE = 3,
    DBG_COUNT      = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal", "velocity", "heatmap", "divergence",
};

static DebugMode g_debug = DBG_NORMAL;

/*
 * dir_char — velocity vector → 8-octant ASCII arrow.
 *
 * Eight octants centred on cardinals; positive y is DOWN in screen
 * coords so the diagonals look rotated relative to math convention.
 * Returns '.' for zero velocity.
 */
static char dir_char(float vx, float vy)
{
    static const char k_dirs[8] = { '>', '\\', 'v', '/', '<', '\\', '^', '/' };
    if (vx == 0.0f && vy == 0.0f) return '.';
    float a = atan2f(vy, vx);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int idx = (int)((a + (float)M_PI / 8.0f) / ((float)M_PI / 4.0f)) & 7;
    return k_dirs[idx];
}

/* ===================================================================== */
/* §4  noise — Perlin scaffold (copied inline per self-contained-file)   */
/* ===================================================================== */

/*
 * §4 PREAMBLE — building the scalar field ψ
 * ──────────────────────────────────────────
 *
 * Curl noise needs ψ(x, y): a smooth, randomly-textured 2-D scalar
 * field.  This section builds ψ from scratch using two layers:
 *
 *   perlin2d(x, y)         ONE octave — one frequency, [-1..+1] range
 *   fbm2(x, y, octaves)    SUM of octaves at doubling frequencies
 *                          (1×, 2×, 4×, 8× ...) and halving amplitudes
 *
 * The Perlin layer is the textbook implementation: 256-entry
 * permutation table + hashed gradient vectors + quintic fade.  Why
 * inline it instead of #include-ing a noise.h?  Per CLAUDE.md project
 * rule: every .c file is one program — no shared headers, no module
 * dependencies.  The cost is ~50 lines of duplicated noise scaffold
 * in every file that uses Perlin.  The benefit is that a learner
 * reading this file sees the FULL algorithm without chasing pointers.
 *
 * The QUINTIC fade function t³(6t² − 15t + 10) is the C²-continuous
 * smoothstep.  This file uses curl-noise: it differentiates the field
 * twice (once for ∂ψ/∂x, once for ∂ψ/∂y).  If the fade were only
 * C¹-continuous (cubic), those derivatives would have visible kinks
 * at lattice boundaries, and the curl would show spurious sources
 * exactly where you don't want them.  See GUIDED TUTORIAL #2.
 */

static uint8_t perm[512];

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade_q(float t) { return t*t*t*(t*(t*6.f-15.f)+10.f); }
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * perlin2d — one Perlin noise sample at (x, y).
 *
 * PURPOSE
 *   The atom of the noise field.  Returns a smoothly-interpolated
 *   value in roughly [-1, +1] that has C²-smooth derivatives everywhere
 *   (so the curl this file computes from fBm of it is well-defined).
 *
 * PSEUDOCODE
 *   (X, Y)       = (floor x, floor y)   AND  with 255 (perm-table size)
 *   (fx, fy)     = (x − X, y − Y)        // local fractional in [0, 1)
 *   (u, v)       = quintic_fade(fx, fy)
 *   At each of 4 corners (X+dx, Y+dy), dx,dy ∈ {0, 1}:
 *     g_corner   = grad2(perm-hashed-index, fx − dx, fy − dy)
 *   noise       = bilerp(g_00, g_10, g_01, g_11, u, v)
 *   return noise
 *
 * MENTAL MODEL
 *   At every integer lattice point a hidden gradient vector points in
 *   some direction.  At a sample point (x, y), the contribution from
 *   each lattice corner is "dot product of that corner's gradient
 *   with the displacement TO our sample".  Bilinearly weight the
 *   four corner contributions using the quintic-smoothed fractional
 *   position.  The result is locally smooth (no jumps), zero exactly
 *   at lattice corners (because the displacement is zero there), and
 *   randomly textured between.
 *
 * INPUTS / OUTPUTS
 *   x, y    → sample coordinates (any float, lattice spacing is 1.0)
 *   return  → noise value in approximately [-1, +1]; mean ≈ 0
 *
 * UNITS
 *   The "lattice unit" — one unit of x or y corresponds to one
 *   perm[] cell.  fbm2 scales x and y by `freq` to put more or fewer
 *   lattice cells per pixel.
 *
 * WHY IT EXISTS (vs using libnoise or an open_simplex.h)
 *   Self-contained-file rule.  Also pedagogical: a reader sees the
 *   ENTIRE algorithm in 14 lines.  No black-box noise function.
 */
static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/*
 * fbm2 — fractional Brownian motion = sum of Perlin octaves.
 *
 * PURPOSE
 *   Stack N copies of perlin2d at doubling frequencies and halving
 *   amplitudes, then normalise.  The result has interesting structure
 *   at multiple scales — one big swirl with smaller eddies inside it
 *   inside it inside it.  Real turbulence has the same scaling
 *   (Kolmogorov's −5/3 law); fBm approximates it cheaply.
 *
 * PSEUDOCODE
 *   total, amp, freq, max_amp = 0, 1, 1, 0
 *   for o in 0..octaves-1:
 *       total   += amp · perlin2d(x · freq, y · freq)
 *       max_amp += amp
 *       amp     *= 0.5
 *       freq    *= 2.0
 *   return (total / max_amp) · 0.5    // normalise to roughly [-0.5, +0.5]
 *
 * MENTAL MODEL
 *   Octave 0 is the BIG eddies (one full wavelength across the screen).
 *   Octave 1 adds eddies HALF the size at HALF the amplitude.
 *   Octave 2 quarter-size, quarter-amplitude.  Etc.
 *   By octave 4, the smallest features are sub-cell — adding more
 *   octaves only adds noise that the screen can't display, so we cap
 *   at 4–5 for performance + visible benefit.
 *
 *   CALM uses 2 octaves: clean broad swirls.
 *   TURBULENT uses 4: complex small-scale chop.  The visual difference
 *   is dramatic from a one-int change.
 *
 * INPUTS / OUTPUTS
 *   x, y     → sample coordinates (caller scales by `freq` separately)
 *   octaves  → octave count, typically 2..5
 *   return   → noise value, approximately [-0.5, +0.5]
 *
 * WHY IT EXISTS (vs computing curl from a single perlin2d call)
 *   Single-octave noise has only one spatial frequency.  The curl
 *   field then has a uniform feature size — every eddy is the same
 *   size.  Visually unsatisfying.  fBm gives the multi-scale eddies
 *   that read as "fluid" instead of "abstract grid pattern".
 */
/* fBm with N octaves. Returns roughly [-0.5, 0.5]. */
static float fbm2(float x, float y, int octaves)
{
    float total = 0, amp = 1, freq = 1, max_amp = 0;
    for (int o = 0; o < octaves; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.0f;
}

/* ===================================================================== */
/* §5  curl — finite-difference curl of fBm                              */
/* ===================================================================== */

/*
 * §5 PREAMBLE — turning the scalar field into a velocity field
 * ─────────────────────────────────────────────────────────────
 *
 * §4 builds ψ(x, y, t) = fBm of a time-shifted lattice.  §5 turns it
 * into v(x, y, t) by taking the 2-D curl:
 *
 *      v_x =  ∂ψ/∂y      v_y = −∂ψ/∂x
 *
 * The whole section is one function — curl_velocity_at — because
 * curl is one operation.  Pattern-specific overlays (HURRICANE
 * rotation, WIND_TUNNEL bias) are added INSIDE this function rather
 * than in a separate step, because they're algebraically just extra
 * velocity terms that compose with the curl.
 *
 * Coordinate-system note: this section works in PHYSICAL space (cells
 * scaled by ASPECT_Y on the y-axis) so the fBm sees square units.
 * Without that, the curl would be biased — y derivatives would be
 * "denser" than x derivatives by a factor of 2 (CELL_H / CELL_W).
 * The caller (scene_tick) converts cell-space ↔ physical space at
 * the boundary, NOT inside the curl function.
 */

/*
 * curl_velocity_at — sample the curl-noise velocity at one position.
 *
 * PURPOSE
 *   The algorithm's HEART.  Given a position (px_phys, py_phys) and
 *   the current simulation time t, return the velocity v that an
 *   advected particle should follow.  v is the 2-D curl of a
 *   time-drifting fBm, plus pattern-specific overlays for the
 *   HURRICANE and WIND_TUNNEL variants.
 *
 * PSEUDOCODE
 *   k, tx, ty = pp.fbm_freq, t·pp.time_drift_x, t·pp.time_drift_y
 *
 *   // Four fBm samples — east, west, north, south of (px, py).
 *   n_xp = fbm2((px + h)·k + tx, py·k + ty, octaves)
 *   n_xm = fbm2((px − h)·k + tx, py·k + ty, octaves)
 *   n_yp = fbm2(px·k + tx, (py + h)·k + ty, octaves)
 *   n_ym = fbm2(px·k + tx, (py − h)·k + ty, octaves)
 *
 *   // Central-difference partial derivatives.
 *   ∂ψ/∂x = (n_xp − n_xm) / (2h)
 *   ∂ψ/∂y = (n_yp − n_ym) / (2h)
 *
 *   // The curl identity: (∂ψ/∂y, −∂ψ/∂x).
 *   v_x =  ∂ψ/∂y · field_mag
 *   v_y = −∂ψ/∂x · field_mag
 *
 *   // Pattern overlays.
 *   if pp.global_rot > 0:      v += ω × (p − screen_centre)
 *   if pp.wind_bias != 0:      v_x += pp.wind_bias
 *
 * MENTAL MODEL
 *   Four fBm taps + two subtractions + two divides = full curl.
 *   That is THE WHOLE PHYSICS — no Navier-Stokes, no pressure solve.
 *   The cost is 4 · octaves Perlin lookups per particle per tick;
 *   at N=400 particles × 4 octaves × 4 taps = 6400 Perlin lookups
 *   per tick, ~12 ms/sec at 60 Hz on a modern CPU.
 *
 *   Time drift: by ADDING t·drift to the noise input, we shift the
 *   lattice each frame.  Particles re-sample at the same screen
 *   position but find a slightly-different field.  The streamlines
 *   morph slowly; the swarm follows.  Without drift the field is
 *   STATIC — particles flow forever along the same fixed streamlines,
 *   which looks dead after a few seconds.
 *
 *   HURRICANE overlay (`v += ω × r`) adds a rigid rotation around the
 *   centre.  Rotation is div-free (rotating a region preserves area),
 *   so it composes safely with the curl-noise base without breaking
 *   incompressibility.  Same for WIND_TUNNEL's uniform `vx` add.
 *
 * INPUTS / OUTPUTS
 *   px_phys, py_phys  → position in PHYSICAL space (cells × aspect)
 *   t                 → simulation time, seconds (for drift)
 *   pp                → which PatternParams entry to read knobs from
 *   screen_cx/cy_phys → centre of the playable rectangle (for HURRICANE)
 *   *out_vx_phys      ← curl-derived x velocity in physical units
 *   *out_vy_phys      ← curl-derived y velocity
 *
 * UNITS
 *   px_phys, py_phys:  cells × ASPECT_Y (so units are SQUARE)
 *   v_phys:            cells × ASPECT_Y / second
 *   caller divides v_phys.y by ASPECT_Y before applying to a cell
 *   coordinate
 *
 * WHY IT EXISTS (vs inlining in scene_tick)
 *   Called TWICE per particle per frame: once for advection (in
 *   scene_tick), once for the speed→colour mapping (in scene_draw).
 *   Inlining would duplicate the 4-tap fBm sampling.  Splitting
 *   keeps the algorithm in one place with one set of constants.
 */
static void curl_velocity_at(float px_phys, float py_phys, float t,
                             const PatternParams *pp,
                             float screen_cx_phys, float screen_cy_phys,
                             float *out_vx_phys, float *out_vy_phys)
{
    float k = pp->fbm_freq;
    float tx = t * pp->time_drift_x;
    float ty = t * pp->time_drift_y;

    float n_xp = fbm2((px_phys + CURL_H) * k + tx, py_phys * k + ty,
                     pp->fbm_octaves);
    float n_xm = fbm2((px_phys - CURL_H) * k + tx, py_phys * k + ty,
                     pp->fbm_octaves);
    float n_yp = fbm2(px_phys * k + tx, (py_phys + CURL_H) * k + ty,
                     pp->fbm_octaves);
    float n_ym = fbm2(px_phys * k + tx, (py_phys - CURL_H) * k + ty,
                     pp->fbm_octaves);

    float dN_dx = (n_xp - n_xm) / (2.0f * CURL_H);
    float dN_dy = (n_yp - n_ym) / (2.0f * CURL_H);

    float vx =  dN_dy * pp->field_mag;
    float vy = -dN_dx * pp->field_mag;

    /* HURRICANE: global rotation around screen centre. */
    if (pp->global_rot > 0.0f) {
        float dx = px_phys - screen_cx_phys;
        float dy = py_phys - screen_cy_phys;
        vx += -dy * pp->global_rot;
        vy +=  dx * pp->global_rot;
    }
    /* WIND_TUNNEL: constant +x bias. */
    if (pp->wind_bias != 0.0f) {
        vx += pp->wind_bias;
    }

    *out_vx_phys = vx;
    *out_vy_phys = vy;
}

/* ===================================================================== */
/* §6  particle                                                           */
/* ===================================================================== */

typedef struct {
    float px, py;        /* current cell-space position                 */
    float trail_x[TRAIL_LEN];
    float trail_y[TRAIL_LEN];
    int   trail_head;
    int   trail_count;
    bool  active;
} Particle;

/* Cheap LCG. */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §7  scene — pool, advection tick, draw                                */
/* ===================================================================== */

/*
 * §7 PREAMBLE — orchestrator
 * ───────────────────────────
 *
 * The Scene owns the particle pool, the time accumulator (drives noise
 * drift), the active pattern + theme, and the RNG.  It exposes the
 * two per-frame operations:
 *
 *   scene_tick   advance all particles by dt — read field, integrate,
 *                wrap, push trail.  Pure state mutation; no I/O.
 *   scene_draw   render trail + head — pure rendering; const Scene *.
 *
 * Plus housekeeping: clear, spawn-one, populate-to-target (called
 * after pattern change to grow/shrink the pool), init, resize, reseed.
 *
 * Particles never DIE in this file (no off-screen kill, no lifetime).
 * They WRAP at the toroidal screen edges and live forever.  Pool size
 * changes only on pattern change (different patterns have different
 * target counts) or 'r' reseed.
 */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    float     time_accum;       /* drives noise time evolution         */

    Particle  particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s)
{
    for (int i = 0; i < MAX_PARTICLES; i++) s->particles[i].active = false;
}

static void scene_spawn_particle(Scene *s, int idx)
{
    Particle *p = &s->particles[idx];
    p->px = lcg_unit(&s->rng) * (float)s->cols;
    p->py = lcg_unit(&s->rng) * (float)(s->rows - 1);
    p->trail_head  = 0;
    p->trail_count = 0;
    /* Pre-fill trail with current position. */
    for (int k = 0; k < TRAIL_LEN; k++) {
        p->trail_x[k] = p->px;
        p->trail_y[k] = p->py;
    }
    p->active = true;
}

static void scene_populate_to_target(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_count;
    if (target > MAX_PARTICLES) target = MAX_PARTICLES;
    int active = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) active++;
    /* Spawn shortfall in first inactive slots. */
    int spawned = 0;
    for (int i = 0; i < MAX_PARTICLES && spawned < target - active; i++) {
        if (!s->particles[i].active) {
            scene_spawn_particle(s, i);
            spawned++;
        }
    }
    /* Deactivate surplus from end of pool. */
    if (active > target) {
        int to_remove = active - target;
        for (int i = MAX_PARTICLES - 1; i >= 0 && to_remove > 0; i--) {
            if (s->particles[i].active) {
                s->particles[i].active = false;
                to_remove--;
            }
        }
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_CALM;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time_accum      = 0.0f;
    perm_shuffle((int)(s->rng & 0xFFFF));
    scene_clear_particles(s);
    scene_populate_to_target(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    perm_shuffle((int)(s->rng & 0xFFFF));
    scene_clear_particles(s);
    scene_populate_to_target(s);
}

/*
 * scene_tick — advance every active particle by dt.
 *
 * PURPOSE
 *   The per-frame physics loop.  For each particle: sample the
 *   curl-noise velocity, integrate position, wrap at screen edges,
 *   push the new position onto the trail ring buffer.  Trivial
 *   structure — all the complexity lives in curl_velocity_at.
 *
 * PSEUDOCODE
 *   if paused:  return
 *   dt *= speed_mul                    // user '+/-' speed dial
 *   time_accum += dt                    // drives noise time drift
 *
 *   for each active particle p:
 *     px_phys = p.px                     // cell → physical (x: no change)
 *     py_phys = p.py × ASPECT_Y          // cell → physical (y: square it up)
 *     curl_velocity_at(px_phys, py_phys, time_accum, pp, ...)
 *     vx_cell = vx_phys                  // physical → cell (x: no change)
 *     vy_cell = vy_phys / ASPECT_Y       // physical → cell (y)
 *     p.px += vx_cell · dt
 *     p.py += vy_cell · dt
 *     wrap p.px to [0, cols)             // toroidal
 *     wrap p.py to [0, rows_eff)
 *     trail.push(p.px, p.py)             // ring-buffer write
 *
 * MENTAL MODEL
 *   Two coordinate systems converge here:
 *       CELL space    p.px, p.py — what the renderer wants
 *       PHYSICAL space  what curl_velocity_at wants (square units)
 *   The conversion happens at the BOUNDARY of this function (px_phys
 *   in, vx_cell out) so the curl algorithm doesn't need to know
 *   about cell aspect ratio, and the renderer doesn't need to know
 *   about physical units.
 *
 *   The toroidal wrap uses `while` rather than `fmodf` because at
 *   sane dt the position rarely overshoots by more than one wrap,
 *   so 1–2 iterations cover the common case without the fmodf call
 *   overhead.  In pathological scenarios (huge dt, paused-then-fast-
 *   forward) the while loop is still O(overshoot_count) — never
 *   infinite.
 *
 * INPUTS / OUTPUTS
 *   s    ← Scene (mutated: time_accum + every active particle)
 *   dt   → frame time in seconds (already speed-multiplied later)
 *
 * UNITS
 *   dt:           seconds (after speed_mul scaling)
 *   ASPECT_Y:     dimensionless ratio (typically 2.0 for terminal cells)
 *
 * WHY IT EXISTS (vs inlining in app's main loop)
 *   Same separation principle as comet.c and constellation.c: app's
 *   main loop is "read input, tick, draw, sleep" at orchestrator
 *   level.  All physics + state mutation lives behind scene_tick().
 *   A future test harness or batch-render tool can drive the same
 *   Scene without dragging in ncurses or signal handlers.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->time_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    int rows_eff = s->rows - 1;
    float screen_cx_phys = (float)s->cols * 0.5f;
    float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Convert cell position → physical position for noise sampling. */
        float px_phys = p->px;
        float py_phys = p->py * ASPECT_Y;

        float vx_phys, vy_phys;
        curl_velocity_at(px_phys, py_phys, s->time_accum, pp,
                         screen_cx_phys, screen_cy_phys,
                         &vx_phys, &vy_phys);

        /* Convert physical velocity back to cell-space. */
        float vx_cell = vx_phys;
        float vy_cell = vy_phys / ASPECT_Y;

        p->px += vx_cell * dt;
        p->py += vy_cell * dt;

        /* Toroidal wrap. */
        while (p->px < 0.0f)              p->px += (float)s->cols;
        while (p->px >= (float)s->cols)   p->px -= (float)s->cols;
        while (p->py < 0.0f)              p->py += (float)rows_eff;
        while (p->py >= (float)rows_eff)  p->py -= (float)rows_eff;

        /* Push trail. */
        p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
        p->trail_x[p->trail_head] = p->px;
        p->trail_y[p->trail_head] = p->py;
        if (p->trail_count < TRAIL_LEN) p->trail_count++;
    }
}

/*
 * scene_draw — paint trail + head for every active particle.
 *
 * PURPOSE
 *   Render one frame.  For each particle: compute current speed
 *   (one curl_velocity_at call), map speed → 8-slot ramp index,
 *   draw the trail oldest-first with dimming ramp slots, then the
 *   head on top with the brightest slot.  No state mutation.
 *
 * PSEUDOCODE
 *   for each active particle:
 *     v       = curl_velocity_at(p.px, p.py, time_accum, pp)
 *     speed   = |v|
 *     f       = clamp(speed / SPEED_REF, 0, 1)
 *     f       = sqrt(f)                          // pseudo-gamma boost
 *     head_slot = clamp(f · 8, 1, 7)             // never 0 — must be visible
 *     for k in 0..trail_count-1:
 *       (tx, ty)  = trail[(head + 1 + k) mod TRAIL_LEN]
 *       slot      = max(0, head_slot − (n − 1 − k))   // older = dimmer
 *       glyph     = pick by slot ('.' / '+' / '*')
 *       attr      = (head_slot ≥ 6) ? A_BOLD : (head_slot ≤ 1) ? A_DIM : 0
 *       mvaddch(ty, tx, glyph) in COLOR_PAIR(slot)
 *     // last iter is the head — overdrawn on top, brightest slot
 *
 * MENTAL MODEL
 *   Three rendering decisions per particle:
 *     1. COLOUR slot — driven by current SPEED (so faster particles
 *        glow brighter; you SEE field magnitude as colour intensity).
 *     2. TRAIL fade — slot drops by 1 per step back into the past
 *        (so the streamline trails off naturally).
 *     3. GLYPH — picked from current slot ('.' faint, '+' medium,
 *        '*' bright).  Three glyphs is enough granularity for ASCII.
 *
 *   The sqrt-of-normalised-speed is a CHEAP gamma boost: it pushes
 *   slow particles into higher slots so they're still visible.
 *   Without it, most of the swarm clusters at speed ≈ 0.2·SPEED_REF
 *   and renders in slot 1, which the theme tints make almost
 *   invisible.  With sqrt, that same speed maps to slot 3 — visible.
 *
 *   We call curl_velocity_at AGAIN here (scene_tick already did).
 *   Caching the speed at tick time would save one fBm batch, but
 *   would couple two layers: tick would need to know about render's
 *   colour scheme.  At the current cost (4·octaves Perlin samples ×
 *   N particles) the duplication is invisible — keep the layers
 *   separate.
 *
 * INPUTS / OUTPUTS
 *   s    → Scene (const — never mutated)
 *
 * UNITS
 *   speed:      same units as curl_velocity_at returns (physical /s)
 *   head_slot:  integer 1..7 (index into theme's 8-slot ramp)
 *
 * WHY IT EXISTS (vs scattering draws across scene_tick)
 *   Tick = STATE; draw = PIXELS.  Splitting them means rendering
 *   tweaks (new glyph table, debug overlay, motion blur) never
 *   touch physics, and physics changes (new pattern, different field
 *   model) never touch rendering.  This is the canonical comet.c /
 *   constellation.c / aafire_port.c split applied here too.
 */
/*
 * bg_heatmap_draw — DBG_HEATMAP: shade every cell by fBm(x, y, t).
 *
 * Sweep every screen cell, sample the scalar field ψ at the matching
 * physical position, map ψ ∈ ≈ [−0.5, +0.5] → 0..7 ramp slot, paint
 * with a density-suggesting glyph.  Cost is cols·rows·octaves Perlin
 * lookups per frame — ~50 k at 200×60 with 4 octaves, well within
 * budget at 60 Hz.
 *
 * Pedagogical view: lets you see the SCALAR FIELD that the curl is
 * derived FROM.  Particles flow along level sets (contour lines) of
 * this field.
 */
static void bg_heatmap_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float k  = pp->fbm_freq;
    float tx = s->time_accum * pp->time_drift_x;
    float ty = s->time_accum * pp->time_drift_y;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            float px_phys = (float)col;
            float py_phys = (float)row * ASPECT_Y;
            float v = fbm2(px_phys * k + tx, py_phys * k + ty,
                           pp->fbm_octaves);
            float f = v + 0.5f;                      /* map [-0.5,+0.5] → [0,1] */
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            int slot = (int)(f * 7.999f);

            char glyph = (slot >= 6) ? '#' :
                         (slot >= 4) ? '+' :
                         (slot >= 2) ? '.' : ' ';
            if (glyph == ' ') continue;
            int pair = PAIR_RAMP_BASE + slot;
            attron(COLOR_PAIR(pair));
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair));
        }
    }
}

/*
 * bg_divergence_draw — DBG_DIVERGENCE: shade every cell by |div v|.
 *
 * For each cell, compute div v via 4 curl_velocity_at samples and
 * map magnitude → ramp slot.  The curl identity guarantees div v ≡ 0
 * ANALYTICALLY; the only non-zero values here come from the central-
 * difference truncation error of the curl scheme — so most cells
 * should be empty (slot 0).
 *
 * Sampled every 2 cells (4× cost reduction) since 4 curl evaluations
 * per cell = 16 fBm samples = ~16 × octaves Perlin lookups per cell.
 * At 200×60 / 2² × 16 × 4 octaves = 192 000 lookups per frame.
 *
 * Pedagogical use: visually verify that curl-of-fBm really is
 * divergence-free, and SEE where the numerical curl breaks down
 * (high-curvature regions of the noise field).
 */
static void bg_divergence_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float screen_cx_phys = (float)s->cols * 0.5f;
    float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

    for (int row = 0; row < rows_eff; row += 2) {
        for (int col = 0; col < s->cols; col += 2) {
            float px_phys = (float)col;
            float py_phys = (float)row * ASPECT_Y;

            float vx_xp, vy_xp, vx_xm, vy_xm, vx_yp, vy_yp, vx_ym, vy_ym;
            curl_velocity_at(px_phys + CURL_H, py_phys, s->time_accum, pp,
                             screen_cx_phys, screen_cy_phys, &vx_xp, &vy_xp);
            curl_velocity_at(px_phys - CURL_H, py_phys, s->time_accum, pp,
                             screen_cx_phys, screen_cy_phys, &vx_xm, &vy_xm);
            curl_velocity_at(px_phys, py_phys + CURL_H, s->time_accum, pp,
                             screen_cx_phys, screen_cy_phys, &vx_yp, &vy_yp);
            curl_velocity_at(px_phys, py_phys - CURL_H, s->time_accum, pp,
                             screen_cx_phys, screen_cy_phys, &vx_ym, &vy_ym);

            float div = (vx_xp - vx_xm) / (2.0f * CURL_H)
                      + (vy_yp - vy_ym) / (2.0f * CURL_H);
            float adiv = fabsf(div);

            /* Threshold tuned so the analytic curl shows ≈ empty;
             * non-zero pixels are central-diff truncation error. */
            float f = adiv / 0.02f;
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            int slot = (int)(f * 7.999f);
            if (slot == 0) continue;

            char glyph = (slot >= 6) ? '#' : (slot >= 3) ? '+' : '.';
            int pair = PAIR_RAMP_BASE + slot;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;
    const PatternParams *pp = &pattern_params[s->current_pattern];

    float screen_cx_phys = (float)s->cols * 0.5f;
    float screen_cy_phys = (float)rows_eff * 0.5f * ASPECT_Y;

    /*
     * Debug-mode background overlay (drawn FIRST, under particles).
     * HEATMAP shows ψ; DIVERGENCE shows the curl identity's residual.
     * Particles still render on top so you can see them flowing
     * relative to the field they're advected by.
     */
    if (g_debug == DBG_HEATMAP)    bg_heatmap_draw(s);
    if (g_debug == DBG_DIVERGENCE) bg_divergence_draw(s);

    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Compute current speed for colour slot. */
        float vx_phys, vy_phys;
        curl_velocity_at(p->px, p->py * ASPECT_Y, s->time_accum, pp,
                         screen_cx_phys, screen_cy_phys,
                         &vx_phys, &vy_phys);
        float speed = sqrtf(vx_phys * vx_phys + vy_phys * vy_phys);
        /* sqrt mapping pushes low speeds into higher ramp slots
         * (a particle at 25% of SPEED_REF gets slot 50% of brightest
         * instead of 25%). Otherwise most particles cluster in slot 0
         * and disappear into the dark theme tints. */
        float f = speed / SPEED_REF;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        f = sqrtf(f);
        int head_slot = (int)(f * 7.999f);
        if (head_slot < 1) head_slot = 1;     /* always visible */
        if (head_slot > 7) head_slot = 7;

        /* Trail (oldest → newest); head overdraws last. */
        int n = p->trail_count;
        for (int k = 0; k < n; k++) {
            int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
            float tx = p->trail_x[idx];
            float ty = p->trail_y[idx];
            int ix = (int)(tx + 0.5f);
            int iy = (int)(ty + 0.5f);
            if (ix < 0 || ix >= s->cols) continue;
            if (iy < 0 || iy >= rows_eff) continue;

            int slot = head_slot - (n - 1 - k);
            if (slot < 0) slot = 0;
            char glyph;
            int  attr;
            if (k == n - 1) {
                /* Head — in DBG_VELOCITY mode replace glyph with an
                 * arrow showing v's direction. */
                if (g_debug == DBG_VELOCITY)
                    glyph = dir_char(vx_phys, vy_phys);
                else
                    glyph = (head_slot >= 5) ? '*'
                          : (head_slot >= 2) ? '+'
                          :                    '.';
                attr  = (head_slot >= 6) ? A_BOLD
                      : (head_slot <= 1) ? A_DIM
                      :                    A_NORMAL;
            } else {
                glyph = (slot >= 4) ? '+' : (slot >= 1) ? '.' : '`';
                attr  = (slot <= 1) ? A_DIM : A_NORMAL;
            }
            int pair = PAIR_RAMP_BASE + slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static int scene_active_count(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (s->particles[i].active) n++;
    return n;
}

/*
 * HUD layout per CLAUDE.md:
 *   row 0          — fps + theme + pattern + N + oct + freq + dbg,
 *                    bright bold yellow (top, right-aligned)
 *   row rows-1     — every interactive key, bright bold cyan (bottom-left)
 * Debug-mode suffix only shown when non-normal; keeps the top tidy.
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);
    const PatternParams *pp = &pattern_params[s->current_pattern];

    const char *state_str = s->paused ? "PAUSED"
                                       : pattern_name(s->current_pattern);

    char top[160];
    if (g_debug == DBG_NORMAL) {
        snprintf(top, sizeof top,
                 " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
                 "N:%d oct:%d freq:%.3f ",
                 fps, sim_fps, s->speed,
                 state_str, themes[s->current_theme].name,
                 active, pp->fbm_octaves, (double)pp->fbm_freq);
    } else {
        snprintf(top, sizeof top,
                 " %5.1f fps  %3d Hz  speed:%-3d  %s  theme:%s  "
                 "N:%d oct:%d freq:%.3f  [dbg:%s] ",
                 fps, sim_fps, s->speed,
                 state_str, themes[s->current_theme].name,
                 active, pp->fbm_octaves, (double)pp->fbm_freq,
                 k_debug_names[g_debug]);
    }
    int top_x = sc->cols - (int)strlen(top);
    if (top_x < 0) top_x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, top_x, "%s", top);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q/ESC:quit  spc:pause  r:reseed  n/p:pattern"
             "  t/T:theme  +/-:speed  ]/[:Hz  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

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
    case 'r': case 'R': scene_reseed(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
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
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_populate_to_target(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_populate_to_target(s);
        break;

    case 'd':
        g_debug = (DebugMode)((g_debug + 1) % DBG_COUNT);
        break;
    case 'D':
        g_debug = (DebugMode)((g_debug + DBG_COUNT - 1) % DBG_COUNT);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

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

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
