/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke.c — a mushroom cloud rising from a Stam stable-fluid simulation
 *
 * DEMO: Detonate a Gaussian "blast" at the origin.  A 2-D axisymmetric
 *       fluid simulation (Stam's stable fluids) runs underneath: hot
 *       gas rises by buoyancy, advects through the velocity field,
 *       cools toward ambient.  A volumetric Beer-Lambert raymarcher
 *       renders the fluid back into 3-D for the screen.  Five blast
 *       presets, six themes (including a photographic-negative one),
 *       four debug overlays that show the raw fluid fields.
 *
 * Study alongside: fluid/navier_stokes.c (a sibling Stam stable-fluid
 *       solver — same advect/project/diffuse pattern in a more
 *       general 2-D form), and raymarcher/raymarcher.c (surface
 *       raymarching of an SDF).  This file sits between the two:
 *       the underlying simulation is a fluid solver, but the
 *       rendering technique is volumetric raymarching.  Different
 *       rendering equation from a surface raymarcher, very
 *       different feel.
 *
 * Section map:
 *   §1   config           — every tunable named, no magic numbers later
 *   §2   clock            — monotonic timer + sleep
 *   §3   color            — theme palette + HUD/hint pairs
 *   §4   vec3             — 3-D vector math used by the raymarcher
 *   §5   fluid state      — the simulation's memory layout
 *   §6   grid helpers     — clampf + bilinear sampling
 *   §7   boundaries       — what walls do to the velocity field
 *   §8   buoyancy         — hot air rises
 *   §9   advection        — fields move with the flow (Stam's trick)
 *   §10  divergence       — measure of "compression" in the velocity
 *   §11  pressure solve   — Jacobi iteration of ∇²p = ∇·v
 *   §12  gradient remove  — subtract ∇p from v
 *   §13  projection       — orchestrator combining §10..§12
 *   §14  cool + decay     — exponential return to ambient
 *   §15  fluid_step       — one full physics tick
 *   §16  detonate         — the initial Gaussian bubble
 *   §17  sampling         — read fluid from world coordinates
 *   §18  raymarch         — Beer-Lambert volumetric integration
 *   §19  camera           — orthonormal basis + per-pixel ray
 *   §20  cell decoration  — (lum, hot) → glyph + colour
 *   §21  scene            — the application's per-frame state
 *   §22  debug overlays   — see the raw fluid fields
 *   §23  screen           — ncurses init, HUD, present
 *   §24  app              — main loop, signals, key handling
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume the simulation
 *   r / R      re-detonate (restart the cloud from t = 0)
 *   n / N      next / previous blast preset
 *                (TACTICAL / STANDARD / MEGATON / AIR_BURST / GROUND)
 *   t / T      next / previous theme
 *                (REALISTIC / MATRIX / OCEAN / NOVA / TOXIC / NEGATIVE)
 *   d / D      cycle debug overlay
 *                (NORMAL / DENSITY 2D / TEMP 2D / VELOCITY)
 *   + / -      simulation rate up / down
 *   z / Z      camera closer / farther
 *   ] / [      render fps up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/nuke.c \
 *       -o nuke -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm and lists references.
 *   • MENTAL MODEL     intuition + an ASCII diagram of one fluid tick.
 *   • GUIDED TUTORIAL  ten short answers walking from "what is a fluid
 *                      sim?" through Stable Fluids' five steps and the
 *                      volumetric raymarcher that turns the result
 *                      back into a 3-D image.
 *   • §1..§24          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      v          — velocity field (vᵣ, vᵧ): radial + vertical components
 *      T          — temperature field
 *      ρ          — density field
 *      p          — pressure field (scratch, not physical)
 *      ∇·v        — divergence (scalar): how much fluid is leaving each cell
 *      ∇p         — gradient of pressure (vector): used to subtract divergence
 *      ∇²f        — Laplacian (scalar): used for the pressure solve
 *      h          — grid cell size (world units)
 *      dt         — timestep (seconds)
 *      β          — buoyancy coefficient (T-excess → vertical accel)
 *
 * Background you need:
 *   • basic vector arithmetic (add, dot, cross, length, normalise)
 *   • familiarity with ∇ (gradient) and ∇· (divergence) helpful but
 *     not required — Tutorials 2 and 5 introduce them informally
 *   • read raymarcher.c first if sphere tracing is unfamiliar; this
 *     file uses VOLUME raymarching (Beer-Lambert) instead, which is
 *     a different beast — see Tutorial 9
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : STAM'S STABLE FLUIDS (Stam 1999) on a 2-D
 *                axisymmetric grid (radial × vertical), driven by
 *                BUOYANCY from a Gaussian temperature bubble.  The
 *                whole simulation is six steps per tick:
 *                  1. apply buoyancy           (hot → upward push)
 *                  2. project                  (zero ∇·v)
 *                  3. advect velocity by itself
 *                  4. project again
 *                  5. advect temperature + density
 *                  6. cool + decay             (return to ambient)
 *
 *                For rendering: VOLUMETRIC RAYMARCHING.  Each pixel
 *                shoots one ray into 3-D space; at each march step
 *                the ray queries (radius, altitude) → bilinear-
 *                sampled (density, temperature) from the 2-D fluid
 *                grid.  Beer-Lambert integration: density attenuates
 *                light, hot temperature emits light.  The accumulated
 *                (total luminance, hot luminance) drives a glyph +
 *                colour decoration per terminal cell.
 *
 * Data         : Eight 2-D float arrays (~168 KB in BSS, no malloc).
 *                Velocity (vᵣ, vᵧ), passive scalars (T, ρ),
 *                projection scratch (p, ∇·v), two general scratch
 *                buffers used by the Jacobi solver and advection
 *                double-buffering.  The whole simulation lives in
 *                one global Fluid struct.
 *
 * Rendering    : One ray per terminal cell.  Glyph from a faint-
 *                to-solid 8-tier ramp ('.' → '@') indexed by total
 *                luminance.  Colour from the active theme's 8-band
 *                ramp indexed by HOT FRACTION (hot luminance ÷ total)
 *                — a thin wisp of cool smoke gets band 0, the bright
 *                core of a fireball gets band 7.
 *
 * Performance  : ~130 march steps per ray × ~80 cols × ~22 rows =
 *                ~230 000 sample calls per frame.  The simulation's
 *                40-iteration Jacobi solver runs at SIM_DT = 25 ms,
 *                so ~40 × 56 × 96 × 2 projections ≈ 430 K cell
 *                updates per fluid step.  Comfortable at 60 fps on
 *                modern CPUs.
 *
 * References   :
 *   • Stam, J. (1999) — "Stable Fluids", *SIGGRAPH '99 Proceedings*,
 *     pp. 121-128.  The paper that introduced semi-Lagrangian
 *     advection and the projection step used here.
 *   • Stam, J. (2003) — "Real-Time Fluid Dynamics for Games",
 *     *Proceedings of the GDC*.  A simpler exposition of the same
 *     algorithm, with code samples close to ours.
 *   • Quílez, I. — "Volumetric raymarching"
 *     https://iquilezles.org/articles/raymarchingvolumes/
 *     The Beer-Lambert volume integration loop in §18.
 *   • Beer-Lambert law — https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law
 *     Background physics for why we multiply by exp(−optical_depth).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two simulations run side-by-side: a FLUID DYNAMICS simulation that
 * tracks how hot gas moves through 2-D space, and a RAYMARCHING
 * RENDERER that turns the 2-D simulation back into a 3-D image by
 * exploiting the cloud's axial symmetry.  The fluid never knows about
 * pixels; the renderer never knows about pressure.  Both meet at one
 * narrow interface: sample_density() and sample_temperature(), which
 * convert a world-space (x, y, z) point to a (radius, altitude) pair
 * and bilinearly sample the 2-D grid.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a slowly-evolving photograph of a smoke column.  The
 * photograph is 2-D (a slice through the column).  Because the column
 * has rotational symmetry around its vertical axis, a 3-D
 * reconstruction is just "rotate the slice around the y-axis".  Every
 * ray from the camera, after tracing through this rotational
 * extrusion of the slice, accumulates light by Beer-Lambert: fire
 * cells emit, smoke cells absorb.  The 2-D slice itself evolves under
 * Stam's stable-fluid rules: hot cells push up, the velocity field is
 * cleaned to be divergence-free, then everything advects with the
 * flow and cools toward ambient.
 *
 * One fluid tick (Stable Fluids cycle):
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   T (temperature) ──────► buoyancy ─┐                   │
 *      │                                     ▼                   │
 *      │   v (velocity) ◄──── project ◄── (v + Δv from buoyancy) │
 *      │                       │                                 │
 *      │                       ▼  (∇·v ≈ 0)                      │
 *      │                  advect v by itself                     │
 *      │                       │                                 │
 *      │                       ▼                                 │
 *      │                    project ◄──── advection re-introduces│
 *      │                       │           a tiny ∇·v            │
 *      │                       ▼                                 │
 *      │           advect (T, ρ) by clean v                      │
 *      │                       │                                 │
 *      │                       ▼                                 │
 *      │           cool toward ambient · density decay           │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once at startup:
 *   0. detonate     paint a Gaussian temperature + density bubble
 *                   at the origin, plus a radial outflow seed
 *
 * Once per simulation tick (SIM_DT = 25 ms):
 *   1. buoyancy     vᵧ += β · (T − T_ambient) · dt
 *   2. project      ∇·v → 0 (Hodge decomposition via Jacobi)
 *   3. advect v     trace each cell backward by v · dt; bilinear sample
 *   4. project      again (advection re-introduces tiny ∇·v)
 *   5. advect T, ρ  same backward-trace, bilinear sample
 *   6. cool + decay T → T_ambient exponentially; ρ fades linearly
 *
 * Once per render frame:
 *   7. ray gen      orthonormal camera basis; per-pixel ray dir
 *   8. raymarch     for each step:
 *                       (radius, altitude) ← world (x, y, z)
 *                       sample (ρ, T) from the 2-D grid
 *                       accumulate luminance with Beer-Lambert decay
 *   9. decorate     (total_lum, hot_lum) → glyph + theme colour
 *  10. emit         with attron/attroff batched on (pair, attr) change
 *
 * KEY FORMULAS
 * ────────────
 * Buoyancy (Boussinesq):
 *      vᵧ_new = vᵧ_old + β · (T − T_ambient) · dt
 *
 * Semi-Lagrangian advection (Stam):
 *      f_new[i, j] = bilinear( f_old, (i, j) − v[i, j] · dt / h )
 *
 * Divergence (centred difference):
 *      ∇·v[i, j] = (vᵣ[i+1,j] − vᵣ[i−1,j]) / (2h)
 *                + (vᵧ[i,j+1] − vᵧ[i,j−1]) / (2h)
 *
 * Poisson equation for pressure (Jacobi update):
 *      p_new[i, j] = ( p[i+1,j] + p[i−1,j] + p[i,j+1] + p[i,j−1]
 *                      − h² · ∇·v[i, j] ) / 4
 *
 * Gradient subtraction (closes the projection):
 *      vᵣ_clean[i, j] = vᵣ[i, j] − (p[i+1,j] − p[i−1,j]) / (2h)
 *      vᵧ_clean[i, j] = vᵧ[i, j] − (p[i,j+1] − p[i,j−1]) / (2h)
 *
 * Newton cooling toward ambient:
 *      T_new = T_ambient + (T_old − T_ambient) · exp(−k_cool · dt)
 *
 * Beer-Lambert volume integration (per ray):
 *      transmittance, L_total, L_hot = 1, 0, 0
 *      for each step at distance s from origin:
 *          dτ      = ρ · STEP · DENSITY_GAIN
 *          emit    = clamp((T − T_ambient) / (T_peak − T_ambient), 0, 1)
 *          source  = emit · EMISSION_GAIN + AMBIENT_FLOOR
 *          L_total += transmittance · dτ · source
 *          L_hot   += transmittance · dτ · emit · EMISSION_GAIN
 *          transmittance *= exp(−dτ)
 *
 * World → grid (axisymmetric reduction):
 *      radius   = √(x² + z²)
 *      altitude = y
 *      → bilinear sample at (radius / h, altitude / h)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Why TWO projections per fluid step (2 and 4 above)?  Buoyancy
 *     adds a vertical impulse → divergence.  We project to clean it
 *     up.  But then advection itself, when applied to a divergence-
 *     free field, can RE-INTRODUCE small divergence (bilinear
 *     sampling doesn't preserve incompressibility exactly).  We
 *     project again before advecting the scalars.  Stam recommends
 *     this; without the second projection the cloud subtly drifts.
 *
 *   • The Jacobi solver runs 40 iterations.  Below ~25 the pressure
 *     field hasn't converged and visible "puff" artefacts appear
 *     where the residual divergence pushes scalars around.  Above 40
 *     gives diminishing returns at significant CPU cost.
 *
 *   • Velocity advection needs DOUBLE BUFFERING: vᵣ and vᵧ are
 *     advected by the SAME field (themselves), so we can't overwrite
 *     vᵣ before reading it for vᵧ's advection.  We use scratch_a
 *     and scratch_b as destinations, then memcpy back.
 *
 *   • The simulation is 2-D AXISYMMETRIC: the radial coordinate runs
 *     0..GRID_RADIAL_EXTENT; we IMPLICITLY rotate around the y-axis
 *     for rendering.  This loses any non-axisymmetric structure (the
 *     cloud can't have a tilted column or wind shear).  Trade-off
 *     for affordable simulation in a terminal.
 *
 *   • CAM_DISTANCE_MIN (8.0) is just outside the typical MEGATON
 *     cap.  Pushing closer puts the camera inside the cloud — visible
 *     as the screen filling with smoke colour.
 *
 *   • The NEGATIVE theme uses white background, dark foreground.
 *     A_BOLD/A_DIM are disabled for it (see decorate_volume_pixel)
 *     because they invert their visual meaning against a light bg.
 *
 *   • SIM_DT_MAX_REAL (80 ms) caps the per-frame sim advance.
 *     Without it, a hiccup that delays the loop for 1 s would try to
 *     advance 1 s of sim time in one frame — the cloud would
 *     teleport to a much later state.  The cap clamps to ~3 sim
 *     steps per real frame.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At t = 0 the screen shows just a hint of glow at altitude
 *     ≈ blast.detonation_altitude.  Within 2 sim seconds a clear
 *     mushroom STEM forms; cap appears around 4-6 seconds; visibly
 *     levels off around 10-15 seconds.
 *
 *   • Press n to cycle blast presets.  TACTICAL is small and brief;
 *     MEGATON is huge and long-lasting; AIR_BURST has no ground stem
 *     (the detonation altitude is high); GROUND has a wide base.
 *
 *   • Press d to cycle debug overlays.  DENSITY 2D shows the
 *     underlying 2-D density slice (no rendering).  TEMPERATURE 2D
 *     shows the heat field — watch buoyancy push hot cells upward.
 *     VELOCITY shows the flow field as ASCII arrows.
 *
 *   • Press t through all 6 themes.  Geometry stays identical;
 *     only the smoke→fire colour ramp changes.
 *
 *   • Press r to re-detonate.  The same physics replays, deterministic.
 *
 *   • Press z to zoom in.  The cloud appears larger; at minimum
 *     distance you can see individual "puff" cells.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: What is a fluid simulation?
 * ───────────────────────────────
 * A grid of cells, each storing some QUANTITIES that vary over space:
 *
 *      VELOCITY   how fast and which way the fluid is moving here
 *      DENSITY    how much "stuff" (smoke) is in this cell
 *      TEMPERATURE  how hot is the fluid here
 *      PRESSURE   internal force (used to enforce mass conservation)
 *
 * Each tick of simulation:
 *   – stuff MOVES according to the velocity (advection)
 *   – velocity CHANGES according to forces (buoyancy, pressure, …)
 *   – temperature changes according to its own laws (cooling, etc.)
 *
 * Run this loop for thousands of ticks → an evolving fluid.  Render
 * the density + temperature each frame → a movie.
 *
 *
 * T2: Why incompressibility matters (∇·v = 0)
 * ───────────────────────────────────────────
 * For air at sub-sonic speeds the volume of any chunk doesn't
 * meaningfully compress or expand — fluid flowing INTO a cell must
 * equal fluid flowing OUT.  The mathematical statement is:
 *
 *      ∇·v = 0     (the divergence of the velocity field is zero)
 *
 * where divergence at a point is the net "outflow per unit volume"
 * — sum of velocity component derivatives along each axis.  In 2-D:
 *
 *      ∇·v(i, j) = ∂vᵣ/∂r + ∂vᵧ/∂y
 *
 * If we don't enforce ∇·v = 0, the simulation creates matter from
 * nowhere or destroys it: cells "fill up" or "empty out" without
 * cause.  Visually: clouds bulge or shrink unphysically, smoke pools
 * at the bottom of the screen.  Stam's projection step (T5) is what
 * keeps the velocity field divergence-free.
 *
 *
 * T3: Buoyancy — the force that lifts hot air
 * ───────────────────────────────────────────
 * Hot gas is less dense than cold gas at the same pressure, so cold
 * surrounding air pushes it up.  The Boussinesq approximation:
 *
 *      acceleration_y  =  β · (T − T_ambient) · g
 *
 * where β is the thermal expansion coefficient.  We bake (β · g)
 * into one constant BUOYANCY_COEFFICIENT.  Per cell, per tick:
 *
 *      vᵧ[i, j]  +=  BUOYANCY · (T[i, j] − T_ambient) · dt
 *
 * That's the entire buoyancy step — five characters of code and one
 * Newton's-law integration.  Removing this function gives a cold,
 * drifting cloud (advection still moves stuff, but nothing lifts
 * it).  Doubling BUOYANCY_COEFFICIENT gives a fast riser; halving
 * it gives a slow, ponderous cloud.
 *
 *
 * T4: Advection — moving stuff with velocity (Stam's trick)
 * ─────────────────────────────────────────────────────────
 * The naïve approach: PUSH each cell's value forward by velocity·dt:
 *
 *      new_field[i + vᵣ·dt][j + vᵧ·dt]  =  field[i][j]
 *
 * Two problems: (a) the "destination" coords are fractional; (b) at
 * big timesteps the value lands outside the grid or overlaps other
 * pushed values.  Either way it BLOWS UP — values amplify each tick.
 *
 * Stam's STABLE FLUIDS trick: trace BACKWARD instead.  For each
 * destination cell, ask "where was the fluid currently here, one
 * step ago?".  Sample the OLD field at that source location:
 *
 *      for each cell c at grid coord (i, j):
 *          source_pos = (i, j) − v·dt / h          (backward trace)
 *          new_field[i, j] = bilinear(old_field, source_pos)
 *
 * Bilinear interpolation gives a clean weighted average of the four
 * neighbouring cells around the source position.  Two key
 * properties:
 *
 *   STABILITY      The bilinear sample is bounded by the four
 *                  neighbours; the field can never blow up no matter
 *                  how big dt is.
 *   DIFFUSION      Bilinear interpolation always loses a little
 *                  high-frequency detail.  Each tick the field gets
 *                  very slightly blurrier.  Acceptable price.
 *
 * One advect_field() works for any scalar field: vᵣ, vᵧ, T, ρ.
 *
 *
 * T5: Pressure projection — clean up divergence
 * ─────────────────────────────────────────────
 * After buoyancy adds vertical velocity (T3), the velocity field
 * has nonzero divergence (some cells are now "creating" upward
 * fluid).  To restore ∇·v = 0:
 *
 *   HODGE DECOMPOSITION (the math fact we exploit).  Any vector
 *   field can be uniquely split:
 *
 *      v  =  v_div_free  +  ∇φ
 *
 *   where ∇φ is the gradient of some scalar field φ.  We don't
 *   know v_div_free directly, but we can find ∇φ by solving:
 *
 *      ∇²φ  =  ∇·v          (Poisson equation for φ)
 *
 *   Then v_div_free = v − ∇φ.  In our notation φ is "pressure" p,
 *   so the algorithm is:
 *
 *     1. compute ∇·v (T2)
 *     2. solve ∇²p = ∇·v for p (T6)
 *     3. subtract ∇p from v
 *
 *   The result is a velocity field whose divergence is approximately
 *   zero — fluid is conserved.
 *
 *
 * T6: Solving the Poisson equation by Jacobi iteration
 * ────────────────────────────────────────────────────
 * The discrete Laplacian on our grid:
 *
 *      ∇²p[i, j] ≈ (p[i+1,j] + p[i−1,j] + p[i,j+1] + p[i,j−1]
 *                   − 4·p[i, j]) / h²
 *
 * Setting this equal to ∇·v[i, j] and solving for p[i, j]:
 *
 *      p[i, j] = (p[i+1,j] + p[i−1,j] + p[i,j+1] + p[i,j−1]
 *                 − h² · ∇·v[i, j]) / 4
 *
 * This is the JACOBI UPDATE: replace each cell with the average of
 * its four neighbours minus a divergence correction.  Iterate until
 * the field converges.  We use 40 sweeps — enough for visual
 * quality, fast enough for 60 fps.
 *
 * Boundary conditions: zero-gradient (mirror the neighbours).  This
 * corresponds to no-flow walls everywhere — consistent with how we
 * treat the axis (no radial flow through r=0) and ground (no
 * vertical flow through y=0).
 *
 *
 * T7: The full fluid_step orchestrator
 * ────────────────────────────────────
 * Six steps per tick:
 *
 *      1. apply_buoyancy(dt)             (§8) — hot → upward push
 *      2. project()                      (§13) — clean ∇·v
 *      3. advect velocity by itself      (§9) — uses scratch buffers
 *      4. project()                      (§13) — clean it again
 *      5. advect (T, ρ) by clean v       (§9) — passive scalars ride
 *      6. cool_and_decay(dt)             (§14) — return to ambient
 *
 * Why TWO projections (steps 2 and 4)?  Buoyancy adds divergence,
 * step 2 cleans it.  Then advection RE-INTRODUCES tiny divergence
 * (bilinear sampling is not exactly volume-preserving), so step 4
 * cleans it again before passive scalars get advected.  Without
 * step 4 the scalars subtly drift over time.
 *
 *
 * T8: Detonation — the only scripted moment
 * ─────────────────────────────────────────
 * Everything in the simulation is pure physics — no scripting,
 * no animation curves.  The only exception: the INITIAL CONDITION,
 * a Gaussian "blast" at t = 0.
 *
 *      gauss(d) = exp(−d² / (2·σ²))         d² = r² + (y − y₀)²
 *
 *      T(r, y) = T_ambient + (T_peak − T_ambient) · gauss(d)
 *      ρ(r, y) = ρ_peak · gauss(d)
 *
 * Plus a small RADIAL OUTFLOW velocity near the centre — the
 * mechanical shock wave that happens before buoyancy takes over.
 * Without this, the bubble just rises with a clean leading-edge
 * vortex.  With it, the bubble first expands radially (~1-2 sec)
 * then transitions to buoyant rise — much closer to a real
 * explosion's two-phase profile.
 *
 * After detonation, no more scripting.  The sim from there is
 * physics: buoyancy lifts hot cells, advection moves stuff,
 * projection conserves mass, cooling returns toward ambient.
 *
 *
 * T9: Volume rendering with Beer-Lambert
 * ──────────────────────────────────────
 * Surface raymarching (raymarcher.c) finds WHERE a ray hits a
 * surface.  Volume rendering integrates LIGHT along a ray as it
 * passes through a SEMI-TRANSPARENT MEDIUM.  For each ray:
 *
 *      transmittance, L = 1, 0
 *      for each step at distance s from origin:
 *          dτ      = ρ · STEP · DENSITY_GAIN     (optical depth gained)
 *          emit    = (T − T_ambient) / (T_peak − T_ambient)    in [0, 1]
 *          source  = emit · EMISSION_GAIN + AMBIENT_FLOOR
 *          L      += transmittance · dτ · source                (accumulate)
 *          transmittance *= exp(−dτ)                            (Beer-Lambert)
 *          if transmittance < ε: break
 *
 * What's happening physically:
 *   – DENSITY blocks light: each step subtracts dτ from optical
 *     transmittance, and exp(−dτ) is what fraction of incoming light
 *     survives the segment (Beer-Lambert law).
 *   – TEMPERATURE emits light: hot cells contribute proportional to
 *     their temperature excess.  A cool foggy cell adds nothing
 *     beyond AMBIENT_FLOOR (a faint Rayleigh-scatter analogue).
 *   – Each contribution is multiplied by transmittance — light from
 *     cells far behind opaque smoke doesn't reach the camera.
 *
 * The TWO outputs:
 *   total_luminance   what overall brightness this pixel sees
 *   hot_luminance     how much of that came from emission
 *
 * The HOT FRACTION (hot/total) is what drives smoke vs. fire colour
 * in the theme palette (T10).
 *
 *
 * T10: Bridging 3-D world → 2-D axisymmetric grid
 * ───────────────────────────────────────────────
 * The fluid simulation is 2-D (radial × vertical = 56 × 96 cells).
 * The volume renderer thinks in 3-D world coordinates (x, y, z).
 * The bridge is one line of math:
 *
 *      radius   = √(x² + z²)
 *      altitude = y
 *
 * The "axisymmetric" assumption: the cloud is rotationally symmetric
 * around the y-axis.  Any 3-D world point's density is determined
 * solely by its radial distance from the y-axis and its altitude.
 * Sample the 2-D fluid grid at (radius/h, altitude/h) bilinearly →
 * the 3-D density at (x, y, z).
 *
 * Ramifications:
 *   – CHEAPER SIM.  We only need to track ~5K grid cells, not ~3M.
 *   – LIMITED PHENOMENA.  No tilted columns, no wind shear, no
 *     non-circular caps.  Everything is mirror-symmetric around y.
 *
 * For a MUSHROOM CLOUD this is approximately fine — real mushroom
 * clouds are very nearly axisymmetric.  For a leaning campfire or
 * wind-blown smoke, it would not work.
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
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
    TARGET_RENDER_FPS_MIN     =  10,
    TARGET_RENDER_FPS_DEFAULT =  60,
    TARGET_RENDER_FPS_MAX     = 120,
    TARGET_RENDER_FPS_STEP    =  10,

    FPS_DISPLAY_UPDATE_MS     = 500,
    HUD_ROWS                  =   2,   /* row 0 status + last row hint */
};

/* §1.2 colour-pair IDs. */
#define PAIR_HUD_STATUS    1     /* yellow + bold (top status row) */
#define PAIR_HUD_HINT      2     /* cyan   + bold (key-hint row)   */
#define PAIR_RAMP_BASE     3     /* +0..+7 — smoke→fire ramp       */

/* §1.3 time helpers. */
#define NS_PER_SECOND      1000000000LL
#define NS_PER_MILLISECOND    1000000LL
#define TICK_NS(target_fps)  (NS_PER_SECOND / (target_fps))

/* §1.4 terminal cell aspect ratio. */
#define TERMINAL_CELL_ASPECT  2.0f      /* physical h / w */

/* §1.5 fluid grid (T10). */
#define GRID_RADIAL_CELLS    56
#define GRID_VERTICAL_CELLS  96
#define GRID_CELL_SIZE       0.125f
#define GRID_RADIAL_EXTENT   ((float)GRID_RADIAL_CELLS   * GRID_CELL_SIZE)
#define GRID_VERTICAL_EXTENT ((float)GRID_VERTICAL_CELLS * GRID_CELL_SIZE)
#define GRID_INV_CELL_SIZE   (1.0f / GRID_CELL_SIZE)

#define POISSON_JACOBI_ITERATIONS  40    /* Jacobi sweeps per project (T6) */

/* §1.6 physics constants. */
#define TEMPERATURE_AMBIENT       1.0f
#define TEMPERATURE_PEAK          8.0f
#define DENSITY_PEAK              4.0f
#define BUOYANCY_COEFFICIENT      2.4f   /* β·g, see T3 */
#define COOL_RATE                 0.06f  /* Newton cooling (T7 step 6) */
#define DENSITY_DECAY             0.009f /* density linear loss / sec   */

/* §1.7 initial conditions per blast type (T8). */
typedef enum {
    BLAST_TACTICAL    = 0,
    BLAST_STANDARD    = 1,
    BLAST_MEGATON     = 2,
    BLAST_AIR_BURST   = 3,
    BLAST_GROUND      = 4,
    BLAST_TYPE_COUNT  = 5,
} BlastType;

typedef struct {
    const char *display_name;
    float       sigma;                   /* Gaussian σ (world units)    */
    float       peak_temperature;        /* T peak above ambient        */
    float       peak_density;            /* ρ peak                      */
    float       detonation_altitude;     /* y of blast centre           */
    float       initial_outflow;         /* radial outflow speed at t=0 */
} BlastParameters;

static const BlastParameters BLAST_PRESETS[BLAST_TYPE_COUNT] = {
    /* TACTICAL  — small, low-altitude, brief mushroom */
    { "TACTICAL  ",  0.35f,  5.0f,  2.5f,  1.0f,  2.0f },
    /* STANDARD  — canonical mid-yield mushroom */
    { "STANDARD  ",  0.55f,  8.0f,  4.0f,  1.6f,  3.0f },
    /* MEGATON   — huge yield, very tall column, long-lasting cap */
    { "MEGATON   ",  0.85f, 12.0f,  5.5f,  2.2f,  4.5f },
    /* AIR_BURST — high-altitude detonation, no ground stem */
    { "AIR_BURST ",  0.55f,  8.0f,  4.0f,  4.5f,  3.0f },
    /* GROUND    — surface burst, modest heat, heavy dust load */
    { "GROUND    ",  0.45f,  7.0f,  7.0f,  0.6f,  3.5f },
};

/* §1.8 simulation timing. */
#define SIM_DT                 0.025f    /* fixed sim step (sim time) */
#define SIM_DT_MAX_REAL        0.080f    /* cap per real frame */
#define SIM_RATE_DEFAULT       1.0f
#define SIM_RATE_MIN           0.10f
#define SIM_RATE_MAX           6.0f
#define SIM_RATE_STEP_FACTOR   1.30f

/* §1.9 camera. */
#define CAM_DISTANCE_DEFAULT   28.0f
#define CAM_DISTANCE_MIN        8.0f      /* keep camera outside MEGATON cap */
#define CAM_DISTANCE_MAX       56.0f
#define CAM_DISTANCE_STEP       2.0f
#define CAM_HEIGHT              5.0f
#define CAM_LOOK_AT_HEIGHT      6.0f
#define CAM_FIELD_OF_VIEW_DEG  52.0f

/* §1.10 volumetric raymarcher (T9). */
#define RM_RAY_NEAR                       0.5f
#define RM_RAY_FAR                       32.0f
#define RM_STEP                           0.18f
#define RM_MAX_STEPS                    130
#define RM_OPAQUE_TRANSMITTANCE_EPS       0.01f
#define RM_DENSITY_GAIN                   1.30f
#define RM_EMISSION_GAIN                  4.5f
#define RM_AMBIENT_FLOOR                  0.06f

/* §1.11 pixel classification. */
#define PIXEL_LUMINANCE_CLAMP    1.10f
#define PIXEL_VISIBLE_LUM_EPS    0.002f
#define GLYPH_SLOT_COUNT         8
#define GLYPH_SLOT_FLOAT         7.999f   /* (GLYPH_SLOT_COUNT - 0.001) */

/* §1.12 themes — 8-band ramp from coolest (slot 0) to hottest (slot 7).
 * The `inverted` flag flips foreground/background semantics for the
 * NEGATIVE theme: white background, dark foreground.
 */
typedef struct {
    const char *display_name;
    short       ramp_256[GLYPH_SLOT_COUNT];
    bool        inverted_background;
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    /* REALISTIC: light grey smoke climbing into orange-yellow fire. */
    { "REALISTIC",
      { 248, 250, 252, 254, 130, 166, 208, 220 }, false },

    /* MATRIX: lime-green smoke + lime fire. */
    { "MATRIX   ",
      {  64,  70, 112, 113, 119, 154, 190, 226 }, false },

    /* OCEAN: deep teal smoke + cyan fire. */
    { "OCEAN    ",
      {  37,  44,  45,  74,  81, 117, 159, 195 }, false },

    /* NOVA: violet-lavender smoke + ice-blue fire. */
    { "NOVA     ",
      {  97,  98,  99, 105, 111, 147, 153, 195 }, false },

    /* TOXIC: chartreuse smoke + acid green-yellow fire. */
    { "TOXIC    ",
      { 107, 113, 149, 155, 191, 192, 226, 228 }, false },

    /* NEGATIVE: white background, dark foreground (photographic
     * negative). */
    { "NEGATIVE ",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true },
};

/* §1.13 luminance glyph ramp.  Slot 0 is `.` (not space) so thin
 * smoke at end-of-life renders as a faint dot; below
 * PIXEL_VISIBLE_LUM_EPS the cell is left as background. */
static const char LUMINANCE_GLYPHS[GLYPH_SLOT_COUNT] =
    { '.', ',', ':', ';', '+', '*', '#', '@' };

/* §1.14 debug overlays — d / D cycles between them.
 *      DEBUG_NORMAL       full 3-D volumetric raymarch (production)
 *      DEBUG_DENSITY      raw 2-D density map
 *      DEBUG_TEMPERATURE  raw 2-D temperature map
 *      DEBUG_VELOCITY     raw 2-D velocity field (ASCII arrows)
 */
typedef enum {
    DEBUG_NORMAL       = 0,
    DEBUG_DENSITY      = 1,
    DEBUG_TEMPERATURE  = 2,
    DEBUG_VELOCITY     = 3,
    DEBUG_MODE_COUNT   = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL    ", "DENSITY 2D", "TEMP 2D   ", "VELOCITY  ",
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_now_nanoseconds(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SECOND + t.tv_nsec;
}

static void clock_sleep_nanoseconds(int64_t nanoseconds)
{
    if (nanoseconds <= 0) return;
    struct timespec request = {
        .tv_sec  = (time_t)(nanoseconds / NS_PER_SECOND),
        .tv_nsec = (long)  (nanoseconds % NS_PER_SECOND),
    };
    nanosleep(&request, NULL);
}

/* ── §3 color — theme palette + HUD/hint pairs ───────────────────────── */

static void apply_theme(int theme_index)
{
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const Theme *theme = &THEMES[theme_index];

    short background_256 = theme->inverted_background ? 231 : -1;
    short background_8   = theme->inverted_background ? COLOR_WHITE : -1;

    if (COLORS >= 256) {
        for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
            init_pair((short)(PAIR_RAMP_BASE + slot),
                      theme->ramp_256[slot], background_256);
    } else {
        /* 8-colour fallback — coarse approximation. */
        static const short FALLBACK_RAMP_8[GLYPH_SLOT_COUNT] = {
            COLOR_BLACK, COLOR_BLACK, COLOR_WHITE, COLOR_WHITE,
            COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_WHITE,
        };
        for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
            init_pair((short)(PAIR_RAMP_BASE + slot),
                      theme->inverted_background ? COLOR_BLACK
                                                 : FALLBACK_RAMP_8[slot],
                      background_8);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD_STATUS, 226, -1);   /* bright yellow */
        init_pair(PAIR_HUD_HINT,    51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD_STATUS, COLOR_YELLOW, -1);
        init_pair(PAIR_HUD_HINT,   COLOR_CYAN,   -1);
    }
    apply_theme(0);
}

/* ── §4 vec3 — 3-D vector math used by the raymarcher ────────────────── */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3   (float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)            { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)            { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3scale(float s, Vec3 a)         { return v3(s*a.x, s*a.y, s*a.z); }
static inline float v3dot(Vec3 a, Vec3 b)            { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3  v3cross(Vec3 a, Vec3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline float v3length(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3  v3normalise(Vec3 a)
{
    float length = v3length(a);
    return (length > 1e-12f) ? v3scale(1.0f / length, a) : v3(0, 0, 1);
}

/* ── §5 fluid state — the simulation's memory layout ─────────────────── *
 *
 * Eight 2-D fields, stored side-by-side in one struct.  Total size
 * 8 × 56 × 96 × 4 bytes ≈ 168 KB, sitting in BSS.  No malloc.
 *
 * Indexing convention:
 *   i ∈ [0, GRID_RADIAL_CELLS)     — radial slot (0 = on the axis)
 *   j ∈ [0, GRID_VERTICAL_CELLS)   — vertical slot (0 = on the ground)
 *
 * The grid is the AXISYMMETRIC slice (T10): a 2-D vertical strip
 * from the y-axis outward.  Rotated around y for rendering.
 */
typedef struct {
    /* Velocity (radial, vertical). */
    float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Advected scalars. */
    float temperature       [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float density           [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Projection scratch (T5, T6). */
    float pressure          [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float divergence        [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* General scratch buffers (Jacobi double-buffer + advection swap). */
    float scratch_a         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float scratch_b         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
} Fluid;

static Fluid g_fluid;

/* ── §6 grid helpers — clampf + bilinear sampling ────────────────────── */

static inline float clampf(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/*
 * sample_field_bilinear — sample a 2-D scalar field at FRACTIONAL
 * cell indices.  Out-of-grid coords clamp to the boundary cell.
 *
 * Bilinear interpolation: weighted average of the four cells
 * surrounding (fi, fj):
 *
 *      (i+1, j)·──────·(i+1, j+1)        weights:
 *           │        │                  (i,    j  ): (1−fri)·(1−fyj)
 *           │   ✦    │                  (i+1,  j  ): fri    ·(1−fyj)
 *           │        │                  (i,    j+1): (1−fri)·fyj
 *      (i,    j)·──────·(i, j+1)         (i+1,  j+1): fri    ·fyj
 */
static float sample_field_bilinear(
        const float field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float fi, float fj)
{
    fi = clampf(fi, 0.0f, (float)(GRID_RADIAL_CELLS   - 1));
    fj = clampf(fj, 0.0f, (float)(GRID_VERTICAL_CELLS - 1));

    int i = (int)floorf(fi);
    int j = (int)floorf(fj);
    if (i >= GRID_RADIAL_CELLS   - 1) i = GRID_RADIAL_CELLS   - 2;
    if (j >= GRID_VERTICAL_CELLS - 1) j = GRID_VERTICAL_CELLS - 2;

    float frac_i = fi - (float)i;
    float frac_j = fj - (float)j;

    float bottom_left  = field[i    ][j    ];
    float bottom_right = field[i + 1][j    ];
    float top_left     = field[i    ][j + 1];
    float top_right    = field[i + 1][j + 1];

    float bottom_row = bottom_left * (1.0f - frac_i) + bottom_right * frac_i;
    float top_row    = top_left    * (1.0f - frac_i) + top_right    * frac_i;
    return bottom_row * (1.0f - frac_j) + top_row * frac_j;
}

/* ── §7 boundaries — what walls do to the velocity field ─────────────── *
 *
 * Two physical walls:
 *   1. AXIS at radius = 0 — mirror; no radial flow through.
 *   2. GROUND at altitude = 0 — solid; no vertical flow through.
 *
 * Other domain edges (top, far radial) are "open" — advection
 * naturally diffuses fluid that tries to leave the grid.
 *
 * Called after every velocity-modifying step (buoyancy, advection,
 * projection).
 */
static void enforce_velocity_boundaries(Fluid *fluid)
{
    /* Axis (radius = 0): no radial flow. */
    for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
        fluid->velocity_radial[0][j] = 0.0f;

    /* Ground (y = 0): no vertical flow. */
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        fluid->velocity_vertical[i][0] = 0.0f;
}

/* ── §8 buoyancy — hot air rises ─────────────────────────────────────── *
 *
 * Tutorial T3 derived this:
 *      vᵧ[i, j]  +=  BUOYANCY · (T[i, j] − T_ambient) · dt
 *
 * Five characters of code and one Newton's-law integration.
 * Removing this function gives a cold, drifting cloud (no lift).
 */
static void apply_buoyancy(Fluid *fluid, float step_seconds)
{
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float temperature_excess =
                fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
            fluid->velocity_vertical[i][j] +=
                BUOYANCY_COEFFICIENT * temperature_excess * step_seconds;
        }
    }

    enforce_velocity_boundaries(fluid);
}

/* ── §9 advection — fields move with the flow (Stam's trick) ─────────── *
 *
 * Tutorial T4 explained the backward-trace.  Generic over the field
 * being advected — we use this for vᵣ, vᵧ, T, ρ.
 *
 *   for each cell (i, j):
 *       source_pos     = (i, j) − v·dt / h
 *       new_field[i,j] = bilinear(old_field, source_pos)
 */
static void advect_field(
        float        destination_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float       source_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float       step_seconds)
{
    /* Velocities are world-units/sec; we want grid-cells, so divide by
     * cell size = multiply by INV_CELL_SIZE. */
    float dt_in_grid_units = step_seconds * GRID_INV_CELL_SIZE;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float source_i = (float)i - velocity_radial  [i][j] * dt_in_grid_units;
            float source_j = (float)j - velocity_vertical[i][j] * dt_in_grid_units;
            destination_field[i][j] =
                sample_field_bilinear(source_field, source_i, source_j);
        }
    }
}

/* ── §10 divergence — measure of "compression" ───────────────────────── *
 *
 * Tutorial T2 explained ∇·v.  Discrete centred difference:
 *      ∇·v[i, j] = (vᵣ[i+1,j] − vᵣ[i−1,j] + vᵧ[i,j+1] − vᵧ[i,j−1]) / (2h)
 *
 * Boundary cells use mirror neighbours via mirror_index, which
 * implements the no-flow axis and ground conditions implicitly.
 */

static int mirror_index(int index, int grid_size)
{
    if (index < 0)         return 1;
    if (index >= grid_size) return grid_size - 2;
    return index;
}

static void compute_divergence(Fluid *fluid)
{
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            int i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
            int i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
            int j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
            int j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);

            float dvr_dr = (fluid->velocity_radial  [i_right][j]
                          - fluid->velocity_radial  [i_left ][j]);
            float dvy_dy = (fluid->velocity_vertical[i][j_above]
                          - fluid->velocity_vertical[i][j_below]);

            fluid->divergence[i][j] =
                (dvr_dr + dvy_dy) * 0.5f * GRID_INV_CELL_SIZE;

            /* Reset pressure for the Jacobi iteration that follows. */
            fluid->pressure[i][j] = 0.0f;
        }
    }
}

/* ── §11 pressure solve — Jacobi iteration of ∇²p = ∇·v ──────────────── *
 *
 * Tutorial T6 derived the Jacobi update.  40 sweeps; double-buffer
 * via scratch_a then memcpy back.
 */
static void solve_pressure_poisson(Fluid *fluid)
{
    float h_squared = GRID_CELL_SIZE * GRID_CELL_SIZE;

    for (int iteration = 0; iteration < POISSON_JACOBI_ITERATIONS; iteration++) {

        /* One Jacobi sweep — write into scratch_a. */
        for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
            for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
                int i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
                int i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
                int j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
                int j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);

                float neighbour_sum = fluid->pressure[i_right][j]
                                    + fluid->pressure[i_left ][j]
                                    + fluid->pressure[i][j_above]
                                    + fluid->pressure[i][j_below];

                fluid->scratch_a[i][j] =
                    (neighbour_sum - h_squared * fluid->divergence[i][j])
                    * 0.25f;
            }
        }

        /* Swap: scratch_a → pressure. */
        memcpy(fluid->pressure, fluid->scratch_a, sizeof fluid->pressure);
    }
}

/* ── §12 gradient remove — subtract ∇p from v ────────────────────────── *
 *
 * Tutorial T5: v_clean = v − ∇p.  Centred-difference gradient.
 * After this step the velocity field has approximately zero
 * divergence everywhere.
 */
static void subtract_pressure_gradient(Fluid *fluid)
{
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            int i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
            int i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
            int j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
            int j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);

            float dp_dr = (fluid->pressure[i_right][j]
                         - fluid->pressure[i_left ][j])
                        * 0.5f * GRID_INV_CELL_SIZE;
            float dp_dy = (fluid->pressure[i][j_above]
                         - fluid->pressure[i][j_below])
                        * 0.5f * GRID_INV_CELL_SIZE;

            fluid->velocity_radial  [i][j] -= dp_dr;
            fluid->velocity_vertical[i][j] -= dp_dy;
        }
    }
}

/* ── §13 projection — orchestrator combining §10..§12 ────────────────── *
 *
 * The Hodge-decomposition step.  Three sub-steps:
 *   1. compute_divergence            (§10) — measure pile-up
 *   2. solve_pressure_poisson        (§11) — find p that fixes it
 *   3. subtract_pressure_gradient    (§12) — apply the fix
 *
 * After this returns, ∇·v ≈ 0 everywhere on the grid.
 */
static void project_to_incompressible(Fluid *fluid)
{
    compute_divergence(fluid);
    solve_pressure_poisson(fluid);
    subtract_pressure_gradient(fluid);
    enforce_velocity_boundaries(fluid);
}

/* ── §14 cool + decay — exponential return to ambient ────────────────── *
 *
 * Tutorial T7 step 6.  Two things every tick:
 *   COOL.  T → T_ambient via Newton cooling (exp decay).
 *   DECAY. ρ fades linearly (models entrainment we don't simulate).
 */
static void cool_and_decay(Fluid *fluid, float step_seconds)
{
    float cool_factor    = expf(-COOL_RATE * step_seconds);
    float density_factor = 1.0f - DENSITY_DECAY * step_seconds;
    if (density_factor < 0.0f) density_factor = 0.0f;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float excess = fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
            fluid->temperature[i][j] = TEMPERATURE_AMBIENT + excess * cool_factor;
            fluid->density    [i][j] *= density_factor;
        }
    }
}

/* ── §15 fluid_step — one full physics tick ──────────────────────────── *
 *
 * Tutorial T7 derived the six-step orchestrator.  Reads top-to-bottom
 * as the algorithm pseudocode.
 */
static void fluid_step(Fluid *fluid, float step_seconds)
{
    /* (1) Buoyancy applies an upward force where T > ambient. */
    apply_buoyancy(fluid, step_seconds);

    /* (2) Project: clean up the divergence buoyancy just added. */
    project_to_incompressible(fluid);

    /* (3) Advect velocity by itself.  Use scratch_a / scratch_b so we
     *     can finish reading both vᵣ and vᵧ before mutating either. */
    advect_field(fluid->scratch_a,
                 fluid->velocity_radial,
                 fluid->velocity_radial,
                 fluid->velocity_vertical,
                 step_seconds);
    advect_field(fluid->scratch_b,
                 fluid->velocity_vertical,
                 fluid->velocity_radial,
                 fluid->velocity_vertical,
                 step_seconds);
    memcpy(fluid->velocity_radial,   fluid->scratch_a, sizeof fluid->velocity_radial);
    memcpy(fluid->velocity_vertical, fluid->scratch_b, sizeof fluid->velocity_vertical);
    enforce_velocity_boundaries(fluid);

    /* (4) Project again (advection re-introduces tiny ∇·v). */
    project_to_incompressible(fluid);

    /* (5) Advect scalars by the (now divergence-free) velocity. */
    advect_field(fluid->scratch_a,
                 fluid->temperature,
                 fluid->velocity_radial,
                 fluid->velocity_vertical,
                 step_seconds);
    memcpy(fluid->temperature, fluid->scratch_a, sizeof fluid->temperature);

    advect_field(fluid->scratch_a,
                 fluid->density,
                 fluid->velocity_radial,
                 fluid->velocity_vertical,
                 step_seconds);
    memcpy(fluid->density, fluid->scratch_a, sizeof fluid->density);

    /* (6) Cool toward ambient; decay density. */
    cool_and_decay(fluid, step_seconds);
}

/* ── §16 detonate — the initial Gaussian bubble ──────────────────────── *
 *
 * Tutorial T8 derived this.  The ONLY scripted moment.  Paint a
 * 3-D-symmetric Gaussian centred at (radius=0, altitude=
 * detonation_altitude), plus a small radial outflow seed.  Everything
 * after this is pure physics.
 */
static void detonate_at_origin(Fluid *fluid, const BlastParameters *blast)
{
    /* Reset all fields to ambient. */
    memset(fluid, 0, sizeof *fluid);
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            fluid->temperature[i][j] = TEMPERATURE_AMBIENT;

    float two_sigma_squared = 2.0f * blast->sigma * blast->sigma;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            /* Cell centre in world coords. */
            float radius   = ((float)i + 0.5f) * GRID_CELL_SIZE;
            float altitude = ((float)j + 0.5f) * GRID_CELL_SIZE;

            /* Distance from blast centre (axis is at radius = 0). */
            float dr = radius;
            float dy = altitude - blast->detonation_altitude;
            float distance_squared = dr * dr + dy * dy;
            float gaussian = expf(-distance_squared / two_sigma_squared);

            /* Paint hot, dense bubble. */
            fluid->temperature[i][j] =
                TEMPERATURE_AMBIENT
              + (blast->peak_temperature - TEMPERATURE_AMBIENT) * gaussian;
            fluid->density[i][j] =
                blast->peak_density * gaussian;

            /* Seed initial radial outflow only NEAR the centre. */
            if (gaussian > 0.05f) {
                float distance = sqrtf(distance_squared);
                if (distance > 1e-3f) {
                    float blast_speed = blast->initial_outflow * gaussian;
                    fluid->velocity_radial  [i][j] = (dr / distance) * blast_speed;
                    fluid->velocity_vertical[i][j] = (dy / distance) * blast_speed;
                }
            }
        }
    }

    enforce_velocity_boundaries(fluid);
}

/* ── §17 sampling — read fluid from world coordinates ────────────────── *
 *
 * Tutorial T10 explained the bridge: world (x, y, z) → field
 * (radius, altitude) → bilinear sample.  These two functions are the
 * entire interface from the renderer to the simulator.
 *
 * Out-of-domain samples return safe defaults: density 0 (no smoke),
 * ambient temperature.
 */
static inline float sample_density_at_world(float radius, float altitude)
{
    if (radius   < 0.0f || radius   >= GRID_RADIAL_EXTENT)   return 0.0f;
    if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT) return 0.0f;
    return sample_field_bilinear(g_fluid.density,
                                 radius   * GRID_INV_CELL_SIZE,
                                 altitude * GRID_INV_CELL_SIZE);
}

static inline float sample_temperature_at_world(float radius, float altitude)
{
    if (radius   < 0.0f || radius   >= GRID_RADIAL_EXTENT)   return TEMPERATURE_AMBIENT;
    if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT) return TEMPERATURE_AMBIENT;
    return sample_field_bilinear(g_fluid.temperature,
                                 radius   * GRID_INV_CELL_SIZE,
                                 altitude * GRID_INV_CELL_SIZE);
}

/* ── §18 raymarch — Beer-Lambert volumetric integration ──────────────── *
 *
 * Tutorial T9 derived the volume rendering equation.  Outputs:
 *   *out_total_luminance  total accumulated brightness (smoke + fire)
 *   *out_hot_luminance    contribution from temperature emission only
 *
 * The hot fraction (L_hot / L_total) drives the smoke→fire palette
 * pick in §20: low fraction = grey/cool smoke, high fraction = bright
 * fire.
 *
 * Optimisation: empty regions skip ahead with a longer step.  Pure-
 * empty rays (which dominate when the cloud is small) cost ~70% less
 * than a uniform march.
 */
static void raymarch_volume(Vec3 origin, Vec3 direction,
                            float *out_total_luminance,
                            float *out_hot_luminance)
{
    float t                       = RM_RAY_NEAR;
    float transmittance           = 1.0f;
    float total_luminance         = 0.0f;
    float hot_luminance           = 0.0f;
    float inverse_temp_range      =
        1.0f / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);

    for (int step = 0; step < RM_MAX_STEPS; step++) {

        Vec3 sample_point = v3add(origin, v3scale(t, direction));

        /* Quick rejects outside the simulation domain. */
        if (sample_point.y < 0.0f || sample_point.y > GRID_VERTICAL_EXTENT) {
            t += RM_STEP * 2.0f;
            if (t > RM_RAY_FAR) break;
            continue;
        }
        float radius = sqrtf(sample_point.x * sample_point.x
                           + sample_point.z * sample_point.z);
        if (radius > GRID_RADIAL_EXTENT) {
            t += RM_STEP * 2.0f;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* Inside the domain — sample density. */
        float density = sample_density_at_world(radius, sample_point.y);
        if (density < 0.001f) {
            /* Empty cell — advance with a normal step, no contribution. */
            t += RM_STEP;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* Density is nontrivial — read temperature too. */
        float temperature = sample_temperature_at_world(radius, sample_point.y);

        /* Optical depth for this segment. */
        float optical_depth_step = density * RM_STEP * RM_DENSITY_GAIN;

        /* Emission factor (0 = ambient, 1 = peak hot). */
        float emission = (temperature - TEMPERATURE_AMBIENT) * inverse_temp_range;
        emission = clampf(emission, 0.0f, 1.0f);

        float source = emission * RM_EMISSION_GAIN + RM_AMBIENT_FLOOR;
        total_luminance += transmittance * optical_depth_step * source;
        hot_luminance   += transmittance * optical_depth_step
                                         * emission * RM_EMISSION_GAIN;

        /* Beer-Lambert decay. */
        transmittance *= expf(-optical_depth_step);
        if (transmittance < RM_OPAQUE_TRANSMITTANCE_EPS) break;
        if (t > RM_RAY_FAR) break;

        t += RM_STEP;
    }

    *out_total_luminance = total_luminance;
    *out_hot_luminance   = hot_luminance;
}

/* ── §19 camera — orthonormal basis + per-pixel ray ──────────────────── *
 *
 * Camera at (x=0, y=CAM_HEIGHT, z=−distance) looking at (0,
 * CAM_LOOK_AT_HEIGHT, 0).  Three orthonormal vectors define the view:
 *      forward = look_at − cam_position, normalised
 *      right   = forward × world_up, normalised
 *      up      = right × forward
 *
 * Per pixel ray: forward + u·tan(FOV/2)·right + v·tan(FOV/2)·aspect·up.
 * The aspect factor compensates for terminal cells being ~2× as tall
 * as they are wide.
 */
typedef struct {
    Vec3  origin;
    Vec3  forward;
    Vec3  right;
    Vec3  up;
    float fov_tangent;
    float aspect_factor;
} Camera;

static Camera build_camera_basis(float distance_behind,
                                 int   visible_rows,
                                 int   visible_cols)
{
    Camera cam;
    cam.origin   = v3(0.0f, CAM_HEIGHT, -distance_behind);
    Vec3 look_at = v3(0.0f, CAM_LOOK_AT_HEIGHT, 0.0f);

    cam.forward = v3normalise(v3sub(look_at, cam.origin));
    Vec3 world_up = v3(0.0f, 1.0f, 0.0f);
    cam.right     = v3normalise(v3cross(cam.forward, world_up));
    cam.up        = v3cross(cam.right, cam.forward);

    cam.fov_tangent   = tanf(CAM_FIELD_OF_VIEW_DEG * (float)M_PI / 180.0f * 0.5f);
    cam.aspect_factor =
        ((float)visible_rows * TERMINAL_CELL_ASPECT) / (float)visible_cols;

    return cam;
}

static Vec3 ray_for_pixel(int col, int row, int cols, int visible_rows,
                          const Camera *cam)
{
    float u =  ((float)col + 0.5f) / (float)cols          * 2.0f - 1.0f;
    float v = -(((float)row + 0.5f) / (float)visible_rows * 2.0f - 1.0f);

    Vec3 ray = v3add(cam->forward,
        v3add(v3scale(u * cam->fov_tangent,                    cam->right),
              v3scale(v * cam->fov_tangent * cam->aspect_factor, cam->up)));
    return v3normalise(ray);
}

/* ── §20 cell decoration — (lum, hot) → glyph + colour ───────────────── *
 *
 * Two scalars per pixel from the raymarcher:
 *   total_luminance   how bright (smoke + fire combined)
 *   hot_luminance     how much of that came from fire
 *
 * Mapping:
 *   GLYPH from total_luminance via 8-tier ramp ('.' → '@').
 *   COLOUR from hot_fraction = hot_luminance / total_luminance.
 *          Cool pixel → ramp slot 0 (smoke colour).
 *          Hot pixel  → ramp slot 7 (fire colour).
 *
 * Decoupled signals: glyph reads as DENSITY, colour reads as
 * TEMPERATURE.  A thin wisp of cool smoke and the bright core of a
 * fireball look visually distinct even at the same brightness.
 */
typedef struct {
    char   glyph;
    int    pair;
    attr_t attr;
    bool   skip;
} Cell;

static int to_slot(float value_01)
{
    int slot = (int)(value_01 * GLYPH_SLOT_FLOAT);
    if (slot < 0)                  slot = 0;
    if (slot >= GLYPH_SLOT_COUNT)  slot = GLYPH_SLOT_COUNT - 1;
    return slot;
}

static Cell decorate_volume_pixel(float total_luminance, float hot_luminance,
                                  bool   inverted_theme)
{
    /* Empty cell — left as default-bg / pre-painted bg. */
    if (total_luminance < PIXEL_VISIBLE_LUM_EPS) {
        return (Cell){ .skip = true };
    }

    /* Glyph from luminance (clamped). */
    float lum_normalised = total_luminance / PIXEL_LUMINANCE_CLAMP;
    if (lum_normalised > 1.0f) lum_normalised = 1.0f;
    int slot_lum = to_slot(lum_normalised);

    /* Colour from hot fraction. */
    float hot_fraction = hot_luminance / (total_luminance + 0.001f);
    if (hot_fraction > 1.0f) hot_fraction = 1.0f;
    if (hot_fraction < 0.0f) hot_fraction = 0.0f;
    int slot_hot = to_slot(hot_fraction);

    /* Attribute modulation:
     *   normal themes  — A_BOLD on hot/bright tiers (extra fire punch)
     *                    A_DIM on cool tiers (fading wisps)
     *   inverted theme — A_NORMAL only.  A_DIM on a light fg over white
     *                    bg DARKENS it, inverting brightness intent.
     */
    attr_t attr;
    if (inverted_theme) {
        attr = A_NORMAL;
    } else {
        attr = (slot_hot >= 6 || slot_lum >= 6) ? A_BOLD
             : (slot_lum  <= 1)                 ? A_DIM
             :                                    A_NORMAL;
    }

    return (Cell){
        .glyph = LUMINANCE_GLYPHS[slot_lum],
        .pair  = PAIR_RAMP_BASE + slot_hot,
        .attr  = attr,
        .skip  = false,
    };
}

/* emit_cell — paint one cell with attron/attroff batched on (pair,
 * attr) change.  Halves attribute thrash on uniform regions. */
static void emit_cell(int row, int col, Cell cell,
                      int *last_pair, attr_t *last_attr)
{
    if (cell.skip) return;

    if (cell.pair != *last_pair || cell.attr != *last_attr) {
        if (*last_pair >= 0) attroff(COLOR_PAIR(*last_pair) | *last_attr);
        attron(COLOR_PAIR(cell.pair) | cell.attr);
        *last_pair = cell.pair;
        *last_attr = cell.attr;
    }
    mvaddch(row, col, (chtype)(unsigned char)cell.glyph);
}

/* ── §21 scene — the application's per-frame state ───────────────────── *
 *
 * Everything that's not "in the fluid" lives here: which blast,
 * which theme, sim time elapsed, camera distance, debug overlay.
 */
typedef struct {
    bool       paused;
    int        theme_index;
    BlastType  blast_type;
    DebugMode  debug_mode;
    int        cols, rows;

    float      simulation_time_seconds;
    float      simulation_rate;             /* real → sim multiplier */
    float      simulation_step_accumulator;
    float      camera_distance;
} Scene;

static void scene_init(Scene *scene, int cols, int rows)
{
    memset(scene, 0, sizeof *scene);
    scene->paused          = false;
    scene->theme_index     = 0;
    scene->blast_type      = BLAST_STANDARD;
    scene->debug_mode      = DEBUG_NORMAL;
    scene->cols            = cols;
    scene->rows            = rows;
    scene->simulation_time_seconds      = 0.0f;
    scene->simulation_rate              = SIM_RATE_DEFAULT;
    scene->simulation_step_accumulator  = 0.0f;
    scene->camera_distance              = CAM_DISTANCE_DEFAULT;

    detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_resize(Scene *scene, int cols, int rows)
{
    scene->cols = cols;
    scene->rows = rows;
}

static void scene_redetonate(Scene *scene)
{
    scene->simulation_time_seconds     = 0.0f;
    scene->simulation_step_accumulator = 0.0f;
    detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_cycle_blast(Scene *scene, int direction)
{
    int new_index = (int)scene->blast_type + direction;
    while (new_index < 0) new_index += BLAST_TYPE_COUNT;
    scene->blast_type = (BlastType)(new_index % BLAST_TYPE_COUNT);
    scene_redetonate(scene);
}

/*
 * scene_tick — accumulate real time into sim time and run as many
 * fixed-dt fluid steps as fit into the accumulator.  Fixed-dt
 * accumulator pattern keeps cloud morphology FRAME-RATE-INDEPENDENT.
 */
static void scene_tick(Scene *scene, float dt_real_seconds)
{
    if (scene->paused) return;

    float dt_sim = dt_real_seconds * scene->simulation_rate;
    if (dt_sim > SIM_DT_MAX_REAL) dt_sim = SIM_DT_MAX_REAL;

    scene->simulation_step_accumulator += dt_sim;
    scene->simulation_time_seconds     += dt_sim;

    while (scene->simulation_step_accumulator >= SIM_DT) {
        fluid_step(&g_fluid, SIM_DT);
        scene->simulation_step_accumulator -= SIM_DT;
    }
}

/* §21.1 — main raymarched render path. */
static void render_volume_view(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_ROWS;
    if (visible_rows < 1) return;

    Camera cam = build_camera_basis(scene->camera_distance,
                                    visible_rows, scene->cols);

    bool inverted = THEMES[scene->theme_index].inverted_background;

    /* For inverted themes, pre-fill the visible region with white. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE));
        for (int row = 0; row < visible_rows; row++)
            for (int col = 0; col < scene->cols; col++)
                mvaddch(row + 1, col, ' ');
        attroff(COLOR_PAIR(PAIR_RAMP_BASE));
    }

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;
    int    y_offset  = 1;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            Vec3  ray = ray_for_pixel(col, row, scene->cols, visible_rows, &cam);
            float total_luminance, hot_luminance;
            raymarch_volume(cam.origin, ray, &total_luminance, &hot_luminance);

            Cell cell = decorate_volume_pixel(total_luminance, hot_luminance,
                                              inverted);
            emit_cell(y_offset + row, col, cell, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §22 debug overlays — see the raw fluid fields ───────────────────── *
 *
 * Each overlay swaps out the volumetric raymarcher for a DIRECT view
 * of one underlying field.  Walks the 2-D fluid grid mapped to
 * terminal cells (with a y-flip so ground stays at the bottom of the
 * screen).
 */

static void render_debug_density(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_ROWS;
    if (visible_rows < 1) return;

    int last_pair = -1;
    attr_t last_attr = 0;
    int y_offset = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float density = g_fluid.density[grid_i][grid_j];

            float normalised = density / DENSITY_PEAK;
            normalised = clampf(normalised, 0.0f, 1.0f);
            int   slot = to_slot(normalised);
            char  glyph = LUMINANCE_GLYPHS[slot];
            int   pair  = PAIR_RAMP_BASE + slot;

            if (slot == 0 && density < 0.01f) continue;

            Cell c = { .glyph = glyph, .pair = pair, .attr = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void render_debug_temperature(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_ROWS;
    if (visible_rows < 1) return;

    int last_pair = -1;
    attr_t last_attr = 0;
    int y_offset = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float temperature = g_fluid.temperature[grid_i][grid_j];

            float normalised = (temperature - TEMPERATURE_AMBIENT)
                             / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);
            normalised = clampf(normalised, 0.0f, 1.0f);
            int   slot  = to_slot(normalised);
            char  glyph = LUMINANCE_GLYPHS[slot];
            int   pair  = PAIR_RAMP_BASE + slot;

            if (slot == 0 && normalised < 0.01f) continue;

            Cell c = { .glyph = glyph, .pair = pair, .attr = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/*
 * arrow_for_velocity — pick an ASCII arrow character that points in
 * the same direction as a velocity vector.  8 cardinal/intercardinal
 * directions; magnitude near zero → space.
 */
static char arrow_for_velocity(float vx, float vy)
{
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 0.05f) return ' ';

    float angle_radians = atan2f(vy, vx);
    int   octant        = (int)((angle_radians + (float)M_PI) /
                                ((float)M_PI / 4.0f) + 0.5f) % 8;

    static const char ARROWS[8] = {
        '<',     /* 0 = west  */
        '/',     /* 1 = sw    */
        'v',     /* 2 = south */
        '\\',    /* 3 = se    */
        '>',     /* 4 = east  */
        '/',     /* 5 = ne    */
        '^',     /* 6 = north */
        '\\',    /* 7 = nw    */
    };
    return ARROWS[octant];
}

static void render_debug_velocity(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_ROWS;
    if (visible_rows < 1) return;

    int last_pair = -1;
    attr_t last_attr = 0;
    int y_offset = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float vr = g_fluid.velocity_radial  [grid_i][grid_j];
            float vy = g_fluid.velocity_vertical[grid_i][grid_j];

            char  glyph = arrow_for_velocity(vr, vy);
            if (glyph == ' ') continue;

            float magnitude = sqrtf(vr*vr + vy*vy);
            float normalised = clampf(magnitude / 4.0f, 0.0f, 1.0f);
            int   slot  = to_slot(normalised);
            int   pair  = PAIR_RAMP_BASE + slot;

            Cell c = { .glyph = glyph, .pair = pair,
                       .attr = (slot >= 5) ? A_BOLD : A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void render_active_view(const Scene *scene)
{
    switch (scene->debug_mode) {
    case DEBUG_NORMAL:      render_volume_view     (scene); break;
    case DEBUG_DENSITY:     render_debug_density   (scene); break;
    case DEBUG_TEMPERATURE: render_debug_temperature(scene); break;
    case DEBUG_VELOCITY:    render_debug_velocity  (scene); break;
    default:                render_volume_view     (scene); break;
    }
}

/* ── §23 screen — ncurses init, HUD, present ─────────────────────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *screen)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_free(Screen *screen) { (void)screen; endwin(); }

static void screen_resize(Screen *screen)
{
    endwin(); refresh();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

/* HUD layout (CLAUDE.md spec):
 *   row 0          PAIR_HUD_STATUS (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HUD_HINT   (cyan   + bold) — key hint */
static void hud_draw(const Screen *screen, const Scene *scene,
                     double real_fps, int sim_target_fps)
{
    char status_text[200];
    snprintf(status_text, sizeof status_text,
             " %5.1f fps  %3d Hz  blast:%s  theme:%s  debug:%s  "
             "t:%6.2fs  rate:%4.2f  dist:%4.1f  %s ",
             real_fps, sim_target_fps,
             BLAST_PRESETS[scene->blast_type].display_name,
             THEMES[scene->theme_index].display_name,
             DEBUG_MODE_NAMES[scene->debug_mode],
             (double)scene->simulation_time_seconds,
             (double)scene->simulation_rate,
             (double)scene->camera_distance,
             scene->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status_text);
    if (slen > screen->cols) slen = screen->cols;

    attron(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);
    mvprintw(0, screen->cols - slen, "%s", status_text);
    mvprintw(0, 0, " NUKE · FLUID ");
    attroff(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
    mvprintw(screen->rows - 1, 0,
             " q:quit  spc:pause  r:detonate  n/N:blast  t/T:theme  "
             "d/D:debug  +/-:rate  z/Z:zoom ");
    attroff(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
}

static void screen_draw(Screen *screen, const Scene *scene,
                        double real_fps, int sim_target_fps)
{
    erase();
    render_active_view(scene);
    hud_draw(screen, scene, real_fps, sim_target_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §24 app — main loop, signals, key handling ──────────────────────── */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   target_render_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup         (void)    { endwin(); }

static void app_handle_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize (&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *scene = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           scene->paused = !scene->paused;       break;
    case 'r': case 'R': scene_redetonate(scene);              break;

    case 'n': scene_cycle_blast(scene, +1); break;
    case 'N': scene_cycle_blast(scene, -1); break;

    case 't':
        scene->theme_index = (scene->theme_index + 1) % THEME_COUNT;
        apply_theme(scene->theme_index);
        break;
    case 'T':
        scene->theme_index = (scene->theme_index + THEME_COUNT - 1) % THEME_COUNT;
        apply_theme(scene->theme_index);
        break;

    case 'd':
        scene->debug_mode = (DebugMode)((scene->debug_mode + 1) % DEBUG_MODE_COUNT);
        break;
    case 'D':
        scene->debug_mode =
            (DebugMode)((scene->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
        break;

    case '=': case '+':
        scene->simulation_rate *= SIM_RATE_STEP_FACTOR;
        if (scene->simulation_rate > SIM_RATE_MAX) scene->simulation_rate = SIM_RATE_MAX;
        break;
    case '-':
        scene->simulation_rate /= SIM_RATE_STEP_FACTOR;
        if (scene->simulation_rate < SIM_RATE_MIN) scene->simulation_rate = SIM_RATE_MIN;
        break;

    case 'z':
        scene->camera_distance -= CAM_DISTANCE_STEP;
        if (scene->camera_distance < CAM_DISTANCE_MIN)
            scene->camera_distance = CAM_DISTANCE_MIN;
        break;
    case 'Z':
        scene->camera_distance += CAM_DISTANCE_STEP;
        if (scene->camera_distance > CAM_DISTANCE_MAX)
            scene->camera_distance = CAM_DISTANCE_MAX;
        break;

    case ']':
        app->target_render_fps += TARGET_RENDER_FPS_STEP;
        if (app->target_render_fps > TARGET_RENDER_FPS_MAX)
            app->target_render_fps = TARGET_RENDER_FPS_MAX;
        break;
    case '[':
        app->target_render_fps -= TARGET_RENDER_FPS_STEP;
        if (app->target_render_fps < TARGET_RENDER_FPS_MIN)
            app->target_render_fps = TARGET_RENDER_FPS_MIN;
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

    App *app                 = &g_app;
    app->running             = 1;
    app->target_render_fps   = TARGET_RENDER_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_start_ns       = clock_now_nanoseconds();
    int64_t fps_window_ns        = 0;
    int     frames_in_window     = 0;
    double  measured_fps         = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_handle_resize(app);
            frame_start_ns = clock_now_nanoseconds();
        }

        int64_t now_ns        = clock_now_nanoseconds();
        int64_t dt_ns         = now_ns - frame_start_ns;
        frame_start_ns        = now_ns;
        if (dt_ns > 100 * NS_PER_MILLISECOND) dt_ns = 100 * NS_PER_MILLISECOND;
        float   dt_real_seconds = (float)dt_ns / (float)NS_PER_SECOND;

        scene_tick(&app->scene, dt_real_seconds);

        frames_in_window++;
        fps_window_ns += dt_ns;
        if (fps_window_ns >= FPS_DISPLAY_UPDATE_MS * NS_PER_MILLISECOND) {
            measured_fps = (double)frames_in_window
                         / ((double)fps_window_ns / (double)NS_PER_SECOND);
            frames_in_window = 0;
            fps_window_ns    = 0;
        }

        int64_t target_frame_ns = TICK_NS(app->target_render_fps);
        int64_t elapsed_ns      = clock_now_nanoseconds() - frame_start_ns + dt_ns;
        clock_sleep_nanoseconds(target_frame_ns - elapsed_ns);

        screen_draw   (&app->screen, &app->scene,
                       measured_fps, app->target_render_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
