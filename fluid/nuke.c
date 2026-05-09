/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke.c — a mushroom cloud rising from a Stam stable-fluid simulation
 *
 * DEMO: Detonate a Gaussian "blast" at the origin.  A 2-D AXISYMMETRIC
 *       fluid simulation runs underneath: hot gas rises by buoyancy,
 *       advects through the velocity field, cools toward ambient.
 *       A VOLUMETRIC RAYMARCHER renders the 2-D slice back into 3-D
 *       for the screen by exploiting the cloud's rotational symmetry
 *       around the y-axis.  Five blast presets, six themes (one of
 *       them photographic-negative), four debug overlays that show
 *       the raw fluid fields.
 *
 *       Two simulators stitched together by ONE NARROW INTERFACE:
 *
 *           sample_density_at_world(radius, altitude)
 *           sample_temperature_at_world(radius, altitude)
 *
 *       The fluid solver writes 2-D fields.  The renderer reads them
 *       at fractional cell coordinates derived from 3-D world (x, y,
 *       z) via the axisymmetric reduction (T7).  Neither side knows
 *       about the other's data structures.
 *
 *       The simulation runs Stam's "Stable Fluids" cycle:
 *
 *           BUOYANCY  →  PROJECT  →  ADVECT v  →  PROJECT  →
 *           ADVECT (T, ρ)  →  COOL & DECAY
 *
 *       Six steps per tick.  Each is unconditionally stable; the
 *       composition is too.  Crank dt — the cloud blurs, never blows.
 *
 * Study alongside:
 *   fluid/navier_stokes.c    — sibling Stam stable-fluid solver, but
 *                               on a Cartesian grid driven by user
 *                               forces / dye instead of buoyancy.
 *                               Same 4-phase advect/project pattern.
 *   raymarcher/raymarcher.c  — surface raymarching of an SDF.  This
 *                               file does VOLUME raymarching instead
 *                               (Beer-Lambert), a different beast (T8).
 *   fluid/fluid_sph.c        — Lagrangian particle alternative to
 *                               grid solvers.  Useful comparison.
 *
 * Section map:
 *   §1   config            — every tunable, every enum, no later magic
 *   §2   clock             — monotonic timer + sleep
 *   §3   vec3              — 3-D vector math (used only by raymarcher)
 *   §4   grid_helpers      — clampf, mirror_index, to_slot, bilinear
 *   §5   themes            — 6 colour palettes
 *   §6   colors            — pair init + theme apply
 *   §7   fluid_state       — the 8-field struct + global instance
 *   §8   boundaries        — wall conditions on the velocity field
 *   §9   buoyancy          — hot air rises
 *   §10  advect            — semi-Lagrangian back-trace
 *   §11  project           — divergence + Jacobi + gradient (one §)
 *   §12  cool_decay        — Newton cooling + density attenuation
 *   §13  fluid_step        — full physics tick (six phases)
 *   §14  detonate          — initial Gaussian bubble (only scripted bit)
 *   §15  sampling          — world (x, y, z) → bilinear 2-D field
 *   §16  raymarch          — Beer-Lambert volumetric integration
 *   §17  camera            — orthonormal basis + per-pixel ray
 *   §18  cell_decorate     — (lum, hot) → glyph + colour + attr
 *   §19  render_volume     — full screen of raymarched cells
 *   §20  debug_overlays    — three direct-field views
 *   §21  render_dispatch   — pick volume vs debug per active mode
 *   §22  hud               — top status + bottom hint (CLAUDE.md spec)
 *   §23  screen            — ncurses init / cleanup / present
 *   §24  scene             — per-frame state + tick + scene helpers
 *   §25  app               — main loop + signals + key handling
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r / R            re-detonate (restart from t = 0)
 *   n / N            next / prev blast preset
 *                      (TACTICAL / STANDARD / MEGATON / AIR_BURST / GROUND)
 *   t / T            next / prev theme
 *                      (REALISTIC / MATRIX / OCEAN / NOVA / TOXIC / NEGATIVE)
 *   d / D            cycle debug overlay
 *                      (NORMAL / DENSITY / TEMP / VELOCITY)
 *   + / -            simulation rate up / down
 *   z / Z            camera closer / farther
 *   ] / [            render fps cap up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/nuke.c -o nuke -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Tutorials are a LADDER: T1 sets the worldview, T2-T6 build the
 *      fluid solver, T7 explains the 2-D-to-3-D bridge, T8-T10 build
 *      the volumetric renderer.
 *   2. §1 config — see all the tunables labelled.  Hint at scope.
 *   3. §7 fluid_state — the data structure.  Eight 2-D arrays in one
 *      struct; everything else manipulates these.
 *   4. §13 fluid_step — six lines of pseudocode.  The whole solver.
 *   5. §9 buoyancy → §10 advect → §11 project → §12 cool_decay —
 *      one § per phase, in the order fluid_step calls them.  Each
 *      pairs with one tutorial (T2, T4, T5, T6).
 *   6. §15 sampling + §16 raymarch + §17 camera — the renderer.
 *      Pairs with T7-T10.
 *   7. §22-§25 — orchestration.  Standard framework code.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   velocity_radial, velocity_vertical    vᵣ, vᵧ on the axisymmetric grid
 *   temperature, density                  T, ρ — passive scalars
 *   pressure, divergence                  scratch for projection (T5)
 *   scratch_a, scratch_b                  generic Jacobi/advection scratch
 *
 *   GRID_RADIAL_CELLS, GRID_VERTICAL_CELLS    grid resolution
 *   GRID_CELL_SIZE                            world-space cell side
 *   GRID_INV_CELL_SIZE                        1 / GRID_CELL_SIZE
 *
 *   simulation_time_seconds                   sim time elapsed (read in HUD)
 *   simulation_step_accumulator               fixed-dt accumulator
 *   simulation_rate                           real → sim time multiplier
 *
 *   total_luminance, hot_luminance            raymarcher per-pixel outputs
 *
 * Background you need
 * ───────────────────
 *   - Vectors in 3-D: dot, cross, normalise.  §3 vec3 has these.
 *   - Familiarity with ∇· (divergence) and ∇p (gradient) helpful but
 *     not required — T2 and T5 introduce them informally.
 *   - Read raymarcher/raymarcher.c first if SDF surface tracing is
 *     unfamiliar; volume raymarching (T8) is a different mode but
 *     reuses the per-pixel ray geometry.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real combustion physics, Mach disks, fireball thermodynamics.
 *     We model heat as a passive scalar that lifts under buoyancy.
 *   - 3-D fluid math.  The simulator is 2-D axisymmetric (T7).
 *   - Compressible flow / shock waves.  Initial radial outflow is
 *     scripted in detonate(), not simulated.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm    : Stam's Stable Fluids (1999) on a 2-D AXISYMMETRIC
 *                grid (radius × altitude), driven by BUOYANCY from a
 *                Gaussian temperature bubble.  Volumetric raymarching
 *                renders the 2-D slice back into 3-D by exploiting
 *                rotational symmetry around the y-axis.
 *
 *                Per fluid tick:
 *                   1. apply buoyancy           (hot → upward push)
 *                   2. project                  (zero ∇·v)
 *                   3. advect velocity by itself
 *                   4. project again            (advect re-introduces ∇·v)
 *                   5. advect temperature + density
 *                   6. cool + decay             (return to ambient)
 *
 *                Per render frame:
 *                   7. ray gen   (orthonormal camera basis, per-pixel ray)
 *                   8. raymarch  (Beer-Lambert volumetric integration)
 *                   9. decorate  ((lum, hot) → glyph + theme colour)
 *                  10. emit      (mvaddch with batched attr changes)
 *
 * Math basis   : Hodge decomposition theorem says any vector field
 *                can be split as v = v_div_free + ∇φ.  PROJECTION
 *                extracts v_div_free by solving the Poisson equation
 *                ∇²φ = ∇·v (Jacobi iteration, 40 sweeps), then
 *                subtracting ∇φ from v.  Buoyancy uses the Boussinesq
 *                approximation: vertical accel ∝ (T - T_ambient).
 *                Volumetric rendering uses the Beer-Lambert law for
 *                light attenuation through a participating medium
 *                with self-emission proportional to temperature.
 *
 * Performance  : Eight 2-D float arrays, total ~168 KB, all in BSS;
 *                no malloc anywhere.  Per render frame: ~80 cols ×
 *                ~22 rows × ~130 march steps ≈ 230K samples.  Per
 *                fluid tick: ~5K cells × 40 Jacobi sweeps × 2
 *                projections ≈ 430K cell updates.  Comfortable
 *                60 fps on any modern CPU.
 *
 * References
 * ──────────
 *   Stam, J. (1999), "Stable Fluids," SIGGRAPH '99 — the foundational
 *     paper, introduces semi-Lagrangian advection + projection.
 *   Stam, J. (2003), "Real-Time Fluid Dynamics for Games," GDC.
 *     Simpler exposition with reference C code.
 *   Quílez, I., "Volumetric raymarching":
 *     https://iquilezles.org/articles/raymarchingvolumes/
 *     — the Beer-Lambert volume integration loop in §16.
 *   Bridson, R. (2008), "Fluid Simulation for Computer Graphics" —
 *     standard textbook; §1-4 cover everything in this file.
 *   https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * TWO SIMULATORS run side-by-side: a 2-D fluid solver tracking how
 * hot gas moves through space, and a 3-D volumetric raymarcher that
 * INFLATES the 2-D slice back into a 3-D image.  The fluid never
 * knows about pixels.  The renderer never knows about pressure.
 * Both meet at one narrow interface: sample_density_at_world() and
 * sample_temperature_at_world(), which convert a world-space (x, y,
 * z) point to a (radius, altitude) pair and bilinearly sample the
 * 2-D grid.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a slowly-evolving photograph of a smoke column.  The
 * photograph is 2-D — a thin slice through the column.  Because real
 * mushroom clouds have rotational symmetry around their vertical
 * axis, a 3-D reconstruction is just "rotate the slice around the
 * y-axis."  Every camera ray, after tracing through this rotational
 * extrusion of the slice, accumulates light by Beer-Lambert: hot
 * cells emit, dense cells absorb.  The 2-D slice itself evolves
 * under Stam's stable-fluid rules: hot cells push up, the velocity
 * field is cleaned to be divergence-free, then everything advects
 * with the flow and cools toward ambient.
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
 * One render frame (volumetric raymarch):
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   for each pixel:                                       │
 *      │     ray ← (camera_origin, ray_direction)                │
 *      │     transmittance = 1; total = hot = 0                  │
 *      │     for step = 0..MAX_STEPS:                            │
 *      │       point = origin + t · direction                    │
 *      │       (radius, altitude) ← (√(x²+z²), y)                │
 *      │       (ρ, T) ← bilinear from 2-D grid                   │
 *      │       dτ = ρ · STEP · DENSITY_GAIN                      │
 *      │       emit = clamp((T-T_amb)/(T_peak-T_amb), 0, 1)       │
 *      │       total += transmittance · dτ · (emit·EM + AMB)     │
 *      │       hot   += transmittance · dτ · emit · EM           │
 *      │       transmittance *= exp(-dτ)                         │
 *      │       if transmittance < ε: break                       │
 *      │     cell ← decorate(total, hot)                         │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   BUOYANCY (Boussinesq approximation):
 *     vᵧ_new = vᵧ_old + β · (T − T_ambient) · dt
 *
 *   SEMI-LAGRANGIAN ADVECTION (Stam):
 *     for each cell (i, j):
 *       (i_back, j_back) = (i, j) − v[i, j] · dt / h
 *       new_field[i, j]  = bilinear(old_field, i_back, j_back)
 *
 *   DIVERGENCE (centred difference):
 *     ∇·v[i, j] = (vᵣ[i+1, j] − vᵣ[i−1, j]) / (2h)
 *               + (vᵧ[i, j+1] − vᵧ[i, j−1]) / (2h)
 *
 *   POISSON FOR PRESSURE (Jacobi update):
 *     p_new[i, j] = ( p[i+1, j] + p[i−1, j] + p[i, j+1] + p[i, j−1]
 *                     − h² · ∇·v[i, j] ) / 4
 *
 *   GRADIENT SUBTRACTION (closes the projection):
 *     vᵣ_clean[i, j] = vᵣ[i, j] − (p[i+1, j] − p[i−1, j]) / (2h)
 *     vᵧ_clean[i, j] = vᵧ[i, j] − (p[i, j+1] − p[i, j−1]) / (2h)
 *
 *   NEWTON COOLING:
 *     T_new = T_ambient + (T_old − T_ambient) · exp(−k_cool · dt)
 *
 *   AXISYMMETRIC LIFT (3-D world → 2-D grid):
 *     radius   = √(x² + z²)
 *     altitude = y
 *     (ρ, T)   = bilinear(2-D field, radius/h, altitude/h)
 *
 *   BEER-LAMBERT VOLUME INTEGRATION (per ray, per step):
 *     dτ              = ρ · STEP · DENSITY_GAIN
 *     emit            = clamp((T − T_amb) / (T_peak − T_amb), 0, 1)
 *     source          = emit · EMISSION_GAIN + AMBIENT_FLOOR
 *     total_lum      += transmittance · dτ · source
 *     hot_lum        += transmittance · dτ · emit · EMISSION_GAIN
 *     transmittance  *= exp(−dτ)
 *
 *   PIXEL DECORATION:
 *     glyph_slot = floor(total_lum / LUM_CLAMP · 8)         ∈ [0, 7]
 *     hot_slot   = floor(hot_lum / total_lum · 8)            ∈ [0, 7]
 *     glyph      = ".,:;+*#@"[glyph_slot]
 *     pair       = PAIR_RAMP_BASE + hot_slot
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - WHY TWO PROJECTIONS per fluid step?  Buoyancy adds vertical
 *     impulse → divergence.  Project to clean.  Then advection on a
 *     divergence-free field RE-INTRODUCES tiny ∇·v (bilinear sampling
 *     is not exactly volume-preserving).  Project again before
 *     scalar advection.  Without the second project, the cloud
 *     subtly drifts over time.
 *
 *   - JACOBI ITERATION COUNT (40) is tuned for visual quality at
 *     this grid size.  Below ~25 iters you see "puff" artefacts
 *     where residual divergence pushes scalars around.  Above 40
 *     gives diminishing returns at significant CPU cost.
 *
 *   - VELOCITY ADVECTION needs DOUBLE BUFFERING.  vᵣ and vᵧ are
 *     advected by the SAME field (themselves), so we can't overwrite
 *     vᵣ before reading it for vᵧ's advection.  scratch_a and
 *     scratch_b are the destination buffers; memcpy back at the end.
 *
 *   - AXISYMMETRIC reduces simulation cost by ~50× vs full 3-D, but
 *     loses any non-axisymmetric structure.  No tilted column, no
 *     wind shear, no off-centre cap.  Trade-off for terminal demo.
 *
 *   - CAM_DISTANCE_MIN (8.0) is just outside the typical MEGATON
 *     cap.  Pushing closer puts the camera INSIDE the cloud — the
 *     screen fills with smoke colour.
 *
 *   - NEGATIVE THEME uses a white background.  A_BOLD/A_DIM are
 *     disabled for it (see decorate_volume_pixel) because they
 *     INVERT their visual meaning against a light bg — bold becomes
 *     "more saturated", dim becomes "harder to see colour", neither
 *     of which is the brightness signal we want.
 *
 *   - SIM_DT_MAX_REAL (80 ms) caps per-frame sim advance.  Without
 *     it, a hiccup that delays the loop 1 s would advance 1 s of sim
 *     time in one frame — the cloud teleports to a much later state.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - At t = 0 the screen shows a hint of glow at altitude ≈
 *     blast.detonation_altitude.  Within ~2 sim seconds a clear
 *     STEM forms; CAP appears around 4-6 seconds; cloud levels off
 *     around 10-15 seconds.
 *   - Press 'n' to cycle blast presets:
 *       TACTICAL  — small, low-altitude, brief.
 *       STANDARD  — canonical mid-yield mushroom.
 *       MEGATON   — huge, very tall column, long-lasting cap.
 *       AIR_BURST — high-altitude, no ground stem.
 *       GROUND    — surface burst, wide base, heavy dust.
 *   - Press 'd' to cycle debug overlays.  DENSITY shows the 2-D
 *     density slice directly (no rendering).  TEMP shows heat —
 *     watch buoyancy push hot cells upward.  VELOCITY shows the
 *     flow field as ASCII arrows.
 *   - Press 't' through all 6 themes.  Geometry stays identical;
 *     only the smoke→fire colour ramp changes.
 *   - Press 'r' to re-detonate.  Same physics replays — deterministic.
 *   - Press 'z' to zoom in.  At minimum distance you can resolve
 *     individual "puff" cells.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1   What is a fluid simulation?
 *   T2   Buoyancy — Boussinesq approximation
 *   T3   Operator splitting — divide and conquer
 *   T4   Semi-Lagrangian advection — backward trace
 *   T5   Pressure projection — Hodge decomposition + Jacobi
 *   T6   Cooling and density decay
 *   T7   Axisymmetric reduction — 2-D sim, 3-D render
 *   T8   Volumetric raymarching — Beer-Lambert
 *   T9   Camera + ray generation
 *   T10  Pixel decoration — decoupled glyph and colour signals
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS A FLUID SIMULATION?
 * ───────────────────────────────
 * A grid of cells, each storing some QUANTITIES that vary over space:
 *
 *      VELOCITY    how fast and which way the fluid is moving
 *      DENSITY     how much "stuff" (smoke) is in this cell
 *      TEMPERATURE how hot is the fluid here
 *      PRESSURE    internal force (used to enforce mass conservation)
 *
 * Each tick of simulation:
 *   - stuff MOVES according to the velocity (advection, T4)
 *   - velocity CHANGES according to forces (buoyancy, pressure, T2/T5)
 *   - temperature changes via its own laws (cooling, T6)
 *
 * Run this loop for thousands of ticks → an evolving fluid.  Render
 * the density + temperature each frame → a movie.
 *
 * Compare with PARTICLE methods (fluid_sph.c, lattice_gas.c): they
 * track DISCRETE PARTICLES that move with the flow.  Eulerian (this
 * file) tracks the FIELD on a fixed grid; Lagrangian (SPH) tracks
 * particles.  Trade-offs are well known: Eulerian wins on pressure
 * / incompressibility (one Poisson solve, T5); Lagrangian wins on
 * free surfaces and splashes.  We're modelling a smoke column —
 * Eulerian fits.
 *
 * T2  BUOYANCY — BOUSSINESQ APPROXIMATION
 * ───────────────────────────────────────
 * Hot gas is less dense than cold gas at the same pressure, so cold
 * surrounding air pushes the hot stuff up.  Real buoyancy is mass-
 * dependent and intricate; the BOUSSINESQ APPROXIMATION says we can
 * treat density variations as small (so mass is constant) and put
 * the temperature dependence ONLY into the buoyancy force:
 *
 *      acceleration_y = β · (T − T_ambient) · g
 *
 * where β is the thermal expansion coefficient and g is gravity.  We
 * bake (β · g) into one constant BUOYANCY_COEFFICIENT.  Per cell, per
 * tick:
 *
 *      vᵧ[i, j] += BUOYANCY_COEFFICIENT · (T[i, j] − T_ambient) · dt
 *
 * That's the entire buoyancy step — five characters of code and one
 * Newton's-law integration.  Try it: removing this function gives a
 * cold, drifting cloud (advection still moves stuff, but nothing
 * lifts it).  Doubling BUOYANCY_COEFFICIENT gives a fast riser;
 * halving gives a slow ponderous cloud.
 *
 * Real bombs: buoyancy lifts the fireball after the initial radial
 * expansion (~1-2 sec).  We seed the radial outflow in detonate()
 * (T8); buoyancy takes over from there.
 *
 * T3  OPERATOR SPLITTING — DIVIDE AND CONQUER
 * ───────────────────────────────────────────
 * Full Navier-Stokes is a coupled non-linear PDE.  Solving it
 * directly each tick is hard.  Stam's TRICK: split into pieces, each
 * with a clean solver, run them in sequence:
 *
 *     (1)  ∂v/∂t = f                FORCES — trivial: v += f·dt
 *     (2)  ∂v/∂t = −(v·∇)v          ADVECTION (T4)
 *     (3)  ∇·v = 0  via  ∇²p = ∇·v  PROJECTION (T5)
 *
 * Plus passive scalars (T, ρ) get advected by the cleaned-up
 * velocity, and temperature additionally cools (T6).
 *
 * Stam's order is: BUOYANCY → PROJECT → ADVECT v → PROJECT → ADVECT
 * scalars → COOL.  Six steps.  TWO projections because advection
 * re-introduces tiny divergence; we clean it up before the scalars
 * are advected.
 *
 * Each step's input is the previous step's output.  Each step is
 * UNCONDITIONALLY STABLE in isolation — the composition is too.
 *
 * Same architectural pattern as Unix pipes: each box knows nothing
 * about the others, plug new physics steps in by inserting a new
 * box (e.g. surface tension, vorticity confinement).
 *
 * T4  SEMI-LAGRANGIAN ADVECTION — BACKWARD TRACE
 * ──────────────────────────────────────────────
 * Naïve advection: PUSH each cell's value forward by velocity·dt:
 *
 *      new_field[i + vᵣ·dt][j + vᵧ·dt] = field[i][j]
 *
 * Two problems: (a) destination is fractional; (b) at big timesteps
 * the value lands outside the grid or overlaps other pushed values.
 * Either way it BLOWS UP — values amplify each tick.
 *
 * Stam's STABLE FLUIDS trick: trace BACKWARD.  For each destination
 * cell, ask "where was the fluid currently here, one step ago?".
 * Sample the OLD field at that source location:
 *
 *      for each cell (i, j):
 *          source_pos        = (i, j) − v[i, j]·dt / h
 *          new_field[i, j]   = bilinear(old_field, source_pos)
 *
 * Bilinear interpolation gives a clean weighted average of the four
 * neighbouring cells around the source position.  Two key
 * properties:
 *
 *   STABILITY  - bilinear sample is bounded by the four neighbours;
 *                the field can never blow up no matter how big dt.
 *   DIFFUSION  - bilinear interpolation always loses a little high-
 *                frequency detail.  Each tick the field gets very
 *                slightly blurrier.  Acceptable price.
 *
 * One advect_field() works for any scalar field: vᵣ, vᵧ, T, ρ.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   t            t + dt                                │
 *      │                                                      │
 *      │   ●────────►   ■                                     │
 *      │  source        cell at t+dt                          │
 *      │                                                      │
 *      │   "where did the fluid in ■ come from at time t?"    │
 *      │   Answer: trace backward by -v·dt → (i_back, j_back) │
 *      │   Then bilinear-interp the OLD field at that point.  │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T5  PRESSURE PROJECTION — HODGE DECOMPOSITION + JACOBI
 * ──────────────────────────────────────────────────────
 * After buoyancy adds vertical velocity, the field has nonzero
 * divergence — some cells "create" upward fluid.  To restore mass
 * conservation (∇·v = 0):
 *
 * HODGE DECOMPOSITION (the math fact we exploit).  Any vector
 * field can be uniquely split:
 *
 *      v = v_div_free + ∇φ
 *
 * where ∇φ is the gradient of some scalar field φ.  We don't know
 * v_div_free directly, but we can find ∇φ by solving:
 *
 *      ∇²φ = ∇·v          (Poisson equation for φ)
 *
 * Then v_div_free = v − ∇φ.  In our notation φ is "pressure" p:
 *
 *      1. compute ∇·v
 *      2. solve ∇²p = ∇·v for p (Jacobi iteration)
 *      3. subtract ∇p from v
 *
 * The result is a velocity field whose divergence is approximately
 * zero — fluid is conserved.
 *
 * JACOBI UPDATE.  Discrete Laplacian on a 5-point stencil:
 *
 *      ∇²p[i, j] ≈ (p[i+1, j] + p[i−1, j] + p[i, j+1] + p[i, j−1]
 *                   − 4·p[i, j]) / h²
 *
 * Setting equal to ∇·v[i, j] and solving for p[i, j]:
 *
 *      p[i, j] = (p[i+1, j] + p[i−1, j] + p[i, j+1] + p[i, j−1]
 *                 − h² · ∇·v[i, j]) / 4
 *
 * Replace each cell with the average of its four neighbours minus a
 * divergence correction.  Iterate until convergence.  We use 40
 * sweeps — enough for visual quality, fast enough for 60 fps.
 *
 * Boundary conditions: zero-gradient (mirror neighbours).  No-flow
 * walls everywhere — consistent with how we treat the axis (no
 * radial flow through r=0) and ground (no vertical flow through
 * y=0).  See §4 mirror_index.
 *
 * T6  COOLING AND DENSITY DECAY
 * ─────────────────────────────
 * Two passive losses keep the simulation from running indefinitely:
 *
 * NEWTON COOLING — temperature returns toward ambient exponentially:
 *
 *      T_new = T_ambient + (T_old − T_ambient) · exp(−k_cool · dt)
 *
 * The constant k_cool sets the time scale.  k_cool = 0.06/sec means
 * temperature excess halves every ~12 sec.  Without cooling, the
 * cloud rises forever; nothing pulls heat back down.
 *
 * DENSITY DECAY — linear loss models entrainment of ambient air:
 *
 *      ρ_new = ρ_old · (1 − k_decay · dt)
 *
 * We don't simulate entrainment as a process; we approximate its net
 * effect with a slow linear fade.  Without decay, dense smoke
 * persists indefinitely and the cloud never dissipates.
 *
 * Both are simple per-cell operations applied at the END of each
 * fluid_step().  They commute with each other (act on independent
 * fields) and with advection (advection is bilinear; both decays
 * are uniform scalar multiplications).
 *
 * T7  AXISYMMETRIC REDUCTION — 2-D SIM, 3-D RENDER
 * ────────────────────────────────────────────────
 * The fluid simulation is 2-D (radius × altitude = 56 × 96 cells).
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
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   2-D simulation slice         3-D rendered scene    │
 *      │   (radius × altitude)          (x, y, z space)       │
 *      │                                                      │
 *      │      ┌───────────┐                                   │
 *      │      │  ● ● ●    │           rotate around y         │
 *      │      │ ●     ●   │           ──────────────►         │
 *      │      │●       ●  │              ╭──────╮             │
 *      │      │  ● ● ●    │              │ ●●●● │             │
 *      │      │           │             │ ●●●●●● │            │
 *      │      └───────────┘              │●●●●●●●│            │
 *      │      r → outward                 ╰──────╯             │
 *      │      y ↑ upward                  axisymmetric         │
 *      │                                  3-D extrusion        │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Ramifications:
 *   - CHEAPER SIM.  ~5K grid cells, not ~3M (50× cost reduction).
 *   - LIMITED PHENOMENA.  No tilted columns, no wind shear, no
 *     non-circular caps.  Everything mirror-symmetric around y.
 *
 * For a MUSHROOM CLOUD this is approximately fine — real mushroom
 * clouds are very nearly axisymmetric.  For wind-blown smoke,
 * leaning campfires, or vortex shedding behind obstacles, you'd
 * need full 3-D simulation.
 *
 * T8  VOLUMETRIC RAYMARCHING — BEER-LAMBERT
 * ─────────────────────────────────────────
 * Surface raymarching (raymarcher.c) finds WHERE a ray hits a
 * surface.  Volume rendering integrates LIGHT along a ray as it
 * passes through a SEMI-TRANSPARENT MEDIUM.  For each ray:
 *
 *      transmittance = 1
 *      total_lum     = 0
 *      hot_lum       = 0
 *      for each step at distance s from origin:
 *          dτ      = ρ · STEP · DENSITY_GAIN          (optical depth)
 *          emit    = (T − T_amb) / (T_peak − T_amb)   in [0, 1]
 *          source  = emit · EMISSION_GAIN + AMBIENT_FLOOR
 *          total_lum += transmittance · dτ · source           (accumulate)
 *          hot_lum   += transmittance · dτ · emit · EMISSION_GAIN
 *          transmittance *= exp(−dτ)                  (Beer-Lambert decay)
 *          if transmittance < ε: break
 *
 * Physical interpretation:
 *   - DENSITY blocks light: each step subtracts dτ from optical
 *     transmittance, and exp(−dτ) is what fraction of incoming light
 *     survives the segment (Beer-Lambert law).
 *   - TEMPERATURE emits light: hot cells contribute proportional to
 *     their temperature excess.  A cool foggy cell adds nothing
 *     beyond AMBIENT_FLOOR (a faint Rayleigh-scatter analogue).
 *   - Each contribution is multiplied by transmittance — light from
 *     cells far behind opaque smoke doesn't reach the camera.
 *
 * Two outputs per ray:
 *   total_luminance   how bright this pixel is overall
 *   hot_luminance     how much of that came from emission
 *
 * The HOT FRACTION (hot_lum / total_lum) drives the smoke vs. fire
 * colour decision in §18 cell_decorate.
 *
 * Speed optimisation: empty regions (ρ < threshold) skip ahead with
 * a longer step.  Pure-empty rays (which dominate when the cloud is
 * small) cost ~70% less than a uniform march.
 *
 * T9  CAMERA + RAY GENERATION
 * ───────────────────────────
 * Standard pinhole camera.  Three orthonormal vectors define the
 * view:
 *
 *      forward = look_at − cam_origin, normalised
 *      right   = forward × world_up, normalised
 *      up      = right × forward
 *
 * Per pixel ray:
 *
 *      u =  (col + 0.5) / cols      ·2 −1     ∈ [−1, +1]
 *      v = −((row + 0.5) / rows·2 − 1)        ∈ [−1, +1]  (y-flip)
 *      ray = forward + u·tan(FOV/2)·right
 *                    + v·tan(FOV/2)·aspect·up
 *
 * The aspect factor accounts for terminal cells being ~2× tall.
 *
 * Camera placed at (0, CAM_HEIGHT, −distance) looking at (0,
 * CAM_LOOK_AT_HEIGHT, 0) so the cloud is centred in the frame and
 * we can see the stem from a slightly-above ground angle.
 *
 * T10 PIXEL DECORATION — DECOUPLED GLYPH AND COLOUR SIGNALS
 * ─────────────────────────────────────────────────────────
 * Two scalars per pixel from the raymarcher: total_lum and hot_lum.
 * Naively you might map both to a single colour-glyph composite.
 * We use TWO INDEPENDENT CHANNELS instead:
 *
 *      GLYPH from total_luminance via 8-tier ramp ".,:;+*#@".
 *            Read as: how DENSE is this pixel?
 *
 *      COLOUR from hot_fraction = hot_lum / total_lum.
 *             Cool pixel → ramp slot 0 (smoke colour).
 *             Hot pixel  → ramp slot 7 (fire colour).
 *             Read as: how HOT is this pixel?
 *
 * Result: a thin wisp of cool smoke and the bright core of a
 * fireball look visually distinct EVEN AT THE SAME BRIGHTNESS.
 * If glyph and colour both encoded brightness, you'd get redundant
 * info — a faint dot in red could mean either "dim fire" or "bright
 * smoke," indistinguishable.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   total_lum                hot_lum / total_lum       │
 *      │      │                              │                │
 *      │      ▼                              ▼                │
 *      │   GLYPH                          COLOUR              │
 *      │   ".,:;+*#@"                     "smoke→fire"        │
 *      │      │                              │                │
 *      │      └──────────┬───────────────────┘                │
 *      │                 ▼                                    │
 *      │              mvaddch(row, col, glyph) with          │
 *      │              COLOR_PAIR(pair) | attr                 │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Decoupled signals = more information per pixel.  Same trick
 * appears in good data visualisations: encode independent variables
 * with independent visual channels (size, colour, position, shape).
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

/* ===================================================================== */
/* §1  config — every constant + enums                                   */
/* ===================================================================== */

/* §1.1 frame rate + UI layout. */
enum {
    TARGET_RENDER_FPS_MIN     =  10,
    TARGET_RENDER_FPS_DEFAULT =  60,
    TARGET_RENDER_FPS_MAX     = 120,
    TARGET_RENDER_FPS_STEP    =  10,
    FPS_DISPLAY_UPDATE_MS     = 500,
    HUD_RESERVED_ROWS         =   2,    /* row 0 status + last row hint */
};

/* §1.2 colour-pair IDs. */
enum {
    PAIR_HUD_STATUS = 1,    /* yellow + bold (top status row)           */
    PAIR_HUD_HINT   = 2,    /* cyan + bold   (bottom hint row)          */
    PAIR_RAMP_BASE  = 3,    /* +0..+7 — smoke→fire ramp                  */
};

/* §1.3 time helpers. */
#define NS_PER_SEC          1000000000LL
#define NS_PER_MS              1000000LL
#define TICK_NS(target_fps)  (NS_PER_SEC / (target_fps))

/* §1.4 terminal cell aspect. */
#define TERMINAL_CELL_ASPECT  2.0f      /* physical h / w */

/* §1.5 fluid grid (T7). */
#define GRID_RADIAL_CELLS     56
#define GRID_VERTICAL_CELLS   96
#define GRID_CELL_SIZE        0.125f
#define GRID_RADIAL_EXTENT    ((float)GRID_RADIAL_CELLS   * GRID_CELL_SIZE)
#define GRID_VERTICAL_EXTENT  ((float)GRID_VERTICAL_CELLS * GRID_CELL_SIZE)
#define GRID_INV_CELL_SIZE    (1.0f / GRID_CELL_SIZE)

#define POISSON_JACOBI_ITERATIONS  40    /* sweeps per project (T5)     */

/* §1.6 physics constants. */
#define TEMPERATURE_AMBIENT       1.0f
#define TEMPERATURE_PEAK          8.0f
#define DENSITY_PEAK              4.0f
#define BUOYANCY_COEFFICIENT      2.4f   /* β·g, see T2                  */
#define COOL_RATE                 0.06f  /* Newton cooling (T6)          */
#define DENSITY_DECAY             0.009f /* density linear loss / sec    */

/* §1.7 simulation timing. */
#define SIM_DT                 0.025f    /* fixed sim step (sim seconds) */
#define SIM_DT_MAX_REAL        0.080f    /* cap per real frame           */
#define SIM_RATE_DEFAULT       1.0f
#define SIM_RATE_MIN           0.10f
#define SIM_RATE_MAX           6.0f
#define SIM_RATE_STEP_FACTOR   1.30f

/* §1.8 camera. */
#define CAM_DISTANCE_DEFAULT   28.0f
#define CAM_DISTANCE_MIN        8.0f      /* keeps camera outside MEGATON cap */
#define CAM_DISTANCE_MAX       56.0f
#define CAM_DISTANCE_STEP       2.0f
#define CAM_HEIGHT              5.0f
#define CAM_LOOK_AT_HEIGHT      6.0f
#define CAM_FIELD_OF_VIEW_DEG  52.0f

/* §1.9 volumetric raymarcher (T8). */
#define RM_RAY_NEAR                       0.5f
#define RM_RAY_FAR                       32.0f
#define RM_STEP                           0.18f
#define RM_MAX_STEPS                    130
#define RM_OPAQUE_TRANSMITTANCE_EPS       0.01f
#define RM_DENSITY_GAIN                   1.30f
#define RM_EMISSION_GAIN                  4.5f
#define RM_AMBIENT_FLOOR                  0.06f
#define RM_EMPTY_DENSITY_EPS              0.001f
#define RM_EMPTY_SKIP_FACTOR              2.0f      /* fast-skip multiplier */

/* §1.10 pixel classification (T10). */
#define PIXEL_LUMINANCE_CLAMP    1.10f
#define PIXEL_VISIBLE_LUM_EPS    0.002f
#define GLYPH_SLOT_COUNT         8
#define GLYPH_SLOT_FLOAT         7.999f   /* (GLYPH_SLOT_COUNT - 0.001)  */

/* §1.11 blast presets (T8 of MENTAL MODEL — initial conditions). */
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
    float       sigma;                   /* Gaussian σ (world units)     */
    float       peak_temperature;        /* T peak above ambient         */
    float       peak_density;            /* ρ peak                       */
    float       detonation_altitude;     /* y of blast centre            */
    float       initial_outflow;         /* radial outflow speed at t=0  */
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

/* §1.12 themes — 8-band ramp from coolest (slot 0) to hottest (slot 7). */
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
     * negative).  See decorate_volume_pixel() for attr handling. */
    { "NEGATIVE ",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true },
};

/* §1.13 luminance glyph ramp (T10).  Slot 0 is `.`, not space, so a
 * thin smoke wisp is faintly visible; below PIXEL_VISIBLE_LUM_EPS the
 * cell is left as background. */
static const char LUMINANCE_GLYPHS[GLYPH_SLOT_COUNT] =
    { '.', ',', ':', ';', '+', '*', '#', '@' };

/* §1.14 debug overlays — d / D cycles between them. */
typedef enum {
    DEBUG_NORMAL       = 0,    /* full 3-D volumetric raymarch */
    DEBUG_DENSITY      = 1,    /* raw 2-D density map          */
    DEBUG_TEMPERATURE  = 2,    /* raw 2-D temperature map      */
    DEBUG_VELOCITY     = 3,    /* raw 2-D velocity arrows      */
    DEBUG_MODE_COUNT   = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL    ", "DENSITY 2D", "TEMP 2D   ", "VELOCITY  ",
};

/* ===================================================================== */
/* §2  clock — monotonic timer + sleep                                    */
/* ===================================================================== */

static int64_t clock_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  vec3 — 3-D vector math (raymarcher only)                           */
/* ===================================================================== */

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

/* ===================================================================== */
/* §4  grid_helpers — clampf, mirror_index, to_slot, bilinear sample      */
/* ===================================================================== */
/*
 * Generic utilities used across both the fluid solver and the
 * renderer.  Consolidated here so the §-numbers below have ONE
 * concern each.
 */

static inline float clampf(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/* mirror_index — reflect an out-of-grid index back into [0, size).
 * Implements the no-flow walls used by the projection (§11) and
 * scalar advection.  Going out by 1 returns 1 (mirrors row 0);
 * going out by N returns N-2 (mirrors row N-1). */
static inline int mirror_index(int index, int grid_size)
{
    if (index < 0)         return 1;
    if (index >= grid_size) return grid_size - 2;
    return index;
}

/* to_slot — map a normalised value ∈ [0, 1] to an integer slot
 * index ∈ [0, GLYPH_SLOT_COUNT-1]. */
static inline int to_slot(float value_01)
{
    int slot = (int)(value_01 * GLYPH_SLOT_FLOAT);
    if (slot < 0)                  slot = 0;
    if (slot >= GLYPH_SLOT_COUNT)  slot = GLYPH_SLOT_COUNT - 1;
    return slot;
}

/*
 * sample_field_bilinear — sample a 2-D scalar at fractional cell
 * coordinates (fi, fj).  Out-of-grid coords clamp to the boundary
 * cell.  Bilinear weights:
 *
 *      (i, j+1)·──────·(i+1, j+1)        weight at (col, row):
 *           │        │                       (i,    j  ) = (1−fri)(1−fyj)
 *           │   ✦    │                       (i+1,  j  ) =   fri ·(1−fyj)
 *           │        │                       (i,    j+1) = (1−fri)·fyj
 *      (i,    j)·──────·(i+1,   j)           (i+1,  j+1) =   fri ·fyj
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

/* ===================================================================== */
/* §5  themes — handled in §1; this section reserved for future      */
/* ===================================================================== */
/* (Theme palette table lives in §1.12 alongside the rest of config.) */

/* ===================================================================== */
/* §6  colors — pair init + theme apply                                   */
/* ===================================================================== */

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

static void colors_init(void)
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

/* ===================================================================== */
/* §7  fluid_state — eight 2-D fields in one struct                       */
/* ===================================================================== */
/*
 * The simulation state.  Eight 2-D fields stored side-by-side in one
 * struct.  Total size 8 × 56 × 96 × 4 ≈ 168 KB, in BSS, no malloc.
 *
 * Indexing convention:
 *   i ∈ [0, GRID_RADIAL_CELLS)     radial slot (0 = on the axis)
 *   j ∈ [0, GRID_VERTICAL_CELLS)   vertical slot (0 = on the ground)
 *
 * The grid is the AXISYMMETRIC slice (T7): a 2-D vertical strip from
 * the y-axis outward.  Rotated around y for rendering.
 */
typedef struct {
    /* Velocity (radial, vertical). */
    float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Advected scalars. */
    float temperature       [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float density           [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Projection scratch (T5). */
    float pressure          [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float divergence        [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* General scratch (Jacobi double-buffer + advection swap). */
    float scratch_a         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float scratch_b         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
} Fluid;

static Fluid g_fluid;

/* ===================================================================== */
/* §8  boundaries — wall conditions on the velocity field                 */
/* ===================================================================== */
/*
 * Two physical walls in the axisymmetric simulation:
 *   1. AXIS at radius = 0    — mirror; no radial flow through (vᵣ=0).
 *   2. GROUND at altitude = 0 — solid; no vertical flow (vᵧ=0).
 *
 * Other domain edges (top, far radial) are "open" — advection
 * naturally diffuses fluid that tries to leave the grid.
 *
 * Called after every velocity-modifying step (buoyancy, advection,
 * projection).  Forgetting this is the #1 bug source for adapting
 * Stam-style solvers — symptom is fluid leaking energy at walls.
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

/* ===================================================================== */
/* §9  buoyancy — hot air rises (T2)                                      */
/* ===================================================================== */
/*
 * Boussinesq approximation: vertical accel ∝ temperature excess.
 * Per cell, per tick:
 *     vᵧ[i, j] += BUOYANCY · (T[i, j] − T_ambient) · dt
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

/* ===================================================================== */
/* §10  advect — semi-Lagrangian back-trace (T4)                          */
/* ===================================================================== */
/*
 * Generic over the field being advected — used for vᵣ, vᵧ, T, ρ.
 *
 *   for each cell (i, j):
 *       source_pos     = (i, j) − v·dt / h
 *       new_field[i,j] = bilinear(old_field, source_pos)
 *
 * Velocities are world-units/sec; we want grid-cells, hence
 * GRID_INV_CELL_SIZE = 1/h.
 */
static void advect_field(
        float        destination_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float       source_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float       step_seconds)
{
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

/* ===================================================================== */
/* §11  project — divergence + Jacobi + gradient subtraction (T5)         */
/* ===================================================================== */
/*
 * The Hodge-decomposition step.  Three sub-phases:
 *
 *   PHASE A — compute_divergence: ∇·v field via centred differences.
 *   PHASE B — solve_pressure_poisson: Jacobi sweeps for ∇²p = ∇·v.
 *   PHASE C — subtract_pressure_gradient: v -= ∇p.
 *
 * Boundary conditions are mirror (no-flow walls) — implemented via
 * mirror_index from §4.
 *
 * After project_to_incompressible() returns, ∇·v ≈ 0 everywhere.
 */

static void compute_divergence(Fluid *fluid)
{
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            int i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
            int i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
            int j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
            int j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);

            float dvr_dr = fluid->velocity_radial  [i_right][j]
                         - fluid->velocity_radial  [i_left ][j];
            float dvy_dy = fluid->velocity_vertical[i][j_above]
                         - fluid->velocity_vertical[i][j_below];

            fluid->divergence[i][j] =
                (dvr_dr + dvy_dy) * 0.5f * GRID_INV_CELL_SIZE;
            fluid->pressure[i][j] = 0.0f;   /* zero initial guess for GS */
        }
    }
}

static void solve_pressure_poisson(Fluid *fluid)
{
    float h_squared = GRID_CELL_SIZE * GRID_CELL_SIZE;

    for (int iter = 0; iter < POISSON_JACOBI_ITERATIONS; iter++) {
        /* One Jacobi sweep — write into scratch_a, then memcpy back. */
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
        memcpy(fluid->pressure, fluid->scratch_a, sizeof fluid->pressure);
    }
}

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

static void project_to_incompressible(Fluid *fluid)
{
    compute_divergence(fluid);
    solve_pressure_poisson(fluid);
    subtract_pressure_gradient(fluid);
    enforce_velocity_boundaries(fluid);
}

/* ===================================================================== */
/* §12  cool_decay — Newton cooling + density attenuation (T6)            */
/* ===================================================================== */
/*
 * Two passive losses applied at the END of every fluid_step:
 *   COOL.  T → T_ambient via Newton cooling (exp decay).
 *   DECAY. ρ fades linearly (models entrainment we don't simulate).
 */
static void apply_cool_and_decay(Fluid *fluid, float step_seconds)
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

/* ===================================================================== */
/* §13  fluid_step — one full physics tick (six phases)                   */
/* ===================================================================== */
/*
 * The whole fluid solver in eight calls.  Reads top-to-bottom as the
 * algorithm pseudocode (T3).
 */
static void fluid_step(Fluid *fluid, float step_seconds)
{
    /* (1) Buoyancy — hot cells get an upward push. */
    apply_buoyancy(fluid, step_seconds);

    /* (2) Project — clean up the divergence buoyancy just added. */
    project_to_incompressible(fluid);

    /* (3) Advect velocity by itself.  Use scratch so we can finish
     *     reading both vᵣ and vᵧ before mutating either. */
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
    memcpy(fluid->velocity_radial,
           fluid->scratch_a, sizeof fluid->velocity_radial);
    memcpy(fluid->velocity_vertical,
           fluid->scratch_b, sizeof fluid->velocity_vertical);
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
    apply_cool_and_decay(fluid, step_seconds);
}

/* ===================================================================== */
/* §14  detonate — initial Gaussian bubble (T8 of MENTAL MODEL)           */
/* ===================================================================== */
/*
 * The ONLY scripted moment.  Paint a 3-D-symmetric Gaussian centred
 * at (radius=0, altitude=detonation_altitude), plus a small radial
 * outflow seed.  Everything after this is pure physics.
 *
 *      gauss(d) = exp(−d² / (2·σ²))         d² = r² + (y − y₀)²
 *      T(r, y) = T_ambient + (T_peak − T_ambient) · gauss
 *      ρ(r, y) = ρ_peak · gauss
 *
 * The radial outflow models the mechanical shock wave that happens
 * before buoyancy takes over (~1-2 sec).  Without it, the bubble
 * just rises with a clean leading-edge vortex.  With it, the bubble
 * first expands radially then transitions to buoyant rise — much
 * closer to a real explosion's two-phase profile.
 */
static void detonate_at_origin(Fluid *fluid, const BlastParameters *blast)
{
    /* Reset to ambient. */
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

            float dr = radius;
            float dy = altitude - blast->detonation_altitude;
            float distance_squared = dr * dr + dy * dy;
            float gaussian = expf(-distance_squared / two_sigma_squared);

            fluid->temperature[i][j] =
                TEMPERATURE_AMBIENT
              + (blast->peak_temperature - TEMPERATURE_AMBIENT) * gaussian;
            fluid->density[i][j] = blast->peak_density * gaussian;

            /* Seed initial radial outflow only NEAR the centre. */
            if (gaussian > 0.05f) {
                float distance = sqrtf(distance_squared);
                if (distance > 1e-3f) {
                    float blast_speed = blast->initial_outflow * gaussian;
                    fluid->velocity_radial  [i][j] =
                        (dr / distance) * blast_speed;
                    fluid->velocity_vertical[i][j] =
                        (dy / distance) * blast_speed;
                }
            }
        }
    }
    enforce_velocity_boundaries(fluid);
}

/* ===================================================================== */
/* §15  sampling — world (x, y, z) → bilinear 2-D field                   */
/* ===================================================================== */
/*
 * The bridge from the renderer to the simulator.  T7 axisymmetric
 * reduction: any 3-D point's density / temperature is determined by
 * (radius, altitude).  Out-of-domain samples return safe defaults.
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

/* ===================================================================== */
/* §16  raymarch — Beer-Lambert volumetric integration (T8)               */
/* ===================================================================== */
/*
 * Outputs:
 *   *out_total_luminance — total accumulated brightness (smoke + fire)
 *   *out_hot_luminance   — contribution from temperature emission only
 *
 * The hot fraction (hot/total) drives the smoke→fire palette pick in
 * §18 cell_decorate.
 *
 * Optimisation: empty regions skip ahead.  Pure-empty rays (which
 * dominate when the cloud is small) cost ~70% less than uniform
 * march.
 */
static void raymarch_volume(Vec3 origin, Vec3 direction,
                            float *out_total_luminance,
                            float *out_hot_luminance)
{
    float t                  = RM_RAY_NEAR;
    float transmittance      = 1.0f;
    float total_luminance    = 0.0f;
    float hot_luminance      = 0.0f;
    float inverse_temp_range =
        1.0f / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);

    for (int step = 0; step < RM_MAX_STEPS; step++) {
        Vec3 sample_point = v3add(origin, v3scale(t, direction));

        /* Quick rejects outside the simulation domain. */
        if (sample_point.y < 0.0f || sample_point.y > GRID_VERTICAL_EXTENT) {
            t += RM_STEP * RM_EMPTY_SKIP_FACTOR;
            if (t > RM_RAY_FAR) break;
            continue;
        }
        float radius = sqrtf(sample_point.x * sample_point.x
                           + sample_point.z * sample_point.z);
        if (radius > GRID_RADIAL_EXTENT) {
            t += RM_STEP * RM_EMPTY_SKIP_FACTOR;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        float density = sample_density_at_world(radius, sample_point.y);
        if (density < RM_EMPTY_DENSITY_EPS) {
            t += RM_STEP;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* Density is nontrivial — read temperature too. */
        float temperature = sample_temperature_at_world(radius, sample_point.y);

        float optical_depth_step = density * RM_STEP * RM_DENSITY_GAIN;
        float emission = (temperature - TEMPERATURE_AMBIENT) * inverse_temp_range;
        emission = clampf(emission, 0.0f, 1.0f);

        float source = emission * RM_EMISSION_GAIN + RM_AMBIENT_FLOOR;
        total_luminance += transmittance * optical_depth_step * source;
        hot_luminance   += transmittance * optical_depth_step
                                         * emission * RM_EMISSION_GAIN;

        transmittance *= expf(-optical_depth_step);
        if (transmittance < RM_OPAQUE_TRANSMITTANCE_EPS) break;
        if (t > RM_RAY_FAR) break;

        t += RM_STEP;
    }

    *out_total_luminance = total_luminance;
    *out_hot_luminance   = hot_luminance;
}

/* ===================================================================== */
/* §17  camera — orthonormal basis + per-pixel ray (T9)                   */
/* ===================================================================== */

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
    cam.origin = v3(0.0f, CAM_HEIGHT, -distance_behind);
    Vec3 look_at = v3(0.0f, CAM_LOOK_AT_HEIGHT, 0.0f);

    cam.forward   = v3normalise(v3sub(look_at, cam.origin));
    Vec3 world_up = v3(0.0f, 1.0f, 0.0f);
    cam.right     = v3normalise(v3cross(cam.forward, world_up));
    cam.up        = v3cross(cam.right, cam.forward);

    cam.fov_tangent   = tanf(CAM_FIELD_OF_VIEW_DEG
                             * (float)M_PI / 180.0f * 0.5f);
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

/* ===================================================================== */
/* §18  cell_decorate — (lum, hot) → glyph + colour + attr (T10)          */
/* ===================================================================== */

typedef struct {
    char   glyph;
    int    pair;
    attr_t attr;
    bool   skip;
} Cell;

static Cell decorate_volume_pixel(float total_luminance, float hot_luminance,
                                  bool   inverted_theme)
{
    /* Empty cell — left as default-bg / pre-painted bg. */
    if (total_luminance < PIXEL_VISIBLE_LUM_EPS) {
        return (Cell){ .skip = true };
    }

    /* GLYPH from luminance (clamped). */
    float lum_normalised = total_luminance / PIXEL_LUMINANCE_CLAMP;
    if (lum_normalised > 1.0f) lum_normalised = 1.0f;
    int slot_lum = to_slot(lum_normalised);

    /* COLOUR from hot fraction. */
    float hot_fraction = hot_luminance / (total_luminance + 0.001f);
    if (hot_fraction > 1.0f) hot_fraction = 1.0f;
    if (hot_fraction < 0.0f) hot_fraction = 0.0f;
    int slot_hot = to_slot(hot_fraction);

    /* Attribute modulation:
     *   Normal themes  — A_BOLD on hot/bright tiers (extra fire punch),
     *                    A_DIM on cool tiers (fading wisps),
     *                    A_NORMAL otherwise.
     *   Inverted theme — A_NORMAL only.  A_DIM / A_BOLD on a light fg
     *                    over white bg INVERT brightness intent.
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

/* emit_cell — paint one cell with attron/attroff batched on
 * (pair, attr) change.  Halves attribute thrash on uniform regions. */
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

/* ===================================================================== */
/* §19  render_volume — full screen of raymarched cells                   */
/* ===================================================================== */

typedef struct {
    bool       paused;
    int        theme_index;
    BlastType  blast_type;
    DebugMode  debug_mode;
    int        cols, rows;
    float      simulation_time_seconds;
    float      simulation_rate;
    float      simulation_step_accumulator;
    float      camera_distance;
} Scene;

static void render_volume_view(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
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
    int    y_offset  = 1;        /* leave row 0 for status */

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            Vec3  ray = ray_for_pixel(col, row, scene->cols,
                                       visible_rows, &cam);
            float total_lum, hot_lum;
            raymarch_volume(cam.origin, ray, &total_lum, &hot_lum);

            Cell cell = decorate_volume_pixel(total_lum, hot_lum, inverted);
            emit_cell(y_offset + row, col, cell, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ===================================================================== */
/* §20  debug_overlays — direct views of the raw 2-D fluid fields         */
/* ===================================================================== */
/*
 * Each overlay swaps the raymarcher for a DIRECT view of one
 * underlying field.  Walks the 2-D fluid grid mapped to terminal
 * cells (with a y-flip so ground stays at the bottom of the screen).
 *
 *   DEBUG_DENSITY      raw 2-D density map
 *   DEBUG_TEMPERATURE  raw 2-D temperature map
 *   DEBUG_VELOCITY     raw 2-D velocity field as ASCII arrows
 */

static void render_debug_density(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return;

    int    last_pair = -1;
    attr_t last_attr = 0;
    int    y_offset  = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float density   = g_fluid.density[grid_i][grid_j];
            float normalise = clampf(density / DENSITY_PEAK, 0.0f, 1.0f);
            int   slot      = to_slot(normalise);

            if (slot == 0 && density < 0.01f) continue;

            Cell c = { .glyph = LUMINANCE_GLYPHS[slot],
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void render_debug_temperature(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return;

    int    last_pair = -1;
    attr_t last_attr = 0;
    int    y_offset  = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float temperature = g_fluid.temperature[grid_i][grid_j];
            float normalise   = (temperature - TEMPERATURE_AMBIENT)
                              / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);
            normalise = clampf(normalise, 0.0f, 1.0f);
            int   slot = to_slot(normalise);

            if (slot == 0 && normalise < 0.01f) continue;

            Cell c = { .glyph = LUMINANCE_GLYPHS[slot],
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* arrow_for_velocity — pick an ASCII arrow that points in the same
 * direction as a 2-D velocity vector.  8 cardinal/intercardinal
 * directions; magnitude near zero → space. */
static char arrow_for_velocity(float vx, float vy)
{
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 0.05f) return ' ';

    float angle = atan2f(vy, vx);
    int   octant = (int)((angle + (float)M_PI)
                       / ((float)M_PI / 4.0f) + 0.5f) % 8;

    static const char ARROWS[8] = {
        '<', '/', 'v', '\\', '>', '/', '^', '\\',
    };
    return ARROWS[octant];
}

static void render_debug_velocity(const Scene *scene)
{
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return;

    int    last_pair = -1;
    attr_t last_attr = 0;
    int    y_offset  = 1;

    for (int row = 0; row < visible_rows; row++) {
        int grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
        for (int col = 0; col < scene->cols; col++) {
            int grid_i = col * GRID_RADIAL_CELLS / scene->cols;

            float vr = g_fluid.velocity_radial  [grid_i][grid_j];
            float vy = g_fluid.velocity_vertical[grid_i][grid_j];

            char glyph = arrow_for_velocity(vr, vy);
            if (glyph == ' ') continue;

            float magnitude = sqrtf(vr * vr + vy * vy);
            float normalise = clampf(magnitude / 4.0f, 0.0f, 1.0f);
            int   slot      = to_slot(normalise);

            Cell c = { .glyph = glyph,
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = (slot >= 5) ? A_BOLD : A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ===================================================================== */
/* §21  render_dispatch — pick volume vs debug per active mode            */
/* ===================================================================== */

static void render_active_view(const Scene *scene)
{
    switch (scene->debug_mode) {
        case DEBUG_NORMAL:      render_volume_view      (scene); break;
        case DEBUG_DENSITY:     render_debug_density    (scene); break;
        case DEBUG_TEMPERATURE: render_debug_temperature(scene); break;
        case DEBUG_VELOCITY:    render_debug_velocity   (scene); break;
        default:                render_volume_view      (scene); break;
    }
}

/* ===================================================================== */
/* §22  hud — top status + bottom hint (CLAUDE.md spec)                   */
/* ===================================================================== */
/*
 *   row 0          PAIR_HUD_STATUS (yellow + bold) — title + status
 *   row rows-1     PAIR_HUD_HINT   (cyan   + bold) — key hint
 */

static void hud_draw(int term_rows, int term_cols, const Scene *scene,
                     double real_fps, int target_render_fps)
{
    char status_text[200];
    snprintf(status_text, sizeof status_text,
             " %5.1f fps  %3d Hz  blast:%s  theme:%s  debug:%s  "
             "t:%6.2fs  rate:%4.2f  dist:%4.1f  %s ",
             real_fps, target_render_fps,
             BLAST_PRESETS[scene->blast_type].display_name,
             THEMES[scene->theme_index].display_name,
             DEBUG_MODE_NAMES[scene->debug_mode],
             (double)scene->simulation_time_seconds,
             (double)scene->simulation_rate,
             (double)scene->camera_distance,
             scene->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status_text);
    if (slen > term_cols) slen = term_cols;

    attron(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);
    mvprintw(0, term_cols - slen, "%s", status_text);
    mvprintw(0, 0, " NUKE · FLUID ");
    attroff(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
             " q:quit  spc:pause  r:detonate  n/N:blast  t/T:theme  "
             "d/D:debug  +/-:rate  z/Z:zoom ");
    attroff(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §23  screen — ncurses init / cleanup / present                         */
/* ===================================================================== */

typedef struct { int rows, cols; } Screen;

static void screen_init(Screen *screen)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    colors_init();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void)
{
    endwin();
}

static void screen_resize(Screen *screen)
{
    endwin();
    refresh();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, const Scene *scene,
                                 double real_fps, int target_render_fps)
{
    erase();
    render_active_view(scene);
    hud_draw(screen->rows, screen->cols, scene,
             real_fps, target_render_fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §24  scene — per-frame state + tick + scene helpers                    */
/* ===================================================================== */

static void scene_init(Scene *scene, int cols, int rows)
{
    memset(scene, 0, sizeof *scene);
    scene->paused                       = false;
    scene->theme_index                  = 0;
    scene->blast_type                   = BLAST_STANDARD;
    scene->debug_mode                   = DEBUG_NORMAL;
    scene->cols                         = cols;
    scene->rows                         = rows;
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
 * fixed-dt fluid steps as fit.  Fixed-dt accumulator pattern keeps
 * cloud morphology FRAME-RATE-INDEPENDENT.
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

/* ===================================================================== */
/* §25  app — main loop + signals + key handling                          */
/* ===================================================================== */

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
        case 'q': case 'Q': case 27:                 return false;
        case ' ':           scene->paused = !scene->paused; break;
        case 'r': case 'R': scene_redetonate(scene);        break;

        case 'n': scene_cycle_blast(scene, +1); break;
        case 'N': scene_cycle_blast(scene, -1); break;

        case 't':
            scene->theme_index = (scene->theme_index + 1) % THEME_COUNT;
            apply_theme(scene->theme_index);
            break;
        case 'T':
            scene->theme_index =
                (scene->theme_index + THEME_COUNT - 1) % THEME_COUNT;
            apply_theme(scene->theme_index);
            break;

        case 'd':
            scene->debug_mode =
                (DebugMode)((scene->debug_mode + 1) % DEBUG_MODE_COUNT);
            break;
        case 'D':
            scene->debug_mode = (DebugMode)
                ((scene->debug_mode + DEBUG_MODE_COUNT - 1) % DEBUG_MODE_COUNT);
            break;

        case '=': case '+':
            scene->simulation_rate *= SIM_RATE_STEP_FACTOR;
            if (scene->simulation_rate > SIM_RATE_MAX)
                scene->simulation_rate = SIM_RATE_MAX;
            break;
        case '-':
            scene->simulation_rate /= SIM_RATE_STEP_FACTOR;
            if (scene->simulation_rate < SIM_RATE_MIN)
                scene->simulation_rate = SIM_RATE_MIN;
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
    atexit(screen_cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app                 = &g_app;
    app->running             = 1;
    app->target_render_fps   = TARGET_RENDER_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev_frame_ns       = clock_now_ns();
    int64_t fps_window_ns       = 0;
    int     frames_in_window    = 0;
    double  measured_fps        = 0.0;

    while (app->running) {
        int64_t frame_start_ns = clock_now_ns();

        /* ── input ── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) {
            app->running = 0;
            break;
        }

        /* ── resize ── */
        if (app->need_resize) {
            app_handle_resize(app);
            prev_frame_ns = clock_now_ns();
        }

        /* ── dt ── */
        int64_t dt_ns  = frame_start_ns - prev_frame_ns;
        prev_frame_ns  = frame_start_ns;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt_real_seconds = (float)dt_ns / (float)NS_PER_SEC;

        /* ── physics ── */
        scene_tick(&app->scene, dt_real_seconds);

        /* ── fps window ── */
        frames_in_window++;
        fps_window_ns += dt_ns;
        if (fps_window_ns >= FPS_DISPLAY_UPDATE_MS * NS_PER_MS) {
            measured_fps = (double)frames_in_window
                         / ((double)fps_window_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            fps_window_ns    = 0;
        }

        /* ── render ── */
        screen_present_frame(&app->screen, &app->scene,
                             measured_fps, app->target_render_fps);

        /* ── frame cap ── */
        int64_t target_frame_ns = TICK_NS(app->target_render_fps);
        int64_t spent           = clock_now_ns() - frame_start_ns;
        if (spent < target_frame_ns) clock_sleep_ns(target_frame_ns - spent);
    }

    return 0;
}
