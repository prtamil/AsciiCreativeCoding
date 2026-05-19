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
 *   ── 2-D fluid solver (Stable Fluids) ──────────────────────────────
 *   [1] Stam, J. (1999), "Stable Fluids", SIGGRAPH '99 — THE
 *       foundational paper.  Introduces semi-Lagrangian advection +
 *       Hodge-projection scheme used in §fluid_*.  Five pages, mostly
 *       diagrams.
 *   [2] Stam, J. (2003), "Real-Time Fluid Dynamics for Games", GDC —
 *       the classroom version with 100-line reference C code that
 *       mirrors our §advect / §diffuse / §project.
 *   [3] Bridson, R. (2008), "Fluid Simulation for Computer Graphics",
 *       CRC Press — chs. 1-4 cover the discretisations used here in
 *       depth (semi-Lagrangian, Gauss-Seidel, projection).
 *
 *   ── Combustion / buoyancy models ─────────────────────────────────
 *   [4] Fedkiw, R., Stam, J. & Jensen, H. W. (2001), "Visual
 *       Simulation of Smoke", SIGGRAPH 2001 — the temperature-driven
 *       buoyancy term (T·ĝ) and vorticity-confinement extension used
 *       in §fluid_buoyancy.
 *   [5] Nguyen, D. Q., Fedkiw, R. & Jensen, H. W. (2002), "Physically
 *       Based Modeling and Animation of Fire", SIGGRAPH — the fire-
 *       core / temperature-emission model behind §raymarch's hot-zone
 *       glow.
 *
 *   ── Volume rendering (3-D raymarch) ──────────────────────────────
 *   [6] Quilez, I., "Volumetric raymarching" —
 *       iquilezles.org/articles/raymarchingvolumes; the Beer-Lambert
 *       integration loop in §raymarch.
 *   [7] Max, N. (1995), "Optical Models for Direct Volume Rendering",
 *       IEEE TVCG 1(2) — the canonical taxonomy of volume-rendering
 *       integrals; §16 implements the "emission-absorption" model.
 *   [8] Beer-Lambert law derivation — any optics textbook;
 *       absorption τ = ∫ κ·ds along the ray.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *   [9] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — the
 *       luminance→glyph ramp used to convert raymarched colour to
 *       cell glyphs.
 *  [10] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [11] https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law
 *  [12] https://en.wikipedia.org/wiki/Volume_rendering
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
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config — every constant + enums                                   */
/* ===================================================================== */

/* §1.1 frame rate + UI layout. */
enum {
  TARGET_RENDER_FPS_MIN = 10,
  TARGET_RENDER_FPS_DEFAULT = 60,
  TARGET_RENDER_FPS_MAX = 120,
  TARGET_RENDER_FPS_STEP = 10,
  FPS_DISPLAY_UPDATE_MS = 500,
  HUD_RESERVED_ROWS = 2, /* row 0 status + last row hint */
};

/* §1.2 colour-pair IDs. */
enum {
  PAIR_HUD_STATUS = 1, /* yellow + bold (top status row)           */
  PAIR_HUD_HINT = 2,   /* cyan + bold   (bottom hint row)          */
  PAIR_RAMP_BASE = 3,  /* +0..+7 — smoke→fire ramp                  */
};

/* §1.3 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(target_fps) (NS_PER_SEC / (target_fps))

/* §1.4 terminal cell aspect. */
#define TERMINAL_CELL_ASPECT 2.0f /* physical h / w */

/* §1.5 fluid grid (T7). */
#define GRID_RADIAL_CELLS 56
#define GRID_VERTICAL_CELLS 96
#define GRID_CELL_SIZE 0.125f
#define GRID_RADIAL_EXTENT ((float)GRID_RADIAL_CELLS * GRID_CELL_SIZE)
#define GRID_VERTICAL_EXTENT ((float)GRID_VERTICAL_CELLS * GRID_CELL_SIZE)
#define GRID_INV_CELL_SIZE (1.0f / GRID_CELL_SIZE)

#define POISSON_JACOBI_ITERATIONS 40 /* sweeps per project (T5)     */

/* §1.6 physics constants. */
#define TEMPERATURE_AMBIENT 1.0f
#define TEMPERATURE_PEAK 8.0f
#define DENSITY_PEAK 4.0f
#define BUOYANCY_COEFFICIENT 2.4f /* β·g, see T2                  */
#define COOL_RATE 0.06f           /* Newton cooling (T6)          */
#define DENSITY_DECAY 0.009f      /* density linear loss / sec    */

/* §1.7 simulation timing. */
#define SIM_DT 0.025f          /* fixed sim step (sim seconds) */
#define SIM_DT_MAX_REAL 0.080f /* cap per real frame           */
#define SIM_RATE_DEFAULT 1.0f
#define SIM_RATE_MIN 0.10f
#define SIM_RATE_MAX 6.0f
#define SIM_RATE_STEP_FACTOR 1.30f

/* §1.8 camera. */
#define CAM_DISTANCE_DEFAULT 28.0f
#define CAM_DISTANCE_MIN 8.0f /* keeps camera outside MEGATON cap */
#define CAM_DISTANCE_MAX 56.0f
#define CAM_DISTANCE_STEP 2.0f
#define CAM_HEIGHT 5.0f
#define CAM_LOOK_AT_HEIGHT 6.0f
#define CAM_FIELD_OF_VIEW_DEG 52.0f

/* §1.9 volumetric raymarcher (T8). */
#define RM_RAY_NEAR 0.5f
#define RM_RAY_FAR 32.0f
#define RM_STEP 0.18f
#define RM_MAX_STEPS 130
#define RM_OPAQUE_TRANSMITTANCE_EPS 0.01f
#define RM_DENSITY_GAIN 1.30f
#define RM_EMISSION_GAIN 4.5f
#define RM_AMBIENT_FLOOR 0.06f
#define RM_EMPTY_DENSITY_EPS 0.001f
#define RM_EMPTY_SKIP_FACTOR 2.0f /* fast-skip multiplier */

/* §1.10 pixel classification (T10). */
#define PIXEL_LUMINANCE_CLAMP 1.10f
#define PIXEL_VISIBLE_LUM_EPS 0.002f
#define GLYPH_SLOT_COUNT 8
#define GLYPH_SLOT_FLOAT 7.999f /* (GLYPH_SLOT_COUNT - 0.001)  */

/* §1.11 blast presets (T8 of MENTAL MODEL — initial conditions). */
typedef enum {
  BLAST_TACTICAL = 0,
  BLAST_STANDARD = 1,
  BLAST_MEGATON = 2,
  BLAST_AIR_BURST = 3,
  BLAST_GROUND = 4,
  BLAST_TYPE_COUNT = 5,
} BlastType;

/*
 * BlastParameters — one of 5 named blast presets (selected by '1'..'5').
 *
 * Intent
 *   Each preset is a YIELD CLASS plus geometric placement: peak
 *   temperature + density (how hot, how dense), Gaussian width
 *   (how big), detonation altitude (where in the world), and
 *   initial outflow speed (the kick that launches the rising
 *   mushroom).  Pressing a number drops the same Gaussian blob with
 *   different magnitudes into the fluid solver.
 *
 * Why these specific fields
 *   Together they parametrise the Gaussian initial condition that
 *   §scene_detonate seeds.  Tuned by hand against real-world yield
 *   classes (tactical / standard / megaton / air-burst / ground).
 *
 * Reference [4] Fedkiw-Stam-Jensen 2001 for the temperature-driven
 *   buoyancy that turns each blast's hot core into a rising column.
 */
typedef struct {
    const char *display_name;       /* short HUD label                  */
    float       sigma;              /* Gaussian σ (world units)         */
    float       peak_temperature;   /* T peak above ambient             */
    float       peak_density;       /* ρ peak                           */
    float       detonation_altitude;/* y of blast centre (world)        */
    float       initial_outflow;    /* radial outflow speed at t=0      */
} BlastParameters;

static const BlastParameters BLAST_PRESETS[BLAST_TYPE_COUNT] = {
    /* TACTICAL  — small, low-altitude, brief mushroom */
    {"TACTICAL  ", 0.35f, 5.0f, 2.5f, 1.0f, 2.0f},
    /* STANDARD  — canonical mid-yield mushroom */
    {"STANDARD  ", 0.55f, 8.0f, 4.0f, 1.6f, 3.0f},
    /* MEGATON   — huge yield, very tall column, long-lasting cap */
    {"MEGATON   ", 0.85f, 12.0f, 5.5f, 2.2f, 4.5f},
    /* AIR_BURST — high-altitude detonation, no ground stem */
    {"AIR_BURST ", 0.55f, 8.0f, 4.0f, 4.5f, 3.0f},
    /* GROUND    — surface burst, modest heat, heavy dust load */
    {"GROUND    ", 0.45f, 7.0f, 7.0f, 0.6f, 3.5f},
};

/* §1.12 themes — 8-band ramp from coolest (slot 0) to hottest (slot 7). */
/*
 * Theme — one of 6 named look palettes.
 *
 * Intent
 *   The raymarched volume's luminance is bucketed into
 *   GLYPH_SLOT_COUNT (=8) tiers, with slot 0 = coolest (faint smoke)
 *   through slot 7 = hottest (fire core).  Each theme maps the same
 *   8 tiers to different colour journeys (realistic / matrix /
 *   sunset / ...) so the visual interpretation of "hot" varies
 *   without changing physics.
 *
 * Why an 8-slot ramp (matching GLYPH_SLOT_COUNT)
 *   The glyph ramp `' . : * ▓ ▒ █ █` (or its ASCII equivalent) has
 *   8 tiers; one colour per glyph keeps the colour↔glyph mapping
 *   diagonal.  Slot 0 (faintest) needs to stay legible against the
 *   default-black bg so all themes pick values ≥ 24 there.
 *
 * Why inverted_background
 *   A few themes ("paper", inverted ramps) want a bright background
 *   instead of black.  The renderer reads this flag to clear the
 *   screen to the right paper colour before painting glyphs.
 *
 * References [9] Bourke for the glyph-ramp design; [10] Raymond for
 *   init_pair semantics.
 */
typedef struct {
    const char *display_name;            /* short HUD label              */
    short       ramp_256[GLYPH_SLOT_COUNT];  /* 8 fg indices, cool→hot   */
    bool        inverted_background;     /* bright-background themes     */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    /* REALISTIC: light grey smoke climbing into orange-yellow fire. */
    {"REALISTIC", {248, 250, 252, 254, 130, 166, 208, 220}, false},

    /* MATRIX: lime-green smoke + lime fire. */
    {"MATRIX   ", {64, 70, 112, 113, 119, 154, 190, 226}, false},

    /* OCEAN: deep teal smoke + cyan fire. */
    {"OCEAN    ", {37, 44, 45, 74, 81, 117, 159, 195}, false},

    /* NOVA: violet-lavender smoke + ice-blue fire. */
    {"NOVA     ", {97, 98, 99, 105, 111, 147, 153, 195}, false},

    /* TOXIC: chartreuse smoke + acid green-yellow fire. */
    {"TOXIC    ", {107, 113, 149, 155, 191, 192, 226, 228}, false},

    /* NEGATIVE: white background, dark foreground (photographic
     * negative).  See decorate_volume_pixel() for attr handling. */
    {"NEGATIVE ", {253, 250, 245, 240, 237, 234, 232, 16}, true},
};

/* §1.13 luminance glyph ramp (T10).  Slot 0 is `.`, not space, so a
 * thin smoke wisp is faintly visible; below PIXEL_VISIBLE_LUM_EPS the
 * cell is left as background. */
static const char LUMINANCE_GLYPHS[GLYPH_SLOT_COUNT] = {'.', ',', ':', ';',
                                                        '+', '*', '#', '@'};

/* §1.14 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,      /* full 3-D volumetric raymarch */
  DEBUG_DENSITY = 1,     /* raw 2-D density map          */
  DEBUG_TEMPERATURE = 2, /* raw 2-D temperature map      */
  DEBUG_VELOCITY = 3,    /* raw 2-D velocity arrows      */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL    ",
    "DENSITY 2D",
    "TEMP 2D   ",
    "VELOCITY  ",
};

/* ===================================================================== */
/* §2  clock — monotonic timer + sleep                                    */
/* ===================================================================== */

static int64_t clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  vec3 — 3-D vector math (raymarcher only)                           */
/* ===================================================================== */

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
static inline Vec3 v3scale(float s, Vec3 a) {
  return v3(s * a.x, s * a.y, s * a.z);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 v3cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline float v3length(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3normalise(Vec3 a) {
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

static inline float clampf(float value, float lower, float upper) {
  if (value < lower)
    return lower;
  if (value > upper)
    return upper;
  return value;
}

/* mirror_index — reflect an out-of-grid index back into [0, size).
 * Implements the no-flow walls used by the projection (§11) and
 * scalar advection.  Going out by 1 returns 1 (mirrors row 0);
 * going out by N returns N-2 (mirrors row N-1). */
static inline int mirror_index(int index, int grid_size) {
  if (index < 0)
    return 1;
  if (index >= grid_size)
    return grid_size - 2;
  return index;
}

/* to_slot — map a normalised value ∈ [0, 1] to an integer slot
 * index ∈ [0, GLYPH_SLOT_COUNT-1]. */
static inline int to_slot(float value_01) {
  int slot = (int)(value_01 * GLYPH_SLOT_FLOAT);
  if (slot < 0)
    slot = 0;
  if (slot >= GLYPH_SLOT_COUNT)
    slot = GLYPH_SLOT_COUNT - 1;
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
/* Clamp the sample point so that both (i, j) and (i+1, j+1) remain
 * inside the allocated grid.  Without this the i+1/j+1 corner reads
 * would seg-fault for samples just past the boundary. */
static inline void clamp_sample_to_grid_bounds(float *fi, float *fj) {
    *fi = clampf(*fi, 0.0f, (float)(GRID_RADIAL_CELLS - 1));
    *fj = clampf(*fj, 0.0f, (float)(GRID_VERTICAL_CELLS - 1));
}

/* Split a real-valued sample (fi, fj) into the integer base cell
 * (i, j) and the fractional offsets (frac_i, frac_j) ∈ [0, 1) used
 * as bilinear blend weights.  The base cell is the BOTTOM-LEFT corner
 * of the 2×2 stencil. */
static inline void integer_cell_and_subcell(float fi, float fj,
                                             int *out_i, int *out_j,
                                             float *out_frac_i, float *out_frac_j) {
    int i = (int)floorf(fi);
    int j = (int)floorf(fj);
    if (i >= GRID_RADIAL_CELLS   - 1) i = GRID_RADIAL_CELLS   - 2;
    if (j >= GRID_VERTICAL_CELLS - 1) j = GRID_VERTICAL_CELLS - 2;
    *out_i = i;
    *out_j = j;
    *out_frac_i = fi - (float)i;
    *out_frac_j = fj - (float)j;
}

/* Two-stage lerp over the 4 corners of a unit cell.  Returns a value
 * BETWEEN the min and max of the four corners — unconditionally stable
 * because every output is a convex combination of inputs. */
static inline float bilinear_blend_corners(float bottom_left, float bottom_right,
                                            float top_left,    float top_right,
                                            float frac_i,      float frac_j) {
    float bottom = bottom_left * (1.0f - frac_i) + bottom_right * frac_i;
    float top    = top_left    * (1.0f - frac_i) + top_right    * frac_i;
    return bottom * (1.0f - frac_j) + top * frac_j;
}

/*
 * sample_field_bilinear — bilinear sample of a 2-D scalar at (fi, fj).
 *
 * Pseudocode:
 *   clamp_sample_to_grid_bounds(&fi, &fj)             ← prevent over-read
 *   integer_cell_and_subcell(fi, fj, …)               ← (i, j, frac_i, frac_j)
 *   return bilinear_blend_corners(field[i][j], field[i+1][j],
 *                                  field[i][j+1], field[i+1][j+1],
 *                                  frac_i, frac_j)
 */
static float
sample_field_bilinear(const float field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
                      float fi, float fj) {
    clamp_sample_to_grid_bounds(&fi, &fj);

    int   i, j;
    float frac_i, frac_j;
    integer_cell_and_subcell(fi, fj, &i, &j, &frac_i, &frac_j);

    return bilinear_blend_corners(field[i    ][j    ], field[i + 1][j    ],
                                   field[i    ][j + 1], field[i + 1][j + 1],
                                   frac_i, frac_j);
}

/* ===================================================================== */
/* §5  themes — handled in §1; this section reserved for future      */
/* ===================================================================== */
/* (Theme palette table lives in §1.12 alongside the rest of config.) */

/* ===================================================================== */
/* §6  colors — pair init + theme apply                                   */
/* ===================================================================== */

static void apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  short background_256 = theme->inverted_background ? 231 : -1;
  short background_8 = theme->inverted_background ? COLOR_WHITE : -1;

  if (COLORS >= 256) {
    for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
      init_pair((short)(PAIR_RAMP_BASE + slot), theme->ramp_256[slot],
                background_256);
  } else {
    /* 8-colour fallback — coarse approximation. */
    static const short FALLBACK_RAMP_8[GLYPH_SLOT_COUNT] = {
        COLOR_BLACK, COLOR_BLACK, COLOR_WHITE,  COLOR_WHITE,
        COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_WHITE,
    };
    for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
      init_pair((short)(PAIR_RAMP_BASE + slot),
                theme->inverted_background ? COLOR_BLACK
                                           : FALLBACK_RAMP_8[slot],
                background_8);
  }
}

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD_STATUS, 226, -1); /* bright yellow */
    init_pair(PAIR_HUD_HINT, 51, -1);    /* bright cyan   */
  } else {
    init_pair(PAIR_HUD_STATUS, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD_HINT, COLOR_CYAN, -1);
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
/*
 * Fluid — the 2-D axisymmetric simulation state.
 *
 * Intent
 *   The (full) 3-D nuclear blast has rotational symmetry around the
 *   vertical axis, so a 2-D RADIAL-VERTICAL slice carries all the
 *   physics.  The 3-D renderer (§raymarch) reconstructs the 3-D field
 *   on demand by computing (r, y) from a world point and bilinearly
 *   sampling THIS slice.  Trade: we get 3-D-looking results at 2-D
 *   computational cost — perfect for a real-time terminal demo.
 *
 *   Grid indexing:
 *     i ∈ [0, GRID_RADIAL_CELLS)     radial slot, 0 = axis
 *     j ∈ [0, GRID_VERTICAL_CELLS)   vertical slot, 0 = ground
 *
 * Why eight 2-D arrays in one struct
 *   The Stable Fluids passes (advect / diffuse / project / buoyancy)
 *   touch FOUR distinct field types — velocity (2 components),
 *   temperature, density — plus FOUR scratch buffers for the iterative
 *   solvers.  Keeping them together as one struct gives a single
 *   pointer to thread through every pass and matches the textbook
 *   formulation in [1] Stam §3.
 *
 * Why velocity is split into radial + vertical (not Vec2[][])
 *   The advection and projection sweeps read each component
 *   independently with different boundary conditions (radial = even
 *   reflection at the axis, vertical = no-flow at ground/sky).
 *   Splitting into two arrays lets the inner loop be tight and
 *   per-component.
 *
 * Why scratch_a / scratch_b (and pressure / divergence)
 *   The Gauss-Seidel / Jacobi solver in §project needs a double
 *   buffer so the previous iteration's values remain readable while
 *   the next one is being written.  scratch_a + scratch_b is the
 *   generic role-swap pair; pressure + divergence are the specific
 *   pair used by the Hodge-projection step.  Both pairs live in BSS
 *   — no allocation in the hot path.
 *
 * References [1] Stam 1999 for the field layout; [4] Fedkiw-Stam-Jensen
 *   for the buoyancy term that turns temperature into vertical lift.
 */
typedef struct {
    /* ── Velocity components (axisymmetric polar) ───────────────── */
    float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* ── Advected scalars (the "content" of the blast) ──────────── */
    float temperature       [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float density           [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* ── Hodge-projection scratch (§project / T5) ───────────────── */
    float pressure          [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float divergence        [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* ── Generic scratch (Jacobi double-buffer + advect swap) ──── */
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
static void enforce_velocity_boundaries(Fluid *fluid) {
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
/* Temperature excess above ambient at one cell.  Positive when the
 * cell is HOTTER than ambient (drives upward buoyant force) and
 * negative when cooler.  Boussinesq approximation [4] Fedkiw §3:
 * density change is a linear function of (T − T_amb). */
static inline float temperature_excess_at(const Fluid *fluid, int i, int j) {
    return fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
}

/* Add the Boussinesq vertical impulse for one cell, one tick:
 *     vᵧ[i, j] += α · (T − T_amb) · Δt
 * where α = BUOYANCY_COEFFICIENT controls how hard hot cells rise. */
static inline void add_boussinesq_vertical_impulse(Fluid *fluid, int i, int j,
                                                    float step_seconds) {
    fluid->velocity_vertical[i][j] +=
        BUOYANCY_COEFFICIENT * temperature_excess_at(fluid, i, j) * step_seconds;
}

/*
 * apply_buoyancy — hot air rises (Boussinesq approximation, T2).
 *
 * Pseudocode:
 *   for each cell (i, j):
 *     add_boussinesq_vertical_impulse(fluid, i, j, dt)
 *   enforce_velocity_boundaries(fluid)
 *
 * Reference [4] Fedkiw, Stam & Jensen 2001 §3 — temperature-driven
 * vertical lift is the entire engine behind the mushroom rise.
 */
static void apply_buoyancy(Fluid *fluid, float step_seconds) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            add_boussinesq_vertical_impulse(fluid, i, j, step_seconds);
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
/* Trace the velocity field BACKWARD from cell (i, j) by one tick of
 * advection.  Returns the DEPARTURE POINT — where the fluid currently
 * at (i, j) came from one Δt ago.  This is the semi-Lagrangian step
 * from [1] Stam §3 / [4] Fedkiw §4: sampling the old field at the
 * departure point and copying it forward is what makes the integrator
 * unconditionally stable (no CFL constraint).
 *
 * Velocities are in world-units/sec; dt_in_grid_units = Δt · (1/h)
 * converts the offset into grid cells. */
static inline void trace_velocity_backward_to_departure(
        int i, int j,
        const float vr[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float vy[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float dt_in_grid_units,
        float *out_source_i, float *out_source_j) {
    *out_source_i = (float)i - vr[i][j] * dt_in_grid_units;
    *out_source_j = (float)j - vy[i][j] * dt_in_grid_units;
}

/*
 * advect_field — semi-Lagrangian advection of one scalar by the
 * velocity field.  Generic — used for vᵣ, vᵧ, T, ρ.
 *
 * Pseudocode:
 *   dt_grid = step_seconds / h
 *   for each cell (i, j):
 *     (src_i, src_j) = trace_velocity_backward_to_departure(i, j, v, dt_grid)
 *     destination[i, j] = sample_field_bilinear(source, src_i, src_j)
 *
 * Refs [1] Stam 1999 §3, [4] Fedkiw 2001 §4.
 */
static void advect_field(
        float destination_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float source_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_radial[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_vertical[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float step_seconds) {
    float dt_in_grid_units = step_seconds * GRID_INV_CELL_SIZE;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float source_i, source_j;
            trace_velocity_backward_to_departure(
                i, j, velocity_radial, velocity_vertical, dt_in_grid_units,
                &source_i, &source_j);
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

/* Pre-compute the 4-neighbour mirror indices for one cell.  Mirror BCs
 * mean an out-of-grid neighbour reads as the cell itself reflected
 * across the boundary, giving zero-flux walls.  Doing this once per
 * cell avoids repeating 4 mirror_index calls in every pass. */
static inline void neighbour_indices_mirrored(int i, int j,
                                              int *out_i_left, int *out_i_right,
                                              int *out_j_below, int *out_j_above) {
    *out_i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
    *out_i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
    *out_j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
    *out_j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);
}

/* Discrete divergence ∇·v at cell (i, j) via centred differences:
 *
 *   ∇·v ≈ ½h · ( (vᵣ[i+1] − vᵣ[i−1]) + (vᵧ[j+1] − vᵧ[j−1]) )
 *
 * Positive divergence means fluid is leaving this cell; negative means
 * it's piling up.  The Hodge projection step zeros this out. */
static inline float centred_divergence_at(const Fluid *fluid, int i, int j) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    float dvr_dr = fluid->velocity_radial  [i_right][j]    -
                   fluid->velocity_radial  [i_left ][j];
    float dvy_dy = fluid->velocity_vertical[i      ][j_above] -
                   fluid->velocity_vertical[i      ][j_below];
    return (dvr_dr + dvy_dy) * 0.5f * GRID_INV_CELL_SIZE;
}

/*
 * compute_divergence — fill the divergence field (RHS of the Poisson
 * problem) and zero the pressure initial guess.
 *
 * Pseudocode:
 *   for each cell (i, j):
 *     fluid->divergence[i, j] = centred_divergence_at(fluid, i, j)
 *     fluid->pressure[i, j]   = 0      (warm-start Gauss-Seidel from zero)
 */
static void compute_divergence(Fluid *fluid) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            fluid->divergence[i][j] = centred_divergence_at(fluid, i, j);
            fluid->pressure  [i][j] = 0.0f;
        }
    }
}

/* One cell's Jacobi update for the Poisson equation ∇²p = ∇·v.
 *   p_new[i, j] = ( Σ p_neighbours − h² · div[i, j] ) / 4
 * Uses the OLD pressure values (read pre-sweep) so the iteration is a
 * true Jacobi (not Gauss-Seidel) — output goes to scratch, then is
 * swapped back at the end of the sweep. */
static inline float jacobi_pressure_update_at(const Fluid *fluid, int i, int j,
                                              float h_squared) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    float neighbour_sum =
        fluid->pressure[i_right][j      ] + fluid->pressure[i_left ][j      ] +
        fluid->pressure[i      ][j_above] + fluid->pressure[i      ][j_below];
    return (neighbour_sum - h_squared * fluid->divergence[i][j]) * 0.25f;
}

/* One full Jacobi SWEEP: read pressure, write next iterate into
 * scratch_a, then copy scratch_a back to pressure.  Repeated
 * POISSON_JACOBI_ITERATIONS times.  Refs [5] Saad ch. 4 for the
 * iterative-Poisson convergence theory. */
static inline void jacobi_pressure_sweep(Fluid *fluid, float h_squared) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            fluid->scratch_a[i][j] = jacobi_pressure_update_at(fluid, i, j,
                                                                h_squared);
        }
    }
    memcpy(fluid->pressure, fluid->scratch_a, sizeof fluid->pressure);
}

/*
 * solve_pressure_poisson — iterative solve of ∇²p = ∇·v via Jacobi.
 *
 * Pseudocode:
 *   for iter = 0 .. POISSON_JACOBI_ITERATIONS-1:
 *     jacobi_pressure_sweep(fluid, h²)
 */
static void solve_pressure_poisson(Fluid *fluid) {
    float h_squared = GRID_CELL_SIZE * GRID_CELL_SIZE;
    for (int iter = 0; iter < POISSON_JACOBI_ITERATIONS; iter++)
        jacobi_pressure_sweep(fluid, h_squared);
}

/* Compute (∂p/∂r, ∂p/∂y) at cell (i, j) via centred differences,
 * scaled by 0.5·(1/h).  These are the components of ∇p in the
 * axisymmetric (r, y) frame. */
static inline void centred_pressure_gradient_at(const Fluid *fluid, int i, int j,
                                                float *out_dp_dr,
                                                float *out_dp_dy) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    *out_dp_dr = (fluid->pressure[i_right][j      ] -
                  fluid->pressure[i_left ][j      ]) * 0.5f * GRID_INV_CELL_SIZE;
    *out_dp_dy = (fluid->pressure[i      ][j_above] -
                  fluid->pressure[i      ][j_below]) * 0.5f * GRID_INV_CELL_SIZE;
}

/* Subtract ∇p from one cell's velocity.  This is the Helmholtz-Hodge
 * projection step — after the loop completes, ∇·v ≈ 0 everywhere. */
static inline void subtract_pressure_gradient_at(Fluid *fluid, int i, int j) {
    float dp_dr, dp_dy;
    centred_pressure_gradient_at(fluid, i, j, &dp_dr, &dp_dy);
    fluid->velocity_radial  [i][j] -= dp_dr;
    fluid->velocity_vertical[i][j] -= dp_dy;
}

/*
 * subtract_pressure_gradient — v_new = v − ∇p, cell by cell.
 *
 * Refs [1] Stam 1999 §3.5 (Hodge decomposition); [6] Quilez volume
 * raymarching for the renderer side.
 */
static void subtract_pressure_gradient(Fluid *fluid) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            subtract_pressure_gradient_at(fluid, i, j);
}

static void project_to_incompressible(Fluid *fluid) {
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
static void apply_cool_and_decay(Fluid *fluid, float step_seconds) {
  float cool_factor = expf(-COOL_RATE * step_seconds);
  float density_factor = 1.0f - DENSITY_DECAY * step_seconds;
  if (density_factor < 0.0f)
    density_factor = 0.0f;

  for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
    for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
      float excess = fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
      fluid->temperature[i][j] = TEMPERATURE_AMBIENT + excess * cool_factor;
      fluid->density[i][j] *= density_factor;
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
/* Advect the velocity field BY ITSELF.  Read both components into
 * scratch buffers first, then swap them back — otherwise the second
 * advect_field call would read partially-updated velocities and the
 * scheme would no longer be true semi-Lagrangian.                  */
static inline void advect_velocity_self(Fluid *fluid, float step_seconds) {
    advect_field(fluid->scratch_a, fluid->velocity_radial,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    advect_field(fluid->scratch_b, fluid->velocity_vertical,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    memcpy(fluid->velocity_radial,   fluid->scratch_a,
           sizeof fluid->velocity_radial);
    memcpy(fluid->velocity_vertical, fluid->scratch_b,
           sizeof fluid->velocity_vertical);
    enforce_velocity_boundaries(fluid);
}

/* Advect a passive scalar field BY THE CURRENT VELOCITY.  Each scalar
 * (temperature, density, ...) needs the same pattern: read into
 * scratch, swap back.  Helper takes the field pointer to deduplicate. */
static inline void advect_scalar_by_velocity(
        Fluid *fluid,
        float field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float step_seconds) {
    advect_field(fluid->scratch_a, field,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    memcpy(field, fluid->scratch_a,
           sizeof(float) * GRID_RADIAL_CELLS * GRID_VERTICAL_CELLS);
}

/*
 * fluid_step — one full physics tick.  Reads top-to-bottom as the
 * Stable Fluids algorithm pseudocode (T3).
 *
 * Pseudocode:
 *   apply_buoyancy                     ← hot cells gain upward impulse
 *   project_to_incompressible          ← cancel ∇·v that buoyancy added
 *   advect_velocity_self               ← v ← v∘(x − v·dt)
 *   project_to_incompressible          ← restore ∇·v ≈ 0
 *   advect_scalar_by_velocity(T)       ← carry temperature along v
 *   advect_scalar_by_velocity(ρ)       ← carry density along v
 *   apply_cool_and_decay               ← Newton cooling + ρ fade
 *
 * Refs [1] Stam 1999 — the canonical "advect-project-advect" cycle;
 * [4] Fedkiw 2001 for the buoyancy + temperature coupling.
 */
static void fluid_step(Fluid *fluid, float step_seconds) {
    apply_buoyancy             (fluid, step_seconds);
    project_to_incompressible  (fluid);

    advect_velocity_self       (fluid, step_seconds);
    project_to_incompressible  (fluid);  /* advection re-injects tiny ∇·v */

    advect_scalar_by_velocity  (fluid, fluid->temperature, step_seconds);
    advect_scalar_by_velocity  (fluid, fluid->density,     step_seconds);

    apply_cool_and_decay       (fluid, step_seconds);
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
/* Zero all fields and refill temperature with TEMPERATURE_AMBIENT.
 * The memset zeros velocity/density/scratch — exactly what we want —
 * but temperature needs the AMBIENT baseline (not 0K) so the
 * Newton-cooling target in apply_cool_and_decay stays well-defined. */
static inline void reset_field_to_ambient(Fluid *fluid) {
    memset(fluid, 0, sizeof *fluid);
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            fluid->temperature[i][j] = TEMPERATURE_AMBIENT;
}

/* Cell centre's world coords from grid indices.  +0.5 picks the centre
 * (not corner) of the cell — keeps the Gaussian symmetric around the
 * intended detonation point. */
static inline void cell_centre_world_coords(int i, int j,
                                             float *out_radius, float *out_altitude) {
    *out_radius   = ((float)i + 0.5f) * GRID_CELL_SIZE;
    *out_altitude = ((float)j + 0.5f) * GRID_CELL_SIZE;
}

/* Evaluate gauss(d) = exp(−d² / 2σ²) where d² = r² + (y − y₀)²
 * is the squared distance from cell (i, j) to the detonation point. */
static inline float gaussian_at_cell_distance(int i, int j,
                                               float detonation_altitude,
                                               float two_sigma_squared,
                                               float *out_dr, float *out_dy,
                                               float *out_distance_squared) {
    float radius, altitude;
    cell_centre_world_coords(i, j, &radius, &altitude);
    *out_dr               = radius;
    *out_dy               = altitude - detonation_altitude;
    *out_distance_squared = (*out_dr) * (*out_dr) + (*out_dy) * (*out_dy);
    return expf(- *out_distance_squared / two_sigma_squared);
}

/* Seed temperature and density at cell (i, j) from the Gaussian
 * intensity:
 *     T(r, y) = T_amb + (T_peak − T_amb) · gauss
 *     ρ(r, y) = ρ_peak · gauss                                       */
static inline void seed_gaussian_scalars_at(Fluid *fluid, int i, int j,
                                             float gaussian,
                                             const BlastParameters *blast) {
    fluid->temperature[i][j] =
        TEMPERATURE_AMBIENT +
        (blast->peak_temperature - TEMPERATURE_AMBIENT) * gaussian;
    fluid->density[i][j] = blast->peak_density * gaussian;
}

/* Seed initial outward radial velocity at one cell, but ONLY where the
 * Gaussian intensity is appreciable (>0.05).  Models the mechanical
 * shock wave that precedes buoyancy.  Without this seed, the bubble
 * rises with a clean leading vortex but no early expansion — looks
 * like a candle flame rather than a blast.
 *
 * The 1e-3 guard avoids div-by-zero exactly at the detonation point. */
static inline void seed_radial_outflow_at(Fluid *fluid, int i, int j,
                                           float gaussian,
                                           float dr, float dy,
                                           float distance_squared,
                                           float initial_outflow) {
    if (gaussian <= 0.05f) return;
    float distance = sqrtf(distance_squared);
    if (distance <= 1e-3f) return;
    float blast_speed = initial_outflow * gaussian;
    fluid->velocity_radial  [i][j] = (dr / distance) * blast_speed;
    fluid->velocity_vertical[i][j] = (dy / distance) * blast_speed;
}

/*
 * detonate_at_origin — the ONLY scripted moment of the simulation.
 * Paint a 3-D-symmetric Gaussian centred at the detonation altitude
 * plus a small radial outflow seed; physics handles the rest.
 *
 * Pseudocode:
 *   reset_field_to_ambient(fluid)
 *   two_sigma_squared = 2·σ²
 *   for each cell (i, j):
 *     gauss = gaussian_at_cell_distance(i, j, …)
 *     seed_gaussian_scalars_at(fluid, i, j, gauss, blast)
 *     seed_radial_outflow_at  (fluid, i, j, gauss, dr, dy, d², …)
 *   enforce_velocity_boundaries(fluid)
 *
 * The Gaussian shape isn't physically derived — it's the simplest
 * radially-symmetric blob that produces a believable mushroom when
 * the buoyancy step takes over after the first second or so.  Refs
 * [4] Fedkiw 2001 (buoyancy); [5] Nguyen et al. 2002 (fire core).
 */
static void detonate_at_origin(Fluid *fluid, const BlastParameters *blast) {
    reset_field_to_ambient(fluid);

    float two_sigma_squared = 2.0f * blast->sigma * blast->sigma;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float dr, dy, distance_squared;
            float gauss = gaussian_at_cell_distance(
                i, j, blast->detonation_altitude, two_sigma_squared,
                &dr, &dy, &distance_squared);

            seed_gaussian_scalars_at(fluid, i, j, gauss, blast);
            seed_radial_outflow_at  (fluid, i, j, gauss, dr, dy,
                                     distance_squared, blast->initial_outflow);
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
static inline float sample_density_at_world(float radius, float altitude) {
  if (radius < 0.0f || radius >= GRID_RADIAL_EXTENT)
    return 0.0f;
  if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT)
    return 0.0f;
  return sample_field_bilinear(g_fluid.density, radius * GRID_INV_CELL_SIZE,
                               altitude * GRID_INV_CELL_SIZE);
}

static inline float sample_temperature_at_world(float radius, float altitude) {
  if (radius < 0.0f || radius >= GRID_RADIAL_EXTENT)
    return TEMPERATURE_AMBIENT;
  if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT)
    return TEMPERATURE_AMBIENT;
  return sample_field_bilinear(g_fluid.temperature, radius * GRID_INV_CELL_SIZE,
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
/* Cylindrical-radius (axisymmetric) coordinate from a 3-D world point.
 *   r = √(x² + z²)
 * The fluid grid is 2-D (r, y); this is the projection from 3-D space. */
static inline float world_radius_at(Vec3 sample_point) {
    return sqrtf(sample_point.x * sample_point.x +
                 sample_point.z * sample_point.z);
}

/* Is the 3-D sample point INSIDE the axisymmetric simulation domain?
 * Rays that haven't entered yet (or have already left) can skip ahead
 * by RM_EMPTY_SKIP_FACTOR steps without missing any density. */
static inline bool sample_inside_simulation_domain(Vec3 sample_point,
                                                    float *out_radius) {
    if (sample_point.y < 0.0f || sample_point.y > GRID_VERTICAL_EXTENT)
        return false;
    *out_radius = world_radius_at(sample_point);
    return *out_radius <= GRID_RADIAL_EXTENT;
}

/* Linear normalisation of temperature into the emission range [0, 1].
 * 0 = ambient (no glow), 1 = peak (full fire colour).  Clamped — over-
 * heated cells (which can happen during a brief overshoot after
 * detonation) still contribute full brightness, not more. */
static inline float temperature_to_emission_normalised(float temperature,
                                                        float inverse_temp_range) {
    float emission = (temperature - TEMPERATURE_AMBIENT) * inverse_temp_range;
    return clampf(emission, 0.0f, 1.0f);
}

/* Accumulate one ray-step's contribution to the running luminance
 * integrals.  Implements the discrete emission-absorption volume
 * rendering integral [7] Max 1995:
 *
 *   L(t) += T(t) · κ(t) · S(t) · Δt
 *
 * where T is the running transmittance (how much light hasn't yet
 * been absorbed), κ·Δt is the optical-depth step, S is the local
 * emission source.  hot_luminance is a separate integral over the
 * EMISSION component only — used downstream to pick the smoke→fire
 * colour ramp in decorate_volume_pixel. */
static inline void accumulate_emission_absorption_step(
        float transmittance, float optical_depth_step,
        float emission, float source,
        float *running_total_luminance, float *running_hot_luminance) {
    *running_total_luminance += transmittance * optical_depth_step * source;
    *running_hot_luminance   += transmittance * optical_depth_step *
                                 emission * RM_EMISSION_GAIN;
}

/* Beer-Lambert attenuation: transmittance decays exponentially with
 * the accumulated optical depth.  T ← T · exp(−κ·Δt).  [8] any optics
 * text; [11] Wikipedia Beer-Lambert. */
static inline float beer_lambert_attenuate(float transmittance,
                                            float optical_depth_step) {
    return transmittance * expf(-optical_depth_step);
}

/*
 * raymarch_volume — Beer-Lambert volumetric integration along one ray.
 *
 * Pseudocode:
 *   t = near; transmittance = 1; total = hot = 0
 *   for step = 0..MAX_STEPS:
 *     sample_point = origin + t·direction
 *     if not sample_inside_simulation_domain(sample_point):
 *       skip-ahead and continue
 *     density = sample_density_at_world(...)
 *     if density < ε: tiny step and continue
 *     temperature = sample_temperature_at_world(...)
 *     emission    = temperature_to_emission_normalised(...)
 *     source      = emission · GAIN + ambient_floor
 *     optical_depth_step = density · STEP · GAIN
 *     accumulate_emission_absorption_step(transmittance, ...)
 *     transmittance = beer_lambert_attenuate(transmittance, ...)
 *     if transmittance < ε:  early-out (opaque)
 *     t += STEP
 *   write out totals
 *
 * Outputs:
 *   *out_total_luminance — total smoke + fire luminance
 *   *out_hot_luminance   — fire-only luminance (drives palette choice)
 *
 * Refs [6] Quilez volumetric raymarching; [7] Max 1995 §3
 *   (emission-absorption); [11] Beer-Lambert law.
 */
static void raymarch_volume(Vec3 origin, Vec3 direction,
                            float *out_total_luminance,
                            float *out_hot_luminance) {
    float t                   = RM_RAY_NEAR;
    float transmittance       = 1.0f;
    float total_luminance     = 0.0f;
    float hot_luminance       = 0.0f;
    float inverse_temp_range  = 1.0f / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);

    for (int step = 0; step < RM_MAX_STEPS; step++) {
        Vec3  sample_point = v3add(origin, v3scale(t, direction));
        float radius;

        /* OUTSIDE the simulation cylinder — skip ahead in big steps. */
        if (!sample_inside_simulation_domain(sample_point, &radius)) {
            t += RM_STEP * RM_EMPTY_SKIP_FACTOR;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* INSIDE the cylinder but the cloud is sparse here — small step. */
        float density = sample_density_at_world(radius, sample_point.y);
        if (density < RM_EMPTY_DENSITY_EPS) {
            t += RM_STEP;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* INSIDE the cloud — read temperature and integrate one step. */
        float temperature        = sample_temperature_at_world(radius,
                                                                sample_point.y);
        float emission           = temperature_to_emission_normalised(
                                       temperature, inverse_temp_range);
        float source             = emission * RM_EMISSION_GAIN + RM_AMBIENT_FLOOR;
        float optical_depth_step = density * RM_STEP * RM_DENSITY_GAIN;

        accumulate_emission_absorption_step(transmittance, optical_depth_step,
                                             emission, source,
                                             &total_luminance, &hot_luminance);
        transmittance = beer_lambert_attenuate(transmittance, optical_depth_step);

        if (transmittance < RM_OPAQUE_TRANSMITTANCE_EPS) break;
        if (t > RM_RAY_FAR)                              break;
        t += RM_STEP;
    }

    *out_total_luminance = total_luminance;
    *out_hot_luminance   = hot_luminance;
}

/* ===================================================================== */
/* §17  camera — orthonormal basis + per-pixel ray (T9)                   */
/* ===================================================================== */

/*
 * Camera — orthonormal basis + projection params for the volume renderer.
 *
 * Intent
 *   For each terminal cell the renderer needs to construct a world-
 *   space ray.  Camera carries the standard pinhole-camera triplet —
 *   origin + (forward, right, up) basis — plus enough projection
 *   information that pixel_to_ray() can produce one normalised ray
 *   direction per cell with no extra parameters.
 *
 * Why right and up are stored (not derived)
 *   The camera always looks at the mushroom column from a fixed
 *   "slightly above and behind" pose; the right/up axes are
 *   constructed ONCE in build_camera_basis() and reused for every
 *   pixel.  Precomputing them keeps the per-pixel loop free of cross
 *   products and length normalisations.
 *
 * Why aspect_factor (not just fov_tangent)
 *   Terminal cells are roughly 2:1 (height:width) so equal pixel
 *   counts horizontally and vertically mean different angular
 *   extents.  aspect_factor folds this 2:1 correction in so the
 *   rendered scene looks proportioned, not vertically stretched.
 *
 * Reference [6] Quilez volumetric raymarching for the per-pixel ray
 *   construction pattern Camera implements.
 */
typedef struct {
    Vec3  origin;          /* world-space camera position             */
    Vec3  forward;         /* unit, points from origin toward target  */
    Vec3  right;           /* unit, world-x in screen space           */
    Vec3  up;              /* unit, world-y in screen space           */
    float fov_tangent;     /* tan(fov/2) for the horizontal axis      */
    float aspect_factor;   /* terminal-cell aspect correction         */
} Camera;

static Camera build_camera_basis(float distance_behind, int visible_rows,
                                 int visible_cols) {
  Camera cam;
  cam.origin = v3(0.0f, CAM_HEIGHT, -distance_behind);
  Vec3 look_at = v3(0.0f, CAM_LOOK_AT_HEIGHT, 0.0f);

  cam.forward = v3normalise(v3sub(look_at, cam.origin));
  Vec3 world_up = v3(0.0f, 1.0f, 0.0f);
  cam.right = v3normalise(v3cross(cam.forward, world_up));
  cam.up = v3cross(cam.right, cam.forward);

  cam.fov_tangent = tanf(CAM_FIELD_OF_VIEW_DEG * (float)M_PI / 180.0f * 0.5f);
  cam.aspect_factor =
      ((float)visible_rows * TERMINAL_CELL_ASPECT) / (float)visible_cols;
  return cam;
}

static Vec3 ray_for_pixel(int col, int row, int cols, int visible_rows,
                          const Camera *cam) {
  float u = ((float)col + 0.5f) / (float)cols * 2.0f - 1.0f;
  float v = -(((float)row + 0.5f) / (float)visible_rows * 2.0f - 1.0f);

  Vec3 ray =
      v3add(cam->forward,
            v3add(v3scale(u * cam->fov_tangent, cam->right),
                  v3scale(v * cam->fov_tangent * cam->aspect_factor, cam->up)));
  return v3normalise(ray);
}

/* ===================================================================== */
/* §18  cell_decorate — (lum, hot) → glyph + colour + attr (T10)          */
/* ===================================================================== */

/*
 * Cell — one cell's drawing instruction, derived from raymarch luminance.
 *
 * Intent
 *   The raymarch loop returns two scalars per cell: total luminance
 *   (smoke opacity) and hot luminance (fire emission).  Cell bundles
 *   the three values the renderer needs (glyph, colour pair, attr)
 *   plus a SKIP flag for cells fully transparent against the
 *   background — those are left at default-bg so the terminal can
 *   skip writing them entirely.
 *
 * Why a struct (not three return values)
 *   C functions return one value; bundling glyph + pair + attr + skip
 *   into one POD struct lets decorate_volume_pixel return them all in
 *   one shot.  Callers read `if (c.skip) continue;` then a single
 *   `mvaddch_with_attr(... c.glyph, c.pair | c.attr ...)`.
 *
 * Reference [9] Bourke for the luminance → glyph design.
 */
typedef struct {
    char   glyph;     /* one of LUMINANCE_GLYPHS[]                  */
    int    pair;      /* colour-pair index (theme ramp)             */
    attr_t attr;      /* extra attributes (A_BOLD for hottest slot) */
    bool   skip;      /* if true, leave the cell blank (transparent) */
} Cell;

/* Map total luminance ∈ [0, ∞) to a GLYPH SLOT 0..GLYPH_SLOT_COUNT-1.
 * Total luminance can in principle exceed the clamp (very thick + very
 * hot cloud), but visually we cap the ramp at PIXEL_LUMINANCE_CLAMP
 * so saturated regions still pick the BRIGHTEST glyph rather than
 * spilling past the table. */
static inline int luminance_to_glyph_slot(float total_luminance) {
    float lum_normalised = total_luminance / PIXEL_LUMINANCE_CLAMP;
    if (lum_normalised > 1.0f) lum_normalised = 1.0f;
    return to_slot(lum_normalised);
}

/* Map "hot fraction" — hot_luminance / total_luminance — to a PALETTE
 * SLOT.  0 = pure smoke (cool end of ramp), 1 = pure fire (hot end).
 * The +0.001 guard prevents division by zero for cells whose total
 * luminance was just barely above PIXEL_VISIBLE_LUM_EPS.            */
static inline int hot_fraction_to_palette_slot(float hot_luminance,
                                                float total_luminance) {
    float hot_fraction = hot_luminance / (total_luminance + 0.001f);
    if (hot_fraction > 1.0f) hot_fraction = 1.0f;
    if (hot_fraction < 0.0f) hot_fraction = 0.0f;
    return to_slot(hot_fraction);
}

/* Pick the right cell attribute based on the theme polarity:
 *   Normal themes — A_BOLD on hot/bright tiers (extra fire punch),
 *                    A_DIM on cool tiers (fading wisps), else A_NORMAL.
 *   Inverted themes — A_NORMAL only.  A_DIM / A_BOLD on a light fg
 *                      over a white bg INVERT the brightness intent,
 *                      so we skip them entirely.                   */
static inline attr_t pick_themed_cell_attribute(int slot_lum, int slot_hot,
                                                 bool inverted_theme) {
    if (inverted_theme) return A_NORMAL;
    if (slot_hot >= 6 || slot_lum >= 6) return A_BOLD;
    if (slot_lum <= 1)                  return A_DIM;
    return A_NORMAL;
}

/*
 * decorate_volume_pixel — (lum, hot) → glyph + colour + attribute.
 *
 * Pseudocode:
 *   if total_luminance < ε:  skip cell (transparent)
 *   slot_lum = luminance_to_glyph_slot(total_luminance)
 *   slot_hot = hot_fraction_to_palette_slot(hot_lum, total_lum)
 *   attr     = pick_themed_cell_attribute(slot_lum, slot_hot, inverted)
 *   return Cell { glyph = LUMINANCE_GLYPHS[slot_lum],
 *                 pair  = PAIR_RAMP_BASE + slot_hot,
 *                 attr  = attr }
 *
 * The "luminance picks glyph, hot picks colour" split lets one
 * dimension (thickness) drive READABILITY and the other (heat) drive
 * VISUAL FEEL.  Smoke and fire share the same glyph ramp but read as
 * very different things because of the colour swap.
 */
static Cell decorate_volume_pixel(float total_luminance, float hot_luminance,
                                  bool inverted_theme) {
    if (total_luminance < PIXEL_VISIBLE_LUM_EPS)
        return (Cell){ .skip = true };

    int    slot_lum = luminance_to_glyph_slot(total_luminance);
    int    slot_hot = hot_fraction_to_palette_slot(hot_luminance, total_luminance);
    attr_t attr     = pick_themed_cell_attribute(slot_lum, slot_hot, inverted_theme);

    return (Cell){
        .glyph = LUMINANCE_GLYPHS[slot_lum],
        .pair  = PAIR_RAMP_BASE + slot_hot,
        .attr  = attr,
        .skip  = false,
    };
}

/* emit_cell — paint one cell with attron/attroff batched on
 * (pair, attr) change.  Halves attribute thrash on uniform regions. */
static void emit_cell(int row, int col, Cell cell, int *last_pair,
                      attr_t *last_attr) {
  if (cell.skip)
    return;
  if (cell.pair != *last_pair || cell.attr != *last_attr) {
    if (*last_pair >= 0)
      attroff(COLOR_PAIR(*last_pair) | *last_attr);
    attron(COLOR_PAIR(cell.pair) | cell.attr);
    *last_pair = cell.pair;
    *last_attr = cell.attr;
  }
  mvaddch(row, col, (chtype)(unsigned char)cell.glyph);
}

/* ===================================================================== */
/* §19  render_volume — full screen of raymarched cells                   */
/* ===================================================================== */

/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Scene composes the user's choices (preset blast, theme, debug
 *   mode), the simulation clock (so multiple ticks per frame can
 *   work), and the camera distance.  The actual fluid field is in
 *   g_fluid (file-scope) since every pass loops over the whole grid
 *   and a struct pointer would just add indirection.
 *
 * Locality (sim vs render)
 *   Fields are GROUPED EXPLICITLY so a reader can tell which
 *   subsystem touches each one:
 *     - scene_tick / fluid_advect / project reads it     → simulation
 *     - render_volume_view / decorate_volume_pixel reads → rendering
 *     - both sides bound their loops (cols, rows)        → geometry
 *     - paused gates the tick AND drives the HUD tag     → control
 *
 *   Mis-classifying theme_index as sim (and re-running advect on a
 *   theme change) would break the architectural invariant that "the
 *   look is decoupled from the physics" — same blast with the same
 *   sim_rate must evolve identically regardless of theme.
 *
 * Why these specific fields and no others
 *   - paused                  gate for scene_tick + HUD "PAUSED" tag.
 *   - theme_index             pure render — palette ramp selector.
 *   - blast_type              user's preset choice; the LAST detonation
 *                              uses this row of BLAST_PRESETS[].
 *   - debug_mode              pure render — picks between full
 *                              raymarch and 2-D diagnostic overlays.
 *   - cols, rows              terminal extent; both sim (camera basis
 *                              aspect) and render (loop bounds) read.
 *   - simulation_time_seconds monotonic clock used by §emitter for the
 *                              time-varying turbulence input.
 *   - simulation_rate         user-adjustable sim Hz (faster mushroom).
 *   - simulation_step_accumulator  fixed-step accumulator for the
 *                              dt-decoupled tick loop.
 *   - camera_distance         user-adjustable zoom (zoom in/out keys).
 *
 * Things that DO NOT live here
 *   - Fluid field state                  → file-scope g_fluid
 *   - 5 blast presets / 6 themes / glyph ramp  → file-scope tables
 *   - Render-frame timing / FPS counter  → locals in main()
 *   - Signal flags                       → file-scope volatile
 *
 * Reference [1] Stam Stable Fluids for the field-state ownership
 *   pattern; [10] Raymond for the scene-paint pipeline.
 */
typedef struct {
    /* ── Control state (gates tick + drives HUD) ────────────────── */
    bool      paused;

    /* ── Pure render state (read by render_volume_view) ─────────── *
     * Changing these MUST NOT touch the fluid state.               */
    int       theme_index;
    DebugMode debug_mode;

    /* ── Simulation selector (which preset to (re)detonate with) ─ */
    BlastType blast_type;

    /* ── Shared geometry (sim AND render) ──────────────────────── */
    int       cols, rows;            /* terminal extent in cells   */

    /* ── Simulation timing ────────────────────────────────────── */
    float     simulation_time_seconds;  /* monotonic clock         */
    float     simulation_rate;          /* sim Hz                  */
    float     simulation_step_accumulator;  /* fixed-step accum    */

    /* ── Pure render: camera placement ─────────────────────────── */
    float     camera_distance;
} Scene;

/* For "inverted" themes (where the background is white-ish), pre-fill
 * the visible field with the background colour so cells we skip (the
 * transparent ones) sit against the right backdrop. */
static inline void prefill_inverted_background(int visible_rows, int cols,
                                                int y_offset) {
    attron(COLOR_PAIR(PAIR_RAMP_BASE));
    for (int row = 0; row < visible_rows; row++)
        for (int col = 0; col < cols; col++)
            mvaddch(row + y_offset, col, ' ');
    attroff(COLOR_PAIR(PAIR_RAMP_BASE));
}

/* Raymarch ONE pixel and convert its (total, hot) luminance into a
 * Cell drawing instruction.  Pure function — no side effects. */
static inline Cell raymarch_and_decorate_pixel(int col, int row,
                                                int cols, int visible_rows,
                                                const Camera *cam,
                                                bool inverted_theme) {
    Vec3  ray = ray_for_pixel(col, row, cols, visible_rows, cam);
    float total_lum, hot_lum;
    raymarch_volume(cam->origin, ray, &total_lum, &hot_lum);
    return decorate_volume_pixel(total_lum, hot_lum, inverted_theme);
}

/* Scan the visible field rect, raymarch every cell, paint via the
 * attribute-batching emit_cell to halve attron/attroff overhead on
 * uniform regions. */
static inline void paint_raymarched_field(const Camera *cam,
                                           int visible_rows, int cols,
                                           int y_offset, bool inverted,
                                           int *last_pair, attr_t *last_attr) {
    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < cols; col++) {
            Cell cell = raymarch_and_decorate_pixel(col, row, cols,
                                                    visible_rows, cam, inverted);
            emit_cell(y_offset + row, col, cell, last_pair, last_attr);
        }
    }
}

/*
 * render_volume_view — full screen of raymarched volume cells.
 *
 * Pseudocode:
 *   visible_rows = rows - HUD_RESERVED_ROWS
 *   cam = build_camera_basis(camera_distance, visible_rows, cols)
 *   if theme is inverted: prefill_inverted_background(...)
 *   paint_raymarched_field(cam, visible_rows, cols, ...)
 *   close out any open attron pair
 */
static void render_volume_view(const Scene *scene) {
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return;

    Camera cam = build_camera_basis(scene->camera_distance,
                                     visible_rows, scene->cols);
    bool   inverted = THEMES[scene->theme_index].inverted_background;
    int    y_offset = 1;          /* leave row 0 for status */

    if (inverted)
        prefill_inverted_background(visible_rows, scene->cols, y_offset);

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;
    paint_raymarched_field(&cam, visible_rows, scene->cols, y_offset, inverted,
                            &last_pair, &last_attr);

    if (last_pair >= 0)
        attroff(COLOR_PAIR(last_pair) | last_attr);
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

/* Convert screen cell (row, col) to fluid-grid cell (grid_i, grid_j).
 * The y-axis is FLIPPED so ground (grid_j = 0) appears at the BOTTOM
 * of the screen rather than the top.  Linear stretch on both axes. */
static inline void screen_cell_to_fluid_grid(int row, int col,
                                              int visible_rows, int cols,
                                              int *out_grid_i, int *out_grid_j) {
    *out_grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
    *out_grid_i =  col                     * GRID_RADIAL_CELLS  / cols;
}

/* Render-pipeline init: bail-out check + y-offset for HUD reservation.
 * Returns the visible row count, or 0 if there isn't enough screen
 * space to draw anything. */
static inline int begin_debug_overlay(const Scene *scene, int *out_y_offset) {
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return 0;
    *out_y_offset = 1;        /* leave row 0 for status */
    return visible_rows;
}

/* Close out any open attron from emit_cell's batching. */
static inline void end_debug_overlay(int last_pair, attr_t last_attr) {
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/*
 * render_debug_density — direct 2-D view of fluid->density[i][j].
 *
 * Pseudocode:
 *   visible_rows = begin_debug_overlay(scene, &y_offset)
 *   for each screen cell (row, col):
 *     (grid_i, grid_j) = screen_cell_to_fluid_grid(row, col, ...)
 *     density   = g_fluid.density[grid_i][grid_j]
 *     normalise = clamp(density / DENSITY_PEAK, 0, 1)
 *     paint glyph + ramp slot
 *   end_debug_overlay
 */
static void render_debug_density(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);
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
    end_debug_overlay(last_pair, last_attr);
}

/*
 * render_debug_temperature — direct 2-D view of fluid->temperature[i][j].
 *
 * Pseudocode:
 *   like render_debug_density but normalises temperature instead:
 *     normalise = (T − T_amb) / (T_peak − T_amb)   ← maps to [0, 1]
 */
static void render_debug_temperature(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);
            float temperature = g_fluid.temperature[grid_i][grid_j];
            float normalise   = (temperature - TEMPERATURE_AMBIENT) /
                                (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);
            normalise         = clampf(normalise, 0.0f, 1.0f);
            int   slot        = to_slot(normalise);

            if (slot == 0 && normalise < 0.01f) continue;

            Cell c = { .glyph = LUMINANCE_GLYPHS[slot],
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    end_debug_overlay(last_pair, last_attr);
}

/* arrow_for_velocity — pick an ASCII arrow that points in the same
 * direction as a 2-D velocity vector.  8 cardinal/intercardinal
 * directions; magnitude near zero → space (invisible). */
static char arrow_for_velocity(float vx, float vy) {
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 0.05f) return ' ';
    float angle  = atan2f(vy, vx);
    int   octant = (int)((angle + (float)M_PI) / ((float)M_PI / 4.0f) + 0.5f) % 8;
    static const char ARROWS[8] = { '<', '/', 'v', '\\', '>', '/', '^', '\\' };
    return ARROWS[octant];
}

/* |v| → ramp slot 0..GLYPH_SLOT_COUNT-1.  Magnitude is normalised by a
 * fixed 4.0 reference speed (faster than which clips to the brightest
 * slot).  The clip is intentional — overflowing velocities (rare) get
 * full visual weight rather than spilling past the table. */
static inline int velocity_magnitude_to_slot(float vx, float vy) {
    float magnitude = sqrtf(vx * vx + vy * vy);
    float normalise = clampf(magnitude / 4.0f, 0.0f, 1.0f);
    return to_slot(normalise);
}

/*
 * render_debug_velocity — direct 2-D view of (vᵣ, vᵧ) as ASCII arrows.
 *
 * Pseudocode:
 *   for each screen cell (row, col):
 *     (grid_i, grid_j) = screen_cell_to_fluid_grid(row, col, ...)
 *     (vr, vy) from fluid
 *     glyph = arrow_for_velocity(vr, vy)
 *     if glyph == ' ': skip
 *     slot  = velocity_magnitude_to_slot(vr, vy)
 *     paint arrow with ramp colour, bold for top tiers
 */
static void render_debug_velocity(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);

            float vr    = g_fluid.velocity_radial  [grid_i][grid_j];
            float vy    = g_fluid.velocity_vertical[grid_i][grid_j];
            char  glyph = arrow_for_velocity(vr, vy);
            if (glyph == ' ') continue;

            int slot = velocity_magnitude_to_slot(vr, vy);
            Cell c   = { .glyph = glyph,
                         .pair  = PAIR_RAMP_BASE + slot,
                         .attr  = (slot >= 5) ? A_BOLD : A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    end_debug_overlay(last_pair, last_attr);
}

/* ===================================================================== */
/* §21  render_dispatch — pick volume vs debug per active mode            */
/* ===================================================================== */

static void render_active_view(const Scene *scene) {
  switch (scene->debug_mode) {
  case DEBUG_NORMAL:
    render_volume_view(scene);
    break;
  case DEBUG_DENSITY:
    render_debug_density(scene);
    break;
  case DEBUG_TEMPERATURE:
    render_debug_temperature(scene);
    break;
  case DEBUG_VELOCITY:
    render_debug_velocity(scene);
    break;
  default:
    render_volume_view(scene);
    break;
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
                     double real_fps, int target_render_fps) {
  char status_text[200];
  snprintf(status_text, sizeof status_text,
           " %5.1f fps  %3d Hz  blast:%s  theme:%s  debug:%s  "
           "t:%6.2fs  rate:%4.2f  dist:%4.1f  %s ",
           real_fps, target_render_fps,
           BLAST_PRESETS[scene->blast_type].display_name,
           THEMES[scene->theme_index].display_name,
           DEBUG_MODE_NAMES[scene->debug_mode],
           (double)scene->simulation_time_seconds,
           (double)scene->simulation_rate, (double)scene->camera_distance,
           scene->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status_text);
  if (slen > term_cols)
    slen = term_cols;

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

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we
 * keep only cell dimensions for HUD placement and camera aspect.
 *
 * Render pipeline (one frame): erase → render_volume_view → hud_*
 *   → wnoutrefresh(stdscr) → doupdate().  Diff-only writes to the
 *   terminal.  See [10] Raymond §11.
 */
typedef struct {
    int rows;   /* terminal height in cells (getmaxyx)             */
    int cols;   /* terminal width  in cells (getmaxyx)             */
} Screen;

static void screen_init(Screen *screen) {
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

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, const Scene *scene,
                                 double real_fps, int target_render_fps) {
  erase();
  render_active_view(scene);
  hud_draw(screen->rows, screen->cols, scene, real_fps, target_render_fps);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §24  scene — per-frame state + tick + scene helpers                    */
/* ===================================================================== */

static void scene_init(Scene *scene, int cols, int rows) {
  memset(scene, 0, sizeof *scene);
  scene->paused = false;
  scene->theme_index = 0;
  scene->blast_type = BLAST_STANDARD;
  scene->debug_mode = DEBUG_NORMAL;
  scene->cols = cols;
  scene->rows = rows;
  scene->simulation_time_seconds = 0.0f;
  scene->simulation_rate = SIM_RATE_DEFAULT;
  scene->simulation_step_accumulator = 0.0f;
  scene->camera_distance = CAM_DISTANCE_DEFAULT;

  detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_resize(Scene *scene, int cols, int rows) {
  scene->cols = cols;
  scene->rows = rows;
}

static void scene_redetonate(Scene *scene) {
  scene->simulation_time_seconds = 0.0f;
  scene->simulation_step_accumulator = 0.0f;
  detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_cycle_blast(Scene *scene, int direction) {
  int new_index = (int)scene->blast_type + direction;
  while (new_index < 0)
    new_index += BLAST_TYPE_COUNT;
  scene->blast_type = (BlastType)(new_index % BLAST_TYPE_COUNT);
  scene_redetonate(scene);
}

/*
 * scene_tick — accumulate real time into sim time and run as many
 * fixed-dt fluid steps as fit.  Fixed-dt accumulator pattern keeps
 * cloud morphology FRAME-RATE-INDEPENDENT.
 */
static void scene_tick(Scene *scene, float dt_real_seconds) {
  if (scene->paused)
    return;

  float dt_sim = dt_real_seconds * scene->simulation_rate;
  if (dt_sim > SIM_DT_MAX_REAL)
    dt_sim = SIM_DT_MAX_REAL;

  scene->simulation_step_accumulator += dt_sim;
  scene->simulation_time_seconds += dt_sim;

  while (scene->simulation_step_accumulator >= SIM_DT) {
    fluid_step(&g_fluid, SIM_DT);
    scene->simulation_step_accumulator -= SIM_DT;
  }
}

/* ===================================================================== */
/* §25  app — main loop + signals + key handling                          */
/* ===================================================================== */

/*
 * App — top-level container; lives in BSS as the single g_app instance.
 *
 * Intent
 *   Signal handlers (on_exit_signal, on_resize_signal) need to reach
 *   state that the main loop polls.  A global App + handlers that
 *   flip its volatile sig_atomic_t flags is the standard POSIX
 *   "wake the main loop" pattern.
 *
 * Why the volatile sig_atomic_t flags
 *   POSIX permits signal handlers to write ONLY sig_atomic_t values
 *   with simple assignments — anything wider is UB.  volatile forces
 *   every read in the main loop to go back to memory (no compiler
 *   caching into a register across signal arrival).
 *
 * Why target_render_fps lives here (not in Scene)
 *   target_render_fps is a frame-loop concern — it picks the inner-
 *   loop sleep budget — and has no meaning inside scene_tick which
 *   receives the resulting dt as a parameter.  Putting it in App
 *   keeps Scene free of timing detail.
 */
typedef struct {
    Scene  scene;                          /* world + control state   */
    Screen screen;                         /* terminal extent         */
    int    target_render_fps;              /* render frame cap        */
    volatile sig_atomic_t running;         /* SIGINT/TERM clears this */
    volatile sig_atomic_t need_resize;     /* SIGWINCH sets this      */
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

static void app_handle_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scene = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    scene->paused = !scene->paused;
    break;
  case 'r':
  case 'R':
    scene_redetonate(scene);
    break;

  case 'n':
    scene_cycle_blast(scene, +1);
    break;
  case 'N':
    scene_cycle_blast(scene, -1);
    break;

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
    scene->debug_mode = (DebugMode)((scene->debug_mode + DEBUG_MODE_COUNT - 1) %
                                    DEBUG_MODE_COUNT);
    break;

  case '=':
  case '+':
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

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(screen_cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->target_render_fps = TARGET_RENDER_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

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
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;
    float dt_real_seconds = (float)dt_ns / (float)NS_PER_SEC;

    /* ── physics ── */
    scene_tick(&app->scene, dt_real_seconds);

    /* ── fps window ── */
    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= FPS_DISPLAY_UPDATE_MS * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── render ── */
    screen_present_frame(&app->screen, &app->scene, measured_fps,
                         app->target_render_fps);

    /* ── frame cap ── */
    int64_t target_frame_ns = TICK_NS(app->target_render_fps);
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < target_frame_ns)
      clock_sleep_ns(target_frame_ns - spent);
  }

  return 0;
}
