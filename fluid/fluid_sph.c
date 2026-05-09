/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fluid_sph.c — interactive Smoothed Particle Hydrodynamics (SPH) playground
 *
 * DEMO: A pool of 800-3000 particles behaves like compressible fluid.
 *       Drop a blob, slam two fronts together, fire a fountain, watch
 *       a rain curtain pile up.  Five preset SCENES (1..5), TEN colour
 *       themes (t to cycle), GRAVITY and VISCOSITY toggleable on the
 *       fly (g, v).  Every particle continuously estimates its local
 *       DENSITY from neighbours; pressure and cohesion forces emerge
 *       from density excess / shortfall.  No grid solver, no Navier-
 *       Stokes — just N particles + a smoothing kernel + a spatial
 *       hash for speed.  Yet you get fluid-like behaviour: surface
 *       tension, sloshing, splashing, settling.
 *
 * Study alongside:
 *   fluid/navier_stokes.c     — the GRID-based alternative; same
 *                                physics goal but Eulerian instead of
 *                                Lagrangian.  Read T1 below to compare.
 *   fluid/cfl_stability_explorer.c
 *                              — CFL number, time-step stability;
 *                                applies to SPH too (T8).
 *   particle_systems/         — particle pools without inter-particle
 *                                forces (sparks, fireworks, embers).
 *   physics/cloth.c           — particles + spring forces; SPH's
 *                                pressure force is the analogue of
 *                                cloth's spring force.
 *
 * Section map:
 *   §1  config          — every tunable constant, grouped by subsystem
 *   §2  clock           — monotonic ns timer + sleep
 *   §3  rng             — small random helper
 *   §4  themes          — 10 palette triples + names
 *   §5  colors          — pair init + theme apply
 *   §6  particle        — Particle struct + global pool + spawn helpers
 *   §7  kernel          — SPH smoothing kernel (compact-support tent²)
 *   §8  grid            — spatial-hash linked-list grid
 *   §9  density_pass    — neighbour density estimation
 *   §10 forces_pass     — pressure + viscosity acceleration
 *   §11 integrate_pass  — symplectic Euler + wall bounce
 *   §12 sph_step        — full physics tick (4 passes)
 *   §13 scenes          — 5 scene loaders
 *   §14 scene           — scene state + tick orchestrator
 *   §15 render_particles— density → glyph + colour
 *   §16 render_border   — frame around the simulation area
 *   §17 hud             — top status + bottom hint strip
 *   §18 screen          — ncurses init / cleanup
 *   §19 app             — main loop + signals + input
 *   §20 main            — entry point
 *
 * Keys:
 *   q / Q / ESC   quit
 *   space         pause / resume
 *   1 .. 5        load scene (blob / column / fountain / collision / rain)
 *   g             toggle gravity
 *   v             toggle viscosity
 *   r             reset (reload current scene)
 *   b             spawn a small extra blob at top
 *   t             cycle colour theme
 *   ] / [         simulation Hz +/-
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/fluid_sph.c -o fluid_sph \
 *       -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      T1-T2 establish the SPH WORLDVIEW (Lagrangian particles + a
 *      smoothing kernel).  T3-T5 derive the physics passes.  T6
 *      explains symplectic Euler.  T7 explains the spatial hash for
 *      speed.  T8 closes the loop with stability conditions.
 *   2. §6 particle + §1 config — the data structure + the knobs.
 *      Together they describe the entire simulation state.
 *   3. §12 sph_step — five lines: build_grid, density_pass,
 *      forces_pass, integrate_pass.  The whole physics is THERE; §9-§11
 *      are the bodies.
 *   4. §9 density_pass — read AFTER tutorial T3.
 *   5. §10 forces_pass — read AFTER tutorials T4 and T5.
 *   6. §11 integrate_pass — read AFTER tutorial T6.
 *   7. §13-§14 scenes / scene — orchestration.
 *   8. §15-§17 rendering / HUD — visual layer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   particle_pool[]            global array of every Particle
 *   particle_count             how many slots are in use
 *   p->pos_col, p->pos_row     position (in cell units)
 *   p->vel_col, p->vel_row     velocity (cells/step)
 *   p->accel_col, p->accel_row force accumulator for one step
 *   p->density_estimate        ρᵢ = Σ kernel²
 *
 *   distance_cells             |pᵢ - pⱼ| in cell units
 *   kernel_signed              w = d/H - 1 (negative inside support)
 *   kernel_squared             w² (always non-negative; used for ρ)
 *
 *   grid_head[gy][gx]          first particle index in cell (gy, gx)
 *   grid_next[i]               next particle in same cell as i, or -1
 *
 *   scene.id                   current scene (1..SCENE_COUNT)
 *   scene.theme_index          current theme (0..THEME_COUNT-1)
 *   scene.paused               run/pause toggle
 *
 *   sim_steps_per_second       physics tick rate (Hz)
 *
 * Background you need
 * ───────────────────
 *   - Newton's second law (F = m·a; we treat m = 1).
 *   - Vectors as (col, row) or (x, y) interchangeably; physics is 2-D
 *     in cell space (no separate pixel coordinates).
 *   - Gradients / forces from a potential ("denser than rest" → a
 *     repulsive force, a la a spring).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Navier-Stokes equations.  SPH is a PARTICLE method; it never
 *     writes ∂v/∂t = -∇p/ρ + ν∇²v explicitly.  See
 *     fluid/navier_stokes.c if you want the grid solver.
 *   - True incompressibility / projection step.  This file uses the
 *     simpler TAIT equation of state (T4), which makes the fluid
 *     SLIGHTLY compressible but cheap.
 *   - Variable density / multiphase fluid.  Single fluid, unit mass.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm     : Smoothed Particle Hydrodynamics (SPH).  A LAGRANGIAN
 *                 fluid simulator: instead of tracking values on a
 *                 fixed grid (Eulerian), we track many small parcels
 *                 of fluid that MOVE WITH the flow.  Each particle
 *                 carries position, velocity, and a locally-estimated
 *                 density.  Forces between nearby particles cause
 *                 collective behaviour that LOOKS LIKE a fluid.
 *
 * Per tick:     1. BUILD SPATIAL HASH — map each particle to a grid
 *                  cell so neighbour lookup becomes O(1) amortised.
 *               2. DENSITY PASS — for each particle, sum kernel²
 *                  contributions from all neighbours.
 *               3. FORCES PASS — for each pair within kernel range:
 *                    a) PRESSURE FORCE proportional to (ρ_rest -
 *                       ρᵢ - ρⱼ).  Repulsive when overcrowded,
 *                       attractive when sparse — the implicit
 *                       Tait equation of state.
 *                    b) VISCOSITY FORCE proportional to (vⱼ - vᵢ).
 *                       Smooths velocity field; prevents tunnelling.
 *               4. INTEGRATE — symplectic Euler:
 *                       v += a·dt
 *                       x += v·dt    (uses NEW v, not old)
 *                  Then bounce off walls with energy loss.
 *
 * Math basis    : Compact-support tent kernel:
 *                       w(d) = (d/H) - 1     for d < H
 *                            = 0             for d ≥ H  (out of range)
 *                 Density:   ρᵢ = Σⱼ w(d_ij)²
 *                 Pressure:  F_pressure ∝ w · (ρ_rest - ρᵢ - ρⱼ)
 *                 Viscosity: F_visc     ∝ |w| · (vⱼ - vᵢ)
 *
 * Performance   : NAÏVE pair search is O(N²): every particle checks
 *                 every other.  With N=2000 that's 4M comparisons per
 *                 pass.  Replaced by a spatial-hash grid with cell
 *                 size ≥ kernel radius H.  Each particle now walks the
 *                 3×3 block of cells around it: ~50× speedup typical.
 *                 Hot-path numbers at N=1500: ~50k particle-pair
 *                 visits per pass; sub-millisecond.
 *
 * Boundary      : Hard walls.  When a particle exits the simulation
 *                 rectangle, push it back to the boundary and flip the
 *                 normal-component velocity, scaled by WALL_DAMPING <
 *                 1.0 to dissipate energy (otherwise the system gains
 *                 KE indefinitely from numerical errors).
 *
 * References
 * ──────────
 *   Müller, Charypar, Gross (2003), "Particle-Based Fluid Simulation
 *     for Interactive Applications" — the canonical real-time SPH
 *     paper this implementation traces back to.
 *   Monaghan (1992), "Smoothed Particle Hydrodynamics" Ann.Rev.A&A —
 *     the rigorous SPH foundation in astrophysics.
 *   Bridson, "Fluid Simulation for Computer Graphics" (CRC, 2008) —
 *     Ch. 8 covers SPH side-by-side with grid solvers.
 *   Stam (1999), "Stable Fluids" — the Eulerian counterpart;
 *     navier_stokes.c uses this approach.
 *   https://en.wikipedia.org/wiki/Smoothed-particle_hydrodynamics
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Imagine the fluid as a CROWD OF TINY DOTS, each with PERSONAL SPACE.
 * When dots get TOO CLOSE, they push each other apart (high local
 * density → repulsion).  When dots are FAR APART (low local density),
 * a weak attraction pulls them back together (surface tension).  The
 * dots also have a habit of MATCHING THEIR NEIGHBOURS' VELOCITIES
 * (viscosity).  Add gravity + walls.  That's it.  The fluid behaviour
 * (waves, splashing, settling) is EMERGENT from these three local
 * rules.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *   Imagine a swimming pool full of bouncy beach balls (the
 *   particles).  Each ball:
 *     - Has roughly fixed volume — it doesn't like being squashed.
 *     - Drifts with its neighbours rather than fighting them.
 *     - Falls under gravity, bounces off the pool's sides.
 *   Watch a million such balls and the SURFACE LOOKS LIKE WATER even
 *   though no individual ball "knows" anything about fluids.  SPH is
 *   the math version of that.
 *
 * COMPARING SPH vs NAVIER-STOKES (Lagrangian vs Eulerian)
 * ───────────────────────────────────────────────────────
 *
 *      ┌────────────────────┬────────────────────────────────────┐
 *      │ SPH (this file)    │ Navier-Stokes grid (navier_stokes.c)│
 *      ├────────────────────┼────────────────────────────────────┤
 *      │ Particles MOVE.    │ Grid cells stay PUT.                │
 *      │ Density is         │ Velocity stored per cell.           │
 *      │  estimated from    │ Advection moves cell values.        │
 *      │  neighbour count.  │                                     │
 *      │ Implicit boundary  │ Explicit boundary masks.            │
 *      │  (particles just   │                                     │
 *      │  bounce).          │                                     │
 *      │ Adaptive: uses     │ Fixed resolution.                   │
 *      │  resolution where  │                                     │
 *      │  fluid is.         │                                     │
 *      │ Open surfaces are  │ Surfaces need an extra phase field  │
 *      │  "free."           │  or VOF tracker.                    │
 *      │ Compressibility    │ Incompressibility easy.             │
 *      │  via Tait EOS      │                                     │
 *      │  (slight artefact).│                                     │
 *      └────────────────────┴────────────────────────────────────┘
 *
 * Why SPH for ASCII fluid?
 *   - Particles map naturally to characters on a screen.
 *   - Free surfaces = thin layer where neighbour count drops →
 *     density-to-glyph mapping makes the surface VISIBLE for free.
 *   - No grid → simulation domain doesn't need a fixed shape;
 *     resize the terminal and walls follow.
 *
 * KEY FORMULAS
 * ────────────
 *
 *   KERNEL (compact-support tent²):
 *     w(d) = d/H - 1            for d < H
 *          = 0                   for d ≥ H
 *     w² = (d/H - 1)²           always ≥ 0
 *     "How much does a particle at distance d contribute to my
 *      neighbourhood?"  Answer: full at d=0, zero at d=H, smooth in
 *      between.  H is the SMOOTHING RADIUS in cell units.
 *
 *   DENSITY:
 *     ρᵢ = Σⱼ  w(d_ij)²            (sum over particles within H)
 *     "Crowded-ness" — high near a clump, low at the surface.
 *
 *   PRESSURE FORCE on i from j (Tait EOS, simplified):
 *     F_pressure(i ← j) = w(d_ij) · (ρ_rest - ρᵢ - ρⱼ) · K_pressure
 *                          / ρᵢ
 *     direction: along (pᵢ - pⱼ)
 *     - When ρᵢ + ρⱼ > ρ_rest (CROWDED), the (ρ_rest - ρᵢ - ρⱼ) term
 *       is NEGATIVE.  Multiplied by w (negative inside support), the
 *       force is POSITIVE along (pᵢ - pⱼ) — i.e. AWAY from j.
 *       REPULSION.
 *     - When ρᵢ + ρⱼ < ρ_rest (SPARSE), the term is POSITIVE,
 *       times w (negative) → NEGATIVE force = TOWARD j.  ATTRACTION
 *       (surface-tension analogue).
 *     - When equal, force = 0 (rest equilibrium).
 *
 *   VISCOSITY FORCE on i from j:
 *     F_viscosity(i ← j) = |w(d_ij)| · (vⱼ - vᵢ) · K_viscosity
 *     direction: along (vⱼ - vᵢ) — i.e. PULL i's velocity toward j's.
 *     Smooths the velocity field; prevents adjacent particles from
 *     ploughing through each other.
 *
 *   SYMPLECTIC EULER (one tick):
 *     vₙ₊₁ = vₙ + aₙ · dt           (update velocity FIRST)
 *     xₙ₊₁ = xₙ + vₙ₊₁ · dt          (use NEW velocity)
 *
 *   WALL BOUNCE (per axis):
 *     if x < x_min:  x = x_min;  vₓ = -vₓ · WALL_DAMPING
 *     if x > x_max:  x = x_max;  vₓ = -vₓ · WALL_DAMPING
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - DIVISION BY DENSITY: if ρᵢ = 0 (an isolated particle with no
 *     neighbours), force / ρᵢ would NaN.  We add 0.001 in the
 *     denominator as a guard.
 *   - PARTICLES ESCAPING WALLS: if a single-step velocity exceeds the
 *     wall thickness, a particle could "tunnel" through.  Mitigated
 *     by capping SPH_DT below H / max_velocity (Courant condition).
 *   - GRID OVERFLOW: if the terminal grows huge, GMAX_W/GMAX_H limit
 *     the spatial-hash size; particles outside are clamped to the
 *     last cell (still works, just uneven distribution).
 *   - REST_SUM CALIBRATION: ρ_rest is hand-tuned for the kernel's
 *     "neutral packing density" at H = SMOOTH_RADIUS_CELLS.  Change
 *     H and ρ_rest must be re-tuned or the fluid will collapse or
 *     explode.
 *   - SPATIAL HASH CELL = 3 ≥ H = 2.2:  if you SHRINK GCELL below H,
 *     neighbours within H can fall outside the 3×3 cell block.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Scene 1 (blob): a sphere falls, splashes flat, settles to a
 *     pancake.  Surface stays visibly distinct from interior.
 *   - Scene 2 (column): a tall block collapses sideways like a
 *     liquid column losing its container.
 *   - Scene 3 (fountain): a dense stack at the floor erupts upward.
 *   - Scene 4 (collision): two blobs slam into each other and SLOSH.
 *   - Scene 5 (rain): top-down particles pile up.
 *   - Toggle 'g' (gravity OFF): particles drift to a Tait-EOS
 *     equilibrium — uniformly spaced grid-like packing.
 *   - Toggle 'v' (viscosity OFF): fluid becomes "rubbery" — pressure
 *     waves bounce visibly without damping.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1  What's a Lagrangian fluid simulator?
 *   T2  Smoothing kernel — compact support and tent²
 *   T3  Density estimation — counting weighted neighbours
 *   T4  Tait equation of state — pressure from density excess
 *   T5  Viscosity — velocity matching between neighbours
 *   T6  Symplectic Euler — why velocity update goes first
 *   T7  Spatial hash grid — O(N²) → O(N · k)
 *   T8  Stability — Courant, REST_SUM tuning, wall damping
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT'S A LAGRANGIAN FLUID SIMULATOR?
 * ────────────────────────────────────────
 * Two ways to simulate a fluid:
 *
 *   EULERIAN (grid-based)         LAGRANGIAN (particle-based)
 *   ─────────────────────         ─────────────────────────
 *                                                              .
 *   Fixed grid cells.             Particles MOVE freely.
 *   Each cell stores ρ, v, p.     Each particle stores its own
 *   Values FLOW through cells.     ρ, v.
 *   Need advection step.          Particles MOVE → no advection.
 *   Easy: incompressibility       Easy: free surfaces, splashes,
 *    via Poisson solve.            droplets.
 *   Hard: tracking surfaces       Hard: enforcing exact
 *    (need VOF/level-set).         incompressibility.
 *   Used in: Stable Fluids,       Used in: SPH (this file),
 *    weather models, smoke.        astrophysics, mud, snow.
 *
 * SPH = "Smoothed Particle Hydrodynamics."  Originally invented
 * for astrophysics (Lucy 1977, Gingold & Monaghan 1977) — modelling
 * star clusters and galaxies as point masses.  Adapted in the
 * 2000s for real-time graphics (Müller et al. 2003).  Your favourite
 * water fountain in a video game is almost certainly SPH or a
 * close cousin.
 *
 * The KEY INSIGHT of SPH:  if you can't carry a continuous density
 * field on a grid, ESTIMATE one at each particle from neighbours:
 *
 *     ρ(p) ≈ Σⱼ  m_j · w(|p - p_j|)
 *
 * where w(r) is a SMOOTHING KERNEL (smooth, peaked at r=0, zero
 * outside some radius H).  This converts particle positions into a
 * continuous density field on the fly.  Once you have ρ everywhere,
 * standard fluid physics applies.
 *
 * In this file we use UNIT MASS (m_j = 1) and the simpler estimator
 * ρᵢ = Σⱼ w(d_ij)² — the squared kernel makes the math simpler
 * downstream and gives the same qualitative behaviour.
 *
 * T2  SMOOTHING KERNEL — COMPACT SUPPORT AND TENT²
 * ────────────────────────────────────────────────
 * A smoothing kernel w(r) needs to:
 *
 *   1. Be SMOOTH (continuously differentiable).
 *   2. Be NON-NEGATIVE.
 *   3. Have COMPACT SUPPORT — be exactly zero for r ≥ H.
 *      (Otherwise every particle interacts with every other;
 *       neighbour search is impossible.)
 *   4. INTEGRATE TO 1 over its support (so density estimates make
 *      sense).
 *
 * The simplest kernel meeting these is the LINEAR TENT:
 *
 *     w_tent(r) = max(0, 1 - r/H)
 *
 * It's smooth(ish) and compact, but not C¹ at r = H (sharp
 * shoulder).  Squaring it gives the TENT² we use:
 *
 *     w_tent²(r) = (1 - r/H)²       for r < H
 *               = 0                  for r ≥ H
 *
 * Properties:
 *   - = 1 at r = 0 (peak)
 *   - = 0 at r = H (smooth shoulder, C¹ now)
 *   - quadratic between
 *
 * In code we store w SIGNED (= d/H - 1, NEGATIVE inside support)
 * and only square it where needed.  This is a tiny optimisation and
 * convenient: the SIGN of w gives the SIGN of pressure force directly
 * (T4 derivation).
 *
 *      ┌───────────────────────────────────────────────────────┐
 *      │                                                       │
 *      │    1 ┤●                                              │
 *      │      │  ●                                             │
 *      │      │    ●●                                          │
 *      │      │       ●●●                                      │
 *      │      │           ●●●●●                                │
 *      │      │                  ●●●●●●●●                      │
 *      │    0 ┤────────────────────────●───────────            │
 *      │      0                        H            r          │
 *      │                                                       │
 *      │      "compact support" = identically zero for r ≥ H    │
 *      │                                                       │
 *      └───────────────────────────────────────────────────────┘
 *
 * H = SMOOTH_RADIUS_CELLS = 2.2 cells in this file.  At unit
 * particle spacing, that means ~15 particles inside any one
 * particle's kernel.
 *
 * T3  DENSITY ESTIMATION — COUNTING WEIGHTED NEIGHBOURS
 * ─────────────────────────────────────────────────────
 * The DENSITY at particle i:
 *
 *     ρᵢ = Σⱼ  w(d_ij)²
 *
 * where the sum is over EVERY particle j (in practice only those
 * with d_ij < H, since w² = 0 outside).  Including SELF (d_ii = 0)
 * gives a baseline of w(0)² = 1, so ρᵢ ≥ 1 always.
 *
 * Worked example.  Take a particle in the middle of a uniformly-
 * packed clump at unit spacing.  Inside H = 2.2 there are roughly:
 *   - 1 self (d = 0, w² = 1)
 *   - 4 nearest neighbours at d=1 (w = 1/2.2 - 1 = -0.545,
 *                                  w² ≈ 0.297, total 1.19)
 *   - 4 diagonals at d ≈ 1.41 (w ≈ -0.36, w² ≈ 0.13, total 0.52)
 *   - 4 next-cell neighbours at d=2 (w ≈ -0.09, w² ≈ 0.008, total 0.03)
 *
 *   ρᵢ ≈ 1 + 1.19 + 0.52 + 0.03  ≈  2.74  ≈  3
 *
 * This "natural packing density ≈ 3" is the calibration anchor
 * for REST_SUM = 6 (= ρᵢ + ρⱼ at equilibrium pair).
 *
 * IMPLEMENTATION (§9): for each particle i, walk the 3×3 grid cells
 * around it (T7), compute distance to each particle j in those cells,
 * accumulate w² into ρᵢ.  ~15 evaluations per particle.
 *
 * T4  TAIT EQUATION OF STATE — PRESSURE FROM DENSITY EXCESS
 * ─────────────────────────────────────────────────────────
 * In a real fluid, PRESSURE is some FUNCTION of density:
 *
 *     p = f(ρ)              equation of state (EOS)
 *
 * For incompressible flow, p adjusts to keep ρ exactly constant —
 * but that requires solving a Poisson equation each step (expensive).
 *
 * The TAIT EOS is the cheap compromise:
 *
 *     p = K · (ρ - ρ_rest)
 *
 * Pressure is LINEARLY proportional to density excess.  Crowded
 * region → positive pressure → outward force.  Sparse region →
 * negative pressure → inward force ("cohesion").  K is the
 * STIFFNESS — bigger K → stiffer fluid (more incompressible).
 *
 * In SPH, the pressure FORCE on particle i from neighbour j is:
 *
 *     F_pressure(i←j) = ∇w(d_ij) · (pᵢ + pⱼ) / 2 · 1/ρⱼ
 *
 * Various simplifications get us to the form used in this file:
 *
 *     F_pressure(i←j) ∝ w(d_ij) · (ρ_rest - ρᵢ - ρⱼ) · K_pressure
 *                       direction: (xᵢ - xⱼ)
 *
 * Now check the SIGN:
 *
 *   Crowded (ρᵢ + ρⱼ > ρ_rest):
 *     (ρ_rest - ρᵢ - ρⱼ) < 0
 *     w < 0 (always inside support)
 *     product > 0
 *     direction is along (xᵢ - xⱼ) — i.e. AWAY from j  ✓ REPULSION
 *
 *   Sparse (ρᵢ + ρⱼ < ρ_rest):
 *     (ρ_rest - ρᵢ - ρⱼ) > 0
 *     w < 0
 *     product < 0
 *     direction is along (xⱼ - xᵢ) — i.e. TOWARD j     ✓ COHESION
 *
 *   Balanced: force = 0.  ✓ EQUILIBRIUM.
 *
 * So one expression handles both repulsion and cohesion, and the
 * sign of w (not its magnitude) does the switching.  Tidy.
 *
 * The cohesion gives surface tension for free: a particle near the
 * SURFACE of a blob has fewer neighbours → its ρᵢ is LOW → the
 * (ρ_rest - ρᵢ - ρⱼ) term is positive → it gets pulled INWARD.
 * The surface "wants" to minimise area — exactly surface tension.
 *
 * T5  VISCOSITY — VELOCITY MATCHING BETWEEN NEIGHBOURS
 * ────────────────────────────────────────────────────
 * Real-fluid viscosity diffuses momentum between neighbouring
 * parcels.  In SPH:
 *
 *     F_viscosity(i←j) = (vⱼ - vᵢ) · |w(d_ij)| · K_viscosity
 *
 * Particle i is PUSHED TOWARD neighbour j's velocity.  Big difference
 * → big force.  Same velocity → no force.
 *
 * This matters mainly for STABILITY: without viscosity, particles
 * with high velocity differences plough through each other,
 * crystalline patterns persist forever, and the simulation looks
 * "rubbery" rather than "fluid."  Toggle 'v' to see this in action.
 *
 * Note: real viscosity is FRAME-INVARIANT (only relative velocity
 * matters).  Our formulation respects that — (vⱼ - vᵢ) is the
 * relative velocity.  So adding a constant velocity to the whole
 * fluid produces no spurious viscous force.
 *
 * T6  SYMPLECTIC EULER — WHY VELOCITY UPDATE GOES FIRST
 * ─────────────────────────────────────────────────────
 * Standard FORWARD EULER:
 *
 *     xₙ₊₁ = xₙ + vₙ · dt
 *     vₙ₊₁ = vₙ + aₙ · dt
 *
 * Compute new x using OLD v, then update v.  Simple, intuitive,
 * UNSTABLE for oscillatory systems: energy slowly grows over time,
 * orbits spiral outward, springs explode.
 *
 * SYMPLECTIC EULER (also called "Euler-Cromer"):
 *
 *     vₙ₊₁ = vₙ + aₙ · dt    ◄── update velocity FIRST
 *     xₙ₊₁ = xₙ + vₙ₊₁ · dt   ◄── use NEW velocity
 *
 * Mathematically: this scheme PRESERVES the SYMPLECTIC FORM of
 * Hamiltonian dynamics (the "phase space volume" stays constant).
 * Practically: energy doesn't grow without bound, oscillators stay
 * bounded, particle systems with stiff springs (like SPH pressure
 * forces) don't explode.
 *
 * The only trick: write velocity update BEFORE position update.
 * One line of code; massive stability difference.
 *
 *      ┌───────────────────────────────────────────────────────┐
 *      │                                                       │
 *      │  Forward Euler on a spring:        Symplectic Euler:  │
 *      │                                                       │
 *      │      ●     spirals outward            ●               │
 *      │     ╱╲                              ╱   ╲             │
 *      │    ╱  ╲                            ●     ●            │
 *      │   ●    ●                            ╲   ╱             │
 *      │    ╲  ╱     each loop is               ●              │
 *      │     ╲╱       BIGGER (gains energy)    bounded loop    │
 *      │                                                       │
 *      └───────────────────────────────────────────────────────┘
 *
 * For SPH, where pressure forces are very stiff (high K_pressure),
 * standard Euler would explode within ~10 ticks.  Symplectic keeps
 * the same dt.
 *
 * T7  SPATIAL HASH GRID — O(N²) → O(N · k)
 * ────────────────────────────────────────
 * Naïve neighbour search: for every particle i, loop over every
 * particle j.  Cost: O(N²).  At N=1500 that's 2.25M comparisons
 * per pass, 4.5M per step (density + forces), 270M/sec at 60 Hz.
 * Slow.
 *
 * INSIGHT: the kernel has compact support — particles farther
 * than H contribute nothing.  Build a grid of CELLS of side ≥ H.
 * Each particle hashes into a cell.  All neighbours within H are
 * GUARANTEED to be in the 3×3 cell block around the particle's
 * cell.  Cost: O(N · k) where k = avg particles in 3×3 ≈ 9 ·
 * (N / cells).  For 1500 particles in a 30×8 grid that's
 * ~9 · 6 = 54 comparisons per particle.  ~30× speedup.
 *
 * Implementation (§8): linked-list-per-cell.  Two arrays:
 *
 *     grid_head[gy][gx]  = index of FIRST particle in cell (gy, gx)
 *     grid_next[i]       = index of NEXT particle in same cell as i
 *     terminator         = -1
 *
 * Build: O(N).  Iterate: walk grid_head[gy][gx], grid_next[i], ...
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   grid_head[gy][gx] ──► p_3 ─► p_8 ─► p_42 ─► -1│
 *      │                                                  │
 *      │   grid_next[3]  =  8                             │
 *      │   grid_next[8]  = 42                             │
 *      │   grid_next[42] = -1                             │
 *      │                                                  │
 *      │   walk pattern:                                  │
 *      │     for j = grid_head[gy][gx]; j != -1;          │
 *      │            j = grid_next[j]) { ... }             │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * T8  STABILITY — COURANT, REST_SUM TUNING, WALL DAMPING
 * ──────────────────────────────────────────────────────
 * Three knobs that determine whether SPH stays stable:
 *
 * (1) COURANT CONDITION on dt.  A particle should NOT move more
 *     than one kernel radius in one step.  Otherwise it could
 *     skip past neighbours entirely:
 *
 *        dt < H / max_velocity
 *
 *     With H = 2.2 and observed max v ≈ 18 cells/sec, dt < 0.12
 *     is safe.  We hard-code SPH_DT = 0.12; the CFL margin
 *     remains positive in all five scenes.
 *
 * (2) REST_SUM CALIBRATION.  REST_SUM = 6 is the (ρᵢ + ρⱼ) value
 *     at which pressure force = 0.  T3 worked example showed
 *     ρᵢ ≈ 3 for natural packing; pair sum = 3 + 3 = 6.  ✓
 *
 *     If REST_SUM is too SMALL, the fluid is COMPRESSED at
 *     equilibrium (denser than nature wants) — leads to under-
 *     puffed appearance.
 *     If too LARGE, the fluid wants to PUFF UP — particles drift
 *     apart unnaturally.
 *
 * (3) WALL DAMPING.  Without it, a particle bouncing off the
 *     floor returns with the same |v|.  Stack up many particles
 *     and energy ACCUMULATES from numerical errors → explosion.
 *     WALL_DAMPING = 0.6 keeps 60% of the kinetic energy on
 *     bounce; the other 40% leaves the simulation as "heat."
 *     Without this dissipation channel, the fluid gradually heats
 *     up.
 *
 * Together: SPH_DT respects Courant, REST_SUM matches the kernel,
 * WALL_DAMPING bleeds energy.  All three are interrelated; tweak
 * one and you may need to tweak the others.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ===================================================================== */
/* §1  config — every constant in one place                              */
/* ===================================================================== */
/*
 * No magic numbers below this section: every literal in the code is
 * a NAME defined here.  Knobs grouped by subsystem so you can find
 * the one that controls the behaviour you want to change.
 */

/* ── PARTICLE POOL ── */
#define PARTICLE_POOL_CAPACITY     5000

/* ── SPH PHYSICS CONSTANTS ── */

/* Kernel support radius (cells).  Particles within this distance
 * contribute to each other's density and forces. */
#define SMOOTH_RADIUS_CELLS        2.2

/* Pressure stiffness K (Tait EOS coefficient).  Higher = stiffer
 * fluid (more incompressible) but unstable at large dt.  Tuned
 * jointly with SPH_DT for stability. */
#define PRESSURE_K                 0.04

/* Viscosity coefficient.  Velocity smoothing rate. */
#define VISCOSITY_K                0.03

/* Gravity (cells / step²).  Low value avoids tunnelling through
 * floor at SPH_DT. */
#define GRAVITY_G                  0.08

/* Wall energy retention on bounce.  0.6 = 60% kinetic energy kept;
 * 40% drained as "heat" (T8 reasoning). */
#define WALL_DAMPING               0.6

/* Fixed physics timestep (seconds-equivalent; units of cells per
 * step).  Honours Courant: dt < H / v_max. */
#define SPH_DT                     0.12

/* Density target at rest equilibrium.  pressure = 0 when
 * ρᵢ + ρⱼ = REST_SUM.  Calibrated for SMOOTH_RADIUS_CELLS = 2.2. */
#define REST_SUM                   6.0

/* Numerical guard so we never divide by zero density. */
#define DENSITY_DIVIDE_GUARD       0.001

/* ── SPATIAL HASH GRID ── */

/* Cell side (cells).  Must be ≥ SMOOTH_RADIUS_CELLS so a 3×3
 * neighbourhood covers the kernel support. */
#define GRID_CELL_SIZE             3

/* Hard maximum grid dimensions (compile-time array bound). */
#define GRID_COLS_MAX              90
#define GRID_ROWS_MAX              22

/* ── DENSITY → GLYPH ZONES ── */

/* Density thresholds for character / colour selection in §15. */
#define DENSITY_THRESHOLD_CORE     3.5    /* dense interior  '#' bold */
#define DENSITY_THRESHOLD_BODY     1.2    /* fluid body       'o'      */
#define DENSITY_THRESHOLD_EDGE     0.1    /* sparse edge      '.'      */

/* ── HUD ── */
#define HUD_RESERVED_ROWS          2

/* ── COLOUR PAIRS ── */
enum {
    PAIR_DENSITY_CORE = 1,
    PAIR_DENSITY_BODY,
    PAIR_DENSITY_EDGE,
    PAIR_BORDER,
    PAIR_HUD,
    PAIR_HINT,
};

/* ── SCENE COUNT ── */
#define SCENE_COUNT                5

/* ── THEMES ── */
#define THEME_COUNT               10

/* ── FRAMEWORK TIMING ── */
#define SIM_HZ_MIN                10
#define SIM_HZ_DEFAULT            60
#define SIM_HZ_MAX               120
#define SIM_HZ_STEP               10
#define RENDER_FPS_CAP            60
#define FPS_RECOMPUTE_MS         500

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(hz)      (NS_PER_SEC / (hz))

/* ===================================================================== */
/* §2  clock — monotonic ns timer + sleep                                */
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
/* §3  rng — small random helper                                         */
/* ===================================================================== */

static int rand_in_range(int lo, int hi_exclusive)
{
    if (hi_exclusive <= lo) return lo;
    return lo + rand() % (hi_exclusive - lo);
}

/* ===================================================================== */
/* §4  themes — 10 palette triples + names                               */
/* ===================================================================== */
/*
 * Each theme is { core_xterm256, body_xterm256, edge_xterm256 }.
 * Core is the densest interior, edge is the sparsest surface;
 * the gradient should read as "less dense" left-to-right.
 *
 * All three colour values lie in the BRIGHT half of the 256-cube
 * (≥ 24), so they remain visible against the default-black
 * background even at A_NORMAL weight.
 */

typedef struct {
    short      core;
    short      body;
    short      edge;
    const char *name;
} ColorTheme;

static const ColorTheme color_theme_table[THEME_COUNT] = {
    {  51,  39,  27, "ocean"  },   /* cyan → blue                      */
    { 196, 208, 220, "lava"   },   /* red → orange → yellow            */
    { 226, 214, 196, "fire"   },   /* white-yellow → orange → red      */
    {  46,  34,  28, "matrix" },   /* bright green → mid → dark        */
    { 231, 141,  93, "nova"   },   /* white → violet → purple          */
    { 231, 159, 117, "ice"    },   /* white → sky → steel              */
    { 220, 208, 197, "sunset" },   /* yellow → orange → rose           */
    { 196, 160, 124, "blood"  },   /* bright red → crimson → dark      */
    { 201, 198, 165, "neon"   },   /* magenta → pink → soft purple     */
    { 154, 118,  46, "acid"   },   /* yellow-green → green             */
};

/* ===================================================================== */
/* §5  colors — pair init + theme apply                                  */
/* ===================================================================== */

static void colors_apply_theme(int theme_index)
{
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const ColorTheme *theme = &color_theme_table[theme_index];

    if (COLORS >= 256) {
        init_pair(PAIR_DENSITY_CORE, theme->core, -1);
        init_pair(PAIR_DENSITY_BODY, theme->body, -1);
        init_pair(PAIR_DENSITY_EDGE, theme->edge, -1);
        init_pair(PAIR_BORDER,       244,         -1);
        init_pair(PAIR_HUD,          226,         -1);   /* yellow */
        init_pair(PAIR_HINT,          51,         -1);   /* cyan   */
    } else {
        init_pair(PAIR_DENSITY_CORE, COLOR_CYAN,    -1);
        init_pair(PAIR_DENSITY_BODY, COLOR_CYAN,    -1);
        init_pair(PAIR_DENSITY_EDGE, COLOR_BLUE,    -1);
        init_pair(PAIR_BORDER,       COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,          COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,         COLOR_CYAN,    -1);
    }
}

static void colors_init(int theme_index)
{
    start_color();
    use_default_colors();
    colors_apply_theme(theme_index);
}

/* ===================================================================== */
/* §6  particle — Particle struct + global pool + spawn helpers          */
/* ===================================================================== */
/*
 * The simulation state is the particle pool — a fixed-size array
 * with a count.  Fields use long descriptive names; positions and
 * velocities are in CELL UNITS (the screen IS the physics grid).
 */

typedef struct {
    double pos_col;            /* x position (cells)                 */
    double pos_row;            /* y position (cells)                 */
    double vel_col;            /* x velocity (cells / step)          */
    double vel_row;            /* y velocity (cells / step)          */
    double accel_col;          /* x force accumulator                */
    double accel_row;          /* y force accumulator                */
    double density_estimate;   /* ρᵢ = Σ kernel² at this particle    */
} Particle;

/* Pool + count — global because every physics pass loops over all. */
static Particle particle_pool[PARTICLE_POOL_CAPACITY];
static int      particle_count = 0;

/* Physics-area dimensions — refreshed each tick from terminal size. */
static int phys_cols = 80;
static int phys_rows = 22;

/* Per-frame physics-toggle flags (driven by 'g', 'v' keys). */
static bool gravity_enabled   = true;
static bool viscosity_enabled = true;

/* particle_spawn_at — append one particle at (col, row) if room. */
static void particle_spawn_at(double pos_col, double pos_row)
{
    if (particle_count >= PARTICLE_POOL_CAPACITY) return;
    Particle *p          = &particle_pool[particle_count];
    p->pos_col           = pos_col;
    p->pos_row           = pos_row;
    p->vel_col           = 0.0;
    p->vel_row           = 0.0;
    p->accel_col         = 0.0;
    p->accel_row         = 0.0;
    p->density_estimate  = 0.0;
    particle_count++;
}

/* Spawn a circular blob of radius r centred at (cx, cy). */
static void particle_spawn_blob(int centre_col, int centre_row, int radius_cells)
{
    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {
            if (dx*dx + dy*dy <= radius_cells*radius_cells)
                particle_spawn_at((double)(centre_col + dx),
                                  (double)(centre_row + dy));
        }
    }
}

/* Spawn a solid rectangle. */
static void particle_spawn_rectangle(int corner_col, int corner_row,
                                     int width_cells, int height_cells)
{
    for (int dy = 0; dy < height_cells; dy++)
        for (int dx = 0; dx < width_cells; dx++)
            particle_spawn_at((double)(corner_col + dx),
                              (double)(corner_row + dy));
}

/* ===================================================================== */
/* §7  kernel — SPH smoothing kernel (compact-support tent²)             */
/* ===================================================================== */
/*
 * sph_kernel_signed — returns (d/H - 1) for d < H, else returns 0.
 *   - SIGNED: the value is NEGATIVE inside the kernel support.
 *     That negativity is what drives the SIGN of pressure forces
 *     in §10 (T4 derivation).
 *   - For density we want the SQUARED value (always non-negative);
 *     §9 squares it in place.
 *   - For "is this neighbour in range?" callers just check
 *     w_signed < 0  (equivalent to d < H).
 */
static double sph_kernel_signed(double distance_cells)
{
    double w = distance_cells / SMOOTH_RADIUS_CELLS - 1.0;
    return (w < 0.0) ? w : 0.0;
}

/* sph_pair_distance — compute |pᵢ - pⱼ| and the components.  Returns
 * the scalar distance; writes (Δcol, Δrow) to *delta_col / *delta_row
 * (i minus j; the convention used by the pressure force in §10). */
static double sph_pair_distance(const Particle *pi, const Particle *pj,
                                double *delta_col, double *delta_row)
{
    double dc = pi->pos_col - pj->pos_col;
    double dr = pi->pos_row - pj->pos_row;
    *delta_col = dc;
    *delta_row = dr;
    return sqrt(dc * dc + dr * dr);
}

/* ===================================================================== */
/* §8  grid — spatial-hash linked-list grid                              */
/* ===================================================================== */
/*
 * The grid converts the O(N²) all-pairs test into O(N · k) with
 * k = average particles in a 3×3 cell block ≈ 9 · (N / total_cells).
 *
 * Linked-list-per-cell representation (T7):
 *   grid_head[gy][gx]  = first particle index in cell, or -1
 *   grid_next[i]       = next particle index in same cell as i, or -1
 *
 * Iteration:
 *   for (int j = grid_head[gy][gx]; j != -1; j = grid_next[j]) ...
 */

static int grid_head[GRID_ROWS_MAX][GRID_COLS_MAX];
static int grid_next[PARTICLE_POOL_CAPACITY];
static int grid_active_cols;
static int grid_active_rows;

static inline int grid_cell_col_of(double pos_col)
{
    int gx = (int)(pos_col / GRID_CELL_SIZE);
    if (gx < 0)                  gx = 0;
    if (gx >= grid_active_cols)  gx = grid_active_cols - 1;
    return gx;
}

static inline int grid_cell_row_of(double pos_row)
{
    int gy = (int)(pos_row / GRID_CELL_SIZE);
    if (gy < 0)                  gy = 0;
    if (gy >= grid_active_rows)  gy = grid_active_rows - 1;
    return gy;
}

/* grid_rebuild — rebuild the spatial-hash from scratch.  Called once
 * at the START of every physics step (before density / forces). */
static void grid_rebuild(void)
{
    grid_active_cols = phys_cols / GRID_CELL_SIZE + 2;
    grid_active_rows = phys_rows / GRID_CELL_SIZE + 2;
    if (grid_active_cols > GRID_COLS_MAX) grid_active_cols = GRID_COLS_MAX;
    if (grid_active_rows > GRID_ROWS_MAX) grid_active_rows = GRID_ROWS_MAX;

    for (int gy = 0; gy < grid_active_rows; gy++)
        for (int gx = 0; gx < grid_active_cols; gx++)
            grid_head[gy][gx] = -1;

    /* Push each particle onto the head of its cell's list. */
    for (int i = 0; i < particle_count; i++) {
        int gx = grid_cell_col_of(particle_pool[i].pos_col);
        int gy = grid_cell_row_of(particle_pool[i].pos_row);
        grid_next[i]      = grid_head[gy][gx];
        grid_head[gy][gx] = i;
    }
}

/* ===================================================================== */
/* §9  density_pass — neighbour density estimation                       */
/* ===================================================================== */
/*
 * For each particle i, walk the 3×3 grid block around it.  Every
 * particle j in those cells contributes w(d_ij)² to ρᵢ.
 *
 * Self-contribution included: at d = 0, w² = 1, so ρᵢ ≥ 1 always.
 *
 * Total cost ≈ N · 9 · k, where k = avg particles per cell.  At
 * N=1500 in 30×8 cells, ~6 particles/cell, ~50 evals per particle.
 */
static void density_pass(void)
{
    for (int i = 0; i < particle_count; i++) {
        Particle *pi = &particle_pool[i];
        pi->density_estimate = 0.0;

        int cell_gx = grid_cell_col_of(pi->pos_col);
        int cell_gy = grid_cell_row_of(pi->pos_row);

        for (int gy = cell_gy - 1; gy <= cell_gy + 1; gy++) {
            if (gy < 0 || gy >= grid_active_rows) continue;
            for (int gx = cell_gx - 1; gx <= cell_gx + 1; gx++) {
                if (gx < 0 || gx >= grid_active_cols) continue;

                for (int j = grid_head[gy][gx]; j != -1; j = grid_next[j]) {
                    Particle *pj = &particle_pool[j];
                    double dc, dr;
                    double distance = sph_pair_distance(pi, pj, &dc, &dr);
                    double w_signed = sph_kernel_signed(distance);
                    /* Always w² ≥ 0; out-of-range w_signed = 0 → 0. */
                    pi->density_estimate += w_signed * w_signed;
                }
            }
        }
    }
}

/* ===================================================================== */
/* §10  forces_pass — pressure + viscosity acceleration                  */
/* ===================================================================== */
/*
 * For each pair (i, j) with j in the 3×3 block around i and j != i:
 *   distance = |pᵢ - pⱼ|
 *   w_signed = d/H - 1   (skip if ≥ 0, out of range)
 *
 *   pressure_term = (REST_SUM - ρᵢ - ρⱼ) · K_pressure
 *   pressure_force_along_dx = w_signed · pressure_term
 *                              / (ρᵢ + DENSITY_DIVIDE_GUARD)
 *   pi.accel += pressure_force_along_dx · (Δcol, Δrow)
 *
 *   if viscosity_enabled:
 *     visc_weight = -w_signed   (positive inside support)
 *     pi.accel += (vⱼ - vᵢ) · K_viscosity · visc_weight
 *
 * Gravity is set as the BASELINE acceleration so we don't add it
 * inside the pair loop.
 */
static void forces_pass(void)
{
    for (int i = 0; i < particle_count; i++) {
        Particle *pi = &particle_pool[i];

        /* Baseline: gravity (vertical). */
        pi->accel_col = 0.0;
        pi->accel_row = gravity_enabled ? GRAVITY_G : 0.0;

        int cell_gx = grid_cell_col_of(pi->pos_col);
        int cell_gy = grid_cell_row_of(pi->pos_row);

        for (int gy = cell_gy - 1; gy <= cell_gy + 1; gy++) {
            if (gy < 0 || gy >= grid_active_rows) continue;
            for (int gx = cell_gx - 1; gx <= cell_gx + 1; gx++) {
                if (gx < 0 || gx >= grid_active_cols) continue;

                for (int j = grid_head[gy][gx]; j != -1; j = grid_next[j]) {
                    if (j == i) continue;
                    Particle *pj = &particle_pool[j];

                    double dc, dr;
                    double distance = sph_pair_distance(pi, pj, &dc, &dr);
                    double w_signed = sph_kernel_signed(distance);
                    if (w_signed == 0.0) continue;     /* out of range */

                    /* PRESSURE FORCE (T4). */
                    double pressure_term =
                        (REST_SUM - pi->density_estimate
                                  - pj->density_estimate) * PRESSURE_K;
                    double pressure_force_per_dx =
                        w_signed * pressure_term
                        / (pi->density_estimate + DENSITY_DIVIDE_GUARD);
                    pi->accel_col += dc * pressure_force_per_dx;
                    pi->accel_row += dr * pressure_force_per_dx;

                    /* VISCOSITY FORCE (T5). */
                    if (viscosity_enabled) {
                        double visc_weight = -w_signed;   /* > 0 inside */
                        pi->accel_col +=
                            (pj->vel_col - pi->vel_col)
                            * VISCOSITY_K * visc_weight;
                        pi->accel_row +=
                            (pj->vel_row - pi->vel_row)
                            * VISCOSITY_K * visc_weight;
                    }
                }
            }
        }
    }
}

/* ===================================================================== */
/* §11  integrate_pass — symplectic Euler + wall bounce                  */
/* ===================================================================== */
/*
 * Symplectic Euler (T6):
 *   v_new = v + a · dt
 *   x_new = x + v_new · dt
 *
 * Then enforce hard walls.  Push back to the boundary, flip velocity
 * sign, multiply by WALL_DAMPING < 1 to bleed energy.
 */
static void integrate_pass(void)
{
    double col_max = (double)(phys_cols - 2);
    double row_max = (double)(phys_rows - 2);
    double col_min = 1.0;
    double row_min = 1.0;

    for (int i = 0; i < particle_count; i++) {
        Particle *p = &particle_pool[i];

        /* velocity update FIRST (symplectic) */
        p->vel_col += p->accel_col * SPH_DT;
        p->vel_row += p->accel_row * SPH_DT;

        /* position update using the NEW velocity */
        p->pos_col += p->vel_col * SPH_DT;
        p->pos_row += p->vel_row * SPH_DT;

        /* wall bounce — left / right */
        if (p->pos_col < col_min) {
            p->pos_col = col_min;
            p->vel_col = -p->vel_col * WALL_DAMPING;
        }
        if (p->pos_col > col_max) {
            p->pos_col = col_max;
            p->vel_col = -p->vel_col * WALL_DAMPING;
        }

        /* wall bounce — top / bottom */
        if (p->pos_row < row_min) {
            p->pos_row = row_min;
            p->vel_row = -p->vel_row * WALL_DAMPING;
        }
        if (p->pos_row > row_max) {
            p->pos_row = row_max;
            p->vel_row = -p->vel_row * WALL_DAMPING;
        }
    }
}

/* ===================================================================== */
/* §12  sph_step — full physics tick (4 passes)                          */
/* ===================================================================== */
/*
 * The complete physics in four lines.  ORDER MATTERS:
 *
 *   1. grid_rebuild     — positions snap to cells (input to next 2)
 *   2. density_pass     — needs grid; produces density_estimate
 *   3. forces_pass      — needs grid + density; produces accel
 *   4. integrate_pass   — needs accel; produces new pos / vel
 */
static void sph_step(void)
{
    grid_rebuild();
    density_pass();
    forces_pass();
    integrate_pass();
}

/* ===================================================================== */
/* §13  scenes — 5 scene loaders                                         */
/* ===================================================================== */
/*
 * Each scene reseeds the particle pool with a different initial
 * configuration.  Velocities are zero unless the scene needs them
 * (only #4 collision sets non-zero initial velocities).
 *
 *   1 BLOB DROP          big sphere falling under gravity
 *   2 COLUMN COLLAPSE    wide rectangle that splashes outward
 *   3 FOUNTAIN           dense stack at floor → erupts
 *   4 COLLISION          two blobs slamming together
 *   5 RAIN               curtain of particles falling from top
 */

static void scene_load_blob_drop(void)
{
    int cx = phys_cols / 2;
    particle_spawn_blob(cx, 6, 12);
}

static void scene_load_column_collapse(void)
{
    int cx = phys_cols / 2;
    particle_spawn_rectangle(cx - 18, 2, 36, 16);
}

static void scene_load_fountain(void)
{
    int cx = phys_cols / 2;
    int floor_row = phys_rows - 4;
    for (int i = 0; i < 700; i++) {
        int dx = rand_in_range(-4, 5);
        particle_spawn_at((double)(cx + dx), (double)floor_row);
    }
}

static void scene_load_collision(void)
{
    int cx = phys_cols / 2;
    int cy = phys_rows / 2;
    particle_spawn_blob(cx - 20, cy, 10);
    particle_spawn_blob(cx + 20, cy, 10);
    /* First half of the spawn list is the LEFT blob, second half the
     * RIGHT.  Slam them together. */
    int half = particle_count / 2;
    for (int i = 0; i < particle_count; i++)
        particle_pool[i].vel_col = (i < half) ?  2.5 : -2.5;
}

static void scene_load_rain(void)
{
    for (int i = 0; i < 800; i++) {
        int col = rand_in_range(2, phys_cols - 2);
        int row = rand_in_range(1, 7);
        particle_spawn_at((double)col, (double)row);
    }
}

/* Dispatch helper. */
static void scene_load_by_id(int id)
{
    particle_count = 0;
    switch (id) {
        case 1: scene_load_blob_drop();        break;
        case 2: scene_load_column_collapse();  break;
        case 3: scene_load_fountain();         break;
        case 4: scene_load_collision();        break;
        case 5: scene_load_rain();             break;
        default: break;
    }
}

static const char *scene_name_of(int id)
{
    switch (id) {
        case 1: return "blob-drop";
        case 2: return "column";
        case 3: return "fountain";
        case 4: return "collision";
        case 5: return "rain";
        default: return "?";
    }
}

/* ===================================================================== */
/* §14  scene — scene state + tick orchestrator                          */
/* ===================================================================== */

typedef struct {
    int  active_id;            /* 1..SCENE_COUNT */
    int  theme_index;          /* 0..THEME_COUNT-1 */
    bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->active_id   = 1;
    s->theme_index = 0;
    s->paused      = false;
    gravity_enabled   = true;
    viscosity_enabled = true;
    phys_cols = cols;
    phys_rows = rows - HUD_RESERVED_ROWS;
    colors_apply_theme(s->theme_index);
    scene_load_by_id(s->active_id);
}

static void scene_tick(Scene *s, int cols, int rows)
{
    if (s->paused) return;
    phys_cols = cols;
    phys_rows = rows - HUD_RESERVED_ROWS;
    sph_step();
}

/* ===================================================================== */
/* §15  render_particles — density → glyph + colour                      */
/* ===================================================================== */
/*
 * One character per particle.  Glyph + colour come from the
 * particle's local DENSITY (computed in §9):
 *
 *   density ≥ T_CORE   → '#'  bold,  PAIR_DENSITY_CORE
 *   density ≥ T_BODY   → 'o'        PAIR_DENSITY_BODY
 *   density ≥ T_EDGE   → '.'        PAIR_DENSITY_EDGE
 *   density  < T_EDGE  → not drawn (isolated particle)
 *
 * This makes the FREE SURFACE visible: surface particles have low
 * density (fewer neighbours than interior).
 */

typedef struct {
    char       glyph;
    int        pair_id;
    attr_t     extra_attr;
} DensityRender;

static DensityRender density_render_for(double density_estimate)
{
    DensityRender out = { '.', PAIR_DENSITY_EDGE, A_NORMAL };
    if (density_estimate >= DENSITY_THRESHOLD_CORE) {
        out.glyph      = '#';
        out.pair_id    = PAIR_DENSITY_CORE;
        out.extra_attr = A_BOLD;
    } else if (density_estimate >= DENSITY_THRESHOLD_BODY) {
        out.glyph      = 'o';
        out.pair_id    = PAIR_DENSITY_BODY;
        out.extra_attr = A_NORMAL;
    } else {
        out.glyph      = '.';
        out.pair_id    = PAIR_DENSITY_EDGE;
        out.extra_attr = A_NORMAL;
    }
    return out;
}

static void render_particles(WINDOW *w, int cols, int phys_area_rows)
{
    for (int i = 0; i < particle_count; i++) {
        const Particle *p = &particle_pool[i];

        int col = (int)(p->pos_col + 0.5);
        int row = (int)(p->pos_row + 0.5);
        if (col < 0 || col >= cols)             continue;
        if (row < 0 || row >= phys_area_rows)   continue;
        if (p->density_estimate < DENSITY_THRESHOLD_EDGE) continue;

        DensityRender dr = density_render_for(p->density_estimate);
        attr_t attrs = COLOR_PAIR(dr.pair_id) | dr.extra_attr;
        wattron(w, attrs);
        mvwaddch(w, row, col, (chtype)(unsigned char)dr.glyph);
        wattroff(w, attrs);
    }
}

/* ===================================================================== */
/* §16  render_border — frame around the simulation area                 */
/* ===================================================================== */
/*
 * Drawn LAST so it always appears on top of any overflow particles.
 * Uses A_NORMAL (not A_DIM) per CLAUDE.md theme-brightness rule.
 */
static void render_border(WINDOW *w, int cols, int phys_area_rows)
{
    wattron(w, COLOR_PAIR(PAIR_BORDER));
    for (int c = 0; c < cols; c++) {
        mvwaddch(w, 0,                  c, '-');
        mvwaddch(w, phys_area_rows - 1, c, '-');
    }
    for (int r = 1; r < phys_area_rows - 1; r++) {
        mvwaddch(w, r, 0,        '|');
        mvwaddch(w, r, cols - 1, '|');
    }
    mvwaddch(w, 0,                  0,        '+');
    mvwaddch(w, 0,                  cols - 1, '+');
    mvwaddch(w, phys_area_rows - 1, 0,        '+');
    mvwaddch(w, phys_area_rows - 1, cols - 1, '+');
    wattroff(w, COLOR_PAIR(PAIR_BORDER));
}

/* ===================================================================== */
/* §17  hud — top status + bottom hint strip                             */
/* ===================================================================== */
/*
 * Two HUD elements per CLAUDE.md spec:
 *   - STATUS at top-right: PAIR_HUD bright yellow + A_BOLD
 *   - HINT   at bottom:    PAIR_HINT bright cyan   + A_BOLD
 *
 * PAUSED indicator centred top.
 */

static void hud_paint_status(WINDOW *w, int cols, double fps_display,
                             int sim_hz, const Scene *s)
{
    char buf[200];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3dHz  scene:%-9s  n:%4d  "
             "g:%s  v:%s  theme:%-7s  %s ",
             fps_display, sim_hz, scene_name_of(s->active_id),
             particle_count,
             gravity_enabled   ? "ON"  : "off",
             viscosity_enabled ? "ON"  : "off",
             color_theme_table[s->theme_index].name,
             s->paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    int x   = cols - len;
    if (x < 0) x = 0;
    wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvwprintw(w, 0, x, "%s", buf);
    wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(WINDOW *w, int rows)
{
    const char *hint =
        " q:quit  spc:pause  1-5:scene  g:grav  v:visc  r:reset  "
        "b:blob  t:theme  ]/[:simHz ";
    wattron(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvwprintw(w, rows - 1, 0, "%s", hint);
    wattroff(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §18  screen — ncurses init / cleanup                                  */
/* ===================================================================== */

typedef struct { int cols; int rows; } Screen;

static void screen_init(Screen *s, int theme_index)
{
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    colors_init(theme_index);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void)
{
    endwin();
}

static void screen_present_frame(Screen *s, const Scene *sc,
                                 double fps_display, int sim_hz)
{
    erase();
    int phys_area_rows = s->rows - HUD_RESERVED_ROWS;
    render_particles(stdscr, s->cols, phys_area_rows);
    render_border   (stdscr, s->cols, phys_area_rows);
    hud_paint_status(stdscr, s->cols, fps_display, sim_hz, sc);
    hud_paint_hint  (stdscr, s->rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §19  app — main loop + signals + input                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_hz;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App app_state;

static void on_signal_quit  (int sig) { (void)sig; app_state.running     = 0; }
static void on_signal_resize(int sig) { (void)sig; app_state.need_resize = 1; }

/* Input handler.  Returns false if the user pressed quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
        case 'q': case 'Q': case 27:
            return false;

        case ' ':
            s->paused = !s->paused;
            break;

        case '1': case '2': case '3': case '4': case '5':
            s->active_id = ch - '0';
            scene_load_by_id(s->active_id);
            break;

        case 'g': case 'G':
            gravity_enabled = !gravity_enabled;
            break;

        case 'v': case 'V':
            viscosity_enabled = !viscosity_enabled;
            break;

        case 'r': case 'R':
            scene_load_by_id(s->active_id);
            break;

        case 'b': case 'B':
            particle_spawn_blob(rand_in_range(3, phys_cols - 3), 3, 3);
            break;

        case 't': case 'T':
            s->theme_index = (s->theme_index + 1) % THEME_COUNT;
            colors_apply_theme(s->theme_index);
            break;

        case ']':
            if (app->sim_hz + SIM_HZ_STEP <= SIM_HZ_MAX)
                app->sim_hz += SIM_HZ_STEP;
            break;
        case '[':
            if (app->sim_hz - SIM_HZ_STEP >= SIM_HZ_MIN)
                app->sim_hz -= SIM_HZ_STEP;
            break;

        default: break;
    }
    return true;
}

/* ===================================================================== */
/* §20  main — entry point                                               */
/* ===================================================================== */

int main(void)
{
    srand((unsigned int)(clock_now_ns() & 0xFFFFFFFF));
    atexit(screen_cleanup);
    signal(SIGINT,   on_signal_quit);
    signal(SIGTERM,  on_signal_quit);
    signal(SIGWINCH, on_signal_resize);

    App *app     = &app_state;
    app->running = 1;
    app->sim_hz  = SIM_HZ_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* Fixed-step accumulator (Glenn Fiedler "Fix Your Timestep!"). */
    int64_t prev_ns      = clock_now_ns();
    int64_t sim_accum_ns = 0;

    /* Sliding-window FPS counter. */
    int     frames_in_window = 0;
    int64_t window_accum_ns  = 0;
    double  fps_display      = 0.0;

    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {
        int64_t frame_start = clock_now_ns();

        /* ── resize ── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            phys_cols        = app->screen.cols;
            phys_rows        = app->screen.rows - HUD_RESERVED_ROWS;
            sim_accum_ns     = 0;
            app->need_resize = 0;
        }

        /* ── dt ── */
        int64_t dt_ns = frame_start - prev_ns;
        prev_ns       = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        /* ── fixed-step physics accumulator ── */
        int64_t tick_ns = TICK_NS(app->sim_hz);
        sim_accum_ns += dt_ns;
        while (sim_accum_ns >= tick_ns) {
            scene_tick(&app->scene, app->screen.cols, app->screen.rows);
            sim_accum_ns -= tick_ns;
        }

        /* ── render + present ── */
        screen_present_frame(&app->screen, &app->scene,
                             fps_display, app->sim_hz);

        /* ── fps window ── */
        frames_in_window++;
        window_accum_ns += dt_ns;
        if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
            fps_display = (double)frames_in_window
                        / ((double)window_accum_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            window_accum_ns  = 0;
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* ── frame cap ── */
        int64_t spent = clock_now_ns() - frame_start;
        if (spent < frame_cap_ns) clock_sleep_ns(frame_cap_ns - spent);
    }

    return 0;
}
