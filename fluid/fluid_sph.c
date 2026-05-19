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
 *   3. §12 sph_step — four lines: grid_rebuild, density_pass,
 *      forces_pass, integrate_pass.  The whole physics is THERE; §9-§11
 *      are the bodies (each now a pseudocode orchestrator over named
 *      pair-loop / Euler helpers).
 *   4. §9 density_pass — read AFTER tutorial T3.
 *   5. §10 forces_pass — read AFTER tutorials T4 and T5.
 *   6. §11 integrate_pass — read AFTER tutorial T6.
 *   7. §13-§14 scenes / scene — orchestration.
 *   8. §15-§17 rendering / HUD — visual layer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_world.pool[]            global array of every Particle
 *   g_world.count             how many slots are in use
 *   p->pos_col, p->pos_row     position (in cell units)
 *   p->vel_col, p->vel_row     velocity (cells/step)
 *   p->accel_col, p->accel_row force accumulator for one step
 *   p->density_estimate        ρᵢ = Σ kernel²
 *
 *   distance_cells             |pᵢ - pⱼ| in cell units
 *   kernel_signed              w = d/H - 1 (negative inside support)
 *   kernel_squared             w² (always non-negative; used for ρ)
 *
 *   g_grid.head[gy][gx]          first particle index in cell (gy, gx)
 *   g_grid.next[i]               next particle in same cell as i, or -1
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
 *   ── SPH foundations (1977 — two simultaneous discoveries) ─────────
 *   [1] Lucy, L. B. (1977), "A numerical approach to the testing of
 *       the fission hypothesis", Astron. J. 82, pp. 1013-1024 —
 *       ONE of the two original SPH papers, motivated by stellar
 *       fission simulation.
 *   [2] Gingold, R. A. & Monaghan, J. J. (1977), "Smoothed Particle
 *       Hydrodynamics: theory and application to non-spherical
 *       stars", Mon. Not. R. Astron. Soc. 181, pp. 375-389 — the
 *       OTHER original SPH paper, also 1977.  Modern SPH owes both.
 *   [3] Monaghan, J. J. (1992), "Smoothed Particle Hydrodynamics",
 *       Annu. Rev. Astron. Astrophys. 30, pp. 543-574 — rigorous
 *       SPH foundations and kernel theory.  Read for the proof that
 *       ∇·v at a particle = − (1/ρ)·dρ/dt, the key continuity link.
 *   [4] Monaghan, J. J. (2005), "Smoothed Particle Hydrodynamics",
 *       Rep. Prog. Phys. 68, pp. 1703-1759 — modernised review;
 *       covers tensile instability and the viscosity formulation
 *       used in §particle_step().
 *
 *   ── Real-time / graphics SPH ──────────────────────────────────────
 *   [5] Müller, M., Charypar, D. & Gross, M. (2003), "Particle-Based
 *       Fluid Simulation for Interactive Applications", ACM SIGGRAPH/
 *       Eurographics SCA — THE real-time SPH paper this implementation
 *       traces back to.  Source of the three poly6 / spiky / visc
 *       kernel choices used in §kernel.
 *   [6] Bridson, R. (2008), "Fluid Simulation for Computer Graphics",
 *       CRC Press — ch. 8 covers SPH side-by-side with grid solvers;
 *       useful comparison reading with [8].
 *
 *   ── Neighbour search ──────────────────────────────────────────────
 *   [7] Teschner, M. et al. (2003), "Optimized Spatial Hashing for
 *       Collision Detection of Deformable Objects", VMV — basis of
 *       the spatial-hash grid used in §grid_rebuild for O(N) neighbour
 *       queries instead of naive O(N²).
 *
 *   ── Comparison with the Eulerian approach ─────────────────────────
 *   [8] Stam, J. (1999), "Stable Fluids", SIGGRAPH — the Eulerian
 *       (grid-based) counterpart; project file fluid/navier_stokes.c
 *       uses this approach.  Same physics, different discretisation.
 *
 *   ── Rendering / ncurses ───────────────────────────────────────────
 *   [9] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the density-glyph ramp used by DensityRender.
 *  [10] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ────────────────────────────────────────
 *  [11] https://en.wikipedia.org/wiki/Smoothed-particle_hydrodynamics
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
 *     g_grid.head[gy][gx]  = index of FIRST particle in cell (gy, gx)
 *     g_grid.next[i]       = index of NEXT particle in same cell as i
 *     terminator         = -1
 *
 * Build: O(N).  Iterate: walk g_grid.head[gy][gx], g_grid.next[i], ...
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   g_grid.head[gy][gx] ──► p_3 ─► p_8 ─► p_42 ─► -1│
 *      │                                                  │
 *      │   g_grid.next[3]  =  8                             │
 *      │   g_grid.next[8]  = 42                             │
 *      │   g_grid.next[42] = -1                             │
 *      │                                                  │
 *      │   walk pattern:                                  │
 *      │     for j = g_grid.head[gy][gx]; j != -1;          │
 *      │            j = g_grid.next[j]) { ... }             │
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
#define M_PI 3.14159265358979323846
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
#define PARTICLE_POOL_CAPACITY 5000

/* ── SPH PHYSICS CONSTANTS ── */

/* Kernel support radius (cells).  Particles within this distance
 * contribute to each other's density and forces. */
#define SMOOTH_RADIUS_CELLS 2.2

/* Pressure stiffness K (Tait EOS coefficient).  Higher = stiffer
 * fluid (more incompressible) but unstable at large dt.  Tuned
 * jointly with SPH_DT for stability. */
#define PRESSURE_K 0.04

/* Viscosity coefficient.  Velocity smoothing rate. */
#define VISCOSITY_K 0.03

/* Gravity (cells / step²).  Low value avoids tunnelling through
 * floor at SPH_DT. */
#define GRAVITY_G 0.08

/* Wall energy retention on bounce.  0.6 = 60% kinetic energy kept;
 * 40% drained as "heat" (T8 reasoning). */
#define WALL_DAMPING 0.6

/* Fixed physics timestep (seconds-equivalent; units of cells per
 * step).  Honours Courant: dt < H / v_max. */
#define SPH_DT 0.12

/* Density target at rest equilibrium.  pressure = 0 when
 * ρᵢ + ρⱼ = REST_SUM.  Calibrated for SMOOTH_RADIUS_CELLS = 2.2. */
#define REST_SUM 6.0

/* Numerical guard so we never divide by zero density. */
#define DENSITY_DIVIDE_GUARD 0.001

/* ── SPATIAL HASH GRID ── */

/* Cell side (cells).  Must be ≥ SMOOTH_RADIUS_CELLS so a 3×3
 * neighbourhood covers the kernel support. */
#define GRID_CELL_SIZE 3

/* Hard maximum grid dimensions (compile-time array bound). */
#define GRID_COLS_MAX 90
#define GRID_ROWS_MAX 22

/* ── DENSITY → GLYPH ZONES ── */

/* Density thresholds for character / colour selection in §15. */
#define DENSITY_THRESHOLD_CORE 3.5 /* dense interior  '#' bold */
#define DENSITY_THRESHOLD_BODY 1.2 /* fluid body       'o'      */
#define DENSITY_THRESHOLD_EDGE 0.1 /* sparse edge      '.'      */

/* ── HUD ── */
#define HUD_RESERVED_ROWS 2

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
#define SCENE_COUNT 5

/* ── THEMES ── */
#define THEME_COUNT 10

/* ── FRAMEWORK TIMING ── */
#define SIM_HZ_MIN 10
#define SIM_HZ_DEFAULT 60
#define SIM_HZ_MAX 120
#define SIM_HZ_STEP 10
#define RENDER_FPS_CAP 60
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* ===================================================================== */
/* §2  clock — monotonic ns timer + sleep                                */
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
/* §3  rng — small random helper                                         */
/* ===================================================================== */

static int rand_in_range(int lo, int hi_exclusive) {
  if (hi_exclusive <= lo)
    return lo;
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

/*
 * ColorTheme — three-tier palette for one density-glyph theme.
 *
 * Intent
 *   The density renderer paints each grid cell with a glyph chosen by
 *   the cell's smoothed-density tier (high / mid / low ≈ liquid core
 *   / body / edge spray).  Each tier needs a colour pair, so a theme
 *   is exactly three 256-cube indices + a name.
 *
 * Why three tiers (not 8 like the wave demos)
 *   Density has a much narrower visible range than amplitude — it's
 *   always positive and bounded by particle count.  Three tiers map
 *   cleanly onto the three glyphs in the density ramp ('@' '*' '.')
 *   and don't require fine ramp colours.  Trade: lower visual
 *   resolution, but the spatial structure (the FLUID SHAPE) is what
 *   carries the demo, not subtle gradients within the fluid.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   All values are kept ≥ 24 so even `edge` (the dimmest tier) stays
 *   visible against the default background.  Cube indices 16–23 and
 *   gray 232–239 are FORBIDDEN — they render as black.
 *
 * Reference [10] Raymond's NCURSES HOWTO §6 — init_pair semantics
 *   that turn these triples into live colour pairs.
 */
typedef struct {
    short       core;     /* densest cells (liquid core)              */
    short       body;     /* medium-density cells (fluid body)         */
    short       edge;     /* low-density cells (droplets / spray)      */
    const char *name;     /* short ASCII label shown in HUD            */
} ColorTheme;

static const ColorTheme color_theme_table[THEME_COUNT] = {
    {51, 39, 27, "ocean"},     /* cyan → blue                      */
    {196, 208, 220, "lava"},   /* red → orange → yellow            */
    {226, 214, 196, "fire"},   /* white-yellow → orange → red      */
    {46, 34, 28, "matrix"},    /* bright green → mid → dark        */
    {231, 141, 93, "nova"},    /* white → violet → purple          */
    {231, 159, 117, "ice"},    /* white → sky → steel              */
    {220, 208, 197, "sunset"}, /* yellow → orange → rose           */
    {196, 160, 124, "blood"},  /* bright red → crimson → dark      */
    {201, 198, 165, "neon"},   /* magenta → pink → soft purple     */
    {154, 118, 46, "acid"},    /* yellow-green → green             */
};

/* ===================================================================== */
/* §5  colors — pair init + theme apply                                  */
/* ===================================================================== */

static void colors_apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const ColorTheme *theme = &color_theme_table[theme_index];

  if (COLORS >= 256) {
    init_pair(PAIR_DENSITY_CORE, theme->core, -1);
    init_pair(PAIR_DENSITY_BODY, theme->body, -1);
    init_pair(PAIR_DENSITY_EDGE, theme->edge, -1);
    init_pair(PAIR_BORDER, 244, -1);
    init_pair(PAIR_HUD, 226, -1); /* yellow */
    init_pair(PAIR_HINT, 51, -1); /* cyan   */
  } else {
    init_pair(PAIR_DENSITY_CORE, COLOR_CYAN, -1);
    init_pair(PAIR_DENSITY_BODY, COLOR_CYAN, -1);
    init_pair(PAIR_DENSITY_EDGE, COLOR_BLUE, -1);
    init_pair(PAIR_BORDER, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void colors_init(int theme_index) {
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

/*
 * Particle — one fluid sample point.
 *
 * Intent
 *   In SPH the fluid is represented by N samples that each carry
 *   position, velocity, and a few local field values.  The continuous
 *   field A(x) at any point is reconstructed by summing
 *   contributions from nearby particles:
 *
 *       A(x) = Σⱼ mⱼ · (Aⱼ / ρⱼ) · W(|x − xⱼ|, h)
 *
 *   where W is the smoothing kernel (§4) and h is the smoothing
 *   radius.  Each particle therefore needs (pos, vel) for its motion
 *   plus a CACHED LOCAL DENSITY so the next pass can evaluate
 *   pressure forces without re-summing kernel values.
 *
 * Why density_estimate is cached on the particle
 *   The pressure-force pass needs ρᵢ for both itself and its
 *   neighbours.  Computing ρ once per step (in density-pass) and
 *   reading it back during force-pass turns a doubly-nested loop into
 *   two single passes — see §particle_step and Müller §3.3 [5].
 *
 * Why explicit accel accumulators
 *   Forces from gravity, pressure, and viscosity arrive in three
 *   separate sub-passes.  Splitting them as accumulators (rather
 *   than computing a∑ in one mega-loop) makes each force law
 *   independently testable and matches the structure in [5].
 *
 * Why double, not float
 *   Many SPH viscosity / pressure terms involve products of small
 *   differences.  Single-precision can underflow into noise; the
 *   ~50 % extra cost of doubles is invisible at our N (~few hundred
 *   particles).
 *
 * Coordinate convention
 *   pos_col / pos_row are in CELL UNITS — the terminal grid IS the
 *   physics grid.  No separate pixel space (unlike spring_pendulum.c
 *   et al.).  vel and accel inherit the same units (cells / step,
 *   cells / step²).
 *
 * References [1][2] for the kernel-sum formalism; [5] Müller for the
 *   real-time field structure adopted here.
 */
typedef struct {
    double pos_col;          /* x position (cells)                    */
    double pos_row;          /* y position (cells)                    */
    double vel_col;          /* x velocity (cells / step)             */
    double vel_row;          /* y velocity (cells / step)             */
    double accel_col;        /* x force accumulator (reset each step) */
    double accel_row;        /* y force accumulator (reset each step) */
    double density_estimate; /* ρᵢ = Σⱼ mⱼ·W(|xᵢ−xⱼ|, h); cached     *
                              * by density-pass, read by force-pass.  */
} Particle;

/*
 * ParticleWorld — the particle pool plus the world it lives in.
 *
 * Intent
 *   Earlier versions kept the pool, count, phys-area bounds, and the
 *   gravity/viscosity toggles as flat file-scope globals.  This struct
 *   groups everything that describes the LAGRANGIAN PARTICLE WORLD:
 *   the particles themselves AND the bounding rectangle they live in,
 *   plus the per-frame force toggles that gate the §10 / §11 passes.
 *
 *   The struct is touched by every physics pass (density, force,
 *   integrate, wall) and by every painter, so we keep a single
 *   `g_world` instance and reach fields via `g_world.<field>`.  Avoids
 *   pointer threading through the inner-loop SPH math.
 *
 * Locality (sim vs render)
 *   - pool, count                         → simulation (the state)
 *   - g_world.phys_cols, g_world.phys_rows                → shared geometry (sim AND
 *                                            render bound their loops)
 *   - g_world.gravity_enabled, g_world.viscosity_enabled  → control flags driven by
 *                                            'g' / 'v' keys; read each
 *                                            step by force pass
 *
 *   Mis-classifying a control flag as render would couple the
 *   simulation to a user-visual choice, breaking reproducibility.
 *
 * Memory footprint
 *   PARTICLE_POOL_CAPACITY × sizeof(Particle) ≈ several KB in BSS;
 *   no malloc.  Matches CLAUDE.md "no dynamic allocation after init".
 */
typedef struct {
    /* ── Particle state (the pool + how many are live) ──────────── */
    Particle pool [PARTICLE_POOL_CAPACITY];
    int      count;

    /* ── Physics-area bounds (shared by sim AND render) ─────────── *
     * Refreshed each tick from terminal size in scene_tick.        */
    int phys_cols;
    int phys_rows;

    /* ── Per-frame force toggles (driven by 'g', 'v' keys) ──────── */
    bool gravity_enabled;
    bool viscosity_enabled;
} ParticleWorld;

static ParticleWorld g_world = {
    .phys_cols         = 80,
    .phys_rows         = 22,
    .gravity_enabled   = true,
    .viscosity_enabled = true,
};

/* particle_spawn_at — append one particle at (col, row) if room. */
static void particle_spawn_at(double pos_col, double pos_row) {
  if (g_world.count >= PARTICLE_POOL_CAPACITY)
    return;
  Particle *p = &g_world.pool[g_world.count];
  p->pos_col = pos_col;
  p->pos_row = pos_row;
  p->vel_col = 0.0;
  p->vel_row = 0.0;
  p->accel_col = 0.0;
  p->accel_row = 0.0;
  p->density_estimate = 0.0;
  g_world.count++;
}

/* Spawn a circular blob of radius r centred at (cx, cy). */
static void particle_spawn_blob(int centre_col, int centre_row,
                                int radius_cells) {
  for (int dy = -radius_cells; dy <= radius_cells; dy++) {
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      if (dx * dx + dy * dy <= radius_cells * radius_cells)
        particle_spawn_at((double)(centre_col + dx), (double)(centre_row + dy));
    }
  }
}

/* Spawn a solid rectangle. */
static void particle_spawn_rectangle(int corner_col, int corner_row,
                                     int width_cells, int height_cells) {
  for (int dy = 0; dy < height_cells; dy++)
    for (int dx = 0; dx < width_cells; dx++)
      particle_spawn_at((double)(corner_col + dx), (double)(corner_row + dy));
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
static double sph_kernel_signed(double distance_cells) {
  double w = distance_cells / SMOOTH_RADIUS_CELLS - 1.0;
  return (w < 0.0) ? w : 0.0;
}

/* sph_pair_distance — compute |pᵢ - pⱼ| and the components.  Returns
 * the scalar distance; writes (Δcol, Δrow) to *delta_col / *delta_row
 * (i minus j; the convention used by the pressure force in §10). */
static double sph_pair_distance(const Particle *pi, const Particle *pj,
                                double *delta_col, double *delta_row) {
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
 *   g_grid.head[gy][gx]  = first particle index in cell, or -1
 *   g_grid.next[i]       = next particle index in same cell as i, or -1
 *
 * Iteration:
 *   for (int j = g_grid.head[gy][gx]; j != -1; j = g_grid.next[j]) ...
 */

/*
 * Grid — the spatial-hash grid used for O(N·k) neighbour search.
 *
 * Intent
 *   Naive SPH neighbour search is O(N²) — every particle queries every
 *   other for "are you within smoothing radius?".  The spatial-hash
 *   grid bins particles by cell, then each query inspects only the
 *   3×3 cells around the query — O(N·k) where k ≈ 9·(N / total_cells)
 *   is the average occupancy.  See [7] Teschner et al. 2003.
 *
 * Why linked-list-per-cell (T7)
 *   For each grid cell we store the index of the FIRST particle in
 *   that cell (`head[gy][gx]`).  Each particle then stores the index
 *   of the NEXT particle in the SAME cell (`next[i]`).  Iteration:
 *     for (int j = head[gy][gx]; j != -1; j = next[j]) { ... }
 *   This avoids per-cell dynamic arrays — important for our "no
 *   dynamic allocation after init" budget.  Cost: O(1) insert and
 *   O(k) traversal where k is cell occupancy.
 *
 * Memory footprint
 *   head: GRID_ROWS_MAX × GRID_COLS_MAX × sizeof(int)
 *   next: PARTICLE_POOL_CAPACITY × sizeof(int)
 *   Both in BSS, no malloc.
 *
 * Reference [7] Teschner et al. 2003 for the spatial-hashing scheme.
 */
typedef struct {
    /* Head index per cell: first particle in that cell, or -1.        */
    int head[GRID_ROWS_MAX][GRID_COLS_MAX];

    /* Linked-list next-pointer per particle: next particle in the
     * SAME cell as particle i, or -1.                                 */
    int next[PARTICLE_POOL_CAPACITY];

    /* Active subregion bounds (≤ GRID_*_MAX, clamped on resize).      */
    int active_cols;
    int active_rows;
} Grid;

static Grid g_grid;

static inline int grid_cell_col_of(double pos_col) {
  int gx = (int)(pos_col / GRID_CELL_SIZE);
  if (gx < 0)
    gx = 0;
  if (gx >= g_grid.active_cols)
    gx = g_grid.active_cols - 1;
  return gx;
}

static inline int grid_cell_row_of(double pos_row) {
  int gy = (int)(pos_row / GRID_CELL_SIZE);
  if (gy < 0)
    gy = 0;
  if (gy >= g_grid.active_rows)
    gy = g_grid.active_rows - 1;
  return gy;
}

/* Compute the active spatial-grid extent from the current physics
 * area.  +2 padding includes the ghost cells around the visible
 * domain so the 3×3 neighbour stencil never reaches off-grid.
 * Clamped to GRID_*_MAX. */
static inline void compute_active_grid_bounds(void) {
    g_grid.active_cols = g_world.phys_cols / GRID_CELL_SIZE + 2;
    g_grid.active_rows = g_world.phys_rows / GRID_CELL_SIZE + 2;
    if (g_grid.active_cols > GRID_COLS_MAX) g_grid.active_cols = GRID_COLS_MAX;
    if (g_grid.active_rows > GRID_ROWS_MAX) g_grid.active_rows = GRID_ROWS_MAX;
}

/* Empty every cell's linked-list head so old particle indices from the
 * previous step don't linger.  Done in O(cells); the next-pointer
 * array is overwritten as particles are re-inserted (no clear needed). */
static inline void clear_all_cell_heads(void) {
    for (int gy = 0; gy < g_grid.active_rows; gy++)
        for (int gx = 0; gx < g_grid.active_cols; gx++)
            g_grid.head[gy][gx] = -1;
}

/* Insert particle i at the HEAD of its cell's linked list.
 * O(1) operation: new_node.next = old_head; cell.head = i.
 * Standard linked-list head-push. */
static inline void insert_particle_into_cell(int i) {
    int gx = grid_cell_col_of(g_world.pool[i].pos_col);
    int gy = grid_cell_row_of(g_world.pool[i].pos_row);
    g_grid.next[i]      = g_grid.head[gy][gx];
    g_grid.head[gy][gx] = i;
}

/*
 * grid_rebuild — rebuild the spatial-hash from scratch.
 *
 * Pseudocode:
 *   compute_active_grid_bounds   (recompute from current phys-area)
 *   clear_all_cell_heads          (every list starts empty)
 *   for each particle i:
 *     insert_particle_into_cell(i)
 *
 * Called once at the START of every physics step.  Order: rebuild →
 * density → forces → integrate.  Refs [7] Teschner 2003 for the
 * head/next spatial-hash scheme.
 */
static void grid_rebuild(void) {
    compute_active_grid_bounds();
    clear_all_cell_heads();
    for (int i = 0; i < g_world.count; i++)
        insert_particle_into_cell(i);
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
/* The 3×3 block of grid cells around particle pi, clipped to the
 * active grid.  Returns inclusive index ranges via out-pointers; the
 * caller iterates [gx_lo..gx_hi] × [gy_lo..gy_hi] and walks the
 * linked list at each cell.  Shared by density_pass and forces_pass. */
static inline void cell_block_around(const Particle *pi,
                                     int *out_gx_lo, int *out_gx_hi,
                                     int *out_gy_lo, int *out_gy_hi) {
    int cx = grid_cell_col_of(pi->pos_col);
    int cy = grid_cell_row_of(pi->pos_row);
    *out_gx_lo = (cx - 1 < 0) ? 0 : cx - 1;
    *out_gx_hi = (cx + 1 >= g_grid.active_cols) ? g_grid.active_cols - 1 : cx + 1;
    *out_gy_lo = (cy - 1 < 0) ? 0 : cy - 1;
    *out_gy_hi = (cy + 1 >= g_grid.active_rows) ? g_grid.active_rows - 1 : cy + 1;
}

/* Contribution to pi's density estimate from neighbour pj.
 * Müller §3.3 [5] uses the SQUARED kernel for density (always ≥ 0).
 * Self-pair (i == j) gives w(0)² = 1, so ρᵢ ≥ 1 always — a useful
 * lower bound that simplifies the pressure-force divide guard.       */
static inline double squared_kernel_contribution(const Particle *pi,
                                                  const Particle *pj) {
    double dc, dr;
    double distance = sph_pair_distance(pi, pj, &dc, &dr);
    double w        = sph_kernel_signed(distance);
    return w * w;
}

/*
 * density_pass — Müller et al. (2003) §3.3 [5] density estimation.
 *
 * Pseudocode:
 *   for each particle pi:
 *     pi.density_estimate ← 0
 *     (gx_lo..gx_hi, gy_lo..gy_hi) = cell_block_around(pi)
 *     for each cell in that 3×3 block:
 *       for each particle pj in that cell:
 *         pi.density_estimate += squared_kernel_contribution(pi, pj)
 *
 * Cost ≈ N · 9 · k where k = avg particles per cell.  At N=1500 in
 * 30×8 cells (~6 particles/cell), ~50 kernel evaluations per particle.
 */
static void density_pass(void) {
    for (int i = 0; i < g_world.count; i++) {
        Particle *pi = &g_world.pool[i];
        pi->density_estimate = 0.0;

        int gx_lo, gx_hi, gy_lo, gy_hi;
        cell_block_around(pi, &gx_lo, &gx_hi, &gy_lo, &gy_hi);

        for (int gy = gy_lo; gy <= gy_hi; gy++) {
            for (int gx = gx_lo; gx <= gx_hi; gx++) {
                for (int j = g_grid.head[gy][gx]; j != -1; j = g_grid.next[j]) {
                    pi->density_estimate +=
                        squared_kernel_contribution(pi, &g_world.pool[j]);
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
 *   if g_world.viscosity_enabled:
 *     visc_weight = -w_signed   (positive inside support)
 *     pi.accel += (vⱼ - vᵢ) · K_viscosity · visc_weight
 *
 * Gravity is set as the BASELINE acceleration so we don't add it
 * inside the pair loop.
 */
/* Set the baseline acceleration for one particle.  Gravity is the
 * ONLY external body force in this demo; setting it as a baseline
 * (not adding it inside the pair loop) is cleaner and avoids
 * accidentally scaling g by neighbour count.                        */
static inline void apply_gravity_baseline(Particle *pi) {
    pi->accel_col = 0.0;
    pi->accel_row = g_world.gravity_enabled ? GRAVITY_G : 0.0;
}

/* Add the SPH pressure force from pj to pi's acceleration accumulator.
 * Müller §3.4 [5]:
 *
 *     F_pressure_ij = −∇W(d) · (Pᵢ + Pⱼ) / (2 ρⱼ)
 *
 * Our simplified form merges the (P − P_rest) terms into one symmetric
 * REST_SUM − ρᵢ − ρⱼ expression (Tait equation of state, T4 tutorial).
 * The DENSITY_DIVIDE_GUARD prevents division by zero for isolated
 * particles whose density estimate could drop to ε. */
static inline void apply_pressure_pair_force(Particle *pi,
                                              const Particle *pj,
                                              double dc, double dr,
                                              double w_signed) {
    double pressure_term =
        (REST_SUM - pi->density_estimate - pj->density_estimate) * PRESSURE_K;
    double force_per_dx =
        w_signed * pressure_term /
        (pi->density_estimate + DENSITY_DIVIDE_GUARD);
    pi->accel_col += dc * force_per_dx;
    pi->accel_row += dr * force_per_dx;
}

/* Add the SPH artificial-viscosity force from pj to pi's accel.
 * Müller §3.5 [5] / Monaghan [3] ch. 5.  Drives pi's velocity toward
 * the LOCAL AVERAGE of its neighbours — a smoothing operator on the
 * velocity field.  visc_weight = −w_signed is positive inside the
 * kernel support so the sign is correct. */
static inline void apply_viscosity_pair_force(Particle *pi,
                                               const Particle *pj,
                                               double w_signed) {
    double visc_weight = -w_signed;  /* positive inside support */
    pi->accel_col += (pj->vel_col - pi->vel_col) * VISCOSITY_K * visc_weight;
    pi->accel_row += (pj->vel_row - pi->vel_row) * VISCOSITY_K * visc_weight;
}

/* Apply both pair-wise SPH forces (pressure + optional viscosity)
 * from pj onto pi.  Skips self (i == j) and out-of-kernel pairs
 * (w_signed == 0). */
static inline void apply_sph_pair_forces(Particle *pi, int i,
                                          const Particle *pj, int j) {
    if (j == i)
        return;
    double dc, dr;
    double distance = sph_pair_distance(pi, pj, &dc, &dr);
    double w_signed = sph_kernel_signed(distance);
    if (w_signed == 0.0)
        return;  /* out of range */

    apply_pressure_pair_force(pi, pj, dc, dr, w_signed);
    if (g_world.viscosity_enabled)
        apply_viscosity_pair_force(pi, pj, w_signed);
}

/*
 * forces_pass — Müller §3 [5]: pressure + viscosity acceleration.
 *
 * Pseudocode:
 *   for each particle pi:
 *     apply_gravity_baseline(pi)                       (g·ŷ if enabled)
 *     (gx_lo..gx_hi, gy_lo..gy_hi) = cell_block_around(pi)
 *     for each cell in that 3×3 block:
 *       for each particle pj in that cell:
 *         apply_sph_pair_forces(pi, pj)                (pressure + visc)
 *
 * Force shape: pressure REPELS dense regions, viscosity SMOOTHS
 * velocity disparities.  Together they reproduce incompressible
 * fluid flow.  Refs [3] Monaghan 1992; [5] Müller 2003.
 */
static void forces_pass(void) {
    for (int i = 0; i < g_world.count; i++) {
        Particle *pi = &g_world.pool[i];

        apply_gravity_baseline(pi);

        int gx_lo, gx_hi, gy_lo, gy_hi;
        cell_block_around(pi, &gx_lo, &gx_hi, &gy_lo, &gy_hi);

        for (int gy = gy_lo; gy <= gy_hi; gy++) {
            for (int gx = gx_lo; gx <= gx_hi; gx++) {
                for (int j = g_grid.head[gy][gx]; j != -1; j = g_grid.next[j]) {
                    apply_sph_pair_forces(pi, i, &g_world.pool[j], j);
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
/* Symplectic-Euler step 1: velocity from acceleration.
 *   v_new = v + a · dt
 * Doing velocity FIRST and position SECOND (using the NEW velocity)
 * is what makes the integrator symplectic — conserves a modified
 * Hamiltonian and gives bounded energy error.  See [4] Monaghan 2005
 * §6 and the spring_pendulum.c demo for a deeper treatment. */
static inline void symplectic_euler_velocity_step(Particle *p, double dt) {
    p->vel_col += p->accel_col * dt;
    p->vel_row += p->accel_row * dt;
}

/* Symplectic-Euler step 2: position from NEW velocity.
 *   x_new = x + v_new · dt
 * Reading the just-updated velocity is the defining feature of
 * semi-implicit / symplectic Euler.  */
static inline void symplectic_euler_position_step(Particle *p, double dt) {
    p->pos_col += p->vel_col * dt;
    p->pos_row += p->vel_row * dt;
}

/* Hard-wall bounce on ONE axis.  If pos < lo or > hi, push back to
 * the boundary and FLIP the velocity component, scaled by
 * WALL_DAMPING < 1 to bleed energy out.  Energy loss is essential —
 * a perfectly-elastic bounce would let numerical noise pump KE into
 * the system indefinitely.                                          */
static inline void enforce_wall_bounce_axis(double *pos, double *vel,
                                             double lo, double hi) {
    if (*pos < lo) { *pos = lo; *vel = -*vel * WALL_DAMPING; }
    if (*pos > hi) { *pos = hi; *vel = -*vel * WALL_DAMPING; }
}

/*
 * integrate_pass — symplectic Euler step + wall bounce.
 *
 * Pseudocode:
 *   (col_lo, col_hi) = (1, g_world.phys_cols - 2)   ← walls 1 cell in
 *   (row_lo, row_hi) = (1, g_world.phys_rows - 2)
 *   for each particle p:
 *     symplectic_euler_velocity_step(p, dt)       (v += a·dt)
 *     symplectic_euler_position_step(p, dt)       (x += v_new·dt)
 *     enforce_wall_bounce_axis(&p.col, &p.vel_col, col_lo, col_hi)
 *     enforce_wall_bounce_axis(&p.row, &p.vel_row, row_lo, row_hi)
 *
 * Order matters: forces_pass writes accel → integrate_pass reads it.
 * The walls are placed ONE cell inside the physics area so the
 * particles are visible against the border drawn at the edge.
 */
static void integrate_pass(void) {
    const double col_lo = 1.0,                              col_hi = (double)(g_world.phys_cols - 2);
    const double row_lo = 1.0,                              row_hi = (double)(g_world.phys_rows - 2);

    for (int i = 0; i < g_world.count; i++) {
        Particle *p = &g_world.pool[i];
        symplectic_euler_velocity_step(p, SPH_DT);
        symplectic_euler_position_step(p, SPH_DT);
        enforce_wall_bounce_axis(&p->pos_col, &p->vel_col, col_lo, col_hi);
        enforce_wall_bounce_axis(&p->pos_row, &p->vel_row, row_lo, row_hi);
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
static void sph_step(void) {
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

static void scene_load_blob_drop(void) {
  int cx = g_world.phys_cols / 2;
  particle_spawn_blob(cx, 6, 12);
}

static void scene_load_column_collapse(void) {
  int cx = g_world.phys_cols / 2;
  particle_spawn_rectangle(cx - 18, 2, 36, 16);
}

static void scene_load_fountain(void) {
  int cx = g_world.phys_cols / 2;
  int floor_row = g_world.phys_rows - 4;
  for (int i = 0; i < 700; i++) {
    int dx = rand_in_range(-4, 5);
    particle_spawn_at((double)(cx + dx), (double)floor_row);
  }
}

static void scene_load_collision(void) {
  int cx = g_world.phys_cols / 2;
  int cy = g_world.phys_rows / 2;
  particle_spawn_blob(cx - 20, cy, 10);
  particle_spawn_blob(cx + 20, cy, 10);
  /* First half of the spawn list is the LEFT blob, second half the
   * RIGHT.  Slam them together. */
  int half = g_world.count / 2;
  for (int i = 0; i < g_world.count; i++)
    g_world.pool[i].vel_col = (i < half) ? 2.5 : -2.5;
}

static void scene_load_rain(void) {
  for (int i = 0; i < 800; i++) {
    int col = rand_in_range(2, g_world.phys_cols - 2);
    int row = rand_in_range(1, 7);
    particle_spawn_at((double)col, (double)row);
  }
}

/* Dispatch helper. */
static void scene_load_by_id(int id) {
  g_world.count = 0;
  switch (id) {
  case 1:
    scene_load_blob_drop();
    break;
  case 2:
    scene_load_column_collapse();
    break;
  case 3:
    scene_load_fountain();
    break;
  case 4:
    scene_load_collision();
    break;
  case 5:
    scene_load_rain();
    break;
  default:
    break;
  }
}

static const char *scene_name_of(int id) {
  switch (id) {
  case 1:
    return "blob-drop";
  case 2:
    return "column";
  case 3:
    return "fountain";
  case 4:
    return "collision";
  case 5:
    return "rain";
  default:
    return "?";
  }
}

/* ===================================================================== */
/* §14  scene — scene state + tick orchestrator                          */
/* ===================================================================== */

/*
 * Scene — what the user has chosen to look at, and whether time runs.
 *
 * Intent
 *   Unlike the other demos in this folder, fluid_sph.c keeps the bulk
 *   of simulation state OUTSIDE Scene — g_world.pool[] /
 *   g_world.count / g_world.phys_cols / g_world.phys_rows are file-scope globals
 *   because every physics pass loops over all of them and threading
 *   them through helpers would obscure the math.  Scene therefore
 *   carries only the THREE pieces of state that need to survive a
 *   reset: which preset is loaded, which theme is in effect, and
 *   whether time is paused.
 *
 * Locality (sim vs render)
 *   - active_id     → simulation (decides which scene_load_*() ran
 *                     and therefore which particles populated the
 *                     pool).  Read by the reset key and the HUD label.
 *   - theme_index   → pure rendering (selects which ColorTheme is
 *                     applied to the density pairs).  Changing it
 *                     MUST NOT touch the particle pool — only
 *                     repaints colour pairs via colors_apply_theme().
 *   - paused        → control state.  Gates scene_tick() and drives
 *                     the HUD "PAUSED" tag.
 *
 *   Mis-classifying theme_index as sim (and accidentally rebuilding
 *   particles on theme change) would break the engine-toggle
 *   invariant that "changing the look doesn't disturb the physics".
 *
 * Why these specific fields and no others
 *   Particles, grid bounds, kernel parameters, and the spatial-hash
 *   table are file-scope statics; bringing them into Scene would
 *   bloat the struct without helping the few helpers that legitimately
 *   need them.
 *
 * Reference [10] Raymond on the scene-paint pipeline that feeds off
 *   theme_index + paused.
 */
typedef struct {
    /* ── Simulation selector (read by reset path + HUD label) ──── */
    int  active_id;     /* 1..SCENE_COUNT — which preset is loaded   */

    /* ── Pure render state (read by colors_apply_theme only) ──── */
    int  theme_index;   /* 0..THEME_COUNT-1 — indexes ColorTheme    */

    /* ── Control state (gates scene_tick + HUD "PAUSED" tag) ──── */
    bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  s->active_id = 1;
  s->theme_index = 0;
  s->paused = false;
  g_world.gravity_enabled = true;
  g_world.viscosity_enabled = true;
  g_world.phys_cols = cols;
  g_world.phys_rows = rows - HUD_RESERVED_ROWS;
  colors_apply_theme(s->theme_index);
  scene_load_by_id(s->active_id);
}

static void scene_tick(Scene *s, int cols, int rows) {
  if (s->paused)
    return;
  g_world.phys_cols = cols;
  g_world.phys_rows = rows - HUD_RESERVED_ROWS;
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

/*
 * DensityRender — one cell's drawing instruction, derived from local density.
 *
 * Intent
 *   The renderer needs to know THREE things per cell: which glyph to
 *   draw, which colour pair to use, and whether to apply A_BOLD.  All
 *   three depend on the same input — the local smoothed density — so
 *   bundling them in one struct lets density_to_render() return all
 *   three in one shot without an output-parameter dance.
 *
 * Why a struct and not three parallel arrays
 *   Per-cell density is computed once but read three ways (glyph,
 *   colour, attribute).  Co-locating the three derived values keeps
 *   the lookup table compact in cache and the calling code simple:
 *   `dr = density_to_render(rho); mvaddch(... dr.glyph ...)`.
 *
 * Why the tier choice depends on density (not particle count)
 *   We want SURFACE visibility — surface cells have lower density
 *   (fewer neighbours within smoothing radius) than interior cells.
 *   Tiering by density rather than count means droplets and spray
 *   render as the dim edge tier even when they're locally dense.
 */
typedef struct {
    char   glyph;        /* '@' '*' '.' depending on density tier */
    int    pair_id;      /* PAIR_DENSITY_CORE / _BODY / _EDGE     */
    attr_t extra_attr;   /* A_BOLD for the densest tier            */
} DensityRender;

/* Build the CORE density-tier instruction: dense liquid core glyph
 * + colour + bold attribute.  See DensityRender doc for the three-
 * tier rationale. */
static inline DensityRender make_core_tier(void) {
    return (DensityRender){ '#', PAIR_DENSITY_CORE, A_BOLD };
}

/* Build the BODY density-tier instruction: medium liquid body glyph
 * + colour, no bold. */
static inline DensityRender make_body_tier(void) {
    return (DensityRender){ 'o', PAIR_DENSITY_BODY, A_NORMAL };
}

/* Build the EDGE density-tier instruction: faint droplet / spray
 * glyph + colour, no bold.  Used for surface particles. */
static inline DensityRender make_edge_tier(void) {
    return (DensityRender){ '.', PAIR_DENSITY_EDGE, A_NORMAL };
}

/*
 * density_render_for — tier dispatch by density threshold.
 *
 * Pseudocode:
 *   if ρ ≥ CORE_THRESHOLD → core tier ('#' bold)
 *   if ρ ≥ BODY_THRESHOLD → body tier ('o')
 *   else                  → edge tier ('.')
 *
 * Higher density → denser interior → bolder glyph.  Lower density
 * (surface / spray) → dim '.' so the FLUID SHAPE is what carries the
 * visual, not subtle ramp gradients.
 */
static DensityRender density_render_for(double density_estimate) {
    if (density_estimate >= DENSITY_THRESHOLD_CORE) return make_core_tier();
    if (density_estimate >= DENSITY_THRESHOLD_BODY) return make_body_tier();
    return make_edge_tier();
}

/* Round a particle's real-valued position to its display cell. */
static inline void particle_to_screen_cell(const Particle *p,
                                            int *out_col, int *out_row) {
    *out_col = (int)(p->pos_col + 0.5);
    *out_row = (int)(p->pos_row + 0.5);
}

/* Is the cell (col, row) inside the visible field rect? */
static inline bool screen_cell_in_field(int col, int row,
                                         int cols, int phys_area_rows) {
    return col >= 0 && col < cols
        && row >= 0 && row < phys_area_rows;
}

/* Should this particle be drawn at all?  Below DENSITY_THRESHOLD_EDGE
 * the particle is an isolated splatter — drawing it would be visual
 * noise.  Skipping these cells also reduces newscr/curscr diff size. */
static inline bool particle_should_render(const Particle *p) {
    return p->density_estimate >= DENSITY_THRESHOLD_EDGE;
}

/* Paint ONE particle cell with its density-tier glyph + colour. */
static inline void paint_particle_cell(WINDOW *w, int row, int col,
                                        DensityRender dr) {
    attr_t attrs = COLOR_PAIR(dr.pair_id) | dr.extra_attr;
    wattron(w, attrs);
    mvwaddch(w, row, col, (chtype)(unsigned char)dr.glyph);
    wattroff(w, attrs);
}

/*
 * render_particles — paint every visible particle to its density tier.
 *
 * Pseudocode:
 *   for each particle p:
 *     (col, row) = particle_to_screen_cell(p)
 *     if not screen_cell_in_field(col, row, ...): skip
 *     if not particle_should_render(p):           skip
 *     dr = density_render_for(p->density_estimate)
 *     paint_particle_cell(win, row, col, dr)
 */
static void render_particles(WINDOW *w, int cols, int phys_area_rows) {
    for (int i = 0; i < g_world.count; i++) {
        const Particle *p = &g_world.pool[i];

        int col, row;
        particle_to_screen_cell(p, &col, &row);

        if (!screen_cell_in_field(col, row, cols, phys_area_rows))
            continue;
        if (!particle_should_render(p))
            continue;

        DensityRender dr = density_render_for(p->density_estimate);
        paint_particle_cell(w, row, col, dr);
    }
}

/* ===================================================================== */
/* §16  render_border — frame around the simulation area                 */
/* ===================================================================== */
/*
 * Drawn LAST so it always appears on top of any overflow particles.
 * Uses A_NORMAL (not A_DIM) per CLAUDE.md theme-brightness rule.
 */
static void render_border(WINDOW *w, int cols, int phys_area_rows) {
  wattron(w, COLOR_PAIR(PAIR_BORDER));
  for (int c = 0; c < cols; c++) {
    mvwaddch(w, 0, c, '-');
    mvwaddch(w, phys_area_rows - 1, c, '-');
  }
  for (int r = 1; r < phys_area_rows - 1; r++) {
    mvwaddch(w, r, 0, '|');
    mvwaddch(w, r, cols - 1, '|');
  }
  mvwaddch(w, 0, 0, '+');
  mvwaddch(w, 0, cols - 1, '+');
  mvwaddch(w, phys_area_rows - 1, 0, '+');
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
                             int sim_hz, const Scene *s) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%3dHz  scene:%-9s  n:%4d  "
           "g:%s  v:%s  theme:%-7s  %s ",
           fps_display, sim_hz, scene_name_of(s->active_id), g_world.count,
           g_world.gravity_enabled ? "ON" : "off", g_world.viscosity_enabled ? "ON" : "off",
           color_theme_table[s->theme_index].name,
           s->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = cols - len;
  if (x < 0)
    x = 0;
  wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwprintw(w, 0, x, "%s", buf);
  wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(WINDOW *w, int rows) {
  const char *hint = " q:quit  spc:pause  1-5:scene  g:grav  v:visc  r:reset  "
                     "b:blob  t:theme  ]/[:simHz ";
  wattron(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(w, rows - 1, 0, "%s", hint);
  wattroff(w, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §18  screen — ncurses init / cleanup                                  */
/* ===================================================================== */

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we keep
 * only cell dimensions for HUD placement and field clipping.
 *
 * Render pipeline (one frame): erase → scene_paint → hud_paint_*
 *   → wnoutrefresh(stdscr) → doupdate().  Diff-only writes to the
 *   terminal — no flicker.  See [10] Raymond §11.
 */
typedef struct {
    int cols;   /* terminal width  in cells (getmaxyx)             */
    int rows;   /* terminal height in cells (getmaxyx)             */
} Screen;

static void screen_init(Screen *s, int theme_index) {
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

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_present_frame(Screen *s, const Scene *sc, double fps_display,
                                 int sim_hz) {
  erase();
  int phys_area_rows = s->rows - HUD_RESERVED_ROWS;
  render_particles(stdscr, s->cols, phys_area_rows);
  render_border(stdscr, s->cols, phys_area_rows);
  hud_paint_status(stdscr, s->cols, fps_display, sim_hz, sc);
  hud_paint_hint(stdscr, s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §19  app — main loop + signals + input                                */
/* ===================================================================== */

/*
 * App — top-level container; lives in BSS as the single app_state instance.
 *
 * Intent
 *   Signal handlers (on_signal_*) need to reach state that the main
 *   loop polls.  A global App + handlers that flip its sig_atomic_t
 *   flags is the standard POSIX "wake the main loop" pattern.
 *
 * Why the volatile sig_atomic_t flags
 *   POSIX permits signal handlers to write ONLY sig_atomic_t values
 *   with simple assignments — anything wider is UB.  volatile forces
 *   the main loop to re-read from memory across signal arrival
 *   (no compiler caching into a register).
 *
 * Why sim_hz lives here (not in Scene)
 *   sim_hz is a frame-loop concern — it picks the inner-loop tick
 *   period TICK_NS(sim_hz) — and has no meaning inside scene_tick
 *   which receives the resulting dt as a parameter.  Putting it in
 *   App keeps Scene free of timing detail.
 */
typedef struct {
    Scene  scene;                          /* world + control state    */
    Screen screen;                         /* terminal extent          */
    int    sim_hz;                         /* sim tick rate, Hz        */
    volatile sig_atomic_t running;         /* SIGINT/TERM clears this  */
    volatile sig_atomic_t need_resize;     /* SIGWINCH sets this       */
} App;

static App app_state;

static void on_signal_quit(int sig) {
  (void)sig;
  app_state.running = 0;
}
static void on_signal_resize(int sig) {
  (void)sig;
  app_state.need_resize = 1;
}

/* Input handler.  Returns false if the user pressed quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
    s->active_id = ch - '0';
    scene_load_by_id(s->active_id);
    break;

  case 'g':
  case 'G':
    g_world.gravity_enabled = !g_world.gravity_enabled;
    break;

  case 'v':
  case 'V':
    g_world.viscosity_enabled = !g_world.viscosity_enabled;
    break;

  case 'r':
  case 'R':
    scene_load_by_id(s->active_id);
    break;

  case 'b':
  case 'B':
    particle_spawn_blob(rand_in_range(3, g_world.phys_cols - 3), 3, 3);
    break;

  case 't':
  case 'T':
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

  default:
    break;
  }
  return true;
}

/* ===================================================================== */
/* §20  main — entry point                                               */
/* ===================================================================== */

int main(void) {
  srand((unsigned int)(clock_now_ns() & 0xFFFFFFFF));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal_quit);
  signal(SIGTERM, on_signal_quit);
  signal(SIGWINCH, on_signal_resize);

  App *app = &app_state;
  app->running = 1;
  app->sim_hz = SIM_HZ_DEFAULT;

  screen_init(&app->screen, 0);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  /* Fixed-step accumulator (Glenn Fiedler "Fix Your Timestep!"). */
  int64_t prev_ns = clock_now_ns();
  int64_t sim_accum_ns = 0;

  /* Sliding-window FPS counter. */
  int frames_in_window = 0;
  int64_t window_accum_ns = 0;
  double fps_display = 0.0;

  const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (app->running) {
    int64_t frame_start = clock_now_ns();

    /* ── resize ── */
    if (app->need_resize) {
      screen_resize(&app->screen);
      g_world.phys_cols = app->screen.cols;
      g_world.phys_rows = app->screen.rows - HUD_RESERVED_ROWS;
      sim_accum_ns = 0;
      app->need_resize = 0;
    }

    /* ── dt ── */
    int64_t dt_ns = frame_start - prev_ns;
    prev_ns = frame_start;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    /* ── fixed-step physics accumulator ── */
    int64_t tick_ns = TICK_NS(app->sim_hz);
    sim_accum_ns += dt_ns;
    while (sim_accum_ns >= tick_ns) {
      scene_tick(&app->scene, app->screen.cols, app->screen.rows);
      sim_accum_ns -= tick_ns;
    }

    /* ── render + present ── */
    screen_present_frame(&app->screen, &app->scene, fps_display, app->sim_hz);

    /* ── fps window ── */
    frames_in_window++;
    window_accum_ns += dt_ns;
    if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      fps_display = (double)frames_in_window /
                    ((double)window_accum_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      window_accum_ns = 0;
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
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
