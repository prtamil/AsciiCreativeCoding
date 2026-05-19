/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vorticity_streamfunction_solver.c
 *   2-D incompressible Navier-Stokes via the VORTICITY-STREAMFUNCTION
 *   formulation, with FOUR CLASSIC SCENARIOS and LAGRANGIAN TRACER
 *   PARTICLES so you can SEE the flow.
 *
 * DEMO: Solves the 2-D NS equations on a rectangular grid using ω
 *       (vorticity = local fluid spin) and ψ (streamfunction = a
 *       scalar whose contours ARE the streamlines).  Pressure is
 *       eliminated entirely.  Each tick advances ω in time, then
 *       solves a Poisson equation for ψ, then derives velocities.
 *
 *       Four scenarios cycle with 'p' / 'P':
 *
 *         KARMAN STREET   — uniform inflow past a circular cylinder.
 *                            At Reynolds ~100-1000, the wake develops
 *                            the famous KARMAN VORTEX STREET: vortices
 *                            shed alternately from the top and bottom
 *                            of the cylinder, drifting downstream in a
 *                            staggered pattern.  Iconic.  THE flow.
 *
 *         LID CAVITY      — square box, top wall slides right at unit
 *                            speed.  Steady recirculating vortex inside.
 *                            CFD validation classic.
 *
 *         FREE JET        — narrow inlet at the centre of the left
 *                            wall, fluid blasts in.  Outflow on the
 *                            right.  Jet entrains surrounding fluid;
 *                            mushroom-tip vortices form at the head.
 *
 *         BACKWARD STEP   — channel flow over a downward step at
 *                            inlet.  Behind the step, the flow
 *                            SEPARATES and reattaches further
 *                            downstream — a recirculation bubble
 *                            forms.  Classic separation flow.
 *
 *       Three core visualisations + tracer overlay:
 *
 *         VORTICITY       — diverging blue/red heatmap of ω.  Red =
 *                            counter-clockwise spin, blue = clockwise.
 *         STREAMLINES     — banded ψ contours (lines of constant ψ
 *                            are streamlines).
 *         VELOCITY        — speed magnitude heatmap.
 *         TRACERS         — 400 LAGRANGIAN PARTICLES advected through
 *                            the velocity field.  Toggle with 'x'.
 *                            This is the killer feature: when you SEE
 *                            dots tracing out the vortex street, the
 *                            abstract numbers become obvious.
 *
 * Study alongside:
 *   fluid/navier_stokes.c      — Eulerian primitive-variables NS
 *                                 (Stam stable fluids).  Different
 *                                 formulation, same underlying physics.
 *   fluid/lattice_gas.c        — particle-level NS (FHP).
 *   fluid/shallow_water_solver.c — depth-averaged 2-D flow.
 *
 * Section map:
 *   §1   config            — every tunable named, scenario presets
 *   §2   clock             — monotonic ns timer + sleep
 *   §3   rng               — small wrapper for tracer respawn
 *   §4   themes            — diverging vorticity + sequential velocity
 *   §5   colors            — pair init + diverging-band picker
 *   §6   ramp              — ASCII glyph + threshold lookup
 *   §7   grid_state        — psi, omega, omega_next, u, v, walls
 *   §8   obstacle_layouts  — cylinder, step builders
 *   §9   apply_boundary    — per-side BC + Thom's formula
 *   §10  vorticity_step    — explicit Euler with upwind + diffusion
 *   §11  poisson_solve     — SOR iteration of ∇²ψ = -ω
 *   §12  velocity_recompute— u = ∂ψ/∂y, v = -∂ψ/∂x; adaptive dt
 *   §13  ns_step           — full physics tick (orchestrator)
 *   §14  tracers           — Lagrangian particle pool + advection
 *   §15  presets           — 4 scenario specs + loaders
 *   §16  render_vorticity  — ω field heatmap
 *   §17  render_streamlines— ψ contour bands
 *   §18  render_velocity   — speed magnitude heatmap
 *   §19  render_tracers    — overlay particles + trails
 *   §20  render_obstacles  — overlay solid cells
 *   §21  hud               — top status + bottom hint (CLAUDE.md spec)
 *   §22  scene             — per-frame state + tick wrapper
 *   §23  screen            — ncurses init / cleanup / present
 *   §24  app               — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   v                vorticity view
 *   s                streamline view
 *   w                velocity view
 *   x                toggle tracer overlay
 *   p / P            next / prev scenario
 *   + / -            Reynolds number up / down
 *   r                reset current scenario
 *   t                cycle theme (vorticity palette)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/vorticity_streamfunction_solver.c \
 *       -o vorticity_solver -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      T1-T2 establish the worldview (vorticity + streamfunction
 *      replace velocity + pressure).  T3-T5 cover the numerical
 *      algorithm (transport equation, Poisson solve, velocity
 *      recovery).  T6-T7 cover boundary conditions for the four
 *      scenarios.  T8 explains the Karman street physics.  T9 covers
 *      tracer particles.  T10 ties everything together.
 *   2. §1 config — every tunable in one place.
 *   3. §7 grid_state — the data structure.
 *   4. §13 ns_step — the algorithm in five calls.
 *   5. §10-§12 — read AFTER tutorials T3, T4, T5.
 *   6. §15 presets — four scenario specs.
 *   7. §16-§20 — visualisation layer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_scene.streamfunction_psi[]         ψ — flat row-major array
 *   g_scene.vorticity_omega[]            ω — flat row-major array
 *   g_scene.vorticity_omega_next[]       scratch for forward-Euler step
 *   g_scene.velocity_x[], g_scene.velocity_y[]   u, v — derived from ψ each tick
 *   g_scene.wall_mask[]                  true = solid obstacle / wall cell
 *
 *   g_scene.grid_cols, g_scene.grid_rows         active grid dimensions
 *   g_scene.cell_size_x, g_scene.cell_size_y     dx, dy
 *   g_scene.kinematic_viscosity          ν = 1/Re
 *   g_scene.reynolds_number              Re = U·L/ν
 *
 *   active_scenario_index        which row of scenario_table is live
 *   active_view_mode             VIEW_VORTICITY / VIEW_STREAMLINES /
 *                                  VIEW_VELOCITY
 *   tracer_overlay_enabled       toggle for the particle layer
 *   simulation_paused            run/pause toggle
 *
 *   inflow_velocity              U at the inflow boundary (= 1.0 for
 *                                  scenarios with a moving boundary)
 *
 * Background you need
 * ───────────────────
 *   - Vector calculus on a grid: ∇·u (divergence), ∇×u (curl, scalar
 *     in 2-D), ∇²ψ (Laplacian).  T1 reviews them informally.
 *   - Forward Euler integration; understand stability bounds.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Pressure-Poisson solvers, projection methods.  Vorticity-
 *     streamfunction sidesteps pressure entirely (T2).
 *   - Compressible flow, turbulence models, free surfaces.  Pure
 *     2-D incompressible viscous flow.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm    : VORTICITY-STREAMFUNCTION formulation of the 2-D
 *                incompressible Navier-Stokes equations.  Two scalar
 *                fields replace the velocity vector + pressure:
 *
 *                  ω = vorticity         = ∂v/∂x − ∂u/∂y
 *                  ψ = streamfunction    where  u = ∂ψ/∂y, v = −∂ψ/∂x
 *
 *                Per tick:
 *                  1. ADVANCE ω by one timestep using the VORTICITY
 *                     TRANSPORT equation:
 *                       ∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y = ν·∇²ω
 *                  2. RECOVER ψ from the new ω by solving the POISSON
 *                     EQUATION ∇²ψ = −ω.  Iterative SOR solver.
 *                  3. RECOMPUTE u, v from the new ψ via finite
 *                     differences:  u = ∂ψ/∂y,  v = −∂ψ/∂x.
 *                  4. APPLY BOUNDARY CONDITIONS — walls, inflow,
 *                     outflow, moving lid, obstacles.
 *
 *                Plus ADVECT a pool of Lagrangian tracer particles
 *                through the velocity field for visualisation.
 *
 * Math basis   : Take the curl of the Navier-Stokes momentum equation:
 *                  ∂u/∂t + (u·∇)u = -∇p/ρ + ν·∇²u
 *                The pressure term ∇p disappears (curl of a gradient
 *                is zero).  In 2-D the curl is a SCALAR ω, and the
 *                resulting equation is the vorticity transport
 *                equation above.  Combined with INCOMPRESSIBILITY
 *                (∇·u = 0), which is automatically satisfied by the
 *                streamfunction definition, the system is closed.
 *
 *                Why this is powerful:
 *                  - One scalar field (ω) carries all the dynamics.
 *                  - One scalar field (ψ) carries all the kinematics.
 *                  - PRESSURE is gone — no Poisson solve for p, no
 *                    projection step.
 *                  - Continuity (∇·u = 0) is automatic, exact, never
 *                    drifts numerically.
 *                Cost:
 *                  - Restricted to 2-D (curl is a scalar only in 2-D).
 *                  - Boundary conditions for ω at walls are tricky
 *                    (Thom's formula, T6).
 *
 * Performance  : SOR Poisson solver runs ~14 sweeps per tick.  Each
 *                sweep is O(N²) cell updates.  At 80 × 22 = 1760
 *                cells × 14 sweeps × 8 sub-steps × 30 fps ≈ 6M cell
 *                updates per second.  Trivial.
 *
 * References
 * ──────────
 *   ── The ψ-ω formulation ──────────────────────────────────────────
 *   [1] Roache, P. J. (1972), "Computational Fluid Dynamics", Hermosa
 *       — foundational textbook; the ψ-ω chapter is the canonical
 *       reference for the formulation used here.
 *   [2] Anderson, J. D. (1995), "Computational Fluid Dynamics: The
 *       Basics with Applications", McGraw-Hill — modern undergraduate
 *       treatment of ψ-ω with worked examples.
 *
 *   ── Historical benchmarks ─────────────────────────────────────────
 *   [3] Thom, A. (1933), "The Flow Past Circular Cylinders at Low
 *       Speeds", Proc. R. Soc. A 141, pp. 651-669 — the wall-
 *       vorticity formula in §9 is "Thom's formula" from this paper.
 *   [4] Ghia, U., Ghia, K. N. & Shin, C. T. (1982), "High-Re
 *       Solutions for Incompressible Flow Using the Navier-Stokes
 *       Equations and a Multigrid Method", J. Comput. Phys. 48(3),
 *       pp. 387-411 — the lid-driven cavity benchmark we use as a
 *       preset; one of the most-cited CFD validation problems.
 *   [5] von Kármán, T. (1911), "Über den Mechanismus des Widerstandes,
 *       den ein bewegter Körper in einer Flüssigkeit erfährt",
 *       Nachr. Ges. Wiss. Göttingen — original vortex-street analysis;
 *       the cylinder-wake preset reproduces this.
 *
 *   ── Poisson solvers ──────────────────────────────────────────────
 *   [6] Saad, Y. (2003), "Iterative Methods for Sparse Linear
 *       Systems", 2nd ed., SIAM — SOR convergence theory for the
 *       streamfunction Poisson solve.  Our SOR_OMEGA default of 1.7
 *       follows the textbook rule-of-thumb.
 *
 *   ── Numerical methods generally ───────────────────────────────────
 *   [7] LeVeque, R. J. (2007), "Finite Difference Methods for ODEs
 *       and PDEs", SIAM — explicit time-step stability for the ω
 *       advection-diffusion equation.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *   [8] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the vorticity-magnitude → glyph ramp.
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; init_pair,
 *       use_default_colors, newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Vorticity
 *  [11] https://en.wikipedia.org/wiki/Stream_function
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't track the velocity directly.  Track the FLUID'S TENDENCY TO
 * SPIN (vorticity ω) and a SCALAR FIELD WHOSE CONTOURS ARE THE
 * STREAMLINES (streamfunction ψ).  The two are linked by a Poisson
 * equation.  You get the flow without ever computing pressure —
 * which means no projection step, no incompressibility constraint
 * to enforce explicitly, just two coupled fields evolving in
 * lockstep.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a swirling river of water on a flat plain.  At each point,
 * two questions:
 *
 *   1. HOW MUCH IS THE WATER ROTATING HERE?  (vorticity ω)
 *      Positive ω = counter-clockwise spin.  Negative = clockwise.
 *      Zero = the water is gliding straight without rotation.
 *
 *   2. WHICH STREAMLINE IS THE WATER ON?  (streamfunction ψ)
 *      Imagine numbering every streamline 0, 1, 2, ...  ψ tells
 *      you which streamline the water at this point is sitting on.
 *      Cells with ψ = 5.0 all share the same streamline.  Cells
 *      with ψ = 5.1 are on a slightly displaced streamline.
 *
 * Now: given ω at every point, can you reconstruct the velocity
 * everywhere?  YES — solve ∇²ψ = -ω.  Once you have ψ, the
 * velocity is just the gradient of ψ rotated 90°: u = ∂ψ/∂y,
 * v = -∂ψ/∂x.
 *
 * And how does ω evolve?  By being CARRIED BY the flow (advection),
 * SPREAD by viscosity (diffusion), and CREATED at walls where
 * no-slip imposes a velocity gradient.  That's the vorticity
 * transport equation.
 *
 * So: ω drives the flow.  Walls inject ω.  ψ encodes ω back into
 * a velocity field.  Loop forever.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   ONE PHYSICS TICK:                                  │
 *      │                                                      │
 *      │     ω_new ← TRANSPORT(ω_old, u, v, ν)                │
 *      │              (advect by u, v; diffuse by ν)          │
 *      │                                                      │
 *      │     ψ ← POISSON_SOLVE(ω_new, BC's)                   │
 *      │              (∇²ψ = -ω, SOR iteration)               │
 *      │                                                      │
 *      │     u, v ← VELOCITY_FROM_PSI(ψ)                      │
 *      │              (u = ∂ψ/∂y, v = -∂ψ/∂x)                 │
 *      │                                                      │
 *      │     APPLY_BC(ψ, ω, u, v)                             │
 *      │              (walls, inflow, outflow, lid)           │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   STREAMFUNCTION DEFINITION (incompressible 2-D):
 *     u =  ∂ψ/∂y
 *     v = -∂ψ/∂x
 *     ⇒  ∇·u = ∂u/∂x + ∂v/∂y
 *           = ∂²ψ/∂x∂y - ∂²ψ/∂y∂x = 0    ✓ (continuity automatic)
 *
 *   VORTICITY DEFINITION (z-component of curl in 2-D):
 *     ω = ∂v/∂x - ∂u/∂y
 *
 *   POISSON EQUATION linking ψ to ω:
 *     ω = ∂v/∂x - ∂u/∂y
 *       = -∂²ψ/∂x² - ∂²ψ/∂y²
 *       = -∇²ψ
 *     ⇒  ∇²ψ = -ω
 *
 *   VORTICITY TRANSPORT (curl of Navier-Stokes):
 *     ∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y = ν · ∇²ω
 *
 *   FORWARD EULER UPDATE (with upwind convection):
 *     ω_new[i, j] = ω + dt · (-u·∂ω/∂x - v·∂ω/∂y + ν·∇²ω)
 *
 *   SOR POISSON UPDATE (per cell, one sweep):
 *     ψ_GS = (dy²·(ψ_E + ψ_W) + dx²·(ψ_N + ψ_S) + dx²·dy²·ω)
 *             / (2·(dx² + dy²))
 *     ψ_new = ψ + α · (ψ_GS - ψ)        α ∈ (1, 2), α ≈ 1.7
 *
 *   THOM'S WALL-VORTICITY FORMULA (no-slip wall, ψ_wall = 0):
 *     ω_wall = -2 · ψ_interior / h²
 *     For a moving wall (lid at speed U_lid):
 *     ω_wall = -2 · ψ_interior / h² - 2 · U_lid / h
 *
 *   ADAPTIVE DT (CFL):
 *     dt_conv = CFL · dx / max|u|       (advection limit)
 *     dt_diff = CFL · dx² / (4 · ν)      (diffusion limit)
 *     dt = min(dt_conv, dt_diff)
 *
 *   TRACER ADVECTION (per particle, forward Euler):
 *     pos += velocity_at(pos) · dt
 *     bilinear-interpolate the velocity field at fractional pos
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - WALL VORTICITY is not freely chosen — it's COMPUTED from the
 *     interior ψ via Thom's formula (T6).  Picking arbitrary ω at
 *     walls breaks the coupling.
 *   - STREAMFUNCTION at walls must be a CONSTANT for each connected
 *     wall segment (no-flow-through condition).  In a single-domain
 *     box, ψ = 0 on the entire boundary.  In Karman / step / jet,
 *     each wall segment has its own constant.
 *   - INFLOW BOUNDARIES need ψ to vary linearly with the
 *     perpendicular coordinate, ω = 0 (uniform incoming flow has
 *     no vorticity).
 *   - OUTFLOW (Neumann zero-gradient) loses energy slowly;
 *     reflections from boundaries are minimal but not zero.
 *   - SOR_OMEGA = 1.7 is a robust default but the optimum α
 *     depends on grid size.  At very small grids, α = 1.5 is safer.
 *   - TRACER PARTICLES that drift outside the domain must be
 *     RESPAWNED at the inflow (or random domain interior for
 *     closed scenarios) to maintain density.
 *   - OBSTACLE CELLS (cylinder, step) get u=v=0 + Thom's wall
 *     vorticity, just like walls.  The implementation iterates
 *     over the obstacle mask and applies wall-style BC at every
 *     internal interface.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - KARMAN STREET (default): within ~5 seconds at Re=200 the
 *     wake develops alternating vortex shedding.  Press 'x' to
 *     toggle tracers — the staggered von Karman pattern becomes
 *     OBVIOUS.
 *   - LID CAVITY: a single recirculating vortex centred near the
 *     middle of the box.  At Re=400+, secondary corner vortices
 *     appear in the bottom-left and bottom-right.
 *   - FREE JET: fluid blasts in from the centre-left, mushroom-tip
 *     vortices form at the leading edge, jet entrains surrounding
 *     fluid.  Tracers visible curving inward.
 *   - BACKWARD STEP: flow over the step separates; a recirculation
 *     bubble forms behind the step.  Tracers in the bubble go
 *     ROUND AND ROUND while the main flow continues downstream.
 *   - Press '+' to bump Re higher: vortices sharper, more chaotic.
 *     Press '-' for lower Re: smooth creeping flow.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1   Vorticity and streamfunction — what are they?
 *   T2   Why ψ-ω formulation — pressure elimination
 *   T3   Vorticity transport equation (curl of NS)
 *   T4   Poisson equation ∇²ψ = -ω and the SOR solver
 *   T5   Velocity recovery + adaptive dt
 *   T6   Boundary conditions and Thom's formula
 *   T7   The four scenarios (BCs in detail)
 *   T8   Karman vortex street — physics + history
 *   T9   Lagrangian tracer particles for visualisation
 *   T10  Reading the screen — what each view shows
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  VORTICITY AND STREAMFUNCTION — WHAT ARE THEY?
 * ─────────────────────────────────────────────────
 * Two scalar fields that replace the usual velocity vector + pressure
 * representation of fluid flow.
 *
 * VORTICITY ω(x, y).  At each point, ω measures the LOCAL ROTATION
 * RATE of the fluid.  Imagine a tiny paddle wheel placed at the
 * point and free to rotate:
 *
 *   - In rigid-body rotation at angular velocity Ω, the paddle
 *     rotates at exactly Ω.  ω = 2Ω.
 *   - In a uniform-velocity flow (wind blowing east), no paddle
 *     rotates.  ω = 0 everywhere.
 *   - In a SHEAR FLOW (top moving right, bottom stationary), the
 *     paddle spins because its top is dragged faster than its
 *     bottom.  ω ≠ 0.
 *
 * Mathematically, ω is the z-component of the curl of velocity
 * (the only non-zero component in 2-D):
 *
 *     ω = ∂v/∂x - ∂u/∂y
 *
 * Sign convention: ω > 0 means COUNTER-CLOCKWISE rotation.
 *
 * STREAMFUNCTION ψ(x, y).  A scalar whose CONTOURS ARE STREAMLINES.
 * If you sketch lines of constant ψ on the flow, you trace out the
 * paths fluid particles follow.
 *
 *   - A HORIZONTAL line of constant ψ → fluid flows horizontally
 *     along it.
 *   - CLOSELY SPACED contours → fluid moves FAST (small spacing
 *     means large gradient → large velocity).
 *   - WIDELY SPACED contours → fluid moves slow.
 *
 * Definition:
 *
 *     u =  ∂ψ/∂y          (horizontal velocity)
 *     v = -∂ψ/∂x          (vertical velocity)
 *
 * The minus sign on v is convention.  With this choice:
 *   ∇·u = ∂u/∂x + ∂v/∂y = ∂²ψ/∂x∂y - ∂²ψ/∂y∂x = 0
 * for any smooth ψ.  Continuity (incompressibility) is satisfied
 * AUTOMATICALLY.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Streamlines (lines of constant ψ):                  │
 *      │                                                      │
 *      │   ──────────────────  ψ = 3.0    (slow)              │
 *      │   ─ ─ ─ ─ ─ ─ ─ ─ ─   ψ = 2.0                        │
 *      │   ───────────────────  ψ = 1.0   (fast — close)      │
 *      │   ────────────────────  ψ = 0.5                      │
 *      │   ────────────────────  ψ = 0.0                      │
 *      │                                                      │
 *      │   Speed = |∇ψ|.  Closely-spaced contours = fast flow.│
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T2  WHY ψ-ω FORMULATION — PRESSURE ELIMINATION
 * ──────────────────────────────────────────────
 * The usual incompressible Navier-Stokes solver tracks (u, v, p):
 *
 *     ∂u/∂t + (u·∇)u = -∇p/ρ + ν·∇²u    (momentum)
 *     ∇·u = 0                            (continuity)
 *
 * Pressure has no time derivative — it's a Lagrange multiplier
 * enforcing continuity.  Each tick you must SOLVE A POISSON EQUATION
 * for p (the "projection step" in Stam stable fluids).  Expensive,
 * fiddly boundary conditions, easy to get wrong.
 *
 * VORTICITY-STREAMFUNCTION SIDESTEPS THIS.
 *
 * Take the curl of the momentum equation.  The curl of a gradient is
 * always zero (∇×∇p = 0), so PRESSURE DROPS OUT entirely:
 *
 *     ∂ω/∂t + (u·∇)ω = ν·∇²ω
 *
 * That's the VORTICITY TRANSPORT EQUATION.  No pressure.
 *
 * For the velocity, instead of solving for u and v separately under
 * the constraint ∇·u = 0, define ψ such that u = ∂ψ/∂y, v = -∂ψ/∂x.
 * Continuity is now AUTOMATIC.  And ψ is linked to ω by:
 *
 *     ∇²ψ = -ω           (Poisson equation)
 *
 * One scalar Poisson equation for ψ, no continuity constraint,
 * pressure ELIMINATED.
 *
 * Trade-offs:
 *   PRO  No pressure variable.
 *   PRO  Continuity exactly satisfied at every step (no drift).
 *   PRO  ψ contours give streamlines for free.
 *   CON  Restricted to 2-D (curl is a scalar in 2-D, vector in 3-D).
 *   CON  Wall boundary conditions for ω are subtle (Thom, T6).
 *
 * For 2-D viscous flow, ψ-ω is often the FORMULATION OF CHOICE in
 * textbooks and small-grid simulations.  Stam's stable fluids
 * (navier_stokes.c) uses primitive variables instead because it
 * generalises to 3-D and handles complex geometry better.
 *
 * T3  VORTICITY TRANSPORT EQUATION (CURL OF NS)
 * ─────────────────────────────────────────────
 * Start with the incompressible 2-D Navier-Stokes momentum equation:
 *
 *     ∂u/∂t + u·∂u/∂x + v·∂u/∂y = -∂p/∂x/ρ + ν·∇²u
 *     ∂v/∂t + u·∂v/∂x + v·∂v/∂y = -∂p/∂y/ρ + ν·∇²v
 *
 * Take ∂(second)/∂x - ∂(first)/∂y:
 *
 *     ∂(∂v/∂x - ∂u/∂y)/∂t  +  (cross-derivative terms)
 *       =  ∂(-∂p/∂y/ρ)/∂x - ∂(-∂p/∂x/ρ)/∂y
 *       +  ν·(∇²(∂v/∂x - ∂u/∂y))
 *
 * The pressure terms cancel (∂²p/∂x∂y = ∂²p/∂y∂x).  The first term
 * is ∂ω/∂t.  After algebra (using ∇·u = 0 to simplify the cross
 * terms), the cross-derivative collapses to (u·∇)ω, and we get:
 *
 *     ∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y = ν·∇²ω
 *      ─────  ─────────────────   ──────
 *      time     advection         diffusion
 *
 * Read this as: "vorticity is CARRIED BY the flow (advection),
 * SPREAD by viscosity (diffusion).  No source term in the
 * interior — ω is created only at boundaries (walls)."
 *
 * NUMERICAL IMPLEMENTATION (§10):
 *   - Discrete time:  forward Euler.
 *   - Advection u·∂ω/∂x:  UPWIND first-order differencing.  Why
 *     upwind?  Central differences for advection have NEGATIVE
 *     numerical diffusion — they amplify oscillations.  Upwind
 *     introduces small positive numerical diffusion (which is fine
 *     for stability).
 *   - Diffusion ν·∇²ω:  central second-difference.  Always
 *     stable for diffusion since the term is dissipative.
 *
 * T4  POISSON EQUATION ∇²ψ = -ω AND THE SOR SOLVER
 * ────────────────────────────────────────────────
 * After advancing ω in time, we need to recover the velocity field.
 * That requires ψ.  ψ satisfies the POISSON EQUATION:
 *
 *     ∇²ψ = -ω
 *
 * Discretise on a 5-point stencil.  At interior cell (x, y):
 *
 *     (ψ_E + ψ_W - 2ψ)/dx² + (ψ_N + ψ_S - 2ψ)/dy² = -ω
 *
 * Solve for ψ at the centre cell:
 *
 *     ψ_GS = [dy²·(ψ_E + ψ_W) + dx²·(ψ_N + ψ_S) + dx²·dy²·ω]
 *             / (2·(dx² + dy²))
 *
 * (When dx = dy this collapses to the simpler ψ_GS = (sum of 4
 * neighbours + h²·ω) / 4.)  This is one GAUSS-SEIDEL iteration:
 * each cell uses the LATEST values of its neighbours.
 *
 * SUCCESSIVE OVER-RELAXATION (SOR) speeds this up.  Instead of
 * setting ψ_new = ψ_GS, we OVER-CORRECT:
 *
 *     ψ_new = ψ + α · (ψ_GS - ψ)         α ∈ (1, 2)
 *
 * α = 1 → pure Gauss-Seidel.  α > 1 → over-correction, faster
 * convergence.  α → 2 → diverges.  Optimum for square grid of
 * size N:
 *
 *     α_opt ≈ 2 / (1 + sin(π/(N+1)))
 *
 * For N ≈ 80 cells, α_opt ≈ 1.92.  We use α = 1.7, a safe value
 * across grid sizes.
 *
 * Convergence rate: each sweep reduces the error by factor
 * (α - 1)/(1 - sin(π/(N+1))).  At α = 1.7 and N = 80, that's
 * roughly 0.7 per sweep.  14 sweeps reduce the error by 0.7¹⁴ ≈
 * 0.007 — three orders of magnitude.  Visually exact.
 *
 * SCHEDULE PER TICK:
 *   - update ω (forward Euler, T3)
 *   - Poisson solve for ψ (14 SOR sweeps, this section)
 *   - recompute (u, v) from ψ (centred differences, T5)
 *   - apply boundary conditions (T6)
 *
 * T5  VELOCITY RECOVERY + ADAPTIVE DT
 * ───────────────────────────────────
 * After the Poisson solve, ψ is up to date.  Recover velocity via:
 *
 *     u[i, j] =  (ψ[i, j+1] - ψ[i, j-1]) / (2 · dy)
 *     v[i, j] = -(ψ[i+1, j] - ψ[i-1, j]) / (2 · dx)
 *
 * Central differences — second-order accurate.  At walls, override:
 *
 *     stationary wall:  u = v = 0
 *     moving lid:       u = U_lid, v = 0
 *
 * ADAPTIVE TIMESTEP.  Forward Euler stability requires:
 *
 *     dt < dx / max|u|             (advection / CFL bound)
 *     dt < dx² / (4 · ν)           (diffusion bound, 2-D)
 *
 * Take the MIN of the two and apply a safety factor (CFL = 0.25):
 *
 *     dt = CFL · min(dx / max|u|,  dx² / (4·ν))
 *
 * This is recomputed every tick because max|u| changes as the flow
 * evolves.  At Re = 100, dt ≈ 0.005.  At Re = 1000, dt ≈ 0.001.
 *
 * The CFL factor of 0.25 is conservative.  Pushing toward 1.0
 * works for diffusion but is risky for the advection bound at
 * high Re where local velocity spikes can occur.
 *
 * T6  BOUNDARY CONDITIONS AND THOM'S FORMULA
 * ──────────────────────────────────────────
 * Boundary conditions for ψ-ω are SUBTLE.  Three boundary types
 * appear in our scenarios:
 *
 *   NO-SLIP WALL (stationary or moving):
 *     ψ_wall = constant       (no flow THROUGH the wall)
 *     u, v at wall = explicit  (= 0 for stationary, = U_lid for lid)
 *     ω_wall = COMPUTED FROM INTERIOR ψ via THOM'S FORMULA:
 *
 *        ω_wall = -2 · ψ_interior / h²       (stationary)
 *        ω_lid  = -2 · ψ_interior / h² - 2·U_lid / h
 *
 *     Why?  The vorticity at a no-slip wall reflects the velocity
 *     gradient there.  Taylor-expand ψ near the wall:
 *
 *        ψ_interior ≈ ψ_wall + h · ∂ψ/∂n + h²/2 · ∂²ψ/∂n²
 *
 *     With ψ_wall = 0 and ∂ψ/∂n = -tangential velocity (ψ derivative
 *     normal to wall = velocity along wall), solving for the second
 *     derivative gives ω = -∇²ψ at the wall, hence the formula.
 *
 *     For a moving lid, the tangential velocity at the wall is U_lid,
 *     hence the extra -2·U_lid/h term.
 *
 *     This is THOM (1933) — the canonical reference.
 *
 *   INFLOW (uniform velocity entering the domain):
 *     u = U_in everywhere on this side.
 *     v = 0.
 *     ψ varies LINEARLY with the perpendicular coordinate:
 *        ψ(x=0, y) = U_in · y
 *     ω = 0 (uniform flow has no rotation).
 *
 *   OUTFLOW (zero gradient — fluid slides out):
 *     ∂ψ/∂n = ∂ω/∂n = 0     (Neumann BC)
 *     Practically: ghost cell value = nearest interior value.
 *     Some reflection occurs; not perfect ABC.
 *
 * Each scenario specifies the BC type for each of the four walls.
 * The boundary application function (§9) handles all combinations.
 *
 * T7  THE FOUR SCENARIOS (BCs IN DETAIL)
 * ──────────────────────────────────────
 * KARMAN STREET:
 *   left   = INFLOW (U_in = 1)
 *   right  = OUTFLOW
 *   top    = WALL or FREE-SLIP (we use wall for simplicity)
 *   bottom = WALL
 *   obstacle = circular cylinder at 25% from left, centred vertically
 *   PHYSICS: at Re ~50-200 the wake oscillates, vortices shed from
 *            top/bottom alternately → KARMAN VORTEX STREET.
 *
 * LID CAVITY:
 *   all walls stationary except TOP, which moves right at U_lid.
 *   no obstacles.
 *   PHYSICS: steady recirculating vortex inside the cavity.
 *
 * FREE JET:
 *   left wall — center band (20% of height) is INFLOW; rest is wall.
 *   right wall — OUTFLOW.
 *   top, bottom — walls.
 *   no obstacle.
 *   PHYSICS: jet enters, entrains surrounding fluid, mushroom tips.
 *
 * BACKWARD STEP:
 *   left wall — top half (above step) is INFLOW; bottom half is wall
 *               (the step).
 *   right    — OUTFLOW.
 *   top, bottom — walls.
 *   step is a solid block in the bottom-left corner.
 *   PHYSICS: flow separates behind the step, recirculation bubble
 *            forms.
 *
 * T8  KARMAN VORTEX STREET — PHYSICS + HISTORY
 * ────────────────────────────────────────────
 * In 1911, Theodore von Karman (a 30-year-old aerodynamicist in
 * Germany) was watching laboratory flow visualisations of fluid
 * past a circular cylinder.  At low Reynolds number (Re < 50) the
 * flow was steady — two symmetric eddies behind the cylinder.  At
 * higher Re, those eddies became UNSTABLE and shed alternately
 * into the wake.
 *
 * The result: a VON KARMAN VORTEX STREET — two parallel rows of
 * counter-rotating vortices drifting downstream.  Equally spaced
 * within each row; offset between rows.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  upstream                                downstream  │
 *      │   →    ●   →                            (CCW)        │
 *      │   →    ●   →    →   ↺   →   ●  →  ↺   →  ●           │
 *      │   →    ●   →                                         │
 *      │   →    ●   →    →   ●   →  ↻   →  ●  →  ↻            │
 *      │   →    ●   →                            (CW)         │
 *      │                                                      │
 *      │   uniform flow      cylinder         alternating     │
 *      │                                      vortex pairs    │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Critical Reynolds numbers:
 *   Re < ~5         creeping flow, no separation
 *   Re ~5-40        steady recirculation (twin eddies behind cylinder)
 *   Re ~40-90       wake oscillates, vortex shedding begins
 *   Re ~100-200     fully developed Karman street, periodic
 *   Re > 300        wake becomes 3-D unstable (we still see 2-D pattern)
 *   Re > 10000      turbulent wake, but Karman street still visible at
 *                    larger scales
 *
 * The STROUHAL NUMBER quantifies the shedding frequency:
 *   St = f · D / U  ≈  0.21 (universal, weak Re dependence)
 * where f = shedding frequency, D = cylinder diameter, U = flow
 * speed.  Telegraph wires sing in the wind at the Strouhal
 * frequency.  Tall buildings sway at it.  Submarine periscopes
 * vibrate at it.  Smokestacks have spiral vanes welded around them
 * to BREAK the regular shedding (otherwise they fatigue and fail).
 *
 * Real-world relevance:
 *   - Aeolian harps + organ pipes: vortex shedding excites resonance.
 *   - The Tacoma Narrows bridge collapse (1940) — vortex-induced
 *     oscillation, not just wind drag.
 *   - Underwater cables ringing in ocean currents.
 *   - Insect flight: vortex shedding from wings drives lift.
 *
 * In our simulator at Re=100, you should see crisp alternating
 * shedding within ~5 sec.  Toggle tracers ('x') for the iconic
 * staggered pattern.
 *
 * T9  LAGRANGIAN TRACER PARTICLES FOR VISUALISATION
 * ─────────────────────────────────────────────────
 * The vorticity / streamfunction / velocity fields are abstract.
 * Heatmaps help, but they don't convey MOVEMENT — and movement is
 * what fluid is.  TRACER PARTICLES fix this.
 *
 * Idea: scatter ~400 dots through the domain.  Each tick, advect
 * each dot by the local velocity field:
 *
 *     pos += velocity_at(pos) · dt
 *
 * `velocity_at(pos)` is bilinearly interpolated from the (u, v)
 * arrays.  After many ticks, the dots TRACE OUT the flow.
 * Vortices become visible as dots circling each other; jets show
 * as streams of dots streaking through; recirculation bubbles
 * become dots going round and round.
 *
 * Particle lifecycle:
 *   - SPAWN at a sensible location (inflow boundary for inflow
 *     scenarios; uniformly distributed in the domain for closed
 *     scenarios like the lid cavity).
 *   - ADVECT each tick.
 *   - RESPAWN when the particle exits the domain or its TRAIL
 *     gets too long (otherwise old data clutters the visualisation).
 *
 * Drawing: just put a single character at the particle's rounded
 * (col, row).  We use '.' for resting, brighter glyphs at higher
 * speeds.  Optionally: keep a short trail of past positions for a
 * "comet" effect.
 *
 * Lagrangian particles are the OLDEST flow-visualisation
 * technique — predates computers.  Real wind tunnels use SMOKE,
 * dye, or oil droplets.  Same principle, real particles.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Without tracers: heatmap                            │
 *      │     red blob ↔ blue blob                             │
 *      │     "vortices?"                                       │
 *      │                                                      │
 *      │  With tracers:                                       │
 *      │     ↻ ↺ ↻ ↺                                          │
 *      │     ●●●●  ●●●●  ●●●●  ●●●●                            │
 *      │     "OH that's a Karman street!"                     │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T10 READING THE SCREEN — WHAT EACH VIEW SHOWS
 * ─────────────────────────────────────────────
 * Three base modes plus the tracer overlay:
 *
 *   VORTICITY ('v'):  diverging blue → grey → red.
 *     RED CELLS    counter-clockwise spinning fluid (positive ω).
 *     BLUE CELLS   clockwise spinning fluid (negative ω).
 *     GREY         irrotational (ω ≈ 0).
 *     INTERPRETATION: each shed vortex shows as a coloured BLOB.
 *     Karman street: alternating red-blue-red-blue blobs in the wake.
 *
 *   STREAMLINES ('s'):  banded ψ contours.
 *     DARK-LIGHT alternating bands trace out the streamlines of
 *     the flow.  Closely-spaced bands = fast flow.  Closed loops =
 *     vortices.
 *
 *   VELOCITY ('w'):  speed-magnitude heatmap.
 *     Bright cells = fast.  Dark cells = slow.
 *     Karman street: a "shadow" stretches downstream of the cylinder
 *     where speeds are reduced.
 *
 *   TRACERS ('x' to toggle):  Lagrangian particles overlaid on top
 *     of any base mode.  ~400 dots tracing the flow.  THE killer
 *     feature for understanding what's happening.
 *
 * Best combination for understanding Karman street:
 *   1. Start in VORTICITY mode (default).  See the alternating
 *      red/blue blobs.
 *   2. Toggle tracers ('x').  Watch dots curve around each blob.
 *   3. Switch to STREAMLINES ('s').  See the wavy wake structure.
 *   4. Switch to VELOCITY ('w').  See the speed shadow.
 *   5. Bump Re ('+'), watch the shedding sharpen.
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

/* §1.1 — frame timing. */

#define RENDER_FPS_CAP 30
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.2 — grid limits. */

#define GRID_COLS_MAX 200
#define GRID_ROWS_MAX 60

/* §1.3 — physics. */

/* Lid / inflow speed (dimensionless; problem non-dimensionalised so
 * that U = L = 1, hence Re = 1/ν). */
#define INFLOW_VELOCITY 1.0f

/* Reynolds number presets — cycled with '+' / '-' or set per scenario. */
static const float reynolds_preset_table[] = {50.0f, 100.0f, 200.0f, 400.0f,
                                              1000.0f};
#define REYNOLDS_PRESET_COUNT 5

/* SOR over-relaxation factor.  α ∈ (1, 2); ~1.7 robust for our size. */
#define SOR_OMEGA 1.7f

/* SOR sweeps per Poisson solve. */
#define POISSON_SOR_SWEEPS 14

/* CFL safety factor for adaptive dt. */
#define CFL_SAFETY_FACTOR 0.25f

/* Sub-steps per render frame (more = faster physics evolution). */
#define SUBSTEPS_PER_FRAME 8

/* §1.4 — view modes. */

enum {
  VIEW_VORTICITY = 0,
  VIEW_STREAMLINES,
  VIEW_VELOCITY,
  VIEW_COUNT,
};

static const char *view_name_table[VIEW_COUNT] = {"vorticity ", "streamlines",
                                                  "velocity  "};

/* §1.5 — boundary side kinds. */

enum {
  BC_SIDE_WALL_STATIONARY = 0,
  BC_SIDE_WALL_MOVING,        /* lid moving at INFLOW_VELOCITY */
  BC_SIDE_INFLOW_UNIFORM,     /* uniform u = INFLOW_VELOCITY  */
  BC_SIDE_INFLOW_TOP_HALF,    /* uniform u for top half       */
  BC_SIDE_INFLOW_CENTER_BAND, /* uniform u for centre 20%     */
  BC_SIDE_OUTFLOW             /* zero-gradient                */
};

/* §1.6 — scenario IDs. */

enum {
  SCENARIO_KARMAN = 0,
  SCENARIO_LID_CAVITY = 1,
  SCENARIO_FREE_JET = 2,
  SCENARIO_BACKWARD_STEP = 3,
  SCENARIO_COUNT,
};

/* §1.7 — colour pair IDs. */

enum {
  PAIR_VORT_NEG_STRONG = 1,
  PAIR_VORT_NEG_MID,
  PAIR_VORT_NEG_WEAK,
  PAIR_VORT_ZERO,
  PAIR_VORT_POS_WEAK,
  PAIR_VORT_POS_MID,
  PAIR_VORT_POS_STRONG,

  PAIR_VEL_FIRST, /* +0..+7  velocity ramp */
  PAIR_VEL_LAST = PAIR_VEL_FIRST + 7,

  PAIR_LID,         /* yellow lid marker     */
  PAIR_OBSTACLE,    /* solid wall / cylinder */
  PAIR_TRACER,      /* tracer particles      */
  PAIR_TRACER_FAST, /* fast tracer particles */
  PAIR_HUD,         /* yellow + bold         */
  PAIR_HINT,        /* cyan + bold           */
};

/* §1.8 — tracer particles (T9). */

#define TRACER_COUNT_MAX 500
#define TRACER_COUNT_DEFAULT 400
#define TRACER_LIFETIME_MIN 200 /* ticks before respawn */
#define TRACER_LIFETIME_MAX 400

/* §1.9 — vorticity diverging-band thresholds. */

#define VORT_BAND_STRONG_FRAC 0.60f /* |ω/ω_max| > 0.60 → strong */
#define VORT_BAND_MID_FRAC 0.25f
#define VORT_BAND_WEAK_FRAC 0.05f

/* §1.10 — velocity ramp glyph count. */

#define VEL_RAMP_SIZE 8

/* §1.11 — streamline contour count. */

#define STREAMLINE_BAND_COUNT 12

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
/* §3  rng — small wrapper for tracer respawn                            */
/* ===================================================================== */

static float rand_uniform_unit(void) { return (float)rand() / (float)RAND_MAX; }

static int rand_in_range(int lo, int hi_exclusive) {
  if (hi_exclusive <= lo)
    return lo;
  return lo + rand() % (hi_exclusive - lo);
}

/* ===================================================================== */
/* §4  themes — diverging vorticity + sequential velocity                */
/* ===================================================================== */
/*
 * Pair init values are chosen so that the bottom of each ramp lies
 * in the BRIGHT half of the 256-cube (≥ 30) so even faint bands
 * stay legible against the default-black terminal.
 *
 * Velocity ramp uses xterm-256 gradients dark-blue → cyan → yellow
 * → red.  Vorticity uses the canonical diverging blue/grey/red.
 */

/* Velocity glyph ramp — sparse to dense. */
static const char velocity_glyph_table[VEL_RAMP_SIZE] = {' ', '.', ':', '+',
                                                         'x', 'X', '#', '@'};

/* Tracer particle glyph ramp by speed band. */
static const char tracer_glyph_table[4] = {'.', 'o', 'O', '@'};

/* ===================================================================== */
/* §5  colors — pair init + diverging-band picker                        */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

static void colors_init(void) {
  start_color();
  use_default_colors();
  terminal_has_256_colours = (COLORS >= 256);

  if (terminal_has_256_colours) {
    /* Vorticity diverging — blue (CW spin) → grey → red (CCW). */
    init_pair(PAIR_VORT_NEG_STRONG, 19, -1);  /* deep navy */
    init_pair(PAIR_VORT_NEG_MID, 27, -1);     /* mid blue  */
    init_pair(PAIR_VORT_NEG_WEAK, 117, -1);   /* light blue*/
    init_pair(PAIR_VORT_ZERO, 244, -1);       /* mid grey  */
    init_pair(PAIR_VORT_POS_WEAK, 217, -1);   /* salmon    */
    init_pair(PAIR_VORT_POS_MID, 202, -1);    /* orange    */
    init_pair(PAIR_VORT_POS_STRONG, 196, -1); /* bright red*/

    /* Velocity ramp — dark blue → cyan → yellow → red. */
    init_pair(PAIR_VEL_FIRST + 0, 19, -1);
    init_pair(PAIR_VEL_FIRST + 1, 27, -1);
    init_pair(PAIR_VEL_FIRST + 2, 39, -1);
    init_pair(PAIR_VEL_FIRST + 3, 51, -1);
    init_pair(PAIR_VEL_FIRST + 4, 118, -1);
    init_pair(PAIR_VEL_FIRST + 5, 226, -1);
    init_pair(PAIR_VEL_FIRST + 6, 208, -1);
    init_pair(PAIR_VEL_FIRST + 7, 196, -1);

    init_pair(PAIR_LID, 226, -1);         /* yellow         */
    init_pair(PAIR_OBSTACLE, 245, -1);    /* mid grey       */
    init_pair(PAIR_TRACER, 255, -1);      /* white          */
    init_pair(PAIR_TRACER_FAST, 226, -1); /* yellow         */
    init_pair(PAIR_HUD, 226, -1);         /* bright yellow  */
    init_pair(PAIR_HINT, 51, -1);         /* bright cyan    */
  } else {
    /* 8-colour fallback. */
    init_pair(PAIR_VORT_NEG_STRONG, COLOR_BLUE, -1);
    init_pair(PAIR_VORT_NEG_MID, COLOR_BLUE, -1);
    init_pair(PAIR_VORT_NEG_WEAK, COLOR_CYAN, -1);
    init_pair(PAIR_VORT_ZERO, COLOR_WHITE, -1);
    init_pair(PAIR_VORT_POS_WEAK, COLOR_MAGENTA, -1);
    init_pair(PAIR_VORT_POS_MID, COLOR_RED, -1);
    init_pair(PAIR_VORT_POS_STRONG, COLOR_RED, -1);
    for (int i = 0; i < VEL_RAMP_SIZE; i++)
      init_pair(PAIR_VEL_FIRST + i, (i < 4) ? COLOR_CYAN : COLOR_YELLOW, -1);
    init_pair(PAIR_LID, COLOR_YELLOW, -1);
    init_pair(PAIR_OBSTACLE, COLOR_WHITE, -1);
    init_pair(PAIR_TRACER, COLOR_WHITE, -1);
    init_pair(PAIR_TRACER_FAST, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/*
 * VortBand — one cell's drawing instruction, derived from local ω.
 *
 * Intent
 *   The vorticity field ω = ∇×v is SIGNED — positive (counter-
 *   clockwise rotation) and negative (clockwise) need OPPOSITE
 *   colours.  vorticity_band_for() maps the normalised ω value to a
 *   diverging-palette tier (strong-CCW / weak-CCW / near-zero /
 *   weak-CW / strong-CW), picking glyph + colour pair + attribute in
 *   one shot.
 *
 * Why a struct (not three return values)
 *   C functions return one value; bundling pair + glyph + attr in a
 *   POD struct keeps the per-cell loop linear:
 *     `b = vorticity_band_for(w); attron(b.pair|b.attr); ...`
 *
 * Reference [8] Bourke for the diverging glyph design.
 */
typedef struct {
    int    pair;      /* colour-pair index (CW/CCW tier)            */
    char   glyph;     /* '.' / '*' / '#' / '@' depending on band    */
    attr_t attr;      /* A_BOLD for strongest tier                  */
} VortBand;

static VortBand vorticity_band_for(float omega_normalised) {
  if (omega_normalised < -VORT_BAND_STRONG_FRAC)
    return (VortBand){PAIR_VORT_NEG_STRONG, '#', A_BOLD};
  if (omega_normalised < -VORT_BAND_MID_FRAC)
    return (VortBand){PAIR_VORT_NEG_MID, 'x', A_NORMAL};
  if (omega_normalised < -VORT_BAND_WEAK_FRAC)
    return (VortBand){PAIR_VORT_NEG_WEAK, '.', A_NORMAL};
  if (omega_normalised < VORT_BAND_WEAK_FRAC)
    return (VortBand){PAIR_VORT_ZERO, ' ', A_NORMAL};
  if (omega_normalised < VORT_BAND_MID_FRAC)
    return (VortBand){PAIR_VORT_POS_WEAK, '.', A_NORMAL};
  if (omega_normalised < VORT_BAND_STRONG_FRAC)
    return (VortBand){PAIR_VORT_POS_MID, 'x', A_NORMAL};
  return (VortBand){PAIR_VORT_POS_STRONG, '#', A_BOLD};
}

/* ===================================================================== */
/* §6  ramp — value to glyph-slot helpers                                */
/* ===================================================================== */

static int speed_to_velocity_slot(float speed, float speed_max) {
  if (speed_max < 1e-6f)
    return 0;
  int slot = (int)(speed / speed_max * (float)(VEL_RAMP_SIZE - 1) + 0.5f);
  if (slot < 0)
    slot = 0;
  if (slot >= VEL_RAMP_SIZE)
    slot = VEL_RAMP_SIZE - 1;
  return slot;
}

/* ===================================================================== */
/* §7  grid_state — psi, omega, omega_next, u, v, walls                  */
/* ===================================================================== */
/*
 * All fields are flat row-major arrays of g_scene.grid_rows * g_scene.grid_cols.
 * Index helper: cell_index(x, y) = y * g_scene.grid_cols + x.
 *
 * GRID coordinates: (x, y) with x ∈ [0, g_scene.grid_cols-1], y ∈ [0,
 * g_scene.grid_rows-1].  y = 0 is the BOTTOM of the physics domain.  When
 * rendering, we INVERT y so the visual top of the screen matches
 * the highest y (matches the lid orientation).
 */

#define GRID_TOTAL_CELLS (GRID_COLS_MAX * GRID_ROWS_MAX)

/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Earlier versions kept scene state as flat file-scope globals.
 *   This Scene struct groups the same fields together so the sim /
 *   tuning / control / render split is enforced by the STRUCT
 *   LAYOUT, not just by convention.  Helpers continue to reach the
 *   state through `g_scene.<field>` so the inner-loop ψ-ω stencil
 *   math stays free of pointer threading.
 *
 *   Tracer pool, active-scenario index, view-mode, and paused flag
 *   are declared in §14/§15/§22 next to the code that owns them; for
 *   the consolidated picture see those sections.  Everything BELOW
 *   is what the per-step solver loop reads/writes.
 *
 * Locality (sim vs render)
 *   Fields are GROUPED EXPLICITLY by subsystem:
 *     - SOR Poisson + ω-advect + ω-diffuse + boundary + velocity-from-ψ
 *                                          → simulation
 *     - render_vorticity_view / render_streamlines_view / tracers
 *                                          → rendering
 *     - both sides bound their loops (g_scene.grid_cols, g_scene.grid_rows,
 *       g_scene.cell_size_x, _y)                   → shared geometry
 *     - simulation_paused gates the tick   → control state (§22)
 *
 *   Mis-classifying a field — e.g. letting render_vorticity_view bump
 *   g_scene.kinematic_viscosity — would couple visuals to physics and break
 *   reproducibility (the same scenario at the same Re must evolve
 *   identically regardless of which view is displayed).
 *
 * Why one big struct (not split across Scene + App + Field)
 *   Every solver pass needs the whole field set.  Splitting would
 *   force pointer chains in the inner SOR loop — already 14 sweeps
 *   per tick × W·H cells — and the extra indirection would slow it
 *   noticeably.  Keeping one Scene + one g_scene instance matches
 *   CLAUDE.md's "no dynamic allocation after init": ~6 × GRID_TOTAL
 *   floats in BSS, no malloc anywhere.
 *
 * Why these specific fields and no others
 *   - g_scene.streamfunction_psi   ψ field — primary unknown.  Solved each
 *                          step by SOR Poisson against vorticity.
 *   - g_scene.vorticity_omega      ω field — second primary unknown.  Advected
 *                          + diffused each step.
 *   - g_scene.vorticity_omega_next ω scratch for advection/diffusion step.
 *   - g_scene.velocity_x / _y      v = ∇×ψ — DERIVED but cached so tracers and
 *                          render passes don't recompute the curl.
 *   - g_scene.wall_mask            true at solid cells.  Scenario-configured.
 *   - g_scene.grid_cols, g_scene.grid_rows active subregion bounds.
 *   - g_scene.cell_size_x, _y      physical Δx, Δy used by the solver.
 *   - g_scene.kinematic_viscosity  ν, derived from Reynolds number.
 *   - g_scene.reynolds_number      Re = U·L/ν.
 *   - g_scene.reynolds_preset_index  index into the REYNOLDS_TABLE tier.
 *   - g_scene.current_dt           adaptive timestep, recomputed from CFL.
 *   - g_scene.velocity_max_smoothed  EMA of |v| max — normalises arrows.
 *   - g_scene.vorticity_max_smoothed EMA of |ω| max — normalises VortBand.
 *
 * Things that DO NOT live here
 *   - Tracer pool / view mode / paused flag    → declared in §14, §22
 *   - 5 scenario presets / 6 reynolds tiers    → file-scope constants
 *   - Render-frame timing / FPS counter        → locals in main()
 *   - Signal flags (SIGINT, SIGWINCH)          → file-scope volatile
 *
 * References [1] Roache for the ψ-ω state layout; [6] Saad for the
 *   SOR convergence properties that motivate this buffer design.
 */
typedef struct {
    /* ── Simulation state (read by all physics passes) ──────────── */
    float streamfunction_psi  [GRID_TOTAL_CELLS];  /* ψ — primary    */
    float vorticity_omega     [GRID_TOTAL_CELLS];  /* ω — primary    */
    float vorticity_omega_next[GRID_TOTAL_CELLS];  /* ω scratch      */
    float velocity_x          [GRID_TOTAL_CELLS];  /* u = ∂ψ/∂y      */
    float velocity_y          [GRID_TOTAL_CELLS];  /* v = -∂ψ/∂x     */
    bool  wall_mask           [GRID_TOTAL_CELLS];  /* solid cells    */

    /* ── Shared geometry (sim AND render) ──────────────────────── */
    int   grid_cols;
    int   grid_rows;
    float cell_size_x;          /* physical Δx                       */
    float cell_size_y;          /* physical Δy                       */

    /* ── Simulation tuning (Re-driven) ─────────────────────────── */
    float kinematic_viscosity;  /* ν                                 */
    float reynolds_number;      /* Re = U·L/ν                        */
    int   reynolds_preset_index;
    float current_dt;           /* adaptive timestep                 */

    /* ── Pure render diagnostics (must not touch sim fields) ───── */
    float velocity_max_smoothed;  /* EMA of |v| max — arrow scale    */
    float vorticity_max_smoothed; /* EMA of |ω| max — VortBand scale */
} Scene;

static Scene g_scene = {
    .cell_size_x            = 1.0f,
    .cell_size_y            = 1.0f,
    .kinematic_viscosity    = 1.0f / 100.0f,
    .reynolds_number        = 100.0f,
    .reynolds_preset_index  = 1,
    .current_dt             = 0.001f,
    .velocity_max_smoothed  = 1.0f,
    .vorticity_max_smoothed = 1.0f,
};

static inline int cell_index(int x, int y) { return y * g_scene.grid_cols + x; }

static void grid_zero_all_fields(void) {
  int n = g_scene.grid_rows * g_scene.grid_cols;
  memset(g_scene.streamfunction_psi, 0, sizeof(float) * n);
  memset(g_scene.vorticity_omega, 0, sizeof(float) * n);
  memset(g_scene.vorticity_omega_next, 0, sizeof(float) * n);
  memset(g_scene.velocity_x, 0, sizeof(float) * n);
  memset(g_scene.velocity_y, 0, sizeof(float) * n);
}

static void grid_clear_walls(void) {
  int n = g_scene.grid_rows * g_scene.grid_cols;
  memset(g_scene.wall_mask, 0, sizeof(bool) * n);
}

/* ===================================================================== */
/* §8  obstacle_layouts — cylinder, step builders                        */
/* ===================================================================== */

static void obstacle_build_cylinder(float cx_frac, float cy_frac,
                                    float radius_frac) {
  float cx = cx_frac * (float)(g_scene.grid_cols - 1);
  float cy = cy_frac * (float)(g_scene.grid_rows - 1);
  float radius =
      radius_frac * (float)((g_scene.grid_cols < g_scene.grid_rows) ? g_scene.grid_cols : g_scene.grid_rows);
  float r_squared = radius * radius;

  for (int y = 0; y < g_scene.grid_rows; y++) {
    for (int x = 0; x < g_scene.grid_cols; x++) {
      float dx = (float)x - cx;
      float dy = (float)y - cy;
      if (dx * dx + dy * dy < r_squared)
        g_scene.wall_mask[cell_index(x, y)] = true;
    }
  }
}

static void obstacle_build_step(float step_height_frac) {
  int step_height = (int)(step_height_frac * (float)g_scene.grid_rows);
  int step_length = g_scene.grid_cols / 8;
  if (step_length < 1)
    step_length = 1;

  for (int y = 0; y < step_height; y++) {
    for (int x = 0; x < step_length; x++) {
      g_scene.wall_mask[cell_index(x, y)] = true;
    }
  }
}

/* ===================================================================== */
/* §9  apply_boundary — per-side BC + Thom's formula (T6)                */
/* ===================================================================== */

/*
 * BoundarySpec — the four-walled boundary configuration of a scenario.
 *
 * Intent
 *   The ψ-ω formulation needs DIFFERENT boundary treatment on each
 *   wall: a lid-driven cavity has a moving top + no-slip sides, a
 *   pipe flow has inflow on the left + outflow on the right + walls
 *   top and bottom, etc.  BoundarySpec carries one BC kind per wall
 *   (top/right/bottom/left) plus the magnitude of any inflow.
 *
 * Why four separate ints (not one global BC_KIND)
 *   Most useful scenarios have ASYMMETRIC boundaries — a single global
 *   kind couldn't express "moving top + no-slip sides".  Splitting per
 *   wall lets each preset wire its own combination from the same set
 *   of BC kinds (NOSLIP / FREE_SLIP / INFLOW / OUTFLOW / PERIODIC).
 *
 * Why inflow_velocity is here (not on each side)
 *   In our presets, when an inflow boundary exists it's always with
 *   the SAME magnitude (a uniform flow speed).  Keeping one scalar
 *   here matches that pattern; a per-side inflow magnitude would be
 *   needed only for scenarios we don't currently demonstrate.
 *
 * Reference [3] Thom 1933 for the wall-vorticity formula that
 *   converts ψ values on no-slip walls into ω boundary values.
 */
typedef struct {
    int   side_top;          /* BC kind for top wall                */
    int   side_right;         /* BC kind for right wall              */
    int   side_bottom;        /* BC kind for bottom wall             */
    int   side_left;          /* BC kind for left wall               */
    float inflow_velocity;    /* speed at any INFLOW side            */
} BoundarySpec;

/* For UNIFORM inflow, ψ varies linearly with y so that ∂ψ/∂y = U_in
 * is constant over the whole left wall — every horizontal streamline
 * enters at the same speed. */
static inline float psi_uniform_inflow_profile(int y, int rows,
                                                float inflow_velocity) {
    (void)rows;
    return inflow_velocity * (float)y * g_scene.cell_size_y;
}

/* For TOP-HALF inflow (backward-facing step) ψ is zero on the lower
 * step face and linear above it — the step acts as a no-slip "shelf"
 * the upper-half inflow has to flow over. */
static inline float psi_top_half_inflow_profile(int y, int rows,
                                                 float inflow_velocity) {
    int step_h = rows / 4;
    if (y < step_h) return 0.0f;
    return inflow_velocity * (float)(y - step_h) * g_scene.cell_size_y;
}

/* For CENTRE-BAND inflow (free jet) ψ ramps linearly inside the band
 * [40 %, 60 %], stays flat above (entrained but not driven), and is
 * zero below.  Models a planar jet from a slot. */
static inline float psi_centre_band_inflow_profile(int y, int rows,
                                                    float inflow_velocity) {
    int band_lo = rows * 4 / 10;
    int band_hi = rows * 6 / 10;
    if (y <  band_lo) return 0.0f;
    if (y >  band_hi)
        return inflow_velocity * (float)(band_hi - band_lo) * g_scene.cell_size_y;
    return inflow_velocity * (float)(y - band_lo) * g_scene.cell_size_y;
}

/* Pick the right ψ profile function for the given inflow KIND and
 * evaluate it at y.  Non-inflow kinds get ψ = 0 (no slip on the left
 * wall). */
static inline float psi_inflow_value_at(int side_kind, int y, int rows,
                                         float inflow_velocity) {
    switch (side_kind) {
        case BC_SIDE_INFLOW_UNIFORM:
            return psi_uniform_inflow_profile    (y, rows, inflow_velocity);
        case BC_SIDE_INFLOW_TOP_HALF:
            return psi_top_half_inflow_profile   (y, rows, inflow_velocity);
        case BC_SIDE_INFLOW_CENTER_BAND:
            return psi_centre_band_inflow_profile(y, rows, inflow_velocity);
        default:
            return 0.0f;
    }
}

/* LEFT side: pick the appropriate inflow profile (or zero for walls)
 * and write it to ψ along column 0. */
static inline void apply_psi_on_left_side(int side_kind, float inflow_velocity,
                                           int rows) {
    for (int y = 0; y < rows; y++)
        g_scene.streamfunction_psi[cell_index(0, y)] =
            psi_inflow_value_at(side_kind, y, rows, inflow_velocity);
}

/* RIGHT side: for OUTFLOW we copy the second-to-last column (zero
 * normal-gradient).  Other kinds (walls) leave the value alone — set
 * elsewhere in scenario load. */
static inline void apply_psi_on_right_side(int side_kind, int rows, int cols) {
    if (side_kind != BC_SIDE_OUTFLOW) return;
    for (int y = 0; y < rows; y++)
        g_scene.streamfunction_psi[cell_index(cols - 1, y)] =
            g_scene.streamfunction_psi[cell_index(cols - 2, y)];
}

/* BOTTOM side: ψ = 0 along the bottom wall — the "ground streamline"
 * by convention so vertical velocities measure absolute height of ψ. */
static inline void apply_psi_on_bottom_side(int cols) {
    for (int x = 0; x < cols; x++)
        g_scene.streamfunction_psi[cell_index(x, 0)] = 0.0f;
}

/* TOP side: for stationary walls coupled to a uniform inflow, ψ at
 * the top must equal the wall streamline value ψ = U_in · H so the
 * top streamline continues smoothly into the inflow profile. */
static inline void apply_psi_on_top_side(int side_kind, float inflow_velocity,
                                          int rows, int cols) {
    float psi_top = 0.0f;
    if (side_kind == BC_SIDE_WALL_STATIONARY)
        psi_top = inflow_velocity * (float)(rows - 1) * g_scene.cell_size_y;
    for (int x = 0; x < cols; x++)
        g_scene.streamfunction_psi[cell_index(x, rows - 1)] = psi_top;
}

/*
 * boundary_set_psi_side — apply the ψ BC for ONE side of the domain.
 *
 * Pseudocode (dispatch by which side):
 *   L : apply_psi_on_left_side   (build inflow profile or zero)
 *   R : apply_psi_on_right_side  (copy column for outflow)
 *   B : apply_psi_on_bottom_side (zero along the floor)
 *   T : apply_psi_on_top_side    (match inflow's wall streamline)
 *
 * For inflow boundaries, ψ varies LINEARLY with the perpendicular
 * coordinate so that ∂ψ/∂(perp) gives a constant velocity component.
 */
static void boundary_set_psi_side(int side_kind, char which_side,
                                   float inflow_velocity) {
    int rows = g_scene.grid_rows;
    int cols = g_scene.grid_cols;
    switch (which_side) {
        case 'L': apply_psi_on_left_side  (side_kind, inflow_velocity, rows);       break;
        case 'R': apply_psi_on_right_side (side_kind, rows, cols);                  break;
        case 'B': apply_psi_on_bottom_side(cols);                                   break;
        case 'T': apply_psi_on_top_side   (side_kind, inflow_velocity, rows, cols); break;
        default:                                                                    break;
    }
}

/* Thom's formula: wall vorticity from the streamfunction value at the
 * inner cell, one step in from the wall.  ω_wall = −2·ψ_inner / h²
 * comes from a second-order Taylor expansion of ψ at the no-slip wall
 * (ψ_wall = 0, ∂ψ/∂n_wall = 0 → ∂²ψ/∂n² gives the wall ω).
 * Refs [3] Thom 1933 (the eponymous paper). */
static inline float thom_omega_from_inner_psi(float psi_inner, float h_squared) {
    return -2.0f * psi_inner / h_squared;
}

/* Bottom wall — stationary no-slip case only.  Uses Thom's formula
 * with the row-1 ψ value. */
static inline void apply_thom_on_bottom_wall(const BoundarySpec *spec,
                                              int cols, float dy_squared) {
    if (spec->side_bottom != BC_SIDE_WALL_STATIONARY) return;
    for (int x = 0; x < cols; x++) {
        float psi_inner = g_scene.streamfunction_psi[cell_index(x, 1)];
        g_scene.vorticity_omega[cell_index(x, 0)] =
            thom_omega_from_inner_psi(psi_inner, dy_squared);
    }
}

/* Top wall — Thom's formula with an EXTRA TERM for the moving-lid
 * case.  The −2·U_lid/h₀ correction adds the wall's own velocity
 * shear to the vorticity it generates. */
static inline void apply_thom_on_top_wall(const BoundarySpec *spec,
                                           int rows, int cols, float dy_squared) {
    for (int x = 0; x < cols; x++) {
        float psi_inner = g_scene.streamfunction_psi[cell_index(x, rows - 2)];
        float psi_wall  = g_scene.streamfunction_psi[cell_index(x, rows - 1)];
        float w_thom    = -2.0f * (psi_inner - psi_wall) / dy_squared;
        if (spec->side_top == BC_SIDE_WALL_MOVING)
            w_thom -= 2.0f * spec->inflow_velocity / g_scene.cell_size_y;
        g_scene.vorticity_omega[cell_index(x, rows - 1)] = w_thom;
    }
}

/* Left side — Thom if it's a wall, ω = 0 if it's an inflow (uniform
 * incoming flow carries no vorticity). */
static inline void apply_thom_on_left_side(const BoundarySpec *spec,
                                            int rows, float dx_squared) {
    if (spec->side_left == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            float psi_inner = g_scene.streamfunction_psi[cell_index(1, y)];
            g_scene.vorticity_omega[cell_index(0, y)] =
                thom_omega_from_inner_psi(psi_inner, dx_squared);
        }
    } else {
        for (int y = 0; y < rows; y++)
            g_scene.vorticity_omega[cell_index(0, y)] = 0.0f;
    }
}

/* Right side — Thom if it's a wall, zero-gradient copy if it's an
 * outflow. */
static inline void apply_thom_on_right_side(const BoundarySpec *spec,
                                             int rows, int cols, float dx_squared) {
    if (spec->side_right == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            float psi_inner = g_scene.streamfunction_psi[cell_index(cols - 2, y)];
            g_scene.vorticity_omega[cell_index(cols - 1, y)] =
                thom_omega_from_inner_psi(psi_inner, dx_squared);
        }
    } else {
        for (int y = 0; y < rows; y++)
            g_scene.vorticity_omega[cell_index(cols - 1, y)] =
                g_scene.vorticity_omega[cell_index(cols - 2, y)];
    }
}

/* Average ψ over the non-wall 4-neighbours of cell (x, y).  Used at
 * INTERIOR OBSTACLE CELLS as a Thom-formula proxy — at the obstacle
 * boundary the average gives the correct Thom magnitude; for cells
 * fully inside the obstacle (no fluid neighbours) it returns 0, which
 * the caller turns into ω = 0. */
static inline float average_psi_of_fluid_neighbours(int x, int y,
                                                     int *out_neighbour_count) {
    int  count    = 0;
    float psi_sum = 0.0f;
    int   neighbours[4][2] = { {x - 1, y}, {x + 1, y},
                               {x, y - 1}, {x, y + 1} };
    for (int n = 0; n < 4; n++) {
        int nx = neighbours[n][0];
        int ny = neighbours[n][1];
        if (!g_scene.wall_mask[cell_index(nx, ny)]) {
            psi_sum += g_scene.streamfunction_psi[cell_index(nx, ny)];
            count++;
        }
    }
    *out_neighbour_count = count;
    return (count > 0) ? psi_sum / (float)count : 0.0f;
}

/* For every INTERIOR obstacle cell, write the Thom-formula ω from
 * the average ψ of non-wall neighbours, and force ψ = 0 inside the
 * obstacle.  Boundary cells of the obstacle generate the correct
 * Thom ω; cells fully interior get ω = 0. */
static inline void apply_thom_inside_obstacle_cells(int rows, int cols,
                                                     float dy_squared) {
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (!g_scene.wall_mask[cell_index(x, y)]) continue;
            int   count;
            float psi_avg = average_psi_of_fluid_neighbours(x, y, &count);
            g_scene.vorticity_omega   [cell_index(x, y)] =
                (count > 0) ? thom_omega_from_inner_psi(psi_avg, dy_squared) : 0.0f;
            g_scene.streamfunction_psi[cell_index(x, y)] = 0.0f;
        }
    }
}

/*
 * boundary_set_thom_wall_vorticity — populate ω at every wall via
 * Thom's formula [3] Thom 1933.
 *
 * Pseudocode:
 *   apply_thom_on_bottom_wall  (stationary case only)
 *   apply_thom_on_top_wall     (handles moving-lid extra term)
 *   apply_thom_on_left_side    (Thom or ω=0 for inflow)
 *   apply_thom_on_right_side   (Thom or zero-grad for outflow)
 *   apply_thom_inside_obstacle_cells   (avg-neighbour proxy)
 */
static void boundary_set_thom_wall_vorticity(const BoundarySpec *spec) {
    int   rows = g_scene.grid_rows;
    int   cols = g_scene.grid_cols;
    float dx2  = g_scene.cell_size_x * g_scene.cell_size_x;
    float dy2  = g_scene.cell_size_y * g_scene.cell_size_y;

    apply_thom_on_bottom_wall      (spec, cols, dy2);
    apply_thom_on_top_wall         (spec, rows, cols, dy2);
    apply_thom_on_left_side        (spec, rows, dx2);
    apply_thom_on_right_side       (spec, rows, cols, dx2);
    apply_thom_inside_obstacle_cells(rows, cols, dy2);
}

static void apply_boundary(const BoundarySpec *spec) {
  boundary_set_psi_side(spec->side_left, 'L', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_right, 'R', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_bottom, 'B', spec->inflow_velocity);
  boundary_set_psi_side(spec->side_top, 'T', spec->inflow_velocity);
  boundary_set_thom_wall_vorticity(spec);
}

/* ===================================================================== */
/* §10  vorticity_step — explicit Euler with upwind + diffusion (T3)     */
/* ===================================================================== */

/* The five values of ω in the 5-point stencil at cell (x, y) plus
 * its (u, v) velocity.  Bundled so the inner-loop body reads as
 * pure math rather than 7 array indexes. */
typedef struct {
    float omega_centre, omega_east, omega_west, omega_north, omega_south;
    float u_local, v_local;
} OmegaStencil;

static inline OmegaStencil sample_omega_stencil_at(int x, int y) {
    OmegaStencil s;
    s.omega_centre = g_scene.vorticity_omega[cell_index(x,     y    )];
    s.omega_east   = g_scene.vorticity_omega[cell_index(x + 1, y    )];
    s.omega_west   = g_scene.vorticity_omega[cell_index(x - 1, y    )];
    s.omega_north  = g_scene.vorticity_omega[cell_index(x,     y + 1)];
    s.omega_south  = g_scene.vorticity_omega[cell_index(x,     y - 1)];
    s.u_local      = g_scene.velocity_x     [cell_index(x,     y    )];
    s.v_local      = g_scene.velocity_y     [cell_index(x,     y    )];
    return s;
}

/* UPWIND finite difference for the convective term u·∇ω.  The
 * stencil leg picked is the one POINTING INTO the wind — guarantees
 * conditional stability for hyperbolic advection.  Refs [7] LeVeque
 * §6 for the upwind scheme. */
static inline float upwind_omega_gradient_x(const OmegaStencil *s, float idx) {
    return (s->u_local > 0.0f)
         ? (s->omega_centre - s->omega_west)  * idx
         : (s->omega_east   - s->omega_centre) * idx;
}

static inline float upwind_omega_gradient_y(const OmegaStencil *s, float idy) {
    return (s->v_local > 0.0f)
         ? (s->omega_centre - s->omega_south) * idy
         : (s->omega_north  - s->omega_centre) * idy;
}

/* Central-difference 5-point Laplacian for the viscous term ν·∇²ω.
 * Central works here (unlike for convection) because diffusion is
 * isotropic — there's no preferred direction. */
static inline float laplacian_omega_at(const OmegaStencil *s, float idx2, float idy2) {
    return (s->omega_east  - 2.0f * s->omega_centre + s->omega_west ) * idx2
         + (s->omega_north - 2.0f * s->omega_centre + s->omega_south) * idy2;
}

/* The vorticity transport equation discretised as a time derivative
 *   ∂ω/∂t = −u·∇ω + ν·∇²ω
 * The minus signs in front of u·∇ω come from moving the advection
 * onto the RHS.  Refs [1] Roache §3.7. */
static inline float vorticity_time_derivative(const OmegaStencil *s,
                                               float idx, float idy,
                                               float idx2, float idy2, float nu) {
    float dw_dx     = upwind_omega_gradient_x(s, idx);
    float dw_dy     = upwind_omega_gradient_y(s, idy);
    float lap_omega = laplacian_omega_at      (s, idx2, idy2);
    return - s->u_local * dw_dx
           - s->v_local * dw_dy
           + nu        * lap_omega;
}

/* Sweep INTERIOR cells; write the Euler update into omega_next.  Wall
 * cells are skipped — their ω is set by apply_boundary, not transport. */
static inline void compute_vorticity_next_field(float dt_seconds) {
    int   cols = g_scene.grid_cols;
    int   rows = g_scene.grid_rows;
    float idx  = 1.0f / g_scene.cell_size_x;
    float idy  = 1.0f / g_scene.cell_size_y;
    float idx2 = idx * idx;
    float idy2 = idy * idy;
    float nu   = g_scene.kinematic_viscosity;

    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            OmegaStencil s   = sample_omega_stencil_at(x, y);
            float        rhs = vorticity_time_derivative(&s, idx, idy,
                                                          idx2, idy2, nu);
            g_scene.vorticity_omega_next[cell_index(x, y)] =
                s.omega_centre + dt_seconds * rhs;
        }
    }
}

/* Copy interior cells from omega_next back to omega.  Cheaper than
 * pointer-swapping the whole arrays since walls were skipped. */
static inline void copy_omega_next_to_omega_interior(void) {
    int cols = g_scene.grid_cols;
    int rows = g_scene.grid_rows;
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            g_scene.vorticity_omega[cell_index(x, y)] =
                g_scene.vorticity_omega_next[cell_index(x, y)];
        }
    }
}

/*
 * vorticity_step — explicit Euler with upwind convection + central
 * diffusion of the vorticity transport equation.
 *
 * Pseudocode:
 *   compute_vorticity_next_field(dt)           ← into omega_next
 *   copy_omega_next_to_omega_interior()        ← into omega
 *
 * Refs [1] Roache 1972 §3.7; [7] LeVeque §6 (upwind hyperbolic).
 */
static void vorticity_step(float dt_seconds) {
    compute_vorticity_next_field(dt_seconds);
    copy_omega_next_to_omega_interior();
}

/* ===================================================================== */
/* §11  poisson_solve — SOR iteration of ∇²ψ = -ω (T4)                   */
/* ===================================================================== */

/* Per-cell Gauss-Seidel update for the 5-point Poisson stencil.
 *   ψ_GS = (dy²·(ψ_E + ψ_W) + dx²·(ψ_N + ψ_S) + dx²·dy²·ω) / (2·(dx² + dy²))
 * Solves the discretised ∇²ψ = −ω at cell (x, y) assuming all four
 * neighbours are correct (the GS approximation — they may not be,
 * but the iteration converges).  Refs [6] Saad §4.1. */
static inline float gauss_seidel_psi_update_at(int x, int y,
                                                float dx2, float dy2,
                                                float denominator) {
    float psi_east  = g_scene.streamfunction_psi[cell_index(x + 1, y    )];
    float psi_west  = g_scene.streamfunction_psi[cell_index(x - 1, y    )];
    float psi_north = g_scene.streamfunction_psi[cell_index(x,     y + 1)];
    float psi_south = g_scene.streamfunction_psi[cell_index(x,     y - 1)];
    float omega     = g_scene.vorticity_omega   [cell_index(x,     y    )];
    return (dy2 * (psi_east + psi_west) + dx2 * (psi_north + psi_south)
            + dx2 * dy2 * omega) / denominator;
}

/* Apply Successive Over-Relaxation: ψ_new = ψ + α·(ψ_GS − ψ).  α > 1
 * (typically 1.5..1.8) ACCELERATES the convergence of Gauss-Seidel by
 * over-shooting the new estimate in the direction of the correction.
 * α = 1 reduces to plain Gauss-Seidel; α > 2 diverges.  [6] Saad §4. */
static inline float sor_relax(float psi_centre, float psi_gauss_seidel,
                               float alpha) {
    return psi_centre + alpha * (psi_gauss_seidel - psi_centre);
}

/* One full SWEEP over the interior cells: apply the GS update then
 * the SOR relaxation in place.  GS sweeps converge in O(N²) iters,
 * SOR converges in O(N) — a real speedup for the streamfunction
 * solve which dominates this file's runtime.  Refs [6] Saad §4.2. */
static inline void sor_sweep_once(float dx2, float dy2,
                                   float denominator, float alpha) {
    int cols = g_scene.grid_cols;
    int rows = g_scene.grid_rows;
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            float psi_centre = g_scene.streamfunction_psi[cell_index(x, y)];
            float psi_gs     = gauss_seidel_psi_update_at(x, y, dx2, dy2,
                                                          denominator);
            g_scene.streamfunction_psi[cell_index(x, y)] =
                sor_relax(psi_centre, psi_gs, alpha);
        }
    }
}

/*
 * poisson_solve_sor — solve ∇²ψ = −ω via Successive Over-Relaxation.
 *
 * Pseudocode:
 *   dx² = cell_size_x²
 *   dy² = cell_size_y²
 *   denom = 2·(dx² + dy²)             ← 5-point Laplacian denominator
 *   α = SOR_OMEGA                     ← over-relaxation factor
 *   for sweep = 0 .. POISSON_SOR_SWEEPS-1:
 *     sor_sweep_once(dx², dy², denom, α)
 *
 * Refs [1] Roache §3.5; [6] Saad §4 (SOR convergence theory).
 */
static void poisson_solve_sor(void) {
    float dx2          = g_scene.cell_size_x * g_scene.cell_size_x;
    float dy2          = g_scene.cell_size_y * g_scene.cell_size_y;
    float denominator  = 2.0f * (dx2 + dy2);
    float alpha        = SOR_OMEGA;

    for (int sweep = 0; sweep < POISSON_SOR_SWEEPS; sweep++)
        sor_sweep_once(dx2, dy2, denominator, alpha);
}

/* ===================================================================== */
/* §12  velocity_recompute — derive u, v from ψ + adaptive dt (T5)       */
/* ===================================================================== */

/* Velocity components from streamfunction at one INTERIOR cell:
 *   u = +∂ψ/∂y     v = −∂ψ/∂x
 * (sign convention: ψ rises to the left of the flow direction).
 * Central differences scaled by 1/(2h).  Refs [1] Roache §1.4. */
static inline void velocity_from_psi_at_cell(int x, int y,
                                              float inv_2dx, float inv_2dy,
                                              float *out_u, float *out_v) {
    *out_u =  (g_scene.streamfunction_psi[cell_index(x,     y + 1)]
             - g_scene.streamfunction_psi[cell_index(x,     y - 1)]) * inv_2dy;
    *out_v = -(g_scene.streamfunction_psi[cell_index(x + 1, y    )]
             - g_scene.streamfunction_psi[cell_index(x - 1, y    )]) * inv_2dx;
}

/* Recompute u, v at every INTERIOR cell from ψ.  Wall cells are
 * forced to zero (they MUST be no-flow even if Thom-style ω made the
 * surrounding ψ non-zero).  Returns the maximum |v| seen — the
 * caller uses it for adaptive dt and colour normalisation.        */
static inline float recompute_interior_velocity_from_psi(void) {
    int   cols    = g_scene.grid_cols;
    int   rows    = g_scene.grid_rows;
    float inv_2dx = 0.5f / g_scene.cell_size_x;
    float inv_2dy = 0.5f / g_scene.cell_size_y;
    float speed_max = 0.0f;

    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) {
                g_scene.velocity_x[cell_index(x, y)] = 0.0f;
                g_scene.velocity_y[cell_index(x, y)] = 0.0f;
                continue;
            }
            float u_here, v_here;
            velocity_from_psi_at_cell(x, y, inv_2dx, inv_2dy, &u_here, &v_here);
            g_scene.velocity_x[cell_index(x, y)] = u_here;
            g_scene.velocity_y[cell_index(x, y)] = v_here;
            float speed = sqrtf(u_here * u_here + v_here * v_here);
            if (speed > speed_max) speed_max = speed;
        }
    }
    return speed_max;
}

/* Inflow u(y) on the left wall per BoundarySpec — same profile that
 * boundary_set_psi_side embeds in ψ, but we set u directly here so the
 * advection step reads correct velocities even before Thom updates ω. */
static inline float left_inflow_u_at(int y, int rows,
                                      const BoundarySpec *spec) {
    switch (spec->side_left) {
        case BC_SIDE_INFLOW_UNIFORM:
            return spec->inflow_velocity;
        case BC_SIDE_INFLOW_TOP_HALF: {
            int step_h = rows / 4;
            return (y >= step_h) ? spec->inflow_velocity : 0.0f;
        }
        case BC_SIDE_INFLOW_CENTER_BAND: {
            int band_lo = rows * 4 / 10;
            int band_hi = rows * 6 / 10;
            return (y >= band_lo && y <= band_hi) ? spec->inflow_velocity : 0.0f;
        }
        default:
            return 0.0f;
    }
}

/* Top & bottom row velocities — the bottom wall is always no-slip;
 * the top wall is the moving lid (u = inflow_velocity) if scenario
 * is LID_CAVITY, no-slip otherwise. */
static inline void apply_wall_velocity_top_and_bottom(const BoundarySpec *spec,
                                                       int rows, int cols) {
    for (int x = 0; x < cols; x++) {
        g_scene.velocity_x[cell_index(x, 0       )] = 0.0f;
        g_scene.velocity_y[cell_index(x, 0       )] = 0.0f;
        g_scene.velocity_x[cell_index(x, rows - 1)] =
            (spec->side_top == BC_SIDE_WALL_MOVING) ? spec->inflow_velocity : 0.0f;
        g_scene.velocity_y[cell_index(x, rows - 1)] = 0.0f;
    }
}

/* Left & right column velocities — left side picks an inflow profile
 * (or no-slip for walls); right side either no-slip or zero-gradient
 * outflow (copy column N-1 into column N). */
static inline void apply_wall_velocity_left_and_right(const BoundarySpec *spec,
                                                       int rows, int cols) {
    for (int y = 0; y < rows; y++) {
        g_scene.velocity_x[cell_index(0, y)] = left_inflow_u_at(y, rows, spec);
        g_scene.velocity_y[cell_index(0, y)] = 0.0f;
        if (spec->side_right == BC_SIDE_OUTFLOW) {
            g_scene.velocity_x[cell_index(cols - 1, y)] =
                g_scene.velocity_x[cell_index(cols - 2, y)];
            g_scene.velocity_y[cell_index(cols - 1, y)] =
                g_scene.velocity_y[cell_index(cols - 2, y)];
        } else {
            g_scene.velocity_x[cell_index(cols - 1, y)] = 0.0f;
            g_scene.velocity_y[cell_index(cols - 1, y)] = 0.0f;
        }
    }
}

/* Update the EMA of max |v| used to normalise the velocity colour
 * ramp.  α = 0.08 gives ~12-frame time constant — slow enough that
 * brief spikes don't flicker the colour scale, fast enough that
 * the visualisation tracks growing flows.  Clamped to a small ε to
 * avoid divide-by-zero downstream. */
static inline void update_smoothed_velocity_max(float speed_max) {
    if (speed_max < 1e-6f) speed_max = 1e-6f;
    g_scene.velocity_max_smoothed = 0.92f * g_scene.velocity_max_smoothed
                                  + 0.08f * speed_max;
    if (g_scene.velocity_max_smoothed < 1e-6f)
        g_scene.velocity_max_smoothed = 1e-6f;
}

/* CFL-derived adaptive timestep:
 *   dt_advection = α · h / |v|_max         (Courant condition)
 *   dt_diffusion = α · h² / (4·ν)          (von Neumann condition)
 *   dt           = min(dt_adv, dt_diff)    (whichever is tighter)
 * Both bounds [4] LeVeque §3-4.  The α is a safety factor < 1. */
static inline void update_adaptive_dt(float speed_max) {
    float h          = g_scene.cell_size_x;
    float dt_adv     = CFL_SAFETY_FACTOR * h / speed_max;
    float dt_diff    = CFL_SAFETY_FACTOR * h * h
                     / (4.0f * g_scene.kinematic_viscosity);
    float dt         = (dt_adv < dt_diff) ? dt_adv : dt_diff;
    if (dt < 1e-6f)  dt = 1e-6f;
    if (dt > 0.01f)  dt = 0.01f;
    g_scene.current_dt = dt;
}

/*
 * velocity_recompute_and_adapt_dt — derive (u, v) from ψ, set wall
 * velocities, then pick the next stable timestep.
 *
 * Pseudocode:
 *   speed_max = recompute_interior_velocity_from_psi()
 *   apply_wall_velocity_top_and_bottom (spec)
 *   apply_wall_velocity_left_and_right (spec)
 *   update_smoothed_velocity_max(speed_max)   ← EMA for colour scale
 *   update_adaptive_dt          (speed_max)   ← CFL min(adv, diff)
 *
 * Refs [1] Roache §1.4 (velocity-streamfunction relations); [4] LeVeque
 * for the CFL bounds.
 */
static void velocity_recompute_and_adapt_dt(const BoundarySpec *spec) {
    int rows = g_scene.grid_rows;
    int cols = g_scene.grid_cols;

    float speed_max = recompute_interior_velocity_from_psi();
    apply_wall_velocity_top_and_bottom (spec, rows, cols);
    apply_wall_velocity_left_and_right (spec, rows, cols);

    update_smoothed_velocity_max(speed_max);
    update_adaptive_dt          (speed_max);
}

/* ===================================================================== */
/* §13  ns_step — one full physics tick (orchestrator)                   */
/* ===================================================================== */

/* Scan the full ω field for max |ω|, then update the EMA used by the
 * vorticity colour ramp.  α = 0.05 gives ~20-frame time constant —
 * slower than velocity_max_smoothed because ω spikes are sharper and
 * we want the band scale to ignore them. */
static inline void update_smoothed_vorticity_max(void) {
    int   n    = g_scene.grid_cols * g_scene.grid_rows;
    float wmax = 1e-6f;
    for (int i = 0; i < n; i++) {
        float aw = fabsf(g_scene.vorticity_omega[i]);
        if (aw > wmax) wmax = aw;
    }
    g_scene.vorticity_max_smoothed = 0.95f * g_scene.vorticity_max_smoothed
                                   + 0.05f * wmax;
    if (g_scene.vorticity_max_smoothed < 1e-6f)
        g_scene.vorticity_max_smoothed = 1e-6f;
}

/*
 * ns_step — one full Navier-Stokes tick in (ψ, ω) form.
 *
 * Pseudocode:
 *   vorticity_step                    ← ∂ω/∂t = −u·∇ω + ν·∇²ω
 *   apply_boundary                    ← Thom + ψ profiles
 *   poisson_solve_sor                 ← ∇²ψ = −ω
 *   velocity_recompute_and_adapt_dt   ← u = ∇×ψ + CFL dt
 *   apply_boundary                    ← refresh for next tick's reads
 *   update_smoothed_vorticity_max     ← EMA for colour band scale
 *
 * Refs [1] Roache §3.7 for the (ψ, ω) split that lets the pressure
 *   equation be replaced by a Poisson solve for ψ — no pressure
 *   solver needed in incompressible 2-D flow.
 */
static void ns_step(const BoundarySpec *spec) {
    vorticity_step                 (g_scene.current_dt);
    apply_boundary                 (spec);
    poisson_solve_sor              ();
    velocity_recompute_and_adapt_dt(spec);
    apply_boundary                 (spec);
    update_smoothed_vorticity_max  ();
}

/* ===================================================================== */
/* §14  tracers — Lagrangian particle pool + advection (T9)              */
/* ===================================================================== */

/*
 * Tracer — one passive Lagrangian particle that follows v = ∇×ψ.
 *
 * Intent
 *   Vorticity by itself is invisible — the field shows positive /
 *   negative spin but no flow direction.  Tracers MAKE THE FLOW
 *   VISIBLE: each tick they sample (u, v) from the streamfunction
 *   and step in that direction, leaving a visible trace.  The
 *   overlay is OPTIONAL (toggle key 'a') because some users prefer
 *   the unobstructed vorticity field.
 *
 * Why a fixed pool (not a list of live particles)
 *   The pool is sized for TRACER_COUNT_MAX; active count varies via
 *   +/- keys.  No allocation in the hot path; respawned tracers
 *   reuse existing slots.
 *
 * Why fractional pos_x / pos_y (not int)
 *   Tracers move continuously through the grid, so their positions
 *   are real numbers and bilinear-sampled.  Integer positions would
 *   produce visible "stair-stepping" at sub-cell speeds.
 *
 * Why explicit ticks_remaining
 *   Tracers have FINITE LIFETIMES so the pool stays evenly mixed —
 *   without lifetimes, all tracers eventually pile up in the slow
 *   recirculation regions and the fast flow becomes unseen.  At end
 *   of life the tracer respawns at a random grid position.
 */
typedef struct {
    float pos_x;            /* fractional grid x (bilinear-sampled) */
    float pos_y;            /* fractional grid y                    */
    int   ticks_remaining;  /* respawn when this hits 0             */
} Tracer;

static Tracer tracer_pool[TRACER_COUNT_MAX];
static int active_tracer_count = TRACER_COUNT_DEFAULT;
static bool tracer_overlay_enabled = true;

/* Clamp a real-valued sample point into the [0, N-1] grid range so
 * the bilinear stencil never reaches outside. */
static inline void clamp_tracer_sample_in_grid(float *px, float *py) {
    if (*px < 0.0f)                            *px = 0.0f;
    if (*py < 0.0f)                            *py = 0.0f;
    if (*px > (float)(g_scene.grid_cols - 1))  *px = (float)(g_scene.grid_cols - 1);
    if (*py > (float)(g_scene.grid_rows - 1))  *py = (float)(g_scene.grid_rows - 1);
}

/* Split (px, py) into the bottom-left integer cell + fractional
 * offsets ∈ [0, 1).  Right/top neighbours are clamped to the grid
 * edge so a sample exactly on the boundary still gives a valid
 * (degenerate) stencil. */
static inline void integer_cell_and_subcell_2d(float px, float py,
                                                int *out_x0, int *out_y0,
                                                int *out_x1, int *out_y1,
                                                float *out_fx, float *out_fy) {
    int x0 = (int)px;
    int y0 = (int)py;
    *out_x0 = x0;
    *out_y0 = y0;
    *out_x1 = (x0 + 1 < g_scene.grid_cols) ? x0 + 1 : x0;
    *out_y1 = (y0 + 1 < g_scene.grid_rows) ? y0 + 1 : y0;
    *out_fx = px - (float)x0;
    *out_fy = py - (float)y0;
}

/* Two-stage bilinear blend over the 4 corners.  Same formula as
 * elsewhere in this folder — duplicated rather than shared so each
 * file stays self-contained per CLAUDE.md. */
static inline float bilinear_blend(float c00, float c10, float c01, float c11,
                                    float fx, float fy) {
    return (1.0f - fx) * (1.0f - fy) * c00
         +         fx  * (1.0f - fy) * c10
         + (1.0f - fx) *         fy  * c01
         +         fx  *         fy  * c11;
}

/*
 * velocity_at_fractional — bilinear (u, v) sample at sub-cell (px, py).
 *
 * Pseudocode:
 *   clamp_tracer_sample_in_grid(&px, &py)
 *   (x0, y0, x1, y1, fx, fy) = integer_cell_and_subcell_2d(px, py)
 *   *out_u = bilinear_blend(u[x0][y0], u[x1][y0], u[x0][y1], u[x1][y1], fx, fy)
 *   *out_v = bilinear_blend(v[x0][y0], v[x1][y0], v[x0][y1], v[x1][y1], fx, fy)
 */
static void velocity_at_fractional(float px, float py, float *out_u,
                                    float *out_v) {
    clamp_tracer_sample_in_grid(&px, &py);

    int   x0, y0, x1, y1;
    float fx, fy;
    integer_cell_and_subcell_2d(px, py, &x0, &y0, &x1, &y1, &fx, &fy);

    *out_u = bilinear_blend(
        g_scene.velocity_x[cell_index(x0, y0)],
        g_scene.velocity_x[cell_index(x1, y0)],
        g_scene.velocity_x[cell_index(x0, y1)],
        g_scene.velocity_x[cell_index(x1, y1)],
        fx, fy);
    *out_v = bilinear_blend(
        g_scene.velocity_y[cell_index(x0, y0)],
        g_scene.velocity_y[cell_index(x1, y0)],
        g_scene.velocity_y[cell_index(x0, y1)],
        g_scene.velocity_y[cell_index(x1, y1)],
        fx, fy);
}

/* Lid-cavity respawn — closed domain, no inflow.  Uniform random
 * interior so tracers re-enter circulation cells evenly. */
static inline void place_tracer_lid_cavity_interior(Tracer *t) {
    t->pos_x = (float)rand_in_range(1, g_scene.grid_cols - 1);
    t->pos_y = (float)rand_in_range(1, g_scene.grid_rows - 1);
}

/* Backward-step respawn — release near the left inflow on the upper
 * half (above the step face).  Tracers then advect over the step. */
static inline void place_tracer_in_step_inflow(Tracer *t) {
    int step_h = g_scene.grid_rows / 4;
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = (float)step_h
             + rand_uniform_unit() * (float)(g_scene.grid_rows - 1 - step_h);
}

/* Free-jet respawn — release in the narrow inflow BAND on the left.
 * Tracers entrain into the jet and reveal its spreading angle. */
static inline void place_tracer_in_jet_inflow(Tracer *t) {
    int band_lo = g_scene.grid_rows * 4 / 10;
    int band_hi = g_scene.grid_rows * 6 / 10;
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = (float)band_lo
             + rand_uniform_unit() * (float)(band_hi - band_lo);
}

/* Kármán-street respawn — release at full left-wall inflow.  Tracers
 * advect past the cylinder and visualise the alternating vortex
 * shedding downstream. */
static inline void place_tracer_in_karman_inflow(Tracer *t) {
    t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
    t->pos_y = 1.0f + rand_uniform_unit() * (float)(g_scene.grid_rows - 3);
}

/* Assign a random lifetime ∈ [MIN, MAX] ticks.  Spread lifetimes
 * desynchronise respawns so the pool stays evenly mixed instead of
 * all dying on the same frame.  See Tracer struct doc. */
static inline void assign_random_tracer_lifetime(Tracer *t) {
    t->ticks_remaining = TRACER_LIFETIME_MIN
                       + rand_in_range(0, TRACER_LIFETIME_MAX - TRACER_LIFETIME_MIN);
}

/*
 * tracer_spawn — initialise (or respawn) one tracer based on the
 * active scenario.
 *
 * Pseudocode (dispatch on scenario):
 *   SCENARIO_LID_CAVITY    → place_tracer_lid_cavity_interior
 *   SCENARIO_BACKWARD_STEP → place_tracer_in_step_inflow
 *   SCENARIO_FREE_JET      → place_tracer_in_jet_inflow
 *   default (Kármán)       → place_tracer_in_karman_inflow
 *   assign_random_tracer_lifetime
 */
static void tracer_spawn(Tracer *t, int active_scenario_index) {
    switch (active_scenario_index) {
        case SCENARIO_LID_CAVITY:    place_tracer_lid_cavity_interior(t); break;
        case SCENARIO_BACKWARD_STEP: place_tracer_in_step_inflow     (t); break;
        case SCENARIO_FREE_JET:      place_tracer_in_jet_inflow      (t); break;
        default:                     place_tracer_in_karman_inflow   (t); break;
    }
    assign_random_tracer_lifetime(t);
}

static void tracers_init_all(int active_scenario_index) {
  for (int i = 0; i < TRACER_COUNT_MAX; i++)
    tracer_spawn(&tracer_pool[i], active_scenario_index);
}

static bool tracer_position_in_wall(float px, float py) {
  int x = (int)(px + 0.5f);
  int y = (int)(py + 0.5f);
  if (x < 0 || x >= g_scene.grid_cols)
    return false;
  if (y < 0 || y >= g_scene.grid_rows)
    return false;
  return g_scene.wall_mask[cell_index(x, y)];
}

/* Lagrangian advection step: sample velocity at the tracer's
 * fractional position and advance position by v·dt.  The
 * VISIBLE_MOTION_GAIN multiplier compensates for the fact that sim
 * time runs much slower than wall clock — without it, tracers would
 * crawl invisibly even when the simulated flow is fast. */
static inline void advect_one_tracer(Tracer *t, float dt_seconds) {
    const float visible_motion_gain = 6.0f;
    float u_here, v_here;
    velocity_at_fractional(t->pos_x, t->pos_y, &u_here, &v_here);
    t->pos_x += u_here * dt_seconds * visible_motion_gain;
    t->pos_y += v_here * dt_seconds * visible_motion_gain;
}

/* Does the tracer need respawning?  Three reasons:
 *   - out of bounds: drifted into the BC ghost halo
 *   - inside wall:  advected into a solid obstacle
 *   - expired:      lifetime ticks_remaining hit zero
 * Any one is enough to trigger a respawn. */
static inline bool tracer_should_respawn(const Tracer *t) {
    bool out_of_bounds =
        t->pos_x < 0.5f
     || t->pos_x > (float)(g_scene.grid_cols - 2)
     || t->pos_y < 0.5f
     || t->pos_y > (float)(g_scene.grid_rows - 2);
    bool inside_wall = tracer_position_in_wall(t->pos_x, t->pos_y);
    bool expired     = (t->ticks_remaining <= 0);
    return out_of_bounds || inside_wall || expired;
}

/* Full single-tracer update: advect by dt, decrement lifetime, respawn
 * if needed.  Self-contained so the outer loop in tracers_advance
 * stays focused on "which tracers to update". */
static inline void update_one_tracer(Tracer *t, float dt_seconds,
                                      int active_scenario_index) {
    advect_one_tracer(t, dt_seconds);
    t->ticks_remaining--;
    if (tracer_should_respawn(t))
        tracer_spawn(t, active_scenario_index);
}

/*
 * tracers_advance — advance all active tracers one tick (skipped
 * entirely when the overlay is disabled).
 *
 * Pseudocode:
 *   if not tracer_overlay_enabled: return
 *   for i = 0 .. active_tracer_count - 1:
 *     update_one_tracer(&tracer_pool[i], dt, scenario)
 */
static void tracers_advance(int active_scenario_index, float dt_seconds) {
    if (!tracer_overlay_enabled) return;
    for (int i = 0; i < active_tracer_count; i++)
        update_one_tracer(&tracer_pool[i], dt_seconds, active_scenario_index);
}

/* ===================================================================== */
/* §15  presets — 4 scenario specs + loaders                             */
/* ===================================================================== */

/*
 * ScenarioPreset — one named CFD scenario (Karman street / lid cavity etc).
 *
 * Intent
 *   Each preset is a SCENARIO that demonstrates a different fluid-
 *   mechanics phenomenon.  A scenario bundles:
 *     - boundary configuration (which BC kind on each of the 4 walls)
 *     - default Reynolds number (controls expected behaviour)
 *     - obstacle geometry (cylinder for Kármán; step for separation)
 *
 *   Pressing 1..N loads a preset, which writes the BoundarySpec into
 *   the live BC and applies any geometry into the obstacle mask.
 *
 * Why obstacle geometry is bundled here
 *   Different scenarios have different obstacles: the Kármán street
 *   needs a cylinder; backward-facing step needs a step.  Encoding
 *   both as optional flags (has_obstacle_*) lets the preset table
 *   describe the full geometry in one row.
 *
 * Why fractional coordinates (not absolute cells)
 *   `cylinder_x_frac = 0.25` means "1/4 of the way across the grid";
 *   this lets the same preset look proportional on any terminal
 *   size.  Conversion to integer cells happens in the loader.
 *
 * References [4] Ghia 1982 for the lid-cavity benchmark preset;
 *   [5] von Kármán 1911 for the cylinder-wake preset.
 */
typedef struct {
    const char  *display_name;            /* short HUD label            */
    BoundarySpec boundary_spec;           /* per-wall BC configuration  */
    int          default_reynolds_index;  /* index into REYNOLDS_TABLE  */

    /* ── Optional cylinder obstacle ──────────────────────────── */
    bool         has_obstacle_cylinder;
    float        cylinder_x_frac;         /* centre x as fraction       */
    float        cylinder_y_frac;         /* centre y as fraction       */
    float        cylinder_radius_frac;    /* radius as fraction of W    */

    /* ── Optional step obstacle (backward-facing step etc.) ──── */
    bool         has_obstacle_step;
    float        step_height_frac;        /* step height as fraction     */
} ScenarioPreset;

static const ScenarioPreset scenario_table[SCENARIO_COUNT] = {
    /* 0 — KARMAN STREET */
    {"KARMAN STREET",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_UNIFORM,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200 default            */
     true,
     0.25f,
     0.50f,
     0.06f,
     false,
     0.0f},
    /* 1 — LID-DRIVEN CAVITY */
    {"LID CAVITY   ",
     {.side_top = BC_SIDE_WALL_MOVING,
      .side_right = BC_SIDE_WALL_STATIONARY,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_WALL_STATIONARY,
      .inflow_velocity = INFLOW_VELOCITY},
     1, /* Re = 100                    */
     false,
     0,
     0,
     0,
     false,
     0},
    /* 2 — FREE JET */
    {"FREE JET     ",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_CENTER_BAND,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200                    */
     false,
     0,
     0,
     0,
     false,
     0},
    /* 3 — BACKWARD STEP */
    {"BACKWARD STEP",
     {.side_top = BC_SIDE_WALL_STATIONARY,
      .side_right = BC_SIDE_OUTFLOW,
      .side_bottom = BC_SIDE_WALL_STATIONARY,
      .side_left = BC_SIDE_INFLOW_TOP_HALF,
      .inflow_velocity = INFLOW_VELOCITY},
     2, /* Re = 200                    */
     false,
     0,
     0,
     0,
     true,
     0.25f},
};

static int active_scenario_index = SCENARIO_KARMAN;

/* Clamp the scenario index to a valid row and pick the corresponding
 * preset.  Updates the active_scenario_index global so render and
 * HUD code reads the same row. */
static inline const ScenarioPreset *select_scenario_preset(int scenario_index) {
    if (scenario_index < 0 || scenario_index >= SCENARIO_COUNT)
        scenario_index = 0;
    active_scenario_index = scenario_index;
    return &scenario_table[scenario_index];
}

/* Pick Reynolds number from the preset's default tier and derive
 * kinematic viscosity ν = 1/Re.  This sets the DIFFUSIVE damping
 * strength on ω in the vorticity transport equation. */
static inline void apply_scenario_reynolds(const ScenarioPreset *preset) {
    g_scene.reynolds_preset_index = preset->default_reynolds_index;
    g_scene.reynolds_number       =
        reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity   = 1.0f / g_scene.reynolds_number;
}

/* Mask out the obstacle cells for the active scenario.  Cylinder for
 * Kármán-street, backward-facing step for the recirculation scenario,
 * neither for lid-cavity / free-jet.  The masks are read by every
 * subsequent solver pass to skip wall cells. */
static inline void apply_scenario_obstacles(const ScenarioPreset *preset) {
    if (preset->has_obstacle_cylinder)
        obstacle_build_cylinder(preset->cylinder_x_frac,
                                preset->cylinder_y_frac,
                                preset->cylinder_radius_frac);
    if (preset->has_obstacle_step)
        obstacle_build_step    (preset->step_height_frac);
}

/* Seed initial ω = 0, ψ from BC.  apply_boundary sets the inflow ψ
 * profile; velocity_recompute_and_adapt_dt derives the matching (u, v)
 * and picks the first stable dt.  After this call the simulator is
 * ready for ns_step. */
static inline void apply_scenario_initial_state(const ScenarioPreset *preset) {
    apply_boundary                 (&preset->boundary_spec);
    velocity_recompute_and_adapt_dt(&preset->boundary_spec);
}

/* Reset the EMAs used by the colour ramps.  Starting from 1.0 (not 0)
 * avoids divide-by-zero in the first few frames before the actual max
 * values rise. */
static inline void reset_smoothed_maxes(void) {
    g_scene.velocity_max_smoothed  = 1.0f;
    g_scene.vorticity_max_smoothed = 1.0f;
}

/*
 * scenario_load — switch to scenario `scenario_index`, reseeding the
 * full simulation state.
 *
 * Pseudocode:
 *   preset = select_scenario_preset(scenario_index)
 *   grid_clear_walls + grid_zero_all_fields
 *   apply_scenario_reynolds      (preset)     ← Re, ν
 *   apply_scenario_obstacles     (preset)     ← cylinder / step
 *   apply_scenario_initial_state (preset)     ← BC + initial (u, v)
 *   tracers_init_all             (scenario)   ← respawn all tracers
 *   reset_smoothed_maxes()                    ← EMA bootstraps
 */
static void scenario_load(int scenario_index) {
    const ScenarioPreset *preset = select_scenario_preset(scenario_index);

    grid_clear_walls();
    grid_zero_all_fields();

    apply_scenario_reynolds     (preset);
    apply_scenario_obstacles    (preset);
    apply_scenario_initial_state(preset);

    tracers_init_all(active_scenario_index);
    reset_smoothed_maxes();
}

/* ===================================================================== */
/* §16  render_vorticity — ω heatmap                                     */
/* ===================================================================== */

/* Standard draw-rect helper used by every render view.  Returns the
 * visible row count (clipped to grid and HUD-reserved rows).  HUD
 * reserves 2 rows: row 0 (status) and row rows-1 (hint). */
static inline int compute_render_max_y(int term_rows) {
    int draw_rows = term_rows - 2;
    if (draw_rows < 1) draw_rows = 1;
    int rows = g_scene.grid_rows;
    return (rows < draw_rows) ? rows : draw_rows;
}

/* Map a screen row to the grid row, with Y-INVERSION so the top of
 * the screen shows the top of the grid (otherwise gravity-aware
 * scenarios would look upside-down). */
static inline int screen_row_to_grid_y(int sy) {
    return g_scene.grid_rows - 1 - sy;
}

/* Normalised vorticity at cell (x, y) in [-1, 1] for diverging-band
 * lookup.  Divides by the EMA-smoothed max, then clamps. */
static inline float normalised_omega_at(int x, int y) {
    float wn = g_scene.vorticity_omega[cell_index(x, y)]
             / g_scene.vorticity_max_smoothed;
    if (wn >  1.0f) wn =  1.0f;
    if (wn < -1.0f) wn = -1.0f;
    return wn;
}

/* Paint one vorticity cell from its normalised value using the
 * diverging band picker. */
static inline void paint_vorticity_cell(int screen_row, int screen_col,
                                         float omega_normalised) {
    VortBand band = vorticity_band_for(omega_normalised);
    attron(COLOR_PAIR(band.pair) | band.attr);
    mvaddch(screen_row, screen_col, (chtype)(unsigned char)band.glyph);
    attroff(COLOR_PAIR(band.pair) | band.attr);
}

/*
 * render_vorticity_view — ω diverging heatmap (red = CCW, blue = CW).
 *
 * Pseudocode:
 *   max_y = compute_render_max_y(term_rows)
 *   for sy = 0 .. max_y-1:
 *     y = screen_row_to_grid_y(sy)
 *     for each x in [0, min(cols, term_cols)):
 *       paint_vorticity_cell(sy + 1, x, normalised_omega_at(x, y))
 */
static void render_vorticity_view(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);
    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++)
            paint_vorticity_cell(sy + 1, x, normalised_omega_at(x, y));
    }
}

/* ===================================================================== */
/* §17  render_streamlines — banded ψ contours                           */
/* ===================================================================== */

/* Single-pass scan over the whole ψ field for its (min, max) range.
 * The range is what we BIN against to colour streamline cells —
 * banding by absolute ψ values would saturate at high Re. */
static inline void scan_streamfunction_range(float *out_min, float *out_max) {
    int   n   = g_scene.grid_cols * g_scene.grid_rows;
    float lo  =  1e9f;
    float hi  = -1e9f;
    for (int i = 0; i < n; i++) {
        float p = g_scene.streamfunction_psi[i];
        if (p < lo) lo = p;
        if (p > hi) hi = p;
    }
    *out_min = lo;
    *out_max = hi;
}

/* Map a ψ value to a STREAMLINE BAND index ∈ [0, STREAMLINE_BAND_COUNT).
 * Adjacent ψ-levels group into the same band, so the resulting pattern
 * looks like CONTOUR LINES — the visual hallmark of streamfunction
 * plots.  Refs [1] Roache §6 on streamline visualisation. */
static inline int psi_to_streamline_band(float psi, float psi_min,
                                          float psi_range) {
    float frac = (psi - psi_min) / psi_range;
    int   band = (int)(frac * (float)STREAMLINE_BAND_COUNT);
    if (band < 0)                         band = 0;
    if (band >= STREAMLINE_BAND_COUNT)    band = STREAMLINE_BAND_COUNT - 1;
    return band;
}

/* Alternate between two contrasting colour pairs based on band PARITY.
 * Gives the alternating-shade contour look that makes streamlines
 * readable even where the field is dense. */
static inline int streamline_pair_for_band(int band) {
    return (band & 1) ? PAIR_VEL_FIRST + 2 : PAIR_VEL_FIRST + 4;
}

/* |v| at one cell.  Local to this file (other helpers in §17/§18 use
 * the same formula but are separate compilation units). */
static inline float speed_magnitude_at(int x, int y) {
    float u = g_scene.velocity_x[cell_index(x, y)];
    float v = g_scene.velocity_y[cell_index(x, y)];
    return sqrtf(u * u + v * v);
}

/* Paint one streamline cell: pick the contour band's colour and the
 * speed-based glyph in one shot. */
static inline void paint_streamline_cell(int screen_row, int screen_col,
                                          int band, int speed_slot) {
    int pair = streamline_pair_for_band(band);
    attron(COLOR_PAIR(pair));
    mvaddch(screen_row, screen_col,
            (chtype)(unsigned char)velocity_glyph_table[speed_slot]);
    attroff(COLOR_PAIR(pair));
}

/*
 * render_streamlines_view — banded ψ contours with speed-shaped glyphs.
 *
 * Pseudocode:
 *   scan_streamfunction_range(&psi_min, &psi_max)
 *   psi_range = max(psi_max − psi_min, 1e-8)
 *   max_y = compute_render_max_y(term_rows)
 *   for each visible cell (sy, x):
 *     skip walls
 *     band       = psi_to_streamline_band(psi, psi_min, psi_range)
 *     speed_slot = speed_to_velocity_slot(speed, max_smoothed)
 *     paint_streamline_cell(sy + 1, x, band, speed_slot)
 *
 * The contour density follows the SPEED visually (slow regions get
 * dotted glyphs, fast regions get dense glyphs) while the colour
 * tracks the band parity — two channels of information per cell.
 */
static void render_streamlines_view(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);

    float psi_min, psi_max;
    scan_streamfunction_range(&psi_min, &psi_max);
    float psi_range = (psi_max - psi_min) > 1e-8f
                    ?  psi_max - psi_min
                    :  1e-8f;

    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (g_scene.wall_mask[cell_index(x, y)]) continue;
            float psi  = g_scene.streamfunction_psi[cell_index(x, y)];
            int   band = psi_to_streamline_band(psi, psi_min, psi_range);
            int   slot = speed_to_velocity_slot(speed_magnitude_at(x, y),
                                                g_scene.velocity_max_smoothed);
            paint_streamline_cell(sy + 1, x, band, slot);
        }
    }
}

/* ===================================================================== */
/* §18  render_velocity — speed magnitude heatmap                        */
/* ===================================================================== */

static void render_velocity_view(int term_rows, int term_cols) {
  int cols = g_scene.grid_cols;
  int rows = g_scene.grid_rows;
  int draw_rows = term_rows - 2;
  if (draw_rows < 1)
    draw_rows = 1;
  int max_y = (rows < draw_rows) ? rows : draw_rows;

  for (int sy = 0; sy < max_y; sy++) {
    int y = rows - 1 - sy;
    if (y < 0)
      continue;
    for (int x = 0; x < cols && x < term_cols; x++) {
      if (g_scene.wall_mask[cell_index(x, y)])
        continue;
      float u = g_scene.velocity_x[cell_index(x, y)];
      float v = g_scene.velocity_y[cell_index(x, y)];
      float speed = sqrtf(u * u + v * v);
      int slot = speed_to_velocity_slot(speed, g_scene.velocity_max_smoothed);
      attr_t attr = COLOR_PAIR(PAIR_VEL_FIRST + slot);
      if (slot >= VEL_RAMP_SIZE - 2)
        attr |= A_BOLD;
      attron(attr);
      mvaddch(sy + 1, x, (chtype)(unsigned char)velocity_glyph_table[slot]);
      attroff(attr);
    }
  }
}

/* ===================================================================== */
/* §19  render_tracers — overlay particles (T9)                          */
/* ===================================================================== */

/* Round a tracer's fractional position to grid cell + map to screen
 * row (with the y-flip).  Returns true if the result is inside both
 * the grid AND the visible field rect AND not a wall cell. */
static inline bool tracer_to_visible_cell(const Tracer *t,
                                           int term_rows, int term_cols,
                                           int *out_gx, int *out_gy,
                                           int *out_screen_row) {
    int gx = (int)(t->pos_x + 0.5f);
    int gy = (int)(t->pos_y + 0.5f);
    if (gx < 0 || gx >= g_scene.grid_cols) return false;
    if (gy < 0 || gy >= g_scene.grid_rows) return false;
    if (g_scene.wall_mask[cell_index(gx, gy)]) return false;

    int sy        = g_scene.grid_rows - 1 - gy;
    int draw_rows = term_rows - 2;
    if (sy < 0 || sy >= draw_rows) return false;
    if (gx >= term_cols)           return false;

    *out_gx         = gx;
    *out_gy         = gy;
    *out_screen_row = sy + 1;  /* +1 because row 0 is the status row */
    return true;
}

/* Pick the glyph SLOT for a tracer from its local speed.  Discrete
 * 4-tier so the tracer visually "speeds up" as it advects faster —
 * an extra channel of information on top of the colour pair. */
static inline int tracer_glyph_slot_for_speed(float speed_normalised) {
    int slot = (int)(speed_normalised * 4.0f);
    if (slot < 0) slot = 0;
    if (slot >= 4) slot = 3;
    return slot;
}

/* Pick a HOT pair (red) for fast tracers, NORMAL pair (white) for
 * slow ones — emphasises where the active flow is. */
static inline int tracer_pair_for_speed(float speed_normalised) {
    return (speed_normalised > 0.6f) ? PAIR_TRACER_FAST : PAIR_TRACER;
}

/* Paint one tracer.  Speed at the tracer's grid cell determines both
 * glyph and colour pair; A_BOLD makes the overlay pop over whatever
 * field view is underneath. */
static inline void paint_one_tracer_at(int screen_row, int gx, int gy) {
    float speed_norm = speed_magnitude_at(gx, gy) / g_scene.velocity_max_smoothed;
    int   slot       = tracer_glyph_slot_for_speed(speed_norm);
    int   pair       = tracer_pair_for_speed     (speed_norm);
    attr_t attr      = COLOR_PAIR(pair) | A_BOLD;
    attron(attr);
    mvaddch(screen_row, gx, (chtype)(unsigned char)tracer_glyph_table[slot]);
    attroff(attr);
}

/*
 * render_tracer_overlay — paint each tracer as a glyph on top of the
 * current field view.
 *
 * Pseudocode:
 *   if not tracer_overlay_enabled: return
 *   for each tracer:
 *     if not tracer_to_visible_cell(...) : skip
 *     paint_one_tracer_at(screen_row, gx, gy)
 */
static void render_tracer_overlay(int term_rows, int term_cols) {
    if (!tracer_overlay_enabled) return;
    for (int i = 0; i < active_tracer_count; i++) {
        const Tracer *t = &tracer_pool[i];
        int gx, gy, screen_row;
        if (!tracer_to_visible_cell(t, term_rows, term_cols,
                                     &gx, &gy, &screen_row))
            continue;
        paint_one_tracer_at(screen_row, gx, gy);
    }
}

/* ===================================================================== */
/* §20  render_obstacles — overlay solid cells                           */
/* ===================================================================== */

/* Paint one obstacle cell as '#' bold in the obstacle colour pair.
 * Walls are the ONLY layer that's always drawn at full brightness —
 * makes scene topology unambiguous regardless of which field view
 * is active underneath. */
static inline void paint_obstacle_cell(int screen_row, int screen_col) {
    attron(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
    mvaddch(screen_row, screen_col, '#');
    attroff(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
}

/* Paint all obstacle cells in the visible field. */
static inline void paint_all_obstacle_cells(int term_rows, int term_cols) {
    int cols  = g_scene.grid_cols;
    int max_y = compute_render_max_y(term_rows);
    for (int sy = 0; sy < max_y; sy++) {
        int y = screen_row_to_grid_y(sy);
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (!g_scene.wall_mask[cell_index(x, y)]) continue;
            paint_obstacle_cell(sy + 1, x);
        }
    }
}

/* Draw a horizontal bar of '=' across the top of the field if the
 * active scenario has a MOVING LID (lid-driven cavity).  Visualises
 * the wall-shear-driving boundary condition that's otherwise
 * invisible — the field underneath just shows the resulting flow. */
static inline void paint_moving_lid_indicator_if_active(int term_cols) {
    const ScenarioPreset *preset = &scenario_table[active_scenario_index];
    if (preset->boundary_spec.side_top != BC_SIDE_WALL_MOVING) return;
    int cols = g_scene.grid_cols;
    attron(COLOR_PAIR(PAIR_LID) | A_BOLD);
    for (int x = 0; x < cols && x < term_cols; x++)
        mvaddch(1, x, '=');
    attroff(COLOR_PAIR(PAIR_LID) | A_BOLD);
}

/*
 * render_obstacle_overlay — paint walls + moving-lid indicator.
 *
 * Pseudocode:
 *   paint_all_obstacle_cells              ← '#' bold per wall cell
 *   paint_moving_lid_indicator_if_active  ← '=' bar if lid scenario
 */
static void render_obstacle_overlay(int term_rows, int term_cols) {
    paint_all_obstacle_cells              (term_rows, term_cols);
    paint_moving_lid_indicator_if_active  (term_cols);
}

/* ===================================================================== */
/* §21  hud — top status + bottom hint (CLAUDE.md spec)                  */
/* ===================================================================== */

static void hud_paint_status(int term_cols, double measured_fps,
                             int active_view_mode, bool simulation_paused) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " VSF-NS  %s  Re=%6.0f  view:%s  tracers:%s  "
           "%4.0f fps  %s ",
           scenario_table[active_scenario_index].display_name,
           (double)g_scene.reynolds_number, view_name_table[active_view_mode],
           tracer_overlay_enabled ? "ON " : "off", measured_fps,
           simulation_paused ? "PAUSED " : "running");
  int slen = (int)strlen(buf);
  int sx = term_cols - slen;
  if (sx < 0)
    sx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0,
           " q:quit  spc:pause  v:vorticity  s:streamlines  w:velocity  "
           "x:tracers  p/P:scene  +/-:Re  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §22  scene — per-frame state + tick wrapper                           */
/* ===================================================================== */

static int active_view_mode = VIEW_VORTICITY;
static bool simulation_paused = false;

static void scene_init(int term_rows, int term_cols) {
  g_scene.grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  g_scene.grid_rows = (term_rows - 2 < GRID_ROWS_MAX) ? term_rows - 2 : GRID_ROWS_MAX;
  if (g_scene.grid_cols < 8)
    g_scene.grid_cols = 8;
  if (g_scene.grid_rows < 8)
    g_scene.grid_rows = 8;
  g_scene.cell_size_x = 1.0f / (float)(g_scene.grid_cols - 1);
  g_scene.cell_size_y = 1.0f / (float)(g_scene.grid_rows - 1);
  scenario_load(SCENARIO_KARMAN);
}

static void scene_resize(int term_rows, int term_cols) {
  int saved = active_scenario_index;
  g_scene.grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  g_scene.grid_rows = (term_rows - 2 < GRID_ROWS_MAX) ? term_rows - 2 : GRID_ROWS_MAX;
  if (g_scene.grid_cols < 8)
    g_scene.grid_cols = 8;
  if (g_scene.grid_rows < 8)
    g_scene.grid_rows = 8;
  g_scene.cell_size_x = 1.0f / (float)(g_scene.grid_cols - 1);
  g_scene.cell_size_y = 1.0f / (float)(g_scene.grid_rows - 1);
  scenario_load(saved);
}

static void scene_tick(void) {
  if (simulation_paused)
    return;
  const BoundarySpec *spec =
      &scenario_table[active_scenario_index].boundary_spec;
  for (int s = 0; s < SUBSTEPS_PER_FRAME; s++)
    ns_step(spec);
  tracers_advance(active_scenario_index, g_scene.current_dt * SUBSTEPS_PER_FRAME);
}

/* ===================================================================== */
/* §23  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we
 * keep only cell dimensions for HUD placement and field clipping.
 *
 * Render pipeline (one frame): erase → paint_field → paint_tracers
 *   → hud_paint_* → wnoutrefresh(stdscr) → doupdate().  Diff-only
 *   writes — no flicker.  See [9] Raymond §11.
 */
typedef struct {
    int rows;   /* terminal height in cells (getmaxyx)             */
    int cols;   /* terminal width  in cells (getmaxyx)             */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
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

static void screen_present_frame(Screen *screen, double measured_fps) {
  erase();

  switch (active_view_mode) {
  case VIEW_VORTICITY:
    render_vorticity_view(screen->rows, screen->cols);
    break;
  case VIEW_STREAMLINES:
    render_streamlines_view(screen->rows, screen->cols);
    break;
  case VIEW_VELOCITY:
    render_velocity_view(screen->rows, screen->cols);
    break;
  }

  render_obstacle_overlay(screen->rows, screen->cols);
  render_tracer_overlay(screen->rows, screen->cols);
  hud_paint_status(screen->cols, measured_fps, active_view_mode,
                   simulation_paused);
  hud_paint_hint(screen->rows);

  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §24  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    simulation_paused = !simulation_paused;
    break;

  case 'v':
  case 'V':
    active_view_mode = VIEW_VORTICITY;
    break;
  case 's':
  case 'S':
    active_view_mode = VIEW_STREAMLINES;
    break;
  case 'w':
  case 'W':
    active_view_mode = VIEW_VELOCITY;
    break;

  case 'x':
  case 'X':
    tracer_overlay_enabled = !tracer_overlay_enabled;
    break;

  case 'p':
    scenario_load((active_scenario_index + 1) % SCENARIO_COUNT);
    break;
  case 'P':
    scenario_load((active_scenario_index + SCENARIO_COUNT - 1) %
                  SCENARIO_COUNT);
    break;

  case '+':
  case '=':
    g_scene.reynolds_preset_index = (g_scene.reynolds_preset_index + 1) % REYNOLDS_PRESET_COUNT;
    g_scene.reynolds_number = reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity = 1.0f / g_scene.reynolds_number;
    break;
  case '-':
    g_scene.reynolds_preset_index =
        (g_scene.reynolds_preset_index + REYNOLDS_PRESET_COUNT - 1) %
        REYNOLDS_PRESET_COUNT;
    g_scene.reynolds_number = reynolds_preset_table[g_scene.reynolds_preset_index];
    g_scene.kinematic_viscosity = 1.0f / g_scene.reynolds_number;
    break;

  case 'r':
  case 'R':
    scenario_load(active_scenario_index);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned)time(NULL));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  Screen screen;
  screen_init(&screen);
  scene_init(screen.rows, screen.cols);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

  while (!g_should_quit) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch)) {
        g_should_quit = 1;
        break;
      }
    }

    /* ── resize ── */
    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_resize(screen.rows, screen.cols);
      prev_frame_ns = clock_now_ns();
    }

    /* ── dt + fps window ── */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── physics + render ── */
    scene_tick();
    screen_present_frame(&screen, measured_fps);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
