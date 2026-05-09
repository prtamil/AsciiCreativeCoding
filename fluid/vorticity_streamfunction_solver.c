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
 *   streamfunction_psi[]         ψ — flat row-major array
 *   vorticity_omega[]            ω — flat row-major array
 *   vorticity_omega_next[]       scratch for forward-Euler step
 *   velocity_x[], velocity_y[]   u, v — derived from ψ each tick
 *   wall_mask[]                  true = solid obstacle / wall cell
 *
 *   grid_cols, grid_rows         active grid dimensions
 *   cell_size_x, cell_size_y     dx, dy
 *   kinematic_viscosity          ν = 1/Re
 *   reynolds_number              Re = U·L/ν
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
 *   Roache, P. J. (1972), "Computational Fluid Dynamics" (Hermosa).
 *     Foundational textbook; the ψ-ω chapter is the canonical
 *     reference for the formulation used here.
 *   Ghia, U., Ghia, K. N., Shin, C. T. (1982), "High-Re Solutions
 *     for Incompressible Flow Using the Navier-Stokes Equations and
 *     a Multigrid Method," J. Comp. Phys. 48 (3): 387-411.  Their
 *     lid-driven cavity benchmark is one of the most cited results
 *     in CFD.
 *   Thom, A. (1933), "The Flow Past Circular Cylinders at Low
 *     Speeds," Proc. R. Soc. A 141: 651-669.  The wall-vorticity
 *     formula in §9 is named after this paper.
 *   von Karman, T. (1911), the original vortex-street analysis.
 *   https://en.wikipedia.org/wiki/Vorticity
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
#  define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config — every constant + enums                                   */
/* ===================================================================== */

/* §1.1 — frame timing. */

#define RENDER_FPS_CAP            30
#define FPS_RECOMPUTE_MS         500

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define RENDER_TICK_NS  (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.2 — grid limits. */

#define GRID_COLS_MAX        200
#define GRID_ROWS_MAX         60

/* §1.3 — physics. */

/* Lid / inflow speed (dimensionless; problem non-dimensionalised so
 * that U = L = 1, hence Re = 1/ν). */
#define INFLOW_VELOCITY      1.0f

/* Reynolds number presets — cycled with '+' / '-' or set per scenario. */
static const float reynolds_preset_table[] = {
    50.0f, 100.0f, 200.0f, 400.0f, 1000.0f
};
#define REYNOLDS_PRESET_COUNT  5

/* SOR over-relaxation factor.  α ∈ (1, 2); ~1.7 robust for our size. */
#define SOR_OMEGA              1.7f

/* SOR sweeps per Poisson solve. */
#define POISSON_SOR_SWEEPS     14

/* CFL safety factor for adaptive dt. */
#define CFL_SAFETY_FACTOR      0.25f

/* Sub-steps per render frame (more = faster physics evolution). */
#define SUBSTEPS_PER_FRAME     8

/* §1.4 — view modes. */

enum {
    VIEW_VORTICITY  = 0,
    VIEW_STREAMLINES,
    VIEW_VELOCITY,
    VIEW_COUNT,
};

static const char *view_name_table[VIEW_COUNT] = {
    "vorticity ", "streamlines", "velocity  "
};

/* §1.5 — boundary side kinds. */

enum {
    BC_SIDE_WALL_STATIONARY = 0,
    BC_SIDE_WALL_MOVING,             /* lid moving at INFLOW_VELOCITY */
    BC_SIDE_INFLOW_UNIFORM,          /* uniform u = INFLOW_VELOCITY  */
    BC_SIDE_INFLOW_TOP_HALF,         /* uniform u for top half       */
    BC_SIDE_INFLOW_CENTER_BAND,      /* uniform u for centre 20%     */
    BC_SIDE_OUTFLOW                  /* zero-gradient                */
};

/* §1.6 — scenario IDs. */

enum {
    SCENARIO_KARMAN          = 0,
    SCENARIO_LID_CAVITY      = 1,
    SCENARIO_FREE_JET        = 2,
    SCENARIO_BACKWARD_STEP   = 3,
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

    PAIR_VEL_FIRST,                  /* +0..+7  velocity ramp */
    PAIR_VEL_LAST = PAIR_VEL_FIRST + 7,

    PAIR_LID,                        /* yellow lid marker     */
    PAIR_OBSTACLE,                   /* solid wall / cylinder */
    PAIR_TRACER,                     /* tracer particles      */
    PAIR_TRACER_FAST,                /* fast tracer particles */
    PAIR_HUD,                        /* yellow + bold         */
    PAIR_HINT,                       /* cyan + bold           */
};

/* §1.8 — tracer particles (T9). */

#define TRACER_COUNT_MAX        500
#define TRACER_COUNT_DEFAULT    400
#define TRACER_LIFETIME_MIN     200    /* ticks before respawn */
#define TRACER_LIFETIME_MAX     400

/* §1.9 — vorticity diverging-band thresholds. */

#define VORT_BAND_STRONG_FRAC   0.60f   /* |ω/ω_max| > 0.60 → strong */
#define VORT_BAND_MID_FRAC      0.25f
#define VORT_BAND_WEAK_FRAC     0.05f

/* §1.10 — velocity ramp glyph count. */

#define VEL_RAMP_SIZE           8

/* §1.11 — streamline contour count. */

#define STREAMLINE_BAND_COUNT   12

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
/* §3  rng — small wrapper for tracer respawn                            */
/* ===================================================================== */

static float rand_uniform_unit(void)
{
    return (float)rand() / (float)RAND_MAX;
}

static int rand_in_range(int lo, int hi_exclusive)
{
    if (hi_exclusive <= lo) return lo;
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
static const char velocity_glyph_table[VEL_RAMP_SIZE] = {
    ' ', '.', ':', '+', 'x', 'X', '#', '@'
};

/* Tracer particle glyph ramp by speed band. */
static const char tracer_glyph_table[4] = { '.', 'o', 'O', '@' };

/* ===================================================================== */
/* §5  colors — pair init + diverging-band picker                        */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

static void colors_init(void)
{
    start_color();
    use_default_colors();
    terminal_has_256_colours = (COLORS >= 256);

    if (terminal_has_256_colours) {
        /* Vorticity diverging — blue (CW spin) → grey → red (CCW). */
        init_pair(PAIR_VORT_NEG_STRONG, 19,  -1);   /* deep navy */
        init_pair(PAIR_VORT_NEG_MID,    27,  -1);   /* mid blue  */
        init_pair(PAIR_VORT_NEG_WEAK,  117,  -1);   /* light blue*/
        init_pair(PAIR_VORT_ZERO,      244,  -1);   /* mid grey  */
        init_pair(PAIR_VORT_POS_WEAK,  217,  -1);   /* salmon    */
        init_pair(PAIR_VORT_POS_MID,   202,  -1);   /* orange    */
        init_pair(PAIR_VORT_POS_STRONG,196,  -1);   /* bright red*/

        /* Velocity ramp — dark blue → cyan → yellow → red. */
        init_pair(PAIR_VEL_FIRST + 0,  19,  -1);
        init_pair(PAIR_VEL_FIRST + 1,  27,  -1);
        init_pair(PAIR_VEL_FIRST + 2,  39,  -1);
        init_pair(PAIR_VEL_FIRST + 3,  51,  -1);
        init_pair(PAIR_VEL_FIRST + 4, 118,  -1);
        init_pair(PAIR_VEL_FIRST + 5, 226,  -1);
        init_pair(PAIR_VEL_FIRST + 6, 208,  -1);
        init_pair(PAIR_VEL_FIRST + 7, 196,  -1);

        init_pair(PAIR_LID,         226, -1);   /* yellow         */
        init_pair(PAIR_OBSTACLE,    245, -1);   /* mid grey       */
        init_pair(PAIR_TRACER,      255, -1);   /* white          */
        init_pair(PAIR_TRACER_FAST, 226, -1);   /* yellow         */
        init_pair(PAIR_HUD,         226, -1);   /* bright yellow  */
        init_pair(PAIR_HINT,         51, -1);   /* bright cyan    */
    } else {
        /* 8-colour fallback. */
        init_pair(PAIR_VORT_NEG_STRONG, COLOR_BLUE,    -1);
        init_pair(PAIR_VORT_NEG_MID,    COLOR_BLUE,    -1);
        init_pair(PAIR_VORT_NEG_WEAK,   COLOR_CYAN,    -1);
        init_pair(PAIR_VORT_ZERO,       COLOR_WHITE,   -1);
        init_pair(PAIR_VORT_POS_WEAK,   COLOR_MAGENTA, -1);
        init_pair(PAIR_VORT_POS_MID,    COLOR_RED,     -1);
        init_pair(PAIR_VORT_POS_STRONG, COLOR_RED,     -1);
        for (int i = 0; i < VEL_RAMP_SIZE; i++)
            init_pair(PAIR_VEL_FIRST + i,
                      (i < 4) ? COLOR_CYAN : COLOR_YELLOW, -1);
        init_pair(PAIR_LID,         COLOR_YELLOW, -1);
        init_pair(PAIR_OBSTACLE,    COLOR_WHITE,  -1);
        init_pair(PAIR_TRACER,      COLOR_WHITE,  -1);
        init_pair(PAIR_TRACER_FAST, COLOR_YELLOW, -1);
        init_pair(PAIR_HUD,         COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,   -1);
    }
}

/* Diverging vorticity-band picker.  Returns (pair, glyph, attr). */
typedef struct {
    int    pair;
    char   glyph;
    attr_t attr;
} VortBand;

static VortBand vorticity_band_for(float omega_normalised)
{
    if (omega_normalised < -VORT_BAND_STRONG_FRAC)
        return (VortBand){ PAIR_VORT_NEG_STRONG, '#', A_BOLD };
    if (omega_normalised < -VORT_BAND_MID_FRAC)
        return (VortBand){ PAIR_VORT_NEG_MID,    'x', A_NORMAL };
    if (omega_normalised < -VORT_BAND_WEAK_FRAC)
        return (VortBand){ PAIR_VORT_NEG_WEAK,   '.', A_NORMAL };
    if (omega_normalised <  VORT_BAND_WEAK_FRAC)
        return (VortBand){ PAIR_VORT_ZERO,       ' ', A_NORMAL };
    if (omega_normalised <  VORT_BAND_MID_FRAC)
        return (VortBand){ PAIR_VORT_POS_WEAK,   '.', A_NORMAL };
    if (omega_normalised <  VORT_BAND_STRONG_FRAC)
        return (VortBand){ PAIR_VORT_POS_MID,    'x', A_NORMAL };
    return     (VortBand){ PAIR_VORT_POS_STRONG, '#', A_BOLD };
}

/* ===================================================================== */
/* §6  ramp — value to glyph-slot helpers                                */
/* ===================================================================== */

static int speed_to_velocity_slot(float speed, float speed_max)
{
    if (speed_max < 1e-6f) return 0;
    int slot = (int)(speed / speed_max * (float)(VEL_RAMP_SIZE - 1) + 0.5f);
    if (slot < 0) slot = 0;
    if (slot >= VEL_RAMP_SIZE) slot = VEL_RAMP_SIZE - 1;
    return slot;
}

/* ===================================================================== */
/* §7  grid_state — psi, omega, omega_next, u, v, walls                  */
/* ===================================================================== */
/*
 * All fields are flat row-major arrays of grid_rows * grid_cols.
 * Index helper: cell_index(x, y) = y * grid_cols + x.
 *
 * GRID coordinates: (x, y) with x ∈ [0, grid_cols-1], y ∈ [0,
 * grid_rows-1].  y = 0 is the BOTTOM of the physics domain.  When
 * rendering, we INVERT y so the visual top of the screen matches
 * the highest y (matches the lid orientation).
 */

#define GRID_TOTAL_CELLS  (GRID_COLS_MAX * GRID_ROWS_MAX)

static float streamfunction_psi      [GRID_TOTAL_CELLS];
static float vorticity_omega         [GRID_TOTAL_CELLS];
static float vorticity_omega_next    [GRID_TOTAL_CELLS];
static float velocity_x              [GRID_TOTAL_CELLS];
static float velocity_y              [GRID_TOTAL_CELLS];
static bool  wall_mask               [GRID_TOTAL_CELLS];

static int   grid_cols = 0;
static int   grid_rows = 0;
static float cell_size_x = 1.0f;
static float cell_size_y = 1.0f;
static float kinematic_viscosity = 1.0f / 100.0f;
static float reynolds_number     = 100.0f;
static int   reynolds_preset_index = 1;
static float current_dt          = 0.001f;
static float velocity_max_smoothed = 1.0f;
static float vorticity_max_smoothed = 1.0f;

static inline int cell_index(int x, int y)
{
    return y * grid_cols + x;
}

static void grid_zero_all_fields(void)
{
    int n = grid_rows * grid_cols;
    memset(streamfunction_psi,   0, sizeof(float) * n);
    memset(vorticity_omega,      0, sizeof(float) * n);
    memset(vorticity_omega_next, 0, sizeof(float) * n);
    memset(velocity_x,           0, sizeof(float) * n);
    memset(velocity_y,           0, sizeof(float) * n);
}

static void grid_clear_walls(void)
{
    int n = grid_rows * grid_cols;
    memset(wall_mask, 0, sizeof(bool) * n);
}

/* ===================================================================== */
/* §8  obstacle_layouts — cylinder, step builders                        */
/* ===================================================================== */

static void obstacle_build_cylinder(float cx_frac, float cy_frac,
                                    float radius_frac)
{
    float cx = cx_frac * (float)(grid_cols - 1);
    float cy = cy_frac * (float)(grid_rows - 1);
    float radius = radius_frac
                 * (float)((grid_cols < grid_rows) ? grid_cols : grid_rows);
    float r_squared = radius * radius;

    for (int y = 0; y < grid_rows; y++) {
        for (int x = 0; x < grid_cols; x++) {
            float dx = (float)x - cx;
            float dy = (float)y - cy;
            if (dx * dx + dy * dy < r_squared)
                wall_mask[cell_index(x, y)] = true;
        }
    }
}

static void obstacle_build_step(float step_height_frac)
{
    int step_height = (int)(step_height_frac * (float)grid_rows);
    int step_length = grid_cols / 8;
    if (step_length < 1) step_length = 1;

    for (int y = 0; y < step_height; y++) {
        for (int x = 0; x < step_length; x++) {
            wall_mask[cell_index(x, y)] = true;
        }
    }
}

/* ===================================================================== */
/* §9  apply_boundary — per-side BC + Thom's formula (T6)                */
/* ===================================================================== */

typedef struct {
    int side_top;
    int side_right;
    int side_bottom;
    int side_left;
    float inflow_velocity;
} BoundarySpec;

/* Apply ψ values on a side based on its kind. */
static void boundary_set_psi_side(int side_kind, char which_side,
                                  float inflow_velocity)
{
    int rows = grid_rows;
    int cols = grid_cols;

    /* For inflow boundaries, ψ varies linearly with the perpendicular
     * coordinate so that ∂ψ/∂(perp) gives uniform u or v. */
    switch (which_side) {
        case 'L': /* left, x = 0; perpendicular is y */
            for (int y = 0; y < rows; y++) {
                float psi_value = 0.0f;
                if (side_kind == BC_SIDE_INFLOW_UNIFORM) {
                    psi_value = inflow_velocity * (float)y * cell_size_y;
                } else if (side_kind == BC_SIDE_INFLOW_TOP_HALF) {
                    int step_h = rows / 4;
                    if (y >= step_h)
                        psi_value = inflow_velocity * (float)(y - step_h)
                                  * cell_size_y;
                } else if (side_kind == BC_SIDE_INFLOW_CENTER_BAND) {
                    int band_lo = rows * 4 / 10;
                    int band_hi = rows * 6 / 10;
                    if (y >= band_lo && y <= band_hi) {
                        psi_value = inflow_velocity * (float)(y - band_lo)
                                  * cell_size_y;
                    } else if (y > band_hi) {
                        psi_value = inflow_velocity
                                  * (float)(band_hi - band_lo)
                                  * cell_size_y;
                    }
                }
                streamfunction_psi[cell_index(0, y)] = psi_value;
            }
            break;
        case 'R': /* right, x = cols-1; outflow → zero gradient */
            if (side_kind == BC_SIDE_OUTFLOW) {
                for (int y = 0; y < rows; y++)
                    streamfunction_psi[cell_index(cols - 1, y)] =
                        streamfunction_psi[cell_index(cols - 2, y)];
            }
            break;
        case 'B': /* bottom, y = 0 */
            for (int x = 0; x < cols; x++)
                streamfunction_psi[cell_index(x, 0)] = 0.0f;
            break;
        case 'T': /* top, y = rows-1 */
            for (int x = 0; x < cols; x++) {
                float psi_top = 0.0f;
                /* For Karman-like flows where left side is uniform
                 * inflow, top wall ψ should match the wall streamline
                 * value: ψ = U_in · H. */
                if (side_kind == BC_SIDE_WALL_STATIONARY) {
                    /* If left side has linear ψ profile, top streamline
                     * is at ψ = U · (rows-1) · dy.  Otherwise zero. */
                    psi_top = inflow_velocity
                            * (float)(rows - 1) * cell_size_y;
                }
                streamfunction_psi[cell_index(x, rows - 1)] = psi_top;
            }
            break;
        default: break;
    }
}

/* Apply ω at walls using Thom's formula (T6). */
static void boundary_set_thom_wall_vorticity(const BoundarySpec *spec)
{
    int rows = grid_rows;
    int cols = grid_cols;
    float dx2 = cell_size_x * cell_size_x;
    float dy2 = cell_size_y * cell_size_y;

    /* Bottom wall. */
    if (spec->side_bottom == BC_SIDE_WALL_STATIONARY) {
        for (int x = 0; x < cols; x++) {
            vorticity_omega[cell_index(x, 0)] =
                -2.0f * streamfunction_psi[cell_index(x, 1)] / dy2;
        }
    }

    /* Top wall (could be moving lid). */
    for (int x = 0; x < cols; x++) {
        float psi_inner_top = streamfunction_psi[cell_index(x, rows - 2)];
        float psi_top       = streamfunction_psi[cell_index(x, rows - 1)];
        float w_thom = -2.0f * (psi_inner_top - psi_top) / dy2;
        if (spec->side_top == BC_SIDE_WALL_MOVING) {
            w_thom -= 2.0f * spec->inflow_velocity / cell_size_y;
        }
        vorticity_omega[cell_index(x, rows - 1)] = w_thom;
    }

    /* Left side. */
    if (spec->side_left == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            vorticity_omega[cell_index(0, y)] =
                -2.0f * streamfunction_psi[cell_index(1, y)] / dx2;
        }
    } else {
        /* Inflow: ω = 0 (uniform incoming flow has no vorticity). */
        for (int y = 0; y < rows; y++)
            vorticity_omega[cell_index(0, y)] = 0.0f;
    }

    /* Right side. */
    if (spec->side_right == BC_SIDE_WALL_STATIONARY) {
        for (int y = 0; y < rows; y++) {
            vorticity_omega[cell_index(cols - 1, y)] =
                -2.0f * streamfunction_psi[cell_index(cols - 2, y)] / dx2;
        }
    } else {
        /* Outflow — zero gradient. */
        for (int y = 0; y < rows; y++)
            vorticity_omega[cell_index(cols - 1, y)] =
                vorticity_omega[cell_index(cols - 2, y)];
    }

    /* Obstacle interior + boundaries: Thom-style.  We use the
     * average of any non-wall neighbour ψ values as the "interior"
     * proxy.  For interior obstacle cells this gives ω = 0
     * effectively; at the obstacle boundary it gives the correct
     * Thom magnitude. */
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (!wall_mask[cell_index(x, y)]) continue;

            float psi_sum = 0.0f;
            int   neighbour_count = 0;
            int neighbours[4][2] = {
                { x - 1, y }, { x + 1, y },
                { x, y - 1 }, { x, y + 1 }
            };
            for (int n = 0; n < 4; n++) {
                int nx = neighbours[n][0];
                int ny = neighbours[n][1];
                if (!wall_mask[cell_index(nx, ny)]) {
                    psi_sum += streamfunction_psi[cell_index(nx, ny)];
                    neighbour_count++;
                }
            }
            if (neighbour_count > 0) {
                float psi_avg = psi_sum / (float)neighbour_count;
                vorticity_omega[cell_index(x, y)] =
                    -2.0f * psi_avg / dy2;
            } else {
                vorticity_omega[cell_index(x, y)] = 0.0f;
            }
            streamfunction_psi[cell_index(x, y)] = 0.0f;
        }
    }
}

static void apply_boundary(const BoundarySpec *spec)
{
    boundary_set_psi_side(spec->side_left,   'L', spec->inflow_velocity);
    boundary_set_psi_side(spec->side_right,  'R', spec->inflow_velocity);
    boundary_set_psi_side(spec->side_bottom, 'B', spec->inflow_velocity);
    boundary_set_psi_side(spec->side_top,    'T', spec->inflow_velocity);
    boundary_set_thom_wall_vorticity(spec);
}

/* ===================================================================== */
/* §10  vorticity_step — explicit Euler with upwind + diffusion (T3)     */
/* ===================================================================== */

static void vorticity_step(float dt_seconds)
{
    int cols = grid_cols;
    int rows = grid_rows;
    float idx2 = 1.0f / (cell_size_x * cell_size_x);
    float idy2 = 1.0f / (cell_size_y * cell_size_y);
    float idx  = 1.0f / cell_size_x;
    float idy  = 1.0f / cell_size_y;
    float nu = kinematic_viscosity;

    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (wall_mask[cell_index(x, y)]) continue;

            float omega_centre = vorticity_omega[cell_index(x,     y)];
            float omega_east   = vorticity_omega[cell_index(x + 1, y)];
            float omega_west   = vorticity_omega[cell_index(x - 1, y)];
            float omega_north  = vorticity_omega[cell_index(x, y + 1)];
            float omega_south  = vorticity_omega[cell_index(x, y - 1)];

            float u_local = velocity_x[cell_index(x, y)];
            float v_local = velocity_y[cell_index(x, y)];

            /* Upwind convection (T3). */
            float dw_dx = (u_local > 0.0f)
                ? (omega_centre - omega_west) * idx
                : (omega_east   - omega_centre) * idx;
            float dw_dy = (v_local > 0.0f)
                ? (omega_centre - omega_south) * idy
                : (omega_north  - omega_centre) * idy;

            /* Central diffusion. */
            float laplacian_omega =
                  (omega_east  - 2.0f * omega_centre + omega_west)  * idx2
                + (omega_north - 2.0f * omega_centre + omega_south) * idy2;

            vorticity_omega_next[cell_index(x, y)] = omega_centre
                + dt_seconds * (-u_local * dw_dx
                              -  v_local * dw_dy
                              +  nu * laplacian_omega);
        }
    }

    /* Copy interior back; boundaries left to apply_boundary. */
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (wall_mask[cell_index(x, y)]) continue;
            vorticity_omega[cell_index(x, y)] =
                vorticity_omega_next[cell_index(x, y)];
        }
    }
}

/* ===================================================================== */
/* §11  poisson_solve — SOR iteration of ∇²ψ = -ω (T4)                   */
/* ===================================================================== */

static void poisson_solve_sor(void)
{
    int cols = grid_cols;
    int rows = grid_rows;
    float dx2 = cell_size_x * cell_size_x;
    float dy2 = cell_size_y * cell_size_y;
    float denominator = 2.0f * (dx2 + dy2);
    float alpha = SOR_OMEGA;

    for (int sweep = 0; sweep < POISSON_SOR_SWEEPS; sweep++) {
        for (int y = 1; y < rows - 1; y++) {
            for (int x = 1; x < cols - 1; x++) {
                if (wall_mask[cell_index(x, y)]) continue;

                float psi_centre = streamfunction_psi[cell_index(x,     y)];
                float psi_east   = streamfunction_psi[cell_index(x + 1, y)];
                float psi_west   = streamfunction_psi[cell_index(x - 1, y)];
                float psi_north  = streamfunction_psi[cell_index(x, y + 1)];
                float psi_south  = streamfunction_psi[cell_index(x, y - 1)];
                float omega_here = vorticity_omega [cell_index(x,     y)];

                float psi_gauss_seidel =
                    (dy2 * (psi_east + psi_west)
                   + dx2 * (psi_north + psi_south)
                   + dx2 * dy2 * omega_here) / denominator;

                streamfunction_psi[cell_index(x, y)] =
                    psi_centre + alpha * (psi_gauss_seidel - psi_centre);
            }
        }
    }
}

/* ===================================================================== */
/* §12  velocity_recompute — derive u, v from ψ + adaptive dt (T5)       */
/* ===================================================================== */

static void velocity_recompute_and_adapt_dt(const BoundarySpec *spec)
{
    int cols = grid_cols;
    int rows = grid_rows;
    float inv_2dx = 0.5f / cell_size_x;
    float inv_2dy = 0.5f / cell_size_y;

    float speed_max = 0.0f;

    /* Interior. */
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (wall_mask[cell_index(x, y)]) {
                velocity_x[cell_index(x, y)] = 0.0f;
                velocity_y[cell_index(x, y)] = 0.0f;
                continue;
            }
            float u_here =  (streamfunction_psi[cell_index(x, y + 1)]
                           - streamfunction_psi[cell_index(x, y - 1)])
                          * inv_2dy;
            float v_here = -(streamfunction_psi[cell_index(x + 1, y)]
                           - streamfunction_psi[cell_index(x - 1, y)])
                          * inv_2dx;
            velocity_x[cell_index(x, y)] = u_here;
            velocity_y[cell_index(x, y)] = v_here;
            float speed = sqrtf(u_here * u_here + v_here * v_here);
            if (speed > speed_max) speed_max = speed;
        }
    }

    /* Walls — explicit. */
    for (int x = 0; x < cols; x++) {
        velocity_x[cell_index(x, 0)]        = 0.0f;
        velocity_y[cell_index(x, 0)]        = 0.0f;
        velocity_x[cell_index(x, rows - 1)] =
            (spec->side_top == BC_SIDE_WALL_MOVING) ? spec->inflow_velocity : 0.0f;
        velocity_y[cell_index(x, rows - 1)] = 0.0f;
    }
    for (int y = 0; y < rows; y++) {
        if (spec->side_left == BC_SIDE_INFLOW_UNIFORM) {
            velocity_x[cell_index(0, y)] = spec->inflow_velocity;
            velocity_y[cell_index(0, y)] = 0.0f;
        } else if (spec->side_left == BC_SIDE_INFLOW_TOP_HALF) {
            int step_h = rows / 4;
            velocity_x[cell_index(0, y)] = (y >= step_h)
                ? spec->inflow_velocity : 0.0f;
            velocity_y[cell_index(0, y)] = 0.0f;
        } else if (spec->side_left == BC_SIDE_INFLOW_CENTER_BAND) {
            int band_lo = rows * 4 / 10;
            int band_hi = rows * 6 / 10;
            velocity_x[cell_index(0, y)] = (y >= band_lo && y <= band_hi)
                ? spec->inflow_velocity : 0.0f;
            velocity_y[cell_index(0, y)] = 0.0f;
        } else {
            velocity_x[cell_index(0, y)] = 0.0f;
            velocity_y[cell_index(0, y)] = 0.0f;
        }
        if (spec->side_right == BC_SIDE_OUTFLOW) {
            velocity_x[cell_index(cols - 1, y)] =
                velocity_x[cell_index(cols - 2, y)];
            velocity_y[cell_index(cols - 1, y)] =
                velocity_y[cell_index(cols - 2, y)];
        } else {
            velocity_x[cell_index(cols - 1, y)] = 0.0f;
            velocity_y[cell_index(cols - 1, y)] = 0.0f;
        }
    }

    /* Smooth velocity_max for stable colour normalisation. */
    if (speed_max < 1e-6f) speed_max = 1e-6f;
    velocity_max_smoothed = 0.92f * velocity_max_smoothed
                          + 0.08f * speed_max;
    if (velocity_max_smoothed < 1e-6f) velocity_max_smoothed = 1e-6f;

    /* Adaptive dt — CFL min of advection + diffusion bounds (T5). */
    float dt_advection = CFL_SAFETY_FACTOR * cell_size_x / speed_max;
    float dt_diffusion = CFL_SAFETY_FACTOR
                       * cell_size_x * cell_size_x
                       / (4.0f * kinematic_viscosity);
    current_dt = (dt_advection < dt_diffusion) ? dt_advection : dt_diffusion;
    if (current_dt < 1e-6f) current_dt = 1e-6f;
    if (current_dt > 0.01f) current_dt = 0.01f;
}

/* ===================================================================== */
/* §13  ns_step — one full physics tick (orchestrator)                   */
/* ===================================================================== */

static void ns_step(const BoundarySpec *spec)
{
    /* 1. Advance vorticity in time. */
    vorticity_step(current_dt);
    /* 2. Apply boundary conditions on ω + ψ. */
    apply_boundary(spec);
    /* 3. Solve Poisson for ψ. */
    poisson_solve_sor();
    /* 4. Recompute u, v from ψ + update adaptive dt. */
    velocity_recompute_and_adapt_dt(spec);
    /* 5. Re-apply BC for the next tick's reads. */
    apply_boundary(spec);

    /* Track smoothed |ω| max for diverging-band normalisation. */
    int n = grid_cols * grid_rows;
    float wmax = 1e-6f;
    for (int i = 0; i < n; i++) {
        float aw = fabsf(vorticity_omega[i]);
        if (aw > wmax) wmax = aw;
    }
    vorticity_max_smoothed = 0.95f * vorticity_max_smoothed
                           + 0.05f * wmax;
    if (vorticity_max_smoothed < 1e-6f) vorticity_max_smoothed = 1e-6f;
}

/* ===================================================================== */
/* §14  tracers — Lagrangian particle pool + advection (T9)              */
/* ===================================================================== */

typedef struct {
    float pos_x;          /* fractional grid coordinate                 */
    float pos_y;
    int   ticks_remaining;/* respawn after lifetime expires             */
} Tracer;

static Tracer  tracer_pool[TRACER_COUNT_MAX];
static int     active_tracer_count = TRACER_COUNT_DEFAULT;
static bool    tracer_overlay_enabled = true;

/* Bilinear interp of velocity at fractional (px, py). */
static void velocity_at_fractional(float px, float py,
                                   float *out_u, float *out_v)
{
    if (px < 0.0f) px = 0.0f;
    if (py < 0.0f) py = 0.0f;
    if (px > (float)(grid_cols - 1)) px = (float)(grid_cols - 1);
    if (py > (float)(grid_rows - 1)) py = (float)(grid_rows - 1);

    int x0 = (int)px;
    int y0 = (int)py;
    int x1 = (x0 + 1 < grid_cols) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < grid_rows) ? y0 + 1 : y0;

    float fx = px - (float)x0;
    float fy = py - (float)y0;

    float u00 = velocity_x[cell_index(x0, y0)];
    float u10 = velocity_x[cell_index(x1, y0)];
    float u01 = velocity_x[cell_index(x0, y1)];
    float u11 = velocity_x[cell_index(x1, y1)];
    float v00 = velocity_y[cell_index(x0, y0)];
    float v10 = velocity_y[cell_index(x1, y0)];
    float v01 = velocity_y[cell_index(x0, y1)];
    float v11 = velocity_y[cell_index(x1, y1)];

    *out_u = (1.0f - fx) * (1.0f - fy) * u00
           + fx          * (1.0f - fy) * u10
           + (1.0f - fx) * fy          * u01
           + fx          * fy          * u11;
    *out_v = (1.0f - fx) * (1.0f - fy) * v00
           + fx          * (1.0f - fy) * v10
           + (1.0f - fx) * fy          * v01
           + fx          * fy          * v11;
}

/* Where to spawn / respawn a tracer based on the active scenario. */
static void tracer_spawn(Tracer *t, int active_scenario_index)
{
    if (active_scenario_index == SCENARIO_LID_CAVITY) {
        /* Closed domain — uniformly random interior. */
        t->pos_x = (float)rand_in_range(1, grid_cols - 1);
        t->pos_y = (float)rand_in_range(1, grid_rows - 1);
    } else if (active_scenario_index == SCENARIO_BACKWARD_STEP) {
        /* Inflow (top half of left wall). */
        int step_h = grid_rows / 4;
        t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
        t->pos_y = (float)step_h + rand_uniform_unit()
                 * (float)(grid_rows - 1 - step_h);
    } else if (active_scenario_index == SCENARIO_FREE_JET) {
        /* Inflow (centre band of left wall). */
        int band_lo = grid_rows * 4 / 10;
        int band_hi = grid_rows * 6 / 10;
        t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
        t->pos_y = (float)band_lo + rand_uniform_unit()
                 * (float)(band_hi - band_lo);
    } else {
        /* Karman — inflow (full left wall). */
        t->pos_x = 1.0f + 2.0f * rand_uniform_unit();
        t->pos_y = 1.0f + rand_uniform_unit()
                 * (float)(grid_rows - 3);
    }
    t->ticks_remaining = TRACER_LIFETIME_MIN
        + rand_in_range(0, TRACER_LIFETIME_MAX - TRACER_LIFETIME_MIN);
}

static void tracers_init_all(int active_scenario_index)
{
    for (int i = 0; i < TRACER_COUNT_MAX; i++)
        tracer_spawn(&tracer_pool[i], active_scenario_index);
}

static bool tracer_position_in_wall(float px, float py)
{
    int x = (int)(px + 0.5f);
    int y = (int)(py + 0.5f);
    if (x < 0 || x >= grid_cols) return false;
    if (y < 0 || y >= grid_rows) return false;
    return wall_mask[cell_index(x, y)];
}

static void tracers_advance(int active_scenario_index, float dt_seconds)
{
    if (!tracer_overlay_enabled) return;

    for (int i = 0; i < active_tracer_count; i++) {
        Tracer *t = &tracer_pool[i];

        float u_here, v_here;
        velocity_at_fractional(t->pos_x, t->pos_y, &u_here, &v_here);

        /* Advect.  Multiplier amplifies tracer motion so they're
         * VISIBLY MOVING at terminal frame rates even when sim time
         * is fractional. */
        const float visible_motion_gain = 6.0f;
        t->pos_x += u_here * dt_seconds * visible_motion_gain;
        t->pos_y += v_here * dt_seconds * visible_motion_gain;

        t->ticks_remaining--;

        bool out_of_bounds = (t->pos_x < 0.5f
                           || t->pos_x > (float)(grid_cols - 2)
                           || t->pos_y < 0.5f
                           || t->pos_y > (float)(grid_rows - 2));
        bool inside_wall = tracer_position_in_wall(t->pos_x, t->pos_y);
        bool expired = (t->ticks_remaining <= 0);

        if (out_of_bounds || inside_wall || expired)
            tracer_spawn(t, active_scenario_index);
    }
}

/* ===================================================================== */
/* §15  presets — 4 scenario specs + loaders                             */
/* ===================================================================== */

typedef struct {
    const char *display_name;
    BoundarySpec boundary_spec;
    int   default_reynolds_index;
    bool  has_obstacle_cylinder;
    float cylinder_x_frac;
    float cylinder_y_frac;
    float cylinder_radius_frac;
    bool  has_obstacle_step;
    float step_height_frac;
} ScenarioPreset;

static const ScenarioPreset scenario_table[SCENARIO_COUNT] = {
    /* 0 — KARMAN STREET */
    {
      "KARMAN STREET",
      { .side_top    = BC_SIDE_WALL_STATIONARY,
        .side_right  = BC_SIDE_OUTFLOW,
        .side_bottom = BC_SIDE_WALL_STATIONARY,
        .side_left   = BC_SIDE_INFLOW_UNIFORM,
        .inflow_velocity = INFLOW_VELOCITY },
      2,                       /* Re = 200 default            */
      true, 0.25f, 0.50f, 0.06f,
      false, 0.0f
    },
    /* 1 — LID-DRIVEN CAVITY */
    {
      "LID CAVITY   ",
      { .side_top    = BC_SIDE_WALL_MOVING,
        .side_right  = BC_SIDE_WALL_STATIONARY,
        .side_bottom = BC_SIDE_WALL_STATIONARY,
        .side_left   = BC_SIDE_WALL_STATIONARY,
        .inflow_velocity = INFLOW_VELOCITY },
      1,                       /* Re = 100                    */
      false, 0, 0, 0, false, 0
    },
    /* 2 — FREE JET */
    {
      "FREE JET     ",
      { .side_top    = BC_SIDE_WALL_STATIONARY,
        .side_right  = BC_SIDE_OUTFLOW,
        .side_bottom = BC_SIDE_WALL_STATIONARY,
        .side_left   = BC_SIDE_INFLOW_CENTER_BAND,
        .inflow_velocity = INFLOW_VELOCITY },
      2,                       /* Re = 200                    */
      false, 0, 0, 0, false, 0
    },
    /* 3 — BACKWARD STEP */
    {
      "BACKWARD STEP",
      { .side_top    = BC_SIDE_WALL_STATIONARY,
        .side_right  = BC_SIDE_OUTFLOW,
        .side_bottom = BC_SIDE_WALL_STATIONARY,
        .side_left   = BC_SIDE_INFLOW_TOP_HALF,
        .inflow_velocity = INFLOW_VELOCITY },
      2,                       /* Re = 200                    */
      false, 0, 0, 0,
      true,  0.25f
    },
};

static int active_scenario_index = SCENARIO_KARMAN;

static void scenario_load(int scenario_index)
{
    if (scenario_index < 0 || scenario_index >= SCENARIO_COUNT)
        scenario_index = 0;
    active_scenario_index = scenario_index;

    const ScenarioPreset *preset = &scenario_table[scenario_index];

    /* Reset all fields. */
    grid_clear_walls();
    grid_zero_all_fields();

    /* Reynolds. */
    reynolds_preset_index = preset->default_reynolds_index;
    reynolds_number       = reynolds_preset_table[reynolds_preset_index];
    kinematic_viscosity   = 1.0f / reynolds_number;

    /* Build obstacles. */
    if (preset->has_obstacle_cylinder)
        obstacle_build_cylinder(preset->cylinder_x_frac,
                                preset->cylinder_y_frac,
                                preset->cylinder_radius_frac);
    if (preset->has_obstacle_step)
        obstacle_build_step(preset->step_height_frac);

    /* Initial fields: zero ω, ψ from BC.  apply_boundary() will set
     * the inflow ψ profile. */
    apply_boundary(&preset->boundary_spec);
    velocity_recompute_and_adapt_dt(&preset->boundary_spec);

    /* Respawn all tracers. */
    tracers_init_all(active_scenario_index);

    /* Reset smoothed maxes. */
    velocity_max_smoothed   = 1.0f;
    vorticity_max_smoothed  = 1.0f;
}

/* ===================================================================== */
/* §16  render_vorticity — ω heatmap                                     */
/* ===================================================================== */

static void render_vorticity_view(int term_rows, int term_cols)
{
    int cols = grid_cols;
    int rows = grid_rows;
    int draw_rows = term_rows - 2;     /* leave row 0 + row rows-1 */
    if (draw_rows < 1) draw_rows = 1;
    int max_y = (rows < draw_rows) ? rows : draw_rows;

    for (int sy = 0; sy < max_y; sy++) {
        int y = rows - 1 - sy;          /* invert: top of screen = top y */
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            float wn = vorticity_omega[cell_index(x, y)]
                     / vorticity_max_smoothed;
            if (wn >  1.0f) wn =  1.0f;
            if (wn < -1.0f) wn = -1.0f;
            VortBand band = vorticity_band_for(wn);
            attron(COLOR_PAIR(band.pair) | band.attr);
            mvaddch(sy + 1, x, (chtype)(unsigned char)band.glyph);
            attroff(COLOR_PAIR(band.pair) | band.attr);
        }
    }
}

/* ===================================================================== */
/* §17  render_streamlines — banded ψ contours                           */
/* ===================================================================== */

static void render_streamlines_view(int term_rows, int term_cols)
{
    int cols = grid_cols;
    int rows = grid_rows;
    int draw_rows = term_rows - 2;
    if (draw_rows < 1) draw_rows = 1;
    int max_y = (rows < draw_rows) ? rows : draw_rows;

    /* Find ψ range. */
    float psi_min =  1e9f;
    float psi_max = -1e9f;
    int n = cols * rows;
    for (int i = 0; i < n; i++) {
        if (streamfunction_psi[i] < psi_min) psi_min = streamfunction_psi[i];
        if (streamfunction_psi[i] > psi_max) psi_max = streamfunction_psi[i];
    }
    float psi_range = (psi_max - psi_min) > 1e-8f
                    ? (psi_max - psi_min) : 1e-8f;

    for (int sy = 0; sy < max_y; sy++) {
        int y = rows - 1 - sy;
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (wall_mask[cell_index(x, y)]) continue;
            float psi = streamfunction_psi[cell_index(x, y)];
            float frac = (psi - psi_min) / psi_range;
            int   band = (int)(frac * (float)STREAMLINE_BAND_COUNT);
            if (band < 0) band = 0;
            if (band >= STREAMLINE_BAND_COUNT) band = STREAMLINE_BAND_COUNT - 1;
            int  pair_index = (band & 1)
                ? PAIR_VEL_FIRST + 2
                : PAIR_VEL_FIRST + 4;
            float speed = sqrtf(velocity_x[cell_index(x, y)]
                              * velocity_x[cell_index(x, y)]
                              + velocity_y[cell_index(x, y)]
                              * velocity_y[cell_index(x, y)]);
            int slot = speed_to_velocity_slot(speed,
                                              velocity_max_smoothed);
            attron(COLOR_PAIR(pair_index));
            mvaddch(sy + 1, x,
                    (chtype)(unsigned char)velocity_glyph_table[slot]);
            attroff(COLOR_PAIR(pair_index));
        }
    }
}

/* ===================================================================== */
/* §18  render_velocity — speed magnitude heatmap                        */
/* ===================================================================== */

static void render_velocity_view(int term_rows, int term_cols)
{
    int cols = grid_cols;
    int rows = grid_rows;
    int draw_rows = term_rows - 2;
    if (draw_rows < 1) draw_rows = 1;
    int max_y = (rows < draw_rows) ? rows : draw_rows;

    for (int sy = 0; sy < max_y; sy++) {
        int y = rows - 1 - sy;
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (wall_mask[cell_index(x, y)]) continue;
            float u = velocity_x[cell_index(x, y)];
            float v = velocity_y[cell_index(x, y)];
            float speed = sqrtf(u * u + v * v);
            int slot = speed_to_velocity_slot(speed,
                                              velocity_max_smoothed);
            attr_t attr = COLOR_PAIR(PAIR_VEL_FIRST + slot);
            if (slot >= VEL_RAMP_SIZE - 2) attr |= A_BOLD;
            attron(attr);
            mvaddch(sy + 1, x,
                    (chtype)(unsigned char)velocity_glyph_table[slot]);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §19  render_tracers — overlay particles (T9)                          */
/* ===================================================================== */

static void render_tracer_overlay(int term_rows, int term_cols)
{
    if (!tracer_overlay_enabled) return;

    int cols = grid_cols;
    int rows = grid_rows;
    int draw_rows = term_rows - 2;

    for (int i = 0; i < active_tracer_count; i++) {
        const Tracer *t = &tracer_pool[i];
        int gx = (int)(t->pos_x + 0.5f);
        int gy = (int)(t->pos_y + 0.5f);
        if (gx < 0 || gx >= cols) continue;
        if (gy < 0 || gy >= rows) continue;
        if (wall_mask[cell_index(gx, gy)]) continue;

        int sy = rows - 1 - gy;
        if (sy < 0 || sy >= draw_rows) continue;
        if (gx >= term_cols) continue;

        float u = velocity_x[cell_index(gx, gy)];
        float v = velocity_y[cell_index(gx, gy)];
        float speed = sqrtf(u * u + v * v);
        float speed_norm = speed / velocity_max_smoothed;
        int   glyph_slot = (int)(speed_norm * 4.0f);
        if (glyph_slot < 0) glyph_slot = 0;
        if (glyph_slot >= 4) glyph_slot = 3;

        int   pair = (speed_norm > 0.6f) ? PAIR_TRACER_FAST : PAIR_TRACER;
        attr_t attr = COLOR_PAIR(pair) | A_BOLD;

        attron(attr);
        mvaddch(sy + 1, gx,
                (chtype)(unsigned char)tracer_glyph_table[glyph_slot]);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §20  render_obstacles — overlay solid cells                           */
/* ===================================================================== */

static void render_obstacle_overlay(int term_rows, int term_cols)
{
    int cols = grid_cols;
    int rows = grid_rows;
    int draw_rows = term_rows - 2;
    if (draw_rows < 1) draw_rows = 1;

    for (int sy = 0; sy < draw_rows && sy < rows; sy++) {
        int y = rows - 1 - sy;
        if (y < 0) continue;
        for (int x = 0; x < cols && x < term_cols; x++) {
            if (!wall_mask[cell_index(x, y)]) continue;
            attron(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
            mvaddch(sy + 1, x, '#');
            attroff(COLOR_PAIR(PAIR_OBSTACLE) | A_BOLD);
        }
    }

    /* Mark moving lid (top row at y = rows-1) for cavity scenario. */
    const ScenarioPreset *preset = &scenario_table[active_scenario_index];
    if (preset->boundary_spec.side_top == BC_SIDE_WALL_MOVING) {
        attron(COLOR_PAIR(PAIR_LID) | A_BOLD);
        for (int x = 0; x < cols && x < term_cols; x++)
            mvaddch(1, x, '=');
        attroff(COLOR_PAIR(PAIR_LID) | A_BOLD);
    }
}

/* ===================================================================== */
/* §21  hud — top status + bottom hint (CLAUDE.md spec)                  */
/* ===================================================================== */

static void hud_paint_status(int term_cols, double measured_fps,
                             int active_view_mode,
                             bool simulation_paused)
{
    char buf[200];
    snprintf(buf, sizeof buf,
             " VSF-NS  %s  Re=%6.0f  view:%s  tracers:%s  "
             "%4.0f fps  %s ",
             scenario_table[active_scenario_index].display_name,
             (double)reynolds_number,
             view_name_table[active_view_mode],
             tracer_overlay_enabled ? "ON " : "off",
             measured_fps,
             simulation_paused ? "PAUSED " : "running");
    int slen = (int)strlen(buf);
    int sx = term_cols - slen;
    if (sx < 0) sx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
             " q:quit  spc:pause  v:vorticity  s:streamlines  w:velocity  "
             "x:tracers  p/P:scene  +/-:Re  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §22  scene — per-frame state + tick wrapper                           */
/* ===================================================================== */

static int  active_view_mode    = VIEW_VORTICITY;
static bool simulation_paused   = false;

static void scene_init(int term_rows, int term_cols)
{
    grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
    grid_rows = (term_rows - 2 < GRID_ROWS_MAX)
              ? term_rows - 2 : GRID_ROWS_MAX;
    if (grid_cols < 8) grid_cols = 8;
    if (grid_rows < 8) grid_rows = 8;
    cell_size_x = 1.0f / (float)(grid_cols - 1);
    cell_size_y = 1.0f / (float)(grid_rows - 1);
    scenario_load(SCENARIO_KARMAN);
}

static void scene_resize(int term_rows, int term_cols)
{
    int saved = active_scenario_index;
    grid_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
    grid_rows = (term_rows - 2 < GRID_ROWS_MAX)
              ? term_rows - 2 : GRID_ROWS_MAX;
    if (grid_cols < 8) grid_cols = 8;
    if (grid_rows < 8) grid_rows = 8;
    cell_size_x = 1.0f / (float)(grid_cols - 1);
    cell_size_y = 1.0f / (float)(grid_rows - 1);
    scenario_load(saved);
}

static void scene_tick(void)
{
    if (simulation_paused) return;
    const BoundarySpec *spec =
        &scenario_table[active_scenario_index].boundary_spec;
    for (int s = 0; s < SUBSTEPS_PER_FRAME; s++)
        ns_step(spec);
    tracers_advance(active_scenario_index, current_dt * SUBSTEPS_PER_FRAME);
}

/* ===================================================================== */
/* §23  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

typedef struct { int rows, cols; } Screen;

static void screen_init(Screen *screen)
{
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

static void screen_present_frame(Screen *screen, double measured_fps)
{
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
    render_tracer_overlay  (screen->rows, screen->cols);
    hud_paint_status(screen->cols, measured_fps, active_view_mode,
                     simulation_paused);
    hud_paint_hint  (screen->rows);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §24  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

static bool app_handle_key(int ch)
{
    switch (ch) {
        case 'q': case 'Q': case 27:
            return false;

        case ' ':
            simulation_paused = !simulation_paused;
            break;

        case 'v': case 'V': active_view_mode = VIEW_VORTICITY;   break;
        case 's': case 'S': active_view_mode = VIEW_STREAMLINES; break;
        case 'w': case 'W': active_view_mode = VIEW_VELOCITY;    break;

        case 'x': case 'X':
            tracer_overlay_enabled = !tracer_overlay_enabled;
            break;

        case 'p':
            scenario_load((active_scenario_index + 1) % SCENARIO_COUNT);
            break;
        case 'P':
            scenario_load((active_scenario_index + SCENARIO_COUNT - 1)
                          % SCENARIO_COUNT);
            break;

        case '+': case '=':
            reynolds_preset_index =
                (reynolds_preset_index + 1) % REYNOLDS_PRESET_COUNT;
            reynolds_number     = reynolds_preset_table[reynolds_preset_index];
            kinematic_viscosity = 1.0f / reynolds_number;
            break;
        case '-':
            reynolds_preset_index =
                (reynolds_preset_index + REYNOLDS_PRESET_COUNT - 1)
                % REYNOLDS_PRESET_COUNT;
            reynolds_number     = reynolds_preset_table[reynolds_preset_index];
            kinematic_viscosity = 1.0f / reynolds_number;
            break;

        case 'r': case 'R':
            scenario_load(active_scenario_index);
            break;

        default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(screen_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    Screen screen;
    screen_init(&screen);
    scene_init(screen.rows, screen.cols);

    int64_t prev_frame_ns       = clock_now_ns();
    int64_t fps_window_ns       = 0;
    int     frames_in_window    = 0;
    double  measured_fps        = 0.0;

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
        int64_t dt_ns  = frame_start_ns - prev_frame_ns;
        prev_frame_ns  = frame_start_ns;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        frames_in_window++;
        fps_window_ns += dt_ns;
        if (fps_window_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
            measured_fps = (double)frames_in_window
                         / ((double)fps_window_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            fps_window_ns    = 0;
        }

        /* ── physics + render ── */
        scene_tick();
        screen_present_frame(&screen, measured_fps);

        /* ── frame cap ── */
        int64_t spent = clock_now_ns() - frame_start_ns;
        if (spent < RENDER_TICK_NS) clock_sleep_ns(RENDER_TICK_NS - spent);
    }

    return 0;
}
