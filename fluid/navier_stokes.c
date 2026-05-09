/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * navier_stokes.c — Stam's "Stable Fluids" (SIGGRAPH 1999), in 1700 lines
 *                   of teaching prose + ~600 lines of code.
 *
 * DEMO: Two counter-rotating EMITTERS at the screen thirds inject dye
 *       and swirling velocity every tick.  Watch the dye plumes meet
 *       in the middle, fold over each other, and develop visible
 *       turbulence.  ARROW KEYS push velocity at the centre; SPACE
 *       drops a dye blob at a random cell; 1/2/3 cycle dye colour
 *       (blue/green/red); +/- adjust viscosity.
 *
 *       The simulation runs INCOMPRESSIBLE NAVIER-STOKES on an 80×80
 *       Eulerian grid using Stam's four-phase OPERATOR SPLITTING:
 *
 *           DIFFUSE  →  PROJECT  →  ADVECT  →  PROJECT     (velocity)
 *           DIFFUSE  →  ADVECT                            (dye)
 *
 *       Each phase is UNCONDITIONALLY STABLE — there is no time-step
 *       limit at which the simulation blows up.  Crank dt as large as
 *       you like; you get blurry but finite output, never NaN.  This
 *       was Stam's killer insight in 1999 and the reason "Stable
 *       Fluids" remained the default real-time fluid solver for two
 *       decades.
 *
 *       Visualisation: the velocity field is invisible (it's just
 *       numbers).  The DYE is a passive scalar that travels with the
 *       flow.  Watching dye = watching velocity.  Glyph ramp from
 *       sparse to dense:  '. : + #'.  Three colour channels.
 *
 *       Numerical safety net: per-frame dye max swings as emitters
 *       pulse; an EXPONENTIAL MOVING AVERAGE smooths the renormaliser
 *       so the SHADE LEVELS don't flicker between frames.  See T9.
 *
 * Study alongside:
 *   fluid/lattice_gas.c          — opposite philosophy: simulate
 *                                   PARTICLES, derive Navier-Stokes
 *                                   implicitly via averaging.
 *   fluid/fluid_sph.c            — Lagrangian PARTICLES with no grid.
 *                                   Read T1 of either to understand
 *                                   the Eulerian/Lagrangian split.
 *   fluid/cfl_stability_explorer.c
 *                                — explores why EXPLICIT schemes blow
 *                                   up; Stam's IMPLICIT scheme
 *                                   sidesteps the CFL bound entirely.
 *   procedural/diffusion/heat_diffusion.c
 *                                — diffusion alone, no advection /
 *                                   pressure.  The simplest cousin.
 *
 * Section map:
 *   §1   config           — every tunable + enum tags
 *   §2   clock            — monotonic ns timer + sleep
 *   §3   rng              — small wrapper for the dye-drop key
 *   §4   themes           — bright dye palettes (blue / green / red)
 *   §5   colors           — pair init + dye_pair_id helper
 *   §6   grid_state       — fields, scratch buffers, all simulation state
 *   §7   boundary_apply   — fill the 1-cell ghost halo
 *   §8   gauss_seidel     — generic linear-system solver (16 sweeps)
 *   §9   diffuse          — implicit diffusion via §8
 *   §10  advect           — semi-Lagrangian back-trace + bilinear lerp
 *   §11  project          — pressure Poisson + gradient subtraction
 *   §12  fluid_step       — one full physics tick (4 vel + 2 dye phases)
 *   §13  sources          — point-source injection helper
 *   §14  emitters         — counter-rotating auto-emitter pair
 *   §15  fluid_reset      — wipe all fields to zero
 *   §16  dye_normaliser   — EMA-smoothed renormaliser (T9 anti-flicker)
 *   §17  glyph_picker     — density → glyph + shade
 *   §18  grid_to_terminal — coordinate map for rendering
 *   §19  render           — paint the dye field
 *   §20  hud              — top status + bottom hint
 *   §21  screen           — ncurses init / cleanup / present
 *   §22  app              — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   p / space        pause / resume
 *   r                reset (wipe all fields, restart emitters)
 *   arrows           push velocity at the screen centre
 *   d                drop dye blob at a random cell
 *   1 / 2 / 3        dye colour: blue / green / red
 *   + / -            viscosity ×2 / ÷2
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/navier_stokes.c -o navier_stokes \
 *       -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Tutorials are a LADDER: each builds on the previous.  Don't
 *      skip ahead to T5 before T2 if it's your first time with
 *      operator splitting.
 *   2. §1 config — calibrates expectations.  Names like
 *      GAUSS_SEIDEL_ITERATIONS = 16 hint at WHY the value was chosen.
 *   3. §6 grid_state — the data structure.  Knowing the FIELDS and
 *      their ROLES illuminates everything below.
 *   4. §12 fluid_step — the SHAPE of the algorithm in eight lines.
 *      Read after T2.
 *   5. §9 diffuse → §10 advect → §11 project — one section per phase.
 *      Read each AFTER the matching tutorial (T3, T4, T5).
 *   6. §16 dye_normaliser → §17 glyph_picker → §19 render — visual
 *      layer.  Read after T7-T9.
 *   7. §22 app — the orchestrator + main loop.  Read last.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   velocity_x, velocity_y                u, v — current velocity
 *   velocity_x_prev, velocity_y_prev      scratch buffers
 *
 *   pressure_correction                   Φ from the Poisson solve
 *                                          (lives in velocity_x_prev
 *                                           during project_step)
 *   divergence_field                      ∇·u, RHS of Poisson
 *                                          (lives in velocity_y_prev)
 *
 *   dye_density, dye_density_prev         ρ (passive scalar) + scratch
 *
 *   cell_index(i, j)                       1-D offset (macro)
 *   GRID_SIDE_INNER                        physics interior side (= N)
 *   GRID_SIDE_TOTAL                        N + 2 (interior + ghost halo)
 *
 *   active_dye_channel                     0 = blue, 1 = green, 2 = red
 *   viscosity_kinematic                    ν (the Navier-Stokes ν)
 *   diffusion_dye                          κ for the passive scalar
 *   simulation_paused                      run/pause toggle
 *   emitter_swirl_phase                    rotation phase ∈ ℝ
 *   dye_max_smoothed                       EMA-tracked renormaliser
 *
 * Background you need
 * ───────────────────
 *   - Vector calculus: divergence ∇·u, gradient ∇p, Laplacian ∇²x
 *     evaluated on a 5-point stencil.
 *   - Bilinear interpolation in 2D.
 *   - Forward vs backward Euler — the explicit/implicit distinction
 *     that motivates the entire method (T3 has a refresher).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Multigrid, conjugate gradient, FFT-based solvers.  We use plain
 *     Gauss-Seidel; 16 sweeps suffices at N=80.
 *   - Free surfaces, multiphase fluid, compressibility, shock waves.
 *   - The Reynolds number / turbulence theory.  We just LET THE FLUID
 *     do its thing on a small grid; turbulent-looking output emerges
 *     for free at low ν.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm     : Jos Stam's "Stable Fluids" (SIGGRAPH 1999).  An
 *                 EULERIAN (grid-based) finite-difference solver for
 *                 the incompressible Navier-Stokes equations.  The
 *                 distinguishing property is UNCONDITIONAL STABILITY:
 *                 each sub-step is provably non-explosive for any
 *                 dt, ν, or grid size.  This trades accuracy for
 *                 robustness — perfect for graphics.
 *
 *                 Per tick (velocity field):
 *                   1. INJECT FORCES (sources, emitters, arrow keys).
 *                   2. DIFFUSE — implicit Gauss-Seidel sweeps on
 *                                (I - dt·ν·∇²) u_new = u_old.
 *                   3. PROJECT — solve ∇²p = ∇·u, subtract ∇p.
 *                   4. ADVECT  — semi-Lagrangian back-trace.
 *                   5. PROJECT — clean up advection's residual
 *                                divergence.
 *
 *                 Per tick (dye / passive scalar):
 *                   6. DIFFUSE.
 *                   7. ADVECT  — by the now-clean velocity.
 *
 *                 The four-phase velocity sequence approximates one
 *                 step of the full NS equation through OPERATOR
 *                 SPLITTING (T2): each phase solves ONE sub-equation
 *                 in isolation, the composition approximates the
 *                 full coupled PDE.
 *
 * Math basis    : Helmholtz decomposition theorem says any vector
 *                 field w can be uniquely written as
 *                       w  =  u  +  ∇p
 *                 where ∇·u = 0 (rotational, divergence-free) and p
 *                 is a scalar (gradient part).  The PROJECT step
 *                 extracts u: solve ∇²p = ∇·w (Poisson), then
 *                 subtract ∇p from w.  After project, ∇·u ≈ 0 — the
 *                 incompressibility constraint is satisfied to the
 *                 Gauss-Seidel residual.
 *
 *                 Implicit diffusion solves the linear system
 *                       (I + a·L) x_new = x_old
 *                 where L is the discrete Laplacian and a = dt·ν·N².
 *                 The matrix is positive-definite for any a > 0, so
 *                 Gauss-Seidel converges geometrically — no CFL bound.
 *
 *                 Semi-Lagrangian advection traces backward by the
 *                 velocity field, then bilinear-interpolates.  Output
 *                 is bounded by input, so values can shrink but never
 *                 explode.  Stable for any dt.
 *
 * Performance   : O(N² · ITERATIONS) per phase.  At N=80 and 16
 *                 iters, ~100K FLOPs per phase, ~600K per tick.  Runs
 *                 at hundreds of fps on any laptop CPU.  Quadratic
 *                 in N; for N > 200 use multigrid.
 *
 * References
 * ──────────
 *   Stam, J. (1999), "Stable Fluids," SIGGRAPH '99 Proc. — the
 *     foundational paper.  Five pages, mostly diagrams.
 *   Stam, J. (2003), "Real-Time Fluid Dynamics for Games," GDC.  The
 *     classroom version, with a 100-line C reference implementation.
 *   Bridson, R. (2008), "Fluid Simulation for Computer Graphics" —
 *     standard textbook; chapters 1-4 cover everything in this file.
 *   Foster, N. & Metaxas, D. (1996), "Realistic Animation of
 *     Liquids."  Pre-Stam grid solver — useful historical contrast.
 *   https://en.wikipedia.org/wiki/Navier%E2%80%93Stokes_equations
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Don't simulate INDIVIDUAL FLUID PARCELS.  Instead, lay down a fixed
 * grid and store one VELOCITY VECTOR per cell.  Evolve the vector
 * field over time using a SEQUENCE OF SIMPLE OPERATIONS, each of
 * which is unconditionally stable in isolation.  The composition is
 * also stable, so the simulation can never blow up — even for
 * absurd time steps.  Trade: accuracy.  Win: robustness.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the fluid as a FOREST OF ARROWS, one arrow per grid cell.
 * Each tick four things happen IN ORDER:
 *
 *   (1) DIFFUSE   each arrow averages with its 4 neighbours
 *                  (smoothing — viscosity)
 *   (2) PROJECT   arrows are STRAIGHTENED so no cell is a source/
 *                  sink (incompressibility)
 *   (3) ADVECT    each arrow LOOKS BACK along itself, finds where
 *                  the fluid CAME FROM, adopts that past arrow
 *   (4) PROJECT   straighten again (advect introduced tiny errors)
 *
 * The DYE field is a SECOND FOREST — one density value per cell.  It
 * doesn't push back on the velocity; it just rides along.  Two
 * phases (diffuse, advect) each tick.  Watching the dye = watching
 * the otherwise-invisible velocity field.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │    forces                                            │
 *      │      │                                               │
 *      │      ▼                                               │
 *      │   ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐  │
 *      │   │ DIFFUSE │ →  │ PROJECT │ →  │ ADVECT  │ →  │ PROJECT │  │
 *      │   └─────────┘    └─────────┘    └─────────┘    └─────────┘  │
 *      │                                                              │
 *      │    each step takes the field IN, returns a new field OUT     │
 *      │    order matters; PROJECT 2 is not redundant                 │
 *      │                                                              │
 *      └──────────────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   GRID INDEXING  (with 1-cell ghost halo around physics interior):
 *     real cells:    1 ≤ i, j ≤ N
 *     ghost cells:   i ∈ {0, N+1}  or  j ∈ {0, N+1}
 *     1-D index:     cell_index(i, j) = i + (N+2) · j
 *
 *   IMPLICIT DIFFUSION (T3) — solve (I + 4a)·x_new = x_old + a·Σ neighbours:
 *     a = dt · ν · N²
 *     for k = 1..ITERATIONS:
 *       for each (i, j):
 *         x_new[i,j] = ( x_old[i,j]
 *                      + a · (x_new[i-1,j] + x_new[i+1,j]
 *                          + x_new[i,j-1] + x_new[i,j+1]) )
 *                      / (1 + 4a)
 *
 *   SEMI-LAGRANGIAN ADVECTION (T4) — backward trace + bilinear lerp:
 *     for each (i, j):
 *       (x_back, y_back) = (i, j) - dt · N · (u[i,j], v[i,j])
 *       (x_back, y_back) = clamp to [0.5, N + 0.5]
 *       new_field[i,j]   = bilinear_lerp(old_field, x_back, y_back)
 *
 *   PROJECT (T5):
 *     1. divergence:    div[i,j] = -h/2 · ((u[i+1,j] - u[i-1,j])
 *                                        + (v[i,j+1] - v[i,j-1]))
 *        where h = 1/N
 *        pressure_correction[i,j] = 0   (initial guess for Gauss-Seidel)
 *     2. solve:         ∇²pressure_correction = div   (16 GS sweeps)
 *     3. correct:       u[i,j] -= 0.5·N · (pc[i+1,j] - pc[i-1,j])
 *                       v[i,j] -= 0.5·N · (pc[i,j+1] - pc[i,j-1])
 *
 *   BOUNDARY APPLY — fill ghost halo so neighbour reads at the edge
 *   stay consistent:
 *     velocity normal to wall: ghost = -interior  (no-slip)
 *     velocity tangential:     ghost = +interior  (smooth)
 *     scalar (mirror):         ghost = +interior  (zero-gradient)
 *
 *   DYE VISUALISATION (T9):
 *     normalise:  d_norm[i,j] = dye_density[i,j] / dye_max_smoothed
 *     glyph:      ramp = " .,+#" indexed by which threshold band
 *     colour:     palette[active_channel][shade_index]
 *     EMA:        dye_max_smoothed = 0.95·old + 0.05·frame_max
 *                  prevents whole-field shade flicker as emitters pulse
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - GHOST HALO is the FIRST THING to forget.  After every step that
 *     modifies a field, call boundary_apply() with the right tag.
 *     Symptom of forgetting: fluid leaks energy at walls; momentum
 *     accumulates; dye disappears off-grid.
 *   - GAUSS-SEIDEL ITERATIONS are tuned for N=80.  At larger grids,
 *     16 sweeps under-converge and the projection is "leaky" —
 *     visible as a non-zero divergence drift.  Bump iterations OR
 *     switch to multigrid.
 *   - SEMI-LAGRANGIAN BACK-TRACE can land outside the grid for fast
 *     cells.  We clamp to [0.5, N+0.5].  Without the clamp, you'd
 *     read out-of-bounds memory or get all-zero advection.
 *   - DYE NORMALISATION: per-frame max swings sharply when emitters
 *     pulse — every visible cell would jump shade levels each frame.
 *     The EMA in §16 smooths the renormaliser to fix this.  See T9.
 *   - DT TRADE-OFF: bigger dt = more numerical diffusion (advection
 *     blurs).  Stam's scheme stays STABLE but loses fine detail.
 *     For sharper plumes, lower dt OR use a higher-order advection
 *     scheme (BFECC, MacCormack).
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Pause and watch a single dye blob: should drift slightly with
 *     the residual velocity, gradually spread (passive diffusion),
 *     and slowly fade (numerical diffusion + boundary loss).
 *   - Resume: the auto-emitters create two counter-rotating dye
 *     plumes.  They mix turbulently in the centre.
 *   - Press '+' a few times (more viscosity): vortices die quickly,
 *     fluid becomes molasses.
 *   - Press '-' a few times (less viscosity): vortices persist
 *     longer; visible Karman streets behind the emitters.
 *   - Arrow keys at centre: dye that's there gets pushed in the
 *     arrow's direction.  Pressure projects the rest.
 *   - 'r' resets: screen blanks; emitters re-fill it over ~30 ticks.
 *   - Switch dye colour with 1/2/3: same field in three palettes.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1  Eulerian fluid solvers — the big picture
 *   T2  Operator splitting — divide and conquer
 *   T3  Implicit diffusion via Gauss-Seidel
 *   T4  Semi-Lagrangian advection — backward trace + lerp
 *   T5  Projection — pressure Poisson via Helmholtz decomposition
 *   T6  Boundaries — ghost halo and the three boundary tags
 *   T7  Dye = passive scalar — making invisible velocity visible
 *   T8  Why "Stable Fluids" — proof sketch of unconditional stability
 *   T9  Visualisation pitfalls — dynamic max + flicker fix
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  EULERIAN FLUID SOLVERS — THE BIG PICTURE
 * ─────────────────────────────────────────────
 * There are two fundamentally different ways to simulate fluid:
 *
 *   LAGRANGIAN — track DISCRETE PARTICLES that move with the flow.
 *                Each particle carries its own velocity, density,
 *                and history.  Examples: SPH (fluid_sph.c), lattice
 *                gas (lattice_gas.c at the bit level), Material
 *                Point Method.
 *
 *   EULERIAN   — lay down a FIXED GRID and store the fluid's
 *                velocity at each cell.  Track how the velocity
 *                FIELD evolves.  Particles, if any, are PASSIVE —
 *                they're advected by the field, they don't ARE the
 *                field.  Examples: weather models, NASA CFD, Stam
 *                (this file), most real-time water in games.
 *
 * Trade-offs:
 *   Eulerian wins on:    pressure / incompressibility (one Poisson
 *                          solve and you're done — T5),
 *                        deterministic compute cost (always O(N²)),
 *                        easy interpolation (values at known cells).
 *   Lagrangian wins on:  free surfaces (where fluid ends, particles
 *                          end), splashes / droplets, mass
 *                          conservation, sparse domains (no compute
 *                          where there's no fluid).
 *
 * Stam's contribution was making the Eulerian solver UNCONDITIONALLY
 * STABLE for any time step.  This was the "gateway drug" that made
 * fluid simulation practical for graphics — production teams pick
 * fast + robust over physically accurate, every time.
 *
 *      ┌───────────────────────────────────────────────────────┐
 *      │                                                       │
 *      │   LAGRANGIAN (SPH, FHP):       EULERIAN (Stam):       │
 *      │                                                       │
 *      │      ●  ↗   ●  ↗   ●            ↗   ↗   ↗            │
 *      │       ●         ●               ↑       ↑            │
 *      │   ↑  ●  ↘    ●                  ↗   →   ↘            │
 *      │      ●      ●                                        │
 *      │   particles MOVE,             grid stays put;        │
 *      │   carry their own data        velocity values        │
 *      │                               flow through cells     │
 *      │                                                       │
 *      └───────────────────────────────────────────────────────┘
 *
 * T2  OPERATOR SPLITTING — DIVIDE AND CONQUER
 * ───────────────────────────────────────────
 * The full incompressible Navier-Stokes is a coupled NON-LINEAR PDE:
 *
 *     ∂u/∂t = -(u·∇)u + ν∇²u - ∇p + f       (momentum)
 *     ∇·u   = 0                              (incompressibility)
 *
 * Solving this DIRECTLY each tick is hard.  Stam's trick: SPLIT the
 * equation into FOUR INDEPENDENT SUB-OPERATORS, each with a clean
 * solver:
 *
 *     (1)  ∂u/∂t = f               ADD FORCES — trivial: u += f·dt
 *     (2)  ∂u/∂t = ν∇²u            DIFFUSE    — implicit GS (T3)
 *     (3)  ∇²p = ∇·u, u -= ∇p      PROJECT    — Poisson + correct (T5)
 *     (4)  ∂u/∂t = -(u·∇)u         ADVECT     — back-trace (T4)
 *
 * Apply them in sequence — each step uses the previous step's output
 * as input.  Stam's order is:  forces → diffuse → project → advect →
 * project.  The second project cleans up advection's residual
 * divergence.
 *
 * The COMPOSITION of stable operators is also stable.  Cost of
 * splitting: first-order accuracy (errors O(dt)).  For graphics this
 * is fine; engineering CFD uses higher-order splitters (Strang etc.).
 *
 * Same architectural pattern as Unix pipes:
 *
 *     u₀ ── diffuse ──> u₁ ── project ──> u₂ ── advect ──> u₃ ── project ──> u₄
 *
 * Each box knows nothing about the others.  Add a new physics step
 * (e.g. surface tension, buoyancy) by inserting a new box in the
 * pipeline.  Decoupled, modular, testable.
 *
 * T3  IMPLICIT DIFFUSION VIA GAUSS-SEIDEL
 * ───────────────────────────────────────
 * The diffusion equation:
 *
 *     ∂x/∂t = ν ∇²x
 *
 * Discretised on a 5-point stencil with grid spacing h:
 *
 *     ∇²x ≈ (x[i-1,j] + x[i+1,j] + x[i,j-1] + x[i,j+1] - 4·x[i,j]) / h²
 *
 * EXPLICIT FORWARD EULER (intuitive, UNSTABLE for big ν·dt):
 *
 *     x_new[i,j] = x_old[i,j] + dt·ν·∇²x_old
 *
 *   Stability bound: ν·dt < h² / 4.  If you violate this, the
 *   scheme oscillates — values get NEGATIVE then explode positive.
 *   At ν = 10⁻⁶ and h = 1/80, you'd need dt < 1.5 · 10⁻⁵.  Tiny.
 *
 * IMPLICIT BACKWARD EULER (less intuitive, UNCONDITIONALLY STABLE):
 *
 *     x_new[i,j] = x_old[i,j] + dt·ν·∇²x_NEW
 *     (I - dt·ν·∇²) x_new = x_old
 *
 *   Each new value depends on its neighbours' NEW values, not old.
 *   This is a linear system A·x = b that we have to SOLVE each tick.
 *   In 2-D, with a = dt·ν·N², the equations rearrange to:
 *
 *     x_new[i,j] · (1 + 4a) - a·(neighbours_new) = x_old[i,j]
 *
 * Solve via row-major Gauss-Seidel sweeps:
 *
 *     for k = 1..ITERATIONS:
 *       for j = 1..N:
 *         for i = 1..N:
 *           x_new[i,j] = (x_old[i,j] + a·(neighbours_new)) / (1 + 4a)
 *       boundary_apply(x_new)
 *
 * Each sweep updates cells in row-major order, using ALREADY-UPDATED
 * left/top neighbours and STALE right/bottom neighbours.  After 16
 * sweeps the residual is ~10⁻⁴ — visually exact at our grid size.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Gauss-Seidel stencil during a sweep:                │
 *      │                                                      │
 *      │           NEW[i,j-1]                                 │
 *      │                │                                     │
 *      │   NEW[i-1,j]──●──OLD[i+1,j]                          │
 *      │                │                                     │
 *      │           OLD[i,j+1]                                 │
 *      │                                                      │
 *      │  After 16 sweeps the new/old distinction blurs out;  │
 *      │  we have approximately solved A·x = b.               │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T4  SEMI-LAGRANGIAN ADVECTION — BACKWARD TRACE + LERP
 * ─────────────────────────────────────────────────────
 * Advection: "the velocity field MOVES the fluid, so the value at
 * each cell at time t+dt was at some OTHER place at time t."  The PDE:
 *
 *     ∂x/∂t + (u·∇)x = 0
 *
 * EXPLICIT FORWARD ADVECTION ("scatter"): for each cell at time t,
 * push its value FORWARD to (i + u·dt, j + v·dt).  Problem: forward
 * push lands on FRACTIONAL grid points; you have to splat to
 * neighbours; fragile, and unstable for big dt.
 *
 * SEMI-LAGRANGIAN BACKWARD TRACE ("gather"): for each cell at time
 * t+dt, ASK "where did this fluid come from at time t?"  Trace
 * backward by -u·dt:
 *
 *     (x_back, y_back) = (i, j) - dt·N·(u[i,j], v[i,j])
 *     new_field[i,j]   = bilinear_lerp(old_field, x_back, y_back)
 *
 * Bilinear interpolation reads the four corners of the (unit) cell
 * containing (x_back, y_back), weights them by fractional position,
 * and returns a smooth value.  Output is BETWEEN min and max of the
 * four corners, so |output| ≤ |corners|.  Hence STABLE.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   t            t + dt                                │
 *      │                                                      │
 *      │   ●────────►   ■                                     │
 *      │  source        cell at t+dt                          │
 *      │   ▲             │                                    │
 *      │   │   "where did the fluid in ■ come from?"          │
 *      │   │                                                  │
 *      │   └─── trace backward by -u·dt → fractional point    │
 *      │                                  inside source cell  │
 *      │                                                      │
 *      │   bilinear-interpolate the OLD field at that point   │
 *      │   to get the NEW value at ■.                         │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Cost: numerical diffusion.  Bilinear lerp is "sticky" — high-
 * frequency content blurs out over many ticks.  Trade: stability for
 * blur.  Modern variants (BFECC, MacCormack) reduce blur at extra
 * compute cost; we use plain bilinear, which Stam's original used.
 *
 * Why MULTIPLY by N inside the trace?  Velocity is in
 * "world-units-per-second" where the domain is [0, 1].  We've
 * discretised the domain into N cells per unit length, so converting
 * to "grid-cells-per-tick" requires the factor of N.
 *
 * T5  PROJECTION — PRESSURE POISSON VIA HELMHOLTZ
 * ───────────────────────────────────────────────
 * After diffusion + advection, the velocity field is no longer
 * divergence-free.  The PROJECT step CORRECTS this — it projects the
 * velocity onto the divergence-free subspace.  This is the step
 * that enforces incompressibility.
 *
 * HELMHOLTZ DECOMPOSITION THEOREM: any vector field w can be
 * uniquely written as
 *
 *     w  =  u  +  ∇p
 *
 * where ∇·u = 0 (divergence-free / "rotational") and p is a scalar
 * (gradient part).  To extract u from w:
 *
 *     1. Take divergence of both sides:
 *          ∇·w = ∇·u + ∇·∇p = 0 + ∇²p = ∇²p
 *        So  ∇²p = ∇·w  — a Poisson equation.
 *
 *     2. Solve the Poisson equation for p.
 *
 *     3. Subtract the gradient:  u = w - ∇p.
 *
 * Step 2 is the same Gauss-Seidel from T3 with different constants
 * (a = 1, c = 4).  Step 1 just computes ∇·w via central differences.
 * Step 3 subtracts ∇p from w, again via central differences.
 *
 * After project, ∇·u ≈ 0 to the GS residual tolerance.  Velocity
 * is now incompressible.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   any vector field w           write as              │
 *      │      ↗   ↑   ↘                                       │
 *      │      →   ●   ←        =     u  (rotational)          │
 *      │      ↘   ↓   ↗                  ↗  ↑  ↘             │
 *      │                                 →  ●  ←              │
 *      │   sources / sinks ALLOWED       ↘  ↓  ↗             │
 *      │                                                      │
 *      │                              + ∇p (gradient,         │
 *      │                                  pure radial)        │
 *      │                                  →   ←               │
 *      │                                  ↑   ↓               │
 *      │                                                      │
 *      │   PROJECT extracts u: solve ∇²p = ∇·w, subtract ∇p.  │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Why the -h/2 SCALING in step 1?  It's chosen so that the
 * correction in step 3 cancels the divergence EXACTLY when GS
 * converges.  The h = 1/N comes from the central-difference
 * derivative.  Trace the algebra; it works out.
 *
 * T6  BOUNDARIES — GHOST HALO AND THE THREE BOUNDARY TAGS
 * ───────────────────────────────────────────────────────
 * The grid has a 1-CELL GHOST HALO around the physics interior:
 *
 *   real cells:    1 ≤ i, j ≤ N
 *   ghost cells:   i ∈ {0, N+1}  or  j ∈ {0, N+1}
 *
 * Ghost cells exist so that the discrete Laplacian and central-
 * difference derivatives can be evaluated AT BOUNDARY CELLS without
 * out-of-bounds reads.  We FILL ghost values such that the desired
 * BOUNDARY CONDITION is automatically satisfied.
 *
 * Three boundary tags:
 *
 *   BOUNDARY_SCALAR        — for dye, pressure, divergence.
 *                              ghost = closest interior cell (mirror).
 *                              Effect: zero-gradient at the wall.
 *                              Mass / dye does NOT flow through.
 *
 *   BOUNDARY_VELOCITY_X    — for u (horizontal velocity).
 *                              At LEFT/RIGHT walls: ghost = -interior
 *                              (no-slip — u at the wall = 0).
 *                              At TOP/BOTTOM walls: ghost = +interior
 *                              (smooth, no jump).
 *
 *   BOUNDARY_VELOCITY_Y    — for v (vertical velocity).
 *                              Mirror of VELOCITY_X by axis swap.
 *
 * The four CORNER ghost cells get the AVERAGE of their two adjacent
 * edge-ghost cells, so the boundary is continuous around the corner.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Ghost halo (g = ghost, # = real interior):          │
 *      │                                                      │
 *      │     g g g g g g g g g g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g # # # # # # # # g                              │
 *      │     g g g g g g g g g g                              │
 *      │                                                      │
 *      │  boundary_apply() fills the g cells based on the     │
 *      │  boundary tag and the interior values.  Called after │
 *      │  every step that modifies a field.                   │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Forgetting boundary_apply is the #1 bug source in Stam-style
 * solvers.  Symptom: fluid leaks energy at walls, momentum slowly
 * accumulates, dye disappears off-grid.
 *
 * T7  DYE = PASSIVE SCALAR — MAKING INVISIBLE VELOCITY VISIBLE
 * ────────────────────────────────────────────────────────────
 * The dye field ρ obeys a SIMPLER equation than velocity:
 *
 *     ∂ρ/∂t + (u·∇)ρ = κ ∇²ρ + s
 *
 * where κ is the dye's diffusion coefficient (small, 10⁻⁴ here)
 * and s is the source term.  Note ρ does NOT appear in the velocity
 * equation — the dye is a PASSIVE TRACER, no back-reaction on flow.
 *
 * Two phases per tick (cf. velocity's four):
 *   (a) DIFFUSE the dye by κ — same Gauss-Seidel as velocity, just a
 *                              different diffusion coefficient.
 *   (b) ADVECT the dye by the (now divergence-free) velocity.
 *
 * No projection — scalars don't have to be divergence-free.  No
 * second project — advect once and we're done.
 *
 * VISUAL PURPOSE:  the velocity field is INVISIBLE (it's just
 * numbers).  The dye MAKES THE VELOCITY VISIBLE by being carried
 * along with it.  Like injecting smoke into a wind tunnel.
 *
 * In real research codes you'd track multiple scalars (heat,
 * salinity, chemical species, smoke).  We track ONE field with
 * THREE COLOUR CHANNELS so the user can see it as blue / green /
 * red — a single bit in the active_dye_channel variable.
 *
 * T8  WHY "STABLE FLUIDS" — UNCONDITIONAL STABILITY
 * ─────────────────────────────────────────────────
 * Each of Stam's four sub-operators is unconditionally stable in
 * isolation:
 *
 *   IMPLICIT DIFFUSION:  the linear system (I + a·L)x = b has a
 *     positive-definite matrix for any a > 0.  Gauss-Seidel
 *     converges.  No CFL bound on dt or ν.
 *
 *   SEMI-LAGRANGIAN ADVECTION:  bilinear lerp is between min and
 *     max of input cells, so |output| ≤ |input|.  Values can shrink
 *     but never explode.  Stable for any dt.
 *
 *   PROJECTION:  the same argument as diffusion — the Poisson
 *     matrix is positive-definite.
 *
 *   FORCES:  trivial; bounded by user input.
 *
 * The COMPOSITION of stable operators is also stable.  Pre-images
 * of bounded sets remain bounded.  Hence the SIMULATION NEVER BLOWS
 * UP.
 *
 * Compare with EXPLICIT FORWARD EULER on diffusion: blows up for
 * dt > h² / (4ν).  At ν = 10⁻⁶ and h = 1/80, that's
 * dt < 1.56 · 10⁻⁵ s.  Our scheme runs at dt = 0.05 s — three
 * thousand times bigger.  THAT'S the win.
 *
 * Price: first-order accuracy + visible numerical diffusion.  For
 * graphics, the trade-off has been winning for 25 years.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Forward Euler (explicit):  blows up if ν·dt > h²/4   │
 *      │                                                      │
 *      │   x ┤●          x ┤●  ●           x ┤●  ●            │
 *      │     │ ●●          │   ●  ●         │   ●     ●       │
 *      │     │   ●●        │      ●●●       │       ●        ●│
 *      │     └─────────    └──────────●     └───────────●     │
 *      │                                                      │
 *      │   small dt:           big dt:        too big:        │
 *      │   converges           oscillates     EXPLODES        │
 *      │                                                      │
 *      │  Stam (implicit + semi-Lagrangian): bounded for any  │
 *      │  dt — values may BLUR but never explode.             │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T9  VISUALISATION PITFALLS — DYNAMIC MAX + FLICKER FIX
 * ──────────────────────────────────────────────────────
 * The dye's absolute density depends on emitter strength, dt, ν, and
 * how long emitters have been running.  Numbers can range from 0.001
 * (sparse trail) to 60+ (emitter cell at peak pulse).  To map this
 * dynamic range to ONLY 4 GLYPH SHADES we have to NORMALISE.
 *
 * NAÏVE APPROACH:  per-frame max:
 *
 *     frame_max = max(dye_density)
 *     for each cell:
 *       d_norm = dye_density[cell] / frame_max
 *       glyph  = ramp[band(d_norm)]
 *
 * FLICKER PROBLEM:  emitter pulses cause frame_max to swing 5-10×
 * between frames.  Take a steady cell with dye = 5.  Last frame
 * max was 10 → cell = 0.5 → ':' glyph.  This frame, an emitter
 * pulse pushed max to 30 → cell = 0.17 → '.' glyph.  EVERY VISIBLE
 * CELL just dropped one shade.  Next frame max relaxes back to
 * 10 → cell = 0.5 → ':' again.  Whole field oscillates.  Visible
 * as full-screen flicker.
 *
 * FIX:  use an EXPONENTIAL MOVING AVERAGE (EMA) of the per-frame
 * max.  The renormaliser changes SLOWLY, so individual cells'
 * normalised values are stable frame-to-frame:
 *
 *     dye_max_smoothed = α · dye_max_smoothed_old + (1-α) · frame_max
 *
 * with α ≈ 0.95.  The smoothed max FOLLOWS the true max with a
 * lag of ~20 frames.  Pulses get smeared out; sustained changes
 * in scale are still tracked.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   per-frame max (jagged):                            │
 *      │     ●●●  ●●●  ●●●●●  ●  ●●●  ●●●  ●●●●●            │
 *      │   ●     ●    ●     ●    ●   ●    ●     ●  ●  ●      │
 *      │                                                      │
 *      │   EMA-smoothed max (gentle):                         │
 *      │           ●●●●●●●●●●●●●●●●●●●●●                     │
 *      │       ●●●                       ●●●●                │
 *      │     ●●                                ●●●           │
 *      │                                                      │
 *      │  Cells normalised by SMOOTHED max stay at the same   │
 *      │  shade frame-to-frame → no flicker.                  │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * This is the same idea as a low-pass filter on a noisy sensor
 * reading.  Anywhere you have to DERIVE A SCALE FROM DATA — colour
 * mapping, audio level meters, exposure auto-adjust in cameras —
 * an EMA on the scale is the simplest cure for shimmer.
 *
 * Implementation in §16 dye_normaliser.
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

/* ===================================================================== */
/* §1  config — every constant + enum tags                               */
/* ===================================================================== */

/* ── GRID SIZE ── */
#define GRID_SIDE_INNER          80       /* "N" — physics interior side */
#define GRID_SIDE_TOTAL          (GRID_SIDE_INNER + 2)
#define GRID_TOTAL_CELLS         (GRID_SIDE_TOTAL * GRID_SIDE_TOTAL)

/* ── PHYSICS ── */
#define DT_DEFAULT               0.05f       /* simulation time step    */
#define DIFFUSION_DYE            0.0001f     /* passive-scalar diffusion */
#define VISCOSITY_INITIAL        0.00001f    /* kinematic viscosity ν   */
#define VISCOSITY_MIN            1e-7f
#define VISCOSITY_MAX            0.1f
#define VISCOSITY_FACTOR         2.0f        /* '+' / '-' multiplier    */

#define GAUSS_SEIDEL_ITERATIONS  16          /* sweeps per linear solve */

/* Forces and dye scales applied inside add_source_at(). */
#define INJECT_FORCE_SCALE       50.0f       /* arrows + emitters       */
#define INJECT_DYE_SCALE         50.0f       /* dye keys + emitters     */

/* Auto-emitter parameters. */
#define EMITTER_SWIRL_INCREMENT  0.04f       /* radians per tick        */
#define EMITTER_FORCE_AMPLITUDE  1.5f
#define EMITTER_DYE_AMPLITUDE    3.0f

/* Pre-warm so the first frame has visible dye. */
#define PREWARM_TICK_COUNT       80

/* ── BOUNDARY-CONDITION TAGS (T6) ── */
enum {
    BOUNDARY_SCALAR     = 0,    /* dye, pressure, divergence — mirror   */
    BOUNDARY_VELOCITY_X = 1,    /* normal flips at L/R walls            */
    BOUNDARY_VELOCITY_Y = 2,    /* normal flips at T/B walls            */
};

/* ── DYE CHANNELS ── */
enum {
    DYE_CHANNEL_BLUE  = 0,
    DYE_CHANNEL_GREEN = 1,
    DYE_CHANNEL_RED   = 2,
    DYE_CHANNEL_COUNT,
};

#define DYE_SHADE_COUNT          4   /* 4 brightness steps per channel  */

/* ── COLOUR PAIRS ── */
enum {
    PAIR_DYE_FIRST = 1,                      /* +0..+(3*4-1)            */
    PAIR_HUD       = PAIR_DYE_FIRST + DYE_CHANNEL_COUNT * DYE_SHADE_COUNT,
    PAIR_HINT,
};

/* ── HUD ── */
#define HUD_RESERVED_ROWS_TOP    1
#define HUD_RESERVED_ROWS_BOTTOM 2

/* ── RENDER FRAME RATE ── */
#define RENDER_FPS               30
#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                1000000LL
#define RENDER_TICK_NS           (NS_PER_SEC / RENDER_FPS)

/* ── GLYPH RAMP THRESHOLDS (in normalised dye space) ── */
#define DENSITY_GLYPH_BLANK      0.02f
#define DENSITY_GLYPH_LOW        0.20f
#define DENSITY_GLYPH_MID        0.50f
#define DENSITY_GLYPH_HIGH       0.80f

/* ── BILINEAR-INTERP CLAMP MARGIN ── */
#define INTERP_CLAMP_MARGIN      0.5f

/* ── DYE NORMALISER (T9) ──
 * Per-frame max swings sharply with emitter pulses.  EMA smooths it. */
#define DYE_MAX_EMA_OLD          0.95f
#define DYE_MAX_EMA_NEW          0.05f
#define DYE_MAX_FLOOR            0.001f
#define DYE_MAX_INITIAL          1.0f

/* ── INDEX MACRO ──
 * Flatten (i, j) into 1-D field offset.  Ghost halo:
 *   real:   i, j ∈ [1, N]
 *   ghost:  i, j ∈ {0, N+1}
 */
#define cell_index(i, j)  ((i) + GRID_SIDE_TOTAL * (j))

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
/* §3  rng — small wrapper for the dye-drop key                          */
/* ===================================================================== */
/*
 * Used only by the 'd' / SPACE key to drop a dye blob at a random
 * interior cell.  Physics is otherwise deterministic.
 */
static int rand_inner_cell(void)
{
    return 1 + rand() % GRID_SIDE_INNER;
}

/* ===================================================================== */
/* §4  themes — bright dye palettes                                      */
/* ===================================================================== */
/*
 * Three colour channels × four shades each.  Each ramp is monotone-
 * brightening from shade 0 (dimmest, used for sparse '.' cells) to
 * shade 3 (brightest, '#' cells).
 *
 * IMPORTANT: every shade-0 colour is in the BRIGHT half of the
 * 256-cube (≥ 30) so even sparsely-dyed cells stay visible against
 * a default-black terminal.  See CLAUDE.md "Theme Palette
 * Brightness" rule.
 */

typedef struct {
    short       colour_256[DYE_SHADE_COUNT];
    short       colour_8  [DYE_SHADE_COUNT];
    const char *name;
} DyePalette;

static const DyePalette dye_palette_table[DYE_CHANNEL_COUNT] = {
    /* BLUE — mid blue → bright cyan.  All four shades clearly visible. */
    { {  33,  39,  51,  87 },
      { COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,  COLOR_WHITE },
      "blue"  },

    /* GREEN — green → bright lime. */
    { {  34,  40,  82, 118 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE },
      "green" },

    /* RED — bright red → orange. */
    { { 124, 160, 196, 208 },
      { COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW },
      "red"   },
};

/* ===================================================================== */
/* §5  colors — pair init + dye_pair_id helper                           */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

/* The pair number for (channel, shade).  Depends on enum layout in §1. */
static int dye_pair_id(int channel, int shade)
{
    return PAIR_DYE_FIRST + channel * DYE_SHADE_COUNT + shade;
}

static void colors_init(void)
{
    start_color();
    use_default_colors();
    terminal_has_256_colours = (COLORS >= 256);

    for (int ch = 0; ch < DYE_CHANNEL_COUNT; ch++) {
        const DyePalette *pal = &dye_palette_table[ch];
        for (int sh = 0; sh < DYE_SHADE_COUNT; sh++) {
            short fg = terminal_has_256_colours
                     ? pal->colour_256[sh]
                     : pal->colour_8[sh];
            init_pair((short)dye_pair_id(ch, sh), fg, -1);
        }
    }

    if (terminal_has_256_colours) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §6  grid_state — fields, scratch buffers, all simulation state        */
/* ===================================================================== */
/*
 * Six 1-D arrays of GRID_TOTAL_CELLS floats hold the entire physics
 * state.  Indexed by cell_index(i, j).  The 1-cell ghost halo around
 * the physics interior is included in the array.
 *
 * Two SCRATCH FIELDS exist for each velocity component because Stam's
 * scheme reads the OLD field while writing into the NEW one.  After
 * each phase, role-swap (or memcpy if the same buffer is reused).
 *
 * Awkward but standard: "_prev" doesn't mean "previous tick".  It
 * means "the OTHER buffer in the current pair".  During project_step,
 * the same _prev buffers double as pressure_correction and
 * divergence_field.  See §11 comments.
 *
 *   velocity_x, velocity_y                    u, v
 *   velocity_x_prev, velocity_y_prev          scratch
 *   dye_density, dye_density_prev             ρ + scratch
 *
 * Plus the run-time tunables (viscosity, theme, paused, etc.) and
 * the EMA smoothed dye max (T9).
 */

static float velocity_x       [GRID_TOTAL_CELLS];
static float velocity_y       [GRID_TOTAL_CELLS];
static float velocity_x_prev  [GRID_TOTAL_CELLS];
static float velocity_y_prev  [GRID_TOTAL_CELLS];
static float dye_density      [GRID_TOTAL_CELLS];
static float dye_density_prev [GRID_TOTAL_CELLS];

static float viscosity_kinematic  = VISCOSITY_INITIAL;
static int   active_dye_channel   = DYE_CHANNEL_BLUE;
static bool  simulation_paused    = false;
static float emitter_swirl_phase  = 0.0f;
static float dye_max_smoothed     = DYE_MAX_INITIAL;   /* T9 — EMA */

/* ===================================================================== */
/* §7  boundary_apply — fill the 1-cell ghost halo                       */
/* ===================================================================== */
/*
 * Called after every step that modifies a field.  The ghost cells are
 * filled so that the desired BOUNDARY CONDITION (T6) is automatically
 * satisfied during the next step's neighbour reads.
 *
 * boundary_kind:
 *   BOUNDARY_SCALAR     — ghost = closest interior cell (mirror).
 *   BOUNDARY_VELOCITY_X — at L/R walls flip sign (no-slip),
 *                          at T/B walls mirror.
 *   BOUNDARY_VELOCITY_Y — symmetric: T/B flip, L/R mirror.
 *
 * Corners get the average of the two adjacent edge ghost cells.
 */
static void boundary_apply(int boundary_kind, float *field)
{
    int N = GRID_SIDE_INNER;

    /* Left + right walls. */
    for (int j = 1; j <= N; j++) {
        float left_inner  = field[cell_index(1, j)];
        float right_inner = field[cell_index(N, j)];
        field[cell_index(0,     j)] =
            (boundary_kind == BOUNDARY_VELOCITY_X) ? -left_inner  : left_inner;
        field[cell_index(N + 1, j)] =
            (boundary_kind == BOUNDARY_VELOCITY_X) ? -right_inner : right_inner;
    }

    /* Top + bottom walls. */
    for (int i = 1; i <= N; i++) {
        float top_inner    = field[cell_index(i, 1)];
        float bottom_inner = field[cell_index(i, N)];
        field[cell_index(i, 0    )] =
            (boundary_kind == BOUNDARY_VELOCITY_Y) ? -top_inner    : top_inner;
        field[cell_index(i, N + 1)] =
            (boundary_kind == BOUNDARY_VELOCITY_Y) ? -bottom_inner : bottom_inner;
    }

    /* Corners — average of the two adjacent edge ghosts. */
    field[cell_index(0,     0)] =
        0.5f * (field[cell_index(1,     0)] + field[cell_index(0,     1)]);
    field[cell_index(0,     N + 1)] =
        0.5f * (field[cell_index(1,     N + 1)] + field[cell_index(0,     N)]);
    field[cell_index(N + 1, 0)] =
        0.5f * (field[cell_index(N,     0)] + field[cell_index(N + 1, 1)]);
    field[cell_index(N + 1, N + 1)] =
        0.5f * (field[cell_index(N,     N + 1)] + field[cell_index(N + 1, N)]);
}

/* ===================================================================== */
/* §8  gauss_seidel — generic linear-system solver                       */
/* ===================================================================== */
/*
 * Solve  c · x[i, j] = b[i, j] + a · (sum of 4 neighbours of x)
 *
 * via 16 row-major Gauss-Seidel sweeps.  Used by both the diffuse
 * step (a = dt·ν·N², c = 1 + 4a) and the project step's Poisson
 * solve (a = 1, c = 4).
 *
 * After each sweep, re-apply the boundary so neighbour reads at the
 * edges remain valid.  Convergence is geometric — residual halves
 * roughly per sweep on a 5-point stencil.
 */
static void gauss_seidel_solve(int boundary_kind,
                               float *x, const float *b,
                               float a, float c)
{
    float inv_c = 1.0f / c;
    int N = GRID_SIDE_INNER;
    for (int sweep = 0; sweep < GAUSS_SEIDEL_ITERATIONS; sweep++) {
        for (int j = 1; j <= N; j++) {
            for (int i = 1; i <= N; i++) {
                float neighbour_sum =
                    x[cell_index(i - 1, j)] + x[cell_index(i + 1, j)] +
                    x[cell_index(i, j - 1)] + x[cell_index(i, j + 1)];
                x[cell_index(i, j)] =
                    (b[cell_index(i, j)] + a * neighbour_sum) * inv_c;
            }
        }
        boundary_apply(boundary_kind, x);
    }
}

/* ===================================================================== */
/* §9  diffuse — implicit diffusion via §8                               */
/* ===================================================================== */
/*
 * Solve (I - dt·ν·∇²) field_new = field_old via Gauss-Seidel.
 *
 *   a = dt · diffusion_coefficient · N²    (the N² comes from 1/h²)
 *   c = 1 + 4a
 *
 * IMPORTANT: warm-start the iterate with the OLD field — the project
 * step in fluid_step() reuses these scratch buffers for divergence /
 * pressure work, so the iterate starts as garbage otherwise.  We
 * memcpy the old field into the iterate before kicking off GS.
 */
static void diffuse(int boundary_kind, float *field_new,
                    const float *field_old,
                    float diffusion_coefficient, float dt)
{
    float a = dt * diffusion_coefficient
            * (float)(GRID_SIDE_INNER * GRID_SIDE_INNER);

    memcpy(field_new, field_old, sizeof(float) * GRID_TOTAL_CELLS);
    gauss_seidel_solve(boundary_kind, field_new, field_old, a, 1.0f + 4.0f * a);
}

/* ===================================================================== */
/* §10  advect — semi-Lagrangian back-trace + bilinear lerp              */
/* ===================================================================== */
/*
 * For each cell (i, j):
 *   1. Trace BACKWARD by dt · N · velocity:
 *        x_back = i - dt · N · vx[i, j]
 *        y_back = j - dt · N · vy[i, j]
 *   2. Clamp (x_back, y_back) to [0.5, N + 0.5] so we stay in-grid.
 *   3. Bilinear-interp the OLD field at (x_back, y_back).
 *   4. Store into the NEW field at (i, j).
 *
 * Bilinear lerp is unconditionally stable: output is a weighted
 * average of four corner values, hence between min and max of those
 * corners.  Values can shrink, never explode.
 */
static void advect(int boundary_kind,
                   float *new_field, const float *old_field,
                   const float *vx, const float *vy, float dt)
{
    int   N        = GRID_SIDE_INNER;
    float dt_cells = dt * (float)N;

    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {

            /* 1. Backward trace. */
            float x_back = (float)i - dt_cells * vx[cell_index(i, j)];
            float y_back = (float)j - dt_cells * vy[cell_index(i, j)];

            /* 2. Clamp. */
            float lo = INTERP_CLAMP_MARGIN;
            float hi = (float)N + INTERP_CLAMP_MARGIN;
            if (x_back < lo) x_back = lo;
            if (x_back > hi) x_back = hi;
            if (y_back < lo) y_back = lo;
            if (y_back > hi) y_back = hi;

            /* 3. Bilinear interp at (x_back, y_back). */
            int   i0 = (int)x_back;
            int   j0 = (int)y_back;
            int   i1 = i0 + 1;
            int   j1 = j0 + 1;
            float s1 = x_back - (float)i0;
            float s0 = 1.0f - s1;
            float t1 = y_back - (float)j0;
            float t0 = 1.0f - t1;

            new_field[cell_index(i, j)] =
                  s0 * (t0 * old_field[cell_index(i0, j0)]
                      + t1 * old_field[cell_index(i0, j1)])
                + s1 * (t0 * old_field[cell_index(i1, j0)]
                      + t1 * old_field[cell_index(i1, j1)]);
        }
    }

    boundary_apply(boundary_kind, new_field);
}

/* ===================================================================== */
/* §11  project — pressure Poisson + gradient subtraction                */
/* ===================================================================== */
/*
 * Project (vx, vy) onto the divergence-free subspace.  Three phases.
 *
 * In code, the SCRATCH BUFFERS are RENAMED for clarity at the top of
 * this function: `pressure_correction` and `divergence_field` are
 * aliases for whichever scratch arrays are passed in.
 *
 * Phase 1 — compute divergence into divergence_field, zero pressure:
 *   div[i, j] = -h/2 · ((vx[i+1, j] - vx[i-1, j])
 *                     + (vy[i, j+1] - vy[i, j-1]))
 *   pressure_correction[i, j] = 0
 *   apply BOUNDARY_SCALAR to both
 *
 * Phase 2 — solve ∇²pressure_correction = divergence_field via
 *   Gauss-Seidel (a = 1, c = 4).
 *
 * Phase 3 — subtract pressure gradient from velocity:
 *   vx[i, j] -= 0.5 · N · (pc[i+1, j] - pc[i-1, j])
 *   vy[i, j] -= 0.5 · N · (pc[i, j+1] - pc[i, j-1])
 *   apply BOUNDARY_VELOCITY_X to vx, BOUNDARY_VELOCITY_Y to vy
 *
 * After phase 3, ∇·u ≈ 0 — incompressibility enforced.
 */
static void project(float *vx, float *vy,
                    float *pressure_correction,
                    float *divergence_field)
{
    int   N = GRID_SIDE_INNER;
    float h = 1.0f / (float)N;

    /* Phase 1: divergence + zero pressure. */
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            divergence_field[cell_index(i, j)] =
                -0.5f * h *
                ( vx[cell_index(i + 1, j)] - vx[cell_index(i - 1, j)]
                + vy[cell_index(i, j + 1)] - vy[cell_index(i, j - 1)] );
            pressure_correction[cell_index(i, j)] = 0.0f;
        }
    }
    boundary_apply(BOUNDARY_SCALAR, divergence_field);
    boundary_apply(BOUNDARY_SCALAR, pressure_correction);

    /* Phase 2: solve ∇²p = div. */
    gauss_seidel_solve(BOUNDARY_SCALAR, pressure_correction,
                       divergence_field, 1.0f, 4.0f);

    /* Phase 3: subtract gradient of pressure_correction from velocity. */
    float n_factor = 0.5f * (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            vx[cell_index(i, j)] -= n_factor
                * (pressure_correction[cell_index(i + 1, j)]
                 - pressure_correction[cell_index(i - 1, j)]);
            vy[cell_index(i, j)] -= n_factor
                * (pressure_correction[cell_index(i, j + 1)]
                 - pressure_correction[cell_index(i, j - 1)]);
        }
    }
    boundary_apply(BOUNDARY_VELOCITY_X, vx);
    boundary_apply(BOUNDARY_VELOCITY_Y, vy);
}

/* ===================================================================== */
/* §12  fluid_step — one full physics tick                               */
/* ===================================================================== */
/*
 * The whole physics in eight calls.
 *
 * VELOCITY phases:
 *   diffuse vx → velocity_x_prev   (using velocity_x as old)
 *   diffuse vy → velocity_y_prev
 *   project    velocity_*_prev     (pressure correction lives in vx, vy)
 *   advect vx  → velocity_x        (advect by velocity_*_prev)
 *   advect vy  → velocity_y
 *   project    velocity_*          (clean residual divergence)
 *
 * DYE phases:
 *   diffuse dye → dye_density_prev
 *   advect  dye → dye_density       (using clean velocity_*)
 *
 * The buffer juggling is awkward but correct.  Stam's reference
 * implementation uses exactly this convention — we keep it.
 */
static void fluid_step(float dt)
{
    /* Velocity diffuse + project. */
    diffuse(BOUNDARY_VELOCITY_X, velocity_x_prev, velocity_x,
            viscosity_kinematic, dt);
    diffuse(BOUNDARY_VELOCITY_Y, velocity_y_prev, velocity_y,
            viscosity_kinematic, dt);
    project(velocity_x_prev, velocity_y_prev,
            velocity_x, velocity_y);

    /* Velocity advect + project. */
    advect(BOUNDARY_VELOCITY_X, velocity_x, velocity_x_prev,
           velocity_x_prev, velocity_y_prev, dt);
    advect(BOUNDARY_VELOCITY_Y, velocity_y, velocity_y_prev,
           velocity_x_prev, velocity_y_prev, dt);
    project(velocity_x, velocity_y,
            velocity_x_prev, velocity_y_prev);

    /* Dye diffuse + advect. */
    diffuse(BOUNDARY_SCALAR, dye_density_prev, dye_density,
            DIFFUSION_DYE, dt);
    advect(BOUNDARY_SCALAR, dye_density, dye_density_prev,
           velocity_x, velocity_y, dt);
}

/* ===================================================================== */
/* §13  sources — point-source injection helper                          */
/* ===================================================================== */
/*
 * Add velocity + dye at one cell.  Used by:
 *   - arrow keys (velocity push at centre)
 *   - SPACE / 'd' (random dye drop)
 *   - the auto-emitters
 *
 * The DT factor is included here so callers specify "force units"
 * and "dye units" rather than "force-per-tick."
 */
static void add_source_at(int i, int j,
                          float force_x, float force_y, float dye_value)
{
    if (i < 1 || i > GRID_SIDE_INNER) return;
    if (j < 1 || j > GRID_SIDE_INNER) return;
    velocity_x [cell_index(i, j)] += DT_DEFAULT * force_x   * INJECT_FORCE_SCALE;
    velocity_y [cell_index(i, j)] += DT_DEFAULT * force_y   * INJECT_FORCE_SCALE;
    dye_density[cell_index(i, j)] += DT_DEFAULT * dye_value * INJECT_DYE_SCALE;
}

/* ===================================================================== */
/* §14  emitters — counter-rotating auto-emitter pair                    */
/* ===================================================================== */
/*
 * Two emitters at 1/3 and 2/3 from the left, both at vertical centre.
 * Each injects dye + a swirling velocity that rotates with
 * emitter_swirl_phase.  The two are 180° out of phase so their
 * resulting flows COUNTER-ROTATE — visually striking, immediately
 * shows off the projection step (without it, the swirls would just
 * bleed into source/sink artefacts).
 */
static void emitters_inject(void)
{
    emitter_swirl_phase += EMITTER_SWIRL_INCREMENT;

    int N = GRID_SIDE_INNER;

    /* Left emitter — clockwise swirl. */
    int   i_left  = N / 3;
    int   j_left  = N / 2;
    float fx_left =  cosf(emitter_swirl_phase) * EMITTER_FORCE_AMPLITUDE;
    float fy_left =  sinf(emitter_swirl_phase) * EMITTER_FORCE_AMPLITUDE;
    add_source_at(i_left, j_left, fx_left, fy_left, EMITTER_DYE_AMPLITUDE);

    /* Right emitter — counter-rotating. */
    int   i_right  = 2 * N / 3;
    int   j_right  = N / 2;
    float fx_right = -cosf(emitter_swirl_phase) * EMITTER_FORCE_AMPLITUDE;
    float fy_right = -sinf(emitter_swirl_phase) * EMITTER_FORCE_AMPLITUDE;
    add_source_at(i_right, j_right, fx_right, fy_right,
                  EMITTER_DYE_AMPLITUDE);
}

/* ===================================================================== */
/* §15  fluid_reset — wipe all fields to zero                            */
/* ===================================================================== */
/*
 * Called at startup, on resize, and on user 'r' key.  Zero out every
 * grid array; the emitters will re-fill the screen over the next ~30
 * ticks.  Reset the EMA tracker so post-reset normalisation starts
 * fresh.
 */
static void fluid_reset(void)
{
    memset(velocity_x,        0, sizeof velocity_x);
    memset(velocity_y,        0, sizeof velocity_y);
    memset(velocity_x_prev,   0, sizeof velocity_x_prev);
    memset(velocity_y_prev,   0, sizeof velocity_y_prev);
    memset(dye_density,       0, sizeof dye_density);
    memset(dye_density_prev,  0, sizeof dye_density_prev);
    emitter_swirl_phase = 0.0f;
    dye_max_smoothed    = DYE_MAX_INITIAL;
}

/* ===================================================================== */
/* §16  dye_normaliser — EMA-smoothed renormaliser (T9 anti-flicker)     */
/* ===================================================================== */
/*
 * Per-frame max swings sharply when emitters pulse.  Naïvely
 * dividing by it makes every visible cell flicker between shade
 * levels.  Track an EMA of the per-frame max instead so the
 * renormaliser changes slowly.
 *
 * Two functions:
 *   dye_max_per_frame() — scan dye_density[] for the actual max.
 *   dye_normaliser_advance() — fold per-frame max into the EMA.
 *
 * The EMA state lives in dye_max_smoothed (declared in §6).
 */

static float dye_max_per_frame(void)
{
    float frame_max = 0.0f;
    for (int j = 1; j <= GRID_SIDE_INNER; j++) {
        for (int i = 1; i <= GRID_SIDE_INNER; i++) {
            float v = dye_density[cell_index(i, j)];
            if (v > frame_max) frame_max = v;
        }
    }
    return frame_max;
}

static void dye_normaliser_advance(void)
{
    float frame_max = dye_max_per_frame();
    dye_max_smoothed = DYE_MAX_EMA_OLD * dye_max_smoothed
                     + DYE_MAX_EMA_NEW * frame_max;
    if (dye_max_smoothed < DYE_MAX_FLOOR)
        dye_max_smoothed = DYE_MAX_FLOOR;
}

/* ===================================================================== */
/* §17  glyph_picker — density → glyph + shade                           */
/* ===================================================================== */
/*
 * Map a normalised dye density (∈ [0, 1] approximately) to a glyph +
 * shade index.  Cells below DENSITY_GLYPH_BLANK are skipped in the
 * renderer (transparent).  Above:
 *
 *   d < LOW   →  '.'  shade 0  (sparse trail)
 *   d < MID   →  ':'  shade 1
 *   d < HIGH  →  '+'  shade 2
 *   else      →  '#'  shade 3  (dense plume)
 */

typedef struct {
    char glyph;
    int  shade_index;
} GlyphChoice;

static GlyphChoice glyph_for_density(float density_normalised)
{
    GlyphChoice out = { '#', 3 };
    if      (density_normalised < DENSITY_GLYPH_LOW ) { out.glyph = '.'; out.shade_index = 0; }
    else if (density_normalised < DENSITY_GLYPH_MID ) { out.glyph = ':'; out.shade_index = 1; }
    else if (density_normalised < DENSITY_GLYPH_HIGH) { out.glyph = '+'; out.shade_index = 2; }
    else                                              { out.glyph = '#'; out.shade_index = 3; }
    return out;
}

/* ===================================================================== */
/* §18  grid_to_terminal — coordinate map for rendering                  */
/* ===================================================================== */
/*
 * Map physics-grid (i, j) ∈ [1, N] to terminal (col, row).  The grid
 * is rendered between rows HUD_RESERVED_ROWS_TOP and
 * (term_rows - HUD_RESERVED_ROWS_BOTTOM - 1).  We INVERT j (grid is
 * y-up; terminal is y-down) so the visual orientation matches the
 * mathematical convention.
 */

static int grid_i_to_term_col(int i, int term_cols)
{
    return (i - 1) * term_cols / GRID_SIDE_INNER;
}

static int grid_j_to_term_row(int j, int term_rows)
{
    int draw_rows = term_rows - HUD_RESERVED_ROWS_TOP
                              - HUD_RESERVED_ROWS_BOTTOM;
    if (draw_rows < 1) draw_rows = 1;
    return (GRID_SIDE_INNER - j) * draw_rows / GRID_SIDE_INNER
         + HUD_RESERVED_ROWS_TOP;
}

/* ===================================================================== */
/* §19  render — paint the dye field                                     */
/* ===================================================================== */
/*
 * For each interior cell:
 *   - Normalise its dye by the EMA-smoothed max (T9, §16).
 *   - If below the BLANK threshold, skip (let erase()'s blank stand).
 *   - Otherwise pick glyph + shade (§17) and draw with the channel's
 *     colour pair (§4-§5).
 *
 * Keep render free of any state-mutation: it READS dye_density and
 * dye_max_smoothed, WRITES only ncurses cells.  Side effects belong
 * upstream in dye_normaliser_advance().
 */
static void render_dye_field(int term_rows, int term_cols)
{
    for (int j = 1; j <= GRID_SIDE_INNER; j++) {
        for (int i = 1; i <= GRID_SIDE_INNER; i++) {
            float density_normalised =
                dye_density[cell_index(i, j)] / dye_max_smoothed;
            if (density_normalised < DENSITY_GLYPH_BLANK) continue;

            int col = grid_i_to_term_col(i, term_cols);
            int row = grid_j_to_term_row(j, term_rows);
            if (col < 0 || col >= term_cols)              continue;
            if (row < HUD_RESERVED_ROWS_TOP)              continue;
            if (row >= term_rows - HUD_RESERVED_ROWS_BOTTOM) continue;

            GlyphChoice gc = glyph_for_density(density_normalised);
            int pair_id = dye_pair_id(active_dye_channel, gc.shade_index);

            attron(COLOR_PAIR(pair_id));
            mvaddch(row, col, (chtype)(unsigned char)gc.glyph);
            attroff(COLOR_PAIR(pair_id));
        }
    }
}

/* ===================================================================== */
/* §20  hud — top status + bottom hint                                   */
/* ===================================================================== */
/*
 * Two HUD elements per CLAUDE.md spec:
 *   - STATUS at top-right  : PAIR_HUD bright yellow + A_BOLD
 *   - HINT   at bottom row : PAIR_HINT bright cyan   + A_BOLD
 *
 * Status shows: solver name, grid size, viscosity, dye colour,
 *               run/paused.
 * Hint shows: the keyboard shortcuts.
 */

static void hud_paint_status(int term_cols)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " StableFluids  grid:%dx%d  visc:%.2e  dye:%-5s  %s ",
             GRID_SIDE_INNER, GRID_SIDE_INNER,
             (double)viscosity_kinematic,
             dye_palette_table[active_dye_channel].name,
             simulation_paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    int x   = term_cols - len;
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows)
{
    const char *hint =
        " q:quit  p:pause  r:reset  arrows:wind  d/spc:dye  "
        "1/2/3:colour  +/-:visc ";
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0, "%s", hint);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §21  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

typedef struct { int rows; int cols; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    colors_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void)
{
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present_frame(Screen *s)
{
    erase();
    render_dye_field(s->rows, s->cols);
    hud_paint_status(s->cols);
    hud_paint_hint  (s->rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §22  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

/* Input helpers. */
static void drop_random_dye_blob(void)
{
    int i = rand_inner_cell();
    int j = rand_inner_cell();
    add_source_at(i, j, 0.0f, 0.0f, 5.0f);
}

static void apply_arrow_force(float force_x, float force_y)
{
    int centre = GRID_SIDE_INNER / 2;
    add_source_at(centre, centre, force_x, force_y, 1.0f);
}

static bool app_handle_key(int ch)
{
    switch (ch) {
        case 'q': case 'Q': case 27:
            return false;

        case 'p': case 'P':
            simulation_paused = !simulation_paused;
            break;

        case 'r': case 'R':
            fluid_reset();
            break;

        case ' ':
        case 'd': case 'D':
            drop_random_dye_blob();
            break;

        case KEY_LEFT:  apply_arrow_force(-1.0f,  0.0f); break;
        case KEY_RIGHT: apply_arrow_force( 1.0f,  0.0f); break;
        case KEY_UP:    apply_arrow_force( 0.0f, -1.0f); break;
        case KEY_DOWN:  apply_arrow_force( 0.0f,  1.0f); break;

        case '+': case '=':
            viscosity_kinematic *= VISCOSITY_FACTOR;
            if (viscosity_kinematic > VISCOSITY_MAX)
                viscosity_kinematic = VISCOSITY_MAX;
            break;
        case '-':
            viscosity_kinematic /= VISCOSITY_FACTOR;
            if (viscosity_kinematic < VISCOSITY_MIN)
                viscosity_kinematic = VISCOSITY_MIN;
            break;

        case '1': active_dye_channel = DYE_CHANNEL_BLUE;  break;
        case '2': active_dye_channel = DYE_CHANNEL_GREEN; break;
        case '3': active_dye_channel = DYE_CHANNEL_RED;   break;

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
    fluid_reset();

    /* Pre-warm: emitters + physics for N ticks so the first frame
     * has visible dye instead of a blank screen. */
    for (int i = 0; i < PREWARM_TICK_COUNT; i++) {
        emitters_inject();
        fluid_step(DT_DEFAULT);
    }
    /* Seed the EMA from the prewarmed state so the first rendered
     * frame doesn't blow out shades. */
    dye_max_smoothed = dye_max_per_frame();
    if (dye_max_smoothed < DYE_MAX_FLOOR) dye_max_smoothed = DYE_MAX_INITIAL;

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
        }

        /* ── physics ── */
        if (!simulation_paused) {
            emitters_inject();
            fluid_step(DT_DEFAULT);
        }

        /* ── visualisation prep + render ── */
        dye_normaliser_advance();
        screen_present_frame(&screen);

        /* ── frame cap ── */
        int64_t spent = clock_now_ns() - frame_start_ns;
        if (spent < RENDER_TICK_NS) clock_sleep_ns(RENDER_TICK_NS - spent);
    }

    return 0;
}
