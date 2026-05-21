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
 *   §5   colors           — dye_pair_id helper + colors_init split into
 *                            apply_dye_palettes (register_one_dye_channel)
 *                            + apply_chrome_palette
 *   §6   grid_state       — Fluid + Emitter + SimControls + DyeRenderer
 *                            sub-structs; Scene root; file-scope g_scene
 *   §7   boundary_apply   — fill the 1-cell ghost halo
 *   §8   gauss_seidel     — generic linear-system solver (16 sweeps)
 *   §9   diffuse          — implicit diffusion via §8
 *   §10  advect           — semi-Lagrangian back-trace + bilinear lerp
 *   §11  project          — pressure Poisson + gradient subtraction
 *   §12  fluid_step       — one full physics tick (3 named half-steps:
 *                            step_velocity_diffuse_and_project /
 *                            _advect_and_project /
 *                            step_dye_diffuse_and_advect)
 *   §13  sources          — point-source injection helper
 *   §14  emitters         — counter-rotating auto-emitter pair
 *                            (reads g_scene.emitter.swirl_phase)
 *   §15  fluid_reset      — wipe all fields to zero
 *   §16  dye_normaliser   — EMA-smoothed renormaliser (T9 anti-flicker)
 *                            (reads/writes g_scene.render.dye_max_smoothed)
 *   §17  glyph_picker     — density → glyph + shade
 *   §18  grid_to_terminal — coordinate map for rendering
 *   §19  render           — paint the dye field (uses g_scene.render.active_channel)
 *   §20  hud              — top status + bottom hint
 *   §21  screen           — ncurses init / cleanup / present
 *   §22  app              — FpsCounter + App (signals + screen + fps);
 *                            named action helpers (toggle_paused,
 *                            adjust_viscosity, set_dye_channel,
 *                            push_velocity_at_centre, drop_random_dye_blob);
 *                            main loop (numbered phases)
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
 *   g_scene.fluid.velocity_x, g_scene.fluid.velocity_y                u, v — current velocity
 *   g_scene.fluid.velocity_x_prev, g_scene.fluid.velocity_y_prev      scratch buffers
 *
 *   pressure_correction                   Φ from the Poisson solve
 *                                          (lives in g_scene.fluid.velocity_x_prev
 *                                           during project_step)
 *   divergence_field                      ∇·u, RHS of Poisson
 *                                          (lives in g_scene.fluid.velocity_y_prev)
 *
 *   g_scene.fluid.dye_density, g_scene.fluid.dye_density_prev         ρ (passive scalar) + scratch
 *
 *   cell_index(i, j)                       1-D offset (macro)
 *   GRID_SIDE_INNER                        physics interior side (= N)
 *   GRID_SIDE_TOTAL                        N + 2 (interior + ghost halo)
 *
 *   g_scene.render.active_channel                     0 = blue, 1 = green, 2 = red
 *   g_scene.fluid.viscosity_kinematic                    ν (the Navier-Stokes ν)
 *   diffusion_dye                          κ for the passive scalar
 *   g_scene.sim.paused                      run/pause toggle
 *   g_scene.emitter.swirl_phase                    rotation phase ∈ ℝ
 *   g_scene.render.dye_max_smoothed                       EMA-tracked renormaliser
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
 *   ── Stable Fluids algorithm (the core method) ─────────────────────
 *   [1] Stam, J. (1999), "Stable Fluids", SIGGRAPH '99 Proc., pp.
 *       121-128 — THE foundational paper.  Five pages, mostly
 *       diagrams.  Introduces the unconditionally-stable composition
 *       of advection + diffusion + projection used by this file.
 *   [2] Stam, J. (2003), "Real-Time Fluid Dynamics for Games", GDC —
 *       the classroom version, includes a 100-line C reference
 *       implementation that mirrors §advect / §diffuse / §project.
 *
 *   ── Underlying numerical methods ──────────────────────────────────
 *   [3] Bridson, R. (2008), "Fluid Simulation for Computer Graphics",
 *       CRC Press — standard textbook; chs. 1-4 cover everything in
 *       this file (semi-Lagrangian advection, Gauss-Seidel diffusion,
 *       Hodge-decomposition projection).
 *   [4] Foster, N. & Metaxas, D. (1996), "Realistic Animation of
 *       Liquids", GMIP 58 — pre-Stam grid solver with CFL-limited
 *       explicit time-stepping; useful historical contrast.
 *   [5] Saad, Y. (2003), "Iterative Methods for Sparse Linear
 *       Systems", 2nd ed., SIAM — Gauss-Seidel convergence theory
 *       behind our linear_solve in §diffuse / §project.
 *
 *   ── Physical theory ──────────────────────────────────────────────
 *   [6] Batchelor, G. K. (1967), "An Introduction to Fluid Dynamics",
 *       Cambridge UP — derivation of incompressible Navier-Stokes
 *       and the Hodge decomposition that motivates the project step.
 *
 *   ── Comparison with the Lagrangian approach ──────────────────────
 *   [7] Müller, M., Charypar, D. & Gross, M. (2003), "Particle-Based
 *       Fluid Simulation for Interactive Applications" — the SPH
 *       (particle) counterpart; project file fluid/fluid_sph.c.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *   [8] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the dye-density glyph ramp.
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; init_pair,
 *       use_default_colors, newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Navier%E2%80%93Stokes_equations
 *  [11] https://en.wikipedia.org/wiki/Stable_fluids
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
 *     normalise:  d_norm[i,j] = g_scene.fluid.dye_density[i,j] / g_scene.render.dye_max_smoothed
 *     glyph:      ramp = " .,+#" indexed by which threshold band
 *     colour:     palette[active_channel][shade_index]
 *     EMA:        g_scene.render.dye_max_smoothed = 0.95·old + 0.05·frame_max
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
 * red — a single bit in the g_scene.render.active_channel variable.
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
 *     frame_max = max(g_scene.fluid.dye_density)
 *     for each cell:
 *       d_norm = g_scene.fluid.dye_density[cell] / frame_max
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
 *     g_scene.render.dye_max_smoothed = α · g_scene.render.dye_max_smoothed_old + (1-α) · frame_max
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
#define GRID_SIDE_INNER 80 /* "N" — physics interior side */
#define GRID_SIDE_TOTAL (GRID_SIDE_INNER + 2)
#define GRID_TOTAL_CELLS (GRID_SIDE_TOTAL * GRID_SIDE_TOTAL)

/* ── PHYSICS ── */
#define DT_DEFAULT 0.05f           /* simulation time step    */
#define DIFFUSION_DYE 0.0001f      /* passive-scalar diffusion */
#define VISCOSITY_INITIAL 0.00001f /* kinematic viscosity ν   */
#define VISCOSITY_MIN 1e-7f
#define VISCOSITY_MAX 0.1f
#define VISCOSITY_FACTOR 2.0f /* '+' / '-' multiplier    */

#define GAUSS_SEIDEL_ITERATIONS 16 /* sweeps per linear solve */

/* Forces and dye scales applied inside add_source_at(). */
#define INJECT_FORCE_SCALE 50.0f /* arrows + emitters       */
#define INJECT_DYE_SCALE 50.0f   /* dye keys + emitters     */

/* Auto-emitter parameters. */
#define EMITTER_SWIRL_INCREMENT 0.04f /* radians per tick        */
#define EMITTER_FORCE_AMPLITUDE 1.5f
#define EMITTER_DYE_AMPLITUDE 3.0f

/* Pre-warm so the first frame has visible dye. */
#define PREWARM_TICK_COUNT 80

/* ── BOUNDARY-CONDITION TAGS (T6) ── */
enum {
  BOUNDARY_SCALAR = 0,     /* dye, pressure, divergence — mirror   */
  BOUNDARY_VELOCITY_X = 1, /* normal flips at L/R walls            */
  BOUNDARY_VELOCITY_Y = 2, /* normal flips at T/B walls            */
};

/* ── DYE CHANNELS ── */
enum {
  DYE_CHANNEL_BLUE = 0,
  DYE_CHANNEL_GREEN = 1,
  DYE_CHANNEL_RED = 2,
  DYE_CHANNEL_COUNT,
};

#define DYE_SHADE_COUNT 4 /* 4 brightness steps per channel  */

/* ── COLOUR PAIRS ── */
enum {
  PAIR_DYE_FIRST = 1, /* +0..+(3*4-1)            */
  PAIR_HUD = PAIR_DYE_FIRST + DYE_CHANNEL_COUNT * DYE_SHADE_COUNT,
  PAIR_HINT,
};

/* ── HUD ── */
#define HUD_RESERVED_ROWS_TOP 1
#define HUD_RESERVED_ROWS_BOTTOM 2

/* ── RENDER FRAME RATE ── */
#define RENDER_FPS 30
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS)

/* ── GLYPH RAMP THRESHOLDS (in normalised dye space) ── */
#define DENSITY_GLYPH_BLANK 0.02f
#define DENSITY_GLYPH_LOW 0.20f
#define DENSITY_GLYPH_MID 0.50f
#define DENSITY_GLYPH_HIGH 0.80f

/* ── BILINEAR-INTERP CLAMP MARGIN ── */
#define INTERP_CLAMP_MARGIN 0.5f

/* ── DYE NORMALISER (T9) ──
 * Per-frame max swings sharply with emitter pulses.  EMA smooths it. */
#define DYE_MAX_EMA_OLD 0.95f
#define DYE_MAX_EMA_NEW 0.05f
#define DYE_MAX_FLOOR 0.001f
#define DYE_MAX_INITIAL 1.0f

/* ── INDEX MACRO ──
 * Flatten (i, j) into 1-D field offset.  Ghost halo:
 *   real:   i, j ∈ [1, N]
 *   ghost:  i, j ∈ {0, N+1}
 */
#define cell_index(i, j) ((i) + GRID_SIDE_TOTAL * (j))

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
/* §3  rng — small wrapper for the dye-drop key                          */
/* ===================================================================== */
/*
 * Used only by the 'd' / SPACE key to drop a dye blob at a random
 * interior cell.  Physics is otherwise deterministic.
 */
static int rand_inner_cell(void) { return 1 + rand() % GRID_SIDE_INNER; }

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

/*
 * DyePalette — one of three dye channels (blue / red / green) as a
 * 4-shade ramp.
 *
 * Intent
 *   The Stable Fluids algorithm advects an arbitrary number of scalar
 *   "dye" fields through the velocity field.  We carry THREE such
 *   fields (one per channel) and visualise their independent
 *   densities at each cell.  Each channel has its own monotone-
 *   brightening 4-shade colour ramp:
 *
 *       shade 0  →  '.'  (dimmest, sparse dye)
 *       shade 1  →  ':'  (light)
 *       shade 2  →  '*'  (medium)
 *       shade 3  →  '#'  (densest)
 *
 *   Three channels × four shades = 12 colour pairs; mixing channels
 *   per cell would explode this to 4³ = 64.  We pick the DOMINANT
 *   channel per cell instead — see GlyphChoice doc.
 *
 * Why two palette tracks (colour_256 + colour_8)
 *   colour_256 holds 256-cube indices for COLORS >= 256.  colour_8 is
 *   the fallback for low-COLORS terminals; on those we lose tier
 *   resolution but keep channel identity.  Picking by terminal
 *   capability at init keeps the renderer hot path identical.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every shade-0 colour is kept in the bright half of the cube
 *   (≥ 30) so even sparsely-dyed cells stay visible against the
 *   default-black background.  Cube 16-23 and gray 232-239 are
 *   FORBIDDEN.
 *
 * Reference [9] Raymond's NCURSES HOWTO §6 — init_pair semantics
 *   that turn these palette arrays into live colour pairs.
 */
/*
 * Members
 *   colour_256[]   DYE_SHADE_COUNT xterm-256 indices, dim → bright.
 *                  Used when COLORS ≥ 256.  Shade index 0 is the dimmest
 *                  cell ('.'); the last index is the brightest ('#').
 *   colour_8[]     Same length, but ANSI-8 fallback indices for
 *                  terminals without the 256-colour cube.  The gradient
 *                  collapses (multiple shade buckets map to the same
 *                  ANSI colour) but stays monotonically brighter.
 *   name           HUD label ("BLUE", "RED", "GREEN").  Cycled by the
 *                  1 / 2 / 3 keys, surfaced in the HUD status line.
 *
 * Invariants
 *   All colour_256[i] ∈ [30, 255] per CLAUDE.md "Theme Palette
 *   Brightness" — cube 16–23 and gray 232–239 render as black on the
 *   default background and are forbidden.
 *   colour_256[i] ≤ colour_256[i+1] in perceived brightness so the
 *   density-to-shade mapping is monotone.
 *   name != NULL.
 *
 * References
 *   [9] Raymond's NCURSES-HOWTO §6 — init_pair semantics that turn
 *       these palette arrays into live colour pairs.
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 */
typedef struct {
    short       colour_256[DYE_SHADE_COUNT];  /* preferred 256-cube indices */
    short       colour_8  [DYE_SHADE_COUNT];  /* 8-colour fallback          */
    const char *name;                         /* "BLUE" / "RED" / "GREEN"   */
} DyePalette;

static const DyePalette dye_palette_table[DYE_CHANNEL_COUNT] = {
    /* BLUE — mid blue → bright cyan.  All four shades clearly visible. */
    {{33, 39, 51, 87},
     {COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
     "blue"},

    /* GREEN — green → bright lime. */
    {{34, 40, 82, 118},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE},
     "green"},

    /* RED — bright red → orange. */
    {{124, 160, 196, 208},
     {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW},
     "red"},
};

/* ===================================================================== */
/* §5  colors — pair init + dye_pair_id helper                           */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

/* The pair number for (channel, shade).  Depends on enum layout in §1. */
static int dye_pair_id(int channel, int shade) {
  return PAIR_DYE_FIRST + channel * DYE_SHADE_COUNT + shade;
}

/* Canonical bright xterm-256 indices for the project-standard HUD pairs
 * (CLAUDE.md §"HUD Standard"). */
enum {
    HUD_YELLOW_256 = 226,
    HUD_CYAN_256   = 51,
};

/* register_one_dye_channel — install DYE_SHADE_COUNT pairs for one
 * channel (blue / red / green), picking 256-cube or ANSI-8 fg per
 * the terminal's capability. */
static inline void register_one_dye_channel(int channel,
                                             const DyePalette *pal,
                                             bool have_256_colors) {
    for (int sh = 0; sh < DYE_SHADE_COUNT; sh++) {
        short fg = have_256_colors ? pal->colour_256[sh] : pal->colour_8[sh];
        init_pair((short)dye_pair_id(channel, sh), fg, -1);
    }
}

/* apply_dye_palettes — register all 3 × DYE_SHADE_COUNT density pairs. */
static inline void apply_dye_palettes(bool have_256_colors) {
    for (int ch = 0; ch < DYE_CHANNEL_COUNT; ch++)
        register_one_dye_channel(ch, &dye_palette_table[ch], have_256_colors);
}

/* apply_chrome_palette — theme-INDEPENDENT HUD pairs (bright yellow
 * status + bright cyan hint, per CLAUDE.md §"HUD Standard"). */
static inline void apply_chrome_palette(bool have_256_colors) {
    if (have_256_colors) {
        init_pair(PAIR_HUD,  HUD_YELLOW_256, -1);
        init_pair(PAIR_HINT, HUD_CYAN_256,   -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void colors_init(void) {
    start_color();
    use_default_colors();
    terminal_has_256_colours = (COLORS >= 256);

    apply_dye_palettes(terminal_has_256_colours);
    apply_chrome_palette(terminal_has_256_colours);
}

/* ===================================================================== */
/* §6  scene — the single owner of all simulation + render state         */
/* ===================================================================== */
/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Earlier versions kept the scene state as flat file-scope globals
 *   (every grid pass needs all of it).  This Scene struct groups the
 *   same fields together so the sim/render/control split is enforced
 *   by the STRUCT LAYOUT, not just by convention.  Helpers continue
 *   to reach the state through `g_scene.<field>` so the inner-loop
 *   math stays free of pointer threading.
 *
 *   Six 1-D arrays of GRID_TOTAL_CELLS floats hold the entire physics
 *   state.  Indexed by cell_index(i, j).  The 1-cell ghost halo around
 *   the physics interior is included in the array.
 *
 *   Two SCRATCH FIELDS exist for each velocity component because
 *   Stam's scheme reads the OLD field while writing into the NEW one.
 *   After each phase, role-swap (or memcpy if the same buffer is
 *   reused).
 *
 *   Awkward but standard: "_prev" doesn't mean "previous tick".  It
 *   means "the OTHER buffer in the current pair".  During project_step,
 *   the same _prev buffers double as pressure_correction and
 *   divergence_field.  See §11 comments.
 *
 * Locality (sim vs render)
 *   Fields are GROUPED EXPLICITLY so a reader can tell which
 *   subsystem touches each one:
 *     - advect / diffuse / project passes read it    → simulation
 *     - paint_field / hud_paint_* / dye normalisation→ rendering
 *     - g_scene.sim.paused gates the tick + HUD tag   → control state
 *
 *   Mis-classifying a field — e.g. accidentally letting a render
 *   helper write to g_scene.fluid.velocity_x — would couple visuals to physics and
 *   break reproducibility.  The "engine-toggle invariant" version of
 *   this warning is in the other fluid demos' Scene docs.
 *
 * Why one big struct (not split across Scene + App + Field)
 *   Every grid pass needs the whole field set.  Splitting would force
 *   pointer chains in the inner loop — the loop already runs ~15-20
 *   floating-point ops per cell, and the extra indirection would
 *   double that.  Keeping one Scene + one g_scene instance matches
 *   CLAUDE.md's "no dynamic allocation after init": ~6 × GRID_TOTAL
 *   floats in BSS, no malloc anywhere.
 *
 * Why these specific fields and no others
 *   - g_scene.fluid.velocity_x / g_scene.fluid.velocity_y          u, v fields — the CORE state.
 *   - g_scene.fluid.velocity_x_prev / g_scene.fluid.velocity_y_prev scratch; role-swapped each step
 *                                       (also used as pressure /
 *                                       divergence in project_step).
 *   - g_scene.fluid.dye_density / g_scene.fluid.dye_density_prev   ρ field + its scratch.
 *   - g_scene.fluid.viscosity_kinematic              ν; user-adjustable via +/-.
 *   - g_scene.emitter.swirl_phase              advances each tick to spin the
 *                                       emitter pattern.
 *   - g_scene.sim.paused                gate for the full step pipeline.
 *   - g_scene.render.active_channel               pure render — which channel
 *                                       palette colours the dye field
 *                                       (BLUE/RED/GREEN).  Must NOT
 *                                       touch any sim state.
 *   - g_scene.render.dye_max_smoothed                 EMA of max dye density used to
 *                                       normalise the colour ramp;
 *                                       pure visualisation.
 *
 * Things that DO NOT live here
 *   - The 3 dye palettes / glyph ramp     → file-scope constants in §4
 *   - Render-frame timing / FPS counter   → §22 App.fps (FpsCounter struct)
 *   - Signal flags (SIGINT, SIGWINCH)     → §22 App.quit / .need_resize
 *   - Terminal extent (cols/rows)         → §22 App.screen (Screen struct)
 *
 * Reference [9] Raymond NCURSES HOWTO on the scene-paint pipeline
 *   that the render fields below feed into.
 */
/*
 * Fluid — the six grid fields + the viscosity coefficient that drive
 * the Stam [1] solver.  This is the AUTHORITATIVE SIMULATION STATE.
 *
 * Intent
 *   advect / diffuse / project all read and write these fields.  The
 *   _prev arrays serve double duty: scratch for the next sub-step AND
 *   the source-of-truth between sub-steps (Stam's ping-pong pattern —
 *   each sub-pass reads from one buffer and writes into the other,
 *   so the previous tick's output is the next tick's input).
 *
 * Why six arrays (and not three pairs)
 *   Velocity has two components; dye is one scalar.  Each needs its
 *   own scratch — 2 × 2 + 1 × 2 = 6.  The naming makes the role
 *   explicit: `*_prev` is always the scratch / previous buffer the
 *   solver reads from when populating the matching live array.
 *
 * Why double-duty for the velocity_*_prev arrays
 *   During the PROJECT pass, velocity_x_prev becomes the PRESSURE
 *   field and velocity_y_prev becomes the DIVERGENCE field.  Naming
 *   them by purpose ("pressure", "divergence") would be clearer per
 *   pass but break the abstraction "this is scratch for velocity_x".
 *   Stam [1] §4 uses the same dual-role naming.
 *
 * Members
 *   velocity_x          u-component of velocity at each cell (cells / step).
 *                       The grid is column-major flattened; access via
 *                       cell_index(i, j) where (i, j) are 1-indexed
 *                       interior coords and 0 / N+1 are the ghost halo.
 *   velocity_y          v-component.  Same layout as velocity_x.
 *   velocity_x_prev     Scratch for the diffuse + advect sub-passes;
 *                       reinterpreted as PRESSURE during project.
 *   velocity_y_prev     Scratch for diffuse + advect; reinterpreted
 *                       as DIVERGENCE during project (§11 sets it to
 *                       ∇·v then solves ∇²p = ∇·v).
 *   dye_density         ρ — the visualisable scalar field that the
 *                       renderer paints (cells with ρ above
 *                       DYE_VISIBLE_FLOOR become a glyph).
 *   dye_density_prev    Scratch for the dye_density advect pass.
 *   viscosity_kinematic ν — kinematic viscosity (T3 tutorial).  Larger
 *                       ν → smoother motion.  User-tunable with + / −
 *                       keys; clamped to [VISCOSITY_MIN, VISCOSITY_MAX].
 *
 * Invariants
 *   All six fields include a 1-cell ghost halo at indices 0 and
 *   GRID_SIDE-1 along each axis; boundary_apply refills them after
 *   every sub-pass that mutates the field.
 *   viscosity_kinematic > 0 (clamped by app_handle_key).
 *   dye_density[idx] ≥ 0 after every step (semi-Lagrangian advect
 *   preserves non-negativity by construction; project doesn't touch
 *   the dye field).
 *
 * Memory footprint
 *   6 × GRID_TOTAL_CELLS × sizeof(float) — sits in BSS via the parent
 *   Scene's file-scope instance.  No malloc.  At GRID_SIDE_INNER = N
 *   that's 6 · (N+2)² · 4 bytes; typical N = 100 → ~250 KB.
 *
 * References
 *   [1] Stam, J. (1999), "Stable Fluids", SIGGRAPH '99 — the
 *       field layout (velocity + scratch + dye + scratch) used here.
 *   [2] Stam, J. (2003), "Real-Time Fluid Dynamics for Games", GDC —
 *       cleaner pedagogical exposition of the same algorithm.
 */
typedef struct {
    float velocity_x       [GRID_TOTAL_CELLS];  /* u                  */
    float velocity_y       [GRID_TOTAL_CELLS];  /* v                  */
    float velocity_x_prev  [GRID_TOTAL_CELLS];  /* u scratch / press  */
    float velocity_y_prev  [GRID_TOTAL_CELLS];  /* v scratch / diverg */
    float dye_density      [GRID_TOTAL_CELLS];  /* ρ                  */
    float dye_density_prev [GRID_TOTAL_CELLS];  /* ρ scratch          */
    float viscosity_kinematic;                  /* ν, kinematic visc  */
} Fluid;

/*
 * Emitter — state that drives the inflow pattern.
 *
 * Intent
 *   The auto-emitter (§14 emitters_inject) injects dye + velocity at
 *   two counter-rotating sources every tick.  The two sources spin
 *   in opposite directions, parameterised by ONE angular accumulator
 *   so they stay phase-locked.  Currently just one float, but the
 *   named type leaves room for future emitter modes (point sources,
 *   curl noise, mouse drag, etc.) without churning Scene.
 *
 * Members
 *   swirl_phase   Accumulator in RADIANS; advanced by emitters_inject
 *                 each tick to rotate the inflow direction.  No wrap
 *                 needed — cos/sin are 2π-periodic so the value can
 *                 grow unboundedly without losing precision over the
 *                 typical session length.
 *
 * Invariants
 *   No range constraint; cos/sin handle any real value.
 *
 * References
 *   None directly — the counter-rotating swirl is a project-specific
 *   visual choice.  The underlying "inject velocity + dye each tick"
 *   pattern is Stam [1] §5 (the "interactive sources" section).
 */
typedef struct {
    float swirl_phase;
} Emitter;

/*
 * SimControls — user-facing toggles that gate the physics pipeline.
 *
 * Intent
 *   The input handler writes these; main reads them to skip
 *   emitters_inject + fluid_step on paused frames.  Bundled into a
 *   named struct for symmetry with the rest of the project — every
 *   demo's Scene has a SimControls sub-struct now, so a reader who
 *   knows one Scene knows them all.
 *
 * Members
 *   paused   true → fluid_step + emitters_inject skipped; HUD shows
 *            "PAUSED".  Toggled by SPACE / 'p' key.  Rendering
 *            continues so the user can study the frozen field.
 *
 * Invariants
 *   None — pure bool.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * DyeRenderer — PURE RENDER STATE; must NOT touch sim fields.
 *
 * Intent
 *   The renderer needs two pieces of state that the SIMULATION must
 *   ignore: which colour channel is active (BLUE / RED / GREEN — the
 *   1 / 2 / 3 keys cycle), and the smoothed maximum dye density used
 *   to normalise the brightness ramp.  Mis-classifying these as sim
 *   would break the engine-toggle invariant that "changing the look
 *   doesn't disturb the physics".
 *
 * Why an EMA (not the raw per-frame max)
 *   The frame-max dye density jumps abruptly when a new dye blob is
 *   added or an emitter pulse hits.  Using the raw max as the
 *   normaliser would FLASH the entire field every time the max
 *   changed (because dim cells would suddenly be rescaled).  An EMA
 *   (with DYE_EMA_ALPHA per-frame mix weight) smooths the
 *   denominator so the ramp re-tunes gradually over ~10 frames
 *   instead of one.  T9 anti-flicker in the tutorial.
 *
 * Members
 *   active_channel     DYE_CHANNEL_BLUE / _RED / _GREEN — selects
 *                      which palette in DYE_PALETTES is used.
 *   dye_max_smoothed   EMA of frame-max dye density.  The ramp uses
 *                      this as its top-end so saturated cells don't
 *                      blow out.  Updated each frame by
 *                      dye_normaliser_advance:
 *                        smoothed = α · max_this_frame
 *                                 + (1−α) · smoothed
 *                      where α = DYE_EMA_ALPHA.
 *
 * Invariants
 *   active_channel ∈ {DYE_CHANNEL_BLUE, DYE_CHANNEL_RED, DYE_CHANNEL_GREEN}.
 *   dye_max_smoothed ≥ DYE_MAX_FLOOR (clamped in
 *   dye_normaliser_advance so the divider never approaches 0).
 *
 * References
 *   [9] Raymond — render-side state convention.
 *   T9 tutorial — EMA anti-flicker rationale.
 */
typedef struct {
    int   active_channel;
    float dye_max_smoothed;
} DyeRenderer;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── fluid    : Fluid        ← 6 grid fields + viscosity
 *       ├── emitter  : Emitter      ← swirl_phase
 *       ├── sim      : SimControls  ← paused
 *       └── render   : DyeRenderer  ← active_channel + dye_max_smoothed
 *
 *   Each sub-struct names its responsibility, so a reader scrolling
 *   the body of any pass can answer "is this touching sim or render?"
 *   from the field path alone.
 *
 * Why one file-scope `g_scene` (and not a pointer threaded everywhere)
 *   The grid arrays sum to several MB; the Stam advection / project
 *   inner loops touch all 6 arrays per cell.  Threading a Scene *
 *   through ~30 helpers would add no clarity and a tiny perf cost.
 *   Keeping ONE global with clearly-named sub-structs gives the
 *   same conceptual partition without signature churn.
 *
 * Members
 *   fluid     The 6 grid fields + ν (see Fluid docblock).  Touched
 *             by every advect / diffuse / project sub-pass.
 *   emitter   Auto-emitter angular state (see Emitter docblock).
 *             Touched only by emitters_inject in §14.
 *   sim       paused flag — user-facing toggle.
 *   render    active_channel + dye_max_smoothed — pure render state;
 *             the SIMULATION passes never touch these fields.
 *
 * Invariants
 *   All four sub-struct invariants hold simultaneously (see each
 *   sub-struct's docblock for its own constraints).
 *   The CRITICAL cross-cutting invariant: SIMULATION passes (advect,
 *   diffuse, project, fluid_step) MUST only touch `fluid` and
 *   `emitter`; RENDER (paint_field, hud_paint_*, dye_normaliser)
 *   must only READ `fluid` (not write) and may freely write `render`.
 *   This "engine ≠ render" separation lets the user toggle theme /
 *   pause without disturbing the physics state.
 *
 * References
 *   [1] Stam 1999 — the simulation-state schema in `fluid`.
 *   [9] Raymond NCURSES HOWTO — the scene-paint pipeline that the
 *       `render` fields feed into.
 */
typedef struct {
    Fluid       fluid;
    Emitter     emitter;
    SimControls sim;
    DyeRenderer render;
} Scene;

static Scene g_scene = {
    .fluid   = { .viscosity_kinematic = VISCOSITY_INITIAL },
    .emitter = { .swirl_phase         = 0.0f },
    .sim     = { .paused              = false },
    .render  = { .active_channel      = DYE_CHANNEL_BLUE,
                 .dye_max_smoothed    = DYE_MAX_INITIAL },
};

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
/* Pick the right ghost-cell value for one wall axis:
 *   - SCALAR  / mirroring axis : copy the inner value verbatim
 *                                 (Neumann zero-flux BC).
 *   - NO-SLIP axis             : negate the inner value so the
 *                                 ghost+inner midpoint averages to 0
 *                                 (Dirichlet zero-velocity BC).
 * Caller passes `is_normal_velocity` true only when (axis_kind) matches
 * the wall's NORMAL component (VELOCITY_X at L/R, VELOCITY_Y at T/B). */
static inline float ghost_value_for_wall(float inner, bool is_normal_velocity) {
    return is_normal_velocity ? -inner : inner;
}

/* Left and right walls: fill columns 0 and N+1 from columns 1 and N.
 * Negate when this is the normal-velocity field (VELOCITY_X), so the
 * no-slip Dirichlet condition u_wall = 0 holds in the limit. */
static inline void fill_left_right_wall_ghosts(int boundary_kind,
                                                float *field, int N) {
    bool is_normal = (boundary_kind == BOUNDARY_VELOCITY_X);
    for (int j = 1; j <= N; j++) {
        float left_inner  = field[cell_index(1,     j)];
        float right_inner = field[cell_index(N,     j)];
        field[cell_index(0,     j)] = ghost_value_for_wall(left_inner,  is_normal);
        field[cell_index(N + 1, j)] = ghost_value_for_wall(right_inner, is_normal);
    }
}

/* Top and bottom walls: fill rows 0 and N+1 from rows 1 and N.
 * Negate when this is the normal-velocity field (VELOCITY_Y). */
static inline void fill_top_bottom_wall_ghosts(int boundary_kind,
                                                float *field, int N) {
    bool is_normal = (boundary_kind == BOUNDARY_VELOCITY_Y);
    for (int i = 1; i <= N; i++) {
        float top_inner    = field[cell_index(i, 1    )];
        float bottom_inner = field[cell_index(i, N    )];
        field[cell_index(i, 0    )] = ghost_value_for_wall(top_inner,    is_normal);
        field[cell_index(i, N + 1)] = ghost_value_for_wall(bottom_inner, is_normal);
    }
}

/* Average the two adjacent edge ghosts to fill each of the four
 * corner ghost cells.  Corners are reached by both the L/R loop and
 * the T/B loop; the average is the simplest consistent reconciliation. */
static inline void fill_corner_ghosts_by_averaging(float *field, int N) {
    field[cell_index(0,     0    )] = 0.5f * (field[cell_index(1,     0    )]
                                           +  field[cell_index(0,     1    )]);
    field[cell_index(0,     N + 1)] = 0.5f * (field[cell_index(1,     N + 1)]
                                           +  field[cell_index(0,     N    )]);
    field[cell_index(N + 1, 0    )] = 0.5f * (field[cell_index(N,     0    )]
                                           +  field[cell_index(N + 1, 1    )]);
    field[cell_index(N + 1, N + 1)] = 0.5f * (field[cell_index(N,     N + 1)]
                                           +  field[cell_index(N + 1, N    )]);
}

/*
 * boundary_apply — fill the 1-cell ghost halo so neighbour reads at
 * interior cells automatically satisfy the desired wall condition.
 *
 * Pseudocode:
 *   fill_left_right_wall_ghosts(kind, field, N)
 *   fill_top_bottom_wall_ghosts(kind, field, N)
 *   fill_corner_ghosts_by_averaging(field, N)
 *
 * boundary_kind:
 *   BOUNDARY_SCALAR     → mirror everywhere (Neumann zero-flux)
 *   BOUNDARY_VELOCITY_X → flip sign on L/R, mirror T/B   (no-slip in u)
 *   BOUNDARY_VELOCITY_Y → flip sign on T/B, mirror L/R   (no-slip in v)
 */
static void boundary_apply(int boundary_kind, float *field) {
    int N = GRID_SIDE_INNER;
    fill_left_right_wall_ghosts(boundary_kind, field, N);
    fill_top_bottom_wall_ghosts(boundary_kind, field, N);
    fill_corner_ghosts_by_averaging(field, N);
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
static void gauss_seidel_solve(int boundary_kind, float *x, const float *b,
                               float a, float c) {
  float inv_c = 1.0f / c;
  int N = GRID_SIDE_INNER;
  for (int sweep = 0; sweep < GAUSS_SEIDEL_ITERATIONS; sweep++) {
    for (int j = 1; j <= N; j++) {
      for (int i = 1; i <= N; i++) {
        float neighbour_sum = x[cell_index(i - 1, j)] +
                              x[cell_index(i + 1, j)] +
                              x[cell_index(i, j - 1)] + x[cell_index(i, j + 1)];
        x[cell_index(i, j)] = (b[cell_index(i, j)] + a * neighbour_sum) * inv_c;
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
static void diffuse(int boundary_kind, float *field_new, const float *field_old,
                    float diffusion_coefficient, float dt) {
  float a =
      dt * diffusion_coefficient * (float)(GRID_SIDE_INNER * GRID_SIDE_INNER);

  memcpy(field_new, field_old, sizeof(float) * GRID_TOTAL_CELLS);
  gauss_seidel_solve(boundary_kind, field_new, field_old, a, 1.0f + 4.0f * a);
}

/* ===================================================================== */
/* §10  advect — semi-Lagrangian back-trace + bilinear lerp              */
/* ===================================================================== */

/*
 * DeparturePoint — Stam's semi-Lagrangian back-trace result.
 *
 * Intent
 *   "Where in the OLD field did the fluid currently at (i, j) come
 *   from?"  Stam's [1] advection looks BACKWARD along the velocity:
 *   a fluid parcel sitting at (i, j) at time t was at
 *
 *      (i − dt·u(i,j),  j − dt·v(i,j))
 *
 *   at time t-dt.  Sampling the OLD field at that location and
 *   copying the value forward is the entire idea behind Stam's
 *   UNCONDITIONAL STABILITY — no values are CREATED, only RELOCATED.
 *
 * Why a struct (vs two floats or two output pointers)
 *   trace_velocity_backward needs to return TWO floats together.
 *   Returning a tiny struct by value (8 bytes = one register pair on
 *   x86-64 / ARM ABIs) gives a clean call-site:
 *
 *      DeparturePoint d = trace_velocity_backward(i, j, vx, vy, dt);
 *      float sample = bilinear_interp_at(field, d.x, d.y);
 *
 *   Versus an out-parameter idiom which would force two separate
 *   reads at the call site.
 *
 * Members
 *   x, y   Departure-point coordinates in CELL UNITS (the same units
 *          as the grid indices).  Float because the back-trace
 *          lands BETWEEN cells — bilinear_interp_at then samples
 *          the four surrounding cells weighted by fractional offset.
 *
 * Invariants
 *   x, y are clamped to the interior [0.5, GRID_SIDE_INNER + 0.5]
 *   by trace_velocity_backward so bilinear sampling never reads off
 *   the boundary ghost cells without their boundary-condition fill.
 *
 * Reference
 *   [1] Stam 1999 §3 — the "Linear backwards solver" that this
 *       struct's two floats parameterise.
 */
typedef struct { float x; float y; } DeparturePoint;

/* Back-trace one cell's velocity by dt cells (in cell units, not
 * world units — dt_cells = dt·N already incorporates the grid scaling). */
static inline DeparturePoint trace_velocity_backward(int i, int j,
                                                    const float *vx,
                                                    const float *vy,
                                                    float dt_cells) {
    DeparturePoint dp;
    dp.x = (float)i - dt_cells * vx[cell_index(i, j)];
    dp.y = (float)j - dt_cells * vy[cell_index(i, j)];
    return dp;
}

/* Clamp the departure point into [INTERP_CLAMP_MARGIN, N + margin]
 * so the four-corner bilinear stencil at (i0..i1, j0..j1) never
 * reaches outside the allocated grid.  Without this, fast-moving
 * fluid would back-trace off the edge and segfault on bilinear_sample. */
static inline void clamp_departure_to_interp_bounds(DeparturePoint *dp, int N) {
    float lo = INTERP_CLAMP_MARGIN;
    float hi = (float)N + INTERP_CLAMP_MARGIN;
    if (dp->x < lo) dp->x = lo;
    if (dp->x > hi) dp->x = hi;
    if (dp->y < lo) dp->y = lo;
    if (dp->y > hi) dp->y = hi;
}

/* Bilinear interpolation at the real-valued sample point (x, y):
 *   sample = s0·(t0·NW + t1·SW) + s1·(t0·NE + t1·SE)
 * where (s0, s1) and (t0, t1) are the (1-α, α) fractional weights in
 * x and y respectively.  Output is a weighted average of the FOUR
 * surrounding corner values, hence always between their min and max
 * — values can shrink but NEVER GROW.  This is what makes Stam's
 * scheme unconditionally stable (no CFL constraint).  [3] Bridson §3. */
static inline float bilinear_sample(const float *field, float x, float y) {
    int   i0 = (int)x;
    int   j0 = (int)y;
    int   i1 = i0 + 1;
    int   j1 = j0 + 1;
    float s1 = x - (float)i0,  s0 = 1.0f - s1;
    float t1 = y - (float)j0,  t0 = 1.0f - t1;
    return s0 * (t0 * field[cell_index(i0, j0)]
              +  t1 * field[cell_index(i0, j1)])
         + s1 * (t0 * field[cell_index(i1, j0)]
              +  t1 * field[cell_index(i1, j1)]);
}

/*
 * advect — semi-Lagrangian advection [1] Stam 1999 §3.
 *
 * Pseudocode:
 *   for each interior cell (i, j):
 *     dp     = trace_velocity_backward(i, j, v, dt)
 *              clamp_departure_to_interp_bounds(&dp, N)
 *     new[i,j] = bilinear_sample(old, dp.x, dp.y)
 *   apply boundary conditions to the new field
 */
static void advect(int boundary_kind, float *new_field, const float *old_field,
                   const float *vx, const float *vy, float dt) {
    int   N        = GRID_SIDE_INNER;
    float dt_cells = dt * (float)N;

    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            DeparturePoint dp = trace_velocity_backward(i, j, vx, vy, dt_cells);
            clamp_departure_to_interp_bounds(&dp, N);
            new_field[cell_index(i, j)] = bilinear_sample(old_field, dp.x, dp.y);
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
/* Compute the discrete divergence ∇·u of the velocity field into
 * `divergence_field` using central differences:
 *
 *   ∇·u ≈ ½·h · ( (vx[i+1, j] − vx[i−1, j]) + (vy[i, j+1] − vy[i, j−1]) )
 *
 * The −½·h scaling (rather than +½·h) folds the sign needed for the
 * Poisson equation ∇²p = ∇·u into the divergence field itself, so the
 * Gauss-Seidel step that solves it doesn't need a separate sign flip.
 * See [1] Stam 1999 §3.5 and [3] Bridson §4.4. */
static inline void compute_divergence_field(const float *vx, const float *vy,
                                            float *divergence_field) {
    int   N = GRID_SIDE_INNER;
    float h = 1.0f / (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            divergence_field[cell_index(i, j)] = -0.5f * h *
                (vx[cell_index(i + 1, j)] - vx[cell_index(i - 1, j)] +
                 vy[cell_index(i, j + 1)] - vy[cell_index(i, j - 1)]);
        }
    }
}

/* Zero the pressure-correction field.  Gauss-Seidel iterates AGAINST
 * the existing values, so this is the standard "warm-start with zero"
 * — gives a clean convergence from a known initial residual. */
static inline void zero_pressure_field(float *pressure_correction) {
    int N = GRID_SIDE_INNER;
    for (int j = 1; j <= N; j++)
        for (int i = 1; i <= N; i++)
            pressure_correction[cell_index(i, j)] = 0.0f;
}

/* Solve the Poisson equation ∇²p = ∇·u for the pressure correction p,
 * via the generic Gauss-Seidel iterator (§8).  Coefficients a=1, c=4
 * are the 2-D 5-point Laplacian stencil:
 *
 *   p[i, j] = (div[i, j] + p[i±1, j] + p[i, j±1]) / 4
 *
 * Converges in O(N) iterations for our grid size; warmer iteration
 * counts produce smoother pressure fields at the cost of one frame's
 * accuracy.  Ref [5] Saad §4.1 on Gauss-Seidel convergence. */
static inline void solve_pressure_poisson(float *pressure_correction,
                                          const float *divergence_field) {
    gauss_seidel_solve(BOUNDARY_SCALAR, pressure_correction, divergence_field,
                       1.0f, 4.0f);
}

/* Subtract ∇p from the velocity field, making it divergence-free:
 *
 *   u_new = u − ∇p
 *
 * where ∇p is computed via central differences, scaled by ½·N (the
 * inverse of the grid spacing h = 1/N used in the divergence step).
 * This is the Helmholtz-Hodge projection step that makes Stam's
 * Stable Fluids method unconditionally divergence-free.  [6] Batchelor
 * §2.7 for the Hodge decomposition theorem.                        */
static inline void subtract_pressure_gradient(float *vx, float *vy,
                                              const float *pressure_correction) {
    int   N        = GRID_SIDE_INNER;
    float n_factor = 0.5f * (float)N;
    for (int j = 1; j <= N; j++) {
        for (int i = 1; i <= N; i++) {
            vx[cell_index(i, j)] -= n_factor *
                (pressure_correction[cell_index(i + 1, j)] -
                 pressure_correction[cell_index(i - 1, j)]);
            vy[cell_index(i, j)] -= n_factor *
                (pressure_correction[cell_index(i, j + 1)] -
                 pressure_correction[cell_index(i, j - 1)]);
        }
    }
}

/*
 * project — Helmholtz-Hodge projection onto the divergence-free subspace.
 *
 * Pseudocode (3 named phases):
 *   PHASE 1 — measure and prepare
 *     compute_divergence_field(vx, vy, divergence_field)
 *     zero_pressure_field    (pressure_correction)
 *     boundary_apply(SCALAR, divergence_field)
 *     boundary_apply(SCALAR, pressure_correction)
 *
 *   PHASE 2 — solve the Poisson system
 *     solve_pressure_poisson(pressure_correction, divergence_field)
 *
 *   PHASE 3 — correct the velocity
 *     subtract_pressure_gradient(vx, vy, pressure_correction)
 *     boundary_apply(VELOCITY_X, vx)
 *     boundary_apply(VELOCITY_Y, vy)
 *
 * After phase 3, ∇·u ≈ 0 — incompressibility enforced.  This is THE
 * step that makes Stable Fluids stable.  Refs [1] Stam 1999 §3.5
 * "Hodge decomposition"; [3] Bridson §4.4; [6] Batchelor §2.7.
 */
static void project(float *vx, float *vy, float *pressure_correction,
                    float *divergence_field) {
    /* PHASE 1 — measure: build the right-hand side, zero the unknown. */
    compute_divergence_field(vx, vy, divergence_field);
    zero_pressure_field    (pressure_correction);
    boundary_apply(BOUNDARY_SCALAR, divergence_field);
    boundary_apply(BOUNDARY_SCALAR, pressure_correction);

    /* PHASE 2 — solve: Gauss-Seidel relaxation of ∇²p = ∇·u. */
    solve_pressure_poisson(pressure_correction, divergence_field);

    /* PHASE 3 — correct: subtract ∇p from u, re-apply velocity BCs. */
    subtract_pressure_gradient(vx, vy, pressure_correction);
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
 *   diffuse vx → g_scene.fluid.velocity_x_prev   (using g_scene.fluid.velocity_x as old)
 *   diffuse vy → g_scene.fluid.velocity_y_prev
 *   project    velocity_*_prev     (pressure correction lives in vx, vy)
 *   advect vx  → g_scene.fluid.velocity_x        (advect by velocity_*_prev)
 *   advect vy  → g_scene.fluid.velocity_y
 *   project    velocity_*          (clean residual divergence)
 *
 * DYE phases:
 *   diffuse dye → g_scene.fluid.dye_density_prev
 *   advect  dye → g_scene.fluid.dye_density       (using clean velocity_*)
 *
 * The buffer juggling is awkward but correct.  Stam's reference
 * implementation uses exactly this convention — we keep it.
 */
/*
 * step_velocity_diffuse_and_project — Stam [1] half-step:
 *   (1) implicit diffuse u, v into the _prev buffers
 *   (2) project (Hodge decompose) the diffused field back to
 *       divergence-free; result lands in u, v.
 *
 * After this returns: (velocity_x, velocity_y) is the diffused +
 * projected field; the _prev arrays hold scratch from the projector.
 */
static inline void step_velocity_diffuse_and_project(Fluid *f, float dt) {
  diffuse(BOUNDARY_VELOCITY_X, f->velocity_x_prev, f->velocity_x,
          f->viscosity_kinematic, dt);
  diffuse(BOUNDARY_VELOCITY_Y, f->velocity_y_prev, f->velocity_y,
          f->viscosity_kinematic, dt);
  project(f->velocity_x_prev, f->velocity_y_prev,
          f->velocity_x,      f->velocity_y);
}

/*
 * step_velocity_advect_and_project — Stam [1] half-step:
 *   (1) semi-Lagrangian advect u and v through the (now-diffused)
 *       velocity field stored in _prev
 *   (2) project again — advection can re-introduce divergence
 *       (T7 tutorial), so a second Hodge decomposition is required.
 */
static inline void step_velocity_advect_and_project(Fluid *f, float dt) {
  advect(BOUNDARY_VELOCITY_X, f->velocity_x, f->velocity_x_prev,
         f->velocity_x_prev, f->velocity_y_prev, dt);
  advect(BOUNDARY_VELOCITY_Y, f->velocity_y, f->velocity_y_prev,
         f->velocity_x_prev, f->velocity_y_prev, dt);
  project(f->velocity_x,      f->velocity_y,
          f->velocity_x_prev, f->velocity_y_prev);
}

/*
 * step_dye_diffuse_and_advect — diffuse + advect the dye scalar
 * field (no projection needed; dye is a passive tracer, not a
 * velocity).  Reads the FINAL (project-cleaned) velocity from
 * (velocity_x, velocity_y).
 */
static inline void step_dye_diffuse_and_advect(Fluid *f, float dt) {
  diffuse(BOUNDARY_SCALAR, f->dye_density_prev, f->dye_density,
          DIFFUSION_DYE, dt);
  advect(BOUNDARY_SCALAR, f->dye_density, f->dye_density_prev,
         f->velocity_x, f->velocity_y, dt);
}

static void fluid_step(float dt) {
  Fluid *f = &g_scene.fluid;
  /* (1) velocity half-step: diffuse → project */
  step_velocity_diffuse_and_project(f, dt);
  /* (2) velocity half-step: advect → project */
  step_velocity_advect_and_project(f, dt);
  /* (3) dye half-step: diffuse → advect (no projection — passive tracer) */
  step_dye_diffuse_and_advect(f, dt);
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
static void add_source_at(int i, int j, float force_x, float force_y,
                          float dye_value) {
  if (i < 1 || i > GRID_SIDE_INNER)
    return;
  if (j < 1 || j > GRID_SIDE_INNER)
    return;
  g_scene.fluid.velocity_x[cell_index(i, j)] += DT_DEFAULT * force_x * INJECT_FORCE_SCALE;
  g_scene.fluid.velocity_y[cell_index(i, j)] += DT_DEFAULT * force_y * INJECT_FORCE_SCALE;
  g_scene.fluid.dye_density[cell_index(i, j)] += DT_DEFAULT * dye_value * INJECT_DYE_SCALE;
}

/* ===================================================================== */
/* §14  emitters — counter-rotating auto-emitter pair                    */
/* ===================================================================== */
/*
 * Two emitters at 1/3 and 2/3 from the left, both at vertical centre.
 * Each injects dye + a swirling velocity that rotates with
 * g_scene.emitter.swirl_phase.  The two are 180° out of phase so their
 * resulting flows COUNTER-ROTATE — visually striking, immediately
 * shows off the projection step (without it, the swirls would just
 * bleed into source/sink artefacts).
 */
static void emitters_inject(void) {
  g_scene.emitter.swirl_phase += EMITTER_SWIRL_INCREMENT;

  int N = GRID_SIDE_INNER;

  /* Left emitter — clockwise swirl. */
  int i_left = N / 3;
  int j_left = N / 2;
  float fx_left = cosf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  float fy_left = sinf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  add_source_at(i_left, j_left, fx_left, fy_left, EMITTER_DYE_AMPLITUDE);

  /* Right emitter — counter-rotating. */
  int i_right = 2 * N / 3;
  int j_right = N / 2;
  float fx_right = -cosf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  float fy_right = -sinf(g_scene.emitter.swirl_phase) * EMITTER_FORCE_AMPLITUDE;
  add_source_at(i_right, j_right, fx_right, fy_right, EMITTER_DYE_AMPLITUDE);
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
static void fluid_reset(void) {
  memset(g_scene.fluid.velocity_x, 0, sizeof g_scene.fluid.velocity_x);
  memset(g_scene.fluid.velocity_y, 0, sizeof g_scene.fluid.velocity_y);
  memset(g_scene.fluid.velocity_x_prev, 0, sizeof g_scene.fluid.velocity_x_prev);
  memset(g_scene.fluid.velocity_y_prev, 0, sizeof g_scene.fluid.velocity_y_prev);
  memset(g_scene.fluid.dye_density, 0, sizeof g_scene.fluid.dye_density);
  memset(g_scene.fluid.dye_density_prev, 0, sizeof g_scene.fluid.dye_density_prev);
  g_scene.emitter.swirl_phase = 0.0f;
  g_scene.render.dye_max_smoothed = DYE_MAX_INITIAL;
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
 *   dye_max_per_frame() — scan g_scene.fluid.dye_density[] for the actual max.
 *   dye_normaliser_advance() — fold per-frame max into the EMA.
 *
 * The EMA state lives in g_scene.render.dye_max_smoothed (declared in §6).
 */

static float dye_max_per_frame(void) {
  float frame_max = 0.0f;
  for (int j = 1; j <= GRID_SIDE_INNER; j++) {
    for (int i = 1; i <= GRID_SIDE_INNER; i++) {
      float v = g_scene.fluid.dye_density[cell_index(i, j)];
      if (v > frame_max)
        frame_max = v;
    }
  }
  return frame_max;
}

static void dye_normaliser_advance(void) {
  float frame_max = dye_max_per_frame();
  g_scene.render.dye_max_smoothed =
      DYE_MAX_EMA_OLD * g_scene.render.dye_max_smoothed + DYE_MAX_EMA_NEW * frame_max;
  if (g_scene.render.dye_max_smoothed < DYE_MAX_FLOOR)
    g_scene.render.dye_max_smoothed = DYE_MAX_FLOOR;
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

/*
 * GlyphChoice — one cell's drawing instruction, derived from dye density.
 *
 * Intent
 *   The renderer needs to know TWO things per cell: which glyph to
 *   draw, and which shade in the active dye channel's palette to
 *   colour it.  density_to_glyph() computes both in one shot.
 *
 * Why a struct and not two return values
 *   C functions return one value; returning two via output pointers
 *   makes call sites read backwards ("look up output_pair, then
 *   output_glyph").  A small return-by-value struct keeps callers
 *   linear: `gc = density_to_glyph(d); mvaddch(... gc.glyph, ...);`.
 *
 * Why glyph + shade_index (not glyph + colour_pair_id)
 *   The CHANNEL of dye in this cell (blue / red / green — picked by
 *   the dominant-channel rule earlier) varies, but the shade WITHIN
 *   that channel is what density_to_glyph computes.  Decoupling lets
 *   the caller combine the two: pair = channel_base[chan] + shade.
 */
/*
 * Members
 *   glyph         ASCII glyph for this cell, from the brightness ramp
 *                 '.' (sparsest) → ':' → '+' → '#' (densest), or
 *                 0  meaning "below DYE_VISIBLE_FLOOR, do not draw".
 *   shade_index   Index 0..DYE_SHADE_COUNT-1 into the active palette's
 *                 colour_256[] / colour_8[] arrays.  The caller
 *                 combines this with the channel base pair-id:
 *                 `pair = channel_base[active_channel] + shade_index`.
 *
 * Invariants
 *   glyph ∈ {0, '.', ':', '+', '#'}.
 *   0 ≤ shade_index < DYE_SHADE_COUNT.
 *   glyph == 0  ⇔  density was below DYE_VISIBLE_FLOOR.
 *
 * Reference
 *   [9] Raymond NCURSES-HOWTO §6 — pair lookup + character output.
 */
typedef struct {
    char glyph;          /* '.' ':' '+' '#' or 0 for "do not draw"  */
    int  shade_index;    /* 0..DYE_SHADE_COUNT-1 within channel ramp */
} GlyphChoice;

static GlyphChoice glyph_for_density(float density_normalised) {
  GlyphChoice out = {'#', 3};
  if (density_normalised < DENSITY_GLYPH_LOW) {
    out.glyph = '.';
    out.shade_index = 0;
  } else if (density_normalised < DENSITY_GLYPH_MID) {
    out.glyph = ':';
    out.shade_index = 1;
  } else if (density_normalised < DENSITY_GLYPH_HIGH) {
    out.glyph = '+';
    out.shade_index = 2;
  } else {
    out.glyph = '#';
    out.shade_index = 3;
  }
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

static int grid_i_to_term_col(int i, int term_cols) {
  return (i - 1) * term_cols / GRID_SIDE_INNER;
}

static int grid_j_to_term_row(int j, int term_rows) {
  int draw_rows = term_rows - HUD_RESERVED_ROWS_TOP - HUD_RESERVED_ROWS_BOTTOM;
  if (draw_rows < 1)
    draw_rows = 1;
  return (GRID_SIDE_INNER - j) * draw_rows / GRID_SIDE_INNER +
         HUD_RESERVED_ROWS_TOP;
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
 * Keep render free of any state-mutation: it READS g_scene.fluid.dye_density and
 * g_scene.render.dye_max_smoothed, WRITES only ncurses cells.  Side effects belong
 * upstream in dye_normaliser_advance().
 */
/* Normalise the raw dye density at grid cell (i, j) by the EMA-
 * smoothed max so the colour ramp tracks the field's CURRENT dynamic
 * range.  Without this, a single bright plume at startup would peg
 * every later frame's ramp to "barely visible" — see [1] Stam §6
 * "tone mapping" for the same trick in real-time fluid graphics. */
static inline float normalise_dye_at(int i, int j) {
    return g_scene.fluid.dye_density[cell_index(i, j)] / g_scene.render.dye_max_smoothed;
}

/* Project grid cell (i, j) onto the terminal screen, return false if
 * the result would fall outside the visible field rect (HUD rows
 * reserved at top and bottom).  Clipping HERE keeps the inner-loop
 * paint helper free of bounds checks. */
static inline bool grid_cell_to_screen(int i, int j,
                                       int term_rows, int term_cols,
                                       int *out_col, int *out_row) {
    int col = grid_i_to_term_col(i, term_cols);
    int row = grid_j_to_term_row(j, term_rows);
    if (col < 0 || col >= term_cols)                          return false;
    if (row < HUD_RESERVED_ROWS_TOP)                          return false;
    if (row >= term_rows - HUD_RESERVED_ROWS_BOTTOM)          return false;
    *out_col = col;
    *out_row = row;
    return true;
}

/* Paint ONE dye cell.  Maps normalised density → glyph+shade via
 * glyph_for_density() (§17); picks the channel-specific colour pair
 * via dye_pair_id() (§4); single mvaddch().  Reference [8] Bourke for
 * the density-to-glyph ramp design. */
static inline void paint_dye_cell(int screen_row, int screen_col,
                                  float density_normalised) {
    GlyphChoice gc      = glyph_for_density(density_normalised);
    int         pair_id = dye_pair_id(g_scene.render.active_channel,
                                       gc.shade_index);
    attron(COLOR_PAIR(pair_id));
    mvaddch(screen_row, screen_col, (chtype)(unsigned char)gc.glyph);
    attroff(COLOR_PAIR(pair_id));
}

/*
 * render_dye_field — paint the live dye density field to ncurses.
 *
 * Pseudocode:
 *   for each interior grid cell (i, j):
 *     rho_norm = normalise_dye_at(i, j)                 (EMA-divided)
 *     if rho_norm < BLANK_THRESHOLD → skip               (let bg show)
 *     (col, row) ← grid_cell_to_screen(i, j, ...)
 *     if out of visible field rect → skip
 *     paint_dye_cell(row, col, rho_norm)
 *
 * READS g_scene.fluid.dye_density and g_scene.render.dye_max_smoothed only.
 * WRITES only ncurses cells — no side effects on the simulation.
 */
static void render_dye_field(int term_rows, int term_cols) {
    for (int j = 1; j <= GRID_SIDE_INNER; j++) {
        for (int i = 1; i <= GRID_SIDE_INNER; i++) {
            float rho_norm = normalise_dye_at(i, j);
            if (rho_norm < DENSITY_GLYPH_BLANK)
                continue;

            int col, row;
            if (!grid_cell_to_screen(i, j, term_rows, term_cols, &col, &row))
                continue;

            paint_dye_cell(row, col, rho_norm);
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

static void hud_paint_status(int term_cols) {
  char buf[160];
  snprintf(buf, sizeof buf,
           " StableFluids  grid:%dx%d  visc:%.2e  dye:%-5s  %s ",
           GRID_SIDE_INNER, GRID_SIDE_INNER, (double)g_scene.fluid.viscosity_kinematic,
           dye_palette_table[g_scene.render.active_channel].name,
           g_scene.sim.paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = term_cols - len;
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  const char *hint = " q:quit  p:pause  r:reset  arrows:wind  d/spc:dye  "
                     "1/2/3:colour  +/-:visc ";
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0, "%s", hint);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §21  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Owns the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the actual back
 *   buffer; this struct is the SOURCE-OF-TRUTH for cell-space
 *   dimensions, refreshed via getmaxyx in screen_init / screen_resize.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_cleanup / screen_present_frame touch ncurses' initscr
 *     / endwin / mvprintw.  Every screen-touching function takes
 *     `Screen *` to make the boundary explicit at the type level.
 *   • Symmetry with the rest of the project — every demo's Screen
 *     uses the same {cols, rows} shape.
 *
 * Render pipeline (one frame)
 *   erase → render_dye_field → hud_paint_status → hud_paint_hint →
 *   wnoutrefresh(stdscr) → doupdate().  Diff-only writes to the
 *   terminal — no flicker.
 *
 * Members
 *   rows   Terminal height in CELLS (getmaxyx).
 *   cols   Terminal width  in CELLS (getmaxyx).
 *
 * Invariants
 *   cols > 0, rows > 0 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize.
 *
 * Reference
 *   [9] Raymond NCURSES-HOWTO §11 — diff-only render with
 *       wnoutrefresh + doupdate.
 */
typedef struct {
    int rows;   /* terminal height in cells (getmaxyx)             */
    int cols;   /* terminal width  in cells (getmaxyx)             */
} Screen;

static void screen_init(Screen *s) {
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

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_present_frame(Screen *s) {
  erase();
  render_dye_field(s->rows, s->cols);
  hud_paint_status(s->cols);
  hud_paint_hint(s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §22  app — main loop + signals + input                                */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter (a 1 ms scheduler hiccup swings the
 *   instantaneous reading by ~6 fps at 60 Hz).  This accumulates
 *   frame_count + elapsed nanoseconds over a 500 ms window and emits
 *   a smoothed `display` value each time the window fills.
 *
 *   Currently NOT shown in the HUD (this demo's HUD reports viscosity
 *   + channel + paused state, not fps).  Kept for symmetry with the
 *   rest of the project — every other file's App has one and a future
 *   HUD tweak can surface it without code churn.
 *
 * Lifecycle
 *   fps_counter_init — zero at startup.
 *   fps_counter_tick — call once per frame with the frame's dt (ns);
 *                      emits a fresh `display` only when the window
 *                      fills (≥ 500 ms accumulated).
 *
 * Members
 *   frame_count   Frames seen in the current window.
 *   window_ns     Nanoseconds accumulated in the current window.
 *   display       Smoothed FPS value (frames per second).
 *
 * Invariants
 *   frame_count ≥ 0, window_ns ≥ 0 between resets.
 *   display ≥ 0 (zero until the first 500 ms window completes).
 *
 * References
 *   Same FpsCounter pattern as flocking.c / slime_mold.c / shepherd.c
 *   / war.c / fluid_sph.c in this project.
 */
typedef struct {
  int     frame_count;
  int64_t window_ns;
  double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt) {
  const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;
  f->frame_count++;
  f->window_ns += dt;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — top-level container; lives in BSS as the single g_app instance.
 *
 * Intent
 *   Signal handlers (on_signal) write to App's sig_atomic_t flags;
 *   the main loop polls them.  This is the standard POSIX "wake the
 *   main loop" pattern.
 *
 * Members
 *   screen        terminal extent + ncurses lifecycle
 *   fps           rolling-window fps estimator (currently unused in HUD)
 *   quit          sig_atomic_t flag cleared by SIGINT / SIGTERM
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 *
 * Note: Scene is NOT embedded in App.  Scene's grid arrays are large
 *   (~6 MB at typical GRID_SIDE) and live in their own file-scope
 *   `g_scene` instance.  The two globals are independent: App holds
 *   the OS-side state (terminal + signals), Scene holds the
 *   simulation state.
 */
typedef struct {
    Screen     screen;
    FpsCounter fps;
    volatile sig_atomic_t quit;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig) {
  if (sig == SIGWINCH) g_app.need_resize = 1;
  else                 g_app.quit        = 1;
}

/* Named action helpers — one per key family.  app_handle_key below
 * becomes a flat "key → named action" dispatcher matching the docblock. */

/* drop_random_dye_blob — inject DYE_BLOB_AMOUNT density at a random
 * interior cell with no initial velocity (the user gets to watch the
 * blob diffuse + advect from rest). */
static void drop_random_dye_blob(void) {
  enum { DYE_BLOB_AMOUNT = 5 };
  int i = rand_inner_cell();
  int j = rand_inner_cell();
  add_source_at(i, j, 0.0f, 0.0f, (float)DYE_BLOB_AMOUNT);
}

/* push_velocity_at_centre — arrow keys inject a unit-magnitude force
 * pulse at the grid centre.  The user "drags" the fluid by repeatedly
 * pressing the arrow they want it to flow in. */
static void push_velocity_at_centre(float force_x, float force_y) {
  enum { ARROW_DYE_PULSE = 1 };
  int centre = GRID_SIDE_INNER / 2;
  add_source_at(centre, centre, force_x, force_y, (float)ARROW_DYE_PULSE);
}

/* toggle_paused — SPACE / p key: flip the gate that
 * emitters_inject + fluid_step check in the main loop. */
static inline void toggle_paused(void) {
  g_scene.sim.paused = !g_scene.sim.paused;
}

/*
 * adjust_viscosity — + / − keys scale ν by VISCOSITY_FACTOR, clamped
 * to [VISCOSITY_MIN, VISCOSITY_MAX].  Multiplicative (not additive)
 * because viscosity spans orders of magnitude — a factor-of-1.5 step
 * gives the same visible change at both ν = 0.001 and ν = 0.1.
 *
 *   dir == +1 → ν ← min(ν · F, MAX)
 *   dir == -1 → ν ← max(ν / F, MIN)
 */
static void adjust_viscosity(int dir) {
  if (dir > 0) {
    g_scene.fluid.viscosity_kinematic *= VISCOSITY_FACTOR;
    if (g_scene.fluid.viscosity_kinematic > VISCOSITY_MAX)
      g_scene.fluid.viscosity_kinematic = VISCOSITY_MAX;
  } else {
    g_scene.fluid.viscosity_kinematic /= VISCOSITY_FACTOR;
    if (g_scene.fluid.viscosity_kinematic < VISCOSITY_MIN)
      g_scene.fluid.viscosity_kinematic = VISCOSITY_MIN;
  }
}

/* set_dye_channel — 1 / 2 / 3 keys pick which palette colours the
 * dye field is rendered with.  Pure render mutation; does not touch
 * sim state. */
static inline void set_dye_channel(int channel) {
  g_scene.render.active_channel = channel;
}

/*
 * app_handle_key — dispatch one keypress.  Returns false on quit.
 *
 *   q / Q / ESC   quit
 *   p / P / SPACE toggle paused
 *   r / R         reset all fields (fluid_reset)
 *   d / D         drop_random_dye_blob (uses ' ' as alias too)
 *   ← → ↑ ↓       push_velocity_at_centre with unit force
 *   + / =         adjust_viscosity ×VISCOSITY_FACTOR (cap MAX)
 *   -             adjust_viscosity ÷VISCOSITY_FACTOR (floor MIN)
 *   1 / 2 / 3     set_dye_channel BLUE / GREEN / RED
 */
static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q': case 'Q': case 27 /* ESC */: return false;

  case 'p': case 'P':           toggle_paused();                  break;
  case 'r': case 'R':           fluid_reset();                    break;

  case ' ': case 'd': case 'D': drop_random_dye_blob();           break;

  case KEY_LEFT:                push_velocity_at_centre(-1, 0);   break;
  case KEY_RIGHT:               push_velocity_at_centre(+1, 0);   break;
  case KEY_UP:                  push_velocity_at_centre(0, -1);   break;
  case KEY_DOWN:                push_velocity_at_centre(0, +1);   break;

  case '+': case '=':           adjust_viscosity(+1);             break;
  case '-':                     adjust_viscosity(-1);             break;

  case '1':                     set_dye_channel(DYE_CHANNEL_BLUE);  break;
  case '2':                     set_dye_channel(DYE_CHANNEL_GREEN); break;
  case '3':                     set_dye_channel(DYE_CHANNEL_RED);   break;

  default: break;
  }
  return true;
}

/*
 * main — fixed-step render loop.
 *
 *   (1) RNG + atexit + signal handlers
 *   (2) bring up Screen + App; prewarm fluid so first frame has dye
 *   (3) seed dye-EMA from the prewarmed state
 *   (4) loop:
 *       (a) drain non-blocking input via app_handle_key
 *       (b) handle pending SIGWINCH
 *       (c) physics tick (skipped if paused)
 *       (d) advance dye-normaliser EMA; render + present
 *       (e) rolling-window fps counter
 *       (f) frame cap: sleep so we don't burn budget
 */
int main(void) {
  /* (1) RNG + atexit + signal handlers */
  srand((unsigned)time(NULL));
  atexit(screen_cleanup);
  signal(SIGINT,   on_signal);
  signal(SIGTERM,  on_signal);
  signal(SIGWINCH, on_signal);

  /* (2) Bring up Screen + App; pre-warm fluid */
  App *app = &g_app;
  fps_counter_init(&app->fps);
  screen_init(&app->screen);
  fluid_reset();

  /* Pre-warm: emitters + physics for N ticks so the first frame
   * has visible dye instead of a blank screen. */
  for (int i = 0; i < PREWARM_TICK_COUNT; i++) {
    emitters_inject();
    fluid_step(DT_DEFAULT);
  }

  /* (3) Seed the EMA from the prewarmed state so the first rendered
   * frame doesn't blow out shades. */
  g_scene.render.dye_max_smoothed = dye_max_per_frame();
  if (g_scene.render.dye_max_smoothed < DYE_MAX_FLOOR)
    g_scene.render.dye_max_smoothed = DYE_MAX_INITIAL;

  /* (4) Loop */
  int64_t prev_ns = clock_now_ns();
  while (!app->quit) {
    int64_t frame_start_ns = clock_now_ns();

    /* (4a) drain input */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch)) {
        app->quit = 1;
        break;
      }
    }

    /* (4b) handle pending SIGWINCH */
    if (app->need_resize) {
      app->need_resize = 0;
      screen_resize(&app->screen);
    }

    /* (4c) physics tick (skipped if paused) */
    if (!g_scene.sim.paused) {
      emitters_inject();
      fluid_step(DT_DEFAULT);
    }

    /* (4d) visualisation prep + render */
    dye_normaliser_advance();
    screen_present_frame(&app->screen);

    /* (4e) rolling-window fps counter */
    int64_t now_ns = clock_now_ns();
    fps_counter_tick(&app->fps, now_ns - prev_ns);
    prev_ns = now_ns;

    /* (4f) frame cap */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
