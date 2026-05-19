/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shallow_water_solver.c — 2-D linearised shallow-water equations
 *
 * DEMO: Solves the LINEARISED 2-D SHALLOW WATER EQUATIONS on the
 *       terminal grid.  Three fields per cell:
 *
 *           h(x, y)   height perturbation (positive = crest, negative
 *                      = trough; rest depth H₀ is implicit)
 *           u(x, y)   horizontal velocity (cells/sec)
 *           v(x, y)   vertical velocity (cells/sec)
 *
 *       Each tick, two coupled updates:
 *
 *           1. velocity update:  ∂u/∂t = -g · ∂h/∂x
 *                                ∂v/∂t = -g · ∂h/∂y
 *           2. height update:    ∂h/∂t = -H₀ · (∂u/∂x + ∂v/∂y)
 *
 *       Pressure gradient drives flow toward LOW height.  Velocity
 *       divergence MOVES water — outflow lowers h, inflow raises it.
 *       Together they produce GRAVITY WAVES at speed c = √(g·H₀).
 *
 *       Four presets:
 *
 *           DAM BREAK     — left half + amplitude, right half −,
 *                            instant dam release.  Bore + rarefaction.
 *           RADIAL DROP   — Gaussian pulse → expanding ring + reflections.
 *           CHANNEL       — narrow gap → diffraction (Huygens).
 *           OBSTACLE      — solid disc → scattering + shadow zone.
 *
 *       Three themes (water / magma / current).  Three boundaries
 *       (wall / open / periodic).  Optional flow-arrow overlay,
 *       optional shoreline tracker (zero-crossing of h).  CFL number
 *       computed and colour-coded in real time.
 *
 * Study alongside:
 *   fluid/navier_stokes.c     — incompressible NS with operator
 *                                splitting + projection.  SWE skip
 *                                projection because the equations
 *                                are simpler (depth-averaged).
 *   fluid/cfl_stability_explorer.c
 *                              — explores the explicit-scheme
 *                                stability bound that this file
 *                                operates within.
 *   fluid/lattice_gas.c        — particle approach to shallow flow.
 *
 * Section map:
 *   §1   config            — every tunable named, no later magic
 *   §2   clock             — monotonic ns timer + sleep
 *   §3   themes            — 3 palettes + names
 *   §4   colors            — pair init + height_attr helper
 *   §5   ramp              — ASCII glyph table + threshold lookup
 *   §6   grid_state        — h, u, v, wall_mask + active dims
 *   §7   boundary          — apply BC (wall / open / periodic)
 *   §8   obstacles         — zero fields inside walls + layout helpers
 *   §9   update_velocity   — forward-difference ∇h → momentum
 *   §10  update_height     — backward-difference ∇·v → continuity
 *   §11  stats             — max-h, avg-vel, wave speed, CFL
 *   §12  swe_step          — one full physics tick
 *   §13  excitation        — drops + dam-break initial states
 *   §14  presets           — 4 scene loaders + table
 *   §15  shoreline         — zero-crossing detection helper
 *   §16  render_field      — glyph + colour + arrows + shoreline
 *   §17  stats_panel       — bottom-left debug overlay
 *   §18  hud               — top status + bottom hint (CLAUDE.md)
 *   §19  scene             — per-frame state + tick wrapper
 *   §20  screen            — ncurses init / cleanup / present
 *   §21  app               — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   s                single-step (paused mode)
 *   r                reset current preset
 *   d                drop a Gaussian pulse at centre
 *   b                re-trigger dam-break initial state
 *   g / G            gravity +/-
 *   a                toggle flow-arrow overlay
 *   l                toggle shoreline (zero-crossing) overlay
 *   o                toggle obstacle visibility (physics still active)
 *   n                cycle boundary condition (wall / open / periodic)
 *   p / P            next / prev preset
 *   t                cycle theme
 *   ] / [            sim Hz +/-
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/shallow_water_solver.c \
 *       -o shallow_water -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Tutorials build a ladder: T1 names the algorithm, T2-T4 derive
 *      the SWE, T5-T7 cover the numerics, T8-T10 the visualisation.
 *   2. §1 config — every constant in one place.
 *   3. §6 grid_state — the three field arrays.  Knowing the data
 *      layout illuminates everything below.
 *   4. §12 swe_step — the algorithm in five calls.
 *   5. §9 update_velocity, §10 update_height — read AFTER tutorials
 *      T2 and T5.  These are the two physics phases.
 *   6. §7 boundary, §8 obstacles — domain edges and walls.
 *   7. §16 render_field — the visualisation orchestrator.
 *   8. §19 scene → §21 app — orchestration.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   height_perturbation[r][c]    h — water height above rest (signed)
 *   velocity_x[r][c]             u — horizontal velocity (cells/sec)
 *   velocity_y[r][c]             v — vertical velocity (cells/sec)
 *   wall_mask[r][c]              true = solid obstacle cell
 *
 *   gravity_acceleration         g — sets wave speed c = √(g·H₀)
 *   bottom_friction_coeff        γ — velocity damping per second
 *   active_boundary_kind         BC_WALL / BC_OPEN / BC_PERIODIC
 *
 *   grid_active_rows / cols      effective dimensions (≤ MAX)
 *   simulation_paused            run/pause toggle
 *   step_request_pending         single-step request from 's' key
 *   simulation_time_seconds      sim time elapsed since last reset
 *
 *   active_preset_index          which row of preset_table is live
 *   active_theme_index           which row of theme_table is live
 *   sim_steps_per_second         physics tick rate (Hz)
 *
 * Background you need
 * ───────────────────
 *   - Vector calculus on a grid: gradient ∂h/∂x, divergence ∂u/∂x +
 *     ∂v/∂y.  T2 reviews these informally.
 *   - Forward Euler integration.  Tutorial T7 covers stability.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - The full nonlinear shallow-water equations.  We linearise.
 *     For dam breaks the linearisation is approximate; the bore
 *     visually still propagates correctly.
 *   - Multigrid solvers, implicit time stepping.  Pure explicit
 *     finite differences.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm    : Linearised 2-D SHALLOW WATER EQUATIONS solved by
 *                EXPLICIT FINITE DIFFERENCES on a colocated grid.
 *                Three fields evolve in lockstep: height h, x-velocity
 *                u, y-velocity v.  Per tick:
 *
 *                  1. update u, v using FORWARD-DIFFERENCE gradients
 *                     of h (the pressure gradient drives flow toward
 *                     low h).
 *                  2. update h using BACKWARD-DIFFERENCE divergence
 *                     of u, v (continuity — outflow lowers h).
 *                  3. apply boundary conditions and zero fields
 *                     inside obstacle cells.
 *
 *                The forward+backward asymmetry creates an IMPLICIT
 *                HALF-CELL STAGGER (T6) that avoids the odd-even
 *                checkerboard instability that plagues purely
 *                centred-difference schemes on colocated grids.
 *
 * Math basis   : SHALLOW WATER EQUATIONS describe flow in a thin
 *                fluid layer where horizontal length scales much
 *                exceed vertical depth.  Depth-averaging the full
 *                Navier-Stokes equations and assuming hydrostatic
 *                vertical pressure yields:
 *
 *                  ∂(Hu)/∂t + ∇·(Hu²) = -gH ∇h          (full)
 *                  ∂H/∂t   + ∇·(Hu)   = 0
 *
 *                with H = H₀ + h (total depth).  For small |h|/H₀
 *                we LINEARISE to:
 *
 *                  ∂u/∂t = -g ∂h/∂x
 *                  ∂v/∂t = -g ∂h/∂y
 *                  ∂h/∂t = -H₀ (∂u/∂x + ∂v/∂y)
 *
 *                These are the WAVE EQUATION in two variables (T4).
 *                Small disturbances propagate at c = √(g·H₀) in
 *                all directions — same equations as light in 2-D
 *                or sound in a thin tube.
 *
 * Performance  : Three 2-D float arrays + one bool array, all in
 *                BSS (no malloc).  Per cell per tick: ~6 multiplies
 *                + 6 adds (velocity update) + 3 subtracts + 2 muls
 *                (height update).  At 200 × 60 = 12000 cells × 60
 *                Hz = 720K cell-updates/sec.  Trivial on any CPU.
 *
 * References
 * ──────────
 *   ── Shallow-water foundations ─────────────────────────────────────
 *   [1] Stoker, J. J. (1957), "Water Waves: The Mathematical Theory
 *       with Applications", Wiley — the classic reference for the
 *       depth-averaged SWE derivation and analytical solutions.
 *   [2] Vreugdenhil, C. B. (1994), "Numerical Methods for Shallow-
 *       Water Flow", Kluwer — standard textbook covering the SWE
 *       discretisations used here.
 *   [3] Saint-Venant, A. J. C. B. de (1871), "Théorie du mouvement
 *       non permanent des eaux", C. R. Acad. Sci. Paris 73 — the
 *       ORIGINAL 1D shallow-water equations.  The 2D version we
 *       solve is the direct generalisation.
 *
 *   ── Numerical methods (finite-volume / staggered grids) ──────────
 *   [4] LeVeque, R. J. (2002), "Finite Volume Methods for Hyperbolic
 *       Problems", CUP — modern numerical treatment; chs. 4-5 cover
 *       the CFL bound (c·dt/dx ≤ 1) that constrains our DT_DEFAULT.
 *   [5] Arakawa, A. & Lamb, V. R. (1977), "Computational Design of
 *       the Basic Dynamical Processes of the UCLA General Circulation
 *       Model", Methods Comput. Phys. 17, pp. 173-265 — the staggered
 *       C-grid layout that places h at cell centres and (u, v) at
 *       cell edges.
 *
 *   ── Graphics applications ────────────────────────────────────────
 *   [6] Foster, N. & Metaxas, D. (1996), "Realistic Animation of
 *       Liquids", GMIP 58 — pre-Stam grid solver, includes SWE for
 *       free-surface waves.
 *   [7] Kass, M. & Miller, G. (1990), "Rapid, Stable Fluid Dynamics
 *       for Computer Graphics", SIGGRAPH '90 — height-field water
 *       surface using a 1D SWE-like scheme; nice intuition for
 *       wave propagation.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *   [8] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the height → glyph ramp.
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Shallow_water_equations
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Water in a thin layer (a pond, a swimming pool, a bathtub, the
 * surface of the ocean) behaves SIMPLY: gravity pulls it down,
 * pressure pushes it sideways from high to low, and wherever
 * water flows OUT of a column the column SHRINKS, wherever water
 * flows IN it GROWS.  Three rules.  Apply them everywhere on a
 * grid every tick.  You get RIPPLES, REFLECTIONS, DIFFRACTION,
 * INTERFERENCE — all the classical wave phenomena, without any
 * "wave equation" anywhere in the code.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a swimming pool with a ball floating at every grid
 * point.  Each ball can BOB UP AND DOWN (the height field) and
 * can DRIFT HORIZONTALLY (the velocity field).
 *
 *   - When a ball is HIGHER than its neighbour, water pushes
 *     toward the neighbour (pressure gradient).  The ball
 *     starts drifting toward the lower side.
 *   - When water DRIFTS AWAY from a ball, the ball SINKS
 *     (less water beneath it).  When water DRIFTS TOWARD it,
 *     the ball RISES.
 *
 * Every tick we update all balls' velocities (from neighbour
 * height differences), then update all balls' heights (from
 * neighbour velocity differences).  A localised disturbance
 * (e.g. a drop) creates an outward-spreading RING of bobbing
 * balls — a GRAVITY WAVE.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   t = 0          t = 1           t = 2               │
 *      │                                                      │
 *      │      ●               ●●●              ●●●●●         │
 *      │     ●●●             ●     ●          ●       ●       │
 *      │      ●               ●●●              ●●●●●         │
 *      │   single             ring is          ring still     │
 *      │   peak               expanding         expanding     │
 *      │                       outward          outward       │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * The simulation BOUNDARIES (T8) decide what happens when the
 * wave reaches the edge:
 *   WALL      — wave bounces back (no flow through wall).
 *   OPEN      — wave slides out of the domain (Neumann copy).
 *   PERIODIC  — wave wraps around (toroidal topology).
 *
 * OBSTACLES (T9) are interior cells where all three fields are
 * zeroed.  Waves reflect off obstacle edges just like off the
 * domain walls.  This gives diffraction (waves curving around
 * obstacles), interference, shadow zones.
 *
 * KEY FORMULAS
 * ────────────
 *
 *   THE LINEARISED 2-D SHALLOW-WATER EQUATIONS:
 *     ∂u/∂t = -g · ∂h/∂x                       (x-momentum)
 *     ∂v/∂t = -g · ∂h/∂y                       (y-momentum)
 *     ∂h/∂t = -H₀ · (∂u/∂x + ∂v/∂y)            (continuity)
 *
 *   WAVE SPEED:
 *     c = √(g · H₀)                            (any direction)
 *
 *   DISCRETE STENCIL — forward gradient + backward divergence:
 *     ∂h/∂x at (r, c) ≈ h[r, c+1] - h[r, c]    (forward)
 *     ∂u/∂x at (r, c) ≈ u[r, c]   - u[r, c-1]  (backward)
 *
 *   FORWARD EULER UPDATE (per cell):
 *     u_new[r, c] = u[r, c] - g · dt · (h[r, c+1] - h[r, c])
 *     v_new[r, c] = v[r, c] - g · dt · (h[r+1, c] - h[r, c])
 *     h_new[r, c] = h[r, c] - H₀ · dt · ((u[r, c]   - u[r, c-1])
 *                                      + (v[r, c]   - v[r-1, c]))
 *
 *   BOTTOM FRICTION (multiplicative damping per tick):
 *     u *= (1 - γ · dt)
 *     v *= (1 - γ · dt)
 *
 *   CFL STABILITY BOUND (2-D explicit SWE):
 *     CFL = c · dt / dx  ≤  1/√2 ≈ 0.707
 *     With dx = 1 (one cell):
 *       CFL = √(g · H₀) / sim_hz
 *     Default g = 400, H₀ = 1, sim_hz = 60:
 *       CFL = 20 / 60 ≈ 0.333  (comfortable)
 *
 *   GAUSSIAN DROP:
 *     h[r, c] += amp · exp(-((c-cx)² + (r-cy)²) / (2 · σ²))
 *
 *   SHORELINE PREDICATE (zero-crossing of h):
 *     IS_SHORELINE(r, c) = (|h[r, c]| < THRESHOLD)
 *                       AND (any neighbour has h > +THRESHOLD)
 *                       AND (any neighbour has h < -THRESHOLD)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - VELOCITY AND HEIGHT MUST BE UPDATED IN SEPARATE LOOPS
 *     (T6).  If you merge them, a velocity update at (r, c)
 *     reads the NEW h value at (r, c+1) instead of the old —
 *     creates a data race that destroys numerical accuracy.
 *   - THE BACKWARD/FORWARD STENCIL ASYMMETRY is INTENTIONAL.
 *     A pure centred-difference scheme on a colocated grid
 *     decouples odd and even cells (the "checkerboard
 *     instability").  Forward + backward couples them.
 *   - CFL HARD LIMIT is 1/√2 in 2-D.  Above that the explicit
 *     scheme amplifies high-frequency modes.  We monitor in
 *     §11 and colour-code the readout (green / yellow / red).
 *   - OBSTACLE BOUNDARIES leak energy.  Zeroing all three
 *     fields inside an obstacle is approximate — a tiny
 *     fraction of incident wave energy is absorbed rather
 *     than reflected.  For most demos this is invisible.
 *   - LINEARISATION breaks at large amplitude.  The dam-break
 *     preset uses ±1.2 amplitude with H₀ = 1, so |h|/H₀ = 1.2
 *     — well outside the linearised regime.  The bore
 *     propagates at the linear wave speed instead of the
 *     true nonlinear speed.  Visually plausible; physically
 *     simplified.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - DAM BREAK preset: at t=0, sharp left/right step in h.
 *     Within 0.5 sec a bore (positive front) propagates right,
 *     a rarefaction (negative front) propagates left.  Both
 *     at speed c = √(g·H₀).
 *   - RADIAL DROP preset: a single peak at centre expands as
 *     a clean circular ring.  With BC_WALL the ring reflects
 *     off all four walls and re-focuses at the origin (sharp
 *     central peak).
 *   - CHANNEL preset: a wave from the left passes through the
 *     gap and DIFFRACTS into a semicircle on the far side —
 *     classic Huygens behaviour.
 *   - OBSTACLE preset: wave from the left scatters off the
 *     central disc.  Visible "shadow zone" behind it; arc
 *     diffraction wrapping around.
 *   - CFL stays green at default parameters.  Press 'g' to
 *     bump gravity past 1000; CFL → yellow; past 1600 → red.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1   What are shallow-water equations?
 *   T2   The three SWE — momentum and continuity term by term
 *   T3   Why "shallow" — depth-averaging and hydrostatic pressure
 *   T4   Wave speed c = √(g·H₀)
 *   T5   Discretisation — forward gradient, backward divergence
 *   T6   The implicit half-cell stagger — avoiding checkerboard
 *   T7   CFL stability for 2-D explicit SWE
 *   T8   Boundary conditions — wall / open / periodic
 *   T9   Obstacles via no-flow zeroing
 *   T10  Visualisation — bands, arrows, shoreline
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT ARE SHALLOW-WATER EQUATIONS?
 * ─────────────────────────────────────
 * Take a swimming pool.  Or a bathtub.  Or the entire OCEAN —
 * yes, the ocean is "shallow" by SWE standards because its
 * depth (~4 km) is tiny compared to wave wavelengths (10s of
 * km for tsunamis, 100s of km for tides).  The SHALLOW WATER
 * APPROXIMATION assumes the fluid is THIN — horizontal length
 * scales much larger than vertical depth — so we can DEPTH-
 * AVERAGE the full 3-D Navier-Stokes equations and end up
 * with a 2-D system.
 *
 * What you GET:
 *   - 2-D fields (one per horizontal direction + one for height).
 *   - Cheap to simulate (no vertical resolution needed).
 *   - Captures gravity waves, tsunamis, tides, river floods,
 *     dam breaks, breaking waves (with non-linear extension),
 *     atmospheric Rossby waves.
 *
 * What you GIVE UP:
 *   - Vertical structure of the flow.  No internal waves, no
 *     stratified layers, no convection cells.
 *   - High-frequency waves where wavelength approaches depth.
 *
 * For this demo we further LINEARISE — assume |h| << H₀ — which
 * makes the equations simpler at the cost of accuracy in
 * high-amplitude scenarios (the dam-break preset is borderline).
 *
 * Real-world examples:
 *   - TSUNAMI propagation across the Pacific.
 *   - WEATHER MODEL atmospheric layers (each layer is a SWE).
 *   - HARBOUR DESIGN — wave reflection off seawalls.
 *   - DAM BREAK and FLOODPLAIN inundation.
 *   - PUDDLE PHYSICS in graphics.
 *   - AURORA dynamics in the magnetosphere.
 *
 * T2  THE THREE SWE — MOMENTUM AND CONTINUITY TERM BY TERM
 * ────────────────────────────────────────────────────────
 * Three coupled PDEs:
 *
 *   ∂u/∂t = -g · ∂h/∂x                       (x-momentum)
 *   ∂v/∂t = -g · ∂h/∂y                       (y-momentum)
 *   ∂h/∂t = -H₀ · (∂u/∂x + ∂v/∂y)            (continuity)
 *
 * Three terms.  Each has a clear physical meaning.
 *
 *   ∂u/∂t = -g · ∂h/∂x:
 *     "Horizontal acceleration is proportional to the slope of
 *     the water surface."
 *     Imagine a sloped surface — water on the high side feels
 *     more pressure at the bottom than water on the low side.
 *     The pressure gradient pushes water from high to low.
 *     Magnitude scales with g (gravity); steeper slope → faster
 *     acceleration.  The minus sign is because positive ∂h/∂x
 *     means height increases going right, so pressure pushes
 *     water LEFT (negative u).
 *
 *   ∂h/∂t = -H₀ · (∂u/∂x + ∂v/∂y):
 *     "Water level changes as net horizontal flow into/out of
 *      the column."
 *     If more water flows OUT of a vertical column than IN,
 *     the column's height drops (dehydrated).  More IN than
 *     OUT → height rises (water piles up).  This is MASS
 *     CONSERVATION.  H₀ is the rest depth — multiplying by it
 *     converts horizontal flow rate (cells/sec) into volume
 *     flow rate per unit width (cells² /sec).
 *
 * The minus signs ensure the equations have the RIGHT FEEDBACK:
 *   - Crests fall (height drops because outflow exceeds inflow
 *     under the pressure gradient).
 *   - Troughs rise (height climbs because inflow exceeds outflow).
 *   - Energy ping-pongs between potential (height) and kinetic
 *     (velocity).  Result: oscillation.  Result: WAVES.
 *
 * T3  WHY "SHALLOW" — DEPTH-AVERAGING AND HYDROSTATIC PRESSURE
 * ────────────────────────────────────────────────────────────
 * Why does the SHALLOW WATER approximation work?  Because under
 * the right conditions, the vertical structure of the flow is
 * BORING and we can throw it away.
 *
 * The two assumptions:
 *
 *   (1) HYDROSTATIC PRESSURE.  In a thin layer with slow
 *       vertical motion, pressure at any depth is just ρgz —
 *       linear from atmospheric at the surface to (ρg·depth) at
 *       the bottom.  This means the horizontal pressure gradient
 *       at any depth is the same as at the surface — depending
 *       only on h, not on z.
 *
 *   (2) UNIFORM HORIZONTAL VELOCITY WITH DEPTH.  Under slow
 *       vertical motion, the horizontal velocity profile through
 *       the depth is approximately constant.  We can REPLACE the
 *       full velocity u(x, y, z, t) with a depth-averaged
 *       u(x, y, t).
 *
 * Together: vertical structure is trivial.  Throw it away.  Get
 * a 2-D system with one variable (h) carrying the surface
 * geometry and two velocity fields (u, v).  All the physical
 * content of the original 3-D flow is preserved in a much
 * cheaper computation.
 *
 * BREAKDOWN: when waves are STEEP (wavelength comparable to
 * depth) or VERTICAL acceleration is significant (capillary
 * waves, breaking waves), the shallow-water approximation
 * fails.  You need the full 3-D Navier-Stokes (or boundary-
 * integral methods for free-surface flow).
 *
 * Tsunamis FIT shallow-water beautifully: even in 4 km of
 * Pacific Ocean, a tsunami's wavelength (200+ km) is 50× the
 * depth.  Capillary ripples on a coffee don't fit at all —
 * wavelength is mm, depth is cm.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Shallow water (good fit):          Deep / steep:    │
 *      │                                                      │
 *      │    ╲────────────╱   surface           ╲╱╲╱╲╱         │
 *      │      ↑     ↑                         ↑  ↑  ↑         │
 *      │      └─────┘  uniform u(z)           variable u(z)   │
 *      │                                                      │
 *      │  λ >> depth                          λ << depth      │
 *      │  hydrostatic OK                      hydrostatic     │
 *      │  depth-averaged OK                    breaks down    │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T4  WAVE SPEED c = √(g · H₀)
 * ────────────────────────────
 * Plug a small disturbance into the linearised SWE.  Take the
 * ANSATZ:
 *
 *     h(x, t) = h₀ · exp(i(kx - ωt))
 *     u(x, t) = u₀ · exp(i(kx - ωt))
 *
 * Substitute into the SWE.  After algebra you get the DISPERSION
 * RELATION:
 *
 *     ω² = g · H₀ · k²
 *     ω/k = ±√(g · H₀)
 *
 * Phase velocity ω/k = √(g · H₀) — INDEPENDENT of k.  All
 * wavelengths travel at the SAME SPEED in shallow water; the
 * medium is NON-DISPERSIVE.  This is why a tsunami stays
 * recognisable across an ocean — different wavelength components
 * don't drift apart.  Compare with deep-water waves where
 * c ∝ √λ — long wavelengths travel faster.
 *
 * Numerically: c = √(g · H₀).  With our H₀ = 1 (normalised) and
 * default g = 400, c = 20 cells/sec.  At sim_hz = 60 the waves
 * advance ~0.33 cells per tick.  Visible motion across a
 * 200-cell domain takes ~10 seconds — comfortable to watch.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   Deep water (dispersive):                           │
 *      │     c = √(g·λ/2π) — long waves are FASTER             │
 *      │                                                      │
 *      │   Shallow water (non-dispersive):                    │
 *      │     c = √(g·H₀)  — all wavelengths travel TOGETHER   │
 *      │                                                      │
 *      │   This is why shallow-water waves are "clean":       │
 *      │   no dispersion, no spreading of pulses over time.   │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T5  DISCRETISATION — FORWARD GRADIENT, BACKWARD DIVERGENCE
 * ──────────────────────────────────────────────────────────
 * Numerically, we need to compute ∂h/∂x and ∂u/∂x on a grid.
 * Three common choices:
 *
 *   FORWARD:    ∂f/∂x ≈ f[c+1] - f[c]
 *   BACKWARD:   ∂f/∂x ≈ f[c]   - f[c-1]
 *   CENTRED:    ∂f/∂x ≈ (f[c+1] - f[c-1]) / 2
 *
 * Centred is the most accurate (O(dx²) vs O(dx) for one-sided)
 * but on a COLOCATED GRID (where u, v, h all live at the same
 * cell centre) it suffers from the CHECKERBOARD INSTABILITY:
 * odd-numbered cells decouple from even-numbered cells, the
 * solution oscillates between two unrelated grids.
 *
 * THE FIX: use FORWARD difference for the height gradient and
 * BACKWARD difference for the velocity divergence:
 *
 *   ∂h/∂x at (r, c) ≈ h[r, c+1] - h[r, c]    (forward)
 *   ∂u/∂x at (r, c) ≈ u[r, c]   - u[r, c-1]  (backward)
 *
 * Why does THIS work?  Because the asymmetry creates an
 * IMPLICIT HALF-CELL STAGGER (T6): u[r, c] effectively lives
 * at the RIGHT EDGE of cell c (between cells c and c+1) instead
 * of the centre.  This is mathematically equivalent to using a
 * STAGGERED GRID (Arakawa C-grid in oceanography) without the
 * code complication of two different cell coordinates.
 *
 * The combination:
 *   forward h-gradient + backward u-divergence
 *   = staggered grid in disguise
 *
 * Same idea works for v and y.
 *
 * T6  THE IMPLICIT HALF-CELL STAGGER — AVOIDING CHECKERBOARD
 * ──────────────────────────────────────────────────────────
 * On a STAGGERED GRID, h lives at cell centres, u lives at
 * vertical edges (between cells), v lives at horizontal edges:
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │     u(0,0)    u(0,1)    u(0,2)                       │
 *      │       │        │        │                            │
 *      │   ────┼─h(0,0)─┼─h(0,1)─┼────                         │
 *      │       │        │        │                            │
 *      │     u(1,0)    u(1,1)    u(1,2)                       │
 *      │       │        │        │                            │
 *      │   ────┼─h(1,0)─┼─h(1,1)─┼────                         │
 *      │                                                      │
 *      │   h at centres, u at vertical edges.                 │
 *      │   "u(r, c)" means "between cells c-1 and c".         │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Centred differences ON the staggered grid become "natural":
 *   ∂h/∂x at u(r, c)'s position = h[r, c] - h[r, c-1]
 *   ∂u/∂x at h(r, c)'s position = u[r, c+1] - u[r, c]
 *
 * Both involve directly-adjacent cells; no skipping of
 * intermediate cells, no checkerboard.
 *
 * IMPLICIT STAGGER trick: keep u, v, h all at cell centres in
 * memory (colocated), but USE the forward/backward asymmetry
 * to PRETEND they're staggered.  When we compute
 * (h[r, c+1] - h[r, c]) we're computing the gradient at the
 * MIDPOINT between c and c+1 — i.e. at a virtual edge.  When
 * we use (u[r, c] - u[r, c-1]) we're computing the divergence
 * at the centre of cell c.  Math: identical to a staggered grid.
 * Code: one set of arrays.
 *
 * Without this trick, you'd either need separate arrays for
 * staggered values (more code) OR live with the checkerboard
 * instability (broken physics).
 *
 * T7  CFL STABILITY FOR 2-D EXPLICIT SWE
 * ──────────────────────────────────────
 * Forward Euler on the wave equation has the COURANT-FRIEDRICHS-
 * LEWY (CFL) stability bound:
 *
 *     CFL = c · dt / dx ≤ CFL_MAX
 *
 * where c is the wave speed.  CFL_MAX depends on dimension:
 *
 *     1-D:  CFL_MAX = 1
 *     2-D:  CFL_MAX = 1/√2 ≈ 0.707
 *     3-D:  CFL_MAX = 1/√3 ≈ 0.577
 *
 * Above CFL_MAX the scheme amplifies short-wavelength modes
 * each tick — high-frequency oscillations grow and the
 * solution explodes within ~10 ticks.
 *
 * For our 2-D solver:
 *     CFL = √(g · H₀) · dt / dx
 *         = √g / sim_hz       (with H₀ = 1, dx = 1, dt = 1/sim_hz)
 *
 * Default: g = 400, sim_hz = 60 → CFL = √400 / 60 ≈ 0.333 ✓
 *
 * If the user pushes gravity to 1600 (max), CFL = √1600 / 60 =
 * 40/60 ≈ 0.667 — CLOSE to the limit, marginally stable.  At
 * gravity 2000 the scheme blows up within seconds.  We monitor
 * CFL in §11 stats and colour-code the readout:
 *
 *     CFL < 0.50         green   (STABLE)
 *     CFL < 0.70         yellow  (MARGINAL)
 *     CFL ≥ 0.70         red     (UNSTABLE)
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   CFL = 0.333    CFL = 0.667    CFL = 0.85           │
 *      │                                                      │
 *      │     stable        margin        ringing → blow-up    │
 *      │     wave runs     fine for      visible high-freq    │
 *      │     cleanly       short runs    noise within seconds │
 *      │                                                      │
 *      │   Always stay under 0.7 for prolonged simulation.    │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T8  BOUNDARY CONDITIONS — WALL / OPEN / PERIODIC
 * ────────────────────────────────────────────────
 * The grid is finite.  Three common ways to handle waves at
 * the edge:
 *
 *   WALL (Dirichlet on normal velocity, Neumann on tangential):
 *     Set u[r, 0] = u[r, cols-1] = 0 → no flow through left/right.
 *     Set v[0, c] = v[rows-1, c] = 0 → no flow through top/bottom.
 *     Other fields use Neumann (mirror) so gradients vanish.
 *     Effect: WAVES REFLECT off the walls.  Energy is conserved.
 *
 *   OPEN (Neumann on all fields):
 *     Boundary cells copy from their nearest interior neighbour.
 *     Effect: waves slide out of the domain (approximately).  Some
 *     energy still reflects (this is a simple absorbing
 *     boundary; real ABC's are more sophisticated).
 *
 *   PERIODIC (toroidal wrap):
 *     Boundary cells copy from the OPPOSITE side.  Domain
 *     becomes a torus.  Effect: waves wrap around continuously.
 *     No reflection, no leakage.
 *
 * The user toggles between these with 'n'.  Watch the radial
 * drop preset under each:
 *   WALL      — circular ring expands, hits walls, reflects,
 *                 re-focuses at centre.
 *   OPEN      — circular ring expands, fades at the edges.
 *   PERIODIC  — circular ring expands, wraps around, eventually
 *                 returns from "the other side."  Curious effect!
 *
 * T9  OBSTACLES VIA NO-FLOW ZEROING
 * ─────────────────────────────────
 * Interior obstacles (a buoy, a piling, a submerged disc) need
 * to BLOCK FLOW.  Implementation: a separate boolean array
 * wall_mask[r][c]; after every velocity and height update,
 * we ZERO ALL THREE FIELDS at obstacle cells:
 *
 *     for each obstacle cell (r, c):
 *         h[r, c] = u[r, c] = v[r, c] = 0
 *
 * Effect: an obstacle cell is INDISTINGUISHABLE from a wall
 * boundary in its local update — it has zero velocity and
 * zero height perturbation.  Waves arriving at an obstacle
 * cell's neighbours see it as a fixed "rest" cell, just like
 * a wall.  Reflection happens for free.
 *
 * Trade-offs:
 *   - One-cell-thick walls can DIFFRACT (waves leak around the
 *     corner).  Two-cell-thick walls are crisper.
 *   - Zeroing introduces a tiny energy loss at the obstacle
 *     edge (real reflections preserve all incident energy;
 *     this method absorbs a few percent).  Not visible for
 *     short demos.
 *   - Curved obstacles are stair-stepped on a square grid.
 *     Diffraction artefacts at corners are visible if you
 *     look closely.
 *
 * The CHANNEL preset uses two horizontal walls with a gap to
 * demonstrate Huygens diffraction.  The OBSTACLE preset uses
 * a circular disc to show scattering and shadow zones.
 *
 * T10 VISUALISATION — BANDS, ARROWS, SHORELINE
 * ────────────────────────────────────────────
 * Three independent visual channels carry information about
 * the fluid state:
 *
 *   GLYPH     density of the ASCII character represents |h|.
 *             Ramp:  '.', ':', '+', 'x', '*', 'X', '#', '@'
 *             Sparse → dense as |h| grows.
 *
 *   COLOUR    sign of h is encoded as palette choice.
 *             Crests (h > 0) get the WARM ramp (red/yellow/etc.)
 *             Troughs (h < 0) get the COOL ramp (blue/violet/etc.)
 *             User cycles 3 themes (water / magma / current)
 *             — same warm/cool semantics, different palettes.
 *
 *   ARROWS    8-direction ASCII arrows show velocity DIRECTION
 *             at cells where speed > threshold.
 *             Replaces the height glyph at fast cells (the height
 *             colour is preserved so depth info isn't lost).
 *
 *   SHORELINE bright cyan '~' marks cells straddling a zero-
 *             crossing of h — i.e. where the WATERLINE passes
 *             through.  Useful for tracking wavefront position
 *             when the amplitude is too low for the height ramp
 *             to register.
 *
 * The combination gives DENSE INFORMATION per cell: brightness
 * encodes amplitude, colour encodes sign, optional overlays
 * add velocity and zero-crossing detail.  Same data-vis
 * principle as fluid/nuke.c (T10 there).
 *
 * Each overlay can be toggled independently:
 *   'a'  toggle arrows
 *   'l'  toggle shoreline
 * Letting the user pick the channel set that matches their
 * current question — flow direction, wave position, or both.
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

#define SIM_HZ_DEFAULT 60
#define SIM_HZ_MIN 5
#define SIM_HZ_MAX 120
#define SIM_HZ_STEP 5

#define RENDER_FPS_CAP 60
#define FPS_RECOMPUTE_MS 500

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(hz) (NS_PER_SEC / (hz))

/* §1.2 — grid size limits. */

#define GRID_COLS_MAX 300
#define GRID_ROWS_MAX 100

/* §1.3 — physics. */

/* Rest depth H₀ (normalised; wave speed driven by gravity only). */
#define REST_DEPTH 1.0f

/* Gravity — sets wave speed: c = √(g · H₀) = √g (with H₀ = 1).
 * Default 400 → c = 20 cells/sec, CFL ≈ 0.33 at 60 Hz. */
#define GRAVITY_DEFAULT 400.0f
#define GRAVITY_MIN 25.0f
#define GRAVITY_MAX 1600.0f
#define GRAVITY_STEP 100.0f

/* Velocity damping (mimics bottom friction). */
#define BOTTOM_FRICTION_DEFAULT 0.004f
#define BOTTOM_FRICTION_MIN 0.000f
#define BOTTOM_FRICTION_MAX 0.060f

/* §1.4 — excitation parameters. */

#define DROP_HEIGHT_AMPLITUDE 1.4f
#define DROP_GAUSSIAN_RADIUS 3.0f
#define DAM_BREAK_AMPLITUDE 1.2f

/* §1.5 — visualisation thresholds. */

/* Display |h| > this is clipped to full ramp. */
#define DISPLAY_HEIGHT_RANGE 1.8f

/* Velocity speed below this skips arrow drawing. */
#define ARROW_SPEED_THRESHOLD 0.05f

/* |h| below this is "near zero" — used by shoreline detection. */
#define SHORELINE_HEIGHT_THRESHOLD 0.04f

/* CFL band thresholds for stability colour-coding (T7). */
#define CFL_STABLE_LIMIT 0.50f
#define CFL_MARGINAL_LIMIT 0.70f

/* §1.6 — boundary condition tags. */

enum {
  BC_WALL = 0,     /* reflecting walls (no normal flow)       */
  BC_OPEN = 1,     /* Neumann copy (waves slide out)          */
  BC_PERIODIC = 2, /* toroidal wrap                           */
  BC_COUNT,
};

static const char *bc_name_table[BC_COUNT] = {"wall    ", "open    ",
                                              "periodic"};

/* §1.7 — preset IDs. */

enum {
  PRESET_DAM_BREAK = 0,
  PRESET_RADIAL_DROP = 1,
  PRESET_CHANNEL = 2,
  PRESET_OBSTACLE = 3,
  PRESET_COUNT,
};

static const char *preset_name_table[PRESET_COUNT] = {
    "dam_break  ", "radial_drop", "channel    ", "obstacle   "};

/* §1.8 — ramp slot count + theme count. */

#define RAMP_SLOT_COUNT 9 /* must match arrays in §3 + §5 */
#define THEME_COUNT 3

/* §1.9 — colour pair IDs.
 *   POS(t, i) and NEG(t, i) span the full 3 × 9 grid for crest/trough
 *   per theme.  Then SHORELINE / OBSTACLE / HUD / HINT trail. */

#define PAIR_POS(theme, slot) (1 + (theme) * (RAMP_SLOT_COUNT * 2) + (slot))
#define PAIR_NEG(theme, slot)                                                  \
  (1 + (theme) * (RAMP_SLOT_COUNT * 2) + RAMP_SLOT_COUNT + (slot))
#define PAIR_SHORELINE (1 + THEME_COUNT * (RAMP_SLOT_COUNT * 2))
#define PAIR_OBSTACLE (PAIR_SHORELINE + 1)
#define PAIR_HUD (PAIR_OBSTACLE + 1)
#define PAIR_HINT (PAIR_HUD + 1)

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
/* §3  themes — 3 palettes + names                                       */
/* ===================================================================== */
/*
 * Each theme has FOUR ramps:
 *   pos_256[]  positive (crest) colours, 256-mode
 *   neg_256[]  negative (trough) colours, 256-mode
 *   pos_8[]    positive, 8-colour fallback
 *   neg_8[]    negative, 8-colour fallback
 *
 * Slot 0 = sparsest, slot RAMP_SLOT_COUNT-1 = densest.
 */

/*
 * Theme — diverging palette for one of N named looks.
 *
 * Intent
 *   Surface displacement η = h − h₀ is signed: crests (η > 0) lift
 *   above mean depth, troughs (η < 0) sink below.  A theme is a
 *   DIVERGING palette: two parallel ramps (positive / negative) so
 *   the eye reads crests and troughs as opposed hues at a glance.
 *
 *   Each ramp has RAMP_SLOT_COUNT tiers; slot 0 = sparsest (faint
 *   surface ripple), slot N-1 = densest (largest crest/trough).
 *
 * Why FOUR ramps (pos + neg, each with 256 and 8 variants)
 *   pos_256 / neg_256 are the preferred 256-cube indices used when
 *   COLORS >= 256.  pos_8 / neg_8 are the 8-colour fallbacks.
 *   Picking by terminal capability at init keeps the renderer hot
 *   path identical regardless.
 *
 * Why "diverging" rather than "monotone"
 *   Monotone ramps lose sign — they'd colour crests and troughs the
 *   same, which destroys the visual asymmetry that makes wave
 *   propagation legible.  Diverging palettes are the standard
 *   choice for signed scalar fields; see [8] Bourke and any
 *   ColorBrewer reference.
 *
 * Reference [9] Raymond NCURSES HOWTO §6 — init_pair semantics.
 */
typedef struct {
    const char *display_name;          /* short HUD label              */
    short       pos_256[RAMP_SLOT_COUNT];   /* crest ramp, 256-cube    */
    short       neg_256[RAMP_SLOT_COUNT];   /* trough ramp, 256-cube   */
    short       pos_8  [RAMP_SLOT_COUNT];   /* crest ramp, 8-colour    */
    short       neg_8  [RAMP_SLOT_COUNT];   /* trough ramp, 8-colour   */
} Theme;

static const Theme theme_table[THEME_COUNT] = {
    /* 0  WATER — cyan / white crests, navy troughs. */
    {
        "water  ",
        /* pos: dark teal → cyan → white */
        {31, 37, 44, 51, 87, 123, 159, 195, 231},
        /* neg: navy → indigo */
        {18, 19, 20, 21, 25, 26, 27, 31, 39},
        {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
         COLOR_BLUE, COLOR_CYAN, COLOR_CYAN},
    },
    /* 1  MAGMA — red / yellow crests, dark violet troughs. */
    {
        "magma  ",
        /* pos: dark orange → amber → yellow → white */
        {88, 130, 166, 202, 208, 214, 220, 226, 231},
        /* neg: dark violet → indigo → blue */
        {53, 54, 55, 56, 57, 93, 99, 135, 141},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE,
         COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN},
    },
    /* 2  CURRENT — green / white crests, dark teal troughs. */
    {
        "current",
        /* pos: dark green → lime → white */
        {28, 34, 40, 46, 82, 118, 154, 191, 231},
        /* neg: dark teal */
        {22, 23, 29, 30, 36, 37, 44, 45, 51},
        {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN,
         COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN},
    },
};

/* ===================================================================== */
/* §4  colors — pair init + height_attr helper                           */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

static void colors_init(void) {
  start_color();
  use_default_colors();
  terminal_has_256_colours = (COLORS >= 256);

  for (int t = 0; t < THEME_COUNT; t++) {
    for (int s = 0; s < RAMP_SLOT_COUNT; s++) {
      short pos = terminal_has_256_colours ? theme_table[t].pos_256[s]
                                           : theme_table[t].pos_8[s];
      short neg = terminal_has_256_colours ? theme_table[t].neg_256[s]
                                           : theme_table[t].neg_8[s];
      init_pair((short)PAIR_POS(t, s), pos, -1);
      init_pair((short)PAIR_NEG(t, s), neg, -1);
    }
  }

  /* Shoreline + obstacle. */
  if (terminal_has_256_colours) {
    init_pair(PAIR_SHORELINE, 51, -1); /* bright cyan  */
    init_pair(PAIR_OBSTACLE, 244, -1); /* mid grey     */
  } else {
    init_pair(PAIR_SHORELINE, COLOR_CYAN, -1);
    init_pair(PAIR_OBSTACLE, COLOR_WHITE, -1);
  }

  /* HUD pairs — bright + bold per CLAUDE.md. */
  if (terminal_has_256_colours) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static attr_t height_pair_attr(int theme_index, bool positive, int slot) {
  int pair =
      positive ? PAIR_POS(theme_index, slot) : PAIR_NEG(theme_index, slot);
  attr_t a = COLOR_PAIR(pair);
  if (slot >= RAMP_SLOT_COUNT - 2)
    a |= A_BOLD; /* extra punch on peaks */
  return a;
}

/* ===================================================================== */
/* §5  ramp — ASCII glyph table + threshold lookup                       */
/* ===================================================================== */
/*
 * Sparsest → densest character ramp.  Both crests and troughs use
 * the same SHAPES; sign is encoded by colour (warm vs cool).
 *
 * Index 0 reserved for "no glyph" (background); indices 1-8 paint.
 */

static const char ramp_glyph_table[RAMP_SLOT_COUNT] = {
    ' ', /* 0 — at rest                        */
    '.', /* 1 — tiny ripple                    */
    ':', /* 2 — gentle swell                   */
    '+', /* 3 — moderate wave                  */
    'x', /* 4 — strong wave                    */
    '*', /* 5 — intense surge                  */
    'X', /* 6 — near-crest                     */
    '#', /* 7 — high crest / deep trough       */
    '@', /* 8 — peak (clipped at DISPLAY_RANGE) */
};

static const float ramp_threshold_table[RAMP_SLOT_COUNT] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f, 0.500f, 0.660f, 0.820f, 0.940f,
};

/* normalised_to_slot — map normalised |h|/range ∈ [0, 1] (gamma 2.2)
 * to a ramp slot index 0..RAMP_SLOT_COUNT-1. */
static int normalised_height_to_slot(float normalised) {
  if (normalised <= 0.0f)
    return 0;
  if (normalised >= 1.0f)
    return RAMP_SLOT_COUNT - 1;
  float gamma_corrected = powf(normalised, 1.0f / 2.2f);
  for (int i = RAMP_SLOT_COUNT - 1; i >= 0; i--)
    if (gamma_corrected >= ramp_threshold_table[i])
      return i;
  return 0;
}

/* ===================================================================== */
/* §6  grid_state — h, u, v, wall_mask + active dims                     */
/* ===================================================================== */
/*
 * Three float fields (h, u, v) and one bool mask — all static 2-D
 * arrays in BSS.  Total ~390 KB.  No malloc.
 *
 * Index convention: [row][col].  row 0 = top of screen.
 */

static float height_perturbation[GRID_ROWS_MAX][GRID_COLS_MAX];
static float velocity_x[GRID_ROWS_MAX][GRID_COLS_MAX];
static float velocity_y[GRID_ROWS_MAX][GRID_COLS_MAX];
static bool wall_mask[GRID_ROWS_MAX][GRID_COLS_MAX];

static int grid_active_rows = 0;
static int grid_active_cols = 0;

/* Zero ALL three fluid fields (does NOT touch wall_mask). */
static void grid_zero_fluid_fields(void) {
  for (int r = 0; r < grid_active_rows; r++) {
    memset(height_perturbation[r], 0, (size_t)grid_active_cols * sizeof(float));
    memset(velocity_x[r], 0, (size_t)grid_active_cols * sizeof(float));
    memset(velocity_y[r], 0, (size_t)grid_active_cols * sizeof(float));
  }
}

static void grid_clear_walls(void) {
  for (int r = 0; r < grid_active_rows; r++)
    memset(wall_mask[r], 0, (size_t)grid_active_cols * sizeof(bool));
}

/* ===================================================================== */
/* §7  boundary — apply BC (wall / open / periodic) (T8)                 */
/* ===================================================================== */
/*
 * Called after every velocity and height update.  Three BC kinds:
 *
 *   BC_WALL      Normal velocity = 0; tangential and h mirror.
 *                Energy-conserving reflection.
 *   BC_OPEN      All fields copy from the nearest interior cell.
 *                Rough Neumann; some reflection.
 *   BC_PERIODIC  All fields copy from the OPPOSITE side.  Torus.
 */

/* Energy-conserving REFLECTION BC on the top/bottom walls:
 * normal velocity (v_y) is forced to ZERO, tangential velocity (v_x)
 * and surface height MIRROR the inner cells.  Mathematically equivalent
 * to placing a phantom cell across the wall with opposite-sign normal
 * velocity — gives a perfect bounce.  [1] Stoker §6.  */
static inline void bc_wall_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        velocity_y         [0       ][c] = 0.0f;
        velocity_y         [rows - 1][c] = 0.0f;
        velocity_x         [0       ][c] = velocity_x         [1       ][c];
        velocity_x         [rows - 1][c] = velocity_x         [rows - 2][c];
        height_perturbation[0       ][c] = height_perturbation[1       ][c];
        height_perturbation[rows - 1][c] = height_perturbation[rows - 2][c];
    }
}

/* Energy-conserving REFLECTION BC on the left/right walls: same idea
 * as the top/bottom helper but with v_x as the normal component. */
static inline void bc_wall_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        velocity_x         [r][0       ] = 0.0f;
        velocity_x         [r][cols - 1] = 0.0f;
        velocity_y         [r][0       ] = velocity_y         [r][1       ];
        velocity_y         [r][cols - 1] = velocity_y         [r][cols - 2];
        height_perturbation[r][0       ] = height_perturbation[r][1       ];
        height_perturbation[r][cols - 1] = height_perturbation[r][cols - 2];
    }
}

/* OPEN (Neumann zero-gradient) BC on top/bottom: every field copied
 * from the nearest interior cell.  Crude radiation BC — some energy
 * leaks out, some still reflects.  Good enough for the visual demo. */
static inline void bc_open_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        height_perturbation[0       ][c] = height_perturbation[1       ][c];
        velocity_x         [0       ][c] = velocity_x         [1       ][c];
        velocity_y         [0       ][c] = velocity_y         [1       ][c];
        height_perturbation[rows - 1][c] = height_perturbation[rows - 2][c];
        velocity_x         [rows - 1][c] = velocity_x         [rows - 2][c];
        velocity_y         [rows - 1][c] = velocity_y         [rows - 2][c];
    }
}

/* OPEN BC on left/right — symmetric to top/bottom. */
static inline void bc_open_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        height_perturbation[r][0       ] = height_perturbation[r][1       ];
        velocity_x         [r][0       ] = velocity_x         [r][1       ];
        velocity_y         [r][0       ] = velocity_y         [r][1       ];
        height_perturbation[r][cols - 1] = height_perturbation[r][cols - 2];
        velocity_x         [r][cols - 1] = velocity_x         [r][cols - 2];
        velocity_y         [r][cols - 1] = velocity_y         [r][cols - 2];
    }
}

/* PERIODIC (wrap-around) BC on top/bottom: each border row copies
 * from the OPPOSITE side's interior row, making the grid a torus
 * vertically.  No reflection, no energy loss. */
static inline void bc_periodic_top_bottom(int rows, int cols) {
    for (int c = 0; c < cols; c++) {
        height_perturbation[0       ][c] = height_perturbation[rows - 2][c];
        velocity_x         [0       ][c] = velocity_x         [rows - 2][c];
        velocity_y         [0       ][c] = velocity_y         [rows - 2][c];
        height_perturbation[rows - 1][c] = height_perturbation[1       ][c];
        velocity_x         [rows - 1][c] = velocity_x         [1       ][c];
        velocity_y         [rows - 1][c] = velocity_y         [1       ][c];
    }
}

/* PERIODIC BC on left/right — symmetric to top/bottom. */
static inline void bc_periodic_left_right(int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        height_perturbation[r][0       ] = height_perturbation[r][cols - 2];
        velocity_x         [r][0       ] = velocity_x         [r][cols - 2];
        velocity_y         [r][0       ] = velocity_y         [r][cols - 2];
        height_perturbation[r][cols - 1] = height_perturbation[r][1       ];
        velocity_x         [r][cols - 1] = velocity_x         [r][1       ];
        velocity_y         [r][cols - 1] = velocity_y         [r][1       ];
    }
}

/*
 * apply_boundary — dispatch the right BC helper for the active kind.
 *
 * Pseudocode:
 *   switch boundary_kind:
 *     BC_WALL     : bc_wall_top_bottom     + bc_wall_left_right
 *     BC_OPEN     : bc_open_top_bottom     + bc_open_left_right
 *     BC_PERIODIC : bc_periodic_top_bottom + bc_periodic_left_right
 *
 * Called after every velocity AND height update so the next pass's
 * neighbour reads automatically satisfy the chosen condition.
 */
static void apply_boundary(int boundary_kind) {
    int rows = grid_active_rows;
    int cols = grid_active_cols;

    switch (boundary_kind) {
        case BC_WALL:     bc_wall_top_bottom    (rows, cols);
                          bc_wall_left_right    (rows, cols); break;
        case BC_OPEN:     bc_open_top_bottom    (rows, cols);
                          bc_open_left_right    (rows, cols); break;
        case BC_PERIODIC: bc_periodic_top_bottom(rows, cols);
                          bc_periodic_left_right(rows, cols); break;
        default:          break;
    }
}

/* ===================================================================== */
/* §8  obstacles — zero fields inside walls + layout helpers (T9)        */
/* ===================================================================== */

/* zero all three fields inside obstacle cells.  Called after every
 * velocity/height update so obstacles act as fixed reflectors. */
static void zero_fields_in_walls(void) {
  for (int r = 0; r < grid_active_rows; r++)
    for (int c = 0; c < grid_active_cols; c++)
      if (wall_mask[r][c]) {
        height_perturbation[r][c] = 0.0f;
        velocity_x[r][c] = 0.0f;
        velocity_y[r][c] = 0.0f;
      }
}

/* obstacle_build_channel — two horizontal walls forming a narrow
 * east-west channel (gap in the centre).  Used by PRESET_CHANNEL. */
static void obstacle_build_channel(void) {
  int wall_top = grid_active_rows / 3;
  int wall_bottom = (grid_active_rows * 2) / 3;
  int left_col = grid_active_cols / 4;
  int right_col = (grid_active_cols * 3) / 4;

  for (int r = 0; r < grid_active_rows; r++) {
    if (r < wall_top || r > wall_bottom) {
      for (int c = left_col; c <= right_col; c++)
        wall_mask[r][c] = true;
    }
  }
}

/* obstacle_build_circle — solid circular disc at domain centre.
 * Used by PRESET_OBSTACLE.  Radius = 12% of smaller dimension. */
static void obstacle_build_circle(void) {
  float cx = (float)(grid_active_cols - 1) * 0.5f;
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float radius = (grid_active_rows < grid_active_cols ? grid_active_rows
                                                      : grid_active_cols) *
                 0.12f;
  float radius_squared = radius * radius;

  for (int r = 0; r < grid_active_rows; r++) {
    for (int c = 0; c < grid_active_cols; c++) {
      float dx = (float)c - cx;
      float dy = (float)r - cy;
      if (dx * dx + dy * dy < radius_squared)
        wall_mask[r][c] = true;
    }
  }
}

/* ===================================================================== */
/* §9  update_velocity — forward-difference ∇h → momentum (T2/T5)        */
/* ===================================================================== */
/*
 *   u_new[r, c] = u[r, c] - g · dt · (h[r, c+1] - h[r, c])
 *   v_new[r, c] = v[r, c] - g · dt · (h[r+1, c] - h[r, c])
 *
 * Plus multiplicative damping for bottom friction.  Reads h
 * (unchanged this tick), writes u and v in place.  Skips obstacle
 * cells.
 */
/* Forward-difference height gradient at cell (r, c):
 *   ∂h/∂x ≈ h[r, c+1] − h[r, c]
 *   ∂h/∂y ≈ h[r+1, c] − h[r, c]
 *
 * "Forward" because the next cell minus this cell; in the SAINT-VENANT
 * SWE [3] the momentum equation reads ∂u/∂t = −g·∇h, so the gradient
 * points UPHILL and the minus sign in update_velocity turns it into a
 * DOWNHILL push.  Refs [4] LeVeque §4. */
static inline void forward_height_gradient_at(int r, int c,
                                               float *out_dh_dx,
                                               float *out_dh_dy) {
    *out_dh_dx = height_perturbation[r    ][c + 1] - height_perturbation[r][c];
    *out_dh_dy = height_perturbation[r + 1][c    ] - height_perturbation[r][c];
}

/* Per-step damping factor for bottom friction.  Implements
 *   v_new = v · (1 − c_f · Δt)
 * which is the explicit-Euler form of the linearised Manning drag.
 * Clamped to [0, 1] so a too-large c_f·dt doesn't flip velocities. */
static inline float bottom_friction_damping_factor(float bottom_friction_coeff,
                                                    float dt_seconds) {
    float damping = 1.0f - bottom_friction_coeff * dt_seconds;
    return damping < 0.0f ? 0.0f : damping;
}

/* One cell's full velocity update: subtract g·dt·∇h (momentum
 * equation), then multiply by the friction damping factor.  Reads h
 * (unchanged this tick), writes (u, v) in place.  Skips obstacle
 * cells — they stay at zero velocity via zero_fields_in_walls. */
static inline void update_velocity_at_cell(int r, int c,
                                            float gravity_acceleration,
                                            float dt_seconds,
                                            float damping_factor) {
    if (wall_mask[r][c]) return;
    float dh_dx, dh_dy;
    forward_height_gradient_at(r, c, &dh_dx, &dh_dy);
    velocity_x[r][c] -= gravity_acceleration * dt_seconds * dh_dx;
    velocity_y[r][c] -= gravity_acceleration * dt_seconds * dh_dy;
    velocity_x[r][c] *= damping_factor;
    velocity_y[r][c] *= damping_factor;
}

/*
 * update_velocity — Saint-Venant momentum equation:
 *
 *   ∂u/∂t = −g·∂h/∂x  −  c_f·u
 *   ∂v/∂t = −g·∂h/∂y  −  c_f·v
 *
 * Pseudocode:
 *   damping = bottom_friction_damping_factor(c_f, dt)
 *   for each interior cell (r, c):
 *     update_velocity_at_cell(r, c, g, dt, damping)
 *
 * Reads h (unchanged), writes (u, v).  Refs [3] Saint-Venant 1871;
 * [4] LeVeque ch. 4 for the CFL bound c·dt/dx ≤ 1.
 */
static void update_velocity(float gravity_acceleration,
                             float bottom_friction_coeff, float dt_seconds) {
    float damping_factor = bottom_friction_damping_factor(bottom_friction_coeff,
                                                          dt_seconds);
    int   rows = grid_active_rows;
    int   cols = grid_active_cols;

    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            update_velocity_at_cell(r, c, gravity_acceleration, dt_seconds,
                                     damping_factor);
}

/* ===================================================================== */
/* §10  update_height — backward-difference ∇·v → continuity (T2/T5)     */
/* ===================================================================== */
/*
 *   h_new[r, c] = h[r, c] - H₀ · dt · ((u[r, c]   - u[r, c-1])
 *                                    + (v[r, c]   - v[r-1, c]))
 *
 * Reads u and v (just updated by §9), writes h in place.  Skips
 * obstacle cells.
 *
 * MUST be a separate loop from update_velocity — see T6.
 */
/* Backward-difference velocity divergence at cell (r, c):
 *   ∇·v ≈ (u[r, c] − u[r, c-1]) + (v[r, c] − v[r-1, c])
 *
 * "Backward" because this cell minus the previous cell — pairs with
 * the FORWARD differences in update_velocity to form a STAGGERED
 * grid that gives second-order accuracy [5] Arakawa & Lamb 1977. */
static inline float backward_velocity_divergence_at(int r, int c) {
    float divergence_x = velocity_x[r    ][c] - velocity_x[r    ][c - 1];
    float divergence_y = velocity_y[r    ][c] - velocity_y[r - 1][c    ];
    return divergence_x + divergence_y;
}

/* One cell's height update from the SWE continuity equation:
 *
 *   ∂h/∂t = −H₀ · ∇·v
 *
 * H₀ is the REST depth (the linearised form assumes |h| ≪ H₀).
 * Reads (u, v) just updated by update_velocity, writes h in place. */
static inline void update_height_at_cell(int r, int c, float dt_seconds) {
    if (wall_mask[r][c]) return;
    float divergence = backward_velocity_divergence_at(r, c);
    height_perturbation[r][c] -= REST_DEPTH * divergence * dt_seconds;
}

/*
 * update_height — Saint-Venant continuity (mass conservation):
 *
 *   ∂h/∂t = −H₀ · ∇·v
 *
 * Pseudocode:
 *   for each interior cell (r, c):
 *     update_height_at_cell(r, c, dt)
 *
 * MUST be a SEPARATE LOOP after update_velocity — the forward/backward
 * staggered scheme requires velocity to be fully updated before h is
 * read for the next velocity step.  Refs [3] Saint-Venant 1871; [5]
 * Arakawa & Lamb 1977 for the C-grid staggering.
 */
static void update_height(float dt_seconds) {
    int rows = grid_active_rows;
    int cols = grid_active_cols;

    for (int r = 1; r < rows - 1; r++)
        for (int c = 1; c < cols - 1; c++)
            update_height_at_cell(r, c, dt_seconds);
}

/* ===================================================================== */
/* §11  stats — max-h, avg-vel, wave speed, CFL                          */
/* ===================================================================== */

/*
 * SimStats — per-frame snapshot of physical-state diagnostics.
 *
 * Intent
 *   The HUD shows live numbers for the wave's energy + speed + stability
 *   margin.  scan_stats() computes all four in ONE pass over the field
 *   so the HUD reads a flattened snapshot rather than re-scanning the
 *   grid in each paint helper.
 *
 * Why these four metrics
 *   - max_abs_height  → "how big are the waves" (most useful visual)
 *   - average_speed   → "how much energy is flowing right now"
 *   - wave_speed      → √(g·h) — the SWE characteristic speed; sets
 *                       the upper bound on dt via CFL.  [4] LeVeque §4.
 *   - cfl_number      → c·dt/dx — when this approaches 1 the scheme is
 *                       at its stability limit; the HUD colours it
 *                       red so the user knows when to back off dt.
 *
 * Lifetime
 *   Built once per frame in the stats scan, passed by const pointer to
 *   the HUD painters.  Not part of the simulation state itself —
 *   discarded at end of frame.
 *
 * Reference [4] LeVeque ch. 4 for the CFL bound on hyperbolic schemes.
 */
typedef struct {
    float max_abs_height;   /* max |η| over the grid this frame      */
    float average_speed;    /* mean |v| over the grid                 */
    float wave_speed;       /* c = √(g·h_mean)                        */
    float cfl_number;       /* c·dt/dx — stability indicator          */
} SimStats;

/* |v| at one cell.  Used by the stats scan to accumulate average
 * flow speed and by the HUD for the "energy is moving" readout. */
static inline float velocity_magnitude_at(int r, int c) {
    return sqrtf(velocity_x[r][c] * velocity_x[r][c] +
                 velocity_y[r][c] * velocity_y[r][c]);
}

/* Single-pass scan of the interior cells for max |η| and average |v|.
 * Output parameters are filled with the running totals; wall cells
 * are skipped (they're forced to zero by zero_fields_in_walls and
 * would bias the average toward zero). */
static inline void scan_max_height_and_average_speed(float *out_max_h,
                                                      float *out_avg_speed) {
    float  max_h     = 0.0f;
    double speed_sum = 0.0;
    int    counted   = 0;

    int rows = grid_active_rows;
    int cols = grid_active_cols;
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (wall_mask[r][c]) continue;
            float ah = fabsf(height_perturbation[r][c]);
            if (ah > max_h) max_h = ah;
            speed_sum += (double)velocity_magnitude_at(r, c);
            counted++;
        }
    }

    *out_max_h     = max_h;
    *out_avg_speed = (counted > 0) ? (float)(speed_sum / counted) : 0.0f;
}

/* The SWE CHARACTERISTIC SPEED c = √(g·H₀) — the speed at which
 * small surface perturbations propagate.  Sets the upper bound on
 * the stable timestep via CFL.  Refs [1] Stoker §2.4, [4] LeVeque §3. */
static inline float swe_wave_speed(float gravity_acceleration) {
    return sqrtf(gravity_acceleration * REST_DEPTH);
}

/* CFL number = c · dt / dx (here dx = 1 cell, so c · dt).  When this
 * approaches 1 the scheme is at its stability cliff; the HUD colours
 * it red so the user knows when to back off dt.  Refs [4] LeVeque
 * ch. 4 — the canonical Courant condition for hyperbolic schemes. */
static inline float swe_cfl_number(float wave_speed, float dt_seconds) {
    return wave_speed * dt_seconds;
}

/*
 * compute_sim_stats — fill the SimStats snapshot in ONE grid pass.
 *
 * Pseudocode:
 *   scan_max_height_and_average_speed(&max_h, &avg_speed)
 *   wave_speed = swe_wave_speed(g)
 *   cfl        = swe_cfl_number(wave_speed, dt)
 *   write all four into *out_stats
 */
static void compute_sim_stats(float gravity_acceleration, float dt_seconds,
                              SimStats *out_stats) {
    scan_max_height_and_average_speed(&out_stats->max_abs_height,
                                       &out_stats->average_speed);
    out_stats->wave_speed = swe_wave_speed(gravity_acceleration);
    out_stats->cfl_number = swe_cfl_number(out_stats->wave_speed, dt_seconds);
}

/* ===================================================================== */
/* §12  swe_step — one full physics tick                                 */
/* ===================================================================== */
/*
 * Five calls.  Order matters:
 *   1. update velocity from height gradient (§9)
 *   2. update height from velocity divergence (§10)
 *   3. zero fields inside obstacles (§8)
 *   4. apply boundary condition (§7)
 *   5. compute live stats (§11)
 */

static void swe_step(int boundary_kind, float gravity_acceleration,
                     float bottom_friction_coeff, float dt_seconds,
                     SimStats *out_stats) {
  update_velocity(gravity_acceleration, bottom_friction_coeff, dt_seconds);
  update_height(dt_seconds);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
  compute_sim_stats(gravity_acceleration, dt_seconds, out_stats);
}

/* ===================================================================== */
/* §13  excitation — drops + dam-break initial states                    */
/* ===================================================================== */

/* apply_gaussian_drop — additive Gaussian pulse at (cx, cy).
 * h[r, c] += amplitude · exp(-((c-cx)² + (r-cy)²) / (2 σ²))
 * u, v left untouched (impulsive vertical displacement, no
 * initial momentum).  The next tick's velocity update launches
 * a ring outward. */
/* Evaluate a 2-D Gaussian at squared distance d² from its centre:
 *   amplitude · exp(−d² / 2σ²)
 * Pre-multiplying 1/(2σ²) lets the caller avoid recomputing it per
 * cell — useful because this function is called inside an inner
 * loop with thousands of cells. */
static inline float gaussian_value(float dx_sq, float dy_sq, float amplitude,
                                    float inv_two_sigma_sq) {
    return amplitude * expf(-(dx_sq + dy_sq) * inv_two_sigma_sq);
}

/* Add a Gaussian pulse to cell (r, c)'s height.  Skips obstacle cells
 * (their height is forced to zero — incrementing them would create
 * spurious wave fronts at the wall edge next tick). */
static inline void add_gaussian_pulse_to_cell(int r, int c,
                                               float cx, float cy,
                                               float amplitude,
                                               float inv_two_sigma_sq) {
    if (wall_mask[r][c]) return;
    float dx    = (float)c - cx;
    float dy    = (float)r - cy;
    height_perturbation[r][c] +=
        gaussian_value(dx * dx, dy * dy, amplitude, inv_two_sigma_sq);
}

/*
 * apply_gaussian_drop — additive Gaussian PULSE in the height field
 * at (cx, cy).  Velocity is left untouched (impulsive vertical
 * displacement, no initial momentum); the next tick's update_velocity
 * call launches a propagating ring.
 *
 *   h[r, c] += A · exp(−((c − cx)² + (r − cy)²) / 2σ²)
 *
 * Pseudocode:
 *   inv_two_sigma_sq = 1 / (2σ²)
 *   for each cell (r, c):
 *     add_gaussian_pulse_to_cell(r, c, cx, cy, A, inv_two_sigma_sq)
 */
static void apply_gaussian_drop(float cx, float cy, float amplitude,
                                 float sigma) {
    float inv_two_sigma_sq = 1.0f / (2.0f * sigma * sigma);
    for (int r = 0; r < grid_active_rows; r++)
        for (int c = 0; c < grid_active_cols; c++)
            add_gaussian_pulse_to_cell(r, c, cx, cy, amplitude,
                                        inv_two_sigma_sq);
}

/* apply_dam_break_initial_state — ±DAM_AMP step at the vertical
 * midline.  Used by PRESET_DAM_BREAK and the 'b' key. */
static void apply_dam_break_initial_state(void) {
  int half_col = grid_active_cols / 2;
  for (int r = 0; r < grid_active_rows; r++) {
    for (int c = 0; c < grid_active_cols; c++) {
      if (wall_mask[r][c])
        continue;
      height_perturbation[r][c] =
          (c < half_col) ? DAM_BREAK_AMPLITUDE : -DAM_BREAK_AMPLITUDE;
    }
  }
}

/* ===================================================================== */
/* §14  presets — 4 scene loaders + table                                */
/* ===================================================================== */

/*
 * Preset — one named scenario for the shallow-water demo.
 *
 * Intent
 *   Each preset demonstrates a different fundamental SWE phenomenon:
 *     - dam_break    : classical Riemann problem (Stoker §6) — water
 *                      column on one side, dry on the other; wavefront
 *                      propagates at √(g·h).
 *     - radial_drop  : single Gaussian impulse → expanding ring.
 *     - channel      : narrow inlet feeds a basin; wave + shoaling.
 *     - obstacle     : flow around a solid; wakes + reflections.
 *
 *   Pressing 1..N calls the preset's loader, which writes initial h
 *   and (u, v) fields directly.
 *
 * Why a function-pointer loader (not a parameter table)
 *   Each scenario differs in INITIAL-CONDITION GEOMETRY rather than
 *   in a few numeric parameters — different spatial patterns, some
 *   with obstacles, some without.  Function pointers let each loader
 *   write whatever pattern it needs without trying to encode every
 *   scene's geometry into a shared parameter struct.
 *
 * Why the loader takes a boundary_kind argument
 *   After loading initial conditions, the preset has to RE-APPLY the
 *   boundary condition (reflecting / periodic / absorbing) to the
 *   newly-written cells.  Passing it explicitly avoids tight coupling
 *   to a global "current boundary kind" variable.
 *
 * Reference [1] Stoker §6 for the dam-break Riemann problem.
 */
typedef struct {
    const char *display_name;                  /* short HUD label      */
    void      (*loader)(int boundary_kind);    /* writes ICs + reapplies
                                                * BC after writing      */
} Preset;

static void preset_dam_break(int boundary_kind);
static void preset_radial_drop(int boundary_kind);
static void preset_channel(int boundary_kind);
static void preset_obstacle(int boundary_kind);

static const Preset preset_table[PRESET_COUNT] = {
    {"dam_break  ", preset_dam_break},
    {"radial_drop", preset_radial_drop},
    {"channel    ", preset_channel},
    {"obstacle   ", preset_obstacle},
};

static void preset_dam_break(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  apply_dam_break_initial_state();
  apply_boundary(boundary_kind);
}

static void preset_radial_drop(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  float cx = (float)(grid_active_cols - 1) * 0.5f;
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  apply_gaussian_drop(cx, cy, DROP_HEIGHT_AMPLITUDE, DROP_GAUSSIAN_RADIUS);
  apply_boundary(boundary_kind);
}

static void preset_channel(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  obstacle_build_channel();
  /* Drop the source on the LEFT side so the wave is funnelled
   * through the gap. */
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float cx_source = (float)(grid_active_cols - 1) * 0.20f;
  apply_gaussian_drop(cx_source, cy, DROP_HEIGHT_AMPLITUDE,
                      DROP_GAUSSIAN_RADIUS);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
}

static void preset_obstacle(int boundary_kind) {
  grid_clear_walls();
  grid_zero_fluid_fields();
  obstacle_build_circle();
  float cy = (float)(grid_active_rows - 1) * 0.5f;
  float cx_source = (float)(grid_active_cols - 1) * 0.15f;
  apply_gaussian_drop(cx_source, cy, DROP_HEIGHT_AMPLITUDE,
                      DROP_GAUSSIAN_RADIUS);
  zero_fields_in_walls();
  apply_boundary(boundary_kind);
}

/* ===================================================================== */
/* §15  shoreline — zero-crossing detection helper (T10)                 */
/* ===================================================================== */
/*
 * A cell straddles a wavefront if:
 *   - its own |h| is small,
 *   - at least one non-wall neighbour has h > +THRESH,
 *   - at least one non-wall neighbour has h < -THRESH.
 *
 * The opposite-sign-neighbour check filters out resting cells
 * (which also have small |h|) and cells next to obstacle walls
 * (where h is forced to zero — without the check those would all
 * look like shorelines).
 */

/* Is (r, c) a valid, NON-WALL grid cell?  Wall cells are excluded
 * because their h is forced to zero by zero_fields_in_walls — without
 * the exclusion every cell adjacent to an obstacle would look like a
 * shoreline.  Off-grid coords also return false. */
static inline bool neighbour_is_water_cell(int r, int c) {
    return r >= 0 && r < grid_active_rows
        && c >= 0 && c < grid_active_cols
        && !wall_mask[r][c];
}

/* Inspect one neighbour cell for evidence of opposite polarities.
 * Updates the seen-positive / seen-negative flags in place rather
 * than returning, so the caller can pass the same pair through four
 * neighbour checks.  Cells below the SHORELINE_HEIGHT_THRESHOLD are
 * treated as "near zero" and contribute neither flag. */
static inline void inspect_neighbour_for_polarities(int r, int c,
                                                     bool *seen_positive,
                                                     bool *seen_negative) {
    if (!neighbour_is_water_cell(r, c)) return;
    float h = height_perturbation[r][c];
    if (h >  SHORELINE_HEIGHT_THRESHOLD) *seen_positive = true;
    if (h < -SHORELINE_HEIGHT_THRESHOLD) *seen_negative = true;
}

/*
 * is_shoreline_cell — does cell (r, c) sit on the ZERO CROSSING
 * between positive and negative surface displacement?
 *
 * Pseudocode:
 *   seen_pos = seen_neg = false
 *   inspect the four cardinal neighbours via
 *     inspect_neighbour_for_polarities(r±1, c) and (r, c±1)
 *   return seen_pos && seen_neg
 *
 * The opposite-sign-neighbour check filters out RESTING cells (also
 * small |h|) and obstacle-adjacent cells (h forced to zero) — without
 * the polarity check those would all look like shorelines.  See §15
 * comment for the visual intuition.
 */
static bool is_shoreline_cell(int r, int c) {
    bool seen_positive = false;
    bool seen_negative = false;

    inspect_neighbour_for_polarities(r - 1, c    , &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r + 1, c    , &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r    , c - 1, &seen_positive, &seen_negative);
    inspect_neighbour_for_polarities(r    , c + 1, &seen_positive, &seen_negative);

    return seen_positive && seen_negative;
}

/* ===================================================================== */
/* §16  render_field — glyph + colour + arrows + shoreline (T10)         */
/* ===================================================================== */

static const char arrow_glyph_table[8] = {'>', '/', 'v', '\\',
                                          '<', '/', '^', '\\'};

static int velocity_to_arrow_octant(float vx, float vy) {
  float angle = atan2f(vy, vx);
  if (angle < 0.0f)
    angle += 2.0f * (float)M_PI;
  return (int)(angle / ((float)M_PI * 0.25f)) % 8;
}

/* Layer 1 — paint an OBSTACLE cell ('#' in PAIR_OBSTACLE).  Called
 * for every wall_mask cell; show_obstacles gates whether it's drawn
 * at all (toggleable so users can see the field underneath). */
static inline void paint_obstacle_layer(WINDOW *w, int r, int c) {
    attr_t attr = COLOR_PAIR(PAIR_OBSTACLE);
    wattron(w, attr);
    mvwaddch(w, r, c, '#');
    wattroff(w, attr);
}

/* Layer 2 — paint a SHORELINE cell ('~' bold in PAIR_SHORELINE).
 * Caller already verified |h| < threshold AND is_shoreline_cell(). */
static inline void paint_shoreline_layer(WINDOW *w, int r, int c) {
    attr_t attr = COLOR_PAIR(PAIR_SHORELINE) | A_BOLD;
    wattron(w, attr);
    mvwaddch(w, r, c, '~');
    wattroff(w, attr);
}

/* Map a signed height value to (positive, normalised_slot).
 *   - positive   → true if h ≥ 0 (crest), false if trough.
 *   - normalised → |h| / display_max ∈ [0, 1] clamped.
 *   - slot       → tier index for the glyph ramp.
 * Slot 0 means "below the visible threshold" — caller should skip
 * the cell.  Refs [8] Bourke on the intensity ramp design. */
static inline int height_to_polarity_and_slot(float h, float inv_display_max,
                                                bool *out_positive) {
    *out_positive = (h >= 0.0f);
    float normalised = fabsf(h) * inv_display_max;
    if (normalised > 1.0f) normalised = 1.0f;
    return normalised_height_to_slot(normalised);
}

/* Layer 4 — paint a velocity ARROW at a fast cell.  Returns true if
 * the arrow was drawn (cell speed > threshold) so the caller skips
 * the height-glyph layer.  Uses the height layer's COLOUR pair —
 * arrows REPLACE the glyph but keep the height information. */
static inline bool paint_arrow_layer_if_fast(WINDOW *w, int r, int c,
                                              attr_t height_attr) {
    float speed = velocity_magnitude_at(r, c);
    if (speed <= ARROW_SPEED_THRESHOLD) return false;
    int octant = velocity_to_arrow_octant(velocity_x[r][c], velocity_y[r][c]);
    wattron(w, height_attr);
    mvwaddch(w, r, c, (chtype)(unsigned char)arrow_glyph_table[octant]);
    wattroff(w, height_attr);
    return true;
}

/* Layer 3 — paint the HEIGHT SHADING glyph at slot tier. */
static inline void paint_height_layer(WINDOW *w, int r, int c, int slot,
                                       attr_t height_attr) {
    wattron(w, height_attr);
    mvwaddch(w, r, c, (chtype)(unsigned char)ramp_glyph_table[slot]);
    wattroff(w, height_attr);
}

/* Paint ONE non-obstacle cell.  Walks the rendering layers in
 * priority order: shoreline (if requested & near-zero) → arrow (if
 * requested & fast) → height shading.  Returns immediately at each
 * matched layer — only one glyph per cell.                          */
static inline void paint_one_cell_layered(WINDOW *w, int r, int c,
                                           int active_theme_index,
                                           bool show_arrows,
                                           bool show_shoreline,
                                           float inv_display_max) {
    float h = height_perturbation[r][c];

    /* Layer 2 — Shoreline.  Only consider it when |h| is small. */
    if (show_shoreline && fabsf(h) < SHORELINE_HEIGHT_THRESHOLD) {
        if (is_shoreline_cell(r, c)) paint_shoreline_layer(w, r, c);
        return;
    }

    /* Layer 3+4 — Height shading + optional arrow override. */
    bool positive;
    int  slot = height_to_polarity_and_slot(h, inv_display_max, &positive);
    if (slot == 0) return;
    attr_t height_attr = height_pair_attr(active_theme_index, positive, slot);

    if (show_arrows && paint_arrow_layer_if_fast(w, r, c, height_attr)) return;
    paint_height_layer(w, r, c, slot, height_attr);
}

/*
 * render_field — paint the SWE field as a layered visual.
 *
 * Pseudocode:
 *   inv_display_max = 1 / max(DISPLAY_HEIGHT_RANGE, 0.05)
 *   for each cell (r, c):
 *     if wall_mask[r][c]:
 *       if show_obstacles: paint_obstacle_layer
 *       continue
 *     paint_one_cell_layered(w, r, c, theme, show_arrows, show_shoreline,
 *                            inv_display_max)
 *
 * Layer priority (highest to lowest):
 *   1. Obstacle (always wins if show_obstacles)
 *   2. Shoreline (zero-crossing near walls)
 *   3. Height shading (the main visual)
 *   4. Arrow overlay (replaces glyph at fast cells; keeps height colour)
 */
static void render_field(WINDOW *w, int active_theme_index, bool show_arrows,
                          bool show_shoreline, bool show_obstacles) {
    float display_max     = DISPLAY_HEIGHT_RANGE;
    if (display_max < 0.05f) display_max = 0.05f;
    float inv_display_max = 1.0f / display_max;

    int rows = grid_active_rows;
    int cols = grid_active_cols;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            /* Layer 1 — Obstacle (highest priority). */
            if (wall_mask[r][c]) {
                if (show_obstacles) paint_obstacle_layer(w, r, c);
                continue;
            }
            paint_one_cell_layered(w, r, c, active_theme_index,
                                    show_arrows, show_shoreline,
                                    inv_display_max);
        }
    }
}

/* ===================================================================== */
/* §17  stats_panel — bottom-left debug overlay                          */
/* ===================================================================== */
/*
 * A small box of live numbers — max-h, avg-vel, wave speed, CFL,
 * gravity, damping, BC, sim-time, current preset.  CFL line is
 * colour-coded by stability band (green / yellow / red).
 */

/*
 * OverlayBox — rectangular footprint for a HUD overlay panel.
 *
 * Intent
 *   The stats panel is a small box of live numbers drawn over the
 *   field.  Its position (ox, oy) and extent (pw, ph) are computed
 *   once per frame by stats_panel_layout(); paint helpers then read
 *   the box back to know where to draw each row.
 *
 * Why a struct (not four parameters threaded through helpers)
 *   The panel has ~10 rows, each painted by its own helper.  Passing
 *   four ints through every helper would be repetitive and easy to
 *   mis-order; one OverlayBox passed by const pointer is cleaner.
 *
 * Lifetime
 *   Built per frame in stats_panel_layout(), passed by const pointer
 *   to paint helpers, discarded at end of frame.
 */
typedef struct {
    int ox;   /* left edge in cells                       */
    int oy;   /* top edge in cells                        */
    int pw;   /* panel width in cells                     */
    int ph;   /* panel height in cells                    */
} OverlayBox;

static OverlayBox stats_panel_layout(int term_rows, int term_cols) {
  OverlayBox box = {1, 0, 30, 13};
  box.oy = term_rows - box.ph - 1;
  if (box.oy < 0)
    box.oy = 0;
  if (box.ox + box.pw > term_cols)
    box.pw = term_cols - box.ox;
  return box;
}

/* CFL-stability colour bands.  Returned pair tags are picked from
 * the THEME 0 palette so they're available at startup. */
static int stats_cfl_pair(float cfl) {
  if (cfl < CFL_STABLE_LIMIT)
    return PAIR_NEG(0, 5); /* dim blue (green-ish) */
  if (cfl < CFL_MARGINAL_LIMIT)
    return PAIR_HUD;     /* yellow */
  return PAIR_POS(1, 7); /* magma red */
}

static const char *stats_cfl_label(float cfl) {
  if (cfl < CFL_STABLE_LIMIT)
    return "STABLE  ";
  if (cfl < CFL_MARGINAL_LIMIT)
    return "MARGINAL";
  return "UNSTABLE";
}

static void render_stats_panel(WINDOW *w, int term_rows, int term_cols,
                               const SimStats *stats,
                               float gravity_acceleration,
                               float bottom_friction_coeff,
                               int active_boundary_kind,
                               float simulation_time_seconds,
                               bool simulation_paused, bool show_arrows,
                               bool show_shoreline, int active_preset_index) {
  OverlayBox box = stats_panel_layout(term_rows, term_cols);
  if (box.pw < 20 || box.ph < 6)
    return; /* terminal too small */

  int ox = box.ox;
  int oy = box.oy;

  /* Box border — use the HUD pair (yellow + bold) so the panel
   * is clearly an overlay, not part of the simulation. */
  attr_t border_attr = COLOR_PAIR(PAIR_HUD) | A_BOLD;
  wattron(w, border_attr);
  mvwprintw(w, oy + 0, ox, "+--- SHALLOW WATER --------+");
  mvwprintw(w, oy + 1, ox, "| max_h   %8.4f           |",
            (double)stats->max_abs_height);
  mvwprintw(w, oy + 2, ox, "| avg_vel %8.4f           |",
            (double)stats->average_speed);
  mvwprintw(w, oy + 3, ox, "| wavespd %7.2f cells/s    |",
            (double)stats->wave_speed);

  mvwprintw(w, oy + 4, ox, "| CFL     ");
  wattroff(w, border_attr);
  {
    attr_t cfl_attr = COLOR_PAIR(stats_cfl_pair(stats->cfl_number)) | A_BOLD;
    wattron(w, cfl_attr);
    wprintw(w, "%5.3f %-8s", (double)stats->cfl_number,
            stats_cfl_label(stats->cfl_number));
    wattroff(w, cfl_attr);
  }
  wattron(w, border_attr);
  wprintw(w, "|");

  mvwprintw(w, oy + 5, ox, "| gravity %7.1f            |",
            (double)gravity_acceleration);
  mvwprintw(w, oy + 6, ox, "| damping %8.4f           |",
            (double)bottom_friction_coeff);
  mvwprintw(w, oy + 7, ox, "| BC      %s         |",
            bc_name_table[active_boundary_kind]);
  mvwprintw(w, oy + 8, ox, "| sim_t   %8.2f s          |",
            (double)simulation_time_seconds);
  mvwprintw(w, oy + 9, ox, "| preset  %s         |",
            preset_name_table[active_preset_index]);
  mvwprintw(w, oy + 10, ox, "| arrows  %-3s shore %-3s      |",
            show_arrows ? "ON " : "OFF", show_shoreline ? "ON " : "OFF");
  mvwprintw(w, oy + 11, ox, "| %s                       |",
            simulation_paused ? "PAUSED        " : "running       ");
  mvwprintw(w, oy + 12, ox, "+---------------------------+");
  wattroff(w, border_attr);
}

/* ===================================================================== */
/* §18  hud — top status + bottom hint (CLAUDE.md spec)                  */
/* ===================================================================== */

static void hud_paint_status(int term_cols, double measured_fps,
                             int sim_steps_per_second,
                             float gravity_acceleration,
                             float wave_speed_cells_per_sec,
                             int active_boundary_kind, int active_theme_index) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " ShallowWater  %5.1f fps  sim:%3dHz  g=%.0f  c=%.1f  "
           "BC=%s  theme:%s ",
           measured_fps, sim_steps_per_second, (double)gravity_acceleration,
           (double)wave_speed_cells_per_sec,
           bc_name_table[active_boundary_kind],
           theme_table[active_theme_index].display_name);
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
           " q:quit  spc:pause  s:step  r:reset  d:drop  b:dam  "
           "g/G:gravity  a:arrows  l:shore  n:BC  o:obs  "
           "p/P:preset  t:theme  ]/[:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §19  scene — per-frame state + tick wrapper                           */
/* ===================================================================== */

/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Scene composes the user's physics tunables (gravity, friction,
 *   boundary kind), the control flags (paused, step-request,
 *   preset), the visual toggles (theme, overlay layers), and the
 *   per-frame stats snapshot.  The actual h / u / v fields live as
 *   file-scope arrays since every solver pass loops over them and
 *   threading a struct pointer through would obscure the math.
 *
 * Locality (sim vs render)
 *   The fields below are GROUPED EXPLICITLY by subsystem.  This file
 *   already had nice section comments — we keep them and document
 *   what each group is for:
 *
 *     - PHYSICS                → simulation (read by step / boundary)
 *     - CONTROL                → gates the tick + signals next-frame
 *                                 behaviour
 *     - VISUAL                 → pure rendering (theme + overlays)
 *     - STATS / TIME           → derived; HUD-only, but written every
 *                                 tick by the stats scan
 *
 *   Mis-classifying a VISUAL field (e.g. show_arrows) as sim would
 *   couple physics to a render toggle, breaking the "look is decoupled
 *   from physics" invariant.
 *
 * Why these specific fields and no others
 *   - gravity_acceleration  / bottom_friction_coeff      physics
 *     constants tunable via keys; sim reads them every step.
 *   - active_boundary_kind   selects reflecting / periodic / absorbing
 *     BC; both sim (next-step ghost cells) and preset-load read it.
 *   - simulation_paused / step_request_pending   gate the tick;
 *     step_request_pending lets the user advance ONE frame while
 *     paused (powers single-stepping).
 *   - active_preset_index    survives reset; HUD reads its name.
 *   - active_theme_index / show_arrows / show_shoreline / show_obstacles
 *     pure render toggles; must not touch sim state.
 *   - stats                  scratch snapshot, rebuilt every frame by
 *     the stats scan.  Lives here so the HUD can read it via Scene*.
 *   - simulation_time_seconds   monotonic clock for HUD readout.
 *
 * Things that DO NOT live here
 *   - h / u / v field arrays           → file-scope (large; touched
 *                                         by every loop)
 *   - The boundary lookup tables       → file-scope constants
 *   - Render-frame timing / FPS        → locals in main()
 *   - Signal flags                     → App
 *
 * Reference [4] LeVeque on the conceptual sim/render separation that
 *   makes this layout reproducible.
 */
typedef struct {
    /* ── Simulation tunables (read by step + boundary passes) ───── */
    float gravity_acceleration;     /* g, gravity (m/s² equivalent)  */
    float bottom_friction_coeff;    /* Manning-like drag             */
    int   active_boundary_kind;     /* REFLECTING / PERIODIC / ABSORB*/

    /* ── Control state (gates tick / preset / single-step) ──────── */
    bool  simulation_paused;        /* if true tick is a no-op       */
    bool  step_request_pending;     /* advance one frame then re-pause*/
    int   active_preset_index;      /* row of preset_table[] in use  */

    /* ── Pure render state (read by paint_field / paint_overlays) ─ *
     * Changing these MUST NOT touch the h / u / v arrays.          */
    int   active_theme_index;       /* row of theme_table[]          */
    bool  show_arrows;              /* draw velocity arrows overlay  */
    bool  show_shoreline;           /* highlight wet/dry boundary    */
    bool  show_obstacles;           /* show solid obstacles overlay  */

    /* ── Per-frame diagnostics + clock (HUD-only consumers) ─────── */
    SimStats stats;                 /* rebuilt by stats scan         */
    float    simulation_time_seconds; /* monotonic clock for HUD     */
} Scene;

static void scene_init(Scene *scene, int cols, int rows) {
  memset(scene, 0, sizeof *scene);
  scene->gravity_acceleration = GRAVITY_DEFAULT;
  scene->bottom_friction_coeff = BOTTOM_FRICTION_DEFAULT;
  scene->active_boundary_kind = BC_WALL;
  scene->active_theme_index = 0;
  scene->show_arrows = true;
  scene->show_shoreline = true;
  scene->show_obstacles = true;

  grid_active_cols = (cols < GRID_COLS_MAX) ? cols : GRID_COLS_MAX;
  grid_active_rows = (rows < GRID_ROWS_MAX) ? rows : GRID_ROWS_MAX;
  if (grid_active_rows < 4)
    grid_active_rows = 4;
  if (grid_active_cols < 4)
    grid_active_cols = 4;

  grid_clear_walls();
  grid_zero_fluid_fields();
  preset_table[PRESET_DAM_BREAK].loader(scene->active_boundary_kind);
  scene->active_preset_index = PRESET_DAM_BREAK;
}

static void scene_resize(Scene *scene, int cols, int rows) {
  grid_active_cols = (cols < GRID_COLS_MAX) ? cols : GRID_COLS_MAX;
  grid_active_rows = (rows < GRID_ROWS_MAX) ? rows : GRID_ROWS_MAX;
  if (grid_active_rows < 4)
    grid_active_rows = 4;
  if (grid_active_cols < 4)
    grid_active_cols = 4;
  /* Reload the current preset so walls + initial conditions
   * adapt to the new domain size. */
  preset_table[scene->active_preset_index].loader(scene->active_boundary_kind);
  scene->simulation_time_seconds = 0.0f;
}

static void scene_load_preset(Scene *scene, int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  scene->active_preset_index = preset_index;
  preset_table[preset_index].loader(scene->active_boundary_kind);
  scene->simulation_time_seconds = 0.0f;
}

static void scene_tick(Scene *scene, float dt_seconds) {
  if (scene->simulation_paused && !scene->step_request_pending)
    return;
  scene->step_request_pending = false;
  scene->simulation_time_seconds += dt_seconds;
  swe_step(scene->active_boundary_kind, scene->gravity_acceleration,
           scene->bottom_friction_coeff, dt_seconds, &scene->stats);
}

/* ===================================================================== */
/* §20  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we
 * keep only cell dimensions for HUD placement + field clipping.
 *
 * Render pipeline (one frame): erase → paint_field → paint_overlays
 *   → hud_paint_* → wnoutrefresh(stdscr) → doupdate().  Diff-only
 *   writes — no flicker.  See [9] Raymond §11.
 */
typedef struct {
    int cols;   /* terminal width  in cells (getmaxyx)             */
    int rows;   /* terminal height in cells (getmaxyx)             */
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
                                 double measured_fps,
                                 int sim_steps_per_second) {
  erase();

  render_field(stdscr, scene->active_theme_index, scene->show_arrows,
               scene->show_shoreline, scene->show_obstacles);

  render_stats_panel(stdscr, screen->rows, screen->cols, &scene->stats,
                     scene->gravity_acceleration, scene->bottom_friction_coeff,
                     scene->active_boundary_kind,
                     scene->simulation_time_seconds, scene->simulation_paused,
                     scene->show_arrows, scene->show_shoreline,
                     scene->active_preset_index);

  hud_paint_status(screen->cols, measured_fps, sim_steps_per_second,
                   scene->gravity_acceleration, scene->stats.wave_speed,
                   scene->active_boundary_kind, scene->active_theme_index);
  hud_paint_hint(screen->rows);

  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §21  app — main loop + signals + input                                */
/* ===================================================================== */

/*
 * App — top-level container; lives in BSS as the single app_state instance.
 *
 * Intent
 *   Signal handlers need to reach state the main loop polls.  A
 *   global App + handlers that flip its volatile sig_atomic_t flags
 *   is the standard POSIX "wake the main loop" pattern.
 *
 * Why the volatile sig_atomic_t flags
 *   POSIX permits signal handlers to write ONLY sig_atomic_t values
 *   with simple assignments — anything wider is UB.  volatile forces
 *   every read in the main loop to go back to memory across signal
 *   arrival (no compiler caching).
 *
 * Why sim_steps_per_second lives here (not in Scene)
 *   sim_steps_per_second is a frame-loop concern — it picks the
 *   inner-loop tick period TICK_NS(rate) — and has no meaning inside
 *   scene_tick which receives the resulting dt as a parameter.
 *   Keeping it on App keeps Scene free of timing detail.
 */
typedef struct {
    Scene  scene;                          /* world + control state    */
    Screen screen;                         /* terminal extent          */
    int    sim_steps_per_second;           /* sim tick rate, Hz        */
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

static bool app_handle_key(App *app, int ch) {
  Scene *scene = &app->scene;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    scene->simulation_paused = !scene->simulation_paused;
    break;

  case 's':
  case 'S':
    scene->simulation_paused = true;
    scene->step_request_pending = true;
    break;

  case 'r':
  case 'R':
    scene_load_preset(scene, scene->active_preset_index);
    break;

  case 'd':
  case 'D': {
    float cx = (float)(grid_active_cols - 1) * 0.5f;
    float cy = (float)(grid_active_rows - 1) * 0.5f;
    apply_gaussian_drop(cx, cy, DROP_HEIGHT_AMPLITUDE, DROP_GAUSSIAN_RADIUS);
    zero_fields_in_walls();
    apply_boundary(scene->active_boundary_kind);
    break;
  }

  case 'b':
  case 'B':
    grid_zero_fluid_fields();
    apply_dam_break_initial_state();
    zero_fields_in_walls();
    apply_boundary(scene->active_boundary_kind);
    scene->simulation_time_seconds = 0.0f;
    break;

  case 'g':
    scene->gravity_acceleration += GRAVITY_STEP;
    if (scene->gravity_acceleration > GRAVITY_MAX)
      scene->gravity_acceleration = GRAVITY_MAX;
    break;
  case 'G':
    scene->gravity_acceleration -= GRAVITY_STEP;
    if (scene->gravity_acceleration < GRAVITY_MIN)
      scene->gravity_acceleration = GRAVITY_MIN;
    break;

  case 'n':
  case 'N':
    scene->active_boundary_kind = (scene->active_boundary_kind + 1) % BC_COUNT;
    scene_load_preset(scene, scene->active_preset_index);
    break;

  case 'p':
    scene_load_preset(scene, (scene->active_preset_index + 1) % PRESET_COUNT);
    break;
  case 'P':
    scene_load_preset(scene, (scene->active_preset_index + PRESET_COUNT - 1) %
                                 PRESET_COUNT);
    break;

  case 'o':
  case 'O':
    scene->show_obstacles = !scene->show_obstacles;
    break;

  case 'a':
  case 'A':
    scene->show_arrows = !scene->show_arrows;
    break;

  case 'l':
  case 'L':
    scene->show_shoreline = !scene->show_shoreline;
    break;

  case 't':
  case 'T':
    scene->active_theme_index = (scene->active_theme_index + 1) % THEME_COUNT;
    break;

  case ']':
    app->sim_steps_per_second += SIM_HZ_STEP;
    if (app->sim_steps_per_second > SIM_HZ_MAX)
      app->sim_steps_per_second = SIM_HZ_MAX;
    break;
  case '[':
    app->sim_steps_per_second -= SIM_HZ_STEP;
    if (app->sim_steps_per_second < SIM_HZ_MIN)
      app->sim_steps_per_second = SIM_HZ_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned)(clock_now_ns() & 0xFFFFFFFF));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal_quit);
  signal(SIGTERM, on_signal_quit);
  signal(SIGWINCH, on_signal_resize);

  App *app = &app_state;
  app->running = 1;
  app->sim_steps_per_second = SIM_HZ_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;
  int64_t sim_accumulator_ns = 0;

  const int64_t render_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (app->running) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }
    if (!app->running)
      break;

    /* ── resize ── */
    if (app->need_resize) {
      app->need_resize = 0;
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      prev_frame_ns = clock_now_ns();
      sim_accumulator_ns = 0;
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

    /* ── physics — fixed-dt accumulator ── */
    int64_t tick_ns = TICK_NS(app->sim_steps_per_second);
    float dt_sec = 1.0f / (float)app->sim_steps_per_second;
    sim_accumulator_ns += dt_ns;
    while (sim_accumulator_ns >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accumulator_ns -= tick_ns;
    }

    /* ── render ── */
    screen_present_frame(&app->screen, &app->scene, measured_fps,
                         app->sim_steps_per_second);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < render_cap_ns)
      clock_sleep_ns(render_cap_ns - spent);
  }

  return 0;
}
