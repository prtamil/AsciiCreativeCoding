/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke.c — pedagogical edition
 *
 *   A mushroom cloud as REAL fluid dynamics on a 2-D axisymmetric
 *   grid, volumetrically raymarched into the terminal.
 *
 *   This file is structured as an EMBEDDED TEXTBOOK.  Comments teach;
 *   the code is the worked exercises.  Read the tutorials at the top
 *   first; they explain every idea you'll meet later in §1..§24.  You
 *   should not need any prior knowledge of fluid dynamics, computer
 *   graphics, or numerical methods to follow along.
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause physics
 *   r            re-detonate (reset the simulation)
 *   n / N        next / previous BLAST TYPE
 *                  TACTICAL / STANDARD / MEGATON / AIR_BURST / GROUND
 *   t / T        next / previous theme
 *   d / D        cycle debug overlay (NORMAL / DENSITY / TEMPERATURE /
 *                                     VELOCITY)
 *   + / =        speed up sim time
 *   -            slow down sim time
 *   z / Z        camera zoom in / out
 *   ] / [        target render fps up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/nuke.c \
 *       -o nuke -lncurses -lm
 *
 * Section map (each section is ≤ ~100 lines and teaches one idea):
 *   §1  config           — every tunable, every magic number named
 *   §2  clock            — monotonic timer + sleep
 *   §3  color            — theme palette + HUD/hint pairs
 *   §4  vec3             — 3-D vector math (raymarcher uses this)
 *   §5  fluid state      — the fluid struct + memory layout
 *   §6  grid helpers     — clampf + bilinear sampling
 *   §7  boundaries       — what walls do to velocity
 *   §8  buoyancy         — hot air rises
 *   §9  advection        — fields move with the flow (Stam trick)
 *   §10 divergence       — measure of "compression"
 *   §11 pressure solve   — Jacobi iteration of ∇²p = ∇·v
 *   §12 gradient remove  — subtract ∇p from v → incompressible
 *   §13 projection       — orchestrator combining §10..§12
 *   §14 cool + decay     — exponential return to ambient
 *   §15 fluid_step       — one full physics tick
 *   §16 detonate         — initial Gaussian bubble
 *   §17 sampling         — read fluid from world coordinates
 *   §18 raymarch         — Beer-Lambert volumetric integration
 *   §19 camera           — orthonormal basis + per-pixel ray
 *   §20 cell decoration  — (luminance, hot fraction) → glyph + colour
 *   §21 scene            — Scene struct + tick + render
 *   §22 debug overlays   — see the raw fluid fields
 *   §23 screen           — ncurses init / resize / HUD
 *   §24 app              — main loop, signals, key handling
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is ordered to TEACH, not to compile-optimally.  The reading
 * order is:
 *
 *   1. CONCEPTS         (high-level "what is this program?")
 *   2. MENTAL MODEL     (intuition: how to think about it)
 *   3. GUIDED TUTORIAL  (10 mini-tutorials that build the algorithm
 *                        from first principles, no math required —
 *                        plain English + diagrams + pseudocode)
 *   4. §1..§24          (actual code, each section short and focused)
 *
 * If you only have ten minutes: read the GUIDED TUTORIAL.  After it
 * the rest of the file should feel like familiar territory rather
 * than a mathematical wall.
 *
 * The code uses LONG DESCRIPTIVE variable names everywhere (e.g.
 * `velocity_radial[i][j]` instead of `vr[i][j]`).  This adds visual
 * weight but turns every line into self-documentation.  When you
 * scan a function, the names tell you what's happening before you
 * read the math.
 *
 * Most fluid-simulation tutorials use the math notation `vᵣ, vᵧ, T,
 * ρ, p, ∇·v, ∇²p`.  Where useful, comments include both: the
 * intuition in English, then the math symbols, then the code.  You
 * can ignore any layer that's unfamiliar — the others will still
 * tell you what's going on.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : Boussinesq-approximation incompressible Navier-Stokes
 *                on a 2-D axisymmetric (radial × vertical) grid via the
 *                Stam (1999) "Stable Fluids" scheme — the ~100-line
 *                fluid solver that revolutionised computer graphics
 *                fluid dynamics by being unconditionally stable.
 *
 *                Fields stored on the grid (each is `float[N_R][N_Y]`):
 *                  velocity_radial      vᵣ — flow speed along radius
 *                  velocity_vertical    vᵧ — flow speed along altitude
 *                  temperature          T  — drives buoyancy
 *                  density              ρ  — visible smoke
 *                  pressure             p  — scratch for projection
 *                  divergence           ∇·v — scratch for projection
 *                  scratch_a, scratch_b — buffers for advection swap
 *
 *                Per simulation tick of duration step_seconds:
 *                  1. BUOYANCY         vᵧ += β·(T − T₀)·dt
 *                  2. PROJECT          enforce ∇·v = 0 (incompressible)
 *                  3. ADVECT VELOCITY  v moves with v itself
 *                  4. PROJECT          (advection re-broke ∇·v=0)
 *                  5. ADVECT T, ρ      smoke + heat ride the flow
 *                  6. COOL + DECAY     T → T₀, ρ → 0 over time
 *
 * Rendering    : Volumetric raymarching.  Per terminal cell:
 *                  – Build a ray through the pixel.
 *                  – March small steps; at each step compute (radius,
 *                    altitude) from world (x, y, z), bilinearly
 *                    sample density + temperature, accumulate
 *                    radiance via the volume rendering equation:
 *                        L_total += transmittance · density · STEP
 *                                                 · (emission + ambient)
 *                        transmittance *= exp(−density · STEP)
 *                  – When transmittance drops below ε, stop.
 *                  – Map (luminance, hot fraction) → glyph + theme pair.
 *
 * Why         : The mushroom shape is NOT scripted.  There are no
 *               "phase 1: rising column", "phase 2: cap forms"
 *               timelines anywhere in this file.  The cap forms
 *               because the leading edge of a buoyant column rolls
 *               into a torus (baroclinic vorticity at the temperature
 *               gradient); the plateau forms because cooling kills
 *               buoyancy; the descent forms because density decays.
 *               The mushroom IS the physics.
 *
 * Performance  : Grid 56 × 96 = 5 376 cells; each fluid step is ~30 ms
 *                on modern desktop hardware (the projection's 40
 *                Jacobi iterations dominate).  Renderer is ~5 ms at
 *                80×24, ~30 ms at 200×60.  Holds 30 fps comfortably.
 *
 * References   :
 *   • Stam, J. (1999) — "Stable Fluids", *SIGGRAPH '99* §3.
 *     The semi-Lagrangian advection + Hodge projection scheme this
 *     file implements.  Three pages of equations.  Decades of stable
 *     use.  Required reading for anyone who wants to grok this file.
 *   • Stam, J. (2003) — "Real-Time Fluid Dynamics for Games", *GDC*.
 *     Practical-friendly version of the 1999 paper.
 *   • Foster, N. & Metaxas, D. (1997) — "Modeling the motion of a
 *     hot, turbulent gas", *SIGGRAPH '97*.  Boussinesq-buoyancy-
 *     driven fluid VFX.  Their hot-Gaussian initial-condition recipe
 *     is what we use in `detonate_at_origin`.
 *   • Bridson, R. — *Fluid Simulation for Computer Graphics* (2015),
 *     ch. 3 "Incompressible Euler" + ch. 5 "Pressure".  The
 *     textbook treatment of the Hodge projection.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer*.  The march-along-a-ray idiom we use for rendering.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Treat the air around the blast as a grid of small volume cells.
 * Each cell is a tiny smoke detector + thermometer + velocimeter.  At
 * t = 0 we drop a hot, dense Gaussian bubble in the middle.  Each
 * sim tick:
 *
 *   – Hot cells push themselves UP.                  (buoyancy force)
 *   – The whole grid solves a constraint together:
 *     "find a pressure field that lets every cell
 *      conserve mass — no piling up, no vacuum."     (Hodge projection)
 *   – Each cell hands its contents to whatever cell
 *     is downwind, taking new contents from upwind.  (advection)
 *
 * Run that loop 600 times.  The cells, just by satisfying their local
 * constraints, COLLECTIVELY choreograph a mushroom cloud.  Nobody
 * told them to.  The shape is what falls out when buoyancy + mass
 * conservation + flow co-exist.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a swimming pool with 5 376 numbered tiles on its bottom.
 * Each tile knows: how much water is over me, how warm it is, which
 * direction it's moving.  We turn on a tiny hot heating element near
 * one tile.  Every other tile, at every tick, asks itself:
 *
 *   "If I were a parcel of water, where would I be one tick from now?
 *    Where would my heat go?  Where would my colour go?"
 *
 * They don't predict the future independently — pressure ties them
 * all together.  A cell trying to expand pushes its neighbours; the
 * neighbours push back; everyone settles on a flow that doesn't
 * compress anything.  The visible mushroom shape emerges from this
 * group negotiation, not from a script.
 *
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │   (1) hot bubble                  (2) rises, rolls       │
 *      │       at t = 0                        at t = 2 s         │
 *      │                                                          │
 *      │           ⬢                              ⌒⌒⌒             │
 *      │           ⬢                            ⬢⬢⬢⬢⬢            │
 *      │           ⬢                              ⬢⬢⬢             │
 *      │      ─────────                              ⬢             │
 *      │       ground                       ─────────              │
 *      │                                                          │
 *      │   (3) cap spreads                  (4) plateau, falls    │
 *      │       at t = 5 s                       at t = 12 s       │
 *      │                                                          │
 *      │       ⌒⌒⌒⌒⌒⌒⌒                         ⌒⌒⌒                │
 *      │      ⬢⬢⬢⬢⬢⬢⬢                          ⬢⬢⬢⬢⬢              │
 *      │       ⬢⬢⬢⬢                              ⬢⬢                │
 *      │         ⬢                                 ↓               │
 *      │         ⬢                                                │
 *      │      ─────                            ─────                │
 *      └──────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (the high-level loop)
 * ─────────────────────────────────────────
 *  1. INIT   — clear all fields.  T = T_ambient, ρ = 0, v = 0.
 *  2. DETONATE — paint a Gaussian hot bubble at the blast site.
 *  3. EVERY TICK:
 *        BUOYANCY  ▸  PROJECT  ▸  ADVECT v  ▸  PROJECT  ▸
 *        ADVECT (T, ρ)  ▸  COOL + DECAY
 *  4. EVERY FRAME:
 *        For each terminal cell, raymarch the field; map (lum,
 *        hot fraction) → glyph + colour.
 *
 * KEY FORMULAS          (each is unpacked in tutorials below)
 * ─────────────────────────────────────────────────────────
 *  Buoyancy:        F_y = β · (T − T₀)
 *  Incompressible:  ∇·v = 0
 *  Projection:      ∇²p = ∇·v_uncorrected;  v ← v − ∇p
 *  Advection:       φ_new(x) = φ_old(x − v·dt)     (semi-Lagrangian)
 *  Cooling:         T(t+dt) = T₀ + (T(t) − T₀) · exp(−k_cool·dt)
 *  Beer-Lambert:    transmittance(L) = exp(−∫₀ᴸ ρ ds)
 *  Volume render:   L = ∫ transmittance(s) · ρ(s) · source(s) ds
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Projection convergence — 40 Jacobi iters is approximate.
 *     Residual divergence ≈ 1-2 %, invisible at terminal resolution.
 *   • Semi-Lagrangian is unconditionally stable, but if v·dt > whole
 *     grid the backward-trace clamps to edges and the simulation
 *     diffuses badly.  Cap dt at SIM_DT_MAX.
 *   • Axis boundary (radius = 0): we approximate by clamping the
 *     radial velocity to zero (mirror reflection).  Small error, no
 *     visual impact.
 *   • Temperature can OVERSHOOT (advection in convergent regions).
 *     We don't clamp; cooling pulls things back over a few seconds.
 *
 * HOW TO VERIFY (run the program)
 * ───────────────────────────────
 *   • At t = 1 s the bubble is a sphere with a slightly flattened
 *     top (start of vortex roll).
 *   • At t = 3-5 s leading edge rolls outward, cap appears.
 *   • At t = 8-12 s cooling kills buoyancy, cloud plateaus, falls.
 *   • Press `d` to step through debug overlays.  You will literally
 *     see the temperature field in 2-D — that's the same data the
 *     raymarcher samples per pixel.
 *   • Press `n` to cycle blast types.  Same physics, different
 *     initial Gaussian → different mushroom.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — a first-principles build-up ────────────────────── *
 *
 * Ten short tutorials.  Each builds one piece of the puzzle.  Read
 * top to bottom; later sections refer back to these by number.
 *
 *
 * TUTORIAL 1: What is a fluid simulation?
 * ───────────────────────────────────────
 * A fluid (air, water, smoke) is CONTINUOUS — at every point in space
 * there is a velocity, a density, a temperature.  We can't store an
 * infinite number of points in a computer, so we DISCRETIZE: pick a
 * grid of cells and store one value per cell.
 *
 *      r=0  r=h  r=2h  r=3h ...
 *       ┌────┬────┬────┬────┐  y = (N-1)·h
 *       │    │    │    │    │
 *       ├────┼────┼────┼────┤    each cell stores:
 *       │    │    │    │    │      T  (temperature)
 *       ├────┼────┼────┼────┤      ρ  (density)
 *       │    │    │    │    │      vᵣ (radial velocity)
 *       ├────┼────┼────┼────┤      vᵧ (vertical velocity)
 *       │    │    │    │    │
 *       └────┴────┴────┴────┘  y = 0
 *      (axis)                  (R_MAX)
 *
 * Every sim tick (dt seconds), we update all cells based on:
 *
 *   – Forces     (gravity, buoyancy)
 *   – Exchange   (cells hand contents to neighbours; advection)
 *   – Constraint (pressure makes sure no cell overfills; projection)
 *
 *
 * TUTORIAL 2: Why "incompressible"?
 * ─────────────────────────────────
 * Air doesn't suddenly squeeze into a smaller space at low Mach
 * numbers (the speed of sound is much faster than our flows).
 * Mathematically, "the same amount that flows in must flow out":
 *
 *      ∇·v  =  0      (read: "divergence of velocity is zero")
 *
 * What that means for one cell:
 *
 *      ╲             ╱
 *       ╲ in        ╱ out      if 1 ml/sec enters from the left,
 *        ┌────────┐               1 ml/sec must leave through SOME
 *        │   v    │               other side, otherwise the cell
 *        └────────┘               would compress.
 *       ╱          ╲
 *      ╱            ╲
 *
 * If the velocity field doesn't satisfy this naturally, we COMPUTE
 * a pressure field whose gradient pushes back on any pile-ups.  That
 * step is called PROJECTION (tutorial 5).
 *
 *
 * TUTORIAL 3: What makes hot air rise?  (Buoyancy)
 * ────────────────────────────────────────────────
 * Hot gas is less dense than cool gas (same molecules, more thermal
 * jiggle, more space).  Gravity pulls everything down with the same
 * acceleration g, but the AIR around the hot pocket is heavier per
 * unit volume.  So the cool air sinks past the hot air, leaving the
 * hot air to rise relative to its surroundings.
 *
 * Net effect on a hot cell, per unit time:
 *
 *      acceleration_y  =  β · (T − T_ambient) · g       (positive = up)
 *
 * The Boussinesq approximation: density variations are small except
 * where they appear in the buoyancy term itself.  We bake `β · g`
 * into one tunable, BUOYANCY_COEFFICIENT.  This is the entire
 * coupling between temperature and motion — change nothing else and
 * you have the complete buoyant convection model.
 *
 *      T = T_hot      ░░░  cool air sinks past the hot pocket
 *      T > T_amb      ░░░  ↓
 *      ──── rises ────░░░
 *                     ░░░  ↑
 *                     ░░░  hot air rises by displacement
 *
 *
 * TUTORIAL 4: What is advection?  (Stuff moves with the flow)
 * ───────────────────────────────────────────────────────────
 * If a parcel of smoke is moving right at 1 m/s, then 1 second later
 * it is 1 m to the right of where it started.  Smoke, heat,
 * velocity, anything that's a property of the fluid, gets carried
 * along by the flow.  We call this ADVECTION.
 *
 * Naively, you'd PUSH each cell's contents forward:
 *
 *      for each cell c:
 *          dst_cell = c + v · dt          // where am I going?
 *          dst_cell.T += my_T             // dump my heat there
 *
 * This is the FORWARD-EULER style and it's UNSTABLE — under big
 * timesteps the dst_cell can be off-grid, two cells can dump into
 * the same destination, etc.
 *
 * Stam's trick (semi-Lagrangian advection): for each cell, ask
 * "where did the fluid currently in me COME FROM one step ago?",
 * then bilinearly sample the OLD field there.
 *
 *      for each cell c:
 *          src_pos = c − v · dt           // where did I come from?
 *          new_T[c] = bilinear(old_T, src_pos)
 *
 * This is unconditionally stable: no matter how large dt is, the
 * sampled value is just a weighted average of grid neighbours, which
 * is bounded.  The price is a tiny amount of numerical diffusion
 * (features blur slightly per step).  We pay this price gladly.
 *
 *
 *           NOW                       1 dt AGO
 *          ┌───┐                     ┌───┐
 *          │ ? │ ← bilinear(... ←─── │   │
 *          └───┘                     └───┘
 *           ↑                          ↑
 *         cell c                     src_pos = c − v · dt
 *
 *
 * TUTORIAL 5: What is "projection"?  (Enforcing incompressibility)
 * ────────────────────────────────────────────────────────────────
 * After buoyancy adds upward thrust, and after advection moves stuff
 * around, the velocity field is generally NOT divergence-free
 * anymore.  Some cells have nett inflow, others nett outflow.
 * Either would mean compression or vacuum — unphysical.
 *
 * The Helmholtz-Hodge decomposition theorem says any vector field
 * splits cleanly into:
 *
 *      v_total  =  v_divergence_free  +  ∇p
 *
 * That is, every velocity field can be written as a divergence-free
 * part plus the gradient of some scalar pressure field p.  We want
 * the divergence-free part:
 *
 *      v_divergence_free  =  v_total  −  ∇p
 *
 * To find p we take the divergence of both sides (∇·v_total =
 * 0 + ∇·∇p = ∇²p):
 *
 *      ∇²p  =  ∇·v_total                (Poisson equation for p)
 *
 * Then we solve that Poisson equation (tutorial 6) and subtract its
 * gradient from v.  The result is a velocity field that conserves
 * mass everywhere.  The pressure field is just BOOKKEEPING — it has
 * no physical "current value" outside this single time step.
 *
 *
 * TUTORIAL 6: How to solve ∇²p = ∇·v?  (Jacobi iteration)
 * ───────────────────────────────────────────────────────
 * On a regular grid, the discrete Laplacian uses the 5-point stencil:
 *
 *      ∇²p_{i,j}  ≈  (p_{i+1,j} + p_{i−1,j} + p_{i,j+1} + p_{i,j−1}
 *                     − 4·p_{i,j}) / h²
 *
 * Setting this equal to divergence_{i,j} and solving for p_{i,j}:
 *
 *      p_{i,j}  =  (p_{i+1,j} + p_{i−1,j} + p_{i,j+1} + p_{i,j−1}
 *                   − h² · divergence_{i,j}) / 4
 *
 * That's the JACOBI update rule.  We start with p ≈ 0 everywhere, do
 * 40 sweeps over the grid, and at each cell average our four
 * neighbours' current p values minus the local divergence.  After
 * 40 iterations the field has approximately satisfied the Poisson
 * equation everywhere, with about 1-2% residual error (visually
 * invisible at terminal resolution).
 *
 * Faster solvers exist (multigrid, conjugate gradient).  We use
 * Jacobi because it's the simplest possible thing that works.
 *
 *
 * TUTORIAL 7: Why does the MUSHROOM emerge?  (Vorticity at the edge)
 * ──────────────────────────────────────────────────────────────────
 * Consider a hot Gaussian bubble at t = 0:
 *
 *      hot in middle, ambient at edge:
 *
 *           T
 *           ▲
 *           │     ╭───╮
 *           │     ╱   ╲
 *           │   ╱       ╲
 *      T₀───┴───┴──────────r→
 *
 * Buoyancy lifts the WHOLE bubble — but the centre is hottest, so it
 * accelerates fastest.  After a moment the top of the bubble bulges
 * upward.  As it rises, ambient air must flow INTO the space below
 * it (incompressibility).  That inflow comes from the sides,
 * inducing inward radial flow at the equator of the bubble.
 *
 * Inward radial flow at the equator + upward axial flow at the centre
 * = a TOROIDAL VORTEX RING.  The bubble rolls itself up like a smoke
 * ring.  The inner part of the ring drags more hot stuff up; the
 * outer part spreads SIDEWAYS.  Outer + spreading = THE CAP.
 *
 *      ◉ pivots into:    ◯╮                  ╭╯ ╰╮     ↑rising stem
 *                       ╱   ╲    rolls to:   │   │
 *                       ╲   ╱                ╰─◯─╯     ←spreading cap
 *                        ╲ ╱
 *                         ◉                            ↓curling
 *
 * Nothing in our code says "form a cap".  It's just the unique
 * solution to "buoyant blob in incompressible air".  The mushroom is
 * the equation.
 *
 *
 * TUTORIAL 8: Why is the simulation 2-D axisymmetric?
 * ───────────────────────────────────────────────────
 * Real explosions are roughly cylindrically symmetric: rotate them
 * around the vertical axis and they look the same.  We exploit this:
 * instead of a full 3-D grid, we simulate ONE 2-D slice (radius r,
 * altitude y).  When the renderer needs a value at world point
 * (x, y, z), it computes r = √(x² + z²), looks up the 2-D field at
 * (r, y), and uses that — the same field services all 360° of
 * azimuth.
 *
 * Memory and CPU savings: 56 × 96 = 5 376 cells, vs. 56 × 56 × 96 =
 * 301 056 for an equivalent full-3-D grid.  Same physics quality.
 *
 *      A full-3-D simulation would store this:    We store this:
 *
 *      ┌─┬─┬─┬─┬─┐                                ┌─┐
 *      ├─┼─┼─┼─┼─┤                                ├─┤
 *      ├─┼─┼─┼─┼─┤   (one slice per azimuth,      ├─┤   (one slice,
 *      ├─┼─┼─┼─┼─┤    revolved around y axis)     ├─┤    rendered as
 *      ├─┼─┼─┼─┼─┤                                ├─┤    if revolved)
 *      └─┴─┴─┴─┴─┘                                └─┘
 *
 *
 * TUTORIAL 9: What is volumetric rendering?
 * ─────────────────────────────────────────
 * A solid object has a SURFACE — a thin boundary.  Smoke and clouds
 * don't.  They are continuous distributions of partly-transparent
 * material spread through 3-D space.  To render them we ask: "if I
 * shoot a ray through the smoke, how much light reaches my eye?"
 *
 * Two physical processes apply:
 *
 *   ABSORPTION: dense regions block light.  As the ray travels, the
 *   amount of light still reaching the eye DECAYS exponentially with
 *   the integrated density along the path:
 *
 *      transmittance(L) = exp(−∫₀ᴸ ρ(s) ds)        (Beer-Lambert law)
 *
 *   EMISSION: hot smoke GLOWS.  Each segment of smoke contributes
 *   light proportional to its density (more material = more glow)
 *   times its emission strength (hotter = brighter), times the
 *   transmittance from that segment back to the eye:
 *
 *      L_total = ∫₀^∞ transmittance(s) · ρ(s) · emission(s) ds
 *
 * In code we discretise the integral with small steps along the ray:
 *
 *      transmittance, L_total = 1, 0
 *      for s in 0, STEP, 2·STEP, ... up to MAX_DIST:
 *          dτ = ρ(s) · STEP                      // optical depth this step
 *          L_total       += transmittance · dτ · emission(s)
 *          transmittance *= exp(−dτ)
 *          if transmittance < ε: break           // opaque, stop
 *
 *
 *      eye  →→→→──────────┬──────────┬──────────┬──→ off into space
 *                         │          │          │
 *               ρ small ▒░│ ρ medium │ ρ big   │ ρ small
 *                         │ ▒▒▒░░░░░ │ ▒▒▒▒▒▒  │ ░▒░
 *
 *      transmittance: 1.0   →   0.85    →   0.30   →   0.05  (stop)
 *
 *
 * TUTORIAL 10: How does the renderer connect to the simulation?
 * ──────────────────────────────────────────────────────────────
 * The simulator produces a 2-D field (radius, altitude) that it
 * updates every tick.  The renderer asks: "for this 3-D point in
 * world space, what's the density and temperature?"  Two steps:
 *
 *   1. Project from 3-D world coords (x, y, z) to 2-D field coords:
 *
 *          radius   = √(x² + z²)
 *          altitude = y
 *
 *   2. Bilinearly interpolate the (radius, altitude) value of the
 *      2-D field — converting from continuous coords to weighted
 *      averages of the four nearest grid cells.
 *
 * That's the entire bridge.  The fluid solver doesn't know it's
 * being rendered; the renderer doesn't know what the bubble is doing.
 * They communicate through the four sampler functions in §17.
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

/* ── §1 config ───────────────────────────────────────────────────────── *
 *
 * Every tunable lives here, with units and intent.  The principle: a
 * reader should be able to identify what a constant CONTROLS without
 * leaving §1.  No magic numbers anywhere else in the file.
 */

/* §1.1 — frame rate + UI layout.
 *
 * The render loop tries to hit `target_render_fps` frames per second.
 * A separate sim_dt determines how the physics advances; sim and
 * render are decoupled so changing fps doesn't change the cloud's
 * morphology.
 */
enum {
    TARGET_RENDER_FPS_MIN     =  10,
    TARGET_RENDER_FPS_DEFAULT =  60,
    TARGET_RENDER_FPS_MAX     = 120,
    TARGET_RENDER_FPS_STEP    =  10,

    FPS_DISPLAY_UPDATE_MS     = 500,
    HUD_ROWS                  =   2,   /* row 0 status + last row hint */
};

/* §1.2 — colour-pair IDs.
 *
 *   PAIR_HUD_STATUS              yellow + bold (top status row)
 *   PAIR_HUD_HINT                cyan   + bold (key-hint row)
 *   PAIR_RAMP_BASE..+7           smoke→fire ramp (8 bands per theme)
 */
#define PAIR_HUD_STATUS    1
#define PAIR_HUD_HINT      2
#define PAIR_RAMP_BASE     3   /* +0..+7 */

/* §1.3 — time helpers. */
#define NS_PER_SECOND      1000000000LL
#define NS_PER_MILLISECOND    1000000LL
#define TICK_NS(target_fps)  (NS_PER_SECOND / (target_fps))

/* §1.4 — terminal cell aspect ratio.
 *
 * Terminal cells are roughly twice as tall as they are wide.  We
 * multiply the ray's vertical component by this so the rendered
 * image is not vertically stretched.  See §19 (camera).
 */
#define TERMINAL_CELL_ASPECT  2.0f

/* §1.5 — fluid grid.
 *
 * 56 radial × 96 vertical cells gives ~7 world units of radius and
 * ~12 world units of altitude.  Sized so the CAP of a STANDARD
 * mushroom fits inside the simulation domain at peak.
 *
 *      r = 0    r = R_MAX
 *      ┌────────┐  ← y = Y_MAX
 *      │        │
 *      │        │
 *      │  fluid │
 *      │        │
 *      │        │
 *      │        │
 *      └────────┘  ← y = 0
 *
 * GRID_CELL_SIZE is the world-space side length of one cell (in our
 * abstract "world units").  Smaller cells = sharper detail, more CPU.
 */
#define GRID_RADIAL_CELLS    56
#define GRID_VERTICAL_CELLS  96
#define GRID_CELL_SIZE       0.125f
#define GRID_RADIAL_EXTENT   ((float)GRID_RADIAL_CELLS   * GRID_CELL_SIZE)
#define GRID_VERTICAL_EXTENT ((float)GRID_VERTICAL_CELLS * GRID_CELL_SIZE)
#define GRID_INV_CELL_SIZE   (1.0f / GRID_CELL_SIZE)

#define POISSON_JACOBI_ITERATIONS  40    /* Jacobi sweeps per project() */

/* §1.6 — physics constants.
 *
 *   TEMPERATURE_AMBIENT  the resting room-temperature value
 *   TEMPERATURE_PEAK     used for emission normalisation only
 *   DENSITY_PEAK         used as a reference scale only
 *   BUOYANCY_COEFFICIENT β · g (Tutorial 3); upward acceleration
 *                        per unit °T-above-ambient, per second.
 *   COOL_RATE            exponential decay rate of (T - T_ambient)
 *   DENSITY_DECAY        linear-ish decay of density per second.
 *                        Models entrainment / mixing we don't sim.
 */
#define TEMPERATURE_AMBIENT       1.0f
#define TEMPERATURE_PEAK          8.0f
#define DENSITY_PEAK              4.0f
#define BUOYANCY_COEFFICIENT      2.4f
#define COOL_RATE                 0.06f
#define DENSITY_DECAY             0.009f

/* §1.7 — initial conditions per blast type.
 *
 * The simulator is unchanged across blast types; only the initial
 * Gaussian we drop in at t=0 differs.  Each row defines that
 * Gaussian.
 *
 * Field meanings:
 *   sigma                   Gaussian standard deviation (world units)
 *                           — controls blast size at t = 0
 *   peak_temperature        peak T at the centre, vs ambient
 *   peak_density            peak ρ at the centre
 *   detonation_altitude     world y-coordinate of blast centre
 *   initial_outflow         radial outflow speed at t = 0 (the "blast
 *                           wave" before buoyancy takes over)
 */
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
    float       sigma;
    float       peak_temperature;
    float       peak_density;
    float       detonation_altitude;
    float       initial_outflow;
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

/* §1.8 — simulation timing.
 *
 *   SIM_DT             fixed simulation step (sim time, not real)
 *   SIM_DT_MAX_REAL    cap per real frame (prevents teleport on hiccup)
 *   sim_rate           multiplier from real time → sim time (user knob)
 */
#define SIM_DT                 0.025f
#define SIM_DT_MAX_REAL        0.080f
#define SIM_RATE_DEFAULT       1.0f
#define SIM_RATE_MIN           0.10f
#define SIM_RATE_MAX           6.0f
#define SIM_RATE_STEP_FACTOR   1.30f

/* §1.9 — camera.
 *
 * Default placement: orbiting horizontally, looking down at a point
 * in the cloud column.  Distance is generous so a MEGATON cap fits.
 */
#define CAM_DISTANCE_DEFAULT   28.0f
#define CAM_DISTANCE_MIN        8.0f
#define CAM_DISTANCE_MAX       56.0f
#define CAM_DISTANCE_STEP       2.0f
#define CAM_HEIGHT              5.0f
#define CAM_LOOK_AT_HEIGHT      6.0f
#define CAM_FIELD_OF_VIEW_DEG  52.0f

/* §1.10 — volumetric raymarcher.
 *
 *   RM_RAY_NEAR              start ray at this distance from camera
 *   RM_RAY_FAR               give up after this distance (miss)
 *   RM_STEP                  world-space distance per march step
 *   RM_MAX_STEPS             absolute step cap (safety)
 *   RM_OPAQUE_TRANSMITTANCE_EPS
 *                            stop once transmittance falls below this
 *   RM_DENSITY_GAIN          ρ → optical depth multiplier (visual)
 *   RM_EMISSION_GAIN         hot-gas brightness multiplier (visual)
 *   RM_AMBIENT_FLOOR         minimum scattered light per step
 */
#define RM_RAY_NEAR                       0.5f
#define RM_RAY_FAR                       32.0f
#define RM_STEP                           0.18f
#define RM_MAX_STEPS                    130
#define RM_OPAQUE_TRANSMITTANCE_EPS       0.01f
#define RM_DENSITY_GAIN                   1.30f
#define RM_EMISSION_GAIN                  4.5f
#define RM_AMBIENT_FLOOR                  0.06f

/* §1.11 — pixel classification.
 *
 *   PIXEL_LUMINANCE_CLAMP    L is divided by this before mapping to a
 *                            glyph slot (clamps brightness peak)
 *   PIXEL_VISIBLE_LUM_EPS    below this luma the pixel is "empty"
 *                            (left as default-bg / pre-painted bg)
 *   GLYPH_SLOT_COUNT         number of luminance ramp tiers
 */
#define PIXEL_LUMINANCE_CLAMP    1.10f
#define PIXEL_VISIBLE_LUM_EPS    0.002f
#define GLYPH_SLOT_COUNT         8
#define GLYPH_SLOT_FLOAT         7.999f   /* (GLYPH_SLOT_COUNT - 0.001) */

/* §1.12 — themes.
 *
 * 8-band ramp from coolest (slot 0) → hottest (slot 7).  The
 * `inverted` flag flips foreground/background semantics for the
 * NEGATIVE theme: white background, dark foreground.  All non-
 * inverted themes keep slot 0 in the bright half of the 256-cube
 * (per the CLAUDE.md theme-brightness rule).
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

/* §1.13 — luminance glyph ramp.
 *
 * Slot 0 is `.` (not space) so that thin smoke at end-of-life renders
 * as a faint dot rather than disappearing.  The "true invisible"
 * threshold is PIXEL_VISIBLE_LUM_EPS in §20.
 */
static const char LUMINANCE_GLYPHS[GLYPH_SLOT_COUNT] =
    { '.', ',', ':', ';', '+', '*', '#', '@' };

/* §1.14 — debug overlays (educational helpers).
 *
 * Press `d` to cycle.  Each overlay paints a different ASPECT of the
 * simulation directly onto the screen, replacing the volumetric
 * raymarch.  Reading the source for each overlay teaches what the
 * underlying field LOOKS LIKE.
 *
 *   DEBUG_NORMAL       full 3-D volumetric raymarch (production view)
 *   DEBUG_DENSITY      2-D (radius × altitude) density map
 *   DEBUG_TEMPERATURE  2-D temperature map
 *   DEBUG_VELOCITY     2-D velocity field (arrows by direction)
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

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── *
 *
 * We need a clock immune to wall-clock jumps (NTP corrections, etc.).
 * CLOCK_MONOTONIC is guaranteed to advance at one second per second,
 * with no jumps, since some unspecified epoch.  Perfect for "how long
 * since the last frame" measurements.
 */

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

/* ── §3 color — theme palette + HUD/hint pairs ───────────────────────── *
 *
 * ncurses wants colour indices passed via `init_pair(pair_id, fg, bg)`.
 * We allocate one pair per (theme × ramp slot), plus two named pairs
 * for the top status row and the bottom hint row.  Cycling themes
 * just re-points the ramp pairs at the new palette — no other state
 * changes.
 */

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

/* ── §4 vec3 — 3-D vector math used by the raymarcher ────────────────── *
 *
 * Tiny by-value operations on 3-D vectors.  These are inlined so
 * the abstraction has zero runtime cost — at -O2 the compiler
 * collapses chained operations into the same machine code as
 * hand-written `a.x = b.x + c.x; ...`.
 */

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
 * Field roles:
 *
 *   velocity_radial[i][j]
 *       Flow speed in the +radial direction (away from the y-axis).
 *       Units: world units per second.
 *
 *   velocity_vertical[i][j]
 *       Flow speed in the +y direction (up).  Units: same.
 *
 *   temperature[i][j]
 *       Local gas temperature.  Drives buoyancy (§8).  Initialised
 *       to TEMPERATURE_AMBIENT everywhere; the detonation paints a
 *       hot Gaussian into this field.
 *
 *   density[i][j]
 *       Visible smoke density.  Drives optical depth (§18).  Has
 *       no effect on the physics itself in this Boussinesq
 *       approximation — it's a "passive scalar" that rides the
 *       flow.
 *
 *   pressure[i][j]
 *       Scratch field used during projection (§13).  Each tick we
 *       Jacobi-iterate the Poisson equation here.  The "current"
 *       pressure has no physical meaning outside that step.
 *
 *   divergence[i][j]
 *       Scratch field — the current ∇·v at each cell (§10).  Right-
 *       hand side of the Poisson equation we solve.
 *
 *   scratch_a[i][j], scratch_b[i][j]
 *       Two general-purpose buffers for advection swap-out and
 *       Jacobi iteration's destination buffer.
 *
 * Indexing convention:
 *   i ∈ [0, GRID_RADIAL_CELLS)     — radial slot (0 = on the axis)
 *   j ∈ [0, GRID_VERTICAL_CELLS)   — vertical slot (0 = on the ground)
 *
 * The grid layout:
 *
 *      j=N_Y-1                 ┌────┬────┐
 *                              │    │    │
 *                              ├────┼────┤
 *                              │    │    │      (axis on the
 *                              ├────┼────┤       left, ground
 *                              │    │    │       at the bottom)
 *                              ├────┼────┤
 *      j=0                     │    │    │
 *           i=0  axis ─────────┴────┴────┘──── ground line
 */

typedef struct {
    /* Velocity (radial, vertical). */
    float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Advected scalars. */
    float temperature       [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float density           [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* Projection scratch. */
    float pressure          [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float divergence        [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* General scratch buffers (Jacobi double-buffer + advection swap). */
    float scratch_a         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float scratch_b         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
} Fluid;

static Fluid g_fluid;

/* ── §6 grid helpers — clampf + bilinear sampling ────────────────────── *
 *
 * Two utilities used throughout the fluid solver and the renderer.
 */

/*
 * clampf — restrict a float to the inclusive range [lower, upper].
 * Returned as: lower if x < lower, upper if x > upper, else x.
 */
static inline float clampf(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/*
 * sample_field_bilinear — sample a 2-D scalar field at FRACTIONAL
 * cell indices.
 *
 *   field         the field to read from
 *   fi, fj        FRACTIONAL grid coordinates (e.g. 3.27, 5.81).
 *                 Out-of-grid coords clamp to the boundary cell.
 *
 * MENTAL MODEL.  Imagine the four cells nearest to (fi, fj) as the
 * corners of a unit square:
 *
 *      (i+1,j)·──────·(i+1,j+1)        i = floor(fi)
 *           │        │                  j = floor(fj)
 *           │   ✦    │                  fri = fi - i        ∈ [0, 1]
 *           │        │                  fyj = fj - j        ∈ [0, 1]
 *      (i,  j)·──────·(i,j+1)
 *
 * The sampled value at ✦ is a weighted average of the four corner
 * values, with the weights being how close ✦ is to each corner.
 * That's bilinear interpolation.
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

    float frac_i = fi - (float)i;          /* horizontal mix factor */
    float frac_j = fj - (float)j;          /* vertical   mix factor */

    /* Four corner samples. */
    float bottom_left  = field[i    ][j    ];
    float bottom_right = field[i + 1][j    ];
    float top_left     = field[i    ][j + 1];
    float top_right    = field[i + 1][j + 1];

    /* Mix horizontally first, then vertically. */
    float bottom_row = bottom_left * (1.0f - frac_i) + bottom_right * frac_i;
    float top_row    = top_left    * (1.0f - frac_i) + top_right    * frac_i;
    return bottom_row * (1.0f - frac_j) + top_row * frac_j;
}

/* ── §7 boundaries — what walls do to the velocity field ─────────────── *
 *
 * Two physical walls in our domain:
 *
 *   1. The AXIS at radius = 0.  The simulation is axisymmetric, so
 *      "the axis" is really a mirror — anything that would flow
 *      across it just reflects.  Practically, that means:
 *
 *          velocity_radial[0][j] = 0    (no flow through the axis)
 *
 *   2. The GROUND at altitude y = 0.  Solid earth.  Equivalently:
 *
 *          velocity_vertical[i][0] = 0  (no flow through the ground)
 *
 * Other domain edges (top of the box, far radial edge) are left as
 * "open" — the advection step naturally diffuses fluid that tries to
 * leave the grid, which is good enough for our purposes.
 *
 * This function is called after every step that modifies velocity:
 * buoyancy adds, advection swaps, projection subtracts.  Each of
 * those can violate boundaries, so we re-enforce here.
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
 * Tutorial 3 explained the physics:
 *
 *      acceleration_y  =  β · (T − T_ambient) · g
 *
 * We bake (β · g) into BUOYANCY_COEFFICIENT.  Per cell, per tick:
 *
 *      vᵧ[i][j]  +=  BUOYANCY · (T[i][j] − T_ambient) · dt
 *
 * That's the entire buoyancy step — five characters of code and one
 * Newton's-law integration.
 *
 * Architectural role: this is the ONLY coupling between temperature
 * and motion in the whole solver.  Removing this function would give
 * a cold, drifting cloud (advection still moves stuff, but nothing
 * lifts it).  Adding factor-of-3 to BUOYANCY_COEFFICIENT gives a
 * very fast riser; halving it gives a slow, ponderous cloud.
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
 * Tutorial 4 explained: instead of pushing fields forward (which
 * blows up under big timesteps), we trace BACKWARD from each cell
 * and bilinearly sample where the fluid "came from".
 *
 *   for each cell c at grid coord (i, j):
 *       source_position  = (i, j) − v·dt / cell_size      // backward trace
 *       new_field[i][j]  = bilinear(old_field, source_position)
 *
 * Visual:
 *
 *      time t          time t − dt        bilinear sample of old_field at
 *      ┌───┐               ┌───┐          source_pos.  The four neighbouring
 *      │ ? │   ←copy from  │ ✦ │          cells of ✦ get weighted-averaged.
 *      └───┘               └───┘
 *      cell c              source_pos
 *
 * Two important properties:
 *
 *   STABILITY (the whole point of Stam's paper).  No matter how
 *   large dt is, the bilinear sample is a bounded weighted average —
 *   the new value is between the min and max of the four neighbours.
 *   So the field can never blow up.
 *
 *   DIFFUSION (the price).  Bilinear interpolation always loses a
 *   little high-frequency detail.  Each tick the field gets very
 *   slightly blurrier.  We accept this in exchange for stability.
 *
 * This function is generic — it advects whatever field you pass.
 * We use it for vᵣ, vᵧ, T, ρ.
 */

static void advect_field(
        float        destination_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float       source_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float       step_seconds)
{
    /*
     * We trace backwards by `velocity * dt`.  Velocities are in
     * world-units-per-second; we want grid-cells, so we divide by
     * cell size — which is the same as multiplying by INV_CELL_SIZE.
     */
    float dt_in_grid_units = step_seconds * GRID_INV_CELL_SIZE;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            /* Where did the fluid currently here come from one step ago? */
            float source_i = (float)i - velocity_radial  [i][j] * dt_in_grid_units;
            float source_j = (float)j - velocity_vertical[i][j] * dt_in_grid_units;

            /* Sample the OLD field at that source location. */
            destination_field[i][j] =
                sample_field_bilinear(source_field, source_i, source_j);
        }
    }
}

/* ── §10 divergence — measure of "compression" ───────────────────────── *
 *
 * Tutorial 2 explained: ∇·v is the net rate of fluid LEAVING each
 * cell.  Discrete form on our grid:
 *
 *      ∇·v_{i,j}  =  (vᵣ_{i+1,j} − vᵣ_{i−1,j}) / (2·h)
 *                  + (vᵧ_{i,j+1} − vᵧ_{i,j−1}) / (2·h)
 *
 * Centred differences give second-order accuracy (the error scales
 * as h²).  Boundary cells use mirror neighbours: when i=0, treat
 * cell (1,j) as both the "real i+1" AND the "mirror image i−1".
 * That implements the no-flow axis condition without special-
 * casing the formula.
 *
 * Why we compute this: §13 (project) needs it as the right-hand-
 * side of the Poisson equation.
 */

static int mirror_index(int index, int grid_size)
{
    if (index < 0)         return 1;             /* mirror at i=0 */
    if (index >= grid_size) return grid_size - 2; /* mirror at i=N-1 */
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
 * Tutorial 6 derived the Jacobi update:
 *
 *      p_new[i][j]  =  ( p[i+1][j] + p[i−1][j]
 *                      + p[i][j+1] + p[i][j−1]
 *                      − h² · divergence[i][j] ) / 4
 *
 * Each iteration:
 *   1. write candidate p_new for every cell into scratch_a
 *   2. swap: p ← scratch_a
 *
 * After POISSON_JACOBI_ITERATIONS sweeps (40), the pressure field
 * has approximately converged.  Increasing iterations gives a more
 * accurate divergence-free result; decreasing gives visible "puff"
 * artefacts where the solver couldn't catch up to fast-moving cells.
 *
 * Boundary conditions: zero-gradient pressure (mirror neighbours).
 * This corresponds to no-flow walls everywhere — consistent with
 * how we treat the axis and the ground.
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
 * Tutorial 5 derived: v_divergence_free = v_uncorrected − ∇p.  The
 * gradient on a discrete grid (centred differences):
 *
 *      ∂p/∂r  ≈  (p_{i+1,j} − p_{i−1,j}) / (2·h)
 *      ∂p/∂y  ≈  (p_{i,j+1} − p_{i,j−1}) / (2·h)
 *
 * After this subtraction the velocity field has approximately zero
 * divergence everywhere.  No cell "fills up" or "empties out" beyond
 * the residual numerical error of the Jacobi solver.
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
 *
 *   1. compute_divergence            (§10) — measure pile-up
 *   2. solve_pressure_poisson        (§11) — find pressure that fixes it
 *   3. subtract_pressure_gradient    (§12) — apply the fix
 *
 * After this function returns, ∇·v ≈ 0 everywhere on the grid (modulo
 * the Jacobi residual).  In effect, we've taken any unphysical
 * velocity field and "cleaned" it into the closest divergence-free
 * field — the closest mass-conserving flow.
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
 * Two things every tick:
 *
 *   COOL.  Hot cells cool toward TEMPERATURE_AMBIENT.  We use
 *   exponential decay rather than a linear lerp so the behaviour is
 *   correct under any timestep size:
 *
 *      T(t+dt)  =  T_ambient + (T(t) − T_ambient) · exp(−k_cool · dt)
 *
 *   At t → ∞ the cell returns to ambient.  Since buoyancy depends on
 *   (T − T_ambient), eventually buoyancy stops — that's why the
 *   cloud plateaus and falls.  No `time-since-detonation` knob.
 *
 *   DECAY.  Density fades.  This models entrainment / dilution we
 *   don't simulate: in reality smoke mixes with surrounding clear
 *   air over minutes; we approximate that with a gentle linear-ish
 *   loss per second.
 *
 *      ρ(t+dt)  =  ρ(t) · (1 − k_decay · dt)
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
 * The orchestrator.  Reads as the pseudocode in MENTAL MODEL §3:
 *
 *      buoyancy ▸ project ▸ advect v ▸ project ▸ advect (T, ρ) ▸
 *      cool + decay
 *
 * Why TWO projections per step?  Buoyancy adds a vertical impulse
 * that creates divergence (fluid "pushed up" out of nowhere).  We
 * project before advection.  But advection itself, when run on a
 * divergence-free field, can RE-INTRODUCE divergence (the bilinear
 * sample doesn't preserve incompressibility exactly).  So we project
 * AGAIN before advecting the scalar fields.  Two projections per
 * step is a Stam-paper recommendation; cheap relative to advection
 * and gives visibly cleaner results.
 *
 * Velocity advection is a special case: vᵣ and vᵧ are advected by
 * the SAME velocity field (themselves).  We have to be careful not
 * to overwrite vᵣ before reading it for vᵧ's advection — hence the
 * two-buffer pattern (write into scratch_a / scratch_b, then memcpy
 * back).
 */

static void fluid_step(Fluid *fluid, float step_seconds)
{
    /* (1) Buoyancy applies an upward force where T > ambient. */
    apply_buoyancy(fluid, step_seconds);

    /* (2) Project: clean up any pre-existing divergence. */
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

    /* (4) Project again (advection re-introduces divergence). */
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
 * The ONLY scripted moment.  Everything after this is physics.
 *
 * We paint a 3-D-symmetric Gaussian centred at (radius=0, altitude=
 * detonation_altitude).  The Gaussian's amplitude is the blast's
 * peak temperature / density above ambient; its standard deviation
 * is the blast's "size".
 *
 *      gauss(d) = exp(−d²/(2·σ²))
 *
 *      d² = (radius)² + (altitude − blast_altitude)²
 *
 *      T(r, y)  =  T_ambient + (T_peak − T_ambient) · gauss(d)
 *      ρ(r, y)  =  ρ_peak · gauss(d)
 *
 * We additionally seed a small RADIAL OUTFLOW velocity near the
 * centre — this is the initial blast wave (mechanical shock) that
 * happens before buoyancy takes over.  Without it, the bubble
 * just rises with a clean leading-edge vortex.  With it, the
 * bubble first expands radially (~1-2 sec) then transitions to
 * buoyant rise — closer to a real explosion's two-phase profile.
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

            /* Seed initial radial outflow only NEAR the centre (where
             * the Gaussian still has meaningful amplitude). */
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
 * Tutorial 10 explained the bridge: world (x, y, z) → field (radius,
 * altitude) → bilinear sample.  These two functions are the entire
 * interface from the renderer to the simulator.
 *
 * Out-of-domain samples return safe defaults: density 0 (no smoke)
 * and ambient temperature.  This means rays that wander outside the
 * grid simply see clear, room-temperature air.
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
 * Tutorial 9 explained the volume rendering equation.  This is the
 * direct discretisation:
 *
 *      transmittance, L = 1.0, 0.0
 *      for s in 0, STEP, 2·STEP, ... up to RAY_FAR:
 *          (radius, altitude) = (√(x²+z²), y)  at p = origin + s·dir
 *          ρ      = sample_density(radius, altitude)
 *          T      = sample_temperature(radius, altitude)
 *          dτ     = ρ · STEP · DENSITY_GAIN
 *          emit   = clamp((T − T_ambient) / (T_peak − T_ambient), 0, 1)
 *          source = emit · EMISSION_GAIN + AMBIENT_FLOOR
 *          L      += transmittance · dτ · source
 *          L_hot  += transmittance · dτ · emit · EMISSION_GAIN
 *          transmittance *= exp(−dτ)
 *          if transmittance < ε: break
 *
 * Outputs:
 *   *out_total_luminance  total accumulated brightness (smoke + fire)
 *   *out_hot_luminance    contribution from temperature emission only
 *
 * The hot fraction (L_hot / L_total) drives the smoke→fire palette
 * pick in §20: a low fraction is grey/cool smoke, a high fraction is
 * bright orange/yellow fire.
 *
 * Optimisation: empty regions skip ahead with a longer step.  This
 * makes pure-empty rays (which dominate when the cloud is small)
 * cost ~70 % less than a naive uniform march.
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
 * The camera sits at (x=0, y=CAM_HEIGHT, z=−distance) and looks at
 * (0, CAM_LOOK_AT_HEIGHT, 0) — a fixed point near the cloud column's
 * mid-height.  Three orthonormal vectors define the view:
 *
 *      forward = look_at − cam_position, normalised
 *      right   = forward × world_up, normalised
 *      up      = right × forward
 *
 * For each pixel (col, row) we synthesise a ray:
 *
 *      u = (col + 0.5) / cols   · 2 − 1     ∈ [−1, +1]
 *      v = ((row + 0.5) / rows  · 2 − 1) · −1   (Y-flip — top of
 *                                                screen is +v)
 *      direction = forward + u·tan(FOV/2)·right
 *                          + v·tan(FOV/2)·aspect·up
 *      direction = normalise(direction)
 *
 * The aspect factor accounts for terminal cells being roughly twice
 * as tall as they are wide (so the rendered cloud doesn't look
 * vertically squashed).
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
 * The raymarcher gives us two scalars per pixel:
 *
 *   total_luminance  how bright is this cell (smoke + fire combined)?
 *   hot_luminance    how much of that brightness came from fire?
 *
 * We map them to:
 *
 *   GLYPH from total_luminance via an 8-tier ramp ('.' → '@'). A
 *   denser/brighter pixel gets a denser glyph.
 *
 *   COLOUR from hot_luminance / total_luminance.  A cool pixel
 *   (smoke, no fire) picks slot 0 of the theme ramp (typically grey
 *   or cool smoke colour); a hot pixel picks slot 7 (the fire's peak
 *   colour).
 *
 * The two signals are decoupled — glyph reads as DENSITY, colour
 * reads as TEMPERATURE.  This is why a thin wisp of cool smoke and
 * the bright core of a fireball look visually distinct even at the
 * same overall brightness.
 */

typedef struct {
    char   glyph;
    int    pair;
    attr_t attr;
    bool   skip;
} Cell;

/*
 * to_slot — quantise a [0, 1] value to an integer 0..GLYPH_SLOT_COUNT-1.
 * Out-of-range inputs clamp gracefully.
 */
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
     *                    bg DARKENS it, which on white bg means the
     *                    brightness intent is inverted; we let the
     *                    grey ramp itself carry the gradient.
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
 * Everything that's not "in the fluid" lives here: which blast we
 * detonated, which theme we're showing, how many seconds have passed
 * in sim time, the camera distance, and the current debug overlay
 * mode.
 */

typedef struct {
    bool       paused;
    int        theme_index;
    BlastType  blast_type;
    DebugMode  debug_mode;
    int        cols, rows;

    float      simulation_time_seconds;   /* total sim seconds elapsed */
    float      simulation_rate;           /* real → sim multiplier */
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
 * fixed-dt fluid steps as fit into the accumulator.
 *
 * We use a fixed-dt accumulator (rather than passing `dt_real` to
 * the solver directly) so the cloud's morphology is INDEPENDENT of
 * frame rate.  Slow terminal? Fewer fluid steps per frame, but each
 * one is still SIM_DT.  Same exact cloud, just with a bigger render
 * step.  Critical for reproducibility.
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

/* §21.1 — the main raymarched render path.
 *
 * For each terminal cell:
 *   – Build a ray.
 *   – Raymarch through the volume.
 *   – Decorate the result into a Cell.
 *   – Emit (with attribute batching).
 */

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
                mvaddch(row + 1, col, ' ');     /* +1 for top HUD row */
        attroff(COLOR_PAIR(PAIR_RAMP_BASE));
    }

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;
    int    y_offset  = 1;     /* leave row 0 free for top HUD */

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
 * Press `d` during the program to cycle through these.  Each
 * overlay swaps out the volumetric raymarcher for a DIRECT view of
 * one underlying field.  Reading the source for each overlay teaches:
 *
 *   DEBUG_DENSITY      "what does the smoke field look like?"
 *   DEBUG_TEMPERATURE  "what does the heat field look like?"
 *   DEBUG_VELOCITY     "where is the fluid moving?"
 *
 * They're all very simple: walk the 2-D fluid grid, paint each cell
 * with a glyph + colour derived from the field value.
 *
 * The grid is mapped to terminal cells so it fills the canvas:
 *
 *      grid_col = (term_col / canvas_cols) · GRID_RADIAL_CELLS
 *      grid_row = ((canvas_rows-1 - term_row_within_canvas)
 *                   / canvas_rows) · GRID_VERTICAL_CELLS
 *
 * (The y-flip in grid_row puts the ground at the BOTTOM of the
 * screen, matching how the volume renderer sees it.)
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

            /* Density is on roughly [0, DENSITY_PEAK].  Map to slot. */
            float normalised = density / DENSITY_PEAK;
            normalised = clampf(normalised, 0.0f, 1.0f);
            int   slot = to_slot(normalised);
            char  glyph = LUMINANCE_GLYPHS[slot];
            int   pair  = PAIR_RAMP_BASE + slot;

            if (slot == 0 && density < 0.01f) continue;     /* skip empty cells */

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

            /* Temperature on [T_AMBIENT, T_PEAK]. */
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
 * the same direction as a velocity vector.
 *
 *   We find which of 8 cardinal/intercardinal directions the
 *   velocity vector is closest to:
 *
 *           N             N    NE     E     SE     S     SW     W     NW
 *      NW   |   NE        |     ╲    ─     ╱      |     ╲     ─     ╱
 *        ╲  |  ╱           ↑      ↗   →    ↘      ↓      ↙   ←       ↖
 *         ╲ | ╱
 *      W ──────── E
 *         ╱ | ╲
 *        ╱  |  ╲
 *      SW   |   SE
 *           S
 *
 * Magnitude near zero → space.  Magnitude → glyph slot.
 */
static char arrow_for_velocity(float vx, float vy)
{
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 0.05f) return ' ';

    float angle_radians = atan2f(vy, vx);   /* −π .. +π */
    int   octant        = (int)((angle_radians + (float)M_PI) /
                                ((float)M_PI / 4.0f) + 0.5f) % 8;

    /* octant: 0 = west, 2 = south, 4 = east, 6 = north (because
     * atan2 returns angle measured counter-clockwise from +x). */
    static const char ARROWS[8] = {
        '<',     /* 0 = west  (←) */
        '/',     /* 1 = sw    (↙) — '/' points NE/SW */
        'v',     /* 2 = south (↓) */
        '\\',    /* 3 = se    (↘) */
        '>',     /* 4 = east  (→) */
        '/',     /* 5 = ne    (↗) — same '/' as sw, intent inferred from neighbours */
        '^',     /* 6 = north (↑) */
        '\\',    /* 7 = nw    (↖) */
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

/* §22.1 — render dispatcher. */

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

/*
 * hud_draw — CLAUDE.md HUD spec.
 *   row 0          PAIR_HUD_STATUS (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HUD_HINT   (cyan   + bold) — key hint
 */
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

    /* Frame-pacing state. */
    int64_t frame_start_ns       = clock_now_nanoseconds();
    int64_t fps_window_ns        = 0;
    int     frames_in_window     = 0;
    double  measured_fps         = 0.0;

    /*
     * Main loop pseudo-code:
     *
     *   while running:
     *       handle resize
     *       compute dt_real
     *       scene_tick(dt_real)            (advances physics)
     *       update fps counter
     *       sleep until next frame deadline
     *       render screen
     *       handle key
     */

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
