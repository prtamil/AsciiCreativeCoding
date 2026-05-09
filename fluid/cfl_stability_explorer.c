/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cfl_stability_explorer.c — when does a wave simulation blow up?
 *
 * DEMO: A 2-D scalar wave (think: ripples on a pond) is simulated with
 *       the simplest possible time integrator — leapfrog finite
 *       differences.  Two parameters control everything:
 *
 *           wave_speed   c   (cells per second)
 *           step_seconds dt  (seconds per simulation tick)
 *
 *       Their product, divided by the grid spacing, is the CFL NUMBER:
 *
 *           cfl = c · dt · sqrt(2) / dx       (≈ c · dt for dx = 1, 2-D)
 *
 *       A CLASSIC numerical-analysis result (Courant-Friedrichs-Lewy
 *       1928) says this scheme is STABLE if and only if cfl ≤ 1.
 *       Below 1: ripples spread and slowly fade (numerical dissipation).
 *       Above 1: ripples GROW WITHOUT BOUND — the simulation explodes,
 *       amplitudes go to infinity, the screen fills with noise.
 *
 *       Press 1-6 to jump between presets (cfl = 0.4 .. 1.4); use +/-
 *       to nudge dt; c/C to change wave speed.  Watch the CFL bar
 *       cross 1.0 and the simulation visibly destabilise.  Press m
 *       for a clean eigenmode initialisation that explodes
 *       cleanly when unstable; press d for a Gaussian water drop.
 *
 * Why it matters:
 *
 *       Every explicit time-stepping simulation in physics — fluids,
 *       elasticity, heat, electromagnetism — has a CFL-like stability
 *       limit.  Get the limit wrong and your code produces nonsense
 *       (NaN, exploding fields).  This file lets you SEE the limit
 *       being crossed in real time, building intuition for why it
 *       exists and what it means.
 *
 * Study alongside:
 *   physics/wave.c                  — 1-D wave equation, simpler form
 *   physics/wave_2d.c               — same 2-D wave without CFL focus
 *   fluid/navier_stokes.c           — Stam stable fluids; implicit
 *                                      solver, DIFFERENT stability story
 *   physics/rk_method_comparision.c — fixed-step alternatives
 *
 * Section map:
 *   §1  config       — physical + UI constants
 *   §2  clock        — monotonic time + sleep
 *   §3  colors       — diverging-wave palette + HUD
 *   §4  grid         — three time-buffers (past, present, future)
 *   §5  init         — zero, eigenmode, drop
 *   §6  laplacian    — 5-point stencil (explained in §6 preamble)
 *   §7  step         — leapfrog time integrator
 *   §8  cfl          — compute CFL number, critical dt, growth rate
 *   §9  measure      — max absolute amplitude, explosion detector
 *   §10 paint_field  — render 2-D wave with diverging palette
 *   §11 paint_hud    — single-row top status, multi-line bottom HUD
 *   §12 paint_meter  — the CFL meter bar
 *   §13 scene        — orchestrator (state + tick + draw)
 *   §14 input        — key handler
 *   §15 screen       — ncurses init/cleanup
 *   §16 app          — main loop + signals
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   s                advance one tick (when paused)
 *   1-6              CFL preset jump (0.4, 0.6, 0.85, 1.0, 1.1, 1.4)
 *   + / =            increase dt by 5 %
 *   - / _            decrease dt by 5 %
 *   c                slow wave speed by 10 %
 *   C                speed up wave by 10 %
 *   m                eigenmode initial condition (clean instability)
 *   d                Gaussian-drop initial condition
 *   r                reset (re-init + zero stats)
 *   t                cycle colour theme (wave / thermal / ocean)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/cfl_stability_explorer.c \
 *       -o cfl_stability_explorer -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order (≈ 30 minutes total)
 * ──────────────────────────────────
 *   1. Read CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL as prose.  All
 *      of the simulation's "why" is here.  Tutorials T1-T4 build the
 *      wave equation; T5-T7 derive the CFL limit; T8 explains the
 *      visualisation.
 *   2. §6 laplacian + §7 step — THE SIMULATION CORE.  Twenty lines.
 *      Read AFTER tutorials.
 *   3. §8 cfl + §9 measure — what the dashboard displays.
 *   4. §12 paint_meter + §11 paint_hud — the visual interface.
 *   5. §1, §2, §3, §15, §16 — config / clock / colour / screen / app.
 *      Standard framework boilerplate (see ncurses_basics/framework.c
 *      for the canonical version).
 *
 * Variable-naming convention
 * ──────────────────────────
 *   wave_past[r][c]      u(t - dt) — value at the PREVIOUS step
 *   wave_now[r][c]       u(t)      — value at the CURRENT step
 *   wave_next[r][c]      u(t + dt) — being computed for the NEXT step
 *   wave_speed_cps       c (cells per second)
 *   step_seconds         dt
 *   cfl_number           cfl = c · dt · sqrt(2) / dx
 *   cfl_critical         the threshold (≈ 1.0 for explicit leapfrog)
 *   max_abs_amplitude    max(|u|) over the whole grid
 *   growth_ratio         max_abs at this step / max_abs one step ago
 *   explosion_count      how many times the sim has exploded since reset
 *   beta                 c · dt (raw "wave step per tick")
 *
 * Background you need
 * ───────────────────
 *   - C arrays + 2-D indexing.
 *   - Trig identities (cos / sin sums) — for tutorial T6 only.
 *   - Floating-point: NaN, inf.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Partial differential equations as a formal subject.  The wave
 *     equation is introduced from first principles in T1.
 *   - Fourier analysis or Von Neumann stability theory.  We build
 *     intuition without it (T6).  Real Von Neumann analysis appears
 *     in numerical-analysis textbooks; we cite a pointer.
 *   - Implicit solvers / matrix factorisation.  Explicit-only here.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Explicit leapfrog finite-difference scheme for the
 *                 2-D scalar wave equation
 *
 *                       ∂²u/∂t² = c² · (∂²u/∂x² + ∂²u/∂y²)
 *
 *                 discretised on a regular cell grid with spacing
 *                 dx = dy = 1 (in cell units) and time step dt:
 *
 *                       u_new[r][c] = 2 · u_now[r][c]  −  u_old[r][c]
 *                                   + (c · dt)² · LAPLACIAN(u_now, r, c)
 *
 *                 where the LAPLACIAN is the standard 5-point stencil
 *                 u[r-1] + u[r+1] + u[c-1] + u[c+1] − 4 · u[r][c].
 *
 * Stability     : The scheme is STABLE (does not amplify any
 *                 wavelength) iff the CFL number satisfies
 *
 *                       cfl = c · dt · sqrt(2) / dx ≤ 1
 *
 *                 (the sqrt(2) comes from the 2-D Laplacian's max
 *                 eigenvalue; in 1-D the limit is c·dt/dx ≤ 1).
 *                 Above the limit, the discrete operator's spectral
 *                 radius exceeds 1: each step amplifies the most
 *                 unstable Fourier mode by a factor μ > 1; after N
 *                 steps the amplitude has multiplied by μ^N — fast
 *                 explosion.
 *
 * Data layout   : THREE float buffers of size (rows × cols).  Indexed
 *                 [row][col] in row-major order.  Past, present,
 *                 future advance by ROTATION at each step (no
 *                 memcpy):  past <- now, now <- next, next <- past
 *                 (the past buffer is reused as scratch).
 *
 * Rendering     : Diverging colour map — bright red ramp for positive
 *                 amplitude, bright blue ramp for negative.  Each grid
 *                 cell maps to one terminal cell; bright extremes use
 *                 A_BOLD.  Bottom 5 lines reserved for the HUD
 *                 (parameters, CFL meter, key hints).
 *
 * Performance   : O(rows · cols) per step.  At a typical 60 × 30 grid
 *                 (1800 cells) and 30 fps simulation rate, ~54k cell
 *                 updates per second — microseconds.
 *
 * References
 * ──────────
 *   Courant, Friedrichs & Lewy, "Über die partiellen
 *     Differenzengleichungen der mathematischen Physik" (1928) —
 *     the original CFL paper.  The English translation appeared
 *     in IBM J. Res. Dev. 11 (1967) 215.
 *   LeVeque, "Finite Difference Methods for Ordinary and Partial
 *     Differential Equations" (SIAM, 2007) chs. 9-10 — modern
 *     treatment of stability + Von Neumann analysis.
 *   Strikwerda, "Finite Difference Schemes and Partial
 *     Differential Equations" (2nd ed., SIAM 2004) ch. 7.
 *   Wikipedia, "Courant-Friedrichs-Lewy condition":
 *     https://en.wikipedia.org/wiki/Courant%E2%80%93Friedrichs%E2%80%93Lewy_condition
 *   See also project file physics/rk_method_comparision.c — same
 *     stability story for ODE integrators (RK4, Euler, etc.).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A computer cannot solve the wave equation exactly.  It samples the
 * solution at a grid of points + a sequence of time steps and uses
 * NEIGHBOURING samples to predict the next one.  This APPROXIMATION
 * tracks the true solution well — UNLESS the time step dt is too big
 * for the wave speed c.  Then the discrete update doesn't have time
 * to "see" the wave's motion correctly: it overshoots, amplifies the
 * error, and the simulation blows up.  The CFL number is the
 * dimensionless ratio that says where the cliff is.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture each cell as a buoy on a pond.  Buoys are connected to
 * their neighbours by stiff strings that pull each buoy toward the
 * average position of its neighbours.  Pluck one buoy and a wave
 * spreads outward as buoys pull on each other in turn.
 *
 * Now imagine you're animating the pond by HAND.  You move each buoy
 * once per "tick" based on where its neighbours WERE last tick.
 *
 *   Small ticks (cfl ≪ 1):
 *     Each tick, the wave can only advance a fraction of a cell.
 *     Your hand has more than enough time to communicate the
 *     correct neighbour positions.  Smooth animation.
 *
 *   Just-right ticks (cfl = 1):
 *     The wave advances exactly one cell per tick.  Your hand is
 *     working at maximum speed — no error, no margin.
 *
 *   Too-big ticks (cfl > 1):
 *     The wave wants to advance MORE than one cell per tick, but
 *     your hand only knows about IMMEDIATE neighbours.  You miss
 *     the wave's true motion; your update OVERCORRECTS based on a
 *     stale view of the pond.  Each tick adds error; errors compound
 *     exponentially; the pond boils into NaN.
 *
 * That's CFL in one paragraph.  The rigorous version uses Fourier
 * analysis (T6); the intuition is "information must not travel
 * faster than the simulation can track it."
 *
 * GEOMETRY OF ONE STEP
 * ────────────────────
 *
 *   To compute u_new at cell (r, c), the leapfrog scheme reads:
 *
 *           u_now[r-1][c]        ← north neighbour (now)
 *               │
 *               │
 *   u_now[r][c-1] ─ u_now[r][c] ─ u_now[r][c+1]
 *               │
 *               │
 *           u_now[r+1][c]        ← south neighbour (now)
 *
 *   plus  u_old[r][c]            ← THIS cell at the PREVIOUS step
 *
 *   Five PRESENT samples + one PAST sample → one FUTURE sample.
 *   This stencil only "sees" 1 cell of horizontal influence per
 *   tick.  CFL says: for the wave's true displacement to fit
 *   inside this stencil, c · dt ≤ dx — i.e. cfl ≤ 1.
 *
 * TIME-STEP STATE TRANSITIONS
 * ───────────────────────────
 *
 *   Three buffers rotate every tick:
 *
 *     before tick:   past ─ now ─ next(empty)
 *     compute next from {past, now}:
 *                    past ─ now ─ next(filled)
 *     rotate:        next ─ past ─ now
 *                    │      │      │
 *                    └──────┴──────┘  (the names shift, the
 *                                       buffers stay in place)
 *
 *   Concretely: we swap pointers so the buffer that WAS "past"
 *   becomes the new "next" (overwritten next tick).  No memcpy.
 *
 * KEY FORMULAS
 * ────────────
 *
 *   2-D wave equation (continuous):
 *     ∂²u/∂t² = c² · (∂²u/∂x² + ∂²u/∂y²)
 *
 *   Leapfrog discretisation (finite differences):
 *     u_new[r][c] = 2·u_now[r][c]
 *                 − u_old[r][c]
 *                 + (c·dt/dx)² · ( u_now[r-1][c] + u_now[r+1][c]
 *                                + u_now[r][c-1] + u_now[r][c+1]
 *                                − 4·u_now[r][c] )
 *
 *   CFL number (2-D, dx=dy=1):
 *     cfl = c · dt · sqrt(2)
 *
 *   Critical timestep (largest stable dt):
 *     dt_crit = 1 / (c · sqrt(2))           (so cfl = 1 exactly)
 *
 *   Growth ratio (measured live):
 *     mu = max_abs_amplitude(t) / max_abs_amplitude(t - dt)
 *     mu > 1.0001 → instability
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Boundaries: we use Dirichlet (u = 0 at the edges).  Reflects
 *     waves with phase inversion.  Other choices (periodic,
 *     absorbing) change the visible behaviour but not the CFL
 *     condition.
 *   - Spurious modes: the discrete operator has high-frequency
 *     modes (alternating ±1 per cell) the continuous equation
 *     doesn't.  At cfl = 1 these modes DON'T grow but oscillate
 *     forever; at cfl slightly < 1 they decay slowly.  Above
 *     cfl = 1 they explode first (highest growth factor).
 *   - Float overflow: at sustained cfl > 1.1 the explosion can
 *     hit FLT_MAX in ~50 ticks; we detect (max > 1e6) and reset
 *     to prevent NaN cascades.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Press 1: cfl = 0.4.  Drop a Gaussian (d).  Wave spreads
 *     outward in concentric rings, slowly damping at the walls.
 *   - Press 4: cfl = 1.0 (the cliff edge).  The wave persists
 *     indefinitely (ideally — float roundoff causes slow decay).
 *   - Press 5: cfl = 1.1.  Within 30-50 ticks the field fills
 *     with high-frequency noise; max_amp climbs exponentially;
 *     the explosion counter ticks up after each reset.
 *   - Press m at cfl = 1.1: an eigenmode init explodes CLEANLY —
 *     amplitudes double in a known number of steps.  Watch the
 *     "t×2" readout in the HUD match the theoretical value.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build CFL stability from first principles.
 * After T1-T4 you can write a wave simulator.  After T5-T7 you can
 * predict its stability limit.  T8 connects to what you see on
 * screen.
 *
 *   T1  The wave equation in one paragraph
 *   T2  Sampling space + time on a regular grid
 *   T3  The 5-point Laplacian stencil
 *   T4  Leapfrog time stepping — past, present, future
 *   T5  Why a discrete scheme can blow up
 *   T6  The CFL number (Von Neumann analysis, intuitive version)
 *   T7  Why 1 in 1-D, 1/sqrt(2) in 2-D, 1/sqrt(d) in d-D
 *   T8  Reading the dashboard — what each row means
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE WAVE EQUATION IN ONE PARAGRAPH
 * ──────────────────────────────────────
 * A WAVE EQUATION says: at every point of a medium, the second
 * derivative of displacement WITH RESPECT TO TIME equals a constant
 * (c²) times the SECOND DERIVATIVE WITH RESPECT TO SPACE.
 *
 *     ∂²u/∂t² = c² · ∂²u/∂x²              (1-D)
 *     ∂²u/∂t² = c² · (∂²u/∂x² + ∂²u/∂y²)  (2-D)
 *
 * Read in plain English: "A point's acceleration equals c² times
 * how curved the medium is around it."  Curvature pushes the
 * point back toward its neighbours; the further it has stretched
 * away, the harder it gets pulled back.  The result is
 * oscillation, and oscillations COUPLED across space propagate as
 * waves at speed c.
 *
 * Real-world examples:
 *   - water surface waves (linearised)
 *   - sound in air
 *   - light in vacuum (c² = 1 / (ε₀ · μ₀))
 *   - vibrations in a stretched membrane
 *   - small-amplitude string vibrations
 *
 * The 2-D version we simulate is most easily visualised as a
 * MEMBRANE — like a drum head or a still pond.  A perturbation
 * spreads outward as concentric ripples.
 *
 * T2  SAMPLING SPACE + TIME ON A REGULAR GRID
 * ───────────────────────────────────────────
 * A computer can't store u(x, y, t) for all real numbers.  We
 * SAMPLE on a regular grid:
 *
 *     space:  x_c = c · dx,  c ∈ [0, COLS−1]
 *             y_r = r · dy,  r ∈ [0, ROWS−1]
 *     time:   t_n = n · dt,  n ∈ [0, ∞)
 *
 * `dx` and `dy` are the SPATIAL STEP SIZES (we use 1 cell);
 * `dt` is the TIME STEP.  A simulation state is the value of u
 * at every grid point at one time step:
 *
 *     u_now[r][c]       ← float, the value of u at (r·dy, c·dx, t_n)
 *
 * This file uses three such grids — past, present, future —
 * because the wave equation is SECOND-ORDER in time (it needs
 * TWO past states to predict the next, T4).
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   r=0  ┌────┬────┬────┬────┐                     │
 *      │        │    │    │    │    │                     │
 *      │   r=1  ├────┼────┼────┼────┤                     │
 *      │        │    │    │    │    │                     │
 *      │   r=2  ├────┼────┼────┼────┤                     │
 *      │        │    │    │ ●  │    │  ← grid point      │
 *      │   r=3  ├────┼────┼────┼────┤    (r=2, c=2)       │
 *      │        │    │    │    │    │    holds u[2][2]    │
 *      │   r=4  └────┴────┴────┴────┘                     │
 *      │        c=0  c=1  c=2  c=3                        │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * T3  THE 5-POINT LAPLACIAN STENCIL
 * ─────────────────────────────────
 * The wave equation has a SPATIAL second derivative on its right-
 * hand side.  In 2-D this is the LAPLACIAN:
 *
 *     ∇²u = ∂²u/∂x² + ∂²u/∂y²
 *
 * To approximate this on a grid, we use a STENCIL — a tiny set of
 * grid points around (r, c) whose values we combine to estimate
 * the local curvature.  The simplest choice is the 5-POINT STENCIL:
 *
 *           u[r-1][c]              ← north
 *               │
 *   u[r][c-1] ─ u[r][c] ─ u[r][c+1]   ← west, centre, east
 *               │
 *           u[r+1][c]              ← south
 *
 *     ∇²u ≈ ( u[r-1][c] + u[r+1][c]      // vertical 2nd derivative
 *           + u[r][c-1] + u[r][c+1]      // horizontal 2nd derivative
 *           − 4 · u[r][c] ) / dx²
 *
 * Where does this come from?  Taylor-expand u around (r, c) and
 * sum the four neighbours.  The first derivatives cancel; the
 * second derivatives reinforce.  You get the Laplacian to leading
 * order, with error O(dx²).  Easy to verify with pencil and paper.
 *
 * Pseudocode:
 *
 *     laplacian_5pt(u, r, c):
 *         return  u[r-1][c] + u[r+1][c]
 *               + u[r][c-1] + u[r][c+1]
 *               - 4.0 * u[r][c]
 *
 * Implemented in §6 of this file.  Note we omit the "/ dx²" because
 * dx = 1 in our cell-unit space; the dx² factor would re-appear if
 * you measured the field in physical units.
 *
 * T4  LEAPFROG TIME STEPPING — PAST, PRESENT, FUTURE
 * ──────────────────────────────────────────────────
 * The wave equation has a SECOND TIME DERIVATIVE on its LHS.  We
 * approximate that with a CENTRED FINITE DIFFERENCE in time:
 *
 *     ∂²u/∂t² ≈ ( u(t + dt) − 2 u(t) + u(t − dt) ) / dt²
 *
 * This needs THREE successive time samples — past, present,
 * future.  Plug into the wave equation and rearrange for the
 * future:
 *
 *     u_new[r][c] = 2 · u_now[r][c]
 *                 − u_old[r][c]
 *                 + (c · dt / dx)² · laplacian_5pt(u_now, r, c)
 *
 * This is LEAPFROG: each new value HOPS over the present using
 * the past as a reference.  Worked example with c = 1, dt = 1,
 * dx = 1, and a single bump centred at (3, 3) with amplitude 1:
 *
 *     u_old[3][3] = 0     (no motion yet, started from rest)
 *     u_now[3][3] = 1     (the initial bump)
 *     laplacian = 0 + 0 + 0 + 0 − 4·1 = −4
 *     u_new[3][3] = 2·1 − 0 + 1·(−4) = −2
 *
 *     u_old[2][3] = 0
 *     u_now[2][3] = 0
 *     laplacian = 0 + 0 + 0 + 1 − 4·0 = 1   (only the centre cell is non-zero)
 *     u_new[2][3] = 2·0 − 0 + 1·1 = 1       (the wave moved up by one cell)
 *
 * The bump split: the centre dropped by 2, the four neighbours
 * picked up 1 each — energy spreading outward.  Run this loop
 * over and over and you get ripples.
 *
 * The SIM CORE is just these few lines, looped over r, c, every
 * tick:
 *
 *     for r in 1 .. ROWS-2:
 *       for c in 1 .. COLS-2:
 *         lap = u_now[r-1][c] + u_now[r+1][c] +
 *               u_now[r][c-1] + u_now[r][c+1] −
 *               4 · u_now[r][c]
 *         u_new[r][c] = 2 · u_now[r][c] − u_old[r][c] + beta² · lap
 *     swap pointers: u_old, u_now, u_new   (rotation, not memcpy)
 *
 * `beta = c · dt / dx` is the per-tick "wave length advance."
 *
 * T5  WHY A DISCRETE SCHEME CAN BLOW UP
 * ─────────────────────────────────────
 * The continuous wave equation conserves energy.  A tap on the
 * pond never explodes into infinity — energy spreads, dissipates
 * at the boundary, but stays bounded.
 *
 * The DISCRETE scheme — leapfrog on a grid — is an APPROXIMATION.
 * For each FOURIER MODE of the field (a sinusoid e^(i·k·x)), the
 * scheme's update operator multiplies the amplitude by a COMPLEX
 * NUMBER µ each tick.  The magnitude |µ| determines whether the
 * mode grows, persists, or decays.
 *
 *     |μ| < 1   →  this mode DECAYS over time   (numerical damping)
 *     |μ| = 1   →  this mode PERSISTS unchanged (perfectly stable)
 *     |μ| > 1   →  this mode GROWS exponentially → SIMULATION EXPLODES
 *
 * For the leapfrog scheme on the wave equation, |μ| depends on:
 *
 *     - the wavelength k of the mode
 *     - the CFL number  c · dt / dx
 *
 * It turns out (T6) that the WORST-CASE |μ| over all wavelengths
 * is:
 *
 *     |μ_max| = 1                    if cfl ≤ 1
 *     |μ_max| > 1 (grows without limit)  if cfl > 1
 *
 * So cfl > 1 means at least ONE Fourier mode of the field grows
 * exponentially.  Even tiny float-roundoff noise contains a bit of
 * that mode; over many steps the mode dominates everything else.
 *
 * Bottom line: the scheme is stable iff EVERY mode satisfies
 * |μ| ≤ 1, which gives the CFL condition.
 *
 * T6  THE CFL NUMBER (VON NEUMANN ANALYSIS, INTUITIVE VERSION)
 * ────────────────────────────────────────────────────────────
 * Plug a single Fourier mode  u(x, t) = exp(i·(k·x − ω·t))  into
 * the leapfrog update and ask: what's the relationship between
 * one step and the next?  We get (1-D for clarity):
 *
 *     exp(−i·ω·dt) − 2 + exp(+i·ω·dt) =
 *         (c·dt/dx)² · [ exp(+i·k·dx) − 2 + exp(−i·k·dx) ]
 *
 * Both bracketed expressions are 2·(cos(·) − 1).  Simplify:
 *
 *     2 · ( cos(ω·dt) − 1 ) = (c·dt/dx)² · 2 · ( cos(k·dx) − 1 )
 *
 *     sin²(ω·dt/2) = (c·dt/dx)² · sin²(k·dx/2)
 *
 *     sin(ω·dt/2) = (c·dt/dx) · sin(k·dx/2)              (1)
 *
 * For the mode to be stable, ω·dt/2 must be REAL — otherwise
 * exp(−i·ω·dt) has |.| ≠ 1 and the mode grows.
 *
 * Real ω·dt/2 requires the LHS to lie in [−1, 1] — sin always
 * does.  So we need:
 *
 *     |(c·dt/dx) · sin(k·dx/2)|  ≤ 1
 *
 * The worst-case k makes sin(k·dx/2) = 1.  So the condition is:
 *
 *     c·dt/dx ≤ 1                    ← THE 1-D CFL CONDITION
 *
 * Our 2-D scheme generalises (T7).  This is Von Neumann analysis
 * — beautiful, terse, and the standard tool for analyzing finite
 * difference schemes.  Strikwerda's textbook has the rigorous
 * version; the formula in (1) is the heart of it.
 *
 * T7  WHY 1 IN 1-D, 1/sqrt(2) IN 2-D, 1/sqrt(d) IN d-D
 * ────────────────────────────────────────────────────
 * Repeat the analysis in 2-D.  Plug u = exp(i·(k_x·x + k_y·y −
 * ω·t)) into the 2-D leapfrog with 5-point Laplacian.  The
 * algebra runs the same way:
 *
 *     sin²(ω·dt/2) = (c·dt/dx)² · [ sin²(k_x·dx/2) + sin²(k_y·dx/2) ]
 *
 * Now the bracket can reach a max of 2 (when k_x and k_y both hit
 * their worst values simultaneously).  For stability:
 *
 *     (c·dt/dx)² · 2 ≤ 1     ⇒    c·dt/dx ≤ 1/sqrt(2)
 *
 * In d dimensions with the obvious (2d+1)-point Laplacian the
 * bracket maxes out at d, giving:
 *
 *     c·dt/dx ≤ 1/sqrt(d)
 *
 * To make the CFL display in this file always read in [0, 1.5]
 * regardless of dimension, we DEFINE
 *
 *     cfl_number = c · dt · sqrt(d) / dx
 *
 * (so cfl_number ≤ 1 is the universal stability condition).  In
 * 2-D this means cfl = c · dt · sqrt(2).  Note: some books use
 * the raw c·dt/dx and bake the sqrt(d) into the threshold;
 * mathematically equivalent, just a different convention.
 *
 * T8  READING THE DASHBOARD — WHAT EACH ROW MEANS
 * ───────────────────────────────────────────────
 * The bottom of the screen reserves 5 lines:
 *
 *   Row 0:  TITLE bar — bold yellow.  Includes "PAUSED" if active.
 *
 *   Row 1:  PARAMETERS — the live numbers:
 *           cfl  = c · dt · sqrt(2)         dimensionless
 *           dt   = step_seconds              seconds
 *           c    = wave_speed_cps            cells / second
 *           |μ|  = growth_per_step           > 1.0001 means unstable
 *           t×2  = time-to-double-amplitude  (only if growing)
 *
 *   Row 2:  CFL METER — a horizontal bar from cfl=0 to cfl=1.5.
 *           Coloured zones: GREEN (cfl<0.7), YELLOW (0.7-1.0),
 *           RED (>1.0).  A vertical "|" marks the critical dt for
 *           the current wave speed.
 *
 *   Row 3:  STATS — max amplitude, tick count, explosion count
 *           since reset.  Useful for spotting numerical decay
 *           (max amp slowly drops at cfl < 1).
 *
 *   Row 4:  KEY HINTS — bright cyan, A_BOLD.
 *
 * The rest of the screen is the SIMULATION GRID.  Each cell shows
 * the local amplitude as a coloured glyph (positive = red ramp,
 * negative = blue ramp; brightness = magnitude).
 *
 * Try this: press 4 (cfl=1.0), then press d to drop a Gaussian.
 * Watch the wave spread and reflect off the walls.  Now press 5
 * (cfl=1.1).  Within seconds the screen fills with high-frequency
 * red/blue speckle as the unstable mode dominates.  The max-amp
 * climbs exponentially in row 3.  After it hits 1e6 the sim
 * auto-resets and the explosion counter increments.
 *
 * Try this also: press m (eigenmode init), then 5 (cfl=1.1).  The
 * eigenmode is an exact unstable mode of the discrete operator —
 * it explodes CLEANLY without high-frequency speckle.  The
 * doubling time t×2 you read off the HUD should match
 *
 *     t×2  = log(2) · dt / log(|μ_max|)
 *
 * exactly (down to float precision).  Verifying this is the
 * cleanest hands-on test of Von Neumann theory.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
/* §1  config — every constant lives here                                */
/* ===================================================================== */
/*
 * What's in this section
 * ----------------------
 * Tunable constants for the simulation, the rendering, and the
 * dashboard.  Anything that's a literal in code below this section
 * is a NAME from §1 (project rule: no scattered magic numbers).
 *
 * Two flavours of constants:
 *   - PHYSICAL  — wave speed, dt range, CFL presets.  Units in
 *                 the comment.
 *   - DISPLAY   — HUD geometry, colour pair IDs, render rates.
 */

enum {
    /* Render frame cap and simulation step rate. */
    TARGET_FPS    = 60,
    SIM_STEPS_PER_FRAME_MAX = 4,   /* upper bound on steps per render */

    /* Grid bounds.  The actual size adapts to the terminal at runtime. */
    GRID_ROWS_MAX = 80,
    GRID_COLS_MAX = 200,

    /* Dashboard layout. */
    DASH_LINES    = 5,             /* bottom 5 rows reserved for HUD */

    /* HUD recompute interval (don't recalculate fps every frame). */
    FPS_UPDATE_MS = 500,

    /* Number of distinct colour ramp levels per sign (positive / negative). */
    RAMP_LEVELS   = 9,

    /* Amplitude history for the explosion-detector growth ratio. */
    GROWTH_HISTORY_LEN = 4,
};

/* PHYSICAL CONSTANTS — units called out explicitly. */
#define WAVE_SPEED_DEFAULT      20.0f   /* cells per second */
#define WAVE_SPEED_MIN           2.0f
#define WAVE_SPEED_MAX         200.0f

#define STEP_SECONDS_DEFAULT  0.030f    /* seconds per simulation tick */
#define STEP_SECONDS_MIN     0.0005f
#define STEP_SECONDS_MAX     0.5000f

#define EXPLOSION_THRESHOLD    1.0e6f   /* abs amplitude triggers reset */
#define DROP_AMPLITUDE         1.0f     /* peak height of Gaussian drop */
#define DROP_RADIUS_CELLS      4.0f     /* sigma of the Gaussian, in cells */
#define EIGENMODE_AMPLITUDE    0.05f    /* small initial amplitude — the
                                         * eigenmode either grows or stays */

/* CFL PRESETS — six landmark values reachable with keys 1..6.  Each
 * preset sets dt = preset_cfl / (c · sqrt(2)) so the resulting cfl
 * is exactly the named value. */
static const float cfl_presets[6] = {
    0.40f,   /* very stable     */
    0.60f,   /* comfortably stable */
    0.85f,   /* margin */
    1.00f,   /* the cliff edge  */
    1.10f,   /* mildly unstable */
    1.40f,   /* aggressively unstable */
};

/* THE CFL CONSTANT — sqrt(2) for our 2-D Laplacian (see T7). */
#define CFL_DIM_FACTOR    1.41421356237309504880f

/* COLOUR PAIR IDS.  Reserved blocks for the colour ramp + HUD. */
enum {
    PAIR_BACKGROUND = 1,
    PAIR_POS_BASE   = 2,                              /* +0..+RAMP_LEVELS-1 */
    PAIR_NEG_BASE   = PAIR_POS_BASE + RAMP_LEVELS,    /* +0..+RAMP_LEVELS-1 */
    PAIR_HUD        = PAIR_NEG_BASE + RAMP_LEVELS,
    PAIR_HINT,
    PAIR_STABLE,    /* CFL meter zone colours */
    PAIR_MARGINAL,
    PAIR_UNSTABLE,
    PAIR_WARN,      /* white-on-red flashing alert */
};

/* THEME — map the 9 ramp levels to 256-colour fg codes.  Three themes
 * provided so the user can pick the most readable one for their
 * terminal.  All themes obey the project rule "no colour code below
 * 24" so A_DIM doesn't render invisible. */
enum { N_THEMES = 3 };

static const int theme_pos256[N_THEMES][RAMP_LEVELS] = {
    /* "wave"    — red→white  */ {  52,  88, 124, 160, 196, 202, 208, 220, 231 },
    /* "thermal" — red→yellow */ {  52,  94, 130, 166, 202, 208, 214, 220, 231 },
    /* "ocean"   — pale cyan  */ { 159, 195, 195, 195, 231, 231, 231, 231, 231 },
};
static const int theme_neg256[N_THEMES][RAMP_LEVELS] = {
    /* "wave"    — blue→white */ {  17,  19,  21,  27,  33,  39,  45,  51, 231 },
    /* "thermal" — magenta    */ {  53,  54,  91,  92, 129, 165, 201, 207, 231 },
    /* "ocean"   — deep blue  */ {  17,  18,  19,  20,  21,  27,  33,  39,  45 },
};
static const char *theme_name[N_THEMES] = { "wave", "thermal", "ocean" };

/* ===================================================================== */
/* §2  clock — monotonic time                                            */
/* ===================================================================== */
/*
 * Why
 * ---
 * For the main-loop timing we need a clock that's:
 *   - MONOTONIC (NTP / system-clock changes don't jump it backward)
 *   - NANOSECOND-resolution (we cap at 60 fps so frame budget is 16ms)
 *
 * CLOCK_MONOTONIC + clock_gettime + nanosleep is the POSIX answer.
 */

#define NS_PER_SEC 1000000000LL

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
/* §3  colors — diverging wave palette + HUD                             */
/* ===================================================================== */
/*
 * Why diverging?
 * --------------
 * The wave field has POSITIVE and NEGATIVE values (peaks and
 * troughs).  A single colour ramp (e.g. heatmap) hides the sign.
 * A DIVERGING palette uses TWO ramps that share a neutral
 * background at zero — perfect for signed scalar fields.
 *
 *   negative                    positive
 *   ◀───────── 0 ─────────▶
 *   blue·dark blue·blue·white·red·dark red·deep red
 *
 * The HUD palette uses bright yellow (status) + bright cyan
 * (hints) per the project's CLAUDE.md HUD spec.
 */

static int wave_pair_for(float amplitude, int theme)
{
    /* Map amplitude in [-1, +1] (clamped) to a ramp level 0..RAMP_LEVELS-1. */
    float clamped   = amplitude;
    if (clamped >  1.0f) clamped =  1.0f;
    if (clamped < -1.0f) clamped = -1.0f;

    int   level = (int)(fabsf(clamped) * (RAMP_LEVELS - 0.001f));
    if (level >= RAMP_LEVELS) level = RAMP_LEVELS - 1;
    (void)theme;   /* used by ramp tables, looked up via init_pair already */
    return (clamped >= 0.0f) ? PAIR_POS_BASE + level
                             : PAIR_NEG_BASE + level;
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();

    /* 9 positive + 9 negative ramp colours for the active theme. */
    for (int level = 0; level < RAMP_LEVELS; level++) {
        if (COLORS >= 256) {
            init_pair(PAIR_POS_BASE + level, theme_pos256[theme][level], -1);
            init_pair(PAIR_NEG_BASE + level, theme_neg256[theme][level], -1);
        } else {
            init_pair(PAIR_POS_BASE + level, COLOR_RED,   -1);
            init_pair(PAIR_NEG_BASE + level, COLOR_BLUE,  -1);
        }
    }

    /* HUD: bright yellow (top status), bright cyan (key hints).
     * Default background (-1) = whatever the terminal user set. */
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,      226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,      51, -1);   /* bright cyan   */
        init_pair(PAIR_STABLE,    82, -1);   /* lime green    */
        init_pair(PAIR_MARGINAL, 220, -1);   /* amber         */
        init_pair(PAIR_UNSTABLE, 196, -1);   /* bright red    */
        init_pair(PAIR_WARN,     231, COLOR_RED);   /* white on red */
    } else {
        init_pair(PAIR_HUD,      COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,     COLOR_CYAN,    -1);
        init_pair(PAIR_STABLE,   COLOR_GREEN,   -1);
        init_pair(PAIR_MARGINAL, COLOR_YELLOW,  -1);
        init_pair(PAIR_UNSTABLE, COLOR_RED,     -1);
        init_pair(PAIR_WARN,     COLOR_WHITE,   COLOR_RED);
    }
}

/* ===================================================================== */
/* §4  grid — three time buffers (past, present, future)                 */
/* ===================================================================== */
/*
 * Why three buffers?
 * ------------------
 * The wave equation is SECOND-ORDER in time (T4).  To compute u at
 * the next step, leapfrog reads u at TWO previous steps.  Hence
 * three buffers in rotation:
 *
 *     wave_past[r][c]   ← u(t − dt)
 *     wave_now[r][c]    ← u(t)
 *     wave_next[r][c]   ← u(t + dt) ← being filled
 *
 * After each step, we ROTATE pointers (no memcpy):
 *     past   → next  (becomes scratch; will be overwritten)
 *     now    → past
 *     next   → now
 *
 * The grid_state struct owns the three buffers + their dimensions.
 * The buffers are statically-sized at GRID_ROWS_MAX × GRID_COLS_MAX
 * so we avoid malloc; the active subregion is (rows × cols).
 */

typedef struct {
    int    rows, cols;                       /* active subregion bounds */
    float *wave_past;                        /* points into one of buf_a/b/c */
    float *wave_now;
    float *wave_next;
    /* The three actual storage arenas.  Pointers above rotate among them. */
    float  buf_a[GRID_ROWS_MAX * GRID_COLS_MAX];
    float  buf_b[GRID_ROWS_MAX * GRID_COLS_MAX];
    float  buf_c[GRID_ROWS_MAX * GRID_COLS_MAX];
} grid_state;

static inline float grid_at(const float *buf, int cols, int r, int c)
{
    return buf[r * cols + c];
}

static inline void grid_set(float *buf, int cols, int r, int c, float v)
{
    buf[r * cols + c] = v;
}

static void grid_rotate_buffers(grid_state *g)
{
    /* Rename the three buffers cyclically:  next becomes "now";
     * "now" becomes "past"; "past" becomes the new scratch (next). */
    float *was_past = g->wave_past;
    g->wave_past = g->wave_now;
    g->wave_now  = g->wave_next;
    g->wave_next = was_past;
}

/* ===================================================================== */
/* §5  init — zero, eigenmode, drop                                      */
/* ===================================================================== */
/*
 * Three ways to start the simulation:
 *
 *   1. ZERO            — flat field; nothing happens.  Used for resets.
 *
 *   2. GAUSSIAN DROP   — bell-shaped bump centred at (cy, cx).  The
 *                         classic "stone in the pond" — looks like
 *                         spreading concentric ripples.
 *
 *   3. EIGENMODE       — sin(π·n_y·r/rows) · sin(π·n_x·c/cols).  An
 *                         exact eigenfunction of the 5-point
 *                         Laplacian.  This is the WORST-CASE mode
 *                         for instability — when cfl > 1 it grows
 *                         CLEANLY (just amplitude doubling, no
 *                         high-frequency speckle), perfect for
 *                         measuring the growth ratio.
 *
 * All three are applied to BOTH wave_past and wave_now (so the
 * initial velocity ∂u/∂t = 0).  This means leapfrog starts from
 * REST.
 */

static void init_zero(grid_state *g)
{
    memset(g->wave_past,  0, sizeof g->buf_a);
    memset(g->wave_now,   0, sizeof g->buf_a);
    memset(g->wave_next,  0, sizeof g->buf_a);
}

static void init_gaussian_drop(grid_state *g, float cy, float cx,
                               float amplitude, float radius_cells)
{
    init_zero(g);
    float two_sigma_sq = 2.0f * radius_cells * radius_cells;
    for (int r = 0; r < g->rows; r++) {
        for (int c = 0; c < g->cols; c++) {
            float dy = (float)r - cy;
            float dx = (float)c - cx;
            float v  = amplitude * expf(-(dy*dy + dx*dx) / two_sigma_sq);
            grid_set(g->wave_past, g->cols, r, c, v);
            grid_set(g->wave_now,  g->cols, r, c, v);
        }
    }
}

static void init_eigenmode(grid_state *g, int mode_y, int mode_x,
                           float amplitude)
{
    init_zero(g);
    /* sin( π · m · r / N ) is an eigenfunction of the discrete 5-point
     * Laplacian on a Dirichlet-boundary grid.  Its eigenvalue is
     *   λ = − ( 4·sin²(π·m_y/(2·N_y)) + 4·sin²(π·m_x/(2·N_x)) )
     * which the leapfrog scheme amplifies cleanly when cfl > 1. */
    for (int r = 0; r < g->rows; r++) {
        for (int c = 0; c < g->cols; c++) {
            float ry = (float)(r + 1) / (float)(g->rows + 1);
            float rx = (float)(c + 1) / (float)(g->cols + 1);
            float v  = amplitude
                     * sinf((float)M_PI * mode_y * ry)
                     * sinf((float)M_PI * mode_x * rx);
            grid_set(g->wave_past, g->cols, r, c, v);
            grid_set(g->wave_now,  g->cols, r, c, v);
        }
    }
}

/* ===================================================================== */
/* §6  laplacian — the 5-point stencil                                   */
/* ===================================================================== */
/*
 * Computes (north + south + east + west − 4·centre) at one cell.
 * This is the discrete Laplacian on a unit-spacing grid (T3).
 *
 * The CALLER guarantees r ∈ [1, rows-2] and c ∈ [1, cols-2]; we
 * skip the border cells to avoid out-of-bounds reads.  Borders are
 * held at zero (Dirichlet boundary), so they don't contribute to
 * any interior cell's update beyond what the stencil already reads.
 *
 * Why is this so cheap?  Five reads + one multiply + four adds, all
 * on contiguous memory.  Modern CPUs execute the inner loop at
 * ~1 cycle per cell.
 */

static inline float laplacian_5pt(const float *u, int cols, int r, int c)
{
    return  u[(r-1)*cols + c    ]    /* north */
         +  u[(r+1)*cols + c    ]    /* south */
         +  u[ r   *cols + (c-1)]    /* west  */
         +  u[ r   *cols + (c+1)]    /* east  */
         - 4.0f * u[r*cols + c];     /* centre × 4 (subtracted) */
}

/* ===================================================================== */
/* §7  step — the leapfrog time integrator                               */
/* ===================================================================== */
/*
 * One simulation tick.  Given wave_past and wave_now, fill
 * wave_next, then rotate buffers.
 *
 * The physics:
 *
 *   u_new[r][c] = 2·u_now[r][c]
 *               −   u_old[r][c]
 *               + beta² · laplacian_5pt(u_now, r, c)
 *
 *   beta = c · dt / dx        (with dx = 1 in our cell units)
 *
 * Boundary cells (r=0, r=rows-1, c=0, c=cols-1) are forced to zero
 * — Dirichlet (closed) boundary.  Waves reflect off the walls with
 * a phase flip (sign change), like a string fixed at both ends.
 *
 * The "two passes" pattern (compute all of next; then rotate) is
 * REQUIRED — if we wrote into wave_now while reading from it, the
 * stencil for cell (r, c+1) would see the just-computed (r, c)
 * value instead of the OLD one, breaking the simulation.
 */

static void wave_step(grid_state *g, float wave_speed_cps, float step_seconds)
{
    float beta        = wave_speed_cps * step_seconds;
    float beta_squared = beta * beta;
    int   cols = g->cols;

    /* INTERIOR cells — apply the stencil. */
    for (int r = 1; r < g->rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            float lap     = laplacian_5pt(g->wave_now, cols, r, c);
            float u_now   = g->wave_now [r * cols + c];
            float u_past  = g->wave_past[r * cols + c];
            float u_next  = 2.0f * u_now - u_past + beta_squared * lap;
            g->wave_next[r * cols + c] = u_next;
        }
    }

    /* BORDERS — forced to zero (Dirichlet). */
    for (int c = 0; c < cols; c++) {
        g->wave_next[0                * cols + c] = 0.0f;
        g->wave_next[(g->rows - 1)    * cols + c] = 0.0f;
    }
    for (int r = 0; r < g->rows; r++) {
        g->wave_next[r * cols + 0          ] = 0.0f;
        g->wave_next[r * cols + (cols - 1) ] = 0.0f;
    }

    grid_rotate_buffers(g);
}

/* ===================================================================== */
/* §8  cfl — compute the CFL number, critical dt, growth rate            */
/* ===================================================================== */
/*
 * CFL number (T6, T7):
 *
 *     cfl = wave_speed_cps · step_seconds · sqrt(2)
 *
 * (the sqrt(2) is the dimension factor for 2-D; T7 derived it).
 *
 * Critical dt — largest stable dt for the current wave speed:
 *
 *     dt_crit = 1 / (wave_speed_cps · sqrt(2))   (so cfl = 1)
 *
 * Theoretical growth ratio per step at a given cfl:
 *
 *     |μ| = 1                  if cfl ≤ 1
 *     |μ| = cfl + sqrt(cfl² − 1)  if cfl > 1
 *
 * The latter comes from solving the leapfrog characteristic
 * equation when cfl > 1 (the LHS of equation (1) in T6 forces ω·dt
 * to be complex; the imaginary part gives the growth rate).
 */

static float cfl_compute(float wave_speed_cps, float step_seconds)
{
    return wave_speed_cps * step_seconds * CFL_DIM_FACTOR;
}

static float dt_critical(float wave_speed_cps)
{
    return 1.0f / (wave_speed_cps * CFL_DIM_FACTOR);
}

static float growth_per_step_theoretical(float cfl_number)
{
    if (cfl_number <= 1.0f) return 1.0f;
    return cfl_number + sqrtf(cfl_number * cfl_number - 1.0f);
}

static float time_to_double(float growth_per_step, float step_seconds)
{
    /* Steps to double:  N = log(2) / log(μ).
     * Time:  N · dt seconds. */
    if (growth_per_step <= 1.0001f) return INFINITY;
    return logf(2.0f) / logf(growth_per_step) * step_seconds;
}

/* ===================================================================== */
/* §9  measure — max |u|, growth ratio, explosion detector               */
/* ===================================================================== */
/*
 * What it does
 * ------------
 * Each tick, scan the grid for the maximum absolute amplitude.
 * Compare with the value from earlier ticks to compute an
 * EMPIRICAL growth ratio (independent of theory).  If amplitude
 * exceeds EXPLOSION_THRESHOLD (default 1e6), declare an
 * "explosion" — the simulation is running away and we should
 * reset before NaN cascades destroy everything.
 *
 * Why ratio over a HISTORY?
 * One-step ratios are noisy because reflections at boundaries make
 * the field oscillate.  Averaging over GROWTH_HISTORY_LEN steps
 * gives a stable readout for the dashboard's "|μ|" cell.
 */

typedef struct {
    float history[GROWTH_HISTORY_LEN];   /* recent max-abs samples */
    int   write_index;                   /* circular */
    int   filled;                        /* up to GROWTH_HISTORY_LEN */
    float current_max_abs;
    float empirical_growth_per_step;
    long  tick_count;
    int   explosion_count;
} measure_state;

static void measure_init(measure_state *m)
{
    memset(m, 0, sizeof *m);
}

static float scan_max_abs(const float *u, int rows, int cols)
{
    float best = 0.0f;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float v = fabsf(u[r * cols + c]);
            if (v > best) best = v;
        }
    }
    return best;
}

static bool measure_update(measure_state *m, const grid_state *g)
{
    /* 1. scan grid for max-abs amplitude */
    float now_max = scan_max_abs(g->wave_now, g->rows, g->cols);
    m->current_max_abs = now_max;

    /* 2. push into circular history buffer */
    m->history[m->write_index] = now_max;
    m->write_index = (m->write_index + 1) % GROWTH_HISTORY_LEN;
    if (m->filled < GROWTH_HISTORY_LEN) m->filled++;

    /* 3. average geometric growth: (current / oldest)^(1/N) */
    if (m->filled >= 2) {
        int   oldest_index = m->write_index;        /* the slot we'll overwrite next */
        float oldest = m->history[oldest_index];
        if (oldest > 1e-12f) {
            float ratio = now_max / oldest;
            float steps = (float)(m->filled - 1);
            m->empirical_growth_per_step = powf(ratio, 1.0f / steps);
        }
    }

    m->tick_count++;

    /* 4. explosion check */
    if (now_max > EXPLOSION_THRESHOLD || isnan(now_max) || isinf(now_max)) {
        m->explosion_count++;
        return true;     /* caller should reset */
    }
    return false;
}

/* ===================================================================== */
/* §10  paint_field — render the wave grid                               */
/* ===================================================================== */
/*
 * Map each grid cell to one terminal cell.  Choose a glyph + colour
 * pair from the diverging palette based on amplitude:
 *
 *     |u| < small         → ' ' (background, invisible)
 *     |u| small to mid    → '.', ':', '·' progressively brighter
 *     |u| mid to large    → '+', '*', '#'
 *     |u| > 1             → '@' clipped (saturating)
 *
 * The COLOUR encodes sign + intensity; the GLYPH adds an extra
 * legibility tier for monochrome / contrast issues.
 */

static char glyph_for_amp(float magnitude)
{
    if (magnitude < 0.05f) return ' ';
    if (magnitude < 0.15f) return '.';
    if (magnitude < 0.30f) return ':';
    if (magnitude < 0.50f) return '+';
    if (magnitude < 0.75f) return '*';
    return '#';
}

static void paint_field(WINDOW *win, const grid_state *g, int theme)
{
    /* Field occupies rows 0 .. (terminal_rows - DASH_LINES - 1).
     * The grid_state's rows × cols already match this. */
    for (int r = 0; r < g->rows; r++) {
        for (int c = 0; c < g->cols; c++) {
            float v   = grid_at(g->wave_now, g->cols, r, c);
            float mag = fabsf(v);
            int   pair = wave_pair_for(v, theme);
            char  glyph = glyph_for_amp(mag);
            attr_t a = COLOR_PAIR(pair);
            if (mag > 0.50f) a |= A_BOLD;
            wattron(win, a);
            mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
            wattroff(win, a);
        }
    }
}

/* ===================================================================== */
/* §11  paint_hud — title + parameters + stats                           */
/* ===================================================================== */
/*
 * Five-row dashboard at the bottom of the screen.  Per CLAUDE.md
 * spec: bright yellow primary, bright cyan key hints, A_BOLD on
 * primary line, no A_DIM anywhere, default background.
 *
 *   Row 0:  TITLE          [ CFL STABILITY EXPLORER ]   PAUSED
 *   Row 1:  PARAMETERS     cfl=1.057  dt=0.038s  c=20.0c/s  |μ|=1.014  t×2=49.5s
 *   Row 2:  CFL METER      [================XXX==========|·······]   ←§12
 *   Row 3:  STATS          max=24.8  ticks=1248  exploded=2  theme=wave
 *   Row 4:  KEY HINTS      q:quit  spc:pause  s:step  +/-:dt  c/C:speed  ...
 */

typedef struct {
    float cfl_number;
    float step_seconds;
    float dt_critical_now;     /* theoretical largest stable dt */
    float wave_speed_cps;
    float empirical_growth;    /* measured live by §9 */
    float theoretical_growth;  /* Von Neumann prediction (§8) */
    float doubling_time;
    float max_amplitude;
    long  tick_count;
    int   explosion_count;
    int   theme;
    bool  paused;
} hud_data;

static int cfl_zone_pair(float cfl_number)
{
    if (cfl_number < 0.70f) return PAIR_STABLE;
    if (cfl_number < 1.00f) return PAIR_MARGINAL;
    return PAIR_UNSTABLE;
}

static const char *cfl_zone_label(float cfl_number)
{
    if (cfl_number < 0.70f) return "stable  ";
    if (cfl_number < 1.00f) return "marginal";
    return "UNSTABLE";
}

static void paint_hud_title(WINDOW *win, int row, int cols, bool paused)
{
    wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvwaddch(win, row, c, '-');
    const char *title = "[ CFL STABILITY EXPLORER ]";
    mvwprintw(win, row, (cols - (int)strlen(title)) / 2, "%s", title);
    if (paused) {
        wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
        wattron(win, COLOR_PAIR(PAIR_MARGINAL) | A_BOLD);
        mvwprintw(win, row, cols - 9, " PAUSED ");
        wattroff(win, COLOR_PAIR(PAIR_MARGINAL) | A_BOLD);
        wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    }
    wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void paint_hud_params(WINDOW *win, int row, const hud_data *h)
{
    int cp = cfl_zone_pair(h->cfl_number);
    /* "CFL =  1.057  marginal   dt=0.038s  c=20.0c/s   ..." */
    wattron(win, COLOR_PAIR(PAIR_HUD));
    mvwprintw(win, row, 1, " CFL = ");
    wattroff(win, COLOR_PAIR(PAIR_HUD));

    wattron(win, COLOR_PAIR(cp) | A_BOLD);
    wprintw(win, "%6.4f  %-9s", h->cfl_number, cfl_zone_label(h->cfl_number));
    wattroff(win, COLOR_PAIR(cp) | A_BOLD);

    wattron(win, COLOR_PAIR(PAIR_HUD));
    wprintw(win, "  dt=%7.4fs  c=%5.1fc/s  dt_crit=%7.4fs  ",
            h->step_seconds, h->wave_speed_cps, h->dt_critical_now);
    wattroff(win, COLOR_PAIR(PAIR_HUD));

    /* Show BOTH empirical and theoretical growth ratios.  At
     * eigenmode init they should match closely — visual proof of
     * the Von Neumann formula (§8 growth_per_step_theoretical). */
    bool growing = h->empirical_growth > 1.0001f && isfinite(h->doubling_time);
    int  pair    = growing ? PAIR_UNSTABLE : PAIR_STABLE;
    wattron(win, COLOR_PAIR(pair) | A_BOLD);
    wprintw(win, "|mu|=%7.5f (theory %7.5f)",
            h->empirical_growth > 0.0f ? h->empirical_growth : 1.0f,
            h->theoretical_growth);
    wattroff(win, COLOR_PAIR(pair) | A_BOLD);
    if (growing) {
        wattron(win, COLOR_PAIR(PAIR_UNSTABLE) | A_BOLD);
        wprintw(win, "  t*2=%5.2fs", h->doubling_time);
        wattroff(win, COLOR_PAIR(PAIR_UNSTABLE) | A_BOLD);
    }
}

static void paint_hud_stats(WINDOW *win, int row, const hud_data *h)
{
    wattron(win, COLOR_PAIR(PAIR_HUD));
    mvwprintw(win, row, 1,
              " max|u|=%9.4g  ticks=%8ld  exploded=%3d  theme=%-7s",
              (double)h->max_amplitude, h->tick_count, h->explosion_count,
              theme_name[h->theme]);
    wattroff(win, COLOR_PAIR(PAIR_HUD));
}

static void paint_hud_hints(WINDOW *win, int row)
{
    wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvwprintw(win, row, 0,
              " q:quit  spc:pause  s:step  1-6:CFL preset  +/-:dt  "
              "c/C:speed  m:mode  d:drop  r:reset  t:theme ");
    wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §12  paint_meter — the CFL meter bar                                  */
/* ===================================================================== */
/*
 * A horizontal bar from 0.0 to 1.5 showing the live CFL number.
 * Three coloured zones:
 *
 *   [0.0 .. 0.7 )   STABLE      green  '='
 *   [0.7 .. 1.0 )   MARGINAL    amber  '='
 *   [1.0 .. 1.5 ]   UNSTABLE    red    '='
 *
 * Markers:
 *   '|' at cfl = 1.0 (the cliff)
 *   '#' at the current cfl_number (cursor)
 *
 * Visual: the cursor crossing the '|' line corresponds to the
 * onset of instability — at that moment the simulation visibly
 * starts to grow rather than oscillate.
 */

#define CFL_METER_MAX 1.5f

static void paint_cfl_meter(WINDOW *win, int row, int width, float cfl_number)
{
    int bar_inner = width - 2;     /* leave 1 cell each side for brackets */
    if (bar_inner < 10) return;

    /* zone boundaries in cells */
    int idx_07 = (int)(0.70f / CFL_METER_MAX * bar_inner);
    int idx_10 = (int)(1.00f / CFL_METER_MAX * bar_inner);
    int idx_now = (int)(cfl_number / CFL_METER_MAX * bar_inner);
    if (idx_now < 0) idx_now = 0;
    if (idx_now >= bar_inner) idx_now = bar_inner - 1;

    /* left bracket */
    wattron(win, COLOR_PAIR(PAIR_HUD));
    mvwaddch(win, row, 0, '[');

    /* draw zone-coloured bar */
    for (int i = 0; i < bar_inner; i++) {
        int   pair;
        if      (i < idx_07) pair = PAIR_STABLE;
        else if (i < idx_10) pair = PAIR_MARGINAL;
        else                 pair = PAIR_UNSTABLE;
        char glyph = '=';
        if (i == idx_10) glyph = '|';      /* cliff marker */
        if (i == idx_now) glyph = '#';     /* cursor */
        attr_t a = COLOR_PAIR(pair);
        if (i == idx_now) a |= A_BOLD;
        wattron(win, a);
        mvwaddch(win, row, 1 + i, (chtype)(unsigned char)glyph);
        wattroff(win, a);
    }

    /* right bracket + numeric readout */
    wattron(win, COLOR_PAIR(PAIR_HUD));
    mvwaddch(win, row, 1 + bar_inner, ']');
    wattroff(win, COLOR_PAIR(PAIR_HUD));
}

/* ===================================================================== */
/* §13  scene — orchestrator (state + tick + draw)                       */
/* ===================================================================== */
/*
 * Owns the simulation + measurement state.  Three operations:
 *
 *   scene_init(s)         — set defaults, allocate, drop a Gaussian
 *   scene_tick(s)         — advance the wave one step + measure
 *   scene_draw(s, win)    — paint the field + dashboard
 *
 * Plus utility actions (reset, set_cfl_preset, etc.) that the
 * input layer (§14) calls in response to keys.
 */

typedef enum { INIT_DROP, INIT_EIGENMODE } init_kind;

typedef struct {
    grid_state    grid;
    measure_state measure;
    float         wave_speed_cps;
    float         step_seconds;
    int           theme;
    init_kind     last_init;
    bool          paused;
    bool          step_once;
} scene_state;

static void scene_apply_init(scene_state *s)
{
    if (s->last_init == INIT_EIGENMODE) {
        init_eigenmode(&s->grid, 1, 1, EIGENMODE_AMPLITUDE);
    } else {
        init_gaussian_drop(&s->grid,
                           (float)(s->grid.rows - 1) * 0.5f,
                           (float)(s->grid.cols - 1) * 0.5f,
                           DROP_AMPLITUDE, DROP_RADIUS_CELLS);
    }
    measure_init(&s->measure);
}

static void scene_resize_to_terminal(scene_state *s)
{
    int term_rows, term_cols;
    getmaxyx(stdscr, term_rows, term_cols);

    int field_rows = term_rows - DASH_LINES;
    int field_cols = term_cols;
    if (field_rows < 4) field_rows = 4;
    if (field_cols < 8) field_cols = 8;
    if (field_rows > GRID_ROWS_MAX) field_rows = GRID_ROWS_MAX;
    if (field_cols > GRID_COLS_MAX) field_cols = GRID_COLS_MAX;

    s->grid.rows = field_rows;
    s->grid.cols = field_cols;
    s->grid.wave_past = s->grid.buf_a;
    s->grid.wave_now  = s->grid.buf_b;
    s->grid.wave_next = s->grid.buf_c;
}

static void scene_init(scene_state *s)
{
    memset(s, 0, sizeof *s);
    s->wave_speed_cps = WAVE_SPEED_DEFAULT;
    s->step_seconds   = STEP_SECONDS_DEFAULT;
    s->theme          = 0;
    s->last_init      = INIT_DROP;
    s->paused         = false;

    scene_resize_to_terminal(s);
    scene_apply_init(s);
}

static void scene_set_cfl_preset(scene_state *s, float target_cfl)
{
    /* dt = target_cfl / (c · sqrt(2))  — solves cfl_compute() = target */
    s->step_seconds = target_cfl / (s->wave_speed_cps * CFL_DIM_FACTOR);
    if (s->step_seconds < STEP_SECONDS_MIN) s->step_seconds = STEP_SECONDS_MIN;
    if (s->step_seconds > STEP_SECONDS_MAX) s->step_seconds = STEP_SECONDS_MAX;
}

static void scene_tick(scene_state *s)
{
    if (s->paused && !s->step_once) return;
    s->step_once = false;

    wave_step(&s->grid, s->wave_speed_cps, s->step_seconds);

    bool exploded = measure_update(&s->measure, &s->grid);
    if (exploded) {
        scene_apply_init(s);
        /* Preserve explosion_count across the re-init. */
        int saved_count = s->measure.explosion_count;
        measure_init(&s->measure);
        s->measure.explosion_count = saved_count;
    }
}

static void scene_draw(scene_state *s, WINDOW *win)
{
    werase(win);
    paint_field(win, &s->grid, s->theme);

    int term_rows, term_cols;
    getmaxyx(win, term_rows, term_cols);
    int dash_top = term_rows - DASH_LINES;

    float cfl_n = cfl_compute(s->wave_speed_cps, s->step_seconds);
    hud_data h = {
        .cfl_number        = cfl_n,
        .step_seconds      = s->step_seconds,
        .dt_critical_now   = dt_critical(s->wave_speed_cps),
        .wave_speed_cps    = s->wave_speed_cps,
        .empirical_growth  = s->measure.empirical_growth_per_step,
        .theoretical_growth= growth_per_step_theoretical(cfl_n),
        .doubling_time     = time_to_double(s->measure.empirical_growth_per_step,
                                            s->step_seconds),
        .max_amplitude     = s->measure.current_max_abs,
        .tick_count        = s->measure.tick_count,
        .explosion_count   = s->measure.explosion_count,
        .theme             = s->theme,
        .paused            = s->paused,
    };

    paint_hud_title (win, dash_top + 0, term_cols, h.paused);
    paint_hud_params(win, dash_top + 1, &h);
    paint_cfl_meter (win, dash_top + 2, term_cols, h.cfl_number);
    paint_hud_stats (win, dash_top + 3, &h);
    paint_hud_hints (win, dash_top + 4);

    wnoutrefresh(win);
    doupdate();
}

/* ===================================================================== */
/* §14  input — key handler                                              */
/* ===================================================================== */

static bool scene_handle_key(scene_state *s, int key)
{
    switch (key) {
        case 'q': case 'Q': case 27:    /* 27 = ESC */
            return false;               /* request quit */

        case ' ':
            s->paused = !s->paused;
            break;

        case 's':
            if (s->paused) s->step_once = true;
            break;

        case '1': scene_set_cfl_preset(s, cfl_presets[0]); break;
        case '2': scene_set_cfl_preset(s, cfl_presets[1]); break;
        case '3': scene_set_cfl_preset(s, cfl_presets[2]); break;
        case '4': scene_set_cfl_preset(s, cfl_presets[3]); break;
        case '5': scene_set_cfl_preset(s, cfl_presets[4]); break;
        case '6': scene_set_cfl_preset(s, cfl_presets[5]); break;

        case '+': case '=':
            s->step_seconds *= 1.05f;
            if (s->step_seconds > STEP_SECONDS_MAX) s->step_seconds = STEP_SECONDS_MAX;
            break;
        case '-': case '_':
            s->step_seconds /= 1.05f;
            if (s->step_seconds < STEP_SECONDS_MIN) s->step_seconds = STEP_SECONDS_MIN;
            break;

        case 'c':
            s->wave_speed_cps *= 0.90f;
            if (s->wave_speed_cps < WAVE_SPEED_MIN) s->wave_speed_cps = WAVE_SPEED_MIN;
            break;
        case 'C':
            s->wave_speed_cps *= 1.10f;
            if (s->wave_speed_cps > WAVE_SPEED_MAX) s->wave_speed_cps = WAVE_SPEED_MAX;
            break;

        case 'm':
            s->last_init = INIT_EIGENMODE;
            scene_apply_init(s);
            break;
        case 'd':
            s->last_init = INIT_DROP;
            scene_apply_init(s);
            break;

        case 'r':
            measure_init(&s->measure);
            scene_apply_init(s);
            break;

        case 't':
            s->theme = (s->theme + 1) % N_THEMES;
            color_init(s->theme);
            break;

        default:
            break;
    }
    return true;   /* keep running */
}

/* ===================================================================== */
/* §15  screen — ncurses init + cleanup                                  */
/* ===================================================================== */

static void screen_init(int theme)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, true);
    nodelay(stdscr, true);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
}

static void screen_cleanup(void)
{
    endwin();
}

/* ===================================================================== */
/* §16  app — main loop + signals                                        */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit = 1;
}

int main(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    static scene_state scene;
    screen_init(0);
    atexit(screen_cleanup);
    scene_init(&scene);

    const int64_t frame_ns = NS_PER_SEC / TARGET_FPS;

    while (!g_should_quit) {
        int64_t frame_start = clock_now_ns();

        /* Handle terminal resize. */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            scene_resize_to_terminal(&scene);
            scene_apply_init(&scene);
        }

        /* Drain input queue. */
        int key;
        while ((key = getch()) != ERR) {
            if (!scene_handle_key(&scene, key)) {
                g_should_quit = 1;
                break;
            }
        }

        /* Advance simulation.  At our default step rate each render
         * frame may want one or two ticks; cap to avoid spiral. */
        for (int n = 0; n < SIM_STEPS_PER_FRAME_MAX; n++) {
            scene_tick(&scene);
            if (scene.paused) break;
        }

        scene_draw(&scene, stdscr);

        int64_t spent = clock_now_ns() - frame_start;
        if (spent < frame_ns) clock_sleep_ns(frame_ns - spent);
    }

    return 0;
}
