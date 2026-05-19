/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_wave.c — FitzHugh-Nagumo excitable medium on a terminal grid
 *
 * DEMO: An EXCITABLE MEDIUM — the kind of system that powers nerve
 *       impulses, heart muscle, slime mold spirals, and the
 *       Belousov-Zhabotinsky chemical reaction.  At rest the field
 *       sits in a stable QUIESCENT state.  A LOCALISED STIMULUS
 *       above threshold fires a wave that propagates outward; once
 *       the wave passes, the medium enters a REFRACTORY PERIOD
 *       (briefly unable to fire again) before slowly returning to
 *       rest.  This combination — threshold, propagation, refractory
 *       — gives rise to:
 *
 *           TARGET RINGS    concentric expanding rings
 *           DOUBLE SOURCE   two sources interfere/annihilate
 *           SPIRAL WAVE     broken ring becomes a rotating spiral
 *           PLANE WAVE      flat wave sweeping across the grid
 *
 *       Two variables per cell:
 *
 *           u  ACTIVATOR    fast, fires then spikes (membrane voltage)
 *           v  INHIBITOR    slow, restores resting state (recovery)
 *
 *       Per tick, every cell evolves under:
 *
 *           du/dt = u − u³/3 − v + D · ∇²u + I(t)
 *           dv/dt = ε · (u + a − b · v)
 *
 *       Six terms.  The CUBIC u − u³/3 gives bistability + threshold.
 *       The −v term lets the inhibitor pull the activator back down.
 *       D · ∇²u is diffusion (only u diffuses; v is local).  ε ≪ 1
 *       makes v slow, so the cell stays "fired" for a while before
 *       recovering.  I(t) is an external impulse (the SPACE key).
 *
 *       Forward Euler with sub-stepping.  CFL-safe at the chosen
 *       dt.  5-point Laplacian.  Neumann (zero-flux) boundaries.
 *
 * Study alongside:
 *   fluid/reaction_diffusion.c   — Gray-Scott reaction-diffusion;
 *                                    different equations, similar
 *                                    numerical pattern.  Compare T2
 *                                    (FN: bistable cubic + slow
 *                                    inhibitor) with that file's T2
 *                                    (RD: activator-inhibitor with
 *                                    Du > Dv).
 *   physics/cellular_automaton_excitable.c
 *                                  — discrete-state cousin (where
 *                                    available).  Same dynamics,
 *                                    no PDE.
 *   procedural/diffusion/heat_diffusion.c
 *                                  — pure diffusion, no reaction.
 *                                    Removes the bistable trick.
 *
 * Section map:
 *   §1   config            — every tunable named, no later magic
 *   §2   clock             — monotonic ns timer + sleep
 *   §3   colors            — 5-band activator ramp + HUD pairs
 *   §4   grid_state        — the four 2-D fields + global instance
 *   §5   neighbours        — boundary-clamped index helpers
 *   §6   laplacian         — 5-point stencil for the activator
 *   §7   reaction_step     — one Euler tick of the FN PDE
 *   §8   reset             — fill the field with the resting state
 *   §9   impulse           — circular suprathreshold blob
 *   §10  presets           — 4 scene loaders + table
 *   §11  glyph_picker      — activator value → glyph + colour
 *   §12  render            — paint the activator field
 *   §13  scene             — paused / preset / speed state + tick
 *   §14  hud               — top status + bottom hint (CLAUDE.md spec)
 *   §15  screen            — ncurses init / cleanup / present
 *   §16  app               — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   p / P            pause / resume
 *   space            inject impulse at screen centre
 *   r / R            reset to resting state
 *   1 / 2 / 3 / 4    preset (rings / double / spiral / plane)
 *   + / -            sim sub-steps per frame +/- (speed)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_wave.c \
 *       -o reaction_wave -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      The tutorials are a ladder: T1 sets the worldview (excitable
 *      media in nature), T2-T6 build the FN equations term by term,
 *      T7-T8 cover spirals and stability, T9 the visualisation.
 *   2. §1 config — every constant in one place with units and CFL
 *      stability comments.
 *   3. §4 grid_state — the four 2-D fields.  Knowing the data layout
 *      illuminates everything below.
 *   4. §7 reaction_step — the algorithm in twenty lines.  Read AFTER
 *      tutorials T2, T4, T8.
 *   5. §8 reset → §9 impulse → §10 presets — initial conditions.
 *   6. §11 glyph_picker → §12 render — the visualisation layer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_scene.activator_voltage[r][c]      u — fast variable (membrane voltage
 *                                     analogue).  Drives the wave.
 *   g_scene.inhibitor_recovery[r][c]     v — slow variable (recovery).
 *                                     Pulls u back to rest.
 *   g_scene.activator_voltage_next[][]   scratch for the new u after a step
 *   g_scene.inhibitor_recovery_next[][]  scratch for the new v
 *
 *   ACTIVATOR_REST,              fixed-point values (u*, v*) at the
 *   INHIBITOR_REST                quiescent state
 *   SUPRATHRESHOLD_KICK_VALUE    u value injected by SPACE / presets
 *
 *   FN_THRESHOLD_SHIFT           a — sets the resting u position
 *   FN_INHIBITOR_FEEDBACK        b — strength of v's pull on u
 *   FN_TIMESCALE_RATIO           ε — v's speed relative to u
 *   ACTIVATOR_DIFFUSION          D — diffusion coefficient for u
 *   EULER_DT                     dt for forward-Euler integration
 *
 *   g_scene.active_preset_index          which row of preset_table is live
 *   g_scene.simulation_paused            run/pause toggle
 *   g_scene.substeps_per_frame           Euler ticks between renders
 *
 * Background you need
 * ───────────────────
 *   - PDEs at a hand-wavy level: "rate of change depends on local
 *     value and on spatial derivatives."  T2 introduces the FN
 *     equations slowly.
 *   - Discrete Laplacian on a 5-point stencil (T6).  Just a
 *     neighbour sum minus the centre.
 *   - Forward Euler integration (T8).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Hodgkin-Huxley channel kinetics, real cardiac action
 *     potentials, or chemical kinetics of BZ.  FN is a
 *     PHENOMENOLOGICAL MODEL — it captures the qualitative
 *     dynamics with one cubic non-linearity.
 *   - Bifurcation theory.  We pick parameters in the excitable
 *     regime and demonstrate; we don't tour the parameter space.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm    : Forward Euler integration of the FITZHUGH-NAGUMO
 *                two-variable PDE on a 2-D grid.  Per cell:
 *                  - 5-point Laplacian for u (the only diffusing
 *                    variable);
 *                  - cubic local reaction u − u³/3;
 *                  - linear coupling −v drains the activator;
 *                  - linear inhibitor dynamics ε(u + a − b·v).
 *                Sub-stepping: each rendered frame advances the
 *                physics N times, so the effective time-resolution
 *                stays high without lowering dt below CFL.
 *
 * Math basis   : FitzHugh (1961) and Nagumo (1962) independently
 *                proposed a SIMPLIFIED HODGKIN-HUXLEY model — the
 *                full neuron model has four variables, FN compresses
 *                to two while preserving the key qualitative
 *                features:
 *                  - a STABLE RESTING STATE;
 *                  - an EXCITATION THRESHOLD;
 *                  - a LARGE-AMPLITUDE EXCURSION when threshold is
 *                    exceeded (the action potential);
 *                  - a REFRACTORY PERIOD before the cell can fire
 *                    again.
 *                These four ingredients are exactly what's needed
 *                for waves to propagate stably across a tissue —
 *                heart muscle, neural cortex, slime mold colony,
 *                BZ chemical reaction.
 *
 * Performance  : Four 2-D float arrays in BSS (no malloc).  Per
 *                cell per Euler tick: ~12 multiplies + 12 adds.
 *                At a typical 200 × 22 grid with 8 sub-steps/frame
 *                at 30 fps, ~10 M cell-updates/sec.  Trivial.
 *
 * References
 * ──────────
 *   ── FitzHugh-Nagumo model foundations ────────────────────────────
 *   [1] FitzHugh, R. (1961), "Impulses and Physiological States in
 *       Theoretical Models of Nerve Membrane", Biophys. J. 1(6),
 *       pp. 445-466 — THE foundational paper introducing the two-
 *       variable simplification of Hodgkin-Huxley.
 *   [2] Nagumo, J., Arimoto, S. & Yoshizawa, S. (1962), "An Active
 *       Pulse Transmission Line Simulating Nerve Axon", Proc. IRE
 *       50(10), pp. 2061-2070 — independent re-derivation in
 *       electrical-circuit form.  Together with [1] this defines the
 *       FitzHugh-Nagumo (FN) model.
 *   [3] Hodgkin, A. L. & Huxley, A. F. (1952), "A quantitative
 *       description of membrane current and its application to
 *       conduction and excitation in nerve", J. Physiol. 117 —
 *       the FULL biophysical model that FN simplifies.  Useful for
 *       context on what's being abstracted.
 *
 *   ── Spatial / pattern-formation theory ────────────────────────────
 *   [4] Murray, J. D. (2003), "Mathematical Biology II: Spatial
 *       Models and Biomedical Applications", Springer — ch. 1
 *       covers FN in biological context; ch. 5-7 cover spiral waves
 *       and excitable media generally.
 *   [5] Tyson, J. J. & Keener, J. P. (1988), "Singular Perturbation
 *       Theory of Travelling Waves in Excitable Media", Physica D
 *       32 — analytical treatment of the wave-front and -back
 *       structure visible in the spiral-wave preset.
 *
 *   ── Numerical methods ─────────────────────────────────────────────
 *   [6] LeVeque, R. J. (2007), "Finite Difference Methods for ODEs
 *       and PDEs", SIAM — explicit Euler + 5-point Laplacian theory
 *       behind §reaction's per-step update.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *   [7] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the membrane-potential glyph ramp.
 *   [8] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *   [9] https://en.wikipedia.org/wiki/FitzHugh%E2%80%93Nagumo_model
 *  [10] https://en.wikipedia.org/wiki/Spiral_wave
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every cell of the grid is a TINY EXCITABLE OSCILLATOR with TWO
 * STATES: resting (sleeping) and fired (excited).  Cells DIFFUSE
 * their excitation to neighbours — fired cells push neighbours
 * over their threshold, propagating the wave.  But after firing,
 * a cell can't immediately fire again (the inhibitor drives it
 * back to rest first).  The result: the wave moves OUTWARD only.
 * It can't reverse, because cells just behind the wavefront are
 * still in their refractory period.  This is what makes excitable-
 * medium waves UNIDIRECTIONAL despite obeying time-symmetric PDEs.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a vast field of dry grass.  Each blade of grass either
 * BURNS (fired state) or is DRY (resting).  Light a match in the
 * middle.  Fire spreads radially outward — burning blades ignite
 * their neighbours.  Once a blade has burned, it's just ash —
 * can't burn again until new grass grows (the refractory period).
 * Result: a circular front of fire, expanding outward, with a ring
 * of ash trailing it.  Eventually the fire reaches the edge and
 * dies out.  No back-wave, because the ash can't burn.
 *
 * Now imagine the grass REGENERATES SLOWLY.  The fire ring expands;
 * behind it, ash gradually returns to dry grass.  If you light a
 * second match near the first, you might create a DOUBLE-RING
 * pattern (preset 2).  If you start with a HALF RING, the open ends
 * curl back on themselves and form a SPIRAL (preset 3).
 *
 * This metaphor exactly mirrors the FN dynamics:
 *   - "burning"  = u high (firing)
 *   - "ash"      = v high (refractory; v has caught up to u)
 *   - "dry"      = both u and v near rest
 *   - "ignition" = diffusion of u from a fired cell to a dry one
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   PHASE PLANE (one cell):                            │
 *      │                                                      │
 *      │     v                                                │
 *      │      │  ●●● firing branch (u high, v rising)         │
 *      │      │ ●                                             │
 *      │      │●        ╲ slow inhibitor pulls v up           │
 *      │   ●──┼───────────●●●  refractory (high v, u dropping)│
 *      │      │                                                │
 *      │      ●  ●                                            │
 *      │      │      ●                                        │
 *      │      │           ●●●●●●● rest (u low, v slowly       │
 *      │      │                    relaxing back)             │
 *      │      └──────────────────────  u                      │
 *      │                                                      │
 *      │   Action potential = closed loop in (u, v) space.    │
 *      │   Triggered by a kick that crosses threshold.        │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   THE FITZHUGH-NAGUMO PDE SYSTEM:
 *     du/dt = u − u³/3 − v + D · ∇²u + I(t)        (activator)
 *     dv/dt = ε · (u + a − b · v)                   (inhibitor)
 *
 *   FORWARD EULER UPDATE (per cell, per Euler tick):
 *     u_new = u + dt · (u − u³/3 − v + D · ∇²u + I)
 *     v_new = v + dt · ε · (u + a − b · v)
 *
 *   5-POINT LAPLACIAN (Neumann boundary; out-of-grid → centre):
 *     ∇²u[r, c] ≈ u[r-1, c] + u[r+1, c] + u[r, c-1] + u[r, c+1]
 *               − 4 · u[r, c]
 *
 *   RESTING-STATE FIXED POINT (analytical):
 *     The system has dU/dt = dV/dt = 0 at (u*, v*).  Solving:
 *       v* = (u* + a) / b
 *       u* = solution of  u* − u*³/3 = v*
 *     For our parameters (a=0.7, b=0.8) this gives:
 *       u* ≈ −1.2,  v* ≈ −0.625
 *
 *   CFL STABILITY BOUND (T8):
 *     dt · D / dx² < 0.25
 *     With D = 0.10 and dx = 1, max dt = 2.5.  We use 0.04.  Massive
 *     margin — required because the CUBIC reaction term has its own
 *     stability constraints not captured in the linear CFL bound.
 *
 *   IMPULSE INJECTION (suprathreshold):
 *     For a circular region of radius R centred at (rc, cc):
 *       u[r, c] = SUPRATHRESHOLD_KICK_VALUE  for all (r, c) in the
 *                                              disc
 *
 *   GLYPH MAPPING (T9):
 *     u < −0.80  →  ' '  (rest)
 *     u < −0.20  →  '.'  (recovery / late refractory)
 *     u <  0.50  →  ':'  (rising / threshold cross)
 *     u <  1.20  →  '+'  (firing body)
 *     else       →  '#'  (peak)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - SUB-STEPPING is essential.  At dt = 0.04 and 8 sub-steps per
 *     frame, the simulation advances ~9.6 ms of model time per
 *     real frame at 30 fps.  Without sub-stepping (one tick per
 *     frame), waves would only advance 3 cells/sec — too slow to
 *     watch.
 *   - SUPRATHRESHOLD KICK must be well above the cubic's threshold
 *     (~0).  We use 2.0 (well above).  Sub-threshold kicks (~0.3)
 *     decay back to rest without firing — biologically correct, but
 *     uninformative for a demo.
 *   - NEUMANN (zero-flux) BOUNDARIES are implemented by clamping
 *     out-of-grid neighbour reads to the boundary cell itself.
 *     This means a wave reaching the edge effectively REFLECTS
 *     (boundary cells see themselves as their neighbour).  For
 *     the spiral preset this can create artefacts; in practice the
 *     refractory period prevents the reflected wave from being
 *     visible.
 *   - DOUBLE-BUFFERING is required.  The new value at (r, c)
 *     depends on the OLD values at (r, c) and its neighbours.
 *     Writing in place would propagate updates within a single
 *     sweep.  We use *_next scratch buffers and copy back.
 *   - SPIRAL FORMATION is sensitive.  The "broken ring" in
 *     preset_spiral() must have just the right shape: a complete
 *     ring would form rings; a totally absent ring would do
 *     nothing.  The half-erased ring leaves two TIPS that curl
 *     into each other.  Tip-curving is what makes spirals.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - At startup (preset 1, RINGS): a small bright spot at centre.
 *     Within ~2 seconds it expands into a clear circular wave with
 *     a fading trail (the refractory tail).
 *   - Press SPACE: a new wave nucleates at centre, propagates
 *     outward, and ANNIHILATES with any existing wave it meets
 *     (because the colliding waves are both refractory just behind
 *     each front).
 *   - Press '2' (double source): two waves meet in the middle and
 *     annihilate, leaving two outward-bound semi-circles.
 *   - Press '3' (spiral): a half-ring becomes a rotating spiral
 *     that fills the grid with a beautiful Belousov-Zhabotinsky-
 *     style pattern.  Takes ~5 seconds to develop.
 *   - Press '4' (plane wave): a single flat wave-front sweeps from
 *     left to right, leaving a wide refractory band behind it.
 *   - Press '+' / '-' to change speed.  At very high speeds the
 *     pattern evolves visibly faster but stays correct.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1   Excitable media — neurons, hearts, slime, BZ
 *   T2   The FitzHugh-Nagumo equations, term by term
 *   T3   Bistability, threshold, action potential
 *   T4   The cubic non-linearity — why u − u³/3
 *   T5   Diffusion of the activator — wave propagation
 *   T6   The 5-point Laplacian + Neumann boundaries
 *   T7   Spirals — broken rings and rotors
 *   T8   Forward Euler with sub-stepping — CFL safety
 *   T9   Visualisation — five activator bands
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  EXCITABLE MEDIA — NEURONS, HEARTS, SLIME, BZ
 * ────────────────────────────────────────────────
 * An EXCITABLE MEDIUM is any spatially-extended system where each
 * point has:
 *
 *   1. A STABLE RESTING STATE.  Left alone, it sits there.
 *   2. A THRESHOLD.  Below threshold → return to rest.
 *      Above threshold → fire (a large excursion in some variable).
 *   3. A REFRACTORY PERIOD.  After firing, the point briefly
 *      cannot fire again.
 *   4. COUPLING to neighbours.  When one point fires, it pushes its
 *      neighbours toward (or over) their thresholds.
 *
 * This combination is enough to produce TRAVELLING WAVES that
 * propagate without dispersion and ANNIHILATE on collision (because
 * both wavefronts are followed by refractory zones).  Waves form
 * RINGS from point sources, SPIRALS from broken rings, and complex
 * patterns from spatial heterogeneity.
 *
 * Real-world examples (all FUNDAMENTALLY THE SAME DYNAMICS):
 *
 *   NEURONS         — action potentials propagating along axons.
 *                     Hodgkin-Huxley (1952) is the full model;
 *                     FN is its 2-variable reduction.  Nobel Prize
 *                     1963.
 *   HEART MUSCLE    — coordinated contraction is a wave of
 *                     depolarisation sweeping through the heart.
 *                     Spiral waves in the heart = arrhythmia
 *                     (potentially fatal).  Defibrillation is
 *                     "reset to resting state."
 *   BZ REACTION     — Belousov-Zhabotinsky chemical oscillator,
 *                     discovered by accident in the 1950s.
 *                     Visible spiral waves in a Petri dish; the
 *                     iconic example of pattern formation.
 *   SLIME MOLD      — Dictyostelium discoideum cells signal each
 *                     other with cAMP pulses; the colony
 *                     aggregates via spiral waves.
 *   FOREST FIRES    — discrete-state version of the same dynamics
 *                     (fired/burnt/dry).  Spreads as fronts;
 *                     has refractory period (regrowth).
 *   CORTICAL SPREADING DEPRESSION
 *                   — slow wave of depolarisation sweeping the
 *                     visual cortex during a migraine aura.
 *
 * Studying ONE excitable medium teaches you ALL of them.  FN is the
 * minimal model: just two variables, one cubic non-linearity, one
 * diffusion term.  Strip anything more and you lose excitability;
 * add more and you're approaching real biophysics.
 *
 * T2  THE FITZHUGH-NAGUMO EQUATIONS, TERM BY TERM
 * ───────────────────────────────────────────────
 * Two coupled PDEs:
 *
 *     du/dt = u − u³/3 − v + D · ∇²u + I(t)        (activator)
 *     dv/dt = ε · (u + a − b · v)                   (inhibitor)
 *
 * Six terms.  Each has a meaning.
 *
 *   du/dt:
 *
 *     u − u³/3   THE BISTABLE CUBIC (T4).  At small |u|, this is
 *                ~u (positive feedback — u amplifies itself).  At
 *                large |u|, the −u³/3 dominates and pulls u back.
 *                The shape gives THREE FIXED POINTS along the
 *                u-axis: rest (u ≈ -1.2), threshold (u ≈ 0), fired
 *                (u ≈ +1.2).  Rest and fired are STABLE; threshold
 *                is UNSTABLE.  This is the source of bistability.
 *
 *     − v        ACTIVATOR-INHIBITOR COUPLING.  When v rises, it
 *                drains the activator.  This is what pulls the
 *                system back from "fired" to "rest" after enough
 *                time has passed.
 *
 *     D · ∇²u    DIFFUSION OF u.  Spreads activation to neighbours.
 *                D is the diffusion coefficient.  Without this, each
 *                cell oscillates independently and there's no
 *                wave propagation.
 *
 *     I(t)       EXTERNAL INPUT.  Stimulus from outside (the SPACE
 *                key).  Must be SUPRATHRESHOLD (push u past the
 *                middle fixed point) to trigger an action potential.
 *
 *   dv/dt:
 *
 *     ε          THE TIMESCALE RATIO.  ε ≪ 1 means v evolves
 *                MUCH SLOWER than u.  This time-scale separation
 *                is what makes the action potential a SLOW LOOP
 *                in (u, v) space rather than a quick decay back.
 *                We use ε = 0.08 — v is ~12× slower than u.
 *
 *     u          v rises in proportion to current u.  When u is
 *                high (firing), v starts rising.  When u is low
 *                (resting), v decays.
 *
 *     + a        A CONSTANT BIAS.  Sets the resting position.
 *                With a > 0, v* (resting v) is positive.  With
 *                a < 0, v* is negative.  Tunes the resting state
 *                relative to the cubic.  We use a = 0.7.
 *
 *     − b · v    LINEAR DECAY.  v decays at rate b.  b sets the
 *                "stiffness" of v's recovery.  We use b = 0.8.
 *
 * Together the system has FOUR PARAMETERS: a, b, ε, D.  Plus dt
 * (numerical, not physical) and the impulse strength.  At our
 * (a, b, ε, D) = (0.7, 0.8, 0.08, 0.10) the system is in the
 * EXCITABLE REGIME — single stable rest state, threshold-triggered
 * firing.  Other parameter regions give:
 *
 *   OSCILLATORY    — no rest state, the cell limit-cycles forever
 *                     (b < 1 with appropriate a).
 *   BISTABLE       — two stable states, no firing
 *                     (cubic dominant, weak inhibitor).
 *
 * T3  BISTABILITY, THRESHOLD, ACTION POTENTIAL
 * ────────────────────────────────────────────
 * Imagine just the cubic term:  du/dt = u − u³/3.
 *
 * Set du/dt = 0 → u(1 − u²/3) = 0 → u = 0, ±√3.  Three fixed points.
 * Linearising: u = ±√3 are stable (slope = -2 < 0); u = 0 is
 * unstable (slope = 1 > 0).  So you have TWO BASINS OF ATTRACTION
 * separated by the unstable threshold:
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  du/dt = u − u³/3                                    │
 *      │                                                      │
 *      │     ●  rest                  ●  fired                │
 *      │  −√3                         +√3                     │
 *      │   ──●─────────────╲▲╱──────────●──── u              │
 *      │           threshold (0)                              │
 *      │                                                      │
 *      │  Push u from −√3 toward 0.  If you stop short of 0,  │
 *      │  u slides back to −√3.  If you cross 0, u jumps to   │
 *      │  +√3.  THRESHOLD-TRIGGERED BISTABILITY.              │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Now add the inhibitor v.  At v = 0 the cell is bistable.  But v
 * is dragged upward when u is high; once v rises enough, it pulls
 * the cubic's basins TOGETHER and only ONE fixed point remains —
 * the resting state.  The cell falls back to rest, v gradually
 * decays, and the cell is ready to fire again.
 *
 * The trajectory in (u, v) space is the ACTION POTENTIAL:
 *
 *   1. Start at rest (u*, v*).
 *   2. Suprathreshold kick → u jumps over threshold.
 *   3. u rapidly climbs to firing branch (u ≈ +√3).
 *   4. v slowly rises while u sits high.
 *   5. v gets high enough that the firing fixed point disappears.
 *   6. u rapidly drops to recovery branch (low u, high v).
 *   7. v slowly decays back to v*.
 *   8. u gradually returns to u*.  Back to rest.
 *
 * Steps 3 and 6 are FAST (u dynamics).  Steps 4 and 7 are SLOW
 * (v dynamics).  Time-scale separation between u and v is what
 * gives the action potential its characteristic shape.
 *
 * T4  THE CUBIC NON-LINEARITY — WHY u − u³/3
 * ──────────────────────────────────────────
 * Why a cubic?  Why not just (u − threshold) — a linear threshold?
 *
 * A LINEAR system can't be bistable.  If du/dt = m·u, every
 * trajectory either decays to 0 (m < 0) or explodes (m > 0).  No
 * fixed points other than 0.
 *
 * The simplest non-linearity that gives bistability is a CUBIC
 * with three real roots.  The form u − u³/3 is the canonical
 * example; you could also use u(1 − u)(1 + u) or any other cubic
 * with three roots.  FitzHugh chose u − u³/3 because it's
 * mathematically clean and the fixed points are at convenient
 * values.
 *
 * Geometric picture: plot du/dt vs u with v fixed.  The cubic gives
 * an S-CURVE (the "nullcline"):
 *
 *      du/dt
 *        ▲
 *        │   ╱╲                    /
 *        │  ╱  ╲                  /
 *        │ ╱    ╲                /
 *        ●──────●               ●
 *        │       ╲             /
 *        │        ╲           /
 *        │         ╲         /
 *        │          ╲╱
 *
 * Where du/dt crosses 0 are the fixed points.  Three crossings →
 * three fixed points.  The middle one is unstable (slope > 0).
 * The outer two are stable (slope < 0).
 *
 * When v changes, the curve shifts UP or DOWN.  If v gets high
 * enough, the upper hump dips below 0 and the upper-stable point
 * disappears.  Now only the resting fixed point exists, and any
 * trajectory there falls back to rest.  This is HOW v cuts the
 * action potential off — by changing the cubic's shape until firing
 * is no longer self-sustaining.
 *
 * T5  DIFFUSION OF THE ACTIVATOR — WAVE PROPAGATION
 * ─────────────────────────────────────────────────
 * Note: in FN, ONLY u DIFFUSES.  v stays local.  Why?
 *
 * Physically, in real neurons the activator (membrane voltage) is
 * spread by ELECTRICAL CABLE conduction along the axon.  The
 * inhibitor (potassium channel kinetics) is LOCAL — each patch of
 * membrane has its own channel state, not shared.  FN preserves
 * this asymmetry.
 *
 * Numerically, this means the Laplacian appears only in the u
 * equation: D · ∇²u.  In code, this is exactly ONE Laplacian per
 * cell per step, computed from the four cardinal neighbours of u.
 *
 * Wave propagation comes from this term.  When u fires somewhere,
 * the firing cell's u value is high.  Diffusion to neighbours
 * means those neighbours' u rises.  If a neighbour's u rises ABOVE
 * THRESHOLD, that neighbour fires too.  The wave propagates one
 * cell-row per few ticks.
 *
 * Wave SPEED is set jointly by D (how fast diffusion spreads u)
 * and the cubic kinetics (how fast a cell crosses threshold once
 * pushed).  Higher D → faster waves.  Steeper cubic → faster waves.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   t=0      t=1      t=2      t=3                     │
 *      │                                                      │
 *      │   .###.    .#####.   .#######.   .#########.         │
 *      │   .###.    .#####.   .#######.   .#########.         │
 *      │     ↑                                                 │
 *      │   firing     wave     wave        wave still          │
 *      │   centre     reaches  reaches     expanding,          │
 *      │              row 1    row 2       refractory tail     │
 *      │                                   forms                │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T6  THE 5-POINT LAPLACIAN + NEUMANN BOUNDARIES
 * ──────────────────────────────────────────────
 * The Laplacian on a 5-point stencil:
 *
 *           u[r-1][c]
 *               │
 *   u[r][c-1]──●──u[r][c+1]
 *               │
 *           u[r+1][c]
 *
 *   ∇²u[r, c] ≈ u[r-1, c] + u[r+1, c] + u[r, c-1] + u[r, c+1]
 *             − 4 · u[r, c]
 *
 * 5-point is ANISOTROPIC — it weights horizontal/vertical
 * differently from diagonals (which are absent).  Visible
 * artefact: spreading waves tend to grow as DIAMONDS rather than
 * circles, especially near the source.  The 9-point isotropic
 * stencil (used in reaction_diffusion.c) fixes this at 2× the
 * memory traffic.  We use 5-point here because FN's wave dynamics
 * are dominated by the cubic non-linearity, not the diffusion
 * geometry — the diamond artefact is barely visible after a few
 * ticks of propagation.
 *
 * BOUNDARY CONDITIONS.  When (r, c) is a boundary cell, some
 * neighbours are out of the grid.  Three common choices:
 *
 *   DIRICHLET (fixed)      — set to a constant.  Waves die at edges.
 *   NEUMANN (zero-flux)    — clamp neighbour read to the BOUNDARY
 *                              cell itself.  Waves reflect at edges.
 *   PERIODIC (toroidal)    — wrap to the opposite edge.  Waves go
 *                              around the torus.
 *
 * We use NEUMANN.  Implementation: when reading a neighbour at
 * row r-1 with r = 0, use row 0 instead:
 *
 *     int r_above = (r > 0)             ? r - 1 : 0;
 *     int r_below = (r < rows - 1)      ? r + 1 : rows - 1;
 *
 * Effect: a wave reaching the boundary "sees itself" as its own
 * neighbour, which means the diffusion gradient at the boundary
 * is approximately zero — the wave reflects.  In practice the
 * refractory tail behind the wavefront prevents the reflected
 * wave from being visible, so this choice gives a clean look.
 *
 * T7  SPIRALS — BROKEN RINGS AND ROTORS
 * ─────────────────────────────────────
 * The most striking pattern in excitable media is the SPIRAL WAVE.
 * In real cardiac tissue, spirals = arrhythmia.  In BZ chemistry,
 * they're the iconic image.  In slime mold colonies, they direct
 * cell aggregation.
 *
 * How does a spiral form?  By BREAKING A RING.
 *
 * Start with a target ring (preset 1): a single point fires, the
 * wave expands as a circle.  Inside the circle, all cells are
 * refractory — they can't fire.  Outside the circle, all cells
 * are ready to fire.  The wavefront is the CIRCLE BOUNDARY.
 *
 * Now imagine you ERASE THE LEFT HALF OF THE CIRCLE.  The right
 * half remains: a SEMI-CIRCULAR WAVEFRONT with two FREE TIPS at
 * top and bottom.  At a tip, the front meets resting tissue on
 * one side and refractory tissue on the other (the inside of the
 * original circle).
 *
 * The tip is a CURVATURE singularity.  Diffusion of u is highest
 * where curvature is highest — at the tip.  This means the tip
 * propagates fastest INWARD (into resting tissue) AND is pulled
 * by the refractory region just behind it.  The combination causes
 * the tip to CURL BACK ON ITSELF.  Both tips do this; they spiral
 * around the centre.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │   t=0 (broken ring)        t=∞ (rotating spiral)     │
 *      │                                                      │
 *      │       ##                       ## ##                  │
 *      │    ## .. ##                 ##       ##               │
 *      │   ##  ..  ##               ##  ## ##  ##              │
 *      │    ## .. ##                ## ##   ## ##              │
 *      │       ##                    ##       ##               │
 *      │   tips at top/bottom        ## ##                     │
 *      │   curl into spirals                                   │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * Once formed, the spiral ROTATES forever (or until it hits the
 * boundary or another wave).  The tip moves along a CIRCULAR ORBIT
 * of fixed radius; the wave fills the rest of the medium.
 *
 * preset_spiral() implements this by drawing a complete ring then
 * erasing the left half before letting the simulation evolve.
 * Cleanly designing the broken-ring shape is non-obvious — too
 * symmetric and you get rings; too asymmetric and the front dies.
 * The chosen geometry consistently produces a stable rotor.
 *
 * T8  FORWARD EULER WITH SUB-STEPPING — CFL SAFETY
 * ────────────────────────────────────────────────
 * Forward Euler is the simplest possible time-integrator:
 *
 *     u_new = u + dt · (du/dt)
 *
 * For diffusion alone, the CFL bound is dt · D / dx² < 0.25.  At
 * D = 0.10 and dx = 1, max dt = 2.5.  We use dt = 0.04 — way
 * below the linear bound.
 *
 * Why so low?  Because the CUBIC reaction term has its own
 * stability requirements that aren't captured by the linear CFL.
 * Empirically, dt = 0.04 keeps the simulation stable across all
 * presets and all parameter choices we use.  Bumping dt to 0.1 or
 * higher introduces visible numerical "ringing" near sharp
 * wavefronts.
 *
 * BUT — at dt = 0.04 with one tick per render frame at 30 fps,
 * the simulation advances 1.2 ms of model time per real second.
 * Waves crawl visibly slowly.  Bored.
 *
 * SOLUTION: SUB-STEPPING.  Run N Euler ticks per rendered frame:
 *
 *     for each render frame:
 *         for k = 1..N:
 *             reaction_step()     // advance dt
 *         render()
 *
 * With N = 8 sub-steps, the simulation advances 8·0.04 = 0.32 of
 * model time per render frame.  At 30 fps, that's 9.6 of model
 * time per real second.  Waves move at a watchable pace.
 *
 * The user controls N via '+' / '-' keys (T1 demo).  Higher N =
 * faster visual evolution, more CPU per frame.
 *
 * T9  VISUALISATION — FIVE ACTIVATOR BANDS
 * ────────────────────────────────────────
 * The activator u ranges from about -1.2 (rest) to about +2.1
 * (kicked), with about +1.2 being the "fired" plateau.  We map
 * u to ONE OF FIVE GLYPHS based on threshold bands:
 *
 *     u < -0.80    →  ' '   resting (transparent — left blank)
 *     u < -0.20    →  '.'   late refractory recovery
 *     u <  0.50    →  ':'   threshold cross / rising
 *     u <  1.20    →  '+'   firing body (action potential plateau)
 *     u ≥  1.20    →  '#'   peak (just-kicked or freshly fired)
 *
 * The boundaries are chosen so that:
 *   - resting cells are INVISIBLE (full screen of '.' would be
 *     visually busy)
 *   - the rising edge gets its own glyph (':') for clear front
 *     marking
 *   - the firing plateau is the densest typical glyph ('+')
 *   - peaks (just-kicked) get bold '#' to emphasise injection
 *
 * Each glyph maps to a colour pair (REST blue → REC darker blue →
 * RISE cyan → WAVE white → FRONT bright white).  The brightness
 * gradient mirrors the temporal evolution: resting is dim, peak is
 * bright.  A_BOLD is added to the FRONT pair for extra punch.
 *
 * Inhibitor v is NOT visualised — the activator already encodes
 * the wave structure.  Adding a v overlay would clutter the
 * display.  In a debug build you might add a v-density overlay
 * toggleable via a key, similar to the debug overlays in
 * fluid/nuke.c.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — every constant + enums                                   */
/* ===================================================================== */

/* §1.1 — FitzHugh-Nagumo physical parameters (T2). */

/* a — threshold-shift parameter.  Sets the position of the resting
 * fixed point u*.  Higher a → more negative u*.  We use 0.7. */
#define FN_THRESHOLD_SHIFT 0.70f

/* b — inhibitor feedback strength.  b < 1 gives oscillatory
 * regime; b ≥ 1 gives excitable.  We use 0.8 for excitability with
 * a clean refractory period. */
#define FN_INHIBITOR_FEEDBACK 0.80f

/* ε — v/u time-scale ratio.  ε ≪ 1 = slow inhibitor.  We use
 * 0.08, so v evolves ~12× slower than u (gives a clear action-
 * potential plateau). */
#define FN_TIMESCALE_RATIO 0.08f

/* D — activator diffusion coefficient.  Only u diffuses; v is
 * local.  Higher D → faster, broader waves. */
#define ACTIVATOR_DIFFUSION 0.10f

/* dt — explicit Euler step (T8).  CFL linear bound: dt · D < 0.25
 * with dx = 1.  We use 0.04 (way below) for cubic stability. */
#define EULER_DT 0.04f

/* §1.2 — Resting-state fixed point + suprathreshold kick. */

/* (u*, v*) — analytical resting state at our (a, b) parameters. */
#define ACTIVATOR_REST (-1.20f)
#define INHIBITOR_REST (-0.625f)

/* Suprathreshold kick value injected by the SPACE key.  Must be
 * well above the cubic threshold (~0) — we use 2.0 (well above). */
#define SUPRATHRESHOLD_KICK_VALUE 2.00f

/* §1.3 — Sub-stepping (T8). */

#define SUBSTEPS_PER_FRAME_DEFAULT 8
#define SUBSTEPS_PER_FRAME_MIN 1
#define SUBSTEPS_PER_FRAME_MAX 20

/* §1.4 — Render frame rate. */

#define RENDER_FPS_CAP 30
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.5 — Grid size limits + HUD reservation. */

#define GRID_COLS_MAX 320
#define GRID_ROWS_MAX 100
#define HUD_RESERVED_ROWS_TOP 1    /* row 0 — status              */
#define HUD_RESERVED_ROWS_BOTTOM 1 /* row rows-1 — hint           */

/* §1.6 — Activator-band glyph thresholds (T9). */

#define U_BAND_REST_HIGH (-0.80f)
#define U_BAND_RECOVERY_HIGH (-0.20f)
#define U_BAND_RISING_HIGH 0.50f
#define U_BAND_WAVE_HIGH 1.20f

/* §1.7 — Impulse geometry. */

#define IMPULSE_RADIUS_CELLS 4
#define SPIRAL_PRIMER_RADIUS 5

/* §1.8 — Colour pair IDs. */

enum {
  PAIR_BAND_REST = 1, /* u < -0.80 — dark blue            */
  PAIR_BAND_RECOVERY, /* u < -0.20 — medium blue           */
  PAIR_BAND_RISING,   /* u <  0.50 — cyan                 */
  PAIR_BAND_WAVE,     /* u <  1.20 — pale cyan-white      */
  PAIR_BAND_FRONT,    /* u ≥  1.20 — bright white + bold  */
  PAIR_HUD,           /* bright yellow + bold (top)        */
  PAIR_HINT,          /* bright cyan   + bold (bottom)     */
};

/* §1.9 — Preset count (table in §10). */

#define PRESET_COUNT 4

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
/* §3  colors — 5-band activator ramp + HUD pairs                        */
/* ===================================================================== */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_BAND_REST, 25, -1);     /* dark blue        */
    init_pair(PAIR_BAND_RECOVERY, 27, -1); /* medium blue      */
    init_pair(PAIR_BAND_RISING, 51, -1);   /* cyan             */
    init_pair(PAIR_BAND_WAVE, 195, -1);    /* pale cyan-white  */
    init_pair(PAIR_BAND_FRONT, 231, -1);   /* bright white     */
    init_pair(PAIR_HUD, 226, -1);          /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);          /* bright cyan      */
  } else {
    init_pair(PAIR_BAND_REST, COLOR_BLUE, -1);
    init_pair(PAIR_BAND_RECOVERY, COLOR_BLUE, -1);
    init_pair(PAIR_BAND_RISING, COLOR_CYAN, -1);
    init_pair(PAIR_BAND_WAVE, COLOR_CYAN, -1);
    init_pair(PAIR_BAND_FRONT, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* ===================================================================== */
/* §4  scene — the single owner of all simulation + render state         */
/* ===================================================================== */
/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Earlier versions kept scene state as flat file-scope globals.
 *   This Scene struct groups the same fields together so the sim /
 *   tuning / control split is enforced by the STRUCT LAYOUT, not just
 *   by convention.  Helpers continue to reach the state through
 *   `g_scene.<field>` so the inner-loop FitzHugh-Nagumo stencil math
 *   stays free of pointer threading.
 *
 *   The two-species FN model needs four 2-D arrays: the activator V
 *   ("voltage", the EXCITABLE variable) + scratch, and the inhibitor
 *   W ("recovery") + scratch.  After each Euler tick the *_next
 *   scratches hold the new values; we COPY them back into the live
 *   arrays.
 *
 * Why copy instead of pointer-swap
 *   Static 2-D arrays (`float arr[ROWS][COLS]`) cannot share an
 *   address with a pointer-rotation, so a clean swap would require
 *   heap allocation.  Per CLAUDE.md "no dynamic allocation after
 *   init" we stick with BSS + memcpy; the O(N²) copy cost is the
 *   same order as the neighbour-read loop so it's invisible.
 *
 * Locality (sim vs render)
 *   Fields are GROUPED EXPLICITLY by subsystem:
 *     - reaction / diffuse passes read it    → simulation
 *     - paint_field / hud_paint_* reads it   → rendering
 *     - both sides bound their loops by
 *       g_scene.grid_active_rows / _cols             → shared geometry
 *     - g_scene.simulation_paused gates the tick     → control state
 *
 *   Mis-classifying a field — e.g. letting a render helper bump
 *   g_scene.substeps_per_frame — would couple visuals to physics and break
 *   the engine-toggle invariant (same preset must evolve identically
 *   regardless of how it's drawn).
 *
 * Why one big struct (not split across Scene + App + Field)
 *   Every reaction-diffusion pass needs the whole field set.
 *   Splitting would force pointer chains in the inner loop — the
 *   loop already runs ~12 multiplies + 12 adds per cell and the
 *   extra indirection would noticeably slow it.  Keeping one Scene
 *   + one g_scene instance matches CLAUDE.md's "no dynamic
 *   allocation after init": 4 × ROWS_MAX × COLS_MAX floats in BSS.
 *
 * Why these specific fields and no others
 *   - g_scene.activator_voltage           V field (the EXCITABLE variable).
 *   - g_scene.inhibitor_recovery          W field (the RECOVERY variable).
 *   - g_scene.activator_voltage_next      V scratch (Euler-step buffer).
 *   - g_scene.inhibitor_recovery_next     W scratch (Euler-step buffer).
 *   - g_scene.grid_active_rows / _cols    active subregion bounds.
 *   - g_scene.active_preset_index         reset-time IC choice; HUD reads name.
 *   - g_scene.simulation_paused           gate for the per-frame step pipeline.
 *   - g_scene.substeps_per_frame          Euler sub-steps per render frame
 *                                   (visual speed knob; doesn't change
 *                                   the per-step PDE stability bound).
 *
 * Things that DO NOT live here
 *   - The 4 preset loaders + name table → file-scope constants
 *   - Render-frame timing / FPS         → locals in main()
 *   - Signal flags                      → file-scope volatile
 *
 * Reference [1] FitzHugh for the two-variable formulation that this
 *   layout implements directly (V = membrane potential, W = recovery).
 */
typedef struct {
    /* ── Simulation state (read by reaction/diffuse passes) ────── */
    float activator_voltage      [GRID_ROWS_MAX][GRID_COLS_MAX]; /* V  */
    float inhibitor_recovery     [GRID_ROWS_MAX][GRID_COLS_MAX]; /* W  */
    float activator_voltage_next [GRID_ROWS_MAX][GRID_COLS_MAX]; /* V' */
    float inhibitor_recovery_next[GRID_ROWS_MAX][GRID_COLS_MAX]; /* W' */

    /* ── Shared geometry (sim AND render) ──────────────────────── */
    int grid_active_rows;
    int grid_active_cols;

    /* ── Control state (gates the tick + drives HUD) ──────────── */
    int  active_preset_index;
    bool simulation_paused;

    /* ── Simulation tuning ───────────────────────────────────── */
    int substeps_per_frame;
} Scene;

static Scene g_scene = {
    .substeps_per_frame = SUBSTEPS_PER_FRAME_DEFAULT,
};

/* ===================================================================== */
/* §5  neighbours — boundary-clamped index helpers (T6)                  */
/* ===================================================================== */
/*
 * Neumann (zero-flux) boundary: out-of-grid neighbour reads return
 * the boundary cell itself.  Effect: waves reflect at edges.
 */

static inline int row_above_clamped(int row) { return (row > 0) ? row - 1 : 0; }

static inline int row_below_clamped(int row) {
  return (row < g_scene.grid_active_rows - 1) ? row + 1 : g_scene.grid_active_rows - 1;
}

static inline int col_left_clamped(int col) { return (col > 0) ? col - 1 : 0; }

static inline int col_right_clamped(int col) {
  return (col < g_scene.grid_active_cols - 1) ? col + 1 : g_scene.grid_active_cols - 1;
}

/* ===================================================================== */
/* §6  laplacian — 5-point stencil for the activator (T6)                */
/* ===================================================================== */
/*
 * 5-point stencil:
 *   ∇²u[r, c] ≈ u[r-1, c] + u[r+1, c] + u[r, c-1] + u[r, c+1]
 *             − 4 · u[r, c]
 *
 * Inlined into reaction_step() for cache efficiency — the stencil
 * shares its centre-cell read with the reaction term computation.
 * §6 has no separate function; the FORMULA lives here as
 * documentation.
 */

#define LAPLACIAN_5POINT_CENTRE_WEIGHT 4.0f

/* ===================================================================== */
/* §7  reaction_step — one Euler tick of the FN PDE                      */
/* ===================================================================== */
/*
 * Per cell:
 *   1. Read u, v at (r, c).
 *   2. Compute the 5-point Laplacian of u.
 *   3. Compute du/dt = u − u³/3 − v + D · ∇²u   (no impulse — those
 *      go through inject_circular_impulse() instead).
 *   4. Compute dv/dt = ε · (u + a − b · v).
 *   5. Forward Euler update.
 *   6. Write into the *_next scratches.
 * Then COPY scratches back to the live arrays.
 */

/* Clamped neighbour indices for cell (r, c) under Neumann (zero-flux)
 * boundary conditions: out-of-grid neighbour reads return the boundary
 * cell itself.  Effect: waves REFLECT at edges rather than passing
 * through or being absorbed.  Compare to reaction_diffusion's TOROIDAL
 * wrap which makes the grid a torus. */
typedef struct {
    int row_above, row_below;
    int col_left,  col_right;
} ClampedNeighbours;

static inline ClampedNeighbours clamped_neighbours_of(int r, int c) {
    ClampedNeighbours n;
    n.row_above = row_above_clamped(r);
    n.row_below = row_below_clamped(r);
    n.col_left  = col_left_clamped (c);
    n.col_right = col_right_clamped(c);
    return n;
}

/* 5-point Laplacian stencil at cell (r, c):
 *   ∇²u ≈ u_N + u_S + u_E + u_W − 4·u_C
 * (LAPLACIAN_5POINT_CENTRE_WEIGHT is 4.0f.)  Anisotropic — the
 * preferred 9-point form in reaction_diffusion.c would give the
 * same spirals more symmetrically, but the 5-point is fine for
 * the FN regime because gradients are gentle.  Refs [6] LeVeque §10. */
static inline float laplacian_5point_at(const float field[GRID_ROWS_MAX][GRID_COLS_MAX],
                                         int r, int c, ClampedNeighbours n) {
    return field[n.row_above][c           ]
         + field[n.row_below][c           ]
         + field[r           ][n.col_left ]
         + field[r           ][n.col_right]
         - LAPLACIAN_5POINT_CENTRE_WEIGHT * field[r][c];
}

/* FitzHugh-Nagumo activator equation [1] FitzHugh 1961:
 *   du/dt = u − u³/3 − v + D·∇²u
 * The cubic term u³/3 is what gives the system its excitable
 * threshold; below it the activator decays, above it it RUNS AWAY
 * until the inhibitor v catches up.  Refs [1] FitzHugh 1961. */
static inline float fitzhugh_nagumo_du_dt(float u, float v, float laplacian_u) {
    return u - (u * u * u) / 3.0f - v + ACTIVATOR_DIFFUSION * laplacian_u;
}

/* FitzHugh-Nagumo inhibitor equation:
 *   dv/dt = ε · (u + a − b·v)
 * The recovery variable that chases u with a slow ε timescale.
 * Refs [1] FitzHugh 1961, [2] Nagumo et al. 1962. */
static inline float fitzhugh_nagumo_dv_dt(float u, float v) {
    return FN_TIMESCALE_RATIO * (u + FN_THRESHOLD_SHIFT
                                   - FN_INHIBITOR_FEEDBACK * v);
}

/* Forward-Euler integration step: value + Δt · derivative.  First-
 * order accurate.  Stability bound D·Δt/(Δx)² ≤ 1/2 (heat eqn rule)
 * is satisfied by EULER_DT in §1. */
static inline float forward_euler_step(float value, float derivative) {
    return value + EULER_DT * derivative;
}

/* Copy the *_next scratches back to the live U/V arrays.  No pointer
 * swap because the arrays are STATIC 2-D (not pointers — see Scene
 * struct doc for the "static + memcpy vs pointer-swap" rationale). */
static inline void copy_scratches_to_live_fields(void) {
    memcpy(g_scene.activator_voltage,    g_scene.activator_voltage_next,
           sizeof g_scene.activator_voltage);
    memcpy(g_scene.inhibitor_recovery,   g_scene.inhibitor_recovery_next,
           sizeof g_scene.inhibitor_recovery);
}

/*
 * reaction_step — one full FitzHugh-Nagumo Euler step.
 *
 * Pseudocode:
 *   for each cell (r, c):
 *     n        = clamped_neighbours_of(r, c)                ← Neumann BC
 *     lap_u    = laplacian_5point_at(U, r, c, n)
 *     du_dt    = fitzhugh_nagumo_du_dt(u, v, lap_u)
 *     dv_dt    = fitzhugh_nagumo_dv_dt(u, v)
 *     U_next[r, c] = forward_euler_step(u, du_dt)
 *     V_next[r, c] = forward_euler_step(v, dv_dt)
 *   copy_scratches_to_live_fields()
 *
 * Refs [1] FitzHugh 1961 (excitable dynamics); [6] LeVeque (Euler).
 */
static void reaction_step(void) {
    for (int r = 0; r < g_scene.grid_active_rows; r++) {
        for (int c = 0; c < g_scene.grid_active_cols; c++) {
            ClampedNeighbours n = clamped_neighbours_of(r, c);
            float u             = g_scene.activator_voltage  [r][c];
            float v             = g_scene.inhibitor_recovery [r][c];

            float lap_u         = laplacian_5point_at(g_scene.activator_voltage,
                                                      r, c, n);
            float du_dt         = fitzhugh_nagumo_du_dt(u, v, lap_u);
            float dv_dt         = fitzhugh_nagumo_dv_dt(u, v);

            g_scene.activator_voltage_next [r][c] = forward_euler_step(u, du_dt);
            g_scene.inhibitor_recovery_next[r][c] = forward_euler_step(v, dv_dt);
        }
    }

    copy_scratches_to_live_fields();
}

/* ===================================================================== */
/* §8  reset — fill the field with the resting state                     */
/* ===================================================================== */
/*
 * Set every cell to the analytical fixed point (u*, v*) = (-1.2,
 * -0.625).  This is the QUIESCENT initial condition; nothing
 * happens after reset until an impulse or preset injects energy.
 */

static void grid_reset_to_rest(void) {
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      g_scene.activator_voltage[r][c] = ACTIVATOR_REST;
      g_scene.inhibitor_recovery[r][c] = INHIBITOR_REST;
    }
  }
}

/* ===================================================================== */
/* §9  impulse — circular suprathreshold blob                            */
/* ===================================================================== */
/*
 * Drop a disc of u = SUPRATHRESHOLD_KICK_VALUE centred at (rc, cc)
 * with radius R (cells).  The disc fires immediately (well above
 * threshold) and propagates outward as a ring.
 *
 * The inhibitor v is NOT touched — leaving v at rest means the
 * fired cells start their action-potential cycle from the rest
 * point of v, which is the natural "spontaneous firing"
 * trajectory.
 */

/* Is cell (r, c) inside the active simulation grid? */
static inline bool cell_in_active_grid(int r, int c) {
    return r >= 0 && r < g_scene.grid_active_rows
        && c >= 0 && c < g_scene.grid_active_cols;
}

/* Is cell (r, c) inside the disc of radius √radius_squared centred
 * at (rc, cc)?  Uses squared distance — avoids the sqrt that
 * |Δr| + |Δc| < r would only approximate. */
static inline bool cell_inside_disc(int r, int c, int rc, int cc,
                                     int radius_squared) {
    int dr = r - rc;
    int dc = c - cc;
    return dr * dr + dc * dc <= radius_squared;
}

/* Fire one cell: set the activator to the suprathreshold kick value.
 * The inhibitor v is intentionally LEFT AT REST so the cell starts
 * its action-potential cycle from the rest point — the natural
 * spontaneous-firing trajectory of the FN excitable medium. */
static inline void fire_one_cell(int r, int c) {
    g_scene.activator_voltage[r][c] = SUPRATHRESHOLD_KICK_VALUE;
}

/*
 * inject_circular_impulse — drop a disc of suprathreshold activator.
 *
 * Pseudocode:
 *   r² = radius²
 *   for each cell (r, c) in the bounding square [rc±radius, cc±radius]:
 *     if not cell_in_active_grid(r, c)            : skip (out of grid)
 *     if not cell_inside_disc(r, c, rc, cc, r²)   : skip (out of disc)
 *     fire_one_cell(r, c)
 *
 * The disc fires immediately and propagates outward as a ring — the
 * canonical "ripple from a stone" demonstration of excitable media.
 */
static void inject_circular_impulse(int rc, int cc, int radius) {
    int radius_squared = radius * radius;
    for (int r = rc - radius; r <= rc + radius; r++) {
        for (int c = cc - radius; c <= cc + radius; c++) {
            if (!cell_in_active_grid (r, c))                          continue;
            if (!cell_inside_disc    (r, c, rc, cc, radius_squared))  continue;
            fire_one_cell(r, c);
        }
    }
}

/* ===================================================================== */
/* §10  presets — 4 scene loaders + table                                */
/* ===================================================================== */

/*
 * Preset — one named initial-condition scene the user can flip to.
 *
 * Intent
 *   Each preset is a SCENARIO that highlights a different aspect of
 *   FN excitable-media dynamics: target rings (point-like trigger
 *   produces concentric circles), double source (interfering targets),
 *   spiral wave (the canonical excitable-media spiral), etc.  Pressing
 *   a number key calls the preset's loader, which writes initial
 *   activator / inhibitor values into the field.
 *
 * Why a function-pointer loader (not a parameter table)
 *   Unlike Gray-Scott where presets differ in (F, k) NUMERIC
 *   parameters, FN presets differ in INITIAL-CONDITION GEOMETRY
 *   (ring vs spiral vs two sources).  A function pointer per preset
 *   lets each loader write whatever spatial pattern it needs without
 *   trying to encode "ring centre + radius + offset + ..." into a
 *   shared parameter struct.
 *
 * References [4] Murray ch. 5-7 for spiral waves; [5] Tyson & Keener
 *   1988 for wave-front / wave-back structure these presets exhibit.
 */
typedef struct {
    const char *display_name;        /* short HUD label                 */
    void      (*loader)(void);       /* writes initial conditions       */
} Preset;

static void preset_target_rings(void);
static void preset_double_source(void);
static void preset_spiral_wave(void);
static void preset_plane_wave(void);

static const Preset preset_table[PRESET_COUNT] = {
    {"TARGET RINGS  ", preset_target_rings},
    {"DOUBLE SOURCE ", preset_double_source},
    {"SPIRAL WAVE   ", preset_spiral_wave},
    {"PLANE WAVE    ", preset_plane_wave},
};

/* Single point source at the centre. */
static void preset_target_rings(void) {
  grid_reset_to_rest();
  inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                          IMPULSE_RADIUS_CELLS);
}

/* Two sources at thirds.  Their wavefronts will meet in the
 * middle and annihilate (both refractory just behind their
 * leading edges). */
static void preset_double_source(void) {
  grid_reset_to_rest();
  int row = g_scene.grid_active_rows / 2;
  inject_circular_impulse(row, g_scene.grid_active_cols / 3, IMPULSE_RADIUS_CELLS);
  inject_circular_impulse(row, g_scene.grid_active_cols * 2 / 3, IMPULSE_RADIUS_CELLS);
}

/* Broken ring → rotating spiral (T7).  Place a full ring, then
 * erase the LEFT HALF.  The two free tips at top and bottom curl
 * back into the rest of the medium and form a stable rotor. */
static void preset_spiral_wave(void) {
  grid_reset_to_rest();
  inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                          SPIRAL_PRIMER_RADIUS);
  /* Erase the left half so the ring is broken. */
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols / 2; c++) {
      if (g_scene.activator_voltage[r][c] > ACTIVATOR_REST + 0.5f)
        g_scene.activator_voltage[r][c] = ACTIVATOR_REST;
    }
  }
}

/* Two columns at the left edge fire — produces a flat wavefront
 * that sweeps across the grid. */
static void preset_plane_wave(void) {
  grid_reset_to_rest();
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    g_scene.activator_voltage[r][0] = SUPRATHRESHOLD_KICK_VALUE;
    g_scene.activator_voltage[r][1] = SUPRATHRESHOLD_KICK_VALUE;
  }
}

static void preset_load(int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  g_scene.active_preset_index = preset_index;
  preset_table[preset_index].loader();
}

/* ===================================================================== */
/* §11  glyph_picker — activator value → glyph + colour (T9)             */
/* ===================================================================== */

/*
 * CellRender — one cell's drawing instruction, derived from membrane V.
 *
 * Intent
 *   The activator V is the "voltage" of the excitable medium.  Each
 *   cell's V is mapped to a glyph + colour pair + attribute combo
 *   so the renderer paints rising fronts (excited) bright and
 *   trailing recovery (refractory) dim.  Bundling all three outputs
 *   in a struct keeps the per-cell loop linear:
 *     `r = pick_cell(v); if (r.skip) continue;`
 *
 * Why a skip flag
 *   Cells with V near rest are LEFT BLANK so the terminal background
 *   shows through and wave fronts pop visually.  Skipping the
 *   `mvaddch` for those cells also reduces newscr/curscr diff
 *   bandwidth.
 *
 * Reference [7] Bourke for the V → glyph ramp design.
 */
typedef struct {
    chtype glyph;        /* '.' ':' '*' '#' or similar ramp glyph    */
    int    pair;         /* colour-pair index (theme ramp tier)      */
    attr_t extra_attr;   /* A_BOLD for hottest tier                  */
    bool   skip;         /* if true, leave the cell blank             */
} CellRender;

/* Construct a "skip this cell" render (leave terminal background
 * showing through).  Used for cells at rest — saves attron/mvaddch
 * and lets the field's RESTING REGIONS read as the bg colour. */
static inline CellRender cell_render_rest_tier(void) {
    return (CellRender){ .skip = true };
}

/* Recovery tier: '.' on the cool-colour pair.  Cells just relaxed
 * back from a firing event — they're below threshold but their
 * inhibitor v is still high (refractory period). */
static inline CellRender cell_render_recovery_tier(void) {
    return (CellRender){ .glyph = '.', .pair = PAIR_BAND_RECOVERY,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* Rising tier: ':' on an intermediate-colour pair.  The activator is
 * climbing above rest but hasn't reached the wavefront yet. */
static inline CellRender cell_render_rising_tier(void) {
    return (CellRender){ .glyph = ':', .pair = PAIR_BAND_RISING,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* Wave tier: '+' on a warm-colour pair.  The bulk of the propagating
 * wave; activator near its plateau value. */
static inline CellRender cell_render_wave_tier(void) {
    return (CellRender){ .glyph = '+', .pair = PAIR_BAND_WAVE,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* Front tier: '#' bold.  The leading edge of the wave — activator at
 * peak.  A_BOLD draws extra eye attention to where the action is. */
static inline CellRender cell_render_front_tier(void) {
    return (CellRender){ .glyph = '#', .pair = PAIR_BAND_FRONT,
                         .extra_attr = A_BOLD,   .skip = false };
}

/*
 * pick_cell_render — activator value → glyph + colour + attribute.
 *
 * Pseudocode (cascade-by-threshold):
 *   if u < U_BAND_REST_HIGH      : rest tier      (skip — bg shows)
 *   if u < U_BAND_RECOVERY_HIGH  : recovery tier  ('.')
 *   if u < U_BAND_RISING_HIGH    : rising tier    (':')
 *   if u < U_BAND_WAVE_HIGH      : wave tier      ('+')
 *   else                         : front tier     ('#' bold)
 *
 * Each tier corresponds to one phase of the FN action-potential
 * cycle.  Refs [1] FitzHugh 1961 fig. 2 for the phase-portrait
 * trajectory these bands sample.
 */
static CellRender pick_cell_render(float u_value) {
    if (u_value < U_BAND_REST_HIGH    ) return cell_render_rest_tier();
    if (u_value < U_BAND_RECOVERY_HIGH) return cell_render_recovery_tier();
    if (u_value < U_BAND_RISING_HIGH  ) return cell_render_rising_tier();
    if (u_value < U_BAND_WAVE_HIGH    ) return cell_render_wave_tier();
    return                                     cell_render_front_tier();
}

/* ===================================================================== */
/* §12  render — paint the activator field                               */
/* ===================================================================== */
/*
 * Walks the grid, picks a glyph + colour per cell from u, and
 * paints.  Skips resting cells (transparent — saves attron/attroff
 * thrash AND erase cost in the main loop).  Reserves rows for HUD.
 */

/* Compute the visible drawing rectangle for the activator field,
 * given the terminal extent and the HUD-reserved rows.  Returns the
 * top-row offset plus per-axis cell counts, all clipped to BOTH the
 * terminal extent AND the active grid extent so the smaller of the
 * two wins.
 *
 *   draw_top              first display row used by the field
 *   *out_max_rows / cols  inclusive iteration bounds                */
static inline void compute_activator_draw_rect(int term_rows, int term_cols,
                                                int *out_draw_top,
                                                int *out_max_rows,
                                                int *out_max_cols) {
    int draw_top    = HUD_RESERVED_ROWS_TOP;
    int draw_bottom = term_rows - HUD_RESERVED_ROWS_BOTTOM;
    int avail_rows  = draw_bottom - draw_top;

    *out_draw_top = draw_top;
    *out_max_rows = (g_scene.grid_active_rows < avail_rows)
                  ?  g_scene.grid_active_rows
                  :  avail_rows;
    *out_max_cols = (g_scene.grid_active_cols < term_cols)
                  ?  g_scene.grid_active_cols
                  :  term_cols;
}

/* Paint ONE activator cell: pick the render tier, skip-or-draw.
 * Caller supplies the (already-offset) display row + column. */
static inline void paint_one_activator_cell(int display_row, int display_col,
                                             float u_value) {
    CellRender cr = pick_cell_render(u_value);
    if (cr.skip) return;
    attr_t a = COLOR_PAIR(cr.pair) | cr.extra_attr;
    attron(a);
    mvaddch(display_row, display_col, cr.glyph);
    attroff(a);
}

/*
 * render_activator_field — paint the activator U as a fading-front
 * heatmap.  Resting cells are LEFT BLANK (background shows through).
 *
 * Pseudocode:
 *   compute_activator_draw_rect(term, ..., &draw_top, &max_r, &max_c)
 *   for each cell (r, c) in [0..max_r) × [0..max_c):
 *     paint_one_activator_cell(draw_top + r, c, U[r][c])
 *
 * Refs [7] Bourke for the V → glyph ramp (here U, but same idea).
 */
static void render_activator_field(int term_rows, int term_cols) {
    int draw_top, max_rows, max_cols;
    compute_activator_draw_rect(term_rows, term_cols,
                                 &draw_top, &max_rows, &max_cols);

    for (int r = 0; r < max_rows; r++) {
        for (int c = 0; c < max_cols; c++) {
            paint_one_activator_cell(draw_top + r, c,
                                      g_scene.activator_voltage[r][c]);
        }
    }
}

/* ===================================================================== */
/* §13  scene — paused / preset / speed state + tick                     */
/* ===================================================================== */
/*
 * Convenience wrappers around the global state declared in §4.
 * Keep the per-frame logic small and named.
 */

static void scene_init(int term_rows, int term_cols) {
  g_scene.grid_active_rows = (term_rows < GRID_ROWS_MAX) ? term_rows : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
  g_scene.simulation_paused = false;
  g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_DEFAULT;
  preset_load(0);
}

static void scene_resize(int term_rows, int term_cols) {
  g_scene.grid_active_rows = (term_rows < GRID_ROWS_MAX) ? term_rows : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
  preset_load(g_scene.active_preset_index);
}

static void scene_tick(void) {
  if (g_scene.simulation_paused)
    return;
  for (int s = 0; s < g_scene.substeps_per_frame; s++)
    reaction_step();
}

/* ===================================================================== */
/* §14  hud — top status + bottom hint (CLAUDE.md spec)                  */
/* ===================================================================== */
/*
 * Two HUD elements:
 *   row 0       — status (yellow + bold), top-right
 *   row rows-1  — key hint (cyan + bold),  bottom
 */

static void hud_paint_status(int term_cols, double measured_fps) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " FitzHugh-Nagumo  preset:%s  speed:%dx  "
           "a=%.2f b=%.2f e=%.2f D=%.2f dt=%.3f  %5.1f fps  %s ",
           preset_table[g_scene.active_preset_index].display_name, g_scene.substeps_per_frame,
           (double)FN_THRESHOLD_SHIFT, (double)FN_INHIBITOR_FEEDBACK,
           (double)FN_TIMESCALE_RATIO, (double)ACTIVATOR_DIFFUSION,
           (double)EULER_DT, measured_fps,
           g_scene.simulation_paused ? "PAUSED " : "running");
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
           " q:quit  spc:impulse  p:pause  r:reset  "
           "1:rings  2:double  3:spiral  4:plane  +/-:speed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §15  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we
 * keep only cell dimensions for HUD placement and field clipping.
 *
 * Render pipeline (one frame): erase → paint_field → hud_paint_*
 *   → wnoutrefresh(stdscr) → doupdate().  Diff-only writes — no flicker.
 *   See [8] Raymond §11.
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
  render_activator_field(screen->rows, screen->cols);
  hud_paint_status(screen->cols, measured_fps);
  hud_paint_hint(screen->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §16  app — main loop + signals + input                                */
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

  case 'p':
  case 'P':
    g_scene.simulation_paused = !g_scene.simulation_paused;
    break;

  case ' ':
    inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                            IMPULSE_RADIUS_CELLS);
    break;

  case 'r':
  case 'R':
    grid_reset_to_rest();
    break;

  case '1':
    preset_load(0);
    break;
  case '2':
    preset_load(1);
    break;
  case '3':
    preset_load(2);
    break;
  case '4':
    preset_load(3);
    break;

  case '+':
  case '=':
    g_scene.substeps_per_frame++;
    if (g_scene.substeps_per_frame > SUBSTEPS_PER_FRAME_MAX)
      g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_MAX;
    break;
  case '-':
    g_scene.substeps_per_frame--;
    if (g_scene.substeps_per_frame < SUBSTEPS_PER_FRAME_MIN)
      g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
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
    int ch = getch();
    if (ch != ERR && !app_handle_key(ch)) {
      g_should_quit = 1;
      break;
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
    if (fps_window_ns >= 500 * NS_PER_MS) {
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
