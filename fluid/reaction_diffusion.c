/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion.c — Gray-Scott Turing patterns on a terminal grid
 *
 * DEMO: Two chemicals U and V on a grid.  U feeds in, reacts with V to
 *       make more V, V dies off.  V also diffuses (slowly) and U
 *       diffuses (faster).  That asymmetry — fast diffuser feeds the
 *       slow diffuser via a non-linear reaction — is enough to make
 *       the chemicals SPONTANEOUSLY ORGANISE into spots, stripes,
 *       coral, mazes, drifting solitons.  Alan Turing predicted this
 *       in 1952.  Pearson explored the parameter space in 1993 and
 *       found the menu of patterns we cycle through here.
 *
 *       Seven presets.  Each is a different (f, k) parameter pair —
 *       SMALL SHIFTS in the parameters produce RADICALLY different
 *       morphologies.  Mitosis splits.  Coral branches.  Stripes
 *       wander.  Worms tunnel.  Mazes settle.  Bubbles tile.
 *       Solitons drift forever.
 *
 *       Per-tick update for each cell:
 *
 *           dU/dt = Du · ∇²U  −  U·V²  +  f·(1 − U)
 *           dV/dt = Dv · ∇²V  +  U·V²  −  (f + k)·V
 *
 *       Six terms.  Two diffusion (Laplacian).  One shared autocatalytic
 *       reaction (U·V²) that consumes U and produces V.  One feed (f
 *       replenishes U).  One kill (k drains V).  Forward Euler, fixed
 *       dt, periodic boundaries, double-buffered to avoid aliasing.
 *
 *       The Laplacian is the 9-point ISOTROPIC stencil: four cardinal
 *       neighbours weighted 0.20, four diagonals weighted 0.05, centre
 *       −1.  This is rounder than the simple 5-point stencil (which
 *       leaves visible diamond artefacts in spot patterns).
 *
 * Study alongside:
 *   fluid/cfl_stability_explorer.c  — explores the stability bound
 *                                      that limits dt for explicit
 *                                      schemes.  This file lives just
 *                                      under the Gray-Scott CFL bound.
 *   procedural/diffusion/heat_diffusion.c
 *                                    — heat equation = pure diffusion,
 *                                      no reaction.  No patterns form.
 *                                      The reaction is what does it.
 *   fluid/navier_stokes.c            — also uses the discrete Laplacian
 *                                      (in the projection's Poisson
 *                                      solve).  Same numerical engine,
 *                                      different physics.
 *   procedural/cellular_automata/    — discrete-state spatial systems.
 *                                      Reaction-diffusion is the
 *                                      continuous-state cousin.
 *
 * Section map:
 *   §1   config            — every tunable named, no later magic
 *   §2   clock             — monotonic ns timer + sleep
 *   §3   rng               — small wrapper around stdlib rand
 *   §4   ramp              — ASCII glyph + density thresholds
 *   §5   themes            — 4 colour palettes
 *   §6   colors            — pair init + theme apply
 *   §7   presets           — Gray-Scott (f, k) parameter table
 *   §8   grid_buffers      — alloc / free / resize the four arrays
 *   §9   laplacian         — the 9-point isotropic stencil
 *   §10  reaction_step     — one Euler tick of the PDE
 *   §11  seed              — initial conditions + drop-blob helper
 *   §12  warmup            — fast-forward steps before first frame
 *   §13  glyph_picker      — V → ramp slot + glyph + attribute
 *   §14  render_grid       — paint the V field on the terminal
 *   §15  scene             — per-frame state + tick + helpers
 *   §16  hud               — top status + bottom hint (CLAUDE.md spec)
 *   §17  screen            — ncurses init / cleanup / present
 *   §18  app               — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   n / p            next / prev preset
 *                      (Mitosis / Coral / Stripes / Worms / Maze /
 *                       Bubbles / Solitons)
 *   t                next colour theme (ocean / forest / magma / violet)
 *   r                reseed (keeps preset; new initial blobs)
 *   s                drop a seed blob at screen centre
 *   + / -            sim steps per render frame +/-
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_diffusion.c \
 *       -o reaction_diffusion -lncurses
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Tutorials build a ladder: T1 names the algorithm, T2 explains
 *      why two diffusers make patterns, T3-T4 introduce the equations
 *      and parameter space, T5-T8 cover numerics, T9-T10 the
 *      visualisation.
 *   2. §1 config — every constant in one place, with units and
 *      stability-bound comments.
 *   3. §10 reaction_step — the algorithm in twenty lines.  Read this
 *      AFTER tutorials T3, T5, T6.
 *   4. §11 seed → §12 warmup → §15 scene → §18 app — orchestration.
 *   5. §13 glyph_picker + §14 render_grid — the visualisation layer.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   substrate_u[]            U field flat array, row-major
 *   catalyst_v[]             V field flat array
 *   substrate_u_next[]       scratch for the new U after one step
 *   catalyst_v_next[]        scratch for the new V
 *
 *   feed_rate, kill_rate     f, k from current preset
 *   diffusion_u, diffusion_v Du, Dv (named constants)
 *
 *   laplacian_u, laplacian_v ∇²U, ∇²V at current cell
 *   reaction_term            U·V² (the autocatalytic term, shared)
 *
 *   active_preset_index      which row of preset_table is live
 *   active_theme_index       which row of theme_table is live
 *   simulation_steps_per_frame  how many ticks between renders
 *   simulation_paused        run/pause toggle
 *
 *   GRID_TOROIDAL_INDEX(i, n)   periodic-wrap helper
 *
 * Background you need
 * ───────────────────
 *   - Partial differential equations (∂u/∂t = ...) at a hand-wavy
 *     level: "rate of change of u depends on u's value and its
 *     spatial derivatives."  T3 introduces the equations slowly.
 *   - Discrete Laplacian on a grid (∇²u ≈ neighbour sum minus
 *     centre).  T5 walks through it.
 *   - Forward Euler integration (u_new = u_old + dt · du/dt).  T6
 *     covers stability; you don't need prior numerics background.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Stochastic PDEs, finite-element methods, multigrid solvers.
 *     This is the simplest possible numerical setup.
 *   - Real chemistry.  U and V are abstract chemicals; "feed" and
 *     "kill" are reaction-rate parameters, not real-world rates.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm    : Forward Euler integration of the GRAY-SCOTT REACTION-
 *                DIFFUSION SYSTEM, a two-chemical PDE that produces
 *                Turing patterns.  Each cell of a 2-D grid stores two
 *                concentrations (U, V).  At each tick, every cell
 *                advances by one time-step using the local PDE rates
 *                computed from the cell's neighbourhood.
 *
 *                Per cell, per tick (forward Euler):
 *                  ∇²u, ∇²v   ← isotropic 9-point Laplacians
 *                  reaction   ← U · V²  (autocatalytic)
 *                  u_new      ← u + dt · (Du·∇²u − reaction + f·(1−u))
 *                  v_new      ← v + dt · (Dv·∇²v + reaction − (f+k)·v)
 *
 *                Boundaries are PERIODIC (toroidal wrap-around).  The
 *                grid is the entire visible terminal area minus 1
 *                row top + 1 row bottom for the HUD.
 *
 * Math basis   : Turing (1952) showed that a system of TWO REACTING
 *                CHEMICALS with DIFFERENT DIFFUSION RATES can amplify
 *                small initial perturbations into stable spatial
 *                patterns.  The KEY ASYMMETRY: the inhibitor (here U)
 *                must diffuse FASTER than the activator (V).  Without
 *                that, both chemicals smooth out together and you
 *                get a uniform field.  With it, V "wins" locally
 *                while U is depleted in a wider neighbourhood —
 *                "long-range inhibition, short-range activation."
 *
 *                Gray-Scott (1984) is one specific reaction system in
 *                this class:
 *                  U + 2V → 3V       (autocatalytic, the U·V² term)
 *                  V → P             (V decays, the (f+k)·V term)
 *                  feed of U at rate f·(1 − U)
 *                Pearson (1993) mapped its parameter space (f, k)
 *                showing 12+ qualitatively distinct pattern regimes.
 *
 * Performance  : Two flat float arrays per chemical (current + scratch).
 *                One pair per chemical, swapped each step (O(1)
 *                pointer swap).  Per-cell work: ~16 multiplies + 16
 *                adds for the two Laplacians + reaction.  At 80×24
 *                = 1920 cells × 16 steps/frame × 30 fps ≈ 1M cell
 *                updates/sec.  Trivial on any CPU.
 *
 * References
 * ──────────
 *   Turing, A. M. (1952), "The Chemical Basis of Morphogenesis,"
 *     Phil. Trans. R. Soc. B 237 (641): 37-72.  THE foundational
 *     paper on reaction-diffusion patterns in biology.
 *   Pearson, J. E. (1993), "Complex Patterns in a Simple System,"
 *     Science 261 (5118): 189-192.  The parameter-space map this
 *     file's preset table is taken from.
 *   Gray, P. & Scott, S. K. (1984), "Autocatalytic Reactions in the
 *     Isothermal CSTR: Oscillations and Instabilities," Chem. Eng.
 *     Sci. 39 (6): 1087-1097.  The chemistry behind Gray-Scott.
 *   Karl Sims, "Reaction-Diffusion Tutorial":
 *     https://www.karlsims.com/rd.html
 *   Munafo, R., "Stable Localized Moving Patterns in the 2-D
 *     Gray-Scott Model": https://mrob.com/pub/comp/xmorphia/
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two chemicals on a grid.  One spreads fast and gets replenished
 * from a uniform source.  The other spreads slow and only grows
 * where the fast one has already been consumed.  At every tick,
 * each cell's amounts are updated from its neighbours and itself
 * via simple ARITHMETIC — no looking at distant cells, no global
 * state.  Yet the grid as a whole self-organises into spots,
 * stripes, mazes, drifting blobs.  This is "MORPHOGENESIS FROM
 * LOCAL RULES" — exactly the framework Turing proposed in 1952 to
 * explain how zebras get stripes and leopards get spots from a
 * single fertilised egg.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a flat sandwich tray covered with a thin layer of milk
 * (U).  Drop a few small puddles of food colouring (V) onto it.
 * The food colouring spreads slowly (V's small diffusion).  Where
 * food colouring meets milk, more food colouring is created (the
 * U·V² reaction).  Meanwhile, fresh milk is poured in evenly
 * across the tray (the f·(1−U) feed term) and food colouring
 * gradually dries up (the (f+k)·V kill term).
 *
 * What you'd expect: the colouring would just spread and dilute.
 * What actually happens: depending on how fast you pour milk and
 * dry colouring, the colouring SETTLES INTO PATTERNS — separate
 * blobs, parallel stripes, branching coral, drifting droplets.
 * Same equation; different (f, k) values give different shapes.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  PHASE SPACE (Pearson 1993):                         │
 *      │                                                      │
 *      │    k                                                 │
 *      │  0.07 ┤    ╭── Coral ──╮                             │
 *      │       │   /     |      \                             │
 *      │  0.06 ┤  / Maze  Stripes  Worms ─╮                    │
 *      │       │ / Mitosis        Bubbles \                    │
 *      │  0.05 ┤ ╰── Solitons ───────────╯                     │
 *      │       │                                              │
 *      │       └──────────────────────────────                 │
 *      │       0.02   0.04   0.06   0.08   0.10  f             │
 *      │                                                      │
 *      │  Each preset is one (f, k) point in this map.        │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   GRAY-SCOTT REACTION-DIFFUSION SYSTEM:
 *     ∂U/∂t = Du · ∇²U − U·V² + f · (1 − U)         (substrate)
 *     ∂V/∂t = Dv · ∇²V + U·V² − (f + k) · V         (catalyst)
 *
 *   FORWARD EULER UPDATE (per cell, per tick):
 *     u_new = u + dt · (Du · ∇²u − u·v² + f · (1 − u))
 *     v_new = v + dt · (Dv · ∇²v + u·v² − (f + k) · v)
 *
 *   9-POINT ISOTROPIC LAPLACIAN:
 *     ∇²x[i, j] = 0.20 · (N + S + E + W)
 *               + 0.05 · (NE + NW + SE + SW)
 *               − x[i, j]
 *
 *   PERIODIC BOUNDARY (toroidal wrap):
 *     x_left   = (x == 0)        ? cols - 1 : x - 1
 *     x_right  = (x == cols - 1) ? 0        : x + 1
 *     y_above  = (y == 0)        ? rows - 1 : y - 1
 *     y_below  = (y == rows - 1) ? 0        : y + 1
 *
 *   CFL STABILITY BOUND (T6):
 *     dt · max(Du, Dv) / dx² < 0.25
 *     With dx = 1 and Du = 0.20:  max dt = 1.25.  We use dt = 1.0.
 *
 *   V → GLYPH MAPPING (T9):
 *     v_scaled  = clamp(v · DISPLAY_SCALE, 0, 1)
 *     slot      = highest threshold v_scaled meets in the ramp table
 *     glyph     = " .:-+*#@"[slot]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - DOUBLE BUFFERING is mandatory.  If you write u[i] before
 *     reading u[i+1], the next cell's update sees the NEW value
 *     instead of the old one.  This silently breaks the algorithm —
 *     you get a "Gauss-Seidel-style" relaxation that looks similar
 *     for a while but slowly drifts wrong.
 *   - INITIAL CONDITIONS matter.  At t=0 most of the grid is U=1,
 *     V=0 (boring uniform).  We seed a few square blobs of V=1 and
 *     let the reaction propagate from there.  Without seeds, the
 *     uniform state is stable forever.
 *   - WARMUP TICKS hide the initial-condition dependence.  At
 *     t < ~30 tick-batches you can see the seed blobs growing; we
 *     run 600 ticks (~30 frames at 16 steps/frame) BEFORE the first
 *     render so the user sees a developed pattern.
 *   - CONTRAST STRETCHING.  Peak V in the self-organised state is
 *     typically 0.3-0.5, not 1.0.  Without scaling, the renderer
 *     would only ever use the dim end of the ramp.  V_DISPLAY_SCALE
 *     = 2.2 maps the typical range to ~[0.66, 1.1] (clamped to 1.0).
 *   - PRESET (f, k) SHIFTS are TINY.  Stripes (0.0600, 0.0620) and
 *     Worms (0.0620, 0.0610) differ in the third decimal yet
 *     produce visibly different morphologies.  The Gray-Scott
 *     bifurcation diagram is densely packed.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Press through every preset with 'n':
 *       Mitosis    spots that periodically divide.
 *       Coral      branching, dendritic fronts.
 *       Stripes    long winding labyrinthine bands.
 *       Worms      shorter bands, more irregular.
 *       Maze       fine-grained, dense maze texture.
 *       Bubbles    stable round bubble lattice.
 *       Solitons   isolated drifting blobs that DON'T divide.
 *   - Press 'r' to reseed: the same preset replays with different
 *     initial blob positions.  Long-time pattern is statistically
 *     similar; transient evolution differs.
 *   - Press 's' to drop a seed at centre: a new V blob appears,
 *     interacts with existing pattern, propagates outward.
 *   - Press 't' to cycle themes: pattern stays IDENTICAL; only the
 *     colour ramp changes.  Geometry is preserved.
 *   - Press '+' / '-' to change steps/frame: pattern evolves
 *     faster / slower.  Doubling steps = doubling visual time-lapse.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1   What is reaction-diffusion?  Turing morphogenesis
 *   T2   Activator-inhibitor — why Du > Dv matters
 *   T3   The Gray-Scott equations, term by term
 *   T4   Phase space — small (f, k) shifts, big visual change
 *   T5   Discrete Laplacian — 5-point vs 9-point isotropic
 *   T6   Forward Euler integration — and the CFL bound
 *   T7   Double buffering — read/write aliasing avoidance
 *   T8   Periodic boundaries — toroidal wrap
 *   T9   Visualisation — V → glyph + colour with contrast scaling
 *   T10  Initial conditions — seeds, warmup, determinism
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS REACTION-DIFFUSION?  TURING MORPHOGENESIS
 * ─────────────────────────────────────────────────────
 * In 1952, Alan Turing asked a simple question: how does a fertilised
 * egg, starting as a uniform sphere of cells, develop into an
 * organism with COMPLEX SPATIAL PATTERNS — zebra stripes, leopard
 * spots, fish tessellations, plant phyllotaxis?
 *
 * His answer: chemicals.  Two chemicals, actually — an ACTIVATOR
 * and an INHIBITOR — both diffusing through the embryo at different
 * rates and reacting with each other.  Under specific conditions
 * (T2), tiny random fluctuations get amplified by the reaction
 * dynamics and STABILISED by the diffusion ratio, producing
 * permanent stripes / spots / whorls.
 *
 * Turing showed this with pure math, not experiment.  It took 40
 * years for chemists to find real chemical systems that exhibited
 * the patterns (BZ reaction, Gray-Scott, Brusselator).  Today
 * "reaction-diffusion" is a unifying framework for emergent
 * pattern formation in:
 *
 *   - Embryology (Turing's original target)
 *   - Animal coat patterns (Murray 1981)
 *   - Slime mold growth (Physarum polycephalum)
 *   - Forest fires, urban sprawl, weather systems
 *   - Visual art (Karl Sims, computer graphics)
 *
 * The recipe is always the same:
 *   1. Two species (or more) on a spatial domain.
 *   2. Each species DIFFUSES at its own rate.
 *   3. Local REACTION between species (some create others, some
 *      destroy them, the relationships are non-linear).
 *   4. With the right parameters, patterns emerge.  Without them,
 *      the system relaxes to uniform.
 *
 * Gray-Scott (this file) is the simplest reaction-diffusion system
 * that produces the FULL Turing pattern menagerie.
 *
 * T2  ACTIVATOR-INHIBITOR — WHY DU > DV MATTERS
 * ─────────────────────────────────────────────
 * Why do TWO diffusion rates make patterns when ONE doesn't?
 *
 * Imagine a single chemical V that reacts to make more of itself
 * and diffuses.  Drop a blob of V on the grid.  The reaction
 * makes more V locally; the diffusion spreads V outward.  Both
 * effects amplify V everywhere.  Result: the blob grows, then
 * fills the grid uniformly.  No pattern.
 *
 * Now add a SECOND chemical U that V CONSUMES (the U·V² term).
 * The reaction needs U to make V.  Where V is high, U gets
 * depleted.  Where U is depleted, the reaction can't run, so V
 * STOPS GROWING in that location.
 *
 * If U's diffusion is FASTER than V's, U gets sucked in from a
 * wide neighbourhood — V locally exhausts a region of U's far
 * faster than U can diffuse back in.  The V blob STARVES ITSELF.
 *
 * But wait — U is also being REPLENISHED uniformly (the f·(1−U)
 * term).  So U eventually returns to background level.  At which
 * point V can grow again — but only NEXT TO existing V blobs (the
 * autocatalytic term needs some V to start).  The result: NEW V
 * BLOBS appear at a characteristic distance from old ones.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Without separation (Du = Dv):                       │
 *      │                                                      │
 *      │    ●   ●   ●   ●   ●   ●          uniform fill       │
 *      │                                                      │
 *      │  With separation (Du > Dv):                          │
 *      │                                                      │
 *      │     ●     ●     ●     ●           spots at fixed     │
 *      │         ●     ●     ●               distance         │
 *      │     ●     ●     ●     ●                              │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * The CHARACTERISTIC LENGTH SCALE of the pattern is set by the
 * ratio of diffusion rates and the reaction-rate parameters
 * (f, k).  This is why the same algorithm produces totally
 * different morphologies at different (f, k) — small changes
 * shift which length scale wins.
 *
 * T3  THE GRAY-SCOTT EQUATIONS, TERM BY TERM
 * ──────────────────────────────────────────
 * The full system is two coupled PDEs:
 *
 *     ∂U/∂t = Du · ∇²U  −  U·V²  +  f · (1 − U)
 *     ∂V/∂t = Dv · ∇²V  +  U·V²  −  (f + k) · V
 *
 * Six terms.  Three per equation.  Each has a physical meaning.
 *
 *     Du · ∇²U     DIFFUSION OF U.  Du is the diffusion coefficient.
 *                  ∇²U (Laplacian) is the local "curvature" of U —
 *                  how much U at this cell differs from its
 *                  neighbours' average.  Positive curvature (cell
 *                  is a valley) → U flows IN.  Negative (cell is
 *                  a peak) → U flows OUT.  Net: U smooths over
 *                  time at rate Du.
 *
 *     − U·V²       CONSUMPTION OF U.  Whenever U meets two V's,
 *                  one U gets consumed and one V is created (the
 *                  reaction is U + 2V → 3V).  The rate is
 *                  proportional to U times V², making this an
 *                  AUTOCATALYTIC second-order reaction in V.
 *
 *     + f · (1 − U) FEED-IN.  U is replenished uniformly across
 *                  the grid at rate f, scaled by how depleted U
 *                  is locally (the (1 − U) factor).  When U = 0,
 *                  feed is fastest; when U = 1, feed shuts off.
 *                  Models a chemostat.
 *
 *     Dv · ∇²V     DIFFUSION OF V.  Same as Du but slower —
 *                  Dv < Du.  This asymmetry is what makes patterns
 *                  possible (T2).
 *
 *     + U·V²       PRODUCTION OF V.  Mirror of U's consumption
 *                  term — same magnitude, opposite sign.
 *
 *     − (f + k)·V  REMOVAL OF V.  Two effects combined:
 *                  - (f · V) — V is washed out by the same flow
 *                    that brings U in (chemostat dilution).
 *                  - (k · V) — V additionally decays into product
 *                    P at rate k (V → P).
 *                  Combined, V decays at rate (f + k).
 *
 * That's Gray-Scott.  Six terms, four parameters (Du, Dv, f, k).
 * We hold Du and Dv fixed at 0.20 / 0.10 and let the user vary
 * (f, k) via presets — that two-parameter slice is where all the
 * famous Turing patterns live.
 *
 * T4  PHASE SPACE — SMALL (f, k) SHIFTS, BIG VISUAL CHANGE
 * ────────────────────────────────────────────────────────
 * Pearson (1993) ran Gray-Scott across a fine (f, k) grid and
 * mapped where each pattern type appears.  The shocker: the map
 * is FRACTALLY DETAILED.  Small parameter changes — changes in
 * the third decimal — cross PHASE BOUNDARIES and switch between:
 *
 *     SPOTS       isolated round dots, sometimes dividing
 *     STRIPES     parallel bands that bend and weave
 *     LABYRINTHS  dense maze-like texture, no straight runs
 *     CORAL       branching tree-like growth from seeds
 *     SOLITONS    isolated stable blobs that drift but don't
 *                 divide or grow — discrete particles emerging
 *                 from a continuous PDE
 *     CHAOS       constantly evolving non-repeating mess
 *     EXTINCTION  V dies out everywhere — uniform U=1 final state
 *
 * Our seven presets sample these regimes:
 *
 *     Preset       (f,      k)       Regime
 *     ─────────   ─────────────────  ──────────────────────────
 *     Mitosis     (0.0367, 0.0649)  spots that divide
 *     Coral       (0.0545, 0.0630)  branching coral fronts
 *     Stripes     (0.0600, 0.0620)  labyrinthine stripes
 *     Worms       (0.0620, 0.0610)  short irregular bands
 *     Maze        (0.0290, 0.0570)  fine-grained maze
 *     Bubbles     (0.0940, 0.0590)  stable bubble lattice
 *     Solitons    (0.0250, 0.0500)  drifting indivisible blobs
 *
 * Look at Stripes vs Worms: f differs by 0.0020, k by 0.0010.
 * That tiny shift moves you from "labyrinthine indefinitely-long
 * stripes" to "short tangled worms."  The phase boundary is
 * SHARPER than any other parameter sensitivity in numerical
 * physics — closer to a phase transition in a real material than
 * to a "tweakable knob."
 *
 * T5  DISCRETE LAPLACIAN — 5-POINT VS 9-POINT ISOTROPIC
 * ─────────────────────────────────────────────────────
 * The Laplacian ∇²x at cell (i, j) needs to be approximated from
 * neighbour values.  Two common stencils:
 *
 * 5-POINT (cardinal only):
 *
 *               1
 *           1  −4  1
 *               1
 *
 *   ∇²x ≈ x[N] + x[S] + x[E] + x[W] − 4·x[centre]
 *
 * Simple, cheap, but ANISOTROPIC.  It treats horizontal/vertical
 * differently from diagonals.  Visible artefact in Gray-Scott
 * spots: they grow as DIAMONDS, not circles, because the diagonal
 * neighbours don't contribute to diffusion.
 *
 * 9-POINT ISOTROPIC (Fornberg):
 *
 *           0.05  0.20  0.05
 *           0.20  −1.00  0.20
 *           0.05  0.20  0.05
 *
 *   ∇²x ≈ 0.20·(N + S + E + W)
 *       + 0.05·(NE + NW + SE + SW)
 *       − 1.00·x[centre]
 *
 * Cardinal weight 0.20, diagonal weight 0.05 (1/4 of cardinal).
 * The 4:1 ratio gives SECOND-ORDER ISOTROPY — the leading-order
 * error term is RADIALLY SYMMETRIC, not direction-dependent.
 * Result: spots are round, stripes don't kink at 45°, mazes look
 * organic instead of grid-aligned.
 *
 * The weights sum to zero over a uniform field — the Laplacian
 * vanishes, as expected.
 *
 * Cost: 8 neighbour reads per cell instead of 4.  Doubles the
 * memory traffic but doubles the visual quality.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  5-point (anisotropic)        9-point (isotropic)    │
 *      │                                                      │
 *      │      ╳  diamond                  ●  round            │
 *      │     ╳ ╳  spots                ●  ●  ●  spots          │
 *      │      ╳                            ●                  │
 *      │                                                      │
 *      │  Stripe kinks at 45°          Stripe smoothly bends   │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T6  FORWARD EULER INTEGRATION — AND THE CFL BOUND
 * ─────────────────────────────────────────────────
 * Forward Euler is the simplest possible time-integrator:
 *
 *     u_new = u_old + dt · (du/dt)
 *
 * "Take the rate of change at the current state, multiply by dt,
 * add to current value."  That's it.
 *
 * For diffusion equations, forward Euler is CONDITIONALLY STABLE.
 * The COURANT-FRIEDRICHS-LEWY (CFL) bound says:
 *
 *     dt · D / dx² < 0.25                  (in 2-D)
 *
 * With our values:
 *   D       = max(Du, Dv) = 0.20
 *   dx      = 1 (one grid cell)
 *   max dt  = 0.25 / 0.20 = 1.25
 *
 * We use dt = 1.0, which is 80% of the bound.  Comfortable
 * margin.  Push dt above 1.25 and the simulation oscillates,
 * grows unboundedly, and eventually NaN's.
 *
 * Why this bound?  Intuitively: in one tick, diffusion shouldn't
 * be able to "fully exchange" a cell's value with its neighbours.
 * If dt is too big, the explicit scheme over-corrects and
 * oscillates.  Implicit schemes (like Stam's stable fluids,
 * fluid/navier_stokes.c) sidestep CFL but require solving a
 * linear system each step.  We trade simplicity for the bound.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  dt = 0.5    │   dt = 1.0       │   dt = 1.5         │
 *      │              │                  │                    │
 *      │  smooth      │   safe operating │   OSCILLATES,      │
 *      │  but slow    │   point          │   eventually NaN   │
 *      │                                                      │
 *      │  dt · D / dx² = 0.10            │   = 0.30 > 0.25 ✗  │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * The reaction terms (U·V², feed, kill) don't impose an
 * additional bound at our parameter values — they're well
 * within the bounded regime where U, V ∈ [0, 1].
 *
 * T7  DOUBLE BUFFERING — READ/WRITE ALIASING AVOIDANCE
 * ────────────────────────────────────────────────────
 * Each tick computes the new value of every cell.  But the new
 * value at cell (i, j) depends on the OLD values at (i, j) and
 * its neighbours.  If you write the new value back to the same
 * array, the next cell's update sees the NEW value of (i, j)
 * instead of the old.
 *
 * BUG (in-place update):
 *     for each cell:
 *         u[i, j] = u[i, j] + dt · ...    // overwrites!
 *     // next cell reads u[i-1, j] = NEW value, not old
 *
 * FIX (double buffer):
 *     for each cell:
 *         u_new[i, j] = u[i, j] + dt · ...   // write to scratch
 *     swap(u, u_new)                         // O(1) pointer swap
 *
 * This is sometimes called "Jacobi-style" iteration (vs in-place
 * "Gauss-Seidel"-style).  For Gray-Scott, double buffering is
 * REQUIRED — in-place updates introduce a subtle order-dependent
 * bias that breaks the Turing instability over long simulations.
 *
 * Cost: one extra array per chemical.  Worth it.
 *
 * Implementation: each grid has u[], v[] (current) and u_next[],
 * v_next[] (scratch).  After a tick, we SWAP POINTERS — no copy
 * needed:
 *
 *     temp        = grid->u;
 *     grid->u     = grid->u_next;
 *     grid->u_next = temp;
 *
 * O(1) per swap regardless of grid size.
 *
 * T8  PERIODIC BOUNDARIES — TOROIDAL WRAP
 * ───────────────────────────────────────
 * The grid is finite, but the algorithm's neighbour reads need to
 * work everywhere — including at the edges.  Three common choices:
 *
 *   FIXED       (Dirichlet) — clamp boundary cells to a fixed value
 *                              (e.g. U=1, V=0).  Patterns die at
 *                              edges.  Looks unnatural in a small
 *                              terminal.
 *
 *   ZERO-GRADIENT (Neumann) — boundary cells mirror their
 *                              interior neighbours.  Patterns
 *                              "see" themselves reflected; visible
 *                              symmetry artefacts.
 *
 *   PERIODIC    (toroidal)   — left edge connects to right, top to
 *                              bottom.  Grid is a TORUS.  Patterns
 *                              wrap continuously across edges; no
 *                              visible boundary.
 *
 * We use periodic.  Implementation: when reading a neighbour at
 * (x-1) and x = 0, use cols-1 instead.  When reading (x+1) and
 * x = cols-1, use 0.  The two helpers used in §10:
 *
 *     int x_left  = (x == 0)        ? cols - 1 : x - 1;
 *     int x_right = (x == cols - 1) ? 0        : x + 1;
 *
 * Branchless variant: x_left = (x + cols - 1) % cols.  We use the
 * if-form because it's clearer and the compiler turns both into
 * the same code on any modern architecture.
 *
 *      ┌──────────────────────────────────────────────────────┐
 *      │                                                      │
 *      │  Periodic = TOROIDAL TOPOLOGY:                       │
 *      │                                                      │
 *      │   ┌─────────────────┐                                │
 *      │   │                 │  ← wrap top  ──────╮            │
 *      │   │                 │                    │            │
 *      │   │                 │                    │            │
 *      │   │     pattern     │ ← wrap left/right ─┤            │
 *      │   │                 │                    │            │
 *      │   │                 │                    │            │
 *      │   │                 │  ← wrap bottom ────╯            │
 *      │   └─────────────────┘                                │
 *      │                                                      │
 *      │   Pattern flowing off the right edge re-appears on   │
 *      │   the left.  Useful side effect: spots near edges    │
 *      │   never feel "alone."                                │
 *      │                                                      │
 *      └──────────────────────────────────────────────────────┘
 *
 * T9  VISUALISATION — V → GLYPH + COLOUR WITH CONTRAST SCALING
 * ────────────────────────────────────────────────────────────
 * The terminal can show roughly 8 brightness levels per cell
 * (varying glyph density) and many colours.  We map V (the
 * pattern chemical) to BOTH:
 *
 *   GLYPH from V's "ink density":  ' ', '.', ':', '-', '+', '*', '#', '@'.
 *   COLOUR from the same V via a per-theme 8-step ramp.
 *
 * Both signals say the same thing — "how much V here?".  We could
 * decouple them (e.g. encode something else in colour), but for
 * a single-scalar simulation this redundancy reads more clearly.
 *
 * THE CONTRAST PROBLEM.  Pristine Gray-Scott V values range over
 * [0, 1] in theory.  In practice, the SELF-ORGANISED PATTERN
 * regime puts peak V around 0.3-0.5.  If we mapped V directly:
 *
 *     v ∈ [0, 0.3]  → glyph slots 0-2 only ('.,:').  Dim, washed
 *     v ∈ [0.3, 0.5] → slots 2-4 ('+').  Mid-range only.
 *
 * The screen would never use the bright end of the ramp.
 *
 * THE FIX: SCALE V UP before the ramp lookup:
 *
 *     v_display = v · DISPLAY_SCALE                  (= 2.2)
 *     v_display = clamp(v_display, 0, 1)
 *     slot      = ramp_lookup(v_display)
 *
 * Multiplying by 2.2 maps the typical [0.3, 0.5] range to
 * [0.66, 1.10] — clamped to [0.66, 1.0].  Now the bright '@', '#',
 * '*' glyphs see use.  Cells with V near zero stay invisible.
 *
 * Same trick appears in HDR tone mapping, audio compressors,
 * camera exposure auto-adjust — anywhere a small range of
 * "interesting" values lives inside a larger nominal range.
 *
 * T10 INITIAL CONDITIONS — SEEDS, WARMUP, DETERMINISM
 * ───────────────────────────────────────────────────
 * Gray-Scott has a UNIFORM TRIVIAL EQUILIBRIUM at U=1, V=0
 * everywhere.  At this state:
 *   ∂U/∂t = 0 - 1·0 + f·(1-1) = 0
 *   ∂V/∂t = 0 + 1·0 - (f+k)·0 = 0
 * Nothing moves.  The system stays uniform forever.
 *
 * To kick the simulation off, we INTRODUCE LOCAL PERTURBATIONS:
 * SEED BLOBS where V = 1 (some preset-dependent number of them).
 * From these, the activator-inhibitor dynamics (T2) propagate
 * outward and self-organise the global pattern.
 *
 * Seed shape: 7×7 squares of V=1, U=0.  Random positions
 * (seeded from time(NULL) so each run is different).  Number of
 * seeds varies by preset (3-8) — more seeds → faster pattern
 * coverage, slightly different transient evolution.
 *
 * WARMUP.  After seeding, the screen is mostly U=1, V=0 with a
 * few bright dots.  Boring.  We RUN 600 SIMULATION TICKS BEFORE
 * THE FIRST RENDER, so the user immediately sees a developed
 * pattern.  At 16 steps/frame and 30 fps, 600 ticks = ~1.25 s of
 * "fast-forward."  The user never sees this pre-roll.
 *
 * DETERMINISM.  Given the same RNG seed, the same preset, and
 * the same grid size, the simulation is EXACTLY DETERMINISTIC.
 * Press 'r' to reseed: a new RNG draw moves the seed blobs but
 * the algorithm is the same.  Long-time pattern morphology is
 * STATISTICALLY similar; transient evolution differs.
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

/* §1.1 — diffusion coefficients (T2). */

/* Du: rate at which U (substrate) diffuses.  Higher = U spreads
 * faster.  Gray-Scott requires Du > Dv for Turing instability;
 * equal diffusion gives a uniform steady state, no patterns. */
#define DIFFUSION_U                    0.20f

/* Dv: rate at which V (catalyst) diffuses.  Half of U's, by design.
 * The asymmetry is the engine of pattern formation. */
#define DIFFUSION_V                    0.10f

/* §1.2 — Euler timestep + CFL margin (T6). */

/* Forward Euler dt.  CFL bound: dt · max(Du, Dv) / dx² < 0.25.
 * With dx = 1 and max D = 0.20, max dt = 1.25.  We use 1.0 (80%
 * of the bound) for a comfortable safety margin. */
#define EULER_DT                       1.00f

/* §1.3 — simulation timing (T9). */

#define STEPS_PER_FRAME_DEFAULT        16
#define STEPS_PER_FRAME_MIN             1
#define STEPS_PER_FRAME_MAX            64
#define STEPS_PER_FRAME_STEP            4

/* §1.4 — warmup + seeding (T10). */

/* Ticks run BEFORE the first frame so the screen shows a developed
 * pattern instead of a few seed blobs in a uniform field. */
#define WARMUP_TICK_COUNT             600

/* Half-width of each square seed blob (cells).  3 → 7×7 blob. */
#define SEED_BLOB_HALF_WIDTH            3

/* §1.5 — visualisation (T9). */

/* Multiplier on V before the ramp lookup.  Peak V in the self-
 * organised regime is ~0.3-0.5; multiplying by 2.2 maps that to
 * ~0.66-1.1 (clamped to 1.0) so the bright end of the ramp sees use. */
#define CATALYST_DISPLAY_SCALE         2.2f

/* Number of glyph slots in the ramp (must match RAMP_GLYPHS / RAMP_THRESHOLDS). */
#define RAMP_SLOT_COUNT                  8

/* Theme rotation interval in render frames (~26 s at 30 fps). */
#define AUTO_THEME_CYCLE_FRAMES        800

/* §1.6 — colour pair IDs. */
enum {
    PAIR_RAMP_FIRST = 1,                                 /* +0..+7 */
    PAIR_HUD        = PAIR_RAMP_FIRST + RAMP_SLOT_COUNT, /* yellow */
    PAIR_HINT,                                           /* cyan   */
};

/* §1.7 — frame timing. */
#define NS_PER_SEC                  1000000000LL
#define NS_PER_MS                      1000000LL
#define RENDER_FPS_CAP                       30
#define RENDER_TICK_NS               (NS_PER_SEC / RENDER_FPS_CAP)
#define FPS_RECOMPUTE_MS                    500

/* §1.8 — number of presets and themes (filled below in §7 / §5). */
#define THEME_COUNT                    4

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
/* §3  rng — small wrapper around stdlib rand                            */
/* ===================================================================== */
/*
 * Used only for seed-blob positions (T10).  Physics is otherwise
 * deterministic.  Seeded from time(NULL) in main().
 */
static int rand_in_range(int lo, int hi_exclusive)
{
    if (hi_exclusive <= lo) return lo;
    return lo + rand() % (hi_exclusive - lo);
}

/* ===================================================================== */
/* §4  ramp — ASCII glyph + density thresholds                           */
/* ===================================================================== */
/*
 * 8 glyphs ordered lightest → darkest by visual ink density.  A cell
 * uses glyph i if its scaled-V falls in [threshold[i], threshold[i+1]).
 * Spacing is slightly non-uniform: more granularity in the typical
 * mid-range concentration values [0.3, 0.7] after scaling.
 */

static const char ramp_glyph_table[RAMP_SLOT_COUNT] = {
    ' ',   /* slot 0 — V ≈ 0 (pure U region, no catalyst) — not drawn */
    '.',   /* slot 1 — V barely present (reaction front edge)         */
    ':',   /* slot 2 — weakly active zone                              */
    '-',   /* slot 3 — transition into pattern interior                */
    '+',   /* slot 4 — pattern body                                    */
    '*',   /* slot 5 — dense catalyst concentration                    */
    '#',   /* slot 6 — near-peak V                                     */
    '@',   /* slot 7 — peak V (spot / stripe centres)                  */
};

static const float ramp_threshold_table[RAMP_SLOT_COUNT] = {
    0.00f, 0.10f, 0.24f, 0.38f, 0.52f, 0.65f, 0.78f, 0.90f,
};

/* glyph_slot_for — map a scaled V ∈ [0, 1] to the ramp slot index.
 * Returns the highest slot whose threshold v meets or exceeds. */
static int glyph_slot_for(float scaled_v)
{
    for (int i = RAMP_SLOT_COUNT - 1; i >= 0; i--)
        if (scaled_v >= ramp_threshold_table[i]) return i;
    return 0;
}

/* ===================================================================== */
/* §5  themes — 4 colour palettes                                        */
/* ===================================================================== */
/*
 * Each theme is a 256-colour 8-step ramp (slot 0 darkest → slot 7
 * brightest).  Slot 0 is technically a dim placeholder (matches the
 * space glyph that never paints), so its value isn't critical.  Slots
 * 1-7 are tuned to brighten progressively without skipping perceptual
 * steps.
 */

typedef struct {
    const char *display_name;
    short       fg256[RAMP_SLOT_COUNT];
} ColourTheme;

static const ColourTheme theme_table[THEME_COUNT] = {
    { "ocean",  { 232,  17,  19,  21,  27,  33,  51, 231 } },
    { "forest", { 232,  22,  28,  34,  40,  46, 118, 231 } },
    { "magma",  { 232,  52,  88, 124, 160, 196, 214, 231 } },
    { "violet", { 232,  54,  56,  93, 129, 165, 201, 231 } },
};

/* ===================================================================== */
/* §6  colors — pair init + theme apply                                  */
/* ===================================================================== */

static bool terminal_has_256_colours = false;

static void apply_theme(int theme_index)
{
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const ColourTheme *theme = &theme_table[theme_index];

    if (terminal_has_256_colours) {
        for (int i = 0; i < RAMP_SLOT_COUNT; i++)
            init_pair((short)(PAIR_RAMP_FIRST + i),
                      theme->fg256[i], -1);
    } else {
        /* 8-colour fallback — coarse but works on monochrome. */
        static const short FALLBACK[RAMP_SLOT_COUNT] = {
            COLOR_BLACK, COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,
            COLOR_CYAN,  COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
        };
        for (int i = 0; i < RAMP_SLOT_COUNT; i++)
            init_pair((short)(PAIR_RAMP_FIRST + i), FALLBACK[i], -1);
    }
}

static void colors_init(int theme_index)
{
    start_color();
    use_default_colors();
    terminal_has_256_colours = (COLORS >= 256);
    apply_theme(theme_index);

    /* HUD pairs per CLAUDE.md: bright + bold + default bg. */
    if (terminal_has_256_colours) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* attribute_for_slot — pair + optional A_BOLD on the brightest two
 * slots (extra punch on peaks).  A_BOLD is allowed on PAIR_RAMP
 * cells; the CLAUDE.md "no A_DIM" rule applies only to the HUD. */
static attr_t attribute_for_slot(int slot)
{
    attr_t a = COLOR_PAIR(PAIR_RAMP_FIRST + slot);
    if (slot >= RAMP_SLOT_COUNT - 2) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §7  presets — Gray-Scott (f, k) parameter table                       */
/* ===================================================================== */
/*
 * Pearson (1993) parameter values.  Tiny shifts in (f, k) cross
 * phase boundaries — Stripes (0.0600, 0.0620) and Worms (0.0620,
 * 0.0610) differ in the third decimal yet look entirely different.
 * See T4.
 */

typedef struct {
    const char *display_name;
    float       feed_rate;       /* f */
    float       kill_rate;       /* k */
    int         seed_blob_count; /* number of initial V blobs */
} GrayScottPreset;

static const GrayScottPreset preset_table[] = {
    { "Mitosis  ",  0.0367f, 0.0649f, 4 },  /* spots that periodically divide */
    { "Coral    ",  0.0545f, 0.0630f, 5 },  /* branching coral-like growth    */
    { "Stripes  ",  0.0600f, 0.0620f, 3 },  /* labyrinthine stripe patterns   */
    { "Worms    ",  0.0620f, 0.0610f, 6 },  /* winding worm tendrils          */
    { "Maze     ",  0.0290f, 0.0570f, 8 },  /* fine-grained maze texture      */
    { "Bubbles  ",  0.0940f, 0.0590f, 3 },  /* stable round bubble lattice    */
    { "Solitons ",  0.0250f, 0.0500f, 4 },  /* slowly drifting soliton blobs  */
};

#define PRESET_COUNT  ((int)(sizeof preset_table / sizeof preset_table[0]))

/* ===================================================================== */
/* §8  grid_buffers — alloc / free / resize the four arrays              */
/* ===================================================================== */
/*
 * The simulation state.  Two flat float arrays per chemical: current
 * (read) and next (written-to scratch).  Pointers swapped each tick
 * (T7) — O(1) regardless of grid size.
 *
 * Layout: row-major.  Index = row * cols + col.
 */

typedef struct {
    int    cols;
    int    rows;
    float *substrate_u;        /* current U field            */
    float *catalyst_v;         /* current V field            */
    float *substrate_u_next;   /* scratch for the new U      */
    float *catalyst_v_next;    /* scratch for the new V      */

    int    active_preset_index;
    int    active_theme_index;
    int    auto_theme_frame_counter;
} Grid;

static void grid_buffers_alloc(Grid *grid, int cols, int rows)
{
    size_t cell_count = (size_t)cols * (size_t)rows;
    grid->cols = cols;
    grid->rows = rows;
    grid->substrate_u      = malloc(cell_count * sizeof(float));
    grid->catalyst_v       = malloc(cell_count * sizeof(float));
    grid->substrate_u_next = malloc(cell_count * sizeof(float));
    grid->catalyst_v_next  = malloc(cell_count * sizeof(float));
}

static void grid_buffers_free(Grid *grid)
{
    free(grid->substrate_u);
    free(grid->catalyst_v);
    free(grid->substrate_u_next);
    free(grid->catalyst_v_next);
    memset(grid, 0, sizeof *grid);
}

static void grid_buffers_resize(Grid *grid, int cols, int rows)
{
    int saved_preset = grid->active_preset_index;
    int saved_theme  = grid->active_theme_index;
    grid_buffers_free(grid);
    grid_buffers_alloc(grid, cols, rows);
    grid->active_preset_index = saved_preset;
    grid->active_theme_index  = saved_theme;
}

/* ===================================================================== */
/* §9  laplacian — the 9-point isotropic stencil (T5)                    */
/* ===================================================================== */
/*
 * Discrete Laplacian on a 9-point stencil:
 *     ∇²x = 0.20 · (N + S + E + W) + 0.05 · (NE + NW + SE + SW) − x
 *
 * Implemented inline in §10 reaction_step for cache efficiency
 * (the Laplacian shares neighbour reads with the centre-cell read).
 * §9 has no separate function; the FORMULA lives here as
 * documentation.
 *
 * Cardinal weights 0.20, diagonal 0.05.  Sum of all weights including
 * centre is 0 — Laplacian of a constant field is zero.
 */

#define LAPLACIAN_CARDINAL_WEIGHT  0.20f
#define LAPLACIAN_DIAGONAL_WEIGHT  0.05f

/* ===================================================================== */
/* §10  reaction_step — one Euler tick of the PDE                        */
/* ===================================================================== */
/*
 * The heart of the simulator.  For every cell:
 *   1. Compute toroidal neighbour indices (T8).
 *   2. Compute 9-point Laplacian for U and V (T5).
 *   3. Compute the autocatalytic reaction term U·V² (shared).
 *   4. Apply forward Euler update (T6).
 *   5. Clamp to [0, 1] (Euler can overshoot bounds slightly).
 *   6. Write into the *_next scratch buffers.
 * Then SWAP buffer pointers (T7).
 */

static void reaction_step(Grid *grid)
{
    int cols = grid->cols;
    int rows = grid->rows;
    float feed_rate = preset_table[grid->active_preset_index].feed_rate;
    float kill_rate = preset_table[grid->active_preset_index].kill_rate;

    for (int y = 0; y < rows; y++) {
        /* Toroidal vertical neighbours (T8). */
        int y_above = (y == 0)        ? rows - 1 : y - 1;
        int y_below = (y == rows - 1) ? 0        : y + 1;

        for (int x = 0; x < cols; x++) {
            /* Toroidal horizontal neighbours. */
            int x_left  = (x == 0)        ? cols - 1 : x - 1;
            int x_right = (x == cols - 1) ? 0        : x + 1;

            int idx = y * cols + x;
            float u = grid->substrate_u[idx];
            float v = grid->catalyst_v [idx];

            /* 9-point isotropic Laplacian for U (T5). */
            float laplacian_u =
                LAPLACIAN_CARDINAL_WEIGHT *
                  ( grid->substrate_u[y       * cols + x_right]
                  + grid->substrate_u[y       * cols + x_left ]
                  + grid->substrate_u[y_below * cols + x      ]
                  + grid->substrate_u[y_above * cols + x      ] )
              + LAPLACIAN_DIAGONAL_WEIGHT *
                  ( grid->substrate_u[y_below * cols + x_right]
                  + grid->substrate_u[y_below * cols + x_left ]
                  + grid->substrate_u[y_above * cols + x_right]
                  + grid->substrate_u[y_above * cols + x_left ] )
              - u;

            /* Same stencil for V. */
            float laplacian_v =
                LAPLACIAN_CARDINAL_WEIGHT *
                  ( grid->catalyst_v[y       * cols + x_right]
                  + grid->catalyst_v[y       * cols + x_left ]
                  + grid->catalyst_v[y_below * cols + x      ]
                  + grid->catalyst_v[y_above * cols + x      ] )
              + LAPLACIAN_DIAGONAL_WEIGHT *
                  ( grid->catalyst_v[y_below * cols + x_right]
                  + grid->catalyst_v[y_below * cols + x_left ]
                  + grid->catalyst_v[y_above * cols + x_right]
                  + grid->catalyst_v[y_above * cols + x_left ] )
              - v;

            /* Shared autocatalytic reaction term. */
            float reaction_term = u * v * v;

            /* Forward Euler update (T6). */
            float u_new = u + EULER_DT * (DIFFUSION_U * laplacian_u
                                        - reaction_term
                                        + feed_rate * (1.0f - u));
            float v_new = v + EULER_DT * (DIFFUSION_V * laplacian_v
                                        + reaction_term
                                        - (feed_rate + kill_rate) * v);

            /* Clamp to physical range. */
            if (u_new < 0.0f) u_new = 0.0f; else if (u_new > 1.0f) u_new = 1.0f;
            if (v_new < 0.0f) v_new = 0.0f; else if (v_new > 1.0f) v_new = 1.0f;

            grid->substrate_u_next[idx] = u_new;
            grid->catalyst_v_next [idx] = v_new;
        }
    }

    /* Swap current ↔ scratch (T7).  O(1) pointer swap. */
    float *tmp;
    tmp = grid->substrate_u;
    grid->substrate_u      = grid->substrate_u_next;
    grid->substrate_u_next = tmp;
    tmp = grid->catalyst_v;
    grid->catalyst_v      = grid->catalyst_v_next;
    grid->catalyst_v_next = tmp;
}

/* ===================================================================== */
/* §11  seed — initial conditions + drop-blob helper (T10)               */
/* ===================================================================== */
/*
 * Reset to the trivial equilibrium U=1, V=0 everywhere, then place
 * the preset's number of square seed blobs (V=1, U=0) at random
 * positions.
 *
 * The seed positions use a simple "evenly spread + jitter" formula:
 * partition the grid horizontally into N equal columns, place each
 * blob at the centre of its column ± random jitter, with vertical
 * placement also randomised.
 */

static void place_seed_blob(Grid *grid, int centre_x, int centre_y)
{
    int cols = grid->cols;
    int rows = grid->rows;
    for (int dy = -SEED_BLOB_HALF_WIDTH; dy <= SEED_BLOB_HALF_WIDTH; dy++) {
        for (int dx = -SEED_BLOB_HALF_WIDTH; dx <= SEED_BLOB_HALF_WIDTH; dx++) {
            int x = ((centre_x + dx) % cols + cols) % cols;
            int y = ((centre_y + dy) % rows + rows) % rows;
            grid->substrate_u[y * cols + x] = 0.0f;
            grid->catalyst_v [y * cols + x] = 1.0f;
        }
    }
}

static void grid_seed_initial_conditions(Grid *grid)
{
    int cols = grid->cols;
    int rows = grid->rows;
    int cell_count = cols * rows;

    /* Reset to trivial equilibrium U=1, V=0. */
    for (int i = 0; i < cell_count; i++) {
        grid->substrate_u[i] = 1.0f;
        grid->catalyst_v [i] = 0.0f;
    }

    /* Distribute seed blobs evenly with jitter. */
    int blob_count = preset_table[grid->active_preset_index].seed_blob_count;
    if (blob_count < 1) blob_count = 1;

    int slot_width   = cols / blob_count;
    int x_jitter_max = (slot_width / 2);
    if (x_jitter_max < 1) x_jitter_max = 1;
    int y_centre     = rows / 2;
    int y_jitter_max = (rows / 4);
    if (y_jitter_max < 1) y_jitter_max = 1;

    for (int s = 0; s < blob_count; s++) {
        int x_base   = s * slot_width + slot_width / 2;
        int x_jitter = rand_in_range(-x_jitter_max, x_jitter_max + 1);
        int y_jitter = rand_in_range(-y_jitter_max, y_jitter_max + 1);
        int cx = x_base + x_jitter;
        int cy = y_centre + y_jitter;
        cx = ((cx % cols) + cols) % cols;
        cy = ((cy % rows) + rows) % rows;
        place_seed_blob(grid, cx, cy);
    }
}

/* ===================================================================== */
/* §12  warmup — fast-forward steps before first frame (T10)             */
/* ===================================================================== */
/*
 * After seeding, the screen is mostly U=1, V=0 with a few bright
 * dots.  Boring.  Run WARMUP_TICK_COUNT ticks before the first
 * render so the user immediately sees a developed pattern.
 */
static void grid_warmup(Grid *grid)
{
    for (int i = 0; i < WARMUP_TICK_COUNT; i++)
        reaction_step(grid);
}

/* Convenience: full reset (seed + warmup). */
static void grid_reseed(Grid *grid)
{
    grid_seed_initial_conditions(grid);
    grid_warmup(grid);
}

static void grid_init(Grid *grid, int cols, int rows,
                      int preset_index, int theme_index)
{
    grid_buffers_alloc(grid, cols, rows);
    grid->active_preset_index      = preset_index;
    grid->active_theme_index       = theme_index;
    grid->auto_theme_frame_counter = 0;
    grid_reseed(grid);
}

/* ===================================================================== */
/* §13  glyph_picker — V → ramp slot + glyph + attribute (T9)            */
/* ===================================================================== */

typedef struct {
    char   glyph;
    attr_t attr;
    bool   skip;
} CellRender;

static CellRender pick_cell_render(float v)
{
    /* Contrast stretch (T9). */
    float scaled = v * CATALYST_DISPLAY_SCALE;
    if (scaled > 1.0f) scaled = 1.0f;
    if (scaled < 0.0f) scaled = 0.0f;

    int slot = glyph_slot_for(scaled);

    /* Slot 0 = blank space; leave the underlying terminal cell as-is.
     * Saves attron/attroff thrash AND erase calls in the main loop. */
    if (slot == 0) return (CellRender){ .skip = true };

    return (CellRender){
        .glyph = ramp_glyph_table[slot],
        .attr  = attribute_for_slot(slot),
        .skip  = false,
    };
}

/* ===================================================================== */
/* §14  render_grid — paint the V field on the terminal                  */
/* ===================================================================== */
/*
 * Walks the grid, picks a glyph + colour for each cell from V, and
 * paints.  Skips slot-0 cells (transparent).  Auto-cycles the theme
 * every AUTO_THEME_CYCLE_FRAMES frames.
 *
 * Returns true if the theme was just rotated (caller should clear
 * the screen so the previous theme's cells don't linger).
 */
static bool render_grid(Grid *grid, int term_cols, int term_rows)
{
    bool theme_just_changed = false;
    grid->auto_theme_frame_counter++;
    if (grid->auto_theme_frame_counter >= AUTO_THEME_CYCLE_FRAMES) {
        grid->auto_theme_frame_counter = 0;
        grid->active_theme_index = (grid->active_theme_index + 1) % THEME_COUNT;
        apply_theme(grid->active_theme_index);
        theme_just_changed = true;
    }

    int cols = grid->cols;
    int rows = grid->rows;
    for (int y = 0; y < rows && y < term_rows; y++) {
        for (int x = 0; x < cols && x < term_cols; x++) {
            float v = grid->catalyst_v[y * cols + x];
            CellRender cr = pick_cell_render(v);
            if (cr.skip) continue;
            attron(cr.attr);
            mvaddch(y, x, (chtype)(unsigned char)cr.glyph);
            attroff(cr.attr);
        }
    }
    return theme_just_changed;
}

/* ===================================================================== */
/* §15  scene — per-frame state + tick + helpers                         */
/* ===================================================================== */

typedef struct {
    Grid grid;
    bool simulation_paused;
    bool needs_clear;          /* request erase() before next render */
    int  simulation_steps_per_frame;
} Scene;

static void scene_init(Scene *scene, int cols, int rows,
                       int preset_index, int theme_index)
{
    memset(scene, 0, sizeof *scene);
    scene->simulation_steps_per_frame = STEPS_PER_FRAME_DEFAULT;
    grid_init(&scene->grid, cols, rows, preset_index, theme_index);
}

static void scene_free(Scene *scene)
{
    grid_buffers_free(&scene->grid);
}

static void scene_set_preset(Scene *scene, int new_preset_index)
{
    new_preset_index = (new_preset_index % PRESET_COUNT + PRESET_COUNT)
                     % PRESET_COUNT;
    scene->grid.active_preset_index      = new_preset_index;
    scene->grid.auto_theme_frame_counter = 0;
    grid_reseed(&scene->grid);
    scene->needs_clear = true;
}

static void scene_cycle_theme(Scene *scene)
{
    scene->grid.active_theme_index =
        (scene->grid.active_theme_index + 1) % THEME_COUNT;
    scene->grid.auto_theme_frame_counter = 0;
    apply_theme(scene->grid.active_theme_index);
    scene->needs_clear = true;
}

static void scene_resize(Scene *scene, int cols, int rows)
{
    grid_buffers_resize(&scene->grid, cols, rows);
    grid_reseed(&scene->grid);
    scene->needs_clear = true;
}

static void scene_tick(Scene *scene)
{
    if (scene->simulation_paused) return;
    for (int i = 0; i < scene->simulation_steps_per_frame; i++)
        reaction_step(&scene->grid);
}

/* ===================================================================== */
/* §16  hud — top status + bottom hint (CLAUDE.md spec)                  */
/* ===================================================================== */
/*
 * Two HUD elements per CLAUDE.md:
 *   row 0       — status (yellow + bold), top-right
 *   row rows-1  — key hint (cyan + bold),  bottom
 */

static void hud_paint_status(int term_cols, const Scene *scene, double fps)
{
    const Grid *grid = &scene->grid;
    char buf[200];
    snprintf(buf, sizeof buf,
             " Reaction-Diffusion  %s  theme:%-6s  steps/frame:%2d  "
             "%5.1f fps  %s ",
             preset_table[grid->active_preset_index].display_name,
             theme_table[grid->active_theme_index].display_name,
             scene->simulation_steps_per_frame, fps,
             scene->simulation_paused ? "PAUSED " : "running");
    int slen = (int)strlen(buf);
    int sx   = term_cols - slen;
    if (sx < 0) sx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
             " q:quit  spc:pause  r:reseed  s:seed  n/p:preset  "
             "t:theme  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §17  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *screen, int theme_index)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    colors_init(theme_index);
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

static void screen_present_frame(Screen *screen, Scene *scene, double fps)
{
    if (scene->needs_clear) {
        erase();
        scene->needs_clear = false;
    }

    bool theme_changed = render_grid(&scene->grid, screen->cols, screen->rows);
    if (theme_changed) scene->needs_clear = true;

    hud_paint_status(screen->cols, scene, fps);
    hud_paint_hint  (screen->rows);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §18  app — main loop + signals + input                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App app_state;

static void on_signal_quit  (int sig) { (void)sig; app_state.running     = 0; }
static void on_signal_resize(int sig) { (void)sig; app_state.need_resize = 1; }

static bool app_handle_key(App *app, int ch)
{
    Scene *scene = &app->scene;
    Grid  *grid  = &scene->grid;

    switch (ch) {
        case 'q': case 'Q': case 27:
            return false;

        case ' ':
            scene->simulation_paused = !scene->simulation_paused;
            break;

        case 'n': case 'N':
            scene_set_preset(scene, grid->active_preset_index + 1);
            break;
        case 'p': case 'P':
            scene_set_preset(scene, grid->active_preset_index + PRESET_COUNT - 1);
            break;

        case 't': case 'T':
            scene_cycle_theme(scene);
            break;

        case 'r': case 'R':
            grid_reseed(grid);
            scene->needs_clear = true;
            break;

        case 's': case 'S':
            place_seed_blob(grid, grid->cols / 2, grid->rows / 2);
            break;

        case '+': case '=':
            scene->simulation_steps_per_frame += STEPS_PER_FRAME_STEP;
            if (scene->simulation_steps_per_frame > STEPS_PER_FRAME_MAX)
                scene->simulation_steps_per_frame = STEPS_PER_FRAME_MAX;
            break;
        case '-':
            scene->simulation_steps_per_frame -= STEPS_PER_FRAME_STEP;
            if (scene->simulation_steps_per_frame < STEPS_PER_FRAME_MIN)
                scene->simulation_steps_per_frame = STEPS_PER_FRAME_MIN;
            break;

        default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(screen_cleanup);
    signal(SIGINT,   on_signal_quit);
    signal(SIGTERM,  on_signal_quit);
    signal(SIGWINCH, on_signal_resize);

    App *app     = &app_state;
    app->running = 1;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

    int64_t prev_frame_ns       = clock_now_ns();
    int64_t fps_window_ns       = 0;
    int     frames_in_window    = 0;
    double  measured_fps        = 0.0;

    while (app->running) {
        int64_t frame_start_ns = clock_now_ns();

        /* ── input ── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) {
            app->running = 0;
            break;
        }

        /* ── resize ── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
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
        scene_tick(&app->scene);
        screen_present_frame(&app->screen, &app->scene, measured_fps);

        /* ── frame cap ── */
        int64_t spent = clock_now_ns() - frame_start_ns;
        if (spent < RENDER_TICK_NS) clock_sleep_ns(RENDER_TICK_NS - spent);
    }

    scene_free(&app->scene);
    return 0;
}
